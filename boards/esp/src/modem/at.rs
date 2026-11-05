//! AT command helpers: line classification, the replies the bring-up reads, and the
//! commands it sends. Pure logic, host-tested; the I/O lives in `channel.rs`.

use core::fmt::Write as _;
use heapless::String;

/// Longest AT command the driver builds (the PDP context with a long APN).
pub const MAX_COMMAND_LEN: usize = 160;

/// Longest response line kept; longer lines are truncated (the bring-up never needs more).
pub const MAX_LINE_LEN: usize = 128;

pub type Command = String<MAX_COMMAND_LEN>;

/// How a line the modem sent ends (or doesn't end) the command in progress.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Final {
    /// `OK`
    Ok,
    /// `CONNECT` (with or without a rate): the modem switched to data mode.
    Connect,
    /// `ERROR`, `+CME ERROR: …`, `+CMS ERROR: …`
    Error,
    /// `NO CARRIER`, `BUSY`, `NO ANSWER`, `NO DIALTONE`: a dial that failed.
    NoCarrier,
}

/// Classifies a response line: `Some` if it ends the command, `None` for an
/// information line (`+CEREG: …`) or an unsolicited one.
pub fn final_result(line: &str) -> Option<Final> {
    match line {
        "OK" => Some(Final::Ok),
        "ERROR" => Some(Final::Error),
        "NO CARRIER" | "BUSY" | "NO ANSWER" | "NO DIALTONE" => Some(Final::NoCarrier),
        _ if line == "CONNECT" || line.starts_with("CONNECT ") => Some(Final::Connect),
        _ if line.starts_with("+CME ERROR:") || line.starts_with("+CMS ERROR:") => {
            Some(Final::Error)
        }
        _ => None,
    }
}

/// The value part of an information line (`+CEREG: 0,1` with prefix `+CEREG` → `0,1`).
pub fn info_value<'a>(line: &'a str, prefix: &str) -> Option<&'a str> {
    line.strip_prefix(prefix)?
        .strip_prefix(':')
        .map(str::trim_start)
}

/// SIM state from `AT+CPIN?`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimState {
    Ready,
    /// The SIM wants its PIN.
    PinRequired,
    /// PUK or any other code: the driver can't unlock it.
    Locked,
}

pub fn parse_cpin(line: &str) -> Option<SimState> {
    let v = info_value(line, "+CPIN")?;
    Some(match v {
        "READY" => SimState::Ready,
        "SIM PIN" => SimState::PinRequired,
        _ => SimState::Locked,
    })
}

/// Network registration from `AT+CEREG?` (LTE) or `AT+CREG?`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Registration {
    NotSearching,
    Searching,
    Denied,
    Unknown,
    Home,
    Roaming,
}

impl Registration {
    pub fn is_registered(self) -> bool {
        matches!(self, Registration::Home | Registration::Roaming)
    }
}

/// Parses the reply to the read command (`+CEREG: <n>,<stat>[,…]`). The unsolicited
/// form (`+CEREG: <stat>[,…]`) is not read: the driver keeps them off (`<n>` = 0).
pub fn parse_registration(line: &str, prefix: &str) -> Option<Registration> {
    let mut fields = info_value(line, prefix)?.split(',');
    let _mode = fields.next()?;
    let stat: u8 = fields.next()?.trim().parse().ok()?;
    Some(match stat {
        0 => Registration::NotSearching,
        1 => Registration::Home,
        2 => Registration::Searching,
        3 => Registration::Denied,
        5 => Registration::Roaming,
        // 6/7 (SMS only) aren't a data registration; 4 and the rest are unknown.
        _ => Registration::Unknown,
    })
}

/// Signal quality from `AT+CSQ`: the RSSI in dBm, `None` when the modem doesn't know.
pub fn parse_csq_dbm(line: &str) -> Option<Option<i16>> {
    let rssi: u8 = info_value(line, "+CSQ")?
        .split(',')
        .next()?
        .trim()
        .parse()
        .ok()?;
    Some(match rssi {
        0..=31 => Some(-113 + 2 * i16::from(rssi)),
        _ => None,
    })
}

/// Why a configured value can't go into an AT command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandError {
    /// It holds a `"`, a control character or non-ASCII text, which can't be quoted.
    Unquotable,
    TooLong,
}

fn push_quoted(cmd: &mut Command, value: &str) -> Result<(), CommandError> {
    if !value
        .bytes()
        .all(|b| (0x20..0x7f).contains(&b) && b != b'"')
    {
        return Err(CommandError::Unquotable);
    }
    write!(cmd, "\"{value}\"").map_err(|_| CommandError::TooLong)
}

/// `AT+CGDCONT=<cid>,"IP","<apn>"`: the PDP context the dial (`ATD*99#`) uses.
pub fn pdp_context(cid: u8, apn: &str) -> Result<Command, CommandError> {
    let mut cmd = Command::new();
    write!(cmd, "AT+CGDCONT={cid},\"IP\",").map_err(|_| CommandError::TooLong)?;
    push_quoted(&mut cmd, apn)?;
    Ok(cmd)
}

/// `AT+CPIN="<pin>"`. The PIN must be 4 to 8 digits.
pub fn enter_pin(pin: &str) -> Result<Command, CommandError> {
    if !(4..=8).contains(&pin.len()) || !pin.bytes().all(|b| b.is_ascii_digit()) {
        return Err(CommandError::Unquotable);
    }
    let mut cmd = Command::new();
    cmd.push_str("AT+CPIN=")
        .map_err(|_| CommandError::TooLong)?;
    push_quoted(&mut cmd, pin)?;
    Ok(cmd)
}

/// Dials the packet data context: the modem answers `CONNECT` and switches to PPP.
pub const DIAL_PPP: &str = "ATD*99#";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn final_results() {
        assert_eq!(final_result("OK"), Some(Final::Ok));
        assert_eq!(final_result("ERROR"), Some(Final::Error));
        assert_eq!(final_result("+CME ERROR: 10"), Some(Final::Error));
        assert_eq!(
            final_result("+CME ERROR: SIM not inserted"),
            Some(Final::Error)
        );
        assert_eq!(final_result("CONNECT"), Some(Final::Connect));
        assert_eq!(final_result("CONNECT 115200"), Some(Final::Connect));
        assert_eq!(final_result("NO CARRIER"), Some(Final::NoCarrier));
        assert_eq!(final_result("BUSY"), Some(Final::NoCarrier));
        assert_eq!(final_result("+CEREG: 0,1"), None);
        assert_eq!(final_result("RDY"), None);
        assert_eq!(final_result("CONNECTED"), None);
        assert_eq!(final_result("OKAY"), None);
    }

    #[test]
    fn info_values() {
        assert_eq!(info_value("+CSQ: 20,99", "+CSQ"), Some("20,99"));
        assert_eq!(info_value("+CSQ:20,99", "+CSQ"), Some("20,99"));
        assert_eq!(info_value("+CSQX: 1", "+CSQ"), None);
        assert_eq!(info_value("OK", "+CSQ"), None);
    }

    #[test]
    fn sim_states() {
        assert_eq!(parse_cpin("+CPIN: READY"), Some(SimState::Ready));
        assert_eq!(parse_cpin("+CPIN: SIM PIN"), Some(SimState::PinRequired));
        assert_eq!(parse_cpin("+CPIN: SIM PUK"), Some(SimState::Locked));
        assert_eq!(parse_cpin("+CEREG: 0,1"), None);
    }

    #[test]
    fn registration() {
        let p = |l| parse_registration(l, "+CEREG");
        assert_eq!(p("+CEREG: 0,1"), Some(Registration::Home));
        assert_eq!(p("+CEREG: 0,5"), Some(Registration::Roaming));
        assert_eq!(
            p("+CEREG: 2,1,\"1A2B\",\"01A2B3C4\",7"),
            Some(Registration::Home)
        );
        assert_eq!(p("+CEREG: 0,2"), Some(Registration::Searching));
        assert_eq!(p("+CEREG: 0,3"), Some(Registration::Denied));
        assert_eq!(p("+CEREG: 0,0"), Some(Registration::NotSearching));
        assert_eq!(p("+CEREG: 0,6"), Some(Registration::Unknown));
        assert_eq!(p("+CEREG: 0"), None);
        assert_eq!(p("+CREG: 0,1"), None);
        assert!(Registration::Roaming.is_registered());
        assert!(!Registration::Searching.is_registered());
    }

    #[test]
    fn signal_quality() {
        assert_eq!(parse_csq_dbm("+CSQ: 0,99"), Some(Some(-113)));
        assert_eq!(parse_csq_dbm("+CSQ: 20,99"), Some(Some(-73)));
        assert_eq!(parse_csq_dbm("+CSQ: 31,0"), Some(Some(-51)));
        assert_eq!(parse_csq_dbm("+CSQ: 99,99"), Some(None));
        assert_eq!(parse_csq_dbm("+CSQ: x,99"), None);
    }

    #[test]
    fn pdp_context_command() {
        assert_eq!(
            pdp_context(1, "internet.example").unwrap().as_str(),
            "AT+CGDCONT=1,\"IP\",\"internet.example\""
        );
        assert_eq!(
            pdp_context(1, "").unwrap().as_str(),
            "AT+CGDCONT=1,\"IP\",\"\""
        );
        assert_eq!(pdp_context(1, "bad\"apn"), Err(CommandError::Unquotable));
        assert_eq!(pdp_context(1, "bad\rapn"), Err(CommandError::Unquotable));
        assert_eq!(pdp_context(1, "é"), Err(CommandError::Unquotable));
        let long = "a".repeat(MAX_COMMAND_LEN);
        assert_eq!(pdp_context(1, &long), Err(CommandError::TooLong));
    }

    #[test]
    fn pin_command() {
        assert_eq!(enter_pin("1234").unwrap().as_str(), "AT+CPIN=\"1234\"");
        assert_eq!(
            enter_pin("12345678").unwrap().as_str(),
            "AT+CPIN=\"12345678\""
        );
        assert!(enter_pin("123").is_err());
        assert!(enter_pin("123456789").is_err());
        assert!(enter_pin("12a4").is_err());
    }
}
