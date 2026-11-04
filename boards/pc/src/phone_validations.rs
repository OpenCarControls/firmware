//! File-backed store for the phones validated in the car (see docs/ble.md, "Phone trust").
//!
//! The file holds `core_interface::encode_phone_validations()` as hex; no file means no
//! validated phone. Anything else reads as none too, so phones validate again.

use std::sync::OnceLock;

static PHONE_VALIDATIONS_PATH: OnceLock<&'static str> = OnceLock::new();

pub(crate) fn parse_validations(content: &str) -> Option<Vec<u8>> {
    let hex = content.trim();
    if hex.is_empty() || !hex.len().is_multiple_of(2) {
        return None;
    }
    (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(hex.get(i..i + 2)?, 16).ok())
        .collect()
}

pub(crate) fn format_validations(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2 + 1);
    for b in bytes {
        use std::fmt::Write as _;
        let _ = write!(&mut out, "{:02x}", b);
    }
    out.push('\n');
    out
}

/// Loads the validations from `path` into `core-interface`. Called once from `main`,
/// before any task starts; the system command task prunes them to the bonded phones once
/// it has restored those.
pub fn restore_phone_validations(path: &'static str) {
    let _ = PHONE_VALIDATIONS_PATH.set(path);
    let bytes = match std::fs::read_to_string(path) {
        Ok(content) => parse_validations(&content),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
        Err(e) => {
            log::warn!("Phone validations: failed to read '{}': {:?}", path, e);
            None
        }
    };
    let restored = bytes.is_some_and(|b| core_interface::restore_phone_validations(&b));
    log::info!(
        "Phone validations: {}",
        if restored {
            "restored from file"
        } else {
            "no validated phone"
        }
    );
}

/// Writes each change of the validations to the file set by [`restore_phone_validations`].
#[embassy_executor::task]
pub async fn phone_validations_store_task() {
    let Some(path) = PHONE_VALIDATIONS_PATH.get().copied() else {
        log::error!("Phone validations: no file set, validations won't survive a restart");
        return;
    };
    loop {
        let bytes = core_interface::phone_validations_to_store().await;
        let result = if bytes.is_empty() {
            match std::fs::remove_file(path) {
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
                other => other,
            }
        } else {
            std::fs::write(path, format_validations(&bytes))
        };
        if let Err(e) = result {
            log::error!("Phone validations: failed to update '{}': {:?}", path, e);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validations_round_trip_through_the_file_format() {
        let bytes: Vec<u8> = (0..45).map(|i| (i * 7) as u8).collect();
        assert_eq!(parse_validations(&format_validations(&bytes)), Some(bytes));
    }

    #[test]
    fn malformed_files_read_as_none() {
        assert_eq!(parse_validations(""), None);
        assert_eq!(parse_validations("abc"), None);
        assert_eq!(parse_validations("zz"), None);
    }
}
