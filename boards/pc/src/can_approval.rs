//! File-backed store for the owner's CAN read-write approval (see docs/can.md).
//!
//! The file holds the approved car's digest as 64 hex characters; no file means not
//! approved. Anything else reads as not approved too, so the bus stays read-only.

use std::sync::OnceLock;

static CAN_RW_APPROVAL_PATH: OnceLock<&'static str> = OnceLock::new();

pub(crate) fn parse_approval(content: &str) -> Option<[u8; 32]> {
    let hex = content.trim();
    if hex.len() != 64 {
        return None;
    }
    let mut digest = [0u8; 32];
    for (i, byte) in digest.iter_mut().enumerate() {
        *byte = u8::from_str_radix(hex.get(i * 2..i * 2 + 2)?, 16).ok()?;
    }
    Some(digest)
}

pub(crate) fn format_approval(digest: &[u8; 32]) -> String {
    let mut out = String::with_capacity(65);
    for b in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut out, "{:02x}", b);
    }
    out.push('\n');
    out
}

/// Loads the approval from `path` into `core-interface`. Called once from `main`,
/// before any task starts, so no command can race the restore.
pub fn restore_can_rw_approval(path: &'static str) {
    let _ = CAN_RW_APPROVAL_PATH.set(path);
    let approval = match std::fs::read_to_string(path) {
        Ok(content) => {
            let approval = parse_approval(&content);
            if approval.is_none() {
                log::warn!(
                    "CAN approval store: '{}' is not a valid approval, ignoring it",
                    path
                );
            }
            approval
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => None,
        Err(e) => {
            log::warn!("CAN approval store: failed to read '{}': {:?}", path, e);
            None
        }
    };
    log::info!(
        "CAN approval store: read-write mode {}",
        if approval.is_some() {
            "approved for a car"
        } else {
            "not approved"
        }
    );
    core_interface::restore_can_read_write_approval(approval);
}

/// Writes each approval change to the file set by [`restore_can_rw_approval`].
#[embassy_executor::task]
pub async fn can_rw_approval_store_task() {
    let Some(path) = CAN_RW_APPROVAL_PATH.get().copied() else {
        log::error!("CAN approval store: no file set, approvals won't survive a restart");
        return;
    };
    loop {
        let result = match core_interface::can_read_write_approval_to_store().await {
            Some(digest) => std::fs::write(path, format_approval(&digest)),
            None => match std::fs::remove_file(path) {
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(()),
                other => other,
            },
        };
        if let Err(e) = result {
            log::error!("CAN approval store: failed to update '{}': {:?}", path, e);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn approval_round_trips_through_the_file_format() {
        let digest: [u8; 32] = core::array::from_fn(|i| (i * 7) as u8);
        assert_eq!(parse_approval(&format_approval(&digest)), Some(digest));
    }

    #[test]
    fn malformed_files_read_as_not_approved() {
        assert_eq!(parse_approval(""), None);
        assert_eq!(parse_approval("abc"), None);
        assert_eq!(parse_approval(&"zz".repeat(32)), None);
        assert_eq!(parse_approval(&"00".repeat(33)), None);
    }
}
