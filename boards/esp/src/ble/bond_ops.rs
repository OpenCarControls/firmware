//! Bond management shared between the system command handler and the BLE task.
//!
//! The stack's bonds (LTKs in the `BondStore`, the controller lists) are what
//! authorizes a phone, so removing a phone has to go through the BLE task, which
//! owns them. `system_command_task` queues a [`BondOp`] here once the pairing
//! window check has passed; the BLE task applies it and keeps the paired-phone
//! registry in `core-interface` mirroring the bonds.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

const DEVICE_ID_PREFIX: &[u8] = b"ble:";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(crate) enum BondOp {
    Remove([u8; 6]),
    Clear,
}

#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(crate) static BOND_OPS: Channel<CriticalSectionRawMutex, BondOp, 4> = Channel::new();

/// The `device_id` of a bonded phone: the same `ble:` + address stamp the
/// transport puts on every inbound message.
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(in crate::ble) fn phone_device_id(addr: &[u8; 6]) -> [u8; 10] {
    let mut id = [0u8; 10];
    id[..4].copy_from_slice(DEVICE_ID_PREFIX);
    id[4..].copy_from_slice(addr);
    id
}

/// Inverse of [`phone_device_id`]; `None` for ids that aren't a BLE address.
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(crate) fn parse_phone_device_id(id: &[u8]) -> Option<[u8; 6]> {
    id.strip_prefix(DEVICE_ID_PREFIX)?.try_into().ok()
}

/// Whether a new phone may bond given the bonds already stored.
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(in crate::ble) fn has_room_for_bond(bond_count: usize, max_bonded_phones: u8) -> bool {
    bond_count < max_bonded_phones as usize
}

/// Whether a disconnect means the stored keys no longer work: a security error, or
/// a hang-up on a link that never got encrypted. The latter is a phone that forgot
/// its side of the bond and gave up (Android reports it as a normal hang-up). A
/// phone that hangs up after an encrypted session keeps its bond.
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(in crate::ble) fn is_broken_bond_disconnect(
    reason: bt_hci::param::Status,
    link_encrypted: bool,
) -> bool {
    use bt_hci::param::Status;
    match reason {
        Status::PIN_OR_KEY_MISSING
        | Status::AUTHENTICATION_FAILURE
        | Status::CONN_TERMINATED_DUE_TO_MIC_FAILURE => true,
        Status::REMOTE_USER_TERMINATED_CONN => !link_encrypted,
        _ => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bt_hci::param::Status;

    #[test]
    fn device_id_round_trips() {
        let addr = [1, 2, 3, 4, 5, 0xC6];
        let id = phone_device_id(&addr);
        assert_eq!(&id[..4], b"ble:");
        assert_eq!(parse_phone_device_id(&id), Some(addr));
    }

    #[test]
    fn foreign_device_ids_are_rejected() {
        assert_eq!(parse_phone_device_id(b"ble:12345"), None);
        assert_eq!(parse_phone_device_id(b"ble:1234567"), None);
        assert_eq!(parse_phone_device_id(b"mqtt:123456"), None);
        assert_eq!(parse_phone_device_id(&[]), None);
    }

    #[test]
    fn bond_limit_is_enforced() {
        assert!(has_room_for_bond(0, 1));
        assert!(!has_room_for_bond(1, 1));
        assert!(has_room_for_bond(7, 8));
        assert!(!has_room_for_bond(8, 8));
    }

    #[test]
    fn normal_disconnect_is_not_a_broken_bond() {
        assert!(!is_broken_bond_disconnect(
            Status::REMOTE_USER_TERMINATED_CONN,
            true
        ));
        assert!(!is_broken_bond_disconnect(Status::CONN_TIMEOUT, true));
        assert!(!is_broken_bond_disconnect(Status::CONN_TIMEOUT, false));
    }

    #[test]
    fn security_errors_are_broken_bonds() {
        for encrypted in [true, false] {
            assert!(is_broken_bond_disconnect(
                Status::PIN_OR_KEY_MISSING,
                encrypted
            ));
            assert!(is_broken_bond_disconnect(
                Status::AUTHENTICATION_FAILURE,
                encrypted
            ));
            assert!(is_broken_bond_disconnect(
                Status::CONN_TERMINATED_DUE_TO_MIC_FAILURE,
                encrypted
            ));
        }
    }

    #[test]
    fn hang_up_before_encryption_is_a_broken_bond() {
        // A phone that forgot its bond: Android hangs up before the link encrypts.
        assert!(is_broken_bond_disconnect(
            Status::REMOTE_USER_TERMINATED_CONN,
            false
        ));
    }
}
