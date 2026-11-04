//! Phone trust: which bonded phones are validated in the car (see docs/ble.md, "Phone trust").
//!
//! Three levels: a phone is bonded (it paired during a pairing window), the device is
//! approved for CAN read-write mode on this car (`can_access.rs`), and a phone is
//! validated, by an in-car action that proves it is the owner's (`validation.rs`). A
//! vehicle command needs a validated phone on top of the read-write policy.
//!
//! Validations belong to one car ([`CarDigest`]), like the approval, and only apply while
//! that car is identified. They are persisted by the board ([`phone_validations_to_store`])
//! and end when the phone's bond is removed (`pairing.rs`), when the phone pairs again,
//! when the bonds are reset, or when another car is approved.

use alloc::vec::Vec;
use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use crate::can_access::CarDigest;
use crate::proto;

/// The validated phones and the car they were validated in.
struct Validations {
    car: CarDigest,
    phones: Vec<Vec<u8>>,
}

static VALIDATIONS: Mutex<CriticalSectionRawMutex, RefCell<Option<Validations>>> =
    Mutex::new(RefCell::new(None));

/// The validations the board has to persist, encoded ([`encode_phone_validations`]); empty
/// to erase them. Only the latest value matters.
static VALIDATIONS_TO_STORE: Signal<CriticalSectionRawMutex, Vec<u8>> = Signal::new();

/// The phone connected over BLE, as stamped in `source_device_id`. The ESP has one BLE
/// connection at a time; what the device publishes over BLE is for this phone.
static BLE_PEER: Mutex<CriticalSectionRawMutex, RefCell<Option<Vec<u8>>>> =
    Mutex::new(RefCell::new(None));

/// Format version of [`encode_phone_validations`]; an unknown one reads as no validation.
const VALIDATIONS_FORMAT_VERSION: u8 = 1;

/// Raised whenever the trust of a phone or its session may have changed, so the access
/// state and the command availability are published again.
pub(crate) fn notify_phone_access_changed() {
    crate::can_access::CAN_ACCESS_CHANGED.signal(());
    crate::availability::notify_availability_changed();
}

fn store_validations() {
    VALIDATIONS_TO_STORE.signal(encode_phone_validations());
    notify_phone_access_changed();
}

// ── Queries ───────────────────────────────────────────────────────────────────

/// `true` while `device_id` is bonded and validated for the car identified now.
pub fn is_phone_validated(device_id: &[u8]) -> bool {
    let Some(car) = crate::can_access::identified_car() else {
        return false;
    };
    !device_id.is_empty()
        && crate::pairing::is_paired(device_id)
        && VALIDATIONS.lock(|v| {
            v.borrow().as_ref().is_some_and(|v| {
                v.car == car && v.phones.iter().any(|id| id.as_slice() == device_id)
            })
        })
}

/// `true` while a bonded phone is validated for the car identified now. Then only a
/// validated phone can change the approval or remove a validated phone.
pub fn car_has_validated_phone() -> bool {
    let Some(car) = crate::can_access::identified_car() else {
        return false;
    };
    VALIDATIONS.lock(|v| {
        v.borrow().as_ref().is_some_and(|v| {
            v.car == car && v.phones.iter().any(|id| crate::pairing::is_paired(id))
        })
    })
}

/// How far the device trusts `device_id`, for the car identified now.
pub fn phone_trust_level(device_id: &[u8]) -> proto::PhoneTrustLevel {
    if !crate::pairing::is_paired(device_id) {
        proto::PhoneTrustLevel::NotBonded
    } else if is_phone_validated(device_id) {
        proto::PhoneTrustLevel::Validated
    } else {
        proto::PhoneTrustLevel::Bonded
    }
}

// ── Changes ───────────────────────────────────────────────────────────────────

/// Records `device_id` as validated in `car`. Validations of another car are dropped:
/// they belong to a car the device is no longer in.
pub(crate) fn add_phone_validation(car: CarDigest, device_id: &[u8]) {
    VALIDATIONS.lock(|v| {
        let mut v = v.borrow_mut();
        match v.as_mut() {
            Some(v) if v.car == car => {
                if !v.phones.iter().any(|id| id.as_slice() == device_id) {
                    v.phones.push(device_id.to_vec());
                }
            }
            _ => {
                *v = Some(Validations {
                    car,
                    phones: alloc::vec![device_id.to_vec()],
                })
            }
        }
    });
    store_validations();
}

/// Ends `device_id`'s validation. Called when its bond is removed, and by the board when
/// the phone pairs again: new keys can belong to another phone claiming its address.
pub fn forget_phone_validation(device_id: &[u8]) {
    let changed = VALIDATIONS.lock(|v| {
        let mut v = v.borrow_mut();
        let Some(validations) = v.as_mut() else {
            return false;
        };
        let before = validations.phones.len();
        validations.phones.retain(|id| id.as_slice() != device_id);
        let changed = validations.phones.len() != before;
        if validations.phones.is_empty() {
            *v = None;
        }
        changed
    });
    if changed {
        store_validations();
    }
}

/// Ends every validation (all bonds cleared, or another car approved).
pub(crate) fn clear_phone_validations() {
    let had_any = VALIDATIONS.lock(|v| v.borrow_mut().take().is_some());
    if had_any {
        store_validations();
    }
}

/// Called when the owner approves `car`: validations made in another car end, so every
/// phone validates again in this one.
pub(crate) fn on_car_approved(car: CarDigest) {
    let other_car = VALIDATIONS.lock(|v| v.borrow().as_ref().is_some_and(|v| v.car != car));
    if other_car {
        clear_phone_validations();
    }
}

/// Drops the validations of phones that are no longer bonded. The board calls it once
/// at boot, after restoring the validations and the bonds, so a bond reset (or bonds
/// lost from flash) also ends their validations.
pub async fn prune_phone_validations() {
    let changed = VALIDATIONS.lock(|v| {
        let mut v = v.borrow_mut();
        let Some(validations) = v.as_mut() else {
            return false;
        };
        let before = validations.phones.len();
        validations
            .phones
            .retain(|id| crate::pairing::is_paired(id));
        let changed = validations.phones.len() != before;
        if validations.phones.is_empty() {
            *v = None;
        }
        changed
    });
    if changed {
        store_validations();
    }
}

// ── Persistence ───────────────────────────────────────────────────────────────

/// The validations in the format the board stores: version, car digest, count, then each
/// device ID with its length. Empty when there is none.
pub fn encode_phone_validations() -> Vec<u8> {
    VALIDATIONS.lock(|v| {
        let v = v.borrow();
        let Some(v) = v.as_ref() else {
            return Vec::new();
        };
        let mut out = Vec::with_capacity(34 + v.phones.len() * 11);
        out.push(VALIDATIONS_FORMAT_VERSION);
        out.extend_from_slice(&v.car);
        out.push(v.phones.len() as u8);
        for id in &v.phones {
            out.push(id.len() as u8);
            out.extend_from_slice(id);
        }
        out
    })
}

fn decode_phone_validations(bytes: &[u8]) -> Option<Validations> {
    let (&version, rest) = bytes.split_first()?;
    if version != VALIDATIONS_FORMAT_VERSION || rest.len() < 33 {
        return None;
    }
    let car: CarDigest = rest[..32].try_into().ok()?;
    let count = rest[32] as usize;
    let mut rest = &rest[33..];
    let mut phones = Vec::with_capacity(count);
    for _ in 0..count {
        let (&len, tail) = rest.split_first()?;
        let len = len as usize;
        if len == 0 || tail.len() < len {
            return None;
        }
        phones.push(tail[..len].to_vec());
        rest = &tail[len..];
    }
    if !rest.is_empty() || phones.is_empty() {
        return None;
    }
    Some(Validations { car, phones })
}

/// Loads the validations the board persisted ([`encode_phone_validations`]). Called once
/// at boot, before the transports start; follow it with [`prune_phone_validations`] once
/// the bonds are restored. Anything malformed reads as no validation.
/// Returns whether any validation was restored.
pub fn restore_phone_validations(bytes: &[u8]) -> bool {
    let validations = decode_phone_validations(bytes);
    let restored = validations.is_some();
    VALIDATIONS.lock(|v| *v.borrow_mut() = validations);
    notify_phone_access_changed();
    restored
}

/// Waits for the next change the board has to persist: the encoded validations, empty to
/// erase them.
pub async fn phone_validations_to_store() -> Vec<u8> {
    VALIDATIONS_TO_STORE.wait().await
}

/// Non-blocking [`phone_validations_to_store`]: `None` if nothing is pending.
pub fn try_take_phone_validations_to_store() -> Option<Vec<u8>> {
    VALIDATIONS_TO_STORE.try_take()
}

// ── The connected phone ───────────────────────────────────────────────────────

/// Reported by the board's BLE transport: the connected phone's `source_device_id`, as it
/// stamps it, or `None` when no phone is connected. Also cleared when the BLE link goes
/// down.
pub fn set_ble_peer(device_id: Option<&[u8]>) {
    let peer = device_id.filter(|id| !id.is_empty()).map(<[u8]>::to_vec);
    let changed = BLE_PEER.lock(|p| {
        let mut p = p.borrow_mut();
        let changed = *p != peer;
        *p = peer;
        changed
    });
    if changed {
        notify_phone_access_changed();
    }
}

/// The connected BLE phone, if any.
pub(crate) fn ble_peer() -> Option<Vec<u8>> {
    BLE_PEER.lock(|p| p.borrow().clone())
}

/// `true` while the connected BLE phone is validated for this car.
pub(crate) fn is_ble_peer_validated() -> bool {
    ble_peer().is_some_and(|id| is_phone_validated(&id))
}

/// What the device allows the connected BLE phone, `None` while no phone is connected.
pub fn phone_access_state(now_ms: u64) -> Option<proto::PhoneAccessState> {
    let peer = ble_peer()?;
    Some(proto::PhoneAccessState {
        trust: Some(phone_trust_level(&peer) as i32),
        car_has_validated_phone: Some(car_has_validated_phone()),
        validation: crate::validation::session_for(&peer, now_ms),
    })
}

/// Validates `device_id` for the car identified now, without a session. For tests only,
/// behind the `test-utils` feature. Returns `false` while no car is identified.
#[cfg(any(test, feature = "test-utils"))]
pub fn validate_phone_for_tests(device_id: &[u8]) -> bool {
    match crate::can_access::identified_car() {
        Some(car) => {
            add_phone_validation(car, device_id);
            true
        }
        None => false,
    }
}

/// Clears the validations, the connected phone and any pending store. For tests only.
pub fn reset_phone_trust_for_tests() {
    VALIDATIONS.lock(|v| *v.borrow_mut() = None);
    BLE_PEER.lock(|p| *p.borrow_mut() = None);
    VALIDATIONS_TO_STORE.reset();
}
