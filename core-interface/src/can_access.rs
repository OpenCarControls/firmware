//! CAN read-write policy: when the device may leave read-only mode on the car's buses.
//!
//! Read-write mode needs all of these, on top of the vehicle's own unlock
//! (`set_can_read_only(false)`) and no CAN logging session:
//! 1. The vehicle recognised its platform on the bus ([`crate::confirm_car_identity`]) and
//!    identified the exact car ([`confirm_car_fingerprint`]).
//! 2. The owner approved read-write mode for that car ([`approve_can_read_write`]). The
//!    approval is bound to the car's digest and persisted by the board, so it survives
//!    reboots and phone changes but not a move to another car.
//! 3. In development builds, the dev lock is unlocked ([`set_can_dev_write_unlocked`]).
//!    It is locked at every boot. Release builds (`cargo xtask build --production`)
//!    don't have it.
//!
//! [`crate::is_can_read_only`] derives the answer from this state every time it is read,
//! so an unmet condition always reads as read-only, whatever was asked before.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use portable_atomic::{AtomicBool, Ordering};
use sha2::{Digest, Sha256};

use crate::channels::BLE_TX_CHANNEL;
use crate::proto;

/// Identifies one car to the policy: SHA-256 of the platform ID and the vehicle's car
/// fingerprint. What is stored for an approval, so no VIN ends up in flash.
pub type CarDigest = [u8; 32];

/// Domain separation for [`CarDigest`]; bump it to invalidate every stored approval.
const CAR_DIGEST_DOMAIN: &[u8] = b"opencar/can-rw-approval/v1";

struct Policy {
    /// The identified car, `None` until the vehicle confirms its fingerprint.
    car: Option<CarDigest>,
    /// The car the owner approved read-write mode for, `None` if never approved or revoked.
    approval: Option<CarDigest>,
    /// The dev lock (development builds only). Not stored: locked at every boot.
    dev_unlocked: bool,
}

/// One lock for the whole policy, so the car, the approval and the dev lock are always
/// read together.
static POLICY: Mutex<CriticalSectionRawMutex, RefCell<Policy>> = Mutex::new(RefCell::new(Policy {
    car: None,
    approval: None,
    dev_unlocked: false,
}));

/// Set once at boot by the generated `main.rs` of a `--production` build. The default
/// (`false`) keeps the dev lock, which only adds a condition.
static PRODUCTION_BUILD: AtomicBool = AtomicBool::new(false);

/// The approval the board has to persist: `Some` to store it, `None` to erase it. Only
/// the latest value matters, so a burst of changes is one write.
static APPROVAL_TO_STORE: Signal<CriticalSectionRawMutex, Option<CarDigest>> = Signal::new();

/// Raised on every change that can affect [`can_access_state`].
pub(crate) static CAN_ACCESS_CHANGED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Why a policy command was refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CanAccessError {
    /// Approving needs the platform and the exact car to be identified first.
    CarNotIdentified,
    /// This is a release build: it has no dev lock.
    NoDevLock,
}

impl CanAccessError {
    pub fn message(self) -> &'static str {
        match self {
            Self::CarNotIdentified => "the car is not identified yet, so it can't be approved",
            Self::NoDevLock => "this is a release build: it has no dev lock",
        }
    }
}

fn car_digest(platform_id: u32, fingerprint: &[u8]) -> CarDigest {
    let mut hasher = Sha256::new();
    hasher.update(CAR_DIGEST_DOMAIN);
    hasher.update(platform_id.to_le_bytes());
    hasher.update(fingerprint);
    hasher.finalize().into()
}

// ── Car identity, second stage ────────────────────────────────────────────────

/// Records which exact car the device is plugged into. The vehicle calls it after
/// [`crate::confirm_car_identity`], with bytes that are stable for this one car and differ
/// between cars of the same model: the VIN where the bus carries it, otherwise other
/// unique identifiers it reports (ECU serial numbers, for example). Never values that
/// change, like the odometer.
///
/// Refused (returns `false`) for an empty fingerprint or while the platform is not
/// confirmed. [`crate::revoke_car_identity`] forgets it.
pub fn confirm_car_fingerprint(fingerprint: &[u8]) -> bool {
    if fingerprint.is_empty() || !crate::can::is_car_identity_confirmed() {
        return false;
    }
    let digest = car_digest(crate::platform_id(), fingerprint);
    POLICY.lock(|p| p.borrow_mut().car = Some(digest));
    // A `revoke_car_identity` that raced the check above must not leave this car behind
    // for a later platform confirmation to pick up.
    if !crate::can::is_car_identity_confirmed() {
        forget_car();
        return false;
    }
    crate::can::notify_can_read_only_changed();
    true
}

/// Returns `true` once the vehicle has confirmed both its platform and the exact car.
pub fn is_car_fingerprint_confirmed() -> bool {
    crate::can::is_car_identity_confirmed() && POLICY.lock(|p| p.borrow().car.is_some())
}

/// Forgets the identified car. Called by `revoke_car_identity`.
pub(crate) fn forget_car() {
    POLICY.lock(|p| p.borrow_mut().car = None);
}

// ── Owner approval ────────────────────────────────────────────────────────────

/// Approves read-write mode for the car identified now, replacing any earlier approval
/// (for this car or another one). Refused while the car isn't identified. The board
/// persists it ([`can_read_write_approval_to_store`]).
pub fn approve_can_read_write() -> Result<(), CanAccessError> {
    let platform_confirmed = crate::can::is_car_identity_confirmed();
    let digest = POLICY.lock(|p| {
        let mut p = p.borrow_mut();
        let car = p.car.filter(|_| platform_confirmed)?;
        p.approval = Some(car);
        Some(car)
    });
    let digest = digest.ok_or(CanAccessError::CarNotIdentified)?;
    APPROVAL_TO_STORE.signal(Some(digest));
    crate::can::notify_can_read_only_changed();
    Ok(())
}

/// Revokes the approval, which makes the bus read-only straight away. The board erases it.
pub fn revoke_can_read_write_approval() {
    POLICY.lock(|p| p.borrow_mut().approval = None);
    APPROVAL_TO_STORE.signal(None);
    crate::can::notify_can_read_only_changed();
}

/// Loads the approval the board persisted. Called once at boot, before the transports
/// start. Until then the device counts as not approved.
pub fn restore_can_read_write_approval(approval: Option<CarDigest>) {
    POLICY.lock(|p| p.borrow_mut().approval = approval);
    crate::can::notify_can_read_only_changed();
}

/// Waits for the next approval change the board has to persist: `Some` to store,
/// `None` to erase.
pub async fn can_read_write_approval_to_store() -> Option<CarDigest> {
    APPROVAL_TO_STORE.wait().await
}

/// Non-blocking [`can_read_write_approval_to_store`]: `None` if nothing is pending.
pub fn try_take_can_read_write_approval_to_store() -> Option<Option<CarDigest>> {
    APPROVAL_TO_STORE.try_take()
}

/// The approval compared with the identified car.
pub fn can_read_write_approval() -> proto::CanReadWriteApproval {
    let platform_confirmed = crate::can::is_car_identity_confirmed();
    POLICY.lock(|p| {
        let p = p.borrow();
        match (p.approval, p.car.filter(|_| platform_confirmed)) {
            (None, _) => proto::CanReadWriteApproval::NotApproved,
            (Some(_), None) => proto::CanReadWriteApproval::AwaitingCar,
            (Some(approved), Some(car)) if approved == car => proto::CanReadWriteApproval::Approved,
            (Some(_), Some(_)) => proto::CanReadWriteApproval::OtherCar,
        }
    })
}

// ── Dev lock ──────────────────────────────────────────────────────────────────

/// Removes the dev lock. Called once at boot by the generated `main.rs` of a
/// `--production` build, and nowhere else.
pub fn declare_production_build() {
    PRODUCTION_BUILD.store(true, Ordering::Relaxed);
    crate::can::notify_can_read_only_changed();
}

/// Returns `true` in development builds, which have the dev lock.
pub fn has_can_dev_write_lock() -> bool {
    !PRODUCTION_BUILD.load(Ordering::Relaxed)
}

/// Unlocks (`true`) or locks (`false`) read-write mode in development builds. Refused by
/// release builds, which have no dev lock.
pub fn set_can_dev_write_unlocked(unlocked: bool) -> Result<(), CanAccessError> {
    if !has_can_dev_write_lock() {
        return Err(CanAccessError::NoDevLock);
    }
    POLICY.lock(|p| p.borrow_mut().dev_unlocked = unlocked);
    crate::can::notify_can_read_only_changed();
    Ok(())
}

/// Returns `true` while the dev lock is unlocked. Always `false` in release builds.
pub fn is_can_dev_write_unlocked() -> bool {
    has_can_dev_write_lock() && POLICY.lock(|p| p.borrow().dev_unlocked)
}

// ── The policy check ──────────────────────────────────────────────────────────

/// `true` only when the identified car is the approved one and, in development builds,
/// the dev lock is unlocked. Read in one critical section.
pub(crate) fn allows_read_write() -> bool {
    let production = PRODUCTION_BUILD.load(Ordering::Relaxed);
    crate::can::is_car_identity_confirmed()
        && POLICY.lock(|p| {
            let p = p.borrow();
            p.car.is_some() && p.approval == p.car && (production || p.dev_unlocked)
        })
}

/// Clears the whole policy (car, approval, dev lock) and any pending store. For tests
/// only: a test process shares these statics across tests.
pub fn reset_can_access_for_tests() {
    POLICY.lock(|p| {
        let mut p = p.borrow_mut();
        p.car = None;
        p.approval = None;
        p.dev_unlocked = false;
    });
    APPROVAL_TO_STORE.reset();
    LAST_PUBLISHED.lock(|last| *last.borrow_mut() = None);
}

/// Why the device is read-only, in words for the app, or `None` while it may transmit.
/// The first unmet condition, in the order the app's summary uses. Derived from
/// [`crate::is_can_read_only`], so `None` exactly when that is `false`.
pub fn can_read_only_reason() -> Option<&'static str> {
    if !crate::can::is_can_read_only() {
        return None;
    }
    Some(if crate::can_log::is_can_logging() {
        "CAN logging is active"
    } else if !crate::can::is_car_identity_confirmed() {
        "the car platform is not recognised yet"
    } else if !is_car_fingerprint_confirmed() {
        "the car is not identified yet"
    } else if can_read_write_approval() == proto::CanReadWriteApproval::OtherCar {
        "control was approved for a different car"
    } else if can_read_write_approval() != proto::CanReadWriteApproval::Approved {
        "control is not approved for this car"
    } else if is_can_dev_write_locked() {
        "the developer write lock is locked"
    } else {
        // The vehicle's own lock: not unlocked yet, or relocked after a bus fault.
        "the vehicle has not enabled control"
    })
}

fn is_can_dev_write_locked() -> bool {
    has_can_dev_write_lock() && !is_can_dev_write_unlocked()
}

// ── What the app sees ─────────────────────────────────────────────────────────

/// The access state the app shows: which conditions hold and the effective state.
pub fn can_access_state() -> proto::CanAccessState {
    proto::CanAccessState {
        platform_confirmed: Some(crate::can::is_car_identity_confirmed()),
        car_identity_confirmed: Some(is_car_fingerprint_confirmed()),
        approval: Some(can_read_write_approval() as i32),
        dev_lock_available: Some(has_can_dev_write_lock()),
        dev_unlocked: Some(is_can_dev_write_unlocked()),
        logging: Some(crate::can_log::is_can_logging()),
        read_only: Some(crate::can::is_can_read_only()),
    }
}

/// A BLE state update carrying only the system state with [`can_access_state`].
pub fn can_access_state_update(timestamp_ms: u64) -> proto::DeviceToApp {
    proto::DeviceToApp {
        timestamp_ms,
        platform_id: crate::platform_id(),
        payload: Some(proto::device_to_app::Payload::StateUpdate(
            proto::StateUpdate {
                system_state: Some(proto::SystemState {
                    can_access: Some(can_access_state()),
                    ..Default::default()
                }),
                vehicle_state: None,
            },
        )),
    }
}

/// How often the access state is sent again while unchanged. A message sent just as the
/// app subscribes can be lost before its notification stream delivers anything, and the
/// state may not change again for a long time; this bounds how long the app goes without it.
pub const CAN_ACCESS_REPEAT_MS: u64 = 5_000;

/// The last state queued for the app and when, so unchanged states aren't sent more often
/// than every [`CAN_ACCESS_REPEAT_MS`].
static LAST_PUBLISHED: Mutex<
    CriticalSectionRawMutex,
    RefCell<Option<(proto::CanAccessState, u64)>>,
> = Mutex::new(RefCell::new(None));

/// Queues [`can_access_state_update`] on BLE if the state changed since the last one
/// queued, or if that was [`CAN_ACCESS_REPEAT_MS`] or more ago. `try_send`: never blocks;
/// a full queue leaves it unsent, to retry on the next call. Returns whether a message
/// was queued.
pub fn publish_can_access_state(timestamp_ms: u64) -> bool {
    let state = can_access_state();
    let due = LAST_PUBLISHED.lock(|last| match last.borrow().as_ref() {
        Some((sent, sent_at)) => {
            *sent != state || timestamp_ms.saturating_sub(*sent_at) >= CAN_ACCESS_REPEAT_MS
        }
        None => true,
    });
    if !due {
        return false;
    }
    let msg = can_access_state_update(timestamp_ms);
    if BLE_TX_CHANNEL.try_send(msg).is_err() {
        return false;
    }
    LAST_PUBLISHED.lock(|last| *last.borrow_mut() = Some((state, timestamp_ms)));
    true
}

/// Sends the access state again even if unchanged, e.g. when the app subscribes.
pub fn resend_can_access_state() {
    LAST_PUBLISHED.lock(|last| *last.borrow_mut() = None);
    CAN_ACCESS_CHANGED.signal(());
}

/// Publishes the access state over BLE whenever it changes, and every
/// [`CAN_ACCESS_REPEAT_MS`] while it doesn't. Also checks once a second, which retries a
/// send that found the queue full.
#[embassy_executor::task]
pub async fn publish_can_access_task() {
    loop {
        publish_can_access_state(Instant::now().as_millis());
        let _ = embassy_futures::select::select(
            CAN_ACCESS_CHANGED.wait(),
            Timer::after(Duration::from_secs(1)),
        )
        .await;
    }
}
