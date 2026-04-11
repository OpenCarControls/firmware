use alloc::vec::Vec;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, with_timeout};

use crate::channels::{BLE_TX_CHANNEL, CAN_DEBUG_RX_CHANNEL};
use crate::proto;
use crate::types::{CanDebugFilter, CanRawCapture};

// ── CAN Debug State ───────────────────────────────────────────────────────────

/// When `true`, board CAN driver loops forward raw (unfiltered) incoming frames
/// to `CAN_DEBUG_RX_CHANNEL` before applying the vehicle `passes_filter` check.
/// Defaults to `false` at boot; toggled via the `SetCanDebugEnabled` BLE command.
static CAN_DEBUG_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Bitmask of bus IDs being observed by the debug feature. Bit N = bus_id N.
/// `0xFF` means all buses. Only meaningful when `CAN_DEBUG_ACTIVE` is `true`.
static DEBUG_BUS_MASK: AtomicU8 = AtomicU8::new(0);

/// Count of frames dropped since the last batch flush because `CAN_DEBUG_RX_CHANNEL`
/// was full (BLE could not drain fast enough). Swapped to 0 on each flush.
static CAN_DEBUG_DROPPED: AtomicU32 = AtomicU32::new(0);

/// The active debug blocklist. Protected by a mutex because it is written from
/// the `process_ble_commands_task` and read from `publish_can_debug_task`.
static DEBUG_FILTERS: Mutex<CriticalSectionRawMutex, Vec<CanDebugFilter>> =
    Mutex::new(Vec::new());

// ── Public API (called by board drivers) ──────────────────────────────────────

/// Returns `true` if CAN debug streaming is currently active.
///
/// Board CAN driver loops call this (lock-free) before deciding whether to
/// forward a raw frame to `CAN_DEBUG_RX_CHANNEL`.
pub fn is_can_debug_active() -> bool {
    CAN_DEBUG_ACTIVE.load(Ordering::Relaxed)
}

/// Returns `true` if CAN debug streaming is active AND this `bus_id` is being observed.
///
/// Lock-free; safe to call from the CAN I/O loop on core 1.
pub fn can_debug_wants_bus(bus_id: u8) -> bool {
    is_can_debug_active() && (DEBUG_BUS_MASK.load(Ordering::Relaxed) & (1 << bus_id)) != 0
}

/// Increments the dropped-frame counter by one. Call this when a `try_send` to
/// `CAN_DEBUG_RX_CHANNEL` fails because the channel is full.
pub fn increment_can_debug_dropped() {
    CAN_DEBUG_DROPPED.fetch_add(1, Ordering::Relaxed);
}

// ── pub(crate) bridge helpers (called by dispatch) ────────────────────────────

/// Enables CAN debug streaming for the given bus IDs.
///
/// Clears the blocklist, resets the dropped counter, programs the bus mask, and
/// sets `CAN_DEBUG_ACTIVE = true`. `bus_ids` is the raw proto field; an empty
/// slice means "all buses" (mask = 0xFF).
///
/// Order: state is fully prepared before the active flag is set so board drivers
/// on core 1 never observe a partially-initialised debug state.
pub(crate) async fn enable_can_debug(bus_ids: &[u32]) {
    {
        let mut filters = DEBUG_FILTERS.lock().await;
        filters.clear();
    }
    let mask = if bus_ids.is_empty() {
        0xFF
    } else {
        bus_ids.iter().fold(0u8, |acc, &id| acc | (1 << (id as u8)))
    };
    DEBUG_BUS_MASK.store(mask, Ordering::Relaxed);
    CAN_DEBUG_DROPPED.store(0, Ordering::Relaxed);
    CAN_DEBUG_ACTIVE.store(true, Ordering::Relaxed);
}

/// Disables CAN debug streaming immediately.
///
/// Sets `CAN_DEBUG_ACTIVE = false` so board drivers on core 1 stop tapping.
pub(crate) fn disable_can_debug() {
    CAN_DEBUG_ACTIVE.store(false, Ordering::Relaxed);
}

/// Replaces the active blocklist. No-op if CAN debug is currently inactive.
pub(crate) async fn update_can_debug_filters(new_filters: Vec<CanDebugFilter>) {
    if is_can_debug_active() {
        let mut filters = DEBUG_FILTERS.lock().await;
        *filters = new_filters;
    }
}

// ── Debug publish helpers ─────────────────────────────────────────────────────

/// Builds and sends a single `CanDebugUpdate` batch to `BLE_TX_CHANNEL`.
///
/// Each capture in `captures` is converted to a `CanDebugFrame`. Captures that
/// match any entry in `debug_filters` (blocklist) are silently excluded — this
/// does NOT increment the dropped counter. The batch is skipped entirely if
/// there are no frames after filtering AND `dropped` is 0 (avoids empty
/// BLE notifications).
///
/// Extracted from the task loop so it can be unit-tested without an executor.
pub async fn publish_single_debug_batch(
    captures: &[CanRawCapture],
    debug_filters: &[CanDebugFilter],
    dropped: u32,
    pid: u32,
    timestamp_ms: u64,
) {
    let frames: Vec<proto::CanDebugFrame> = captures
        .iter()
        .filter(|cap| {
            let (cap_raw, cap_extended) = match cap.id {
                embedded_can::Id::Standard(sid) => (sid.as_raw() as u32, false),
                embedded_can::Id::Extended(eid) => (eid.as_raw(), true),
            };
            // Include the frame unless it matches a blocklist entry.
            !debug_filters.iter().any(|f| {
                f.is_extended_id == cap_extended
                    && (cap_raw & f.mask) == (f.can_id & f.mask)
            })
        })
        .map(|cap| {
            let (can_id, is_extended_id) = match cap.id {
                embedded_can::Id::Standard(sid) => (sid.as_raw() as u32, false),
                embedded_can::Id::Extended(eid) => (eid.as_raw(), true),
            };
            proto::CanDebugFrame {
                timestamp_ms: cap.timestamp_ms,
                bus_id: cap.bus_id as u32,
                can_id,
                is_extended_id,
                data: cap.data[..cap.dlc as usize].to_vec(),
            }
        })
        .collect();

    if frames.is_empty() && dropped == 0 {
        return;
    }

    BLE_TX_CHANNEL
        .sender()
        .send(proto::DeviceToApp {
            timestamp_ms,
            platform_id: pid,
            payload: Some(proto::device_to_app::Payload::CanDebugUpdate(
                proto::CanDebugUpdate {
                    frames,
                    dropped_frames: dropped,
                },
            )),
        })
        .await;
}

/// Collects raw CAN frames from `CAN_DEBUG_RX_CHANNEL`, applies the active
/// blocklist, and flushes batches to `BLE_TX_CHANNEL`.
///
/// Flush triggers: 20 frames accumulated OR 50 ms elapsed, whichever comes
/// first. When debug is inactive the task still drains any lingering frames
/// from the channel so it never fills from a now-stale enable.
#[embassy_executor::task]
pub async fn publish_can_debug_task() {
    let receiver = CAN_DEBUG_RX_CHANNEL.receiver();
    loop {
        if !is_can_debug_active() {
            // Drain and discard — keeps the channel clear when debug is off.
            while receiver.try_receive().is_ok() {}
            embassy_time::Timer::after(Duration::from_millis(50)).await;
            continue;
        }

        // Accumulate up to 20 frames or 50 ms, whichever comes first.
        let mut batch: Vec<CanRawCapture> = Vec::new();
        loop {
            if batch.len() >= 20 {
                break;
            }
            match with_timeout(Duration::from_millis(50), receiver.receive()).await {
                Ok(cap) => batch.push(cap),
                Err(_timeout) => break,
            }
        }

        // Atomically take the dropped count for this batch.
        let dropped = CAN_DEBUG_DROPPED.swap(0, Ordering::Relaxed);

        // Clone current blocklist under lock, then release before the await.
        let filters: Vec<CanDebugFilter> = {
            let guard = DEBUG_FILTERS.lock().await;
            guard.iter().map(|f| CanDebugFilter {
                can_id: f.can_id,
                is_extended_id: f.is_extended_id,
                mask: f.mask,
            }).collect()
        };

        publish_single_debug_batch(
            &batch,
            &filters,
            dropped,
            crate::PLATFORM_ID.load(Ordering::Relaxed),
            Instant::now().as_millis(),
        )
        .await;
    }
}

// ── Test-only accessors ───────────────────────────────────────────────────────

/// Returns the number of entries in the current debug blocklist.
/// Intended for use in tests to verify that `UpdateCanDebugFilters` dispatch
/// correctly stored the new list without exposing the mutex publicly.
pub async fn debug_filter_count() -> usize {
    DEBUG_FILTERS.lock().await.len()
}

/// Returns the current value of the dropped-frame counter without resetting it.
/// Intended for use in tests to verify that `SetCanDebugEnabled` resets the
/// counter on enable.
pub fn debug_dropped_count() -> u32 {
    CAN_DEBUG_DROPPED.load(Ordering::Relaxed)
}
