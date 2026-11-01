//! Recognises an E-GMP bus: the first stage of the car identity (see `docs/can.md`, "Car
//! identity"). Passive: it only looks at frames the car already sends.
//!
//! Two kinds of evidence, both needed:
//! - **The maker.** Hyundai, Kia and Genesis protect many frames with a CRC-16 that also
//!   covers the frame ID (opendbc's `hkg_can_fd_checksum`). A frame from another make
//!   passes it by chance about once in 65,536, so a few passing IDs mean a recent
//!   Hyundai/Kia/Genesis car.
//! - **An EV.** Frames only an electric car sends (battery and regen status).
//!
//! It doesn't tell E-GMP apart from other Hyundai/Kia EVs: they share these IDs and what
//! they mean, and the owner still approves the exact car.
//!
//! Cars whose M-CAN carries these frames split into 29-bit pieces instead (an Ioniq 6 and
//! a GV60, probably with a newer head unit) are not supported, and don't pass.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/// IDs whose frames carry the checksum in bytes 0-1 (the climate group, DISPLAY_SETTINGS_472
/// and its neighbours), checked on captures from an EV6 and a second E-GMP car, probably an
/// Ioniq 5. None of them is on the bus in every state (not with the car off and only the
/// doors used), so the check waits for them.
pub(crate) const CHECKSUM_IDS: [u32; 13] = [
    0x2FF, 0x380, 0x382, 0x383, 0x384, 0x3B0, 0x3B1, 0x3B2, 0x427, 0x42A, 0x472, 0x473, 0x474,
];

/// BATTERY_INFO_764 and REGEN_STATUS: only an EV sends them.
pub(crate) const EV_IDS: [u32; 2] = [0x2FC, 0x2E2];

/// How many different checksum IDs must pass before the platform is confirmed.
const MIN_PASSING_IDS: usize = 3;
/// How many frames of an ID must pass for it to count.
const MIN_PASSES_PER_ID: u8 = 2;
/// Failures in a row on one ID that revoke a confirmed platform. No real EV6 frame ever
/// failed, and the CAN controller already drops frames damaged on the wire.
const MAX_FAILURES_IN_A_ROW: u8 = 3;

/// The Hyundai/Kia/Genesis checksum of a frame: CRC-16 (polynomial 0x1021, initial 0)
/// over the bytes after the checksum, then the ID's low and high bytes, XOR a constant
/// for the frame length.
pub(crate) fn hkg_checksum(id: u32, data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    let id_bytes = [id as u8, (id >> 8) as u8];
    for &byte in data[2..].iter().chain(id_bytes.iter()) {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc ^ match data.len() {
        8 => 0x5F29,
        16 => 0x041D,
        24 => 0x819D,
        32 => 0x9F5B,
        _ => 0,
    }
}

/// What the check decided after a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Verdict {
    Confirmed,
    Revoked,
}

pub(crate) struct PlatformCheck {
    passes: [u8; CHECKSUM_IDS.len()],
    failures: [u8; CHECKSUM_IDS.len()],
    ev_seen: [bool; EV_IDS.len()],
    confirmed: bool,
}

impl PlatformCheck {
    pub(crate) const fn new() -> Self {
        Self {
            passes: [0; CHECKSUM_IDS.len()],
            failures: [0; CHECKSUM_IDS.len()],
            ev_seen: [false; EV_IDS.len()],
            confirmed: false,
        }
    }

    /// Feeds one frame. Returns a verdict only when the platform becomes confirmed or
    /// stops matching; after a revocation, it needs fresh evidence to confirm again.
    pub(crate) fn on_frame(&mut self, id: u32, data: &[u8]) -> Option<Verdict> {
        if let Some(i) = EV_IDS.iter().position(|&e| e == id) {
            self.ev_seen[i] |= data.len() == 8;
        } else if let Some(i) = CHECKSUM_IDS.iter().position(|&c| c == id) {
            // A sender with no value yet fills the frame with zeros or puts FF FF in the
            // checksum (seen as a car woke up); that says nothing about the car.
            if data.iter().all(|&b| b == 0) || data.starts_with(&[0xFF, 0xFF]) {
                return None;
            }
            let passed =
                data.len() == 8 && hkg_checksum(id, data) == u16::from_le_bytes([data[0], data[1]]);
            if passed {
                self.passes[i] = self.passes[i].saturating_add(1);
                self.failures[i] = 0;
            } else {
                // Evidence for an ID only counts while its frames keep passing.
                self.passes[i] = 0;
                self.failures[i] = self.failures[i].saturating_add(1);
                if self.confirmed && self.failures[i] >= MAX_FAILURES_IN_A_ROW {
                    *self = Self::new();
                    return Some(Verdict::Revoked);
                }
            }
        }

        if !self.confirmed && self.matches() {
            self.confirmed = true;
            return Some(Verdict::Confirmed);
        }
        None
    }

    fn matches(&self) -> bool {
        let passing = self
            .passes
            .iter()
            .filter(|&&n| n >= MIN_PASSES_PER_ID)
            .count();
        passing >= MIN_PASSING_IDS && self.ev_seen.iter().all(|&seen| seen)
    }
}

static PLATFORM: embassy_sync::blocking_mutex::Mutex<
    CriticalSectionRawMutex,
    core::cell::RefCell<PlatformCheck>,
> = embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(PlatformCheck::new()));

/// Runs the check on a received frame and tells `core-interface` when the platform is
/// confirmed or revoked.
pub(crate) fn track_platform(id: u32, data: &[u8]) {
    match PLATFORM.lock(|p| p.borrow_mut().on_frame(id, data)) {
        Some(Verdict::Confirmed) => core_interface::confirm_car_identity(),
        Some(Verdict::Revoked) => core_interface::revoke_car_identity(),
        None => {}
    }
}

#[cfg(test)]
pub(crate) fn reset_platform_for_tests() {
    PLATFORM.lock(|p| *p.borrow_mut() = PlatformCheck::new());
}

#[cfg(test)]
mod tests {
    use super::*;

    // Real frames from EV6 captures (log_41, car on; log_08 for the placeholders).
    const CLIMATE_380: [u8; 8] = [0x06, 0x16, 0x03, 0x0C, 0x0C, 0x0C, 0x0C, 0x01];
    const DATC_382: [u8; 8] = [0xDB, 0xEA, 0x0C, 0x08, 0x00, 0x04, 0x09, 0x00];
    const DISPLAY_472: [u8; 8] = [0x34, 0xA5, 0x00, 0x40, 0x88, 0x21, 0xC0, 0x44];
    const BATTERY_2FC: [u8; 8] = [0x00, 0x00, 0x08, 0x0A, 0x00, 0x00, 0x8F, 0x75];
    const REGEN_2E2: [u8; 8] = [0xFF, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x00, 0x08];
    const PLACEHOLDER_474: [u8; 8] = [0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3E];

    fn corrupted(mut data: [u8; 8]) -> [u8; 8] {
        data[5] ^= 0x01;
        data
    }

    fn feed(check: &mut PlatformCheck, frames: &[(u32, [u8; 8])]) -> Option<Verdict> {
        frames
            .iter()
            .filter_map(|(id, data)| check.on_frame(*id, data))
            .last()
    }

    fn ev6_frames() -> [(u32, [u8; 8]); 8] {
        [
            (0x2FC, BATTERY_2FC),
            (0x2E2, REGEN_2E2),
            (0x380, CLIMATE_380),
            (0x382, DATC_382),
            (0x472, DISPLAY_472),
            (0x380, CLIMATE_380),
            (0x382, DATC_382),
            (0x472, DISPLAY_472),
        ]
    }

    #[test]
    fn real_frames_pass_the_checksum() {
        assert_eq!(hkg_checksum(0x380, &CLIMATE_380), 0x1606);
        assert_eq!(hkg_checksum(0x382, &DATC_382), 0xEADB);
        assert_eq!(hkg_checksum(0x472, &DISPLAY_472), 0xA534);
    }

    #[test]
    fn an_ev6_bus_confirms_the_platform_once() {
        let mut check = PlatformCheck::new();
        assert_eq!(feed(&mut check, &ev6_frames()), Some(Verdict::Confirmed));
        assert_eq!(feed(&mut check, &ev6_frames()), None, "confirmed only once");
    }

    /// Captures from another E-GMP car (probably an Ioniq 5): other payloads, same IDs.
    #[test]
    fn a_second_e_gmp_car_confirms_the_platform() {
        let mut check = PlatformCheck::new();
        let frames = [
            (0x2FC, [0x00, 0x00, 0x07, 0x09, 0x00, 0x00, 0x78, 0xA0]),
            (0x2E2, [0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x04]),
            (0x380, [0xAD, 0xD7, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x01]),
            (0x382, [0xF1, 0x7A, 0x01, 0x00, 0x00, 0x01, 0x08, 0x00]),
            (0x472, [0x5C, 0xEC, 0x00, 0x40, 0x08, 0x41, 0x00, 0x45]),
        ];
        assert_eq!(feed(&mut check, &frames), None, "each ID passed once");
        assert_eq!(feed(&mut check, &frames), Some(Verdict::Confirmed));
    }

    #[test]
    fn checksum_frames_alone_are_not_enough() {
        let mut check = PlatformCheck::new();
        let frames: alloc::vec::Vec<_> = ev6_frames()
            .into_iter()
            .filter(|(id, _)| !EV_IDS.contains(id))
            .collect();
        assert_eq!(feed(&mut check, &frames), None, "no EV frames");
    }

    #[test]
    fn frames_with_wrong_checksums_never_confirm() {
        let mut check = PlatformCheck::new();
        let frames = ev6_frames().map(|(id, data)| {
            if EV_IDS.contains(&id) {
                (id, data)
            } else {
                (id, corrupted(data))
            }
        });
        for _ in 0..10 {
            assert_eq!(feed(&mut check, &frames), None);
        }
    }

    #[test]
    fn the_same_frames_on_other_ids_never_confirm() {
        // The checksum covers the ID: these payloads on another car's IDs fail.
        let mut check = PlatformCheck::new();
        let frames = ev6_frames().map(|(id, data)| match id {
            0x380 => (0x383, data),
            0x382 => (0x384, data),
            0x472 => (0x473, data),
            _ => (id, data),
        });
        assert_eq!(feed(&mut check, &frames), None);
    }

    /// An Ioniq 6 capture: the same checksummed frames, but as 29-bit pieces (0x380 is
    /// sent as 0x0E038001), and no 0x2FC or 0x2E2. Not supported, so it must not pass.
    #[test]
    fn an_ioniq_6_bus_does_not_confirm() {
        let mut check = PlatformCheck::new();
        let frames = [
            (
                0x0E03_8001,
                [0xAD, 0xD7, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x01],
            ),
            (
                0x0E03_8201,
                [0x22, 0x3D, 0x00, 0x00, 0x00, 0x01, 0x08, 0x00],
            ),
            (
                0x11C4_7201,
                [0xC0, 0x16, 0x00, 0x40, 0x08, 0x26, 0xC0, 0x44],
            ),
            (
                0x0BC2_FF01,
                [0xE2, 0x01, 0x00, 0xF1, 0x3F, 0x00, 0x00, 0x00],
            ),
            (0x4F0, [0x4A, 0x0D, 0x1D, 0x14, 0x68, 0x1A, 0x07, 0x02]),
            (0x411, [0xE4, 0x92, 0x50, 0x50, 0x00, 0x00, 0x00, 0x00]),
            (0x3E4, [0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
        ];
        for _ in 0..10 {
            assert_eq!(feed(&mut check, &frames), None);
        }
    }

    #[test]
    fn evidence_for_an_id_resets_when_it_fails() {
        let mut check = PlatformCheck::new();
        let mut frames = ev6_frames().to_vec();
        frames.insert(6, (0x382, corrupted(DATC_382)));
        assert_eq!(feed(&mut check, &frames), None);
        assert_eq!(
            check.on_frame(0x382, &DATC_382),
            Some(Verdict::Confirmed),
            "the second pass after the failure"
        );
    }

    #[test]
    fn placeholder_frames_are_ignored() {
        let mut check = PlatformCheck::new();
        assert_eq!(feed(&mut check, &ev6_frames()), Some(Verdict::Confirmed));
        for _ in 0..10 {
            assert_eq!(check.on_frame(0x474, &PLACEHOLDER_474), None);
            assert_eq!(check.on_frame(0x380, &[0; 8]), None);
        }
    }

    #[test]
    fn traffic_that_stops_matching_revokes_the_platform() {
        let mut check = PlatformCheck::new();
        assert_eq!(feed(&mut check, &ev6_frames()), Some(Verdict::Confirmed));
        let bad = (0x380, corrupted(CLIMATE_380));
        assert_eq!(
            feed(&mut check, &[bad, bad]),
            None,
            "two in a row is not enough"
        );
        assert_eq!(check.on_frame(0x380, &CLIMATE_380), None);
        assert_eq!(feed(&mut check, &[bad, bad, bad]), Some(Verdict::Revoked));

        // Revoked: it needs the full evidence again.
        assert_eq!(check.on_frame(0x380, &CLIMATE_380), None);
        assert_eq!(feed(&mut check, &ev6_frames()), Some(Verdict::Confirmed));
    }

    #[test]
    fn tracking_confirms_and_revokes_in_core_interface() {
        core_interface::revoke_car_identity();
        reset_platform_for_tests();
        for (id, data) in ev6_frames() {
            track_platform(id, &data);
        }
        assert!(core_interface::is_car_identity_confirmed());

        for _ in 0..MAX_FAILURES_IN_A_ROW {
            track_platform(0x472, &corrupted(DISPLAY_472));
        }
        assert!(!core_interface::is_car_identity_confirmed());
        reset_platform_for_tests();
    }
}
