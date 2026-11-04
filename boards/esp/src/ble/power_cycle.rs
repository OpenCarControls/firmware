//! Bond reset by power-cycling, for boards without a pairing button.
//!
//! Once a phone is bonded, nothing else opens such a board's pairing window,
//! so an owner whose phone lost its bond would be locked out. Power-cycling it
//! quickly a few times in a row clears every bond, and the board then opens the
//! first-setup window as a new device does. The thresholds come from the
//! contract (`[ble.pairing]` in `transport.toml`), so the app's instructions
//! match. Only power-on resets count: a watchdog, panic or brownout loop must
//! never clear the bonds.

/// The gesture's thresholds. The generated `main.rs` passes `None` on boards
/// with a pairing button, which open the window with it instead.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub struct PowerCycleReset {
    /// Short boots in a row after which the next power-on clears the bonds.
    pub short_boots: u8,
    /// A boot counts as short when it loses power before this uptime.
    pub uptime_s: u32,
}

/// What a boot does with the short-boot count stored in flash.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(in crate::ble) struct BootCount {
    /// The count to store now, cleared again once the boot passes the uptime
    /// threshold.
    pub store: u8,
    /// Whether this boot clears every bond.
    pub reset_bonds: bool,
}

/// Counts a boot. `stored` is the number of short power-on boots in a row
/// before this one. Every power-on boot counts as short until it reaches the
/// uptime threshold; any other reset ends the run, so a crash loop between
/// power cycles can't add up to a reset.
#[cfg_attr(not(feature = "hardware"), allow(dead_code))]
pub(in crate::ble) fn count_boot(stored: u8, power_on: bool, config: PowerCycleReset) -> BootCount {
    if !power_on {
        return BootCount {
            store: 0,
            reset_bonds: false,
        };
    }
    let reset_bonds = stored >= config.short_boots;
    BootCount {
        store: if reset_bonds {
            1
        } else {
            stored.saturating_add(1)
        },
        reset_bonds,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CONFIG: PowerCycleReset = PowerCycleReset {
        short_boots: 3,
        uptime_s: 10,
    };

    /// Boots `power_ons` times in a row with power-on resets, each one short,
    /// and returns whether any of them reset the bonds.
    fn short_power_ons(mut stored: u8, power_ons: usize) -> (u8, bool) {
        let mut reset = false;
        for _ in 0..power_ons {
            let boot = count_boot(stored, true, CONFIG);
            reset |= boot.reset_bonds;
            stored = boot.store;
        }
        (stored, reset)
    }

    #[test]
    fn power_on_after_the_short_boots_resets_the_bonds() {
        // Three short boots don't reset yet...
        assert_eq!(short_power_ons(0, 3), (3, false));
        // ...the power-on after them does.
        assert_eq!(
            count_boot(3, true, CONFIG),
            BootCount {
                store: 1,
                reset_bonds: true
            }
        );
    }

    #[test]
    fn other_resets_never_count_and_end_the_run() {
        for stored in [0, 2, 3, u8::MAX] {
            assert_eq!(
                count_boot(stored, false, CONFIG),
                BootCount {
                    store: 0,
                    reset_bonds: false
                }
            );
        }
    }
}
