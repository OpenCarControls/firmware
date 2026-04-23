#[cfg(feature = "hardware")]
use esp_hal::gpio::{AnyPin, Input, InputConfig, Pull};

#[cfg(feature = "hardware")]
#[embassy_executor::task]
pub async fn ble_lifecycle_task(
    pairing_button_pin: u8,
    pairing_button_hold_s: u32,
    pairing_window_s: u32,
    max_bonded_phones: u8,
    controller_lease_ttl_s: u32,
) {
    core_interface::set_ble_controller_lease_ttl_s(controller_lease_ttl_s);
    core_interface::set_ble_max_bonded_phones(max_bonded_phones);
    log::info!(
        "BLE lifecycle config loaded: button_gpio={}, hold_s={}, window_s={}, max_bonds={}, lease_ttl_s={}",
        pairing_button_pin,
        pairing_button_hold_s,
        pairing_window_s,
        max_bonded_phones,
        controller_lease_ttl_s
    );

    let hold_s = if pairing_button_hold_s == 0 {
        1
    } else {
        pairing_button_hold_s
    };
    let window_s = if pairing_window_s == 0 {
        1
    } else {
        pairing_window_s
    };

    let button = Input::new(
        unsafe { AnyPin::steal(pairing_button_pin) },
        InputConfig::default().with_pull(Pull::Up),
    );

    let mut pressed_since: Option<embassy_time::Instant> = None;
    let mut hold_fired = false;
    let mut pairing_was_open = core_interface::is_pairing_window_open();
    if pairing_was_open {
        log::info!("BLE pairing window open on startup");
    }

    loop {
        let now = embassy_time::Instant::now();
        let pressed = button.is_low();

        if pressed {
            if pressed_since.is_none() {
                pressed_since = Some(now);
                hold_fired = false;
            }
            if !hold_fired
                && pressed_since
                    .map(|t0| {
                        (now.as_millis().saturating_sub(t0.as_millis())) >= (hold_s as u64 * 1_000)
                    })
                    .unwrap_or(false)
            {
                core_interface::open_pairing_window_for(window_s);
                hold_fired = true;
                log::info!(
                    "BLE pairing window opened for {}s via button hold ({}s)",
                    window_s,
                    hold_s
                );
            }
        } else {
            pressed_since = None;
            hold_fired = false;
        }

        let pairing_open = core_interface::is_pairing_window_open();
        if pairing_open != pairing_was_open {
            if pairing_open {
                log::info!("BLE pairing window is now OPEN");
            } else {
                log::info!("BLE pairing window is now CLOSED");
            }
            pairing_was_open = pairing_open;
        }

        embassy_time::Timer::after(embassy_time::Duration::from_millis(25)).await;
    }
}
