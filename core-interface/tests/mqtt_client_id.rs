//! The device tells the app its MQTT client ID over BLE. Its own test binary, since
//! `set_mqtt_client_id()` holds for the rest of the process.

use core_interface::{can_access_state_update, init, proto, set_mqtt_client_id};

fn reported_client_id() -> Option<String> {
    match can_access_state_update(0).payload {
        Some(proto::device_to_app::Payload::StateUpdate(update)) => {
            update.system_state.unwrap().mqtt_client_id
        }
        other => panic!("expected a state update, got {other:?}"),
    }
}

#[test]
fn the_access_state_reports_the_mqtt_client_id_only_when_there_is_one() {
    init(0xDEAD_BEEF);
    assert_eq!(reported_client_id(), None, "no MQTT in this build");

    set_mqtt_client_id("dev-car");
    assert_eq!(reported_client_id().as_deref(), Some("dev-car"));
}
