use core_interface::{
    CanFrame, CanLogSession, CanTxFrame, IdentityProbe, MAX_PROBE_ATTEMPTS, ProbeRefused,
    confirm_car_fingerprint, confirm_car_identity, extended_id, is_can_bus_listen_only,
    is_can_bus_read_only, is_can_read_only, iso_tp_flow_control, log_can_frame,
    open_identity_probe, reset_can_access_for_tests, reset_identity_probe_for_tests,
    revoke_car_identity, set_can_read_only, standard_id, uds_single_frame,
};
use embedded_can::{ExtendedId, Id};

const REQUEST: [u8; 8] = [0x03, 0x22, 0xF1, 0x8C, 0x00, 0x00, 0x00, 0x00];
const FLOW_CONTROL: [u8; 8] = [0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];

/// The E-GMP head unit serial read: the request and the flow control, both on 0x780.
static PROBE: IdentityProbe = IdentityProbe {
    bus_id: 0,
    frames: &[
        (
            standard_id(0x780),
            uds_single_frame(&[0x22, 0xF1, 0x8C], 0x00),
        ),
        (standard_id(0x780), iso_tp_flow_control(0x00)),
    ],
};

/// The probe's frames are platform frames.
fn allow_can_tx(bus_id: u8, frame: &CanFrame) -> bool {
    core_interface::allow_can_tx(bus_id, &CanTxFrame::platform(*frame))
}

fn frame(bus_id: u8, id: Id, data: [u8; 8]) -> CanFrame {
    CanFrame {
        bus_id,
        id,
        data,
        dlc: 8,
    }
}

fn on_780(data: [u8; 8]) -> CanFrame {
    frame(0, standard_id(0x780), data)
}

/// A frame the car sent, so the bus counts as awake.
fn car_frame(bus_id: u8) {
    log_can_frame(&frame(bus_id, standard_id(0x380), [0; 8]));
}

/// Boot state, then the platform confirmed on an awake bus: when the probe is allowed.
fn platform_confirmed() {
    reset_identity_probe_for_tests();
    reset_can_access_for_tests();
    revoke_car_identity();
    confirm_car_identity();
    car_frame(0);
}

fn boot() {
    reset_identity_probe_for_tests();
    reset_can_access_for_tests();
    revoke_car_identity();
}

#[test]
fn the_helpers_build_the_declared_frames() {
    assert_eq!(PROBE.frames[0].1, REQUEST);
    assert_eq!(PROBE.frames[1].1, FLOW_CONTROL);
}

#[test]
fn nothing_can_be_sent_without_a_window() {
    platform_confirmed();
    assert!(is_can_bus_listen_only(0));
    assert!(!allow_can_tx(0, &on_780(REQUEST)));
    boot();
}

#[test]
fn a_window_sends_each_declared_frame_once() {
    platform_confirmed();
    let window = open_identity_probe(&PROBE).unwrap();

    // The controller leaves listen-only, but the bus is still read-only for everyone else.
    assert!(!is_can_bus_listen_only(0));
    assert!(is_can_bus_read_only(0));
    assert!(is_can_read_only());
    assert!(is_can_bus_listen_only(1), "only the probe's bus");

    assert!(allow_can_tx(0, &on_780(REQUEST)));
    assert!(
        !allow_can_tx(0, &on_780(REQUEST)),
        "the request goes out once"
    );
    assert!(allow_can_tx(0, &on_780(FLOW_CONTROL)));
    assert!(!allow_can_tx(0, &on_780(FLOW_CONTROL)));

    drop(window);
    assert!(is_can_bus_listen_only(0), "back to listen-only");
    boot();
}

#[test]
fn a_window_sends_nothing_else() {
    platform_confirmed();
    let _window = open_identity_probe(&PROBE).unwrap();

    let mut other_data = REQUEST;
    other_data[3] = 0x90; // 22 F190
    assert!(!allow_can_tx(0, &on_780(other_data)), "other bytes");
    assert!(
        !allow_can_tx(0, &frame(0, standard_id(0x7DF), REQUEST)),
        "another ID"
    );
    assert!(
        !allow_can_tx(1, &frame(1, standard_id(0x780), REQUEST)),
        "another bus"
    );
    assert!(
        !allow_can_tx(
            0,
            &frame(0, Id::Extended(ExtendedId::new(0x780).unwrap()), REQUEST)
        ),
        "an extended ID with the same number"
    );
    let short = CanFrame {
        dlc: 4,
        ..on_780(REQUEST)
    };
    assert!(!allow_can_tx(0, &short), "another length");

    assert!(allow_can_tx(0, &on_780(REQUEST)), "still unsent");
    drop(_window);
    boot();
}

/// OBD's VIN request to every ECU: the request goes to 0x7DF, the flow control to the
/// engine ECU's own ID (0x7E0), the one that answers from 0x7E8.
#[test]
fn each_frame_is_allowed_only_on_its_own_id() {
    static BROADCAST: IdentityProbe = IdentityProbe {
        bus_id: 0,
        frames: &[
            (standard_id(0x7DF), uds_single_frame(&[0x09, 0x02], 0x00)),
            (standard_id(0x7E0), iso_tp_flow_control(0x00)),
        ],
    };
    let request = [0x02, 0x09, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00];
    platform_confirmed();
    let _window = open_identity_probe(&BROADCAST).unwrap();
    assert!(!allow_can_tx(0, &frame(0, standard_id(0x7E0), request)));
    assert!(!allow_can_tx(
        0,
        &frame(0, standard_id(0x7DF), FLOW_CONTROL)
    ));
    assert!(allow_can_tx(0, &frame(0, standard_id(0x7DF), request)));
    assert!(allow_can_tx(0, &frame(0, standard_id(0x7E0), FLOW_CONTROL)));
    drop(_window);
    boot();
}

#[test]
fn a_probe_can_use_29_bit_ids() {
    static EXTENDED: IdentityProbe = IdentityProbe {
        bus_id: 0,
        frames: &[(
            extended_id(0x18DA_30F1),
            uds_single_frame(&[0x22, 0xF1, 0x90], 0xAA),
        )],
    };
    let request = [0x03, 0x22, 0xF1, 0x90, 0xAA, 0xAA, 0xAA, 0xAA];
    platform_confirmed();
    let _window = open_identity_probe(&EXTENDED).unwrap();
    assert!(
        !allow_can_tx(0, &frame(0, extended_id(0x18DA_31F1), request)),
        "another ECU"
    );
    assert!(allow_can_tx(
        0,
        &frame(0, extended_id(0x18DA_30F1), request)
    ));
    drop(_window);
    boot();
}

#[test]
fn a_window_needs_the_platform_and_no_car_yet() {
    boot();
    car_frame(0);
    assert_eq!(
        open_identity_probe(&PROBE).err(),
        Some(ProbeRefused::NotNeeded),
        "platform not confirmed"
    );

    platform_confirmed();
    assert!(confirm_car_fingerprint(b"hu-serial:TEST"));
    assert_eq!(
        open_identity_probe(&PROBE).err(),
        Some(ProbeRefused::NotNeeded),
        "car already identified"
    );
    boot();
}

#[test]
fn identifying_the_car_closes_the_window() {
    platform_confirmed();
    let _window = open_identity_probe(&PROBE).unwrap();
    assert!(confirm_car_fingerprint(b"hu-serial:TEST"));
    assert!(is_can_bus_listen_only(0));
    assert!(!allow_can_tx(0, &on_780(REQUEST)));
    drop(_window);
    boot();
}

#[test]
fn a_window_needs_an_awake_bus() {
    boot();
    confirm_car_identity();
    assert_eq!(
        open_identity_probe(&PROBE).err(),
        Some(ProbeRefused::BusAsleep)
    );
    car_frame(1);
    assert_eq!(
        open_identity_probe(&PROBE).err(),
        Some(ProbeRefused::BusAsleep),
        "traffic on another bus"
    );
    car_frame(0);
    assert!(open_identity_probe(&PROBE).is_ok());
    boot();
}

#[test]
fn a_logging_session_refuses_and_closes_windows() {
    platform_confirmed();
    let window = open_identity_probe(&PROBE).unwrap();
    let session = CanLogSession::start().unwrap();
    assert!(is_can_bus_listen_only(0));
    assert!(!allow_can_tx(0, &on_780(REQUEST)));
    drop(window);
    assert_eq!(
        open_identity_probe(&PROBE).err(),
        Some(ProbeRefused::Logging)
    );
    drop(session);
    boot();
}

#[test]
fn a_relock_closes_the_window() {
    platform_confirmed();
    let _window = open_identity_probe(&PROBE).unwrap();
    set_can_read_only(true); // a bus-off, for example
    assert!(is_can_bus_listen_only(0));
    assert!(!allow_can_tx(0, &on_780(REQUEST)));
    drop(_window);
    boot();
}

#[test]
fn one_window_at_a_time_and_a_few_per_boot() {
    platform_confirmed();
    let first = open_identity_probe(&PROBE).unwrap();
    assert_eq!(open_identity_probe(&PROBE).err(), Some(ProbeRefused::Busy));
    drop(first);
    for _ in 1..MAX_PROBE_ATTEMPTS {
        drop(open_identity_probe(&PROBE).unwrap());
    }
    assert_eq!(
        open_identity_probe(&PROBE).err(),
        Some(ProbeRefused::NoAttemptsLeft)
    );
    boot();
}

#[test]
fn invalid_probes_are_refused() {
    static ON_A_MISSING_BUS: IdentityProbe = IdentityProbe {
        bus_id: 9,
        frames: &[(standard_id(0x780), REQUEST)],
    };
    static NO_FRAMES: IdentityProbe = IdentityProbe {
        bus_id: 0,
        frames: &[],
    };
    static TOO_MANY_FRAMES: IdentityProbe = IdentityProbe {
        bus_id: 0,
        frames: &[(standard_id(0x780), REQUEST); 5],
    };
    platform_confirmed();
    for probe in [&ON_A_MISSING_BUS, &NO_FRAMES, &TOO_MANY_FRAMES] {
        assert_eq!(
            open_identity_probe(probe).err(),
            Some(ProbeRefused::Invalid)
        );
    }
    boot();
}
