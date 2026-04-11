use core_interface::{CanFilter, CanFrame, CAN_RX_CHANNEL, CAN_TX_CHANNEL};
use embedded_can::{Frame as EmbeddedFrame, Id};
use embassy_time::{Duration, Timer};
use socketcan::{CanSocket, Socket};
use std::io::ErrorKind;

pub fn start(spawner: &embassy_executor::Spawner) {
    spawner.spawn(core_interface::process_ble_commands_task()).unwrap();
    spawner.spawn(core_interface::process_mqtt_commands_task()).unwrap();
    spawner.spawn(core_interface::route_responses_task()).unwrap();
    spawner.spawn(core_interface::publish_state_task()).unwrap();
}

// ── Frame conversion helpers ──────────────────────────────────────────────────

/// Converts a received `socketcan::CanDataFrame` into a `core_interface::CanFrame`,
/// tagging it with the given `bus_id`.
pub(crate) fn socketcan_to_core_frame(f: &socketcan::CanDataFrame, bus_id: u8) -> CanFrame {
    let id = match f.id() {
        Id::Standard(sid) => Id::Standard(sid),
        Id::Extended(eid) => Id::Extended(eid),
    };
    let dlc = f.dlc() as u8;
    let mut data = [0u8; 8];
    let bytes = f.data();
    data[..bytes.len()].copy_from_slice(bytes);
    CanFrame { bus_id, id, data, dlc }
}

/// Converts a `core_interface::CanFrame` into a `socketcan::CanFrame` for
/// transmission. Returns `None` if the frame data is malformed.
pub(crate) fn core_to_socketcan_frame(frame: &CanFrame) -> Option<socketcan::CanFrame> {
    let data = &frame.data[..frame.dlc as usize];
    match frame.id {
        Id::Standard(sid) => {
            let sc_sid = socketcan::StandardId::new(sid.as_raw()).unwrap();
            socketcan::CanDataFrame::new(sc_sid, data).map(socketcan::CanFrame::Data)
        }
        Id::Extended(eid) => {
            let sc_eid = socketcan::ExtendedId::new(eid.as_raw()).unwrap();
            socketcan::CanDataFrame::new(sc_eid, data).map(socketcan::CanFrame::Data)
        }
    }
}

/// Opens a SocketCAN socket on `interface` and runs a bidirectional CAN loop
/// for bus number `bus_id`. Up to 4 concurrent instances are supported (`pool_size`).
///
/// - RX: Polls the non-blocking socket every 1 ms via Embassy timer. Software-filters
///   received frames against `filters` and forwards matches to `CAN_RX_CHANNEL`.
/// - TX: After each RX poll, drains `CAN_TX_CHANNEL` for frames addressed to
///   `bus_id` and writes them to the socket. Frames for other buses are returned
///   to the channel so the corresponding task can pick them up.
#[embassy_executor::task(pool_size = 4)]
pub async fn socket_can_task(
    interface: &'static str,
    bus_id: u8,
    filters: &'static [CanFilter],
) {
    let socket = CanSocket::open(interface)
        .unwrap_or_else(|e| panic!("Failed to open SocketCAN interface '{}': {}", interface, e));

    socket.set_nonblocking(true).expect("Failed to set SocketCAN socket to non-blocking");

    loop {
        Timer::after(Duration::from_millis(1)).await;

        // RX: drain all available frames from the kernel rx buffer
        loop {
            match socket.read_frame() {
                Ok(socketcan::CanFrame::Data(f)) => {
                    let core_frame = socketcan_to_core_frame(&f, bus_id);
                    if core_interface::passes_filter(&core_frame, filters) {
                        CAN_RX_CHANNEL.sender().send(core_frame).await;
                    }
                }
                // Remote frames and error frames are ignored
                Ok(_) => {}
                Err(ref e) if e.kind() == ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }

        // TX: drain outbound frames for this bus, return others to the channel
        while let Ok(outbound) = CAN_TX_CHANNEL.receiver().try_receive() {
            if outbound.bus_id != bus_id {
                // Not ours — return to the back of the channel for the correct bus task
                let _ = CAN_TX_CHANNEL.sender().try_send(outbound);
                break; // avoid spinning on someone else's frames
            }
            // Drop silently when in read-only mode; do not transmit on the bus.
            if core_interface::is_can_read_only() {
                continue;
            }
            let sc_frame = core_to_socketcan_frame(&outbound);
            if let Some(frame) = sc_frame {
                let _ = socket.write_frame(&frame);
            }
        }
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_can::{ExtendedId, StandardId};

    fn sc_std_frame(raw_id: u16, data: &[u8]) -> socketcan::CanDataFrame {
        let sid = socketcan::StandardId::new(raw_id).unwrap();
        socketcan::CanDataFrame::new(sid, data).unwrap()
    }

    fn sc_ext_frame(raw_id: u32, data: &[u8]) -> socketcan::CanDataFrame {
        let eid = socketcan::ExtendedId::new(raw_id).unwrap();
        socketcan::CanDataFrame::new(eid, data).unwrap()
    }

    fn core_std_frame(bus_id: u8, raw_id: u16, data: &[u8]) -> CanFrame {
        let mut buf = [0u8; 8];
        buf[..data.len()].copy_from_slice(data);
        CanFrame {
            bus_id,
            id: Id::Standard(StandardId::new(raw_id).unwrap()),
            data: buf,
            dlc: data.len() as u8,
        }
    }

    fn core_ext_frame(bus_id: u8, raw_id: u32, data: &[u8]) -> CanFrame {
        let mut buf = [0u8; 8];
        buf[..data.len()].copy_from_slice(data);
        CanFrame {
            bus_id,
            id: Id::Extended(ExtendedId::new(raw_id).unwrap()),
            data: buf,
            dlc: data.len() as u8,
        }
    }

    // ── socketcan_to_core_frame ───────────────────────────────────────────────

    #[test]
    fn sc_to_core_standard_id_preserved() {
        let f = sc_std_frame(0x123, &[1, 2, 3]);
        let core = socketcan_to_core_frame(&f, 0);
        assert_eq!(core.id, Id::Standard(StandardId::new(0x123).unwrap()));
    }

    #[test]
    fn sc_to_core_extended_id_preserved() {
        let f = sc_ext_frame(0x1234_5678, &[0xAA]);
        let core = socketcan_to_core_frame(&f, 2);
        assert_eq!(core.id, Id::Extended(ExtendedId::new(0x1234_5678).unwrap()));
    }

    #[test]
    fn sc_to_core_bus_id_tagged() {
        let f = sc_std_frame(0x100, &[]);
        assert_eq!(socketcan_to_core_frame(&f, 0).bus_id, 0);
        assert_eq!(socketcan_to_core_frame(&f, 3).bus_id, 3);
    }

    #[test]
    fn sc_to_core_data_and_dlc_copied() {
        let data = [0x11, 0x22, 0x33, 0x44];
        let f = sc_std_frame(0x200, &data);
        let core = socketcan_to_core_frame(&f, 0);
        assert_eq!(core.dlc, 4);
        assert_eq!(&core.data[..4], &data);
    }

    #[test]
    fn sc_to_core_empty_frame() {
        let f = sc_std_frame(0x300, &[]);
        let core = socketcan_to_core_frame(&f, 0);
        assert_eq!(core.dlc, 0);
    }

    // ── core_to_socketcan_frame ───────────────────────────────────────────────

    #[test]
    fn core_to_sc_standard_id_roundtrip() {
        let core = core_std_frame(0, 0x123, &[1, 2, 3]);
        let sc = core_to_socketcan_frame(&core).unwrap();
        let back = socketcan_to_core_frame(match &sc {
            socketcan::CanFrame::Data(f) => f,
            _ => panic!("expected data frame"),
        }, 0);
        assert_eq!(back.id, core.id);
        assert_eq!(back.dlc, core.dlc);
        assert_eq!(&back.data[..back.dlc as usize], &core.data[..core.dlc as usize]);
    }

    #[test]
    fn core_to_sc_extended_id_roundtrip() {
        let core = core_ext_frame(1, 0x1FFFFFFF, &[0xDE, 0xAD]);
        let sc = core_to_socketcan_frame(&core).unwrap();
        let back = socketcan_to_core_frame(match &sc {
            socketcan::CanFrame::Data(f) => f,
            _ => panic!("expected data frame"),
        }, 1);
        assert_eq!(back.id, core.id);
        assert_eq!(&back.data[..back.dlc as usize], &[0xDE, 0xAD]);
    }

    #[test]
    fn core_to_sc_max_payload() {
        let core = core_std_frame(0, 0x100, &[1, 2, 3, 4, 5, 6, 7, 8]);
        assert!(core_to_socketcan_frame(&core).is_some());
    }
}

