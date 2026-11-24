#[cfg(target_os = "linux")]
mod linux_can {
    use core_interface::{CAN_RX_CHANNEL, CanFilter, CanFrame};
    use embassy_time::{Duration, Instant, Timer};
    use embedded_can::{Frame as EmbeddedFrame, Id};
    use socketcan::{CanError, CanSocket, Socket, SocketOptions};
    use std::io::ErrorKind;

    /// Converts a received `socketcan::CanDataFrame` into a `core_interface::CanFrame`,
    /// tagging it with the given `bus_id`.
    pub(crate) fn socketcan_to_core_frame(f: &socketcan::CanDataFrame, bus_id: u8) -> CanFrame {
        let id = f.id();
        let dlc = f.dlc() as u8;
        let mut data = [0u8; 8];
        let bytes = f.data();
        data[..bytes.len()].copy_from_slice(bytes);
        CanFrame {
            bus_id,
            id,
            data,
            dlc,
        }
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

    /// How often each bus reports its RX counters for the bus health the app shows.
    const RX_STATS_INTERVAL: Duration = Duration::from_secs(5);

    /// Counts one SocketCAN error frame: a bus-off, a receive buffer overflow, or else a
    /// receive error.
    pub(crate) fn record_error_frame(counts: &mut core_interface::CanRxCounts, error: &CanError) {
        use socketcan::ErrorCause;
        use socketcan::errors::ControllerProblems;
        if error.causes().any(|c| matches!(c, ErrorCause::BusOff)) {
            counts.record_bus_off();
        } else if error.causes().any(|c| {
            matches!(c, ErrorCause::Controller(p) if p.contains(ControllerProblems::RX_OVERFLOW))
        }) {
            counts.record_overrun();
        } else {
            counts.record_error();
        }
    }

    /// Opens a SocketCAN socket on `interface` and runs a bidirectional CAN loop
    /// for bus number `bus_id`. Up to 4 concurrent instances are supported (`pool_size`).
    ///
    /// - RX: Polls the non-blocking socket every 1 ms via Embassy timer. Software-filters
    ///   received frames against `filters` and forwards matches to `CAN_RX_CHANNEL`.
    /// - TX: After each RX poll, drains this bus's `CAN_TX_CHANNELS` entry and writes
    ///   the frames to the socket.
    /// - Health: counts frames and error frames, and reports them every 5 s for the bus
    ///   health the app shows (`core_interface::report_can_bus_window`).
    ///
    /// If the interface cannot be opened (e.g. it does not exist), the task logs a
    /// warning and retries every 5 seconds so the rest of the system keeps running.
    #[embassy_executor::task(pool_size = 4)]
    pub async fn socket_can_task(
        interface: &'static str,
        bus_id: u8,
        filters: &'static [CanFilter],
    ) {
        let socket = loop {
            match CanSocket::open(interface) {
                Ok(s) => break s,
                Err(e) => {
                    log::warn!(
                        "CAN bus {}: failed to open '{}': {} — retrying in 5s",
                        bus_id,
                        interface,
                        e
                    );
                    Timer::after(Duration::from_secs(5)).await;
                }
            }
        };

        if let Err(e) = socket.set_nonblocking(true) {
            log::error!(
                "CAN bus {}: failed to set '{}' non-blocking: {}",
                bus_id,
                interface,
                e
            );
            return;
        }

        let Some(tx_channel) = core_interface::can_tx_channel(bus_id) else {
            log::error!("CAN bus {}: bus id exceeds MAX_CAN_BUSES", bus_id);
            return;
        };

        // Error frames feed the bus health. A virtual (vcan) bus never sends any.
        if let Err(e) = socket.set_error_filter_accept_all() {
            log::warn!(
                "CAN bus {}: no error frames from '{}': {}",
                bus_id,
                interface,
                e
            );
        }
        let mut counts = core_interface::CanRxCounts::new();
        let mut next_report = Instant::now() + RX_STATS_INTERVAL;

        loop {
            Timer::after(Duration::from_millis(1)).await;

            if Instant::now() >= next_report {
                // The window started one interval before it was due.
                let window_ms = Instant::now()
                    .saturating_duration_since(next_report - RX_STATS_INTERVAL)
                    .as_millis();
                // SocketCAN has no listen-only mode here: read-only drops TX in software.
                core_interface::report_can_bus_window(bus_id, &counts.take(), window_ms, None);
                next_report = Instant::now() + RX_STATS_INTERVAL;
            }

            // RX: drain all available frames from the kernel rx buffer
            loop {
                match socket.read_frame() {
                    Ok(socketcan::CanFrame::Data(f)) => {
                        let core_frame = socketcan_to_core_frame(&f, bus_id);
                        core_interface::log_can_frame(&core_frame);
                        let matched = core_interface::passes_filter(&core_frame, filters);
                        counts.record_frame(matched);
                        // The vehicle doesn't see frames while SavvyCAN has the buses.
                        if matched && !core_interface::is_can_logging() {
                            CAN_RX_CHANNEL.sender().send(core_frame).await;
                        }
                    }
                    Ok(socketcan::CanFrame::Error(f)) => {
                        record_error_frame(&mut counts, &f.into_error());
                    }
                    // Remote frames are ignored
                    Ok(socketcan::CanFrame::Remote(_)) => {}
                    Err(ref e) if e.kind() == ErrorKind::WouldBlock => break,
                    Err(_) => break,
                }
            }

            // TX: drain this bus's own outbound channel
            while let Ok(outbound) = tx_channel.try_receive() {
                // Drop silently when in read-only mode; do not transmit on the bus.
                // `allow_can_tx` also lets an identity probe's own frames through, and
                // checks a command frame's phone is still validated.
                if !core_interface::allow_can_tx(bus_id, &outbound) {
                    continue;
                }
                let sc_frame = core_to_socketcan_frame(outbound.frame());
                if let Some(frame) = sc_frame {
                    let _ = socket.write_frame(&frame);
                }
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        fn counted(error: CanError) -> core_interface::CanRxCounts {
            let mut counts = core_interface::CanRxCounts::new();
            record_error_frame(&mut counts, &error);
            counts
        }

        #[test]
        fn error_frames_are_counted_by_cause() {
            use socketcan::ErrorCause;
            use socketcan::errors::ControllerProblems;
            assert_eq!(counted(CanError::new(ErrorCause::BusOff)).bus_offs, 1);
            let overflow = ErrorCause::Controller(ControllerProblems::RX_OVERFLOW);
            assert_eq!(counted(CanError::new(overflow)).overruns, 1);
            let both = CanError::from_multiple(overflow, [ErrorCause::BusOff]);
            assert_eq!(counted(both.clone()).bus_offs, 1);
            assert_eq!(counted(both).overruns, 0);
            assert_eq!(counted(CanError::new(ErrorCause::NoAck)).errors, 1);
        }
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

        // -- socketcan_to_core_frame -------------------------------------------------

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

        // -- core_to_socketcan_frame -------------------------------------------------

        #[test]
        fn core_to_sc_standard_id_roundtrip() {
            let core = core_std_frame(0, 0x123, &[1, 2, 3]);
            let sc = core_to_socketcan_frame(&core).unwrap();
            let back = socketcan_to_core_frame(
                match &sc {
                    socketcan::CanFrame::Data(f) => f,
                    _ => panic!("expected data frame"),
                },
                0,
            );
            assert_eq!(back.id, core.id);
            assert_eq!(back.dlc, core.dlc);
            assert_eq!(
                &back.data[..back.dlc as usize],
                &core.data[..core.dlc as usize]
            );
        }

        #[test]
        fn core_to_sc_extended_id_roundtrip() {
            let core = core_ext_frame(1, 0x1FFFFFFF, &[0xDE, 0xAD]);
            let sc = core_to_socketcan_frame(&core).unwrap();
            let back = socketcan_to_core_frame(
                match &sc {
                    socketcan::CanFrame::Data(f) => f,
                    _ => panic!("expected data frame"),
                },
                1,
            );
            assert_eq!(back.id, core.id);
            assert_eq!(&back.data[..back.dlc as usize], &[0xDE, 0xAD]);
        }

        #[test]
        fn core_to_sc_max_payload() {
            let core = core_std_frame(0, 0x100, &[1, 2, 3, 4, 5, 6, 7, 8]);
            assert!(core_to_socketcan_frame(&core).is_some());
        }
    }
}

#[cfg(target_os = "linux")]
pub use linux_can::socket_can_task;

#[cfg(not(target_os = "linux"))]
mod dummy_can {
    use core_interface::CanFilter;
    use embassy_time::{Duration, Instant, Timer};

    #[embassy_executor::task(pool_size = 4)]
    pub async fn socket_can_task(
        interface: &'static str,
        bus_id: u8,
        _filters: &'static [CanFilter],
    ) {
        log::warn!(
            "CAN bus {}: Dummy CAN interface '{}' active (Windows/Mac). RX ignored, TX dropped.",
            bus_id,
            interface
        );

        let Some(tx_channel) = core_interface::can_tx_channel(bus_id) else {
            log::error!("CAN bus {}: bus id exceeds MAX_CAN_BUSES", bus_id);
            return;
        };

        loop {
            Timer::after(Duration::from_millis(10)).await;

            // TX: drain this bus's own outbound channel
            while let Ok(_outbound) = tx_channel.try_receive() {
                // Silently drop on non-Linux
            }
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub use dummy_can::socket_can_task;
