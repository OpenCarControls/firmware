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
                    let id = match f.id() {
                        Id::Standard(sid) => Id::Standard(sid),
                        Id::Extended(eid) => Id::Extended(eid),
                    };
                    let dlc = f.dlc() as u8;
                    let mut data = [0u8; 8];
                    let bytes = f.data();
                    data[..bytes.len()].copy_from_slice(bytes);
                    let core_frame = CanFrame { bus_id, id, data, dlc };
                    if passes_filter(&core_frame, filters) {
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
            let data = &outbound.data[..outbound.dlc as usize];
            let sc_frame: Option<socketcan::CanFrame> = match outbound.id {
                Id::Standard(sid) => {
                    let sc_sid = socketcan::StandardId::new(sid.as_raw()).unwrap();
                    socketcan::CanDataFrame::new(sc_sid, data)
                        .map(socketcan::CanFrame::Data)
                }
                Id::Extended(eid) => {
                    let sc_eid = socketcan::ExtendedId::new(eid.as_raw()).unwrap();
                    socketcan::CanDataFrame::new(sc_eid, data)
                        .map(socketcan::CanFrame::Data)
                }
            };
            if let Some(frame) = sc_frame {
                let _ = socket.write_frame(&frame);
            }
        }
    }
}

fn passes_filter(frame: &CanFrame, filters: &[CanFilter]) -> bool {
    let frame_raw = match frame.id {
        Id::Standard(sid) => sid.as_raw() as u32,
        Id::Extended(eid) => eid.as_raw(),
    };
    filters
        .iter()
        .filter(|f| f.bus_id == frame.bus_id)
        .any(|f| {
            let filter_raw = match f.id {
                Id::Standard(sid) => sid.as_raw() as u32,
                Id::Extended(eid) => eid.as_raw(),
            };
            (frame_raw & f.mask) == (filter_raw & f.mask)
        })
}


