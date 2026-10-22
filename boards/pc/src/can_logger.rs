//! SavvyCAN network logger: a TCP listener that serves one GVRET client at a time.
//! The protocol and session logic live in `core-interface`.

use crate::network::NonBlockingTcpStream;
use embassy_time::{Duration, Timer};

/// Listens on `port` and streams the CAN buses to a connected SavvyCAN until it
/// disconnects. Only one client at a time; further connections wait in the backlog.
#[embassy_executor::task]
pub async fn can_logger_task(port: u16, num_buses: u8) {
    let listener = loop {
        match std::net::TcpListener::bind(("0.0.0.0", port)) {
            Ok(l) => break l,
            Err(e) => {
                log::warn!("CAN logger: cannot listen on port {port}: {e}, retrying in 5s");
                Timer::after(Duration::from_secs(5)).await;
            }
        }
    };
    if let Err(e) = listener.set_nonblocking(true) {
        log::error!("CAN logger: set_nonblocking failed: {e}");
        return;
    }
    log::info!("CAN logger: listening on port {port}");

    loop {
        let stream = match listener.accept() {
            Ok((stream, peer)) => {
                log::info!("CAN logger: SavvyCAN connected from {peer}");
                stream
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                Timer::after(Duration::from_millis(100)).await;
                continue;
            }
            Err(e) => {
                log::warn!("CAN logger: accept failed: {e}");
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
        };
        if stream.set_nonblocking(true).is_err() {
            continue;
        }
        let mut stream = NonBlockingTcpStream(stream);
        let end = core_interface::run_can_log_session(&mut stream, num_buses).await;
        log::info!(
            "CAN logger: SavvyCAN disconnected ({} frame(s) dropped, {} transmit request(s) ignored)",
            end.dropped,
            end.tx_requests_ignored
        );
    }
}
