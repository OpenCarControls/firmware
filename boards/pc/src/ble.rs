use embassy_time::{Duration, Timer};
use std::path::Path;
use std::sync::OnceLock;

static BLE_PAIRED_STORE_PATH: OnceLock<&'static str> = OnceLock::new();

pub(crate) fn ble_paired_store_path() -> &'static str {
    BLE_PAIRED_STORE_PATH
        .get()
        .copied()
        .unwrap_or("/tmp/opencar-paired-phones.txt")
}

pub fn set_ble_paired_store_path(path: &'static str) {
    let _ = BLE_PAIRED_STORE_PATH.set(path);
}

fn to_hex(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        use std::fmt::Write as _;
        let _ = write!(&mut out, "{:02x}", b);
    }
    out
}

fn from_hex(s: &str) -> Option<Vec<u8>> {
    if s.len() % 2 != 0 {
        return None;
    }
    let mut out = Vec::with_capacity(s.len() / 2);
    let bytes = s.as_bytes();
    let mut i = 0usize;
    while i < bytes.len() {
        let hi = (bytes[i] as char).to_digit(16)?;
        let lo = (bytes[i + 1] as char).to_digit(16)?;
        out.push(((hi << 4) | lo) as u8);
        i += 2;
    }
    Some(out)
}

pub(crate) fn load_paired_phones_from_file(path: &str) -> Vec<Vec<u8>> {
    let content = match std::fs::read_to_string(path) {
        Ok(v) => v,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Vec::new(),
        Err(e) => {
            log::warn!("BLE store: failed to read '{}': {:?}", path, e);
            return Vec::new();
        }
    };
    content
        .lines()
        .filter_map(|line| from_hex(line.trim()))
        .filter(|id| !id.is_empty())
        .collect()
}

fn save_paired_phones_to_file(path: &str, phones: &[Vec<u8>]) {
    let parent = Path::new(path).parent();
    if let Some(dir) = parent {
        let _ = std::fs::create_dir_all(dir);
    }
    let mut body = String::new();
    for id in phones {
        body.push_str(&to_hex(id));
        body.push('\n');
    }
    if let Err(e) = std::fs::write(path, body) {
        log::warn!("BLE store: failed to write '{}': {:?}", path, e);
    }
}

pub(crate) async fn persist_paired_phones() {
    let phones = core_interface::list_paired_phones().await;
    save_paired_phones_to_file(ble_paired_store_path(), &phones);
}

fn find_header_end(buf: &[u8]) -> Option<usize> {
    buf.windows(4).position(|w| w == b"\r\n\r\n")
}

fn parse_request_line(headers: &str) -> Option<(&str, &str)> {
    let first_line = headers.lines().next()?;
    let mut parts = first_line.split_whitespace();
    let method = parts.next()?;
    let path = parts.next()?;
    Some((method, path))
}

fn parse_content_length(headers: &str) -> usize {
    for line in headers.lines() {
        let lower = line.to_ascii_lowercase();
        if let Some(v) = lower.strip_prefix("content-length:") {
            return v.trim().parse::<usize>().unwrap_or(0);
        }
    }
    0
}

async fn read_http_request(
    stream: &mut std::net::TcpStream,
) -> Result<(String, String, Vec<u8>), ()> {
    use std::io::Read;

    let mut buf: Vec<u8> = Vec::new();
    let mut temp = [0u8; 1024];

    loop {
        if find_header_end(&buf).is_some() {
            break;
        }
        match stream.read(&mut temp) {
            Ok(0) => return Err(()),
            Ok(n) => {
                buf.extend_from_slice(&temp[..n]);
                if buf.len() > 16 * 1024 {
                    return Err(());
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                Timer::after(Duration::from_millis(1)).await;
            }
            Err(_) => return Err(()),
        }
    }

    let header_end = match find_header_end(&buf) {
        Some(v) => v,
        None => return Err(()),
    };

    let header_bytes = &buf[..header_end];
    let headers = match std::str::from_utf8(header_bytes) {
        Ok(v) => v,
        Err(_) => return Err(()),
    };

    let (method, path) = match parse_request_line(headers) {
        Some(v) => v,
        None => return Err(()),
    };

    let content_len = parse_content_length(headers);
    let body_start = header_end + 4;
    let mut body = if body_start < buf.len() {
        buf[body_start..].to_vec()
    } else {
        Vec::new()
    };

    while body.len() < content_len {
        match stream.read(&mut temp) {
            Ok(0) => return Err(()),
            Ok(n) => body.extend_from_slice(&temp[..n]),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                Timer::after(Duration::from_millis(1)).await;
            }
            Err(_) => return Err(()),
        }
    }

    body.truncate(content_len);
    Ok((method.to_string(), path.to_string(), body))
}

async fn write_http_response(
    stream: &mut std::net::TcpStream,
    status: &str,
    content_type: &str,
    body: &[u8],
) -> Result<(), ()> {
    use std::io::Write;

    let mut head = format!(
        "HTTP/1.1 {}\r\nContent-Length: {}\r\nConnection: close\r\n",
        status,
        body.len()
    );
    if !content_type.is_empty() {
        head.push_str(&format!("Content-Type: {}\r\n", content_type));
    }
    head.push_str("\r\n");

    let mut data = head.into_bytes();
    data.extend_from_slice(body);

    let mut written = 0usize;
    while written < data.len() {
        match stream.write(&data[written..]) {
            Ok(0) => return Err(()),
            Ok(n) => written += n,
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                Timer::after(Duration::from_millis(1)).await;
            }
            Err(_) => return Err(()),
        }
    }

    let _ = stream.flush();
    Ok(())
}

fn peer_device_id(stream: &std::net::TcpStream) -> Vec<u8> {
    use std::net::IpAddr;

    let peer = match stream.peer_addr() {
        Ok(v) => v,
        Err(_) => return vec![0],
    };

    match peer.ip() {
        IpAddr::V4(v4) => v4.octets().to_vec(),
        IpAddr::V6(v6) => v6.octets().to_vec(),
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PairHttpResult {
    WindowClosed,
    Paired,
    ListFull,
}

async fn pair_phone_for_http(device_id: &[u8], pairing_window_open: bool) -> PairHttpResult {
    if !pairing_window_open {
        return PairHttpResult::WindowClosed;
    }

    if core_interface::add_paired_phone(device_id).await {
        PairHttpResult::Paired
    } else {
        PairHttpResult::ListFull
    }
}

async fn handle_request(
    stream: &mut std::net::TcpStream,
    method: &str,
    path: &str,
    body: &[u8],
    pairing_window_s: u32,
) -> Result<(), ()> {
    use prost::Message as _;

    match (method, path) {
        ("POST", "/cmd") => {
            let mut msg = match core_interface::proto::AppToDevice::decode(body) {
                Ok(v) => v,
                Err(_) => {
                    return write_http_response(
                        stream,
                        "400 Bad Request",
                        "text/plain",
                        b"invalid AppToDevice payload",
                    )
                    .await;
                }
            };

            msg.source_device_id = peer_device_id(stream);
            core_interface::BLE_RX_CHANNEL.send(msg).await;
            write_http_response(stream, "204 No Content", "", &[]).await
        }
        ("GET", "/state") => match core_interface::BLE_TX_CHANNEL.try_receive() {
            Ok(msg) => {
                let mut encoded = Vec::new();
                if msg.encode(&mut encoded).is_err() {
                    return write_http_response(
                        stream,
                        "500 Internal Server Error",
                        "text/plain",
                        b"encode failed",
                    )
                    .await;
                }
                write_http_response(stream, "200 OK", "application/octet-stream", &encoded).await
            }
            Err(_) => write_http_response(stream, "204 No Content", "", &[]).await,
        },
        ("POST", "/pairing") => {
            core_interface::open_pairing_window_for(pairing_window_s);
            write_http_response(stream, "200 OK", "text/plain", b"pairing window opened").await
        }
        ("POST", "/pair") => {
            let pairing_window_open = core_interface::is_pairing_window_open();
            match pair_phone_for_http(&peer_device_id(stream), pairing_window_open).await {
                PairHttpResult::WindowClosed => {
                    write_http_response(
                        stream,
                        "403 Forbidden",
                        "text/plain",
                        b"pairing window is closed",
                    )
                    .await
                }
                PairHttpResult::Paired => {
                    persist_paired_phones().await;
                    write_http_response(stream, "200 OK", "text/plain", b"paired").await
                }
                PairHttpResult::ListFull => {
                    write_http_response(stream, "409 Conflict", "text/plain", b"bond list is full")
                        .await
                }
            }
        }
        ("POST", "/clear-bonds") => {
            let removed = core_interface::clear_paired_phones().await;
            persist_paired_phones().await;
            let response = format!("cleared {} bonded phone(s)", removed);
            write_http_response(stream, "200 OK", "text/plain", response.as_bytes()).await
        }
        _ => write_http_response(stream, "404 Not Found", "text/plain", b"not found").await,
    }
}

#[embassy_executor::task]
pub async fn ble_http_task(port: u16, device_name: &'static str, pairing_window_s: u32) {
    use std::io;

    let addr = format!("0.0.0.0:{}", port);
    let listener = match std::net::TcpListener::bind(&addr) {
        Ok(v) => v,
        Err(e) => {
            log::error!("BLE test HTTP: bind failed on {}: {}", addr, e);
            return;
        }
    };
    if let Err(e) = listener.set_nonblocking(true) {
        log::error!("BLE test HTTP: set_nonblocking failed: {}", e);
        return;
    }

    log::info!(
        "BLE test HTTP server ready on {} (device_name_base={})",
        addr,
        device_name
    );

    loop {
        match listener.accept() {
            Ok((mut stream, _peer)) => {
                if let Err(e) = stream.set_nonblocking(true) {
                    log::warn!("BLE test HTTP: failed to set stream non-blocking: {}", e);
                    continue;
                }

                let request = read_http_request(&mut stream).await;
                match request {
                    Ok((method, path, body)) => {
                        let _ =
                            handle_request(&mut stream, &method, &path, &body, pairing_window_s)
                                .await;
                    }
                    Err(()) => {
                        let _ = write_http_response(
                            &mut stream,
                            "400 Bad Request",
                            "text/plain",
                            b"invalid HTTP request",
                        )
                        .await;
                    }
                }
            }
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => {
                Timer::after(Duration::from_millis(1)).await;
            }
            Err(e) => {
                log::warn!("BLE test HTTP: accept error: {}", e);
                Timer::after(Duration::from_millis(20)).await;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{PairHttpResult, pair_phone_for_http};

    fn reset_pairing_state() {
        embassy_futures::block_on(core_interface::clear_paired_phones());
        core_interface::set_ble_max_bonded_phones(8);
    }

    #[test]
    fn pair_rejected_when_window_closed() {
        reset_pairing_state();

        let result = embassy_futures::block_on(pair_phone_for_http(&[1, 2, 3, 4], false));
        assert_eq!(result, PairHttpResult::WindowClosed);

        let paired = embassy_futures::block_on(core_interface::is_phone_paired(&[1, 2, 3, 4]));
        assert!(!paired);
    }

    #[test]
    fn pair_succeeds_when_window_open() {
        reset_pairing_state();

        let result = embassy_futures::block_on(pair_phone_for_http(&[5, 6, 7, 8], true));
        assert_eq!(result, PairHttpResult::Paired);

        let paired = embassy_futures::block_on(core_interface::is_phone_paired(&[5, 6, 7, 8]));
        assert!(paired);
        reset_pairing_state();
    }

    #[test]
    fn pair_returns_list_full_when_capacity_reached() {
        reset_pairing_state();
        core_interface::set_ble_max_bonded_phones(1);

        let first = embassy_futures::block_on(pair_phone_for_http(&[0xAA], true));
        assert_eq!(first, PairHttpResult::Paired);

        let second = embassy_futures::block_on(pair_phone_for_http(&[0xBB], true));
        assert_eq!(second, PairHttpResult::ListFull);

        let first_paired = embassy_futures::block_on(core_interface::is_phone_paired(&[0xAA]));
        let second_paired = embassy_futures::block_on(core_interface::is_phone_paired(&[0xBB]));
        assert!(first_paired);
        assert!(!second_paired);

        reset_pairing_state();
    }
}
