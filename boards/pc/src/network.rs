use std::sync::Arc;

use core_interface::{MQTT_RX_CHANNEL, MQTT_STATE_TX, MQTT_TX_CHANNEL, MqttTls, Transport, proto};
use prost::Message as _;
use rust_mqtt::{
    Bytes,
    buffer::AllocBuffer,
    client::Client,
    client::MqttError,
    client::event::Event,
    client::options::{ConnectOptions, PublicationOptions, SubscriptionOptions, TopicReference},
    types::{MqttBinary, MqttString, TopicFilter, TopicName},
};

/// Adapts a non-blocking stream (a `std::net::TcpStream`, or TLS over one) to
/// `embedded_io_async`'s `Read` and `Write` traits by polling every 1 ms via Embassy
/// timer when the socket would block. This mirrors the pattern used by `socket_can_task`.
pub(crate) struct NonBlockingTcpStream<S = std::net::TcpStream>(pub(crate) S);

impl<S> embedded_io_async::ErrorType for NonBlockingTcpStream<S> {
    type Error = embedded_io_async::ErrorKind;
}

impl<S: std::io::Read + std::io::Write> embedded_io_async::Read for NonBlockingTcpStream<S> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        loop {
            match self.0.read(buf) {
                Ok(n) => return Ok(n),
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
                }
                Err(_) => return Err(embedded_io_async::ErrorKind::Other),
            }
        }
    }
}

impl<S: std::io::Read + std::io::Write> embedded_io_async::Write for NonBlockingTcpStream<S> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        loop {
            match self.0.write(buf) {
                Ok(n) => return Ok(n),
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
                }
                Err(_) => return Err(embedded_io_async::ErrorKind::Other),
            }
        }
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        loop {
            match self.0.flush() {
                Ok(()) => return Ok(()),
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
                }
                Err(_) => return Err(embedded_io_async::ErrorKind::Other),
            }
        }
    }
}

/// Shared MQTT session logic. Authenticates (if credentials provided),
/// subscribes, then drives inbound/outbound message exchange until an error
/// occurs, at which point it returns `Err(())` so the caller reconnects.
async fn run_mqtt_session<S: std::io::Read + std::io::Write>(
    transport: NonBlockingTcpStream<S>,
    client_id: &'static str,
    cmd_topic: &'static str,
    data_topic: &'static str,
    username: &'static str,
    password: &'static str,
) -> Result<(), ()> {
    use embassy_time::{Duration, Instant};

    let mut buffer = AllocBuffer;
    let mut client = Client::<_, _, 1, 1, 1, 0>::new(&mut buffer);

    let mut connect_opts = ConnectOptions::new().clean_start();
    if !username.is_empty() {
        connect_opts = connect_opts
            .user_name(MqttString::from_str_unchecked(username))
            .password(MqttBinary::from_slice_unchecked(password.as_bytes()));
    }

    client
        .connect(
            transport,
            &connect_opts,
            Some(MqttString::from_str_unchecked(client_id)),
        )
        .await
        .map_err(|e| {
            log::warn!("MQTT: connect failed: {:?}", e);
        })?;

    log::info!("MQTT: connected, subscribing to '{}'", cmd_topic);

    let filter = TopicFilter::new_unchecked(MqttString::from_str_unchecked(cmd_topic));
    client
        .subscribe(filter, SubscriptionOptions::new())
        .await
        .map_err(|e| {
            log::warn!("MQTT: subscribe failed: {:?}", e);
        })?;

    log::info!("MQTT: ready");
    core_interface::set_link_up(Transport::Mqtt, true);

    let keepalive = Duration::from_secs(55);
    let mut last_ping = Instant::now();
    let data_topic_name = TopicName::new_unchecked(MqttString::from_str_unchecked(data_topic));

    loop {
        // Drain outbound before polling inbound: responses and system state
        // first, then the newest vehicle state.
        while let Some(msg) = MQTT_TX_CHANNEL
            .try_receive()
            .ok()
            .or_else(|| MQTT_STATE_TX.try_take())
        {
            let mut buf = Vec::new();
            if msg.encode(&mut buf).is_ok() {
                let t = data_topic_name.as_borrowed();
                let pub_opts = PublicationOptions::new(TopicReference::Name(t));
                client
                    .publish(&pub_opts, Bytes::Borrowed(buf.as_slice()))
                    .await
                    .map_err(|e| {
                        log::warn!("MQTT: publish failed: {:?}", e);
                    })?;
            }
        }

        // Keepalive ping
        if last_ping.elapsed() >= keepalive {
            client.ping().await.map_err(|e| {
                log::warn!("MQTT: ping failed: {:?}", e);
            })?;
            last_ping = Instant::now();
        }

        // Poll for inbound with 100 ms timeout
        let poll_result =
            embassy_time::with_timeout(Duration::from_millis(100), client.poll()).await;

        match poll_result {
            Err(_timeout) => continue,
            Ok(Err(e)) => {
                log::warn!("MQTT: poll error: {:?}", e);
                // abort() panics if the broker sent a clean DISCONNECT
                // (e.g. SessionTakenOver) because the network is still in
                // Ok state. Only call it for IO/protocol errors.
                if !matches!(e, MqttError::Disconnect { .. }) {
                    client.abort().await;
                }
                return Err(());
            }
            Ok(Ok(Event::Publish(p))) => {
                if p.topic.as_ref().as_str() == cmd_topic {
                    let payload = p.message.as_bytes();
                    if let Ok(msg) = proto::AppToDevice::decode(payload) {
                        // Use try_send to avoid blocking here. If we blocked on
                        // MQTT_RX_CHANNEL.send().await while downstream tasks are
                        // simultaneously blocked on BASIC_CMD_CHANNEL and
                        // CMD_RESP_CHANNEL, route_responses_task would be stuck
                        // trying to send to MQTT_TX_CHANNEL — which we can't drain
                        // because we're blocked. That forms a deadlock cycle.
                        // Dropping one inbound command under back-pressure is safe;
                        // the app can retry. What we must never do is stall the
                        // outbound drain loop.
                        if MQTT_RX_CHANNEL.try_send(msg).is_err() {
                            log::warn!("MQTT: RX channel full, dropping inbound command");
                        }
                    }
                }
            }
            Ok(Ok(_)) => {}
        }
    }
}

/// The rustls client configuration for `tls`: the broker must chain to `ca_pem`;
/// with a client certificate the device presents it (mTLS).
fn tls_client_config(tls: &MqttTls) -> Result<Arc<rustls::ClientConfig>, String> {
    use rustls::pki_types::pem::PemObject;
    use rustls::pki_types::{CertificateDer, PrivateKeyDer};

    let mut roots = rustls::RootCertStore::empty();
    for cert in CertificateDer::pem_slice_iter(tls.ca_pem) {
        let cert = cert.map_err(|e| format!("CA certificate: {e}"))?;
        roots
            .add(cert)
            .map_err(|e| format!("CA certificate: {e}"))?;
    }
    if roots.is_empty() {
        return Err("CA certificate: no certificate in the PEM".into());
    }
    let builder = rustls::ClientConfig::builder_with_provider(Arc::new(
        rustls::crypto::ring::default_provider(),
    ))
    .with_safe_default_protocol_versions()
    .map_err(|e| e.to_string())?
    .with_root_certificates(roots);
    let config = match &tls.client {
        Some(client) => {
            let chain = CertificateDer::pem_slice_iter(client.cert_pem)
                .collect::<Result<Vec<_>, _>>()
                .map_err(|e| format!("client certificate: {e}"))?;
            let key = PrivateKeyDer::from_pem_slice(client.key_pem)
                .map_err(|e| format!("client key: {e}"))?;
            builder
                .with_client_auth_cert(chain, key)
                .map_err(|e| format!("client certificate: {e}"))?
        }
        None => builder.with_no_client_auth(),
    };
    Ok(Arc::new(config))
}

/// Runs the TLS handshake on a non-blocking `stream`, verifying the broker's
/// certificate against `broker_host`.
async fn tls_handshake(
    config: &Arc<rustls::ClientConfig>,
    broker_host: &'static str,
    mut stream: std::net::TcpStream,
) -> Result<rustls::StreamOwned<rustls::ClientConnection, std::net::TcpStream>, String> {
    let name = rustls::pki_types::ServerName::try_from(broker_host)
        .map_err(|e| format!("broker host '{broker_host}': {e}"))?;
    let mut conn =
        rustls::ClientConnection::new(config.clone(), name).map_err(|e| e.to_string())?;
    let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_secs(30);
    while conn.is_handshaking() {
        match conn.complete_io(&mut stream) {
            Ok(_) => {}
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                if embassy_time::Instant::now() > deadline {
                    return Err("handshake timed out".into());
                }
                embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
            }
            Err(e) => return Err(e.to_string()),
        }
    }
    Ok(rustls::StreamOwned::new(conn, stream))
}

/// Public MQTT driver task. Opens a non-blocking TCP connection to the broker (with
/// TLS when `tls` is set), runs the session, and reconnects on any failure.
#[embassy_executor::task]
pub async fn mqtt_driver_task(
    broker_host: &'static str,
    broker_port: u16,
    client_id: &'static str,
    cmd_topic: &'static str,
    data_topic: &'static str,
    username: &'static str,
    password: &'static str,
    tls: Option<MqttTls>,
) {
    use embassy_time::{Duration, Timer};

    let tls_config = match tls.as_ref().map(tls_client_config).transpose() {
        Ok(config) => config,
        Err(e) => {
            // A bad certificate won't fix itself: stay offline rather than fall back
            // to plain TCP.
            log::error!("MQTT: TLS settings rejected ({e}); MQTT stays off");
            return;
        }
    };

    loop {
        let addr = format!("{}:{}", broker_host, broker_port);
        log::info!("MQTT: connecting to {}", addr);

        let stream = match std::net::TcpStream::connect(&addr) {
            Ok(s) => s,
            Err(e) => {
                log::warn!("MQTT: TCP connect failed: {}, retrying in 5s", e);
                Timer::after(Duration::from_secs(5)).await;
                continue;
            }
        };
        stream
            .set_nonblocking(true)
            .expect("set_nonblocking failed");

        let result = match &tls_config {
            None => {
                run_mqtt_session(
                    NonBlockingTcpStream(stream),
                    client_id,
                    cmd_topic,
                    data_topic,
                    username,
                    password,
                )
                .await
            }
            Some(config) => match tls_handshake(config, broker_host, stream).await {
                Ok(tls_stream) => {
                    log::info!("MQTT: TLS established");
                    run_mqtt_session(
                        NonBlockingTcpStream(tls_stream),
                        client_id,
                        cmd_topic,
                        data_topic,
                        username,
                        password,
                    )
                    .await
                }
                Err(e) => {
                    log::warn!("MQTT: TLS handshake failed: {e}");
                    Err(())
                }
            },
        };
        if result.is_err() {
            log::info!("MQTT: session ended, reconnecting in 5s");
        }
        // Drops what was queued for this session instead of sending it on the next.
        core_interface::set_link_up(Transport::Mqtt, false);
        Timer::after(Duration::from_secs(5)).await;
    }
}
