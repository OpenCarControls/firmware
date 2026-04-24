use core_interface::{MQTT_RX_CHANNEL, MQTT_TX_CHANNEL, proto};
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

/// Adapts a non-blocking `std::net::TcpStream` to `embedded_io_async`'s
/// `Read` and `Write` traits by polling every 1 ms via Embassy timer when the
/// socket would block. This mirrors the pattern used by `socket_can_task`.
struct NonBlockingTcpStream(std::net::TcpStream);

impl embedded_io_async::ErrorType for NonBlockingTcpStream {
    type Error = embedded_io_async::ErrorKind;
}

impl embedded_io_async::Read for NonBlockingTcpStream {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        use std::io::Read;
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

impl embedded_io_async::Write for NonBlockingTcpStream {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        use std::io::Write;
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
        use std::io::Write;
        self.0
            .flush()
            .map_err(|_| embedded_io_async::ErrorKind::Other)
    }
}

/// Shared MQTT session logic. Authenticates (if credentials provided),
/// subscribes, then drives inbound/outbound message exchange until an error
/// occurs, at which point it returns `Err(())` so the caller reconnects.
async fn run_mqtt_session(
    transport: NonBlockingTcpStream,
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

    let keepalive = Duration::from_secs(55);
    let mut last_ping = Instant::now();
    let data_topic_name = TopicName::new_unchecked(MqttString::from_str_unchecked(data_topic));

    loop {
        // Drain outbound channel before polling inbound
        while let Ok(msg) = MQTT_TX_CHANNEL.try_receive() {
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

/// Public MQTT driver task. Opens a non-blocking TCP connection to the broker,
/// runs the session, and reconnects on any failure.
#[embassy_executor::task]
pub async fn mqtt_driver_task(
    broker_host: &'static str,
    broker_port: u16,
    client_id: &'static str,
    cmd_topic: &'static str,
    data_topic: &'static str,
    username: &'static str,
    password: &'static str,
) {
    use embassy_time::{Duration, Timer};

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

        let transport = NonBlockingTcpStream(stream);

        if run_mqtt_session(
            transport, client_id, cmd_topic, data_topic, username, password,
        )
        .await
        .is_err()
        {
            log::warn!("MQTT: session ended, reconnecting in 5s");
        }
        Timer::after(Duration::from_secs(5)).await;
    }
}
