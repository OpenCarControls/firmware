//! WiFi stack initialisation and MQTT driver for the ESP32 board.
//!
//! `init_wifi` brings up the ESP radio WiFi peripheral and creates the `embassy-net` stack,
//! spawning an internal connection-management task that reconnects automatically.
//!
//! `mqtt_driver_task` connects to the MQTT broker over TCP, subscribes to the command
//! topic, and bridges the `MQTT_RX_CHANNEL` / `MQTT_TX_CHANNEL` channel pair.
//! It reconnects from scratch on any session failure.

#[cfg(feature = "hardware")]
mod hardware {
    use alloc::string::String;
    use alloc::vec::Vec;

    use core_interface::{MQTT_RX_CHANNEL, MQTT_TX_CHANNEL, proto};
    use embassy_net::{Runner, Stack, StackResources, tcp::TcpSocket};
    use esp_radio::wifi::sta::StationConfig;
    use esp_radio::wifi::{
        Config as WifiModeConfig, ControllerConfig as WifiControllerConfig,
        Interface as WifiInterface, WifiController,
    };
    use prost::Message as _;
    use rust_mqtt::{
        Bytes,
        buffer::AllocBuffer,
        client::Client,
        client::MqttError,
        client::event::Event,
        client::options::{
            ConnectOptions, PublicationOptions, SubscriptionOptions, TopicReference,
        },
        types::{MqttBinary, MqttString, TopicFilter, TopicName},
    };

    // 1 socket for the MQTT TCP connection + 2 for smoltcp internals (DHCP, ARP).
    const NET_RESOURCES_SOCKETS: usize = 3;

    /// The concrete WiFi stack type produced by `init_wifi`.
    pub type WifiStack = Stack<'static>;

    /// Internal task: drives the smoltcp network stack.
    #[embassy_executor::task]
    async fn net_task(mut runner: Runner<'static, WifiInterface<'static>>) {
        runner.run().await;
    }

    /// Internal task: manages WiFi association with automatic reconnect.
    #[embassy_executor::task]
    async fn wifi_connection_task(
        mut controller: WifiController<'static>,
        ssid: &'static str,
        password: &'static str,
    ) {
        use embassy_time::{Duration, Timer};

        controller
            .set_config(&WifiModeConfig::Station(
                StationConfig::default()
                    .with_ssid(String::from(ssid))
                    .with_password(String::from(password).into()),
            ))
            .unwrap();

        log::info!("WiFi: started");

        loop {
            log::info!("WiFi: connecting to '{}'", ssid);
            match controller.connect_async().await {
                Ok(_) => {
                    log::info!("WiFi: connected");
                    let _ = controller.wait_for_disconnect_async().await;
                    log::warn!("WiFi: disconnected, reconnecting in 5s");
                }
                Err(e) => {
                    log::warn!("WiFi: connect failed: {:?}, retrying in 5s", e);
                }
            }
            Timer::after(Duration::from_millis(5_000)).await;
        }
    }

    /// Initialises the WiFi peripheral, creates the `embassy-net` stack, and
    /// spawns the network runner and connection tasks. Returns a reference to
    /// the stack that can be passed to `mqtt_driver_task`.
    pub fn init_wifi(
        spawner: &embassy_executor::Spawner,
        wifi_peri: esp_hal::peripherals::WIFI<'static>,
        ssid: &'static str,
        password: &'static str,
    ) -> &'static WifiStack {
        use static_cell::StaticCell;

        static RESOURCES: StaticCell<StackResources<NET_RESOURCES_SOCKETS>> = StaticCell::new();
        static STACK: StaticCell<WifiStack> = StaticCell::new();

        let (controller, interfaces) =
            esp_radio::wifi::new(wifi_peri, WifiControllerConfig::default())
                .expect("WiFi init failed");

        let net_config = embassy_net::Config::dhcpv4(Default::default());
        // A fixed seed is acceptable here; embassy-net uses it only for its internal
        // PRNG (TCP ephemeral port selection, etc.) — not for cryptographic purposes.
        let seed: u64 = 0xDEAD_BEEF_CAFE_F00D;
        let (stack, runner) = embassy_net::new(
            interfaces.station,
            net_config,
            RESOURCES.init(StackResources::new()),
            seed,
        );
        let stack_ref = STACK.init(stack);

        spawner.spawn(net_task(runner).unwrap());
        spawner.spawn(wifi_connection_task(controller, ssid, password).unwrap());

        stack_ref
    }

    /// Inner MQTT session logic. Authenticates, subscribes, then loops until an
    /// error occurs. Returns `Err(())` on any protocol or IO fault so the caller
    /// can reconnect.
    async fn run_mqtt_session(
        network: TcpSocket<'_>,
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
                network,
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
            while let Ok(msg) = MQTT_TX_CHANNEL.try_receive() {
                let mut buf = Vec::<u8>::new();
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

            if last_ping.elapsed() >= keepalive {
                client.ping().await.map_err(|e| {
                    log::warn!("MQTT: ping failed: {:?}", e);
                })?;
                last_ping = Instant::now();
            }

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

    /// Public MQTT driver task. Waits for DHCP, opens a TCP connection to the
    /// broker, runs the session, and reconnects on any failure.
    #[embassy_executor::task]
    pub async fn mqtt_driver_task(
        stack: &'static WifiStack,
        broker_host: &'static str,
        broker_port: u16,
        client_id: &'static str,
        cmd_topic: &'static str,
        data_topic: &'static str,
        username: &'static str,
        password: &'static str,
    ) {
        use embassy_time::{Duration, Timer};

        const TCP_RX: usize = 4096;
        const TCP_TX: usize = 4096;

        let mut rx_buffer = [0u8; TCP_RX];
        let mut tx_buffer = [0u8; TCP_TX];

        loop {
            if !stack.is_config_up() {
                log::debug!("MQTT: waiting for network...");
                stack.wait_config_up().await;
            }

            let mut socket = TcpSocket::new(*stack, &mut rx_buffer, &mut tx_buffer);
            socket.set_timeout(Some(embassy_time::Duration::from_secs(30)));

            let addr: embassy_net::IpAddress = match broker_host.parse() {
                Ok(ip) => ip,
                Err(_) => {
                    log::error!(
                        "MQTT: broker_host '{}' is not a numeric IP - DNS not yet supported",
                        broker_host
                    );
                    Timer::after(Duration::from_secs(30)).await;
                    continue;
                }
            };
            let remote = (addr, broker_port);

            log::info!("MQTT: connecting to {}:{}", broker_host, broker_port);
            if socket.connect(remote).await.is_err() {
                log::warn!("MQTT: TCP connect failed, retrying in 5s");
                Timer::after(Duration::from_secs(5)).await;
                continue;
            }

            if run_mqtt_session(socket, client_id, cmd_topic, data_topic, username, password)
                .await
                .is_err()
            {
                log::info!("MQTT: session ended, reconnecting in 5s");
            }
            Timer::after(Duration::from_secs(5)).await;
        }
    }
}

#[cfg(feature = "hardware")]
pub use hardware::*;
