//! WiFi stack initialisation and MQTT driver for the ESP32 board.
//!
//! `init_wifi` brings up the ESP radio WiFi peripheral and creates the `embassy-net` stack,
//! spawning an internal connection-management task that reconnects automatically. The
//! LTE uplink (`modem::init_lte`) creates the same kind of stack over PPP instead.
//!
//! `mqtt_driver_task` connects to the MQTT broker over TCP on whichever stack it is
//! given (with TLS through MbedTLS for `mqtts://`, see `docs/mqtt-tls.md`), subscribes
//! to the command topic, and bridges the `MQTT_RX_CHANNEL` / `MQTT_TX_CHANNEL` channel
//! pair. It reconnects from scratch on any session failure.

#[cfg(feature = "hardware")]
mod hardware {
    use alloc::string::String;
    use alloc::vec::Vec;

    use core_interface::{
        MQTT_RX_CHANNEL, MQTT_STATE_TX, MQTT_TX_CHANNEL, MqttTls, Transport, proto,
    };
    use embassy_net::{Runner, Stack, StackResources, dns::DnsQueryType, tcp::TcpSocket};
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

    // 1 socket for the MQTT TCP connection + 1 for the SavvyCAN CAN logger + 1 for DNS
    // + 2 for smoltcp internals (DHCP, ARP).
    pub(crate) const NET_RESOURCES_SOCKETS: usize = 5;

    /// A network stack, over WiFi (`init_wifi`) or LTE (`init_lte`).
    pub type NetStack = Stack<'static>;

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
                    crate::status_leds::set_flag(crate::status_leds::FLAG_WIFI, true);
                    let _ = controller.wait_for_disconnect_async().await;
                    crate::status_leds::set_flag(crate::status_leds::FLAG_WIFI, false);
                    log::warn!("WiFi: disconnected, reconnecting in 5s");
                }
                Err(e) => {
                    crate::status_leds::set_flag(crate::status_leds::FLAG_WIFI, false);
                    log::warn!("WiFi: connect failed: {:?}, retrying in 5s", e);
                }
            }
            Timer::after(Duration::from_secs(5)).await;
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
    ) -> &'static NetStack {
        use static_cell::StaticCell;

        static RESOURCES: StaticCell<StackResources<NET_RESOURCES_SOCKETS>> = StaticCell::new();
        static STACK: StaticCell<NetStack> = StaticCell::new();

        let (controller, interfaces) =
            esp_radio::wifi::new(wifi_peri, WifiControllerConfig::default())
                .expect("WiFi init failed");

        let net_config = embassy_net::Config::dhcpv4(Default::default());
        // Seeds TCP sequence numbers and ephemeral ports: a fixed seed would make them
        // predictable after every boot.
        let mut seed = [0u8; 8];
        esp_hal::rng::Rng::new().read(&mut seed);
        let (stack, runner) = embassy_net::new(
            interfaces.station,
            net_config,
            RESOURCES.init(StackResources::new()),
            u64::from_le_bytes(seed),
        );
        let stack_ref = STACK.init(stack);

        spawner.spawn(net_task(runner).unwrap());
        spawner.spawn(wifi_connection_task(controller, ssid, password).unwrap());

        stack_ref
    }

    // ─── TLS ──────────────────────────────────────────────────────────────────

    /// TLS through MbedTLS, only in builds for a mqtts:// broker.
    #[cfg(feature = "mqtt-tls")]
    mod tls {
        use alloc::ffi::CString;
        use core::ffi::CStr;

        use core_interface::MqttTls;

        /// MbedTLS's random source: the hardware RNG, a true random source while the radio
        /// runs (BLE always does).
        struct TlsRng;

        impl rand_core_010::TryRng for TlsRng {
            type Error = core::convert::Infallible;
            fn try_next_u32(&mut self) -> Result<u32, Self::Error> {
                Ok(esp_hal::rng::Rng::new().random())
            }
            fn try_next_u64(&mut self) -> Result<u64, Self::Error> {
                let mut bytes = [0u8; 8];
                esp_hal::rng::Rng::new().read(&mut bytes);
                Ok(u64::from_le_bytes(bytes))
            }
            fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), Self::Error> {
                esp_hal::rng::Rng::new().read(dest);
                Ok(())
            }
        }

        impl rand_core_010::TryCryptoRng for TlsRng {}

        /// The parsed TLS settings, built once per boot.
        pub(super) struct TlsSetup {
            pub(super) tls: mbedtls_rs::Tls<'static>,
            pub(super) config: mbedtls_rs::ClientSessionConfig<'static>,
        }

        /// MbedTLS parses PEM from a NUL-terminated string.
        fn pem_cstr(pem: &[u8]) -> Option<CString> {
            let pem = pem.strip_suffix(&[0]).unwrap_or(pem);
            CString::new(pem).ok()
        }

        /// Parses the CA, the device certificate and key, and pins the broker's name for
        /// hostname verification.
        pub(super) fn tls_setup(
            tls: &MqttTls,
            broker_host: &'static str,
        ) -> Result<TlsSetup, &'static str> {
            use mbedtls_rs::{
                AuthMode, Certificate, ClientSessionConfig, Credentials, PrivateKey, X509,
            };
            use static_cell::StaticCell;

            static RNG: StaticCell<TlsRng> = StaticCell::new();

            let ca = pem_cstr(tls.ca_pem).ok_or("CA certificate isn't PEM text")?;
            let ca_chain = Certificate::new(X509::PEM(&ca)).map_err(|_| "CA certificate")?;
            let creds = match &tls.client {
                Some(client) => {
                    let cert =
                        pem_cstr(client.cert_pem).ok_or("client certificate isn't PEM text")?;
                    let key = pem_cstr(client.key_pem).ok_or("client key isn't PEM text")?;
                    Some(Credentials {
                        certificate: Certificate::new(X509::PEM(&cert))
                            .map_err(|_| "client certificate")?,
                        private_key: PrivateKey::new(X509::PEM(&key), None)
                            .map_err(|_| "client key")?,
                    })
                }
                None => None,
            };
            // Lives for the whole boot, like the session config that points to it.
            let server_name: &'static CStr = alloc::boxed::Box::leak(
                CString::new(broker_host)
                    .map_err(|_| "broker host")?
                    .into_boxed_c_str(),
            );
            let mut config = ClientSessionConfig::new();
            config.ca_chain = Some(ca_chain);
            config.creds = creds;
            config.server_name = Some(server_name);
            config.auth_mode = AuthMode::Required;
            let tls = mbedtls_rs::Tls::new(RNG.init(TlsRng)).map_err(|_| "TLS already started")?;
            Ok(TlsSetup { tls, config })
        }
    }

    /// Inner MQTT session logic. Authenticates, subscribes, then loops until an
    /// error occurs. Returns `Err(())` on any protocol or IO fault so the caller
    /// can reconnect.
    async fn run_mqtt_session<N: embedded_io_async::Read + embedded_io_async::Write>(
        network: N,
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
        core_interface::set_link_up(Transport::Mqtt, true);

        let keepalive = Duration::from_secs(55);
        let mut last_ping = Instant::now();
        let data_topic_name = TopicName::new_unchecked(MqttString::from_str_unchecked(data_topic));

        loop {
            // Responses and system state first, then the newest vehicle state.
            while let Some(msg) = MQTT_TX_CHANNEL
                .try_receive()
                .ok()
                .or_else(|| MQTT_STATE_TX.try_take())
            {
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

    /// Public MQTT driver task. Waits for the network, resolves the broker, opens a TCP connection to the
    /// broker, runs the session, and reconnects on any failure.
    #[embassy_executor::task]
    pub async fn mqtt_driver_task(
        stack: &'static NetStack,
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

        #[cfg(not(feature = "mqtt-tls"))]
        if tls.is_some() {
            log::error!("MQTT: built without TLS (the mqtt-tls feature); MQTT stays off");
            return;
        }
        #[cfg(feature = "mqtt-tls")]
        let tls = match tls.map(|t| tls::tls_setup(&t, broker_host)).transpose() {
            Ok(tls) => tls,
            Err(e) => {
                // A bad certificate won't fix itself: stay offline rather than fall
                // back to plain TCP.
                log::error!("MQTT: TLS settings rejected ({}); MQTT stays off", e);
                return;
            }
        };

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

            // Numeric addresses are returned as they are, without a query.
            let addr = match stack.dns_query(broker_host, DnsQueryType::A).await {
                Ok(addrs) if !addrs.is_empty() => addrs[0],
                Ok(_) | Err(_) => {
                    log::warn!("MQTT: can't resolve '{}', retrying in 10s", broker_host);
                    Timer::after(Duration::from_secs(10)).await;
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

            #[cfg(not(feature = "mqtt-tls"))]
            let result =
                run_mqtt_session(socket, client_id, cmd_topic, data_topic, username, password)
                    .await;
            #[cfg(feature = "mqtt-tls")]
            let result = match &tls {
                None => {
                    run_mqtt_session(socket, client_id, cmd_topic, data_topic, username, password)
                        .await
                }
                Some(setup) => {
                    match mbedtls_rs::Session::new(
                        setup.tls.reference(),
                        socket,
                        &mbedtls_rs::SessionConfig::Client(setup.config.clone()),
                    ) {
                        Err(e) => {
                            log::warn!("MQTT: TLS session setup failed: {:?}", e);
                            Err(())
                        }
                        Ok(mut session) => match session.connect().await {
                            Err(e) => {
                                log::warn!(
                                    "MQTT: TLS handshake failed: {:?} (verify flags {:#x})",
                                    e,
                                    session.tls_verification_details()
                                );
                                Err(())
                            }
                            Ok(()) => {
                                log::info!(
                                    "MQTT: TLS established, {} B heap free",
                                    esp_alloc::HEAP.free()
                                );
                                run_mqtt_session(
                                    session, client_id, cmd_topic, data_topic, username, password,
                                )
                                .await
                            }
                        },
                    }
                }
            };
            if result.is_err() {
                log::info!("MQTT: session ended, reconnecting in 5s");
            }
            // Drops what was queued for this session instead of sending it on the next.
            core_interface::set_link_up(Transport::Mqtt, false);
            Timer::after(Duration::from_secs(5)).await;
        }
    }

    /// SavvyCAN network logger: accepts one client at a time on `port` and streams the
    /// CAN buses to it until it disconnects. The protocol and session logic live in
    /// `core-interface`.
    #[embassy_executor::task]
    pub async fn can_logger_task(stack: &'static NetStack, port: u16, num_buses: u8) {
        use embassy_time::{Duration, Timer};

        let mut rx_buffer = [0u8; 1024];
        // The send buffer bounds the bytes in flight, so throughput is at most its size
        // per round trip. A busy bus needs ~42 KB/s, and WiFi round trips (with BLE
        // coexistence) reach ~100 ms: 2 KB lost three quarters of the frames.
        let mut tx_buffer = [0u8; 16 * 1024];

        loop {
            if !stack.is_config_up() {
                stack.wait_config_up().await;
            }

            let mut socket = TcpSocket::new(*stack, &mut rx_buffer, &mut tx_buffer);
            // Notice a SavvyCAN that vanished (laptop slept, left WiFi) even when the
            // bus is silent, so the buses are handed back to the vehicle.
            socket.set_timeout(Some(Duration::from_secs(30)));
            socket.set_keep_alive(Some(Duration::from_secs(10)));

            log::info!("CAN logger: waiting for SavvyCAN on port {}", port);
            if socket.accept(port).await.is_err() {
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
            log::info!("CAN logger: SavvyCAN connected");

            let end = core_interface::run_can_log_session(&mut socket, num_buses).await;
            log::info!(
                "CAN logger: SavvyCAN disconnected ({} frame(s) dropped, {} transmit request(s) ignored)",
                end.dropped,
                end.tx_requests_ignored
            );
            socket.abort();
            let _ = socket.flush().await;
        }
    }
}

#[cfg(feature = "hardware")]
pub use hardware::*;
