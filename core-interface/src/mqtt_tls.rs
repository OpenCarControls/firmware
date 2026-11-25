//! The TLS settings of the MQTT connection, shared by the boards' MQTT drivers.
//!
//! Today xtask compiles them into the image from `[network.mqtt]` (see
//! `docs/mqtt-tls.md`). The setup wizard will later write them to the device over
//! BLE; the drivers only see this type, so where the bytes come from can change
//! without touching them.

/// TLS for the MQTT connection: the broker is verified against `ca_pem`, and with
/// `client` set the device also presents its own certificate (mTLS).
#[derive(Clone, Copy)]
pub struct MqttTls {
    /// The CA certificate(s) the broker's certificate must chain to, in PEM.
    pub ca_pem: &'static [u8],
    /// The device's certificate and key, for mTLS. `None` when the broker only takes
    /// a username and password (sent inside TLS).
    pub client: Option<MqttClientCert>,
}

/// The device's own certificate and private key, both in PEM.
#[derive(Clone, Copy)]
pub struct MqttClientCert {
    /// The device's certificate, optionally followed by intermediate CAs.
    pub cert_pem: &'static [u8],
    /// The private key matching `cert_pem` (PKCS#8, SEC1 or PKCS#1), unencrypted.
    pub key_pem: &'static [u8],
}

impl core::fmt::Debug for MqttClientCert {
    // Never prints the key.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("MqttClientCert")
            .field("cert_pem", &self.cert_pem.len())
            .finish_non_exhaustive()
    }
}
