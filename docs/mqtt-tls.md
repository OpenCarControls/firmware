# MQTT over TLS (mTLS)

Once the device uses LTE, the broker is on the public internet. With a `mqtts://` broker URL the device talks to it over TLS: it checks that the broker's certificate chains to your CA and names the broker's host, so nobody between the car and the broker can read the vehicle state or pose as the broker. With `auth_mode = "mtls"` the device also presents its own certificate, and the broker lets it in on that alone (no password in the image). The app does the same with a certificate of its own (see the app's `docs/mqtt-tls.md`).

TLS protects the link to the broker, not the commands themselves: anyone the broker lets publish to a car's command topic could still send one. MQTT commands therefore stay refused (`MQTT_COMMANDS_AUTHENTICATED` in `dispatch.rs`) until they are signed per phone (contracts#8, firmware#16).

## Configuration

`[network.mqtt]` in `config.toml`:

```toml
[network.mqtt]
broker_url = "mqtts://broker.example.com"   # port 8883 unless given
client_id  = "my-car"                       # also the topics' {client_id}
auth_mode  = "mtls"

ca_cert_file     = "certs/ca.pem"           # the CA the broker's certificate chains to
client_cert_file = "certs/my-car.pem"       # this device's certificate
client_key_file  = "certs/my-car.key"       # its private key, unencrypted
```

| `broker_url` | `auth_mode` | What the device does |
|---|---|---|
| `mqtt://` | `basic` | Plain TCP with the username and password. xtask warns: use it only on a network you trust. |
| `mqtts://` | `basic` | TLS, verifies the broker, then sends the username and password inside TLS. |
| `mqtts://` | `mtls` | TLS, verifies the broker, presents its certificate. `username` and `password` are ignored and not compiled in. |

xtask refuses `mtls` with a `mqtt://` URL, an unknown `auth_mode`, and a `mqtts://` URL without `ca_cert_file`, and checks that every file it needs exists. Paths are relative to the firmware directory (`certs/` is gitignored) or absolute. All files are PEM; the CA file and the certificate file may hold a chain.

Use the broker's DNS name in `broker_url`, the name its certificate is issued for. The device resolves it over DNS, on WiFi and on LTE.

## Where the certificates live

For now xtask compiles the three PEM files into the image (`MQTT_TLS` in the generated `main.rs`, a `core_interface::MqttTls`). Anyone who can read the device's flash can read the key, so give each device its own certificate, and revoke it at the broker (by removing its ACL entry) if a device is lost.

The setup wizard will later write them to the device over BLE instead of compiling them in. The MQTT drivers only take a `MqttTls`, so that change stays outside them. The wizard will need:

*   A flash region for the credentials, next to the bond and phone stores (see [BLE](ble.md)), read at boot into a `MqttTls` before `mqtt_driver_task` is spawned.
*   Ideally a key generated on the device, never leaving it: the wizard asks for a CSR, has it signed, and sends back the certificate and the CA. That needs MbedTLS's key generation and CSR writing (both are in the prebuilt libraries).
*   A system command to set or replace them, BLE-only and limited to a validated phone in the pairing window, like the other system commands.

## Making certificates

Use your own CA, not a public one: the broker should trust only certificates you issued. ECDSA P-256 keys are much faster than RSA on the ESP. With OpenSSL:

```bash
mkdir -p certs && cd certs

# The CA. Keep ca.key offline: whoever has it can mint certificates for your broker.
openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 -nodes \
  -keyout ca.key -out ca.pem -days 3650 -subj "/CN=Open Car Controls CA" \
  -addext "basicConstraints=critical,CA:TRUE" -addext "keyUsage=critical,keyCertSign,cRLSign"

# The broker: its DNS name in the subjectAltName.
openssl req -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 -nodes \
  -keyout broker.key -out broker.csr -subj "/CN=broker.example.com"
openssl x509 -req -in broker.csr -CA ca.pem -CAkey ca.key -CAcreateserial -days 825 \
  -extfile <(printf "subjectAltName=DNS:broker.example.com\nextendedKeyUsage=serverAuth") \
  -out broker.pem

# One per device: the CN is the device's client_id, which the broker uses as its username.
openssl req -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 -nodes \
  -keyout my-car.key -out my-car.csr -subj "/CN=my-car"
openssl x509 -req -in my-car.csr -CA ca.pem -CAkey ca.key -CAcreateserial -days 825 \
  -extfile <(printf "extendedKeyUsage=clientAuth") -out my-car.pem
```

Make one for each phone the same way (CN `phone-<name>`, see the app's docs).

## Broker setup

The broker must require a client certificate signed by your CA and map each certificate to a user. With Mosquitto 2.x (`mosquitto.conf`):

```conf
listener 8883
cafile   /etc/mosquitto/certs/ca.pem
certfile /etc/mosquitto/certs/broker.pem
keyfile  /etc/mosquitto/certs/broker.key
tls_version tlsv1.2
require_certificate true
use_identity_as_username true
allow_anonymous false
acl_file /etc/mosquitto/acl
```

`use_identity_as_username` makes the certificate's CN the username, so the ACL (`/etc/mosquitto/acl`) can keep each device to its own topics and give each phone only the cars it drives:

```conf
# Every device: only its own command and data topics (CN = client_id).
pattern read  opencar/%u/cmd
pattern write opencar/%u/data

# Each phone: the cars it may reach.
user phone-michael
topic write opencar/my-car/cmd
topic read  opencar/my-car/data
```

Close port 1883, or keep it on localhost only. Other brokers (EMQX, HiveMQ, a cloud IoT broker) have the same settings under other names: verify client certificates against the CA, take the username from the certificate's CN, and an ACL per user.

To check the broker from a computer:

```bash
mosquitto_sub -h broker.example.com -p 8883 --cafile certs/ca.pem \
  --cert certs/phone-michael.pem --key certs/phone-michael.key -t 'opencar/my-car/data' -v
```

## On the device

*   **ESP:** MbedTLS through `mbedtls-rs` 0.2, the last release on esp-hal 1.1, from its prebuilt libraries with software crypto. It is behind `board-esp`'s `mqtt-tls` feature, which xtask turns on only for a `mqtts://` URL: it adds about 540 KB of flash. The handshake allocates its buffers from the heap (two 16 KB records plus the handshake state); the device logs the free heap once the handshake succeeds (`MQTT: TLS established, … B heap free`). TLS 1.2 and 1.3 are offered.
*   **PC:** rustls with ring, always built in.
*   Both log a failed handshake with its reason (on the ESP, MbedTLS's verify flags) and retry like any other failed connection. Bad TLS settings (a PEM that doesn't parse) stop the MQTT driver at boot instead of falling back to plain TCP.
*   The TCP and TLS stacks get their randomness from the hardware RNG, which is a true random source while the radio runs (BLE always does).

## Limits

*   **Certificate dates aren't checked on the ESP.** It has no clock at boot, and the prebuilt MbedTLS is built without date checks, so an expired or not-yet-valid certificate is accepted (chain, signature and hostname are still checked). Revoke a certificate at the broker (its ACL entry), not by letting it expire. Checking dates needs the time from the network (LTE `AT+CCLK?` or SNTP) and MbedTLS built from source with its `hook-wall-clock` feature. The PC checks dates.
*   **No hardware crypto on the ESP:** MbedTLS's hardware hooks need `mbedtls-rs` on esp-hal 1.2, held back until esp-radio supports it. A handshake takes noticeably longer than on a PC; it happens once per connection.
*   **No session resumption:** each reconnect does a full handshake.
*   **Flash size:** the prebuilt libraries carry features the device doesn't use (DTLS, a TLS server, X.509 writing). A from-source build with only the client parts would be much smaller, but needs a C cross-compiler in the build.
