use alloc::string::String;
use alloc::vec::Vec;
use portable_atomic::Ordering;

use crate::channels::{
    ADVANCED_CMD_CHANNEL, BASIC_CMD_CHANNEL, BLE_RX_CHANNEL, MQTT_RX_CHANNEL,
    SYSTEM_COMMAND_CHANNEL,
};
use crate::dedupe::is_duplicate_command;
use crate::proto;
use crate::routing::{send_to, try_send_to};
use crate::types::{InboundCommand, Transport};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;

static BLE_CONTROLLER_LEASE_TTL_S: portable_atomic::AtomicU32 = portable_atomic::AtomicU32::new(15);

#[derive(Default)]
struct BleControllerLease {
    owner_device_id: Vec<u8>,
    expires_at_ms: u64,
}

static BLE_CONTROLLER_LEASE: Mutex<CriticalSectionRawMutex, BleControllerLease> =
    Mutex::new(BleControllerLease {
        owner_device_id: Vec::new(),
        expires_at_ms: 0,
    });

pub fn set_ble_controller_lease_ttl_s(seconds: u32) {
    if seconds > 0 {
        BLE_CONTROLLER_LEASE_TTL_S.store(seconds, Ordering::Relaxed);
    }
}

async fn ble_controller_allows(source_device_id: &[u8], now_ms: u64) -> bool {
    if source_device_id.is_empty() {
        return false;
    }
    let ttl_ms = (BLE_CONTROLLER_LEASE_TTL_S.load(Ordering::Relaxed) as u64) * 1_000;
    let mut lease = BLE_CONTROLLER_LEASE.lock().await;
    if lease.owner_device_id.is_empty()
        || now_ms >= lease.expires_at_ms
        || lease.owner_device_id.as_slice() == source_device_id
    {
        lease.owner_device_id.clear();
        lease.owner_device_id.extend_from_slice(source_device_id);
        lease.expires_at_ms = now_ms.saturating_add(ttl_ms);
        return true;
    }
    false
}

async fn send_not_controller_response(message_id: u64) {
    send_to(
        Transport::Ble,
        proto::DeviceToApp {
            timestamp_ms: embassy_time::Instant::now().as_millis(),
            platform_id: crate::PLATFORM_ID.load(Ordering::Relaxed),
            payload: Some(proto::device_to_app::Payload::CommandResponse(
                proto::CommandResponse {
                    message_id,
                    success: false,
                    error_message: String::from(
                        "command rejected: active controller lease held by another phone",
                    ),
                    response_data: None,
                    status_code: proto::CommandStatusCode::RejectedNotController as i32,
                },
            )),
        },
    )
    .await;
}

/// Why a system command was refused: the status code, the reason shown to the user, and
/// data that explains it (a validation session with no attempts left, say).
struct Refusal {
    status: proto::CommandStatusCode,
    message: &'static str,
    result: Option<proto::system_command_result::Result>,
}

impl Refusal {
    fn failed(message: &'static str) -> Self {
        Self {
            status: proto::CommandStatusCode::Failed,
            message,
            result: None,
        }
    }

    fn not_validated(message: &'static str) -> Self {
        Self {
            status: proto::CommandStatusCode::RejectedNotValidated,
            message,
            result: None,
        }
    }
}

/// What a system command handled here answers: its data on success.
type SystemAnswer = Result<Option<proto::system_command_result::Result>, Refusal>;

/// Answers a system command handled here (the CAN read-write policy, phone trust).
async fn send_system_command_response(message_id: u64, answer: SystemAnswer) {
    let (success, error_message, status_code, result) = match answer {
        Ok(result) => (true, String::new(), proto::CommandStatusCode::Ok, result),
        Err(refusal) => (
            false,
            String::from(refusal.message),
            refusal.status,
            refusal.result,
        ),
    };
    send_to(
        Transport::Ble,
        proto::DeviceToApp {
            timestamp_ms: embassy_time::Instant::now().as_millis(),
            platform_id: crate::PLATFORM_ID.load(Ordering::Relaxed),
            payload: Some(proto::device_to_app::Payload::CommandResponse(
                proto::CommandResponse {
                    message_id,
                    success,
                    error_message,
                    response_data: result.map(|result| {
                        proto::command_response::ResponseData::SystemResult(
                            proto::SystemCommandResult {
                                result: Some(result),
                            },
                        )
                    }),
                    status_code: status_code as i32,
                },
            )),
        },
    )
    .await;
}

/// The CAN read-write policy commands change whether the device may transmit on the
/// car's bus, so only a bonded phone may send them. `source_device_id` is the one the
/// transport stamped, never the app's: on ESP, a phone that isn't bonded can still write
/// while the pairing window is open.
const NOT_BONDED: &str = "command rejected: only a bonded phone can change CAN read-write mode";

/// Once a phone is validated for this car, a merely bonded phone can't change the approval:
/// it could otherwise lock the owner out, or approve a car it was never validated in.
const APPROVAL_NOT_VALIDATED: &str =
    "command rejected: only a phone validated in this car can change CAN read-write mode";

async fn handle_can_read_write_approval(source_device_id: &[u8], approved: bool) -> SystemAnswer {
    if !crate::pairing::is_phone_paired(source_device_id).await {
        return Err(Refusal::failed(NOT_BONDED));
    }
    if crate::phone_trust::car_has_validated_phone()
        && !crate::phone_trust::is_phone_validated(source_device_id)
    {
        return Err(Refusal::not_validated(APPROVAL_NOT_VALIDATED));
    }
    if approved {
        crate::can_access::approve_can_read_write().map_err(|e| Refusal::failed(e.message()))?;
    } else {
        crate::can_access::revoke_can_read_write_approval();
    }
    Ok(None)
}

async fn handle_can_dev_write_unlock(source_device_id: &[u8], unlocked: bool) -> SystemAnswer {
    if !crate::pairing::is_phone_paired(source_device_id).await {
        return Err(Refusal::failed(NOT_BONDED));
    }
    crate::can_access::set_can_dev_write_unlocked(unlocked)
        .map_err(|e| Refusal::failed(e.message()))?;
    Ok(None)
}

// ── Phone trust ───────────────────────────────────────────────────────────────

const VALIDATION_NOT_BONDED: &str = "command rejected: only a bonded phone can be validated";

/// A merely bonded phone can't remove a validated one, or every phone once one is
/// validated: it could lock the owner out. `None` when removing is allowed (the board
/// still needs the pairing window open).
fn phone_removal_refusal(source_device_id: &[u8], target: Option<&[u8]>) -> Option<Refusal> {
    let protected = match target {
        Some(device_id) => crate::phone_trust::is_phone_validated(device_id),
        None => crate::phone_trust::car_has_validated_phone(),
    };
    (protected && !crate::phone_trust::is_phone_validated(source_device_id)).then(|| {
        Refusal::not_validated(
            "command rejected: only a phone validated in this car can remove a validated phone",
        )
    })
}

fn paired_phones_answer(source_device_id: &[u8], phones: Vec<Vec<u8>>) -> SystemAnswer {
    Ok(Some(proto::system_command_result::Result::PairedPhones(
        proto::PairedPhones {
            phones: phones
                .into_iter()
                .map(|device_id| proto::PairedPhone {
                    trust: crate::phone_trust::phone_trust_level(&device_id) as i32,
                    is_requester: device_id.as_slice() == source_device_id,
                    device_id,
                })
                .collect(),
        },
    )))
}

fn validation_refusal((error, session): crate::validation::Refused) -> Refusal {
    Refusal {
        status: proto::CommandStatusCode::Failed,
        message: error.message(),
        result: session.map(proto::system_command_result::Result::ValidationSession),
    }
}

/// The validation commands, from a bonded phone only (`source_device_id` is the stamped
/// one). The session belongs to the sending phone.
async fn handle_phone_validation(
    source_device_id: &[u8],
    action: proto::system_command::Action,
) -> SystemAnswer {
    use proto::system_command::Action;
    use proto::system_command_result::Result as Data;

    if !crate::pairing::is_phone_paired(source_device_id).await {
        return Err(Refusal::failed(VALIDATION_NOT_BONDED));
    }
    let now_ms = embassy_time::Instant::now().as_millis();
    match action {
        Action::ListPhoneValidationMethods(_) => Ok(Some(Data::ValidationMethods(
            crate::validation::validation_methods(),
        ))),
        Action::StartPhoneValidation(cmd) => {
            crate::validation::start_validation(source_device_id, &cmd.method_id, now_ms)
                .map(|session| Some(Data::ValidationSession(session)))
                .map_err(validation_refusal)
        }
        Action::SubmitPhoneValidationCode(cmd) => {
            crate::validation::submit_validation_code(source_device_id, &cmd.code, now_ms)
                .map(|session| Some(Data::ValidationSession(session)))
                .map_err(validation_refusal)
        }
        Action::CancelPhoneValidation(_) => {
            crate::validation::cancel_validation(source_device_id, now_ms)
                .map(|()| None)
                .map_err(|e| Refusal::failed(e.message()))
        }
        _ => Ok(None),
    }
}

/// Vehicle commands act on the car, so they are refused while the device must not
/// transmit (the CAN read-write policy, see `can_access.rs`), with the reason. Refusing
/// them here means every vehicle, the simulator included, reports a read-only device
/// truthfully instead of claiming a command it never sent worked. System commands
/// (approval, dev lock, pairing) are not gated.
fn read_only_rejection(message_id: u64) -> Option<proto::DeviceToApp> {
    let reason = crate::can_access::can_read_only_reason()?;
    Some(proto::DeviceToApp {
        timestamp_ms: embassy_time::Instant::now().as_millis(),
        platform_id: crate::PLATFORM_ID.load(Ordering::Relaxed),
        payload: Some(proto::device_to_app::Payload::CommandResponse(
            proto::CommandResponse {
                message_id,
                success: false,
                error_message: alloc::format!(
                    "command rejected: the device is read-only: {reason}"
                ),
                response_data: None,
                status_code: proto::CommandStatusCode::RejectedUnavailable as i32,
            },
        )),
    })
}

/// Vehicle commands also need the sending phone to be validated for this car (see
/// `phone_trust.rs`): a phone that only bonded during a pairing window can't act on the car.
fn not_validated_rejection(message_id: u64) -> proto::DeviceToApp {
    proto::DeviceToApp {
        timestamp_ms: embassy_time::Instant::now().as_millis(),
        platform_id: crate::PLATFORM_ID.load(Ordering::Relaxed),
        payload: Some(proto::device_to_app::Payload::CommandResponse(
            proto::CommandResponse {
                message_id,
                success: false,
                error_message: String::from(
                    "command rejected: this phone is not validated in this car",
                ),
                response_data: None,
                status_code: proto::CommandStatusCode::RejectedNotValidated as i32,
            },
        )),
    }
}

/// The checks a BLE vehicle command passes before it reaches the vehicle, in order: the
/// device is read-write, the phone is validated, the phone holds the controller lease.
/// Returns the command's authorization, or the rejection to answer with.
async fn admit_ble_vehicle_command(
    message_id: u64,
    source_device_id: &[u8],
) -> Result<crate::can_tx::CommandAuthorization, Option<alloc::boxed::Box<proto::DeviceToApp>>> {
    if let Some(rejection) = read_only_rejection(message_id) {
        return Err(Some(alloc::boxed::Box::new(rejection)));
    }
    if !crate::phone_trust::is_phone_validated(source_device_id) {
        return Err(Some(alloc::boxed::Box::new(not_validated_rejection(
            message_id,
        ))));
    }
    if !ble_controller_allows(source_device_id, embassy_time::Instant::now().as_millis()).await {
        // Answered by `send_not_controller_response`.
        return Err(None);
    }
    Ok(crate::can_tx::CommandAuthorization::new(
        source_device_id.to_vec(),
    ))
}

pub async fn reset_ble_controller_lease_for_tests() {
    let mut lease = BLE_CONTROLLER_LEASE.lock().await;
    lease.owner_device_id.clear();
    lease.expires_at_ms = 0;
}

// ── BLE Dispatcher ────────────────────────────────────────────────────────────

/// Processes a single `AppToDevice` message arriving from the BLE driver.
/// Validates `platform_id` and routes each payload variant to the correct channel:
/// - `system_command`:
///   - `restart_command` → `SYSTEM_COMMAND_CHANNEL` (board handles it)
///   - `set_can_debug_enabled` / `update_can_debug_filters` → dropped (deprecated: the
///     BLE CAN debug stream is gone, CAN logging goes to SavvyCAN instead)
///   - `set_can_read_write_approval` / `set_can_dev_write_unlock` → handled here (the CAN
///     read-write policy), from a bonded phone only (a validated one for the approval once
///     a phone is validated for this car), and answered with a `CommandResponse`
///   - `list_paired_phones` and the phone validation commands → handled here and answered
///     with a `CommandResponse` carrying a `SystemCommandResult`
///   - `remove_paired_phone` / `clear_paired_phones` → `SYSTEM_COMMAND_CHANNEL`, unless
///     they would remove a validated phone and the sender isn't validated (answered with a
///     `REJECTED_NOT_VALIDATED` response)
/// - `basic_command_bytes` → `BASIC_CMD_CHANNEL` (Transport::Ble)
/// - `advanced_command_bytes` → `ADVANCED_CMD_CHANNEL` (Transport::Ble)
///
/// Basic and advanced commands are answered with a failed `CommandResponse` instead,
/// giving the reason, while the device is CAN read-only or the phone isn't validated for
/// this car (`REJECTED_NOT_VALIDATED`). Those that pass carry a `CommandAuthorization`.
///
/// Messages with a mismatched `platform_id` are silently dropped, and so are commands
/// whose (source, `message_id`) pair was seen recently on either transport (see
/// `dedupe.rs`).
///
/// Responses are only queued while the BLE link is up (see `set_link_up`).
pub async fn handle_ble_message(msg: proto::AppToDevice) {
    if msg.platform_id != crate::PLATFORM_ID.load(Ordering::Relaxed) {
        return;
    }
    let source_device_id = msg.source_device_id.clone();
    let is_command = !matches!(
        msg.payload,
        None | Some(proto::app_to_device::Payload::Heartbeat(_))
    );
    if is_command && is_duplicate_command(&source_device_id, msg.message_id) {
        return;
    }
    match msg.payload {
        Some(proto::app_to_device::Payload::SystemCommand(cmd)) => match cmd.action {
            Some(proto::system_command::Action::RestartCommand(restart)) => {
                SYSTEM_COMMAND_CHANNEL
                    .sender()
                    .send(proto::SystemCommand {
                        action: Some(proto::system_command::Action::RestartCommand(restart)),
                    })
                    .await;
            }
            #[allow(deprecated)]
            Some(proto::system_command::Action::SetCanDebugEnabled(_))
            | Some(proto::system_command::Action::UpdateCanDebugFilters(_)) => {}
            Some(proto::system_command::Action::ListPairedPhones(_)) => {
                let phones = crate::pairing::list_paired_phones().await;
                let answer = paired_phones_answer(&source_device_id, phones);
                send_system_command_response(msg.message_id, answer).await;
            }
            Some(proto::system_command::Action::RemovePairedPhone(cmd)) => {
                if let Some(refusal) =
                    phone_removal_refusal(&source_device_id, Some(&cmd.device_id))
                {
                    send_system_command_response(msg.message_id, Err(refusal)).await;
                    return;
                }
                SYSTEM_COMMAND_CHANNEL
                    .sender()
                    .send(proto::SystemCommand {
                        action: Some(proto::system_command::Action::RemovePairedPhone(cmd)),
                    })
                    .await;
            }
            Some(proto::system_command::Action::ClearPairedPhones(cmd)) => {
                if let Some(refusal) = phone_removal_refusal(&source_device_id, None) {
                    send_system_command_response(msg.message_id, Err(refusal)).await;
                    return;
                }
                SYSTEM_COMMAND_CHANNEL
                    .sender()
                    .send(proto::SystemCommand {
                        action: Some(proto::system_command::Action::ClearPairedPhones(cmd)),
                    })
                    .await;
            }
            Some(proto::system_command::Action::UpsertPairedPhone(cmd)) => {
                SYSTEM_COMMAND_CHANNEL
                    .sender()
                    .send(proto::SystemCommand {
                        action: Some(proto::system_command::Action::UpsertPairedPhone(cmd)),
                    })
                    .await;
            }
            Some(proto::system_command::Action::SetCanReadWriteApproval(cmd)) => {
                let result = handle_can_read_write_approval(&source_device_id, cmd.approved).await;
                send_system_command_response(msg.message_id, result).await;
            }
            Some(proto::system_command::Action::SetCanDevWriteUnlock(cmd)) => {
                let result = handle_can_dev_write_unlock(&source_device_id, cmd.unlocked).await;
                send_system_command_response(msg.message_id, result).await;
            }
            Some(
                action @ (proto::system_command::Action::ListPhoneValidationMethods(_)
                | proto::system_command::Action::StartPhoneValidation(_)
                | proto::system_command::Action::SubmitPhoneValidationCode(_)
                | proto::system_command::Action::CancelPhoneValidation(_)),
            ) => {
                let result = handle_phone_validation(&source_device_id, action).await;
                send_system_command_response(msg.message_id, result).await;
            }
            None => {}
        },
        Some(proto::app_to_device::Payload::BasicCommandBytes(bytes)) => {
            let authorization =
                match admit_ble_vehicle_command(msg.message_id, &source_device_id).await {
                    Ok(authorization) => authorization,
                    Err(Some(rejection)) => {
                        send_to(Transport::Ble, *rejection).await;
                        return;
                    }
                    Err(None) => {
                        send_not_controller_response(msg.message_id).await;
                        return;
                    }
                };
            BASIC_CMD_CHANNEL
                .sender()
                .send(InboundCommand {
                    message_id: msg.message_id,
                    transport: Transport::Ble,
                    source_device_id,
                    bytes,
                    authorization: Some(authorization),
                })
                .await;
        }
        Some(proto::app_to_device::Payload::AdvancedCommandBytes(bytes)) => {
            let authorization =
                match admit_ble_vehicle_command(msg.message_id, &source_device_id).await {
                    Ok(authorization) => authorization,
                    Err(Some(rejection)) => {
                        send_to(Transport::Ble, *rejection).await;
                        return;
                    }
                    Err(None) => {
                        send_not_controller_response(msg.message_id).await;
                        return;
                    }
                };
            ADVANCED_CMD_CHANNEL
                .sender()
                .send(InboundCommand {
                    message_id: msg.message_id,
                    transport: Transport::Ble,
                    source_device_id,
                    bytes,
                    authorization: Some(authorization),
                })
                .await;
        }
        // Heartbeat is an MQTT-only concept — silently ignored on BLE.
        Some(proto::app_to_device::Payload::Heartbeat(_)) => {}
        None => {}
    }
}

#[embassy_executor::task]
pub async fn process_ble_commands_task() {
    let receiver = BLE_RX_CHANNEL.receiver();
    loop {
        handle_ble_message(receiver.receive().await).await;
    }
}

// ── MQTT Dispatcher ───────────────────────────────────────────────────────────

/// Whether commands arriving over MQTT are authenticated. They are not: anyone
/// with the broker credentials (which ship in the app) could publish one, or
/// replay a captured one. Until they are (contracts#8, firmware#16), basic
/// commands from MQTT are rejected. Remove the gate together with this flag.
pub(crate) const MQTT_COMMANDS_AUTHENTICATED: bool = false;

/// Answers a rejected MQTT command. `try_send`: the MQTT driver must never be
/// stalled by a full queue, and the app can retry or notice the missing reply.
fn send_mqtt_unauthenticated_response(message_id: u64) {
    try_send_to(
        Transport::Mqtt,
        proto::DeviceToApp {
            timestamp_ms: embassy_time::Instant::now().as_millis(),
            platform_id: crate::PLATFORM_ID.load(Ordering::Relaxed),
            payload: Some(proto::device_to_app::Payload::CommandResponse(
                proto::CommandResponse {
                    message_id,
                    success: false,
                    error_message: String::from(
                        "command rejected: commands over MQTT are not authenticated yet, use BLE",
                    ),
                    response_data: None,
                    status_code: proto::CommandStatusCode::Failed as i32,
                },
            )),
        },
    );
}

/// Processes a single `AppToDevice` message arriving from the MQTT driver.
/// Validates `platform_id` and routes `basic_command_bytes` →
/// `BASIC_CMD_CHANNEL` (Transport::Mqtt) once MQTT commands are authenticated;
/// until then they are answered with a failed `CommandResponse` instead.
/// `SystemCommand` and `advanced_command_bytes` are silently dropped — both
/// are restricted to BLE only. That includes the CAN read-write approval and the dev
/// lock: MQTT can never approve or unlock read-write mode.
/// `heartbeat` carries no payload and is not routed; its presence alone resets
/// the MQTT activity window so state updates continue at full rate.
/// A basic command whose `message_id` was seen recently from MQTT (a QoS 1
/// redelivery) is dropped without an answer; see `dedupe.rs`.
pub async fn handle_mqtt_message(msg: proto::AppToDevice) {
    if msg.platform_id != crate::PLATFORM_ID.load(Ordering::Relaxed) {
        return;
    }
    // Any valid inbound MQTT message (command or app heartbeat) keeps the
    // activity window alive so state updates flow at full rate.
    crate::routing::record_mqtt_activity((embassy_time::Instant::now().as_millis() / 1000) as u32);
    if let Some(proto::app_to_device::Payload::BasicCommandBytes(bytes)) = msg.payload {
        // No verified source over MQTT yet, so the key is the message ID alone.
        if is_duplicate_command(&[], msg.message_id) {
            return;
        }
        if !MQTT_COMMANDS_AUTHENTICATED {
            send_mqtt_unauthenticated_response(msg.message_id);
            return;
        }
        if let Some(rejection) = read_only_rejection(msg.message_id) {
            try_send_to(Transport::Mqtt, rejection);
            return;
        }
        BASIC_CMD_CHANNEL
            .sender()
            .send(InboundCommand {
                message_id: msg.message_id,
                transport: Transport::Mqtt,
                source_device_id: Vec::new(),
                bytes,
                // MQTT has no phone identity, so it can never send a command's frames.
                authorization: None,
            })
            .await;
    }
}

#[embassy_executor::task]
pub async fn process_mqtt_commands_task() {
    let receiver = MQTT_RX_CHANNEL.receiver();
    loop {
        handle_mqtt_message(receiver.receive().await).await;
    }
}
