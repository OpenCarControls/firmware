//! The two classes of CAN frames a vehicle sends (see docs/can.md, "TX classes").
//!
//! - **Platform** frames are the platform's own traffic that doesn't act on the car: the
//!   identity request, text on the cluster, other display frames. They need read-write
//!   mode (the car identified and approved, the dev lock unlocked), or an open identity
//!   probe window for the probe's exact frames.
//! - **Command** frames are part of a vehicle command (lock the doors, open the trunk).
//!   They also need the phone that sent the command to be validated for this car, which
//!   the board's TX check ([`crate::allow_can_tx`]) reads again when the frame is sent.
//!
//! A frame can only reach a TX channel as a [`CanTxFrame`], built with
//! [`CanTxFrame::platform`] or [`CanTxFrame::command`]. A command frame needs a
//! [`CommandAuthorization`], which only the dispatcher creates, for a command from a
//! validated phone, and hands to the vehicle in [`crate::InboundCommand`]. So the class of
//! every frame is visible where it is built: review every `CanTxFrame::platform` call to
//! check that the frame doesn't act on the car.

use alloc::vec::Vec;

use crate::types::CanFrame;

/// Proof that a vehicle command came from a phone validated for this car. Created by the
/// dispatcher only, and carried by the command's [`crate::InboundCommand`].
#[derive(Debug, PartialEq, Eq)]
pub struct CommandAuthorization {
    device_id: Vec<u8>,
}

impl CommandAuthorization {
    pub(crate) fn new(device_id: Vec<u8>) -> Self {
        Self { device_id }
    }

    /// For vehicle crate tests, which can't receive one from the dispatcher. Enabled by
    /// the `test-utils` feature, which only dev-dependencies turn on. The TX check still
    /// needs `device_id` to be validated.
    #[cfg(any(test, feature = "test-utils"))]
    pub fn for_tests(device_id: &[u8]) -> Self {
        Self::new(device_id.to_vec())
    }

    /// The validated phone that sent the command.
    pub fn device_id(&self) -> &[u8] {
        &self.device_id
    }
}

enum TxClass {
    Platform,
    /// For the command of this phone.
    Command(Vec<u8>),
}

/// A CAN frame queued for transmission, with its class.
pub struct CanTxFrame {
    frame: CanFrame,
    class: TxClass,
}

impl CanTxFrame {
    /// A frame of the platform's own traffic that doesn't act on the car.
    pub fn platform(frame: CanFrame) -> Self {
        Self {
            frame,
            class: TxClass::Platform,
        }
    }

    /// A frame of the vehicle command `authorization` came with.
    pub fn command(authorization: &CommandAuthorization, frame: CanFrame) -> Self {
        Self {
            frame,
            class: TxClass::Command(authorization.device_id.clone()),
        }
    }

    pub fn frame(&self) -> &CanFrame {
        &self.frame
    }

    pub fn bus_id(&self) -> u8 {
        self.frame.bus_id
    }

    /// `true` for a frame of a vehicle command.
    pub fn is_command(&self) -> bool {
        matches!(self.class, TxClass::Command(_))
    }

    /// The phone a command frame is for, `None` for a platform frame.
    pub(crate) fn command_device_id(&self) -> Option<&[u8]> {
        match &self.class {
            TxClass::Platform => None,
            TxClass::Command(id) => Some(id),
        }
    }
}
