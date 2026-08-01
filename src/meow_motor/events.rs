//! Meow Motor manager 的结构化事件。

use crate::types::MotorIdentity;

use super::identity::MeowMotorModel;
use super::types::{MeowMotorLogic, MeowMotorReinitReason};

#[derive(Debug, Clone)]
pub enum MeowMotorEvent {
    NodeAppeared {
        node_id: u8,
    },
    NodeOnline {
        node_id: u8,
    },
    NodeOffline {
        node_id: u8,
    },
    Identified {
        node_id: u8,
        model: MeowMotorModel,
        identity: MotorIdentity,
    },
    UnsupportedIdentity {
        node_id: u8,
        identity: MotorIdentity,
    },
    IdentifyFailed {
        node_id: u8,
        reason: String,
    },
    Initialized {
        node_id: u8,
    },
    NeedsReinit {
        node_id: u8,
        reason: MeowMotorReinitReason,
    },
    LogicChanged {
        node_id: u8,
        logic: MeowMotorLogic,
    },
}
