//! 公共错误类型。

use thiserror::Error;

use crate::types::MotorErrorKind;

#[derive(Debug, Error)]
pub enum Error {
    #[error("CAN transport: {0}")]
    Transport(#[from] can_transport::CanIoError),

    #[error("SDO: {0}")]
    Sdo(#[from] canopen_sdo::asynch::AsyncSdoError),

    #[error("unknown node {0}")]
    UnknownNode(u8),

    #[error("invalid node id {0} (must be 1..=127)")]
    InvalidNodeId(u8),

    #[error("invalid COB-ID 0x{cob_id:X}: {reason}")]
    InvalidCobId { cob_id: u16, reason: &'static str },

    #[error("motor reported error: {0:?}")]
    InErrorState(MotorErrorKind),

    #[error("nid 0x{nid:02X} not ready (lifecycle = {lifecycle})")]
    NotReady { nid: u8, lifecycle: String },

    #[error("target `{given}` does not match current mode `{expected}`")]
    TargetModeMismatch {
        expected: String,
        given: &'static str,
    },

    #[error("set_mode confirmation timed out (waiting for TPDO feedback)")]
    ModeConfirmTimeout,

    #[error("internal: {0}")]
    Internal(String),
}

pub type Result<T> = std::result::Result<T, Error>;
