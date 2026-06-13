//! HEX-MOTOR
//!
//! 针对 HEX-MECHA 电机的 Rust 驱动库。
//!
//! v0.1 范围：仅 CiA402 形态电机；仅"上位机交互"使用范式。
//! 详见 `DESIGN.md`。
//!
//! 通信层使用 [`can_transport`] 的 trait，跨平台。
//! SDO 客户端来自 [`canopen_sdo`]。

#![warn(missing_debug_implementations)]
#![warn(rust_2018_idioms)]

pub mod canopen;
pub mod cia402;
pub mod error;
pub mod types;

pub use error::{Error, Result};
