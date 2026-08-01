//! HEX-MOTOR
//!
//! 针对 HEX-MECHA 电机的 Rust 驱动库。
//!
//! 支持 CiA402 电机以及拥有独立对象字典的 Meow Motor 产品族。
//! 两类驱动保持显式分离，只共享 CANopen 通信原语。
//!
//! 通信层使用 [`can_transport`] 的 trait，跨平台。
//! SDO 客户端来自 [`canopen_sdo`]。

#![warn(missing_debug_implementations)]
#![warn(rust_2018_idioms)]

pub mod canopen;
pub mod cia402;
pub mod error;
pub mod meow_motor;
pub mod types;

pub use error::{Error, Result};
