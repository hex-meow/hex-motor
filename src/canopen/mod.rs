//! CANopen 公共原语（多为纯函数）。
//!
//! 不依赖任何 motor-family 协议；CiA402 / HexMeow 等 driver 共享。

pub mod cob;
pub mod heartbeat;
pub mod nmt;
pub mod sdo;
pub mod tpdo_config;
