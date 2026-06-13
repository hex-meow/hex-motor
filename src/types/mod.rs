//! 跨协议复用的纯数据类型。

pub mod errors;
pub mod identity;
pub mod mode;
pub mod target;

pub use errors::MotorErrorKind;
pub use identity::MotorIdentity;
pub use mode::MotorMode;
pub use target::{MitMapping, MotorTarget, ProfileParams};
