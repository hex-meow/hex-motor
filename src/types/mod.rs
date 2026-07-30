//! 跨协议复用的纯数据类型。

pub mod can_config;
pub mod device_settings;
pub mod errors;
pub mod identity;
pub mod mode;
pub mod target;

pub use can_config::{DeviceCanConfig, DeviceCanConfigStatus};
pub use device_settings::{DeviceSettingsResult, DeviceSettingsUpdate};
pub use errors::MotorErrorKind;
pub use identity::MotorIdentity;
pub use mode::MotorMode;
pub use target::{MitMapping, MotorTarget, ProfileParams};
