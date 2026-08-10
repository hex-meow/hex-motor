//! Meow Motor 产品族的独立 CANopen 驱动协议核心。
//!
//! 本模块不复用 CiA402 状态机或对象地址。它包含可确定性测试的身份识别、定点数、
//! PDO codec/recipe、通信设置和 SDO 命令构造，以及建立在这些纯协议原语之上的异步
//! manager。

pub mod codec;
pub mod commands;
pub mod events;
pub mod fixed;
pub mod identity;
pub mod manager;
pub mod pdo;
pub mod settings;
pub mod types;

mod discovery;
mod entry;
mod listener;

pub use codec::{decode_tpdo1, decode_tpdo2, CodecError, Tpdo1Frame, Tpdo2Frame};
pub use commands::{
    build_clear_error_sdo_write, build_compressed_mit_setup_sdo_writes, build_disable_sdo_write,
    build_max_torque_sdo_write, build_mode_command_sdo_write, build_profile_limits_sdo_writes,
    build_set_mode_sdo_writes, build_target_sdo_writes, CommandBuildError, MeowMitTarget,
    MeowMotorTarget, MeowProfileLimits,
};
pub use events::MeowMotorEvent;
pub use fixed::{Q8_24Error, SignedQ8_24};
pub use identity::{
    lookup_model, model_for_identity, MeowMotorModel, PRODUCT_CODE_4310, PRODUCT_CODE_4342,
    VENDOR_ID,
};
pub use manager::{
    MeowMotorInitializeOptions, MeowMotorManager, MeowMotorManagerOptions, SharedHostPdoConfig,
};
// The 64-bit packer is protocol-neutral.  Re-exporting it here keeps the new
// object's address/initialization separate while preserving the established
// public CiA402 path for existing users.
pub use crate::cia402::compressed_mit::{
    pack_shared_frame, CompressedMitMapping, CompressedMitTarget,
};
pub use pdo::{
    compressed_mit_rpdo1_recipe, device_tpdo1_recipe, device_tpdo2_recipe, host_tpdo_cob_id,
    mode_rpdo2_recipe, pdo_frame_kind,
};
pub use settings::{
    build_settings_update_plan, pdo_communication_save_write, MeowMotorCanSettings,
    MeowMotorCanSettingsPlan, SettingsError, DATA_BITRATES, NOMINAL_BITRATES,
};
pub use types::{
    MeowMotorCanSettingsStatus, MeowMotorConnection, MeowMotorInfo, MeowMotorLifecycle,
    MeowMotorLiveState, MeowMotorLogic, MeowMotorMeasurements, MeowMotorMode,
    MeowMotorReinitReason,
};
