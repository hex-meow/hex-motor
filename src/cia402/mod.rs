//! CiA402 形态电机驱动（v0.1 唯一支持的协议形态）。
//!
//! 详见 `DESIGN.md`。
//!
//! 入口是 [`Cia402Manager`]。

pub mod known_devices;

pub mod codec;
pub mod compressed_mit;
pub mod events;
pub mod initialize;
pub mod manager;
pub mod sequences;
pub mod subscribe;
pub mod types;

// 内部模块，不导出
mod discovery;
mod heartbeat;
mod motor_entry;
mod tpdo_listener;
mod velocity;

pub use compressed_mit::{
    CompressedMitMapping, CompressedMitTarget, DEFAULT_SHARED_COB_ID,
};
pub use events::{Cia402Event, EventStream, EventStreamItem};
pub use initialize::{
    default_tpdo1_recipe, default_tpdo2_recipe, DEFAULT_TPDO1_COMM, DEFAULT_TPDO1_ENTRIES,
    DEFAULT_TPDO2_COMM, DEFAULT_TPDO2_ENTRIES,
};
pub use manager::{Cia402Manager, Cia402ManagerOptions};
pub use subscribe::{
    OverflowPolicy, StatusStream, StatusStreamItem, StreamOptions, DEFAULT_STREAM_CAPACITY,
};
pub use types::{
    Connection, LiveState, Logic, Measurements, MotorInfo, MotorLifecycle, ReinitReason,
};
