//! `initialize()` 完整序列 + 默认 TPDO1/TPDO2 配方。详见 `DESIGN.md` §6。
//!
//! 步骤回顾：
//! 1. lifecycle → `Initializing`；发 `Cia402Event::Initializing`
//! 2. NMT `EnterPreOperational`（目标=该 nid）
//!    等 HB 反馈 NMT 状态变成 `PreOperational`（最多 2 × `motor_heartbeat_period`）
//! 3. SDO 读 `0x6041` 状态字探活（顺便确认 SDO 在 PreOp 仍然通）
//! 4. 用 [`crate::canopen::tpdo_config::build_tpdo_config_writes`] 配 TPDO1（高速，1 ms）
//! 5. 同上配 TPDO2（低速，20 ms）
//! 6. best-effort 读厂家运行时常量（`0x6076` peak_torque / `0x2003:07`
//!    MIT factor）；并把 `0x2003:06` 预设为 1000
//! 7. NMT `StartRemoteNode` → Operational，等 HB 反馈变成 `Operational`
//! 8. 清心跳/CiA402 故障并让看门狗在"好相位"arm：循环 关 `0x1016`→`0x00→0x80→0x00`
//!    复位→开 `0x1016`→等一个心跳超时窗→同时读 `0x6041` 与 `0x603F`，并等 TPDO2
//!    收敛到无故障；两者都干净才算命中正确相位（最多 `init_fault_clear_attempts` 次）。
//!    固件该相位约一半概率单次清不掉，关→开监控等价一次"心跳丢失→恢复"会翻转它，
//!    所以多试几次基本必中。**唯一**自动清错处。
//! 9. lifecycle → `Initialized`；发 `Cia402Event::Initialized`
//!
//! 失败时 [`LifecycleRollback`] 自动把 lifecycle 退回 `Identified`（如果
//! identity 已知）或 `Unknown`，**不会**主动撤销已经下到电机的 SDO 写。
//! 调用方可以直接再次调用 `initialize()` 重试。
//!
//! ## 关于默认 TPDO 映射
//!
//! v0.1 的默认映射是为 HexMeow CiA402 电机量身做的：
//!
//! - **TPDO1（高速 1 ms）**：`0x6064` actual_position(32b) + `0x1013`
//!   high_res_timestamp(32b) + `0x6077` actual_torque(16b) + `0x603F`
//!   error_code(16b) = **12 字节**
//! - **TPDO2（低速 20 ms）**：`0x6041` status(16b) + `0x2204:01` drv_temp(16b)
//!   plus `0x2204:02` motor_temp(16b) + `0x6040` ctrl(16b) + `0x603F`
//!   error_code(16b) = **10 字节**
//!
//! 注意：
//! - 速度故意**不** map，由上位机用 (pos_now-pos_prev)/(ts_now-ts_prev) 算
//!   （HexMeow 的 `0x6064` 是单圈 f32，需要在 host 侧做多圈累积）。
//! - `0x1013`、`0x2204:01/02` 是 vendor-specific 实现，标准 CiA402 不保证
//!   有；其他厂家电机要走自定义 recipe，未来会暴露 `initialize_with_recipes()` API。

use std::sync::Arc;
use std::time::{Duration, Instant};

use can_transport::CanBus;
use tokio::sync::broadcast;

use crate::canopen::{
    heartbeat::encode_consumer_heartbeat_entry,
    nmt::{self, NmtCommand, NmtState},
    sdo,
    tpdo_config::{build_tpdo_config_writes, TpdoCommParams, TpdoEntry, TpdoRecipe},
};
use crate::error::{Error, Result};
use crate::types::MotorIdentity;

use super::events::Cia402Event;
use super::manager::Cia402ManagerOptions;
use super::motor_entry::{MotorEntry, MotorEntryInner};
use super::types::{Logic, MotorLifecycle};

/// 单次完整初始化的结果。`NeedsCommunicationReset` 只表示一个经过严格 SDO
/// 复核的、特定的黏滞心跳错误；是否执行一次通信复位由 manager 统一编排。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum InitializeOutcome {
    Initialized,
    NeedsCommunicationReset {
        status_word: u16,
        error_code: u16,
        attempts: u8,
    },
}

/// 默认 TPDO1（高速 1 ms）映射：位置 + 时间戳 + 力矩 + 错误码 = 12 字节。
/// 速度由上位机用 (pos_now-pos_prev)/(ts_now-ts_prev) 计算。
pub const DEFAULT_TPDO1_ENTRIES: &[TpdoEntry] = &[
    TpdoEntry {
        index: 0x6064,
        subindex: 0,
        bit_len: 32,
    }, // actual_position（HexMeow CiA402: 单圈 f32；标准 CiA402: i32 encoder pulse）
    TpdoEntry {
        index: 0x1013,
        subindex: 0,
        bit_len: 32,
    }, // high_resolution_time_stamp (us, u32)
    TpdoEntry {
        index: 0x6077,
        subindex: 0,
        bit_len: 16,
    }, // actual_torque (i16, ‰ of peak)
    TpdoEntry {
        index: 0x603F,
        subindex: 0,
        bit_len: 16,
    }, // error_code (u16)
];

/// 默认 TPDO2（低速 20 ms）映射：状态字 + 驱动器/电机温度 + 控制字 + 错误码 = 10 字节。
pub const DEFAULT_TPDO2_ENTRIES: &[TpdoEntry] = &[
    TpdoEntry {
        index: 0x6041,
        subindex: 0,
        bit_len: 16,
    }, // status_word
    TpdoEntry {
        index: 0x2204,
        subindex: 1,
        bit_len: 16,
    }, // driver_temperature_x10 (vendor-specific, i16 ×0.1 ℃)
    TpdoEntry {
        index: 0x2204,
        subindex: 2,
        bit_len: 16,
    }, // motor_temperature_x10 (vendor-specific, i16 ×0.1 ℃)
    TpdoEntry {
        index: 0x6040,
        subindex: 0,
        bit_len: 16,
    }, // control_word（回读当前 CW）
    TpdoEntry {
        index: 0x603F,
        subindex: 0,
        bit_len: 16,
    }, // error_code (u16) —— 和 TPDO1 重复一份，方便低速通道也能看到
];

/// 默认 TPDO1 通信参数：异步事件 + 0.5 ms inhibit + 1 ms 周期（1000 Hz）。
pub const DEFAULT_TPDO1_COMM: TpdoCommParams = TpdoCommParams {
    transmission_type: 255,
    inhibit_time_x100us: 5, // 0.5 ms
    event_timer_ms: 1,      // 1000 Hz
};

/// 默认 TPDO2 通信参数：异步事件 + 19 ms inhibit + 20 ms 周期（50 Hz）。
pub const DEFAULT_TPDO2_COMM: TpdoCommParams = TpdoCommParams {
    transmission_type: 255,
    inhibit_time_x100us: 190, // 19 ms
    event_timer_ms: 20,       // 50 Hz
};

/// 给指定 nid 构造默认 TPDO1 recipe (`cob_id = 0x180 + nid`)。
pub fn default_tpdo1_recipe(nid: u8) -> TpdoRecipe {
    TpdoRecipe {
        tpdo_index: 0,
        cob_id: 0x180 + nid as u16,
        entries: DEFAULT_TPDO1_ENTRIES.to_vec(),
        comm: DEFAULT_TPDO1_COMM,
    }
}

/// 给指定 nid 构造默认 TPDO2 recipe (`cob_id = 0x280 + nid`)。
pub fn default_tpdo2_recipe(nid: u8) -> TpdoRecipe {
    TpdoRecipe {
        tpdo_index: 1,
        cob_id: 0x280 + nid as u16,
        entries: DEFAULT_TPDO2_ENTRIES.to_vec(),
        comm: DEFAULT_TPDO2_COMM,
    }
}

/// 故障复位时控制字边沿之间的 settle。比 [`INTER_WRITE_DELAY`](super::sequences::INTER_WRITE_DELAY)
/// (10 ms) 长得多：
/// 电机对控制字是"采样最新值"，写太快只有最后一次生效；fault reset 又依赖 bit7
/// 的干净 0→1→0 边沿，所以这里给足时间，确保每个边沿都被电机登记。
const FAULT_RESET_SETTLE: Duration = Duration::from_millis(50);

/// SDO 已确认清错后，给 50 Hz TPDO2 足够时间发布同一状态。这样
/// `set_mode()` 不会因为一帧仍带旧 `0x603F` 的缓存而立即拒绝刚完成的初始化。
const TPDO2_CLEAR_CONFIRM_TIMEOUT: Duration = Duration::from_millis(100);

fn fault_clear_observation_is_clean(status_word: u16, error_code: u16) -> bool {
    !super::codec::status_word_has_fault(status_word) && error_code == 0
}

/// 仅识别真机调查中确认过的黏滞状态：CiA402 Fault 位已经清掉，但 heartbeat
/// error code 仍保持 0x8130。普通 CiA402 故障不得借此触发通信复位。
fn needs_reset_communication_recovery(status_word: u16, error_code: u16) -> bool {
    !super::codec::status_word_has_fault(status_word) && error_code == 0x8130
}

fn has_fresh_clean_tpdo2(inner: &MotorEntryInner, baseline_revision: u64) -> bool {
    inner.tpdo2_revision != baseline_revision
        && inner
            .measurements
            .status_word
            .is_some_and(|sw| !super::codec::status_word_has_fault(sw))
        && matches!(inner.logic, Some(ref logic) if !matches!(logic, Logic::Error { .. }))
}

async fn wait_for_clean_tpdo2(
    entry: &Arc<MotorEntry>,
    baseline_revision: u64,
    timeout: Duration,
) -> bool {
    let deadline = Instant::now() + timeout;
    loop {
        let clean = {
            let inner = entry.inner.lock().unwrap();
            has_fresh_clean_tpdo2(&inner, baseline_revision)
        };
        if clean {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
}

fn initialization_context_problem(
    inner: &MotorEntryInner,
    expected_session_epoch: u64,
    expected_identity: &MotorIdentity,
    expected_nmt_revision: Option<u64>,
    max_heartbeat_age: Option<Duration>,
    now: Instant,
) -> Option<String> {
    if inner.session_epoch != expected_session_epoch {
        return Some(format!(
            "device session changed (expected epoch {expected_session_epoch}, observed {})",
            inner.session_epoch
        ));
    }
    if inner.identity.as_ref() != Some(expected_identity) {
        return Some("device identity changed or was invalidated".into());
    }
    if !matches!(inner.lifecycle, MotorLifecycle::Initializing) {
        return Some(format!(
            "lifecycle left Initializing ({:?})",
            inner.lifecycle
        ));
    }
    if !inner.online {
        return Some("device is offline".into());
    }
    if inner.nmt_state != Some(NmtState::Operational) {
        return Some(format!(
            "NMT state is not Operational ({:?})",
            inner.nmt_state
        ));
    }
    if expected_nmt_revision.is_some_and(|revision| inner.nmt_revision != revision) {
        return Some(format!(
            "NMT state changed after Operational fence (expected revision {}, observed {})",
            expected_nmt_revision.expect("checked Some"),
            inner.nmt_revision
        ));
    }
    if let Some(max_age) = max_heartbeat_age {
        let Some(last_heartbeat) = inner.last_heartbeat else {
            return Some("no motor heartbeat observed".into());
        };
        let age = now.saturating_duration_since(last_heartbeat);
        if age > max_age {
            return Some(format!(
                "last motor heartbeat is stale ({age:?} > {max_age:?})"
            ));
        }
    }
    None
}

/// 一个干净的 fault-reset 上升沿：`0x00 → 0x80 → 0x00`，每步之间
/// [`FAULT_RESET_SETTLE`]。`0x80` 的 bit7 0→1 触发复位，末尾落回 `0x00`
/// 让状态机停在 Switch On Disabled，同时为下一次复位重新备好 bit7=0 基线。
async fn clear_fault_edge(bus: &dyn CanBus, nid: u8, sdo_timeout: Option<Duration>) -> Result<()> {
    sdo::download_u16(bus, nid, 0x6040, 0, 0x0000, sdo_timeout).await?;
    tokio::time::sleep(FAULT_RESET_SETTLE).await;
    sdo::download_u16(bus, nid, 0x6040, 0, 0x0080, sdo_timeout).await?;
    tokio::time::sleep(FAULT_RESET_SETTLE).await;
    sdo::download_u16(bus, nid, 0x6040, 0, 0x0000, sdo_timeout).await?;
    tokio::time::sleep(FAULT_RESET_SETTLE).await;
    Ok(())
}

/// 完整 initialize 序列。**调用方必须先把 lifecycle != Initializing 并保留
/// `inflight_ops` 中的标记**，详见 [`crate::cia402::manager::Cia402Manager::initialize`]。
pub(crate) async fn run_initialize(
    bus: &dyn CanBus,
    entry: Arc<MotorEntry>,
    events_tx: &broadcast::Sender<Cia402Event>,
    opts: &Cia402ManagerOptions,
) -> Result<InitializeOutcome> {
    let nid = entry.node_id;
    let sdo_timeout = Some(opts.sdo_timeout);

    // 1. lifecycle → Initializing；同时清掉 control-side 缓存（旧的 mode/logic
    //    在 motor 重新配过 OD 之后都失效了）
    let (session_epoch, expected_identity) = {
        let mut inner = entry.inner.lock().unwrap();
        if matches!(inner.lifecycle, MotorLifecycle::Initializing) {
            return Err(Error::Internal(format!(
                "nid 0x{nid:02X}: initialize already running"
            )));
        }
        let identity = inner.identity.clone().ok_or_else(|| {
            Error::Internal(format!(
                "nid 0x{nid:02X}: initialize requires a current device identity"
            ))
        })?;
        let session_epoch = inner.session_epoch;
        inner.lifecycle = MotorLifecycle::Initializing;
        inner.target_mode = None;
        inner.logic = None;
        inner.peak_torque_nm = None;
        inner.mit_kp_kd_factor = None;
        inner.measurements = Default::default();
        inner.vel_filter = Default::default();
        (session_epoch, identity)
    };
    let _ = events_tx.send(Cia402Event::Initializing { nid });

    // 失败时自动回退 lifecycle 到 Identified / Unknown
    let mut rollback = LifecycleRollback::new(entry.clone());

    // 2. NMT EnterPreOperational + 等待 HB 反馈
    let preop_cmd = nmt::build_nmt_command(NmtCommand::EnterPreOperational, nid)?;
    bus.send(preop_cmd).await?;
    wait_for_nmt_state(
        &entry,
        NmtState::PreOperational,
        opts.motor_heartbeat_period * 2,
    )
    .await?;
    log::info!("nid 0x{nid:02X}: NMT = PreOperational");

    // 3. SDO 探活：读 0x6041 status_word
    let _sw = sdo::upload_u16(bus, nid, 0x6041, 0, sdo_timeout).await?;

    // 4. 配置 TPDO1（高速）
    apply_tpdo_recipe(bus, nid, &default_tpdo1_recipe(nid), sdo_timeout).await?;

    // 5. 配置 TPDO2（低速）
    apply_tpdo_recipe(bus, nid, &default_tpdo2_recipe(nid), sdo_timeout).await?;

    // 6. best-effort 读厂家运行时常量（HexMeow CiA402 vendor-specific）：
    //    - 0x6076 Motor Peak Torque (REAL32, mNm) —— 后面 Torque target 用
    //    - 0x2003:07 MIT KP/KD Factor (REAL32) —— 后面 Mit target 用
    //    - 0x2003:06 MIT KP/KD Limit (UNSIGNED16) 预设为 1000 (full PD authority)
    //    任意一条失败只 log warn，不影响 init 成功；用对应模式时再报错。
    read_runtime_constants(bus, &entry, sdo_timeout).await;
    let _ = sdo::download_u16(bus, nid, 0x2003, 0x06, 1000, sdo_timeout)
        .await
        .map_err(|e| {
            log::debug!(
                "nid 0x{nid:02X}: 0x2003:06 (MIT PD limit) not writable ({e}); \
                 Mit mode will rely on motor default"
            );
        });

    // 7. NMT StartRemoteNode → Operational（PDO 开始流；CiA402 故障与否都会发
    //    PDO 反馈，所以即便此刻仍带故障，上位机也已经能看到数据）。
    let op_cmd = nmt::build_nmt_command(NmtCommand::StartRemoteNode, nid)?;
    bus.send(op_cmd).await?;
    wait_for_nmt_state(
        &entry,
        NmtState::Operational,
        opts.motor_heartbeat_period * 2,
    )
    .await?;
    log::info!("nid 0x{nid:02X}: NMT = Operational");
    let operational_nmt_revision = {
        let inner = entry.inner.lock().unwrap();
        if let Some(problem) = initialization_context_problem(
            &inner,
            session_epoch,
            &expected_identity,
            None,
            None,
            Instant::now(),
        ) {
            return Err(Error::Internal(format!(
                "nid 0x{nid:02X}: initialization context invalid after entering Operational: {problem}"
            )));
        }
        inner.nmt_revision
    };

    // 8. 清心跳故障 + 让看门狗在"好相位"上 arm（含上次掉电造成的 HeartbeatLost）。
    //
    //    真机现象（连可跑通的 C 参考也一样）：心跳故障能否清掉跟固件内部一个随
    //    "心跳丢失→恢复"翻转的相位有关，**单次清除约一半概率失败**。0x1016 监控
    //    的关→开恰好等价于一次"丢失→恢复"，会翻转这个相位。于是这里循环：
    //      关监控(0x1016=0) → 干净 fault-reset 边沿 → 开监控(0x1016=consumer)
    //      → 等一个心跳超时窗口看会不会重新 latch → 同时读 0x6041 与 0x603F，
    //        再确认 TPDO2 缓存也已收敛。
    //    没清掉就再来一轮（每轮翻一次相位），命中干净相位就停。
    //
    //    这是**唯一**自动清错的地方，且只在 initialize 时发生。init 完成后运行
    //    中再出故障**不自动清**：由上层报给用户，用户手动 clear + 重新 initialize。
    let timeout_ms = opts
        .consumer_heartbeat_timeout
        .as_millis()
        .min(u16::MAX as u128) as u16;
    let consumer = encode_consumer_heartbeat_entry(opts.heartbeat_node_id, timeout_ms);
    // 验证窗口要盖过一个消费者超时周期，才能观察到坏相位下的重新 latch。
    let verify_wait = opts.consumer_heartbeat_timeout + Duration::from_millis(100);
    let attempts = opts.init_fault_clear_attempts.max(1);

    let mut cleared = false;
    // 只允许最后一次完整 SDO 观测决定是否升级到通信复位。若最后一次任一读取
    // 失败，绝不能沿用更早的 0x8130 观测。
    let mut last_complete_observation = None;
    for attempt in 1..=attempts {
        // a. 关监控（等价一次"心跳丢失"，翻转固件相位；也让随后的复位能落实）
        sdo::download_u32(bus, nid, 0x1016, 1, 0, sdo_timeout).await?;
        // b. 干净的 fault-reset 边沿：0x00 → 0x80 → 0x00（详见 clear_fault_edge）
        clear_fault_edge(bus, nid, sdo_timeout).await?;
        // c. 重新开监控（等价"心跳恢复"）
        sdo::download_u32(bus, nid, 0x1016, 1, consumer, sdo_timeout).await?;
        // d. 等一个心跳超时窗口，坏相位会在这期间重新 latch
        tokio::time::sleep(verify_wait).await;
        // e. 状态字与错误码必须同时干净。真机上 0x6041.bit3 可能先清，而
        //    0x603F 仍保持 0x8130；只检查 bit3 会让随后的 set_mode 立即拒绝。
        let status_word = sdo::upload_u16(bus, nid, 0x6041, 0, sdo_timeout).await;
        let error_code = sdo::upload_u16(bus, nid, 0x603F, 0, sdo_timeout).await;
        match (status_word, error_code) {
            (Ok(sw), Ok(code)) if fault_clear_observation_is_clean(sw, code) => {
                last_complete_observation = Some((sw, code));
                // Fence after both SDO reads: an older clean cache cannot satisfy this wait.
                let tpdo2_revision = entry.inner.lock().unwrap().tpdo2_revision;
                if wait_for_clean_tpdo2(&entry, tpdo2_revision, TPDO2_CLEAR_CONFIRM_TIMEOUT).await {
                    log::info!(
                        "nid 0x{nid:02X}: heartbeat/CiA402 fault cleared & armed \
                         (sw=0x{sw:04X}, error=0x{code:04X}, \
                         0x1016=0x{consumer:08X}) on attempt {attempt}/{attempts}"
                    );
                    cleared = true;
                    break;
                }
                let (cached_sw, cached_logic) = {
                    let inner = entry.inner.lock().unwrap();
                    (inner.measurements.status_word, inner.logic.clone())
                };
                log::warn!(
                    "nid 0x{nid:02X}: SDO fault state cleared but TPDO2 did not converge \
                     (cached sw={cached_sw:?}, logic={cached_logic:?}) on attempt \
                     {attempt}/{attempts}; re-toggling heartbeat monitor"
                );
            }
            (Ok(sw), Ok(code)) => {
                last_complete_observation = Some((sw, code));
                log::warn!(
                    "nid 0x{nid:02X}: still faulted (sw=0x{sw:04X}, error=0x{code:04X}) \
                     after attempt {attempt}/{attempts}; re-toggling heartbeat monitor to flip phase"
                );
            }
            (Err(error), Ok(code)) => {
                last_complete_observation = None;
                log::warn!(
                    "nid 0x{nid:02X}: read 0x6041 failed on attempt {attempt}/{attempts} \
                     (0x603F=0x{code:04X}): {error}"
                );
            }
            (Ok(sw), Err(error)) => {
                last_complete_observation = None;
                log::warn!(
                    "nid 0x{nid:02X}: read 0x603F failed on attempt {attempt}/{attempts} \
                     (0x6041=0x{sw:04X}): {error}"
                );
            }
            (Err(status_error), Err(code_error)) => {
                last_complete_observation = None;
                log::warn!(
                    "nid 0x{nid:02X}: read 0x6041 and 0x603F failed on attempt \
                     {attempt}/{attempts}: status={status_error}; error_code={code_error}"
                );
            }
        }
    }
    if !cleared {
        if let Some((status_word, error_code)) = last_complete_observation
            .filter(|(sw, code)| needs_reset_communication_recovery(*sw, *code))
        {
            let inner = entry.inner.lock().unwrap();
            if let Some(problem) = initialization_context_problem(
                &inner,
                session_epoch,
                &expected_identity,
                Some(operational_nmt_revision),
                Some(opts.motor_heartbeat_period * 2),
                Instant::now(),
            ) {
                return Err(Error::Internal(format!(
                    "nid 0x{nid:02X}: refusing heartbeat recovery because initialization context changed: {problem}"
                )));
            }
            drop(inner);
            return Ok(InitializeOutcome::NeedsCommunicationReset {
                status_word,
                error_code,
                attempts,
            });
        }
        return Err(Error::Internal(format!(
            "nid 0x{nid:02X}: could not clear heartbeat/CiA402 fault after {attempts} \
             attempts (last complete observation: {last_complete_observation:?})"
        )));
    }

    // 11. 同一把锁内复核 session/NMT/heartbeat 后再提交 Initialized。
    // discovery 在 Initializing 期间不会替我们写 NeedsReinit，因此这里不能只看
    // fault-clear 的结果；Boot-Up、离开 Operational 或过期心跳都必须失败关闭。
    {
        let mut inner = entry.inner.lock().unwrap();
        if let Some(problem) = initialization_context_problem(
            &inner,
            session_epoch,
            &expected_identity,
            Some(operational_nmt_revision),
            Some(opts.motor_heartbeat_period * 2),
            Instant::now(),
        ) {
            drop(inner);
            return Err(Error::Internal(format!(
                "nid 0x{nid:02X}: initialization commit rejected: {problem}"
            )));
        }
        inner.lifecycle = MotorLifecycle::Initialized;
    }
    rollback.disarm();
    let _ = events_tx.send(Cia402Event::Initialized { nid });
    Ok(InitializeOutcome::Initialized)
}

/// Best-effort 读 0x6076 (Motor Peak Torque) + 0x2003:07 (MIT KP/KD Factor)，
/// 缓存到 [`MotorEntry`]。失败只 log，不返回 Error。
async fn read_runtime_constants(
    bus: &dyn CanBus,
    entry: &Arc<MotorEntry>,
    sdo_timeout: Option<Duration>,
) {
    let nid = entry.node_id;

    // 0x6076 Motor Peak Torque：设备对象字典规定为 REAL32，单位 **Nm**。
    // 直接缓存，不做单位换算。
    match sdo::upload_f32(bus, nid, 0x6076, 0, sdo_timeout).await {
        Ok(nm) => {
            log::info!("nid 0x{nid:02X}: 0x6076 (Motor Peak Torque) = {nm} Nm");
            entry.inner.lock().unwrap().peak_torque_nm = Some(nm);
        }
        Err(e) => {
            log::warn!(
                "nid 0x{nid:02X}: 0x6076 (Motor Peak Torque) not readable ({e}); \
                 Torque-mode target writes will be unavailable"
            );
        }
    }

    // 0x2003:07 MIT KP/KD Factor：REAL32。物理 Kp [Nm/Rev] = kp_int × factor。
    match sdo::upload_f32(bus, nid, 0x2003, 0x07, sdo_timeout).await {
        Ok(factor) => {
            log::info!("nid 0x{nid:02X}: 0x2003:07 (MIT KP/KD Factor) = {factor}");
            entry.inner.lock().unwrap().mit_kp_kd_factor = Some(factor);
        }
        Err(e) => {
            log::warn!(
                "nid 0x{nid:02X}: 0x2003:07 (MIT KP/KD Factor) not readable ({e}); \
                 Mit-mode target writes will be unavailable"
            );
        }
    }
}

/// 一次性把 recipe 编译出的所有 SDO 写顺序下发给电机。
async fn apply_tpdo_recipe(
    bus: &dyn CanBus,
    nid: u8,
    recipe: &TpdoRecipe,
    sdo_timeout: Option<Duration>,
) -> Result<()> {
    let writes = build_tpdo_config_writes(recipe)?;
    log::debug!(
        "nid 0x{nid:02X}: TPDO{} cob_id=0x{:03X}: {} SDO ops ({} bytes/frame)",
        recipe.tpdo_index + 1,
        recipe.cob_id,
        writes.len(),
        recipe.total_bytes(),
    );
    let last = writes.len().saturating_sub(1);
    for (i, w) in writes.iter().enumerate() {
        sdo::download(bus, nid, w.index, w.subindex, &w.data, sdo_timeout).await?;
        // 同一 recipe 会先后两次写 0x1A0x:0（映射数）和 0x180x:1
        //（disable/enable）。设备的对象字典需要同址写至少间隔 5 ms；沿用
        // 控制序列已经验证过的 10 ms settle，也避免依赖中间 SDO 往返耗时。
        if i != last {
            tokio::time::sleep(super::sequences::INTER_WRITE_DELAY).await;
        }
    }
    Ok(())
}

/// 轮询 [`MotorEntry::nmt_state`]（由 discovery task 在每帧 HB 时写入）直到
/// 等于 `target` 或超时。
async fn wait_for_nmt_state(
    entry: &Arc<MotorEntry>,
    target: NmtState,
    timeout: Duration,
) -> Result<()> {
    // 先看一眼现在的状态，命中就直接返回
    {
        let inner = entry.inner.lock().unwrap();
        if inner.nmt_state == Some(target) {
            return Ok(());
        }
    }
    let deadline = Instant::now() + timeout;
    let poll_period = Duration::from_millis(20);
    loop {
        tokio::time::sleep(poll_period).await;
        {
            let inner = entry.inner.lock().unwrap();
            if inner.nmt_state == Some(target) {
                return Ok(());
            }
        }
        if Instant::now() >= deadline {
            let observed = entry.inner.lock().unwrap().nmt_state;
            return Err(Error::Internal(format!(
                "nid 0x{:02X}: timeout waiting NMT {:?} (last observed {:?})",
                entry.node_id, target, observed,
            )));
        }
    }
}

/// RAII：函数提前返回（错误 / panic）时把 lifecycle 退回。
struct LifecycleRollback {
    entry: Arc<MotorEntry>,
    armed: bool,
}

impl LifecycleRollback {
    fn new(entry: Arc<MotorEntry>) -> Self {
        Self { entry, armed: true }
    }

    /// 成功路径在最后调用一次。
    fn disarm(&mut self) {
        self.armed = false;
    }
}

impl Drop for LifecycleRollback {
    fn drop(&mut self) {
        if !self.armed {
            return;
        }
        let mut inner = self.entry.inner.lock().unwrap();
        // 仅当还卡在 Initializing 时回退（避免覆盖成功后的状态）
        if matches!(inner.lifecycle, MotorLifecycle::Initializing) {
            inner.lifecycle = if inner.identity.is_some() {
                MotorLifecycle::Identified
            } else {
                MotorLifecycle::Unknown
            };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_tpdo1_is_12_bytes_4_entries() {
        let r = default_tpdo1_recipe(0x10);
        assert_eq!(r.total_bytes(), 12);
        assert_eq!(r.entries.len(), 4);
        assert_eq!(r.cob_id, 0x190);
        assert_eq!(r.tpdo_index, 0);
        assert!(r.validate().is_ok());
    }

    #[test]
    fn default_tpdo2_is_10_bytes_5_entries() {
        let r = default_tpdo2_recipe(0x10);
        assert_eq!(r.total_bytes(), 10);
        assert_eq!(r.entries.len(), 5);
        assert_eq!(r.cob_id, 0x290);
        assert_eq!(r.tpdo_index, 1);
        assert!(r.validate().is_ok());
    }

    #[test]
    fn default_tpdo1_timing_is_high_speed() {
        assert_eq!(DEFAULT_TPDO1_COMM.transmission_type, 255);
        assert_eq!(DEFAULT_TPDO1_COMM.inhibit_time_x100us, 5);
        assert_eq!(DEFAULT_TPDO1_COMM.event_timer_ms, 1);
    }

    #[test]
    fn default_tpdo2_timing_is_low_speed() {
        assert_eq!(DEFAULT_TPDO2_COMM.transmission_type, 255);
        assert_eq!(DEFAULT_TPDO2_COMM.inhibit_time_x100us, 190);
        assert_eq!(DEFAULT_TPDO2_COMM.event_timer_ms, 20);
    }

    #[test]
    fn tpdo1_and_tpdo2_use_different_cob_and_index() {
        let r1 = default_tpdo1_recipe(0x21);
        let r2 = default_tpdo2_recipe(0x21);
        assert_eq!(r1.cob_id, 0x1A1);
        assert_eq!(r2.cob_id, 0x2A1);
        assert_ne!(r1.tpdo_index, r2.tpdo_index);
    }

    #[test]
    fn fault_clear_requires_clean_status_word_and_zero_error_code() {
        assert!(fault_clear_observation_is_clean(0x0270, 0x0000));
        assert!(!fault_clear_observation_is_clean(0x0270, 0x8130));
        assert!(!fault_clear_observation_is_clean(0x0278, 0x0000));
    }

    #[test]
    fn communication_reset_recovery_only_matches_the_confirmed_sticky_signature() {
        assert!(needs_reset_communication_recovery(0x0270, 0x8130));
        assert!(!needs_reset_communication_recovery(0x0278, 0x8130));
        assert!(!needs_reset_communication_recovery(0x0270, 0x2310));
        assert!(!needs_reset_communication_recovery(0x0270, 0x0000));
    }

    #[test]
    fn clean_tpdo2_must_be_newer_than_the_sdo_fence() {
        let entry = MotorEntry::new(0x15);
        let mut inner = entry.inner.lock().unwrap();
        inner.measurements.status_word = Some(0x0270);
        inner.logic = Some(Logic::Disabled);
        inner.tpdo2_revision = 7;

        assert!(!has_fresh_clean_tpdo2(&inner, 7));
        inner.tpdo2_revision = 8;
        assert!(has_fresh_clean_tpdo2(&inner, 7));
        inner.logic = Some(Logic::Error {
            kind: crate::types::MotorErrorKind::HeartbeatLost,
            raw_code: 0x8130,
        });
        assert!(!has_fresh_clean_tpdo2(&inner, 7));
    }

    #[test]
    fn initialization_commit_requires_same_operational_session_and_fresh_heartbeat() {
        let entry = MotorEntry::new(0x15);
        let identity = MotorIdentity {
            node_id: 0x15,
            vendor_id: 1,
            product_code: 2,
            revision_number: 3,
            serial_number: 4,
            product_name: None,
        };
        let now = Instant::now();
        let mut inner = entry.inner.lock().unwrap();
        inner.session_epoch = 5;
        inner.identity = Some(identity.clone());
        inner.lifecycle = MotorLifecycle::Initializing;
        inner.online = true;
        inner.nmt_state = Some(NmtState::Operational);
        inner.nmt_revision = 9;
        inner.last_heartbeat = Some(now - Duration::from_millis(20));

        assert!(initialization_context_problem(
            &inner,
            5,
            &identity,
            Some(9),
            Some(Duration::from_secs(1)),
            now,
        )
        .is_none());

        inner.nmt_state = Some(NmtState::PreOperational);
        assert!(initialization_context_problem(
            &inner,
            5,
            &identity,
            Some(9),
            Some(Duration::from_secs(1)),
            now,
        )
        .is_some());
        inner.nmt_state = Some(NmtState::Operational);
        inner.session_epoch = 6;
        assert!(initialization_context_problem(
            &inner,
            5,
            &identity,
            Some(9),
            Some(Duration::from_secs(1)),
            now,
        )
        .is_some());
    }
}
