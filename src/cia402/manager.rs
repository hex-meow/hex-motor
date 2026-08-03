//! [`Cia402Manager`]：v0.1 唯一的对外门面。
//!
//! 详细职责见 `DESIGN.md` §3。截至 M3：
//! - 构造启动 4 个常驻 task：HB 广播 / HB 监听 (discovery + auto-identify) /
//!   TPDO 监听 (更新 last_tpdo) / 在线状态监控
//! - `list()` / `subscribe_events()` / `identify(nid)`
//! - `initialize(nid)` / `initialize_all()`
//!
//! M4+ 会在此基础上叠加 `set_mode()` / `set_target()` 等。

use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex as StdMutex, RwLock};
use std::time::{Duration, Instant};

use can_transport::CanBus;
use canopen_sdo::{asynch::AsyncSdoError, SdoAbortCode, SdoError};
use futures::future::join_all;
use tokio::sync::broadcast;
use tokio::task::JoinHandle;
use tokio_util::sync::CancellationToken;

use crate::canopen::nmt::{self, NmtCommand, NmtState};
use crate::canopen::sdo;
use crate::canopen::tpdo_config::SdoWrite;
use crate::error::{Error, Result};
use crate::types::{
    DeviceCanConfig, DeviceCanConfigStatus, DeviceSettingsResult, DeviceSettingsUpdate, MotorMode,
    MotorIdentity, MotorTarget,
};

use super::device_can::{
    begin_device_can_config_read, build_device_settings_plan, dialect_for_tuple,
    finish_device_can_config_read, is_object_2001_tuple, read_device_can_config,
    read_device_settings_snapshot, validate_role_node_id, DeviceSettingsSnapshot,
};
use super::discovery::{identify_once, run_discovery, run_liveness_monitor};
use super::events::{Cia402Event, EventStream, DEFAULT_EVENTS_CAPACITY};
use super::heartbeat::run_hb_broadcast;
use super::initialize::{run_initialize, InitializeOutcome};
use super::known_devices;
use super::motor_entry::MotorEntry;
use super::sequences::{
    build_clear_error_writes, build_disable_writes, build_set_mode_writes,
    build_set_target_writes, SetTargetContext, INTER_WRITE_DELAY,
};
use super::subscribe::{StatusStream, StreamOptions, Subscriber};
use super::tpdo_listener::run_tpdo_listener;
use super::types::{LiveState, Logic, MotorInfo, MotorLifecycle};

/// 构造 [`Cia402Manager`] 时的可调参数。
#[derive(Debug, Clone)]
pub struct Cia402ManagerOptions {
    /// 我方在总线上的 NMT 节点 ID。电机端 0x1016 监听这个 producer。
    /// 必须 1..=127。和总线上任何电机的 nid 都不一样。
    pub heartbeat_node_id: u8,
    /// 我方出向心跳周期，默认 50 ms。电机端 0x1016 据此判我们是否还活着。
    pub heartbeat_period: Duration,
    /// 写到电机 0x1016 的 consumer 超时（M3+ 用），默认 250 ms。
    pub consumer_heartbeat_timeout: Duration,
    /// SDO 超时，默认 200 ms。
    pub sdo_timeout: Duration,
    /// `set_mode` 等 TPDO 反馈的超时（M4+ 用），默认 1 s。
    pub mode_confirm_timeout: Duration,
    /// 事件 channel 容量，默认 [`DEFAULT_EVENTS_CAPACITY`]。
    pub events_capacity: usize,

    /// 期望的电机出向心跳周期（电机 → 我们）。**pre-init 阶段**电机只发心跳，
    /// 用这个推 offline 阈值；HEX-MECHA 电机出厂默认 0.5 s，所以默认 500 ms。
    pub motor_heartbeat_period: Duration,

    /// **post-init 阶段**（lifecycle == Initialized，TPDO 在流）的 offline 阈值。
    /// 默认 200 ms。TPDO 默认 ~20 ms 一发，给 10x 容错。
    pub initialized_stale_threshold: Duration,

    /// 速度滤波的滑动时间窗。TPDO1 ~1 kHz，默认 15 ms（≈15 个样本做最小二乘
    /// 斜率）。调大更平滑、相位滞后更多；调小更跟手、噪声更大。
    pub velocity_window: Duration,

    /// `initialize()` 里清心跳/CiA402 故障的最多尝试次数。固件清故障有个随
    /// "心跳丢失→恢复"翻转的相位、单次约一半概率失败，每次尝试翻一次相位，
    /// 所以多试几次基本必中。默认 6（最坏 ~6×(超时+100ms)，只在 init 时发生）。
    pub init_fault_clear_attempts: u8,

    /// 是否广播我方心跳（`0x700 + heartbeat_node_id`）。默认 `true`。
    ///
    /// 控制电机时需要 `true`（电机端 0x1016 靠它判我们是否在线）。但**纯发现 /
    /// 改 ID 等只读/配置场景应设为 `false`**：否则一旦总线上没有其它在线节点
    /// （比如把唯一的电机断电），我方持续广播的帧无人 ACK，CAN 控制器会疯狂
    /// 重发、错误计数飙升。关掉广播后只剩 RX（监听电机心跳/TPDO），不会发 TX。
    pub broadcast_heartbeat: bool,
}

impl Default for Cia402ManagerOptions {
    fn default() -> Self {
        Self {
            heartbeat_node_id: 0x10,
            heartbeat_period: Duration::from_millis(50),
            consumer_heartbeat_timeout: Duration::from_millis(250),
            sdo_timeout: Duration::from_millis(200),
            mode_confirm_timeout: Duration::from_secs(1),
            events_capacity: DEFAULT_EVENTS_CAPACITY,
            motor_heartbeat_period: Duration::from_millis(500),
            initialized_stale_threshold: Duration::from_millis(200),
            velocity_window: Duration::from_millis(15),
            init_fault_clear_attempts: 6,
            broadcast_heartbeat: true,
        }
    }
}

/// CiA402 形态电机的 manager。一条 CAN 线对应一个 Manager。
pub struct Cia402Manager {
    bus: Arc<dyn CanBus>,
    opts: Cia402ManagerOptions,
    motors: Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>>,
    /// 所有"独占 SDO 操作"(identify / initialize) 共享此集合：
    /// 同 nid 同时只能有一种在跑，否则 SDO 段会互撞响应帧。
    inflight_ops: Arc<StdMutex<HashSet<u8>>>,
    events_tx: broadcast::Sender<Cia402Event>,
    cancel: CancellationToken,
    /// 后台 task 句柄。`drop` 时通过 `cancel` 让它们退出，不在 drop 里 await。
    tasks: Vec<JoinHandle<()>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct DeviceSessionToken {
    session_epoch: u64,
    nmt_revision: u64,
    nmt_state: NmtState,
}

impl std::fmt::Debug for Cia402Manager {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Cia402Manager")
            .field("opts", &self.opts)
            .field("motors_known", &self.motors.read().unwrap().len())
            .field("running_tasks", &self.tasks.len())
            .finish()
    }
}

impl Cia402Manager {
    /// 构造 + 启动后台 task（HB 广播 / discovery / liveness）。
    ///
    /// 必须在 tokio runtime 上下文里调用。
    pub fn new(bus: Arc<dyn CanBus>, opts: Cia402ManagerOptions) -> Result<Self> {
        if !(1..=127).contains(&opts.heartbeat_node_id) {
            return Err(crate::error::Error::Internal(format!(
                "heartbeat_node_id must be 1..=127, got 0x{:02X}",
                opts.heartbeat_node_id
            )));
        }
        if opts.heartbeat_period.is_zero() {
            return Err(crate::error::Error::Internal(
                "heartbeat_period must be > 0".into(),
            ));
        }
        if opts.events_capacity == 0 {
            return Err(crate::error::Error::Internal(
                "events_capacity must be > 0".into(),
            ));
        }
        if opts.motor_heartbeat_period.is_zero() {
            return Err(crate::error::Error::Internal(
                "motor_heartbeat_period must be > 0".into(),
            ));
        }
        if opts.initialized_stale_threshold.is_zero() {
            return Err(crate::error::Error::Internal(
                "initialized_stale_threshold must be > 0".into(),
            ));
        }

        let motors: Arc<RwLock<HashMap<u8, Arc<MotorEntry>>>> =
            Arc::new(RwLock::new(HashMap::new()));
        let inflight_ops: Arc<StdMutex<HashSet<u8>>> = Arc::new(StdMutex::new(HashSet::new()));
        let (events_tx, _) = broadcast::channel(opts.events_capacity);
        let cancel = CancellationToken::new();

        let mut tasks = Vec::new();
        // 心跳广播：只读 / 改 ID 等场景可关掉，避免总线无人 ACK 时的 TX 错误风暴。
        if opts.broadcast_heartbeat {
            tasks.push(tokio::spawn(run_hb_broadcast(
                bus.clone(),
                opts.heartbeat_node_id,
                opts.heartbeat_period,
                cancel.clone(),
            )));
        } else {
            log::info!("heartbeat broadcast disabled (RX-only manager)");
        }
        tasks.push(tokio::spawn(run_discovery(
            bus.clone(),
            opts.heartbeat_node_id,
            motors.clone(),
            inflight_ops.clone(),
            events_tx.clone(),
            opts.sdo_timeout,
            cancel.clone(),
        )));
        tasks.push(tokio::spawn(run_tpdo_listener(
            bus.clone(),
            motors.clone(),
            events_tx.clone(),
            opts.velocity_window,
            cancel.clone(),
        )));
        tasks.push(tokio::spawn(run_liveness_monitor(
            motors.clone(),
            events_tx.clone(),
            opts.motor_heartbeat_period,
            opts.initialized_stale_threshold,
            cancel.clone(),
        )));

        Ok(Self {
            bus,
            opts,
            motors,
            inflight_ops,
            events_tx,
            cancel,
            tasks,
        })
    }

    /// 当前可见的所有电机快照。
    ///
    /// 返回顺序按 node id 升序，方便人类阅读。
    pub fn list(&self) -> Vec<MotorInfo> {
        let motors = self.motors.read().unwrap();
        let mut out: Vec<MotorInfo> = motors
            .values()
            .map(|e| {
                let inner = e.inner.lock().unwrap();
                MotorInfo {
                    node_id: e.node_id,
                    session_epoch: inner.session_epoch,
                    identity: inner.identity.clone(),
                    can_config: inner.can_config.clone(),
                    lifecycle: inner.lifecycle.clone(),
                    online: inner.online,
                    logic: inner.logic.clone(),
                    nmt_state: inner.nmt_state,
                    peak_torque_nm: inner.peak_torque_nm,
                }
            })
            .collect();
        out.sort_by_key(|m| m.node_id);
        out
    }

    /// 订阅事件流。Manager drop 时流自动收到 EOF（`recv()` 返回 `None`）。
    pub fn subscribe_events(&self) -> EventStream {
        EventStream::new(self.events_tx.subscribe())
    }

    /// 强制重读指定节点的 0x1018（+ 0x1008 可选）和设备 CAN 配置。
    ///
    /// - 若节点尚未在 list 中，会先创建一个 `Unknown` 条目。
    /// - 与背景自动 identify / 任何 `initialize()` 互斥（共享 `inflight_ops`），
    ///   重复时返回 `Error::Internal("... already in progress")`。
    /// - CAN 配置失败会记录在 `MotorInfo::can_config`，不会让已成功的身份读取失败。
    pub async fn identify(&self, nid: u8) -> Result<()> {
        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "identify")?;

        let entry = self.get_or_insert_entry(nid);
        let session_epoch = entry.inner.lock().unwrap().session_epoch;
        self.identify_and_cache_in_session(&entry, session_epoch, None)
            .await?;
        Ok(())
    }

    /// 跑完整的初始化流程：NMT PreOp → 配 TPDO1 → 配 0x1016 → NMT Op。
    /// 见 [`crate::cia402::initialize::run_initialize`] 和 `DESIGN.md` §6。
    ///
    /// - 节点必须已 `Identified`（lifecycle == Identified 或 NeedsReinit）。
    ///   `Unknown` 节点会先返回 [`crate::error::Error::NotReady`]，调用方应先
    ///   等 `Identified` 事件或手动 `identify()`。
    /// - 与 identify / 其他 initialize 互斥。
    /// - 失败时 lifecycle 退回 `Identified`（identity 已知）/ `Unknown`，
    ///   电机端可能残留部分 TPDO 配置（v0.1 不主动撤销）；调用方可直接重试。
    /// - 已知型号但 `0x1018:03` 低于 [`known_devices::check_revision`] 要求的，
    ///   在发出任何 SDO 之前就拒绝：旧版对象字典与本驱动不兼容，重试没有意义。
    pub async fn initialize(&self, nid: u8) -> Result<()> {
        let entry = {
            let g = self.motors.read().unwrap();
            g.get(&nid).cloned().ok_or_else(|| {
                crate::error::Error::Internal(format!("nid 0x{nid:02X} not in list yet"))
            })?
        };
        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "initialize")?;

        // 必须已 Identified（或 NeedsReinit）。检查与 identity 快照都放在独占区内，
        // 避免 identify/其它 initialize 在 precheck 与首个 SDO 之间插入。
        let (initial_session_epoch, expected_identity) = {
            let inner = entry.inner.lock().unwrap();
            if matches!(inner.lifecycle, MotorLifecycle::Unknown) {
                return Err(crate::error::Error::NotReady {
                    nid,
                    lifecycle: format!("{:?}", inner.lifecycle),
                });
            }
            let identity = inner.identity.clone().ok_or_else(|| Error::NotReady {
                nid,
                lifecycle: format!("{:?} without identity", inner.lifecycle),
            })?;
            (inner.session_epoch, identity)
        };

        // 固件版本门控：旧版对象字典下，本流程写的 0x2004 / 0x2040 根本不存在，
        // 读 0x6076 / 0x2003:07 也会把定点整数当 REAL32 解出垃圾值。与其让它在
        // configure 深处以 ObjectDoesNotExist 中止并被上层反复重试，不如在这里
        // 一次性拒绝，并把实际 revision 报给用户。
        if let Err(unsupported) = known_devices::check_revision(&expected_identity) {
            return Err(Error::DeviceOperationRejected {
                nid,
                reason: unsupported.to_string(),
            });
        }

        match run_initialize(
            self.bus.as_ref(),
            entry.clone(),
            &self.events_tx,
            &self.opts,
        )
        .await?
        {
            InitializeOutcome::Initialized => Ok(()),
            InitializeOutcome::NeedsCommunicationReset {
                status_word,
                error_code,
                attempts,
            } => {
                // run_initialize 的 rollback 已把 lifecycle 从 Initializing 退回。
                // 再次在同一把锁内确认旧 session/identity，才允许发破坏通信配置的
                // targeted Reset Communication。
                let bootup_revision = {
                    let inner = entry.inner.lock().unwrap();
                    let identity_matches = inner
                        .identity
                        .as_ref()
                        .is_some_and(|actual| same_physical_device(&expected_identity, actual));
                    if inner.session_epoch != initial_session_epoch || !identity_matches {
                        return Err(device_operation_rejected(
                            nid,
                            "device session or identity changed before heartbeat recovery",
                        ));
                    }
                    inner.bootup_revision
                };

                log::warn!(
                    "nid 0x{nid:02X}: sticky heartbeat fault confirmed after {attempts} attempts \
                     (sw=0x{status_word:04X}, error=0x{error_code:04X}); trying one targeted \
                     Reset Communication"
                );
                let reset = nmt::build_nmt_command(NmtCommand::ResetCommunication, nid)?;
                self.bus.send(reset).await?;

                let reset_timeout =
                    (self.opts.motor_heartbeat_period * 4).max(Duration::from_secs(2));
                let new_session_epoch = wait_for_reset_communication_bootup(
                    &entry,
                    initial_session_epoch,
                    bootup_revision,
                    reset_timeout,
                )
                .await?;

                self.identify_and_cache_in_session(
                    &entry,
                    new_session_epoch,
                    Some(&expected_identity),
                )
                .await?;

                match run_initialize(self.bus.as_ref(), entry, &self.events_tx, &self.opts).await? {
                    InitializeOutcome::Initialized => {
                        log::info!(
                            "nid 0x{nid:02X}: initialize recovered after one Reset Communication"
                        );
                        Ok(())
                    }
                    InitializeOutcome::NeedsCommunicationReset {
                        status_word,
                        error_code,
                        attempts,
                    } => Err(device_operation_rejected(
                        nid,
                        format!(
                            "sticky heartbeat fault remained after one Reset Communication \
                             (sw=0x{status_word:04X}, error=0x{error_code:04X}, \
                             attempts={attempts}); refusing another reset"
                        ),
                    )),
                }
            }
        }
    }

    /// 对所有 `Identified` / `NeedsReinit` 节点**并发**跑 `initialize`。
    /// 返回 `(nid, Result)` 列表，按 nid 升序。
    pub async fn initialize_all(&self) -> Vec<(u8, Result<()>)> {
        let targets: Vec<u8> = {
            let g = self.motors.read().unwrap();
            let mut v: Vec<u8> = g
                .iter()
                .filter_map(|(nid, e)| {
                    let inner = e.inner.lock().unwrap();
                    matches!(
                        inner.lifecycle,
                        MotorLifecycle::Identified | MotorLifecycle::NeedsReinit { .. }
                    )
                    .then_some(*nid)
                })
                .collect();
            v.sort();
            v
        };

        let futures = targets.into_iter().map(|nid| async move {
            let r = self.initialize(nid).await;
            (nid, r)
        });
        join_all(futures).await
    }

    /// 切换电机的控制模式（M4）。
    ///
    /// 内部跑 `sequences::build_set_mode_writes` 给出的 CiA402 状态机 ramp：
    /// `(若 Error 则 CW=0x80) → CW=0x06 → 0x6060=mode → CW=0x06 → CW=0x07 → CW=0x0F`，
    /// 每条之间 sleep [`super::sequences::INTER_WRITE_DELAY`]。所有 SDO
    /// 下完后会**轮询 `entry.logic`** 直到等于 `Logic::Enabled(mode)`，最多
    /// 等 `Cia402ManagerOptions::mode_confirm_timeout`（默认 1 s）。
    ///
    /// 错误：
    /// - [`Error::NotReady`]：节点未到 Initialized
    /// - [`Error::ModeConfirmTimeout`]：写完了但等不到 TPDO 反馈
    /// - [`Error::InErrorState`]：等待期间电机报 Fault
    /// - [`Error::Sdo`] / [`Error::Transport`]：底层透传
    pub async fn set_mode(&self, nid: u8, mode: MotorMode) -> Result<()> {
        self.set_mode_with_pre_enable_guard(nid, mode, || Ok(()))
            .await
    }

    /// [`set_mode`](Self::set_mode) with one synchronous guard immediately
    /// before the final `EnableOperation` controlword.  The guard must not
    /// block; an error skips electrical enable and is returned to the caller.
    pub async fn set_mode_with_pre_enable_guard<F>(
        &self,
        nid: u8,
        mode: MotorMode,
        pre_enable: F,
    ) -> Result<()>
    where
        F: FnOnce() -> Result<()>,
    {
        let entry = self.require_initialized(nid)?;
        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "set_mode")?;

        // **不自动清错**（设计取舍，见 DESIGN/对话）：清心跳故障只发生在
        // `initialize()` 里。运行中（程序没退、心跳没断）正常切模式不会有 Fault；
        // 一旦电机带 Fault（缓存 logic 或最新状态字任一显示），这里直接报
        // [`Error::InErrorState`] 让用户决定——手动 `clear_error` + 重新
        // `initialize`，而不是悄悄替用户清掉一个真实故障。
        {
            let inner = entry.inner.lock().unwrap();
            let fault = match &inner.logic {
                Some(Logic::Error { kind, .. }) => Some(*kind),
                _ => inner
                    .measurements
                    .status_word
                    .filter(|sw| super::codec::status_word_has_fault(*sw))
                    .map(|_| crate::types::MotorErrorKind::Other),
            };
            if let Some(kind) = fault {
                return Err(Error::InErrorState(kind));
            }
        }
        // 过了故障检查再缓存目标模式（让 tpdo_listener 能把 OE 翻译成
        // Enabled(mode)）。
        entry.inner.lock().unwrap().target_mode = Some(mode);

        // 已确认无 Fault，所以 enable ramp 不需要 fault-reset 前缀（传 None）。
        let writes = build_set_mode_writes(mode, None);
        log::info!(
            "nid 0x{nid:02X}: set_mode({mode:?}) -> {} SDO writes",
            writes.len()
        );
        let (enable, preparation) = writes
            .split_last()
            .ok_or_else(|| Error::Internal("set_mode produced no SDO writes".into()))?;
        self.sdo_download_sequential(nid, preparation).await?;
        tokio::time::sleep(INTER_WRITE_DELAY).await;
        pre_enable()?;
        self.sdo_download_sequential(nid, std::slice::from_ref(enable))
            .await?;

        wait_for_mode(&entry, mode, self.opts.mode_confirm_timeout).await
    }

    /// 失能（短刹车）。等价 `CW = 0x06`。一次性 SDO 写完即返回，不等 TPDO 反馈。
    ///
    /// 任意模式下都合法。
    pub async fn disable(&self, nid: u8) -> Result<()> {
        let entry = self.require_initialized(nid)?;
        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "disable")?;
        let _ = entry;
        let writes = build_disable_writes();
        self.sdo_download_sequential(nid, &writes).await
    }

    /// 清错（`CW = 0x80`）。一次性 SDO 写完即返回。
    ///
    /// 之后通常要再调一次 `set_mode` 才能继续控制；本调用本身不重新使能电机。
    pub async fn clear_error(&self, nid: u8) -> Result<()> {
        let entry = self.require_initialized(nid)?;
        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "clear_error")?;
        let _ = entry;
        let writes = build_clear_error_writes();
        self.sdo_download_sequential(nid, &writes).await
    }

    /// 按精确 `(vendor_id, product_code)` 应用设备通信设置。
    ///
    /// 安全边界：
    /// - 整个事务与 identify / initialize / 其它 SDO 操作共享同一独占区；
    /// - 只接受本次会话已收到心跳、且当前处于 Pre-Operational 或 Stopped 的节点；
    /// - 写入前强制重读 0x1018 并与调用方给出的身份比较，未知方言一律拒绝；
    /// - 随后强制重读设置对象的形状和值，并按该设备的持久化协议写入目标配置。
    ///
    /// 驱动不会发送 NMT、心跳或复位命令。返回值会说明是否需要用户自行重启。
    #[allow(clippy::too_many_arguments)]
    pub async fn apply_device_settings(
        &self,
        nid: u8,
        expected_vendor_id: u32,
        expected_product_code: u32,
        update: DeviceSettingsUpdate,
    ) -> Result<DeviceSettingsResult> {
        self.validate_settings_target(
            nid,
            expected_vendor_id,
            expected_product_code,
            update,
        )?;
        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "apply_device_settings")?;
        let (entry, session) = self.require_current_session_online(nid, true)?;
        self.verify_identity_tuple_in_session(
            nid,
            &entry,
            session,
            true,
            expected_vendor_id,
            expected_product_code,
        )
        .await?;
        let snapshot = self
            .read_current_settings(nid, expected_vendor_id, expected_product_code)
            .await?;
        self.ensure_device_session(nid, &entry, session, true)?;
        self.reject_online_node_id_collision(nid, update.new_node_id)?;
        self.probe_target_node_id_available(nid, update.new_node_id)
            .await?;
        self.ensure_device_session(nid, &entry, session, true)?;
        let plan = build_device_settings_plan(&snapshot, update)
            .map_err(|reason| device_operation_rejected(nid, reason))?;
        self.sdo_download_session_checked(nid, &entry, session, true, &plan.writes)
            .await?;
        self.cache_applied_settings(nid, &entry, session, update)?;
        log::info!(
            "nid 0x{nid:02X}: device settings transaction complete \
             (changed={}, restart_required={}, persistence_pending={})",
            plan.result.changed,
            plan.result.restart_required,
            plan.result.persistence_pending
        );
        Ok(plan.result)
    }

    /// 兼容旧调用方的仅改 Node-ID API。
    ///
    /// 与旧实现不同，本方法不会猜测对象字典：它只接受已经缓存为受支持精确身份
    /// 的节点，并在同一独占事务中重读 0x1018 和当前设置后，选择正确的对象。
    pub async fn change_node_id(&self, nid: u8, new_id: u8) -> Result<()> {
        let (expected_vendor_id, expected_product_code) =
            self.cached_supported_identity_tuple(nid)?;
        self.validate_node_id_target(
            nid,
            expected_vendor_id,
            expected_product_code,
            new_id,
        )?;

        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "change_node_id")?;
        let (entry, session) = self.require_current_session_online(nid, true)?;
        self.verify_identity_tuple_in_session(
            nid,
            &entry,
            session,
            true,
            expected_vendor_id,
            expected_product_code,
        )
        .await?;
        let snapshot = self
            .read_current_settings(nid, expected_vendor_id, expected_product_code)
            .await?;
        self.ensure_device_session(nid, &entry, session, true)?;
        self.reject_online_node_id_collision(nid, new_id)?;
        self.probe_target_node_id_available(nid, new_id).await?;
        self.ensure_device_session(nid, &entry, session, true)?;
        let update = update_with_new_node_id(&snapshot, new_id);
        let plan = build_device_settings_plan(&snapshot, update)
            .map_err(|reason| device_operation_rejected(nid, reason))?;
        self.sdo_download_session_checked(nid, &entry, session, true, &plan.writes)
            .await?;
        self.cache_applied_settings(nid, &entry, session, update)?;
        Ok(())
    }

    /// 把当前**离线**的电机条目从列表里删掉（批量改 ID 时清掉旧 ID 残留）。
    /// 仍在发心跳的电机会被保留；删掉的若之后又出现心跳会被重新发现。
    pub fn forget_offline(&self) {
        self.motors
            .write()
            .unwrap()
            .retain(|_, e| e.inner.lock().unwrap().online);
    }

    /// 身份严格的位置预设。只允许三条已知 0x2001 电机身份。
    ///
    /// 写控制字 0 后会有界轮询 0x6041，确认进入 Switch On Disabled，随后才写
    /// 位置和 `"pres"`。整个过程不会发送 NMT 或后台重试。
    pub async fn set_position_preset_for(
        &self,
        nid: u8,
        expected_vendor_id: u32,
        expected_product_code: u32,
        pos: f32,
    ) -> Result<()> {
        if !is_object_2001_tuple(expected_vendor_id, expected_product_code) {
            return Err(device_operation_rejected(
                nid,
                "position preset is only supported for an exact known motor identity",
            ));
        }
        let pos = pos.clamp(-0.5, 0.5);
        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "set_position_preset")?;
        let (entry, session) = self.require_current_session_online(nid, false)?;
        self.verify_identity_tuple_in_session(
            nid,
            &entry,
            session,
            false,
            expected_vendor_id,
            expected_product_code,
        )
        .await?;
        let timeout = Some(self.opts.sdo_timeout);
        self.sdo_download_session_checked(
            nid,
            &entry,
            session,
            false,
            &[SdoWrite::u16(0x6040, 0, 0x0000)],
        )
        .await?;

        let deadline = Instant::now() + self.opts.mode_confirm_timeout;
        loop {
            self.ensure_device_session(nid, &entry, session, false)?;
            let status_word =
                sdo::upload_u16(self.bus.as_ref(), nid, 0x6041, 0, timeout).await?;
            self.ensure_device_session(nid, &entry, session, false)?;
            // CiA 402 Switch On Disabled uses mask 0x004F/value 0x0040.
            // Bit 5 (Quick Stop) is not part of this state classification, so
            // both device-reported 0x0250 and 0x0270 must be accepted.
            if status_word & 0x004F == 0x0040 {
                break;
            }
            if Instant::now() >= deadline {
                return Err(Error::DeviceOperationTimeout {
                    nid,
                    operation: "waiting for Switch On Disabled before position preset",
                });
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }

        self.sdo_download_session_checked(
            nid,
            &entry,
            session,
            false,
            &[
                SdoWrite::f32(0x3001, 1, pos),
                SdoWrite::u32(0x3001, 2, 0x7365_7270),
            ],
        )
        .await?;
        log::info!("nid 0x{nid:02X}: position preset -> {pos} Rev");
        Ok(())
    }

    /// 身份严格地按需读一次已知电机当前位置 `0x6064`。
    pub async fn read_position_for(
        &self,
        nid: u8,
        expected_vendor_id: u32,
        expected_product_code: u32,
    ) -> Result<f32> {
        if !is_object_2001_tuple(expected_vendor_id, expected_product_code) {
            return Err(device_operation_rejected(
                nid,
                "position read is only supported for an exact known motor identity",
            ));
        }
        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "read_position")?;
        let (entry, session) = self.require_current_session_online(nid, false)?;
        self.verify_identity_tuple_in_session(
            nid,
            &entry,
            session,
            false,
            expected_vendor_id,
            expected_product_code,
        )
        .await?;
        self.ensure_device_session(nid, &entry, session, false)?;
        let position =
            sdo::upload_f32(self.bus.as_ref(), nid, 0x6064, 0, Some(self.opts.sdo_timeout))
                .await?;
        self.ensure_device_session(nid, &entry, session, false)?;
        Ok(position)
    }

    /// [`set_position_preset_for`](Self::set_position_preset_for) 的显式命名别名。
    pub async fn set_position_preset_for_identity(
        &self,
        nid: u8,
        expected_vendor_id: u32,
        expected_product_code: u32,
        pos: f32,
    ) -> Result<()> {
        self.set_position_preset_for(nid, expected_vendor_id, expected_product_code, pos)
            .await
    }

    /// [`read_position_for`](Self::read_position_for) 的显式命名别名。
    pub async fn read_position_for_identity(
        &self,
        nid: u8,
        expected_vendor_id: u32,
        expected_product_code: u32,
    ) -> Result<f32> {
        self.read_position_for(nid, expected_vendor_id, expected_product_code)
            .await
    }

    /// 兼容旧调用方的位置预设；仍会从缓存取精确身份，并在写前强制重读 0x1018。
    pub async fn set_position_preset(&self, nid: u8, pos: f32) -> Result<()> {
        let (vendor_id, product_code) = self.cached_motor_identity_tuple(nid)?;
        self.set_position_preset_for(nid, vendor_id, product_code, pos)
            .await
    }

    /// 兼容旧调用方的按需位置读取；未知设备不会收到任何位置对象访问。
    pub async fn read_position(&self, nid: u8) -> Result<f32> {
        let (vendor_id, product_code) = self.cached_motor_identity_tuple(nid)?;
        self.read_position_for(nid, vendor_id, product_code)
            .await
    }

    /// 写 `0x6072`（Max Torque），限制**所有模式**下的最大力矩输出。
    ///
    /// `permille` 是峰值力矩 (`0x6076`) 的千分比，内部 clamp 到 `0..=1000`。
    /// 与使能状态无关、任意模式下都可调；只要求电机已 `Initialized`。
    /// 一次性 SDO 写完即返回，不等 TPDO 反馈。
    pub async fn set_max_torque(&self, nid: u8, permille: u16) -> Result<()> {
        let entry = self.require_initialized(nid)?;
        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "set_max_torque")?;
        let _ = entry;
        let value = permille.min(1000);
        sdo::download_u16(
            self.bus.as_ref(),
            nid,
            0x6072,
            0,
            value,
            Some(self.opts.sdo_timeout),
        )
        .await
    }

    /// 写目标值。enum variant 必须和上次 `set_mode` 设的模式匹配，否则返回
    /// [`Error::TargetModeMismatch`]。`MotorTarget::Disable` 任意模式下都行。
    ///
    /// v0.1 已覆盖的 target / mode 组合：
    ///
    /// | target | 当前模式 | 行为 |
    /// |---|---|---|
    /// | `Disable` | 任意 | 写 `0x6040 = 0x06` |
    /// | `Velocity{rev_per_s}` | `ProfileVelocity` | 写 `0x60FF` f32 |
    /// | `Position{rev}` | `ProfilePosition` | CW=0x2F → `0x607A` f32 → CW=0x3F（CSI 上升沿）|
    /// | `Torque{nm}` | `Torque` | 写 `0x6071` i16 = `round(nm / peak * 1000)`（需 `0x6076` 缓存）|
    /// | `Mit{pos,vel,tor,kp,kd}` | `Mit` | 写 `0x2003:01-05`（需 `0x2003:07` factor 缓存）|
    ///
    /// `Torque` / `Mit` 用的 `peak_torque_nm` / `mit_kp_kd_factor` 在
    /// `initialize()` 末尾从 `0x6076` / `0x2003:07` best-effort 读取并缓存；
    /// 缺失时返回 [`Error::Internal`]。
    pub async fn set_target(&self, nid: u8, target: MotorTarget) -> Result<()> {
        let entry = self.require_initialized(nid)?;
        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "set_target")?;
        let ctx = {
            let inner = entry.inner.lock().unwrap();
            SetTargetContext {
                current_mode: inner.target_mode,
                peak_torque_nm: inner.peak_torque_nm,
                mit_kp_kd_factor: inner.mit_kp_kd_factor,
            }
        };
        let writes = build_set_target_writes(&target, ctx)?;
        self.sdo_download_sequential(nid, &writes).await
    }

    /// 拿指定节点最近一次的 [`LiveState`] 快照（无锁，原子读 `ArcSwap`）。
    ///
    /// - 节点不在 list 里 → 返回 `LiveState::empty(now)`，所有字段都是空。
    /// - **不返回 `Result`**：v0.1 故意做成最轻量的读取，给 UI / 监控用。
    ///
    /// 这是给"我已经知道有这个 nid 想看看现状"的用法。若想在节点出现 / 消失
    /// 时被通知，订阅 [`Cia402Manager::subscribe_events`]。
    pub fn status(&self, nid: u8) -> LiveState {
        let entry = self.motors.read().unwrap().get(&nid).cloned();
        match entry {
            Some(e) => (*e.snapshot.load_full()).clone(),
            None => LiveState::empty(Instant::now()),
        }
    }

    /// 订阅指定节点的 `LiveState` 流。每路订阅一个独立 mpsc，容量由
    /// `opts.capacity` 指定。channel 满时按 [`super::subscribe::OverflowPolicy`]
    /// 处理（v0.1 只有 `Lagged`）。
    ///
    /// 返回的 [`StatusStream`] 用 `recv().await` 取下一条；详见
    /// [`crate::cia402::subscribe`]。
    ///
    /// 节点必须**已经被 list() 看到**（至少收到过一次 HB），否则返回
    /// [`Error::UnknownNode`]。不要求 `Initialized`：Unknown 阶段订阅也行，
    /// 你会先看到 connection 字段在变，等 TPDO 配好后才有 measurements / logic。
    pub fn subscribe_status(&self, nid: u8, opts: StreamOptions) -> Result<StatusStream> {
        if opts.capacity == 0 {
            return Err(Error::Internal("subscribe_status: capacity must be > 0".into()));
        }
        let entry = self
            .motors
            .read()
            .unwrap()
            .get(&nid)
            .cloned()
            .ok_or(Error::UnknownNode(nid))?;
        let (sub, stream) = Subscriber::new(&opts);
        // 先把当前 snapshot 喂进队列一条，让用户立刻拿到"初始状态"。
        // 这里直接 push 一次再挂入列表 —— Sub 还没人能并发访问，安全。
        let initial = (*entry.snapshot.load_full()).clone();
        let mut sub = sub;
        let _ = sub.push(&initial);
        entry.add_subscriber(sub);
        Ok(stream)
    }

    /// 暴露当前选项，方便上层日志 / UI 显示。
    pub fn options(&self) -> &Cia402ManagerOptions {
        &self.opts
    }

    /// 共享底层 CAN 总线句柄。
    ///
    /// 上层"机器人应用"（例如以固定频率向多电机广播 RPDO 的底盘）需要在
    /// manager 已经接管发现 / TPDO 监听 / 心跳广播的同一条总线上**直接发原始
    /// 帧**。一个物理适配器（如 gs_usb）不能被打开两次，所以这里把 manager 持有的
    /// `Arc<dyn CanBus>` clone 出去共享。`send` 是 `&self`，与后台 task 并发安全。
    pub fn bus(&self) -> Arc<dyn CanBus> {
        self.bus.clone()
    }

    /// 拿到 / 建立指定 nid 的 entry。
    fn get_or_insert_entry(&self, nid: u8) -> Arc<MotorEntry> {
        let mut g = self.motors.write().unwrap();
        g.entry(nid)
            .or_insert_with(|| Arc::new(MotorEntry::new(nid)))
            .clone()
    }

    /// 已持有该 nid 的 inflight guard 时，在指定 session 内完整读取并缓存身份及
    /// CAN 配置。恢复路径可额外要求与 Reset Communication 前是同一颗设备。
    async fn identify_and_cache_in_session(
        &self,
        entry: &Arc<MotorEntry>,
        expected_epoch: u64,
        expected_device: Option<&MotorIdentity>,
    ) -> Result<MotorIdentity> {
        let nid = entry.node_id;
        if entry.inner.lock().unwrap().session_epoch != expected_epoch {
            return Err(device_operation_rejected(
                nid,
                "device session changed before identity read",
            ));
        }

        let identity = identify_once(self.bus.as_ref(), nid, self.opts.sdo_timeout).await?;
        if entry.inner.lock().unwrap().session_epoch != expected_epoch {
            return Err(device_operation_rejected(
                nid,
                "device session changed during identity read",
            ));
        }

        if let Some(expected) = expected_device {
            if !same_physical_device(expected, &identity) {
                return Err(device_operation_rejected(
                    nid,
                    format!(
                        "identity changed across Reset Communication: expected {}, observed {}",
                        stable_identity_summary(expected),
                        stable_identity_summary(&identity),
                    ),
                ));
            }
        }

        // 身份不匹配时必须在这里之前退出，不能按旧设备方言去探测替换设备。
        if !begin_device_can_config_read(entry, expected_epoch, &identity) {
            return Err(device_operation_rejected(
                nid,
                "device session changed before configuration read",
            ));
        }
        let can_config =
            read_device_can_config(self.bus.as_ref(), &identity, self.opts.sdo_timeout).await;
        if let DeviceCanConfigStatus::ReadFailed { reason } = &can_config {
            log::warn!("read CAN config for nid 0x{nid:02X} failed: {reason}");
        }
        if !finish_device_can_config_read(entry, expected_epoch, can_config) {
            return Err(device_operation_rejected(
                nid,
                "device session changed during configuration read",
            ));
        }

        let inner = entry.inner.lock().unwrap();
        if inner.session_epoch != expected_epoch {
            return Err(device_operation_rejected(
                nid,
                "device session changed before identity result was published",
            ));
        }
        let _ = self.events_tx.send(Cia402Event::Identified {
            nid,
            identity: identity.clone(),
        });
        drop(inner);
        Ok(identity)
    }

    fn validate_settings_target(
        &self,
        nid: u8,
        vendor_id: u32,
        product_code: u32,
        update: DeviceSettingsUpdate,
    ) -> Result<()> {
        if dialect_for_tuple(vendor_id, product_code).is_none() {
            return Err(device_operation_rejected(
                nid,
                "the exact vendor/product tuple has no supported settings dialect",
            ));
        }
        self.validate_node_id_target(nid, vendor_id, product_code, update.new_node_id)?;
        if update.nominal_bitrate != 1_000_000 {
            return Err(device_operation_rejected(
                nid,
                format!(
                    "nominal bitrate must be 1000000, got {}",
                    update.nominal_bitrate
                ),
            ));
        }
        if let Some(data_bitrate) = update.data_bitrate {
            if !matches!(
                data_bitrate,
                1_000_000 | 2_000_000 | 4_000_000 | 5_000_000
            ) {
                return Err(device_operation_rejected(
                    nid,
                    format!(
                        "data bitrate must be 1000000, 2000000, 4000000, or 5000000, \
                         got {data_bitrate}"
                    ),
                ));
            }
        }
        if update.data_bitrate.is_some() != update.transmit_pdo_brs.is_some() {
            return Err(device_operation_rejected(
                nid,
                "data bitrate and TPDO BRS must either both be present or both be absent",
            ));
        }
        if is_object_2001_tuple(vendor_id, product_code) && update.data_bitrate.is_none() {
            return Err(device_operation_rejected(
                nid,
                "the exact motor dialect requires CAN FD data bitrate and TPDO BRS values",
            ));
        }
        Ok(())
    }

    fn validate_node_id_target(
        &self,
        nid: u8,
        vendor_id: u32,
        product_code: u32,
        new_node_id: u8,
    ) -> Result<()> {
        if !(1..=126).contains(&new_node_id) || matches!(new_node_id, 100 | 101 | 127) {
            return Err(device_operation_rejected(
                nid,
                format!(
                    "new node ID must be 1..=126 and must not be 100, 101, or 127; \
                     got {new_node_id}"
                ),
            ));
        }
        if new_node_id == self.opts.heartbeat_node_id {
            return Err(device_operation_rejected(
                nid,
                format!(
                    "new node ID {new_node_id} conflicts with the manager heartbeat node ID"
                ),
            ));
        }
        validate_role_node_id(vendor_id, product_code, new_node_id)
            .map_err(|reason| device_operation_rejected(nid, reason))
    }

    fn reject_online_node_id_collision(&self, nid: u8, new_node_id: u8) -> Result<()> {
        if new_node_id == nid {
            return Ok(());
        }
        let conflict = self
            .motors
            .read()
            .unwrap()
            .get(&new_node_id)
            .is_some_and(|entry| entry.inner.lock().unwrap().online);
        if conflict {
            return Err(device_operation_rejected(
                nid,
                format!("node ID {new_node_id} is already online"),
            ));
        }
        Ok(())
    }

    /// 尚未发首帧心跳的节点可能不在 map 中；对目标 ID 做一次按需 0x1018 probe。
    ///
    /// 只有本地 SDO client 的明确 ProtocolTimeout 能证明“没有任何响应”。任何
    /// 成功响应、server abort、其它协议错误或 transport 错误都保守拒绝。
    async fn probe_target_node_id_available(&self, nid: u8, new_node_id: u8) -> Result<()> {
        if new_node_id == nid {
            return Ok(());
        }
        match sdo::upload_u32(
            self.bus.as_ref(),
            new_node_id,
            0x1018,
            1,
            Some(self.opts.sdo_timeout),
        )
        .await
        {
            Ok(_) => Err(device_operation_rejected(
                nid,
                format!("target node ID {new_node_id} responded to 0x1018 and is occupied"),
            )),
            Err(Error::Sdo(AsyncSdoError::Sdo(SdoError::ClientAborted(
                SdoAbortCode::ProtocolTimeout,
            )))) => Ok(()),
            Err(Error::Sdo(AsyncSdoError::Sdo(SdoError::ServerAborted(code)))) => {
                Err(device_operation_rejected(
                    nid,
                    format!(
                        "target node ID {new_node_id} returned SDO abort {code} and is occupied"
                    ),
                ))
            }
            Err(error) => Err(device_operation_rejected(
                nid,
                format!(
                    "could not safely prove target node ID {new_node_id} is free: {error}"
                ),
            )),
        }
    }

    fn require_current_session_online(
        &self,
        nid: u8,
        require_configuration_nmt_state: bool,
    ) -> Result<(Arc<MotorEntry>, DeviceSessionToken)> {
        let entry = self
            .motors
            .read()
            .unwrap()
            .get(&nid)
            .cloned()
            .ok_or_else(|| {
                device_operation_rejected(nid, "node has not sent a heartbeat in this session")
            })?;
        let inner = entry.inner.lock().unwrap();
        if !inner.online || inner.last_heartbeat.is_none() || inner.nmt_state.is_none() {
            return Err(device_operation_rejected(
                nid,
                "node has not sent a current-session heartbeat or is offline",
            ));
        }
        if require_configuration_nmt_state
            && !matches!(
                inner.nmt_state,
                Some(NmtState::PreOperational | NmtState::Stopped)
            )
        {
            return Err(device_operation_rejected(
                nid,
                format!(
                    "communication settings require Pre-Operational or Stopped, got {:?}",
                    inner.nmt_state
                ),
            ));
        }
        let token = DeviceSessionToken {
            session_epoch: inner.session_epoch,
            nmt_revision: inner.nmt_revision,
            nmt_state: inner.nmt_state.expect("checked above"),
        };
        drop(inner);
        Ok((entry, token))
    }

    fn ensure_device_session(
        &self,
        nid: u8,
        entry: &MotorEntry,
        expected: DeviceSessionToken,
        require_configuration_nmt_state: bool,
    ) -> Result<()> {
        let inner = entry.inner.lock().unwrap();
        if !inner.online || inner.last_heartbeat.is_none() {
            return Err(device_operation_rejected(
                nid,
                "device went offline during the operation",
            ));
        }
        if inner.session_epoch != expected.session_epoch {
            return Err(device_operation_rejected(
                nid,
                "device session changed during the operation",
            ));
        }
        if inner.nmt_revision != expected.nmt_revision
            || inner.nmt_state != Some(expected.nmt_state)
        {
            return Err(device_operation_rejected(
                nid,
                "device NMT state changed during the operation",
            ));
        }
        if require_configuration_nmt_state
            && !matches!(
                inner.nmt_state,
                Some(NmtState::PreOperational | NmtState::Stopped)
            )
        {
            return Err(device_operation_rejected(
                nid,
                "device left Pre-Operational/Stopped during the settings transaction",
            ));
        }
        Ok(())
    }

    async fn verify_identity_tuple_in_session(
        &self,
        nid: u8,
        entry: &MotorEntry,
        session: DeviceSessionToken,
        require_configuration_nmt_state: bool,
        expected_vendor_id: u32,
        expected_product_code: u32,
    ) -> Result<()> {
        let timeout = Some(self.opts.sdo_timeout);
        self.ensure_device_session(
            nid,
            entry,
            session,
            require_configuration_nmt_state,
        )?;
        let actual_vendor_id =
            sdo::upload_u32(self.bus.as_ref(), nid, 0x1018, 1, timeout).await?;
        self.ensure_device_session(
            nid,
            entry,
            session,
            require_configuration_nmt_state,
        )?;
        let actual_product_code =
            sdo::upload_u32(self.bus.as_ref(), nid, 0x1018, 2, timeout).await?;
        self.ensure_device_session(
            nid,
            entry,
            session,
            require_configuration_nmt_state,
        )?;
        if actual_vendor_id != expected_vendor_id || actual_product_code != expected_product_code {
            return Err(device_operation_rejected(
                nid,
                format!(
                    "identity changed: expected 0x{expected_vendor_id:08X}/\
                     0x{expected_product_code:08X}, got 0x{actual_vendor_id:08X}/\
                     0x{actual_product_code:08X}"
                ),
            ));
        }
        Ok(())
    }

    async fn read_current_settings(
        &self,
        nid: u8,
        vendor_id: u32,
        product_code: u32,
    ) -> Result<DeviceSettingsSnapshot> {
        read_device_settings_snapshot(
            self.bus.as_ref(),
            nid,
            vendor_id,
            product_code,
            self.opts.sdo_timeout,
        )
        .await
        .map_err(|reason| device_operation_rejected(nid, reason))
    }

    fn cache_applied_settings(
        &self,
        nid: u8,
        entry: &MotorEntry,
        session: DeviceSessionToken,
        update: DeviceSettingsUpdate,
    ) -> Result<()> {
        let mut inner = entry.inner.lock().unwrap();
        if !inner.online
            || inner.session_epoch != session.session_epoch
            || inner.nmt_revision != session.nmt_revision
            || inner.nmt_state != Some(session.nmt_state)
        {
            return Err(device_operation_rejected(
                nid,
                "device session changed before applied settings could be cached",
            ));
        }
        inner.can_config = DeviceCanConfigStatus::Available(DeviceCanConfig {
            stored_node_id: update.new_node_id,
            nominal_bitrate: update.nominal_bitrate,
            data_bitrate: update.data_bitrate,
            transmit_pdo_brs: update.transmit_pdo_brs,
        });
        Ok(())
    }

    fn cached_supported_identity_tuple(&self, nid: u8) -> Result<(u32, u32)> {
        let entry = self
            .motors
            .read()
            .unwrap()
            .get(&nid)
            .cloned()
            .ok_or_else(|| device_operation_rejected(nid, "node is not known"))?;
        let identity = entry
            .inner
            .lock()
            .unwrap()
            .identity
            .clone()
            .ok_or_else(|| device_operation_rejected(nid, "exact identity is not available"))?;
        if dialect_for_tuple(identity.vendor_id, identity.product_code).is_none() {
            return Err(device_operation_rejected(
                nid,
                "cached exact identity has no supported settings dialect",
            ));
        }
        Ok((identity.vendor_id, identity.product_code))
    }

    fn cached_motor_identity_tuple(&self, nid: u8) -> Result<(u32, u32)> {
        let (vendor_id, product_code) = self.cached_supported_identity_tuple(nid)?;
        if !is_object_2001_tuple(vendor_id, product_code) {
            return Err(device_operation_rejected(
                nid,
                "position operations require an exact known motor identity",
            ));
        }
        Ok((vendor_id, product_code))
    }

    /// 拿到 lifecycle == Initialized 的 entry，否则返回 NotReady。
    fn require_initialized(&self, nid: u8) -> Result<Arc<MotorEntry>> {
        let g = self.motors.read().unwrap();
        let entry = g
            .get(&nid)
            .cloned()
            .ok_or_else(|| Error::NotReady {
                nid,
                lifecycle: "not in list".into(),
            })?;
        drop(g);
        let inner = entry.inner.lock().unwrap();
        if !matches!(inner.lifecycle, MotorLifecycle::Initialized) {
            return Err(Error::NotReady {
                nid,
                lifecycle: format!("{:?}", inner.lifecycle),
            });
        }
        drop(inner);
        Ok(entry)
    }

    /// 顺序 SDO 下发；每条之间 sleep [`INTER_WRITE_DELAY`] 给电机 settle。
    async fn sdo_download_sequential(&self, nid: u8, writes: &[SdoWrite]) -> Result<()> {
        let sdo_timeout = Some(self.opts.sdo_timeout);
        let last = writes.len().saturating_sub(1);
        for (i, w) in writes.iter().enumerate() {
            sdo::download(
                self.bus.as_ref(),
                nid,
                w.index,
                w.subindex,
                &w.data,
                sdo_timeout,
            )
            .await?;
            if i != last {
                tokio::time::sleep(INTER_WRITE_DELAY).await;
            }
        }
        Ok(())
    }

    /// 顺序写设置对象，并在每条 SDO 前后核对同一设备 session / NMT revision。
    async fn sdo_download_session_checked(
        &self,
        nid: u8,
        entry: &MotorEntry,
        session: DeviceSessionToken,
        require_configuration_nmt_state: bool,
        writes: &[SdoWrite],
    ) -> Result<()> {
        let sdo_timeout = Some(self.opts.sdo_timeout);
        let last = writes.len().saturating_sub(1);
        for (i, write) in writes.iter().enumerate() {
            self.ensure_device_session(
                nid,
                entry,
                session,
                require_configuration_nmt_state,
            )?;
            sdo::download(
                self.bus.as_ref(),
                nid,
                write.index,
                write.subindex,
                &write.data,
                sdo_timeout,
            )
            .await?;
            self.ensure_device_session(
                nid,
                entry,
                session,
                require_configuration_nmt_state,
            )?;
            if i != last {
                tokio::time::sleep(INTER_WRITE_DELAY).await;
            }
        }
        self.ensure_device_session(
            nid,
            entry,
            session,
            require_configuration_nmt_state,
        )
    }
}

fn update_with_new_node_id(
    current: &DeviceSettingsSnapshot,
    new_node_id: u8,
) -> DeviceSettingsUpdate {
    DeviceSettingsUpdate {
        new_node_id,
        nominal_bitrate: current.config.nominal_bitrate,
        data_bitrate: current.config.data_bitrate,
        transmit_pdo_brs: current.config.transmit_pdo_brs,
    }
}

fn same_physical_device(expected: &MotorIdentity, observed: &MotorIdentity) -> bool {
    expected.node_id == observed.node_id
        && expected.vendor_id == observed.vendor_id
        && expected.product_code == observed.product_code
        && expected.revision_number == observed.revision_number
        && expected.serial_number == observed.serial_number
}

fn stable_identity_summary(identity: &MotorIdentity) -> String {
    format!(
        "nid=0x{:02X}, vendor=0x{:08X}, product=0x{:08X}, revision=0x{:08X}, serial=0x{:08X}",
        identity.node_id,
        identity.vendor_id,
        identity.product_code,
        identity.revision_number,
        identity.serial_number,
    )
}

/// Reset Communication 的完成 fence：必须看见一帧新的 Boot-Up（不能仅靠
/// offline 也会推进的 session_epoch），并等该新 session 进入 Pre-Operational。
async fn wait_for_reset_communication_bootup(
    entry: &Arc<MotorEntry>,
    previous_session_epoch: u64,
    previous_bootup_revision: u64,
    timeout: Duration,
) -> Result<u64> {
    let deadline = Instant::now() + timeout;
    loop {
        let ready = {
            let inner = entry.inner.lock().unwrap();
            (inner.bootup_revision != previous_bootup_revision
                && inner.session_epoch != previous_session_epoch
                && inner.online
                && inner.nmt_state == Some(NmtState::PreOperational))
            .then_some(inner.session_epoch)
        };
        if let Some(session_epoch) = ready {
            return Ok(session_epoch);
        }
        if Instant::now() >= deadline {
            let inner = entry.inner.lock().unwrap();
            log::warn!(
                "nid 0x{:02X}: timed out waiting for Reset Communication Boot-Up \
                 (session {} -> {}, bootup revision {} -> {}, NMT={:?}, online={})",
                entry.node_id,
                previous_session_epoch,
                inner.session_epoch,
                previous_bootup_revision,
                inner.bootup_revision,
                inner.nmt_state,
                inner.online,
            );
            return Err(Error::DeviceOperationTimeout {
                nid: entry.node_id,
                operation: "waiting for Reset Communication Boot-Up/Pre-Operational",
            });
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

fn device_operation_rejected(nid: u8, reason: impl Into<String>) -> Error {
    Error::DeviceOperationRejected {
        nid,
        reason: reason.into(),
    }
}

/// 轮询 [`MotorEntry::inner.logic`]（由 [`super::tpdo_listener`] 在每帧 TPDO2
/// 到达时刷新）直到等于 `Logic::Enabled(target)` 或超时 / 进入 Error。
async fn wait_for_mode(
    entry: &Arc<MotorEntry>,
    target: MotorMode,
    timeout: Duration,
) -> Result<()> {
    let deadline = Instant::now() + timeout;
    let poll = Duration::from_millis(10);
    loop {
        let snapshot = entry.inner.lock().unwrap().logic.clone();
        match snapshot {
            Some(Logic::Enabled(m)) if m == target => return Ok(()),
            Some(Logic::Error { kind, .. }) => return Err(Error::InErrorState(kind)),
            _ => {}
        }
        if Instant::now() >= deadline {
            return Err(Error::ModeConfirmTimeout);
        }
        tokio::time::sleep(poll).await;
    }
}

/// 进入临界区前 `inflight_ops.insert(nid)`；drop 时移除。
struct InflightGuard<'a> {
    set: &'a StdMutex<HashSet<u8>>,
    nid: u8,
}

impl<'a> InflightGuard<'a> {
    fn acquire(set: &'a StdMutex<HashSet<u8>>, nid: u8, op_name: &str) -> Result<Self> {
        let mut g = set.lock().unwrap();
        if !g.insert(nid) {
            return Err(crate::error::Error::Internal(format!(
                "nid 0x{nid:02X}: another exclusive op already in progress \
                 (requested: {op_name})"
            )));
        }
        Ok(Self { set, nid })
    }
}

impl Drop for InflightGuard<'_> {
    fn drop(&mut self) {
        self.set.lock().unwrap().remove(&self.nid);
    }
}

impl Drop for Cia402Manager {
    fn drop(&mut self) {
        self.cancel.cancel();
        for h in self.tasks.drain(..) {
            h.abort();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::{HashMap, VecDeque};
    use std::sync::Mutex;

    use async_trait::async_trait;
    use can_transport::{
        CanCapabilities, CanFilter, CanFrame, CanId, CanIoError, CanRx,
    };
    use tokio::sync::mpsc;

    const MOTOR_VENDOR: u32 = 0x4859_444C;
    const MOTOR_PRODUCT: u32 = 0xAAAA_0001;
    type SdoKey = (u8, u16, u8);
    type ScriptedValue = (SdoKey, Vec<Vec<u8>>);
    type SendHook = Arc<dyn Fn(u8, &RecordedSdo) + Send + Sync>;

    fn recovery_identity(product_name: Option<&str>) -> MotorIdentity {
        MotorIdentity {
            node_id: 1,
            vendor_id: MOTOR_VENDOR,
            product_code: MOTOR_PRODUCT,
            revision_number: 3,
            serial_number: 4,
            product_name: product_name.map(str::to_owned),
        }
    }

    #[test]
    fn reset_recovery_identity_uses_only_mandatory_1018_fields() {
        let expected = recovery_identity(Some("cached name"));
        let observed_without_optional_name = recovery_identity(None);
        assert!(same_physical_device(
            &expected,
            &observed_without_optional_name
        ));

        let mut changed = observed_without_optional_name.clone();
        changed.serial_number += 1;
        assert!(!same_physical_device(&expected, &changed));
        changed = observed_without_optional_name.clone();
        changed.revision_number += 1;
        assert!(!same_physical_device(&expected, &changed));
        changed = observed_without_optional_name.clone();
        changed.product_code += 1;
        assert!(!same_physical_device(&expected, &changed));
        changed = observed_without_optional_name.clone();
        changed.vendor_id += 1;
        assert!(!same_physical_device(&expected, &changed));
        changed = observed_without_optional_name;
        changed.node_id += 1;
        assert!(!same_physical_device(&expected, &changed));
    }

    #[tokio::test]
    async fn reset_wait_requires_a_new_bootup_not_only_a_new_session_epoch() {
        let entry = Arc::new(MotorEntry::new(1));
        {
            let mut inner = entry.inner.lock().unwrap();
            // 模拟 offline invalidation 后又收到普通 Pre-Operational：epoch 已变，
            // 但没有 Boot-Up，不能证明 Reset Communication 被执行。
            inner.session_epoch = 4;
            inner.bootup_revision = 2;
            inner.online = true;
            inner.nmt_state = Some(NmtState::PreOperational);
        }
        let error = wait_for_reset_communication_bootup(
            &entry,
            3,
            2,
            Duration::from_millis(15),
        )
        .await
        .unwrap_err();
        assert!(matches!(error, Error::DeviceOperationTimeout { .. }));

        // Boot-Up 即使已经快速过渡为 Pre-Operational，revision 仍会保留证据。
        {
            let mut inner = entry.inner.lock().unwrap();
            inner.session_epoch = 5;
            inner.bootup_revision = 3;
            inner.nmt_state = Some(NmtState::PreOperational);
        }
        assert_eq!(
            wait_for_reset_communication_bootup(
                &entry,
                4,
                2,
                Duration::from_millis(20),
            )
            .await
            .unwrap(),
            5
        );
    }

    #[derive(Debug, Clone, PartialEq, Eq)]
    enum RecordedSdo {
        Upload {
            index: u16,
            subindex: u8,
        },
        Download {
            index: u16,
            subindex: u8,
            data: Vec<u8>,
        },
    }

    struct ScriptedSettingsBus {
        values: Mutex<HashMap<SdoKey, VecDeque<Vec<u8>>>>,
        requests: Mutex<Vec<RecordedSdo>>,
        response_tx: Mutex<Option<mpsc::UnboundedSender<CanFrame>>>,
        send_hook: Mutex<Option<SendHook>>,
    }

    impl ScriptedSettingsBus {
        fn new<const N: usize>(values: [ScriptedValue; N]) -> Self {
            Self {
                values: Mutex::new(
                    values
                        .into_iter()
                        .map(|(key, values)| (key, values.into()))
                        .collect(),
                ),
                requests: Mutex::new(Vec::new()),
                response_tx: Mutex::new(None),
                send_hook: Mutex::new(None),
            }
        }

        fn requests(&self) -> Vec<RecordedSdo> {
            self.requests.lock().unwrap().clone()
        }

        fn set_send_hook(&self, hook: SendHook) {
            *self.send_hook.lock().unwrap() = Some(hook);
        }

        fn record(&self, nid: u8, request: RecordedSdo) {
            self.requests.lock().unwrap().push(request.clone());
            let hook = self.send_hook.lock().unwrap().clone();
            if let Some(hook) = hook {
                hook(nid, &request);
            }
        }
    }

    #[async_trait]
    impl CanBus for ScriptedSettingsBus {
        async fn send(&self, frame: CanFrame) -> std::result::Result<(), CanIoError> {
            let request = frame.data();
            if request.len() != 8 {
                return Err(CanIoError::backend(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    "test bus expected an 8-byte expedited SDO request",
                )));
            }
            let nid = match frame.id() {
                CanId::Standard(id) if (0x601..=0x67F).contains(&id) => (id - 0x600) as u8,
                _ => {
                    return Err(CanIoError::backend(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        "test bus received an invalid RSDO COB-ID",
                    )))
                }
            };
            let index = u16::from_le_bytes([request[1], request[2]]);
            let subindex = request[3];
            let mut response = [0u8; 8];
            response[1] = request[1];
            response[2] = request[2];
            response[3] = subindex;

            if request[0] == 0x40 {
                self.record(nid, RecordedSdo::Upload { index, subindex });
                let value = {
                    let mut values = self.values.lock().unwrap();
                    let queue = values.get_mut(&(nid, index, subindex)).ok_or_else(|| {
                        CanIoError::backend(std::io::Error::new(
                            std::io::ErrorKind::InvalidData,
                            format!(
                                "unexpected test upload nid {nid} 0x{index:04X}:{subindex:02X}"
                            ),
                        ))
                    })?;
                    if queue.is_empty() {
                        // Deliberately silent node: drive the SDO client to ProtocolTimeout.
                        return Ok(());
                    }
                    if queue.len() > 1 {
                        queue.pop_front().unwrap()
                    } else {
                        queue.front().cloned().unwrap()
                    }
                };
                if !(1..=4).contains(&value.len()) {
                    return Err(CanIoError::backend(std::io::Error::new(
                        std::io::ErrorKind::InvalidData,
                        "test upload value must contain 1..=4 bytes",
                    )));
                }
                response[0] = 0x43 | (((4 - value.len()) as u8) << 2);
                response[4..4 + value.len()].copy_from_slice(&value);
            } else if request[0] & 0xE0 == 0x20 && request[0] & 0x02 != 0 {
                let unused = ((request[0] >> 2) & 0x03) as usize;
                let len = 4 - unused;
                self.record(
                    nid,
                    RecordedSdo::Download {
                        index,
                        subindex,
                        data: request[4..4 + len].to_vec(),
                    },
                );
                response[0] = 0x60;
            } else {
                return Err(CanIoError::backend(std::io::Error::new(
                    std::io::ErrorKind::InvalidData,
                    format!("unexpected SDO command 0x{:02X}", request[0]),
                )));
            }

            let response = CanFrame::new_data(0x580u16 + nid as u16, &response)?;
            self.response_tx
                .lock()
                .unwrap()
                .as_ref()
                .ok_or(CanIoError::Disconnected)?
                .send(response)
                .map_err(|_| CanIoError::Disconnected)
        }

        async fn subscribe(
            &self,
            _filter: CanFilter,
        ) -> std::result::Result<Box<dyn CanRx>, CanIoError> {
            let (tx, rx) = mpsc::unbounded_channel();
            *self.response_tx.lock().unwrap() = Some(tx);
            Ok(Box::new(ScriptedSettingsRx { rx }))
        }

        fn capabilities(&self) -> CanCapabilities {
            CanCapabilities {
                fd: true,
                max_dlen: 64,
            }
        }
    }

    struct ScriptedSettingsRx {
        rx: mpsc::UnboundedReceiver<CanFrame>,
    }

    #[async_trait]
    impl CanRx for ScriptedSettingsRx {
        async fn recv(&mut self) -> std::result::Result<CanFrame, CanIoError> {
            self.rx.recv().await.ok_or(CanIoError::Disconnected)
        }

        fn try_recv(&mut self) -> std::result::Result<Option<CanFrame>, CanIoError> {
            match self.rx.try_recv() {
                Ok(frame) => Ok(Some(frame)),
                Err(mpsc::error::TryRecvError::Empty) => Ok(None),
                Err(mpsc::error::TryRecvError::Disconnected) => Err(CanIoError::Disconnected),
            }
        }
    }

    fn test_manager(bus: Arc<dyn CanBus>) -> Cia402Manager {
        let (events_tx, _) = broadcast::channel(4);
        let opts = Cia402ManagerOptions {
            broadcast_heartbeat: false,
            sdo_timeout: Duration::from_millis(50),
            mode_confirm_timeout: Duration::from_millis(50),
            ..Default::default()
        };
        Cia402Manager {
            bus,
            opts,
            motors: Arc::new(RwLock::new(HashMap::new())),
            inflight_ops: Arc::new(StdMutex::new(HashSet::new())),
            events_tx,
            cancel: CancellationToken::new(),
            tasks: Vec::new(),
        }
    }

    fn add_online_entry(
        manager: &Cia402Manager,
        nid: u8,
        vendor_id: u32,
        product_code: u32,
        nmt_state: NmtState,
    ) {
        let entry = Arc::new(MotorEntry::new(nid));
        {
            let mut inner = entry.inner.lock().unwrap();
            inner.identity = Some(crate::types::MotorIdentity {
                node_id: nid,
                vendor_id,
                product_code,
                revision_number: 0,
                serial_number: 0,
                product_name: None,
            });
            inner.lifecycle = MotorLifecycle::Identified;
            inner.online = true;
            inner.last_heartbeat = Some(Instant::now());
            inner.nmt_state = Some(nmt_state);
        }
        manager.motors.write().unwrap().insert(nid, entry);
    }

    fn entry(manager: &Cia402Manager, nid: u8) -> Arc<MotorEntry> {
        manager.motors.read().unwrap()[&nid].clone()
    }

    fn motor_settings_values(
        node_id: u8,
        data_bitrate: u32,
        brs: bool,
    ) -> [ScriptedValue; 8] {
        [
            ((1, 0x1018, 1), vec![MOTOR_VENDOR.to_le_bytes().to_vec()]),
            ((1, 0x1018, 2), vec![MOTOR_PRODUCT.to_le_bytes().to_vec()]),
            ((1, 0x2001, 0), vec![vec![4]]),
            ((1, 0x2001, 1), vec![vec![node_id]]),
            (
                (1, 0x2001, 2),
                vec![1_000_000u32.to_le_bytes().to_vec()],
            ),
            (
                (1, 0x2001, 3),
                vec![data_bitrate.to_le_bytes().to_vec()],
            ),
            ((1, 0x2001, 4), vec![vec![u8::from(brs)]]),
            ((42, 0x1018, 1), vec![]),
        ]
    }

    #[tokio::test]
    async fn initialize_rejects_unsupported_revision_before_touching_the_bus() {
        let bus = Arc::new(ScriptedSettingsBus::new(motor_settings_values(
            1, 5_000_000, false,
        )));
        let manager = test_manager(bus.clone());
        // add_online_entry 建的 entry 是 revision 0，低于该 vendor 要求的 8。
        add_online_entry(
            &manager,
            1,
            MOTOR_VENDOR,
            MOTOR_PRODUCT,
            NmtState::PreOperational,
        );

        let error = manager
            .initialize(1)
            .await
            .expect_err("an outdated object dictionary must not be initialized");
        let text = error.to_string();
        assert!(text.contains("revision 0"), "{text}");
        assert!(text.contains(">= 8"), "{text}");
        // 门控必须发生在任何 NMT/SDO 之前：旧固件重试再多次也不会变得可用。
        assert!(
            bus.requests().is_empty(),
            "rejected node must not receive any traffic, got {:?}",
            bus.requests()
        );
        assert_eq!(
            entry(&manager, 1).inner.lock().unwrap().lifecycle,
            MotorLifecycle::Identified,
            "a rejected initialize must leave the lifecycle untouched"
        );
    }

    #[tokio::test]
    async fn settings_transaction_rechecks_identity_and_writes_motor_fields_in_order() {
        let bus = Arc::new(ScriptedSettingsBus::new(motor_settings_values(
            1, 5_000_000, false,
        )));
        let manager = test_manager(bus.clone());
        add_online_entry(
            &manager,
            1,
            MOTOR_VENDOR,
            MOTOR_PRODUCT,
            NmtState::PreOperational,
        );

        let result = manager
            .apply_device_settings(
                1,
                MOTOR_VENDOR,
                MOTOR_PRODUCT,
                DeviceSettingsUpdate {
                    new_node_id: 42,
                    nominal_bitrate: 1_000_000,
                    data_bitrate: Some(2_000_000),
                    transmit_pdo_brs: Some(true),
                },
            )
            .await
            .unwrap();

        assert!(result.changed);
        assert!(result.restart_required);
        assert_eq!(
            manager.list()[0].can_config,
            DeviceCanConfigStatus::Available(DeviceCanConfig {
                stored_node_id: 42,
                nominal_bitrate: 1_000_000,
                data_bitrate: Some(2_000_000),
                transmit_pdo_brs: Some(true),
            })
        );
        assert_eq!(
            bus.requests(),
            vec![
                RecordedSdo::Upload {
                    index: 0x1018,
                    subindex: 1,
                },
                RecordedSdo::Upload {
                    index: 0x1018,
                    subindex: 2,
                },
                RecordedSdo::Upload {
                    index: 0x2001,
                    subindex: 0,
                },
                RecordedSdo::Upload {
                    index: 0x2001,
                    subindex: 1,
                },
                RecordedSdo::Upload {
                    index: 0x2001,
                    subindex: 2,
                },
                RecordedSdo::Upload {
                    index: 0x2001,
                    subindex: 3,
                },
                RecordedSdo::Upload {
                    index: 0x2001,
                    subindex: 4,
                },
                RecordedSdo::Upload {
                    index: 0x1018,
                    subindex: 1,
                },
                RecordedSdo::Download {
                    index: 0x2001,
                    subindex: 3,
                    data: 2_000_000u32.to_le_bytes().to_vec(),
                },
                RecordedSdo::Download {
                    index: 0x2001,
                    subindex: 4,
                    data: vec![1],
                },
                RecordedSdo::Download {
                    index: 0x2001,
                    subindex: 1,
                    data: vec![42],
                },
                RecordedSdo::Download {
                    index: 0x1010,
                    subindex: 1,
                    data: 0x6576_6173u32.to_le_bytes().to_vec(),
                },
            ]
        );
    }

    #[tokio::test]
    async fn target_node_probe_only_treats_explicit_protocol_timeout_as_free() {
        let free_bus = Arc::new(ScriptedSettingsBus::new([((42, 0x1018, 1), vec![])]));
        let free_manager = test_manager(free_bus);
        free_manager
            .probe_target_node_id_available(1, 42)
            .await
            .unwrap();

        let occupied_bus = Arc::new(ScriptedSettingsBus::new([(
            (42, 0x1018, 1),
            vec![0x1234_5678u32.to_le_bytes().to_vec()],
        )]));
        let occupied_manager = test_manager(occupied_bus);
        let occupied = occupied_manager
            .probe_target_node_id_available(1, 42)
            .await
            .unwrap_err();
        assert!(occupied.to_string().contains("responded"));
        assert!(occupied.to_string().contains("occupied"));

        let uncertain_bus = Arc::new(ScriptedSettingsBus::new([]));
        let uncertain_manager = test_manager(uncertain_bus);
        let uncertain = uncertain_manager
            .probe_target_node_id_available(1, 42)
            .await
            .unwrap_err();
        assert!(uncertain.to_string().contains("could not safely prove"));
    }

    #[tokio::test]
    async fn settings_epoch_change_during_forced_read_blocks_all_writes() {
        let bus = Arc::new(ScriptedSettingsBus::new(motor_settings_values(
            1, 5_000_000, false,
        )));
        let manager = test_manager(bus.clone());
        add_online_entry(
            &manager,
            1,
            MOTOR_VENDOR,
            MOTOR_PRODUCT,
            NmtState::PreOperational,
        );
        let active_entry = entry(&manager, 1);
        bus.set_send_hook(Arc::new(move |nid, request| {
            if nid == 1
                && matches!(
                    request,
                    RecordedSdo::Upload {
                        index: 0x2001,
                        subindex: 4
                    }
                )
            {
                active_entry.inner.lock().unwrap().session_epoch += 1;
            }
        }));

        let error = manager
            .apply_device_settings(
                1,
                MOTOR_VENDOR,
                MOTOR_PRODUCT,
                DeviceSettingsUpdate {
                    new_node_id: 42,
                    nominal_bitrate: 1_000_000,
                    data_bitrate: Some(2_000_000),
                    transmit_pdo_brs: Some(true),
                },
            )
            .await
            .unwrap_err();

        assert!(error.to_string().contains("session changed"));
        assert!(!bus
            .requests()
            .iter()
            .any(|request| matches!(request, RecordedSdo::Download { .. })));
    }

    #[tokio::test]
    async fn settings_nmt_change_after_first_write_stops_remaining_writes_and_keeps_cache() {
        let bus = Arc::new(ScriptedSettingsBus::new(motor_settings_values(
            1, 5_000_000, false,
        )));
        let manager = test_manager(bus.clone());
        add_online_entry(
            &manager,
            1,
            MOTOR_VENDOR,
            MOTOR_PRODUCT,
            NmtState::PreOperational,
        );
        let active_entry = entry(&manager, 1);
        {
            active_entry.inner.lock().unwrap().can_config =
                DeviceCanConfigStatus::Available(DeviceCanConfig {
                    stored_node_id: 1,
                    nominal_bitrate: 1_000_000,
                    data_bitrate: Some(5_000_000),
                    transmit_pdo_brs: Some(false),
                });
        }
        let hook_entry = active_entry.clone();
        bus.set_send_hook(Arc::new(move |nid, request| {
            if nid == 1 && matches!(request, RecordedSdo::Download { .. }) {
                let mut inner = hook_entry.inner.lock().unwrap();
                if inner.nmt_state != Some(NmtState::Operational) {
                    inner.nmt_state = Some(NmtState::Operational);
                    inner.nmt_revision += 1;
                }
            }
        }));

        let error = manager
            .apply_device_settings(
                1,
                MOTOR_VENDOR,
                MOTOR_PRODUCT,
                DeviceSettingsUpdate {
                    new_node_id: 1,
                    nominal_bitrate: 1_000_000,
                    data_bitrate: Some(2_000_000),
                    transmit_pdo_brs: Some(true),
                },
            )
            .await
            .unwrap_err();

        assert!(error.to_string().contains("NMT state changed"));
        assert_eq!(
            bus.requests()
                .iter()
                .filter(|request| matches!(request, RecordedSdo::Download { .. }))
                .count(),
            1
        );
        assert_eq!(
            active_entry.inner.lock().unwrap().can_config,
            DeviceCanConfigStatus::Available(DeviceCanConfig {
                stored_node_id: 1,
                nominal_bitrate: 1_000_000,
                data_bitrate: Some(5_000_000),
                transmit_pdo_brs: Some(false),
            })
        );
    }

    #[tokio::test]
    async fn object_2100_noop_apply_rewrites_full_target_and_updates_cache() {
        const VENDOR: u32 = 0x6865_786D;
        const PRODUCT: u32 = 0x6169_6D75;
        let bus = Arc::new(ScriptedSettingsBus::new([
            ((31, 0x1018, 1), vec![VENDOR.to_le_bytes().to_vec()]),
            ((31, 0x1018, 2), vec![PRODUCT.to_le_bytes().to_vec()]),
            ((31, 0x2100, 0), vec![vec![3]]),
            ((31, 0x2101, 0), vec![vec![31]]),
            (
                (31, 0x2100, 1),
                vec![1_000_000u32.to_le_bytes().to_vec()],
            ),
            (
                (31, 0x2100, 2),
                vec![4_000_000u32.to_le_bytes().to_vec()],
            ),
            ((31, 0x2100, 3), vec![vec![1]]),
        ]));
        let manager = test_manager(bus.clone());
        add_online_entry(&manager, 31, VENDOR, PRODUCT, NmtState::Stopped);

        let result = manager
            .apply_device_settings(
                31,
                VENDOR,
                PRODUCT,
                DeviceSettingsUpdate {
                    new_node_id: 31,
                    nominal_bitrate: 1_000_000,
                    data_bitrate: Some(4_000_000),
                    transmit_pdo_brs: Some(true),
                },
            )
            .await
            .unwrap();

        assert_eq!(
            result,
            DeviceSettingsResult {
                changed: false,
                restart_required: true,
                persistence_pending: true,
                brs_applied_immediately: false,
            }
        );
        assert_eq!(
            bus.requests()
                .into_iter()
                .filter(|request| matches!(request, RecordedSdo::Download { .. }))
                .collect::<Vec<_>>(),
            vec![
                RecordedSdo::Download {
                    index: 0x2100,
                    subindex: 1,
                    data: 1_000_000u32.to_le_bytes().to_vec(),
                },
                RecordedSdo::Download {
                    index: 0x2100,
                    subindex: 2,
                    data: 4_000_000u32.to_le_bytes().to_vec(),
                },
                RecordedSdo::Download {
                    index: 0x2100,
                    subindex: 3,
                    data: vec![1],
                },
                RecordedSdo::Download {
                    index: 0x2101,
                    subindex: 0,
                    data: vec![31],
                },
            ]
        );
        assert_eq!(
            entry(&manager, 31).inner.lock().unwrap().can_config,
            DeviceCanConfigStatus::Available(DeviceCanConfig {
                stored_node_id: 31,
                nominal_bitrate: 1_000_000,
                data_bitrate: Some(4_000_000),
                transmit_pdo_brs: Some(true),
            })
        );
    }

    #[tokio::test]
    async fn settings_rejects_operational_state_without_sending_any_sdo() {
        let bus = Arc::new(ScriptedSettingsBus::new([]));
        let manager = test_manager(bus.clone());
        add_online_entry(
            &manager,
            1,
            MOTOR_VENDOR,
            MOTOR_PRODUCT,
            NmtState::Operational,
        );

        let error = manager
            .apply_device_settings(
                1,
                MOTOR_VENDOR,
                MOTOR_PRODUCT,
                DeviceSettingsUpdate {
                    new_node_id: 42,
                    nominal_bitrate: 1_000_000,
                    data_bitrate: Some(5_000_000),
                    transmit_pdo_brs: Some(false),
                },
            )
            .await
            .unwrap_err();

        assert!(error.to_string().contains("Pre-Operational or Stopped"));
        assert!(bus.requests().is_empty());
    }

    #[tokio::test]
    async fn identity_mismatch_aborts_before_config_probe_or_write() {
        let bus = Arc::new(ScriptedSettingsBus::new([
            ((1, 0x1018, 1), vec![MOTOR_VENDOR.to_le_bytes().to_vec()]),
            (
                (1, 0x1018, 2),
                vec![0xAAAA_0002u32.to_le_bytes().to_vec()],
            ),
        ]));
        let manager = test_manager(bus.clone());
        add_online_entry(
            &manager,
            1,
            MOTOR_VENDOR,
            MOTOR_PRODUCT,
            NmtState::Stopped,
        );

        let error = manager
            .apply_device_settings(
                1,
                MOTOR_VENDOR,
                MOTOR_PRODUCT,
                DeviceSettingsUpdate {
                    new_node_id: 42,
                    nominal_bitrate: 1_000_000,
                    data_bitrate: Some(5_000_000),
                    transmit_pdo_brs: Some(false),
                },
            )
            .await
            .unwrap_err();

        assert!(error.to_string().contains("identity changed"));
        assert_eq!(
            bus.requests(),
            vec![
                RecordedSdo::Upload {
                    index: 0x1018,
                    subindex: 1,
                },
                RecordedSdo::Upload {
                    index: 0x1018,
                    subindex: 2,
                },
            ]
        );
    }

    #[tokio::test]
    async fn strict_position_preset_waits_for_switch_on_disabled() {
        let bus = Arc::new(ScriptedSettingsBus::new([
            ((1, 0x1018, 1), vec![MOTOR_VENDOR.to_le_bytes().to_vec()]),
            ((1, 0x1018, 2), vec![MOTOR_PRODUCT.to_le_bytes().to_vec()]),
            (
                (1, 0x6041, 0),
                vec![
                    0x0021u16.to_le_bytes().to_vec(),
                    0x0270u16.to_le_bytes().to_vec(),
                ],
            ),
        ]));
        let manager = test_manager(bus.clone());
        add_online_entry(
            &manager,
            1,
            MOTOR_VENDOR,
            MOTOR_PRODUCT,
            NmtState::Operational,
        );

        manager
            .set_position_preset_for(1, MOTOR_VENDOR, MOTOR_PRODUCT, 0.25)
            .await
            .unwrap();

        let requests = bus.requests();
        assert_eq!(
            &requests[0..3],
            &[
                RecordedSdo::Upload {
                    index: 0x1018,
                    subindex: 1,
                },
                RecordedSdo::Upload {
                    index: 0x1018,
                    subindex: 2,
                },
                RecordedSdo::Download {
                    index: 0x6040,
                    subindex: 0,
                    data: 0u16.to_le_bytes().to_vec(),
                },
            ]
        );
        assert_eq!(
            requests
                .iter()
                .filter(|request| {
                    matches!(
                        request,
                        RecordedSdo::Upload {
                            index: 0x6041,
                            subindex: 0
                        }
                    )
                })
                .count(),
            2
        );
        assert_eq!(
            &requests[requests.len() - 2..],
            &[
                RecordedSdo::Download {
                    index: 0x3001,
                    subindex: 1,
                    data: 0.25f32.to_le_bytes().to_vec(),
                },
                RecordedSdo::Download {
                    index: 0x3001,
                    subindex: 2,
                    data: 0x7365_7270u32.to_le_bytes().to_vec(),
                },
            ]
        );
    }

    #[tokio::test]
    async fn position_session_change_after_identity_blocks_first_position_sdo() {
        let bus = Arc::new(ScriptedSettingsBus::new([
            ((1, 0x1018, 1), vec![MOTOR_VENDOR.to_le_bytes().to_vec()]),
            ((1, 0x1018, 2), vec![MOTOR_PRODUCT.to_le_bytes().to_vec()]),
        ]));
        let manager = test_manager(bus.clone());
        add_online_entry(
            &manager,
            1,
            MOTOR_VENDOR,
            MOTOR_PRODUCT,
            NmtState::Operational,
        );
        let active_entry = entry(&manager, 1);
        bus.set_send_hook(Arc::new(move |nid, request| {
            if nid == 1
                && matches!(
                    request,
                    RecordedSdo::Upload {
                        index: 0x1018,
                        subindex: 2
                    }
                )
            {
                active_entry.inner.lock().unwrap().session_epoch += 1;
            }
        }));

        let error = manager
            .set_position_preset_for(1, MOTOR_VENDOR, MOTOR_PRODUCT, 0.0)
            .await
            .unwrap_err();
        assert!(error.to_string().contains("session changed"));
        assert!(!bus
            .requests()
            .iter()
            .any(|request| matches!(request, RecordedSdo::Download { .. })));
    }

    #[tokio::test]
    async fn strict_position_operation_rejects_unknown_tuple_without_sdo() {
        let bus = Arc::new(ScriptedSettingsBus::new([]));
        let manager = test_manager(bus.clone());
        add_online_entry(
            &manager,
            1,
            0xDEAD_BEEF,
            0xCAFE_BABE,
            NmtState::PreOperational,
        );

        let error = manager
            .read_position_for(1, 0xDEAD_BEEF, 0xCAFE_BABE)
            .await
            .unwrap_err();
        assert!(error.to_string().contains("exact known motor identity"));
        assert!(bus.requests().is_empty());
    }
}
