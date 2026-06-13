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
use futures::future::join_all;
use tokio::sync::broadcast;
use tokio::task::JoinHandle;
use tokio_util::sync::CancellationToken;

use crate::canopen::sdo;
use crate::canopen::tpdo_config::SdoWrite;
use crate::error::{Error, Result};
use crate::types::{MotorMode, MotorTarget};

use super::discovery::{identify_once, run_discovery, run_liveness_monitor};
use super::events::{Cia402Event, EventStream, DEFAULT_EVENTS_CAPACITY};
use super::heartbeat::run_hb_broadcast;
use super::initialize::run_initialize;
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
                    identity: inner.identity.clone(),
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

    /// 强制重读指定节点的 0x1018（+ 0x1008 可选）。
    ///
    /// - 若节点尚未在 list 中，会先创建一个 `Unknown` 条目。
    /// - 与背景自动 identify / 任何 `initialize()` 互斥（共享 `inflight_ops`），
    ///   重复时返回 `Error::Internal("... already in progress")`。
    pub async fn identify(&self, nid: u8) -> Result<()> {
        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "identify")?;

        let entry = self.get_or_insert_entry(nid);
        let identity = identify_once(self.bus.as_ref(), nid, self.opts.sdo_timeout).await?;

        {
            let mut inner = entry.inner.lock().unwrap();
            inner.identity = Some(identity.clone());
            if matches!(inner.lifecycle, MotorLifecycle::Unknown) {
                inner.lifecycle = MotorLifecycle::Identified;
            }
        }
        let _ = self.events_tx.send(Cia402Event::Identified { nid, identity });
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
    pub async fn initialize(&self, nid: u8) -> Result<()> {
        let entry = {
            let g = self.motors.read().unwrap();
            g.get(&nid).cloned().ok_or_else(|| {
                crate::error::Error::Internal(format!("nid 0x{nid:02X} not in list yet"))
            })?
        };
        // 必须已 Identified（或 NeedsReinit）。Unknown 还没拉到 identity 不能配。
        {
            let inner = entry.inner.lock().unwrap();
            if matches!(inner.lifecycle, MotorLifecycle::Unknown) {
                return Err(crate::error::Error::NotReady {
                    nid,
                    lifecycle: format!("{:?}", inner.lifecycle),
                });
            }
        }

        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "initialize")?;
        run_initialize(self.bus.as_ref(), entry, &self.events_tx, &self.opts).await
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
        self.sdo_download_sequential(nid, &writes).await?;

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

    /// 改电机 Node-ID（出厂 / 批量配置工具用）。
    ///
    /// 写 `0x2001:01 = new_id`（Uint8），再写 `0x1010:01 = 0x65766173`（ASCII
    /// `"save"`）触发保存。**电机重新上电后才生效**；改完旧 ID 仍在用直到掉电。
    ///
    /// **不要求 Initialized**：电机只要在线（PreOperational/Operational 下 SDO
    /// 可达）即可，所以发现到就能改。与该 nid 上的 identify/initialize 互斥。
    pub async fn change_node_id(&self, nid: u8, new_id: u8) -> Result<()> {
        if !(1..=127).contains(&new_id) {
            return Err(Error::Internal(format!(
                "new node id must be 1..=127, got {new_id}"
            )));
        }
        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "change_node_id")?;
        let timeout = Some(self.opts.sdo_timeout);
        // 0x2001:01 Node-ID (Uint8)
        sdo::download_u8(self.bus.as_ref(), nid, 0x2001, 1, new_id, timeout).await?;
        // 0x1010:01 store parameters = "save" = 0x65766173
        sdo::download_u32(self.bus.as_ref(), nid, 0x1010, 1, 0x6576_6173, timeout).await?;
        log::info!(
            "nid 0x{nid:02X}: wrote new Node-ID 0x{new_id:02X} + save \
             (power-cycle the motor to apply)"
        );
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

    /// 用户位置预设（零点工具用）。把电机**当前**转子位置设成 `pos`（Rev，
    /// clamp 到 -0.5..=0.5）。详见 huayi.md §3.6。
    ///
    /// 序列：`0x6040 = 0x0000`（确保 Switch On Disabled，预设只在此状态生效）→
    /// `0x3001:01 = f32(pos)` → `0x3001:02 = 0x73657270`（ASCII `"pres"`，写入后
    /// 电机把当前位置设为 0x3001:01 并自动保存、自动清零）。
    ///
    /// **不要求 Initialized**：电机刚上电即在 Switch On Disabled，发现到就能调。
    pub async fn set_position_preset(&self, nid: u8, pos: f32) -> Result<()> {
        let pos = pos.clamp(-0.5, 0.5);
        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "set_position_preset")?;
        let timeout = Some(self.opts.sdo_timeout);
        // 确保处于 Switch On Disabled（写控制字 0x0000 = Disable Voltage）
        sdo::download_u16(self.bus.as_ref(), nid, 0x6040, 0, 0x0000, timeout).await?;
        // 0x3001:01 期望位置 (REAL32, Rev)
        sdo::download_f32(self.bus.as_ref(), nid, 0x3001, 1, pos, timeout).await?;
        // 0x3001:02 = "pres" = 0x73657270 → 把当前位置设为 0x3001:01
        sdo::download_u32(self.bus.as_ref(), nid, 0x3001, 2, 0x7365_7270, timeout).await?;
        log::info!("nid 0x{nid:02X}: position preset -> {pos} Rev");
        Ok(())
    }

    /// 按需读一次电机当前位置 `0x6064`（REAL32，单圈 Rev）。零点工具用——**只在
    /// 发现/点按/保存后读，不轮询**（总线上电机可能随时掉电，避免无谓 TX）。
    /// 不要求 Initialized。
    pub async fn read_position(&self, nid: u8) -> Result<f32> {
        let _guard = InflightGuard::acquire(&self.inflight_ops, nid, "read_position")?;
        sdo::upload_f32(self.bus.as_ref(), nid, 0x6064, 0, Some(self.opts.sdo_timeout)).await
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

    /// 拿到 / 建立指定 nid 的 entry。
    fn get_or_insert_entry(&self, nid: u8) -> Arc<MotorEntry> {
        let mut g = self.motors.write().unwrap();
        g.entry(nid)
            .or_insert_with(|| Arc::new(MotorEntry::new(nid)))
            .clone()
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
