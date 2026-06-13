# hex-motor

针对 HEX-MECHA / HexMeow CiA402 形态电机的 Rust 驱动库。

通信层基于 [`can-transport`](https://crates.io/crates/can-transport)，与
具体的 CAN 后端（socketcan / pcan / vcan / 自定义）解耦。

```toml
[dependencies]
hex-motor    = "0.1"
can-transport = { version = "0.1", features = ["socketcan"] }
tokio         = { version = "1", features = ["full"] }
```

> v0.1 状态：仅支持 **CiA402** 形态电机，仅覆盖 **"上位机交互"** 这一种使用
> 范式（动态发现 / SDO 配置 / 周期 TPDO 读状态 / SDO 控制）。**不**面向
> 1 kHz 硬实时 RPDO 控制环；如需后者请走 sans-IO 的下层 crate 自行实装。

---

## 这个库帮你做什么

- **被动发现** — 你不需要预先知道总线上有哪些电机，谁开机我们就识别谁
  （读 `0x1018` Identity + 可选 `0x1008` Manufacturer Device Name）。
- **生命周期管理** — 每个节点有 5 态 `Unknown → Identified → Initializing
  → Initialized → NeedsReinit`，加上正交的 `online`，可以一眼分辨"还没
  握上手"和"通了但掉线了"。
- **`initialize()` 一把梭** — `NMT PreOp → 配 TPDO1+TPDO2 → 配 0x1016
  心跳消费者 → NMT Operational`，失败自动回退 lifecycle，可重试。
- **CiA402 控制字 ramp** — `set_mode(ProfileVelocity)` 这种调用会自动跑
  完保险路径 (`(可选 fault reset) → CW=0x06 → 0x6060=M → CW=0x06 →
  0x07 → 0x0F`)，并等 TPDO 反馈状态字确认 `Operation Enabled` 之后才返回。
- **零阻塞的状态读取** — `status(nid)` 是无锁 `ArcSwap` fast path；要
  连续流则用 `subscribe_status(nid, opts)` 拿一个 mpsc 流，channel 满时
  按 `OverflowPolicy::Lagged` 给丢弃计数，不会阻塞主循环。
- **出向心跳广播** — Manager 持续广播 NMT Operational 心跳，电机侧
  `0x1016` 监听超时会自动触发安全保护；上位机崩了电机自动短刹车。

---

## 30 秒上手

```rust
use std::sync::Arc;
use std::time::Duration;
use can_transport::{socketcan::SocketCanBus, CanBus};
use hex_motor::cia402::{
    Cia402Manager, Cia402ManagerOptions, MotorLifecycle, OverflowPolicy,
    StatusStreamItem, StreamOptions,
};
use hex_motor::types::{MotorMode, MotorTarget};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let bus: Arc<dyn CanBus> = Arc::new(SocketCanBus::open("can0")?);
    let mgr = Cia402Manager::new(
        bus,
        Cia402ManagerOptions {
            heartbeat_node_id: 0x10, // 我方在总线上的 NMT 节点 ID
            ..Default::default()
        },
    )?;

    // 1) 等到目标节点被自动发现 / identify。
    let target: u8 = 0x21;
    loop {
        if mgr.list().iter().any(|m| {
            m.node_id == target
                && matches!(m.lifecycle, MotorLifecycle::Identified | MotorLifecycle::NeedsReinit { .. })
        }) {
            break;
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }

    // 2) initialize：跑完整的 PreOp → TPDO → 0x1016 → Operational 序列。
    mgr.initialize(target).await?;

    // 3) 切到 ProfileVelocity 模式（自动跑控制字 ramp 并等 TPDO 确认）。
    mgr.set_mode(target, MotorMode::ProfileVelocity).await?;

    // 4) 写一次目标速度（0x60FF，f32 Rev/s）。
    mgr.set_target(target, MotorTarget::Velocity { rev_per_s: 0.3 }).await?;

    // 5a) 一次性快照（无锁）：
    let snap = mgr.status(target);
    println!("pos = {:?} rev, drv_temp = {:?} C",
        snap.measurements.position_rev,
        snap.measurements.driver_temp_c);

    // 5b) 或者订阅状态流（每路独立缓冲）：
    let mut stream = mgr.subscribe_status(target, StreamOptions {
        capacity: 4096,
        on_overflow: OverflowPolicy::Lagged,
    })?;
    while let Some(item) = stream.recv().await {
        match item {
            StatusStreamItem::Sample(s) => {
                // s 是 LiveState 快照：connection / logic / measurements / timestamp
                println!("{:?}", s.measurements);
            }
            StatusStreamItem::Lagged { dropped } => {
                eprintln!("lagged: dropped {dropped} samples");
            }
        }
    }

    // 6) 退出前短刹车（也可以直接 drop manager —— 心跳一停电机会自己刹车）。
    mgr.disable(target).await?;
    Ok(())
}
```

---

## 概念速览

### 生命周期

| Lifecycle                | 含义                                     | 可控制？       |
| ------------------------ | ---------------------------------------- | -------------- |
| `Unknown`                | 看到了心跳，`0x1018` 还没读到            | 否             |
| `Identified`             | identity 已知，TPDO 还没配               | 否             |
| `Initializing`           | `initialize()` 正在跑                    | 否             |
| `Initialized`            | TPDO 在流、心跳消费者已设、NMT Op        | **是**         |
| `NeedsReinit { reason }` | 曾经 Initialized，电机离开了 Operational | 否，需 re-init |

`online` 是正交的布尔：最近一段时间内是否收到过 HB 或 TPDO。
`MotorInfo::is_ready()` 等价 `lifecycle == Initialized && online`。

### 两条数据通路

- **状态上行**（电机 → 我们）：HB / TPDO 帧 → 全局 listener 解码 → 写
  `MotorEntry` 内部字段 → `publish` 原子刷新 `ArcSwap<LiveState>` + 推
  给所有订阅者。
- **控制下行**（我们 → 电机）：用户调 `set_mode` / `set_target` / ... →
  Manager 拼出 `Vec<SdoWrite>` 序列 → 顺序 `await canopen-sdo::download`
  → 必要时再轮询 `entry.logic` 等 TPDO 反馈 → 返回 `Result`。

### 核心 API

```rust
// 构造（每条 CAN 线一个 manager）
let mgr = Cia402Manager::new(bus, opts)?;

// 列表 / 事件
mgr.list() -> Vec<MotorInfo>;
mgr.subscribe_events() -> EventStream; // NodeAppeared / Identified /
                                       // Initialized / NodeOnline / NodeOffline /
                                       // NeedsReinit / EnteredError / ...

// 显式触发
mgr.identify(nid).await?;       // 强制重读 0x1018
mgr.initialize(nid).await?;     // 完整 init 序列
mgr.initialize_all().await;     // 对所有 Identified 节点并发

// 控制（必须 lifecycle == Initialized）
mgr.set_mode(nid, MotorMode::ProfileVelocity).await?;
mgr.set_target(nid, MotorTarget::Velocity { rev_per_s }).await?;
mgr.disable(nid).await?;
mgr.clear_error(nid).await?;

// 状态读取
let snap: LiveState = mgr.status(nid);             // ArcSwap 无锁快照
let stream = mgr.subscribe_status(nid, opts)?;     // mpsc 流，per-subscriber 容量
```

### v0.1 控制范围（HexMeow CiA402 电机约定）

| 模式                  | `set_mode` | `set_target(...)` 行为                                                                                                                    |
| --------------------- | ---------- | ----------------------------------------------------------------------------------------------------------------------------------------- |
| `ProfilePosition` (1) | ✅          | 写 `0x607A` f32 Rev → CW=0x2F → CW=0x3F（CSI: Change Set Immediately 上升沿）                                                             |
| `ProfileVelocity` (3) | ✅          | 写 `0x60FF` f32 Rev/s                                                                                                                     |
| `Torque` (4)          | ✅          | 写 `0x6071` i16 = `round(nm / peak * 1000)`，clamp 到 ±1000 ‰                                                                             |
| `Mit` (5)             | ✅          | 写 `0x2003:01` pos (f32) / `:02` vel (f32) / `:03` tor (f32) / `:04` kp_int (u16) / `:05` kd_int (u16) —— **uncompressed** 形态，5 条 SDO |
| 任意 → `Disable`      | —          | `0x6040 = 0x06`，任意模式都合法                                                                                                           |

模式 / target enum variant 不匹配会得到 `Error::TargetModeMismatch`。

- `Torque` target 需要 `initialize()` 时 `0x6076` (Motor Peak Torque,
  REAL32 mNm) 能读到 —— 缺失会 `Error::Internal("peak_torque not cached")`。
- `Mit` target 需要 `0x2003:07` (MIT KP/KD Factor, REAL32) 能读到 ——
  缺失会 `Error::Internal("mit_kp_kd_factor not cached")`。
  - 物理 Kp [Nm/Rev] 与 OD u16 值的关系：`kp_int = round(kp_phys / factor)`。
  - `initialize()` 还会写一次 `0x2003:06 = 1000`，把 MIT 的 PD 输出力矩
    上限放到 ±100% peak（不写默认 0 → kp/kd 无效）。
- `ProfilePosition` 默认走 **Change Set Immediately**：每次 `set_target`
  立刻替换当前目标，不等当前 motion profile 跑完。如果想要"打表"行为
  （等当前完成再切下一个）请直接走 SDO 自行设置控制字 bit 5 = 0。

---

## 真机 examples

按里程碑顺序，每个都附 `cargo run --example ... -- can0 0x10 0x21` 的
用法（第一个参数是 CAN interface 名，第二个是我方心跳 nid，第三个是
目标电机 nid）：

| Example                   | 验证内容                                       |
| ------------------------- | ---------------------------------------------- |
| `m1_heartbeat`            | 出向心跳广播                                   |
| `m1_sdo_identity`         | 单节点 SDO 读 `0x1018` + 友好名称              |
| `m2_discover`             | 被动发现 + 自动 identify + online/offline 事件 |
| `m3_initialize_and_read`  | 完整 `initialize()` + 双路 TPDO 在流           |
| `m4_run_profile_velocity` | `set_mode(PV)` + `set_target(Velocity)` 真转   |
| `m5_subscribe_to_csv`     | `status()` 无锁快照 + `subscribe_status` → CSV |

每个 example 的开头有详细的注释和参数说明。

### CAN 后端：SocketCAN 或 gs_usb (CAN-FD)

第一个参数（interface 名）决定后端：

- `can0` / `vcan0` 等 —— Linux SocketCAN。
- `gs_usb` / `gs_usb0` / `gs_usb1` —— gs_usb / candleLight 适配器（USB
  直连，CAN-FD，Windows / macOS / Linux 通用）。结尾数字选多路适配器
  的通道（`can0` = 0）。

```bash
# 同一个 example，换成 gs_usb 适配器跑（电机在节点 0x01）：
cargo run --example m4_run_profile_velocity -- gs_usb 0x10 0x01 0.3
```

Linux 上 gs_usb 需要 usbfs 访问权限（sudo 或 udev 规则），后端会自动
卸载内核 `gs_usb` 驱动；详见 `can-transport` 的 README。

---

## 添加新电机型号

`src/cia402/known_devices.rs` 里的 `KNOWN_DEVICES` 是一个 `const &[KnownDevice]`，
直接编辑追加一条即可：

```rust
KnownDevice {
    vendor_id:    0x12_34_56_78,
    product_code: Some(0xAABB_CCDD), // None 表示该 vendor 兜底
    name:         "MyMotor Pro Max",
},
```

`MotorInfo::friendly_name()` 的优先级：
1. 电机自己上报的 `0x1008` Manufacturer Device Name（非空且非纯空白）
2. 查 `KNOWN_DEVICES`：精确匹配 `(vendor, product)` > 同 vendor 兜底
3. 都没命中 → `"Unknown CiA402 device (vendor 0x..., product 0x...)"`

---

## 默认 TPDO 映射

`initialize()` 默认配两路 TPDO（你也可以拿
`default_tpdo1_recipe(nid)` / `default_tpdo2_recipe(nid)` 自行调用底层 API）：

- **TPDO1**（高速 1 ms，COB-ID `0x180 + nid`，12 字节）：
  `0x6064`(32) + `0x1013`(32, timestamp) + `0x6077`(16, torque) + `0x603F`(16, error)
- **TPDO2**（低速 20 ms，COB-ID `0x280 + nid`，10 字节）：
  `0x6041`(16) + `0x2204:01`(16, drv_temp) + `0x2204:02`(16, motor_temp) +
  `0x6040`(16, ctrl readback) + `0x603F`(16, error)

两路加起来都 > 8 B，需要 **CAN-FD**。`0x1013` / `0x2204:01-02` 是 vendor-specific 字段，
标准 CiA402 不保证；其他厂家的电机请提供自定义 recipe。

---

## 限制与未支持

- **没有 RPDO**：所有控制走 SDO。如需 1 kHz+ 闭环请用 sans-IO 层自己写。
- **NMT 只切换一次**：`initialize()` 里 PreOp → Op 之后就不再动 NMT。
- **没有自定义协议**：HexMeow 的 0x3000 自定义 OD 形态在 v0.1 之外，由
  独立 manager 在后续版本提供。
- `drop(manager)` 不发 NMT Stop —— 出向心跳一断，电机端 `0x1016` 监听
  超时（默认 250 ms）后自动触发安全保护。

---

## 许可证

双协议：MIT OR Apache-2.0，任选其一。
