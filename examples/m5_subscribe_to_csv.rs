//! M5 真机自测：订阅一台电机的 `LiveState` 流，边跑 ProfileVelocity 边录 CSV。
//!
//! 流程（叠加在 M4 之上）：
//! 1. 自动 identify → `mgr.initialize(target)` 配 TPDO1+TPDO2 → NMT Operational
//! 2. `mgr.set_mode(target, ProfileVelocity)` + `mgr.set_target(Velocity)` 把电机转起来
//! 3. **`mgr.status(target)`** 拿一次无锁快照打印（验证 ArcSwap fast path）
//! 4. **`mgr.subscribe_status(target, …)`** 拿一个 stream，逐条写 CSV
//! 5. 每秒打印一次"已收样本 / 累计 dropped"汇总
//! 6. Ctrl+C → `disable()` 短刹车 → flush CSV 退出
//!
//! ## 用法
//!
//! ```bash
//! cargo run --example m5_subscribe_to_csv -- <iface> <our_hb_nid> <target_nid> \
//!     [rev_per_s] [csv_path]
//! # 例如：
//! cargo run --example m5_subscribe_to_csv -- can0 0x10 0x21 0.5 m5.csv
//! ```
//!
//! 默认 `rev_per_s=0.3`，`csv_path="m5_status_<nid>.csv"`。**先把电机架空**。
//!
//! CSV 列：
//!   `t_ms` — 自订阅开始的毫秒数（i64，可能跳变如果系统 sleep 过）
//!   `online` — 0/1
//!   `nmt` — Operational / PreOperational / ...
//!   `lifecycle` — N/A（M5 不在 LiveState 里，只能从 list() 拿）
//!   `logic` — Disabled / Enabled(<mode>) / Error(<kind>,0xRRRR)
//!   `pos_rev` — 单圈 Rev `[-0.5, 0.5)`（HexMeow 私有 f32），空 → 空字段
//!   `driver_temp_c`，`motor_temp_c` — ℃
//!   `status_word` — `0xXXXX` (raw)
//!
//! 故意把 `pos_rev` 留成单圈未 unwrap：multi-turn 累积是上位机的事，详见
//! `DESIGN.md` §6。

use std::io::Write;
use std::sync::Arc;
use std::time::{Duration, Instant};

use can_transport::CanBus;
mod common;
use hex_motor::cia402::{
    Cia402Event, Cia402Manager, Cia402ManagerOptions, EventStreamItem, Logic, MotorLifecycle,
    OverflowPolicy, StatusStreamItem, StreamOptions,
};
use hex_motor::types::{MotorMode, MotorTarget};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let iface = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "vcan0".to_string());
    let our_nid: u8 = std::env::args()
        .nth(2)
        .map(|s| parse_u8(&s))
        .transpose()?
        .unwrap_or(0x10);
    let target: u8 = std::env::args()
        .nth(3)
        .map(|s| parse_u8(&s))
        .transpose()?
        .ok_or_else(|| anyhow::anyhow!("3rd arg required: target motor nid"))?;
    let rev_per_s: f32 = std::env::args()
        .nth(4)
        .map(|s| s.parse::<f32>())
        .transpose()?
        .unwrap_or(0.3);
    let csv_path = std::env::args()
        .nth(5)
        .unwrap_or_else(|| format!("m5_status_0x{target:02X}.csv"));

    println!(
        "Opening {iface}\n  our HB nid = 0x{our_nid:02X}\n  target motor nid = 0x{target:02X}\n  \
         target velocity = {rev_per_s:.3} Rev/s\n  CSV path = {csv_path}"
    );

    let bus: Arc<dyn CanBus> = common::open_bus(&iface).await?;
    let mgr = Cia402Manager::new(
        bus.clone(),
        Cia402ManagerOptions {
            heartbeat_node_id: our_nid,
            ..Default::default()
        },
    )?;

    // ===== 1. 等到 target 被 Identified =====
    println!("\n=== Waiting for nid 0x{target:02X} to be Identified ===");
    wait_for_identified(&mgr, target, Duration::from_secs(10)).await?;
    {
        let m = mgr
            .list()
            .into_iter()
            .find(|m| m.node_id == target)
            .unwrap();
        println!("Identified: {} ({:?})", m.friendly_name(), m.identity);
    }

    // ===== 2. Initialize =====
    println!("\n=== Initialize ===");
    let t0 = Instant::now();
    mgr.initialize(target).await?;
    println!("  -> Initialized in {:?}", t0.elapsed());

    // ===== 3. set_mode + set_target =====
    println!("\n=== set_mode(ProfileVelocity) ===");
    let t0 = Instant::now();
    mgr.set_mode(target, MotorMode::ProfileVelocity).await?;
    println!(
        "  -> Enabled(ProfileVelocity) confirmed in {:?}",
        t0.elapsed()
    );

    println!("\n=== set_target(Velocity {{ rev_per_s = {rev_per_s:.3} }}) ===");
    mgr.set_target(target, MotorTarget::Velocity { rev_per_s })
        .await?;
    println!("  -> 0x60FF written");

    // ===== 4. 验证 status() fast path =====
    let s = mgr.status(target);
    println!(
        "\n=== status() snapshot ===\n  online={}  nmt={:?}  logic={:?}\n  pos={:?}  drv_temp={:?}  motor_temp={:?}  status_word={:?}",
        s.connection.online,
        s.connection.nmt_state,
        s.logic,
        s.measurements.position_rev,
        s.measurements.driver_temp_c,
        s.measurements.motor_temp_c,
        s.measurements.status_word,
    );

    // ===== 5. 订阅状态流，开始写 CSV =====
    let mut stream = mgr.subscribe_status(
        target,
        StreamOptions {
            // 1 kHz TPDO1 + 50 Hz TPDO2 + 心跳 + offline ~= 高峰 ~1100 / s。
            // 4096 给 ~3.7 s 的缓冲。如果 CSV 写盘真慢到压不住 lagged 一定会出现。
            capacity: 4096,
            on_overflow: OverflowPolicy::Lagged,
        },
    )?;
    println!("\n=== Subscribing and writing CSV to {csv_path} (Ctrl+C to stop) ===");

    let csv = std::fs::File::create(&csv_path)?;
    let mut csv = std::io::BufWriter::new(csv);
    writeln!(
        csv,
        "t_ms,online,nmt,logic,pos_rev,driver_temp_c,motor_temp_c,status_word"
    )?;

    let start = Instant::now();
    let mut samples: u64 = 0;
    let mut dropped_total: u64 = 0;
    let mut last_print = Instant::now();
    let mut events = mgr.subscribe_events();

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                println!("\nCtrl+C, disabling motor...");
                if let Err(e) = mgr.disable(target).await {
                    println!("  disable() failed: {e}");
                } else {
                    println!("  disable() ok");
                }
                break;
            }

            ev = events.recv() => {
                match ev {
                    Some(EventStreamItem::Event(Cia402Event::EnteredError { nid, kind, raw }))
                        if nid == target => {
                        println!(
                            "  !! [EVENT] EnteredError kind={kind:?} raw=0x{raw:04X}"
                        );
                    }
                    Some(EventStreamItem::Event(ev)) => println!("  [EVENT] {ev:?}"),
                    Some(EventStreamItem::Lagged { dropped }) =>
                        println!("  [EVENT] !! event stream lagged, dropped {dropped}"),
                    None => break,
                }
            }

            item = stream.recv() => {
                match item {
                    None => {
                        println!("  status stream closed (manager dropped?)");
                        break;
                    }
                    Some(StatusStreamItem::Lagged { dropped }) => {
                        dropped_total += dropped;
                        eprintln!("  !! status stream lagged, dropped {dropped}");
                    }
                    Some(StatusStreamItem::Sample(s)) => {
                        samples += 1;
                        write_csv_row(&mut csv, start, &s)?;
                    }
                }
            }
        }

        if last_print.elapsed() >= Duration::from_secs(1) {
            let snap = mgr.status(target);
            println!(
                "  recorded {samples} samples (lagged {dropped_total})  \
                 online={} logic={:?} pos={:?} drv={:?}°C",
                snap.connection.online,
                snap.logic,
                snap.measurements.position_rev,
                snap.measurements.driver_temp_c,
            );
            last_print = Instant::now();
        }
    }

    csv.flush()?;
    println!("\nWrote {samples} sample rows to {csv_path} (lagged {dropped_total} during run)");

    Ok(())
}

fn write_csv_row(
    out: &mut impl Write,
    start: Instant,
    s: &hex_motor::cia402::LiveState,
) -> std::io::Result<()> {
    let t_ms = (Instant::now() - start).as_millis();
    let online = if s.connection.online { 1 } else { 0 };
    let nmt = s
        .connection
        .nmt_state
        .map(|n| format!("{n:?}"))
        .unwrap_or_default();
    let logic = match &s.logic {
        None => String::new(),
        Some(Logic::Disabled) => "Disabled".into(),
        Some(Logic::Enabled(m)) => format!("Enabled({})", m.name()),
        Some(Logic::Error { kind, raw_code }) => format!("Error({kind:?},0x{raw_code:04X})"),
    };
    let pos = s
        .measurements
        .position_rev
        .map(|v| format!("{v:.6}"))
        .unwrap_or_default();
    let drv = s
        .measurements
        .driver_temp_c
        .map(|v| format!("{v:.1}"))
        .unwrap_or_default();
    let mot = s
        .measurements
        .motor_temp_c
        .map(|v| format!("{v:.1}"))
        .unwrap_or_default();
    let sw = s
        .measurements
        .status_word
        .map(|v| format!("0x{v:04X}"))
        .unwrap_or_default();
    writeln!(out, "{t_ms},{online},{nmt},{logic},{pos},{drv},{mot},{sw}")
}

async fn wait_for_identified(
    mgr: &Cia402Manager,
    nid: u8,
    timeout: Duration,
) -> anyhow::Result<()> {
    let deadline = Instant::now() + timeout;
    let mut events = mgr.subscribe_events();

    if mgr.list().iter().any(|m| {
        m.node_id == nid
            && matches!(
                m.lifecycle,
                MotorLifecycle::Identified | MotorLifecycle::NeedsReinit { .. }
            )
    }) {
        return Ok(());
    }

    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            anyhow::bail!("timeout waiting for nid 0x{nid:02X} to be Identified");
        }
        match tokio::time::timeout(remaining, events.recv()).await {
            Ok(Some(EventStreamItem::Event(Cia402Event::Identified { nid: n, .. })))
                if n == nid =>
            {
                return Ok(());
            }
            Ok(Some(_)) => continue,
            Ok(None) => anyhow::bail!("event stream closed"),
            Err(_) => anyhow::bail!("timeout waiting for nid 0x{nid:02X} to be Identified"),
        }
    }
}

fn parse_u8(s: &str) -> anyhow::Result<u8> {
    let s = s.trim_start_matches("0x");
    u8::from_str_radix(s, 16)
        .or_else(|_| s.parse())
        .map_err(|e| anyhow::anyhow!("invalid u8 '{s}': {e}"))
}
