//! 真机控制冒烟测试：GUI 无关，在一条命令里依次验证 **使能 / 发目标 / 不关电
//! 切模式 / 限力矩 / 失能** 整条 CiA402 控制链路。
//!
//! 对每个被点名的模式：`set_mode` → `set_max_torque` 保护 → 发一个**温和**的
//! 目标 → 持续若干秒打印 `position / velocity(滤波) / torque / status` →
//! `disable`。多个模式按顺序跑，**全程同一个 Manager、不 NMT Reset、不关电**，
//! 因此模式之间的切换就是在演示"不关电切模式"。
//!
//! ## 用法
//!
//! ```bash
//! cargo run --example control_smoke -- <iface> <our_nid> <target_nid> [modes] [max_torque_permille] [secs_per_mode]
//! # 例：依次跑 PV、PP、Torque、MIT，最大力矩限 200‰，每个模式跑 4 秒
//! cargo run --example control_smoke -- can0 0x10 0x21 pv,pp,torque,mit 200 4
//! ```
//!
//! 参数默认：`modes=pv`、`max_torque_permille=200`(20%)、`secs_per_mode=4`。
//! modes 可选值：`pp` `pv` `torque` `mit`（逗号分隔，大小写不敏感）。
//!
//! ⚠️ **先把电机架空 / 卸负载再跑**。目标值都取得很温和，但请人留在急停旁。
//!
//! ## 退出
//!
//! 正常跑完会对电机 `disable`。中途 Ctrl+C 也会尝试 `disable` 再退出；即便没
//! 来得及，drop 时我方心跳一停，电机端 0x1016 消费者 250ms 后自触发安全保护。

use std::sync::Arc;
use std::time::{Duration, Instant};

use can_transport::CanBus;
mod common;
use hex_motor::cia402::{
    Cia402Event, Cia402Manager, Cia402ManagerOptions, EventStreamItem, LiveState, MotorLifecycle,
};
use hex_motor::types::{MotorMode, MotorTarget};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let iface = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "vcan0".to_string());
    let our_nid: u8 = arg_u8(2)?.unwrap_or(0x10);
    let target: u8 =
        arg_u8(3)?.ok_or_else(|| anyhow::anyhow!("3rd arg required: target motor nid"))?;
    let modes = parse_modes(&std::env::args().nth(4).unwrap_or_else(|| "pv".into()))?;
    let max_torque_permille: u16 = std::env::args()
        .nth(5)
        .map(|s| s.parse::<u16>())
        .transpose()?
        .unwrap_or(200);
    let secs_per_mode: u64 = std::env::args()
        .nth(6)
        .map(|s| s.parse::<u64>())
        .transpose()?
        .unwrap_or(4);

    println!(
        "Opening {iface}\n  our HB nid       = 0x{our_nid:02X}\n  target motor nid = 0x{target:02X}\n  \
         modes            = {modes:?}\n  max torque       = {max_torque_permille}‰ of peak\n  \
         secs per mode    = {secs_per_mode}\n  ⚠️  motor should be UNLOADED / free to spin."
    );

    let bus: Arc<dyn CanBus> = common::open_bus(&iface).await?;
    let mgr = Cia402Manager::new(
        bus.clone(),
        Cia402ManagerOptions {
            heartbeat_node_id: our_nid,
            ..Default::default()
        },
    )?;

    // ===== 1. 等 Identified + Initialize =====
    println!("\n=== Waiting for nid 0x{target:02X} to be Identified ===");
    wait_for_identified(&mgr, target, Duration::from_secs(10)).await?;
    println!("=== Initialize ===");
    mgr.initialize(target).await?;
    println!("  -> Initialized");

    // 读一下缓存的峰值力矩（Torque 模式把 Nm 目标换算成 ‰ 要用；这里也用它把
    // 限力矩的 ‰ 打印成 Nm 方便人看）。
    let peak_nm = mgr
        .list()
        .into_iter()
        .find(|m| m.node_id == target)
        .and_then(|m| m.peak_torque_nm);
    match peak_nm {
        Some(p) => println!("  peak torque (0x6076) = {p:.3} Nm"),
        None => println!("  peak torque (0x6076) = <not read> (Torque mode will be skipped)"),
    }

    // ===== 2. 限制最大力矩（所有模式生效）=====
    println!("\n=== set_max_torque({max_torque_permille}‰) ===");
    mgr.set_max_torque(target, max_torque_permille).await?;
    if let Some(p) = peak_nm {
        println!(
            "  -> 0x6072 = {max_torque_permille}‰  (≈ {:.3} Nm)",
            p * max_torque_permille as f32 / 1000.0
        );
    }

    // ===== 3. 逐模式跑 =====
    let mut events = mgr.subscribe_events();
    for (i, mode) in modes.iter().enumerate() {
        if i > 0 {
            println!("\n--- switching mode WITHOUT power cycle ---");
        }
        let r = run_one_mode(
            &mgr,
            target,
            *mode,
            peak_nm,
            Duration::from_secs(secs_per_mode),
            &mut events,
        )
        .await;
        if let Err(e) = r {
            println!("  !! mode {mode:?} aborted: {e}");
            // 出错就尽量失能后继续下一个模式
            let _ = mgr.disable(target).await;
        }
    }

    // ===== 4. 收尾 =====
    println!("\n=== all modes done, disabling ===");
    match mgr.disable(target).await {
        Ok(()) => println!("  disable() ok"),
        Err(e) => println!("  disable() failed: {e}"),
    }
    Ok(())
}

/// 跑单个模式：set_mode → set_target(温和) → 打印 N 秒 → disable。
async fn run_one_mode(
    mgr: &Cia402Manager,
    nid: u8,
    mode: MotorMode,
    peak_nm: Option<f32>,
    dwell: Duration,
    events: &mut hex_motor::cia402::EventStream,
) -> anyhow::Result<()> {
    println!("\n=== set_mode({mode:?}) ===");
    let t0 = Instant::now();
    mgr.set_mode(nid, mode).await?;
    println!("  -> Enabled({mode:?}) confirmed in {:?}", t0.elapsed());

    // 当前位置（给 MIT 当保持目标用）。
    let cur_pos = mgr.status(nid).measurements.position_rev.unwrap_or(0.0);

    let target = match mode {
        MotorMode::ProfilePosition => Some(MotorTarget::Position { rev: 0.1 }),
        MotorMode::ProfileVelocity => Some(MotorTarget::Velocity { rev_per_s: 0.3 }),
        MotorMode::Torque => match peak_nm {
            // 5% 峰值力矩，很温和。
            Some(p) => Some(MotorTarget::Torque { nm: p * 0.05 }),
            None => {
                println!("  (skip target: peak torque unknown, can't build a safe Nm target)");
                None
            }
        },
        // 纯阻尼：kp=0、小 kd，电机不会被往某个位置猛拽，只是转起来有阻力。
        MotorMode::Mit => Some(MotorTarget::Mit {
            pos: cur_pos,
            vel: 3.0,
            tor: 0.0,
            kp: 0.0,
            kd: 1.0,
        }),
    };

    if let Some(t) = target {
        println!("  set_target({t:?})");
        mgr.set_target(nid, t).await?;
    }

    // 打印 dwell 时间内的实时反馈；期间监听 error 事件。
    let deadline = Instant::now() + dwell;
    let mut last_print = Instant::now() - Duration::from_secs(1);
    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                println!("\n  Ctrl+C → disable + exit");
                let _ = mgr.disable(nid).await;
                std::process::exit(0);
            }
            ev = events.recv() => {
                if let Some(EventStreamItem::Event(Cia402Event::EnteredError { nid: n, kind, raw })) = ev {
                    if n == nid {
                        anyhow::bail!("motor entered Error: kind={kind:?} raw=0x{raw:04X}");
                    }
                }
            }
            _ = tokio::time::sleep(Duration::from_millis(50)) => {}
        }

        if last_print.elapsed() >= Duration::from_millis(500) {
            print_live(nid, &mgr.status(nid));
            last_print = Instant::now();
        }
        if Instant::now() >= deadline {
            break;
        }
    }

    println!("  disable()");
    mgr.disable(nid).await?;
    Ok(())
}

fn print_live(nid: u8, s: &LiveState) {
    let m = &s.measurements;
    let f = |o: Option<f32>| {
        o.map(|v| format!("{v:+.4}"))
            .unwrap_or_else(|| "  --  ".into())
    };
    println!(
        "  0x{nid:02X} pos={} rev  vel={} rev/s  tau={} Nm  sw={}  ts={}us  logic={:?}",
        f(m.position_rev),
        f(m.velocity_rev_per_s),
        f(m.torque_nm),
        m.status_word
            .map(|w| format!("0x{w:04X}"))
            .unwrap_or_else(|| "----".into()),
        m.timestamp_us
            .map(|t| t.to_string())
            .unwrap_or_else(|| "--".into()),
        s.logic,
    );
}

fn parse_modes(s: &str) -> anyhow::Result<Vec<MotorMode>> {
    s.split(',')
        .map(|tok| match tok.trim().to_lowercase().as_str() {
            "pp" | "profileposition" | "position" => Ok(MotorMode::ProfilePosition),
            "pv" | "profilevelocity" | "velocity" => Ok(MotorMode::ProfileVelocity),
            "torque" | "pt" | "t" => Ok(MotorMode::Torque),
            "mit" => Ok(MotorMode::Mit),
            other => Err(anyhow::anyhow!(
                "unknown mode '{other}' (use pp/pv/torque/mit)"
            )),
        })
        .collect()
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
                return Ok(())
            }
            Ok(Some(_)) => continue,
            Ok(None) => anyhow::bail!("event stream closed"),
            Err(_) => anyhow::bail!("timeout waiting for nid 0x{nid:02X} to be Identified"),
        }
    }
}

fn arg_u8(idx: usize) -> anyhow::Result<Option<u8>> {
    std::env::args()
        .nth(idx)
        .map(|s| {
            let s = s.trim_start_matches("0x");
            u8::from_str_radix(s, 16)
                .or_else(|_| s.parse())
                .map_err(|e| anyhow::anyhow!("invalid u8 '{s}': {e}"))
        })
        .transpose()
}
