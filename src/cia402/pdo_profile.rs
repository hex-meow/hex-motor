//! 一条物理总线在一次会话内冻结的 PDO 节拍。
//!
//! 这个类型只描述**机制**：给定一个 TPDO1 事件定时器，派生出与之自洽的
//! inhibit time 和速度窗口。**策略**（这条总线该跑多快）不属于驱动 —— 它取决于
//! 数据段波特率、BRS、总线上有几个电机、共享帧多宽，这些只有上层知道。
//! 上层算出结论后用 [`Cia402Manager::freeze_pdo_profile`] 冻结一次。
//!
//! [`Cia402Manager::freeze_pdo_profile`]: crate::cia402::Cia402Manager::freeze_pdo_profile
//!
//! # 为什么三个参数必须一起派生
//!
//! - **inhibit time 才是真正的限流器。** TPDO1 的 transmission type 是 255
//!   （异步事件触发），event timer 只是其中一个触发源，设备数据变化时还能额外
//!   发。只把 event timer 从 1 ms 改成 4 ms、inhibit 仍留在 0.5 ms，一个会
//!   on-change 触发的固件照样能发到 2 kHz —— 降频完全落空，而且现象是"带宽没降"
//!   而不是报错，很难定位。
//! - **速度窗口是按样本数定的。** 速度由上位机对 `(timestamp, position)` 做最小
//!   二乘斜率，窗口 15 ms 这个默认值的真实含义是"≈15 个样本 @ 1 kHz"。周期变成
//!   4 ms 而窗口不动，就只剩 3.75 个样本，速度噪声爆炸；现象是关节抖，没有人会
//!   往 TPDO 配置上想。
//!
//! 所以对外只暴露一个"速率"入口，另外两个由本模块派生；确有需要再逐项覆写。

use std::time::Duration;

use crate::error::{Error, Result};

/// 一条总线的 TPDO1 节拍。用 [`PdoProfile::from_event_timer_ms`] 构造。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PdoProfile {
    event_timer_ms: u16,
    inhibit_time_x100us: u16,
    velocity_window: Duration,
}

impl PdoProfile {
    /// 出厂节拍：1 ms 事件定时器 = 1000 Hz。
    pub const DEFAULT_EVENT_TIMER_MS: u16 = 1;

    /// event timer 的合法上界。再慢就和 TPDO1"高速通道"的定位冲突了，而且会撞上
    /// [`Cia402ManagerOptions::initialized_stale_threshold`] 的在线判定。
    ///
    /// [`Cia402ManagerOptions::initialized_stale_threshold`]: crate::cia402::Cia402ManagerOptions::initialized_stale_threshold
    pub const MAX_EVENT_TIMER_MS: u16 = 100;

    /// 速度最小二乘窗口里期望的样本数。默认窗口 15 ms @ 1 kHz 就是这么来的。
    pub const VELOCITY_WINDOW_SAMPLES: u32 = 15;

    /// 按 TPDO1 事件定时器构造，inhibit time 与速度窗口自动派生。
    ///
    /// 实际速率是 `1000 / event_timer_ms` Hz —— 只有 1000/N 是可表达的
    /// （OD 单位就是整毫秒），调用方需要自己接受这个量化。
    ///
    /// `event_timer_ms == 0` 会被拒绝：CiA-301 里 0 表示**关闭**定时触发，
    /// 那样 TPDO1 就退化成纯 on-change，节拍完全不可预测。
    pub fn from_event_timer_ms(event_timer_ms: u16) -> Result<Self> {
        if event_timer_ms == 0 {
            return Err(Error::Internal(
                "PdoProfile: event_timer_ms 0 disables the timer in CiA-301; \
                 TPDO1 would degrade to on-change only"
                    .into(),
            ));
        }
        if event_timer_ms > Self::MAX_EVENT_TIMER_MS {
            return Err(Error::Internal(format!(
                "PdoProfile: event_timer_ms {event_timer_ms} exceeds the {} ms maximum",
                Self::MAX_EVENT_TIMER_MS
            )));
        }
        Ok(Self {
            event_timer_ms,
            inhibit_time_x100us: derive_inhibit_time_x100us(event_timer_ms),
            velocity_window: derive_velocity_window(event_timer_ms),
        })
    }

    /// 覆写派生出的 inhibit time（单位 100 µs）。
    ///
    /// 必须 ≤ event timer：inhibit 比周期还长的话，限流器就压过了定时器，实际
    /// 速率不再是 `1000 / event_timer_ms`，profile 自身的描述就不成立了。
    pub fn with_inhibit_time_x100us(mut self, inhibit_time_x100us: u16) -> Result<Self> {
        if inhibit_time_x100us > self.event_timer_ms * 10 {
            return Err(Error::Internal(format!(
                "PdoProfile: inhibit {inhibit_time_x100us}×100µs exceeds the \
                 {} ms event timer; the inhibit would become the real rate limit",
                self.event_timer_ms
            )));
        }
        self.inhibit_time_x100us = inhibit_time_x100us;
        Ok(self)
    }

    /// 覆写派生出的速度窗口。
    pub fn with_velocity_window(mut self, velocity_window: Duration) -> Result<Self> {
        if velocity_window.is_zero() {
            return Err(Error::Internal(
                "PdoProfile: velocity_window must be > 0".into(),
            ));
        }
        self.velocity_window = velocity_window;
        Ok(self)
    }

    pub const fn event_timer_ms(&self) -> u16 {
        self.event_timer_ms
    }

    pub const fn inhibit_time_x100us(&self) -> u16 {
        self.inhibit_time_x100us
    }

    pub const fn velocity_window(&self) -> Duration {
        self.velocity_window
    }

    /// 名义速率（Hz）。只用于日志和人类阅读 —— 权威值是 [`Self::event_timer_ms`]。
    pub fn hz(&self) -> f32 {
        1000.0 / self.event_timer_ms as f32
    }
}

impl Default for PdoProfile {
    /// 与 [`DEFAULT_TPDO1_COMM`] 逐位一致：1 ms / 0.5 ms inhibit / 15 ms 窗口。
    /// 即"没人冻结 profile"时的行为和引入本类型之前完全相同。
    fn default() -> Self {
        Self {
            event_timer_ms: Self::DEFAULT_EVENT_TIMER_MS,
            inhibit_time_x100us: derive_inhibit_time_x100us(Self::DEFAULT_EVENT_TIMER_MS),
            velocity_window: derive_velocity_window(Self::DEFAULT_EVENT_TIMER_MS),
        }
    }
}

impl std::fmt::Display for PdoProfile {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{:.1}Hz (event timer {}ms, inhibit {:.1}ms, velocity window {}ms)",
            self.hz(),
            self.event_timer_ms,
            self.inhibit_time_x100us as f32 / 10.0,
            self.velocity_window.as_millis(),
        )
    }
}

/// inhibit 取比周期略短半格：`timer×10 - 5`（单位 100 µs）。
///
/// timer = 1 ms 时正好得到 5，与历史上的 [`DEFAULT_TPDO1_COMM`] 完全一致 ——
/// 也就是引入派生之后，默认路径是恒等变换。
fn derive_inhibit_time_x100us(event_timer_ms: u16) -> u16 {
    event_timer_ms * 10 - 5
}

/// 窗口 = [`PdoProfile::VELOCITY_WINDOW_SAMPLES`] 个样本。
fn derive_velocity_window(event_timer_ms: u16) -> Duration {
    Duration::from_millis(event_timer_ms as u64 * PdoProfile::VELOCITY_WINDOW_SAMPLES as u64)
}

#[cfg(test)]
mod tests {
    use super::super::initialize::DEFAULT_TPDO1_COMM;
    use super::*;

    #[test]
    fn default_profile_reproduces_the_historical_tpdo1_constants() {
        // 这条断言是本次改动的"不改变默认行为"证明：任何调整了派生公式却没有
        // 同步 DEFAULT_TPDO1_COMM 的改动都会在这里失败。
        let p = PdoProfile::default();
        assert_eq!(p.event_timer_ms(), DEFAULT_TPDO1_COMM.event_timer_ms);
        assert_eq!(
            p.inhibit_time_x100us(),
            DEFAULT_TPDO1_COMM.inhibit_time_x100us
        );
        assert_eq!(p.velocity_window(), Duration::from_millis(15));
        assert_eq!(p.hz(), 1000.0);
    }

    #[test]
    fn derived_rates_cover_the_controller_policy_table() {
        // 控制器的判据表只会产生这三档；派生值必须跟着一起缩放。
        for (ms, hz, inhibit, window_ms) in [(1u16, 1000.0, 5u16, 15u64), (2, 500.0, 15, 30), (4, 250.0, 35, 60)] {
            let p = PdoProfile::from_event_timer_ms(ms).unwrap();
            assert_eq!(p.hz(), hz);
            assert_eq!(p.inhibit_time_x100us(), inhibit, "inhibit @ {ms}ms");
            assert_eq!(
                p.velocity_window(),
                Duration::from_millis(window_ms),
                "window @ {ms}ms"
            );
        }
    }

    #[test]
    fn inhibit_always_stays_within_one_period() {
        // 派生出的 inhibit 绝不能超过周期，否则限流器会压过定时器。
        for ms in 1..=PdoProfile::MAX_EVENT_TIMER_MS {
            let p = PdoProfile::from_event_timer_ms(ms).unwrap();
            assert!(
                p.inhibit_time_x100us() <= ms * 10,
                "inhibit {} exceeds period at {ms}ms",
                p.inhibit_time_x100us()
            );
        }
    }

    #[test]
    fn zero_event_timer_is_rejected() {
        // 0 在 CiA-301 里是"关掉定时触发"，静默接受它等于把节拍交给固件。
        assert!(PdoProfile::from_event_timer_ms(0).is_err());
        assert!(PdoProfile::from_event_timer_ms(PdoProfile::MAX_EVENT_TIMER_MS + 1).is_err());
    }

    #[test]
    fn overrides_are_validated_against_the_period() {
        let p = PdoProfile::from_event_timer_ms(4).unwrap();
        assert_eq!(p.with_inhibit_time_x100us(40).unwrap().inhibit_time_x100us(), 40);
        assert!(p.with_inhibit_time_x100us(41).is_err(), "inhibit > 周期必须拒绝");
        assert!(p.with_velocity_window(Duration::ZERO).is_err());
        assert_eq!(
            p.with_velocity_window(Duration::from_millis(80))
                .unwrap()
                .velocity_window(),
            Duration::from_millis(80)
        );
    }
}
