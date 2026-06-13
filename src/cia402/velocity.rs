//! 速度估计：基于电机时间戳 `0x1013`（μs）对**单圈位置**做解卷绕，再在滑动
//! 时间窗内做**最小二乘斜率**，得到平滑的 rev/s。
//!
//! 为什么不在电机端 map `0x606C` 速度、也不简单两帧差分：
//! - HexMeow 的 `0x6064` 是单圈 f32 `[-0.5, 0.5)`，跨圈会跳变，必须 host 侧
//!   累积成多圈位置才能差分。
//! - 单帧差分（`Δpos/Δt`）在 1 kHz + 编码器量化下噪声很大；对一个短时间窗
//!   做最小二乘斜率等价于一个带相位补偿的低通，既平滑又几乎无偏。
//!
//! 不连续保护：时间戳重复（Δt≤0）或出现大间隔（重连 / 初始化 / 长暂停，
//! 见 [`DISCONTINUITY_GAP_S`]）时丢弃历史、以当前帧为新基线重新累积。

use std::collections::VecDeque;
use std::time::Duration;

/// 相邻两帧时间戳差超过这个秒数，认为发生了不连续，重置滤波器。
/// 默认 TPDO1 是 1 ms 一帧；0.5 s 足够宽，不会误伤正常抖动。
const DISCONTINUITY_GAP_S: f64 = 0.5;

/// 每电机一个。`MotorEntryInner` 持有，TPDO1 listener 每帧 `update`。
#[derive(Debug, Default)]
pub(crate) struct VelocityEstimator {
    /// 上一帧的原始单圈位置（用于解卷绕）。
    last_raw_pos: Option<f32>,
    /// 上一帧的原始时间戳（用于 wrapping 求 Δt）。
    last_ts_us: Option<u32>,
    /// 累积多圈位置（rev）。
    accum_pos: f64,
    /// 累积时间（s），单调递增，作为最小二乘的横轴。
    accum_time_s: f64,
    /// 窗口内的 `(accum_time_s, accum_pos)` 样本。
    samples: VecDeque<(f64, f64)>,
}

impl VelocityEstimator {
    /// 喂一帧 `(电机时间戳 μs, 单圈位置 rev)`，返回滤波速度（rev/s）。
    /// 样本不足 / 刚重置时返回 `None`。
    pub fn update(&mut self, ts_us: u32, pos_rev: f32, window: Duration) -> Option<f32> {
        let (Some(last_raw), Some(last_ts)) = (self.last_raw_pos, self.last_ts_us) else {
            self.reset_to(ts_us, pos_rev);
            return None;
        };

        // wrapping_sub 处理 u32 μs 回绕；正常 Δt 很小所以差值就是真实间隔。
        let dt_s = ts_us.wrapping_sub(last_ts) as f64 * 1e-6;
        if dt_s <= 0.0 || dt_s > DISCONTINUITY_GAP_S {
            self.reset_to(ts_us, pos_rev);
            return None;
        }

        // 位置解卷绕：单圈 [-0.5, 0.5)，一帧内的真实位移不会超过半圈。
        let mut dpos = (pos_rev - last_raw) as f64;
        if dpos > 0.5 {
            dpos -= 1.0;
        } else if dpos < -0.5 {
            dpos += 1.0;
        }

        self.accum_pos += dpos;
        self.accum_time_s += dt_s;
        self.last_raw_pos = Some(pos_rev);
        self.last_ts_us = Some(ts_us);
        self.samples.push_back((self.accum_time_s, self.accum_pos));

        // 丢掉窗口外的旧样本，但至少留 2 个点（保证退化成两帧差分也能出值）。
        let cutoff = self.accum_time_s - window.as_secs_f64();
        while self.samples.len() > 2 {
            match self.samples.front() {
                Some(&(t, _)) if t < cutoff => {
                    self.samples.pop_front();
                }
                _ => break,
            }
        }

        self.least_squares_slope()
    }

    /// 把当前帧设成新基线，清空历史。
    fn reset_to(&mut self, ts_us: u32, pos_rev: f32) {
        self.last_raw_pos = Some(pos_rev);
        self.last_ts_us = Some(ts_us);
        self.accum_pos = 0.0;
        self.accum_time_s = 0.0;
        self.samples.clear();
        self.samples.push_back((0.0, 0.0));
    }

    /// 对窗口内 `(t, pos)` 做最小二乘，返回斜率（rev/s）。
    fn least_squares_slope(&self) -> Option<f32> {
        let n = self.samples.len();
        if n < 2 {
            return None;
        }
        let nf = n as f64;
        let (mut st, mut sp, mut stt, mut stp) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
        for &(t, p) in &self.samples {
            st += t;
            sp += p;
            stt += t * t;
            stp += t * p;
        }
        let denom = nf * stt - st * st;
        if denom.abs() < 1e-12 {
            return None;
        }
        let slope = (nf * stp - st * sp) / denom;
        if slope.is_finite() {
            Some(slope as f32)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const WIN: Duration = Duration::from_millis(20);

    #[test]
    fn first_sample_returns_none() {
        let mut e = VelocityEstimator::default();
        assert!(e.update(1000, 0.0, WIN).is_none());
    }

    #[test]
    fn constant_velocity_recovered() {
        // 1 kHz，每帧 +0.0003 rev → 0.3 rev/s。
        let mut e = VelocityEstimator::default();
        let mut pos: f32 = 0.0;
        let mut v = None;
        for i in 1..=50u32 {
            pos += 0.0003;
            // 保持在 [-0.5, 0.5)
            if pos >= 0.5 {
                pos -= 1.0;
            }
            v = e.update(i * 1000, pos, WIN);
        }
        let v = v.expect("should have a velocity after warmup");
        assert!((v - 0.3).abs() < 1e-3, "v={v}");
    }

    #[test]
    fn handles_single_turn_wrap() {
        // 位置从 0.49 走到 -0.49（其实是 +0.02），不应被当成 -0.98。
        let mut e = VelocityEstimator::default();
        e.update(1000, 0.49, WIN);
        let v = e.update(2000, -0.49, WIN); // Δt = 1ms, Δpos = +0.02 → 20 rev/s
        let v = v.expect("some velocity");
        assert!(v > 0.0, "wrap should give positive velocity, got {v}");
        assert!((v - 20.0).abs() < 1.0, "v={v}");
    }

    #[test]
    fn large_gap_resets() {
        let mut e = VelocityEstimator::default();
        e.update(1000, 0.0, WIN);
        e.update(2000, 0.001, WIN);
        // 1 秒大间隔 → 重置，返回 None。
        assert!(e.update(2000 + 1_000_000, 0.5, WIN).is_none());
    }

    #[test]
    fn timestamp_wraparound_is_handled() {
        let mut e = VelocityEstimator::default();
        let near_max = u32::MAX - 500; // 距回绕 500 μs
        e.update(near_max, 0.0, WIN);
        // 回绕后 +500μs：真实 Δt = 1000μs, Δpos = +0.0003 → 0.3 rev/s
        let v = e.update(near_max.wrapping_add(1000), 0.0003, WIN);
        let v = v.expect("velocity across wrap");
        assert!((v - 0.3).abs() < 1e-2, "v={v}");
    }
}
