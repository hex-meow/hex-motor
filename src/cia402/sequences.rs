//! 纯函数：CiA402 控制类操作的 SDO 序列规划。
//!
//! 不做 I/O。Manager 拿到 `Vec<SdoWrite>` 后用 `canopen::sdo::download` 逐条
//! 下发，每条之间按 [`INTER_WRITE_DELAY`] (默认 10 ms) 给电机一点时间应用变化。
//!
//! ## 设计基线 —— "统一保险路径"
//!
//! v0.1 不去判"当前状态是否允许直接切",  统一走最保险的顺序：
//! `(可选 fault reset) → 失能 → 写模式 → CiA402 enable ramp`。
//! 多几个 SDO 不影响上位机使用。详见 `DESIGN.md` §8。

use std::time::Duration;

use crate::canopen::tpdo_config::SdoWrite;
use crate::error::{Error, Result};
use crate::types::{MotorMode, MotorTarget};

use super::codec::mode_to_cia402_code;
use super::types::Logic;

/// 控制字 `0x6040:00`。
const OD_CONTROL_WORD: u16 = 0x6040;
/// 操作模式 `0x6060:00`（i8）。
const OD_MODE_OF_OPERATION: u16 = 0x6060;
/// PP 目标位置 `0x607A:00`。HexMeow CiA402 按 **f32 Rev** 写（vendor-specific；
/// 标准 CiA402 是 i32 user-units）。
const OD_TARGET_POSITION: u16 = 0x607A;
/// PT 目标力矩 `0x6071:00`，i16 = ‰ of peak_torque (`0x6076`)，范围 -1000..=1000。
const OD_TARGET_TORQUE: u16 = 0x6071;
/// PV 速度目标 `0x60FF:00`。HexMeow CiA402 电机按 **f32 Rev/s** 写
/// （vendor-specific；标准 CiA402 是 i32 user-units）。
const OD_TARGET_VELOCITY: u16 = 0x60FF;
/// MIT 控制参数 `0x2003`（uncompressed REAL32 形态，子项见 [`mit`]）。
const OD_MIT_CONTROL_PARAM: u16 = 0x2003;
mod mit {
    pub const SUB_POSITION: u8 = 0x01; // REAL32, Rev
    pub const SUB_VELOCITY: u8 = 0x02; // REAL32, Rev/s
    pub const SUB_TORQUE: u8 = 0x03; // REAL32, Nm (feedforward)
    pub const SUB_KP: u8 = 0x04; // UNSIGNED16, 0..=10000 (kp_int)
    pub const SUB_KD: u8 = 0x05; // UNSIGNED16, 0..=10000 (kd_int)
}

/// CiA402 控制字常用值。
mod cw {
    /// `0x06`: Shutdown.
    pub const SHUTDOWN: u16 = 0x0006;
    /// `0x07`: Switch On.
    pub const SWITCH_ON: u16 = 0x0007;
    /// `0x0F`: Enable Operation（bits 0..3 = 1）。
    pub const ENABLE_OPERATION: u16 = 0x000F;
    /// `0x80`: Fault Reset (bit 7)。
    pub const FAULT_RESET: u16 = 0x0080;
    /// `0x2F = 0x0F | bit5`：Enable Operation + Change Set Immediately
    /// (PP mode)。每次 `set_target(Position)` 前先写这个把 bit4 落回 0。
    pub const ENABLE_PP_NEW_SP_CLEARED: u16 = 0x002F;
    /// `0x3F = 0x0F | bit4 | bit5`：Enable Operation + New Set-Point +
    /// Change Set Immediately (PP mode)。bit4 的上升沿告诉电机"收下新
    /// 目标"。
    pub const ENABLE_PP_NEW_SP_LATCHED: u16 = 0x003F;
}

/// `set_target` 时除 target 本身外需要从 Manager 缓存里取的上下文。
///
/// `current_mode` 必填（从 [`MotorEntry`] 的 `target_mode` 取）；其它
/// 字段只有相应模式才需要：
/// - `peak_torque_nm`：`Torque` target 把 Nm → `0x6071` i16 ‰ 时用
/// - `mit_kp_kd_factor`：`Mit` target 把 Nm/Rev → `0x2003:04/05` u16
///   `kp_int`/`kd_int` 时用
///
/// [`MotorEntry`]: crate::cia402::motor_entry::MotorEntry
#[derive(Debug, Clone, Copy, Default)]
pub struct SetTargetContext {
    pub current_mode: Option<MotorMode>,
    pub peak_torque_nm: Option<f32>,
    pub mit_kp_kd_factor: Option<f32>,
}

/// 两次 SDO 写之间建议的 settle 间隔。
///
/// CiA402 控制字 ramp 的状态机切换不是瞬时的；10 ms 是经验值，保证最坏
/// 情况也能稳定地观察到上一条 SDO 的效果再下一条。
pub const INTER_WRITE_DELAY: Duration = Duration::from_millis(10);

/// `set_mode`：把电机切到指定模式。
///
/// 序列（统一保险路径）：
/// 1. 若 `current_logic` 是 `Logic::Error`，前置 `CW=0x80` (fault reset)
/// 2. `CW=0x06` (Shutdown → SwitchOnDisabled / ReadyToSwitchOn)
/// 3. `0x6060 = mode_code`（写期望模式）
/// 4. `CW=0x06`（再来一次 Shutdown，确保 ReadyToSwitchOn）
/// 5. `CW=0x07` (SwitchOn → SwitchedOn)
/// 6. `CW=0x0F` (EnableOperation → OperationEnabled)
pub fn build_set_mode_writes(target: MotorMode, current_logic: Option<&Logic>) -> Vec<SdoWrite> {
    let mut out = Vec::with_capacity(7);
    if matches!(current_logic, Some(Logic::Error { .. })) {
        out.push(SdoWrite::u16(OD_CONTROL_WORD, 0, cw::FAULT_RESET));
    }
    out.push(SdoWrite::u16(OD_CONTROL_WORD, 0, cw::SHUTDOWN));
    out.push(SdoWrite::i8(
        OD_MODE_OF_OPERATION,
        0,
        mode_to_cia402_code(target),
    ));
    out.push(SdoWrite::u16(OD_CONTROL_WORD, 0, cw::SHUTDOWN));
    out.push(SdoWrite::u16(OD_CONTROL_WORD, 0, cw::SWITCH_ON));
    out.push(SdoWrite::u16(OD_CONTROL_WORD, 0, cw::ENABLE_OPERATION));
    out
}

/// `disable`：写控制字 `0x06`（短刹车 / 移出 OperationEnabled）。
pub fn build_disable_writes() -> Vec<SdoWrite> {
    vec![SdoWrite::u16(OD_CONTROL_WORD, 0, cw::SHUTDOWN)]
}

/// `clear_error`：写控制字 `0x80` (fault reset)。
pub fn build_clear_error_writes() -> Vec<SdoWrite> {
    vec![SdoWrite::u16(OD_CONTROL_WORD, 0, cw::FAULT_RESET)]
}

/// `set_target`：构造往电机写目标值需要的 SDO 序列。
///
/// `ctx.current_mode` 是 Manager 缓存的"上一次 `set_mode` 设的模式"。
/// `target` 的 enum variant 必须和它匹配，否则返回
/// [`Error::TargetModeMismatch`]。
///
/// **v0.1 全模式覆盖**（HexMeow CiA402 电机约定）：
///
/// | target / 当前模式 | SDO 序列 |
/// |---|---|
/// | `Disable` (任意模式) | `0x6040 = 0x06` |
/// | `Velocity{rev_per_s}` + `ProfileVelocity` | `0x60FF = f32(rev_per_s)` |
/// | `Position{rev}` + `ProfilePosition` | `0x6040 = 0x2F`（清掉 bit4） → `0x607A = f32(rev)` → `0x6040 = 0x3F`（bit4 上升沿，Change Set Immediately） |
/// | `Torque{nm}` + `Torque` | `0x6071 = i16(round(nm / peak * 1000))`，需要 `ctx.peak_torque_nm` |
/// | `Mit{pos,vel,tor,kp,kd}` + `Mit` | `0x2003:01 = f32(pos)` → `:02 = f32(vel)` → `:03 = f32(tor)` → `:04 = u16(round(kp / factor))` → `:05 = u16(round(kd / factor))`，需要 `ctx.mit_kp_kd_factor` |
///
/// 缺少必需的 `peak_torque_nm` / `mit_kp_kd_factor` 时返回 [`Error::Internal`]
/// （通常意味着 `initialize()` 时电机没暴露 `0x6076` / `0x2003:07`）。
pub fn build_set_target_writes(
    target: &MotorTarget,
    ctx: SetTargetContext,
) -> Result<Vec<SdoWrite>> {
    // Disable 全模式通用。
    if matches!(target, MotorTarget::Disable) {
        return Ok(build_disable_writes());
    }

    let Some(mode) = ctx.current_mode else {
        return Err(Error::Internal(
            "set_target: motor mode unknown (call set_mode first)".into(),
        ));
    };

    if !target.matches_mode(mode) {
        return Err(Error::TargetModeMismatch {
            expected: format!("{:?}", mode),
            given: target.variant_name(),
        });
    }

    match (target, mode) {
        (MotorTarget::Velocity { rev_per_s }, MotorMode::ProfileVelocity) => {
            Ok(vec![SdoWrite::f32(OD_TARGET_VELOCITY, 0, *rev_per_s)])
        }
        (MotorTarget::Position { rev }, MotorMode::ProfilePosition) => {
            Ok(build_pp_position_writes(*rev))
        }
        (MotorTarget::Torque { nm }, MotorMode::Torque) => build_torque_writes(*nm, &ctx),
        (MotorTarget::Mit { pos, vel, tor, kp, kd }, MotorMode::Mit) => {
            build_mit_writes(*pos, *vel, *tor, *kp, *kd, &ctx)
        }
        // 已被上面的 matches_mode 拒绝过；这里是 exhaustiveness 兜底。
        _ => Err(Error::TargetModeMismatch {
            expected: format!("{:?}", mode),
            given: target.variant_name(),
        }),
    }
}

/// `set_target(Position)` 的 PP / Change-Set-Immediately 序列。详见
/// CiA402 §6.4.2.1 (Profile Position Mode)。
///
/// 三条写：
/// 1. CW = `0x002F` —— 把 bit4 (`new_setpoint`) 落回 0，保持 enable + CSI
///    bit5 = 1。第一次调用时上一条 CW 来自 `set_mode` 末尾的 `0x000F`，
///    所以 bit4 本来就是 0；但写一次保证之后多次调用的 bit4 0→1 上升沿都
///    存在。
/// 2. `0x607A` = f32 Rev —— 新目标位置（vendor-specific f32；标准 CiA402 是
///    i32 user-units）。
/// 3. CW = `0x003F` —— bit4 0→1 上升沿告诉电机"latch 新目标"；CSI bit5 = 1
///    要求立刻替换当前目标，不等当前 motion profile 跑完。
fn build_pp_position_writes(rev: f32) -> Vec<SdoWrite> {
    vec![
        SdoWrite::u16(OD_CONTROL_WORD, 0, cw::ENABLE_PP_NEW_SP_CLEARED),
        SdoWrite::f32(OD_TARGET_POSITION, 0, rev),
        SdoWrite::u16(OD_CONTROL_WORD, 0, cw::ENABLE_PP_NEW_SP_LATCHED),
    ]
}

/// `set_target(Torque)`：Nm → i16 (‰ of peak_torque)。
fn build_torque_writes(nm: f32, ctx: &SetTargetContext) -> Result<Vec<SdoWrite>> {
    let peak = ctx.peak_torque_nm.ok_or_else(|| {
        Error::Internal(
            "set_target(Torque): peak_torque not cached. \
             initialize() must read 0x6076 first; this motor may not expose it."
                .into(),
        )
    })?;
    if !peak.is_finite() || peak.abs() < f32::EPSILON {
        return Err(Error::Internal(format!(
            "set_target(Torque): cached peak_torque is {peak} Nm; cannot convert"
        )));
    }
    let permille = (nm / peak * 1000.0).round();
    let clamped = permille.clamp(-1000.0, 1000.0) as i16;
    Ok(vec![SdoWrite::i16(OD_TARGET_TORQUE, 0, clamped)])
}

/// `set_target(Mit)`：写 0x2003:01..=05 五条。
///
/// - kp/kd 物理单位 [Nm/Rev] / [Nm·s/Rev] → u16 dimensionless：
///   `kp_int = round(kp_phys / factor)`，clamp 到 `0..=10000`。
fn build_mit_writes(
    pos: f32,
    vel: f32,
    tor: f32,
    kp: f32,
    kd: f32,
    ctx: &SetTargetContext,
) -> Result<Vec<SdoWrite>> {
    let factor = ctx.mit_kp_kd_factor.ok_or_else(|| {
        Error::Internal(
            "set_target(Mit): mit_kp_kd_factor not cached. \
             initialize() must read 0x2003:07 first; this motor may not expose it."
                .into(),
        )
    })?;
    if !factor.is_finite() || factor.abs() < f32::EPSILON {
        return Err(Error::Internal(format!(
            "set_target(Mit): cached mit_kp_kd_factor is {factor}; cannot convert"
        )));
    }
    let kp_int = (kp / factor).round().clamp(0.0, u16::MAX as f32) as u16;
    let kd_int = (kd / factor).round().clamp(0.0, u16::MAX as f32) as u16;
    // OD-08 文档说 KP / KD 范围 0..=10000；再加一层 clamp 保险。
    let kp_int = kp_int.min(10_000);
    let kd_int = kd_int.min(10_000);
    Ok(vec![
        SdoWrite::f32(OD_MIT_CONTROL_PARAM, mit::SUB_POSITION, pos),
        SdoWrite::f32(OD_MIT_CONTROL_PARAM, mit::SUB_VELOCITY, vel),
        SdoWrite::f32(OD_MIT_CONTROL_PARAM, mit::SUB_TORQUE, tor),
        SdoWrite::u16(OD_MIT_CONTROL_PARAM, mit::SUB_KP, kp_int),
        SdoWrite::u16(OD_MIT_CONTROL_PARAM, mit::SUB_KD, kd_int),
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cw_writes_only(writes: &[SdoWrite]) -> Vec<u16> {
        writes
            .iter()
            .filter(|w| w.index == OD_CONTROL_WORD)
            .map(|w| u16::from_le_bytes([w.data[0], w.data[1]]))
            .collect()
    }

    #[test]
    fn set_mode_default_ramp_no_fault_reset() {
        let w = build_set_mode_writes(MotorMode::ProfileVelocity, None);
        // CW=0x06, 0x6060=3, CW=0x06, CW=0x07, CW=0x0F
        assert_eq!(w.len(), 5);
        assert_eq!(
            cw_writes_only(&w),
            vec![cw::SHUTDOWN, cw::SHUTDOWN, cw::SWITCH_ON, cw::ENABLE_OPERATION]
        );
        let mode_w = &w[1];
        assert_eq!(mode_w.index, OD_MODE_OF_OPERATION);
        assert_eq!(mode_w.data, vec![3u8]);
    }

    #[test]
    fn set_mode_prepends_fault_reset_when_in_error() {
        let logic = Logic::Error {
            kind: crate::types::MotorErrorKind::OverCurrent,
            raw_code: 0x2310,
        };
        let w = build_set_mode_writes(MotorMode::Mit, Some(&logic));
        assert_eq!(w.len(), 6);
        // 第一条必须是 fault reset
        assert_eq!(
            u16::from_le_bytes([w[0].data[0], w[0].data[1]]),
            cw::FAULT_RESET
        );
        // 第三条应该是 0x6060 = 5 (Mit)
        assert_eq!(w[2].index, OD_MODE_OF_OPERATION);
        assert_eq!(w[2].data, vec![5u8]);
    }

    #[test]
    fn set_mode_for_each_mode_writes_correct_code() {
        for (m, code) in [
            (MotorMode::ProfilePosition, 1u8),
            (MotorMode::ProfileVelocity, 3),
            (MotorMode::Torque, 4),
            (MotorMode::Mit, 5),
        ] {
            let w = build_set_mode_writes(m, None);
            let mode_w = w.iter().find(|w| w.index == OD_MODE_OF_OPERATION).unwrap();
            assert_eq!(mode_w.data[0], code, "mode {m:?}");
        }
    }

    #[test]
    fn disable_is_single_shutdown() {
        let w = build_disable_writes();
        assert_eq!(w.len(), 1);
        assert_eq!(w[0].index, OD_CONTROL_WORD);
        assert_eq!(u16::from_le_bytes([w[0].data[0], w[0].data[1]]), cw::SHUTDOWN);
    }

    #[test]
    fn clear_error_is_single_fault_reset() {
        let w = build_clear_error_writes();
        assert_eq!(w.len(), 1);
        assert_eq!(
            u16::from_le_bytes([w[0].data[0], w[0].data[1]]),
            cw::FAULT_RESET
        );
    }

    fn ctx_pv() -> SetTargetContext {
        SetTargetContext {
            current_mode: Some(MotorMode::ProfileVelocity),
            ..Default::default()
        }
    }

    #[test]
    fn target_disable_works_in_any_mode() {
        assert!(build_set_target_writes(&MotorTarget::Disable, SetTargetContext::default()).is_ok());
        assert!(build_set_target_writes(&MotorTarget::Disable, ctx_pv()).is_ok());
    }

    #[test]
    fn target_velocity_in_pv_mode_writes_60ff_f32() {
        let w = build_set_target_writes(&MotorTarget::Velocity { rev_per_s: 1.5 }, ctx_pv()).unwrap();
        assert_eq!(w.len(), 1);
        assert_eq!(w[0].index, OD_TARGET_VELOCITY);
        assert_eq!(w[0].subindex, 0);
        assert_eq!(w[0].data.len(), 4);
        let v = f32::from_le_bytes([w[0].data[0], w[0].data[1], w[0].data[2], w[0].data[3]]);
        assert!((v - 1.5).abs() < f32::EPSILON);
    }

    #[test]
    fn target_velocity_without_known_mode_errs() {
        let r = build_set_target_writes(
            &MotorTarget::Velocity { rev_per_s: 1.0 },
            SetTargetContext::default(),
        );
        assert!(matches!(r, Err(Error::Internal(_))));
    }

    #[test]
    fn target_velocity_in_wrong_mode_errs() {
        let r = build_set_target_writes(
            &MotorTarget::Velocity { rev_per_s: 1.0 },
            SetTargetContext {
                current_mode: Some(MotorMode::Torque),
                ..Default::default()
            },
        );
        assert!(matches!(r, Err(Error::TargetModeMismatch { .. })));
    }

    // ===== Profile Position =====

    #[test]
    fn target_position_in_pp_mode_emits_csi_handshake() {
        let w = build_set_target_writes(
            &MotorTarget::Position { rev: 0.25 },
            SetTargetContext {
                current_mode: Some(MotorMode::ProfilePosition),
                ..Default::default()
            },
        )
        .unwrap();
        // 三条：CW=0x2F, 0x607A=f32(0.25), CW=0x3F
        assert_eq!(w.len(), 3);
        assert_eq!(w[0].index, OD_CONTROL_WORD);
        assert_eq!(u16::from_le_bytes([w[0].data[0], w[0].data[1]]), 0x002F);
        assert_eq!(w[1].index, OD_TARGET_POSITION);
        let pos = f32::from_le_bytes([w[1].data[0], w[1].data[1], w[1].data[2], w[1].data[3]]);
        assert!((pos - 0.25).abs() < f32::EPSILON);
        assert_eq!(w[2].index, OD_CONTROL_WORD);
        assert_eq!(u16::from_le_bytes([w[2].data[0], w[2].data[1]]), 0x003F);
    }

    #[test]
    fn target_position_in_wrong_mode_errs() {
        let r = build_set_target_writes(
            &MotorTarget::Position { rev: 0.5 },
            SetTargetContext {
                current_mode: Some(MotorMode::ProfileVelocity),
                ..Default::default()
            },
        );
        assert!(matches!(r, Err(Error::TargetModeMismatch { .. })));
    }

    // ===== Torque =====

    #[test]
    fn target_torque_converts_nm_to_permille_of_peak() {
        // peak = 4 Nm，target = 1 Nm → 250 ‰
        let w = build_set_target_writes(
            &MotorTarget::Torque { nm: 1.0 },
            SetTargetContext {
                current_mode: Some(MotorMode::Torque),
                peak_torque_nm: Some(4.0),
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(w.len(), 1);
        assert_eq!(w[0].index, OD_TARGET_TORQUE);
        let v = i16::from_le_bytes([w[0].data[0], w[0].data[1]]);
        assert_eq!(v, 250);
    }

    #[test]
    fn target_torque_clamps_to_plus_minus_1000_permille() {
        let w = build_set_target_writes(
            &MotorTarget::Torque { nm: 99.0 },
            SetTargetContext {
                current_mode: Some(MotorMode::Torque),
                peak_torque_nm: Some(4.0),
                ..Default::default()
            },
        )
        .unwrap();
        let v = i16::from_le_bytes([w[0].data[0], w[0].data[1]]);
        assert_eq!(v, 1000);

        let w = build_set_target_writes(
            &MotorTarget::Torque { nm: -99.0 },
            SetTargetContext {
                current_mode: Some(MotorMode::Torque),
                peak_torque_nm: Some(4.0),
                ..Default::default()
            },
        )
        .unwrap();
        let v = i16::from_le_bytes([w[0].data[0], w[0].data[1]]);
        assert_eq!(v, -1000);
    }

    #[test]
    fn target_torque_without_peak_cached_errs() {
        let r = build_set_target_writes(
            &MotorTarget::Torque { nm: 1.0 },
            SetTargetContext {
                current_mode: Some(MotorMode::Torque),
                peak_torque_nm: None,
                ..Default::default()
            },
        );
        assert!(matches!(r, Err(Error::Internal(_))));
    }

    // ===== MIT =====

    #[test]
    fn target_mit_emits_five_writes_with_kp_kd_converted() {
        // factor = 0.01 means kp_int = round(kp_phys / 0.01) = round(kp_phys * 100)
        let w = build_set_target_writes(
            &MotorTarget::Mit {
                pos: 0.1,
                vel: 0.2,
                tor: 0.3,
                kp: 5.0,   // → 500
                kd: 0.5,   // → 50
            },
            SetTargetContext {
                current_mode: Some(MotorMode::Mit),
                mit_kp_kd_factor: Some(0.01),
                ..Default::default()
            },
        )
        .unwrap();
        assert_eq!(w.len(), 5);
        assert_eq!(w[0].index, 0x2003);
        assert_eq!(w[0].subindex, 0x01);
        let pos = f32::from_le_bytes([w[0].data[0], w[0].data[1], w[0].data[2], w[0].data[3]]);
        assert!((pos - 0.1).abs() < 1e-6);
        assert_eq!(w[3].subindex, 0x04);
        let kp = u16::from_le_bytes([w[3].data[0], w[3].data[1]]);
        assert_eq!(kp, 500);
        assert_eq!(w[4].subindex, 0x05);
        let kd = u16::from_le_bytes([w[4].data[0], w[4].data[1]]);
        assert_eq!(kd, 50);
    }

    #[test]
    fn target_mit_clamps_kp_to_10000() {
        let w = build_set_target_writes(
            &MotorTarget::Mit {
                pos: 0.0,
                vel: 0.0,
                tor: 0.0,
                kp: 1e6,
                kd: 0.0,
            },
            SetTargetContext {
                current_mode: Some(MotorMode::Mit),
                mit_kp_kd_factor: Some(0.01),
                ..Default::default()
            },
        )
        .unwrap();
        let kp = u16::from_le_bytes([w[3].data[0], w[3].data[1]]);
        assert_eq!(kp, 10_000);
    }

    #[test]
    fn target_mit_without_factor_cached_errs() {
        let r = build_set_target_writes(
            &MotorTarget::Mit {
                pos: 0.0, vel: 0.0, tor: 0.0, kp: 1.0, kd: 0.0,
            },
            SetTargetContext {
                current_mode: Some(MotorMode::Mit),
                mit_kp_kd_factor: None,
                ..Default::default()
            },
        );
        assert!(matches!(r, Err(Error::Internal(_))));
    }
}
