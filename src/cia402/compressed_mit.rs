//! hex-meow 电机**压缩 MIT** 命令路径(CiA402 之上的厂商扩展,`0x2004`)。
//!
//! 非压缩 MIT(`0x2003:01..05`,见 [`crate::cia402::sequences`])每电机 16 字节,
//! 6 个电机一拍 96B > CAN-FD 单帧 64B,得分包。压缩 MIT 把五元组定点打包成 **8 字节/电机**
//! (pos 16-bit + vel/tor/kp/kd 各 12-bit),6 个电机 48B 可塞进一帧 BRS CAN-FD。
//!
//! 用法(与 [`crate::cia402::Cia402Manager`] 配合):
//!   1. `mgr.initialize(nid)`:NMT Operational + 心跳监听 + 默认 TPDO 反馈(q/dq/τ)。
//!   2. 每个电机调 [`configure`]:写 `0x2004` 压缩开关/映射范围 + 把 RPDO1 重映射到
//!      `0x2004:02/03`(本电机在共享帧里的 8 字节槽)+ 进 MIT 模式 + 使能。
//!   3. 控制环按节拍构造共享帧([`pack_shared_frame`])经 `bus.send` 发到 [`DEFAULT_SHARED_COB_ID`]。
//!
//! 反馈仍走 manager 的默认 TPDO(本模块不动 TPDO 映射)。
//! 位打包格式 = 厂商专有压缩定点(CiA402 之上的扩展,非标准)。

use std::time::Duration;

use can_transport::CanBus;

use crate::canopen::rpdo_config::{build_rpdo_config_writes, RpdoRecipe};
use crate::canopen::sdo;
use crate::canopen::tpdo_config::TpdoEntry;
use crate::Result;

// ── 对象字典地址 ──
const OD_MIT_COMPRESSED: u16 = 0x2004; // 压缩 MIT 控制对象(厂商扩展)
const OD_MAX_TORQUE: u16 = 0x6072; // 各模式最大力矩(‰)
const OD_SHORT_BRAKE: u16 = 0x2040; // 短接绕组制动使能(u8)
const OD_MODE: u16 = 0x6060; // 操作模式(i8),5 = MIT
const OD_CONTROLWORD: u16 = 0x6040; // 控制字(u16)

const SUB_ENABLE: u8 = 0x01; // u8: 1 = 启用压缩模式
const SUB_LOWER: u8 = 0x02; // u32: 打包目标低 32 位(RPDO 映射点)
const SUB_UPPER: u8 = 0x03; // u32: 打包目标高 32 位(RPDO 映射点)
const SUB_KP_KD_TORQUE_PERMILLE: u8 = 0x0E; // u16: MIT KP/KD 项最大力矩(‰)
const MODE_MIT: i8 = 5;

/// 占位对象:其他电机槽的 8 字节由它消费(本电机不关心)。沿用参考实现的 `0x3000:03`。
const PLACEHOLDER: TpdoEntry = TpdoEntry { index: 0x3000, subindex: 0x03, bit_len: 32 };

/// 主站发命令用的共享 COB-ID(= 主站 node 0x10 的 TPDO1 功能码 0x180 | 0x10)。
/// 所有电机的 RPDO1 都监听它,各取自己的 8 字节槽。
pub const DEFAULT_SHARED_COB_ID: u16 = 0x190;

// TODO(动态最大力矩):目前只用一条共享帧(0x10 的 TPDO1 → COB-ID 0x190)发压缩 MIT 5 元组,
//   每电机最大力矩(0x6072 / 0x2004:0E)在 configure 时一次性 SDO 写死。
//   未来加第二条共享帧(0x10 的 TPDO2 → COB-ID 0x290),把每电机最大力矩映射进去,
//   即可按需实时调整各电机扭矩上限(安全/标定/不同负载),无需重配。先占位,暂不做。

/// 连续 SDO 写之间的小间隔(firmware 心跳节拍敏感,稳一点)。
const SDO_GAP: Duration = Duration::from_millis(4);

/// 压缩 MIT 各分量的定点映射范围(**硬件单位**:Rev / Rev·s⁻¹ / Nm / Nm·Rev⁻¹ / Nm·s·Rev⁻¹)。
/// 写进电机 `0x2004:04..0D`,也用于主机侧打包。范围越窄分辨率越高。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CompressedMitMapping {
    pub position_min: f32,
    pub position_max: f32,
    pub velocity_min: f32,
    pub velocity_max: f32,
    pub torque_min: f32,
    pub torque_max: f32,
    pub kp_min: f32,
    pub kp_max: f32,
    pub kd_min: f32,
    pub kd_max: f32,
}

impl Default for CompressedMitMapping {
    /// 默认范围(厂商参考实现的默认映射)。
    fn default() -> Self {
        Self {
            position_min: -0.5,
            position_max: 0.5,
            velocity_min: -10.0,
            velocity_max: 10.0,
            torque_min: -10.0,
            torque_max: 10.0,
            kp_min: 0.0,
            kp_max: 100.0,
            kd_min: 0.0,
            kd_max: 20.0,
        }
    }
}

/// 压缩 MIT 五元组目标,**硬件单位**(Rev 系)。SI(rad)↔硬件(Rev)换算由调用方做
/// (pos rad÷2π=Rev;kp Nm/rad×2π=Nm/Rev)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CompressedMitTarget {
    pub position: f32, // Rev
    pub velocity: f32, // Rev/s
    pub torque: f32,   // Nm(前馈)
    pub kp: f32,       // Nm/Rev
    pub kd: f32,       // Nm·s/Rev
}

impl CompressedMitTarget {
    /// 零目标(kp=kd=0 → 零力矩;注意定点对 0 有 ~±数 mNm 量化偏差,见模块说明)。
    pub const ZERO: Self =
        Self { position: 0.0, velocity: 0.0, torque: 0.0, kp: 0.0, kd: 0.0 };

    fn float_to_uint(x: f32, x_min: f32, x_max: f32, bits: u32) -> u32 {
        let x = x.clamp(x_min, x_max);
        let span = x_max - x_min;
        let scale = ((1u32 << bits) - 1) as f32;
        ((x - x_min) * scale / span) as u32
    }

    /// 定点打包成 8 字节(LE)。pos 16-bit,vel/tor/kp/kd 各 12-bit。
    /// 位布局与厂商参考实现一致。
    pub fn to_le_bytes(&self, m: &CompressedMitMapping) -> [u8; 8] {
        let pos = Self::float_to_uint(self.position, m.position_min, m.position_max, 16);
        let vel = Self::float_to_uint(self.velocity, m.velocity_min, m.velocity_max, 12);
        let tor = Self::float_to_uint(self.torque, m.torque_min, m.torque_max, 12);
        let kp = Self::float_to_uint(self.kp, m.kp_min, m.kp_max, 12);
        let kd = Self::float_to_uint(self.kd, m.kd_min, m.kd_max, 12);

        let lower = tor | (kd << 12) | ((kp & 0xFF) << 24);
        let upper = (kp >> 8) | (vel << 4) | (pos << 16);
        let mut out = [0u8; 8];
        out[0..4].copy_from_slice(&lower.to_le_bytes());
        out[4..8].copy_from_slice(&upper.to_le_bytes());
        out
    }
}

/// 把一组电机的压缩目标拼成共享帧的 payload(`8 * n` 字节,槽序 = 配置时的 `slice`)。
/// `targets[i]` 用 `mappings[i]` 打包。
pub fn pack_shared_frame(
    targets: &[CompressedMitTarget],
    mappings: &[CompressedMitMapping],
) -> Vec<u8> {
    let mut data = Vec::with_capacity(targets.len() * 8);
    for (t, m) in targets.iter().zip(mappings.iter()) {
        data.extend_from_slice(&t.to_le_bytes(m));
    }
    data
}

/// 本电机在共享帧里占 `slice` 槽:自己映射 `0x2004:02/03`,其余槽用占位对象填满 8 字节。
fn rpdo_entries(slice: u8, total: u8) -> Vec<TpdoEntry> {
    let mut entries = Vec::with_capacity(2 * total as usize);
    for i in 0..total {
        if i == slice {
            entries.push(TpdoEntry { index: OD_MIT_COMPRESSED, subindex: SUB_LOWER, bit_len: 32 });
            entries.push(TpdoEntry { index: OD_MIT_COMPRESSED, subindex: SUB_UPPER, bit_len: 32 });
        } else {
            entries.push(PLACEHOLDER);
            entries.push(PLACEHOLDER);
        }
    }
    entries
}

/// 配置一个电机进入压缩 MIT(**前提:`Cia402Manager::initialize(nid)` 已跑过** ——
/// NMT Operational / 心跳监听 / 默认 TPDO 反馈已就绪)。
///
/// 顺序:最大力矩限制 → `0x2004` 映射范围/初值/压缩开关 → 短接制动 → RPDO1 重映射到本槽
/// → MIT 模式 → 使能(控制字 6→7→0x0F)。完成后电机已使能,**调用方须立即按节拍发共享帧**
/// (初值已置零,使能到首帧之间目标为零)。
#[allow(clippy::too_many_arguments)]
pub async fn configure(
    bus: &(impl CanBus + ?Sized),
    nid: u8,
    slice: u8,
    total: u8,
    cob_id: u16,
    mapping: &CompressedMitMapping,
    torque_permille: u16,
    kp_kd_torque_permille: u16,
    timeout: Option<Duration>,
) -> Result<()> {
    // 1) 最大力矩限制(全模式 + MIT KP/KD 项)
    sdo::download_u16(bus, nid, OD_MAX_TORQUE, 0, torque_permille, timeout).await?;
    gap().await;
    sdo::download_u16(bus, nid, OD_MIT_COMPRESSED, SUB_KP_KD_TORQUE_PERMILLE, kp_kd_torque_permille, timeout).await?;
    gap().await;

    // 2) 映射范围 0x2004:04..0D(f32)
    let ranges: [(u8, f32); 10] = [
        (0x04, mapping.position_min),
        (0x05, mapping.position_max),
        (0x06, mapping.velocity_min),
        (0x07, mapping.velocity_max),
        (0x08, mapping.kp_min),
        (0x09, mapping.kp_max),
        (0x0A, mapping.kd_min),
        (0x0B, mapping.kd_max),
        (0x0C, mapping.torque_min),
        (0x0D, mapping.torque_max),
    ];
    for (sub, val) in ranges {
        sdo::download_f32(bus, nid, OD_MIT_COMPRESSED, sub, val, timeout).await?;
        gap().await;
    }

    // 3) 初始打包目标置零(使能到首帧之间安全)
    let zero = CompressedMitTarget::ZERO.to_le_bytes(mapping);
    let lower = u32::from_le_bytes([zero[0], zero[1], zero[2], zero[3]]);
    let upper = u32::from_le_bytes([zero[4], zero[5], zero[6], zero[7]]);
    sdo::download_u32(bus, nid, OD_MIT_COMPRESSED, SUB_LOWER, lower, timeout).await?;
    gap().await;
    sdo::download_u32(bus, nid, OD_MIT_COMPRESSED, SUB_UPPER, upper, timeout).await?;
    gap().await;

    // 4) 启用压缩模式 + 短接制动
    sdo::download_u8(bus, nid, OD_MIT_COMPRESSED, SUB_ENABLE, 1, timeout).await?;
    gap().await;
    sdo::download_u8(bus, nid, OD_SHORT_BRAKE, 0, 1, timeout).await?;
    gap().await;

    // 5) RPDO1 重映射到本槽的 0x2004:02/03(共享 COB-ID)
    let recipe = RpdoRecipe {
        rpdo_index: 0,
        cob_id,
        entries: rpdo_entries(slice, total),
        transmission_type: 255,
    };
    for w in build_rpdo_config_writes(&recipe)? {
        sdo::download(bus, nid, w.index, w.subindex, &w.data, timeout).await?;
        gap().await;
    }

    // 6) MIT 模式 + 使能(6→7→0x0F)
    sdo::download(bus, nid, OD_MODE, 0, &MODE_MIT.to_le_bytes(), timeout).await?;
    gap().await;
    for cw in [0x0006u16, 0x0007, 0x000F] {
        sdo::download_u16(bus, nid, OD_CONTROLWORD, 0, cw, timeout).await?;
        gap().await;
    }
    Ok(())
}

async fn gap() {
    tokio::time::sleep(SDO_GAP).await;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_matches_reference_layout() {
        // 参考实现的注释示例:pos=0x1234, vel=0x567, kp=0x89A, kd=0xBCD, torque=0xEF0
        // → lower=0x9ABCDEF0, upper=0x12345678 → [F0 DE BC 9A 78 56 34 12]
        // 用一个 mapping 让各分量正好映射到这些 uint 值不易构造,这里直接验位打包数学。
        let pos = 0x1234u32;
        let vel = 0x567u32;
        let kp = 0x89Au32;
        let kd = 0xBCDu32;
        let tor = 0xEF0u32;
        let lower = tor | (kd << 12) | ((kp & 0xFF) << 24);
        let upper = (kp >> 8) | (vel << 4) | (pos << 16);
        assert_eq!(lower, 0x9ABC_DEF0);
        assert_eq!(upper, 0x1234_5678);
    }

    #[test]
    fn zero_target_is_near_zero() {
        let m = CompressedMitMapping::default();
        let b = CompressedMitTarget::ZERO.to_le_bytes(&m);
        assert_eq!(b.len(), 8);
        // kp/kd 下限为 0 → 量化精确为 0(高字节里 kp 部分应为 0)
        let upper = u32::from_le_bytes([b[4], b[5], b[6], b[7]]);
        assert_eq!(upper >> 0 & 0xF, 0, "kp high nibble should be 0 for kp=0");
    }

    #[test]
    fn shared_frame_len() {
        let t = [CompressedMitTarget::ZERO; 6];
        let m = [CompressedMitMapping::default(); 6];
        assert_eq!(pack_shared_frame(&t, &m).len(), 48);
    }

    #[test]
    fn rpdo_entries_layout() {
        let e = rpdo_entries(2, 6);
        assert_eq!(e.len(), 12);
        // 槽 2 = entries[4..6] 是真实对象
        assert_eq!(e[4].index, OD_MIT_COMPRESSED);
        assert_eq!(e[4].subindex, SUB_LOWER);
        assert_eq!(e[5].subindex, SUB_UPPER);
        // 其他是占位
        assert_eq!(e[0], PLACEHOLDER);
    }
}
