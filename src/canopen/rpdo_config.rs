//! 纯函数：给定 RPDO 配置描述，生成需要的 SDO 写操作序列。
//!
//! 与 [`crate::canopen::tpdo_config`] 对称，只是面向**接收** PDO
//! (0x1400+idx 通信参数 / 0x1600+idx 映射)。本模块同样**不发任何帧**，只输出
//! "该写哪些 OD 项、写什么值"；runner 拿到序列后逐个调 [`crate::canopen::sdo`]。
//!
//! 典型用法：让一组从站监听同一个 COB-ID 的一帧 CAN-FD，每个从站只映射属于
//! 自己的那几个字节（其余用占位对象消费掉），即"主站一帧打包多电机目标"。

use crate::canopen::tpdo_config::{SdoWrite, TpdoEntry};
use crate::error::{Error, Result};

/// 一条 RPDO 的完整配置。
#[derive(Debug, Clone)]
pub struct RpdoRecipe {
    /// 0..=3 (RPDO1..=RPDO4)。
    pub rpdo_index: u8,
    /// COB-ID 的低 11 位。默认布局为 `0x200 + 0x100 * rpdo_index + node_id`，
    /// 但"多电机共享一帧"时会被故意设成所有电机相同的一个值。
    pub cob_id: u16,
    /// 映射项。复用 [`TpdoEntry`]（编码格式一致）。占位字节用一个"可映射但被
    /// 忽略"的对象（厂商对象或 CiA 哑元 0x0005/0x0006/0x0007）填充。
    pub entries: Vec<TpdoEntry>,
    /// 0x1400+idx sub 2。255 = 异步（收到即生效），推荐。
    pub transmission_type: u8,
}

impl RpdoRecipe {
    /// 验证 recipe 的合法性。
    pub fn validate(&self) -> Result<()> {
        if self.rpdo_index > 3 {
            return Err(Error::Internal(format!(
                "invalid rpdo_index {}, must be 0..=3",
                self.rpdo_index
            )));
        }
        if self.entries.is_empty() {
            return Err(Error::Internal("RpdoRecipe has no entries".into()));
        }
        if self.entries.len() > 64 {
            return Err(Error::Internal(format!(
                "too many entries: {}, max 64",
                self.entries.len()
            )));
        }
        let total_bits: u32 = self.entries.iter().map(|e| e.bit_len as u32).sum();
        if total_bits > 64 * 8 {
            return Err(Error::Internal(format!(
                "RpdoRecipe total {total_bits} bits > 512 (CAN-FD max payload)"
            )));
        }
        if self.cob_id > 0x7FF {
            return Err(Error::InvalidCobId {
                cob_id: self.cob_id,
                reason: "exceeds 11-bit range",
            });
        }
        Ok(())
    }

    /// 总字节数（向上取整到字节边界）。
    pub fn total_bytes(&self) -> usize {
        self.entries
            .iter()
            .map(|e| e.bit_len as usize)
            .sum::<usize>()
            .div_ceil(8)
    }
}

/// COB-ID 高位 VALID 标志（CiA-301 §7.4.3.1，RPDO）：bit 31 = `1` 失能、`0` 使能。
/// RPDO 不强制 NO_RTR（与 TPDO 不同），所以这里只翻 VALID 位。
const RPDO_COB_DISABLE_BIT: u32 = 0x8000_0000;

/// 生成"配置一条 RPDO"所需的全部 SDO 写操作（顺序敏感）。
///
/// 序列：
///   1. 失能 (0x1400+idx sub 1 = 0x80000000 | cob_id)
///   2. 设 transmission type (sub 2)
///   3. 清映射计数 (0x1600+idx sub 0 = 0)
///   4. 写所有映射项 (0x1600+idx sub 1..N)
///   5. 设回映射计数 (0x1600+idx sub 0 = N)
///   6. 使能 (0x1400+idx sub 1 = cob_id)
pub fn build_rpdo_config_writes(recipe: &RpdoRecipe) -> Result<Vec<SdoWrite>> {
    recipe.validate()?;
    let idx = recipe.rpdo_index as u16;
    let comm_index = 0x1400 + idx;
    let map_index = 0x1600 + idx;
    let cob_id = recipe.cob_id as u32;

    let mut writes = Vec::with_capacity(5 + recipe.entries.len());
    writes.push(SdoWrite::u32(comm_index, 1, RPDO_COB_DISABLE_BIT | cob_id));
    writes.push(SdoWrite::u8(comm_index, 2, recipe.transmission_type));
    writes.push(SdoWrite::u8(map_index, 0, 0));
    for (i, entry) in recipe.entries.iter().enumerate() {
        writes.push(SdoWrite::u32(map_index, (i + 1) as u8, entry.packed()));
    }
    writes.push(SdoWrite::u8(map_index, 0, recipe.entries.len() as u8));
    writes.push(SdoWrite::u32(comm_index, 1, cob_id)); // valid=0 → 使能
    Ok(writes)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vel_then_maxtorque() -> Vec<TpdoEntry> {
        vec![
            TpdoEntry {
                index: 0x60FF,
                subindex: 0,
                bit_len: 32,
            },
            TpdoEntry {
                index: 0x6072,
                subindex: 0,
                bit_len: 16,
            },
        ]
    }

    #[test]
    fn build_writes_full_sequence() {
        let recipe = RpdoRecipe {
            rpdo_index: 0,
            cob_id: 0x210,
            entries: vel_then_maxtorque(),
            transmission_type: 255,
        };
        let w = build_rpdo_config_writes(&recipe).unwrap();
        // 1 disable + 1 ttype + 1 clear-count + 2 entries + 1 set-count + 1 enable = 7
        assert_eq!(w.len(), 7);
        assert_eq!(w[0], SdoWrite::u32(0x1400, 1, 0x8000_0210)); // disable
        assert_eq!(w[1], SdoWrite::u8(0x1400, 2, 255)); // ttype
        assert_eq!(w[2], SdoWrite::u8(0x1600, 0, 0)); // clear count
        assert_eq!(w[3], SdoWrite::u32(0x1600, 1, 0x60FF_0020)); // target velocity
        assert_eq!(w[4], SdoWrite::u32(0x1600, 2, 0x6072_0010)); // max torque
        assert_eq!(w[5], SdoWrite::u8(0x1600, 0, 2)); // set count
        assert_eq!(w[6], SdoWrite::u32(0x1400, 1, 0x0000_0210)); // enable
    }

    #[test]
    fn total_bytes_round_up() {
        let recipe = RpdoRecipe {
            rpdo_index: 0,
            cob_id: 0x210,
            entries: vel_then_maxtorque(),
            transmission_type: 255,
        };
        assert_eq!(recipe.total_bytes(), 6);
    }

    #[test]
    fn validate_bad_rpdo_index() {
        let recipe = RpdoRecipe {
            rpdo_index: 4,
            cob_id: 0x210,
            entries: vel_then_maxtorque(),
            transmission_type: 255,
        };
        assert!(recipe.validate().is_err());
    }

    #[test]
    fn validate_bad_cob_id() {
        let recipe = RpdoRecipe {
            rpdo_index: 0,
            cob_id: 0x800,
            entries: vel_then_maxtorque(),
            transmission_type: 255,
        };
        assert!(matches!(recipe.validate(), Err(Error::InvalidCobId { .. })));
    }
}
