//! 纯函数：给定 TPDO 配置描述，生成需要的 SDO 写操作序列。
//!
//! 本模块**不发任何帧**；只输出"该写哪些 OD 项、写什么值"。runner 拿到
//! 序列后逐个调 [`crate::canopen::sdo`]。
//!
//! 注意：本模块面向"在 Pre-Operational 状态下完整重配 TPDO"。即便不进
//! Pre-Operational，序列中也会先用 valid bit 失能再修改，符合 CiA-301。

use crate::error::{Error, Result};

/// 单条映射到 PDO 的 OD entry。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TpdoEntry {
    pub index: u16,
    pub subindex: u8,
    pub bit_len: u8,
}

impl TpdoEntry {
    /// 一个映射条目的 32-bit 编码：`(index << 16) | (sub << 8) | bit_len`。
    pub fn packed(&self) -> u32 {
        ((self.index as u32) << 16) | ((self.subindex as u32) << 8) | self.bit_len as u32
    }
}

/// TPDO 通信参数 (0x1800+idx 的 sub 2 / 3 / 5)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TpdoCommParams {
    /// sub 2。255 = 异步事件触发（推荐）。
    pub transmission_type: u8,
    /// sub 3。单位 100us，最快发送间隔。
    pub inhibit_time_x100us: u16,
    /// sub 5。单位 ms，定时触发周期。0 = 关闭定时。
    pub event_timer_ms: u16,
}

/// 一条 TPDO 的完整配置。
#[derive(Debug, Clone)]
pub struct TpdoRecipe {
    /// 0..=3 (TPDO1..=TPDO4)。
    pub tpdo_index: u8,
    /// COB-ID 的低 11 位。一般为 `0x180 + 0x100 * tpdo_index + node_id`。
    pub cob_id: u16,
    pub entries: Vec<TpdoEntry>,
    pub comm: TpdoCommParams,
}

impl TpdoRecipe {
    /// 验证 recipe 的合法性。
    pub fn validate(&self) -> Result<()> {
        if self.tpdo_index > 3 {
            return Err(Error::Internal(format!(
                "invalid tpdo_index {}, must be 0..=3",
                self.tpdo_index
            )));
        }
        if self.entries.is_empty() {
            return Err(Error::Internal("TpdoRecipe has no entries".into()));
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
                "TpdoRecipe total {total_bits} bits > 512 (CAN-FD max payload)"
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

/// 描述一条 SDO download 操作（纯数据；runner 翻译成 [`crate::canopen::sdo::download`]）。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SdoWrite {
    pub index: u16,
    pub subindex: u8,
    pub data: Vec<u8>,
}

impl SdoWrite {
    pub fn u8(index: u16, sub: u8, v: u8) -> Self {
        Self {
            index,
            subindex: sub,
            data: vec![v],
        }
    }
    pub fn u16(index: u16, sub: u8, v: u16) -> Self {
        Self {
            index,
            subindex: sub,
            data: v.to_le_bytes().to_vec(),
        }
    }
    pub fn u32(index: u16, sub: u8, v: u32) -> Self {
        Self {
            index,
            subindex: sub,
            data: v.to_le_bytes().to_vec(),
        }
    }
    pub fn i8(index: u16, sub: u8, v: i8) -> Self {
        Self {
            index,
            subindex: sub,
            data: vec![v as u8],
        }
    }
    pub fn i16(index: u16, sub: u8, v: i16) -> Self {
        Self {
            index,
            subindex: sub,
            data: v.to_le_bytes().to_vec(),
        }
    }
    pub fn i32(index: u16, sub: u8, v: i32) -> Self {
        Self {
            index,
            subindex: sub,
            data: v.to_le_bytes().to_vec(),
        }
    }
    pub fn f32(index: u16, sub: u8, v: f32) -> Self {
        Self {
            index,
            subindex: sub,
            data: v.to_le_bytes().to_vec(),
        }
    }
}

/// COB-ID 字段的高位标志（CiA-301 §7.4.3.2）：
/// - bit 31 (`VALID`)：`1` = invalid (PDO 关闭)，`0` = valid (PDO 工作)
/// - bit 30 (`NO_RTR`)：`1` = remote frame 不允许，`0` = 允许
///
/// HexMeow CiA402 电机 **强制要求 TPDO 不允许 RTR**，否则写 0x1800:01
/// 会得到 SDO abort `0x06090030` (Value range exceeded)。所以 disable/enable
/// 两种写值都带上 bit 30。
const TPDO_COB_DISABLE_BITS: u32 = 0xC000_0000; // valid=1, no_rtr=1
const TPDO_COB_ENABLE_BITS: u32 = 0x4000_0000; // valid=0, no_rtr=1

/// 生成"配置一条 TPDO"所需的全部 SDO 写操作（顺序敏感）。
///
/// 序列：
///   1. 失能 (0x1800+idx sub 1 = 0xC0000000 | cob_id)
///   2. 清映射计数 (0x1A00+idx sub 0 = 0)
///   3. 写所有映射项 (0x1A00+idx sub 1..N)
///   4. 设回映射计数 (0x1A00+idx sub 0 = N)
///   5. 设 transmission type (sub 2)
///   6. 设 inhibit time (sub 3)
///   7. 设 event timer (sub 5)
///   8. 使能 (0x1800+idx sub 1 = 0x40000000 | cob_id)
pub fn build_tpdo_config_writes(recipe: &TpdoRecipe) -> Result<Vec<SdoWrite>> {
    recipe.validate()?;
    let idx = recipe.tpdo_index as u16;
    let comm_index = 0x1800 + idx;
    let map_index = 0x1A00 + idx;
    let cob_id = recipe.cob_id as u32;

    let mut writes = Vec::with_capacity(7 + recipe.entries.len());
    writes.push(SdoWrite::u32(comm_index, 1, TPDO_COB_DISABLE_BITS | cob_id));
    writes.push(SdoWrite::u8(map_index, 0, 0));
    for (i, entry) in recipe.entries.iter().enumerate() {
        writes.push(SdoWrite::u32(map_index, (i + 1) as u8, entry.packed()));
    }
    writes.push(SdoWrite::u8(map_index, 0, recipe.entries.len() as u8));
    writes.push(SdoWrite::u8(comm_index, 2, recipe.comm.transmission_type));
    writes.push(SdoWrite::u16(
        comm_index,
        3,
        recipe.comm.inhibit_time_x100us,
    ));
    writes.push(SdoWrite::u16(comm_index, 5, recipe.comm.event_timer_ms));
    writes.push(SdoWrite::u32(comm_index, 1, TPDO_COB_ENABLE_BITS | cob_id));

    Ok(writes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packed_status_word() {
        let e = TpdoEntry {
            index: 0x6041,
            subindex: 0,
            bit_len: 16,
        };
        assert_eq!(e.packed(), 0x6041_0010);
    }

    #[test]
    fn packed_actual_position() {
        let e = TpdoEntry {
            index: 0x6064,
            subindex: 0,
            bit_len: 32,
        };
        assert_eq!(e.packed(), 0x6064_0020);
    }

    #[test]
    fn packed_subindex() {
        // 0x2204:02 i16
        let e = TpdoEntry {
            index: 0x2204,
            subindex: 0x02,
            bit_len: 16,
        };
        assert_eq!(e.packed(), 0x2204_0210);
    }

    #[test]
    fn total_bytes_round_up() {
        let recipe = TpdoRecipe {
            tpdo_index: 0,
            cob_id: 0x190,
            entries: vec![
                TpdoEntry {
                    index: 0x6041,
                    subindex: 0,
                    bit_len: 16,
                },
                TpdoEntry {
                    index: 0x6061,
                    subindex: 0,
                    bit_len: 8,
                },
            ],
            comm: TpdoCommParams {
                transmission_type: 255,
                inhibit_time_x100us: 5,
                event_timer_ms: 20,
            },
        };
        assert_eq!(recipe.total_bytes(), 3);
    }

    #[test]
    fn build_writes_full_sequence() {
        let recipe = TpdoRecipe {
            tpdo_index: 0,
            cob_id: 0x190,
            entries: vec![
                TpdoEntry {
                    index: 0x6041,
                    subindex: 0,
                    bit_len: 16,
                },
                TpdoEntry {
                    index: 0x6064,
                    subindex: 0,
                    bit_len: 32,
                },
            ],
            comm: TpdoCommParams {
                transmission_type: 255,
                inhibit_time_x100us: 5,
                event_timer_ms: 20,
            },
        };
        let w = build_tpdo_config_writes(&recipe).unwrap();
        // 1 disable + 1 clear-count + 2 entries + 1 set-count + 3 comm-params + 1 enable = 9
        assert_eq!(w.len(), 9);

        // 顺序 + 高位标志（VALID + NO_RTR）断言
        assert_eq!(w[0], SdoWrite::u32(0x1800, 1, 0xC000_0190)); // disable (valid=1, no_rtr=1)
        assert_eq!(w[1], SdoWrite::u8(0x1A00, 0, 0)); // clear count
        assert_eq!(w[2], SdoWrite::u32(0x1A00, 1, 0x6041_0010)); // status word
        assert_eq!(w[3], SdoWrite::u32(0x1A00, 2, 0x6064_0020)); // actual pos
        assert_eq!(w[4], SdoWrite::u8(0x1A00, 0, 2)); // set count
        assert_eq!(w[5], SdoWrite::u8(0x1800, 2, 255)); // ttype
        assert_eq!(w[6], SdoWrite::u16(0x1800, 3, 5)); // inhibit
        assert_eq!(w[7], SdoWrite::u16(0x1800, 5, 20)); // event timer
        assert_eq!(w[8], SdoWrite::u32(0x1800, 1, 0x4000_0190)); // enable (valid=0, no_rtr=1)
    }

    #[test]
    fn build_writes_tpdo2() {
        let recipe = TpdoRecipe {
            tpdo_index: 1,
            cob_id: 0x290,
            entries: vec![TpdoEntry {
                index: 0x6041,
                subindex: 0,
                bit_len: 16,
            }],
            comm: TpdoCommParams {
                transmission_type: 255,
                inhibit_time_x100us: 190,
                event_timer_ms: 20,
            },
        };
        let w = build_tpdo_config_writes(&recipe).unwrap();
        // 1 + 1 + 1 + 1 + 3 + 1 = 8
        assert_eq!(w.len(), 8);
        assert_eq!(w[0].index, 0x1801);
        assert_eq!(w[1].index, 0x1A01);
    }

    #[test]
    fn validate_empty_entries() {
        let recipe = TpdoRecipe {
            tpdo_index: 0,
            cob_id: 0x190,
            entries: vec![],
            comm: TpdoCommParams {
                transmission_type: 255,
                inhibit_time_x100us: 0,
                event_timer_ms: 0,
            },
        };
        assert!(recipe.validate().is_err());
    }

    #[test]
    fn validate_bad_tpdo_index() {
        let recipe = TpdoRecipe {
            tpdo_index: 4,
            cob_id: 0x190,
            entries: vec![TpdoEntry {
                index: 0x6041,
                subindex: 0,
                bit_len: 16,
            }],
            comm: TpdoCommParams {
                transmission_type: 255,
                inhibit_time_x100us: 0,
                event_timer_ms: 0,
            },
        };
        assert!(recipe.validate().is_err());
    }

    #[test]
    fn validate_oversize_payload() {
        // 9 个 64-bit entry = 72 bytes，超过 CAN-FD 64-byte
        let recipe = TpdoRecipe {
            tpdo_index: 0,
            cob_id: 0x190,
            entries: (0..9)
                .map(|_| TpdoEntry {
                    index: 0x6041,
                    subindex: 0,
                    bit_len: 64,
                })
                .collect(),
            comm: TpdoCommParams {
                transmission_type: 255,
                inhibit_time_x100us: 0,
                event_timer_ms: 0,
            },
        };
        assert!(recipe.validate().is_err());
    }

    #[test]
    fn validate_bad_cob_id() {
        let recipe = TpdoRecipe {
            tpdo_index: 0,
            cob_id: 0x800,
            entries: vec![TpdoEntry {
                index: 0x6041,
                subindex: 0,
                bit_len: 16,
            }],
            comm: TpdoCommParams {
                transmission_type: 255,
                inhibit_time_x100us: 0,
                event_timer_ms: 0,
            },
        };
        assert!(matches!(recipe.validate(), Err(Error::InvalidCobId { .. })));
    }
}
