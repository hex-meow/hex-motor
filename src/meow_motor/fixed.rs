//! 有符号 Q8.24 位置值。

use std::fmt;

const SCALE: f64 = (1u32 << 24) as f64;

/// Q8.24 构造错误。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Q8_24Error {
    NotFinite,
    OutOfRange { revolutions: f64 },
}

impl fmt::Display for Q8_24Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotFinite => write!(f, "Q8.24 value must be finite"),
            Self::OutOfRange { revolutions } => write!(
                f,
                "Q8.24 value {revolutions} rev is outside [-128, 127.9999999403953552]"
            ),
        }
    }
}

impl std::error::Error for Q8_24Error {}

/// Signed `i32` with 24 fractional bits.
///
/// The exact representable range is `[-128, 128 - 2^-24]` revolutions.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct SignedQ8_24(i32);

impl SignedQ8_24 {
    pub const FRACTIONAL_BITS: u32 = 24;
    pub const MIN: Self = Self(i32::MIN);
    pub const MAX: Self = Self(i32::MAX);
    pub const ZERO: Self = Self(0);
    pub const MIN_REVOLUTIONS: f64 = -128.0;
    pub const MAX_REVOLUTIONS: f64 = i32::MAX as f64 / SCALE;

    pub const fn from_raw(raw: i32) -> Self {
        Self(raw)
    }

    pub const fn raw(self) -> i32 {
        self.0
    }

    /// 从圈数编码，使用 round-to-nearest、half-away-from-zero 量化。
    ///
    /// 超范围、NaN 和 infinity 都返回错误；PP target 不做 clamp 或回绕。
    pub fn from_revolutions(revolutions: f64) -> Result<Self, Q8_24Error> {
        if !revolutions.is_finite() {
            return Err(Q8_24Error::NotFinite);
        }
        if !(Self::MIN_REVOLUTIONS..=Self::MAX_REVOLUTIONS).contains(&revolutions) {
            return Err(Q8_24Error::OutOfRange { revolutions });
        }
        let scaled = (revolutions * SCALE).round();
        if scaled < i32::MIN as f64 || scaled > i32::MAX as f64 {
            return Err(Q8_24Error::OutOfRange { revolutions });
        }
        Ok(Self(scaled as i32))
    }

    pub fn to_revolutions(self) -> f64 {
        self.0 as f64 / SCALE
    }

    /// 两次回绕位置采样的 modular delta。
    ///
    /// 调用方仍须保证采样间真实位移小于 128 圈；掉线后应重建累计基线。
    pub fn wrapping_delta(self, previous: Self) -> f64 {
        self.0.wrapping_sub(previous.0) as f64 / SCALE
    }

    pub const fn to_le_bytes(self) -> [u8; 4] {
        self.0.to_le_bytes()
    }

    pub const fn from_le_bytes(bytes: [u8; 4]) -> Self {
        Self(i32::from_le_bytes(bytes))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exact_vectors_and_endpoints() {
        assert_eq!(SignedQ8_24::from_revolutions(0.0).unwrap().raw(), 0);
        assert_eq!(
            SignedQ8_24::from_revolutions(1.0).unwrap().raw(),
            0x0100_0000
        );
        assert_eq!(
            SignedQ8_24::from_revolutions(-1.0).unwrap().raw(),
            -0x0100_0000
        );
        assert_eq!(
            SignedQ8_24::from_revolutions(-128.0).unwrap(),
            SignedQ8_24::MIN
        );
        assert_eq!(
            SignedQ8_24::from_revolutions(SignedQ8_24::MAX_REVOLUTIONS).unwrap(),
            SignedQ8_24::MAX
        );
    }

    #[test]
    fn rejects_non_finite_and_out_of_range_pp_targets() {
        assert_eq!(
            SignedQ8_24::from_revolutions(f64::NAN),
            Err(Q8_24Error::NotFinite)
        );
        assert!(SignedQ8_24::from_revolutions(128.0).is_err());
        assert!(SignedQ8_24::from_revolutions(-128.000_001).is_err());
    }

    #[test]
    fn wrapping_delta_crosses_positive_to_negative_boundary() {
        let previous = SignedQ8_24::from_raw(i32::MAX - (1 << 22));
        let current = SignedQ8_24::from_raw(i32::MIN + (1 << 22));
        let delta = current.wrapping_delta(previous);
        assert!((delta - (0.5 + 1.0 / SCALE)).abs() < 1e-12);
    }

    #[test]
    fn bytes_are_signed_little_endian() {
        assert_eq!(SignedQ8_24::MIN.to_le_bytes(), [0, 0, 0, 0x80]);
        assert_eq!(
            SignedQ8_24::from_le_bytes([0, 0, 0, 0x80]),
            SignedQ8_24::MIN
        );
    }
}
