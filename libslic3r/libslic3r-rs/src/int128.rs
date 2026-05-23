//! 128-bit integer arithmetic for exact geometric predicates.
//!
//! C++ Reference:
//! - Int128.hpp
//!
//! This module provides 128-bit signed integer arithmetic operations used in exact
//! geometric predicates to avoid floating-point rounding errors. It uses Rust's
//! native i128 type when available, providing multiplication, addition, subtraction,
//! comparison, and specialized geometric functions like determinant evaluation.
//!
//! Originally from the Clipper library by Angus Johnson, extended by Vojtech Bubnik.

use std::cmp::Ordering;
use std::ops::{Add, AddAssign, Neg, Sub, SubAssign};

/// 128-bit signed integer for exact geometric computations
/// Int128.hpp:74-124
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Int128 {
    /// Internal 128-bit value
    /// Int128.hpp:78
    value: i128,
}

impl Int128 {
    /// Create a new Int128 from an i64
    /// Int128.hpp:80
    pub fn new(lo: i64) -> Self {
        Self { value: lo as i128 }
    }

    /// Create Int128 from raw i128 value
    pub fn from_i128(value: i128) -> Self {
        Self { value }
    }

    /// Get the low 64 bits as unsigned
    /// Int128.hpp:84
    pub fn lo(&self) -> u64 {
        self.value as u64
    }

    /// Get the high 64 bits as signed
    /// Int128.hpp:85
    pub fn hi(&self) -> i64 {
        (self.value >> 64) as i64
    }

    /// Get the sign: -1 for negative, 0 for zero, 1 for positive
    /// Int128.hpp:86
    pub fn sign(&self) -> i32 {
        match self.value.cmp(&0) {
            Ordering::Greater => 1,
            Ordering::Less => -1,
            Ordering::Equal => 0,
        }
    }

    /// Multiply two i64 values producing an Int128 result
    /// Int128.hpp:101
    pub fn multiply(lhs: i64, rhs: i64) -> Self {
        Self {
            value: (lhs as i128) * (rhs as i128),
        }
    }

    /// Evaluate the signum of a 2x2 determinant
    /// Int128.hpp:104-108
    ///
    /// Computes sign(a11*a22 - a12*a21) using exact 128-bit arithmetic.
    /// Returns -1, 0, or 1.
    pub fn sign_determinant_2x2(a11: i64, a12: i64, a21: i64, a22: i64) -> i32 {
        let det = (a11 as i128) * (a22 as i128) - (a12 as i128) * (a21 as i128);
        match det.cmp(&0) {
            Ordering::Greater => 1,
            Ordering::Less => -1,
            Ordering::Equal => 0,
        }
    }

    /// Compare two rational numbers p1/q1 and p2/q2
    /// Int128.hpp:111-116
    ///
    /// Compares p1/q1 vs p2/q2 using exact arithmetic by cross-multiplying:
    /// sign(p1*q2 - p2*q1) with sign correction for negative denominators.
    /// Returns -1 if p1/q1 < p2/q2, 0 if equal, 1 if p1/q1 > p2/q2.
    pub fn compare_rationals(p1: i64, q1: i64, p2: i64, q2: i64) -> i32 {
        let invert = if (q1 < 0) == (q2 < 0) { 1 } else { -1 };
        let det = (p1 as i128) * (q2 as i128) - (p2 as i128) * (q1 as i128);
        let sign = match det.cmp(&0) {
            Ordering::Greater => 1,
            Ordering::Less => -1,
            Ordering::Equal => 0,
        };
        sign * invert
    }

    /// Evaluate signum of a 2x2 determinant with numeric filtering
    /// Int128.hpp:316-330
    ///
    /// Uses a fast approximate calculation on the upper 31 bits first.
    /// If the approximate result is conclusive, returns immediately.
    /// Otherwise falls back to exact 128-bit arithmetic.
    pub fn sign_determinant_2x2_filtered(a11: i64, a12: i64, a21: i64, a22: i64) -> i32 {
        // Round to upper 31 bits (divide by 2^32 with rounding)
        let a11s = (a11 + (1 << 31)) >> 32;
        let a12s = (a12 + (1 << 31)) >> 32;
        let a21s = (a21 + (1 << 31)) >> 32;
        let a22s = (a22 + (1 << 31)) >> 32;

        // Approximate determinant (fits in 63 bits)
        let det = a11s * a22s - a12s * a21s;

        // Maximum possible error in the approximation
        let err = ((a11s.abs() + a12s.abs() + a21s.abs() + a22s.abs()) << 1) + 1;

        // If approximate result is conclusive, use it
        if det.abs() > err {
            if det > 0 {
                1
            } else {
                -1
            }
        } else {
            // Fall back to exact calculation
            Self::sign_determinant_2x2(a11, a12, a21, a22)
        }
    }

    /// Compare two rational numbers with numeric filtering
    /// Int128.hpp:333-349
    ///
    /// Uses a fast approximate calculation on the upper 31 bits first.
    /// If the approximate result is conclusive, returns immediately.
    /// Otherwise falls back to exact 128-bit arithmetic.
    pub fn compare_rationals_filtered(p1: i64, q1: i64, p2: i64, q2: i64) -> i32 {
        let invert = if (q1 < 0) == (q2 < 0) { 1 } else { -1 };

        // Round to upper 31 bits
        let q1s = (q1 + (1 << 31)) >> 32;
        let q2s = (q2 + (1 << 31)) >> 32;

        if q1s != 0 && q2s != 0 {
            let p1s = (p1 + (1 << 31)) >> 32;
            let p2s = (p2 + (1 << 31)) >> 32;

            // Approximate determinant
            let det = p1s * q2s - p2s * q1s;

            // Maximum possible error
            let err = ((p1s.abs() + q1s.abs() + p2s.abs() + q2s.abs()) << 1) + 1;

            if det.abs() > err {
                return if det > 0 { 1 } else { -1 } * invert;
            }
        }

        // Fall back to exact calculation
        Self::sign_determinant_2x2(p1, q1, p2, q2) * invert
    }

    /// Convert to f64 (may lose precision for very large values)
    /// Int128.hpp:99
    pub fn to_f64(&self) -> f64 {
        self.value as f64
    }

    /// Get the raw i128 value
    pub fn value(&self) -> i128 {
        self.value
    }
}

// Trait implementations

impl From<i64> for Int128 {
    fn from(val: i64) -> Self {
        Self::new(val)
    }
}

impl From<i128> for Int128 {
    fn from(val: i128) -> Self {
        Self::from_i128(val)
    }
}

impl From<Int128> for f64 {
    fn from(val: Int128) -> Self {
        val.to_f64()
    }
}

impl PartialOrd for Int128 {
    /// Int128.hpp:88-92
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Int128 {
    /// Int128.hpp:88-92
    fn cmp(&self, other: &Self) -> Ordering {
        self.value.cmp(&other.value)
    }
}

impl Add for Int128 {
    type Output = Self;

    /// Int128.hpp:94-95
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            value: self.value + rhs.value,
        }
    }
}

impl AddAssign for Int128 {
    /// Int128.hpp:93
    fn add_assign(&mut self, rhs: Self) {
        self.value += rhs.value;
    }
}

impl Sub for Int128 {
    type Output = Self;

    /// Int128.hpp:97
    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            value: self.value - rhs.value,
        }
    }
}

impl SubAssign for Int128 {
    /// Int128.hpp:96
    fn sub_assign(&mut self, rhs: Self) {
        self.value -= rhs.value;
    }
}

impl Neg for Int128 {
    type Output = Self;

    /// Int128.hpp:98
    fn neg(self) -> Self::Output {
        Self { value: -self.value }
    }
}

impl Default for Int128 {
    fn default() -> Self {
        Self::new(0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_int128_creation() {
        let a = Int128::new(42);
        assert_eq!(a.sign(), 1);

        let b = Int128::new(-42);
        assert_eq!(b.sign(), -1);

        let c = Int128::new(0);
        assert_eq!(c.sign(), 0);
    }

    #[test]
    fn test_int128_lo_hi() {
        let a = Int128::new(0x1234567890ABCDEF_i64);
        assert_eq!(a.lo(), 0x1234567890ABCDEF_u64);

        // High bits should be sign-extended for positive values
        let b = Int128::new(i64::MAX);
        assert_eq!(b.hi(), 0);

        // High bits should be sign-extended for negative values
        let c = Int128::new(-1);
        assert_eq!(c.hi(), -1);
    }

    #[test]
    fn test_int128_arithmetic() {
        let a = Int128::new(100);
        let b = Int128::new(50);

        let sum = a + b;
        assert_eq!(sum.sign(), 1);
        assert_eq!((sum - Int128::new(150)).sign(), 0);

        let diff = a - b;
        assert_eq!((diff - Int128::new(50)).sign(), 0);

        let neg = -a;
        assert_eq!(neg.sign(), -1);
    }

    #[test]
    fn test_int128_comparison() {
        let a = Int128::new(100);
        let b = Int128::new(50);
        let c = Int128::new(100);

        assert!(a > b);
        assert!(b < a);
        assert_eq!(a, c);
        assert!(a >= c);
        assert!(a <= c);
    }

    #[test]
    fn test_int128_multiply() {
        let a = 1000_i64;
        let b = 2000_i64;
        let result = Int128::multiply(a, b);
        assert_eq!(result.value(), 2_000_000);

        // Test with large values that would overflow i64
        let large1 = 9_223_372_036_854_775_807_i64; // i64::MAX
        let large2 = 2_i64;
        let result = Int128::multiply(large1, large2);
        assert!(result.value() > i64::MAX as i128);
    }

    #[test]
    fn test_sign_determinant_2x2() {
        // Positive determinant: | 2  1 | = 2*4 - 1*3 = 8 - 3 = 5
        //                       | 3  4 |
        assert_eq!(Int128::sign_determinant_2x2(2, 1, 3, 4), 1);

        // Negative determinant: | 1  2 | = 1*4 - 2*3 = 4 - 6 = -2
        //                       | 3  4 |
        assert_eq!(Int128::sign_determinant_2x2(1, 2, 3, 4), -1);

        // Zero determinant: | 2  4 | = 2*2 - 4*1 = 4 - 4 = 0
        //                   | 1  2 |
        assert_eq!(Int128::sign_determinant_2x2(2, 4, 1, 2), 0);

        // Test with large values
        let large = 1_000_000_000_i64;
        assert_eq!(Int128::sign_determinant_2x2(large, 0, 0, large), 1);
    }

    #[test]
    fn test_compare_rationals() {
        // 3/2 > 4/3
        assert_eq!(Int128::compare_rationals(3, 2, 4, 3), 1);

        // 2/3 < 3/4
        assert_eq!(Int128::compare_rationals(2, 3, 3, 4), -1);

        // 2/4 == 3/6
        assert_eq!(Int128::compare_rationals(2, 4, 3, 6), 0);

        // Handle negative denominators: 3/-2 < 4/3
        assert_eq!(Int128::compare_rationals(3, -2, 4, 3), -1);

        // Both denominators negative: -3/-2 > 4/3
        assert_eq!(Int128::compare_rationals(3, -2, 4, -3), -1);
    }

    #[test]
    fn test_sign_determinant_2x2_filtered() {
        // Small values - should use fast path
        assert_eq!(Int128::sign_determinant_2x2_filtered(2, 1, 3, 4), 1);
        assert_eq!(Int128::sign_determinant_2x2_filtered(1, 2, 3, 4), -1);
        assert_eq!(Int128::sign_determinant_2x2_filtered(2, 4, 1, 2), 0);

        // Large values - may fall back to exact calculation
        let large = 1_000_000_000_000_i64;
        let result = Int128::sign_determinant_2x2_filtered(large, 1, 1, large);
        assert_eq!(result, 1);
    }

    #[test]
    fn test_compare_rationals_filtered() {
        // Small values - should use fast path
        assert_eq!(Int128::compare_rationals_filtered(3, 2, 4, 3), 1);
        assert_eq!(Int128::compare_rationals_filtered(2, 3, 3, 4), -1);
        assert_eq!(Int128::compare_rationals_filtered(2, 4, 3, 6), 0);

        // Large values - may fall back to exact calculation
        let large = 1_000_000_000_000_i64;
        let result = Int128::compare_rationals_filtered(large, 1, large + 1, 1);
        assert_eq!(result, -1);
    }

    #[test]
    fn test_overflow_safety() {
        // Test that multiplication doesn't overflow with large i64 values
        let max_half = i64::MAX / 2;
        let result = Int128::multiply(max_half, 3);
        assert!(result.value() > i64::MAX as i128);
        assert_eq!(result.sign(), 1);

        // Test negative overflow
        let result = Int128::multiply(-max_half, 3);
        assert!(result.value() < i64::MIN as i128);
        assert_eq!(result.sign(), -1);
    }

    #[test]
    fn test_to_f64() {
        let a = Int128::new(1000);
        assert!((a.to_f64() - 1000.0).abs() < 1e-6);

        let b = Int128::new(-2000);
        assert!((b.to_f64() - (-2000.0)).abs() < 1e-6);
    }

    #[test]
    fn test_add_assign_sub_assign() {
        let mut a = Int128::new(100);
        a += Int128::new(50);
        assert_eq!(a, Int128::new(150));

        a -= Int128::new(25);
        assert_eq!(a, Int128::new(125));
    }
}
