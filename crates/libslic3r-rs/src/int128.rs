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

/// Int128 class (enables safe math on signed 64bit integers)
/// eg Int128 val1((int64_t)9223372036854775807); //ie 2^63 -1
///    Int128 val2((int64_t)9223372036854775807);
///    Int128 val3 = val1 * val2;
/// Int128.hpp:72
///
/// The Rust port uses the native `i128` type, mirroring the C++
/// `HAS_INTRINSIC_128_TYPE` branch (Int128.hpp:75-121).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Int128 {
    /// `__int128 value;`
    /// Int128.hpp:80
    value: i128,
}

impl Int128 {
    /// `Int128(int64_t lo = 0) : value(lo) {}`
    /// Int128.hpp:82
    pub fn new(lo: i64) -> Self {
        Self { value: lo as i128 }
    }

    /// Create Int128 from raw i128 value
    /// (mirrors the implicit `Int128(__int128)` used at Int128.hpp:106)
    pub fn from_i128(value: i128) -> Self {
        Self { value }
    }

    /// `uint64_t lo() const { return uint64_t(value); }`
    /// Int128.hpp:87
    pub fn lo(&self) -> u64 {
        self.value as u64
    }

    /// `int64_t hi() const { return int64_t(value >> 64); }`
    /// Int128.hpp:88
    pub fn hi(&self) -> i64 {
        (self.value >> 64) as i64
    }

    /// `int sign() const { return (value > 0) - (value < 0); }`
    /// Int128.hpp:89
    pub fn sign(&self) -> i32 {
        match self.value.cmp(&0) {
            Ordering::Greater => 1,
            Ordering::Less => -1,
            Ordering::Equal => 0,
        }
    }

    /// `static inline Int128 multiply(int64_t lhs, int64_t rhs) { return Int128(__int128(lhs) * __int128(rhs)); }`
    /// Int128.hpp:106
    pub fn multiply(lhs: i64, rhs: i64) -> Self {
        Self {
            value: (lhs as i128) * (rhs as i128),
        }
    }

    /// Evaluate signum of a 2x2 determinant.
    /// Int128.hpp:108-113
    ///
    /// `__int128 det = __int128(a11) * __int128(a22) - __int128(a12) * __int128(a21);`
    /// `return (det > 0) - (det < 0);`
    pub fn sign_determinant_2x2(a11: i64, a12: i64, a21: i64, a22: i64) -> i32 {
        let det = (a11 as i128) * (a22 as i128) - (a12 as i128) * (a21 as i128);
        match det.cmp(&0) {
            Ordering::Greater => 1,
            Ordering::Less => -1,
            Ordering::Equal => 0,
        }
    }

    /// Compare two rational numbers.
    /// Int128.hpp:115-121
    ///
    /// `int invert = ((q1 < 0) == (q2 < 0)) ? 1 : -1;`
    /// `__int128 det = __int128(p1) * __int128(q2) - __int128(p2) * __int128(q1);`
    /// `return ((det > 0) - (det < 0)) * invert;`
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

    /// Evaluate signum of a 2x2 determinant, use a numeric filter to avoid 128 bit multiply if possible.
    /// Int128.hpp:265-281
    pub fn sign_determinant_2x2_filtered(a11: i64, a12: i64, a21: i64, a22: i64) -> i32 {
        // First try to calculate the determinant over the upper 31 bits.
        // Round p1, p2, q1, q2 to 31 bits.
        // Int128.hpp:269-272
        //
        // C++ FAITHFULNESS: in C++ `(1 << 31)` is an `int` (32-bit) literal that
        // overflows the signed `int` range, yielding `-2147483648` (= `i32::MIN`),
        // which is then promoted to `int64_t` before being added. So the rounding
        // bias is actually `-2^31`, not `+2^31` (despite the "round" comment). We
        // reproduce the exact C++ value here. (Verified: for a11=1e9 C++ yields -1.)
        const ROUND: i64 = i32::MIN as i64; // C++ `(1 << 31)` == int(-2147483648)
        let a11s = (a11 + ROUND) >> 32;
        let a12s = (a12 + ROUND) >> 32;
        let a21s = (a21 + ROUND) >> 32;
        let a22s = (a22 + ROUND) >> 32;
        // Result fits 63 bits, it is an approximate of the determinant divided by 2^64.
        // Int128.hpp:274
        let det = a11s * a22s - a12s * a21s;
        // Maximum absolute of the remainder of the exact determinant, divided by 2^64.
        // Int128.hpp:276
        let err = ((a11s.abs() + a12s.abs() + a21s.abs() + a22s.abs()) << 1) + 1;
        // assert (Int128.hpp:277) elided: the exact path validates this in debug C++ only.
        // Int128.hpp:278-280
        if det.abs() > err {
            if det > 0 {
                1
            } else {
                -1
            }
        } else {
            Self::sign_determinant_2x2(a11, a12, a21, a22)
        }
    }

    /// Compare two rational numbers, use a numeric filter to avoid 128 bit multiply if possible.
    /// Int128.hpp:284-303
    pub fn compare_rationals_filtered(p1: i64, q1: i64, p2: i64, q2: i64) -> i32 {
        // First try to calculate the determinant over the upper 31 bits.
        // Round p1, p2, q1, q2 to 31 bits.
        // Int128.hpp:288
        let invert = if (q1 < 0) == (q2 < 0) { 1 } else { -1 };
        // Int128.hpp:289-290
        //
        // C++ FAITHFULNESS: `(1 << 31)` overflows `int` to `-2147483648` (= i32::MIN),
        // promoted to int64_t, so the rounding bias is `-2^31`. See note in
        // `sign_determinant_2x2_filtered`. Reproduced exactly here.
        const ROUND: i64 = i32::MIN as i64; // C++ `(1 << 31)` == int(-2147483648)
        let q1s = (q1 + ROUND) >> 32;
        let q2s = (q2 + ROUND) >> 32;
        if q1s != 0 && q2s != 0 {
            // Int128.hpp:292-293
            let p1s = (p1 + ROUND) >> 32;
            let p2s = (p2 + ROUND) >> 32;
            // Result fits 63 bits, it is an approximate of the determinant divided by 2^64.
            // Int128.hpp:295
            let det = p1s * q2s - p2s * q1s;
            // Maximum absolute of the remainder of the exact determinant, divided by 2^64.
            // Int128.hpp:297
            let err = ((p1s.abs() + q1s.abs() + p2s.abs() + q2s.abs()) << 1) + 1;
            // assert (Int128.hpp:298) elided.
            // Int128.hpp:299-300
            if det.abs() > err {
                return if det > 0 { 1 } else { -1 } * invert;
            }
        }
        // Int128.hpp:302
        Self::sign_determinant_2x2(p1, q1, p2, q2) * invert
    }

    /// `operator double() const { return double(value); }`
    /// Int128.hpp:104
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
    /// `Int128(int64_t lo = 0)` / `Int128& operator=(const int64_t &rhs)`
    /// Int128.hpp:82, 85
    fn from(val: i64) -> Self {
        Self::new(val)
    }
}

impl From<i128> for Int128 {
    /// Implicit `Int128(__int128)` used at Int128.hpp:106
    fn from(val: i128) -> Self {
        Self::from_i128(val)
    }
}

impl From<Int128> for f64 {
    /// `operator double() const`
    /// Int128.hpp:104
    fn from(val: Int128) -> Self {
        val.to_f64()
    }
}

impl PartialOrd for Int128 {
    /// `operator>`/`operator<`/`operator>=`/`operator<=`
    /// Int128.hpp:93-96
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Int128 {
    /// `operator>`/`operator<`/`operator>=`/`operator<=`
    /// Int128.hpp:93-96
    fn cmp(&self, other: &Self) -> Ordering {
        self.value.cmp(&other.value)
    }
}

impl Add for Int128 {
    type Output = Self;

    /// `Int128 operator+ (const Int128 &rhs) const { return Int128(value + rhs.value); }`
    /// Int128.hpp:99
    fn add(self, rhs: Self) -> Self::Output {
        Self {
            value: self.value + rhs.value,
        }
    }
}

impl AddAssign for Int128 {
    /// `Int128& operator+=(const Int128 &rhs) { value += rhs.value; return *this; }`
    /// Int128.hpp:98
    fn add_assign(&mut self, rhs: Self) {
        self.value += rhs.value;
    }
}

impl Sub for Int128 {
    type Output = Self;

    /// `Int128 operator -(const Int128 &rhs) const { return Int128(value - rhs.value); }`
    /// Int128.hpp:101
    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            value: self.value - rhs.value,
        }
    }
}

impl SubAssign for Int128 {
    /// `Int128& operator-=(const Int128 &rhs) { value -= rhs.value; return *this; }`
    /// Int128.hpp:100
    fn sub_assign(&mut self, rhs: Self) {
        self.value -= rhs.value;
    }
}

impl Neg for Int128 {
    type Output = Self;

    /// `Int128 operator -() const { return Int128(- value); }`
    /// Int128.hpp:102
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
