//! Faithful `std::mt19937_64` (64-bit Mersenne Twister) reproduction.
//!
//! BambuStudio's `FillRectilinear::chain_monotonic_regions` (the ACO infill
//! ordering, FillRectilinear.cpp:2942/3397) default-constructs a
//! `std::mt19937_64 rng;` — i.e. the fixed default seed 5489 — and consumes it
//! deterministically. The Rust port previously used `rand::thread_rng()`
//! (entropy-seeded), which made the whole gcode nondeterministic run-to-run.
//!
//! This is the exact MT19937-64 generator (raw `next_u64()` sequence is the ISO
//! reference sequence, verified in the test below) plus a faithful port of the
//! libc++ `uniform_int_distribution<>(0, n-1)` draw used at the first-region
//! pick, so the infill order matches the native slicer bit-for-bit.

const NN: usize = 312;
const MM: usize = 156;
const MATRIX_A: u64 = 0xB502_6F5A_A966_19E9;
const UPPER_MASK: u64 = 0xFFFF_FFFF_8000_0000; // most significant 33 bits
const LOWER_MASK: u64 = 0x0000_0000_7FFF_FFFF; // least significant 31 bits
const INIT_MULT: u64 = 6_364_136_223_846_793_005;

/// The 64-bit Mersenne Twister, matching `std::mt19937_64`.
pub struct Mt19937_64 {
    mt: [u64; NN],
    mti: usize,
}

impl Mt19937_64 {
    /// Seed exactly like `std::mt19937_64(seed)` (default ctor uses seed 5489).
    pub fn new(seed: u64) -> Self {
        let mut mt = [0u64; NN];
        mt[0] = seed;
        for i in 1..NN {
            mt[i] = INIT_MULT
                .wrapping_mul(mt[i - 1] ^ (mt[i - 1] >> 62))
                .wrapping_add(i as u64);
        }
        Mt19937_64 { mt, mti: NN }
    }

    /// One 64-bit draw = `std::mt19937_64::operator()`. `max()` is `u64::MAX`.
    pub fn next_u64(&mut self) -> u64 {
        if self.mti >= NN {
            for i in 0..NN {
                let x = (self.mt[i] & UPPER_MASK) | (self.mt[(i + 1) % NN] & LOWER_MASK);
                let mag = if x & 1 != 0 { MATRIX_A } else { 0 };
                self.mt[i] = self.mt[(i + MM) % NN] ^ (x >> 1) ^ mag;
            }
            self.mti = 0;
        }
        let mut x = self.mt[self.mti];
        self.mti += 1;
        // Tempering.
        x ^= (x >> 29) & 0x5555_5555_5555_5555;
        x ^= (x << 17) & 0x71D6_7FFF_EDA6_0000;
        x ^= (x << 37) & 0xFFF7_EEE0_0000_0000;
        x ^= x >> 43;
        x
    }

    /// Faithful `std::uniform_int_distribution<>(0, n-1)(rng)` as implemented by
    /// libc++ (the native toolchain's stdlib): compute the number of bits `w`
    /// needed for the range `n`, then reject-sample the low `w` bits of the
    /// engine until the draw is `< n`. For `n == 1` no draw is consumed (matches
    /// libc++, which returns `a` immediately when the range is 1).
    pub fn uniform_int_below(&mut self, n: u64) -> u64 {
        debug_assert!(n >= 1);
        if n == 1 {
            return 0;
        }
        if n == 0 {
            // Full 64-bit range (unreachable for a queue length, kept faithful).
            return self.next_u64();
        }
        // __w = digits - clz(n) - 1, then round up if n is not a power of two.
        let wdt: u32 = 64;
        let mut w = wdt - n.leading_zeros() - 1;
        if (n & (u64::MAX >> (wdt - w))) != 0 {
            w += 1;
        }
        let mask = if w == 64 { u64::MAX } else { (1u64 << w) - 1 };
        loop {
            let u = self.next_u64() & mask;
            if u < n {
                return u;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reference_sequence_seed_5489() {
        // ISO C++ reference: the 10000th consecutive draw of the default-seeded
        // mt19937_64 (seed 5489) is 9981545732273789042.
        let mut rng = Mt19937_64::new(5489);
        let mut v = 0u64;
        for _ in 0..10_000 {
            v = rng.next_u64();
        }
        assert_eq!(v, 9_981_545_732_273_789_042);
    }

    #[test]
    fn uniform_int_below_ranges_and_is_bounded() {
        let mut rng = Mt19937_64::new(5489);
        for _ in 0..10_000 {
            let n = 1 + (rng.next_u64() % 37);
            let x = rng.uniform_int_below(n);
            assert!(x < n);
        }
    }
}
