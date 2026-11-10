//! Faithful port of the vendored AGG header `src/agg/agg_color_gray.h`
//! (Anti-Grain Geometry 2.4, as bundled with BambuStudio) — the `gray8`
//! instantiation (`gray8T<linear>`) used by `pixfmt_gray8`.
//!
//! C++ Reference:
//! - agg/agg_color_gray.h
//!
//! `gray8T` is a template over a colorspace tag; only the integer math (which
//! is colorspace-independent) is exercised by the rasterizer, so the single
//! concrete `Gray8` type is ported. RGBA conversions (luminance etc.) are not
//! used by the SLA raster and are not ported.

use super::basics::CoverType;

// agg_color_gray.h:37-39  template<class Colorspace> struct gray8T
// agg_color_gray.h:40-42
// typedef int8u  value_type;
// typedef int32u calc_type;
// typedef int32  long_type;
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Gray8 {
    // agg_color_gray.h:52-53  value_type v;  value_type a;
    pub v: u8,
    pub a: u8,
}

impl Gray8 {
    // agg_color_gray.h:43-49  enum base_scale_e
    pub const BASE_SHIFT: u32 = 8; // base_shift = 8
    pub const BASE_SCALE: u32 = 1 << Self::BASE_SHIFT; // base_scale
    pub const BASE_MASK: u32 = Self::BASE_SCALE - 1; // base_mask
    pub const BASE_MSB: u32 = 1 << (Self::BASE_SHIFT - 1); // base_MSB

    // agg_color_gray.h:105  gray8T() {}
    // (uninitialized in C++; Rust `Default` zero-initializes)

    // agg_color_gray.h:108-109
    // explicit gray8T(unsigned v_, unsigned a_ = base_mask) :
    //     v(int8u(v_)), a(int8u(a_)) {}
    pub fn new(v_: u32) -> Self {
        Self {
            v: v_ as u8,
            a: Self::BASE_MASK as u8,
        }
    }

    pub fn new_with_alpha(v_: u32, a_: u32) -> Self {
        Self {
            v: v_ as u8,
            a: a_ as u8,
        }
    }

    // agg_color_gray.h:240-243
    // AGG_INLINE bool is_transparent() const
    // {
    //     return a == 0;
    // }
    #[inline]
    pub fn is_transparent(&self) -> bool {
        self.a == 0
    }

    // agg_color_gray.h:246-249
    // AGG_INLINE bool is_opaque() const
    // {
    //     return a == base_mask;
    // }
    #[inline]
    pub fn is_opaque(&self) -> bool {
        self.a as u32 == Self::BASE_MASK
    }

    // agg_color_gray.h:252-257
    // // Fixed-point multiply, exact over int8u.
    // static AGG_INLINE value_type multiply(value_type a, value_type b)
    // {
    //     calc_type t = a * b + base_MSB;
    //     return value_type(((t >> base_shift) + t) >> base_shift);
    // }
    #[inline]
    pub fn multiply(a: u8, b: u8) -> u8 {
        let t: u32 = a as u32 * b as u32 + Self::BASE_MSB;
        (((t >> Self::BASE_SHIFT) + t) >> Self::BASE_SHIFT) as u8
    }

    // agg_color_gray.h:260-270
    // static AGG_INLINE value_type demultiply(value_type a, value_type b)
    #[inline]
    pub fn demultiply(a: u8, b: u8) -> u8 {
        if a as u32 * b as u32 == 0 {
            0
        } else if a >= b {
            Self::BASE_MASK as u8
        } else {
            ((a as u32 * Self::BASE_MASK + (b as u32 >> 1)) / b as u32) as u8
        }
    }

    // agg_color_gray.h:288-293
    // // Fixed-point multiply, exact over int8u.
    // // Specifically for multiplying a color component by a cover.
    // static AGG_INLINE value_type mult_cover(value_type a, value_type b)
    // {
    //     return multiply(a, b);
    // }
    #[inline]
    pub fn mult_cover(a: u8, b: u8) -> u8 {
        Self::multiply(a, b)
    }

    // agg_color_gray.h:296-299
    // static AGG_INLINE cover_type scale_cover(cover_type a, value_type b)
    // {
    //     return multiply(b, a);
    // }
    #[inline]
    pub fn scale_cover(a: CoverType, b: u8) -> CoverType {
        Self::multiply(b, a)
    }

    // agg_color_gray.h:301-306
    // // Interpolate p to q by a, assuming q is premultiplied by a.
    // static AGG_INLINE value_type prelerp(value_type p, value_type q, value_type a)
    // {
    //     return p + q - multiply(p, a);
    // }
    // (the C++ arithmetic is done in `int` then truncated to value_type)
    #[inline]
    pub fn prelerp(p: u8, q: u8, a: u8) -> u8 {
        (p as i32 + q as i32 - Self::multiply(p, a) as i32) as u8
    }

    // agg_color_gray.h:309-314
    // // Interpolate p to q by a.
    // static AGG_INLINE value_type lerp(value_type p, value_type q, value_type a)
    // {
    //     int t = (q - p) * a + base_MSB - (p > q);
    //     return value_type(p + (((t >> base_shift) + t) >> base_shift));
    // }
    #[inline]
    pub fn lerp(p: u8, q: u8, a: u8) -> u8 {
        let t: i32 =
            (q as i32 - p as i32) * a as i32 + Self::BASE_MSB as i32 - (p > q) as i32;
        (p as i32 + (((t >> Self::BASE_SHIFT) + t) >> Self::BASE_SHIFT)) as u8
    }
}
