//! Faithful port of the vendored AGG header `src/agg/agg_clip_liang_barsky.h`
//! (Anti-Grain Geometry 2.4, as bundled with BambuStudio) — the clipping-flag
//! helpers used by `rasterizer_sl_clip`.
//!
//! C++ Reference:
//! - agg/agg_clip_liang_barsky.h
//!
//! Only the Cyrus-Beck clipping-flag functions are needed by the scanline
//! rasterizer clipper; the full Liang-Barsky polygon clipper
//! (agg_clip_liang_barsky.h:82-229) has no users in the SLA raster path and is
//! not ported.

use super::basics::RectBase;

// agg_clip_liang_barsky.h:28-36  enum clipping_flags_e
pub const CLIPPING_FLAGS_X1_CLIPPED: u32 = 4;
pub const CLIPPING_FLAGS_X2_CLIPPED: u32 = 1;
pub const CLIPPING_FLAGS_Y1_CLIPPED: u32 = 8;
pub const CLIPPING_FLAGS_Y2_CLIPPED: u32 = 2;
pub const CLIPPING_FLAGS_X_CLIPPED: u32 = CLIPPING_FLAGS_X1_CLIPPED | CLIPPING_FLAGS_X2_CLIPPED;
pub const CLIPPING_FLAGS_Y_CLIPPED: u32 = CLIPPING_FLAGS_Y1_CLIPPED | CLIPPING_FLAGS_Y2_CLIPPED;

// agg_clip_liang_barsky.h:38-63
// //----------------------------------------------------------clipping_flags
// // Determine the clipping code of the vertex according to the
// // Cyrus-Beck line clipping algorithm
// //
// //        |        |
// //  0110  |  0010  | 0011
// //        |        |
// // -------+--------+-------- clip_box.y2
// //        |        |
// //  0100  |  0000  | 0001
// //        |        |
// // -------+--------+-------- clip_box.y1
// //        |        |
// //  1100  |  1000  | 1001
// //        |        |
// //  clip_box.x1  clip_box.x2
// template<class T>
// inline unsigned clipping_flags(T x, T y, const rect_base<T>& clip_box)
#[inline]
pub fn clipping_flags<T: PartialOrd + Copy>(x: T, y: T, clip_box: &RectBase<T>) -> u32 {
    (x > clip_box.x2) as u32
        | (((y > clip_box.y2) as u32) << 1)
        | (((x < clip_box.x1) as u32) << 2)
        | (((y < clip_box.y1) as u32) << 3)
}

// agg_clip_liang_barsky.h:65-70
// template<class T>
// inline unsigned clipping_flags_x(T x, const rect_base<T>& clip_box)
// {
//     return  (x > clip_box.x2) | ((x < clip_box.x1) << 2);
// }
#[inline]
pub fn clipping_flags_x<T: PartialOrd + Copy>(x: T, clip_box: &RectBase<T>) -> u32 {
    (x > clip_box.x2) as u32 | (((x < clip_box.x1) as u32) << 2)
}

// agg_clip_liang_barsky.h:73-78
// template<class T>
// inline unsigned clipping_flags_y(T y, const rect_base<T>& clip_box)
// {
//     return ((y > clip_box.y2) << 1) | ((y < clip_box.y1) << 3);
// }
#[inline]
pub fn clipping_flags_y<T: PartialOrd + Copy>(y: T, clip_box: &RectBase<T>) -> u32 {
    (((y > clip_box.y2) as u32) << 1) | (((y < clip_box.y1) as u32) << 3)
}
