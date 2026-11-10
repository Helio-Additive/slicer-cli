//! Faithful port of the vendored AGG header `src/agg/agg_basics.h`
//! (Anti-Grain Geometry 2.4, as bundled with BambuStudio) — the subset used by
//! the SLA grayscale rasterizer (`SLA/AGGRaster.hpp`).
//!
//! C++ Reference:
//! - agg/agg_basics.h
//!
//! Type mapping: `int` -> `i32`, `unsigned` -> `u32`, `double` -> `f64`,
//! `agg::int8u` -> `u8`. The `pod_allocator`/`obj_allocator` helpers
//! (agg_basics.h:36-54) are raw-memory allocation details with no Rust
//! equivalent (Vec handles allocation) and are not ported.

// agg_basics.h:188-191
// AGG_INLINE int iround(double v)
// {
//     return int((v < 0.0) ? v - 0.5 : v + 0.5);
// }
#[inline]
pub fn iround(v: f64) -> i32 {
    (if v < 0.0 { v - 0.5 } else { v + 0.5 }) as i32
}

// agg_basics.h:192-195
// AGG_INLINE int uround(double v)
// {
//     return unsigned(v + 0.5);
// }
// (declared `int` but computes `unsigned(v + 0.5)`; ported as u32 — every use
// site stores it into an `int` slot, mirrored with an `as i32` cast there)
#[inline]
pub fn uround(v: f64) -> u32 {
    (v + 0.5) as u32
}

// agg_basics.h:196-200
// AGG_INLINE int ifloor(double v)
// {
//     int i = int(v);
//     return i - (i > v);
// }
#[inline]
pub fn ifloor(v: f64) -> i32 {
    let i = v as i32;
    i - (i as f64 > v) as i32
}

// agg_basics.h:201-204
// AGG_INLINE unsigned ufloor(double v) { return unsigned(v); }
#[inline]
pub fn ufloor(v: f64) -> u32 {
    v as u32
}

// agg_basics.h:205-208
// AGG_INLINE int iceil(double v) { return int(ceil(v)); }
#[inline]
pub fn iceil(v: f64) -> i32 {
    v.ceil() as i32
}

// agg_basics.h:209-212
// AGG_INLINE unsigned uceil(double v) { return unsigned(ceil(v)); }
#[inline]
pub fn uceil(v: f64) -> u32 {
    v.ceil() as u32
}

// agg_basics.h:237  typedef unsigned char cover_type;    //----cover_type
pub type CoverType = u8;

// agg_basics.h:238-245  enum cover_scale_e
pub const COVER_SHIFT: u32 = 8; //----cover_shift
pub const COVER_SIZE: u32 = 1 << COVER_SHIFT; //----cover_size
pub const COVER_MASK: u32 = COVER_SIZE - 1; //----cover_mask
pub const COVER_NONE: u32 = 0; //----cover_none
pub const COVER_FULL: u32 = COVER_MASK; //----cover_full

// agg_basics.h:247-258  enum poly_subpixel_scale_e
// These constants determine the subpixel accuracy, to be more precise,
// the number of bits of the fractional part of the coordinates.
// The possible coordinate capacity in bits can be calculated by formula:
// sizeof(int) * 8 - poly_subpixel_shift, i.e, for 32-bit integers and
// 8-bits fractional part the capacity is 24 bits.
pub const POLY_SUBPIXEL_SHIFT: i32 = 8; //----poly_subpixel_shift
pub const POLY_SUBPIXEL_SCALE: i32 = 1 << POLY_SUBPIXEL_SHIFT; //----poly_subpixel_scale
pub const POLY_SUBPIXEL_MASK: i32 = POLY_SUBPIXEL_SCALE - 1; //----poly_subpixel_mask

// agg_basics.h:260-265  enum filling_rule_e
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FillingRule {
    FillNonZero,
    FillEvenOdd,
}

// agg_basics.h:282-330  template<class T> struct rect_base
// (only the i32 instantiation `rect_i` is needed by the rasterizer clipper)
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RectBase<T> {
    pub x1: T,
    pub y1: T,
    pub x2: T,
    pub y2: T,
}

impl<T: PartialOrd + Copy> RectBase<T> {
    // agg_basics.h:290-291
    // rect_base(T x1_, T y1_, T x2_, T y2_) : x1(x1_), y1(y1_), x2(x2_), y2(y2_) {}
    pub fn new(x1: T, y1: T, x2: T, y2: T) -> Self {
        Self { x1, y1, x2, y2 }
    }

    // agg_basics.h:293-296  void init(T x1_, T y1_, T x2_, T y2_)
    pub fn init(&mut self, x1: T, y1: T, x2: T, y2: T) {
        self.x1 = x1;
        self.y1 = y1;
        self.x2 = x2;
        self.y2 = y2;
    }

    // agg_basics.h:298-304  const self_type& normalize()
    pub fn normalize(&mut self) -> &Self {
        if self.x1 > self.x2 {
            std::mem::swap(&mut self.x1, &mut self.x2);
        }
        if self.y1 > self.y2 {
            std::mem::swap(&mut self.y1, &mut self.y2);
        }
        self
    }

    // agg_basics.h:306-313  bool clip(const self_type& r)
    pub fn clip(&mut self, r: &Self) -> bool {
        if self.x2 > r.x2 {
            self.x2 = r.x2;
        }
        if self.y2 > r.y2 {
            self.y2 = r.y2;
        }
        if self.x1 < r.x1 {
            self.x1 = r.x1;
        }
        if self.y1 < r.y1 {
            self.y1 = r.y1;
        }
        self.x1 <= self.x2 && self.y1 <= self.y2
    }

    // agg_basics.h:315-318  bool is_valid() const
    pub fn is_valid(&self) -> bool {
        self.x1 <= self.x2 && self.y1 <= self.y2
    }

    // agg_basics.h:320-323  bool hit_test(T x, T y) const
    pub fn hit_test(&self, x: T, y: T) -> bool {
        x >= self.x1 && x <= self.x2 && y >= self.y1 && y <= self.y2
    }

    // agg_basics.h:325-329  bool overlaps(const self_type& r) const
    pub fn overlaps(&self, r: &Self) -> bool {
        !(r.x1 > self.x2 || r.x2 < self.x1 || r.y1 > self.y2 || r.y2 < self.y1)
    }
}

// agg_basics.h:363  typedef rect_base<int>    rect_i; //----rect_i
pub type RectI = RectBase<i32>;
// agg_basics.h:365  typedef rect_base<double> rect_d; //----rect_d
pub type RectD = RectBase<f64>;

// agg_basics.h:367-380  enum path_commands_e
pub const PATH_CMD_STOP: u32 = 0; //----path_cmd_stop
pub const PATH_CMD_MOVE_TO: u32 = 1; //----path_cmd_move_to
pub const PATH_CMD_LINE_TO: u32 = 2; //----path_cmd_line_to
pub const PATH_CMD_CURVE3: u32 = 3; //----path_cmd_curve3
pub const PATH_CMD_CURVE4: u32 = 4; //----path_cmd_curve4
pub const PATH_CMD_CURVE_N: u32 = 5; //----path_cmd_curveN
pub const PATH_CMD_CATROM: u32 = 6; //----path_cmd_catrom
pub const PATH_CMD_UBSPLINE: u32 = 7; //----path_cmd_ubspline
pub const PATH_CMD_END_POLY: u32 = 0x0F; //----path_cmd_end_poly
pub const PATH_CMD_MASK: u32 = 0x0F; //----path_cmd_mask

// agg_basics.h:382-390  enum path_flags_e
pub const PATH_FLAGS_NONE: u32 = 0; //----path_flags_none
pub const PATH_FLAGS_CCW: u32 = 0x10; //----path_flags_ccw
pub const PATH_FLAGS_CW: u32 = 0x20; //----path_flags_cw
pub const PATH_FLAGS_CLOSE: u32 = 0x40; //----path_flags_close
pub const PATH_FLAGS_MASK: u32 = 0xF0; //----path_flags_mask

// agg_basics.h:392-396  inline bool is_vertex(unsigned c)
#[inline]
pub fn is_vertex(c: u32) -> bool {
    c >= PATH_CMD_MOVE_TO && c < PATH_CMD_END_POLY
}

// agg_basics.h:398-402  inline bool is_drawing(unsigned c)
#[inline]
pub fn is_drawing(c: u32) -> bool {
    c >= PATH_CMD_LINE_TO && c < PATH_CMD_END_POLY
}

// agg_basics.h:404-408  inline bool is_stop(unsigned c)
#[inline]
pub fn is_stop(c: u32) -> bool {
    c == PATH_CMD_STOP
}

// agg_basics.h:410-414  inline bool is_move_to(unsigned c)
#[inline]
pub fn is_move_to(c: u32) -> bool {
    c == PATH_CMD_MOVE_TO
}

// agg_basics.h:416-420  inline bool is_line_to(unsigned c)
#[inline]
pub fn is_line_to(c: u32) -> bool {
    c == PATH_CMD_LINE_TO
}

// agg_basics.h:422-426  inline bool is_curve(unsigned c)
#[inline]
pub fn is_curve(c: u32) -> bool {
    c == PATH_CMD_CURVE3 || c == PATH_CMD_CURVE4
}

// agg_basics.h:440-444  inline bool is_end_poly(unsigned c)
#[inline]
pub fn is_end_poly(c: u32) -> bool {
    (c & PATH_CMD_MASK) == PATH_CMD_END_POLY
}

// agg_basics.h:446-451
// inline bool is_close(unsigned c)
// {
//     return (c & ~(path_flags_cw | path_flags_ccw)) ==
//            (path_cmd_end_poly | path_flags_close);
// }
#[inline]
pub fn is_close(c: u32) -> bool {
    (c & !(PATH_FLAGS_CW | PATH_FLAGS_CCW)) == (PATH_CMD_END_POLY | PATH_FLAGS_CLOSE)
}

// agg_basics.h:453-457  inline bool is_next_poly(unsigned c)
#[inline]
pub fn is_next_poly(c: u32) -> bool {
    is_stop(c) || is_move_to(c) || is_end_poly(c)
}

// agg_basics.h:459-463  inline bool is_cw(unsigned c)
#[inline]
pub fn is_cw(c: u32) -> bool {
    (c & PATH_FLAGS_CW) != 0
}

// agg_basics.h:465-469  inline bool is_ccw(unsigned c)
#[inline]
pub fn is_ccw(c: u32) -> bool {
    (c & PATH_FLAGS_CCW) != 0
}

// agg_basics.h:471-475  inline bool is_oriented(unsigned c)
#[inline]
pub fn is_oriented(c: u32) -> bool {
    (c & (PATH_FLAGS_CW | PATH_FLAGS_CCW)) != 0
}

// agg_basics.h:477-481  inline bool is_closed(unsigned c)
#[inline]
pub fn is_closed(c: u32) -> bool {
    (c & PATH_FLAGS_CLOSE) != 0
}

// agg_basics.h:483-487  inline unsigned get_close_flag(unsigned c)
#[inline]
pub fn get_close_flag(c: u32) -> u32 {
    c & PATH_FLAGS_CLOSE
}

// agg_basics.h:489-493  inline unsigned clear_orientation(unsigned c)
#[inline]
pub fn clear_orientation(c: u32) -> u32 {
    c & !(PATH_FLAGS_CW | PATH_FLAGS_CCW)
}

// agg_basics.h:495-499  inline unsigned get_orientation(unsigned c)
#[inline]
pub fn get_orientation(c: u32) -> u32 {
    c & (PATH_FLAGS_CW | PATH_FLAGS_CCW)
}

// agg_basics.h:501-505  inline unsigned set_orientation(unsigned c, unsigned o)
#[inline]
pub fn set_orientation(c: u32, o: u32) -> u32 {
    clear_orientation(c) | o
}
