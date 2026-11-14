//! `boost::polygon`'s Voronoi builder for points and segments (Boost 1.90), in Rust.
//!
//! What `boost::polygon::construct_voronoi(first, last, &vd)` does with the default
//! `voronoi_builder<int32>` and `voronoi_diagram<double>`, header for header:
//!
//! | header | here |
//! |-|-|
//! | `detail/voronoi_ctypes.hpp` | [`ulp_comparison`], [`ExtendedExponentFpt`], [`ExtendedInt`], the converters |
//! | `detail/voronoi_robust_fpt.hpp` | [`RobustFpt`], [`RobustDif`], [`RobustSqrtExpr`] |
//! | `detail/voronoi_structures.hpp` | [`Point2D`], [`SiteEvent`], [`CircleEvent`], [`OrderedQueue`], [`BeachLineNodeKey`], [`BeachLineNodeData`] |
//! | `detail/voronoi_predicates.hpp` | the predicates and the circle formation functors |
//! | `voronoi_builder.hpp` | [`VoronoiBuilder`] |
//! | `voronoi_diagram.hpp` | [`VoronoiDiagram`], [`VoronoiCell`], [`VoronoiVertex`], [`VoronoiEdge`] |
//! | `voronoi.hpp` | [`construct_voronoi`], [`construct_voronoi_points`], [`construct_voronoi_points_segments`] |
//!
//! The result is the C++ one bit for bit: the same vertices in the same order, the same
//! half-edges in the same order with the same links, the same cells. Three things beyond the
//! headers decide that, and each is reproduced here:
//!
//! * **The standard library.** The beach line is a `std::map`, and its comparison is a
//!   floating-point predicate that is only consistent for the node being inserted: which
//!   stored nodes a search compares with depends on the shape of the red-black tree. The
//!   tree below is libc++'s `__tree` (`__find_equal` with a hint, `__tree_balance_after_insert`,
//!   `__tree_remove`, `__lower_bound`). The circle events are popped from a
//!   `std::priority_queue`, where events with equal keys leave in heap order: the heap is
//!   libc++'s `__sift_up` / `__floyd_sift_down`. The sites are sorted with libc++'s
//!   `std::sort` ([`crate::clipper::libcxx_sort`]).
//! * **Floating-point contraction.** The reference is compiled with `-ffp-contract=on`, which
//!   turns `a * b + c` inside one expression into a fused multiply-add. `mul_add` stands
//!   exactly where the compiler contracts, with the C++ expression next to it.
//! * **Pointers are indices.** A pointer into `cells_`, `vertices_` or `edges_` is the index
//!   in that vector (`Option<usize>` where the C++ pointer may be null). `_build()` moves
//!   edges and vertices down inside their vectors and patches the pointers; the same moves
//!   and patches are made on the indices.

//!
//! What is not the C++:
//!
//! * FIDELITY-NOTE: the standard library reproduced is libc++ (what the reference is built
//!   with on macOS). libstdc++ and the MSVC library have their own `std::sort`, heap and
//!   red-black tree; where those leave an order open (site events that compare equivalent,
//!   circle events with equal `(lower_x, y)`), a reference built with them may order
//!   differently. That has not been compared.
//! * FIDELITY-NOTE: input outside the contract (segments that cross or overlap) makes the
//!   C++ dereference null or dangling pointers, which is undefined; here it is a panic
//!   (`expect`) or a diagram that need not be the one a given C++ build happens to produce.
//!   The same holds for more sites than `_reserve` made room for, where the C++ vectors
//!   reallocate under its pointers and the indices here simply stay valid.
//! * Values the C++ leaves uninitialised (the chunks of `extended_int`, the coordinates of a
//!   default `circle_event`, `initial_index_`) are zero here; none is read before it is set.

#![allow(clippy::too_many_arguments, clippy::needless_range_loop, clippy::collapsible_else_if, clippy::many_single_char_names)]

use crate::clipper::libcxx_sort;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};
use std::sync::atomic::{AtomicU64, Ordering};

// ---------------------------------------------------------------------------
// Counters. They are for the tests, which assert that the rare paths were
// reached; nothing reads them to take a decision.
// ---------------------------------------------------------------------------

/// How often the rare paths ran, process-wide. Read with [`counters`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counters {
    /// `mp_circle_formation_functor::ppp` calls (the exact fallback).
    pub exact_ppp: u64,
    /// `mp_circle_formation_functor::pps` calls.
    pub exact_pps: u64,
    /// `mp_circle_formation_functor::pss` calls.
    pub exact_pss: u64,
    /// `mp_circle_formation_functor::sss` calls.
    pub exact_sss: u64,
    /// `robust_sqrt_expr::eval2..4` and `sqrt_expr_evaluator_pss3/4` taking the
    /// branch that multiplies by the conjugate (operands of opposite sign).
    pub sqrt_expr_conjugate: u64,
    /// Degenerate edges removed by `_build()`.
    pub degenerate_edges_removed: u64,
    /// Vertices left without an incident edge and removed by `_build()`.
    pub degenerate_vertices_removed: u64,
    /// Beach line insertions whose hint was wrong (`__find_equal` fell back to the
    /// search from the root).
    pub beach_line_hint_missed: u64,
    /// Beach line insertions that found an equivalent key and inserted nothing.
    pub beach_line_key_found: u64,
    /// Circle events popped while another event with the same `(lower_x, y)` was queued.
    pub circle_event_ties: u64,
    /// `distance_predicate::ps` calls that `fast_ps` left undefined.
    pub fast_ps_undefined: u64,
    /// `init_beach_line_collinear_sites` calls.
    pub collinear_init: u64,
}

static EXACT_PPP: AtomicU64 = AtomicU64::new(0);
static EXACT_PPS: AtomicU64 = AtomicU64::new(0);
static EXACT_PSS: AtomicU64 = AtomicU64::new(0);
static EXACT_SSS: AtomicU64 = AtomicU64::new(0);
static SQRT_EXPR_CONJUGATE: AtomicU64 = AtomicU64::new(0);
static DEGENERATE_EDGES_REMOVED: AtomicU64 = AtomicU64::new(0);
static DEGENERATE_VERTICES_REMOVED: AtomicU64 = AtomicU64::new(0);
static BEACH_LINE_HINT_MISSED: AtomicU64 = AtomicU64::new(0);
static BEACH_LINE_KEY_FOUND: AtomicU64 = AtomicU64::new(0);
static CIRCLE_EVENT_TIES: AtomicU64 = AtomicU64::new(0);
static FAST_PS_UNDEFINED: AtomicU64 = AtomicU64::new(0);
static COLLINEAR_INIT: AtomicU64 = AtomicU64::new(0);

#[inline]
fn count(c: &AtomicU64) {
    c.fetch_add(1, Ordering::Relaxed);
}

/// The counters since the process started.
pub fn counters() -> Counters {
    let g = |c: &AtomicU64| c.load(Ordering::Relaxed);
    Counters {
        exact_ppp: g(&EXACT_PPP),
        exact_pps: g(&EXACT_PPS),
        exact_pss: g(&EXACT_PSS),
        exact_sss: g(&EXACT_SSS),
        sqrt_expr_conjugate: g(&SQRT_EXPR_CONJUGATE),
        degenerate_edges_removed: g(&DEGENERATE_EDGES_REMOVED),
        degenerate_vertices_removed: g(&DEGENERATE_VERTICES_REMOVED),
        beach_line_hint_missed: g(&BEACH_LINE_HINT_MISSED),
        beach_line_key_found: g(&BEACH_LINE_KEY_FOUND),
        circle_event_ties: g(&CIRCLE_EVENT_TIES),
        fast_ps_undefined: g(&FAST_PS_UNDEFINED),
        collinear_init: g(&COLLINEAR_INIT),
    }
}

// ===========================================================================
// voronoi_geometry_type.hpp
// ===========================================================================

/// voronoi_geometry_type.hpp:18-21 `GeometryCategory`.
pub const GEOMETRY_CATEGORY_POINT: usize = 0x0;
pub const GEOMETRY_CATEGORY_SEGMENT: usize = 0x1;

/// voronoi_geometry_type.hpp:24-36 `SourceCategory`.
pub const SOURCE_CATEGORY_SINGLE_POINT: usize = 0x0;
pub const SOURCE_CATEGORY_SEGMENT_START_POINT: usize = 0x1;
pub const SOURCE_CATEGORY_SEGMENT_END_POINT: usize = 0x2;
pub const SOURCE_CATEGORY_INITIAL_SEGMENT: usize = 0x8;
pub const SOURCE_CATEGORY_REVERSE_SEGMENT: usize = 0x9;
pub const SOURCE_CATEGORY_GEOMETRY_SHIFT: usize = 0x3;
pub const SOURCE_CATEGORY_BITMASK: usize = 0x1F;

/// voronoi_geometry_type.hpp:38-44 `belongs`.
pub fn belongs(source_category: usize, geometry_category: usize) -> bool {
    (source_category >> SOURCE_CATEGORY_GEOMETRY_SHIFT) == geometry_category
}

// ===========================================================================
// detail/voronoi_ctypes.hpp
// ===========================================================================

/// voronoi_ctypes.hpp:40-44 `ulp_comparison<fpt64>::Result`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UlpResult {
    Less = -1,
    Equal = 0,
    More = 1,
}

/// voronoi_ctypes.hpp:46-68 `ulp_comparison<fpt64>::operator()`.
pub fn ulp_comparison(a: f64, b: f64, max_ulps: u32) -> UlpResult {
    let mut ll_a = a.to_bits();
    let mut ll_b = b.to_bits();
    if ll_a < 0x8000_0000_0000_0000 {
        ll_a = 0x8000_0000_0000_0000 - ll_a;
    }
    if ll_b < 0x8000_0000_0000_0000 {
        ll_b = 0x8000_0000_0000_0000 - ll_b;
    }
    if ll_a > ll_b {
        return if ll_a - ll_b <= max_ulps as u64 { UlpResult::Equal } else { UlpResult::Less };
    }
    if ll_b - ll_a <= max_ulps as u64 {
        UlpResult::Equal
    } else {
        UlpResult::More
    }
}

/// `std::frexp`: `x = m * 2^e` with `0.5 <= |m| < 1`; `(x, 0)` for zero, infinities and NaN.
fn frexp(x: f64) -> (f64, i32) {
    let bits = x.to_bits();
    let ee = ((bits >> 52) & 0x7ff) as i32;
    if ee == 0 {
        if x != 0.0 {
            // Subnormal: scale into the normal range first (2^64 is exact).
            let (y, e) = frexp(x * 18446744073709551616.0);
            return (y, e - 64);
        }
        return (x, 0);
    } else if ee == 0x7ff {
        return (x, 0);
    }
    (f64::from_bits((bits & 0x800f_ffff_ffff_ffff) | 0x3fe0_0000_0000_0000), ee - 0x3fe)
}

/// `std::ldexp`: `x * 2^n`, rounded once (the steps before the last multiplication are exact
/// or already saturated).
fn ldexp(x: f64, n: i32) -> f64 {
    let mut y = x;
    let mut n = n;
    if n > 1023 {
        y *= f64::from_bits(0x7fe0_0000_0000_0000); // 2^1023
        n -= 1023;
        if n > 1023 {
            y *= f64::from_bits(0x7fe0_0000_0000_0000);
            n -= 1023;
            if n > 1023 {
                n = 1023;
            }
        }
    } else if n < -1022 {
        // 2^-1022 * 2^53: keeps the intermediate normal so the final product rounds once.
        let k = f64::from_bits(0x0010_0000_0000_0000) * 9007199254740992.0;
        y *= k;
        n += 1022 - 53;
        if n < -1022 {
            y *= k;
            n += 1022 - 53;
            if n < -1022 {
                n = -1022;
            }
        }
    }
    y * f64::from_bits(((0x3ff + n) as u64) << 52)
}

/// voronoi_ctypes.hpp:75-81 `extened_exponent_fpt_traits<fpt64>::MAX_SIGNIFICANT_EXP_DIF`.
const MAX_SIGNIFICANT_EXP_DIF: i32 = 54;

/// voronoi_ctypes.hpp:86-202 `extended_exponent_fpt<double>`: a double with an `int` exponent.
#[derive(Clone, Copy, Debug)]
pub struct ExtendedExponentFpt {
    val: f64,
    exp: i32,
}

impl ExtendedExponentFpt {
    /// voronoi_ctypes.hpp:92-94.
    pub fn new(val: f64) -> Self {
        let (val, exp) = frexp(val);
        ExtendedExponentFpt { val, exp }
    }

    /// voronoi_ctypes.hpp:96-99.
    pub fn new2(val: f64, exp: i32) -> Self {
        let (val, e) = frexp(val);
        ExtendedExponentFpt { val, exp: e.wrapping_add(exp) }
    }

    /// voronoi_ctypes.hpp:101-103.
    pub fn is_pos(&self) -> bool {
        self.val > 0.0
    }

    /// voronoi_ctypes.hpp:105-107.
    pub fn is_neg(&self) -> bool {
        self.val < 0.0
    }

    /// voronoi_ctypes.hpp:109-111.
    pub fn is_zero(&self) -> bool {
        self.val == 0.0
    }

    /// voronoi_ctypes.hpp:185-193.
    pub fn sqrt(&self) -> Self {
        let mut val = self.val;
        let mut exp = self.exp;
        if exp & 1 != 0 {
            val *= 2.0;
            exp -= 1;
        }
        ExtendedExponentFpt::new2(val.sqrt(), exp >> 1)
    }

    /// voronoi_ctypes.hpp:195-197.
    pub fn d(&self) -> f64 {
        ldexp(self.val, self.exp)
    }
}

/// voronoi_ctypes.hpp:113-115.
impl Neg for ExtendedExponentFpt {
    type Output = Self;
    fn neg(self) -> Self {
        ExtendedExponentFpt::new2(-self.val, self.exp)
    }
}

/// voronoi_ctypes.hpp:117-135.
impl Add for ExtendedExponentFpt {
    type Output = Self;
    fn add(self, that: Self) -> Self {
        if self.val == 0.0 || that.exp > self.exp + MAX_SIGNIFICANT_EXP_DIF {
            return that;
        }
        if that.val == 0.0 || self.exp > that.exp + MAX_SIGNIFICANT_EXP_DIF {
            return self;
        }
        if self.exp >= that.exp {
            let exp_dif = self.exp - that.exp;
            let val = ldexp(self.val, exp_dif) + that.val;
            ExtendedExponentFpt::new2(val, that.exp)
        } else {
            let exp_dif = that.exp - self.exp;
            let val = ldexp(that.val, exp_dif) + self.val;
            ExtendedExponentFpt::new2(val, self.exp)
        }
    }
}

/// voronoi_ctypes.hpp:137-155.
impl Sub for ExtendedExponentFpt {
    type Output = Self;
    fn sub(self, that: Self) -> Self {
        if self.val == 0.0 || that.exp > self.exp + MAX_SIGNIFICANT_EXP_DIF {
            return ExtendedExponentFpt::new2(-that.val, that.exp);
        }
        if that.val == 0.0 || self.exp > that.exp + MAX_SIGNIFICANT_EXP_DIF {
            return self;
        }
        if self.exp >= that.exp {
            let exp_dif = self.exp - that.exp;
            let val = ldexp(self.val, exp_dif) - that.val;
            ExtendedExponentFpt::new2(val, that.exp)
        } else {
            let exp_dif = that.exp - self.exp;
            let val = ldexp(-that.val, exp_dif) + self.val;
            ExtendedExponentFpt::new2(val, self.exp)
        }
    }
}

/// voronoi_ctypes.hpp:157-161.
impl Mul for ExtendedExponentFpt {
    type Output = Self;
    fn mul(self, that: Self) -> Self {
        let val = self.val * that.val;
        let exp = self.exp.wrapping_add(that.exp);
        ExtendedExponentFpt::new2(val, exp)
    }
}

/// voronoi_ctypes.hpp:163-167.
impl Div for ExtendedExponentFpt {
    type Output = Self;
    fn div(self, that: Self) -> Self {
        let val = self.val / that.val;
        let exp = self.exp.wrapping_sub(that.exp);
        ExtendedExponentFpt::new2(val, exp)
    }
}

/// The `N` of `extended_int<N>` in `voronoi_ctype_traits<int32>` (voronoi_ctypes.hpp:632).
pub const EXTENDED_INT_N: usize = 64;

/// voronoi_ctypes.hpp:227-580 `extended_int<64>`: sign-magnitude, 32-bit chunks, least
/// significant first; `count` is the number of chunks, negative for a negative number.
///
/// The C++ default constructor leaves the chunks uninitialised; here they start as zero. No
/// operation reads a chunk at or beyond `size()`, so the difference cannot be observed.
#[derive(Clone, Copy)]
pub struct ExtendedInt {
    chunks: [u32; EXTENDED_INT_N],
    count: i32,
}

impl std::fmt::Debug for ExtendedInt {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "ExtendedInt({}{:x?})", if self.count < 0 { "-" } else { "" }, &self.chunks[..self.size()])
    }
}

impl Default for ExtendedInt {
    /// voronoi_ctypes.hpp:230 (see the note on the type).
    fn default() -> Self {
        ExtendedInt { chunks: [0; EXTENDED_INT_N], count: 0 }
    }
}

impl From<i32> for ExtendedInt {
    /// voronoi_ctypes.hpp:232-242 (and `operator=(int32)`, 273-284).
    fn from(that: i32) -> Self {
        let mut r = ExtendedInt::default();
        if that > 0 {
            r.chunks[0] = that as u32;
            r.count = 1;
        } else if that < 0 {
            r.chunks[0] = that.wrapping_neg() as u32;
            r.count = -1;
        } else {
            r.count = 0;
        }
        r
    }
}

impl From<i64> for ExtendedInt {
    /// voronoi_ctypes.hpp:244-257 (and `operator=(int64)`, 286-300).
    fn from(that: i64) -> Self {
        let mut r = ExtendedInt::default();
        if that > 0 {
            r.chunks[0] = that as u32;
            r.chunks[1] = (that >> 32) as u32;
            r.count = if r.chunks[1] != 0 { 2 } else { 1 };
        } else if that < 0 {
            let that = that.wrapping_neg();
            r.chunks[0] = that as u32;
            r.chunks[1] = (that >> 32) as u32;
            r.count = if r.chunks[1] != 0 { -2 } else { -1 };
        } else {
            r.count = 0;
        }
        r
    }
}

impl ExtendedInt {
    /// voronoi_ctypes.hpp:259-265 `extended_int(const std::vector<uint32>&, bool plus)`:
    /// `chunks` most significant first.
    pub fn from_chunks(chunks: &[u32], plus: bool) -> Self {
        let mut r = ExtendedInt::default();
        r.count = EXTENDED_INT_N.min(chunks.len()) as i32;
        for i in 0..r.count as usize {
            r.chunks[i] = chunks[chunks.len() - i - 1];
        }
        if !plus {
            r.count = -r.count;
        }
        r
    }

    /// voronoi_ctypes.hpp:309-311.
    pub fn is_pos(&self) -> bool {
        self.count > 0
    }

    /// voronoi_ctypes.hpp:313-315.
    pub fn is_neg(&self) -> bool {
        self.count < 0
    }

    /// voronoi_ctypes.hpp:317-319.
    pub fn is_zero(&self) -> bool {
        self.count == 0
    }

    /// voronoi_ctypes.hpp:334-346 `operator<`.
    pub fn lt(&self, that: &ExtendedInt) -> bool {
        if self.count != that.count {
            return self.count < that.count;
        }
        let mut i = self.size();
        if i == 0 {
            return false;
        }
        loop {
            i -= 1;
            if self.chunks[i] != that.chunks[i] {
                return (self.chunks[i] < that.chunks[i]) ^ (self.count < 0);
            }
            if i == 0 {
                break;
            }
        }
        false
    }

    /// voronoi_ctypes.hpp:366-368.
    pub fn neg_in_place(&mut self) {
        self.count = -self.count;
    }

    /// voronoi_ctypes.hpp:376-392 `add(e1, e2)`.
    pub fn add(&mut self, e1: &ExtendedInt, e2: &ExtendedInt) {
        if e1.count == 0 {
            *self = *e2;
            return;
        }
        if e2.count == 0 {
            *self = *e1;
            return;
        }
        if (e1.count > 0) ^ (e2.count > 0) {
            self.dif_chunks(&e1.chunks, e1.size(), &e2.chunks, e2.size(), false);
        } else {
            self.add_chunks(&e1.chunks, e1.size(), &e2.chunks, e2.size());
        }
        if e1.count < 0 {
            self.count = -self.count;
        }
    }

    /// voronoi_ctypes.hpp:400-417 `dif(e1, e2)`.
    pub fn dif(&mut self, e1: &ExtendedInt, e2: &ExtendedInt) {
        if e1.count == 0 {
            *self = *e2;
            self.count = -self.count;
            return;
        }
        if e2.count == 0 {
            *self = *e1;
            return;
        }
        if (e1.count > 0) ^ (e2.count > 0) {
            self.add_chunks(&e1.chunks, e1.size(), &e2.chunks, e2.size());
        } else {
            self.dif_chunks(&e1.chunks, e1.size(), &e2.chunks, e2.size(), false);
        }
        if e1.count < 0 {
            self.count = -self.count;
        }
    }

    /// voronoi_ctypes.hpp:435-443 `mul(e1, e2)`.
    pub fn mul(&mut self, e1: &ExtendedInt, e2: &ExtendedInt) {
        if e1.count == 0 || e2.count == 0 {
            self.count = 0;
            return;
        }
        self.mul_chunks(&e1.chunks, e1.size(), &e2.chunks, e2.size());
        if (e1.count > 0) ^ (e2.count > 0) {
            self.count = -self.count;
        }
    }

    /// voronoi_ctypes.hpp:445-447.
    pub fn chunks(&self) -> &[u32] {
        &self.chunks[..self.size()]
    }

    /// voronoi_ctypes.hpp:449-451.
    pub fn count(&self) -> i32 {
        self.count
    }

    /// voronoi_ctypes.hpp:453-455.
    pub fn size(&self) -> usize {
        self.count.unsigned_abs() as usize
    }

    /// voronoi_ctypes.hpp:457-480 `p()`: the three most significant chunks as a double and
    /// the exponent of the rest.
    pub fn p(&self) -> (f64, i32) {
        let mut ret_val: (f64, i32) = (0.0, 0);
        let sz = self.size();
        if sz == 0 {
            return ret_val;
        } else if sz == 1 {
            ret_val.0 = self.chunks[0] as f64;
        } else if sz == 2 {
            // `chunks_[1] * 2^32 + chunks_[0]` in one expression: contracted. (The product
            // is exact, so the fused and the separate form agree.)
            ret_val.0 = (self.chunks[1] as f64).mul_add(4294967296.0, self.chunks[0] as f64);
        } else {
            for i in 1..=3 {
                ret_val.0 *= 4294967296.0;
                ret_val.0 += self.chunks[sz - i] as f64;
            }
            ret_val.1 = ((sz - 3) << 5) as i32;
        }
        if self.count < 0 {
            ret_val.0 = -ret_val.0;
        }
        ret_val
    }

    /// voronoi_ctypes.hpp:482-485 `d()`.
    pub fn d(&self) -> f64 {
        let p = self.p();
        ldexp(p.0, p.1)
    }

    /// voronoi_ctypes.hpp:488-510 the private `add` on chunk arrays.
    fn add_chunks(&mut self, c1: &[u32; EXTENDED_INT_N], sz1: usize, c2: &[u32; EXTENDED_INT_N], sz2: usize) {
        if sz1 < sz2 {
            self.add_chunks(c2, sz2, c1, sz1);
            return;
        }
        self.count = sz1 as i32;
        let mut temp: u64 = 0;
        for i in 0..sz2 {
            temp += c1[i] as u64 + c2[i] as u64;
            self.chunks[i] = temp as u32;
            temp >>= 32;
        }
        for i in sz2..sz1 {
            temp += c1[i] as u64;
            self.chunks[i] = temp as u32;
            temp >>= 32;
        }
        if temp != 0 && self.count as usize != EXTENDED_INT_N {
            self.chunks[self.count as usize] = temp as u32;
            self.count += 1;
        }
    }

    /// voronoi_ctypes.hpp:512-550 the private `dif` on chunk arrays. As in the C++, the
    /// result may keep zero chunks below the top one it drops (`count_` is only corrected
    /// by one).
    fn dif_chunks(&mut self, c1: &[u32; EXTENDED_INT_N], mut sz1: usize, c2: &[u32; EXTENDED_INT_N], mut sz2: usize, rec: bool) {
        if sz1 < sz2 {
            self.dif_chunks(c2, sz2, c1, sz1, true);
            self.count = -self.count;
            return;
        } else if sz1 == sz2 && !rec {
            loop {
                sz1 -= 1;
                if c1[sz1] < c2[sz1] {
                    sz1 += 1;
                    self.dif_chunks(c2, sz1, c1, sz1, true);
                    self.count = -self.count;
                    return;
                } else if c1[sz1] > c2[sz1] {
                    sz1 += 1;
                    break;
                }
                if sz1 == 0 {
                    break;
                }
            }
            if sz1 == 0 {
                self.count = 0;
                return;
            }
            sz2 = sz1;
        }
        self.count = sz1 as i32 - 1;
        let mut flag = false;
        for i in 0..sz2 {
            self.chunks[i] = c1[i].wrapping_sub(c2[i]).wrapping_sub(flag as u32);
            flag = (c1[i] < c2[i]) || ((c1[i] == c2[i]) && flag);
        }
        for i in sz2..sz1 {
            self.chunks[i] = c1[i].wrapping_sub(flag as u32);
            flag = c1[i] == 0 && flag;
        }
        if self.chunks[self.count as usize] != 0 {
            self.count += 1;
        }
    }

    /// voronoi_ctypes.hpp:552-576 the private `mul` on chunk arrays.
    fn mul_chunks(&mut self, c1: &[u32; EXTENDED_INT_N], sz1: usize, c2: &[u32; EXTENDED_INT_N], sz2: usize) {
        let mut cur: u64 = 0;
        self.count = EXTENDED_INT_N.min(sz1 + sz2 - 1) as i32;
        for shift in 0..self.count as usize {
            let mut nxt: u64 = 0;
            for first in 0..=shift {
                if first >= sz1 {
                    break;
                }
                let second = shift - first;
                if second >= sz2 {
                    continue;
                }
                let tmp = c1[first] as u64 * c2[second] as u64;
                cur = cur.wrapping_add((tmp as u32) as u64);
                nxt = nxt.wrapping_add(tmp >> 32);
            }
            self.chunks[shift] = cur as u32;
            cur = nxt.wrapping_add(cur >> 32);
        }
        if cur != 0 && self.count as usize != EXTENDED_INT_N {
            self.chunks[self.count as usize] = cur as u32;
            self.count += 1;
        }
    }
}

/// voronoi_ctypes.hpp:321-328 `operator==`.
impl PartialEq for ExtendedInt {
    fn eq(&self, that: &ExtendedInt) -> bool {
        if self.count != that.count {
            return false;
        }
        for i in 0..self.size() {
            if self.chunks[i] != that.chunks[i] {
                return false;
            }
        }
        true
    }
}

/// voronoi_ctypes.hpp:360-364 `operator-()`.
impl Neg for ExtendedInt {
    type Output = ExtendedInt;
    fn neg(self) -> ExtendedInt {
        let mut ret_val = self;
        ret_val.neg_in_place();
        ret_val
    }
}

/// voronoi_ctypes.hpp:370-374 `operator+`.
impl Add for ExtendedInt {
    type Output = ExtendedInt;
    fn add(self, that: ExtendedInt) -> ExtendedInt {
        let mut ret_val = ExtendedInt::default();
        ExtendedInt::add(&mut ret_val, &self, &that);
        ret_val
    }
}

/// voronoi_ctypes.hpp:394-398 `operator-`.
impl Sub for ExtendedInt {
    type Output = ExtendedInt;
    fn sub(self, that: ExtendedInt) -> ExtendedInt {
        let mut ret_val = ExtendedInt::default();
        ret_val.dif(&self, &that);
        ret_val
    }
}

/// voronoi_ctypes.hpp:429-433 `operator*(const extended_int&)`.
impl Mul for ExtendedInt {
    type Output = ExtendedInt;
    fn mul(self, that: ExtendedInt) -> ExtendedInt {
        let mut ret_val = ExtendedInt::default();
        ExtendedInt::mul(&mut ret_val, &self, &that);
        ret_val
    }
}

/// voronoi_ctypes.hpp:419-422 `operator*(int32)`.
impl Mul<i32> for ExtendedInt {
    type Output = ExtendedInt;
    fn mul(self, that: i32) -> ExtendedInt {
        self * ExtendedInt::from(that)
    }
}

/// voronoi_ctypes.hpp:424-427 `operator*(int64)`.
impl Mul<i64> for ExtendedInt {
    type Output = ExtendedInt;
    fn mul(self, that: i64) -> ExtendedInt {
        self * ExtendedInt::from(that)
    }
}

/// voronoi_ctypes.hpp:603-606 `type_converter_fpt::operator()(const extended_int<N>&)`.
fn to_fpt_int(that: &ExtendedInt) -> f64 {
    that.d()
}

/// voronoi_ctypes.hpp:608-610 `type_converter_fpt::operator()(const extended_exponent_fpt&)`.
fn to_fpt_efpt(that: &ExtendedExponentFpt) -> f64 {
    that.d()
}

/// voronoi_ctypes.hpp:613-619 `type_converter_efpt`.
fn to_efpt(that: &ExtendedInt) -> ExtendedExponentFpt {
    let p = that.p();
    ExtendedExponentFpt::new2(p.0, p.1)
}

// ===========================================================================
// detail/voronoi_robust_fpt.hpp
// ===========================================================================

/// voronoi_robust_fpt.hpp:86-88 `ROUNDING_ERROR`.
const ROUNDING_ERROR: f64 = 1.0;

/// voronoi_robust_fpt.hpp:79-213 `robust_fpt<double>`: a value and its relative error in
/// epsilons.
#[derive(Clone, Copy, Debug)]
pub struct RobustFpt {
    fpv: f64,
    re: f64,
}

impl Default for RobustFpt {
    /// voronoi_robust_fpt.hpp:90.
    fn default() -> Self {
        RobustFpt { fpv: 0.0, re: 0.0 }
    }
}

impl RobustFpt {
    /// voronoi_robust_fpt.hpp:91-92.
    pub fn new(fpv: f64) -> Self {
        RobustFpt { fpv, re: 0.0 }
    }

    /// voronoi_robust_fpt.hpp:93-94.
    pub fn new2(fpv: f64, error: f64) -> Self {
        RobustFpt { fpv, re: error }
    }

    /// voronoi_robust_fpt.hpp:96.
    pub fn fpv(&self) -> f64 {
        self.fpv
    }

    /// voronoi_robust_fpt.hpp:97.
    pub fn re(&self) -> f64 {
        self.re
    }

    /// voronoi_robust_fpt.hpp:98.
    pub fn ulp(&self) -> f64 {
        self.re
    }

    /// voronoi_robust_fpt.hpp:100-102.
    pub fn has_pos_value(&self) -> bool {
        self.fpv > 0.0
    }

    /// voronoi_robust_fpt.hpp:104-106.
    pub fn has_neg_value(&self) -> bool {
        self.fpv < 0.0
    }

    /// voronoi_robust_fpt.hpp:108-110.
    pub fn has_zero_value(&self) -> bool {
        self.fpv == 0.0
    }

    /// voronoi_robust_fpt.hpp:204-208.
    pub fn sqrt(&self) -> RobustFpt {
        // `re_ * 0.5 + ROUNDING_ERROR`: contracted.
        RobustFpt::new2(self.fpv.sqrt(), self.re.mul_add(0.5, ROUNDING_ERROR))
    }
}

/// `(std::max)(a, b)`: `a < b ? b : a`.
#[inline]
fn std_max(a: f64, b: f64) -> f64 {
    if a < b {
        b
    } else {
        a
    }
}

/// voronoi_robust_fpt.hpp:112-114.
impl Neg for RobustFpt {
    type Output = RobustFpt;
    fn neg(self) -> RobustFpt {
        RobustFpt::new2(-self.fpv, self.re)
    }
}

/// voronoi_robust_fpt.hpp:160-174 `operator+` (and `operator+=`, 116-130: the same arithmetic).
impl Add for RobustFpt {
    type Output = RobustFpt;
    fn add(self, that: RobustFpt) -> RobustFpt {
        let fpv = self.fpv + that.fpv;
        let re;
        if (!(self.fpv < 0.0) && !(that.fpv < 0.0)) || (!(self.fpv > 0.0) && !(that.fpv > 0.0)) {
            re = std_max(self.re, that.re) + ROUNDING_ERROR;
        } else {
            // `(this->fpv_ * this->re_ - that.fpv_ * that.re_) / fpv`: the left product is fused.
            let mut temp = self.fpv.mul_add(self.re, -(that.fpv * that.re)) / fpv;
            if temp < 0.0 {
                temp = -temp;
            }
            re = temp + ROUNDING_ERROR;
        }
        RobustFpt::new2(fpv, re)
    }
}

impl AddAssign for RobustFpt {
    fn add_assign(&mut self, that: RobustFpt) {
        *self = *self + that;
    }
}

/// voronoi_robust_fpt.hpp:176-190 `operator-` (and `operator-=`, 132-146).
impl Sub for RobustFpt {
    type Output = RobustFpt;
    fn sub(self, that: RobustFpt) -> RobustFpt {
        let fpv = self.fpv - that.fpv;
        let re;
        if (!(self.fpv < 0.0) && !(that.fpv > 0.0)) || (!(self.fpv > 0.0) && !(that.fpv < 0.0)) {
            re = std_max(self.re, that.re) + ROUNDING_ERROR;
        } else {
            // `(this->fpv_ * this->re_ + that.fpv_ * that.re_) / fpv`: the left product is fused.
            let mut temp = self.fpv.mul_add(self.re, that.fpv * that.re) / fpv;
            if temp < 0.0 {
                temp = -temp;
            }
            re = temp + ROUNDING_ERROR;
        }
        RobustFpt::new2(fpv, re)
    }
}

impl SubAssign for RobustFpt {
    fn sub_assign(&mut self, that: RobustFpt) {
        *self = *self - that;
    }
}

/// voronoi_robust_fpt.hpp:192-196 `operator*` (and `operator*=`, 148-152:
/// `re_ += that.re_ + 1`, which is `re_ + (that.re_ + 1)`).
impl Mul for RobustFpt {
    type Output = RobustFpt;
    fn mul(self, that: RobustFpt) -> RobustFpt {
        let fpv = self.fpv * that.fpv;
        let re = self.re + that.re + ROUNDING_ERROR;
        RobustFpt::new2(fpv, re)
    }
}

impl MulAssign for RobustFpt {
    fn mul_assign(&mut self, that: RobustFpt) {
        self.re += that.re + ROUNDING_ERROR;
        self.fpv *= that.fpv;
    }
}

/// voronoi_robust_fpt.hpp:198-202 `operator/` (and `operator/=`, 154-158).
impl Div for RobustFpt {
    type Output = RobustFpt;
    fn div(self, that: RobustFpt) -> RobustFpt {
        let fpv = self.fpv / that.fpv;
        let re = self.re + that.re + ROUNDING_ERROR;
        RobustFpt::new2(fpv, re)
    }
}

impl DivAssign for RobustFpt {
    fn div_assign(&mut self, that: RobustFpt) {
        self.re += that.re + ROUNDING_ERROR;
        self.fpv /= that.fpv;
    }
}

/// voronoi_robust_fpt.hpp:240-340 `robust_dif<robust_fpt<double>>`: a difference kept as
/// two non-negative sums.
///
/// The one-argument constructor (247-249) compares `value > 0`, which `robust_fpt` has no
/// operator for: it cannot be instantiated for this `T` and is not here.
#[derive(Clone, Copy, Debug, Default)]
pub struct RobustDif {
    positive_sum: RobustFpt,
    negative_sum: RobustFpt,
}

impl RobustDif {
    /// voronoi_robust_fpt.hpp:243-245.
    pub fn new() -> Self {
        RobustDif { positive_sum: RobustFpt::new(0.0), negative_sum: RobustFpt::new(0.0) }
    }

    /// voronoi_robust_fpt.hpp:251-253.
    pub fn new2(pos: RobustFpt, neg: RobustFpt) -> Self {
        RobustDif { positive_sum: pos, negative_sum: neg }
    }

    /// voronoi_robust_fpt.hpp:255-257.
    pub fn dif(&self) -> RobustFpt {
        self.positive_sum - self.negative_sum
    }

    /// voronoi_robust_fpt.hpp:259-261.
    pub fn pos(&self) -> RobustFpt {
        self.positive_sum
    }

    /// voronoi_robust_fpt.hpp:263-265 `neg()`. Named `neg_sum` here: `x.neg()` on a value
    /// would resolve to `std::ops::Neg::neg` (the `operator-()` of 267-269) before it.
    pub fn neg_sum(&self) -> RobustFpt {
        self.negative_sum
    }

    /// voronoi_robust_fpt.hpp:334-336.
    fn swap(&mut self) {
        std::mem::swap(&mut self.positive_sum, &mut self.negative_sum);
    }
}

/// voronoi_robust_fpt.hpp:267-269.
impl Neg for RobustDif {
    type Output = RobustDif;
    fn neg(self) -> RobustDif {
        RobustDif::new2(self.negative_sum, self.positive_sum)
    }
}

/// voronoi_robust_fpt.hpp:271-277.
impl AddAssign<RobustFpt> for RobustDif {
    fn add_assign(&mut self, val: RobustFpt) {
        if !val.has_neg_value() {
            self.positive_sum += val;
        } else {
            self.negative_sum -= val;
        }
    }
}

/// voronoi_robust_fpt.hpp:279-283.
impl AddAssign<RobustDif> for RobustDif {
    fn add_assign(&mut self, that: RobustDif) {
        self.positive_sum += that.positive_sum;
        self.negative_sum += that.negative_sum;
    }
}

/// voronoi_robust_fpt.hpp:285-291.
impl SubAssign<RobustFpt> for RobustDif {
    fn sub_assign(&mut self, val: RobustFpt) {
        if !val.has_neg_value() {
            self.negative_sum += val;
        } else {
            self.positive_sum -= val;
        }
    }
}

/// voronoi_robust_fpt.hpp:293-297.
impl SubAssign<RobustDif> for RobustDif {
    fn sub_assign(&mut self, that: RobustDif) {
        self.positive_sum += that.negative_sum;
        self.negative_sum += that.positive_sum;
    }
}

/// voronoi_robust_fpt.hpp:299-309.
impl MulAssign<RobustFpt> for RobustDif {
    fn mul_assign(&mut self, val: RobustFpt) {
        if !val.has_neg_value() {
            self.positive_sum *= val;
            self.negative_sum *= val;
        } else {
            self.positive_sum *= -val;
            self.negative_sum *= -val;
            self.swap();
        }
    }
}

/// voronoi_robust_fpt.hpp:311-319.
impl MulAssign<RobustDif> for RobustDif {
    fn mul_assign(&mut self, that: RobustDif) {
        let positive_sum = self.positive_sum * that.positive_sum + self.negative_sum * that.negative_sum;
        let negative_sum = self.positive_sum * that.negative_sum + self.negative_sum * that.positive_sum;
        self.positive_sum = positive_sum;
        self.negative_sum = negative_sum;
    }
}

/// voronoi_robust_fpt.hpp:321-331.
impl DivAssign<RobustFpt> for RobustDif {
    fn div_assign(&mut self, val: RobustFpt) {
        if !val.has_neg_value() {
            self.positive_sum /= val;
            self.negative_sum /= val;
        } else {
            self.positive_sum /= -val;
            self.negative_sum /= -val;
            self.swap();
        }
    }
}

/// voronoi_robust_fpt.hpp:342-346.
impl Add<RobustDif> for RobustDif {
    type Output = RobustDif;
    fn add(self, rhs: RobustDif) -> RobustDif {
        RobustDif::new2(self.pos() + rhs.pos(), self.neg_sum() + rhs.neg_sum())
    }
}

/// voronoi_robust_fpt.hpp:348-355.
impl Add<RobustFpt> for RobustDif {
    type Output = RobustDif;
    fn add(self, rhs: RobustFpt) -> RobustDif {
        if !rhs.has_neg_value() {
            RobustDif::new2(self.pos() + rhs, self.neg_sum())
        } else {
            RobustDif::new2(self.pos(), self.neg_sum() - rhs)
        }
    }
}

/// voronoi_robust_fpt.hpp:357-364.
impl Add<RobustDif> for RobustFpt {
    type Output = RobustDif;
    fn add(self, rhs: RobustDif) -> RobustDif {
        if !self.has_neg_value() {
            RobustDif::new2(self + rhs.pos(), rhs.neg_sum())
        } else {
            RobustDif::new2(rhs.pos(), rhs.neg_sum() - self)
        }
    }
}

/// voronoi_robust_fpt.hpp:366-370.
impl Sub<RobustDif> for RobustDif {
    type Output = RobustDif;
    fn sub(self, rhs: RobustDif) -> RobustDif {
        RobustDif::new2(self.pos() + rhs.neg_sum(), self.neg_sum() + rhs.pos())
    }
}

/// voronoi_robust_fpt.hpp:372-379.
impl Sub<RobustFpt> for RobustDif {
    type Output = RobustDif;
    fn sub(self, rhs: RobustFpt) -> RobustDif {
        if !rhs.has_neg_value() {
            RobustDif::new2(self.pos(), self.neg_sum() + rhs)
        } else {
            RobustDif::new2(self.pos() - rhs, self.neg_sum())
        }
    }
}

/// voronoi_robust_fpt.hpp:381-388.
impl Sub<RobustDif> for RobustFpt {
    type Output = RobustDif;
    fn sub(self, rhs: RobustDif) -> RobustDif {
        if !self.has_neg_value() {
            RobustDif::new2(self + rhs.neg_sum(), rhs.pos())
        } else {
            RobustDif::new2(rhs.neg_sum(), rhs.pos() - self)
        }
    }
}

/// voronoi_robust_fpt.hpp:390-396.
impl Mul<RobustDif> for RobustDif {
    type Output = RobustDif;
    fn mul(self, rhs: RobustDif) -> RobustDif {
        let res_pos = self.pos() * rhs.pos() + self.neg_sum() * rhs.neg_sum();
        let res_neg = self.pos() * rhs.neg_sum() + self.neg_sum() * rhs.pos();
        RobustDif::new2(res_pos, res_neg)
    }
}

/// voronoi_robust_fpt.hpp:398-405.
impl Mul<RobustFpt> for RobustDif {
    type Output = RobustDif;
    fn mul(self, val: RobustFpt) -> RobustDif {
        if !val.has_neg_value() {
            RobustDif::new2(self.pos() * val, self.neg_sum() * val)
        } else {
            RobustDif::new2(-self.neg_sum() * val, -self.pos() * val)
        }
    }
}

/// voronoi_robust_fpt.hpp:407-414.
impl Mul<RobustDif> for RobustFpt {
    type Output = RobustDif;
    fn mul(self, rhs: RobustDif) -> RobustDif {
        if !self.has_neg_value() {
            RobustDif::new2(self * rhs.pos(), self * rhs.neg_sum())
        } else {
            RobustDif::new2(-self * rhs.neg_sum(), -self * rhs.pos())
        }
    }
}

/// voronoi_robust_fpt.hpp:416-423.
impl Div<RobustFpt> for RobustDif {
    type Output = RobustDif;
    fn div(self, val: RobustFpt) -> RobustDif {
        if !val.has_neg_value() {
            RobustDif::new2(self.pos() / val, self.neg_sum() / val)
        } else {
            RobustDif::new2(-self.neg_sum() / val, -self.pos() / val)
        }
    }
}

/// The sign test every `eval` makes: both operands non-negative or both non-positive.
#[inline]
fn same_sign(a: &ExtendedExponentFpt, b: &ExtendedExponentFpt) -> bool {
    (!a.is_neg() && !b.is_neg()) || (!a.is_pos() && !b.is_pos())
}

/// voronoi_robust_fpt.hpp:428-496 `robust_sqrt_expr<extended_int<64>, efpt64, type_converter_efpt>`:
/// `sum(A[i] * sqrt(B[i]))` for up to four terms with a bounded relative error.
#[derive(Clone, Debug, Default)]
pub struct RobustSqrtExpr {
    t_a: [ExtendedInt; 5],
    t_b: [ExtendedInt; 5],
}

impl RobustSqrtExpr {
    /// voronoi_robust_fpt.hpp:440-444 `eval1`: `A[0] * sqrt(B[0])`.
    pub fn eval1(&mut self, a_: &[ExtendedInt], b_: &[ExtendedInt]) -> ExtendedExponentFpt {
        let a = to_efpt(&a_[0]);
        let b = to_efpt(&b_[0]);
        a * b.sqrt()
    }

    /// voronoi_robust_fpt.hpp:448-455 `eval2`.
    pub fn eval2(&mut self, a_: &[ExtendedInt], b_: &[ExtendedInt]) -> ExtendedExponentFpt {
        let a = self.eval1(a_, b_);
        let b = self.eval1(&a_[1..], &b_[1..]);
        if same_sign(&a, &b) {
            return a + b;
        }
        count(&SQRT_EXPR_CONJUGATE);
        to_efpt(&(a_[0] * a_[0] * b_[0] - a_[1] * a_[1] * b_[1])) / (a - b)
    }

    /// voronoi_robust_fpt.hpp:459-470 `eval3`.
    pub fn eval3(&mut self, a_: &[ExtendedInt], b_: &[ExtendedInt]) -> ExtendedExponentFpt {
        let a = self.eval2(a_, b_);
        let b = self.eval1(&a_[2..], &b_[2..]);
        if same_sign(&a, &b) {
            return a + b;
        }
        count(&SQRT_EXPR_CONJUGATE);
        // `A` may be `tA` itself (eval4 calls `eval3(tA, tB)`): elements 0..2 are read here,
        // 3 and 4 written, as in the C++.
        self.t_a[3] = a_[0] * a_[0] * b_[0] + a_[1] * a_[1] * b_[1] - a_[2] * a_[2] * b_[2];
        self.t_b[3] = ExtendedInt::from(1i32);
        self.t_a[4] = a_[0] * a_[1] * 2i32;
        self.t_b[4] = b_[0] * b_[1];
        let (ta, tb) = ([self.t_a[3], self.t_a[4]], [self.t_b[3], self.t_b[4]]);
        self.eval2(&ta, &tb) / (a - b)
    }

    /// voronoi_robust_fpt.hpp:476-490 `eval4`.
    pub fn eval4(&mut self, a_: &[ExtendedInt], b_: &[ExtendedInt]) -> ExtendedExponentFpt {
        let a = self.eval2(a_, b_);
        let b = self.eval2(&a_[2..], &b_[2..]);
        if same_sign(&a, &b) {
            return a + b;
        }
        count(&SQRT_EXPR_CONJUGATE);
        self.t_a[0] = a_[0] * a_[0] * b_[0] + a_[1] * a_[1] * b_[1] - a_[2] * a_[2] * b_[2] - a_[3] * a_[3] * b_[3];
        self.t_b[0] = ExtendedInt::from(1i32);
        self.t_a[1] = a_[0] * a_[1] * 2i32;
        self.t_b[1] = b_[0] * b_[1];
        self.t_a[2] = a_[2] * a_[3] * -2i32;
        self.t_b[2] = b_[2] * b_[3];
        let (ta, tb) = ([self.t_a[0], self.t_a[1], self.t_a[2]], [self.t_b[0], self.t_b[1], self.t_b[2]]);
        self.eval3(&ta, &tb) / (a - b)
    }
}

// ===========================================================================
// detail/voronoi_structures.hpp
// ===========================================================================

/// voronoi_structures.hpp:23-63 `point_2d<int32>`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Point2D {
    pub x: i32,
    pub y: i32,
}

impl Point2D {
    /// voronoi_structures.hpp:30-32.
    pub fn new(x: i32, y: i32) -> Self {
        Point2D { x, y }
    }
}

/// voronoi_structures.hpp:211-213 `site_event::Bits::IS_INVERSE`.
const IS_INVERSE: usize = 0x20;

/// voronoi_structures.hpp:86-220 `site_event<int32>`: a point site, or a segment site with
/// its direction. Equality (123-131) compares the two points only.
#[derive(Clone, Copy, Debug)]
pub struct SiteEvent {
    point0: Point2D,
    point1: Point2D,
    sorted_index: usize,
    initial_index: usize,
    flags: usize,
}

impl PartialEq for SiteEvent {
    /// voronoi_structures.hpp:123-126.
    fn eq(&self, that: &SiteEvent) -> bool {
        self.point0 == that.point0 && self.point1 == that.point1
    }
}

impl Default for SiteEvent {
    /// voronoi_structures.hpp:92-96 (`initial_index_` is left uninitialised there).
    fn default() -> Self {
        SiteEvent { point0: Point2D::new(0, 0), point1: Point2D::new(0, 0), sorted_index: 0, initial_index: 0, flags: 0 }
    }
}

impl SiteEvent {
    /// voronoi_structures.hpp:98-108: a point site.
    pub fn from_point(point: Point2D) -> Self {
        SiteEvent { point0: point, point1: point, sorted_index: 0, initial_index: 0, flags: 0 }
    }

    /// voronoi_structures.hpp:110-121: a segment site.
    pub fn from_points(point1: Point2D, point2: Point2D) -> Self {
        SiteEvent { point0: point1, point1: point2, sorted_index: 0, initial_index: 0, flags: 0 }
    }

    /// voronoi_structures.hpp:133-135.
    pub fn x(&self) -> i32 {
        self.point0.x
    }

    /// voronoi_structures.hpp:137-139.
    pub fn y(&self) -> i32 {
        self.point0.y
    }

    /// voronoi_structures.hpp:141-143.
    pub fn x0(&self) -> i32 {
        self.point0.x
    }

    /// voronoi_structures.hpp:145-147.
    pub fn y0(&self) -> i32 {
        self.point0.y
    }

    /// voronoi_structures.hpp:149-151.
    pub fn x1(&self) -> i32 {
        self.point1.x
    }

    /// voronoi_structures.hpp:153-155.
    pub fn y1(&self) -> i32 {
        self.point1.y
    }

    /// voronoi_structures.hpp:157-159.
    pub fn point0(&self) -> Point2D {
        self.point0
    }

    /// voronoi_structures.hpp:161-163.
    pub fn point1(&self) -> Point2D {
        self.point1
    }

    /// voronoi_structures.hpp:165-167.
    pub fn sorted_index(&self) -> usize {
        self.sorted_index
    }

    /// voronoi_structures.hpp:169-172.
    pub fn set_sorted_index(&mut self, index: usize) {
        self.sorted_index = index;
    }

    /// voronoi_structures.hpp:174-176.
    pub fn initial_index(&self) -> usize {
        self.initial_index
    }

    /// voronoi_structures.hpp:178-181.
    pub fn set_initial_index(&mut self, index: usize) {
        self.initial_index = index;
    }

    /// voronoi_structures.hpp:183-185.
    pub fn is_inverse(&self) -> bool {
        self.flags & IS_INVERSE != 0
    }

    /// voronoi_structures.hpp:187-191.
    pub fn inverse(&mut self) {
        std::mem::swap(&mut self.point0, &mut self.point1);
        self.flags ^= IS_INVERSE;
    }

    /// voronoi_structures.hpp:193-195.
    pub fn source_category(&self) -> usize {
        self.flags & SOURCE_CATEGORY_BITMASK
    }

    /// voronoi_structures.hpp:197-200.
    pub fn set_source_category(&mut self, source_category: usize) {
        self.flags |= source_category;
    }

    /// voronoi_structures.hpp:202-204.
    pub fn is_point(&self) -> bool {
        self.point0.x == self.point1.x && self.point0.y == self.point1.y
    }

    /// voronoi_structures.hpp:206-208.
    pub fn is_segment(&self) -> bool {
        self.point0.x != self.point1.x || self.point0.y != self.point1.y
    }
}

/// voronoi_structures.hpp:234-294 `circle_event<double>`.
#[derive(Clone, Copy, Debug)]
pub struct CircleEvent {
    center_x: f64,
    center_y: f64,
    lower_x: f64,
    is_active: bool,
}

impl Default for CircleEvent {
    /// voronoi_structures.hpp:239 (the coordinates are left uninitialised there).
    fn default() -> Self {
        CircleEvent { center_x: 0.0, center_y: 0.0, lower_x: 0.0, is_active: true }
    }
}

impl CircleEvent {
    /// voronoi_structures.hpp:241-247.
    pub fn new(c_x: f64, c_y: f64, lower_x: f64) -> Self {
        CircleEvent { center_x: c_x, center_y: c_y, lower_x, is_active: true }
    }

    /// voronoi_structures.hpp:249-251.
    pub fn x(&self) -> f64 {
        self.center_x
    }

    /// voronoi_structures.hpp:253-256.
    pub fn set_x(&mut self, center_x: f64) {
        self.center_x = center_x;
    }

    /// voronoi_structures.hpp:258-260.
    pub fn y(&self) -> f64 {
        self.center_y
    }

    /// voronoi_structures.hpp:262-265.
    pub fn set_y(&mut self, center_y: f64) {
        self.center_y = center_y;
    }

    /// voronoi_structures.hpp:267-269.
    pub fn lower_x(&self) -> f64 {
        self.lower_x
    }

    /// voronoi_structures.hpp:271-274.
    pub fn set_lower_x(&mut self, lower_x: f64) {
        self.lower_x = lower_x;
    }

    /// voronoi_structures.hpp:276-278.
    pub fn lower_y(&self) -> f64 {
        self.center_y
    }

    /// voronoi_structures.hpp:280-282.
    pub fn is_active(&self) -> bool {
        self.is_active
    }

    /// voronoi_structures.hpp:284-287.
    pub fn deactivate(&mut self) {
        self.is_active = false;
    }
}

// --- libc++ <__algorithm/push_heap.h>, pop_heap.h, sift_down.h ---------------
// `std::priority_queue::push` is `c.push_back(v); std::push_heap(...)`, `pop` is
// `std::pop_heap(...); c.pop_back()`. Elements that compare equal leave the queue in the
// order the heap holds them, so the sift algorithms are libc++'s, step for step.

/// push_heap.h `__sift_up(first, last, comp, len)`; `last` is an index one past the element
/// to sift.
fn libcxx_sift_up<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], last: usize, comp: &mut F, len: usize) {
    if len > 1 {
        let mut len = (len - 2) / 2;
        let mut ptr = len;
        let mut last = last - 1;
        if comp(&v[ptr], &v[last]) {
            let t = v[last];
            loop {
                v[last] = v[ptr];
                last = ptr;
                if len == 0 {
                    break;
                }
                len = (len - 1) / 2;
                ptr = len;
                if !comp(&v[ptr], &t) {
                    break;
                }
            }
            v[last] = t;
        }
    }
}

/// sift_down.h `__floyd_sift_down(first, comp, len)`: returns the index of the hole.
fn libcxx_floyd_sift_down<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], comp: &mut F, len: usize) -> usize {
    let mut hole = 0usize;
    let mut child_i = 0usize;
    let mut child = 0usize;
    loop {
        child_i += child + 1;
        child = 2 * child + 1;
        if child + 1 < len && comp(&v[child_i], &v[child_i + 1]) {
            child_i += 1;
            child += 1;
        }
        v[hole] = v[child_i];
        hole = child_i;
        if child > (len - 2) / 2 {
            return hole;
        }
    }
}

/// push_heap.h `std::push_heap` over the whole of `v`.
fn libcxx_push_heap<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], comp: &mut F) {
    let len = v.len();
    libcxx_sift_up(v, len, comp, len);
}

/// pop_heap.h `__pop_heap` over the whole of `v`: the top goes to the last element.
fn libcxx_pop_heap<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], comp: &mut F) {
    let len = v.len();
    if len > 1 {
        let top = v[0];
        let mut hole = libcxx_floyd_sift_down(v, comp, len);
        let last = len - 1;
        if hole == last {
            v[hole] = top;
        } else {
            v[hole] = v[last];
            hole += 1;
            v[last] = top;
            libcxx_sift_up(v, hole, comp, hole);
        }
    }
}

/// The element of the circle event queue: `std::pair<circle_event_type, beach_line_iterator>`
/// (voronoi_builder.hpp:159).
pub type EventType = (CircleEvent, usize);

/// voronoi_structures.hpp:303-353 `ordered_queue<event_type, event_comparison_type>`: a
/// `std::list` of the events and a `std::priority_queue` of iterators into it.
///
/// The list is a slab here: an iterator is the slot index, `erase` frees the slot. Only the
/// identity of an element matters to the builder, never the list order.
#[derive(Clone, Debug, Default)]
pub struct OrderedQueue {
    /// `c_`: the heap of list iterators.
    c: Vec<usize>,
    /// `c_list_`.
    c_list: Vec<EventType>,
    free: Vec<usize>,
}

impl OrderedQueue {
    /// voronoi_structures.hpp:337-343 `comparison`, with `event_comparison_type`
    /// (voronoi_builder.hpp:160-165): `predicate(rhs.first, lhs.first)`.
    fn comparison(list: &[EventType], it1: usize, it2: usize) -> bool {
        event_comparison_cc(&list[it2].0, &list[it1].0)
    }

    /// voronoi_structures.hpp:308-310.
    pub fn empty(&self) -> bool {
        self.c.is_empty()
    }

    /// voronoi_structures.hpp:312-314 `top()`.
    pub fn top(&self) -> &EventType {
        &self.c_list[self.c[0]]
    }

    /// The list iterator of `top()`.
    pub fn top_iterator(&self) -> usize {
        self.c[0]
    }

    /// The element a list iterator points to.
    pub fn get_mut(&mut self, it: usize) -> &mut EventType {
        &mut self.c_list[it]
    }

    /// voronoi_structures.hpp:316-320.
    pub fn pop(&mut self) {
        let it = self.c[0];
        let list = &self.c_list;
        libcxx_pop_heap(&mut self.c, &mut |a: &usize, b: &usize| Self::comparison(list, *a, *b));
        self.c.pop();
        self.free.push(it);
    }

    /// voronoi_structures.hpp:322-326: returns the list iterator of the new element.
    pub fn push(&mut self, e: EventType) -> usize {
        let it = match self.free.pop() {
            Some(i) => {
                self.c_list[i] = e;
                i
            }
            None => {
                self.c_list.push(e);
                self.c_list.len() - 1
            }
        };
        self.c.push(it);
        let list = &self.c_list;
        libcxx_push_heap(&mut self.c, &mut |a: &usize, b: &usize| Self::comparison(list, *a, *b));
        it
    }

    /// voronoi_structures.hpp:328-332.
    pub fn clear(&mut self) {
        self.c.clear();
        self.c_list.clear();
        self.free.clear();
    }
}

/// voronoi_structures.hpp:364-411 `beach_line_node_key<site_event>`: the bisector of two
/// neighbouring arcs.
#[derive(Clone, Copy, Debug)]
pub struct BeachLineNodeKey {
    left_site: SiteEvent,
    right_site: SiteEvent,
}

impl BeachLineNodeKey {
    /// voronoi_structures.hpp:371-373.
    pub fn new(new_site: SiteEvent) -> Self {
        BeachLineNodeKey { left_site: new_site, right_site: new_site }
    }

    /// voronoi_structures.hpp:377-380.
    pub fn new2(left_site: SiteEvent, right_site: SiteEvent) -> Self {
        BeachLineNodeKey { left_site, right_site }
    }

    /// voronoi_structures.hpp:382-388.
    pub fn left_site(&self) -> &SiteEvent {
        &self.left_site
    }

    /// voronoi_structures.hpp:395-401.
    pub fn right_site(&self) -> &SiteEvent {
        &self.right_site
    }
}

/// voronoi_structures.hpp:417-445 `beach_line_node_data<void, circle_event>`: the half-edge
/// of the bisector (an index into the diagram's edges) and the circle event it is the right
/// bisector of (an iterator into the event queue's list).
#[derive(Clone, Copy, Debug)]
pub struct BeachLineNodeData {
    circle_event: Option<usize>,
    edge: Option<usize>,
}

impl BeachLineNodeData {
    /// voronoi_structures.hpp:420-422.
    pub fn new(new_edge: Option<usize>) -> Self {
        BeachLineNodeData { circle_event: None, edge: new_edge }
    }
}

// ===========================================================================
// detail/voronoi_predicates.hpp
// ===========================================================================

/// voronoi_predicates.hpp:36-39.
const ULPS: u32 = 64;

/// voronoi_predicates.hpp:41-44 `is_vertical(point1, point2)`.
fn is_vertical_points(point1: Point2D, point2: Point2D) -> bool {
    point1.x == point2.x
}

/// voronoi_predicates.hpp:46-49 `is_vertical(site)`.
fn is_vertical(site: &SiteEvent) -> bool {
    is_vertical_points(site.point0, site.point1)
}

/// voronoi_predicates.hpp:54-78 `robust_cross_product`: `a1 * b2 - b1 * a2` with a relative
/// error of one epsilon. The products are of unsigned 64-bit values and wrap as they do there.
pub fn robust_cross_product(a1_: i64, b1_: i64, a2_: i64, b2_: i64) -> f64 {
    let a1 = (if a1_ < 0 { a1_.wrapping_neg() } else { a1_ }) as u64;
    let b1 = (if b1_ < 0 { b1_.wrapping_neg() } else { b1_ }) as u64;
    let a2 = (if a2_ < 0 { a2_.wrapping_neg() } else { a2_ }) as u64;
    let b2 = (if b2_ < 0 { b2_.wrapping_neg() } else { b2_ }) as u64;

    let l = a1.wrapping_mul(b2);
    let r = b1.wrapping_mul(a2);

    if (a1_ < 0) ^ (b2_ < 0) {
        if (a2_ < 0) ^ (b1_ < 0) {
            if l > r {
                -((l - r) as f64)
            } else {
                (r - l) as f64
            }
        } else {
            -(l.wrapping_add(r) as f64)
        }
    } else {
        if (a2_ < 0) ^ (b1_ < 0) {
            l.wrapping_add(r) as f64
        } else {
            if l < r {
                -((r - l) as f64)
            } else {
                (l - r) as f64
            }
        }
    }
}

/// voronoi_predicates.hpp:83-87 `orientation_test::Orientation`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Orientation {
    Right = -1,
    Collinear = 0,
    Left = 1,
}

/// voronoi_predicates.hpp:91-95 `orientation_test::eval(T value)`.
fn ot_eval_value(value: f64) -> Orientation {
    if value == 0.0 {
        return Orientation::Collinear;
    }
    if value < 0.0 {
        Orientation::Right
    } else {
        Orientation::Left
    }
}

/// voronoi_predicates.hpp:97-102 `orientation_test::eval(dif_x1, dif_y1, dif_x2, dif_y2)`.
fn ot_eval_difs(dif_x1: i64, dif_y1: i64, dif_x2: i64, dif_y2: i64) -> Orientation {
    ot_eval_value(robust_cross_product(dif_x1, dif_y1, dif_x2, dif_y2))
}

/// voronoi_predicates.hpp:104-117 `orientation_test::eval(point1, point2, point3)`.
fn ot_eval_points(point1: Point2D, point2: Point2D, point3: Point2D) -> Orientation {
    let dx1 = point1.x as i64 - point2.x as i64;
    let dx2 = point2.x as i64 - point3.x as i64;
    let dy1 = point1.y as i64 - point2.y as i64;
    let dy2 = point2.y as i64 - point3.y as i64;
    ot_eval_value(robust_cross_product(dx1, dy1, dx2, dy2))
}

/// voronoi_predicates.hpp:121-131 `point_comparison_predicate`.
fn point_comparison(lhs: Point2D, rhs: Point2D) -> bool {
    if lhs.x == rhs.x {
        return lhs.y < rhs.y;
    }
    lhs.x < rhs.x
}

/// voronoi_predicates.hpp:139-160 `event_comparison_predicate(site, site)`.
pub fn event_comparison_ss(lhs: &SiteEvent, rhs: &SiteEvent) -> bool {
    if lhs.x0() != rhs.x0() {
        return lhs.x0() < rhs.x0();
    }
    if !lhs.is_segment() {
        if !rhs.is_segment() {
            return lhs.y0() < rhs.y0();
        }
        if is_vertical(rhs) {
            return lhs.y0() <= rhs.y0();
        }
        true
    } else {
        if is_vertical(rhs) {
            if is_vertical(lhs) {
                return lhs.y0() < rhs.y0();
            }
            return false;
        }
        if is_vertical(lhs) {
            return true;
        }
        if lhs.y0() != rhs.y0() {
            return lhs.y0() < rhs.y0();
        }
        ot_eval_points(lhs.point1(), lhs.point0(), rhs.point1()) == Orientation::Left
    }
}

/// voronoi_predicates.hpp:162-166 `event_comparison_predicate(site, circle)`.
fn event_comparison_sc(lhs: &SiteEvent, rhs: &CircleEvent) -> bool {
    ulp_comparison(lhs.x0() as f64, rhs.lower_x(), ULPS) == UlpResult::Less
}

/// voronoi_predicates.hpp:168-172 `event_comparison_predicate(circle, site)`.
#[allow(dead_code)]
fn event_comparison_cs(lhs: &CircleEvent, rhs: &SiteEvent) -> bool {
    ulp_comparison(lhs.lower_x(), rhs.x0() as f64, ULPS) == UlpResult::Less
}

/// voronoi_predicates.hpp:174-179 `event_comparison_predicate(circle, circle)`.
fn event_comparison_cc(lhs: &CircleEvent, rhs: &CircleEvent) -> bool {
    if lhs.lower_x() != rhs.lower_x() {
        return lhs.lower_x() < rhs.lower_x();
    }
    lhs.y() < rhs.y()
}

/// voronoi_predicates.hpp:216-220 `distance_predicate::kPredicateResult`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KPredicateResult {
    Less = -1,
    Undefined = 0,
    More = 1,
}

/// voronoi_predicates.hpp:195-211 `distance_predicate::operator()`: whether the horizontal
/// line through `new_point` meets the right arc first.
pub fn distance_predicate(left_site: &SiteEvent, right_site: &SiteEvent, new_point: Point2D) -> bool {
    if !left_site.is_segment() {
        if !right_site.is_segment() {
            distance_predicate_pp(left_site, right_site, new_point)
        } else {
            distance_predicate_ps(left_site, right_site, new_point, false)
        }
    } else {
        if !right_site.is_segment() {
            distance_predicate_ps(right_site, left_site, new_point, true)
        } else {
            distance_predicate_ss(left_site, right_site, new_point)
        }
    }
}

/// voronoi_predicates.hpp:226-248 `distance_predicate::pp`.
fn distance_predicate_pp(left_site: &SiteEvent, right_site: &SiteEvent, new_point: Point2D) -> bool {
    let left_point = left_site.point0();
    let right_point = right_site.point0();
    if left_point.x > right_point.x {
        if new_point.y <= left_point.y {
            return false;
        }
    } else if left_point.x < right_point.x {
        if new_point.y >= right_point.y {
            return true;
        }
    } else {
        return (left_point.y as i64 + right_point.y as i64) < new_point.y as i64 * 2;
    }

    let dist1 = find_distance_to_point_arc(left_site, new_point);
    let dist2 = find_distance_to_point_arc(right_site, new_point);

    dist1 < dist2
}

/// voronoi_predicates.hpp:250-263 `distance_predicate::ps`.
fn distance_predicate_ps(left_site: &SiteEvent, right_site: &SiteEvent, new_point: Point2D, reverse_order: bool) -> bool {
    let fast_res = fast_ps(left_site, right_site, new_point, reverse_order);
    if fast_res != KPredicateResult::Undefined {
        return fast_res == KPredicateResult::Less;
    }
    count(&FAST_PS_UNDEFINED);

    let dist1 = find_distance_to_point_arc(left_site, new_point);
    let dist2 = find_distance_to_segment_arc(right_site, new_point);

    reverse_order ^ (dist1 < dist2)
}

/// voronoi_predicates.hpp:265-279 `distance_predicate::ss`.
fn distance_predicate_ss(left_site: &SiteEvent, right_site: &SiteEvent, new_point: Point2D) -> bool {
    // Handle temporary segment sites.
    if left_site.sorted_index() == right_site.sorted_index() {
        return ot_eval_points(left_site.point0(), left_site.point1(), new_point) == Orientation::Left;
    }

    let dist1 = find_distance_to_segment_arc(left_site, new_point);
    let dist2 = find_distance_to_segment_arc(right_site, new_point);

    dist1 < dist2
}

/// voronoi_predicates.hpp:281-287 `find_distance_to_point_arc`.
pub fn find_distance_to_point_arc(site: &SiteEvent, point: Point2D) -> f64 {
    let dx = site.x() as f64 - point.x as f64;
    let dy = site.y() as f64 - point.y as f64;
    // `(dx * dx + dy * dy) / (2.0 * dx)`: the left product is fused.
    dx.mul_add(dx, dy * dy) / (2.0 * dx)
}

/// voronoi_predicates.hpp:289-316 `find_distance_to_segment_arc`.
pub fn find_distance_to_segment_arc(site: &SiteEvent, point: Point2D) -> f64 {
    if is_vertical(site) {
        (site.x() as f64 - point.x as f64) * 0.5
    } else {
        let segment0 = site.point0();
        let segment1 = site.point1();
        let a1 = segment1.x as f64 - segment0.x as f64;
        let b1 = segment1.y as f64 - segment0.y as f64;
        // `get_sqrt(a1 * a1 + b1 * b1)`: the left product is fused.
        let mut k = a1.mul_add(a1, b1 * b1).sqrt();
        // Avoid subtraction while computing k.
        if !(b1 < 0.0) {
            k = 1.0 / (b1 + k);
        } else {
            k = (k - b1) / (a1 * a1);
        }
        k * robust_cross_product(
            segment1.x as i64 - segment0.x as i64,
            segment1.y as i64 - segment0.y as i64,
            point.x as i64 - segment0.x as i64,
            point.y as i64 - segment0.y as i64,
        )
    }
}

/// voronoi_predicates.hpp:318-366 `fast_ps`.
pub fn fast_ps(left_site: &SiteEvent, right_site: &SiteEvent, new_point: Point2D, reverse_order: bool) -> KPredicateResult {
    let site_point = left_site.point0();
    let segment_start = right_site.point0();
    let segment_end = right_site.point1();

    if ot_eval_points(segment_start, segment_end, new_point) != Orientation::Right {
        return if !right_site.is_inverse() { KPredicateResult::Less } else { KPredicateResult::More };
    }

    let dif_x = new_point.x as f64 - site_point.x as f64;
    let dif_y = new_point.y as f64 - site_point.y as f64;
    let a = segment_end.x as f64 - segment_start.x as f64;
    let b = segment_end.y as f64 - segment_start.y as f64;

    if is_vertical(right_site) {
        if new_point.y < site_point.y && !reverse_order {
            return KPredicateResult::More;
        } else if new_point.y > site_point.y && reverse_order {
            return KPredicateResult::Less;
        }
        return KPredicateResult::Undefined;
    } else {
        let orientation = ot_eval_difs(
            segment_end.x as i64 - segment_start.x as i64,
            segment_end.y as i64 - segment_start.y as i64,
            new_point.x as i64 - site_point.x as i64,
            new_point.y as i64 - site_point.y as i64,
        );
        if orientation == Orientation::Left {
            if !right_site.is_inverse() {
                return if reverse_order { KPredicateResult::Less } else { KPredicateResult::Undefined };
            }
            return if reverse_order { KPredicateResult::Undefined } else { KPredicateResult::More };
        }
    }

    let fast_left_expr = a * (dif_y + dif_x) * (dif_y - dif_x);
    let fast_right_expr = (2.0 * b) * dif_x * dif_y;
    let expr_cmp = ulp_comparison(fast_left_expr, fast_right_expr, 4);
    if expr_cmp != UlpResult::Equal {
        if (expr_cmp == UlpResult::More) ^ reverse_order {
            return if reverse_order { KPredicateResult::Less } else { KPredicateResult::More };
        }
        return KPredicateResult::Undefined;
    }
    KPredicateResult::Undefined
}

/// voronoi_predicates.hpp:389-422 `node_comparison_predicate::operator()`: the order of the
/// beach line. Only meaningful when one of the nodes holds the newest site.
pub fn node_comparison_predicate(node1: &BeachLineNodeKey, node2: &BeachLineNodeKey) -> bool {
    // Get x coordinate of the rightmost site from both nodes.
    let site1 = get_comparison_site(node1);
    let site2 = get_comparison_site(node2);
    let point1 = get_comparison_point(site1);
    let point2 = get_comparison_point(site2);

    if point1.x < point2.x {
        // The second node contains a new site.
        distance_predicate(&node1.left_site, &node1.right_site, point2)
    } else if point1.x > point2.x {
        // The first node contains a new site.
        !distance_predicate(&node2.left_site, &node2.right_site, point1)
    } else {
        // This checks were evaluated experimentally.
        if site1.sorted_index() == site2.sorted_index() {
            // Both nodes are new (inserted during same site event processing).
            get_comparison_y(node1, true) < get_comparison_y(node2, true)
        } else if site1.sorted_index() < site2.sorted_index() {
            let y1 = get_comparison_y(node1, false);
            let y2 = get_comparison_y(node2, true);
            if y1.0 != y2.0 {
                return y1.0 < y2.0;
            }
            if !site1.is_segment() {
                y1.1 < 0
            } else {
                false
            }
        } else {
            let y1 = get_comparison_y(node1, true);
            let y2 = get_comparison_y(node2, false);
            if y1.0 != y2.0 {
                return y1.0 < y2.0;
            }
            if !site2.is_segment() {
                y2.1 > 0
            } else {
                true
            }
        }
    }
}

/// voronoi_predicates.hpp:426-431 `get_comparison_site`: the newer site of the node.
fn get_comparison_site(node: &BeachLineNodeKey) -> &SiteEvent {
    if node.left_site.sorted_index() > node.right_site.sorted_index() {
        return &node.left_site;
    }
    &node.right_site
}

/// voronoi_predicates.hpp:433-436 `get_comparison_point`.
fn get_comparison_point(site: &SiteEvent) -> Point2D {
    if point_comparison(site.point0(), site.point1()) {
        site.point0()
    } else {
        site.point1()
    }
}

/// voronoi_predicates.hpp:439-454 `get_comparison_y`: the y of the newer site and its
/// direction. The pair compares as `std::pair` does (lexicographically).
fn get_comparison_y(node: &BeachLineNodeKey, is_new_node: bool) -> (i32, i32) {
    if node.left_site.sorted_index() == node.right_site.sorted_index() {
        return (node.left_site.y0(), 0);
    }
    if node.left_site.sorted_index() > node.right_site.sorted_index() {
        if !is_new_node && node.left_site.is_segment() && is_vertical(&node.left_site) {
            return (node.left_site.y0(), 1);
        }
        return (node.left_site.y1(), 1);
    }
    (node.right_site.y0(), -1)
}

/// voronoi_predicates.hpp:466-472 `circle_existence_predicate::ppp`.
fn circle_existence_ppp(site1: &SiteEvent, site2: &SiteEvent, site3: &SiteEvent) -> bool {
    ot_eval_points(site1.point0(), site2.point0(), site3.point0()) == Orientation::Right
}

/// voronoi_predicates.hpp:474-497 `circle_existence_predicate::pps`.
fn circle_existence_pps(site1: &SiteEvent, site2: &SiteEvent, site3: &SiteEvent, segment_index: i32) -> bool {
    if segment_index != 2 {
        let orient1 = ot_eval_points(site1.point0(), site2.point0(), site3.point0());
        let orient2 = ot_eval_points(site1.point0(), site2.point0(), site3.point1());
        if segment_index == 1 && site1.x0() >= site2.x0() {
            if orient1 != Orientation::Right {
                return false;
            }
        } else if segment_index == 3 && site2.x0() >= site1.x0() {
            if orient2 != Orientation::Right {
                return false;
            }
        } else if orient1 != Orientation::Right && orient2 != Orientation::Right {
            return false;
        }
    } else {
        return (site3.point0() != site1.point0()) || (site3.point1() != site2.point0());
    }
    true
}

/// voronoi_predicates.hpp:499-516 `circle_existence_predicate::pss`.
fn circle_existence_pss(site1: &SiteEvent, site2: &SiteEvent, site3: &SiteEvent, point_index: i32) -> bool {
    if site2.sorted_index() == site3.sorted_index() {
        return false;
    }
    if point_index == 2 {
        if !site2.is_inverse() && site3.is_inverse() {
            return false;
        }
        if site2.is_inverse() == site3.is_inverse()
            && ot_eval_points(site2.point0(), site1.point0(), site3.point1()) != Orientation::Right
        {
            return false;
        }
    }
    true
}

/// voronoi_predicates.hpp:518-523 `circle_existence_predicate::sss`.
fn circle_existence_sss(site1: &SiteEvent, site2: &SiteEvent, site3: &SiteEvent) -> bool {
    (site1.sorted_index() != site2.sorted_index()) && (site2.sorted_index() != site3.sorted_index())
}

/// `big_int_type(int_x2_type)`.
#[inline]
fn bi(v: i64) -> ExtendedInt {
    ExtendedInt::from(v)
}

/// `is_neg(big_int)`, `is_zero(big_int)` (voronoi_ctypes.hpp:582-595) are the methods.
///
/// voronoi_predicates.hpp:526-982 `mp_circle_formation_functor`: the circle of three sites
/// with multi-precision integers, used for the coordinates the lazy functor could not bound.
#[derive(Clone, Debug, Default)]
pub struct MpCircleFormationFunctor {
    sqrt_expr: RobustSqrtExpr,
}

impl MpCircleFormationFunctor {
    /// voronoi_predicates.hpp:535-600 `ppp`.
    pub fn ppp(
        &mut self,
        site1: &SiteEvent,
        site2: &SiteEvent,
        site3: &SiteEvent,
        circle: &mut CircleEvent,
        recompute_c_x: bool,
        recompute_c_y: bool,
        recompute_lower_x: bool,
    ) {
        count(&EXACT_PPP);
        let (x1, y1) = (site1.x() as i64, site1.y() as i64);
        let (x2, y2) = (site2.x() as i64, site2.y() as i64);
        let (x3, y3) = (site3.x() as i64, site3.y() as i64);
        let dif_x = [bi(x1 - x2), bi(x2 - x3), bi(x1 - x3)];
        let dif_y = [bi(y1 - y2), bi(y2 - y3), bi(y1 - y3)];
        let sum_x = [bi(x1 + x2), bi(x2 + x3)];
        let sum_y = [bi(y1 + y2), bi(y2 + y3)];
        let inv_denom = 0.5 / to_fpt_int(&(dif_x[0] * dif_y[1] - dif_x[1] * dif_y[0]));
        let numer1 = dif_x[0] * sum_x[0] + dif_y[0] * sum_y[0];
        let numer2 = dif_x[1] * sum_x[1] + dif_y[1] * sum_y[1];

        if recompute_c_x || recompute_lower_x {
            let c_x = numer1 * dif_y[1] - numer2 * dif_y[0];
            circle.set_x(to_fpt_int(&c_x) * inv_denom);

            if recompute_lower_x {
                // Evaluate radius of the circle.
                let sqr_r = (dif_x[0] * dif_x[0] + dif_y[0] * dif_y[0])
                    * (dif_x[1] * dif_x[1] + dif_y[1] * dif_y[1])
                    * (dif_x[2] * dif_x[2] + dif_y[2] * dif_y[2]);
                let r = to_fpt_int(&sqr_r).sqrt();

                // If c_x >= 0 then lower_x = c_x + r,
                // else lower_x = (c_x * c_x - r * r) / (c_x - r).
                // To guarantee epsilon relative error.
                if !(circle.x() < 0.0) {
                    if !(inv_denom < 0.0) {
                        // `circle.x() + r * inv_denom`: contracted.
                        circle.set_lower_x(r.mul_add(inv_denom, circle.x()));
                    } else {
                        // `circle.x() - r * inv_denom`: contracted.
                        circle.set_lower_x((-r).mul_add(inv_denom, circle.x()));
                    }
                } else {
                    let numer = c_x * c_x - sqr_r;
                    let lower_x = to_fpt_int(&numer) * inv_denom / (to_fpt_int(&c_x) + r);
                    circle.set_lower_x(lower_x);
                }
            }
        }

        if recompute_c_y {
            let c_y = numer2 * dif_x[0] - numer1 * dif_x[1];
            circle.set_y(to_fpt_int(&c_y) * inv_denom);
        }
    }

    /// voronoi_predicates.hpp:603-696 `pps`.
    pub fn pps(
        &mut self,
        site1: &SiteEvent,
        site2: &SiteEvent,
        site3: &SiteEvent,
        segment_index: i32,
        c_event: &mut CircleEvent,
        recompute_c_x: bool,
        recompute_c_y: bool,
        recompute_lower_x: bool,
    ) {
        count(&EXACT_PPS);
        let mut c_a = [ExtendedInt::default(); 4];
        let mut c_b = [ExtendedInt::default(); 4];
        let line_a = bi(site3.y1() as i64 - site3.y0() as i64);
        let line_b = bi(site3.x0() as i64 - site3.x1() as i64);
        let segm_len = line_a * line_a + line_b * line_b;
        let vec_x = bi(site2.y() as i64 - site1.y() as i64);
        let vec_y = bi(site1.x() as i64 - site2.x() as i64);
        let sum_x = bi(site1.x() as i64 + site2.x() as i64);
        let sum_y = bi(site1.y() as i64 + site2.y() as i64);
        let teta = line_a * vec_x + line_b * vec_y;
        let mut denom = vec_x * line_b - vec_y * line_a;

        let mut dif0 = bi(site3.y1() as i64 - site1.y() as i64);
        let mut dif1 = bi(site1.x() as i64 - site3.x1() as i64);
        let a = line_a * dif1 - line_b * dif0;
        dif0 = bi(site3.y1() as i64 - site2.y() as i64);
        dif1 = bi(site2.x() as i64 - site3.x1() as i64);
        let b = line_a * dif1 - line_b * dif0;
        let sum_ab = a + b;

        if denom.is_zero() {
            let numer = teta * teta - sum_ab * sum_ab;
            denom = teta * sum_ab;
            c_a[0] = denom * sum_x * 2i32 + numer * vec_x;
            c_b[0] = segm_len;
            c_a[1] = denom * sum_ab * 2i32 + numer * teta;
            c_b[1] = ExtendedInt::from(1i32);
            c_a[2] = denom * sum_y * 2i32 + numer * vec_y;
            let inv_denom = 1.0 / to_fpt_int(&denom);
            if recompute_c_x {
                c_event.set_x(0.25 * to_fpt_int(&c_a[0]) * inv_denom);
            }
            if recompute_c_y {
                c_event.set_y(0.25 * to_fpt_int(&c_a[2]) * inv_denom);
            }
            if recompute_lower_x {
                c_event.set_lower_x(
                    0.25 * to_fpt_efpt(&self.sqrt_expr.eval2(&c_a, &c_b)) * inv_denom / to_fpt_int(&segm_len).sqrt(),
                );
            }
            return;
        }

        let det = (teta * teta + denom * denom) * a * b * 4i32;
        let mut inv_denom_sqr = 1.0 / to_fpt_int(&denom);
        inv_denom_sqr *= inv_denom_sqr;

        if recompute_c_x || recompute_lower_x {
            c_a[0] = sum_x * denom * denom + teta * sum_ab * vec_x;
            c_b[0] = ExtendedInt::from(1i32);
            c_a[1] = if segment_index == 2 { -vec_x } else { vec_x };
            c_b[1] = det;
            if recompute_c_x {
                c_event.set_x(0.5 * to_fpt_efpt(&self.sqrt_expr.eval2(&c_a, &c_b)) * inv_denom_sqr);
            }
        }

        if recompute_c_y || recompute_lower_x {
            c_a[2] = sum_y * denom * denom + teta * sum_ab * vec_y;
            c_b[2] = ExtendedInt::from(1i32);
            c_a[3] = if segment_index == 2 { -vec_y } else { vec_y };
            c_b[3] = det;
            if recompute_c_y {
                c_event.set_y(0.5 * to_fpt_efpt(&self.sqrt_expr.eval2(&c_a[2..], &c_b[2..])) * inv_denom_sqr);
            }
        }

        if recompute_lower_x {
            c_b[0] = c_b[0] * segm_len;
            c_b[1] = c_b[1] * segm_len;
            c_a[2] = sum_ab * (denom * denom + teta * teta);
            c_b[2] = ExtendedInt::from(1i32);
            c_a[3] = if segment_index == 2 { -teta } else { teta };
            c_b[3] = det;
            c_event.set_lower_x(
                0.5 * to_fpt_efpt(&self.sqrt_expr.eval4(&c_a, &c_b)) * inv_denom_sqr / to_fpt_int(&segm_len).sqrt(),
            );
        }
    }

    /// voronoi_predicates.hpp:699-829 `pss`.
    pub fn pss(
        &mut self,
        site1: &SiteEvent,
        site2: &SiteEvent,
        site3: &SiteEvent,
        point_index: i32,
        c_event: &mut CircleEvent,
        recompute_c_x: bool,
        recompute_c_y: bool,
        recompute_lower_x: bool,
    ) {
        count(&EXACT_PSS);
        let mut a = [ExtendedInt::default(); 2];
        let mut b = [ExtendedInt::default(); 2];
        let mut c = [ExtendedInt::default(); 2];
        let mut c_a = [ExtendedInt::default(); 4];
        let mut c_b = [ExtendedInt::default(); 4];
        let segm_start1 = site2.point1();
        let segm_end1 = site2.point0();
        let segm_start2 = site3.point0();
        let segm_end2 = site3.point1();
        a[0] = bi(segm_end1.x as i64 - segm_start1.x as i64);
        b[0] = bi(segm_end1.y as i64 - segm_start1.y as i64);
        a[1] = bi(segm_end2.x as i64 - segm_start2.x as i64);
        b[1] = bi(segm_end2.y as i64 - segm_start2.y as i64);
        let orientation = a[1] * b[0] - a[0] * b[1];
        if orientation.is_zero() {
            let denom = 2.0 * to_fpt_int(&(a[0] * a[0] + b[0] * b[0]));
            c[0] = b[0] * (segm_start2.x as i64 - segm_start1.x as i64)
                - a[0] * (segm_start2.y as i64 - segm_start1.y as i64);
            let dx = a[0] * (site1.y() as i64 - segm_start1.y as i64) - b[0] * (site1.x() as i64 - segm_start1.x as i64);
            let dy = b[0] * (site1.x() as i64 - segm_start2.x as i64) - a[0] * (site1.y() as i64 - segm_start2.y as i64);
            c_b[0] = dx * dy;
            c_b[1] = ExtendedInt::from(1i32);

            if recompute_c_y {
                c_a[0] = b[0] * (if point_index == 2 { 2i32 } else { -2i32 });
                c_a[1] = a[0] * a[0] * (segm_start1.y as i64 + segm_start2.y as i64)
                    - a[0] * b[0] * (segm_start1.x as i64 + segm_start2.x as i64 - site1.x() as i64 * 2)
                    + b[0] * b[0] * (site1.y() as i64 * 2);
                let c_y = to_fpt_efpt(&self.sqrt_expr.eval2(&c_a, &c_b));
                c_event.set_y(c_y / denom);
            }

            if recompute_c_x || recompute_lower_x {
                c_a[0] = a[0] * (if point_index == 2 { 2i32 } else { -2i32 });
                c_a[1] = b[0] * b[0] * (segm_start1.x as i64 + segm_start2.x as i64)
                    - a[0] * b[0] * (segm_start1.y as i64 + segm_start2.y as i64 - site1.y() as i64 * 2)
                    + a[0] * a[0] * (site1.x() as i64 * 2);

                if recompute_c_x {
                    let c_x = to_fpt_efpt(&self.sqrt_expr.eval2(&c_a, &c_b));
                    c_event.set_x(c_x / denom);
                }

                if recompute_lower_x {
                    c_a[2] = if c[0].is_neg() { -c[0] } else { c[0] };
                    c_b[2] = a[0] * a[0] + b[0] * b[0];
                    let lower_x = to_fpt_efpt(&self.sqrt_expr.eval3(&c_a, &c_b));
                    c_event.set_lower_x(lower_x / denom);
                }
            }
            return;
        }
        c[0] = b[0] * segm_end1.x - a[0] * segm_end1.y;
        c[1] = a[1] * segm_end2.y - b[1] * segm_end2.x;
        let ix = a[0] * c[1] + a[1] * c[0];
        let iy = b[0] * c[1] + b[1] * c[0];
        let dx = ix - orientation * site1.x();
        let dy = iy - orientation * site1.y();
        if dx.is_zero() && dy.is_zero() {
            let denom = to_fpt_int(&orientation);
            let c_x = to_fpt_int(&ix) / denom;
            let c_y = to_fpt_int(&iy) / denom;
            *c_event = CircleEvent::new(c_x, c_y, c_x);
            return;
        }

        let sign = ExtendedInt::from((if point_index == 2 { 1i32 } else { -1 }) * (if orientation.is_neg() { 1 } else { -1 }));
        c_a[0] = a[1] * -dx + b[1] * -dy;
        c_a[1] = a[0] * -dx + b[0] * -dy;
        c_a[2] = sign;
        c_a[3] = ExtendedInt::from(0i32);
        c_b[0] = a[0] * a[0] + b[0] * b[0];
        c_b[1] = a[1] * a[1] + b[1] * b[1];
        c_b[2] = a[0] * a[1] + b[0] * b[1];
        c_b[3] = (a[0] * dy - b[0] * dx) * (a[1] * dy - b[1] * dx) * -2i32;
        let temp = to_fpt_efpt(&self.sqrt_expr_evaluator_pss4(&c_a, &c_b));
        let denom = temp * to_fpt_int(&orientation);

        if recompute_c_y {
            c_a[0] = b[1] * (dx * dx + dy * dy) - iy * (dx * a[1] + dy * b[1]);
            c_a[1] = b[0] * (dx * dx + dy * dy) - iy * (dx * a[0] + dy * b[0]);
            c_a[2] = iy * sign;
            let cy = to_fpt_efpt(&self.sqrt_expr_evaluator_pss4(&c_a, &c_b));
            c_event.set_y(cy / denom);
        }

        if recompute_c_x || recompute_lower_x {
            c_a[0] = a[1] * (dx * dx + dy * dy) - ix * (dx * a[1] + dy * b[1]);
            c_a[1] = a[0] * (dx * dx + dy * dy) - ix * (dx * a[0] + dy * b[0]);
            c_a[2] = ix * sign;

            if recompute_c_x {
                let cx = to_fpt_efpt(&self.sqrt_expr_evaluator_pss4(&c_a, &c_b));
                c_event.set_x(cx / denom);
            }

            if recompute_lower_x {
                c_a[3] = orientation * (dx * dx + dy * dy) * (if temp < 0.0 { -1i32 } else { 1i32 });
                let lower_x = to_fpt_efpt(&self.sqrt_expr_evaluator_pss4(&c_a, &c_b));
                c_event.set_lower_x(lower_x / denom);
            }
        }
    }

    /// voronoi_predicates.hpp:832-911 `sss`.
    pub fn sss(
        &mut self,
        site1: &SiteEvent,
        site2: &SiteEvent,
        site3: &SiteEvent,
        c_event: &mut CircleEvent,
        recompute_c_x: bool,
        recompute_c_y: bool,
        recompute_lower_x: bool,
    ) {
        count(&EXACT_SSS);
        let mut c_a = [ExtendedInt::default(); 4];
        let mut c_b = [ExtendedInt::default(); 4];
        // cA - corresponds to the cross product.
        // cB - corresponds to the squared length.
        let sites = [site1, site2, site3];
        let mut a = [ExtendedInt::default(); 3];
        let mut b = [ExtendedInt::default(); 3];
        let mut c = [ExtendedInt::default(); 3];
        for i in 0..3 {
            a[i] = bi(sites[i].x1() as i64 - sites[i].x0() as i64);
        }
        for i in 0..3 {
            b[i] = bi(sites[i].y1() as i64 - sites[i].y0() as i64);
        }
        for i in 0..3 {
            c[i] = bi(sites[i].x0() as i64 * sites[i].y1() as i64 - sites[i].y0() as i64 * sites[i].x1() as i64);
        }

        for i in 0..3 {
            c_b[i] = a[i] * a[i] + b[i] * b[i];
        }

        for i in 0..3 {
            let j = (i + 1) % 3;
            let k = (i + 2) % 3;
            c_a[i] = a[j] * b[k] - a[k] * b[j];
        }
        let denom = to_fpt_efpt(&self.sqrt_expr.eval3(&c_a, &c_b));

        if recompute_c_y {
            for i in 0..3 {
                let j = (i + 1) % 3;
                let k = (i + 2) % 3;
                c_a[i] = b[j] * c[k] - b[k] * c[j];
            }
            let c_y = to_fpt_efpt(&self.sqrt_expr.eval3(&c_a, &c_b));
            c_event.set_y(c_y / denom);
        }

        if recompute_c_x || recompute_lower_x {
            c_a[3] = ExtendedInt::from(0i32);
            for i in 0..3 {
                let j = (i + 1) % 3;
                let k = (i + 2) % 3;
                c_a[i] = a[j] * c[k] - a[k] * c[j];
                if recompute_lower_x {
                    c_a[3] = c_a[3] + c_a[i] * b[i];
                }
            }

            if recompute_c_x {
                let c_x = to_fpt_efpt(&self.sqrt_expr.eval3(&c_a, &c_b));
                c_event.set_x(c_x / denom);
            }

            if recompute_lower_x {
                c_b[3] = ExtendedInt::from(1i32);
                let lower_x = to_fpt_efpt(&self.sqrt_expr.eval4(&c_a, &c_b));
                c_event.set_lower_x(lower_x / denom);
            }
        }
    }

    /// voronoi_predicates.hpp:914-959 `sqrt_expr_evaluator_pss4`:
    /// `A[3] + A[0] * sqrt(B[0]) + A[1] * sqrt(B[1]) + A[2] * sqrt(B[3] * (sqrt(B[0] * B[1]) + B[2]))`.
    fn sqrt_expr_evaluator_pss4(&mut self, a: &[ExtendedInt; 4], b: &[ExtendedInt; 4]) -> ExtendedExponentFpt {
        let mut c_a = [ExtendedInt::default(); 4];
        let mut c_b = [ExtendedInt::default(); 4];
        if a[3].is_zero() {
            let lh = self.sqrt_expr.eval2(a, b);
            c_a[0] = ExtendedInt::from(1i32);
            c_b[0] = b[0] * b[1];
            c_a[1] = b[2];
            c_b[1] = ExtendedInt::from(1i32);
            let rh = self.sqrt_expr.eval1(&a[2..], &b[3..]) * self.sqrt_expr.eval2(&c_a, &c_b).sqrt();
            if same_sign(&lh, &rh) {
                return lh + rh;
            }
            count(&SQRT_EXPR_CONJUGATE);
            c_a[0] = a[0] * a[0] * b[0] + a[1] * a[1] * b[1] - a[2] * a[2] * b[3] * b[2];
            c_b[0] = ExtendedInt::from(1i32);
            c_a[1] = a[0] * a[1] * 2i32 - a[2] * a[2] * b[3];
            c_b[1] = b[0] * b[1];
            let numer = self.sqrt_expr.eval2(&c_a, &c_b);
            return numer / (lh - rh);
        }
        c_a[0] = ExtendedInt::from(1i32);
        c_b[0] = b[0] * b[1];
        c_a[1] = b[2];
        c_b[1] = ExtendedInt::from(1i32);
        let rh = self.sqrt_expr.eval1(&a[2..], &b[3..]) * self.sqrt_expr.eval2(&c_a, &c_b).sqrt();
        c_a[0] = a[0];
        c_b[0] = b[0];
        c_a[1] = a[1];
        c_b[1] = b[1];
        c_a[2] = a[3];
        c_b[2] = ExtendedInt::from(1i32);
        let lh = self.sqrt_expr.eval3(&c_a, &c_b);
        if same_sign(&lh, &rh) {
            return lh + rh;
        }
        count(&SQRT_EXPR_CONJUGATE);
        c_a[0] = a[3] * a[0] * 2i32;
        c_a[1] = a[3] * a[1] * 2i32;
        c_a[2] = a[0] * a[0] * b[0] + a[1] * a[1] * b[1] + a[3] * a[3] - a[2] * a[2] * b[2] * b[3];
        c_a[3] = a[0] * a[1] * 2i32 - a[2] * a[2] * b[3];
        c_b[3] = b[0] * b[1];
        let numer = self.sqrt_expr_evaluator_pss3(&c_a, &c_b);
        numer / (lh - rh)
    }

    /// voronoi_predicates.hpp:961-978 `sqrt_expr_evaluator_pss3`:
    /// `A[0] * sqrt(B[0]) + A[1] * sqrt(B[1]) + A[2] + A[3] * sqrt(B[0] * B[1])`, `B[3] = B[0] * B[1]`.
    fn sqrt_expr_evaluator_pss3(&mut self, a: &[ExtendedInt; 4], b: &[ExtendedInt; 4]) -> ExtendedExponentFpt {
        let mut c_a = [ExtendedInt::default(); 2];
        let mut c_b = [ExtendedInt::default(); 2];
        let lh = self.sqrt_expr.eval2(a, b);
        let rh = self.sqrt_expr.eval2(&a[2..], &b[2..]);
        if same_sign(&lh, &rh) {
            return lh + rh;
        }
        count(&SQRT_EXPR_CONJUGATE);
        c_a[0] = a[0] * a[0] * b[0] + a[1] * a[1] * b[1] - a[2] * a[2] - a[3] * a[3] * b[0] * b[1];
        c_b[0] = ExtendedInt::from(1i32);
        c_a[1] = (a[0] * a[1] - a[2] * a[3]) * 2i32;
        c_b[1] = b[3];
        let numer = self.sqrt_expr.eval2(&c_a, &c_b);
        numer / (lh - rh)
    }
}

#[inline]
fn rf(fpv: f64) -> RobustFpt {
    RobustFpt::new(fpv)
}

#[inline]
fn rf2(fpv: f64, error: f64) -> RobustFpt {
    RobustFpt::new2(fpv, error)
}

/// `robust_cross_product` of four `int32` differences given as `int_x2_type`.
#[inline]
fn rcp(a1: i64, b1: i64, a2: i64, b2: i64) -> f64 {
    robust_cross_product(a1, b1, a2, b2)
}

/// voronoi_predicates.hpp:984-1431 `lazy_circle_formation_functor`: the circle in doubles
/// with the relative error of every coordinate tracked; a coordinate whose error exceeds
/// `ULPS` is recomputed by [`MpCircleFormationFunctor`].
#[derive(Clone, Debug, Default)]
pub struct LazyCircleFormationFunctor {
    exact_circle_formation_functor: MpCircleFormationFunctor,
}

impl LazyCircleFormationFunctor {
    /// voronoi_predicates.hpp:995-1045 `ppp`.
    pub fn ppp(&mut self, site1: &SiteEvent, site2: &SiteEvent, site3: &SiteEvent, c_event: &mut CircleEvent) {
        let (x1, y1) = (site1.x() as f64, site1.y() as f64);
        let (x2, y2) = (site2.x() as f64, site2.y() as f64);
        let (x3, y3) = (site3.x() as f64, site3.y() as f64);
        let dif_x1 = x1 - x2;
        let dif_x2 = x2 - x3;
        let dif_y1 = y1 - y2;
        let dif_y2 = y2 - y3;
        let orientation = rcp(
            site1.x() as i64 - site2.x() as i64,
            site2.x() as i64 - site3.x() as i64,
            site1.y() as i64 - site2.y() as i64,
            site2.y() as i64 - site3.y() as i64,
        );
        let inv_orientation = rf2(0.5 / orientation, 2.0);
        let sum_x1 = x1 + x2;
        let sum_x2 = x2 + x3;
        let sum_y1 = y1 + y2;
        let sum_y2 = y2 + y3;
        let dif_x3 = x1 - x3;
        let dif_y3 = y1 - y3;
        let mut c_x = RobustDif::new();
        let mut c_y = RobustDif::new();
        c_x += rf2(dif_x1 * sum_x1 * dif_y2, 2.0);
        c_x += rf2(dif_y1 * sum_y1 * dif_y2, 2.0);
        c_x -= rf2(dif_x2 * sum_x2 * dif_y1, 2.0);
        c_x -= rf2(dif_y2 * sum_y2 * dif_y1, 2.0);
        c_y += rf2(dif_x2 * sum_x2 * dif_x1, 2.0);
        c_y += rf2(dif_y2 * sum_y2 * dif_x1, 2.0);
        c_y -= rf2(dif_x1 * sum_x1 * dif_x2, 2.0);
        c_y -= rf2(dif_y1 * sum_y1 * dif_x2, 2.0);
        let mut lower_x = c_x;
        // `(dif_x1 * dif_x1 + dif_y1 * dif_y1) * (...) * (...)`: the left product of each sum
        // is fused.
        lower_x -= rf2(
            (dif_x1.mul_add(dif_x1, dif_y1 * dif_y1)
                * dif_x2.mul_add(dif_x2, dif_y2 * dif_y2)
                * dif_x3.mul_add(dif_x3, dif_y3 * dif_y3))
            .sqrt(),
            5.0,
        );
        *c_event = CircleEvent::new(
            c_x.dif().fpv() * inv_orientation.fpv(),
            c_y.dif().fpv() * inv_orientation.fpv(),
            lower_x.dif().fpv() * inv_orientation.fpv(),
        );
        let recompute_c_x = c_x.dif().ulp() > ULPS as f64;
        let recompute_c_y = c_y.dif().ulp() > ULPS as f64;
        let recompute_lower_x = lower_x.dif().ulp() > ULPS as f64;
        if recompute_c_x || recompute_c_y || recompute_lower_x {
            self.exact_circle_formation_functor.ppp(site1, site2, site3, c_event, recompute_c_x, recompute_c_y, recompute_lower_x);
        }
    }

    /// voronoi_predicates.hpp:1047-1132 `pps`.
    pub fn pps(&mut self, site1: &SiteEvent, site2: &SiteEvent, site3: &SiteEvent, segment_index: i32, c_event: &mut CircleEvent) {
        let line_a = site3.y1() as f64 - site3.y0() as f64;
        let line_b = site3.x0() as f64 - site3.x1() as f64;
        let vec_x = site2.y() as f64 - site1.y() as f64;
        let vec_y = site1.x() as f64 - site2.x() as f64;
        let teta = rf2(
            rcp(
                site3.y1() as i64 - site3.y0() as i64,
                site3.x0() as i64 - site3.x1() as i64,
                site2.x() as i64 - site1.x() as i64,
                site2.y() as i64 - site1.y() as i64,
            ),
            1.0,
        );
        let a = rf2(
            rcp(
                site3.y0() as i64 - site3.y1() as i64,
                site3.x0() as i64 - site3.x1() as i64,
                site3.y1() as i64 - site1.y() as i64,
                site3.x1() as i64 - site1.x() as i64,
            ),
            1.0,
        );
        let b = rf2(
            rcp(
                site3.y0() as i64 - site3.y1() as i64,
                site3.x0() as i64 - site3.x1() as i64,
                site3.y1() as i64 - site2.y() as i64,
                site3.x1() as i64 - site2.x() as i64,
            ),
            1.0,
        );
        let denom = rf2(
            rcp(
                site1.y() as i64 - site2.y() as i64,
                site1.x() as i64 - site2.x() as i64,
                site3.y1() as i64 - site3.y0() as i64,
                site3.x1() as i64 - site3.x0() as i64,
            ),
            1.0,
        );
        // `get_sqrt(line_a * line_a + line_b * line_b)`: the left product is fused.
        let inv_segm_len = rf2(1.0 / line_a.mul_add(line_a, line_b * line_b).sqrt(), 3.0);
        let mut t = RobustDif::new();
        if ot_eval_value(denom.fpv()) == Orientation::Collinear {
            t += teta / (rf(8.0) * a);
            t -= a / (rf(2.0) * teta);
        } else {
            let det = ((teta * teta + denom * denom) * a * b).sqrt();
            if segment_index == 2 {
                t -= det / (denom * denom);
            } else {
                t += det / (denom * denom);
            }
            t += teta * (a + b) / (rf(2.0) * denom * denom);
        }
        let mut c_x = RobustDif::new();
        let mut c_y = RobustDif::new();
        c_x += rf(0.5 * (site1.x() as f64 + site2.x() as f64));
        c_x += rf(vec_x) * t;
        c_y += rf(0.5 * (site1.y() as f64 + site2.y() as f64));
        c_y += rf(vec_y) * t;
        let mut r = RobustDif::new();
        let mut lower_x = c_x;
        r -= rf(line_a) * rf(site3.x0() as f64);
        r -= rf(line_b) * rf(site3.y0() as f64);
        r += rf(line_a) * c_x;
        r += rf(line_b) * c_y;
        if r.pos().fpv() < r.neg_sum().fpv() {
            r = -r;
        }
        lower_x += r * inv_segm_len;
        *c_event = CircleEvent::new(c_x.dif().fpv(), c_y.dif().fpv(), lower_x.dif().fpv());
        let recompute_c_x = c_x.dif().ulp() > ULPS as f64;
        let recompute_c_y = c_y.dif().ulp() > ULPS as f64;
        let recompute_lower_x = lower_x.dif().ulp() > ULPS as f64;
        if recompute_c_x || recompute_c_y || recompute_lower_x {
            self.exact_circle_formation_functor.pps(
                site1,
                site2,
                site3,
                segment_index,
                c_event,
                recompute_c_x,
                recompute_c_y,
                recompute_lower_x,
            );
        }
    }

    /// voronoi_predicates.hpp:1134-1327 `pss`.
    pub fn pss(&mut self, site1: &SiteEvent, site2: &SiteEvent, site3: &SiteEvent, point_index: i32, c_event: &mut CircleEvent) {
        let segm_start1 = site2.point1();
        let segm_end1 = site2.point0();
        let segm_start2 = site3.point0();
        let segm_end2 = site3.point1();
        let (s1x, s1y) = (segm_start1.x as i64, segm_start1.y as i64);
        let (e1x, e1y) = (segm_end1.x as i64, segm_end1.y as i64);
        let (s2x, s2y) = (segm_start2.x as i64, segm_start2.y as i64);
        let (e2x, e2y) = (segm_end2.x as i64, segm_end2.y as i64);
        let (px, py) = (site1.x() as i64, site1.y() as i64);
        let a1 = segm_end1.x as f64 - segm_start1.x as f64;
        let b1 = segm_end1.y as f64 - segm_start1.y as f64;
        let a2 = segm_end2.x as f64 - segm_start2.x as f64;
        let b2 = segm_end2.y as f64 - segm_start2.y as f64;
        let recompute_c_x;
        let recompute_c_y;
        let recompute_lower_x;
        let orientation = rf2(rcp(e1y - s1y, e1x - s1x, e2y - s2y, e2x - s2x), 1.0);
        if ot_eval_value(orientation.fpv()) == Orientation::Collinear {
            // `a1 * a1 + b1 * b1`: the left product is fused.
            let a = rf2(a1.mul_add(a1, b1 * b1), 2.0);
            let c = rf2(rcp(e1y - s1y, e1x - s1x, s2y - s1y, s2x - s1x), 1.0);
            let det = rf2(
                rcp(e1x - s1x, e1y - s1y, px - s1x, py - s1y) * rcp(e1y - s1y, e1x - s1x, py - s2y, px - s2x),
                3.0,
            );
            let mut t = RobustDif::new();
            // `(start1 + start2) * 0.5 - site1`: contracted.
            t -= rf(a1) * rf((segm_start1.x as f64 + segm_start2.x as f64).mul_add(0.5, -(site1.x() as f64)));
            t -= rf(b1) * rf((segm_start1.y as f64 + segm_start2.y as f64).mul_add(0.5, -(site1.y() as f64)));
            if point_index == 2 {
                t += det.sqrt();
            } else {
                t -= det.sqrt();
            }
            t /= a;
            let mut c_x = RobustDif::new();
            let mut c_y = RobustDif::new();
            c_x += rf(0.5 * (segm_start1.x as f64 + segm_start2.x as f64));
            c_x += rf(a1) * t;
            c_y += rf(0.5 * (segm_start1.y as f64 + segm_start2.y as f64));
            c_y += rf(b1) * t;
            let mut lower_x = c_x;
            if c.has_neg_value() {
                lower_x -= rf(0.5) * c / a.sqrt();
            } else {
                lower_x += rf(0.5) * c / a.sqrt();
            }
            recompute_c_x = c_x.dif().ulp() > ULPS as f64;
            recompute_c_y = c_y.dif().ulp() > ULPS as f64;
            recompute_lower_x = lower_x.dif().ulp() > ULPS as f64;
            *c_event = CircleEvent::new(c_x.dif().fpv(), c_y.dif().fpv(), lower_x.dif().fpv());
        } else {
            // `get_sqrt(a * a + b * b)`: the left product is fused.
            let sqr_sum1 = rf2(a1.mul_add(a1, b1 * b1).sqrt(), 2.0);
            let sqr_sum2 = rf2(a2.mul_add(a2, b2 * b2).sqrt(), 2.0);
            let mut a = rf2(rcp(e1x - s1x, e1y - s1y, s2y - e2y, e2x - s2x), 1.0);
            if !a.has_neg_value() {
                a += sqr_sum1 * sqr_sum2;
            } else {
                a = (orientation * orientation) / (sqr_sum1 * sqr_sum2 - a);
            }
            let or1 = rf2(rcp(e1y - s1y, e1x - s1x, e1y - py, e1x - px), 1.0);
            let or2 = rf2(rcp(e2x - s2x, e2y - s2y, e2x - px, e2y - py), 1.0);
            let det = rf(2.0) * a * or1 * or2;
            let c1 = rf2(rcp(e1y - s1y, e1x - s1x, e1y, e1x), 1.0);
            let c2 = rf2(rcp(e2x - s2x, e2y - s2y, e2x, e2y), 1.0);
            let inv_orientation = rf(1.0) / orientation;
            let mut t = RobustDif::new();
            let mut b = RobustDif::new();
            let mut ix = RobustDif::new();
            let mut iy = RobustDif::new();
            ix += rf(a2) * c1 * inv_orientation;
            ix += rf(a1) * c2 * inv_orientation;
            iy += rf(b1) * c2 * inv_orientation;
            iy += rf(b2) * c1 * inv_orientation;

            b += ix * (rf(a1) * sqr_sum2);
            b += ix * (rf(a2) * sqr_sum1);
            b += iy * (rf(b1) * sqr_sum2);
            b += iy * (rf(b2) * sqr_sum1);
            // `static_cast<int_x2_type>(-site1.y())`: the negation is of the `int32`.
            let neg_py = site1.y().wrapping_neg() as i64;
            b -= sqr_sum1 * rf2(rcp(e2x - s2x, e2y - s2y, neg_py, px), 1.0);
            b -= sqr_sum2 * rf2(rcp(e1x - s1x, e1y - s1y, neg_py, px), 1.0);
            t -= b;
            if point_index == 2 {
                t += det.sqrt();
            } else {
                t -= det.sqrt();
            }
            t /= a * a;
            let mut c_x = ix;
            let mut c_y = iy;
            c_x += t * (rf(a1) * sqr_sum2);
            c_x += t * (rf(a2) * sqr_sum1);
            c_y += t * (rf(b1) * sqr_sum2);
            c_y += t * (rf(b2) * sqr_sum1);
            if t.pos().fpv() < t.neg_sum().fpv() {
                t = -t;
            }
            let mut lower_x = c_x;
            if orientation.has_neg_value() {
                lower_x -= t * orientation;
            } else {
                lower_x += t * orientation;
            }
            recompute_c_x = c_x.dif().ulp() > ULPS as f64;
            recompute_c_y = c_y.dif().ulp() > ULPS as f64;
            recompute_lower_x = lower_x.dif().ulp() > ULPS as f64;
            *c_event = CircleEvent::new(c_x.dif().fpv(), c_y.dif().fpv(), lower_x.dif().fpv());
        }
        if recompute_c_x || recompute_c_y || recompute_lower_x {
            self.exact_circle_formation_functor.pss(
                site1,
                site2,
                site3,
                point_index,
                c_event,
                recompute_c_x,
                recompute_c_y,
                recompute_lower_x,
            );
        }
    }

    /// voronoi_predicates.hpp:1329-1426 `sss`.
    pub fn sss(&mut self, site1: &SiteEvent, site2: &SiteEvent, site3: &SiteEvent, c_event: &mut CircleEvent) {
        let a1 = rf(site1.x1() as f64 - site1.x0() as f64);
        let b1 = rf(site1.y1() as f64 - site1.y0() as f64);
        let c1 = rf2(rcp(site1.x0() as i64, site1.y0() as i64, site1.x1() as i64, site1.y1() as i64), 1.0);

        let a2 = rf(site2.x1() as f64 - site2.x0() as f64);
        let b2 = rf(site2.y1() as f64 - site2.y0() as f64);
        let c2 = rf2(rcp(site2.x0() as i64, site2.y0() as i64, site2.x1() as i64, site2.y1() as i64), 1.0);

        let a3 = rf(site3.x1() as f64 - site3.x0() as f64);
        let b3 = rf(site3.y1() as f64 - site3.y0() as f64);
        let c3 = rf2(rcp(site3.x0() as i64, site3.y0() as i64, site3.x1() as i64, site3.y1() as i64), 1.0);

        let len1 = (a1 * a1 + b1 * b1).sqrt();
        let len2 = (a2 * a2 + b2 * b2).sqrt();
        let len3 = (a3 * a3 + b3 * b3).sqrt();
        let d = |s: &SiteEvent| (s.x1() as i64 - s.x0() as i64, s.y1() as i64 - s.y0() as i64);
        let (d1, d2, d3) = (d(site1), d(site2), d(site3));
        let cross_12 = rf2(rcp(d1.0, d1.1, d2.0, d2.1), 1.0);
        let cross_23 = rf2(rcp(d2.0, d2.1, d3.0, d3.1), 1.0);
        let cross_31 = rf2(rcp(d3.0, d3.1, d1.0, d1.1), 1.0);

        // denom = cross_12 * len3 + cross_23 * len1 + cross_31 * len2.
        let mut denom = RobustDif::new();
        denom += cross_12 * len3;
        denom += cross_23 * len1;
        denom += cross_31 * len2;

        // denom * r = (b2 * c_x - a2 * c_y - c2 * denom) / len2.
        let mut r = RobustDif::new();
        r -= cross_12 * c3;
        r -= cross_23 * c1;
        r -= cross_31 * c2;

        let mut c_x = RobustDif::new();
        c_x += a1 * c2 * len3;
        c_x -= a2 * c1 * len3;
        c_x += a2 * c3 * len1;
        c_x -= a3 * c2 * len1;
        c_x += a3 * c1 * len2;
        c_x -= a1 * c3 * len2;

        let mut c_y = RobustDif::new();
        c_y += b1 * c2 * len3;
        c_y -= b2 * c1 * len3;
        c_y += b2 * c3 * len1;
        c_y -= b3 * c2 * len1;
        c_y += b3 * c1 * len2;
        c_y -= b1 * c3 * len2;

        let lower_x = c_x + r;

        let denom_dif = denom.dif();
        let c_x_dif = c_x.dif() / denom_dif;
        let c_y_dif = c_y.dif() / denom_dif;
        let lower_x_dif = lower_x.dif() / denom_dif;

        let recompute_c_x = c_x_dif.ulp() > ULPS as f64;
        let recompute_c_y = c_y_dif.ulp() > ULPS as f64;
        let recompute_lower_x = lower_x_dif.ulp() > ULPS as f64;
        *c_event = CircleEvent::new(c_x_dif.fpv(), c_y_dif.fpv(), lower_x_dif.fpv());
        if recompute_c_x || recompute_c_y || recompute_lower_x {
            self.exact_circle_formation_functor.sss(site1, site2, site3, c_event, recompute_c_x, recompute_c_y, recompute_lower_x);
        }
    }
}

/// voronoi_predicates.hpp:1433-1527 `circle_formation_predicate`.
#[derive(Clone, Debug, Default)]
pub struct CircleFormationPredicate {
    circle_formation_functor: LazyCircleFormationFunctor,
}

impl CircleFormationPredicate {
    /// voronoi_predicates.hpp:1447-1508 `operator()`: the circle event of three consecutive
    /// arcs, or `false` when they form none.
    pub fn call(&mut self, site1: &SiteEvent, site2: &SiteEvent, site3: &SiteEvent, circle: &mut CircleEvent) -> bool {
        let f = &mut self.circle_formation_functor;
        if !site1.is_segment() {
            if !site2.is_segment() {
                if !site3.is_segment() {
                    // (point, point, point) sites.
                    if !circle_existence_ppp(site1, site2, site3) {
                        return false;
                    }
                    f.ppp(site1, site2, site3, circle);
                } else {
                    // (point, point, segment) sites.
                    if !circle_existence_pps(site1, site2, site3, 3) {
                        return false;
                    }
                    f.pps(site1, site2, site3, 3, circle);
                }
            } else {
                if !site3.is_segment() {
                    // (point, segment, point) sites.
                    if !circle_existence_pps(site1, site3, site2, 2) {
                        return false;
                    }
                    f.pps(site1, site3, site2, 2, circle);
                } else {
                    // (point, segment, segment) sites.
                    if !circle_existence_pss(site1, site2, site3, 1) {
                        return false;
                    }
                    f.pss(site1, site2, site3, 1, circle);
                }
            }
        } else {
            if !site2.is_segment() {
                if !site3.is_segment() {
                    // (segment, point, point) sites.
                    if !circle_existence_pps(site2, site3, site1, 1) {
                        return false;
                    }
                    f.pps(site2, site3, site1, 1, circle);
                } else {
                    // (segment, point, segment) sites.
                    if !circle_existence_pss(site2, site1, site3, 2) {
                        return false;
                    }
                    f.pss(site2, site1, site3, 2, circle);
                }
            } else {
                if !site3.is_segment() {
                    // (segment, segment, point) sites.
                    if !circle_existence_pss(site3, site1, site2, 3) {
                        return false;
                    }
                    f.pss(site3, site1, site2, 3, circle);
                } else {
                    // (segment, segment, segment) sites.
                    if !circle_existence_sss(site1, site2, site3) {
                        return false;
                    }
                    f.sss(site1, site2, site3, circle);
                }
            }
        }
        if lies_outside_vertical_segment(circle, site1)
            || lies_outside_vertical_segment(circle, site2)
            || lies_outside_vertical_segment(circle, site3)
        {
            return false;
        }
        true
    }
}

/// voronoi_predicates.hpp:1511-1520 `lies_outside_vertical_segment`.
fn lies_outside_vertical_segment(c: &CircleEvent, s: &SiteEvent) -> bool {
    if !s.is_segment() || !is_vertical(s) {
        return false;
    }
    let y0 = (if s.is_inverse() { s.y1() } else { s.y0() }) as f64;
    let y1 = (if s.is_inverse() { s.y0() } else { s.y1() }) as f64;
    ulp_comparison(c.y(), y0, ULPS) == UlpResult::Less || ulp_comparison(c.y(), y1, ULPS) == UlpResult::More
}

// ===========================================================================
// The beach line: std::map<key_type, value_type, node_comparer_type> of libc++
// (<__tree>). The comparison is only consistent for the node being inserted, so
// the searches have to visit the nodes the C++ ones visit: the tree keeps the
// shape libc++'s has.
// ===========================================================================

const NIL: usize = usize::MAX;

/// `__tree_node`: the links of `__tree_node_base` and the value. Node 0 is the end node,
/// whose `left` is the root.
#[derive(Clone, Copy, Debug)]
struct BeachLineNode {
    left: usize,
    right: usize,
    parent: usize,
    is_black: bool,
    key: BeachLineNodeKey,
    data: BeachLineNodeData,
}

/// `beach_line_type` (voronoi_builder.hpp:157). An iterator is a node index; [`BeachLine::END`]
/// is `end()`.
#[derive(Clone, Debug)]
pub struct BeachLine {
    nodes: Vec<BeachLineNode>,
    free: Vec<usize>,
    begin_node: usize,
    size: usize,
}

impl Default for BeachLine {
    fn default() -> Self {
        Self::new()
    }
}

impl BeachLine {
    /// `end()`: the end node.
    pub const END: usize = 0;

    pub fn new() -> Self {
        let end = BeachLineNode {
            left: NIL,
            right: NIL,
            parent: NIL,
            is_black: false,
            key: BeachLineNodeKey::new(SiteEvent::default()),
            data: BeachLineNodeData::new(None),
        };
        BeachLine { nodes: vec![end], free: Vec::new(), begin_node: Self::END, size: 0 }
    }

    /// `begin()`.
    pub fn begin(&self) -> usize {
        self.begin_node
    }

    pub fn len(&self) -> usize {
        self.size
    }

    pub fn is_empty(&self) -> bool {
        self.size == 0
    }

    /// `clear()`.
    pub fn clear(&mut self) {
        self.nodes.truncate(1);
        self.nodes[Self::END].left = NIL;
        self.free.clear();
        self.begin_node = Self::END;
        self.size = 0;
    }

    pub fn key(&self, it: usize) -> &BeachLineNodeKey {
        &self.nodes[it].key
    }

    /// `const_cast<key_type&>(it->first)`.
    pub fn key_mut(&mut self, it: usize) -> &mut BeachLineNodeKey {
        &mut self.nodes[it].key
    }

    pub fn data(&self, it: usize) -> &BeachLineNodeData {
        &self.nodes[it].data
    }

    pub fn data_mut(&mut self, it: usize) -> &mut BeachLineNodeData {
        &mut self.nodes[it].data
    }

    fn root(&self) -> usize {
        self.nodes[Self::END].left
    }

    /// __tree:107-109 `__tree_is_left_child`.
    fn is_left_child(&self, x: usize) -> bool {
        x == self.nodes[self.nodes[x].parent].left
    }

    /// __tree:164-169 `__tree_min`.
    fn tree_min(&self, mut x: usize) -> usize {
        while self.nodes[x].left != NIL {
            x = self.nodes[x].left;
        }
        x
    }

    /// __tree:173-178 `__tree_max`.
    fn tree_max(&self, mut x: usize) -> usize {
        while self.nodes[x].right != NIL {
            x = self.nodes[x].right;
        }
        x
    }

    /// `++it`: __tree `__tree_next_iter`.
    pub fn next(&self, mut x: usize) -> usize {
        if self.nodes[x].right != NIL {
            return self.tree_min(self.nodes[x].right);
        }
        while !self.is_left_child(x) {
            x = self.nodes[x].parent;
        }
        self.nodes[x].parent
    }

    /// `--it`: __tree `__tree_prev_iter`.
    pub fn prev(&self, mut x: usize) -> usize {
        if self.nodes[x].left != NIL {
            return self.tree_max(self.nodes[x].left);
        }
        while self.is_left_child(x) {
            x = self.nodes[x].parent;
        }
        self.nodes[x].parent
    }

    /// __tree `__tree_left_rotate`.
    fn left_rotate(&mut self, x: usize) {
        let y = self.nodes[x].right;
        self.nodes[x].right = self.nodes[y].left;
        if self.nodes[x].right != NIL {
            let r = self.nodes[x].right;
            self.nodes[r].parent = x;
        }
        self.nodes[y].parent = self.nodes[x].parent;
        let p = self.nodes[x].parent;
        if self.is_left_child(x) {
            self.nodes[p].left = y;
        } else {
            self.nodes[p].right = y;
        }
        self.nodes[y].left = x;
        self.nodes[x].parent = y;
    }

    /// __tree `__tree_right_rotate`.
    fn right_rotate(&mut self, x: usize) {
        let y = self.nodes[x].left;
        self.nodes[x].left = self.nodes[y].right;
        if self.nodes[x].left != NIL {
            let l = self.nodes[x].left;
            self.nodes[l].parent = x;
        }
        self.nodes[y].parent = self.nodes[x].parent;
        let p = self.nodes[x].parent;
        if self.is_left_child(x) {
            self.nodes[p].left = y;
        } else {
            self.nodes[p].right = y;
        }
        self.nodes[y].right = x;
        self.nodes[x].parent = y;
    }

    /// __tree:279-325 `__tree_balance_after_insert(root, x)`.
    fn balance_after_insert(&mut self, root: usize, mut x: usize) {
        self.nodes[x].is_black = x == root;
        while x != root && !self.nodes[self.nodes[x].parent].is_black {
            let xp = self.nodes[x].parent;
            let xpp = self.nodes[xp].parent;
            if self.is_left_child(xp) {
                let y = self.nodes[xpp].right;
                if y != NIL && !self.nodes[y].is_black {
                    x = xp;
                    self.nodes[x].is_black = true;
                    x = xpp;
                    self.nodes[x].is_black = x == root;
                    self.nodes[y].is_black = true;
                } else {
                    if !self.is_left_child(x) {
                        x = xp;
                        self.left_rotate(x);
                    }
                    x = self.nodes[x].parent;
                    self.nodes[x].is_black = true;
                    x = self.nodes[x].parent;
                    self.nodes[x].is_black = false;
                    self.right_rotate(x);
                    break;
                }
            } else {
                let y = self.nodes[xpp].left;
                if y != NIL && !self.nodes[y].is_black {
                    x = xp;
                    self.nodes[x].is_black = true;
                    x = xpp;
                    self.nodes[x].is_black = x == root;
                    self.nodes[y].is_black = true;
                } else {
                    if self.is_left_child(x) {
                        x = xp;
                        self.right_rotate(x);
                    }
                    x = self.nodes[x].parent;
                    self.nodes[x].is_black = true;
                    x = self.nodes[x].parent;
                    self.nodes[x].is_black = false;
                    self.left_rotate(x);
                    break;
                }
            }
        }
    }

    /// __tree:335-470 `__tree_remove(root, z)`.
    fn tree_remove(&mut self, mut root: usize, z: usize) {
        let y = if self.nodes[z].left == NIL || self.nodes[z].right == NIL { z } else { self.next(z) };
        let mut x = if self.nodes[y].left != NIL { self.nodes[y].left } else { self.nodes[y].right };
        let mut w = NIL;
        if x != NIL {
            self.nodes[x].parent = self.nodes[y].parent;
        }
        let yp = self.nodes[y].parent;
        if self.is_left_child(y) {
            self.nodes[yp].left = x;
            if y != root {
                w = self.nodes[yp].right;
            } else {
                root = x; // w == NIL
            }
        } else {
            self.nodes[yp].right = x;
            w = self.nodes[yp].left;
        }
        let removed_black = self.nodes[y].is_black;
        if y != z {
            self.nodes[y].parent = self.nodes[z].parent;
            let zp = self.nodes[z].parent;
            if self.is_left_child(z) {
                self.nodes[zp].left = y;
            } else {
                self.nodes[zp].right = y;
            }
            self.nodes[y].left = self.nodes[z].left;
            let yl = self.nodes[y].left;
            self.nodes[yl].parent = y;
            self.nodes[y].right = self.nodes[z].right;
            if self.nodes[y].right != NIL {
                let yr = self.nodes[y].right;
                self.nodes[yr].parent = y;
            }
            self.nodes[y].is_black = self.nodes[z].is_black;
            if root == z {
                root = y;
            }
        }
        if removed_black && root != NIL {
            if x != NIL {
                self.nodes[x].is_black = true;
            } else {
                loop {
                    if !self.is_left_child(w) {
                        // x is a left child
                        if !self.nodes[w].is_black {
                            self.nodes[w].is_black = true;
                            let wp = self.nodes[w].parent;
                            self.nodes[wp].is_black = false;
                            self.left_rotate(wp);
                            if root == self.nodes[w].left {
                                root = w;
                            }
                            w = self.nodes[self.nodes[w].left].right;
                        }
                        let wl = self.nodes[w].left;
                        let wr = self.nodes[w].right;
                        if (wl == NIL || self.nodes[wl].is_black) && (wr == NIL || self.nodes[wr].is_black) {
                            self.nodes[w].is_black = false;
                            x = self.nodes[w].parent;
                            if x == root || !self.nodes[x].is_black {
                                self.nodes[x].is_black = true;
                                break;
                            }
                            let xp = self.nodes[x].parent;
                            w = if self.is_left_child(x) { self.nodes[xp].right } else { self.nodes[xp].left };
                        } else {
                            // w has a red child
                            if wr == NIL || self.nodes[wr].is_black {
                                self.nodes[wl].is_black = true;
                                self.nodes[w].is_black = false;
                                self.right_rotate(w);
                                w = self.nodes[w].parent;
                            }
                            let wp = self.nodes[w].parent;
                            self.nodes[w].is_black = self.nodes[wp].is_black;
                            self.nodes[wp].is_black = true;
                            let wr = self.nodes[w].right;
                            self.nodes[wr].is_black = true;
                            self.left_rotate(wp);
                            break;
                        }
                    } else {
                        if !self.nodes[w].is_black {
                            self.nodes[w].is_black = true;
                            let wp = self.nodes[w].parent;
                            self.nodes[wp].is_black = false;
                            self.right_rotate(wp);
                            if root == self.nodes[w].right {
                                root = w;
                            }
                            w = self.nodes[self.nodes[w].right].left;
                        }
                        let wl = self.nodes[w].left;
                        let wr = self.nodes[w].right;
                        if (wl == NIL || self.nodes[wl].is_black) && (wr == NIL || self.nodes[wr].is_black) {
                            self.nodes[w].is_black = false;
                            x = self.nodes[w].parent;
                            if !self.nodes[x].is_black || x == root {
                                self.nodes[x].is_black = true;
                                break;
                            }
                            let xp = self.nodes[x].parent;
                            w = if self.is_left_child(x) { self.nodes[xp].right } else { self.nodes[xp].left };
                        } else {
                            // w has a red child
                            if wl == NIL || self.nodes[wl].is_black {
                                self.nodes[wr].is_black = true;
                                self.nodes[w].is_black = false;
                                self.left_rotate(w);
                                w = self.nodes[w].parent;
                            }
                            let wp = self.nodes[w].parent;
                            self.nodes[w].is_black = self.nodes[wp].is_black;
                            self.nodes[wp].is_black = true;
                            let wl = self.nodes[w].left;
                            self.nodes[wl].is_black = true;
                            self.right_rotate(wp);
                            break;
                        }
                    }
                }
            }
        }
    }

    /// `lower_bound(key)`: __tree:2140-2150 `__lower_bound(v, root, end_node)`.
    pub fn lower_bound(&self, v: &BeachLineNodeKey) -> usize {
        let mut root = self.root();
        let mut result = Self::END;
        while root != NIL {
            if !node_comparison_predicate(&self.nodes[root].key, v) {
                result = root;
                root = self.nodes[root].left;
            } else {
                root = self.nodes[root].right;
            }
        }
        result
    }

    /// __tree:1679-1709 `__find_equal(parent, v)`: the parent and the side (`true` = left)
    /// of the null leaf where `v` goes, or the node holding an equivalent key.
    fn find_equal(&self, v: &BeachLineNodeKey) -> Result<(usize, bool), usize> {
        let mut nd = self.root();
        if nd != NIL {
            loop {
                if node_comparison_predicate(v, &self.nodes[nd].key) {
                    if self.nodes[nd].left != NIL {
                        nd = self.nodes[nd].left;
                    } else {
                        return Ok((nd, true));
                    }
                } else if node_comparison_predicate(&self.nodes[nd].key, v) {
                    if self.nodes[nd].right != NIL {
                        nd = self.nodes[nd].right;
                    } else {
                        return Ok((nd, false));
                    }
                } else {
                    return Err(nd);
                }
            }
        }
        Ok((Self::END, true))
    }

    /// __tree:1719-1757 `__find_equal(hint, parent, dummy, v)`.
    fn find_equal_hint(&self, hint: usize, v: &BeachLineNodeKey) -> Result<(usize, bool), usize> {
        if hint == Self::END || node_comparison_predicate(v, &self.nodes[hint].key) {
            // v < *hint
            let mut prior = hint;
            if prior == self.begin_node || {
                prior = self.prev(prior);
                node_comparison_predicate(&self.nodes[prior].key, v)
            } {
                // *prev(hint) < v < *hint
                if self.nodes[hint].left == NIL {
                    return Ok((hint, true));
                } else {
                    return Ok((prior, false));
                }
            }
            // v <= *prev(hint)
            count(&BEACH_LINE_HINT_MISSED);
            return self.find_equal(v);
        } else if node_comparison_predicate(&self.nodes[hint].key, v) {
            // *hint < v
            let next = self.next(hint);
            if next == Self::END || node_comparison_predicate(v, &self.nodes[next].key) {
                // *hint < v < *next(hint)
                if self.nodes[hint].right == NIL {
                    return Ok((hint, false));
                } else {
                    return Ok((next, true));
                }
            }
            // *next(hint) <= v
            count(&BEACH_LINE_HINT_MISSED);
            return self.find_equal(v);
        }
        // else v == *hint
        Err(hint)
    }

    /// `insert(hint, value)`: __tree:1794-1808 `__emplace_hint_unique_key_args` and
    /// 1762-1772 `__insert_node_at`. Returns the iterator of the new node, or of the node
    /// that already holds an equivalent key.
    pub fn insert(&mut self, hint: usize, key: BeachLineNodeKey, data: BeachLineNodeData) -> usize {
        let (parent, is_left) = match self.find_equal_hint(hint, &key) {
            Ok(place) => place,
            Err(existing) => {
                count(&BEACH_LINE_KEY_FOUND);
                return existing;
            }
        };
        let node = BeachLineNode { left: NIL, right: NIL, parent, is_black: false, key, data };
        let new_node = match self.free.pop() {
            Some(i) => {
                self.nodes[i] = node;
                i
            }
            None => {
                self.nodes.push(node);
                self.nodes.len() - 1
            }
        };
        if is_left {
            self.nodes[parent].left = new_node;
        } else {
            self.nodes[parent].right = new_node;
        }
        if self.nodes[self.begin_node].left != NIL {
            self.begin_node = self.nodes[self.begin_node].left;
        }
        let root = self.root();
        self.balance_after_insert(root, new_node);
        self.size += 1;
        new_node
    }

    /// `erase(it)`: __tree:1911-1920 `__remove_node_pointer`. Returns the next iterator.
    pub fn erase(&mut self, ptr: usize) -> usize {
        let r = self.next(ptr);
        if self.begin_node == ptr {
            self.begin_node = r;
        }
        self.size -= 1;
        let root = self.root();
        self.tree_remove(root, ptr);
        self.free.push(ptr);
        r
    }
}

// ===========================================================================
// voronoi_diagram.hpp
// ===========================================================================

/// voronoi_diagram.hpp:84-87, 129-132, 244-250: the colour bits.
const BITS_SHIFT: usize = 0x5;
const BITS_MASK: usize = 0x1F;
const BIT_IS_LINEAR: usize = 0x1;
const BIT_IS_PRIMARY: usize = 0x2;

/// voronoi_diagram.hpp:34-92 `voronoi_cell<double>`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VoronoiCell {
    source_index: usize,
    incident_edge: Option<usize>,
    color: usize,
}

impl VoronoiCell {
    /// voronoi_diagram.hpp:43-47.
    pub fn new(source_index: usize, source_category: usize) -> Self {
        VoronoiCell { source_index, incident_edge: None, color: source_category }
    }

    /// voronoi_diagram.hpp:50-53.
    pub fn contains_point(&self) -> bool {
        belongs(self.source_category(), GEOMETRY_CATEGORY_POINT)
    }

    /// voronoi_diagram.hpp:56-59.
    pub fn contains_segment(&self) -> bool {
        belongs(self.source_category(), GEOMETRY_CATEGORY_SEGMENT)
    }

    /// voronoi_diagram.hpp:61-63.
    pub fn source_index(&self) -> usize {
        self.source_index
    }

    /// voronoi_diagram.hpp:65-67.
    pub fn source_category(&self) -> usize {
        self.color & SOURCE_CATEGORY_BITMASK
    }

    /// voronoi_diagram.hpp:70.
    pub fn is_degenerate(&self) -> bool {
        self.incident_edge.is_none()
    }

    /// voronoi_diagram.hpp:72-73: an index into [`VoronoiDiagram::edges`].
    pub fn incident_edge(&self) -> Option<usize> {
        self.incident_edge
    }

    /// voronoi_diagram.hpp:76.
    pub fn color(&self) -> usize {
        self.color >> BITS_SHIFT
    }

    /// voronoi_diagram.hpp:77-80.
    pub fn set_color(&mut self, color: usize) {
        self.color &= BITS_MASK;
        self.color |= color << BITS_SHIFT;
    }
}

/// voronoi_diagram.hpp:99-138 `voronoi_vertex<double>`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VoronoiVertex {
    x: f64,
    y: f64,
    incident_edge: Option<usize>,
    color: usize,
}

impl VoronoiVertex {
    /// voronoi_diagram.hpp:106-110.
    pub fn new(x: f64, y: f64) -> Self {
        VoronoiVertex { x, y, incident_edge: None, color: 0 }
    }

    /// voronoi_diagram.hpp:112.
    pub fn x(&self) -> f64 {
        self.x
    }

    /// voronoi_diagram.hpp:113.
    pub fn y(&self) -> f64 {
        self.y
    }

    /// voronoi_diagram.hpp:115.
    pub fn is_degenerate(&self) -> bool {
        self.incident_edge.is_none()
    }

    /// voronoi_diagram.hpp:117-118: an index into [`VoronoiDiagram::edges`].
    pub fn incident_edge(&self) -> Option<usize> {
        self.incident_edge
    }

    /// voronoi_diagram.hpp:121.
    pub fn color(&self) -> usize {
        self.color >> BITS_SHIFT
    }

    /// voronoi_diagram.hpp:122-125.
    pub fn set_color(&mut self, color: usize) {
        self.color &= BITS_MASK;
        self.color |= color << BITS_SHIFT;
    }
}

/// voronoi_diagram.hpp:149-258 `voronoi_edge<double>`: a half-edge. Every link is an index
/// into the diagram's vectors. What needs another edge (`vertex1`, `rot_next`, `rot_prev`,
/// `is_finite`, `is_infinite`) is a method of [`VoronoiDiagram`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VoronoiEdge {
    cell: Option<usize>,
    vertex: Option<usize>,
    twin: Option<usize>,
    next: Option<usize>,
    prev: Option<usize>,
    color: usize,
}

impl VoronoiEdge {
    /// voronoi_diagram.hpp:158-169.
    pub fn new(is_linear: bool, is_primary: bool) -> Self {
        let mut color = 0;
        if is_linear {
            color |= BIT_IS_LINEAR;
        }
        if is_primary {
            color |= BIT_IS_PRIMARY;
        }
        VoronoiEdge { cell: None, vertex: None, twin: None, next: None, prev: None, color }
    }

    /// voronoi_diagram.hpp:171-172.
    pub fn cell(&self) -> Option<usize> {
        self.cell
    }

    /// voronoi_diagram.hpp:175-176.
    pub fn vertex0(&self) -> Option<usize> {
        self.vertex
    }

    /// voronoi_diagram.hpp:182-183.
    pub fn twin(&self) -> Option<usize> {
        self.twin
    }

    /// voronoi_diagram.hpp:186-187.
    pub fn next(&self) -> Option<usize> {
        self.next
    }

    /// voronoi_diagram.hpp:190-191.
    pub fn prev(&self) -> Option<usize> {
        self.prev
    }

    /// voronoi_diagram.hpp:214-216.
    pub fn is_linear(&self) -> bool {
        self.color & BIT_IS_LINEAR != 0
    }

    /// voronoi_diagram.hpp:220-222.
    pub fn is_curved(&self) -> bool {
        self.color & BIT_IS_LINEAR == 0
    }

    /// voronoi_diagram.hpp:226-228.
    pub fn is_primary(&self) -> bool {
        self.color & BIT_IS_PRIMARY != 0
    }

    /// voronoi_diagram.hpp:232-234.
    pub fn is_secondary(&self) -> bool {
        self.color & BIT_IS_PRIMARY == 0
    }

    /// voronoi_diagram.hpp:236.
    pub fn color(&self) -> usize {
        self.color >> BITS_SHIFT
    }

    /// voronoi_diagram.hpp:237-240.
    pub fn set_color(&mut self, color: usize) {
        self.color &= BITS_MASK;
        self.color |= color << BITS_SHIFT;
    }
}

/// voronoi_diagram.hpp:266-277 `vertex_equality_predicate_type` (`ULPS = 128`).
fn vertex_equality_predicate(v1: &VoronoiVertex, v2: &VoronoiVertex) -> bool {
    const ULPS: u32 = 128;
    ulp_comparison(v1.x, v2.x, ULPS) == UlpResult::Equal && ulp_comparison(v1.y, v2.y, ULPS) == UlpResult::Equal
}

/// voronoi_diagram.hpp:282-616 `voronoi_diagram<double>`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct VoronoiDiagram {
    cells: Vec<VoronoiCell>,
    vertices: Vec<VoronoiVertex>,
    edges: Vec<VoronoiEdge>,
}

impl VoronoiDiagram {
    /// voronoi_diagram.hpp:299.
    pub fn new() -> Self {
        Self::default()
    }

    /// voronoi_diagram.hpp:301-305.
    pub fn clear(&mut self) {
        self.cells.clear();
        self.vertices.clear();
        self.edges.clear();
    }

    /// voronoi_diagram.hpp:307-309.
    pub fn cells(&self) -> &[VoronoiCell] {
        &self.cells
    }

    /// voronoi_diagram.hpp:311-313.
    pub fn vertices(&self) -> &[VoronoiVertex] {
        &self.vertices
    }

    /// voronoi_diagram.hpp:315-317.
    pub fn edges(&self) -> &[VoronoiEdge] {
        &self.edges
    }

    /// voronoi_diagram.hpp:319-321.
    pub fn num_cells(&self) -> usize {
        self.cells.len()
    }

    /// voronoi_diagram.hpp:323-325.
    pub fn num_edges(&self) -> usize {
        self.edges.len()
    }

    /// voronoi_diagram.hpp:327-329.
    pub fn num_vertices(&self) -> usize {
        self.vertices.len()
    }

    /// voronoi_diagram.hpp:179-180 `voronoi_edge::vertex1`.
    pub fn vertex1(&self, edge: usize) -> Option<usize> {
        self.edges[self.edges[edge].twin.expect("voronoi_edge::vertex1: no twin")].vertex
    }

    /// voronoi_diagram.hpp:196-197 `voronoi_edge::rot_next`.
    pub fn rot_next(&self, edge: usize) -> Option<usize> {
        self.edges[self.edges[edge].prev.expect("voronoi_edge::rot_next: no prev")].twin
    }

    /// voronoi_diagram.hpp:201-202 `voronoi_edge::rot_prev`.
    pub fn rot_prev(&self, edge: usize) -> Option<usize> {
        self.edges[self.edges[edge].twin.expect("voronoi_edge::rot_prev: no twin")].next
    }

    /// voronoi_diagram.hpp:206 `voronoi_edge::is_finite`.
    pub fn is_finite(&self, edge: usize) -> bool {
        self.edges[edge].vertex.is_some() && self.vertex1(edge).is_some()
    }

    /// voronoi_diagram.hpp:210 `voronoi_edge::is_infinite`.
    pub fn is_infinite(&self, edge: usize) -> bool {
        self.edges[edge].vertex.is_none() || self.vertex1(edge).is_none()
    }

    /// voronoi_diagram.hpp:331-335 `_reserve`. The C++ relies on these capacities: its
    /// pointers into the vectors must not move. Indices do not care.
    pub fn _reserve(&mut self, num_sites: usize) {
        self.cells.reserve(num_sites);
        self.vertices.reserve(num_sites << 1);
        self.edges.reserve((num_sites << 2) + (num_sites << 1));
    }

    /// voronoi_diagram.hpp:337-340 `_process_single_site`.
    pub fn _process_single_site(&mut self, site: &SiteEvent) {
        self.cells.push(VoronoiCell::new(site.initial_index(), site.source_category()));
    }

    /// voronoi_diagram.hpp:345-385 `_insert_new_edge(site1, site2)`: the two half-edges of a
    /// new bisector.
    pub fn _insert_new_edge(&mut self, site1: &SiteEvent, site2: &SiteEvent) -> (usize, usize) {
        // Get sites' indexes.
        let site_index1 = site1.sorted_index();
        let site_index2 = site2.sorted_index();

        let is_linear = is_linear_edge(site1, site2);
        let is_primary = is_primary_edge(site1, site2);

        // Create a new half-edge that belongs to the first site.
        self.edges.push(VoronoiEdge::new(is_linear, is_primary));
        let edge1 = self.edges.len() - 1;

        // Create a new half-edge that belongs to the second site.
        self.edges.push(VoronoiEdge::new(is_linear, is_primary));
        let edge2 = self.edges.len() - 1;

        // Add the initial cell during the first edge insertion.
        if self.cells.is_empty() {
            self.cells.push(VoronoiCell::new(site1.initial_index(), site1.source_category()));
        }

        // The second site represents a new site during site event
        // processing. Add a new cell to the cell records.
        self.cells.push(VoronoiCell::new(site2.initial_index(), site2.source_category()));

        // Set up pointers to cells.
        self.edges[edge1].cell = Some(site_index1);
        self.edges[edge2].cell = Some(site_index2);

        // Set up twin pointers.
        self.edges[edge1].twin = Some(edge2);
        self.edges[edge2].twin = Some(edge1);

        // Return a pointer to the new half-edge.
        (edge1, edge2)
    }

    /// voronoi_diagram.hpp:393-440 `_insert_new_edge(site1, site3, circle, data12, data23)`:
    /// the new vertex where two bisectors meet and the two half-edges of the bisector that
    /// leaves it.
    pub fn _insert_new_edge_at_circle(
        &mut self,
        site1: &SiteEvent,
        site3: &SiteEvent,
        circle: &CircleEvent,
        edge12: usize,
        edge23: usize,
    ) -> (usize, usize) {
        // Add a new Voronoi vertex.
        self.vertices.push(VoronoiVertex::new(circle.x(), circle.y()));
        let new_vertex = self.vertices.len() - 1;

        // Update vertex pointers of the old edges.
        self.edges[edge12].vertex = Some(new_vertex);
        self.edges[edge23].vertex = Some(new_vertex);

        let is_linear = is_linear_edge(site1, site3);
        let is_primary = is_primary_edge(site1, site3);

        // Add a new half-edge.
        self.edges.push(VoronoiEdge::new(is_linear, is_primary));
        let new_edge1 = self.edges.len() - 1;
        self.edges[new_edge1].cell = Some(site1.sorted_index());

        // Add a new half-edge.
        self.edges.push(VoronoiEdge::new(is_linear, is_primary));
        let new_edge2 = self.edges.len() - 1;
        self.edges[new_edge2].cell = Some(site3.sorted_index());

        // Update twin pointers.
        self.edges[new_edge1].twin = Some(new_edge2);
        self.edges[new_edge2].twin = Some(new_edge1);

        // Update vertex pointer.
        self.edges[new_edge2].vertex = Some(new_vertex);

        // Update Voronoi prev/next pointers.
        self.edges[edge12].prev = Some(new_edge1);
        self.edges[new_edge1].next = Some(edge12);
        let edge12_twin = self.edges[edge12].twin.expect("_insert_new_edge: edge12 has no twin");
        self.edges[edge12_twin].next = Some(edge23);
        self.edges[edge23].prev = self.edges[edge12].twin;
        let edge23_twin = self.edges[edge23].twin.expect("_insert_new_edge: edge23 has no twin");
        self.edges[edge23_twin].next = Some(new_edge2);
        self.edges[new_edge2].prev = self.edges[edge23].twin;

        // Return a pointer to the new half-edge.
        (new_edge1, new_edge2)
    }

    /// voronoi_diagram.hpp:442-550 `_build`: the clean-up after the sweep.
    pub fn _build(&mut self) {
        // Remove degenerate edges.
        let mut last_edge = 0usize;
        let mut it = 0usize;
        while it != self.edges.len() {
            let v1 = self.edges[it].vertex;
            let v2 = self.vertex1(it);
            if v1.is_some() && v2.is_some() && vertex_equality_predicate(&self.vertices[v1.unwrap()], &self.vertices[v2.unwrap()]) {
                count(&DEGENERATE_EDGES_REMOVED);
                self.remove_edge(it);
            } else {
                if it != last_edge {
                    self.edges[last_edge] = self.edges[it];
                    let e1 = last_edge;
                    self.edges[last_edge + 1] = self.edges[it + 1];
                    let e2 = last_edge + 1;
                    self.edges[e1].twin = Some(e2);
                    self.edges[e2].twin = Some(e1);
                    if self.edges[e1].prev.is_some() {
                        let p = self.edges[e1].prev.unwrap();
                        self.edges[p].next = Some(e1);
                        let n = self.edges[e2].next.expect("_build: e2 has no next");
                        self.edges[n].prev = Some(e2);
                    }
                    if self.edges[e2].prev.is_some() {
                        let n = self.edges[e1].next.expect("_build: e1 has no next");
                        self.edges[n].prev = Some(e1);
                        let p = self.edges[e2].prev.expect("_build: e2 has no prev");
                        self.edges[p].next = Some(e2);
                    }
                }
                last_edge += 2;
            }
            it += 2;
        }
        self.edges.truncate(last_edge);

        // Set up incident edge pointers for cells and vertices.
        for it in 0..self.edges.len() {
            let cell = self.edges[it].cell.expect("_build: edge without a cell");
            self.cells[cell].incident_edge = Some(it);
            if let Some(v) = self.edges[it].vertex {
                self.vertices[v].incident_edge = Some(it);
            }
        }

        // Remove degenerate vertices.
        let mut last_vertex = 0usize;
        for it in 0..self.vertices.len() {
            if self.vertices[it].incident_edge.is_some() {
                if it != last_vertex {
                    self.vertices[last_vertex] = self.vertices[it];
                    let v = last_vertex;
                    let mut e = self.vertices[v].incident_edge.unwrap();
                    loop {
                        self.edges[e].vertex = Some(v);
                        e = self.rot_next(e).expect("_build: rot_next is null");
                        if Some(e) == self.vertices[v].incident_edge {
                            break;
                        }
                    }
                }
                last_vertex += 1;
            } else {
                count(&DEGENERATE_VERTICES_REMOVED);
            }
        }
        self.vertices.truncate(last_vertex);

        // Set up next/prev pointers for infinite edges.
        if self.vertices.is_empty() {
            if !self.edges.is_empty() {
                // Update prev/next pointers for the line edges.
                let end = self.edges.len();
                let mut edge_it = 0usize;
                let mut edge1 = edge_it;
                self.edges[edge1].next = Some(edge1);
                self.edges[edge1].prev = Some(edge1);
                edge_it += 1;
                edge1 = edge_it;
                edge_it += 1;

                while edge_it != end {
                    let edge2 = edge_it;
                    edge_it += 1;

                    self.edges[edge1].next = Some(edge2);
                    self.edges[edge1].prev = Some(edge2);
                    self.edges[edge2].next = Some(edge1);
                    self.edges[edge2].prev = Some(edge1);

                    edge1 = edge_it;
                    edge_it += 1;
                }

                self.edges[edge1].next = Some(edge1);
                self.edges[edge1].prev = Some(edge1);
            }
        } else {
            // Update prev/next pointers for the ray edges.
            for cell_it in 0..self.cells.len() {
                if self.cells[cell_it].is_degenerate() {
                    continue;
                }
                let incident = self.cells[cell_it].incident_edge.unwrap();
                // Move to the previous edge while
                // it is possible in the CW direction.
                let mut left_edge = incident;
                while let Some(p) = self.edges[left_edge].prev {
                    left_edge = p;
                    // Terminate if this is not a boundary cell.
                    if left_edge == incident {
                        break;
                    }
                }

                if self.edges[left_edge].prev.is_some() {
                    continue;
                }

                let mut right_edge = incident;
                while let Some(n) = self.edges[right_edge].next {
                    right_edge = n;
                }
                self.edges[left_edge].prev = Some(right_edge);
                self.edges[right_edge].next = Some(left_edge);
            }
        }
    }

    /// voronoi_diagram.hpp:583-606 `remove_edge`: unlink an edge whose two vertices are equal.
    fn remove_edge(&mut self, edge: usize) {
        // Update the endpoints of the incident edges to the second vertex.
        let vertex = self.edges[edge].vertex;
        let twin = self.edges[edge].twin.expect("remove_edge: no twin");
        let mut updated_edge = self.rot_next(twin).expect("remove_edge: rot_next is null");
        while updated_edge != twin {
            self.edges[updated_edge].vertex = vertex;
            updated_edge = self.rot_next(updated_edge).expect("remove_edge: rot_next is null");
        }

        let edge1 = edge;
        let edge2 = twin;

        let edge1_rot_prev = self.rot_prev(edge1).expect("remove_edge: rot_prev is null");
        let edge1_rot_next = self.rot_next(edge1).expect("remove_edge: rot_next is null");

        let edge2_rot_prev = self.rot_prev(edge2).expect("remove_edge: rot_prev is null");
        let edge2_rot_next = self.rot_next(edge2).expect("remove_edge: rot_next is null");

        // Update prev/next pointers for the incident edges.
        let t = self.edges[edge1_rot_next].twin.expect("remove_edge: no twin");
        self.edges[t].next = Some(edge2_rot_prev);
        self.edges[edge2_rot_prev].prev = self.edges[edge1_rot_next].twin;
        self.edges[edge1_rot_prev].prev = self.edges[edge2_rot_next].twin;
        let t = self.edges[edge2_rot_next].twin.expect("remove_edge: no twin");
        self.edges[t].next = Some(edge1_rot_prev);
    }
}

/// voronoi_diagram.hpp:559-572 `is_primary_edge`.
fn is_primary_edge(site1: &SiteEvent, site2: &SiteEvent) -> bool {
    let flag1 = site1.is_segment();
    let flag2 = site2.is_segment();
    if flag1 && !flag2 {
        return (site1.point0() != site2.point0()) && (site1.point1() != site2.point0());
    }
    if !flag1 && flag2 {
        return (site2.point0() != site1.point0()) && (site2.point1() != site1.point0());
    }
    true
}

/// voronoi_diagram.hpp:574-580 `is_linear_edge`.
fn is_linear_edge(site1: &SiteEvent, site2: &SiteEvent) -> bool {
    if !is_primary_edge(site1, site2) {
        return true;
    }
    !(site1.is_segment() ^ site2.is_segment())
}

// ===========================================================================
// voronoi_builder.hpp
// ===========================================================================

/// `end_point_type` (voronoi_builder.hpp:168): the far endpoint of a segment and the
/// temporary beach line node that goes when the sweep reaches it.
type EndPointType = (Point2D, usize);

/// voronoi_builder.hpp:495-501 `end_point_comparison`.
fn end_point_comparison(end1: &EndPointType, end2: &EndPointType) -> bool {
    point_comparison(end2.0, end1.0)
}

/// voronoi_builder.hpp:48-515 `voronoi_builder<int32>`: the sweep line.
#[derive(Clone, Debug, Default)]
pub struct VoronoiBuilder {
    site_events: Vec<SiteEvent>,
    /// `site_event_iterator_`: an index into `site_events`.
    site_event_iterator: usize,
    /// `end_points_`: a `std::priority_queue`, kept as libc++'s heap.
    end_points: Vec<EndPointType>,
    circle_events: OrderedQueue,
    beach_line: BeachLine,
    circle_formation_predicate: CircleFormationPredicate,
    index: usize,
}

impl VoronoiBuilder {
    /// voronoi_builder.hpp:56.
    pub fn new() -> Self {
        Self::default()
    }

    /// voronoi_builder.hpp:59-64 `insert_point`.
    pub fn insert_point(&mut self, x: i32, y: i32) -> usize {
        let mut site = SiteEvent::from_point(Point2D::new(x, y));
        site.set_initial_index(self.index);
        site.set_source_category(SOURCE_CATEGORY_SINGLE_POINT);
        self.site_events.push(site);
        let index = self.index;
        self.index += 1;
        index
    }

    /// voronoi_builder.hpp:70-95 `insert_segment`.
    pub fn insert_segment(&mut self, x1: i32, y1: i32, x2: i32, y2: i32) -> usize {
        // Set up start point site.
        let p1 = Point2D::new(x1, y1);
        let mut site = SiteEvent::from_point(p1);
        site.set_initial_index(self.index);
        site.set_source_category(SOURCE_CATEGORY_SEGMENT_START_POINT);
        self.site_events.push(site);

        // Set up end point site.
        let p2 = Point2D::new(x2, y2);
        let mut site = SiteEvent::from_point(p2);
        site.set_initial_index(self.index);
        site.set_source_category(SOURCE_CATEGORY_SEGMENT_END_POINT);
        self.site_events.push(site);

        // Set up segment site.
        let mut site;
        if point_comparison(p1, p2) {
            site = SiteEvent::from_points(p1, p2);
            site.set_source_category(SOURCE_CATEGORY_INITIAL_SEGMENT);
        } else {
            site = SiteEvent::from_points(p2, p1);
            site.set_source_category(SOURCE_CATEGORY_REVERSE_SEGMENT);
        }
        site.set_initial_index(self.index);
        self.site_events.push(site);
        let index = self.index;
        self.index += 1;
        index
    }

    /// voronoi_builder.hpp:98-130 `construct`.
    pub fn construct(&mut self, output: &mut VoronoiDiagram) {
        // Init structures.
        output._reserve(self.site_events.len());
        self.init_sites_queue();
        self.init_beach_line(output);

        // The algorithm stops when there are no events to process.
        while !self.circle_events.empty() || !(self.site_event_iterator == self.site_events.len()) {
            if self.circle_events.empty() {
                self.process_site_event(output);
            } else if self.site_event_iterator == self.site_events.len() {
                self.process_circle_event(output);
            } else {
                if event_comparison_sc(&self.site_events[self.site_event_iterator], &self.circle_events.top().0) {
                    self.process_site_event(output);
                } else {
                    self.process_circle_event(output);
                }
            }
            while !self.circle_events.empty() && !self.circle_events.top().0.is_active() {
                self.circle_events.pop();
            }
        }
        self.beach_line.clear();

        // Finish construction.
        output._build();
    }

    /// voronoi_builder.hpp:132-135 `clear`.
    pub fn clear(&mut self) {
        self.index = 0;
        self.site_events.clear();
    }

    /// voronoi_builder.hpp:170-186 `init_sites_queue`.
    fn init_sites_queue(&mut self) {
        // Sort site events.
        libcxx_sort(&mut self.site_events, event_comparison_ss);

        // Remove duplicates: `std::unique` keeps the first of each run.
        self.site_events.dedup_by(|later, first| *first == *later);

        // Index sites.
        for cur in 0..self.site_events.len() {
            self.site_events[cur].set_sorted_index(cur);
        }

        // Init site iterator.
        self.site_event_iterator = 0;
    }

    /// voronoi_builder.hpp:188-215 `init_beach_line`.
    fn init_beach_line(&mut self, output: &mut VoronoiDiagram) {
        if self.site_events.is_empty() {
            return;
        }
        if self.site_events.len() == 1 {
            // Handle single site event case.
            output._process_single_site(&self.site_events[0]);
            self.site_event_iterator += 1;
        } else {
            let mut skip = 0;

            while self.site_event_iterator != self.site_events.len()
                && is_vertical_points(self.site_events[self.site_event_iterator].point0(), self.site_events[0].point0())
                && is_vertical(&self.site_events[self.site_event_iterator])
            {
                self.site_event_iterator += 1;
                skip += 1;
            }

            if skip == 1 {
                // Init beach line with the first two sites.
                self.init_beach_line_default(output);
            } else {
                // Init beach line with collinear vertical sites.
                self.init_beach_line_collinear_sites(output);
            }
        }
    }

    /// voronoi_builder.hpp:219-229 `init_beach_line_default`.
    fn init_beach_line_default(&mut self, output: &mut VoronoiDiagram) {
        // Get the first and the second site event.
        let first = self.site_events[0];
        let second = self.site_events[1];
        self.insert_new_arc(&first, &first, &second, BeachLine::END, output);
        // The second site was already processed. Move the iterator.
        self.site_event_iterator += 1;
    }

    /// voronoi_builder.hpp:232-252 `init_beach_line_collinear_sites`.
    fn init_beach_line_collinear_sites(&mut self, output: &mut VoronoiDiagram) {
        count(&COLLINEAR_INIT);
        let mut it_first = 0usize;
        let mut it_second = 1usize;
        while it_second != self.site_event_iterator {
            // Create a new beach line node.
            let new_node = BeachLineNodeKey::new2(self.site_events[it_first], self.site_events[it_second]);

            // Update the output.
            let edge = output._insert_new_edge(&self.site_events[it_first], &self.site_events[it_second]).0;

            // Insert a new bisector into the beach line.
            self.beach_line.insert(BeachLine::END, new_node, BeachLineNodeData::new(Some(edge)));

            // Update iterators.
            it_first += 1;
            it_second += 1;
        }
    }

    /// voronoi_builder.hpp:254-259 `deactivate_circle_event`, on the value of a beach line node.
    fn deactivate_circle_event(&mut self, node: usize) {
        if let Some(circle_event) = self.beach_line.data(node).circle_event {
            self.circle_events.get_mut(circle_event).0.deactivate();
            self.beach_line.data_mut(node).circle_event = None;
        }
    }

    /// voronoi_builder.hpp:261-362 `process_site_event`.
    fn process_site_event(&mut self, output: &mut VoronoiDiagram) {
        // Get next site event to process.
        let mut site_event = self.site_events[self.site_event_iterator];

        // Move site iterator.
        let mut last = self.site_event_iterator + 1;

        // If a new site is an end point of some segment,
        // remove temporary nodes from the beach line data structure.
        if !site_event.is_segment() {
            while !self.end_points.is_empty() && self.end_points[0].0 == site_event.point0() {
                let b_it = self.end_points[0].1;
                libcxx_pop_heap(&mut self.end_points, &mut end_point_comparison);
                self.end_points.pop();
                self.beach_line.erase(b_it);
            }
        } else {
            while last != self.site_events.len()
                && self.site_events[last].is_segment()
                && self.site_events[last].point0() == site_event.point0()
            {
                last += 1;
            }
        }

        // Find the node in the binary search tree with left arc
        // lying above the new site point.
        let new_key = BeachLineNodeKey::new(self.site_events[self.site_event_iterator]);
        let mut right_it = self.beach_line.lower_bound(&new_key);

        while self.site_event_iterator != last {
            site_event = self.site_events[self.site_event_iterator];
            let mut left_it = right_it;

            // Do further processing depending on the above node position.
            // For any two neighboring nodes the second site of the first node
            // is the same as the first site of the second node.
            if right_it == BeachLine::END {
                // The above arc corresponds to the second arc of the last node.
                // Move the iterator to the last node.
                left_it = self.beach_line.prev(left_it);

                // Get the second site of the last node
                let site_arc = *self.beach_line.key(left_it).right_site();

                // Insert new nodes into the beach line. Update the output.
                right_it = self.insert_new_arc(&site_arc, &site_arc, &site_event, right_it, output);

                // Add a candidate circle to the circle event queue.
                // There could be only one new circle event formed by
                // a new bisector and the one on the left.
                let left_key = *self.beach_line.key(left_it);
                self.activate_circle_event(left_key.left_site(), left_key.right_site(), &site_event, right_it);
            } else if right_it == self.beach_line.begin() {
                // The above arc corresponds to the first site of the first node.
                let site_arc = *self.beach_line.key(right_it).left_site();

                // Insert new nodes into the beach line. Update the output.
                left_it = self.insert_new_arc(&site_arc, &site_arc, &site_event, right_it, output);

                // If the site event is a segment, update its direction.
                if site_event.is_segment() {
                    site_event.inverse();
                }

                // Add a candidate circle to the circle event queue.
                // There could be only one new circle event formed by
                // a new bisector and the one on the right.
                let right_key = *self.beach_line.key(right_it);
                self.activate_circle_event(&site_event, right_key.left_site(), right_key.right_site(), right_it);
                right_it = left_it;
            } else {
                // The above arc corresponds neither to the first,
                // nor to the last site in the beach line.
                let site_arc2 = *self.beach_line.key(right_it).left_site();
                let site3 = *self.beach_line.key(right_it).right_site();

                // Remove the candidate circle from the event queue.
                self.deactivate_circle_event(right_it);
                left_it = self.beach_line.prev(left_it);
                let site_arc1 = *self.beach_line.key(left_it).right_site();
                let site1 = *self.beach_line.key(left_it).left_site();

                // Insert new nodes into the beach line. Update the output.
                let new_node_it = self.insert_new_arc(&site_arc1, &site_arc2, &site_event, right_it, output);

                // Add candidate circles to the circle event queue.
                // There could be up to two circle events formed by
                // a new bisector and the one on the left or right.
                self.activate_circle_event(&site1, &site_arc1, &site_event, new_node_it);

                // If the site event is a segment, update its direction.
                if site_event.is_segment() {
                    site_event.inverse();
                }
                self.activate_circle_event(&site_event, &site_arc2, &site3, right_it);
                right_it = new_node_it;
            }
            self.site_event_iterator += 1;
        }
    }

    /// voronoi_builder.hpp:373-430 `process_circle_event`. The key of the (A, B) node is
    /// changed in place to (A, C), as the C++ does through `const_cast`; the node keeps its
    /// place in the tree.
    fn process_circle_event(&mut self, output: &mut VoronoiDiagram) {
        // Get the topmost circle event.
        let e = *self.circle_events.top();
        let circle_event = e.0;
        let mut it_first = e.1;
        let mut it_last = it_first;

        // Get the C site.
        let mut site3 = *self.beach_line.key(it_first).right_site();

        // Get the half-edge corresponding to the second bisector - (B, C).
        let bisector2 = self.beach_line.data(it_first).edge;

        // Get the half-edge corresponding to the first bisector - (A, B).
        it_first = self.beach_line.prev(it_first);
        let bisector1 = self.beach_line.data(it_first).edge;

        // Get the A site.
        let site1 = *self.beach_line.key(it_first).left_site();

        if !site1.is_segment() && site3.is_segment() && site3.point1() == site1.point0() {
            site3.inverse();
        }

        // Change the (A, B) bisector node to the (A, C) bisector node.
        self.beach_line.key_mut(it_first).right_site = site3;

        // Insert the new bisector into the beach line.
        let new_edge = output
            ._insert_new_edge_at_circle(
                &site1,
                &site3,
                &circle_event,
                bisector1.expect("process_circle_event: the (A, B) bisector has no edge"),
                bisector2.expect("process_circle_event: the (B, C) bisector has no edge"),
            )
            .0;
        self.beach_line.data_mut(it_first).edge = Some(new_edge);

        // Remove the (B, C) bisector node from the beach line.
        self.beach_line.erase(it_last);
        it_last = it_first;

        // Pop the topmost circle event from the event queue.
        self.circle_events.pop();
        if !self.circle_events.empty() {
            let top = &self.circle_events.top().0;
            if top.lower_x() == circle_event.lower_x() && top.y() == circle_event.y() {
                count(&CIRCLE_EVENT_TIES);
            }
        }

        // Check new triplets formed by the neighboring arcs
        // to the left for potential circle events.
        if it_first != self.beach_line.begin() {
            self.deactivate_circle_event(it_first);
            it_first = self.beach_line.prev(it_first);
            let site_l1 = *self.beach_line.key(it_first).left_site();
            self.activate_circle_event(&site_l1, &site1, &site3, it_last);
        }

        // Check the new triplet formed by the neighboring arcs
        // to the right for potential circle events.
        it_last = self.beach_line.next(it_last);
        if it_last != BeachLine::END {
            self.deactivate_circle_event(it_last);
            let site_r1 = *self.beach_line.key(it_last).right_site();
            self.activate_circle_event(&site1, &site3, &site_r1, it_last);
        }
    }

    /// voronoi_builder.hpp:433-472 `insert_new_arc`.
    fn insert_new_arc(
        &mut self,
        site_arc1: &SiteEvent,
        site_arc2: &SiteEvent,
        site_event: &SiteEvent,
        mut position: usize,
        output: &mut VoronoiDiagram,
    ) -> usize {
        // Create two new bisectors with opposite directions.
        let new_left_node = BeachLineNodeKey::new2(*site_arc1, *site_event);
        let mut new_right_node = BeachLineNodeKey::new2(*site_event, *site_arc2);

        // Set correct orientation for the first site of the second node.
        if site_event.is_segment() {
            new_right_node.left_site.inverse();
        }

        // Update the output.
        let edges = output._insert_new_edge(site_arc2, site_event);
        position = self.beach_line.insert(position, new_right_node, BeachLineNodeData::new(Some(edges.1)));

        if site_event.is_segment() {
            // Update the beach line with temporary bisector, that will
            // disappear after processing site event corresponding to the
            // second endpoint of the segment site.
            let mut new_node = BeachLineNodeKey::new2(*site_event, *site_event);
            new_node.right_site.inverse();
            position = self.beach_line.insert(position, new_node, BeachLineNodeData::new(None));

            // Update the data structure that holds temporary bisectors.
            self.end_points.push((site_event.point1(), position));
            libcxx_push_heap(&mut self.end_points, &mut end_point_comparison);
        }

        position = self.beach_line.insert(position, new_left_node, BeachLineNodeData::new(Some(edges.0)));

        position
    }

    /// voronoi_builder.hpp:476-491 `activate_circle_event`: `bisector_node` is the
    /// (site2, site3) bisector.
    fn activate_circle_event(&mut self, site1: &SiteEvent, site2: &SiteEvent, site3: &SiteEvent, bisector_node: usize) {
        let mut c_event = CircleEvent::default();
        // Check if the three input sites create a circle event.
        if self.circle_formation_predicate.call(site1, site2, site3, &mut c_event) {
            // Add the new circle event to the circle events queue.
            // Update bisector's circle event iterator to point to the
            // new circle event in the circle event queue.
            let e = self.circle_events.push((c_event, bisector_node));
            self.beach_line.data_mut(bisector_node).circle_event = Some(e);
        }
    }
}

// ===========================================================================
// voronoi.hpp
// ===========================================================================

/// voronoi.hpp:112-127 `construct_voronoi(first, last, vd)` for segments
/// `[x1, y1, x2, y2]` (`low` then `high`, voronoi.hpp:57-69).
pub fn construct_voronoi(segments: &[[i32; 4]], vd: &mut VoronoiDiagram) {
    let mut builder = VoronoiBuilder::new();
    for s in segments {
        builder.insert_segment(s[0], s[1], s[2], s[3]);
    }
    builder.construct(vd);
}

/// voronoi.hpp:91-110 `construct_voronoi(first, last, vd)` for points `[x, y]`.
pub fn construct_voronoi_points(points: &[[i32; 2]], vd: &mut VoronoiDiagram) {
    let mut builder = VoronoiBuilder::new();
    for p in points {
        builder.insert_point(p[0], p[1]);
    }
    builder.construct(vd);
}

/// voronoi.hpp:129-153 `construct_voronoi(p_first, p_last, s_first, s_last, vd)`.
pub fn construct_voronoi_points_segments(points: &[[i32; 2]], segments: &[[i32; 4]], vd: &mut VoronoiDiagram) {
    let mut builder = VoronoiBuilder::new();
    for p in points {
        builder.insert_point(p[0], p[1]);
    }
    for s in segments {
        builder.insert_segment(s[0], s[1], s[2], s[3]);
    }
    builder.construct(vd);
}
