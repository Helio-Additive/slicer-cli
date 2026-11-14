//! `include/clipper2/clipper.engine.h` and `src/clipper.engine.cpp` of Clipper2 as the
//! engine pins it (version 1.5.2, `src/clipper2/Clipper2Lib`): the polygon clipping module,
//! `ClipperBase` and `Clipper64`. Compiled twice, see `mod.rs`; `super::clipper2_build` is
//! the build file that stands for `USINGZ`, and `if USINGZ { .. }` is an `#ifdef USINGZ` arm.
//!
//! The C++ links its objects with pointers: vertices, local minima, edges (`Active`),
//! `OutRec` and `OutPt`. Here each kind lives in a `Vec` that `ClipperBase` owns and a
//! pointer is an index into it, `NONE` for `nullptr`. An object the C++ deletes stays in its
//! `Vec` until `clean_up`, unlinked and never looked at again, so that comparing two
//! pointers is comparing two indices. `outrec_list_` is the `Vec` of the `OutRec`s itself:
//! an `OutRec*` is its `idx`.
//!
//! FIDELITY-NOTE: an index is never used again for another object, where the allocator of
//! the C++ may hand the address of a deleted `Active` or `OutPt` to a new one. The C++ would
//! see that only if it compared a pointer it kept to a deleted object with a live one, which
//! the code read here does not do (it clears `front_edge`, `back_edge` and `outrec` when an
//! edge leaves, and tests `OutPt::horz` for null only); the parity tests found no case of it.
//!
//! Line numbers are those of the pinned files. Floating point follows the reference build
//! (Apple clang, arm64, `-ffp-contract=on`), see `clipper_core.rs`.
//!
//! `tests/clipper2_engine_native_parity.rs` and `tests/clipper2_engine_z_native_parity.rs`
//! hold `Clipper64::execute` bit-equal to the pinned C++ in both builds. `ClipperStats`
//! exists for them.
//!
//! Not here: `ClipperD`, `PolyPathD`, `BuildPathD` (clipper.engine.h:397-468, :531-642,
//! clipper.engine.cpp:3048-3152) and `ReuseableDataContainer64` (clipper.engine.h:182-192,
//! clipper.engine.cpp:726-751, :850-862), which `Clipper64::Execute` does not run.

use super::clipper2_build::{point_set_z, point_xyz, point_z};
use super::clipper_core::*;
use std::collections::BinaryHeap;

/// `nullptr` of every index that stands for a pointer.
pub const NONE: usize = usize::MAX;

/// clipper.engine.h:33
/// C++: enum class ClipType { NoClip, Intersection, Union, Difference, Xor };
/// Note: all clipping operations except for Difference are commutative.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClipType {
    NoClip,
    Intersection,
    Union,
    Difference,
    Xor,
}

/// clipper.engine.h:35
/// C++: enum class PathType { Subject, Clip };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathType {
    Subject,
    Clip,
}

/// clipper.engine.h:36
/// C++: enum class JoinWith { NoJoin, Left, Right };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JoinWith {
    NoJoin,
    Left,
    Right,
}

/// clipper.engine.h:38-50
/// C++: enum class VertexFlags : uint32_t { Empty = 0, OpenStart = 1, OpenEnd = 2, LocalMax = 4, LocalMin = 8 };
/// with `operator &` and `operator |`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct VertexFlags(pub u32);

impl VertexFlags {
    pub const EMPTY: VertexFlags = VertexFlags(0);
    pub const OPEN_START: VertexFlags = VertexFlags(1);
    pub const OPEN_END: VertexFlags = VertexFlags(2);
    pub const LOCAL_MAX: VertexFlags = VertexFlags(4);
    pub const LOCAL_MIN: VertexFlags = VertexFlags(8);
}

impl std::ops::BitAnd for VertexFlags {
    type Output = VertexFlags;
    fn bitand(self, b: VertexFlags) -> VertexFlags {
        VertexFlags(self.0 & b.0)
    }
}

impl std::ops::BitOr for VertexFlags {
    type Output = VertexFlags;
    fn bitor(self, b: VertexFlags) -> VertexFlags {
        VertexFlags(self.0 | b.0)
    }
}

/// clipper.engine.h:52-57
/// C++: struct Vertex
#[derive(Debug, Clone, Copy)]
pub struct Vertex {
    pub pt: Point64,
    pub next: usize,
    pub prev: usize,
    pub flags: VertexFlags,
}

impl Default for Vertex {
    fn default() -> Self {
        Vertex { pt: Point64::default(), next: NONE, prev: NONE, flags: VertexFlags::EMPTY }
    }
}

/// clipper.engine.h:59-70
/// C++: struct OutPt
/// `horz` is a `HorzSegment*` in the C++ that is only ever tested for null (it is left
/// pointing into `horz_seg_list_` after that list is sorted and cleared): a flag here.
#[derive(Debug, Clone, Copy)]
pub struct OutPt {
    pub pt: Point64,
    pub next: usize,
    pub prev: usize,
    pub outrec: usize,
    pub horz: bool,
}

/// clipper.engine.h:79 `typedef std::vector<OutRec*> OutRecList;`
pub type OutRecList = Vec<usize>;

/// clipper.engine.h:83-101
/// C++: struct OutRec
/// OutRec: contains a path in the clipping solution. Edges in the AEL will
/// have OutRec pointers assigned when they form part of the clipping solution.
#[derive(Debug, Clone)]
pub struct OutRec {
    pub idx: usize,
    pub owner: usize,
    pub front_edge: usize,
    pub back_edge: usize,
    pub pts: usize,
    /// `PolyPath* polypath`: the number of the node in the tree being built.
    pub polypath: usize,
    /// `OutRecList* splits`: `None` is `nullptr`.
    pub splits: Option<OutRecList>,
    pub recursive_split: usize,
    pub bounds: Rect64,
    pub path: Path64,
    pub is_open: bool,
}

/// clipper.engine.h:108-133
/// C++: struct Active
/// Important: UP and DOWN here are premised on Y-axis positive down
/// displays, which is the orientation used in Clipper's development.
#[derive(Debug, Clone, Copy)]
pub struct Active {
    pub bot: Point64,
    pub top: Point64,
    pub curr_x: i64, //current (updated at every new scanline)
    pub dx: f64,
    pub wind_dx: i32, //1 or -1 depending on winding direction
    pub wind_cnt: i32,
    pub wind_cnt2: i32, //winding count of the opposite polytype
    pub outrec: usize,
    //AEL: 'active edge list' (Vatti's AET - active edge table)
    //     a linked list of all edges (from left to right) that are present
    //     (or 'active') within the current scanbeam (a horizontal 'beam' that
    //     sweeps from bottom to top over the paths in the clipping operation).
    pub prev_in_ael: usize,
    pub next_in_ael: usize,
    //SEL: 'sorted edge list' (Vatti's ST - sorted table)
    //     linked list used when sorting edges into their new positions at the
    //     top of scanbeams, but also (re)used to process horizontals.
    pub prev_in_sel: usize,
    pub next_in_sel: usize,
    pub jump: usize,
    pub vertex_top: usize,
    pub local_min: usize, // the bottom of an edge 'bound' (also Vatti)
    pub is_left_bound: bool,
    pub join_with: JoinWith,
}

impl Default for Active {
    fn default() -> Self {
        Active {
            bot: Point64::default(),
            top: Point64::default(),
            curr_x: 0,
            dx: 0.0,
            wind_dx: 1,
            wind_cnt: 0,
            wind_cnt2: 0,
            outrec: NONE,
            prev_in_ael: NONE,
            next_in_ael: NONE,
            prev_in_sel: NONE,
            next_in_sel: NONE,
            jump: NONE,
            vertex_top: NONE,
            local_min: NONE,
            is_left_bound: false,
            join_with: JoinWith::NoJoin,
        }
    }
}

/// clipper.engine.h:135-141
/// C++: struct LocalMinima
#[derive(Debug, Clone, Copy)]
pub struct LocalMinima {
    pub vertex: usize,
    pub polytype: PathType,
    pub is_open: bool,
}

/// clipper.engine.h:143-150
/// C++: struct IntersectNode
#[derive(Debug, Clone, Copy)]
pub struct IntersectNode {
    pub pt: Point64,
    pub edge1: usize,
    pub edge2: usize,
}

/// clipper.engine.h:152-158
/// C++: struct HorzSegment
#[derive(Debug, Clone, Copy)]
pub struct HorzSegment {
    pub left_op: usize,
    pub right_op: usize,
    pub left_to_right: bool,
}

/// clipper.engine.h:160-165
/// C++: struct HorzJoin
#[derive(Debug, Clone, Copy)]
pub struct HorzJoin {
    pub op1: usize,
    pub op2: usize,
}

/// How often one run went through the rarer parts of the sweep. Diagnostics the parity
/// tests read to know their inputs reach those parts; they change nothing.
#[derive(Debug, Clone, Copy, Default)]
pub struct ClipperStats {
    /// Joins `ProcessHorzJoins` went through.
    pub horz_joins: u64,
    /// Those of them that split one path in two.
    pub horz_join_splits: u64,
    /// `DoSplitOp` calls, each one a self-intersection `FixSelfIntersects` found.
    pub split_ops: u64,
    /// `DoSplitOp` calls that made a new `OutRec` of the part cut off.
    pub split_new_outrecs: u64,
    /// `Split` calls: joined edges taken apart.
    pub edge_splits: u64,
    /// `CheckJoinLeft` and `CheckJoinRight` calls that joined two edges.
    pub edge_joins: u64,
    /// Intersection points `AddNewIntersectNode` had to bring back into the scanbeam.
    pub intersect_corrections: u64,
    /// `Path1InsidePath2` calls that came down to the midpoint test.
    pub path1_inside_path2_midpoint: u64,
    /// `CheckSplitOwner` calls that found the owner among the splits.
    pub split_owners_found: u64,
    /// `AddLocalMaxPoly` calls that failed the clipping (`succeeded_ = false`).
    pub failures: u64,
    /// Points `CleanCollinear` removed.
    pub collinear_removed: u64,
    /// `CheckSplitOwner` loops whose `splits` vector changed while they ran.
    pub splits_changed_in_loop: u64,
    /// The longest list `ProcessIntersectList` sorted (the nodes of one scanbeam).
    pub intersect_list_max: u64,
    /// Nodes `ProcessIntersectList` took out of their sorted place because their edges
    /// were not adjacent.
    pub intersect_reorders: u64,
    /// `DoHorizontal` calls.
    pub horizontals: u64,
}

/// clipper.engine.cpp:123-132
/// C++: inline double GetDx(const Point64& pt1, const Point64& pt2)
///  Dx:                             0(90deg)
///                                  |
///               +inf (180deg) <--- o ---> -inf (0deg)
#[inline]
fn get_dx(pt1: &Point64, pt2: &Point64) -> f64 {
    let dy = (pt2.y - pt1.y) as f64;
    if dy != 0.0 {
        (pt2.x - pt1.x) as f64 / dy
    } else if pt2.x > pt1.x {
        -f64::MAX
    } else {
        f64::MAX
    }
}

/// clipper.engine.cpp:134-141
/// C++: inline int64_t TopX(const Active& ae, const int64_t currentY)
///
/// FIDELITY-NOTE: `DoHorizontal` (clipper.engine.cpp:2627-2638) calls this with the y of the
/// vertex after a horizontal edge, which can lie far outside the edge `ae`: the edge is
/// extrapolated. With coordinates near `MAX_COORD` and a flat `ae` the product then leaves
/// the range of `int64_t`, or the sum does. Both are undefined behaviour in the C++
/// (`static_cast<int64_t>` of a double out of range, signed overflow of `+`). What is
/// reproduced here is what the reference build (clang, arm64) does: the conversion
/// saturates (`fcvtzs`, as Rust's `as i64` does everywhere) and the addition wraps. On
/// x86-64 the conversion of the C++ gives `INT64_MIN` instead; the result is only compared
/// with the x of that vertex. `tests/clipper2_engine_native_parity.rs` (huge_coordinates)
/// reaches it.
#[inline]
fn top_x(ae: &Active, current_y: i64) -> i64 {
    if (current_y == ae.top.y) || (ae.top.x == ae.bot.x) {
        ae.top.x
    } else if current_y == ae.bot.y {
        ae.bot.x
    } else {
        // nearbyint: to the nearest integer, halves to even (the default rounding mode).
        // `current_y - ae.bot.y` cannot overflow: both are coordinates of the input.
        ae.bot.x.wrapping_add((ae.dx * ((current_y - ae.bot.y) as f64)).round_ties_even() as i64)
    }
    // nb: std::nearbyint (or std::round) substantially *improves* performance here
    // as it greatly improves the likelihood of edge adjacency in ProcessIntersectList().
}

/// clipper.engine.cpp:144-147
/// C++: inline bool IsHorizontal(const Active& e)
#[inline]
fn is_horizontal(e: &Active) -> bool {
    e.top.y == e.bot.y
}

/// clipper.engine.cpp:150-153
/// C++: inline bool IsHeadingRightHorz(const Active& e)
#[inline]
fn is_heading_right_horz(e: &Active) -> bool {
    e.dx == -f64::MAX
}

/// clipper.engine.cpp:156-159
/// C++: inline bool IsHeadingLeftHorz(const Active& e)
#[inline]
fn is_heading_left_horz(e: &Active) -> bool {
    e.dx == f64::MAX
}

/// clipper.engine.cpp:179-182
/// C++: inline void SetDx(Active& e)
#[inline]
fn set_dx(e: &mut Active) {
    e.dx = get_dx(&e.bot, &e.top);
}

/// clipper.engine.cpp:68-71
/// C++: inline bool IsOdd(int val)
#[inline]
fn is_odd(val: i32) -> bool {
    val & 1 != 0
}

/// clipper.engine.cpp:329-333
/// C++: bool IntersectListSort(const IntersectNode& a, const IntersectNode& b)
fn intersect_list_sort(a: &IntersectNode, b: &IntersectNode) -> bool {
    //note different inequality tests ...
    if a.pt.y == b.pt.y {
        a.pt.x < b.pt.x
    } else {
        a.pt.y > b.pt.y
    }
}

/// clipper.engine.cpp:387-393
/// C++: inline double AreaTriangle(const Point64& pt1, const Point64& pt2, const Point64& pt3)
/// `a * b + c * d + e * f` in one expression: fma(e, f, fma(a, b, c * d)).
#[inline]
fn area_triangle(pt1: &Point64, pt2: &Point64, pt3: &Point64) -> f64 {
    let ab_cd = ((pt3.y + pt1.y) as f64).mul_add((pt3.x - pt1.x) as f64, ((pt1.y + pt2.y) as f64) * ((pt1.x - pt2.x) as f64));
    ((pt2.y + pt3.y) as f64).mul_add((pt2.x - pt3.x) as f64, ab_cd)
}

/// clipper.engine.cpp:443-446
/// C++: inline bool PtsReallyClose(const Point64& pt1, const Point64& pt2)
#[inline]
fn pts_really_close(pt1: &Point64, pt2: &Point64) -> bool {
    ((pt1.x - pt2.x).abs() < 2) && ((pt1.y - pt2.y).abs() < 2)
}

/// clipper.engine.cpp:2164-2180
/// C++: inline bool SetHorzSegHeadingForward(HorzSegment& hs, OutPt* opP, OutPt* opN)
#[inline]
fn set_horz_seg_heading_forward(hs: &mut HorzSegment, op_p: usize, op_n: usize, op_p_x: i64, op_n_x: i64) -> bool {
    if op_p_x == op_n_x {
        return false;
    }
    if op_p_x < op_n_x {
        hs.left_op = op_p;
        hs.right_op = op_n;
        hs.left_to_right = true;
    } else {
        hs.left_op = op_n;
        hs.right_op = op_p;
        hs.left_to_right = false;
    }
    true
}

/// clipper.engine.h:302-342 `class PolyPath` and clipper.engine.h:347-395
/// `class PolyPath64 : public PolyPath`.
/// PolyTree: is intended as a READ-ONLY data structure for CLOSED paths returned
/// by clipping operations. While this structure is more complex than the
/// alternative Paths structure, it does preserve path 'ownership' - ie those
/// paths that contain (or own) other paths. This will be useful to some users.
///
/// A node owns its children (`std::vector<std::unique_ptr<PolyPath64>> childs_`). The
/// C++ node also points at its parent, and `Level()` counts the parents; here the node
/// holds its level.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PolyPath64 {
    level_: u32,
    childs_: Vec<PolyPath64>,
    polygon_: Path64,
}

/// clipper.engine.h:75 `using PolyTree64 = PolyPath64;`
pub type PolyTree64 = PolyPath64;

impl PolyPath64 {
    /// clipper.engine.h:352
    /// C++: explicit PolyPath64(PolyPath64* parent = nullptr) : PolyPath(parent) {}
    pub fn new() -> Self {
        PolyPath64::default()
    }

    /// clipper.engine.h:312-318
    /// C++: unsigned Level() const
    pub fn level(&self) -> u32 {
        self.level_
    }

    /// clipper.engine.h:327-332
    /// C++: bool IsHole() const
    pub fn is_hole(&self) -> bool {
        let lvl = self.level();
        //Even levels except level 0
        lvl != 0 && (lvl & 1) == 0
    }

    /// clipper.engine.h:359-362 `operator []` and :364-367
    /// C++: PolyPath64* Child(size_t index) const
    pub fn child(&self, index: usize) -> &PolyPath64 {
        &self.childs_[index]
    }

    /// clipper.engine.h:369-370 `begin()` and `end()`.
    pub fn iter(&self) -> std::slice::Iter<'_, PolyPath64> {
        self.childs_.iter()
    }

    /// clipper.engine.h:372-375
    /// C++: PolyPath64* AddChild(const Path64& path) override
    pub fn add_child(&mut self, path: &Path64) -> &mut PolyPath64 {
        self.childs_.push(PolyPath64 { level_: self.level_ + 1, childs_: Vec::new(), polygon_: path.clone() });
        self.childs_.last_mut().unwrap()
    }

    /// clipper.engine.h:377-380
    /// C++: void Clear() override
    pub fn clear(&mut self) {
        self.childs_.clear();
    }

    /// clipper.engine.h:382-385
    /// C++: size_t Count() const override
    pub fn count(&self) -> usize {
        self.childs_.len()
    }

    /// clipper.engine.h:387
    /// C++: const Path64& Polygon() const { return polygon_; };
    pub fn polygon(&self) -> &Path64 {
        &self.polygon_
    }

    /// clipper.engine.h:389-393
    /// C++: double Area() const
    pub fn area(&self) -> f64 {
        let mut a = area(&self.polygon_);
        for child in &self.childs_ {
            a += child.area();
        }
        a
    }
}

/// The tree while `BuildTree64` makes it: `OutRec::polypath` points at a node of it while
/// other nodes are added, so the nodes are kept flat, each with the numbers of its children
/// in the order `AddChild` was called, and nested into `PolyPath64` at the end. Node 0 is
/// the root, the `polytree` of `RecursiveCheckOwners`.
struct PolyPathArena {
    polygons: Vec<Path64>,
    childs: Vec<Vec<usize>>,
}

impl PolyPathArena {
    fn new() -> Self {
        PolyPathArena { polygons: vec![Path64::new()], childs: vec![Vec::new()] }
    }

    /// clipper.engine.h:372-375
    /// C++: PolyPath64* AddChild(const Path64& path) override
    fn add_child(&mut self, parent: usize, path: &Path64) -> usize {
        let node = self.polygons.len();
        self.polygons.push(path.clone());
        self.childs.push(Vec::new());
        self.childs[parent].push(node);
        node
    }

    fn fill(&mut self, node: usize, out: &mut PolyPath64) {
        let childs = std::mem::take(&mut self.childs[node]);
        for c in childs {
            let polygon = std::mem::take(&mut self.polygons[c]);
            out.childs_.push(PolyPath64 { level_: out.level_ + 1, childs_: Vec::new(), polygon_: polygon });
            let last = out.childs_.len() - 1;
            self.fill(c, &mut out.childs_[last]);
        }
    }
}

/// clipper.engine.h:196-293
/// C++: class ClipperBase
#[derive(Debug)]
pub struct ClipperBase {
    cliptype_: ClipType,
    fillrule_: FillRule,
    fillpos: FillRule,
    bot_y_: i64,
    minima_list_sorted_: bool,
    using_polytree_: bool,
    /// `Active* actives_`: the first edge of the AEL.
    actives_: usize,
    sel_: usize,
    /// `LocalMinimaList minima_list_`: a vector of pointers to `LocalMinima`, which is
    /// sorted while edges point at the minima. The minima are in `local_minima_`, which is
    /// never reordered, and this is the vector of pointers.
    minima_list_: Vec<usize>,
    /// `LocalMinimaList::iterator current_locmin_iter_`: a position in `minima_list_`.
    current_locmin_iter_: usize,
    /// `std::vector<Vertex*> vertex_lists_`: the arrays `AddPaths_` allocates, one after
    /// the other in one vector.
    vertex_lists_: Vec<Vertex>,
    /// `std::priority_queue<int64_t> scanline_list_`: only the largest value is ever read.
    scanline_list_: BinaryHeap<i64>,
    intersect_nodes_: Vec<IntersectNode>,
    horz_seg_list_: Vec<HorzSegment>,
    horz_join_list_: Vec<HorzJoin>,
    // protected:
    preserve_collinear_: bool,
    reverse_solution_: bool,
    error_code_: i32,
    has_open_paths_: bool,
    succeeded_: bool,
    /// `OutRecList outrec_list_`: the `OutRec`s themselves, `outrec_list_[i].idx == i`.
    outrec_list_: Vec<OutRec>,
    /// `ZCallback64 zCallback_` (clipper.engine.h:275), a member under `USINGZ`.
    z_callback_: ZCallback64,
    /// `int64_t DefaultZ = 0;` (clipper.engine.h:291), a member under `USINGZ`.
    pub default_z: i64,
    // What the C++ allocates with `new`:
    /// The `LocalMinima` objects.
    local_minima_: Vec<LocalMinima>,
    /// The `Active` objects.
    active_pool_: Vec<Active>,
    /// The `OutPt` objects.
    outpt_pool_: Vec<OutPt>,
    /// See `ClipperStats`. Set to zero by `execute_internal`.
    pub stats: ClipperStats,
}

impl Default for ClipperBase {
    fn default() -> Self {
        ClipperBase::new()
    }
}

impl ClipperBase {
    /// The members as clipper.engine.h:198-292 initialises them.
    pub fn new() -> Self {
        ClipperBase {
            cliptype_: ClipType::NoClip,
            fillrule_: FillRule::EvenOdd,
            fillpos: FillRule::Positive,
            bot_y_: 0,
            minima_list_sorted_: false,
            using_polytree_: false,
            actives_: NONE,
            sel_: NONE,
            minima_list_: Vec::new(),
            current_locmin_iter_: 0,
            vertex_lists_: Vec::new(),
            scanline_list_: BinaryHeap::new(),
            intersect_nodes_: Vec::new(),
            horz_seg_list_: Vec::new(),
            horz_join_list_: Vec::new(),
            preserve_collinear_: true,
            reverse_solution_: false,
            error_code_: 0,
            has_open_paths_: false,
            succeeded_: true,
            outrec_list_: Vec::new(),
            z_callback_: ZCallback64::none(),
            default_z: 0,
            local_minima_: Vec::new(),
            active_pool_: Vec::new(),
            outpt_pool_: Vec::new(),
            stats: ClipperStats::default(),
        }
    }

    /// clipper.engine.h:283
    /// C++: int ErrorCode() const { return error_code_; };
    pub fn error_code(&self) -> i32 {
        self.error_code_
    }

    /// clipper.engine.h:284
    /// C++: void PreserveCollinear(bool val) { preserve_collinear_ = val; };
    pub fn set_preserve_collinear(&mut self, val: bool) {
        self.preserve_collinear_ = val;
    }

    /// clipper.engine.h:285
    /// C++: bool PreserveCollinear() const { return preserve_collinear_;};
    pub fn preserve_collinear(&self) -> bool {
        self.preserve_collinear_
    }

    /// clipper.engine.h:286
    /// C++: void ReverseSolution(bool val) { reverse_solution_ = val; };
    pub fn set_reverse_solution(&mut self, val: bool) {
        self.reverse_solution_ = val;
    }

    /// clipper.engine.h:287
    /// C++: bool ReverseSolution() const { return reverse_solution_; };
    pub fn reverse_solution(&self) -> bool {
        self.reverse_solution_
    }

    // ---------------------------------------------------------------------------------
    // The free functions of clipper.engine.cpp that follow pointers: they take the
    // indices and read the vectors of `self`.
    // ---------------------------------------------------------------------------------

    /// clipper.engine.cpp:74-77
    /// C++: inline bool IsHotEdge(const Active& e)
    #[inline]
    fn is_hot_edge(&self, e: usize) -> bool {
        self.active_pool_[e].outrec != NONE
    }

    /// clipper.engine.cpp:80-83
    /// C++: inline bool IsOpen(const Active& e)
    #[inline]
    fn is_open(&self, e: usize) -> bool {
        self.local_minima_[self.active_pool_[e].local_min].is_open
    }

    /// clipper.engine.cpp:86-90
    /// C++: inline bool IsOpenEnd(const Vertex& v)
    #[inline]
    fn is_open_end_vertex(&self, v: usize) -> bool {
        (self.vertex_lists_[v].flags & (VertexFlags::OPEN_START | VertexFlags::OPEN_END)) != VertexFlags::EMPTY
    }

    /// clipper.engine.cpp:93-96
    /// C++: inline bool IsOpenEnd(const Active& ae)
    #[inline]
    fn is_open_end(&self, ae: usize) -> bool {
        self.is_open_end_vertex(self.active_pool_[ae].vertex_top)
    }

    /// clipper.engine.cpp:99-105
    /// C++: inline Active* GetPrevHotEdge(const Active& e)
    fn get_prev_hot_edge(&self, e: usize) -> usize {
        let mut prev = self.active_pool_[e].prev_in_ael;
        while prev != NONE && (self.is_open(prev) || !self.is_hot_edge(prev)) {
            prev = self.active_pool_[prev].prev_in_ael;
        }
        prev
    }

    /// clipper.engine.cpp:107-110
    /// C++: inline bool IsFront(const Active& e)
    #[inline]
    fn is_front(&self, e: usize) -> bool {
        e == self.outrec_list_[self.active_pool_[e].outrec].front_edge
    }

    /// clipper.engine.cpp:112-115
    /// C++: inline bool IsInvalidPath(OutPt* op)
    #[allow(dead_code)]
    #[inline]
    fn is_invalid_path(&self, op: usize) -> bool {
        op == NONE || self.outpt_pool_[op].next == op
    }

    /// clipper.engine.cpp:169-172
    /// C++: inline PathType GetPolyType(const Active& e)
    #[inline]
    fn get_poly_type(&self, e: usize) -> PathType {
        self.local_minima_[self.active_pool_[e].local_min].polytype
    }

    /// clipper.engine.cpp:174-177
    /// C++: inline bool IsSamePolyType(const Active& e1, const Active& e2)
    #[inline]
    fn is_same_poly_type(&self, e1: usize, e2: usize) -> bool {
        self.get_poly_type(e1) == self.get_poly_type(e2)
    }

    /// clipper.engine.cpp:184-190
    /// C++: inline Vertex* NextVertex(const Active& e)
    #[inline]
    fn next_vertex(&self, e: usize) -> usize {
        let ae = &self.active_pool_[e];
        if ae.wind_dx > 0 {
            self.vertex_lists_[ae.vertex_top].next
        } else {
            self.vertex_lists_[ae.vertex_top].prev
        }
    }

    /// clipper.engine.cpp:192-200
    /// C++: inline Vertex* PrevPrevVertex(const Active& ae)
    /// PrevPrevVertex: useful to get the (inverted Y-axis) top of the
    /// alternate edge (ie left or right bound) during edge insertion.
    #[inline]
    fn prev_prev_vertex(&self, e: usize) -> usize {
        let ae = &self.active_pool_[e];
        if ae.wind_dx > 0 {
            self.vertex_lists_[self.vertex_lists_[ae.vertex_top].prev].prev
        } else {
            self.vertex_lists_[self.vertex_lists_[ae.vertex_top].next].next
        }
    }

    /// clipper.engine.cpp:203-210
    /// C++: inline Active* ExtractFromSEL(Active* ae)
    fn extract_from_sel(&mut self, ae: usize) -> usize {
        let res = self.active_pool_[ae].next_in_sel;
        let prev = self.active_pool_[ae].prev_in_sel;
        if res != NONE {
            self.active_pool_[res].prev_in_sel = prev;
        }
        self.active_pool_[prev].next_in_sel = res;
        res
    }

    /// clipper.engine.cpp:213-220
    /// C++: inline void Insert1Before2InSEL(Active* ae1, Active* ae2)
    fn insert1_before2_in_sel(&mut self, ae1: usize, ae2: usize) {
        let prev = self.active_pool_[ae2].prev_in_sel;
        self.active_pool_[ae1].prev_in_sel = prev;
        if prev != NONE {
            self.active_pool_[prev].next_in_sel = ae1;
        }
        self.active_pool_[ae1].next_in_sel = ae2;
        self.active_pool_[ae2].prev_in_sel = ae1;
    }

    /// clipper.engine.cpp:222-225
    /// C++: inline bool IsMaxima(const Vertex& v)
    #[inline]
    fn is_maxima_vertex(&self, v: usize) -> bool {
        (self.vertex_lists_[v].flags & VertexFlags::LOCAL_MAX) != VertexFlags::EMPTY
    }

    /// clipper.engine.cpp:228-231
    /// C++: inline bool IsMaxima(const Active& e)
    #[inline]
    fn is_maxima(&self, e: usize) -> bool {
        self.is_maxima_vertex(self.active_pool_[e].vertex_top)
    }

    /// clipper.engine.cpp:233-248
    /// C++: inline Vertex* GetCurrYMaximaVertex_Open(const Active& e)
    fn get_curr_y_maxima_vertex_open(&self, e: usize) -> usize {
        let v = &self.vertex_lists_;
        let mut result = self.active_pool_[e].vertex_top;
        if self.active_pool_[e].wind_dx > 0 {
            while (v[v[result].next].pt.y == v[result].pt.y)
                && ((v[result].flags & (VertexFlags::OPEN_END | VertexFlags::LOCAL_MAX)) == VertexFlags::EMPTY)
            {
                result = v[result].next;
            }
        } else {
            while v[v[result].prev].pt.y == v[result].pt.y
                && ((v[result].flags & (VertexFlags::OPEN_END | VertexFlags::LOCAL_MAX)) == VertexFlags::EMPTY)
            {
                result = v[result].prev;
            }
        }
        if !self.is_maxima_vertex(result) {
            result = NONE; // not a maxima
        }
        result
    }

    /// clipper.engine.cpp:250-259
    /// C++: inline Vertex* GetCurrYMaximaVertex(const Active& e)
    fn get_curr_y_maxima_vertex(&self, e: usize) -> usize {
        let v = &self.vertex_lists_;
        let mut result = self.active_pool_[e].vertex_top;
        if self.active_pool_[e].wind_dx > 0 {
            while v[v[result].next].pt.y == v[result].pt.y {
                result = v[result].next;
            }
        } else {
            while v[v[result].prev].pt.y == v[result].pt.y {
                result = v[result].prev;
            }
        }
        if !self.is_maxima_vertex(result) {
            result = NONE; // not a maxima
        }
        result
    }

    /// clipper.engine.cpp:261-271
    /// C++: Active* GetMaximaPair(const Active& e)
    fn get_maxima_pair(&self, e: usize) -> usize {
        let mut e2 = self.active_pool_[e].next_in_ael;
        while e2 != NONE {
            if self.active_pool_[e2].vertex_top == self.active_pool_[e].vertex_top {
                return e2; // Found!
            }
            e2 = self.active_pool_[e2].next_in_ael;
        }
        NONE
    }

    /// clipper.engine.cpp:273-283
    /// C++: inline int PointCount(OutPt* op)
    #[allow(dead_code)]
    fn point_count(&self, op: usize) -> i32 {
        let mut op2 = op;
        let mut cnt = 0;
        loop {
            op2 = self.outpt_pool_[op2].next;
            cnt += 1;
            if op2 == op {
                break;
            }
        }
        cnt
    }

    /// clipper.engine.h:66-69
    /// C++: OutPt(const Point64& pt_, OutRec* outrec_): pt(pt_), outrec(outrec_) { next = this; prev = this; }
    fn new_out_pt(&mut self, pt: Point64, outrec: usize) -> usize {
        let op = self.outpt_pool_.len();
        self.outpt_pool_.push(OutPt { pt, next: op, prev: op, outrec, horz: false });
        op
    }

    /// clipper.engine.cpp:285-303
    /// C++: inline OutPt* DuplicateOp(OutPt* op, bool insert_after)
    fn duplicate_op(&mut self, op: usize, insert_after: bool) -> usize {
        let result = self.new_out_pt(self.outpt_pool_[op].pt, self.outpt_pool_[op].outrec);
        if insert_after {
            let next = self.outpt_pool_[op].next;
            self.outpt_pool_[result].next = next;
            self.outpt_pool_[next].prev = result;
            self.outpt_pool_[result].prev = op;
            self.outpt_pool_[op].next = result;
        } else {
            let prev = self.outpt_pool_[op].prev;
            self.outpt_pool_[result].prev = prev;
            self.outpt_pool_[prev].next = result;
            self.outpt_pool_[result].next = op;
            self.outpt_pool_[op].prev = result;
        }
        result
    }

    /// clipper.engine.cpp:305-312
    /// C++: inline OutPt* DisposeOutPt(OutPt* op)
    fn dispose_out_pt(&mut self, op: usize) -> usize {
        let result = self.outpt_pool_[op].next;
        let prev = self.outpt_pool_[op].prev;
        self.outpt_pool_[prev].next = result;
        self.outpt_pool_[result].prev = prev;
        // delete op;
        result
    }

    /// clipper.engine.cpp:315-326
    /// C++: inline void DisposeOutPts(OutRec* outrec)
    fn dispose_out_pts(&mut self, outrec: usize) {
        // The points are deleted; nothing reads them afterwards.
        self.outrec_list_[outrec].pts = NONE;
    }

    /// clipper.engine.cpp:336-340
    /// C++: inline void SetSides(OutRec& outrec, Active& start_edge, Active& end_edge)
    #[inline]
    fn set_sides(&mut self, outrec: usize, start_edge: usize, end_edge: usize) {
        self.outrec_list_[outrec].front_edge = start_edge;
        self.outrec_list_[outrec].back_edge = end_edge;
    }

    /// clipper.engine.cpp:343-370
    /// C++: void SwapOutrecs(Active& e1, Active& e2)
    fn swap_outrecs(&mut self, e1: usize, e2: usize) {
        let or1 = self.active_pool_[e1].outrec;
        let or2 = self.active_pool_[e2].outrec;
        if or1 == or2 {
            let e = self.outrec_list_[or1].front_edge;
            self.outrec_list_[or1].front_edge = self.outrec_list_[or1].back_edge;
            self.outrec_list_[or1].back_edge = e;
            return;
        }
        if or1 != NONE {
            if e1 == self.outrec_list_[or1].front_edge {
                self.outrec_list_[or1].front_edge = e2;
            } else {
                self.outrec_list_[or1].back_edge = e2;
            }
        }
        if or2 != NONE {
            if e2 == self.outrec_list_[or2].front_edge {
                self.outrec_list_[or2].front_edge = e1;
            } else {
                self.outrec_list_[or2].back_edge = e1;
            }
        }
        self.active_pool_[e1].outrec = or2;
        self.active_pool_[e2].outrec = or1;
    }

    /// clipper.engine.cpp:373-385
    /// C++: double Area(OutPt* op)
    fn area_op(&self, op: usize) -> f64 {
        //https://en.wikipedia.org/wiki/Shoelace_formula
        let p = &self.outpt_pool_;
        let mut result = 0.0f64;
        let mut op2 = op;
        loop {
            // `result += a * b`: fma(a, b, result).
            let prev = p[op2].prev;
            result = ((p[prev].pt.y + p[op2].pt.y) as f64).mul_add((p[prev].pt.x - p[op2].pt.x) as f64, result);
            op2 = p[op2].next;
            if op2 == op {
                break;
            }
        }
        result * 0.5
    }

    /// clipper.engine.cpp:395-409
    /// C++: void ReverseOutPts(OutPt* op)
    #[allow(dead_code)]
    fn reverse_out_pts(&mut self, op: usize) {
        if op == NONE {
            return;
        }

        let mut op1 = op;
        loop {
            let op2 = self.outpt_pool_[op1].next;
            self.outpt_pool_[op1].next = self.outpt_pool_[op1].prev;
            self.outpt_pool_[op1].prev = op2;
            op1 = op2;
            if op1 == op {
                break;
            }
        }
    }

    /// clipper.engine.cpp:411-417
    /// C++: inline void SwapSides(OutRec& outrec)
    #[allow(dead_code)]
    fn swap_sides(&mut self, outrec: usize) {
        let e2 = self.outrec_list_[outrec].front_edge;
        self.outrec_list_[outrec].front_edge = self.outrec_list_[outrec].back_edge;
        self.outrec_list_[outrec].back_edge = e2;
        self.outrec_list_[outrec].pts = self.outpt_pool_[self.outrec_list_[outrec].pts].next;
    }

    /// clipper.engine.cpp:419-423
    /// C++: inline OutRec* GetRealOutRec(OutRec* outrec)
    #[inline]
    fn get_real_out_rec(&self, mut outrec: usize) -> usize {
        while outrec != NONE && self.outrec_list_[outrec].pts == NONE {
            outrec = self.outrec_list_[outrec].owner;
        }
        outrec
    }

    /// clipper.engine.cpp:425-430
    /// C++: inline bool IsValidOwner(OutRec* outrec, OutRec* testOwner)
    #[inline]
    fn is_valid_owner(&self, outrec: usize, mut test_owner: usize) -> bool {
        // prevent outrec owning itself either directly or indirectly
        while test_owner != NONE && test_owner != outrec {
            test_owner = self.outrec_list_[test_owner].owner;
        }
        test_owner == NONE
    }

    /// clipper.engine.cpp:432-440
    /// C++: inline void UncoupleOutRec(Active ae)
    fn uncouple_out_rec(&mut self, ae: usize) {
        let outrec = self.active_pool_[ae].outrec;
        if outrec == NONE {
            return;
        }
        let front = self.outrec_list_[outrec].front_edge;
        self.active_pool_[front].outrec = NONE;
        let back = self.outrec_list_[outrec].back_edge;
        self.active_pool_[back].outrec = NONE;
        self.outrec_list_[outrec].front_edge = NONE;
        self.outrec_list_[outrec].back_edge = NONE;
    }

    /// clipper.engine.cpp:448-454
    /// C++: inline bool IsVerySmallTriangle(const OutPt& op)
    fn is_very_small_triangle(&self, op: usize) -> bool {
        let p = &self.outpt_pool_;
        let (next, prev) = (p[op].next, p[op].prev);
        p[next].next == prev
            && (pts_really_close(&p[prev].pt, &p[next].pt)
                || pts_really_close(&p[op].pt, &p[next].pt)
                || pts_really_close(&p[op].pt, &p[prev].pt))
    }

    /// clipper.engine.cpp:456-460
    /// C++: inline bool IsValidClosedPath(const OutPt* op)
    fn is_valid_closed_path(&self, op: usize) -> bool {
        op != NONE
            && (self.outpt_pool_[op].next != op)
            && (self.outpt_pool_[op].next != self.outpt_pool_[op].prev)
            && !self.is_very_small_triangle(op)
    }

    /// clipper.engine.cpp:462-465
    /// C++: inline bool OutrecIsAscending(const Active* hotEdge)
    #[inline]
    fn outrec_is_ascending(&self, hot_edge: usize) -> bool {
        hot_edge == self.outrec_list_[self.active_pool_[hot_edge].outrec].front_edge
    }

    /// clipper.engine.cpp:467-473
    /// C++: inline void SwapFrontBackSides(OutRec& outrec)
    fn swap_front_back_sides(&mut self, outrec: usize) {
        let tmp = self.outrec_list_[outrec].front_edge;
        self.outrec_list_[outrec].front_edge = self.outrec_list_[outrec].back_edge;
        self.outrec_list_[outrec].back_edge = tmp;
        self.outrec_list_[outrec].pts = self.outpt_pool_[self.outrec_list_[outrec].pts].next;
    }

    /// clipper.engine.cpp:475-478
    /// C++: inline bool EdgesAdjacentInAEL(const IntersectNode& inode)
    #[inline]
    fn edges_adjacent_in_ael(&self, inode: &IntersectNode) -> bool {
        (self.active_pool_[inode.edge1].next_in_ael == inode.edge2) || (self.active_pool_[inode.edge1].prev_in_ael == inode.edge2)
    }

    /// clipper.engine.cpp:480-483
    /// C++: inline bool IsJoined(const Active& e)
    #[inline]
    fn is_joined(&self, e: usize) -> bool {
        self.active_pool_[e].join_with != JoinWith::NoJoin
    }

    /// clipper.engine.cpp:485-494
    /// C++: inline void SetOwner(OutRec* outrec, OutRec* new_owner)
    fn set_owner(&mut self, outrec: usize, new_owner: usize) {
        //precondition1: new_owner is never null
        loop {
            let o = self.outrec_list_[new_owner].owner;
            if o != NONE && self.outrec_list_[o].pts == NONE {
                self.outrec_list_[new_owner].owner = self.outrec_list_[o].owner;
            } else {
                break;
            }
        }
        let mut tmp = new_owner;
        while tmp != NONE && tmp != outrec {
            tmp = self.outrec_list_[tmp].owner;
        }
        if tmp != NONE {
            self.outrec_list_[new_owner].owner = self.outrec_list_[outrec].owner;
        }
        self.outrec_list_[outrec].owner = new_owner;
    }

    /// clipper.engine.cpp:496-559
    /// C++: static PointInPolygonResult PointInOpPolygon(const Point64& pt, OutPt* op)
    fn point_in_op_polygon(&self, pt: &Point64, mut op: usize) -> PointInPolygonResult {
        let p = &self.outpt_pool_;
        if op == p[op].next || p[op].prev == p[op].next {
            return PointInPolygonResult::IsOutside;
        }

        let mut op2 = op;
        loop {
            if p[op].pt.y != pt.y {
                break;
            }
            op = p[op].next;
            if op == op2 {
                break;
            }
        }
        if p[op].pt.y == pt.y {
            // not a proper polygon
            return PointInPolygonResult::IsOutside;
        }

        let mut is_above = p[op].pt.y < pt.y;
        let starting_above = is_above;
        let mut val = 0;
        op2 = p[op].next;
        while op2 != op {
            if is_above {
                while op2 != op && p[op2].pt.y < pt.y {
                    op2 = p[op2].next;
                }
            } else {
                while op2 != op && p[op2].pt.y > pt.y {
                    op2 = p[op2].next;
                }
            }
            if op2 == op {
                break;
            }

            // must have touched or crossed the pt.Y horizontal
            // and this must happen an even number of times

            let prev = p[op2].prev;
            if p[op2].pt.y == pt.y {
                // touching the horizontal
                if p[op2].pt.x == pt.x || (p[op2].pt.y == p[prev].pt.y && (pt.x < p[prev].pt.x) != (pt.x < p[op2].pt.x)) {
                    return PointInPolygonResult::IsOn;
                }

                op2 = p[op2].next;
                if op2 == op {
                    break;
                }
                continue;
            }

            if pt.x < p[op2].pt.x && pt.x < p[prev].pt.x {
                // do nothing because
                // we're only interested in edges crossing on the left
            } else if pt.x > p[prev].pt.x && pt.x > p[op2].pt.x {
                val = 1 - val; // toggle val
            } else {
                let d = cross_product(&p[prev].pt, &p[op2].pt, pt);
                if d == 0.0 {
                    return PointInPolygonResult::IsOn;
                }
                if (d < 0.0) == is_above {
                    val = 1 - val;
                }
            }
            is_above = !is_above;
            op2 = p[op2].next;
        }

        if is_above != starting_above {
            let d = cross_product(&p[p[op2].prev].pt, &p[op2].pt, pt);
            if d == 0.0 {
                return PointInPolygonResult::IsOn;
            }
            if (d < 0.0) == is_above {
                val = 1 - val;
            }
        }

        if val == 0 {
            PointInPolygonResult::IsOutside
        } else {
            PointInPolygonResult::IsInside
        }
    }

    /// clipper.engine.cpp:561-582
    /// C++: inline Path64 GetCleanPath(OutPt* op)
    fn get_clean_path(&self, op: usize) -> Path64 {
        let p = &self.outpt_pool_;
        let mut result = Path64::new();
        let mut op2 = op;
        while p[op2].next != op
            && ((p[op2].pt.x == p[p[op2].next].pt.x && p[op2].pt.x == p[p[op2].prev].pt.x)
                || (p[op2].pt.y == p[p[op2].next].pt.y && p[op2].pt.y == p[p[op2].prev].pt.y))
        {
            op2 = p[op2].next;
        }
        result.push(p[op2].pt);
        let mut prev_op = op2;
        op2 = p[op2].next;
        while op2 != op {
            if (p[op2].pt.x != p[p[op2].next].pt.x || p[op2].pt.x != p[prev_op].pt.x)
                && (p[op2].pt.y != p[p[op2].next].pt.y || p[op2].pt.y != p[prev_op].pt.y)
            {
                result.push(p[op2].pt);
                prev_op = op2;
            }
            op2 = p[op2].next;
        }
        result
    }

    /// clipper.engine.cpp:584-603
    /// C++: inline bool Path1InsidePath2(OutPt* op1, OutPt* op2)
    fn path1_inside_path2(&mut self, op1: usize, op2: usize) -> bool {
        // we need to make some accommodation for rounding errors
        // so we won't jump if the first vertex is found outside
        let mut outside_cnt: i32 = 0;
        let mut op = op1;
        loop {
            let result = self.point_in_op_polygon(&self.outpt_pool_[op].pt, op2);
            if result == PointInPolygonResult::IsOutside {
                outside_cnt += 1;
            } else if result == PointInPolygonResult::IsInside {
                outside_cnt -= 1;
            }
            op = self.outpt_pool_[op].next;
            if !(op != op1 && outside_cnt.abs() < 2) {
                break;
            }
        }
        if outside_cnt.abs() > 1 {
            return outside_cnt < 0;
        }
        // since path1's location is still equivocal, check its midpoint
        self.stats.path1_inside_path2_midpoint += 1;
        let mp = get_bounds(&self.get_clean_path(op1)).mid_point();
        let path2 = self.get_clean_path(op2);
        point_in_polygon(&mp, &path2) != PointInPolygonResult::IsOutside
    }

    /// clipper.engine.cpp:608-616
    /// C++: void AddLocMin(LocalMinimaList& list, Vertex& vert, PathType polytype, bool is_open)
    /// (`ClipperBase::AddLocMin`, clipper.engine.cpp:906-913, is the same on `minima_list_`
    /// and has no caller.)
    fn add_loc_min(&mut self, vert: usize, polytype: PathType, is_open: bool) {
        //make sure the vertex is added only once ...
        if (VertexFlags::LOCAL_MIN & self.vertex_lists_[vert].flags) != VertexFlags::EMPTY {
            return;
        }

        self.vertex_lists_[vert].flags = self.vertex_lists_[vert].flags | VertexFlags::LOCAL_MIN;
        self.local_minima_.push(LocalMinima { vertex: vert, polytype, is_open });
        self.minima_list_.push(self.local_minima_.len() - 1);
    }

    /// clipper.engine.cpp:618-720
    /// C++: void AddPaths_(const Paths64& paths, PathType polytype, bool is_open, std::vector<Vertex*>& vertexLists, LocalMinimaList& locMinList)
    /// `new Vertex[total_vertex_count]` is `total_vertex_count` more elements of
    /// `vertex_lists_`; `v`, `v0`, `curr_v` and `prev_v` are indices into it.
    fn add_paths_(&mut self, paths: &Paths64, polytype: PathType, is_open: bool) {
        let total_vertex_count: usize = paths.iter().map(|path| path.len()).sum();
        if total_vertex_count == 0 {
            return;
        }

        let vertices = self.vertex_lists_.len();
        self.vertex_lists_.resize(vertices + total_vertex_count, Vertex::default());
        let mut v = vertices;
        for path in paths {
            //for each path create a circular double linked list of vertices
            let v0 = v;
            let mut curr_v = v;
            let mut prev_v = NONE;

            if path.is_empty() {
                continue;
            }

            self.vertex_lists_[v].prev = NONE;
            let mut cnt = 0;
            for pt in path {
                if prev_v != NONE {
                    if self.vertex_lists_[prev_v].pt == *pt {
                        continue; // ie skips duplicates
                    }
                    self.vertex_lists_[prev_v].next = curr_v;
                }
                self.vertex_lists_[curr_v].prev = prev_v;
                self.vertex_lists_[curr_v].pt = *pt;
                self.vertex_lists_[curr_v].flags = VertexFlags::EMPTY;
                prev_v = curr_v;
                curr_v += 1;
                cnt += 1;
            }
            if prev_v == NONE || self.vertex_lists_[prev_v].prev == NONE {
                continue;
            }
            if !is_open && self.vertex_lists_[prev_v].pt == self.vertex_lists_[v0].pt {
                prev_v = self.vertex_lists_[prev_v].prev;
            }
            self.vertex_lists_[prev_v].next = v0;
            self.vertex_lists_[v0].prev = prev_v;
            v = curr_v; // ie get ready for next path
            if cnt < 2 || (cnt == 2 && !is_open) {
                continue;
            }

            //now find and assign local minima
            let mut going_up;
            if is_open {
                curr_v = self.vertex_lists_[v0].next;
                while curr_v != v0 && self.vertex_lists_[curr_v].pt.y == self.vertex_lists_[v0].pt.y {
                    curr_v = self.vertex_lists_[curr_v].next;
                }
                going_up = self.vertex_lists_[curr_v].pt.y <= self.vertex_lists_[v0].pt.y;
                if going_up {
                    self.vertex_lists_[v0].flags = VertexFlags::OPEN_START;
                    self.add_loc_min(v0, polytype, true);
                } else {
                    self.vertex_lists_[v0].flags = VertexFlags::OPEN_START | VertexFlags::LOCAL_MAX;
                }
            } else {
                // closed path
                prev_v = self.vertex_lists_[v0].prev;
                while prev_v != v0 && self.vertex_lists_[prev_v].pt.y == self.vertex_lists_[v0].pt.y {
                    prev_v = self.vertex_lists_[prev_v].prev;
                }
                if prev_v == v0 {
                    continue; // only open paths can be completely flat
                }
                going_up = self.vertex_lists_[prev_v].pt.y > self.vertex_lists_[v0].pt.y;
            }

            let going_up0 = going_up;
            prev_v = v0;
            curr_v = self.vertex_lists_[v0].next;
            while curr_v != v0 {
                if self.vertex_lists_[curr_v].pt.y > self.vertex_lists_[prev_v].pt.y && going_up {
                    self.vertex_lists_[prev_v].flags = self.vertex_lists_[prev_v].flags | VertexFlags::LOCAL_MAX;
                    going_up = false;
                } else if self.vertex_lists_[curr_v].pt.y < self.vertex_lists_[prev_v].pt.y && !going_up {
                    going_up = true;
                    self.add_loc_min(prev_v, polytype, is_open);
                }
                prev_v = curr_v;
                curr_v = self.vertex_lists_[curr_v].next;
            }

            if is_open {
                self.vertex_lists_[prev_v].flags = self.vertex_lists_[prev_v].flags | VertexFlags::OPEN_END;
                if going_up {
                    self.vertex_lists_[prev_v].flags = self.vertex_lists_[prev_v].flags | VertexFlags::LOCAL_MAX;
                } else {
                    self.add_loc_min(prev_v, polytype, is_open);
                }
            } else if going_up != going_up0 {
                if going_up0 {
                    self.add_loc_min(prev_v, polytype, false);
                } else {
                    self.vertex_lists_[prev_v].flags = self.vertex_lists_[prev_v].flags | VertexFlags::LOCAL_MAX;
                }
            }
        } // end processing current path

        // vertexLists.emplace_back(vertices);
    }

    // ---------------------------------------------------------------------------------
    // ClipperBase methods ...
    // ---------------------------------------------------------------------------------

    /// clipper.engine.cpp:762-770
    /// C++: void ClipperBase::DeleteEdges(Active*& e)
    fn delete_edges(&mut self) {
        // Every edge is deleted, those of the AEL here and the others when they left it.
        self.actives_ = NONE;
        self.active_pool_.clear();
    }

    /// clipper.engine.cpp:772-780
    /// C++: void ClipperBase::CleanUp()
    /// unlike Clear, CleanUp preserves added paths
    pub fn clean_up(&mut self) {
        self.delete_edges();
        self.scanline_list_ = BinaryHeap::new();
        self.intersect_nodes_.clear();
        self.dispose_all_out_recs();
        self.horz_seg_list_.clear();
        self.horz_join_list_.clear();
    }

    /// clipper.engine.cpp:783-790
    /// C++: void ClipperBase::Clear()
    pub fn clear(&mut self) {
        self.clean_up();
        self.dispose_vertices_and_local_minima();
        self.current_locmin_iter_ = 0;
        self.minima_list_sorted_ = false;
        self.has_open_paths_ = false;
    }

    /// clipper.engine.cpp:793-808
    /// C++: void ClipperBase::Reset()
    fn reset(&mut self) {
        if !self.minima_list_sorted_ {
            // std::stable_sort with LocMinSorter (clipper.engine.cpp:56-65): the larger y
            // first, and among equal y the smaller x first. (#594)
            let local_minima = &self.local_minima_;
            let vertex_lists = &self.vertex_lists_;
            self.minima_list_.sort_by(|&loc_min1, &loc_min2| {
                let p1 = &vertex_lists[local_minima[loc_min1].vertex].pt;
                let p2 = &vertex_lists[local_minima[loc_min2].vertex].pt;
                if p2.y != p1.y {
                    p2.y.cmp(&p1.y)
                } else {
                    p1.x.cmp(&p2.x)
                }
            });
            self.minima_list_sorted_ = true;
        }
        for i in (0..self.minima_list_.len()).rev() {
            let y = self.vertex_lists_[self.local_minima_[self.minima_list_[i]].vertex].pt.y;
            self.insert_scanline(y);
        }

        self.current_locmin_iter_ = 0;
        self.actives_ = NONE;
        self.sel_ = NONE;
        self.succeeded_ = true;
    }

    /// clipper.engine.cpp:811-836
    /// C++: void ClipperBase::SetZ(const Active& e1, const Active& e2, Point64& ip)
    /// (under `USINGZ`). `ip` is the point of the `OutPt` `op`.
    fn set_z(&mut self, e1: usize, e2: usize, op: usize) {
        if !self.z_callback_.is_set() {
            return;
        }
        let (e1bot, e1top) = (self.active_pool_[e1].bot, self.active_pool_[e1].top);
        let (e2bot, e2top) = (self.active_pool_[e2].bot, self.active_pool_[e2].top);
        let mut ip = self.outpt_pool_[op].pt;
        // prioritize subject over clip vertices by passing
        // subject vertices before clip vertices in the callback
        if self.get_poly_type(e1) == PathType::Subject {
            if ip == e1bot {
                point_set_z(&mut ip, point_z(&e1bot));
            } else if ip == e1top {
                point_set_z(&mut ip, point_z(&e1top));
            } else if ip == e2bot {
                point_set_z(&mut ip, point_z(&e2bot));
            } else if ip == e2top {
                point_set_z(&mut ip, point_z(&e2top));
            } else {
                point_set_z(&mut ip, self.default_z);
            }
            self.z_callback_.call(&e1bot, &e1top, &e2bot, &e2top, &mut ip);
        } else {
            if ip == e2bot {
                point_set_z(&mut ip, point_z(&e2bot));
            } else if ip == e2top {
                point_set_z(&mut ip, point_z(&e2top));
            } else if ip == e1bot {
                point_set_z(&mut ip, point_z(&e1bot));
            } else if ip == e1top {
                point_set_z(&mut ip, point_z(&e1top));
            } else {
                point_set_z(&mut ip, self.default_z);
            }
            self.z_callback_.call(&e2bot, &e2top, &e1bot, &e1top, &mut ip);
        }
        self.outpt_pool_[op].pt = ip;
    }

    /// clipper.engine.cpp:838-841
    /// C++: void ClipperBase::AddPath(const Path64& path, PathType polytype, bool is_open)
    pub fn add_path(&mut self, path: &Path64, polytype: PathType, is_open: bool) {
        self.add_paths(&vec![path.clone()], polytype, is_open);
    }

    /// clipper.engine.cpp:843-848
    /// C++: void ClipperBase::AddPaths(const Paths64& paths, PathType polytype, bool is_open)
    pub fn add_paths(&mut self, paths: &Paths64, polytype: PathType, is_open: bool) {
        if is_open {
            self.has_open_paths_ = true;
        }
        self.minima_list_sorted_ = false;
        self.add_paths_(paths, polytype, is_open);
    }

    /// clipper.engine.cpp:864-867
    /// C++: void ClipperBase::InsertScanline(int64_t y)
    #[inline]
    fn insert_scanline(&mut self, y: i64) {
        self.scanline_list_.push(y);
    }

    /// clipper.engine.cpp:870-878
    /// C++: bool ClipperBase::PopScanline(int64_t& y)
    fn pop_scanline(&mut self, y: &mut i64) -> bool {
        match self.scanline_list_.pop() {
            None => false,
            Some(top) => {
                *y = top;
                while self.scanline_list_.peek() == Some(&top) {
                    self.scanline_list_.pop(); // Pop duplicates.
                }
                true
            }
        }
    }

    /// clipper.engine.cpp:881-886
    /// C++: bool ClipperBase::PopLocalMinima(int64_t y, LocalMinima*& local_minima)
    fn pop_local_minima(&mut self, y: i64, local_minima: &mut usize) -> bool {
        if self.current_locmin_iter_ == self.minima_list_.len()
            || self.vertex_lists_[self.local_minima_[self.minima_list_[self.current_locmin_iter_]].vertex].pt.y != y
        {
            return false;
        }
        *local_minima = self.minima_list_[self.current_locmin_iter_];
        self.current_locmin_iter_ += 1;
        true
    }

    /// clipper.engine.cpp:888-896
    /// C++: void ClipperBase::DisposeAllOutRecs()
    fn dispose_all_out_recs(&mut self) {
        self.outpt_pool_.clear();
        self.outrec_list_.clear();
    }

    /// clipper.engine.cpp:898-903
    /// C++: void ClipperBase::DisposeVerticesAndLocalMinima()
    fn dispose_vertices_and_local_minima(&mut self) {
        self.minima_list_.clear();
        self.local_minima_.clear();
        self.vertex_lists_.clear();
    }

    /// clipper.engine.cpp:915-988
    /// C++: bool ClipperBase::IsContributingClosed(const Active& e) const
    fn is_contributing_closed(&self, e: usize) -> bool {
        let ae = &self.active_pool_[e];
        match self.fillrule_ {
            FillRule::EvenOdd => {}
            FillRule::NonZero => {
                if ae.wind_cnt.abs() != 1 {
                    return false;
                }
            }
            FillRule::Positive => {
                if ae.wind_cnt != 1 {
                    return false;
                }
            }
            FillRule::Negative => {
                if ae.wind_cnt != -1 {
                    return false;
                }
            }
        }

        match self.cliptype_ {
            ClipType::NoClip => false,
            ClipType::Intersection => match self.fillrule_ {
                FillRule::Positive => ae.wind_cnt2 > 0,
                FillRule::Negative => ae.wind_cnt2 < 0,
                _ => ae.wind_cnt2 != 0,
            },
            ClipType::Union => match self.fillrule_ {
                FillRule::Positive => ae.wind_cnt2 <= 0,
                FillRule::Negative => ae.wind_cnt2 >= 0,
                _ => ae.wind_cnt2 == 0,
            },
            ClipType::Difference => {
                let result = match self.fillrule_ {
                    FillRule::Positive => ae.wind_cnt2 <= 0,
                    FillRule::Negative => ae.wind_cnt2 >= 0,
                    _ => ae.wind_cnt2 == 0,
                };
                if self.get_poly_type(e) == PathType::Subject {
                    result
                } else {
                    !result
                }
            }
            ClipType::Xor => true,
        }
    }

    /// clipper.engine.cpp:991-1015
    /// C++: inline bool ClipperBase::IsContributingOpen(const Active& e) const
    fn is_contributing_open(&self, e: usize) -> bool {
        let ae = &self.active_pool_[e];
        let (is_in_clip, is_in_subj) = match self.fillrule_ {
            FillRule::Positive => (ae.wind_cnt2 > 0, ae.wind_cnt > 0),
            FillRule::Negative => (ae.wind_cnt2 < 0, ae.wind_cnt < 0),
            _ => (ae.wind_cnt2 != 0, ae.wind_cnt != 0),
        };

        match self.cliptype_ {
            ClipType::Intersection => is_in_clip,
            ClipType::Union => !is_in_subj && !is_in_clip,
            _ => !is_in_clip,
        }
    }

    /// clipper.engine.cpp:1018-1093
    /// C++: void ClipperBase::SetWindCountForClosedPathEdge(Active& e)
    fn set_wind_count_for_closed_path_edge(&mut self, e: usize) {
        //Wind counts refer to polygon regions not edges, so here an edge's WindCnt
        //indicates the higher of the wind counts for the two regions touching the
        //edge. (NB Adjacent regions can only ever have their wind counts differ by
        //one. Also, open paths have no meaningful wind directions or counts.)

        let mut e2 = self.active_pool_[e].prev_in_ael;
        //find the nearest closed path edge of the same PolyType in AEL (heading left)
        let pt = self.get_poly_type(e);
        while e2 != NONE && (self.get_poly_type(e2) != pt || self.is_open(e2)) {
            e2 = self.active_pool_[e2].prev_in_ael;
        }

        if e2 == NONE {
            self.active_pool_[e].wind_cnt = self.active_pool_[e].wind_dx;
            e2 = self.actives_;
        } else if self.fillrule_ == FillRule::EvenOdd {
            self.active_pool_[e].wind_cnt = self.active_pool_[e].wind_dx;
            self.active_pool_[e].wind_cnt2 = self.active_pool_[e2].wind_cnt2;
            e2 = self.active_pool_[e2].next_in_ael;
        } else {
            //NonZero, positive, or negative filling here ...
            //if e's WindCnt is in the SAME direction as its WindDx, then polygon
            //filling will be on the right of 'e'.
            //NB neither e2.WindCnt nor e2.WindDx should ever be 0.
            let (e2_wind_cnt, e2_wind_dx) = (self.active_pool_[e2].wind_cnt, self.active_pool_[e2].wind_dx);
            let e_wind_dx = self.active_pool_[e].wind_dx;
            if e2_wind_cnt * e2_wind_dx < 0 {
                //opposite directions so 'e' is outside 'e2' ...
                if e2_wind_cnt.abs() > 1 {
                    //outside prev poly but still inside another.
                    if e2_wind_dx * e_wind_dx < 0 {
                        //reversing direction so use the same WC
                        self.active_pool_[e].wind_cnt = e2_wind_cnt;
                    } else {
                        //otherwise keep 'reducing' the WC by 1 (ie towards 0) ...
                        self.active_pool_[e].wind_cnt = e2_wind_cnt + e_wind_dx;
                    }
                } else {
                    //now outside all polys of same polytype so set own WC ...
                    self.active_pool_[e].wind_cnt = if self.is_open(e) { 1 } else { e_wind_dx };
                }
            } else {
                //'e' must be inside 'e2'
                if e2_wind_dx * e_wind_dx < 0 {
                    //reversing direction so use the same WC
                    self.active_pool_[e].wind_cnt = e2_wind_cnt;
                } else {
                    //otherwise keep 'increasing' the WC by 1 (ie away from 0) ...
                    self.active_pool_[e].wind_cnt = e2_wind_cnt + e_wind_dx;
                }
            }
            self.active_pool_[e].wind_cnt2 = self.active_pool_[e2].wind_cnt2;
            e2 = self.active_pool_[e2].next_in_ael; // ie get ready to calc WindCnt2
        }

        //update wind_cnt2 ...
        if self.fillrule_ == FillRule::EvenOdd {
            while e2 != e {
                if self.get_poly_type(e2) != pt && !self.is_open(e2) {
                    self.active_pool_[e].wind_cnt2 = if self.active_pool_[e].wind_cnt2 == 0 { 1 } else { 0 };
                }
                e2 = self.active_pool_[e2].next_in_ael;
            }
        } else {
            while e2 != e {
                if self.get_poly_type(e2) != pt && !self.is_open(e2) {
                    self.active_pool_[e].wind_cnt2 += self.active_pool_[e2].wind_dx;
                }
                e2 = self.active_pool_[e2].next_in_ael;
            }
        }
    }

    /// clipper.engine.cpp:1096-1124
    /// C++: void ClipperBase::SetWindCountForOpenPathEdge(Active& e)
    fn set_wind_count_for_open_path_edge(&mut self, e: usize) {
        let mut e2 = self.actives_;
        if self.fillrule_ == FillRule::EvenOdd {
            let mut cnt1 = 0;
            let mut cnt2 = 0;
            while e2 != e {
                if self.get_poly_type(e2) == PathType::Clip {
                    cnt2 += 1;
                } else if !self.is_open(e2) {
                    cnt1 += 1;
                }
                e2 = self.active_pool_[e2].next_in_ael;
            }
            self.active_pool_[e].wind_cnt = if is_odd(cnt1) { 1 } else { 0 };
            self.active_pool_[e].wind_cnt2 = if is_odd(cnt2) { 1 } else { 0 };
        } else {
            while e2 != e {
                if self.get_poly_type(e2) == PathType::Clip {
                    self.active_pool_[e].wind_cnt2 += self.active_pool_[e2].wind_dx;
                } else if !self.is_open(e2) {
                    self.active_pool_[e].wind_cnt += self.active_pool_[e2].wind_dx;
                }
                e2 = self.active_pool_[e2].next_in_ael;
            }
        }
    }

    /// clipper.engine.cpp:1126-1163
    /// C++: bool IsValidAelOrder(const Active& resident, const Active& newcomer)
    fn is_valid_ael_order(&self, resident: usize, newcomer: usize) -> bool {
        let r = &self.active_pool_[resident];
        let n = &self.active_pool_[newcomer];
        if n.curr_x != r.curr_x {
            return n.curr_x > r.curr_x;
        }

        //get the turning direction  a1.top, a2.bot, a2.top
        let d = cross_product(&r.top, &n.bot, &n.top);
        if d != 0.0 {
            return d < 0.0;
        }

        //edges must be collinear to get here
        //for starting open paths, place them according to
        //the direction they're about to turn
        if !self.is_maxima(resident) && (r.top.y > n.top.y) {
            return cross_product(&n.bot, &r.top, &self.vertex_lists_[self.next_vertex(resident)].pt) <= 0.0;
        } else if !self.is_maxima(newcomer) && (n.top.y > r.top.y) {
            return cross_product(&n.bot, &n.top, &self.vertex_lists_[self.next_vertex(newcomer)].pt) >= 0.0;
        }

        let y = n.bot.y;
        let newcomer_is_left = n.is_left_bound;

        if r.bot.y != y || self.vertex_lists_[self.local_minima_[r.local_min].vertex].pt.y != y {
            n.is_left_bound
        }
        //resident must also have just been inserted
        else if r.is_left_bound != newcomer_is_left {
            newcomer_is_left
        } else if is_collinear(&self.vertex_lists_[self.prev_prev_vertex(resident)].pt, &r.bot, &r.top) {
            true
        } else {
            //compare turning direction of the alternate bound
            (cross_product(
                &self.vertex_lists_[self.prev_prev_vertex(resident)].pt,
                &n.bot,
                &self.vertex_lists_[self.prev_prev_vertex(newcomer)].pt,
            ) > 0.0)
                == newcomer_is_left
        }
    }

    /// clipper.engine.cpp:1166-1195
    /// C++: void ClipperBase::InsertLeftEdge(Active& e)
    fn insert_left_edge(&mut self, e: usize) {
        if self.actives_ == NONE {
            self.active_pool_[e].prev_in_ael = NONE;
            self.active_pool_[e].next_in_ael = NONE;
            self.actives_ = e;
        } else if !self.is_valid_ael_order(self.actives_, e) {
            self.active_pool_[e].prev_in_ael = NONE;
            self.active_pool_[e].next_in_ael = self.actives_;
            self.active_pool_[self.actives_].prev_in_ael = e;
            self.actives_ = e;
        } else {
            let mut e2 = self.actives_;
            while self.active_pool_[e2].next_in_ael != NONE && self.is_valid_ael_order(self.active_pool_[e2].next_in_ael, e) {
                e2 = self.active_pool_[e2].next_in_ael;
            }
            if self.active_pool_[e2].join_with == JoinWith::Right {
                e2 = self.active_pool_[e2].next_in_ael;
            }
            if e2 == NONE {
                return; // should never happen and stops compiler warning :)
            }
            let next = self.active_pool_[e2].next_in_ael;
            self.active_pool_[e].next_in_ael = next;
            if next != NONE {
                self.active_pool_[next].prev_in_ael = e;
            }
            self.active_pool_[e].prev_in_ael = e2;
            self.active_pool_[e2].next_in_ael = e;
        }
    }

    /// clipper.engine.cpp:1198-1204
    /// C++: void InsertRightEdge(Active& e, Active& e2)
    fn insert_right_edge(&mut self, e: usize, e2: usize) {
        let next = self.active_pool_[e].next_in_ael;
        self.active_pool_[e2].next_in_ael = next;
        if next != NONE {
            self.active_pool_[next].prev_in_ael = e2;
        }
        self.active_pool_[e2].prev_in_ael = e;
        self.active_pool_[e].next_in_ael = e2;
    }

    /// `new Active()`
    fn new_active(&mut self) -> usize {
        self.active_pool_.push(Active::default());
        self.active_pool_.len() - 1
    }

    /// clipper.engine.cpp:1207-1322
    /// C++: void ClipperBase::InsertLocalMinimaIntoAEL(int64_t bot_y)
    fn insert_local_minima_into_ael(&mut self, bot_y: i64) {
        let mut local_minima = NONE;
        //Add any local minima (if any) at BotY ...
        //nb: horizontal local minima edges should contain locMin.vertex.prev

        while self.pop_local_minima(bot_y, &mut local_minima) {
            let lm_vertex = self.local_minima_[local_minima].vertex;
            let mut left_bound;
            let mut right_bound;
            if (self.vertex_lists_[lm_vertex].flags & VertexFlags::OPEN_START) != VertexFlags::EMPTY {
                left_bound = NONE;
            } else {
                left_bound = self.new_active();
                let vertex_top = self.vertex_lists_[lm_vertex].prev; // ie descending
                let top = self.vertex_lists_[vertex_top].pt;
                let lb = &mut self.active_pool_[left_bound];
                lb.bot = self.vertex_lists_[lm_vertex].pt;
                lb.curr_x = lb.bot.x;
                lb.wind_dx = -1;
                lb.vertex_top = vertex_top;
                lb.top = top;
                lb.local_min = local_minima;
                set_dx(lb);
            }

            if (self.vertex_lists_[lm_vertex].flags & VertexFlags::OPEN_END) != VertexFlags::EMPTY {
                right_bound = NONE;
            } else {
                right_bound = self.new_active();
                let vertex_top = self.vertex_lists_[lm_vertex].next; // ie ascending
                let top = self.vertex_lists_[vertex_top].pt;
                let rb = &mut self.active_pool_[right_bound];
                rb.bot = self.vertex_lists_[lm_vertex].pt;
                rb.curr_x = rb.bot.x;
                rb.wind_dx = 1;
                rb.vertex_top = vertex_top;
                rb.top = top;
                rb.local_min = local_minima;
                set_dx(rb);
            }

            //Currently LeftB is just the descending bound and RightB is the ascending.
            //Now if the LeftB isn't on the left of RightB then we need swap them.
            if left_bound != NONE && right_bound != NONE {
                if is_horizontal(&self.active_pool_[left_bound]) {
                    if is_heading_right_horz(&self.active_pool_[left_bound]) {
                        std::mem::swap(&mut left_bound, &mut right_bound);
                    }
                } else if is_horizontal(&self.active_pool_[right_bound]) {
                    if is_heading_left_horz(&self.active_pool_[right_bound]) {
                        std::mem::swap(&mut left_bound, &mut right_bound);
                    }
                } else if self.active_pool_[left_bound].dx < self.active_pool_[right_bound].dx {
                    std::mem::swap(&mut left_bound, &mut right_bound);
                }
            } else if left_bound == NONE {
                left_bound = right_bound;
                right_bound = NONE;
            }

            let contributing;
            self.active_pool_[left_bound].is_left_bound = true;
            self.insert_left_edge(left_bound);

            if self.is_open(left_bound) {
                self.set_wind_count_for_open_path_edge(left_bound);
                contributing = self.is_contributing_open(left_bound);
            } else {
                self.set_wind_count_for_closed_path_edge(left_bound);
                contributing = self.is_contributing_closed(left_bound);
            }

            if right_bound != NONE {
                self.active_pool_[right_bound].is_left_bound = false;
                self.active_pool_[right_bound].wind_cnt = self.active_pool_[left_bound].wind_cnt;
                self.active_pool_[right_bound].wind_cnt2 = self.active_pool_[left_bound].wind_cnt2;
                self.insert_right_edge(left_bound, right_bound); ///////
                if contributing {
                    let bot = self.active_pool_[left_bound].bot;
                    self.add_local_min_poly(left_bound, right_bound, &bot, true);
                    if !is_horizontal(&self.active_pool_[left_bound]) {
                        self.check_join_left(left_bound, &bot, false);
                    }
                }

                while self.active_pool_[right_bound].next_in_ael != NONE
                    && self.is_valid_ael_order(self.active_pool_[right_bound].next_in_ael, right_bound)
                {
                    let next = self.active_pool_[right_bound].next_in_ael;
                    let bot = self.active_pool_[right_bound].bot;
                    self.intersect_edges(right_bound, next, &bot);
                    self.swap_positions_in_ael(right_bound, next);
                }

                if is_horizontal(&self.active_pool_[right_bound]) {
                    self.push_horz(right_bound);
                } else {
                    let bot = self.active_pool_[right_bound].bot;
                    self.check_join_right(right_bound, &bot, false);
                    self.insert_scanline(self.active_pool_[right_bound].top.y);
                }
            } else if contributing {
                let bot = self.active_pool_[left_bound].bot;
                self.start_open_path(left_bound, &bot);
            }

            if is_horizontal(&self.active_pool_[left_bound]) {
                self.push_horz(left_bound);
            } else {
                self.insert_scanline(self.active_pool_[left_bound].top.y);
            }
        } // while (PopLocalMinima())
    }

    /// clipper.engine.cpp:1325-1329
    /// C++: inline void ClipperBase::PushHorz(Active& e)
    #[inline]
    fn push_horz(&mut self, e: usize) {
        self.active_pool_[e].next_in_sel = self.sel_;
        self.sel_ = e;
    }

    /// clipper.engine.cpp:1332-1338
    /// C++: inline bool ClipperBase::PopHorz(Active*& e)
    #[inline]
    fn pop_horz(&mut self, e: &mut usize) -> bool {
        *e = self.sel_;
        if *e == NONE {
            return false;
        }
        self.sel_ = self.active_pool_[self.sel_].next_in_sel;
        true
    }

    /// clipper.engine.cpp:1341-1386
    /// C++: OutPt* ClipperBase::AddLocalMinPoly(Active& e1, Active& e2, const Point64& pt, bool is_new)
    fn add_local_min_poly(&mut self, e1: usize, e2: usize, pt: &Point64, is_new: bool) -> usize {
        let outrec = self.new_out_rec();
        self.active_pool_[e1].outrec = outrec;
        self.active_pool_[e2].outrec = outrec;

        if self.is_open(e1) {
            self.outrec_list_[outrec].owner = NONE;
            self.outrec_list_[outrec].is_open = true;
            if self.active_pool_[e1].wind_dx > 0 {
                self.set_sides(outrec, e1, e2);
            } else {
                self.set_sides(outrec, e2, e1);
            }
        } else {
            let prev_hot_edge = self.get_prev_hot_edge(e1);
            //e.windDx is the winding direction of the **input** paths
            //and unrelated to the winding direction of output polygons.
            //Output orientation is determined by e.outrec.frontE which is
            //the ascending edge (see AddLocalMinPoly).
            if prev_hot_edge != NONE {
                if self.using_polytree_ {
                    self.set_owner(outrec, self.active_pool_[prev_hot_edge].outrec);
                }
                if self.outrec_is_ascending(prev_hot_edge) == is_new {
                    self.set_sides(outrec, e2, e1);
                } else {
                    self.set_sides(outrec, e1, e2);
                }
            } else {
                self.outrec_list_[outrec].owner = NONE;
                if is_new {
                    self.set_sides(outrec, e1, e2);
                } else {
                    self.set_sides(outrec, e2, e1);
                }
            }
        }

        let op = self.new_out_pt(*pt, outrec);
        self.outrec_list_[outrec].pts = op;
        op
    }

    /// clipper.engine.cpp:1389-1442
    /// C++: OutPt* ClipperBase::AddLocalMaxPoly(Active& e1, Active& e2, const Point64& pt)
    fn add_local_max_poly(&mut self, e1: usize, e2: usize, pt: &Point64) -> usize {
        if self.is_joined(e1) {
            self.split(e1, pt);
        }
        if self.is_joined(e2) {
            self.split(e2, pt);
        }

        if self.is_front(e1) == self.is_front(e2) {
            if self.is_open_end(e1) {
                self.swap_front_back_sides(self.active_pool_[e1].outrec);
            } else if self.is_open_end(e2) {
                self.swap_front_back_sides(self.active_pool_[e2].outrec);
            } else {
                self.succeeded_ = false;
                self.stats.failures += 1;
                return NONE;
            }
        }

        let mut result = self.add_out_pt(e1, pt);
        if self.active_pool_[e1].outrec == self.active_pool_[e2].outrec {
            let outrec = self.active_pool_[e1].outrec;
            self.outrec_list_[outrec].pts = result;

            if self.using_polytree_ {
                let e = self.get_prev_hot_edge(e1);
                if e == NONE {
                    self.outrec_list_[outrec].owner = NONE;
                } else {
                    self.set_owner(outrec, self.active_pool_[e].outrec);
                }
                // nb: outRec.owner here is likely NOT the real
                // owner but this will be checked in RecursiveCheckOwners()
            }

            self.uncouple_out_rec(e1);
            result = self.outrec_list_[outrec].pts;
            let owner = self.outrec_list_[outrec].owner;
            if owner != NONE && self.outrec_list_[owner].front_edge == NONE {
                self.outrec_list_[outrec].owner = self.get_real_out_rec(owner);
            }
        }
        //and to preserve the winding orientation of outrec ...
        else if self.is_open(e1) {
            if self.active_pool_[e1].wind_dx < 0 {
                self.join_outrec_paths(e1, e2);
            } else {
                self.join_outrec_paths(e2, e1);
            }
        } else if self.outrec_list_[self.active_pool_[e1].outrec].idx < self.outrec_list_[self.active_pool_[e2].outrec].idx {
            self.join_outrec_paths(e1, e2);
        } else {
            self.join_outrec_paths(e2, e1);
        }
        result
    }

    /// clipper.engine.cpp:1444-1490
    /// C++: void ClipperBase::JoinOutrecPaths(Active& e1, Active& e2)
    fn join_outrec_paths(&mut self, e1: usize, e2: usize) {
        //join e2 outrec path onto e1 outrec path and then delete e2 outrec path
        //pointers. (NB Only very rarely do the joining ends share the same coords.)
        let or1 = self.active_pool_[e1].outrec;
        let or2 = self.active_pool_[e2].outrec;
        let p1_st = self.outrec_list_[or1].pts;
        let p2_st = self.outrec_list_[or2].pts;
        let p1_end = self.outpt_pool_[p1_st].next;
        let p2_end = self.outpt_pool_[p2_st].next;
        if self.is_front(e1) {
            self.outpt_pool_[p2_end].prev = p1_st;
            self.outpt_pool_[p1_st].next = p2_end;
            self.outpt_pool_[p2_st].next = p1_end;
            self.outpt_pool_[p1_end].prev = p2_st;
            self.outrec_list_[or1].pts = p2_st;
            let front = self.outrec_list_[or2].front_edge;
            self.outrec_list_[or1].front_edge = front;
            if front != NONE {
                self.active_pool_[front].outrec = or1;
            }
        } else {
            self.outpt_pool_[p1_end].prev = p2_st;
            self.outpt_pool_[p2_st].next = p1_end;
            self.outpt_pool_[p1_st].next = p2_end;
            self.outpt_pool_[p2_end].prev = p1_st;
            let back = self.outrec_list_[or2].back_edge;
            self.outrec_list_[or1].back_edge = back;
            if back != NONE {
                self.active_pool_[back].outrec = or1;
            }
        }

        //after joining, the e2.OutRec must contains no vertices ...
        // (`e2.outrec` is read again here: the lines above may have given e2 the outrec
        // of e1, when e2 was the front or back edge of its own.)
        let or2 = self.active_pool_[e2].outrec;
        self.outrec_list_[or2].front_edge = NONE;
        self.outrec_list_[or2].back_edge = NONE;
        self.outrec_list_[or2].pts = NONE;

        if self.is_open_end(e1) {
            let or1 = self.active_pool_[e1].outrec;
            self.outrec_list_[or2].pts = self.outrec_list_[or1].pts;
            self.outrec_list_[or1].pts = NONE;
        } else {
            let or1 = self.active_pool_[e1].outrec;
            self.set_owner(or2, or1);
        }

        //and e1 and e2 are maxima and are about to be dropped from the Actives list.
        self.active_pool_[e1].outrec = NONE;
        self.active_pool_[e2].outrec = NONE;
    }

    /// clipper.engine.cpp:1492-1503
    /// C++: OutRec* ClipperBase::NewOutRec()
    fn new_out_rec(&mut self) -> usize {
        let idx = self.outrec_list_.len();
        self.outrec_list_.push(OutRec {
            idx,
            owner: NONE,
            front_edge: NONE,
            back_edge: NONE,
            pts: NONE,
            polypath: NONE,
            splits: None,
            recursive_split: NONE,
            bounds: Rect64::default(),
            path: Path64::new(),
            is_open: false,
        });
        idx
    }

    /// clipper.engine.cpp:1506-1532
    /// C++: OutPt* ClipperBase::AddOutPt(const Active& e, const Point64& pt)
    fn add_out_pt(&mut self, e: usize, pt: &Point64) -> usize {
        //Outrec.OutPts: a circular doubly-linked-list of POutPt where ...
        //op_front[.Prev]* ~~~> op_back & op_back == op_front.Next
        let outrec = self.active_pool_[e].outrec;
        let to_front = self.is_front(e);
        let op_front = self.outrec_list_[outrec].pts;
        let op_back = self.outpt_pool_[op_front].next;

        if to_front {
            if *pt == self.outpt_pool_[op_front].pt {
                return op_front;
            }
        } else if *pt == self.outpt_pool_[op_back].pt {
            return op_back;
        }

        let new_op = self.new_out_pt(*pt, outrec);
        self.outpt_pool_[op_back].prev = new_op;
        self.outpt_pool_[new_op].prev = op_front;
        self.outpt_pool_[new_op].next = op_back;
        self.outpt_pool_[op_front].next = new_op;
        if to_front {
            self.outrec_list_[outrec].pts = new_op;
        }
        new_op
    }

    /// clipper.engine.cpp:1534-1569
    /// C++: void ClipperBase::CleanCollinear(OutRec* outrec)
    fn clean_collinear(&mut self, outrec: usize) {
        let outrec = self.get_real_out_rec(outrec);
        if outrec == NONE || self.outrec_list_[outrec].is_open {
            return;
        }
        if !self.is_valid_closed_path(self.outrec_list_[outrec].pts) {
            self.dispose_out_pts(outrec);
            return;
        }

        let mut start_op = self.outrec_list_[outrec].pts;
        let mut op2 = start_op;
        loop {
            //NB if preserveCollinear == true, then only remove 180 deg. spikes
            let (prev, next) = (self.outpt_pool_[op2].prev, self.outpt_pool_[op2].next);
            let (prev_pt, pt, next_pt) = (self.outpt_pool_[prev].pt, self.outpt_pool_[op2].pt, self.outpt_pool_[next].pt);
            if is_collinear(&prev_pt, &pt, &next_pt)
                && (pt == prev_pt || pt == next_pt || !self.preserve_collinear_ || dot_product(&prev_pt, &pt, &next_pt) < 0.0)
            {
                if op2 == self.outrec_list_[outrec].pts {
                    self.outrec_list_[outrec].pts = prev;
                }

                self.stats.collinear_removed += 1;
                op2 = self.dispose_out_pt(op2);
                if !self.is_valid_closed_path(op2) {
                    self.dispose_out_pts(outrec);
                    return;
                }
                start_op = op2;
                continue;
            }
            op2 = next;
            if op2 == start_op {
                break;
            }
        }
        self.fix_self_intersects(outrec);
    }

    /// clipper.engine.cpp:1571-1653
    /// C++: void ClipperBase::DoSplitOp(OutRec* outrec, OutPt* splitOp)
    fn do_split_op(&mut self, outrec: usize, split_op: usize) {
        self.stats.split_ops += 1;
        // splitOp.prev -> splitOp &&
        // splitOp.next -> splitOp.next.next are intersecting
        let prev_op = self.outpt_pool_[split_op].prev;
        let split_next = self.outpt_pool_[split_op].next;
        let next_next_op = self.outpt_pool_[split_next].next;
        self.outrec_list_[outrec].pts = prev_op;

        let (prev_pt, split_pt, split_next_pt, next_next_pt) =
            (self.outpt_pool_[prev_op].pt, self.outpt_pool_[split_op].pt, self.outpt_pool_[split_next].pt, self.outpt_pool_[next_next_op].pt);
        let mut ip = Point64::default();
        get_segment_intersect_pt(&prev_pt, &split_pt, &split_next_pt, &next_next_pt, &mut ip);

        if USINGZ && self.z_callback_.is_set() {
            self.z_callback_.call(&prev_pt, &split_pt, &split_next_pt, &next_next_pt, &mut ip);
        }
        let area1 = self.area_op(self.outrec_list_[outrec].pts);
        let abs_area1 = area1.abs();
        if abs_area1 < 2.0 {
            self.dispose_out_pts(outrec);
            return;
        }

        let area2 = area_triangle(&ip, &split_pt, &split_next_pt);
        let abs_area2 = area2.abs();

        // de-link splitOp and splitOp.next from the path
        // while inserting the intersection point
        if ip == prev_pt || ip == next_next_pt {
            self.outpt_pool_[next_next_op].prev = prev_op;
            self.outpt_pool_[prev_op].next = next_next_op;
        } else {
            let new_op2 = self.new_out_pt(ip, self.outpt_pool_[prev_op].outrec);
            self.outpt_pool_[new_op2].prev = prev_op;
            self.outpt_pool_[new_op2].next = next_next_op;
            self.outpt_pool_[next_next_op].prev = new_op2;
            self.outpt_pool_[prev_op].next = new_op2;
        }

        // area1 is the path's area *before* splitting, whereas area2 is
        // the area of the triangle containing splitOp & splitOp.next.
        // So the only way for these areas to have the same sign is if
        // the split triangle is larger than the path containing prevOp or
        // if there's more than one self-intersection.
        if abs_area2 >= 1.0 && (abs_area2 > abs_area1 || (area2 > 0.0) == (area1 > 0.0)) {
            self.stats.split_new_outrecs += 1;
            let new_or = self.new_out_rec();
            self.outrec_list_[new_or].owner = self.outrec_list_[outrec].owner;

            self.outpt_pool_[split_op].outrec = new_or;
            self.outpt_pool_[split_next].outrec = new_or;
            let new_op = self.new_out_pt(ip, new_or);
            self.outpt_pool_[new_op].prev = split_next;
            self.outpt_pool_[new_op].next = split_op;
            self.outrec_list_[new_or].pts = new_op;
            self.outpt_pool_[split_op].prev = new_op;
            self.outpt_pool_[split_next].next = new_op;

            if self.using_polytree_ {
                if self.path1_inside_path2(prev_op, new_op) {
                    self.outrec_list_[new_or].splits = Some(vec![outrec]);
                } else {
                    self.outrec_list_[outrec].splits.get_or_insert_with(Vec::new).push(new_or);
                }
            }
        } else {
            // delete splitOp->next;
            // delete splitOp;
        }
    }

    /// clipper.engine.cpp:1655-1677
    /// C++: void ClipperBase::FixSelfIntersects(OutRec* outrec)
    fn fix_self_intersects(&mut self, outrec: usize) {
        let mut op2 = self.outrec_list_[outrec].pts;
        loop {
            let p = &self.outpt_pool_;
            let (prev, next) = (p[op2].prev, p[op2].next);
            let next_next = p[next].next;
            // triangles can't self-intersect
            if prev == next_next {
                break;
            }
            if segments_intersect(&p[prev].pt, &p[op2].pt, &p[next].pt, &p[next_next].pt, false) {
                if op2 == self.outrec_list_[outrec].pts || next == self.outrec_list_[outrec].pts {
                    self.outrec_list_[outrec].pts = self.outpt_pool_[self.outrec_list_[outrec].pts].prev;
                }
                self.do_split_op(outrec, op2);
                if self.outrec_list_[outrec].pts == NONE {
                    break;
                }
                op2 = self.outrec_list_[outrec].pts;
                continue;
            } else {
                op2 = next;
            }

            if op2 == self.outrec_list_[outrec].pts {
                break;
            }
        }
    }

    /// clipper.engine.cpp:1680-1689
    /// C++: inline void UpdateOutrecOwner(OutRec* outrec)
    #[allow(dead_code)]
    fn update_outrec_owner(&mut self, outrec: usize) {
        let mut op_curr = self.outrec_list_[outrec].pts;
        loop {
            self.outpt_pool_[op_curr].outrec = outrec;
            op_curr = self.outpt_pool_[op_curr].next;
            if op_curr == self.outrec_list_[outrec].pts {
                return;
            }
        }
    }

    /// clipper.engine.cpp:1692-1713
    /// C++: OutPt* ClipperBase::StartOpenPath(Active& e, const Point64& pt)
    fn start_open_path(&mut self, e: usize, pt: &Point64) -> usize {
        let outrec = self.new_out_rec();
        self.outrec_list_[outrec].is_open = true;

        if self.active_pool_[e].wind_dx > 0 {
            self.outrec_list_[outrec].front_edge = e;
            self.outrec_list_[outrec].back_edge = NONE;
        } else {
            self.outrec_list_[outrec].front_edge = NONE;
            self.outrec_list_[outrec].back_edge = e;
        }

        self.active_pool_[e].outrec = outrec;

        let op = self.new_out_pt(*pt, outrec);
        self.outrec_list_[outrec].pts = op;
        op
    }

    /// clipper.engine.cpp:1715-1735
    /// C++: inline void TrimHorz(Active& horzEdge, bool preserveCollinear)
    fn trim_horz(&mut self, horz_edge: usize, preserve_collinear: bool) {
        let mut was_trimmed = false;
        let mut pt = self.vertex_lists_[self.next_vertex(horz_edge)].pt;
        while pt.y == self.active_pool_[horz_edge].top.y {
            //always trim 180 deg. spikes (in closed paths)
            //but otherwise break if preserveCollinear = true
            let he = &self.active_pool_[horz_edge];
            if preserve_collinear && ((pt.x < he.top.x) != (he.bot.x < he.top.x)) {
                break;
            }

            let nv = self.next_vertex(horz_edge);
            self.active_pool_[horz_edge].vertex_top = nv;
            self.active_pool_[horz_edge].top = pt;
            was_trimmed = true;
            if self.is_maxima(horz_edge) {
                break;
            }
            pt = self.vertex_lists_[self.next_vertex(horz_edge)].pt;
        }

        if was_trimmed {
            set_dx(&mut self.active_pool_[horz_edge]); // +/-infinity
        }
    }

    /// clipper.engine.cpp:1738-1757
    /// C++: inline void ClipperBase::UpdateEdgeIntoAEL(Active* e)
    fn update_edge_into_ael(&mut self, e: usize) {
        let nv = self.next_vertex(e);
        let nv_pt = self.vertex_lists_[nv].pt;
        {
            let ae = &mut self.active_pool_[e];
            ae.bot = ae.top;
            ae.vertex_top = nv;
            ae.top = nv_pt;
            ae.curr_x = ae.bot.x;
            set_dx(ae);
        }

        if self.is_joined(e) {
            let bot = self.active_pool_[e].bot;
            self.split(e, &bot);
        }

        if is_horizontal(&self.active_pool_[e]) {
            if !self.is_open(e) {
                self.trim_horz(e, self.preserve_collinear_);
            }
            return;
        }

        self.insert_scanline(self.active_pool_[e].top.y);
        let bot = self.active_pool_[e].bot;
        self.check_join_left(e, &bot, false);
        self.check_join_right(e, &bot, true); // (#500)
    }

    /// clipper.engine.cpp:1759-1776
    /// C++: Active* FindEdgeWithMatchingLocMin(Active* e)
    fn find_edge_with_matching_loc_min(&self, e: usize) -> usize {
        let a = &self.active_pool_;
        let mut result = a[e].next_in_ael;
        while result != NONE {
            if a[result].local_min == a[e].local_min {
                return result;
            } else if !is_horizontal(&a[result]) && a[e].bot != a[result].bot {
                result = NONE;
            } else {
                result = a[result].next_in_ael;
            }
        }
        result = a[e].prev_in_ael;
        while result != NONE {
            if a[result].local_min == a[e].local_min {
                return result;
            } else if !is_horizontal(&a[result]) && a[e].bot != a[result].bot {
                return NONE;
            } else {
                result = a[result].prev_in_ael;
            }
        }
        result
    }

    /// clipper.engine.cpp:1779-2093
    /// C++: void ClipperBase::IntersectEdges(Active& e1, Active& e2, const Point64& pt)
    /// The `#ifdef USINGZ` arms keep the `OutPt` a call returns and pass its point to
    /// `SetZ`; the other arms make the same calls and drop what they return.
    fn intersect_edges(&mut self, e1: usize, e2: usize, pt: &Point64) {
        //MANAGE OPEN PATH INTERSECTIONS SEPARATELY ...
        if self.has_open_paths_ && (self.is_open(e1) || self.is_open(e2)) {
            if self.is_open(e1) && self.is_open(e2) {
                return;
            }
            let (edge_o, edge_c) = if self.is_open(e1) { (e1, e2) } else { (e2, e1) };
            if self.is_joined(edge_c) {
                self.split(edge_c, pt); // needed for safety
            }

            if self.active_pool_[edge_c].wind_cnt.abs() != 1 {
                return;
            }
            match self.cliptype_ {
                ClipType::Union => {
                    if !self.is_hot_edge(edge_c) {
                        return;
                    }
                }
                _ => {
                    if self.local_minima_[self.active_pool_[edge_c].local_min].polytype == PathType::Subject {
                        return;
                    }
                }
            }

            match self.fillrule_ {
                FillRule::Positive => {
                    if self.active_pool_[edge_c].wind_cnt != 1 {
                        return;
                    }
                }
                FillRule::Negative => {
                    if self.active_pool_[edge_c].wind_cnt != -1 {
                        return;
                    }
                }
                _ => {
                    if self.active_pool_[edge_c].wind_cnt.abs() != 1 {
                        return;
                    }
                }
            }

            let result_op;
            //toggle contribution ...
            if self.is_hot_edge(edge_o) {
                result_op = self.add_out_pt(edge_o, pt);
                let outrec = self.active_pool_[edge_o].outrec;
                if self.is_front(edge_o) {
                    self.outrec_list_[outrec].front_edge = NONE;
                } else {
                    self.outrec_list_[outrec].back_edge = NONE;
                }
                self.active_pool_[edge_o].outrec = NONE;
            }
            //horizontal edges can pass under open paths at a LocMins
            else if *pt == self.vertex_lists_[self.local_minima_[self.active_pool_[edge_o].local_min].vertex].pt
                && !self.is_open_end_vertex(self.local_minima_[self.active_pool_[edge_o].local_min].vertex)
            {
                //find the other side of the LocMin and
                //if it's 'hot' join up with it ...
                let e3 = self.find_edge_with_matching_loc_min(edge_o);
                if e3 != NONE && self.is_hot_edge(e3) {
                    let e3_outrec = self.active_pool_[e3].outrec;
                    self.active_pool_[edge_o].outrec = e3_outrec;
                    if self.active_pool_[edge_o].wind_dx > 0 {
                        self.set_sides(e3_outrec, edge_o, e3);
                    } else {
                        self.set_sides(e3_outrec, e3, edge_o);
                    }
                    return;
                } else {
                    result_op = self.start_open_path(edge_o, pt);
                }
            } else {
                result_op = self.start_open_path(edge_o, pt);
            }

            if USINGZ && self.z_callback_.is_set() {
                self.set_z(edge_o, edge_c, result_op);
            }
            return;
        } // end of an open path intersection

        //MANAGING CLOSED PATHS FROM HERE ON

        if self.is_joined(e1) {
            self.split(e1, pt);
        }
        if self.is_joined(e2) {
            self.split(e2, pt);
        }

        //UPDATE WINDING COUNTS...

        let old_e1_windcnt: i32;
        let old_e2_windcnt: i32;
        if self.local_minima_[self.active_pool_[e1].local_min].polytype == self.local_minima_[self.active_pool_[e2].local_min].polytype {
            if self.fillrule_ == FillRule::EvenOdd {
                let old = self.active_pool_[e1].wind_cnt;
                self.active_pool_[e1].wind_cnt = self.active_pool_[e2].wind_cnt;
                self.active_pool_[e2].wind_cnt = old;
            } else {
                if self.active_pool_[e1].wind_cnt + self.active_pool_[e2].wind_dx == 0 {
                    self.active_pool_[e1].wind_cnt = -self.active_pool_[e1].wind_cnt;
                } else {
                    self.active_pool_[e1].wind_cnt += self.active_pool_[e2].wind_dx;
                }
                if self.active_pool_[e2].wind_cnt - self.active_pool_[e1].wind_dx == 0 {
                    self.active_pool_[e2].wind_cnt = -self.active_pool_[e2].wind_cnt;
                } else {
                    self.active_pool_[e2].wind_cnt -= self.active_pool_[e1].wind_dx;
                }
            }
        } else if self.fillrule_ != FillRule::EvenOdd {
            self.active_pool_[e1].wind_cnt2 += self.active_pool_[e2].wind_dx;
            self.active_pool_[e2].wind_cnt2 -= self.active_pool_[e1].wind_dx;
        } else {
            self.active_pool_[e1].wind_cnt2 = if self.active_pool_[e1].wind_cnt2 == 0 { 1 } else { 0 };
            self.active_pool_[e2].wind_cnt2 = if self.active_pool_[e2].wind_cnt2 == 0 { 1 } else { 0 };
        }

        match self.fillrule_ {
            FillRule::EvenOdd | FillRule::NonZero => {
                old_e1_windcnt = self.active_pool_[e1].wind_cnt.abs();
                old_e2_windcnt = self.active_pool_[e2].wind_cnt.abs();
            }
            _ => {
                if self.fillrule_ == self.fillpos {
                    old_e1_windcnt = self.active_pool_[e1].wind_cnt;
                    old_e2_windcnt = self.active_pool_[e2].wind_cnt;
                } else {
                    old_e1_windcnt = -self.active_pool_[e1].wind_cnt;
                    old_e2_windcnt = -self.active_pool_[e2].wind_cnt;
                }
            }
        }

        let e1_windcnt_in_01 = old_e1_windcnt == 0 || old_e1_windcnt == 1;
        let e2_windcnt_in_01 = old_e2_windcnt == 0 || old_e2_windcnt == 1;

        if (!self.is_hot_edge(e1) && !e1_windcnt_in_01) || (!self.is_hot_edge(e2) && !e2_windcnt_in_01) {
            return;
        }

        //NOW PROCESS THE INTERSECTION ...
        let z = USINGZ && self.z_callback_.is_set();
        //if both edges are 'hot' ...
        if self.is_hot_edge(e1) && self.is_hot_edge(e2) {
            if (old_e1_windcnt != 0 && old_e1_windcnt != 1)
                || (old_e2_windcnt != 0 && old_e2_windcnt != 1)
                || (self.local_minima_[self.active_pool_[e1].local_min].polytype
                    != self.local_minima_[self.active_pool_[e2].local_min].polytype
                    && self.cliptype_ != ClipType::Xor)
            {
                let result_op = self.add_local_max_poly(e1, e2, pt);
                if z && result_op != NONE {
                    self.set_z(e1, e2, result_op);
                }
            } else if self.is_front(e1) || (self.active_pool_[e1].outrec == self.active_pool_[e2].outrec) {
                //this 'else if' condition isn't strictly needed but
                //it's sensible to split polygons that only touch at
                //a common vertex (not at common edges).

                let result_op = self.add_local_max_poly(e1, e2, pt);
                let op2 = self.add_local_min_poly(e1, e2, pt, false);
                if z && result_op != NONE {
                    self.set_z(e1, e2, result_op);
                }
                if z {
                    self.set_z(e1, e2, op2);
                }
            } else {
                let result_op = self.add_out_pt(e1, pt);
                let op2 = self.add_out_pt(e2, pt);
                if z {
                    self.set_z(e1, e2, result_op);
                    self.set_z(e1, e2, op2);
                }
                self.swap_outrecs(e1, e2);
            }
        } else if self.is_hot_edge(e1) {
            let result_op = self.add_out_pt(e1, pt);
            if z {
                self.set_z(e1, e2, result_op);
            }
            self.swap_outrecs(e1, e2);
        } else if self.is_hot_edge(e2) {
            let result_op = self.add_out_pt(e2, pt);
            if z {
                self.set_z(e1, e2, result_op);
            }
            self.swap_outrecs(e1, e2);
        } else {
            let e1_wc2: i64;
            let e2_wc2: i64;
            match self.fillrule_ {
                FillRule::EvenOdd | FillRule::NonZero => {
                    e1_wc2 = self.active_pool_[e1].wind_cnt2.abs() as i64;
                    e2_wc2 = self.active_pool_[e2].wind_cnt2.abs() as i64;
                }
                _ => {
                    if self.fillrule_ == self.fillpos {
                        e1_wc2 = self.active_pool_[e1].wind_cnt2 as i64;
                        e2_wc2 = self.active_pool_[e2].wind_cnt2 as i64;
                    } else {
                        e1_wc2 = -self.active_pool_[e1].wind_cnt2 as i64;
                        e2_wc2 = -self.active_pool_[e2].wind_cnt2 as i64;
                    }
                }
            }

            if !self.is_same_poly_type(e1, e2) {
                let result_op = self.add_local_min_poly(e1, e2, pt, false);
                if z {
                    self.set_z(e1, e2, result_op);
                }
            } else if old_e1_windcnt == 1 && old_e2_windcnt == 1 {
                let mut result_op = NONE;
                match self.cliptype_ {
                    ClipType::Union => {
                        if e1_wc2 <= 0 && e2_wc2 <= 0 {
                            result_op = self.add_local_min_poly(e1, e2, pt, false);
                        }
                    }
                    ClipType::Difference => {
                        if ((self.get_poly_type(e1) == PathType::Clip) && (e1_wc2 > 0) && (e2_wc2 > 0))
                            || ((self.get_poly_type(e1) == PathType::Subject) && (e1_wc2 <= 0) && (e2_wc2 <= 0))
                        {
                            result_op = self.add_local_min_poly(e1, e2, pt, false);
                        }
                    }
                    ClipType::Xor => {
                        result_op = self.add_local_min_poly(e1, e2, pt, false);
                    }
                    _ => {
                        if e1_wc2 > 0 && e2_wc2 > 0 {
                            result_op = self.add_local_min_poly(e1, e2, pt, false);
                        }
                    }
                }
                if result_op != NONE && z {
                    self.set_z(e1, e2, result_op);
                }
            }
        }
    }

    /// clipper.engine.cpp:2095-2106
    /// C++: inline void ClipperBase::DeleteFromAEL(Active& e)
    fn delete_from_ael(&mut self, e: usize) {
        let prev = self.active_pool_[e].prev_in_ael;
        let next = self.active_pool_[e].next_in_ael;
        if prev == NONE && next == NONE && (e != self.actives_) {
            return; // already deleted
        }
        if prev != NONE {
            self.active_pool_[prev].next_in_ael = next;
        } else {
            self.actives_ = next;
        }
        if next != NONE {
            self.active_pool_[next].prev_in_ael = prev;
        }
        // delete& e;
    }

    /// clipper.engine.cpp:2109-2124
    /// C++: inline void ClipperBase::AdjustCurrXAndCopyToSEL(const int64_t top_y)
    fn adjust_curr_x_and_copy_to_sel(&mut self, top_y: i64) {
        let mut e = self.actives_;
        self.sel_ = e;
        while e != NONE {
            let prev = self.active_pool_[e].prev_in_ael;
            let next = self.active_pool_[e].next_in_ael;
            self.active_pool_[e].prev_in_sel = prev;
            self.active_pool_[e].next_in_sel = next;
            self.active_pool_[e].jump = next;
            if self.active_pool_[e].join_with == JoinWith::Left {
                self.active_pool_[e].curr_x = self.active_pool_[prev].curr_x; // also avoids complications
            } else {
                self.active_pool_[e].curr_x = top_x(&self.active_pool_[e], top_y);
            }
            e = next;
        }
    }

    /// clipper.engine.cpp:2126-2153
    /// C++: bool ClipperBase::ExecuteInternal(ClipType ct, FillRule fillrule, bool use_polytrees)
    pub fn execute_internal(&mut self, ct: ClipType, fillrule: FillRule, use_polytrees: bool) -> bool {
        self.stats = ClipperStats::default();
        self.cliptype_ = ct;
        self.fillrule_ = fillrule;
        self.using_polytree_ = use_polytrees;
        self.reset();
        let mut y = 0i64;
        if ct == ClipType::NoClip || !self.pop_scanline(&mut y) {
            return true;
        }

        while self.succeeded_ {
            self.insert_local_minima_into_ael(y);
            let mut e = NONE;
            while self.pop_horz(&mut e) {
                self.do_horizontal(e);
            }
            if !self.horz_seg_list_.is_empty() {
                self.convert_horz_segs_to_joins();
                self.horz_seg_list_.clear();
            }
            self.bot_y_ = y; // bot_y_ == bottom of scanbeam
            if !self.pop_scanline(&mut y) {
                break; // y new top of scanbeam
            }
            self.do_intersections(y);
            self.do_top_of_scanbeam(y);
            while self.pop_horz(&mut e) {
                self.do_horizontal(e);
            }
        }
        if self.succeeded_ {
            self.process_horz_joins();
        }
        self.succeeded_
    }

    /// clipper.engine.cpp:2155-2162
    /// C++: inline void FixOutRecPts(OutRec* outrec)
    fn fix_out_rec_pts(&mut self, outrec: usize) {
        let mut op = self.outrec_list_[outrec].pts;
        loop {
            self.outpt_pool_[op].outrec = outrec;
            op = self.outpt_pool_[op].next;
            if op == self.outrec_list_[outrec].pts {
                break;
            }
        }
    }

    /// clipper.engine.cpp:2182-2213
    /// C++: inline bool UpdateHorzSegment(HorzSegment& hs)
    fn update_horz_segment(&mut self, hs_index: usize) -> bool {
        let mut hs = self.horz_seg_list_[hs_index];
        let p = &self.outpt_pool_;
        let op = hs.left_op;
        let outrec = self.get_real_out_rec(p[op].outrec);
        let outrec_has_edges = self.outrec_list_[outrec].front_edge != NONE;
        let curr_y = p[op].pt.y;
        let mut op_p = op;
        let mut op_n = op;
        if outrec_has_edges {
            let op_a = self.outrec_list_[outrec].pts;
            let op_z = p[op_a].next;
            while op_p != op_z && p[p[op_p].prev].pt.y == curr_y {
                op_p = p[op_p].prev;
            }
            while op_n != op_a && p[p[op_n].next].pt.y == curr_y {
                op_n = p[op_n].next;
            }
        } else {
            while p[op_p].prev != op_n && p[p[op_p].prev].pt.y == curr_y {
                op_p = p[op_p].prev;
            }
            while p[op_n].next != op_p && p[p[op_n].next].pt.y == curr_y {
                op_n = p[op_n].next;
            }
        }
        let result = set_horz_seg_heading_forward(&mut hs, op_p, op_n, p[op_p].pt.x, p[op_n].pt.x) && !p[hs.left_op].horz;

        if result {
            self.outpt_pool_[hs.left_op].horz = true;
        } else {
            hs.right_op = NONE; // (for sorting)
        }
        self.horz_seg_list_[hs_index] = hs;
        result
    }

    /// clipper.engine.cpp:2215-2264
    /// C++: void ClipperBase::ConvertHorzSegsToJoins()
    fn convert_horz_segs_to_joins(&mut self) {
        let mut j = 0usize;
        for i in 0..self.horz_seg_list_.len() {
            if self.update_horz_segment(i) {
                j += 1;
            }
        }
        if j < 2 {
            return;
        }

        // std::stable_sort with HorzSegSorter (clipper.engine.cpp:48-54): the segments
        // with a right_op first, by the x of their left_op; the others after them.
        {
            let p = &self.outpt_pool_;
            self.horz_seg_list_.sort_by(|hs1, hs2| {
                use std::cmp::Ordering;
                match (hs1.right_op != NONE, hs2.right_op != NONE) {
                    (true, true) => p[hs1.left_op].pt.x.cmp(&p[hs2.left_op].pt.x),
                    (true, false) => Ordering::Less,
                    (false, true) => Ordering::Greater,
                    (false, false) => Ordering::Equal,
                }
            });
        }

        let hs_end = j;
        let hs_end1 = hs_end - 1;

        for hs1 in 0..hs_end1 {
            for hs2 in (hs1 + 1)..hs_end {
                let (s1, s2) = (self.horz_seg_list_[hs1], self.horz_seg_list_[hs2]);
                let p = &self.outpt_pool_;
                if (p[s2.left_op].pt.x >= p[s1.right_op].pt.x)
                    || (s2.left_to_right == s1.left_to_right)
                    || (p[s2.right_op].pt.x <= p[s1.left_op].pt.x)
                {
                    continue;
                }
                let curr_y = p[s1.left_op].pt.y;
                let mut l1 = s1.left_op;
                let mut l2 = s2.left_op;
                if s1.left_to_right {
                    while p[p[l1].next].pt.y == curr_y && p[p[l1].next].pt.x <= p[l2].pt.x {
                        l1 = p[l1].next;
                    }
                    while p[p[l2].prev].pt.y == curr_y && p[p[l2].prev].pt.x <= p[l1].pt.x {
                        l2 = p[l2].prev;
                    }
                    self.horz_seg_list_[hs1].left_op = l1;
                    self.horz_seg_list_[hs2].left_op = l2;
                    let op1 = self.duplicate_op(l1, true);
                    let op2 = self.duplicate_op(l2, false);
                    self.horz_join_list_.push(HorzJoin { op1, op2 });
                } else {
                    while p[p[l1].prev].pt.y == curr_y && p[p[l1].prev].pt.x <= p[l2].pt.x {
                        l1 = p[l1].prev;
                    }
                    while p[p[l2].next].pt.y == curr_y && p[p[l2].next].pt.x <= p[l1].pt.x {
                        l2 = p[l2].next;
                    }
                    self.horz_seg_list_[hs1].left_op = l1;
                    self.horz_seg_list_[hs2].left_op = l2;
                    let op1 = self.duplicate_op(l2, true);
                    let op2 = self.duplicate_op(l1, false);
                    self.horz_join_list_.push(HorzJoin { op1, op2 });
                }
            }
        }
    }

    /// clipper.engine.cpp:2266-2274
    /// C++: void MoveSplits(OutRec* fromOr, OutRec* toOr)
    fn move_splits(&mut self, from_or: usize, to_or: usize) {
        if self.outrec_list_[from_or].splits.is_none() {
            return;
        }
        // Every element of fromOr->splits is appended to toOr->splits, and then
        // fromOr->splits is emptied (it stays allocated).
        let moved = std::mem::take(self.outrec_list_[from_or].splits.as_mut().unwrap());
        self.outrec_list_[to_or].splits.get_or_insert_with(Vec::new).extend(moved);
    }

    /// clipper.engine.cpp:2277-2342
    /// C++: void ClipperBase::ProcessHorzJoins()
    fn process_horz_joins(&mut self) {
        for ji in 0..self.horz_join_list_.len() {
            let j = self.horz_join_list_[ji];
            self.stats.horz_joins += 1;
            let or1 = self.get_real_out_rec(self.outpt_pool_[j.op1].outrec);
            let mut or2 = self.get_real_out_rec(self.outpt_pool_[j.op2].outrec);

            let op1b = self.outpt_pool_[j.op1].next;
            let op2b = self.outpt_pool_[j.op2].prev;
            self.outpt_pool_[j.op1].next = j.op2;
            self.outpt_pool_[j.op2].prev = j.op1;
            self.outpt_pool_[op1b].prev = op2b;
            self.outpt_pool_[op2b].next = op1b;

            if or1 == or2 {
                // 'join' is really a split
                self.stats.horz_join_splits += 1;
                or2 = self.new_out_rec();
                self.outrec_list_[or2].pts = op1b;
                self.fix_out_rec_pts(or2);

                //if or1->pts has moved to or2 then update or1->pts!!
                if self.outpt_pool_[self.outrec_list_[or1].pts].outrec == or2 {
                    self.outrec_list_[or1].pts = j.op1;
                    self.outpt_pool_[j.op1].outrec = or1;
                }

                if self.using_polytree_ {
                    //#498, #520, #584, D#576, #618
                    if self.path1_inside_path2(self.outrec_list_[or1].pts, self.outrec_list_[or2].pts) {
                        //swap or1's & or2's pts
                        let tmp = self.outrec_list_[or1].pts;
                        self.outrec_list_[or1].pts = self.outrec_list_[or2].pts;
                        self.outrec_list_[or2].pts = tmp;
                        self.fix_out_rec_pts(or1);
                        self.fix_out_rec_pts(or2);
                        //or2 is now inside or1
                        self.outrec_list_[or2].owner = or1;
                    } else if self.path1_inside_path2(self.outrec_list_[or2].pts, self.outrec_list_[or1].pts) {
                        self.outrec_list_[or2].owner = or1;
                    } else {
                        self.outrec_list_[or2].owner = self.outrec_list_[or1].owner;
                    }

                    self.outrec_list_[or1].splits.get_or_insert_with(Vec::new).push(or2);
                } else {
                    self.outrec_list_[or2].owner = or1;
                }
            } else {
                self.outrec_list_[or2].pts = NONE;
                if self.using_polytree_ {
                    self.set_owner(or2, or1);
                    self.move_splits(or2, or1); //#618
                } else {
                    self.outrec_list_[or2].owner = or1;
                }
            }
        }
    }

    /// clipper.engine.cpp:2344-2351
    /// C++: void ClipperBase::DoIntersections(const int64_t top_y)
    fn do_intersections(&mut self, top_y: i64) {
        if self.build_intersect_list(top_y) {
            self.process_intersect_list();
            self.intersect_nodes_.clear();
        }
    }

    /// clipper.engine.cpp:2353-2385
    /// C++: void ClipperBase::AddNewIntersectNode(Active& e1, Active& e2, int64_t top_y)
    fn add_new_intersect_node(&mut self, e1: usize, e2: usize, top_y: i64) {
        let a1 = self.active_pool_[e1];
        let a2 = self.active_pool_[e2];
        let mut ip = Point64::default();
        if !get_segment_intersect_pt(&a1.bot, &a1.top, &a2.bot, &a2.top, &mut ip) {
            ip = Point64::new(a1.curr_x, top_y); //parallel edges
        }

        //rounding errors can occasionally place the calculated intersection
        //point either below or above the scanbeam, so check and correct ...
        if ip.y > self.bot_y_ || ip.y < top_y {
            self.stats.intersect_corrections += 1;
            let abs_dx1 = a1.dx.abs();
            let abs_dx2 = a2.dx.abs();
            if abs_dx1 > 100.0 && abs_dx2 > 100.0 {
                if abs_dx1 > abs_dx2 {
                    ip = get_closest_point_on_segment(&ip, &a1.bot, &a1.top);
                } else {
                    ip = get_closest_point_on_segment(&ip, &a2.bot, &a2.top);
                }
            } else if abs_dx1 > 100.0 {
                ip = get_closest_point_on_segment(&ip, &a1.bot, &a1.top);
            } else if abs_dx2 > 100.0 {
                ip = get_closest_point_on_segment(&ip, &a2.bot, &a2.top);
            } else {
                if ip.y < top_y {
                    ip.y = top_y;
                } else {
                    ip.y = self.bot_y_;
                }
                if abs_dx1 < abs_dx2 {
                    ip.x = top_x(&a1, ip.y);
                } else {
                    ip.x = top_x(&a2, ip.y);
                }
            }
        }
        self.intersect_nodes_.push(IntersectNode { pt: ip, edge1: e1, edge2: e2 });
    }

    /// clipper.engine.cpp:2387-2443
    /// C++: bool ClipperBase::BuildIntersectList(const int64_t top_y)
    fn build_intersect_list(&mut self, top_y: i64) -> bool {
        if self.actives_ == NONE || self.active_pool_[self.actives_].next_in_ael == NONE {
            return false;
        }

        //Calculate edge positions at the top of the current scanbeam, and from this
        //we will determine the intersections required to reach these new positions.
        self.adjust_curr_x_and_copy_to_sel(top_y);
        //Find all edge intersections in the current scanbeam using a stable merge
        //sort that ensures only adjacent edges are intersecting. Intersect info is
        //stored in FIntersectList ready to be processed in ProcessIntersectList.
        //Re merge sorts see https://stackoverflow.com/a/46319131/359538

        let mut left = self.sel_;

        while left != NONE && self.active_pool_[left].jump != NONE {
            let mut prev_base = NONE;
            while left != NONE && self.active_pool_[left].jump != NONE {
                let mut curr_base = left;
                let mut right = self.active_pool_[left].jump;
                let mut l_end = right;
                let r_end = self.active_pool_[right].jump;
                self.active_pool_[left].jump = r_end;
                while left != l_end && right != r_end {
                    if self.active_pool_[right].curr_x < self.active_pool_[left].curr_x {
                        let mut tmp = self.active_pool_[right].prev_in_sel;
                        loop {
                            self.add_new_intersect_node(tmp, right, top_y);
                            if tmp == left {
                                break;
                            }
                            tmp = self.active_pool_[tmp].prev_in_sel;
                        }

                        tmp = right;
                        right = self.extract_from_sel(tmp);
                        l_end = right;
                        self.insert1_before2_in_sel(tmp, left);
                        if left == curr_base {
                            curr_base = tmp;
                            self.active_pool_[curr_base].jump = r_end;
                            if prev_base == NONE {
                                self.sel_ = curr_base;
                            } else {
                                self.active_pool_[prev_base].jump = curr_base;
                            }
                        }
                    } else {
                        left = self.active_pool_[left].next_in_sel;
                    }
                }
                prev_base = curr_base;
                left = r_end;
            }
            left = self.sel_;
        }
        !self.intersect_nodes_.is_empty()
    }

    /// clipper.engine.cpp:2445-2477
    /// C++: void ClipperBase::ProcessIntersectList()
    fn process_intersect_list(&mut self) {
        //We now have a list of intersections required so that edges will be
        //correctly positioned at the top of the scanbeam. However, it's important
        //that edge intersections are processed from the bottom up, but it's also
        //crucial that intersections only occur between adjacent edges.

        //First we do a quicksort so intersections proceed in a bottom up order ...
        // `std::sort` of libc++, which is not stable: the order of nodes at one point is
        // what that algorithm leaves.
        self.stats.intersect_list_max = self.stats.intersect_list_max.max(self.intersect_nodes_.len() as u64);
        crate::clipper::libcxx_sort(&mut self.intersect_nodes_, intersect_list_sort);
        //Now as we process these intersections, we must sometimes adjust the order
        //to ensure that intersecting edges are always adjacent ...

        for node_iter in 0..self.intersect_nodes_.len() {
            if !self.edges_adjacent_in_ael(&self.intersect_nodes_[node_iter]) {
                let mut node_iter2 = node_iter + 1;
                while !self.edges_adjacent_in_ael(&self.intersect_nodes_[node_iter2]) {
                    node_iter2 += 1;
                }
                self.stats.intersect_reorders += 1;
                self.intersect_nodes_.swap(node_iter, node_iter2);
            }

            let node = self.intersect_nodes_[node_iter];
            self.intersect_edges(node.edge1, node.edge2, &node.pt);
            self.swap_positions_in_ael(node.edge1, node.edge2);

            self.active_pool_[node.edge1].curr_x = node.pt.x;
            self.active_pool_[node.edge2].curr_x = node.pt.x;
            self.check_join_left(node.edge2, &node.pt, true);
            self.check_join_right(node.edge1, &node.pt, true);
        }
    }

    /// clipper.engine.cpp:2479-2491
    /// C++: void ClipperBase::SwapPositionsInAEL(Active& e1, Active& e2)
    fn swap_positions_in_ael(&mut self, e1: usize, e2: usize) {
        //preconditon: e1 must be immediately to the left of e2
        let next = self.active_pool_[e2].next_in_ael;
        if next != NONE {
            self.active_pool_[next].prev_in_ael = e1;
        }
        let prev = self.active_pool_[e1].prev_in_ael;
        if prev != NONE {
            self.active_pool_[prev].next_in_ael = e2;
        }
        self.active_pool_[e2].prev_in_ael = prev;
        self.active_pool_[e2].next_in_ael = e1;
        self.active_pool_[e1].prev_in_ael = e2;
        self.active_pool_[e1].next_in_ael = next;
        if self.active_pool_[e2].prev_in_ael == NONE {
            self.actives_ = e2;
        }
    }

    /// clipper.engine.cpp:2493-2500
    /// C++: inline OutPt* GetLastOp(const Active& hot_edge)
    fn get_last_op(&self, hot_edge: usize) -> usize {
        let outrec = self.active_pool_[hot_edge].outrec;
        let mut result = self.outrec_list_[outrec].pts;
        if hot_edge != self.outrec_list_[outrec].front_edge {
            result = self.outpt_pool_[result].next;
        }
        result
    }

    /// clipper.engine.cpp:2502-2506
    /// C++: void ClipperBase::AddTrialHorzJoin(OutPt* op)
    fn add_trial_horz_join(&mut self, op: usize) {
        if self.outrec_list_[self.outpt_pool_[op].outrec].is_open {
            return;
        }
        self.horz_seg_list_.push(HorzSegment { left_op: op, right_op: NONE, left_to_right: true });
    }

    /// clipper.engine.cpp:2508-2532
    /// C++: bool ClipperBase::ResetHorzDirection(const Active& horz, const Vertex* max_vertex, int64_t& horz_left, int64_t& horz_right)
    fn reset_horz_direction(&self, horz: usize, max_vertex: usize, horz_left: &mut i64, horz_right: &mut i64) -> bool {
        let h = &self.active_pool_[horz];
        if h.bot.x == h.top.x {
            //the horizontal edge is going nowhere ...
            *horz_left = h.curr_x;
            *horz_right = h.curr_x;
            let mut e = h.next_in_ael;
            while e != NONE && self.active_pool_[e].vertex_top != max_vertex {
                e = self.active_pool_[e].next_in_ael;
            }
            e != NONE
        } else if h.curr_x < h.top.x {
            *horz_left = h.curr_x;
            *horz_right = h.top.x;
            true
        } else {
            *horz_left = h.top.x;
            *horz_right = h.curr_x;
            false // right to left
        }
    }

    /// clipper.engine.cpp:2534-2703
    /// C++: void ClipperBase::DoHorizontal(Active& horz)
    /// Notes: Horizontal edges (HEs) at scanline intersections (ie at the top or
    /// bottom of a scanbeam) are processed as if layered.The order in which HEs
    /// are processed doesn't matter. HEs intersect with the bottom vertices of
    /// other HEs[#] and with non-horizontal edges [*]. Once these intersections
    /// are completed, intermediate HEs are 'promoted' to the next edge in their
    /// bounds, and they in turn may be intersected[%] by other HEs.
    ///
    /// eg: 3 horizontals at a scanline:    /   |                     /           /
    ///              |                     /    |     (HE3)o ========%========== o
    ///              o ======= o(HE2)     /     |         /         /
    ///          o ============#=========*======*========#=========o (HE1)
    ///         /              |        /       |       /
    fn do_horizontal(&mut self, horz: usize) {
        self.stats.horizontals += 1;
        let mut pt;
        let horz_is_open = self.is_open(horz);
        let y = self.active_pool_[horz].bot.y;
        let vertex_max =
            if horz_is_open { self.get_curr_y_maxima_vertex_open(horz) } else { self.get_curr_y_maxima_vertex(horz) };

        //// remove 180 deg.spikes and also simplify
        //// consecutive horizontals when PreserveCollinear = true
        //if (!horzIsOpen && vertex_max != horz.vertex_top)
        //  TrimHorz(horz, PreserveCollinear);

        let mut horz_left = 0i64;
        let mut horz_right = 0i64;
        let mut is_left_to_right = self.reset_horz_direction(horz, vertex_max, &mut horz_left, &mut horz_right);

        if self.is_hot_edge(horz) {
            // `Point64(horz.curr_x, y, horz.bot.z)` under USINGZ, `Point64(horz.curr_x, y)`
            // otherwise.
            let p = point_xyz(self.active_pool_[horz].curr_x, y, point_z(&self.active_pool_[horz].bot));
            let op = self.add_out_pt(horz, &p);
            self.add_trial_horz_join(op);
        }

        loop {
            // loop through consec. horizontal edges
            let mut e = if is_left_to_right { self.active_pool_[horz].next_in_ael } else { self.active_pool_[horz].prev_in_ael };

            while e != NONE {
                if self.active_pool_[e].vertex_top == vertex_max {
                    if self.is_hot_edge(horz) && self.is_joined(e) {
                        let top = self.active_pool_[e].top;
                        self.split(e, &top);
                    }

                    //if (IsHotEdge(horz) != IsHotEdge(*e))
                    //    DoError(undefined_error_i);

                    if self.is_hot_edge(horz) {
                        while self.active_pool_[horz].vertex_top != vertex_max {
                            let top = self.active_pool_[horz].top;
                            self.add_out_pt(horz, &top);
                            self.update_edge_into_ael(horz);
                        }
                        let top = self.active_pool_[horz].top;
                        if is_left_to_right {
                            self.add_local_max_poly(horz, e, &top);
                        } else {
                            self.add_local_max_poly(e, horz, &top);
                        }
                    }
                    self.delete_from_ael(e);
                    self.delete_from_ael(horz);
                    return;
                }

                //if horzEdge is a maxima, keep going until we reach
                //its maxima pair, otherwise check for break conditions
                if vertex_max != self.active_pool_[horz].vertex_top || self.is_open_end(horz) {
                    //otherwise stop when 'ae' is beyond the end of the horizontal line
                    if (is_left_to_right && self.active_pool_[e].curr_x > horz_right)
                        || (!is_left_to_right && self.active_pool_[e].curr_x < horz_left)
                    {
                        break;
                    }

                    if self.active_pool_[e].curr_x == self.active_pool_[horz].top.x && !is_horizontal(&self.active_pool_[e]) {
                        pt = self.vertex_lists_[self.next_vertex(horz)].pt;
                        if is_left_to_right {
                            //with open paths we'll only break once past horz's end
                            if self.is_open(e) && !self.is_same_poly_type(e, horz) && !self.is_hot_edge(e) {
                                if top_x(&self.active_pool_[e], pt.y) > pt.x {
                                    break;
                                }
                            }
                            //otherwise we'll only break when horz's outslope is greater than e's
                            else if top_x(&self.active_pool_[e], pt.y) >= pt.x {
                                break;
                            }
                        } else if self.is_open(e) && !self.is_same_poly_type(e, horz) && !self.is_hot_edge(e) {
                            if top_x(&self.active_pool_[e], pt.y) < pt.x {
                                break;
                            }
                        } else if top_x(&self.active_pool_[e], pt.y) <= pt.x {
                            break;
                        }
                    }
                }

                pt = Point64::new(self.active_pool_[e].curr_x, self.active_pool_[horz].bot.y);
                if is_left_to_right {
                    self.intersect_edges(horz, e, &pt);
                    self.swap_positions_in_ael(horz, e);
                    self.check_join_left(e, &pt, false);
                    self.active_pool_[horz].curr_x = self.active_pool_[e].curr_x;
                    e = self.active_pool_[horz].next_in_ael;
                } else {
                    self.intersect_edges(e, horz, &pt);
                    self.swap_positions_in_ael(e, horz);
                    self.check_join_right(e, &pt, false);
                    self.active_pool_[horz].curr_x = self.active_pool_[e].curr_x;
                    e = self.active_pool_[horz].prev_in_ael;
                }

                if self.active_pool_[horz].outrec != NONE {
                    //nb: The outrec containing the op returned by IntersectEdges
                    //above may no longer be associated with horzEdge.
                    let last_op = self.get_last_op(horz);
                    self.add_trial_horz_join(last_op);
                }
            }

            //check if we've finished with (consecutive) horizontals ...
            if horz_is_open && self.is_open_end(horz) {
                // ie open at top
                if self.is_hot_edge(horz) {
                    let top = self.active_pool_[horz].top;
                    self.add_out_pt(horz, &top);
                    let outrec = self.active_pool_[horz].outrec;
                    if self.is_front(horz) {
                        self.outrec_list_[outrec].front_edge = NONE;
                    } else {
                        self.outrec_list_[outrec].back_edge = NONE;
                    }
                    self.active_pool_[horz].outrec = NONE;
                }
                self.delete_from_ael(horz);
                return;
            } else if self.vertex_lists_[self.next_vertex(horz)].pt.y != self.active_pool_[horz].top.y {
                break;
            }

            //still more horizontals in bound to process ...
            if self.is_hot_edge(horz) {
                let top = self.active_pool_[horz].top;
                self.add_out_pt(horz, &top);
            }
            self.update_edge_into_ael(horz);

            is_left_to_right = self.reset_horz_direction(horz, vertex_max, &mut horz_left, &mut horz_right);
        }

        if self.is_hot_edge(horz) {
            let top = self.active_pool_[horz].top;
            let op = self.add_out_pt(horz, &top);
            self.add_trial_horz_join(op);
        }

        self.update_edge_into_ael(horz); // end of an intermediate horiz.
    }

    /// clipper.engine.cpp:2705-2734
    /// C++: void ClipperBase::DoTopOfScanbeam(const int64_t y)
    fn do_top_of_scanbeam(&mut self, y: i64) {
        self.sel_ = NONE; // sel_ is reused to flag horizontals (see PushHorz below)
        let mut e = self.actives_;
        while e != NONE {
            //nb: 'e' will never be horizontal here
            if self.active_pool_[e].top.y == y {
                self.active_pool_[e].curr_x = self.active_pool_[e].top.x;
                if self.is_maxima(e) {
                    e = self.do_maxima(e); // TOP OF BOUND (MAXIMA)
                    continue;
                } else {
                    //INTERMEDIATE VERTEX ...
                    if self.is_hot_edge(e) {
                        let top = self.active_pool_[e].top;
                        self.add_out_pt(e, &top);
                    }
                    self.update_edge_into_ael(e);
                    if is_horizontal(&self.active_pool_[e]) {
                        self.push_horz(e); // horizontals are processed later
                    }
                }
            } else {
                // i.e. not the top of the edge
                self.active_pool_[e].curr_x = top_x(&self.active_pool_[e], y);
            }

            e = self.active_pool_[e].next_in_ael;
        }
    }

    /// clipper.engine.cpp:2737-2791
    /// C++: Active* ClipperBase::DoMaxima(Active& e)
    fn do_maxima(&mut self, e: usize) -> usize {
        let prev_e = self.active_pool_[e].prev_in_ael;
        let mut next_e = self.active_pool_[e].next_in_ael;
        if self.is_open_end(e) {
            if self.is_hot_edge(e) {
                let top = self.active_pool_[e].top;
                self.add_out_pt(e, &top);
            }
            if !is_horizontal(&self.active_pool_[e]) {
                if self.is_hot_edge(e) {
                    let outrec = self.active_pool_[e].outrec;
                    if self.is_front(e) {
                        self.outrec_list_[outrec].front_edge = NONE;
                    } else {
                        self.outrec_list_[outrec].back_edge = NONE;
                    }
                    self.active_pool_[e].outrec = NONE;
                }
                self.delete_from_ael(e);
            }
            return next_e;
        }

        let max_pair = self.get_maxima_pair(e);
        if max_pair == NONE {
            return next_e; // eMaxPair is horizontal
        }

        if self.is_joined(e) {
            let top = self.active_pool_[e].top;
            self.split(e, &top);
        }
        if self.is_joined(max_pair) {
            let top = self.active_pool_[max_pair].top;
            self.split(max_pair, &top);
        }

        //only non-horizontal maxima here.
        //process any edges between maxima pair ...
        while next_e != max_pair {
            let top = self.active_pool_[e].top;
            self.intersect_edges(e, next_e, &top);
            self.swap_positions_in_ael(e, next_e);
            next_e = self.active_pool_[e].next_in_ael;
        }

        if self.is_open(e) {
            if self.is_hot_edge(e) {
                let top = self.active_pool_[e].top;
                self.add_local_max_poly(e, max_pair, &top);
            }
            self.delete_from_ael(max_pair);
            self.delete_from_ael(e);
            return if prev_e != NONE { self.active_pool_[prev_e].next_in_ael } else { self.actives_ };
        }

        // e.next_in_ael== max_pair ...
        if self.is_hot_edge(e) {
            let top = self.active_pool_[e].top;
            self.add_local_max_poly(e, max_pair, &top);
        }

        self.delete_from_ael(e);
        self.delete_from_ael(max_pair);
        if prev_e != NONE {
            self.active_pool_[prev_e].next_in_ael
        } else {
            self.actives_
        }
    }

    /// clipper.engine.cpp:2793-2807
    /// C++: void ClipperBase::Split(Active& e, const Point64& pt)
    fn split(&mut self, e: usize, pt: &Point64) {
        self.stats.edge_splits += 1;
        if self.active_pool_[e].join_with == JoinWith::Right {
            let next = self.active_pool_[e].next_in_ael;
            self.active_pool_[e].join_with = JoinWith::NoJoin;
            self.active_pool_[next].join_with = JoinWith::NoJoin;
            self.add_local_min_poly(e, next, pt, true);
        } else {
            let prev = self.active_pool_[e].prev_in_ael;
            self.active_pool_[e].join_with = JoinWith::NoJoin;
            self.active_pool_[prev].join_with = JoinWith::NoJoin;
            self.add_local_min_poly(prev, e, pt, true);
        }
    }

    /// clipper.engine.cpp:2809-2835
    /// C++: void ClipperBase::CheckJoinLeft(Active& e, const Point64& pt, bool check_curr_x)
    fn check_join_left(&mut self, e: usize, pt: &Point64, check_curr_x: bool) {
        let prev = self.active_pool_[e].prev_in_ael;
        if prev == NONE
            || !self.is_hot_edge(e)
            || !self.is_hot_edge(prev)
            || is_horizontal(&self.active_pool_[e])
            || is_horizontal(&self.active_pool_[prev])
            || self.is_open(e)
            || self.is_open(prev)
        {
            return;
        }
        let (ae, ap) = (self.active_pool_[e], self.active_pool_[prev]);
        if (pt.y < ae.top.y + 2 || pt.y < ap.top.y + 2) && ((ae.bot.y > pt.y) || (ap.bot.y > pt.y)) {
            return; // avoid trivial joins
        }

        if check_curr_x {
            if perpendic_dist_from_line_sqrd(pt, &ap.bot, &ap.top) > 0.25 {
                return;
            }
        } else if ae.curr_x != ap.curr_x {
            return;
        }
        if !is_collinear(&ae.top, pt, &ap.top) {
            return;
        }

        self.stats.edge_joins += 1;
        let (e_idx, prev_idx) = (self.outrec_list_[ae.outrec].idx, self.outrec_list_[ap.outrec].idx);
        if e_idx == prev_idx {
            self.add_local_max_poly(prev, e, pt);
        } else if e_idx < prev_idx {
            self.join_outrec_paths(e, prev);
        } else {
            self.join_outrec_paths(prev, e);
        }
        self.active_pool_[prev].join_with = JoinWith::Right;
        self.active_pool_[e].join_with = JoinWith::Left;
    }

    /// clipper.engine.cpp:2837-2864
    /// C++: void ClipperBase::CheckJoinRight(Active& e, const Point64& pt, bool check_curr_x)
    fn check_join_right(&mut self, e: usize, pt: &Point64, check_curr_x: bool) {
        let next = self.active_pool_[e].next_in_ael;
        if next == NONE
            || !self.is_hot_edge(e)
            || !self.is_hot_edge(next)
            || is_horizontal(&self.active_pool_[e])
            || is_horizontal(&self.active_pool_[next])
            || self.is_open(e)
            || self.is_open(next)
        {
            return;
        }
        let (ae, an) = (self.active_pool_[e], self.active_pool_[next]);
        if (pt.y < ae.top.y + 2 || pt.y < an.top.y + 2) && ((ae.bot.y > pt.y) || (an.bot.y > pt.y)) {
            return; // avoid trivial joins
        }

        if check_curr_x {
            if perpendic_dist_from_line_sqrd(pt, &an.bot, &an.top) > 0.35 {
                return;
            }
        } else if ae.curr_x != an.curr_x {
            return;
        }
        if !is_collinear(&ae.top, pt, &an.top) {
            return;
        }

        self.stats.edge_joins += 1;
        let (e_idx, next_idx) = (self.outrec_list_[ae.outrec].idx, self.outrec_list_[an.outrec].idx);
        if e_idx == next_idx {
            self.add_local_max_poly(e, next, pt);
        } else if e_idx < next_idx {
            self.join_outrec_paths(e, next);
        } else {
            self.join_outrec_paths(next, e);
        }

        self.active_pool_[e].join_with = JoinWith::Right;
        self.active_pool_[next].join_with = JoinWith::Left;
    }

    /// clipper.engine.cpp:2866-2886
    /// C++: inline bool GetHorzExtendedHorzSeg(OutPt*& op, OutPt*& op2)
    #[allow(dead_code)]
    fn get_horz_extended_horz_seg(&self, op: &mut usize, op2: &mut usize) -> bool {
        let p = &self.outpt_pool_;
        let outrec = self.get_real_out_rec(p[*op].outrec);
        *op2 = *op;
        if self.outrec_list_[outrec].front_edge != NONE {
            let pts = self.outrec_list_[outrec].pts;
            while p[*op].prev != pts && p[p[*op].prev].pt.y == p[*op].pt.y {
                *op = p[*op].prev;
            }
            while *op2 != pts && p[p[*op2].next].pt.y == p[*op2].pt.y {
                *op2 = p[*op2].next;
            }
            *op2 != *op
        } else {
            while p[*op].prev != *op2 && p[p[*op].prev].pt.y == p[*op].pt.y {
                *op = p[*op].prev;
            }
            while p[*op2].next != *op && p[p[*op2].next].pt.y == p[*op2].pt.y {
                *op2 = p[*op2].next;
            }
            *op2 != *op && p[*op2].next != *op
        }
    }

    /// clipper.engine.cpp:2888-2924
    /// C++: bool BuildPath64(OutPt* op, bool reverse, bool isOpen, Path64& path)
    fn build_path64(&self, mut op: usize, reverse: bool, is_open: bool, path: &mut Path64) -> bool {
        let p = &self.outpt_pool_;
        if op == NONE || p[op].next == op || (!is_open && p[op].next == p[op].prev) {
            return false;
        }

        path.clear();
        let mut last_pt;
        let mut op2;
        if reverse {
            last_pt = p[op].pt;
            op2 = p[op].prev;
        } else {
            op = p[op].next;
            last_pt = p[op].pt;
            op2 = p[op].next;
        }
        path.push(last_pt);

        while op2 != op {
            if p[op2].pt != last_pt {
                last_pt = p[op2].pt;
                path.push(last_pt);
            }
            if reverse {
                op2 = p[op2].prev;
            } else {
                op2 = p[op2].next;
            }
        }

        !(!is_open && path.len() == 3 && self.is_very_small_triangle(op2))
    }

    /// clipper.engine.cpp:2926-2936
    /// C++: bool ClipperBase::CheckBounds(OutRec* outrec)
    fn check_bounds(&mut self, outrec: usize) -> bool {
        if self.outrec_list_[outrec].pts == NONE {
            return false;
        }
        if !self.outrec_list_[outrec].bounds.is_empty() {
            return true;
        }
        self.clean_collinear(outrec);
        if self.outrec_list_[outrec].pts == NONE {
            return false;
        }
        let mut path = std::mem::take(&mut self.outrec_list_[outrec].path);
        let built = self.build_path64(self.outrec_list_[outrec].pts, self.reverse_solution_, false, &mut path);
        self.outrec_list_[outrec].path = path;
        if !built {
            return false;
        }
        self.outrec_list_[outrec].bounds = get_bounds(&self.outrec_list_[outrec].path);
        true
    }

    /// clipper.engine.cpp:2938-2958
    /// C++: bool ClipperBase::CheckSplitOwner(OutRec* outrec, OutRecList* splits)
    /// `splits` is the member of the `OutRec` `splits_of`.
    ///
    /// FIDELITY-NOTE: the C++ walks `*splits` with a range-for while the body can reach
    /// `DoSplitOp` (through `CheckBounds`, `CleanCollinear`, `FixSelfIntersects`), which
    /// appends to the `splits` of an `OutRec`. If that is the vector being walked and it
    /// reallocates, the C++ reads freed memory (undefined behaviour); if it does not
    /// reallocate, the loop ends at the old size. Here the loop walks the elements the
    /// vector had when the loop started, which is what the C++ does in the second case
    /// and, as long as the freed block keeps its bytes, in the first.
    /// `stats.splits_changed_in_loop` counts the loops in which it happened; the parity
    /// tests reach it (rarely: a few times in a million cases, in `nearly_parallel` and
    /// `coincident_and_collinear`) and the results are equal there.
    fn check_split_owner(&mut self, outrec: usize, splits_of: usize) -> bool {
        let splits: OutRecList = self.outrec_list_[splits_of].splits.clone().unwrap();
        for &split in &splits {
            let split = self.get_real_out_rec(split);
            if split == NONE || split == outrec || self.outrec_list_[split].recursive_split == outrec {
                continue;
            }
            self.outrec_list_[split].recursive_split = outrec; // prevent infinite loops

            if self.outrec_list_[split].splits.is_some() && self.check_split_owner(outrec, split) {
                self.note_splits_changed(splits_of, &splits);
                return true;
            } else if self.check_bounds(split)
                && self.is_valid_owner(outrec, split)
                && self.outrec_list_[split].bounds.contains(&self.outrec_list_[outrec].bounds)
                && self.path1_inside_path2(self.outrec_list_[outrec].pts, self.outrec_list_[split].pts)
            {
                self.outrec_list_[outrec].owner = split; //found in split
                self.stats.split_owners_found += 1;
                self.note_splits_changed(splits_of, &splits);
                return true;
            }
        }
        self.note_splits_changed(splits_of, &splits);
        false
    }

    /// Counts a `CheckSplitOwner` loop whose vector is not what it was at the start.
    fn note_splits_changed(&mut self, splits_of: usize, before: &OutRecList) {
        if self.outrec_list_[splits_of].splits.as_ref() != Some(before) {
            self.stats.splits_changed_in_loop += 1;
        }
    }

    /// clipper.engine.cpp:2960-2984
    /// C++: void ClipperBase::RecursiveCheckOwners(OutRec* outrec, PolyPath* polypath)
    fn recursive_check_owners(&mut self, outrec: usize, polypath: usize, tree: &mut PolyPathArena) {
        // pre-condition: outrec will have valid bounds
        // post-condition: if a valid path, outrec will have a polypath

        if self.outrec_list_[outrec].polypath != NONE || self.outrec_list_[outrec].bounds.is_empty() {
            return;
        }

        while self.outrec_list_[outrec].owner != NONE {
            let owner = self.outrec_list_[outrec].owner;
            if self.outrec_list_[owner].splits.is_some() && self.check_split_owner(outrec, owner) {
                break;
            }
            // (`outrec->owner` is read again: CheckSplitOwner does not change it when it
            // returns false.)
            let owner = self.outrec_list_[outrec].owner;
            if self.outrec_list_[owner].pts != NONE
                && self.check_bounds(owner)
                && self.outrec_list_[owner].bounds.contains(&self.outrec_list_[outrec].bounds)
                && self.path1_inside_path2(self.outrec_list_[outrec].pts, self.outrec_list_[owner].pts)
            {
                break;
            }
            self.outrec_list_[outrec].owner = self.outrec_list_[owner].owner;
        }

        let owner = self.outrec_list_[outrec].owner;
        if owner != NONE {
            if self.outrec_list_[owner].polypath == NONE {
                self.recursive_check_owners(owner, polypath, tree);
            }
            let node = tree.add_child(self.outrec_list_[owner].polypath, &self.outrec_list_[outrec].path);
            self.outrec_list_[outrec].polypath = node;
        } else {
            let node = tree.add_child(polypath, &self.outrec_list_[outrec].path);
            self.outrec_list_[outrec].polypath = node;
        }
    }
}

/// clipper.engine.h:470-529
/// C++: class Clipper64 : public ClipperBase
/// The base class is the member `base`, and `Deref` gives its public methods.
#[derive(Debug, Default)]
pub struct Clipper64 {
    base: ClipperBase,
}

impl std::ops::Deref for Clipper64 {
    type Target = ClipperBase;
    fn deref(&self) -> &ClipperBase {
        &self.base
    }
}

impl std::ops::DerefMut for Clipper64 {
    fn deref_mut(&mut self) -> &mut ClipperBase {
        &mut self.base
    }
}

impl Clipper64 {
    pub fn new() -> Self {
        Clipper64 { base: ClipperBase::new() }
    }

    /// clipper.engine.h:476-478
    /// C++: void SetZCallback(ZCallback64 cb) { zCallback_ = cb; }
    /// (under `USINGZ`; in the build without it `ZCallback64` holds nothing.)
    pub fn set_z_callback(&mut self, cb: ZCallback64) {
        self.base.z_callback_ = cb;
    }

    /// clipper.engine.h:480-483
    /// C++: void AddSubject(const Paths64& subjects)
    pub fn add_subject(&mut self, subjects: &Paths64) {
        self.base.add_paths(subjects, PathType::Subject, false);
    }

    /// clipper.engine.h:484-487
    /// C++: void AddOpenSubject(const Paths64& open_subjects)
    pub fn add_open_subject(&mut self, open_subjects: &Paths64) {
        self.base.add_paths(open_subjects, PathType::Subject, true);
    }

    /// clipper.engine.h:488-491
    /// C++: void AddClip(const Paths64& clips)
    pub fn add_clip(&mut self, clips: &Paths64) {
        self.base.add_paths(clips, PathType::Clip, false);
    }

    /// clipper.engine.h:493-498
    /// C++: bool Execute(ClipType clip_type, FillRule fill_rule, Paths64& closed_paths)
    pub fn execute(&mut self, clip_type: ClipType, fill_rule: FillRule, closed_paths: &mut Paths64) -> bool {
        let mut dummy = Paths64::new();
        self.execute_open(clip_type, fill_rule, closed_paths, &mut dummy)
    }

    /// clipper.engine.h:500-509
    /// C++: bool Execute(ClipType clip_type, FillRule fill_rule, Paths64& closed_paths, Paths64& open_paths)
    pub fn execute_open(&mut self, clip_type: ClipType, fill_rule: FillRule, closed_paths: &mut Paths64, open_paths: &mut Paths64) -> bool {
        closed_paths.clear();
        open_paths.clear();
        if self.base.execute_internal(clip_type, fill_rule, false) {
            self.build_paths64(closed_paths, Some(open_paths));
        }
        self.base.clean_up();
        self.base.succeeded_
    }

    /// clipper.engine.h:511-515
    /// C++: bool Execute(ClipType clip_type, FillRule fill_rule, PolyTree64& polytree)
    pub fn execute_tree(&mut self, clip_type: ClipType, fill_rule: FillRule, polytree: &mut PolyTree64) -> bool {
        let mut dummy = Paths64::new();
        self.execute_tree_open(clip_type, fill_rule, polytree, &mut dummy)
    }

    /// clipper.engine.h:517-528
    /// C++: bool Execute(ClipType clip_type, FillRule fill_rule, PolyTree64& polytree, Paths64& open_paths)
    pub fn execute_tree_open(&mut self, clip_type: ClipType, fill_rule: FillRule, polytree: &mut PolyTree64, open_paths: &mut Paths64) -> bool {
        if self.base.execute_internal(clip_type, fill_rule, true) {
            open_paths.clear();
            polytree.clear();
            self.build_tree64(polytree, open_paths);
        }
        self.base.clean_up();
        self.base.succeeded_
    }

    /// clipper.engine.cpp:2986-3019
    /// C++: void Clipper64::BuildPaths64(Paths64& solutionClosed, Paths64* solutionOpen)
    fn build_paths64(&mut self, solution_closed: &mut Paths64, mut solution_open: Option<&mut Paths64>) {
        let b = &mut self.base;
        solution_closed.clear();
        solution_closed.reserve(b.outrec_list_.len());
        if let Some(open) = solution_open.as_deref_mut() {
            open.clear();
            open.reserve(b.outrec_list_.len());
        }

        // nb: outrec_list_.size() may change in the following
        // while loop because polygons may be split during
        // calls to CleanCollinear which calls FixSelfIntersects
        let mut i = 0;
        while i < b.outrec_list_.len() {
            let outrec = i;
            i += 1;
            if b.outrec_list_[outrec].pts == NONE {
                continue;
            }

            let mut path = Path64::new();
            if solution_open.is_some() && b.outrec_list_[outrec].is_open {
                if b.build_path64(b.outrec_list_[outrec].pts, b.reverse_solution_, true, &mut path) {
                    solution_open.as_deref_mut().unwrap().push(path);
                }
            } else {
                // nb: CleanCollinear can add to outrec_list_
                b.clean_collinear(outrec);
                //closed paths should always return a Positive orientation
                if b.build_path64(b.outrec_list_[outrec].pts, b.reverse_solution_, false, &mut path) {
                    solution_closed.push(path);
                }
            }
        }
    }

    /// clipper.engine.cpp:3021-3046
    /// C++: void Clipper64::BuildTree64(PolyPath64& polytree, Paths64& open_paths)
    fn build_tree64(&mut self, polytree: &mut PolyPath64, open_paths: &mut Paths64) {
        let b = &mut self.base;
        polytree.clear();
        open_paths.clear();
        if b.has_open_paths_ {
            open_paths.reserve(b.outrec_list_.len());
        }
        let mut tree = PolyPathArena::new();

        // outrec_list_.size() is not static here because
        // CheckBounds below can indirectly add additional
        // OutRec (via FixOutRecPts & CleanCollinear)
        let mut i = 0;
        while i < b.outrec_list_.len() {
            let outrec = i;
            i += 1;
            if b.outrec_list_[outrec].pts == NONE {
                continue;
            }
            if b.outrec_list_[outrec].is_open {
                let mut path = Path64::new();
                if b.build_path64(b.outrec_list_[outrec].pts, b.reverse_solution_, true, &mut path) {
                    open_paths.push(path);
                }
                continue;
            }

            if b.check_bounds(outrec) {
                b.recursive_check_owners(outrec, 0, &mut tree);
            }
        }
        tree.fill(0, polytree);
    }
}
