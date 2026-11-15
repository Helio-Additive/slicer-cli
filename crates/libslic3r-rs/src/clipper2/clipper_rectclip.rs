//! `include/clipper2/clipper.rectclip.h` and `src/clipper.rectclip.cpp` of Clipper2 as the
//! engine pins it (version 1.5.2, `src/clipper2/Clipper2Lib`): FAST rectangular clipping,
//! `RectClip64` and `RectClipLines64`. Compiled twice, see `mod.rs`.
//!
//! The C++ links its `OutPt2` with pointers into `op_container_` (a `std::deque`, whose
//! elements never move). Here `op_container_` is a `Vec` and a pointer is an index into it,
//! `NONE` for `nullptr`. `OutPt2::edge` points at one of the eight vectors `edges_`: it is
//! the number of that vector.
//!
//! Line numbers are those of the pinned files. Floating point follows the reference build,
//! see `clipper_core.rs`: the only arithmetic on doubles here is in `CrossProduct` and
//! `GetSegmentIntersectPt` of clipper.core.h.
//!
//! `tests/clipper2_rectclip_native_parity.rs` and its `_z` twin hold `RectClip64::Execute`
//! and `RectClipLines64::Execute` equal to the pinned C++ in both builds.

use super::clipper_core::*;

/// `nullptr`
const NONE: usize = usize::MAX;

/// clipper.rectclip.h:23-24
/// C++: enum class Location { Left, Top, Right, Bottom, Inside };
/// Location: the order is important here, see StartLocsIsClockwise()
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Location {
    Left = 0,
    Top = 1,
    Right = 2,
    Bottom = 3,
    Inside = 4,
}

impl Location {
    /// `static_cast<Location>(int)` for 0..=4.
    fn from_int(v: i32) -> Location {
        match v {
            0 => Location::Left,
            1 => Location::Top,
            2 => Location::Right,
            3 => Location::Bottom,
            4 => Location::Inside,
            _ => unreachable!("a Location is 0..=4"),
        }
    }
}

/// clipper.rectclip.h:27 `typedef std::vector<OutPt2*> OutPt2List;`
pub type OutPt2List = Vec<usize>;

/// clipper.rectclip.h:29-36
/// C++: class OutPt2
#[derive(Debug, Clone, Copy)]
pub struct OutPt2 {
    pub pt: Point64,
    pub owner_idx: usize,
    /// `OutPt2List* edge`: the number of the vector of `edges_`, `NONE` for `nullptr`.
    pub edge: usize,
    pub next: usize,
    pub prev: usize,
}

impl Default for OutPt2 {
    fn default() -> Self {
        OutPt2 { pt: Point64::default(), owner_idx: 0, edge: NONE, next: NONE, prev: NONE }
    }
}

//------------------------------------------------------------------------------
// Miscellaneous methods
//------------------------------------------------------------------------------

/// clipper.rectclip.cpp:23-39
/// C++: inline bool Path1ContainsPath2(const Path64& path1, const Path64& path2)
fn path1_contains_path2(path1: &Path64, path2: &Path64) -> bool {
    let mut io_count: i32 = 0;
    // precondition: no (significant) overlap
    for pt in path2 {
        let pip = point_in_polygon(pt, path1);
        match pip {
            PointInPolygonResult::IsOutside => io_count += 1,
            PointInPolygonResult::IsInside => io_count -= 1,
            _ => continue,
        }
        if io_count.abs() > 1 {
            break;
        }
    }
    io_count <= 0
}

/// clipper.rectclip.cpp:41-70
/// C++: inline bool GetLocation(const Rect64& rec, const Point64& pt, Location& loc)
fn get_location(rec: &Rect64, pt: &Point64, loc: &mut Location) -> bool {
    if pt.x == rec.left && pt.y >= rec.top && pt.y <= rec.bottom {
        *loc = Location::Left;
        return false;
    } else if pt.x == rec.right && pt.y >= rec.top && pt.y <= rec.bottom {
        *loc = Location::Right;
        return false;
    } else if pt.y == rec.top && pt.x >= rec.left && pt.x <= rec.right {
        *loc = Location::Top;
        return false;
    } else if pt.y == rec.bottom && pt.x >= rec.left && pt.x <= rec.right {
        *loc = Location::Bottom;
        return false;
    } else if pt.x < rec.left {
        *loc = Location::Left;
    } else if pt.x > rec.right {
        *loc = Location::Right;
    } else if pt.y < rec.top {
        *loc = Location::Top;
    } else if pt.y > rec.bottom {
        *loc = Location::Bottom;
    } else {
        *loc = Location::Inside;
    }
    true
}

/// clipper.rectclip.cpp:72-75
/// C++: inline bool IsHorizontal(const Point64& pt1, const Point64& pt2)
#[inline]
fn is_horizontal(pt1: &Point64, pt2: &Point64) -> bool {
    pt1.y == pt2.y
}

/// clipper.rectclip.cpp:77-120
/// C++: bool GetSegmentIntersection(const Point64& p1, const Point64& p2, const Point64& p3, const Point64& p4, Point64& ip)
/// With `USINGZ`, `ip = p1` (and the like) copies the z of that point.
pub fn get_segment_intersection(p1: &Point64, p2: &Point64, p3: &Point64, p4: &Point64, ip: &mut Point64) -> bool {
    let res1 = cross_product(p1, p3, p4);
    let res2 = cross_product(p2, p3, p4);
    if res1 == 0.0 {
        *ip = *p1;
        if res2 == 0.0 {
            return false; // segments are collinear
        } else if *p1 == *p3 || *p1 == *p4 {
            return true;
        }
        //else if (p2 == p3 || p2 == p4) { ip = p2; return true; }
        else if is_horizontal(p3, p4) {
            return (p1.x > p3.x) == (p1.x < p4.x);
        } else {
            return (p1.y > p3.y) == (p1.y < p4.y);
        }
    } else if res2 == 0.0 {
        *ip = *p2;
        if *p2 == *p3 || *p2 == *p4 {
            return true;
        } else if is_horizontal(p3, p4) {
            return (p2.x > p3.x) == (p2.x < p4.x);
        } else {
            return (p2.y > p3.y) == (p2.y < p4.y);
        }
    }
    if (res1 > 0.0) == (res2 > 0.0) {
        return false;
    }

    let res3 = cross_product(p3, p1, p2);
    let res4 = cross_product(p4, p1, p2);
    if res3 == 0.0 {
        *ip = *p3;
        if *p3 == *p1 || *p3 == *p2 {
            return true;
        } else if is_horizontal(p1, p2) {
            return (p3.x > p1.x) == (p3.x < p2.x);
        } else {
            return (p3.y > p1.y) == (p3.y < p2.y);
        }
    } else if res4 == 0.0 {
        *ip = *p4;
        if *p4 == *p1 || *p4 == *p2 {
            return true;
        } else if is_horizontal(p1, p2) {
            return (p4.x > p1.x) == (p4.x < p2.x);
        } else {
            return (p4.y > p1.y) == (p4.y < p2.y);
        }
    }
    if (res3 > 0.0) == (res4 > 0.0) {
        return false;
    }

    // segments must intersect to get here
    get_segment_intersect_pt(p1, p2, p3, p4, ip)
}

/// clipper.rectclip.cpp:122-208
/// C++: inline bool GetIntersection(const Path64& rectPath, const Point64& p, const Point64& p2, Location& loc, Point64& ip)
fn get_intersection(rect_path: &Path64, p: &Point64, p2: &Point64, loc: &mut Location, ip: &mut Point64) -> bool {
    // gets the intersection closest to 'p'
    // when Result = false, loc will remain unchanged
    match *loc {
        Location::Left => {
            if get_segment_intersection(p, p2, &rect_path[0], &rect_path[3], ip) {
                true
            } else if (p.y < rect_path[0].y) && get_segment_intersection(p, p2, &rect_path[0], &rect_path[1], ip) {
                *loc = Location::Top;
                true
            } else if get_segment_intersection(p, p2, &rect_path[2], &rect_path[3], ip) {
                *loc = Location::Bottom;
                true
            } else {
                false
            }
        }

        Location::Top => {
            if get_segment_intersection(p, p2, &rect_path[0], &rect_path[1], ip) {
                true
            } else if (p.x < rect_path[0].x) && get_segment_intersection(p, p2, &rect_path[0], &rect_path[3], ip) {
                *loc = Location::Left;
                true
            } else if get_segment_intersection(p, p2, &rect_path[1], &rect_path[2], ip) {
                *loc = Location::Right;
                true
            } else {
                false
            }
        }

        Location::Right => {
            if get_segment_intersection(p, p2, &rect_path[1], &rect_path[2], ip) {
                true
            } else if (p.y < rect_path[1].y) && get_segment_intersection(p, p2, &rect_path[0], &rect_path[1], ip) {
                *loc = Location::Top;
                true
            } else if get_segment_intersection(p, p2, &rect_path[2], &rect_path[3], ip) {
                *loc = Location::Bottom;
                true
            } else {
                false
            }
        }

        Location::Bottom => {
            if get_segment_intersection(p, p2, &rect_path[2], &rect_path[3], ip) {
                true
            } else if (p.x < rect_path[3].x) && get_segment_intersection(p, p2, &rect_path[0], &rect_path[3], ip) {
                *loc = Location::Left;
                true
            } else if get_segment_intersection(p, p2, &rect_path[1], &rect_path[2], ip) {
                *loc = Location::Right;
                true
            } else {
                false
            }
        }

        // loc == rInside
        Location::Inside => {
            if get_segment_intersection(p, p2, &rect_path[0], &rect_path[3], ip) {
                *loc = Location::Left;
                true
            } else if get_segment_intersection(p, p2, &rect_path[0], &rect_path[1], ip) {
                *loc = Location::Top;
                true
            } else if get_segment_intersection(p, p2, &rect_path[1], &rect_path[2], ip) {
                *loc = Location::Right;
                true
            } else if get_segment_intersection(p, p2, &rect_path[2], &rect_path[3], ip) {
                *loc = Location::Bottom;
                true
            } else {
                false
            }
        }
    }
}

/// clipper.rectclip.cpp:210-214
/// C++: inline Location GetAdjacentLocation(Location loc, bool isClockwise)
#[inline]
fn get_adjacent_location(loc: Location, is_clockwise: bool) -> Location {
    let delta = if is_clockwise { 1 } else { 3 };
    Location::from_int((loc as i32 + delta) % 4)
}

/// clipper.rectclip.cpp:216-219
/// C++: inline bool HeadingClockwise(Location prev, Location curr)
#[inline]
fn heading_clockwise(prev: Location, curr: Location) -> bool {
    (prev as i32 + 1) % 4 == curr as i32
}

/// clipper.rectclip.cpp:221-224
/// C++: inline bool AreOpposites(Location prev, Location curr)
#[inline]
fn are_opposites(prev: Location, curr: Location) -> bool {
    (prev as i32 - curr as i32).abs() == 2
}

/// clipper.rectclip.cpp:226-233
/// C++: inline bool IsClockwise(Location prev, Location curr, const Point64& prev_pt, const Point64& curr_pt, const Point64& rect_mp)
#[inline]
fn is_clockwise(prev: Location, curr: Location, prev_pt: &Point64, curr_pt: &Point64, rect_mp: &Point64) -> bool {
    if are_opposites(prev, curr) {
        cross_product(prev_pt, rect_mp, curr_pt) < 0.0
    } else {
        heading_clockwise(prev, curr)
    }
}

/// clipper.rectclip.cpp:235-241
/// C++: inline OutPt2* UnlinkOp(OutPt2* op)
#[inline]
fn unlink_op(ops: &mut [OutPt2], op: usize) -> usize {
    if ops[op].next == op {
        return NONE;
    }
    let (prev, next) = (ops[op].prev, ops[op].next);
    ops[prev].next = next;
    ops[next].prev = prev;
    next
}

/// clipper.rectclip.cpp:243-249
/// C++: inline OutPt2* UnlinkOpBack(OutPt2* op)
#[inline]
fn unlink_op_back(ops: &mut [OutPt2], op: usize) -> usize {
    if ops[op].next == op {
        return NONE;
    }
    let (prev, next) = (ops[op].prev, ops[op].next);
    ops[prev].next = next;
    ops[next].prev = prev;
    prev
}

/// clipper.rectclip.cpp:251-259
/// C++: inline uint32_t GetEdgesForPt(const Point64& pt, const Rect64& rec)
#[inline]
fn get_edges_for_pt(pt: &Point64, rec: &Rect64) -> u32 {
    let mut result = 0u32;
    if pt.x == rec.left {
        result = 1;
    } else if pt.x == rec.right {
        result = 4;
    }
    if pt.y == rec.top {
        result += 2;
    } else if pt.y == rec.bottom {
        result += 8;
    }
    result
}

/// clipper.rectclip.cpp:261-270
/// C++: inline bool IsHeadingClockwise(const Point64& pt1, const Point64& pt2, int edgeIdx)
#[inline]
fn is_heading_clockwise(pt1: &Point64, pt2: &Point64, edge_idx: i32) -> bool {
    match edge_idx {
        0 => pt2.y < pt1.y,
        1 => pt2.x > pt1.x,
        2 => pt2.y > pt1.y,
        _ => pt2.x < pt1.x,
    }
}

/// clipper.rectclip.cpp:272-276
/// C++: inline bool HasHorzOverlap(const Point64& left1, const Point64& right1, const Point64& left2, const Point64& right2)
#[inline]
fn has_horz_overlap(left1: &Point64, right1: &Point64, left2: &Point64, right2: &Point64) -> bool {
    (left1.x < right2.x) && (right1.x > left2.x)
}

/// clipper.rectclip.cpp:278-282
/// C++: inline bool HasVertOverlap(const Point64& top1, const Point64& bottom1, const Point64& top2, const Point64& bottom2)
#[inline]
fn has_vert_overlap(top1: &Point64, bottom1: &Point64, top2: &Point64, bottom2: &Point64) -> bool {
    (top1.y < bottom2.y) && (bottom1.y > top2.y)
}

/// clipper.rectclip.cpp:284-289
/// C++: inline void AddToEdge(OutPt2List& edge, OutPt2* op)
/// `edge` is `edges[edge]`.
#[inline]
fn add_to_edge(edges: &mut [OutPt2List; 8], ops: &mut [OutPt2], edge: usize, op: usize) {
    if ops[op].edge != NONE {
        return;
    }
    ops[op].edge = edge;
    edges[edge].push(op);
}

/// clipper.rectclip.cpp:291-304
/// C++: inline void UncoupleEdge(OutPt2* op)
#[inline]
fn uncouple_edge(edges: &mut [OutPt2List; 8], ops: &mut [OutPt2], op: usize) {
    if ops[op].edge == NONE {
        return;
    }
    let edge = ops[op].edge;
    for i in 0..edges[edge].len() {
        let op2 = edges[edge][i];
        if op2 == op {
            edges[edge][i] = NONE;
            break;
        }
    }
    ops[op].edge = NONE;
}

/// clipper.rectclip.cpp:306-315
/// C++: inline void SetNewOwner(OutPt2* op, size_t new_idx)
#[inline]
fn set_new_owner(ops: &mut [OutPt2], op: usize, new_idx: usize) {
    ops[op].owner_idx = new_idx;
    let mut op2 = ops[op].next;
    while op2 != op {
        ops[op2].owner_idx = new_idx;
        op2 = ops[op2].next;
    }
}

/// clipper.rectclip.cpp:430-445
/// C++: bool StartLocsAreClockwise(const std::vector<Location>& startlocs)
pub fn start_locs_are_clockwise(startlocs: &[Location]) -> bool {
    let mut result = 0;
    for i in 1..startlocs.len() {
        let d = startlocs[i] as i32 - startlocs[i - 1] as i32;
        match d {
            -1 => result -= 1,
            1 => result += 1,
            -3 => result += 1,
            3 => result -= 1,
            _ => {}
        }
    }
    result > 0
}

//----------------------------------------------------------------------------
// RectClip64
//----------------------------------------------------------------------------

/// How often a `RectClip64` took the arms the parity tests want to know they reached. A
/// diagnostic: nothing reads it but the tests, and it changes nothing.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RectClipStats {
    /// `ExecuteInternal`: all of the path is on the rectangle's edges. (`RectClip64::Execute`
    /// never gets there: such a path has its bounds in the rectangle and is taken whole
    /// before `ExecuteInternal`. `RectClipLines64::Execute` does.)
    pub all_inside: u64,
    /// `ExecuteInternal`: a segment enters the rectangle
    pub entering: u64,
    /// `ExecuteInternal`: a segment leaves the rectangle
    pub exiting: u64,
    /// `ExecuteInternal`: a segment passes right through the rectangle
    pub passing_through: u64,
    /// `ExecuteInternal`: the second intersection of a segment passing through is the first
    pub passing_through_one_point: u64,
    /// `ExecuteInternal`: the path never crosses and contains the rectangle
    pub contains_rect: u64,
    /// `AddCorner`, both overloads
    pub corners: u64,
    /// `CheckEdges` and `GetPath`: a collinear vertex unlinked
    pub collinear_unlinked: u64,
    /// `TidyEdges`: a path split in two
    pub splits: u64,
    /// `TidyEdges`: two paths joined
    pub rejoins: u64,
}

/// clipper.rectclip.h:42-67
/// C++: class RectClip64
#[derive(Debug, Clone)]
pub struct RectClip64 {
    rect_: Rect64,
    rect_as_path_: Path64,
    rect_mp_: Point64,
    path_bounds_: Rect64,
    /// `std::deque<OutPt2> op_container_`
    op_container_: Vec<OutPt2>,
    /// each path can be broken into multiples
    results_: OutPt2List,
    /// clockwise and counter-clockwise
    edges_: [OutPt2List; 8],
    start_locs_: Vec<Location>,
    /// See `RectClipStats`.
    pub stats: RectClipStats,
}

impl RectClip64 {
    /// clipper.rectclip.h:62-65
    /// C++: explicit RectClip64(const Rect64& rect) : rect_(rect), rect_as_path_(rect.AsPath()), rect_mp_(rect.MidPoint()) {}
    pub fn new(rect: &Rect64) -> Self {
        RectClip64 {
            rect_: *rect,
            rect_as_path_: rect.as_path(),
            rect_mp_: rect.mid_point(),
            // `Rect64 path_bounds_;` is `Rect(bool is_valid = true)`.
            path_bounds_: Rect64::default(),
            op_container_: Vec::new(),
            results_: Vec::new(),
            edges_: Default::default(),
            start_locs_: Vec::new(),
            stats: RectClipStats::default(),
        }
    }

    /// clipper.rectclip.cpp:321-351
    /// C++: OutPt2* RectClip64::Add(Point64 pt, bool start_new)
    fn add(&mut self, pt: Point64, start_new: bool) -> usize {
        // this method is only called by InternalExecute.
        // Later splitting & rejoining won't create additional op's,
        // though they will change the (non-storage) results_ count.
        let mut curr_idx = self.results_.len();
        let result;
        if curr_idx == 0 || start_new {
            self.op_container_.push(OutPt2::default());
            result = self.op_container_.len() - 1;
            self.op_container_[result].pt = pt;
            self.op_container_[result].next = result;
            self.op_container_[result].prev = result;
            self.results_.push(result);
        } else {
            curr_idx -= 1;
            let prev_op = self.results_[curr_idx];
            if self.op_container_[prev_op].pt == pt {
                return prev_op;
            }
            self.op_container_.push(OutPt2::default());
            result = self.op_container_.len() - 1;
            self.op_container_[result].owner_idx = curr_idx;
            self.op_container_[result].pt = pt;
            let prev_next = self.op_container_[prev_op].next;
            self.op_container_[result].next = prev_next;
            self.op_container_[prev_next].prev = result;
            self.op_container_[prev_op].next = result;
            self.op_container_[result].prev = prev_op;
            self.results_[curr_idx] = result;
        }
        result
    }

    /// clipper.rectclip.cpp:353-359
    /// C++: void RectClip64::AddCorner(Location prev, Location curr)
    fn add_corner(&mut self, prev: Location, curr: Location) {
        self.stats.corners += 1;
        if heading_clockwise(prev, curr) {
            self.add(self.rect_as_path_[prev as usize], false);
        } else {
            self.add(self.rect_as_path_[curr as usize], false);
        }
    }

    /// clipper.rectclip.cpp:361-373
    /// C++: void RectClip64::AddCorner(Location& loc, bool isClockwise)
    fn add_corner_loc(&mut self, loc: &mut Location, is_clockwise: bool) {
        self.stats.corners += 1;
        if is_clockwise {
            self.add(self.rect_as_path_[*loc as usize], false);
            *loc = get_adjacent_location(*loc, true);
        } else {
            *loc = get_adjacent_location(*loc, false);
            self.add(self.rect_as_path_[*loc as usize], false);
        }
    }

    /// clipper.rectclip.cpp:375-428
    /// C++: void RectClip64::GetNextLocation(const Path64& path, Location& loc, size_t& i, size_t highI)
    fn get_next_location(&mut self, path: &Path64, loc: &mut Location, i: &mut usize, high_i: usize) {
        match *loc {
            Location::Left => {
                while *i <= high_i && path[*i].x <= self.rect_.left {
                    *i += 1;
                }
                if *i > high_i {
                } else if path[*i].x >= self.rect_.right {
                    *loc = Location::Right;
                } else if path[*i].y <= self.rect_.top {
                    *loc = Location::Top;
                } else if path[*i].y >= self.rect_.bottom {
                    *loc = Location::Bottom;
                } else {
                    *loc = Location::Inside;
                }
            }

            Location::Top => {
                while *i <= high_i && path[*i].y <= self.rect_.top {
                    *i += 1;
                }
                if *i > high_i {
                } else if path[*i].y >= self.rect_.bottom {
                    *loc = Location::Bottom;
                } else if path[*i].x <= self.rect_.left {
                    *loc = Location::Left;
                } else if path[*i].x >= self.rect_.right {
                    *loc = Location::Right;
                } else {
                    *loc = Location::Inside;
                }
            }

            Location::Right => {
                while *i <= high_i && path[*i].x >= self.rect_.right {
                    *i += 1;
                }
                if *i > high_i {
                } else if path[*i].x <= self.rect_.left {
                    *loc = Location::Left;
                } else if path[*i].y <= self.rect_.top {
                    *loc = Location::Top;
                } else if path[*i].y >= self.rect_.bottom {
                    *loc = Location::Bottom;
                } else {
                    *loc = Location::Inside;
                }
            }

            Location::Bottom => {
                while *i <= high_i && path[*i].y >= self.rect_.bottom {
                    *i += 1;
                }
                if *i > high_i {
                } else if path[*i].y <= self.rect_.top {
                    *loc = Location::Top;
                } else if path[*i].x <= self.rect_.left {
                    *loc = Location::Left;
                } else if path[*i].x >= self.rect_.right {
                    *loc = Location::Right;
                } else {
                    *loc = Location::Inside;
                }
            }

            Location::Inside => {
                while *i <= high_i {
                    if path[*i].x < self.rect_.left {
                        *loc = Location::Left;
                    } else if path[*i].x > self.rect_.right {
                        *loc = Location::Right;
                    } else if path[*i].y > self.rect_.bottom {
                        *loc = Location::Bottom;
                    } else if path[*i].y < self.rect_.top {
                        *loc = Location::Top;
                    } else {
                        self.add(path[*i], false);
                        *i += 1;
                        continue;
                    }
                    break; //inner loop
                }
            }
        } //switch
    }

    /// clipper.rectclip.cpp:447-608
    /// C++: void RectClip64::ExecuteInternal(const Path64& path)
    fn execute_internal(&mut self, path: &Path64) {
        if path.len() < 1 {
            return;
        }

        let high_i = path.len() - 1;
        let mut prev = Location::Inside;
        // `Location loc;`: `GetLocation` always assigns it.
        let mut loc = Location::Inside;
        let mut crossing_loc = Location::Inside;
        let mut first_cross_ = Location::Inside;
        if !get_location(&self.rect_, &path[high_i], &mut loc) {
            let mut i = high_i;
            while i > 0 && !get_location(&self.rect_, &path[i - 1], &mut prev) {
                i -= 1;
            }
            if i == 0 {
                // all of path must be inside fRect
                self.stats.all_inside += 1;
                for pt in path {
                    self.add(*pt, false);
                }
                return;
            }
            if prev == Location::Inside {
                loc = Location::Inside;
            }
        }
        let starting_loc = loc;

        ///////////////////////////////////////////////////
        let mut i = 0usize;
        while i <= high_i {
            prev = loc;
            let crossing_prev = crossing_loc;

            self.get_next_location(path, &mut loc, &mut i, high_i);

            if i > high_i {
                break;
            }
            let mut ip = Point64::default();
            let mut ip2 = Point64::default();
            let prev_pt = if i != 0 { path[i - 1] } else { path[high_i] };

            crossing_loc = loc;
            if !get_intersection(&self.rect_as_path_, &path[i], &prev_pt, &mut crossing_loc, &mut ip) {
                // ie remaining outside
                if crossing_prev == Location::Inside {
                    let is_clockw = is_clockwise(prev, loc, &prev_pt, &path[i], &self.rect_mp_);
                    loop {
                        self.start_locs_.push(prev);
                        prev = get_adjacent_location(prev, is_clockw);
                        if !(prev != loc) {
                            break;
                        }
                    }
                    crossing_loc = crossing_prev; // still not crossed
                } else if prev != Location::Inside && prev != loc {
                    let is_clockw = is_clockwise(prev, loc, &prev_pt, &path[i], &self.rect_mp_);
                    loop {
                        self.add_corner_loc(&mut prev, is_clockw);
                        if !(prev != loc) {
                            break;
                        }
                    }
                }
                i += 1;
                continue;
            }

            ////////////////////////////////////////////////////
            // we must be crossing the rect boundary to get here
            ////////////////////////////////////////////////////

            if loc == Location::Inside {
                // path must be entering rect
                self.stats.entering += 1;
                if first_cross_ == Location::Inside {
                    first_cross_ = crossing_loc;
                    self.start_locs_.push(prev);
                } else if prev != crossing_loc {
                    let is_clockw = is_clockwise(prev, crossing_loc, &prev_pt, &path[i], &self.rect_mp_);
                    loop {
                        self.add_corner_loc(&mut prev, is_clockw);
                        if !(prev != crossing_loc) {
                            break;
                        }
                    }
                }
            } else if prev != Location::Inside {
                // passing right through rect. 'ip' here will be the second
                // intersect pt but we'll also need the first intersect pt (ip2)
                self.stats.passing_through += 1;
                loc = prev;
                get_intersection(&self.rect_as_path_, &prev_pt, &path[i], &mut loc, &mut ip2);
                if crossing_prev != Location::Inside && crossing_prev != loc {
                    //579
                    self.add_corner(crossing_prev, loc);
                }

                if first_cross_ == Location::Inside {
                    first_cross_ = loc;
                    self.start_locs_.push(prev);
                }

                loc = crossing_loc;
                self.add(ip2, false);
                if ip == ip2 {
                    // it's very likely that path[i] is on rect
                    self.stats.passing_through_one_point += 1;
                    get_location(&self.rect_, &path[i], &mut loc);
                    self.add_corner(crossing_loc, loc);
                    crossing_loc = loc;
                    continue;
                }
            } else {
                // path must be exiting rect
                self.stats.exiting += 1;
                loc = crossing_loc;
                if first_cross_ == Location::Inside {
                    first_cross_ = crossing_loc;
                }
            }

            self.add(ip, false);
        } //while i <= highI
        ///////////////////////////////////////////////////

        if first_cross_ == Location::Inside {
            // path never intersects
            if starting_loc != Location::Inside {
                // path is outside rect
                // but being outside, it still may not contain rect
                if self.path_bounds_.contains(&self.rect_) && path1_contains_path2(path, &self.rect_as_path_) {
                    // yep, the path does fully contain rect
                    self.stats.contains_rect += 1;
                    // so add rect to the solution
                    let is_clockwise_path = start_locs_are_clockwise(&self.start_locs_);
                    for j in 0..4usize {
                        let k = if is_clockwise_path { j } else { 3 - j }; // reverses result path
                        self.add(self.rect_as_path_[k], false);
                        // we may well need to do some splitting later, so
                        add_to_edge(&mut self.edges_, &mut self.op_container_, k * 2, self.results_[0]);
                    }
                }
            }
        } else if loc != Location::Inside && (loc != first_cross_ || self.start_locs_.len() > 2) {
            if !self.start_locs_.is_empty() {
                prev = loc;
                // (`AddCorner` does not touch `start_locs_`.)
                for k in 0..self.start_locs_.len() {
                    let loc2 = self.start_locs_[k];
                    if prev == loc2 {
                        continue;
                    }
                    let is_clockwise = heading_clockwise(prev, loc2);
                    self.add_corner_loc(&mut prev, is_clockwise);
                    prev = loc2;
                }
                loc = prev;
            }
            if loc != first_cross_ {
                let is_clockwise = heading_clockwise(loc, first_cross_);
                self.add_corner_loc(&mut loc, is_clockwise);
            }
        }
    }

    /// clipper.rectclip.cpp:610-667
    /// C++: void RectClip64::CheckEdges()
    fn check_edges(&mut self) {
        for i in 0..self.results_.len() {
            let mut op = self.results_[i];
            if op == NONE {
                continue;
            }
            let mut op2 = op;
            loop {
                let ops = &mut self.op_container_;
                if is_collinear(&ops[ops[op2].prev].pt, &ops[op2].pt, &ops[ops[op2].next].pt) {
                    self.stats.collinear_unlinked += 1;
                    if op2 == op {
                        op2 = unlink_op_back(ops, op2);
                        if op2 == NONE {
                            break;
                        }
                        op = ops[op2].prev;
                    } else {
                        op2 = unlink_op_back(ops, op2);
                        if op2 == NONE {
                            break;
                        }
                    }
                } else {
                    op2 = ops[op2].next;
                }
                if !(op2 != op) {
                    break;
                }
            }

            if op2 == NONE {
                self.results_[i] = NONE;
                continue;
            }
            self.results_[i] = op; // safety first

            let mut edge_set1 = get_edges_for_pt(&self.op_container_[self.op_container_[op].prev].pt, &self.rect_);
            op2 = op;
            loop {
                let edge_set2 = get_edges_for_pt(&self.op_container_[op2].pt, &self.rect_);
                if edge_set2 != 0 && self.op_container_[op2].edge == NONE {
                    let combined_set = edge_set1 & edge_set2;
                    for j in 0..4i32 {
                        if combined_set & (1 << j) != 0 {
                            let prev_pt = self.op_container_[self.op_container_[op2].prev].pt;
                            if is_heading_clockwise(&prev_pt, &self.op_container_[op2].pt, j) {
                                add_to_edge(&mut self.edges_, &mut self.op_container_, j as usize * 2, op2);
                            } else {
                                add_to_edge(&mut self.edges_, &mut self.op_container_, j as usize * 2 + 1, op2);
                            }
                        }
                    }
                }
                edge_set1 = edge_set2;
                op2 = self.op_container_[op2].next;
                if !(op2 != op) {
                    break;
                }
            }
        }
    }

    /// clipper.rectclip.cpp:669-845
    /// C++: void RectClip64::TidyEdges(size_t idx, OutPt2List& cw, OutPt2List& ccw)
    /// `cw` is `edges_[cw]` and `ccw` is `edges_[ccw]`.
    fn tidy_edges(&mut self, idx: usize, cw: usize, ccw: usize) {
        if self.edges_[ccw].is_empty() {
            return;
        }
        let is_horz = (idx == 1) || (idx == 3);
        let cw_is_toward_larger = (idx == 1) || (idx == 2);
        let mut i = 0usize;
        let mut j = 0usize;
        while i < self.edges_[cw].len() {
            // (`p1`, `p2`, `p1a`, `p2a`, `op` and `op2` are declared before the loop in the C++
            // and assigned in every round before they are read.)
            let ops = &mut self.op_container_;
            let mut p1 = self.edges_[cw][i];
            if p1 == NONE || ops[p1].next == ops[p1].prev {
                self.edges_[cw][i] = NONE;
                i += 1;
                j = 0;
                continue;
            }

            let j_lim = self.edges_[ccw].len();
            while j < j_lim && (self.edges_[ccw][j] == NONE || ops[self.edges_[ccw][j]].next == ops[self.edges_[ccw][j]].prev) {
                j += 1;
            }

            if j == j_lim {
                i += 1;
                j = 0;
                continue;
            }

            let cw_i = self.edges_[cw][i];
            let ccw_j = self.edges_[ccw][j];
            let (p2, p1a, p2a, op, op2);
            if cw_is_toward_larger {
                // p1 >>>> p1a;
                // p2 <<<< p2a;
                p1 = ops[cw_i].prev;
                p1a = cw_i;
                p2 = ccw_j;
                p2a = ops[ccw_j].prev;
            } else {
                // p1 <<<< p1a;
                // p2 >>>> p2a;
                p1 = cw_i;
                p1a = ops[cw_i].prev;
                p2 = ops[ccw_j].prev;
                p2a = ccw_j;
            }

            if (is_horz && !has_horz_overlap(&ops[p1].pt, &ops[p1a].pt, &ops[p2].pt, &ops[p2a].pt))
                || (!is_horz && !has_vert_overlap(&ops[p1].pt, &ops[p1a].pt, &ops[p2].pt, &ops[p2a].pt))
            {
                j += 1;
                continue;
            }

            // to get here we're either splitting or rejoining
            let is_rejoining = ops[cw_i].owner_idx != ops[ccw_j].owner_idx;

            if is_rejoining {
                self.stats.rejoins += 1;
                self.results_[ops[p2].owner_idx] = NONE;
                let new_idx = ops[p1].owner_idx;
                set_new_owner(ops, p2, new_idx);
            }

            // do the split or re-join
            if cw_is_toward_larger {
                // p1 >> | >> p1a;
                // p2 << | << p2a;
                ops[p1].next = p2;
                ops[p2].prev = p1;
                ops[p1a].prev = p2a;
                ops[p2a].next = p1a;
            } else {
                // p1 << | << p1a;
                // p2 >> | >> p2a;
                ops[p1].prev = p2;
                ops[p2].next = p1;
                ops[p1a].next = p2a;
                ops[p2a].prev = p1a;
            }

            if !is_rejoining {
                self.stats.splits += 1;
                let new_idx = self.results_.len();
                self.results_.push(p1a);
                set_new_owner(ops, p1a, new_idx);
            }

            if cw_is_toward_larger {
                op = p2;
                op2 = p1a;
            } else {
                op = p1;
                op2 = p2a;
            }
            self.results_[ops[op].owner_idx] = op;
            self.results_[ops[op2].owner_idx] = op2;

            // and now lots of work to get ready for the next loop

            let (op_is_larger, op2_is_larger);
            if is_horz {
                // X
                op_is_larger = ops[op].pt.x > ops[ops[op].prev].pt.x;
                op2_is_larger = ops[op2].pt.x > ops[ops[op2].prev].pt.x;
            } else {
                // Y
                op_is_larger = ops[op].pt.y > ops[ops[op].prev].pt.y;
                op2_is_larger = ops[op2].pt.y > ops[ops[op2].prev].pt.y;
            }

            if (ops[op].next == ops[op].prev) || (ops[op].pt == ops[ops[op].prev].pt) {
                if op2_is_larger == cw_is_toward_larger {
                    self.edges_[cw][i] = op2;
                    self.edges_[ccw][j] = NONE;
                    j += 1;
                } else {
                    self.edges_[ccw][j] = op2;
                    self.edges_[cw][i] = NONE;
                    i += 1;
                }
            } else if (ops[op2].next == ops[op2].prev) || (ops[op2].pt == ops[ops[op2].prev].pt) {
                if op_is_larger == cw_is_toward_larger {
                    self.edges_[cw][i] = op;
                    self.edges_[ccw][j] = NONE;
                    j += 1;
                } else {
                    self.edges_[ccw][j] = op;
                    self.edges_[cw][i] = NONE;
                    i += 1;
                }
            } else if op_is_larger == op2_is_larger {
                if op_is_larger == cw_is_toward_larger {
                    self.edges_[cw][i] = op;
                    uncouple_edge(&mut self.edges_, ops, op2);
                    add_to_edge(&mut self.edges_, ops, cw, op2);
                    self.edges_[ccw][j] = NONE;
                    j += 1;
                } else {
                    self.edges_[cw][i] = NONE;
                    i += 1;
                    self.edges_[ccw][j] = op2;
                    uncouple_edge(&mut self.edges_, ops, op);
                    add_to_edge(&mut self.edges_, ops, ccw, op);
                    j = 0;
                }
            } else {
                if op_is_larger == cw_is_toward_larger {
                    self.edges_[cw][i] = op;
                } else {
                    self.edges_[ccw][j] = op;
                }
                if op2_is_larger == cw_is_toward_larger {
                    self.edges_[cw][i] = op2;
                } else {
                    self.edges_[ccw][j] = op2;
                }
            }
        }
    }

    /// clipper.rectclip.cpp:847-875
    /// C++: Path64 RectClip64::GetPath(OutPt2*& op)
    /// `op` is `results_[k]`.
    fn get_path(&mut self, k: usize) -> Path64 {
        let ops = &mut self.op_container_;
        let mut op = self.results_[k];
        if op == NONE || ops[op].next == ops[op].prev {
            return Path64::new();
        }

        let mut op2 = ops[op].next;
        while op2 != NONE && op2 != op {
            if is_collinear(&ops[ops[op2].prev].pt, &ops[op2].pt, &ops[ops[op2].next].pt) {
                self.stats.collinear_unlinked += 1;
                op = ops[op2].prev;
                op2 = unlink_op(ops, op2);
            } else {
                op2 = ops[op2].next;
            }
        }
        op = op2; // needed for op cleanup
        self.results_[k] = op;
        if op2 == NONE {
            return Path64::new();
        }

        let mut result = Path64::new();
        result.push(ops[op].pt);
        op2 = ops[op].next;
        while op2 != op {
            result.push(ops[op2].pt);
            op2 = ops[op2].next;
        }
        result
    }

    /// clipper.rectclip.cpp:877-914
    /// C++: Paths64 RectClip64::Execute(const Paths64& paths)
    pub fn execute(&mut self, paths: &Paths64) -> Paths64 {
        let mut result = Paths64::new();
        if self.rect_.is_empty() {
            return result;
        }

        for path in paths {
            if path.len() < 3 {
                continue;
            }
            self.path_bounds_ = get_bounds(path);
            if !self.rect_.intersects(&self.path_bounds_) {
                continue; // the path must be completely outside rect_
            } else if self.rect_.contains(&self.path_bounds_) {
                // the path must be completely inside rect_
                result.push(path.clone());
                continue;
            }

            self.execute_internal(path);
            self.check_edges();
            for i in 0..4usize {
                self.tidy_edges(i, i * 2, i * 2 + 1);
            }

            for k in 0..self.results_.len() {
                let tmp = self.get_path(k);
                if !tmp.is_empty() {
                    result.push(tmp);
                }
            }

            //clean up after every loop
            self.op_container_ = Vec::new();
            self.results_.clear();
            for edge in self.edges_.iter_mut() {
                edge.clear();
            }
            self.start_locs_.clear();
        }
        result
    }
}

//------------------------------------------------------------------------------
// RectClipLines64
//------------------------------------------------------------------------------

/// clipper.rectclip.h:73-80
/// C++: class RectClipLines64 : public RectClip64
/// The base class is the member `base`.
#[derive(Debug, Clone)]
pub struct RectClipLines64 {
    base: RectClip64,
}

impl RectClipLines64 {
    /// clipper.rectclip.h:78
    /// C++: explicit RectClipLines64(const Rect64& rect) : RectClip64(rect) {};
    pub fn new(rect: &Rect64) -> Self {
        RectClipLines64 { base: RectClip64::new(rect) }
    }

    /// See `RectClipStats`: `all_inside`, `entering`, `exiting` and `passing_through` of
    /// `RectClipLines64::ExecuteInternal`.
    pub fn stats(&self) -> RectClipStats {
        self.base.stats
    }

    /// clipper.rectclip.cpp:920-944
    /// C++: Paths64 RectClipLines64::Execute(const Paths64& paths)
    pub fn execute(&mut self, paths: &Paths64) -> Paths64 {
        let mut result = Paths64::new();
        if self.base.rect_.is_empty() {
            return result;
        }

        for path in paths {
            let pathrec = get_bounds(path);
            if !self.base.rect_.intersects(&pathrec) {
                continue;
            }

            self.execute_internal(path);

            for k in 0..self.base.results_.len() {
                let tmp = self.get_path(k);
                if !tmp.is_empty() {
                    result.push(tmp);
                }
            }
            self.base.results_.clear();

            self.base.op_container_ = Vec::new();
            self.base.start_locs_.clear();
        }
        result
    }

    /// clipper.rectclip.cpp:946-1014
    /// C++: void RectClipLines64::ExecuteInternal(const Path64& path)
    fn execute_internal(&mut self, path: &Path64) {
        if self.base.rect_.is_empty() || path.len() < 2 {
            return;
        }

        self.base.results_.clear();
        self.base.op_container_ = Vec::new();
        self.base.start_locs_.clear();

        let mut i = 1usize;
        let high_i = path.len() - 1;

        let mut prev = Location::Inside;
        // `Location loc;`: `GetLocation` always assigns it.
        let mut loc = Location::Inside;
        let mut crossing_loc;
        if !get_location(&self.base.rect_, &path[0], &mut loc) {
            while i <= high_i && !get_location(&self.base.rect_, &path[i], &mut prev) {
                i += 1;
            }
            if i > high_i {
                // all of path must be inside fRect
                self.base.stats.all_inside += 1;
                for pt in path {
                    self.base.add(*pt, false);
                }
                return;
            }
            if prev == Location::Inside {
                loc = Location::Inside;
            }
            i = 1;
        }
        if loc == Location::Inside {
            self.base.add(path[0], false);
        }

        ///////////////////////////////////////////////////
        while i <= high_i {
            prev = loc;
            self.base.get_next_location(path, &mut loc, &mut i, high_i);
            if i > high_i {
                break;
            }
            let mut ip = Point64::default();
            let mut ip2 = Point64::default();
            let prev_pt = path[i - 1];

            crossing_loc = loc;
            if !get_intersection(&self.base.rect_as_path_, &path[i], &prev_pt, &mut crossing_loc, &mut ip) {
                // ie remaining outside
                i += 1;
                continue;
            }

            ////////////////////////////////////////////////////
            // we must be crossing the rect boundary to get here
            ////////////////////////////////////////////////////

            if loc == Location::Inside {
                // path must be entering rect
                self.base.stats.entering += 1;
                self.base.add(ip, true);
            } else if prev != Location::Inside {
                // passing right through rect. 'ip' here will be the second
                // intersect pt but we'll also need the first intersect pt (ip2)
                self.base.stats.passing_through += 1;
                crossing_loc = prev;
                get_intersection(&self.base.rect_as_path_, &prev_pt, &path[i], &mut crossing_loc, &mut ip2);
                self.base.add(ip2, true);
                self.base.add(ip, false);
            } else {
                // path must be exiting rect
                self.base.stats.exiting += 1;
                self.base.add(ip, false);
            }
        } //while i <= highI
        ///////////////////////////////////////////////////
    }

    /// clipper.rectclip.cpp:1016-1029
    /// C++: Path64 RectClipLines64::GetPath(OutPt2*& op)
    /// `op` is `results_[k]`.
    fn get_path(&mut self, k: usize) -> Path64 {
        let ops = &self.base.op_container_;
        let mut result = Path64::new();
        let mut op = self.base.results_[k];
        if op == NONE || op == ops[op].next {
            return result;
        }
        op = ops[op].next; // starting at path beginning
        self.base.results_[k] = op;
        result.push(ops[op].pt);
        let mut op2 = ops[op].next;
        while op2 != op {
            result.push(ops[op2].pt);
            op2 = ops[op2].next;
        }
        result
    }
}
