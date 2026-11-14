//! `include/clipper2/clipper.h` of Clipper2 as the engine pins it (version 1.5.2,
//! `src/clipper2/Clipper2Lib`): the simple interface to the library, for `Path64`.
//! Compiled twice, see `mod.rs`. (The file is `clipper_h.rs` because `crate::clipper` is
//! the older ClipperLib.)
//!
//! Ported: `BooleanOp`, `Intersect`, `Union`, `Difference`, `Xor`, `InflatePaths`,
//! `TranslatePath(s)`, `PolyTreeToPaths64`, `CheckPolytreeFullyContainsChildren`,
//! `TrimCollinear`, `Distance`, `Length`, `NearCollinear`, `Ellipse`, `SimplifyPath(s)`,
//! `RamerDouglasPeucker`. Not ported: the `PathsD` overloads (they need `ClipperD` and the
//! scaling templates), `RectClip` and `RectClipLines` (clipper.rectclip.cpp), `MakePath`,
//! the stream operators. Nothing the engine calls reaches those.

use super::clipper_core::*;
use super::clipper_engine::*;
use super::clipper_offset::{ClipperOffset, EndType, JoinType};

/// clipper.h:26-35
/// C++: inline Paths64 BooleanOp(ClipType cliptype, FillRule fillrule, const Paths64& subjects, const Paths64& clips)
pub fn boolean_op(cliptype: ClipType, fillrule: FillRule, subjects: &Paths64, clips: &Paths64) -> Paths64 {
    let mut result = Paths64::new();
    let mut clipper = Clipper64::new();
    clipper.add_subject(subjects);
    clipper.add_clip(clips);
    clipper.execute(cliptype, fillrule, &mut result);
    result
}

/// clipper.h:37-45
/// C++: inline void BooleanOp(ClipType cliptype, FillRule fillrule, const Paths64& subjects, const Paths64& clips, PolyTree64& solution)
pub fn boolean_op_tree(cliptype: ClipType, fillrule: FillRule, subjects: &Paths64, clips: &Paths64, solution: &mut PolyTree64) {
    let mut sol_open = Paths64::new();
    let mut clipper = Clipper64::new();
    clipper.add_subject(subjects);
    clipper.add_clip(clips);
    clipper.execute_tree_open(cliptype, fillrule, solution, &mut sol_open);
}

/// clipper.h:75-78
/// C++: inline Paths64 Intersect(const Paths64& subjects, const Paths64& clips, FillRule fillrule)
pub fn intersect(subjects: &Paths64, clips: &Paths64, fillrule: FillRule) -> Paths64 {
    boolean_op(ClipType::Intersection, fillrule, subjects, clips)
}

/// clipper.h:85-88
/// C++: inline Paths64 Union(const Paths64& subjects, const Paths64& clips, FillRule fillrule)
pub fn union(subjects: &Paths64, clips: &Paths64, fillrule: FillRule) -> Paths64 {
    boolean_op(ClipType::Union, fillrule, subjects, clips)
}

/// clipper.h:95-102
/// C++: inline Paths64 Union(const Paths64& subjects, FillRule fillrule)
pub fn union_subjects(subjects: &Paths64, fillrule: FillRule) -> Paths64 {
    let mut result = Paths64::new();
    let mut clipper = Clipper64::new();
    clipper.add_subject(subjects);
    clipper.execute(ClipType::Union, fillrule, &mut result);
    result
}

/// clipper.h:116-119
/// C++: inline Paths64 Difference(const Paths64& subjects, const Paths64& clips, FillRule fillrule)
pub fn difference(subjects: &Paths64, clips: &Paths64, fillrule: FillRule) -> Paths64 {
    boolean_op(ClipType::Difference, fillrule, subjects, clips)
}

/// clipper.h:126-129
/// C++: inline Paths64 Xor(const Paths64& subjects, const Paths64& clips, FillRule fillrule)
pub fn xor(subjects: &Paths64, clips: &Paths64, fillrule: FillRule) -> Paths64 {
    boolean_op(ClipType::Xor, fillrule, subjects, clips)
}

/// clipper.h:136-146
/// C++: inline Paths64 InflatePaths(const Paths64& paths, double delta, JoinType jt, EndType et, double miter_limit = 2.0, double arc_tolerance = 0.0)
pub fn inflate_paths(paths: &Paths64, delta: f64, jt: JoinType, et: EndType, miter_limit: f64, arc_tolerance: f64) -> Paths64 {
    // `if (!delta) return paths;`
    if delta == 0.0 {
        return paths.clone();
    }
    let mut clip_offset = ClipperOffset::new(miter_limit, arc_tolerance, false, false);
    clip_offset.add_paths(paths, jt, et);
    let mut solution = Paths64::new();
    clip_offset.execute(delta, &mut solution);
    solution
}

/// clipper.h:166-179 with `T = int64_t`
/// C++: inline Path<T> TranslatePath(const Path<T>& path, T dx, T dy)
/// `Point<T>(pt.x + dx, pt.y + dy)`: a new point, z = 0.
pub fn translate_path(path: &Path64, dx: i64, dy: i64) -> Path64 {
    let mut result = Path64::with_capacity(path.len());
    for pt in path {
        result.push(Point64::new(pt.x.wrapping_add(dx), pt.y.wrapping_add(dy)));
    }
    result
}

/// clipper.h:186-199 with `T = int64_t`
/// C++: inline Paths<T> TranslatePaths(const Paths<T>& paths, T dx, T dy)
pub fn translate_paths(paths: &Paths64, dx: i64, dy: i64) -> Paths64 {
    let mut result = Paths64::with_capacity(paths.len());
    for path in paths {
        result.push(translate_path(path, dx, dy));
    }
    result
}

/// clipper.h:274-279 `details::PolyPathToPaths64`
/// C++: inline void PolyPathToPaths64(const PolyPath64& polypath, Paths64& paths)
fn poly_path_to_paths64(polypath: &PolyPath64, paths: &mut Paths64) {
    paths.push(polypath.polygon().clone());
    for child in polypath.iter() {
        poly_path_to_paths64(child, paths);
    }
}

/// clipper.h:288-312 `details::PolyPath64ContainsChildren`
/// C++: inline bool PolyPath64ContainsChildren(const PolyPath64& pp)
fn poly_path64_contains_children(pp: &PolyPath64) -> bool {
    for child in pp.iter() {
        // return false if this child isn't fully contained by its parent

        // checking for a single vertex outside is a bit too crude since
        // it doesn't account for rounding errors. It's better to check
        // for consecutive vertices found outside the parent's polygon.

        let mut outside_cnt = 0;
        for pt in child.polygon() {
            let result = point_in_polygon(pt, pp.polygon());
            if result == PointInPolygonResult::IsInside {
                outside_cnt -= 1;
            } else if result == PointInPolygonResult::IsOutside {
                outside_cnt += 1;
            }
            if outside_cnt > 1 {
                return false;
            } else if outside_cnt < -1 {
                break;
            }
        }

        // now check any nested children too
        if child.count() > 0 && !poly_path64_contains_children(child) {
            return false;
        }
    }
    true
}

/// clipper.h:384-390
/// C++: inline Paths64 PolyTreeToPaths64(const PolyTree64& polytree)
pub fn poly_tree_to_paths64(polytree: &PolyTree64) -> Paths64 {
    let mut result = Paths64::new();
    for child in polytree.iter() {
        poly_path_to_paths64(child, &mut result);
    }
    result
}

/// clipper.h:400-407
/// C++: inline bool CheckPolytreeFullyContainsChildren(const PolyTree64& polytree)
pub fn check_polytree_fully_contains_children(polytree: &PolyTree64) -> bool {
    for child in polytree.iter() {
        if child.count() > 0 && !poly_path64_contains_children(child) {
            return false;
        }
    }
    true
}

/// clipper.h:503-549
/// C++: inline Path64 TrimCollinear(const Path64& p, bool is_open_path = false)
/// The iterators are indices into `p`.
pub fn trim_collinear(p: &Path64, is_open_path: bool) -> Path64 {
    let len = p.len();
    if len < 3 {
        if !is_open_path || len < 2 || p[0] == p[1] {
            return Path64::new();
        } else {
            return p.clone();
        }
    }

    let mut dst = Path64::with_capacity(len);
    let mut src_it = 0usize;
    let mut prev_it;
    let mut stop = len - 1;

    if !is_open_path {
        while src_it != stop && is_collinear(&p[stop], &p[src_it], &p[src_it + 1]) {
            src_it += 1;
        }
        while src_it != stop && is_collinear(&p[stop - 1], &p[stop], &p[src_it]) {
            stop -= 1;
        }
        if src_it == stop {
            return Path64::new();
        }
    }

    prev_it = src_it;
    src_it += 1;
    dst.push(p[prev_it]);
    while src_it != stop {
        if !is_collinear(&p[prev_it], &p[src_it], &p[src_it + 1]) {
            prev_it = src_it;
            dst.push(p[prev_it]);
        }
        src_it += 1;
    }

    if is_open_path {
        dst.push(p[src_it]);
    } else if !is_collinear(&p[prev_it], &p[stop], &dst[0]) {
        dst.push(p[stop]);
    } else {
        while dst.len() > 2 && is_collinear(&dst[dst.len() - 1], &dst[dst.len() - 2], &dst[0]) {
            dst.pop();
        }
        if dst.len() < 3 {
            return Path64::new();
        }
    }
    dst
}

/// clipper.h:563-567 with `T = int64_t`
/// C++: inline double Distance(const Point<T> pt1, const Point<T> pt2)
pub fn distance(pt1: Point64, pt2: Point64) -> f64 {
    distance_sqr(pt1, pt2).sqrt()
}

/// clipper.h:569-580 with `T = int64_t`
/// C++: inline double Length(const Path<T>& path, bool is_closed_path = false)
pub fn length(path: &Path64, is_closed_path: bool) -> f64 {
    let mut result = 0.0;
    if path.len() < 2 {
        return result;
    }
    let stop = path.len() - 1;
    for it in 0..stop {
        result += distance(path[it], path[it + 1]);
    }
    if is_closed_path {
        result += distance(path[stop], path[0]);
    }
    result
}

/// clipper.h:583-588 with `T = int64_t`
/// C++: inline bool NearCollinear(const Point<T>& pt1, const Point<T>& pt2, const Point<T>& pt3, double sin_sqrd_min_angle_rads)
pub fn near_collinear(pt1: &Point64, pt2: &Point64, pt3: &Point64, sin_sqrd_min_angle_rads: f64) -> bool {
    let cp = cross_product(pt1, pt2, pt3).abs();
    (cp * cp) / (distance_sqr(*pt1, *pt2) * distance_sqr(*pt2, *pt3)) < sin_sqrd_min_angle_rads
}

/// clipper.h:598-621 with `T = int64_t`
/// C++: inline Path<T> Ellipse(const Point<T>& center, double radiusX, double radiusY = 0, size_t steps = 0)
/// The points are `Point64(double, double)`: rounded halves away from zero, z = 0.
pub fn ellipse(center: &Point64, radius_x: f64, mut radius_y: f64, mut steps: usize) -> Path64 {
    if radius_x <= 0.0 {
        return Path64::new();
    }
    if radius_y <= 0.0 {
        radius_y = radius_x;
    }
    if steps <= 2 {
        steps = (PI * ((radius_x + radius_y) / 2.0).sqrt()) as usize;
    }

    // `std::sin(2 * PI / steps)` and `std::cos(2 * PI / steps)`: one argument, one call
    // of `__sincos_stret` in the reference build.
    let (si, co) = crate::clipper::sin_cos(2.0 * PI / steps as f64);
    let (mut dx, mut dy) = (co, si);
    let mut result = Path64::with_capacity(steps);
    result.push(point64_from_doubles(center.x as f64 + radius_x, center.y as f64, 0));
    for _ in 1..steps {
        // `center.x + radiusX * dx`: fma(radiusX, dx, center.x)
        result.push(point64_from_doubles(radius_x.mul_add(dx, center.x as f64), radius_y.mul_add(dy, center.y as f64), 0));
        // `dx * co - dy * si`: fma(dx, co, -(dy * si)); `dy * co + dx * si`: fma(dy, co, dx * si)
        let x = dx.mul_add(co, -(dy * si));
        dy = dy.mul_add(co, dx * si);
        dx = x;
    }
    result
}

/// clipper.h:623-632
/// C++: inline size_t GetNext(size_t current, size_t high, const std::vector<bool>& flags)
fn get_next(mut current: usize, high: usize, flags: &[bool]) -> usize {
    current += 1;
    while current <= high && flags[current] {
        current += 1;
    }
    if current <= high {
        return current;
    }
    current = 0;
    while flags[current] {
        current += 1;
    }
    current
}

/// clipper.h:634-644
/// C++: inline size_t GetPrior(size_t current, size_t high, const std::vector<bool>& flags)
fn get_prior(mut current: usize, high: usize, flags: &[bool]) -> usize {
    if current == 0 {
        current = high;
    } else {
        current -= 1;
    }
    while current > 0 && flags[current] {
        current -= 1;
    }
    if !flags[current] {
        return current;
    }
    current = high;
    while flags[current] {
        current -= 1;
    }
    current
}

/// clipper.h:646-709 with `T = int64_t`
/// C++: inline Path<T> SimplifyPath(const Path<T> &path, double epsilon, bool isClosedPath = true)
pub fn simplify_path(path: &Path64, epsilon: f64, is_closed_path: bool) -> Path64 {
    let len = path.len();
    let high = len.wrapping_sub(1);
    let eps_sqr = sqr(epsilon);
    if len < 4 {
        return path.clone();
    }

    let mut flags = vec![false; len];
    let mut dist_sqr = vec![0.0f64; len];
    let mut prior;
    let mut curr = 0usize;
    let mut start;
    let mut next;
    let mut prior2;
    if is_closed_path {
        dist_sqr[0] = perpendic_dist_from_line_sqrd(&path[0], &path[high], &path[1]);
        dist_sqr[high] = perpendic_dist_from_line_sqrd(&path[high], &path[0], &path[high - 1]);
    } else {
        dist_sqr[0] = MAX_DBL;
        dist_sqr[high] = MAX_DBL;
    }
    for i in 1..high {
        dist_sqr[i] = perpendic_dist_from_line_sqrd(&path[i], &path[i - 1], &path[i + 1]);
    }

    loop {
        if dist_sqr[curr] > eps_sqr {
            start = curr;
            loop {
                curr = get_next(curr, high, &flags);
                if !(curr != start && dist_sqr[curr] > eps_sqr) {
                    break;
                }
            }
            if curr == start {
                break;
            }
        }

        prior = get_prior(curr, high, &flags);
        next = get_next(curr, high, &flags);
        if next == prior {
            break;
        }

        // flag for removal the smaller of adjacent 'distances'
        if dist_sqr[next] < dist_sqr[curr] {
            prior2 = prior;
            prior = curr;
            curr = next;
            next = get_next(next, high, &flags);
        } else {
            prior2 = get_prior(prior, high, &flags);
        }

        flags[curr] = true;
        curr = next;
        next = get_next(next, high, &flags);

        if is_closed_path || ((curr != high) && (curr != 0)) {
            dist_sqr[curr] = perpendic_dist_from_line_sqrd(&path[curr], &path[prior], &path[next]);
        }
        if is_closed_path || ((prior != 0) && (prior != high)) {
            dist_sqr[prior] = perpendic_dist_from_line_sqrd(&path[prior], &path[prior2], &path[curr]);
        }
    }
    let mut result = Path64::with_capacity(len);
    for i in 0..len {
        if !flags[i] {
            result.push(path[i]);
        }
    }
    result
}

/// clipper.h:711-720 with `T = int64_t`
/// C++: inline Paths<T> SimplifyPaths(const Paths<T> &paths, double epsilon, bool isClosedPath = true)
pub fn simplify_paths(paths: &Paths64, epsilon: f64, is_closed_path: bool) -> Paths64 {
    let mut result = Paths64::with_capacity(paths.len());
    for path in paths {
        result.push(simplify_path(path, epsilon, is_closed_path));
    }
    result
}

/// clipper.h:722-741 with `T = int64_t`
/// C++: inline void RDP(const Path<T> path, std::size_t begin, std::size_t end, double epsSqrd, std::vector<bool>& flags)
fn rdp(path: &Path64, begin: usize, mut end: usize, eps_sqrd: f64, flags: &mut [bool]) {
    let mut idx = 0usize;
    let mut max_d = 0.0f64;
    while end > begin && path[begin] == path[end] {
        flags[end] = false;
        end -= 1;
    }
    for i in begin + 1..end {
        // PerpendicDistFromLineSqrd - avoids expensive Sqrt()
        let d = perpendic_dist_from_line_sqrd(&path[i], &path[begin], &path[end]);
        if d <= max_d {
            continue;
        }
        max_d = d;
        idx = i;
    }
    if max_d <= eps_sqrd {
        return;
    }
    flags[idx] = true;
    if idx > begin + 1 {
        rdp(path, begin, idx, eps_sqrd, flags);
    }
    if idx < end - 1 {
        rdp(path, idx, end, eps_sqrd, flags);
    }
}

/// clipper.h:743-757 with `T = int64_t`
/// C++: inline Path<T> RamerDouglasPeucker(const Path<T>& path, double epsilon)
pub fn ramer_douglas_peucker(path: &Path64, epsilon: f64) -> Path64 {
    let len = path.len();
    if len < 5 {
        return path.clone();
    }
    let mut flags = vec![false; len];
    flags[0] = true;
    flags[len - 1] = true;
    rdp(path, 0, len - 1, sqr(epsilon), &mut flags);
    let mut result = Path64::with_capacity(len);
    for i in 0..len {
        if flags[i] {
            result.push(path[i]);
        }
    }
    result
}

/// clipper.h:759-769 with `T = int64_t`
/// C++: inline Paths<T> RamerDouglasPeucker(const Paths<T>& paths, double epsilon)
pub fn ramer_douglas_peucker_paths(paths: &Paths64, epsilon: f64) -> Paths64 {
    let mut result = Paths64::with_capacity(paths.len());
    for path in paths {
        result.push(ramer_douglas_peucker(path, epsilon));
    }
    result
}
