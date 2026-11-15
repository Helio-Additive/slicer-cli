//! `include/clipper2/clipper.h` of Clipper2 as the engine pins it (version 1.5.2,
//! `src/clipper2/Clipper2Lib`): the simple interface to the library, for `Path64`.
//! Compiled twice, see `mod.rs`. (The file is `clipper_h.rs` because `crate::clipper` is
//! the older ClipperLib.)
//!
//! A name without a suffix is the `Path64` overload or the `int64_t` instance of a
//! template; `_d` is the `PathD` one. The `PathsD` interface scales by a power of ten
//! (`std::pow(10, precision)`, the C library's `pow`) and reports a precision outside
//! -8..8 by throwing from `CheckPrecisionRange`: those functions return a `Result`, and the
//! `if (error_code) return ...` the C++ has for a build without exceptions never runs.
//!
//! `tests/clipper2_offset_native_parity.rs` holds the `Path64` half equal to the library,
//! `tests/clipper2_d_native_parity.rs` and `tests/clipper2_rectclip_native_parity.rs` the
//! rest (and their `_z` twins the build with `USINGZ`).

use super::clipper_core::*;
use super::clipper_engine::*;
use super::clipper_offset::{ClipperOffset, EndType, JoinType};
use super::clipper_rectclip::{RectClip64, RectClipLines64};
use std::fmt::Write as _;

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

// ---------------------------------------------------------------------------------------
// The `PathsD` overloads, `RectClip`, `RectClipLines`, `MakePath`, the outline of a
// PolyTree, and the templates with `T = double`.
// ---------------------------------------------------------------------------------------

/// clipper.h:47-59
/// C++: inline PathsD BooleanOp(ClipType cliptype, FillRule fillrule, const PathsD& subjects, const PathsD& clips, int precision = 2)
pub fn boolean_op_d(cliptype: ClipType, fillrule: FillRule, subjects: &PathsD, clips: &PathsD, mut precision: i32) -> Result<PathsD, Clipper2Exception> {
    let mut error_code = 0;
    check_precision_range(&mut precision, &mut error_code)?;
    let mut result = PathsD::new();
    if error_code != 0 {
        return Ok(result);
    }
    let mut clipper = ClipperD::new(precision)?;
    clipper.add_subject(subjects)?;
    clipper.add_clip(clips)?;
    clipper.execute(cliptype, fillrule, &mut result);
    Ok(result)
}

/// clipper.h:61-73
/// C++: inline void BooleanOp(ClipType cliptype, FillRule fillrule, const PathsD& subjects, const PathsD& clips, PolyTreeD& polytree, int precision = 2)
pub fn boolean_op_tree_d(cliptype: ClipType, fillrule: FillRule, subjects: &PathsD, clips: &PathsD, polytree: &mut PolyTreeD, mut precision: i32) -> Result<(), Clipper2Exception> {
    polytree.clear();
    let mut error_code = 0;
    check_precision_range(&mut precision, &mut error_code)?;
    if error_code != 0 {
        return Ok(());
    }
    let mut clipper = ClipperD::new(precision)?;
    clipper.add_subject(subjects)?;
    clipper.add_clip(clips)?;
    clipper.execute_tree(cliptype, fillrule, polytree);
    Ok(())
}

/// clipper.h:80-83
/// C++: inline PathsD Intersect(const PathsD& subjects, const PathsD& clips, FillRule fillrule, int decimal_prec = 2)
pub fn intersect_d(subjects: &PathsD, clips: &PathsD, fillrule: FillRule, decimal_prec: i32) -> Result<PathsD, Clipper2Exception> {
    boolean_op_d(ClipType::Intersection, fillrule, subjects, clips, decimal_prec)
}

/// clipper.h:90-93
/// C++: inline PathsD Union(const PathsD& subjects, const PathsD& clips, FillRule fillrule, int decimal_prec = 2)
pub fn union_d(subjects: &PathsD, clips: &PathsD, fillrule: FillRule, decimal_prec: i32) -> Result<PathsD, Clipper2Exception> {
    boolean_op_d(ClipType::Union, fillrule, subjects, clips, decimal_prec)
}

/// clipper.h:104-114
/// C++: inline PathsD Union(const PathsD& subjects, FillRule fillrule, int precision = 2)
pub fn union_subjects_d(subjects: &PathsD, fillrule: FillRule, mut precision: i32) -> Result<PathsD, Clipper2Exception> {
    let mut result = PathsD::new();
    let mut error_code = 0;
    check_precision_range(&mut precision, &mut error_code)?;
    if error_code != 0 {
        return Ok(result);
    }
    let mut clipper = ClipperD::new(precision)?;
    clipper.add_subject(subjects)?;
    clipper.execute(ClipType::Union, fillrule, &mut result);
    Ok(result)
}

/// clipper.h:121-124
/// C++: inline PathsD Difference(const PathsD& subjects, const PathsD& clips, FillRule fillrule, int decimal_prec = 2)
pub fn difference_d(subjects: &PathsD, clips: &PathsD, fillrule: FillRule, decimal_prec: i32) -> Result<PathsD, Clipper2Exception> {
    boolean_op_d(ClipType::Difference, fillrule, subjects, clips, decimal_prec)
}

/// clipper.h:131-134
/// C++: inline PathsD Xor(const PathsD& subjects, const PathsD& clips, FillRule fillrule, int decimal_prec = 2)
pub fn xor_d(subjects: &PathsD, clips: &PathsD, fillrule: FillRule, decimal_prec: i32) -> Result<PathsD, Clipper2Exception> {
    boolean_op_d(ClipType::Xor, fillrule, subjects, clips, decimal_prec)
}

/// clipper.h:148-163
/// C++: inline PathsD InflatePaths(const PathsD& paths, double delta, JoinType jt, EndType et, double miter_limit = 2.0, int precision = 2, double arc_tolerance = 0.0)
pub fn inflate_paths_d(paths: &PathsD, delta: f64, jt: JoinType, et: EndType, miter_limit: f64, mut precision: i32, arc_tolerance: f64) -> Result<PathsD, Clipper2Exception> {
    let mut error_code = 0;
    check_precision_range(&mut precision, &mut error_code)?;
    // `if (!delta) return paths;`
    if delta == 0.0 {
        return Ok(paths.clone());
    }
    if error_code != 0 {
        return Ok(PathsD::new());
    }
    let scale = 10.0f64.powf(precision as f64);
    let mut clip_offset = ClipperOffset::new(miter_limit, arc_tolerance, false, false);
    clip_offset.add_paths(&scale_paths64_from_d_uniform(paths, scale, &mut error_code)?, jt, et);
    if error_code != 0 {
        return Ok(PathsD::new());
    }
    let mut solution = Paths64::new();
    clip_offset.execute(delta * scale, &mut solution);
    scale_pathsd_from_64_uniform(&solution, 1.0 / scale, &mut error_code)
}

/// clipper.h:165-173, :180-183 with `T = double`
/// C++: inline Path<T> TranslatePath(const Path<T>& path, T dx, T dy)
/// `Point<T>(pt.x + dx, pt.y + dy)`: a new point, z = 0.
pub fn translate_path_d(path: &PathD, dx: f64, dy: f64) -> PathD {
    let mut result = PathD::with_capacity(path.len());
    for pt in path {
        result.push(PointD::new(pt.x + dx, pt.y + dy));
    }
    result
}

/// clipper.h:185-193, :200-203 with `T = double`
/// C++: inline Paths<T> TranslatePaths(const Paths<T>& paths, T dx, T dy)
pub fn translate_paths_d(paths: &PathsD, dx: f64, dy: f64) -> PathsD {
    let mut result = PathsD::with_capacity(paths.len());
    for path in paths {
        result.push(translate_path_d(path, dx, dy));
    }
    result
}

/// clipper.h:205-210
/// C++: inline Paths64 RectClip(const Rect64& rect, const Paths64& paths)
pub fn rect_clip(rect: &Rect64, paths: &Paths64) -> Paths64 {
    if rect.is_empty() || paths.is_empty() {
        return Paths64::new();
    }
    let mut rc = RectClip64::new(rect);
    rc.execute(paths)
}

/// clipper.h:212-217
/// C++: inline Paths64 RectClip(const Rect64& rect, const Path64& path)
pub fn rect_clip_path(rect: &Rect64, path: &Path64) -> Paths64 {
    if rect.is_empty() || path.is_empty() {
        return Paths64::new();
    }
    let mut rc = RectClip64::new(rect);
    rc.execute(&vec![path.clone()])
}

/// clipper.h:219-232
/// C++: inline PathsD RectClip(const RectD& rect, const PathsD& paths, int precision = 2)
pub fn rect_clip_d(rect: &RectD, paths: &PathsD, mut precision: i32) -> Result<PathsD, Clipper2Exception> {
    if rect.is_empty() || paths.is_empty() {
        return Ok(PathsD::new());
    }
    let mut error_code = 0;
    check_precision_range(&mut precision, &mut error_code)?;
    if error_code != 0 {
        return Ok(PathsD::new());
    }
    let scale = 10.0f64.powf(precision as f64);
    let r = scale_rect64_from_d(rect, scale);
    let mut rc = RectClip64::new(&r);
    let pp = scale_paths64_from_d_uniform(paths, scale, &mut error_code)?;
    if error_code != 0 {
        return Ok(PathsD::new()); // ie: error_code result is lost
    }
    scale_pathsd_from_64_uniform(&rc.execute(&pp), 1.0 / scale, &mut error_code)
}

/// clipper.h:234-237
/// C++: inline PathsD RectClip(const RectD& rect, const PathD& path, int precision = 2)
pub fn rect_clip_path_d(rect: &RectD, path: &PathD, precision: i32) -> Result<PathsD, Clipper2Exception> {
    rect_clip_d(rect, &vec![path.clone()], precision)
}

/// clipper.h:239-244
/// C++: inline Paths64 RectClipLines(const Rect64& rect, const Paths64& lines)
pub fn rect_clip_lines(rect: &Rect64, lines: &Paths64) -> Paths64 {
    if rect.is_empty() || lines.is_empty() {
        return Paths64::new();
    }
    let mut rcl = RectClipLines64::new(rect);
    rcl.execute(lines)
}

/// clipper.h:246-249
/// C++: inline Paths64 RectClipLines(const Rect64& rect, const Path64& line)
pub fn rect_clip_lines_path(rect: &Rect64, line: &Path64) -> Paths64 {
    rect_clip_lines(rect, &vec![line.clone()])
}

/// clipper.h:251-264
/// C++: inline PathsD RectClipLines(const RectD& rect, const PathsD& lines, int precision = 2)
pub fn rect_clip_lines_d(rect: &RectD, lines: &PathsD, mut precision: i32) -> Result<PathsD, Clipper2Exception> {
    if rect.is_empty() || lines.is_empty() {
        return Ok(PathsD::new());
    }
    let mut error_code = 0;
    check_precision_range(&mut precision, &mut error_code)?;
    if error_code != 0 {
        return Ok(PathsD::new());
    }
    let scale = 10.0f64.powf(precision as f64);
    let r = scale_rect64_from_d(rect, scale);
    let mut rcl = RectClipLines64::new(&r);
    let mut p = scale_paths64_from_d_uniform(lines, scale, &mut error_code)?;
    if error_code != 0 {
        return Ok(PathsD::new());
    }
    p = rcl.execute(&p);
    scale_pathsd_from_64_uniform(&p, 1.0 / scale, &mut error_code)
}

/// clipper.h:266-269
/// C++: inline PathsD RectClipLines(const RectD& rect, const PathD& line, int precision = 2)
pub fn rect_clip_lines_path_d(rect: &RectD, line: &PathD, precision: i32) -> Result<PathsD, Clipper2Exception> {
    rect_clip_lines_d(rect, &vec![line.clone()], precision)
}

/// clipper.h:281-286 `details::PolyPathToPathsD`
/// C++: inline void PolyPathToPathsD(const PolyPathD& polypath, PathsD& paths)
fn poly_path_to_paths_d(polypath: &PolyPathD, paths: &mut PathsD) {
    paths.push(polypath.polygon().clone());
    for child in polypath.iter() {
        poly_path_to_paths_d(child, paths);
    }
}

/// clipper.h:315-325 `details::OutlinePolyPath`
/// C++: static void OutlinePolyPath(std::ostream& os, size_t idx, bool isHole, size_t count, const std::string& preamble)
/// `std::endl` is a newline.
fn outline_poly_path(os: &mut String, idx: usize, is_hole: bool, count: usize, preamble: &str) {
    let plural = if count == 1 { "." } else { "s." };
    if is_hole {
        let _ = writeln!(os, "{preamble}+- Hole ({idx}) contains {count} nested polygon{plural}");
    } else {
        let _ = writeln!(os, "{preamble}+- Polygon ({idx}) contains {count} hole{plural}");
    }
}

/// clipper.h:327-334 `details::OutlinePolyPath64`
/// C++: static void OutlinePolyPath64(std::ostream& os, const PolyPath64& pp, size_t idx, std::string preamble)
fn outline_poly_path64(os: &mut String, pp: &PolyPath64, idx: usize, preamble: String) {
    outline_poly_path(os, idx, pp.is_hole(), pp.count(), &preamble);
    for i in 0..pp.count() {
        if pp.child(i).count() != 0 {
            outline_poly_path64(os, pp.child(i), i, preamble.clone() + "  ");
        }
    }
}

/// clipper.h:336-343 `details::OutlinePolyPathD`
/// C++: static void OutlinePolyPathD(std::ostream& os, const PolyPathD& pp, size_t idx, std::string preamble)
fn outline_poly_path_d(os: &mut String, pp: &PolyPathD, idx: usize, preamble: String) {
    outline_poly_path(os, idx, pp.is_hole(), pp.count(), &preamble);
    for i in 0..pp.count() {
        if pp.child(i).count() != 0 {
            outline_poly_path_d(os, pp.child(i), i, preamble.clone() + "  ");
        }
    }
}

/// clipper.h:360-369
/// C++: inline std::ostream& operator<< (std::ostream& os, const PolyTree64& pp)
/// What the operator writes to the stream.
pub fn poly_tree64_to_string(pp: &PolyTree64) -> String {
    let mut os = String::new();
    let plural = if pp.count() == 1 { " polygon." } else { " polygons." };
    let _ = write!(os, "\nPolytree with {}{}\n", pp.count(), plural);
    for i in 0..pp.count() {
        if pp.child(i).count() != 0 {
            outline_poly_path64(&mut os, pp.child(i), i, "  ".to_string());
        }
    }
    os.push_str("\n\n");
    os
}

/// clipper.h:371-381
/// C++: inline std::ostream& operator<< (std::ostream& os, const PolyTreeD& pp)
/// What the operator writes to the stream.
pub fn poly_tree_d_to_string(pp: &PolyTreeD) -> String {
    let mut os = String::new();
    let plural = if pp.count() == 1 { " polygon." } else { " polygons." };
    let _ = write!(os, "\nPolytree with {}{}\n", pp.count(), plural);
    for i in 0..pp.count() {
        if pp.child(i).count() != 0 {
            outline_poly_path_d(&mut os, pp.child(i), i, "  ".to_string());
        }
    }
    os.push_str("\n\n");
    if pp.level() == 0 {
        os.push('\n');
    }
    os
}

/// clipper.h:391-397
/// C++: inline PathsD PolyTreeToPathsD(const PolyTreeD& polytree)
pub fn poly_tree_to_paths_d(polytree: &PolyTreeD) -> PathsD {
    let mut result = PathsD::new();
    for child in polytree.iter() {
        poly_path_to_paths_d(child, &mut result);
    }
    result
}

/// clipper.h:345-356 `details::MakePathGeneric` with `T` a list of `int64_t` and `U = Point64`
/// C++: inline constexpr void MakePathGeneric(const T an_array, size_t array_size, std::vector<U>& result)
/// `result.emplace_back(an_array[i], an_array[i + 1], 0)`.
fn make_path_generic(an_array: &[i64], array_size: usize, result: &mut Path64) {
    result.reserve(array_size / 2);
    let mut i = 0;
    while i < array_size {
        result.push(Point64::new(an_array[i], an_array[i + 1]));
        i += 2;
    }
}

/// clipper.h:345-356 `details::MakePathGeneric` with `T` a list of `double` and `U = PointD`
fn make_path_generic_d(an_array: &[f64], array_size: usize, result: &mut PathD) {
    result.reserve(array_size / 2);
    let mut i = 0;
    while i < array_size {
        result.push(PointD::new(an_array[i], an_array[i + 1]));
        i += 2;
    }
}

/// clipper.h:408-421
/// C++: inline Path64 MakePath(const std::vector<T>& list)
/// A list of odd length throws.
pub fn make_path(list: &[i64]) -> Result<Path64, Clipper2Exception> {
    let size = list.len() - list.len() % 2;
    if list.len() != size {
        do_error(NON_PAIR_ERROR_I)?; // non-fatal without exception handling
    }
    let mut result = Path64::new();
    make_path_generic(list, size, &mut result);
    Ok(result)
}

/// clipper.h:423-435
/// C++: inline Path64 MakePath(const T(&list)[N])
/// Make the compiler error on unpaired value (i.e. no runtime effects).
pub fn make_path_array<const N: usize>(list: &[i64; N]) -> Path64 {
    const { assert!(N % 2 == 0, "MakePath requires an even number of arguments") };
    let mut result = Path64::new();
    make_path_generic(list, N, &mut result);
    result
}

/// clipper.h:437-450
/// C++: inline PathD MakePathD(const std::vector<T>& list)
pub fn make_path_d(list: &[f64]) -> Result<PathD, Clipper2Exception> {
    let size = list.len() - list.len() % 2;
    if list.len() != size {
        do_error(NON_PAIR_ERROR_I)?; // non-fatal without exception handling
    }
    let mut result = PathD::new();
    make_path_generic_d(list, size, &mut result);
    Ok(result)
}

/// clipper.h:452-464
/// C++: inline PathD MakePathD(const T(&list)[N])
pub fn make_path_d_array<const N: usize>(list: &[f64; N]) -> PathD {
    const { assert!(N % 2 == 0, "MakePath requires an even number of arguments") };
    let mut result = PathD::new();
    make_path_generic_d(list, N, &mut result);
    result
}

/// clipper.h:467-478 (under `USINGZ`; the z is dropped in the build without it)
/// C++: inline Path64 MakePathZ(const T2(&list)[N])
pub fn make_path_z<const N: usize>(list: &[i64; N]) -> Path64 {
    const { assert!(N % 3 == 0, "MakePathZ requires integer values in multiples of 3") };
    let size = N / 3;
    let mut result = vec![Point64::default(); size];
    for i in 0..size {
        result[i] = super::clipper2_build::point_xyz(list[i * 3], list[i * 3 + 1], list[i * 3 + 2]);
    }
    result
}

/// clipper.h:480-496 (under `USINGZ`; the z is dropped in the build without it)
/// C++: inline PathD MakePathZD(const T2(&list)[N])
/// The arm for a `T2` that is not an integer: `static_cast<int64_t>(list[i * 3 + 2])`,
/// truncated toward zero.
pub fn make_path_zd<const N: usize>(list: &[f64; N]) -> PathD {
    const { assert!(N % 3 == 0, "MakePathZD requires values in multiples of 3") };
    let size = N / 3;
    let mut result = vec![PointD::default(); size];
    for i in 0..size {
        result[i] = PointD::new_xyz(list[i * 3], list[i * 3 + 1], list[i * 3 + 2] as i64);
    }
    result
}

/// clipper.h:546-556
/// C++: inline PathD TrimCollinear(const PathD& path, int precision, bool is_open_path = false)
pub fn trim_collinear_d(path: &PathD, mut precision: i32, is_open_path: bool) -> Result<PathD, Clipper2Exception> {
    let mut error_code = 0;
    check_precision_range(&mut precision, &mut error_code)?;
    if error_code != 0 {
        return Ok(PathD::new());
    }
    let scale = 10.0f64.powf(precision as f64);
    let mut p = scale_path64_from_d_uniform(path, scale, &mut error_code)?;
    if error_code != 0 {
        return Ok(PathD::new());
    }
    p = trim_collinear(&p, is_open_path);
    scale_pathd_from_64_uniform(&p, 1.0 / scale, &mut error_code)
}

/// clipper.h:558-562 with `T = double`
/// C++: inline double Distance(const Point<T> pt1, const Point<T> pt2)
pub fn distance_d(pt1: PointD, pt2: PointD) -> f64 {
    distance_sqr_d(pt1, pt2).sqrt()
}

/// clipper.h:564-575 with `T = double`
/// C++: inline double Length(const Path<T>& path, bool is_closed_path = false)
pub fn length_d(path: &PathD, is_closed_path: bool) -> f64 {
    let mut result = 0.0;
    if path.len() < 2 {
        return result;
    }
    let stop = path.len() - 1;
    for it in 0..stop {
        result += distance_d(path[it], path[it + 1]);
    }
    if is_closed_path {
        result += distance_d(path[stop], path[0]);
    }
    result
}

/// clipper.h:578-583 with `T = double`
/// C++: inline bool NearCollinear(const Point<T>& pt1, const Point<T>& pt2, const Point<T>& pt3, double sin_sqrd_min_angle_rads)
pub fn near_collinear_d(pt1: &PointD, pt2: &PointD, pt3: &PointD, sin_sqrd_min_angle_rads: f64) -> bool {
    let cp = cross_product_d(pt1, pt2, pt3).abs();
    (cp * cp) / (distance_sqr_d(*pt1, *pt2) * distance_sqr_d(*pt2, *pt3)) < sin_sqrd_min_angle_rads
}

/// clipper.h:585-591 with `T = int64_t`
/// C++: inline Path<T> Ellipse(const Rect<T>& rect, size_t steps = 0)
pub fn ellipse_rect(rect: &Rect64, steps: usize) -> Path64 {
    ellipse(&rect.mid_point(), rect.width() as f64 * 0.5, rect.height() as f64 * 0.5, steps)
}

/// clipper.h:585-591 with `T = double`
/// C++: inline Path<T> Ellipse(const Rect<T>& rect, size_t steps = 0)
pub fn ellipse_rect_d(rect: &RectD, steps: usize) -> PathD {
    ellipse_d(&rect.mid_point(), rect.width() * 0.5, rect.height() * 0.5, steps)
}

/// clipper.h:593-616 with `T = double`
/// C++: inline Path<T> Ellipse(const Point<T>& center, double radiusX, double radiusY = 0, size_t steps = 0)
/// The points are `PointD(double, double)`: z = 0.
pub fn ellipse_d(center: &PointD, radius_x: f64, mut radius_y: f64, mut steps: usize) -> PathD {
    if radius_x <= 0.0 {
        return PathD::new();
    }
    if radius_y <= 0.0 {
        radius_y = radius_x;
    }
    if steps <= 2 {
        steps = (PI * ((radius_x + radius_y) / 2.0).sqrt()) as usize;
    }

    let (si, co) = crate::clipper::sin_cos(2.0 * PI / steps as f64);
    let (mut dx, mut dy) = (co, si);
    let mut result = PathD::with_capacity(steps);
    result.push(PointD::new(center.x + radius_x, center.y));
    for _ in 1..steps {
        // `center.x + radiusX * dx`: fma(radiusX, dx, center.x)
        result.push(PointD::new(radius_x.mul_add(dx, center.x), radius_y.mul_add(dy, center.y)));
        let x = dx.mul_add(co, -(dy * si));
        dy = dy.mul_add(co, dx * si);
        dx = x;
    }
    result
}

/// clipper.h:641-706 with `T = double`
/// C++: inline Path<T> SimplifyPath(const Path<T> &path, double epsilon, bool isClosedPath = true)
pub fn simplify_path_d(path: &PathD, epsilon: f64, is_closed_path: bool) -> PathD {
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
        dist_sqr[0] = perpendic_dist_from_line_sqrd_d(&path[0], &path[high], &path[1]);
        dist_sqr[high] = perpendic_dist_from_line_sqrd_d(&path[high], &path[0], &path[high - 1]);
    } else {
        dist_sqr[0] = MAX_DBL;
        dist_sqr[high] = MAX_DBL;
    }
    for i in 1..high {
        dist_sqr[i] = perpendic_dist_from_line_sqrd_d(&path[i], &path[i - 1], &path[i + 1]);
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
            dist_sqr[curr] = perpendic_dist_from_line_sqrd_d(&path[curr], &path[prior], &path[next]);
        }
        if is_closed_path || ((prior != 0) && (prior != high)) {
            dist_sqr[prior] = perpendic_dist_from_line_sqrd_d(&path[prior], &path[prior2], &path[curr]);
        }
    }
    let mut result = PathD::with_capacity(len);
    for i in 0..len {
        if !flags[i] {
            result.push(path[i]);
        }
    }
    result
}

/// clipper.h:708-717 with `T = double`
/// C++: inline Paths<T> SimplifyPaths(const Paths<T> &paths, double epsilon, bool isClosedPath = true)
pub fn simplify_paths_d(paths: &PathsD, epsilon: f64, is_closed_path: bool) -> PathsD {
    let mut result = PathsD::with_capacity(paths.len());
    for path in paths {
        result.push(simplify_path_d(path, epsilon, is_closed_path));
    }
    result
}

/// clipper.h:719-738 with `T = double`
/// C++: inline void RDP(const Path<T> path, std::size_t begin, std::size_t end, double epsSqrd, std::vector<bool>& flags)
fn rdp_d(path: &PathD, begin: usize, mut end: usize, eps_sqrd: f64, flags: &mut [bool]) {
    let mut idx = 0usize;
    let mut max_d = 0.0f64;
    while end > begin && path[begin] == path[end] {
        flags[end] = false;
        end -= 1;
    }
    for i in begin + 1..end {
        // PerpendicDistFromLineSqrd - avoids expensive Sqrt()
        let d = perpendic_dist_from_line_sqrd_d(&path[i], &path[begin], &path[end]);
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
        rdp_d(path, begin, idx, eps_sqrd, flags);
    }
    if idx < end - 1 {
        rdp_d(path, idx, end, eps_sqrd, flags);
    }
}

/// clipper.h:740-755 with `T = double`
/// C++: inline Path<T> RamerDouglasPeucker(const Path<T>& path, double epsilon)
pub fn ramer_douglas_peucker_d(path: &PathD, epsilon: f64) -> PathD {
    let len = path.len();
    if len < 5 {
        return path.clone();
    }
    let mut flags = vec![false; len];
    flags[0] = true;
    flags[len - 1] = true;
    rdp_d(path, 0, len - 1, sqr(epsilon), &mut flags);
    let mut result = PathD::with_capacity(len);
    for i in 0..len {
        if flags[i] {
            result.push(path[i]);
        }
    }
    result
}

/// clipper.h:757-766 with `T = double`
/// C++: inline Paths<T> RamerDouglasPeucker(const Paths<T>& paths, double epsilon)
pub fn ramer_douglas_peucker_paths_d(paths: &PathsD, epsilon: f64) -> PathsD {
    let mut result = PathsD::with_capacity(paths.len());
    for path in paths {
        result.push(ramer_douglas_peucker_d(path, epsilon));
    }
    result
}
