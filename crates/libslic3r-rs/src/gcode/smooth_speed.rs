//! Smooth-speed discontinuity ramping for perimeter loops.
//!
//! Faithful port of the BambuStudio `GCode` smooth-speed helpers
//! (src/libslic3r/GCode.cpp): `mapping_speed`, `get_speed_coor_x`,
//! `need_smooth_speed`, `split_and_mapping_speed`, `merge_same_speed_paths`,
//! `set_speed_transition`, and `smooth_speed_discontinuity_area`.
//!
//! These split perimeter sub-paths at speed discontinuities and ramp the
//! feedrate `F` gradually (f(x) = coeff * x^2) across the transition, producing
//! the near-continuous F distribution that the reference slicer emits.
//!
//! All lengths are in scaled coord units (coord_t == i64), matching C++.
//! `scale_`/`unscale_` are the double-valued macros (val / SCALING_FACTOR and
//! val * SCALING_FACTOR with SCALING_FACTOR = 0.00001), i.e. multiply/divide by
//! 100000.0 — NOT crate::scale (which rounds to i64).

use crate::extrusion_entity::{ExtrusionPath, ExtrusionRole};
use crate::geometry::{Point, Polyline};

// C++: using ExtrusionPaths = std::vector<ExtrusionPath>;
pub type ExtrusionPaths = Vec<ExtrusionPath>;

// GCode.cpp:88 — static const double smooth_speed_step = 10;
const SMOOTH_SPEED_STEP: f64 = 10.0;
// GCode.cpp:91 — static const double min_step_length = scale_(0.4);
#[inline]
fn min_step_length() -> f64 {
    scale_(0.4)
}

/// `scale_` macro (double-valued): val / SCALING_FACTOR == val * 100000.0.
#[inline]
fn scale_(val: f64) -> f64 {
    val / 0.00001
}
/// `unscale_` macro (double-valued): val * SCALING_FACTOR == val / 100000.0.
#[inline]
fn unscale_(val: f64) -> f64 {
    val * 0.00001
}

/// `Point operator*(const Point&, const double&)` — coord_t(double) truncates.
#[inline]
fn point_mul_f64(l: Point, r: f64) -> Point {
    Point::new((l.x() as f64 * r) as i64, (l.y() as f64 * r) as i64)
}

/// `ExtrusionPath(Polyline, const ExtrusionPath&)`: copy the source path's
/// properties (role/width/mm3/etc.) and set a new polyline.
#[inline]
fn path_from(polyline: Polyline, src: &ExtrusionPath) -> ExtrusionPath {
    let mut p = src.clone();
    p.polyline = polyline;
    p
}

/// GCode.cpp:5918-5923 — `double GCode::mapping_speed(double dist)` ; f(x)=coeff*x^2.
#[inline]
fn mapping_speed(coeff: f64, dist: f64) -> f64 {
    // GCode.cpp:5920-5921
    if dist <= 0.0 {
        return 0.0;
    }
    coeff * dist.powi(2)
}

/// GCode.cpp:5925-5929 — `double GCode::get_speed_coor_x(double speed)`.
#[inline]
fn get_speed_coor_x(coeff: f64, speed: f64) -> f64 {
    // GCode.cpp:5927-5928
    let temp = speed / coeff;
    temp.sqrt()
}

/// GCode.cpp:5965-5971 — `static bool need_smooth_speed(other, this)`.
#[inline]
fn need_smooth_speed(other_path: &ExtrusionPath, this_path: &ExtrusionPath) -> bool {
    // GCode.cpp:5967
    this_path.smooth_speed - other_path.smooth_speed > SMOOTH_SPEED_STEP
}

/// Polyline length helper (scaled units).
#[inline]
fn poly_len(p: &Polyline) -> f64 {
    p.length()
}

/// GCode.cpp:5973-6121 — `void GCode::split_and_mapping_speed(...)`.
///
/// Splits `this_path` (consumed from the front, or rear when `!split_from_left`)
/// into a sequence of short sub-paths whose `smooth_speed` ramps from
/// `other_path_v` up toward `final_v` over at most `max_smooth_length`. The
/// produced (ramped) sub-paths are appended to `interpolated_paths`, and
/// `this_path` is left holding the unconsumed remainder (all set to `final_v`).
#[allow(clippy::too_many_arguments)]
pub fn split_and_mapping_speed(
    coeff: f64,
    other_path_v: f64,
    mut final_v: f64,
    this_path: &mut ExtrusionPaths,
    max_smooth_length: f64,
    interpolated_paths: &mut ExtrusionPaths,
    split_from_left: bool,
) {
    // GCode.cpp:5975-5976
    if this_path.is_empty() || max_smooth_length == 0.0 {
        return;
    }

    // GCode.cpp:5978
    let mut splited_path: ExtrusionPaths = Vec::new();

    // GCode.cpp:5981-5983 — get params
    let this_path_x = scale_(get_speed_coor_x(coeff, final_v));
    let mut x_base = scale_(get_speed_coor_x(coeff, other_path_v));
    let mut smooth_length = this_path_x - x_base;
    let mut smooth_length_count = 0.0;
    let mut split_line_speed = 0.0;
    // GCode.cpp:5987-5990 — adjust final_v if length not enough.
    if smooth_length > max_smooth_length {
        smooth_length = max_smooth_length;
        final_v = mapping_speed(coeff, unscale_(x_base + max_smooth_length));
    }

    // GCode.cpp:5991-6000 — insert_speed lambda.
    let insert_speed = |coeff: f64,
                        line_length: f64,
                        pos_x: &mut f64,
                        smooth_length_count: &mut f64,
                        target_v: f64|
     -> f64 {
        *pos_x += line_length;
        let mut pos_x_speed = mapping_speed(coeff, unscale_(*pos_x));
        *smooth_length_count += line_length;
        if pos_x_speed > target_v {
            pos_x_speed = target_v;
        }
        pos_x_speed
    };

    // GCode.cpp:6003-6008 — length_enough lambda.
    let length_enough = |length: f64| -> bool {
        let min = min_step_length();
        !(length < min || length - min < min / 2.0)
    };

    // GCode.cpp:6010
    let mut left_paths: ExtrusionPaths = Vec::new();
    // GCode.cpp:6011 — for (int idx = 0; idx < this_path.size(); idx++)
    let mut idx = 0usize;
    while idx < this_path.len() {
        // GCode.cpp:6014-6016 — polyline error
        if this_path[idx].polyline.points().len() < 2 {
            idx += 1;
            continue;
        }

        // GCode.cpp:6018-6023 — stop and push the rest back.
        if smooth_length_count >= smooth_length {
            left_paths.extend(this_path.drain(idx..));
            *this_path = std::mem::take(&mut left_paths);
            // C++ does `this_path = std::move(left_paths); break;`
            interpolated_paths.append(&mut splited_path);
            return;
        }

        // GCode.cpp:6025-6036 — the path is too short.
        let extrusion_len = this_path[idx].length();
        if !length_enough(extrusion_len) {
            split_line_speed = insert_speed(
                coeff,
                extrusion_len,
                &mut x_base,
                &mut smooth_length_count,
                final_v,
            );
            let mut sp = this_path[idx].clone();
            sp.smooth_speed = split_line_speed;
            splited_path.push(sp);
            // GCode.cpp:6031-6035
            if idx < this_path.len() - 1 {
                idx += 1;
                continue;
            }
            this_path.clear();
            interpolated_paths.append(&mut splited_path);
            return;
        }

        // GCode.cpp:6039-6041 — reverse if slowing down.
        let mut input_polyline = this_path[idx].polyline.clone();
        if !split_from_left {
            input_polyline.reverse();
        }

        // GCode.cpp:6043-6046
        let mut line_start_pt = input_polyline.points()[0];
        let mut line_end_pt = input_polyline.points()[1];
        let mut get_next_line = false;
        let mut end_pt_idx = 1usize;

        // GCode.cpp:6049 — split long extrusion.
        let mut last_point = line_start_pt;
        // GCode.cpp:6050
        while split_line_speed < final_v && end_pt_idx < input_polyline.points().len() {
            // GCode.cpp:6052-6055 — move to next line
            if get_next_line {
                line_start_pt = input_polyline.points()[end_pt_idx - 1];
                line_end_pt = input_polyline.points()[end_pt_idx];
            }
            // GCode.cpp:6057-6058
            let mut cuted_polyline = Polyline::new();
            let line_a = line_start_pt;
            let line_b = line_end_pt;
            let line_len = line_a.distance(&line_b);

            // GCode.cpp:6060
            cuted_polyline.push(line_start_pt);
            // GCode.cpp:6062-6076 — split polyline and set speed.
            if !length_enough(line_len) {
                split_line_speed = insert_speed(
                    coeff,
                    line_len,
                    &mut x_base,
                    &mut smooth_length_count,
                    final_v,
                );
                end_pt_idx += 1;
                get_next_line = true;
                cuted_polyline.push(line_b);
            } else {
                // GCode.cpp:6069-6075 — path is too long, split it.
                let rate = min_step_length() / line_len;
                // insert_p = line.a + (line.b - line.a) * rate
                let insert_p = line_a + point_mul_f64(line_b - line_a, rate);
                split_line_speed = insert_speed(
                    coeff,
                    min_step_length(),
                    &mut x_base,
                    &mut smooth_length_count,
                    final_v,
                );
                line_start_pt = insert_p;
                get_next_line = false;
                cuted_polyline.push(insert_p);
            }
            // GCode.cpp:6078-6080 — reverse back.
            last_point = *cuted_polyline.points().last().unwrap();
            if !split_from_left {
                cuted_polyline.reverse();
            }
            // GCode.cpp:6081-6084
            let mut path_step = path_from(cuted_polyline, &this_path[idx]);
            path_step.smooth_speed = split_line_speed;
            splited_path.push(path_step);
        }

        // GCode.cpp:6087-6091
        if last_point == *input_polyline.points().last().unwrap() {
            if idx == this_path.len() - 1 {
                this_path.clear();
            }
            idx += 1;
            // C++ `continue;` of the for loop. But note when this_path was cleared,
            // the while bound stops it.
            if this_path.is_empty() {
                break;
            }
            continue;
        }

        // GCode.cpp:6092-6094 — split polyline at last_point.
        //
        // C++ `Polyline::split_at(Point &point, ...)` takes `point` by NON-CONST
        // reference and SNAPS it in place to the actual split location on the
        // polyline (Polyline.cpp:223). The "avoid travel" patch below then writes
        // that SAME (snapped) `last_point` into the consumed sub-path's boundary
        // vertex, so the consumed path and the remaining `polyline_left` (which
        // starts/ends at the snapped split vertex) share an identical endpoint —
        // no travel is emitted between them.
        //
        // PARITY-FIX (outer-wall fragmentation): the previous code passed a *copy*
        // (`split_pt`) to split_at_point and then patched with the ORIGINAL,
        // un-snapped `last_point`. When split_at_point snapped the point (the
        // common case — `last_point` is an interpolated insert_p that rarely lands
        // exactly on a stored vertex), the consumed path's patched endpoint
        // (un-snapped) no longer matched `polyline_left`'s endpoint (snapped),
        // leaving a ~1-150 scaled-unit (≈0.001-0.012 mm) gap. extrude_path then
        // emitted a spurious sub-0.1 mm `G1 F60000` travel between the two same-
        // speed paths, fragmenting the outer wall (rust 251 vs native 21 sub-0.1mm
        // outer-wall travels). Mirroring C++, we let split_at_point mutate
        // `last_point` directly and patch with the snapped value.
        let mut p1 = Polyline::new();
        let mut p2 = Polyline::new();
        this_path[idx].polyline.split_at_point(&mut last_point, &mut p1, &mut p2);

        // GCode.cpp:6096-6105
        if split_from_left {
            // update split point to avoid travel path
            if let Some(last) = splited_path.last_mut() {
                let n = last.polyline.points().len();
                if n > 0 {
                    last.polyline.points[n - 1] = last_point;
                }
            }
            let polyline_left = path_from(p2, &this_path[idx]);
            left_paths.push(polyline_left);
        } else {
            if let Some(last) = splited_path.last_mut() {
                if !last.polyline.points().is_empty() {
                    last.polyline.points[0] = last_point;
                }
            }
            let polyline_left = path_from(p1, &this_path[idx]);
            left_paths.push(polyline_left);
        }

        // GCode.cpp:6107
        left_paths.extend(this_path.drain(idx + 1..));
        // remove current consumed path and replace this_path with the remainder
        this_path.truncate(idx); // drop everything from idx onward (already moved tail)
        *this_path = std::mem::take(&mut left_paths);
        // GCode.cpp:6110 — break;
        break;
    }

    // GCode.cpp:6114-6116 — set left path speed.
    if !this_path.is_empty() && final_v != this_path[0].smooth_speed {
        for left in this_path.iter_mut() {
            left.smooth_speed = final_v;
        }
    }

    // GCode.cpp:6118
    interpolated_paths.append(&mut splited_path);
}

/// GCode.cpp:6123-6161 — `std::vector<ExtrusionPaths> GCode::merge_same_speed_paths`.
///
/// `path_speed_fn(path)` is the per-path normal speed (mm/s) — for perimeters
/// this is the overhang-degree-corrected wall speed (GCode::get_path_speed).
pub fn merge_same_speed_paths<F>(paths: &ExtrusionPaths, path_speed_fn: F) -> Vec<ExtrusionPaths>
where
    F: Fn(&ExtrusionPath) -> f64,
{
    // GCode.cpp:6125
    let mut paths_category_by_speed: Vec<ExtrusionPaths> = Vec::new();
    // GCode.cpp:6127
    let mut path_collection: Option<ExtrusionPaths> = None;

    // GCode.cpp:6129
    for path_src in paths.iter() {
        // GCode.cpp:6130-6131 — path.smooth_speed = get_path_speed(path);
        let mut path = path_src.clone();
        path.smooth_speed = path_speed_fn(&path);

        // GCode.cpp:6133-6141 — overhang paths stand alone.
        if path.role == ExtrusionRole::OverhangPerimeter {
            if let Some(c) = path_collection.take() {
                paths_category_by_speed.push(c);
            }
            paths_category_by_speed.push(vec![path]);
            continue;
        }

        // GCode.cpp:6143-6154
        match path_collection.as_mut() {
            None => {
                path_collection = Some(vec![path]);
            }
            Some(c) => {
                if c.last().unwrap().can_merge(&path) {
                    c.push(path);
                } else {
                    let finished = path_collection.take().unwrap();
                    paths_category_by_speed.push(finished);
                    path_collection = Some(vec![path]);
                }
            }
        }
    }

    // GCode.cpp:6157-6158
    if let Some(c) = path_collection.take() {
        paths_category_by_speed.push(c);
    }

    paths_category_by_speed
}

/// GCode.cpp:6163-6257 — `ExtrusionPaths GCode::set_speed_transition`.
pub fn set_speed_transition(coeff: f64, paths: &mut Vec<ExtrusionPaths>) -> ExtrusionPaths {
    // GCode.cpp:6165
    let mut interpolated_paths: ExtrusionPaths = Vec::new();

    // get_path_length helper (GCode.cpp:6188-6190)
    let get_path_length = |p: &ExtrusionPaths| -> f64 { p.iter().map(|x| x.length()).sum() };

    // GCode.cpp:6166 — for (int path_idx = 0; path_idx < paths.size(); path_idx++)
    let mut path_idx = 0usize;
    while path_idx < paths.len() {
        // GCode.cpp:6171-6174 — overhang paths emit as-is.
        if paths[path_idx].first().map(|p| p.role) == Some(ExtrusionRole::OverhangPerimeter) {
            let mut seg = std::mem::take(&mut paths[path_idx]);
            interpolated_paths.append(&mut seg);
            path_idx += 1;
            continue;
        }

        // GCode.cpp:6176-6177
        if paths[path_idx].is_empty() {
            path_idx += 1;
            continue;
        }

        // GCode.cpp:6179
        let smooth_left_path = path_idx > 0
            && !interpolated_paths.is_empty()
            && need_smooth_speed(
                interpolated_paths.last().unwrap(),
                &paths[path_idx][0],
            );
        // GCode.cpp:6181
        let smooth_right_path = path_idx < paths.len() - 1
            && need_smooth_speed(&paths[path_idx + 1][0], &paths[path_idx][0]);

        // GCode.cpp:6183-6186
        if !smooth_left_path && !smooth_right_path {
            let mut seg = std::mem::take(&mut paths[path_idx]);
            interpolated_paths.append(&mut seg);
            path_idx += 1;
            continue;
        }

        // GCode.cpp:6192-6195
        let mut max_smooth_path_length = get_path_length(&paths[path_idx]);
        if smooth_right_path && smooth_left_path {
            max_smooth_path_length /= 2.0;
        }

        // GCode.cpp:6198-6207 — smooth left.
        if smooth_left_path {
            let other_v = interpolated_paths.last().unwrap().smooth_speed;
            let final_v = paths[path_idx][0].smooth_speed;
            let mut seg = std::mem::take(&mut paths[path_idx]);
            split_and_mapping_speed(
                coeff,
                other_v,
                final_v,
                &mut seg,
                max_smooth_path_length,
                &mut interpolated_paths,
                true,
            );
            paths[path_idx] = seg;
            // GCode.cpp:6202-6204
            if paths[path_idx].is_empty() {
                path_idx += 1;
                continue;
            }
            // GCode.cpp:6206
            max_smooth_path_length = get_path_length(&paths[path_idx]);
        }

        // GCode.cpp:6209-6212
        if !smooth_right_path {
            let mut seg = std::mem::take(&mut paths[path_idx]);
            interpolated_paths.append(&mut seg);
            path_idx += 1;
            continue;
        }

        // GCode.cpp:6216-6229 — smooth right; build smoothing window of indices.
        let _ = max_smooth_path_length;
        let mut window: Vec<usize> = vec![path_idx];
        let mut right_end = path_idx + 1;
        while right_end < paths.len() - 1 {
            if paths[right_end].first().map(|p| p.role) == Some(ExtrusionRole::OverhangPerimeter) {
                break;
            }
            window.push(right_end);
            if !need_smooth_speed(&paths[right_end + 1][0], &paths[right_end][0]) {
                break;
            }
            right_end += 1;
        }

        // GCode.cpp:6231-6232
        path_idx += window.len() - 1;
        let mut prev_speed = paths[path_idx + 1][0].smooth_speed;

        // GCode.cpp:5866-5868 — reverse window order and reverse the path-order
        // within each window entry.
        //
        // C++:  std::reverse(paths_cpoy.begin(), paths_cpoy.end());
        //       for (ExtrusionPaths *paths_temp : paths_cpoy)
        //           std::reverse(paths_temp->begin(), paths_temp->end());
        //
        // `paths_temp` is an `ExtrusionPaths` (a `vector<ExtrusionPath>`), so the
        // inner `std::reverse` reverses the ORDER OF THE PATH ELEMENTS — it does
        // NOT reverse the points inside each ExtrusionPath's polyline. The
        // direction reversal of the geometry is done internally by
        // split_and_mapping_speed (split_from_left=false: it reverses the input
        // polyline, cuts, then reverses the cut polyline back). A previous port
        // additionally called `p.reverse()` on every ExtrusionPath here, which
        // double-flipped each sub-path's polyline and left the smoothed output
        // non-contiguous (last_point(path_i) != first_point(path_{i+1})). That
        // made GCode::_extrude emit a travel before every smoothed sub-path,
        // shattering each overhang-graded outer-wall loop into dozens of
        // travel-separated runs (~5400 spurious outer-wall travels). Reversing
        // only the element order (as C++ does) keeps the loop continuous.
        window.reverse();
        for &wi in &window {
            paths[wi].reverse();
        }

        // GCode.cpp:6238-6249 — smooth right path.
        let mut transition_right: ExtrusionPaths = Vec::new();
        for (k, &wi) in window.iter().enumerate() {
            if k != 0 {
                prev_speed = transition_right.last().unwrap().smooth_speed;
            }
            let final_v = paths[wi][0].smooth_speed;
            let max_len = get_path_length(&paths[wi]);
            let mut seg = std::mem::take(&mut paths[wi]);
            split_and_mapping_speed(
                coeff,
                prev_speed,
                final_v,
                &mut seg,
                max_len,
                &mut transition_right,
                false,
            );
            // GCode.cpp:6247 — append remaining seg.
            transition_right.append(&mut seg);
            paths[wi] = Vec::new();
        }
        // GCode.cpp:6249
        transition_right.reverse();

        // GCode.cpp:6252
        interpolated_paths.append(&mut transition_right);

        path_idx += 1;
    }

    interpolated_paths
}

/// GCode.cpp:6259-6272 — `void GCode::smooth_speed_discontinuity_area(ExtrusionPaths &paths)`.
///
/// `coeff` is `m_smooth_coefficient` (filament_velocity_adaptation_factor *
/// smooth_coefficient). `path_speed_fn` is the per-path normal speed (mm/s).
pub fn smooth_speed_discontinuity_area<F>(coeff: f64, paths: &mut ExtrusionPaths, path_speed_fn: F)
where
    F: Fn(&ExtrusionPath) -> f64,
{
    // GCode.cpp:6261-6262
    if paths.len() <= 1 || coeff == 0.0 {
        return;
    }
    // GCode.cpp:6266 — step 1 merge same speed paths.
    let mut prepare_paths = merge_same_speed_paths(paths, path_speed_fn);
    // GCode.cpp:6269-6271 — step 2 split path.
    *paths = set_speed_transition(coeff, &mut prepare_paths);
}
