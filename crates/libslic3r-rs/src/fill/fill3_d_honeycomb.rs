//! Faithful 1:1 line-by-line port of BambuStudio
//! `src/libslic3r/Fill/Fill3DHoneycomb.cpp` (+ `.hpp`).
//!
//! C++ Reference:
//! - Fill/Fill3DHoneycomb.hpp
//! - Fill/Fill3DHoneycomb.cpp
//!
//! `coord_t` -> `i64` (`Coord`), `coordf_t` -> `f64` (`CoordF`).
//!
//! Modelling notes (see also the per-line comments):
//! - The C++ class `Fill3DHoneycomb` derives from `Fill`. There is no shared `Fill`
//!   base struct in the Rust port (the fill module is otherwise procedural), so the
//!   base members this fill reads — `angle`, `spacing`, `z` — are carried directly on
//!   this struct, exactly as the sibling `FillPlanePath` / `FillConcentric` ports do.
//! - `connect_infill` (the boundary-graph variant) maps to the crate-level
//!   `super::connect_infill_expolygon`, which is the project's simplified port of
//!   `Fill::connect_infill`. This matches how `FillPlanePath::_fill_surface_single`
//!   wires connection up.
//! - C++ `triWave` computes the fractional part with a `float t` (single precision)
//!   and `(int)t` truncation toward zero. This is reproduced exactly with `f32` and
//!   `as i32 as f32` (NOT `floor`), since the f32 quantisation feeds directly into
//!   scaled integer point coordinates.

// Fill3DHoneycomb.cpp:1-5
// #include "../ClipperUtils.hpp"
// #include "../ShortestPath.hpp"
// #include "../Surface.hpp"
// #include "Fill3DHoneycomb.hpp"
use crate::clipper_utils::intersection_pl;
use crate::geometry::{ExPolygon, Point, Polyline, Polylines};
use crate::shortest_path::chain_polylines;
use crate::{scale, Coord, CoordF};

use super::{connect_infill_expolygon, multiline_fill, FillParams};

// Fill3DHoneycomb.cpp:7
// namespace Slic3r {

/// Fill3DHoneycomb.hpp:12-29  class Fill3DHoneycomb : public Fill
///
/// Carries the `Fill` base members this fill reads (`angle`, `spacing`, `z`).
#[derive(Debug, Clone, Default)]
pub struct Fill3DHoneycomb {
    // Fill base (FillBase.hpp): `float angle;` — infill rotation, in radians.
    pub angle: f32,
    // Fill base (FillBase.hpp): `coordf_t spacing;` — in unscaled coordinates.
    pub spacing: f64,
    // Fill base (FillBase.hpp): `coordf_t z;` — current slice Z, in unscaled coordinates.
    pub z: f64,
}

impl Fill3DHoneycomb {
    /// Fill3DHoneycomb.hpp:18-19  bool use_bridge_flow() const override { return true; }
    // require bridge flow since most of this pattern hangs in air
    pub fn use_bridge_flow(&self) -> bool {
        true
    }

    /// Fill3DHoneycomb.hpp:20  bool is_self_crossing() override { return false; }
    pub fn is_self_crossing(&self) -> bool {
        false
    }
}

// Fill3DHoneycomb.cpp:9-12
// sign function
// template <typename T> int sgn(T val) {
//   return (T(0) < val) - (val < T(0));
// }
fn sgn(val: CoordF) -> i32 {
    (0. < val) as i32 - (val < 0.) as i32
}

/*
Creates a contiguous sequence of points at a specified height that make
up a horizontal slice of the edges of a space filling truncated
octahedron tesselation. The octahedrons are oriented so that the
square faces are in the horizontal plane with edges parallel to the X
and Y axes.

Credits: David Eccles (gringer).
*/

// Fill3DHoneycomb.cpp:24-32
// triangular wave function
// this has period (gridSize * 2), and amplitude (gridSize / 2),
// with triWave(pos = 0) = 0
fn tri_wave(pos: CoordF, grid_size: CoordF) -> CoordF {
    // Fill3DHoneycomb.cpp:29
    let mut t = ((pos / (grid_size * 2.)) + 0.25) as f32; // convert relative to grid size
    // Fill3DHoneycomb.cpp:30
    t = t - (t as i32) as f32; // extract fractional part
    // Fill3DHoneycomb.cpp:31
    ((1. - ((t * 8. - 4.) as CoordF).abs()) * (grid_size / 4.)) + (grid_size / 4.)
}

// Fill3DHoneycomb.cpp:34-46
// truncated octagonal waveform, with period and offset
// as per the triangular wave function. The Z position adjusts
// the maximum offset [between -(gridSize / 4) and (gridSize / 4)], with a
// period of (gridSize * 2) and troctWave(Zpos = 0) = 0
fn troct_wave(pos: CoordF, grid_size: CoordF, z_pos: CoordF) -> CoordF {
    // Fill3DHoneycomb.cpp:40
    let z_cycle: CoordF = tri_wave(z_pos, grid_size);
    // Fill3DHoneycomb.cpp:41
    let perp_offset: CoordF = z_cycle / 2.;
    // Fill3DHoneycomb.cpp:42
    let y: CoordF = tri_wave(pos, grid_size);
    // Fill3DHoneycomb.cpp:43-45
    if y.abs() > perp_offset.abs() {
        sgn(y) as CoordF * perp_offset
    } else {
        y * sgn(perp_offset) as CoordF
    }
}

// Fill3DHoneycomb.cpp:48-61
// Identify the important points of curve change within a truncated
// octahedron wave (as waveform fraction t):
// 1. Start of wave (always 0.0)
// 2. Transition to upper "horizontal" part
// 3. Transition from upper "horizontal" part
// 4. Transition to lower "horizontal" part
// 5. Transition from lower "horizontal" part
/*    o---o
 *   /     \
 * o/       \
 *           \       /
 *            \     /
 *             o---o
 */
fn get_critical_points(z_pos: CoordF, grid_size: CoordF) -> Vec<CoordF> {
    // Fill3DHoneycomb.cpp:64
    let mut res: Vec<CoordF> = vec![0.];
    // Fill3DHoneycomb.cpp:65
    let perp_offset: CoordF = (tri_wave(z_pos, grid_size) / 2.).abs();

    // Fill3DHoneycomb.cpp:67
    let normalised_offset: CoordF = perp_offset / grid_size;
    // // for debugging: just generate evenly-distributed points
    // for(coordf_t i = 0; i < 2; i += 0.05){
    //   res.push_back(gridSize * i);
    // }
    // note: 0 == straight line
    // Fill3DHoneycomb.cpp:73
    if normalised_offset > 0. {
        // Fill3DHoneycomb.cpp:74
        res.push(grid_size * (0. + normalised_offset));
        // Fill3DHoneycomb.cpp:75
        res.push(grid_size * (1. - normalised_offset));
        // Fill3DHoneycomb.cpp:76
        res.push(grid_size * (1. + normalised_offset));
        // Fill3DHoneycomb.cpp:77
        res.push(grid_size * (2. - normalised_offset));
    }
    // Fill3DHoneycomb.cpp:79
    res
}

// Fill3DHoneycomb.cpp:82-98
// Generate an array of points that are in the same direction as the
// basic printing line (i.e. Y points for columns, X points for rows)
// Note: a negative offset only causes a change in the perpendicular
// direction
fn colinear_points(
    _z_pos: CoordF,
    grid_size: CoordF,
    crit_points: &[CoordF],
    base_location: usize,
    grid_length: usize,
) -> Vec<CoordF> {
    // Fill3DHoneycomb.cpp:89
    let mut points: Vec<CoordF> = Vec::new();
    // Fill3DHoneycomb.cpp:90
    points.push(base_location as CoordF);
    // Fill3DHoneycomb.cpp:91
    let mut c_loc: CoordF = base_location as CoordF;
    while c_loc < grid_length as CoordF {
        // Fill3DHoneycomb.cpp:92
        for &cp in crit_points {
            // Fill3DHoneycomb.cpp:93
            points.push(base_location as CoordF + c_loc + cp);
        }
        c_loc += grid_size * 2.;
    }
    // Fill3DHoneycomb.cpp:96
    points.push(grid_length as CoordF);
    // Fill3DHoneycomb.cpp:97
    points
}

// Fill3DHoneycomb.cpp:100-116
// Generate an array of points for the dimension that is perpendicular to
// the basic printing line (i.e. X points for columns, Y points for rows)
#[allow(clippy::too_many_arguments)]
fn perpend_points(
    z_pos: CoordF,
    grid_size: CoordF,
    crit_points: &[CoordF],
    base_location: usize,
    grid_length: usize,
    offset_base: usize,
    perp_dir: CoordF,
) -> Vec<CoordF> {
    // Fill3DHoneycomb.cpp:106
    let mut points: Vec<CoordF> = Vec::new();
    // Fill3DHoneycomb.cpp:107
    points.push(offset_base as CoordF);
    // Fill3DHoneycomb.cpp:108
    let mut c_loc: CoordF = base_location as CoordF;
    while c_loc < grid_length as CoordF {
        // Fill3DHoneycomb.cpp:109
        for &cp in crit_points {
            // Fill3DHoneycomb.cpp:110
            let offset: CoordF = troct_wave(cp, grid_size, z_pos);
            // Fill3DHoneycomb.cpp:111
            points.push(offset_base as CoordF + (offset * perp_dir));
        }
        c_loc += grid_size * 2.;
    }
    // Fill3DHoneycomb.cpp:114
    points.push(offset_base as CoordF);
    // Fill3DHoneycomb.cpp:115
    points
}

// Fill3DHoneycomb.cpp:118-126
// static inline Pointfs zip(const std::vector<coordf_t> &x, const std::vector<coordf_t> &y)
fn zip(x: &[CoordF], y: &[CoordF]) -> Vec<(CoordF, CoordF)> {
    // Fill3DHoneycomb.cpp:120
    debug_assert_eq!(x.len(), y.len());
    // Fill3DHoneycomb.cpp:121-122
    let mut out: Vec<(CoordF, CoordF)> = Vec::with_capacity(x.len());
    // Fill3DHoneycomb.cpp:123-124
    for i in 0..x.len() {
        out.push((x[i], y[i]));
    }
    // Fill3DHoneycomb.cpp:125
    out
}

// Fill3DHoneycomb.cpp:128-160
// Generate a set of curves (array of array of 2d points) that describe a
// horizontal slice of a truncated regular octahedron.
fn make_actual_grid(
    z_pos: CoordF,
    grid_size: CoordF,
    bounds_x: usize,
    bounds_y: usize,
) -> Vec<Vec<(CoordF, CoordF)>> {
    // Fill3DHoneycomb.cpp:132
    let mut points: Vec<Vec<(CoordF, CoordF)>> = Vec::new();
    // Fill3DHoneycomb.cpp:133
    let crit_points: Vec<CoordF> = get_critical_points(z_pos, grid_size);
    // Fill3DHoneycomb.cpp:134
    // C++ `fmod` is the truncated remainder (sign of dividend); Rust's `%` on f64
    // matches `fmod` exactly (NOT `rem_euclid`, which is always non-negative).
    let z_cycle: CoordF = (z_pos + grid_size / 2.) % (grid_size * 2.) / (grid_size * 2.);
    // Fill3DHoneycomb.cpp:135
    let print_vert: bool = z_cycle < 0.5;
    // Fill3DHoneycomb.cpp:136
    if print_vert {
        // Fill3DHoneycomb.cpp:137
        let mut perp_dir: i32 = -1;
        // Fill3DHoneycomb.cpp:138
        let mut x: CoordF = 0.;
        while x <= bounds_x as CoordF {
            // Fill3DHoneycomb.cpp:139-140
            points.push(Vec::new());
            let new_points = points.last_mut().unwrap();
            // Fill3DHoneycomb.cpp:141-143
            *new_points = zip(
                &perpend_points(
                    z_pos,
                    grid_size,
                    &crit_points,
                    0,
                    bounds_y,
                    x as usize,
                    perp_dir as CoordF,
                ),
                &colinear_points(z_pos, grid_size, &crit_points, 0, bounds_y),
            );
            // Fill3DHoneycomb.cpp:144-145
            if perp_dir == 1 {
                new_points.reverse();
            }
            // Fill3DHoneycomb.cpp:138 (loop increment)
            x += grid_size;
            perp_dir *= -1;
        }
    } else {
        // Fill3DHoneycomb.cpp:148
        let mut perp_dir: i32 = 1;
        // Fill3DHoneycomb.cpp:149
        let mut y: CoordF = grid_size;
        while y <= bounds_y as CoordF {
            // Fill3DHoneycomb.cpp:150-151
            points.push(Vec::new());
            let new_points = points.last_mut().unwrap();
            // Fill3DHoneycomb.cpp:152-154
            *new_points = zip(
                &colinear_points(z_pos, grid_size, &crit_points, 0, bounds_x),
                &perpend_points(
                    z_pos,
                    grid_size,
                    &crit_points,
                    0,
                    bounds_x,
                    y as usize,
                    perp_dir as CoordF,
                ),
            );
            // Fill3DHoneycomb.cpp:155-156
            if perp_dir == -1 {
                new_points.reverse();
            }
            // Fill3DHoneycomb.cpp:149 (loop increment)
            y += grid_size;
            perp_dir *= -1;
        }
    }
    // Fill3DHoneycomb.cpp:159
    points
}

// Fill3DHoneycomb.cpp:162-179
// Generate a set of curves (array of array of 2d points) that describe a
// horizontal slice of a truncated regular octahedron with a specified
// grid square size.
// gridWidth and gridHeight define the width and height of the bounding box respectively
fn make_grid(
    z: CoordF,
    grid_size: CoordF,
    bound_width: CoordF,
    bound_height: CoordF,
    _fill_evenly: bool,
) -> Polylines {
    // Fill3DHoneycomb.cpp:168
    let polylines: Vec<Vec<(CoordF, CoordF)>> =
        make_actual_grid(z, grid_size, bound_width as usize, bound_height as usize);
    // Fill3DHoneycomb.cpp:169-170
    let mut result: Polylines = Vec::with_capacity(polylines.len());
    // Fill3DHoneycomb.cpp:171-177
    for it_polylines in polylines.iter() {
        // Fill3DHoneycomb.cpp:173-174
        result.push(Polyline::default());
        let polyline = result.last_mut().unwrap();
        // Fill3DHoneycomb.cpp:175-176
        for it in it_polylines.iter() {
            polyline
                .points
                .push(Point::new(it.0 as Coord, it.1 as Coord));
        }
    }
    // Fill3DHoneycomb.cpp:178
    result
}

// Fill3DHoneycomb.cpp:181-188
// FillParams has the following useful information:
// density <0 .. 1>  [proportion of space to fill]
// anchor_length     [???]
// anchor_length_max [???]
// dont_connect()    [avoid connect lines]
// dont_adjust       [avoid filling space evenly]
// monotonic         [fill strictly left to right]
// complete          [complete each loop]

impl Fill3DHoneycomb {
    // Fill3DHoneycomb.cpp:190-298
    // void Fill3DHoneycomb::_fill_surface_single(
    //     const FillParams                &params,
    //     unsigned int                     thickness_layers,
    //     const std::pair<float, Point>   &direction,
    //     ExPolygon                        expolygon,
    //     Polylines                       &polylines_out)
    pub fn _fill_surface_single(
        &mut self,
        params: &FillParams,
        thickness_layers: u32,
        _direction: &(f32, Point),
        mut expolygon: ExPolygon,
        polylines_out: &mut Polylines,
    ) {
        // no rotation is supported for this infill pattern
        // BBL: add support for rotation
        // Fill3DHoneycomb.cpp:199
        let infill_angle: f32 = self.angle;
        // Fill3DHoneycomb.cpp:200
        if infill_angle.abs() as f64 >= EPSILON {
            expolygon.rotate(-infill_angle as CoordF);
        }
        // Fill3DHoneycomb.cpp:201
        let mut bb = expolygon.contour.bounding_box();

        // Note: with equally-scaled X/Y/Z, the pattern will create a vertically-stretched
        // truncated octahedron; so Z is pre-adjusted first by scaling by sqrt(2)
        // Fill3DHoneycomb.cpp:205
        let mut z_scale: CoordF = 2.0_f64.sqrt();

        // adjustment to account for the additional distance of octagram curves
        // note: this only strictly applies for a rectangular area where the total
        //       Z travel distance is a multiple of the spacing... but it should
        //       be at least better than the prevous estimate which assumed straight
        //       lines
        // = 4 * integrate(func=4*x(sqrt(2) - 1) + 1, from=0, to=0.25)
        // = (sqrt(2) + 1) / 2 [... I think]
        // make a first guess at the preferred grid Size
        // Fill3DHoneycomb.cpp:215
        let mut grid_size: CoordF = scale(self.spacing) as CoordF * ((z_scale + 1.) / 2.)
            * params.multiline as CoordF
            / params.density as CoordF;

        // This density calculation is incorrect for many values > 25%, possibly
        // due to quantisation error, so this value is used as a first guess, then the
        // Z scale is adjusted to make the layer patterns consistent / symmetric
        // This means that the resultant infill won't be an ideal truncated octahedron,
        // but it should look better than the equivalent quantised version

        // Fill3DHoneycomb.cpp:223
        let layer_height: CoordF = scale(thickness_layers as CoordF) as CoordF;
        // ceiling to an integer value of layers per Z
        // (with a little nudge in case it's close to perfect)
        // Fill3DHoneycomb.cpp:226
        let mut layers_per_module: CoordF =
            ((grid_size * 2.) / (z_scale * layer_height) + 0.05).floor();
        // Fill3DHoneycomb.cpp:227
        if params.density > 0.42 {
            // exact layer pattern for >42% density
            // Fill3DHoneycomb.cpp:228
            layers_per_module = 2.;
            // re-adjust the grid size for a partial octahedral path
            // (scale of 1.1 guessed based on modeling)
            // Fill3DHoneycomb.cpp:231
            grid_size =
                scale(self.spacing) as CoordF * 1.1 * params.multiline as CoordF / params.density as CoordF;
            // re-adjust zScale to make layering consistent
            // Fill3DHoneycomb.cpp:233
            z_scale = (grid_size * 2.) / (layers_per_module * layer_height);
        } else {
            // Fill3DHoneycomb.cpp:235-237
            if layers_per_module < 2. {
                layers_per_module = 2.;
            }
            // re-adjust zScale to make layering consistent
            // Fill3DHoneycomb.cpp:239
            z_scale = (grid_size * 2.) / (layers_per_module * layer_height);
            // re-adjust the grid size to account for the new zScale
            // Fill3DHoneycomb.cpp:241
            grid_size = scale(self.spacing) as CoordF * ((z_scale + 1.) / 2.)
                * params.multiline as CoordF
                / params.density as CoordF;
            // re-calculate layersPerModule and zScale
            // Fill3DHoneycomb.cpp:243
            layers_per_module = ((grid_size * 2.) / (z_scale * layer_height) + 0.05).floor();
            // Fill3DHoneycomb.cpp:244-246
            if layers_per_module < 2. {
                layers_per_module = 2.;
            }
            // Fill3DHoneycomb.cpp:247
            z_scale = (grid_size * 2.) / (layers_per_module * layer_height);
        }

        // align bounding box to a multiple of our honeycomb grid module
        // (a module is 2*$gridSize since one $gridSize half-module is
        // growing while the other $gridSize half-module is shrinking)
        // Fill3DHoneycomb.cpp:253
        bb.merge_point(crate::geometry::align_to_grid_point(
            bb.min,
            Point::new((grid_size * 4.) as Coord, (grid_size * 4.) as Coord),
        ));

        // generate pattern
        // Fill3DHoneycomb.cpp:256-262
        let mut polylines: Polylines = make_grid(
            scale(self.z) as CoordF * z_scale,
            grid_size,
            bb.size().x() as CoordF,
            bb.size().y() as CoordF,
            !params.dont_adjust,
        );

        // move pattern in place
        // Fill3DHoneycomb.cpp:265-268
        for pl in polylines.iter_mut() {
            pl.translate(bb.min);
            pl.simplify(5. * self.spacing);
        }
        // Apply multiline offset if needed
        // Fill3DHoneycomb.cpp:270
        multiline_fill(&mut polylines, params, self.spacing as f32);
        // clip pattern to boundaries, chain the clipped polylines
        // Fill3DHoneycomb.cpp:272
        polylines = intersection_pl(&polylines, std::slice::from_ref(&expolygon));

        // Fill3DHoneycomb.cpp:274
        if !polylines.is_empty() {
            // Remove very small bits, but be careful to not remove infill lines connecting thin walls!
            // The infill perimeter lines should be separated by around a single infill line width.
            // Fill3DHoneycomb.cpp:277
            let minlength: f64 = scale(0.8 * self.spacing) as f64;
            // Fill3DHoneycomb.cpp:278-280
            polylines.retain(|pl| pl.length() >= minlength);
        }

        // copy from fliplines
        // Fill3DHoneycomb.cpp:284
        if !polylines.is_empty() {
            // Fill3DHoneycomb.cpp:285  only rotate what belongs to us.
            let infill_start_idx: usize = polylines_out.len();
            // connect lines
            // Fill3DHoneycomb.cpp:287
            if params.dont_connect() || polylines.len() <= 1 {
                // Fill3DHoneycomb.cpp:288
                polylines_out.extend(chain_polylines(std::mem::take(&mut polylines), None));
            } else {
                // Fill3DHoneycomb.cpp:290
                connect_infill_expolygon(polylines, &expolygon, self.spacing, params, polylines_out);
            }

            // rotate back
            // Fill3DHoneycomb.cpp:293
            if infill_angle.abs() as f64 >= EPSILON {
                // Fill3DHoneycomb.cpp:294-295
                for it in polylines_out.iter_mut().skip(infill_start_idx) {
                    it.rotate(infill_angle as CoordF);
                }
            }
        }
    }
}

// Fill3DHoneycomb.cpp:300
// } // namespace Slic3r

/// libslic3r.h:52 — `static constexpr double EPSILON = 1e-4;`
const EPSILON: f64 = 1e-4;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Polygon;

    fn make_square_boundary(size_mm: CoordF) -> ExPolygon {
        let size = scale(size_mm);
        let contour = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(size, 0),
            Point::new(size, size),
            Point::new(0, size),
        ]);
        ExPolygon::new(contour)
    }

    #[test]
    fn test_sgn() {
        assert_eq!(sgn(5.0), 1);
        assert_eq!(sgn(-5.0), -1);
        assert_eq!(sgn(0.0), 0);
    }

    #[test]
    fn test_tri_wave() {
        let grid_size = 10.0;
        // triWave(pos = 0) is gridSize/4 (the +gridSize/4 baseline at the midpoint).
        let v0 = tri_wave(0.0, grid_size);
        // period of (gridSize * 2)
        let v_period = tri_wave(grid_size * 2.0, grid_size);
        assert!((v0 - v_period).abs() < 1e-6);
        // bounded within [0, gridSize/2]
        assert!(v0 >= 0.0 && v0 <= grid_size / 2.0 + 1e-9);
    }

    #[test]
    fn test_get_critical_points() {
        let points = get_critical_points(0.0, 10.0);
        assert!(!points.is_empty());
        assert!((points[0] - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_make_actual_grid() {
        let grid = make_actual_grid(0.0, 10.0, 100, 100);
        assert!(!grid.is_empty());
        for pl in &grid {
            assert!(pl.len() >= 2);
        }
    }

    #[test]
    fn test_fill_surface_single_runs() {
        let boundary = make_square_boundary(50.0);
        let mut fill = Fill3DHoneycomb {
            angle: 0.0,
            spacing: 0.45,
            z: 0.2,
        };
        let mut params = FillParams::default();
        params.density = 0.3;
        let mut out: Polylines = Vec::new();
        fill._fill_surface_single(&params, 0, &(0.0, Point::new(0, 0)), boundary, &mut out);
        // Just verify no crash; output count is geometry-dependent.
        let _ = out.len();
    }

    #[test]
    fn test_use_bridge_flow() {
        let fill = Fill3DHoneycomb::default();
        assert!(fill.use_bridge_flow());
        assert!(!fill.is_self_crossing());
    }
}
