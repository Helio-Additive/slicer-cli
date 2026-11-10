//! 1:1 line-by-line port of `Geometry.cpp` (+ `Geometry.hpp`).
//!
//! C++ Reference:
//! - `src/libslic3r/Geometry.cpp`
//! - `src/libslic3r/Geometry.hpp`
//!
//! This file mirrors `Slic3r::Geometry`. The free functions that were already
//! ported live in the parent module (`geometry/mod.rs`) — see
//! `directions_parallel`, `directions_perpendicular`, `rad2deg_dir`, `linint`,
//! `segments_intersect`, `liang_barsky_line_clipping`. This file ports the
//! remaining tractable symbols: the header math helpers and the full
//! `Transformation` / `TransformationSVD` family.
//!
//! coord_t -> i64, coordf_t -> f64. Eigen types map to nalgebra:
//! `Vec3d` == `Vector3<f64>`, `Matrix3d` == `Matrix3<f64>`,
//! `Transform3d` == `Eigen::Transform<double,3,Affine>` ~ nalgebra `Matrix4<f64>`
//! (we store the affine transform as a homogeneous 4x4 matrix, matching Eigen's
//! `.matrix()`).

// Geometry.cpp:1-21 — includes
use crate::geometry::{ExPolygons, Point, Polygon, Polygons};
use crate::libslic3r::EPSILON;
use nalgebra::{Matrix3, Matrix4, UnitQuaternion, Vector3};

// Geometry.hpp:24-28
// Generic result of an orientation predicate.
/// Geometry.hpp:23-28 `enum Orientation`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum Orientation {
    // Geometry.hpp:25
    OrientationCcw = 1,
    // Geometry.hpp:26
    OrientationCw = -1,
    // Geometry.hpp:27
    OrientationColinear = 0,
}

// Geometry.hpp:30-44
// Return orientation of the three points (clockwise, counter-clockwise, colinear)
// The predicate is exact for the coord_t type, using 64bit signed integers for the temporaries.
// which means, the coord_t types must not have some of the topmost bits utilized.
// As the points are limited to 30 bits + signum,
// the temporaries u, v, w are limited to 61 bits + signum,
// and d is limited to 63 bits + signum and we are good.
//
// NOTE: in the C++ codebase coord_t is a 32-bit int and the predicate widens to
// int64_t. Here coord_t == i64, so to retain exactness we widen to i128.
/// Geometry.hpp:36-44 `orient`
#[inline]
pub fn orient(a: &Point, b: &Point, c: &Point) -> Orientation {
    // Geometry.hpp:39
    let u = b.x as i128 * c.y as i128 - b.y as i128 * c.x as i128;
    // Geometry.hpp:40
    let v = a.x as i128 * c.y as i128 - a.y as i128 * c.x as i128;
    // Geometry.hpp:41
    let w = a.x as i128 * b.y as i128 - a.y as i128 * b.x as i128;
    // Geometry.hpp:42
    let d = u - v + w;
    // Geometry.hpp:43
    if d > 0 {
        Orientation::OrientationCcw
    } else if d == 0 {
        Orientation::OrientationColinear
    } else {
        Orientation::OrientationCw
    }
}

// Geometry.hpp:46-73
// Return orientation of the polygon by checking orientation of the left bottom corner of the polygon
// using exact arithmetics. The input polygon must not contain duplicate points
// (or at least the left bottom corner point must not have duplicates).
/// Geometry.hpp:49-73 `is_ccw`
pub fn is_ccw(poly: &Polygon) -> bool {
    // The polygon shall be at least a triangle.
    // Geometry.hpp:52-54
    debug_assert!(poly.points.len() >= 3);
    if poly.points.len() < 3 {
        return true;
    }

    // 1) Find the lowest lexicographical point.
    // Geometry.hpp:57-63
    let mut imin: usize = 0;
    for i in 1..poly.points.len() {
        let pmin = &poly.points[imin];
        let p = &poly.points[i];
        if p.x < pmin.x || (p.x == pmin.x && p.y < pmin.y) {
            imin = i;
        }
    }

    // 2) Detect the orientation of the corner imin.
    // Geometry.hpp:66-67
    let i_prev = (if imin == 0 { poly.points.len() } else { imin }) - 1;
    let i_next = if imin + 1 == poly.points.len() { 0 } else { imin + 1 };
    // Geometry.hpp:68
    let o = orient(&poly.points[i_prev], &poly.points[imin], &poly.points[i_next]);
    // The lowest bottom point must not be collinear if the polygon does not contain duplicate points
    // or overlapping segments.
    // Geometry.hpp:71
    debug_assert!(o != Orientation::OrientationColinear);
    // Geometry.hpp:72
    o == Orientation::OrientationCcw
}

// Geometry.hpp:75-84
/// Geometry.hpp:75-84 `ray_ray_intersection`
#[inline]
pub fn ray_ray_intersection(p1: &Vec2d, v1: &Vec2d, p2: &Vec2d, v2: &Vec2d, res: &mut Vec2d) -> bool {
    // Geometry.hpp:77
    let denom = v1.x * v2.y - v2.x * v1.y;
    // Geometry.hpp:78-79
    if denom.abs() < EPSILON {
        return false;
    }
    // Geometry.hpp:80
    let t = (v2.x * (p1.y - p2.y) - v2.y * (p1.x - p2.x)) / denom;
    // Geometry.hpp:81-82
    res.x = p1.x + t * v1.x;
    res.y = p1.y + t * v1.y;
    // Geometry.hpp:83
    true
}

// Geometry.hpp:86-115
/// Geometry.hpp:86-115 `segment_segment_intersection`
#[inline]
pub fn segment_segment_intersection(
    p1: &Vec2d,
    v1: &Vec2d,
    p2: &Vec2d,
    v2: &Vec2d,
    res: &mut Vec2d,
) -> bool {
    // Geometry.hpp:88
    let mut denom = v1.x * v2.y - v2.x * v1.y;
    // Geometry.hpp:89-91
    if denom.abs() < EPSILON {
        // Lines are collinear.
        return false;
    }
    // Geometry.hpp:92-93
    let s12_x = p1.x - p2.x;
    let s12_y = p1.y - p2.y;
    // Geometry.hpp:94
    let mut s_numer = v1.x * s12_y - v1.y * s12_x;
    // Geometry.hpp:95
    let mut denom_is_positive = false;
    // Geometry.hpp:96-100
    if denom < 0. {
        denom_is_positive = true;
        denom = -denom;
        s_numer = -s_numer;
    }
    // Geometry.hpp:101-103
    if s_numer < 0. {
        // Intersection outside of the 1st segment.
        return false;
    }
    // Geometry.hpp:104
    let mut t_numer = v2.x * s12_y - v2.y * s12_x;
    // Geometry.hpp:105-106
    if !denom_is_positive {
        t_numer = -t_numer;
    }
    // Geometry.hpp:107-109
    if t_numer < 0. || s_numer > denom || t_numer > denom {
        // Intersection outside of the 1st or 2nd segment.
        return false;
    }
    // Intersection inside both of the segments.
    // Geometry.hpp:111
    let t = t_numer / denom;
    // Geometry.hpp:112-113
    res.x = p1.x + t * v1.x;
    res.y = p1.y + t * v1.y;
    // Geometry.hpp:114
    true
}

// Geometry.hpp:169-175
/// Geometry.hpp:169-175 `foot_pt` (generic, here specialized to 2D `Vec2d`)
#[inline]
pub fn foot_pt_dir(line_pt: &Vec2d, line_dir: &Vec2d, pt: &Vec2d) -> Vec2d {
    // Geometry.hpp:171
    let v = Vec2d::new(pt.x - line_pt.x, pt.y - line_pt.y);
    // Geometry.hpp:172 — squaredNorm
    let l2 = line_dir.x * line_dir.x + line_dir.y * line_dir.y;
    // Geometry.hpp:173
    let t = if l2 == 0. { 0. } else { (v.x * line_dir.x + v.y * line_dir.y) / l2 };
    // Geometry.hpp:174
    Vec2d::new(line_pt.x + line_dir.x * t, line_pt.y + line_dir.y * t)
}

// Geometry.hpp:293
/// Geometry.hpp:293 `rad2deg`
#[inline]
pub fn rad2deg(angle: f64) -> f64 {
    180.0 * angle / std::f64::consts::PI
}

// Geometry.hpp:295
/// Geometry.hpp:295 `deg2rad`
#[inline]
pub fn deg2rad(angle: f64) -> f64 {
    std::f64::consts::PI * angle / 180.0
}

// Geometry.hpp:296-309
/// Geometry.hpp:296-309 `angle_to_0_2PI`
#[inline]
pub fn angle_to_0_2pi(mut angle: f64) -> f64 {
    // Geometry.hpp:298
    const TWO_PI: f64 = 2.0 * std::f64::consts::PI;
    // Geometry.hpp:299-302
    while angle < 0.0 {
        angle += TWO_PI;
    }
    // Geometry.hpp:303-306
    while TWO_PI < angle {
        angle -= TWO_PI;
    }
    // Geometry.hpp:308
    angle
}

// Geometry.hpp:311-319
/// Geometry.hpp:311-319 `to_range_pi_pi`
#[inline]
pub fn to_range_pi_pi(angle: &mut f64) {
    let pi = std::f64::consts::PI;
    // Geometry.hpp:314
    if *angle > pi || *angle <= -pi {
        // Geometry.hpp:315
        let count = (*angle / (2.0 * pi)).round() as i32;
        // Geometry.hpp:316
        *angle -= count as f64 * 2.0 * pi;
        // Geometry.hpp:317
        debug_assert!(*angle <= pi && *angle > -pi);
    }
}

// Geometry.hpp:493-499
// Is the angle close to a multiple of 90 degrees?
/// Geometry.hpp:493-499 `is_rotation_ninety_degrees(double)`
#[inline]
pub fn is_rotation_ninety_degrees_angle(mut a: f64) -> bool {
    let pi = std::f64::consts::PI;
    // Geometry.hpp:495
    a = a.abs() % (0.5 * pi);
    // Geometry.hpp:496-497
    if a > 0.25 * pi {
        a = 0.5 * pi - a;
    }
    // Geometry.hpp:498
    a < 0.001
}

// Geometry.hpp:501-505
// Is the angle close to a multiple of 90 degrees?
/// Geometry.hpp:501-505 `is_rotation_ninety_degrees(const Vec3d&)`
#[inline]
pub fn is_rotation_ninety_degrees(rotation: &Vec3d) -> bool {
    // Geometry.hpp:504
    is_rotation_ninety_degrees_angle(rotation.x)
        && is_rotation_ninety_degrees_angle(rotation.y)
        && is_rotation_ninety_degrees_angle(rotation.z)
}

// ===========================================================================
// Geometry.cpp free functions
// ===========================================================================

// Geometry.cpp:43-51
// template<class T>
// bool contains(const std::vector<T> &vector, const Point &point)
/// Geometry.cpp:43-51 `contains` (instantiated for `ExPolygons`, Geometry.cpp:51)
pub fn contains(vector: &ExPolygons, point: &Point) -> bool {
    // Geometry.cpp:46-48
    // it->contains(point) — ExPolygon::contains(const Point&, bool border_result = true)
    for it in vector.iter() {
        if it.contains(point, true) {
            return true;
        }
    }
    // Geometry.cpp:49
    false
}

// Geometry.cpp:60-71
/// Geometry.cpp:60-71 `simplify_polygons`
///
/// NOTE: the final `Slic3r::simplify_polygons(pp)` call (Geometry.cpp:70) is the
/// Clipper-based `ClipperUtils::simplify_polygons` which performs a NonZero
/// union to remove self-intersections. That symbol lives in `ClipperUtils.cpp`
/// and is not yet ported, so it is threaded in via `clipper_simplify`.
pub fn simplify_polygons<F>(polygons: &Polygons, tolerance: f64, clipper_simplify: F) -> Polygons
where
    F: FnOnce(&Polygons) -> Polygons,
{
    // Geometry.cpp:62
    let mut pp: Polygons = Vec::new();
    // Geometry.cpp:63-69
    for it in polygons.iter() {
        // Geometry.cpp:64
        let mut p: Polygon = it.clone();
        // Geometry.cpp:65 — p.points.push_back(p.points.front());
        let front = p.points[0];
        p.points.push(front);
        // Geometry.cpp:66 — p.points = MultiPoint::_douglas_peucker(p.points, tolerance);
        p.points = crate::geometry::simplify::douglas_peucker(&p.points, tolerance);
        // Geometry.cpp:67 — p.points.pop_back();
        p.points.pop();
        // Geometry.cpp:68
        pp.push(p);
    }
    // Geometry.cpp:70 — *retval = Slic3r::simplify_polygons(pp);
    clipper_simplify(&pp)
}

// Geometry.cpp:147-158 — ArrangeItem / ArrangeItemIndex (the active #else branch)
/// Geometry.cpp:147-152 `class ArrangeItem`
// `pos` mirrors the C++ struct member (Geometry.cpp:149); it is assigned but the
// active arrange() algorithm only reads `index_x`/`index_y`/`dist`, exactly as
// in C++.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
struct ArrangeItem {
    // Geometry.cpp:149 — Vec2d pos = Vec2d::Zero();
    pos: crate::geometry::PointF,
    // Geometry.cpp:150 — size_t index_x, index_y;
    index_x: usize,
    index_y: usize,
    // Geometry.cpp:151 — coordf_t dist;
    dist: f64,
}

/// Geometry.cpp:153-158 `class ArrangeItemIndex`
#[derive(Debug, Clone, Copy)]
struct ArrangeItemIndex {
    // Geometry.cpp:155 — coordf_t index;
    index: f64,
    // Geometry.cpp:156 — ArrangeItem item;
    item: ArrangeItem,
}

// Geometry.cpp:160-282
// bool arrange(size_t total_parts, const Vec2d &part_size, coordf_t dist, const BoundingBoxf* bb, Pointfs &positions)
/// Geometry.cpp:160-282 `arrange` (active `#else` branch, Geometry.cpp:146-283)
pub fn arrange(
    total_parts: usize,
    part_size: &crate::geometry::PointF,
    dist: f64,
    bb: Option<&crate::bounding_box::BoundingBoxf>,
    positions: &mut Vec<crate::geometry::PointF>,
) -> bool {
    use crate::bounding_box::BoundingBoxf;
    use crate::geometry::linint; // Geometry.cpp:73 `linint` lives in geometry/mod.rs
    use crate::geometry::PointF;

    // Geometry.cpp:163
    positions.clear();

    // Geometry.cpp:165
    let mut part = *part_size;

    // use actual part size (the largest) plus separation distance (half on each side) in spacing algorithm
    // Geometry.cpp:168-169
    part.x += dist;
    part.y += dist;

    // Geometry.cpp:171-178
    let mut area = PointF::new(0.0, 0.0);
    if let Some(bb) = bb {
        if bb.defined {
            area = bb.size();
        } else {
            area.x = part.x * total_parts as f64;
            area.y = part.y * total_parts as f64;
        }
    } else {
        // bogus area size, large enough not to trigger the error below
        area.x = part.x * total_parts as f64;
        area.y = part.y * total_parts as f64;
    }

    // this is how many cells we have available into which to put parts
    // Geometry.cpp:181-182
    let cellw = ((area.x + dist) / part.x).floor() as usize;
    let cellh = ((area.y + dist) / part.y).floor() as usize;
    // Geometry.cpp:183-184
    if total_parts > cellw * cellh {
        return false;
    }

    // total space used by cells
    // Geometry.cpp:187
    let cells = PointF::new(cellw as f64 * part.x, cellh as f64 * part.y);

    // bounding box of total space used by cells
    // Geometry.cpp:190-192
    let mut cells_bb = BoundingBoxf::new();
    cells_bb.merge_point(PointF::new(0.0, 0.0)); // min
    cells_bb.merge_point(cells); // max

    // center bounding box to area
    // Geometry.cpp:195-198
    cells_bb.translate((area.x - cells.x) / 2.0, (area.y - cells.y) / 2.0);

    // list of cells, sorted by distance from center
    // Geometry.cpp:201
    let mut cellsorder: Vec<ArrangeItemIndex> = Vec::new();

    // work out distance for all cells, sort into list
    // Geometry.cpp:204-205
    for i in 0..=cellw - 1 {
        for j in 0..=cellh - 1 {
            // Geometry.cpp:206-207
            let cx = linint(i as f64 + 0.5, 0.0, cellw as f64, cells_bb.min.x, cells_bb.max.x);
            let cy = linint(j as f64 + 0.5, 0.0, cellh as f64, cells_bb.min.y, cells_bb.max.y);

            // Geometry.cpp:209-210
            let xd = ((area.x / 2.0) - cx).abs();
            let yd = ((area.y / 2.0) - cy).abs();

            // Geometry.cpp:212-217
            let c = ArrangeItem {
                pos: PointF::new(cx, cy),
                index_x: i,
                index_y: j,
                dist: xd * xd + yd * yd - ((cellw / 2) as f64 - (i as f64 + 0.5)).abs(),
            };

            // binary insertion sort
            // Geometry.cpp:220-238
            {
                let index = c.dist;
                let mut low = 0usize;
                let mut high = cellsorder.len();
                let mut inserted = false;
                while low < high {
                    // Geometry.cpp:225 — (low + ((high - low) / 2)) | 0
                    let mid = low + ((high - low) / 2);
                    let midval = cellsorder[mid].index;

                    if midval < index {
                        low = mid + 1;
                    } else if midval > index {
                        high = mid;
                    } else {
                        cellsorder.insert(mid, ArrangeItemIndex { index, item: c });
                        inserted = true;
                        break;
                    }
                }
                if !inserted {
                    // Geometry.cpp:237
                    cellsorder.insert(low, ArrangeItemIndex { index, item: c });
                }
            }
            // Geometry.cpp:239 ENDSORT: ;
        }
    }

    // the extents of cells actually used by objects
    // Geometry.cpp:244-247
    let mut lx = 0.0f64;
    let mut ty = 0.0f64;
    let mut rx = 0.0f64;
    let mut by = 0.0f64;

    // now find cells actually used by objects, map out the extents so we can position correctly
    // Geometry.cpp:250-263
    for i in 1..=total_parts {
        let c = cellsorder[i - 1];
        let cx = c.item.index_x as f64;
        let cy = c.item.index_y as f64;
        if i == 1 {
            lx = cx;
            rx = cx;
            ty = cy;
            by = cy;
        } else {
            if cx > rx {
                rx = cx;
            }
            if cx < lx {
                lx = cx;
            }
            if cy > by {
                by = cy;
            }
            if cy < ty {
                ty = cy;
            }
        }
    }
    let _ = (rx, by);

    // now we actually place objects into cells, positioned such that the left and bottom borders are at 0
    // Geometry.cpp:265-272
    for _i in 1..=total_parts {
        let c = cellsorder[0];
        cellsorder.remove(0);
        let cx = c.item.index_x as f64 - lx;
        let cy = c.item.index_y as f64 - ty;

        positions.push(PointF::new(cx * part.x, cy * part.y));
    }

    // Geometry.cpp:274-279
    if let Some(bb) = bb {
        if bb.defined {
            for p in positions.iter_mut() {
                p.x += bb.min.x;
                p.y += bb.min.y;
            }
        }
    }

    // Geometry.cpp:281
    true
}

// ===========================================================================
// Transform assembly / extraction (Geometry.cpp:308-440)
// ===========================================================================
//
// Eigen's `Transform3d` is `Eigen::Transform<double, 3, Affine>`, stored as a
// homogeneous 4x4 matrix. We model it as `nalgebra::Matrix4<f64>`. Eigen's
// fluent mutators (`translate`, `rotate`, `scale`) *post-multiply* the current
// matrix (apply the new transform in the current local frame); the helpers
// below reproduce that exactly.

/// Eigen `Transform3d` == homogeneous affine 4x4 matrix.
pub type Transform3d = Matrix4<f64>;

#[inline]
fn make_translation(t: &Vec3d) -> Matrix4<f64> {
    let mut m = Matrix4::<f64>::identity();
    m[(0, 3)] = t.x;
    m[(1, 3)] = t.y;
    m[(2, 3)] = t.z;
    m
}

#[inline]
fn make_rotation4(r: &Matrix3<f64>) -> Matrix4<f64> {
    let mut m = Matrix4::<f64>::identity();
    m.fixed_view_mut::<3, 3>(0, 0).copy_from(r);
    m
}

#[inline]
fn make_scaling(s: &Vec3d) -> Matrix4<f64> {
    let mut m = Matrix4::<f64>::identity();
    m[(0, 0)] = s.x;
    m[(1, 1)] = s.y;
    m[(2, 2)] = s.z;
    m
}

// Eigen's AngleAxisd(angle, unit_axis) -> rotation matrix.
#[inline]
fn angle_axis_matrix(angle: f64, axis: &Vec3d) -> Matrix3<f64> {
    UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(*axis), angle).to_rotation_matrix().into_inner()
}

#[inline]
fn unit_x() -> Vec3d {
    Vec3d::new(1.0, 0.0, 0.0)
}
#[inline]
fn unit_y() -> Vec3d {
    Vec3d::new(0.0, 1.0, 0.0)
}
#[inline]
fn unit_z() -> Vec3d {
    Vec3d::new(0.0, 0.0, 1.0)
}

// Geometry.cpp:308-314
/// Geometry.cpp:308-314 `assemble_transform(Transform3d&, ...)` (mutating form)
pub fn assemble_transform_into(
    transform: &mut Transform3d,
    translation: &Vec3d,
    rotation: &Vec3d,
    scale: &Vec3d,
    mirror: &Vec3d,
) {
    // Geometry.cpp:310
    *transform = Transform3d::identity();
    // Geometry.cpp:311 — transform.translate(translation);
    *transform *= make_translation(translation);
    // Geometry.cpp:312 — transform.rotate(AngleAxisd(z,UnitZ) * AngleAxisd(y,UnitY) * AngleAxisd(x,UnitX));
    let rot = angle_axis_matrix(rotation.z, &unit_z())
        * angle_axis_matrix(rotation.y, &unit_y())
        * angle_axis_matrix(rotation.x, &unit_x());
    *transform *= make_rotation4(&rot);
    // Geometry.cpp:313 — transform.scale(scale.cwiseProduct(mirror));
    *transform *= make_scaling(&scale.component_mul(mirror));
}

// Geometry.cpp:316-321
/// Geometry.cpp:316-321 `assemble_transform(...)` (returning form)
pub fn assemble_transform(
    translation: &Vec3d,
    rotation: &Vec3d,
    scale: &Vec3d,
    mirror: &Vec3d,
) -> Transform3d {
    // Geometry.cpp:318
    let mut transform = Transform3d::identity();
    // Geometry.cpp:319
    assemble_transform_into(&mut transform, translation, rotation, scale, mirror);
    // Geometry.cpp:320
    transform
}

// Geometry.cpp:323-331
// The extracted "rotation" is a triplet of numbers such that Geometry::rotation_transform
// returns the original transform. Because of the chosen order of rotations, the triplet
// is not equivalent to Euler angles in the usual sense.
/// Geometry.cpp:323-331 `extract_euler_angles(const Matrix3d&)`
pub fn extract_euler_angles_from_matrix(rotation_matrix: &Matrix3<f64>) -> Vec3d {
    // Geometry.cpp:328 — rotation_matrix.eulerAngles(2, 1, 0)
    let angles = euler_angles_zyx(rotation_matrix);
    // Geometry.cpp:329 — std::swap(angles(0), angles(2));
    // Geometry.cpp:330
    Vec3d::new(angles.z, angles.y, angles.x)
}

// Reproduces Eigen's `Matrix3::eulerAngles(2, 1, 0)` ordering. Eigen returns
// (a0, a1, a2) such that m == AngleAxis(a0,UnitZ) * AngleAxis(a1,UnitY) *
// AngleAxis(a2,UnitX). nalgebra's `Rotation3::euler_angles()` returns
// (roll, pitch, yaw) == (x, y, z) for the *same* composition
// R = Rz(yaw) * Ry(pitch) * Rx(roll), i.e. it equals Eigen's eulerAngles(2,1,0)
// already swapped (a0<->a2). We therefore swap back here so that this helper
// returns the *unswapped* Eigen result (matches C++ `eulerAngles(2, 1, 0)`).
fn euler_angles_zyx(m: &Matrix3<f64>) -> Vec3d {
    let rot = nalgebra::Rotation3::from_matrix_unchecked(*m);
    let (x, y, z) = rot.euler_angles();
    // nalgebra returns (x, y, z); Eigen eulerAngles(2,1,0) is (z, y, x).
    Vec3d::new(z, y, x)
}

// Geometry.cpp:333-335
/// Geometry.cpp:333-335 `extract_rotation(const Transform3d&)`
pub fn extract_rotation(transform: &Transform3d) -> Vec3d {
    // Geometry.cpp:334
    extract_euler_angles(transform)
}

// Geometry.cpp:337-346
// use only the non-translational part of the transform
/// Geometry.cpp:337-346 `extract_euler_angles(const Transform3d&)`
pub fn extract_euler_angles(transform: &Transform3d) -> Vec3d {
    // Geometry.cpp:340 — m = transform.matrix().block(0, 0, 3, 3);
    let mut m: Matrix3<f64> = transform.fixed_view::<3, 3>(0, 0).into();
    // Geometry.cpp:342-344 — remove scale (normalize columns)
    for col in 0..3 {
        let mut c = m.column_mut(col);
        let n = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
        if n != 0.0 {
            c[0] /= n;
            c[1] /= n;
            c[2] /= n;
        }
    }
    // Geometry.cpp:345
    extract_euler_angles_from_matrix(&m)
}

// Geometry.cpp:348-354
// static Transform3d extract_rotation_matrix(const Transform3d &trafo)
/// Geometry.cpp:348-354 `extract_rotation_matrix`
fn extract_rotation_matrix(trafo: &Transform3d) -> Transform3d {
    // Geometry.cpp:350-352 — trafo.computeRotationScaling(&rotation, &scale);
    let (rotation, _scale) = compute_rotation_scaling(trafo);
    // Geometry.cpp:353
    make_rotation4(&rotation)
}

// Geometry.cpp:356-362
/// Geometry.cpp:356-362 `extract_rotation_scale`
fn extract_rotation_scale(trafo: &Transform3d) -> (Transform3d, Transform3d) {
    // Geometry.cpp:358-360
    let (rotation, scale) = compute_rotation_scaling(trafo);
    // Geometry.cpp:361
    (make_rotation4(&rotation), make_rotation4(&scale))
}

// Geometry.cpp:364-370
/// Geometry.cpp:364-370 `extract_scale`
fn extract_scale(trafo: &Transform3d) -> Transform3d {
    // Geometry.cpp:366-368
    let (_rotation, scale) = compute_rotation_scaling(trafo);
    // Geometry.cpp:369
    make_rotation4(&scale)
}

// Eigen's `Transform::computeRotationScaling(&R, &S)`: polar decomposition of
// the linear (3x3) part M = R * S, where R is a rotation (det >= 0) and S is a
// symmetric (possibly negative-definite) scaling matrix. Implemented via SVD:
// M = U Σ Vᵀ; R = U Vᵀ (with a sign correction so det(R) > 0); S = V Σ' Vᵀ where
// Σ' absorbs the sign correction.
fn compute_rotation_scaling(trafo: &Transform3d) -> (Matrix3<f64>, Matrix3<f64>) {
    let m: Matrix3<f64> = trafo.fixed_view::<3, 3>(0, 0).into();
    let svd = m.svd(true, true);
    let u = svd.u.unwrap();
    let v_t = svd.v_t.unwrap();
    let mut sing = svd.singular_values;
    // Eigen flips the sign of the smallest singular value and the corresponding
    // column of U so that det(R) == +1 (rotation, no reflection).
    let mut u_corr = u;
    let det = (u * v_t).determinant();
    if det < 0.0 {
        let last = 2usize;
        for r in 0..3 {
            u_corr[(r, last)] = -u_corr[(r, last)];
        }
        sing[last] = -sing[last];
    }
    let rotation = u_corr * v_t;
    let v = v_t.transpose();
    let scale = v * Matrix3::from_diagonal(&sing) * v_t;
    (rotation, scale)
}

// Geometry.cpp:374-396
// static bool contains_skew(const Transform3d &trafo)
/// Geometry.cpp:374-396 `contains_skew`
fn contains_skew(trafo: &Transform3d) -> bool {
    // Geometry.cpp:376-378
    let (_rotation, scale) = compute_rotation_scaling(trafo);

    // Geometry.cpp:380 — if (scale.isDiagonal()) return false;
    if is_diagonal(&scale) {
        return false;
    }

    // Geometry.cpp:382 — if (scale.determinant() >= 0.0) return true;
    if scale.determinant() >= 0.0 {
        return true;
    }

    // the matrix contains mirror
    // Geometry.cpp:385 — ratio = scale.cwiseQuotient(trafo.matrix().block<3,3>(0,0));
    let linear: Matrix3<f64> = trafo.fixed_view::<3, 3>(0, 0).into();
    let mut ratio = Matrix3::<f64>::zeros();
    for r in 0..3 {
        for c in 0..3 {
            ratio[(r, c)] = scale[(r, c)] / linear[(r, c)];
        }
    }

    // Geometry.cpp:387-389
    let check_skew = |i: usize, j: usize, skew: &mut bool| {
        if !ratio[(i, j)].is_nan() && !ratio[(j, i)].is_nan() {
            *skew |= (ratio[(i, j)] * ratio[(j, i)] - 1.0).abs() > EPSILON;
        }
    };

    // Geometry.cpp:391-395
    let mut has_skew = false;
    check_skew(0, 1, &mut has_skew);
    check_skew(0, 2, &mut has_skew);
    check_skew(1, 2, &mut has_skew);
    has_skew
}

// Eigen `Matrix3::isDiagonal()` with the default precision.
fn is_diagonal(m: &Matrix3<f64>) -> bool {
    // Eigen's isDiagonal compares off-diagonals against prec * maxDiagAbs.
    let prec = f64::EPSILON;
    let mut max_diag = 0.0_f64;
    for i in 0..3 {
        max_diag = max_diag.max(m[(i, i)].abs());
    }
    for r in 0..3 {
        for c in 0..3 {
            if r != c && m[(r, c)].abs() > prec * max_diag {
                return false;
            }
        }
    }
    true
}

// Geometry.cpp:398-406
// get rotation from two vectors.
/// Geometry.cpp:398-406 `rotation_from_two_vectors`
pub fn rotation_from_two_vectors(
    from: Vec3d,
    to: Vec3d,
    rotation_axis: &mut Vec3d,
    phi: &mut f64,
    rotation_matrix: Option<&mut Matrix3<f64>>,
) {
    // Geometry.cpp:400 — Quaterniond().setFromTwoVectors(from, to).toRotationMatrix();
    let quat = UnitQuaternion::rotation_between(&from, &to)
        .unwrap_or_else(UnitQuaternion::identity);
    let m: Matrix3<f64> = quat.to_rotation_matrix().into_inner();
    // Geometry.cpp:401 — AngleAxisd aa(m);
    let rot = nalgebra::Rotation3::from_matrix_unchecked(m);
    let (axis, angle) = match rot.axis_angle() {
        Some((axis, angle)) => (axis.into_inner(), angle),
        None => (Vec3d::new(1.0, 0.0, 0.0), 0.0),
    };
    // Geometry.cpp:402
    *rotation_axis = axis;
    // Geometry.cpp:403
    *phi = angle;
    // Geometry.cpp:404-405
    if let Some(rm) = rotation_matrix {
        *rm = m;
    }
}

// Geometry.cpp:408-413
/// Geometry.cpp:408-413 `translation_transform`
pub fn translation_transform(translation: &Vec3d) -> Transform3d {
    // Geometry.cpp:410
    let mut transform = Transform3d::identity();
    // Geometry.cpp:411 — transform.translate(translation);
    transform *= make_translation(translation);
    // Geometry.cpp:412
    transform
}

// Geometry.cpp:415-420
/// Geometry.cpp:415-420 `rotation_transform`
pub fn rotation_transform(rotation: &Vec3d) -> Transform3d {
    // Geometry.cpp:417
    let mut transform = Transform3d::identity();
    // Geometry.cpp:418 — transform.rotate(AngleAxisd(z,UnitZ)*AngleAxisd(y,UnitY)*AngleAxisd(x,UnitX));
    let rot = angle_axis_matrix(rotation.z, &unit_z())
        * angle_axis_matrix(rotation.y, &unit_y())
        * angle_axis_matrix(rotation.x, &unit_x());
    transform *= make_rotation4(&rot);
    // Geometry.cpp:419
    transform
}

// Geometry.cpp:422-424
/// Geometry.cpp:422-424 `scale_transform(Transform3d&, double)`
pub fn scale_transform_uniform_into(transform: &mut Transform3d, scale: f64) {
    // Geometry.cpp:423 — scale_transform(transform, scale * Vec3d::Ones());
    scale_transform_into(transform, &Vec3d::new(scale, scale, scale));
}

// Geometry.cpp:426-430
/// Geometry.cpp:426-430 `scale_transform(Transform3d&, const Vec3d&)`
pub fn scale_transform_into(transform: &mut Transform3d, scale: &Vec3d) {
    // Geometry.cpp:428
    *transform = Transform3d::identity();
    // Geometry.cpp:429 — transform.scale(scale);
    *transform *= make_scaling(scale);
}

// Geometry.cpp:431-433
/// Geometry.cpp:431-433 `scale_transform(double)`
pub fn scale_transform_uniform(scale: f64) -> Transform3d {
    // Geometry.cpp:432 — scale_transform(scale * Vec3d::Ones());
    scale_transform(&Vec3d::new(scale, scale, scale))
}

// Geometry.cpp:435-440
/// Geometry.cpp:435-440 `scale_transform(const Vec3d&)`
pub fn scale_transform(scale: &Vec3d) -> Transform3d {
    // Geometry.cpp:437
    let mut transform = Transform3d::identity();
    // Geometry.cpp:438
    scale_transform_into(&mut transform, scale);
    // Geometry.cpp:439
    transform
}

// ===========================================================================
// Transformation (Geometry.cpp:442-630, Geometry.hpp:376-461)
// ===========================================================================

/// Geometry.hpp:23-28 axis indices for `Transformation` accessors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    X = 0,
    Y = 1,
    Z = 2,
}

/// Geometry.hpp:376-461 `class Transformation`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transformation {
    // Geometry.hpp:378
    m_matrix: Transform3d,
}

impl Default for Transformation {
    fn default() -> Self {
        // Geometry.hpp:378 — Transform3d m_matrix{Transform3d::Identity()};
        Self { m_matrix: Transform3d::identity() }
    }
}

impl Transformation {
    // Geometry.hpp:385
    pub fn new() -> Self {
        Self::default()
    }

    // Geometry.hpp:387 — explicit Transformation(const Transform3d &transform)
    pub fn from_transform(transform: Transform3d) -> Self {
        Self { m_matrix: transform }
    }

    // Geometry.hpp:389 — const Vec3d& get_offset() const
    pub fn get_offset(&self) -> Vec3d {
        Vec3d::new(self.m_matrix[(0, 3)], self.m_matrix[(1, 3)], self.m_matrix[(2, 3)])
    }

    // Geometry.hpp:390 — double get_offset(Axis axis) const
    pub fn get_offset_axis(&self, axis: Axis) -> f64 {
        self.get_offset()[axis as usize]
    }

    // Geometry.cpp:442 — Transform3d Transformation::get_offset_matrix() const
    pub fn get_offset_matrix(&self) -> Transform3d {
        translation_transform(&self.get_offset())
    }

    // Geometry.hpp:394 — void set_offset(const Vec3d &offset)
    pub fn set_offset(&mut self, offset: &Vec3d) {
        self.m_matrix[(0, 3)] = offset.x;
        self.m_matrix[(1, 3)] = offset.y;
        self.m_matrix[(2, 3)] = offset.z;
    }

    // Geometry.hpp:395 — void set_offset(Axis axis, double offset)
    pub fn set_offset_axis(&mut self, axis: Axis, offset: f64) {
        self.m_matrix[(axis as usize, 3)] = offset;
    }

    // Geometry.cpp:444-448 — const Vec3d &Transformation::get_rotation() const
    pub fn get_rotation(&self) -> Vec3d {
        // Geometry.cpp:446
        extract_rotation(&extract_rotation_matrix(&self.m_matrix))
    }

    // Geometry.cpp:450-458 — get_rotation_by_quaternion
    pub fn get_rotation_by_quaternion(&self) -> Vec3d {
        // Geometry.cpp:452
        let rotation_matrix: Matrix3<f64> = self.m_matrix.fixed_view::<3, 3>(0, 0).into();
        // Geometry.cpp:453-454 — Quaterniond(rotation_matrix); normalize();
        let quaternion = UnitQuaternion::from_matrix(&rotation_matrix);
        // Geometry.cpp:455 — quaternion.matrix().eulerAngles(2, 1, 0);
        let m: Matrix3<f64> = quaternion.to_rotation_matrix().into_inner();
        let mut temp = euler_angles_zyx(&m);
        // Geometry.cpp:456 — std::swap(m_temp_rotation(0), m_temp_rotation(2));
        let tmp = temp.x;
        temp.x = temp.z;
        temp.z = tmp;
        // Geometry.cpp:457
        temp
    }

    // Geometry.hpp:399 — double get_rotation(Axis axis) const
    pub fn get_rotation_axis(&self, axis: Axis) -> f64 {
        self.get_rotation()[axis as usize]
    }

    // Geometry.cpp:460 — Transform3d Transformation::get_rotation_matrix() const
    pub fn get_rotation_matrix(&self) -> Transform3d {
        extract_rotation_matrix(&self.m_matrix)
    }

    // Geometry.cpp:462-467 — void Transformation::set_rotation_matrix(const Transform3d &rot_mat)
    pub fn set_rotation_matrix(&mut self, rot_mat: &Transform3d) {
        // Geometry.cpp:464
        let offset = self.get_offset();
        // Geometry.cpp:465 — m_matrix = rot_mat * extract_scale(m_matrix);
        self.m_matrix = rot_mat * extract_scale(&self.m_matrix);
        // Geometry.cpp:466
        self.set_offset(&offset);
    }

    // Geometry.cpp:469-474 — void Transformation::set_rotation(const Vec3d &rotation)
    pub fn set_rotation(&mut self, rotation: &Vec3d) {
        // Geometry.cpp:471
        let offset = self.get_offset();
        // Geometry.cpp:472 — m_matrix = rotation_transform(rotation) * extract_scale(m_matrix);
        self.m_matrix = rotation_transform(rotation) * extract_scale(&self.m_matrix);
        // Geometry.cpp:473
        self.set_offset(&offset);
    }

    // Geometry.cpp:476-481 — const Vec3d &Transformation::get_scaling_factor() const
    pub fn get_scaling_factor(&self) -> Vec3d {
        // Geometry.cpp:478
        let scale = extract_scale(&self.m_matrix);
        // Geometry.cpp:479
        Vec3d::new(scale[(0, 0)].abs(), scale[(1, 1)].abs(), scale[(2, 2)].abs())
    }

    // Geometry.hpp:407 — double get_scaling_factor(Axis axis) const
    pub fn get_scaling_factor_axis(&self, axis: Axis) -> f64 {
        self.get_scaling_factor()[axis as usize]
    }

    // Geometry.cpp:483-490 — Transform3d Transformation::get_scaling_factor_matrix() const
    pub fn get_scaling_factor_matrix(&self) -> Transform3d {
        // Geometry.cpp:485
        let mut scale = extract_scale(&self.m_matrix);
        // Geometry.cpp:486-488
        scale[(0, 0)] = scale[(0, 0)].abs();
        scale[(1, 1)] = scale[(1, 1)].abs();
        scale[(2, 2)] = scale[(2, 2)].abs();
        // Geometry.cpp:489
        scale
    }

    // Geometry.hpp:411-415 — bool is_scaling_uniform() const
    pub fn is_scaling_uniform(&self) -> bool {
        // Geometry.hpp:413
        let scale = self.get_scaling_factor();
        // Geometry.hpp:414
        (scale.x - scale.y).abs() < 1e-8 && (scale.x - scale.z).abs() < 1e-8
    }

    // Geometry.cpp:492-499 — void Transformation::set_scaling_factor(const Vec3d &scaling_factor)
    pub fn set_scaling_factor(&mut self, scaling_factor: &Vec3d) {
        // Geometry.cpp:494
        debug_assert!(scaling_factor.x > 0.0 && scaling_factor.y > 0.0 && scaling_factor.z > 0.0);
        // Geometry.cpp:496
        let offset = self.get_offset();
        // Geometry.cpp:497 — m_matrix = extract_rotation_matrix(m_matrix) * scale_transform(scaling_factor);
        self.m_matrix = extract_rotation_matrix(&self.m_matrix) * scale_transform(scaling_factor);
        // Geometry.cpp:498
        self.set_offset(&offset);
    }

    // Geometry.cpp:501-511 — void Transformation::set_scaling_factor(Axis axis, double scaling_factor)
    pub fn set_scaling_factor_axis(&mut self, axis: Axis, scaling_factor: f64) {
        // Geometry.cpp:503
        debug_assert!(scaling_factor > 0.0);
        // Geometry.cpp:505
        let (rotation, mut scale) = extract_rotation_scale(&self.m_matrix);
        // Geometry.cpp:506
        scale[(axis as usize, axis as usize)] = scaling_factor;
        // Geometry.cpp:508
        let offset = self.get_offset();
        // Geometry.cpp:509
        self.m_matrix = rotation * scale;
        // Geometry.cpp:510
        self.set_offset(&offset);
    }

    // Geometry.cpp:513-518 — const Vec3d &Transformation::get_mirror() const
    pub fn get_mirror(&self) -> Vec3d {
        // Geometry.cpp:515
        let scale = extract_scale(&self.m_matrix);
        // Geometry.cpp:516
        Vec3d::new(
            scale[(0, 0)] / scale[(0, 0)].abs(),
            scale[(1, 1)] / scale[(1, 1)].abs(),
            scale[(2, 2)] / scale[(2, 2)].abs(),
        )
    }

    // Geometry.hpp:421 — double get_mirror(Axis axis) const
    pub fn get_mirror_axis(&self, axis: Axis) -> f64 {
        self.get_mirror()[axis as usize]
    }

    // Geometry.cpp:520-527 — Transform3d Transformation::get_mirror_matrix() const
    pub fn get_mirror_matrix(&self) -> Transform3d {
        // Geometry.cpp:522
        let mut scale = extract_scale(&self.m_matrix);
        // Geometry.cpp:523-525
        scale[(0, 0)] /= scale[(0, 0)].abs();
        scale[(1, 1)] /= scale[(1, 1)].abs();
        scale[(2, 2)] /= scale[(2, 2)].abs();
        // Geometry.cpp:526
        scale
    }

    // Geometry.hpp:425 — bool is_left_handed() const
    pub fn is_left_handed(&self) -> bool {
        let linear: Matrix3<f64> = self.m_matrix.fixed_view::<3, 3>(0, 0).into();
        linear.determinant() < 0.0
    }

    // Geometry.cpp:529-551 — void Transformation::set_mirror(const Vec3d &mirror)
    pub fn set_mirror(&mut self, mirror: &Vec3d) {
        // Geometry.cpp:531
        let mut copy = *mirror;
        // Geometry.cpp:532 — const Vec3d abs_mirror = copy.cwiseAbs();
        let abs_mirror = Vec3d::new(copy.x.abs(), copy.y.abs(), copy.z.abs());
        // Geometry.cpp:533-538
        for i in 0..3 {
            if abs_mirror[i] == 0.0 {
                copy[i] = 1.0;
            } else if abs_mirror[i] != 1.0 {
                copy[i] /= abs_mirror[i];
            }
        }

        // Geometry.cpp:540
        let (rotation, mut scale) = extract_rotation_scale(&self.m_matrix);
        // Geometry.cpp:541 — const Vec3d curr_scales = {scale(0,0), scale(1,1), scale(2,2)};
        let curr_scales = Vec3d::new(scale[(0, 0)], scale[(1, 1)], scale[(2, 2)]);
        // Geometry.cpp:542 — const Vec3d signs = curr_scales.cwiseProduct(copy);
        let signs = curr_scales.component_mul(&copy);

        // Geometry.cpp:544-546
        if signs[0] < 0.0 {
            scale[(0, 0)] = -scale[(0, 0)];
        }
        if signs[1] < 0.0 {
            scale[(1, 1)] = -scale[(1, 1)];
        }
        if signs[2] < 0.0 {
            scale[(2, 2)] = -scale[(2, 2)];
        }

        // Geometry.cpp:548
        let offset = self.get_offset();
        // Geometry.cpp:549
        self.m_matrix = rotation * scale;
        // Geometry.cpp:550
        self.set_offset(&offset);
    }

    // Geometry.cpp:553-570 — void Transformation::set_mirror(Axis axis, double mirror)
    pub fn set_mirror_axis(&mut self, axis: Axis, mut mirror: f64) {
        // Geometry.cpp:555
        let abs_mirror = mirror.abs();
        // Geometry.cpp:556-559
        if abs_mirror == 0.0 {
            mirror = 1.0;
        } else if abs_mirror != 1.0 {
            mirror /= abs_mirror;
        }

        // Geometry.cpp:561
        let (rotation, mut scale) = extract_rotation_scale(&self.m_matrix);
        // Geometry.cpp:562
        let curr_scale = scale[(axis as usize, axis as usize)];
        // Geometry.cpp:563
        let sign = curr_scale * mirror;

        // Geometry.cpp:565
        if sign < 0.0 {
            scale[(axis as usize, axis as usize)] = -scale[(axis as usize, axis as usize)];
        }

        // Geometry.cpp:567
        let offset = self.get_offset();
        // Geometry.cpp:568
        self.m_matrix = rotation * scale;
        // Geometry.cpp:569
        self.set_offset(&offset);
    }

    // Geometry.cpp:572 — bool Transformation::has_skew() const
    pub fn has_skew(&self) -> bool {
        contains_skew(&self.m_matrix)
    }

    // Geometry.cpp:574 — void Transformation::reset()
    pub fn reset(&mut self) {
        self.m_matrix = Transform3d::identity();
    }

    // Geometry.hpp:433 — void reset_offset()
    pub fn reset_offset(&mut self) {
        self.set_offset(&Vec3d::new(0.0, 0.0, 0.0));
    }

    // Geometry.cpp:576-580 — void Transformation::reset_rotation()
    pub fn reset_rotation(&mut self) {
        // Geometry.cpp:578
        let svd = TransformationSVD::from_transformation(self);
        // Geometry.cpp:579 — m_matrix = get_offset_matrix() * Transform3d(svd.v * svd.s * svd.v.transpose()) * svd.mirror_matrix();
        self.m_matrix = self.get_offset_matrix()
            * make_rotation4(&(svd.v * svd.s * svd.v.transpose()))
            * svd.mirror_matrix();
    }

    // Geometry.cpp:582-586 — void Transformation::reset_scaling_factor()
    pub fn reset_scaling_factor(&mut self) {
        // Geometry.cpp:584
        let svd = TransformationSVD::from_transformation(self);
        // Geometry.cpp:585 — m_matrix = get_offset_matrix() * Transform3d(svd.u) * Transform3d(svd.v.transpose()) * svd.mirror_matrix();
        self.m_matrix = self.get_offset_matrix()
            * make_rotation4(&svd.u)
            * make_rotation4(&svd.v.transpose())
            * svd.mirror_matrix();
    }

    // Geometry.hpp:436 — void reset_mirror()
    pub fn reset_mirror(&mut self) {
        self.set_mirror(&Vec3d::new(1.0, 1.0, 1.0));
    }

    // Geometry.cpp:588-596 — void Transformation::reset_skew()
    pub fn reset_skew(&mut self) {
        // Geometry.cpp:590-592
        let new_scale_factor = |s: &Matrix3<f64>| -> f64 {
            (s[(0, 0)] * s[(1, 1)] * s[(2, 2)]).powf(1.0 / 3.0) // scale average
        };

        // Geometry.cpp:594
        let svd = TransformationSVD::from_transformation(self);
        // Geometry.cpp:595
        self.m_matrix = self.get_offset_matrix()
            * make_rotation4(&svd.u)
            * scale_transform_uniform(new_scale_factor(&svd.s))
            * make_rotation4(&svd.v.transpose())
            * svd.mirror_matrix();
    }

    // Geometry.cpp:598-614 — const Transform3d &Transformation::get_matrix(bool, bool, bool, bool) const
    pub fn get_matrix(
        &self,
        dont_translate: bool,
        dont_rotate: bool,
        dont_scale: bool,
        dont_mirror: bool,
    ) -> Transform3d {
        // Geometry.cpp:600-602
        if !dont_translate && !dont_rotate && !dont_scale && !dont_mirror {
            return self.m_matrix;
        }
        // Geometry.cpp:603
        let mut refence_tran = Transformation::from_transform(self.m_matrix);
        // Geometry.cpp:604-605
        if dont_translate {
            refence_tran.reset_offset();
        }
        // Geometry.cpp:606-607
        if dont_rotate {
            refence_tran.reset_rotation();
        }
        // Geometry.cpp:608-609
        if dont_scale {
            refence_tran.reset_scaling_factor();
        }
        // Geometry.cpp:610-611
        if dont_mirror {
            refence_tran.reset_mirror();
        }
        // Geometry.cpp:612-613
        refence_tran.get_matrix(false, false, false, false)
    }

    // Geometry.hpp:439 — convenience default-arg overload: get_matrix() == get_matrix(false,false,false,false)
    pub fn matrix(&self) -> Transform3d {
        self.m_matrix
    }

    // Geometry.cpp:616-621 — Transform3d Transformation::get_matrix_no_offset() const
    pub fn get_matrix_no_offset(&self) -> Transform3d {
        // Geometry.cpp:618
        let mut copy = *self;
        // Geometry.cpp:619
        copy.reset_offset();
        // Geometry.cpp:620
        copy.matrix()
    }

    // Geometry.cpp:623-628 — Transform3d Transformation::get_matrix_no_scaling_factor() const
    pub fn get_matrix_no_scaling_factor(&self) -> Transform3d {
        // Geometry.cpp:625
        let mut copy = *self;
        // Geometry.cpp:626
        copy.reset_scaling_factor();
        // Geometry.cpp:627
        copy.matrix()
    }

    // Geometry.hpp:444 — void set_matrix(const Transform3d &transform)
    pub fn set_matrix(&mut self, transform: Transform3d) {
        self.m_matrix = transform;
    }

    // Geometry.hpp:445 — void set_from_transform(const Transform3d &transform)
    pub fn set_from_transform(&mut self, transform: Transform3d) {
        self.m_matrix = transform;
    }

    // Geometry.cpp:634-690 / Geometry.hpp:447 — static Transformation volume_to_bed_transformation(...)
    pub fn volume_to_bed_transformation(
        instance_transformation: &Transformation,
        bbox: &crate::bounding_box::BoundingBoxf3,
    ) -> Transformation {
        // Geometry.cpp:636
        let mut out = Transformation::new();

        // Geometry.cpp:638
        if instance_transformation.is_scaling_uniform() {
            // No need to run the non-linear least squares fitting for uniform scaling.
            // Just set the inverse.
            // Geometry.cpp:641
            let m = instance_transformation.get_matrix(true, false, false, false);
            let inv = m.try_inverse().unwrap_or_else(Transform3d::identity);
            out.set_from_transform(inv);
        }
        // Geometry.cpp:643
        else if is_rotation_ninety_degrees(&instance_transformation.get_rotation()) {
            // Anisotropic scaling, rotation by multiples of ninety degrees.
            // Geometry.cpp:646-649
            let rot = instance_transformation.get_rotation();
            let instance_rotation_trafo = angle_axis_matrix(rot.z, &unit_z())
                * angle_axis_matrix(rot.y, &unit_y())
                * angle_axis_matrix(rot.x, &unit_x());
            // Geometry.cpp:650-653
            let volume_rotation_trafo = angle_axis_matrix(-rot.x, &unit_x())
                * angle_axis_matrix(-rot.y, &unit_y())
                * angle_axis_matrix(-rot.z, &unit_z());

            // 8 corners of the bounding box.
            // Geometry.cpp:656-664
            let mut pts = nalgebra::DMatrix::<f64>::zeros(8, 3);
            let set_pt = |pts: &mut nalgebra::DMatrix<f64>, r: usize, x: f64, y: f64, z: f64| {
                pts[(r, 0)] = x;
                pts[(r, 1)] = y;
                pts[(r, 2)] = z;
            };
            set_pt(&mut pts, 0, bbox.min.x, bbox.min.y, bbox.min.z);
            set_pt(&mut pts, 1, bbox.min.x, bbox.min.y, bbox.max.z);
            set_pt(&mut pts, 2, bbox.min.x, bbox.max.y, bbox.min.z);
            set_pt(&mut pts, 3, bbox.min.x, bbox.max.y, bbox.max.z);
            set_pt(&mut pts, 4, bbox.max.x, bbox.min.y, bbox.min.z);
            set_pt(&mut pts, 5, bbox.max.x, bbox.min.y, bbox.max.z);
            set_pt(&mut pts, 6, bbox.max.x, bbox.max.y, bbox.min.z);
            set_pt(&mut pts, 7, bbox.max.x, bbox.max.y, bbox.max.z);

            // Corners of the bounding box transformed into the modifier mesh coordinate space,
            // with inverse rotation applied to the modifier.
            // Geometry.cpp:667-670
            let sf = instance_transformation.get_scaling_factor();
            let mr = instance_transformation.get_mirror();
            let scaling = make_scaling3(&sf.component_mul(&mr));
            let combined = instance_rotation_trafo * scaling * volume_rotation_trafo;
            let combined_inv = combined.try_inverse().unwrap_or_else(Matrix3::identity);
            let qs = &pts * combined_inv.transpose();

            // Fill in scaling based on least squares fitting of the bounding box corners.
            // Geometry.cpp:672-674
            let mut scale = Vec3d::new(0.0, 0.0, 0.0);
            for i in 0..3 {
                let col_p = pts.column(i);
                let col_q = qs.column(i);
                scale[i] = col_p.dot(&col_q) / col_p.dot(&col_p);
            }

            // Geometry.cpp:676
            out.set_rotation(&extract_euler_angles_from_matrix(&volume_rotation_trafo));
            // Geometry.cpp:677
            out.set_scaling_factor(&Vec3d::new(scale[0].abs(), scale[1].abs(), scale[2].abs()));
            // Geometry.cpp:678
            out.set_mirror(&Vec3d::new(
                if scale[0] > 0.0 { 1.0 } else { -1.0 },
                if scale[1] > 0.0 { 1.0 } else { -1.0 },
                if scale[2] > 0.0 { 1.0 } else { -1.0 },
            ));
        }
        // Geometry.cpp:680
        else {
            // General anisotropic scaling, general rotation.
            // Keep the modifier mesh in the instance coordinate system, so the modifier mesh will not be aligned with the world.
            // Scale it to get the required size.
            // Geometry.cpp:685
            let scling_facor = instance_transformation.get_scaling_factor();
            // Geometry.cpp:686 — out.set_scaling_factor(scling_facor.cwiseInverse());
            out.set_scaling_factor(&Vec3d::new(
                1.0 / scling_facor.x,
                1.0 / scling_facor.y,
                1.0 / scling_facor.z,
            ));
        }

        // Geometry.cpp:689
        out
    }
}

// Geometry.cpp:630 — Transformation Transformation::operator*(const Transformation &other) const
impl std::ops::Mul for Transformation {
    type Output = Transformation;
    fn mul(self, other: Transformation) -> Transformation {
        // { return Transformation(get_matrix() * other.get_matrix()); }
        Transformation::from_transform(self.matrix() * other.matrix())
    }
}

#[inline]
fn make_scaling3(s: &Vec3d) -> Matrix3<f64> {
    let mut m = Matrix3::<f64>::identity();
    m[(0, 0)] = s.x;
    m[(1, 1)] = s.y;
    m[(2, 2)] = s.z;
    m
}

// ===========================================================================
// TransformationSVD (Geometry.cpp:744-786, Geometry.hpp:463-480)
// ===========================================================================

/// Geometry.hpp:463-480 `struct TransformationSVD`
#[derive(Debug, Clone)]
pub struct TransformationSVD {
    // Geometry.hpp:465-467
    pub u: Matrix3<f64>,
    pub s: Matrix3<f64>,
    pub v: Matrix3<f64>,

    // Geometry.hpp:469-474
    pub mirror: bool,
    pub scale: bool,
    pub anisotropic_scale: bool,
    pub rotation: bool,
    pub rotation_90_degrees: bool,
    pub skew: bool,
}

impl TransformationSVD {
    // Geometry.hpp:476 — explicit TransformationSVD(const Transformation &trafo) : TransformationSVD(trafo.get_matrix()) {}
    pub fn from_transformation(trafo: &Transformation) -> Self {
        Self::from_transform(&trafo.matrix())
    }

    // Geometry.cpp:744-786 — TransformationSVD::TransformationSVD(const Transform3d &trafo)
    pub fn from_transform(trafo: &Transform3d) -> Self {
        // Geometry.cpp:746 — const auto &m0 = trafo.matrix().block<3, 3>(0, 0);
        let m0: Matrix3<f64> = trafo.fixed_view::<3, 3>(0, 0).into();
        // Geometry.cpp:747 — mirror = m0.determinant() < 0.0;
        let mirror = m0.determinant() < 0.0;

        // Geometry.cpp:749-753
        let m = if mirror {
            m0 * Matrix3::from_diagonal(&Vec3d::new(-1.0, 1.0, 1.0))
        } else {
            m0
        };
        // Geometry.cpp:754 — JacobiSVD<Matrix3d> svd(m, ComputeFullU | ComputeFullV);
        let svd = m.svd(true, true);
        // Geometry.cpp:755-757
        let u = svd.u.unwrap();
        let v_t = svd.v_t.unwrap();
        let v = v_t.transpose();
        let s = Matrix3::from_diagonal(&svd.singular_values);

        // Geometry.cpp:759 — scale = !s.isApprox(Matrix3d::Identity());
        let scale = !s.relative_eq(&Matrix3::identity(), f64::EPSILON.sqrt(), f64::EPSILON.sqrt());
        // Geometry.cpp:760 — anisotropic_scale = !is_approx(s(0,0), s(1,1)) || !is_approx(s(1,1), s(2,2));
        let anisotropic_scale =
            !crate::geometry::geometry::is_approx(s[(0, 0)], s[(1, 1)])
                || !crate::geometry::geometry::is_approx(s[(1, 1)], s[(2, 2)]);
        // Geometry.cpp:761 — rotation = !v.isApprox(u);
        let rotation = !v.relative_eq(&u, f64::EPSILON.sqrt(), f64::EPSILON.sqrt());

        let mut rotation_90_degrees = false;
        let skew;
        // Geometry.cpp:763
        if anisotropic_scale {
            // Geometry.cpp:764
            rotation_90_degrees = true;
            // Geometry.cpp:765-773
            for i in 0..3 {
                // const Vec3d row = v.row(i).cwiseAbs();
                let row = Vec3d::new(v[(i, 0)].abs(), v[(i, 1)].abs(), v[(i, 2)].abs());
                let num_zeros = (is_approx(row[0], 0.0) as usize)
                    + (is_approx(row[1], 0.0) as usize)
                    + (is_approx(row[2], 0.0) as usize);
                let num_ones = (is_approx(row[0], 1.0) as usize)
                    + (is_approx(row[1], 1.0) as usize)
                    + (is_approx(row[2], 1.0) as usize);
                if num_zeros != 2 || num_ones != 1 {
                    rotation_90_degrees = false;
                    break;
                }
            }
            // Detect skew by brute force: check if the axes are still orthogonal after transformation
            // Geometry.cpp:775
            let trafo_linear: Matrix3<f64> = trafo.fixed_view::<3, 3>(0, 0).into();
            // Geometry.cpp:776
            let axes = [unit_x(), unit_y(), unit_z()];
            // Geometry.cpp:777-778
            let mut transformed_axes = [Vec3d::zeros(); 3];
            for i in 0..3 {
                transformed_axes[i] = trafo_linear * axes[i];
            }
            // Geometry.cpp:779-780
            skew = transformed_axes[0].dot(&transformed_axes[1]).abs() > EPSILON
                || transformed_axes[1].dot(&transformed_axes[2]).abs() > EPSILON
                || transformed_axes[2].dot(&transformed_axes[0]).abs() > EPSILON;

            // This following old code does not work under all conditions. The v matrix can become non diagonal (see SPE-1492)
            //        skew = ! rotation_90_degrees;
        } else {
            // Geometry.cpp:785
            skew = false;
        }

        TransformationSVD {
            u,
            s,
            v,
            mirror,
            scale,
            anisotropic_scale,
            rotation,
            rotation_90_degrees,
            skew,
        }
    }

    // Geometry.hpp:479 — Eigen::DiagonalMatrix<double, 3, 3> mirror_matrix() const
    pub fn mirror_matrix(&self) -> Transform3d {
        // { return DiagonalMatrix(this->mirror ? -1. : 1., 1., 1.); }
        make_scaling(&Vec3d::new(if self.mirror { -1.0 } else { 1.0 }, 1.0, 1.0))
    }
}

// `is_approx(a, b)` with the BambuStudio default precision (matches libslic3r.h
// `is_approx`, which uses `EPSILON` as default precision for double).
#[inline]
pub fn is_approx(value: f64, test_value: f64) -> bool {
    (value - test_value).abs() < EPSILON
}

// Geometry.cpp:693-718
// For parsing a transformation matrix from 3MF / AMF.
/// Geometry.cpp:693-718 `transform3d_from_string`
pub fn transform3d_from_string(transform_str: &str) -> Transform3d {
    // Geometry.cpp:695 — assert(is_decimal_separator_point());
    // Geometry.cpp:696
    let mut transform = Transform3d::identity();

    // Geometry.cpp:698
    if !transform_str.is_empty() {
        // Geometry.cpp:700-701 — boost::split(... is_any_of(" "), token_compress_on);
        let mat_elements_str: Vec<&str> =
            transform_str.split(' ').filter(|s| !s.is_empty()).collect();

        // Geometry.cpp:703
        let size = mat_elements_str.len();
        // Geometry.cpp:704
        if size == 16 {
            // Geometry.cpp:706
            let mut i = 0usize;
            // Geometry.cpp:707-714
            for r in 0..4 {
                for c in 0..4 {
                    // transform(r, c) = ::atof(mat_elements_str[i++].c_str());
                    transform[(r, c)] = mat_elements_str[i].parse::<f64>().unwrap_or(0.0);
                    i += 1;
                }
            }
        }
    }

    // Geometry.cpp:717
    transform
}

// Geometry.cpp:720-727
/// Geometry.cpp:720-727 `rotation_xyz_diff`
pub fn rotation_xyz_diff(rot_xyz_from: &Vec3d, rot_xyz_to: &Vec3d) -> UnitQuaternion<f64> {
    // Geometry.cpp:722-726
    let from_to_world = angle_axis_matrix(rot_xyz_to.z, &unit_z())
        * angle_axis_matrix(rot_xyz_to.y, &unit_y())
        * angle_axis_matrix(rot_xyz_to.x, &unit_x());
    let world_to_init = angle_axis_matrix(-rot_xyz_from.x, &unit_x())
        * angle_axis_matrix(-rot_xyz_from.y, &unit_y())
        * angle_axis_matrix(-rot_xyz_from.z, &unit_z());
    let m = from_to_world * world_to_init;
    UnitQuaternion::from_matrix(&m)
}

// Geometry.cpp:729-742
// This should only be called if it is known, that the two rotations only differ in rotation around the Z axis.
/// Geometry.cpp:730-742 `rotation_diff_z`
pub fn rotation_diff_z(rot_xyz_from: &Vec3d, rot_xyz_to: &Vec3d) -> f64 {
    // Geometry.cpp:732 — AngleAxisd angle_axis(rotation_xyz_diff(...));
    let quat = rotation_xyz_diff(rot_xyz_from, rot_xyz_to);
    let rot = quat.to_rotation_matrix();
    let (axis, angle) = match rot.axis_angle() {
        Some((axis, angle)) => (axis.into_inner(), angle),
        None => (Vec3d::new(0.0, 0.0, 1.0), 0.0),
    };
    // Geometry.cpp:733-734
    let _ = &axis;
    // Geometry.cpp:735-740 (NDEBUG-guarded asserts)
    #[cfg(debug_assertions)]
    {
        if angle.abs() > 1e-8 {
            debug_assert!(axis.x.abs() < 1e-8);
            debug_assert!(axis.y.abs() < 1e-8);
        }
    }
    // Geometry.cpp:741 — return (axis.z() < 0) ? -angle : angle;
    if axis.z < 0.0 {
        -angle
    } else {
        angle
    }
}

// Geometry.cpp:788-813
/// Geometry.cpp:788-813 `mat_around_a_point_rotate`
pub fn mat_around_a_point_rotate(
    in_mat: &Transformation,
    pt: &Vec3d,
    axis: &Vec3d,
    rotate_theta_radian: f32,
) -> Transformation {
    // Geometry.cpp:790
    let xyz = in_mat.get_offset();
    // Geometry.cpp:791-792 — left.set_offset(-xyz);
    let mut left = Transformation::new();
    left.set_offset(&(-xyz)); // at world origin
    // Geometry.cpp:793 — auto curMat = left * InMat;
    let mut cur_mat = left * *in_mat;

    // Geometry.cpp:795-796 — qua = Quaterniond(AngleAxisd(rotate_theta_radian, axis)); normalize();
    let qua = UnitQuaternion::from_axis_angle(
        &nalgebra::Unit::new_normalize(*axis),
        rotate_theta_radian as f64,
    );
    // Geometry.cpp:797-799 — rotateMat4.set_from_transform(fromPositionOrientationScale(0, qua, 1));
    let mut rotate_mat4 = Transformation::new();
    rotate_mat4.set_from_transform(from_position_orientation_scale(
        &Vec3d::new(0., 0., 0.),
        &qua,
        &Vec3d::new(1., 1., 1.),
    ));

    // Geometry.cpp:801 — curMat = rotateMat4 * curMat;  // along_fix_axis
    cur_mat = rotate_mat4 * cur_mat;
    // rotate mat4 along fix pt
    // Geometry.cpp:803-804
    let mut temp_world = Transformation::new();
    let qua_world =
        UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(*axis), 0.0);
    // Geometry.cpp:806-807
    temp_world.set_from_transform(from_position_orientation_scale(
        pt,
        &qua_world,
        &Vec3d::new(1., 1., 1.),
    ));
    // Geometry.cpp:808 — auto temp_xyz = temp_world.get_matrix().inverse() * xyz;
    let temp_world_mat = temp_world.matrix();
    let temp_world_inv = temp_world_mat.try_inverse().unwrap_or_else(Transform3d::identity);
    let temp_xyz = transform_point(&temp_world_inv, &xyz);
    // Geometry.cpp:809 — auto new_pos = temp_world.get_matrix() * (rotateMat4.get_matrix() * temp_xyz);
    let inner = transform_point(&rotate_mat4.matrix(), &temp_xyz);
    let new_pos = transform_point(&temp_world_mat, &inner);
    // Geometry.cpp:810
    cur_mat.set_offset(&new_pos);

    // Geometry.cpp:812
    cur_mat
}

// Eigen `Transform::fromPositionOrientationScale(pos, q, scale)` builds an affine
// transform: translation(pos) * rotation(q) * scaling(scale).
fn from_position_orientation_scale(
    pos: &Vec3d,
    q: &UnitQuaternion<f64>,
    scale: &Vec3d,
) -> Transform3d {
    let rot = q.to_rotation_matrix().into_inner();
    make_translation(pos) * make_rotation4(&rot) * make_scaling(scale)
}

// Apply an affine 4x4 transform to a 3D point (Eigen's `Transform3d * Vec3d`
// treats the Vec3d as a point: result = linear * v + translation).
#[inline]
fn transform_point(m: &Transform3d, v: &Vec3d) -> Vec3d {
    let linear: Matrix3<f64> = m.fixed_view::<3, 3>(0, 0).into();
    let t = Vec3d::new(m[(0, 3)], m[(1, 3)], m[(2, 3)]);
    linear * v + t
}

// Geometry.cpp:815-824
/// Geometry.cpp:815-824 `generate_transform`
pub fn generate_transform(x_dir: &Vec3d, y_dir: &Vec3d, z_dir: &Vec3d, origin: &Vec3d) -> Transformation {
    // Geometry.cpp:816-819
    let mut m = Matrix3::<f64>::zeros();
    m.column_mut(0).copy_from(&x_dir.normalize());
    m.column_mut(1).copy_from(&y_dir.normalize());
    m.column_mut(2).copy_from(&z_dir.normalize());
    // Geometry.cpp:820 — Transform3d mm(m);
    let mm = make_rotation4(&m);
    // Geometry.cpp:821 — Transformation tran(mm);
    let mut tran = Transformation::from_transform(mm);
    // Geometry.cpp:822
    tran.set_offset(origin);
    // Geometry.cpp:823
    tran
}

/// Eigen `Vec3d` == nalgebra `Vector3<f64>`.
pub type Vec3d = Vector3<f64>;
/// Eigen `Vec2d` == nalgebra `Vector2<f64>`.
pub type Vec2d = nalgebra::Vector2<f64>;

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f64, b: f64) -> bool {
        (a - b).abs() < 1e-9
    }

    #[test]
    fn test_orient() {
        let a = Point::new(0, 0);
        let b = Point::new(10, 0);
        let c = Point::new(10, 10);
        assert_eq!(orient(&a, &b, &c), Orientation::OrientationCcw);
        let d = Point::new(10, -10);
        assert_eq!(orient(&a, &b, &d), Orientation::OrientationCw);
        let e = Point::new(20, 0);
        assert_eq!(orient(&a, &b, &e), Orientation::OrientationColinear);
    }

    #[test]
    fn test_rad2deg_deg2rad() {
        assert!(approx(rad2deg(std::f64::consts::PI), 180.0));
        assert!(approx(deg2rad(180.0), std::f64::consts::PI));
    }

    #[test]
    fn test_translation_transform() {
        let t = translation_transform(&Vec3d::new(1.0, 2.0, 3.0));
        assert!(approx(t[(0, 3)], 1.0));
        assert!(approx(t[(1, 3)], 2.0));
        assert!(approx(t[(2, 3)], 3.0));
    }

    #[test]
    fn test_assemble_identity() {
        let t = assemble_transform(
            &Vec3d::new(0.0, 0.0, 0.0),
            &Vec3d::new(0.0, 0.0, 0.0),
            &Vec3d::new(1.0, 1.0, 1.0),
            &Vec3d::new(1.0, 1.0, 1.0),
        );
        assert!(t.relative_eq(&Transform3d::identity(), 1e-12, 1e-12));
    }

    #[test]
    fn test_transformation_offset_roundtrip() {
        let mut tr = Transformation::new();
        tr.set_offset(&Vec3d::new(5.0, -3.0, 2.0));
        let o = tr.get_offset();
        assert!(approx(o.x, 5.0) && approx(o.y, -3.0) && approx(o.z, 2.0));
    }

    #[test]
    fn test_transformation_scaling_roundtrip() {
        let mut tr = Transformation::new();
        tr.set_scaling_factor(&Vec3d::new(2.0, 3.0, 4.0));
        let s = tr.get_scaling_factor();
        assert!(approx(s.x, 2.0) && approx(s.y, 3.0) && approx(s.z, 4.0));
    }

    #[test]
    fn test_is_rotation_ninety_degrees() {
        assert!(is_rotation_ninety_degrees_angle(0.0));
        assert!(is_rotation_ninety_degrees_angle(std::f64::consts::FRAC_PI_2));
        assert!(is_rotation_ninety_degrees_angle(std::f64::consts::PI));
        assert!(!is_rotation_ninety_degrees_angle(0.5));
    }

    #[test]
    fn test_transform3d_from_string() {
        let s = "1 0 0 0 0 1 0 0 0 0 1 0 5 6 7 1";
        let t = transform3d_from_string(s);
        // C++ reads row-major into transform(r,c). So transform(3,0)=5 etc.
        assert!(approx(t[(3, 0)], 5.0));
        assert!(approx(t[(3, 1)], 6.0));
        assert!(approx(t[(3, 2)], 7.0));
    }
}
