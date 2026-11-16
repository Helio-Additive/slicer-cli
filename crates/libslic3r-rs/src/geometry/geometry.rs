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
    if transformation_eigen() {
        // Geometry.cpp:310-313
        *transform = eigen::identity();
        eigen::translate(transform, translation);
        eigen::rotate(transform, &eigen::zyx_quaternion(rotation).to_rotation_matrix());
        eigen::scale(transform, &scale.component_mul(mirror));
        return;
    }
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
    if transformation_eigen() {
        // Geometry.cpp:328-330
        let angles = eigen::euler_angles(rotation_matrix, 2, 1, 0);
        return Vec3d::new(angles.z, angles.y, angles.x);
    }
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
    if transformation_eigen() {
        // Geometry.cpp:342-344 — m.col(k).normalize()
        for col in 0..3 {
            let c = eigen::normalized3(&Vec3d::new(m[(0, col)], m[(1, col)], m[(2, col)]));
            m[(0, col)] = c.x;
            m[(1, col)] = c.y;
            m[(2, col)] = c.z;
        }
        // Geometry.cpp:345
        return extract_euler_angles_from_matrix(&m);
    }
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
    if transformation_eigen() {
        return eigen::transform_from_linear(&rotation);
    }
    make_rotation4(&rotation)
}

// Geometry.cpp:356-362
/// Geometry.cpp:356-362 `extract_rotation_scale`
fn extract_rotation_scale(trafo: &Transform3d) -> (Transform3d, Transform3d) {
    // Geometry.cpp:358-360
    let (rotation, scale) = compute_rotation_scaling(trafo);
    // Geometry.cpp:361
    if transformation_eigen() {
        return (eigen::transform_from_linear(&rotation), eigen::transform_from_linear(&scale));
    }
    (make_rotation4(&rotation), make_rotation4(&scale))
}

// Geometry.cpp:364-370
/// Geometry.cpp:364-370 `extract_scale`
fn extract_scale(trafo: &Transform3d) -> Transform3d {
    // Geometry.cpp:366-368
    let (_rotation, scale) = compute_rotation_scaling(trafo);
    // Geometry.cpp:369
    if transformation_eigen() {
        return eigen::transform_from_linear(&scale);
    }
    make_rotation4(&scale)
}

// Eigen's `Transform::computeRotationScaling(&R, &S)`: polar decomposition of
// the linear (3x3) part M = R * S, where R is a rotation (det >= 0) and S is a
// symmetric (possibly negative-definite) scaling matrix. Implemented via SVD:
// M = U Σ Vᵀ; R = U Vᵀ (with a sign correction so det(R) > 0); S = V Σ' Vᵀ where
// Σ' absorbs the sign correction.
fn compute_rotation_scaling(trafo: &Transform3d) -> (Matrix3<f64>, Matrix3<f64>) {
    let m: Matrix3<f64> = trafo.fixed_view::<3, 3>(0, 0).into();
    if transformation_eigen() {
        return eigen::compute_rotation_scaling(&m);
    }
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

    let eigen_arm = transformation_eigen();
    // Geometry.cpp:380 — if (scale.isDiagonal()) return false;
    if if eigen_arm { eigen::is_diagonal3(&scale) } else { is_diagonal(&scale) } {
        return false;
    }

    // Geometry.cpp:382 — if (scale.determinant() >= 0.0) return true;
    if (if eigen_arm { eigen::determinant3(&scale) } else { scale.determinant() }) >= 0.0 {
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
    if transformation_eigen() {
        // Geometry.cpp:399
        let m = eigen::Quaterniond::set_from_two_vectors(&from, &to).to_rotation_matrix();
        // Geometry.cpp:400
        let aa = eigen::AngleAxisd::from_rotation_matrix(&m);
        // Geometry.cpp:401-402
        *rotation_axis = aa.axis;
        *phi = aa.angle;
        // Geometry.cpp:403-404
        if let Some(rm) = rotation_matrix {
            *rm = m;
        }
        return;
    }
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
    if transformation_eigen() {
        // Geometry.cpp:410-412
        let mut transform = eigen::identity();
        eigen::translate(&mut transform, translation);
        return transform;
    }
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
    if transformation_eigen() {
        // Geometry.cpp:417-419
        let mut transform = eigen::identity();
        eigen::rotate(&mut transform, &eigen::zyx_quaternion(rotation).to_rotation_matrix());
        return transform;
    }
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
    if transformation_eigen() {
        // Geometry.cpp:428-429
        *transform = eigen::identity();
        eigen::scale(transform, scale);
        return;
    }
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

/// `Transform3d * Transform3d` on the arm of [`transformation_eigen`].
#[inline]
fn transform_product(a: &Transform3d, b: &Transform3d) -> Transform3d {
    if transformation_eigen() {
        eigen::transform_mul(a, b)
    } else {
        a * b
    }
}

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
        if transformation_eigen() {
            // Geometry.cpp:453-454
            let mut quaternion = eigen::Quaterniond::from_rotation_matrix(&rotation_matrix);
            quaternion.normalize();
            // Geometry.cpp:455-456
            let t = eigen::euler_angles(&quaternion.to_rotation_matrix(), 2, 1, 0);
            return Vec3d::new(t.z, t.y, t.x);
        }
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
        self.m_matrix = transform_product(rot_mat, &extract_scale(&self.m_matrix));
        // Geometry.cpp:466
        self.set_offset(&offset);
    }

    // Geometry.cpp:469-474 — void Transformation::set_rotation(const Vec3d &rotation)
    pub fn set_rotation(&mut self, rotation: &Vec3d) {
        // Geometry.cpp:471
        let offset = self.get_offset();
        // Geometry.cpp:472 — m_matrix = rotation_transform(rotation) * extract_scale(m_matrix);
        self.m_matrix = transform_product(&rotation_transform(rotation), &extract_scale(&self.m_matrix));
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
        self.m_matrix = transform_product(&extract_rotation_matrix(&self.m_matrix), &scale_transform(scaling_factor));
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
        self.m_matrix = transform_product(&rotation, &scale);
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
        if transformation_eigen() {
            return eigen::determinant3(&linear) < 0.0;
        }
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
        self.m_matrix = transform_product(&rotation, &scale);
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
        self.m_matrix = transform_product(&rotation, &scale);
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
        if transformation_eigen() {
            // Geometry.cpp:579
            // `Transform3d(svd.v * svd.s * svd.v.transpose())`: `v * s` into a temporary, then the
            // product assigned to the linear part through a row-major temporary
            let vsv = eigen::mat3_mul_coeff(&eigen::mat3_mul(&svd.v, &svd.s), &svd.v.transpose());
            self.m_matrix = eigen::transform_mul_diag(
                &eigen::transform_mul(&self.get_offset_matrix(), &eigen::transform_from_linear(&vsv)),
                &svd.mirror_diagonal(),
            );
            return;
        }
        // Geometry.cpp:579 — m_matrix = get_offset_matrix() * Transform3d(svd.v * svd.s * svd.v.transpose()) * svd.mirror_matrix();
        self.m_matrix = self.get_offset_matrix()
            * make_rotation4(&(svd.v * svd.s * svd.v.transpose()))
            * svd.mirror_matrix();
    }

    // Geometry.cpp:582-586 — void Transformation::reset_scaling_factor()
    pub fn reset_scaling_factor(&mut self) {
        // Geometry.cpp:584
        let svd = TransformationSVD::from_transformation(self);
        if transformation_eigen() {
            // Geometry.cpp:585
            let ou = eigen::transform_mul(&self.get_offset_matrix(), &eigen::transform_from_linear(&svd.u));
            let ouv = eigen::transform_mul(&ou, &eigen::transform_from_linear(&svd.v.transpose()));
            self.m_matrix = eigen::transform_mul_diag(&ouv, &svd.mirror_diagonal());
            return;
        }
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
        if transformation_eigen() {
            // Geometry.cpp:595
            let ou = eigen::transform_mul(&self.get_offset_matrix(), &eigen::transform_from_linear(&svd.u));
            let ous = eigen::transform_mul(&ou, &scale_transform_uniform(new_scale_factor(&svd.s)));
            let ousv = eigen::transform_mul(&ous, &eigen::transform_from_linear(&svd.v.transpose()));
            self.m_matrix = eigen::transform_mul_diag(&ousv, &svd.mirror_diagonal());
            return;
        }
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
        if transformation_eigen() {
            Self::volume_to_bed_transformation_eigen(&mut out, instance_transformation, bbox);
            return out;
        }

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

impl Transformation {
    /// Geometry.cpp:638-688 on the Eigen arm of [`transformation_eigen`].
    fn volume_to_bed_transformation_eigen(
        out: &mut Transformation,
        instance_transformation: &Transformation,
        bbox: &crate::bounding_box::BoundingBoxf3,
    ) {
        if instance_transformation.is_scaling_uniform() {
            // Geometry.cpp:641
            out.set_from_transform(eigen::transform_inverse(&instance_transformation.get_matrix(true, false, false, false)));
        } else if is_rotation_ninety_degrees(&instance_transformation.get_rotation()) {
            // Geometry.cpp:646-653
            let r = instance_transformation.get_rotation();
            let qz = eigen::Quaterniond::from_angle_axis(r.z, &unit_z());
            let qy = eigen::Quaterniond::from_angle_axis(r.y, &unit_y());
            let qx = eigen::Quaterniond::from_angle_axis(r.x, &unit_x());
            let instance_rotation_trafo = qz.mul(&qy).mul(&qx).to_rotation_matrix();
            let r = instance_transformation.get_rotation();
            let qx = eigen::Quaterniond::from_angle_axis(-r.x, &unit_x());
            let qy = eigen::Quaterniond::from_angle_axis(-r.y, &unit_y());
            let qz = eigen::Quaterniond::from_angle_axis(-r.z, &unit_z());
            let volume_rotation_trafo = qx.mul(&qy).mul(&qz).to_rotation_matrix();
            // Geometry.cpp:656-664 — 8 corners of the bounding box.
            let pts: [[f64; 3]; 8] = [
                [bbox.min.x, bbox.min.y, bbox.min.z],
                [bbox.min.x, bbox.min.y, bbox.max.z],
                [bbox.min.x, bbox.max.y, bbox.min.z],
                [bbox.min.x, bbox.max.y, bbox.max.z],
                [bbox.max.x, bbox.min.y, bbox.min.z],
                [bbox.max.x, bbox.min.y, bbox.max.z],
                [bbox.max.x, bbox.max.y, bbox.min.z],
                [bbox.max.x, bbox.max.y, bbox.max.z],
            ];
            // Geometry.cpp:667-670 — qs = pts * (rot * Scaling(sf .* mirror) * vrot).inverse().transpose(),
            // an 8x3 product of a dynamic-size matrix, every row in a packet.
            let sm = instance_transformation.get_scaling_factor().component_mul(&instance_transformation.get_mirror());
            let p = eigen::mat3_mul(&eigen::mat3_mul_diag(&instance_rotation_trafo, &sm), &volume_rotation_trafo);
            let rt = eigen::mat3_inverse(&p).transpose();
            let mut qs = [[0.0_f64; 3]; 8];
            for (r, pt) in pts.iter().enumerate() {
                for c in 0..3 {
                    qs[r][c] = eigen::packet_dot3(pt[0], pt[1], pt[2], rt[(0, c)], rt[(1, c)], rt[(2, c)]);
                }
            }
            // Geometry.cpp:672-674 — pts.col(i).dot(qs.col(i)) / pts.col(i).dot(pts.col(i)): inner
            // products of size 8 (InnerProduct.h, four packets added as p0 + (p1 + (p2 + p3)))
            let dot8 = |a: &dyn Fn(usize) -> f64, b: &dyn Fn(usize) -> f64| -> f64 {
                let e = |k: usize| a(k) * b(k);
                let lane0 = e(0) + (e(2) + (e(4) + e(6)));
                let lane1 = e(1) + (e(3) + (e(5) + e(7)));
                lane0 + lane1
            };
            let mut scale = Vec3d::zeros();
            for i in 0..3 {
                let pc = |k: usize| pts[k][i];
                let qc = |k: usize| qs[k][i];
                scale[i] = dot8(&pc, &qc) / dot8(&pc, &pc);
            }
            // Geometry.cpp:676-678
            out.set_rotation(&extract_euler_angles_from_matrix(&volume_rotation_trafo));
            out.set_scaling_factor(&Vec3d::new(scale[0].abs(), scale[1].abs(), scale[2].abs()));
            out.set_mirror(&Vec3d::new(
                if scale[0] > 0.0 { 1.0 } else { -1.0 },
                if scale[1] > 0.0 { 1.0 } else { -1.0 },
                if scale[2] > 0.0 { 1.0 } else { -1.0 },
            ));
        } else {
            // Geometry.cpp:685-686
            let scling_facor = instance_transformation.get_scaling_factor();
            out.set_scaling_factor(&Vec3d::new(1.0 / scling_facor.x, 1.0 / scling_facor.y, 1.0 / scling_facor.z));
        }
    }
}

// Geometry.cpp:630 — Transformation Transformation::operator*(const Transformation &other) const
impl std::ops::Mul for Transformation {
    type Output = Transformation;
    fn mul(self, other: Transformation) -> Transformation {
        // { return Transformation(get_matrix() * other.get_matrix()); }
        Transformation::from_transform(transform_product(&self.matrix(), &other.matrix()))
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
        if transformation_eigen() {
            return Self::from_transform_eigen(trafo);
        }
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

    /// Geometry.cpp:744-786 on the Eigen arm of [`transformation_eigen`].
    fn from_transform_eigen(trafo: &Transform3d) -> Self {
        // Geometry.cpp:746-747
        let m0 = eigen::linear(trafo);
        let mirror = eigen::determinant3(&m0) < 0.0;
        // Geometry.cpp:749-753
        let m = if mirror { eigen::mat3_mul_diag(&m0, &Vec3d::new(-1.0, 1.0, 1.0)) } else { m0 };
        // Geometry.cpp:754-757
        let (u, v, sv) = eigen::jacobi_svd3(&m);
        let s = Matrix3::from_diagonal(&sv);
        // Geometry.cpp:759-761
        let scale = !eigen::is_approx3(&s, &Matrix3::identity());
        let anisotropic_scale = !is_approx(s[(0, 0)], s[(1, 1)]) || !is_approx(s[(1, 1)], s[(2, 2)]);
        let rotation = !eigen::is_approx3(&v, &u);
        let mut rotation_90_degrees = false;
        let skew;
        // Geometry.cpp:763
        if anisotropic_scale {
            // Geometry.cpp:764-773
            rotation_90_degrees = true;
            for i in 0..3 {
                let row = Vec3d::new(v[(i, 0)].abs(), v[(i, 1)].abs(), v[(i, 2)].abs());
                let num_zeros =
                    (is_approx(row[0], 0.0) as usize) + (is_approx(row[1], 0.0) as usize) + (is_approx(row[2], 0.0) as usize);
                let num_ones =
                    (is_approx(row[0], 1.0) as usize) + (is_approx(row[1], 1.0) as usize) + (is_approx(row[2], 1.0) as usize);
                if num_zeros != 2 || num_ones != 1 {
                    rotation_90_degrees = false;
                    break;
                }
            }
            // Geometry.cpp:775-780
            let trafo_linear = eigen::linear(trafo);
            let axes = [unit_x(), unit_y(), unit_z()];
            let mut transformed_axes = [Vec3d::zeros(); 3];
            for i in 0..3 {
                transformed_axes[i] = eigen::mat3_mul_vec(&trafo_linear, &axes[i]);
            }
            skew = eigen::dot3(&transformed_axes[0], &transformed_axes[1]).abs() > EPSILON
                || eigen::dot3(&transformed_axes[1], &transformed_axes[2]).abs() > EPSILON
                || eigen::dot3(&transformed_axes[2], &transformed_axes[0]).abs() > EPSILON;
        } else {
            // Geometry.cpp:785
            skew = false;
        }
        TransformationSVD { u, s, v, mirror, scale, anisotropic_scale, rotation, rotation_90_degrees, skew }
    }

    /// The diagonal of `mirror_matrix()`.
    pub fn mirror_diagonal(&self) -> Vec3d {
        Vec3d::new(if self.mirror { -1.0 } else { 1.0 }, 1.0, 1.0)
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
    if transformation_eigen() {
        let q = rotation_xyz_diff_eigen(rot_xyz_from, rot_xyz_to);
        return UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(q.w, q.x, q.y, q.z));
    }
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

/// Geometry.cpp:720-727 `rotation_xyz_diff` as Eigen evaluates it: the product of six angle-axis
/// rotations is a chain of quaternion products, left to right.
pub fn rotation_xyz_diff_eigen(rot_xyz_from: &Vec3d, rot_xyz_to: &Vec3d) -> eigen::Quaterniond {
    let q = |a: f64, axis: Vec3d| eigen::Quaterniond::from_angle_axis(a, &axis);
    q(rot_xyz_to.z, unit_z())
        .mul(&q(rot_xyz_to.y, unit_y()))
        .mul(&q(rot_xyz_to.x, unit_x()))
        .mul(&q(-rot_xyz_from.x, unit_x()))
        .mul(&q(-rot_xyz_from.y, unit_y()))
        .mul(&q(-rot_xyz_from.z, unit_z()))
}

// Geometry.cpp:729-742
// This should only be called if it is known, that the two rotations only differ in rotation around the Z axis.
/// Geometry.cpp:730-742 `rotation_diff_z`
pub fn rotation_diff_z(rot_xyz_from: &Vec3d, rot_xyz_to: &Vec3d) -> f64 {
    if transformation_eigen() {
        // Geometry.cpp:732-734
        let angle_axis = eigen::AngleAxisd::from_quaternion(&rotation_xyz_diff_eigen(rot_xyz_from, rot_xyz_to));
        let (axis, angle) = (angle_axis.axis, angle_axis.angle);
        // Geometry.cpp:741
        return if axis.z < 0.0 { -angle } else { angle };
    }
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
    if transformation_eigen() {
        return mat_around_a_point_rotate_eigen(in_mat, pt, axis, rotate_theta_radian);
    }
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

/// Geometry.cpp:788-813 on the Eigen arm of [`transformation_eigen`].
fn mat_around_a_point_rotate_eigen(in_mat: &Transformation, pt: &Vec3d, axis: &Vec3d, rotate_theta_radian: f32) -> Transformation {
    // Geometry.cpp:790-793
    let xyz = in_mat.get_offset();
    let mut left = Transformation::new();
    left.set_offset(&(-xyz)); // at world origin
    let mut cur_mat = left * *in_mat;
    // Geometry.cpp:795-799
    let mut qua = eigen::Quaterniond::from_angle_axis(rotate_theta_radian as f64, axis);
    qua.normalize();
    let mut rotate_mat4 = Transformation::new();
    rotate_mat4.set_from_transform(from_position_orientation_scale_eigen(&Vec3d::zeros(), &qua, &Vec3d::new(1.0, 1.0, 1.0)));
    // Geometry.cpp:801
    cur_mat = rotate_mat4 * cur_mat; // along_fix_axis
    // Geometry.cpp:803-807
    let mut temp_world = Transformation::new();
    let mut qua_world = eigen::Quaterniond::from_angle_axis(0.0, axis);
    qua_world.normalize();
    temp_world.set_from_transform(from_position_orientation_scale_eigen(pt, &qua_world, &Vec3d::new(1.0, 1.0, 1.0)));
    // Geometry.cpp:808-810
    let temp_xyz = eigen::transform_mul_vec(&eigen::transform_inverse(&temp_world.matrix()), &xyz);
    let new_pos = eigen::transform_mul_vec(&temp_world.matrix(), &eigen::transform_mul_vec(&rotate_mat4.matrix(), &temp_xyz));
    cur_mat.set_offset(&new_pos);
    // Geometry.cpp:812
    cur_mat
}

/// Eigen `Transform::fromPositionOrientationScale(position, orientation, scale)` (Transform.h
/// 1105-1115): `linear() = toRotationMatrix(orientation); linear() *= scale.asDiagonal();
/// translation() = position; makeAffine()`.
fn from_position_orientation_scale_eigen(pos: &Vec3d, q: &eigen::Quaterniond, scale: &Vec3d) -> Transform3d {
    let l = eigen::mat3_mul_diag(&q.to_rotation_matrix(), scale);
    let mut t = eigen::transform_from_linear(&l);
    t[(0, 3)] = pos.x;
    t[(1, 3)] = pos.y;
    t[(2, 3)] = pos.z;
    t
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
    if transformation_eigen() {
        // Geometry.cpp:816-823
        let mut m = Matrix3::<f64>::zeros();
        m.column_mut(0).copy_from(&eigen::normalized3(x_dir));
        m.column_mut(1).copy_from(&eigen::normalized3(y_dir));
        m.column_mut(2).copy_from(&eigen::normalized3(z_dir));
        let mut tran = Transformation::from_transform(eigen::transform_from_linear(&m));
        tran.set_offset(origin);
        return tran;
    }
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

/// The gate of the Eigen arithmetic of the transformation functions of this file (faithful, on
/// unless `TRANSFORMATION_EIGEN=0`): on, `Transformation`, `TransformationSVD`, the transform
/// builders and decompositions go through [`eigen`] (Eigen 5.0.1's algorithms in the C++ build's
/// order of evaluation); off, through nalgebra (its SVD and its 4x4 products), the earlier arm.
#[inline]
pub fn transformation_eigen() -> bool {
    crate::faithful_gate("TRANSFORMATION_EIGEN")
}

/// Eigen 5.0.1 arithmetic as the C++ build evaluates it: clang -O3 with `-ffp-contract=on` (a
/// product and a sum of one expression fuse, nothing else), on arm64 with NEON packets of two
/// doubles. The operations Geometry.cpp's transformation functions are made of, each with the
/// Eigen source it follows; checked against the pin by
/// `tests/geometry_transformation_native_parity.rs`.
///
/// Products of small fixed-size matrices are Eigen's lazy coefficient-based products
/// (ProductEvaluators.h): where the destination is evaluated two rows at a time (a packet), a row
/// is `pmul` then `pmadd`, `fma(a2, x2, fma(a1, x1, a0 * x0))` ([`packet_dot3`]); a row evaluated
/// alone is `coeff()`, the unrolled sum of the products `e0 + (e1 + e2)` ([`coeff_dot3`]). A 3x3
/// destination is two rows in a packet and the third alone (SliceVectorizedTraversal); a 4-row
/// destination (`Transform3d * Vec3d`, which multiplies the 4x4 matrix by `(x, y, z, 1)`) is two
/// packets.
pub mod eigen {
    use nalgebra::{Matrix3, Matrix4, Vector3};

    pub type Matrix3d = Matrix3<f64>;
    pub type Vec3d = Vector3<f64>;
    pub type Transform3d = Matrix4<f64>;

    /// `std::max(a, b)`: `(a < b) ? b : a`.
    #[inline]
    pub fn maxi(a: f64, b: f64) -> f64 {
        if a < b {
            b
        } else {
            a
        }
    }

    /// `std::min(a, b)`: `(b < a) ? b : a`.
    #[inline]
    pub fn mini(a: f64, b: f64) -> f64 {
        if b < a {
            b
        } else {
            a
        }
    }

    /// One row of a lazy product evaluated in a packet (ProductEvaluators.h
    /// `etor_product_packet_impl<ColMajor>`): `pmul` then `pmadd` (vfmaq_f64).
    #[inline]
    pub fn packet_dot3(a0: f64, a1: f64, a2: f64, x0: f64, x1: f64, x2: f64) -> f64 {
        a2.mul_add(x2, a1.mul_add(x1, a0 * x0))
    }

    /// One coefficient of a lazy product evaluated alone (ProductEvaluators.h `coeff()`, the redux
    /// of Redux.h `redux_novec_unroller`): `e0 + (e1 + e2)`, unfused.
    #[inline]
    pub fn coeff_dot3(a0: f64, a1: f64, a2: f64, x0: f64, x1: f64, x2: f64) -> f64 {
        a0 * x0 + (a1 * x1 + a2 * x2)
    }

    /// `Matrix3d * Matrix3d` (also `Matrix3d * Matrix3d.transpose()`: pass the transpose).
    pub fn mat3_mul(a: &Matrix3d, b: &Matrix3d) -> Matrix3d {
        let mut r = Matrix3d::zeros();
        for c in 0..3 {
            for row in 0..2 {
                r[(row, c)] = packet_dot3(a[(row, 0)], a[(row, 1)], a[(row, 2)], b[(0, c)], b[(1, c)], b[(2, c)]);
            }
            r[(2, c)] = coeff_dot3(a[(2, 0)], a[(2, 1)], a[(2, 2)], b[(0, c)], b[(1, c)], b[(2, c)]);
        }
        r
    }

    /// A 3x3 product evaluated one coefficient at a time ([`coeff_dot3`] for every coefficient):
    /// the product assigned through a temporary of the opposite storage order, as Eigen does for
    /// `(a * b) * c.transpose()` assigned to a block (Product.h: a product whose left side is a
    /// product and whose right side is row-major is row-major; the lazy product evaluates column
    /// major, so the assignment does not vectorize).
    pub fn mat3_mul_coeff(a: &Matrix3d, b: &Matrix3d) -> Matrix3d {
        let mut r = Matrix3d::zeros();
        for c in 0..3 {
            for row in 0..3 {
                r[(row, c)] = coeff_dot3(a[(row, 0)], a[(row, 1)], a[(row, 2)], b[(0, c)], b[(1, c)], b[(2, c)]);
            }
        }
        r
    }

    /// `Matrix3d * Vec3d`.
    pub fn mat3_mul_vec(m: &Matrix3d, v: &Vec3d) -> Vec3d {
        Vec3d::new(
            packet_dot3(m[(0, 0)], m[(0, 1)], m[(0, 2)], v.x, v.y, v.z),
            packet_dot3(m[(1, 0)], m[(1, 1)], m[(1, 2)], v.x, v.y, v.z),
            coeff_dot3(m[(2, 0)], m[(2, 1)], m[(2, 2)], v.x, v.y, v.z),
        )
    }

    /// `Matrix3d * DiagonalMatrix` (DiagonalProduct: `m(i, j) * d(j)`).
    pub fn mat3_mul_diag(m: &Matrix3d, d: &Vec3d) -> Matrix3d {
        let mut r = *m;
        for c in 0..3 {
            for row in 0..3 {
                r[(row, c)] = m[(row, c)] * d[c];
            }
        }
        r
    }

    /// Eigen/src/LU/Determinant.h:21-24, 47-51: `bruteforce_det3_helper(m,0,1,2) -
    /// bruteforce_det3_helper(m,1,0,2) + bruteforce_det3_helper(m,2,0,1)`, each helper
    /// `m(0,a) * (m(1,b) * m(2,c) - m(1,c) * m(2,b))` with the difference fused.
    pub fn determinant3(m: &Matrix3d) -> f64 {
        let helper = |a: usize, b: usize, c: usize| m[(0, a)] * m[(1, b)].mul_add(m[(2, c)], -(m[(1, c)] * m[(2, b)]));
        helper(0, 1, 2) - helper(1, 0, 2) + helper(2, 0, 1)
    }

    /// Eigen/src/LU/InverseImpl.h `compute_inverse<MatrixType, ResultType, 3>`: cofactors
    /// (`cofactor_3x3`, the difference fused), `det = cofactors_col0.cwiseProduct(col(0)).sum()`
    /// (a packet of two products, then the third added), `invdet = 1 / det`.
    pub fn mat3_inverse(m: &Matrix3d) -> Matrix3d {
        let cofactor = |i: usize, j: usize| {
            let (i1, i2, j1, j2) = ((i + 1) % 3, (i + 2) % 3, (j + 1) % 3, (j + 2) % 3);
            m[(i1, j1)].mul_add(m[(i2, j2)], -(m[(i1, j2)] * m[(i2, j1)]))
        };
        let c0 = [cofactor(0, 0), cofactor(1, 0), cofactor(2, 0)];
        let det = (c0[0] * m[(0, 0)] + c0[1] * m[(1, 0)]) + c0[2] * m[(2, 0)];
        let invdet = 1.0 / det;
        let mut inv = Matrix3d::zeros();
        let c01 = cofactor(0, 1) * invdet;
        let c11 = cofactor(1, 1) * invdet;
        let c02 = cofactor(0, 2) * invdet;
        inv[(1, 2)] = cofactor(2, 1) * invdet;
        inv[(2, 1)] = cofactor(1, 2) * invdet;
        inv[(2, 2)] = cofactor(2, 2) * invdet;
        inv[(1, 0)] = c01;
        inv[(1, 1)] = c11;
        inv[(2, 0)] = c02;
        inv[(0, 0)] = c0[0] * invdet;
        inv[(0, 1)] = c0[1] * invdet;
        inv[(0, 2)] = c0[2] * invdet;
        inv
    }

    /// The linear part of an affine transform.
    #[inline]
    pub fn linear(t: &Transform3d) -> Matrix3d {
        t.fixed_view::<3, 3>(0, 0).into_owned()
    }

    /// The translation of an affine transform.
    #[inline]
    pub fn translation(t: &Transform3d) -> Vec3d {
        Vec3d::new(t[(0, 3)], t[(1, 3)], t[(2, 3)])
    }

    /// `Transform::makeAffine()`: the last row `(0, 0, 0, 1)`.
    #[inline]
    pub fn make_affine(t: &mut Transform3d) {
        t[(3, 0)] = 0.0;
        t[(3, 1)] = 0.0;
        t[(3, 2)] = 0.0;
        t[(3, 3)] = 1.0;
    }

    /// `Transform3d(const Matrix3d &)` (Transform.h `transform_construct_from_matrix<..., Dim,
    /// Dim>`): the linear part, a zero translation, `makeAffine()`.
    pub fn transform_from_linear(m: &Matrix3d) -> Transform3d {
        let mut t = Transform3d::zeros();
        t.fixed_view_mut::<3, 3>(0, 0).copy_from(m);
        make_affine(&mut t);
        t
    }

    /// `Transform * Transform` of two affine transforms (Transform.h
    /// `transform_transform_product_impl`): `res.linear() = lhs.linear() * rhs.linear();
    /// res.translation() = lhs.linear() * rhs.translation() + lhs.translation(); res.makeAffine()`.
    /// The product of the translation is evaluated before the sum.
    pub fn transform_mul(a: &Transform3d, b: &Transform3d) -> Transform3d {
        let mut r = Transform3d::zeros();
        for c in 0..4 {
            for row in 0..2 {
                r[(row, c)] = packet_dot3(a[(row, 0)], a[(row, 1)], a[(row, 2)], b[(0, c)], b[(1, c)], b[(2, c)]);
            }
            r[(2, c)] = coeff_dot3(a[(2, 0)], a[(2, 1)], a[(2, 2)], b[(0, c)], b[(1, c)], b[(2, c)]);
        }
        for row in 0..3 {
            r[(row, 3)] += a[(row, 3)];
        }
        make_affine(&mut r);
        r
    }

    /// `Transform * DiagonalMatrix` (Transform.h `operator*(const DiagonalBase &)`: `res.linearExt()
    /// *= b`, the columns of the linear part scaled).
    pub fn transform_mul_diag(t: &Transform3d, d: &Vec3d) -> Transform3d {
        let mut r = *t;
        for c in 0..3 {
            for row in 0..3 {
                r[(row, c)] = t[(row, c)] * d[c];
            }
        }
        r
    }

    /// `Transform3d * Vec3d` (Transform.h `transform_right_product_impl<..., 2, 1>`: the 4x4 matrix
    /// times `(x, y, z, 1)`, two packets of two rows; the last term `t * 1` adds the translation).
    pub fn transform_mul_vec(t: &Transform3d, v: &Vec3d) -> Vec3d {
        let row = |r: usize| t[(r, 3)].mul_add(1.0, packet_dot3(t[(r, 0)], t[(r, 1)], t[(r, 2)], v.x, v.y, v.z));
        Vec3d::new(row(0), row(1), row(2))
    }

    /// `Transform3d::inverse()` of an affine transform (Transform.h 1171-1192, `hint & Affine`): the
    /// linear part inverted ([`mat3_inverse`]), the translation `-linear_inv * translation()`, the
    /// negated block times the translation (the signs of zero products follow the negated block).
    pub fn transform_inverse(t: &Transform3d) -> Transform3d {
        let inv = mat3_inverse(&linear(t));
        let mut r = transform_from_linear(&inv);
        let tr = translation(t);
        let n = -inv;
        for row in 0..2 {
            r[(row, 3)] = packet_dot3(n[(row, 0)], n[(row, 1)], n[(row, 2)], tr.x, tr.y, tr.z);
        }
        r[(2, 3)] = coeff_dot3(n[(2, 0)], n[(2, 1)], n[(2, 2)], tr.x, tr.y, tr.z);
        r
    }

    /// `Transform::translate(v)` (Transform.h: `translationExt() += linearExt() * other`).
    pub fn translate(t: &mut Transform3d, v: &Vec3d) {
        let p0 = packet_dot3(t[(0, 0)], t[(0, 1)], t[(0, 2)], v.x, v.y, v.z);
        let p1 = packet_dot3(t[(1, 0)], t[(1, 1)], t[(1, 2)], v.x, v.y, v.z);
        let p2 = coeff_dot3(t[(2, 0)], t[(2, 1)], t[(2, 2)], v.x, v.y, v.z);
        t[(0, 3)] += p0;
        t[(1, 3)] += p1;
        t[(2, 3)] += p2;
    }

    /// `Transform::rotate(rotation)` with the rotation as a matrix (Transform.h: `linearExt() *=
    /// toRotationMatrix(rotation)`, the product evaluated into a temporary).
    pub fn rotate(t: &mut Transform3d, rot: &Matrix3d) {
        let l = mat3_mul(&linear(t), rot);
        t.fixed_view_mut::<3, 3>(0, 0).copy_from(&l);
    }

    /// `Transform::scale(const Vec3d &)` (Transform.h: `linearExt().noalias() = linearExt() *
    /// other.asDiagonal()`).
    pub fn scale(t: &mut Transform3d, s: &Vec3d) {
        for c in 0..3 {
            for row in 0..3 {
                t[(row, c)] *= s[c];
            }
        }
    }

    /// `Transform3d::Identity()`.
    #[inline]
    pub fn identity() -> Transform3d {
        Transform3d::identity()
    }

    /// `Vec3d::squaredNorm()` (Dot.h `squared_norm_impl`: the redux of the squares, Redux.h
    /// LinearVectorizedTraversal: a packet of the first two, `predux`, then the third added).
    #[inline]
    pub fn squared_norm3(v: &Vec3d) -> f64 {
        (v.x * v.x + v.y * v.y) + v.z * v.z
    }

    /// `Vec3d::norm()`.
    #[inline]
    pub fn norm3(v: &Vec3d) -> f64 {
        squared_norm3(v).sqrt()
    }

    /// `Vec3d::normalized()` (Dot.h: `n / sqrt(z)` when `z > 0`, else a copy).
    pub fn normalized3(v: &Vec3d) -> Vec3d {
        let z = squared_norm3(v);
        if z > 0.0 {
            let s = z.sqrt();
            Vec3d::new(v.x / s, v.y / s, v.z / s)
        } else {
            *v
        }
    }

    /// `Vec3d::normalize()`.
    pub fn normalize3(v: &mut Vec3d) {
        *v = normalized3(v);
    }

    /// `a.dot(b)` of two `Vec3d` (InnerProduct.h `inner_product_impl<Evaluator, true>`: a packet of
    /// the first two products, `predux`, then `pmadd` of the third).
    #[inline]
    pub fn dot3(a: &Vec3d, b: &Vec3d) -> f64 {
        a.z.mul_add(b.z, a.x * b.x + a.y * b.y)
    }

    /// `a.cross(b)` (OrthoMethods.h `cross_impl<..., 3>`, each difference fused).
    #[inline]
    pub fn cross3(a: &Vec3d, b: &Vec3d) -> Vec3d {
        Vec3d::new(
            a.y.mul_add(b.z, -(a.z * b.y)),
            a.z.mul_add(b.x, -(a.x * b.z)),
            a.x.mul_add(b.y, -(a.y * b.x)),
        )
    }

    /// `m.trace()` of a 3x3 matrix (the redux of the diagonal, not packet accessible:
    /// `d0 + (d1 + d2)`).
    #[inline]
    pub fn trace3(m: &Matrix3d) -> f64 {
        m[(0, 0)] + (m[(1, 1)] + m[(2, 2)])
    }

    /// The sum of the squares of the nine coefficients of a 3x3 matrix (Redux.h
    /// LinearVectorizedTraversal over 9: four packets added as `(p0 + p1) + (p2 + p3)`, `predux`,
    /// then the ninth added).
    fn sum_abs2_9(e: &[f64; 9]) -> f64 {
        let s = |k: usize| e[k] * e[k];
        let lane0 = (s(0) + s(2)) + (s(4) + s(6));
        let lane1 = (s(1) + s(3)) + (s(5) + s(7));
        (lane0 + lane1) + s(8)
    }

    /// `a.isApprox(b)` of two 3x3 matrices with the default precision (Fuzzy.h
    /// `isApprox_selector`): `(a - b).cwiseAbs2().sum() <= prec * prec * min(a.cwiseAbs2().sum(),
    /// b.cwiseAbs2().sum())`, prec = 1e-12.
    pub fn is_approx3(a: &Matrix3d, b: &Matrix3d) -> bool {
        let prec = 1e-12;
        let mut d = [0.0; 9];
        let mut ea = [0.0; 9];
        let mut eb = [0.0; 9];
        for k in 0..9 {
            // column-major linear index
            let (r, c) = (k % 3, k / 3);
            d[k] = a[(r, c)] - b[(r, c)];
            ea[k] = a[(r, c)];
            eb[k] = b[(r, c)];
        }
        sum_abs2_9(&d) <= prec * prec * mini(sum_abs2_9(&ea), sum_abs2_9(&eb))
    }

    /// `m.isDiagonal()` with the default precision (Eigen/src/Core/Fuzzy.h, MatrixBase::isDiagonal):
    /// every off-diagonal coefficient much smaller than the largest diagonal one, `|x| <= |y| *
    /// prec`, prec = 1e-12.
    pub fn is_diagonal3(m: &Matrix3d) -> bool {
        let prec = 1e-12;
        let mut max_abs_on_diagonal = -1.0;
        for j in 0..3 {
            let abs_on_diagonal = m[(j, j)].abs();
            if abs_on_diagonal > max_abs_on_diagonal {
                max_abs_on_diagonal = abs_on_diagonal;
            }
        }
        for j in 0..3 {
            for i in 0..j {
                if !(m[(i, j)].abs() <= max_abs_on_diagonal.abs() * prec) {
                    return false;
                }
                if !(m[(j, i)].abs() <= max_abs_on_diagonal.abs() * prec) {
                    return false;
                }
            }
        }
        true
    }

    /// Eigen's `numext::hypot` (MathFunctionsImpl.h `positive_real_hypot`): `p * sqrt(1 + qp * qp)`
    /// with the sum fused.
    pub fn hypot(x: f64, y: f64) -> f64 {
        let (x, y) = (x.abs(), y.abs());
        if x.is_infinite() || y.is_infinite() {
            return f64::INFINITY;
        }
        if x.is_nan() || y.is_nan() {
            return f64::NAN;
        }
        let p = maxi(x, y);
        if p == 0.0 {
            return 0.0;
        }
        let qp = mini(y, x) / p;
        p * qp.mul_add(qp, 1.0).sqrt()
    }

    /// `MatrixBase::eulerAngles(a0, a1, a2)` of a 3x3 matrix (Eigen/src/Geometry/EulerAngles.h).
    pub fn euler_angles(m: &Matrix3d, a0: usize, a1: usize, a2: usize) -> Vec3d {
        let pi = std::f64::consts::PI;
        let mut res = [0.0_f64; 3];
        let odd = if (a0 + 1) % 3 == a1 { 0 } else { 1 };
        let i = a0;
        let j = (a0 + 1 + odd) % 3;
        let k = (a0 + 2 - odd) % 3;
        let odd = odd != 0;
        if a0 == a2 {
            res[0] = m[(j, i)].atan2(m[(k, i)]);
            if (odd && res[0] < 0.0) || (!odd && res[0] > 0.0) {
                if res[0] > 0.0 {
                    res[0] -= pi;
                } else {
                    res[0] += pi;
                }
                let s2 = hypot(m[(j, i)], m[(k, i)]);
                res[1] = -s2.atan2(m[(i, i)]);
            } else {
                let s2 = hypot(m[(j, i)], m[(k, i)]);
                res[1] = s2.atan2(m[(i, i)]);
            }
            let s1 = res[0].sin();
            let c1 = res[0].cos();
            res[2] = c1.mul_add(m[(j, k)], -(s1 * m[(k, k)])).atan2(c1.mul_add(m[(j, j)], -(s1 * m[(k, j)])));
        } else {
            res[0] = m[(j, k)].atan2(m[(k, k)]);
            let c2 = hypot(m[(i, i)], m[(i, j)]);
            if (odd && res[0] < 0.0) || (!odd && res[0] > 0.0) {
                if res[0] > 0.0 {
                    res[0] -= pi;
                } else {
                    res[0] += pi;
                }
                res[1] = (-m[(i, k)]).atan2(-c2);
            } else {
                res[1] = (-m[(i, k)]).atan2(c2);
            }
            let s1 = res[0].sin();
            let c1 = res[0].cos();
            res[2] = s1.mul_add(m[(k, i)], -(c1 * m[(j, i)])).atan2(c1.mul_add(m[(j, j)], -(s1 * m[(k, j)])));
        }
        if !odd {
            for r in res.iter_mut() {
                *r = -*r;
            }
        }
        Vec3d::new(res[0], res[1], res[2])
    }

    /// `Eigen::Quaterniond`, its coefficients in Eigen's order (x, y, z, w).
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct Quaterniond {
        pub x: f64,
        pub y: f64,
        pub z: f64,
        pub w: f64,
    }

    impl Quaterniond {
        pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
            Quaterniond { x, y, z, w }
        }

        pub fn identity() -> Self {
            Quaterniond { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
        }

        /// `Quaterniond(AngleAxisd(angle, axis))` (Quaternion.h `QuaternionBase::operator=(const
        /// AngleAxisType &)`): `w = cos(angle / 2)`, `vec = sin(angle / 2) * axis`.
        pub fn from_angle_axis(angle: f64, axis: &Vec3d) -> Self {
            let ha = 0.5 * angle;
            let s = ha.sin();
            Quaterniond { w: ha.cos(), x: s * axis.x, y: s * axis.y, z: s * axis.z }
        }

        /// `Quaterniond(const Matrix3d &)` (Quaternion.h `quaternionbase_assign_impl<Other, 3, 3>`,
        /// Shoemake's algorithm).
        pub fn from_rotation_matrix(mat: &Matrix3d) -> Self {
            let mut q = [0.0_f64; 4]; // x y z w
            let mut t = trace3(mat);
            if t > 0.0 {
                t = (t + 1.0).sqrt();
                q[3] = 0.5 * t;
                t = 0.5 / t;
                q[0] = (mat[(2, 1)] - mat[(1, 2)]) * t;
                q[1] = (mat[(0, 2)] - mat[(2, 0)]) * t;
                q[2] = (mat[(1, 0)] - mat[(0, 1)]) * t;
            } else {
                let mut i = 0;
                if mat[(1, 1)] > mat[(0, 0)] {
                    i = 1;
                }
                if mat[(2, 2)] > mat[(i, i)] {
                    i = 2;
                }
                let j = (i + 1) % 3;
                let k = (j + 1) % 3;
                t = (mat[(i, i)] - mat[(j, j)] - mat[(k, k)] + 1.0).sqrt();
                q[i] = 0.5 * t;
                t = 0.5 / t;
                q[3] = (mat[(k, j)] - mat[(j, k)]) * t;
                q[j] = (mat[(j, i)] + mat[(i, j)]) * t;
                q[k] = (mat[(k, i)] + mat[(i, k)]) * t;
            }
            Quaterniond { x: q[0], y: q[1], z: q[2], w: q[3] }
        }

        /// `a * b` (Geometry/arch/Geometry_SIMD.h `quat_product<Architecture::Target, ..., double>`,
        /// the arm64 / SSE specialization: packets, nothing fused).
        pub fn mul(&self, b: &Quaterniond) -> Quaterniond {
            let a = self;
            // t1 = ww*xy + yy*zw; t2 = zz*xy - xx*zw; res.xy = t1 +/- swap(t2)
            let t1_0 = a.w * b.x + a.y * b.z;
            let t1_1 = a.w * b.y + a.y * b.w;
            let t2_0 = a.z * b.x - a.x * b.z;
            let t2_1 = a.z * b.y - a.x * b.w;
            let x = t1_0 - t2_1;
            let y = t1_1 + t2_0;
            // t1 = ww*zw - yy*xy; t2 = zz*zw + xx*xy; res.zw = swap(swap(t1) +/- t2)
            let u1_0 = a.w * b.z - a.y * b.x;
            let u1_1 = a.w * b.w - a.y * b.y;
            let u2_0 = a.z * b.z + a.x * b.x;
            let u2_1 = a.z * b.w + a.x * b.y;
            let z = u1_0 + u2_1;
            let w = u1_1 - u2_0;
            Quaterniond { x, y, z, w }
        }

        /// `toRotationMatrix()` (Quaternion.h).
        pub fn to_rotation_matrix(&self) -> Matrix3d {
            let tx = 2.0 * self.x;
            let ty = 2.0 * self.y;
            let tz = 2.0 * self.z;
            let twx = tx * self.w;
            let twy = ty * self.w;
            let twz = tz * self.w;
            let txx = tx * self.x;
            let txy = ty * self.x;
            let txz = tz * self.x;
            let tyy = ty * self.y;
            let tyz = tz * self.y;
            let tzz = tz * self.z;
            let mut res = Matrix3d::zeros();
            res[(0, 0)] = 1.0 - (tyy + tzz);
            res[(0, 1)] = txy - twz;
            res[(0, 2)] = txz + twy;
            res[(1, 0)] = txy + twz;
            res[(1, 1)] = 1.0 - (txx + tzz);
            res[(1, 2)] = tyz - twx;
            res[(2, 0)] = txz - twy;
            res[(2, 1)] = tyz + twx;
            res[(2, 2)] = 1.0 - (txx + tyy);
            res
        }

        /// `coeffs().squaredNorm()` of the four coefficients (Redux.h LinearVectorizedTraversal:
        /// two packets added, then `predux`).
        pub fn squared_norm(&self) -> f64 {
            (self.x * self.x + self.z * self.z) + (self.y * self.y + self.w * self.w)
        }

        /// `normalize()` (Dot.h `MatrixBase::normalize`: divided by the norm when the squared norm
        /// is positive).
        pub fn normalize(&mut self) {
            let z = self.squared_norm();
            if z > 0.0 {
                let s = z.sqrt();
                self.x /= s;
                self.y /= s;
                self.z /= s;
                self.w /= s;
            }
        }

        /// `vec()`.
        pub fn vec(&self) -> Vec3d {
            Vec3d::new(self.x, self.y, self.z)
        }

        /// `setFromTwoVectors(a, b)` (Quaternion.h 690-731). The branch of nearly opposite vectors
        /// takes the axis from a 2x3 JacobiSVD with a column-pivoting QR preconditioner
        /// ([`jacobi_svd_2x3_v_col2`]).
        pub fn set_from_two_vectors(a: &Vec3d, b: &Vec3d) -> Self {
            let v0 = normalized3(a);
            let v1 = normalized3(b);
            let mut c = dot3(&v1, &v0);
            // if dot == -1, vectors are nearly opposites
            if c < -1.0 + 1e-12 {
                c = maxi(c, -1.0);
                let axis = jacobi_svd_2x3_v_col2(&v0, &v1);
                let w2 = (1.0 + c) * 0.5;
                let s = (1.0 - w2).sqrt();
                return Quaterniond { w: w2.sqrt(), x: axis.x * s, y: axis.y * s, z: axis.z * s };
            }
            let axis = cross3(&v0, &v1);
            let s = ((1.0 + c) * 2.0).sqrt();
            let invs = 1.0 / s;
            Quaterniond { x: axis.x * invs, y: axis.y * invs, z: axis.z * invs, w: s * 0.5 }
        }
    }

    /// `Eigen::AngleAxisd`.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct AngleAxisd {
        pub angle: f64,
        pub axis: Vec3d,
    }

    impl AngleAxisd {
        pub fn new(angle: f64, axis: Vec3d) -> Self {
            AngleAxisd { angle, axis }
        }

        /// `AngleAxis::operator=(const QuaternionBase &)` (AngleAxis.h 178-193).
        pub fn from_quaternion(q: &Quaterniond) -> Self {
            let v = q.vec();
            let mut n = norm3(&v);
            if n < f64::EPSILON {
                n = stable_norm3(&v);
            }
            if n != 0.0 {
                let angle = 2.0 * n.atan2(q.w.abs());
                if q.w < 0.0 {
                    n = -n;
                }
                AngleAxisd { angle, axis: Vec3d::new(v.x / n, v.y / n, v.z / n) }
            } else {
                AngleAxisd { angle: 0.0, axis: Vec3d::new(1.0, 0.0, 0.0) }
            }
        }

        /// `AngleAxisd(const Matrix3d &)` (AngleAxis.h 199-203: through the quaternion).
        pub fn from_rotation_matrix(m: &Matrix3d) -> Self {
            Self::from_quaternion(&Quaterniond::from_rotation_matrix(m))
        }

        /// `toRotationMatrix()` (AngleAxis.h 213-240; the diagonal `cos1_axis.cwiseProduct(axis) +
        /// c` vector operations, unfused).
        pub fn to_rotation_matrix(&self) -> Matrix3d {
            let axis = &self.axis;
            let sa = self.angle.sin();
            let sin_axis = Vec3d::new(sa * axis.x, sa * axis.y, sa * axis.z);
            let c = self.angle.cos();
            let c1 = 1.0 - c;
            let cos1_axis = Vec3d::new(c1 * axis.x, c1 * axis.y, c1 * axis.z);
            let mut res = Matrix3d::zeros();
            let mut tmp = cos1_axis.x * axis.y;
            res[(0, 1)] = tmp - sin_axis.z;
            res[(1, 0)] = tmp + sin_axis.z;
            tmp = cos1_axis.x * axis.z;
            res[(0, 2)] = tmp + sin_axis.y;
            res[(2, 0)] = tmp - sin_axis.y;
            tmp = cos1_axis.y * axis.z;
            res[(1, 2)] = tmp - sin_axis.x;
            res[(2, 1)] = tmp + sin_axis.x;
            res[(0, 0)] = cos1_axis.x * axis.x + c;
            res[(1, 1)] = cos1_axis.y * axis.y + c;
            res[(2, 2)] = cos1_axis.z * axis.z + c;
            res
        }
    }

    /// `Vec3d::stableNorm()` (StableNorm.h `stable_norm_impl`, one block).
    pub fn stable_norm3(v: &Vec3d) -> f64 {
        let mut scale = 0.0_f64;
        let mut inv_scale = 1.0_f64;
        let mut ssq = 0.0_f64;
        let max_coeff = maxi(maxi(v.x.abs(), v.y.abs()), v.z.abs());
        if max_coeff > scale {
            let r = scale / max_coeff;
            ssq = ssq * (r * r);
            let tmp = 1.0 / max_coeff;
            if tmp > f64::MAX {
                inv_scale = f64::MAX;
                scale = 1.0 / inv_scale;
            } else if max_coeff > f64::MAX {
                inv_scale = 1.0;
                scale = max_coeff;
            } else {
                scale = max_coeff;
                inv_scale = tmp;
            }
        } else if max_coeff != max_coeff {
            scale = max_coeff;
        }
        if scale > 0.0 {
            let b = Vec3d::new(v.x * inv_scale, v.y * inv_scale, v.z * inv_scale);
            ssq += squared_norm3(&b);
        }
        scale * ssq.sqrt()
    }

    /// `Transform::rotate(Eigen::AngleAxisd(z, UnitZ) * Eigen::AngleAxisd(y, UnitY) *
    /// Eigen::AngleAxisd(x, UnitX))`'s rotation: the quaternion product `(qz * qy) * qx`.
    pub fn zyx_quaternion(rotation: &Vec3d) -> Quaterniond {
        let qz = Quaterniond::from_angle_axis(rotation.z, &Vec3d::new(0.0, 0.0, 1.0));
        let qy = Quaterniond::from_angle_axis(rotation.y, &Vec3d::new(0.0, 1.0, 0.0));
        let qx = Quaterniond::from_angle_axis(rotation.x, &Vec3d::new(1.0, 0.0, 0.0));
        qz.mul(&qy).mul(&qx)
    }

    /// The scalar loop of Jacobi.h `apply_rotation_in_the_plane_selector` (the rows and columns of a
    /// 3x3 or 2x2 matrix are not packet aligned): `x = c * x + s * y`, `y = -s * x + c * y`, each
    /// fused; nothing when `c == 1 && s == 0`.
    #[inline]
    fn rotate_pair(x: &mut f64, y: &mut f64, c: f64, s: f64) {
        let (xi, yi) = (*x, *y);
        *x = c.mul_add(xi, s * yi);
        *y = (-s).mul_add(xi, c * yi);
    }

    /// `JacobiRotation<double>`.
    #[derive(Debug, Clone, Copy)]
    struct JacobiRotation {
        c: f64,
        s: f64,
    }

    impl JacobiRotation {
        fn transpose(&self) -> Self {
            JacobiRotation { c: self.c, s: -self.s }
        }
        /// Jacobi.h `operator*`: `(c * o.c - s * o.s, c * o.s + s * o.c)`, each fused.
        fn mul(&self, o: &JacobiRotation) -> Self {
            JacobiRotation { c: self.c.mul_add(o.c, -(self.s * o.s)), s: self.c.mul_add(o.s, self.s * o.c) }
        }
        fn is_identity(&self) -> bool {
            self.c == 1.0 && self.s == 0.0
        }
        /// Jacobi.h `makeJacobi(x, y, z)`.
        fn make_jacobi(x: f64, y: f64, z: f64) -> Self {
            let deno = 2.0 * y.abs();
            if deno < f64::MIN_POSITIVE {
                return JacobiRotation { c: 1.0, s: 0.0 };
            }
            let tau = (x - z) / deno;
            let w = (tau * tau + 1.0).sqrt();
            let t = if tau > 0.0 { 1.0 / (tau + w) } else { 1.0 / (tau - w) };
            let sign_t = if t > 0.0 { 1.0 } else { -1.0 };
            let n = 1.0 / (t * t + 1.0).sqrt();
            JacobiRotation { c: n, s: -sign_t * (y / y.abs()) * t.abs() * n }
        }
    }

    /// `applyOnTheLeft(p, q, j)` of a 3x3 matrix: rows p and q.
    fn apply_on_the_left3(m: &mut Matrix3d, p: usize, q: usize, j: &JacobiRotation) {
        if j.is_identity() {
            return;
        }
        for i in 0..3 {
            let (mut x, mut y) = (m[(p, i)], m[(q, i)]);
            rotate_pair(&mut x, &mut y, j.c, j.s);
            m[(p, i)] = x;
            m[(q, i)] = y;
        }
    }

    /// `applyOnTheRight(p, q, j)` of a 3x3 matrix: columns p and q with `j.transpose()`.
    fn apply_on_the_right3(m: &mut Matrix3d, p: usize, q: usize, j: &JacobiRotation) {
        let jt = j.transpose();
        if jt.is_identity() {
            return;
        }
        for i in 0..3 {
            let (mut x, mut y) = (m[(i, p)], m[(i, q)]);
            rotate_pair(&mut x, &mut y, jt.c, jt.s);
            m[(i, p)] = x;
            m[(i, q)] = y;
        }
    }

    /// misc/RealSvd2x2.h `real_2x2_jacobi_svd` of the block (p, q) of `m`: `(j_left, j_right)`.
    fn real_2x2_jacobi_svd(m: &Matrix3d, p: usize, q: usize) -> (JacobiRotation, JacobiRotation) {
        let mut m2 = [[m[(p, p)], m[(p, q)]], [m[(q, p)], m[(q, q)]]];
        let t = m2[0][0] + m2[1][1];
        let d = m2[1][0] - m2[0][1];
        let rot1 = if d.abs() < f64::MIN_POSITIVE {
            JacobiRotation { c: 1.0, s: 0.0 }
        } else {
            let u = t / d;
            let tmp = (1.0 + u * u).sqrt();
            JacobiRotation { c: u / tmp, s: 1.0 / tmp }
        };
        // m.applyOnTheLeft(0, 1, rot1)
        if !rot1.is_identity() {
            for i in 0..2 {
                let (mut x, mut y) = (m2[0][i], m2[1][i]);
                rotate_pair(&mut x, &mut y, rot1.c, rot1.s);
                m2[0][i] = x;
                m2[1][i] = y;
            }
        }
        let j_right = JacobiRotation::make_jacobi(m2[0][0], m2[0][1], m2[1][1]);
        let j_left = rot1.mul(&j_right.transpose());
        (j_left, j_right)
    }

    /// `JacobiSVD<Matrix3d, ComputeFullU | ComputeFullV>(m)` (SVD/JacobiSVD.h `compute_impl`, the
    /// square case without a QR preconditioner, real scalars): `(U, V, singular values)`, the
    /// singular values sorted in decreasing order. A matrix with a coefficient that is not finite
    /// returns the identities and zeros (Eigen marks it invalid).
    pub fn jacobi_svd3(matrix: &Matrix3d) -> (Matrix3d, Matrix3d, Vec3d) {
        let precision = 2.0 * f64::EPSILON;
        let consider_as_zero = f64::MIN_POSITIVE;
        // maxCoeff<PropagateNaN> of the absolute values
        let mut scale = 0.0_f64;
        let mut nan = false;
        for k in 0..9 {
            let a = matrix[(k % 3, k / 3)].abs();
            if a.is_nan() {
                nan = true;
            }
            scale = maxi(scale, a);
        }
        if nan || !scale.is_finite() {
            return (Matrix3d::identity(), Matrix3d::identity(), Vec3d::zeros());
        }
        if scale == 0.0 {
            scale = 1.0;
        }
        let mut work = Matrix3d::zeros();
        for c in 0..3 {
            for r in 0..3 {
                work[(r, c)] = matrix[(r, c)] / scale;
            }
        }
        let mut u = Matrix3d::identity();
        let mut v = Matrix3d::identity();
        let mut max_diag_entry = maxi(maxi(work[(0, 0)].abs(), work[(1, 1)].abs()), work[(2, 2)].abs());
        let mut finished = false;
        while !finished {
            finished = true;
            for p in 1..3 {
                for q in 0..p {
                    let threshold = maxi(consider_as_zero, precision * max_diag_entry);
                    if work[(p, q)].abs() > threshold || work[(q, p)].abs() > threshold {
                        finished = false;
                        let (j_left, j_right) = real_2x2_jacobi_svd(&work, p, q);
                        apply_on_the_left3(&mut work, p, q, &j_left);
                        apply_on_the_right3(&mut u, p, q, &j_left.transpose());
                        apply_on_the_right3(&mut work, p, q, &j_right);
                        apply_on_the_right3(&mut v, p, q, &j_right);
                        max_diag_entry = maxi(max_diag_entry, maxi(work[(p, p)].abs(), work[(q, q)].abs()));
                    }
                }
            }
        }
        let mut sv = Vec3d::zeros();
        for i in 0..3 {
            let a = work[(i, i)];
            sv[i] = a.abs();
            if a < 0.0 {
                for r in 0..3 {
                    u[(r, i)] = -u[(r, i)];
                }
            }
        }
        for i in 0..3 {
            sv[i] *= scale;
        }
        for i in 0..3 {
            // maxCoeff(&pos) of the tail: the first index of the largest value
            let mut pos = 0;
            let mut max = sv[i];
            for k in (i + 1)..3 {
                if sv[k] > max {
                    max = sv[k];
                    pos = k - i;
                }
            }
            if max == 0.0 {
                break;
            }
            if pos != 0 {
                let pos = pos + i;
                sv.swap_rows(i, pos);
                u.swap_columns(i, pos);
                v.swap_columns(i, pos);
            }
        }
        (u, v, sv)
    }

    /// `Transform::computeRotationScaling(&rotation, &scaling)` (Geometry/Transform.h 1052-1067)
    /// of the linear part.
    pub fn compute_rotation_scaling(lin: &Matrix3d) -> (Matrix3d, Matrix3d) {
        let (u, v, sv) = jacobi_svd3(lin);
        let vt = v.transpose();
        let x = if determinant3(&mat3_mul(&u, &vt)) < 0.0 { -1.0 } else { 1.0 };
        let mut sv = sv;
        sv[2] *= x;
        // scaling = V * sv.asDiagonal() * V.adjoint()
        let scaling = mat3_mul(&mat3_mul_diag(&v, &sv), &vt);
        // rotation = (U with its last column times x) * V.adjoint()
        let mut m = u;
        for r in 0..3 {
            m[(r, 2)] *= x;
        }
        let rotation = mat3_mul(&m, &vt);
        (rotation, scaling)
    }

    /// The third column of V of `JacobiSVD<Matrix<double, 2, 3>, ComputeFullV>` of the matrix with
    /// rows `v0` and `v1` (Quaternion.h setFromTwoVectors). Eigen preconditions the 2x3 matrix with
    /// a ColPivHouseholderQR of its adjoint (SVD/JacobiSVD.h `qr_preconditioner_impl<...,
    /// PreconditionIfMoreColsThanRows>`), `V = householderQ()`; the Jacobi sweeps of the 2x2 block
    /// and the sorting only touch columns 0 and 1, so the column is the third column of that Q.
    /// QR/ColPivHouseholderQR.h `computeInPlace`, Householder/Householder.h `makeHouseholder` /
    /// `applyHouseholderOnTheLeft`, Householder/HouseholderSequence.h `evalTo`; the products of
    /// `applyHouseholderOnTheLeft` are an inner product (a 1x1 result) or a row-major GEMV
    /// (Core/products/GeneralMatrixVector.h: packets of two products from zero, `predux`, the
    /// result added to the zeroed destination).
    pub fn jacobi_svd_2x3_v_col2(v0: &Vec3d, v1: &Vec3d) -> Vec3d {
        // JacobiSVD compute_impl: scale = max |m_ij|; the matrix divided by it
        let mut scale = 0.0_f64;
        for k in 0..3 {
            scale = maxi(scale, maxi(v0[k].abs(), v1[k].abs()));
        }
        if !scale.is_finite() {
            return Vec3d::new(f64::NAN, f64::NAN, f64::NAN);
        }
        if scale == 0.0 {
            scale = 1.0;
        }
        // m_adjoint = (matrix / scale).adjoint(): 3x2, column 0 = v0, column 1 = v1
        let mut a = [[0.0_f64; 2]; 3];
        for i in 0..3 {
            a[i][0] = v0[i] / scale;
            a[i][1] = v1[i] / scale;
        }
        // ColPivHouseholderQR::computeInPlace (rows 3, cols 2, size 2)
        let (rows, cols, size) = (3usize, 2usize, 2usize);
        let col_norm = |a: &[[f64; 2]; 3], k: usize| squared_norm3(&Vec3d::new(a[0][k], a[1][k], a[2][k])).sqrt();
        let mut norms_direct = [col_norm(&a, 0), col_norm(&a, 1)];
        let mut norms_updated = norms_direct;
        let max_norm = maxi(norms_updated[0], norms_updated[1]);
        let th = max_norm * f64::EPSILON;
        let threshold_helper = (th * th) / rows as f64;
        let norm_downdate_threshold = f64::EPSILON.sqrt();
        let mut nonzero_pivots = size;
        let mut h_coeffs = [0.0_f64; 2];
        for k in 0..size {
            // the first column of the largest updated norm
            let mut biggest = k;
            for j in (k + 1)..cols {
                if norms_updated[j] > norms_updated[biggest] {
                    biggest = j;
                }
            }
            let biggest_col_sq_norm = norms_updated[biggest] * norms_updated[biggest];
            if nonzero_pivots == size && biggest_col_sq_norm < threshold_helper * (rows - k) as f64 {
                nonzero_pivots = k;
            }
            if k != biggest {
                for row in a.iter_mut() {
                    row.swap(k, biggest);
                }
                norms_updated.swap(k, biggest);
                norms_direct.swap(k, biggest);
            }
            // makeHouseholderInPlace of a[k..3][k]
            let c0 = a[k][k];
            let tail_sq_norm = if rows - k == 1 {
                0.0
            } else if rows - k - 1 == 1 {
                a[k + 1][k] * a[k + 1][k]
            } else {
                a[k + 1][k] * a[k + 1][k] + a[k + 2][k] * a[k + 2][k]
            };
            let (tau, beta);
            if tail_sq_norm <= f64::MIN_POSITIVE {
                tau = 0.0;
                beta = c0;
                for i in (k + 1)..rows {
                    a[i][k] = 0.0;
                }
            } else {
                let mut b = (c0 * c0 + tail_sq_norm).sqrt();
                if c0 >= 0.0 {
                    b = -b;
                }
                beta = b;
                for i in (k + 1)..rows {
                    a[i][k] /= c0 - beta;
                }
                tau = (beta - c0) / beta;
            }
            h_coeffs[k] = tau;
            a[k][k] = beta;
            // bottomRightCorner(rows - k, cols - k - 1).applyHouseholderOnTheLeft(essential, tau)
            for j in (k + 1)..cols {
                if rows - k == 1 {
                    a[k][j] *= 1.0 - tau;
                } else if tau != 0.0 {
                    // tmp = essential.adjoint() * bottom: a 1x1 inner product (packet, predux)
                    let mut d = 0.0;
                    let n = rows - k - 1;
                    if n == 2 {
                        d = a[k + 1][k] * a[k + 1][j] + a[k + 2][k] * a[k + 2][j];
                    } else if n == 1 {
                        d = a[k + 1][k] * a[k + 1][j];
                    }
                    let mut tmp = 1.0f64.mul_add(d, 0.0);
                    tmp += a[k][j];
                    a[k][j] -= tau * tmp;
                    for i in (k + 1)..rows {
                        let l = tau * a[i][k];
                        a[i][j] -= tmp * l;
                    }
                }
            }
            // update the table of norms of the columns
            for j in (k + 1)..cols {
                if norms_updated[j] != 0.0 {
                    let mut temp = a[k][j].abs() / norms_updated[j];
                    temp = (1.0 + temp) * (1.0 - temp);
                    temp = if temp < 0.0 { 0.0 } else { temp };
                    let q = norms_updated[j] / norms_direct[j];
                    let temp2 = temp * (q * q);
                    if temp2 <= norm_downdate_threshold {
                        let n = rows - k - 1;
                        let sq = if n == 2 {
                            a[k + 1][j] * a[k + 1][j] + a[k + 2][j] * a[k + 2][j]
                        } else if n == 1 {
                            a[k + 1][j] * a[k + 1][j]
                        } else {
                            0.0
                        };
                        norms_direct[j] = sq.sqrt();
                        norms_updated[j] = norms_direct[j];
                    } else {
                        norms_updated[j] *= temp.sqrt();
                    }
                }
            }
        }
        let _ = nonzero_pivots;
        // householderQ().evalTo(V): V = I, then for k = 1, 0 the bottom-right corner of size
        // 3 - k applyHouseholderOnTheLeft(essentialVector(k), hCoeffs(k))
        let mut q = [[0.0_f64; 3]; 3];
        for (i, row) in q.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        for k in (0..size).rev() {
            let tau = h_coeffs[k];
            let corner = rows - k;
            if corner == 1 {
                q[k][k] *= 1.0 - tau;
                continue;
            }
            if tau == 0.0 {
                continue;
            }
            let n = corner - 1; // essential size
            let ess: Vec<f64> = ((k + 1)..rows).map(|i| a[i][k]).collect();
            let mut tmp = vec![0.0_f64; corner];
            for (jj, t) in tmp.iter_mut().enumerate() {
                let j = k + jj;
                if corner == 1 {
                    // a 1x1 result: the inner product
                    *t = 1.0f64.mul_add(ess[0] * q[k + 1][j], 0.0);
                } else if n == 2 {
                    // row-major GEMV, two products in a packet from zero, predux
                    let l0 = ess[0].mul_add(q[k + 1][j], 0.0);
                    let l1 = ess[1].mul_add(q[k + 2][j], 0.0);
                    *t = 1.0f64.mul_add(l0 + l1, 0.0);
                } else {
                    // row-major GEMV of depth 1: no packet, `cc = 0; cc += l * b`
                    let cc = 0.0 + ess[0] * q[k + 1][j];
                    *t = 1.0f64.mul_add(cc, 0.0);
                }
            }
            for (jj, t) in tmp.iter_mut().enumerate() {
                *t += q[k][k + jj];
            }
            for (jj, t) in tmp.iter().enumerate() {
                q[k][k + jj] -= tau * t;
            }
            let l: Vec<f64> = ess.iter().map(|e| tau * e).collect();
            for (jj, t) in tmp.iter().enumerate() {
                for (ii, li) in l.iter().enumerate() {
                    q[k + 1 + ii][k + jj] -= t * li;
                }
            }
        }
        Vec3d::new(q[0][2], q[1][2], q[2][2])
    }
}

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
