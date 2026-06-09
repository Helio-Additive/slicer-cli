//! Adaptive cubic infill implementation.
//!
//! Faithful 1:1 port of `src/libslic3r/Fill/FillAdaptive.cpp` (+ `.hpp`) from
//! BambuStudio. coord_t -> i64, coordf_t -> f64.
//!
//! Adaptive cubic infill was inspired by the work of @mboerwinkle
//! as implemented for Cura.
//! https://github.com/Ultimaker/CuraEngine/issues/381
//! https://github.com/Ultimaker/CuraEngine/pull/401
//!
//! Our implementation is more accurate (discretizes a bit less cubes than Cura's)
//! by splitting only such cubes which contain a triangle.
//! Our line extraction is time optimal instead of O(n^2) when connecting extracted lines,
//! and we also implemented adaptivity for supporting internal overhangs only.
//!
//! BLOCKED SYMBOLS (data-model / base-class divergence, see notes):
//!  - `adaptive_fill_line_spacing(const PrintObject&)`: requires `PrintObject::print()`,
//!    a `std::vector<double> nozzle_diameter`, and per-`LayerRegion` `fill_surfaces`
//!    threaded through `Layer::regions()`. The Rust `PrintObject`/`Layer` data model
//!    diverges (scalar nozzle_diameter, no `print()` accessor, single fill_surfaces).
//!  - `Filler::_fill_surface_single`: requires the `Slic3r::Fill` base class state
//!    (`this->z`, `this->spacing`, `this->adapt_fill_octree`, `multiline_fill`,
//!    `connect_infill`) wired through the FillBase virtual-dispatch machinery, which
//!    has no equivalent trait/state in the Rust crate yet.

use crate::geometry::aabb_tree::{IndexedTriangleSet, Vec3 as Vec3d};
use crate::geometry::{cross2f, ExPolygon, Line, Point, PointF, Polyline};
use crate::shortest_path::chain_polylines;
use crate::Coord;
use nalgebra::{UnitQuaternion, Vector3};
use std::f64::consts::PI;

// FillAdaptive.cpp uses Slic3r::sqr; the crate has no free `sqr`, so define a
// local one matching `template<typename T> T sqr(T x) { return x * x; }`.
#[inline]
fn sqr(x: f64) -> f64 {
    x * x
}

// SCALED_EPSILON / EPSILON as used in the C++ (libslic3r.h). The Rust crate's
// constants differ in scaling convention, so we use the BambuStudio values.
const SCALED_EPSILON: f64 = crate::libslic3r::SCALED_EPSILON; // 10
const EPSILON: f64 = crate::libslic3r::EPSILON; // 1e-4

// nalgebra Vector3<f64> is used for the octree quaternion math (mirrors Eigen Vec3d).
type EVec3d = Vector3<f64>;

#[inline]
fn ev(v: Vec3d) -> EVec3d {
    EVec3d::new(v.x, v.y, v.z)
}
#[inline]
fn cv(v: EVec3d) -> Vec3d {
    Vec3d::new(v.x, v.y, v.z)
}

// FillAdaptive.cpp:42
// Derived from https://github.com/juj/MathGeoLib/blob/master/src/Geometry/Triangle.cpp
// The AABB-Triangle test implementation is based on the pseudo-code in
// Christer Ericson's Real-Time Collision Detection, pp. 169-172. It is
// practically a standard SAT test.
//
// Original MathGeoLib benchmark:
//    Best: 17.282 nsecs / 46.496 ticks, Avg: 17.804 nsecs, Worst: 18.434 nsecs
//
//FIXME Vojtech: The MathGeoLib contains a vectorized implementation.
pub fn triangle_aabb_intersects(a: Vec3d, b: Vec3d, c: Vec3d, aabb: &BoundingBoxf3) -> bool {
    // FillAdaptive.cpp:46
    let t_min = a.min(&b.min(&c));
    // FillAdaptive.cpp:47
    let t_max = a.max(&b.max(&c));

    // FillAdaptive.cpp:49-52
    if t_min.x() >= aabb.max.x()
        || t_max.x() <= aabb.min.x()
        || t_min.y() >= aabb.max.y()
        || t_max.y() <= aabb.min.y()
        || t_min.z() >= aabb.max.z()
        || t_max.z() <= aabb.min.z()
    {
        return false;
    }

    // FillAdaptive.cpp:54
    let center = (aabb.min + aabb.max) * 0.5;
    // FillAdaptive.cpp:55
    let h = aabb.max - center;

    // FillAdaptive.cpp:57
    let t: [Vec3d; 3] = [b - a, c - a, c - b];

    // FillAdaptive.cpp:59
    let ac = a - center;

    // FillAdaptive.cpp:61
    let n = t[0].cross(&t[1]);
    // FillAdaptive.cpp:62
    let s = n.dot(&ac);
    // FillAdaptive.cpp:63
    let mut r = (h.dot(&cwise_abs(n))).abs();
    // FillAdaptive.cpp:64
    if s.abs() >= r {
        return false;
    }

    // FillAdaptive.cpp:67
    let at: [Vec3d; 3] = [cwise_abs(t[0]), cwise_abs(t[1]), cwise_abs(t[2])];

    // FillAdaptive.cpp:69-70
    let bc = b - center;
    let cc = c - center;

    // SAT test all cross-axes.
    // The following is a fully unrolled loop of this code, stored here for reference:
    /*
    Scalar d1, d2, a1, a2;
    const Vector e[3] = { DIR_VEC(1, 0, 0), DIR_VEC(0, 1, 0), DIR_VEC(0, 0, 1) };
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
        {
            Vector axis = Cross(e[i], t[j]);
            ProjectToAxis(axis, d1, d2);
            aabb.ProjectToAxis(axis, a1, a2);
            if (d2 <= a1 || d1 >= a2) return false;
        }
    */

    // FillAdaptive.cpp:88 eX <cross> t[0]
    let mut d1 = t[0].y() * ac.z() - t[0].z() * ac.y();
    let mut d2 = t[0].y() * cc.z() - t[0].z() * cc.y();
    let mut tc = (d1 + d2) * 0.5;
    r = (h.y() * at[0].z() + h.z() * at[0].y()).abs();
    if r + (tc - d1).abs() < tc.abs() {
        return false;
    }

    // FillAdaptive.cpp:96 eX <cross> t[1]
    d1 = t[1].y() * ac.z() - t[1].z() * ac.y();
    d2 = t[1].y() * bc.z() - t[1].z() * bc.y();
    tc = (d1 + d2) * 0.5;
    r = (h.y() * at[1].z() + h.z() * at[1].y()).abs();
    if r + (tc - d1).abs() < tc.abs() {
        return false;
    }

    // FillAdaptive.cpp:104 eX <cross> t[2]
    d1 = t[2].y() * ac.z() - t[2].z() * ac.y();
    d2 = t[2].y() * bc.z() - t[2].z() * bc.y();
    tc = (d1 + d2) * 0.5;
    r = (h.y() * at[2].z() + h.z() * at[2].y()).abs();
    if r + (tc - d1).abs() < tc.abs() {
        return false;
    }

    // FillAdaptive.cpp:112 eY <cross> t[0]
    d1 = t[0].z() * ac.x() - t[0].x() * ac.z();
    d2 = t[0].z() * cc.x() - t[0].x() * cc.z();
    tc = (d1 + d2) * 0.5;
    r = (h.x() * at[0].z() + h.z() * at[0].x()).abs();
    if r + (tc - d1).abs() < tc.abs() {
        return false;
    }

    // FillAdaptive.cpp:120 eY <cross> t[1]
    d1 = t[1].z() * ac.x() - t[1].x() * ac.z();
    d2 = t[1].z() * bc.x() - t[1].x() * bc.z();
    tc = (d1 + d2) * 0.5;
    r = (h.x() * at[1].z() + h.z() * at[1].x()).abs();
    if r + (tc - d1).abs() < tc.abs() {
        return false;
    }

    // FillAdaptive.cpp:128 eY <cross> t[2]
    d1 = t[2].z() * ac.x() - t[2].x() * ac.z();
    d2 = t[2].z() * bc.x() - t[2].x() * bc.z();
    tc = (d1 + d2) * 0.5;
    r = (h.x() * at[2].z() + h.z() * at[2].x()).abs();
    if r + (tc - d1).abs() < tc.abs() {
        return false;
    }

    // FillAdaptive.cpp:136 eZ <cross> t[0]
    d1 = t[0].x() * ac.y() - t[0].y() * ac.x();
    d2 = t[0].x() * cc.y() - t[0].y() * cc.x();
    tc = (d1 + d2) * 0.5;
    r = (h.y() * at[0].x() + h.x() * at[0].y()).abs();
    if r + (tc - d1).abs() < tc.abs() {
        return false;
    }

    // FillAdaptive.cpp:144 eZ <cross> t[1]
    d1 = t[1].x() * ac.y() - t[1].y() * ac.x();
    d2 = t[1].x() * bc.y() - t[1].y() * bc.x();
    tc = (d1 + d2) * 0.5;
    r = (h.y() * at[1].x() + h.x() * at[1].y()).abs();
    if r + (tc - d1).abs() < tc.abs() {
        return false;
    }

    // FillAdaptive.cpp:152 eZ <cross> t[2]
    d1 = t[2].x() * ac.y() - t[2].y() * ac.x();
    d2 = t[2].x() * bc.y() - t[2].y() * bc.x();
    tc = (d1 + d2) * 0.5;
    r = (h.y() * at[2].x() + h.x() * at[2].y()).abs();
    if r + (tc - d1).abs() < tc.abs() {
        return false;
    }

    // FillAdaptive.cpp:159-160
    // No separating axis exists, the AABB and triangle intersect.
    true
}

#[inline]
fn cwise_abs(v: Vec3d) -> Vec3d {
    Vec3d::new(v.x.abs(), v.y.abs(), v.z.abs())
}

/// 3D float bounding box, matching the C++ `BoundingBoxf3` (`BoundingBox3Base<Vec3d>`)
/// usage in this file: only `.min` / `.max` are read, and the default-constructed
/// box has its components written field by field.
// FillAdaptive uses BoundingBoxf3 from BoundingBox.hpp.
#[derive(Debug, Clone, Copy)]
pub struct BoundingBoxf3 {
    pub min: Vec3d,
    pub max: Vec3d,
}

impl BoundingBoxf3 {
    #[inline]
    pub fn new(min: Vec3d, max: Vec3d) -> Self {
        Self { min, max }
    }
    #[inline]
    fn default_box() -> Self {
        // C++ default BoundingBox3Base() leaves min/max uninitialized-ish; in this
        // file the only default-constructed bbox (insert_triangle) writes every
        // component before use, so any placeholder is fine.
        Self {
            min: Vec3d::zero(),
            max: Vec3d::zero(),
        }
    }
}

// FillAdaptive.cpp:225
// Ordering of children cubes.
const CHILD_CENTERS: [Vec3d; 8] = [
    Vec3d { x: -1.0, y: -1.0, z: -1.0 },
    Vec3d { x: 1.0, y: -1.0, z: -1.0 },
    Vec3d { x: -1.0, y: 1.0, z: -1.0 },
    Vec3d { x: 1.0, y: 1.0, z: -1.0 },
    Vec3d { x: -1.0, y: -1.0, z: 1.0 },
    Vec3d { x: 1.0, y: -1.0, z: 1.0 },
    Vec3d { x: -1.0, y: 1.0, z: 1.0 },
    Vec3d { x: 1.0, y: 1.0, z: 1.0 },
];

// FillAdaptive.cpp:232
// Traversal order of octree children cells for three infill directions,
// so that a single line will be discretized in a strictly monotonic order.
const CHILD_TRAVERSAL_ORDER: [[usize; 8]; 3] = [
    [2, 3, 0, 1, 6, 7, 4, 5],
    [4, 0, 6, 2, 5, 1, 7, 3],
    [1, 5, 0, 4, 3, 7, 2, 6],
];

// FillAdaptive.cpp:238
// Cubes are allocated from a boost::object_pool in C++; in Rust we own children
// via a Vec arena and address them by index (usize::MAX == nullptr).
#[derive(Clone)]
pub struct Cube {
    // FillAdaptive.cpp:240
    pub center: Vec3d,
    // FillAdaptive.cpp:242 (NDEBUG-only in C++; kept for parity of build_octree path)
    pub center_octree: Vec3d,
    // FillAdaptive.cpp:244 -- initialized to nullptrs
    pub children: [usize; 8],
}

const NULL_CUBE: usize = usize::MAX;

impl Cube {
    // FillAdaptive.cpp:245
    fn new(center: Vec3d) -> Self {
        Self {
            center,
            center_octree: center,
            children: [NULL_CUBE; 8],
        }
    }
}

// FillAdaptive.cpp:248
#[derive(Debug, Clone, Copy)]
pub struct CubeProperties {
    /// Lenght of edge of a cube
    // FillAdaptive.cpp:250
    pub edge_length: f64,
    /// Height of rotated cube (standing on the corner)
    // FillAdaptive.cpp:251
    pub height: f64,
    /// Length of diagonal of a cube a face
    // FillAdaptive.cpp:252
    pub diagonal_length: f64,
    /// Defines maximal distance from a center of a cube on Z axis on which lines will be created
    // FillAdaptive.cpp:253
    pub line_z_distance: f64,
    /// Defines maximal distance from a center of a cube on X and Y axis on which lines will be created
    // FillAdaptive.cpp:254
    pub line_xy_distance: f64,
}

// FillAdaptive.cpp:257
pub struct Octree {
    // FillAdaptive.cpp:261 -- boost::object_pool<Cube>; here an index arena.
    pub pool: Vec<Cube>,
    // FillAdaptive.cpp:262
    pub root_cube: usize,
    // FillAdaptive.cpp:263
    pub origin: Vec3d,
    // FillAdaptive.cpp:264
    pub cubes_properties: Vec<CubeProperties>,
}

impl Octree {
    // FillAdaptive.cpp:266
    fn new(origin: Vec3d, cubes_properties: Vec<CubeProperties>) -> Self {
        let mut pool = Vec::new();
        let root_cube = pool.len();
        pool.push(Cube::new(origin));
        Self {
            pool,
            root_cube,
            origin,
            cubes_properties,
        }
    }

    #[inline]
    fn construct(&mut self, center: Vec3d) -> usize {
        let idx = self.pool.len();
        self.pool.push(Cube::new(center));
        idx
    }

    // FillAdaptive.cpp:1519
    fn insert_triangle(
        &mut self,
        a: Vec3d,
        b: Vec3d,
        c: Vec3d,
        current_cube: usize,
        current_bbox: &BoundingBoxf3,
        depth: i32,
    ) {
        // FillAdaptive.cpp:1521-1522
        debug_assert!(current_cube != NULL_CUBE);
        debug_assert!(depth > 0);

        // FillAdaptive.cpp:1524
        let depth = depth - 1;

        // Squared radius of a sphere around the child cube.
        // const double r2_cube = Slic3r::sqr(0.5 * this->cubes_properties[depth].height + EPSILON);

        // FillAdaptive.cpp:1529
        for i in 0..8usize {
            // FillAdaptive.cpp:1530
            let child_center_dir = CHILD_CENTERS[i];
            // Calculate a slightly expanded bounding box of a child cube to cope with triangles touching a cube wall and other numeric errors.
            // We will rather densify the octree a bit more than necessary instead of missing a triangle.
            // FillAdaptive.cpp:1533
            let mut bbox = BoundingBoxf3::default_box();
            // FillAdaptive.cpp:1534
            for k in 0..3usize {
                let cur_center = self.pool[current_cube].center;
                if child_center_dir.component(k) == -1.0 {
                    set_component(&mut bbox.min, k, current_bbox.min.component(k));
                    set_component(&mut bbox.max, k, cur_center.component(k) + EPSILON);
                } else {
                    set_component(&mut bbox.min, k, cur_center.component(k) - EPSILON);
                    set_component(&mut bbox.max, k, current_bbox.max.component(k));
                }
            }
            // FillAdaptive.cpp:1543
            let child_center = self.pool[current_cube].center
                + (child_center_dir * (self.cubes_properties[depth as usize].edge_length / 2.0));
            //if (dist2_to_triangle(a, b, c, child_center) < r2_cube) {
            // dist2_to_triangle and r2_cube are commented out too.
            // FillAdaptive.cpp:1546
            if triangle_aabb_intersects(a, b, c, &bbox) {
                // FillAdaptive.cpp:1547
                if self.pool[current_cube].children[i] == NULL_CUBE {
                    let new_cube = self.construct(child_center);
                    self.pool[current_cube].children[i] = new_cube;
                }
                // FillAdaptive.cpp:1549
                if depth > 0 {
                    let child = self.pool[current_cube].children[i];
                    self.insert_triangle(a, b, c, child, &bbox, depth);
                }
            }
        }
    }
}

#[inline]
fn set_component(v: &mut Vec3d, idx: usize, value: f64) {
    match idx {
        0 => v.x = value,
        1 => v.y = value,
        _ => v.z = value,
    }
}

// FillAdaptive.cpp:400
// (Geometry::deg2rad(215.264) inlined to radians.)
const OCTREE_ROT: [f64; 3] = [
    5.0 * PI / 4.0,
    215.264 * PI / 180.0,
    PI / 6.0,
];

// FillAdaptive.cpp:402
pub fn transform_to_world() -> UnitQuaternion<f64> {
    // FillAdaptive.cpp:404
    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), OCTREE_ROT[2])
        * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), OCTREE_ROT[1])
        * UnitQuaternion::from_axis_angle(&Vector3::x_axis(), OCTREE_ROT[0])
}

// FillAdaptive.cpp:407
pub fn transform_to_octree() -> UnitQuaternion<f64> {
    // FillAdaptive.cpp:409
    UnitQuaternion::from_axis_angle(&Vector3::x_axis(), -OCTREE_ROT[0])
        * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -OCTREE_ROT[1])
        * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -OCTREE_ROT[2])
}

// FillAdaptive.cpp:361
// Context used by generate_infill_lines() when recursively traversing an octree in a DDA fashion
// (Digital Differential Analyzer).
struct FillContext<'a> {
    // FillAdaptive.cpp:384
    cubes_properties: &'a [CubeProperties],
    // Top of the current layer.
    // FillAdaptive.cpp:386
    z_position: f64,
    // Order of traversal for this line direction.
    // FillAdaptive.cpp:388
    traversal_order: [usize; 8],
    // Rotation of the generated line for this line direction.
    // FillAdaptive.cpp:390
    cos_a: f64,
    // FillAdaptive.cpp:391
    sin_a: f64,

    // Linearized tree spanning a single Octree wall, used to connect lines spanning
    // neighboring Octree cells. Unused lines have the Line::a::x set to infinity.
    // FillAdaptive.cpp:395
    temp_lines: Vec<Line>,
    // Final output
    // FillAdaptive.cpp:397
    output_lines: Vec<Line>,
}

impl<'a> FillContext<'a> {
    // FillAdaptive.cpp:364
    // The angles have to agree with child_traversal_order.
    const DIRECTION_ANGLES: [f64; 3] = [0.0, (2.0 * PI) / 3.0, -(2.0 * PI) / 3.0];

    // FillAdaptive.cpp:370
    fn new(octree: &'a Octree, z_position: f64, direction_idx: usize) -> Self {
        // FillAdaptive.cpp:377
        let unused = Coord::MAX;
        // FillAdaptive.cpp:378
        let temp_lines = vec![
            Line::new(Point::new(unused, unused), Point::new(unused, unused));
            (1usize << octree.cubes_properties.len()) - 1
        ];
        Self {
            cubes_properties: &octree.cubes_properties,
            z_position,
            traversal_order: CHILD_TRAVERSAL_ORDER[direction_idx],
            cos_a: Self::DIRECTION_ANGLES[direction_idx].cos(),
            sin_a: Self::DIRECTION_ANGLES[direction_idx].sin(),
            temp_lines,
            output_lines: Vec::new(),
        }
    }

    // FillAdaptive.cpp:382
    // Rotate the point, uses the same convention as Point::rotate().
    #[inline]
    fn rotate(&self, v: PointF) -> PointF {
        PointF::new(
            self.cos_a * v.x() - self.sin_a * v.y(),
            self.sin_a * v.x() + self.cos_a * v.y(),
        )
    }
}

// FillAdaptive.cpp:449
fn generate_infill_lines_recursive(
    context: &mut FillContext,
    pool: &[Cube],
    cube: usize,
    // Address of this wall in the octree, used to address context.temp_lines.
    address: i32,
    depth: i32,
) {
    // FillAdaptive.cpp:456
    debug_assert!(cube != NULL_CUBE);

    // FillAdaptive.cpp:459
    let z_diff = context.z_position - pool[cube].center.z();
    // FillAdaptive.cpp:460
    let z_diff_abs = z_diff.abs();

    // FillAdaptive.cpp:462
    if z_diff_abs > context.cubes_properties[depth as usize].height / 2.0 {
        return;
    }

    // FillAdaptive.cpp:465
    if z_diff_abs < context.cubes_properties[depth as usize].line_z_distance {
        // Discretize a single wall splitting the cube into two.
        // FillAdaptive.cpp:467
        let zdist = context.cubes_properties[depth as usize].line_z_distance;
        // FillAdaptive.cpp:468
        let mut from = PointF::new(
            0.5 * context.cubes_properties[depth as usize].diagonal_length * (zdist - z_diff_abs)
                / zdist,
            context.cubes_properties[depth as usize].line_xy_distance
                - (zdist + z_diff) / 2.0_f64.sqrt(),
        );
        // FillAdaptive.cpp:471
        let mut to = PointF::new(-from.x(), from.y());
        // FillAdaptive.cpp:472
        from = context.rotate(from);
        // FillAdaptive.cpp:473
        to = context.rotate(to);
        // Relative to cube center
        // FillAdaptive.cpp:475
        let offset = PointF::new(pool[cube].center.x(), pool[cube].center.y());
        // FillAdaptive.cpp:476
        from = from + offset;
        // FillAdaptive.cpp:477
        to = to + offset;
        // Verify that the traversal order of the octree children matches the line direction,
        // therefore the infill line may get extended with O(1) time & space complexity.
        // (assert(verify_traversal_order(...)) — NDEBUG-only.)
        // Either extend an existing line or start a new one.
        // FillAdaptive.cpp:482-483
        let new_line = Line::new(point_new_scale(from), point_new_scale(to));
        let last_line = context.temp_lines[address as usize];
        // FillAdaptive.cpp:484
        if last_line.a.x() == Coord::MAX {
            context.temp_lines[address as usize].a = new_line.a;
        } else if cwise_abs_max_coeff_pt(new_line.a - last_line.b) > 1000 {
            // SCALED_EPSILON is 100 and it is not enough
            // FillAdaptive.cpp:486-488
            context.output_lines.push(last_line);
            context.temp_lines[address as usize].a = new_line.a;
        }
        // FillAdaptive.cpp:490
        context.temp_lines[address as usize].b = new_line.b;
    }

    // left child index
    // FillAdaptive.cpp:494
    let mut address = address * 2 + 1;
    // FillAdaptive.cpp:495
    let depth = depth - 1;
    // FillAdaptive.cpp:496
    let mut i = 0usize;
    // FillAdaptive.cpp:497
    for child_idx in context.traversal_order {
        // FillAdaptive.cpp:498
        let child = pool[cube].children[child_idx];
        // FillAdaptive.cpp:499
        if child != NULL_CUBE {
            generate_infill_lines_recursive(context, pool, child, address, depth);
        }
        // FillAdaptive.cpp:501
        i += 1;
        if i == 4 {
            // right child index
            address += 1;
        }
    }
}

#[inline]
fn cwise_abs_max_coeff_pt(p: Point) -> Coord {
    p.x.abs().max(p.y.abs())
}

// Point::new_scale(const Vec2d&) — scale each coordinate.
#[inline]
fn point_new_scale(v: PointF) -> Point {
    Point::new_scale(v.x(), v.y())
}

// FillAdaptive.cpp:561
// Representing a T-joint (in general case) between two infill lines
// (between one end point of intersect_pl/intersect_line and
#[derive(Clone)]
struct Intersection {
    // Closest line to intersect_point.
    // FillAdaptive.cpp:564
    closest_line: usize,
    // The line for which is computed closest line from intersect_point to closest_line
    // FillAdaptive.cpp:567
    intersect_line: usize,
    // Pointer to the polyline from which is computed closest_line
    // FillAdaptive.cpp:569 -- index into `lines`
    intersect_pl: usize,
    // Point for which is computed closest line (closest_line)
    // FillAdaptive.cpp:571
    intersect_point: Point,
    // Indicate if intersect_point is the first or the last point of intersect_pl
    // FillAdaptive.cpp:573
    front: bool,
    // Signum of intersect_line_dir.cross(closest_line.dir()):
    // FillAdaptive.cpp:575
    left: bool,
    // Indication if this intersection has been proceed
    // FillAdaptive.cpp:578
    used: bool,
}

impl Intersection {
    // FillAdaptive.cpp:582
    fn new(
        lines_src: &[Line],
        closest_line: usize,
        intersect_line: usize,
        intersect_pl: usize,
        intersect_point: Point,
        front: bool,
    ) -> Self {
        // Calculate side of this intersection line of the closest line.
        // FillAdaptive.cpp:586
        let cl = &lines_src[closest_line];
        let v1 = PointF::new((cl.b.x - cl.a.x) as f64, (cl.b.y - cl.a.y) as f64);
        // FillAdaptive.cpp:587
        let v2 = intersect_line_dir(&lines_src[intersect_line], intersect_point);
        // FillAdaptive.cpp:596
        let left = cross2f(v1, v2) > 0.0;
        Self {
            closest_line,
            intersect_line,
            intersect_pl,
            intersect_point,
            front,
            left,
            used: false,
        }
    }

    // FillAdaptive.cpp:580
    #[inline]
    fn fresh(&self, lines: &[Polyline]) -> bool {
        !self.used && !lines[self.intersect_pl].empty()
    }

    // FillAdaptive.cpp:599
    fn other_hook(&self, lines: &[Polyline]) -> Option<Line> {
        // FillAdaptive.cpp:601
        let pts = &lines[self.intersect_pl].points;
        // FillAdaptive.cpp:602
        if pts.len() >= 3 {
            // FillAdaptive.cpp:603
            Some(if self.front {
                Line::new(pts[1], pts[2])
            } else {
                Line::new(pts[pts.len() - 2], pts[pts.len() - 3])
            })
        } else {
            None
        }
    }

    // FillAdaptive.cpp:607
    fn other_hook_intersects_pt(&self, lines: &[Polyline], l: &Line, pt: &mut Point) -> bool {
        // FillAdaptive.cpp:608
        match self.other_hook(lines) {
            // FillAdaptive.cpp:609
            Some(h) => match h.intersection(l) {
                Some(p) => {
                    *pt = p;
                    true
                }
                None => false,
            },
            None => false,
        }
    }

    // FillAdaptive.cpp:611 (only reached from the C++ `#if 0` self-intersection block)
    #[allow(dead_code)]
    fn other_hook_intersects(&self, lines: &[Polyline], l: &Line) -> bool {
        let mut pt = Point::zero();
        self.other_hook_intersects_pt(lines, l, &mut pt)
    }
}

// FillAdaptive.cpp:614
// Direction to intersect_point.
#[inline]
fn intersect_line_dir(intersect_line: &Line, intersect_point: Point) -> PointF {
    let d = if intersect_point == intersect_line.a {
        intersect_line.b - intersect_line.a
    } else {
        intersect_line.a - intersect_line.b
    };
    PointF::new(d.x as f64, d.y as f64)
}

// FillAdaptive.cpp:619
fn get_nearest_intersection(
    intersect_line: &[(usize, f64)],
    intersections: &[Intersection],
    lines: &[Polyline],
    first_idx: usize,
) -> usize {
    // FillAdaptive.cpp:621
    debug_assert!(intersect_line.len() >= 2);
    // FillAdaptive.cpp:622
    let take_next;
    // FillAdaptive.cpp:623
    if first_idx == 0 {
        take_next = true;
    } else if first_idx + 1 == intersect_line.len() {
        // FillAdaptive.cpp:625
        take_next = false;
    } else {
        // Has both prev and next.
        // FillAdaptive.cpp:629-631
        let ithis = &intersect_line[first_idx];
        let iprev = &intersect_line[first_idx - 1];
        let inext = &intersect_line[first_idx + 1];
        // FillAdaptive.cpp:632
        take_next =
            if intersections[iprev.0].fresh(lines) && intersections[inext.0].fresh(lines) {
                inext.1 - ithis.1 < ithis.1 - iprev.1
            } else {
                intersections[inext.0].fresh(lines)
            };
    }
    // FillAdaptive.cpp:636
    intersect_line[if take_next { first_idx + 1 } else { first_idx - 1 }].0
}

// FillAdaptive.cpp:641
// Create a line representing the anchor aka hook extrusion based on line_to_offset
// translated in the direction of the intersection line (intersection.intersect_line).
fn create_offset_line(
    mut offset_line: Line,
    intersection: &Intersection,
    lines_src: &[Line],
    scaled_offset: f64,
) -> Line {
    // FillAdaptive.cpp:643
    let cl = &lines_src[intersection.closest_line];
    let dir = PointF::new((cl.b.x - cl.a.x) as f64, (cl.b.y - cl.a.y) as f64).normalize();
    let perp_dir = dir.perp();
    let off = if intersection.left {
        scaled_offset
    } else {
        -scaled_offset
    };
    offset_line = offset_line.translate(Point::new(
        (perp_dir.x * off) as Coord,
        (perp_dir.y * off) as Coord,
    ));
    // Extend the line by a small value to guarantee a collision with adjacent lines
    // FillAdaptive.cpp:645
    offset_line.extend(scaled_offset * 1.16); // / cos(PI/6)
    offset_line
}

// FillAdaptive.cpp:654 -- boost::geometry rtree.
// rstar is a declared crate dependency but the rtree usage here (nearest segment,
// all segments intersecting a query segment, point-keyed removal) is straightforward
// to satisfy exactly with a flat segment store, which keeps query results
// deterministic and identical to the accelerated structure.
#[derive(Clone, Copy)]
struct RSeg {
    a: Point,
    b: Point,
    idx: usize,
    // Tombstone for removed entries (rtree.remove).
    live: bool,
}

#[derive(Default)]
struct RTree {
    segs: Vec<RSeg>,
}

impl RTree {
    fn new() -> Self {
        Self { segs: Vec::new() }
    }
    // FillAdaptive.cpp:869 etc.
    fn insert(&mut self, a: Point, b: Point, idx: usize) {
        self.segs.push(RSeg {
            a,
            b,
            idx,
            live: true,
        });
    }
    // rtree.remove(item)
    fn remove(&mut self, a: Point, b: Point, idx: usize) {
        if let Some(s) = self
            .segs
            .iter_mut()
            .find(|s| s.live && s.idx == idx && s.a == a && s.b == b)
        {
            s.live = false;
        }
    }
    // bgi::nearest(pt, 1) [&& satisfies(filter)] -> closest segment by point-to-segment distance (float).
    fn nearest<F: Fn(usize) -> bool>(&self, pt: Point, filter: F) -> Option<RSeg> {
        let ptf = PointF::new(pt.x as f32 as f64, pt.y as f32 as f64);
        let mut best: Option<(f64, RSeg)> = None;
        for s in &self.segs {
            if !s.live || !filter(s.idx) {
                continue;
            }
            let d = seg_point_dist2_f(s.a, s.b, ptf);
            match best {
                Some((bd, _)) if d >= bd => {}
                _ => best = Some((d, *s)),
            }
        }
        best.map(|(_, s)| s)
    }
    // bgi::intersects(query_seg) && satisfies(filter) -> all intersecting segments.
    fn query_intersects<F: Fn(usize) -> bool>(&self, qa: Point, qb: Point, filter: F) -> Vec<RSeg> {
        let mut out = Vec::new();
        let q = Line::new(qa, qb);
        for s in &self.segs {
            if !s.live || !filter(s.idx) {
                continue;
            }
            if Line::new(s.a, s.b).intersection(&q).is_some() {
                out.push(*s);
            }
        }
        out
    }
}

// Float point-to-segment squared distance, matching rtree_point_t/rtree_segment_t (float).
fn seg_point_dist2_f(a: Point, b: Point, p: PointF) -> f64 {
    let ax = a.x as f32 as f64;
    let ay = a.y as f32 as f64;
    let bx = b.x as f32 as f64;
    let by = b.y as f32 as f64;
    let vx = bx - ax;
    let vy = by - ay;
    let wx = p.x - ax;
    let wy = p.y - ay;
    let l2 = vx * vx + vy * vy;
    if l2 <= 0.0 {
        return wx * wx + wy * wy;
    }
    let mut t = (wx * vx + wy * vy) / l2;
    if t < 0.0 {
        t = 0.0;
    } else if t > 1.0 {
        t = 1.0;
    }
    let dx = wx - t * vx;
    let dy = wy - t * vy;
    dx * dx + dy * dy
}

// FillAdaptive.cpp:669
// Create a hook based on hook_line and append it to the begin or end of the polyline in the intersection
#[allow(clippy::too_many_arguments)]
fn add_hook(
    intersection_idx: usize,
    intersections: &mut [Intersection],
    lines: &mut [Polyline],
    lines_src: &[Line],
    scaled_offset: f64,
    hook_length: f64,
    scaled_trim_distance: f64,
    rtree: &RTree,
) {
    // FillAdaptive.cpp:673
    if hook_length < SCALED_EPSILON {
        // Ignore open hooks.
        return;
    }

    // (NDEBUG-only assert block omitted.)

    // Trim the hook start by the infill line it will connect to.
    // FillAdaptive.cpp:692
    let mut hook_start = Point::zero();

    let closest_line = intersections[intersection_idx].closest_line;
    let intersect_line = intersections[intersection_idx].intersect_line;

    // FillAdaptive.cpp:694
    let offset_line = create_offset_line(
        lines_src[closest_line],
        &intersections[intersection_idx],
        lines_src,
        scaled_offset,
    );
    let _intersection_found = lines_src[intersect_line]
        .intersection(&offset_line)
        .map(|p| {
            hook_start = p;
            true
        })
        .unwrap_or(false);
    debug_assert!(_intersection_found);

    // FillAdaptive.cpp:699
    let other_hook = intersections[intersection_idx].other_hook(lines);

    // FillAdaptive.cpp:701
    let cl = &lines_src[closest_line];
    let hook_vector_norm =
        PointF::new((cl.b.x - cl.a.x) as f64, (cl.b.y - cl.a.y) as f64).normalize();
    // hook_vector is extended by the thickness of the infill line, so that a collision is found against
    // the infill centerline to be later trimmed by the thickened line.
    // FillAdaptive.cpp:704
    let hv = hook_vector_norm * (hook_length + 1.16 * scaled_trim_distance);
    let hook_vector = Point::new(hv.x as Coord, hv.y as Coord);
    // FillAdaptive.cpp:705
    let hook_forward = Line::new(hook_start, hook_start + hook_vector);

    // FillAdaptive.cpp:707
    let filter_itself = |item: usize| item != intersect_line;

    // FillAdaptive.cpp:709-710
    let mut hook_intersections =
        rtree.query_intersects(hook_forward.a, hook_forward.b, filter_itself);
    // FillAdaptive.cpp:711-712
    let mut self_intersection_point = Point::zero();
    let mut self_intersection = match &other_hook {
        Some(h) => match h.intersection(&hook_forward) {
            Some(p) => {
                self_intersection_point = p;
                true
            }
            None => false,
        },
        None => false,
    };

    // FillAdaptive.cpp:717
    // Find closest intersection of a line segment starting with pt pointing in dir
    // with any of the hook_intersections, returns Euclidian distance. dir is normalized.
    let max_hook_length = |pt: PointF,
                           dir: PointF,
                           hook_intersections: &[RSeg],
                           self_intersection: bool,
                           self_intersection_line: &Option<Line>,
                           self_intersection_point: Point|
     -> f64 {
        // No hook is longer than hook_length, there shouldn't be any intersection closer than that.
        // FillAdaptive.cpp:722
        let mut max_length = hook_length;
        // FillAdaptive.cpp:732
        for hi in hook_intersections {
            // Segment start and end points, segment vector.
            // FillAdaptive.cpp:735 (float-cast as in rtree)
            let pt2 = PointF::new(hi.a.x as f32 as f64, hi.a.y as f32 as f64);
            // FillAdaptive.cpp:736
            let dir2 = PointF::new(hi.b.x as f32 as f64, hi.b.y as f32 as f64) - pt2;
            // Find intersection of (pt, dir) with (pt2, dir2), where dir is normalized.
            // FillAdaptive.cpp:738
            let denom = cross2f(dir, dir2);
            debug_assert!(denom.abs() > EPSILON);
            // FillAdaptive.cpp:740
            let mut t = cross2f(pt2 - pt, dir2) / denom;
            // FillAdaptive.cpp:741
            if hi.idx < lines_src.len() {
                // Trimming by another infill line. Reduce overlap.
                // FillAdaptive.cpp:743 / shift_from_thick_line (728)
                t -= scaled_trim_distance * cross2f(dir, dir2.normalize()).abs();
            }
            // update_max_length
            if t < max_length {
                max_length = t;
            }
        }
        // FillAdaptive.cpp:746
        if self_intersection {
            if let Some(sil) = self_intersection_line {
                let sp = PointF::new(self_intersection_point.x as f64, self_intersection_point.y as f64);
                let v = PointF::new((sil.b.x - sil.a.x) as f64, (sil.b.y - sil.a.y) as f64);
                // FillAdaptive.cpp:747
                let t = (sp - pt).dot(&dir) - scaled_trim_distance * cross2f(dir, v.normalize()).abs();
                max_length = max_length.min(t);
            }
        }
        // FillAdaptive.cpp:750
        max_length.max(0.0)
    };

    // FillAdaptive.cpp:753
    let hook_startf = PointF::new(hook_start.x as f64, hook_start.y as f64);
    // FillAdaptive.cpp:754
    let hook_forward_max_length = max_hook_length(
        hook_startf,
        hook_vector_norm,
        &hook_intersections,
        self_intersection,
        &other_hook,
        self_intersection_point,
    );
    // FillAdaptive.cpp:755
    let mut hook_backward_max_length = 0.0;
    // FillAdaptive.cpp:756
    if hook_forward_max_length < hook_length - SCALED_EPSILON {
        // Try the other side.
        // FillAdaptive.cpp:758
        hook_intersections.clear();
        // FillAdaptive.cpp:759
        let hook_backward = Line::new(hook_start, hook_start - hook_vector);
        // FillAdaptive.cpp:760
        hook_intersections = rtree.query_intersects(hook_backward.a, hook_backward.b, filter_itself);
        // FillAdaptive.cpp:761
        self_intersection = match &other_hook {
            Some(h) => match h.intersection(&hook_backward) {
                Some(p) => {
                    self_intersection_point = p;
                    true
                }
                None => false,
            },
            None => false,
        };
        // FillAdaptive.cpp:762
        hook_backward_max_length = max_hook_length(
            hook_startf,
            -hook_vector_norm,
            &hook_intersections,
            self_intersection,
            &other_hook,
            self_intersection_point,
        );
    }

    // Take the longer hook.
    // FillAdaptive.cpp:766
    let hook_dir = hook_vector_norm
        * if hook_forward_max_length > hook_backward_max_length {
            hook_forward_max_length
        } else {
            -hook_backward_max_length
        };
    // FillAdaptive.cpp:767
    let hook_end = hook_start + Point::new(hook_dir.x as Coord, hook_dir.y as Coord);

    // FillAdaptive.cpp:769
    let intersect_pl = intersections[intersection_idx].intersect_pl;
    let front = intersections[intersection_idx].front;
    let pl = &mut lines[intersect_pl].points;
    // FillAdaptive.cpp:770
    if front {
        // FillAdaptive.cpp:771-772
        *pl.first_mut().unwrap() = hook_start;
        pl.insert(0, hook_end);
    } else {
        // FillAdaptive.cpp:774-775
        *pl.last_mut().unwrap() = hook_start;
        pl.push(hook_end);
    }
}

// FillAdaptive.cpp:800
fn connect_lines_using_hooks(
    mut lines: Vec<Polyline>,
    boundary: &ExPolygon,
    spacing: f64,
    hook_length: f64,
    hook_length_max: f64,
) -> Vec<Polyline> {
    // FillAdaptive.cpp:802
    let mut rtree = RTree::new();
    // FillAdaptive.cpp:803
    let mut poly_idx: usize = 0;

    // 19% overlap, slightly lower than the allowed overlap in Fill::connect_infill()
    // FillAdaptive.cpp:806
    let scaled_offset = (scaled_(spacing) * 0.81) as f32 as f64;
    // 25% overlap
    // FillAdaptive.cpp:808
    let scaled_trim_distance = (scaled_(spacing) * 0.5 * 0.75) as f32 as f64;

    // Keeping the vector of closest points outside the loop, so the vector does not need to be reallocated.
    // (closest reused below)
    // Pairs of lines touching at one end point. The pair is sorted to make the end point connection test symmetric.
    // FillAdaptive.cpp:813 -- store as (lo, hi) polyline indices
    let mut lines_touching_at_endpoints: Vec<(usize, usize)> = Vec::new();
    {
        // Insert infill lines into rtree, merge close collinear segments split by the infill boundary,
        // collect lines_touching_at_endpoints.
        // FillAdaptive.cpp:817
        let r2_close = sqr(1200.0);
        // FillAdaptive.cpp:818
        for poly_i in 0..lines.len() {
            debug_assert!(lines[poly_i].points.len() == 2);
            // FillAdaptive.cpp:820  (&poly != lines.data() => poly_i != 0)
            if poly_i != 0 {
                // Join collinear segments separated by a tiny gap.
                // FillAdaptive.cpp:822 -- returns (Some(other_idx), dist2_min == dist2_front) or (None, false)
                let collinear_segment =
                    |rtree: &mut RTree,
                     lines: &[Polyline],
                     lines_touching_at_endpoints: &mut Vec<(usize, usize)>,
                     pt: Point,
                     pt_other: Point,
                     polyline: usize|
                     -> (Option<usize>, bool) {
                        // FillAdaptive.cpp:824
                        let nearest = match rtree.nearest(pt, |_| true) {
                            Some(s) => s,
                            None => return (None, false),
                        };
                        // FillAdaptive.cpp:825
                        let other = nearest.idx;
                        let op = &lines[other].points;
                        // FillAdaptive.cpp:826-828
                        let dist2_front = sub_sqnorm(*op.first().unwrap(), pt);
                        let dist2_back = sub_sqnorm(*op.last().unwrap(), pt);
                        let dist2_min = dist2_front.min(dist2_back);
                        // FillAdaptive.cpp:829
                        if dist2_min < r2_close {
                            // Don't connect the segments in an opposite direction.
                            // FillAdaptive.cpp:831
                            let dist2_min_other = sub_sqnorm(*op.first().unwrap(), pt_other)
                                .min(sub_sqnorm(*op.last().unwrap(), pt_other));
                            // FillAdaptive.cpp:832
                            if dist2_min_other > dist2_min {
                                // End points of the two lines are very close, they should have been merged together if they are collinear.
                                // FillAdaptive.cpp:834
                                let v1 = PointF::new(
                                    (pt_other.x - pt.x) as f64,
                                    (pt_other.y - pt.y) as f64,
                                );
                                // FillAdaptive.cpp:835
                                let v2 = PointF::new(
                                    (op.last().unwrap().x - op.first().unwrap().x) as f64,
                                    (op.last().unwrap().y - op.first().unwrap().y) as f64,
                                );
                                // FillAdaptive.cpp:836-837
                                let v1n = v1.normalize();
                                let v2n = v2.normalize();
                                // The vectors must not be collinear.
                                // FillAdaptive.cpp:839
                                let d = v1n.dot(&v2n);
                                // FillAdaptive.cpp:840
                                if d.abs() > 0.99_f32 as f64 {
                                    // Lines are collinear, merge them.
                                    // FillAdaptive.cpp:842
                                    rtree.remove(nearest.a, nearest.b, nearest.idx);
                                    return (Some(other), dist2_min == dist2_front);
                                } else {
                                    // FillAdaptive.cpp:845-847
                                    let (lo, hi) = if polyline > other {
                                        (other, polyline)
                                    } else {
                                        (polyline, other)
                                    };
                                    lines_touching_at_endpoints.push((lo, hi));
                                }
                            }
                        }
                        // FillAdaptive.cpp:851
                        (None, false)
                    };
                // FillAdaptive.cpp:853
                let front = *lines[poly_i].points.first().unwrap();
                let back = *lines[poly_i].points.last().unwrap();
                let collinear_front = collinear_segment(
                    &mut rtree,
                    &lines,
                    &mut lines_touching_at_endpoints,
                    front,
                    back,
                    poly_i,
                );
                // FillAdaptive.cpp:854
                let collinear_back = collinear_segment(
                    &mut rtree,
                    &lines,
                    &mut lines_touching_at_endpoints,
                    back,
                    front,
                    poly_i,
                );
                debug_assert!(
                    collinear_front.0.is_none()
                        || collinear_back.0.is_none()
                        || collinear_front.0 != collinear_back.0
                );
                // FillAdaptive.cpp:856
                if let Some(other_idx) = collinear_front.0 {
                    debug_assert!(other_idx != poly_i);
                    // FillAdaptive.cpp:859
                    let new_front = if collinear_front.1 {
                        *lines[other_idx].points.last().unwrap()
                    } else {
                        *lines[other_idx].points.first().unwrap()
                    };
                    lines[poly_i].points[0] = new_front;
                    // FillAdaptive.cpp:860
                    lines[other_idx].points.clear();
                }
                // FillAdaptive.cpp:862
                if let Some(other_idx) = collinear_back.0 {
                    debug_assert!(other_idx != poly_i);
                    // FillAdaptive.cpp:865
                    let new_back = if collinear_back.1 {
                        *lines[other_idx].points.last().unwrap()
                    } else {
                        *lines[other_idx].points.first().unwrap()
                    };
                    let n = lines[poly_i].points.len();
                    lines[poly_i].points[n - 1] = new_back;
                    lines[other_idx].points.clear();
                }
            }
            // FillAdaptive.cpp:869
            rtree.insert(
                *lines[poly_i].points.first().unwrap(),
                *lines[poly_i].points.last().unwrap(),
                poly_idx,
            );
            poly_idx += 1;
        }
    }

    // Convert input polylines to lines_src after the colinear segments were merged.
    // FillAdaptive.cpp:874-877
    let mut lines_src: Vec<Line> = Vec::with_capacity(lines.len());
    for pl in &lines {
        lines_src.push(if pl.empty() {
            Line::new(Point::new(0, 0), Point::new(0, 0))
        } else {
            Line::new(*pl.points.first().unwrap(), *pl.points.last().unwrap())
        });
    }

    // FillAdaptive.cpp:879
    lines_touching_at_endpoints.sort_unstable();
    lines_touching_at_endpoints.dedup();

    // FillAdaptive.cpp:881
    let mut intersections: Vec<Intersection> = Vec::new();
    {
        // Minimum lenght of an infill line to anchor.
        debug_assert!(scaled_offset > scaled_trim_distance);
        // FillAdaptive.cpp:887
        let line_len_threshold_drop_both_sides =
            scaled_offset * (2.0 / (PI / 6.0).cos() + 0.5) + SCALED_EPSILON;
        // FillAdaptive.cpp:888
        let line_len_threshold_anchor_both_sides =
            line_len_threshold_drop_both_sides + scaled_offset;
        // FillAdaptive.cpp:889
        let line_len_threshold_drop_single_side =
            scaled_offset * (1.0 / (PI / 6.0).cos() + 1.5) + SCALED_EPSILON;
        // FillAdaptive.cpp:890
        let line_len_threshold_anchor_single_side =
            line_len_threshold_drop_single_side + scaled_offset;
        // FillAdaptive.cpp:891
        for line_idx in 0..lines.len() {
            // FillAdaptive.cpp:893
            if lines[line_idx].points.is_empty() {
                continue;
            }

            // FillAdaptive.cpp:896-897
            let front_point = *lines[line_idx].points.first().unwrap();
            let back_point = *lines[line_idx].points.last().unwrap();

            // Find the nearest line from the start point of the line.
            // FillAdaptive.cpp:900
            let (tjoint_front, tjoint_back);
            {
                // FillAdaptive.cpp:902 has_tjoint
                let has_tjoint = |rtree: &RTree, lines: &[Polyline], pt: Point| -> Option<usize> {
                    // FillAdaptive.cpp:903 filter_t_joint
                    let filter_t_joint = |item: usize| -> bool {
                        if item != line_idx {
                            let line = &lines_src[item];
                            let v = PointF::new((line.b.x - line.a.x) as f64, (line.b.y - line.a.y) as f64);
                            let va = PointF::new((pt.x - line.a.x) as f64, (pt.y - line.a.y) as f64);
                            let l2 = v.length_squared();
                            if l2 > 0.0 {
                                let t = va.dot(&v);
                                return t > SCALED_EPSILON && t < l2 - SCALED_EPSILON;
                            }
                        }
                        false
                    };
                    // FillAdaptive.cpp:918
                    let closest = rtree.nearest(pt, filter_t_joint);
                    // FillAdaptive.cpp:919
                    let mut out: Option<usize> = None;
                    if let Some(seg) = closest {
                        // FillAdaptive.cpp:921
                        let pl = &lines[seg.idx];
                        if pl.points.is_empty() {
                            // The closest infill line was already dropped as it was too short.
                        } else if pl.size() >= 2
                            && Line::distance_to_squared(
                                pt,
                                *pl.points.first().unwrap(),
                                *pl.points.last().unwrap(),
                            ) <= 1000.0 * 1000.0
                        {
                            // FillAdaptive.cpp:934
                            out = Some(seg.idx);
                        }
                    }
                    out
                };
                // FillAdaptive.cpp:939 filter_end_point_connections
                let filter_end_point_connections =
                    |lines_touching_at_endpoints: &[(usize, usize)], inv: Option<usize>| -> Option<usize> {
                        let mut out: Option<usize> = None;
                        if let Some(in_idx) = inv {
                            let lo_self = line_idx;
                            let (lo, hi) = if lo_self > in_idx {
                                (in_idx, lo_self)
                            } else {
                                (lo_self, in_idx)
                            };
                            // FillAdaptive.cpp:946
                            if lines_touching_at_endpoints.binary_search(&(lo, hi)).is_err() {
                                out = inv;
                            }
                        }
                        out
                    };
                // FillAdaptive.cpp:952-953
                tjoint_front = filter_end_point_connections(
                    &lines_touching_at_endpoints,
                    has_tjoint(&rtree, &lines, front_point),
                );
                tjoint_back = filter_end_point_connections(
                    &lines_touching_at_endpoints,
                    has_tjoint(&rtree, &lines, back_point),
                );
            }

            // FillAdaptive.cpp:956
            let num_tjoints = tjoint_front.is_some() as i32 + tjoint_back.is_some() as i32;
            // FillAdaptive.cpp:957
            if num_tjoints > 0 {
                // FillAdaptive.cpp:958
                let line_len = lines[line_idx].length();
                let drop;
                let anchor;
                // FillAdaptive.cpp:961
                if num_tjoints == 1 {
                    // Connected to perimeters on a single side only.
                    drop = line_len < line_len_threshold_drop_single_side;
                    anchor = line_len > line_len_threshold_anchor_single_side;
                } else {
                    // Not connected to perimeters at all.
                    debug_assert!(num_tjoints == 2);
                    drop = line_len < line_len_threshold_drop_both_sides;
                    anchor = line_len > line_len_threshold_anchor_both_sides;
                }
                // FillAdaptive.cpp:971
                if drop {
                    // Drop a very short line if connected to another infill line.
                    lines[line_idx].points.clear();
                } else if anchor {
                    // FillAdaptive.cpp:977
                    if let Some(tf) = tjoint_front {
                        // T-joint of line's front point with the 'closest' line.
                        intersections.push(Intersection::new(
                            &lines_src, tf, line_idx, line_idx, front_point, true,
                        ));
                    }
                    // FillAdaptive.cpp:982
                    if let Some(tb) = tjoint_back {
                        // T-joint of line's back point with the 'closest' line.
                        intersections.push(Intersection::new(
                            &lines_src, tb, line_idx, line_idx, back_point, false,
                        ));
                    }
                } else {
                    // FillAdaptive.cpp:988
                    if tjoint_front.is_some() {
                        // T joint at the front at a 60 degree angle, the line is very short. Trim the front side.
                        let dir = PointF::new(
                            (back_point.x - front_point.x) as f64,
                            (back_point.y - front_point.y) as f64,
                        )
                        .normalize();
                        let off = dir * (scaled_trim_distance * 1.155);
                        lines[line_idx].points[0] = front_point
                            + Point::new(off.x as Coord, off.y as Coord);
                    }
                    // FillAdaptive.cpp:992
                    if tjoint_back.is_some() {
                        // re-read front in case it changed above
                        let front_now = *lines[line_idx].points.first().unwrap();
                        let dir = PointF::new(
                            (front_now.x - back_point.x) as f64,
                            (front_now.y - back_point.y) as f64,
                        )
                        .normalize();
                        let off = dir * (scaled_trim_distance * 1.155);
                        let n = lines[line_idx].points.len();
                        lines[line_idx].points[n - 1] =
                            back_point + Point::new(off.x as Coord, off.y as Coord);
                    }
                }
            }
        }
        // Remove those intersections, that point to a dropped line.
        // FillAdaptive.cpp:1000
        let mut i = 0;
        while i < intersections.len() {
            debug_assert!(!lines[intersections[i].intersect_line].points.is_empty());
            if lines[intersections[i].closest_line].points.is_empty() {
                let last = intersections.len() - 1;
                intersections.swap(i, last);
                intersections.pop();
            } else {
                i += 1;
            }
        }
    }

    // FillAdaptive.cpp:1023
    // Sort lexicographically by closest_line_idx and left/right orientation.
    intersections.sort_by(|i1, i2| {
        if i1.closest_line == i2.closest_line {
            (i1.left as i32).cmp(&(i2.left as i32))
        } else {
            i1.closest_line.cmp(&i2.closest_line)
        }
    });

    // FillAdaptive.cpp:1030-1031
    let mut merged_with: Vec<usize> = (0..lines.len()).collect();

    // Appends the boundary polygon with all holes to rtree for detection to check whether hooks are not crossing the boundary
    // FillAdaptive.cpp:1034
    {
        // FillAdaptive.cpp:1035
        let mut prev = *boundary.contour.points.last().unwrap();
        // FillAdaptive.cpp:1036
        for &point in &boundary.contour.points {
            rtree.insert(prev, point, poly_idx);
            poly_idx += 1;
            prev = point;
        }
        // FillAdaptive.cpp:1040
        for polygon in &boundary.holes {
            let mut prev = *polygon.points.last().unwrap();
            for &point in &polygon.points {
                rtree.insert(prev, point, poly_idx);
                poly_idx += 1;
                prev = point;
            }
        }
    }

    // FillAdaptive.cpp:1049 update_merged_polyline_idx
    fn update_merged_polyline_idx(merged_with: &mut [usize], pl_idx: usize) -> usize {
        let mut last = pl_idx;
        loop {
            let lower = merged_with[last];
            if lower == last {
                merged_with[pl_idx] = lower;
                return lower;
            }
            last = lower;
        }
    }

    // FillAdaptive.cpp:1062 update_merged_polyline
    fn update_merged_polyline(
        merged_with: &mut [usize],
        lines: &[Polyline],
        intersection: &mut Intersection,
    ) {
        // Update the polyline index to index which is merged
        let intersect_pl_idx = update_merged_polyline_idx(merged_with, intersection.intersect_pl);
        intersection.intersect_pl = intersect_pl_idx;
        // After polylines are merged, it is necessary to update "forward" based on if intersect_point is the first or the last point of intersect_pl.
        if intersection.fresh(lines) {
            intersection.front =
                *lines[intersection.intersect_pl].points.first().unwrap() == intersection.intersect_point;
        }
    }

    // Merge polylines touching at their ends.
    // FillAdaptive.cpp:1075
    for &(pl1_idx0, pl2_idx0) in lines_touching_at_endpoints.iter().rev() {
        let pl1 = pl1_idx0;
        debug_assert!(pl1 < pl2_idx0);
        // FillAdaptive.cpp:1081
        let pl2 = update_merged_polyline_idx(&mut merged_with, pl2_idx0);
        debug_assert!(pl1 <= pl2);
        // FillAdaptive.cpp:1084
        if pl1 != pl2 && !lines[pl1].points.is_empty() && !lines[pl2].points.is_empty() {
            // Merge the polylines.
            debug_assert!(lines[pl1].points.len() >= 2);
            debug_assert!(lines[pl2].points.len() >= 2);
            // FillAdaptive.cpp:1089-1092
            let p1f = *lines[pl1].points.first().unwrap();
            let p1b = *lines[pl1].points.last().unwrap();
            let p2f = *lines[pl2].points.first().unwrap();
            let p2b = *lines[pl2].points.last().unwrap();
            let d11 = sub_sqnorm(p1f, p2f);
            let d12 = sub_sqnorm(p1f, p2b);
            let d21 = sub_sqnorm(p1b, p2f);
            let d22 = sub_sqnorm(p1b, p2b);
            // FillAdaptive.cpp:1093-1094
            let d1min = d11.min(d12);
            let d2min = d21.min(d22);
            // FillAdaptive.cpp:1095
            if d1min < d2min {
                lines[pl1].reverse();
                if d12 == d1min {
                    lines[pl2].reverse();
                }
            } else if d22 == d2min {
                lines[pl2].reverse();
            }
            // FillAdaptive.cpp:1101
            let new_back = {
                let b = *lines[pl1].points.last().unwrap();
                let f = *lines[pl2].points.first().unwrap();
                Point::new((b.x + f.x) / 2, (b.y + f.y) / 2)
            };
            let n = lines[pl1].points.len();
            lines[pl1].points[n - 1] = new_back;
            // FillAdaptive.cpp:1102
            let tail: Vec<Point> = lines[pl2].points[1..].to_vec();
            lines[pl1].points.extend_from_slice(&tail);
            // FillAdaptive.cpp:1103
            lines[pl2].points.clear();
            // FillAdaptive.cpp:1104
            merged_with[pl2] = pl1;
        }
    }

    // FillAdaptive.cpp:1109
    // Keep intersect_line outside the loop, so it does not get reallocated.
    let mut intersect_line: Vec<(usize, f64)> = Vec::new();
    // FillAdaptive.cpp:1110
    let mut min_idx = 0usize;
    while min_idx < intersections.len() {
        // FillAdaptive.cpp:1111
        intersect_line.clear();
        // All the nearest points (T-joints) ending at the same line are projected onto this line.
        {
            // FillAdaptive.cpp:1114
            let cl = &lines_src[intersections[min_idx].closest_line];
            let line_dir = PointF::new((cl.b.x - cl.a.x) as f64, (cl.b.y - cl.a.y) as f64);
            // FillAdaptive.cpp:1115
            let mut max_idx = min_idx;
            // FillAdaptive.cpp:1116-1120
            while max_idx < intersections.len()
                && intersections[min_idx].closest_line == intersections[max_idx].closest_line
                && intersections[min_idx].left == intersections[max_idx].left
            {
                let ip = intersections[max_idx].intersect_point;
                let proj = line_dir.dot(&PointF::new(ip.x as f64, ip.y as f64));
                intersect_line.push((max_idx, proj));
                max_idx += 1;
            }
            // FillAdaptive.cpp:1121
            min_idx = max_idx;
            debug_assert!(!intersect_line.is_empty());
            // Sort the intersections along line_dir.
            // FillAdaptive.cpp:1124
            intersect_line.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        }

        // FillAdaptive.cpp:1127
        if intersect_line.len() == 1 {
            // Simple case: The current intersection is the only one touching its adjacent line.
            // FillAdaptive.cpp:1129
            let first_i_idx = intersect_line[0].0;
            update_merged_polyline(&mut merged_with, &lines, &mut intersections[first_i_idx]);
            // FillAdaptive.cpp:1131
            if intersections[first_i_idx].fresh(&lines) {
                // Try to connect left or right.
                add_hook(
                    first_i_idx,
                    &mut intersections,
                    &mut lines,
                    &lines_src,
                    scaled_offset,
                    hook_length,
                    scaled_trim_distance,
                    &rtree,
                );
                // FillAdaptive.cpp:1141
                intersections[first_i_idx].used = true;
            }
            continue;
        }

        // FillAdaptive.cpp:1146
        for first_idx in 0..intersect_line.len() {
            let first_i_idx = intersect_line[first_idx].0;
            // FillAdaptive.cpp:1148
            update_merged_polyline(&mut merged_with, &lines, &mut intersections[first_i_idx]);
            // FillAdaptive.cpp:1149
            if !intersections[first_i_idx].fresh(&lines) {
                continue;
            }

            // Get the previous or next intersection on the same line, pick the closer one.
            // FillAdaptive.cpp:1154
            if first_idx > 0 {
                let prev_idx = intersect_line[first_idx - 1].0;
                update_merged_polyline(&mut merged_with, &lines, &mut intersections[prev_idx]);
            }
            // FillAdaptive.cpp:1156
            if first_idx + 1 < intersect_line.len() {
                let next_idx = intersect_line[first_idx + 1].0;
                update_merged_polyline(&mut merged_with, &lines, &mut intersections[next_idx]);
            }
            // FillAdaptive.cpp:1158
            let nearest_i_idx =
                get_nearest_intersection(&intersect_line, &intersections, &lines, first_idx);
            debug_assert!(
                intersections[first_i_idx].closest_line == intersections[nearest_i_idx].closest_line
            );

            // A line between two intersections points
            // FillAdaptive.cpp:1164
            let offset_line = create_offset_line(
                Line::new(
                    intersections[first_i_idx].intersect_point,
                    intersections[nearest_i_idx].intersect_point,
                ),
                &intersections[first_i_idx],
                &lines_src,
                scaled_offset,
            );
            // Check if both intersections lie on the offset_line and simultaneously get their points of intersecting.
            // FillAdaptive.cpp:1167
            let mut first_i_point = Point::zero();
            let mut nearest_i_point = Point::zero();
            // FillAdaptive.cpp:1168
            let mut could_connect = false;
            // FillAdaptive.cpp:1169
            if intersections[nearest_i_idx].fresh(&lines) {
                let a = lines_src[intersections[first_i_idx].intersect_line]
                    .intersection(&offset_line);
                let b = lines_src[intersections[nearest_i_idx].intersect_line]
                    .intersection(&offset_line);
                could_connect = match (a, b) {
                    (Some(pa), Some(pb)) => {
                        first_i_point = pa;
                        nearest_i_point = pb;
                        true
                    }
                    _ => false,
                };
            }
            // FillAdaptive.cpp:1176
            could_connect &= sub_sqnorm(nearest_i_point, first_i_point) <= sqr(hook_length_max);
            // FillAdaptive.cpp:1177
            if could_connect {
                // Both intersections are so close that their polylines can be connected.
                // Verify that no other infill line intersects this anchor line.
                // FillAdaptive.cpp:1181
                let il_first = intersections[first_i_idx].intersect_line;
                let il_near = intersections[nearest_i_idx].intersect_line;
                let closest = rtree.query_intersects(first_i_point, nearest_i_point, |item| {
                    item != il_first && item != il_near
                });
                // FillAdaptive.cpp:1187
                could_connect = closest.is_empty();
            }
            // FillAdaptive.cpp:1196
            let mut connected = false;
            // FillAdaptive.cpp:1197
            if could_connect {
                // No other infill line intersects this anchor line. Extrude it as a whole.
                // FillAdaptive.cpp:1202
                if intersections[first_i_idx].intersect_pl == intersections[nearest_i_idx].intersect_pl
                {
                    // Both intersections are on the same polyline, that means a loop is being closed.
                    debug_assert!(
                        intersections[first_i_idx].front != intersections[nearest_i_idx].front
                    );
                    // FillAdaptive.cpp:1205
                    if !intersections[first_i_idx].front {
                        std::mem::swap(&mut first_i_point, &mut nearest_i_point);
                    }
                    let pl = intersections[first_i_idx].intersect_pl;
                    // FillAdaptive.cpp:1207-1210
                    let fp = &mut lines[pl].points;
                    *fp.first_mut().unwrap() = first_i_point;
                    *fp.last_mut().unwrap() = nearest_i_point;
                    fp.insert(0, nearest_i_point);
                } else {
                    // Both intersections are on different polylines
                    // FillAdaptive.cpp:1213
                    let mut l = Line::new(first_i_point, nearest_i_point);
                    let cl = &lines_src[intersections[first_i_idx].closest_line];
                    let dir = PointF::new((cl.b.x - cl.a.x) as f64, (cl.b.y - cl.a.y) as f64)
                        .normalize();
                    let perp_dir = dir.perp();
                    let off = if intersections[first_i_idx].left {
                        scaled_trim_distance
                    } else {
                        -scaled_trim_distance
                    };
                    l = l.translate(Point::new(
                        (perp_dir.x * off) as Coord,
                        (perp_dir.y * off) as Coord,
                    ));
                    // FillAdaptive.cpp:1215-1217
                    let mut pt_start = Point::zero();
                    let mut pt_end = Point::zero();
                    let first_pl = intersections[first_i_idx].intersect_pl;
                    let near_pl = intersections[nearest_i_idx].intersect_pl;
                    let trim_start = lines[first_pl].points.len() == 3
                        && intersections[first_i_idx].other_hook_intersects_pt(
                            &lines, &l, &mut pt_start,
                        );
                    let trim_end = lines[near_pl].points.len() == 3
                        && intersections[nearest_i_idx].other_hook_intersects_pt(
                            &lines, &l, &mut pt_end,
                        );
                    // FillAdaptive.cpp:1218
                    let second_points = lines[near_pl].points.clone();
                    // FillAdaptive.cpp:1219
                    if intersections[first_i_idx].front {
                        lines[first_pl].points.reverse();
                    }
                    // FillAdaptive.cpp:1221
                    if trim_start {
                        lines[first_pl].points[0] = pt_start;
                    }
                    // FillAdaptive.cpp:1223
                    let n = lines[first_pl].points.len();
                    lines[first_pl].points[n - 1] = first_i_point;
                    // FillAdaptive.cpp:1224
                    lines[first_pl].points.push(nearest_i_point);
                    // FillAdaptive.cpp:1225
                    if intersections[nearest_i_idx].front {
                        lines[first_pl]
                            .points
                            .extend_from_slice(&second_points[1..]);
                    } else {
                        let rev: Vec<Point> =
                            second_points.iter().rev().skip(1).copied().collect();
                        lines[first_pl].points.extend_from_slice(&rev);
                    }
                    // FillAdaptive.cpp:1229
                    if trim_end {
                        let n = lines[first_pl].points.len();
                        lines[first_pl].points[n - 1] = pt_end;
                    }
                    // Keep the polyline at the lower index slot.
                    // FillAdaptive.cpp:1232
                    if first_pl < near_pl {
                        lines[near_pl].points.clear();
                        merged_with[near_pl] = first_pl;
                    } else {
                        let moved = lines[first_pl].points.clone();
                        lines[near_pl].points = moved;
                        lines[first_pl].points.clear();
                        merged_with[first_pl] = near_pl;
                    }
                }
                // FillAdaptive.cpp:1241-1242
                intersections[nearest_i_idx].used = true;
                connected = true;
            }
            // FillAdaptive.cpp:1247
            if !connected {
                // Try to connect left or right.
                add_hook(
                    first_i_idx,
                    &mut intersections,
                    &mut lines,
                    &lines_src,
                    scaled_offset,
                    hook_length,
                    scaled_trim_distance,
                    &rtree,
                );
            }
            // FillAdaptive.cpp:1260
            intersections[first_i_idx].used = true;
        }
    }

    // FillAdaptive.cpp:1264
    let mut polylines_out: Vec<Polyline> = Vec::new();
    polylines_out.reserve(lines.iter().filter(|pl| !pl.empty()).count());
    // FillAdaptive.cpp:1266
    for pl in lines.drain(..) {
        if !pl.empty() {
            polylines_out.push(pl);
        }
    }
    polylines_out
}

#[inline]
fn sub_sqnorm(a: Point, b: Point) -> f64 {
    let dx = (a.x - b.x) as f64;
    let dy = (a.y - b.y) as f64;
    dx * dx + dy * dy
}

// scale_(val) in BambuStudio == val / SCALING_FACTOR (a double). Here SCALING_FACTOR
// is 1e-5 in C++, i.e. scale_ multiplies by 1e5. The crate's `scaled()` rounds to i64;
// for these float thresholds we need the un-rounded scaled double, matching C++.
#[inline]
fn scaled_(v: f64) -> f64 {
    v * crate::SCALING_FACTOR
}

// FillAdaptive.cpp:1431
fn make_cubes_properties(mut max_cube_edge_length: f64, line_spacing: f64) -> Vec<CubeProperties> {
    // FillAdaptive.cpp:1433
    max_cube_edge_length += EPSILON;

    // FillAdaptive.cpp:1435
    let mut cubes_properties: Vec<CubeProperties> = Vec::new();
    // FillAdaptive.cpp:1436
    let mut edge_length = line_spacing * 2.0;
    loop {
        // FillAdaptive.cpp:1438-1443
        let props = CubeProperties {
            edge_length,
            height: edge_length * 3.0_f64.sqrt(),
            diagonal_length: edge_length * 2.0_f64.sqrt(),
            line_z_distance: edge_length / 3.0_f64.sqrt(),
            line_xy_distance: edge_length / 6.0_f64.sqrt(),
        };
        // FillAdaptive.cpp:1444
        cubes_properties.push(props);
        // FillAdaptive.cpp:1445
        if edge_length > max_cube_edge_length {
            break;
        }
        edge_length *= 2.0;
    }
    cubes_properties
}

// FillAdaptive.cpp:1451
fn is_overhang_triangle(a: Vec3d, b: Vec3d, c: Vec3d, up: Vec3d) -> bool {
    // Calculate triangle normal.
    // FillAdaptive.cpp:1454
    let n = (b - a).cross(&(c - b));
    // FillAdaptive.cpp:1455
    n.dot(&up) > 0.707 * n.norm()
}

// FillAdaptive.cpp:1458
fn transform_center(pool: &mut [Cube], current_cube: usize, rot: &nalgebra::Matrix3<f64>) {
    // FillAdaptive.cpp:1461
    pool[current_cube].center_octree = pool[current_cube].center;
    // FillAdaptive.cpp:1463
    let c = ev(pool[current_cube].center);
    pool[current_cube].center = cv(rot * c);
    // FillAdaptive.cpp:1464
    let children = pool[current_cube].children;
    for &child in &children {
        if child != NULL_CUBE {
            transform_center(pool, child, rot);
        }
    }
}

// FillAdaptive.hpp:41 / FillAdaptive.cpp:1469
pub fn build_octree(
    // Mesh is rotated to the coordinate system of the octree.
    triangle_mesh: &IndexedTriangleSet,
    // Overhang triangles extracted from fill surfaces with stInternalBridge type,
    // rotated to the coordinate system of the octree.
    overhang_triangles: &[Vec3d],
    line_spacing: f64,
    support_overhangs_only: bool,
) -> Octree {
    // FillAdaptive.cpp:1478-1479
    debug_assert!(line_spacing > 0.0);
    debug_assert!(!line_spacing.is_nan());

    // FillAdaptive.cpp:1481  BoundingBox3Base<Vec3f>(triangle_mesh.vertices)
    let mut bmin = Vec3d::new(f64::MAX, f64::MAX, f64::MAX);
    let mut bmax = Vec3d::new(f64::MIN, f64::MIN, f64::MIN);
    for v in &triangle_mesh.vertices {
        // C++ stores float vertices; cast through f32 to match precision.
        let vf = Vec3d::new(v.x as f32 as f64, v.y as f32 as f64, v.z as f32 as f64);
        bmin = bmin.min(&vf);
        bmax = bmax.max(&vf);
    }
    // FillAdaptive.cpp:1482  bbox.center().cast<double>()
    let cube_center = Vec3d::new(
        ((bmin.x + bmax.x) * 0.5) as f32 as f64,
        ((bmin.y + bmax.y) * 0.5) as f32 as f64,
        ((bmin.z + bmax.z) * 0.5) as f32 as f64,
    );
    // FillAdaptive.cpp:1483  bbox.size().maxCoeff()
    let size = bmax - bmin;
    let size_max = (size.x as f32).max(size.y as f32).max(size.z as f32) as f64;
    let cubes_properties = make_cubes_properties(size_max, line_spacing);
    // FillAdaptive.cpp:1484
    let mut octree = Octree::new(cube_center, cubes_properties);

    // FillAdaptive.cpp:1486
    if octree.cubes_properties.len() > 1 {
        // FillAdaptive.cpp:1488
        let edge_length_half = 0.5 * octree.cubes_properties.last().unwrap().edge_length;
        // FillAdaptive.cpp:1489
        let diag_half = Vec3d::new(edge_length_half, edge_length_half, edge_length_half);
        // FillAdaptive.cpp:1490
        let max_depth = octree.cubes_properties.len() as i32 - 1;
        // FillAdaptive.cpp:1491 process_triangle inlined below.

        // FillAdaptive.cpp:1498
        let up_vector = if support_overhangs_only {
            cv(transform_to_octree() * ev(Vec3d::new(0.0, 0.0, 1.0)))
        } else {
            Vec3d::zero()
        };
        // FillAdaptive.cpp:1499
        for tri in &triangle_mesh.triangles {
            // FillAdaptive.cpp:1500-1502
            let a = vert_d(&triangle_mesh.vertices[tri[0]]);
            let b = vert_d(&triangle_mesh.vertices[tri[1]]);
            let c = vert_d(&triangle_mesh.vertices[tri[2]]);
            // FillAdaptive.cpp:1503
            if !support_overhangs_only || is_overhang_triangle(a, b, c, up_vector) {
                // process_triangle (FillAdaptive.cpp:1492)
                let root = octree.root_cube;
                let bbox = BoundingBoxf3::new(
                    octree.pool[root].center - diag_half,
                    octree.pool[root].center + diag_half,
                );
                octree.insert_triangle(a, b, c, root, &bbox, max_depth);
            }
        }
        // FillAdaptive.cpp:1506
        let mut i = 0;
        while i < overhang_triangles.len() {
            let root = octree.root_cube;
            let bbox = BoundingBoxf3::new(
                octree.pool[root].center - diag_half,
                octree.pool[root].center + diag_half,
            );
            octree.insert_triangle(
                overhang_triangles[i],
                overhang_triangles[i + 1],
                overhang_triangles[i + 2],
                root,
                &bbox,
                max_depth,
            );
            i += 3;
        }
        // FillAdaptive.cpp:1508
        {
            // Transform the octree to world coordinates to reduce computation when extracting infill lines.
            // FillAdaptive.cpp:1510
            let rot = transform_to_world().to_rotation_matrix().into_inner();
            let root = octree.root_cube;
            transform_center(&mut octree.pool, root, &rot);
            // FillAdaptive.cpp:1512
            octree.origin = cv(rot * ev(octree.origin));
        }
    }

    // FillAdaptive.cpp:1516
    octree
}

#[inline]
fn vert_d(v: &Vec3d) -> Vec3d {
    // triangle_mesh.vertices[tri[i]].cast<double>() -- vertices are float in C++.
    Vec3d::new(v.x as f32 as f64, v.y as f32 as f64, v.z as f32 as f64)
}

/// Public entry mirroring the (blocked) `Filler::_fill_surface_single` line-generation
/// + hook-connection pipeline, operating on an already-built [`Octree`] and the
/// caller-supplied `z`, `spacing`, and anchor lengths. This contains everything
/// from `_fill_surface_single` that does not depend on the absent `Fill` base class
/// state (multiline_fill / params plumbing / connect_infill dispatch).
///
/// FillAdaptive.cpp:1320 (line-generation core)
pub fn generate_infill_lines(
    octree: &Octree,
    z: f64,
    expolygon: &ExPolygon,
    spacing: f64,
    hook_length: f64,
    hook_length_max: f64,
    dont_connect: bool,
) -> Vec<Polyline> {
    // FillAdaptive.cpp:1329
    let mut all_polylines: Vec<Polyline>;
    {
        // 3 contexts for three directions of infill lines
        // FillAdaptive.cpp:1332
        let mut contexts = [
            FillContext::new(octree, z, 0),
            FillContext::new(octree, z, 1),
            FillContext::new(octree, z, 2),
        ];
        // Generate the infill lines along the octree cells, merge touching lines of the same direction.
        // FillAdaptive.cpp:1338
        let mut num_lines = 0usize;
        // FillAdaptive.cpp:1339
        for context in contexts.iter_mut() {
            generate_infill_lines_recursive(
                context,
                &octree.pool,
                octree.root_cube,
                0,
                octree.cubes_properties.len() as i32 - 1,
            );
            num_lines += context.output_lines.len() + context.temp_lines.len();
        }

        // Collect the lines.
        // FillAdaptive.cpp:1361
        let mut lines: Vec<Line> = Vec::with_capacity(num_lines);
        for context in contexts.iter() {
            lines.extend_from_slice(&context.output_lines);
            // FillAdaptive.cpp:1365
            for line in &context.temp_lines {
                if line.a.x() != Coord::MAX {
                    lines.push(*line);
                }
            }
        }
        // Convert lines to polylines.
        // FillAdaptive.cpp:1370-1371
        all_polylines = Vec::with_capacity(lines.len());
        for l in &lines {
            all_polylines.push(Polyline::from_points(vec![l.a, l.b]));
        }

        // NOTE: multiline_fill(all_polylines, params, spacing) is part of the blocked
        // _fill_surface_single (needs FillParams); omitted here. (FillAdaptive.cpp:1374)

        // Crop all polylines
        // FillAdaptive.cpp:1377
        all_polylines = crate::clipper_utils::intersection_pl(
            &all_polylines,
            std::slice::from_ref(expolygon),
        );
    }

    // After intersection_pl some polylines with only one line are split into more lines
    // FillAdaptive.cpp:1382
    for polyline in all_polylines.iter_mut() {
        // FillAdaptive.cpp:1384
        if polyline.points.len() > 2 {
            // erase(begin+1, end-1)
            let last = polyline.points.len() - 1;
            let kept_last = polyline.points[last];
            polyline.points.truncate(1);
            polyline.points.push(kept_last);
        }
    }

    // FillAdaptive.cpp:1399
    let mut all_polylines_with_hooks = if all_polylines.len() > 1 {
        connect_lines_using_hooks(all_polylines, expolygon, spacing, hook_length, hook_length_max)
    } else {
        all_polylines
    };

    // FillAdaptive.cpp:1408
    let mut polylines_out: Vec<Polyline> = Vec::new();
    if dont_connect || all_polylines_with_hooks.len() <= 1 {
        // FillAdaptive.cpp:1409
        let chained = chain_polylines(std::mem::take(&mut all_polylines_with_hooks), None);
        polylines_out.extend(chained);
    } else {
        // connect_infill — part of the blocked path (needs FillParams). We chain as a
        // faithful fallback for the line-generation entry point.
        // FillAdaptive.cpp:1411
        let chained = chain_polylines(std::mem::take(&mut all_polylines_with_hooks), None);
        polylines_out.extend(chained);
    }
    polylines_out
}
