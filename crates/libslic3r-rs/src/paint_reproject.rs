//! Port of `PaintReproject.hpp` / `PaintReproject.cpp`: carrying the painted facet annotations of
//! a volume (support, seam, multi-material, fuzzy skin) over to a rebuilt mesh.
//!
//! C++ Reference:
//! - src/libslic3r/PaintReproject.hpp
//! - src/libslic3r/PaintReproject.cpp
//!
//! In the order of the .cpp: the 2D planar helpers (33-139), `PaintReprojector` (141-649),
//! `PaintRegionSource` (651-921), `reproject_one_annotation` (923-1095), `RegionEdgeLimit`
//! (1097-1168), `DestinationInSourceFrame` (1170-1210), `DestinationSeedIndex` and the region
//! reprojection (1212-1382), `count_valid_nodes`, the cut classification and splicing (1385-1576),
//! the entry points `reproject_paint` (1582-1723) and `reproject_paint_geometric` (1725-1872), and
//! the boolean/merge path (1874-2425). Compared with the pin in
//! tests/paint_reproject_native_parity.rs.
//!
//! The float `AABBTreeIndirect::Tree3f` queries go through the float tree of
//! `aabb_tree_indirect.rs` (`TreeF`). `ModelVolume` is a stand-in with the members
//! `reproject_paint` reads and writes. The progress and cancel callbacks are `FnMut` trait objects;
//! an empty `std::function` is `None`.
//!
//! Floating point follows the reference build (clang, `-ffp-contract=on`): a fused operation
//! inside one source expression is written `mul_add`; Eigen's operators do not fuse across their
//! calls, except where noted (measured: the affine product, the float cross product).
//!
//! A `const TriangleMesh &` of the C++ is read here only through its `its`, which is what these
//! functions take.

use crate::aabb_tree_indirect::{build_aabb_tree_over_indexed_triangle_set_f, intersecting_f, squared_distance_to_indexed_triangle_set_f, traverse_f, BoundingBox3f, NodeF, TreeF};
use crate::geometry::geometry::is_approx;
use crate::geometry::{Point3F, Transformation};
use crate::libslic3r::EPSILON;
use crate::model::FacetsAnnotation;
use crate::triangle_mesh::{indexed_triangle_set, its_face_neighbors, Triangle as MeshTriangle, TriangleMesh, Vec3f, Vec3i};
use crate::triangle_selector::{
    EnforcerBlockerType, FacetSubdivisionCancelCallback, FacetSubdivisionMeasurement, FacetSubdivisionResult, RegionCoverage, SerializedData, Transform3f, TriangleSelector,
};
use crate::triangle_set_sampling::{eigen_cross3f, eigen_dot3f, eigen_squared_norm3f};

/// `Vec2d`
pub type Vec2d = nalgebra::Vector2<f64>;
/// `Vec3d`
pub type Vec3d = nalgebra::Vector3<f64>;
/// `Transform3d` (an affine `Eigen::Transform<double, 3>`, its 4x4 matrix)
pub type Transform3d = nalgebra::Matrix4<f64>;

/// PaintReproject.cpp:29-31: a serialized annotation: per-root entries of (root index, first bit)
/// plus the concatenated depth-first bit streams of those roots.
pub type AnnotationData = SerializedData;

/// `Slic3r::Axis` (libslic3r.h): the axes `project_to_2d` and `dominant_projection_axis` name
pub const X: i32 = 0;
pub const Y: i32 = 1;
pub const Z: i32 = 2;

// ---------------------------------------------------------------------------
// 2D planar helpers (shared by the coplanar-overlap sampling path).
// ---------------------------------------------------------------------------

/// PaintReproject.cpp:37-40
pub fn cross_2d(a: &Vec2d, b: &Vec2d) -> f64 {
    a.x.mul_add(b.y, -(a.y * b.x))
}

/// PaintReproject.cpp:42-49: the point with `dropped_axis` left out
pub fn project_to_2d(point: &Vec3f, dropped_axis: i32) -> Vec2d {
    match dropped_axis {
        X => Vec2d::new(point.y as f64, point.z as f64),
        Y => Vec2d::new(point.x as f64, point.z as f64),
        _ => Vec2d::new(point.x as f64, point.y as f64),
    }
}

/// PaintReproject.cpp:51-57
pub fn polygon_signed_area(polygon: &[Vec2d]) -> f64 {
    let mut area = 0.0f64;
    for i in 0..polygon.len() {
        area += cross_2d(&polygon[i], &polygon[(i + 1) % polygon.len()]);
    }
    0.5 * area
}

/// Eigen `squaredNorm()` of a `Vec2d`: `x * x + y * y`, not fused
#[inline]
fn squared_norm_2d(v: &Vec2d) -> f64 {
    let (xx, yy) = (v.x * v.x, v.y * v.y);
    xx + yy
}

/// PaintReproject.cpp:59-103: the polygon clipped by the triangle (Sutherland-Hodgman, the
/// triangle made counter-clockwise first)
pub fn clip_polygon_by_triangle(mut polygon: Vec<Vec2d>, mut clip: [Vec2d; 3]) -> Vec<Vec2d> {
    let clip_signed_area = 0.5 * (cross_2d(&clip[0], &clip[1]) + cross_2d(&clip[1], &clip[2]) + cross_2d(&clip[2], &clip[0]));
    if clip_signed_area < 0.0 {
        clip.swap(1, 2);
    }

    let mut edge_idx = 0;
    while edge_idx < 3 && !polygon.is_empty() {
        let edge_start = clip[edge_idx];
        let edge_end = clip[(edge_idx + 1) % 3];
        let edge = edge_end - edge_start;
        let edge_epsilon = 1e-12f64.max(squared_norm_2d(&edge) * 1e-12);
        let signed_distance = |point: &Vec2d| -> f64 { cross_2d(&edge, &(point - edge_start)) };
        let inside = |point: &Vec2d| -> bool { signed_distance(point) >= -edge_epsilon };
        let intersection = |from: &Vec2d, to: &Vec2d| -> Vec2d {
            let from_distance = signed_distance(from);
            let to_distance = signed_distance(to);
            let denom = from_distance - to_distance;
            if denom.abs() <= edge_epsilon {
                return if from_distance.abs() <= to_distance.abs() { *from } else { *to };
            }
            let q = from_distance / denom;
            // std::clamp(q, 0.0, 1.0)
            let t = if q < 0.0 {
                0.0
            } else if 1.0 < q {
                1.0
            } else {
                q
            };
            // (from + t * (to - from)).eval(), coefficient by coefficient, not fused
            let d = to - from;
            let (tx, ty) = (t * d.x, t * d.y);
            Vec2d::new(from.x + tx, from.y + ty)
        };

        let mut output: Vec<Vec2d> = Vec::with_capacity(polygon.len() + 1);
        let mut previous = *polygon.last().expect("a non-empty polygon");
        let mut previous_inside = inside(&previous);
        for current in polygon.iter() {
            let current_inside = inside(current);
            if current_inside != previous_inside {
                output.push(intersection(&previous, current));
            }
            if current_inside {
                output.push(*current);
            }
            previous = *current;
            previous_inside = current_inside;
        }
        polygon = output;
        edge_idx += 1;
    }
    polygon
}

/// PaintReproject.cpp:105-119: the area of the overlap of two triangles projected along
/// `dropped_axis`
pub fn triangle_overlap_area(lhs: &[Vec3f; 3], rhs: &[Vec3f; 3], dropped_axis: i32) -> f64 {
    let lhs_polygon: Vec<Vec2d> = vec![project_to_2d(&lhs[0], dropped_axis), project_to_2d(&lhs[1], dropped_axis), project_to_2d(&lhs[2], dropped_axis)];
    let rhs_triangle: [Vec2d; 3] = [project_to_2d(&rhs[0], dropped_axis), project_to_2d(&rhs[1], dropped_axis), project_to_2d(&rhs[2], dropped_axis)];
    let overlap = clip_polygon_by_triangle(lhs_polygon, rhs_triangle);
    if overlap.len() < 3 {
        0.0
    } else {
        polygon_signed_area(&overlap).abs()
    }
}

/// Eigen `a.cross(b)` of two `Vec3f`: each component `fma(a_i, b_j, -(a_j * b_i))` (measured)
#[inline]
fn cross_f32(a: &Vec3f, b: &Vec3f) -> Vec3f {
    Vec3f::new(a.y.mul_add(b.z, -(a.z * b.y)), a.z.mul_add(b.x, -(a.x * b.z)), a.x.mul_add(b.y, -(a.y * b.x)))
}

/// PaintReproject.cpp:121-129: the axis to drop when projecting a triangle to 2D, the one its
/// normal points along most strongly
pub fn dominant_projection_axis(triangle: &[Vec3f; 3]) -> i32 {
    let n = cross_f32(&(triangle[1] - triangle[0]), &(triangle[2] - triangle[0]));
    let normal = Vec3f::new(n.x.abs(), n.y.abs(), n.z.abs());
    if normal.x >= normal.y && normal.x >= normal.z {
        return X;
    }
    if normal.y >= normal.z {
        return Y;
    }
    Z
}

/// Eigen `transformation * v` of an affine `Transform3d` and a `Vec3d`: per row
/// `fma(m2, v2, fma(m1, v1, m0 * v0)) + m3`, as `triangle_mesh_slicer::transform3f_mul_vec3f`
/// measures it in float
#[inline]
fn transform3d_mul_vec3d(t: &Transform3d, v: [f64; 3]) -> [f64; 3] {
    let mut out = [0f64; 3];
    for (r, o) in out.iter_mut().enumerate() {
        *o = t[(r, 2)].mul_add(v[2], t[(r, 1)].mul_add(v[1], t[(r, 0)] * v[0])) + t[(r, 3)];
    }
    out
}

/// `(transformation * point.cast<double>()).cast<float>()`
#[inline]
fn transform_point(point: &Vec3f, transformation: &Transform3d) -> Vec3f {
    let p = transform3d_mul_vec3d(transformation, [point.x as f64, point.y as f64, point.z as f64]);
    Vec3f::new(p[0] as f32, p[1] as f32, p[2] as f32)
}

/// PaintReproject.cpp:131-139
pub fn transform_triangle(triangle: &[Vec3f; 3], transformation: &Transform3d) -> [Vec3f; 3] {
    [transform_point(&triangle[0], transformation), transform_point(&triangle[1], transformation), transform_point(&triangle[2], transformation)]
}

// ---------------------------------------------------------------------------
// Callbacks, float/double Eigen helpers and the selector of an indexed triangle set.
// ---------------------------------------------------------------------------

/// PaintReproject.hpp:15-16 `using PaintReprojectProgressCallback = std::function<void(int percent, const char *message)>;`:
/// progress in [0, 100]; the message (`nullptr` is `None`) is a short, non-localized stage description.
pub type PaintReprojectProgressCallback<'a> = dyn FnMut(i32, Option<&str>) + 'a;
/// PaintReproject.hpp:17-18 `using PaintReprojectCancelCallback = std::function<bool()>;`: returns
/// true to request early termination. An empty `std::function` is `None` where one is taken.
pub type PaintReprojectCancelCallback<'a> = dyn FnMut() -> bool + 'a;

/// `Vec3f::norm()`: the square root of the unfused squared norm
#[inline]
fn norm_f32(v: &Vec3f) -> f32 {
    eigen_squared_norm3f(v).sqrt()
}

/// `Vec3d::cross`, each component `fma(a, b, -(c * d))` (the order found for `Vec3f`, R876)
#[inline]
fn cross_f64(a: &Vec3d, b: &Vec3d) -> Vec3d {
    Vec3d::new(a.y.mul_add(b.z, -(a.z * b.y)), a.z.mul_add(b.x, -(a.x * b.z)), a.x.mul_add(b.y, -(a.y * b.x)))
}

/// `Vec3d::dot`, `fma(a.z, b.z, a.x * b.x + a.y * b.y)` (the order found for `Vec3f`, R880)
#[inline]
fn dot_f64(a: &Vec3d, b: &Vec3d) -> f64 {
    a.z.mul_add(b.z, a.x * b.x + a.y * b.y)
}

/// `Vec3d::norm()`: the square root of `(x * x + y * y) + z * z`, unfused
#[inline]
fn norm_f64(v: &Vec3d) -> f64 {
    ((v.x * v.x + v.y * v.y) + v.z * v.z).sqrt()
}

/// `(a + b + c) / 3.f` of three `Vec3f`, coefficient by coefficient
#[inline]
fn centroid_f32(a: &Vec3f, b: &Vec3f, c: &Vec3f) -> Vec3f {
    let s = (a + b) + c;
    Vec3f::new(s.x / 3.0f32, s.y / 3.0f32, s.z / 3.0f32)
}

/// `0.5 * double((t[1] - t[0]).cross(t[2] - t[0]).norm())`
#[inline]
fn triangle_area_f32(t: &[Vec3f; 3]) -> f64 {
    0.5 * norm_f32(&eigen_cross3f(&(t[1] - t[0]), &(t[2] - t[0]))) as f64
}

/// `std::max(a, b)` of two doubles: `a < b ? b : a`
#[inline]
fn std_max_f64(a: f64, b: f64) -> f64 {
    if a < b {
        b
    } else {
        a
    }
}

/// `std::min(a, b)` of two doubles: `b < a ? b : a`
#[inline]
fn std_min_f64(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// `TriangleSelector(const TriangleMesh &mesh)` over the indexed triangle set of the mesh (the
/// default edge limit 0.6f).
fn selector_of(its: &indexed_triangle_set) -> TriangleSelector {
    TriangleSelector::new(
        TriangleMesh::from_parts(
            its.vertices.iter().map(|v| Point3F::new(v.x as f64, v.y as f64, v.z as f64)).collect(),
            its.indices.iter().map(|f| MeshTriangle::new(f[0] as u32, f[1] as u32, f[2] as u32)).collect(),
        ),
        0.6,
    )
}

/// `TriangleSelector::deserialize(data, needs_reset)` with the default `max_ebt` (`ExtruderMax`)
/// and no filament deletion.
fn deserialize_default(sel: &mut TriangleSelector, data: &AnnotationData, needs_reset: bool) {
    sel.deserialize(data, needs_reset, EnforcerBlockerType::EXTRUDER_MAX, EnforcerBlockerType::NONE, EnforcerBlockerType::NONE);
}

/// `static_cast<size_t>(EnforcerBlockerType::ExtruderMax) + 1`, the `StateCount` of
/// PaintReprojector (PaintReproject.cpp:486-487) and PaintRegionSource (848-849).
fn state_count() -> usize {
    EnforcerBlockerType::extruder_max().0 as usize + 1
}

/// `static_cast<size_t>(state)` of an `EnforcerBlockerType` (an int8_t): a negative state wraps
/// to a huge index, which every caller rejects against its table size.
#[inline]
fn state_index(state: EnforcerBlockerType) -> usize {
    state.0 as i64 as usize
}

/// TriangleMesh.hpp:373-386 `bounding_box(const indexed_triangle_set &its)` as
/// `TriangleMesh::bounding_box()` (TriangleMesh.cpp:492-499) returns it through the float stats:
/// (min, max), zero for an empty set.
pub fn mesh_bounding_box(its: &indexed_triangle_set) -> (Vec3d, Vec3d) {
    if its.vertices.is_empty() {
        return (Vec3d::zeros(), Vec3d::zeros());
    }
    let mut bmin = its.vertices[0];
    let mut bmax = its.vertices[0];
    for p in &its.vertices {
        // p.cwiseMin(bmin), p.cwiseMax(bmax)
        for k in 0..3 {
            bmin[k] = if bmin[k] < p[k] { bmin[k] } else { p[k] };
            bmax[k] = if p[k] < bmax[k] { bmax[k] } else { p[k] };
        }
    }
    (Vec3d::new(bmin.x as f64, bmin.y as f64, bmin.z as f64), Vec3d::new(bmax.x as f64, bmax.y as f64, bmax.z as f64))
}

/// `bounding_box().size().norm()` of a mesh
fn mesh_diagonal(its: &indexed_triangle_set) -> f64 {
    let (mn, mx) = mesh_bounding_box(its);
    norm_f64(&(mx - mn))
}

/// TriangleMesh.cpp:501-507 `transformed_bounding_box(const Transform3d &trafo)`: the box of the
/// transformed vertices (`BoundingBoxf3::merge`), its `size().norm()`.
fn transformed_diagonal(its: &indexed_triangle_set, trafo: &Transform3d) -> f64 {
    let mut defined = false;
    let (mut mn, mut mx) = (Vec3d::zeros(), Vec3d::zeros());
    for v in &its.vertices {
        let p = transform3d_mul_vec3d(trafo, [v.x as f64, v.y as f64, v.z as f64]);
        let p = Vec3d::new(p[0], p[1], p[2]);
        if defined {
            for k in 0..3 {
                mn[k] = if p[k] < mn[k] { p[k] } else { mn[k] };
                mx[k] = if mx[k] < p[k] { p[k] } else { mx[k] };
            }
        } else {
            mn = p;
            mx = p;
            defined = true;
        }
    }
    norm_f64(&(mx - mn))
}

// ---------------------------------------------------------------------------
// PaintReprojector (PaintReproject.cpp:141-649): samples a source annotation for the subdivision
// engine. It owns a TriangleSelector deserialized from the source annotation, the source face
// adjacency (for the coplanar neighborhood walk) and, in the geometric mode, an AABB tree over the
// source mesh (for nearest-face / nearest-point queries).
// ---------------------------------------------------------------------------

/// PaintReproject.cpp:488-490 — a state must cover at least this fraction of a target leaf's
/// overlap to be counted as present.
const MIXED_STATE_AREA_FRACTION: f64 = 1e-3;
/// PaintReproject.cpp:491-493
const PREFERRED_COVERAGE_EPSILON: f64 = 1e-4;
/// PaintReproject.cpp:494-496
const MIN_ROOT_OVERLAP_FRACTION: f64 = 1e-2;
/// PaintReproject.cpp:497-500
const MAX_SOURCE_ROOT_SET: usize = 4;
/// PaintReproject.cpp:501-504
const MAX_COPLANAR_SEARCH: i32 = 32;
/// PaintReproject.cpp:505-509 — 1/4^7
const BOUNDARY_LEAF_AREA_RATIO: f64 = 1.0 / 16384.0;

/// PaintReproject.cpp:510-513 `struct StateAreaInfo`
struct StateAreaInfo {
    area_by_state: Vec<f64>,
    total_area: f64,
}

/// PaintReproject.cpp:149-649 `class PaintReprojector`
pub struct PaintReprojector<'a> {
    m_source_mesh: &'a indexed_triangle_set,
    m_selector: TriangleSelector,
    m_source_root_count: i32,
    m_face_neighbors: Vec<Vec3i>,
    m_aabb: TreeF,
    m_has_aabb: bool,
    // Squared distance threshold beyond which a destination face is considered newly created by
    // the repair and skipped; < 0 disables the cull.
    m_max_sample_distance_sq: f64,
    m_fallback_count: usize,
    m_rebound_count: usize,
}

impl<'a> PaintReprojector<'a> {
    /// PaintReproject.cpp:152-171. max_sample_distance: when >= 0, a destination face whose
    /// centroid is farther than this from the source surface is left unpainted (-1.0 disables).
    pub fn new(source_mesh: &'a indexed_triangle_set, annotation: &FacetsAnnotation, build_aabb: bool, max_sample_distance: f64) -> Self {
        let mut m_selector = selector_of(source_mesh);
        deserialize_default(&mut m_selector, &annotation.data, true); // PaintReproject.cpp:165
        let mut out = PaintReprojector {
            m_source_mesh: source_mesh,
            m_selector,
            m_source_root_count: source_mesh.indices.len() as i32,
            m_face_neighbors: its_face_neighbors(source_mesh),
            m_aabb: TreeF::new(),
            m_has_aabb: false,
            m_max_sample_distance_sq: if max_sample_distance >= 0.0 { max_sample_distance * max_sample_distance } else { -1.0 },
            m_fallback_count: 0,
            m_rebound_count: 0,
        };
        if build_aabb && !source_mesh.indices.is_empty() {
            // PaintReproject.cpp:166-170
            out.m_aabb = build_aabb_tree_over_indexed_triangle_set_f(&source_mesh.vertices, &source_mesh.indices, 0.0);
            out.m_has_aabb = true;
        }
        out
    }

    /// PaintReproject.cpp:173-176
    pub fn source_node_count(&self) -> usize {
        self.m_selector.get_triangles().len()
    }

    /// PaintReproject.cpp:178-181
    pub fn fallback_count(&self) -> usize {
        self.m_fallback_count
    }

    /// PaintReproject.cpp:183-186
    pub fn rebound_count(&self) -> usize {
        self.m_rebound_count
    }

    /// PaintReproject.cpp:188-278: the source roots whose in-plane overlap with the target
    /// fragment is significant, by decreasing overlap; the first entry is the dominant source.
    pub fn resolve_source_roots(&mut self, preferred_root: i32, target_triangle_in_source: &[Vec3f; 3]) -> Vec<i32> {
        let mut resolved: Vec<i32> = Vec::new();
        let dropped_axis = Self::projection_axis(target_triangle_in_source);
        let target_area = triangle_overlap_area(target_triangle_in_source, target_triangle_in_source, dropped_axis);
        if target_area <= f64::EPSILON {
            resolved.push(preferred_root);
            return resolved;
        }

        let preferred_overlap = self.root_overlap_area(preferred_root, target_triangle_in_source, dropped_axis);
        // Fast path: provenance already covers essentially the whole fragment.
        if preferred_overlap >= target_area * (1.0 - PREFERRED_COVERAGE_EPSILON) {
            resolved.push(preferred_root);
            return resolved;
        }

        // Gather candidate source roots from the coplanar neighborhood of the provenance face only.
        let mut candidates: Vec<(i32, f64)> = Vec::new();
        let mut frontier: Vec<i32> = Vec::new();
        let mut visited: Vec<i32> = Vec::new();
        frontier.push(preferred_root);
        while !frontier.is_empty() && (visited.len() as i32) < MAX_COPLANAR_SEARCH {
            let root = frontier.pop().unwrap();
            if root < 0 || root >= self.m_source_root_count {
                continue;
            }
            if visited.contains(&root) {
                continue;
            }
            visited.push(root);

            let overlap = self.root_overlap_area(root, target_triangle_in_source, dropped_axis);
            if overlap > target_area * MIN_ROOT_OVERLAP_FRACTION {
                candidates.push((root, overlap));
            }

            // Only expand through coplanar edge neighbors so the walk stays on the flat face.
            if (root as usize) < self.m_face_neighbors.len() {
                let neighbors = self.m_face_neighbors[root as usize];
                for e in 0..3usize {
                    let neighbor = neighbors[e];
                    if neighbor >= 0 && neighbor != preferred_root && self.normals_parallel(preferred_root, neighbor) {
                        frontier.push(neighbor);
                    }
                }
            }
        }
        // std::sort(candidates, a.second > b.second) — libc++'s unstable sort
        crate::clipper::libcxx_sort(&mut candidates, |a, b| a.1 > b.1);

        // No coplanar source overlaps the fragment: it is a genuinely new surface.
        if candidates.is_empty() {
            resolved.push(preferred_root);
            return resolved;
        }

        let mut covered = 0.0f64;
        for candidate in &candidates {
            resolved.push(candidate.0);
            covered += candidate.1;
            if resolved.len() >= MAX_SOURCE_ROOT_SET || covered >= target_area * (1.0 - PREFERRED_COVERAGE_EPSILON) {
                break;
            }
        }

        if resolved[0] != preferred_root {
            self.m_rebound_count += 1;
        }

        resolved
    }

    /// PaintReproject.cpp:280-388: coplanar-overlap sampling (cutting).
    pub fn measure_overlap(&mut self, source_roots: &[i32], target_triangle_in_source: &[Vec3f; 3], target_triangle_in_destination: &[Vec3f; 3]) -> FacetSubdivisionMeasurement {
        let surface_area = triangle_area_f32(target_triangle_in_destination);
        if surface_area <= f64::EPSILON {
            return FacetSubdivisionMeasurement::default();
        }

        let triangles_len = self.m_selector.get_triangles().len() as i32;
        let root_valid = |sel: &TriangleSelector, root: i32| -> bool { root >= 0 && root < triangles_len && sel.get_triangles()[root as usize].valid() };

        // All roots in the set are coplanar, so any valid one defines the shared projection plane.
        let mut primary_root = -1;
        for &root in source_roots {
            if root_valid(&self.m_selector, root) {
                primary_root = root;
                break;
            }
        }
        if primary_root < 0 {
            self.m_fallback_count += 1;
            return FacetSubdivisionMeasurement { dominant_state: EnforcerBlockerType::NONE, surface_area, error_area: 0.0 };
        }

        let dropped_axis = self.projection_axis_of_root(primary_root);
        let projected_area = triangle_overlap_area(target_triangle_in_source, target_triangle_in_source, dropped_axis);
        let area_epsilon = std_max_f64(1e-12, projected_area * 1e-10);
        // Fixed boundary leaf-size floor derived only from the source root.
        let primary_root_triangle = self.triangle_vertices(primary_root);
        let primary_root_area = triangle_overlap_area(&primary_root_triangle, &primary_root_triangle, dropped_axis);
        let boundary_leaf_floor = primary_root_area * BOUNDARY_LEAF_AREA_RATIO;
        // Accumulate paint area across the whole source-root set.
        let mut area_info = StateAreaInfo { area_by_state: vec![0.0; state_count()], total_area: 0.0 };
        for &root in source_roots {
            if root_valid(&self.m_selector, root) {
                self.collect_overlap_areas(root, target_triangle_in_source, dropped_axis, area_epsilon, &mut area_info);
            }
        }

        if area_info.total_area <= area_epsilon {
            // The target face does not measurably overlap any source leaf: point-sample the
            // source faces at the fragment centroid.
            self.m_fallback_count += 1;
            let centroid = centroid_f32(&target_triangle_in_source[0], &target_triangle_in_source[1], &target_triangle_in_source[2]);
            for &root in source_roots {
                if !root_valid(&self.m_selector, root) {
                    continue;
                }
                let containing_leaf = self.m_selector.select_unsplit_triangle(&centroid, root);
                if containing_leaf >= 0 {
                    let leaf = &self.m_selector.get_triangles()[containing_leaf as usize];
                    if leaf.valid() && !leaf.is_split() {
                        return FacetSubdivisionMeasurement { dominant_state: leaf.get_state(), surface_area, error_area: 0.0 };
                    }
                }
            }
            return FacetSubdivisionMeasurement { dominant_state: EnforcerBlockerType::NONE, surface_area, error_area: 0.0 };
        }

        let mut dominant_state = 0usize;
        let mut present_states = 0usize;
        for state in 0..area_info.area_by_state.len() {
            if area_info.area_by_state[state] > area_info.area_by_state[dominant_state] {
                dominant_state = state;
            }
            if area_info.area_by_state[state] > area_info.total_area * MIXED_STATE_AREA_FRACTION {
                present_states += 1;
            }
        }

        // Refine a boundary-crossing leaf until it drops below the fixed root-relative floor.
        let below_floor = primary_root_area > 0.0 && projected_area <= boundary_leaf_floor;

        let crosses_boundary = present_states >= 2;
        let dominant_fraction = area_info.area_by_state[dominant_state] / area_info.total_area;
        let error_area = if below_floor {
            0.0
        } else if crosses_boundary {
            surface_area
        } else {
            surface_area * std_max_f64(0.0, 1.0 - dominant_fraction)
        };
        FacetSubdivisionMeasurement { dominant_state: EnforcerBlockerType(dominant_state as i8), surface_area, error_area }
    }

    /// PaintReproject.cpp:390-483: 3D nearest-point sampling (repair/boolean).
    pub fn measure_point_sample(&mut self, target_triangle_in_source: &[Vec3f; 3], target_triangle_in_destination: &[Vec3f; 3]) -> FacetSubdivisionMeasurement {
        let surface_area = triangle_area_f32(target_triangle_in_destination);
        if surface_area <= f64::EPSILON {
            return FacetSubdivisionMeasurement::default();
        }
        if !self.m_has_aabb {
            self.m_fallback_count += 1;
            return FacetSubdivisionMeasurement { dominant_state: EnforcerBlockerType::NONE, surface_area, error_area: 0.0 };
        }

        let centroid = centroid_f32(&target_triangle_in_source[0], &target_triangle_in_source[1], &target_triangle_in_source[2]);
        let mut centroid_hit_face: usize = usize::MAX;
        let mut centroid_closest_point = Vec3f::zeros();
        let centroid_distance_sq = squared_distance_to_indexed_triangle_set_f(&self.m_source_mesh.vertices, &self.m_source_mesh.indices, &self.m_aabb, &centroid, &mut centroid_hit_face, &mut centroid_closest_point) as f64;
        if centroid_distance_sq < 0.0 || centroid_hit_face == usize::MAX || (centroid_hit_face as i32) >= self.m_source_root_count {
            self.m_fallback_count += 1;
            return FacetSubdivisionMeasurement { dominant_state: EnforcerBlockerType::NONE, surface_area, error_area: 0.0 };
        }
        // New-geometry cull.
        if self.m_max_sample_distance_sq >= 0.0 && centroid_distance_sq > self.m_max_sample_distance_sq {
            return FacetSubdivisionMeasurement { dominant_state: EnforcerBlockerType::NONE, surface_area, error_area: 0.0 };
        }

        let sample_points = [target_triangle_in_source[0], target_triangle_in_source[1], target_triangle_in_source[2], centroid];
        let mut sample_counts: Vec<i32> = vec![0; state_count()];
        let mut sampled = 0i32;
        for point in &sample_points {
            let state = state_index(self.sample_state(point));
            if state < sample_counts.len() {
                sample_counts[state] += 1;
                sampled += 1;
            }
        }
        if sampled == 0 {
            return FacetSubdivisionMeasurement { dominant_state: EnforcerBlockerType::NONE, surface_area, error_area: 0.0 };
        }

        let mut dominant_state = 0usize;
        for state in 1..sample_counts.len() {
            if sample_counts[state] > sample_counts[dominant_state] {
                dominant_state = state;
            }
        }

        // Area-proportional error: the share of samples that disagree with the dominant state.
        let minority_samples = sampled - sample_counts[dominant_state];
        let mut error_area = surface_area * (minority_samples as f64 / sampled as f64);

        // Safety floor (not a forced target).
        if error_area > 0.0 {
            let nearest_root = centroid_hit_face as i32;
            let dropped_axis = Self::projection_axis(target_triangle_in_source);
            let leaf_area = triangle_overlap_area(target_triangle_in_source, target_triangle_in_source, dropped_axis);
            let root_triangle = self.triangle_vertices(nearest_root);
            let root_area = triangle_overlap_area(&root_triangle, &root_triangle, dropped_axis);
            if root_area > 0.0 && leaf_area <= root_area * BOUNDARY_LEAF_AREA_RATIO {
                error_area = 0.0;
            }
        }
        FacetSubdivisionMeasurement { dominant_state: EnforcerBlockerType(dominant_state as i8), surface_area, error_area }
    }

    /// PaintReproject.cpp:515-524
    fn triangle_vertices(&self, triangle_idx: i32) -> [Vec3f; 3] {
        let triangle = &self.m_selector.get_triangles()[triangle_idx as usize];
        let vertices = self.m_selector.get_vertices();
        [vertices[triangle.verts_idxs[0] as usize].v, vertices[triangle.verts_idxs[1] as usize].v, vertices[triangle.verts_idxs[2] as usize].v]
    }

    /// PaintReproject.cpp:526-545: snap a source-frame point to the nearest point on the source
    /// mesh and read the (sub-triangle) leaf state there.
    fn sample_state(&self, point_in_source: &Vec3f) -> EnforcerBlockerType {
        if !self.m_has_aabb {
            return EnforcerBlockerType::NONE;
        }
        let mut hit_face: usize = usize::MAX;
        let mut closest_point = Vec3f::zeros();
        let distance_squared = squared_distance_to_indexed_triangle_set_f(&self.m_source_mesh.vertices, &self.m_source_mesh.indices, &self.m_aabb, point_in_source, &mut hit_face, &mut closest_point) as f64;
        if distance_squared < 0.0 || hit_face == usize::MAX || (hit_face as i32) >= self.m_source_root_count {
            return EnforcerBlockerType::NONE;
        }
        let leaf = self.m_selector.select_unsplit_triangle(&closest_point, hit_face as i32);
        if leaf < 0 {
            return EnforcerBlockerType::NONE;
        }
        let triangles = self.m_selector.get_triangles();
        if leaf >= triangles.len() as i32 || !triangles[leaf as usize].valid() || triangles[leaf as usize].is_split() {
            return EnforcerBlockerType::NONE;
        }
        triangles[leaf as usize].get_state()
    }

    /// PaintReproject.cpp:547-576
    fn root_overlap_area(&self, source_root: i32, target_triangle: &[Vec3f; 3], dropped_axis: i32) -> f64 {
        let triangles = self.m_selector.get_triangles();
        if source_root < 0 || source_root >= self.m_source_root_count || source_root >= triangles.len() as i32 || !triangles[source_root as usize].valid() {
            return 0.0;
        }

        let source_triangle = self.triangle_vertices(source_root);
        let e1 = source_triangle[1] - source_triangle[0];
        let e2 = source_triangle[2] - source_triangle[0];
        let source_edge_1 = Vec3d::new(e1.x as f64, e1.y as f64, e1.z as f64);
        let source_edge_2 = Vec3d::new(e2.x as f64, e2.y as f64, e2.z as f64);
        let source_normal = cross_f64(&source_edge_1, &source_edge_2);
        let normal_length = norm_f64(&source_normal);
        if normal_length <= f64::EPSILON {
            return 0.0;
        }

        let edge_scale = std_max_f64(norm_f64(&source_edge_1), norm_f64(&source_edge_2));
        let plane_tolerance = std_max_f64(1e-5, edge_scale * 1e-5);
        let unit_normal = Vec3d::new(source_normal.x / normal_length, source_normal.y / normal_length, source_normal.z / normal_length);
        let source_origin = Vec3d::new(source_triangle[0].x as f64, source_triangle[0].y as f64, source_triangle[0].z as f64);
        for point in target_triangle {
            let d = Vec3d::new(point.x as f64, point.y as f64, point.z as f64) - source_origin;
            if dot_f64(&d, &unit_normal).abs() > plane_tolerance {
                return 0.0;
            }
        }

        triangle_overlap_area(target_triangle, &source_triangle, dropped_axis)
    }

    /// PaintReproject.cpp:578-607
    fn collect_overlap_areas(&self, triangle_idx: i32, target_triangle: &[Vec3f; 3], dropped_axis: i32, area_epsilon: f64, info: &mut StateAreaInfo) {
        let triangle = &self.m_selector.get_triangles()[triangle_idx as usize];
        if !triangle.valid() {
            return;
        }

        let source_triangle = self.triangle_vertices(triangle_idx);
        let overlap_area = triangle_overlap_area(target_triangle, &source_triangle, dropped_axis);
        if overlap_area <= area_epsilon {
            return;
        }

        if !triangle.is_split() {
            let state = state_index(triangle.get_state());
            if state < info.area_by_state.len() {
                info.area_by_state[state] += overlap_area;
                info.total_area += overlap_area;
            }
            return;
        }

        let child_count = triangle.number_of_split_sides() + 1;
        let children = triangle.children;
        for child_idx in 0..child_count {
            self.collect_overlap_areas(children[child_idx as usize], target_triangle, dropped_axis, area_epsilon, info);
        }
    }

    /// PaintReproject.cpp:609-612 `static int projection_axis(const std::array<Vec3f, 3> &triangle)`
    fn projection_axis(triangle: &[Vec3f; 3]) -> i32 {
        dominant_projection_axis(triangle)
    }

    /// PaintReproject.cpp:614-631: whether two source roots have (anti)parallel normals.
    fn normals_parallel(&self, root_a: i32, root_b: i32) -> bool {
        if root_a < 0 || root_a >= self.m_source_root_count || root_b < 0 || root_b >= self.m_source_root_count {
            return false;
        }
        let ta = self.triangle_vertices(root_a);
        let tb = self.triangle_vertices(root_b);
        let to_d = |v: Vec3f| Vec3d::new(v.x as f64, v.y as f64, v.z as f64);
        let na = cross_f64(&to_d(ta[1] - ta[0]), &to_d(ta[2] - ta[0]));
        let nb = cross_f64(&to_d(tb[1] - tb[0]), &to_d(tb[2] - tb[0]));
        let la = norm_f64(&na);
        let lb = norm_f64(&nb);
        if la <= f64::EPSILON || lb <= f64::EPSILON {
            return false;
        }
        (dot_f64(&na, &nb) / (la * lb)).abs() >= 1.0 - 1e-4
    }

    /// PaintReproject.cpp:633-636 `int projection_axis(int source_root)` (the second overload)
    fn projection_axis_of_root(&self, source_root: i32) -> i32 {
        Self::projection_axis(&self.triangle_vertices(source_root))
    }
}

// ---------------------------------------------------------------------------
// PaintRegionSource (PaintReproject.cpp:651-921): the source annotation flattened into its leaf
// triangles, indexed for area queries, and grouped by paint state.
// ---------------------------------------------------------------------------

/// `PaintRegionSource::Box3f` (PaintReproject.cpp:669-670)
pub type Box3f = BoundingBox3f;

/// PaintReproject.cpp:672-676 `struct PaintRegionSource::StateInfo`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StateInfo {
    pub state: EnforcerBlockerType,
    pub area: f64,
    pub bbox: Box3f,
}

impl Default for StateInfo {
    fn default() -> Self {
        StateInfo { state: EnforcerBlockerType::NONE, area: 0.0, bbox: Box3f::default() }
    }
}

/// PaintReproject.cpp:850-852 — cos(60 deg)
const NORMAL_AGREEMENT_LIMIT: f32 = 0.5;
/// PaintReproject.cpp:853-855
const MIN_OVERLAP_FRACTION: f64 = 1e-3;
/// PaintReproject.cpp:856-857
const COVERAGE_EPSILON: f64 = 1e-3;
/// PaintReproject.cpp:858-860
const MIN_MATCHED_FRACTION: f64 = 0.75;
/// PaintReproject.cpp:861
const SQRT3: f32 = 1.7320509f32;

/// PaintReproject.cpp:666-921 `class PaintRegionSource`
pub struct PaintRegionSource {
    // Released once the leaves have been flattened out of it (`m_selector.reset()`): the
    // selector lives only in the constructor here.
    m_band: f64,
    m_leaf_vertices: Vec<Vec3f>,
    m_leaf_indices: Vec<Vec3i>,
    m_leaf_states: Vec<EnforcerBlockerType>,
    m_leaf_normals: Vec<Vec3f>,
    m_states: Vec<StateInfo>,
    m_tree: TreeF,
    m_finest_leaf_edge: f32,
}

impl PaintRegionSource {
    /// PaintReproject.cpp:678-747. band: half-width of the slab around a destination triangle's
    /// plane within which a source leaf still lies on the same surface (negative disables);
    /// root_filter, when given, restricts the flattening to those source faces.
    pub fn new(source_mesh: &indexed_triangle_set, annotation: &AnnotationData, band: f64, root_filter: Option<&[i8]>) -> Self {
        let mut m_selector = selector_of(source_mesh); // PaintReproject.cpp:695
        let mut out = PaintRegionSource {
            m_band: band,
            m_leaf_vertices: Vec::new(),
            m_leaf_indices: Vec::new(),
            m_leaf_states: Vec::new(),
            m_leaf_normals: Vec::new(),
            m_states: Vec::new(),
            m_tree: TreeF::new(),
            m_finest_leaf_edge: f32::MAX,
        };
        deserialize_default(&mut m_selector, annotation, true); // PaintReproject.cpp:698

        let root_count = source_mesh.indices.len() as i32;
        let mut vertex_map: Vec<i32> = vec![-1; m_selector.get_vertices().len()];
        if root_filter.is_none() {
            out.m_leaf_indices.reserve(m_selector.get_triangles().len());
        }
        for root in 0..root_count {
            if root_filter.is_none() || root_filter.unwrap()[root as usize] != 0 {
                out.collect_leaves(&m_selector, root, &mut vertex_map);
            }
        }
        // The flattened leaves carry everything the coverage test needs, so drop the
        // subdivision tree rather than keeping a second copy of it alive.
        drop(m_selector); // PaintReproject.cpp:709
        if out.m_leaf_indices.is_empty() {
            return out;
        }

        // Per-leaf normal and area, plus the per-state totals and bounding boxes.
        out.m_leaf_normals.reserve(out.m_leaf_indices.len());
        let mut by_state: Vec<StateInfo> = vec![StateInfo::default(); state_count()];
        for state in 0..by_state.len() {
            by_state[state].state = EnforcerBlockerType(state as i8);
        }
        for leaf in 0..out.m_leaf_indices.len() {
            let triangle = out.leaf_triangle(leaf);
            let normal = eigen_cross3f(&(triangle[1] - triangle[0]), &(triangle[2] - triangle[0]));
            let normal_length = norm_f32(&normal);
            out.m_leaf_normals.push(if normal_length > 0.0 { Vec3f::new(normal.x / normal_length, normal.y / normal_length, normal.z / normal_length) } else { Vec3f::new(0.0, 0.0, 0.0) });
            let state = state_index(out.m_leaf_states[leaf]);
            if state >= by_state.len() {
                continue;
            }
            by_state[state].area += 0.5 * normal_length as f64;
            for vertex in &triangle {
                by_state[state].bbox.extend(vertex);
            }
        }

        // Paint order: descending painted area. NONE is never painted.
        for state in 1..by_state.len() {
            if by_state[state].area > 0.0 {
                out.m_states.push(by_state[state]);
            }
        }
        // std::sort(m_states, a.area > b.area) — libc++'s unstable sort
        crate::clipper::libcxx_sort(&mut out.m_states, |a, b| a.area > b.area);

        out.m_tree = build_aabb_tree_over_indexed_triangle_set_f(&out.m_leaf_vertices, &out.m_leaf_indices, 0.0); // PaintReproject.cpp:745-746
        out
    }

    /// PaintReproject.cpp:749
    pub fn empty(&self) -> bool {
        self.m_leaf_indices.is_empty() || self.m_states.is_empty()
    }
    /// PaintReproject.cpp:750
    pub fn leaf_count(&self) -> usize {
        self.m_leaf_indices.len()
    }
    /// PaintReproject.cpp:751
    pub fn states(&self) -> &[StateInfo] {
        &self.m_states
    }

    /// PaintReproject.cpp:753-759: shortest edge among the flattened leaves (diagnostic only).
    pub fn finest_leaf_edge(&self) -> f32 {
        if self.m_finest_leaf_edge < f32::MAX {
            self.m_finest_leaf_edge
        } else {
            0.0
        }
    }

    /// PaintReproject.cpp:761-845: how much of the source area that maps onto this destination
    /// triangle (given in the source frame) carries the requested state.
    pub fn coverage(&self, state: EnforcerBlockerType, triangle_in_source: &[Vec3f; 3]) -> RegionCoverage {
        let normal = eigen_cross3f(&(triangle_in_source[1] - triangle_in_source[0]), &(triangle_in_source[2] - triangle_in_source[0]));
        let normal_length = norm_f32(&normal);
        if normal_length <= f32::MIN_POSITIVE {
            return RegionCoverage::None;
        }
        let unit_normal = Vec3f::new(normal.x / normal_length, normal.y / normal_length, normal.z / normal_length);

        let dropped_axis = dominant_projection_axis(triangle_in_source);
        let target_area = triangle_overlap_area(triangle_in_source, triangle_in_source, dropped_axis);
        if target_area <= f64::EPSILON {
            return RegionCoverage::None;
        }
        let area_epsilon = std_max_f64(1e-12, target_area * 1e-10);

        let mut query_box = Box3f::default();
        for vertex in triangle_in_source {
            query_box.extend(vertex);
        }
        // Only the projection axis is widened by the slab (by sqrt(3) * band).
        if self.m_band > 0.0 {
            let widening = (self.m_band as f32) * SQRT3;
            query_box.min[dropped_axis as usize] -= widening;
            query_box.max[dropped_axis as usize] += widening;
        }

        let mut area_in_state = 0.0f64;
        let mut area_matched = 0.0f64;
        traverse_f(&self.m_tree, intersecting_f(query_box), |node: &NodeF| {
            let leaf = node.idx;
            // Reject source surface that faces elsewhere.
            if eigen_dot3f(&self.m_leaf_normals[leaf], &unit_normal) < NORMAL_AGREEMENT_LIMIT {
                return true;
            }
            let leaf_triangle = self.leaf_triangle(leaf);
            // Reject source surface lying off this triangle's plane.
            if self.m_band > 0.0 {
                let centroid = centroid_f32(&leaf_triangle[0], &leaf_triangle[1], &leaf_triangle[2]);
                if (eigen_dot3f(&(centroid - triangle_in_source[0]), &unit_normal) as f64).abs() > self.m_band {
                    return true;
                }
            }
            let overlap = triangle_overlap_area(&leaf_triangle, triangle_in_source, dropped_axis);
            if overlap <= area_epsilon {
                return true;
            }
            area_matched += overlap;
            if self.m_leaf_states[leaf] == state {
                area_in_state += overlap;
            }
            true
        });

        if area_matched <= area_epsilon || area_in_state <= area_matched * MIN_OVERLAP_FRACTION {
            return RegionCoverage::None;
        }
        // Report Full only when the state owns essentially all of the matched source area AND
        // enough of the triangle was matched at all.
        if area_in_state >= area_matched * (1.0 - COVERAGE_EPSILON) && area_matched >= target_area * MIN_MATCHED_FRACTION {
            return RegionCoverage::Full;
        }
        RegionCoverage::Partial
    }

    /// PaintReproject.cpp:863-898
    fn collect_leaves(&mut self, selector: &TriangleSelector, triangle_idx: i32, vertex_map: &mut Vec<i32>) {
        let triangles = selector.get_triangles();
        if triangle_idx < 0 || triangle_idx >= triangles.len() as i32 {
            return;
        }
        let triangle = &triangles[triangle_idx as usize];
        if !triangle.valid() {
            return;
        }
        if triangle.is_split() {
            let child_count = triangle.number_of_split_sides() + 1;
            let children = triangle.children;
            for child in 0..child_count {
                self.collect_leaves(selector, children[child as usize], vertex_map);
            }
            return;
        }

        let vertices = selector.get_vertices();
        let mut indices = Vec3i::new(0, 0, 0);
        for i in 0..3usize {
            let vertex = triangle.verts_idxs[i];
            if vertex_map[vertex as usize] < 0 {
                vertex_map[vertex as usize] = self.m_leaf_vertices.len() as i32;
                self.m_leaf_vertices.push(vertices[vertex as usize].v);
            }
            indices[i] = vertex_map[vertex as usize];
        }
        self.m_leaf_indices.push(indices);
        self.m_leaf_states.push(triangle.get_state());

        let a = self.m_leaf_vertices[indices[0] as usize];
        let b = self.m_leaf_vertices[indices[1] as usize];
        let c = self.m_leaf_vertices[indices[2] as usize];
        // std::sqrt(std::min((b - a).squaredNorm(), std::min((c - b).squaredNorm(), (a - c).squaredNorm())))
        let (ab, bc, ca) = (eigen_squared_norm3f(&(b - a)), eigen_squared_norm3f(&(c - b)), eigen_squared_norm3f(&(a - c)));
        let inner = if ca < bc { ca } else { bc };
        let shortest = (if inner < ab { inner } else { ab }).sqrt();
        if shortest > 0.0 {
            self.m_finest_leaf_edge = if shortest < self.m_finest_leaf_edge { shortest } else { self.m_finest_leaf_edge };
        }
    }

    /// PaintReproject.cpp:900-908
    fn leaf_triangle(&self, leaf: usize) -> [Vec3f; 3] {
        let indices = &self.m_leaf_indices[leaf];
        [self.m_leaf_vertices[indices[0] as usize], self.m_leaf_vertices[indices[1] as usize], self.m_leaf_vertices[indices[2] as usize]]
    }
}

// ---------------------------------------------------------------------------
// Subdivision driver (PaintReproject.cpp:923-1095).
// ---------------------------------------------------------------------------

/// PaintReproject.cpp:927 `enum class ReprojectMode`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReprojectMode {
    OverlapCoplanar,
    PointNearest,
}

/// PaintReproject.cpp:929
pub const REPROJECT_RELATIVE_ERROR_LIMIT: f64 = 1e-3;
/// PaintReproject.cpp:930
pub const REPROJECT_ABSOLUTE_ERROR_EPSILON: f64 = 1e-8;
/// PaintReproject.cpp:931-936
pub const REPROJECT_ABSOLUTE_TARGET_AREA_FRACTION: f64 = 1e-6;
/// PaintReproject.cpp:937
pub const REPROJECT_MAX_SUBDIVISION_DEPTH: i32 = 20;
/// PaintReproject.cpp:938
pub const REPROJECT_MIN_NODE_BUDGET: usize = 65536;
/// PaintReproject.cpp:939
pub const REPROJECT_MAX_NODE_BUDGET: usize = 4000000;

/// The node budget of PaintReproject.cpp:972-975 (and 1333-1337, 2170-2173): 16 nodes per source
/// node, floored at ReprojectMinNodeBudget and capped at ReprojectMaxNodeBudget.
fn node_budget_for(source_nodes: usize) -> usize {
    let scaled = if source_nodes > REPROJECT_MAX_NODE_BUDGET / 16 { REPROJECT_MAX_NODE_BUDGET } else { source_nodes * 16 };
    let capped = if scaled < REPROJECT_MAX_NODE_BUDGET { scaled } else { REPROJECT_MAX_NODE_BUDGET }; // std::min
    if REPROJECT_MIN_NODE_BUDGET < capped {
        capped
    } else {
        REPROJECT_MIN_NODE_BUDGET
    } // std::max
}

/// PaintReproject.cpp:941-1095 `reproject_one_annotation`: returns false if the operation was
/// canceled via the cancel callback, true otherwise (including when there was nothing to do).
#[allow(clippy::too_many_arguments)]
pub fn reproject_one_annotation(
    src: &FacetsAnnotation,
    dst: &mut FacetsAnnotation,
    src_mesh: &indexed_triangle_set,
    dst_mesh: &indexed_triangle_set,
    dst_to_src: &Transform3d,
    dst_to_src_face: Option<&[i32]>,
    mode: ReprojectMode,
    annotation_name: &str,
    mut progress: Option<&mut PaintReprojectProgressCallback<'_>>,
    mut cancel: Option<&mut PaintReprojectCancelCallback<'_>>,
    max_sample_distance: f64,
    progress_message: Option<&str>,
    absolute_target_error: f64,
) -> bool {
    if src.is_empty() {
        return true;
    }
    if mode == ReprojectMode::OverlapCoplanar && (dst_to_src_face.is_none() || dst_to_src_face.unwrap().is_empty()) {
        return true;
    }

    // User-facing progress label (English); falls back to the internal log tag.
    let progress_label: &str = progress_message.unwrap_or(annotation_name);

    let mut projector = PaintReprojector::new(src_mesh, src, mode == ReprojectMode::PointNearest, max_sample_distance);
    let mut dst_sel = selector_of(dst_mesh);
    let source_nodes = projector.source_node_count();
    let node_budget = node_budget_for(source_nodes);

    let mut n_dst = dst_mesh.indices.len() as i32;
    if mode == ReprojectMode::OverlapCoplanar {
        n_dst = n_dst.min(dst_to_src_face.unwrap().len() as i32);
    }

    let mut destination_roots: Vec<i32> = Vec::with_capacity(n_dst.max(0) as usize);
    let mut resolved_source_root_sets: Vec<Vec<i32>> = vec![Vec::new(); n_dst.max(0) as usize];
    let mut source_roots_resolved: Vec<i8> = vec![0; n_dst.max(0) as usize];
    if mode == ReprojectMode::OverlapCoplanar {
        let map = dst_to_src_face.unwrap();
        for new_idx in 0..n_dst {
            if map[new_idx as usize] >= 0 {
                destination_roots.push(new_idx);
            }
        }
    } else {
        for new_idx in 0..n_dst {
            destination_roots.push(new_idx);
        }
    }

    // PaintReproject.cpp:998-1016
    let mut evaluator = |destination_root: i32, triangle: &[Vec3f; 3]| -> FacetSubdivisionMeasurement {
        let triangle_in_source = transform_triangle(triangle, dst_to_src);
        if mode == ReprojectMode::OverlapCoplanar {
            let first_visit = source_roots_resolved[destination_root as usize] == 0;
            if first_visit {
                resolved_source_root_sets[destination_root as usize] = projector.resolve_source_roots(dst_to_src_face.unwrap()[destination_root as usize], &triangle_in_source);
                source_roots_resolved[destination_root as usize] = 1;
            }
            return projector.measure_overlap(&resolved_source_root_sets[destination_root as usize], &triangle_in_source, triangle);
        }
        projector.measure_point_sample(&triangle_in_source, triangle)
    };

    // Converge each destination face independently to the same relative accuracy.
    let mut result = FacetSubdivisionResult::default();
    let mut canceled = false;
    let total_faces = destination_roots.len() as i32;
    let mut processed_faces = 0i32;
    let mut last_reported_percent = -1i32;
    for &destination_root in &destination_roots {
        // Poll cancellation between faces.
        if let Some(c) = cancel.as_mut() {
            if c() {
                canceled = true;
                break;
            }
        }
        let cancel_cb: Option<&mut FacetSubdivisionCancelCallback> = match cancel.as_mut() {
            Some(c) => Some(&mut **c),
            None => None,
        };
        let face_result = dst_sel.set_facets_with_subdivision(&[destination_root], &mut evaluator, node_budget, REPROJECT_RELATIVE_ERROR_LIMIT, absolute_target_error, REPROJECT_MAX_SUBDIVISION_DEPTH, cancel_cb);
        result.surface_area += face_result.surface_area;
        result.error_area += face_result.error_area;
        result.nodes_created += face_result.nodes_created;
        result.node_budget_exhausted |= face_result.node_budget_exhausted;
        result.depth_limit_reached |= face_result.depth_limit_reached;
        if face_result.canceled {
            canceled = true;
            break;
        }

        processed_faces += 1;
        if let Some(p) = progress.as_mut() {
            if total_faces > 0 {
                let percent = (processed_faces as usize * 100 / total_faces as usize) as i32;
                if percent != last_reported_percent {
                    last_reported_percent = percent;
                    p(percent, Some(progress_label));
                }
            }
        }
    }
    if canceled {
        return false;
    }
    drop(evaluator);
    let relative_error = if result.surface_area > 0.0 { result.error_area / result.surface_area } else { 0.0 };
    let target_nodes = dst_sel.get_triangles().iter().filter(|t| t.valid()).count();
    crate::boost_log_trivial!(
        Info,
        "Paint reprojection [", annotation_name, "]: source nodes=", source_nodes, ", target nodes=", target_nodes,
        ", relative error=", relative_error, ", budget exhausted=", result.node_budget_exhausted,
        ", depth limit reached=", result.depth_limit_reached, ", rebound roots=", projector.rebound_count(),
        ", fallbacks=", projector.fallback_count()
    );
    if result.node_budget_exhausted || result.depth_limit_reached {
        crate::boost_log_trivial!(
            Warning,
            "Paint reprojection [", annotation_name, "] stopped with relative error ", relative_error,
            " (budget exhausted=", result.node_budget_exhausted, ", depth limit reached=", result.depth_limit_reached, ")"
        );
    }
    if projector.rebound_count() > 0 {
        crate::boost_log_trivial!(Warning, "Paint reprojection [", annotation_name, "] rebound ", projector.rebound_count(), " destination roots to geometrically matching source roots");
    }
    if projector.fallback_count() > 0 {
        crate::boost_log_trivial!(Warning, "Paint reprojection [", annotation_name, "] used point-sample fallback ", projector.fallback_count(), " times");
    }

    dst.set(&dst_sel);
    true
}

// ---------------------------------------------------------------------------
// Region re-painting driver (repair path), PaintReproject.cpp:1097-1383.
// ---------------------------------------------------------------------------

/// PaintReproject.cpp:1101-1119 — target edge length for destination leaves along a paint
/// boundary, in mm.
pub const REGION_PAINT_TARGET_EDGE: f32 = 0.2;
/// PaintReproject.cpp:1120-1122 — model-relative floor (diagonal / 4096).
pub const REGION_PAINT_DIAGONAL_EDGE_DIVISOR: i32 = 4096;
/// PaintReproject.cpp:1123-1125
pub const REGION_PAINT_MAX_SOURCE_LEAVES: usize = 4000000;

/// PaintReproject.cpp:1127-1168 `class RegionEdgeLimit`: the edge-length floor in the frame
/// split_triangle() will measure in.
pub struct RegionEdgeLimit {
    /// `m_storage`, set (and `m_trafo` pointing at it) for a non-uniformly scaled destination
    m_trafo: Option<Transform3f>,
    m_value: f32,
}

impl RegionEdgeLimit {
    /// PaintReproject.cpp:1136-1155. dst_world_matrix maps the destination mesh into world space;
    /// None means the mesh is already there.
    ///
    /// The uniform-scaling test reads `Geometry::Transformation::get_scaling_factor()`, which
    /// the crate computes with nalgebra's SVD where the C++ uses Eigen's JacobiSVD: for a rotated
    /// uniform scale the two can differ in the last bits of the factor.
    pub fn new(dst_mesh: &indexed_triangle_set, dst_world_matrix: Option<&Transform3d>) -> Self {
        let mut diagonal = mesh_diagonal(dst_mesh);
        let mut target = REGION_PAINT_TARGET_EDGE as f64;
        let mut m_trafo = None;
        if let Some(m) = dst_world_matrix {
            let sf = Transformation::from_transform(*m).get_scaling_factor();
            if is_approx(sf.x, sf.y) && is_approx(sf.y, sf.z) {
                if sf.x > EPSILON {
                    target /= sf.x;
                }
            } else {
                m_trafo = Some(m.cast::<f32>());
                diagonal = transformed_diagonal(dst_mesh, m);
            }
        }
        let floor = if diagonal > 0.0 { (diagonal / REGION_PAINT_DIAGONAL_EDGE_DIVISOR as f64) as f32 } else { 0.0f32 };
        let t = target as f32;
        // std::max(float(target), floor)
        let m_value = if t < floor { floor } else { t };
        RegionEdgeLimit { m_trafo, m_value }
    }

    /// PaintReproject.cpp:1161
    pub fn value(&self) -> f32 {
        self.m_value
    }

    /// PaintReproject.cpp:1162
    pub fn trafo(&self) -> Option<&Transform3f> {
        self.m_trafo.as_ref()
    }
}


// ---------------------------------------------------------------------------
// Destination geometry in the source frame (PaintReproject.cpp:1170-1210).
// ---------------------------------------------------------------------------

/// Eigen `t.isApprox(Transform3d::Identity())`: `isApprox` of the 4x4 matrices with the default
/// precision `1e-12`, `|a - b|^2 <= prec^2 * min(|a|^2, |b|^2)` over all coefficients, summed
/// column by column.
fn is_approx_identity(t: &Transform3d) -> bool {
    let (mut diff, mut na, mut nb) = (0.0f64, 0.0f64, 0.0f64);
    for c in 0..4 {
        for r in 0..4 {
            let a = t[(r, c)];
            let b = if r == c { 1.0 } else { 0.0 };
            diff += (a - b) * (a - b);
            na += a * a;
            nb += b * b;
        }
    }
    let prec = 1e-12f64;
    diff <= prec * prec * na.min(nb)
}

/// PaintReproject.cpp:1170-1210: the destination vertices expressed in the source frame, built
/// once per reprojection and shared by all four annotation layers
pub struct DestinationInSourceFrame<'a> {
    m_dst_to_src: Transform3d,
    m_identity: bool,
    /// the transformed vertices, or the destination's own when the transformation is the identity
    m_vertices: std::borrow::Cow<'a, [Vec3f]>,
}

impl<'a> DestinationInSourceFrame<'a> {
    /// PaintReproject.cpp:1176-1186
    pub fn new(dst_mesh: &'a indexed_triangle_set, dst_to_src: &Transform3d) -> Self {
        let m_identity = is_approx_identity(dst_to_src);
        let m_vertices = if m_identity {
            std::borrow::Cow::Borrowed(&dst_mesh.vertices[..])
        } else {
            let mut storage: Vec<Vec3f> = Vec::with_capacity(dst_mesh.vertices.len());
            for vertex in dst_mesh.vertices.iter() {
                storage.push(transform_point(vertex, dst_to_src));
            }
            std::borrow::Cow::Owned(storage)
        };
        DestinationInSourceFrame { m_dst_to_src: *dst_to_src, m_identity, m_vertices }
    }

    /// PaintReproject.cpp:1192
    pub fn vertices(&self) -> &[Vec3f] {
        &self.m_vertices
    }

    /// PaintReproject.cpp:1194-1197
    pub fn triangle(&self, face: &Vec3i) -> [Vec3f; 3] {
        [self.m_vertices[face[0] as usize], self.m_vertices[face[1] as usize], self.m_vertices[face[2] as usize]]
    }

    /// PaintReproject.cpp:1199-1203
    pub fn to_source(&self, triangle_in_destination: &[Vec3f; 3]) -> [Vec3f; 3] {
        if self.m_identity {
            *triangle_in_destination
        } else {
            transform_triangle(triangle_in_destination, &self.m_dst_to_src)
        }
    }
}

/// PaintReproject.cpp:1212-1214
pub const REGION_SEED_TREE_MIN_STATES: usize = 3;

/// PaintReproject.cpp:1216-1298 `class DestinationSeedIndex`: destination faces prepared for
/// seeding the region walks.
pub struct DestinationSeedIndex<'a> {
    m_vertices: &'a [Vec3f],
    m_all_indices: &'a [Vec3i],
    m_masked: bool,
    // Paintable original face indices and their boxes, kept parallel.
    m_faces: Vec<i32>,
    m_boxes: Vec<Box3f>,
    // Only populated when a mask is given; unmasked the tree runs over m_all_indices directly.
    m_masked_indices: Vec<Vec3i>,
    m_tree: TreeF,
    m_has_tree: bool,
}

impl<'a> DestinationSeedIndex<'a> {
    /// PaintReproject.cpp:1226-1250
    pub fn new(dst_mesh: &'a indexed_triangle_set, dst_in_source: &'a DestinationInSourceFrame<'a>, paintable_facets: Option<&[i8]>) -> Self {
        let mut out = DestinationSeedIndex {
            m_vertices: dst_in_source.vertices(),
            m_all_indices: &dst_mesh.indices,
            m_masked: paintable_facets.is_some(),
            m_faces: Vec::new(),
            m_boxes: Vec::new(),
            m_masked_indices: Vec::new(),
            m_tree: TreeF::new(),
            m_has_tree: false,
        };
        let face_count = out.m_all_indices.len();
        if !out.m_masked {
            out.m_faces.reserve(face_count);
            out.m_boxes.reserve(face_count);
        }
        for face_idx in 0..face_count {
            if out.m_masked && paintable_facets.unwrap()[face_idx] == 0 {
                continue;
            }
            let face = out.m_all_indices[face_idx];
            let mut bx = Box3f::default();
            for i in 0..3usize {
                bx.extend(&out.m_vertices[face[i] as usize]);
            }
            out.m_faces.push(face_idx as i32);
            out.m_boxes.push(bx);
            if out.m_masked {
                out.m_masked_indices.push(face);
            }
        }
        out
    }

    /// PaintReproject.cpp:1256-1263
    pub fn ensure_tree(&mut self) {
        if self.m_has_tree || self.m_faces.is_empty() {
            return;
        }
        self.m_tree = build_aabb_tree_over_indexed_triangle_set_f(self.m_vertices, if self.m_masked { &self.m_masked_indices } else { self.m_all_indices }, 0.0);
        self.m_has_tree = true;
    }

    /// PaintReproject.cpp:1265-1285: every paintable destination face whose box meets query, as
    /// original face indices.
    pub fn seeds(&self, query: &Box3f, out: &mut Vec<i32>) {
        out.clear();
        if self.m_has_tree {
            traverse_f(&self.m_tree, intersecting_f(*query), |node: &NodeF| {
                out.push(self.m_faces[node.idx]);
                true
            });
            // Tree order is arbitrary: sort so the result does not depend on whether the tree
            // was built. (Equal ints are indistinguishable, so any sort gives std::sort's result.)
            out.sort_unstable();
            return;
        }
        for i in 0..self.m_faces.len() {
            if query.intersects(&self.m_boxes[i]) {
                out.push(self.m_faces[i]);
            }
        }
    }
}

/// PaintReproject.cpp:1300-1304 `struct RegionReprojectStats`
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RegionReprojectStats {
    pub nodes_created: usize,
    pub budget_exhausted: bool,
    pub canceled: bool,
}

/// PaintReproject.cpp:1306-1382 `reproject_one_annotation_by_region`: paints every state of the
/// source onto dst_sel, one state per stroke.
#[allow(clippy::too_many_arguments)]
pub fn reproject_one_annotation_by_region(
    source: &PaintRegionSource,
    dst_sel: &mut TriangleSelector,
    seed_index: &mut DestinationSeedIndex,
    dst_in_source: &DestinationInSourceFrame,
    edge_limit: f32,
    seed_inflate: f32,
    mut progress: Option<&mut PaintReprojectProgressCallback<'_>>,
    mut cancel: Option<&mut PaintReprojectCancelCallback<'_>>,
    progress_message: Option<&str>,
    paintable_facets: Option<&[i8]>,
    edge_limit_trafo: Option<&Transform3f>,
) -> RegionReprojectStats {
    let mut stats = RegionReprojectStats::default();
    let states = source.states();
    if states.is_empty() {
        return stats;
    }

    if states.len() >= REGION_SEED_TREE_MIN_STATES {
        seed_index.ensure_tree();
    }

    // Boundary-driven refinement scales with paint boundary length.
    let node_budget = node_budget_for(source.leaf_count());

    let mut last_reported_percent = -1i32;
    let mut start_facets: Vec<i32> = Vec::new();
    for state_idx in 0..states.len() {
        let info = &states[state_idx];

        // Seed the walk with every destination face whose box meets this state's box, inflated.
        let mut state_box = info.bbox;
        if seed_inflate > 0.0 {
            state_box.min -= Vec3f::new(seed_inflate, seed_inflate, seed_inflate);
            state_box.max += Vec3f::new(seed_inflate, seed_inflate, seed_inflate);
        }
        seed_index.seeds(&state_box, &mut start_facets);
        if start_facets.is_empty() {
            continue;
        }

        let state = info.state;
        let mut coverage = |triangle: &[Vec3f; 3]| -> RegionCoverage { source.coverage(state, &dst_in_source.to_source(triangle)) };

        let cancel_cb: Option<&mut FacetSubdivisionCancelCallback> = match cancel.as_mut() {
            Some(c) => Some(&mut **c),
            None => None,
        };
        let result = dst_sel.paint_region(&start_facets, Some(&mut coverage), state, edge_limit, node_budget, REPROJECT_MAX_SUBDIVISION_DEPTH, cancel_cb, paintable_facets, edge_limit_trafo);
        if result.canceled {
            stats.canceled = true;
            return stats;
        }
        stats.nodes_created += result.nodes_created;
        stats.budget_exhausted |= result.node_budget_exhausted;

        if let Some(p) = progress.as_mut() {
            let percent = ((state_idx + 1) * 100 / states.len()) as i32;
            if percent != last_reported_percent {
                last_reported_percent = percent;
                p(percent, progress_message);
            }
        }
    }
    stats
}

/// PaintReproject.cpp:1385-1390: the valid nodes of the selector's trees
pub fn count_valid_nodes(selector: &TriangleSelector) -> usize {
    selector.get_triangles().iter().filter(|triangle| triangle.valid()).count()
}

// ---------------------------------------------------------------------------
// Cut re-painting driver (PaintReproject.cpp:1392-1576).
// ---------------------------------------------------------------------------

/// PaintReproject.cpp:1407-1413: relative tolerance for recognising a destination face as the
/// untouched source face
pub const CUT_CONGRUENCE_RELATIVE_TOLERANCE: f64 = 1e-5;

/// PaintReproject.cpp:1415-1423: how a destination face relates to the source face the cut
/// recorded for it
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CutFaceOrigin {
    /// Surface the cut invented (the cross-section cap): no source paint applies.
    Invented,
    /// Carried over untouched: the source subdivision tree can be copied verbatim.
    Congruent,
    /// Fragment of a source face the cut plane crossed: needs geometric coverage.
    Fragment,
}

/// PaintReproject.cpp:1425-1435
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct CutFaceClassification {
    pub origin: Vec<CutFaceOrigin>,
    /// Indexed by destination face, for paint_region()'s walk restriction.
    pub fragment_faces: Vec<i8>,
    /// Indexed by source face: the roots the fragments descend from.
    pub fragment_source_roots: Vec<i8>,
    pub congruent_count: usize,
    pub fragment_count: usize,
    pub invented_count: usize,
}

/// Eigen `squaredNorm()` of a `Vec3f`: `x * x + y * y + z * z`, not fused
#[inline]
fn squared_norm_f32(v: &Vec3f) -> f32 {
    let (xx, yy, zz) = (v.x * v.x, v.y * v.y, v.z * v.z);
    xx + yy + zz
}

/// PaintReproject.cpp:1437-1492: every destination face classified as invented (no source face),
/// congruent (its three corners, in order, within `tolerance` of its source face's) or a fragment
pub fn classify_cut_faces(
    src_mesh: &indexed_triangle_set,
    dst_mesh: &indexed_triangle_set,
    dst_to_src_face: &[i32],
    dst_in_source: &DestinationInSourceFrame,
    tolerance: f32,
) -> CutFaceClassification {
    let dst_face_count = dst_mesh.indices.len();
    let src_root_count = src_mesh.indices.len() as i32;
    let tolerance_sq = tolerance * tolerance;

    let mut classification = CutFaceClassification {
        origin: vec![CutFaceOrigin::Invented; dst_face_count],
        fragment_faces: vec![0; dst_face_count],
        fragment_source_roots: vec![0; src_root_count as usize],
        ..CutFaceClassification::default()
    };

    for dst_face in 0..dst_face_count {
        // Faces past the end of the provenance map, and the cap faces cut_mesh() marks with -1,
        // have no source face at all.
        let source_root = if dst_face < dst_to_src_face.len() { dst_to_src_face[dst_face] } else { -1 };
        if source_root < 0 || source_root >= src_root_count {
            classification.invented_count += 1;
            continue;
        }

        // Congruence requires the same three points in the same order.
        let dst_triangle = dst_in_source.triangle(&dst_mesh.indices[dst_face]);
        let src_face = &src_mesh.indices[source_root as usize];
        let mut congruent = true;
        let mut i = 0;
        while i < 3 && congruent {
            congruent = squared_norm_f32(&(dst_triangle[i] - src_mesh.vertices[src_face[i] as usize])) <= tolerance_sq;
            i += 1;
        }

        if congruent {
            classification.origin[dst_face] = CutFaceOrigin::Congruent;
            classification.congruent_count += 1;
        } else {
            classification.origin[dst_face] = CutFaceOrigin::Fragment;
            classification.fragment_faces[dst_face] = 1;
            classification.fragment_source_roots[source_root as usize] = 1;
            classification.fragment_count += 1;
        }
    }
    classification
}

/// PaintReproject.cpp:1494-1523: per-root bit ranges of a serialized annotation, indexed by
/// source face (empty for a root without paint); an empty table when the layout `serialize()`
/// makes (roots and first bits increasing) does not hold
pub fn annotation_bit_ranges(annotation: &FacetsAnnotation, root_count: i32) -> Vec<(i32, i32)> {
    let data = &annotation.data;
    let total_bits = data.1.len() as i32;
    for i in 1..data.0.len() {
        if data.0[i].0 <= data.0[i - 1].0 || data.0[i].1 <= data.0[i - 1].1 {
            return Vec::new();
        }
    }

    let mut ranges: Vec<(i32, i32)> = vec![(0, 0); root_count.max(0) as usize];
    for i in 0..data.0.len() {
        let root = data.0[i].0;
        if root < 0 || root >= root_count {
            return Vec::new();
        }
        let first = data.0[i].1;
        let last = if i + 1 < data.0.len() { data.0[i + 1].1 } else { total_bits };
        if first < 0 || first >= last || last > total_bits {
            return Vec::new();
        }
        ranges[root as usize] = (first, last);
    }
    ranges
}

/// PaintReproject.cpp:1525-1550: the paint of the selected source roots on its own, with the root
/// numbering unchanged
pub fn extract_source_roots(src: &FacetsAnnotation, roots: &[i8], src_ranges: &[(i32, i32)]) -> AnnotationData {
    let mut extracted: AnnotationData = (Vec::new(), Vec::new());
    if src_ranges.is_empty() {
        return extracted;
    }

    let src_bits = &src.data.1;
    for root in 0..roots.len() {
        if roots[root] == 0 {
            continue;
        }
        let (first, last) = src_ranges[root];
        if first >= last {
            continue;
        }
        extracted.0.push((root as i32, extracted.1.len() as i32));
        extracted.1.extend_from_slice(&src_bits[first as usize..last as usize]);
    }
    extracted
}

/// PaintReproject.cpp:1552-1576: the subdivision tree of every congruent destination face moved
/// over from its source face
pub fn splice_congruent_paint(src: &FacetsAnnotation, dst_to_src_face: &[i32], origin: &[CutFaceOrigin], src_ranges: &[(i32, i32)]) -> AnnotationData {
    let mut spliced: AnnotationData = (Vec::new(), Vec::new());
    if src_ranges.is_empty() {
        return spliced;
    }

    let src_bits = &src.data.1;
    for dst_face in 0..origin.len() {
        if origin[dst_face] != CutFaceOrigin::Congruent {
            continue;
        }
        let (first, last) = src_ranges[dst_to_src_face[dst_face] as usize];
        if first >= last {
            continue; // The source face is unpainted and unsplit.
        }
        spliced.0.push((dst_face as i32, spliced.1.len() as i32));
        spliced.1.extend_from_slice(&src_bits[first as usize..last as usize]);
    }
    spliced
}

// ---------------------------------------------------------------------------
// High-level entry points (PaintReproject.cpp:1578-1872).
// ---------------------------------------------------------------------------

/// `L("...")`: the progress labels are handed over untranslated (the CLI has no translation).
const RESTORING_SUPPORT_PAINTING: &str = "Restoring support painting";
const RESTORING_SEAM_PAINTING: &str = "Restoring seam painting";
const RESTORING_PAINT_COLOR: &str = "Restoring paint color";
const RESTORING_FUZZY_SKIN: &str = "Restoring fuzzy skin";
const RESTORING_PAINTING: &str = "Restoring painting";

/// Model.hpp:923 `class ModelVolume`, the members `reproject_paint` reads and writes: its mesh
/// (`mesh().its`) and its four facet-annotation layers.
#[derive(Debug, Clone, Default)]
pub struct ModelVolume {
    pub mesh: indexed_triangle_set,
    pub supported_facets: FacetsAnnotation,
    pub seam_facets: FacetsAnnotation,
    pub mmu_segmentation_facets: FacetsAnnotation,
    pub fuzzy_skin_facets: FacetsAnnotation,
}

/// PaintReproject.cpp:1582-1723 `reproject_paint`: cutting. Provenance from the explicit
/// destination->source face map, measured by in-plane coplanar overlap; destination_to_source maps
/// a destination-mesh vertex back into the source-mesh frame. Returns false if canceled, leaving the
/// destination annotations for the caller to discard.
///
pub fn reproject_paint(
    src_volume: &ModelVolume,
    dst_volume: &mut ModelVolume,
    dst_to_src_face: &[i32],
    destination_to_source: &Transform3d,
    mut progress: Option<&mut PaintReprojectProgressCallback<'_>>,
    mut cancel: Option<&mut PaintReprojectCancelCallback<'_>>,
) -> bool {
    // Before anything else: a cut that is already being canceled should not pay for the prologue.
    if let Some(c) = cancel.as_mut() {
        if c() {
            return false;
        }
    }
    if dst_to_src_face.is_empty() {
        return true;
    }
    let src_mesh = &src_volume.mesh;
    let dst_mesh = &dst_volume.mesh;
    if src_mesh.indices.is_empty() || dst_mesh.indices.is_empty() {
        return true;
    }

    // Same layer labels as the repair path.
    let src_layers: [&FacetsAnnotation; 4] = [&src_volume.supported_facets, &src_volume.seam_facets, &src_volume.mmu_segmentation_facets, &src_volume.fuzzy_skin_facets];
    let names = ["support", "seam", "mmu", "fuzzy skin"];
    let displays = [RESTORING_SUPPORT_PAINTING, RESTORING_SEAM_PAINTING, RESTORING_PAINT_COLOR, RESTORING_FUZZY_SKIN];
    let mut active_layers = 0;
    for layer in &src_layers {
        if !layer.is_empty() {
            active_layers += 1;
        }
    }
    if active_layers == 0 {
        return true;
    }

    // Geometry-only, so it is computed once and shared by all four layers.
    let dst_in_source = DestinationInSourceFrame::new(dst_mesh, destination_to_source);
    let src_diagonal = mesh_diagonal(src_mesh);
    let classification = classify_cut_faces(src_mesh, dst_mesh, dst_to_src_face, &dst_in_source, (src_diagonal * CUT_CONGRUENCE_RELATIVE_TOLERANCE) as f32);

    // Same slab half-width and refinement target as the repair path.
    let band = if src_diagonal > 0.0 { src_diagonal / 100.0 } else { -1.0 };
    // No world matrix: the cut slices a mesh it already transformed by instance x volume.
    let edge_limit = RegionEdgeLimit::new(dst_mesh, None);
    // The mask is the same for all four layers, so this is geometry-only as well.
    let mut seed_index = DestinationSeedIndex::new(dst_mesh, &dst_in_source, Some(&classification.fragment_faces));

    let mut layer_index = 0;
    for li in 0..4usize {
        let layer_src = src_layers[li];
        if layer_src.is_empty() {
            continue;
        }
        if let Some(c) = cancel.as_mut() {
            if c() {
                return false;
            }
        }

        // Distribute the [0, 100] range evenly across the non-empty layers.
        let base_percent = layer_index * 100 / active_layers;
        let span_percent = 100 / active_layers;
        layer_index += 1;
        if let Some(p) = progress.as_mut() {
            p(base_percent, Some(displays[li]));
        }
        let mut layer_progress_fn = progress.as_mut().map(|p| move |percent: i32, message: Option<&str>| p(base_percent + percent * span_percent / 100, message));
        let layer_progress: Option<&mut PaintReprojectProgressCallback> = match layer_progress_fn.as_mut() {
            Some(f) => Some(f),
            None => None,
        };

        let started_at = std::time::Instant::now();
        let src_ranges = annotation_bit_ranges(layer_src, src_mesh.indices.len() as i32);
        let layer_dst: &mut FacetsAnnotation = match li {
            0 => &mut dst_volume.supported_facets,
            1 => &mut dst_volume.seam_facets,
            2 => &mut dst_volume.mmu_segmentation_facets,
            _ => &mut dst_volume.fuzzy_skin_facets,
        };
        if src_ranges.is_empty() {
            // The serialized layout the splice relies on does not hold: resample the whole volume.
            crate::boost_log_trivial!(Error, "Paint reprojection [", names[li], "] has an unexpected serialized layout, falling back to resampling");
            let cancel_cb: Option<&mut PaintReprojectCancelCallback> = match cancel.as_mut() {
                Some(c) => Some(&mut **c),
                None => None,
            };
            if !reproject_one_annotation(layer_src, layer_dst, src_mesh, dst_mesh, destination_to_source, Some(dst_to_src_face), ReprojectMode::OverlapCoplanar, names[li], layer_progress, cancel_cb, -1.0, Some(displays[li]), REPROJECT_ABSOLUTE_ERROR_EPSILON) {
                return false;
            }
            continue;
        }

        let mut dst_sel = selector_of(dst_mesh);
        deserialize_default(&mut dst_sel, &splice_congruent_paint(layer_src, dst_to_src_face, &classification.origin, &src_ranges), true);
        let spliced_nodes = count_valid_nodes(&dst_sel);

        // Measure the paint only where the cut re-triangulated the surface.
        let mut stats = RegionReprojectStats::default();
        let mut fragment_source_leaves = 0usize;
        if classification.fragment_count > 0 {
            let fragment_paint = extract_source_roots(layer_src, &classification.fragment_source_roots, &src_ranges);
            let source = PaintRegionSource::new(src_mesh, &fragment_paint, band, Some(&classification.fragment_source_roots));
            fragment_source_leaves = source.leaf_count();
            if !source.empty() {
                let cancel_cb: Option<&mut PaintReprojectCancelCallback> = match cancel.as_mut() {
                    Some(c) => Some(&mut **c),
                    None => None,
                };
                stats = reproject_one_annotation_by_region(&source, &mut dst_sel, &mut seed_index, &dst_in_source, edge_limit.value(), if band > 0.0 { band as f32 } else { 0.0 }, layer_progress, cancel_cb, Some(displays[li]), Some(&classification.fragment_faces), edge_limit.trafo());
            }
        }
        if stats.canceled {
            return false;
        }
        layer_dst.set(&dst_sel);

        let elapsed_ms = started_at.elapsed().as_millis() as u64;
        crate::boost_log_trivial!(
            Info,
            "Paint reprojection [", names[li], "] by cut splice: congruent faces=", classification.congruent_count,
            ", fragments=", classification.fragment_count, ", invented=", classification.invented_count,
            ", spliced nodes=", spliced_nodes, ", fragment source leaves=", fragment_source_leaves,
            ", edge limit=", edge_limit.value(), ", fragment nodes created=", stats.nodes_created,
            ", target nodes=", count_valid_nodes(&dst_sel), ", budget exhausted=", stats.budget_exhausted,
            ", elapsed=", elapsed_ms, "ms"
        );
    }
    if let Some(p) = progress.as_mut() {
        p(100, Some(RESTORING_PAINTING));
    }
    true
}

/// PaintReproject.cpp:1725-1872 `reproject_paint_geometric`: repair/boolean. No face map; the
/// paint of each layer is re-painted state by state by region (the point-sampling path only for
/// a pathologically large source). dst_to_src maps a destination-mesh vertex into the source frame;
/// dst_world_matrix (None for a mesh already in millimeters) keeps the subdivision floor a real
/// millimeter distance. Returns false if canceled.
#[allow(clippy::too_many_arguments)]
pub fn reproject_paint_geometric(
    src_mesh: &indexed_triangle_set,
    src_supported: &FacetsAnnotation,
    src_seam: &FacetsAnnotation,
    src_mmu: &FacetsAnnotation,
    src_fuzzy: &FacetsAnnotation,
    dst_mesh: &indexed_triangle_set,
    dst_to_src: &Transform3d,
    dst_supported: &mut FacetsAnnotation,
    dst_seam: &mut FacetsAnnotation,
    dst_mmu: &mut FacetsAnnotation,
    dst_fuzzy: &mut FacetsAnnotation,
    mut progress: Option<&mut PaintReprojectProgressCallback<'_>>,
    mut cancel: Option<&mut PaintReprojectCancelCallback<'_>>,
    dst_world_matrix: Option<&Transform3d>,
) -> bool {
    // Bail out before the whole-mesh prologue, and before resetting anything.
    if let Some(c) = cancel.as_mut() {
        if c() {
            return false;
        }
    }

    // Geometric callers rebuild the whole annotation from scratch, so clear any stale paint first.
    dst_supported.reset();
    dst_seam.reset();
    dst_mmu.reset();
    dst_fuzzy.reset();
    if dst_mesh.indices.is_empty() || src_mesh.indices.is_empty() {
        return true;
    }

    let src_layers: [&FacetsAnnotation; 4] = [src_supported, src_seam, src_mmu, src_fuzzy];
    let dst_layers: [&mut FacetsAnnotation; 4] = [dst_supported, dst_seam, dst_mmu, dst_fuzzy];
    let names = ["support", "seam", "mmu", "fuzzy skin"];
    let displays = [RESTORING_SUPPORT_PAINTING, RESTORING_SEAM_PAINTING, RESTORING_PAINT_COLOR, RESTORING_FUZZY_SKIN];
    let mut active_layers = 0;
    for layer in &src_layers {
        if !layer.is_empty() {
            active_layers += 1;
        }
    }
    if active_layers == 0 {
        return true;
    }

    // New-geometry tolerance: 1/100 of the source bounding-box diagonal.
    let src_diagonal = mesh_diagonal(src_mesh);
    let max_sample_distance = if src_diagonal > 0.0 { src_diagonal / 100.0 } else { -1.0 };

    // Floor the per-face convergence target at a fraction of the whole destination mesh area.
    let mut dst_total_area = 0.0f64;
    for face in &dst_mesh.indices {
        let a = dst_mesh.vertices[face[0] as usize];
        let b = dst_mesh.vertices[face[1] as usize];
        let c = dst_mesh.vertices[face[2] as usize];
        dst_total_area += 0.5 * norm_f32(&eigen_cross3f(&(b - a), &(c - a))) as f64;
    }
    let absolute_target_error = std_max_f64(REPROJECT_ABSOLUTE_ERROR_EPSILON, REPROJECT_ABSOLUTE_TARGET_AREA_FRACTION * dst_total_area);

    // Destination leaf edge length for the region path.
    let edge_limit = RegionEdgeLimit::new(dst_mesh, dst_world_matrix);

    let dst_in_source = DestinationInSourceFrame::new(dst_mesh, dst_to_src);
    // Repair paints the whole destination, so no mask; shared by all four layers.
    let mut seed_index = DestinationSeedIndex::new(dst_mesh, &dst_in_source, None);

    let mut layer_index = 0;
    for li in 0..4usize {
        let layer_src = src_layers[li];
        if layer_src.is_empty() {
            continue;
        }
        let base_percent = layer_index * 100 / active_layers;
        let span_percent = 100 / active_layers;
        let mut layer_progress_fn = progress.as_mut().map(|p| move |percent: i32, message: Option<&str>| p(base_percent + percent * span_percent / 100, message));
        let layer_progress: Option<&mut PaintReprojectProgressCallback> = match layer_progress_fn.as_mut() {
            Some(f) => Some(f),
            None => None,
        };

        // Re-paint each paint state as one brush stroke.
        let source = PaintRegionSource::new(src_mesh, &layer_src.data, max_sample_distance, None);
        let mut completed = true;
        if !source.empty() && source.leaf_count() <= REGION_PAINT_MAX_SOURCE_LEAVES {
            let started_at = std::time::Instant::now();
            let mut dst_sel = selector_of(dst_mesh);
            let cancel_cb: Option<&mut PaintReprojectCancelCallback> = match cancel.as_mut() {
                Some(c) => Some(&mut **c),
                None => None,
            };
            let stats = reproject_one_annotation_by_region(&source, &mut dst_sel, &mut seed_index, &dst_in_source, edge_limit.value(), if max_sample_distance > 0.0 { max_sample_distance as f32 } else { 0.0 }, layer_progress, cancel_cb, Some(displays[li]), None, edge_limit.trafo());
            completed = !stats.canceled;
            if completed {
                let elapsed_ms = started_at.elapsed().as_millis() as u64;
                crate::boost_log_trivial!(
                    Info,
                    "Paint reprojection [", names[li], "] by region: source leaves=", source.leaf_count(),
                    ", states=", source.states().len(), ", source finest leaf=", source.finest_leaf_edge(),
                    ", edge limit=", edge_limit.value(), ", nodes created=", stats.nodes_created,
                    ", target nodes=", count_valid_nodes(&dst_sel), ", budget exhausted=", stats.budget_exhausted,
                    ", elapsed=", elapsed_ms, "ms"
                );
                if stats.budget_exhausted {
                    crate::boost_log_trivial!(Warning, "Paint reprojection [", names[li], "] by region exhausted its node budget; the paint boundary stays coarser than ", edge_limit.value(), " in places");
                }
                dst_layers[li].set(&dst_sel);
            }
        } else if !source.empty() {
            // Pathological annotation: fall back to point sampling for this layer.
            crate::boost_log_trivial!(Warning, "Paint reprojection [", names[li], "] has ", source.leaf_count(), " source leaves, falling back to point sampling");
            let cancel_cb: Option<&mut PaintReprojectCancelCallback> = match cancel.as_mut() {
                Some(c) => Some(&mut **c),
                None => None,
            };
            completed = reproject_one_annotation(layer_src, &mut *dst_layers[li], src_mesh, dst_mesh, dst_to_src, None, ReprojectMode::PointNearest, names[li], layer_progress, cancel_cb, max_sample_distance, Some(displays[li]), absolute_target_error);
        }
        if !completed {
            return false;
        }
        layer_index += 1;
    }
    if let Some(p) = progress.as_mut() {
        p(100, Some(RESTORING_PAINTING));
    }
    true
}

// ---------------------------------------------------------------------------
// Boolean / merge helpers (PaintReproject.cpp:1877-1978, in part).
// ---------------------------------------------------------------------------

/// PaintReproject.cpp:1895-1905: every painted triangle of `src` appended onto `dst`, its
/// triangle id shifted by `face_offset`
pub fn append_shifted_annotation(dst: &mut FacetsAnnotation, src: &FacetsAnnotation, face_offset: i32) {
    if src.is_empty() {
        return;
    }
    for entry in src.data.0.iter() {
        let tid = entry.0;
        let s = src.get_triangle_as_string(tid);
        if !s.is_empty() {
            dst.set_triangle_from_string(face_offset + tid, &s);
        }
    }
}

/// PaintReproject.cpp:1967-1978: the centroid of a face and its three corners pulled in towards
/// it by a quarter
pub fn face_samples(its: &indexed_triangle_set, f: i32) -> [Vec3f; 4] {
    let tri = &its.indices[f as usize];
    let v0 = its.vertices[tri[0] as usize];
    let v1 = its.vertices[tri[1] as usize];
    let v2 = its.vertices[tri[2] as usize];
    let sum = (v0 + v1) + v2;
    let centroid = Vec3f::new(sum.x / 3.0f32, sum.y / 3.0f32, sum.z / 3.0f32);
    let pull = |v: Vec3f| -> Vec3f {
        // centroid + (v - centroid) * 0.75f, coefficient by coefficient, not fused
        let d = v - centroid;
        let (dx, dy, dz) = (d.x * 0.75f32, d.y * 0.75f32, d.z * 0.75f32);
        Vec3f::new(centroid.x + dx, centroid.y + dy, centroid.z + dz)
    };
    [centroid, pull(v0), pull(v1), pull(v2)]
}

// ---------------------------------------------------------------------------
// Boolean: merged-source reprojection (PaintReproject.cpp:1874-2425).
// ---------------------------------------------------------------------------

/// PaintReproject.hpp:89-105 `struct PaintSourceVolume`: one painted operand feeding a boolean
/// result. Any annotation may be `None` (unpainted for that layer).
#[derive(Debug, Clone, Copy)]
pub struct PaintSourceVolume<'a> {
    pub mesh: Option<&'a indexed_triangle_set>,
    pub supported: Option<&'a FacetsAnnotation>,
    pub seam: Option<&'a FacetsAnnotation>,
    pub mmu: Option<&'a FacetsAnnotation>,
    pub fuzzy: Option<&'a FacetsAnnotation>,
    /// maps this operand's mesh-local coordinates into the destination mesh-local frame
    pub source_to_destination: Transform3d,
    /// filament (1-based) this operand is assigned to; 0 = do not bake
    pub base_filament: i32,
}

impl<'a> Default for PaintSourceVolume<'a> {
    fn default() -> Self {
        PaintSourceVolume { mesh: None, supported: None, seam: None, mmu: None, fuzzy: None, source_to_destination: Transform3d::identity(), base_filament: 0 }
    }
}

/// Model.hpp:850-858 `struct PaintKeepPrepared`: a mesh plus re-projected paint layers.
#[derive(Debug, Clone, Default)]
pub struct PaintKeepPrepared {
    pub mesh: indexed_triangle_set,
    pub supported: FacetsAnnotation,
    pub seam: FacetsAnnotation,
    pub mmu: FacetsAnnotation,
    pub fuzzy: FacetsAnnotation,
}

/// PaintReproject.cpp:1882-1893 `fill_operand_colour`: bake an operand's assigned filament into
/// every unpainted area of its colour map, keeping its brush strokes.
pub fn fill_operand_colour(mesh: &indexed_triangle_set, src_mmu: Option<&FacetsAnnotation>, base_filament: i32, out: &mut FacetsAnnotation) {
    let mut sel = selector_of(mesh);
    if let Some(m) = src_mmu {
        if !m.is_empty() {
            deserialize_default(&mut sel, &m.data, false);
        }
    }
    sel.fill_unpainted(EnforcerBlockerType(base_filament as i8));
    out.set(&sel);
}

/// Eigen's 3x3 `determinant()` (`bruteforce_det3_helper(m,0,1,2) - bruteforce_det3_helper(m,1,0,2)
/// + bruteforce_det3_helper(m,2,0,1)`, each `m(0,a) * (m(1,b) * m(2,c) - m(1,c) * m(2,b))` with the
/// difference fused) of the linear part of a transform
fn determinant3(t: &Transform3d) -> f64 {
    let helper = |a: usize, b: usize, c: usize| -> f64 { t[(0, a)] * t[(1, b)].mul_add(t[(2, c)], -(t[(1, c)] * t[(2, b)])) };
    (helper(0, 1, 2) - helper(1, 0, 2)) + helper(2, 0, 1)
}

/// TriangleMesh.cpp:362-372 `TriangleMesh::transform(const Transform3d &t, bool fix_left_handed)`
/// on the indexed triangle set: every vertex `(t * v.cast<double>()).cast<float>()`
/// (admesh/stl.h:371-380), and the faces flipped (stl `its_flip_triangles`) for a left-handed t.
pub fn its_transform_fixed(its: &mut indexed_triangle_set, t: &Transform3d, fix_left_handed: bool) {
    for v in its.vertices.iter_mut() {
        *v = transform_point(v, t);
    }
    let det = determinant3(t);
    if fix_left_handed && det < 0.0 {
        for face in its.indices.iter_mut() {
            face.swap_rows(1, 2); // std::swap(face(1), face(2))
        }
    }
}

/// TriangleMesh.cpp:1827-1837 `its_merge(A, B)`
fn its_merge(a: &mut indexed_triangle_set, b: &indexed_triangle_set) {
    let n = a.vertices.len() as i32;
    let n_f = a.indices.len();
    a.vertices.extend_from_slice(&b.vertices);
    a.indices.extend_from_slice(&b.indices);
    for idx in n_f..a.indices.len() {
        a.indices[idx] += Vec3i::new(n, n, n);
    }
}

/// PaintReproject.cpp:1907-1914 `struct BooleanOperand`
pub struct BooleanOperand {
    pub mesh: indexed_triangle_set,
    pub ann: PaintKeepPrepared,
    pub base_filament: i32,
    pub tree: TreeF,
    /// `BoundingBoxf3 bbox`: (min, max)
    pub bbox: (Vec3d, Vec3d),
    pub extent: f64,
}

/// PaintReproject.cpp:1916-1917: slab half-width reproject_paint_geometric() uses for an operand.
pub fn operand_band(op: &BooleanOperand) -> f64 {
    op.extent / 100.0
}

/// PaintReproject.cpp:1919-1923: how far from its bounding box an operand can still put paint on
/// a destination face.
pub fn operand_reach(op: &BooleanOperand) -> f64 {
    // `2.0 * 1.7320508 * operand_band(op) + 1.0e-3`: one expression, fused as
    // `fma(2.0 * 1.7320508, band, 1.0e-3)` (the constant product is exact)
    (2.0f64 * 1.7320508f64).mul_add(operand_band(op), 1.0e-3)
}

/// PaintReproject.cpp:1925-1932 `struct OperandHit`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OperandHit {
    pub op: i32,
    pub distance: f32,
    pub covers: bool,
}

impl Default for OperandHit {
    fn default() -> Self {
        OperandHit { op: -1, distance: f32::MAX, covers: false }
    }
}

/// PaintReproject.cpp:1934-1965 `measure_operand`: the squared distance from the face centroid,
/// and whether the closest point of some sample falls strictly inside one of the operand's faces.
pub fn measure_operand(op: &BooleanOperand, op_idx: i32, samples: &[Vec3f; 4]) -> OperandHit {
    let mut hit = OperandHit { op: op_idx, ..OperandHit::default() };
    for s in 0..4usize {
        let mut hit_face: usize = 0;
        let mut closest = Vec3f::zeros();
        let d2 = squared_distance_to_indexed_triangle_set_f(&op.mesh.vertices, &op.mesh.indices, &op.tree, &samples[s], &mut hit_face, &mut closest) as f64;
        if s == 0 {
            hit.distance = if d2 < 0.0 { f32::MAX } else { d2 as f32 };
        }
        if hit.covers || d2 < 0.0 || d2 > 1.0e-6 || hit_face >= op.mesh.indices.len() {
            continue;
        }
        let sf = op.mesh.indices[hit_face];
        let a = op.mesh.vertices[sf[0] as usize];
        let v0 = op.mesh.vertices[sf[1] as usize] - a;
        let v1 = op.mesh.vertices[sf[2] as usize] - a;
        let v2 = closest - a;
        let (d00, d01, d11) = (eigen_dot3f(&v0, &v0), eigen_dot3f(&v0, &v1), eigen_dot3f(&v1, &v1));
        let (d20, d21) = (eigen_dot3f(&v2, &v0), eigen_dot3f(&v2, &v1));
        let den = d00.mul_add(d11, -(d01 * d01));
        if den.abs() > 0.0 {
            let v = d11.mul_add(d20, -(d01 * d21)) / den;
            let w = d00.mul_add(d21, -(d01 * d20)) / den;
            let u = 1.0f32 - v - w;
            let e = 1.0e-4f32;
            if u > e && v > e && w > e {
                hit.covers = true;
            }
        }
    }
    hit
}

/// `Tree3f` input node of a reach box (PaintReproject.cpp:1995-2002 `struct ReachBox`)
#[derive(Clone, Copy)]
struct ReachBox {
    m_idx: usize,
    m_bbox: BoundingBox3f,
    m_centroid: Vec3f,
}

impl crate::aabb_tree_indirect::SourceNodeF for ReachBox {
    fn idx(&self) -> usize {
        self.m_idx
    }
    fn bbox(&self) -> BoundingBox3f {
        self.m_bbox
    }
    fn centroid(&self) -> Vec3f {
        self.m_centroid
    }
}

/// PaintReproject.cpp:1980-2095 `rank_operands_per_face`: the operands that can own each
/// destination face, nearest first.
///
/// FIDELITY-NOTE: the C++ ranks the faces on the TBB pool (`tbb::parallel_for` over blocks of
/// 1024 faces) while its calling thread polls `cancel` and reports progress every 50 ms until the
/// workers finish, so how often the callbacks run depends on timing. Here the faces are ranked in
/// order on the calling thread (each face writes only its own entry, so the ranking is the same)
/// and the callbacks run once per block of 1024 faces, before it.
pub fn rank_operands_per_face(
    dst: &indexed_triangle_set,
    ops: &[BooleanOperand],
    ranked: &mut Vec<Vec<OperandHit>>,
    progress: Option<&mut dyn FnMut(i32)>,
    mut cancel: Option<&mut PaintReprojectCancelCallback<'_>>,
) -> bool {
    let mut progress = progress;
    let dst_nf = dst.indices.len() as i32;
    ranked.clear();
    ranked.resize(dst_nf.max(0) as usize, Vec::new());

    // A tree over the operands' reach boxes picks the candidates for a face in log time.
    let mut reach_boxes: Vec<ReachBox> = Vec::with_capacity(ops.len());
    for (i, op) in ops.iter().enumerate() {
        let r = operand_reach(op) as f32;
        let rv = Vec3f::new(r, r, r);
        let mn = Vec3f::new(op.bbox.0.x as f32, op.bbox.0.y as f32, op.bbox.0.z as f32) - rv;
        let mx = Vec3f::new(op.bbox.1.x as f32, op.bbox.1.y as f32, op.bbox.1.z as f32) + rv;
        let bbox = BoundingBox3f::new(mn, mx);
        reach_boxes.push(ReachBox { m_idx: i, m_bbox: bbox, m_centroid: bbox.center() });
    }
    let mut reach_tree = TreeF::new();
    reach_tree.build(reach_boxes);
    // A candidate nearer than the smallest reach cannot be beaten by any non-candidate.
    let mut min_reach = f64::MAX;
    for op in ops {
        min_reach = std_min_f64(min_reach, operand_reach(op));
    }
    let min_reach_sq = (min_reach * min_reach) as f32;

    let distance_key = |d: f32| -> i64 { (std_min_f64(d as f64, 1.0e12) * 1.0e6).round() as i64 };

    let mut canceled = false;
    let mut done: i64 = 0;
    let mut candidates: Vec<usize> = Vec::new();
    let mut block_begin: i32 = 0;
    while block_begin < dst_nf {
        // The calling thread's poll of PaintReproject.cpp:2084-2090.
        if !canceled {
            if let Some(c) = cancel.as_mut() {
                if c() {
                    canceled = true;
                }
            }
        }
        if let Some(p) = progress.as_mut() {
            if dst_nf > 0 {
                p((done * 100 / dst_nf as i64) as i32);
            }
        }
        if canceled {
            break;
        }
        let block_end = (block_begin + 1024).min(dst_nf);
        for f in block_begin..block_end {
            let samples = face_samples(dst, f);
            let mut face_box = BoundingBox3f::new(samples[0], samples[0]);
            for k in 0..3usize {
                face_box.extend(&dst.vertices[dst.indices[f as usize][k] as usize]);
            }
            candidates.clear();
            traverse_f(&reach_tree, intersecting_f(face_box), |node: &NodeF| {
                candidates.push(node.idx);
                true
            });
            let hits = &mut ranked[f as usize];
            hits.reserve(candidates.len());
            let mut any_covers = false;
            let mut nearest = f32::MAX;
            for &i in &candidates {
                hits.push(measure_operand(&ops[i], i as i32, &samples));
                any_covers |= hits.last().unwrap().covers;
                let d = hits.last().unwrap().distance;
                nearest = if d < nearest { d } else { nearest };
            }
            // Otherwise the owner is the nearest operand, and only a candidate nearer than
            // min_reach is sure to be it; failing that, measure every operand.
            if !any_covers && nearest > min_reach_sq && candidates.len() < ops.len() {
                hits.clear();
                for i in 0..ops.len() {
                    hits.push(measure_operand(&ops[i], i as i32, &samples));
                }
            }
            crate::clipper::libcxx_sort(hits, |a, b| {
                if a.covers != b.covers {
                    return a.covers;
                }
                let (ka, kb) = (distance_key(a.distance), distance_key(b.distance));
                if ka != kb {
                    return ka < kb;
                }
                if ops[a.op as usize].extent != ops[b.op as usize].extent {
                    return ops[a.op as usize].extent < ops[b.op as usize].extent;
                }
                a.op < b.op
            });
        }
        done += (block_end - block_begin) as i64;
        block_begin = block_end;
    }
    !canceled
}

/// PaintReproject.cpp:2097-2197 `paint_by_operand_filament`: a boolean result whose operands
/// carry no brush paint, only their own filament.
#[allow(clippy::too_many_arguments)]
fn paint_by_operand_filament(
    dst_mesh: &indexed_triangle_set,
    ops: &[BooleanOperand],
    ranked: &[Vec<OperandHit>],
    dst_mmu: &mut FacetsAnnotation,
    dst_world_matrix: Option<&Transform3d>,
    cover_span: i32,
    report: &mut dyn FnMut(i32),
    mut cancel: Option<&mut PaintReprojectCancelCallback<'_>>,
    started_at: std::time::Instant,
) -> bool {
    let dst_nf = dst_mesh.indices.len() as i32;
    let near_enough = |hit: &OperandHit| -> bool {
        if hit.covers {
            return true;
        }
        let band = operand_band(&ops[hit.op as usize]);
        hit.distance as f64 <= band * band
    };

    let mut filament: Vec<i32> = vec![0; dst_nf as usize];
    let mut seam: Vec<i8> = vec![0; dst_nf as usize];
    let mut seam_count = 0usize;
    for f in 0..dst_nf as usize {
        let hits = &ranked[f];
        if hits.is_empty() {
            continue;
        }
        let own = ops[hits[0].op as usize].base_filament;
        filament[f] = own;
        for k in 1..hits.len() {
            if ops[hits[k].op as usize].base_filament != own && near_enough(&hits[k]) {
                seam[f] = 1;
                seam_count += 1;
                break;
            }
        }
    }

    let mut colour = selector_of(dst_mesh);
    for f in 0..dst_nf as usize {
        if seam[f] == 0 && filament[f] > 0 {
            colour.set_facet(f as i32, EnforcerBlockerType(filament[f] as i8));
        }
    }

    if seam_count > 0 {
        let edge_limit = RegionEdgeLimit::new(dst_mesh, dst_world_matrix);
        let mut order: Vec<usize> = (0..ops.len()).collect();
        // std::stable_sort(order, ops[a].extent > ops[b].extent)
        order.sort_by(|&a, &b| {
            if ops[a].extent > ops[b].extent {
                std::cmp::Ordering::Less
            } else if ops[b].extent > ops[a].extent {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Equal
            }
        });

        // Seam faces each operand can paint, collected in one pass over the faces.
        let mut seeds: Vec<Vec<i32>> = vec![Vec::new(); ops.len()];
        for f in 0..dst_nf as usize {
            if seam[f] != 0 {
                for hit in &ranked[f] {
                    if near_enough(hit) {
                        seeds[hit.op as usize].push(f as i32);
                    }
                }
            }
        }

        for n in 0..order.len() {
            if let Some(c) = cancel.as_mut() {
                if c() {
                    return false;
                }
            }
            report(cover_span + (n * (99 - cover_span) as usize / order.len()) as i32);
            let i = order[n];
            let op = &ops[i];
            if op.base_filament <= 0 || seeds[i].is_empty() {
                continue;
            }
            let source = PaintRegionSource::new(&op.mesh, &op.ann.mmu.data, operand_band(op), None);
            if source.empty() {
                continue;
            }
            // Same budget rule as reproject_one_annotation_by_region().
            let node_budget = node_budget_for(source.leaf_count());
            let state = EnforcerBlockerType(op.base_filament as i8);
            let mut coverage = |triangle: &[Vec3f; 3]| -> RegionCoverage { source.coverage(state, triangle) };
            let cancel_cb: Option<&mut FacetSubdivisionCancelCallback> = match cancel.as_mut() {
                Some(c) => Some(&mut **c),
                None => None,
            };
            let result = colour.paint_region(&seeds[i], Some(&mut coverage), state, edge_limit.value(), node_budget, REPROJECT_MAX_SUBDIVISION_DEPTH, cancel_cb, Some(&seam), edge_limit.trafo());
            if result.canceled {
                return false;
            }
        }
        // A seam face that no operand painted keeps its owner's filament.
        for f in 0..dst_nf as usize {
            if seam[f] != 0 && filament[f] > 0 {
                let tr = &colour.get_triangles()[f];
                if !tr.is_split() && tr.get_state() == EnforcerBlockerType::NONE {
                    colour.set_facet(f as i32, EnforcerBlockerType(filament[f] as i8));
                }
            }
        }
    }
    dst_mmu.set(&colour);
    report(99);
    let ms = started_at.elapsed().as_millis() as u64;
    crate::boost_log_trivial!(Info, "Paint reprojection by filament: operands ", ops.len(), ", faces ", dst_nf, ", seam faces ", seam_count, ", elapsed ", ms, " ms");
    true
}

/// PaintReproject.cpp:2200-2425 `reproject_paint_from_volumes`: boolean/merge. Each operand is
/// reprojected on its own through reproject_paint_geometric and every destination face takes the
/// paint of the nearest operand that covers it; concatenated_result marks a destination that is
/// exactly the operands concatenated (a merge), copied face by face.
#[allow(clippy::too_many_arguments)]
pub fn reproject_paint_from_volumes(
    dst_mesh: &indexed_triangle_set,
    sources: &[PaintSourceVolume],
    dst_supported: &mut FacetsAnnotation,
    dst_seam: &mut FacetsAnnotation,
    dst_mmu: &mut FacetsAnnotation,
    dst_fuzzy: &mut FacetsAnnotation,
    mut progress: Option<&mut PaintReprojectProgressCallback<'_>>,
    mut cancel: Option<&mut PaintReprojectCancelCallback<'_>>,
    dst_world_matrix: Option<&Transform3d>,
    concatenated_result: bool,
) -> bool {
    if let Some(c) = cancel.as_mut() {
        if c() {
            return false;
        }
    }

    // Merge every painted operand mesh (in the destination frame) and its four annotation layers
    // into a single combined source.
    let mut combined = PaintKeepPrepared::default();
    let mut face_offset = 0i32;

    for src in sources {
        let src_mesh = match src.mesh {
            Some(m) if !m.indices.is_empty() => m,
            _ => continue,
        };
        // Each operand's colour is filled in its own frame first.
        let mut filled = PaintKeepPrepared::default();
        let mut mmu_src: Option<&FacetsAnnotation> = src.mmu;
        if src.base_filament > 0 {
            fill_operand_colour(src_mesh, src.mmu, src.base_filament, &mut filled.mmu);
            mmu_src = Some(&filled.mmu);
        }
        if let Some(a) = src.supported {
            append_shifted_annotation(&mut combined.supported, a, face_offset);
        }
        if let Some(a) = src.seam {
            append_shifted_annotation(&mut combined.seam, a, face_offset);
        }
        if let Some(a) = mmu_src {
            append_shifted_annotation(&mut combined.mmu, a, face_offset);
        }
        if let Some(a) = src.fuzzy {
            append_shifted_annotation(&mut combined.fuzzy, a, face_offset);
        }
        let mut m = src_mesh.clone();
        its_transform_fixed(&mut m, &src.source_to_destination, true);
        its_merge(&mut combined.mesh, &m);
        face_offset += src_mesh.indices.len() as i32;
    }

    if combined.mesh.indices.is_empty() {
        dst_supported.reset();
        dst_seam.reset();
        dst_mmu.reset();
        dst_fuzzy.reset();
        return true;
    }

    // Merge fast path: the destination mesh is exactly the operands concatenated in order.
    if concatenated_result && dst_mesh.indices.len() == combined.mesh.indices.len() {
        dst_supported.reset();
        dst_seam.reset();
        dst_mmu.reset();
        dst_fuzzy.reset();
        for f in 0..dst_mesh.indices.len() as i32 {
            let s = combined.supported.get_triangle_as_string(f);
            if !s.is_empty() {
                dst_supported.set_triangle_from_string(f, &s);
            }
            let s = combined.seam.get_triangle_as_string(f);
            if !s.is_empty() {
                dst_seam.set_triangle_from_string(f, &s);
            }
            let s = combined.mmu.get_triangle_as_string(f);
            if !s.is_empty() {
                dst_mmu.set_triangle_from_string(f, &s);
            }
            let s = combined.fuzzy.get_triangle_as_string(f);
            if !s.is_empty() {
                dst_fuzzy.set_triangle_from_string(f, &s);
            }
        }
        return true;
    }

    // Boolean: attribute each destination face to the operand whose surface it lies on.
    let mut ops: Vec<BooleanOperand> = Vec::with_capacity(sources.len());
    let mut any_brush_paint = false;
    for src in sources {
        let src_mesh = match src.mesh {
            Some(m) if !m.indices.is_empty() => m,
            _ => continue,
        };
        let painted = |a: Option<&FacetsAnnotation>| a.map_or(false, |a| !a.is_empty());
        any_brush_paint |= painted(src.supported) || painted(src.seam) || painted(src.fuzzy) || painted(src.mmu);
        let mut op = BooleanOperand { mesh: src_mesh.clone(), ann: PaintKeepPrepared::default(), base_filament: src.base_filament, tree: TreeF::new(), bbox: (Vec3d::zeros(), Vec3d::zeros()), extent: 0.0 };
        its_transform_fixed(&mut op.mesh, &src.source_to_destination, true);
        // Copy by triangle string, not assign().
        if painted(src.supported) {
            append_shifted_annotation(&mut op.ann.supported, src.supported.unwrap(), 0);
        }
        if painted(src.seam) {
            append_shifted_annotation(&mut op.ann.seam, src.seam.unwrap(), 0);
        }
        if painted(src.fuzzy) {
            append_shifted_annotation(&mut op.ann.fuzzy, src.fuzzy.unwrap(), 0);
        }
        if src.base_filament > 0 {
            fill_operand_colour(src_mesh, src.mmu, src.base_filament, &mut op.ann.mmu);
        } else if painted(src.mmu) {
            append_shifted_annotation(&mut op.ann.mmu, src.mmu.unwrap(), 0);
        }
        ops.push(op);
    }

    dst_supported.reset();
    dst_seam.reset();
    dst_mmu.reset();
    dst_fuzzy.reset();
    if ops.is_empty() {
        return true;
    }

    let dst_nf = dst_mesh.indices.len() as i32;
    let started_at = std::time::Instant::now();
    for op in ops.iter_mut() {
        op.tree = build_aabb_tree_over_indexed_triangle_set_f(&op.mesh.vertices, &op.mesh.indices, 0.0);
        op.bbox = mesh_bounding_box(&op.mesh);
        op.extent = norm_f64(&(op.bbox.1 - op.bbox.0));
    }

    // Progress: ranking takes the first CoverSpan percent (only with more than one operand).
    let cover_span: i32 = if ops.len() > 1 { 20 } else { 0 };
    let progress_cell = std::cell::RefCell::new(&mut progress);
    let report = |percent: i32| {
        if let Some(p) = progress_cell.borrow_mut().as_mut() {
            p(percent.clamp(0, 99), None);
        }
    };
    report(0);

    // ranked[f]: the operands that can reach face f, nearest first.
    let mut ranked: Vec<Vec<OperandHit>> = Vec::new();
    if ops.len() > 1 {
        let mut rank_progress = |p: i32| report(p * cover_span / 100);
        let cancel_cb: Option<&mut PaintReprojectCancelCallback> = match cancel.as_mut() {
            Some(c) => Some(&mut **c),
            None => None,
        };
        if !rank_operands_per_face(dst_mesh, &ops, &mut ranked, Some(&mut rank_progress), cancel_cb) {
            return false;
        }
    } else {
        ranked = vec![vec![OperandHit { op: 0, distance: 0.0, covers: true }]; dst_nf as usize];
    }

    // Fast path for unpainted operands that only differ in filament.
    if !any_brush_paint {
        let mut report_mut = |p: i32| report(p);
        let cancel_cb: Option<&mut PaintReprojectCancelCallback> = match cancel.as_mut() {
            Some(c) => Some(&mut **c),
            None => None,
        };
        return paint_by_operand_filament(dst_mesh, &ops, &ranked, dst_mmu, dst_world_matrix, cover_span, &mut report_mut, cancel_cb, started_at);
    }

    // Reproject every operand's paint onto the whole destination, then keep per face only its
    // attributed operand's paint.
    let mut per_op: Vec<PaintKeepPrepared> = vec![PaintKeepPrepared::default(); ops.len()];
    let identity = Transform3d::identity();
    for i in 0..ops.len() {
        if let Some(c) = cancel.as_mut() {
            if c() {
                return false;
            }
        }
        let op_base = cover_span + i as i32 * (99 - cover_span) / ops.len() as i32;
        let op_span = (99 - cover_span) / ops.len() as i32;
        let has_progress = progress_cell.borrow().is_some();
        let mut op_progress_fn = |percent: i32, _message: Option<&str>| report(op_base + percent.clamp(0, 100) * op_span / 100);
        let op_progress: Option<&mut PaintReprojectProgressCallback> = if has_progress { Some(&mut op_progress_fn) } else { None };
        report(op_base);
        let cancel_cb: Option<&mut PaintReprojectCancelCallback> = match cancel.as_mut() {
            Some(c) => Some(&mut **c),
            None => None,
        };
        let out = &mut per_op[i];
        let op = &ops[i];
        if !reproject_paint_geometric(&op.mesh, &op.ann.supported, &op.ann.seam, &op.ann.mmu, &op.ann.fuzzy, dst_mesh, &identity, &mut out.supported, &mut out.seam, &mut out.mmu, &mut out.fuzzy, op_progress, cancel_cb, dst_world_matrix) {
            return false;
        }
    }
    report(99);

    // Take, per destination face, only its attributed operand's reprojected paint.
    let take_nearest = |order: &[OperandHit], f: i32, layer: fn(&PaintKeepPrepared) -> &FacetsAnnotation, dst: &mut FacetsAnnotation| -> i32 {
        for hit in order {
            let s = layer(&per_op[hit.op as usize]).get_triangle_as_string(f);
            if !s.is_empty() {
                dst.set_triangle_from_string(f, &s);
                return hit.op;
            }
        }
        -1
    };
    let mut plain_fill: Vec<i32> = vec![0; dst_nf as usize];
    let mut any_plain_fill = false;
    for f in 0..dst_nf {
        let order = &ranked[f as usize];
        take_nearest(order, f, |k| &k.supported, dst_supported);
        take_nearest(order, f, |k| &k.seam, dst_seam);
        take_nearest(order, f, |k| &k.fuzzy, dst_fuzzy);
        if take_nearest(order, f, |k| &k.mmu, dst_mmu) < 0 && !order.is_empty() {
            // No operand reached this face: give it the nearest operand's plain colour.
            let filament = ops[order[0].op as usize].base_filament;
            if filament > 0 {
                plain_fill[f as usize] = filament;
                any_plain_fill = true;
            }
        }
    }
    if any_plain_fill {
        let mut colour = selector_of(dst_mesh);
        if !dst_mmu.is_empty() {
            deserialize_default(&mut colour, &dst_mmu.data, false);
        }
        for f in 0..dst_nf {
            if plain_fill[f as usize] > 0 {
                colour.set_facet(f, EnforcerBlockerType(plain_fill[f as usize] as i8));
            }
        }
        dst_mmu.set(&colour);
    }
    let ms = started_at.elapsed().as_millis() as u64;
    crate::boost_log_trivial!(Info, "Paint reprojection by operand: operands ", ops.len(), ", faces ", dst_nf, ", elapsed ", ms, " ms");
    true
}
