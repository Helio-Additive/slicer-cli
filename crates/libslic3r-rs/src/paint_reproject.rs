//! Port of `PaintReproject.hpp` / `PaintReproject.cpp`, in part: carrying the painted facet
//! annotations of a volume (support, seam, multi-material, fuzzy skin) over to a rebuilt mesh.
//!
//! C++ Reference:
//! - src/libslic3r/PaintReproject.hpp
//! - src/libslic3r/PaintReproject.cpp
//!
//! PARTIAL. Ported here, in the order of the .cpp: the 2D planar helpers of the coplanar-overlap
//! sampling (PaintReproject.cpp:33-139), `DestinationInSourceFrame` (1170-1210),
//! `count_valid_nodes` (1385-1390), the cut classification and the splicing of congruent paint
//! (1392-1576), `append_shifted_annotation` (1895-1905) and `face_samples` (1967-1978). Not ported
//! yet: `PaintReprojector` (149-649), `PaintRegionSource` (651-925), `reproject_one_annotation`
//! (927-1117), `RegionEdgeLimit` and the region reprojection (1119-1383, except
//! `DestinationInSourceFrame`), the entry points `reproject_paint` (1582-1723),
//! `reproject_paint_geometric` (1725-1875) and `reproject_paint_from_volumes` (2200-2425), and the
//! boolean-operand helpers they use (1877-2198, except the two ported functions). They are built
//! on `TriangleSelector::set_facets_with_subdivision`, `paint_region` and `fill_unpainted` of the
//! pin's TriangleSelector.cpp (1033-1300 and on), which `triangle_selector.rs` does not have yet,
//! and on `AABBTreeIndirect::Tree3f` queries in float, which `aabb_tree_indirect.rs` makes in
//! double.
//!
//! Floating point follows the reference build (clang, `-ffp-contract=on`): a fused operation
//! inside one source expression is written `mul_add`; Eigen's operators do not fuse across their
//! calls, except where noted (measured: the affine product, the float cross product).
//!
//! A `const TriangleMesh &` of the C++ is read here only through its `its`, which is what these
//! functions take.

use crate::model::FacetsAnnotation;
use crate::triangle_mesh::{indexed_triangle_set, Vec3f, Vec3i};
use crate::triangle_selector::{SerializedData, TriangleSelector};

/// `Vec2d`
pub type Vec2d = nalgebra::Vector2<f64>;
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
