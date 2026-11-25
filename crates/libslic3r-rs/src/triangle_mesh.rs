//! Faithful 1:1 port of `TriangleMesh.{cpp,hpp}` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/TriangleMesh.hpp (397 lines)
//! - src/libslic3r/TriangleMesh.cpp (2022 lines)
//!
//! Fidelity notes (byte-exact G-code parity):
//! - C++ `coord_t` -> `i64`, `coordf_t` -> `f64`. Mesh vertices are `stl_vertex`
//!   (Eigen `Vec3f`, i.e. `f32`); triangle indices are `stl_triangle_vertex_indices`
//!   (Eigen `Vec3i`). We reuse `crate::normal_utils::indexed_triangle_set` (the
//!   canonical ITS used by `mesh_split_impl`) and `nalgebra` vectors throughout so the
//!   free functions here operate on the same byte layout as the rest of the crate.
//! - `scaled<coord_t>(float)` is reproduced exactly: `coord_t(v / float(SCALING_FACTOR))`
//!   — a *truncating* `f32` division (NOT the rounding `Point::new_scale`/`scale()`).
//!
//! DIVERGENCE (documented, intentional): the application-wide `TriangleMesh` *struct*
//! kept at the bottom of this file is the pre-existing hand-written type consumed by
//! ~24 other modules (`model`, `print_object`, `csg_mesh/*`, `format/*`, `orient`,
//! `slicer`, ...). The C++ `TriangleMesh` class wraps `its: indexed_triangle_set`
//! plus `TriangleMeshStats m_stats` with `f32` vertices, but replacing the existing
//! struct wholesale would break every dependent (the crate must keep building). The
//! parity-critical pieces of `TriangleMesh.cpp` are the *free functions* operating on
//! `indexed_triangle_set`, which are ported faithfully below. The `TriangleMesh` class
//! methods (`from_stl`, `ReadSTLFile`, `scale`, `transform`, `slice`, `convex_hull`,
//! `convex_hull_3d`, ...) depend on not-yet-ported native backends (admesh `stl_file`,
//! qhull, `TriangleMeshSlicer`, `its_transform`/`its_rotate_*`) and are listed as
//! blocked in the porter report.

use crate::geometry::{convex_hull_points, BoundingBox3F, Point, Point3F, Polygon, Transform3D};
pub use crate::normal_utils::indexed_triangle_set;
use crate::normal_utils::{StlTriangleVertexIndices, StlVertex};
use crate::{CoordF, Error, Result, SCALING_FACTOR};
use nalgebra::{Vector2, Vector3};
use serde::{Deserialize, Serialize};
use std::fmt;

/// 3D single-precision vector, mirroring C++ `Vec3f` (Eigen `Matrix<float,3,1>`).
/// Point.hpp
pub type Vec3f = Vector3<f32>;
/// 3D integer index vector, mirroring C++ `Vec3i` (Eigen `Matrix<int,3,1>`).
/// Point.hpp
pub type Vec3i = Vector3<i32>;
/// 2D integer vector, mirroring C++ `Vec2i` (Eigen `Matrix<int,2,1>`).
/// Point.hpp
pub type Vec2i = Vector2<i32>;
/// 2D single-precision vector, mirroring C++ `Vec2f`.
/// Point.hpp
pub type Vec2f = Vector2<f32>;

/// `scaled<coord_t>(const Tin &v)` for a floating `Tin = float`.
/// Point.hpp:536-542
/// C++: `return Tout(v / Tin(SCALING_FACTOR));`
/// Note the truncating `f32` division (no rounding) — required for byte parity with the
/// C++ `coord_t(...)` narrowing conversion.
#[inline]
fn scaled_coord_f32(v: f32) -> i64 {
    // Point.hpp:540: Tout(v / Tin(SCALING_FACTOR)) with Tin = float.
    if float_pin() {
        // libslic3r.h:58 `SCALING_FACTOR = 0.00001`, narrowed to float.
        (v / (SCALING_FACTOR_PIN as f32)) as i64
    } else {
        // The earlier arm: the crate's SCALING_FACTOR is the inverse (100000), so this divided by it.
        (v / SCALING_FACTOR as f32) as i64
    }
}

/// Gate `TRIANGLE_MESH_FLOAT_PIN` (on by default): the float arithmetic of TriangleMesh.cpp as the
/// C++ build evaluates it (Eigen's `cross`, `normalized`, `norm`, `dot` of `Vec3f` in
/// `triangle_set_sampling`'s `eigen_*3f`, and `scaled<coord_t>(float)` dividing by the float
/// SCALING_FACTOR), checked by tests/triangle_mesh_native_parity.rs. Off: the earlier arm,
/// nalgebra's `cross`/`normalize`/`dot` and a division by the crate's inverse SCALING_FACTOR.
#[inline]
fn float_pin() -> bool {
    crate::faithful_gate("TRIANGLE_MESH_FLOAT_PIN")
}

// ============================================================================
// TriangleMesh.hpp:19-45 — RepairedMeshErrors
// ============================================================================

/// TriangleMesh.hpp:19-45
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RepairedMeshErrors {
    /// How many edges were united by merging their end points with some other end points in epsilon neighborhood?
    /// TriangleMesh.hpp:21
    pub edges_fixed: i32,
    /// How many degenerate faces were removed?
    /// TriangleMesh.hpp:23
    pub degenerate_facets: i32,
    /// How many faces were removed during fixing? Includes degenerate_faces and disconnected faces.
    /// TriangleMesh.hpp:25
    pub facets_removed: i32,
    // New faces could only be created with stl_fill_holes() and we ditched stl_fill_holes(), because mostly it does more harm than good.
    //int          facets_added             = 0;
    /// How many facets were revesed? Faces are reversed by admesh while it connects patches of triangles togeter and a flipped triangle is encountered.
    /// Also the facets are reversed when a negative volume is corrected by flipping all facets.
    /// TriangleMesh.hpp:30
    pub facets_reversed: i32,
    /// Edges shared by two triangles, oriented incorrectly.
    /// TriangleMesh.hpp:32
    pub backwards_edges: i32,
}

impl Default for RepairedMeshErrors {
    fn default() -> Self {
        // TriangleMesh.hpp:21-32 — all members default to 0.
        Self {
            edges_fixed: 0,
            degenerate_facets: 0,
            facets_removed: 0,
            facets_reversed: 0,
            backwards_edges: 0,
        }
    }
}

impl RepairedMeshErrors {
    /// TriangleMesh.hpp:34
    /// C++: `void clear() { *this = RepairedMeshErrors(); }`
    pub fn clear(&mut self) {
        *self = RepairedMeshErrors::default();
    }

    /// TriangleMesh.hpp:36-42
    /// C++: `void merge(const RepairedMeshErrors& rhs)`
    pub fn merge(&mut self, rhs: &RepairedMeshErrors) {
        // TriangleMesh.hpp:37-41
        self.edges_fixed += rhs.edges_fixed;
        self.degenerate_facets += rhs.degenerate_facets;
        self.facets_removed += rhs.facets_removed;
        self.facets_reversed += rhs.facets_reversed;
        self.backwards_edges += rhs.backwards_edges;
    }

    /// TriangleMesh.hpp:44
    /// C++: `bool repaired() const { return degenerate_facets > 0 || edges_fixed > 0 || facets_removed > 0 || facets_reversed > 0 || backwards_edges > 0; }`
    pub fn repaired(&self) -> bool {
        self.degenerate_facets > 0
            || self.edges_fixed > 0
            || self.facets_removed > 0
            || self.facets_reversed > 0
            || self.backwards_edges > 0
    }
}

// ============================================================================
// TriangleMesh.hpp:47-93 — TriangleMeshStats
// ============================================================================

/// TriangleMesh.hpp:47-93
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TriangleMeshStats {
    // Mesh metrics.
    /// TriangleMesh.hpp:49
    pub number_of_facets: u32,
    /// TriangleMesh.hpp:50
    pub max: StlVertex,
    /// TriangleMesh.hpp:51
    pub min: StlVertex,
    /// TriangleMesh.hpp:52
    pub size: StlVertex,
    /// TriangleMesh.hpp:53
    pub volume: f32,
    /// TriangleMesh.hpp:54
    pub number_of_parts: i32,

    // Mesh errors, remaining.
    /// TriangleMesh.hpp:57
    pub open_edges: i32,
    /// TriangleMesh.hpp:58
    pub non_manifold_edges: i32,
    /// TriangleMesh.hpp:59
    pub non_manifold_vertices: i32,
    /// TriangleMesh.hpp:60
    pub has_reversed_faces: bool,

    // Mesh errors, fixed.
    /// TriangleMesh.hpp:63
    pub repaired_errors: RepairedMeshErrors,
}

impl Default for TriangleMeshStats {
    fn default() -> Self {
        // TriangleMesh.hpp:49-60 — defaults.
        Self {
            number_of_facets: 0,
            max: StlVertex::zeros(),  // TriangleMesh.hpp:50: stl_vertex::Zero()
            min: StlVertex::zeros(),  // TriangleMesh.hpp:51: stl_vertex::Zero()
            size: StlVertex::zeros(), // TriangleMesh.hpp:52: stl_vertex::Zero()
            volume: -1.0,             // TriangleMesh.hpp:53: -1.f
            number_of_parts: 0,
            open_edges: 0,
            non_manifold_edges: 0,     // TriangleMesh.hpp:58
            non_manifold_vertices: 0,  // TriangleMesh.hpp:59
            has_reversed_faces: false, // TriangleMesh.hpp:60
            repaired_errors: RepairedMeshErrors::default(),
        }
    }
}

impl TriangleMeshStats {
    /// TriangleMesh.hpp:62
    /// C++: `void clear() { *this = TriangleMeshStats(); }`
    pub fn clear(&mut self) {
        *self = TriangleMeshStats::default();
    }

    /// TriangleMesh.hpp:64-81
    /// C++: `TriangleMeshStats merge(const TriangleMeshStats &rhs) const`
    pub fn merge(&self, rhs: &TriangleMeshStats) -> TriangleMeshStats {
        // TriangleMesh.hpp:65-66
        if self.number_of_facets == 0 {
            *rhs
        }
        // TriangleMesh.hpp:67-68
        else if rhs.number_of_facets == 0 {
            *self
        }
        // TriangleMesh.hpp:69-80
        else {
            let mut out = TriangleMeshStats::default();
            out.number_of_facets = self.number_of_facets + rhs.number_of_facets;
            out.min = self.min.inf(&rhs.min); // cwiseMin
            out.max = self.max.sup(&rhs.max); // cwiseMax
            out.size = out.max - out.min;
            out.number_of_parts = self.number_of_parts + rhs.number_of_parts;
            out.open_edges = self.open_edges + rhs.open_edges;
            // TriangleMesh.hpp:78-80
            out.non_manifold_edges = self.non_manifold_edges + rhs.non_manifold_edges;
            out.non_manifold_vertices = self.non_manifold_vertices + rhs.non_manifold_vertices;
            out.has_reversed_faces = self.has_reversed_faces || rhs.has_reversed_faces;
            out.volume = self.volume + rhs.volume;
            out.repaired_errors.merge(&rhs.repaired_errors);
            out
        }
    }

    /// TriangleMesh.hpp:89
    /// C++: `bool manifold() const { return non_manifold_edges == 0 && non_manifold_vertices == 0; }`
    pub fn manifold(&self) -> bool {
        self.non_manifold_edges == 0 && self.non_manifold_vertices == 0
    }

    /// TriangleMesh.hpp:90
    /// C++: `bool has_open_edges() const { return open_edges > 0; }`
    pub fn has_open_edges(&self) -> bool {
        self.open_edges > 0
    }

    /// TriangleMesh.hpp:91
    /// C++: `bool has_any_issue() const { return !manifold() || has_open_edges() || has_reversed_faces; }`
    pub fn has_any_issue(&self) -> bool {
        !self.manifold() || self.has_open_edges() || self.has_reversed_faces
    }

    /// TriangleMesh.hpp:92
    /// C++: `bool repaired() const { return repaired_errors.repaired(); }`
    pub fn repaired(&self) -> bool {
        self.repaired_errors.repaired()
    }
}

// ============================================================================
// TriangleMesh.cpp:37-54 — static helpers used by the TriangleMesh ctors
// ============================================================================

/// TriangleMesh.cpp:37-43
/// C++: `static void update_bounding_box(const indexed_triangle_set &its, TriangleMeshStats &out)`
pub fn update_bounding_box(its: &indexed_triangle_set, out: &mut TriangleMeshStats) {
    // TriangleMesh.cpp:39
    let bbox = bounding_box(its);
    // TriangleMesh.cpp:40-41 — bbox.min/max are f64; cast<float>() back to f32.
    out.min = StlVertex::new(bbox.min.x as f32, bbox.min.y as f32, bbox.min.z as f32);
    out.max = StlVertex::new(bbox.max.x as f32, bbox.max.y as f32, bbox.max.z as f32);
    // TriangleMesh.cpp:42
    out.size = out.max - out.min;
}

/// TriangleMesh.cpp:46-77
/// C++: `static void fill_initial_stats(const indexed_triangle_set &its, TriangleMeshStats &out)`
///
/// Gate `TRIANGLE_MESH_STATS_PIN` (on by default): the pin's body, the edge statistics and the
/// face neighbours of `its_quick_diagnostics` (MeshDiagnostics.cpp:321-326). Off: the earlier
/// arm, the neighbours of `its_face_neighbors` and the open edges counted on them, the
/// non-manifold counts and the reversed-face flag left at zero. Neither arm reaches the G-code:
/// the statistics are read by the mesh reports only.
pub fn fill_initial_stats(its: &indexed_triangle_set, out: &mut TriangleMeshStats) {
    // TriangleMesh.cpp:48
    out.number_of_facets = its.indices.len() as u32;
    // TriangleMesh.cpp:49
    out.volume = its_volume(its);
    // TriangleMesh.cpp:50
    update_bounding_box(its, out);

    if !crate::faithful_gate("TRIANGLE_MESH_STATS_PIN") {
        // The earlier arm.
        let face_neighbors = its_face_neighbors(its);
        out.number_of_parts = its_number_of_patches_neighbors(its, &face_neighbors) as i32;
        out.open_edges = its_num_open_edges_neighbors(&face_neighbors) as i32;
        return;
    }

    // TriangleMesh.cpp:52-53
    let mut face_neighbors: Vec<Vec3i> = Vec::new();
    // mesh_diagnostics.rs takes the `indexed_triangle_set` of triangle_set_sampling.rs (the same
    // two vectors under another type): a copy of them.
    let its_diag = crate::triangle_set_sampling::indexed_triangle_set {
        vertices: its.vertices.clone(),
        indices: its.indices.clone(),
    };
    let nm_stats = crate::mesh_diagnostics::its_quick_diagnostics(&its_diag, Some(&mut face_neighbors));
    // TriangleMesh.cpp:54
    out.number_of_parts = its_number_of_patches_neighbors(its, &face_neighbors) as i32;
    // TriangleMesh.cpp:55 assert(... <= INT_MAX)
    debug_assert!(
        nm_stats.open_edges <= i32::MAX as usize
            && nm_stats.non_manifold_edges <= i32::MAX as usize
            && nm_stats.non_manifold_vertices <= i32::MAX as usize
    );
    // TriangleMesh.cpp:56-59
    out.open_edges = nm_stats.open_edges as i32;
    out.non_manifold_edges = nm_stats.non_manifold_edges as i32;
    out.non_manifold_vertices = nm_stats.non_manifold_vertices as i32;
    out.has_reversed_faces = nm_stats.has_reversed_faces;
    // TriangleMesh.cpp:60-68 BOOST_LOG_TRIVIAL(info)
    log::info!(
        "reversed-faces: mesh-stats faces={} verts={} parts={} volume={} open_edges={} nm_edges={} nm_verts={} has_reversed_faces={}",
        out.number_of_facets,
        its.vertices.len(),
        out.number_of_parts,
        out.volume,
        out.open_edges,
        out.non_manifold_edges,
        out.non_manifold_vertices,
        if out.has_reversed_faces { 1 } else { 0 }
    );
    // TriangleMesh.cpp:69-76
    if out.has_reversed_faces {
        log::info!(
            "reversed-faces: FLAG mesh-stats faces={} verts={} parts={} volume={} open_edges={} nm_edges={}",
            out.number_of_facets,
            its.vertices.len(),
            out.number_of_parts,
            out.volume,
            out.open_edges,
            out.non_manifold_edges
        );
    }
}

/// TriangleMesh.cpp:390-398, the tail of `TriangleMesh::flip_triangles()`: "Topology is unchanged
/// by a global flip; reuse edge counts and only re-run the reversed-face test.
/// same_direction_edges is not stored on TriangleMeshStats: watertight meshes with same-dir edges
/// fall through to the ray test instead of the shortcut."
///
/// Behind the gate of [`fill_initial_stats`], `TRIANGLE_MESH_STATS_PIN`: off, the flag is left
/// as it is (the earlier arm).
pub(crate) fn flip_triangles_detect_reversed_faces(its: &indexed_triangle_set, m_stats: &mut TriangleMeshStats) {
    if !crate::faithful_gate("TRIANGLE_MESH_STATS_PIN") {
        return;
    }
    // TriangleMesh.cpp:394
    let mut st = crate::mesh_diagnostics::MeshDiagnosticStats::default();
    // TriangleMesh.cpp:395-396
    st.open_edges = m_stats.open_edges.max(0) as usize;
    st.non_manifold_edges = m_stats.non_manifold_edges.max(0) as usize;
    // TriangleMesh.cpp:397
    let its_diag = crate::triangle_set_sampling::indexed_triangle_set {
        vertices: its.vertices.clone(),
        indices: its.indices.clone(),
    };
    crate::mesh_diagnostics::its_detect_reversed_faces(&its_diag, &mut st);
    // TriangleMesh.cpp:398
    m_stats.has_reversed_faces = st.has_reversed_faces;
}

// ============================================================================
// TriangleMesh.cpp:575-725 — Face edge IDs
// ============================================================================

/// Create a mapping from triangle edge into face.
/// TriangleMesh.cpp:575-586
#[derive(Clone, Copy, Debug)]
struct EdgeToFace {
    /// Index of the 1st vertex of the triangle edge. vertex_low <= vertex_high.
    /// TriangleMesh.cpp:577
    vertex_low: i32,
    /// Index of the 2nd vertex of the triangle edge.
    /// TriangleMesh.cpp:579
    vertex_high: i32,
    /// Index of a triangular face.
    /// TriangleMesh.cpp:581
    face: i32,
    /// Index of edge in the face, starting with 1. Negative indices if the edge was stored reverse in (vertex_low, vertex_high).
    /// TriangleMesh.cpp:583
    face_edge: i32,
}

impl EdgeToFace {
    /// TriangleMesh.cpp:584
    /// C++: `bool operator==(const EdgeToFace &other) const`
    #[inline]
    fn eq_edge(&self, other: &EdgeToFace) -> bool {
        self.vertex_low == other.vertex_low && self.vertex_high == other.vertex_high
    }
}

/// TriangleMesh.cpp:588-615
/// C++: `template<typename FaceFilter, typename ThrowOnCancelCallback>
///       static std::vector<EdgeToFace> create_edge_map(const indexed_triangle_set &its, FaceFilter face_filter, ThrowOnCancelCallback throw_on_cancel)`
fn create_edge_map(
    its: &indexed_triangle_set,
    mut face_filter: impl FnMut(u32) -> bool,
    throw_on_cancel: impl Fn(),
) -> Vec<EdgeToFace> {
    // TriangleMesh.cpp:592-593
    let mut edges_map: Vec<EdgeToFace> = Vec::with_capacity(its.indices.len() * 3);
    // TriangleMesh.cpp:594-610
    for facet_idx in 0..its.indices.len() as u32 {
        if face_filter(facet_idx) {
            for i in 0..3 {
                // TriangleMesh.cpp:597-598
                let mut e2f = EdgeToFace {
                    // TriangleMesh.cpp:599
                    vertex_low: its.indices[facet_idx as usize][i],
                    // TriangleMesh.cpp:600
                    vertex_high: its.indices[facet_idx as usize][(i + 1) % 3],
                    // TriangleMesh.cpp:601
                    face: facet_idx as i32,
                    // TriangleMesh.cpp:603 — 1 based indexing, to be always strictly positive.
                    face_edge: i as i32 + 1,
                };
                // TriangleMesh.cpp:604
                if e2f.vertex_low > e2f.vertex_high {
                    // TriangleMesh.cpp:606 — Sort the vertices
                    std::mem::swap(&mut e2f.vertex_low, &mut e2f.vertex_high);
                    // TriangleMesh.cpp:608 — and make the face_edge negative to indicate a flipped edge.
                    e2f.face_edge = -e2f.face_edge;
                }
                edges_map.push(e2f);
            }
        }
    }
    // TriangleMesh.cpp:611
    throw_on_cancel();
    // TriangleMesh.cpp:612 — std::sort by operator<: (vertex_low, vertex_high).
    // Gate `TRIANGLE_MESH_EDGE_SORT_PIN` (on by default): libc++'s `std::sort`, which is not
    // stable: the order of the edges of one vertex pair (more than two on a non-manifold edge)
    // decides which faces share an edge ID. Off: the earlier arm, a stable sort.
    if crate::faithful_gate("TRIANGLE_MESH_EDGE_SORT_PIN") {
        crate::clipper::libcxx_sort(&mut edges_map, |a, b| {
            a.vertex_low < b.vertex_low || (a.vertex_low == b.vertex_low && a.vertex_high < b.vertex_high)
        });
    } else {
        edges_map.sort_by(|a, b| (a.vertex_low, a.vertex_high).cmp(&(b.vertex_low, b.vertex_high)));
    }

    // TriangleMesh.cpp:614
    edges_map
}

/// Map from a face edge to a unique edge identifier or -1 if no neighbor exists.
/// Two neighbor faces share a unique edge identifier even if they are flipped.
/// TriangleMesh.cpp:617-668
/// C++: `template<typename FaceFilter, typename ThrowOnCancelCallback>
///       static inline std::vector<Vec3i> its_face_edge_ids_impl(...)`
fn its_face_edge_ids_impl(
    its: &indexed_triangle_set,
    face_filter: impl FnMut(u32) -> bool,
    throw_on_cancel: impl Fn(),
) -> Vec<Vec3i> {
    // TriangleMesh.cpp:622
    let mut out: Vec<Vec3i> = vec![Vec3i::new(-1, -1, -1); its.indices.len()];

    // TriangleMesh.cpp:624
    let mut edges_map = create_edge_map(its, face_filter, &throw_on_cancel);

    // TriangleMesh.cpp:626-665 — Assign a unique common edge id to touching triangle edges.
    let mut num_edges: i32 = 0;
    for i in 0..edges_map.len() {
        // TriangleMesh.cpp:629
        let edge_i = edges_map[i];
        // TriangleMesh.cpp:630-632 — This edge has been connected to some neighbor already.
        if edge_i.face == -1 {
            continue;
        }
        // TriangleMesh.cpp:633-634 — Unconnected edge. Find its neighbor with the correct orientation.
        let mut j: usize;
        let mut found = false;
        // TriangleMesh.cpp:636-641
        j = i + 1;
        while j < edges_map.len() && edge_i.eq_edge(&edges_map[j]) {
            if edge_i.face_edge * edges_map[j].face_edge < 0 && edges_map[j].face != -1 {
                // Faces touching with opposite oriented edges and none of the edges is connected yet.
                found = true;
                break;
            }
            j += 1;
        }
        // TriangleMesh.cpp:642-653
        if !found {
            //FIXME Vojtech: Trying to find an edge with equal orientation. This smells.
            // admesh can assign the same edge ID to more than two facets (which is
            // still topologically correct), so we have to search for a duplicate of
            // this edge too in case it was already seen in this orientation
            j = i + 1;
            while j < edges_map.len() && edge_i.eq_edge(&edges_map[j]) {
                if edges_map[j].face != -1 {
                    // Faces touching with equally oriented edges and none of the edges is connected yet.
                    found = true;
                    break;
                }
                j += 1;
            }
        }
        // TriangleMesh.cpp:655 — Assign an edge index to the 1st face.
        out[edge_i.face as usize][(edge_i.face_edge.abs() - 1) as usize] = num_edges;
        // TriangleMesh.cpp:656-661
        if found {
            let edge_j = edges_map[j];
            out[edge_j.face as usize][(edge_j.face_edge.abs() - 1) as usize] = num_edges;
            // Mark the edge as connected.
            edges_map[j].face = -1;
        }
        // TriangleMesh.cpp:662
        num_edges += 1;
        // TriangleMesh.cpp:663-664
        if (i & 0x0ffff) == 0 {
            throw_on_cancel();
        }
    }

    // TriangleMesh.cpp:667
    out
}

/// TriangleMesh.cpp:670-673
pub fn its_face_edge_ids(its: &indexed_triangle_set) -> Vec<Vec3i> {
    its_face_edge_ids_impl(its, |_| true, || {})
}

/// TriangleMesh.cpp:706-709, the overload of `its_face_edge_ids` with a cancel callback
/// (TriangleMesh.hpp:202).
pub mod its_face_edge_ids_with_cancel {
    use super::*;
    /// TriangleMesh.cpp:706-709
    pub fn its_face_edge_ids(its: &indexed_triangle_set, throw_on_cancel_callback: impl Fn()) -> Vec<Vec3i> {
        its_face_edge_ids_impl(its, |_| true, throw_on_cancel_callback)
    }
}
pub use its_face_edge_ids_with_cancel::its_face_edge_ids as its_face_edge_ids_cb;

/// TriangleMesh.cpp:711-714, the overload of `its_face_edge_ids` with a face mask
/// (TriangleMesh.hpp:203).
pub mod its_face_edge_ids_with_mask {
    use super::*;
    /// TriangleMesh.cpp:711-714
    pub fn its_face_edge_ids(its: &indexed_triangle_set, face_mask: &[bool]) -> Vec<Vec3i> {
        its_face_edge_ids_impl(its, |idx| face_mask[idx as usize], || {})
    }
}
pub use its_face_edge_ids_with_mask::its_face_edge_ids as its_face_edge_ids_mask;

/// TriangleMesh.cpp:717-756, the overload of `its_face_edge_ids` over the face neighbours
/// (TriangleMesh.hpp:205).
pub mod its_face_edge_ids_with_neighbors {
    use super::*;
    /// Having the face neighbors available, assign unique edge IDs to face edges for chaining of polygons over slices.
    /// TriangleMesh.cpp:686-725
    /// C++: `std::vector<Vec3i> its_face_edge_ids(const indexed_triangle_set &its, std::vector<Vec3i> &face_neighbors, bool assign_unbound_edges, int *num_edges)`
    pub fn its_face_edge_ids(
        its: &indexed_triangle_set,
        face_neighbors: &[Vec3i],
        assign_unbound_edges: bool,
        num_edges: Option<&mut i32>,
    ) -> Vec<Vec3i> {
        // TriangleMesh.cpp:689 — out elements are not initialized!
        let mut out: Vec<Vec3i> = vec![Vec3i::zeros(); face_neighbors.len()];
        // TriangleMesh.cpp:690
        let mut last_edge_id: i32 = 0;
        // TriangleMesh.cpp:691
        for i in 0..face_neighbors.len() as i32 {
            // TriangleMesh.cpp:692
            let triangle = its.indices[i as usize];
            // TriangleMesh.cpp:693
            let neighbors = face_neighbors[i as usize];
            // TriangleMesh.cpp:694
            for j in 0..3 {
                // TriangleMesh.cpp:695
                let n = neighbors[j];
                // TriangleMesh.cpp:696
                if n > i {
                    // TriangleMesh.cpp:697
                    let triangle2 = its.indices[n as usize];
                    // TriangleMesh.cpp:698
                    let edge_id = last_edge_id;
                    last_edge_id += 1;
                    // TriangleMesh.cpp:699
                    let mut edge = its_triangle_edge(&triangle, j as i32);
                    // TriangleMesh.cpp:701 — First find an edge with opposite orientation.
                    let tmp = edge[0];
                    edge[0] = edge[1];
                    edge[1] = tmp;
                    // TriangleMesh.cpp:702
                    let mut k = its_triangle_edge_index(&triangle2, &edge);
                    //FIXME is the following realistic? Could face_neighbors contain such faces?
                    // And if it does, do we want to produce the same edge ID for those mutually incorrectly oriented edges?
                    // TriangleMesh.cpp:705
                    if k == -1 {
                        // TriangleMesh.cpp:707 — Second find an edge with the same orientation (the neighbor triangle may be flipped).
                        let tmp2 = edge[0];
                        edge[0] = edge[1];
                        edge[1] = tmp2;
                        // TriangleMesh.cpp:708
                        k = its_triangle_edge_index(&triangle2, &edge);
                    }
                    // TriangleMesh.cpp:710
                    debug_assert!(k >= 0);
                    // TriangleMesh.cpp:711
                    out[i as usize][j] = edge_id;
                    // TriangleMesh.cpp:712
                    out[n as usize][k as usize] = edge_id;
                } else if n == -1 {
                    // TriangleMesh.cpp:714
                    out[i as usize][j] = if assign_unbound_edges {
                        let v = last_edge_id;
                        last_edge_id += 1;
                        v
                    } else {
                        -1
                    };
                } else {
                    // TriangleMesh.cpp:716-718 — Triangle shall never be neighbor of itself.
                    debug_assert!(n < i);
                    // Don't do anything, the neighbor will assign us an edge ID in later iterations.
                }
            }
        }
        // TriangleMesh.cpp:722-723
        if let Some(ne) = num_edges {
            *ne = last_edge_id;
        }
        // TriangleMesh.cpp:724
        out
    }
}
pub use its_face_edge_ids_with_neighbors::its_face_edge_ids as its_face_edge_ids_neighbors;

// ============================================================================
// TriangleMesh.cpp:727-839 — ITS cleanup helpers
// ============================================================================

/// Merge duplicate vertices, return number of vertices removed.
/// TriangleMesh.cpp:727-788
pub fn its_merge_vertices(its: &mut indexed_triangle_set, shrink_to_fit: bool) -> i32 {
    // TriangleMesh.cpp:731-733 — 1) Sort indices to vertices lexicographically by coordinates AND vertex index.
    let mut sorted: Vec<i32> = Vec::with_capacity(its.vertices.len());
    for i in 0..its.vertices.len() as i32 {
        sorted.push(i);
    }
    // TriangleMesh.cpp:734-739
    sorted.sort_by(|&il, &ir| {
        let l = &its.vertices[il as usize];
        let r = &its.vertices[ir as usize];
        // Sort lexicographically by coordinates AND vertex index.
        // l.x() < r.x() || (l.x() == r.x() && (l.y() < r.y() || (l.y() == r.y() && (l.z() < r.z() || (l.z() == r.z() && il < ir)))))
        (l.x, l.y, l.z, il)
            .partial_cmp(&(r.x, r.y, r.z, ir))
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // TriangleMesh.cpp:741-743 — 2) Map duplicate vertices to the one with the lowest vertex index.
    let mut map_vertices: Vec<i32> = vec![-1; its.vertices.len()];
    // TriangleMesh.cpp:744-757
    let mut i = 0usize;
    while i < sorted.len() {
        // TriangleMesh.cpp:745-746
        let u = sorted[i];
        let p = its.vertices[u as usize];
        // TriangleMesh.cpp:747
        let mut j = i;
        // TriangleMesh.cpp:748
        j += 1;
        while j < sorted.len() {
            // TriangleMesh.cpp:749-750
            let v = sorted[j];
            let q = its.vertices[v as usize];
            // TriangleMesh.cpp:751-752
            if p != q {
                break;
            }
            // TriangleMesh.cpp:753
            debug_assert!(v > u);
            // TriangleMesh.cpp:754
            map_vertices[v as usize] = u;
            j += 1;
        }
        // TriangleMesh.cpp:756
        i = j;
    }

    // TriangleMesh.cpp:760-771 — 3) Shrink its.vertices, update map_vertices with the new vertex indices.
    let mut k: i32 = 0;
    for i in 0..its.vertices.len() {
        // TriangleMesh.cpp:762
        if map_vertices[i] == -1 {
            // TriangleMesh.cpp:763
            map_vertices[i] = k;
            // TriangleMesh.cpp:764-765
            if (k as usize) < i {
                its.vertices[k as usize] = its.vertices[i];
            }
            // TriangleMesh.cpp:766
            k += 1;
        } else {
            // TriangleMesh.cpp:768
            debug_assert!(map_vertices[i] < i as i32);
            // TriangleMesh.cpp:769
            map_vertices[i] = map_vertices[map_vertices[i] as usize];
        }
    }

    // TriangleMesh.cpp:773
    let num_erased = its.vertices.len() as i32 - k;

    // TriangleMesh.cpp:775-785
    if num_erased != 0 {
        // TriangleMesh.cpp:777 — Shrink the vertices.
        its.vertices.truncate(k as usize);
        // TriangleMesh.cpp:779-781 — Remap face indices.
        for face in &mut its.indices {
            for i in 0..3 {
                face[i] = map_vertices[face[i] as usize];
            }
        }
        // TriangleMesh.cpp:783-784 — Optionally shrink to fit (reallocate) vertices.
        if shrink_to_fit {
            its.vertices.shrink_to_fit();
        }
    }

    // TriangleMesh.cpp:787
    num_erased
}

/// TriangleMesh.cpp:790-794
/// C++: `void its_flip_triangles(indexed_triangle_set &its)`
pub fn its_flip_triangles(its: &mut indexed_triangle_set) {
    // TriangleMesh.cpp:792-793 — std::swap(face(1), face(2));
    for face in &mut its.indices {
        let tmp = face[1];
        face[1] = face[2];
        face[2] = tmp;
    }
}

/// TriangleMesh.cpp:796-809
/// C++: `int its_remove_degenerate_faces(indexed_triangle_set &its, bool shrink_to_fit)`
pub fn its_remove_degenerate_faces(its: &mut indexed_triangle_set, shrink_to_fit: bool) -> i32 {
    // TriangleMesh.cpp:798-803 — std::remove_if predicate: face(0)==face(1) || face(0)==face(2) || face(1)==face(2).
    let before = its.indices.len();
    its.indices
        .retain(|face| !(face[0] == face[1] || face[0] == face[2] || face[1] == face[2]));
    // TriangleMesh.cpp:802 — number removed.
    let removed = (before - its.indices.len()) as i32;

    // TriangleMesh.cpp:805-806
    if removed != 0 && shrink_to_fit {
        its.indices.shrink_to_fit();
    }

    // TriangleMesh.cpp:808
    removed
}

/// TriangleMesh.cpp:811-839
/// C++: `int its_compactify_vertices(indexed_triangle_set &its, bool shrink_to_fit)`
pub fn its_compactify_vertices(its: &mut indexed_triangle_set, shrink_to_fit: bool) -> i32 {
    // TriangleMesh.cpp:814 — First used to mark referenced vertices, later used for mapping old vertex index to a new one.
    let mut vertex_map: Vec<i32> = vec![0; its.vertices.len()];
    // TriangleMesh.cpp:816-818 — Mark referenced vertices.
    for face in &its.indices {
        for i in 0..3 {
            vertex_map[face[i] as usize] = 1;
        }
    }
    // TriangleMesh.cpp:820-826 — Compactify vertices, update map from old vertex index to a new one.
    let mut last: i32 = 0;
    for i in 0..vertex_map.len() {
        if vertex_map[i] != 0 {
            // TriangleMesh.cpp:823-824
            if (last as usize) < i {
                its.vertices[last as usize] = its.vertices[i];
            }
            // TriangleMesh.cpp:825
            vertex_map[i] = last;
            last += 1;
        }
    }
    // TriangleMesh.cpp:827
    let removed = its.vertices.len() as i32 - last;
    // TriangleMesh.cpp:828-837
    if removed != 0 {
        // TriangleMesh.cpp:829
        its.vertices.truncate(last as usize);
        // TriangleMesh.cpp:831-833 — Update faces with the new vertex indices.
        for face in &mut its.indices {
            for i in 0..3 {
                face[i] = vertex_map[face[i] as usize];
            }
        }
        // TriangleMesh.cpp:835-836 — Optionally shrink the vertices.
        if shrink_to_fit {
            its.vertices.shrink_to_fit();
        }
    }
    // TriangleMesh.cpp:838
    removed
}

/// TriangleMesh.cpp:883-887
/// C++: `void its_shrink_to_fit(indexed_triangle_set &its)`
pub fn its_shrink_to_fit(its: &mut indexed_triangle_set) {
    // TriangleMesh.cpp:885-886
    its.indices.shrink_to_fit();
    its.vertices.shrink_to_fit();
}

// ============================================================================
// TriangleMesh.cpp:920-968 — Mesh projection points / 2D convex hull above plane
// ============================================================================

/// `Matrix3f * Vec3f` as the C++ build evaluates it (Eigen 5.0.1, clang -O3 `-ffp-contract=on`,
/// arm64 NEON): the lazy coefficient-based product (ProductEvaluators.h) over a 3-row float
/// destination takes the half packet `Packet2f` (XprHelper.h `find_best_packet`: 3 is not a
/// multiple of 4 nor of 2, the search stops at the smallest packet). Rows 0 and 1 are one packet,
/// `pmul` then two `pmadd` (vfma_f32): `fma(m2, z, fma(m1, y, m0 * x))`; row 2 is `coeff()`, the
/// unrolled redux `e0 + (e1 + e2)` (Redux.h `redux_novec_unroller`), unfused. The `f64`
/// counterpart is `geometry::eigen::mat3_mul_vec`.
pub fn eigen_mat3f_mul_vec(m: &nalgebra::Matrix3<f32>, p: &Vec3f) -> Vec3f {
    let packet = |i: usize| m[(i, 2)].mul_add(p.z, m[(i, 1)].mul_add(p.y, m[(i, 0)] * p.x));
    let coeff = |i: usize| m[(i, 0)] * p.x + (m[(i, 1)] * p.y + m[(i, 2)] * p.z);
    Vec3f::new(packet(0), packet(1), coeff(2))
}

/// `Transform3f * Vec3f` as the C++ build evaluates it: Transform.h:1329-1350 (Eigen 5.0.1)
/// multiplies the 4x4 matrix by `(x, y, z, 1)` into a 4-row float destination, one `Packet4f`:
/// `pmul` then three `pmadd` (vfmaq_f32), `fma(m3, 1, fma(m2, z, fma(m1, y, m0 * x)))`, of which
/// the first three rows are kept.
pub fn eigen_transform3f_mul_vec(t: &nalgebra::Matrix4<f32>, p: &Vec3f) -> Vec3f {
    let row = |i: usize| t[(i, 3)].mul_add(1.0, t[(i, 2)].mul_add(p.z, t[(i, 1)].mul_add(p.y, t[(i, 0)] * p.x)));
    Vec3f::new(row(0), row(1), row(2))
}

/// TriangleMesh.cpp:920-940, the template `its_collect_mesh_projection_points_above` over a
/// vertex transformation.
pub mod its_collect_mesh_projection_points_above_template {
    use super::*;
    /// TriangleMesh.cpp:920-940
    /// C++: `template<typename TransformVertex>
    ///       void its_collect_mesh_projection_points_above(const indexed_triangle_set &its, const TransformVertex &transform_fn, const float z, Points &all_pts)`
    pub fn its_collect_mesh_projection_points_above(
        its: &indexed_triangle_set,
        transform_fn: impl Fn(&Vec3f) -> Vec3f,
        z: f32,
        all_pts: &mut Vec<Point>,
    ) {
        // TriangleMesh.cpp:923
        all_pts.reserve(its.indices.len() * 3);
        // TriangleMesh.cpp:924-939
        for tri in &its.indices {
            // TriangleMesh.cpp:925
            let pts: [Vec3f; 3] = [
                transform_fn(&its.vertices[tri[0] as usize]),
                transform_fn(&its.vertices[tri[1] as usize]),
                transform_fn(&its.vertices[tri[2] as usize]),
            ];
            // TriangleMesh.cpp:926
            let mut iprev = 2usize;
            // TriangleMesh.cpp:927
            for iedge in 0..3usize {
                // TriangleMesh.cpp:928-929
                let p1 = &pts[iprev];
                let p2 = &pts[iedge];
                // TriangleMesh.cpp:930
                if (p1.z < z && p2.z > z) || (p2.z < z && p1.z > z) {
                    // TriangleMesh.cpp:931-932 — Edge crosses the z plane. Calculate intersection point with the plane.
                    let t = (z - p1.z) / (p2.z - p1.z);
                    // TriangleMesh.cpp:933: `p1.x() + (p2.x() - p1.x()) * t` (the product and the sum fuse)
                    all_pts.push(Point::new(
                        scaled_coord_f32((p2.x - p1.x).mul_add(t, p1.x)),
                        scaled_coord_f32((p2.y - p1.y).mul_add(t, p1.y)),
                    ));
                }
                // TriangleMesh.cpp:935-936
                if p2.z >= z {
                    all_pts.push(Point::new(scaled_coord_f32(p2.x), scaled_coord_f32(p2.y)));
                }
                // TriangleMesh.cpp:937
                iprev = iedge;
            }
        }
    }
}
use its_collect_mesh_projection_points_above_template::its_collect_mesh_projection_points_above as its_collect_mesh_projection_points_above_impl;

/// TriangleMesh.cpp:942-945
/// C++ matrix overload (`Matrix3f m`): `m * p`.
pub fn its_collect_mesh_projection_points_above(
    its: &indexed_triangle_set,
    m: &nalgebra::Matrix3<f32>,
    z: f32,
    all_pts: &mut Vec<Point>,
) {
    its_collect_mesh_projection_points_above_impl(its, |p| eigen_mat3f_mul_vec(m, p), z, all_pts);
}

/// TriangleMesh.cpp:947-950, the overload over a `Transform3f`.
pub mod its_collect_mesh_projection_points_above_transform {
    use super::*;
    /// TriangleMesh.cpp:947-950: `t * p`.
    pub fn its_collect_mesh_projection_points_above(
        its: &indexed_triangle_set,
        t: &nalgebra::Matrix4<f32>,
        z: f32,
        all_pts: &mut Vec<Point>,
    ) {
        its_collect_mesh_projection_points_above_impl(its, |p| eigen_transform3f_mul_vec(t, p), z, all_pts);
    }
}

/// TriangleMesh.cpp:952-958, the template `its_convex_hull_2d_above` over a vertex
/// transformation.
pub mod its_convex_hull_2d_above_template {
    use super::*;
    /// TriangleMesh.cpp:952-958
    /// C++: `template<typename TransformVertex>
    ///       Polygon its_convex_hull_2d_above(const indexed_triangle_set &its, const TransformVertex &transform_fn, const float z)`
    pub fn its_convex_hull_2d_above(its: &indexed_triangle_set, transform_fn: impl Fn(&Vec3f) -> Vec3f, z: f32) -> Polygon {
        // TriangleMesh.cpp:955
        let mut all_pts: Vec<Point> = Vec::new();
        // TriangleMesh.cpp:956
        its_collect_mesh_projection_points_above_impl(its, transform_fn, z, &mut all_pts);
        // TriangleMesh.cpp:957
        convex_hull_points(all_pts)
    }
}
use its_convex_hull_2d_above_template::its_convex_hull_2d_above as its_convex_hull_2d_above_impl;

/// TriangleMesh.cpp:960-963
/// C++ matrix overload (`Matrix3f m`).
pub fn its_convex_hull_2d_above(
    its: &indexed_triangle_set,
    m: &nalgebra::Matrix3<f32>,
    z: f32,
) -> Polygon {
    its_convex_hull_2d_above_impl(its, |p| eigen_mat3f_mul_vec(m, p), z)
}

/// TriangleMesh.cpp:965-968, the overload over a `Transform3f`.
pub mod its_convex_hull_2d_above_transform {
    use super::*;
    /// TriangleMesh.cpp:965-968
    pub fn its_convex_hull_2d_above(its: &indexed_triangle_set, t: &nalgebra::Matrix4<f32>, z: f32) -> Polygon {
        its_convex_hull_2d_above_impl(its, |p| eigen_transform3f_mul_vec(t, p), z)
    }
}

// ============================================================================
// TriangleMesh.cpp:939-1267 — ITS mesh generators
// ============================================================================

/// Helper to build an `indexed_triangle_set` from inline index/vertex literals
/// matching the C++ aggregate initialization `{ {indices...}, {vertices...} }`.
fn its_from(indices: Vec<[i32; 3]>, vertices: Vec<[f32; 3]>) -> indexed_triangle_set {
    indexed_triangle_set {
        indices: indices
            .into_iter()
            .map(|f| StlTriangleVertexIndices::new(f[0], f[1], f[2]))
            .collect(),
        vertices: vertices
            .into_iter()
            .map(|v| StlVertex::new(v[0], v[1], v[2]))
            .collect(),
    }
}

/// TriangleMesh.cpp:939-951
/// C++: `indexed_triangle_set its_make_xoy_center_rect(float width, float height, float depth)`
pub fn its_make_xoy_center_rect(width: f32, height: f32, depth: f32) -> indexed_triangle_set {
    // TriangleMesh.cpp:941
    let x = width / 2.0;
    let y = height / 2.0;
    let mut z = 0.0f32;
    // TriangleMesh.cpp:942
    if depth > 0.01 {
        // TriangleMesh.cpp:943
        z = depth / 2.0;
        // TriangleMesh.cpp:944-947
        its_from(
            vec![
                [0, 3, 2], [0, 2, 1], [4, 5, 6], [4, 6, 7], [0, 4, 7], [0, 7, 3], [7, 6, 2],
                [7, 2, 3], [2, 6, 5], [2, 5, 1], [1, 5, 4], [1, 4, 0],
            ],
            vec![
                [-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z], [-x, -y, z], [x, -y, z],
                [x, y, z], [-x, y, z],
            ],
        )
    } else {
        // TriangleMesh.cpp:949
        its_from(
            vec![[0, 1, 2], [0, 2, 3]],
            vec![[-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]],
        )
    }
}

/// Generate the vertex list for a cube solid of arbitrary size in X/Y/Z.
/// TriangleMesh.cpp:953-964
pub fn its_make_cube(xd: f64, yd: f64, zd: f64) -> indexed_triangle_set {
    // TriangleMesh.cpp:955
    let x = xd as f32;
    let y = yd as f32;
    let z = zd as f32;
    // TriangleMesh.cpp:957-963
    its_from(
        vec![
            [0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7], [0, 4, 7], [0, 7, 1], [1, 7, 6],
            [1, 6, 2], [2, 6, 5], [2, 5, 3], [4, 0, 3], [4, 3, 5],
        ],
        vec![
            [x, y, 0.0], [x, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, y, 0.0], [x, y, z], [0.0, y, z],
            [0.0, 0.0, z], [x, 0.0, z],
        ],
    )
}

/// TriangleMesh.cpp:966-983
/// C++: `indexed_triangle_set its_make_prism(float width, float length, float height)`
pub fn its_make_prism(width: f32, length: f32, height: f32) -> indexed_triangle_set {
    // TriangleMesh.cpp:969 — We need two upward facing triangles
    let x = width / 2.0;
    let y = length / 2.0;
    // TriangleMesh.cpp:970-982
    its_from(
        vec![
            [0, 1, 2], // side 1
            [4, 3, 5], // side 2
            [1, 4, 2],
            [2, 4, 5], // roof 1
            [0, 2, 5],
            [0, 5, 3], // roof 2
            [3, 4, 1],
            [3, 1, 0], // bottom
        ],
        vec![
            [-x, -y, 0.0], [x, -y, 0.0], [0.0, -y, height], [-x, y, 0.0], [x, y, 0.0],
            [0.0, y, height],
        ],
    )
}

/// Generate the mesh for a cylinder.
/// TriangleMesh.cpp:988-1029
pub fn its_make_cylinder(r: f64, h: f64, fa: f64) -> indexed_triangle_set {
    // TriangleMesh.cpp:990
    let mut mesh = indexed_triangle_set::default();
    // TriangleMesh.cpp:991
    let n_steps = (2.0 * std::f64::consts::PI / fa).ceil() as usize;
    // TriangleMesh.cpp:992
    let angle_step = 2.0 * std::f64::consts::PI / n_steps as f64;

    // TriangleMesh.cpp:996-997
    mesh.vertices.reserve(2 * n_steps + 2);
    mesh.indices.reserve(4 * n_steps);

    // TriangleMesh.cpp:1000-1001 — 2 special vertices, top and bottom center.
    mesh.vertices.push(Vec3f::new(0.0, 0.0, 0.0));
    mesh.vertices.push(Vec3f::new(0.0, 0.0, h as f32));

    // TriangleMesh.cpp:1007-1009 — Eigen::Rotation2Df(0) * Eigen::Vector2f(0, r) = (0, r).
    let mut p = rotate2d(0.0, Vec2f::new(0.0, r as f32));
    mesh.vertices.push(Vec3f::new(p[0], p[1], 0.0));
    mesh.vertices.push(Vec3f::new(p[0], p[1], h as f32));
    // TriangleMesh.cpp:1010-1019
    for i in 1..n_steps {
        // TriangleMesh.cpp:1011
        p = rotate2d((angle_step * i as f64) as f32, Vec2f::new(0.0, r as f32));
        // TriangleMesh.cpp:1012-1013
        mesh.vertices.push(Vec3f::new(p[0], p[1], 0.0));
        mesh.vertices.push(Vec3f::new(p[0], p[1], h as f32));
        // TriangleMesh.cpp:1014
        let id = mesh.vertices.len() as i32 - 1;
        // TriangleMesh.cpp:1015-1018
        mesh.indices.push(Vec3i::new(0, id - 1, id - 3)); // top
        mesh.indices.push(Vec3i::new(id, 1, id - 2)); // bottom
        mesh.indices.push(Vec3i::new(id, id - 2, id - 3)); // upper-right of side
        mesh.indices.push(Vec3i::new(id, id - 3, id - 1)); // bottom-left of side
    }
    // TriangleMesh.cpp:1021 — Connect the last set of vertices with the first.
    let id = mesh.vertices.len() as i32 - 1;
    // TriangleMesh.cpp:1022-1025
    mesh.indices.push(Vec3i::new(0, 2, id - 1));
    mesh.indices.push(Vec3i::new(3, 1, id));
    mesh.indices.push(Vec3i::new(id, 2, 3));
    mesh.indices.push(Vec3i::new(id, id - 1, 2));

    // TriangleMesh.cpp:1027
    mesh
}

/// Eigen `Rotation2Df(angle) * v` for a 2D float vector.
/// Eigen rotates by `[cos -sin; sin cos] * v`.
#[inline]
fn rotate2d(angle: f32, v: Vec2f) -> Vec2f {
    let c = angle.cos();
    let s = angle.sin();
    Vec2f::new(c * v[0] - s * v[1], s * v[0] + c * v[1])
}

/// CUSTOM GENERATOR: 3D THREAD
/// Generates a helical cosine thread for the Advanced Cut tool
/// TriangleMesh.cpp:1035-1109
pub fn its_make_thread(radius: f64, height: f64, pitch: f64, fa: f64) -> indexed_triangle_set {
    // TriangleMesh.cpp:1037
    let mut mesh = indexed_triangle_set::default();

    // TriangleMesh.cpp:1040 — 1. Calculate the resolution.
    let n_steps = (2.0 * std::f64::consts::PI / fa).ceil() as usize;
    // TriangleMesh.cpp:1041
    let angle_step = 2.0 * std::f64::consts::PI / n_steps as f64;

    // TriangleMesh.cpp:1044 — We need lots of vertical slices. 12 slices per pitch.
    let mut z_steps = ((height / pitch) * 12.0).ceil() as usize;
    // TriangleMesh.cpp:1045
    if z_steps < 2 {
        z_steps = 2;
    }
    // TriangleMesh.cpp:1046
    let z_step_size = height / z_steps as f64;

    // TriangleMesh.cpp:1048 — How deep the threads cut into the core.
    let thread_depth = pitch * 0.4;

    // TriangleMesh.cpp:1054-1069 — 2. Generate the Point Cloud (Vertices)
    for j in 0..=z_steps {
        // TriangleMesh.cpp:1055
        let z = j as f64 * z_step_size;
        // TriangleMesh.cpp:1056
        for i in 0..n_steps {
            // TriangleMesh.cpp:1057
            let angle = i as f64 * angle_step;
            // TriangleMesh.cpp:1060 — The magic spiral math.
            let phase = angle - ((z / pitch) * 2.0 * std::f64::consts::PI);
            // TriangleMesh.cpp:1063 — Calculate the bulging radius for this specific point.
            let r_current = radius - thread_depth + (thread_depth * 0.5 * (1.0 + phase.cos()));
            // TriangleMesh.cpp:1065-1066
            let x = r_current * angle.cos();
            let y = r_current * angle.sin();
            // TriangleMesh.cpp:1067
            mesh.vertices.push(Vec3f::new(x as f32, y as f32, z as f32));
        }
    }

    // TriangleMesh.cpp:1072-1086 — 3. Stitch the Wall Triangles (Facets)
    for j in 0..z_steps {
        for i in 0..n_steps {
            // TriangleMesh.cpp:1074
            let next_i = (i + 1) % n_steps;
            // TriangleMesh.cpp:1077-1080 — 4 corners of our current "square".
            let v0 = (j * n_steps + i) as i32; // Bottom-left
            let v1 = (j * n_steps + next_i) as i32; // Bottom-right
            let v2 = ((j + 1) * n_steps + i) as i32; // Top-left
            let v3 = ((j + 1) * n_steps + next_i) as i32; // Top-right
            // TriangleMesh.cpp:1083-1084 — Draw two triangles to fill the square.
            mesh.indices.push(Vec3i::new(v0, v1, v2));
            mesh.indices.push(Vec3i::new(v1, v3, v2));
        }
    }

    // TriangleMesh.cpp:1089 — 4. Create the Top and Bottom Center Vertices.
    let bottom_center_idx = mesh.vertices.len() as i32;
    mesh.vertices.push(Vec3f::new(0.0, 0.0, 0.0));

    // TriangleMesh.cpp:1092
    let top_center_idx = mesh.vertices.len() as i32;
    mesh.vertices.push(Vec3f::new(0.0, 0.0, height as f32));

    // TriangleMesh.cpp:1096-1106 — 5. Stitch the Top and Bottom Caps.
    for i in 0..n_steps {
        // TriangleMesh.cpp:1097
        let next_i = ((i + 1) % n_steps) as i32;
        // TriangleMesh.cpp:1100 — Bottom cap triangle.
        mesh.indices.push(Vec3i::new(bottom_center_idx, next_i, i as i32));
        // TriangleMesh.cpp:1103-1104 — Top cap triangle.
        let top_v0 = (z_steps * n_steps + i) as i32;
        let top_v1 = (z_steps * n_steps) as i32 + next_i;
        // TriangleMesh.cpp:1105
        mesh.indices.push(Vec3i::new(top_center_idx, top_v0, top_v1));
    }

    // TriangleMesh.cpp:1108
    mesh
}

/// TriangleMesh.cpp:1111-1135
/// C++: `indexed_triangle_set its_make_cone(double r, double h, double fa)`
pub fn its_make_cone(r: f64, h: f64, fa: f64) -> indexed_triangle_set {
    // TriangleMesh.cpp:1113
    let mut mesh = indexed_triangle_set::default();
    // TriangleMesh.cpp:1116
    mesh.vertices
        .reserve(3 + 2 * (2.0 * std::f64::consts::PI / fa) as usize);

    // TriangleMesh.cpp:1119-1120 — base center and top vertex.
    mesh.vertices.push(Vec3f::zeros());
    mesh.vertices.push(Vec3f::new(0.0, 0.0, h as f32));

    // TriangleMesh.cpp:1122
    let mut i: i32 = 0;
    // TriangleMesh.cpp:1123 — for (double angle=0; angle<2*PI; angle+=fa)
    let mut angle = 0.0f64;
    while angle < 2.0 * std::f64::consts::PI {
        // TriangleMesh.cpp:1124
        mesh.vertices.push(Vec3f::new(
            (r * angle.cos()) as f32,
            (r * angle.sin()) as f32,
            0.0,
        ));
        // TriangleMesh.cpp:1125-1128
        if angle > 0.0 {
            mesh.indices.push(Vec3i::new(0, i + 2, i + 1));
            mesh.indices.push(Vec3i::new(1, i + 1, i + 2));
        }
        // TriangleMesh.cpp:1129
        i += 1;
        angle += fa;
    }
    // TriangleMesh.cpp:1131-1132 — close the shape.
    mesh.indices.push(Vec3i::new(0, 2, i + 1));
    mesh.indices.push(Vec3i::new(1, i + 1, 2));

    // TriangleMesh.cpp:1134
    mesh
}

/// The gate of the float arithmetic of the sphere-like mesh makers (faithful, on unless
/// `ITS_MAKE_FLOAT_STEP=0`). On: `sectorStep * j` and `stackStep * i` are float products (the steps
/// are floats, the indices ints), and its_make_snap calls the float overloads of libc++'s
/// `<math.h>` (`acos`, `sin` of a float are acosf, sinf) with the sum of get_m_len fused, as the C++
/// build compiles TriangleMesh.cpp. Off: the earlier double products and double functions.
fn its_make_float_step() -> bool {
    crate::faithful_gate("ITS_MAKE_FLOAT_STEP")
}

/// `float * int` of a step and an index, promoted to double (TriangleMesh.cpp 1181, 1189, 1258,
/// 1266): a float product on the gate's faithful arm.
#[inline]
fn step_times(step: f32, i: i32) -> f64 {
    if its_make_float_step() {
        (step * i as f32) as f64
    } else {
        step as f64 * i as f64
    }
}

/// Generates mesh for a frustum dowel centered about the origin, using the count of sectors.
/// TriangleMesh.cpp:1139-1190
pub fn its_make_frustum_dowel(radius: f64, h: f64, sector_count: i32) -> indexed_triangle_set {
    // TriangleMesh.cpp:1141
    let stack_count: i32 = 2;
    // TriangleMesh.cpp:1142
    let sector_step = (2.0 * std::f64::consts::PI / sector_count as f64) as f32;
    // TriangleMesh.cpp:1143
    let stack_step = (std::f64::consts::PI / stack_count as f64) as f32;

    // TriangleMesh.cpp:1145
    let mut mesh = indexed_triangle_set::default();
    // TriangleMesh.cpp:1147
    mesh.vertices
        .reserve(((stack_count - 1) * sector_count + 2) as usize);
    // TriangleMesh.cpp:1148-1161
    for i in 0..=stack_count {
        // TriangleMesh.cpp:1181 — from pi/2 to -pi/2.
        let stack_angle = 0.5 * std::f64::consts::PI - step_times(stack_step, i);
        // TriangleMesh.cpp:1151
        let xy = radius * stack_angle.cos();
        // TriangleMesh.cpp:1152
        let z = radius * stack_angle.sin();
        // TriangleMesh.cpp:1153-1154
        if i == 0 || i == stack_count {
            mesh.vertices.push(Vec3f::new(
                xy as f32,
                0.0,
                (h * stack_angle.sin()) as f32,
            ));
        } else {
            // TriangleMesh.cpp:1156-1160
            for j in 0..sector_count {
                // TriangleMesh.cpp:1189 — from 0 to 2pi.
                let sector_angle = step_times(sector_step, j) + 0.25 * std::f64::consts::PI;
                // TriangleMesh.cpp:1159 — Vec3d(...).cast<float>()
                mesh.vertices.push(Vec3f::new(
                    (xy * sector_angle.cos()) as f32,
                    (xy * sector_angle.sin()) as f32,
                    z as f32,
                ));
            }
        }
    }

    // TriangleMesh.cpp:1164
    mesh.indices
        .reserve((2 * (stack_count - 1) * sector_count) as usize);
    // TriangleMesh.cpp:1165-1187
    for i in 0..stack_count {
        // TriangleMesh.cpp:1167 — Beginning of current stack.
        let mut k1 = if i == 0 { 0 } else { 1 + (i - 1) * sector_count };
        let k1_first = k1;
        // TriangleMesh.cpp:1170 — Beginning of next stack.
        let mut k2 = if i == 0 { 1 } else { k1 + sector_count };
        let k2_first = k2;
        // TriangleMesh.cpp:1172-1186
        for j in 0..sector_count {
            // TriangleMesh.cpp:1174-1175
            let mut k1_next = k1;
            let mut k2_next = k2;
            // TriangleMesh.cpp:1176-1179
            if i != 0 {
                k1_next = if j + 1 == sector_count { k1_first } else { k1 + 1 };
                mesh.indices.push(Vec3i::new(k1, k2, k1_next));
            }
            // TriangleMesh.cpp:1180-1183
            if i + 1 != stack_count {
                k2_next = if j + 1 == sector_count { k2_first } else { k2 + 1 };
                mesh.indices.push(Vec3i::new(k1_next, k2, k2_next));
            }
            // TriangleMesh.cpp:1184-1185
            k1 = k1_next;
            k2 = k2_next;
        }
    }

    // TriangleMesh.cpp:1189
    mesh
}

/// TriangleMesh.cpp:1192-1209
/// C++: `indexed_triangle_set its_make_pyramid(float base, float height)`
pub fn its_make_pyramid(base: f32, height: f32) -> indexed_triangle_set {
    // TriangleMesh.cpp:1194
    let a = base / 2.0;
    // TriangleMesh.cpp:1195-1208
    its_from(
        vec![
            [0, 1, 2], [0, 2, 3], [0, 1, 4], [1, 2, 4], [2, 3, 4], [3, 0, 4],
        ],
        vec![
            [-a, -a, 0.0], [a, -a, 0.0], [a, a, 0.0], [-a, a, 0.0], [0.0, 0.0, height],
        ],
    )
}

/// Generates mesh for a sphere centered about the origin.
/// TriangleMesh.cpp:1215-1267
pub fn its_make_sphere(radius: f64, fa: f64) -> indexed_triangle_set {
    // TriangleMesh.cpp:1217
    let sector_count = (2.0 * std::f64::consts::PI / fa).ceil() as i32;
    // TriangleMesh.cpp:1218
    let stack_count = (std::f64::consts::PI / fa).ceil() as i32;
    // TriangleMesh.cpp:1219
    let sector_step = (2.0 * std::f64::consts::PI / sector_count as f64) as f32;
    // TriangleMesh.cpp:1220
    let stack_step = (std::f64::consts::PI / stack_count as f64) as f32;

    // TriangleMesh.cpp:1222
    let mut mesh = indexed_triangle_set::default();
    // TriangleMesh.cpp:1224
    mesh.vertices
        .reserve(((stack_count - 1) * sector_count + 2) as usize);
    // TriangleMesh.cpp:1225-1238
    for i in 0..=stack_count {
        // TriangleMesh.cpp:1258 — from pi/2 to -pi/2.
        let stack_angle = 0.5 * std::f64::consts::PI - step_times(stack_step, i);
        // TriangleMesh.cpp:1228
        let xy = radius * stack_angle.cos();
        // TriangleMesh.cpp:1229
        let z = radius * stack_angle.sin();
        // TriangleMesh.cpp:1230-1231
        if i == 0 || i == stack_count {
            mesh.vertices.push(Vec3f::new(xy as f32, 0.0, z as f32));
        } else {
            // TriangleMesh.cpp:1233-1237
            for j in 0..sector_count {
                // TriangleMesh.cpp:1266 — from 0 to 2pi.
                let sector_angle = step_times(sector_step, j);
                // TriangleMesh.cpp:1236 — Vec3d(...).cast<float>()
                mesh.vertices.push(Vec3f::new(
                    (xy * sector_angle.cos()) as f32,
                    (xy * sector_angle.sin()) as f32,
                    z as f32,
                ));
            }
        }
    }

    // TriangleMesh.cpp:1241
    mesh.indices
        .reserve((2 * (stack_count - 1) * sector_count) as usize);
    // TriangleMesh.cpp:1242-1264
    for i in 0..stack_count {
        // TriangleMesh.cpp:1244 — Beginning of current stack.
        let mut k1 = if i == 0 { 0 } else { 1 + (i - 1) * sector_count };
        let k1_first = k1;
        // TriangleMesh.cpp:1247 — Beginning of next stack.
        let mut k2 = if i == 0 { 1 } else { k1 + sector_count };
        let k2_first = k2;
        // TriangleMesh.cpp:1249-1263
        for j in 0..sector_count {
            // TriangleMesh.cpp:1251-1252
            let mut k1_next = k1;
            let mut k2_next = k2;
            // TriangleMesh.cpp:1253-1256
            if i != 0 {
                k1_next = if j + 1 == sector_count { k1_first } else { k1 + 1 };
                mesh.indices.push(Vec3i::new(k1, k2, k1_next));
            }
            // TriangleMesh.cpp:1257-1260
            if i + 1 != stack_count {
                k2_next = if j + 1 == sector_count { k2_first } else { k2 + 1 };
                mesh.indices.push(Vec3i::new(k1_next, k2, k2_next));
            }
            // TriangleMesh.cpp:1261-1262
            k1 = k1_next;
            k2 = k2_next;
        }
    }

    // TriangleMesh.cpp:1266
    mesh
}

/// TriangleMesh.cpp:1269-1385
/// C++: `indexed_triangle_set its_make_snap(double r, double h, float space_proportion, float bulge_proportion)`
pub fn its_make_snap(
    r: f64,
    h: f64,
    space_proportion: f32,
    bulge_proportion: f32,
) -> indexed_triangle_set {
    // TriangleMesh.cpp:1271
    let radius = r as f32;
    // TriangleMesh.cpp:1272
    let height = h as f32;
    // TriangleMesh.cpp:1273 — (float)fa;
    let sectors_cnt: usize = 10;
    // TriangleMesh.cpp:1274 — 0.5f * (float) PI
    let half_pi = 0.5f32 * std::f64::consts::PI as f32;

    // TriangleMesh.cpp:1276
    let space_len = space_proportion * radius;

    // TriangleMesh.cpp:1278-1280
    let b_len = radius;
    let m_len = (1.0 + bulge_proportion) * radius;
    let t_len = 0.5f32 * radius;

    // TriangleMesh.cpp:1282-1284
    let b_height = 0.0f32;
    let m_height = 0.5f32 * height;
    let t_height = height;

    // TriangleMesh.cpp:1286-1287 — unqualified acos(double) on a promoted float arg,
    // result narrowed back to float.
    // (TriangleMesh.cpp:1317-1318; the faithful arm: libc++'s <math.h> float overload, acosf)
    let float_math = its_make_float_step();
    let b_angle = if float_math { (space_len / b_len).acos() } else { ((space_len / b_len) as f64).acos() as f32 };
    let t_angle = if float_math { (space_len / t_len).acos() } else { ((space_len / t_len) as f64).acos() as f32 };

    // TriangleMesh.cpp:1289-1290
    let b_angle_step = b_angle / sectors_cnt as f32;
    let t_angle_step = t_angle / sectors_cnt as f32;

    // TriangleMesh.cpp:1292-1293
    let b_vec = Vec2f::new(0.0, b_len);
    let t_vec = Vec2f::new(0.0, t_len);

    // TriangleMesh.cpp:1295-1303 — add_side_vertices lambda.
    let add_side_vertices =
        move |vertices: &mut Vec<StlVertex>, b_angle: f32, t_angle: f32, m_vec: &Vec2f| {
            // TriangleMesh.cpp:1296-1298 — Eigen::Rotation2Df(angle) * vec
            let b_pt = rotate2d(b_angle, b_vec);
            let m_pt = rotate2d(b_angle, *m_vec);
            let t_pt = rotate2d(t_angle, t_vec);

            // TriangleMesh.cpp:1300-1302
            vertices.push(Vec3f::new(b_pt[0], b_pt[1], b_height));
            vertices.push(Vec3f::new(m_pt[0], m_pt[1], m_height));
            vertices.push(Vec3f::new(t_pt[0], t_pt[1], t_height));
        };

    // TriangleMesh.cpp:1305-1316 — add_side_facets lambda.
    let add_side_facets =
        |facets: &mut Vec<StlTriangleVertexIndices>, vertices_cnt: i32, frst_id: i32, scnd_id: i32| {
            // TriangleMesh.cpp:1306
            let id = vertices_cnt - 1;

            // TriangleMesh.cpp:1308
            facets.push(Vec3i::new(frst_id, id - 2, id - 5));

            // TriangleMesh.cpp:1310-1313
            facets.push(Vec3i::new(id - 2, id - 1, id - 5));
            facets.push(Vec3i::new(id - 1, id - 4, id - 5));
            facets.push(Vec3i::new(id - 4, id - 1, id));
            facets.push(Vec3i::new(id, id - 3, id - 4));

            // TriangleMesh.cpp:1315
            facets.push(Vec3i::new(id, scnd_id, id - 3));
        };

    // TriangleMesh.cpp:1318 — Flattening.
    let f = (b_len - m_len) / m_len;

    // TriangleMesh.cpp:1320-1325 — get_m_len lambda.
    let get_m_len = move |angle: f32| -> f32 {
        // TriangleMesh.cpp:1321
        let rad_sqr: f32 = b_len * b_len;
        if float_math {
            // TriangleMesh.cpp:1353 — sin(float) is sinf; the product of floats
            let sin_sqr: f32 = angle.sin() * angle.sin();
            // TriangleMesh.cpp:1354
            let f_sqr: f32 = (1.0 - f) * (1.0 - f);
            // TriangleMesh.cpp:1355 — sqrtf, `1 + (1 / f_sqr - 1) * sin_sqr` fused
            return (rad_sqr / (1.0 / f_sqr - 1.0).mul_add(sin_sqr, 1.0)).sqrt();
        }
        // TriangleMesh.cpp:1322 — sin(float) -> double sin, product narrowed to float.
        let sin_d = (angle as f64).sin();
        let sin_sqr: f32 = (sin_d * sin_d) as f32;
        // TriangleMesh.cpp:1323
        let f_sqr: f32 = (1.0 - f) * (1.0 - f);
        // TriangleMesh.cpp:1324 — sqrtf (float)
        (rad_sqr / (1.0 + (1.0 / f_sqr - 1.0) * sin_sqr)).sqrt()
    };

    // TriangleMesh.cpp:1327-1374 — add_sub_mesh lambda.
    let add_sub_mesh =
        |mesh: &mut indexed_triangle_set, center_x: f32, angle_rotation: f32, frst_vertex_id: i32| {
            // TriangleMesh.cpp:1333-1334 — 2 special vertices, top and bottom center.
            mesh.vertices.push(Vec3f::new(center_x, 0.0, b_height));
            mesh.vertices.push(Vec3f::new(center_x, 0.0, t_height));

            // TriangleMesh.cpp:1336-1338
            let mut b_angle_start = angle_rotation - b_angle;
            let mut t_angle_start = angle_rotation - t_angle;
            let b_angle_stop = angle_rotation + b_angle;

            // TriangleMesh.cpp:1340-1341
            let frst_id = frst_vertex_id;
            let scnd_id = frst_id + 1;

            // TriangleMesh.cpp:1343-1353 — add first side vertices and internal facets.
            {
                // TriangleMesh.cpp:1345
                let m_vec = Vec2f::new(0.0, get_m_len(b_angle_start));
                // TriangleMesh.cpp:1346
                add_side_vertices(&mut mesh.vertices, b_angle_start, t_angle_start, &m_vec);

                // TriangleMesh.cpp:1348
                let id = mesh.vertices.len() as i32 - 1;

                // TriangleMesh.cpp:1350-1352
                mesh.indices.push(Vec3i::new(frst_id, id - 2, id - 1));
                mesh.indices.push(Vec3i::new(frst_id, id - 1, id));
                mesh.indices.push(Vec3i::new(frst_id, id, scnd_id));
            }

            // TriangleMesh.cpp:1356-1364 — add d side vertices and facets.
            // C++: while (!is_approx(b_angle_start, b_angle_stop)) — is_approx<float>
            // promotes both to double and compares with EPSILON (1e-4).
            // is_approx<float>: the precision is EPSILON converted to float
            let precision = if float_math { (crate::libslic3r::EPSILON as f32) as f64 } else { crate::libslic3r::EPSILON };
            while (b_angle_start as f64 - b_angle_stop as f64).abs() >= precision {
                // TriangleMesh.cpp:1357-1358
                b_angle_start += b_angle_step;
                t_angle_start += t_angle_step;

                // TriangleMesh.cpp:1360
                let m_vec = Vec2f::new(0.0, get_m_len(b_angle_start));
                // TriangleMesh.cpp:1361
                add_side_vertices(&mut mesh.vertices, b_angle_start, t_angle_start, &m_vec);

                // TriangleMesh.cpp:1363
                let vertices_cnt = mesh.vertices.len() as i32;
                add_side_facets(&mut mesh.indices, vertices_cnt, frst_id, scnd_id);
            }

            // TriangleMesh.cpp:1366-1373 — add last internal facets to close the mesh.
            {
                // TriangleMesh.cpp:1368
                let id = mesh.vertices.len() as i32 - 1;

                // TriangleMesh.cpp:1370-1372
                mesh.indices.push(Vec3i::new(frst_id, scnd_id, id));
                mesh.indices.push(Vec3i::new(frst_id, id, id - 1));
                mesh.indices.push(Vec3i::new(frst_id, id - 1, id - 2));
            }
        };

    // TriangleMesh.cpp:1376
    let mut mesh = indexed_triangle_set::default();

    // TriangleMesh.cpp:1378-1379
    mesh.vertices
        .reserve(2 * (3 * (2 * sectors_cnt + 1) + 2));
    mesh.indices.reserve(2 * (6 * 2 * sectors_cnt + 6));

    // TriangleMesh.cpp:1381
    add_sub_mesh(&mut mesh, -space_len, half_pi, 0);
    // TriangleMesh.cpp:1382
    let frst = mesh.vertices.len() as i32;
    add_sub_mesh(&mut mesh, space_len, 3.0 * half_pi, frst);

    // TriangleMesh.cpp:1384
    mesh
}

/// CutUtils.hpp:63 `const float CUT_TOLERANCE = 0.1f;`
pub const CUT_TOLERANCE: f32 = 0.1;

/// CutUtils.hpp:64-76 `struct Groove`, which TriangleMesh.hpp:18 declares and
/// `its_make_groove_plane` reads. (cut_utils.rs still carries a field-less placeholder of the
/// name; this is the struct of the pin.)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Groove {
    /// CutUtils.hpp:66
    pub depth: f32,
    /// CutUtils.hpp:67
    pub width: f32,
    /// CutUtils.hpp:68
    pub flaps_angle: f32,
    /// CutUtils.hpp:69
    pub angle: f32,
    /// CutUtils.hpp:70
    pub depth_init: f32,
    /// CutUtils.hpp:71
    pub width_init: f32,
    /// CutUtils.hpp:72
    pub flaps_angle_init: f32,
    /// CutUtils.hpp:73
    pub angle_init: f32,
    /// CutUtils.hpp:74
    pub depth_tolerance: f32,
    /// CutUtils.hpp:75
    pub width_tolerance: f32,
}

impl Default for Groove {
    fn default() -> Self {
        Self {
            depth: 0.0,
            width: 0.0,
            flaps_angle: 0.0,
            angle: 0.0,
            depth_init: 0.0,
            width_init: 0.0,
            flaps_angle_init: 0.0,
            angle_init: 0.0,
            depth_tolerance: CUT_TOLERANCE,
            width_tolerance: CUT_TOLERANCE,
        }
    }
}

/// TriangleMesh.cpp:1418-1734
/// C++: `indexed_triangle_set its_make_groove_plane(const Groove &cur_groove, float rotate_radius, std::vector<Vec3d> &cur_groove_vertices)`
///
/// `sin`, `cos` and `tan` of a `float` are the `float` overloads (`sinf`, `cosf`, `tanf`).
pub fn its_make_groove_plane(cur_groove: &Groove, rotate_radius: f32, cur_groove_vertices: &mut Vec<crate::geometry::Vec3d>) -> indexed_triangle_set {
    // TriangleMesh.cpp:1420 — is_approx<float>(flaps_angle, 0.f): |double(a) - double(b)| < double(float(EPSILON))
    let side_width: f32 = if ((cur_groove.flaps_angle as f64) - 0.0).abs() < (crate::libslic3r::EPSILON as f32) as f64 {
        cur_groove.depth
    } else {
        cur_groove.depth / cur_groove.flaps_angle.sin()
    };
    // TriangleMesh.cpp:1421
    let flaps_width: f32 = 2.0f32 * side_width * cur_groove.flaps_angle.cos();

    // TriangleMesh.cpp:1423-1424
    let groove_half_width_upper: f32 = 0.5f32 * cur_groove.width;
    let groove_half_width_lower: f32 = 0.5f32 * (cur_groove.width + flaps_width);

    // TriangleMesh.cpp:1426-1427
    let cut_plane_radius: f32 = 1.5f32 * rotate_radius;
    let cut_plane_length: f32 = 1.5f32 * cut_plane_radius;

    // TriangleMesh.cpp:1429
    let groove_half_depth: f32 = 0.5f32 * cur_groove.depth;

    // TriangleMesh.cpp:1431-1434
    let x: f32 = 0.5f32 * cut_plane_radius;
    let y: f32 = 0.5f32 * cut_plane_length;
    let mut z_upper: f32 = groove_half_depth;
    let mut z_lower: f32 = -groove_half_depth;

    // TriangleMesh.cpp:1436
    let proj: f32 = y * cur_groove.angle.tan();

    // TriangleMesh.cpp:1438-1442
    let mut ext_upper_x: f32 = groove_half_width_upper + proj; // upper_x extension
    let mut ext_lower_x: f32 = groove_half_width_lower + proj; // lower_x extension

    let mut nar_upper_x: f32 = groove_half_width_upper - proj; // upper_x narrowing
    let mut nar_lower_x: f32 = groove_half_width_lower - proj; // lower_x narrowing

    // TriangleMesh.cpp:1444
    let cut_plane_thiknes: f32 = 0.02f32;

    // TriangleMesh.cpp:1446-1462 — Vertices of the groove used to detection if groove is valid
    {
        let d = |v: Vec3f| crate::geometry::Vec3d::new(v.x as f64, v.y as f64, v.z as f64);
        cur_groove_vertices.clear();
        cur_groove_vertices.reserve(8);

        cur_groove_vertices.push(d(Vec3f::new(-ext_lower_x, -y, z_lower)));
        cur_groove_vertices.push(d(Vec3f::new(-nar_lower_x, y, z_lower)));
        cur_groove_vertices.push(d(Vec3f::new(-ext_upper_x, -y, z_upper)));
        cur_groove_vertices.push(d(Vec3f::new(-nar_upper_x, y, z_upper)));
        cur_groove_vertices.push(d(Vec3f::new(ext_lower_x, -y, z_lower)));
        cur_groove_vertices.push(d(Vec3f::new(nar_lower_x, y, z_lower)));
        cur_groove_vertices.push(d(Vec3f::new(ext_upper_x, -y, z_upper)));
        cur_groove_vertices.push(d(Vec3f::new(nar_upper_x, y, z_upper)));
    }
    // TriangleMesh.cpp:1463-1465 — Different cases of groove plane: groove is open
    if groove_half_width_upper > proj && groove_half_width_lower > proj {
        let mut mesh = indexed_triangle_set::default();

        // TriangleMesh.cpp:1468-1483
        let get_vertices = |z_upper: f32, z_lower: f32, nar_upper_x: f32, nar_lower_x: f32, ext_upper_x: f32, ext_lower_x: f32| -> Vec<StlVertex> {
            vec![
                // upper left part vertices
                Vec3f::new(-x, -y, z_upper),
                Vec3f::new(-x, y, z_upper),
                Vec3f::new(-nar_upper_x, y, z_upper),
                Vec3f::new(-ext_upper_x, -y, z_upper),
                // lower part vertices
                Vec3f::new(-ext_lower_x, -y, z_lower),
                Vec3f::new(-nar_lower_x, y, z_lower),
                Vec3f::new(nar_lower_x, y, z_lower),
                Vec3f::new(ext_lower_x, -y, z_lower),
                // upper right part vertices
                Vec3f::new(ext_upper_x, -y, z_upper),
                Vec3f::new(nar_upper_x, y, z_upper),
                Vec3f::new(x, y, z_upper),
                Vec3f::new(x, -y, z_upper),
            ]
        };

        // TriangleMesh.cpp:1485-1486
        mesh.vertices = get_vertices(z_upper, z_lower, nar_upper_x, nar_lower_x, ext_upper_x, ext_lower_x);
        mesh.vertices.reserve(mesh.vertices.len());

        // TriangleMesh.cpp:1488-1489
        z_upper -= cut_plane_thiknes;
        z_lower -= cut_plane_thiknes;

        // TriangleMesh.cpp:1491
        let under_x_shift: f32 = cut_plane_thiknes / (0.5f32 * cur_groove.flaps_angle).tan();

        // TriangleMesh.cpp:1493-1496
        nar_upper_x += under_x_shift;
        nar_lower_x += under_x_shift;
        ext_upper_x += under_x_shift;
        ext_lower_x += under_x_shift;

        // TriangleMesh.cpp:1498-1499
        let vertices = get_vertices(z_upper, z_lower, nar_upper_x, nar_lower_x, ext_upper_x, ext_lower_x);
        mesh.vertices.extend_from_slice(&vertices);

        // TriangleMesh.cpp:1501-1549
        mesh.indices = vec![
        Vec3i::new(5, 4, 7), Vec3i::new(5, 7, 6), Vec3i::new(3, 4, 5), Vec3i::new(3, 5, 2),
        Vec3i::new(9, 6, 8), Vec3i::new(8, 6, 7), Vec3i::new(1, 0, 2), Vec3i::new(2, 0, 3),
        Vec3i::new(9, 8, 10), Vec3i::new(10, 8, 11), Vec3i::new(20, 21, 22), Vec3i::new(20, 22, 23),
        Vec3i::new(12, 13, 14), Vec3i::new(12, 14, 15), Vec3i::new(18, 21, 20), Vec3i::new(18, 20, 19),
        Vec3i::new(16, 15, 14), Vec3i::new(16, 14, 17), Vec3i::new(16, 17, 18), Vec3i::new(16, 18, 19),
        Vec3i::new(1, 13, 0), Vec3i::new(0, 13, 12), Vec3i::new(0, 12, 3), Vec3i::new(3, 12, 15),
        Vec3i::new(3, 15, 4), Vec3i::new(4, 15, 16), Vec3i::new(4, 16, 7), Vec3i::new(7, 16, 19),
        Vec3i::new(7, 19, 20), Vec3i::new(7, 20, 8), Vec3i::new(8, 20, 11), Vec3i::new(11, 20, 23),
        Vec3i::new(11, 23, 10), Vec3i::new(10, 23, 22), Vec3i::new(1, 13, 2), Vec3i::new(2, 13, 14),
        Vec3i::new(2, 14, 17), Vec3i::new(2, 17, 5), Vec3i::new(5, 17, 6), Vec3i::new(6, 17, 18),
        Vec3i::new(6, 18, 9), Vec3i::new(9, 18, 21), Vec3i::new(9, 21, 10), Vec3i::new(10, 21, 22),
        ];
        return mesh;
    }

    // TriangleMesh.cpp:1553
    let mut cross_pt_upper_y: f32 = groove_half_width_upper / cur_groove.angle.tan();
    // TriangleMesh.cpp:1554-1556 — groove is closed
    if groove_half_width_upper < proj && groove_half_width_lower < proj {
        // TriangleMesh.cpp:1557
        let mut cross_pt_lower_y: f32 = groove_half_width_lower / cur_groove.angle.tan();

        let mut mesh = indexed_triangle_set::default();

        // TriangleMesh.cpp:1561-1574
        let get_vertices = |z_upper: f32, z_lower: f32, cross_pt_upper_y: f32, cross_pt_lower_y: f32, ext_upper_x: f32, ext_lower_x: f32| -> Vec<StlVertex> {
            vec![
                // upper part vertices
                Vec3f::new(-x, -y, z_upper),
                Vec3f::new(-x, y, z_upper),
                Vec3f::new(x, y, z_upper),
                Vec3f::new(x, -y, z_upper),
                Vec3f::new(ext_upper_x, -y, z_upper),
                Vec3f::new(0.0f32, cross_pt_upper_y, z_upper),
                Vec3f::new(-ext_upper_x, -y, z_upper),
                // lower part vertices
                Vec3f::new(-ext_lower_x, -y, z_lower),
                Vec3f::new(0.0f32, cross_pt_lower_y, z_lower),
                Vec3f::new(ext_lower_x, -y, z_lower),
            ]
        };

        // TriangleMesh.cpp:1576-1577
        mesh.vertices = get_vertices(z_upper, z_lower, cross_pt_upper_y, cross_pt_lower_y, ext_upper_x, ext_lower_x);
        mesh.vertices.reserve(mesh.vertices.len());

        // TriangleMesh.cpp:1579-1580
        z_upper -= cut_plane_thiknes;
        z_lower -= cut_plane_thiknes;

        // TriangleMesh.cpp:1582
        let under_x_shift: f32 = cut_plane_thiknes / (0.5f32 * cur_groove.flaps_angle).tan();

        // TriangleMesh.cpp:1584-1587
        cross_pt_upper_y += cut_plane_thiknes;
        cross_pt_lower_y += cut_plane_thiknes;
        ext_upper_x += under_x_shift;
        ext_lower_x += under_x_shift;

        // TriangleMesh.cpp:1589-1590
        let vertices = get_vertices(z_upper, z_lower, cross_pt_upper_y, cross_pt_lower_y, ext_upper_x, ext_lower_x);
        mesh.vertices.extend_from_slice(&vertices);

        // TriangleMesh.cpp:1592-1634
        mesh.indices = vec![
        Vec3i::new(8, 7, 9), Vec3i::new(5, 8, 6), Vec3i::new(6, 8, 7), Vec3i::new(4, 9, 8),
        Vec3i::new(4, 8, 5), Vec3i::new(1, 0, 6), Vec3i::new(1, 6, 5), Vec3i::new(1, 5, 2),
        Vec3i::new(2, 5, 4), Vec3i::new(2, 4, 3), Vec3i::new(10, 11, 16), Vec3i::new(16, 11, 15),
        Vec3i::new(15, 11, 12), Vec3i::new(15, 12, 14), Vec3i::new(14, 12, 13), Vec3i::new(18, 15, 14),
        Vec3i::new(14, 18, 19), Vec3i::new(17, 16, 15), Vec3i::new(17, 15, 18), Vec3i::new(17, 18, 19),
        Vec3i::new(1, 11, 0), Vec3i::new(0, 11, 10), Vec3i::new(0, 10, 6), Vec3i::new(6, 10, 16),
        Vec3i::new(6, 17, 16), Vec3i::new(6, 7, 17), Vec3i::new(7, 17, 19), Vec3i::new(7, 19, 9),
        Vec3i::new(4, 14, 19), Vec3i::new(4, 19, 9), Vec3i::new(4, 14, 13), Vec3i::new(4, 13, 3),
        Vec3i::new(3, 13, 12), Vec3i::new(3, 12, 2), Vec3i::new(2, 12, 11), Vec3i::new(2, 11, 1),
        ];

        return mesh;
    }

    // TriangleMesh.cpp:1639-1653 — groove is closed from the roof
    let mut mesh = indexed_triangle_set::default();
    mesh.vertices = vec![
        // upper part vertices
        Vec3f::new(-x, -y, z_upper),
        Vec3f::new(-x, y, z_upper),
        Vec3f::new(x, y, z_upper),
        Vec3f::new(x, -y, z_upper),
        Vec3f::new(ext_upper_x, -y, z_upper),
        Vec3f::new(0.0f32, cross_pt_upper_y, z_upper),
        Vec3f::new(-ext_upper_x, -y, z_upper),
        // lower part vertices
        Vec3f::new(-ext_lower_x, -y, z_lower),
        Vec3f::new(-nar_lower_x, y, z_lower),
        Vec3f::new(nar_lower_x, y, z_lower),
        Vec3f::new(ext_lower_x, -y, z_lower),
    ];

    // TriangleMesh.cpp:1655
    mesh.vertices.reserve(mesh.vertices.len() + 1);

    // TriangleMesh.cpp:1657-1658
    z_upper -= cut_plane_thiknes;
    z_lower -= cut_plane_thiknes;

    // TriangleMesh.cpp:1660
    let under_x_shift: f32 = cut_plane_thiknes / (0.5f32 * cur_groove.flaps_angle).tan();

    // TriangleMesh.cpp:1662-1664
    nar_lower_x += under_x_shift;
    ext_upper_x += under_x_shift;
    ext_lower_x += under_x_shift;

    // TriangleMesh.cpp:1666-1679
    let vertices: Vec<StlVertex> = vec![
        // upper part vertices
        Vec3f::new(-x, -y, z_upper),
        Vec3f::new(-x, y, z_upper),
        Vec3f::new(x, y, z_upper),
        Vec3f::new(x, -y, z_upper),
        Vec3f::new(ext_upper_x, -y, z_upper),
        Vec3f::new(under_x_shift, cross_pt_upper_y, z_upper),
        Vec3f::new(-under_x_shift, cross_pt_upper_y, z_upper),
        Vec3f::new(-ext_upper_x, -y, z_upper),
        // lower part vertices
        Vec3f::new(-ext_lower_x, -y, z_lower),
        Vec3f::new(-nar_lower_x, y, z_lower),
        Vec3f::new(nar_lower_x, y, z_lower),
        Vec3f::new(ext_lower_x, -y, z_lower),
    ];
    // TriangleMesh.cpp:1680
    mesh.vertices.extend_from_slice(&vertices);
    // TriangleMesh.cpp:1681-1731
    mesh.indices = vec![
        Vec3i::new(8, 7, 10), Vec3i::new(8, 10, 9), Vec3i::new(5, 8, 7), Vec3i::new(5, 7, 6),
        Vec3i::new(4, 10, 9), Vec3i::new(4, 9, 5), Vec3i::new(1, 0, 6), Vec3i::new(1, 6, 5),
        Vec3i::new(1, 5, 2), Vec3i::new(2, 5, 4), Vec3i::new(2, 4, 3), Vec3i::new(11, 12, 18),
        Vec3i::new(18, 12, 17), Vec3i::new(17, 12, 16), Vec3i::new(16, 12, 13), Vec3i::new(16, 13, 15),
        Vec3i::new(15, 13, 14), Vec3i::new(21, 16, 15), Vec3i::new(21, 15, 22), Vec3i::new(19, 18, 17),
        Vec3i::new(19, 17, 20), Vec3i::new(19, 20, 21), Vec3i::new(19, 21, 22), Vec3i::new(1, 12, 11),
        Vec3i::new(1, 11, 0), Vec3i::new(0, 11, 18), Vec3i::new(0, 18, 6), Vec3i::new(7, 19, 18),
        Vec3i::new(7, 18, 6), Vec3i::new(7, 19, 22), Vec3i::new(7, 22, 10), Vec3i::new(10, 22, 15),
        Vec3i::new(10, 15, 4), Vec3i::new(4, 15, 14), Vec3i::new(4, 14, 3), Vec3i::new(3, 14, 13),
        Vec3i::new(3, 14, 2), Vec3i::new(2, 13, 12), Vec3i::new(2, 12, 1), Vec3i::new(5, 16, 21),
        Vec3i::new(5, 21, 9), Vec3i::new(9, 21, 20), Vec3i::new(9, 20, 8), Vec3i::new(5, 17, 20),
        Vec3i::new(5, 20, 8),
    ];

    mesh
}

// ============================================================================
// TriangleMesh.cpp:1790-1945 — ITS merge / volume / neighbors / normals
// ============================================================================

/// TriangleMesh.cpp:1790-1794
/// C++: `void its_reverse_all_facets(indexed_triangle_set &its)`
pub fn its_reverse_all_facets(its: &mut indexed_triangle_set) {
    // TriangleMesh.cpp:1792-1793 — std::swap(face[0], face[1]);
    for face in &mut its.indices {
        let tmp = face[0];
        face[0] = face[1];
        face[1] = tmp;
    }
}

/// TriangleMesh.cpp:1796-1806
/// C++: `void its_merge(indexed_triangle_set &A, const indexed_triangle_set &B)`
pub fn its_merge(a: &mut indexed_triangle_set, b: &indexed_triangle_set) {
    // TriangleMesh.cpp:1798
    let n = a.vertices.len() as i32;
    // TriangleMesh.cpp:1799
    let n_f = a.indices.len();

    // TriangleMesh.cpp:1801-1802
    a.vertices.extend_from_slice(&b.vertices);
    a.indices.extend_from_slice(&b.indices);

    // TriangleMesh.cpp:1804-1805
    for idx in n_f..a.indices.len() {
        a.indices[idx] += Vec3i::new(n, n, n);
    }
}

/// TriangleMesh.cpp:1839-1847, the overload of `its_merge` over a triangle soup (TriangleMesh.hpp:334).
pub mod its_merge_triangles_overload {
    use super::*;
    /// TriangleMesh.cpp:1808-1816
    /// C++: `void its_merge(indexed_triangle_set &A, const std::vector<Vec3f> &triangles)`
    pub fn its_merge(a: &mut indexed_triangle_set, triangles: &[Vec3f]) {
        // TriangleMesh.cpp:1810
        let offs = a.vertices.len();
        // TriangleMesh.cpp:1811
        a.vertices.extend_from_slice(triangles);
        // TriangleMesh.cpp:1812
        a.indices.reserve(a.vertices.len() / 3);

        // TriangleMesh.cpp:1814-1815
        let mut i = offs as i32;
        let end = a.vertices.len() as i32;
        while i < end {
            a.indices.push(Vec3i::new(i, i + 1, i + 2));
            i += 3;
        }
    }
}
pub use its_merge_triangles_overload::its_merge as its_merge_triangles;

/// TriangleMesh.cpp:1849-1856, the overload of `its_merge` over `Pointf3s` (TriangleMesh.hpp:335).
pub mod its_merge_pointf3s_overload {
    use super::*;
    /// TriangleMesh.cpp:1818-1825
    /// C++: `void its_merge(indexed_triangle_set &A, const Pointf3s &triangles)`
    pub fn its_merge(a: &mut indexed_triangle_set, triangles: &[Point3F]) {
        // TriangleMesh.cpp:1820-1822
        let mut trianglesf: Vec<Vec3f> = Vec::with_capacity(triangles.len());
        for t in triangles {
            trianglesf.push(Vec3f::new(t.x as f32, t.y as f32, t.z as f32));
        }
        // TriangleMesh.cpp:1824
        its_merge_triangles(a, &trianglesf);
    }
}
pub use its_merge_pointf3s_overload::its_merge as its_merge_pointf3s;

/// TriangleMesh.cpp:1827-1846
/// C++: `float its_volume(const indexed_triangle_set &its)`
pub fn its_volume(its: &indexed_triangle_set) -> f32 {
    // TriangleMesh.cpp:1829
    if its.indices.is_empty() {
        return 0.0;
    }

    // TriangleMesh.cpp:1832 — Choose a point, any point as the reference.
    let p0 = its.vertices[0];
    // TriangleMesh.cpp:1833
    let mut volume = 0.0f32;
    // TriangleMesh.cpp:1834-1844
    for i in 0..its.indices.len() {
        // TriangleMesh.cpp:1836
        let triangle = its_triangle_vertices(its, i);
        // TriangleMesh.cpp:1837
        let u = triangle[1] - triangle[0];
        // TriangleMesh.cpp:1838
        let v = triangle[2] - triangle[0];
        if float_pin() {
            use crate::triangle_set_sampling::{eigen_cross3f, eigen_dot3f, eigen_normalized3f, eigen_squared_norm3f};
            // TriangleMesh.cpp:1839
            let c = eigen_cross3f(&u, &v);
            // TriangleMesh.cpp:1840
            let normal = eigen_normalized3f(&c);
            // TriangleMesh.cpp:1841 — float area = 0.5 * C.norm(); (computed in double then narrowed to float)
            let area = (0.5 * eigen_squared_norm3f(&c).sqrt() as f64) as f32;
            // TriangleMesh.cpp:1842
            let height = eigen_dot3f(&normal, &(triangle[0] - p0));
            // TriangleMesh.cpp:1843
            volume += (area * height) / 3.0;
            continue;
        }
        // TriangleMesh.cpp:1839
        let c = u.cross(&v);
        // TriangleMesh.cpp:1840
        let normal = c.normalize();
        // TriangleMesh.cpp:1841 — float area = 0.5 * C.norm(); (computed in double then narrowed to float)
        let area = (0.5 * c.norm() as f64) as f32;
        // TriangleMesh.cpp:1842
        let height = normal.dot(&(triangle[0] - p0));
        // TriangleMesh.cpp:1843
        volume += (area * height) / 3.0;
    }
    // TriangleMesh.cpp:1845
    volume
}

/// TriangleMesh.cpp:1848-1861
/// C++: `float its_average_edge_length(const indexed_triangle_set &its)`
pub fn its_average_edge_length(its: &indexed_triangle_set) -> f32 {
    // TriangleMesh.cpp:1850-1851
    if its.indices.is_empty() {
        return 0.0;
    }

    // TriangleMesh.cpp:1853
    let mut edge_length: f64 = 0.0;
    // TriangleMesh.cpp:1854-1859
    for i in 0..its.indices.len() {
        let v = its_triangle_vertices(its, i);
        // (v[1]-v[0]).cast<double>().norm() + (v[2]-v[0]).cast<double>().norm() + (v[1]-v[2]).cast<double>().norm()
        edge_length += (v[1] - v[0]).cast::<f64>().norm()
            + (v[2] - v[0]).cast::<f64>().norm()
            + (v[1] - v[2]).cast::<f64>().norm();
    }
    // TriangleMesh.cpp:1860
    (edge_length / (3 * its.indices.len()) as f64) as f32
}

/// TriangleMesh.cpp:1863-1866
/// C++: `std::vector<indexed_triangle_set> its_split(const indexed_triangle_set &its)`
pub fn its_split(its: &indexed_triangle_set) -> Vec<indexed_triangle_set> {
    // TriangleMesh.cpp:1865 — its_split<>(its) computes the neighbor index internally
    // via ItsWithNeighborsIndex_<indexed_triangle_set>::get_index = its_face_neighbors(its).
    let neighbor_index = its_face_neighbors(its);
    crate::mesh_split_impl::its_split_collect(its, &neighbor_index)
}

/// Number of disconnected patches.
/// TriangleMesh.cpp:1869-1872
pub fn its_number_of_patches(its: &indexed_triangle_set) -> usize {
    // TriangleMesh.cpp:1871 — its_number_of_patches<>(its) computes neighbors internally.
    let neighbor_index = its_face_neighbors(its);
    crate::mesh_split_impl::its_number_of_patches(its, &neighbor_index)
}

/// TriangleMesh.cpp:1904-1907, the overload of `its_number_of_patches` over the face neighbours.
pub mod its_number_of_patches_with_neighbors {
    use super::*;
    /// TriangleMesh.cpp:1873-1876
    pub fn its_number_of_patches(
        its: &indexed_triangle_set,
        face_neighbors: &[Vec3i],
    ) -> usize {
        // TriangleMesh.cpp:1875 — its_number_of_patches<>(ItsNeighborsWrapper{ its, face_neighbors });
        crate::mesh_split_impl::its_number_of_patches(its, face_neighbors)
    }
}
pub use its_number_of_patches_with_neighbors::its_number_of_patches as its_number_of_patches_neighbors;

/// Same as its_number_of_patches(its) > 1, but faster.
/// TriangleMesh.cpp:1879-1882
pub fn its_is_splittable(its: &indexed_triangle_set) -> bool {
    // TriangleMesh.cpp:1881 — its_is_splittable<>(its)
    let neighbor_index = its_face_neighbors(its);
    crate::mesh_split_impl::its_is_splittable(its, &neighbor_index)
}

/// TriangleMesh.cpp:1914-1917, the overload of `its_is_splittable` over the face neighbours.
pub mod its_is_splittable_with_neighbors {
    use super::*;
    /// TriangleMesh.cpp:1883-1886
    pub fn its_is_splittable(its: &indexed_triangle_set, face_neighbors: &[Vec3i]) -> bool {
        // TriangleMesh.cpp:1885
        crate::mesh_split_impl::its_is_splittable(its, face_neighbors)
    }
}
pub use its_is_splittable_with_neighbors::its_is_splittable as its_is_splittable_neighbors;

/// The open edges counted on a face neighbour index (an edge without a neighbour). Not a
/// function of the pin, which counts the undirected edges referenced once
/// ([`its_num_open_edges`]); kept for the earlier arm of `TRIANGLE_MESH_STATS_PIN`.
pub fn its_num_open_edges_neighbors(face_neighbors: &[Vec3i]) -> usize {
    // TriangleMesh.cpp:1890
    let mut num_open_edges: usize = 0;
    // TriangleMesh.cpp:1891-1894
    for neighbors in face_neighbors {
        for k in 0..3 {
            if neighbors[k] < 0 {
                num_open_edges += 1;
            }
        }
    }
    // TriangleMesh.cpp:1895
    num_open_edges
}

/// TriangleMesh.cpp:1919-1942
/// C++: `size_t its_num_open_edges(const indexed_triangle_set &its)`: the undirected edges
/// referenced by exactly one half-edge.
pub fn its_num_open_edges(its: &indexed_triangle_set) -> usize {
    // TriangleMesh.cpp:1921-1922
    let mut edges: Vec<i64> = Vec::with_capacity(its.indices.len() * 3);
    // TriangleMesh.cpp:1923-1930
    for face in &its.indices {
        for i in 0..3 {
            let mut v0 = face[i];
            let mut v1 = face[(i + 1) % 3];
            if v0 > v1 {
                std::mem::swap(&mut v0, &mut v1);
            }
            // (int64_t(v0) << 32) | int64_t(v1): the sign of v1 extends over the high word.
            edges.push(((v0 as i64) << 32) | (v1 as i64));
        }
    }
    // TriangleMesh.cpp:1931 std::sort (integers: the order of equal keys cannot be told apart)
    edges.sort_unstable();

    // TriangleMesh.cpp:1933-1941
    let mut num_open: usize = 0;
    let mut i = 0usize;
    while i < edges.len() {
        let mut j = i + 1;
        while j < edges.len() && edges[j] == edges[i] {
            j += 1;
        }
        if j - i == 1 {
            num_open += 1;
        }
        i = j;
    }
    num_open
}

/// TriangleMesh.cpp:1928-1931
/// C++: `std::vector<Vec3i> its_face_neighbors(const indexed_triangle_set &its)`
pub fn its_face_neighbors(its: &indexed_triangle_set) -> Vec<Vec3i> {
    // TriangleMesh.cpp:1930 — create_face_neighbors_index(ex_seq, its);
    crate::mesh_split_impl::create_face_neighbors_index(&crate::execution::EX_SEQ, its)
}

/// TriangleMesh.cpp:1933-1936
/// C++: `std::vector<Vec3i> its_face_neighbors_par(const indexed_triangle_set &its)`
pub fn its_face_neighbors_par(its: &indexed_triangle_set) -> Vec<Vec3i> {
    // TriangleMesh.cpp:1935 — create_face_neighbors_index(ex_tbb, its);
    crate::mesh_split_impl::create_face_neighbors_index(&crate::execution::EX_TBB, its)
}

/// TriangleMesh.cpp:1938-1945
/// C++: `std::vector<Vec3f> its_face_normals(const indexed_triangle_set &its)`
pub fn its_face_normals(its: &indexed_triangle_set) -> Vec<Vec3f> {
    // TriangleMesh.cpp:1940
    let mut normals: Vec<Vec3f> = Vec::new();
    // TriangleMesh.cpp:1941
    normals.reserve(its.indices.len());
    // TriangleMesh.cpp:1942-1943
    for face in &its.indices {
        normals.push(its_face_normal_indices(its, face));
    }
    // TriangleMesh.cpp:1944
    normals
}

// ============================================================================
// TriangleMesh.hpp:249-336 — inline header helpers
// ============================================================================

/// Index of a vertex inside triangle_indices.
/// TriangleMesh.hpp:249-254
pub fn its_triangle_vertex_index(
    triangle_indices: &StlTriangleVertexIndices,
    vertex_idx: i32,
) -> i32 {
    // TriangleMesh.hpp:251-253
    if vertex_idx == triangle_indices[0] {
        0
    } else if vertex_idx == triangle_indices[1] {
        1
    } else if vertex_idx == triangle_indices[2] {
        2
    } else {
        -1
    }
}

/// TriangleMesh.hpp:256-260
/// C++: `inline Vec2i its_triangle_edge(const stl_triangle_vertex_indices &triangle_indices, int edge_idx)`
pub fn its_triangle_edge(triangle_indices: &StlTriangleVertexIndices, edge_idx: i32) -> Vec2i {
    // TriangleMesh.hpp:258
    let next_edge_idx = if edge_idx == 2 { 0 } else { edge_idx + 1 };
    // TriangleMesh.hpp:259
    Vec2i::new(
        triangle_indices[edge_idx as usize],
        triangle_indices[next_edge_idx as usize],
    )
}

/// Index of an edge inside triangle.
/// TriangleMesh.hpp:263-268
pub fn its_triangle_edge_index(
    triangle_indices: &StlTriangleVertexIndices,
    triangle_edge: &Vec2i,
) -> i32 {
    // TriangleMesh.hpp:265-267
    if triangle_edge[0] == triangle_indices[0] && triangle_edge[1] == triangle_indices[1] {
        0
    } else if triangle_edge[0] == triangle_indices[1] && triangle_edge[1] == triangle_indices[2] {
        1
    } else if triangle_edge[0] == triangle_indices[2] && triangle_edge[1] == triangle_indices[0] {
        2
    } else {
        -1
    }
}

/// juedge whether two triangles has the same vertices
/// TriangleMesh.hpp:271-303
pub fn its_triangle_vertex_the_same(
    triangle_indices_1: &StlTriangleVertexIndices,
    triangle_indices_2: &StlTriangleVertexIndices,
) -> bool {
    // TriangleMesh.hpp:273
    let mut ret = false;
    // TriangleMesh.hpp:274
    if triangle_indices_1[0] == triangle_indices_2[0] {
        // TriangleMesh.hpp:276-278
        if triangle_indices_1[1] == triangle_indices_2[1]
            && triangle_indices_1[2] == triangle_indices_2[2]
        {
            ret = true;
        }
        // TriangleMesh.hpp:279-281
        else if triangle_indices_1[1] == triangle_indices_2[2]
            && triangle_indices_1[2] == triangle_indices_2[1]
        {
            ret = true;
        }
    }
    // TriangleMesh.hpp:283
    else if triangle_indices_1[0] == triangle_indices_2[1] {
        // TriangleMesh.hpp:285-287
        if triangle_indices_1[1] == triangle_indices_2[0]
            && triangle_indices_1[2] == triangle_indices_2[2]
        {
            ret = true;
        }
        // TriangleMesh.hpp:288-290
        else if triangle_indices_1[1] == triangle_indices_2[2]
            && triangle_indices_1[2] == triangle_indices_2[0]
        {
            ret = true;
        }
    }
    // TriangleMesh.hpp:292
    else if triangle_indices_1[0] == triangle_indices_2[2] {
        // TriangleMesh.hpp:294-296
        if triangle_indices_1[1] == triangle_indices_2[0]
            && triangle_indices_1[2] == triangle_indices_2[1]
        {
            ret = true;
        }
        // TriangleMesh.hpp:297-299
        else if triangle_indices_1[1] == triangle_indices_2[1]
            && triangle_indices_1[2] == triangle_indices_2[0]
        {
            ret = true;
        }
    }

    // TriangleMesh.hpp:302
    ret
}

/// `using its_triangle = std::array<stl_vertex, 3>;`
/// TriangleMesh.hpp:306
pub type ItsTriangle = [StlVertex; 3];

/// TriangleMesh.hpp:308-314
/// C++: `inline its_triangle its_triangle_vertices(const indexed_triangle_set &its, size_t face_id)`
pub fn its_triangle_vertices(its: &indexed_triangle_set, face_id: usize) -> ItsTriangle {
    // TriangleMesh.hpp:311-313
    [
        its.vertices[its.indices[face_id][0] as usize],
        its.vertices[its.indices[face_id][1] as usize],
        its.vertices[its.indices[face_id][2] as usize],
    ]
}

/// TriangleMesh.hpp:316-321
/// C++: `inline stl_normal its_unnormalized_normal(const indexed_triangle_set &its, size_t face_id)`
pub fn its_unnormalized_normal(its: &indexed_triangle_set, face_id: usize) -> Vec3f {
    // TriangleMesh.hpp:319
    let tri = its_triangle_vertices(its, face_id);
    // TriangleMesh.hpp:320
    if float_pin() {
        return crate::triangle_set_sampling::eigen_cross3f(&(tri[1] - tri[0]), &(tri[2] - tri[0]));
    }
    (tri[1] - tri[0]).cross(&(tri[2] - tri[0]))
}

/// TriangleMesh.hpp:331
/// C++: `inline Vec3f face_normal(const stl_vertex vertex[3]) { return (vertex[1] - vertex[0]).cross(vertex[2] - vertex[1]).normalized(); }`
pub fn face_normal(vertex: &[StlVertex; 3]) -> Vec3f {
    if float_pin() {
        return crate::triangle_set_sampling::eigen_normalized3f(&crate::triangle_set_sampling::eigen_cross3f(
            &(vertex[1] - vertex[0]),
            &(vertex[2] - vertex[1]),
        ));
    }
    (vertex[1] - vertex[0]).cross(&(vertex[2] - vertex[1])).normalize()
}

/// TriangleMesh.hpp:332
/// C++: `inline Vec3f face_normal_normalized(const stl_vertex vertex[3]) { return face_normal(vertex).normalized(); }`
pub fn face_normal_normalized(vertex: &[StlVertex; 3]) -> Vec3f {
    if float_pin() {
        return crate::triangle_set_sampling::eigen_normalized3f(&face_normal(vertex));
    }
    face_normal(vertex).normalize()
}

/// TriangleMesh.hpp:340-341, the overload of `its_face_normal` over the vertex indices of a face.
pub mod its_face_normal_of_indices {
    use super::*;
    /// TriangleMesh.hpp:340-341
    /// C++: `inline Vec3f its_face_normal(const indexed_triangle_set &its, const stl_triangle_vertex_indices face)`
    pub fn its_face_normal(its: &indexed_triangle_set, face: &StlTriangleVertexIndices) -> Vec3f {
        let vertices: [StlVertex; 3] =
            [its.vertices[face[0] as usize], its.vertices[face[1] as usize], its.vertices[face[2] as usize]];
        face_normal_normalized(&vertices)
    }
}
pub use its_face_normal_of_indices::its_face_normal as its_face_normal_indices;

/// TriangleMesh.hpp:342-343
/// C++: `inline Vec3f its_face_normal(const indexed_triangle_set &its, const int face_idx)`
pub fn its_face_normal(its: &indexed_triangle_set, face_idx: i32) -> Vec3f {
    its_face_normal_indices(its, &its.indices[face_idx as usize])
}

/// TriangleMesh.hpp:366-379
/// C++: `inline BoundingBoxf3 bounding_box(const indexed_triangle_set& its)`
pub fn bounding_box(its: &indexed_triangle_set) -> BoundingBox3F {
    // TriangleMesh.hpp:368-369
    if its.vertices.is_empty() {
        return BoundingBox3F::new();
    }

    // TriangleMesh.hpp:371
    let mut bmin = its.vertices[0];
    let mut bmax = its.vertices[0];

    // TriangleMesh.hpp:373-376
    for p in &its.vertices {
        bmin = p.inf(&bmin); // cwiseMin
        bmax = p.sup(&bmax); // cwiseMax
    }

    // TriangleMesh.hpp:378 — {bmin.cast<double>(), bmax.cast<double>()}
    BoundingBox3F::from_points_minmax(
        Point3F::new(bmin.x as f64, bmin.y as f64, bmin.z as f64),
        Point3F::new(bmax.x as f64, bmax.y as f64, bmax.z as f64),
    )
}

// ============================================================================
// TriangleMesh.cpp:1947-2020 — STL writers
// ============================================================================

/// TriangleMesh.cpp:1990-1991 (`#if BOOST_ENDIAN_LITTLE_BYTE`): a no-op on a little-endian host.
#[cfg(target_endian = "little")]
#[inline]
fn big_endian_reverse_quads(_buf: &mut [u8], _cnt: usize) {}

/// TriangleMesh.cpp:1992-1999 (the big-endian host): swap the bytes of every quad.
#[cfg(target_endian = "big")]
#[inline]
fn big_endian_reverse_quads(buf: &mut [u8], cnt: usize) {
    // TriangleMesh.cpp:1994-1997
    let mut i = 0usize;
    while i < cnt {
        buf.swap(i, i + 3);
        buf.swap(i + 1, i + 2);
        i += 4;
    }
}

/// TriangleMesh.cpp:1959-1984
/// C++: `bool its_write_stl_ascii(const char *file, const char *label, const std::vector<stl_triangle_vertex_indices> &indices, const std::vector<stl_vertex> &vertices)`
pub fn its_write_stl_ascii(
    file: &str,
    label: &str,
    indices: &[StlTriangleVertexIndices],
    vertices: &[StlVertex],
) -> bool {
    use std::io::Write;
    // TriangleMesh.cpp:1961-1965
    let f = match std::fs::File::create(file) {
        Ok(f) => f,
        Err(_) => {
            // TriangleMesh.cpp:1963 — log + return false.
            return false;
        }
    };
    let mut fp = std::io::BufWriter::new(f);

    // TriangleMesh.cpp:1967 — fprintf(fp, "solid  %s\n", label);
    if write!(fp, "solid  {}\n", label).is_err() {
        return false;
    }

    // TriangleMesh.cpp:1969-1979
    for face in indices {
        // TriangleMesh.cpp:1970
        let vertex: [Vec3f; 3] = [
            vertices[face[0] as usize],
            vertices[face[1] as usize],
            vertices[face[2] as usize],
        ];
        // TriangleMesh.cpp:1971
        let normal = if float_pin() {
            face_normal(&vertex)
        } else {
            (vertex[1] - vertex[0]).cross(&(vertex[2] - vertex[1])).normalize()
        };
        // TriangleMesh.cpp:1972 — "  facet normal % .8E % .8E % .8E\n"
        let _ = write!(
            fp,
            "  facet normal {} {} {}\n",
            fmt_e8(normal[0]),
            fmt_e8(normal[1]),
            fmt_e8(normal[2])
        );
        // TriangleMesh.cpp:1973
        let _ = write!(fp, "    outer loop\n");
        // TriangleMesh.cpp:1974-1976
        for v in &vertex {
            let _ = write!(
                fp,
                "      vertex {} {} {}\n",
                fmt_e8(v[0]),
                fmt_e8(v[1]),
                fmt_e8(v[2])
            );
        }
        // TriangleMesh.cpp:1977
        let _ = write!(fp, "    endloop\n");
        // TriangleMesh.cpp:1978
        let _ = write!(fp, "  endfacet\n");
    }

    // TriangleMesh.cpp:1981
    let _ = write!(fp, "endsolid  {}\n", label);
    // TriangleMesh.cpp:1982-1983
    fp.flush().is_ok()
}

/// Format a float like C `% .8E` (space for sign, 8 fractional digits, uppercase E
/// exponent with at least two digits).
fn fmt_e8(v: f32) -> String {
    // C `% .8E`: leading space if non-negative, capital E, 2+ digit signed exponent.
    let s = format!("{:.8E}", v);
    // Rust prints exponent without leading zero / explicit sign: e.g. "1.00000000E0".
    // Normalize to C form "1.00000000E+00".
    let normalized = normalize_c_exponent(&s);
    if v.is_sign_negative() {
        normalized
    } else {
        format!(" {}", normalized)
    }
}

/// Convert Rust `{:E}` exponent (`E0`, `E-3`, `E12`) to C `printf` form (`E+00`, `E-03`, `E+12`).
fn normalize_c_exponent(s: &str) -> String {
    if let Some(pos) = s.find('E') {
        let (mantissa, exp) = s.split_at(pos);
        let exp = &exp[1..]; // strip 'E'
        let (sign, digits) = if let Some(rest) = exp.strip_prefix('-') {
            ('-', rest)
        } else if let Some(rest) = exp.strip_prefix('+') {
            ('+', rest)
        } else {
            ('+', exp)
        };
        format!("{}E{}{:0>2}", mantissa, sign, digits)
    } else {
        s.to_string()
    }
}

/// TriangleMesh.cpp:1986-2020
/// C++: `bool its_write_stl_binary(const char *file, const char *label, const std::vector<stl_triangle_vertex_indices> &indices, const std::vector<stl_vertex> &vertices)`
pub fn its_write_stl_binary(
    file: &str,
    label: &str,
    indices: &[StlTriangleVertexIndices],
    vertices: &[StlVertex],
) -> bool {
    use std::io::Write;
    // TriangleMesh.cpp:1988-1992
    let f = match std::fs::File::create(file) {
        Ok(f) => f,
        Err(_) => return false,
    };
    let mut fp = std::io::BufWriter::new(f);

    // TriangleMesh.cpp:1994-2000 — 80 byte header.
    {
        const HEADER_SIZE: usize = 80;
        let mut header = vec![0u8; HEADER_SIZE];
        // TriangleMesh.cpp:1997 — copy up to 80 bytes of label.
        let label_bytes = label.as_bytes();
        let header_len = std::cmp::min(label_bytes.len(), HEADER_SIZE);
        if header_len > 0 {
            header[..header_len].copy_from_slice(&label_bytes[..header_len]);
        }
        // TriangleMesh.cpp:1999
        if fp.write_all(&header).is_err() {
            return false;
        }
    }

    // TriangleMesh.cpp:2002-2004
    let mut nfaces: u32 = indices.len() as u32;
    big_endian_reverse_quads(bytemuck_u32(&mut nfaces), 4);
    if fp.write_all(&nfaces.to_le_bytes()).is_err() {
        return false;
    }

    // TriangleMesh.cpp:2006-2016 — write each facet (normal + 3 vertices + 2 extra bytes = 50 bytes).
    for face in indices {
        // TriangleMesh.cpp:2010-2012
        let v0 = vertices[face[0] as usize];
        let v1 = vertices[face[1] as usize];
        let v2 = vertices[face[2] as usize];
        // TriangleMesh.cpp:2013
        let normal = if float_pin() { face_normal(&[v0, v1, v2]) } else { (v1 - v0).cross(&(v2 - v1)).normalize() };

        // 48-byte float block: normal, v0, v1, v2 (big_endian_reverse_quads is a no-op LE).
        let mut buf = [0u8; 50];
        buf[0..4].copy_from_slice(&normal[0].to_le_bytes());
        buf[4..8].copy_from_slice(&normal[1].to_le_bytes());
        buf[8..12].copy_from_slice(&normal[2].to_le_bytes());
        buf[12..16].copy_from_slice(&v0[0].to_le_bytes());
        buf[16..20].copy_from_slice(&v0[1].to_le_bytes());
        buf[20..24].copy_from_slice(&v0[2].to_le_bytes());
        buf[24..28].copy_from_slice(&v1[0].to_le_bytes());
        buf[28..32].copy_from_slice(&v1[1].to_le_bytes());
        buf[32..36].copy_from_slice(&v1[2].to_le_bytes());
        buf[36..40].copy_from_slice(&v2[0].to_le_bytes());
        buf[40..44].copy_from_slice(&v2[1].to_le_bytes());
        buf[44..48].copy_from_slice(&v2[2].to_le_bytes());
        // buf[48..50] are the two `extra` bytes (0).
        big_endian_reverse_quads(&mut buf, 48);
        // TriangleMesh.cpp:2015 — fwrite(&f, 50, 1, fp);
        if fp.write_all(&buf).is_err() {
            return false;
        }
    }

    // TriangleMesh.cpp:2018-2019
    fp.flush().is_ok()
}

/// TriangleMesh.hpp:368, the overload of `its_write_stl_ascii` over an `indexed_triangle_set`.
pub mod its_write_stl_ascii_its {
    use super::*;
    /// TriangleMesh.hpp:368
    pub fn its_write_stl_ascii(file: &str, label: &str, its: &indexed_triangle_set) -> bool {
        super::its_write_stl_ascii(file, label, &its.indices, &its.vertices)
    }
}

/// TriangleMesh.hpp:370, the overload of `its_write_stl_binary` over an `indexed_triangle_set`.
pub mod its_write_stl_binary_its {
    use super::*;
    /// TriangleMesh.hpp:370
    pub fn its_write_stl_binary(file: &str, label: &str, its: &indexed_triangle_set) -> bool {
        super::its_write_stl_binary(file, label, &its.indices, &its.vertices)
    }
}

/// View a `&mut u32` as a `&mut [u8]` for the (LE no-op) byte-swap call.
#[inline]
fn bytemuck_u32(v: &mut u32) -> &mut [u8] {
    // SAFETY: u32 is 4 bytes with no padding; we view it as raw bytes for an in-place
    // (no-op on LE) byte swap matching the C++ `reinterpret_cast<char*>(&nfaces)`.
    unsafe { std::slice::from_raw_parts_mut(v as *mut u32 as *mut u8, 4) }
}

// ============================================================================
// VertexFaceIndex — TriangleMesh.hpp:168-190 / TriangleMesh.cpp:1903-1926
// ============================================================================

/// Index of face indices incident with a vertex index.
/// TriangleMesh.hpp:168-190
#[derive(Default)]
pub struct VertexFaceIndex {
    /// TriangleMesh.hpp:188
    m_vertex_to_face_start: Vec<usize>,
    /// TriangleMesh.hpp:189
    m_vertex_faces_all: Vec<usize>,
}

impl VertexFaceIndex {
    /// TriangleMesh.hpp:173
    /// C++: `VertexFaceIndex(const indexed_triangle_set &its) { this->create(its); }`
    pub fn new(its: &indexed_triangle_set) -> Self {
        let mut idx = VertexFaceIndex::default();
        idx.create(its);
        idx
    }

    /// TriangleMesh.hpp:177
    /// C++: `void clear() { m_vertex_to_face_start.clear(); m_vertex_faces_all.clear(); }`
    pub fn clear(&mut self) {
        self.m_vertex_to_face_start.clear();
        self.m_vertex_faces_all.clear();
    }

    /// Iterators of face indices incident with the input vertex_id.
    /// TriangleMesh.hpp:180-181
    pub fn begin(&self, vertex_id: usize) -> usize {
        self.m_vertex_to_face_start[vertex_id]
    }
    pub fn end(&self, vertex_id: usize) -> usize {
        self.m_vertex_to_face_start[vertex_id + 1]
    }

    /// Vertex incidence.
    /// TriangleMesh.hpp:183
    pub fn count(&self, vertex_id: usize) -> usize {
        self.m_vertex_to_face_start[vertex_id + 1] - self.m_vertex_to_face_start[vertex_id]
    }

    /// Returns the slice of face indices incident with `vertex_id`.
    /// TriangleMesh.hpp:185 — operator[]
    pub fn faces(&self, vertex_id: usize) -> &[usize] {
        &self.m_vertex_faces_all[self.begin(vertex_id)..self.end(vertex_id)]
    }

    /// TriangleMesh.cpp:1903-1926
    /// C++: `void VertexFaceIndex::create(const indexed_triangle_set &its)`
    pub fn create(&mut self, its: &indexed_triangle_set) {
        // TriangleMesh.cpp:1905
        self.m_vertex_to_face_start = vec![0usize; its.vertices.len() + 1];
        // TriangleMesh.cpp:1906-1911 — 1) Calculate vertex incidence by scatter.
        for face in &its.indices {
            self.m_vertex_to_face_start[(face[0] + 1) as usize] += 1;
            self.m_vertex_to_face_start[(face[1] + 1) as usize] += 1;
            self.m_vertex_to_face_start[(face[2] + 1) as usize] += 1;
        }
        // TriangleMesh.cpp:1912-1914 — 2) Prefix sum to calculate offsets to m_vertex_faces_all.
        for i in 2..self.m_vertex_to_face_start.len() {
            self.m_vertex_to_face_start[i] += self.m_vertex_to_face_start[i - 1];
        }
        // TriangleMesh.cpp:1915-1921 — 3) Scatter indices of faces incident to a vertex.
        let total = *self.m_vertex_to_face_start.last().unwrap();
        self.m_vertex_faces_all = vec![0usize; total];
        for face_idx in 0..its.indices.len() {
            let face = &its.indices[face_idx];
            for i in 0..3 {
                let slot = self.m_vertex_to_face_start[face[i] as usize];
                self.m_vertex_faces_all[slot] = face_idx;
                self.m_vertex_to_face_start[face[i] as usize] += 1;
            }
        }
        // TriangleMesh.cpp:1922-1925 — 4) The previous loop modified m_vertex_to_face_start. Revert the change.
        let mut i = self.m_vertex_to_face_start.len() as i64 - 1;
        while i > 0 {
            self.m_vertex_to_face_start[i as usize] = self.m_vertex_to_face_start[(i - 1) as usize];
            i -= 1;
        }
        self.m_vertex_to_face_start[0] = 0;
    }
}

// ============================================================================
// TriangleMesh.cpp:872-908 — store part of an indexed triangle set
// ============================================================================

/// admesh/shared.cpp:193-208 `bool its_write_obj(const indexed_triangle_set &its, const char *file)`,
/// which `its_store_triangle(s)` and `TriangleMesh::WriteOBJFile` call: `v %f %f %f` lines (the
/// `float` promoted to `double`, six decimals, the C locale) and 1-based `f %d %d %d` lines.
fn its_write_obj(its: &indexed_triangle_set, file: &str) -> bool {
    use std::io::Write;
    // admesh/shared.cpp:196-200
    let f = match std::fs::File::create(file) {
        Ok(f) => f,
        Err(_) => {
            log::error!("stl_write_obj: Couldn't open {} for writing", file);
            return false;
        }
    };
    let mut fp = std::io::BufWriter::new(f);
    // admesh/shared.cpp:202-203
    for v in &its.vertices {
        if write!(fp, "v {:.6} {:.6} {:.6}\n", v[0] as f64, v[1] as f64, v[2] as f64).is_err() {
            return false;
        }
    }
    // admesh/shared.cpp:204-205
    for t in &its.indices {
        if write!(fp, "f {} {} {}\n", t[0] + 1, t[1] + 1, t[2] + 1).is_err() {
            return false;
        }
    }
    // admesh/shared.cpp:206-207
    fp.flush().is_ok()
}

/// TriangleMesh.cpp:872-883
/// C++: `bool its_store_triangle(const indexed_triangle_set &its, const char *obj_filename, size_t triangle_index)`
pub fn its_store_triangle(its: &indexed_triangle_set, obj_filename: &str, triangle_index: usize) -> bool {
    // TriangleMesh.cpp:876
    if its.indices.len() <= triangle_index {
        return false;
    }
    // TriangleMesh.cpp:877
    let t: Vec3i = its.indices[triangle_index];
    // TriangleMesh.cpp:878-881
    let mut its2 = indexed_triangle_set::default();
    its2.indices = vec![Vec3i::new(0, 1, 2)];
    its2.vertices = vec![its.vertices[t[0] as usize], its.vertices[t[1] as usize], its.vertices[t[2] as usize]];
    // TriangleMesh.cpp:882
    its_write_obj(&its2, obj_filename)
}

/// TriangleMesh.cpp:885-908
/// C++: `bool its_store_triangles(const indexed_triangle_set &its, const char *obj_filename, const std::vector<size_t>& triangles)`
pub fn its_store_triangles(its: &indexed_triangle_set, obj_filename: &str, triangles: &[usize]) -> bool {
    // TriangleMesh.cpp:889-891
    let mut its2 = indexed_triangle_set::default();
    its2.vertices.reserve(triangles.len() * 3);
    its2.indices.reserve(triangles.len());
    // TriangleMesh.cpp:892
    let mut vertex_map: std::collections::BTreeMap<usize, usize> = std::collections::BTreeMap::new();
    // TriangleMesh.cpp:893-906
    for &ti in triangles {
        // TriangleMesh.cpp:894
        if its.indices.len() <= ti {
            return false;
        }
        // TriangleMesh.cpp:895-896
        let t: Vec3i = its.indices[ti];
        let mut new_t = Vec3i::zeros();
        // TriangleMesh.cpp:897-904
        for i in 0..3usize {
            // size_t vi = t[i]: a negative index extends its sign
            let vi = t[i] as isize as usize;
            if let Some(&nv) = vertex_map.get(&vi) {
                new_t[i] = nv as i32;
                continue;
            }
            let new_vi = its2.vertices.len();
            its2.vertices.push(its.vertices[vi]);
            vertex_map.insert(vi, new_vi);
            new_t[i] = new_vi as i32;
        }
        // TriangleMesh.cpp:905
        its2.indices.push(new_t);
    }
    // TriangleMesh.cpp:907
    its_write_obj(&its2, obj_filename)
}

// ============================================================================
// TriangleMesh.hpp:360-365 — the TriangleMesh makers
// ============================================================================

/// TriangleMesh.hpp:360 `TriangleMesh make_cube(double x, double y, double z)`
pub fn make_cube(x: f64, y: f64, z: f64) -> crate::model::ModelMesh {
    crate::model::ModelMesh::from_its(its_make_cube(x, y, z))
}

/// TriangleMesh.hpp:361 `TriangleMesh make_prism(float width, float length, float height)`
pub fn make_prism(width: f32, length: f32, height: f32) -> crate::model::ModelMesh {
    crate::model::ModelMesh::from_its(its_make_prism(width, length, height))
}

/// TriangleMesh.hpp:362 `TriangleMesh make_cylinder(double r, double h, double fa=(2*PI/360))`
pub fn make_cylinder(r: f64, h: f64, fa: f64) -> crate::model::ModelMesh {
    crate::model::ModelMesh::from_its(its_make_cylinder(r, h, fa))
}

/// TriangleMesh.hpp:363 `TriangleMesh make_cone(double r, double h, double fa=(2*PI/360))`
pub fn make_cone(r: f64, h: f64, fa: f64) -> crate::model::ModelMesh {
    crate::model::ModelMesh::from_its(its_make_cone(r, h, fa))
}

/// TriangleMesh.hpp:364 `TriangleMesh make_pyramid(float base, float height)`
pub fn make_pyramid(base: f32, height: f32) -> crate::model::ModelMesh {
    crate::model::ModelMesh::from_its(its_make_pyramid(base, height))
}

/// TriangleMesh.hpp:365 `TriangleMesh make_sphere(double rho, double fa=(2*PI/360))`
pub fn make_sphere(rho: f64, fa: f64) -> crate::model::ModelMesh {
    crate::model::ModelMesh::from_its(its_make_sphere(rho, fa))
}

// ============================================================================
// The TriangleMesh class: the methods `model::ModelMesh` (the crate's port of the class,
// TriangleMesh.hpp:95-173) does not carry, written over its public interface.
// ============================================================================

/// admesh/util.cpp:150-157 `static void rotate_point_2d(float &x, float &y, const double c, const double s)`:
/// `x = float(c * xold - s * yold); y = float(s * xold + c * yold);`, each the first product fused.
fn rotate_point_2d(x: &mut f32, y: &mut f32, c: f64, s: f64) {
    let xold = *x as f64;
    let yold = *y as f64;
    *x = c.mul_add(xold, -(s * yold)) as f32;
    *y = s.mul_add(xold, c * yold) as f32;
}

/// admesh/util.cpp:180-187 `void its_rotate_z(indexed_triangle_set& its, float angle)`, the angle
/// in degrees.
fn its_rotate_z(its: &mut indexed_triangle_set, angle: f32) {
    // admesh/util.cpp:182: (angle / 180.0) * M_PI
    let radian_angle = (angle as f64 / 180.0) * std::f64::consts::PI;
    let c = radian_angle.cos();
    let s = radian_angle.sin();
    for v in &mut its.vertices {
        let (mut vx, mut vy) = (v[0], v[1]);
        rotate_point_2d(&mut vx, &mut vy, c, s);
        v[0] = vx;
        v[1] = vy;
    }
}

/// admesh/util.cpp:162-169 `void its_rotate_x(indexed_triangle_set &its, float angle)`, the angle
/// in degrees: `rotate_point_2d(v(1), v(2), c, s)`.
fn its_rotate_x(its: &mut indexed_triangle_set, angle: f32) {
    // admesh/util.cpp:164: (angle / 180.0) * M_PI
    let radian_angle = (angle as f64 / 180.0) * std::f64::consts::PI;
    let c = radian_angle.cos();
    let s = radian_angle.sin();
    for v in &mut its.vertices {
        let (mut a, mut b) = (v[1], v[2]);
        rotate_point_2d(&mut a, &mut b, c, s);
        v[1] = a;
        v[2] = b;
    }
}

/// admesh/util.cpp:171-178 `void its_rotate_y(indexed_triangle_set& its, float angle)`, the angle
/// in degrees: `rotate_point_2d(v(2), v(0), c, s)`.
fn its_rotate_y(its: &mut indexed_triangle_set, angle: f32) {
    // admesh/util.cpp:173: (angle / 180.0) * M_PI
    let radian_angle = (angle as f64 / 180.0) * std::f64::consts::PI;
    let c = radian_angle.cos();
    let s = radian_angle.sin();
    for v in &mut its.vertices {
        let (mut a, mut b) = (v[2], v[0]);
        rotate_point_2d(&mut a, &mut b, c, s);
        v[2] = a;
        v[0] = b;
    }
}

/// TriangleMesh.hpp:113 `void rotate(float angle, const Axis &axis)`, the overload of `rotate`
/// about a coordinate axis (the inherent `rotate` is `rotate(double angle, Point* center)`).
pub trait TriangleMeshRotateAxis {
    fn rotate(&mut self, angle: f32, axis: crate::geometry::geometry::Axis);
}

impl TriangleMeshRotateAxis for crate::model::ModelMesh {
    /// TriangleMesh.cpp:311-323
    fn rotate(&mut self, angle: f32, axis: crate::geometry::geometry::Axis) {
        use crate::geometry::geometry::Axis;
        // TriangleMesh.cpp:313
        if angle != 0.0f32 {
            // TriangleMesh.cpp:314 — Geometry::rad2deg<float>: `T(180.0) * angle / T(PI)` (Geometry.hpp:293)
            let angle = 180.0f32 * angle / (std::f64::consts::PI as f32);
            // TriangleMesh.cpp:315-320
            match axis {
                Axis::X => its_rotate_x(&mut self.its, angle),
                Axis::Y => its_rotate_y(&mut self.its, angle),
                Axis::Z => its_rotate_z(&mut self.its, angle),
                #[allow(unreachable_patterns)]
                _ => {
                    debug_assert!(false);
                    return;
                }
            }
            // TriangleMesh.cpp:321
            let its = std::mem::take(&mut self.its);
            update_bounding_box(&its, self.m_stats_mut());
            self.its = its;
        }
    }
}

/// TriangleMesh.hpp:114 `void rotate(float angle, const Vec3d& axis)`, the overload of `rotate`
/// about an arbitrary axis.
pub trait TriangleMeshRotateVec3d {
    fn rotate(&mut self, angle: f32, axis: &crate::geometry::geometry::Vec3d);
}

impl TriangleMeshRotateVec3d for crate::model::ModelMesh {
    /// TriangleMesh.cpp:325-334
    fn rotate(&mut self, angle: f32, axis: &crate::geometry::geometry::Vec3d) {
        use crate::geometry::geometry::eigen;
        // TriangleMesh.cpp:327
        if angle != 0.0f32 {
            // TriangleMesh.cpp:328
            let axis_norm = eigen::normalized3(axis);
            // TriangleMesh.cpp:329-330 — `m.rotate(r)` is `linearExt() *= r.toRotationMatrix()`
            // (Eigen Transform.h), the identity times the rotation.
            let r = eigen::AngleAxisd::new(angle as f64, axis_norm).to_rotation_matrix();
            let l = eigen::mat3_mul(&nalgebra::Matrix3::<f64>::identity(), &r);
            let mut m = nalgebra::Matrix4::<f64>::identity();
            m.fixed_view_mut::<3, 3>(0, 0).copy_from(&l);
            // TriangleMesh.cpp:331 — admesh/stl.h:371-380 its_transform(its, m): `(t * v.cast<double>()).cast<float>()`
            for v in &mut self.its.vertices {
                let p = eigen::transform_mul_vec(&m, &crate::geometry::geometry::Vec3d::new(v.x as f64, v.y as f64, v.z as f64));
                *v = Vec3f::new(p.x as f32, p.y as f32, p.z as f32);
            }
            // TriangleMesh.cpp:332
            let its = std::mem::take(&mut self.its);
            update_bounding_box(&its, self.m_stats_mut());
            self.its = its;
        }
    }
}

/// `sizeof(indexed_triangle_set)` at the pin (admesh/stl.h:220-266): three `std::vector`s
/// (`indices`, `vertices`, `properties`) of 24 bytes each on a 64-bit libc++.
const SIZEOF_INDEXED_TRIANGLE_SET: usize = 72;
/// `sizeof(stl_triangle_vertex_indices)` (`Vec3i`).
const SIZEOF_STL_TRIANGLE_VERTEX_INDICES: usize = 12;
/// `sizeof(FaceProperty)` (admesh/stl.h:172-218: an `int` enum and a `double`).
const SIZEOF_FACE_PROPERTY: usize = 16;
/// `sizeof(stl_vertex)` (`Vec3f`).
const SIZEOF_STL_VERTEX: usize = 12;
/// `sizeof(TriangleMeshStats)` (TriangleMesh.hpp:47-93): 61 bytes of metrics and remaining
/// errors, padded to 64, then `RepairedMeshErrors` (five `int`s).
const SIZEOF_TRIANGLE_MESH_STATS: usize = 84;

/// admesh/stl.h:230-232 `size_t indexed_triangle_set::memsize() const`; the crate's set has no
/// `properties`, which the pin counts at one per face.
pub fn its_memsize(its: &indexed_triangle_set) -> usize {
    SIZEOF_INDEXED_TRIANGLE_SET
        + (SIZEOF_STL_TRIANGLE_VERTEX_INDICES + SIZEOF_FACE_PROPERTY) * its.indices.len()
        + SIZEOF_STL_VERTEX * its.vertices.len()
}

/// TriangleMesh.hpp:111 `void translate(float x, float y, float z)`, the overload of
/// `translate` past `model::ModelMesh::translate(&Vec3f)`.
pub trait TriangleMeshTranslateXyz {
    fn translate(&mut self, x: f32, y: f32, z: f32);
}

impl TriangleMeshTranslateXyz for crate::model::ModelMesh {
    /// TriangleMesh.cpp:306-309
    fn translate(&mut self, x: f32, y: f32, z: f32) {
        crate::model::ModelMesh::translate(self, &Vec3f::new(x, y, z));
    }
}

/// TriangleMesh.hpp:109 `void scale(float factor)`, the overload of `scale` past
/// `model::ModelMesh::scale(&Vec3f)`.
pub trait TriangleMeshScaleUniform {
    fn scale(&mut self, factor: f32);
}

impl TriangleMeshScaleUniform for crate::model::ModelMesh {
    /// TriangleMesh.cpp:267-270
    fn scale(&mut self, factor: f32) {
        crate::model::ModelMesh::scale(self, &Vec3f::new(factor, factor, factor));
    }
}

/// TriangleMesh.hpp:139 `BoundingBoxf3 transformed_bounding_box(const Transform3d& trafo, double world_min_z) const`,
/// the overload past `model::ModelMesh::transformed_bounding_box(&Transform3d)`.
pub trait TriangleMeshTransformedBoundingBoxAbove {
    fn transformed_bounding_box(&self, trafod: &nalgebra::Matrix4<f64>, world_min_z: f64) -> crate::bounding_box::BoundingBoxf3;
}

impl TriangleMeshTransformedBoundingBoxAbove for crate::model::ModelMesh {
    /// TriangleMesh.cpp:509-558
    fn transformed_bounding_box(&self, trafod: &nalgebra::Matrix4<f64>, world_min_z: f64) -> crate::bounding_box::BoundingBoxf3 {
        let its = &self.its;
        // TriangleMesh.cpp:511-517 — 1) Allocate transformed vertices with their position with respect to print bed surface.
        let mut sides: Vec<i8> = Vec::with_capacity(its.vertices.len());
        let mut num_above: usize = 0;
        // Eigen::AlignedBox<float, 3>: empty is (max float, lowest float).
        let mut bbox_min = Vec3f::new(f32::MAX, f32::MAX, f32::MAX);
        let mut bbox_max = Vec3f::new(f32::MIN, f32::MIN, f32::MIN);
        let extend = |bmin: &mut Vec3f, bmax: &mut Vec3f, p: &Vec3f| {
            // AlignedBox::extend: m_min = m_min.cwiseMin(p), m_max = m_max.cwiseMax(p)
            for k in 0..3 {
                bmin[k] = if p[k] < bmin[k] { p[k] } else { bmin[k] };
                bmax[k] = if bmax[k] < p[k] { p[k] } else { bmax[k] };
            }
        };
        // Transform3f trafo = trafod.cast<float>()
        let trafo: nalgebra::Matrix4<f32> = trafod.map(|v| v as f32);
        // TriangleMesh.cpp:518-527
        for v in &its.vertices {
            let pt = eigen_transform3f_mul_vec(&trafo, v);
            // pt.z() > world_min_z: the float promoted to double
            let sign: i8 = if (pt.z as f64) > world_min_z {
                1
            } else if (pt.z as f64) < world_min_z {
                -1
            } else {
                0
            };
            sides.push(sign);
            if sign >= 0 {
                // Vertex above or on print bed surface. Test whether it is inside the build volume.
                num_above += 1;
                extend(&mut bbox_min, &mut bbox_max, &pt);
            }
        }

        // TriangleMesh.cpp:529-549 — 2) Calculate intersections of triangle edges with the build surface.
        if num_above < its.vertices.len() {
            for tri in &its.indices {
                let s: [i32; 3] = [sides[tri[0] as usize] as i32, sides[tri[1] as usize] as i32, sides[tri[2] as usize] as i32];
                if s[0].min(s[1].min(s[2])) < 0 && s[0].max(s[1].max(s[2])) > 0 {
                    // Some edge of this triangle intersects the build platform. Calculate the intersection.
                    let mut iprev = 2usize;
                    for iedge in 0..3usize {
                        if s[iprev] * s[iedge] == -1 {
                            // edge intersects the build surface. Calculate intersection point.
                            let p1 = eigen_transform3f_mul_vec(&trafo, &its.vertices[tri[iprev] as usize]);
                            let p2 = eigen_transform3f_mul_vec(&trafo, &its.vertices[tri[iedge] as usize]);
                            // TriangleMesh.cpp:543: `(world_min_z - p1.z()) / (p2.z() - p1.z())`, double, stored as float
                            let t: f32 = ((world_min_z - p1.z as f64) / ((p2.z - p1.z) as f64)) as f32;
                            // TriangleMesh.cpp:544: `p1.x() + (p2.x() - p1.x()) * t` (fused), z = float(world_min_z)
                            extend(
                                &mut bbox_min,
                                &mut bbox_max,
                                &Vec3f::new((p2.x - p1.x).mul_add(t, p1.x), (p2.y - p1.y).mul_add(t, p1.y), world_min_z as f32),
                            );
                        }
                        iprev = iedge;
                    }
                }
            }
        }

        // TriangleMesh.cpp:551-557
        let mut out = crate::bounding_box::BoundingBoxf3::new();
        // AlignedBox::isEmpty(): (m_min.array() > m_max.array()).any()
        let is_empty = (0..3).any(|k| bbox_min[k] > bbox_max[k]);
        if !is_empty {
            out.min = crate::geometry::Vec3d::new(bbox_min.x as f64, bbox_min.y as f64, bbox_min.z as f64);
            out.max = crate::geometry::Vec3d::new(bbox_max.x as f64, bbox_max.y as f64, bbox_max.z as f64);
            out.defined = true;
        }
        out
    }
}

impl crate::model::ModelMesh {
    /// TriangleMesh.hpp:119 `void mirror_x() { this->mirror(X); }`
    pub fn mirror_x(&mut self) {
        self.mirror(crate::geometry::geometry::Axis::X);
    }

    /// TriangleMesh.hpp:120 `void mirror_y() { this->mirror(Y); }`
    pub fn mirror_y(&mut self) {
        self.mirror(crate::geometry::geometry::Axis::Y);
    }

    /// TriangleMesh.hpp:121 `void mirror_z() { this->mirror(Z); }`
    pub fn mirror_z(&mut self) {
        self.mirror(crate::geometry::geometry::Axis::Z);
    }

    /// TriangleMesh.hpp:115 `void rotate_x(float angle) { this->rotate(angle, X); }`
    pub fn rotate_x(&mut self, angle: f32) {
        TriangleMeshRotateAxis::rotate(self, angle, crate::geometry::geometry::Axis::X);
    }

    /// TriangleMesh.hpp:116 `void rotate_y(float angle) { this->rotate(angle, Y); }`
    pub fn rotate_y(&mut self, angle: f32) {
        TriangleMeshRotateAxis::rotate(self, angle, crate::geometry::geometry::Axis::Y);
    }

    /// TriangleMesh.hpp:117 `void rotate_z(float angle) { this->rotate(angle, Z); }`
    pub fn rotate_z(&mut self, angle: f32) {
        TriangleMeshRotateAxis::rotate(self, angle, crate::geometry::geometry::Axis::Z);
    }

    /// TriangleMesh.cpp:245-248 `bool write_ascii(const char* output_file)`
    pub fn write_ascii(&mut self, output_file: &str) -> bool {
        its_write_stl_ascii_its::its_write_stl_ascii(output_file, "", &self.its)
    }

    /// TriangleMesh.cpp:250-253 `bool write_binary(const char* output_file)`
    pub fn write_binary(&mut self, output_file: &str) -> bool {
        its_write_stl_binary_its::its_write_stl_binary(output_file, "", &self.its)
    }

    /// TriangleMesh.cpp:260-263 `void WriteOBJFile(const char* output_file) const`
    pub fn write_obj_file(&self, output_file: &str) {
        // The C++ discards the result of its_write_obj.
        let _ = its_write_obj(&self.its, output_file);
    }

    /// TriangleMesh.cpp:401-404 `void align_to_origin()`
    pub fn align_to_origin(&mut self) {
        let min = self.stats().min;
        TriangleMeshTranslateXyz::translate(self, -min[0], -min[1], -min[2]);
    }

    /// TriangleMesh.cpp:406-414 `void rotate(double angle, Point* center)`: about `center`, the
    /// angle in degrees (admesh's `its_rotate_z`), the statistics left as they are past the two
    /// translations.
    pub fn rotate(&mut self, angle: f64, center: &crate::geometry::Point) {
        if angle != 0.0 {
            // Vec2f c = center->cast<float>()
            let c = Vec2f::new(center.x() as f32, center.y() as f32);
            TriangleMeshTranslateXyz::translate(self, -c[0], -c[1], 0.0);
            its_rotate_z(&mut self.its, angle as f32);
            TriangleMeshTranslateXyz::translate(self, c[0], c[1], 0.0);
        }
    }

    /// TriangleMesh.cpp:480-490 `Polygon convex_hull() const`: the 2D convex hull of the mesh
    /// projected into the Z=0 plane.
    pub fn convex_hull(&self) -> Polygon {
        // TriangleMesh.cpp:482-483
        let mut pp: Vec<Point> = Vec::with_capacity(self.its.vertices.len());
        // TriangleMesh.cpp:484-487 — Point::new_scale(v(0), v(1)): coord_t(double(v) / SCALING_FACTOR)
        for v in &self.its.vertices {
            pp.push(Point::new((v[0] as f64 / SCALING_FACTOR_PIN) as i64, (v[1] as f64 / SCALING_FACTOR_PIN) as i64));
        }
        // TriangleMesh.cpp:488
        convex_hull_points(pp)
    }

    /// TriangleMesh.cpp:568-590 `void extract_son_mesh(TriangleMesh &temp_mesh, int face_start, int face_end) const`:
    /// the faces `face_start..=face_end` as a triangle soup in `temp_mesh.its` (its statistics
    /// are not touched).
    pub fn extract_son_mesh(&self, temp_mesh: &mut crate::model::ModelMesh, face_start: i32, face_end: i32) {
        let its = &self.its;
        // TriangleMesh.cpp:570
        let mut temp_its = indexed_triangle_set::default();
        // TriangleMesh.cpp:571: `face_end < its.indices.size()` compares as size_t
        if face_end > face_start && (face_end as isize as usize) < its.indices.len() {
            // TriangleMesh.cpp:572-574
            let new_face_count = (face_end - face_start + 1) as usize;
            temp_its.vertices.reserve(new_face_count * 3);
            temp_its.indices.reserve(new_face_count);

            // TriangleMesh.cpp:576-587
            for i in face_start..=face_end {
                let face = its.indices[i as usize];
                let v0 = its.vertices[face[0] as usize];
                let v1 = its.vertices[face[1] as usize];
                let v2 = its.vertices[face[2] as usize];
                temp_its.vertices.push(v0);
                temp_its.vertices.push(v1);
                temp_its.vertices.push(v2);
                let new_i = i - face_start;
                let new_face = Vec3i::new(new_i * 3, new_i * 3 + 1, new_i * 3 + 2);
                temp_its.indices.push(new_face);
            }
            // TriangleMesh.cpp:588
            temp_mesh.its = temp_its;
        }
    }

    /// TriangleMesh.cpp:592-597 `std::vector<ExPolygons> slice(const std::vector<double>& z) const`:
    /// `slice_mesh_ex(its, z_f, 0.0004f)` (TriangleMeshSlicer.hpp:86-95, the closing radius).
    pub fn slice(&self, z: &[f64]) -> Vec<crate::geometry::ExPolygons> {
        // convert doubles to floats
        let z_f: Vec<f32> = z.iter().map(|&v| v as f32).collect();
        let mut params = crate::triangle_mesh_slicer::MeshSlicingParamsEx::default();
        params.closing_radius = 0.0004f32;
        crate::triangle_mesh_slicer::slice_mesh_ex_its(&self.its, &z_f, &params, &|| {})
    }

    /// TriangleMesh.cpp:599-603 `size_t memsize() const`: `8 + its.memsize() + sizeof(m_stats)`.
    pub fn memsize(&self) -> usize {
        8 + its_memsize(&self.its) + SIZEOF_TRIANGLE_MESH_STATS
    }

    /// TriangleMesh.hpp:162 `size_t release_optional() { return 0; }`: nothing is cached.
    pub fn release_optional(&mut self) -> usize {
        0
    }

    /// TriangleMesh.hpp:164 `void restore_optional() {}`
    pub fn restore_optional(&mut self) {}
}

/// libslic3r.h:58 `static constexpr double SCALING_FACTOR = 0.00001;` (`scale_(v)` divides by it).
const SCALING_FACTOR_PIN: f64 = 0.00001;

// ============================================================================
// DIVERGENCE: pre-existing application-wide TriangleMesh struct (see module doc).
// Kept intact so the ~24 dependent modules continue to build. NOT a faithful
// representation of the C++ `class TriangleMesh` (which wraps `indexed_triangle_set`
// + `TriangleMeshStats` with f32 vertices); reconciling that requires a crate-wide
// migration tracked separately.
// ============================================================================

/// Triangle face type classification.
///
/// admesh/stl.h:51-57
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(i32)]
pub enum EnumFaceTypes {
    /// normal face
    /// admesh/stl.h:52
    ENormal = 0,
    /// small overhang
    /// admesh/stl.h:53
    ESmallOverhang = 1,
    /// face with small hole
    /// admesh/stl.h:54
    ESmallHole = 2,
    /// exterior appearance
    /// admesh/stl.h:55
    EExteriorAppearance = 3,
    /// admesh/stl.h:56
    EMaxNumFaceTypes = 4,
}

/// Triangle face property.
///
/// admesh/stl.h:172-218
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct FaceProperty {
    /// admesh/stl.h:174
    pub type_: EnumFaceTypes,
    /// admesh/stl.h:175
    pub area: f64,
}

impl Default for FaceProperty {
    fn default() -> Self {
        // admesh/stl.h: properties default to eNormal with zero area.
        Self {
            type_: EnumFaceTypes::ENormal,
            area: 0.0,
        }
    }
}

/// Mesh statistics, mirroring the relevant subset of `stl_stats`.
///
/// admesh/stl.h:101-150
#[derive(Clone, Copy, Debug, Default)]
pub struct MeshStats {
    /// Should always match the number of facets stored inside the indexed
    /// triangle set's `indices`.
    /// admesh/stl.h:110
    pub number_of_facets: u32,
}

/// A single triangle defined by three vertex indices.
#[derive(Clone, Copy, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Triangle {
    /// Indices into the vertex array for the three corners.
    pub indices: [u32; 3],
}

impl Triangle {
    // Create a new triangle from vertex indices.
    #[inline]
    pub const fn new(v0: u32, v1: u32, v2: u32) -> Self {
        Self {
            indices: [v0, v1, v2],
        }
    }

    /// Get the vertex index at position i (0, 1, or 2).
    #[inline]
    pub fn vertex(&self, i: usize) -> u32 {
        self.indices[i]
    }

    /// Check if this triangle is degenerate (has duplicate vertices).
    #[inline]
    pub fn is_degenerate(&self) -> bool {
        self.indices[0] == self.indices[1]
            || self.indices[1] == self.indices[2]
            || self.indices[2] == self.indices[0]
    }
}

impl fmt::Debug for Triangle {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Triangle({}, {}, {})",
            self.indices[0], self.indices[1], self.indices[2]
        )
    }
}

impl From<[u32; 3]> for Triangle {
    #[inline]
    fn from(indices: [u32; 3]) -> Self {
        Self { indices }
    }
}

impl From<Triangle> for [u32; 3] {
    #[inline]
    fn from(tri: Triangle) -> Self {
        tri.indices
    }
}

/// A triangle with both vertex indices and resolved vertex positions.
/// Returned by [`TriangleMesh::triangles()`].
pub struct TriangleData {
    /// The vertex indices (into the mesh's vertex array).
    pub indices: [u32; 3],
    /// The resolved vertex positions.
    pub vertices: [Point3F; 3],
}

/// A 3D triangle mesh represented as an indexed triangle set.
///
/// This is the primary mesh representation used throughout the slicer,
/// mirroring BambuStudio's indexed_triangle_set structure.
#[derive(Clone, Default, Serialize, Deserialize)]
pub struct TriangleMesh {
    /// Vertex positions (in mm, floating-point).
    vertices: Vec<Point3F>,
    /// Triangle indices into the vertex array.
    indices: Vec<Triangle>,
    /// Per-face properties (face-type classification, area).
    /// admesh/stl.h:245 (`indexed_triangle_set::properties`)
    #[serde(default)]
    properties: Vec<FaceProperty>,
    /// Cached bounding box (lazily computed).
    #[serde(skip)]
    bounding_box: Option<BoundingBox3F>,
}

/// TriangleMesh.cpp:102-197 `trianglemesh_repair_on_import`.
pub fn trianglemesh_repair_on_import(stl: &mut crate::stl_repair::StlFile) {
    if stl.stats.number_of_facets == 0 {
        return;
    }
    crate::stl_repair::stl_check_facets_exact(stl);
    stl.stats.facets_w_1_bad_edge = stl.stats.connected_facets_2_edge - stl.stats.connected_facets_3_edge;
    stl.stats.facets_w_2_bad_edge = stl.stats.connected_facets_1_edge - stl.stats.connected_facets_2_edge;
    stl.stats.facets_w_3_bad_edge = stl.stats.number_of_facets as i32 - stl.stats.connected_facets_1_edge;
    // TriangleMesh.cpp:124-142
    let mut tolerance = stl.stats.shortest_edge;
    let increment = stl.stats.bounding_diameter / 10000.0f32;
    let iterations = 2;
    if stl.stats.connected_facets_3_edge < stl.stats.number_of_facets as i32 {
        for _ in 0..iterations {
            if stl.stats.connected_facets_3_edge < stl.stats.number_of_facets as i32 {
                crate::stl_repair::stl_check_facets_nearby(stl, tolerance);
                tolerance += increment;
            } else {
                break;
            }
        }
    }
    // TriangleMesh.cpp:146-152
    if stl.stats.connected_facets_3_edge < stl.stats.number_of_facets as i32 {
        crate::stl_repair::stl_remove_unconnected_facets(stl);
    }
    // TriangleMesh.cpp:167-187
    crate::stl_repair::stl_fix_normal_directions(stl);
    crate::stl_repair::stl_fix_normal_values(stl);
    crate::stl_repair::stl_calculate_volume(stl);
    crate::stl_repair::stl_verify_neighbors(stl);
    // TriangleMesh.cpp:190-191
    let nr_degenerated = stl.stats.degenerate_facets;
    if stl.stats.number_of_facets > 0 && nr_degenerated > 0 {
        crate::stl_repair::stl_check_facets_exact(stl);
    }
}

impl TriangleMesh {
    /// TriangleMesh.cpp:204-235 `bool TriangleMesh::from_stl(stl_file& stl, bool repair)`: admesh's
    /// import repair, `stl_generate_shared_vertices` (shared.cpp:34-122) and the flip when
    /// `its_volume(its) < 0` (TriangleMesh.cpp:231-232).
    pub fn from_stl(&mut self, stl: &mut crate::stl_repair::StlFile, repair: bool) -> bool {
        if repair {
            trianglemesh_repair_on_import(stl);
        }
        let (vertices, mut indices) = crate::stl_repair::stl_generate_shared_vertices(stl);
        let its = indexed_triangle_set {
            vertices: vertices.iter().map(|v| StlVertex::new(v[0], v[1], v[2])).collect(),
            indices: indices.iter().map(|t| StlTriangleVertexIndices::new(t[0], t[1], t[2])).collect(),
        };
        if (its_volume(&its) as f64) < 0.0 {
            // its_flip_triangles: std::swap(face(1), face(2))
            for t in indices.iter_mut() {
                t.swap(1, 2);
            }
        }
        *self = TriangleMesh::from_parts(
            vertices.iter().map(|v| Point3F::new(v[0] as f64, v[1] as f64, v[2] as f64)).collect(),
            indices.iter().map(|t| Triangle::new(t[0] as u32, t[1] as u32, t[2] as u32)).collect(),
        );
        true
    }
}

impl TriangleMesh {
    // Create a new empty mesh.
    #[inline]
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
            properties: Vec::new(),
            bounding_box: None,
        }
    }

    /// Create a mesh with preallocated capacity.
    pub fn with_capacity(vertex_count: usize, triangle_count: usize) -> Self {
        Self {
            vertices: Vec::with_capacity(vertex_count),
            indices: Vec::with_capacity(triangle_count),
            properties: Vec::with_capacity(triangle_count),
            bounding_box: None,
        }
    }

    /// Create a mesh from vertices and indices.
    pub fn from_parts(vertices: Vec<Point3F>, indices: Vec<Triangle>) -> Self {
        // admesh/stl.h:224 — indexed_triangle_set sizes `properties` to match `indices`.
        let properties = vec![FaceProperty::default(); indices.len()];
        Self {
            vertices,
            indices,
            properties,
            bounding_box: None,
        }
    }

    /// Get the vertices of the mesh.
    #[inline]
    pub fn vertices(&self) -> &[Point3F] {
        &self.vertices
    }

    /// Get mutable access to the vertices.
    #[inline]
    pub fn vertices_mut(&mut self) -> &mut Vec<Point3F> {
        self.bounding_box = None; // Invalidate cache
        &mut self.vertices
    }

    /// Get the triangle indices.
    #[inline]
    pub fn indices(&self) -> &[Triangle] {
        &self.indices
    }

    /// Get mutable access to the triangle indices.
    #[inline]
    pub fn indices_mut(&mut self) -> &mut Vec<Triangle> {
        &mut self.indices
    }

    /// Get the number of vertices.
    #[inline]
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Get the number of triangles.
    #[inline]
    pub fn triangle_count(&self) -> usize {
        self.indices.len()
    }

    /// Check if the mesh is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.indices.is_empty()
    }

    /// Add a vertex and return its index.
    pub fn add_vertex(&mut self, v: Point3F) -> u32 {
        let idx = self.vertices.len() as u32;
        self.vertices.push(v);
        self.bounding_box = None;
        idx
    }

    /// Add a triangle.
    pub fn add_triangle(&mut self, tri: Triangle) {
        self.indices.push(tri);
        self.properties.push(FaceProperty::default());
    }

    /// Add a triangle from vertex indices.
    pub fn add_triangle_indices(&mut self, v0: u32, v1: u32, v2: u32) {
        self.indices.push(Triangle::new(v0, v1, v2));
        self.properties.push(FaceProperty::default());
    }

    /// Get a vertex by index.
    #[inline]
    pub fn vertex(&self, idx: u32) -> Point3F {
        self.vertices[idx as usize]
    }

    /// Get the three vertices of a triangle.
    #[inline]
    pub fn triangle_vertices(&self, tri_idx: usize) -> [Point3F; 3] {
        let tri = &self.indices[tri_idx];
        [
            self.vertices[tri.indices[0] as usize],
            self.vertices[tri.indices[1] as usize],
            self.vertices[tri.indices[2] as usize],
        ]
    }

    /// Get the vertex indices of a triangle.
    #[inline]
    pub fn triangle_indices(&self, tri_idx: usize) -> [u32; 3] {
        self.indices[tri_idx].indices
    }

    /// Iterate over triangles, yielding both indices and resolved vertices.
    /// This matches the libslic3r pattern of iterating over triangle data.
    pub fn triangles(&self) -> impl Iterator<Item = TriangleData> + '_ {
        (0..self.triangle_count()).map(move |i| TriangleData {
            indices: self.triangle_indices(i),
            vertices: self.triangle_vertices(i),
        })
    }

    /// Get the bounding box of the mesh.
    pub fn bounding_box(&mut self) -> BoundingBox3F {
        if let Some(bb) = self.bounding_box {
            return bb;
        }

        let mut bb = BoundingBox3F::new();
        for v in &self.vertices {
            bb.merge_point(*v);
        }
        self.bounding_box = Some(bb);
        bb
    }

    /// Get the bounding box without caching (const method).
    pub fn compute_bounding_box(&self) -> BoundingBox3F {
        let mut bb = BoundingBox3F::new();
        for v in &self.vertices {
            bb.merge_point(*v);
        }
        bb
    }

    /// Get mesh statistics.
    ///
    /// Mirrors C++ `TriangleMesh::stats()` for the `number_of_facets` field
    /// used by FaceDetector. admesh/stl.h:110
    #[inline]
    pub fn stats(&self) -> MeshStats {
        MeshStats {
            // admesh/stl.h:110 — number_of_facets matches indices count.
            number_of_facets: self.indices.len() as u32,
        }
    }

    /// Get the per-face property for a facet, growing the property store to
    /// match `indices` if it is out of sync.
    ///
    /// admesh/stl.h:255-261
    pub fn get_property(&mut self, face_idx: usize) -> &mut FaceProperty {
        // admesh/stl.h:256-259
        if self.properties.len() != self.indices.len() {
            self.properties.clear();
            self.properties
                .resize(self.indices.len(), FaceProperty::default());
        }
        // admesh/stl.h:260
        &mut self.properties[face_idx]
    }

    /// Apply an affine transform to every vertex.
    ///
    /// Mirrors C++ `TriangleMesh::transform(const Transform3d&)`, which maps
    /// each vertex through the matrix.
    pub fn transform(&mut self, trafo: &Transform3D) {
        for vertex in &mut self.vertices {
            *vertex = trafo.apply(*vertex);
        }
        self.bounding_box = None;
    }

    /// Merge another mesh into this one, appending its vertices, triangles
    /// (with shifted vertex indices), and per-face properties.
    ///
    /// Mirrors C++ `TriangleMesh::merge(TriangleMesh&&)`.
    pub fn merge(&mut self, mut other: TriangleMesh) {
        let vertex_offset = self.vertices.len() as u32;
        self.vertices.append(&mut other.vertices);
        for tri in &mut other.indices {
            tri.indices[0] += vertex_offset;
            tri.indices[1] += vertex_offset;
            tri.indices[2] += vertex_offset;
        }
        self.indices.append(&mut other.indices);
        // Append the incoming face properties, then keep the property store
        // aligned with the new triangle count.
        self.properties.append(&mut other.properties);
        if self.properties.len() != self.indices.len() {
            self.properties
                .resize(self.indices.len(), FaceProperty::default());
        }
        self.bounding_box = None;
    }

    /// Get the center of the mesh.
    pub fn center(&mut self) -> Point3F {
        self.bounding_box().center()
    }

    /// Get the size of the mesh (bounding box dimensions).
    pub fn size(&mut self) -> Point3F {
        self.bounding_box().size()
    }

    /// Calculate the normal of a triangle.
    pub fn triangle_normal(&self, tri_idx: usize) -> Point3F {
        let [v0, v1, v2] = self.triangle_vertices(tri_idx);
        let e1 = v1 - v0;
        let e2 = v2 - v0;
        e1.cross(&e2).normalize()
    }

    /// Calculate the area of a triangle.
    pub fn triangle_area(&self, tri_idx: usize) -> CoordF {
        let [v0, v1, v2] = self.triangle_vertices(tri_idx);
        let e1 = v1 - v0;
        let e2 = v2 - v0;
        e1.cross(&e2).length() / 2.0
    }

    /// Calculate the total surface area of the mesh.
    pub fn surface_area(&self) -> CoordF {
        let mut total = 0.0;
        for i in 0..self.indices.len() {
            total += self.triangle_area(i);
        }
        total
    }

    /// Calculate the volume of the mesh (assumes watertight mesh).
    /// Uses the signed volume formula based on tetrahedra from origin.
    pub fn volume(&self) -> CoordF {
        let mut total = 0.0;
        for tri in &self.indices {
            let v0 = self.vertices[tri.indices[0] as usize];
            let v1 = self.vertices[tri.indices[1] as usize];
            let v2 = self.vertices[tri.indices[2] as usize];

            // Signed volume of tetrahedron from origin to triangle
            total += v0.dot(&v1.cross(&v2)) / 6.0;
        }
        total.abs()
    }

    /// Translate the mesh by a vector.
    pub fn translate(&mut self, v: Point3F) {
        for vertex in &mut self.vertices {
            *vertex = *vertex + v;
        }
        self.bounding_box = None;
    }

    /// Replicate BambuStudio's f32 center-store / instance-place round trip on the
    /// vertices. C++ stores the ModelVolume mesh as f32 CENTERED on its bbox and the
    /// instance trafo translates it back at slice time (PrintObjectSlice.cpp:60). In
    /// EXACT math this is identity, but f32 is coarser away from the origin, so a vertex
    /// at f32(0.3) with a large center (Benchy center_z=24) round-trips to
    /// f32(f32(0.3 - 24) + 24) = 0.299999237 — ~21 ULPs of loss. Our pipeline stores
    /// vertices in f64 and only casts to f32 at slice time, so without this it keeps the
    /// exact f32(0.3) and sits bit-coincident with the layer-midpoint slice plane (the
    /// degenerate on-plane facet case that emits spurious interior holes — R63-R65).
    /// Bake the f32 round trip into the f64 storage so the slice-time cast reproduces
    /// C++'s coordinates. Net-zero for geometry not sitting exactly on a slice plane.
    pub fn quantize_f32_center_roundtrip(&mut self) {
        let c = self.center();
        let (cx, cy, cz) = (c.x as f32, c.y as f32, c.z as f32);
        for v in &mut self.vertices {
            v.x = ((v.x as f32 - cx) + cx) as f64;
            v.y = ((v.y as f32 - cy) + cy) as f64;
            v.z = ((v.z as f32 - cz) + cz) as f64;
        }
        self.bounding_box = None;
    }

    /// Faithful slice-time XY centering (FRAME_PAIR): C++ slices the mesh through
    /// `trafo_centered()` = `v - center_offset` in XY (Print.hpp:376), where
    /// center_offset = the raw bbox XY center (PrintObject.cpp:88). This SHIFTS the
    /// slice frame to C++'s (it does NOT re-add the XY center — that's what
    /// `quantize_f32_center_roundtrip` does for the net-identity Z floor fix).
    /// Z is left EXACTLY as the Z round-trip leaves it (R65 floor preserved).
    /// Returns the applied center (so the exporter can subtract the matching origin).
    /// Apply AFTER `quantize_f32_center_roundtrip` so the Z behavior is unchanged.
    pub fn slice_center_xy(&mut self) -> (f64, f64) {
        let c = self.center();
        // EXACT center_offset = unscale(Point::new_scale(center.xy)) — C++ quantizes
        // the f64 center to the coord_t grid with TRUNCATION (coord_t(scale_(x)) =
        // coord_t(x / SCALING_FACTOR), truncates toward zero), then unscales back.
        // rust's scale() uses round(), so do the truncation explicitly here.
        let cx = (c.x / crate::libslic3r::SCALING_FACTOR).trunc() * crate::libslic3r::SCALING_FACTOR;
        let cy = (c.y / crate::libslic3r::SCALING_FACTOR).trunc() * crate::libslic3r::SCALING_FACTOR;
        for v in &mut self.vertices {
            // Clean f64 subtract: verts are exact f32 values, center_offset is a
            // coord-grid multiple → the result stays exact f32 (bit-checked:
            // rust_vert − center == C++ slice vert exactly). Z untouched (R65).
            v.x -= cx;
            v.y -= cy;
        }
        self.bounding_box = None;
        (cx, cy)
    }

    /// Scale the mesh uniformly about the origin.
    pub fn scale(&mut self, factor: CoordF) {
        for vertex in &mut self.vertices {
            *vertex = *vertex * factor;
        }
        self.bounding_box = None;
    }

    /// Scale the mesh non-uniformly.
    pub fn scale_xyz(&mut self, sx: CoordF, sy: CoordF, sz: CoordF) {
        for vertex in &mut self.vertices {
            vertex.x *= sx;
            vertex.y *= sy;
            vertex.z *= sz;
        }
        self.bounding_box = None;
    }

    /// Center the mesh at the origin.
    pub fn center_at_origin(&mut self) {
        let center = self.center();
        self.translate(-center);
    }

    /// Place the mesh on the Z=0 plane (bottom touching).
    pub fn place_on_bed(&mut self) {
        let bb = self.bounding_box();
        let offset = Point3F::new(0.0, 0.0, -bb.min.z);
        self.translate(offset);
    }

    /// Flip all triangle normals (reverse winding order).
    pub fn flip_normals(&mut self) {
        for tri in &mut self.indices {
            tri.indices.swap(0, 2);
        }
    }

    /// Remove degenerate triangles.
    pub fn remove_degenerate_triangles(&mut self) {
        self.indices.retain(|tri| !tri.is_degenerate());
    }

    /// Check if the mesh has any degenerate triangles.
    pub fn has_degenerate_triangles(&self) -> bool {
        self.indices.iter().any(|tri| tri.is_degenerate())
    }

    /// Merge vertices that are within a tolerance distance.
    pub fn merge_close_vertices(&mut self, tolerance: CoordF) {
        if self.vertices.is_empty() {
            return;
        }

        let tolerance_sq = tolerance * tolerance;
        let mut vertex_map: Vec<u32> = (0..self.vertices.len() as u32).collect();
        let mut new_vertices: Vec<Point3F> = Vec::new();

        for (i, v) in self.vertices.iter().enumerate() {
            let mut found = false;
            for (j, nv) in new_vertices.iter().enumerate() {
                if v.distance_squared(nv) < tolerance_sq {
                    vertex_map[i] = j as u32;
                    found = true;
                    break;
                }
            }
            if !found {
                vertex_map[i] = new_vertices.len() as u32;
                new_vertices.push(*v);
            }
        }

        // Remap triangle indices
        for tri in &mut self.indices {
            for idx in &mut tri.indices {
                *idx = vertex_map[*idx as usize];
            }
        }

        self.vertices = new_vertices;
        self.bounding_box = None;
    }

    /// Remove unused vertices (vertices not referenced by any triangle).
    pub fn remove_unused_vertices(&mut self) {
        let mut used = vec![false; self.vertices.len()];
        for tri in &self.indices {
            for &idx in &tri.indices {
                used[idx as usize] = true;
            }
        }

        // Build mapping from old to new indices
        let mut new_indices: Vec<u32> = vec![0; self.vertices.len()];
        let mut new_vertices: Vec<Point3F> = Vec::new();
        for (i, &is_used) in used.iter().enumerate() {
            if is_used {
                new_indices[i] = new_vertices.len() as u32;
                new_vertices.push(self.vertices[i]);
            }
        }

        // Remap triangle indices
        for tri in &mut self.indices {
            for idx in &mut tri.indices {
                *idx = new_indices[*idx as usize];
            }
        }

        self.vertices = new_vertices;
        self.bounding_box = None;
    }

    /// Clear the mesh.
    pub fn clear(&mut self) {
        self.vertices.clear();
        self.indices.clear();
        self.bounding_box = None;
    }

    /// Reserve capacity for vertices and triangles.
    pub fn reserve(&mut self, vertex_count: usize, triangle_count: usize) {
        self.vertices.reserve(vertex_count);
        self.indices.reserve(triangle_count);
    }

    /// Validate the mesh (check for valid indices).
    pub fn validate(&self) -> Result<()> {
        let vertex_count = self.vertices.len() as u32;
        for (i, tri) in self.indices.iter().enumerate() {
            for &idx in &tri.indices {
                if idx >= vertex_count {
                    return Err(Error::Mesh(format!(
                        "Triangle {} has invalid vertex index {} (only {} vertices)",
                        i, idx, vertex_count
                    )));
                }
            }
        }
        Ok(())
    }

    /// Create a simple cube mesh for testing.
    pub fn cube(size: CoordF) -> Self {
        let half = size / 2.0;
        let vertices = vec![
            // Bottom face
            Point3F::new(-half, -half, -half),
            Point3F::new(half, -half, -half),
            Point3F::new(half, half, -half),
            Point3F::new(-half, half, -half),
            // Top face
            Point3F::new(-half, -half, half),
            Point3F::new(half, -half, half),
            Point3F::new(half, half, half),
            Point3F::new(-half, half, half),
        ];

        let indices = vec![
            // Bottom
            Triangle::new(0, 2, 1),
            Triangle::new(0, 3, 2),
            // Top
            Triangle::new(4, 5, 6),
            Triangle::new(4, 6, 7),
            // Front
            Triangle::new(0, 1, 5),
            Triangle::new(0, 5, 4),
            // Back
            Triangle::new(2, 3, 7),
            Triangle::new(2, 7, 6),
            // Left
            Triangle::new(0, 4, 7),
            Triangle::new(0, 7, 3),
            // Right
            Triangle::new(1, 2, 6),
            Triangle::new(1, 6, 5),
        ];

        Self::from_parts(vertices, indices)
    }
}

impl fmt::Debug for TriangleMesh {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "TriangleMesh({} vertices, {} triangles)",
            self.vertices.len(),
            self.indices.len()
        )
    }
}

impl fmt::Display for TriangleMesh {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "TriangleMesh: {} vertices, {} triangles",
            self.vertices.len(),
            self.indices.len()
        )?;
        if let Some(bb) = self.bounding_box {
            write!(f, ", bounds: {}", bb)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_triangle_new() {
        let tri = Triangle::new(0, 1, 2);
        assert_eq!(tri.indices[0], 0);
        assert_eq!(tri.indices[1], 1);
        assert_eq!(tri.indices[2], 2);
    }

    #[test]
    fn test_triangle_degenerate() {
        let good = Triangle::new(0, 1, 2);
        assert!(!good.is_degenerate());

        let bad1 = Triangle::new(0, 0, 2);
        assert!(bad1.is_degenerate());

        let bad2 = Triangle::new(0, 1, 0);
        assert!(bad2.is_degenerate());
    }

    #[test]
    fn test_mesh_new() {
        let mesh = TriangleMesh::new();
        assert!(mesh.is_empty());
        assert_eq!(mesh.vertex_count(), 0);
        assert_eq!(mesh.triangle_count(), 0);
    }

    #[test]
    fn test_mesh_cube() {
        let mut mesh = TriangleMesh::cube(10.0);
        assert_eq!(mesh.vertex_count(), 8);
        assert_eq!(mesh.triangle_count(), 12);

        let bb = mesh.bounding_box();
        assert!((bb.min.x - (-5.0)).abs() < 1e-10);
        assert!((bb.max.x - 5.0).abs() < 1e-10);
    }

    // ---- faithful free-function ports ----

    #[test]
    fn test_its_make_cube_counts() {
        // TriangleMesh.cpp:953-964 — 12 facets, 8 vertices.
        let its = its_make_cube(10.0, 20.0, 30.0);
        assert_eq!(its.indices.len(), 12);
        assert_eq!(its.vertices.len(), 8);
    }

    #[test]
    fn test_its_flip_triangles_swaps_1_2() {
        let mut its = its_make_cube(1.0, 1.0, 1.0);
        let before = its.indices[0];
        its_flip_triangles(&mut its);
        let after = its.indices[0];
        assert_eq!(before[0], after[0]);
        assert_eq!(before[1], after[2]);
        assert_eq!(before[2], after[1]);
    }

    #[test]
    fn test_its_remove_degenerate_faces() {
        let mut its = its_make_cube(1.0, 1.0, 1.0);
        // Append a degenerate face.
        its.indices.push(Vec3i::new(0, 0, 1));
        let removed = its_remove_degenerate_faces(&mut its, true);
        assert_eq!(removed, 1);
        assert_eq!(its.indices.len(), 12);
    }

    #[test]
    fn test_its_triangle_edge() {
        let tri = StlTriangleVertexIndices::new(5, 7, 9);
        assert_eq!(its_triangle_edge(&tri, 0), Vec2i::new(5, 7));
        assert_eq!(its_triangle_edge(&tri, 1), Vec2i::new(7, 9));
        assert_eq!(its_triangle_edge(&tri, 2), Vec2i::new(9, 5));
    }

    #[test]
    fn test_its_triangle_edge_index() {
        let tri = StlTriangleVertexIndices::new(5, 7, 9);
        assert_eq!(its_triangle_edge_index(&tri, &Vec2i::new(5, 7)), 0);
        assert_eq!(its_triangle_edge_index(&tri, &Vec2i::new(7, 9)), 1);
        assert_eq!(its_triangle_edge_index(&tri, &Vec2i::new(9, 5)), 2);
        assert_eq!(its_triangle_edge_index(&tri, &Vec2i::new(7, 5)), -1);
    }

    #[test]
    fn test_its_volume_cube_positive() {
        // A unit cube built by its_make_cube has positive volume magnitude ~1.
        let its = its_make_cube(1.0, 1.0, 1.0);
        let v = its_volume(&its);
        assert!((v.abs() - 1.0).abs() < 1e-4, "volume = {}", v);
    }

    #[test]
    fn test_its_merge_offsets_indices() {
        let mut a = its_make_cube(1.0, 1.0, 1.0);
        let b = its_make_cube(1.0, 1.0, 1.0);
        let n = a.vertices.len() as i32;
        let nf = a.indices.len();
        its_merge(&mut a, &b);
        assert_eq!(a.vertices.len(), 16);
        assert_eq!(a.indices.len(), 24);
        // First merged face indices are offset by N.
        assert_eq!(a.indices[nf][0], b.indices[0][0] + n);
    }

    #[test]
    fn test_vertex_face_index() {
        let its = its_make_cube(1.0, 1.0, 1.0);
        let vfi = VertexFaceIndex::new(&its);
        // Total scattered face references == 3 * number_of_facets.
        let mut total = 0usize;
        for v in 0..its.vertices.len() {
            total += vfi.count(v);
        }
        assert_eq!(total, 3 * its.indices.len());
    }

    #[test]
    fn test_normalize_c_exponent() {
        assert_eq!(normalize_c_exponent("1.00000000E0"), "1.00000000E+00");
        assert_eq!(normalize_c_exponent("1.23000000E-3"), "1.23000000E-03");
        assert_eq!(normalize_c_exponent("9.99000000E12"), "9.99000000E+12");
    }
}
