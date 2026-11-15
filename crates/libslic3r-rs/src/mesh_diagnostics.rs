//! Port of `libslic3r/MeshDiagnostics.hpp` and `MeshDiagnostics.cpp`.
//!
//! Counts of the defects of an indexed triangle set: open edges, non-manifold edges and
//! vertices, half-edges that run the same way, and whether faces are reversed.
//!
//! C++ reference: `references/BambuStudio/src/libslic3r/MeshDiagnostics.{hpp,cpp}`.

use crate::mesh_raycast::{RayMeshHit, TriangleRaycaster};
use crate::triangle_set_sampling::{indexed_triangle_set, Vec3d, Vec3f, Vec3i};

/// MeshDiagnostics.hpp:11-17
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct MeshDiagnosticStats {
    pub non_manifold_edges: usize,
    pub non_manifold_vertices: usize,
    pub open_edges: usize,
    pub same_direction_edges: usize,
    pub has_reversed_faces: bool,
}

/// MeshDiagnostics.cpp:20-23
#[derive(Default, Clone)]
struct VertexFan {
    faces: Vec<usize>,
    parent: Vec<usize>,
}

/// MeshDiagnostics.cpp:25-31
#[derive(Clone, Copy)]
struct EdgeRef {
    v0: usize,
    v1: usize,
    v0_fan_idx: usize,
    v1_fan_idx: usize,
    /// true if the face winding is v0 -> v1
    forward: bool,
}

/// MeshDiagnostics.cpp:33-46
fn fan_root(parent: &mut [usize], mut idx: usize) -> usize {
    let mut root = idx;
    while parent[root] != root {
        root = parent[root];
    }

    while parent[idx] != idx {
        let next = parent[idx];
        parent[idx] = root;
        idx = next;
    }

    root
}

/// MeshDiagnostics.cpp:48-54
fn fan_union(fan: &mut VertexFan, a: usize, b: usize) {
    let root_a = fan_root(&mut fan.parent, a);
    let root_b = fan_root(&mut fan.parent, b);
    if root_a != root_b {
        fan.parent[root_b] = root_a;
    }
}

/// MeshDiagnostics.cpp:56-63
fn face_vertices_valid(its: &indexed_triangle_set, idx: &Vec3i) -> bool {
    let n = its.vertices.len();
    idx[0] >= 0
        && idx[1] >= 0
        && idx[2] >= 0
        && (idx[0] as usize) < n
        && (idx[1] as usize) < n
        && (idx[2] as usize) < n
}

/// MeshDiagnostics.cpp:65-71
///
/// A hit whose smallest barycentric coordinate is this close to 0 sits on a
/// triangle edge or vertex, where neighbouring faces answer the same query
/// differently. 0.02 stays below the interior barycentric of known
/// reversed-face samples (~0.06) while covering those near-edge false hits.
const K_BARY_EPS: f64 = 0.02;

/// `std::min(a, b)` for doubles: `(b < a) ? b : a`.
fn std_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

/// `std::max(a, b)` for doubles: `(a < b) ? b : a`.
fn std_max(a: f64, b: f64) -> f64 {
    if a < b {
        b
    } else {
        a
    }
}

/// MeshDiagnostics.cpp:73-79
///
/// True if the hit lies on a triangle edge or vertex (barycentric coord ~ 0).
/// RayMeshHit uses P = (1-u-v)*v0 + u*v1 + v*v2.
fn hit_on_edge_or_vertex(hit: &RayMeshHit, bary_eps: f64) -> bool {
    let w = 1.0 - hit.u - hit.v;
    std_min(hit.u, std_min(hit.v, w)) <= bary_eps
}

// The Eigen operations of detect_visible_backfaces on `Vec3d`, as the reference build rounds
// them (clang, -ffp-contract=on, the Eigen `eigen-transform-sys` is built with): the sum of
// `norm()` is `(x*x + y*y) + z*z` with no fused step, `dot()` is `fma(a.z, b.z, a.x*b.x + a.y*b.y)`,
// a component of `cross()` is `fma(a, b, -(c * d))`, and `a + b * s`, `a / s`, `s * (a + b)` are
// computed per component as written, nothing fused.

/// `Vec3d::norm()`
fn norm(a: &Vec3d) -> f64 {
    let xx = a[0] * a[0];
    let yy = a[1] * a[1];
    let zz = a[2] * a[2];
    let xy = xx + yy;
    (xy + zz).sqrt()
}

/// `Vec3d::dot()`
fn dot(a: &Vec3d, b: &Vec3d) -> f64 {
    let xy = a[0] * b[0] + a[1] * b[1];
    a[2].mul_add(b[2], xy)
}

/// `Vec3d::cross()`
fn cross(a: &Vec3d, b: &Vec3d) -> Vec3d {
    Vec3d::new(
        a[1].mul_add(b[2], -(a[2] * b[1])),
        a[2].mul_add(b[0], -(a[0] * b[2])),
        a[0].mul_add(b[1], -(a[1] * b[0])),
    )
}

/// `a + b * s`
fn add_scaled(a: &Vec3d, b: &Vec3d, s: f64) -> Vec3d {
    let p = [b[0] * s, b[1] * s, b[2] * s];
    Vec3d::new(a[0] + p[0], a[1] + p[1], a[2] + p[2])
}

/// `Vec3f::cast<double>()`
fn cast_double(v: &Vec3f) -> Vec3d {
    Vec3d::new(f64::from(v[0]), f64::from(v[1]), f64::from(v[2]))
}

/// MeshDiagnostics.cpp:81-267
///
/// Visible backfaces from outside the AABB. The caller must pass a watertight
/// mesh (no open edges, no non-manifold edges). Per ray: all hits. A closed
/// mesh must produce an even hit count (enter/leave pairs); an odd count is
/// discarded as a leak. The whole ray is then discarded if the two smallest t
/// differ by less than k_coincident_t, or if its outermost hit is a sliver,
/// lands on an edge / vertex, or is grazing. Only the outermost surviving hit
/// is tested; one back-facing hit sets the flag.
fn detect_visible_backfaces(its: &indexed_triangle_set) -> bool {
    if its.indices.is_empty() || its.vertices.is_empty() {
        return false;
    }

    // Skip on malformed indices; later face lookups index vertices without a
    // bounds check. Face count is not a reason to skip.
    for idx in &its.indices {
        if !face_vertices_valid(its, idx) {
            log::info!("reversed-faces: layer3 skip invalid_index");
            return false;
        }
    }

    let mut bmin_f: Vec3f = its.vertices[0];
    let mut bmax_f: Vec3f = bmin_f;
    for v in &its.vertices {
        for k in 0..3 {
            // cwiseMin / cwiseMax
            if v[k] < bmin_f[k] {
                bmin_f[k] = v[k];
            }
            if bmax_f[k] < v[k] {
                bmax_f[k] = v[k];
            }
        }
    }

    let bmin: Vec3d = cast_double(&bmin_f);
    let bmax: Vec3d = cast_double(&bmax_f);
    let center = Vec3d::new(0.5 * (bmin[0] + bmax[0]), 0.5 * (bmin[1] + bmax[1]), 0.5 * (bmin[2] + bmax[2]));
    let extent = Vec3d::new(bmax[0] - bmin[0], bmax[1] - bmin[1], bmax[2] - bmin[2]);
    let radius: f64 = 0.5 * norm(&extent);
    if radius <= 0.0 {
        log::info!("reversed-faces: layer3 skip zero_radius");
        return false;
    }
    // `extent.maxCoeff()`
    let mut max_ext: f64 = extent[0];
    for k in 1..3 {
        if extent[k] > max_ext {
            max_ext = extent[k];
        }
    }
    let standoff: f64 = radius + std_max(max_ext * 0.05, 1.0);

    log::info!(
        "reversed-faces: layer3 aabb extent=[{},{},{}] faces={} verts={}",
        extent[0],
        extent[1],
        extent[2],
        its.indices.len(),
        its.vertices.len()
    );

    let caster = TriangleRaycaster::new(its);

    let mut dirs: Vec<Vec3d> = Vec::with_capacity(46);
    for x in -1..=1 {
        for y in -1..=1 {
            for z in -1..=1 {
                if x == 0 && y == 0 && z == 0 {
                    continue;
                }
                dirs.push(Vec3d::new(f64::from(x), f64::from(y), f64::from(z)));
            }
        }
    }
    const EXTRA: i32 = 20;
    const GOLDEN: f64 = 2.399963229728653;
    for i in 0..EXTRA {
        let y: f64 = 1.0 - 2.0 * (f64::from(i) + 0.5) / f64::from(EXTRA);
        let r: f64 = std_max(0.0, (-y).mul_add(y, 1.0)).sqrt();
        let phi: f64 = f64::from(i) * GOLDEN;
        dirs.push(Vec3d::new(r * phi.cos(), y, r * phi.sin()));
    }

    const GRAZE: f64 = 0.08;
    const BACKFACE: f64 = 1e-4;
    const K_LOG_BACK_HITS: i32 = 8;
    // Two crossings closer than this are treated as coincident faces. Relative
    // to the AABB so a 2 mm part and a 300 mm part do not share one absolute
    // threshold; the factor keeps a typical 100 mm part near the former 0.01 mm.
    let k_coincident_t: f64 = radius * 1e-4;
    // nlen = ||(b-a)×(c-a)|| = 2 * area. Drop slivers whose area is tiny
    // relative to the AABB; their normals are not reliable.
    let min_nlen: f64 = 2.0 * radius * radius * 1e-12;

    let (mut n_ray, mut n_miss, mut n_odd, mut n_coincident, mut n_boundary) = (0i32, 0i32, 0i32, 0i32, 0i32);
    let (mut n_graze, mut n_sliver, mut n_front, mut n_back, mut n_coplanar) = (0i32, 0i32, 0i32, 0i32, 0i32);

    for mut dir in dirs {
        let len = norm(&dir);
        if len == 0.0 {
            continue;
        }
        dir = Vec3d::new(dir[0] / len, dir[1] / len, dir[2] / len);
        n_ray += 1;
        let origin: Vec3d = add_scaled(&center, &dir, standoff);
        let ray_dir: Vec3d = Vec3d::new(-dir[0], -dir[1], -dir[2]);

        // Coplanar crossings yield no hit, so they can break the parity check
        // below; count them to explain an elevated odd count in the log.
        let mut ray_coplanar: i32 = 0;
        let mut hits: Vec<RayMeshHit> = caster.all_hits(&origin, &ray_dir, Some(&mut ray_coplanar));
        n_coplanar += ray_coplanar;
        hits.retain(|h| !(h.t < 1e-6));
        if hits.is_empty() {
            n_miss += 1;
            continue;
        }
        // Closed meshes must enter and leave in pairs; an odd hit count is a leak.
        if hits.len() % 2 == 1 {
            n_odd += 1;
            continue;
        }

        if hits[1].t - hits[0].t < k_coincident_t {
            n_coincident += 1;
            continue;
        }

        let hit = &hits[0];
        if hit.face < 0 || hit.face as usize >= its.indices.len() {
            n_miss += 1;
            continue;
        }

        let idx = &its.indices[hit.face as usize];
        if !face_vertices_valid(its, idx) {
            n_miss += 1;
            continue;
        }

        let a: Vec3d = cast_double(&its.vertices[idx[0] as usize]);
        let b: Vec3d = cast_double(&its.vertices[idx[1] as usize]);
        let c: Vec3d = cast_double(&its.vertices[idx[2] as usize]);
        let mut n: Vec3d = cross(
            &Vec3d::new(b[0] - a[0], b[1] - a[1], b[2] - a[2]),
            &Vec3d::new(c[0] - a[0], c[1] - a[1], c[2] - a[2]),
        );
        let nlen = norm(&n);
        if nlen < min_nlen {
            n_sliver += 1;
            continue;
        }
        n = Vec3d::new(n[0] / nlen, n[1] / nlen, n[2] / nlen);

        if hit_on_edge_or_vertex(hit, K_BARY_EPS) {
            n_boundary += 1;
            continue;
        }

        let n_ray_dot = dot(&n, &ray_dir);
        if n_ray_dot.abs() < GRAZE {
            n_graze += 1;
            continue;
        }

        let hit_pt: Vec3d = add_scaled(&origin, &ray_dir, hit.t);
        let v = Vec3d::new(origin[0] - hit_pt[0], origin[1] - hit_pt[1], origin[2] - hit_pt[2]);
        let ndot = dot(&n, &v);

        if ndot < -BACKFACE {
            n_back += 1;
            if n_back <= K_LOG_BACK_HITS {
                let w = 1.0 - hit.u - hit.v;
                log::info!(
                    "reversed-faces: layer3 backface#{} face={} t={} uvw=[{},{},{}] ndot={} n_dot_ray={} dir=[{},{},{}] N=[{},{},{}] hit=[{},{},{}]",
                    n_back,
                    hit.face,
                    hit.t,
                    hit.u,
                    hit.v,
                    w,
                    ndot,
                    n_ray_dot,
                    dir[0],
                    dir[1],
                    dir[2],
                    n[0],
                    n[1],
                    n[2],
                    hit_pt[0],
                    hit_pt[1],
                    hit_pt[2]
                );
            }
        } else {
            n_front += 1;
        }
    }

    log::info!(
        "reversed-faces: layer3 summary rays={n_ray} miss={n_miss} odd={n_odd} coplanar={n_coplanar} coincident={n_coincident} sliver={n_sliver} boundary={n_boundary} graze={n_graze} front={n_front} back={n_back}"
    );

    let flagged = n_back > 0;
    if !flagged {
        log::info!("reversed-faces: layer3 no visible backface");
    }
    flagged
}

/// MeshDiagnostics.cpp:269-290
fn log_reversed_face_decision(src: &str, its: &indexed_triangle_set, stats: &MeshDiagnosticStats) {
    log::info!(
        "reversed-faces: {} has_reversed_faces={} open_edges={} nm_edges={} same_dir={} faces={} verts={}",
        src,
        if stats.has_reversed_faces { 1 } else { 0 },
        stats.open_edges,
        stats.non_manifold_edges,
        stats.same_direction_edges,
        its.indices.len(),
        its.vertices.len()
    );
    if stats.has_reversed_faces {
        log::info!(
            "reversed-faces: FLAG src={} faces={} verts={} open_edges={} nm_edges={} same_dir={}",
            src,
            its.indices.len(),
            its.vertices.len(),
            stats.open_edges,
            stats.non_manifold_edges,
            stats.same_direction_edges
        );
    }
}

/// MeshDiagnostics.cpp:294-314
fn detect_reversed_faces_impl(its: &indexed_triangle_set, stats: &mut MeshDiagnosticStats) {
    if stats.non_manifold_edges != 0 || stats.same_direction_edges != 0 {
        stats.has_reversed_faces = true;
        log::info!(
            "reversed-faces: edge shortcut nm_edges={} same_dir={} open_edges={}",
            stats.non_manifold_edges,
            stats.same_direction_edges,
            stats.open_edges
        );
        log_reversed_face_decision("detect", its, stats);
        return;
    }
    if stats.open_edges != 0 {
        log::info!("reversed-faces: skip not_watertight open_edges={}", stats.open_edges);
        log_reversed_face_decision("detect", its, stats);
        return;
    }
    stats.has_reversed_faces = detect_visible_backfaces(its);
    log_reversed_face_decision("detect", its, stats);
}

/// MeshDiagnostics.cpp:316-319
///
/// Set has_reversed_faces from already-computed edge stats:
///   non-manifold edges or same-direction edges -> true, no rays;
///   open edges          -> leave the flag unchanged (not watertight);
///   otherwise (watertight) -> rays from outside the AABB.
pub fn its_detect_reversed_faces(its: &indexed_triangle_set, stats: &mut MeshDiagnosticStats) {
    detect_reversed_faces_impl(its, stats);
}

/// MeshDiagnostics.cpp:321-326
///
/// Import-path diagnostics: its_edge_diagnostics followed by
/// its_detect_reversed_faces. Skips non-manifold vertex detection;
/// non_manifold_vertices in the returned stats is always 0.
/// Optional neighbors is forwarded to its_edge_diagnostics.
pub fn its_quick_diagnostics(its: &indexed_triangle_set, neighbors: Option<&mut Vec<Vec3i>>) -> MeshDiagnosticStats {
    let mut stats = its_edge_diagnostics(its, neighbors);
    detect_reversed_faces_impl(its, &mut stats);
    stats
}

/// `static_cast<size_t>(int)`: the sign is extended, a negative index becomes a huge one.
fn index_as_size_t(v: i32) -> usize {
    v as isize as usize
}

/// MeshDiagnostics.cpp:328-455
///
/// Full diagnostics: open / non-manifold edges, non-manifold vertices, and the
/// same reversed-face test as its_detect_reversed_faces. Each defect is counted
/// exactly once regardless of how many anomalies overlap on it.
pub fn its_mesh_diagnostics(its: &indexed_triangle_set) -> MeshDiagnosticStats {
    let mut result = MeshDiagnosticStats::default();
    let num_vertices = its.vertices.len();
    let num_faces = its.indices.len();

    if num_faces == 0 {
        return result;
    }

    // --- Pass 1: build per-vertex face fans and flat edge refs --------------
    let mut vertex_fans: Vec<VertexFan> = vec![VertexFan::default(); num_vertices];
    let mut edge_refs: Vec<EdgeRef> = Vec::with_capacity(num_faces * 3);

    for fid in 0..num_faces {
        let face = &its.indices[fid];

        // Skip degenerate faces (two or more identical vertex indices).
        if face[0] == face[1] || face[1] == face[2] || face[2] == face[0] {
            continue;
        }

        let vertices: [usize; 3] = [index_as_size_t(face[0]), index_as_size_t(face[1]), index_as_size_t(face[2])];
        let mut fan_indices: [usize; 3] = [usize::MAX, usize::MAX, usize::MAX];

        for i in 0..3 {
            let vid = vertices[i];
            if vid >= num_vertices {
                continue;
            }

            fan_indices[i] = vertex_fans[vid].faces.len();
            vertex_fans[vid].faces.push(fid);
        }

        for i in 0..3 {
            let j = (i + 1) % 3;
            let va = vertices[i];
            let vb = vertices[j];

            if va >= num_vertices || vb >= num_vertices {
                continue;
            }

            if va < vb {
                edge_refs.push(EdgeRef { v0: va, v1: vb, v0_fan_idx: fan_indices[i], v1_fan_idx: fan_indices[j], forward: true });
            } else {
                edge_refs.push(EdgeRef { v0: vb, v1: va, v0_fan_idx: fan_indices[j], v1_fan_idx: fan_indices[i], forward: false });
            }
        }
    }

    // Initialize per-vertex union-find storage. Non-degenerate faces insert
    // each incident face only once per vertex, so no sort/unique pass is needed.
    for fan in &mut vertex_fans {
        fan.parent = (0..fan.faces.len()).collect();
    }

    // --- Edge classification (each undirected edge counted at most once) -----
    // Also mark vertices incident on non-manifold edges so that the vertex
    // fan-connectivity test below can skip them (same strategy as VCGlib).
    // Two-face edge groups connect those two face fans at both edge endpoints.
    let mut on_nm_edge: Vec<bool> = vec![false; num_vertices];

    // `std::sort` by (v0, v1). It is not stable, but nothing below depends on the order within a
    // group of equal edges: the counts do not, and for a group of two the union of the two fans is
    // the same whichever comes first (only which root survives changes, and only the number of
    // distinct roots is read).
    edge_refs.sort_by(|a, b| (a.v0, a.v1).cmp(&(b.v0, b.v1)));

    let mut i = 0;
    while i < edge_refs.len() {
        let mut j = i + 1;
        while j < edge_refs.len() && edge_refs[j].v0 == edge_refs[i].v0 && edge_refs[j].v1 == edge_refs[i].v1 {
            j += 1;
        }

        let edge_face_count = j - i;
        let (mut plus, mut minus) = (0usize, 0usize);
        for k in i..j {
            if edge_refs[k].forward {
                plus += 1;
            } else {
                minus += 1;
            }
        }
        if plus >= 2 || minus >= 2 {
            result.same_direction_edges += 1;
        }

        if edge_face_count == 1 {
            result.open_edges += 1;
        } else if edge_face_count == 2 {
            fan_union(&mut vertex_fans[edge_refs[i].v0], edge_refs[i].v0_fan_idx, edge_refs[i + 1].v0_fan_idx);
            fan_union(&mut vertex_fans[edge_refs[i].v1], edge_refs[i].v1_fan_idx, edge_refs[i + 1].v1_fan_idx);
        } else {
            result.non_manifold_edges += 1;
            on_nm_edge[edge_refs[i].v0] = true;
            on_nm_edge[edge_refs[i].v1] = true;
        }

        i = j;
    }

    // --- Pass 2: non-manifold vertex detection ------------------------------
    // A vertex is non-manifold if its incident faces form more than one
    // component when connected through regular two-face edges.
    // Vertices on non-manifold edges are excluded: an edge with >2 faces
    // cannot define a reliable two-face fan traversal, and these vertices are
    // already accounted for by non_manifold_edges.
    let mut roots: Vec<usize> = Vec::new();

    for vid in 0..num_vertices {
        if on_nm_edge[vid] {
            continue;
        }

        let fan = &mut vertex_fans[vid];
        if fan.faces.len() <= 1 {
            continue;
        }

        roots.clear();
        roots.reserve(fan.parent.len());
        for i in 0..fan.parent.len() {
            roots.push(fan_root(&mut fan.parent, i));
        }

        roots.sort_unstable();
        roots.dedup();
        // `std::unique(roots.begin(), roots.end()) != roots.begin() + 1`
        if roots.len() != 1 {
            result.non_manifold_vertices += 1;
        }
    }

    its_detect_reversed_faces(its, &mut result);
    result
}

/// MeshDiagnostics.cpp:471-486 — the local `struct HalfEdge` of its_edge_diagnostics. The derived
/// order is its `operator<`: key, then face, then slot, then forward.
#[derive(Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
struct HalfEdge {
    key: u64,
    face: u32,
    slot: u8,
    forward: u8,
}

/// MeshDiagnostics.cpp:457-558
///
/// Count open edges (undirected face count == 1), non-manifold edges
/// (undirected face count > 2), and same-direction half-edges (one
/// undirected edge with at least two half-edges in the same winding).
/// Does not inspect vertices, does not run ray tests, and leaves
/// has_reversed_faces false. If neighbors is given, it is resized to
/// the face count and filled with opposite-winding partners (-1 if none).
pub fn its_edge_diagnostics(its: &indexed_triangle_set, mut neighbors: Option<&mut Vec<Vec3i>>) -> MeshDiagnosticStats {
    let mut result = MeshDiagnosticStats::default();
    let num_vertices = its.vertices.len();
    let num_faces = its.indices.len();

    if num_faces == 0 {
        return result;
    }

    if let Some(n) = neighbors.as_deref_mut() {
        n.clear();
        n.resize(num_faces, Vec3i::new(-1, -1, -1));
        if num_faces > u32::MAX as usize {
            neighbors = None;
        }
    }

    let mut edges: Vec<HalfEdge> = Vec::with_capacity(num_faces * 3);

    for fid in 0..num_faces {
        let face = &its.indices[fid];

        if face[0] == face[1] || face[1] == face[2] || face[2] == face[0] {
            continue;
        }

        let v: [usize; 3] = [index_as_size_t(face[0]), index_as_size_t(face[1]), index_as_size_t(face[2])];

        for i in 0..3 {
            let (mut va, mut vb) = (v[i], v[(i + 1) % 3]);
            if va >= num_vertices || vb >= num_vertices {
                continue;
            }
            let forward: u8 = if va < vb { 1 } else { 0 };
            if forward == 0 {
                std::mem::swap(&mut va, &mut vb);
            }
            edges.push(HalfEdge { key: ((va as u64) << 32) | (vb as u64), face: fid as u32, slot: i as u8, forward });
        }
    }

    // `tbb::parallel_sort` with `HalfEdge::operator<`, a total order over all four members.
    edges.sort_unstable();

    let mut plus_ids: Vec<u32> = Vec::with_capacity(4);
    let mut minus_ids: Vec<u32> = Vec::with_capacity(4);

    let mut i = 0;
    while i < edges.len() {
        let mut j = i + 1;
        while j < edges.len() && edges[j].key == edges[i].key {
            j += 1;
        }

        let count = j - i;
        plus_ids.clear();
        minus_ids.clear();
        for k in i..j {
            if edges[k].forward != 0 {
                plus_ids.push(k as u32);
            } else {
                minus_ids.push(k as u32);
            }
        }
        if plus_ids.len() >= 2 || minus_ids.len() >= 2 {
            result.same_direction_edges += 1;
        }
        if count == 1 {
            result.open_edges += 1;
        } else if count > 2 {
            result.non_manifold_edges += 1;
        }

        if let Some(neighbors) = neighbors.as_deref_mut() {
            let n_pairs = plus_ids.len().min(minus_ids.len());
            for p in 0..n_pairs {
                let a = edges[plus_ids[p] as usize];
                let b = edges[minus_ids[p] as usize];
                neighbors[a.face as usize][a.slot as usize] = b.face as i32;
                neighbors[b.face as usize][b.slot as usize] = a.face as i32;
            }
        }

        i = j;
    }

    result
}
