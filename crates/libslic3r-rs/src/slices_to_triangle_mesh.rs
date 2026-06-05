//! Mesh reconstruction from 2D slices.
//!
//! C++ Reference:
//! - SlicesToTriangleMesh.hpp
//! - SlicesToTriangleMesh.cpp
//!
//! This is a faithful 1:1 line-by-line port of BambuStudio's
//! `SlicesToTriangleMesh.cpp`. It reconstructs a 3D triangle mesh from a stack
//! of 2D polygon slices: it creates vertical walls between consecutive layers
//! and triangulates the exposed horizontal (top/bottom/overhang) surfaces.

// SlicesToTriangleMesh.cpp:6  #include "libslic3r/Execution/ExecutionTBB.hpp"
// SlicesToTriangleMesh.cpp:7  #include "libslic3r/ClipperUtils.hpp"
use crate::clipper_utils::difference;
// SlicesToTriangleMesh.cpp:8  #include "libslic3r/Tesselate.hpp"
use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon};
use crate::tesselate::{self, Vec3d, NORMALS_DOWN, NORMALS_UP};

/// Simple 3D vertex (`float`) for mesh construction.
///
/// Mirrors the `Vec3f` element type of `indexed_triangle_set::vertices`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3f {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3f {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
}

/// Indexed triangle set for mesh representation.
///
/// Mirrors libslic3r's `indexed_triangle_set` (admesh/stl.h): a list of
/// `Vec3f` vertices and a list of triangles, each storing three vertex
/// indices.
#[derive(Debug, Clone, Default)]
pub struct IndexedTriangleSet {
    pub vertices: Vec<Vec3f>,
    pub indices: Vec<[usize; 3]>,
}

impl IndexedTriangleSet {
    pub fn new() -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
        }
    }
}

// =============================================================================
// indexed_triangle_set helpers (TriangleMesh.cpp).
//
// These are dependencies of slices_to_mesh that are not yet hosted in
// triangle_mesh.rs (which currently models a separate `TriangleMesh` type with
// `Point3F`/`Triangle`). They are ported faithfully here against the local
// `IndexedTriangleSet`/`Vec3f` so this file matches the C++ behaviour exactly.
// =============================================================================

/// TriangleMesh.cpp:1796 — `void its_merge(indexed_triangle_set &A, const indexed_triangle_set &B)`
fn its_merge(a: &mut IndexedTriangleSet, b: &IndexedTriangleSet) {
    // TriangleMesh.cpp:1798
    let n = a.vertices.len() as i64;
    // TriangleMesh.cpp:1799
    let n_f = a.indices.len();

    // TriangleMesh.cpp:1801
    a.vertices.extend_from_slice(&b.vertices);
    // TriangleMesh.cpp:1802
    a.indices.extend_from_slice(&b.indices);

    // TriangleMesh.cpp:1804-1805
    for n_idx in n_f..a.indices.len() {
        a.indices[n_idx][0] = (a.indices[n_idx][0] as i64 + n) as usize;
        a.indices[n_idx][1] = (a.indices[n_idx][1] as i64 + n) as usize;
        a.indices[n_idx][2] = (a.indices[n_idx][2] as i64 + n) as usize;
    }
}

/// TriangleMesh.cpp:1808 — `void its_merge(indexed_triangle_set &A, const std::vector<Vec3f> &triangles)`
fn its_merge_vec3f(a: &mut IndexedTriangleSet, triangles: &[Vec3f]) {
    // TriangleMesh.cpp:1810
    let offs = a.vertices.len();
    // TriangleMesh.cpp:1811
    a.vertices.extend_from_slice(triangles);
    // TriangleMesh.cpp:1812
    a.indices.reserve(a.indices.len() + a.vertices.len() / 3);

    // TriangleMesh.cpp:1814-1815
    let mut i = offs;
    while i < a.vertices.len() {
        a.indices.push([i, i + 1, i + 2]);
        i += 3;
    }
}

/// TriangleMesh.cpp:1818 — `void its_merge(indexed_triangle_set &A, const Pointf3s &triangles)`
///
/// `Pointf3s == std::vector<Vec3d>`; each `Vec3d` is `cast<float>()`-ed to a
/// `Vec3f` before forwarding to the `Vec3f` overload.
fn its_merge_vec3d(a: &mut IndexedTriangleSet, triangles: &[Vec3d]) {
    // TriangleMesh.cpp:1820-1822
    let mut trianglesf: Vec<Vec3f> = Vec::with_capacity(triangles.len());
    for t in triangles {
        trianglesf.push(Vec3f::new(t.x as f32, t.y as f32, t.z as f32));
    }

    // TriangleMesh.cpp:1824
    its_merge_vec3f(a, &trianglesf);
}

/// TriangleMesh.cpp:728 — `int its_merge_vertices(indexed_triangle_set &its, bool shrink_to_fit)`
fn its_merge_vertices(its: &mut IndexedTriangleSet) -> i32 {
    // 1) Sort indices to vertices lexicographically by coordinates AND vertex index.
    // TriangleMesh.cpp:731-733
    let mut sorted: Vec<i32> = Vec::with_capacity(its.vertices.len());
    for i in 0..its.vertices.len() as i32 {
        sorted.push(i);
    }
    // TriangleMesh.cpp:734-739
    sorted.sort_by(|&il, &ir| {
        let l = its.vertices[il as usize];
        let r = its.vertices[ir as usize];
        // Sort lexicographically by coordinates AND vertex index.
        let less = l.x < r.x
            || (l.x == r.x
                && (l.y < r.y
                    || (l.y == r.y && (l.z < r.z || (l.z == r.z && il < ir)))));
        if less {
            std::cmp::Ordering::Less
        } else {
            std::cmp::Ordering::Greater
        }
    });

    // 2) Map duplicate vertices to the one with the lowest vertex index.
    // The vertex to stay will have a map_vertices[...] == -1 index assigned, the other vertices will point to it.
    // TriangleMesh.cpp:743
    let mut map_vertices: Vec<i32> = vec![-1; its.vertices.len()];
    // TriangleMesh.cpp:744
    let mut i = 0usize;
    while i < sorted.len() {
        // TriangleMesh.cpp:745-746
        let u = sorted[i];
        let p = its.vertices[u as usize];
        // TriangleMesh.cpp:747-748
        let mut j = i;
        j += 1;
        while j < sorted.len() {
            // TriangleMesh.cpp:749-750
            let v = sorted[j];
            let q = its.vertices[v as usize];
            // TriangleMesh.cpp:751-752
            if p != q {
                break;
            }
            // TriangleMesh.cpp:753 assert(v > u);
            debug_assert!(v > u);
            // TriangleMesh.cpp:754
            map_vertices[v as usize] = u;
            j += 1;
        }
        // TriangleMesh.cpp:756
        i = j;
    }

    // 3) Shrink its.vertices, update map_vertices with the new vertex indices.
    // TriangleMesh.cpp:760
    let mut k = 0i32;
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
            // TriangleMesh.cpp:768 assert(map_vertices[i] < i);
            debug_assert!((map_vertices[i] as usize) < i);
            // TriangleMesh.cpp:769
            map_vertices[i] = map_vertices[map_vertices[i] as usize];
        }
    }

    // TriangleMesh.cpp:773
    let num_erased = its.vertices.len() as i32 - k;

    // TriangleMesh.cpp:775
    if num_erased != 0 {
        // Shrink the vertices.
        // TriangleMesh.cpp:777
        its.vertices.truncate(k as usize);
        // Remap face indices.
        // TriangleMesh.cpp:779-781
        for face in its.indices.iter_mut() {
            for i in 0..3 {
                face[i] = map_vertices[face[i]] as usize;
            }
        }
        // Optionally shrink to fit (reallocate) vertices.
        // TriangleMesh.cpp:783-784
        its.vertices.shrink_to_fit();
    }

    // TriangleMesh.cpp:787
    num_erased
}

/// TriangleMesh.cpp:796 — `int its_remove_degenerate_faces(indexed_triangle_set &its, bool shrink_to_fit)`
fn its_remove_degenerate_faces(its: &mut IndexedTriangleSet) -> i32 {
    // TriangleMesh.cpp:798-803 — std::remove_if of degenerate faces (any two equal indices) then erase.
    let before = its.indices.len();
    its.indices
        .retain(|face| !(face[0] == face[1] || face[0] == face[2] || face[1] == face[2]));
    // TriangleMesh.cpp:802
    let removed = (before - its.indices.len()) as i32;

    // TriangleMesh.cpp:805-806
    if removed != 0 {
        its.indices.shrink_to_fit();
    }

    // TriangleMesh.cpp:808
    removed
}

/// TriangleMesh.cpp:811 — `int its_compactify_vertices(indexed_triangle_set &its, bool shrink_to_fit)`
fn its_compactify_vertices(its: &mut IndexedTriangleSet) -> i32 {
    // First used to mark referenced vertices, later used for mapping old vertex index to a new one.
    // TriangleMesh.cpp:814
    let mut vertex_map: Vec<i32> = vec![0; its.vertices.len()];
    // Mark referenced vertices.
    // TriangleMesh.cpp:816-818
    for face in &its.indices {
        for i in 0..3 {
            vertex_map[face[i]] = 1;
        }
    }
    // Compactify vertices, update map from old vertex index to a new one.
    // TriangleMesh.cpp:820
    let mut last = 0i32;
    for i in 0..vertex_map.len() {
        // TriangleMesh.cpp:822
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
    // TriangleMesh.cpp:828
    if removed != 0 {
        // TriangleMesh.cpp:829
        its.vertices.truncate(last as usize);
        // Update faces with the new vertex indices.
        // TriangleMesh.cpp:831-833
        for face in its.indices.iter_mut() {
            for i in 0..3 {
                face[i] = vertex_map[face[i]] as usize;
            }
        }
        // Optionally shrink the vertices.
        // TriangleMesh.cpp:835-836
        its.vertices.shrink_to_fit();
    }

    removed
}

// =============================================================================
// SlicesToTriangleMesh.cpp
// =============================================================================

/// SlicesToTriangleMesh.cpp:15-46
/// `inline indexed_triangle_set wall_strip(const Polygon &poly, double lower_z_mm, double upper_z_mm)`
fn wall_strip(poly: &Polygon, lower_z_mm: f64, upper_z_mm: f64) -> IndexedTriangleSet {
    // SlicesToTriangleMesh.cpp:19
    let mut ret = IndexedTriangleSet::new();

    // SlicesToTriangleMesh.cpp:21
    let startidx = ret.vertices.len();
    // SlicesToTriangleMesh.cpp:22
    let offs = poly.points().len();

    // SlicesToTriangleMesh.cpp:24
    ret.vertices.reserve(ret.vertices.len() + 2 * offs);

    // The expression unscaled(p).cast<float>().eval() is important here
    // as it ensures identical conversion of 2D scaled coordinates to float 3D
    // to that used by the tesselation. This way, the duplicated vertices in the
    // output mesh can be found with the == operator of the points.
    // its_merge_vertices will then reliably remove the duplicates.
    // SlicesToTriangleMesh.cpp:31-32
    for p in poly.points() {
        ret.vertices
            .push(to_3d_unscaled(*p, lower_z_mm as f32));
    }

    // SlicesToTriangleMesh.cpp:34-35
    for p in poly.points() {
        ret.vertices
            .push(to_3d_unscaled(*p, upper_z_mm as f32));
    }

    // SlicesToTriangleMesh.cpp:37-40
    for i in (startidx + 1)..(startidx + offs) {
        ret.indices.push([i - 1, i, i + offs - 1]);
        ret.indices.push([i, i + offs, i + offs - 1]);
    }

    // SlicesToTriangleMesh.cpp:42
    ret.indices
        .push([startidx + offs - 1, startidx, startidx + 2 * offs - 1]);
    // SlicesToTriangleMesh.cpp:43
    ret.indices
        .push([startidx, startidx + offs, startidx + 2 * offs - 1]);

    // SlicesToTriangleMesh.cpp:45
    ret
}

/// Helper mirroring `to_3d(unscaled(p).cast<float>().eval(), float(z))`.
///
/// SlicesToTriangleMesh.cpp:32 — `unscaled(p)` converts scaled `coord_t`
/// coordinates to `double` mm (== `coord * SCALING_FACTOR`), then `.cast<float>()`
/// narrows to `f32`. The z component is supplied already as `f32`.
fn to_3d_unscaled(p: Point, z: f32) -> Vec3f {
    Vec3f::new(crate::unscale(p.x) as f32, crate::unscale(p.y) as f32, z)
}

// Same as walls() but with identical higher and lower polygons.
/// SlicesToTriangleMesh.cpp:49-54
/// `indexed_triangle_set inline straight_walls(const Polygon &plate, double lo_z, double hi_z)`
fn straight_walls_polygon(plate: &Polygon, lo_z: f64, hi_z: f64) -> IndexedTriangleSet {
    // SlicesToTriangleMesh.cpp:53
    wall_strip(plate, lo_z, hi_z)
}

/// SlicesToTriangleMesh.cpp:56-65
/// `indexed_triangle_set inline straight_walls(const ExPolygon &plate, double lo_z, double hi_z)`
fn straight_walls_expolygon(plate: &ExPolygon, lo_z: f64, hi_z: f64) -> IndexedTriangleSet {
    // SlicesToTriangleMesh.cpp:60
    let mut ret = straight_walls_polygon(&plate.contour, lo_z, hi_z);
    // SlicesToTriangleMesh.cpp:61-62
    for h in &plate.holes {
        its_merge(&mut ret, &straight_walls_polygon(h, lo_z, hi_z));
    }

    // SlicesToTriangleMesh.cpp:64
    ret
}

/// SlicesToTriangleMesh.cpp:67-76
/// `indexed_triangle_set inline straight_walls(const ExPolygons &slice, double lo_z, double hi_z)`
fn straight_walls(slice: &ExPolygons, lo_z: f64, hi_z: f64) -> IndexedTriangleSet {
    // SlicesToTriangleMesh.cpp:71
    let mut ret = IndexedTriangleSet::new();
    // SlicesToTriangleMesh.cpp:72-73
    for poly in slice {
        its_merge(&mut ret, &straight_walls_expolygon(poly, lo_z, hi_z));
    }

    // SlicesToTriangleMesh.cpp:75
    ret
}

/// SlicesToTriangleMesh.cpp:78-122
/// `indexed_triangle_set slices_to_mesh(const std::vector<ExPolygons> &slices, double zmin, const std::vector<float> &grid)`
pub fn slices_to_mesh_grid(
    slices: &[ExPolygons],
    zmin: f64,
    grid: &[f32],
) -> IndexedTriangleSet {
    // SlicesToTriangleMesh.cpp:83
    assert_eq!(slices.len(), grid.len());

    // SlicesToTriangleMesh.cpp:85-86 — using Layers = std::vector<indexed_triangle_set>; Layers layers(slices.size());
    let mut layers: Vec<IndexedTriangleSet> =
        vec![IndexedTriangleSet::new(); slices.len()];
    // SlicesToTriangleMesh.cpp:87
    let len = slices.len() - 1;

    // SlicesToTriangleMesh.cpp:89-101 — tbb::parallel_for over [0, len). Ported as a
    // sequential loop (semantically identical; the reduce/merge below depends only on
    // per-layer results, not iteration order).
    for i in 0..len {
        // SlicesToTriangleMesh.cpp:90
        let upper = &slices[i + 1];
        // SlicesToTriangleMesh.cpp:91
        let lower = &slices[i];

        // Small 0 area artefacts can be created by diff_ex, and the
        // tesselation also can create 0 area triangles. These will be removed
        // by its_remove_degenerate_faces.
        // SlicesToTriangleMesh.cpp:96
        let free_top: ExPolygons = difference(lower, upper);
        // SlicesToTriangleMesh.cpp:97
        let overhang: ExPolygons = difference(upper, lower);
        // SlicesToTriangleMesh.cpp:98
        let free_top_tris = tesselate::triangulate_expolygons_3d(&free_top, grid[i] as f64, NORMALS_UP)
            .unwrap_or_default();
        its_merge_vec3d(&mut layers[i], &free_top_tris);
        // SlicesToTriangleMesh.cpp:99
        let overhang_tris = tesselate::triangulate_expolygons_3d(&overhang, grid[i] as f64, NORMALS_DOWN)
            .unwrap_or_default();
        its_merge_vec3d(&mut layers[i], &overhang_tris);
        // SlicesToTriangleMesh.cpp:100
        let walls = straight_walls(upper, grid[i] as f64, grid[i + 1] as f64);
        its_merge(&mut layers[i], &walls);
    }

    // SlicesToTriangleMesh.cpp:103-105 — merge_fn lambda.
    // SlicesToTriangleMesh.cpp:107-108 — execution::reduce(ex_tbb, layers.begin(), layers.end(),
    //                                    indexed_triangle_set{}, merge_fn);
    let mut ret = IndexedTriangleSet::new();
    for layer in &layers {
        its_merge(&mut ret, layer);
    }

    // SlicesToTriangleMesh.cpp:110
    let front_down = tesselate::triangulate_expolygons_3d(&slices[0], zmin, NORMALS_DOWN)
        .unwrap_or_default();
    its_merge_vec3d(&mut ret, &front_down);
    // SlicesToTriangleMesh.cpp:111
    let front_walls = straight_walls(&slices[0], zmin, grid[0] as f64);
    its_merge(&mut ret, &front_walls);
    // SlicesToTriangleMesh.cpp:112
    let back_up = tesselate::triangulate_expolygons_3d(
        &slices[slices.len() - 1],
        grid[grid.len() - 1] as f64,
        NORMALS_UP,
    )
    .unwrap_or_default();
    its_merge_vec3d(&mut ret, &back_up);

    // FIXME: these repairs do not fix the mesh entirely. There will be cracks
    // in the output. It is very hard to do the meshing in a way that does not
    // leave errors.
    // SlicesToTriangleMesh.cpp:117
    its_merge_vertices(&mut ret);
    // SlicesToTriangleMesh.cpp:118
    its_remove_degenerate_faces(&mut ret);
    // SlicesToTriangleMesh.cpp:119
    its_compactify_vertices(&mut ret);

    // SlicesToTriangleMesh.cpp:121
    ret
}

/// SlicesToTriangleMesh.cpp:124-136
/// `void slices_to_mesh(indexed_triangle_set &mesh, const std::vector<ExPolygons> &slices, double zmin, double lh, double ilh)`
pub fn slices_to_mesh(
    mesh: &mut IndexedTriangleSet,
    slices: &[ExPolygons],
    zmin: f64,
    lh: f64,
    ilh: f64,
) {
    // SlicesToTriangleMesh.cpp:130 — std::vector<float> grid(slices.size(), zmin + ilh);
    let mut grid: Vec<f32> = vec![(zmin + ilh) as f32; slices.len()];

    // SlicesToTriangleMesh.cpp:132
    for i in 1..grid.len() {
        grid[i] = grid[i - 1] + lh as f32;
    }

    // SlicesToTriangleMesh.cpp:134
    let cntr = slices_to_mesh_grid(slices, zmin, &grid);
    // SlicesToTriangleMesh.cpp:135
    its_merge(mesh, &cntr);
}

/// SlicesToTriangleMesh.hpp:15-22
/// `inline indexed_triangle_set slices_to_mesh(const std::vector<ExPolygons> &slices, double zmin, double lh, double ilh)`
pub fn slices_to_mesh_out(
    slices: &[ExPolygons],
    zmin: f64,
    lh: f64,
    ilh: f64,
) -> IndexedTriangleSet {
    // SlicesToTriangleMesh.hpp:18
    let mut out = IndexedTriangleSet::new();
    // SlicesToTriangleMesh.hpp:19
    slices_to_mesh(&mut out, slices, zmin, lh, ilh);

    // SlicesToTriangleMesh.hpp:21
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vec3f_creation() {
        let v = Vec3f::new(1.0, 2.0, 3.0);
        assert_eq!(v.x, 1.0);
        assert_eq!(v.y, 2.0);
        assert_eq!(v.z, 3.0);
    }

    #[test]
    fn test_its_merge_offsets_indices() {
        let mut mesh1 = IndexedTriangleSet {
            vertices: vec![Vec3f::new(0.0, 0.0, 0.0), Vec3f::new(1.0, 0.0, 0.0)],
            indices: vec![[0, 1, 0]],
        };

        let mesh2 = IndexedTriangleSet {
            vertices: vec![Vec3f::new(2.0, 0.0, 0.0)],
            indices: vec![[0, 0, 0]],
        };

        its_merge(&mut mesh1, &mesh2);

        assert_eq!(mesh1.vertices.len(), 3);
        assert_eq!(mesh1.indices.len(), 2);
        // Second mesh indices should be offset by 2.
        assert_eq!(mesh1.indices[1], [2, 2, 2]);
    }

    #[test]
    fn test_wall_strip_quad() {
        // Create a simple square polygon (1mm x 1mm). SCALING_FACTOR = 100_000.
        let points = vec![
            Point::new(0, 0),
            Point::new(100_000, 0),
            Point::new(100_000, 100_000),
            Point::new(0, 100_000),
        ];
        let poly = Polygon::from_points(points);

        let mesh = wall_strip(&poly, 0.0, 1.0);

        // 8 vertices (4 lower + 4 upper).
        assert_eq!(mesh.vertices.len(), 8);
        // 8 triangles (2 per edge, 4 edges).
        assert_eq!(mesh.indices.len(), 8);

        for i in 0..4 {
            assert_eq!(mesh.vertices[i].z, 0.0);
            assert_eq!(mesh.vertices[i + 4].z, 1.0);
        }
    }

    #[test]
    fn test_straight_walls_polygon() {
        let points = vec![
            Point::new(0, 0),
            Point::new(100_000, 0),
            Point::new(0, 100_000),
        ];
        let poly = Polygon::from_points(points);

        let mesh = straight_walls_polygon(&poly, 0.0, 1.0);

        // Triangle has 3 vertices, so 6 total (lower + upper).
        assert_eq!(mesh.vertices.len(), 6);
        // 2 triangles per edge, 3 edges = 6 triangles.
        assert_eq!(mesh.indices.len(), 6);
    }

    #[test]
    fn test_to_3d_unscaled() {
        // 1mm, 2mm in scaled coordinates (SCALING_FACTOR = 100_000).
        let p = Point::new(100_000, 200_000);
        let v = to_3d_unscaled(p, 5.0);

        assert!((v.x - 1.0).abs() < 0.0001);
        assert!((v.y - 2.0).abs() < 0.0001);
        assert_eq!(v.z, 5.0);
    }

    #[test]
    fn test_its_remove_degenerate_faces() {
        let mut its = IndexedTriangleSet {
            vertices: vec![
                Vec3f::new(0.0, 0.0, 0.0),
                Vec3f::new(1.0, 0.0, 0.0),
                Vec3f::new(0.0, 1.0, 0.0),
            ],
            indices: vec![[0, 1, 2], [0, 0, 2], [1, 1, 1]],
        };
        let removed = its_remove_degenerate_faces(&mut its);
        assert_eq!(removed, 2);
        assert_eq!(its.indices.len(), 1);
        assert_eq!(its.indices[0], [0, 1, 2]);
    }

    #[test]
    fn test_its_merge_vertices_dedup() {
        // Two coincident vertices (index 0 and 2) get merged to the lowest index.
        let mut its = IndexedTriangleSet {
            vertices: vec![
                Vec3f::new(0.0, 0.0, 0.0),
                Vec3f::new(1.0, 0.0, 0.0),
                Vec3f::new(0.0, 0.0, 0.0),
            ],
            indices: vec![[0, 1, 2]],
        };
        let erased = its_merge_vertices(&mut its);
        assert_eq!(erased, 1);
        assert_eq!(its.vertices.len(), 2);
        // The duplicate (old index 2) should remap to vertex 0.
        assert_eq!(its.indices[0][2], its.indices[0][0]);
    }
}
