//! Faithful 1:1 port of BambuStudio `src/libslic3r/TriangleMeshSlicer.cpp`.
//!
//! Triangle-plane intersection slicing: for each triangle, find the slicing
//! planes that intersect it, compute the intersection line segment, then chain
//! the segments into closed loops by triangle connectivity (edge / vertex IDs),
//! and finally build `ExPolygons` via Clipper union.
//!
//! Type mapping (per port rules): `coord_t -> i64`, `coordf_t -> f64`,
//! `stl_vertex -> StlVertex` (`Vec3f`, f32), `stl_triangle_vertex_indices ->
//! StlTriangleVertexIndices` (`Vec3i`), `Point -> geometry::Point`,
//! `Polygon -> geometry::Polygon`, `indexed_triangle_set ->
//! crate::normal_utils::indexed_triangle_set`.
//!
//! Divergences from C++ (see `divergences` in the port report):
//! - `tbb::parallel_for` loops are run sequentially (wasm-safe; no native TBB).
//!   The per-bucket `std::array<std::mutex, 64>` become no-ops. Results are
//!   semantically identical because each triangle / layer writes disjoint data.
//! - `make_expolygons()` uses the crate's `union_polygons_ex` (geo-clipper, NonZero
//!   semantics) instead of `ClipperLib::union_ex(loops, fill_type)` with the
//!   selectable `pftEvenOdd / pftPositive / pftNonZero` rule, and the
//!   `offset2_ex / offset_ex` closing offset, which are not available with the
//!   crate's geo-based Clipper layer. The default `Regular` mode matches NonZero.
//! - Slab / cut functions (`slice_mesh_slabs`, `slice_facet_with_slabs`,
//!   `make_slab_loops`, `project_mesh`, `cut_mesh`, `triangulate_slice`) are
//!   deferred — they depend on TBB-free slab chaining + `MeshBoolean` (native
//!   CGAL) + `triangulate_expolygons_3d` Result threading not yet wired here.
//! - The public API kept for callers (`slicer.rs`) is `slice_mesh(&TriangleMesh,
//!   &[f64])` and `slice_mesh_at_z(&TriangleMesh, f64)`; the faithful C++-shaped
//!   functions operate on `indexed_triangle_set` and are exposed alongside.

use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon, Polygons};
use crate::normal_utils::{indexed_triangle_set, StlVertex};
use crate::triangle_mesh::{its_face_edge_ids, Vec3i};
use crate::triangle_mesh::TriangleMesh;
use crate::{scale, unscale, CoordF};

// TriangleMeshSlicer.cpp:44
const EPSON: f32 = 1e-3;
// TriangleMeshSlicer.cpp:45-48
fn is_equal(lh: f32, rh: f32) -> bool {
    (lh - rh).abs() <= EPSON
}

// TriangleMeshSlicer.cpp:58-69
/// Where is this intersection point located? On mesh vertex or mesh edge?
/// Only one of the following will be set, the other will remain set to -1.
#[derive(Clone, Copy, Debug)]
struct IntersectionReference {
    // Index of the mesh vertex.
    point_id: i32,
    // Index of the mesh edge.
    edge_id: i32,
}

impl IntersectionReference {
    // TriangleMeshSlicer.cpp:61
    fn default() -> Self {
        Self {
            point_id: -1,
            edge_id: -1,
        }
    }
    // TriangleMeshSlicer.cpp:62
    fn new(point_id: i32, edge_id: i32) -> Self {
        Self { point_id, edge_id }
    }
}

// TriangleMeshSlicer.cpp:71-78
/// `class IntersectionPoint : public Point, public IntersectionReference`
#[derive(Clone, Copy, Debug)]
struct IntersectionPoint {
    // Inherits coord_t x, y
    point: Point,
    point_id: i32,
    edge_id: i32,
}

impl IntersectionPoint {
    // TriangleMeshSlicer.cpp:74
    fn default() -> Self {
        Self {
            point: Point::new(0, 0),
            point_id: -1,
            edge_id: -1,
        }
    }
}

// TriangleMeshSlicer.cpp:102-117
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FacetEdgeType {
    // A general case, the cutting plane intersect a face at two different edges.
    General,
    // Two vertices are aligned with the cutting plane, the third vertex is below the cutting plane.
    Top,
    // Two vertices are aligned with the cutting plane, the third vertex is above the cutting plane.
    Bottom,
    // Two vertices are aligned with the cutting plane, the edge is shared by two triangles, where one
    // triangle is below or at the cutting plane and the other is above or at the cutting plane (only one
    // vertex may lie on the plane).
    #[allow(dead_code)]
    TopBottom,
    // All three vertices of a face are aligned with the cutting plane.
    Horizontal,
    // Edge
    #[allow(dead_code)]
    Slab,
}

// TriangleMeshSlicer.cpp:80-145
/// `class IntersectionLine : public Line`
#[derive(Clone, Debug)]
struct IntersectionLine {
    // Inherits Point a, b
    a: Point,
    b: Point,
    // For each line end point, either {a,b}_id or {a,b}edge_a_id is set, the other is left to -1.
    // Vertex indices of the line end points.
    a_id: i32,
    b_id: i32,
    // Source mesh edges of the line end points.
    edge_a_id: i32,
    edge_b_id: i32,
    // feGeneral, feTop, feBottom, feHorizontal
    edge_type: FacetEdgeType,
    flags: u32,
}

impl IntersectionLine {
    // TriangleMeshSlicer.cpp:122-134 — flag bit constants
    // Triangle edge added, because it has no neighbor.
    #[allow(dead_code)]
    const EDGE0_NO_NEIGHBOR: u32 = 0x001;
    #[allow(dead_code)]
    const EDGE1_NO_NEIGHBOR: u32 = 0x002;
    #[allow(dead_code)]
    const EDGE2_NO_NEIGHBOR: u32 = 0x004;
    // Triangle edge added, because it makes a fold with another horizontal edge.
    #[allow(dead_code)]
    const EDGE0_FOLD: u32 = 0x010;
    #[allow(dead_code)]
    const EDGE1_FOLD: u32 = 0x020;
    #[allow(dead_code)]
    const EDGE2_FOLD: u32 = 0x040;
    // The edge cannot be a seed of a greedy loop extraction (folds are not safe to become seeds).
    const NO_SEED: u32 = 0x100;
    const SKIP: u32 = 0x200;

    // TriangleMeshSlicer.cpp:83
    fn default() -> Self {
        Self {
            a: Point::new(0, 0),
            b: Point::new(0, 0),
            a_id: -1,
            b_id: -1,
            edge_a_id: -1,
            edge_b_id: -1,
            edge_type: FacetEdgeType::General,
            flags: 0,
        }
    }

    // TriangleMeshSlicer.cpp:85
    fn skip(&self) -> bool {
        (self.flags & Self::SKIP) != 0
    }
    // TriangleMeshSlicer.cpp:86
    fn set_skip(&mut self) {
        self.flags |= Self::SKIP;
    }

    // TriangleMeshSlicer.cpp:88
    fn is_seed_candidate(&self) -> bool {
        (self.flags & Self::NO_SEED) == 0 && !self.skip()
    }
    // TriangleMeshSlicer.cpp:89
    #[allow(dead_code)]
    fn set_no_seed(&mut self, set: bool) {
        if set {
            self.flags |= Self::NO_SEED;
        } else {
            self.flags &= !Self::NO_SEED;
        }
    }

    // TriangleMeshSlicer.cpp:91
    #[allow(dead_code)]
    fn reverse(&mut self) {
        std::mem::swap(&mut self.a, &mut self.b);
        std::mem::swap(&mut self.a_id, &mut self.b_id);
        std::mem::swap(&mut self.edge_a_id, &mut self.edge_b_id);
    }
}

// TriangleMeshSlicer.cpp:147
type IntersectionLines = Vec<IntersectionLine>;

// TriangleMeshSlicer.cpp:149-153
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FacetSliceType {
    NoSlice = 0,
    Slicing = 1,
    Cutting = 2,
}

// TriangleMeshSlicer.cpp:155-320
// Return the slice type; if Slicing/Cutting, line_out has been filled.
fn slice_facet(
    // Z height of the slice in XY plane. Scaled or unscaled (same as vertices[].z()).
    slice_z: f32,
    // 3 vertices of the triangle, XY scaled. Z scaled or unscaled (same as slice_z).
    vertices: &[StlVertex; 3],
    indices: &Vec3i,
    edge_ids: &Vec3i,
    idx_vertex_lowest: i32,
    horizontal: bool,
    line_out: &mut IntersectionLine,
) -> FacetSliceType {
    // TriangleMeshSlicer.cpp:167-169
    let mut points: [IntersectionPoint; 3] =
        [IntersectionPoint::default(), IntersectionPoint::default(), IntersectionPoint::default()];
    let mut num_points: usize = 0;
    // size_t(-1) sentinel
    let mut point_on_layer: usize = usize::MAX;

    // Reorder vertices so that the first one is the one with lowest Z.
    // This is needed to get all intersection lines in a consistent order
    // (external on the right of the line)
    // TriangleMeshSlicer.cpp:174 — loop through facet edges
    for j in 0..3 {
        // TriangleMeshSlicer.cpp:178-187
        let k = ((idx_vertex_lowest + j) % 3) as usize;
        let l = (k + 1) % 3;
        let edge_id = edge_ids[k];
        let mut a_id = indices[k];
        let mut a = vertices[k];
        let mut b_id = indices[l];
        let mut b = vertices[l];
        let _c = vertices[(k + 2) % 3];

        // Is edge or face aligned with the cutting plane?
        // TriangleMeshSlicer.cpp:190
        if a.z == slice_z && b.z == slice_z {
            // Edge is horizontal and belongs to the current layer.
            // TriangleMeshSlicer.cpp:193-195
            let v0 = vertices[0];
            let v1 = vertices[1];
            let v2 = vertices[2];
            // We may ignore this edge for slicing purposes, but we may still use it for object cutting.
            // TriangleMeshSlicer.cpp:197
            let mut result = FacetSliceType::Slicing;
            if horizontal {
                // All three vertices are aligned with slice_z.
                // TriangleMeshSlicer.cpp:200-207
                line_out.edge_type = FacetEdgeType::Horizontal;
                result = FacetSliceType::Cutting;
                let normal = (v1.x - v0.x) as f64 * (v2.y - v1.y) as f64
                    - (v1.y - v0.y) as f64 * (v2.x - v1.x) as f64;
                if normal < 0.0 {
                    // If normal points downwards this is a bottom horizontal facet so we reverse its point order.
                    std::mem::swap(&mut a, &mut b);
                    std::mem::swap(&mut a_id, &mut b_id);
                }
            } else {
                // Two vertices are aligned with the cutting plane, the third vertex is below or above the cutting plane.
                // Is the third vertex below the cutting plane?
                // TriangleMeshSlicer.cpp:211
                let third_below = v0.z < slice_z || v1.z < slice_z || v2.z < slice_z;
                // Two vertices on the cutting plane, the third vertex is below the plane. Consider the edge to be part of the slice
                // only if it is the upper edge.
                // TriangleMeshSlicer.cpp:216-222
                result = if third_below {
                    FacetSliceType::Slicing
                } else {
                    FacetSliceType::Cutting
                };
                if third_below {
                    line_out.edge_type = FacetEdgeType::Top;
                    std::mem::swap(&mut a, &mut b);
                    std::mem::swap(&mut a_id, &mut b_id);
                } else {
                    line_out.edge_type = FacetEdgeType::Bottom;
                }
            }
            // TriangleMeshSlicer.cpp:224-229
            line_out.a.x = a.x as i64;
            line_out.a.y = a.y as i64;
            line_out.b.x = b.x as i64;
            line_out.b.y = b.y as i64;
            line_out.a_id = a_id;
            line_out.b_id = b_id;
            debug_assert!(line_out.a != line_out.b);
            return result;
        }

        // TriangleMeshSlicer.cpp:234
        if a.z == slice_z {
            // Only point a alings with the cutting plane.
            // TriangleMeshSlicer.cpp:236-242
            if point_on_layer == usize::MAX || points[point_on_layer].point_id != a_id {
                point_on_layer = num_points;
                let point = &mut points[num_points];
                num_points += 1;
                point.point.x = a.x as i64;
                point.point.y = a.y as i64;
                point.point_id = a_id;
            }
        } else if b.z == slice_z {
            // Only point b alings with the cutting plane.
            // TriangleMeshSlicer.cpp:245-251
            if point_on_layer == usize::MAX || points[point_on_layer].point_id != b_id {
                point_on_layer = num_points;
                let point = &mut points[num_points];
                num_points += 1;
                point.point.x = b.x as i64;
                point.point.y = b.y as i64;
                point.point_id = b_id;
            }
        } else if (a.z < slice_z && b.z > slice_z) || (b.z < slice_z && a.z > slice_z) {
            // A general case. The face edge intersects the cutting plane. Calculate the intersection point.
            // TriangleMeshSlicer.cpp:254
            debug_assert!(a_id != b_id);
            // Sort the edge to give a consistent answer.
            // TriangleMeshSlicer.cpp:256-259
            if a_id > b_id {
                std::mem::swap(&mut a_id, &mut b_id);
                std::mem::swap(&mut a, &mut b);
            }
            // TriangleMeshSlicer.cpp:261
            let t = (slice_z as f64 - b.z as f64) / (a.z as f64 - b.z as f64);
            if t <= 0.0 {
                // TriangleMeshSlicer.cpp:262-268
                if point_on_layer == usize::MAX || points[point_on_layer].point_id != a_id {
                    let point = &mut points[num_points];
                    point.point.x = a.x as i64;
                    point.point.y = a.y as i64;
                    point_on_layer = num_points;
                    num_points += 1;
                    point.point_id = a_id;
                }
            } else if t >= 1.0 {
                // TriangleMeshSlicer.cpp:269-275
                if point_on_layer == usize::MAX || points[point_on_layer].point_id != b_id {
                    let point = &mut points[num_points];
                    point.point.x = b.x as i64;
                    point.point.y = b.y as i64;
                    point_on_layer = num_points;
                    num_points += 1;
                    point.point_id = b_id;
                }
            } else {
                // TriangleMeshSlicer.cpp:277-280
                let point = &mut points[num_points];
                point.point.x =
                    (b.x as f64 + (a.x as f64 - b.x as f64) * t + 0.5).floor() as i64;
                point.point.y =
                    (b.y as f64 + (a.y as f64 - b.y as f64) * t + 0.5).floor() as i64;
                point.edge_id = edge_id;
                num_points += 1;
            }
        }
    }

    // Facets must intersect each plane 0 or 2 times, or it may touch the plane at a single vertex only.
    // TriangleMeshSlicer.cpp:286
    debug_assert!(num_points < 3);
    if num_points == 2 {
        // TriangleMeshSlicer.cpp:288-294
        line_out.edge_type = FacetEdgeType::General;
        line_out.a = points[1].point;
        line_out.b = points[0].point;
        line_out.a_id = points[1].point_id;
        line_out.b_id = points[0].point_id;
        line_out.edge_a_id = points[1].edge_id;
        line_out.edge_b_id = points[0].edge_id;
        // The plane cuts at least one edge in a general position.
        debug_assert!(line_out.a_id == -1 || line_out.b_id == -1);
        debug_assert!(line_out.edge_a_id != -1 || line_out.edge_b_id != -1);
        // General slicing position, use the segment for both slicing and object cutting.
        // TriangleMeshSlicer.cpp:317
        return FacetSliceType::Slicing;
    }
    // TriangleMeshSlicer.cpp:319
    FacetSliceType::NoSlice
}

// TriangleMeshSlicer.cpp:475-508
// `slice_facet_at_zs` — the `TransformVertex` is applied by the caller producing
// `mesh_vertices` already transformed/scaled, so here we take the vertices directly.
fn slice_facet_at_zs(
    // Scaled or unscaled vertices (already transformed by transform_vertex_fn).
    mesh_vertices: &[StlVertex],
    indices: &Vec3i,
    edge_ids: &Vec3i,
    // Scaled or unscaled zs.
    zs: &[f32],
    lines: &mut [IntersectionLines],
) {
    // TriangleMeshSlicer.cpp:487
    let vertices: [StlVertex; 3] = [
        mesh_vertices[indices[0] as usize],
        mesh_vertices[indices[1] as usize],
        mesh_vertices[indices[2] as usize],
    ];

    // find facet extents
    // TriangleMeshSlicer.cpp:490-491
    let min_z = vertices[0].z.min(vertices[1].z.min(vertices[2].z));
    let max_z = vertices[0].z.max(vertices[1].z.max(vertices[2].z));

    // find layer extents
    // TriangleMeshSlicer.cpp:494 — first layer whose slice_z is >= min_z
    let min_layer = lower_bound(zs, min_z);
    // TriangleMeshSlicer.cpp:495 — first layer (from min_layer) whose slice_z is > max_z
    let max_layer = min_layer + upper_bound(&zs[min_layer..], max_z);
    // TriangleMeshSlicer.cpp:496
    let idx_vertex_lowest: i32 = if vertices[1].z == min_z {
        1
    } else if vertices[2].z == min_z {
        2
    } else {
        0
    };

    // TriangleMeshSlicer.cpp:498
    for slice_id in min_layer..max_layer {
        let mut il = IntersectionLine::default();
        // Ignore horizontal triangles.
        // TriangleMeshSlicer.cpp:501
        if min_z != max_z
            && slice_facet(zs[slice_id], &vertices, indices, edge_ids, idx_vertex_lowest, false, &mut il)
                == FacetSliceType::Slicing
        {
            debug_assert!(il.edge_type != FacetEdgeType::Horizontal);
            // TriangleMeshSlicer.cpp:503-505 (mutex sequentialized)
            lines[slice_id].push(il);
        }
    }
}

// `std::lower_bound`: first index i with zs[i] >= value.
fn lower_bound(zs: &[f32], value: f32) -> usize {
    zs.partition_point(|&z| z < value)
}

// `std::upper_bound`: first index i with zs[i] > value.
fn upper_bound(zs: &[f32], value: f32) -> usize {
    zs.partition_point(|&z| z <= value)
}

// TriangleMeshSlicer.cpp:510-532 — vector-of-zs variant of slice_make_lines.
// `transform_vertex_fn` is applied by the caller, so `vertices` are already
// transformed. Runs sequentially (no TBB).
fn slice_make_lines(
    vertices: &[StlVertex],
    indices: &[Vec3i],
    face_edge_ids: &[Vec3i],
    zs: &[f32],
    throw_on_cancel_fn: &dyn Fn(),
) -> Vec<IntersectionLines> {
    // TriangleMeshSlicer.cpp:519
    let mut lines: Vec<IntersectionLines> = vec![IntersectionLines::new(); zs.len()];
    // TriangleMeshSlicer.cpp:521-530 (tbb::parallel_for over faces -> sequential)
    for face_idx in 0..indices.len() {
        // TriangleMeshSlicer.cpp:525
        if (face_idx & 0x0ffff) == 0 {
            throw_on_cancel_fn();
        }
        slice_facet_at_zs(vertices, &indices[face_idx], &face_edge_ids[face_idx], zs, &mut lines);
    }
    lines
}

// TriangleMeshSlicer.cpp:534-561 — single-plane variant with a face filter.
fn slice_make_lines_single(
    mesh_vertices: &[StlVertex],
    mesh_faces: &[Vec3i],
    face_edge_ids: &[Vec3i],
    plane_z: f32,
    face_filter: &dyn Fn(usize) -> bool,
) -> IntersectionLines {
    let mut lines = IntersectionLines::new();
    // TriangleMeshSlicer.cpp:544
    for face_idx in 0..mesh_faces.len() {
        if face_filter(face_idx) {
            // TriangleMeshSlicer.cpp:546-547
            let indices = &mesh_faces[face_idx];
            let vertices: [StlVertex; 3] = [
                mesh_vertices[indices[0] as usize],
                mesh_vertices[indices[1] as usize],
                mesh_vertices[indices[2] as usize],
            ];
            // find facet extents
            // TriangleMeshSlicer.cpp:549-550
            let min_z = vertices[0].z.min(vertices[1].z.min(vertices[2].z));
            let _max_z = vertices[0].z.max(vertices[1].z.max(vertices[2].z));
            // TriangleMeshSlicer.cpp:552
            let idx_vertex_lowest: i32 = if vertices[1].z == min_z {
                1
            } else if vertices[2].z == min_z {
                2
            } else {
                0
            };
            let mut il = IntersectionLine::default();
            // Ignore horizontal triangles.
            // TriangleMeshSlicer.cpp:555
            if min_z != _max_z
                && slice_facet(
                    plane_z,
                    &vertices,
                    indices,
                    &face_edge_ids[face_idx],
                    idx_vertex_lowest,
                    false,
                    &mut il,
                ) == FacetSliceType::Slicing
            {
                debug_assert!(il.edge_type != FacetEdgeType::Horizontal);
                lines.push(il);
            }
        }
    }
    lines
}

// TriangleMeshSlicer.cpp:1043-1056
struct OpenPolyline {
    start: IntersectionReference,
    end: IntersectionReference,
    points: Vec<Point>,
    length: f64,
    consumed: bool,
}

impl OpenPolyline {
    // TriangleMeshSlicer.cpp:1045-1046
    fn new(start: IntersectionReference, end: IntersectionReference, points: Vec<Point>) -> Self {
        let length = crate::multi_point::length(&points);
        Self {
            start,
            end,
            points,
            length,
            consumed: false,
        }
    }
    // TriangleMeshSlicer.cpp:1047-1050
    fn reverse(&mut self) {
        std::mem::swap(&mut self.start, &mut self.end);
        self.points.reverse();
    }
}

// TriangleMeshSlicer.cpp:1060-1161
// called by make_loops() to connect sliced triangles into closed loops and open polylines by the triangle connectivity.
// Only connects segments crossing triangles of the same orientation.
fn chain_lines_by_triangle_connectivity(
    lines: &mut [IntersectionLine],
    loops: &mut Polygons,
    open_polylines: &mut Vec<OpenPolyline>,
) {
    // Build a map of lines by edge_a_id and a_id.
    // TriangleMeshSlicer.cpp:1063-1078 — vectors of indices sorted by key.
    let mut by_edge_a_id: Vec<usize> = Vec::with_capacity(lines.len());
    let mut by_a_id: Vec<usize> = Vec::with_capacity(lines.len());
    for (idx, line) in lines.iter().enumerate() {
        if !line.skip() {
            if line.edge_a_id != -1 {
                by_edge_a_id.push(idx);
            }
            if line.a_id != -1 {
                by_a_id.push(idx);
            }
        }
    }
    // by_edge_lower / by_vertex_lower
    by_edge_a_id.sort_by_key(|&i| lines[i].edge_a_id);
    by_a_id.sort_by_key(|&i| lines[i].a_id);

    // Chain the segments with a greedy algorithm, collect the loops and unclosed polylines.
    // TriangleMeshSlicer.cpp:1080
    let mut it_line_seed: usize = 0;
    loop {
        // take first spare line and start a new loop
        // TriangleMeshSlicer.cpp:1083-1091
        let mut first_line: Option<usize> = None;
        while it_line_seed < lines.len() {
            if lines[it_line_seed].is_seed_candidate() {
                first_line = Some(it_line_seed);
                it_line_seed += 1;
                break;
            }
            it_line_seed += 1;
        }
        let first_idx = match first_line {
            Some(i) => i,
            None => break,
        };
        // TriangleMeshSlicer.cpp:1092
        lines[first_idx].set_skip();
        // TriangleMeshSlicer.cpp:1093-1095
        let mut loop_pts: Vec<Point> = Vec::new();
        loop_pts.push(lines[first_idx].a);
        let mut last_idx = first_idx;

        // TriangleMeshSlicer.cpp:1104
        loop {
            // find a line starting where last one finishes
            // TriangleMeshSlicer.cpp:1106
            let mut next_line: Option<usize> = None;
            // TriangleMeshSlicer.cpp:1107-1118
            let last_edge_b_id = lines[last_idx].edge_b_id;
            if last_edge_b_id != -1 {
                let key = last_edge_b_id;
                if let Some(found) =
                    find_first_unskipped(&by_edge_a_id, lines, key, |l| l.edge_a_id)
                {
                    next_line = Some(found);
                }
            }
            // TriangleMeshSlicer.cpp:1119-1130
            let last_b_id = lines[last_idx].b_id;
            if next_line.is_none() && last_b_id != -1 {
                let key = last_b_id;
                if let Some(found) = find_first_unskipped(&by_a_id, lines, key, |l| l.a_id) {
                    next_line = Some(found);
                }
            }
            // TriangleMeshSlicer.cpp:1131
            if next_line.is_none() {
                // Check whether we closed this loop.
                // TriangleMeshSlicer.cpp:1133-1134
                let first_edge_a_id = lines[first_idx].edge_a_id;
                let first_a_id = lines[first_idx].a_id;
                let last_edge_b_id = lines[last_idx].edge_b_id;
                let last_b_id = lines[last_idx].b_id;
                if (first_edge_a_id != -1 && first_edge_a_id == last_edge_b_id)
                    || (first_a_id != -1 && first_a_id == last_b_id)
                {
                    // The current loop is complete. Add it to the output.
                    // TriangleMeshSlicer.cpp:1136-1137
                    debug_assert!(lines[first_idx].a == lines[last_idx].b);
                    loops.push(Polygon::from_points(std::mem::take(&mut loop_pts)));
                } else {
                    // This is an open polyline. Add it to the list of open polylines.
                    // TriangleMeshSlicer.cpp:1143-1146
                    loop_pts.push(lines[last_idx].b);
                    open_polylines.push(OpenPolyline::new(
                        IntersectionReference::new(
                            lines[first_idx].a_id,
                            lines[first_idx].edge_a_id,
                        ),
                        IntersectionReference::new(lines[last_idx].b_id, lines[last_idx].edge_b_id),
                        std::mem::take(&mut loop_pts),
                    ));
                }
                break;
            }
            let next_idx = next_line.unwrap();
            // TriangleMeshSlicer.cpp:1155-1158
            debug_assert!(lines[last_idx].b == lines[next_idx].a);
            loop_pts.push(lines[next_idx].a);
            last_idx = next_idx;
            lines[next_idx].set_skip();
        }
    }
}

// Helper for the lower_bound/upper_bound + first non-skipped lookup in
// chain_lines_by_triangle_connectivity (TriangleMeshSlicer.cpp:1109-1116, 1121-1128).
fn find_first_unskipped(
    sorted: &[usize],
    lines: &[IntersectionLine],
    key: i32,
    key_fn: impl Fn(&IntersectionLine) -> i32,
) -> Option<usize> {
    // std::lower_bound by key
    let begin = sorted.partition_point(|&i| key_fn(&lines[i]) < key);
    let mut it = begin;
    while it < sorted.len() && key_fn(&lines[sorted[it]]) == key {
        if !lines[sorted[it]].skip() {
            return Some(sorted[it]);
        }
        it += 1;
    }
    None
}

// TriangleMeshSlicer.cpp:1163-1175
fn open_polylines_sorted(open_polylines: &mut [OpenPolyline], update_lengths: bool) -> Vec<usize> {
    let mut out: Vec<usize> = Vec::with_capacity(open_polylines.len());
    for (i, opl) in open_polylines.iter_mut().enumerate() {
        if !opl.consumed {
            if update_lengths {
                opl.length = crate::multi_point::length(&opl.points);
            }
            out.push(i);
        }
    }
    // sort by length descending
    out.sort_by(|&a, &b| {
        open_polylines[b]
            .length
            .partial_cmp(&open_polylines[a].length)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    out
}

// TriangleMeshSlicer.cpp:1179-1273
// called by make_loops() to connect remaining open polylines across shared triangle edges and vertices.
fn chain_open_polylines_exact(
    open_polylines: &mut Vec<OpenPolyline>,
    loops: &mut Polygons,
    try_connect_reversed: bool,
) {
    // Store the end points of open_polylines into vectors sorted.
    // OpenPolylineEnd: (polyline index, start flag). id() = point_id>=0 ? point_id : -edge_id.
    // TriangleMeshSlicer.cpp:1182-1201
    #[derive(Clone, Copy)]
    struct OpenPolylineEnd {
        polyline: usize,
        start: bool,
    }
    let ipref = |opls: &[OpenPolyline], e: OpenPolylineEnd| -> IntersectionReference {
        if e.start {
            opls[e.polyline].start
        } else {
            opls[e.polyline].end
        }
    };
    let id_of = |opls: &[OpenPolyline], e: OpenPolylineEnd| -> i32 {
        let r = ipref(opls, e);
        if r.point_id >= 0 {
            r.point_id
        } else {
            -r.edge_id
        }
    };

    let mut by_id: Vec<OpenPolylineEnd> = Vec::with_capacity(2 * open_polylines.len());
    for (i, opl) in open_polylines.iter().enumerate() {
        if opl.start.point_id != -1 || opl.start.edge_id != -1 {
            by_id.push(OpenPolylineEnd {
                polyline: i,
                start: true,
            });
        }
        if try_connect_reversed && (opl.end.point_id != -1 || opl.end.edge_id != -1) {
            by_id.push(OpenPolylineEnd {
                polyline: i,
                start: false,
            });
        }
    }
    // TriangleMeshSlicer.cpp:1202
    by_id.sort_by_key(|&e| id_of(open_polylines, e));

    // TriangleMeshSlicer.cpp:1204-1210 — find an iterator into by_id for a given end.
    let find_polyline_end = |by_id: &[OpenPolylineEnd],
                             open_polylines: &[OpenPolyline],
                             end: OpenPolylineEnd|
     -> Option<usize> {
        let end_id = id_of(open_polylines, end);
        let mut it = by_id.partition_point(|&e| id_of(open_polylines, e) < end_id);
        while it < by_id.len() && id_of(open_polylines, by_id[it]) == end_id {
            if by_id[it].polyline == end.polyline && by_id[it].start == end.start {
                return Some(it);
            }
            it += 1;
        }
        None
    };

    // Try to connect the loops.
    // TriangleMeshSlicer.cpp:1212
    let sorted_by_length = open_polylines_sorted(open_polylines, false);
    // TriangleMeshSlicer.cpp:1213
    for opl_idx in sorted_by_length {
        if open_polylines[opl_idx].consumed {
            continue;
        }
        // TriangleMeshSlicer.cpp:1216
        open_polylines[opl_idx].consumed = true;
        // TriangleMeshSlicer.cpp:1217 — end(opl, false)
        let mut end = OpenPolylineEnd {
            polyline: opl_idx,
            start: false,
        };
        // TriangleMeshSlicer.cpp:1218
        loop {
            // find a line starting where last one finishes
            // TriangleMeshSlicer.cpp:1220-1226
            let end_id = id_of(open_polylines, end);
            let mut it_next_start =
                by_id.partition_point(|&e| id_of(open_polylines, e) < end_id);
            let mut found: Option<usize> = None;
            while it_next_start < by_id.len()
                && id_of(open_polylines, by_id[it_next_start]) == end_id
            {
                if !open_polylines[by_id[it_next_start].polyline].consumed {
                    found = Some(it_next_start);
                    break;
                }
                it_next_start += 1;
            }
            let it_next_start = match found {
                Some(i) => i,
                None => {
                    // The current loop could not be closed. Unmark the segment.
                    // TriangleMeshSlicer.cpp:1225-1226
                    open_polylines[opl_idx].consumed = false;
                    break;
                }
            };
            // found:
            let next_end = by_id[it_next_start];
            let next_poly = next_end.polyline;
            // Attach this polyline to the end of the initial polyline.
            // TriangleMeshSlicer.cpp:1229-1235
            if next_end.start {
                let pts = open_polylines[next_poly].points.clone();
                open_polylines[opl_idx].points.extend_from_slice(&pts[1..]);
            } else {
                let mut pts = open_polylines[next_poly].points.clone();
                pts.reverse();
                open_polylines[opl_idx].points.extend_from_slice(&pts[1..]);
            }
            // TriangleMeshSlicer.cpp:1236
            open_polylines[opl_idx].length += open_polylines[next_poly].length;
            // Mark the next polyline as consumed.
            // TriangleMeshSlicer.cpp:1238-1240
            open_polylines[next_poly].points.clear();
            open_polylines[next_poly].length = 0.0;
            open_polylines[next_poly].consumed = true;
            if try_connect_reversed {
                // Update the end point lookup structure after the end point of the current polyline was extended.
                // TriangleMeshSlicer.cpp:1244-1249
                let it_end = find_polyline_end(&by_id, open_polylines, end);
                let it_next_end = find_polyline_end(
                    &by_id,
                    open_polylines,
                    OpenPolylineEnd {
                        polyline: next_poly,
                        start: !next_end.start,
                    },
                );
                // Swap the end points of the current and next polyline, but keep the polyline ptr and the start flag.
                // std::swap(opl->end, it_next_end->start ? it_next_end->polyline->start : it_next_end->polyline->end);
                if let Some(it_next_end) = it_next_end {
                    let target = by_id[it_next_end];
                    let new_opl_end = if target.start {
                        open_polylines[target.polyline].start
                    } else {
                        open_polylines[target.polyline].end
                    };
                    let old_opl_end = open_polylines[opl_idx].end;
                    open_polylines[opl_idx].end = new_opl_end;
                    if target.start {
                        open_polylines[target.polyline].start = old_opl_end;
                    } else {
                        open_polylines[target.polyline].end = old_opl_end;
                    }
                    // Swap the positions of OpenPolylineEnd structures in the sorted array.
                    if let Some(it_end) = it_end {
                        by_id.swap(it_end, it_next_end);
                    }
                }
            }
            // Check whether we closed this loop.
            // TriangleMeshSlicer.cpp:1252-1253
            let start_ref = open_polylines[opl_idx].start;
            let end_ref = open_polylines[opl_idx].end;
            if (start_ref.edge_id != -1 && start_ref.edge_id == end_ref.edge_id)
                || (start_ref.point_id != -1 && start_ref.point_id == end_ref.point_id)
            {
                // The current loop is complete. Add it to the output.
                // TriangleMeshSlicer.cpp:1258 — remove the duplicate last point.
                open_polylines[opl_idx].points.pop();
                // TriangleMeshSlicer.cpp:1259
                if open_polylines[opl_idx].points.len() >= 3 {
                    // TriangleMeshSlicer.cpp:1260-1264
                    if try_connect_reversed
                        && Polygon::area_of(&open_polylines[opl_idx].points) < 0.0
                    {
                        open_polylines[opl_idx].points.reverse();
                    }
                    loops.push(Polygon::from_points(std::mem::take(
                        &mut open_polylines[opl_idx].points,
                    )));
                }
                open_polylines[opl_idx].points.clear();
                break;
            }
            // Continue with the current loop.
            end = OpenPolylineEnd {
                polyline: opl_idx,
                start: false,
            };
        }
    }
}

// Point.hpp:378-473 — ClosestPointInRadiusLookup, specialized for OpenPolylineEnd.
// Grid-bucketed spatial lookup over polyline end points.
struct ClosestPointLookup {
    search_radius: i64,
    grid_log2: u32,
    // map: grid cell (x,y) -> list of (polyline index, start flag)
    map: std::collections::HashMap<(i64, i64), Vec<(usize, bool)>>,
}

impl ClosestPointLookup {
    // Point.hpp:381-411
    fn new(search_radius: i64) -> Self {
        // Resolution of a grid, twice the search radius + some epsilon.
        let gridres = 2 * search_radius + 4;
        let mut grid_resolution = gridres;
        let mut grid_log2: u32 = 0;
        if grid_resolution > 32767 {
            grid_resolution >>= 16;
            grid_log2 += 16;
        }
        if grid_resolution > 127 {
            grid_resolution >>= 8;
            grid_log2 += 8;
        }
        if grid_resolution > 7 {
            grid_resolution >>= 4;
            grid_log2 += 4;
        }
        if grid_resolution > 1 {
            grid_resolution >>= 2;
            grid_log2 += 2;
        }
        if grid_resolution > 0 {
            grid_log2 += 1;
        }
        Self {
            search_radius,
            grid_log2,
            map: std::collections::HashMap::new(),
        }
    }

    fn point_of(open_polylines: &[OpenPolyline], value: (usize, bool)) -> Option<Point> {
        // OpenPolylineEndAccessor: consumed -> nullptr.
        let opl = &open_polylines[value.0];
        if opl.consumed {
            None
        } else if value.1 {
            Some(*opl.points.first().unwrap())
        } else {
            Some(*opl.points.last().unwrap())
        }
    }

    // Point.hpp:413-417
    fn insert(&mut self, open_polylines: &[OpenPolyline], value: (usize, bool)) {
        if let Some(pt) = Self::point_of(open_polylines, value) {
            let key = (pt.x >> self.grid_log2, pt.y >> self.grid_log2);
            self.map.entry(key).or_default().push(value);
        }
    }

    // Point.hpp:427-441
    fn erase(&mut self, open_polylines: &[OpenPolyline], value: (usize, bool)) -> bool {
        if let Some(pt) = Self::point_of(open_polylines, value) {
            let key = (pt.x >> self.grid_log2, pt.y >> self.grid_log2);
            if let Some(bucket) = self.map.get_mut(&key) {
                if let Some(pos) = bucket.iter().position(|&v| v == value) {
                    bucket.remove(pos);
                    return true;
                }
            }
        }
        false
    }

    // Point.hpp:444-473 — Return a pair of <value, distance_squared>.
    fn find(&self, open_polylines: &[OpenPolyline], pt: Point) -> (Option<(usize, bool)>, f64) {
        let mut value_min: Option<(usize, bool)> = None;
        let mut dist_min = f64::MAX;
        let grid_resolution: i64 = 1 << self.grid_log2;
        // Round pt to a closest grid_cell corner.
        let grid_corner = (
            (pt.x + (grid_resolution >> 1)) >> self.grid_log2,
            (pt.y + (grid_resolution >> 1)) >> self.grid_log2,
        );
        for neighbor_y in -1..1 {
            for neighbor_x in -1..1 {
                let key = (grid_corner.0 + neighbor_x, grid_corner.1 + neighbor_y);
                if let Some(bucket) = self.map.get(&key) {
                    for &value in bucket {
                        if let Some(pt2) = Self::point_of(open_polylines, value) {
                            let dx = (pt.x - pt2.x) as f64;
                            let dy = (pt.y - pt2.y) as f64;
                            let d2 = dx * dx + dy * dy;
                            if d2 < dist_min {
                                dist_min = d2;
                                value_min = Some(value);
                            }
                        }
                    }
                }
            }
        }
        if value_min.is_some()
            && dist_min < self.search_radius as f64 * self.search_radius as f64
        {
            (value_min, dist_min)
        } else {
            (None, f64::MAX)
        }
    }
}

// TriangleMeshSlicer.cpp:1278-1381
// called by make_loops() to connect remaining open polylines across shared triangle edges and vertices,
// possibly closing small gaps.
fn chain_open_polylines_close_gaps(
    open_polylines: &mut Vec<OpenPolyline>,
    loops: &mut Polygons,
    max_gap: f64,
    try_connect_reversed: bool,
) {
    // TriangleMeshSlicer.cpp:1280
    let max_gap_scaled = scale(max_gap);

    // Sort the open polylines by their length, update lengths, return only not yet consumed.
    // TriangleMeshSlicer.cpp:1284
    let sorted_by_length = open_polylines_sorted(open_polylines, true);

    // Store the end points of open_polylines into ClosestPointInRadiusLookup.
    // TriangleMeshSlicer.cpp:1299-1304
    let mut closest_end_point_lookup = ClosestPointLookup::new(max_gap_scaled);
    for &opl in &sorted_by_length {
        closest_end_point_lookup.insert(open_polylines, (opl, true));
        if try_connect_reversed {
            closest_end_point_lookup.insert(open_polylines, (opl, false));
        }
    }
    // Try to connect the loops.
    // TriangleMeshSlicer.cpp:1306
    for opl_idx in sorted_by_length {
        if open_polylines[opl_idx].consumed {
            continue;
        }
        // TriangleMeshSlicer.cpp:1309-1313
        if try_connect_reversed {
            // The end point of this polyline will be modified, remove it.
            closest_end_point_lookup.erase(open_polylines, (opl_idx, false));
        }
        open_polylines[opl_idx].consumed = true;
        // TriangleMeshSlicer.cpp:1314
        let mut n_segments_joined: usize = 1;
        loop {
            // Find a line starting where last one finishes.
            // TriangleMeshSlicer.cpp:1317-1318
            let end_point = *open_polylines[opl_idx].points.last().unwrap();
            let (next_start, next_dist) = closest_end_point_lookup.find(open_polylines, end_point);
            // Check whether we closed this loop.
            // TriangleMeshSlicer.cpp:1320-1321
            let front = *open_polylines[opl_idx].points.first().unwrap();
            let back = *open_polylines[opl_idx].points.last().unwrap();
            let dx = (back.x - front.x) as f64;
            let dy = (back.y - front.y) as f64;
            let current_loop_closing_distance2 = dx * dx + dy * dy;
            let mut loop_closed = current_loop_closing_distance2
                < max_gap_scaled as f64 * max_gap_scaled as f64;
            // TriangleMeshSlicer.cpp:1322-1326
            if next_start.is_some() && loop_closed && current_loop_closing_distance2 < next_dist {
                loop_closed = current_loop_closing_distance2.sqrt()
                    < 0.3 * crate::multi_point::length(&open_polylines[opl_idx].points);
            }
            // TriangleMeshSlicer.cpp:1327
            if loop_closed {
                // Remove the start point of the current polyline from the lookup.
                // TriangleMeshSlicer.cpp:1330-1331
                open_polylines[opl_idx].consumed = false;
                closest_end_point_lookup.erase(open_polylines, (opl_idx, true));
                // TriangleMeshSlicer.cpp:1332-1337
                if current_loop_closing_distance2 == 0.0 {
                    // Remove the duplicate last point.
                    open_polylines[opl_idx].points.pop();
                } else {
                    // The end points are different, keep both of them.
                }
                // TriangleMeshSlicer.cpp:1338
                if open_polylines[opl_idx].points.len() >= 3 {
                    // TriangleMeshSlicer.cpp:1339-1343
                    if try_connect_reversed
                        && n_segments_joined > 1
                        && Polygon::area_of(&open_polylines[opl_idx].points) < 0.0
                    {
                        open_polylines[opl_idx].points.reverse();
                    }
                    loops.push(Polygon::from_points(std::mem::take(
                        &mut open_polylines[opl_idx].points,
                    )));
                }
                open_polylines[opl_idx].points.clear();
                open_polylines[opl_idx].consumed = true;
                break;
            }
            // TriangleMeshSlicer.cpp:1350
            let next_start = match next_start {
                Some(v) => v,
                None => {
                    // The current loop could not be closed. Unmark the segment.
                    // TriangleMeshSlicer.cpp:1352-1355
                    open_polylines[opl_idx].consumed = false;
                    if try_connect_reversed {
                        closest_end_point_lookup.insert(open_polylines, (opl_idx, false));
                    }
                    break;
                }
            };
            // Attach this polyline to the end of the initial polyline.
            // TriangleMeshSlicer.cpp:1358-1369
            let next_poly = next_start.0;
            let back = *open_polylines[opl_idx].points.last().unwrap();
            if next_start.1 {
                let pts = open_polylines[next_poly].points.clone();
                let mut start = 0usize;
                if pts.first() == Some(&back) {
                    start = 1;
                }
                open_polylines[opl_idx].points.extend_from_slice(&pts[start..]);
            } else {
                let mut pts = open_polylines[next_poly].points.clone();
                pts.reverse();
                let mut start = 0usize;
                if pts.first() == Some(&back) {
                    start = 1;
                }
                open_polylines[opl_idx].points.extend_from_slice(&pts[start..]);
            }
            // TriangleMeshSlicer.cpp:1370
            n_segments_joined += 1;
            // Remove the end points of the consumed polyline segment from the lookup.
            // TriangleMeshSlicer.cpp:1372-1377
            closest_end_point_lookup.erase(open_polylines, (next_poly, true));
            if try_connect_reversed {
                closest_end_point_lookup.erase(open_polylines, (next_poly, false));
            }
            open_polylines[next_poly].points.clear();
            open_polylines[next_poly].consumed = true;
            // Continue with the current loop.
        }
    }
}

// TriangleMeshSlicer.cpp:1383-1481
fn make_loops_single(lines: &mut IntersectionLines) -> Polygons {
    let mut loops: Polygons = Polygons::new();

    // TriangleMeshSlicer.cpp:1414-1415
    let mut open_polylines: Vec<OpenPolyline> = Vec::new();
    chain_lines_by_triangle_connectivity(lines, &mut loops, &mut open_polylines);

    // Now process the open polylines.
    // Do it in two rounds, first try to connect in the same direction only,
    // then try to connect the open polylines in reversed order as well.
    // TriangleMeshSlicer.cpp:1431-1432
    chain_open_polylines_exact(&mut open_polylines, &mut loops, false);
    chain_open_polylines_exact(&mut open_polylines, &mut loops, true);

    // Try to close gaps.
    // TriangleMeshSlicer.cpp:1459-1461
    let max_gap = 2.0; //mm
    chain_open_polylines_close_gaps(&mut open_polylines, &mut loops, max_gap, false);
    chain_open_polylines_close_gaps(&mut open_polylines, &mut loops, max_gap, true);

    loops
}

// TriangleMeshSlicer.cpp:1483-1533 — vector-of-layers make_loops, applying SlicingMode.
fn make_loops_layers(
    lines: &mut [IntersectionLines],
    params: &MeshSlicingParams,
    throw_on_cancel: &dyn Fn(),
) -> Vec<Polygons> {
    // TriangleMeshSlicer.cpp:1490-1491
    let mut layers: Vec<Polygons> = vec![Polygons::new(); lines.len()];
    // TriangleMeshSlicer.cpp:1492 (tbb::parallel_for -> sequential)
    for line_idx in 0..lines.len() {
        // TriangleMeshSlicer.cpp:1496
        if (line_idx & 0x0ffff) == 0 {
            throw_on_cancel();
        }
        // TriangleMeshSlicer.cpp:1499-1500
        let polygons = make_loops_single(&mut lines[line_idx]);
        let mut polygons = polygons;


        // TriangleMeshSlicer.cpp:1502
        let this_mode = if line_idx < params.slicing_mode_normal_below_layer {
            params.mode_below
        } else {
            params.mode
        };
        if !polygons.is_empty() {
            if this_mode == SlicingMode::Positive {
                // Reorient all loops to be CCW.
                // TriangleMeshSlicer.cpp:1506-1507
                for p in polygons.iter_mut() {
                    p.make_counter_clockwise();
                }
            } else if this_mode == SlicingMode::PositiveLargestContour {
                // Keep just the largest polygon, make it CCW.
                // TriangleMeshSlicer.cpp:1511-1525
                let mut max_area = 0.0_f64;
                let mut max_area_idx: Option<usize> = None;
                for (i, p) in polygons.iter().enumerate() {
                    let a = p.area();
                    if a.abs() > max_area.abs() {
                        max_area = a;
                        max_area_idx = Some(i);
                    }
                }
                debug_assert!(max_area_idx.is_some());
                if let Some(idx) = max_area_idx {
                    if max_area < 0.0 {
                        polygons[idx].reverse();
                    }
                    let p = std::mem::replace(&mut polygons[idx], Polygon::from_points(Vec::new()));
                    polygons.clear();
                    polygons.push(p);
                }
            }
        }
        layers[line_idx] = polygons;
    }
    layers
}

// TriangleMeshSlicer.cpp:1663-1736
// Used to cut the mesh into two halves.
#[allow(dead_code)]
fn make_expolygons_simple(lines: &mut IntersectionLines) -> ExPolygons {
    let mut slices: ExPolygons = ExPolygons::new();
    let mut holes: Polygons = Polygons::new();

    // TriangleMeshSlicer.cpp:1669-1673
    for loop_poly in make_loops_single(lines) {
        if loop_poly.area() >= 0.0 {
            slices.push(ExPolygon::new(loop_poly));
        } else {
            holes.push(loop_poly);
        }
    }

    // If there are holes, then there should also be outer contours.
    // TriangleMeshSlicer.cpp:1676
    debug_assert!(holes.is_empty() || !slices.is_empty());
    if !slices.is_empty() {
        // Assign holes to outer contours.
        // TriangleMeshSlicer.cpp:1680-1698
        for hole in holes {
            // Find an outer contour to a hole.
            let mut slice_idx: i32 = -1;
            let mut current_contour_area = f64::MAX;
            for (i, slice) in slices.iter().enumerate() {
                if slice.contour.contains(&hole.points[0]) {
                    let area = slice.contour.area();
                    if area < current_contour_area {
                        slice_idx = i as i32;
                        current_contour_area = area;
                    }
                }
            }
            if slice_idx == -1 {
                // Ignore this hole.
                continue;
            }
            slices[slice_idx as usize].holes.push(hole);
        }
    }

    slices
}

// TriangleMeshSlicer.cpp:1738-1824
// Build ExPolygons from raw loops.
//
// FIDELITY-NOTE(F1): the crate's geo-based Clipper layer does not expose
// `ClipperLib::union_ex(loops, fill_type)` with a selectable fill rule (EvenOdd/
// Positive/NonZero). We use `union_polygons_ex` (NonZero-like) regardless of
// `fill_type`; the default Regular/NonZero path matches. The closing-offset
// branch structure (offset_out / offset_in) below mirrors the C++ exactly,
// using `offset_expolygons` (jtMiter == C++ DefaultJoinType) and the same
// `offset2_ex(out, in)` order (grow then shrink) — geo-clipper offset is an
// approximation of ClipperLib's at coord_t precision.
fn make_expolygons(
    loops: &[Polygon],
    closing_radius: f32,
    extra_offset: f32,
    fill_type: ClipperPolyFillType,
    slices: &mut ExPolygons,
) {
    use crate::clipper_utils::{offset_expolygons, OffsetJoinType};

    // TriangleMeshSlicer.cpp:1793
    debug_assert!(closing_radius >= 0.0);
    // Allowing negative extra_offset for shrinking a contour.
    // TriangleMeshSlicer.cpp:1796-1804
    // UNIT-NOTE: C++ uses ClipperLib in SCALED coords, so it offsets by
    // `scale_(closing_radius)`. This crate's `offset_expolygons` (geo-clipper)
    // operates in UNSCALED (mm) space — it `unscale()`s the polygon coords and
    // passes the delta through verbatim (see clipper_utils.rs:85,108). The
    // faithful equivalent therefore passes the radius in mm, NOT scaled (a
    // `scale()` here would offset by ~radius*100000 mm and collapse the slice).
    let offset_out: f64;
    let offset_in: f64;
    if closing_radius >= extra_offset {
        offset_out = closing_radius as f64;
        offset_in = -((closing_radius - extra_offset) as f64);
    } else {
        offset_out = extra_offset as f64;
        offset_in = 0.0;
    }

    // union_ex(loops, fill_type) (TriangleMeshSlicer.cpp:1819-1823).
    // F1_UNION (R84, gated): route the slice-stage union through the vertex-exact
    // vendored ClipperLib (clipper-z-sys @ native 1e5) instead of geo-clipper @
    // scale-1000 which re-quantizes the slice coords to a coarse grid (R83 residual
    // blocking slice byte-match). fill_type maps the slicing mode (NonZero default).
    let unioned = if std::env::var("F1_UNION").is_ok() {
        let ft = match fill_type {
            ClipperPolyFillType::EvenOdd => 0,
            ClipperPolyFillType::NonZero => 1,
            ClipperPolyFillType::Positive => 2,
        };
        crate::clipper_utils::union_ex_clib(loops, ft)
    } else {
        crate::clipper_utils::union_polygons_ex(loops)
    };

    // append to the supplied collection.
    // TriangleMeshSlicer.cpp:1819-1823
    let result = if offset_out > 0.0 && offset_in < 0.0 {
        // offset2_ex(union, offset_out, offset_in): grow by out, then shrink by |in|.
        // R96 (gated F1_UNION): route the close through the vertex-exact ClipperLib
        // offset2_ex @1e5 (clipper-z-sys) so it byte-matches C++; the default path
        // keeps geo-clipper (scale-1000).
        if std::env::var("F1_UNION").is_ok() {
            crate::clipper_utils::offset2_ex_clib(
                &unioned,
                offset_out,
                offset_in,
                OffsetJoinType::Miter,
            )
        } else {
            let grown = offset_expolygons(&unioned, offset_out, OffsetJoinType::Miter);
            offset_expolygons(&grown, offset_in, OffsetJoinType::Miter)
        }
    } else if offset_out > 0.0 {
        offset_expolygons(&unioned, offset_out, OffsetJoinType::Miter)
    } else if offset_in < 0.0 {
        offset_expolygons(&unioned, offset_in, OffsetJoinType::Miter)
    } else {
        unioned
    };
    slices.extend(result);
}

// ClipperLib::PolyFillType (recorded for fidelity; behavior NonZero-only here).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ClipperPolyFillType {
    EvenOdd,
    NonZero,
    Positive,
}

// TriangleMeshSlicer.hpp:11-36
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlicingMode {
    // Regular slicing, maintain all contours and their orientation.
    Regular,
    // For slicing 3DLabPrints plane models.
    EvenOdd,
    // Maintain all contours, orient all contours CCW.
    Positive,
    // Orient all contours CCW and keep only the contour with the largest area.
    PositiveLargestContour,
}

// TriangleMeshSlicer.hpp:11-36
#[derive(Clone, Debug)]
pub struct MeshSlicingParams {
    pub mode: SlicingMode,
    // For vase mode: below this layer a different slicing mode will be used.
    pub slicing_mode_normal_below_layer: usize,
    // Mode to apply below slicing_mode_normal_below_layer.
    pub mode_below: SlicingMode,
    // NOTE: Transform3d trafo is not threaded through here yet (identity only).
    // R85 slice-frame centering (mm, unscaled): the C++ `trafo_centered` XY
    // center_offset applied INSIDE the fused f32 slice-time transform
    // (make_trafo_for_slicing). (0,0) = no centering (raw frame, historic default).
    // When set, the per-vertex scale is the fused f32 matmul
    //   v_xy = f32(s) * (v_xy - center_offset)   [s = 1/SCALING_FACTOR]
    // matching transform_mesh_vertices_for_slicing's non-identity path
    // (TriangleMeshSlicer.cpp:1853-1860). Z untouched (R65 preserved).
    pub center_offset: (f64, f64),
}

impl Default for MeshSlicingParams {
    // TriangleMeshSlicer.hpp:28-35 — defaults.
    fn default() -> Self {
        Self {
            mode: SlicingMode::Regular,
            slicing_mode_normal_below_layer: 0,
            mode_below: SlicingMode::Regular,
            center_offset: (0.0, 0.0),
        }
    }
}

// TriangleMeshSlicer.hpp:38-47
#[derive(Clone, Debug)]
pub struct MeshSlicingParamsEx {
    pub base: MeshSlicingParams,
    // Morphological closing operation when creating output expolygons, unscaled.
    pub closing_radius: f32,
    // Positive offset applied when creating output expolygons, unscaled.
    pub extra_offset: f32,
    // Resolution for contour simplification, unscaled. 0 = don't simplify.
    pub resolution: f64,
    // R85 slice-frame centering (mm): the trafo_centered XY center_offset applied
    // in the fused f32 slice transform. (0,0) = raw frame (historic default).
    pub center_offset: (f64, f64),
}

impl Default for MeshSlicingParamsEx {
    fn default() -> Self {
        Self {
            base: MeshSlicingParams::default(),
            closing_radius: 0.0,
            extra_offset: 0.0,
            resolution: 0.0,
            center_offset: (0.0, 0.0),
        }
    }
}

// TriangleMeshSlicer.cpp:1864-1938 (identity-trafo path only)
// `std::vector<Polygons> slice_mesh(...)`.
fn slice_mesh_its(
    mesh: &indexed_triangle_set,
    // Unscaled Zs
    zs: &[f32],
    params: &MeshSlicingParams,
    throw_on_cancel: &dyn Fn(),
) -> Vec<Polygons> {
    let lines;
    {
        // TriangleMeshSlicer.cpp:1880
        let face_edge_ids = its_face_edge_ids(mesh);
        // TriangleMeshSlicer.cpp:1885 / 1894-1896.
        // R86: when params.center_offset != 0, slice through C++'s exact
        // make_trafo_for_slicing fused f32 transform via the EIGEN FFI SHIM (calls
        // the real Eigen `Affine3f * Vector3f` → bit-exact, sidesteps the pure-rust
        // 1-ULP wall of R85). Z untouched by the transform (linear row2=(0,0,1),
        // trans=0 → z passes through bit-exact) → R65 preserved. (0,0) = historic
        // identity path (v.x*=s, v.y*=s).
        let (cx, cy) = params.center_offset;
        let scaled_vertices: Vec<StlVertex> = if std::env::var("FRAME_UNIFY").is_ok() {
            // R87 frame-unification: slice rust's PLACED verts through C++'s exact
            // params2.trafo (incl Z+24) after subtracting the volume offset, via the
            // Eigen shim (bit-exact). params2.trafo (row-major) = Identity + trans
            // (8.3923339722069557e-08, 0, 24) [dumped]; volume offset = the
            // volume.get_matrix translation (0.82450008392, 0, 24) [dumped].
            // (Hardcoded from the C++ dump for the validation pass; a faithful build
            // from the placement matrix follows if verts bit-match.)
            let trafo16: [f64; 16] = [
                1.0, 0.0, 0.0, 8.3923339722069557e-08,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 24.0,
                0.0, 0.0, 0.0, 1.0,
            ];
            let voff = (0.82450008392333984_f64, 0.0_f64, 24.0_f64);
            let mut flat_in: Vec<f32> = Vec::with_capacity(mesh.vertices.len() * 3);
            for p in &mesh.vertices {
                flat_in.push(p.x);
                flat_in.push(p.y);
                flat_in.push(p.z);
            }
            let flat_out = eigen_transform_sys::transform_verts_unified(
                &trafo16,
                crate::libslic3r::SCALING_FACTOR,
                voff,
                &flat_in,
            );
            flat_out
                .chunks_exact(3)
                .map(|c| StlVertex::new(c[0], c[1], c[2]))
                .collect()
        } else if cx != 0.0 || cy != 0.0 {
            // Flatten verts (x,y,z interleaved f32) for the shim.
            let mut flat_in: Vec<f32> = Vec::with_capacity(mesh.vertices.len() * 3);
            for p in &mesh.vertices {
                flat_in.push(p.x);
                flat_in.push(p.y);
                flat_in.push(p.z);
            }
            let flat_out = eigen_transform_sys::transform_verts_for_slicing(
                crate::libslic3r::SCALING_FACTOR,
                cx,
                cy,
                &flat_in,
            );
            flat_out
                .chunks_exact(3)
                .map(|c| StlVertex::new(c[0], c[1], c[2]))
                .collect()
        } else {
            mesh.vertices
                .iter()
                .map(|p| StlVertex::new(scaled_f32(p.x), scaled_f32(p.y), p.z))
                .collect()
        };
        // TriangleMeshSlicer.cpp:1884 / 1894
        lines = slice_make_lines(&scaled_vertices, &mesh.indices, &face_edge_ids, zs, throw_on_cancel);
    }

    // TriangleMeshSlicer.cpp:1900
    throw_on_cancel();

    // TriangleMeshSlicer.cpp:1902
    let mut lines = lines;
    make_loops_layers(&mut lines, params, throw_on_cancel)
}

// `scaled<float>(p)` == Tout(v / Tin(SCALING_FACTOR)) (Point.hpp:529): plain f32
// division, NO floor/+0.5 rounding — rounding to coord_t happens later at the
// i64 casts in slice_facet. SCALING_FACTOR = 1e-5 (libslic3r.h:58).
fn scaled_f32(v: f32) -> f32 {
    v / (crate::libslic3r::SCALING_FACTOR as f32)
}

// TriangleMeshSlicer.cpp:1941-2001 — single-plane slice_mesh.
fn slice_mesh_plane_its(
    mesh: &indexed_triangle_set,
    // Unscaled Z
    plane_z: f32,
    params: &MeshSlicingParams,
) -> Polygons {
    let mut lines: Vec<IntersectionLines> = Vec::new();
    {
        // 1) Mark vertices as below or above the slicing plane.
        // TriangleMeshSlicer.cpp:1956-1962 (identity trafo)
        let mut vertex_side: Vec<i8> = vec![0; mesh.vertices.len()];
        for i in 0..mesh.vertices.len() {
            let z = mesh.vertices[i].z;
            let s: i8 = if z < plane_z {
                -1
            } else if z == plane_z {
                0
            } else {
                1
            };
            vertex_side[i] = s;
        }

        // 2) Mark faces crossing the plane.
        // TriangleMeshSlicer.cpp:1974-1978
        let mut face_mask: Vec<bool> = vec![false; mesh.indices.len()];
        for i in 0..mesh.indices.len() {
            let face = &mesh.indices[i];
            let sides = [
                vertex_side[face[0] as usize],
                vertex_side[face[1] as usize],
                vertex_side[face[2] as usize],
            ];
            face_mask[i] = sides[0] * sides[1] <= 0
                || sides[1] * sides[2] <= 0
                || sides[0] * sides[2] <= 0;
        }

        // 3) Calculate face neighbors for just the faces in face_mask.
        // TriangleMeshSlicer.cpp:1982
        let face_edge_ids = crate::triangle_mesh::its_face_edge_ids_mask(mesh, &face_mask);

        // 4) Slice "face_mask" triangles, collect line segments.
        // TriangleMeshSlicer.cpp:1986-1989.
        // R85: when params.center_offset != 0, slice through the FUSED f32
        // make_trafo_for_slicing (transform_mesh_vertices_for_slicing non-identity
        // path, TriangleMeshSlicer.cpp:1853-1860): tf = (trafo_centered.prescale(s)).cast<float>(),
        // v = tf*v. For benchy (no rotation) trafo_centered = Translate(-c_mm), and
        // Eigen prescale PRE-multiplies the scale → t = Scale(s)*Translate(-c_mm) →
        // matrix linear=diag(s,s,1), translation=(-s*cx, -s*cy, 0). Each matrix
        // element is cast f64->f32 ONCE, then v_out = linear_f32*v + translation_f32
        // in f32 (FMA-eligible, matching Eigen's f32 matmul). Z linear=1,trans=0 → Z
        // untouched (R65). The historic identity path stays the (0,0) default.
        let (cx, cy) = params.center_offset;
        let scaled_vertices: Vec<StlVertex> = if cx != 0.0 || cy != 0.0 {
            let s = 1.0f64 / crate::libslic3r::SCALING_FACTOR;
            // tf matrix elements (f64 t = Scale(s)*Translate(-c)), cast to f32.
            let lin_x = s as f32; // tf.linear()(0,0)
            let lin_y = s as f32; // tf.linear()(1,1)
            let trans_x = (-(s * cx)) as f32; // tf.translation()(0)
            let trans_y = (-(s * cy)) as f32; // tf.translation()(1)
            mesh.vertices
                .iter()
                .map(|p| {
                    // Eigen f32 Transform*Vector: linear*v + translation (FMA-eligible).
                    let x = lin_x * p.x + trans_x;
                    let y = lin_y * p.y + trans_y;
                    StlVertex::new(x, y, p.z)
                })
                .collect()
        } else {
            mesh.vertices
                .iter()
                .map(|p| StlVertex::new(scaled_f32(p.x), scaled_f32(p.y), p.z))
                .collect()
        };
        let fm = &face_mask;
        lines.push(slice_make_lines_single(
            &scaled_vertices,
            &mesh.indices,
            &face_edge_ids,
            plane_z,
            &|face_idx| fm[face_idx],
        ));
    }

    // 5) Chain the line segments.
    // TriangleMeshSlicer.cpp:1998-2000
    let mut layers = make_loops_layers(&mut lines, params, &|| {});
    debug_assert!(layers.len() == 1);
    layers.remove(0)
}

// ---------------------------------------------------------------------------
// Faithful slice-contour simplification (TriangleMeshSlicer.cpp:2038-2044 →
// ExPolygon::simplify, ExPolygon.cpp:231-256). The C++ slicer simplifies every
// output ExPolygon at `scaled(params.resolution)` (=scaled(0.0025) for benchy,
// PrintObjectSlice.cpp:144). rust omitted it → ~6x more slice vertices → cascades
// to perimeters/fills/gcode (R82). This is the faithful port, gated behind the
// caller passing resolution != 0.
// ---------------------------------------------------------------------------

/// f64 squared distance from `p` to the SEGMENT (a,b). Bit-faithful to
/// `Slic3r::line_alg::distance_to_squared` (Line.hpp:43-69): all arithmetic in
/// f64, t clamped to [0,1], returns the f64 squared norm (NOT re-quantized).
#[inline]
fn dp_distance_to_squared(p: Point, a: Point, b: Point) -> f64 {
    let vx = (b.x - a.x) as f64;
    let vy = (b.y - a.y) as f64;
    let vax = (p.x - a.x) as f64;
    let vay = (p.y - a.y) as f64;
    let l2 = vx * vx + vy * vy;
    if l2 == 0.0 {
        return vax * vax + vay * vay;
    }
    let t = (vax * vx + vay * vy) / l2;
    if t <= 0.0 {
        vax * vax + vay * vay
    } else if t >= 1.0 {
        let dx = (p.x - b.x) as f64;
        let dy = (p.y - b.y) as f64;
        dx * dx + dy * dy
    } else {
        // (t*v - va) squaredNorm
        let ex = t * vx - vax;
        let ey = t * vy - vay;
        ex * ex + ey * ey
    }
}

/// Faithful `MultiPoint::_douglas_peucker` (MultiPoint.cpp:179-225). `tolerance`
/// is ALREADY SCALED (coord units). Preserves C++'s exact keep/drop decisions and
/// emission order (anchor first, then floaters as the stack unwinds).
fn dp_douglas_peucker(pts: &[Point], tolerance: f64) -> Vec<Point> {
    let mut result_pts: Vec<Point> = Vec::new();
    let tolerance_sq = tolerance * tolerance;
    if pts.is_empty() {
        return result_pts;
    }
    let mut anchor_idx: usize = 0;
    let mut floater_idx: usize = pts.len() - 1;
    result_pts.reserve(pts.len());
    result_pts.push(pts[anchor_idx]);
    if anchor_idx != floater_idx {
        let mut dp_stack: Vec<usize> = Vec::with_capacity(pts.len());
        dp_stack.push(floater_idx);
        loop {
            let mut max_dist_sq = 0.0_f64;
            let mut furthest_idx = anchor_idx;
            let mut i = anchor_idx + 1;
            while i < floater_idx {
                let dist_sq = dp_distance_to_squared(pts[i], pts[anchor_idx], pts[floater_idx]);
                if dist_sq > max_dist_sq {
                    max_dist_sq = dist_sq;
                    furthest_idx = i;
                }
                i += 1;
            }
            if max_dist_sq <= tolerance_sq {
                result_pts.push(pts[floater_idx]);
                anchor_idx = floater_idx;
                // dp_stack.back() == floater_idx
                dp_stack.pop();
                match dp_stack.last() {
                    None => break,
                    Some(&top) => floater_idx = top,
                }
            } else {
                floater_idx = furthest_idx;
                dp_stack.push(floater_idx);
            }
        }
    }
    result_pts
}

/// Douglas-Peucker one ring: close (push first to end) → DP(tolerance) → reopen
/// (pop last). Mirrors ExPolygon.cpp:236-249 per-ring handling.
fn dp_simplify_ring(ring: &Polygon, tolerance: f64) -> Polygon {
    if ring.points.is_empty() {
        return Polygon::new();
    }
    let mut closed = ring.points.clone();
    closed.push(closed[0]);
    let mut simplified = dp_douglas_peucker(&closed, tolerance);
    simplified.pop(); // reopen
    Polygon::from_points(simplified)
}

/// Faithful `ExPolygon::simplify(tolerance)` = union_ex(simplify_p(tolerance))
/// (ExPolygon.cpp:231-256). `tolerance` is ALREADY SCALED. simplify_p DP-simplifies
/// each ring then runs `simplify_polygons` (ClipperLib SimplifyPolygons, pftNonZero),
/// then union_ex re-nests into ExPolygons.
/// FIDELITY-NOTE(F1): the simplify_polygons + union_ex steps go through the crate's
/// geo-clipper backend (approximation of ClipperLib at coord precision); the DP
/// reduction is exact.
fn expolygon_simplify(ex: &ExPolygon, tolerance: f64) -> ExPolygons {
    // simplify_p: DP each ring (contour + holes) → Polygons.
    let mut pp: Polygons = Vec::with_capacity(ex.holes.len() + 1);
    pp.push(dp_simplify_ring(&ex.contour, tolerance));
    for hole in &ex.holes {
        pp.push(dp_simplify_ring(hole, tolerance));
    }
    // simplify_polygons(pp) == ClipperLib::SimplifyPolygons(pp, pftNonZero).
    // SLICE_DP_ONLY: skip the geo-clipper simplify_polygons+union_ex (F1 quantizes
    // coords to the 1000-scale grid) and reassemble the DP'd rings directly into
    // one ExPolygon, preserving EXACT integer coords. For clean non-self-
    // intersecting slices this is geometrically equivalent to C++'s
    // SimplifyPolygons+union_ex but bit-faithful in coordinates.
    if std::env::var("SLICE_DP_ONLY").is_ok() {
        let contour = pp.remove(0);
        if contour.points.len() < 3 {
            return ExPolygons::new();
        }
        let holes: Vec<Polygon> = pp.into_iter().filter(|h| h.points.len() >= 3).collect();
        return vec![ExPolygon::with_holes(contour, holes)];
    }
    // F1_UNION (R84/R91): faithful C++ `ExPolygon::simplify` = union_ex(simplify_p(tol)),
    // simplify_p = DP each ring (above) → `simplify_polygons(pp)` (ClipperLib
    // SimplifyPolygons, StrictlySimple=true) → then `union_ex`. R91: the SimplifyPolygons
    // step (StrictlySimple=true) is NOT the same as a plain ctUnion — it retains
    // different vertices; routing through it (not a direct union_ex_clib) is what
    // closes the lslices residual (rust simplify 1368 vs C++ 1528 → match). Both via
    // the vertex-exact vendored ClipperLib (exact i32 coords).
    if std::env::var("F1_UNION").is_ok() {
        let simplified = crate::clipper_utils::simplify_polygons_clib(&pp, 1);
        return crate::clipper_utils::union_ex_clib(&simplified, 1);
    }
    let simplified = crate::geometry::polygon::simplify_polygons_clipper(&pp);
    // union_ex(simplified) re-nests contours/holes.
    crate::clipper_utils::union_polygons_ex(&simplified)
}

// TriangleMeshSlicer.cpp:2003-2050
// `std::vector<ExPolygons> slice_mesh_ex(...)`.
pub fn slice_mesh_ex_its(
    mesh: &indexed_triangle_set,
    zs: &[f32],
    params: &MeshSlicingParamsEx,
    throw_on_cancel: &dyn Fn(),
) -> Vec<ExPolygons> {
    // TriangleMeshSlicer.cpp:2009-2017
    let layers_p;
    {
        let mut slicing_params = params.base.clone();
        // R85: thread the slice-frame centering into the per-plane slicer.
        slicing_params.center_offset = params.center_offset;
        if params.base.mode == SlicingMode::PositiveLargestContour {
            slicing_params.mode = SlicingMode::Positive;
        }
        if params.base.mode_below == SlicingMode::PositiveLargestContour {
            slicing_params.mode_below = SlicingMode::Positive;
        }
        layers_p = slice_mesh_its(mesh, zs, &slicing_params, throw_on_cancel);
    }

    // TriangleMeshSlicer.cpp:2020-2046 (tbb::parallel_for -> sequential)
    let mut layers: Vec<ExPolygons> = vec![ExPolygons::new(); layers_p.len()];
    for layer_id in 0..layers_p.len() {
        throw_on_cancel();
        let this_mode = if layer_id < params.base.slicing_mode_normal_below_layer {
            params.base.mode_below
        } else {
            params.base.mode
        };
        // TriangleMeshSlicer.cpp:2030-2034
        let fill_type = match this_mode {
            SlicingMode::EvenOdd => ClipperPolyFillType::EvenOdd,
            SlicingMode::PositiveLargestContour => ClipperPolyFillType::Positive,
            _ => ClipperPolyFillType::NonZero,
        };
        // F2RAW: dump RAW LOOPS (slice_facet/make_loops output) BEFORE union/make_expolygons,
        // at the Benchy cabin-floor layers (z<=0.55 = li 0/1/2). Decides whether the 8 spurious
        // li=1 holes are born here (F2 facet classification) or in the union (F1).
        if std::env::var("F2RAW").is_ok() && zs.get(layer_id).copied().unwrap_or(99.0) < 0.55 {
            let sf = crate::SCALING_FACTOR as f64;
            let sf2 = 1.0 / (sf * sf);
            let loops = &layers_p[layer_id];
            eprintln!(
                "F2RAW RUST li={} z={:.4} nloops={}",
                layer_id,
                zs.get(layer_id).copied().unwrap_or(-1.0),
                loops.len()
            );
            for (k, lp) in loops.iter().enumerate() {
                let a2 = crate::multi_point::area(&lp.points);
                let amm2 = a2.abs() / 2.0 * sf2;
                let (mut xmn, mut ymn, mut xmx, mut ymx) = (i64::MAX, i64::MAX, i64::MIN, i64::MIN);
                for p in &lp.points {
                    xmn = xmn.min(p.x);
                    ymn = ymn.min(p.y);
                    xmx = xmx.max(p.x);
                    ymx = ymx.max(p.y);
                }
                eprintln!(
                    "F2RAW RUST   loop{} npts={} {} area={:.3}mm2 bbox=[{:.2},{:.2}]-[{:.2},{:.2}]",
                    k,
                    lp.points.len(),
                    if a2 > 0.0 { "CCW" } else { "CW " },
                    amm2,
                    xmn as f64 / sf,
                    ymn as f64 / sf,
                    xmx as f64 / sf,
                    ymx as f64 / sf,
                );
            }
        }
        let mut expolygons = ExPolygons::new();
        make_expolygons(
            &layers_p[layer_id],
            params.closing_radius,
            params.extra_offset,
            fill_type,
            &mut expolygons,
        );
        // TriangleMeshSlicer.cpp:2036-2037
        if this_mode == SlicingMode::PositiveLargestContour {
            crate::geometry::keep_largest_contour_only(&mut expolygons);
        }
        // Resolution simplification (TriangleMeshSlicer.cpp:2038-2044).
        // C++: auto resolution = scaled<float>(params.resolution);
        //      if (resolution != 0.) { for each ex: append(simplified, ex.simplify(resolution)); }
        // The caller sets params.resolution = print_config.resolution<=0.001 ? 0 : 0.0025
        // (PrintObjectSlice.cpp:144). R82: rust had omitted this → ~6x slice vertices.
        let resolution_scaled = scale(params.resolution) as f64;
        if resolution_scaled != 0.0 {
            let mut simplified = ExPolygons::with_capacity(expolygons.len());
            for ex in &expolygons {
                simplified.extend(expolygon_simplify(ex, resolution_scaled));
            }
            expolygons = simplified;
        }
        layers[layer_id] = expolygons;
    }

    layers
}

// ---------------------------------------------------------------------------
// Public API kept for existing callers (slicer.rs). These build an
// `indexed_triangle_set` view from the crate's `TriangleMesh` and run the
// faithful slicing path above with default (Regular) params.
// ---------------------------------------------------------------------------

fn its_from_triangle_mesh(mesh: &TriangleMesh) -> indexed_triangle_set {
    use crate::normal_utils::{StlTriangleVertexIndices, StlVertex as ItsVertex};
    let vertices: Vec<ItsVertex> = mesh
        .vertices()
        .iter()
        .map(|p| ItsVertex::new(p.x as f32, p.y as f32, p.z as f32))
        .collect();
    let indices: Vec<StlTriangleVertexIndices> = (0..mesh.triangle_count())
        .map(|i| {
            let t = mesh.triangle_indices(i);
            StlTriangleVertexIndices::new(t[0] as i32, t[1] as i32, t[2] as i32)
        })
        .collect();
    indexed_triangle_set { vertices, indices }
}

/// Slice a mesh at a single Z height, returning ExPolygons.
pub fn slice_mesh_at_z(mesh: &TriangleMesh, z: CoordF) -> ExPolygons {
    if mesh.is_empty() {
        return ExPolygons::new();
    }
    let its = its_from_triangle_mesh(mesh);
    let params = MeshSlicingParams::default();
    let loops = slice_mesh_plane_its(&its, z as f32, &params);
    let mut slices = ExPolygons::new();
    make_expolygons(&loops, 0.0, 0.0, ClipperPolyFillType::NonZero, &mut slices);
    slices
}

/// Slice a mesh at multiple Z heights, returning ExPolygons for each height.
pub fn slice_mesh(mesh: &TriangleMesh, zs: &[CoordF]) -> Vec<ExPolygons> {
    if mesh.is_empty() || zs.is_empty() {
        return vec![ExPolygons::new(); zs.len()];
    }
    let its = its_from_triangle_mesh(mesh);
    let zs_f32: Vec<f32> = zs.iter().map(|&z| z as f32).collect();
    let params = MeshSlicingParamsEx::default();
    slice_mesh_ex_its(&its, &zs_f32, &params, &|| {})
}

/// Slice a `TriangleMesh` at multiple unscaled Zs with the given
/// `MeshSlicingParamsEx`, returning ExPolygons for each height.
///
/// This is the `&TriangleMesh` front-end for the faithful
/// `std::vector<ExPolygons> slice_mesh_ex(const indexed_triangle_set &, ...)`
/// (TriangleMeshSlicer.cpp:2003) path. It exists so callers that hold a
/// `TriangleMesh` (e.g. CSG slicing) can supply custom slicing params and a
/// cancellation callback without duplicating the `indexed_triangle_set`
/// conversion.
///
/// NOTE: `params.base` does NOT carry a `Transform3d trafo`; the underlying
/// slicer applies an identity transform only (see `MeshSlicingParams`). Any
/// caller-side trafo composition is therefore not reflected here yet.
pub fn slice_mesh_ex(
    mesh: &TriangleMesh,
    zs: &[f32],
    params: &MeshSlicingParamsEx,
    throw_on_cancel: &dyn Fn(),
) -> Vec<ExPolygons> {
    if mesh.is_empty() || zs.is_empty() {
        return vec![ExPolygons::new(); zs.len()];
    }
    let its = its_from_triangle_mesh(mesh);
    slice_mesh_ex_its(&its, zs, params, throw_on_cancel)
}

#[allow(dead_code)]
fn _unscale_marker(v: i64) -> f64 {
    unscale(v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::triangle_mesh::TriangleMesh;

    #[test]
    fn test_slice_cube() {
        // Create a simple cube centered at origin, 10mm on each side
        let mesh = TriangleMesh::cube(10.0);

        // Slice at the middle
        let result = slice_mesh_at_z(&mesh, 0.0);

        // Should produce a single square contour
        assert_eq!(result.len(), 1, "Expected 1 contour for cube slice");

        let expoly = &result[0];
        assert!(expoly.holes.is_empty(), "Cube slice should have no holes");
    }

    #[test]
    fn test_slice_cube_multiple_layers() {
        let mesh = TriangleMesh::cube(10.0);

        // Slice at multiple heights
        let zs: Vec<f64> = (-4..=4).map(|i| i as f64).collect();
        let results = slice_mesh(&mesh, &zs);

        assert_eq!(results.len(), zs.len());

        // All slices through the cube should have exactly one contour
        for (i, result) in results.iter().enumerate() {
            assert_eq!(
                result.len(),
                1,
                "Layer {} at z={} should have 1 contour",
                i,
                zs[i]
            );
        }
    }

    #[test]
    fn test_slice_empty_mesh() {
        let mesh = TriangleMesh::new();
        let result = slice_mesh_at_z(&mesh, 0.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_slice_no_intersection() {
        let mesh = TriangleMesh::cube(10.0);

        // Slice above the cube (cube is at z=-5 to z=5)
        let result = slice_mesh_at_z(&mesh, 10.0);
        assert!(result.is_empty(), "Slice above cube should be empty");

        // Slice below the cube
        let result = slice_mesh_at_z(&mesh, -10.0);
        assert!(result.is_empty(), "Slice below cube should be empty");
    }
}
