//! FFI binding to the REAL `boost::polygon` Voronoi builder (see
//! `shim/boost_voronoi_shim.cpp`).
//!
//! Motivation (R772): both the multi-material segmentation and the Arachne
//! skeletal trapezoidation consume Voronoi diagrams. The rust `boostvoronoi`
//! crate reproduces the algorithm but not the exact f64 vertex arithmetic of
//! the C++ `boost::polygon` headers compiled at `-O3`; the resulting ULP drift
//! welds/gaps piece boundaries (llround flips) and shifts every Arachne
//! junction (AWALL census: 0/726 exact calls with byte-exact inputs). This
//! shim runs the same headers with the same codegen, giving byte-exact
//! diagrams by construction — the eigen-transform-sys / clipper-z-sys pattern.

use std::os::raw::c_double;

#[repr(C)]
#[derive(Clone, Copy)]
struct BvDiagramRaw {
    vert_xy: *mut c_double,
    num_vertices: i32,
    edges: *mut i32,
    edge_flags: *mut u8,
    num_edges: i32,
    cells: *mut i32,
    cell_flags: *mut u8,
    num_cells: i32,
}

extern "C" {
    fn bv_construct_segments(seg_xy: *const i32, num_segs: i32) -> BvDiagramRaw;
    fn bv_free(d: BvDiagramRaw);
}

/// One Voronoi edge; all pointer fields are indices (`-1` ⇒ none ⇒ `None`).
#[derive(Clone, Copy, Debug)]
pub struct Edge {
    pub vertex0: i32,
    pub vertex1: i32,
    pub twin: i32,
    pub next: i32,
    pub prev: i32,
    pub cell: i32,
    pub is_primary: bool,
    pub is_curved: bool,
    pub is_finite: bool,
}

/// One Voronoi cell.
#[derive(Clone, Copy, Debug)]
pub struct Cell {
    pub source_index: usize,
    /// Index into `edges`, or -1.
    pub incident_edge: i32,
    /// Raw `boost::polygon::SourceCategory` value:
    /// 0 = SINGLE_POINT, 1 = SEGMENT_START_POINT, 2 = SEGMENT_END_POINT,
    /// 3 = INITIAL_SEGMENT, 4 = REVERSE_SEGMENT.
    pub source_category: i32,
    pub contains_point: bool,
    pub contains_segment: bool,
    pub is_degenerate: bool,
}

/// An owned, index-based copy of a `boost::polygon::voronoi_diagram<double>`,
/// in the exact container order the native engine iterates.
pub struct Diagram {
    /// (x, y) per vertex, `vd.vertices()` order.
    pub vertices: Vec<(f64, f64)>,
    pub edges: Vec<Edge>,
    pub cells: Vec<Cell>,
}

/// Construct the Voronoi diagram of `segments` (scaled integer endpoints,
/// `(x1, y1, x2, y2)` per segment — must fit i32, as everywhere else in the
/// scaled-coordinate pipeline).
pub fn construct_segments(segments: &[[i32; 4]]) -> Diagram {
    let flat: &[i32] =
        unsafe { std::slice::from_raw_parts(segments.as_ptr() as *const i32, segments.len() * 4) };
    let raw = unsafe { bv_construct_segments(flat.as_ptr(), segments.len() as i32) };

    let mut out = Diagram {
        vertices: Vec::with_capacity(raw.num_vertices.max(0) as usize),
        edges: Vec::with_capacity(raw.num_edges.max(0) as usize),
        cells: Vec::with_capacity(raw.num_cells.max(0) as usize),
    };
    unsafe {
        if !raw.vert_xy.is_null() {
            let v = std::slice::from_raw_parts(raw.vert_xy, 2 * raw.num_vertices as usize);
            for i in 0..raw.num_vertices as usize {
                out.vertices.push((v[2 * i], v[2 * i + 1]));
            }
        }
        if !raw.edges.is_null() && !raw.edge_flags.is_null() {
            let e = std::slice::from_raw_parts(raw.edges, 6 * raw.num_edges as usize);
            let f = std::slice::from_raw_parts(raw.edge_flags, raw.num_edges as usize);
            for i in 0..raw.num_edges as usize {
                let r = &e[6 * i..6 * i + 6];
                out.edges.push(Edge {
                    vertex0: r[0],
                    vertex1: r[1],
                    twin: r[2],
                    next: r[3],
                    prev: r[4],
                    cell: r[5],
                    is_primary: f[i] & 1 != 0,
                    is_curved: f[i] & 2 != 0,
                    is_finite: f[i] & 4 != 0,
                });
            }
        }
        if !raw.cells.is_null() && !raw.cell_flags.is_null() {
            let c = std::slice::from_raw_parts(raw.cells, 3 * raw.num_cells as usize);
            let f = std::slice::from_raw_parts(raw.cell_flags, raw.num_cells as usize);
            for i in 0..raw.num_cells as usize {
                let r = &c[3 * i..3 * i + 3];
                out.cells.push(Cell {
                    source_index: r[0] as usize,
                    incident_edge: r[1],
                    source_category: r[2],
                    contains_point: f[i] & 1 != 0,
                    contains_segment: f[i] & 2 != 0,
                    is_degenerate: f[i] & 4 != 0,
                });
            }
        }
        bv_free(raw);
    }
    out
}

// ---------------------------------------------------------------------------
// Test-only exports (`shim/boost_voronoi_test_exports.cpp`): single predicates
// and functors of `detail/voronoi_predicates.hpp`, for comparing a port with
// them one call at a time. Nothing in the engine calls these.
// ---------------------------------------------------------------------------

/// A `site_event<int32>`: `[x0, y0, x1, y1, sorted_index, is_inverse]`, the points as the
/// site holds them (`point0`, `point1`).
pub type TestSite = [i32; 6];

extern "C" {
    fn bv_test_circle_formation(s1: *const i32, s2: *const i32, s3: *const i32, out: *mut c_double) -> i32;
    fn bv_test_lazy_circle(kind: i32, index: i32, s1: *const i32, s2: *const i32, s3: *const i32, out: *mut c_double);
    fn bv_test_mp_circle(kind: i32, index: i32, s1: *const i32, s2: *const i32, s3: *const i32, out: *mut c_double);
    fn bv_test_distance(left: *const i32, right: *const i32, px: i32, py: i32, reverse: i32, out: *mut c_double) -> i32;
    fn bv_test_node_comparison(l1: *const i32, r1: *const i32, l2: *const i32, r2: *const i32) -> i32;
    fn bv_test_event_comparison(s1: *const i32, s2: *const i32) -> i32;
}

/// `circle_formation_predicate::operator()`: whether the three sites form a circle event, and
/// its `[x, y, lower_x]` (zeros where nothing was computed).
pub fn test_circle_formation(s1: &TestSite, s2: &TestSite, s3: &TestSite) -> (bool, [f64; 3]) {
    let mut out = [0.0f64; 3];
    let r = unsafe { bv_test_circle_formation(s1.as_ptr(), s2.as_ptr(), s3.as_ptr(), out.as_mut_ptr()) };
    (r != 0, out)
}

/// `lazy_circle_formation_functor`: `kind` 0 = ppp, 1 = pps, 2 = pss, 3 = sss; `index` is the
/// `segment_index` of pps and the `point_index` of pss. Returns `[x, y, lower_x]`.
pub fn test_lazy_circle(kind: i32, index: i32, s1: &TestSite, s2: &TestSite, s3: &TestSite) -> [f64; 3] {
    let mut out = [0.0f64; 3];
    unsafe { bv_test_lazy_circle(kind, index, s1.as_ptr(), s2.as_ptr(), s3.as_ptr(), out.as_mut_ptr()) };
    out
}

/// `mp_circle_formation_functor` with every coordinate recomputed; arguments as [`test_lazy_circle`].
pub fn test_mp_circle(kind: i32, index: i32, s1: &TestSite, s2: &TestSite, s3: &TestSite) -> [f64; 3] {
    let mut out = [0.0f64; 3];
    unsafe { bv_test_mp_circle(kind, index, s1.as_ptr(), s2.as_ptr(), s3.as_ptr(), out.as_mut_ptr()) };
    out
}

/// `distance_predicate`: `(operator()(left, right, point), fast_ps(left, right, point, reverse)
/// as -1 / 0 / 1 when left is a point and right a segment, find_distance_to_point_arc(left,
/// point), find_distance_to_segment_arc(right, point))`.
pub fn test_distance(left: &TestSite, right: &TestSite, point: [i32; 2], reverse: bool) -> (bool, Option<i32>, f64, f64) {
    let mut out = [0.0f64; 2];
    let r = unsafe { bv_test_distance(left.as_ptr(), right.as_ptr(), point[0], point[1], reverse as i32, out.as_mut_ptr()) };
    let fast = r >> 1;
    (r & 1 != 0, if fast == 0 { None } else { Some(fast - 2) }, out[0], out[1])
}

/// `node_comparison_predicate` on the beach line nodes `(l1, r1)` and `(l2, r2)`.
pub fn test_node_comparison(l1: &TestSite, r1: &TestSite, l2: &TestSite, r2: &TestSite) -> bool {
    unsafe { bv_test_node_comparison(l1.as_ptr(), r1.as_ptr(), l2.as_ptr(), r2.as_ptr()) != 0 }
}

/// `event_comparison_predicate` on two sites.
pub fn test_event_comparison(s1: &TestSite, s2: &TestSite) -> bool {
    unsafe { bv_test_event_comparison(s1.as_ptr(), s2.as_ptr()) != 0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn square_diagram_smoke() {
        // A 1000-unit square as 4 CCW segments.
        let segs = [
            [0, 0, 1000, 0],
            [1000, 0, 1000, 1000],
            [1000, 1000, 0, 1000],
            [0, 1000, 0, 0],
        ];
        let d = construct_segments(&segs);
        // 4 segment cells + 4 endpoint cells.
        assert_eq!(d.cells.len(), 8);
        assert!(d.vertices.len() >= 4);
        assert!(!d.edges.is_empty());
        // Center vertex must exist at (500, 500).
        assert!(d
            .vertices
            .iter()
            .any(|&(x, y)| (x - 500.0).abs() < 1e-9 && (y - 500.0).abs() < 1e-9));
        // Twin symmetry.
        for (i, e) in d.edges.iter().enumerate() {
            assert!(e.twin >= 0);
            assert_eq!(d.edges[e.twin as usize].twin as usize, i);
        }
    }
}
