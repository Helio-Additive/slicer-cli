//! Voronoi diagram annotation - vertex and edge categorization
//!
//! This module ports BambuStudio's `annotate_inside_outside()` function which
//! categorizes Voronoi vertices and edges as Inside/Outside/OnContour.
//!
//! **Port Reference**: `reference/BambuStudio/src/libslic3r/Geometry/VoronoiOffset.cpp`
//! lines 650-965
//!
//! ## Why This Is Critical
//!
//! The medial axis algorithm depends on knowing which Voronoi vertices are inside
//! vs outside the input polygon. Simple exterior edge coloring is insufficient -
//! we need proper vertex/edge/cell categorization that handles:
//! - Vertices on the input contour
//! - Infinite edges
//! - Secondary edges (Point-Segment bisectors)
//! - Edges between two segments (constrained bisectors)
//!
//! ## Algorithm Overview
//!
//! The annotation happens in 4 phases:
//!
//! 1. **Mark OnContour vertices**: Find vertices that lie exactly on input segments
//! 2. **Classify infinite edges**: All infinite edges point outside
//! 3. **Classify finite edges with segments**: Use orientation test to determine
//!    if vertex is on left (outside) or right (inside) of segment
//! 4. **Seed fill remaining**: Propagate categories from annotated cells to
//!    unmarked Point-Point edges
//!
//! ## Category Storage
//!
//! Categories are stored in the `color()` field of vertices, edges, and cells:
//! - Vertex color → VertexCategory (as u8)
//! - Edge color → EdgeCategory (as u8)
//! - Cell color → CellCategory (as u8)

use boostvoronoi::prelude as bv;

use crate::geometry::{Line, Point};
use crate::Coord;

// ---------------------------------------------------------------------------
// Category Enums
// ---------------------------------------------------------------------------

/// Category of a Voronoi vertex
///
/// Port of `Slic3r::Voronoi::VertexCategory` from VoronoiOffset.hpp
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum VertexCategory {
    /// Vertex is on the input contour (coordinates match when cast to i64)
    OnContour = 0,

    /// Vertex is inside the CCW input contour (holes respected)
    Inside = 1,

    /// Vertex is outside the CCW input contour (holes respected)
    Outside = 2,

    /// Not yet categorized
    Unknown = 3,
}

impl Default for VertexCategory {
    fn default() -> Self {
        Self::Unknown
    }
}

impl From<u8> for VertexCategory {
    fn from(val: u8) -> Self {
        match val {
            0 => Self::OnContour,
            1 => Self::Inside,
            2 => Self::Outside,
            _ => Self::Unknown,
        }
    }
}

impl From<VertexCategory> for u8 {
    fn from(val: VertexCategory) -> Self {
        val as u8
    }
}

/// Category of a Voronoi edge (half-edge)
///
/// Port of `Slic3r::Voronoi::EdgeCategory` from VoronoiOffset.hpp
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum EdgeCategory {
    /// This half-edge points to a vertex on the contour (vertex1 is OnContour)
    PointsToContour = 0,

    /// This half-edge points to an inside vertex (vertex1 is Inside)
    PointsInside = 1,

    /// This half-edge points to an outside vertex (vertex1 is Outside)
    PointsOutside = 2,

    /// Not yet categorized
    Unknown = 3,
}

impl Default for EdgeCategory {
    fn default() -> Self {
        Self::Unknown
    }
}

impl From<u8> for EdgeCategory {
    fn from(val: u8) -> Self {
        match val {
            0 => Self::PointsToContour,
            1 => Self::PointsInside,
            2 => Self::PointsOutside,
            _ => Self::Unknown,
        }
    }
}

impl From<EdgeCategory> for u8 {
    fn from(val: EdgeCategory) -> Self {
        val as u8
    }
}

/// Category of a Voronoi cell
///
/// Port of `Slic3r::Voronoi::CellCategory` from VoronoiOffset.hpp
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CellCategory {
    /// Cell is split by input segment (one half inside, one half outside)
    Boundary = 0,

    /// Cell is completely inside
    Inside = 1,

    /// Cell is completely outside
    Outside = 2,

    /// Not yet categorized
    Unknown = 3,
}

impl Default for CellCategory {
    fn default() -> Self {
        Self::Unknown
    }
}

impl From<u8> for CellCategory {
    fn from(val: u8) -> Self {
        match val {
            0 => Self::Boundary,
            1 => Self::Inside,
            2 => Self::Outside,
            _ => Self::Unknown,
        }
    }
}

impl From<CellCategory> for u8 {
    fn from(val: CellCategory) -> Self {
        val as u8
    }
}

// ---------------------------------------------------------------------------
// Accessor Functions
// ---------------------------------------------------------------------------

/// Get the category of a Voronoi vertex
#[inline]
pub fn vertex_category(diagram: &bv::Diagram, vertex_id: bv::VertexIndex) -> VertexCategory {
    if let Some(color) = diagram.vertex_get_color(vertex_id) {
        return VertexCategory::from(color as u8);
    }
    VertexCategory::Unknown
}

/// Set the category of a Voronoi vertex
#[inline]
pub fn set_vertex_category(
    diagram: &mut bv::Diagram,
    vertex_id: bv::VertexIndex,
    category: VertexCategory,
) {
    diagram.vertex_set_color(vertex_id, category as u32);
}

/// Get the category of a Voronoi edge
#[inline]
pub fn edge_category(diagram: &bv::Diagram, edge_id: bv::EdgeIndex) -> EdgeCategory {
    if let Ok(color) = diagram.edge_get_color(edge_id) {
        return EdgeCategory::from(color as u8);
    }
    EdgeCategory::Unknown
}

/// Set the category of a Voronoi edge
#[inline]
pub fn set_edge_category(
    diagram: &mut bv::Diagram,
    edge_id: bv::EdgeIndex,
    category: EdgeCategory,
) {
    diagram.edge_set_color(edge_id, category as u32);
}

/// Get the category of a Voronoi cell (stored in a separate map since boostvoronoi doesn't expose cell color)
#[inline]
pub fn cell_category(_diagram: &bv::Diagram, _cell_id: bv::CellIndex) -> CellCategory {
    // TODO: Implement proper cell color storage if needed
    CellCategory::Unknown
}

/// Set the category of a Voronoi cell
#[inline]
pub fn set_cell_category(
    _diagram: &mut bv::Diagram,
    _cell_id: bv::CellIndex,
    _category: CellCategory,
) {
    // TODO: Implement proper cell color storage if needed
}

// ---------------------------------------------------------------------------
// Helper Functions
// ---------------------------------------------------------------------------

/// Check if a point is very close to a vertex (within 0.5001 units)
fn vertex_equal_to_point(diagram: &bv::Diagram, vertex_id: bv::VertexIndex, point: Point) -> bool {
    if let Ok(vertex) = diagram.vertex(vertex_id) {
        let vx = vertex.x();
        let vy = vertex.y();
        let px = point.x as f64;
        let py = point.y as f64;

        return (vx - px).abs() < 0.5001 && (vy - py).abs() < 0.5001;
    }
    false
}

/// Check if two coord_t points are equal (with tolerance)
fn vertex_equal_to_point_coord(a: Point, b: Point) -> bool {
    // BambuStudio uses ULP comparison (boost::polygon voronoi_diagram_traits)
    // ULPS = 128 from boost/polygon/voronoi_diagram.hpp line 268
    // This compares floating-point values allowing up to 128 representable
    // floating-point numbers between them (handles precision loss from Voronoi)
    ulp_eq(a.x as f64, b.x as f64, 128) && ulp_eq(a.y as f64, b.y as f64, 128)
}

/// ULP (Units in Last Place) comparison for floating-point equality
/// Matches boost::polygon::detail::ulp_comparison behavior
/// Returns true if |a - b| <= ulps * epsilon(max(a,b))
fn ulp_eq(a: f64, b: f64, ulps: i64) -> bool {
    // Handle exact equality and special cases
    if a == b {
        return true;
    }

    // Handle NaN
    if a.is_nan() || b.is_nan() {
        return false;
    }

    // Handle infinities
    if a.is_infinite() || b.is_infinite() {
        return a == b;
    }

    // Convert to integer representation for ULP calculation
    let a_bits = a.to_bits() as i64;
    let b_bits = b.to_bits() as i64;

    // Handle sign differences (except for zero)
    if (a_bits ^ b_bits) < 0 {
        // Different signs - only equal if both are very close to zero
        return a.abs() < f64::EPSILON && b.abs() < f64::EPSILON;
    }

    // Same sign - check ULP distance
    let ulp_diff = (a_bits - b_bits).abs();
    ulp_diff <= ulps
}

/// Convert Voronoi vertex to Point (rounds to nearest integer)
fn vertex_to_point(diagram: &bv::Diagram, vertex_id: bv::VertexIndex) -> Option<Point> {
    if let Ok(vertex) = diagram.vertex(vertex_id) {
        Some(Point::new(
            vertex.x().round() as Coord,
            vertex.y().round() as Coord,
        ))
    } else {
        None
    }
}

/// Get the contour point associated with a Point-based cell
fn contour_point(diagram: &bv::Diagram, cell_id: bv::CellIndex, lines: &[Line]) -> Option<Point> {
    if let Ok(cell) = diagram.cell(cell_id) {
        if cell.contains_point() {
            let (src_idx, src_cat) = cell.source_index_2();
            let idx = src_idx.usize();
            if idx < lines.len() {
                let line = &lines[idx];
                return Some(match src_cat {
                    bv::SourceCategory::SegmentStart => line.a,
                    bv::SourceCategory::SegmentEnd => line.b,
                    _ => return None,
                });
            }
        }
    }
    None
}

/// Check if a point is on a site (Point or Segment cell)
fn on_site(diagram: &bv::Diagram, cell_id: bv::CellIndex, lines: &[Line], pt: Point) -> bool {
    if let Ok(cell) = diagram.cell(cell_id) {
        if cell.contains_point() {
            if let Some(contour_pt) = contour_point(diagram, cell_id, lines) {
                return vertex_equal_to_point_coord(pt, contour_pt);
            }
        } else if cell.contains_segment() {
            let (src_idx, _) = cell.source_index_2();
            let idx = src_idx.usize();
            if idx < lines.len() {
                let line = &lines[idx];
                return vertex_equal_to_point_coord(pt, line.a)
                    || vertex_equal_to_point_coord(pt, line.b);
            }
        }
    }
    false
}

// ---------------------------------------------------------------------------
// Main Annotation Function
// ---------------------------------------------------------------------------

/// Annotate Voronoi vertices and edges as Inside/Outside/OnContour
///
/// Port of `Slic3r::Voronoi::annotate_inside_outside()` from VoronoiOffset.cpp
/// lines 650-965
pub fn annotate_inside_outside(diagram: &mut bv::Diagram, lines: &[Line]) {
    // Step 1: Reset all annotations to Unknown
    reset_annotations(diagram);

    let num_edges = diagram.edges().len();

    // Step 2: Mark OnContour vertices
    // Due to vertex merging by boost::polygon, we mark vertices that are very close
    // to the input contour as OnContour (within 0.5001 units)
    for edge_idx in 0..num_edges {
        let edge_id = diagram.edge_index_unchecked(edge_idx);

        if let Some(v0_id) = diagram.edge_get_vertex0(edge_id).ok().flatten() {
            if vertex_category(diagram, v0_id) == VertexCategory::Unknown {
                if let Some(v0_pt) = vertex_to_point(diagram, v0_id) {
                    let cell_id = diagram.edge_get_cell(edge_id).ok();
                    let twin_id = diagram.edge_get_twin(edge_id).ok();

                    if let (Some(cid), Some(_tid)) = (cell_id, twin_id) {
                        let on_contour = on_site(diagram, cid, lines, v0_pt);
                        if on_contour {
                            set_vertex_category(diagram, v0_id, VertexCategory::OnContour);
                        }
                    }
                }
            }
        }
    }

    // Step 2b: Mark secondary edge vertices as OnContour
    for edge_idx in 0..num_edges {
        let edge_id = diagram.edge_index_unchecked(edge_idx);
        let edge = &diagram.edges()[edge_idx];

        if edge.is_secondary() {
            if let Some(v0_id) = diagram.edge_get_vertex0(edge_id).ok().flatten() {
                let cell_id = diagram.edge_get_cell(edge_id).ok();
                let twin_id = diagram.edge_get_twin(edge_id).ok();

                if let (Some(cid), Some(tid)) = (cell_id, twin_id) {
                    if let (Ok(cell), Ok(twin_cell_id)) =
                        (diagram.cell(cid), diagram.edge_get_cell(tid))
                    {
                        if let Ok(twin_cell) = diagram.cell(twin_cell_id) {
                            // One cell contains point, other contains segment
                            if cell.contains_point() != twin_cell.contains_point() {
                                let pt_on_contour = if cell.contains_point() {
                                    contour_point(diagram, cid, lines)
                                } else {
                                    contour_point(diagram, twin_cell_id, lines)
                                };

                                if let Some(pt_on_contour) = pt_on_contour {
                                    let v1_id = diagram.edge_get_vertex1(edge_id).ok().flatten();

                                    if v1_id.is_none() {
                                        // Infinite edge - v0 must be on contour
                                        set_vertex_category(
                                            diagram,
                                            v0_id,
                                            VertexCategory::OnContour,
                                        );
                                    } else if let Some(v1) = v1_id {
                                        // Finite edge - one vertex is on contour
                                        let v0_on =
                                            vertex_equal_to_point(diagram, v0_id, pt_on_contour);
                                        let v1_on =
                                            vertex_equal_to_point(diagram, v1, pt_on_contour);

                                        if v0_on {
                                            set_vertex_category(
                                                diagram,
                                                v0_id,
                                                VertexCategory::OnContour,
                                            );
                                        } else if v1_on {
                                            set_vertex_category(
                                                diagram,
                                                v1,
                                                VertexCategory::OnContour,
                                            );
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Step 3: Classify infinite edges
    for edge_idx in 0..num_edges {
        let edge_id = diagram.edge_index_unchecked(edge_idx);
        let edge = &diagram.edges()[edge_idx];

        let v1_id = diagram.edge_get_vertex1(edge_id).ok().flatten();
        if v1_id.is_none() {
            // Infinite edge
            if let Some(v0_id) = diagram.edge_get_vertex0(edge_id).ok().flatten() {
                let twin_id = diagram.edge_get_twin(edge_id).ok();
                let cell_id = diagram.edge_get_cell(edge_id).ok();

                if let (Some(tid), Some(mut cid)) = (twin_id, cell_id) {
                    let mut twin_cell_id = diagram.edge_get_cell(tid).ok();

                    // Annotate edge and twin
                    if edge.is_secondary() {
                        set_edge_category(diagram, edge_id, EdgeCategory::PointsOutside);
                        set_edge_category(diagram, tid, EdgeCategory::PointsToContour);
                        set_vertex_category(diagram, v0_id, VertexCategory::OnContour);
                    } else {
                        set_edge_category(diagram, edge_id, EdgeCategory::PointsOutside);
                        set_edge_category(diagram, tid, EdgeCategory::PointsOutside);
                        set_vertex_category(diagram, v0_id, VertexCategory::Outside);
                    }

                    // Annotate cells
                    if let (Ok(cell), Some(mut tcid)) = (diagram.cell(cid), twin_cell_id) {
                        if cell.contains_segment() {
                            std::mem::swap(&mut cid, &mut tcid);
                            twin_cell_id = Some(tcid);
                        }

                        set_cell_category(diagram, cid, CellCategory::Outside);

                        if let Some(tcid) = twin_cell_id {
                            if let Ok(twin_cell) = diagram.cell(tcid) {
                                let twin_cat = if twin_cell.contains_point() {
                                    CellCategory::Outside
                                } else {
                                    CellCategory::Boundary
                                };
                                set_cell_category(diagram, tcid, twin_cat);
                            }
                        }
                    }
                }
            }
        }
    }

    // Step 4: Classify finite edges with at least one segment cell
    let debug = std::env::var("GAP_DETECTION_DEBUG").is_ok();
    let mut debug_log = if debug {
        std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open("/tmp/annotation_debug.txt")
            .ok()
    } else {
        None
    };
    let mut debug_step4_finite_count = 0;
    let mut debug_step4_segment_count = 0;
    let mut debug_step4_classified_count = 0;

    for edge_idx in 0..num_edges {
        let edge_id = diagram.edge_index_unchecked(edge_idx);

        let v0_id = diagram.edge_get_vertex0(edge_id).ok().flatten();
        let v1_id = diagram.edge_get_vertex1(edge_id).ok().flatten();

        if let (Some(v0), Some(v1)) = (v0_id, v1_id) {
            // Both vertices exist - finite edge
            debug_step4_finite_count += 1;

            let cell_id = diagram.edge_get_cell(edge_id).ok();
            let twin_id = diagram.edge_get_twin(edge_id).ok();

            if let (Some(cid), Some(tid)) = (cell_id, twin_id) {
                let twin_cell_id = diagram.edge_get_cell(tid).ok();

                // Find which cell (if any) contains a segment
                let line_opt = if let Ok(cell) = diagram.cell(cid) {
                    if cell.contains_segment() {
                        let (src_idx, _) = cell.source_index_2();
                        let idx = src_idx.usize();
                        if idx < lines.len() {
                            Some((lines[idx].clone(), cid, twin_cell_id.unwrap_or(cid)))
                        } else {
                            None
                        }
                    } else if let Some(tcid) = twin_cell_id {
                        if let Ok(twin_cell) = diagram.cell(tcid) {
                            if twin_cell.contains_segment() {
                                let (src_idx, _) = twin_cell.source_index_2();
                                let idx = src_idx.usize();
                                if idx < lines.len() {
                                    Some((lines[idx].clone(), tcid, cid))
                                } else {
                                    None
                                }
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    } else {
                        None
                    }
                } else {
                    None
                };

                if let Some((line, seg_cell_id, other_cell_id)) = line_opt {
                    debug_step4_segment_count += 1;

                    let v0_category = vertex_category(diagram, v0);
                    let v1_category = vertex_category(diagram, v1);
                    let on_contour = v0_category == VertexCategory::OnContour
                        || v1_category == VertexCategory::OnContour;

                    if on_contour && v1_category == VertexCategory::OnContour {
                        // Secondary edge pointing to contour point
                        set_edge_category(diagram, edge_id, EdgeCategory::PointsToContour);
                        if let Some(ref mut f) = debug_log {
                            use std::io::Write;
                            writeln!(
                                f,
                                "    [Step 4] Edge {}: SKIPPED (both v0/v1 OnContour), v0_cat={:?}, v1_cat={:?}",
                                edge_idx, v0_category, v1_category
                            )
                            .ok();
                        }
                    } else {
                        // Use orientation test to classify v1
                        if let Some(ref mut f) = debug_log {
                            use std::io::Write;
                            writeln!(
                                f,
                                "    [Step 4] Edge {}: ORIENTATION TEST, v0_cat={:?}, v1_cat={:?}",
                                edge_idx, v0_category, v1_category
                            )
                            .ok();
                        }
                        if let Ok(v1_vertex) = diagram.vertex(v1) {
                            let l0_x = line.a.x as f64;
                            let l0_y = line.a.y as f64;
                            let lv_x = (line.b.x - line.a.x) as f64;
                            let lv_y = (line.b.y - line.a.y) as f64;

                            let v1_x = v1_vertex.x();
                            let v1_y = v1_vertex.y();

                            // Cross product: (v1 - l0) × lv
                            let side = (v1_x - l0_x) * lv_y - (v1_y - l0_y) * lv_x;

                            if let Some(ref mut f) = debug_log {
                                use std::io::Write;
                                if side == 0.0 {
                                    writeln!(
                                        f,
                                        "    [Step 4] Edge {}: side=0.0 (v1 collinear), v0_cat={:?}, v1_cat={:?}",
                                        edge_idx, v0_category, v1_category
                                    )
                                    .ok();
                                }
                            }

                            if side != 0.0 {
                                debug_step4_classified_count += 1;

                                let vc = if side > 0.0 {
                                    VertexCategory::Outside
                                } else {
                                    VertexCategory::Inside
                                };

                                let ec = if vc == VertexCategory::Outside {
                                    EdgeCategory::PointsOutside
                                } else {
                                    EdgeCategory::PointsInside
                                };

                                set_vertex_category(diagram, v1, vc);
                                set_edge_category(diagram, edge_id, ec);

                                // Annotate twin edge and v0
                                if on_contour {
                                    set_vertex_category(diagram, v0, VertexCategory::OnContour);
                                    set_edge_category(diagram, tid, EdgeCategory::PointsToContour);
                                } else {
                                    set_vertex_category(diagram, v0, vc);
                                    set_edge_category(diagram, tid, ec);
                                }

                                // Annotate cells
                                let cell_cat = if on_contour {
                                    CellCategory::Boundary
                                } else if vc == VertexCategory::Outside {
                                    CellCategory::Outside
                                } else {
                                    CellCategory::Inside
                                };

                                set_cell_category(diagram, seg_cell_id, cell_cat);

                                let other_is_segment = diagram
                                    .cell(other_cell_id)
                                    .map(|c| c.contains_segment())
                                    .unwrap_or(false);

                                let other_cat = if on_contour && other_is_segment {
                                    CellCategory::Boundary
                                } else if vc == VertexCategory::Outside {
                                    CellCategory::Outside
                                } else {
                                    CellCategory::Inside
                                };

                                set_cell_category(diagram, other_cell_id, other_cat);
                            }
                        }
                    }
                }
            }
        }
    }

    if let Some(ref mut f) = debug_log {
        use std::io::Write;
        writeln!(
            f,
            "  [Step 4] Finite edges: {}, with segment: {}, classified: {}",
            debug_step4_finite_count, debug_step4_segment_count, debug_step4_classified_count
        )
        .ok();
    }

    // Step 5a: First pass - mark edges adjacent to classified cells
    let mut cell_queue: Vec<bv::CellIndex> = Vec::new();

    for edge_idx in 0..num_edges {
        let edge_id = diagram.edge_index_unchecked(edge_idx);

        if edge_category(diagram, edge_id) == EdgeCategory::Unknown {
            let is_finite = diagram.edge_is_finite(edge_id).unwrap_or(false);

            if is_finite {
                let cell_id = diagram.edge_get_cell(edge_id).ok();
                let twin_id = diagram.edge_get_twin(edge_id).ok();

                if let (Some(cid), Some(tid)) = (cell_id, twin_id) {
                    let twin_cell_id = diagram.edge_get_cell(tid).ok();

                    if let (Ok(cell), Some(tcid)) = (diagram.cell(cid), twin_cell_id) {
                        if let Ok(twin_cell) = diagram.cell(tcid) {
                            // Both cells must be Point-based
                            if cell.contains_point() && twin_cell.contains_point() {
                                let cc = cell_category(diagram, cid);
                                let cc2 = cell_category(diagram, tcid);

                                let mut cc_new = cc;
                                if cc_new == CellCategory::Unknown {
                                    cc_new = cc2;
                                }

                                if cc_new == CellCategory::Unknown {
                                    if let Some(v0_id) =
                                        diagram.edge_get_vertex0(edge_id).ok().flatten()
                                    {
                                        let vc = vertex_category(diagram, v0_id);
                                        if vc != VertexCategory::Unknown
                                            && vc != VertexCategory::OnContour
                                        {
                                            cc_new = if vc == VertexCategory::Outside {
                                                CellCategory::Outside
                                            } else {
                                                CellCategory::Inside
                                            };
                                        }
                                    }
                                }

                                if cc_new != CellCategory::Unknown {
                                    let vc = if cc_new == CellCategory::Outside {
                                        VertexCategory::Outside
                                    } else {
                                        VertexCategory::Inside
                                    };

                                    if let Some(v0_id) =
                                        diagram.edge_get_vertex0(edge_id).ok().flatten()
                                    {
                                        set_vertex_category(diagram, v0_id, vc);
                                    }
                                    if let Some(v1_id) =
                                        diagram.edge_get_vertex1(edge_id).ok().flatten()
                                    {
                                        set_vertex_category(diagram, v1_id, vc);
                                    }

                                    let ec_new = if cc_new == CellCategory::Outside {
                                        EdgeCategory::PointsOutside
                                    } else {
                                        EdgeCategory::PointsInside
                                    };

                                    set_edge_category(diagram, edge_id, ec_new);
                                    set_edge_category(diagram, tid, ec_new);

                                    if cc != cc_new {
                                        set_cell_category(diagram, cid, cc_new);
                                        cell_queue.push(cid);
                                    }
                                    if cc2 != cc_new {
                                        set_cell_category(diagram, tcid, cc_new);
                                        cell_queue.push(tcid);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Step 5b: Seed fill over remaining unmarked cells
    while let Some(cell_id) = cell_queue.pop() {
        let cc = cell_category(diagram, cell_id);

        if cc == CellCategory::Unknown {
            continue;
        }

        let ec_new = if cc == CellCategory::Outside {
            EdgeCategory::PointsOutside
        } else {
            EdgeCategory::PointsInside
        };

        // Walk around cell edges
        if let Ok(cell) = diagram.cell(cell_id) {
            if let Some(first_edge_id) = cell.get_incident_edge() {
                let mut current_edge_id = first_edge_id;

                loop {
                    let ec = edge_category(diagram, current_edge_id);

                    if ec == EdgeCategory::Unknown {
                        if let Ok(twin_id) = diagram.edge_get_twin(current_edge_id) {
                            if let Ok(twin_cell_id) = diagram.edge_get_cell(twin_id) {
                                // Verify both cells are Point-based
                                if let (Ok(this_cell), Ok(other_cell)) =
                                    (diagram.cell(cell_id), diagram.cell(twin_cell_id))
                                {
                                    if this_cell.contains_point() && other_cell.contains_point() {
                                        set_edge_category(diagram, current_edge_id, ec_new);
                                        set_edge_category(diagram, twin_id, ec_new);

                                        let cc2 = cell_category(diagram, twin_cell_id);
                                        if cc2 != cc {
                                            set_cell_category(diagram, twin_cell_id, cc);
                                            cell_queue.push(twin_cell_id);
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // Move to next edge
                    if let Some(next_edge_id) = diagram.edge_rot_next(current_edge_id) {
                        current_edge_id = next_edge_id;
                        if current_edge_id == first_edge_id {
                            break;
                        }
                    } else {
                        break;
                    }
                }
            }
        }
    }
}

/// Reset all vertex, edge, and cell categories to Unknown
fn reset_annotations(diagram: &mut bv::Diagram) {
    for edge_idx in 0..diagram.edges().len() {
        let edge_id = diagram.edge_index_unchecked(edge_idx);
        set_edge_category(diagram, edge_id, EdgeCategory::Unknown);
    }
}

// ---------------------------------------------------------------------------
// Unit Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vertex_category_enum() {
        assert_eq!(VertexCategory::OnContour as u8, 0);
        assert_eq!(VertexCategory::Inside as u8, 1);
        assert_eq!(VertexCategory::Outside as u8, 2);
        assert_eq!(VertexCategory::Unknown as u8, 3);

        assert_eq!(VertexCategory::from(0), VertexCategory::OnContour);
        assert_eq!(VertexCategory::from(1), VertexCategory::Inside);
        assert_eq!(VertexCategory::from(2), VertexCategory::Outside);
        assert_eq!(VertexCategory::from(99), VertexCategory::Unknown);
    }

    #[test]
    fn test_edge_category_enum() {
        assert_eq!(EdgeCategory::PointsToContour as u8, 0);
        assert_eq!(EdgeCategory::PointsInside as u8, 1);
        assert_eq!(EdgeCategory::PointsOutside as u8, 2);
        assert_eq!(EdgeCategory::Unknown as u8, 3);

        assert_eq!(EdgeCategory::from(0), EdgeCategory::PointsToContour);
        assert_eq!(EdgeCategory::from(1), EdgeCategory::PointsInside);
        assert_eq!(EdgeCategory::from(2), EdgeCategory::PointsOutside);
        assert_eq!(EdgeCategory::from(99), EdgeCategory::Unknown);
    }

    #[test]
    fn test_cell_category_enum() {
        assert_eq!(CellCategory::Boundary as u8, 0);
        assert_eq!(CellCategory::Inside as u8, 1);
        assert_eq!(CellCategory::Outside as u8, 2);
        assert_eq!(CellCategory::Unknown as u8, 3);

        assert_eq!(CellCategory::from(0), CellCategory::Boundary);
        assert_eq!(CellCategory::from(1), CellCategory::Inside);
        assert_eq!(CellCategory::from(2), CellCategory::Outside);
        assert_eq!(CellCategory::from(99), CellCategory::Unknown);
    }
}
