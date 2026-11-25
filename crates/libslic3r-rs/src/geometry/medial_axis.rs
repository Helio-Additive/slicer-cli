//! Medial axis transform for gap fill.
//!
//! This is a faithful port of BambuStudio's `Geometry/MedialAxis.cpp` (685 lines)
//! and `ExPolygon::medial_axis()` (lines 263–380 of `ExPolygon.cpp`).
//!
//! The algorithm uses a Voronoi diagram of the polygon's line segments to extract
//! the medial axis (skeleton) as ThickPolylines with per-vertex variable widths.
//!
//! ## Algorithm (from MedialAxis.cpp)
//!
//! 1. Extract line segments from the ExPolygon (contour + holes)
//! 2. Construct a Voronoi diagram from these segments using `boostvoronoi`
//!    (matching C++ `boost::polygon::voronoi_diagram`)
//! 3. Color exterior edges (matching `Slic3r::Voronoi::annotate_inside_outside`)
//! 4. For each primary, finite edge with at least one interior vertex:
//!    - `validate_edge()`: check width is in [min_width, max_width] and
//!      generating segments are nearly anti-parallel (facing each other)
//! 5. Walk valid ("active") edges to build ThickPolylines
//!    (`process_edge_neighbors()`)
//! 6. Post-process (from `ExPolygon::medial_axis()`):
//!    - Extend endpoints to the ExPolygon boundary
//!    - Remove too-short polylines
//!    - Reconnect fragments after removal
//!
//! ## BambuStudio Reference Files
//!
//! - `Geometry/MedialAxis.cpp` lines 445–685
//! - `Geometry/MedialAxis.hpp`
//! - `ExPolygon.cpp` lines 263–380
//! - `Geometry/Voronoi.hpp` / `Voronoi.cpp`

use crate::geometry::{
    ExPolygon, Line, Point, Polygon, Polyline, ThickPolyline, ThickPolylines,
};
use crate::{Coord, CoordF, SCALING_FACTOR};

use boostvoronoi::prelude as bv;

/// R125: true on the gated (F1_UNION) byte-match path — enables the faithful
/// `lrint`-rounded Voronoi-vertex → `Point` conversion. Native constructs these
/// points via `Point(double,double)` / `Line{double,double}` (MedialAxis.cpp:463-464,
/// 492-493, 566, 606-607), all of which route through `Vec2crd(coord_t(lrint(x)),
/// coord_t(lrint(y)))` (Point.hpp:179) — round-to-nearest (default FE_TONEAREST,
/// ties-to-even). The prior `as Coord` truncated toward zero (an F2-class ±1-unit
/// divergence, same root as the R124 arc center). Default path keeps the legacy
/// truncation so the byte-locked 147987 default is unchanged. Cached: medial-axis
/// runs once per gap ExPolygon.
fn f1_union_vd_round() -> bool {
    use std::sync::OnceLock;
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| crate::faithful_gate("F1_UNION"))
}

/// `MEDIAL_AXIS_EIGEN_PIN` (faithful, R953): the medial axis's vector arithmetic as the pin
/// compiles it. MedialAxis.cpp:641-647 / 665 (`Line::distance_to`, `.norm()`, `line.length()`)
/// and ExPolygon.cpp:296-316, 325 (`normalized()`, `p1 -= n * max_width`, `Polygon::intersection`,
/// `on_boundary`, `Polyline::length`) run on Eigen `Vec2d`s, whose `squaredNorm`, `dot` and
/// `a + t * v` clang does not contract (each product is rounded before the sum; the scalar
/// `cross2` of Point.hpp:86-89 and the scalar `floor(pt0 + t * v + 0.5)` of Polygon.cpp:334 stay
/// fused). `=0` keeps the fused scalar spelling of the old medial-axis shim
/// (medial_axis_shim.cpp), the engine's arithmetic until R952.
pub(crate) fn medial_axis_eigen_pin() -> bool {
    use std::sync::OnceLock;
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| crate::faithful_gate("MEDIAL_AXIS_EIGEN_PIN"))
}

/// Convert a Voronoi-vertex `double` coordinate to `Coord`, matching native
/// `Point(double,double)`'s `coord_t(lrint(x))`. `round_ties_even` mirrors `lrint`'s
/// default rounding mode EXACTLY (NOT `.round()`, which is ties-away-from-zero). Gated
/// F1_UNION; the default path keeps the legacy truncation toward zero.
#[inline]
fn vd_coord(x: f64) -> Coord {
    if f1_union_vd_round() {
        x.round_ties_even() as Coord
    } else {
        x as Coord
    }
}

// ---------------------------------------------------------------------------
// Public configuration (kept for backward compatibility with callers)
// ---------------------------------------------------------------------------

/// Configuration for medial axis computation.
///
/// Maps to the constructor parameters of `Slic3r::Geometry::MedialAxis`.
#[derive(Debug, Clone)]
pub struct MedialAxisConfig {
    /// Minimum width to include in the result (mm, unscaled).
    pub min_width: CoordF,
    /// Maximum width to include in the result (mm, unscaled).
    pub max_width: CoordF,
}

impl Default for MedialAxisConfig {
    fn default() -> Self {
        Self {
            min_width: 0.05,
            max_width: 0.8,
        }
    }
}

impl MedialAxisConfig {
    // Create config for gap fill with the given width range (in mm).
    //
    // BambuStudio calls: `ex.medial_axis(min, max, &polylines)`
    // where min/max are in **scaled** coordinates.  Our public API takes mm
    // and we convert internally.
    pub fn for_gap_fill(min_width: CoordF, max_width: CoordF) -> Self {
        Self {
            min_width,
            max_width,
        }
    }
}

// ---------------------------------------------------------------------------
// Constants matching BambuStudio
// ---------------------------------------------------------------------------

/// SCALED_EPSILON from libslic3r (used in validate_edge)
const SCALED_EPSILON: f64 = SCALING_FACTOR * 1e-6; // 1 nm in scaled units = 1.0

/// CLIPPER_MAX_COORD_UNSCALED from BambuStudio (clipper.hpp hiRange = 0x3FFFFFFFFFFFFFFF).
/// Used by MedialAxis::validate_edge (MedialAxis.cpp:598-601) to reject almost-infinite
/// Voronoi vertices that would overflow ClipperLib. C++: double(CLIPPER_MAX_COORD_UNSCALED).
const CLIPPER_MAX_COORD_UNSCALED: f64 = 0x3FFFFFFFFFFFFFFFi64 as f64;

/// Color used to mark exterior edges (matching BambuStudio's EXTERNAL_COLOR = 1)
const EXTERNAL_COLOR: bv::ColorType = 1;

// ---------------------------------------------------------------------------
// Edge data annotation (mirrors MedialAxis::EdgeData in C++)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
struct EdgeData {
    active: bool,
    width_start: f64, // scaled
    width_end: f64,   // scaled
}

impl Default for EdgeData {
    fn default() -> Self {
        Self {
            active: false,
            width_start: 0.0,
            width_end: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Compute the medial axis of a single polygon (legacy API, returns plain Polylines).
///
/// Equivalent to BambuStudio's `ExPolygon::medial_axis(min, max, Polylines*)`.
pub fn compute_medial_axis(polygon: &Polygon) -> Vec<Polyline> {
    let config = MedialAxisConfig::default();
    let expoly = ExPolygon::new(polygon.clone());
    let thick = compute_medial_axis_thick(&expoly, &config);
    thick.into_iter().map(|tp| tp.to_polyline()).collect()
}

/// Compute the medial axis of an ExPolygon, producing ThickPolylines with
/// per-vertex variable widths.
///
/// This is the main entry point matching BambuStudio's
/// `ExPolygon::medial_axis(double min_width, double max_width, ThickPolylines*)`.
///
/// `config.min_width` and `config.max_width` are in **mm** (unscaled).
/// Internally we convert to scaled coordinates for the Voronoi computation
/// (matching the C++ code which operates entirely in scaled space).
pub fn compute_medial_axis_thick(expoly: &ExPolygon, config: &MedialAxisConfig) -> ThickPolylines {
    if expoly.contour.len() < 3 {
        return vec![];
    }

    // Convert width bounds to scaled coordinates (matching C++)
    let min_width = config.min_width * SCALING_FACTOR;
    let max_width = config.max_width * SCALING_FACTOR;

    // --- Step 1: Extract line segments from the ExPolygon ---
    // BambuStudio: m_lines = expolygon.lines();
    let lines = expolygon_lines(expoly);
    if lines.is_empty() {
        return vec![];
    }

    // --- Step 2: Build Voronoi diagram ---
    // BambuStudio: m_vd.construct_voronoi(m_lines.begin(), m_lines.end());
    let bv_segments: Vec<bv::Line<i64>> = lines
        .iter()
        .map(|l| {
            bv::Line::new(
                bv::Point { x: l.a.x, y: l.a.y },
                bv::Point { x: l.b.x, y: l.b.y },
            )
        })
        .collect();

    let diagram = match bv::Builder::<i64>::default()
        .with_segments(bv_segments.iter())
        .and_then(|b| b.build())
    {
        Ok(d) => d,
        Err(_) => return vec![],
    };

    let num_edges = diagram.edges().len();
    if num_edges == 0 {
        return vec![];
    }

    // --- Step 3: Annotate inside/outside ---
    // MedialAxis.cpp:452 — Slic3r::Voronoi::annotate_inside_outside(m_vd, m_lines);
    // FIDELITY-NOTE(Voronoi): C++ uses Slic3r::Voronoi::annotate_inside_outside()
    // (VoronoiOffset.cpp), which tags each vertex with a 4-state VertexCategory
    // {OnContour, Inside, Outside, Unknown} and lets build() keep only edges whose
    // vertex0/vertex1 is *strictly* Inside (excluding OnContour). The boostvoronoi
    // crate does not expose that classifier; we substitute its color_exterior_edges()
    // and treat any non-exterior vertex as "inside". This is a cross-cutting Voronoi
    // primitive substitution (analogous to F1), not re-routed per-file: OnContour
    // vertices are treated as inside here whereas C++ excludes them.
    let mut diagram = diagram;
    diagram.color_exterior_edges(EXTERNAL_COLOR);

    // --- Step 4: Validate edges and mark active ones ---
    // BambuStudio: m_edge_data.assign(m_vd.edges().size() / 2, EdgeData{});
    let mut edge_data: Vec<EdgeData> = vec![EdgeData::default(); num_edges / 2];

    // BambuStudio iterates edge += 2 (stepping over twin pairs)
    let mut edge_idx = 0usize;
    while edge_idx < num_edges {
        let edge_id = diagram.edge_index_unchecked(edge_idx);

        let edge = &diagram.edges()[edge_idx];

        // BambuStudio: edge->is_primary() && edge->is_finite()
        let is_finite = diagram.edge_is_finite(edge_id).unwrap_or(false);
        if edge.is_primary() && is_finite {
            // Check that at least one vertex is inside
            // BambuStudio: Voronoi::vertex_category(edge->vertex0()) == Inside
            // We use: vertex color != EXTERNAL_COLOR means interior
            let v0_id = diagram.edge_get_vertex0(edge_id).ok().flatten();
            let v1_id = diagram.edge_get_vertex1(edge_id).ok().flatten();

            let v0_inside = v0_id
                .and_then(|vid| diagram.vertex_get_color(vid))
                .map(|c| c & EXTERNAL_COLOR == 0)
                .unwrap_or(false);
            let v1_inside = v1_id
                .and_then(|vid| diagram.vertex_get_color(vid))
                .map(|c| c & EXTERNAL_COLOR == 0)
                .unwrap_or(false);

            if v0_inside || v1_inside {
                if let Some((w0, w1)) = validate_edge(
                    &diagram,
                    edge_id,
                    &lines,
                    &bv_segments,
                    min_width,
                    max_width,
                ) {
                    let data_idx = edge_idx / 2;
                    // Determine if this edge_id is the "reversed" one of the pair
                    // BambuStudio: edge_id = &edge - &m_vd.edges().front();
                    //   reversed = (edge_id & 1) != 0
                    let reversed = (edge_idx & 1) != 0;
                    if reversed {
                        edge_data[data_idx].width_start = w1;
                        edge_data[data_idx].width_end = w0;
                    } else {
                        edge_data[data_idx].width_start = w0;
                        edge_data[data_idx].width_end = w1;
                    }
                    edge_data[data_idx].active = true;
                }
            }
        }

        edge_idx += 2;
    }

    // Helper closures to access edge_data like the C++
    // BambuStudio: edge_data(edge) returns (EdgeData&, bool reversed)
    //   edge_id = &edge - &front; data_idx = edge_id / 2; reversed = edge_id & 1

    // --- Step 5: Walk active edges to build ThickPolylines ---
    // BambuStudio: MedialAxis::build() lines 484–530
    let mut polylines: ThickPolylines = Vec::new();

    // We iterate by 2 (same as C++)
    let mut seed_idx = 0usize;
    while seed_idx < num_edges {
        let data_idx = seed_idx / 2;
        if !edge_data[data_idx].active {
            seed_idx += 2;
            continue;
        }

        // Mark this edge as visited
        edge_data[data_idx].active = false;

        let seed_edge_id = diagram.edge_index_unchecked(seed_idx);
        let data_idx = seed_idx / 2;

        // Get vertex positions
        let v0_pos = match get_vertex_pos(&diagram, seed_edge_id, true) {
            Some(p) => p,
            None => {
                seed_idx += 2;
                continue;
            }
        };
        let v1_pos = match get_vertex_pos(&diagram, seed_edge_id, false) {
            Some(p) => p,
            None => {
                seed_idx += 2;
                continue;
            }
        };

        // Start a polyline
        // MedialAxis.cpp:491-495 — seed polyline gets points {v0, v1} and widths
        //   {width_start, width_end}.
        // FIDELITY-NOTE(width-model): C++ ThickPolyline.width is a 2*(N-1) edge-pair
        // array (asserted polyline.width.size()==points.size()*2-2 at build:498,506).
        // The crate's ThickPolyline.widths is used inconsistently across the codebase
        // (per-vertex N in width_at_distance/clip_front/split_by_width_variation;
        // edge-pair 2*(N-1) in fill_concentric.rs/arachne). This port emits the
        // per-vertex form. Reconciling the two representations is a crate-wide
        // Polyline.hpp/ThickPolyline rework, not a per-function MedialAxis.cpp fix.
        let seed_w_start = edge_data[data_idx].width_start;
        let seed_w_end = edge_data[data_idx].width_end;

        // MedialAxis.cpp:492-493 — emplace_back(vertex0->x(), vertex0->y()) constructs
        // Point(double,double) = coord_t(lrint(x)): round-to-nearest, NOT truncate
        // (R125). `vd_coord` reproduces lrint on the gated path; default truncates.
        let mut points: Vec<Point> = vec![
            Point::new(vd_coord(v0_pos.0), vd_coord(v0_pos.1)),
            Point::new(vd_coord(v1_pos.0), vd_coord(v1_pos.1)),
        ];
        let mut widths: Vec<f64> = vec![seed_w_start, seed_w_end];
        let mut end_is_endpoint = false;

        // Grow forward
        // BambuStudio: process_edge_neighbors(&*seed_edge, &polyline)
        process_edge_neighbors(
            &diagram,
            &mut edge_data,
            seed_edge_id,
            &mut points,
            &mut widths,
            &mut end_is_endpoint,
        );

        // Grow backward
        // BambuStudio: reverse_polyline.clear();
        //   process_edge_neighbors(seed_edge->twin(), &reverse_polyline);
        let twin_id = match diagram.edge_get_twin(seed_edge_id) {
            Ok(t) => t,
            Err(_) => {
                seed_idx += 2;
                continue;
            }
        };

        let mut rev_points: Vec<Point> = Vec::new();
        let mut rev_widths: Vec<f64> = Vec::new();
        let mut rev_end_is_endpoint = false;
        // For the reverse walk, we need a "starting" edge which is the twin
        // We add the twin's vertex1 as the starting point of the reverse polyline
        // (this is actually seed's vertex0, which is already in our forward polyline)
        process_edge_neighbors(
            &diagram,
            &mut edge_data,
            twin_id,
            &mut rev_points,
            &mut rev_widths,
            &mut rev_end_is_endpoint,
        );

        // Prepend reverse to forward
        // BambuStudio:
        //   polyline.points.insert(begin, rev.points.rbegin(), rev.points.rend());
        //   polyline.endpoints.first = rev.endpoints.second;
        if !rev_points.is_empty() {
            rev_points.reverse();
            rev_widths.reverse();
            rev_points.append(&mut points);
            rev_widths.append(&mut widths);
            points = rev_points;
            widths = rev_widths;
        }
        let start_is_endpoint = rev_end_is_endpoint;

        // Prevent loop endpoints from being extended
        // BambuStudio: if (polyline.first_point() == polyline.last_point()) { ... }
        let is_loop = points.len() >= 2 && points.first() == points.last();

        let mut tp = ThickPolyline::new();
        for (i, pt) in points.iter().enumerate() {
            // Convert width from scaled to mm for the ThickPolyline output
            tp.push(*pt, widths[i] / SCALING_FACTOR);
        }
        tp.endpoints = if is_loop {
            [false, false]
        } else {
            [start_is_endpoint, end_is_endpoint]
        };

        polylines.push(tp);

        seed_idx += 2;
    }

    // --- Step 6: Post-process (ExPolygon::medial_axis, lines 287–371) ---
    // FIDELITY-NOTE(scope): MedialAxis::build() in MedialAxis.cpp ends at the loop
    // above (line 516); endpoint extension / short-polyline removal / reconnection
    // belong to ExPolygon::medial_axis() in ExPolygon.cpp, NOT MedialAxis.cpp. They
    // are kept here because the crate's ExPolygon::medial_axis wrapper omits them and
    // relies on this entry point performing the post-processing. Auditing/fixing that
    // logic is an ExPolygon.cpp concern, outside this file's MedialAxis.cpp audit.
    postprocess_medial_axis(&mut polylines, expoly, max_width);

    polylines
}

/// Compute medial axis for multiple ExPolygons.
///
/// BambuStudio: loops over gap_regions calling `ex.medial_axis(min, max, &polylines)`
pub fn compute_medial_axis_multi(
    expolygons: &[ExPolygon],
    config: &MedialAxisConfig,
) -> ThickPolylines {
    let mut result = Vec::new();
    for expoly in expolygons {
        let thick = compute_medial_axis_thick(expoly, config);
        result.extend(thick);
    }
    result
}

/// Compute distance from a point to the nearest boundary of the ExPolygon.
pub fn distance_to_boundary(expoly: &ExPolygon, point: Point) -> CoordF {
    let lines = expolygon_lines(expoly);
    nearest_boundary_distance_lines(point, &lines)
}

// ---------------------------------------------------------------------------
// Internal: Voronoi edge validation (MedialAxis::validate_edge, lines 588–683)
// ---------------------------------------------------------------------------

/// Validate a Voronoi edge for inclusion in the medial axis.
///
/// Returns `Some((w0, w1))` with the widths at vertex0 and vertex1 (in scaled
/// coordinates) if the edge is valid, `None` otherwise.
///
/// Faithfully ports `MedialAxis::validate_edge()` from MedialAxis.cpp lines 588–683.
fn validate_edge(
    diagram: &bv::Diagram,
    edge_id: bv::EdgeIndex,
    lines: &[Line],
    _bv_segments: &[bv::Line<i64>],
    min_width: f64,
    max_width: f64,
) -> Option<(f64, f64)> {
    // Get vertex positions (both must exist for finite edges)
    let (v0x, v0y) = get_vertex_pos(diagram, edge_id, true)?;
    let (v1x, v1y) = get_vertex_pos(diagram, edge_id, false)?;

    // Overflow/infinite check (MedialAxis.cpp:597-603)
    // C++: if (std::abs(edge->vertexN()->{x,y}()) > double(CLIPPER_MAX_COORD_UNSCALED)) return false;
    if v0x.abs() > CLIPPER_MAX_COORD_UNSCALED
        || v0y.abs() > CLIPPER_MAX_COORD_UNSCALED
        || v1x.abs() > CLIPPER_MAX_COORD_UNSCALED
        || v1y.abs() > CLIPPER_MAX_COORD_UNSCALED
    {
        return None;
    }

    // Construct the Voronoi edge as a line.
    // MedialAxis.cpp:606-607 — Line({vertex0->x(), vertex0->y()}, {vertex1->x(), vertex1->y()})
    // brace-inits Point(double,double) = coord_t(lrint(x)): round-to-nearest, NOT
    // truncate (double→int narrowing is ill-formed in a braced-init-list, so this
    // binds the double,double constructor). These points feed the w0/w1 width and
    // edge-length checks, so the ±1 propagates to gap-fill widths + validation (R125).
    let edge_a = Point::new(vd_coord(v0x), vd_coord(v0y));
    let edge_b = Point::new(vd_coord(v1x), vd_coord(v1y));

    // Retrieve the cells on each side
    // BambuStudio: cell_l = edge->cell(); cell_r = edge->twin()->cell();
    let cell_l_id = diagram.edge_get_cell(edge_id).ok()?;
    let twin_id = diagram.edge_get_twin(edge_id).ok()?;
    let cell_r_id = diagram.edge_get_cell(twin_id).ok()?;

    let cell_l = diagram.cell(cell_l_id).ok()?;
    let cell_r = diagram.cell(cell_r_id).ok()?;

    // Retrieve source segments
    let segment_l = get_cell_segment(cell_l, lines)?;
    let segment_r = get_cell_segment(cell_r, lines)?;

    // Compute widths at both endpoints
    // BambuStudio lines 639–645:
    //   w0 = cell_r->contains_segment()
    //       ? segment_r.distance_to(line.a)*2
    //       : (retrieve_endpoint(cell_r) - line.a).norm()*2;
    //   w1 = cell_l->contains_segment()
    //       ? segment_l.distance_to(line.b)*2
    //       : (retrieve_endpoint(cell_l) - line.b).norm()*2;
    let w0 = if cell_r.contains_segment() {
        line_distance_to_point(&segment_r, edge_a) * 2.0
    } else {
        let ep = retrieve_endpoint(cell_r, lines)?;
        point_dist(edge_a, ep) * 2.0
    };

    let w1 = if cell_l.contains_segment() {
        line_distance_to_point(&segment_l, edge_b) * 2.0
    } else {
        let ep = retrieve_endpoint(cell_l, lines)?;
        point_dist(edge_b, ep) * 2.0
    };

    // Angle filter (BambuStudio lines 647–670)
    if cell_l.contains_segment() && cell_r.contains_segment() {
        let angle_l = line_orientation(&segment_l);
        let angle_r = line_orientation(&segment_r);
        let mut angle = (angle_r - angle_l).abs();
        if angle > std::f64::consts::PI {
            angle = 2.0 * std::f64::consts::PI - angle;
        }

        // BambuStudio: if (PI - angle > PI / 8.) ...
        if std::f64::consts::PI - angle > std::f64::consts::PI / 8.0 {
            // Angle is not narrow enough
            let edge_len = point_dist(edge_a, edge_b);
            if w0 < SCALED_EPSILON || w1 < SCALED_EPSILON || edge_len >= min_width {
                return None;
            }
        }
    } else {
        if w0 < SCALED_EPSILON || w1 < SCALED_EPSILON {
            return None;
        }
    }

    // Width bounds check (BambuStudio lines 674–682)
    if (w0 >= min_width || w1 >= min_width) && (w0 <= max_width || w1 <= max_width) {
        Some((w0, w1))
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Internal: Edge neighbor walking (MedialAxis::process_edge_neighbors, lines 543–586)
// ---------------------------------------------------------------------------

/// Walk along connected active Voronoi edges to extend a polyline.
///
/// Faithfully ports `MedialAxis::process_edge_neighbors()`.
fn process_edge_neighbors(
    diagram: &bv::Diagram,
    edge_data: &mut [EdgeData],
    start_edge_id: bv::EdgeIndex,
    points: &mut Vec<Point>,
    widths: &mut Vec<f64>,
    end_is_endpoint: &mut bool,
) {
    let mut current_edge_id = start_edge_id;

    loop {
        // BambuStudio: twin = edge->twin();
        let twin_id = match diagram.edge_get_twin(current_edge_id) {
            Ok(t) => t,
            Err(_) => break,
        };

        // Count active neighbors around the ending vertex
        // BambuStudio: for (neighbor = twin->rot_next(); neighbor != twin; neighbor = neighbor->rot_next())
        let mut num_neighbors = 0usize;
        let mut first_neighbor: Option<bv::EdgeIndex> = None;

        if let Some(first_rot) = diagram.edge_rot_next(twin_id) {
            let mut neighbor = first_rot;
            loop {
                if neighbor == twin_id {
                    break;
                }
                let n_data_idx = neighbor.usize() / 2;
                if n_data_idx < edge_data.len() && edge_data[n_data_idx].active {
                    if num_neighbors == 0 {
                        first_neighbor = Some(neighbor);
                    }
                    num_neighbors += 1;
                }
                match diagram.edge_rot_next(neighbor) {
                    Some(next) => neighbor = next,
                    None => break,
                }
            }
        }

        if num_neighbors == 1 {
            // Single neighbor: continue the chain
            let neighbor_id = first_neighbor.unwrap();
            let n_data_idx = neighbor_id.usize() / 2;
            let reversed = (neighbor_id.usize() & 1) != 0;

            if n_data_idx < edge_data.len() && edge_data[n_data_idx].active {
                edge_data[n_data_idx].active = false;

                // Get the far vertex of the neighbor edge.
                // MedialAxis.cpp:566 — emplace_back(first_neighbor->vertex1()->x(),
                // ...->y()) constructs Point(double,double) = coord_t(lrint(x)):
                // round-to-nearest, NOT truncate (R125).
                if let Some((vx, vy)) = get_vertex_pos(diagram, neighbor_id, false) {
                    let pt = Point::new(vd_coord(vx), vd_coord(vy));
                    points.push(pt);

                    // BambuStudio: push width_start then width_end (or swapped if reversed)
                    // For our per-vertex model, we push the width at the new vertex
                    let w = if reversed {
                        edge_data[n_data_idx].width_start
                    } else {
                        edge_data[n_data_idx].width_end
                    };
                    widths.push(w);

                    current_edge_id = neighbor_id;
                    continue;
                }
            }
        } else if num_neighbors == 0 {
            // Dead end — this endpoint can be extended to boundary
            *end_is_endpoint = true;
        }
        // else: T-shaped or star-shaped joint — stop

        break;
    }
}

// ---------------------------------------------------------------------------
// Internal: Post-processing (ExPolygon::medial_axis, lines 287–371)
// ---------------------------------------------------------------------------

/// Post-process medial axis polylines: extend endpoints, remove short ones,
/// reconnect fragments.
///
/// Faithfully ports `ExPolygon::medial_axis()` lines 287–371.
fn postprocess_medial_axis(
    polylines: &mut ThickPolylines,
    expoly: &ExPolygon,
    max_width: f64, // scaled
) {
    if polylines.is_empty() {
        return;
    }

    // Find the maximum width across all polylines
    // BambuStudio: double max_w = 0; for (...) max_w = fmaxf(max_w, *max_element(...))
    let max_w = polylines
        .iter()
        .flat_map(|tp| tp.widths.iter())
        .cloned()
        .fold(0.0f64, f64::max);

    // max_w is in mm (our ThickPolyline stores mm widths)
    // max_width is in scaled coordinates; convert max_w to scaled for length comparison
    let max_w_scaled = max_w * SCALING_FACTOR;

    // Extend endpoints to boundary and remove too-short polylines
    // BambuStudio lines 291–333
    let mut removed = false;
    let mut i = 0;
    while i < polylines.len() {
        let polyline = &mut polylines[i];

        // Extend start endpoint
        if polyline.endpoints[0] && polyline.len() >= 2 {
            let front = polyline.points[0];
            if !is_on_boundary(expoly, front) {
                let p1f = (front.x as f64, front.y as f64);
                let p2f = if polyline.len() == 2 {
                    // Prevent touching the other side
                    let p2 = polyline.points[1];
                    ((p1f.0 + p2.x as f64) * 0.5, (p1f.1 + p2.y as f64) * 0.5)
                } else {
                    let p2 = polyline.points[1];
                    (p2.x as f64, p2.y as f64)
                };
                // Extend away from p2
                let dx = p2f.0 - p1f.0;
                let dy = p2f.1 - p1f.1;
                let len = (dx * dx + dy * dy).sqrt();
                if len > 0.0 {
                    let nx = dx / len;
                    let ny = dy / len;
                    let ext_x = p1f.0 - nx * max_width;
                    let ext_y = p1f.1 - ny * max_width;
                    let ext_pt = Point::new(ext_x.round() as Coord, ext_y.round() as Coord);
                    let orig_pt = Point::new(p2f.0.round() as Coord, p2f.1.round() as Coord);
                    if let Some(intersection) =
                        contour_intersection(&expoly.contour, Line::new(ext_pt, orig_pt))
                    {
                        polyline.points[0] = intersection;
                    }
                }
            }
        }

        // Extend end endpoint
        let n = polyline.len();
        if polyline.endpoints[1] && n >= 2 {
            let back = polyline.points[n - 1];
            if !is_on_boundary(expoly, back) {
                let p2f = (back.x as f64, back.y as f64);
                let p1f = if n == 2 {
                    let p1 = polyline.points[0];
                    ((p1.x as f64 + p2f.0) * 0.5, (p1.y as f64 + p2f.1) * 0.5)
                } else {
                    let p1 = polyline.points[n - 2];
                    (p1.x as f64, p1.y as f64)
                };
                let dx = p2f.0 - p1f.0;
                let dy = p2f.1 - p1f.1;
                let len = (dx * dx + dy * dy).sqrt();
                if len > 0.0 {
                    let nx = dx / len;
                    let ny = dy / len;
                    let ext_x = p2f.0 + nx * max_width;
                    let ext_y = p2f.1 + ny * max_width;
                    let ext_pt = Point::new(ext_x.round() as Coord, ext_y.round() as Coord);
                    let orig_pt = Point::new(p1f.0.round() as Coord, p1f.1.round() as Coord);
                    if let Some(intersection) =
                        contour_intersection(&expoly.contour, Line::new(orig_pt, ext_pt))
                    {
                        let n = polyline.len();
                        polyline.points[n - 1] = intersection;
                    }
                }
            }
        }

        // Remove too-short polylines
        // BambuStudio: if ((endpoints.first || endpoints.second) && length() < max_w*2)
        if (polyline.endpoints[0] || polyline.endpoints[1])
            && polyline.length() < max_w_scaled * 2.0
        {
            polylines.remove(i);
            removed = true;
            continue;
        }

        i += 1;
    }

    // Reconnect fragments after removal (BambuStudio lines 341–371)
    if removed {
        let mut i = 0;
        while i < polylines.len() {
            if polylines[i].endpoints[0] && polylines[i].endpoints[1] {
                i += 1;
                continue; // optimization
            }

            let mut j = i + 1;
            while j < polylines.len() {
                // Try to connect polyline[i] and polyline[j]
                let i_last = polylines[i].last_point().unwrap();
                let j_last = polylines[j].last_point().unwrap();
                let i_first = polylines[i].first_point().unwrap();
                let j_first = polylines[j].first_point().unwrap();

                if i_last == j_last {
                    // Reverse j, then append
                    polylines[j].reverse();
                } else if i_first == j_last {
                    polylines[i].reverse();
                    polylines[j].reverse();
                } else if i_first == j_first {
                    polylines[i].reverse();
                } else if i_last != j_first {
                    j += 1;
                    continue;
                }

                // Now polylines[i].last == polylines[j].first, append j to i
                let other = polylines.remove(j);
                let poly_i = &mut polylines[i];
                // Append other's points (skip the first which is the connection point)
                for k in 1..other.points.len() {
                    poly_i.points.push(other.points[k]);
                    poly_i.widths.push(other.widths[k]);
                }
                poly_i.endpoints[1] = other.endpoints[1];

                // Restart search from i+1
                j = i + 1;
            }

            i += 1;
        }
    }
}

// ---------------------------------------------------------------------------
// Helpers: geometry and Voronoi accessors
// ---------------------------------------------------------------------------

/// Extract all line segments from an ExPolygon (contour + holes).
/// Matches BambuStudio's `ExPolygon::lines()`.
fn expolygon_lines(expoly: &ExPolygon) -> Vec<Line> {
    let mut lines = Vec::new();
    add_polygon_lines(&expoly.contour, &mut lines);
    for hole in &expoly.holes {
        add_polygon_lines(hole, &mut lines);
    }
    lines
}

fn add_polygon_lines(polygon: &Polygon, lines: &mut Vec<Line>) {
    let pts = polygon.points();
    if pts.len() < 2 {
        return;
    }
    for i in 0..pts.len() {
        let j = (i + 1) % pts.len();
        // Skip degenerate segments
        if pts[i] != pts[j] {
            lines.push(Line::new(pts[i], pts[j]));
        }
    }
}

/// Get vertex position for edge's vertex0 (if `start` is true) or vertex1.
fn get_vertex_pos(
    diagram: &bv::Diagram,
    edge_id: bv::EdgeIndex,
    start: bool,
) -> Option<(f64, f64)> {
    let vid = if start {
        diagram.edge_get_vertex0(edge_id).ok()??
    } else {
        diagram.edge_get_vertex1(edge_id).ok()??
    };
    let v = diagram.vertex(vid).ok()?;
    Some((v.x(), v.y()))
}

/// Get the source line segment for a Voronoi cell.
///
/// For cells that contain a segment, returns the segment directly.
/// For cells that contain a point, returns the segment it came from
/// (the cell's source_index still refers to the segment).
fn get_cell_segment(cell: &bv::Cell, lines: &[Line]) -> Option<Line> {
    let (src_idx, _) = cell.source_index_2();
    let idx = src_idx.usize();
    if idx < lines.len() {
        Some(lines[idx])
    } else {
        None
    }
}

/// Retrieve the point for a cell that contains a segment endpoint.
///
/// BambuStudio: `retrieve_endpoint(cell)` — returns segment start or end
/// depending on source_category.
fn retrieve_endpoint(cell: &bv::Cell, lines: &[Line]) -> Option<Point> {
    let (src_idx, src_cat) = cell.source_index_2();
    let idx = src_idx.usize();
    if idx >= lines.len() {
        return None;
    }
    let line = &lines[idx];
    // MedialAxis.cpp:591-594 — retrieve_endpoint:
    //   return cell->source_category() == SOURCE_CATEGORY_SEGMENT_START_POINT ? line.a : line.b;
    // i.e. SegmentStart -> a, anything else -> b (binary).
    Some(if src_cat == bv::SourceCategory::SegmentStart {
        line.a
    } else {
        line.b
    })
}

/// Distance from a point to a line segment (in scaled coordinates).
fn line_distance_to_point(line: &Line, point: Point) -> f64 {
    let ax = line.a.x as f64;
    let ay = line.a.y as f64;
    let bx = line.b.x as f64;
    let by = line.b.y as f64;
    let px = point.x as f64;
    let py = point.y as f64;

    let dx = bx - ax;
    let dy = by - ay;
    let len_sq = dx * dx + dy * dy;

    if len_sq < 1e-20 {
        return ((px - ax).powi(2) + (py - ay).powi(2)).sqrt();
    }

    let t = ((px - ax) * dx + (py - ay) * dy) / len_sq;
    let t = t.clamp(0.0, 1.0);
    let proj_x = ax + t * dx;
    let proj_y = ay + t * dy;
    ((px - proj_x).powi(2) + (py - proj_y).powi(2)).sqrt()
}

/// Orientation of a line segment (angle in radians, normalized to [0, 2*PI)).
/// Matches BambuStudio's `Line::orientation()` (Line.cpp:53-58):
///   double angle = this->atan2_();   // atan2(b.y - a.y, b.x - a.x)
///   if (angle < 0) angle = 2*PI + angle;
///   return angle;
fn line_orientation(line: &Line) -> f64 {
    // Line.cpp:55 — this->atan2_() == atan2(b.y - a.y, b.x - a.x)
    let dx = (line.b.x - line.a.x) as f64;
    let dy = (line.b.y - line.a.y) as f64;
    let mut angle = dy.atan2(dx);
    // Line.cpp:56 — if (angle < 0) angle = 2*PI + angle;
    if angle < 0.0 {
        angle += 2.0 * std::f64::consts::PI;
    }
    angle
}

/// Euclidean distance between two points (in scaled coordinates).
fn point_dist(a: Point, b: Point) -> f64 {
    let dx = (b.x - a.x) as f64;
    let dy = (b.y - a.y) as f64;
    (dx * dx + dy * dy).sqrt()
}

/// Check if a point is on the ExPolygon boundary (within SCALED_EPSILON).
fn is_on_boundary(expoly: &ExPolygon, point: Point) -> bool {
    let eps = SCALED_EPSILON;
    for line in expolygon_lines(expoly) {
        if line_distance_to_point(&line, point) < eps {
            return true;
        }
    }
    false
}

/// Find intersection of a line with a polygon contour.
/// Returns the closest intersection point to line.a.
///
/// Matches BambuStudio's `Polygon::intersection(Line, Point*)`.
fn contour_intersection(contour: &Polygon, line: Line) -> Option<Point> {
    let pts = contour.points();
    if pts.len() < 2 {
        return None;
    }

    let mut closest: Option<(Point, f64)> = None;

    for i in 0..pts.len() {
        let j = (i + 1) % pts.len();
        let seg = Line::new(pts[i], pts[j]);
        if let Some(intersection) = line_line_intersection(line, seg) {
            let dist = point_dist(line.a, intersection);
            if closest.is_none() || dist < closest.unwrap().1 {
                closest = Some((intersection, dist));
            }
        }
    }

    closest.map(|(p, _)| p)
}

/// Compute the intersection point of two line segments.
/// Returns None if they don't intersect.
fn line_line_intersection(l1: Line, l2: Line) -> Option<Point> {
    let x1 = l1.a.x as f64;
    let y1 = l1.a.y as f64;
    let x2 = l1.b.x as f64;
    let y2 = l1.b.y as f64;
    let x3 = l2.a.x as f64;
    let y3 = l2.a.y as f64;
    let x4 = l2.b.x as f64;
    let y4 = l2.b.y as f64;

    let denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if denom.abs() < 1e-12 {
        return None; // parallel or coincident
    }

    let t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    let u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;

    // Allow slight extension beyond endpoints (matching BambuStudio's
    // extended intersection which tests the extended line against the contour segment)
    if t >= -0.001 && t <= 1.001 && u >= -0.001 && u <= 1.001 {
        let ix = x1 + t * (x2 - x1);
        let iy = y1 + t * (y2 - y1);
        Some(Point::new(ix.round() as Coord, iy.round() as Coord))
    } else {
        None
    }
}

/// Compute distance from a point to the nearest boundary segment.
fn nearest_boundary_distance_lines(point: Point, lines: &[Line]) -> CoordF {
    lines
        .iter()
        .map(|l| line_distance_to_point(l, point) / SCALING_FACTOR)
        .fold(CoordF::INFINITY, f64::min)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// class MedialAxis (Geometry/MedialAxis.hpp, Geometry/MedialAxis.cpp:445-690)
// over `boost_polygon_voronoi::VoronoiDiagram`, the port of boost::polygon's
// builder. This is what `ExPolygon::medial_axis` (ExPolygon.cpp:263-371) runs.
// The functions above are an older port over the `boostvoronoi` crate; they
// keep their callers.
// ---------------------------------------------------------------------------

use crate::boost_polygon_voronoi::{
    construct_voronoi, VoronoiDiagram as VD, SOURCE_CATEGORY_SEGMENT_START_POINT,
};
use crate::geometry::voronoi_offset::boost_polygon as voronoi;
use crate::geometry::voronoi_offset::VertexCategory;

/// `ThickPolyline` as the C++ lays it out (Polyline.hpp:256-274): `width` holds two
/// values per segment (`width.len() == 2 * points.len() - 2`), in the diagram's scaled
/// units, and `endpoints` is the pair `(first, second)`. The crate's own
/// [`ThickPolyline`] keeps one width per point, so the medial axis is built in this
/// form and converted by its caller.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MedialAxisThickPolyline {
    pub points: Vec<Point>,
    pub width: Vec<f64>,
    pub endpoints: (bool, bool),
}

impl MedialAxisThickPolyline {
    /// Polyline.hpp:260-264 `ThickPolyline::reverse`.
    pub fn reverse(&mut self) {
        self.points.reverse();
        self.width.reverse();
        std::mem::swap(&mut self.endpoints.0, &mut self.endpoints.1);
    }

    /// Polyline.hpp:265-268 `ThickPolyline::clear`.
    ///
    /// FIDELITY-NOTE: the C++ clears the points and the widths and leaves `endpoints`
    /// as it is, so in `MedialAxis::build` the `reverse_polyline.endpoints.second` set
    /// for one seed edge is still set for every later seed edge of the same build.
    /// The medial-axis shim's `MaThickPolyline::clear` (medial_axis_shim.cpp:313)
    /// resets both flags as well. The shim is followed.
    pub fn clear(&mut self) {
        self.points.clear();
        self.width.clear();
        // Polyline.hpp:265-268 leaves `endpoints` as it is. `MEDIAL_AXIS_CLEAR_CPP=0` resets
        // the flags, as the shim did (the engine's output until R899). On by default since
        // R900: on Benchy classic it brings 74 more lines onto the oracle's verbatim and 50
        // more in order by a true longest common subsequence; the recorded in-order score,
        // whose matcher is a lower bound, reads 37 fewer (docs/rust-engine/parity, R899-R900).
        if !crate::faithful_gate("MEDIAL_AXIS_CLEAR_CPP") {
            self.endpoints = (false, false);
        }
    }
}

/// `Point(double x, double y)` (Point.hpp:179): `coord_t(std::lrint(v))`, round to nearest,
/// ties to even.
#[inline]
fn point_from_doubles(x: f64, y: f64) -> Point {
    Point::new(x.round_ties_even() as Coord, y.round_ties_even() as Coord)
}

/// Geometry/MedialAxis.hpp:9-41 `class MedialAxis`.
pub struct MedialAxis {
    /// MedialAxis.hpp:17 `Lines m_lines`, `expolygon.lines()`.
    m_lines: Vec<Line>,
    /// MedialAxis.hpp:19-20, for filtering of the skeleton edges.
    m_min_width: f64,
    m_max_width: f64,
    /// MedialAxis.hpp:24 the Voronoi diagram.
    m_vd: VD,
    /// MedialAxis.hpp:37 annotations of the VD skeleton edges, one per pair of twins.
    m_edge_data: Vec<EdgeData>,
}

impl MedialAxis {
    /// MedialAxis.cpp:445-447 `MedialAxis::MedialAxis(min_width, max_width, expolygon)`.
    /// `m_lines` is `ExPolygon::lines()` (ExPolygon.cpp:433-441): the contour's lines, then
    /// each hole's, a polygon of fewer than three points giving none (Polygon.hpp:199-209).
    pub fn new(min_width: f64, max_width: f64, expolygon: &ExPolygon) -> Self {
        let mut m_lines = crate::geometry::polygon::to_lines(&expolygon.contour);
        for h in &expolygon.holes {
            m_lines.extend(crate::geometry::polygon::to_lines(h));
        }
        MedialAxis { m_lines, m_min_width: min_width, m_max_width: max_width, m_vd: VD::new(), m_edge_data: Vec::new() }
    }

    /// MedialAxis.hpp:32-35 `edge_data(edge)`: the index of the `EdgeData` of the pair and
    /// whether `edge` is the second of the pair ("reversed").
    #[inline]
    fn edge_data(&self, edge: usize) -> (usize, bool) {
        (edge / 2, (edge & 1) != 0)
    }

    /// MedialAxis.cpp:449-519 `MedialAxis::build(ThickPolylines* polylines)`.
    pub fn build(&mut self, polylines: &mut Vec<MedialAxisThickPolyline>) {
        // MedialAxis.cpp:451 `m_vd.construct_voronoi(m_lines.begin(), m_lines.end())`: `m_vd` is
        // a `Geometry::VoronoiDiagram` (MedialAxis.hpp:24-25), whose `construct_voronoi`
        // (Voronoi.cpp:25-60) runs boost's builder and then, with `try_to_repair_if_needed`
        // at its default, looks for a degenerated diagram and rebuilds it from rotated input.
        // `MEDIAL_AXIS_VD_REPAIR=0` calls boost's builder alone, as the medial-axis shim did
        // (medial_axis_shim.cpp:323; the engine's behaviour until R901). boost's
        // `insert_segment` takes `int32` coordinates (voronoi_builder.hpp), so the `coord_t`
        // ones are converted as C++ converts them.
        self.m_vd.clear();
        if crate::faithful_gate("MEDIAL_AXIS_VD_REPAIR") {
            let mut vd = crate::geometry::voronoi_diagram::boost_polygon::VoronoiDiagram::new();
            vd.construct_voronoi(&self.m_lines, true);
            self.m_vd = vd.diagram().clone();
        } else {
            let segments: Vec<[i32; 4]> =
                self.m_lines.iter().map(|l| [l.a.x as i32, l.a.y as i32, l.b.x as i32, l.b.y as i32]).collect();
            construct_voronoi(&segments, &mut self.m_vd);
        }
        // MedialAxis.cpp:452.
        voronoi::annotate_inside_outside(&mut self.m_vd, &self.m_lines);

        // MedialAxis.cpp:471-480. collect valid edges (i.e. prune those not belonging to MAT)
        // note: this keeps twins, so it inserts twice the number of the valid edges
        let num_edges = self.m_vd.num_edges();
        self.m_edge_data.clear();
        self.m_edge_data.resize(num_edges / 2, EdgeData::default());
        let mut edge = 0;
        while edge < num_edges {
            if self.m_vd.edges()[edge].is_primary() && self.m_vd.is_finite(edge) && {
                let v0 = self.m_vd.edges()[edge].vertex0().unwrap();
                let v1 = self.m_vd.vertex1(edge).unwrap();
                voronoi::vertex_category(&self.m_vd, v0) == VertexCategory::Inside
                    || voronoi::vertex_category(&self.m_vd, v1) == VertexCategory::Inside
            } && self.validate_edge(edge)
            {
                // Valid skeleton edge.
                let ed = self.edge_data(edge).0;
                self.m_edge_data[ed].active = true;
            }
            edge += 2;
        }

        // MedialAxis.cpp:482-518. iterate through the valid edges to build polylines
        let mut reverse_polyline = MedialAxisThickPolyline::default();
        let mut seed_edge = 0;
        while seed_edge < num_edges {
            let seed = self.edge_data(seed_edge).0;
            if self.m_edge_data[seed].active {
                // Mark this edge as visited.
                self.m_edge_data[seed].active = false;

                // Start a polyline.
                let mut polyline = MedialAxisThickPolyline::default();
                let v0 = self.m_vd.vertices()[self.m_vd.edges()[seed_edge].vertex0().unwrap()];
                let v1 = self.m_vd.vertices()[self.m_vd.vertex1(seed_edge).unwrap()];
                polyline.points.push(point_from_doubles(v0.x(), v0.y()));
                polyline.points.push(point_from_doubles(v1.x(), v1.y()));
                polyline.width.push(self.m_edge_data[seed].width_start);
                polyline.width.push(self.m_edge_data[seed].width_end);
                // Grow the polyline in a forward direction.
                self.process_edge_neighbors(seed_edge, &mut polyline);

                // Grow the polyline in a backward direction.
                reverse_polyline.clear();
                let seed_twin = self.m_vd.edges()[seed_edge].twin().unwrap();
                self.process_edge_neighbors(seed_twin, &mut reverse_polyline);
                polyline.points.splice(0..0, reverse_polyline.points.iter().rev().copied());
                polyline.width.splice(0..0, reverse_polyline.width.iter().rev().copied());
                polyline.endpoints.0 = reverse_polyline.endpoints.1;

                // Prevent loop endpoints from being extended.
                if polyline.points.first() == polyline.points.last() {
                    polyline.endpoints.0 = false;
                    polyline.endpoints.1 = false;
                }

                // Append polyline to result.
                polylines.push(polyline);
            }
            seed_edge += 2;
        }
    }

    /// MedialAxis.cpp:521-528 `MedialAxis::build(Polylines* polylines)`: the thick build, each
    /// polyline's points appended to `polylines`.
    pub fn build_polylines(&mut self, polylines: &mut Vec<crate::geometry::Polyline>) {
        let mut tp: Vec<MedialAxisThickPolyline> = Vec::new();
        self.build(&mut tp);
        polylines.reserve(tp.len());
        for pl in tp {
            polylines.push(crate::geometry::Polyline::from_points(pl.points));
        }
    }

    /// MedialAxis.cpp:545-590 `MedialAxis::process_edge_neighbors(edge, polyline)`.
    pub fn process_edge_neighbors(&mut self, mut edge: usize, polyline: &mut MedialAxisThickPolyline) {
        loop {
            // Since rot_next() works on the edge starting point but we want
            // to find neighbors on the ending point, we just swap edge with
            // its twin.
            let twin = self.m_vd.edges()[edge].twin().unwrap();

            // count neighbors for this edge
            let mut num_neighbors = 0usize;
            let mut first_neighbor: Option<usize> = None;
            let mut neighbor = self.m_vd.rot_next(twin).unwrap();
            while neighbor != twin {
                if self.m_edge_data[self.edge_data(neighbor).0].active {
                    if num_neighbors == 0 {
                        first_neighbor = Some(neighbor);
                    }
                    num_neighbors += 1;
                }
                neighbor = self.m_vd.rot_next(neighbor).unwrap();
            }

            // if we have a single neighbor then we can continue recursively
            if num_neighbors == 1 {
                let first_neighbor = first_neighbor.unwrap();
                let (neighbor_data, reversed) = self.edge_data(first_neighbor);
                if self.m_edge_data[neighbor_data].active {
                    self.m_edge_data[neighbor_data].active = false;
                    let v1 = self.m_vd.vertices()[self.m_vd.vertex1(first_neighbor).unwrap()];
                    polyline.points.push(point_from_doubles(v1.x(), v1.y()));
                    if reversed {
                        polyline.width.push(self.m_edge_data[neighbor_data].width_end);
                        polyline.width.push(self.m_edge_data[neighbor_data].width_start);
                    } else {
                        polyline.width.push(self.m_edge_data[neighbor_data].width_start);
                        polyline.width.push(self.m_edge_data[neighbor_data].width_end);
                    }
                    edge = first_neighbor;
                    // Continue chaining.
                    continue;
                }
            } else if num_neighbors == 0 {
                polyline.endpoints.1 = true;
            } else {
                // T-shaped or star-shaped joint
            }
            // Stop chaining.
            break;
        }
    }

    /// MedialAxis.cpp:594, the lambda `retrieve_segment`: the input line of a cell.
    #[inline]
    pub fn retrieve_segment(&self, cell: usize) -> Line {
        self.m_lines[self.m_vd.cells()[cell].source_index()]
    }

    /// MedialAxis.cpp:595-598, the lambda `retrieve_endpoint`: the end of the input line
    /// that a point cell stands for.
    #[inline]
    pub fn retrieve_endpoint(&self, cell: usize) -> Point {
        let line = self.retrieve_segment(cell);
        if self.m_vd.cells()[cell].source_category() == SOURCE_CATEGORY_SEGMENT_START_POINT {
            line.a
        } else {
            line.b
        }
    }

    /// `Line::distance_to(const Point&)` (Line.hpp:79, 37-58 `line_alg::distance_to_squared`)
    /// as the medial-axis shim writes it (medial_axis_shim.cpp:39-58).
    ///
    /// FIDELITY-NOTE: the C++ evaluates this with Eigen vectors (`squaredNorm`, `dot`,
    /// `(t * v - va).squaredNorm()`), which the compiler does not contract. The shim spells
    /// the same steps as scalar expressions, and every `a * b + c * d` of it is contracted
    /// to `fma(a, b, c * d)` and `t * v - va` to `fma(t, v, -va)`. The shim is followed.
    #[inline]
    fn distance_to(line: &Line, p: Point) -> f64 {
        if medial_axis_eigen_pin() {
            // Line.hpp:43-68, 79 `line_alg::distance_to_squared` on Eigen vectors: nothing fused.
            return line.distance_to(&p);
        }
        let vx = (line.b.x - line.a.x) as f64;
        let vy = (line.b.y - line.a.y) as f64;
        let vax = (p.x - line.a.x) as f64;
        let vay = (p.y - line.a.y) as f64;
        let l2 = vx.mul_add(vx, vy * vy);
        if l2 == 0.0 {
            return vax.mul_add(vax, vay * vay).sqrt();
        }
        let t = vax.mul_add(vx, vay * vy) / l2;
        if t <= 0.0 {
            return vax.mul_add(vax, vay * vay).sqrt();
        }
        if t >= 1.0 {
            let vbx = (p.x - line.b.x) as f64;
            let vby = (p.y - line.b.y) as f64;
            return vbx.mul_add(vbx, vby * vby).sqrt();
        }
        let ex = t.mul_add(vx, -vax);
        let ey = t.mul_add(vy, -vay);
        ex.mul_add(ex, ey * ey).sqrt()
    }

    /// `Line::orientation()` (Line.cpp): `atan2` of the direction, in `[0, 2 PI)`
    /// (medial_axis_shim.cpp:60-64).
    #[inline]
    fn orientation(line: &Line) -> f64 {
        let mut angle = ((line.b.y - line.a.y) as f64).atan2((line.b.x - line.a.x) as f64);
        if angle < 0. {
            angle += 2.0 * std::f64::consts::PI;
        }
        angle
    }

    /// `(a - b).cast<double>().norm()` of two points.
    ///
    /// FIDELITY-NOTE: Eigen's `norm()` is not contracted; the shim's `MaPoint::norm`
    /// (medial_axis_shim.cpp:31) is `sqrt(x * x + y * y)` on scalars, contracted to
    /// `sqrt(fma(x, x, y * y))`. The shim is followed.
    #[inline]
    fn norm(a: Point, b: Point) -> f64 {
        let x = (a.x - b.x) as f64;
        let y = (a.y - b.y) as f64;
        if medial_axis_eigen_pin() {
            // MedialAxis.cpp:643, 647 `(p - line.a).cast<double>().norm()`: Eigen's squaredNorm,
            // the two products rounded, then summed.
            return (x * x + y * y).sqrt();
        }
        x.mul_add(x, y * y).sqrt()
    }

    /// MedialAxis.cpp:592-688 `MedialAxis::validate_edge(edge)`.
    pub fn validate_edge(&mut self, edge: usize) -> bool {
        // MedialAxis.cpp:600-607: the overflow guard is inside `#ifndef CLIPPERLIB_INT32`, and
        // clipper.hpp defines CLIPPERLIB_INT32, so the engine compiles it out.

        // construct the line representing this edge of the Voronoi diagram
        let v0 = self.m_vd.vertices()[self.m_vd.edges()[edge].vertex0().unwrap()];
        let v1 = self.m_vd.vertices()[self.m_vd.vertex1(edge).unwrap()];
        let line = Line::new(point_from_doubles(v0.x(), v0.y()), point_from_doubles(v1.x(), v1.y()));

        // retrieve the original line segments which generated the edge we're checking
        let cell_l = self.m_vd.edges()[edge].cell().unwrap();
        let cell_r = self.m_vd.edges()[self.m_vd.edges()[edge].twin().unwrap()].cell().unwrap();
        let segment_l = self.retrieve_segment(cell_l);
        let segment_r = self.retrieve_segment(cell_r);
        let l_contains_segment = self.m_vd.cells()[cell_l].contains_segment();
        let r_contains_segment = self.m_vd.cells()[cell_r].contains_segment();

        // MedialAxis.cpp:641-647. Thickness of the cross-section at both endpoints of the edge.
        let mut w0 = if r_contains_segment {
            Self::distance_to(&segment_r, line.a) * 2.
        } else {
            Self::norm(self.retrieve_endpoint(cell_r), line.a) * 2.
        };
        let mut w1 = if l_contains_segment {
            Self::distance_to(&segment_l, line.b) * 2.
        } else {
            Self::norm(self.retrieve_endpoint(cell_l), line.b) * 2.
        };

        // SCALED_EPSILON = scale_(EPSILON) = 1e-4 / 1e-5 (libslic3r.h), 10 as the shim has it.
        const SCALED_EPSILON: f64 = 10.0;
        const PI: f64 = std::f64::consts::PI;

        if l_contains_segment && r_contains_segment {
            // calculate the relative angle between the two boundary segments
            let mut angle = (Self::orientation(&segment_r) - Self::orientation(&segment_l)).abs();
            if angle > PI {
                angle = 2. * PI - angle;
            }

            // fabs(angle) ranges from 0 (collinear, same direction) to PI (collinear, opposite direction)
            // we're interested only in segments close to the second case (facing segments)
            // so we allow some tolerance.
            if PI - angle > PI / 8. {
                // angle is not narrow enough
                // only apply this filter to segments that are not too short otherwise their
                // angle could possibly be not meaningful
                // `line.length()`: sqrt(dx * dx + dy * dy) on scalars in the shim, contracted.
                let dx = (line.b.x - line.a.x) as f64;
                let dy = (line.b.y - line.a.y) as f64;
                // Line.hpp:163 `(b - a).cast<double>().norm()` at the pin: not fused.
                let line_length =
                    if medial_axis_eigen_pin() { (dx * dx + dy * dy).sqrt() } else { dx.mul_add(dx, dy * dy).sqrt() };
                if w0 < SCALED_EPSILON || w1 < SCALED_EPSILON || line_length >= self.m_min_width {
                    return false;
                }
            }
        } else if w0 < SCALED_EPSILON || w1 < SCALED_EPSILON {
            return false;
        }

        if (w0 >= self.m_min_width || w1 >= self.m_min_width) && (w0 <= self.m_max_width || w1 <= self.m_max_width) {
            let (ed, reversed) = self.edge_data(edge);
            if reversed {
                std::mem::swap(&mut w0, &mut w1);
            }
            self.m_edge_data[ed].width_start = w0;
            self.m_edge_data[ed].width_end = w1;
            return true;
        }

        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{ExPolygon, Point, Polygon};

    fn s(mm: f64) -> Coord {
        (mm * SCALING_FACTOR).round() as Coord
    }

    fn make_rect(x: f64, y: f64, w: f64, h: f64) -> ExPolygon {
        ExPolygon::new(Polygon::from_points(vec![
            Point::new(s(x), s(y)),
            Point::new(s(x + w), s(y)),
            Point::new(s(x + w), s(y + h)),
            Point::new(s(x), s(y + h)),
        ]))
    }

    fn make_narrow_gap(width: f64, length: f64) -> ExPolygon {
        make_rect(0.0, 0.0, length, width)
    }

    #[test]
    fn test_medial_axis_config_default() {
        let config = MedialAxisConfig::default();
        assert!(config.min_width > 0.0);
        assert!(config.max_width > config.min_width);
    }

    #[test]
    fn test_medial_axis_config_for_gap_fill() {
        let config = MedialAxisConfig::for_gap_fill(0.1, 0.5);
        assert_eq!(config.min_width, 0.1);
        assert_eq!(config.max_width, 0.5);
    }

    #[test]
    fn test_medial_axis_empty_polygon() {
        let config = MedialAxisConfig::default();
        let expoly = ExPolygon::new(Polygon::new());
        let result = compute_medial_axis_thick(&expoly, &config);
        assert!(result.is_empty());
    }

    #[test]
    fn test_medial_axis_too_small() {
        // A polygon narrower than min_width should produce no medial axis
        let config = MedialAxisConfig {
            min_width: 1.0,
            max_width: 2.0,
        };
        let expoly = make_narrow_gap(0.01, 0.01);
        let result = compute_medial_axis_thick(&expoly, &config);
        assert!(result.is_empty());
    }

    #[test]
    fn test_medial_axis_narrow_rectangle() {
        // A 10mm × 0.3mm rectangle should produce a medial axis roughly 10mm long
        let config = MedialAxisConfig::for_gap_fill(0.05, 0.5);
        let expoly = make_narrow_gap(0.3, 10.0);
        let result = compute_medial_axis_thick(&expoly, &config);

        // Should produce at least one polyline
        assert!(
            !result.is_empty(),
            "Narrow rectangle should have a medial axis"
        );

        // Total length should be roughly the rectangle's length (~10mm)
        let total_length: CoordF = result.iter().map(|tp| tp.length_mm()).sum();
        assert!(
            total_length > 5.0,
            "Medial axis too short: {:.2}mm (expected ~10mm)",
            total_length
        );
        assert!(
            total_length < 15.0,
            "Medial axis too long: {:.2}mm (expected ~10mm)",
            total_length
        );

        // All widths should be approximately 0.3mm
        for tp in &result {
            for &w in &tp.widths {
                assert!(
                    w > 0.0 && w < 0.6,
                    "Width {:.4}mm out of range for 0.3mm gap",
                    w
                );
            }
        }
    }

    #[test]
    fn test_medial_axis_variable_width_output() {
        // Create a trapezoidal gap that tapers from 0.4mm to 0.2mm
        let expoly = ExPolygon::new(Polygon::from_points(vec![
            Point::new(s(0.0), s(0.0)),
            Point::new(s(10.0), s(0.0)),
            Point::new(s(10.0), s(0.2)),
            Point::new(s(0.0), s(0.4)),
        ]));

        let config = MedialAxisConfig::for_gap_fill(0.05, 0.5);
        let result = compute_medial_axis_thick(&expoly, &config);

        // Should produce polylines with variable widths
        if !result.is_empty() {
            let has_varying_width = result.iter().any(|tp| {
                if tp.widths.len() < 2 {
                    return false;
                }
                let min_w = tp.widths.iter().cloned().fold(f64::INFINITY, f64::min);
                let max_w = tp.widths.iter().cloned().fold(0.0f64, f64::max);
                (max_w - min_w) > 0.01
            });
            // Trapezoidal shape should produce varying widths
            assert!(
                has_varying_width,
                "Trapezoidal gap should have varying width"
            );
        }
    }

    #[test]
    fn test_medial_axis_multi() {
        let config = MedialAxisConfig::for_gap_fill(0.05, 0.5);
        let gaps = vec![make_narrow_gap(0.3, 5.0), make_narrow_gap(0.2, 8.0)];

        let result = compute_medial_axis_multi(&gaps, &config);
        // Should produce results from both gap regions
        assert!(
            !result.is_empty(),
            "Multi-region medial axis should produce results"
        );
    }

    #[test]
    fn test_compute_medial_axis_legacy_api() {
        // Test the legacy API that returns plain Polylines
        let polygon = Polygon::from_points(vec![
            Point::new(s(0.0), s(0.0)),
            Point::new(s(10.0), s(0.0)),
            Point::new(s(10.0), s(0.3)),
            Point::new(s(0.0), s(0.3)),
        ]);

        let result = compute_medial_axis(&polygon);
        // Narrow rectangle should produce a medial axis
        assert!(!result.is_empty(), "Legacy API should produce polylines");
    }

    #[test]
    fn test_thick_polyline_widths_are_positive() {
        let config = MedialAxisConfig::for_gap_fill(0.05, 0.5);
        let expoly = make_narrow_gap(0.3, 10.0);
        let result = compute_medial_axis_thick(&expoly, &config);

        for tp in &result {
            for &w in &tp.widths {
                assert!(w > 0.0, "All widths should be positive, got {}", w);
            }
        }
    }

    #[test]
    fn test_distance_to_boundary_public() {
        let expoly = make_rect(0.0, 0.0, 10.0, 10.0);
        // Center of 10×10 square — should be 5mm from boundary
        let center = Point::new(s(5.0), s(5.0));
        let dist = distance_to_boundary(&expoly, center);
        assert!(
            (dist - 5.0).abs() < 0.1,
            "Distance to boundary should be ~5mm, got {:.2}",
            dist
        );

        // Point near edge — should be close to 0
        let near_edge = Point::new(s(0.1), s(5.0));
        let dist2 = distance_to_boundary(&expoly, near_edge);
        assert!(
            dist2 < 0.2,
            "Distance near edge should be ~0.1mm, got {:.2}",
            dist2
        );
    }

    #[test]
    fn test_line_distance_to_point() {
        let line = Line::new(Point::new(0, 0), Point::new(s(10.0), 0));
        // Point directly above the midpoint
        let pt = Point::new(s(5.0), s(3.0));
        let dist = line_distance_to_point(&line, pt);
        let expected = s(3.0) as f64;
        assert!(
            (dist - expected).abs() < 1.0,
            "Distance should be ~{}  got {}",
            expected,
            dist
        );
    }

    #[test]
    fn test_line_line_intersection_basic() {
        // Two perpendicular lines crossing at (5, 5) in mm
        let l1 = Line::new(Point::new(s(0.0), s(5.0)), Point::new(s(10.0), s(5.0)));
        let l2 = Line::new(Point::new(s(5.0), s(0.0)), Point::new(s(5.0), s(10.0)));
        let result = line_line_intersection(l1, l2);
        assert!(result.is_some(), "Perpendicular lines should intersect");
        let pt = result.unwrap();
        assert!(
            (pt.x - s(5.0)).abs() < 2 && (pt.y - s(5.0)).abs() < 2,
            "Intersection should be near (5, 5)mm, got ({}, {})",
            pt.x,
            pt.y
        );
    }
}
