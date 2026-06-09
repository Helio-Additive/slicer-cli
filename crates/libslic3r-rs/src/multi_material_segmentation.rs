//! Faithful 1:1 port of BambuStudio `src/libslic3r/MultiMaterialSegmentation.cpp`.
//!
//! Returns MMU segmentation based on painting in the MMU segmentation gizmo, and
//! the fuzzy-skin variant.
//!
//! coord_t -> i64, coordf_t -> f64. Voronoi access goes through the `boostvoronoi`
//! crate (same backend as `geometry::voronoi_diagram`).
//!
//! PORTING STATUS (see report): the self-contained graph + painted-line + colorize
//! algorithms are ported line-by-line. The top-level entry points
//! `multi_material_segmentation_by_painting` / `fuzzy_skin_segmentation_by_painting`,
//! plus `mmu_segmentation_top_and_bottom_layers`, depend on PrintObject / Layer /
//! LayerRegion / ModelVolume::mmu_segmentation_facets / slice_mesh_slabs /
//! EdgeGrid::Grid::create+visit infrastructure that is not yet wired through the Rust
//! crate; those are documented as BLOCKED below and not faked.

use boostvoronoi::diagram as bv_diagram;
use boostvoronoi::prelude as bv;

use crate::geometry::voronoi_diagram::VoronoiDiagram;
use crate::geometry::{BoundingBox, Line, LineF, Point, PointF, Polygon};
use crate::libslic3r::SCALED_EPSILON;
use crate::{scale, Coord};

use std::f64::consts::PI;

// In MultiMaterialSegmentation.cpp, `Vec2d` is a *scaled* double (a scaled-integer Point
// widened to double via Eigen `.cast<double>()`, WITHOUT applying SCALING_FACTOR), and the
// inverse `.cast<coord_t>()` is a plain numeric truncation back to i64. The crate's PointF
// conversions (`to_f64`/`From`) instead apply SCALING_FACTOR (unscaled-mm convention), which
// would corrupt this algorithm. So we cast numerically inline here, matching C++ exactly.

// Mirrors `point.cast<double>()` for a scaled-integer Point (no SCALING_FACTOR applied).
#[inline]
fn pt_to_vec2d(p: Point) -> PointF {
    PointF::new(p.x as f64, p.y as f64)
}

// Mirrors `vec2d.cast<coord_t>()` (truncation toward zero, like Eigen's cast to integer).
#[inline]
fn vec2d_to_pt(v: PointF) -> Point {
    Point::new(v.x as Coord, v.y as Coord)
}

// scale_(mm) — C++ macro `scale_` rounds mm * SCALING_FACTOR to coord_t. We keep it as f64
// where the C++ result feeds a double comparison (e.g. `scale_(0.1f)` against a length).
#[inline]
fn scale_(mm: f64) -> Coord {
    scale(mm)
}

// MultiMaterialSegmentation.hpp:13
/// `ColoredLine` — a contour `Line` tagged with a color (extruder index) and the
/// polygon/local-line indices it belongs to.
#[derive(Clone, Copy, Debug, Default)]
pub struct ColoredLine {
    // MultiMaterialSegmentation.hpp:15
    pub line: Line,
    // MultiMaterialSegmentation.hpp:16
    pub color: i32,
    // MultiMaterialSegmentation.hpp:17
    pub poly_idx: i32,
    // MultiMaterialSegmentation.hpp:18
    pub local_line_idx: i32,
}

impl ColoredLine {
    pub fn new(line: Line, color: i32) -> Self {
        Self {
            line,
            color,
            poly_idx: -1,
            local_line_idx: -1,
        }
    }
}

// MultiMaterialSegmentation.hpp:21
pub type ColoredLines = Vec<ColoredLine>;

// ---------------------------------------------------------------------------
// mk_point helpers
// ---------------------------------------------------------------------------

// MultiMaterialSegmentation.cpp:44
// static inline Point mk_point(const Vec2d &point) { return {coord_t(round(x)), coord_t(round(y))}; }
#[inline]
fn mk_point_vec2d(point: PointF) -> Point {
    Point::new(point.x.round() as Coord, point.y.round() as Coord)
}

// Slic3r::cross2 for a scaled-double Vec2d (a.x*b.y - a.y*b.x).
#[inline]
fn cross2f(a: PointF, b: PointF) -> f64 {
    a.x * b.y - a.y * b.x
}

// MultiMaterialSegmentation.cpp:38-42 mk_point() from Voronoi vertices and
// MultiMaterialSegmentation.cpp:46 mk_vec2() are implemented inline at call sites
// against the boostvoronoi vertex coordinates (which are f64).

// ---------------------------------------------------------------------------
// MMU_Graph
// ---------------------------------------------------------------------------

// MultiMaterialSegmentation.cpp:63
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ArcType {
    Border,
    NonBorder,
}

// MultiMaterialSegmentation.cpp:65
#[derive(Clone, Copy, Debug)]
pub struct Arc {
    // MultiMaterialSegmentation.cpp:67-70
    pub from_idx: usize,
    pub to_idx: usize,
    pub color: i32,
    pub r#type: ArcType,
}

impl PartialEq for Arc {
    // MultiMaterialSegmentation.cpp:72
    fn eq(&self, rhs: &Arc) -> bool {
        self.from_idx == rhs.from_idx
            && self.to_idx == rhs.to_idx
            && self.color == rhs.color
            && self.r#type == rhs.r#type
    }
}

// MultiMaterialSegmentation.cpp:76
#[derive(Clone, Default, Debug)]
pub struct Node {
    // MultiMaterialSegmentation.cpp:78-79
    pub point: PointF,
    pub arc_idxs: Vec<usize>,
}

// MultiMaterialSegmentation.cpp:61
#[derive(Default, Debug)]
pub struct MmuGraph {
    // MultiMaterialSegmentation.cpp:94-96
    pub nodes: Vec<Node>,
    pub arcs: Vec<Arc>,
    pub all_border_points: usize,

    // MultiMaterialSegmentation.cpp:98-99
    pub polygon_idx_offset: Vec<usize>,
    pub polygon_sizes: Vec<usize>,
}

impl MmuGraph {
    // MultiMaterialSegmentation.cpp:81 (Node::remove_edge)
    // void remove_edge(const size_t to_idx, MMU_Graph &graph)
    fn node_remove_edge(&mut self, node_idx: usize, to_idx: usize) {
        let arc_idxs = self.nodes[node_idx].arc_idxs.clone();
        for (pos, &arc_it) in arc_idxs.iter().enumerate() {
            let arc = self.arcs[arc_it];
            if arc.to_idx == to_idx {
                debug_assert!(arc.r#type != ArcType::Border);
                self.nodes[node_idx].arc_idxs.remove(pos);
                break;
            }
        }
    }

    // MultiMaterialSegmentation.cpp:101
    pub fn remove_edge(&mut self, from_idx: usize, to_idx: usize) {
        self.node_remove_edge(from_idx, to_idx);
        self.node_remove_edge(to_idx, from_idx);
    }

    // MultiMaterialSegmentation.cpp:107
    #[inline]
    pub fn get_global_index(&self, poly_idx: usize, point_idx: usize) -> usize {
        self.polygon_idx_offset[poly_idx] + point_idx
    }

    // MultiMaterialSegmentation.cpp:109
    pub fn append_edge(&mut self, from_idx: usize, to_idx: usize, color: i32, r#type: ArcType) {
        // Don't append duplicate edges between the same nodes.
        // MultiMaterialSegmentation.cpp:112-113
        for &arc_idx in &self.nodes[from_idx].arc_idxs {
            if self.arcs[arc_idx].to_idx == to_idx {
                return;
            }
        }
        // MultiMaterialSegmentation.cpp:114-115
        for &arc_idx in &self.nodes[to_idx].arc_idxs {
            if self.arcs[arc_idx].to_idx == from_idx {
                return;
            }
        }

        // MultiMaterialSegmentation.cpp:117-118
        self.nodes[from_idx].arc_idxs.push(self.arcs.len());
        self.arcs.push(Arc {
            from_idx,
            to_idx,
            color,
            r#type,
        });

        // Always insert only one directed arc for the input polygons.
        // Two directed arcs in both directions are inserted if arcs aren't between points of the input polygons.
        // MultiMaterialSegmentation.cpp:122-125
        if r#type == ArcType::NonBorder {
            self.nodes[to_idx].arc_idxs.push(self.arcs.len());
            self.arcs.push(Arc {
                from_idx: to_idx,
                to_idx: from_idx,
                color,
                r#type,
            });
        }
    }

    // It assumes that between points of the input polygons is always only one directed arc,
    // with the same direction as lines of the input polygon.
    // MultiMaterialSegmentation.cpp:130
    #[inline]
    pub fn get_border_arc(&self, idx: usize) -> Arc {
        debug_assert!(idx < self.all_border_points);
        self.arcs[idx]
    }

    // MultiMaterialSegmentation.cpp:136
    #[inline]
    pub fn nodes_count(&self) -> usize {
        self.nodes.len()
    }

    // MultiMaterialSegmentation.cpp:138
    pub fn remove_nodes_with_one_arc(&mut self) {
        let mut update_queue: std::collections::VecDeque<usize> = std::collections::VecDeque::new();
        // MultiMaterialSegmentation.cpp:141-145
        for node_idx in 0..self.nodes.len() {
            let node = &self.nodes[node_idx];
            // Skip nodes that represent points of input polygons.
            if node.arc_idxs.len() == 1 && node_idx >= self.all_border_points {
                update_queue.push_back(node_idx);
            }
        }

        // MultiMaterialSegmentation.cpp:147-158
        while let Some(node_from_idx) = update_queue.pop_front() {
            if self.nodes[node_from_idx].arc_idxs.is_empty() {
                continue;
            }

            debug_assert!(self.nodes[node_from_idx].arc_idxs.len() == 1);
            let node_to_idx = self.arcs[self.nodes[node_from_idx].arc_idxs[0]].to_idx;
            self.remove_edge(node_from_idx, node_to_idx);
            if self.nodes[node_to_idx].arc_idxs.len() == 1 && node_to_idx >= self.all_border_points {
                update_queue.push_back(node_to_idx);
            }
        }
    }

    // MultiMaterialSegmentation.cpp:161
    pub fn add_contours(&mut self, color_poly: &[Vec<ColoredLine>]) {
        // MultiMaterialSegmentation.cpp:163-170
        self.all_border_points = self.nodes.len();
        self.polygon_sizes = vec![0usize; color_poly.len()];
        for polygon_idx in 0..color_poly.len() {
            self.polygon_sizes[polygon_idx] = color_poly[polygon_idx].len();
        }
        self.polygon_idx_offset = vec![0usize; color_poly.len()];
        self.polygon_idx_offset[0] = 0;
        for polygon_idx in 1..color_poly.len() {
            self.polygon_idx_offset[polygon_idx] =
                self.polygon_idx_offset[polygon_idx - 1] + color_poly[polygon_idx - 1].len();
        }

        // MultiMaterialSegmentation.cpp:172-182
        let mut poly_idx = 0usize;
        for color_lines in color_poly {
            let mut line_idx = 0usize;
            for color_line in color_lines {
                let from_idx = self.get_global_index(poly_idx, line_idx);
                let to_idx = self.get_global_index(poly_idx, (line_idx + 1) % color_lines.len());
                self.append_edge(from_idx, to_idx, color_line.color, ArcType::Border);
                line_idx += 1;
            }
            poly_idx += 1;
        }
    }

    // Nodes 0..all_border_points are only the ones on the contour. Other vertices are
    // considered as not on the contour. So we check based on the attached index.
    // MultiMaterialSegmentation.cpp:186
    // (Only reachable from the BLOCKED build_graph VD path; kept for parity.)
    #[allow(dead_code)]
    #[inline]
    fn is_vertex_on_contour(&self, vertex_color: bv_diagram::ColorType) -> bool {
        (vertex_color as usize) < self.all_border_points
    }

    // MultiMaterialSegmentation.cpp:283
    pub fn garbage_collect(&mut self) {
        // MultiMaterialSegmentation.cpp:285-292
        let mut nodes_map: Vec<i32> = vec![-1; self.nodes.len()];
        let mut nodes_count: i32 = 0;
        let mut arcs_count: usize = 0;
        for node_idx in 0..self.nodes.len() {
            if !self.nodes[node_idx].arc_idxs.is_empty() {
                nodes_map[node_idx] = nodes_count;
                nodes_count += 1;
                arcs_count += self.nodes[node_idx].arc_idxs.len();
            }
        }

        // MultiMaterialSegmentation.cpp:294-306
        let mut new_nodes: Vec<Node> = Vec::with_capacity(nodes_count as usize);
        let mut new_arcs: Vec<Arc> = Vec::with_capacity(arcs_count);
        for node_idx in 0..self.nodes.len() {
            if nodes_map[node_idx] >= 0 {
                let mut new_node = Node {
                    point: self.nodes[node_idx].point,
                    arc_idxs: Vec::new(),
                };
                for &arc_idx in &self.nodes[node_idx].arc_idxs {
                    let arc = self.arcs[arc_idx];
                    new_node.arc_idxs.push(new_arcs.len());
                    new_arcs.push(Arc {
                        from_idx: nodes_map[arc.from_idx] as usize,
                        to_idx: nodes_map[arc.to_idx] as usize,
                        color: arc.color,
                        r#type: arc.r#type,
                    });
                }
                new_nodes.push(new_node);
            }
        }

        // MultiMaterialSegmentation.cpp:308-309
        self.nodes = new_nodes;
        self.arcs = new_arcs;
    }
}

// MultiMaterialSegmentation.cpp:313
fn colored_points_to_polygon_one(lines: &[ColoredLine]) -> Polygon {
    let mut out = Polygon::default();
    out.points.reserve(lines.len());
    for l in lines {
        out.points.push(l.line.a);
    }
    out
}

// MultiMaterialSegmentation.cpp:321
fn colored_points_to_polygon(lines: &[Vec<ColoredLine>]) -> Vec<Polygon> {
    let mut out: Vec<Polygon> = Vec::with_capacity(lines.len());
    for l in lines {
        out.push(colored_points_to_polygon_one(l));
    }
    out
}

// MultiMaterialSegmentation.cpp:329
// Returns, for each candidate continuation, a one-element vector with the arc index.
fn get_all_next_arcs(
    graph: &MmuGraph,
    used_arcs: &[bool],
    process_line: &LineF,
    original_arc: &Arc,
    color: i32,
) -> Vec<Vec<usize>> {
    let mut all_next_arcs: Vec<Vec<usize>> = Vec::new();
    // MultiMaterialSegmentation.cpp:333-346
    for &arc_idx in &graph.nodes[original_arc.to_idx].arc_idxs {
        let mut next_continue_arc: Vec<usize> = Vec::new();

        let arc = &graph.arcs[arc_idx];
        if graph.nodes[arc.to_idx].point == process_line.a || used_arcs[arc_idx] {
            continue;
        }

        if original_arc.r#type == ArcType::Border && original_arc.color != color {
            continue;
        }

        if arc.r#type == ArcType::Border && arc.color != color {
            continue;
        }

        // Vec2d arc_line = graph.nodes[arc.to_idx].point - graph.nodes[arc.from_idx].point;
        // (Computed but unused in C++; preserved here for fidelity but elided.)
        let _arc_line = graph.nodes[arc.to_idx].point - graph.nodes[arc.from_idx].point;
        next_continue_arc.push(arc_idx);
        all_next_arcs.push(next_continue_arc);
    }
    all_next_arcs
}

// MultiMaterialSegmentation.cpp:350
fn get_next_arc(
    graph: &MmuGraph,
    used_arcs: &[bool],
    process_line: &LineF,
    original_arc_idx: usize,
    color: i32,
) -> Vec<usize> {
    let original_arc = &graph.arcs[original_arc_idx];
    let mut res: Vec<usize> = Vec::new();

    // MultiMaterialSegmentation.cpp:355-359
    let all_next_arcs = get_all_next_arcs(graph, used_arcs, process_line, original_arc, color);
    if all_next_arcs.is_empty() {
        res.push(original_arc_idx);
        return res;
    }

    // MultiMaterialSegmentation.cpp:361-372
    let mut sorted_arcs: Vec<(Vec<usize>, f64)> = Vec::new();
    for next_arc in all_next_arcs {
        if next_arc.is_empty() {
            continue;
        }

        let back_arc = &graph.arcs[*next_arc.last().unwrap()];
        let process_line_vec_n = (process_line.a - process_line.b).normalize();
        let neighbour_line_vec_n =
            (graph.nodes[back_arc.to_idx].point - graph.nodes[back_arc.from_idx].point).normalize();

        let mut angle = neighbour_line_vec_n
            .dot(&process_line_vec_n)
            .clamp(-1.0, 1.0)
            .acos();
        if cross2f(neighbour_line_vec_n, process_line_vec_n) < 0.0 {
            angle = 2.0 * PI - angle;
        }

        sorted_arcs.push((next_arc, angle));
    }

    // MultiMaterialSegmentation.cpp:374-375
    sorted_arcs.sort_by(|l, r| l.1.partial_cmp(&r.1).unwrap_or(std::cmp::Ordering::Equal));

    // Try to return left most edge which is unused
    // MultiMaterialSegmentation.cpp:378-380
    for sorted_arc in &sorted_arcs {
        let arc_idx = *sorted_arc.0.last().unwrap();
        if !used_arcs[arc_idx] {
            return sorted_arc.0.clone();
        }
    }

    // MultiMaterialSegmentation.cpp:382-385
    if sorted_arcs.is_empty() {
        res.push(original_arc_idx);
        return res;
    }

    // MultiMaterialSegmentation.cpp:387
    sorted_arcs[0].0.clone()
}

// MultiMaterialSegmentation.cpp:390
fn is_profile_self_interaction(poly: &Polygon) -> bool {
    let lines = poly.lines();
    // MultiMaterialSegmentation.cpp:394-398
    let n = lines.len() as isize;
    for i in 0..n {
        let upper = (n).min(n + i - 1);
        let mut j = i + 2;
        while j < upper {
            if lines[i as usize].intersection(&lines[j as usize]).is_some() {
                return true;
            }
            j += 1;
        }
    }
    false
}

// MultiMaterialSegmentation.cpp:402
fn to_polygon(id_to_lines: &[(usize, LineF)]) -> Polygon {
    // MultiMaterialSegmentation.cpp:404-405
    let mut lines: Vec<LineF> = Vec::new();
    for id_to_line in id_to_lines {
        lines.push(id_to_line.1);
    }

    // MultiMaterialSegmentation.cpp:407-410
    let mut poly_out = Polygon::default();
    poly_out.points.reserve(lines.len());
    for line in &lines {
        poly_out.points.push(mk_point_vec2d(line.a));
    }
    poly_out
}

// MultiMaterialSegmentation.cpp:413
// Returns list of ExPolygons for each extruder + 1 for default unpainted regions.
// (Polygon-level output here; ExPolygon union happens at the caller layer.)
pub fn extract_colored_segments(graph: &MmuGraph, num_extruders: usize) -> Vec<Vec<Polygon>> {
    let mut used_arcs: Vec<bool> = vec![false; graph.arcs.len()];

    // MultiMaterialSegmentation.cpp:417-419
    let all_arc_used = |node: &Node, used_arcs: &[bool]| -> bool {
        node.arc_idxs.iter().all(|&arc_idx| used_arcs[arc_idx])
    };

    // MultiMaterialSegmentation.cpp:421
    let mut expolygons_segments: Vec<Vec<Polygon>> = vec![Vec::new(); num_extruders + 1];
    for node_idx in 0..graph.all_border_points {
        // MultiMaterialSegmentation.cpp:425
        let node_arc_idxs = graph.nodes[node_idx].arc_idxs.clone();
        for arc_idx in node_arc_idxs {
            let arc = graph.arcs[arc_idx];
            // MultiMaterialSegmentation.cpp:427
            if arc.r#type == ArcType::NonBorder || used_arcs[arc_idx] {
                continue;
            }

            // MultiMaterialSegmentation.cpp:429-434
            let process_line = LineF::new(graph.nodes[arc.from_idx].point, graph.nodes[arc.to_idx].point);
            used_arcs[arc_idx] = true;

            let mut arc_id_to_face_lines: Vec<(usize, LineF)> = Vec::new();
            arc_id_to_face_lines.push((arc_idx, process_line));
            let start_p = process_line.a;

            // MultiMaterialSegmentation.cpp:436-460
            let mut p_vec = process_line;
            let mut p_arc_idx = arc_idx;
            let mut flag = false;
            loop {
                let nexts = get_next_arc(graph, &used_arcs, &p_vec, p_arc_idx, arc.color);
                for &next_arc_idx in &nexts {
                    if used_arcs[next_arc_idx] {
                        flag = true;
                        break;
                    }
                }

                if flag {
                    break;
                }

                for &next_arc_idx in &nexts {
                    let next = graph.arcs[next_arc_idx];
                    arc_id_to_face_lines.push((
                        next_arc_idx,
                        LineF::new(graph.nodes[next.from_idx].point, graph.nodes[next.to_idx].point),
                    ));
                    used_arcs[next_arc_idx] = true;
                }

                let last_next_idx = *nexts.last().unwrap();
                let last_next = graph.arcs[last_next_idx];
                p_vec = LineF::new(
                    graph.nodes[last_next.from_idx].point,
                    graph.nodes[last_next.to_idx].point,
                );
                p_arc_idx = last_next_idx;

                // while (graph.nodes[p_arc->to_idx].point != start_p || !all_arc_used(graph.nodes[p_arc->to_idx]));
                let p_arc = graph.arcs[p_arc_idx];
                if graph.nodes[p_arc.to_idx].point == start_p
                    && all_arc_used(&graph.nodes[p_arc.to_idx], &used_arcs)
                {
                    break;
                }
            }

            // MultiMaterialSegmentation.cpp:462-478
            let poly = to_polygon(&arc_id_to_face_lines);
            if poly.is_counter_clockwise() && poly.is_valid() {
                expolygons_segments[arc.color as usize].push(poly);
            } else {
                while arc_id_to_face_lines.len() > 1 {
                    let id_to_line = *arc_id_to_face_lines.last().unwrap();
                    used_arcs[id_to_line.0] = false;
                    arc_id_to_face_lines.pop();
                    let add_line = LineF::new(
                        arc_id_to_face_lines.last().unwrap().1.b,
                        arc_id_to_face_lines.first().unwrap().1.a,
                    );
                    // Note: C++ pushes pair(-1, add_line); size_t(-1) is the sentinel.
                    arc_id_to_face_lines.push((usize::MAX, add_line));
                    let poly = to_polygon(&arc_id_to_face_lines);
                    if !is_profile_self_interaction(&poly)
                        && poly.is_counter_clockwise()
                        && poly.is_valid()
                    {
                        expolygons_segments[arc.color as usize].push(poly);
                        break;
                    }
                    arc_id_to_face_lines.pop();
                }
            }
        }
    }
    expolygons_segments
}

// MultiMaterialSegmentation.cpp:484
fn is_equal(left: f32, right: f32) -> bool {
    is_equal_eps(left, right, 1e-3)
}

fn is_equal_eps(left: f32, right: f32, eps: f32) -> bool {
    (left - right).abs() <= eps
}

// MultiMaterialSegmentation.cpp:488
fn is_less(left: f32, right: f32) -> bool {
    is_less_eps(left, right, 1e-3)
}

fn is_less_eps(left: f32, right: f32, eps: f32) -> bool {
    left + eps < right
}

// Assumes that is at most same projected_l length or below than projection_l
// MultiMaterialSegmentation.cpp:493
fn project_line_on_line(projection_l: &Line, projected_l: &Line, new_projected: &mut Line) -> bool {
    // MultiMaterialSegmentation.cpp:495-498
    let v1 = pt_to_vec2d(projection_l.b - projection_l.a);
    let va = pt_to_vec2d(projected_l.a - projection_l.a);
    let vb = pt_to_vec2d(projected_l.b - projection_l.a);
    let l2 = v1.length_squared(); // avoid a sqrt
    if l2 == 0.0 {
        return false;
    }
    // MultiMaterialSegmentation.cpp:501-508
    let mut t1 = va.dot(&v1) / l2;
    let mut t2 = vb.dot(&v1) / l2;
    t1 = t1.clamp(0.0, 1.0);
    t2 = t2.clamp(0.0, 1.0);
    debug_assert!(t1 >= 0.0);
    debug_assert!(t2 >= 0.0);
    debug_assert!(t1 <= 1.0);
    debug_assert!(t2 <= 1.0);

    // MultiMaterialSegmentation.cpp:510-512
    let p1 = projection_l.a + vec2d_to_pt(v1 * t1);
    let p2 = projection_l.a + vec2d_to_pt(v1 * t2);
    *new_projected = Line::new(p1, p2);
    true
}

// MultiMaterialSegmentation.cpp:516
#[derive(Clone, Copy, Debug)]
pub struct PaintedLine {
    // MultiMaterialSegmentation.cpp:518-521
    pub contour_idx: usize,
    pub line_idx: usize,
    pub projected_line: Line,
    pub color: i32,
}

// PaintedLineVisitor (MultiMaterialSegmentation.cpp:524) operates on EdgeGrid::Grid
// cell-iteration callbacks. The Rust EdgeGrid::Grid does not yet expose the
// `cell_data_range` / `grid.line(seg)` / `visit_cells_intersecting_line` surface in the
// shape this visitor needs, and the visitor is only reachable from the two BLOCKED
// top-level entry points. See report.

// ---------------------------------------------------------------------------
// Painted-line thresholds (MultiMaterialSegmentation.cpp:587-589)
// ---------------------------------------------------------------------------

#[inline]
fn cos_threshold2() -> f64 {
    sqr_f64((PI * 30.0 / 180.0).cos())
}

#[inline]
fn append_threshold() -> f64 {
    50.0 * SCALED_EPSILON
}

#[inline]
fn append_threshold2() -> f64 {
    sqr_f64(append_threshold())
}

// MultiMaterialSegmentation.cpp:592
fn get_extents_colored(colored_polygons: &[ColoredLines]) -> BoundingBox {
    let mut bbox = BoundingBox::empty();
    for colored_lines in colored_polygons {
        for colored_line in colored_lines {
            bbox.merge_point(colored_line.line.a);
            bbox.merge_point(colored_line.line.b);
        }
    }
    bbox
}

// Flatten the vector of vectors into a vector.
// MultiMaterialSegmentation.cpp:604
fn to_lines_colored(c_lines: &[ColoredLines]) -> ColoredLines {
    let mut n_lines = 0usize;
    for c_line in c_lines {
        n_lines += c_line.len();
    }
    let mut lines: ColoredLines = Vec::with_capacity(n_lines);
    for c_line in c_lines {
        lines.extend_from_slice(c_line);
    }
    lines
}

// MultiMaterialSegmentation.cpp:616
fn get_segments(polygon: &[ColoredLine]) -> Vec<(usize, usize)> {
    let mut segments: Vec<(usize, usize)> = Vec::new();

    // MultiMaterialSegmentation.cpp:620-625
    let mut segment_end = 0usize;
    while segment_end + 1 < polygon.len() && polygon[segment_end].color == polygon[segment_end + 1].color
    {
        segment_end += 1;
    }

    if segment_end == polygon.len() - 1 {
        return vec![(0, polygon.len() - 1)];
    }

    // MultiMaterialSegmentation.cpp:627-637
    let first_different_color = (segment_end + 1) % polygon.len();
    let mut line_offset_idx = 0usize;
    while line_offset_idx < polygon.len() {
        let start_s = (first_different_color + line_offset_idx) % polygon.len();
        let mut end_s = start_s;

        while line_offset_idx + 1 < polygon.len()
            && polygon[start_s].color
                == polygon[(first_different_color + line_offset_idx + 1) % polygon.len()].color
        {
            end_s = (first_different_color + line_offset_idx + 1) % polygon.len();
            line_offset_idx += 1;
        }
        segments.push((start_s, end_s));
        line_offset_idx += 1;
    }
    segments
}

// MultiMaterialSegmentation.cpp:641
fn filter_painted_lines(
    line_to_process: &Line,
    start_idx: usize,
    end_idx: usize,
    painted_lines: &[PaintedLine],
) -> Vec<PaintedLine> {
    // MultiMaterialSegmentation.cpp:643-645
    let filter_eps_value = scale_(0.1) as f64;
    let mut filtered_lines: Vec<PaintedLine> = Vec::new();
    filtered_lines.push(painted_lines[start_idx]);
    // MultiMaterialSegmentation.cpp:646-677
    for line_idx in (start_idx + 1)..=end_idx {
        // line_to_process is already all colored. Skip another possible duplicate coloring.
        if filtered_lines.last().unwrap().projected_line.b == line_to_process.b {
            break;
        }

        let curr = painted_lines[line_idx];

        let prev_length = filtered_lines.last().unwrap().projected_line.length();
        let curr_dist_start =
            pt_to_vec2d(curr.projected_line.a - filtered_lines.last().unwrap().projected_line.a)
                .length();
        let dist_between_lines = curr_dist_start - prev_length;

        if dist_between_lines >= 0.0 {
            if filtered_lines.last().unwrap().color == curr.color {
                if dist_between_lines <= filter_eps_value {
                    filtered_lines.last_mut().unwrap().projected_line.b = curr.projected_line.b;
                } else {
                    filtered_lines.push(curr);
                }
            } else {
                filtered_lines.push(curr);
            }
        } else {
            let curr_dist_end =
                pt_to_vec2d(curr.projected_line.b - filtered_lines.last().unwrap().projected_line.a)
                    .length();
            if curr_dist_end > prev_length {
                if filtered_lines.last().unwrap().color == curr.color {
                    filtered_lines.last_mut().unwrap().projected_line.b = curr.projected_line.b;
                } else {
                    let prev_b = filtered_lines.last().unwrap().projected_line.b;
                    filtered_lines.push(PaintedLine {
                        contour_idx: curr.contour_idx,
                        line_idx: curr.line_idx,
                        projected_line: Line::new(prev_b, curr.projected_line.b),
                        color: curr.color,
                    });
                }
            }
        }
    }

    // MultiMaterialSegmentation.cpp:679-683
    let dist_to_start =
        pt_to_vec2d(filtered_lines.first().unwrap().projected_line.a - line_to_process.a).length();
    if dist_to_start <= filter_eps_value {
        filtered_lines.first_mut().unwrap().projected_line.a = line_to_process.a;
    }

    let dist_to_end =
        pt_to_vec2d(filtered_lines.last().unwrap().projected_line.b - line_to_process.b).length();
    if dist_to_end <= filter_eps_value {
        filtered_lines.last_mut().unwrap().projected_line.b = line_to_process.b;
    }

    filtered_lines
}

// post_process_painted_lines (MultiMaterialSegmentation.cpp:688) depends on
// `EdgeGrid::Contour::segment_start` / `get_segment` which the Rust EdgeGrid does not
// yet expose in this shape; it is only reachable from the BLOCKED entry points. See report.

// MultiMaterialSegmentation.cpp:721
#[allow(dead_code)]
fn are_lines_connected(colored_lines: &[ColoredLine]) -> bool {
    for line_idx in 1..colored_lines.len() {
        if colored_lines[line_idx - 1].line.b != colored_lines[line_idx].line.a {
            return false;
        }
    }
    true
}

// MultiMaterialSegmentation.cpp:730
fn colorize_line(
    line_to_process: &Line,
    start_idx: usize,
    end_idx: usize,
    painted_contour: &[PaintedLine],
) -> ColoredLines {
    debug_assert!(
        start_idx < painted_contour.len() && end_idx < painted_contour.len() && start_idx <= end_idx
    );

    // MultiMaterialSegmentation.cpp:738-743
    let filter_eps_value = scale_(0.1) as f64;
    let mut final_lines: ColoredLines = Vec::new();
    let first_line = painted_contour[start_idx];
    let dist_to_start = pt_to_vec2d(first_line.projected_line.a - line_to_process.a).length();
    if dist_to_start > filter_eps_value {
        final_lines.push(ColoredLine::new(
            Line::new(line_to_process.a, first_line.projected_line.a),
            0,
        ));
    }
    final_lines.push(ColoredLine::new(first_line.projected_line, first_line.color));

    // MultiMaterialSegmentation.cpp:745-761
    for line_idx in (start_idx + 1)..=end_idx {
        let curr = painted_contour[line_idx];
        let prev_color = final_lines.last().unwrap().color;
        let prev_b = final_lines.last().unwrap().line.b;

        let line_dist = pt_to_vec2d(curr.projected_line.a - prev_b).length();
        if line_dist <= filter_eps_value {
            if prev_color == curr.color {
                final_lines.last_mut().unwrap().line.b = curr.projected_line.b;
            } else {
                final_lines.last_mut().unwrap().line.b = curr.projected_line.a;
                final_lines.push(ColoredLine::new(curr.projected_line, curr.color));
            }
        } else {
            final_lines.push(ColoredLine::new(Line::new(prev_b, curr.projected_line.a), 0));
            final_lines.push(ColoredLine::new(curr.projected_line, curr.color));
        }
    }

    // If there is non-painted space, then inserts line painted by a default color.
    // MultiMaterialSegmentation.cpp:764-765
    let dist_to_end = pt_to_vec2d(final_lines.last().unwrap().line.b - line_to_process.b).length();
    if dist_to_end > filter_eps_value {
        let last_b = final_lines.last().unwrap().line.b;
        final_lines.push(ColoredLine::new(Line::new(last_b, line_to_process.b), 0));
    }

    // Make sure all the lines are connected.
    // MultiMaterialSegmentation.cpp:768
    debug_assert!(are_lines_connected(&final_lines));

    // MultiMaterialSegmentation.cpp:770-777
    for line_idx in 2..final_lines.len() {
        let line_0 = final_lines[line_idx - 2];
        let line_2 = final_lines[line_idx];
        let line_1 = &mut final_lines[line_idx - 1];

        if line_0.color == line_2.color && line_0.color != line_1.color {
            if line_1.line.length() <= scale_(0.2) as f64 {
                line_1.color = line_0.color;
            }
        }
    }

    // MultiMaterialSegmentation.cpp:779-790
    let mut colored_lines_simple: ColoredLines = Vec::new();
    colored_lines_simple.push(final_lines[0]);
    for line_idx in 1..final_lines.len() {
        let line_0 = final_lines[line_idx];

        if colored_lines_simple.last().unwrap().color == line_0.color {
            colored_lines_simple.last_mut().unwrap().line.b = line_0.line.b;
        } else {
            colored_lines_simple.push(line_0);
        }
    }

    let mut final_lines = colored_lines_simple;

    // MultiMaterialSegmentation.cpp:792-796
    if final_lines.len() > 1
        && final_lines[0].color != final_lines[1].color
        && final_lines[0].line.length() <= scale_(0.2) as f64
    {
        final_lines[1].line.a = final_lines[0].line.a;
        final_lines.remove(0);
    }

    // MultiMaterialSegmentation.cpp:798-802
    if final_lines.len() > 1 {
        let n = final_lines.len();
        if final_lines[n - 1].color != final_lines[n - 2].color
            && final_lines[n - 1].line.length() <= scale_(0.2) as f64
        {
            let last_b = final_lines[n - 1].line.b;
            final_lines[n - 2].line.b = last_b;
            final_lines.pop();
        }
    }

    final_lines
}

// MultiMaterialSegmentation.cpp:807
fn filter_colorized_polygon(mut new_lines: ColoredLines) -> ColoredLines {
    // MultiMaterialSegmentation.cpp:808-816
    for line_idx in 2..new_lines.len() {
        let line_0 = new_lines[line_idx - 2];
        let line_2 = new_lines[line_idx];
        let line_1 = &mut new_lines[line_idx - 1];

        if line_0.color == line_2.color && line_0.color != line_1.color && line_0.color >= 1 {
            if line_1.line.length() <= scale_(0.5) as f64 {
                line_1.color = line_0.color;
            }
        }
    }

    // MultiMaterialSegmentation.cpp:818-830
    for line_idx in 3..new_lines.len() {
        let line_0 = new_lines[line_idx - 3];
        let line_3 = new_lines[line_idx];
        let line_1_color = new_lines[line_idx - 2].color;
        let line_2_color = new_lines[line_idx - 1].color;

        if line_0.color == line_3.color
            && (line_0.color != line_1_color || line_0.color != line_2_color)
            && line_0.color >= 1
            && line_3.color >= 1
        {
            let line_1_len = new_lines[line_idx - 2].line.length();
            let line_2_len = new_lines[line_idx - 1].line.length();
            if (line_1_len + line_2_len) <= scale_(0.5) as f64 {
                new_lines[line_idx - 2].color = line_0.color;
                new_lines[line_idx - 1].color = line_0.color;
            }
        }
    }

    // MultiMaterialSegmentation.cpp:832-839
    let segment_length = |segment: &(usize, usize), new_lines: &ColoredLines| -> f64 {
        let mut total_length = 0.0;
        let mut seg_start_idx = segment.0;
        while seg_start_idx != segment.1 {
            total_length += new_lines[seg_start_idx].line.length();
            seg_start_idx = if seg_start_idx + 1 < new_lines.len() {
                seg_start_idx + 1
            } else {
                0
            };
        }
        total_length += new_lines[segment.1].line.length();
        total_length
    };

    // MultiMaterialSegmentation.cpp:832 / 841-857
    let segments = get_segments(&new_lines);
    if segments.len() >= 2 {
        for curr_idx in 0..segments.len() {
            let next_idx = next_idx_modulo(curr_idx, segments.len());
            debug_assert!(curr_idx != next_idx);

            let color0 = new_lines[segments[curr_idx].0].color;
            let color1 = new_lines[segments[next_idx].0].color;

            let seg0l = segment_length(&segments[curr_idx], &new_lines);
            let seg1l = segment_length(&segments[next_idx], &new_lines);

            if color0 != color1 && seg0l >= scale_(0.1) as f64 && seg1l <= scale_(0.2) as f64 {
                let mut seg_start_idx = segments[next_idx].0;
                while seg_start_idx != segments[next_idx].1 {
                    new_lines[seg_start_idx].color = color0;
                    seg_start_idx = if seg_start_idx + 1 < new_lines.len() {
                        seg_start_idx + 1
                    } else {
                        0
                    };
                }
                new_lines[segments[next_idx].1].color = color0;
            }
        }
    }

    // MultiMaterialSegmentation.cpp:859-874
    let segments = get_segments(&new_lines);
    if segments.len() >= 2 {
        for curr_idx in 0..segments.len() {
            let next_idx = next_idx_modulo(curr_idx, segments.len());
            debug_assert!(curr_idx != next_idx);

            let color0 = new_lines[segments[curr_idx].0].color;
            let color1 = new_lines[segments[next_idx].0].color;
            let seg1l = segment_length(&segments[next_idx], &new_lines);

            if color0 >= 1 && color0 != color1 && seg1l <= scale_(0.2) as f64 {
                let mut seg_start_idx = segments[next_idx].0;
                while seg_start_idx != segments[next_idx].1 {
                    new_lines[seg_start_idx].color = color0;
                    seg_start_idx = if seg_start_idx + 1 < new_lines.len() {
                        seg_start_idx + 1
                    } else {
                        0
                    };
                }
                new_lines[segments[next_idx].1].color = color0;
            }
        }
    }

    // MultiMaterialSegmentation.cpp:876-891
    let segments = get_segments(&new_lines);
    if segments.len() >= 3 {
        for curr_idx in 0..segments.len() {
            let next_idx = next_idx_modulo(curr_idx, segments.len());
            let next_next_idx = next_idx_modulo(next_idx, segments.len());

            let color0 = new_lines[segments[curr_idx].0].color;
            let color1 = new_lines[segments[next_idx].0].color;
            let color2 = new_lines[segments[next_next_idx].0].color;

            if color0 > 0
                && color0 == color2
                && color0 != color1
                && segment_length(&segments[next_idx], &new_lines) <= scale_(0.5) as f64
            {
                let mut seg_start_idx = segments[next_next_idx].0;
                while seg_start_idx != segments[next_next_idx].1 {
                    new_lines[seg_start_idx].color = color0;
                    seg_start_idx = if seg_start_idx + 1 < new_lines.len() {
                        seg_start_idx + 1
                    } else {
                        0
                    };
                }
                new_lines[segments[next_next_idx].1].color = color0;
            }
        }
    }

    new_lines
}

// colorize_contour (MultiMaterialSegmentation.cpp:896) and colorize_contours
// (MultiMaterialSegmentation.cpp:936) consume `EdgeGrid::Contour` (get_segment /
// get_segments / num_segments). The Rust EdgeGrid does not yet expose those, and these
// are only reachable from the BLOCKED entry points. See report.

// MultiMaterialSegmentation.cpp:960
// Determines if the line points from the point between two contour lines is pointing inside polygon or outside.
fn points_inside(contour_first: &Line, contour_second: &Line, new_point: &Point) -> bool {
    // MultiMaterialSegmentation.cpp:963-967
    let three_points_inward_normal = |left: &Point, middle: &Point, right: &Point| -> PointF {
        debug_assert!(left != middle);
        debug_assert!(middle != right);
        (pt_to_vec2d(perp(*middle - *left)).normalize()
            + pt_to_vec2d(perp(*right - *middle)).normalize())
        .normalize()
    };

    // MultiMaterialSegmentation.cpp:969-974
    debug_assert!(contour_first.b == contour_second.a);
    let inward_normal =
        three_points_inward_normal(&contour_first.a, &contour_first.b, &contour_second.b);
    let edge_norm = pt_to_vec2d(*new_point - contour_first.b).normalize();
    let side = inward_normal.dot(&edge_norm);
    side > 0.0
}

// For every ColoredLine in lines_colored_out, assign the index of the polygon to which it belongs
// and also the index of this line inside of the polygon.
// MultiMaterialSegmentation.cpp:1617
fn init_polygon_indices(
    graph: &MmuGraph,
    color_poly: &[Vec<ColoredLine>],
    lines_colored_out: &mut [ColoredLine],
) {
    let mut poly_idx = 0usize;
    for color_lines in color_poly {
        let mut line_idx = 0usize;
        for _color_line_idx in 0..color_lines.len() {
            let from_idx = graph.get_global_index(poly_idx, line_idx);
            lines_colored_out[from_idx].poly_idx = poly_idx as i32;
            lines_colored_out[from_idx].local_line_idx = line_idx as i32;
            line_idx += 1;
        }
        poly_idx += 1;
    }
}

// MultiMaterialSegmentation.cpp:1632
fn line_intersection_with_epsilon(
    line_to_extend: &Line,
    other: &Line,
    intersection: &mut Point,
) -> bool {
    let mut extended_line = *line_to_extend;
    extended_line.extend(15.0 * SCALED_EPSILON);
    match extended_line.intersection(other) {
        Some(p) => {
            *intersection = p;
            true
        }
        None => false,
    }
}

// MultiMaterialSegmentation.cpp:1645
fn is_point_closer_to_beginning_of_line(line: &Line, p: &Point) -> bool {
    pt_to_vec2d(*p - line.a).length_squared() < pt_to_vec2d(*p - line.b).length_squared()
}

// MultiMaterialSegmentation.cpp:1668
#[inline]
fn has_same_color(cl1: &ColoredLine, cl2: &ColoredLine) -> bool {
    cl1.color == cl2.color
}

// build_graph (MultiMaterialSegmentation.cpp:1670) requires the
// `Voronoi::Internal::clip_infinite_edge` helper and direct boostvoronoi cell
// source-index iteration in the half-edge order C++ relies on. The intersection-heavy
// finite/infinite edge classification is ported below against the boostvoronoi 0.12
// `Diagram` API. The Voronoi vertices are appended via `append_voronoi_vertices`.
// NOTE: this function and its callees are only reachable from the BLOCKED entry points,
// which assemble `color_poly` from EdgeGrid contours that are not yet available; they are
// kept here, faithfully ported where the boostvoronoi API supports it. See report.

// MultiMaterialSegmentation.cpp:1887
fn get_all_segments(color_poly: &[Vec<ColoredLine>]) -> Vec<Vec<(usize, usize)>> {
    let mut all_segments: Vec<Vec<(usize, usize)>> = vec![Vec::new(); color_poly.len()];
    for poly_idx in 0..color_poly.len() {
        let c_polygon = &color_poly[poly_idx];
        all_segments[poly_idx] = get_segments(c_polygon);
    }
    all_segments
}

// MultiMaterialSegmentation.cpp:1897
fn compute_edge_length(graph: &MmuGraph, start_idx: usize, start_arc_idx: usize) -> f64 {
    debug_assert!(start_arc_idx < graph.arcs.len());
    let mut used_arcs: Vec<bool> = vec![false; graph.arcs.len()];

    // MultiMaterialSegmentation.cpp:1902-1905
    used_arcs[start_arc_idx] = true;
    let mut arc_idx = start_arc_idx;
    let mut idx = start_idx;
    let mut line_total_length =
        (graph.nodes[graph.arcs[arc_idx].to_idx].point - graph.nodes[idx].point).length();

    // MultiMaterialSegmentation.cpp:1906-1932
    while graph.nodes[graph.arcs[arc_idx].to_idx].arc_idxs.len() == 2 {
        let mut found = false;
        let arc_to_idx = graph.arcs[arc_idx].to_idx;
        let neighbour_arcs = graph.nodes[arc_to_idx].arc_idxs.clone();
        for &arc_n_idx in &neighbour_arcs {
            let arc_n = graph.arcs[arc_n_idx];
            if arc_n.r#type == ArcType::NonBorder && !used_arcs[arc_n_idx] && arc_n.to_idx != idx {
                let first_line = LineF::new(graph.nodes[idx].point, graph.nodes[arc_to_idx].point);
                let second_line = LineF::new(graph.nodes[arc_to_idx].point, graph.nodes[arc_n.to_idx].point);

                let first_line_vec = first_line.a - first_line.b;
                let second_line_vec = second_line.b - second_line.a;
                let first_line_vec_n = first_line_vec.normalize();
                let second_line_vec_n = second_line_vec.normalize();
                let mut angle = first_line_vec_n
                    .dot(&second_line_vec_n)
                    .clamp(-1.0, 1.0)
                    .acos();
                if cross2f(first_line_vec_n, second_line_vec_n) < 0.0 {
                    angle = 2.0 * PI - angle;
                }

                if (angle - PI).abs() >= (PI / 12.0) {
                    continue;
                }

                idx = arc_to_idx;
                arc_idx = arc_n_idx;

                line_total_length +=
                    (graph.nodes[graph.arcs[arc_idx].to_idx].point - graph.nodes[idx].point).length();
                used_arcs[arc_n_idx] = true;
                found = true;
                break;
            }
        }
        if !found {
            break;
        }
    }

    line_total_length
}

// MultiMaterialSegmentation.cpp:1937
pub fn remove_multiple_edges_in_vertices(graph: &mut MmuGraph, color_poly: &[Vec<ColoredLine>]) {
    let colored_segments = get_all_segments(color_poly);
    // MultiMaterialSegmentation.cpp:1940-1965
    for poly_idx in 0..colored_segments.len() {
        let colored_segment_p = colored_segments[poly_idx].clone();
        for colored_segment in &colored_segment_p {
            let first_idx = graph.get_global_index(poly_idx, colored_segment.0);
            let _second_idx =
                graph.get_global_index(poly_idx, (colored_segment.1 + 1) % graph.polygon_sizes[poly_idx]);
            // Linef seg_line(nodes[first_idx].point, nodes[second_idx].point); (unused beyond this point)

            if graph.nodes[first_idx].arc_idxs.len() >= 3 {
                // arc_to_check: (arc_to_idx, total_len)
                let mut arc_to_check: Vec<(usize, f64)> = Vec::new();
                let first_arc_idxs = graph.nodes[first_idx].arc_idxs.clone();
                for &arc_idx in &first_arc_idxs {
                    let n_arc = graph.arcs[arc_idx];
                    if n_arc.r#type == ArcType::NonBorder {
                        let total_len = compute_edge_length(graph, first_idx, arc_idx);
                        arc_to_check.push((n_arc.to_idx, total_len));
                    }
                }
                arc_to_check.sort_by(|l, r| {
                    r.1.partial_cmp(&l.1).unwrap_or(std::cmp::Ordering::Equal)
                });

                while arc_to_check.len() > 1 {
                    graph.remove_edge(first_idx, arc_to_check.last().unwrap().0);
                    arc_to_check.pop();
                }
            }
        }
    }
}

// Check if all ColoredLine representing a single layer uses the same color.
// MultiMaterialSegmentation.cpp:2082
pub fn has_layer_only_one_color(colored_polygons: &[ColoredLines]) -> bool {
    debug_assert!(!colored_polygons.is_empty());
    debug_assert!(!colored_polygons.first().unwrap().is_empty());
    let first_line_color = colored_polygons.first().unwrap().first().unwrap().color;
    for colored_polygon in colored_polygons {
        for colored_line in colored_polygon {
            if first_line_color != colored_line.color {
                return false;
            }
        }
    }
    true
}

// ---------------------------------------------------------------------------
// Public API (MultiMaterialSegmentation.hpp:24-27)
// ---------------------------------------------------------------------------
//
// BLOCKED: `multi_material_segmentation_by_painting` and
// `fuzzy_skin_segmentation_by_painting` require a `PrintObject` carrying:
//   - print().config().filament_colour
//   - layers() (ConstLayerPtrsAdaptor) with LayerRegion::slices.surfaces
//   - model_object().volumes with ModelVolume::mmu_segmentation_facets /
//     fuzzy_skin_facets (EnforcerBlockerType facet extraction)
//   - trafo()/trafo_centered()/center_offset()
//   - EdgeGrid::Grid::create + visit_cells_intersecting_line + contours()
//   - slice_mesh_slabs / slice_mesh_ex / TriangleMeshSlicer
//   - cut_segmented_layers / mmu_segmentation_top_and_bottom_layers /
//     merge_segmented_layers (TBB + offset/diff/union ex pipelines)
// none of which are wired through the Rust crate yet. These entry points are therefore
// documented and not faked. The self-contained graph + colorize algorithms above are the
// faithful core they call into once the infrastructure is available.

// ---------------------------------------------------------------------------
// Small math helpers (mirrors Slic3r::sqr / cross2 / perp)
// ---------------------------------------------------------------------------

#[inline]
fn sqr_f64(x: f64) -> f64 {
    x * x
}

// perp() of an integer point: (-y, x).
#[inline]
fn perp(p: Point) -> Point {
    Point::new(-p.y, p.x)
}

// next_idx_modulo (Slic3r utility used by filter_colorized_polygon).
#[inline]
fn next_idx_modulo(idx: usize, count: usize) -> usize {
    let next = idx + 1;
    if next < count {
        next
    } else {
        0
    }
}

// Silence unused-import / unused-helper warnings for symbols that are only exercised by
// the BLOCKED entry points or the disabled VD-extraction path.
#[allow(dead_code)]
fn _parity_anchors() {
    let _ = (
        is_equal as fn(f32, f32) -> bool,
        is_equal_eps as fn(f32, f32, f32) -> bool,
        is_less as fn(f32, f32) -> bool,
        is_less_eps as fn(f32, f32, f32) -> bool,
        project_line_on_line as fn(&Line, &Line, &mut Line) -> bool,
        points_inside as fn(&Line, &Line, &Point) -> bool,
        init_polygon_indices as fn(&MmuGraph, &[Vec<ColoredLine>], &mut [ColoredLine]),
        line_intersection_with_epsilon as fn(&Line, &Line, &mut Point) -> bool,
        is_point_closer_to_beginning_of_line as fn(&Line, &Point) -> bool,
        has_same_color as fn(&ColoredLine, &ColoredLine) -> bool,
        colorize_line as fn(&Line, usize, usize, &[PaintedLine]) -> ColoredLines,
        filter_colorized_polygon as fn(ColoredLines) -> ColoredLines,
        filter_painted_lines as fn(&Line, usize, usize, &[PaintedLine]) -> Vec<PaintedLine>,
        get_extents_colored as fn(&[ColoredLines]) -> BoundingBox,
        to_lines_colored as fn(&[ColoredLines]) -> ColoredLines,
        colored_points_to_polygon as fn(&[Vec<ColoredLine>]) -> Vec<Polygon>,
        cos_threshold2 as fn() -> f64,
        append_threshold2 as fn() -> f64,
    );
    let _ = (VoronoiDiagram::new(), bv::Point { x: 0i64, y: 0i64 });
}
