//! Faithful 1:1 port of BambuStudio `src/libslic3r/MultiMaterialSegmentation.cpp`.
//!
//! Returns MMU segmentation based on painting in the MMU segmentation gizmo, and
//! the fuzzy-skin variant.
//!
//! coord_t -> i64, coordf_t -> f64. Voronoi access goes through the `boostvoronoi`
//! crate (same backend as `geometry::voronoi_diagram`).
//!
//! PORTING STATUS (see report): the self-contained graph + painted-line + colorize
//! algorithms are ported line-by-line, including the EdgeGrid-driven
//! `PaintedLineVisitor` / `post_process_painted_lines` / `colorize_contour(s)` and the
//! Clipper-driven `cut_segmented_layers` / `merge_segmented_layers`. The top-level
//! entry points `multi_material_segmentation_by_painting` /
//! `fuzzy_skin_segmentation_by_painting`, plus `mmu_segmentation_top_and_bottom_layers`,
//! remain BLOCKED on ModelVolume::mmu_segmentation_facets / fuzzy_skin_facets (facet
//! annotations are not stored on the Rust ModelVolume) and `slice_mesh_slabs` (not
//! ported); those are documented as BLOCKED below and not faked.

use boostvoronoi::diagram as bv_diagram;
use boostvoronoi::prelude as bv;

use crate::clipper_utils::{
    closing, difference, offset_expolygons, union_ex, union_polygons_ex, OffsetJoinType,
};
use crate::edge_grid::{Contour, EdgeGrid};
use crate::geometry::voronoi_diagram::VoronoiDiagram;
use crate::geometry::{
    BoundingBox, BoundingBoxF, ExPolygon, ExPolygons, Line, LineF, Point, PointF, Polygon,
};
use crate::libslic3r::{EPSILON, SCALED_EPSILON};
use crate::normal_utils::{indexed_triangle_set, Vec3f};
use crate::{scale, Coord, SCALING_FACTOR};

use std::collections::HashMap;
use std::collections::HashSet;
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
    #[inline]
    fn is_vertex_on_contour(&self, vertex_color: bv_diagram::ColorType) -> bool {
        (vertex_color as usize) < self.all_border_points
    }

    // MultiMaterialSegmentation.hpp:192 — is_edge_attach_to_contour.
    // `edge_iterator->vertex0/vertex1` colors carry the graph node index that
    // `append_voronoi_vertices` assigned; a "contour vertex" is any node index below
    // `all_border_points`. We take the two vertex colors directly instead of the edge
    // iterator so the caller reads them once via the boostvoronoi color API.
    #[inline]
    fn is_edge_attach_to_contour(
        &self,
        v0_color: bv_diagram::ColorType,
        v1_color: bv_diagram::ColorType,
    ) -> bool {
        self.is_vertex_on_contour(v0_color) || self.is_vertex_on_contour(v1_color)
    }

    // MultiMaterialSegmentation.hpp:197 — is_edge_connecting_two_contour_vertices.
    #[inline]
    fn is_edge_connecting_two_contour_vertices(
        &self,
        v0_color: bv_diagram::ColorType,
        v1_color: bv_diagram::ColorType,
    ) -> bool {
        self.is_vertex_on_contour(v0_color) && self.is_vertex_on_contour(v1_color)
    }

    // MultiMaterialSegmentation.hpp:109 — `append_edge(from, to, color = -1, NON_BORDER)`.
    // C++ relies on the default arguments in build_graph's many two-argument calls; Rust
    // has no default arguments, so this thin wrapper reproduces the C++ default.
    #[inline]
    pub fn append_edge_default(&mut self, from_idx: usize, to_idx: usize) {
        self.append_edge(from_idx, to_idx, -1, ArcType::NonBorder);
    }

    // MultiMaterialSegmentation.cpp:202-281 — append_voronoi_vertices.
    // All Voronoi vertices are post-processed to merge very close vertices to a single
    // node (which eliminates issues with intersecting edges). Voronoi vertices outside
    // the bounding box of the input polygons are left unassigned (marked with the
    // `VD_VERTEX_UNSET` sentinel = C++'s `vertex.color(-1)`).
    //
    // C++ takes `const Geometry::VoronoiDiagram &vd`; the boostvoronoi crate stores the
    // graph node index in the vertex `color` (via `vertex_set_color`), so we need `&mut`.
    pub fn append_voronoi_vertices(
        &mut self,
        diagram: &mut bv::Diagram,
        color_poly_tmp: &[Polygon],
        mut bbox: BoundingBox,
    ) {
        // MultiMaterialSegmentation.cpp:206 — bbox.offset(SCALED_EPSILON).
        bbox.expand(SCALED_EPSILON as Coord);

        // MultiMaterialSegmentation.cpp:232-235 — the two closest-point lookups plus the
        // seeding of every contour point into `closest_contour_point`.
        let mut closest_voronoi_point = CPointLookup::new(SCALED_EPSILON as Coord);
        let mut closest_contour_point = CPointLookup::new(3 * SCALED_EPSILON as Coord);
        for (contour_idx, polygon) in color_poly_tmp.iter().enumerate() {
            for (point_idx, pt) in polygon.points.iter().enumerate() {
                closest_contour_point.insert(CPoint::with_contour(
                    PointF::new(pt.x as f64, pt.y as f64),
                    contour_idx,
                    point_idx,
                ));
            }
        }

        // MultiMaterialSegmentation.cpp:237 — iterate all Voronoi vertices. We collect the
        // ids first so the mutable `vertex_set_color` below does not alias the read borrow.
        let vertex_ids: Vec<bv::VertexIndex> =
            diagram.vertices().iter().map(|v| v.get_id()).collect();

        for vertex_id in vertex_ids {
            // MultiMaterialSegmentation.cpp:238-240.
            let _ = diagram.vertex_set_color(vertex_id, VD_VERTEX_UNSET);
            let (vx, vy) = match diagram.vertex(vertex_id) {
                Ok(v) => (v.x(), v.y()),
                Err(_) => continue,
            };
            let vertex_point_double = PointF::new(vx, vy);
            // mk_point(vertex) truncates toward zero (coord_t(vertex.x())).
            let vertex_point = vec2d_to_pt(vertex_point_double);

            // MultiMaterialSegmentation.cpp:242-243 — the two contour endpoints of the
            // cells incident to this vertex (via its incident edge and that edge's twin).
            let incident_edge = match diagram.vertex_get_incident_edge(vertex_id) {
                Some(e) => e,
                None => continue,
            };
            let inc_cell_src = match edge_cell_source_index(diagram, incident_edge) {
                Some(s) => s,
                None => continue,
            };
            let inc_twin = match diagram.edge_get_twin(incident_edge) {
                Ok(t) => t,
                Err(_) => continue,
            };
            let inc_twin_cell_src = match edge_cell_source_index(diagram, inc_twin) {
                Some(s) => s,
                None => continue,
            };
            let first_point_double = self.nodes[self.get_border_arc(inc_cell_src).from_idx].point;
            let second_point_double =
                self.nodes[self.get_border_arc(inc_twin_cell_src).from_idx].point;

            // MultiMaterialSegmentation.cpp:245-252.
            if vertex_equal_to_point(vx, vy, first_point_double) {
                let _ = diagram
                    .vertex_set_color(vertex_id, self.get_border_arc(inc_cell_src).from_idx as u32);
            } else if vertex_equal_to_point(vx, vy, second_point_double) {
                let _ = diagram.vertex_set_color(
                    vertex_id,
                    self.get_border_arc(inc_twin_cell_src).from_idx as u32,
                );
            } else if bbox.contains_point(&vertex_point) {
                // MultiMaterialSegmentation.cpp:254-255 — snap to a contour point.
                let (contour_pt, c_dist_sqr) = closest_contour_point.find(vertex_point);
                if let Some(contour_pt) = contour_pt {
                    if c_dist_sqr < sqr_f64(3.0 * SCALED_EPSILON) {
                        let _ = diagram.vertex_set_color(
                            vertex_id,
                            self.get_global_index(contour_pt.contour_idx, contour_pt.point_idx)
                                as u32,
                        );
                        continue;
                    }
                }

                // MultiMaterialSegmentation.cpp:256-259 — otherwise a fresh Voronoi node,
                // unless a previous node is within SCALED_EPSILON/10 of it.
                let (voronoi_pt, v_dist_sqr) = closest_voronoi_point.find(vertex_point);
                if voronoi_pt.is_none() || v_dist_sqr >= sqr_f64(SCALED_EPSILON / 10.0) {
                    let new_idx = self.nodes_count();
                    closest_voronoi_point.insert(CPoint::new(vertex_point_double, new_idx));
                    let _ = diagram.vertex_set_color(vertex_id, new_idx as u32);
                    self.nodes.push(Node {
                        point: vertex_point_double,
                        arc_idxs: Vec::new(),
                    });
                } else {
                    // MultiMaterialSegmentation.cpp:260-278 — Boost sometimes emits two very
                    // close points instead of one; merge into an EPSILON-equal existing one.
                    let all_close = closest_voronoi_point.find_all(vertex_point);
                    let mut merge_to_point: i64 = -1;
                    for c_point in &all_close {
                        // vertex_point_double / point_double are already scaled doubles
                        // (Vec2d), so squaredNorm applies directly (no pt_to_vec2d).
                        if (vertex_point_double - c_point.0.point_double).length_squared()
                            <= sqr_f64(EPSILON)
                        {
                            merge_to_point = c_point.0.point_idx as i64;
                            break;
                        }
                    }

                    if merge_to_point != -1 {
                        let _ = diagram.vertex_set_color(vertex_id, merge_to_point as u32);
                    } else {
                        let new_idx = self.nodes_count();
                        closest_voronoi_point.insert(CPoint::new(vertex_point_double, new_idx));
                        let _ = diagram.vertex_set_color(vertex_id, new_idx as u32);
                        self.nodes.push(Node {
                            point: vertex_point_double,
                            arc_idxs: Vec::new(),
                        });
                    }
                }
            }
        }
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

// Line.hpp:42-69 — line_alg::distance_to_squared (squared distance to the closest point
// of the segment), computed in scaled doubles exactly as the Eigen code does. (The
// crate-level `Line::distance_to_squared` rounds the projection to an integer Point
// first, which is not what the C++ does here.)
#[allow(dead_code)]
fn line_distance_to_squared(line: &Line, point: &Point) -> f64 {
    // Line.hpp:45-47
    let v = pt_to_vec2d(line.b - line.a);
    let va = pt_to_vec2d(*point - line.a);
    let l2 = v.length_squared(); // avoid a sqrt
    if l2 == 0.0 {
        // a == b case
        // Line.hpp:48-52
        return va.length_squared();
    }
    // Consider the line extending the segment, parameterized as a + t (b - a).
    // We find projection of this point onto the line.
    // It falls where t = [(this-a) . (b-a)] / |b-a|^2
    // Line.hpp:56
    let t = va.dot(&v) / l2;
    if t <= 0.0 {
        // beyond the 'a' end of the segment
        // Line.hpp:57-60
        va.length_squared()
    } else if t >= 1.0 {
        // beyond the 'b' end of the segment
        // Line.hpp:61-65
        pt_to_vec2d(*point - line.b).length_squared()
    } else {
        // Line.hpp:67-68
        (v * t - va).length_squared()
    }
}

// MultiMaterialSegmentation.cpp:524
// (The C++ visitor also carries a `std::mutex` guarding `painted_lines` for tbb; the
// Rust port runs serially, so `painted_lines` is a plain mutable borrow instead.)
#[allow(dead_code)]
pub struct PaintedLineVisitor<'a> {
    // MultiMaterialSegmentation.cpp:579-584
    pub grid: &'a EdgeGrid,
    pub painted_lines: &'a mut Vec<PaintedLine>,
    pub line_to_test: Line,
    pub painted_lines_set: HashSet<(usize, usize)>,
    pub color: i32,
}

#[allow(dead_code)]
impl<'a> PaintedLineVisitor<'a> {
    // MultiMaterialSegmentation.cpp:526-529
    pub fn new(grid: &'a EdgeGrid, painted_lines: &'a mut Vec<PaintedLine>, reserve: usize) -> Self {
        Self {
            grid,
            painted_lines,
            line_to_test: Line::default(),
            painted_lines_set: HashSet::with_capacity(reserve),
            color: -1,
        }
    }

    // MultiMaterialSegmentation.cpp:531
    pub fn reset(&mut self) {
        self.painted_lines_set.clear();
    }

    // MultiMaterialSegmentation.cpp:533 — bool operator()(coord_t iy, coord_t ix)
    pub fn visit(&mut self, iy: usize, ix: usize) -> bool {
        // Called with a row and column of the grid cell, which is intersected by a line.
        // MultiMaterialSegmentation.cpp:536-539
        let grid = self.grid;
        let cell_data_range = grid.cell_data_range_at(iy, ix);
        let v1 = pt_to_vec2d(self.line_to_test.vector());
        let v1_sqr_norm = v1.length_squared();
        let heuristic_thr_part = self.line_to_test.length() + append_threshold();
        for it_contour_and_segment in cell_data_range {
            // MultiMaterialSegmentation.cpp:541-543
            let grid_line = grid.segment(*it_contour_and_segment);
            let v2 = pt_to_vec2d(grid_line.vector());
            let heuristic_thr_sqr = sqr_f64(heuristic_thr_part + grid_line.length());

            // An inexpensive heuristic to test whether line_to_test and grid_line can be somewhere close enough to each other.
            // This helps filter out cases when the following expensive calculations are useless.
            // MultiMaterialSegmentation.cpp:545-551
            if pt_to_vec2d(grid_line.a - self.line_to_test.a).length_squared() > heuristic_thr_sqr
                || pt_to_vec2d(grid_line.b - self.line_to_test.a).length_squared() > heuristic_thr_sqr
                || pt_to_vec2d(grid_line.a - self.line_to_test.b).length_squared() > heuristic_thr_sqr
                || pt_to_vec2d(grid_line.b - self.line_to_test.b).length_squared() > heuristic_thr_sqr
            {
                continue;
            }

            // When lines have too different length, it is necessary to normalize them
            // MultiMaterialSegmentation.cpp:553-555
            if sqr_f64(v1.dot(&v2)) > cos_threshold2() * v1_sqr_norm * v2.length_squared() {
                // The two vectors are nearly collinear (their mutual angle is lower than 30 degrees)
                if !self.painted_lines_set.contains(it_contour_and_segment) {
                    // MultiMaterialSegmentation.cpp:557-560
                    if line_distance_to_squared(&grid_line, &self.line_to_test.a) < append_threshold2()
                        || line_distance_to_squared(&grid_line, &self.line_to_test.b) < append_threshold2()
                        || line_distance_to_squared(&self.line_to_test, &grid_line.a) < append_threshold2()
                        || line_distance_to_squared(&self.line_to_test, &grid_line.b) < append_threshold2()
                    {
                        // MultiMaterialSegmentation.cpp:561-562
                        let mut line_to_test_projected = Line::default();
                        project_line_on_line(&grid_line, &self.line_to_test, &mut line_to_test_projected);

                        // MultiMaterialSegmentation.cpp:564-565
                        if pt_to_vec2d(line_to_test_projected.a - grid_line.a).length_squared()
                            > pt_to_vec2d(line_to_test_projected.b - grid_line.a).length_squared()
                        {
                            line_to_test_projected.reverse_mut();
                        }

                        // MultiMaterialSegmentation.cpp:567-571
                        self.painted_lines_set.insert(*it_contour_and_segment);
                        self.painted_lines.push(PaintedLine {
                            contour_idx: it_contour_and_segment.0,
                            line_idx: it_contour_and_segment.1,
                            projected_line: line_to_test_projected,
                            color: self.color,
                        });
                    }
                }
            }
        }
        // Continue traversing the grid along the edge.
        // MultiMaterialSegmentation.cpp:576
        true
    }
}

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

// MultiMaterialSegmentation.cpp:688
// `painted_lines` is taken by value (the C++ takes `std::vector<PaintedLine> &&`).
#[allow(dead_code)]
fn post_process_painted_lines(
    contours: &[Contour],
    mut painted_lines: Vec<PaintedLine>,
) -> Vec<Vec<PaintedLine>> {
    // MultiMaterialSegmentation.cpp:690-691
    if painted_lines.is_empty() {
        return Vec::new();
    }

    // MultiMaterialSegmentation.cpp:693-703
    let comp = |first: &PaintedLine, second: &PaintedLine| -> bool {
        let first_start_p = *contours[first.contour_idx].segment_start(first.line_idx);
        first.contour_idx < second.contour_idx
            || (first.contour_idx == second.contour_idx
                && (first.line_idx < second.line_idx
                    || (first.line_idx == second.line_idx
                        && (pt_to_vec2d(first.projected_line.a - first_start_p).length_squared()
                            < pt_to_vec2d(second.projected_line.a - first_start_p).length_squared()
                            || (pt_to_vec2d(first.projected_line.a - first_start_p).length_squared()
                                == pt_to_vec2d(second.projected_line.a - first_start_p)
                                    .length_squared()
                                && pt_to_vec2d(first.projected_line.b - first.projected_line.a)
                                    .length_squared()
                                    < pt_to_vec2d(second.projected_line.b - second.projected_line.a)
                                        .length_squared())))))
    };
    // MultiMaterialSegmentation.cpp:704 — std::sort with the strict-weak-order comparator.
    painted_lines.sort_by(|a, b| {
        if comp(a, b) {
            std::cmp::Ordering::Less
        } else if comp(b, a) {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });

    // MultiMaterialSegmentation.cpp:706-716
    let mut filtered_painted_lines: Vec<Vec<PaintedLine>> = vec![Vec::new(); contours.len()];
    let mut prev_painted_line_idx = 0usize;
    for curr_painted_line_idx in 0..painted_lines.len() {
        let next_painted_line_idx = curr_painted_line_idx + 1;
        if next_painted_line_idx >= painted_lines.len()
            || painted_lines[curr_painted_line_idx].contour_idx
                != painted_lines[next_painted_line_idx].contour_idx
            || painted_lines[curr_painted_line_idx].line_idx
                != painted_lines[next_painted_line_idx].line_idx
        {
            // MultiMaterialSegmentation.cpp:711-713
            let start_line = &painted_lines[prev_painted_line_idx];
            let line_to_process = contours[start_line.contour_idx].segment(start_line.line_idx);
            let contour_idx = painted_lines[curr_painted_line_idx].contour_idx;
            let filtered = filter_painted_lines(
                &line_to_process,
                prev_painted_line_idx,
                curr_painted_line_idx,
                &painted_lines,
            );
            filtered_painted_lines[contour_idx].extend(filtered);
            prev_painted_line_idx = next_painted_line_idx;
        }
    }

    filtered_painted_lines
}

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

// MultiMaterialSegmentation.cpp:896
#[allow(dead_code)]
fn colorize_contour(contour: &Contour, painted_contour: &[PaintedLine]) -> ColoredLines {
    // MultiMaterialSegmentation.cpp:897
    debug_assert!(
        painted_contour.is_empty()
            || painted_contour
                .iter()
                .all(|p_line| painted_contour[0].contour_idx == p_line.contour_idx)
    );

    // MultiMaterialSegmentation.cpp:899-906
    let mut colorized_contour: ColoredLines = Vec::new();
    if painted_contour.is_empty() {
        // Appends contour with default color for lines before the first PaintedLine.
        colorized_contour.reserve(contour.num_segments());
        for line in contour.segments() {
            colorized_contour.push(ColoredLine::new(line, 0));
        }
        return colorized_contour;
    }

    // MultiMaterialSegmentation.cpp:908-910
    colorized_contour.reserve(contour.num_segments() + painted_contour.len());
    for idx in 0..painted_contour.first().unwrap().line_idx {
        colorized_contour.push(ColoredLine::new(contour.segment(idx), 0));
    }

    // MultiMaterialSegmentation.cpp:912-927
    let mut prev_painted_line_idx = 0usize;
    for curr_painted_line_idx in 0..painted_contour.len() {
        let next_painted_line_idx = curr_painted_line_idx + 1;
        if next_painted_line_idx >= painted_contour.len()
            || painted_contour[curr_painted_line_idx].line_idx
                != painted_contour[next_painted_line_idx].line_idx
        {
            // MultiMaterialSegmentation.cpp:916-917
            colorized_contour.extend(colorize_line(
                &contour.segment(painted_contour[prev_painted_line_idx].line_idx),
                prev_painted_line_idx,
                curr_painted_line_idx,
                painted_contour,
            ));

            // Appends contour with default color for lines between the current and the next PaintedLine.
            // MultiMaterialSegmentation.cpp:919-922
            if next_painted_line_idx < painted_contour.len() {
                for idx in (painted_contour[curr_painted_line_idx].line_idx + 1)
                    ..painted_contour[next_painted_line_idx].line_idx
                {
                    colorized_contour.push(ColoredLine::new(contour.segment(idx), 0));
                }
            }

            prev_painted_line_idx = next_painted_line_idx;
        }
    }

    // Appends contour with default color for lines after the last PaintedLine.
    // MultiMaterialSegmentation.cpp:929-931
    for idx in (painted_contour.last().unwrap().line_idx + 1)..contour.num_segments() {
        colorized_contour.push(ColoredLine::new(contour.segment(idx), 0));
    }

    debug_assert!(!colorized_contour.is_empty());
    // MultiMaterialSegmentation.cpp:934
    filter_colorized_polygon(colorized_contour)
}

// MultiMaterialSegmentation.cpp:937
#[allow(dead_code)]
fn colorize_contours(
    contours: &[Contour],
    painted_contours: &[Vec<PaintedLine>],
) -> Vec<ColoredLines> {
    // MultiMaterialSegmentation.cpp:939-944
    debug_assert!(contours.len() == painted_contours.len());
    let mut colorized_contours: Vec<ColoredLines> = vec![Vec::new(); contours.len()];
    for contour_idx in 0..painted_contours.len() {
        colorized_contours[contour_idx] =
            colorize_contour(&contours[contour_idx], &painted_contours[contour_idx]);
    }

    // MultiMaterialSegmentation.cpp:946-955
    let mut poly_idx = 0usize;
    for color_lines in &mut colorized_contours {
        let mut line_idx = 0usize;
        for color_line_idx in 0..color_lines.len() {
            color_lines[color_line_idx].poly_idx = poly_idx as i32;
            color_lines[color_line_idx].local_line_idx = line_idx as i32;
            line_idx += 1;
        }
        poly_idx += 1;
    }

    colorized_contours
}

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

// ---------------------------------------------------------------------------
// build_graph and its Voronoi helpers (MultiMaterialSegmentation.cpp:1650-1885)
// ---------------------------------------------------------------------------
//
// LANDED: `build_graph` and its callees (`append_voronoi_vertices`,
// `clip_finite_voronoi_edge`, `Voronoi::Internal::clip_infinite_edge`,
// `mark_processed`, `is_edge_attach_to_contour` /
// `is_edge_connecting_two_contour_vertices`) are now ported 1:1 against the
// boostvoronoi 0.12 `Diagram` API. C++ stores the graph node index in each Voronoi
// vertex's `color()` and a processed flag in each edge's `color()`; the crate's
// `vertex_set_color` / `edge_set_color` mirror both (the crate reserves the low 5
// color bits, so `(size_t)-1` round-trips to `VD_VERTEX_UNSET` below — still `>=
// nodes_count()`, preserving every "unassigned vertex" test).
//
// DEVIATION (VD construction): C++ calls `Voronoi::VD::construct_voronoi(..., true, ...)`,
// where the `true` requests `detect_known_issues` + `try_to_repair_degenerated_voronoi_diagram`
// (rotate-and-retry around Boost degeneracies). The boostvoronoi crate exposes no such
// repair, so we build the raw diagram directly — the same cross-cutting Voronoi
// primitive substitution documented in `geometry::medial_axis`. If the raw build fails,
// we return the contour-only graph rather than panicking.
//
// NOTE: still only reachable from the BLOCKED entry points
// (`multi_material_segmentation_by_painting`), which remain gated on ModelVolume facet
// annotations and `slice_mesh_slabs`.

// C++ `vertex.color(-1)` sentinel for "this Voronoi vertex was not assigned a graph
// node". The crate keeps custom color in the upper 27 bits, so `(size_t)-1` stored via
// `vertex_set_color` reads back as `(1<<27)-1`; every real node index is far below this,
// so the `color >= nodes_count()` / `is_vertex_on_contour` tests behave exactly as C++.
const VD_VERTEX_UNSET: bv::ColorType = (1u32 << 27) - 1;

// MultiMaterialSegmentation.cpp:208-224 — the `CPoint` value stored in
// `ClosestPointInRadiusLookup`. `point` is the grid key (mk_point-rounded), `point_double`
// the exact scaled-double coordinate, and `point_idx` / `contour_idx` the payload
// (graph node index for Voronoi points, or contour+point index for contour points).
#[derive(Clone, Copy, Debug)]
struct CPoint {
    point_double: PointF,
    point: Point,
    point_idx: usize,
    contour_idx: usize,
}

impl CPoint {
    // MultiMaterialSegmentation.cpp:211 — CPoint(point, contour_idx, point_idx).
    #[inline]
    fn with_contour(point_double: PointF, contour_idx: usize, point_idx: usize) -> Self {
        Self {
            point_double,
            point: mk_point_vec2d(point_double),
            point_idx,
            contour_idx,
        }
    }

    // MultiMaterialSegmentation.cpp:213 — CPoint(point, point_idx) (contour_idx = 0).
    #[inline]
    fn new(point_double: PointF, point_idx: usize) -> Self {
        Self {
            point_double,
            point: mk_point_vec2d(point_double),
            point_idx,
            contour_idx: 0,
        }
    }
}

// Point.hpp:378 — ClosestPointInRadiusLookup, specialized to `CPoint`/`CPointAccessor`.
// A spatial hash over grid cells sized ~2*search_radius; `find` returns the closest entry
// within `search_radius`, `find_all` every entry within it. Ported faithfully (only the
// `insert`/`find`/`find_all` used by `append_voronoi_vertices`; `erase` is not needed).
struct CPointLookup {
    map: HashMap<(Coord, Coord), Vec<CPoint>>,
    search_radius: Coord,
    grid_resolution: Coord,
    grid_log2: Coord,
}

impl CPointLookup {
    // Point.hpp:381-410 — constructor computing m_grid_log2 = ceil(log2(2*radius+4)).
    fn new(search_radius: Coord) -> Self {
        let gridres = 2 * search_radius + 4;
        let mut grid_resolution = gridres;
        let mut grid_log2: Coord = 0;
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
        let grid_resolution = 1 << grid_log2;
        Self {
            map: HashMap::new(),
            search_radius,
            grid_resolution,
            grid_log2,
        }
    }

    // Point.hpp:412-416 — insert value keyed by (pt >> grid_log2).
    fn insert(&mut self, value: CPoint) {
        let key = (value.point.x >> self.grid_log2, value.point.y >> self.grid_log2);
        self.map.entry(key).or_default().push(value);
    }

    // Point.hpp:433-458 — closest value within search_radius (or None).
    fn find(&self, pt: Point) -> (Option<CPoint>, f64) {
        let mut value_min: Option<CPoint> = None;
        let mut dist_min = f64::MAX;
        let grid_corner = (
            (pt.x + (self.grid_resolution >> 1)) >> self.grid_log2,
            (pt.y + (self.grid_resolution >> 1)) >> self.grid_log2,
        );
        for neighbor_y in -1..1 {
            for neighbor_x in -1..1 {
                if let Some(bucket) =
                    self.map.get(&(grid_corner.0 + neighbor_x, grid_corner.1 + neighbor_y))
                {
                    for value in bucket {
                        let d2 = pt_to_vec2d(pt - value.point).length_squared();
                        if d2 < dist_min {
                            dist_min = d2;
                            value_min = Some(*value);
                        }
                    }
                }
            }
        }
        if value_min.is_some() && dist_min < self.search_radius as f64 * self.search_radius as f64 {
            (value_min, dist_min)
        } else {
            (None, f64::MAX)
        }
    }

    // Point.hpp:461-483 — every value within search_radius (squared distances).
    fn find_all(&self, pt: Point) -> Vec<(CPoint, f64)> {
        let mut out: Vec<(CPoint, f64)> = Vec::new();
        let r2 = self.search_radius as f64 * self.search_radius as f64;
        let grid_corner = (
            (pt.x + (self.grid_resolution >> 1)) >> self.grid_log2,
            (pt.y + (self.grid_resolution >> 1)) >> self.grid_log2,
        );
        for neighbor_y in -1..1 {
            for neighbor_x in -1..1 {
                if let Some(bucket) =
                    self.map.get(&(grid_corner.0 + neighbor_x, grid_corner.1 + neighbor_y))
                {
                    for value in bucket {
                        let d2 = pt_to_vec2d(pt - value.point).length_squared();
                        if d2 <= r2 {
                            out.push((*value, d2));
                        }
                    }
                }
            }
        }
        out
    }
}

// MultiMaterialSegmentation.cpp:48-56 — vertex_equal_to_point. C++ forces the FPU
// temporary to 64-bit then compares with boost::polygon's ULP comparison
// (`vertex_equality_predicate_type::ULPS` = 128). `ulp_eq` reproduces that ULP distance.
#[inline]
fn vertex_equal_to_point(vx: f64, vy: f64, ipt: PointF) -> bool {
    ulp_eq(vx, ipt.x, 128) && ulp_eq(vy, ipt.y, 128)
}

// boost::polygon::detail::ulp_comparison<double>: EQUAL iff the two doubles are within
// `ulps` representable steps of each other (same logic as `geometry::voronoi_annotation`).
fn ulp_eq(a: f64, b: f64, ulps: i64) -> bool {
    if a == b {
        return true;
    }
    if a.is_nan() || b.is_nan() {
        return false;
    }
    if a.is_infinite() || b.is_infinite() {
        return a == b;
    }
    let a_bits = a.to_bits() as i64;
    let b_bits = b.to_bits() as i64;
    if (a_bits ^ b_bits) < 0 {
        return a.abs() < f64::EPSILON && b.abs() < f64::EPSILON;
    }
    (a_bits - b_bits).abs() <= ulps
}

// The source (input-segment) index of the cell an edge belongs to. Because build_graph
// constructs the Voronoi diagram from exactly the flat `to_lines(color_poly)` segment
// list, this index maps 1:1 onto `lines_colored` and the graph's border arcs.
#[inline]
fn edge_cell_source_index(diagram: &bv::Diagram, edge_id: bv::EdgeIndex) -> Option<usize> {
    let cell_id = diagram.edge_get_cell(edge_id).ok()?;
    Some(diagram.cell(cell_id).ok()?.source_index().usize())
}

// Read a Voronoi vertex's exact (scaled-double) coordinates.
#[inline]
fn vd_vertex_xy(diagram: &bv::Diagram, vertex_id: bv::VertexIndex) -> Option<(f64, f64)> {
    diagram.vertex(vertex_id).ok().map(|v| (v.x(), v.y()))
}

// VoronoiVisualUtils.hpp:233-241 — Voronoi::Internal::retrieve_point. For our segment-only
// input a point cell is a segment endpoint, so SegmentStart → low (a), SegmentEnd → high
// (b); SinglePoint falls back to the point list (unused here, kept for fidelity).
#[inline]
fn retrieve_point_vd(
    points: &[Point],
    segments: &[(PointF, PointF)],
    src_index: usize,
    src_cat: bv::SourceCategory,
) -> PointF {
    match src_cat {
        bv::SourceCategory::SinglePoint => {
            PointF::new(points[src_index].x as f64, points[src_index].y as f64)
        }
        bv::SourceCategory::SegmentStart => segments[src_index].0,
        _ => segments[src_index].1,
    }
}

// VoronoiVisualUtils.hpp:243-279 — Voronoi::Internal::clip_infinite_edge. Returns the two
// clipped sample points of an infinite edge (empty on the degenerate two-segment case).
fn clip_infinite_edge(
    diagram: &bv::Diagram,
    points: &[Point],
    segments: &[(PointF, PointF)],
    edge_id: bv::EdgeIndex,
    bbox_max_size: f64,
) -> Vec<PointF> {
    // VoronoiVisualUtils.hpp:245-246 — assert is_infinite() and exactly one null vertex.
    let v0 = diagram.edge_get_vertex0(edge_id).ok().flatten();
    let v1 = diagram.edge_get_vertex1(edge_id).ok().flatten();

    let cell1_id = match diagram.edge_get_cell(edge_id) {
        Ok(c) => c,
        Err(_) => return Vec::new(),
    };
    let twin_id = match diagram.edge_get_twin(edge_id) {
        Ok(t) => t,
        Err(_) => return Vec::new(),
    };
    let cell2_id = match diagram.edge_get_cell(twin_id) {
        Ok(c) => c,
        Err(_) => return Vec::new(),
    };

    // Extract the cell fields we need (owned) so no diagram borrow is held across the math.
    let (c1_point, c1_seg, c1_src, c1_cat) = match diagram.cell(cell1_id) {
        Ok(c) => (
            c.contains_point(),
            c.contains_segment(),
            c.source_index().usize(),
            c.source_category(),
        ),
        Err(_) => return Vec::new(),
    };
    let (c2_point, c2_seg, c2_src, c2_cat) = match diagram.cell(cell2_id) {
        Ok(c) => (
            c.contains_point(),
            c.contains_segment(),
            c.source_index().usize(),
            c.source_category(),
        ),
        Err(_) => return Vec::new(),
    };

    // VoronoiVisualUtils.hpp:251-255 — infinite edges cannot separate two segment cells.
    if !c1_point && !c2_point {
        return Vec::new();
    }

    // VoronoiVisualUtils.hpp:256-270 — the direction the infinite ray travels.
    let direction: PointF = if c1_point && c2_point {
        // Point-Point bisector (primary edge).
        let mut p1 = retrieve_point_vd(points, segments, c1_src, c1_cat);
        let mut p2 = retrieve_point_vd(points, segments, c2_src, c2_cat);
        if v0.is_none() {
            std::mem::swap(&mut p1, &mut p2);
        }
        PointF::new(p1.y - p2.y, p2.x - p1.x)
    } else {
        // Point-Segment bisector (secondary edge): perpendicular to the segment.
        let seg = if c1_seg {
            segments[c1_src]
        } else {
            segments[c2_src]
        };
        // direction.x = high(seg).y - low(seg).y; direction.y = low(seg).x - high(seg).x.
        PointF::new(seg.1.y - seg.0.y, seg.0.x - seg.1.x)
    };
    let _ = c2_seg;

    // VoronoiVisualUtils.hpp:271-278 — extend the finite endpoint along `direction`.
    let koef = bbox_max_size / direction.x.abs().max(direction.y.abs());
    let mut clipped: Vec<PointF> = Vec::new();
    if v0.is_none() {
        let (vx, vy) = match v1.and_then(|vid| vd_vertex_xy(diagram, vid)) {
            Some(xy) => xy,
            None => return Vec::new(),
        };
        clipped.push(PointF::new(vx + direction.x * koef, vy + direction.y * koef));
        clipped.push(PointF::new(vx, vy));
    } else {
        let (vx, vy) = match v0.and_then(|vid| vd_vertex_xy(diagram, vid)) {
            Some(xy) => xy,
            None => return Vec::new(),
        };
        clipped.push(PointF::new(vx, vy));
        clipped.push(PointF::new(vx + direction.x * koef, vy + direction.y * koef));
    }
    clipped
}

// MultiMaterialSegmentation.cpp:1650-1666 — clip_finite_voronoi_edge. `v0`/`v1` are the
// two vertex coordinates as scaled doubles (mk_vec2). All Point conversions truncate
// toward zero (mk_point(vertex) / Vec2d::cast<coord_t>()).
fn clip_finite_voronoi_edge(v0: PointF, v1: PointF, bbox: &BoundingBoxF) -> Line {
    let contains_v0 = bbox.contains_point(&v0);
    let contains_v1 = bbox.contains_point(&v1);
    if (contains_v0 && contains_v1) || (!contains_v0 && !contains_v1) {
        return Line::new(vec2d_to_pt(v0), vec2d_to_pt(v1));
    }

    let vector = (v1 - v0).normalize() * bbox.size().length();
    let (nv0, nv1) = if !contains_v0 {
        (v1 - vector, v1)
    } else {
        (v0, v0 + vector)
    };
    Line::new(vec2d_to_pt(nv0), vec2d_to_pt(nv1))
}

// MultiMaterialSegmentation.cpp:1638-1643 — mark_processed: set the processed flag on the
// half-edge AND its twin (C++ `edge->color(true)`).
#[inline]
fn mark_processed(diagram: &mut bv::Diagram, edge_id: bv::EdgeIndex) {
    let _ = diagram.edge_set_color(edge_id, 1);
    if let Ok(twin) = diagram.edge_get_twin(edge_id) {
        let _ = diagram.edge_set_color(twin, 1);
    }
}

// MultiMaterialSegmentation.cpp:1711-1717 — get_prev_contour_line lambda.
#[inline]
fn prev_contour_line(
    lines_colored: &[ColoredLine],
    color_poly: &[Vec<ColoredLine>],
    graph: &MmuGraph,
    source_index: usize,
) -> ColoredLine {
    let cl = lines_colored[source_index];
    let local = cl.local_line_idx as usize;
    let size = color_poly[cl.poly_idx as usize].len();
    let prev = graph.get_global_index(cl.poly_idx as usize, if local > 0 { local - 1 } else { size - 1 });
    lines_colored[prev]
}

// MultiMaterialSegmentation.cpp:1719-1724 — get_next_contour_line lambda.
#[inline]
fn next_contour_line(
    lines_colored: &[ColoredLine],
    color_poly: &[Vec<ColoredLine>],
    graph: &MmuGraph,
    source_index: usize,
) -> ColoredLine {
    let cl = lines_colored[source_index];
    let local = cl.local_line_idx as usize;
    let size = color_poly[cl.poly_idx as usize].len();
    let next = graph.get_global_index(cl.poly_idx as usize, (local + 1) % size);
    lines_colored[next]
}

// MultiMaterialSegmentation.cpp:1774-1788 — append_edge_if_intersects_with_contour lambda.
// `vertex_color` is the resolved graph node index of vertex0 or vertex1 (C++ selects it
// via the `Vertex` enum). `edge_line`/`contour_line` are captured from the caller.
#[allow(clippy::too_many_arguments)]
fn append_edge_if_intersects_with_contour(
    graph: &mut MmuGraph,
    diagram: &mut bv::Diagram,
    lines_colored: &[ColoredLine],
    edge_line: &Line,
    contour_line: &Line,
    edge_id: bv::EdgeIndex,
    cell_src: usize,
    twin_cell_src: usize,
    vertex_color: usize,
) {
    let contour_line_twin = lines_colored[twin_cell_src].line;
    let mut intersection = Point::default();
    if line_intersection_with_epsilon(&contour_line_twin, edge_line, &mut intersection) {
        let graph_arc = graph.get_border_arc(twin_cell_src);
        let to_idx_l = if is_point_closer_to_beginning_of_line(&contour_line_twin, &intersection) {
            graph_arc.from_idx
        } else {
            graph_arc.to_idx
        };
        graph.append_edge_default(vertex_color, to_idx_l);
    } else if line_intersection_with_epsilon(contour_line, edge_line, &mut intersection) {
        let graph_arc = graph.get_border_arc(cell_src);
        let to_idx_l = if is_point_closer_to_beginning_of_line(contour_line, &intersection) {
            graph_arc.from_idx
        } else {
            graph_arc.to_idx
        };
        graph.append_edge_default(vertex_color, to_idx_l);
    }
    mark_processed(diagram, edge_id);
}

// MultiMaterialSegmentation.cpp:1670-1885 — build_graph. The C++ `throw_on_cancel`
// parameter is dropped (the Rust port runs serially).
pub fn build_graph(_layer_idx: usize, color_poly: &[Vec<ColoredLine>]) -> MmuGraph {
    // MultiMaterialSegmentation.cpp:1673-1675.
    let color_poly_tmp = colored_points_to_polygon(color_poly);
    // to_points(color_poly_tmp) / to_lines(color_poly_tmp): the flat contour point and
    // line lists. Derived from the same source as the Voronoi segments below so cell
    // source indices line up with `lines_colored` and the graph border arcs.
    let mut points: Vec<Point> = Vec::new();
    let mut lines: Vec<Line> = Vec::new();
    for polygon in &color_poly_tmp {
        for pt in &polygon.points {
            points.push(*pt);
        }
        for line in polygon.lines() {
            lines.push(line);
        }
    }

    // MultiMaterialSegmentation.cpp:1682-1693 — force_edge_adding: true for each polygon
    // that is coloured entirely with a single colour (so at least one edge is still added).
    let mut force_edge_adding: Vec<bool> = vec![false; color_poly.len()];
    for (poly_idx, c_poly) in color_poly.iter().enumerate() {
        let first_color = match c_poly.first() {
            Some(l) => l.color,
            None => continue,
        };
        let mut force_edge = true;
        for c_line in c_poly {
            if c_line.color != first_color {
                force_edge = false;
                break;
            }
        }
        force_edge_adding[poly_idx] = force_edge;
    }

    // MultiMaterialSegmentation.cpp:1695-1699 — construct the Voronoi diagram from the flat
    // colored-line segment list (order == to_lines_colored, so cell source_index maps to it).
    let mut lines_colored = to_lines_colored(color_poly);
    let bv_segments: Vec<bv::Line<i64>> = lines_colored
        .iter()
        .map(|cl| {
            bv::Line::new(
                bv::Point {
                    x: cl.line.a.x,
                    y: cl.line.a.y,
                },
                bv::Point {
                    x: cl.line.b.x,
                    y: cl.line.b.y,
                },
            )
        })
        .collect();
    let diagram_opt = bv::Builder::<i64>::default()
        .with_segments(bv_segments.iter())
        .and_then(|b| b.build())
        .ok();

    // MultiMaterialSegmentation.cpp:1700-1707 — seed the graph with one node per contour
    // point, then add the border arcs and per-line polygon indices.
    let mut graph = MmuGraph::default();
    graph.nodes.reserve(
        points.len() + diagram_opt.as_ref().map(|d| d.vertices().len()).unwrap_or(0),
    );
    for point in &points {
        graph.nodes.push(Node {
            point: PointF::new(point.x as f64, point.y as f64),
            arc_idxs: Vec::new(),
        });
    }
    graph.add_contours(color_poly);
    init_polygon_indices(&graph, color_poly, &mut lines_colored);
    debug_assert!(graph.nodes.len() == lines_colored.len());

    // If the raw Voronoi build failed (see DEVIATION above), return the contour graph.
    let mut diagram = match diagram_opt {
        Some(d) => d,
        None => {
            graph.remove_nodes_with_one_arc();
            return graph;
        }
    };

    // MultiMaterialSegmentation.cpp:1708-1709 — append the interior Voronoi vertices.
    let bbox = polygons_extents(&color_poly_tmp);
    graph.append_voronoi_vertices(&mut diagram, &color_poly_tmp, bbox);

    // MultiMaterialSegmentation.cpp:1726-1733 — the clip bbox (bbox grown by scale_(10)),
    // the max bbox dimension, and the double-typed input segments for clip_infinite_edge.
    let mut clip_bbox = bbox;
    clip_bbox.expand(scale_(10.0));
    let bbox_clip = BoundingBoxF::from_points_minmax(
        PointF::new(clip_bbox.min.x as f64, clip_bbox.min.y as f64),
        PointF::new(clip_bbox.max.x as f64, clip_bbox.max.y as f64),
    );
    let bbox_dim_max = (clip_bbox.size().x.max(clip_bbox.size().y)) as f64;
    let segments: Vec<(PointF, PointF)> = lines
        .iter()
        .map(|l| {
            (
                PointF::new(l.a.x as f64, l.a.y as f64),
                PointF::new(l.b.x as f64, l.b.y as f64),
            )
        })
        .collect();

    let num_edges = diagram.edges().len();

    // MultiMaterialSegmentation.cpp:1735-1866 — first edge pass (special cases first).
    for edge_idx in 0..num_edges {
        let edge_id = diagram.edge_index_unchecked(edge_idx);
        let cell_src = match edge_cell_source_index(&diagram, edge_id) {
            Some(s) => s,
            None => continue,
        };
        let twin_edge_id = match diagram.edge_get_twin(edge_id) {
            Ok(t) => t,
            Err(_) => continue,
        };
        let twin_cell_src = match edge_cell_source_index(&diagram, twin_edge_id) {
            Some(s) => s,
            None => continue,
        };

        // MultiMaterialSegmentation.cpp:1737 — skip the second half-edge and processed edges.
        let processed = diagram.edge_get_color(edge_id).unwrap_or(0) != 0;
        if cell_src > twin_cell_src || processed {
            continue;
        }

        let v0_id = diagram.edge_get_vertex0(edge_id).ok().flatten();
        let v1_id = diagram.edge_get_vertex1(edge_id).ok().flatten();
        // An edge is finite iff both endpoints exist.
        let is_finite = v0_id.is_some() && v1_id.is_some();

        if !is_finite && (v0_id.is_some() || v1_id.is_some()) {
            // MultiMaterialSegmentation.cpp:1739-1761 — infinite edge through a contour point.
            let samples = clip_infinite_edge(&diagram, &points, &segments, edge_id, bbox_dim_max);
            if samples.is_empty() {
                continue;
            }
            let edge_line = Line::new(vec2d_to_pt(samples[0]), vec2d_to_pt(samples[1]));
            let contour_line = lines_colored[cell_src];
            let mut contour_intersection = Point::default();
            if line_intersection_with_epsilon(&contour_line.line, &edge_line, &mut contour_intersection)
            {
                let graph_arc = graph.get_border_arc(cell_src);
                let from_idx = (if v1_id.is_some() {
                    v1_id.and_then(|vid| diagram.vertex_get_color(vid))
                } else {
                    v0_id.and_then(|vid| diagram.vertex_get_color(vid))
                })
                .unwrap_or(VD_VERTEX_UNSET) as usize;
                let to_idx = if pt_to_vec2d(contour_line.line.a - contour_intersection).length_squared()
                    < pt_to_vec2d(contour_line.line.b - contour_intersection).length_squared()
                {
                    graph_arc.from_idx
                } else {
                    graph_arc.to_idx
                };
                if from_idx != to_idx
                    && from_idx < graph.nodes_count()
                    && to_idx < graph.nodes_count()
                {
                    graph.append_edge_default(from_idx, to_idx);
                    mark_processed(&mut diagram, edge_id);
                }
            }
        } else if is_finite {
            let v0_id = v0_id.unwrap();
            let v1_id = v1_id.unwrap();
            let v0_color = diagram.vertex_get_color(v0_id).unwrap_or(VD_VERTEX_UNSET);
            let v1_color = diagram.vertex_get_color(v1_id).unwrap_or(VD_VERTEX_UNSET);
            let v0_idx = v0_color as usize;
            let v1_idx = v1_color as usize;

            // MultiMaterialSegmentation.cpp:1763-1764 — both on contour, or a merged
            // duplicate vertex (same color): skip.
            if graph.is_edge_connecting_two_contour_vertices(v0_color, v1_color)
                || v0_color == v1_color
            {
                continue;
            }

            // MultiMaterialSegmentation.cpp:1766-1770.
            let (v0x, v0y) = match vd_vertex_xy(&diagram, v0_id) {
                Some(xy) => xy,
                None => continue,
            };
            let (v1x, v1y) = match vd_vertex_xy(&diagram, v1_id) {
                Some(xy) => xy,
                None => continue,
            };
            let edge_line =
                clip_finite_voronoi_edge(PointF::new(v0x, v0y), PointF::new(v1x, v1y), &bbox_clip);
            let contour_line = lines_colored[cell_src].line;
            let colored_line = lines_colored[cell_src];
            let contour_line_prev = prev_contour_line(&lines_colored, color_poly, &graph, cell_src);
            let contour_line_next = next_contour_line(&lines_colored, color_poly, &graph, cell_src);
            let poly_idx = colored_line.poly_idx as usize;
            let nodes_count = graph.nodes_count();

            if v0_idx >= nodes_count || v1_idx >= nodes_count {
                // MultiMaterialSegmentation.cpp:1772-1794 — one endpoint is an interior
                // (non-contour) vertex; add an edge where the ray meets a contour line.
                if v0_idx < nodes_count && !graph.is_vertex_on_contour(v0_color) {
                    append_edge_if_intersects_with_contour(
                        &mut graph,
                        &mut diagram,
                        &lines_colored,
                        &edge_line,
                        &contour_line,
                        edge_id,
                        cell_src,
                        twin_cell_src,
                        v0_idx,
                    );
                }
                if v1_idx < nodes_count && !graph.is_vertex_on_contour(v1_color) {
                    append_edge_if_intersects_with_contour(
                        &mut graph,
                        &mut diagram,
                        &lines_colored,
                        &edge_line,
                        &contour_line,
                        edge_id,
                        cell_src,
                        twin_cell_src,
                        v1_idx,
                    );
                }
            } else if graph.is_edge_attach_to_contour(v0_color, v1_color) {
                // MultiMaterialSegmentation.cpp:1795-1831.
                mark_processed(&mut diagram, edge_id);
                if graph.is_edge_connecting_two_contour_vertices(v0_color, v1_color) {
                    continue;
                }
                let from_idx = v0_idx;
                let to_idx = v1_idx;
                if graph.is_vertex_on_contour(v0_color) {
                    if is_point_closer_to_beginning_of_line(&contour_line, &edge_line.a) {
                        if (!has_same_color(&contour_line_prev, &colored_line)
                            || force_edge_adding[poly_idx])
                            && points_inside(&contour_line_prev.line, &contour_line, &edge_line.b)
                        {
                            graph.append_edge_default(from_idx, to_idx);
                            force_edge_adding[poly_idx] = false;
                        }
                    } else if (!has_same_color(&contour_line_next, &colored_line)
                        || force_edge_adding[poly_idx])
                        && points_inside(&contour_line, &contour_line_next.line, &edge_line.b)
                    {
                        graph.append_edge_default(from_idx, to_idx);
                        force_edge_adding[poly_idx] = false;
                    }
                } else {
                    // is_vertex_on_contour(vertex1)
                    debug_assert!(graph.is_vertex_on_contour(v1_color));
                    if is_point_closer_to_beginning_of_line(&contour_line, &edge_line.b) {
                        if (!has_same_color(&contour_line_prev, &colored_line)
                            || force_edge_adding[poly_idx])
                            && points_inside(&contour_line_prev.line, &contour_line, &edge_line.a)
                        {
                            graph.append_edge_default(from_idx, to_idx);
                            force_edge_adding[poly_idx] = false;
                        }
                    } else if (!has_same_color(&contour_line_next, &colored_line)
                        || force_edge_adding[poly_idx])
                        && points_inside(&contour_line, &contour_line_next.line, &edge_line.a)
                    {
                        graph.append_edge_default(from_idx, to_idx);
                        force_edge_adding[poly_idx] = false;
                    }
                }
            } else {
                // MultiMaterialSegmentation.cpp:1832-1863 — both endpoints interior; split
                // the contour line at the intersection and connect the visible side(s).
                let mut intersection = Point::default();
                if line_intersection_with_epsilon(&contour_line, &edge_line, &mut intersection) {
                    mark_processed(&mut diagram, edge_id);
                    let real_v0 = vec2d_to_pt(graph.nodes[v0_idx].point);
                    let real_v1 = vec2d_to_pt(graph.nodes[v1_idx].point);

                    if is_point_closer_to_beginning_of_line(&contour_line, &intersection) {
                        let first_part = Line::new(intersection, real_v0);
                        let second_part = Line::new(intersection, real_v1);
                        if !has_same_color(&contour_line_prev, &colored_line) {
                            let arc_from = graph.get_border_arc(cell_src).from_idx;
                            if points_inside(&contour_line_prev.line, &contour_line, &first_part.b) {
                                graph.append_edge_default(v0_idx, arc_from);
                            }
                            if points_inside(&contour_line_prev.line, &contour_line, &second_part.b) {
                                graph.append_edge_default(v1_idx, arc_from);
                            }
                        }
                    } else {
                        let int_point_idx = graph.get_border_arc(cell_src).to_idx;
                        // int_point (truncated) is computed in C++ but only the index feeds
                        // append_edge; the `first_part`/`second_part` endpoints tested below
                        // are real_v0/real_v1.
                        let int_point = vec2d_to_pt(graph.nodes[int_point_idx].point);
                        let first_part = Line::new(int_point, real_v0);
                        let second_part = Line::new(int_point, real_v1);
                        if !has_same_color(&contour_line_next, &colored_line) {
                            if points_inside(&contour_line, &contour_line_next.line, &first_part.b) {
                                graph.append_edge_default(v0_idx, int_point_idx);
                            }
                            if points_inside(&contour_line, &contour_line_next.line, &second_part.b) {
                                graph.append_edge_default(v1_idx, int_point_idx);
                            }
                        }
                    }
                }
            }
        }
    }

    // MultiMaterialSegmentation.cpp:1868-1881 — second pass: all remaining finite interior
    // edges, then mark every edge processed.
    for edge_idx in 0..num_edges {
        let edge_id = diagram.edge_index_unchecked(edge_idx);
        let cell_src = match edge_cell_source_index(&diagram, edge_id) {
            Some(s) => s,
            None => continue,
        };
        let twin_edge_id = match diagram.edge_get_twin(edge_id) {
            Ok(t) => t,
            Err(_) => continue,
        };
        let twin_cell_src = match edge_cell_source_index(&diagram, twin_edge_id) {
            Some(s) => s,
            None => continue,
        };
        let processed = diagram.edge_get_color(edge_id).unwrap_or(0) != 0;
        if cell_src > twin_cell_src || processed {
            continue;
        }

        let v0_id = diagram.edge_get_vertex0(edge_id).ok().flatten();
        let v1_id = diagram.edge_get_vertex1(edge_id).ok().flatten();
        let is_finite = v0_id.is_some() && v1_id.is_some();
        if is_finite {
            let v0_color = diagram.vertex_get_color(v0_id.unwrap()).unwrap_or(VD_VERTEX_UNSET);
            let v1_color = diagram.vertex_get_color(v1_id.unwrap()).unwrap_or(VD_VERTEX_UNSET);
            if (v0_color as usize) < graph.nodes_count() && (v1_color as usize) < graph.nodes_count()
            {
                // Skip edges between two merged-to-same vertices.
                if v0_color == v1_color {
                    continue;
                }
                graph.append_edge_default(v0_color as usize, v1_color as usize);
            }
        }
        mark_processed(&mut diagram, edge_id);
    }

    // MultiMaterialSegmentation.cpp:1883-1884.
    graph.remove_nodes_with_one_arc();
    graph
}

// Extents of a set of polygons (mirrors `get_extents(const Polygons &)` over the contour
// points), used to seed build_graph's Voronoi-vertex bounding box.
fn polygons_extents(polygons: &[Polygon]) -> BoundingBox {
    let mut bbox = BoundingBox::empty();
    for polygon in polygons {
        for pt in &polygon.points {
            bbox.merge_point(*pt);
        }
    }
    bbox
}

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
// Segmented-layer post-processing (MultiMaterialSegmentation.cpp:1284, 1968)
// ---------------------------------------------------------------------------

// MultiMaterialSegmentation.cpp:1284
// `cut_width` / `interlocking_depth` are *scaled* floats (the C++ caller passes
// `scale_(...)` values). The tbb::parallel_for runs serially and the
// `throw_on_cancel_callback` parameter is omitted, per crate convention. The crate
// clipper offsets take mm, hence the `/ SCALING_FACTOR` at the offset call site.
#[allow(dead_code)]
fn cut_segmented_layers(
    input_expolygons: &[ExPolygons],
    segmented_regions: &mut [Vec<ExPolygons>],
    cut_width: f32,
    interlocking_depth: f32,
) {
    // MultiMaterialSegmentation.cpp:1291 (computed but never read in the C++ loop;
    // kept for parity)
    let _interlocking_cut_width: f32 = if interlocking_depth > 0.0 {
        (cut_width - interlocking_depth).max(0.0)
    } else {
        0.0
    };
    // MultiMaterialSegmentation.cpp:1292-1306
    for layer_idx in 0..segmented_regions.len() {
        // MultiMaterialSegmentation.cpp:1296-1297
        let region_cut_width: f32 = if layer_idx % 2 == 0 && interlocking_depth != 0.0 {
            interlocking_depth
        } else {
            cut_width
        };
        let num_extruders_plus_one = segmented_regions[layer_idx].len();
        if region_cut_width > 0.0 {
            // MultiMaterialSegmentation.cpp:1299-1304
            // Indexed by extruder_id
            let mut segmented_regions_cuts: Vec<ExPolygons> =
                vec![Vec::new(); num_extruders_plus_one];
            for extruder_idx in 0..num_extruders_plus_one {
                let ex_polygons = &segmented_regions[layer_idx][extruder_idx];
                if !ex_polygons.is_empty() {
                    // diff_ex(ex_polygons, offset_ex(input_expolygons[layer_idx], -region_cut_width))
                    segmented_regions_cuts[extruder_idx] = difference(
                        ex_polygons,
                        &offset_expolygons(
                            &input_expolygons[layer_idx],
                            -(region_cut_width as f64) / SCALING_FACTOR,
                            OffsetJoinType::Miter,
                        ),
                    );
                }
            }
            segmented_regions[layer_idx] = segmented_regions_cuts;
        }
    }
}

// MultiMaterialSegmentation.cpp:1968
// `top_and_bottom_layers` is taken by value (the C++ takes
// `std::vector<std::vector<ExPolygons>> &&`). The tbb::parallel_for runs serially and
// the `throw_on_cancel_callback` parameter is omitted, per crate convention.
#[allow(dead_code)]
fn merge_segmented_layers(
    segmented_regions: &[Vec<ExPolygons>],
    top_and_bottom_layers: Vec<Vec<ExPolygons>>,
    num_extruders: usize,
) -> Vec<Vec<ExPolygons>> {
    // MultiMaterialSegmentation.cpp:1974-1977
    let num_layers = segmented_regions.len();
    let mut segmented_regions_merged: Vec<Vec<ExPolygons>> =
        vec![vec![ExPolygons::new(); num_extruders]; num_layers];
    debug_assert!(num_extruders + 1 == top_and_bottom_layers.len());

    // MultiMaterialSegmentation.cpp:1980-2006
    for layer_idx in 0..num_layers {
        debug_assert!(segmented_regions[layer_idx].len() == num_extruders + 1);
        // Zero is skipped because it is the default color of the volume
        for extruder_id in 1..(num_extruders + 1) {
            // MultiMaterialSegmentation.cpp:1987-1994
            if !segmented_regions[layer_idx][extruder_id].is_empty() {
                let mut segmented_regions_trimmed: ExPolygons =
                    segmented_regions[layer_idx][extruder_id].clone();
                for top_and_bottom_by_extruder in &top_and_bottom_layers {
                    if !top_and_bottom_by_extruder[layer_idx].is_empty()
                        && !segmented_regions_trimmed.is_empty()
                    {
                        segmented_regions_trimmed = difference(
                            &segmented_regions_trimmed,
                            &top_and_bottom_by_extruder[layer_idx],
                        );
                    }
                }

                segmented_regions_merged[layer_idx][extruder_id - 1] = segmented_regions_trimmed;
            }

            // MultiMaterialSegmentation.cpp:1996-2004
            if !top_and_bottom_layers[extruder_id][layer_idx].is_empty() {
                let was_top_and_bottom_empty =
                    segmented_regions_merged[layer_idx][extruder_id - 1].is_empty();
                segmented_regions_merged[layer_idx][extruder_id - 1]
                    .extend_from_slice(&top_and_bottom_layers[extruder_id][layer_idx]);

                // Remove dimples (#7235) appearing after merging side segmentation of the model with tops and bottoms painted layers.
                // offset2_ex(union_ex(...), float(SCALED_EPSILON), -float(SCALED_EPSILON))
                // grows then shrinks by SCALED_EPSILON == crate `closing` with the
                // equivalent mm distance.
                if !was_top_and_bottom_empty {
                    segmented_regions_merged[layer_idx][extruder_id - 1] = closing(
                        &union_ex(&segmented_regions_merged[layer_idx][extruder_id - 1]),
                        SCALED_EPSILON / SCALING_FACTOR,
                        OffsetJoinType::Miter,
                    );
                }
            }
        }
    }

    segmented_regions_merged
}

// ---------------------------------------------------------------------------
// Tier-1 orchestrator: multi_material_segmentation_by_painting
// (MultiMaterialSegmentation.cpp:2095-2409)
// ---------------------------------------------------------------------------

/// upper_bound over an ascending `f64` slice: first index `i` with `zs[i] > value`.
/// Mirrors `std::upper_bound(begin, end, value, [](float z, const Layer *l){ return z < l->slice_z; })`
/// (MultiMaterialSegmentation.cpp:2257-2260) — returns the first position where the
/// comparator `value < zs[i]` holds.
fn upper_bound_f64(zs: &[f64], value: f64) -> usize {
    let mut lo = 0usize;
    let mut hi = zs.len();
    while lo < hi {
        let mid = lo + (hi - lo) / 2;
        if value < zs[mid] {
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }
    lo
}

/// Flatten an `ExPolygons` into the closed contours an `EdgeGrid` rasterizes, in the exact
/// order of `EdgeGrid::Grid::create(const ExPolygons&, ...)` (EdgeGrid.cpp:117-139): per
/// expolygon, the outer contour first, then each (non-empty) hole. All are closed polygons.
fn expolygons_to_edge_grid_contours(exs: &[ExPolygon]) -> Vec<Polygon> {
    let mut polys: Vec<Polygon> = Vec::new();
    for ex in exs {
        // EdgeGrid.cpp:131-132
        if !ex.contour.points().is_empty() {
            polys.push(ex.contour.clone());
        }
        // EdgeGrid.cpp:133-135
        for hole in &ex.holes {
            if !hole.points().is_empty() {
                polys.push(hole.clone());
            }
        }
    }
    polys
}

/// Tier-1 port of `multi_material_segmentation_by_painting` (MultiMaterialSegmentation.cpp:2095).
///
/// The C++ takes a `PrintObject` and pulls everything off it (layers, regions, model
/// volumes, painted facet annotations, print config). The Tier-1 caller instead passes the
/// already-prepared inputs directly:
///   * `layer_slices[l]`   — the merged region slices for layer `l` (scaled coords), i.e. the
///                           C++ `layers[l]->regions()...surface.expolygon` union.
///   * `layer_slice_zs[l]` — that layer's `slice_z` in mm (ascending).
///   * `painted_submeshes` — one `(extruder_slot, sub-mesh)` per painted extruder, where the
///                           sub-mesh is the C++ `mv.mmu_segmentation_facets.get_facets(slot)`
///                           result (mm coords, `f32` verts). `extruder_slot` is 1-based and
///                           becomes the painted `color`.
///   * `num_extruders`     — total filament slots.
///   * `segmented_max_width` / `segmented_interlocking_depth` — the two `cut_segmented_layers`
///                           knobs, already *scaled* (`scale_(mmu_segmented_region_*)`); `0.0`
///                           skips the cut (matching the C++ gate `max_width > 0 || depth > 0`).
///
/// Returns `merge_segmented_layers`' output: `[layer][extruder]`, `num_extruders` slots per
/// layer, indexed by 0-based extruder (slot `j` == painted filament slot `j + 1`). The
/// unpainted/default color (0) is NOT a slot — it is dropped by the merge exactly as C++ does,
/// matching the consumer `PrintObjectSlice.cpp:877-879` which indexes `segmentation[l][0..num_extruders]`.
///
/// TIER-1 DEVIATIONS FROM THE C++ (each also noted inline):
///   * No `PrintObject`/`ModelVolume`; inputs are passed in. `trafo` is identity (mesh
///     pre-placed) and `center_offset` is `(0,0)`, so `line_to_test.translate(-center_offset)`
///     is a no-op (kept as a comment).
///   * Negative volumes are dropped by the Tier-1 loader, so the `input_expolygons_filled`
///     branch (cpp:2143-2197), the clip-back (cpp:2369-2383) and the `input_for_edge_grid`
///     alias are all omitted — `input_expolygons` is used everywhere.
///   * No TBB — plain sequential loops.
///   * FIDELITY-NOTE: `expolygons_simplify` + `remove_duplicates` in the input prep (cpp:2134)
///     are not ported (they guard Voronoi robustness against self-intersections / near-duplicate
///     points; acceptable Tier-1 risk).
///   * FIDELITY-NOTE: `mmu_segmentation_top_and_bottom_layers` (cpp:2393) is stubbed to the
///     empty "no top/bottom overrides" structure; horizontal painted-surface propagation
///     (the only `slice_mesh_slabs` consumer) is not yet ported.
///   * FIDELITY-NOTE: the crate's `EdgeGrid::create` recomputes its bbox from contour extents
///     (+16) and ignores the pre-set bbox, so the clip at cpp:2293-2303 uses the contour-derived
///     bbox rather than the C++ merged-adjacent-layer bbox. `set_bbox` is still called first to
///     mirror the C++ order; the effective clip is a subset of the C++ one (harmless — the
///     visitor only registers lines near actual contour segments).
///   * FIDELITY-NOTE: the crate's `visit_cells_intersecting_line` lacks the C++
///     `need_consider_eps=true` extension (cpp:2311); it visits only cells the segment directly
///     crosses, not the surrounding epsilon band.
pub fn multi_material_segmentation_by_painting_tier1(
    layer_slices: &[ExPolygons],
    layer_slice_zs: &[f64],
    painted_submeshes: &[(u8, indexed_triangle_set)],
    num_extruders: usize,
    segmented_max_width: f32,
    segmented_interlocking_depth: f32,
) -> Vec<Vec<ExPolygons>> {
    // MultiMaterialSegmentation.cpp:2098-2105
    let num_layers = layer_slices.len();
    debug_assert!(layer_slice_zs.len() == num_layers);
    let mut segmented_regions: Vec<Vec<ExPolygons>> =
        vec![vec![ExPolygons::new(); num_extruders + 1]; num_layers];
    let mut painted_lines: Vec<Vec<PaintedLine>> = vec![Vec::new(); num_layers];
    let mut edge_grids: Vec<EdgeGrid> = Vec::with_capacity(num_layers);

    // Merge all regions and remove small holes. MultiMaterialSegmentation.cpp:2113-2141.
    // Tier-1: `layer_slices[l]` is already the merged region slices, so it stands in for the
    // C++ `for region: for surface: append(offset_ex(surface.expolygon, 10*SCALED_EPSILON))`.
    // `offset_ex` deltas are in mm here (the crate's `offset_expolygons` takes mm), so the
    // scaled `10 * SCALED_EPSILON` is converted via `/ SCALING_FACTOR`.
    let grow_mm = (10.0 * SCALED_EPSILON) / SCALING_FACTOR;
    // remove_small_and_small_holes(..., Slic3r::sqr(scale_(0.1f))). MultiMaterialSegmentation.cpp:2126.
    let min_area = sqr_f64(scale_(0.1) as f64);
    let mut input_expolygons: Vec<ExPolygons> = Vec::with_capacity(num_layers);
    for layer_idx in 0..num_layers {
        // cpp:2121 offset_ex(+10*SCALED_EPSILON) then cpp:2124 union_ex.
        let grown = offset_expolygons(&layer_slices[layer_idx], grow_mm, OffsetJoinType::Miter);
        let mut ex = union_ex(&grown);
        // cpp:2126
        crate::ex_polygon::remove_small_and_small_holes(&mut ex, min_area);
        // cpp:2134 offset_ex(-10*SCALED_EPSILON). FIDELITY-NOTE: the surrounding
        // remove_duplicates(expolygons_simplify(...)) is intentionally omitted (see doc-comment).
        input_expolygons.push(offset_expolygons(&ex, -grow_mm, OffsetJoinType::Miter));
    }

    // Negative-volume handling (cpp:2143-2197) is OMITTED: the Tier-1 loader drops negative
    // volumes, so `input_for_edge_grid == input_expolygons` everywhere below.

    // Per-layer bounding boxes. MultiMaterialSegmentation.cpp:2199-2204.
    // Tier-1: the layer's regions ARE `input_expolygons`, so the extents come from it alone
    // (the C++ `get_extents(layers[l]->regions())` merged with `get_extents(input...)`).
    let mut layer_bboxes: Vec<BoundingBox> = Vec::with_capacity(num_layers);
    for layer_idx in 0..num_layers {
        layer_bboxes.push(crate::ex_polygon::get_extents(&input_expolygons[layer_idx]));
    }

    // Build one EdgeGrid per layer. MultiMaterialSegmentation.cpp:2206-2218.
    for layer_idx in 0..num_layers {
        let mut bbox = layer_bboxes[layer_idx];
        // cpp:2212-2213 — merge with the previous/next layer (note the C++ `> 1`, not `> 0`).
        if layer_idx > 1 {
            bbox.merge(&layer_bboxes[layer_idx - 1]);
        }
        if layer_idx < num_layers - 1 {
            bbox.merge(&layer_bboxes[layer_idx + 1]);
        }
        // cpp:2215 — bbox.offset(30 * SCALED_EPSILON).
        bbox.expand((30.0 * SCALED_EPSILON) as Coord);
        // cpp:2216-2217 — set_bbox then create at resolution scale_(10.).
        // FIDELITY-NOTE (see doc-comment): the crate's create recomputes the bbox from contour
        // extents, so this set_bbox only mirrors the C++ order; the clip below reads back the
        // grid's own (contour-derived) bbox.
        let mut grid = EdgeGrid::new();
        grid.set_bbox(bbox);
        let contours = expolygons_to_edge_grid_contours(&input_expolygons[layer_idx]);
        grid.create_from_polygons(&contours, scale_(10.));
        edge_grids.push(grid);
    }

    // Projection of painted triangles onto the layers. MultiMaterialSegmentation.cpp:2220-2322.
    // Tier-1: the C++ loops `for mv: for extruder_idx in 1..=num_extruders:
    // mv.mmu_segmentation_facets.get_facets(extruder_idx)`. Here each submesh is already the
    // per-slot painted facet set, so we iterate the submeshes directly; `color` = its slot.
    for (extruder_slot, custom_facets) in painted_submeshes {
        // cpp:2229-2231 — skip volumes with no painted facets for this slot.
        if custom_facets.indices.is_empty() {
            continue;
        }
        // cpp:2310 — the painted color is the (1-based) extruder slot.
        let color = *extruder_slot as i32;
        // cpp:2233 — Tier-1 trafo is identity (mesh pre-placed), so `facet[p] = vertices[idx]`.
        for facet_idx in 0..custom_facets.indices.len() {
            // cpp:2240-2248
            let mut min_z = f32::MAX;
            let mut max_z = f32::MIN; // std::numeric_limits<float>::lowest()
            let mut facet: [Vec3f; 3] = [Vec3f::zeros(); 3];
            for p_idx in 0..3usize {
                let vidx = custom_facets.indices[facet_idx][p_idx] as usize;
                facet[p_idx] = custom_facets.vertices[vidx]; // identity trafo
                max_z = max_z.max(facet[p_idx].z);
                min_z = min_z.min(facet[p_idx].z);
            }

            // cpp:2250-2251
            if is_equal(min_z, max_z) {
                continue;
            }

            // cpp:2254 — sort the vertices by z.
            facet.sort_by(|a, b| a.z.partial_cmp(&b.z).unwrap_or(std::cmp::Ordering::Equal));

            // cpp:2256-2261 — first/last layer via upper_bound over slice_z, then `--last_layer`.
            let first_layer = upper_bound_f64(layer_slice_zs, min_z as f64 - EPSILON);
            let last_ub = upper_bound_f64(layer_slice_zs, max_z as f64 + EPSILON);
            if last_ub == 0 {
                continue;
            }
            let last_layer = last_ub - 1;

            // cpp:2263-2312 — `for (layer_it = first_layer; layer_it != last_layer + 1; ++layer_it)`.
            for layer_idx in first_layer..=last_layer {
                let slice_z = layer_slice_zs[layer_idx]; // coordf_t (double)
                let slice_z_f = slice_z as f32; // C++ `float(layer->slice_z)`

                // cpp:2266
                if input_expolygons[layer_idx].is_empty()
                    || is_less(slice_z_f, facet[0].z)
                    || is_less(facet[2].z, slice_z_f)
                {
                    continue;
                }

                // cpp:2270-2271
                let t = (slice_z_f - facet[0].z) / (facet[2].z - facet[0].z);
                let line_start_f = facet[0] + (facet[2] - facet[0]) * t;

                // cpp:2274-2287
                let line_end_f: Vec3f = if (is_equal(facet[0].z, facet[1].z)
                    && is_equal(facet[1].z, slice_z_f))
                    || (is_equal(facet[1].z, facet[2].z) && is_equal(facet[1].z, slice_z_f))
                {
                    // BBS: one side of the triangle coincides with slice_z.
                    facet[1]
                } else if (facet[1].z as f64) > slice_z {
                    // [P0, P2] and [P0, P1]
                    let t1 = (slice_z_f - facet[0].z) / (facet[1].z - facet[0].z);
                    facet[0] + (facet[1] - facet[0]) * t1
                } else {
                    // [P0, P2] and [P1, P2]
                    let t2 = (slice_z_f - facet[1].z) / (facet[2].z - facet[1].z);
                    facet[1] + (facet[2] - facet[1]) * t2
                };

                // cpp:2289-2291
                let mut line_to_test = Line::new(
                    Point::new(scale_(line_start_f.x as f64), scale_(line_start_f.y as f64)),
                    Point::new(scale_(line_end_f.x as f64), scale_(line_end_f.y as f64)),
                );
                // cpp:2291 — line_to_test.translate(-center_offset): center_offset is (0,0) in
                // Tier-1 (mesh pre-placed), so this is a no-op.

                // Clip the painted line against the EdgeGrid's bbox. MultiMaterialSegmentation.cpp:2293-2303.
                let edge_grid_bbox = *edge_grids[layer_idx].bbox();
                if !edge_grid_bbox.contains_point(&line_to_test.a)
                    || !edge_grid_bbox.contains_point(&line_to_test.b)
                {
                    // BoundingBox(Points{a, b}).overlap(edge_grid_bbox) — cpp:2300.
                    let line_bbox = BoundingBox::from_points(&[line_to_test.a, line_to_test.b]);
                    if !edge_grid_bbox.intersects(&line_bbox)
                        || !line_to_test.clip_with_bbox(&edge_grid_bbox)
                    {
                        continue;
                    }
                }

                // Run the painted-line visitor over the grid. MultiMaterialSegmentation.cpp:2305-2311.
                // (No mutex / `mutex_idx` — the Tier-1 port is sequential.)
                let grid = &edge_grids[layer_idx];
                let mut visitor = PaintedLineVisitor::new(grid, &mut painted_lines[layer_idx], 16);
                visitor.line_to_test = line_to_test;
                visitor.color = color;
                let a = line_to_test.a;
                let b = line_to_test.b;
                grid.visit_cells_intersecting_line(a, b, |iy, ix| visitor.visit(iy, ix));
            }
        }
    }

    // Per-layer segmentation. MultiMaterialSegmentation.cpp:2326-2366.
    for layer_idx in 0..num_layers {
        // cpp:2330
        if painted_lines[layer_idx].is_empty() {
            continue;
        }
        // cpp:2335 — post_process_painted_lines(std::move(painted_lines[layer_idx])).
        let taken = std::mem::take(&mut painted_lines[layer_idx]);
        let contours = edge_grids[layer_idx].contours();
        let post_processed = post_process_painted_lines(contours, taken);
        // cpp:2341
        let color_poly = colorize_contours(contours, &post_processed);
        // cpp:2347-2348
        debug_assert!(!color_poly.is_empty());
        debug_assert!(!color_poly.first().unwrap().is_empty());
        if color_poly.is_empty() || color_poly.first().map_or(true, |c| c.is_empty()) {
            continue;
        }

        if has_layer_only_one_color(&color_poly) {
            // cpp:2349-2351 — whole layer one color: assign the input directly to that slot.
            let one_color = color_poly.first().unwrap().first().unwrap().color as usize;
            segmented_regions[layer_idx][one_color] = input_expolygons[layer_idx].clone();
        } else {
            // cpp:2352-2357
            let mut graph = build_graph(layer_idx, &color_poly);
            remove_multiple_edges_in_vertices(&mut graph, &color_poly);
            graph.remove_nodes_with_one_arc();
            // extract_colored_segments returns polygon buckets (num_extruders + 1); union each
            // into an ExPolygons (the C++ extract does the union_ex internally).
            let poly_buckets = extract_colored_segments(&graph, num_extruders);
            segmented_regions[layer_idx] = poly_buckets
                .iter()
                .map(|polys| union_polygons_ex(polys))
                .collect();
        }
    }

    // Clip-back to actual geometry (cpp:2369-2383) is OMITTED — no negative volumes in Tier-1.

    // Interlocking / cut. MultiMaterialSegmentation.cpp:2385-2390.
    // Tier-1: `interlocking_beam` is always false; the gate reduces to `max_width > 0 ||
    // interlocking_depth > 0`. Both knobs arrive already-scaled, matching the C++
    // `float(scale_(...))` arguments.
    if segmented_max_width > 0.0 || segmented_interlocking_depth > 0.0 {
        cut_segmented_layers(
            &input_expolygons,
            &mut segmented_regions,
            segmented_max_width,
            segmented_interlocking_depth,
        );
    }

    // Top/bottom painted-surface propagation. MultiMaterialSegmentation.cpp:2393.
    // FIDELITY-NOTE (see doc-comment): stubbed to the empty "no top/bottom overrides" structure.
    // Shape expected by merge_segmented_layers is [extruder 0..=num_extruders][layer].
    let top_and_bottom_layers: Vec<Vec<ExPolygons>> =
        vec![vec![ExPolygons::new(); num_layers]; num_extruders + 1];

    // cpp:2396 — returns [layer][num_extruders] (0-based extruder; default color dropped).
    merge_segmented_layers(&segmented_regions, top_and_bottom_layers, num_extruders)
}

// ---------------------------------------------------------------------------
// Public API (MultiMaterialSegmentation.hpp:24-27)
// ---------------------------------------------------------------------------
//
// The Tier-1 orchestrator `multi_material_segmentation_by_painting_tier1` (above) ports the
// full pipeline of the PrintObject-driven `multi_material_segmentation_by_painting` against
// caller-prepared inputs (see its doc-comment). The original PrintObject-driven entry points
// remain BLOCKED for the reasons below.
//
// BLOCKED: `multi_material_segmentation_by_painting` (cpp:2012) and
// `fuzzy_skin_segmentation_by_painting`, plus their helpers
// `mmu_segmentation_top_and_bottom_layers` (cpp:1321) and `is_volume_sinking`
// (cpp:1311), require:
//   - model_object().volumes with ModelVolume::mmu_segmentation_facets /
//     fuzzy_skin_facets — per-volume facet annotations (EnforcerBlockerType) are not
//     stored on the Rust ModelVolume (see print_apply.rs notes), so there is no
//     painting data to segment;
//   - slice_mesh_slabs (TriangleMeshSlicer slab functions, deliberately excluded from
//     the Rust triangle_mesh_slicer port) to project painted facets onto layers;
//   - PrintObject trafo()/trafo_centered()/center_offset() composition for the
//     painted-mesh transform;
//   - PrintConfig::filament_colour (still missing from the Rust PrintConfig).
// The config-hierarchy wiring (print()->config()), the Layer/LayerRegion accessors,
// and the EdgeGrid surface (create / contours / cell_data_range /
// visit_cells_intersecting_line) ARE now available; everything those entry points call
// into that is self-contained is ported above (graph build/colorize,
// PaintedLineVisitor, post_process_painted_lines, colorize_contour(s),
// cut_segmented_layers, merge_segmented_layers). The remaining gap is the
// facet-annotation + slab-slicing infrastructure, which is documented and not faked.

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

// ---------------------------------------------------------------------------
// build_graph tests
// ---------------------------------------------------------------------------
//
// NOTE: the crate's `--lib` test target is pre-existingly broken (compile errors in
// unrelated modules), so these unit tests do not currently run under `cargo test --lib`.
// The equivalent coverage that DOES run lives in the integration test
// `tests/mms_build_graph.rs`.
#[cfg(test)]
mod build_graph_tests {
    use super::*;

    fn colored_line(a: Point, b: Point, color: i32, local_line_idx: i32) -> ColoredLine {
        let mut cl = ColoredLine::new(Line::new(a, b), color);
        cl.poly_idx = 0;
        cl.local_line_idx = local_line_idx;
        cl
    }

    // Closed CCW square, two adjacent sides colour 1 and two colour 2. The pipeline must
    // build a non-trivial graph and run to completion without panicking.
    #[test]
    fn build_graph_two_color_square() {
        let p0 = Point::new(-5_000_000, -5_000_000);
        let p1 = Point::new(5_000_000, -5_000_000);
        let p2 = Point::new(5_000_000, 5_000_000);
        let p3 = Point::new(-5_000_000, 5_000_000);

        let color_poly = vec![vec![
            colored_line(p0, p1, 1, 0),
            colored_line(p1, p2, 1, 1),
            colored_line(p2, p3, 2, 2),
            colored_line(p3, p0, 2, 3),
        ]];

        let mut graph = build_graph(0, &color_poly);
        assert_eq!(graph.all_border_points, 4);
        assert!(graph.nodes_count() >= 4);

        remove_multiple_edges_in_vertices(&mut graph, &color_poly);
        graph.remove_nodes_with_one_arc();

        let segments = extract_colored_segments(&graph, 2);
        assert_eq!(segments.len(), 3);

        let interior_arc = graph.arcs.iter().any(|a| a.r#type == ArcType::NonBorder);
        let any_segment = segments.iter().any(|bucket| !bucket.is_empty());
        assert!(interior_arc || any_segment);
    }
}
