//! EdgeGrid - Spatial acceleration structure for polygon edge queries.
//!
//! This module provides a grid-based spatial index for efficient queries on polygon edges,
//! including intersection testing, closest point queries, and signed distance calculations.
//!
//! # libslic3r Mapping
//!
//! This module corresponds to `EdgeGrid.cpp` and `EdgeGrid.hpp` in BambuStudio/libslic3r.
//!
//! # Key Features
//!
//! - Fast line-polygon intersection testing
//! - Closest point on polygon edge queries
//! - Signed distance field computation
//! - Support for both open polylines and closed polygons
//!
//! # Example
//!
//! ```ignore
//! use slicer::edge_grid::EdgeGrid;
//! use slicer::geometry::{Polygon, Point};
//!
//! let polygons = vec![Polygon::from_points(vec![
//!     Point::new(0, 0),
//!     Point::new(1000000, 0),
//!     Point::new(1000000, 1000000),
//!     Point::new(0, 1000000),
//! ])];
//!
//! let grid = EdgeGrid::from_polygons(&polygons, 100000); // 0.1mm resolution
//!
//! // Check if a line intersects any polygon edge
//! let intersects = grid.line_intersects_any(&Point::new(500000, -100000), &Point::new(500000, 500000));
//! ```

use crate::geometry::{BoundingBox, ExPolygon, Line, Point, Polygon, Polyline};

/// A contour represents a sequence of points forming either an open polyline or closed polygon
/// EdgeGrid.hpp:15-89
#[derive(Clone, Debug)]
pub struct Contour {
    /// Points of the contour
    points: Vec<Point>,
    /// Whether this contour is open (polyline) or closed (polygon)
    open: bool,
}

/// Implementation of Contour methods
/// EdgeGrid.hpp:15-89
impl Contour {
    // Create a new closed contour from points
    // EdgeGrid.hpp:17-18
    pub fn new_closed(points: Vec<Point>) -> Self {
        Self {
            points,
            open: false,
        }
    }

    /// Create a new open contour from points
    /// EdgeGrid.hpp:17-18
    pub fn new_open(points: Vec<Point>) -> Self {
        Self { points, open: true }
    }

    /// Create from a polygon (closed)
    /// EdgeGrid.hpp:20
    pub fn from_polygon(polygon: &Polygon) -> Self {
        Self::new_closed(polygon.points().to_vec())
    }

    /// Create from a polyline (open)
    /// EdgeGrid.hpp:20
    pub fn from_polyline(polyline: &Polyline) -> Self {
        Self::new_open(polyline.points().to_vec())
    }

    /// Returns true if this contour is open (polyline)
    /// EdgeGrid.hpp:24
    pub fn is_open(&self) -> bool {
        self.open
    }

    /// Returns true if this contour is closed (polygon)
    /// EdgeGrid.hpp:25
    pub fn is_closed(&self) -> bool {
        !self.open
    }

    /// Get the points of this contour
    /// EdgeGrid.hpp:22-23
    pub fn points(&self) -> &[Point] {
        &self.points
    }

    /// Get the number of segments in this contour
    /// EdgeGrid.hpp:64
    pub fn num_segments(&self) -> usize {
        // EdgeGrid.hpp:64 — size() - (m_open ? 1 : 0)
        if self.points.len() < 2 {
            return 0;
        }
        if self.open {
            self.points.len() - 1
        } else {
            self.points.len()
        }
    }

    /// Get the start point of a segment
    /// EdgeGrid.hpp:31-34
    pub fn segment_start(&self, idx: usize) -> &Point {
        &self.points[idx]
    }

    /// Get the end point of a segment, wrapping to first point for closed contours
    /// EdgeGrid.hpp:37-41
    pub fn segment_end(&self, idx: usize) -> &Point {
        // EdgeGrid.hpp:39-40 — const Point *ptr = m_begin + idx + 1; return ptr == m_end ? *m_begin : *ptr;
        let next_idx = if idx + 1 >= self.points.len() {
            0
        } else {
            idx + 1
        };
        &self.points[next_idx]
    }

    /// Start point of the segment preceding `idx` (closed contour wraps to the last point).
    /// EdgeGrid.hpp:44-49 — `return idx == 0 ? m_end[-1] : m_begin[idx - 1];`
    pub fn segment_prev(&self, idx: usize) -> &Point {
        if idx == 0 {
            &self.points[self.points.len() - 1]
        } else {
            &self.points[idx - 1]
        }
    }

    /// Index of a segment following `idx` (closed contour wraps to 0).
    /// EdgeGrid.hpp:57-62 — `++ idx; return m_begin + idx == m_end ? 0 : idx;`
    pub fn segment_idx_next(&self, idx: usize) -> usize {
        let idx = idx + 1;
        if idx == self.points.len() {
            0
        } else {
            idx
        }
    }

    /// Get a segment as a Line
    /// EdgeGrid.hpp:66-70
    pub fn segment(&self, idx: usize) -> Line {
        Line::new(*self.segment_start(idx), *self.segment_end(idx))
    }

    /// Get all segments as Lines
    /// EdgeGrid.hpp:72-81
    pub fn segments(&self) -> Vec<Line> {
        // EdgeGrid.hpp:72-81 — only emit if num_segments() > 2; iterate begin..end-1
        // plus the closing segment for non-open contours.
        let mut lines: Vec<Line> = Vec::with_capacity(self.num_segments());
        if self.num_segments() > 2 {
            for i in 0..self.points.len() - 1 {
                lines.push(Line::new(self.points[i], self.points[i + 1]));
            }
            if !self.open {
                lines.push(Line::new(
                    self.points[self.points.len() - 1],
                    self.points[0],
                ));
            }
        }
        lines
    }
}

/// A cell in the edge grid, stores range into cell_data array
/// EdgeGrid.hpp:410-414
#[derive(Clone, Copy, Debug, Default)]
struct Cell {
    /// Start index in the cell_data array
    begin: usize,
    /// End index in the cell_data array (exclusive)
    end: usize,
}

/// Implementation of Cell methods
/// EdgeGrid.hpp:410-414
impl Cell {
    // Check if cell is empty (no segments)
    // EdgeGrid.hpp:413
    fn is_empty(&self) -> bool {
        self.begin >= self.end
    }
}

/// Result of a closest point query on the grid
/// EdgeGrid.hpp:139-148
#[derive(Clone, Debug)]
pub struct ClosestPointResult {
    /// Index of the contour
    pub contour_idx: usize,
    /// Index of the segment start point
    pub start_point_idx: usize,
    /// Signed distance to the closest point
    pub distance: f64,
    /// Parameter t on the segment [0, 1)
    pub t: f64,
    /// The closest point itself
    pub point: Point,
}

/// ClosestPointResult methods
/// EdgeGrid.hpp:139-148
impl ClosestPointResult {
    // Create an invalid result with sentinel values
    // EdgeGrid.hpp:140-145
    pub fn invalid() -> Self {
        Self {
            contour_idx: usize::MAX,
            start_point_idx: usize::MAX,
            distance: f64::MAX,
            t: 0.0,
            point: Point::new(0, 0),
        }
    }

    /// Check if this result is valid (contour_idx not sentinel)
    /// EdgeGrid.hpp:147
    pub fn is_valid(&self) -> bool {
        self.contour_idx != usize::MAX
    }
}

/// Intersection result for line-edge queries
/// No direct C++ equivalent — Rust-specific convenience struct
#[derive(Clone, Debug)]
pub struct Intersection {
    /// Index of the contour (boundary polygon)
    pub contour_idx: usize,
    /// Index of the segment within the contour
    pub segment_idx: usize,
    /// The intersection point
    pub point: Point,
    /// Distance along the original line from start
    pub distance: f64,
}

/// EdgeGrid - A spatial acceleration structure for polygon edges
/// EdgeGrid.hpp:91-454
///
/// The grid divides the bounding box into cells and stores which polygon edges
/// pass through each cell, enabling fast spatial queries.
#[derive(Clone, Debug)]
pub struct EdgeGrid {
    /// Bounding box of the grid
    bbox: BoundingBox,
    /// Resolution (cell size) in scaled coordinates
    resolution: i64,
    /// Number of rows in the grid
    rows: usize,
    /// Number of columns in the grid
    cols: usize,
    /// Contours stored in the grid
    contours: Vec<Contour>,
    /// Cell data: (contour_idx, segment_idx) pairs
    cell_data: Vec<(usize, usize)>,
    /// Cells indexing into cell_data
    cells: Vec<Cell>,
    /// Pre-computed signed distance field (optional)
    signed_distance_field: Vec<f32>,
}

/// EdgeGrid methods
/// EdgeGrid.hpp:91-454
impl EdgeGrid {
    // Create a new empty EdgeGrid
    // EdgeGrid.hpp:94
    pub fn new() -> Self {
        Self {
            bbox: BoundingBox::new(),
            resolution: 1,
            rows: 0,
            cols: 0,
            contours: Vec::new(),
            cell_data: Vec::new(),
            cells: Vec::new(),
            signed_distance_field: Vec::new(),
        }
    }

    /// Create an EdgeGrid from polygons with the given resolution
    /// EdgeGrid.cpp:28-38
    pub fn from_polygons(polygons: &[Polygon], resolution: i64) -> Self {
        // EdgeGrid.cpp:28-38 — create(polygons, resolution)
        let mut grid = Self::new();
        grid.create_from_polygons(polygons, resolution);
        grid
    }

    /// Create an EdgeGrid from a single polygon
    /// No direct C++ equivalent — Rust convenience wrapper
    pub fn from_polygon(polygon: &Polygon, resolution: i64) -> Self {
        Self::from_polygons(&[polygon.clone()], resolution)
    }

    /// Set the bounding box
    /// EdgeGrid.hpp:97
    pub fn set_bbox(&mut self, bbox: BoundingBox) {
        self.bbox = bbox;
    }

    /// Get the bounding box
    /// EdgeGrid.hpp:159
    pub fn bbox(&self) -> &BoundingBox {
        &self.bbox
    }

    /// Get the resolution
    /// EdgeGrid.hpp:160
    pub fn resolution(&self) -> i64 {
        self.resolution
    }

    /// Get the number of rows
    /// EdgeGrid.hpp:161
    pub fn rows(&self) -> usize {
        self.rows
    }

    /// Get the number of columns
    /// EdgeGrid.hpp:162
    pub fn cols(&self) -> usize {
        self.cols
    }

    /// Get the contours
    /// EdgeGrid.hpp:115
    pub fn contours(&self) -> &[Contour] {
        &self.contours
    }

    /// Return the (contour_idx, segment_idx) entries stored in the cell at (row, col).
    /// Mirrors the C++ `cell_data_range`, which returns a pair of iterators over the
    /// cell's slice of `m_cell_data`.
    /// EdgeGrid.hpp:387-393
    pub fn cell_data_range_at(&self, row: usize, col: usize) -> &[(usize, usize)] {
        // EdgeGrid.hpp:391-392
        self.cell_data_range(row, col)
    }

    /// Return the segment (start, end) referenced by a (contour_idx, segment_idx) pair.
    /// EdgeGrid.hpp:395-400
    pub fn segment(&self, contour_and_segment_idx: (usize, usize)) -> Line {
        // EdgeGrid.hpp:397-399
        let contour = &self.contours[contour_and_segment_idx.0];
        let iseg = contour_and_segment_idx.1;
        Line::new(*contour.segment_start(iseg), *contour.segment_end(iseg))
    }

    /// Whether any two stored edges intersect (other than at a shared endpoint of
    /// two consecutive segments on the same contour).
    ///
    /// EdgeGrid.cpp:1452-1480 `bool EdgeGrid::Grid::has_intersecting_edges() const`.
    /// The C++ skips the adjacency case `&icontour == &jcontour && (&ip1 == &jp2
    /// || &jp1 == &ip2)` via pointer identity of the shared vertex; here the grid
    /// builder de-duplicates vertices within a contour, so comparing the same
    /// contour index plus point-value equality of the shared endpoint is
    /// equivalent (segment i's start == segment j's end, or vice versa).
    pub fn has_intersecting_edges(&self) -> bool {
        // EdgeGrid.cpp:1454-1455 — for each cell:
        for r in 0..self.rows {
            for c in 0..self.cols {
                // EdgeGrid.cpp:1457
                let cell = self.cells[r * self.cols + c];
                // EdgeGrid.cpp:1459 — for each pair of segments in the cell:
                for i in cell.begin..cell.end {
                    // EdgeGrid.cpp:1460-1464
                    let (ic, ipt) = self.cell_data[i];
                    let icontour = &self.contours[ic];
                    let ip1 = *icontour.segment_start(ipt);
                    let ip2 = *icontour.segment_end(ipt);
                    // EdgeGrid.cpp:1465
                    for j in (i + 1)..cell.end {
                        // EdgeGrid.cpp:1466-1470
                        let (jc, jpt) = self.cell_data[j];
                        let jcontour = &self.contours[jc];
                        let jp1 = *jcontour.segment_start(jpt);
                        let jp2 = *jcontour.segment_end(jpt);
                        // EdgeGrid.cpp:1471-1473 — native skips adjacency by
                        // POINTER equality (&ip1 == &jp2 || &jp1 == &ip2), i.e.
                        // the same point SLOT, not the same value. Two segments
                        // that merely touch at equal coordinates DO count as
                        // intersecting (R799 — value-compare here made
                        // has_intersecting_edges inert on arachne loops: rust
                        // 0/11212 self-intersections vs native 8545/11195).
                        let end_slot = |cont: &Contour, k: usize| -> usize {
                            if k + 1 >= cont.points().len() { 0 } else { k + 1 }
                        };
                        let adjacent = if crate::faithful_gate("FVS_SELFX") {
                            ic == jc
                                && (ipt == end_slot(jcontour, jpt)
                                    || jpt == end_slot(icontour, ipt))
                        } else {
                            ic == jc && (ip1 == jp2 || jp1 == ip2)
                        };
                        if !adjacent
                            && crate::geometry::segments_intersect(ip1, ip2, jp1, jp2)
                        {
                            return true;
                        }
                    }
                }
            }
        }
        // EdgeGrid.cpp:1479
        false
    }

    /// Visit all grid cells intersected by the line segment (p1, p2), calling
    /// `visitor(iy, ix)` for each. The visitor returns `false` to stop early.
    /// EdgeGrid.hpp:291-366 with `need_consider_eps = false` (the PolygonTrimmer
    /// call site never sets it).
    pub fn visit_cells_intersecting_line<F>(&self, p1: Point, p2: Point, visitor: F)
    where
        F: FnMut(usize, usize) -> bool,
    {
        // EdgeGrid.hpp:360-365 — single start/end pair when need_consider_eps is false.
        self.visit_cells_for_segment(&p1, &p2, visitor);
    }

    /// `visit_cells_intersecting_line` with C++'s `need_consider_eps = true`
    /// (EdgeGrid.hpp:317-359). When an endpoint sits within `eps` of a cell
    /// boundary, the traversal is ALSO run from the neighbouring cell(s), so a
    /// segment grazing a cell border still finds the contour segments stored
    /// there. MultiMaterialSegmentation.cpp:2254 and :2492 are the only callers
    /// that pass true, and both are painted-triangle projection: without this,
    /// painted lines whose endpoints land on a cell boundary are silently
    /// dropped, which costs contour colour coverage (R454).
    ///
    /// C++ builds up to 5 start and 5 end positions and runs the traversal for
    /// every (start, end) pair, so a cell may be visited more than once — the
    /// visitors here are idempotent per cell, matching native.
    pub fn visit_cells_intersecting_line_eps<F>(&self, p1: Point, p2: Point, mut visitor: F)
    where
        F: FnMut(usize, usize) -> bool,
    {
        // EdgeGrid.hpp:295-296 — coordinates are relative to the grid bbox.
        if self.cols == 0 || self.rows == 0 || self.resolution <= 0 {
            return;
        }
        let q1 = Point::new(p1.x - self.bbox.min.x, p1.y - self.bbox.min.y);
        let q2 = Point::new(p2.x - self.bbox.min.x, p2.y - self.bbox.min.y);
        let res = self.resolution;
        let ix = q1.x / res;
        let iy = q1.y / res;
        let ixb = q2.x / res;
        let iyb = q2.y / res;

        // EdgeGrid.hpp:326 — const double eps = scale_(10 * EPSILON).
        let eps = (10.0 * crate::libslic3r::EPSILON * crate::SCALING_FACTOR) as i64;

        // (cell_x, cell_y, point) triples. C++ pushes the unmodified endpoint first.
        let mut start_pos: Vec<(i64, i64, Point)> = vec![(ix, iy, q1)];
        let mut end_pos: Vec<(i64, i64, Point)> = vec![(ixb, iyb, q2)];
        // EdgeGrid.hpp:318-323 — note C++ perturbs ONE axis at a time and keeps the
        // other axis' ORIGINAL cell index.
        let mut push_variants = |v: &mut Vec<(i64, i64, Point)>, p: Point, cx: i64, cy: i64| {
            let xu = (p.x + eps) / res;
            if xu != cx {
                v.push((xu, cy, Point::new(p.x + eps, p.y)));
            }
            let xl = (p.x - eps) / res;
            if xl != cx {
                v.push((xl, cy, Point::new(p.x - eps, p.y)));
            }
            let yu = (p.y + eps) / res;
            if yu != cy {
                v.push((cx, yu, Point::new(p.x, p.y + eps)));
            }
            let yl = (p.y - eps) / res;
            if yl != cy {
                v.push((cx, yl, Point::new(p.x, p.y - eps)));
            }
        };
        push_variants(&mut start_pos, q1, ix, iy);
        push_variants(&mut end_pos, q2, ixb, iyb);

        // EdgeGrid.hpp:360-365 — every (start, end) combination.
        for &(sx, sy, sp) in &start_pos {
            for &(ex, ey, ep) in &end_pos {
                Self::rasterize_segment(res, sx, sy, &sp, ex, ey, &ep, &mut visitor);
            }
        }
    }

    /// Create the grid from polygons (all closed). Skips empty polygons.
    /// EdgeGrid.cpp:28-38
    pub fn create_from_polygons(&mut self, polygons: &[Polygon], resolution: i64) {
        // EdgeGrid.cpp:33-35 — only non-empty polygons, all closed.
        self.contours = polygons
            .iter()
            .filter(|p| !p.points().is_empty())
            .map(Contour::from_polygon)
            .collect();
        self.create_from_contours(resolution);
    }

    /// Create the grid from polylines (open by default). Mirrors
    /// `create(std::vector<Points>, resolution, open_polylines=true)`: a polyline
    /// whose first point equals its last point is treated as closed and the
    /// repeated last point is dropped.
    /// EdgeGrid.cpp:52-75
    pub fn create_from_polylines(&mut self, polylines: &[Polyline], resolution: i64) {
        self.create_from_polylines_flag(polylines, resolution, true);
    }

    /// Native `create(const Polylines&, coord_t, bool open)` (EdgeGrid.cpp:50-75):
    /// `open == false` stores each polyline as a CLOSED contour with its points
    /// AS-IS — a duplicated first==last point keeps a degenerate closing edge,
    /// exactly like native (feeds the pointer-adjacency semantics above).
    pub fn create_from_polylines_flag(
        &mut self,
        polylines: &[Polyline],
        resolution: i64,
        open: bool,
    ) {
        self.contours = Vec::new();
        for polyline in polylines {
            let pts = polyline.points();
            // EdgeGrid.cpp:58 — only points with size > 1.
            if pts.len() > 1 {
                if open {
                    self.contours.push(Self::contour_from_open_points(pts));
                } else {
                    self.contours.push(Contour::new_closed(pts.to_vec()));
                }
            }
        }
        self.create_from_contours(resolution);
    }

    /// Build a contour from a polyline's points using the C++ open-polyline rule.
    /// EdgeGrid.cpp:59-71
    fn contour_from_open_points(pts: &[Point]) -> Contour {
        // open = open_polylines (true); if *begin == end[-1], close and drop last point.
        if pts[0] == pts[pts.len() - 1] {
            Contour::new_closed(pts[..pts.len() - 1].to_vec())
        } else {
            Contour::new_open(pts.to_vec())
        }
    }

    /// Create the grid from both polygons and polylines.
    /// Mirrors `create(const Polygons&, const Polylines&, coord_t)`: polylines are
    /// inserted first (with open/closed detection), then polygons (always closed).
    /// EdgeGrid.cpp:77-102
    pub fn create_from_mixed(
        &mut self,
        polygons: &[Polygon],
        polylines: &[Polyline],
        resolution: i64,
    ) {
        self.contours = Vec::new();
        // EdgeGrid.cpp:85-95 — polylines first.
        for polyline in polylines {
            let pts = polyline.points();
            if pts.len() > 1 {
                self.contours.push(Self::contour_from_open_points(pts));
            }
        }
        // EdgeGrid.cpp:97-99 — then polygons (closed).
        for polygon in polygons {
            if polygon.points().len() > 1 {
                self.contours
                    .push(Contour::new_closed(polygon.points().to_vec()));
            }
        }
        self.create_from_contours(resolution);
    }

    /// Create the grid from an ExPolygon (outer contour + holes, all closed).
    /// EdgeGrid.cpp:104-115
    pub fn create_from_expolygon(&mut self, expoly: &ExPolygon, resolution: i64) {
        // EdgeGrid.cpp:106-112
        self.contours = Vec::new();
        if !expoly.contour.points().is_empty() {
            self.contours.push(Contour::new_closed(expoly.contour.points().to_vec()));
        }
        for hole in &expoly.holes {
            if !hole.points().is_empty() {
                self.contours.push(Contour::new_closed(hole.points().to_vec()));
            }
        }
        // EdgeGrid.cpp:114
        self.create_from_contours(resolution);
    }

    /// Visit all grid cells intersected by the bounding box `bbox`, calling
    /// `visitor(iy, ix)` for each. The visitor returns `false` to stop early.
    /// EdgeGrid.hpp:368-385
    pub fn visit_cells_intersecting_box<F>(&self, bbox: BoundingBox, mut visitor: F)
    where
        F: FnMut(usize, usize) -> bool,
    {
        // EdgeGrid.hpp:371-372 — End points of the line segment.
        let mut bmin = Point::new(bbox.min.x - self.bbox.min.x, bbox.min.y - self.bbox.min.y);
        let mut bmax = Point::new(
            bbox.max.x - (self.bbox.min.x + 1),
            bbox.max.y - (self.bbox.min.y + 1),
        );
        // EdgeGrid.hpp:374-375 — Get the cells of the end points.
        bmin.x /= self.resolution;
        bmin.y /= self.resolution;
        bmax.x /= self.resolution;
        bmax.y /= self.resolution;
        // EdgeGrid.hpp:377-380 — Trim with the cells.
        bmin.x = bmin.x.max(0);
        bmin.y = bmin.y.max(0);
        bmax.x = bmax.x.min(self.cols as i64 - 1);
        bmax.y = bmax.y.min(self.rows as i64 - 1);
        // EdgeGrid.hpp:381-384
        let mut iy = bmin.y;
        while iy <= bmax.y {
            let mut ix = bmin.x;
            while ix <= bmax.x {
                if !visitor(iy as usize, ix as usize) {
                    return;
                }
                ix += 1;
            }
            iy += 1;
        }
    }

    /// Internal: create the grid from stored contours
    /// EdgeGrid.cpp:142-334
    fn create_from_contours(&mut self, resolution: i64) {
        // EdgeGrid.cpp:145 — m_resolution = resolution
        self.resolution = resolution.max(1);

        // EdgeGrid.cpp:145-151 — measure the bounding box by MERGING the contour
        // points into whatever `m_bbox` already holds. C++ deliberately does NOT
        // reset it here: callers such as MultiMaterialSegmentation.cpp:2216/2476
        // call `set_bbox(bbox)` first with the merged ADJACENT-LAYER bbox, so the
        // grid ends up covering the union of that and the contours.
        //
        // R447: this port used to do `self.bbox = BoundingBox::new()` first, which
        // silently discarded the pre-set bbox and produced a grid tightly clipped
        // to the contours (+16 eps). A painted facet lying on the object's own
        // silhouette can then project to a line a few scaled units OUTSIDE that
        // tighter box and be dropped by the clip at MMS ~2925 — measured on
        // painted_cube with THREEMF_NO_CENTER=1: every extruder-2 line discarded,
        // 50 tool changes became 0 (R446).
        for contour in &self.contours {
            for point in contour.points() {
                self.bbox.merge_point(*point);
            }
        }

        // EdgeGrid.cpp:155-160 — bail out if bbox is empty
        if self.bbox.is_empty() {
            self.rows = 0;
            self.cols = 0;
            self.cells.clear();
            self.cell_data.clear();
            return;
        }

        // EdgeGrid.cpp:153-157 — add a fixed eps margin of 16 to the bbox.
        let eps: i64 = 16;
        self.bbox = BoundingBox::from_points_minmax(
            Point::new(self.bbox.min.x - eps, self.bbox.min.y - eps),
            Point::new(self.bbox.max.x + eps, self.bbox.max.y + eps),
        );

        // EdgeGrid.cpp:161-162 — calculate grid dimensions
        //   m_cols = (max(0) - min(0) + m_resolution - 1) / m_resolution
        //   m_rows = (max(1) - min(1) + m_resolution - 1) / m_resolution
        self.cols =
            ((self.bbox.max.x - self.bbox.min.x + self.resolution - 1) / self.resolution) as usize;
        self.rows =
            ((self.bbox.max.y - self.bbox.min.y + self.resolution - 1) / self.resolution) as usize;

        // EdgeGrid.cpp:174-190 — count edges per cell (first pass)
        let num_cells = self.rows * self.cols;
        let mut cell_counts = vec![0usize; num_cells];

        for (contour_idx, contour) in self.contours.iter().enumerate() {
            for seg_idx in 0..contour.num_segments() {
                // EdgeGrid.cpp:180-188 — visit cells for each segment
                let p1 = contour.segment_start(seg_idx);
                let p2 = contour.segment_end(seg_idx);
                self.visit_cells_for_segment(p1, p2, |row, col| {
                    if row < self.rows && col < self.cols {
                        let cell_idx = row * self.cols + col;
                        if cell_idx < num_cells {
                            cell_counts[cell_idx] += 1;
                        }
                    }
                    true
                });
                // Mark we processed this segment (for borrow checker)
                let _ = (contour_idx, seg_idx);
            }
        }

        // EdgeGrid.cpp:192-200 — build cell offsets from counts
        self.cells = vec![Cell::default(); num_cells];
        let mut offset = 0;
        for (i, count) in cell_counts.iter().enumerate() {
            self.cells[i].begin = offset;
            self.cells[i].end = offset;
            offset += count;
        }

        // EdgeGrid.cpp:202-204 — allocate cell data array
        self.cell_data = vec![(0, 0); offset];

        // EdgeGrid.cpp:206-330 — collect and fill segment data (second pass)
        let segment_data: Vec<(usize, usize, Point, Point)> = self
            .contours
            .iter()
            .enumerate()
            .flat_map(|(contour_idx, contour)| {
                (0..contour.num_segments()).map(move |seg_idx| {
                    let p1 = *contour.segment_start(seg_idx);
                    let p2 = *contour.segment_end(seg_idx);
                    (contour_idx, seg_idx, p1, p2)
                })
            })
            .collect();

        // EdgeGrid.cpp:310-330 — fill cell_data with segment references
        for (contour_idx, seg_idx, p1, p2) in segment_data {
            self.visit_cells_for_segment_mut(&p1, &p2, contour_idx, seg_idx);
        }
    }

    /// Convert a point to cell coordinates (row, col), clamped to the grid.
    /// EdgeGrid.cpp:177-184
    #[allow(dead_code)]
    fn point_to_cell(&self, point: &Point) -> (usize, usize) {
        // Compute column index from x offset divided by resolution
        // EdgeGrid.cpp:177-178
        let x = ((point.x - self.bbox.min.x) / self.resolution).max(0) as usize;
        // Compute row index from y offset divided by resolution
        // EdgeGrid.cpp:179-180
        let y = ((point.y - self.bbox.min.y) / self.resolution).max(0) as usize;
        // Clamp to valid grid bounds
        // EdgeGrid.cpp:181-184
        (
            y.min(self.rows.saturating_sub(1)),
            x.min(self.cols.saturating_sub(1)),
        )
    }

    /// Visit all grid cells intersected by the line segment (p1, p2) in source
    /// coordinates, calling `visitor(iy, ix)` (row, col) for each. The visitor
    /// returns `false` to stop early. Mirrors `visit_cells_intersecting_line`
    /// without the `need_consider_eps` extension (no call site sets it).
    /// EdgeGrid.hpp:291-366
    fn visit_cells_for_segment<F>(&self, p1: &Point, p2: &Point, mut visitor: F)
    where
        F: FnMut(usize, usize) -> bool,
    {
        // Early return if grid is empty.
        if self.cols == 0 || self.rows == 0 {
            return;
        }
        // EdgeGrid.hpp:296-297 — translate the end points by -m_bbox.min.
        let p1 = Point::new(p1.x - self.bbox.min.x, p1.y - self.bbox.min.y);
        let p2 = Point::new(p2.x - self.bbox.min.x, p2.y - self.bbox.min.y);
        // EdgeGrid.hpp:303-306 — get the cells of the end points.
        let ix = p1.x / self.resolution;
        let iy = p1.y / self.resolution;
        let ixb = p2.x / self.resolution;
        let iyb = p2.y / self.resolution;
        Self::rasterize_segment(self.resolution, ix, iy, &p1, ixb, iyb, &p2, &mut visitor);
    }

    /// Visit cells along a segment and insert segment reference into cell_data (mutable fill pass).
    /// Mirrors the C++ Visitor that does `cell_data[cells[iy*cols+ix].end++] = (i, j)`.
    /// EdgeGrid.cpp:311-333
    fn visit_cells_for_segment_mut(
        &mut self,
        p1: &Point,
        p2: &Point,
        contour_idx: usize,
        seg_idx: usize,
    ) {
        // Early return if grid is empty.
        if self.cols == 0 || self.rows == 0 {
            return;
        }
        let cols = self.cols;
        let rows = self.rows;
        let cell_data_len = self.cell_data.len();
        // Borrow split: pass mutable cells/cell_data into a local visitor.
        let cells = &mut self.cells;
        let cell_data = &mut self.cell_data;
        // EdgeGrid.hpp:296-306 — translate, get end-point cells.
        let p1t = Point::new(p1.x - self.bbox.min.x, p1.y - self.bbox.min.y);
        let p2t = Point::new(p2.x - self.bbox.min.x, p2.y - self.bbox.min.y);
        let ix = p1t.x / self.resolution;
        let iy = p1t.y / self.resolution;
        let ixb = p2t.x / self.resolution;
        let iyb = p2t.y / self.resolution;
        // Re-inline the rasterizer here because it needs &self.bbox/resolution while we
        // hold mutable borrows of cells/cell_data. Use a captured closure as the visitor.
        let m_resolution = self.resolution;
        let mut visitor = |row: usize, col: usize| -> bool {
            // EdgeGrid.cpp:316 — cell_data[cells[iy*cols + ix].end++] = (i, j)
            if col < cols && row < rows {
                let cell_idx = row * cols + col;
                let insert_idx = cells[cell_idx].end;
                if insert_idx < cell_data_len {
                    cell_data[insert_idx] = (contour_idx, seg_idx);
                    cells[cell_idx].end += 1;
                }
            }
            true
        };
        Self::rasterize_segment(m_resolution, ix, iy, &p1t, ixb, iyb, &p2t, &mut visitor);
    }

    /// Free-function form of `visit_intersect_line_impl` so it can be used while
    /// `self.cells`/`self.cell_data` are mutably borrowed.
    /// The C++ edge-crossing accumulators (`ex`/`ey`) are explicitly `int64_t`
    /// (EdgeGrid.hpp:183 etc.); Coord = i64 (F2) matches that width, so the
    /// products do not truncate.
    /// EdgeGrid.hpp:172-289
    #[allow(clippy::too_many_arguments)]
    fn rasterize_segment<F>(
        m_resolution: i64,
        mut ix: i64,
        mut iy: i64,
        p1: &Point,
        ixb: i64,
        iyb: i64,
        p2: &Point,
        visitor: &mut F,
    ) where
        F: FnMut(usize, usize) -> bool,
    {
        // Account for the end points.
        if !visitor(iy as usize, ix as usize) || (ix == ixb && iy == iyb) {
            return;
        }
        let dx = (p2.x - p1.x).abs();
        let dy = (p2.y - p1.y).abs();
        if p1.x < p2.x {
            let mut ex = ((ix + 1) * m_resolution - p1.x) * dy;
            if p1.y < p2.y {
                let mut ey = ((iy + 1) * m_resolution - p1.y) * dx;
                loop {
                    if ex < ey {
                        ey -= ex;
                        ex = dy * m_resolution;
                        ix += 1;
                    } else if ex == ey {
                        ex = dy * m_resolution;
                        ey = dx * m_resolution;
                        ix += 1;
                        iy += 1;
                    } else {
                        ex -= ey;
                        ey = dx * m_resolution;
                        iy += 1;
                    }
                    if !visitor(iy as usize, ix as usize) {
                        return;
                    }
                    if !(ix != ixb || iy != iyb) {
                        break;
                    }
                }
            } else {
                let mut ey = (p1.y - iy * m_resolution) * dx;
                loop {
                    if ex <= ey {
                        ey -= ex;
                        ex = dy * m_resolution;
                        ix += 1;
                    } else {
                        ex -= ey;
                        ey = dx * m_resolution;
                        iy -= 1;
                    }
                    if !visitor(iy as usize, ix as usize) {
                        return;
                    }
                    if !(ix != ixb || iy != iyb) {
                        break;
                    }
                }
            }
        } else {
            let mut ex = (p1.x - ix * m_resolution) * dy;
            if p1.y < p2.y {
                let mut ey = ((iy + 1) * m_resolution - p1.y) * dx;
                loop {
                    if ex < ey {
                        ey -= ex;
                        ex = dy * m_resolution;
                        ix -= 1;
                    } else {
                        ex -= ey;
                        ey = dx * m_resolution;
                        iy += 1;
                    }
                    if !visitor(iy as usize, ix as usize) {
                        return;
                    }
                    if !(ix != ixb || iy != iyb) {
                        break;
                    }
                }
            } else {
                let mut ey = (p1.y - iy * m_resolution) * dx;
                loop {
                    if ex < ey {
                        ey -= ex;
                        ex = dy * m_resolution;
                        ix -= 1;
                    } else if ex == ey {
                        if dx > 0 {
                            ex = dy * m_resolution;
                            ix -= 1;
                        }
                        if dy > 0 {
                            ey = dx * m_resolution;
                            iy -= 1;
                        }
                    } else {
                        ex -= ey;
                        ey = dx * m_resolution;
                        iy -= 1;
                    }
                    if !visitor(iy as usize, ix as usize) {
                        return;
                    }
                    if !(ix != ixb || iy != iyb) {
                        break;
                    }
                }
            }
        }
    }

    /// Return the slice of cell_data entries for a given cell at (row, col).
    /// EdgeGrid.hpp:387-393
    fn cell_data_range(&self, row: usize, col: usize) -> &[(usize, usize)] {
        // Bounds check on row and col
        // EdgeGrid.hpp:389-390
        if row >= self.rows || col >= self.cols {
            return &[];
        }
        // Compute linear cell index
        // EdgeGrid.hpp:391
        let cell_idx = row * self.cols + col;
        if cell_idx >= self.cells.len() {
            return &[];
        }
        // Return the slice from cell.begin to cell.end
        // EdgeGrid.hpp:392
        let cell = &self.cells[cell_idx];
        if cell.begin >= cell.end || cell.end > self.cell_data.len() {
            return &[];
        }
        &self.cell_data[cell.begin..cell.end]
    }

    /// Check if a line segment intersects any edge stored in the grid.
    /// EdgeGrid.hpp:291-366
    pub fn line_intersects_any(&self, p1: &Point, p2: &Point) -> bool {
        // Early return if grid is empty
        // EdgeGrid.hpp:294-295
        if self.cols == 0 || self.rows == 0 {
            return false;
        }

        // Create line from the two endpoints
        // EdgeGrid.hpp:296
        let line = Line::new(*p1, *p2);
        let mut found = false;

        // Visit all cells along the segment and test edges in each cell
        // EdgeGrid.hpp:291-366
        self.visit_cells_for_segment(p1, p2, |row, col| {
            // Test each edge in this cell against the query line
            // EdgeGrid.hpp:300-365
            for &(contour_idx, seg_idx) in self.cell_data_range(row, col) {
                let segment = self.contours[contour_idx].segment(seg_idx);
                if line.intersects(&segment) {
                    found = true;
                    return false; // stop traversal
                }
            }
            true
        });

        found
    }

    /// Find all intersection points between a query line and edges stored in the grid.
    /// EdgeGrid.hpp:291-366
    pub fn find_intersections(&self, p1: &Point, p2: &Point) -> Vec<Intersection> {
        // Initialize results and deduplication set
        // No direct C++ equivalent — Rust-specific
        let mut intersections = Vec::new();
        let mut seen = std::collections::HashSet::new();

        // Early return if grid is empty
        // EdgeGrid.hpp:294-295
        if self.cols == 0 || self.rows == 0 {
            return intersections;
        }

        // Create line and compute direction vector
        // EdgeGrid.hpp:296
        let line = Line::new(*p1, *p2);
        let line_vec = (p2.x - p1.x, p2.y - p1.y);

        // Visit all cells along the segment and collect intersections
        // EdgeGrid.hpp:291-366
        self.visit_cells_for_segment(p1, p2, |row, col| {
            for &(contour_idx, seg_idx) in self.cell_data_range(row, col) {
                // Skip already-tested segments (cell overlap deduplication)
                // No direct C++ equivalent — Rust-specific
                if !seen.insert((contour_idx, seg_idx)) {
                    continue;
                }

                // Test intersection with this edge segment
                // EdgeGrid.hpp:300-365
                let segment = self.contours[contour_idx].segment(seg_idx);
                if let Some(intersection_point) = line.intersection(&segment) {
                    // Calculate parametric distance along the query line
                    // EdgeGrid.hpp:300-365
                    let dx = intersection_point.x - p1.x;
                    let dy = intersection_point.y - p1.y;
                    let distance = if line_vec.0.abs() > line_vec.1.abs() {
                        dx as f64 / line_vec.0 as f64
                    } else if line_vec.1 != 0 {
                        dy as f64 / line_vec.1 as f64
                    } else {
                        0.0
                    };

                    // Store the intersection result
                    // EdgeGrid.hpp:300-365
                    intersections.push(Intersection {
                        contour_idx,
                        segment_idx: seg_idx,
                        point: intersection_point,
                        distance,
                    });
                }
            }
            true
        });

        // Sort intersections by distance along the query line
        // No direct C++ equivalent — Rust-specific
        intersections.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());

        intersections
    }

    /// Calculate a signed distance to the contours in `search_radius` from `pt`.
    /// Faithful port of `closest_point_signed_distance`. Only call for closed contours.
    /// EdgeGrid.cpp:1047-1176
    pub fn closest_point(&self, query: &Point, search_radius: i64) -> ClosestPointResult {
        let pt = *query;
        let mut result = ClosestPointResult::invalid();

        // EdgeGrid.cpp:1049-1051 — bbox starts at pt - m_bbox.min.
        // bbox.min/max in grid-local coordinates.
        // EdgeGrid.cpp:1052-1063 — upper boundary, round to grid and test validity.
        let mut bmax_x = pt.x - self.bbox.min.x + search_radius;
        let mut bmax_y = pt.y - self.bbox.min.y + search_radius;
        if bmax_x < 0 || bmax_y < 0 {
            return result;
        }
        bmax_x /= self.resolution;
        bmax_y /= self.resolution;
        if bmax_x >= self.cols as i64 {
            bmax_x = self.cols as i64 - 1;
        }
        if bmax_y >= self.rows as i64 {
            bmax_y = self.rows as i64 - 1;
        }
        // EdgeGrid.cpp:1064-1072 — lower boundary, round to grid and test validity.
        let mut bmin_x = pt.x - self.bbox.min.x - search_radius;
        let mut bmin_y = pt.y - self.bbox.min.y - search_radius;
        if bmin_x < 0 {
            bmin_x = 0;
        }
        if bmin_y < 0 {
            bmin_y = 0;
        }
        bmin_x /= self.resolution;
        bmin_y /= self.resolution;
        // EdgeGrid.cpp:1073-1076 — is the interval empty?
        if bmin_x > bmax_x || bmin_y > bmax_y {
            return result;
        }

        // EdgeGrid.cpp:1078-1081 — traverse all cells in the bounding box.
        let mut d_min = search_radius as f64;
        // Signum of the distance field at pt.
        let mut sign_min: i32 = 0;
        let mut l2_seg_min: f64 = 1.0;

        // EdgeGrid.cpp:1082-1153
        for r in bmin_y..=bmax_y {
            for c in bmin_x..=bmax_x {
                for &(contour_idx, ipt) in self.cell_data_range(r as usize, c as usize) {
                    let contour = &self.contours[contour_idx];
                    // End points of the line segment. EdgeGrid.cpp:1091-1092
                    let p1 = *contour.segment_start(ipt);
                    let p2 = *contour.segment_end(ipt);
                    // EdgeGrid.cpp:1093-1094
                    let v_seg = (p2.x - p1.x, p2.y - p1.y);
                    let v_pt = (pt.x - p1.x, pt.y - p1.y);
                    // dot(p2-p1, pt-p1). EdgeGrid.cpp:1096
                    let t_pt = v_seg.0 * v_pt.0 + v_seg.1 * v_pt.1;
                    // l2 of seg. EdgeGrid.cpp:1098
                    let l2_seg = v_seg.0 * v_seg.0 + v_seg.1 * v_seg.1;
                    if t_pt < 0 {
                        // Closest to p1. EdgeGrid.cpp:1099-1125
                        let dabs = ((v_pt.0 * v_pt.0 + v_pt.1 * v_pt.1) as f64).sqrt();
                        if dabs < d_min {
                            // Previous point. EdgeGrid.cpp:1104
                            let p0 = *contour.segment_prev(ipt);
                            let v_seg_prev = (p1.x - p0.x, p1.y - p0.y);
                            let t2_pt = v_seg_prev.0 * v_pt.0 + v_seg_prev.1 * v_pt.1;
                            if t2_pt > 0 {
                                // Inside the wedge between the previous and the next segment.
                                d_min = dabs;
                                // Set signum depending on whether the vertex is convex or reflex.
                                // EdgeGrid.cpp:1111-1113
                                let det = v_seg_prev.0 * v_seg.1 - v_seg_prev.1 * v_seg.0;
                                sign_min = if det > 0 { 1 } else { -1 };
                                result.contour_idx = contour_idx;
                                result.start_point_idx = ipt;
                                result.t = 0.0;
                            }
                        }
                    } else if t_pt > l2_seg {
                        // Closest to p2. p2 is the start of another segment in the same cell.
                        // EdgeGrid.cpp:1126-1128
                        continue;
                    } else {
                        // Closest to the segment. EdgeGrid.cpp:1129-1149
                        let d_seg = v_seg.1 * v_pt.0 - v_seg.0 * v_pt.1;
                        let d = d_seg as f64 / (l2_seg as f64).sqrt();
                        let dabs = d.abs();
                        if dabs < d_min {
                            d_min = dabs;
                            sign_min = if d_seg < 0 {
                                -1
                            } else if d_seg == 0 {
                                0
                            } else {
                                1
                            };
                            l2_seg_min = l2_seg as f64;
                            result.contour_idx = contour_idx;
                            result.start_point_idx = ipt;
                            result.t = t_pt as f64;
                        }
                    }
                }
            }
        }

        // EdgeGrid.cpp:1154-1174
        if result.contour_idx != usize::MAX && d_min <= search_radius as f64 {
            result.distance = d_min * sign_min as f64;
            result.t /= l2_seg_min;
            // Derive the foot point (Rust extension; C++ has no `point` field).
            let contour = &self.contours[result.contour_idx];
            let p1 = *contour.segment_start(result.start_point_idx);
            let p2 = *contour.segment_end(result.start_point_idx);
            let foot_x = p1.x as f64 * (1.0 - result.t) + p2.x as f64 * result.t;
            let foot_y = p1.y as f64 * (1.0 - result.t) + p2.y as f64 * result.t;
            result.point = Point::new(foot_x.round() as i64, foot_y.round() as i64);
        } else {
            result = ClosestPointResult::invalid();
        }
        result
    }

    /// Test whether a point is inside the contours using ray casting (odd-even rule).
    /// EdgeGrid.cpp:536-597
    pub fn point_inside(&self, point: &Point) -> bool {
        // Return false if no contours loaded
        // EdgeGrid.cpp:537-538
        if self.contours.is_empty() {
            return false;
        }

        // Count ray crossings for inside/outside determination
        // EdgeGrid.cpp:550
        let mut crossings = 0;

        // Iterate over all closed contours
        // EdgeGrid.cpp:555-592
        for contour in &self.contours {
            // Skip open polylines (only closed polygons define inside)
            // EdgeGrid.cpp:555
            if contour.is_open() {
                continue;
            }

            // Test each segment for intersection with horizontal ray from point
            // EdgeGrid.cpp:556-591
            for seg_idx in 0..contour.num_segments() {
                let p1 = contour.segment_start(seg_idx);
                let p2 = contour.segment_end(seg_idx);

                // Ray crossing test: horizontal ray to the right from point
                // EdgeGrid.cpp:566-591
                if (p1.y > point.y) != (p2.y > point.y) {
                    // Compute x-coordinate of intersection with the horizontal ray
                    // EdgeGrid.cpp:570-573
                    let slope = (p2.x - p1.x) as f64 / (p2.y - p1.y) as f64;
                    let x_intersect = p1.x as f64 + (point.y - p1.y) as f64 * slope;
                    if (point.x as f64) < x_intersect {
                        crossings += 1;
                    }
                }
            }
        }

        // Odd number of crossings means inside
        // EdgeGrid.cpp:595-596
        crossings % 2 == 1
    }

    /// Fill in a rough `m_signed_distance_field` from the edge grid using the
    /// Danielsson chamfer metric. The SDF is stored on grid corners as an
    /// `(rows+1) x (cols+1)` array. Only call for closed contours.
    /// Faithful port of `EdgeGrid::Grid::calculate_sdf`.
    /// EdgeGrid.cpp:672-980
    pub fn calculate_sdf(&mut self) {
        // 1) Initialize a signum and an unsigned vector to a zero iso surface.
        // EdgeGrid.cpp:680-691
        let nrows = self.rows + 1;
        let ncols = self.cols + 1;
        // Unsigned vectors towards the closest point on the surface (interleaved x,y).
        let mut big_l = vec![f32::MAX; nrows * ncols * 2];
        // Bit 0 set - negative.
        // Bit 1 set - original value, the distance value shall not be changed by the Danielsson propagation.
        // Bit 2 set - signum not propagated yet.
        let mut signs = vec![4u8; nrows * ncols];
        // SDF will be initially filled with the search radius (unsigned DF placeholder).
        let search_radius = (self.resolution << 1) as f32;
        let mut sdf = vec![search_radius; nrows * ncols];

        // For each cell: EdgeGrid.cpp:693-775
        for r in 0..self.rows {
            for c in 0..self.cols {
                let cell = self.cells[r * self.cols + c];
                // For each segment in the cell.
                for i in cell.begin..cell.end {
                    let (contour_idx, ipt) = self.cell_data[i];
                    let contour = &self.contours[contour_idx];
                    // End points of the line segment. EdgeGrid.cpp:702-703
                    let p1 = *contour.segment_start(ipt);
                    let p2 = *contour.segment_end(ipt);
                    // Segment vector and its squared length. EdgeGrid.cpp:705-707
                    let v_seg = (p2.x - p1.x, p2.y - p1.y);
                    let l2_seg = v_seg.0 * v_seg.0 + v_seg.1 * v_seg.1;
                    // For each corner of this cell and its 1-ring neighbours.
                    // EdgeGrid.cpp:709-771
                    for corner_y in -1i64..3 {
                        let corner_r = r as i64 + corner_y;
                        if corner_r < 0 || corner_r as usize >= nrows {
                            continue;
                        }
                        for corner_x in -1i64..3 {
                            let corner_c = c as i64 + corner_x;
                            if corner_c < 0 || corner_c as usize >= ncols {
                                continue;
                            }
                            let addr = corner_r as usize * ncols + corner_c as usize;
                            // EdgeGrid.cpp:718
                            let pt = Point::new(
                                self.bbox.min.x + corner_c * self.resolution,
                                self.bbox.min.y + corner_r * self.resolution,
                            );
                            let v_pt = (pt.x - p1.x, pt.y - p1.y);
                            // dot(p2-p1, pt-p1). EdgeGrid.cpp:721
                            let t_pt = v_seg.0 * v_pt.0 + v_seg.1 * v_pt.1;
                            if t_pt < 0 {
                                // Closest to p1. EdgeGrid.cpp:722-747
                                let dabs = ((v_pt.0 * v_pt.0 + v_pt.1 * v_pt.1) as f64).sqrt();
                                if (dabs as f32) < sdf[addr] {
                                    // Previous point.
                                    let p0 = *contour.segment_prev(ipt);
                                    let v_seg_prev = (p1.x - p0.x, p1.y - p0.y);
                                    let t2_pt = v_seg_prev.0 * v_pt.0 + v_seg_prev.1 * v_pt.1;
                                    if t2_pt > 0 {
                                        // Inside the wedge between the previous and the next segment.
                                        let det =
                                            v_seg_prev.0 * v_seg.1 - v_seg_prev.1 * v_seg.0;
                                        sdf[addr] = dabs as f32;
                                        // Fill in an unsigned vector towards the zero iso surface.
                                        big_l[addr << 1] = v_pt.0.abs() as f32;
                                        big_l[(addr << 1) + 1] = v_pt.1.abs() as f32;
                                        signs[addr] = (if det < 0 { 1 } else { 0 }) | 2;
                                    }
                                }
                            } else if t_pt > l2_seg {
                                // Closest to p2. EdgeGrid.cpp:748-750
                                continue;
                            } else {
                                // Closest to the segment. EdgeGrid.cpp:751-769
                                let d_seg = v_seg.1 * v_pt.0 - v_seg.0 * v_pt.1;
                                let d = d_seg as f64 / (l2_seg as f64).sqrt();
                                let dabs = d.abs();
                                if (dabs as f32) < sdf[addr] {
                                    sdf[addr] = dabs as f32;
                                    // Fill in an unsigned vector towards the zero iso surface.
                                    let linv = d_seg as f32 / l2_seg as f32;
                                    big_l[addr << 1] = (v_seg.1 as f32 * linv).abs();
                                    big_l[(addr << 1) + 1] = (v_seg.0 as f32 * linv).abs();
                                    signs[addr] = (if d_seg < 0 { 1 } else { 0 }) | 2;
                                }
                            }
                        }
                    }
                }
            }
        }

        // 2) Propagate the signum. EdgeGrid.cpp:834-862
        // PROPAGATE_SIGNUM_SINGLE_STEP(DELTA): if cur has bit 2 and neighbour does
        // not, copy neighbour's bit 0.
        let propagate_signum = |signs: &mut [u8], addr: usize, neighbour: usize| {
            if signs[addr] & 4 != 0 {
                let old_val = signs[neighbour];
                if old_val & 4 == 0 {
                    signs[addr] = old_val & 1;
                }
            }
        };
        // Top to bottom propagation. EdgeGrid.cpp:844-852
        for r in 0..nrows {
            if r > 0 {
                for c in 0..ncols {
                    let addr = r * ncols + c;
                    propagate_signum(&mut signs, addr, addr - ncols);
                }
            }
            for c in 1..ncols {
                let addr = r * ncols + c;
                propagate_signum(&mut signs, addr, addr - 1);
            }
            for c in (0..ncols.saturating_sub(1)).rev() {
                let addr = r * ncols + c;
                propagate_signum(&mut signs, addr, addr + 1);
            }
        }
        // Bottom to top propagation. EdgeGrid.cpp:854-861
        for r in (0..nrows.saturating_sub(1)).rev() {
            for c in 0..ncols {
                let addr = r * ncols + c;
                propagate_signum(&mut signs, addr, addr + ncols);
            }
            for c in 1..ncols {
                let addr = r * ncols + c;
                propagate_signum(&mut signs, addr, addr - 1);
            }
            for c in (0..ncols.saturating_sub(1)).rev() {
                let addr = r * ncols + c;
                propagate_signum(&mut signs, addr, addr + 1);
            }
        }

        // 3) Propagate the distance by the Danielsson chamfer metric.
        // EdgeGrid.cpp:599-624 helper, applied EdgeGrid.cpp:864-889.
        let res = self.resolution as f32;
        // PropagateDanielssonSingleStep<INCX, INCY>: only updates cells without bit 1.
        let danielsson_step =
            |big_l: &mut [f32], signs: &[u8], addr: usize, addr_delta: isize, incx: f32, incy: f32| {
                if signs[addr] & 2 == 0 {
                    let l = big_l[addr << 1] * big_l[addr << 1]
                        + big_l[(addr << 1) + 1] * big_l[(addr << 1) + 1];
                    let v2s = ((addr as isize + addr_delta) as usize) << 1;
                    let v2x = big_l[v2s] + incx * res;
                    let v2y = big_l[v2s + 1] + incy * res;
                    let l2 = v2x * v2x + v2y * v2y;
                    if l2 < l {
                        big_l[addr << 1] = v2x;
                        big_l[(addr << 1) + 1] = v2y;
                    }
                }
            };
        let ncols_i = ncols as isize;
        // Top to bottom propagation. EdgeGrid.cpp:870-879
        for r in 0..nrows {
            if r > 0 {
                for c in 0..ncols {
                    let addr = r * ncols + c;
                    danielsson_step(&mut big_l, &signs, addr, -ncols_i, 0.0, 1.0);
                }
            }
            for c in 1..ncols {
                let addr = r * ncols + c;
                danielsson_step(&mut big_l, &signs, addr, -1, 1.0, 0.0);
            }
            for c in (0..ncols.saturating_sub(1)).rev() {
                let addr = r * ncols + c;
                danielsson_step(&mut big_l, &signs, addr, 1, 1.0, 0.0);
            }
        }
        // Bottom to top propagation. EdgeGrid.cpp:881-889
        for r in (0..nrows.saturating_sub(1)).rev() {
            for c in 0..ncols {
                let addr = r * ncols + c;
                danielsson_step(&mut big_l, &signs, addr, ncols_i, 0.0, 1.0);
            }
            for c in 1..ncols {
                let addr = r * ncols + c;
                danielsson_step(&mut big_l, &signs, addr, -1, 1.0, 0.0);
            }
            for c in (0..ncols.saturating_sub(1)).rev() {
                let addr = r * ncols + c;
                danielsson_step(&mut big_l, &signs, addr, 1, 1.0, 0.0);
            }
        }

        // Update signed distance field from absolute vectors to the iso-surface.
        // EdgeGrid.cpp:892-901
        for r in 0..nrows {
            for c in 0..ncols {
                let addr = r * ncols + c;
                let vx = big_l[addr << 1];
                let vy = big_l[(addr << 1) + 1];
                let mut d = (vx * vx + vy * vy).sqrt();
                if signs[addr] & 1 != 0 {
                    d = -d;
                }
                sdf[addr] = d;
            }
        }

        self.signed_distance_field = sdf;
    }

    /// Return an estimate of the signed distance based on the corner SDF grid,
    /// bilinearly interpolated. Faithful port of `signed_distance_bilinear`.
    /// EdgeGrid.cpp:982-1045
    pub fn signed_distance_bilinear(&self, pt: &Point) -> f32 {
        // EdgeGrid.cpp:984-987
        let x = pt.x - self.bbox.min.x;
        let y = pt.y - self.bbox.min.y;
        let w = self.resolution * self.cols as i64;
        let h = self.resolution * self.rows as i64;
        let mut clamped = false;
        let mut xcl = x;
        let mut ycl = y;
        // EdgeGrid.cpp:991-1004
        if x < 0 {
            xcl = 0;
            clamped = true;
        } else if x >= w {
            xcl = w - 1;
            clamped = true;
        }
        if y < 0 {
            ycl = 0;
            clamped = true;
        } else if y >= h {
            ycl = h - 1;
            clamped = true;
        }

        // EdgeGrid.cpp:1006-1011
        let cell_c = (xcl as f64 / self.resolution as f64).floor() as i64;
        let cell_r = (ycl as f64 / self.resolution as f64).floor() as i64;
        let tx = (xcl - cell_c * self.resolution) as f32 / self.resolution as f32;
        let ty = (ycl - cell_r * self.resolution) as f32 / self.resolution as f32;
        // EdgeGrid.cpp:1012-1020 — corner SDF stride is (m_cols + 1).
        let stride = self.cols + 1;
        let mut addr = cell_r as usize * stride + cell_c as usize;
        let f00 = self.signed_distance_field[addr];
        let f01 = self.signed_distance_field[addr + 1];
        addr += stride;
        let f10 = self.signed_distance_field[addr];
        let f11 = self.signed_distance_field[addr + 1];
        let f0 = (1.0 - tx) * f00 + tx * f01;
        let f1 = (1.0 - tx) * f10 + tx * f11;
        let mut f = (1.0 - ty) * f0 + ty * f1;

        // EdgeGrid.cpp:1022-1042 — adjust the interpolated value for clamped points.
        if clamped {
            if f > 0.0 {
                if x < 0 {
                    f += (-x) as f32;
                } else if x >= w {
                    f += (x - w + 1) as f32;
                }
                if y < 0 {
                    f += (-y) as f32;
                } else if y >= h {
                    f += (y - h + 1) as f32;
                }
            } else {
                if x < 0 {
                    f -= (-x) as f32;
                } else if x >= w {
                    f -= (x - w + 1) as f32;
                }
                if y < 0 {
                    f -= (-y) as f32;
                } else if y >= h {
                    f -= (y - h + 1) as f32;
                }
            }
        }

        f
    }

    /// Exact signed distance from `pt` to the nearest contour edge within `search_radius`.
    /// Returns `(signed_distance, on_segment)`, or None if no edge is within the radius.
    /// Sign: positive outside, negative inside (left of the CCW contour is interior).
    /// EdgeGrid.cpp:1178 `Grid::signed_distance_edges`.
    pub fn signed_distance_edges(&self, pt: &Point, search_radius: i64) -> Option<(f64, bool)> {
        // Cell-index window around pt (in grid-local coordinates), clamped to the grid.
        let mut max_c = pt.x - self.bbox.min.x;
        let mut max_r = pt.y - self.bbox.min.y;
        let mut min_c = max_c;
        let mut min_r = max_r;
        // Upper boundary, round to grid and test validity.
        max_c += search_radius;
        max_r += search_radius;
        if max_c < 0 || max_r < 0 {
            return None;
        }
        max_c /= self.resolution;
        max_r /= self.resolution;
        if max_c as usize >= self.cols {
            max_c = self.cols as i64 - 1;
        }
        if max_r as usize >= self.rows {
            max_r = self.rows as i64 - 1;
        }
        // Lower boundary, round to grid and test validity.
        min_c -= search_radius;
        min_r -= search_radius;
        if min_c < 0 {
            min_c = 0;
        }
        if min_r < 0 {
            min_r = 0;
        }
        min_c /= self.resolution;
        min_r /= self.resolution;
        if min_c > max_c || min_r > max_r {
            return None;
        }
        // Traverse all cells in the window.
        let mut d_min = search_radius as f64;
        let mut sign_min: i32 = 0;
        let mut on_segment = false;
        for r in min_r..=max_r {
            for c in min_c..=max_c {
                let cell = &self.cells[r as usize * self.cols + c as usize];
                for i in cell.begin..cell.end {
                    let (contour_idx, ipt) = self.cell_data[i];
                    let contour = &self.contours[contour_idx];
                    // End points of the line segment.
                    let p1 = *contour.segment_start(ipt);
                    let p2 = *contour.segment_end(ipt);
                    let v_seg = p2 - p1;
                    let v_pt = *pt - p1;
                    // dot(p2-p1, pt-p1) and squared length of the segment.
                    let t_pt = v_seg.x * v_pt.x + v_seg.y * v_pt.y;
                    let l2_seg = v_seg.x * v_seg.x + v_seg.y * v_seg.y;
                    if t_pt < 0 {
                        // Closest to p1.
                        let dabs = ((v_pt.x * v_pt.x + v_pt.y * v_pt.y) as f64).sqrt();
                        if dabs < d_min {
                            // Previous point.
                            let p0 = *contour.segment_prev(ipt);
                            let v_seg_prev = p1 - p0;
                            let t2_pt = v_seg_prev.x * v_pt.x + v_seg_prev.y * v_pt.y;
                            if t2_pt > 0 {
                                // Inside the wedge between the previous and the next segment.
                                d_min = dabs;
                                // Signum depending on whether the vertex is convex or reflex.
                                let det = v_seg_prev.x * v_seg.y - v_seg_prev.y * v_seg.x;
                                sign_min = if det > 0 { 1 } else { -1 };
                                on_segment = false;
                            }
                        }
                    } else if t_pt > l2_seg {
                        // Closest to p2; handled as p1 of the following segment.
                        continue;
                    } else {
                        // Closest to the segment interior.
                        let d_seg = v_seg.y * v_pt.x - v_seg.x * v_pt.y;
                        let d = d_seg as f64 / (l2_seg as f64).sqrt();
                        let dabs = d.abs();
                        if dabs < d_min {
                            d_min = dabs;
                            sign_min = if d_seg < 0 {
                                -1
                            } else if d_seg == 0 {
                                0
                            } else {
                                1
                            };
                            on_segment = true;
                        }
                    }
                }
            }
        }
        if d_min >= search_radius as f64 {
            return None;
        }
        Some((d_min * sign_min as f64, on_segment))
    }

    /// Signed distance from `pt` to the contour, falling back to the precomputed SDF.
    /// EdgeGrid.cpp:1273 `Grid::signed_distance`.
    pub fn signed_distance(&self, pt: &Point, search_radius: i64) -> Option<f64> {
        if let Some((d, _on_segment)) = self.signed_distance_edges(pt, search_radius) {
            return Some(d);
        }
        if self.signed_distance_field.is_empty() {
            return None;
        }
        Some(self.signed_distance_bilinear(pt) as f64)
    }
}

/// Default implementation delegates to EdgeGrid::new()
/// EdgeGrid.hpp:94
impl Default for EdgeGrid {
    // Return default-constructed grid
    // EdgeGrid.hpp:94
    fn default() -> Self {
        Self::new()
    }
}

/// Find the closest point on a line segment to a query point.
/// Returns (closest_point, t) where t is the parameter along the segment [0, 1].
fn closest_point_on_segment(query: (f64, f64), p1: &Point, p2: &Point) -> ((f64, f64), f64) {
    let p1f = (p1.x as f64, p1.y as f64);
    let p2f = (p2.x as f64, p2.y as f64);

    let dx = p2f.0 - p1f.0;
    let dy = p2f.1 - p1f.1;
    let len_sq = dx * dx + dy * dy;

    if len_sq < 1e-10 {
        // Degenerate segment
        return (p1f, 0.0);
    }

    // Project query onto segment
    let t = ((query.0 - p1f.0) * dx + (query.1 - p1f.1) * dy) / len_sq;
    let t_clamped = t.clamp(0.0, 1.0);

    let closest = (p1f.0 + t_clamped * dx, p1f.1 + t_clamped * dy);

    (closest, t_clamped)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_square() -> Polygon {
        Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(1_000_000, 0),
            Point::new(1_000_000, 1_000_000),
            Point::new(0, 1_000_000),
        ])
    }

    #[test]
    fn test_contour_from_polygon() {
        let square = make_square();
        let contour = Contour::from_polygon(&square);

        assert!(contour.is_closed());
        assert!(!contour.is_open());
        assert_eq!(contour.num_segments(), 4);
    }

    #[test]
    fn test_contour_segments() {
        let square = make_square();
        let contour = Contour::from_polygon(&square);

        let seg0 = contour.segment(0);
        assert_eq!(seg0.a, Point::new(0, 0));
        assert_eq!(seg0.b, Point::new(1_000_000, 0));

        let seg3 = contour.segment(3);
        assert_eq!(seg3.a, Point::new(0, 1_000_000));
        assert_eq!(seg3.b, Point::new(0, 0));
    }

    #[test]
    fn test_edge_grid_creation() {
        let square = make_square();
        let grid = EdgeGrid::from_polygon(&square, 100_000);

        assert!(!grid.bbox().is_empty());
        assert!(grid.rows() > 0);
        assert!(grid.cols() > 0);
        assert_eq!(grid.contours().len(), 1);
    }

    #[test]
    fn test_line_intersects_any() {
        let square = make_square();
        let grid = EdgeGrid::from_polygon(&square, 100_000);

        // Line from outside to inside should intersect
        let p1 = Point::new(-500_000, 500_000);
        let p2 = Point::new(500_000, 500_000);
        assert!(grid.line_intersects_any(&p1, &p2));

        // Line completely outside should not intersect
        let p1 = Point::new(-500_000, -500_000);
        let p2 = Point::new(-100_000, -100_000);
        assert!(!grid.line_intersects_any(&p1, &p2));
    }

    #[test]
    fn test_find_intersections() {
        let square = make_square();
        let grid = EdgeGrid::from_polygon(&square, 100_000);

        // Line that crosses two edges
        let p1 = Point::new(-500_000, 500_000);
        let p2 = Point::new(1_500_000, 500_000);
        let intersections = grid.find_intersections(&p1, &p2);

        assert_eq!(intersections.len(), 2);
    }

    #[test]
    fn test_closest_point() {
        let square = make_square();
        let grid = EdgeGrid::from_polygon(&square, 100_000);

        // Point outside, closest to bottom edge
        let query = Point::new(500_000, -100_000);
        let result = grid.closest_point(&query, 200_000);

        assert!(result.is_valid());
        assert_eq!(result.point.y, 0); // Should be on bottom edge
        assert!((result.distance - 100_000.0).abs() < 1000.0);
    }

    #[test]
    fn test_point_inside() {
        let square = make_square();
        let grid = EdgeGrid::from_polygon(&square, 100_000);

        // Point inside
        assert!(grid.point_inside(&Point::new(500_000, 500_000)));

        // Point outside
        assert!(!grid.point_inside(&Point::new(-100_000, 500_000)));
    }

    #[test]
    fn test_closest_point_on_segment() {
        let p1 = Point::new(0, 0);
        let p2 = Point::new(1_000_000, 0);

        // Query point perpendicular to segment midpoint
        let query = (500_000.0, 100_000.0);
        let (closest, t) = closest_point_on_segment(query, &p1, &p2);

        assert!((closest.0 - 500_000.0).abs() < 1.0);
        assert!((closest.1 - 0.0).abs() < 1.0);
        assert!((t - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_empty_grid() {
        let grid = EdgeGrid::new();

        assert!(!grid.line_intersects_any(&Point::new(0, 0), &Point::new(1, 1)));
        assert!(grid
            .find_intersections(&Point::new(0, 0), &Point::new(1, 1))
            .is_empty());
        assert!(!grid.closest_point(&Point::new(0, 0), 1000).is_valid());
    }

    #[test]
    fn test_multiple_polygons() {
        let square1 = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(100_000, 0),
            Point::new(100_000, 100_000),
            Point::new(0, 100_000),
        ]);
        let square2 = Polygon::from_points(vec![
            Point::new(200_000, 0),
            Point::new(300_000, 0),
            Point::new(300_000, 100_000),
            Point::new(200_000, 100_000),
        ]);

        let grid = EdgeGrid::from_polygons(&[square1, square2], 10_000);

        assert_eq!(grid.contours().len(), 2);

        // Line through both squares
        let p1 = Point::new(-50_000, 50_000);
        let p2 = Point::new(350_000, 50_000);
        let intersections = grid.find_intersections(&p1, &p2);

        // Should intersect 4 edges (2 per square)
        assert_eq!(intersections.len(), 4);
    }

    #[test]
    fn test_contour_open_polyline() {
        let polyline = Polyline::from_points(vec![
            Point::new(0, 0),
            Point::new(100_000, 0),
            Point::new(100_000, 100_000),
        ]);
        let contour = Contour::from_polyline(&polyline);

        assert!(contour.is_open());
        assert_eq!(contour.num_segments(), 2); // Open: n-1 segments
    }
}
