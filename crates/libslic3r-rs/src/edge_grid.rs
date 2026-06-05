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

use crate::geometry::{BoundingBox, Line, Point, Polygon, Polyline};

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

    /// Get a segment as a Line
    /// EdgeGrid.hpp:66-70
    pub fn segment(&self, idx: usize) -> Line {
        Line::new(*self.segment_start(idx), *self.segment_end(idx))
    }

    /// Get all segments as Lines
    /// EdgeGrid.hpp:72-81
    pub fn segments(&self) -> Vec<Line> {
        (0..self.num_segments()).map(|i| self.segment(i)).collect()
    }
}

/// A cell in the edge grid, stores range into cell_data array
/// EdgeGrid.hpp:410-414
#[derive(Clone, Debug, Default)]
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

    /// Visit all grid cells intersected by the line segment (p1, p2), calling
    /// `visitor(iy, ix)` for each. Equivalent to the C++ template method without the
    /// `need_consider_eps` extension (the PolygonTrimmer call site never sets it).
    /// EdgeGrid.hpp:291-366
    pub fn visit_cells_intersecting_line<F>(&self, p1: Point, p2: Point, mut visitor: F)
    where
        F: FnMut(usize, usize),
    {
        // EdgeGrid.hpp:360-365 — single start/end pair when need_consider_eps is false.
        self.visit_cells_for_segment(&p1, &p2, |row, col| visitor(row, col));
    }

    /// Create the grid from polygons
    /// EdgeGrid.cpp:28-38
    pub fn create_from_polygons(&mut self, polygons: &[Polygon], resolution: i64) {
        self.contours = polygons.iter().map(Contour::from_polygon).collect();
        self.create_from_contours(resolution);
    }

    /// Create the grid from polylines
    /// EdgeGrid.cpp:52-75
    pub fn create_from_polylines(&mut self, polylines: &[Polyline], resolution: i64) {
        self.contours = polylines.iter().map(Contour::from_polyline).collect();
        self.create_from_contours(resolution);
    }

    /// Create the grid from both polygons and polylines
    /// EdgeGrid.cpp:77-102
    pub fn create_from_mixed(
        &mut self,
        polygons: &[Polygon],
        polylines: &[Polyline],
        resolution: i64,
    ) {
        self.contours = polygons
            .iter()
            .map(Contour::from_polygon)
            .chain(polylines.iter().map(Contour::from_polyline))
            .collect();
        self.create_from_contours(resolution);
    }

    /// Internal: create the grid from stored contours
    /// EdgeGrid.cpp:142-334
    fn create_from_contours(&mut self, resolution: i64) {
        // EdgeGrid.cpp:145 — m_resolution = resolution
        self.resolution = resolution.max(1);

        // EdgeGrid.cpp:147-153 — compute bounding box from all contour points
        self.bbox = BoundingBox::new();
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

        // EdgeGrid.cpp:162-167 — add margin to avoid edge cases
        let margin = self.resolution;
        self.bbox = BoundingBox::from_points_minmax(
            Point::new(self.bbox.min.x - margin, self.bbox.min.y - margin),
            Point::new(self.bbox.max.x + margin, self.bbox.max.y + margin),
        );

        // EdgeGrid.cpp:169-172 — calculate grid dimensions
        let size = self.bbox.size();
        self.cols = ((size.x as i64 + self.resolution - 1) / self.resolution).max(1) as usize;
        self.rows = ((size.y as i64 + self.resolution - 1) / self.resolution).max(1) as usize;

        // EdgeGrid.cpp:174-190 — count edges per cell (first pass)
        let num_cells = self.rows * self.cols;
        let mut cell_counts = vec![0usize; num_cells];

        for (contour_idx, contour) in self.contours.iter().enumerate() {
            for seg_idx in 0..contour.num_segments() {
                // EdgeGrid.cpp:180-188 — visit cells for each segment
                let p1 = contour.segment_start(seg_idx);
                let p2 = contour.segment_end(seg_idx);
                self.visit_cells_for_segment(p1, p2, |row, col| {
                    let cell_idx = row * self.cols + col;
                    if cell_idx < num_cells {
                        cell_counts[cell_idx] += 1;
                    }
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

    /// Convert a point to cell coordinates (row, col).
    /// EdgeGrid.cpp:177-180
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

    /// Visit all cells that a line segment passes through using Bresenham-like rasterization.
    /// EdgeGrid.hpp:172-289
    fn visit_cells_for_segment<F>(&self, p1: &Point, p2: &Point, mut visitor: F)
    where
        F: FnMut(usize, usize),
    {
        // Early return if grid is empty
        // EdgeGrid.hpp:176-178
        if self.cols == 0 || self.rows == 0 {
            return;
        }

        // Convert endpoints to cell coordinates
        // EdgeGrid.hpp:173-174
        let (row1, col1) = self.point_to_cell(p1);
        let (row2, col2) = self.point_to_cell(p2);

        // Bresenham-like algorithm for cell traversal
        // EdgeGrid.hpp:180-181
        let col1 = col1 as i64;
        let row1 = row1 as i64;
        let col2 = col2 as i64;
        let row2 = row2 as i64;

        // Compute absolute deltas and step directions
        // EdgeGrid.hpp:180-181
        let dx = (col2 - col1).abs();
        let dy = (row2 - row1).abs();
        let sx: i64 = if col1 < col2 { 1 } else { -1 };
        let sy: i64 = if row1 < row2 { 1 } else { -1 };

        // Initialize error accumulator
        // EdgeGrid.hpp:183-186
        let mut col = col1;
        let mut row = row1;
        let mut err = dx - dy;

        // Walk from start cell to end cell visiting each cell along the way
        // EdgeGrid.hpp:187-289
        loop {
            // Visit current cell if within bounds
            // EdgeGrid.hpp:208-209
            if col >= 0 && col < self.cols as i64 && row >= 0 && row < self.rows as i64 {
                visitor(row as usize, col as usize);
            }

            // Check if we've reached the destination cell
            // EdgeGrid.hpp:210
            if col == col2 && row == row2 {
                break;
            }

            // Compute doubled error for direction decision
            // EdgeGrid.hpp:189-207
            let e2 = 2 * err;

            // Move diagonally or along one axis based on error
            // EdgeGrid.hpp:189-207
            if e2 > -dy && e2 < dx {
                // Move diagonally
                // EdgeGrid.hpp:194-199
                err += -dy + dx;
                col += sx;
                row += sy;
            } else if e2 > -dy {
                // Move horizontally
                // EdgeGrid.hpp:189-193
                err -= dy;
                col += sx;
            } else {
                // Move vertically
                // EdgeGrid.hpp:201-206
                err += dx;
                row += sy;
            }
        }
    }

    /// Visit cells along a segment and insert segment reference into cell_data (mutable fill pass).
    /// EdgeGrid.cpp:311-333
    fn visit_cells_for_segment_mut(
        &mut self,
        p1: &Point,
        p2: &Point,
        contour_idx: usize,
        seg_idx: usize,
    ) {
        // Early return if grid is empty
        // EdgeGrid.cpp:311
        if self.cols == 0 || self.rows == 0 {
            return;
        }

        // Convert endpoints to cell coordinates
        // EdgeGrid.cpp:170-176
        let (row1, col1) = self.point_to_cell(p1);
        let (row2, col2) = self.point_to_cell(p2);

        // Cast to signed integers for Bresenham arithmetic
        // EdgeGrid.cpp:177-180
        let col1 = col1 as i64;
        let row1 = row1 as i64;
        let col2 = col2 as i64;
        let row2 = row2 as i64;

        // Compute absolute deltas and step directions
        // EdgeGrid.cpp:191-192
        let dx = (col2 - col1).abs();
        let dy = (row2 - row1).abs();
        let sx: i64 = if col1 < col2 { 1 } else { -1 };
        let sy: i64 = if row1 < row2 { 1 } else { -1 };

        // Initialize position and error accumulator
        // EdgeGrid.cpp:186-189
        let mut col = col1;
        let mut row = row1;
        let mut err = dx - dy;

        // Walk the segment inserting (contour_idx, seg_idx) into each visited cell
        // EdgeGrid.cpp:315-316
        loop {
            // Insert segment reference into current cell's data
            // EdgeGrid.cpp:316
            if col >= 0 && col < self.cols as i64 && row >= 0 && row < self.rows as i64 {
                let cell_idx = row as usize * self.cols + col as usize;
                if cell_idx < self.cells.len() {
                    // Append to cell by advancing end pointer
                    // EdgeGrid.cpp:316
                    let insert_idx = self.cells[cell_idx].end;
                    if insert_idx < self.cell_data.len() {
                        self.cell_data[insert_idx] = (contour_idx, seg_idx);
                        self.cells[cell_idx].end += 1;
                    }
                }
            }

            // Check if we've reached the destination cell
            // EdgeGrid.cpp:218
            if col == col2 && row == row2 {
                break;
            }

            // Bresenham step to next cell
            // EdgeGrid.cpp:198-291
            let e2 = 2 * err;

            if e2 > -dy && e2 < dx {
                err += -dy + dx;
                col += sx;
                row += sy;
            } else if e2 > -dy {
                err -= dy;
                col += sx;
            } else {
                err += dx;
                row += sy;
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
            // Short-circuit if already found an intersection
            // EdgeGrid.hpp:293
            if found {
                return;
            }
            // Test each edge in this cell against the query line
            // EdgeGrid.hpp:300-365
            for &(contour_idx, seg_idx) in self.cell_data_range(row, col) {
                let segment = self.contours[contour_idx].segment(seg_idx);
                if line.intersects(&segment) {
                    found = true;
                    return;
                }
            }
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
        });

        // Sort intersections by distance along the query line
        // No direct C++ equivalent — Rust-specific
        intersections.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());

        intersections
    }

    /// Find the closest point on any edge to a query point within a search radius.
    /// EdgeGrid.cpp:1047-1176
    pub fn closest_point(&self, query: &Point, search_radius: i64) -> ClosestPointResult {
        // Initialize result as invalid (no match found yet)
        // EdgeGrid.cpp:1055
        let mut result = ClosestPointResult::invalid();

        // Early return if grid is empty
        // EdgeGrid.cpp:1056-1057
        if self.cols == 0 || self.rows == 0 {
            return result;
        }

        // Compute squared search radius for distance comparison
        // EdgeGrid.cpp:1078
        let search_radius_sq = (search_radius as f64) * (search_radius as f64);

        // Determine bounding box of cells to search (min/max cell from search radius)
        // EdgeGrid.cpp:1049-1072
        let min_cell = self.point_to_cell(&Point::new(
            query.x - search_radius,
            query.y - search_radius,
        ));
        let max_cell = self.point_to_cell(&Point::new(
            query.x + search_radius,
            query.y + search_radius,
        ));

        // Deduplication set for segments spanning multiple cells
        // No direct C++ equivalent — Rust-specific
        let mut seen = std::collections::HashSet::new();
        let query_f = (query.x as f64, query.y as f64);

        // Traverse all cells in the bounding box
        // EdgeGrid.cpp:1082-1153
        for row in min_cell.0..=max_cell.0 {
            for col in min_cell.1..=max_cell.1 {
                for &(contour_idx, seg_idx) in self.cell_data_range(row, col) {
                    // Skip already-tested segments
                    // No direct C++ equivalent — Rust-specific
                    if !seen.insert((contour_idx, seg_idx)) {
                        continue;
                    }

                    // Get segment endpoints
                    // EdgeGrid.cpp:1087-1092
                    let contour = &self.contours[contour_idx];
                    let p1 = contour.segment_start(seg_idx);
                    let p2 = contour.segment_end(seg_idx);

                    // Find closest point on this segment to the query point
                    // EdgeGrid.cpp:1099-1149
                    let (closest, t) = closest_point_on_segment(query_f, p1, p2);
                    let dx = closest.0 - query_f.0;
                    let dy = closest.1 - query_f.1;
                    let dist_sq = dx * dx + dy * dy;

                    // Update result if this is closer than previous best
                    // EdgeGrid.cpp:1135-1141
                    if dist_sq < search_radius_sq && dist_sq < result.distance * result.distance {
                        result.contour_idx = contour_idx;
                        result.start_point_idx = seg_idx;
                        result.distance = dist_sq.sqrt();
                        result.t = t;
                        result.point = Point::new(closest.0 as i64, closest.1 as i64);
                    }
                }
            }
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

    /// Calculate the signed distance field for all grid cell corners using Danielsson chamfer metric.
    /// EdgeGrid.cpp:672-980
    pub fn calculate_sdf(&mut self) {
        // Initialize SDF storage to max distance for all cells
        // EdgeGrid.cpp:679-691
        let num_cells = self.rows * self.cols;
        self.signed_distance_field = vec![f32::MAX; num_cells];

        // For each grid cell, find closest edge and compute signed distance
        // EdgeGrid.cpp:693-775
        for row in 0..self.rows {
            for col in 0..self.cols {
                // Compute cell center point
                // EdgeGrid.cpp:718
                let cell_idx = row * self.cols + col;
                let center = Point::new(
                    self.bbox.min.x + (col as i64) * self.resolution + self.resolution / 2,
                    self.bbox.min.y + (row as i64) * self.resolution + self.resolution / 2,
                );

                // Find closest edge within search radius
                // EdgeGrid.cpp:717-773
                let result = self.closest_point(&center, self.resolution * 10);
                let dist = if result.is_valid() {
                    result.distance as f32
                } else {
                    f32::MAX
                };

                // Determine sign: negative inside, positive outside
                // EdgeGrid.cpp:744-768
                let sign = if self.point_inside(&center) {
                    -1.0
                } else {
                    1.0
                };

                // Store signed distance
                // EdgeGrid.cpp:768
                self.signed_distance_field[cell_idx] = sign * dist;
            }
        }
    }

    /// Get the signed distance at a point using bilinear interpolation of the SDF grid.
    /// EdgeGrid.cpp:982-1045
    pub fn signed_distance_bilinear(&self, point: &Point) -> f32 {
        // Return MAX if SDF has not been computed
        // EdgeGrid.cpp:983
        if self.signed_distance_field.is_empty() {
            return f32::MAX;
        }

        // Compute fractional cell coordinates
        // EdgeGrid.cpp:984-985
        let fx = (point.x - self.bbox.min.x) as f64 / self.resolution as f64;
        let fy = (point.y - self.bbox.min.y) as f64 / self.resolution as f64;

        // Get integer cell corners for the 2x2 interpolation window
        // EdgeGrid.cpp:1006-1007
        let x0 = fx.floor() as i64;
        let y0 = fy.floor() as i64;
        let x1 = x0 + 1;
        let y1 = y0 + 1;

        // Compute interpolation weights
        // EdgeGrid.cpp:1008-1010
        let tx = fx - x0 as f64;
        let ty = fy - y0 as f64;

        // Helper to fetch SDF value with bounds checking
        // EdgeGrid.cpp:1013-1017
        let get_sdf = |row: i64, col: i64| -> f32 {
            if row < 0 || col < 0 || row >= self.rows as i64 || col >= self.cols as i64 {
                return f32::MAX;
            }
            self.signed_distance_field[row as usize * self.cols + col as usize]
        };

        // Fetch four corner SDF values
        // EdgeGrid.cpp:1013-1017
        let v00 = get_sdf(y0, x0);
        let v10 = get_sdf(y0, x1);
        let v01 = get_sdf(y1, x0);
        let v11 = get_sdf(y1, x1);

        // Bilinear interpolation: first along x, then along y
        // EdgeGrid.cpp:1018-1020
        let v0 = v00 * (1.0 - tx as f32) + v10 * tx as f32;
        let v1 = v01 * (1.0 - tx as f32) + v11 * tx as f32;

        v0 * (1.0 - ty as f32) + v1 * ty as f32
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
