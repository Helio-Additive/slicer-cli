//! Extrusion line for variable-width perimeters.
//!
//! This module provides the `ExtrusionLine` type which represents a polyline
//! (sequence of junctions) that forms a variable-width extrusion path.
//!
//! # BambuStudio Reference
//!
//! This corresponds to `src/libslic3r/Arachne/utils/ExtrusionLine.hpp`

use super::junction::ExtrusionJunction;
use crate::geometry::{Point, Polygon, Polyline};
use crate::{scale, unscale, Coord, CoordF};

/// A variable-width extrusion line (polyline with width at each vertex).
///
/// This represents a path that should be extruded with varying width along
/// its length. Each junction in the line specifies the width at that point,
/// and the width is linearly interpolated between junctions.
#[derive(Debug, Clone, PartialEq)]
pub struct ExtrusionLine {
    /// The junctions (vertices with width) along this path.
    pub junctions: Vec<ExtrusionJunction>,

    /// Which inset/perimeter index this line represents.
    /// Counted from outside inwards (0 = outer wall).
    pub inset_idx: usize,

    /// Whether this is an "odd" wall in a thin section.
    /// When printing thin walls with an odd number of walls, there's
    /// one wall in the middle that has no partner on the other side.
    pub is_odd: bool,

    /// Whether this path forms a closed loop.
    pub is_closed: bool,
}

impl ExtrusionLine {
    // Create a new empty extrusion line.
    pub fn new(inset_idx: usize, is_odd: bool, is_closed: bool) -> Self {
        Self {
            junctions: Vec::new(),
            inset_idx,
            is_odd,
            is_closed,
        }
    }

    /// Create a closed extrusion loop.
    pub fn closed(inset_idx: usize) -> Self {
        Self::new(inset_idx, false, true)
    }

    /// Create an open extrusion line.
    pub fn open(inset_idx: usize) -> Self {
        Self::new(inset_idx, false, false)
    }

    /// Create an odd (center) wall line.
    pub fn odd(inset_idx: usize) -> Self {
        Self::new(inset_idx, true, false)
    }

    /// Create an extrusion line from junctions.
    pub fn from_junctions(
        junctions: Vec<ExtrusionJunction>,
        inset_idx: usize,
        is_odd: bool,
        is_closed: bool,
    ) -> Self {
        Self {
            junctions,
            inset_idx,
            is_odd,
            is_closed,
        }
    }

    /// Create a constant-width extrusion line from a polygon.
    pub fn from_polygon(polygon: &Polygon, width: Coord, inset_idx: usize) -> Self {
        let junctions = polygon
            .points()
            .iter()
            .map(|p| ExtrusionJunction::new(*p, width, inset_idx))
            .collect();

        Self {
            junctions,
            inset_idx,
            is_odd: false,
            is_closed: true,
        }
    }

    /// Create a constant-width extrusion line from a polyline.
    pub fn from_polyline(polyline: &Polyline, width: Coord, inset_idx: usize) -> Self {
        let junctions = polyline
            .points()
            .iter()
            .map(|p| ExtrusionJunction::new(*p, width, inset_idx))
            .collect();

        Self {
            junctions,
            inset_idx,
            is_odd: false,
            is_closed: false,
        }
    }

    /// Check if the line is empty (no junctions).
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.junctions.is_empty()
    }

    /// Get the number of junctions.
    #[inline]
    pub fn len(&self) -> usize {
        self.junctions.len()
    }

    /// Add a junction to the end of the line.
    pub fn push(&mut self, junction: ExtrusionJunction) {
        self.junctions.push(junction);
    }

    /// Add a junction at a specific position.
    pub fn insert(&mut self, index: usize, junction: ExtrusionJunction) {
        self.junctions.insert(index, junction);
    }

    /// Remove and return the last junction.
    pub fn pop(&mut self) -> Option<ExtrusionJunction> {
        self.junctions.pop()
    }

    /// Clear all junctions.
    pub fn clear(&mut self) {
        self.junctions.clear();
    }

    /// Reverse the order of junctions.
    pub fn reverse(&mut self) {
        self.junctions.reverse();
    }

    /// Get the first junction (if any).
    pub fn first(&self) -> Option<&ExtrusionJunction> {
        self.junctions.first()
    }

    /// Get the last junction (if any).
    pub fn last(&self) -> Option<&ExtrusionJunction> {
        self.junctions.last()
    }

    /// Get a junction by index.
    pub fn get(&self, index: usize) -> Option<&ExtrusionJunction> {
        self.junctions.get(index)
    }

    /// Get a mutable reference to a junction by index.
    pub fn get_mut(&mut self, index: usize) -> Option<&mut ExtrusionJunction> {
        self.junctions.get_mut(index)
    }

    /// Calculate the total length of this extrusion line in mm.
    pub fn length(&self) -> CoordF {
        if self.junctions.len() < 2 {
            return 0.0;
        }

        let mut total = 0.0;
        for i in 0..self.junctions.len() - 1 {
            total += self.junctions[i].distance_to(&self.junctions[i + 1]);
        }

        // Add closing segment for closed loops
        if self.is_closed && self.junctions.len() >= 2 {
            total += self
                .junctions
                .last()
                .unwrap()
                .distance_to(self.junctions.first().unwrap());
        }

        total
    }

    /// Calculate the total length of this extrusion line in millimeters.
    pub fn length_mm(&self) -> CoordF {
        crate::unscale(self.length() as crate::Coord)
    }

    /// Get the minimum width along this line.
    pub fn min_width(&self) -> Coord {
        self.junctions.iter().map(|j| j.width).min().unwrap_or(0)
    }

    /// Get the maximum width along this line.
    pub fn max_width(&self) -> Coord {
        self.junctions.iter().map(|j| j.width).max().unwrap_or(0)
    }

    /// Get the average width along this line (length-weighted).
    pub fn average_width(&self) -> CoordF {
        if self.junctions.len() < 2 {
            return self.junctions.first().map_or(0.0, |j| j.width as CoordF);
        }

        let mut total_width_length = 0.0;
        let mut total_length = 0.0;

        for i in 0..self.junctions.len() - 1 {
            let j1 = &self.junctions[i];
            let j2 = &self.junctions[i + 1];
            let segment_length = j1.distance_to(j2);
            let avg_width = (j1.width + j2.width) as f64 / 2.0;

            total_width_length += avg_width * segment_length;
            total_length += segment_length;
        }

        if total_length > 0.0 {
            total_width_length / total_length
        } else {
            0.0
        }
    }

    /// Get the average width in millimeters.
    pub fn average_width_mm(&self) -> CoordF {
        unscale(self.average_width() as Coord)
    }

    /// Check if this line is external (outer perimeter).
    pub fn is_external(&self) -> bool {
        self.inset_idx == 0
    }

    /// Check if this is a contour (outer boundary, counterclockwise).
    pub fn is_contour(&self) -> bool {
        if !self.is_closed || self.junctions.len() < 3 {
            return false;
        }
        self.signed_area() > 0.0
    }

    /// Calculate the signed area enclosed by this line (for closed loops).
    /// Positive = counterclockwise (contour), negative = clockwise (hole).
    pub fn signed_area(&self) -> CoordF {
        if self.junctions.len() < 3 {
            return 0.0;
        }

        let mut area = 0.0;
        let n = self.junctions.len();

        for i in 0..n {
            let j1 = &self.junctions[i];
            let j2 = &self.junctions[(i + 1) % n];
            area += (j1.x() as f64) * (j2.y() as f64);
            area -= (j2.x() as f64) * (j1.y() as f64);
        }

        area / 2.0
    }

    /// Calculate the absolute area enclosed by this line.
    pub fn area(&self) -> CoordF {
        self.signed_area().abs()
    }

    /// Convert to a polygon (loses width information).
    pub fn to_polygon(&self) -> Polygon {
        Polygon::from_points(self.junctions.iter().map(|j| j.position).collect())
    }

    /// Convert to a polyline (loses width information).
    pub fn to_polyline(&self) -> Polyline {
        Polyline::from_points(self.junctions.iter().map(|j| j.position).collect())
    }

    /// Get just the points (positions) from this line.
    pub fn points(&self) -> Vec<Point> {
        self.junctions.iter().map(|j| j.position).collect()
    }

    /// Get the widths at each junction.
    pub fn widths(&self) -> Vec<Coord> {
        self.junctions.iter().map(|j| j.width).collect()
    }

    /// Get the widths in millimeters.
    pub fn widths_mm(&self) -> Vec<CoordF> {
        self.junctions.iter().map(|j| j.width_mm()).collect()
    }

    /// Simplify the line by removing junctions that don't significantly
    /// affect the path geometry or width distribution.
    ///
    /// # Arguments
    /// * `min_segment_length` - Minimum length between junctions (scaled)
    /// * `max_deviation` - Maximum allowed deviation from original path (scaled)
    /// * `max_width_deviation` - Maximum allowed width change to skip a junction
    pub fn simplify(
        &mut self,
        min_segment_length: Coord,
        max_deviation: Coord,
        max_width_deviation: Coord,
    ) {
        if self.junctions.len() <= 2 {
            return;
        }

        let mut result = Vec::with_capacity(self.junctions.len());
        result.push(self.junctions[0].clone());

        let mut i = 1;
        while i < self.junctions.len() - 1 {
            let prev = result.last().unwrap();
            let curr = &self.junctions[i];
            let next = &self.junctions[i + 1];

            // Check if we should keep this junction
            let segment_length_sq = prev.distance_squared_to(curr);
            let width_diff = (curr.width - prev.width).abs();
            let width_diff_next = (next.width - curr.width).abs();

            // Keep if segment is long enough or width changes significantly
            let min_segment_sq = (min_segment_length as i128) * (min_segment_length as i128);
            let keep = segment_length_sq > min_segment_sq
                || width_diff > max_width_deviation
                || width_diff_next > max_width_deviation;

            // Also keep if removing would cause too much deviation
            let keep = keep || {
                // Calculate distance from curr to line prev->next
                let deviation =
                    Self::point_line_distance(curr.position, prev.position, next.position);
                deviation > max_deviation as f64
            };

            if keep {
                result.push(curr.clone());
            }
            i += 1;
        }

        // Always keep the last junction
        if self.junctions.len() >= 2 {
            result.push(self.junctions.last().unwrap().clone());
        }

        self.junctions = result;
    }

    /// Calculate perpendicular distance from point to line segment.
    fn point_line_distance(point: Point, line_start: Point, line_end: Point) -> f64 {
        let dx = (line_end.x - line_start.x) as f64;
        let dy = (line_end.y - line_start.y) as f64;
        let length_sq = dx * dx + dy * dy;

        if length_sq < 1e-10 {
            // Line segment is essentially a point
            return ((point.x - line_start.x) as f64).hypot((point.y - line_start.y) as f64);
        }

        // Project point onto line
        let t = (((point.x - line_start.x) as f64 * dx) + ((point.y - line_start.y) as f64 * dy))
            / length_sq;
        let t = t.clamp(0.0, 1.0);

        let proj_x = line_start.x as f64 + t * dx;
        let proj_y = line_start.y as f64 + t * dy;

        ((point.x as f64 - proj_x).powi(2) + (point.y as f64 - proj_y).powi(2)).sqrt()
    }

    /// Iterator over junctions.
    pub fn iter(&self) -> impl Iterator<Item = &ExtrusionJunction> {
        self.junctions.iter()
    }

    /// Mutable iterator over junctions.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut ExtrusionJunction> {
        self.junctions.iter_mut()
    }
}

impl IntoIterator for ExtrusionLine {
    type Item = ExtrusionJunction;
    type IntoIter = std::vec::IntoIter<ExtrusionJunction>;

    fn into_iter(self) -> Self::IntoIter {
        self.junctions.into_iter()
    }
}

impl<'a> IntoIterator for &'a ExtrusionLine {
    type Item = &'a ExtrusionJunction;
    type IntoIter = std::slice::Iter<'a, ExtrusionJunction>;

    fn into_iter(self) -> Self::IntoIter {
        self.junctions.iter()
    }
}

impl std::ops::Index<usize> for ExtrusionLine {
    type Output = ExtrusionJunction;

    fn index(&self, index: usize) -> &Self::Output {
        &self.junctions[index]
    }
}

impl std::ops::IndexMut<usize> for ExtrusionLine {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.junctions[index]
    }
}

/// A collection of variable-width extrusion lines.
pub type VariableWidthLines = Vec<ExtrusionLine>;

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_line() -> ExtrusionLine {
        let mut line = ExtrusionLine::closed(0);
        line.push(ExtrusionJunction::new(Point::new(0, 0), scale(0.4), 0));
        line.push(ExtrusionJunction::new(
            Point::new(scale(10.0), 0),
            scale(0.45),
            0,
        ));
        line.push(ExtrusionJunction::new(
            Point::new(scale(10.0), scale(10.0)),
            scale(0.5),
            0,
        ));
        line.push(ExtrusionJunction::new(
            Point::new(0, scale(10.0)),
            scale(0.4),
            0,
        ));
        line
    }

    #[test]
    fn test_extrusion_line_new() {
        let line = ExtrusionLine::new(1, true, false);
        assert!(line.is_empty());
        assert_eq!(line.inset_idx, 1);
        assert!(line.is_odd);
        assert!(!line.is_closed);
    }

    #[test]
    fn test_extrusion_line_closed() {
        let line = ExtrusionLine::closed(0);
        assert!(line.is_closed);
        assert!(!line.is_odd);
        assert!(line.is_external());
    }

    #[test]
    fn test_extrusion_line_from_polygon() {
        let polygon = Polygon::rectangle(Point::new(0, 0), Point::new(scale(10.0), scale(10.0)));
        let line = ExtrusionLine::from_polygon(&polygon, scale(0.4), 0);

        assert!(line.is_closed);
        assert_eq!(line.len(), polygon.points().len());
        assert!(line.is_external());
    }

    #[test]
    fn test_extrusion_line_length() {
        let line = make_test_line();
        // 10 + 10 + 10 + 10 = 40mm perimeter
        let length = line.length_mm();
        assert!((length - 40.0).abs() < 0.01);
    }

    #[test]
    fn test_extrusion_line_width_stats() {
        let line = make_test_line();

        assert!((unscale(line.min_width()) - 0.4).abs() < 0.001);
        assert!((unscale(line.max_width()) - 0.5).abs() < 0.001);

        // Average should be between min and max
        let avg = line.average_width_mm();
        assert!(avg >= 0.4 && avg <= 0.5);
    }

    #[test]
    fn test_extrusion_line_area() {
        let line = make_test_line();
        // 10mm x 10mm = 100mm²
        let area_mm2 = line.area() / (crate::SCALING_FACTOR * crate::SCALING_FACTOR);
        assert!((area_mm2 - 100.0).abs() < 0.1);
    }

    #[test]
    fn test_extrusion_line_is_contour() {
        let mut line = ExtrusionLine::closed(0);
        // Counter-clockwise square (contour)
        line.push(ExtrusionJunction::new(Point::new(0, 0), scale(0.4), 0));
        line.push(ExtrusionJunction::new(
            Point::new(scale(10.0), 0),
            scale(0.4),
            0,
        ));
        line.push(ExtrusionJunction::new(
            Point::new(scale(10.0), scale(10.0)),
            scale(0.4),
            0,
        ));
        line.push(ExtrusionJunction::new(
            Point::new(0, scale(10.0)),
            scale(0.4),
            0,
        ));

        assert!(line.is_contour());
    }

    #[test]
    fn test_extrusion_line_to_polygon() {
        let line = make_test_line();
        let polygon = line.to_polygon();

        assert_eq!(polygon.points().len(), line.len());
    }

    #[test]
    fn test_extrusion_line_points() {
        let line = make_test_line();
        let points = line.points();

        assert_eq!(points.len(), 4);
        assert_eq!(points[0], Point::new(0, 0));
    }

    #[test]
    fn test_extrusion_line_widths() {
        let line = make_test_line();
        let widths = line.widths_mm();

        assert_eq!(widths.len(), 4);
        assert!((widths[0] - 0.4).abs() < 0.001);
        assert!((widths[1] - 0.45).abs() < 0.001);
        assert!((widths[2] - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_extrusion_line_reverse() {
        let mut line = make_test_line();
        let first_width = line.first().unwrap().width;
        let last_width = line.last().unwrap().width;

        line.reverse();

        assert_eq!(line.first().unwrap().width, last_width);
        assert_eq!(line.last().unwrap().width, first_width);
    }

    #[test]
    fn test_extrusion_line_indexing() {
        let line = make_test_line();

        assert_eq!(line[0].position, Point::new(0, 0));
        assert!((line[1].width_mm() - 0.45).abs() < 0.001);
    }

    #[test]
    fn test_extrusion_line_iteration() {
        let line = make_test_line();
        let count = line.iter().count();

        assert_eq!(count, 4);
    }
}
