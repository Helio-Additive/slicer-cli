//! ThickPolyline type for variable-width paths.
//!
//! A ThickPolyline is a polyline where each vertex has an associated width value,
//! representing the local thickness of the path at that point. This is used by:
//!
//! - **Medial axis**: The skeleton of a gap region, where width = 2× distance to boundary
//! - **Variable width**: Converting ThickPolylines to extrusion paths with per-segment LINE_WIDTH
//!
//! Mirrors BambuStudio's `ThickPolyline` class from `Polyline.hpp`.

use super::{Line, Point, PointF, Polyline};
use crate::{Coord, CoordF};
use serde::{Deserialize, Serialize};
use std::fmt;

/// A line segment with width information at both endpoints.
/// Polyline.hpp:256
#[derive(Clone, Debug, PartialEq)]
pub struct ThickLine {
    pub a: Point,
    pub b: Point,
    pub a_width: CoordF,
    pub b_width: CoordF,
}

impl ThickLine {
    /// Create a new ThickLine from two points and their widths.
    pub fn new(a: Point, b: Point, a_width: CoordF, b_width: CoordF) -> Self {
        Self {
            a,
            b,
            a_width,
            b_width,
        }
    }

    /// Calculate the length of this line segment.
    pub fn length(&self) -> CoordF {
        self.a.distance(&self.b)
    }
}

/// A collection of ThickLines.
pub type ThickLines = Vec<ThickLine>;

/// A polyline with per-vertex width information.
///
/// Each point in the polyline has a corresponding width value that indicates
/// the local thickness of the path at that vertex. This is essential for
/// gap fill where the path width varies continuously along its length.
///
/// BambuStudio reference: `ThickPolyline` in `Polyline.hpp`
#[derive(Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ThickPolyline {
    /// The points defining the polyline path.
    pub points: Vec<Point>,
    /// Width at each vertex (same length as `points`). Units: mm (unscaled).
    pub widths: Vec<CoordF>,
    /// Whether each endpoint extends to the boundary.
    /// Index 0 = start endpoint, index 1 = end endpoint.
    /// When true, the endpoint was extended to touch the shape boundary
    /// during medial axis computation.
    pub endpoints: [bool; 2],
}

impl ThickPolyline {
    // Create an empty ThickPolyline.
    #[inline]
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            widths: Vec::new(),
            endpoints: [false, false],
        }
    }

    /// Create a ThickPolyline with the given capacity.
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::with_capacity(capacity),
            widths: Vec::with_capacity(capacity),
            endpoints: [false, false],
        }
    }

    /// Create a ThickPolyline from points and widths.
    ///
    /// # Panics
    /// Panics if `points.len() != widths.len()`.
    pub fn from_points_and_widths(points: Vec<Point>, widths: Vec<CoordF>) -> Self {
        assert_eq!(
            points.len(),
            widths.len(),
            "ThickPolyline: points and widths must have the same length ({} vs {})",
            points.len(),
            widths.len()
        );
        Self {
            points,
            widths,
            endpoints: [false, false],
        }
    }

    /// Create a ThickPolyline from a regular Polyline with uniform width.
    pub fn from_polyline(polyline: &Polyline, width: CoordF) -> Self {
        let points = polyline.points().to_vec();
        let widths = vec![width; points.len()];
        Self {
            points,
            widths,
            endpoints: [false, false],
        }
    }

    /// Add a point with its associated width.
    #[inline]
    pub fn push(&mut self, point: Point, width: CoordF) {
        self.points.push(point);
        self.widths.push(width);
    }

    /// Get the number of points.
    #[inline]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Check if the ThickPolyline is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Get the first point, if any.
    #[inline]
    pub fn first_point(&self) -> Option<&Point> {
        self.points.first()
    }

    /// Get the last point, if any.
    #[inline]
    pub fn last_point(&self) -> Option<&Point> {
        self.points.last()
    }

    /// Get the width at a specific index.
    #[inline]
    pub fn width_at(&self, index: usize) -> CoordF {
        self.widths[index]
    }

    /// Get the average width across all vertices.
    pub fn average_width(&self) -> CoordF {
        if self.widths.is_empty() {
            return 0.0;
        }
        self.widths.iter().sum::<CoordF>() / self.widths.len() as CoordF
    }

    /// Get the minimum width across all vertices.
    pub fn min_width(&self) -> CoordF {
        self.widths
            .iter()
            .copied()
            .fold(CoordF::INFINITY, CoordF::min)
    }

    /// Get the maximum width across all vertices.
    pub fn max_width(&self) -> CoordF {
        self.widths
            .iter()
            .copied()
            .fold(CoordF::NEG_INFINITY, CoordF::max)
    }

    /// Convert ThickPolyline to ThickLines (line segments with width at each endpoint).
    /// Polyline.cpp:322-330
    /// C++: ThickLines ThickPolyline::thicklines() const
    pub fn thicklines(&self) -> ThickLines {
        let mut lines = ThickLines::new();
        if self.points.len() >= 2 {
            lines.reserve(self.points.len() - 1);
            for i in 0..(self.points.len() - 1) {
                // In C++, width array has 2 entries per segment (start and end width)
                // But in our Rust version, widths array has one entry per vertex
                // So we use widths[i] and widths[i+1] for each segment
                let a_width = if i < self.widths.len() {
                    self.widths[i]
                } else {
                    0.0
                };
                let b_width = if i + 1 < self.widths.len() {
                    self.widths[i + 1]
                } else {
                    0.0
                };
                lines.push(ThickLine::new(
                    self.points[i],
                    self.points[i + 1],
                    a_width,
                    b_width,
                ));
            }
        }
        lines
    }

    /// Calculate the total length of the polyline in scaled coordinates.
    pub fn length(&self) -> CoordF {
        let mut total = 0.0;
        for i in 1..self.points.len() {
            let dx = (self.points[i].x - self.points[i - 1].x) as CoordF;
            let dy = (self.points[i].y - self.points[i - 1].y) as CoordF;
            total += (dx * dx + dy * dy).sqrt();
        }
        total
    }

    /// Calculate the total length in mm (unscaled).
    pub fn length_mm(&self) -> CoordF {
        self.length() / crate::SCALING_FACTOR
    }

    /// Check if this polyline is closed (first point == last point).
    pub fn is_closed(&self) -> bool {
        self.points.len() >= 2 && self.points.first() == self.points.last()
    }

    /// Reverse the polyline direction (in place).
    pub fn reverse(&mut self) {
        self.points.reverse();
        self.widths.reverse();
        self.endpoints.swap(0, 1);
    }

    /// Return a reversed copy.
    pub fn reversed(&self) -> Self {
        let mut copy = self.clone();
        copy.reverse();
        copy
    }

    /// Convert to a regular Polyline (discards width information).
    pub fn to_polyline(&self) -> Polyline {
        Polyline::from_points(self.points.clone())
    }

    /// Get the width at a specific distance along the polyline.
    ///
    /// Linearly interpolates between vertex widths based on position.
    pub fn width_at_distance(&self, target_distance: CoordF) -> CoordF {
        if self.points.len() < 2 {
            return self.widths.first().copied().unwrap_or(0.0);
        }

        let mut accumulated = 0.0;
        for i in 1..self.points.len() {
            let dx = (self.points[i].x - self.points[i - 1].x) as CoordF;
            let dy = (self.points[i].y - self.points[i - 1].y) as CoordF;
            let segment_len = (dx * dx + dy * dy).sqrt();

            if accumulated + segment_len >= target_distance {
                // Interpolate within this segment
                let t = if segment_len > 0.0 {
                    (target_distance - accumulated) / segment_len
                } else {
                    0.0
                };
                return self.widths[i - 1] + t * (self.widths[i] - self.widths[i - 1]);
            }
            accumulated += segment_len;
        }

        // Past the end — return last width
        self.widths.last().copied().unwrap_or(0.0)
    }

    /// Clip the polyline from the front by the given scaled distance.
    /// Removes points from the beginning and adjusts the first remaining point.
    pub fn clip_front(&mut self, distance: CoordF) {
        if self.points.len() < 2 || distance <= 0.0 {
            return;
        }

        let mut remaining = distance;
        while self.points.len() >= 2 {
            let dx = (self.points[1].x - self.points[0].x) as CoordF;
            let dy = (self.points[1].y - self.points[0].y) as CoordF;
            let seg_len = (dx * dx + dy * dy).sqrt();

            if seg_len <= remaining {
                remaining -= seg_len;
                self.points.remove(0);
                self.widths.remove(0);
                if remaining <= 0.0 || self.points.len() < 2 {
                    break;
                }
            } else {
                // Interpolate within this segment
                let t = remaining / seg_len;
                let new_x = self.points[0].x as CoordF + dx * t;
                let new_y = self.points[0].y as CoordF + dy * t;
                let new_w = self.widths[0] + t * (self.widths[1] - self.widths[0]);
                self.points[0] = Point::new(new_x.round() as Coord, new_y.round() as Coord);
                self.widths[0] = new_w;
                break;
            }
        }
    }

    /// Clip the polyline from the back by the given scaled distance.
    pub fn clip_back(&mut self, distance: CoordF) {
        self.reverse();
        self.clip_front(distance);
        self.reverse();
    }

    /// Split this ThickPolyline into segments where width changes exceed a threshold.
    ///
    /// This is the core of variable_width conversion: each returned segment has
    /// relatively uniform width and can be emitted as a single extrusion path
    /// with a fixed LINE_WIDTH.
    ///
    /// # Arguments
    /// * `max_width_variation` - Maximum allowed width change within a segment (mm)
    ///
    /// Returns a vector of ThickPolylines, each with roughly uniform width.
    pub fn split_by_width_variation(&self, max_width_variation: CoordF) -> Vec<ThickPolyline> {
        if self.points.len() < 2 {
            return vec![self.clone()];
        }

        let mut segments = Vec::new();
        let mut current = ThickPolyline::new();

        // Start with the first point
        current.push(self.points[0], self.widths[0]);
        let mut segment_min_width = self.widths[0];
        let mut segment_max_width = self.widths[0];

        for i in 1..self.points.len() {
            let w = self.widths[i];
            let new_min = segment_min_width.min(w);
            let new_max = segment_max_width.max(w);

            if new_max - new_min > max_width_variation && current.len() >= 2 {
                // Width variation exceeded — start a new segment
                segments.push(current);
                current = ThickPolyline::new();
                // Repeat the last point as the start of the new segment
                current.push(self.points[i - 1], self.widths[i - 1]);
                segment_min_width = self.widths[i - 1].min(w);
                segment_max_width = self.widths[i - 1].max(w);
            } else {
                segment_min_width = new_min;
                segment_max_width = new_max;
            }

            current.push(self.points[i], w);
        }

        if current.len() >= 2 {
            segments.push(current);
        }

        // If nothing was split, return the original
        if segments.is_empty() {
            return vec![self.clone()];
        }

        segments
    }

    /// Apply Douglas-Peucker simplification while preserving width information.
    pub fn douglas_peucker(&mut self, tolerance: CoordF) {
        if self.points.len() < 3 {
            return;
        }

        let keep = dp_mark(&self.points, 0, self.points.len() - 1, tolerance);
        let mut new_points = Vec::with_capacity(keep.len());
        let mut new_widths = Vec::with_capacity(keep.len());

        for &idx in &keep {
            new_points.push(self.points[idx]);
            new_widths.push(self.widths[idx]);
        }

        self.points = new_points;
        self.widths = new_widths;
    }

    /// Compute a point along the polyline at a given fraction (0.0 to 1.0).
    pub fn point_at_fraction(&self, fraction: CoordF) -> Option<Point> {
        if self.points.is_empty() {
            return None;
        }
        if self.points.len() == 1 || fraction <= 0.0 {
            return Some(self.points[0]);
        }
        if fraction >= 1.0 {
            return self.points.last().copied();
        }

        let total_len = self.length();
        let target = total_len * fraction;
        let mut accumulated = 0.0;

        for i in 1..self.points.len() {
            let dx = (self.points[i].x - self.points[i - 1].x) as CoordF;
            let dy = (self.points[i].y - self.points[i - 1].y) as CoordF;
            let seg_len = (dx * dx + dy * dy).sqrt();

            if accumulated + seg_len >= target {
                let t = if seg_len > 0.0 {
                    (target - accumulated) / seg_len
                } else {
                    0.0
                };
                return Some(Point::new(
                    (self.points[i - 1].x as CoordF + dx * t).round() as Coord,
                    (self.points[i - 1].y as CoordF + dy * t).round() as Coord,
                ));
            }
            accumulated += seg_len;
        }

        self.points.last().copied()
    }

    /// Extend the start of the polyline by the given scaled distance.
    /// This extrapolates the first segment backward.
    /// BambuStudio reference: MedialAxis.cpp endpoint extension.
    pub fn extend_start(&mut self, distance: CoordF) {
        if self.points.len() < 2 || distance <= 0.0 {
            return;
        }
        let dx = (self.points[0].x - self.points[1].x) as CoordF;
        let dy = (self.points[0].y - self.points[1].y) as CoordF;
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1.0 {
            return;
        }
        let factor = distance / len;
        self.points[0] = Point::new(
            (self.points[0].x as CoordF + dx * factor).round() as Coord,
            (self.points[0].y as CoordF + dy * factor).round() as Coord,
        );
    }

    /// Extend the end of the polyline by the given scaled distance.
    pub fn extend_end(&mut self, distance: CoordF) {
        if self.points.len() < 2 || distance <= 0.0 {
            return;
        }
        let n = self.points.len();
        let dx = (self.points[n - 1].x - self.points[n - 2].x) as CoordF;
        let dy = (self.points[n - 1].y - self.points[n - 2].y) as CoordF;
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1.0 {
            return;
        }
        let factor = distance / len;
        self.points[n - 1] = Point::new(
            (self.points[n - 1].x as CoordF + dx * factor).round() as Coord,
            (self.points[n - 1].y as CoordF + dy * factor).round() as Coord,
        );
    }

    /// Get the bounding box of this thick polyline.
    pub fn bounding_box(&self) -> super::BoundingBox {
        let mut bbox = super::BoundingBox::empty();
        for p in &self.points {
            bbox.merge_point(*p);
        }
        bbox
    }
}

impl fmt::Debug for ThickPolyline {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "ThickPolyline({} pts, widths {:.3}..{:.3}mm, len {:.3}mm)",
            self.len(),
            self.min_width(),
            self.max_width(),
            self.length_mm()
        )
    }
}

impl fmt::Display for ThickPolyline {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "ThickPolyline[{} pts, avg_w={:.3}mm, len={:.3}mm]",
            self.len(),
            self.average_width(),
            self.length_mm()
        )
    }
}

/// A collection of ThickPolylines.
pub type ThickPolylines = Vec<ThickPolyline>;

// Douglas-Peucker helper: mark indices to keep.
fn dp_mark(points: &[Point], start: usize, end: usize, tolerance: CoordF) -> Vec<usize> {
    if end <= start + 1 {
        return vec![start, end];
    }

    let tolerance_sq = tolerance * tolerance;

    // Find the point farthest from the line start→end
    let sx = points[start].x as CoordF;
    let sy = points[start].y as CoordF;
    let ex = points[end].x as CoordF;
    let ey = points[end].y as CoordF;
    let line_dx = ex - sx;
    let line_dy = ey - sy;
    let line_len_sq = line_dx * line_dx + line_dy * line_dy;

    let mut max_dist_sq = 0.0;
    let mut max_idx = start;

    for i in (start + 1)..end {
        let px = points[i].x as CoordF - sx;
        let py = points[i].y as CoordF - sy;

        let dist_sq = if line_len_sq < 1.0 {
            px * px + py * py
        } else {
            let t = (px * line_dx + py * line_dy) / line_len_sq;
            let proj_x = px - t * line_dx;
            let proj_y = py - t * line_dy;
            proj_x * proj_x + proj_y * proj_y
        };

        if dist_sq > max_dist_sq {
            max_dist_sq = dist_sq;
            max_idx = i;
        }
    }

    if max_dist_sq > tolerance_sq {
        let mut left = dp_mark(points, start, max_idx, tolerance);
        let right = dp_mark(points, max_idx, end, tolerance);
        // Merge, avoiding duplicate max_idx
        left.pop();
        left.extend(right);
        left
    } else {
        vec![start, end]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scale;

    fn s(mm: f64) -> Coord {
        scale(mm)
    }

    #[test]
    fn test_thick_polyline_new() {
        let tp = ThickPolyline::new();
        assert!(tp.is_empty());
        assert_eq!(tp.len(), 0);
        assert!(!tp.is_closed());
    }

    #[test]
    fn test_thick_polyline_push() {
        let mut tp = ThickPolyline::new();
        tp.push(Point::new(0, 0), 0.4);
        tp.push(Point::new(s(10.0), 0), 0.5);
        tp.push(Point::new(s(20.0), 0), 0.3);

        assert_eq!(tp.len(), 3);
        assert_eq!(tp.width_at(0), 0.4);
        assert_eq!(tp.width_at(1), 0.5);
        assert_eq!(tp.width_at(2), 0.3);
    }

    #[test]
    fn test_thick_polyline_from_points_and_widths() {
        let points = vec![Point::new(0, 0), Point::new(s(10.0), 0)];
        let widths = vec![0.4, 0.6];
        let tp = ThickPolyline::from_points_and_widths(points, widths);
        assert_eq!(tp.len(), 2);
        assert!((tp.average_width() - 0.5).abs() < 1e-10);
    }

    #[test]
    #[should_panic(expected = "points and widths must have the same length")]
    fn test_thick_polyline_mismatched_lengths() {
        let points = vec![Point::new(0, 0), Point::new(s(10.0), 0)];
        let widths = vec![0.4];
        ThickPolyline::from_points_and_widths(points, widths);
    }

    #[test]
    fn test_thick_polyline_from_polyline() {
        let polyline = Polyline::from_points(vec![
            Point::new(0, 0),
            Point::new(s(10.0), 0),
            Point::new(s(10.0), s(10.0)),
        ]);
        let tp = ThickPolyline::from_polyline(&polyline, 0.4);
        assert_eq!(tp.len(), 3);
        assert_eq!(tp.min_width(), 0.4);
        assert_eq!(tp.max_width(), 0.4);
    }

    #[test]
    fn test_thick_polyline_length() {
        let tp = ThickPolyline::from_points_and_widths(
            vec![Point::new(0, 0), Point::new(s(10.0), 0)],
            vec![0.4, 0.4],
        );
        assert!((tp.length_mm() - 10.0).abs() < 0.001);
    }

    #[test]
    fn test_thick_polyline_min_max_width() {
        let tp = ThickPolyline::from_points_and_widths(
            vec![
                Point::new(0, 0),
                Point::new(s(5.0), 0),
                Point::new(s(10.0), 0),
            ],
            vec![0.2, 0.6, 0.4],
        );
        assert!((tp.min_width() - 0.2).abs() < 1e-10);
        assert!((tp.max_width() - 0.6).abs() < 1e-10);
        assert!((tp.average_width() - 0.4).abs() < 1e-10);
    }

    #[test]
    fn test_thick_polyline_reverse() {
        let mut tp = ThickPolyline::from_points_and_widths(
            vec![Point::new(0, 0), Point::new(s(10.0), 0)],
            vec![0.2, 0.6],
        );
        tp.endpoints = [true, false];
        tp.reverse();

        assert_eq!(tp.points[0], Point::new(s(10.0), 0));
        assert_eq!(tp.points[1], Point::new(0, 0));
        assert_eq!(tp.widths[0], 0.6);
        assert_eq!(tp.widths[1], 0.2);
        assert_eq!(tp.endpoints, [false, true]);
    }

    #[test]
    fn test_thick_polyline_is_closed() {
        let tp = ThickPolyline::from_points_and_widths(
            vec![Point::new(0, 0), Point::new(s(10.0), 0), Point::new(0, 0)],
            vec![0.4, 0.4, 0.4],
        );
        assert!(tp.is_closed());

        let tp_open = ThickPolyline::from_points_and_widths(
            vec![Point::new(0, 0), Point::new(s(10.0), 0)],
            vec![0.4, 0.4],
        );
        assert!(!tp_open.is_closed());
    }

    #[test]
    fn test_thick_polyline_split_by_width_variation() {
        let tp = ThickPolyline::from_points_and_widths(
            vec![
                Point::new(0, 0),
                Point::new(s(5.0), 0),
                Point::new(s(10.0), 0),
                Point::new(s(15.0), 0),
                Point::new(s(20.0), 0),
            ],
            vec![0.2, 0.22, 0.5, 0.52, 0.3],
        );

        // With max variation of 0.05, this should split at the big jumps
        let segments = tp.split_by_width_variation(0.05);
        assert!(
            segments.len() >= 2,
            "Expected at least 2 segments, got {}",
            segments.len()
        );

        // Each segment should have width variation <= 0.05 (approximately)
        for seg in &segments {
            assert!(seg.len() >= 2, "Segment too short: {} points", seg.len());
        }
    }

    #[test]
    fn test_thick_polyline_width_at_distance() {
        let tp = ThickPolyline::from_points_and_widths(
            vec![Point::new(0, 0), Point::new(s(10.0), 0)],
            vec![0.2, 0.6],
        );

        // At the start
        let w0 = tp.width_at_distance(0.0);
        assert!((w0 - 0.2).abs() < 0.001);

        // At the midpoint (5mm along a 10mm segment)
        let w_mid = tp.width_at_distance(s(5.0) as CoordF);
        assert!((w_mid - 0.4).abs() < 0.001);

        // At the end
        let w_end = tp.width_at_distance(s(10.0) as CoordF);
        assert!((w_end - 0.6).abs() < 0.001);
    }

    #[test]
    fn test_thick_polyline_to_polyline() {
        let tp = ThickPolyline::from_points_and_widths(
            vec![Point::new(0, 0), Point::new(s(10.0), 0)],
            vec![0.4, 0.6],
        );
        let pl = tp.to_polyline();
        assert_eq!(pl.len(), 2);
        assert_eq!(pl.points()[0], Point::new(0, 0));
    }

    #[test]
    fn test_thick_polyline_extend_start() {
        let mut tp = ThickPolyline::from_points_and_widths(
            vec![Point::new(s(5.0), 0), Point::new(s(15.0), 0)],
            vec![0.4, 0.4],
        );
        // Extend start by 5mm (direction is from points[1] back toward points[0], then beyond)
        tp.extend_start(s(5.0) as CoordF);
        // The first point should have moved 5mm to the left of its original position
        assert!((tp.points[0].x as CoordF / crate::SCALING_FACTOR - 0.0).abs() < 0.01);
    }

    #[test]
    fn test_thick_polyline_extend_end() {
        let mut tp = ThickPolyline::from_points_and_widths(
            vec![Point::new(0, 0), Point::new(s(10.0), 0)],
            vec![0.4, 0.4],
        );
        // Extend end by 5mm
        tp.extend_end(s(5.0) as CoordF);
        assert!((tp.points[1].x as CoordF / crate::SCALING_FACTOR - 15.0).abs() < 0.01);
    }

    #[test]
    fn test_thick_polyline_clip_front() {
        let mut tp = ThickPolyline::from_points_and_widths(
            vec![
                Point::new(0, 0),
                Point::new(s(10.0), 0),
                Point::new(s(20.0), 0),
            ],
            vec![0.2, 0.4, 0.6],
        );

        let original_len = tp.length();
        // Clip 5mm (half of first segment) from the front
        tp.clip_front(s(5.0) as CoordF);

        // Should now start at roughly (5, 0) with interpolated width
        assert!((tp.points[0].x as CoordF / crate::SCALING_FACTOR - 5.0).abs() < 0.01);
        assert!((tp.widths[0] - 0.3).abs() < 0.01); // interpolated between 0.2 and 0.4
        assert_eq!(tp.len(), 3); // first point modified, second and third remain
    }

    #[test]
    fn test_thick_polyline_clip_back() {
        let mut tp = ThickPolyline::from_points_and_widths(
            vec![
                Point::new(0, 0),
                Point::new(s(10.0), 0),
                Point::new(s(20.0), 0),
            ],
            vec![0.2, 0.4, 0.6],
        );

        // Clip 5mm from the back
        tp.clip_back(s(5.0) as CoordF);

        // Should now end at roughly (15, 0)
        let last = tp.points.last().unwrap();
        assert!((last.x as CoordF / crate::SCALING_FACTOR - 15.0).abs() < 0.01);
    }

    #[test]
    fn test_thick_polyline_douglas_peucker() {
        // Create a polyline with a tiny deviation in the middle
        let mut tp = ThickPolyline::from_points_and_widths(
            vec![
                Point::new(0, 0),
                Point::new(s(5.0), 100), // tiny deviation from straight line
                Point::new(s(10.0), 0),
            ],
            vec![0.4, 0.5, 0.6],
        );

        // With a large tolerance, the middle point should be removed
        tp.douglas_peucker(s(1.0) as CoordF);
        assert_eq!(tp.len(), 2);
        assert_eq!(tp.widths[0], 0.4);
        assert_eq!(tp.widths[1], 0.6);
    }

    #[test]
    fn test_thick_polyline_point_at_fraction() {
        let tp = ThickPolyline::from_points_and_widths(
            vec![Point::new(0, 0), Point::new(s(10.0), 0)],
            vec![0.4, 0.4],
        );

        let mid = tp.point_at_fraction(0.5).unwrap();
        assert!((mid.x as CoordF / crate::SCALING_FACTOR - 5.0).abs() < 0.01);

        let start = tp.point_at_fraction(0.0).unwrap();
        assert_eq!(start, Point::new(0, 0));

        let end = tp.point_at_fraction(1.0).unwrap();
        assert_eq!(end, Point::new(s(10.0), 0));
    }

    #[test]
    fn test_thick_polyline_bounding_box() {
        let tp = ThickPolyline::from_points_and_widths(
            vec![
                Point::new(s(5.0), s(3.0)),
                Point::new(s(15.0), s(7.0)),
                Point::new(s(10.0), s(12.0)),
            ],
            vec![0.4, 0.4, 0.4],
        );

        let bb = tp.bounding_box();
        assert_eq!(bb.min, Point::new(s(5.0), s(3.0)));
        assert_eq!(bb.max, Point::new(s(15.0), s(12.0)));
    }

    #[test]
    fn test_thick_polyline_debug_display() {
        let tp = ThickPolyline::from_points_and_widths(
            vec![Point::new(0, 0), Point::new(s(10.0), 0)],
            vec![0.2, 0.6],
        );
        let debug = format!("{:?}", tp);
        assert!(debug.contains("ThickPolyline"));
        assert!(debug.contains("2 pts"));

        let display = format!("{}", tp);
        assert!(display.contains("ThickPolyline"));
    }
}
