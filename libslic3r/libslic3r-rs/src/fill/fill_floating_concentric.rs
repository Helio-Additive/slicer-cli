//! Floating Concentric Infill Pattern
//!
//! This module implements the Floating Concentric infill pattern, which generates
//! concentric loops for top surfaces while detecting "floating" sections that are
//! not supported by the layer below.
//!
//! # Overview
//!
//! Floating Concentric is primarily used for top solid surfaces where print quality
//! is critical. Unlike regular concentric infill, it identifies which parts of the
//! infill lines are "floating" (not supported by infill or perimeters from the layer
//! below) and can adjust extrusion parameters accordingly.
//!
//! # Algorithm
//!
//! 1. Generate concentric loops from the fill area by repeatedly shrinking inward
//! 2. For each loop/line, detect which segments intersect with "floating areas"
//!    (regions not supported by the layer below)
//! 3. Mark each vertex/segment as floating or supported
//! 4. Optionally split paths at floating/supported transitions
//! 5. Reorder loop starting points to prefer non-floating areas (better seam quality)
//!
//! # BambuStudio Reference
//!
//! This module corresponds to:
//! - `src/libslic3r/Fill/FillFloatingConcentric.cpp`
//! - `src/libslic3r/Fill/FillFloatingConcentric.hpp`
//!
//! Key differences from C++ implementation:
//! - Uses point sampling for floating detection instead of Clipper_Z
//! - Simplified path merging without complex Z-value tracking
//! - Direct integration with existing concentric generation

use crate::clipper_utils::{shrink, OffsetJoinType};
use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon, Polyline};
use crate::{scale, unscale, Coord, CoordF};

/// Configuration for floating concentric infill generation.
#[derive(Debug, Clone)]
pub struct FloatingConcentricConfig {
    /// Line spacing in mm (distance between concentric loops).
    pub spacing: CoordF,

    /// Minimum loop length to include (mm). Loops shorter than this are dropped.
    pub min_loop_length: CoordF,

    /// Loop clipping distance (mm). Amount to clip from loop ends to avoid
    /// extruder getting exactly on first point.
    pub loop_clipping: CoordF,

    /// Number of sample points per mm for floating detection.
    pub samples_per_mm: CoordF,

    /// Whether to split paths at floating/supported transitions.
    pub split_at_transitions: bool,

    /// Whether to prefer starting loops at non-floating positions.
    pub prefer_non_floating_start: bool,

    /// Default line width for variable-width output (mm).
    pub default_width: CoordF,
}

impl Default for FloatingConcentricConfig {
    fn default() -> Self {
        Self {
            spacing: 0.4,
            min_loop_length: 1.0,
            loop_clipping: 0.15,
            samples_per_mm: 5.0,
            split_at_transitions: true,
            prefer_non_floating_start: true,
            default_width: 0.4,
        }
    }
}

impl FloatingConcentricConfig {
    // Create a new config with specified spacing.
    pub fn new(spacing: CoordF) -> Self {
        Self {
            spacing,
            ..Default::default()
        }
    }

    /// Builder method to set line spacing.
    pub fn with_spacing(mut self, spacing: CoordF) -> Self {
        self.spacing = spacing;
        self
    }

    /// Builder method to set minimum loop length.
    pub fn with_min_loop_length(mut self, min_length: CoordF) -> Self {
        self.min_loop_length = min_length;
        self
    }

    /// Builder method to set loop clipping distance.
    pub fn with_loop_clipping(mut self, clipping: CoordF) -> Self {
        self.loop_clipping = clipping;
        self
    }

    /// Builder method to set split at transitions.
    pub fn with_split_at_transitions(mut self, split: bool) -> Self {
        self.split_at_transitions = split;
        self
    }

    /// Builder method to set prefer non-floating start.
    pub fn with_prefer_non_floating_start(mut self, prefer: bool) -> Self {
        self.prefer_non_floating_start = prefer;
        self
    }

    /// Builder method to set default line width.
    pub fn with_default_width(mut self, width: CoordF) -> Self {
        self.default_width = width;
        self
    }
}

/// A thick line segment with floating flags at each endpoint.
#[derive(Debug, Clone)]
pub struct FloatingThickLine {
    /// Start point.
    pub a: Point,
    /// End point.
    pub b: Point,
    /// Width at start point.
    pub width_a: CoordF,
    /// Width at end point.
    pub width_b: CoordF,
    /// Whether start point is floating (unsupported).
    pub is_a_floating: bool,
    /// Whether end point is floating (unsupported).
    pub is_b_floating: bool,
}

impl FloatingThickLine {
    // Create a new floating thick line.
    pub fn new(
        a: Point,
        b: Point,
        width_a: CoordF,
        width_b: CoordF,
        is_a_floating: bool,
        is_b_floating: bool,
    ) -> Self {
        Self {
            a,
            b,
            width_a,
            width_b,
            is_a_floating,
            is_b_floating,
        }
    }

    /// Create a floating thick line with uniform width.
    pub fn with_uniform_width(a: Point, b: Point, width: CoordF, floating: bool) -> Self {
        Self {
            a,
            b,
            width_a: width,
            width_b: width,
            is_a_floating: floating,
            is_b_floating: floating,
        }
    }

    /// Get the length of this line segment.
    pub fn length(&self) -> CoordF {
        let dx = (self.b.x - self.a.x) as f64;
        let dy = (self.b.y - self.a.y) as f64;
        unscale((dx * dx + dy * dy).sqrt() as Coord)
    }

    /// Get the scaled length of this line segment.
    pub fn length_scaled(&self) -> Coord {
        let dx = (self.b.x - self.a.x) as f64;
        let dy = (self.b.y - self.a.y) as f64;
        (dx * dx + dy * dy).sqrt() as Coord
    }

    /// Check if the entire line is floating.
    pub fn is_fully_floating(&self) -> bool {
        self.is_a_floating && self.is_b_floating
    }

    /// Check if the entire line is supported (not floating).
    pub fn is_fully_supported(&self) -> bool {
        !self.is_a_floating && !self.is_b_floating
    }

    /// Check if this line has a floating transition.
    pub fn has_transition(&self) -> bool {
        self.is_a_floating != self.is_b_floating
    }

    /// Interpolate width at a given parameter t (0 = start, 1 = end).
    pub fn width_at(&self, t: CoordF) -> CoordF {
        self.width_a + t * (self.width_b - self.width_a)
    }

    /// Interpolate point at a given parameter t (0 = start, 1 = end).
    pub fn point_at(&self, t: CoordF) -> Point {
        Point::new(
            self.a.x + ((self.b.x - self.a.x) as f64 * t) as Coord,
            self.a.y + ((self.b.y - self.a.y) as f64 * t) as Coord,
        )
    }
}

/// A polyline with width and floating information at each vertex.
///
/// This is the core data structure for floating concentric infill.
/// Each vertex has an associated width and a flag indicating whether
/// it's floating (unsupported by the layer below).
#[derive(Debug, Clone)]
pub struct FloatingThickPolyline {
    /// The points of the polyline.
    pub points: Vec<Point>,
    /// Width at each vertex (same length as points).
    pub widths: Vec<CoordF>,
    /// Floating flag for each vertex (same length as points).
    pub is_floating: Vec<bool>,
}

impl FloatingThickPolyline {
    // Create a new empty floating thick polyline.
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            widths: Vec::new(),
            is_floating: Vec::new(),
        }
    }

    /// Create a floating thick polyline with capacity.
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::with_capacity(capacity),
            widths: Vec::with_capacity(capacity),
            is_floating: Vec::with_capacity(capacity),
        }
    }

    /// Create from a regular polyline with uniform width and no floating.
    pub fn from_polyline(polyline: &Polyline, width: CoordF) -> Self {
        let n = polyline.len();
        Self {
            points: polyline.points().to_vec(),
            widths: vec![width; n],
            is_floating: vec![false; n],
        }
    }

    /// Create from a polygon (closed loop) with uniform width and no floating.
    pub fn from_polygon(polygon: &Polygon, width: CoordF) -> Self {
        let mut points = polygon.points().to_vec();
        // Close the loop by adding first point at the end
        if !points.is_empty() && points.first() != points.last() {
            points.push(points[0]);
        }
        let n = points.len();
        Self {
            points,
            widths: vec![width; n],
            is_floating: vec![false; n],
        }
    }

    /// Get the number of points.
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Check if empty.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Check if this is a closed loop.
    pub fn is_closed(&self) -> bool {
        self.points.len() >= 3 && self.points.first() == self.points.last()
    }

    /// Get the first point.
    pub fn first_point(&self) -> Option<&Point> {
        self.points.first()
    }

    /// Get the last point.
    pub fn last_point(&self) -> Option<&Point> {
        self.points.last()
    }

    /// Push a new vertex.
    pub fn push(&mut self, point: Point, width: CoordF, floating: bool) {
        self.points.push(point);
        self.widths.push(width);
        self.is_floating.push(floating);
    }

    /// Get the total length of the polyline in mm.
    pub fn length(&self) -> CoordF {
        if self.points.len() < 2 {
            return 0.0;
        }

        let mut total = 0.0;
        for i in 0..self.points.len() - 1 {
            let dx = (self.points[i + 1].x - self.points[i].x) as f64;
            let dy = (self.points[i + 1].y - self.points[i].y) as f64;
            total += (dx * dx + dy * dy).sqrt();
        }
        unscale(total as Coord)
    }

    /// Check if this polyline is valid (has at least 2 points).
    pub fn is_valid(&self) -> bool {
        self.points.len() >= 2
    }

    /// Convert to floating thick lines.
    pub fn to_thick_lines(&self) -> Vec<FloatingThickLine> {
        if self.points.len() < 2 {
            return Vec::new();
        }

        let mut lines = Vec::with_capacity(self.points.len() - 1);
        for i in 0..self.points.len() - 1 {
            lines.push(FloatingThickLine::new(
                self.points[i],
                self.points[i + 1],
                self.widths[i],
                self.widths[i + 1],
                self.is_floating[i],
                self.is_floating[i + 1],
            ));
        }
        lines
    }

    /// Get the index of the first non-floating vertex.
    pub fn first_non_floating_index(&self) -> Option<usize> {
        self.is_floating.iter().position(|&f| !f)
    }

    /// Rebase a closed polyline to start at the given index.
    pub fn rebase_at(&self, idx: usize) -> Option<FloatingThickPolyline> {
        if !self.is_closed() || idx >= self.points.len() - 1 {
            return None;
        }

        let n = self.points.len() - 1; // Exclude duplicate closing point
        let mut result = FloatingThickPolyline::with_capacity(self.points.len());

        for j in 0..n {
            let src_idx = (idx + j) % n;
            result.push(
                self.points[src_idx],
                self.widths[src_idx],
                self.is_floating[src_idx],
            );
        }

        // Close the loop
        result.push(result.points[0], result.widths[0], result.is_floating[0]);

        Some(result)
    }

    /// Clip the end of the polyline by the specified distance (mm).
    pub fn clip_end(&mut self, distance: CoordF) {
        if self.points.len() < 2 || distance <= 0.0 {
            return;
        }

        let distance_scaled = scale(distance);
        let mut remaining = distance_scaled as f64;

        while self.points.len() >= 2 && remaining > 0.0 {
            let last_idx = self.points.len() - 1;
            let dx = (self.points[last_idx].x - self.points[last_idx - 1].x) as f64;
            let dy = (self.points[last_idx].y - self.points[last_idx - 1].y) as f64;
            let seg_len = (dx * dx + dy * dy).sqrt();

            if seg_len <= remaining {
                // Remove the last point entirely
                self.points.pop();
                self.widths.pop();
                self.is_floating.pop();
                remaining -= seg_len;
            } else {
                // Shorten the last segment
                let t = (seg_len - remaining) / seg_len;
                let new_x = self.points[last_idx - 1].x + (dx * t) as Coord;
                let new_y = self.points[last_idx - 1].y + (dy * t) as Coord;
                self.points[last_idx] = Point::new(new_x, new_y);
                // Interpolate width
                let w1 = self.widths[last_idx - 1];
                let w2 = self.widths[last_idx];
                self.widths[last_idx] = w1 + (w2 - w1) * t as CoordF;
                break;
            }
        }
    }

    /// Reverse the polyline in place.
    pub fn reverse(&mut self) {
        self.points.reverse();
        self.widths.reverse();
        self.is_floating.reverse();
    }

    /// Get a reversed copy of this polyline.
    pub fn reversed(&self) -> Self {
        let mut result = self.clone();
        result.reverse();
        result
    }

    /// Split this polyline into segments at floating/supported transitions.
    pub fn split_at_transitions(&self) -> Vec<FloatingThickPolyline> {
        if self.points.len() < 2 {
            return vec![self.clone()];
        }

        let mut segments = Vec::new();
        let mut current = FloatingThickPolyline::new();

        for i in 0..self.points.len() {
            let is_first = current.is_empty();
            current.push(self.points[i], self.widths[i], self.is_floating[i]);

            // Check if we have a transition
            if !is_first && i < self.points.len() {
                let prev_floating = self.is_floating[i - 1];
                let curr_floating = self.is_floating[i];

                if prev_floating != curr_floating && current.len() >= 2 {
                    // End current segment and start new one
                    segments.push(current);
                    current = FloatingThickPolyline::new();
                    // Start new segment with current point
                    current.push(self.points[i], self.widths[i], self.is_floating[i]);
                }
            }
        }

        if current.len() >= 2 {
            segments.push(current);
        }

        if segments.is_empty() {
            segments.push(self.clone());
        }

        segments
    }

    /// Get the fraction of the polyline that is floating.
    pub fn floating_fraction(&self) -> CoordF {
        if self.points.len() < 2 {
            return 0.0;
        }

        let mut floating_length = 0.0;
        let mut total_length = 0.0;

        for i in 0..self.points.len() - 1 {
            let dx = (self.points[i + 1].x - self.points[i].x) as f64;
            let dy = (self.points[i + 1].y - self.points[i].y) as f64;
            let seg_len = (dx * dx + dy * dy).sqrt();

            total_length += seg_len;

            // Consider a segment floating if both endpoints are floating
            if self.is_floating[i] && self.is_floating[i + 1] {
                floating_length += seg_len;
            }
        }

        if total_length > 0.0 {
            floating_length / total_length
        } else {
            0.0
        }
    }

    /// Convert to a regular polyline (loses width and floating info).
    pub fn to_polyline(&self) -> Polyline {
        Polyline::from_points(self.points.clone())
    }
}

impl Default for FloatingThickPolyline {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of floating concentric generation.
#[derive(Debug, Clone)]
pub struct FloatingConcentricResult {
    /// The generated floating thick polylines.
    pub polylines: Vec<FloatingThickPolyline>,
    /// Total length of all polylines in mm.
    pub total_length_mm: CoordF,
    /// Fraction of total length that is floating (0.0 - 1.0).
    pub floating_fraction: CoordF,
    /// Number of loops generated.
    pub loop_count: usize,
}

impl FloatingConcentricResult {
    // Create an empty result.
    pub fn empty() -> Self {
        Self {
            polylines: Vec::new(),
            total_length_mm: 0.0,
            floating_fraction: 0.0,
            loop_count: 0,
        }
    }

    /// Check if the result contains any infill.
    pub fn has_infill(&self) -> bool {
        !self.polylines.is_empty()
    }

    /// Convert to regular polylines (loses floating and width info).
    pub fn to_polylines(&self) -> Vec<Polyline> {
        self.polylines.iter().map(|p| p.to_polyline()).collect()
    }
}

/// Generator for floating concentric infill.
pub struct FloatingConcentricGenerator {
    config: FloatingConcentricConfig,
}

impl FloatingConcentricGenerator {
    // Create a new generator with the given config.
    pub fn new(config: FloatingConcentricConfig) -> Self {
        Self { config }
    }

    /// Create a generator with default config.
    pub fn with_defaults() -> Self {
        Self::new(FloatingConcentricConfig::default())
    }

    /// Get a reference to the config.
    pub fn config(&self) -> &FloatingConcentricConfig {
        &self.config
    }

    /// Get a mutable reference to the config.
    pub fn config_mut(&mut self) -> &mut FloatingConcentricConfig {
        &mut self.config
    }

    /// Generate floating concentric infill for the given fill area.
    ///
    /// # Arguments
    /// * `fill_area` - The area to fill with concentric loops
    /// * `floating_areas` - Areas that are not supported by the layer below.
    ///                      Points inside these areas are marked as floating.
    ///
    /// # Returns
    /// A `FloatingConcentricResult` containing the generated polylines with
    /// floating information.
    pub fn generate(
        &self,
        fill_area: &ExPolygons,
        floating_areas: &ExPolygons,
    ) -> FloatingConcentricResult {
        if fill_area.is_empty() {
            return FloatingConcentricResult::empty();
        }

        let spacing = self.config.spacing;

        // Generate concentric loops by repeatedly shrinking
        let mut all_loops: Vec<Polygon> = Vec::new();
        let mut current_area = fill_area.clone();

        while !current_area.is_empty() {
            // Add all current contours and holes as loops
            for expoly in &current_area {
                if !expoly.contour.is_empty() && expoly.contour.len() >= 3 {
                    all_loops.push(expoly.contour.clone());
                }
                for hole in &expoly.holes {
                    if !hole.is_empty() && hole.len() >= 3 {
                        all_loops.push(hole.clone());
                    }
                }
            }

            // Shrink for next iteration
            current_area = shrink(&current_area, spacing, OffsetJoinType::Miter);
        }

        if all_loops.is_empty() {
            return FloatingConcentricResult::empty();
        }

        // Convert loops to floating thick polylines and detect floating sections
        let mut polylines = Vec::with_capacity(all_loops.len());

        for loop_poly in all_loops {
            let mut ftp =
                FloatingThickPolyline::from_polygon(&loop_poly, self.config.default_width);

            // Detect floating sections
            self.detect_floating(&mut ftp, floating_areas);

            // Rebase to start at non-floating point if configured
            if self.config.prefer_non_floating_start && ftp.is_closed() {
                if let Some(idx) = ftp.first_non_floating_index() {
                    if let Some(rebased) = ftp.rebase_at(idx) {
                        ftp = rebased;
                    }
                }
            }

            // Clip the end to avoid extruder landing exactly on start
            if self.config.loop_clipping > 0.0 {
                ftp.clip_end(self.config.loop_clipping);
            }

            // Check minimum length
            if ftp.is_valid() && ftp.length() >= self.config.min_loop_length {
                polylines.push(ftp);
            }
        }

        // Optionally split at transitions
        if self.config.split_at_transitions {
            let mut split_polylines = Vec::new();
            for ftp in polylines {
                let segments = ftp.split_at_transitions();
                for seg in segments {
                    if seg.is_valid() && seg.length() >= self.config.min_loop_length {
                        split_polylines.push(seg);
                    }
                }
            }
            polylines = split_polylines;
        }

        // Calculate statistics
        let total_length_mm: CoordF = polylines.iter().map(|p| p.length()).sum();

        let floating_fraction = if total_length_mm > 0.0 {
            let floating_length: CoordF = polylines
                .iter()
                .map(|p| p.length() * p.floating_fraction())
                .sum();
            floating_length / total_length_mm
        } else {
            0.0
        };

        FloatingConcentricResult {
            loop_count: polylines.len(),
            polylines,
            total_length_mm,
            floating_fraction,
        }
    }

    /// Detect floating sections in a polyline and update the is_floating flags.
    fn detect_floating(&self, polyline: &mut FloatingThickPolyline, floating_areas: &ExPolygons) {
        if floating_areas.is_empty() || polyline.is_empty() {
            return;
        }

        // Check each point against the floating areas
        for i in 0..polyline.points.len() {
            let point = &polyline.points[i];
            polyline.is_floating[i] = self.point_in_expolygons(point, floating_areas);
        }
    }

    /// Check if a point is inside any of the given expolygons.
    fn point_in_expolygons(&self, point: &Point, expolygons: &ExPolygons) -> bool {
        for expoly in expolygons {
            if self.point_in_expolygon(point, expoly) {
                return true;
            }
        }
        false
    }

    /// Check if a point is inside an expolygon (inside contour, outside holes).
    fn point_in_expolygon(&self, point: &Point, expolygon: &ExPolygon) -> bool {
        // Must be inside contour
        if !self.point_in_polygon(point, &expolygon.contour) {
            return false;
        }

        // Must not be inside any hole
        for hole in &expolygon.holes {
            if self.point_in_polygon(point, hole) {
                return false;
            }
        }

        true
    }

    /// Check if a point is inside a polygon using ray casting.
    fn point_in_polygon(&self, point: &Point, polygon: &Polygon) -> bool {
        if polygon.len() < 3 {
            return false;
        }

        let x = point.x as f64;
        let y = point.y as f64;

        let mut inside = false;
        let n = polygon.len();

        let mut j = n - 1;
        for i in 0..n {
            let xi = polygon[i].x as f64;
            let yi = polygon[i].y as f64;
            let xj = polygon[j].x as f64;
            let yj = polygon[j].y as f64;

            if ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
                inside = !inside;
            }

            j = i;
        }

        inside
    }
}

impl Default for FloatingConcentricGenerator {
    fn default() -> Self {
        Self::with_defaults()
    }
}

/// Convenience function to generate floating concentric infill.
///
/// # Arguments
/// * `fill_area` - The area to fill with concentric loops
/// * `floating_areas` - Areas not supported by the layer below
/// * `spacing` - Line spacing in mm
///
/// # Returns
/// A `FloatingConcentricResult` containing the generated polylines.
pub fn generate_floating_concentric(
    fill_area: &ExPolygons,
    floating_areas: &ExPolygons,
    spacing: CoordF,
) -> FloatingConcentricResult {
    let config = FloatingConcentricConfig::new(spacing);
    let generator = FloatingConcentricGenerator::new(config);
    generator.generate(fill_area, floating_areas)
}

/// Convenience function to generate floating concentric infill with full config.
pub fn generate_floating_concentric_with_config(
    fill_area: &ExPolygons,
    floating_areas: &ExPolygons,
    config: FloatingConcentricConfig,
) -> FloatingConcentricResult {
    let generator = FloatingConcentricGenerator::new(config);
    generator.generate(fill_area, floating_areas)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_square_mm(size: CoordF) -> ExPolygon {
        let half = scale(size / 2.0);
        ExPolygon {
            contour: Polygon::from_points(vec![
                Point::new(-half, -half),
                Point::new(half, -half),
                Point::new(half, half),
                Point::new(-half, half),
            ]),
            holes: Vec::new(),
        }
    }

    fn make_square_with_hole_mm(outer_size: CoordF, inner_size: CoordF) -> ExPolygon {
        let half_outer = scale(outer_size / 2.0);
        let half_inner = scale(inner_size / 2.0);
        ExPolygon {
            contour: Polygon::from_points(vec![
                Point::new(-half_outer, -half_outer),
                Point::new(half_outer, -half_outer),
                Point::new(half_outer, half_outer),
                Point::new(-half_outer, half_outer),
            ]),
            holes: vec![Polygon::from_points(vec![
                Point::new(-half_inner, -half_inner),
                Point::new(-half_inner, half_inner),
                Point::new(half_inner, half_inner),
                Point::new(half_inner, -half_inner),
            ])],
        }
    }

    #[test]
    fn test_floating_concentric_config_default() {
        let config = FloatingConcentricConfig::default();
        assert!((config.spacing - 0.4).abs() < 0.01);
        assert!(config.split_at_transitions);
        assert!(config.prefer_non_floating_start);
    }

    #[test]
    fn test_floating_concentric_config_builder() {
        let config = FloatingConcentricConfig::new(0.5)
            .with_min_loop_length(2.0)
            .with_split_at_transitions(false);

        assert!((config.spacing - 0.5).abs() < 0.01);
        assert!((config.min_loop_length - 2.0).abs() < 0.01);
        assert!(!config.split_at_transitions);
    }

    #[test]
    fn test_floating_thick_line_basic() {
        let line = FloatingThickLine::new(
            Point::new(0, 0),
            Point::new(scale(10.0), 0),
            0.4,
            0.5,
            false,
            true,
        );

        assert!((line.length() - 10.0).abs() < 0.01);
        assert!(!line.is_fully_floating());
        assert!(!line.is_fully_supported());
        assert!(line.has_transition());
        assert!((line.width_at(0.5) - 0.45).abs() < 0.01);
    }

    #[test]
    fn test_floating_thick_polyline_basic() {
        let polyline = Polyline::from_points(vec![
            Point::new(0, 0),
            Point::new(scale(10.0), 0),
            Point::new(scale(10.0), scale(10.0)),
        ]);

        let ftp = FloatingThickPolyline::from_polyline(&polyline, 0.4);

        assert_eq!(ftp.len(), 3);
        assert!(!ftp.is_closed());
        assert!(ftp.is_valid());
        assert!((ftp.length() - 20.0).abs() < 0.1);
        assert_eq!(ftp.floating_fraction(), 0.0);
    }

    #[test]
    fn test_floating_thick_polyline_from_polygon() {
        let polygon = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(scale(10.0), 0),
            Point::new(scale(10.0), scale(10.0)),
            Point::new(0, scale(10.0)),
        ]);

        let ftp = FloatingThickPolyline::from_polygon(&polygon, 0.4);

        assert_eq!(ftp.len(), 5); // Polygon + closing point
        assert!(ftp.is_closed());
        assert!(ftp.is_valid());
    }

    #[test]
    fn test_floating_thick_polyline_rebase() {
        let polygon = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(scale(10.0), 0),
            Point::new(scale(10.0), scale(10.0)),
            Point::new(0, scale(10.0)),
        ]);

        let mut ftp = FloatingThickPolyline::from_polygon(&polygon, 0.4);
        // Mark first two points as floating
        ftp.is_floating[0] = true;
        ftp.is_floating[1] = true;

        let rebased = ftp.rebase_at(2).unwrap();

        // After rebasing at index 2, the first point should be the old point at index 2
        assert_eq!(rebased.points[0], ftp.points[2]);
        // And first point should not be floating
        assert!(!rebased.is_floating[0]);
    }

    #[test]
    fn test_floating_thick_polyline_split_at_transitions() {
        let mut ftp = FloatingThickPolyline::new();
        ftp.push(Point::new(0, 0), 0.4, false);
        ftp.push(Point::new(scale(5.0), 0), 0.4, false);
        ftp.push(Point::new(scale(10.0), 0), 0.4, true); // Transition here
        ftp.push(Point::new(scale(15.0), 0), 0.4, true);
        ftp.push(Point::new(scale(20.0), 0), 0.4, false); // Transition here
        ftp.push(Point::new(scale(25.0), 0), 0.4, false);

        let segments = ftp.split_at_transitions();

        // Should have 3 segments: supported -> floating -> supported
        assert_eq!(segments.len(), 3);
    }

    #[test]
    fn test_floating_thick_polyline_clip_end() {
        let mut ftp = FloatingThickPolyline::new();
        ftp.push(Point::new(0, 0), 0.4, false);
        ftp.push(Point::new(scale(10.0), 0), 0.4, false);
        ftp.push(Point::new(scale(20.0), 0), 0.4, false);

        let original_len = ftp.length();
        ftp.clip_end(2.0);

        assert!((original_len - ftp.length() - 2.0).abs() < 0.1);
    }

    #[test]
    fn test_generate_floating_concentric_no_floating() {
        let fill_area = vec![make_square_mm(20.0)];
        let floating_areas: ExPolygons = vec![];

        let result = generate_floating_concentric(&fill_area, &floating_areas, 0.5);

        assert!(result.has_infill());
        assert!(result.loop_count > 0);
        assert_eq!(result.floating_fraction, 0.0);
    }

    #[test]
    fn test_generate_floating_concentric_with_floating() {
        let fill_area = vec![make_square_mm(20.0)];
        // Create a floating area in the center
        let floating_areas = vec![make_square_mm(10.0)];

        let result = generate_floating_concentric(&fill_area, &floating_areas, 0.5);

        assert!(result.has_infill());
        assert!(result.loop_count > 0);
        // Some portion should be floating
        assert!(result.floating_fraction > 0.0);
    }

    #[test]
    fn test_generate_floating_concentric_with_hole() {
        let fill_area = vec![make_square_with_hole_mm(20.0, 10.0)];
        let floating_areas: ExPolygons = vec![];

        let result = generate_floating_concentric(&fill_area, &floating_areas, 0.5);

        assert!(result.has_infill());
        assert!(result.loop_count > 0);
    }

    #[test]
    fn test_generate_floating_concentric_empty() {
        let fill_area: ExPolygons = vec![];
        let floating_areas: ExPolygons = vec![];

        let result = generate_floating_concentric(&fill_area, &floating_areas, 0.5);

        assert!(!result.has_infill());
        assert_eq!(result.loop_count, 0);
    }

    #[test]
    fn test_generate_floating_concentric_all_floating() {
        // Fill area is entirely inside the floating area
        let fill_area = vec![make_square_mm(10.0)];
        let floating_areas = vec![make_square_mm(30.0)];

        let config = FloatingConcentricConfig::new(0.5).with_split_at_transitions(false);
        let result = generate_floating_concentric_with_config(&fill_area, &floating_areas, config);

        assert!(result.has_infill());
        // Should be nearly all floating (1.0)
        assert!(result.floating_fraction > 0.9);
    }

    #[test]
    fn test_floating_thick_polyline_to_thick_lines() {
        let mut ftp = FloatingThickPolyline::new();
        ftp.push(Point::new(0, 0), 0.4, false);
        ftp.push(Point::new(scale(10.0), 0), 0.5, true);
        ftp.push(Point::new(scale(20.0), 0), 0.4, false);

        let lines = ftp.to_thick_lines();

        assert_eq!(lines.len(), 2);
        assert!(!lines[0].is_a_floating);
        assert!(lines[0].is_b_floating);
        assert!(lines[1].is_a_floating);
        assert!(!lines[1].is_b_floating);
    }

    #[test]
    fn test_floating_concentric_generator_config() {
        let config = FloatingConcentricConfig::new(0.6);
        let generator = FloatingConcentricGenerator::new(config);

        assert!((generator.config().spacing - 0.6).abs() < 0.01);
    }

    #[test]
    fn test_floating_fraction_calculation() {
        let mut ftp = FloatingThickPolyline::new();
        // Create a polyline where half is floating
        ftp.push(Point::new(0, 0), 0.4, false);
        ftp.push(Point::new(scale(10.0), 0), 0.4, false);
        ftp.push(Point::new(scale(20.0), 0), 0.4, true);
        ftp.push(Point::new(scale(30.0), 0), 0.4, true);

        let fraction = ftp.floating_fraction();
        // Second half (10mm of 30mm total) should be floating ≈ 0.33
        assert!((fraction - 0.333).abs() < 0.1);
    }

    #[test]
    fn test_to_polylines() {
        let fill_area = vec![make_square_mm(20.0)];
        let floating_areas: ExPolygons = vec![];

        let result = generate_floating_concentric(&fill_area, &floating_areas, 1.0);
        let polylines = result.to_polylines();

        assert!(!polylines.is_empty());
        assert_eq!(polylines.len(), result.polylines.len());
    }

    #[test]
    fn test_point_in_polygon() {
        let generator = FloatingConcentricGenerator::with_defaults();

        let polygon = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(scale(10.0), 0),
            Point::new(scale(10.0), scale(10.0)),
            Point::new(0, scale(10.0)),
        ]);

        // Point inside
        assert!(generator.point_in_polygon(&Point::new(scale(5.0), scale(5.0)), &polygon));

        // Point outside
        assert!(!generator.point_in_polygon(&Point::new(scale(15.0), scale(5.0)), &polygon));

        // Point on edge (implementation dependent, usually considered outside by ray casting)
        // This is acceptable for our use case
    }
}
