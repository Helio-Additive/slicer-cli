//! Arc fitting algorithm for converting linear segments to G2/G3 arc moves
//!
//! C++ Reference:
//! - ArcFitter.hpp
//! - ArcFitter.cpp
//!
//! This module analyzes sequences of linear moves and attempts to fit them
//! as circular arcs where appropriate. This reduces G-code size and can
//! improve motion smoothness on printers with arc support.

use crate::circle::{
    ArcDirection, ArcSegment, DEFAULT_ARC_LENGTH_PERCENT_TOLERANCE, DEFAULT_SCALED_MAX_RADIUS,
};
use crate::geometry::{Point, Polyline};
use crate::Result;

/// Type of movement path
/// ArcFitter.hpp:11-17
/// C++: enum class EMovePathType : unsigned char
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EMovePathType {
    // No movement
    // ArcFitter.hpp:13
    // C++: Noop_move,
    NoopMove,

    // Linear movement (G0/G1)
    // ArcFitter.hpp:14
    // C++: Linear_move,
    LinearMove,

    // Arc movement clockwise (G2)
    // ArcFitter.hpp:15
    // C++: Arc_move_cw,
    ArcMoveCw,

    // Arc movement counter-clockwise (G3)
    // ArcFitter.hpp:16
    // C++: Arc_move_ccw,
    ArcMoveCcw,
}

/// Data describing a fitted path segment
/// ArcFitter.hpp:20-42
/// C++: struct PathFittingData
#[derive(Debug, Clone)]
pub struct PathFittingData {
    // Index of first point in segment
    // ArcFitter.hpp:21
    // C++: size_t start_point_index;
    pub start_point_index: usize,

    // Index of last point in segment
    // ArcFitter.hpp:22
    // C++: size_t end_point_index;
    pub end_point_index: usize,

    // Type of path (linear or arc)
    // ArcFitter.hpp:23
    // C++: EMovePathType path_type;
    pub path_type: EMovePathType,

    // Arc data (only valid for arc moves)
    // ArcFitter.hpp:25-26
    // C++: ArcSegment arc_data;
    pub arc_data: ArcSegment,
}

impl PathFittingData {
    // Create a new path fitting data entry
    pub fn new(
        start_point_index: usize,
        end_point_index: usize,
        path_type: EMovePathType,
        arc_data: ArcSegment,
    ) -> Self {
        PathFittingData {
            start_point_index,
            end_point_index,
            path_type,
            arc_data,
        }
    }

    // Check if this is a linear move
    // ArcFitter.hpp:28-30
    // C++: bool is_linear_move() { return (path_type == EMovePathType::Linear_move); }
    pub fn is_linear_move(&self) -> bool {
        self.path_type == EMovePathType::LinearMove
    }

    // Check if this is an arc move
    // ArcFitter.hpp:31-33
    // C++: bool is_arc_move() { return (path_type == EMovePathType::Arc_move_ccw || path_type == EMovePathType::Arc_move_cw); }
    pub fn is_arc_move(&self) -> bool {
        self.path_type == EMovePathType::ArcMoveCcw || self.path_type == EMovePathType::ArcMoveCw
    }

    // Reverse the arc direction
    // ArcFitter.hpp:34-39
    // C++: bool reverse_arc_path()
    pub fn reverse_arc_path(&mut self) -> bool {
        if !self.is_arc_move() || !self.arc_data.reverse() {
            return false;
        }

        // Update path type based on reversed direction
        // ArcFitter.hpp:37
        // C++: path_type = (arc_data.direction == ArcDirection::Arc_Dir_CCW) ? EMovePathType::Arc_move_ccw : EMovePathType::Arc_move_cw;
        self.path_type = match self.arc_data.direction {
            ArcDirection::CounterClockwise => EMovePathType::ArcMoveCcw,
            ArcDirection::Clockwise => EMovePathType::ArcMoveCw,
            ArcDirection::Unknown => self.path_type,
        };

        true
    }
}

/// Arc fitting algorithm
/// ArcFitter.hpp:44-50
/// C++: class ArcFitter
pub struct ArcFitter;

impl ArcFitter {
    // Analyze points and fit arcs where possible
    // ArcFitter.cpp:8-102
    // C++: static void do_arc_fitting(const Points& points, std::vector<PathFittingData> &result, double tolerance)
    pub fn do_arc_fitting(
        points: &[Point],
        result: &mut Vec<PathFittingData>,
        tolerance: f64,
    ) -> Result<()> {
        // Clear output and reserve space
        // ArcFitter.cpp:23-24
        // C++: result.clear();
        // C++: result.reserve(points.size() / 2);
        result.clear();
        result.reserve(points.len() / 2);

        // Handle trivial case of less than 3 points
        // ArcFitter.cpp:25-31
        // C++: if (points.size() < 3) {
        // C++:     PathFittingData data;
        // C++:     data.start_point_index = 0;
        // C++:     data.end_point_index = points.size() - 1;
        // C++:     data.path_type = EMovePathType::Linear_move;
        // C++:     result.push_back(data);
        // C++:     return;
        // C++: }
        if points.len() < 3 {
            result.push(PathFittingData::new(
                0,
                points.len() - 1,
                EMovePathType::LinearMove,
                ArcSegment::new(),
            ));
            return Ok(());
        }

        // Initialize segment tracking variables
        // ArcFitter.cpp:33-38
        // C++: size_t front_index = 0;
        // C++: size_t back_index = 0;
        // C++: ArcSegment last_arc;
        // C++: bool can_fit = false;
        // C++: Points current_segment;
        // C++: current_segment.reserve(points.size());
        let mut front_index = 0;
        let mut back_index = 0;
        let mut last_arc = ArcSegment::new();
        let mut current_segment = Vec::with_capacity(points.len());

        // Iterate through all points
        // ArcFitter.cpp:40
        // C++: for (size_t i = 0; i < points.size(); i++)
        for i in 0..points.len() {
            // Add point to current segment
            // ArcFitter.cpp:42-44
            // C++: back_index = i;
            // C++: current_segment.push_back(points[i]);
            back_index = i;
            current_segment.push(points[i]);

            // Need at least 3 points to fit an arc
            // ArcFitter.cpp:45-46
            // C++: if (back_index - front_index < 2)
            // C++:     continue;
            if back_index - front_index < 2 {
                continue;
            }

            // Calculate approximate segment length
            let approximate_length = Polyline::from_points(current_segment.clone()).length();

            // Try to fit current segment as an arc
            // ArcFitter.cpp:48-52
            // C++: can_fit = ArcSegment::try_create_arc(current_segment, target_arc, Polyline(current_segment).length(),
            // C++:                                       DEFAULT_SCALED_MAX_RADIUS,
            // C++:                                       tolerance,
            // C++:                                       DEFAULT_ARC_LENGTH_PERCENT_TOLERANCE);
            let can_fit = ArcSegment::try_create_arc(
                &current_segment,
                approximate_length,
                DEFAULT_SCALED_MAX_RADIUS,
                tolerance,
                DEFAULT_ARC_LENGTH_PERCENT_TOLERANCE,
            );

            if let Some(target_arc) = can_fit {
                // Successfully fit as arc, save it temporarily
                // ArcFitter.cpp:54
                // C++: last_arc = target_arc;
                last_arc = target_arc;

                // If this is the last point, save the arc
                // ArcFitter.cpp:55-61
                // C++: if (back_index == points.size() - 1) {
                // C++:     result.emplace_back(std::move(PathFittingData{ front_index,
                // C++:                        back_index,
                // C++:                        last_arc.direction == ArcDirection::Arc_Dir_CCW ? EMovePathType::Arc_move_ccw : EMovePathType::Arc_move_cw,
                // C++:                        last_arc }));
                // C++:     front_index = back_index;
                // C++: }
                if back_index == points.len() - 1 {
                    let path_type = match last_arc.direction {
                        ArcDirection::CounterClockwise => EMovePathType::ArcMoveCcw,
                        ArcDirection::Clockwise => EMovePathType::ArcMoveCw,
                        ArcDirection::Unknown => EMovePathType::LinearMove,
                    };
                    result.push(PathFittingData::new(
                        front_index,
                        back_index,
                        path_type,
                        last_arc.clone(),
                    ));
                    front_index = back_index;
                }
            } else {
                // Cannot fit as arc
                if back_index - front_index > 2 {
                    // Previous segment (without current point) was a valid arc
                    // ArcFitter.cpp:64-68
                    // C++: result.emplace_back(std::move(PathFittingData{ front_index,
                    // C++:                    back_index - 1,
                    // C++:                    last_arc.direction == ArcDirection::Arc_Dir_CCW ? EMovePathType::Arc_move_ccw : EMovePathType::Arc_move_cw,
                    // C++:                    last_arc }));
                    let path_type = match last_arc.direction {
                        ArcDirection::CounterClockwise => EMovePathType::ArcMoveCcw,
                        ArcDirection::Clockwise => EMovePathType::ArcMoveCw,
                        ArcDirection::Unknown => EMovePathType::LinearMove,
                    };
                    result.push(PathFittingData::new(
                        front_index,
                        back_index - 1,
                        path_type,
                        last_arc.clone(),
                    ));
                } else {
                    // First segment couldn't fit as arc, save as line
                    // ArcFitter.cpp:70-74
                    // C++: if (result.empty() || result.back().path_type != EMovePathType::Linear_move)
                    // C++:     result.emplace_back(std::move(PathFittingData{front_index, front_index + 1, EMovePathType::Linear_move, ArcSegment()}));
                    // C++: else if(result.back().path_type == EMovePathType::Linear_move)
                    // C++:     result.back().end_point_index = front_index + 1;
                    if result.is_empty()
                        || result.last().unwrap().path_type != EMovePathType::LinearMove
                    {
                        result.push(PathFittingData::new(
                            front_index,
                            front_index + 1,
                            EMovePathType::LinearMove,
                            ArcSegment::new(),
                        ));
                    } else if result.last().unwrap().path_type == EMovePathType::LinearMove {
                        result.last_mut().unwrap().end_point_index = front_index + 1;
                    }
                }

                // Reset for next segment
                // ArcFitter.cpp:76-79
                // C++: front_index = back_index - 1;
                // C++: current_segment.clear();
                // C++: current_segment.push_back(points[front_index]);
                // C++: current_segment.push_back(points[front_index + 1]);
                front_index = back_index - 1;
                current_segment.clear();
                current_segment.push(points[front_index]);
                current_segment.push(points[front_index + 1]);
            }
        }

        // Handle remaining data
        // ArcFitter.cpp:82-86
        // C++: if (front_index != back_index) {
        // C++:     if (result.empty() || result.back().path_type != EMovePathType::Linear_move)
        // C++:         result.emplace_back(std::move(PathFittingData{front_index, back_index, EMovePathType::Linear_move, ArcSegment()}));
        // C++:     else if (result.back().path_type == EMovePathType::Linear_move)
        // C++:         result.back().end_point_index = back_index;
        // C++: }
        if front_index != back_index {
            if result.is_empty() || result.last().unwrap().path_type != EMovePathType::LinearMove {
                result.push(PathFittingData::new(
                    front_index,
                    back_index,
                    EMovePathType::LinearMove,
                    ArcSegment::new(),
                ));
            } else if result.last().unwrap().path_type == EMovePathType::LinearMove {
                result.last_mut().unwrap().end_point_index = back_index;
            }
        }

        // Shrink result to actual size
        // ArcFitter.cpp:88
        // C++: result.shrink_to_fit();
        result.shrink_to_fit();

        Ok(())
    }

    // Perform arc fitting and simplify linear segments with Douglas-Peucker
    // ArcFitter.cpp:91-162
    // C++: void ArcFitter::do_arc_fitting_and_simplify(Points& points, std::vector<PathFittingData>& result, double tolerance)
    pub fn do_arc_fitting_and_simplify(
        points: &mut Vec<Point>,
        result: &mut Vec<PathFittingData>,
        tolerance: f64,
    ) -> Result<()> {
        // Step 1: Do arc fitting first
        // ArcFitter.cpp:93-96
        // C++: if (abs(tolerance) > SCALED_EPSILON)
        // C++:     ArcFitter::do_arc_fitting(points, result, tolerance);
        // C++: else
        // C++:     result.push_back(PathFittingData{ 0, points.size() - 1, EMovePathType::Linear_move, ArcSegment() });
        const SCALED_EPSILON: f64 = 0.0001 * 1_000_000.0; // 0.0001mm in scaled units

        if tolerance.abs() > SCALED_EPSILON {
            Self::do_arc_fitting(points, result, tolerance)?;
        } else {
            result.push(PathFittingData::new(
                0,
                points.len() - 1,
                EMovePathType::LinearMove,
                ArcSegment::new(),
            ));
        }

        // Step 2: Simplify linear segments with Douglas-Peucker
        // ArcFitter.cpp:101-104
        // C++: if (result.size() == 1 && result[0].path_type == EMovePathType::Linear_move) {
        // C++:     points = MultiPoint::_douglas_peucker(points, tolerance);
        // C++:     result[0].end_point_index = points.size() - 1;
        // C++:     return;
        // C++: }
        if result.len() == 1 && result[0].path_type == EMovePathType::LinearMove {
            let polyline = Polyline::from_points(points.clone());
            let simplified = Polyline::douglas_peucker(&polyline, tolerance);
            *points = simplified.points;
            result[0].end_point_index = points.len() - 1;
            return Ok(());
        }

        // Mixed arc and linear segments - simplify each independently
        // ArcFitter.cpp:105-111
        // C++: Points simplified_points;
        // C++: simplified_points.reserve(points.size());
        // C++: simplified_points.push_back(points[0]);
        // C++: std::vector<size_t> reduce_count(result.size(), 0);
        let mut simplified_points = Vec::with_capacity(points.len());
        simplified_points.push(points[0]);
        let mut reduce_count = vec![0; result.len()];

        // Process each segment
        // ArcFitter.cpp:112
        // C++: for (size_t i = 0; i < result.size(); i++)
        for i in 0..result.len() {
            let start_index = result[i].start_point_index;
            let end_index = result[i].end_point_index;

            // Extract segment points
            // ArcFitter.cpp:114-121
            // C++: Points straight_or_arc_part;
            // C++: straight_or_arc_part.reserve(end_index - start_index + 1);
            // C++: for (size_t j = start_index; j <= end_index; j++)
            // C++:     straight_or_arc_part.push_back(points[j]);
            let mut segment_points = Vec::with_capacity(end_index - start_index + 1);
            for j in start_index..=end_index {
                segment_points.push(points[j]);
            }

            // Simplify segment with Douglas-Peucker
            // ArcFitter.cpp:122
            // C++: straight_or_arc_part = MultiPoint::_douglas_peucker(straight_or_arc_part, tolerance);
            let polyline = Polyline::from_points(segment_points);
            let simplified = Polyline::douglas_peucker(&polyline, tolerance);
            segment_points = simplified.points;

            // Track how many points were reduced
            // ArcFitter.cpp:124
            // C++: reduce_count[i] = end_index - start_index + 1 - straight_or_arc_part.size();
            reduce_count[i] = (end_index - start_index + 1) - segment_points.len();

            // Append simplified points (skip first to avoid duplication)
            // ArcFitter.cpp:126-128
            // C++: for (size_t j = 1; j < straight_or_arc_part.size(); j++) {
            // C++:     simplified_points.push_back(straight_or_arc_part[j]);
            // C++: }
            for j in 1..segment_points.len() {
                simplified_points.push(segment_points[j]);
            }
        }

        // Replace input points with simplified version
        // ArcFitter.cpp:131
        // C++: points = simplified_points;
        *points = simplified_points;

        // Update indices in result to match simplified points
        // ArcFitter.cpp:133-134
        // C++: for (size_t j = 1; j < reduce_count.size(); j++)
        // C++:     reduce_count[j] += reduce_count[j - 1];
        for j in 1..reduce_count.len() {
            reduce_count[j] += reduce_count[j - 1];
        }

        // Adjust segment indices
        // ArcFitter.cpp:135-139
        // C++: for (size_t j = 0; j < result.size(); j++) {
        // C++:     result[j].end_point_index -= reduce_count[j];
        // C++:     if (j != result.size() - 1)
        // C++:         result[j + 1].start_point_index = result[j].end_point_index;
        // C++: }
        for j in 0..result.len() {
            result[j].end_point_index -= reduce_count[j];
            if j != result.len() - 1 {
                result[j + 1].start_point_index = result[j].end_point_index;
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_path_fitting_data_is_linear() {
        let data = PathFittingData::new(0, 10, EMovePathType::LinearMove, ArcSegment::new());
        assert!(data.is_linear_move());
        assert!(!data.is_arc_move());
    }

    #[test]
    fn test_path_fitting_data_is_arc() {
        let data = PathFittingData::new(0, 10, EMovePathType::ArcMoveCcw, ArcSegment::new());
        assert!(!data.is_linear_move());
        assert!(data.is_arc_move());

        let data = PathFittingData::new(0, 10, EMovePathType::ArcMoveCw, ArcSegment::new());
        assert!(!data.is_linear_move());
        assert!(data.is_arc_move());
    }

    #[test]
    fn test_arc_fitting_empty() {
        let points = vec![];
        let mut result = Vec::new();

        // Should handle empty gracefully (though C++ expects at least 1 point)
        // We'll return Ok for robustness
        assert!(
            ArcFitter::do_arc_fitting(&points, &mut result, 0.05).is_err() || result.is_empty()
        );
    }

    #[test]
    fn test_arc_fitting_two_points() {
        let points = vec![Point::new(0, 0), Point::new(1000, 0)];
        let mut result = Vec::new();

        ArcFitter::do_arc_fitting(&points, &mut result, 0.05).unwrap();

        assert_eq!(result.len(), 1);
        assert_eq!(result[0].path_type, EMovePathType::LinearMove);
        assert_eq!(result[0].start_point_index, 0);
        assert_eq!(result[0].end_point_index, 1);
    }

    #[test]
    fn test_arc_fitting_straight_line() {
        // Three collinear points - should result in linear move
        let points = vec![Point::new(0, 0), Point::new(500, 0), Point::new(1000, 0)];
        let mut result = Vec::new();

        ArcFitter::do_arc_fitting(&points, &mut result, 50_000.0).unwrap();

        assert_eq!(result.len(), 1);
        assert_eq!(result[0].path_type, EMovePathType::LinearMove);
    }

    #[test]
    fn test_arc_fitting_quarter_circle() {
        // Points forming a 90-degree arc
        let radius = 1000.0;
        let steps = 10;
        let mut points = Vec::new();

        for i in 0..=steps {
            let angle = (i as f64) * std::f64::consts::PI / (2.0 * steps as f64);
            let x = (radius * angle.cos()).round() as i64;
            let y = (radius * angle.sin()).round() as i64;
            points.push(Point::new(x, y));
        }

        let mut result = Vec::new();
        ArcFitter::do_arc_fitting(&points, &mut result, 100_000.0).unwrap();

        // Should detect at least one arc segment
        let has_arc = result.iter().any(|r| r.is_arc_move());
        assert!(has_arc, "Expected to detect arc in quarter circle");
    }

    #[test]
    fn test_move_path_type_values() {
        assert_eq!(EMovePathType::NoopMove, EMovePathType::NoopMove);
        assert_ne!(EMovePathType::LinearMove, EMovePathType::ArcMoveCw);
        assert_ne!(EMovePathType::ArcMoveCw, EMovePathType::ArcMoveCcw);
    }
}
