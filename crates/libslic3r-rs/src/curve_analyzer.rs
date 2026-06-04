//! Curve analysis for extrusion paths
//!
//! C++ Reference: CurveAnalyzer.hpp, CurveAnalyzer.cpp
//!
//! This module analyzes the curvature of extrusion paths and splits them into segments
//! with different curve degrees. This is used to adjust printing speed based on the
//! sharpness of curves.

use crate::extrusion_entity::{ExtrusionPath, ExtrusionRole};
use crate::geometry::{Point, Polygon, Polyline};
use crate::{scale, CoordF, Error, Result, SCALING_FACTOR};
use std::f64::consts::PI;

/// Constants for curvature analysis
/// CurveAnalyzer.cpp:6-12
const CURVATURES_SAMPLING_NUMBER: usize = 6;
const CURVATURES_DENSIFY_WIDTH: f64 = 1.0; // mm
const CURVATURES_SAMPLING_WIDTH: f64 = 6.0; // mm
const CURVATURES_ANGLE_BEST: f64 = PI / 6.0;
const CURVATURES_ANGLE_WORST: f64 = 5.0 * PI / 6.0;

/// Pre-calculated curvature thresholds
/// CurveAnalyzer.cpp:11-12
const CURVATURES_BEST: f64 = CURVATURES_ANGLE_BEST * 1000.0 / CURVATURES_SAMPLING_WIDTH;
const CURVATURES_WORST: f64 = CURVATURES_ANGLE_WORST * 1000.0 / CURVATURES_SAMPLING_WIDTH;

/// Curve analysis mode
/// CurveAnalyzer.hpp:8-12
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ECurveAnalyseMode {
    // Relative mode - uses absolute value of cross product
    // CurveAnalyzer.hpp:10
    RelativeMode,

    // Absolute mode - uses signed cross product
    // CurveAnalyzer.hpp:11
    AbsoluteMode,
}

/// Analyzer for calculating path curvatures
/// CurveAnalyzer.hpp:15-25
#[derive(Debug, Default)]
pub struct CurveAnalyzer;

impl CurveAnalyzer {
    // Create a new curve analyzer
    // CurveAnalyzer.hpp:15
    pub fn new() -> Self {
        Self
    }

    // Calculate curvatures for extrusion paths and split them by curve degree
    ///
    // This function analyzes a closed polygon's curvature and splits the input paths
    // into segments with different curve degrees (0 = flattest, 5 = sharpest).
    // The input paths are modified in place.
    ///
    // # Arguments
    // * `paths` - Extrusion paths to analyze (must form a closed polygon)
    // * `mode` - Analysis mode (relative or absolute)
    ///
    // CurveAnalyzer.cpp:21-203
    pub fn calculate_curvatures(
        &self,
        paths: &mut Vec<ExtrusionPath>,
        mode: ECurveAnalyseMode,
    ) -> Result<()> {
        if paths.is_empty() {
            return Ok(());
        }

        // Step 1: Build polygon and track path lengths
        // CurveAnalyzer.cpp:23-35
        let mut polygon = Polygon::new();
        let mut paths_length = Vec::with_capacity(paths.len());

        for (i, path) in paths.iter().enumerate() {
            let path_len = path.polyline.length();
            if i == 0 {
                paths_length.push(path_len);
            } else {
                paths_length.push(paths_length[i - 1] + path_len);
            }

            // Add all points except the last (to avoid duplication in closed loop)
            // CurveAnalyzer.cpp:32-34
            let points = &path.polyline.points;
            if !points.is_empty() {
                polygon
                    .points
                    .extend_from_slice(&points[..points.len() - 1]);
            }
        }

        if polygon.points.is_empty() {
            return Ok(());
        }

        // Step 2: Densify polygon to have points every ~1mm
        // CurveAnalyzer.cpp:36-37
        // C++ uses scale_(curvatures_densify_width) but densify expects float/double
        polygon.densify(CURVATURES_DENSIFY_WIDTH);
        let polygon_length = polygon.parameter_by_length();

        let point_num = polygon.points.len();
        if point_num == 0 {
            return Ok(());
        }

        // Step 3: Calculate angle at each segment
        // CurveAnalyzer.cpp:39-55
        let mut angles = vec![0.0_f32; point_num];
        for i in 0..point_num {
            let curr = i;
            let prev = if curr == 0 { point_num - 1 } else { curr - 1 };
            let next = if curr == point_num - 1 { 0 } else { curr + 1 };

            // Calculate vectors between consecutive points
            // CurveAnalyzer.cpp:45-46
            let v1 = polygon.points[curr] - polygon.points[prev];
            let v2 = polygon.points[next] - polygon.points[curr];

            // Calculate dot and cross products
            // CurveAnalyzer.cpp:47-48
            let dot = v1.x as i64 * v2.x as i64 + v1.y as i64 * v2.y as i64;
            let mut cross = v1.x as i64 * v2.y as i64 - v1.y as i64 * v2.x as i64;

            // In relative mode, use absolute value of cross product
            // CurveAnalyzer.cpp:49-50
            if mode == ECurveAnalyseMode::RelativeMode {
                cross = cross.abs();
            }

            // Calculate angle using atan2
            // CurveAnalyzer.cpp:51
            angles[curr] = (cross as f64).atan2(dot as f64) as f32;
        }

        // Step 4: Calculate average curvatures using sliding window
        // CurveAnalyzer.cpp:57-94
        let mut average_curvatures = vec![0.0_f64; point_num];
        let total_length = *paths_length.last().unwrap_or(&0.0);
        // C++ compares paths_length.back() (double) with scale_(width) (int64)
        // In Rust we need explicit conversion - compare in unscaled space
        let sampling_width_scaled = CURVATURES_SAMPLING_WIDTH;

        if total_length < sampling_width_scaled {
            // Loop too short - use maximum curvature
            // CurveAnalyzer.cpp:61-65
            let temp = 1000.0 * 2.0 * PI / (total_length * SCALING_FACTOR);
            for curvature in &mut average_curvatures {
                *curvature = temp;
            }
        } else {
            // Calculate curvature for each point using sliding window
            // CurveAnalyzer.cpp:67-92
            let half_sampling = sampling_width_scaled / 2.0;

            for i in 0..point_num {
                let mut sum_angle = 0.0_f32;

                // Right segment
                // CurveAnalyzer.cpp:69-75
                let mut j = i;
                let mut right_length = 0.0_f32;
                while right_length < half_sampling as f32 {
                    let next_j = if j + 1 >= point_num { 0 } else { j + 1 };
                    sum_angle += angles[j];
                    let diff = polygon.points[next_j] - polygon.points[j];
                    right_length += ((diff.x as f32).powi(2) + (diff.y as f32).powi(2)).sqrt();
                    j = next_j;
                }

                // Left segment
                // CurveAnalyzer.cpp:76-82
                let mut k = i;
                let mut left_length = 0.0_f32;
                while left_length < half_sampling as f32 {
                    let next_k = if k < 1 { point_num - 1 } else { k - 1 };
                    sum_angle += angles[k];
                    let diff = polygon.points[k] - polygon.points[next_k];
                    left_length += ((diff.x as f32).powi(2) + (diff.y as f32).powi(2)).sqrt();
                    k = next_k;
                }

                // Subtract center angle and calculate curvature
                // CurveAnalyzer.cpp:83-84
                sum_angle -= angles[i];
                average_curvatures[i] =
                    1000.0 * (sum_angle.abs() as f64) / CURVATURES_SAMPLING_WIDTH;
            }
        }

        // Step 5: Normalize curvatures to discrete levels
        // CurveAnalyzer.cpp:96-114
        let mut curvatures_norm = vec![0_i32; point_num];

        // Calculate sampling steps
        // CurveAnalyzer.cpp:99-103
        let mut sampling_step = vec![0_i32; CURVATURES_SAMPLING_NUMBER - 1];
        for i in 0..CURVATURES_SAMPLING_NUMBER - 1 {
            sampling_step[i] = ((2 * i + 1) * 50 / (CURVATURES_SAMPLING_NUMBER - 1)) as i32;
        }
        sampling_step[0] = 0;
        sampling_step[CURVATURES_SAMPLING_NUMBER - 2] = 100;

        // Normalize each curvature to 0..(CURVATURES_SAMPLING_NUMBER-1)
        // CurveAnalyzer.cpp:104-113
        for i in 0..point_num {
            let normalized = (100.0 * (average_curvatures[i] - CURVATURES_BEST)
                / (CURVATURES_WORST - CURVATURES_BEST)) as i32;

            if normalized >= 100 {
                curvatures_norm[i] = (CURVATURES_SAMPLING_NUMBER - 1) as i32;
            } else {
                for j in 0..CURVATURES_SAMPLING_NUMBER - 1 {
                    if normalized < sampling_step[j] {
                        curvatures_norm[i] = j as i32;
                        break;
                    }
                }
            }
        }

        // Step 6: Build list of curvature changes
        // CurveAnalyzer.cpp:115-124
        let mut curvature_list: Vec<(Point, usize, i32)> = Vec::new();
        let mut last_curvature_norm = -1_i32;

        for i in 0..point_num {
            if curvatures_norm[i] != last_curvature_norm {
                last_curvature_norm = curvatures_norm[i];
                curvature_list.push((polygon.points[i], i, last_curvature_norm));
            }
        }

        // Add final point (wrapping to start)
        // CurveAnalyzer.cpp:123
        curvature_list.push((polygon.points[0], point_num, curvatures_norm[0]));

        // Step 7: Split and modify paths according to curve degree
        // CurveAnalyzer.cpp:126-198
        if curvature_list.len() == 2 {
            // All paths have same curve degree
            // CurveAnalyzer.cpp:127-130
            let degree = curvature_list[0].2 as u8;
            for path in paths.iter_mut() {
                path.set_curve_degree(degree as i32);
            }
        } else {
            // Split paths at curvature boundaries
            // CurveAnalyzer.cpp:132-194
            let mut out = Vec::new();
            out.reserve(paths.len() + curvature_list.len() - 1);

            let mut j = 1_usize;
            let mut current_curva_norm = curvature_list[0].2;

            for (i, path) in paths.iter().enumerate() {
                if j >= curvature_list.len() {
                    break;
                }

                // Check if path end matches curvature boundary
                // CurveAnalyzer.cpp:137-142
                if path.last_point() == curvature_list[j].0 {
                    let mut path_copy = path.clone();
                    path_copy.set_curve_degree((current_curva_norm as u8) as i32);
                    out.push(path_copy);
                    current_curva_norm = curvature_list[j].2;
                    j += 1;
                    continue;
                }

                // Check if path start matches curvature boundary
                // CurveAnalyzer.cpp:143-153
                if path.first_point() == curvature_list[j].0 {
                    if path.polyline.points.first() == path.polyline.points.last() {
                        let mut path_copy = path.clone();
                        path_copy.set_curve_degree((current_curva_norm as u8) as i32);
                        out.push(path_copy);
                        current_curva_norm = curvature_list[j].2;
                        j += 1;
                        continue;
                    }
                }

                // Check if path doesn't need splitting
                // CurveAnalyzer.cpp:155-161
                if paths_length[i] <= polygon_length[curvature_list[j].1]
                    || path.last_point() == curvature_list[j].0
                {
                    let mut path_copy = path.clone();
                    path_copy.set_curve_degree((current_curva_norm as u8) as i32);
                    out.push(path_copy);

                    if path.last_point() == curvature_list[j].0 {
                        current_curva_norm = curvature_list[j].2;
                        j += 1;
                    }
                } else {
                    // Split path at curvature boundaries
                    // CurveAnalyzer.cpp:163-189
                    let mut current_path = path.clone();

                    while j < curvature_list.len() {
                        let split_point = curvature_list[j].0;

                        // Find the index of the split point in the polyline
                        // C++: Polyline::split_at expects a Point, Rust expects index
                        let split_index = current_path
                            .polyline
                            .points
                            .iter()
                            .position(|p| *p == split_point)
                            .unwrap_or(0);

                        let (left, right) = current_path.polyline.split_at(split_index);
                        {
                            let mut left_path = ExtrusionPath {
                                polyline: left,
                                role: current_path.role,
                                mm3_per_mm: current_path.mm3_per_mm,
                                width: current_path.width,
                                height: current_path.height,
                                overhang_degree: current_path.overhang_degree,
                                curve_degree: current_curva_norm as i32,
                                customize_flag: current_path.customize_flag,
                            };
                            out.push(left_path);

                            current_path = ExtrusionPath {
                                polyline: right,
                                role: current_path.role,
                                mm3_per_mm: current_path.mm3_per_mm,
                                width: current_path.width,
                                height: current_path.height,
                                overhang_degree: current_path.overhang_degree,
                                curve_degree: current_path.curve_degree,
                                customize_flag: current_path.customize_flag,
                            };
                        }

                        current_curva_norm = curvature_list[j].2;
                        j += 1;

                        if j < curvature_list.len()
                            && (paths_length[i] <= polygon_length[curvature_list[j].1]
                                || path.last_point() == curvature_list[j].0)
                        {
                            current_path.set_curve_degree((current_curva_norm as u8) as i32);

                            // Check if we need to advance j before moving current_path
                            let should_advance_j = current_path.last_point() == curvature_list[j].0;
                            out.push(current_path);

                            if should_advance_j {
                                current_curva_norm = curvature_list[j].2;
                                j += 1;
                            }
                            break;
                        }
                    }
                }
            }

            // Replace input paths with split paths
            // CurveAnalyzer.cpp:196-198
            *paths = out;
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_curve_analyzer_creation() {
        let analyzer = CurveAnalyzer::new();
        assert_eq!(std::mem::size_of_val(&analyzer), 0);
    }

    #[test]
    fn test_empty_paths() {
        let analyzer = CurveAnalyzer::new();
        let mut paths = Vec::new();

        let result = analyzer.calculate_curvatures(&mut paths, ECurveAnalyseMode::RelativeMode);
        assert!(result.is_ok());
        assert!(paths.is_empty());
    }

    #[test]
    fn test_straight_path() {
        let analyzer = CurveAnalyzer::new();

        // Create a straight horizontal line
        let mut polyline = Polyline::new();
        for i in 0..10 {
            polyline.points.push(Point::new(i * 1000000, 0)); // 1mm increments
        }

        let path = ExtrusionPath {
            polyline,
            role: ExtrusionRole::Perimeter,
            mm3_per_mm: 1.0,
            width: 0.4,
            height: 0.2,
            overhang_degree: 0,
            curve_degree: 0,
            customize_flag: crate::extrusion_entity::CustomizeFlag::None,
        };

        let mut paths = vec![path];

        // Straight line should get lowest curve degree
        let result = analyzer.calculate_curvatures(&mut paths, ECurveAnalyseMode::RelativeMode);
        assert!(result.is_ok());
    }

    #[test]
    fn test_curve_mode_enum() {
        assert_ne!(
            ECurveAnalyseMode::RelativeMode,
            ECurveAnalyseMode::AbsoluteMode
        );
    }
}
