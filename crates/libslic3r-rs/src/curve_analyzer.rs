//! Curve analysis for extrusion paths
//!
//! C++ Reference: CurveAnalyzer.hpp, CurveAnalyzer.cpp
//!
//! This module analyzes the curvature of extrusion paths and splits them into segments
//! with different curve degrees. This is used to adjust printing speed based on the
//! sharpness of curves.

use crate::extrusion_entity::ExtrusionPath;
use crate::geometry::{Point, Polygon};
use crate::{scale, CoordF, Result};
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

        // CurveAnalyzer.cpp:23-33
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

        // 1 generate point series which is on the line of polygon, point distance along the polygon is smaller than 1mm
        // CurveAnalyzer.cpp:34-36
        // C++: polygon.densify(scale_(curvatures_densify_width));
        // C++: std::vector<float> polygon_length = polygon.parameter_by_length();
        polygon.densify(scale(CURVATURES_DENSIFY_WIDTH) as f32, None);
        let polygon_length = polygon.parameter_by_length();

        let point_num = polygon.points.len();
        if point_num == 0 {
            return Ok(());
        }

        // 2 calculate angle of every segment
        // CurveAnalyzer.cpp:38-52
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

        // 3 generate sum of angle and length of the adjacent segment for eveny point, range is approximately curvatures_sampling_width.
        //   And then calculate the curvature
        // CurveAnalyzer.cpp:54-87
        let mut sum_angles = vec![0.0_f32; point_num];
        let mut average_curvatures = vec![0.0_f64; point_num];
        // C++: if (paths_length.back() < scale_(curvatures_sampling_width))
        // paths_length holds scaled lengths (polyline.length() over scaled points),
        // so compare against scale(curvatures_sampling_width).
        if *paths_length.last().unwrap() < scale(CURVATURES_SAMPLING_WIDTH) as f64 {
            // loop is too short, so the curvatures is max
            // CurveAnalyzer.cpp:58-63
            // C++: double temp = 1000.0 * 2.0 * PI / ((double)(paths_length.back()) * SCALING_FACTOR);
            // C++ SCALING_FACTOR == 0.00001, so (scaled * SCALING_FACTOR) == unscale(scaled).
            let temp = 1000.0 * 2.0 * PI / (*paths_length.last().unwrap() / crate::SCALING_FACTOR);
            for i in 0..point_num {
                average_curvatures[i] = temp;
            }
        } else {
            // CurveAnalyzer.cpp:65-87
            for i in 0..point_num {
                // right segment
                // CurveAnalyzer.cpp:67-74
                let mut j = i;
                let mut right_length = 0.0_f32;
                while right_length < scale(CURVATURES_SAMPLING_WIDTH / 2.0) as f32 {
                    let next_j = if j + 1 >= point_num { 0 } else { j + 1 };
                    sum_angles[i] += angles[j];
                    let diff = polygon.points[next_j] - polygon.points[j];
                    right_length += ((diff.x as f32).powi(2) + (diff.y as f32).powi(2)).sqrt();
                    j = next_j;
                }
                // left segment
                // CurveAnalyzer.cpp:76-83
                let mut k = i;
                let mut left_length = 0.0_f32;
                while left_length < scale(CURVATURES_SAMPLING_WIDTH / 2.0) as f32 {
                    let next_k = if k < 1 { point_num - 1 } else { k - 1 };
                    sum_angles[i] += angles[k];
                    let diff = polygon.points[k] - polygon.points[next_k];
                    left_length += ((diff.x as f32).powi(2) + (diff.y as f32).powi(2)).sqrt();
                    k = next_k;
                }
                // CurveAnalyzer.cpp:84-85
                sum_angles[i] -= angles[i];
                average_curvatures[i] =
                    1000.0 * (sum_angles[i].abs() as f64) / CURVATURES_SAMPLING_WIDTH;
            }
        }

        // 4 calculate the degree of curve
        //   For angle >= curvatures_angle_worst, we think it's enough to be worst. Should make the speed to be slowest.
        //   For angle <= curvatures_angle_best, we thins it's enough to be best. Should make the speed to be fastest.
        //   Use several steps [0 1 2...curvatures_sampling_number - 1] to describe the degree of curve. 0 is the flatest. curvatures_sampling_number - 1 is the sharpest
        // CurveAnalyzer.cpp:90-94
        let mut curvatures_norm = vec![0_i32; point_num];

        // CurveAnalyzer.cpp:95-100
        let mut sampling_step = vec![0_i32; CURVATURES_SAMPLING_NUMBER - 1];
        for i in 0..CURVATURES_SAMPLING_NUMBER - 1 {
            sampling_step[i] = ((2 * i + 1) * 50 / (CURVATURES_SAMPLING_NUMBER - 1)) as i32;
        }
        sampling_step[0] = 0;
        sampling_step[CURVATURES_SAMPLING_NUMBER - 2] = 100;

        // CurveAnalyzer.cpp:101-112
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

        // point, index, curve_degree
        // CurveAnalyzer.cpp:113-120
        let mut curvature_list: Vec<(Point, usize, i32)> = Vec::new();
        let mut last_curvature_norm = -1_i32;

        for i in 0..point_num {
            if curvatures_norm[i] != last_curvature_norm {
                last_curvature_norm = curvatures_norm[i];
                curvature_list.push((polygon.points[i], i, last_curvature_norm));
            }
        }

        // the last point should be the first point
        // CurveAnalyzer.cpp:121
        curvature_list.push((polygon.points[0], point_num, curvatures_norm[0]));

        // 5 split and modify the path according to the degree of curve
        // CurveAnalyzer.cpp:123-200
        if curvature_list.len() == 2 {
            // all paths has same curva_degree
            // CurveAnalyzer.cpp:124-128
            for path in paths.iter_mut() {
                path.set_curve_degree(curvature_list[0].2);
            }
        } else {
            // CurveAnalyzer.cpp:129-200
            let mut out: Vec<ExtrusionPath> = Vec::new();
            out.reserve(paths.len() + curvature_list.len() - 1);
            let mut j = 1_usize;
            let mut current_curva_norm = curvature_list[0].2;
            // C++: for (size_t i = 0; i < paths.size() && j < curvature_list.size(); i++)
            // CurveAnalyzer.cpp:134
            for i in 0..paths.len() {
                if j >= curvature_list.len() {
                    break;
                }

                // CurveAnalyzer.cpp:135-141
                if paths[i].last_point() == curvature_list[j].0 {
                    paths[i].set_curve_degree(current_curva_norm);
                    out.push(paths[i].clone());
                    current_curva_norm = curvature_list[j].2;
                    j += 1;
                    continue;
                }
                // CurveAnalyzer.cpp:142-154
                else if paths[i].first_point() == curvature_list[j].0 {
                    if paths[i].polyline.points.first() == paths[i].polyline.points.last() {
                        paths[i].set_curve_degree(current_curva_norm);
                        out.push(paths[i].clone());
                        current_curva_norm = curvature_list[j].2;
                        j += 1;
                        continue;
                    } else {
                        // should never happen
                        // CurveAnalyzer.cpp:151-152: assert(0);
                        debug_assert!(false);
                    }
                }

                // CurveAnalyzer.cpp:156-165
                if paths_length[i] <= polygon_length[curvature_list[j].1] as f64
                    || paths[i].last_point() == curvature_list[j].0
                {
                    // save paths[i] directly
                    paths[i].set_curve_degree(current_curva_norm);
                    out.push(paths[i].clone());
                    if paths[i].last_point() == curvature_list[j].0 {
                        current_curva_norm = curvature_list[j].2;
                        j += 1;
                    }
                } else {
                    // split paths[i]
                    // CurveAnalyzer.cpp:167-191
                    let mut current_path = paths[i].clone();
                    while j < curvature_list.len() {
                        // C++: current_path.polyline.split_at(curvature_list[j].first.first, &left, &right);
                        // The split point is a vertex of the polygon and therefore a vertex of the
                        // polyline, so Polyline::split_at resolves it via find_point -> split_at_index.
                        // CurveAnalyzer.cpp:170-171
                        let split_point = curvature_list[j].0;
                        let split_index = current_path
                            .polyline
                            .points
                            .iter()
                            .position(|p| *p == split_point)
                            .unwrap_or(0);
                        let (left, right) = current_path.polyline.split_at(split_index);

                        // C++: ExtrusionPath left_path(left, current_path);
                        // C++: left_path.set_curve_degree(current_curva_norm);
                        // CurveAnalyzer.cpp:172-174
                        let mut left_path = current_path.clone();
                        left_path.polyline = left;
                        left_path.set_curve_degree(current_curva_norm);
                        out.push(left_path);

                        // C++: ExtrusionPath right_path(right, current_path);
                        // C++: current_path = right_path;
                        // CurveAnalyzer.cpp:175-176
                        let mut right_path = current_path.clone();
                        right_path.polyline = right;
                        current_path = right_path;

                        current_curva_norm = curvature_list[j].2;
                        j += 1;
                        // CurveAnalyzer.cpp:180-190
                        if j < curvature_list.len()
                            && (paths_length[i] <= polygon_length[curvature_list[j].1] as f64
                                || paths[i].last_point() == curvature_list[j].0)
                        {
                            current_path.set_curve_degree(current_curva_norm);
                            // C++: out.push_back(current_path); then check last_point.
                            let advance = current_path.last_point() == curvature_list[j].0;
                            out.push(current_path);
                            if advance {
                                current_curva_norm = curvature_list[j].2;
                                j += 1;
                            }
                            break;
                        }
                    }
                }
            }

            // CurveAnalyzer.cpp:195-199
            paths.clear();
            paths.reserve(out.len());
            for p in out {
                paths.push(p);
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::extrusion_entity::ExtrusionRole;
    use crate::geometry::Polyline;

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
