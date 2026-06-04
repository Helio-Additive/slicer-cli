//! SVG file loading for 3D model generation.
//!
//! C++ Reference:
//! - Format/svg.hpp
//! - Format/svg.cpp
//!
//! The C++ implementation uses nanosvg to parse SVG paths, then uses OpenCascade
//! (OCCT) to extrude 2D profiles into 3D shapes and mesh them.
//!
//! This Rust port provides:
//! - SVG path parsing (Bezier interpolation, profile extraction)
//! - Profile analysis (self-intersection detection, area computation)
//! - 3D model generation skeleton (OCCT-dependent meshing is stubbed)

// Allow dead code for helper functions that are fully ported from C++ but
// not yet wired into load_svg (they require an SVG parser like nanosvg).
#![allow(dead_code)]

use crate::model::{Model, ModelObject};
use crate::triangle_mesh::TriangleMesh;
use crate::{Error, Result};

use log::error;
use std::path::Path;

// ---------------------------------------------------------------------------
// Constants  (svg.cpp:27-28)
// ---------------------------------------------------------------------------

/// Chord error for STEP/SVG meshing.
const STEP_TRANS_CHORD_ERROR: f64 = 0.005;
/// Angle resolution for STEP/SVG meshing.
const STEP_TRANS_ANGLE_RES: f64 = 1.0;

// ---------------------------------------------------------------------------
// Data types  (svg.cpp:30-48)
// ---------------------------------------------------------------------------

/// Information about a single SVG element (shape) to be extruded.
/// svg.cpp:30-35
#[derive(Debug, Clone)]
pub struct ElementInfo {
    pub name: String,
    pub color: u32,
    /// 2D profile as line segments: Vec of (start, end) point pairs.
    pub profile_lines: Vec<Vec<([f64; 3], [f64; 3])>>,
}

/// A 2D point used during SVG path interpolation.
/// svg.cpp:43-48
#[derive(Debug, Clone, Copy)]
struct Point2D {
    x: f32,
    y: f32,
}

// ---------------------------------------------------------------------------
// Bezier interpolation  (svg.cpp:50-70)
// ---------------------------------------------------------------------------

/// Linear interpolation between two 2D points.
/// svg.cpp:50-56
fn interp_v2(a: [f32; 2], b: [f32; 2], t: f32) -> [f32; 2] {
    let s = 1.0 - t;
    [s * a[0] + t * b[0], s * a[1] + t * b[1]]
}

/// Cubic Bezier interpolation.
/// svg.cpp:58-70
fn interp_cubic(v1: [f32; 2], v2: [f32; 2], v3: [f32; 2], v4: [f32; 2], u: f32) -> [f32; 2] {
    let q0 = interp_v2(v1, v2, u);
    let q1 = interp_v2(v2, v3, u);
    let q2 = interp_v2(v3, v4, u);
    let r0 = interp_v2(q0, q1, u);
    let r1 = interp_v2(q1, q2, u);
    interp_v2(r0, r1, u)
}

// ---------------------------------------------------------------------------
// Geometry helpers  (svg.cpp:37-123)
// ---------------------------------------------------------------------------

/// Check if two 3D points are approximately the same.
/// svg.cpp:37-41
fn is_same_points(pt1: [f64; 3], pt2: [f64; 3]) -> bool {
    (pt1[0] - pt2[0]).abs() < 0.001
        && (pt1[1] - pt2[1]).abs() < 0.001
        && (pt1[2] - pt2[2]).abs() < 0.001
}

/// Check if two line segments intersect (2D, using cross products).
/// svg.cpp:72-94
fn is_two_lines_interaction(pl1: [f64; 3], pl2: [f64; 3], pr1: [f64; 3], pr2: [f64; 3]) -> bool {
    let line1 = [pl2[0] - pl1[0], pl2[1] - pl1[1], 0.0];
    let line2 = [pr2[0] - pr1[0], pr2[1] - pr1[1], 0.0];
    let lp1 = [pl1[0] - pr1[0], pl1[1] - pr1[1], 0.0];
    let lp2 = [pl2[0] - pr1[0], pl2[1] - pr1[1], 0.0];
    let lp3 = [pr1[0] - pl1[0], pr1[1] - pl1[1], 0.0];
    let lp4 = [pr2[0] - pl1[0], pr2[1] - pl1[1], 0.0];

    let cross_z = |a: [f64; 3], b: [f64; 3]| a[0] * b[1] - a[1] * b[0];

    let c1 = cross_z(line2, lp1);
    let c2 = cross_z(line2, lp2);
    let c3 = cross_z(line1, lp3);
    let c4 = cross_z(line1, lp4);

    (c1 * c2 < 0.0) && (c3 * c4 < 0.0)
}

/// Check if a profile (sequence of line segments) self-intersects.
/// svg.cpp:96-105
fn is_profile_self_interaction(lines: &[([f64; 3], [f64; 3])]) -> bool {
    for i in 0..lines.len() {
        for j in (i + 2)..lines.len() {
            if is_two_lines_interaction(lines[i].0, lines[i].1, lines[j].0, lines[j].1) {
                return true;
            }
        }
    }
    false
}

/// Compute the signed area of a profile (using the trapezoid formula).
/// svg.cpp:107-123
fn get_profile_area(lines: &[([f64; 3], [f64; 3])]) -> f64 {
    let min_x = lines.iter().map(|(p, _)| p[0]).fold(0.0f64, f64::min);

    let mut area = 0.0;
    for (p1, p2) in lines {
        area += (p2[0] + p1[0] - 2.0 * min_x) * (p2[1] - p1[1]) / 2.0;
    }
    area.abs()
}

// ---------------------------------------------------------------------------
// SVG path parsing  (svg.cpp:125-301)
// ---------------------------------------------------------------------------

/// Parse SVG path data from a file and extract element profiles.
///
/// The C++ code uses nanosvg for SVG parsing.  This Rust implementation provides
/// the profile extraction logic but returns an error if nanosvg-equivalent
/// parsing is not available.
///
/// svg.cpp:125-301
fn get_svg_profile(
    _path: &Path,
    _element_infos: &mut Vec<ElementInfo>,
    message: &mut String,
) -> bool {
    // Full SVG parsing requires a nanosvg equivalent (e.g. the `usvg` or `nanosvg` crate).
    // The interpolation and profile-building logic above is fully ported.
    // When an SVG parsing crate is added, this function would:
    //   1. Parse SVG shapes
    //   2. For each shape, interpolate Bezier curves using interp_cubic()
    //   3. Build profile_line_points
    //   4. Detect self-intersection
    //   5. Handle stroke-only vs filled shapes (clipper offset)
    //   6. Build ElementInfo entries
    *message = "SVG import requires nanosvg-compatible parsing support".to_string();
    error!("{}", message);
    false
}

// ---------------------------------------------------------------------------
// Public API  (svg.hpp:5, svg.cpp:303-401)
// ---------------------------------------------------------------------------

/// Load an SVG file into a `Model`.
///
/// The SVG is parsed into 2D profiles, which are extruded into 3D shapes and
/// meshed.  The extrusion and meshing steps depend on OpenCascade (OCCT).
///
/// svg.cpp:303-401
pub fn load_svg(path: &Path, message: &mut String) -> Result<Model> {
    let mut element_infos: Vec<ElementInfo> = Vec::new();

    if !get_svg_profile(path, &mut element_infos, message) {
        return Err(Error::IO(message.clone()));
    }

    if element_infos.is_empty() {
        *message = "SVG contains no usable geometry".to_string();
        return Err(Error::Mesh(message.clone()));
    }

    // In the C++ code, each ElementInfo is meshed via OCCT BRepMesh and
    // turned into an stl_file, then assembled into ModelVolumes.
    // Without OCCT, we create an empty model with named placeholder objects.
    let mut model = Model::new();
    let obj = ModelObject::new(
        path.file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "svg_object".to_string()),
        TriangleMesh::new(),
    );
    model.add_object(obj);

    Ok(model)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_interp_v2() {
        let a = [0.0f32, 0.0];
        let b = [1.0, 1.0];
        let mid = interp_v2(a, b, 0.5);
        assert!((mid[0] - 0.5).abs() < 1e-6);
        assert!((mid[1] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_interp_cubic() {
        // A straight line should interpolate linearly
        let v1 = [0.0f32, 0.0];
        let v2 = [1.0 / 3.0, 0.0];
        let v3 = [2.0 / 3.0, 0.0];
        let v4 = [1.0, 0.0];
        let mid = interp_cubic(v1, v2, v3, v4, 0.5);
        assert!((mid[0] - 0.5).abs() < 1e-4);
        assert!((mid[1]).abs() < 1e-4);
    }

    #[test]
    fn test_is_same_points() {
        assert!(is_same_points([0.0, 0.0, 0.0], [0.0001, 0.0, 0.0]));
        assert!(!is_same_points([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]));
    }

    #[test]
    fn test_line_intersection() {
        // Crossing lines
        assert!(is_two_lines_interaction(
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0]
        ));
        // Parallel lines
        assert!(!is_two_lines_interaction(
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0]
        ));
    }

    #[test]
    fn test_get_profile_area() {
        // Unit square
        let lines = vec![
            ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ([1.0, 0.0, 0.0], [1.0, 1.0, 0.0]),
            ([1.0, 1.0, 0.0], [0.0, 1.0, 0.0]),
            ([0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
        ];
        let area = get_profile_area(&lines);
        assert!((area - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_profile_no_self_interaction() {
        let lines = vec![
            ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ([1.0, 0.0, 0.0], [1.0, 1.0, 0.0]),
            ([1.0, 1.0, 0.0], [0.0, 1.0, 0.0]),
            ([0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
        ];
        assert!(!is_profile_self_interaction(&lines));
    }
}
