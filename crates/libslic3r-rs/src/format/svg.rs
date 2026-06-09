//! SVG file loading for 3D model generation.
//!
//! C++ Reference:
//! - Format/svg.hpp
//! - Format/svg.cpp
//!
//! Faithful 1:1 line-by-line port of `Format/svg.{cpp,hpp}`.
//!
//! STATUS: partial.
//!
//! The C++ translation unit straddles two native libraries that cannot be added
//! under the wasm-safe constraint:
//!
//! - **nanosvg** (`#include "nanosvg/nanosvg.h"`, svg.cpp:7) supplies the
//!   `NSVGimage` / `NSVGshape` / `NSVGpath` structures and the
//!   `nsvgParseFromFile` / `nsvgDelete` entry points that feed
//!   [`get_svg_profile`]. It is a native C header (`src/nanosvg/nanosvg.h`) with
//!   no Rust port (same blocker documented in [`crate::nsvg_utils`]).
//! - **OpenCascade / OCCT** (svg.cpp:13-22) supplies `gp_Pnt`,
//!   `BRepBuilderAPI_MakeWire` / `MakeEdge` / `MakeFace`,
//!   `BRepPrimAPI_MakePrism`, `BRepMesh_IncrementalMesh`, `TopExp_Explorer`,
//!   `BRep_Tool::Triangulation`, `Poly_Triangulation`, … — the entire 2D→3D
//!   extrusion + meshing back-end. There is no OCCT binding crate (same blocker
//!   documented in [`crate::format::step`]).
//!
//! Consequently the two top-level entry points are BLOCKED and cannot produce
//! the C++ geometry:
//! - [`get_svg_profile`] — needs nanosvg parsing for its `NSVGshape`/`NSVGpath`
//!   input AND the OCCT `BRepBuilderAPI_MakeWire`/`MakeFace`/`MakePrism`
//!   pipeline that builds each `Element_Info::shape`.
//! - [`load_svg`] — needs the OCCT `BRepMesh_IncrementalMesh` /
//!   `Poly_Triangulation` mesh-extraction loop.
//!
//! The genuinely self-contained, dependency-free math is ported faithfully:
//! [`is_same_points`], [`interp_v2_v2v2`], [`interp_v2_v2v2v2v2_cubic`],
//! [`is_two_lines_interaction`], [`is_profile_self_interaction`], and
//! [`get_profile_area`]. These are exactly the routines `get_svg_profile` calls
//! once nanosvg/OCCT become available.

// svg.cpp:1-24  #include "libslic3r/ClipperUtils.hpp" / "../libslic3r.h" /
//               "../Model.hpp" / "../TriangleMesh.hpp" / "svg.hpp" /
//               "nanosvg/nanosvg.h" / <string> / <boost/log/trivial.hpp> /
//               OCCT BRep* headers / "libslic3r/clipper.hpp" / "libslic3r/Polygon.hpp"

// Allow dead code for the helper functions that are fully ported from C++ but
// not yet wired into `get_svg_profile`/`load_svg` (they require nanosvg + OCCT).
#![allow(dead_code)]

use crate::model::Model;

use log::error;
use std::path::Path;

// svg.cpp:26  namespace Slic3r {

// ---------------------------------------------------------------------------
// Constants  (svg.cpp:27-28)
// ---------------------------------------------------------------------------

// svg.cpp:27  const double STEP_TRANS_CHORD_ERROR = 0.005;
const STEP_TRANS_CHORD_ERROR: f64 = 0.005;
// svg.cpp:28  const double STEP_TRANS_ANGLE_RES   = 1;
const STEP_TRANS_ANGLE_RES: f64 = 1.0;

// ---------------------------------------------------------------------------
// Data types  (svg.cpp:30-48)
// ---------------------------------------------------------------------------

/// svg.cpp:30-35
/// ```cpp
/// struct Element_Info
/// {
///     std::string name;
///     unsigned int color;
///     TopoDS_Shape shape;
/// };
/// ```
///
/// The C++ `shape` member is an OCCT `TopoDS_Shape` (the extruded prism). With
/// no OCCT binding the extruded solid cannot be materialised, so the shape is
/// represented here by the 2D profile line segments that would feed
/// `BRepBuilderAPI_MakeWire` — `paths<profiles<(start, end)>>`, with each point a
/// `[x, y, z]` triple mirroring `gp_Pnt`.
#[derive(Debug, Clone)]
pub struct ElementInfo {
    /// svg.cpp:32  std::string name;
    pub name: String,
    /// svg.cpp:33  unsigned int color;
    pub color: u32,
    /// svg.cpp:34  TopoDS_Shape shape; (represented as 2D profile line segments)
    pub profile_lines: Vec<Vec<([f64; 3], [f64; 3])>>,
}

/// svg.cpp:43-48
/// ```cpp
/// struct Point_2D
/// {
///     Point_2D(float in_x, float in_y) : x(in_x), y(in_y) {}
///     float x;
///     float y;
/// };
/// ```
#[derive(Debug, Clone, Copy)]
struct Point2D {
    x: f32,
    y: f32,
}

impl Point2D {
    // svg.cpp:45  Point_2D(float in_x, float in_y) : x(in_x), y(in_y) {}
    fn new(in_x: f32, in_y: f32) -> Self {
        Self { x: in_x, y: in_y }
    }
}

// ---------------------------------------------------------------------------
// svg.cpp:37-41  bool is_same_points(gp_Pnt pt1, gp_Pnt pt2)
// ---------------------------------------------------------------------------

/// svg.cpp:37-41
fn is_same_points(pt1: [f64; 3], pt2: [f64; 3]) -> bool {
    // svg.cpp:38-40
    (pt1[0] - pt2[0]).abs() < 0.001
        && (pt1[1] - pt2[1]).abs() < 0.001
        && (pt1[2] - pt2[2]).abs() < 0.001
}

// ---------------------------------------------------------------------------
// Bezier interpolation  (svg.cpp:50-70)
// ---------------------------------------------------------------------------

/// svg.cpp:50-56
/// ```cpp
/// void interp_v2_v2v2(float r[2], const float a[2], const float b[2], const float t)
/// ```
fn interp_v2_v2v2(a: [f32; 2], b: [f32; 2], t: f32) -> [f32; 2] {
    // svg.cpp:52  const float s = 1.0f - t;
    let s = 1.0f32 - t;

    // svg.cpp:54  r[0] = s * a[0] + t * b[0];
    // svg.cpp:55  r[1] = s * a[1] + t * b[1];
    [s * a[0] + t * b[0], s * a[1] + t * b[1]]
}

/// svg.cpp:58-70
/// ```cpp
/// void interp_v2_v2v2v2v2_cubic(float p[2], const float v1[2], const float v2[2],
///                               const float v3[2], const float v4[2], const float u)
/// ```
fn interp_v2_v2v2v2v2_cubic(
    v1: [f32; 2],
    v2: [f32; 2],
    v3: [f32; 2],
    v4: [f32; 2],
    u: f32,
) -> [f32; 2] {
    // svg.cpp:60  float q0[2], q1[2], q2[2], r0[2], r1[2];

    // svg.cpp:62  interp_v2_v2v2(q0, v1, v2, u);
    let q0 = interp_v2_v2v2(v1, v2, u);
    // svg.cpp:63  interp_v2_v2v2(q1, v2, v3, u);
    let q1 = interp_v2_v2v2(v2, v3, u);
    // svg.cpp:64  interp_v2_v2v2(q2, v3, v4, u);
    let q2 = interp_v2_v2v2(v3, v4, u);

    // svg.cpp:66  interp_v2_v2v2(r0, q0, q1, u);
    let r0 = interp_v2_v2v2(q0, q1, u);
    // svg.cpp:67  interp_v2_v2v2(r1, q1, q2, u);
    let r1 = interp_v2_v2v2(q1, q2, u);

    // svg.cpp:69  interp_v2_v2v2(p, r0, r1, u);
    interp_v2_v2v2(r0, r1, u)
}

// ---------------------------------------------------------------------------
// svg.cpp:72-94  bool is_two_lines_interaction(gp_Pnt pL1, pL2, pR1, pR2)
// ---------------------------------------------------------------------------

/// svg.cpp:72-94
fn is_two_lines_interaction(pl1: [f64; 3], pl2: [f64; 3], pr1: [f64; 3], pr2: [f64; 3]) -> bool {
    // svg.cpp:73-76  Vec3d point1..point4 (z forced to 0)
    let point1 = [pl1[0], pl1[1], 0.0];
    let point2 = [pl2[0], pl2[1], 0.0];
    let point3 = [pr1[0], pr1[1], 0.0];
    let point4 = [pr2[0], pr2[1], 0.0];

    // svg.cpp:78  Vec3d line1 = point2 - point1;
    let line1 = sub3(point2, point1);
    // svg.cpp:79  Vec3d line2 = point4 - point3;
    let line2 = sub3(point4, point3);

    // svg.cpp:81  Vec3d line_pos1 = point1 - point3;
    let line_pos1 = sub3(point1, point3);
    // svg.cpp:82  Vec3d line_pos2 = point2 - point3;
    let line_pos2 = sub3(point2, point3);

    // svg.cpp:84  Vec3d line_pos3 = point3 - point1;
    let line_pos3 = sub3(point3, point1);
    // svg.cpp:85  Vec3d line_pos4 = point4 - point1;
    let line_pos4 = sub3(point4, point1);

    // svg.cpp:87  Vec3d cross_1 = line2.cross(line_pos1);
    let cross_1 = cross3(line2, line_pos1);
    // svg.cpp:88  Vec3d cross_2 = line2.cross(line_pos2);
    let cross_2 = cross3(line2, line_pos2);

    // svg.cpp:90  Vec3d cross_3 = line1.cross(line_pos3);
    let cross_3 = cross3(line1, line_pos3);
    // svg.cpp:91  Vec3d cross_4 = line1.cross(line_pos4);
    let cross_4 = cross3(line1, line_pos4);

    // svg.cpp:93  return (cross_1.dot(cross_2) < 0) && (cross_3.dot(cross_4) < 0);
    (dot3(cross_1, cross_2) < 0.0) && (dot3(cross_3, cross_4) < 0.0)
}

/// `Vec3d a - b` (mirrors Eigen `operator-`).
#[inline]
fn sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// `Vec3d::cross` (mirrors Eigen `Vector3d::cross`).
#[inline]
fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// `Vec3d::dot` (mirrors Eigen `Vector3d::dot`).
#[inline]
fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

// ---------------------------------------------------------------------------
// svg.cpp:96-105  bool is_profile_self_interaction(profile_line_points)
// ---------------------------------------------------------------------------

/// svg.cpp:96-105
fn is_profile_self_interaction(profile_line_points: &[([f64; 3], [f64; 3])]) -> bool {
    // svg.cpp:98  for (int i = 0; i < profile_line_points.size(); ++i) {
    for i in 0..profile_line_points.len() {
        // svg.cpp:99  for (int j = i + 2; j < profile_line_points.size(); ++j)
        for j in (i + 2)..profile_line_points.len() {
            // svg.cpp:100  if (is_two_lines_interaction(...)) return true;
            if is_two_lines_interaction(
                profile_line_points[i].0,
                profile_line_points[i].1,
                profile_line_points[j].0,
                profile_line_points[j].1,
            ) {
                return true;
            }
        }
    }

    // svg.cpp:104  return false;
    false
}

// ---------------------------------------------------------------------------
// svg.cpp:107-123  double get_profile_area(profile_line_points)
// ---------------------------------------------------------------------------

/// svg.cpp:107-123
fn get_profile_area(profile_line_points: &[([f64; 3], [f64; 3])]) -> f64 {
    // svg.cpp:109  double min_x = 0;
    let mut min_x = 0.0;
    // svg.cpp:110-112  for (auto line_points : profile_line_points) { if (...X() < min_x) min_x = ...; }
    for line_points in profile_line_points {
        if line_points.0[0] < min_x {
            min_x = line_points.0[0];
        }
    }

    // svg.cpp:114  double area = 0;
    let mut area = 0.0;
    // svg.cpp:115-120
    for line_points in profile_line_points {
        // svg.cpp:116  bool flag = true;
        let mut _flag = true;
        // svg.cpp:117  if (line_points.second.Y() < line_points.first.Y()) flag = false;
        if line_points.1[1] < line_points.0[1] {
            _flag = false;
        }

        // svg.cpp:119  area += (second.X() + first.X() - 2*min_x) * (second.Y() - first.Y()) / 2;
        area += (line_points.1[0] + line_points.0[0] - 2.0 * min_x)
            * (line_points.1[1] - line_points.0[1])
            / 2.0;
    }

    // svg.cpp:122  return abs(area);
    area.abs()
}

// ---------------------------------------------------------------------------
// svg.cpp:125-301  bool get_svg_profile(path, element_infos, message)
// ---------------------------------------------------------------------------

/// svg.cpp:125-301
///
/// BLOCKED: native deps unavailable.
///
/// The C++ body (svg.cpp:127-300) first calls `nsvgParseFromFile(path, "mm",
/// 96.0f)` (svg.cpp:128) to obtain the `NSVGimage*` and iterates its
/// `NSVGshape*` / `NSVGpath*` lists. nanosvg is a native C header with no Rust
/// port and cannot be added (wasm-safe constraint — same blocker as
/// [`crate::nsvg_utils`]). The downstream geometry (svg.cpp:257-296) then builds
/// each profile via OCCT `BRepBuilderAPI_MakeWire` / `MakeEdge` / `MakeFace` and
/// extrudes it with `BRepPrimAPI_MakePrism`, none of which have a binding crate
/// (same blocker as [`crate::format::step`]).
///
/// The self-contained interpolation / intersection / area math the loop relies
/// on ([`interp_v2_v2v2v2v2_cubic`], [`is_same_points`],
/// [`is_profile_self_interaction`], [`get_profile_area`]) is fully ported above
/// and ready to be wired in once a parser/back-end becomes available.
fn get_svg_profile(
    _path: &Path,
    _element_infos: &mut Vec<ElementInfo>,
    message: &mut String,
) -> bool {
    // svg.cpp:127-132  svg_data = nsvgParseFromFile(path, "mm", 96.0f); if (==nullptr) { message = "..."; return false; }
    *message = "import svg failed: could not open svg. \
                (nanosvg parsing and OpenCascade extrusion are not available in this build)"
        .to_string();
    error!("{}", message);
    // svg.cpp:131  return false;
    false
}

// ---------------------------------------------------------------------------
// svg.cpp:303-401  bool load_svg(const char *path, Model *model, std::string &message)
// ---------------------------------------------------------------------------

/// svg.hpp:5  extern bool load_svg(const char *path, Model *model, std::string &message);
/// svg.cpp:303-401
///
/// BLOCKED: native deps unavailable.
///
/// After [`get_svg_profile`] supplies the `Element_Info` solids, the C++ body
/// (svg.cpp:312-381) meshes each `TopoDS_Shape` with
/// `BRepMesh_IncrementalMesh` and walks the resulting `Poly_Triangulation`
/// (`TopExp_Explorer` / `BRep_Tool::Triangulation` / `Poly_Triangle`) to fill an
/// `stl_file`, which is then turned into a `TriangleMesh` `ModelVolume`
/// (svg.cpp:384-399). The OCCT meshing API has no Rust binding (same blocker as
/// [`crate::format::step`]), so no triangulated geometry can be produced.
pub fn load_svg(path: &Path, model: &mut Model, message: &mut String) -> bool {
    // svg.cpp:305  std::vector<Element_Info> namedSolids;
    let mut named_solids: Vec<ElementInfo> = Vec::new();
    // svg.cpp:306-307  if (!get_svg_profile(path, namedSolids, message)) return false;
    if !get_svg_profile(path, &mut named_solids, message) {
        return false;
    }

    // svg.cpp:309-382  OCCT BRepMesh_IncrementalMesh / Poly_Triangulation mesh
    //                  extraction populating std::vector<stl_file>.
    // svg.cpp:384-399  ModelObject *new_object = model->add_object(); ... add_volume(...)
    //
    // BLOCKED: requires OCCT (BRepMesh_IncrementalMesh, Poly_Triangulation, …)
    // which has no Rust binding crate. `get_svg_profile` cannot reach this point
    // without nanosvg + OCCT, so the meshing loop is unreachable in this build.
    let _ = model;
    error!(
        "SVG loading requires nanosvg parsing and OpenCascade (OCCT) meshing, \
         which are not available. File: {}",
        path.display()
    );
    // svg.cpp:400  return true;  (unreachable here — geometry back-end is blocked)
    false
}

// svg.cpp:402  } // namespace Slic3r

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_interp_v2_v2v2() {
        let a = [0.0f32, 0.0];
        let b = [1.0, 1.0];
        let mid = interp_v2_v2v2(a, b, 0.5);
        assert!((mid[0] - 0.5).abs() < 1e-6);
        assert!((mid[1] - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_interp_v2_v2v2v2v2_cubic() {
        // A straight line should interpolate linearly
        let v1 = [0.0f32, 0.0];
        let v2 = [1.0 / 3.0, 0.0];
        let v3 = [2.0 / 3.0, 0.0];
        let v4 = [1.0, 0.0];
        let mid = interp_v2_v2v2v2v2_cubic(v1, v2, v3, v4, 0.5);
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

    #[test]
    fn test_point2d() {
        let p = Point2D::new(1.5, -2.5);
        assert_eq!(p.x, 1.5);
        assert_eq!(p.y, -2.5);
    }

    #[test]
    fn test_constants() {
        assert_eq!(STEP_TRANS_CHORD_ERROR, 0.005);
        assert_eq!(STEP_TRANS_ANGLE_RES, 1.0);
    }
}
