//! Object color utilities.
//!
//! C++ Reference:
//! - ObjColorUtils.hpp
//! - ObjColorUtils.cpp
//!
//! 1:1 faithful port of `ObjColorUtils.cpp`.
//!
//! PARTIAL PORT. Two symbols are blocked on dependencies that are not (yet)
//! available in the Rust crate and are noted inline:
//!
//! * `QuantKMeans` (ObjColorUtils.hpp:16-334) and the `obj_color_deal_algo`
//!   entry point (ObjColorUtils.cpp:7-19) are implemented entirely in terms of
//!   OpenCV (`cv::Mat`, `cv::kmeans`, `cvtColor`, `cv::theRNG`, ...). OpenCV is a
//!   native/dylib dependency and is wasm-unsafe, so it is intentionally NOT
//!   added. These symbols are blocked.
//!
//! * `extract_colors_to_obj_dialog` (ObjColorUtils.cpp:76-321) walks
//!   `model->objects[]->volumes[]` and mutates GUI/render-side structures
//!   (`Slic3r::ModelVolume`, `volume->origin_render_info_ptr`,
//!   `Slic3r::OriginRenderInfo`, `mesh_with_colors`, `vertices_with_colors`,
//!   `set_origin_mesh_render_type`). The Rust `Model`/`ModelObject`
//!   (`crate::model`) is a simplified port with no `volumes` and none of the
//!   render-info structures, so this symbol is blocked until those are ported.
//!
//! The tractable, self-contained free functions are ported faithfully below:
//! `check_is_all_undefined_color` and `get_face_color_from_binding`. Their
//! `UNDEFINE_COLOR` / `color_is_equal` dependencies are reused from
//! `crate::color` (the `Color.cpp` port) rather than duplicated.

use std::collections::HashMap;

use crate::format::obj::{TriangleColor, RGBA};

// ---------------------------------------------------------------------------
// Color helpers from `Color.hpp` / `Color.cpp`.
//
// `UNDEFINE_COLOR` (Color.hpp:10) and `color_is_equal` (Color.cpp:9-18) are now
// provided canonically by the `Color.cpp` port in `crate::color`, so they are
// reused here rather than duplicated. `crate::color::RGBA` and
// `crate::format::obj::RGBA` are both `[f32; 4]`, so they interoperate.
// ---------------------------------------------------------------------------

use crate::color::{color_is_equal, UNDEFINE_COLOR};

// ---------------------------------------------------------------------------
// obj_color_deal_algo  (ObjColorUtils.cpp:7-19)
//
// BLOCKED: depends on `QuantKMeans::apply`, which is implemented entirely with
// OpenCV (`cv::Mat`, `cv::kmeans`, `cvtColor`, ...). OpenCV is a native/dylib,
// wasm-unsafe dependency and is intentionally not added. Faithful signature is
// retained here for reference; it is not implemented.
// ---------------------------------------------------------------------------
//
// bool obj_color_deal_algo(std::vector<Slic3r::RGBA> & input_colors,
//                          std::vector<Slic3r::RGBA> & cluster_colors_from_algo,
//                          std::vector<int> &         cluster_labels_from_algo,
//                          char &                     cluster_number,
//                          int                        max_cluster)
// {
//     QuantKMeans quant(10);                                          // ObjColorUtils.cpp:13
//     quant.apply(input_colors, cluster_colors_from_algo,            // ObjColorUtils.cpp:14
//                 cluster_labels_from_algo, (int) cluster_number, max_cluster);
//     if (cluster_number == -1) {                                     // ObjColorUtils.cpp:15
//         return false;                                               // ObjColorUtils.cpp:16
//     }
//     return true;                                                    // ObjColorUtils.cpp:18
// }

// Check if all colors are undefined.
// ObjColorUtils.cpp:22-29
pub fn check_is_all_undefined_color(colors: &[RGBA]) -> bool {
    // ObjColorUtils.cpp:23
    for color in colors {
        // ObjColorUtils.cpp:24
        if !color_is_equal(*color, &UNDEFINE_COLOR) {
            // ObjColorUtils.cpp:25
            return false;
        }
    }
    // ObjColorUtils.cpp:28
    true
}

// Construct the color of each face from TriangleColors and color_group_map
// ObjColorUtils.cpp:32-74
pub fn get_face_color_from_binding(
    binding: &TriangleColor,
    color_group_map: &HashMap<i32, Vec<String>>,
    color_str_to_rgba: &HashMap<String, RGBA>,
) -> RGBA {
    // If the PID is invalid, return "undefined color".
    // ObjColorUtils.cpp:38
    if binding.pid < 0 {
        // ObjColorUtils.cpp:39
        return UNDEFINE_COLOR;
    }
    // Find color group
    // ObjColorUtils.cpp:42
    let group_iter = color_group_map.get(&binding.pid);
    // ObjColorUtils.cpp:43
    let group = match group_iter {
        Some(g) if !g.is_empty() => g,
        // ObjColorUtils.cpp:44
        _ => return UNDEFINE_COLOR,
    };
    // Check if there are vertex - level colors(p1 / p2 / p3 are different).
    // ObjColorUtils.cpp:47
    let mut has_vertex_colors = false;
    // ObjColorUtils.cpp:48
    for i in 0..3 {
        // ObjColorUtils.cpp:49-50
        if binding.indices[i] >= 0 && binding.indices[i] < group.len() as i32 {
            // ObjColorUtils.cpp:51
            has_vertex_colors = true;
            // ObjColorUtils.cpp:52
            break;
        }
    }
    // If vertex colors are available, use the color of the first vertex as the representative color of the face
    // (For vertex color cases, ObjColorDialog will use deal_vertex_color = true)
    // ObjColorUtils.cpp:57-58
    if has_vertex_colors && binding.indices[0] >= 0 && binding.indices[0] < group.len() as i32 {
        // ObjColorUtils.cpp:59
        let color_str = &group[binding.indices[0] as usize];
        // ObjColorUtils.cpp:60
        let rgba_iter = color_str_to_rgba.get(color_str);
        // ObjColorUtils.cpp:61
        if let Some(rgba) = rgba_iter {
            // ObjColorUtils.cpp:62
            return *rgba;
        }
    }
    // Otherwise, use the default color(pindex or the first color).
    // ObjColorUtils.cpp:66
    if !group.is_empty() {
        // ObjColorUtils.cpp:67
        let color_str = &group[0];
        // ObjColorUtils.cpp:68
        let rgba_iter = color_str_to_rgba.get(color_str);
        // ObjColorUtils.cpp:69
        if let Some(rgba) = rgba_iter {
            // ObjColorUtils.cpp:70
            return *rgba;
        }
    }
    // ObjColorUtils.cpp:73
    UNDEFINE_COLOR
}

// ---------------------------------------------------------------------------
// extract_colors_to_obj_dialog  (ObjColorUtils.cpp:76-321)
//
// BLOCKED: operates on `model->objects[]->volumes[]` and mutates GUI/render-side
// structures (`Slic3r::ModelVolume`, `volume->origin_render_info_ptr`,
// `Slic3r::OriginRenderInfo`, `mesh_with_colors`, `vertices_with_colors`,
// `set_origin_mesh_render_type`). The Rust `Model`/`ModelObject`
// (`crate::model`) is a simplified port with no `volumes` and none of the
// render-info structures. Cannot be faithfully ported until those land.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn color_is_equal_within_tolerance() {
        // Color.cpp:9 - difference under 0.9/255 is considered equal.
        let a: RGBA = [0.5, 0.5, 0.5, 1.0];
        let b: RGBA = [0.5 + 0.001, 0.5, 0.5, 1.0];
        assert!(color_is_equal(a, &b));
        let c: RGBA = [0.6, 0.5, 0.5, 1.0];
        assert!(!color_is_equal(a, &c));
    }

    #[test]
    fn all_undefined_detected() {
        // ObjColorUtils.cpp:22
        let colors = vec![UNDEFINE_COLOR, UNDEFINE_COLOR];
        assert!(check_is_all_undefined_color(&colors));
        let mixed = vec![UNDEFINE_COLOR, [1.0, 0.0, 0.0, 1.0]];
        assert!(!check_is_all_undefined_color(&mixed));
        // Empty vector -> all (vacuously) undefined.
        assert!(check_is_all_undefined_color(&[]));
    }

    #[test]
    fn face_color_invalid_pid_is_undefined() {
        // ObjColorUtils.cpp:38
        let mut binding = TriangleColor::new();
        binding.pid = -1;
        let color = get_face_color_from_binding(&binding, &HashMap::new(), &HashMap::new());
        assert_eq!(color, UNDEFINE_COLOR);
    }

    #[test]
    fn face_color_uses_first_vertex_when_present() {
        // ObjColorUtils.cpp:57-62
        let mut binding = TriangleColor::new();
        binding.pid = 1;
        binding.indices = [0, 1, 2];
        let mut group_map: HashMap<i32, Vec<String>> = HashMap::new();
        group_map.insert(
            1,
            vec!["#FF0000FF".to_string(), "#00FF00FF".to_string(), "#0000FFFF".to_string()],
        );
        let mut str_to_rgba: HashMap<String, RGBA> = HashMap::new();
        str_to_rgba.insert("#FF0000FF".to_string(), [1.0, 0.0, 0.0, 1.0]);
        str_to_rgba.insert("#00FF00FF".to_string(), [0.0, 1.0, 0.0, 1.0]);
        str_to_rgba.insert("#0000FFFF".to_string(), [0.0, 0.0, 1.0, 1.0]);
        let color = get_face_color_from_binding(&binding, &group_map, &str_to_rgba);
        assert_eq!(color, [1.0, 0.0, 0.0, 1.0]);
    }
}
