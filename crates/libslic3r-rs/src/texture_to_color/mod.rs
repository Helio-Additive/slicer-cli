//! Port of `src/libslic3r/TextureToColor/`: turning a textured mesh into a mesh whose faces carry
//! one of a few colors (`namespace Slic3r::tex2color`).
//!
//! One Rust file per C++ file of the directory, named after it.

pub mod callbacks;
pub mod cgal_utils;
pub mod color_utils;
pub mod repair;
#[allow(clippy::module_inception)]
pub mod texture_to_color;
pub mod tri_mesh;
