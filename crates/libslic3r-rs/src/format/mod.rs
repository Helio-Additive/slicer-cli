//! Format module - file format handling for 3D models
//!
//! This module provides support for loading and saving various 3D model file formats
//! including STL, AMF, 3MF, OBJ, STEP, and others.
//!
//! C++ Reference: BambuStudio/src/libslic3r/Format/

pub mod amf;
pub mod bbs_3mf;
pub mod model_io;
pub mod obj;
pub mod objparser;
pub mod sl1;
/// 1:1 port of `libslic3r/format.hpp` (the `Slic3r::format(...)` boost::format
/// wrapper). Lives here because `src/format.rs` would collide with this
/// `src/format/` module directory; the public macro is `slic3r_format!`,
/// exported at crate root.
pub mod slic3r_format;
pub mod step;
pub mod stl;
pub mod svg;
pub mod three_mf;
pub mod utilities;

// Re-export common utilities
pub use utilities::{
    check_file_exists, detect_format, get_file_size, is_format_supported, normalize_path,
    FileFormat,
};
