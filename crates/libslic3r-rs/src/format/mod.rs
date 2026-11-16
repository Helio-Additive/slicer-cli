//! Format module - file format handling for 3D models
//!
//! This module provides support for loading and saving various 3D model file formats
//! including STL, AMF, 3MF, OBJ, STEP, and others.
//!
//! C++ Reference: BambuStudio/src/libslic3r/Format/

pub mod amf;
pub mod assembly_steps_json;
pub mod assembly_tree_json;
pub mod bbs_3mf;
/// Port of `libslic3r/format.hpp` (`Slic3r::format(...)`, the wrapper around boost::format). It is
/// here, under its own name, because `src/format.rs` cannot stand beside this `src/format/`
/// directory; the macro `slic3r_format!` is exported at the crate root.
pub mod format;
pub mod model_io;
pub mod obj;
pub mod objparser;
pub mod resource_path_utils;
pub mod sl1;
pub mod step;
pub mod svg;
pub mod three_mf;
pub mod utilities;

// Re-export common utilities
pub use utilities::{
    check_file_exists, detect_format, get_file_size, is_format_supported, normalize_path,
    FileFormat,
};
