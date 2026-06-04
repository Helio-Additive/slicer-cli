//! Model I/O utilities for converting between formats via a temporary STL.
//!
//! C++ Reference:
//! - Format/ModelIO.hpp
//!
//! Provides conversion from supported model formats to a temporary STL file
//! that can then be loaded by the existing STL loader, plus a convenience
//! function to delete the temporary file.

use crate::{Error, Result};
use std::path::{Path, PathBuf};

// ---------------------------------------------------------------------------
// Public API  (ModelIO.hpp)
// ---------------------------------------------------------------------------

/// Convert a supported model file to a temporary STL file.
///
/// Uses ModelIO (or an equivalent converter) to read the input file and write
/// a temporary STL.  The caller is responsible for deleting the temp file
/// afterwards via `delete_temp_file`.
///
/// Returns the path to the temporary STL file, or an empty string if
/// conversion failed.
///
/// ModelIO.hpp:10-11
pub fn make_temp_stl_with_modelio(input_file: &Path) -> Result<PathBuf> {
    // Determine the output path in the system temp directory.
    let stem = input_file
        .file_stem()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "model".to_string());

    let temp_dir = std::env::temp_dir();
    let temp_path = temp_dir.join(format!("{}_converted.stl", stem));

    // Detect format and convert.
    let ext = input_file
        .extension()
        .map(|e| e.to_string_lossy().to_lowercase())
        .unwrap_or_default();

    match ext.as_str() {
        "stl" => {
            // Already STL – just copy.
            std::fs::copy(input_file, &temp_path)
                .map_err(|e| Error::IO(format!("Failed to copy STL file to temp: {}", e)))?;
        }
        "obj" => {
            // Load OBJ and write as binary STL.
            let mut obj_info = crate::format::obj::ObjInfo::new();
            let mut message = String::new();
            let mesh =
                crate::format::obj::load_obj(input_file, &mut obj_info, &mut message, false)?;
            crate::format::stl::store_stl(&temp_path, &mesh, true)?;
        }
        _ => {
            // Unsupported format for direct conversion.
            return Err(Error::IO(format!(
                "Unsupported format for ModelIO conversion: {}",
                ext
            )));
        }
    }

    Ok(temp_path)
}

/// Delete a temporary file created by `make_temp_stl_with_modelio`.
///
/// Silently ignores errors (matching the C++ behaviour where the return
/// value is void and failure is not required).
///
/// ModelIO.hpp:17
pub fn delete_temp_file(temp_file: &Path) {
    let _ = std::fs::remove_file(temp_file);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_delete_temp_file_nonexistent() {
        // Should not panic even if the file doesn't exist.
        delete_temp_file(Path::new("/tmp/nonexistent_file_12345.stl"));
    }

    #[test]
    fn test_make_temp_stl_unsupported_format() {
        let result = make_temp_stl_with_modelio(Path::new("model.xyz"));
        assert!(result.is_err());
    }
}
