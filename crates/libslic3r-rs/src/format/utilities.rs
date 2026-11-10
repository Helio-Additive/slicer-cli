//! Format utilities - common helpers for file format handling
//!
//! This module provides common utilities used across different file format
//! handlers (STL, AMF, 3MF, OBJ, etc.).

/// C++ Reference: format.hpp
/// Common format handling utilities
use crate::{Error, Result};
use std::path::Path;

// ---------------------------------------------------------------------------
// Format Detection
// ---------------------------------------------------------------------------

/// Supported 3D model file formats
/// format.hpp:15-25
/// C++: enum class FileFormat {
/// C++:     STL,
/// C++:     AMF,
/// C++:     ThreeMF,
/// C++:     OBJ,
/// C++:     STEP,
/// C++:     Unknown
/// C++: };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FileFormat {
    STL,
    AMF,
    ThreeMF,
    OBJ,
    STEP,
    Unknown,
}

/// Detect file format from file extension
/// format.hpp:28
/// C++: FileFormat detect_format(const std::string& path);
pub fn detect_format(path: &Path) -> FileFormat {
    // Extract extension and match against known formats
    // format.hpp:29-45
    // C++: FileFormat detect_format(const std::string& path) {
    // C++:     std::string ext = boost::filesystem::extension(path);
    // C++:     boost::algorithm::to_lower(ext);
    // C++:     if (ext == ".stl") return FileFormat::STL;
    // C++:     if (ext == ".amf") return FileFormat::AMF;
    // C++:     if (ext == ".3mf") return FileFormat::ThreeMF;
    // C++:     if (ext == ".obj") return FileFormat::OBJ;
    // C++:     if (ext == ".step" || ext == ".stp") return FileFormat::STEP;
    // C++:     return FileFormat::Unknown;
    // C++: }
    match path.extension().and_then(|s| s.to_str()) {
        Some("stl") | Some("STL") => FileFormat::STL,
        Some("amf") | Some("AMF") => FileFormat::AMF,
        Some("3mf") | Some("3MF") => FileFormat::ThreeMF,
        Some("obj") | Some("OBJ") => FileFormat::OBJ,
        Some("step") | Some("stp") | Some("STEP") | Some("STP") => FileFormat::STEP,
        _ => FileFormat::Unknown,
    }
}

/// Check if file format is supported for reading
/// format.hpp:48
/// C++: bool is_format_supported(FileFormat format);
pub fn is_format_supported(format: FileFormat) -> bool {
    // Return true for formats we can load
    // format.hpp:49-55
    // C++: bool is_format_supported(FileFormat format) {
    // C++:     return format == FileFormat::STL ||
    // C++:            format == FileFormat::AMF ||
    // C++:            format == FileFormat::ThreeMF ||
    // C++:            format == FileFormat::OBJ;
    // C++: }
    matches!(
        format,
        FileFormat::STL | FileFormat::AMF | FileFormat::ThreeMF | FileFormat::OBJ
    )
}

// ---------------------------------------------------------------------------
// File I/O Utilities
// ---------------------------------------------------------------------------

/// Check if file exists and is readable
/// format.hpp:58
/// C++: bool check_file_exists(const std::string& path);
pub fn check_file_exists(path: &Path) -> bool {
    // Verify file exists and can be opened
    // format.hpp:59-62
    // C++: bool check_file_exists(const std::string& path) {
    // C++:     return boost::filesystem::exists(path) && boost::filesystem::is_regular_file(path);
    // C++: }
    path.exists() && path.is_file()
}

/// Get file size in bytes
/// format.hpp:65
/// C++: size_t get_file_size(const std::string& path);
pub fn get_file_size(path: &Path) -> Result<u64> {
    // Return file size or error if not accessible
    // format.hpp:66-72
    // C++: size_t get_file_size(const std::string& path) {
    // C++:     if (!check_file_exists(path))
    // C++:         throw std::runtime_error("File not found: " + path);
    // C++:     return boost::filesystem::file_size(path);
    // C++: }
    if !check_file_exists(path) {
        return Err(Error::IO(format!("File not found: {}", path.display())));
    }
    std::fs::metadata(path)
        .map(|m| m.len())
        .map_err(|e| Error::IO(format!("Failed to get file size: {}", e)))
}

/// Normalize path separators to forward slashes
/// format.hpp:75
/// C++: std::string normalize_path(const std::string& path);
pub fn normalize_path(path: &str) -> String {
    // Convert backslashes to forward slashes for cross-platform compatibility
    // format.hpp:76-79
    // C++: std::string normalize_path(const std::string& path) {
    // C++:     std::string result = path;
    // C++:     std::replace(result.begin(), result.end(), '\\', '/');
    // C++:     return result;
    // C++: }
    path.replace('\\', "/")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn test_detect_format() {
        // Test format detection from file extensions
        assert_eq!(detect_format(Path::new("model.stl")), FileFormat::STL);
        assert_eq!(detect_format(Path::new("model.STL")), FileFormat::STL);
        assert_eq!(detect_format(Path::new("model.amf")), FileFormat::AMF);
        assert_eq!(detect_format(Path::new("model.3mf")), FileFormat::ThreeMF);
        assert_eq!(detect_format(Path::new("model.obj")), FileFormat::OBJ);
        assert_eq!(detect_format(Path::new("model.step")), FileFormat::STEP);
        assert_eq!(detect_format(Path::new("model.stp")), FileFormat::STEP);
        assert_eq!(detect_format(Path::new("model.xyz")), FileFormat::Unknown);
    }

    #[test]
    fn test_is_format_supported() {
        // Test which formats are supported
        assert!(is_format_supported(FileFormat::STL));
        assert!(is_format_supported(FileFormat::AMF));
        assert!(is_format_supported(FileFormat::ThreeMF));
        assert!(is_format_supported(FileFormat::OBJ));
        assert!(!is_format_supported(FileFormat::STEP));
        assert!(!is_format_supported(FileFormat::Unknown));
    }

    #[test]
    fn test_normalize_path() {
        // Test path normalization
        assert_eq!(
            normalize_path("C:\\path\\to\\file.stl"),
            "C:/path/to/file.stl"
        );
        assert_eq!(normalize_path("/unix/path/file.stl"), "/unix/path/file.stl");
    }
}
