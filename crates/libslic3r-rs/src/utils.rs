//! Utility functions and error codes
//!
//! Provides common utility functions, CLI error codes, and system utilities
//! matching BambuStudio's Utils.hpp/cpp functionality.
//!
//! C++ Reference: Utils.hpp, Utils.cpp

use std::path::{Path, PathBuf};

// ============================================================================
// CLI Error Codes
// ============================================================================

/// CLI operation completed successfully
/// Utils.hpp:20
pub const CLI_SUCCESS: i32 = 0;

/// Environment setup error
/// Utils.hpp:21
pub const CLI_ENVIRONMENT_ERROR: i32 = -1;

/// Invalid command-line parameters
/// Utils.hpp:22
pub const CLI_INVALID_PARAMS: i32 = -2;

/// File not found error
/// Utils.hpp:23
pub const CLI_FILE_NOTFOUND: i32 = -3;

/// File list has invalid order
/// Utils.hpp:24
pub const CLI_FILELIST_INVALID_ORDER: i32 = -4;

/// Configuration file error
/// Utils.hpp:25
pub const CLI_CONFIG_FILE_ERROR: i32 = -5;

/// Data file error
/// Utils.hpp:26
pub const CLI_DATA_FILE_ERROR: i32 = -6;

/// Invalid printer technology specified
/// Utils.hpp:27
pub const CLI_INVALID_PRINTER_TECH: i32 = -7;

/// Unsupported operation requested
/// Utils.hpp:28
pub const CLI_UNSUPPORTED_OPERATION: i32 = -8;

/// Error copying objects
/// Utils.hpp:30
pub const CLI_COPY_OBJECTS_ERROR: i32 = -9;

/// Error scaling to fit
/// Utils.hpp:31
pub const CLI_SCALE_TO_FIT_ERROR: i32 = -10;

/// Error exporting STL
/// Utils.hpp:32
pub const CLI_EXPORT_STL_ERROR: i32 = -11;

/// Error exporting OBJ
/// Utils.hpp:33
pub const CLI_EXPORT_OBJ_ERROR: i32 = -12;

/// Error exporting 3MF
/// Utils.hpp:34
pub const CLI_EXPORT_3MF_ERROR: i32 = -13;

/// Out of memory
/// Utils.hpp:35
pub const CLI_OUT_OF_MEMORY: i32 = -14;

/// 3MF does not support machine change
/// Utils.hpp:36
pub const CLI_3MF_NOT_SUPPORT_MACHINE_CHANGE: i32 = -15;

/// New machine in 3MF not supported
/// Utils.hpp:37
pub const CLI_3MF_NEW_MACHINE_NOT_SUPPORTED: i32 = -16;

/// Process not compatible
/// Utils.hpp:38
pub const CLI_PROCESS_NOT_COMPATIBLE: i32 = -17;

/// Invalid values in 3MF
/// Utils.hpp:39
pub const CLI_INVALID_VALUES_IN_3MF: i32 = -18;

/// Post-process not supported
/// Utils.hpp:40
pub const CLI_POSTPROCESS_NOT_SUPPORTED: i32 = -19;

/// Printable size reduced
/// Utils.hpp:41
pub const CLI_PRINTABLE_SIZE_REDUCED: i32 = -20;

/// Object arrangement failed
/// Utils.hpp:42
pub const CLI_OBJECT_ARRANGE_FAILED: i32 = -21;

/// Object orientation failed
/// Utils.hpp:43
pub const CLI_OBJECT_ORIENT_FAILED: i32 = -22;

/// Modified parameters to match printer
/// Utils.hpp:44
pub const CLI_MODIFIED_PARAMS_TO_PRINTER: i32 = -23;

/// File version not supported
/// Utils.hpp:45
pub const CLI_FILE_VERSION_NOT_SUPPORTED: i32 = -24;

/// No suitable objects found
/// Utils.hpp:48
pub const CLI_NO_SUITABLE_OBJECTS: i32 = -50;

/// Validation error
/// Utils.hpp:49
pub const CLI_VALIDATE_ERROR: i32 = -51;

/// Objects partly inside build volume
/// Utils.hpp:50
pub const CLI_OBJECTS_PARTLY_INSIDE: i32 = -52;

/// Failed to create export cache directory
/// Utils.hpp:51
pub const CLI_EXPORT_CACHE_DIRECTORY_CREATE_FAILED: i32 = -53;

/// Export cache write failed
/// Utils.hpp:52
pub const CLI_EXPORT_CACHE_WRITE_FAILED: i32 = -54;

/// Import cache not found
/// Utils.hpp:53
pub const CLI_IMPORT_CACHE_NOT_FOUND: i32 = -55;

/// Import cache data cannot be used
/// Utils.hpp:54
pub const CLI_IMPORT_CACHE_DATA_CAN_NOT_USE: i32 = -56;

/// Import cache load failed
/// Utils.hpp:55
pub const CLI_IMPORT_CACHE_LOAD_FAILED: i32 = -57;

/// Slicing time exceeds limit
/// Utils.hpp:56
pub const CLI_SLICING_TIME_EXCEEDS_LIMIT: i32 = -58;

/// Triangle count exceeds limit
/// Utils.hpp:57
pub const CLI_TRIANGLE_COUNT_EXCEEDS_LIMIT: i32 = -59;

/// No suitable objects after skip
/// Utils.hpp:58
pub const CLI_NO_SUITABLE_OBJECTS_AFTER_SKIP: i32 = -60;

/// Filament does not match bed type
/// Utils.hpp:59
pub const CLI_FILAMENT_NOT_MATCH_BED_TYPE: i32 = -61;

/// Filaments have different temperatures
/// Utils.hpp:60
pub const CLI_FILAMENTS_DIFFERENT_TEMP: i32 = -62;

/// Object collision in sequential print
/// Utils.hpp:61
pub const CLI_OBJECT_COLLISION_IN_SEQ_PRINT: i32 = -63;

/// Object collision in layer print
/// Utils.hpp:62
pub const CLI_OBJECT_COLLISION_IN_LAYER_PRINT: i32 = -64;

/// Spiral mode has invalid parameters
/// Utils.hpp:63
pub const CLI_SPIRAL_MODE_INVALID_PARAMS: i32 = -65;

/// Filament cannot be mapped
/// Utils.hpp:64
pub const CLI_FILAMENT_CAN_NOT_MAP: i32 = -66;

/// Only one TPU filament supported
/// Utils.hpp:65
pub const CLI_ONLY_ONE_TPU_SUPPORTED: i32 = -67;

/// Filaments not supported by extruder
/// Utils.hpp:66
pub const CLI_FILAMENTS_NOT_SUPPORTED_BY_EXTRUDER: i32 = -68;

/// General slicing error
/// Utils.hpp:68
pub const CLI_SLICING_ERROR: i32 = -100;

/// G-code path conflicts detected
/// Utils.hpp:69
pub const CLI_GCODE_PATH_CONFLICTS: i32 = -101;

/// G-code path in unprintable area
/// Utils.hpp:70
pub const CLI_GCODE_PATH_IN_UNPRINTABLE_AREA: i32 = -102;

/// Filament unprintable on first layer
/// Utils.hpp:71
pub const CLI_FILAMENT_UNPRINTABLE_ON_FIRST_LAYER: i32 = -103;

/// G-code path outside build volume
/// Utils.hpp:72
pub const CLI_GCODE_PATH_OUTSIDE: i32 = -104;

/// G-code in wrapping detect area
/// Utils.hpp:73
pub const CLI_GCODE_IN_WRAPPING_DETECT_AREA: i32 = -105;

// ============================================================================
// Utility Functions
// ============================================================================

/// Variable/resource directory path (set by GUI or CLI)
/// Utils.hpp:93-96
static mut VAR_DIR: Option<PathBuf> = None;

/// Set the path with GUI/CLI resource files
/// Utils.hpp:93
pub fn set_var_dir(path: impl AsRef<Path>) {
    unsafe {
        VAR_DIR = Some(path.as_ref().to_path_buf());
    }
}

/// Return the full path to resource files
/// Utils.hpp:95
pub fn var_dir() -> PathBuf {
    unsafe { VAR_DIR.clone().unwrap_or_else(|| PathBuf::from(".")) }
}

/// Return a full resource path for a file name
/// Utils.hpp:97
pub fn var_path(file_name: &str) -> PathBuf {
    var_dir().join(file_name)
}

/// Resources directory path (set by GUI or CLI)
/// Utils.hpp:107-109
static mut RESOURCES_DIR: Option<PathBuf> = None;

/// Set the path with various resources
/// Utils.hpp:107
pub fn set_resources_dir(path: impl AsRef<Path>) {
    unsafe {
        RESOURCES_DIR = Some(path.as_ref().to_path_buf());
    }
}

/// Return the full path to the resources directory
/// Utils.hpp:109
pub fn resources_dir() -> PathBuf {
    unsafe { RESOURCES_DIR.clone().unwrap_or_else(|| PathBuf::from(".")) }
}

/// Format memory size in MB with comma separators
/// Utils.hpp:86
pub fn format_memsize_mb(bytes: usize) -> String {
    let mb = bytes / (1024 * 1024);
    format_with_commas(mb)
}

/// Format number with comma thousands separators
/// Utils.hpp:86 (helper)
pub fn format_with_commas(n: usize) -> String {
    let s = n.to_string();
    let mut result = String::new();
    let chars: Vec<char> = s.chars().collect();

    for (i, ch) in chars.iter().enumerate() {
        if i > 0 && (chars.len() - i) % 3 == 0 {
            result.push(',');
        }
        result.push(*ch);
    }

    result
}

/// Get total physical memory (RAM) in bytes
/// Utils.hpp:92
pub fn total_physical_memory() -> usize {
    #[cfg(target_os = "linux")]
    {
        use std::fs;
        if let Ok(contents) = fs::read_to_string("/proc/meminfo") {
            for line in contents.lines() {
                if line.starts_with("MemTotal:") {
                    let parts: Vec<&str> = line.split_whitespace().collect();
                    if parts.len() >= 2 {
                        if let Ok(kb) = parts[1].parse::<usize>() {
                            return kb * 1024; // Convert KB to bytes
                        }
                    }
                }
            }
        }
    }

    #[cfg(target_os = "macos")]
    {
        use std::process::Command;
        if let Ok(output) = Command::new("sysctl").arg("-n").arg("hw.memsize").output() {
            if let Ok(s) = String::from_utf8(output.stdout) {
                if let Ok(bytes) = s.trim().parse::<usize>() {
                    return bytes;
                }
            }
        }
    }

    #[cfg(target_os = "windows")]
    {
        // Windows: use GlobalMemoryStatusEx via winapi
        // For now, return a reasonable default
        return 8 * 1024 * 1024 * 1024; // 8GB default
    }

    // Default fallback: assume 8GB
    8 * 1024 * 1024 * 1024
}

/// Calculate next highest power of 2
/// Utils.hpp:16 (referenced for AABB tree)
#[inline]
pub fn next_highest_power_of_2(mut v: usize) -> usize {
    if v == 0 {
        return 1;
    }
    v -= 1;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v |= v >> 32;
    v + 1
}

/// Convert CLI error code to human-readable string
/// Utils.hpp (error handling utility)
pub fn cli_error_string(code: i32) -> &'static str {
    match code {
        CLI_SUCCESS => "Success",
        CLI_ENVIRONMENT_ERROR => "Environment error",
        CLI_INVALID_PARAMS => "Invalid parameters",
        CLI_FILE_NOTFOUND => "File not found",
        CLI_FILELIST_INVALID_ORDER => "File list has invalid order",
        CLI_CONFIG_FILE_ERROR => "Configuration file error",
        CLI_DATA_FILE_ERROR => "Data file error",
        CLI_INVALID_PRINTER_TECH => "Invalid printer technology",
        CLI_UNSUPPORTED_OPERATION => "Unsupported operation",
        CLI_COPY_OBJECTS_ERROR => "Error copying objects",
        CLI_SCALE_TO_FIT_ERROR => "Error scaling to fit",
        CLI_EXPORT_STL_ERROR => "Error exporting STL",
        CLI_EXPORT_OBJ_ERROR => "Error exporting OBJ",
        CLI_EXPORT_3MF_ERROR => "Error exporting 3MF",
        CLI_OUT_OF_MEMORY => "Out of memory",
        CLI_NO_SUITABLE_OBJECTS => "No suitable objects",
        CLI_VALIDATE_ERROR => "Validation error",
        CLI_SLICING_ERROR => "Slicing error",
        CLI_GCODE_PATH_CONFLICTS => "G-code path conflicts",
        CLI_GCODE_PATH_IN_UNPRINTABLE_AREA => "G-code path in unprintable area",
        _ => "Unknown error",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cli_error_codes() {
        assert_eq!(CLI_SUCCESS, 0);
        assert_eq!(CLI_ENVIRONMENT_ERROR, -1);
        assert_eq!(CLI_SLICING_ERROR, -100);
    }

    #[test]
    fn test_format_with_commas() {
        assert_eq!(format_with_commas(1000), "1,000");
        assert_eq!(format_with_commas(1000000), "1,000,000");
        assert_eq!(format_with_commas(123), "123");
    }

    #[test]
    fn test_format_memsize_mb() {
        assert_eq!(format_memsize_mb(1024 * 1024), "1");
        assert_eq!(format_memsize_mb(1024 * 1024 * 1024), "1,024");
    }

    #[test]
    fn test_next_highest_power_of_2() {
        assert_eq!(next_highest_power_of_2(0), 1);
        assert_eq!(next_highest_power_of_2(1), 1);
        assert_eq!(next_highest_power_of_2(2), 2);
        assert_eq!(next_highest_power_of_2(3), 4);
        assert_eq!(next_highest_power_of_2(7), 8);
        assert_eq!(next_highest_power_of_2(9), 16);
    }

    #[test]
    fn test_var_dir() {
        set_var_dir("/tmp/resources");
        assert_eq!(var_dir(), PathBuf::from("/tmp/resources"));

        let path = var_path("config.json");
        assert_eq!(path, PathBuf::from("/tmp/resources/config.json"));
    }

    #[test]
    fn test_cli_error_string() {
        assert_eq!(cli_error_string(CLI_SUCCESS), "Success");
        assert_eq!(cli_error_string(CLI_FILE_NOTFOUND), "File not found");
        assert_eq!(cli_error_string(-999), "Unknown error");
    }

    #[test]
    fn test_total_physical_memory() {
        let mem = total_physical_memory();
        // Should return a reasonable value (at least 1GB)
        assert!(mem >= 1024 * 1024 * 1024);
    }
}
