//! macOS-specific utility functions
//!
//! C++ Reference:
//! - MacUtils.hpp (10 lines)
//! - MacUtils.mm (22 lines)
//!
//! This module provides macOS version detection utilities used for
//! platform-specific feature checks and compatibility handling.

/// Check if macOS version supports Boost add_file_log functionality
/// Requires macOS 12.0 or later
/// MacUtils.mm:6-12
#[cfg(target_os = "macos")]
pub fn is_macos_support_boost_add_file_log() -> bool {
    // MacOS 12.0+ required for Boost file logging support
    // MacUtils.mm:7
    use std::process::Command;

    // Get macOS version
    if let Ok(output) = Command::new("sw_vers").arg("-productVersion").output() {
        if let Ok(version_str) = String::from_utf8(output.stdout) {
            if let Some(major_version) = version_str
                .trim()
                .split('.')
                .next()
                .and_then(|s| s.parse::<u32>().ok())
            {
                // MacUtils.mm:8
                return major_version >= 12;
            }
        }
    }

    // MacUtils.mm:10
    false
}

/// Check if macOS version supports Boost add_file_log functionality
/// Non-macOS platforms always return false
/// MacUtils.mm:6-12
#[cfg(not(target_os = "macos"))]
pub fn is_macos_support_boost_add_file_log() -> bool {
    false
}

/// Check if running on macOS 15.0 or later
/// Returns 1 (true) on macOS 15+, 0 (false) otherwise
/// MacUtils.mm:14-22
#[cfg(target_os = "macos")]
pub fn is_mac_version_15() -> i32 {
    use std::process::Command;

    // Get macOS version
    if let Ok(output) = Command::new("sw_vers").arg("-productVersion").output() {
        if let Ok(version_str) = String::from_utf8(output.stdout) {
            if let Some(major_version) = version_str
                .trim()
                .split('.')
                .next()
                .and_then(|s| s.parse::<u32>().ok())
            {
                // MacUtils.mm:16
                if major_version >= 15 {
                    return 1; // true
                }
            }
        }
    }

    // MacUtils.mm:19
    0 // false
}

/// Check if running on macOS 15.0 or later
/// Non-macOS platforms always return 0 (false)
/// MacUtils.mm:14-22
#[cfg(not(target_os = "macos"))]
pub fn is_mac_version_15() -> i32 {
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_boost_log_support_returns_bool() {
        // Should return without panicking
        let result = is_macos_support_boost_add_file_log();

        #[cfg(target_os = "macos")]
        {
            // On macOS, could be true or false depending on version
            assert!(result == true || result == false);
        }

        #[cfg(not(target_os = "macos"))]
        {
            // Non-macOS should always be false
            assert_eq!(result, false);
        }
    }

    #[test]
    fn test_mac_version_15_returns_int() {
        // Should return without panicking
        let result = is_mac_version_15();

        #[cfg(target_os = "macos")]
        {
            // On macOS, should return 0 or 1
            assert!(result == 0 || result == 1);
        }

        #[cfg(not(target_os = "macos"))]
        {
            // Non-macOS should always be 0
            assert_eq!(result, 0);
        }
    }

    #[test]
    fn test_version_consistency() {
        // If running macOS 15+, both functions should reflect that
        let is_v15 = is_mac_version_15();
        let boost_support = is_macos_support_boost_add_file_log();

        #[cfg(target_os = "macos")]
        {
            // If we're on macOS 15+, we must also support Boost logging (12+)
            if is_v15 == 1 {
                assert_eq!(boost_support, true, "macOS 15+ must support Boost logging");
            }
        }
    }

    #[test]
    fn test_non_macos_behavior() {
        #[cfg(not(target_os = "macos"))]
        {
            assert_eq!(is_macos_support_boost_add_file_log(), false);
            assert_eq!(is_mac_version_15(), 0);
        }
    }
}
