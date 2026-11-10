//! macOS-specific utility functions.
//!
//! 1:1 port of `MacUtils.mm` / `MacUtils.hpp` (BambuStudio).
//!
//! The C++ source is an Objective-C++ (`.mm`) translation unit that is only
//! ever compiled into the macOS build. Its two functions use the Clang
//! `@available(macOS X.0, *)` runtime check, which evaluates to `true` iff the
//! process is *running* on a macOS whose version is `>= X.0` (the `*` clause
//! means "available on any other platform"). Because the whole unit is
//! macOS-only, the faithful translation gates every body behind
//! `#[cfg(target_os = "macos")]`; on every other platform (including wasm) the
//! check is statically false, exactly as the unit would never compile / link
//! there.
//!
//! Comparing only the parsed major version against the threshold is equivalent
//! to the `@available(macOS X.0, *)` minor-version-zero comparison.

// MacUtils.hpp:1  #ifndef __MAC_UTILS_H
// MacUtils.hpp:4  namespace Slic3r {

// MacUtils.mm:6
// bool is_macos_support_boost_add_file_log()
#[cfg(target_os = "macos")]
pub fn is_macos_support_boost_add_file_log() -> bool {
    // MacUtils.mm:8  if (@available(macOS 12.0, *)) {
    if macos_running_version_at_least(12) {
        // MacUtils.mm:9
        true
    } else {
        // MacUtils.mm:11
        false
    }
}

// MacUtils.mm:6  (non-macOS: `@available(macOS 12.0, *)` is never reached;
// the unit does not compile off macOS, so the answer is false.)
#[cfg(not(target_os = "macos"))]
pub fn is_macos_support_boost_add_file_log() -> bool {
    false
}

// MacUtils.mm:15
// int is_mac_version_15()
#[cfg(target_os = "macos")]
pub fn is_mac_version_15() -> i32 {
    // MacUtils.mm:17  if (@available(macOS 15.0, *)) {//This code runs on macOS 15 or later.
    if macos_running_version_at_least(15) {
        // MacUtils.mm:18  return true;
        1
    } else {
        // MacUtils.mm:20  return false;
        0
    }
}

// MacUtils.mm:15  (non-macOS: see note above; false -> 0.)
#[cfg(not(target_os = "macos"))]
pub fn is_mac_version_15() -> i32 {
    0
}

/// Runtime equivalent of `@available(macOS <major>.0, *)`: returns `true` iff
/// the process is currently running on macOS whose major version is
/// `>= major`. Reads `sw_vers -productVersion` (e.g. `"15.3.1"`) and compares
/// the leading major component, matching the existing system-query precedent
/// in `utils.rs` (`sysctl hw.memsize`). Any failure to obtain/parse the version
/// is treated as "not available", mirroring the safe default.
#[cfg(target_os = "macos")]
fn macos_running_version_at_least(major: u32) -> bool {
    use std::process::Command;

    if let Ok(output) = Command::new("sw_vers").arg("-productVersion").output() {
        if let Ok(version_str) = String::from_utf8(output.stdout) {
            if let Some(running_major) = version_str
                .trim()
                .split('.')
                .next()
                .and_then(|s| s.parse::<u32>().ok())
            {
                return running_major >= major;
            }
        }
    }

    false
}

// MacUtils.mm:23  }; // namespace Slic3r
// MacUtils.hpp:11 #endif

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_boost_log_support_returns_bool() {
        // Should return without panicking.
        let result = is_macos_support_boost_add_file_log();

        #[cfg(target_os = "macos")]
        {
            // On macOS, could be true or false depending on version.
            assert!(result == true || result == false);
        }

        #[cfg(not(target_os = "macos"))]
        {
            // Non-macOS should always be false.
            assert_eq!(result, false);
        }
    }

    #[test]
    fn test_mac_version_15_returns_int() {
        // Should return without panicking.
        let result = is_mac_version_15();

        #[cfg(target_os = "macos")]
        {
            // On macOS, should return 0 or 1.
            assert!(result == 0 || result == 1);
        }

        #[cfg(not(target_os = "macos"))]
        {
            // Non-macOS should always be 0.
            assert_eq!(result, 0);
        }
    }

    #[test]
    fn test_version_consistency() {
        // If running macOS 15+, both functions should reflect that.
        let is_v15 = is_mac_version_15();
        let _boost_support = is_macos_support_boost_add_file_log();

        #[cfg(target_os = "macos")]
        {
            // If we're on macOS 15+, we must also support Boost logging (12+).
            if is_v15 == 1 {
                assert_eq!(
                    _boost_support, true,
                    "macOS 15+ must support Boost logging"
                );
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
