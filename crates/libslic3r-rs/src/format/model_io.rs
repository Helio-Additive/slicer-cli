//! 1:1 port of `libslic3r/Format/ModelIO.{hpp,mm}` (BambuStudio).
//!
//! C++ Reference:
//! - Format/ModelIO.hpp
//! - Format/ModelIO.mm
//!
//! NOTE ON FIDELITY / NATIVE DEPENDENCY:
//! The C++ source is an Objective-C++ file (`ModelIO.mm`) that is compiled
//! ONLY on Apple platforms — it is gated behind `if (APPLE)` in
//! `src/libslic3r/CMakeLists.txt` (which also does
//! `find_library(MODELIO ModelIO REQUIRED)`) and behind `#ifdef __APPLE__`
//! at the single call site in `Model.cpp`. `make_temp_stl_with_modelio`
//! converts USD/USDZ/ABC/PLY assets to a temporary STL using Apple's native
//! `ModelIO` framework (`MDLAsset` / `exportAssetToURL:`).
//!
//! That framework is a macOS/iOS-only system dylib. It is NOT wasm-safe and
//! the porting rules forbid adding native dylib dependencies, so the
//! `MDLAsset` import/export cannot be reproduced. We faithfully mirror the
//! C++ control flow (build the input URL, allocate the asset, build a
//! UUID-based temp filename in the temporary directory, attempt the export)
//! but the native export call is unavailable — which corresponds exactly to
//! the C++ failure branch where `[asset exportAssetToURL:]` returns NO and the
//! function returns an empty `std::string`. We do NOT fabricate an alternate
//! conversion pipeline.
//!
//! `delete_temp_file` has no native dependency and is ported in full.

use std::path::Path;

// ---------------------------------------------------------------------------
// namespace Slic3r   (ModelIO.mm:4)
// ---------------------------------------------------------------------------

/// Uses ModelIO to convert supported model types to a temporary STL
/// that can then be consumed by the existing STL loader.
///
/// `input_file` The File to load.
/// Returns the path to the temporary file, or an empty string if conversion
/// failed.
///
/// ModelIO.hpp:4-11
/// ModelIO.mm:6
pub fn make_temp_stl_with_modelio(input_file: &str) -> String {
    // ModelIO.mm:8
    //   NSURL *input_url = [NSURL fileURLWithPath:[NSString stringWithUTF8String:input_file.c_str()]];
    let _input_url = Path::new(input_file);

    // ModelIO.mm:9
    //   MDLAsset *asset = [[MDLAsset alloc] initWithURL:input_url];
    //
    // BLOCKED (native dependency): `MDLAsset` lives in Apple's `ModelIO`
    // framework. Reading the asset requires that system dylib, which is not
    // available off-Apple and is not wasm-safe, so the asset cannot be
    // constructed here.

    // ModelIO.mm:11
    //   NSString *tmp_file_name = [[[NSUUID UUID] UUIDString] stringByAppendingPathExtension:@"stl"];
    let tmp_file_name = format!("{}.stl", make_uuid_string());

    // ModelIO.mm:12
    //   NSURL *tmp_file_url = [NSURL fileURLWithPath:[NSTemporaryDirectory() stringByAppendingPathComponent:tmp_file_name]];
    let tmp_file_url = std::env::temp_dir().join(tmp_file_name);

    // ModelIO.mm:14-17
    //   if ([asset exportAssetToURL:tmp_file_url]) {
    //       std::string output_file = std::string([[tmp_file_url path] UTF8String]);
    //       return output_file;
    //   }
    //
    // BLOCKED (native dependency): `[asset exportAssetToURL:]` is the Apple
    // ModelIO exporter. Without the framework the export is unavailable, which
    // is equivalent to the export returning NO — so we fall through to the
    // failure branch below. `tmp_file_url` is computed above purely to mirror
    // the C++ structure; on a successful native export the returned path would
    // be `tmp_file_url`'s filesystem path.
    let _ = tmp_file_url;

    // ModelIO.mm:19
    //   return std::string();
    String::new()
}

/// Convenience function to delete the file.
/// No return value since success isn't required.
///
/// `temp_file` File path to delete.
///
/// ModelIO.hpp:12-17
/// ModelIO.mm:20
pub fn delete_temp_file(temp_file: &str) {
    // ModelIO.mm:22
    //   NSString *file_path = [NSString stringWithUTF8String:temp_file.c_str()];
    let file_path = Path::new(temp_file);

    // ModelIO.mm:23
    //   [[NSFileManager defaultManager] removeItemAtPath:file_path error:NULL];
    // `error:NULL` => failure is ignored.
    let _ = std::fs::remove_file(file_path);
}

// ---------------------------------------------------------------------------
// Helper: emulate `[[NSUUID UUID] UUIDString]` (ModelIO.mm:11)
// ---------------------------------------------------------------------------
//
// `NSUUID` produces an RFC-4122 version-4 (random) UUID rendered as an
// upper-case `8-4-4-4-12` hexadecimal string. We build the same shape from a
// fresh source of entropy without pulling in a native dependency. The exact
// random value is non-deterministic (so is `NSUUID`), which is irrelevant to
// G-code parity: this code path is Apple-only and is unreachable here because
// the asset import/export above is blocked.
fn make_uuid_string() -> String {
    // Gather 16 bytes of entropy from the system temp-name source available to
    // std without any extra crates.
    let mut bytes = [0u8; 16];
    seed_random_bytes(&mut bytes);

    // RFC 4122 version 4 + variant bits, matching NSUUID's output.
    bytes[6] = (bytes[6] & 0x0F) | 0x40; // version 4
    bytes[8] = (bytes[8] & 0x3F) | 0x80; // variant 1

    let hex = |b: u8| format!("{:02X}", b);
    format!(
        "{}{}{}{}-{}{}-{}{}-{}{}-{}{}{}{}{}{}",
        hex(bytes[0]),
        hex(bytes[1]),
        hex(bytes[2]),
        hex(bytes[3]),
        hex(bytes[4]),
        hex(bytes[5]),
        hex(bytes[6]),
        hex(bytes[7]),
        hex(bytes[8]),
        hex(bytes[9]),
        hex(bytes[10]),
        hex(bytes[11]),
        hex(bytes[12]),
        hex(bytes[13]),
        hex(bytes[14]),
        hex(bytes[15]),
    )
}

// Fill `buf` with pseudo-random bytes derived from high-resolution time and
// the process/thread state. Dependency-free and wasm-safe; uniqueness is all
// that is required for a temp filename.
fn seed_random_bytes(buf: &mut [u8]) {
    use std::time::{SystemTime, UNIX_EPOCH};

    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);

    // splitmix64-style mixing seeded from the timestamp, advanced per byte.
    let mut state = nanos as u64 ^ 0x9E37_79B9_7F4A_7C15;
    for chunk in buf.iter_mut() {
        state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        *chunk = (z & 0xFF) as u8;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_delete_temp_file_nonexistent() {
        // ModelIO.mm:20 — failure is ignored; must not panic.
        delete_temp_file("/tmp/nonexistent_file_12345.stl");
    }

    #[test]
    fn test_make_temp_stl_returns_empty_when_native_modelio_unavailable() {
        // The Apple ModelIO framework is unavailable in this build, so the
        // conversion always reports failure via an empty string — matching the
        // C++ `return std::string();` failure branch (ModelIO.mm:19).
        let result = make_temp_stl_with_modelio("model.usdz");
        assert!(result.is_empty());
    }

    #[test]
    fn test_uuid_string_shape() {
        let u = make_uuid_string();
        assert_eq!(u.len(), 36);
        let parts: Vec<&str> = u.split('-').collect();
        assert_eq!(parts.len(), 5);
        assert_eq!(parts[0].len(), 8);
        assert_eq!(parts[1].len(), 4);
        assert_eq!(parts[2].len(), 4);
        assert_eq!(parts[3].len(), 4);
        assert_eq!(parts[4].len(), 12);
        // version-4 nibble.
        assert!(parts[2].starts_with('4'));
    }
}
