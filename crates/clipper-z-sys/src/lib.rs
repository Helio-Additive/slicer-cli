//! Low-level FFI bindings to the vendored BambuStudio ClipperLib / ClipperLib_Z
//! via a C shim (see `shim/clipper_z_shim.{h,cpp}` and `vendor/`).
//!
//! ClipperLib here is built with `CLIPPERLIB_INT32`, so the engine's coordinate
//! type (`cInt`) and the Z tag are `i32`. Callers must scale libslic3r `i64`
//! coordinates into the `i32` range before calling. The safe Rust wrapper that
//! does this marshalling lives in `libslic3r-rs/src/clipper_z.rs`.
//!
//! The only runtime dependency is the C++ standard library (statically resolved
//! C++ otherwise); no system/dynamic clipper library is required.

use std::os::raw::{c_char, c_int};

/// Mirror of `CzZPaths` in `clipper_z_shim.h`. Flat layout: `coords` is
/// `3 * total_points` i32s (x,y,z triples), `path_lens` is `num_paths` i32s
/// whose sum equals `total_points`. Must be freed via [`cz_free_zpaths`].
#[repr(C)]
pub struct CzZPaths {
    pub coords: *mut i32,
    pub path_lens: *mut i32,
    pub num_paths: i32,
    pub total_points: i32,
}

extern "C" {
    /// M1 smoke: returns the static ClipperLib version C string ("6.2.6").
    pub fn cz_version() -> *const c_char;

    /// M1 smoke: union of a closed polygon (flat `xy` int32 pairs, `n` points)
    /// with itself via the non-Z ClipperLib; returns total output point count.
    pub fn cz_union_point_count(xy: *const i32, n: i32) -> i32;

    /// M2: faithful `clip_extrusion` (OverhangDetector.cpp). Open subject ZPath
    /// (`subject_xyz` = `subject_n` x,y,z triples) clipped against closed clip
    /// ZPaths (`clip_xyz` flat triples + `clip_lens` per-path lengths,
    /// `clip_num` paths). `clip_type`: 0=Intersection,1=Union,2=Difference,3=Xor.
    pub fn cz_clip_extrusion(
        subject_xyz: *const i32,
        subject_n: i32,
        clip_xyz: *const i32,
        clip_lens: *const i32,
        clip_num: i32,
        clip_type: c_int,
    ) -> CzZPaths;

    /// Free a [`CzZPaths`] returned by [`cz_clip_extrusion`].
    pub fn cz_free_zpaths(paths: CzZPaths);
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ffi::CStr;

    #[test]
    fn version_links() {
        // M1: proves the vendored C++ compiles, links, and is callable.
        let v = unsafe { CStr::from_ptr(cz_version()) };
        let s = v.to_str().unwrap();
        assert!(!s.is_empty(), "version string should be non-empty");
        assert_eq!(s, "6.2.6");
    }

    #[test]
    fn union_smoke_nonz_tu() {
        // M1: a closed unit square union with itself. The non-Z ClipperLib TU
        // must compile + link; the result should be a single 4-point square.
        let xy: [i32; 8] = [0, 0, 100, 0, 100, 100, 0, 100];
        let count = unsafe { cz_union_point_count(xy.as_ptr(), 4) };
        assert_eq!(count, 4, "union of a square with itself is a 4-point square");
    }
}
