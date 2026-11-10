//! Raw FFI bindings to the Clipper2-Z C-ABI shim (`clipper2_z_shim.h`).
//!
//! The shim wraps the vendored Clipper2 v1.5.4 built with `-DUSINGZ`, namespace-
//! isolated as `Clipper2ZSys` so it never collides with `clipper2c-sys`'s non-Z
//! `Clipper2Lib` (ODR safety; see `build.rs`).
//!
//! This exposes exactly the two Clipper2-Z ops BambuStudio's RegionExpansion.cpp
//! `wave_seeds` needs: a Z-carrying `ClipperOffset` and a Z-callback `Clipper64`
//! open-subject intersection that returns closed+open Z-segments plus the
//! recorded intersections table. Coordinates are `i64` (Clipper2 is 64-bit), so
//! libslic3r coords pass straight through with no narrowing.

use std::os::raw::c_char;

/// Flat Z-paths: `coords` = `total_points` (x,y,z) i64 triples; `path_lens` =
/// `num_paths` lengths summing to `total_points`. Freed by [`cz2_free_zpaths`].
#[repr(C)]
pub struct Cz2ZPaths {
    pub coords: *mut i64,
    pub path_lens: *mut i32,
    pub num_paths: i32,
    pub total_points: i32,
}

/// Result of [`cz2_intersect_open_z`]: the closed+open Z-segments (closed first)
/// and the intersections table. Freed by [`cz2_free_wave_clip`].
#[repr(C)]
pub struct Cz2WaveClip {
    pub segs: Cz2ZPaths,
    pub num_closed: i32,
    pub is_a: *mut i64,
    pub is_b: *mut i64,
    pub num_is: i32,
}

extern "C" {
    /// Vendored Clipper2 version ("1.5.4"); proves the USINGZ TU links.
    pub fn cz2_version() -> *const c_char;

    /// ClipperOffset (JoinType::Square, EndType::Polygon) of a single closed
    /// contour (`contour_xyz` = `n` x,y,z i64 triples) by `delta`, Z preserved.
    pub fn cz2_offset_z(contour_xyz: *const i64, n: i32, delta: f64) -> Cz2ZPaths;

    /// Clipper64 + SetZCallback (Clipper2ZIntersectionVisitor) +
    /// AddClip(closed) + AddOpenSubject(open) + Execute(Intersection, NonZero)
    /// returning closed+open Z-segments and the intersections table.
    pub fn cz2_intersect_open_z(
        src_xyz: *const i64,
        src_lens: *const i32,
        src_num: i32,
        clip_xyz: *const i64,
        clip_lens: *const i32,
        clip_num: i32,
    ) -> Cz2WaveClip;
    pub fn cz2_pl_open(
        clip_type: i32,
        src_xyz: *const i64,
        src_lens: *const i32,
        src_num: i32,
        clip_xyz: *const i64,
        clip_lens: *const i32,
        clip_num: i32,
    ) -> Cz2ZPaths;


    pub fn cz2_free_zpaths(paths: Cz2ZPaths);
    pub fn cz2_free_wave_clip(wc: Cz2WaveClip);
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ffi::CStr;

    #[test]
    fn version_links() {
        // Proves the vendored Clipper2 compiles with USINGZ, links, is callable.
        let v = unsafe { CStr::from_ptr(cz2_version()) };
        assert_eq!(v.to_str().unwrap(), "1.5.4");
    }

    #[test]
    fn intersect_open_z_tags_intersection() {
        // Boundary: a 0..100 square (closed clip), all vertices z = 1 (one source).
        let clip_xyz: [i64; 12] = [0, 0, 1, 100, 0, 1, 100, 100, 1, 0, 100, 1];
        let clip_lens: [i32; 1] = [4];
        // Open subject: a line crossing the left boundary edge. ALL subject vertices
        // share the same source z (z=7) — this mirrors wave_seeds, where each open
        // subject is a single offset contour whose vertices all carry one base_idx.
        // So a src x clip intersection has exactly 2 unique source z's {1, 7} and the
        // visitor records it (the differing-z-per-endpoint case never occurs in real
        // input — the C++ visitor asserts unique==1||2).
        let src_xyz: [i64; 6] = [-10, 50, 7, 50, 50, 7];
        let src_lens: [i32; 1] = [2];

        let wc = unsafe {
            cz2_intersect_open_z(
                src_xyz.as_ptr(),
                src_lens.as_ptr(),
                1,
                clip_xyz.as_ptr(),
                clip_lens.as_ptr(),
                1,
            )
        };
        // Expect one open segment from the boundary crossing (0,50) to (50,50).
        assert!(wc.segs.num_paths >= 1, "expected at least one clipped segment");
        // The boundary-crossing intersection (src z=7 x clip z=1) must record a table
        // entry (1, 7) and tag the crossing point with negative z (= -1).
        assert!(wc.num_is >= 1, "expected a recorded intersection (src x clip)");
        unsafe {
            assert_eq!(*wc.is_a, 1, "intersection lower z");
            assert_eq!(*wc.is_b, 7, "intersection upper z");
        }
        unsafe { cz2_free_wave_clip(wc) };
    }

    #[test]
    fn offset_z_preserves_z() {
        // A 0..100 square, z=5; offset out by 10 should keep z=5 on outputs.
        let contour: [i64; 12] = [0, 0, 5, 100, 0, 5, 100, 100, 5, 0, 100, 5];
        let res = unsafe { cz2_offset_z(contour.as_ptr(), 4, 10.0) };
        assert!(res.num_paths >= 1);
        // Spot-check the first output vertex's z.
        if res.total_points > 0 {
            let z0 = unsafe { *res.coords.offset(2) };
            assert_eq!(z0, 5, "offset must preserve the source z tag");
        }
        unsafe { cz2_free_zpaths(res) };
    }
}
