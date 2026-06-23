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

    /// Faithful `offset_expolygon_inner` (ClipperUtils.cpp:437-506): vertex-exact
    /// offset of a SINGLE ExPolygon (`contour_xy` = `contour_n` int32 x,y pairs;
    /// holes via `holes_xy` flat x,y pairs + `hole_lens` per-hole counts +
    /// `hole_num`) by `delta` (scaled integer units). `join_type`:
    /// 0=jtMiter,1=jtRound,2=jtSquare; `miter_limit` is MiterLimit (or
    /// ArcTolerance for jtRound). Output z is always 0; NOT unioned across
    /// ExPolygons (caller unions). Free via [`cz_free_zpaths`].
    pub fn cz_offset_expolygon(
        contour_xy: *const i32,
        contour_n: i32,
        holes_xy: *const i32,
        hole_lens: *const i32,
        hole_num: i32,
        delta: f64,
        join_type: i32,
        miter_limit: f64,
    ) -> CzZPaths;

    /// Faithful closed-path boolean DIFFERENCE (subject - clip), mirroring
    /// `clipper_do<ClipperLib::Paths>(ctDifference, subject, clip, pftNonZero)`
    /// (ClipperUtils.cpp:309-322), the engine behind `diff` / `diff_ex`
    /// (ApplySafetyOffset::No). `subject_xy`/`clip_xy` are flat int32 (x,y) pairs;
    /// `subject_lens`/`clip_lens` give per-path point counts; `subject_num`/
    /// `clip_num` the path counts. Each ExPolygon contributes its contour + holes
    /// as separate closed paths in natural orientation. Output is the raw
    /// difference paths (z always 0); the caller re-unions into ExPolygons. Free
    /// via [`cz_free_zpaths`].
    pub fn cz_difference_closed(
        subject_xy: *const i32,
        subject_lens: *const i32,
        subject_num: i32,
        clip_xy: *const i32,
        clip_lens: *const i32,
        clip_num: i32,
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

    /// Read a CzZPaths back into owned Vec<(x,y,z)> paths, then free it.
    fn collect_and_free(raw: CzZPaths) -> Vec<Vec<(i32, i32, i32)>> {
        let mut out = Vec::new();
        if raw.num_paths > 0 && !raw.coords.is_null() && !raw.path_lens.is_null() {
            let lens = unsafe { std::slice::from_raw_parts(raw.path_lens, raw.num_paths as usize) };
            let coords =
                unsafe { std::slice::from_raw_parts(raw.coords, (raw.total_points * 3) as usize) };
            let mut cur = 0usize;
            for &len in lens {
                let mut path = Vec::new();
                for _ in 0..len {
                    path.push((coords[cur * 3], coords[cur * 3 + 1], coords[cur * 3 + 2]));
                    cur += 1;
                }
                out.push(path);
            }
        }
        unsafe { cz_free_zpaths(raw) };
        out
    }

    #[test]
    fn clip_extrusion_partial_overlap() {
        // M2: open horizontal subject crossing the right edge of a [0,100]^2
        // clip square (ctIntersection=0), constant width Z=40. Only the x in
        // [0,100] portion survives, and every output vertex keeps a positive Z.
        //
        // NOTE: the subject has 3 points. The faithful OverhangDetector.cpp
        // post-pass that re-derives Z for clip-boundary vertices is guarded by
        // `if (subject.size() <= 2) continue;`, so a degenerate 2-point subject
        // would (correctly, per C++) leave the clip-boundary vertex at Z=0. Real
        // callers always sample the extrusion path to >2 points; mirror that.
        let subject: [i32; 9] = [-50, 50, 40, 50, 50, 40, 150, 50, 40];
        let clip: [i32; 12] = [0, 0, 0, 100, 0, 0, 100, 100, 0, 0, 100, 0];
        let clip_lens: [i32; 1] = [4];
        let raw = unsafe {
            cz_clip_extrusion(subject.as_ptr(), 3, clip.as_ptr(), clip_lens.as_ptr(), 1, 0)
        };
        let paths = collect_and_free(raw);
        assert_eq!(paths.len(), 1, "expected one clipped open path");
        let p = &paths[0];
        assert!(p.len() >= 2);
        let min_x = p.iter().map(|v| v.0).min().unwrap();
        let max_x = p.iter().map(|v| v.0).max().unwrap();
        assert_eq!(min_x, 0, "clip starts at left boundary x=0");
        assert_eq!(max_x, 100, "clip ends at right boundary x=100");
        for v in p {
            assert_eq!(v.1, 50, "y stays on the subject line");
            assert!(v.2 > 0, "every clipped vertex carries a positive Z width: {v:?}");
            assert_eq!(v.2, 40, "constant-width subject => Z=40 at the boundary");
        }
    }

    #[test]
    fn clip_extrusion_no_overlap() {
        // Subject fully outside the clip => nothing survives.
        let subject: [i32; 6] = [500, 500, 40, 600, 500, 40];
        let clip: [i32; 12] = [0, 0, 0, 100, 0, 0, 100, 100, 0, 0, 100, 0];
        let clip_lens: [i32; 1] = [4];
        let raw = unsafe {
            cz_clip_extrusion(subject.as_ptr(), 2, clip.as_ptr(), clip_lens.as_ptr(), 1, 0)
        };
        let paths = collect_and_free(raw);
        assert!(paths.is_empty(), "disjoint subject clips to nothing");
    }

    #[test]
    fn clip_extrusion_closed_loop_fully_inside() {
        // A closed square loop (first==last) fully inside a larger clip square
        // must survive intersection intact (de-risks the perimeter-loop case
        // where the subject is a closed loop opened at its first point).
        // Loop [10,90]^2, clip [0,100]^2.
        let subject: [i32; 15] = [
            10, 10, 40, 90, 10, 40, 90, 90, 40, 10, 90, 40, 10, 10, 40,
        ];
        let clip: [i32; 12] = [0, 0, 0, 100, 0, 0, 100, 100, 0, 0, 100, 0];
        let clip_lens: [i32; 1] = [4];
        let raw = unsafe {
            cz_clip_extrusion(subject.as_ptr(), 5, clip.as_ptr(), clip_lens.as_ptr(), 1, 0)
        };
        let paths = collect_and_free(raw);
        let total: usize = paths.iter().map(|p| p.len()).sum();
        assert!(!paths.is_empty(), "closed loop inside clip must survive");
        // Every vertex of the loop should be preserved (5 input points) and Z=40.
        assert!(total >= 5, "expected the full loop (>=5 pts), got {total}");
        for p in &paths {
            for v in p {
                assert_eq!(v.2, 40, "constant-width loop keeps Z=40: {v:?}");
            }
        }
    }

    #[test]
    fn clip_extrusion_interpolates_z() {
        // Subject from x=-100 (width 20) to x=100 (width 60); clip [0,200]^2.
        // The clip boundary at x=0 is the subject midpoint => width ~40.
        let subject: [i32; 6] = [-100, 50, 20, 100, 50, 60];
        let clip: [i32; 12] = [0, 0, 0, 200, 0, 0, 200, 100, 0, 0, 100, 0];
        let clip_lens: [i32; 1] = [4];
        let raw = unsafe {
            cz_clip_extrusion(subject.as_ptr(), 2, clip.as_ptr(), clip_lens.as_ptr(), 1, 0)
        };
        let paths = collect_and_free(raw);
        assert_eq!(paths.len(), 1);
        let p = &paths[0];
        let boundary = p.iter().find(|v| v.0 == 0).expect("boundary vertex at x=0");
        assert!(
            (boundary.2 - 40).abs() <= 1,
            "width at clip boundary interpolates to ~40, got {}",
            boundary.2
        );
    }
}
