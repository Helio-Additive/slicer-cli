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

    /// M1 (bridges/wave_seeds): Z-preserving OPEN-PATH offset
    /// (`expolygons_to_zpaths_expanded_opened`, RegionExpansion.cpp:83-106).
    /// Offsets each contour (outer +`expansion`, holes -`expansion`), opens each
    /// offset polygon (first point repeated) and tags every vertex Z with the
    /// running per-expolygon `base_idx` (returned, advanced, via `base_idx_out`).
    pub fn cz_offset_open(
        contour_xy: *const i32,
        contour_lens: *const i32,
        contour_per_ex: *const i32,
        num_ex: i32,
        expansion: f64,
        shortest_edge_length: f64,
        base_idx_start: i32,
        base_idx_out: *mut i32,
    ) -> CzZPaths;
}

/// Mirror of `CzWaveSeeds` in `clipper_z_shim.h`. Z-tagged output segments
/// (first `num_closed` are closed, rest open) plus the provenance
/// `intersections` table (`2 * num_intersections` i32 `(first, second)` pairs).
/// A negative output Z value `-k` refers to `intersections[k-1]`.
/// Must be freed via [`cz_free_wave_seeds`].
#[repr(C)]
pub struct CzWaveSeeds {
    pub coords: *mut i32,
    pub path_lens: *mut i32,
    pub num_paths: i32,
    pub total_points: i32,
    pub num_closed: i32,
    pub intersections: *mut i32,
    pub num_intersections: i32,
}

extern "C" {
    /// M1 (bridges/wave_seeds): provenance Z-clip core (RegionExpansion.cpp:302-327,
    /// ClipperLib_Z engine). Boundary added as CLOSED clip, offset-opened src as
    /// OPEN subject; `Execute(ctIntersection, pftNonZero)` with the
    /// `ClipperZIntersectionVisitor` ZFillFunction. Returns Z-tagged closed+open
    /// segments and the populated intersections table.
    pub fn cz_wave_seeds_clip(
        subj_xyz: *const i32,
        subj_lens: *const i32,
        subj_num: i32,
        clip_xyz: *const i32,
        clip_lens: *const i32,
        clip_num: i32,
    ) -> CzWaveSeeds;

    /// Free a [`CzWaveSeeds`] returned by [`cz_wave_seeds_clip`].
    pub fn cz_free_wave_seeds(seeds: CzWaveSeeds);
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
    fn wave_seeds_primitives_produce_tagged_segments() {
        // M1 (bridges): a src square [0,100]^2 (idx 1 boundary tag start) sitting
        // inside a larger boundary square [-50,150]^2. The offset-opened src is
        // clipped against the boundary; with provenance Z tags we expect non-empty
        // Z-tagged segments and (because src is fully inside) at least one closed
        // seed whose Z points back to the src index, no boundary intersections.
        //
        // boundary tagged Z=1 (idx_boundary_begin=1); src base_idx starts at
        // idx_boundary_end=2 (one boundary expolygon).
        let boundary_xy: [i32; 8] = [-50, -50, 150, -50, 150, 150, -50, 150];
        let boundary_lens: [i32; 1] = [4];
        // Pre-tag the boundary clip Z with its index = 1.
        let mut clip_xyz: Vec<i32> = Vec::new();
        for i in 0..4 {
            clip_xyz.push(boundary_xy[2 * i]);
            clip_xyz.push(boundary_xy[2 * i + 1]);
            clip_xyz.push(1);
        }

        // (a) offset-open the src square. expansion = a few scaled units.
        let src_xy: [i32; 8] = [0, 0, 100, 0, 100, 100, 0, 100];
        let src_lens: [i32; 1] = [4];
        let per_ex: [i32; 1] = [1];
        let mut base_out: i32 = 0;
        let offset = unsafe {
            cz_offset_open(
                src_xy.as_ptr(),
                src_lens.as_ptr(),
                per_ex.as_ptr(),
                1,
                5.0,   // expansion (scaled units)
                0.0,   // shortest_edge_length
                2,     // base_idx_start = idx_boundary_end
                &mut base_out as *mut i32,
            )
        };
        let subj_paths = collect_and_free(offset);
        assert!(!subj_paths.is_empty(), "offset-open must produce >=1 opened path");
        assert_eq!(base_out, 3, "base_idx advances once per expolygon (2 -> 3)");
        // Opened paths repeat the first point at the end and carry Z = base_idx = 2.
        for p in &subj_paths {
            assert!(p.len() >= 4);
            assert_eq!(p.first().unwrap().0, p.last().unwrap().0, "opened: first.x == last.x");
            assert_eq!(p.first().unwrap().1, p.last().unwrap().1, "opened: first.y == last.y");
            for v in p {
                assert_eq!(v.2, 2, "src vertices tagged with base_idx=2: {v:?}");
            }
        }

        // Flatten the offset-opened subject back into x,y,z triples.
        let mut subj_xyz: Vec<i32> = Vec::new();
        let mut subj_lens: Vec<i32> = Vec::new();
        for p in &subj_paths {
            subj_lens.push(p.len() as i32);
            for v in p {
                subj_xyz.push(v.0);
                subj_xyz.push(v.1);
                subj_xyz.push(v.2);
            }
        }

        // (b) the provenance Z-clip.
        let raw = unsafe {
            cz_wave_seeds_clip(
                subj_xyz.as_ptr(),
                subj_lens.as_ptr(),
                subj_lens.len() as i32,
                clip_xyz.as_ptr(),
                boundary_lens.as_ptr(),
                1,
            )
        };
        assert!(raw.num_paths > 0, "wave_seeds clip must produce >=1 Z-tagged segment");
        // Collect the segments + intersections, then free.
        let num_closed = raw.num_closed;
        let mut segs: Vec<Vec<(i32, i32, i32)>> = Vec::new();
        {
            let lens =
                unsafe { std::slice::from_raw_parts(raw.path_lens, raw.num_paths as usize) };
            let coords =
                unsafe { std::slice::from_raw_parts(raw.coords, (raw.total_points * 3) as usize) };
            let mut cur = 0usize;
            for &len in lens {
                let mut path = Vec::new();
                for _ in 0..len {
                    path.push((coords[cur * 3], coords[cur * 3 + 1], coords[cur * 3 + 2]));
                    cur += 1;
                }
                segs.push(path);
            }
        }
        unsafe { cz_free_wave_seeds(raw) };

        // src fully inside boundary => no boundary crossing, so the offset-opened
        // src loop survives the intersection intact. NOTE: old ClipperLib (clipper1)
        // routes an open subject loop with no clip crossing into the OPEN-paths
        // bucket of the PolyTree (Clipper2's wave_seeds would classify it CLOSED via
        // its closed_segs output); either way the loop is preserved with its src Z
        // tag. The Rust wave_seeds port keys off the per-point Z (front==back &&
        // Z>=idx_boundary_end => closed/in-src), not the bucket, so this difference
        // is immaterial to seed classification.
        let _ = num_closed;
        let total: usize = segs.iter().map(|s| s.len()).sum();
        assert!(total >= 4, "the in-src seed loop must retain its vertices");
        // Every vertex Z is either the src tag (2) or a negative provenance index.
        for s in &segs {
            for v in s {
                assert!(v.2 == 2 || v.2 < 0, "Z is src tag or provenance idx: {v:?}");
            }
        }
    }

    #[test]
    fn wave_seeds_primitives_crossing_boundary() {
        // src square straddling the boundary edge so the offset-opened src crosses
        // the clip contour => boundary intersection points (negative Z) recorded.
        // boundary [0,100]^2 tagged Z=1; src [50,150]x[20,80] partly outside.
        let mut clip_xyz: Vec<i32> = Vec::new();
        for &(x, y) in &[(0, 0), (100, 0), (100, 100), (0, 100)] {
            clip_xyz.push(x);
            clip_xyz.push(y);
            clip_xyz.push(1);
        }
        let clip_lens: [i32; 1] = [4];

        let src_xy: [i32; 8] = [50, 20, 150, 20, 150, 80, 50, 80];
        let src_lens: [i32; 1] = [4];
        let per_ex: [i32; 1] = [1];
        let mut base_out: i32 = 0;
        let offset = unsafe {
            cz_offset_open(
                src_xy.as_ptr(),
                src_lens.as_ptr(),
                per_ex.as_ptr(),
                1,
                2.0,
                0.0,
                2,
                &mut base_out as *mut i32,
            )
        };
        let subj_paths = collect_and_free(offset);
        let mut subj_xyz: Vec<i32> = Vec::new();
        let mut subj_lens: Vec<i32> = Vec::new();
        for p in &subj_paths {
            subj_lens.push(p.len() as i32);
            for v in p {
                subj_xyz.push(v.0);
                subj_xyz.push(v.1);
                subj_xyz.push(v.2);
            }
        }

        let raw = unsafe {
            cz_wave_seeds_clip(
                subj_xyz.as_ptr(),
                subj_lens.as_ptr(),
                subj_lens.len() as i32,
                clip_xyz.as_ptr(),
                clip_lens.as_ptr(),
                1,
            )
        };
        assert!(raw.num_paths > 0, "crossing src must produce clipped segments");
        let num_intersections = raw.num_intersections;
        let intersections: Vec<(i32, i32)> = if num_intersections > 0 {
            let s = unsafe {
                std::slice::from_raw_parts(raw.intersections, (num_intersections * 2) as usize)
            };
            (0..num_intersections as usize).map(|i| (s[2 * i], s[2 * i + 1])).collect()
        } else {
            Vec::new()
        };
        unsafe { cz_free_wave_seeds(raw) };

        assert!(
            num_intersections > 0,
            "src crossing the boundary edge must record boundary intersections"
        );
        // Each recorded intersection mixes the boundary tag (1) with the src tag (2).
        for (a, b) in &intersections {
            assert!(
                (*a == 1 && *b == 2) || (*a == 2 && *b == 1),
                "intersection mixes boundary(1) and src(2): ({a},{b})"
            );
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
