//! Safe Rust wrapper over the vendored `ClipperLib_Z` `clip_extrusion`
//! primitive (`clipper-z-sys`), the faithful Z-tagged open-path clip from
//! BambuStudio's `OverhangDetector.cpp:18-108`.
//!
//! This is the primitive the crate's Clipper2 backend cannot express: a boolean
//! clip of an *open* subject path against *closed* clip paths, where each output
//! vertex carries an interpolated Z value (here used to track per-point
//! extrusion width). It replaces the midpoint-band approximation previously used
//! in overhang grading.
//!
//! ## Coordinate scaling
//!
//! The vendored ClipperLib is built with `CLIPPERLIB_INT32`, so its coordinate
//! type and Z tag are `i32`. libslic3r coordinates are `i64` (mm * 1e5). For the
//! print volumes this slicer targets (bed ≤ ~256 mm → scaled ≤ 2.56e7, and
//! extrusion-width Z ≤ scale_(nozzle) ≈ 4e4) every value fits comfortably in
//! `i32` (max ≈ 2.1e9). The marshalling narrows `i64 → i32` on the way in and
//! widens `i32 → i64` on the way out; in debug builds it asserts the inputs are
//! within `i32` range so any out-of-range geometry is caught immediately.

use crate::clipper_z_utils::{ZPath, ZPaths, ZPoint};

/// `ClipperLib_Z::ClipType` (clipper.hpp:72). Discriminants match the C++ enum
/// exactly so they can be passed straight through the FFI.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum ClipType {
    /// `ctIntersection` — the case used by overhang grading.
    Intersection = 0,
    Union = 1,
    Difference = 2,
    Xor = 3,
}

#[inline]
fn narrow_i32(v: i64) -> i32 {
    debug_assert!(
        v >= i32::MIN as i64 && v <= i32::MAX as i64,
        "clipper_z: coordinate {v} out of i32 range (ClipperLib is built with CLIPPERLIB_INT32)"
    );
    v as i32
}

/// Faithful `clip_extrusion(const ZPath& subject, const ZPaths& clip, ClipType)`
/// (OverhangDetector.cpp:18). `subject` is an OPEN path whose Z holds extrusion
/// width; `clip` are CLOSED clip paths. Returns the clipped open paths with
/// interpolated Z (width) tags. Empty `subject` yields an empty result.
pub fn clip_extrusion(subject: &ZPath, clip: &ZPaths, clip_type: ClipType) -> ZPaths {
    if subject.is_empty() {
        return Vec::new();
    }

    // Flatten the subject into x,y,z i32 triples.
    let mut subject_flat: Vec<i32> = Vec::with_capacity(subject.len() * 3);
    for &(x, y, z) in subject {
        subject_flat.push(narrow_i32(x));
        subject_flat.push(narrow_i32(y));
        subject_flat.push(narrow_i32(z));
    }

    // Flatten the clip paths + per-path lengths.
    let mut clip_flat: Vec<i32> = Vec::new();
    let mut clip_lens: Vec<i32> = Vec::with_capacity(clip.len());
    for path in clip {
        clip_lens.push(path.len() as i32);
        clip_flat.reserve(path.len() * 3);
        for &(x, y, z) in path {
            clip_flat.push(narrow_i32(x));
            clip_flat.push(narrow_i32(y));
            clip_flat.push(narrow_i32(z));
        }
    }

    // SAFETY: pointers reference live, correctly-sized Vecs for the duration of
    // the call; the shim only reads them. The returned CzZPaths owns malloc'd
    // buffers that we copy out and then free via cz_free_zpaths.
    let raw = unsafe {
        clipper_z_sys::cz_clip_extrusion(
            subject_flat.as_ptr(),
            subject.len() as i32,
            if clip_flat.is_empty() {
                std::ptr::null()
            } else {
                clip_flat.as_ptr()
            },
            if clip_lens.is_empty() {
                std::ptr::null()
            } else {
                clip_lens.as_ptr()
            },
            clip.len() as i32,
            clip_type as i32,
        )
    };

    let mut out: ZPaths = Vec::with_capacity(raw.num_paths.max(0) as usize);
    if raw.num_paths > 0 && !raw.coords.is_null() && !raw.path_lens.is_null() {
        // SAFETY: the shim guarantees path_lens has num_paths entries and coords
        // has 3*total_points i32s, with sum(path_lens) == total_points.
        let path_lens =
            unsafe { std::slice::from_raw_parts(raw.path_lens, raw.num_paths as usize) };
        let coords =
            unsafe { std::slice::from_raw_parts(raw.coords, (raw.total_points * 3) as usize) };

        let mut cursor = 0usize;
        for &len in path_lens {
            let len = len.max(0) as usize;
            let mut path: ZPath = Vec::with_capacity(len);
            for _ in 0..len {
                let x = coords[cursor * 3] as i64;
                let y = coords[cursor * 3 + 1] as i64;
                let z = coords[cursor * 3 + 2] as i64;
                path.push((x, y, z) as ZPoint);
                cursor += 1;
            }
            out.push(path);
        }
    }

    // SAFETY: `raw` was produced by cz_clip_extrusion and not freed yet.
    unsafe { clipper_z_sys::cz_free_zpaths(raw) };

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::clipper_z_utils::to_zpath;
    use crate::geometry::Point;

    /// A horizontal extrusion segment crossing the right edge of a square clip
    /// region: only the part inside the clip survives the ctIntersection, and
    /// every surviving vertex keeps a positive (interpolated) Z width.
    ///
    /// The subject has 3 points: OverhangDetector.cpp's clip-boundary Z re-derive
    /// post-pass is guarded by `if (subject.size() <= 2) continue;`, so a 2-point
    /// subject would (faithfully) leave the boundary vertex at Z=0. Real callers
    /// always sample to >2 points.
    #[test]
    fn clip_extrusion_partial_overlap() {
        // Subject: open polyline (-50,50)->(50,50)->(150,50), constant width Z=40.
        let subject: ZPath = vec![(-50, 50, 40), (50, 50, 40), (150, 50, 40)];

        // Clip: closed unit-ish square [0,100] x [0,100].
        let clip_poly = vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ];
        let clip: ZPaths = vec![to_zpath(&clip_poly, 0, false)];

        let result = clip_extrusion(&subject, &clip, ClipType::Intersection);

        // Exactly one clipped open path is expected.
        assert_eq!(result.len(), 1, "expected one clipped open path, got {result:?}");
        let path = &result[0];
        assert!(path.len() >= 2, "clipped path should have >=2 points");

        // The clipped segment must lie within x in [0,100] (the part of the
        // subject inside the clip), at y=50.
        let xs: Vec<i64> = path.iter().map(|p| p.0).collect();
        let min_x = *xs.iter().min().unwrap();
        let max_x = *xs.iter().max().unwrap();
        assert_eq!(min_x, 0, "clip should start at the left clip boundary x=0");
        assert_eq!(max_x, 100, "clip should end at the right clip boundary x=100");
        for p in path {
            assert_eq!(p.1, 50, "y should stay on the subject line");
            // Crucial: every output vertex carries a positive Z (width) tag —
            // this is exactly what the midpoint-band approximation could not do.
            assert!(p.2 > 0, "every clipped vertex must carry a positive Z width, got {p:?}");
            assert_eq!(p.2, 40, "constant-width subject => constant Z at clip boundary");
        }
    }

    /// A subject fully outside the clip yields no surviving path.
    #[test]
    fn clip_extrusion_no_overlap() {
        let subject: ZPath = vec![(500, 500, 40), (600, 500, 40)];
        let clip_poly = vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ];
        let clip: ZPaths = vec![to_zpath(&clip_poly, 0, false)];
        let result = clip_extrusion(&subject, &clip, ClipType::Intersection);
        assert!(result.is_empty(), "disjoint subject should clip to nothing");
    }

    /// Z (width) is interpolated, not constant: a subject whose endpoints have
    /// different widths must produce an intermediate width at the clip boundary.
    #[test]
    fn clip_extrusion_interpolates_z() {
        // Subject from x=-100 (width 20) through (0,40) to x=100 (width 60). The
        // clip boundary at x=0 is the midpoint, so the width there should be ~40.
        // (3 points so the boundary-Z re-derive post-pass is active.)
        let subject: ZPath = vec![(-100, 50, 20), (0, 50, 40), (100, 50, 60)];
        let clip_poly = vec![
            Point::new(0, 0),
            Point::new(200, 0),
            Point::new(200, 100),
            Point::new(0, 100),
        ];
        let clip: ZPaths = vec![to_zpath(&clip_poly, 0, false)];

        let result = clip_extrusion(&subject, &clip, ClipType::Intersection);
        assert_eq!(result.len(), 1);
        let path = &result[0];
        // Find the vertex at the clip boundary x=0.
        let boundary = path.iter().find(|p| p.0 == 0).expect("boundary vertex at x=0");
        // Width at x=0 (midpoint of -100..100) interpolates 20..60 => ~40.
        assert!(
            (boundary.2 - 40).abs() <= 1,
            "width at clip boundary should interpolate to ~40, got {}",
            boundary.2
        );
    }
}
