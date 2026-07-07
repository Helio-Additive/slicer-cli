//! Safe Rust wrapper over the Clipper2-Z C-ABI shim (`clipper2-z-sys`), the
//! faithful engine BambuStudio's `RegionExpansion.cpp::wave_seeds` uses
//! (`Clipper2Lib_Z`). Unlike `clipper_z.rs` (ClipperLib_Z = Clipper1, used by
//! OverhangDetector / FloatingConcentric), this is Clipper2 with `USINGZ`, the
//! only engine whose boolean/offset output is byte-faithful to wave_seeds.
//!
//! Coordinates are `i64` (Clipper2 is 64-bit) and pass through unscaled — no
//! narrowing, unlike the int32 ClipperLib_Z shim.

use crate::clipper_z_utils::{ZPath, ZPaths, ZPoint};

/// One recorded intersection from the wave_seeds Z-callback: the sorted-unique
/// pair `(lower_z, upper_z)` of the two source contour indices that produced an
/// intersection point. A segment point tagged with negative `z` indexes this
/// table at `-z - 1`. Mirrors `Clipper2ZIntersectionVisitor::Intersection`.
pub type Intersection = (i64, i64);

/// Result of [`intersect_open_z`]: the clipped Z-segments (the leading
/// `num_closed` are the closed ones, the rest open) plus the intersections table.
pub struct WaveClip {
    pub segments: ZPaths,
    pub num_closed: usize,
    pub intersections: Vec<Intersection>,
}

/// `Clipper2Lib_Z::ClipperOffset` of a single CLOSED contour (JoinType::Square,
/// EndType::Polygon) by `delta`, preserving the per-vertex Z. Faithful to the
/// per-contour offset inside `expolygons_to_zpaths64_expanded_opened`
/// (RegionExpansion.cpp:118-131): the caller passes `+expansion` for the outer
/// contour, `-expansion` for holes.
pub fn offset_z(contour: &ZPath, delta: f64) -> ZPaths {
    if contour.is_empty() {
        return Vec::new();
    }
    let mut flat: Vec<i64> = Vec::with_capacity(contour.len() * 3);
    for &(x, y, z) in contour {
        flat.push(x);
        flat.push(y);
        flat.push(z);
    }
    // SAFETY: `flat` is a live, correctly-sized buffer for the call; the shim only
    // reads it. The returned Cz2ZPaths owns malloc'd buffers copied out then freed.
    let raw = unsafe { clipper2_z_sys::cz2_offset_z(flat.as_ptr(), contour.len() as i32, delta) };
    let out = unsafe { read_cz2_zpaths(&raw) };
    unsafe { clipper2_z_sys::cz2_free_zpaths(raw) };
    out
}

/// `Clipper2Lib_Z::Clipper64` + `SetZCallback` (the Clipper2ZIntersectionVisitor)
/// + `AddClip(boundary)` + `AddOpenSubject(src)` + `Execute(Intersection,
/// NonZero, closed, open)`. `src` are the OPEN subject Z-paths (the offset source
/// contours, each vertex carrying one `base_idx`); `boundary` are the CLOSED clip
/// Z-paths. Returns the closed+open Z-segments and the recorded intersections
/// table. Faithful to RegionExpansion.cpp:301-322.
pub fn intersect_open_z(src: &ZPaths, boundary: &ZPaths) -> WaveClip {
    let (src_flat, src_lens) = flatten(src);
    let (clip_flat, clip_lens) = flatten(boundary);

    // SAFETY: all pointers reference live, correctly-sized buffers for the call;
    // the shim only reads them. The returned Cz2WaveClip owns malloc'd buffers
    // copied out below and freed via cz2_free_wave_clip.
    let raw = unsafe {
        clipper2_z_sys::cz2_intersect_open_z(
            ptr_or_null(&src_flat),
            ptr_or_null_i32(&src_lens),
            src.len() as i32,
            ptr_or_null(&clip_flat),
            ptr_or_null_i32(&clip_lens),
            boundary.len() as i32,
        )
    };

    let segments = unsafe { read_cz2_zpaths(&raw.segs) };
    let num_closed = raw.num_closed.max(0) as usize;
    let mut intersections: Vec<Intersection> = Vec::with_capacity(raw.num_is.max(0) as usize);
    if raw.num_is > 0 && !raw.is_a.is_null() && !raw.is_b.is_null() {
        let a = unsafe { std::slice::from_raw_parts(raw.is_a, raw.num_is as usize) };
        let b = unsafe { std::slice::from_raw_parts(raw.is_b, raw.num_is as usize) };
        for i in 0..raw.num_is as usize {
            intersections.push((a[i], b[i]));
        }
    }

    unsafe { clipper2_z_sys::cz2_free_wave_clip(raw) };

    WaveClip {
        segments,
        num_closed,
        intersections,
    }
}

// ---------------------------------------------------------------------------
// marshalling helpers
// ---------------------------------------------------------------------------

fn flatten(paths: &ZPaths) -> (Vec<i64>, Vec<i32>) {
    let mut flat: Vec<i64> = Vec::new();
    let mut lens: Vec<i32> = Vec::with_capacity(paths.len());
    for path in paths {
        lens.push(path.len() as i32);
        flat.reserve(path.len() * 3);
        for &(x, y, z) in path {
            flat.push(x);
            flat.push(y);
            flat.push(z);
        }
    }
    (flat, lens)
}

#[inline]
fn ptr_or_null(v: &[i64]) -> *const i64 {
    if v.is_empty() {
        std::ptr::null()
    } else {
        v.as_ptr()
    }
}

#[inline]
fn ptr_or_null_i32(v: &[i32]) -> *const i32 {
    if v.is_empty() {
        std::ptr::null()
    } else {
        v.as_ptr()
    }
}

/// Copy a `Cz2ZPaths` (flat i64 triples + per-path lens) into owned `ZPaths`.
///
/// # Safety
/// `raw` must be a valid `Cz2ZPaths` whose `coords`/`path_lens` are non-dangling
/// for `total_points`/`num_paths` (as produced by the shim).
unsafe fn read_cz2_zpaths(raw: &clipper2_z_sys::Cz2ZPaths) -> ZPaths {
    let mut out: ZPaths = Vec::with_capacity(raw.num_paths.max(0) as usize);
    if raw.num_paths > 0 && !raw.coords.is_null() && !raw.path_lens.is_null() {
        let path_lens = std::slice::from_raw_parts(raw.path_lens, raw.num_paths as usize);
        let coords = std::slice::from_raw_parts(raw.coords, (raw.total_points * 3) as usize);
        let mut cursor = 0usize;
        for &len in path_lens {
            let len = len.max(0) as usize;
            let mut path: ZPath = Vec::with_capacity(len);
            for _ in 0..len {
                let x = coords[cursor * 3];
                let y = coords[cursor * 3 + 1];
                let z = coords[cursor * 3 + 2];
                path.push((x, y, z) as ZPoint);
                cursor += 1;
            }
            out.push(path);
        }
    }
    out
}
