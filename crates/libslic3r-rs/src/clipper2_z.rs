//! The two Clipper2-Z operations BambuStudio's `RegionExpansion.cpp::wave_seeds` uses
//! (`Clipper2Lib_Z`). Behind the gate `CLIPPER2_RUST` (and its sub-gate `CLIPPER2_RUST_Z`)
//! they run on `crate::clipper2::clipper2_z`, the Rust port of the library; with the gate
//! off, on the library itself through the C-ABI shim (`clipper2-z-sys`). Unlike `clipper_z.rs` (ClipperLib_Z = Clipper1, used by
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
    offset_z_impl(clipper2_rust_z(), contour, delta)
}

/// The gate of this file's two functions: `CLIPPER2_RUST` and its sub-gate
/// `CLIPPER2_RUST_Z`, both on unless set to 0. On: `crate::clipper2::clipper2_z`, the Rust
/// port of `Clipper2Lib_Z`. Off: the C++ library through `clipper2-z-sys`.
#[inline]
fn clipper2_rust_z() -> bool {
    crate::faithful_gate("CLIPPER2_RUST") && crate::faithful_gate("CLIPPER2_RUST_Z")
}

/// `offset_z` with the arm of the gate given (`rust`): what
/// `tests/clipper2_utils_native_parity.rs` compares the two arms through.
#[doc(hidden)]
pub fn offset_z_impl(rust: bool, contour: &ZPath, delta: f64) -> ZPaths {
    if rust {
        return offset_z_rust(contour, delta);
    }
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
    intersect_open_z_impl(clipper2_rust_z(), src, boundary)
}

/// `intersect_open_z` with the arm of the gate given (`rust`).
#[doc(hidden)]
pub fn intersect_open_z_impl(rust: bool, src: &ZPaths, boundary: &ZPaths) -> WaveClip {
    if rust {
        return intersect_open_z_rust(src, boundary);
    }
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
// The Rust arms: the bodies of `cz2_offset_z` and `cz2_intersect_open_z`
// (crates/clipper2-z-sys/shim/clipper2_z_shim.cpp), statement for statement, on
// `crate::clipper2::clipper2_z` (`namespace Clipper2Lib_Z`).
// ---------------------------------------------------------------------------

use crate::clipper2::clipper2_z as c2z;

/// `read_zpaths` of clipper2_z_shim.cpp: (x, y, z) triples into `Clipper2Lib_Z::Paths64`.
fn read_zpaths(paths: &ZPaths) -> c2z::Paths64 {
    let mut out = c2z::Paths64::with_capacity(paths.len());
    for p in paths {
        let mut path = c2z::Path64::with_capacity(p.len());
        for &(x, y, z) in p {
            path.push(c2z::Point64::new_xyz(x, y, z));
        }
        out.push(path);
    }
    out
}

/// `marshal_zpaths` of clipper2_z_shim.cpp and `read_cz2_zpaths` below in one step:
/// `Clipper2Lib_Z::Paths64` into (x, y, z) triples.
fn marshal_zpaths(paths: &c2z::Paths64) -> ZPaths {
    let mut out: ZPaths = Vec::with_capacity(paths.len());
    for p in paths {
        let mut path: ZPath = Vec::with_capacity(p.len());
        for ip in p {
            path.push((ip.x, ip.y, ip.z) as ZPoint);
        }
        out.push(path);
    }
    out
}

/// `cz2_offset_z` (clipper2_z_shim.cpp), which is the body of the loop of
/// `expolygons_to_zpaths64_expanded_opened` (RegionExpansion.cpp:117-131) for one contour:
/// C++: offsetter.Clear();
/// C++: path.push_back(Clipper2Lib_Z::Point64{p.x(), p.y(), base_idx});
/// C++: offsetter.AddPath(path, Clipper2Lib_Z::JoinType::Square, Clipper2Lib_Z::EndType::Polygon);
/// C++: offsetter.Execute(offset_distance, expansion_cache);
/// The C++ clears one offsetter for every contour; the shim, and this, make a new one.
/// `ClipperOffset::Execute` takes nothing from an earlier run with these arguments.
fn offset_z_rust(contour: &ZPath, delta: f64) -> ZPaths {
    // `offset_z` returns before the call for a contour without points.
    if contour.is_empty() {
        return Vec::new();
    }
    let mut offsetter = c2z::ClipperOffset::default();
    let mut path = c2z::Path64::with_capacity(contour.len());
    for &(x, y, z) in contour {
        path.push(c2z::Point64::new_xyz(x, y, z));
    }
    offsetter.add_path(&path, c2z::JoinType::Square, c2z::EndType::Polygon);

    let mut result = c2z::Paths64::new();
    offsetter.execute(delta, &mut result);
    marshal_zpaths(&result)
}

/// `cz2_intersect_open_z` (clipper2_z_shim.cpp), which is `wave_seeds`
/// (RegionExpansion.cpp:301-322) from the clipper to `segments`, with
/// `Clipper2ZIntersectionVisitor::operator()` (Clipper2ZUtils.hpp:137-151) as the callback:
/// C++: zclipper.SetZCallback(visitor.clipper_callback());
/// C++: zclipper.AddClip(Clipper2ZUtils::expolygons_to_zpaths64(boundary, idx_boundary_end));
/// C++: zclipper.AddOpenSubject(zsrc);
/// C++: zclipper.Execute(Clipper2Lib_Z::ClipType::Intersection, Clipper2Lib_Z::FillRule::NonZero, closed_segs, open_segs);
/// C++: segments.insert(segments.end(), closed_segs...); segments.insert(segments.end(), open_segs...);
fn intersect_open_z_rust(src_paths: &ZPaths, boundary: &ZPaths) -> WaveClip {
    let src = read_zpaths(src_paths); // open subject
    let clip = read_zpaths(boundary); // closed clip

    // The Intersections table the Z-callback records (Clipper2ZIntersectionVisitor,
    // Clipper2ZUtils.hpp). Each entry is the (sorted, unique) pair of the two source
    // z-values that produced an intersection point; the point's z becomes
    // -(table_size) so callers can index it as `-z - 1`.
    let intersections: std::rc::Rc<std::cell::RefCell<Vec<Intersection>>> = std::rc::Rc::new(std::cell::RefCell::new(Vec::new()));

    let mut clipper = c2z::Clipper64::new();
    // SetZCallback — verbatim Clipper2ZIntersectionVisitor::operator() logic.
    let table = intersections.clone();
    clipper.set_z_callback(c2z::ZCallback64::new(
        move |e1bot: &c2z::Point64, e1top: &c2z::Point64, e2bot: &c2z::Point64, e2top: &c2z::Point64, pt: &mut c2z::Point64| {
            let mut srcs = [e1bot.z, e1top.z, e2bot.z, e2top.z];
            // std::sort of four int64_t, then std::unique: equal values are not told apart.
            srcs.sort_unstable();
            let mut new_size = 1usize;
            for i in 1..4 {
                if srcs[i] != srcs[new_size - 1] {
                    srcs[new_size] = srcs[i];
                    new_size += 1;
                }
            }
            if new_size == 1 {
                pt.z = srcs[0];
            } else if new_size == 2 {
                let mut table = table.borrow_mut();
                table.push((srcs[0], srcs[1]));
                pt.z = -(table.len() as i64);
            }
        },
    ));

    clipper.add_clip(&clip);
    clipper.add_open_subject(&src);

    let mut closed_segs = c2z::Paths64::new();
    let mut open_segs = c2z::Paths64::new();
    clipper.execute_open(c2z::ClipType::Intersection, c2z::FillRule::NonZero, &mut closed_segs, &mut open_segs);

    // segments = closed_segs ++ open_segs (the order wave_seeds consumes).
    let num_closed = closed_segs.len();
    let mut segments = marshal_zpaths(&closed_segs);
    segments.extend(marshal_zpaths(&open_segs));

    drop(clipper);
    let intersections = intersections.borrow().clone();
    WaveClip { segments, num_closed, intersections }
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
