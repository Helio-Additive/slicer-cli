//! The two Clipper2-Z operations BambuStudio's `RegionExpansion.cpp::wave_seeds` uses
//! (`Clipper2Lib_Z`), on `crate::clipper2::clipper2_z`, the Rust port of the library.
//! `tests/clipper2_utils_native_parity.rs` holds them equal to the library itself
//! (`cz2_offset_z`, `cz2_intersect_open_z` of `clipper2-z-sys`). Unlike `clipper_z.rs` (ClipperLib_Z = Clipper1, used by
//! OverhangDetector / FloatingConcentric), this is Clipper2 with `USINGZ`, the
//! only engine whose boolean/offset output is byte-faithful to wave_seeds.
//!
//! Coordinates are `i64` (Clipper2 is 64-bit) and pass through unscaled — no
//! narrowing, unlike the int32 ClipperLib_Z.

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

// ---------------------------------------------------------------------------
// The bodies of `cz2_offset_z` and `cz2_intersect_open_z`
// (crates/clipper2-z-sys/shim/clipper2_z_shim.cpp, the test reference), statement for
// statement, on `crate::clipper2::clipper2_z` (`namespace Clipper2Lib_Z`).
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

/// `marshal_zpaths` of clipper2_z_shim.cpp and the reading back of its result in one step:
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

/// `Clipper2Lib_Z::ClipperOffset` of a single CLOSED contour (JoinType::Square,
/// EndType::Polygon) by `delta`, preserving the per-vertex Z. Faithful to the
/// per-contour offset inside `expolygons_to_zpaths64_expanded_opened`
/// (RegionExpansion.cpp:118-131): the caller passes `+expansion` for the outer
/// contour, `-expansion` for holes.
///
/// `cz2_offset_z` (clipper2_z_shim.cpp), which is the body of the loop of
/// `expolygons_to_zpaths64_expanded_opened` (RegionExpansion.cpp:117-131) for one contour:
/// C++: offsetter.Clear();
/// C++: path.push_back(Clipper2Lib_Z::Point64{p.x(), p.y(), base_idx});
/// C++: offsetter.AddPath(path, Clipper2Lib_Z::JoinType::Square, Clipper2Lib_Z::EndType::Polygon);
/// C++: offsetter.Execute(offset_distance, expansion_cache);
/// The C++ clears one offsetter for every contour; the shim, and this, make a new one.
/// `ClipperOffset::Execute` takes nothing from an earlier run with these arguments.
pub fn offset_z(contour: &ZPath, delta: f64) -> ZPaths {
    // Nothing to offset: the shim is not called for a contour without points.
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

/// `Clipper2Lib_Z::Clipper64` + `SetZCallback` (the Clipper2ZIntersectionVisitor)
/// + `AddClip(boundary)` + `AddOpenSubject(src)` + `Execute(Intersection,
/// NonZero, closed, open)`. `src` are the OPEN subject Z-paths (the offset source
/// contours, each vertex carrying one `base_idx`); `boundary` are the CLOSED clip
/// Z-paths. Returns the closed+open Z-segments and the recorded intersections
/// table. Faithful to RegionExpansion.cpp:301-322.
///
/// `cz2_intersect_open_z` (clipper2_z_shim.cpp), which is `wave_seeds`
/// (RegionExpansion.cpp:301-322) from the clipper to `segments`, with
/// `Clipper2ZIntersectionVisitor::operator()` (Clipper2ZUtils.hpp:137-151) as the callback:
/// C++: zclipper.SetZCallback(visitor.clipper_callback());
/// C++: zclipper.AddClip(Clipper2ZUtils::expolygons_to_zpaths64(boundary, idx_boundary_end));
/// C++: zclipper.AddOpenSubject(zsrc);
/// C++: zclipper.Execute(Clipper2Lib_Z::ClipType::Intersection, Clipper2Lib_Z::FillRule::NonZero, closed_segs, open_segs);
/// C++: segments.insert(segments.end(), closed_segs...); segments.insert(segments.end(), open_segs...);
pub fn intersect_open_z(src_paths: &ZPaths, boundary: &ZPaths) -> WaveClip {
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
