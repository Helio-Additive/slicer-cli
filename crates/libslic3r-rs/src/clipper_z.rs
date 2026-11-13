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

use crate::clipper_lib_z;
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
///
/// Runs on the Rust `ClipperLib_Z` unless the environment has `CLIPPER_Z_RUST=0`, which
/// runs the C++ of `clipper-z-sys` (`clip_extrusion_shim`). On the Rust `ClipperLib_Z` the
/// pass over the vertices left with Z == 0 is the engine's (OverhangDetector.cpp:68-98,
/// `clip_extrusion_cpp`) unless the environment has `CLIP_EXTRUSION_CPP=0`, which runs
/// the shim's pass (`clip_extrusion_port`).
pub fn clip_extrusion(subject: &ZPath, clip: &ZPaths, clip_type: ClipType) -> ZPaths {
    if crate::faithful_gate("CLIPPER_Z_RUST") {
        if crate::faithful_gate("CLIP_EXTRUSION_CPP") {
            return clip_extrusion_cpp(subject, clip, clip_type);
        }
        return clip_extrusion_port(subject, clip, clip_type);
    }
    clip_extrusion_shim(subject, clip, clip_type)
}

/// `ZPath` (libslic3r coordinates, `i64`) to `ClipperLib_Z::Path` (`cInt` is `i32`).
fn to_clipper_lib_z_path(path: &ZPath) -> clipper_lib_z::Path {
    path.iter().map(|&(x, y, z)| clipper_lib_z::IntPoint::new_xyz(narrow_i32(x), narrow_i32(y), narrow_i32(z))).collect()
}

/// `ClipperLib_Z::Paths` back to `ZPaths`.
fn from_clipper_lib_z_paths(paths: &clipper_lib_z::Paths) -> ZPaths {
    paths.iter().map(|path| path.iter().map(|p| (p.x as i64, p.y as i64, p.z as i64) as ZPoint).collect()).collect()
}

/// `(a - b).cast<double>().squaredNorm()` of two `ClipperLib_Z::IntPoint`: the difference
/// is taken in `cInt` over the three coordinates, Z among them, and each is squared as a
/// double. The build sums them left to right, `(x*x + y*y) + z*z`, and fuses nothing (the
/// squares and the sums are separate functions of Eigen): measured on the build's Eigen
/// and flags with two million random pairs, of which 974,656 tell the orders and the
/// fused forms apart. The tests here cannot tell them apart: the Z that comes of it is
/// truncated to an integer.
#[inline]
fn squared_norm_of_difference(a: &clipper_lib_z::IntPoint, b: &clipper_lib_z::IntPoint) -> f64 {
    let dx = a.x.wrapping_sub(b.x) as f64;
    let dy = a.y.wrapping_sub(b.y) as f64;
    let dz = a.z.wrapping_sub(b.z) as f64;
    (dx * dx + dy * dy) + dz * dz
}

/// `clip_extrusion` on the Rust `ClipperLib_Z` (`crate::clipper_lib_z`): the body of
/// `cz_clip_extrusion` (`crates/clipper-z-sys/shim/clipper_z_shim.cpp`), statement for
/// statement, which follows OverhangDetector.cpp:18-108.
/// `tests/clipper_z_engine_native_parity.rs` holds it equal to `clip_extrusion_shim`.
/// Its pass over the vertices left with Z == 0 is the shim's, which is not the engine's:
/// `clip_extrusion_cpp` has the engine's.
pub fn clip_extrusion_port(subject: &ZPath, clip: &ZPaths, clip_type: ClipType) -> ZPaths {
    clip_extrusion_impl(subject, clip, clip_type, false)
}

/// `clip_extrusion` of OverhangDetector.cpp:18-108 on the Rust `ClipperLib_Z`, statement
/// for statement: `clip_extrusion_port` with the engine's own pass over the vertices left
/// with Z == 0 (OverhangDetector.cpp:68-98, `assign_z_from_subject_cpp`) in place of the
/// shim's.
pub fn clip_extrusion_cpp(subject: &ZPath, clip: &ZPaths, clip_type: ClipType) -> ZPaths {
    clip_extrusion_impl(subject, clip, clip_type, true)
}

/// `(a - b).cast<double>().squaredNorm()` of two `Point` (`Vec2crd`, `coord_t` is
/// `int32_t`, libslic3r.h:40): the difference is taken in `coord_t`, each coordinate is
/// squared as a double and the two are summed; nothing is fused (the squares and the sum
/// are separate functions of Eigen).
#[inline]
fn squared_norm_of_difference_2d(a: (i32, i32), b: (i32, i32)) -> f64 {
    let dx = a.0.wrapping_sub(b.0) as f64;
    let dy = a.1.wrapping_sub(b.1) as f64;
    dx * dx + dy * dy
}

/// `Point::projection_onto(const Line &line)` (Point.cpp:157-180), `self` being `pt` and
/// the line `(a, b)`, on `coord_t` (`int32_t`) coordinates.
///
/// Written here and not taken from `Point::project_onto_segment` (geometry/point.rs),
/// whose `theta` does not fuse the numerator as the arm64 build does and which works on
/// the crate's 64-bit `Point`; the two agree while the products are exact in a double.
#[inline]
fn projection_onto_line_cpp(pt: (i32, i32), a: (i32, i32), b: (i32, i32)) -> (i32, i32) {
    // Point.cpp:159
    if a == b {
        return a;
    }
    // Point.cpp:170-171
    let lx = b.0.wrapping_sub(a.0) as f64;
    let ly = b.1.wrapping_sub(a.1) as f64;
    // Point.cpp:172-173. The numerator is one expression, `p*lx + q*ly`: the left product
    // is fused into the sum. The denominator sums two calls of sqr<double>: not fused.
    let theta = (b.0.wrapping_sub(pt.0) as f64).mul_add(lx, (b.1.wrapping_sub(pt.1) as f64) * ly) / (lx * lx + ly * ly);
    // Point.cpp:175-176: (theta * a.cast<coordf_t>() + (1.0-theta) * b.cast<coordf_t>()).cast<coord_t>(),
    // an Eigen expression (products and sum are separate functions: not fused), truncated.
    if 0.0 <= theta && theta <= 1.0 {
        return (
            (theta * a.0 as f64 + (1.0 - theta) * b.0 as f64) as i32,
            (theta * a.1 as f64 + (1.0 - theta) * b.1 as f64) as i32,
        );
    }
    // Point.cpp:178-179: else pick closest endpoint.
    if squared_norm_of_difference_2d(a, pt) < squared_norm_of_difference_2d(b, pt) {
        a
    } else {
        b
    }
}

/// OverhangDetector.cpp:68-98: the vertices of the clipped paths that came from the clip
/// have Z == 0; each takes the width of the subject at the nearest point of the subject.
fn assign_z_from_subject_cpp(clipped_paths: &mut clipper_lib_z::Paths, subject: &clipper_lib_z::Path) {
    // OverhangDetector.cpp:68-70
    for path in clipped_paths.iter_mut() {
        for c_pt in path.iter_mut() {
            if c_pt.z != 0 {
                continue;
            }
            // OverhangDetector.cpp:72
            if subject.len() <= 2 {
                continue;
            }

            // OverhangDetector.cpp:74-78
            let pt = (c_pt.x, c_pt.y);
            let mut projected_pt_min = (0i32, 0i32); // Point() is (0, 0), Point.hpp:176
            let mut it_min: usize = 0;
            let mut dist_sqr_min = f64::MAX;
            let mut prev = (subject[0].x, subject[0].y);
            // OverhangDetector.cpp:79-88
            for it in 1..subject.len() {
                let curr = (subject[it].x, subject[it].y);
                // Point::projection_onto(Line): on the segment, in integer coordinates.
                let projected_pt = projection_onto_line_cpp(pt, prev, curr);
                let dist_sqr = squared_norm_of_difference_2d(projected_pt, pt);
                if dist_sqr < dist_sqr_min {
                    dist_sqr_min = dist_sqr;
                    projected_pt_min = projected_pt;
                    it_min = it - 1;
                }
                prev = curr;
            }

            // OverhangDetector.cpp:93-97
            let pt_a = (subject[it_min].x, subject[it_min].y);
            let pt_b = (subject[it_min + 1].x, subject[it_min + 1].y);
            let line_len = squared_norm_of_difference_2d(pt_b, pt_a).sqrt();
            let dist = squared_norm_of_difference_2d(projected_pt_min, pt_a).sqrt();
            // c_pt.z() = coord_t(double(it_min->z()) + (dist / line_len) * double(std::next(it_min)->z() - it_min->z()));
            // The product is fused into the sum. A subject segment of no length gives
            // 0 / 0, a NaN, which the arm64 build converts to 0, as Rust's `as` does;
            // the shim's pass gives the Z of the segment's first point there.
            let z_a = subject[it_min].z;
            let z_b = subject[it_min + 1].z;
            c_pt.z = (dist / line_len).mul_add(z_b.wrapping_sub(z_a) as f64, z_a as f64) as i32;
        }
    }
}

/// The body of `clip_extrusion_port` and `clip_extrusion_cpp`: `cpp_post_pass` chooses
/// the pass over the vertices left with Z == 0, the engine's or the shim's.
fn clip_extrusion_impl(subject: &ZPath, clip: &ZPaths, clip_type: ClipType, cpp_post_pass: bool) -> ZPaths {
    use clipper_lib_z::IntPoint;
    // The wrapper of the shim returns before it calls the shim.
    if subject.is_empty() {
        return Vec::new();
    }

    // Build the open subject ZPath.
    let subject: clipper_lib_z::Path = to_clipper_lib_z_path(subject);

    // Build the closed clip ZPaths.
    let clip: clipper_lib_z::Paths = clip.iter().map(to_clipper_lib_z_path).collect();

    let mut clipper = clipper_lib_z::Clipper::new(0);
    // OverhangDetector.cpp:21-55 — the ZFillFunction lambda.
    clipper.z_fill_function(clipper_lib_z::ZFillCallback::new(
        |e1bot: &IntPoint, e1top: &IntPoint, e2bot: &IntPoint, e2top: &IntPoint, pt: &mut IntPoint| {
            // Both ends of each edge belong to the same source: subject or clip.

            // Start & end points of the clipped polyline (extrusion path).
            let mut start = *e1bot;
            let mut end = *e1top;
            if start.z <= 0 && end.z <= 0 {
                start = *e2bot;
                end = *e2top;
            }

            if start.z <= 0 && end.z <= 0 {
                // Self intersection on the source contour.
                pt.z = 0;
            } else {
                // Interpolate extrusion line width.
                let length_sqr = squared_norm_of_difference(&end, &start);
                let dist_sqr = squared_norm_of_difference(pt, &start);
                let t = (dist_sqr / length_sqr).sqrt();
                // pt.z() = start.z() + coord_t((end.z() - start.z()) * t);
                // FIDELITY-NOTE: a double outside coord_t (a subject edge of no length
                // gives an infinite or NaN `t`) is undefined in the C++; the arm64 build
                // converts as Rust's `as` does (toward zero, saturating, 0 for a NaN),
                // which is what stands here. Other targets may give the C++ another value.
                pt.z = start.z.wrapping_add((end.z.wrapping_sub(start.z) as f64 * t) as i32);
            }
        },
    ));

    clipper.add_path(&subject, clipper_lib_z::PolyType::ptSubject, false);
    clipper.add_paths(&clip, clipper_lib_z::PolyType::ptClip, true);

    let mut clipped_polytree = clipper_lib_z::PolyTree::new();
    let mut clipped_paths = clipper_lib_z::Paths::new();
    let clip_type = match clip_type {
        ClipType::Intersection => clipper_lib_z::ClipType::ctIntersection,
        ClipType::Union => clipper_lib_z::ClipType::ctUnion,
        ClipType::Difference => clipper_lib_z::ClipType::ctDifference,
        ClipType::Xor => clipper_lib_z::ClipType::ctXor,
    };
    clipper.execute_poly_tree(
        clip_type,
        &mut clipped_polytree,
        clipper_lib_z::PolyFillType::pftNonZero,
        clipper_lib_z::PolyFillType::pftNonZero,
    );
    clipper_lib_z::poly_tree_to_paths(&clipped_polytree, &mut clipped_paths);

    // Clipped path could contain vertices from the clip with Z == 0; assign a
    // value from the subject by projecting onto the nearest subject segment.
    if cpp_post_pass {
        // OverhangDetector.cpp:68-98, the engine's pass.
        assign_z_from_subject_cpp(&mut clipped_paths, &subject);
        return from_clipper_lib_z_paths(&clipped_paths);
    }
    // FIDELITY-NOTE: what follows is the shim's post-pass, not OverhangDetector.cpp:68-98
    // (that one is `assign_z_from_subject_cpp`). The engine projects with
    // Point::projection_onto(Line) (Point.cpp:157-180), which keeps the projection on the
    // segment (the nearer end when it falls outside) and truncates it to integer
    // coordinates; the shim projects onto the infinite line and keeps the doubles, and
    // gives a subject segment of no length the Z of its first point where the engine's
    // 0 / 0 converts to 0. Kept for `CLIP_EXTRUSION_CPP=0` and for the test against the shim.
    for path in clipped_paths.iter_mut() {
        for c_pt in path.iter_mut() {
            if c_pt.z != 0 {
                continue;
            }
            if subject.len() <= 2 {
                continue;
            }

            let px = c_pt.x as f64;
            let py = c_pt.y as f64;
            let mut dist_sqr_min = f64::MAX;
            let mut it_min: usize = 0;
            let (mut proj_min_x, mut proj_min_y) = (0.0f64, 0.0f64);

            let mut prev_x = subject[0].x as f64;
            let mut prev_y = subject[0].y as f64;
            for i in 1..subject.len() {
                let cx = subject[i].x as f64;
                let cy = subject[i].y as f64;
                // Projection of pt onto segment (prev -> curr) as an infinite line.
                let dx = cx - prev_x;
                let dy = cy - prev_y;
                // dx * dx + dy * dy: the left product is fused into the sum.
                let seg_len_sqr = dx.mul_add(dx, dy * dy);
                let (proj_x, proj_y);
                if seg_len_sqr <= 0.0 {
                    proj_x = prev_x;
                    proj_y = prev_y;
                } else {
                    // ((px - prev_x) * dx + (py - prev_y) * dy) / seg_len_sqr
                    let tparam = (px - prev_x).mul_add(dx, (py - prev_y) * dy) / seg_len_sqr;
                    // prev_x + tparam * dx
                    proj_x = tparam.mul_add(dx, prev_x);
                    proj_y = tparam.mul_add(dy, prev_y);
                }
                let ddx = proj_x - px;
                let ddy = proj_y - py;
                let dist_sqr = ddx.mul_add(ddx, ddy * ddy);
                if dist_sqr < dist_sqr_min {
                    dist_sqr_min = dist_sqr;
                    proj_min_x = proj_x;
                    proj_min_y = proj_y;
                    it_min = i - 1;
                }
                prev_x = cx;
                prev_y = cy;
            }

            let pa_x = subject[it_min].x as f64;
            let pa_y = subject[it_min].y as f64;
            let pb_x = subject[it_min + 1].x as f64;
            let pb_y = subject[it_min + 1].y as f64;
            let line_len = (pb_x - pa_x).mul_add(pb_x - pa_x, (pb_y - pa_y) * (pb_y - pa_y)).sqrt();
            let dist = (proj_min_x - pa_x).mul_add(proj_min_x - pa_x, (proj_min_y - pa_y) * (proj_min_y - pa_y)).sqrt();
            let za = subject[it_min].z as f64;
            let zb = subject[it_min + 1].z as f64;
            if line_len > 0.0 {
                // (coord_t)(za + (dist / line_len) * (zb - za))
                c_pt.z = (dist / line_len).mul_add(zb - za, za) as i32;
            } else {
                c_pt.z = za as i32;
            }
        }
    }

    // Marshal into the output.
    from_clipper_lib_z_paths(&clipped_paths)
}

/// `clip_extrusion` through the C++ of `clipper-z-sys` (`cz_clip_extrusion`).
pub fn clip_extrusion_shim(subject: &ZPath, clip: &ZPaths, clip_type: ClipType) -> ZPaths {
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

/// Faithful `detect_floating_line` Z-clipper (FillFloatingConcentric.cpp:431-475).
///
/// Runs the ClipperLib_Z Clipper twice on the same inputs — `ctIntersection` and
/// `ctDifference` — under the detect_floating_line `ZFillFunction` (negative-hash
/// tagging of subject×clip intersections; common-subject-z for subject self-
/// intersections). `subject` is the OPEN thick-polyline path whose Z is the
/// vertex index `0..subject_idx_range`; `clip` are the CLOSED floating-area paths
/// whose Z is a per-vertex index `>= subject_idx_range`.
///
/// Returns `(to_merge, num_diff_paths)` where `to_merge == [diff_paths...,
/// intersect_paths...]` and the leading `num_diff_paths` are the non-floating
/// (ctDifference) paths; the rest are the floating (ctIntersection) paths. This
/// is exactly C++ `to_merge = diff_out ++ intersect_out` with `floating_flags`
/// set true for the intersect tail.
///
/// Note: unlike `clip_extrusion`, the Z values here are vertex indices / hashes,
/// NOT coordinates — they are passed through unscaled (and fit i32: the hash is
/// masked to `0x7fffffff` then negated).
///
/// Runs on the Rust `ClipperLib_Z` (`detect_floating_port`) unless the environment has
/// `CLIPPER_Z_RUST=0`, which runs the C++ of `clipper-z-sys` (`detect_floating_shim`).
pub fn detect_floating(subject: &ZPath, clip: &ZPaths, subject_idx_range: i32) -> (ZPaths, usize) {
    if crate::faithful_gate("CLIPPER_Z_RUST") {
        return detect_floating_port(subject, clip, subject_idx_range);
    }
    detect_floating_shim(subject, clip, subject_idx_range)
}

/// `detect_floating` on the Rust `ClipperLib_Z` (`crate::clipper_lib_z`): the body of
/// `cz_detect_floating` (`crates/clipper-z-sys/shim/clipper_z_shim.cpp`), statement for
/// statement, which follows FillFloatingConcentric.cpp:407-481 (`detect_floating_line`).
/// `tests/clipper_z_engine_native_parity.rs` holds it equal to `detect_floating_shim`.
pub fn detect_floating_port(subject: &ZPath, clip: &ZPaths, subject_idx_range: i32) -> (ZPaths, usize) {
    use clipper_lib_z::IntPoint;
    // The wrapper of the shim returns before it calls the shim.
    if subject.is_empty() {
        return (Vec::new(), 0);
    }

    // FillFloatingConcentric.cpp:412-415 — build the OPEN subject ZPath (one path).
    let subject_paths: clipper_lib_z::Paths = vec![to_clipper_lib_z_path(subject)];

    // FillFloatingConcentric.cpp:418-426 — build the CLOSED clip ZPaths.
    let clip_paths: clipper_lib_z::Paths = clip.iter().map(to_clipper_lib_z_path).collect();

    // FillFloatingConcentric.cpp:407-411 — the hash function. The arithmetic is `int`,
    // which wraps in the build.
    let hash_function = |a1: i32, b1: i32, a2: i32, b2: i32| -> i32 {
        // int32_t hash_val = 1000 * (a1 * 13 + b1) + (a2 * 17 + b2) + 1;
        let mut hash_val = 1000i32
            .wrapping_mul(a1.wrapping_mul(13).wrapping_add(b1))
            .wrapping_add(a2.wrapping_mul(17).wrapping_add(b2))
            .wrapping_add(1);
        hash_val &= 0x7fffffff;
        hash_val
    };

    // FillFloatingConcentric.cpp:431-456 — the ZFillFunction. The lambda captures
    // subject_idx_range and reads it in its diagnostics only, which the shim drops.
    let _ = subject_idx_range;
    let z_filler = clipper_lib_z::ZFillCallback::new(
        move |e1_a: &IntPoint, e1_b: &IntPoint, e2_a: &IntPoint, e2_b: &IntPoint, d: &mut IntPoint| {
            // FillFloatingConcentric.cpp:440-445 — both edges from the subject:
            // the intersect is generated by two lines in subject -> keep subject z.
            if e1_a.z == e1_b.z && e1_b.z == e2_a.z && e2_a.z == e2_b.z {
                d.z = e1_a.z;
                return;
            }
            // FillFloatingConcentric.cpp:447 — subject x clip intersection -> negative hash.
            d.z = hash_function(e1_a.z, e1_b.z, e2_a.z, e2_b.z).wrapping_neg();
        },
    );

    // FillFloatingConcentric.cpp:457-464 — ctIntersection pass.
    let mut intersect_out = clipper_lib_z::Paths::new();
    {
        let mut c = clipper_lib_z::Clipper::new(0);
        let mut polytree = clipper_lib_z::PolyTree::new();
        c.z_fill_function(z_filler.clone());
        c.add_paths(&subject_paths, clipper_lib_z::PolyType::ptSubject, false);
        c.add_paths(&clip_paths, clipper_lib_z::PolyType::ptClip, true);
        c.execute_poly_tree(
            clipper_lib_z::ClipType::ctIntersection,
            &mut polytree,
            clipper_lib_z::PolyFillType::pftNonZero,
            clipper_lib_z::PolyFillType::pftNonZero,
        );
        clipper_lib_z::poly_tree_to_paths(&polytree, &mut intersect_out);
    }

    // FillFloatingConcentric.cpp:467-474 — ctDifference pass.
    let mut diff_out = clipper_lib_z::Paths::new();
    {
        let mut c = clipper_lib_z::Clipper::new(0);
        let mut polytree = clipper_lib_z::PolyTree::new();
        c.z_fill_function(z_filler.clone());
        c.add_paths(&subject_paths, clipper_lib_z::PolyType::ptSubject, false);
        c.add_paths(&clip_paths, clipper_lib_z::PolyType::ptClip, true);
        c.execute_poly_tree(
            clipper_lib_z::ClipType::ctDifference,
            &mut polytree,
            clipper_lib_z::PolyFillType::pftNonZero,
            clipper_lib_z::PolyFillType::pftNonZero,
        );
        clipper_lib_z::poly_tree_to_paths(&polytree, &mut diff_out);
    }

    // FillFloatingConcentric.cpp:477-481 — to_merge = diff_out ++ intersect_out;
    // floating_flags true for the intersect tail.
    let out_num_diff_paths = diff_out.len();
    let mut to_merge = diff_out;
    to_merge.extend(intersect_out.iter().cloned());

    (from_clipper_lib_z_paths(&to_merge), out_num_diff_paths)
}

/// `detect_floating` through the C++ of `clipper-z-sys` (`cz_detect_floating`).
pub fn detect_floating_shim(subject: &ZPath, clip: &ZPaths, subject_idx_range: i32) -> (ZPaths, usize) {
    if subject.is_empty() {
        return (Vec::new(), 0);
    }

    let mut subject_flat: Vec<i32> = Vec::with_capacity(subject.len() * 3);
    for &(x, y, z) in subject {
        subject_flat.push(narrow_i32(x));
        subject_flat.push(narrow_i32(y));
        subject_flat.push(narrow_i32(z));
    }

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

    let mut num_diff_paths: i32 = 0;
    // SAFETY: pointers reference live, correctly-sized Vecs for the duration of
    // the call; the shim only reads them. The returned CzZPaths owns malloc'd
    // buffers copied out and freed below.
    let raw = unsafe {
        clipper_z_sys::cz_detect_floating(
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
            subject_idx_range,
            &mut num_diff_paths as *mut i32,
        )
    };

    let mut out: ZPaths = Vec::with_capacity(raw.num_paths.max(0) as usize);
    if raw.num_paths > 0 && !raw.coords.is_null() && !raw.path_lens.is_null() {
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

    // SAFETY: `raw` was produced by cz_detect_floating and not freed yet.
    unsafe { clipper_z_sys::cz_free_zpaths(raw) };

    (out, num_diff_paths.max(0) as usize)
}

// ---------------------------------------------------------------------------
// Open-path polyline clipping built on clip_extrusion.
//
// These mirror BambuStudio's `intersection_pl_2` / `diff_pl_2`
// (Clipper2Utils.cpp): clip an OPEN polyline subject against CLOSED polygons,
// returning the surviving open sub-polylines. Implemented here on top of the
// ClipperLib_Z `clip_extrusion` engine so the clip carries a per-point Z tag —
// which the overhang grader uses to recover the extrusion width along each
// surviving segment (the midpoint-band approximation could not do this).
//
// The subject Z is set to a caller-provided constant (the scaled extrusion
// width). With a constant Z, clip_extrusion's interpolation is a no-op, so every
// surviving vertex carries exactly that width. The clip polygons get Z=0.
// ---------------------------------------------------------------------------

use crate::geometry::{Point, Polygon, Polyline};

/// Build a closed clip ZPaths from polygons (Z = 0, matching C++ clip paths).
fn polygons_to_clip_zpaths(clip: &[Polygon]) -> ZPaths {
    clip.iter()
        .map(|poly| poly.points.iter().map(|p| (p.x(), p.y(), 0i64)).collect::<ZPath>())
        .collect()
}

/// Clip an open polyline `subject` against closed `clip` polygons and return the
/// surviving open sub-polylines, with each output vertex tagged by the scaled
/// extrusion width `z`. `clip_type` selects intersection (inside) vs difference
/// (outside). Empty / degenerate subjects yield an empty result.
pub fn clip_polyline(
    subject: &Polyline,
    clip: &[Polygon],
    z: i64,
    clip_type: ClipType,
) -> Vec<(Polyline, ZPath)> {
    if subject.points.len() < 2 {
        return Vec::new();
    }
    let subject_zpath: ZPath = subject.points.iter().map(|p| (p.x(), p.y(), z)).collect();
    let clip_zpaths = polygons_to_clip_zpaths(clip);
    let clipped = clip_extrusion(&subject_zpath, &clip_zpaths, clip_type);
    clipped
        .into_iter()
        .filter(|zp| zp.len() >= 2)
        .map(|zp| {
            let pl = Polyline::from_points(zp.iter().map(|&(x, y, _)| Point::new(x, y)).collect());
            (pl, zp)
        })
        .collect()
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
