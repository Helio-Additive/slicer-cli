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

    /// Faithful replica of `ClipperUtils.cpp` `offset(const Polygons&, delta)`
    /// (`offset_paths` -> `expand_paths`/`shrink_paths` over `raw_offset`).
    /// Unlike [`cz_offset_expolygon`], the input is a flat set of paths whose
    /// ORIENTATION is meaningful: each path is offset on its own with
    /// `Execute(ccw ? delta : -delta)` and its output reversed when the input
    /// was CW, then the whole set is unioned (pftNonZero for delta > 0; the
    /// bounding-frame pftNegative trick with the outermost polygon dropped for
    /// delta < 0). `xy` is flat (x,y) pairs, `lens` per-path point counts.
    /// Free via [`cz_free_zpaths`].
    pub fn cz_offset_paths(
        xy: *const i32,
        lens: *const i32,
        num: i32,
        delta: f64,
        join_type: i32,
        miter_limit: f64,
    ) -> CzZPaths;

    /// Faithful replica of `RegionExpansion.cpp` `propagate_wave_from_boundary`
    /// (+ `wavefront_initial`/`wavefront_step`/`wavefront_clip`): the ClipperLib
    /// wavefront propagation for one (boundary, src) seed group. `seed_*` are the
    /// open seed polylines (flat i32 (x,y) pairs + per-path lens); `bnd_*` are the
    /// boundary ExPolygon's closed paths (contour CCW + holes CW). Returns the
    /// expanded closed polygons (z=0); free via [`cz_free_zpaths`].
    pub fn cz_propagate_wave(
        seed_xy: *const i32,
        seed_lens: *const i32,
        seed_num: i32,
        bnd_xy: *const i32,
        bnd_lens: *const i32,
        bnd_num: i32,
        initial_step: f64,
        other_step: f64,
        num_other_steps: i32,
        arc_tolerance: f64,
        shortest_edge_length: f64,
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
    /// Faithful `union_safety_offset_ex(Polygons)`: subject safety-offset (+10u
    /// raw, no union) then the two-pass NonZero union; grouped output like
    /// [`cz_union_ex`]. Free via [`cz_free_zpaths`].
    pub fn cz_union_ex_safety(xy: *const i32, lens: *const i32, num: i32) -> CzZPaths;

    /// Faithful `Slic3r::offset(Polyline, delta)`: per-path jtSquare/etOpenButt
    /// ClipperOffset (ShortestEdgeLength=|delta|*0.005) + NonZero union. `xy` =
    /// `n` open-path (x,y) i32 pairs; `delta` scaled. Free via [`cz_free_zpaths`].
    pub fn cz_offset_polyline(xy: *const i32, n: i32, delta: f64, miter_limit: f64) -> CzZPaths;

    pub fn cz_difference_closed(
        subject_xy: *const i32,
        subject_lens: *const i32,
        subject_num: i32,
        clip_xy: *const i32,
        clip_lens: *const i32,
        clip_num: i32,
    ) -> CzZPaths;

    /// `diff_ex(subject, clip, ApplySafetyOffset::Yes)`: safety-offsets (raw +10u
    /// ClipperOffset, jtMiter/ML3, orientation-aware) the clip paths before the
    /// ctDifference. Same output/marshalling as [`cz_difference_closed`].
    pub fn cz_difference_closed_safety(
        subject_xy: *const i32,
        subject_lens: *const i32,
        subject_num: i32,
        clip_xy: *const i32,
        clip_lens: *const i32,
        clip_num: i32,
    ) -> CzZPaths;

    /// Faithful closed-path boolean INTERSECTION (subject ∩ clip), mirroring
    /// `intersection_ex(subject, clip)` = `clipper_do<Paths>(ctIntersection, ...,
    /// pftNonZero)` (ClipperUtils.cpp:802). Same marshalling/caller re-union as
    /// [`cz_difference_closed`]. Replaces the `A - (A - B)` double-difference
    /// intersection, which added a near-collinear vertex on some geometries.
    pub fn cz_intersection_closed(
        subject_xy: *const i32,
        subject_lens: *const i32,
        subject_num: i32,
        clip_xy: *const i32,
        clip_lens: *const i32,
        clip_num: i32,
    ) -> CzZPaths;

    /// Faithful `detect_floating_line` Z-clipper (FillFloatingConcentric.cpp:431-475):
    /// runs the ClipperLib_Z Clipper twice on the same inputs (ctIntersection +
    /// ctDifference) under the detect_floating_line ZFillFunction (tags intersection
    /// points with a negative hash of the four edge z-indices, or the common subject
    /// z for a subject-self-intersection). `subject_xyz` = `subject_n` open x,y,z
    /// triples (z = polyline vertex index); `clip_xyz`/`clip_lens`/`clip_num` the
    /// closed floating-area paths (z = per-vertex index >= `subject_idx_range`).
    /// Returns `[diff_paths..., intersect_paths...]`; `*out_num_diff_paths` is the
    /// count of leading ctDifference paths (the rest are the floating ctIntersection
    /// paths). Free via [`cz_free_zpaths`].
    pub fn cz_detect_floating(
        subject_xyz: *const i32,
        subject_n: i32,
        clip_xyz: *const i32,
        clip_lens: *const i32,
        clip_num: i32,
        subject_idx_range: i32,
        out_num_diff_paths: *mut i32,
    ) -> CzZPaths;

    /// Faithful `union_ex(const Polygons&, PolyFillType)` (ClipperUtils.cpp:813-814)
    /// = PolyTreeToExPolygons(clipper_do_polytree(ctUnion, subject, Empty, fill_type)).
    /// The slice-stage F1 union behind make_expolygons. `xy` = flat (x,y) i32 pairs,
    /// `lens` = per-path point counts, `num` = path count. `fill_type`:
    /// 0=EvenOdd,1=NonZero,2=Positive,3=Negative. Output encodes ExPolygon grouping
    /// in each point's z: contour points z=0 (new ExPolygon), hole points z=1
    /// (attach to current contour); paths emitted in PolyTreeToExPolygons order.
    /// Free via [`cz_free_zpaths`].
    pub fn cz_union_ex(xy: *const i32, lens: *const i32, num: i32, fill_type: i32) -> CzZPaths;

    /// R320: faithful `variable_offset_inner_ex` (ClipperUtils.cpp:1390) —
    /// verbatim mitered per-vertex offset + Clipper1 NEGATIVE-fill cleanup.
    /// One ExPolygon (rings flat i32 xy + lens; ring 0 = contour) + per-ring
    /// per-vertex SCALED f32 deltas (flat, ring-major) + miter_limit. Output
    /// grouped like [`cz_union_ex`]. Free via [`cz_free_zpaths`].
    /// R324: verbatim `smooth_compensation_banded` (ElephantFootCompensation.cpp:
    /// 465-532) compiled with native's clang — bit-exact FMA. `xy` = `n` scaled
    /// i32 (x,y) pairs; `comp` = `n` f32 compensation values, modified in place;
    /// `band`/`strength` f32, `num_iterations` count.
    pub fn cz_smooth_compensation_banded(
        xy: *const i32,
        n: i32,
        comp: *mut f32,
        band: f32,
        strength: f32,
        num_iterations: i32,
    );

    pub fn cz_variable_offset_inner_ex(
        xy: *const i32,
        lens: *const i32,
        num: i32,
        deltas: *const f32,
        miter_limit: f64,
    ) -> CzZPaths;

    /// Faithful `offset2_ex(ExPolygons, delta1, delta2)` (ClipperUtils.cpp:581) — the
    /// post-union morphological close in make_expolygons (TriangleMeshSlicer.cpp:1820).
    /// `xy`/`lens`/`is_hole`/`num` are the grouped cz_union_ex output layout (is_hole:
    /// 0=contour starts a new ExPolygon, 1=hole attaches to current). `delta1`/`delta2`
    /// are SCALED (1e5): make_expolygons passes delta1=+scale(r), delta2=-scale(r).
    /// join_type: 0=miter,1=round,2=square; miter_limit=3.0. Output uses the same
    /// grouped z-encoding as [`cz_union_ex`]. Free via [`cz_free_zpaths`].
    pub fn cz_offset2_ex(
        xy: *const i32,
        lens: *const i32,
        is_hole: *const i32,
        num: i32,
        delta1: f64,
        delta2: f64,
        join_type: i32,
        miter_limit: f64,
    ) -> CzZPaths;

    /// Faithful `ClipperLib::SimplifyPolygons` (clipper.hpp:559): ctUnion with
    /// StrictlySimple(true). The ExPolygon::simplify_p post-DP step. Returns flat
    /// Paths (z=0); caller re-unions into ExPolygons. fill_type: 0..3 as above.
    pub fn cz_simplify_polygons(xy: *const i32, lens: *const i32, num: i32, fill_type: i32) -> CzZPaths;
    pub fn cz_union_flat(xy: *const i32, lens: *const i32, num: i32) -> CzZPaths;

    /// `intersection(subject, clip, ApplySafetyOffset::Yes)` — ctIntersection
    /// with safety_offset(clip) (ClipperUtils.cpp:334).
    pub fn cz_intersection_closed_safety(
        subject_xy: *const i32,
        subject_lens: *const i32,
        subject_num: i32,
        clip_xy: *const i32,
        clip_lens: *const i32,
        clip_num: i32,
    ) -> CzZPaths;

    /// Free a [`CzZPaths`] returned by [`cz_clip_extrusion`].
    pub fn cz_free_zpaths(paths: CzZPaths);

    /// R269 medial-axis shim smoke test: boost voronoi over a unit square,
    /// returns the count of finite primary edges (links the boost builder).
    pub fn ma_selftest() -> i64;

    /// R273: faithful `ExPolygon::medial_axis` (boost voronoi + verbatim
    /// MedialAxis walk + ExPolygon.cpp post-processing). Inputs are SCALED
    /// i64 (x,y) pairs; `min_width`/`max_width` scaled doubles. Free the
    /// result via [`ma_free`].
    pub fn ma_build(
        contour_xy: *const i64,
        contour_n: i32,
        holes_xy: *const i64,
        hole_lens: *const i32,
        hole_num: i32,
        min_width: f64,
        max_width: f64,
    ) -> MaBuildResult;

    /// Free a [`MaBuildResult`] returned by [`ma_build`].
    pub fn ma_free(res: MaBuildResult);
}

/// Mirror of `MaBuildResult` in medial_axis_shim.cpp. Flat layout:
/// `coords` = 2*total_points i64 (x,y per point), `widths` = total_widths f64
/// (2*(points-1) per polyline), `pl_sizes` = per-polyline point counts,
/// `endpoints` = 2 flags per polyline (first, second).
#[repr(C)]
pub struct MaBuildResult {
    pub coords: *mut i64,
    pub widths: *mut f64,
    pub pl_sizes: *mut i32,
    pub endpoints: *mut u8,
    pub num_polylines: i32,
    pub total_points: i32,
    pub total_widths: i32,
}

/// One thick polyline returned by [`medial_axis_native`].
pub struct MaThickPolyline {
    pub points: Vec<(i64, i64)>,
    pub width: Vec<f64>,
    pub endpoints: (bool, bool),
}

/// Safe wrapper over [`ma_build`]/[`ma_free`]: faithful ExPolygon::medial_axis
/// on a scaled-i64 expolygon (contour + holes as (x,y) point lists).
pub fn medial_axis_native(
    contour: &[(i64, i64)],
    holes: &[Vec<(i64, i64)>],
    min_width: f64,
    max_width: f64,
) -> Vec<MaThickPolyline> {
    let contour_flat: Vec<i64> = contour.iter().flat_map(|&(x, y)| [x, y]).collect();
    let holes_flat: Vec<i64> = holes
        .iter()
        .flat_map(|h| h.iter().flat_map(|&(x, y)| [x, y]))
        .collect();
    let hole_lens: Vec<i32> = holes.iter().map(|h| h.len() as i32).collect();
    let raw = unsafe {
        ma_build(
            contour_flat.as_ptr(),
            contour.len() as i32,
            holes_flat.as_ptr(),
            hole_lens.as_ptr(),
            holes.len() as i32,
            min_width,
            max_width,
        )
    };
    let mut out = Vec::with_capacity(raw.num_polylines as usize);
    unsafe {
        let mut ci = 0isize;
        let mut wi = 0isize;
        for k in 0..raw.num_polylines as isize {
            let n = *raw.pl_sizes.offset(k) as isize;
            let mut points = Vec::with_capacity(n as usize);
            for _ in 0..n {
                points.push((*raw.coords.offset(2 * ci), *raw.coords.offset(2 * ci + 1)));
                ci += 1;
            }
            let nw = if n > 1 { 2 * (n - 1) } else { 0 };
            let mut width = Vec::with_capacity(nw as usize);
            for _ in 0..nw {
                width.push(*raw.widths.offset(wi));
                wi += 1;
            }
            let endpoints = (
                *raw.endpoints.offset(2 * k) != 0,
                *raw.endpoints.offset(2 * k + 1) != 0,
            );
            out.push(MaThickPolyline { points, width, endpoints });
        }
        ma_free(raw);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::ffi::CStr;

    #[test]
    fn ma_build_thin_rect() {
        // 10mm x 0.4mm rectangle at scale 1e5 -> expect a single medial-axis
        // spine along Y=20000 with widths ~40000, endpoints extended.
        let contour = [
            (0i64, 0i64),
            (1_000_000, 0),
            (1_000_000, 40_000),
            (0, 40_000),
        ];
        let pls = medial_axis_native(&contour, &[], 20_000.0, 80_000.0);
        assert!(!pls.is_empty(), "no polylines returned");
        let pl = &pls[0];
        assert!(pl.points.len() >= 2);
        assert_eq!(pl.width.len(), 2 * (pl.points.len() - 1));
        for &w in &pl.width {
            assert!((w - 40_000.0).abs() < 2_000.0, "width {w} far from 40000");
        }
        for &(_, y) in &pl.points {
            assert!((y - 20_000).abs() <= 1, "spine y {y} not centered");
        }
    }

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
    #[test]
    fn replay_smooth() {
        let path = match std::env::var("SMREPLAY") { Ok(p) => p, Err(_) => return };
        let txt = std::fs::read_to_string(&path).unwrap();
        let mut lines = txt.lines();
        let hdr = lines.next().unwrap();
        // "n <N> band <B> strength <S> iters <I>"
        let toks: Vec<&str> = hdr.split_whitespace().collect();
        let n: usize = toks[1].parse().unwrap();
        let band: f32 = toks[3].parse().unwrap();
        let strength: f32 = toks[5].parse().unwrap();
        let iters: i32 = toks[7].parse().unwrap();
        let mut xy: Vec<i32> = Vec::with_capacity(n * 2);
        for _ in 0..n {
            let l = lines.next().unwrap();
            let mut it = l.split_whitespace();
            xy.push(it.next().unwrap().parse::<i64>().unwrap() as i32);
            xy.push(it.next().unwrap().parse::<i64>().unwrap() as i32);
        }
        assert_eq!(lines.next().unwrap(), "comp");
        let mut comp: Vec<f32> = Vec::with_capacity(n);
        for _ in 0..n {
            comp.push(f32::from_bits(u32::from_str_radix(lines.next().unwrap().trim(), 16).unwrap()));
        }
        unsafe {
            cz_smooth_compensation_banded(xy.as_ptr(), n as i32, comp.as_mut_ptr(), band, strength, iters);
        }
        let mut h: u64 = 1469598103934665603;
        for &d in comp.iter() { h ^= d.to_bits() as u64; h = h.wrapping_mul(1099511628211); }
        eprintln!("SMREPLAY rust shim outhash={:016x}", h);
    }

    #[test]
    fn replay_varoff() {
        let path = match std::env::var("VOREPLAY") {
            Ok(p) => p,
            Err(_) => return,
        };
        let txt = std::fs::read_to_string(&path).unwrap();
        let mut lines = txt.lines();
        let ncontours: usize = lines.next().unwrap().strip_prefix("ncontours ").unwrap().parse().unwrap();
        let mut xy: Vec<i32> = Vec::new();
        let mut lens: Vec<i32> = Vec::new();
        let mut deltas: Vec<f32> = Vec::new();
        let mut read_ring = |hdr_pt: &str, hdr_d: &str, lines: &mut std::str::Lines| {
            let n: usize = lines.next().unwrap().strip_prefix(hdr_pt).unwrap().parse().unwrap();
            lens.push(n as i32);
            for _ in 0..n {
                let l = lines.next().unwrap();
                let mut it = l.split_whitespace();
                xy.push(it.next().unwrap().parse::<i64>().unwrap() as i32);
                xy.push(it.next().unwrap().parse::<i64>().unwrap() as i32);
            }
            let nd: usize = lines.next().unwrap().strip_prefix(hdr_d).unwrap().parse().unwrap();
            for _ in 0..nd {
                deltas.push(f32::from_bits(u32::from_str_radix(lines.next().unwrap().trim(), 16).unwrap()));
            }
        };
        read_ring("contour ", "deltas ", &mut lines);
        for _ in 1..ncontours {
            read_ring("hole ", "hdeltas ", &mut lines);
        }
        let raw = unsafe {
            cz_variable_offset_inner_ex(xy.as_ptr(), lens.as_ptr(), lens.len() as i32, deltas.as_ptr(), 2.0)
        };
        let paths = collect_and_free(raw);
        let np: usize = paths.iter().map(|p| p.len()).sum();
        eprintln!("VOREPLAY rust: vout_paths={} vout_np={}", paths.len(), np);
    }

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
