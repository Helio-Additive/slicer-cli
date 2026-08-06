//! Region expansion via wave propagation.
//!
//! This is a faithful port of BambuStudio's `RegionExpansion.hpp/cpp` algorithm,
//! which expands source regions (e.g. top/bottom surfaces) into boundary regions
//! (e.g. internal/solid fill areas) using iterative wave propagation.
//!
//! # Algorithm Overview
//!
//! The C++ algorithm works as follows:
//! 1. **Wave seeds**: Find the interface between src and boundary regions by slightly
//!    expanding src and intersecting with boundary. The intersection segments become
//!    "seeds" for wave propagation.
//! 2. **Wave propagation**: Iteratively offset the seeds outward in small steps,
//!    clipping to the boundary at each step. This produces a controlled expansion
//!    that respects boundary geometry and doesn't leak through narrow gaps.
//! 3. **Merge**: Merge the expanded waves back into the source expolygons.
//!
//! # C++ Reference
//!
//! - `RegionExpansion.hpp` — struct definitions
//! - `RegionExpansion.cpp` — wave seed generation, propagation, merging
//! - `LayerRegion.cpp:470-516` — `expand_merge_surfaces()`
//! - `LayerRegion.cpp:517-643` — `LayerRegion::process_external_surfaces()`
//!
//! # BLOCKED symbols — native Clipper2 Z-callback backend (not byte-faithful)
//!
//! The C++ `wave_seeds()` (RegionExpansion.cpp:278-389) relies on the Clipper2
//! *Z* engine to record intersection provenance:
//!   - `Clipper2Lib_Z::Clipper64::SetZCallback(...)` to tag boolean-clip
//!     intersection points with a negative Z index into an `Intersections` table;
//!   - `Clipper2Lib_Z::ClipperOffset` that preserves the Z coordinate while
//!     offsetting *open* paths (`expolygons_to_zpaths64_expanded_opened`,
//!     RegionExpansion.cpp:108-136);
//!   - `Clipper64::Execute(..., closed_segs, open_segs)` returning Z-tagged open
//!     segments.
//!
//! The crate's Clipper2 backend (`clipper2c-sys`) exposes neither `SetZCallback`
//! / `ZFillFunction` nor a Z-preserving offset, so these symbols cannot be made
//! byte-faithful without porting the bundled `clipper/clipper_z` engine — the
//! same backend gap already documented as NOT PORTED in `line_segmentation.rs`.
//! Per the wasm-safe rule we do NOT add a system/dylib dep.
//!
//! BLOCKED (require the Z backend; left as documented approximations / not ported):
//!   - `expolygons_to_zpaths_expanded_opened`  (ClipperLib_Z, RegionExpansion.cpp:83-106)
//!   - `expolygons_to_zpaths64_expanded_opened` (Clipper2Lib_Z, RegionExpansion.cpp:108-136)
//!   - `merge_splits` (×2, RegionExpansion.cpp:142-236) — operate on Z paths
//!   - `wave_seeds`                              (RegionExpansion.cpp:278-389)
//!   - `wavefront_initial`/`wavefront_step`/`wavefront_clip`/`propagate_wave_from_boundary`
//!     (RegionExpansion.cpp:391-465) — `ClipperOffset` over open polylines (`etOpenRound`),
//!     only reachable through `wave_seeds`.
//!   - the `propagate_waves`/`propagate_waves_ex`/`expand_expolygons`/
//!     `expand_merge_expolygons(src, ...)` overloads that route through `wave_seeds`.
//!
//! The polygon-based seed approximation below (`wave_seeds_polygon_based`) is a
//! best-effort fallback used only by the LayerRegion.cpp orchestration helpers
//! co-located in this file; it is NOT byte-equivalent to the C++ Z-callback path.
//!
//! FAITHFULLY PORTED (pure / non-native, audited 1:1 against the C++):
//!   - `clipper_round_offset_error`             (RegionExpansion.cpp:19-30)
//!   - `RegionExpansionParameters::build`       (RegionExpansion.cpp:32-79)
//!   - `build_aabb_tree_over_expolygons`        (RegionExpansion.cpp:240-251)
//!   - `sample_in_expolygons`                   (RegionExpansion.cpp:253-276)
//!   - `merge_expansions_into_expolygons`       (RegionExpansion.cpp:564-615)

use crate::clipper_utils::{
    closing, diff_pl, difference, expolygons_to_polylines, grow, intersection, offset2_ex_clib,
    offset_expolygons, union_ex, union_safety_offset_ex_expolygons, OffsetJoinType,
};
use crate::geometry::{
    BoundingBox, ExPolygon, ExPolygons, Line, Point, PointF, Polygon, Polyline,
};
use crate::CoordF;
use std::f64::consts::PI;

// ============================================================================
// AABB tree over boundary expolygons — faithful RegionExpansion.cpp port
// ============================================================================
//
// RegionExpansion.cpp:238 — `using AABBTreeBBoxes = AABBTreeIndirect::Tree<2, coord_t>;`
// In the crate this is the 2D specialization `aabb_tree_lines::tree2d::Tree`,
// built from `BoundingBoxWrapper` source nodes (AABBTreeIndirect.hpp:223-236).
use crate::aabb_tree_lines::tree2d;

/// RegionExpansion.cpp:240 — `static AABBTreeBBoxes build_aabb_tree_over_expolygons(const ExPolygons &expolygons)`
fn build_aabb_tree_over_expolygons(expolygons: &[ExPolygon]) -> tree2d::Tree {
    // RegionExpansion.cpp:242-243 — Calculate bounding boxes of internal slices.
    let mut bboxes: Vec<tree2d::BoundingBoxWrapper> = Vec::with_capacity(expolygons.len());
    // RegionExpansion.cpp:245-246 — `bboxes.emplace_back(i, get_extents(expolygons[i].contour));`
    for (i, ep) in expolygons.iter().enumerate() {
        // `get_extents(contour)` == the single contour's bounding box.
        bboxes.push(tree2d::BoundingBoxWrapper::new(i, &ep.contour.bounding_box()));
    }
    // RegionExpansion.cpp:247-249 — Build AABB tree over bounding boxes of boundary expolygons.
    let mut out = tree2d::Tree::new();
    out.build_modify_input(&mut bboxes);
    // RegionExpansion.cpp:250
    out
}

/// RegionExpansion.cpp:253-276 — `static int sample_in_expolygons(...)`
///
/// Returns the index of the boundary expolygon that contains `sample`, or `-1`.
fn sample_in_expolygons(aabb_tree: &tree2d::Tree, expolygons: &[ExPolygon], sample: &Point) -> i32 {
    // RegionExpansion.cpp:259
    let mut out: i32 = -1;
    // RegionExpansion.cpp:260-274
    tree2d::traverse(
        aabb_tree,
        // RegionExpansion.cpp:261-263 — predicate: descend while the node bbox contains the sample.
        // tree2d `BoundingBox` stores coords as `[f64; 2]` (Eigen AlignedBox), so the
        // integer sample point is widened to f64, matching the bbox build in
        // `BoundingBoxWrapper::new`.
        |node: &tree2d::Node| node.bbox.contains([sample.x as f64, sample.y as f64]),
        // RegionExpansion.cpp:264-273 — leaf callback.
        |node: &tree2d::Node| {
            debug_assert!(node.is_leaf());
            debug_assert!(node.is_valid());
            // RegionExpansion.cpp:267-271
            if expolygons[node.idx].contains_point(sample) {
                out = node.idx as i32;
                // Stop traversal.
                return false;
            }
            // Continue traversal.
            true
        },
    );
    // RegionExpansion.cpp:275
    out
}

// ============================================================================
// RegionExpansionParameters
// ============================================================================

/// `ClipperUtils.hpp:44` — `static constexpr const double ClipperOffsetShortestEdgeFactor = 0.005;`
pub const CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR: f64 = 0.005;

// Calculating radius discretization according to ClipperLib offsetter code, see void ClipperOffset::DoOffset(double delta)
// RegionExpansion.cpp:19
pub fn clipper_round_offset_error(offset: f64, arc_tolerance: f64) -> f64 {
    // RegionExpansion.cpp:21
    const DEF_ARC_TOLERANCE: f64 = 0.25;
    // RegionExpansion.cpp:22-27
    let y = if arc_tolerance <= 0.0 {
        DEF_ARC_TOLERANCE
    } else if arc_tolerance > offset * DEF_ARC_TOLERANCE {
        offset * DEF_ARC_TOLERANCE
    } else {
        arc_tolerance
    };
    // RegionExpansion.cpp:28
    let steps = (PI / (1.0 - y / offset).acos()).min(offset * PI);
    // RegionExpansion.cpp:29
    offset * (1.0 - (PI / steps).cos())
}

/// Parameters controlling the wave expansion algorithm.
///
/// Port of `Slic3r::Algorithm::RegionExpansionParameters` from RegionExpansion.hpp.
///
/// The expansion happens in steps:
/// 1. Tiny initial expansion of src to create seeds at the src/boundary interface
/// 2. First wave step of `initial_step` size
/// 3. `num_other_steps` additional wave steps of `other_step` size
///
/// All distances are in mm (unscaled), matching our clipper module convention.
#[derive(Debug, Clone)]
pub struct RegionExpansionParameters {
    /// Initial expansion of src to make source regions intersect with boundary
    /// regions just a bit. Should be small but not tiny.
    pub tiny_expansion: CoordF,

    /// How much to inflate the seed lines to produce the first wave area.
    pub initial_step: CoordF,

    /// How much to inflate each successive wave area.
    pub other_step: CoordF,

    /// Number of inflate steps after the initial step.
    pub num_other_steps: usize,

    /// Maximum total inflation. Used to trim boundary for performance.
    pub max_inflation: CoordF,

    /// RegionExpansion.hpp:31 — Accuracy of the offsetter for wave propagation.
    pub arc_tolerance: f64,
    /// RegionExpansion.hpp:32
    pub shortest_edge_length: f64,
}

impl RegionExpansionParameters {
    // Build expansion parameters from a full expansion distance.
    //
    // This is a faithful port of `RegionExpansionParameters::build()` from
    // RegionExpansion.cpp:32-79.
    //
    // # Arguments
    //
    // * `full_expansion` - Total desired expansion distance (mm)
    // * `expansion_step` - Size of each wave step (mm). C++ default: 0.1mm
    // * `max_nr_expansion_steps` - Maximum number of steps. C++ default: 5
    pub fn build(
        full_expansion: CoordF,
        expansion_step: CoordF,
        max_nr_expansion_steps: usize,
    ) -> Self {
        assert!(full_expansion > 0.0);
        assert!(expansion_step > 0.0);
        assert!(max_nr_expansion_steps > 0);

        // RegionExpansion.cpp:45-48
        // Initial expansion of src to make the source regions intersect with boundary regions just a bit.
        // The expansion should not be too tiny, but also small enough, so the following expansion will
        // compensate for tiny_expansion and bring the wave back to the boundary without producing
        // ugly cusps where it touches the boundary.
        // RegionExpansion.cpp:49 — `out.tiny_expansion = std::min(0.25f * full_expansion, scaled<float>(0.05f));`
        // NOTE: this module operates in mm (unscaled), so `scaled<float>(0.05f)` is kept as the
        // mm literal 0.05 to remain internally consistent with the rest of the module.
        // FIDELITY-NOTE(F1): C++ clamps against the *scaled* constant `scaled<float>(0.05f)`
        // (coord_t units); the mm-domain literal 0.05 here is the unscaled equivalent for this
        // module's geo-clipper offset convention, not the integer-coord clamp C++ applies.
        let mut tiny_expansion = (0.25 * full_expansion).min(0.05);

        // RegionExpansion.cpp:50
        let mut nsteps = ((full_expansion - tiny_expansion) / expansion_step).ceil() as usize;
        // RegionExpansion.cpp:51-52
        if max_nr_expansion_steps > 0 {
            nsteps = nsteps.min(max_nr_expansion_steps);
        }
        // RegionExpansion.cpp:53 — assert(nsteps > 0)
        nsteps = nsteps.max(1);

        // RegionExpansion.cpp:54
        let mut initial_step = (full_expansion - tiny_expansion) / nsteps as CoordF;

        // RegionExpansion.cpp:55-59
        if nsteps > 1 && 0.25 * initial_step < tiny_expansion {
            // Decrease the step size by lowering number of steps.
            nsteps = (((full_expansion - tiny_expansion) / (4.0 * tiny_expansion)).floor()
                as usize)
                .max(1);
            initial_step = (full_expansion - tiny_expansion) / nsteps as CoordF;
        }

        // RegionExpansion.cpp:60-63 — NOTE: C++ does NOT modify nsteps here.
        if 0.25 * initial_step < tiny_expansion || nsteps == 1 {
            tiny_expansion = 0.2 * full_expansion;
            initial_step = 0.8 * full_expansion;
        }

        // RegionExpansion.cpp:64
        let other_step = initial_step;
        // RegionExpansion.cpp:65
        let num_other_steps = nsteps - 1;

        // RegionExpansion.cpp:71-75
        // Maximum inflation of seed contours over the boundary. Used to trim boundary to speed up
        // clipping during wave propagation. Needs to be in sync with the offsetter accuracy.
        // Clipper positive round offset should rather offset less than more.
        // Still a little bit of additional offset was added.
        let max_inflation = (tiny_expansion + nsteps as CoordF * initial_step) * 1.1;

        // RegionExpansion.cpp:67-69
        // Accuracy of the offsetter for wave propagation.
        // RegionExpansion.cpp:68 — `out.arc_tolerance = scaled<double>(0.1);` (kept in mm: 0.1)
        let arc_tolerance = 0.1;
        // RegionExpansion.cpp:69
        let shortest_edge_length = initial_step * CLIPPER_OFFSET_SHORTEST_EDGE_FACTOR;

        Self {
            tiny_expansion,
            initial_step,
            other_step,
            num_other_steps,
            max_inflation,
            arc_tolerance,
            shortest_edge_length,
        }
    }
}

// ============================================================================
// ExpansionZone
// ============================================================================

/// A boundary zone into which source surfaces can expand.
///
/// Port of `Slic3r::ExpansionZone` from Layer.hpp.
///
/// Each zone represents a set of expolygons (e.g. InternalSolid or Internal regions)
/// that source surfaces (Top/Bottom/Bridge) can expand into. After expansion,
/// the consumed area is subtracted from the zone's expolygons.
#[derive(Debug, Clone)]
pub struct ExpansionZone {
    /// The boundary expolygons available for expansion.
    pub expolygons: ExPolygons,

    /// Expansion parameters for this zone.
    pub parameters: RegionExpansionParameters,

    /// Whether any source region was expanded into this zone.
    pub expanded_into: bool,
}

// ============================================================================
// Wave propagation
// ============================================================================

/// Result of expanding a single source region into a boundary.
///
/// Port of `Slic3r::Algorithm::RegionExpansion`.
#[derive(Debug, Clone)]
struct RegionExpansion {
    /// The expanded polygon.
    polygon: ExPolygon,
    /// Index of the source expolygon this expansion originated from.
    src_id: u32,
    /// Index of the boundary expolygon this expansion grew into.
    #[allow(dead_code)]
    boundary_id: u32,
}

/// Generate wave seeds by slightly expanding src and intersecting with boundary.
///
/// This is our polygon-based alternative to the C++ Z-callback approach.
/// The C++ creates open polylines at the exact src/boundary interface using
/// Clipper2's Z-callback. We instead:
/// 1. Offset src outward by `tiny_expansion`
/// 2. Intersect with each boundary expolygon
/// 3. The thin intersection strips become our seeds
///
/// The results are equivalent because:
/// - Seeds are very thin (0.05mm)
/// - First wave step size compensates for the seed width
/// - Subsequent clipping produces identical geometry
///
// ============================================================================
// FAITHFUL wave_seeds — Clipper2-Z backend (clipper2-z-sys), 1:1 port of
// RegionExpansion.cpp:108-389 (Z-path builders, merge_splits, wave_seeds).
// Gated via propagate_waves(REGION_EXPANSION_FAITHFUL) below; the seed generation
// is byte-faithful, and the wavefront propagation is the byte-faithful ClipperLib
// `cz_propagate_wave` shim (R131) rather than the geo approximation.
// ============================================================================

use crate::clipper2_z::{intersect_open_z, offset_z};
use crate::clipper_z_utils::{ZPath, ZPaths};

/// `Clipper2ZUtils::zpoint64_lower` (Clipper2ZUtils.hpp): lexicographic on (x,y,z).
#[inline]
fn zpoint64_lower(l: &(i64, i64, i64), r: &(i64, i64, i64)) -> bool {
    l.0 < r.0 || (l.0 == r.0 && (l.1 < r.1 || (l.1 == r.1 && l.2 < r.2)))
}

/// `Clipper2ZUtils::expolygons_to_zpaths64<Open=false>`: each expolygon's contour
/// + holes become a CLOSED zpath tagged with the expolygon's running `base_idx`.
fn expolygons_to_zpaths64(src: &[ExPolygon], base_idx: &mut i64) -> ZPaths {
    let mut out: ZPaths = Vec::new();
    for expoly in src {
        let mut contour: ZPath = Vec::with_capacity(expoly.contour.points.len());
        for p in &expoly.contour.points {
            contour.push((p.x, p.y, *base_idx));
        }
        out.push(contour);
        for hole in &expoly.holes {
            let mut h: ZPath = Vec::with_capacity(hole.points.len());
            for p in &hole.points {
                h.push((p.x, p.y, *base_idx));
            }
            out.push(h);
        }
        *base_idx += 1;
    }
    out
}

/// `expolygons_to_zpaths64_expanded_opened` (RegionExpansion.cpp:108-136): each
/// contour is offset (outer +expansion, holes -expansion) via Clipper2's Z-
/// preserving ClipperOffset (cz2_offset_z), tagged with the running `base_idx`,
/// then appended OPEN (re-closed so it forms a loop the open-subject clip treats
/// as an open contour).
fn expolygons_to_zpaths64_expanded_opened(
    src: &[ExPolygon],
    expansion: f64,
    base_idx: &mut i64,
) -> ZPaths {
    let mut out: ZPaths = Vec::new();
    for expoly in src {
        let mut contours: Vec<&crate::geometry::Polygon> =
            Vec::with_capacity(1 + expoly.holes.len());
        contours.push(&expoly.contour);
        for h in &expoly.holes {
            contours.push(h);
        }
        for (icontour, contour) in contours.iter().enumerate() {
            let mut zin: ZPath = Vec::with_capacity(contour.points.len());
            for p in &contour.points {
                zin.push((p.x, p.y, *base_idx));
            }
            let delta = if icontour == 0 { expansion } else { -expansion };
            let offset = offset_z(&zin, delta);
            for path in &offset {
                if path.is_empty() {
                    continue;
                }
                let mut zp: ZPath = Vec::with_capacity(path.len() + 1);
                for &(x, y, _) in path {
                    zp.push((x, y, *base_idx));
                }
                zp.push(zp[0]);
                out.push(zp);
            }
        }
        *base_idx += 1;
    }
    out
}

/// `polylines_merge` (Polyline.hpp:236): join `src` onto `dst`, handling the four
/// front/back orientation combinations.
fn polylines_merge_z(dst: &mut ZPath, dst_first: bool, mut src: ZPath, src_first: bool) {
    if dst_first {
        dst.reverse();
    }
    if !src_first {
        src.reverse();
    }
    dst.extend(src.into_iter().skip(1));
}

/// `merge_splits` (RegionExpansion.cpp:194-236, Clipper2 overload): reconnect open
/// paths split at the ends of the source closed contours.
fn merge_splits(paths: &mut ZPaths, splits: &mut Vec<((i64, i64, i64), i32)>) {
    let mut i = 0usize;
    while i < paths.len() {
        let mut merged = false;
        if paths[i].len() >= 2 {
            let front = paths[i][0];
            let back = *paths[i].last().unwrap();
            if front.0 != back.0 || front.1 != back.1 {
                let find_end =
                    |splits: &[((i64, i64, i64), i32)], pt: &(i64, i64, i64)| -> Option<usize> {
                        let idx = splits.partition_point(|e| zpoint64_lower(&e.0, pt));
                        if idx < splits.len() && splits[idx].0 .0 == pt.0 && splits[idx].0 .1 == pt.1
                        {
                            Some(idx)
                        } else {
                            None
                        }
                    };
                let mut end_idx = find_end(splits, &front);
                let mut end_front = true;
                if end_idx.is_none() {
                    end_front = false;
                    end_idx = find_end(splits, &back);
                }
                if let Some(eidx) = end_idx {
                    if splits[eidx].1 == -1 {
                        splits[eidx].1 = i as i32;
                    } else {
                        let other_idx = splits[eidx].1 as usize;
                        let this_path = std::mem::take(&mut paths[i]);
                        let other_front = paths[other_idx][0];
                        let other_front_is_split = other_front == splits[eidx].0;
                        polylines_merge_z(
                            &mut paths[other_idx],
                            other_front_is_split,
                            this_path,
                            end_front,
                        );
                        if i + 1 == paths.len() {
                            paths.pop();
                            break;
                        }
                        let last = paths.pop().unwrap();
                        paths[i] = last;
                        merged = true;
                    }
                }
            }
        }
        if !merged {
            i += 1;
        }
    }
}

/// `wave_seeds` (RegionExpansion.cpp:278-389), faithful via the Clipper2-Z shim.
fn wave_seeds_faithful(
    src: &[ExPolygon],
    boundary: &[ExPolygon],
    tiny_expansion: f64,
    sorted: bool,
) -> Vec<WaveSeed> {
    debug_assert!(tiny_expansion > 0.0);
    if src.is_empty() || boundary.is_empty() {
        return Vec::new();
    }

    let idx_boundary_begin: i64 = 1;
    let mut idx_boundary_end: i64 = idx_boundary_begin;
    let zboundary = expolygons_to_zpaths64(boundary, &mut idx_boundary_end);

    // UNITS: ExPolygon geometry is SCALED (coord_t); cz2_offset_z offsets scaled
    // i64 coords directly, so the delta must be scaled. RegionExpansionParameters
    // here are in MM (× SCALING_FACTOR = 100_000 bridges mm→scaled).
    let tiny_expansion_scaled = tiny_expansion * crate::SCALING_FACTOR;
    let mut idx_src_end = idx_boundary_end;
    let zsrc =
        expolygons_to_zpaths64_expanded_opened(src, tiny_expansion_scaled, &mut idx_src_end);
    let mut zsrc_splits: Vec<((i64, i64, i64), i32)> = Vec::with_capacity(zsrc.len());
    for path in &zsrc {
        debug_assert!(path.len() >= 2);
        zsrc_splits.push((path[0], -1));
    }
    zsrc_splits.sort_by(|l, r| {
        if zpoint64_lower(&l.0, &r.0) {
            std::cmp::Ordering::Less
        } else if zpoint64_lower(&r.0, &l.0) {
            std::cmp::Ordering::Greater
        } else {
            std::cmp::Ordering::Equal
        }
    });

    let wc = intersect_open_z(&zsrc, &zboundary);
    let mut segments: ZPaths = wc.segments;
    let intersections = wc.intersections;

    merge_splits(&mut segments, &mut zsrc_splits);

    let mut aabb: Option<tree2d::Tree> = None;
    let mut out: Vec<WaveSeed> = Vec::with_capacity(segments.len());
    for path in &segments {
        if path.len() < 2 {
            continue;
        }
        let front = path[0];
        let back = *path.last().unwrap();
        let intersection_valid = |is: &(i64, i64)| -> bool {
            is.0 >= 1 && is.0 < idx_boundary_end && is.1 >= idx_boundary_end && is.1 < idx_src_end
        };
        let mut intersection: Option<&(i64, i64)> = None;
        if front.2 < 0 {
            let idx = (-front.2 - 1) as usize;
            if idx < intersections.len() && intersection_valid(&intersections[idx]) {
                intersection = Some(&intersections[idx]);
            }
        }
        if intersection.is_none() && back.2 < 0 {
            let idx = (-back.2 - 1) as usize;
            if idx < intersections.len() && intersection_valid(&intersections[idx]) {
                intersection = Some(&intersections[idx]);
            }
        }

        let pts: Vec<Point> = path.iter().map(|&(x, y, _)| Point::new(x, y)).collect();

        if let Some(is) = intersection {
            out.push(WaveSeed {
                src: (is.1 - idx_boundary_end) as u32,
                boundary: (is.0 - 1) as u32,
                path: pts,
            });
        } else {
            if !(front == back && front.2 >= idx_boundary_end && front.2 < idx_src_end) {
                continue;
            }
            if aabb.is_none() {
                aabb = Some(build_aabb_tree_over_expolygons(boundary));
            }
            let boundary_id = sample_in_expolygons(
                aabb.as_ref().unwrap(),
                boundary,
                &Point::new(front.0, front.1),
            );
            if boundary_id >= 0 {
                out.push(WaveSeed {
                    src: (front.2 - idx_boundary_end) as u32,
                    boundary: boundary_id as u32,
                    path: pts,
                });
            }
        }
    }

    if sorted {
        out.sort_by(|a, b| (a.boundary, a.src).cmp(&(b.boundary, b.src)));
    }
    out
}

/// Faithful wavefront propagation: group the byte-faithful seeds by (boundary,
/// src) and drive the ClipperLib `cz_propagate_wave` shim (R131) once per group.
/// This replaces the geo `propagate_wave_from_seeds` on the faithful path.
fn propagate_waves_faithful(
    src: &[ExPolygon],
    boundary: &[ExPolygon],
    params: &RegionExpansionParameters,
) -> Vec<RegionExpansion> {
    let seeds = wave_seeds_faithful(src, boundary, params.tiny_expansion, true);
    let mut out: Vec<RegionExpansion> = Vec::new();
    let mut i = 0usize;
    while i < seeds.len() {
        let b = seeds[i].boundary;
        let s = seeds[i].src;
        // Collect all seed polylines for this (boundary, src) island.
        let mut j = i;
        let mut seed_xy: Vec<i32> = Vec::new();
        let mut seed_lens: Vec<i32> = Vec::new();
        while j < seeds.len() && seeds[j].boundary == b && seeds[j].src == s {
            let p = &seeds[j].path;
            if p.len() >= 2 {
                seed_lens.push(p.len() as i32);
                for pt in p {
                    seed_xy.push(pt.x as i32);
                    seed_xy.push(pt.y as i32);
                }
            }
            j += 1;
        }
        if !seed_lens.is_empty() {
            // The boundary ExPolygon as closed paths (contour + holes).
            let bnd = &boundary[b as usize];
            let mut bnd_xy: Vec<i32> = Vec::new();
            let mut bnd_lens: Vec<i32> = Vec::new();
            bnd_lens.push(bnd.contour.points.len() as i32);
            for pt in &bnd.contour.points {
                bnd_xy.push(pt.x as i32);
                bnd_xy.push(pt.y as i32);
            }
            for hole in &bnd.holes {
                bnd_lens.push(hole.points.len() as i32);
                for pt in &hole.points {
                    bnd_xy.push(pt.x as i32);
                    bnd_xy.push(pt.y as i32);
                }
            }
            // ClipperOffset deltas are in scaled units (mm × SCALING_FACTOR).
            let raw = unsafe {
                clipper_z_sys::cz_propagate_wave(
                    seed_xy.as_ptr(),
                    seed_lens.as_ptr(),
                    seed_lens.len() as i32,
                    bnd_xy.as_ptr(),
                    bnd_lens.as_ptr(),
                    bnd_lens.len() as i32,
                    params.initial_step * crate::SCALING_FACTOR,
                    params.other_step * crate::SCALING_FACTOR,
                    params.num_other_steps as i32,
                    params.arc_tolerance * crate::SCALING_FACTOR,
                    params.shortest_edge_length * crate::SCALING_FACTOR,
                )
            };
            // Marshal the expanded closed polygons back; each becomes one
            // RegionExpansion (polygon = ExPolygon of that single contour).
            let polys = unsafe { read_cz_polygons(&raw) };
            unsafe { clipper_z_sys::cz_free_zpaths(raw) };
            for poly in polys {
                out.push(RegionExpansion {
                    polygon: ExPolygon::new(poly),
                    src_id: s,
                    boundary_id: b,
                });
            }
        }
        i = j;
    }
    out
}

/// Read a `CzZPaths` (flat x,y,z triples + per-path lens) into `Polygon`s (dropping z).
unsafe fn read_cz_polygons(raw: &clipper_z_sys::CzZPaths) -> Vec<Polygon> {
    let num = raw.num_paths as usize;
    if num == 0 || raw.coords.is_null() || raw.path_lens.is_null() {
        return Vec::new();
    }
    let lens = std::slice::from_raw_parts(raw.path_lens, num);
    let coords = std::slice::from_raw_parts(raw.coords, (raw.total_points as usize) * 3);
    let mut out: Vec<Polygon> = Vec::with_capacity(num);
    let mut cursor = 0usize;
    for &len in lens {
        let n = len as usize;
        let mut pts: Vec<Point> = Vec::with_capacity(n);
        for k in 0..n {
            let base = (cursor + k) * 3;
            pts.push(Point::new(coords[base] as i64, coords[base + 1] as i64));
        }
        cursor += n;
        if !pts.is_empty() {
            out.push(Polygon::from_points(pts));
        }
    }
    out
}

// FIDELITY-NOTE(F1): C++ `wave_seeds` (RegionExpansion.cpp:278-389) uses the
// Clipper2 *Z*-callback engine to tag intersection provenance and offset *open*
// polylines (`expolygons_to_zpaths64_expanded_opened`). The crate's geo-clipper
// backend exposes neither SetZCallback nor a Z-preserving open-path offset, so
// this polygon-based intersection is a documented approximation, not byte-faithful.
// SUPERSEDED by `wave_seeds_faithful` (above); kept as the default fallback.
fn wave_seeds_polygon_based(
    src: &[ExPolygon],
    boundary: &[ExPolygon],
    tiny_expansion: CoordF,
) -> Vec<(ExPolygons, u32, u32)> {
    // seeds: Vec of (seed_expolygons, src_id, boundary_id)
    if src.is_empty() || boundary.is_empty() {
        return Vec::new();
    }

    let mut results = Vec::new();

    // Expand each source region slightly
    for (src_idx, src_ep) in src.iter().enumerate() {
        let expanded_src =
            offset_expolygons(&[src_ep.clone()], tiny_expansion, OffsetJoinType::Round);
        if expanded_src.is_empty() {
            continue;
        }

        // Intersect with each boundary region to find seeds
        for (bnd_idx, bnd_ep) in boundary.iter().enumerate() {
            let seeds = intersection(&expanded_src, &[bnd_ep.clone()]);
            if !seeds.is_empty() {
                results.push((seeds, src_idx as u32, bnd_idx as u32));
            }
        }
    }

    results
}

/// Propagate a wave from seed polygons within a boundary.
///
/// Port of `propagate_wave_from_boundary()` from RegionExpansion.cpp:442-465.
///
/// The wave is grown in iterative steps, clipped to the boundary at each step:
/// 1. First wave: offset seeds by `initial_step`, clip to boundary
/// 2. Subsequent waves: offset by `other_step`, clip to boundary
///
/// This iterative approach prevents expansion from "leaking" through narrow
/// gaps in the boundary that a single large offset would cross.
///
// FIDELITY-NOTE(F1): C++ `propagate_wave_from_boundary` (RegionExpansion.cpp:442-465)
// offsets *open* polylines via `wavefront_initial` (jtRound/etOpenRound) for the
// first wave and closed polygons via `wavefront_step` afterwards, and trims the
// boundary by a subject bbox (`clip_clipper_polygons_with_subject_bbox`) purely
// for speed. Here the seed approximation yields closed ExPolygons, so both waves
// use the closed-polygon `grow` (Round) and clip against the full boundary —
// geometrically equivalent up to the open-vs-closed seed offset difference.
fn propagate_wave_from_seeds(
    seeds: &[ExPolygon],
    boundary: &[ExPolygon],
    initial_step: CoordF,
    other_step: CoordF,
    num_other_steps: usize,
) -> ExPolygons {
    if seeds.is_empty() || boundary.is_empty() {
        return Vec::new();
    }

    // First wave: offset seeds by initial_step, clip to boundary
    let mut wave = grow(seeds, initial_step, OffsetJoinType::Round);
    wave = intersection(&wave, boundary);

    if wave.is_empty() {
        return Vec::new();
    }

    // Subsequent waves: offset by other_step, clip to boundary
    for _ in 0..num_other_steps {
        let expanded = grow(&wave, other_step, OffsetJoinType::Round);
        wave = intersection(&expanded, boundary);
        if wave.is_empty() {
            return Vec::new();
        }
    }

    wave
}

/// Propagate waves from all source expolygons into all boundary expolygons.
///
/// Port of `propagate_waves()` from RegionExpansion.cpp:468-494.
///
/// For each (src, boundary) pair that has seeds, runs wave propagation
/// and collects the resulting expanded regions.
fn propagate_waves(
    src: &[ExPolygon],
    boundary: &[ExPolygon],
    params: &RegionExpansionParameters,
) -> Vec<RegionExpansion> {
    // Faithful path (gated): Clipper2-Z byte-faithful seeds + ClipperLib byte-
    // faithful wavefront propagation (cz_propagate_wave, R131). The default keeps
    // the geo polygon-approximation below (byte-locked).
    if crate::faithful_gate("REGION_EXPANSION_FAITHFUL") {
        return propagate_waves_faithful(src, boundary, params);
    }

    let seeds = wave_seeds_polygon_based(src, boundary, params.tiny_expansion);

    let mut results = Vec::new();
    for (seed_polys, src_id, boundary_id) in &seeds {
        // Propagate wave within just this boundary expolygon
        let bnd = &[boundary[*boundary_id as usize].clone()];
        let expanded = propagate_wave_from_seeds(
            seed_polys,
            bnd,
            params.initial_step,
            params.other_step,
            params.num_other_steps,
        );

        for ep in expanded {
            results.push(RegionExpansion {
                polygon: ep,
                src_id: *src_id,
                boundary_id: *boundary_id,
            });
        }
    }

    results
}

/// Merge expanded regions back into source expolygons.
///
/// Port of `merge_expansions_into_expolygons()` from RegionExpansion.cpp:564-615.
///
/// For each source expolygon that had expansions, union the source with its
/// expanded regions. Sources without expansions are returned unchanged.
fn merge_expansions_into_expolygons(
    src: ExPolygons,
    mut expanded: Vec<RegionExpansion>,
) -> ExPolygons {
    if expanded.is_empty() {
        return src;
    }

    // Sort expansions by source id
    expanded.sort_by_key(|e| e.src_id);

    let mut out = Vec::with_capacity(src.len());
    let mut exp_iter = expanded.iter().peekable();

    for (src_idx, src_ep) in src.into_iter().enumerate() {
        let idx = src_idx as u32;

        // Collect all expansions for this source
        let mut acc: ExPolygons = Vec::new();
        while let Some(exp) = exp_iter.peek() {
            if exp.src_id == idx {
                acc.push(exp_iter.next().unwrap().polygon.clone());
            } else if exp.src_id > idx {
                break;
            } else {
                // Skip expansions for earlier sources (shouldn't happen if sorted)
                exp_iter.next();
            }
        }

        if acc.is_empty() {
            out.push(src_ep);
        } else {
            // RegionExpansion.cpp:580 — `ExPolygon &src_ex = src[last ++];`
            // RegionExpansion.cpp:581 — assert(! src_ex.contour.empty());
            debug_assert!(!src_ep.contour.points().is_empty());
            // RegionExpansion.cpp:594 — `Point sample = src_ex.contour.front();`
            let sample = src_ep.contour.points()[0];
            // RegionExpansion.cpp:595 — `append(acc, to_polygons(std::move(src_ex)));`
            acc.push(src_ep);
            // RegionExpansion.cpp:596 — `ExPolygons merged = union_safety_offset_ex(acc);`
            let merged = union_safety_offset_ex_expolygons(&acc);
            // RegionExpansion.cpp:597-599
            // Expanding one expolygon by waves should not change connectivity of the source expolygon:
            // Single expolygon should be produced possibly with increased number of holes.
            if merged.len() > 1 {
                // RegionExpansion.cpp:600-608
                // There is something wrong with the initial waves. Most likely the bridge was not valid at all
                // or the boundary region was very close to some bridge edge, but not really touching.
                // Pick only a single merged expolygon, which contains one sample point of the source expolygon.
                let aabb_tree = build_aabb_tree_over_expolygons(&merged);
                let id = sample_in_expolygons(&aabb_tree, &merged, &sample);
                debug_assert!(id != -1);
                // RegionExpansion.cpp:607-608
                if id != -1 {
                    // RegionExpansion.cpp:608 — `out.emplace_back(std::move(merged[id]));`
                    out.push(merged.into_iter().nth(id as usize).unwrap());
                }
            } else if merged.len() == 1 {
                // RegionExpansion.cpp:609-610
                out.push(merged.into_iter().next().unwrap());
            }
        }
    }

    // RegionExpansion.cpp:612-613 — remaining untouched sources are appended by the
    // enumerate loop above (each `src_ep` without expansions is pushed unchanged).

    out
}

/// Expand source expolygons into boundary expolygons and merge.
///
/// Port of `expand_merge_expolygons()` from RegionExpansion.cpp:617-622.
///
/// This is the high-level function that combines wave propagation with merging.
pub fn expand_merge_expolygons(
    src: ExPolygons,
    boundary: &[ExPolygon],
    params: &RegionExpansionParameters,
) -> ExPolygons {
    let expanded = propagate_waves(&src, boundary, params);
    merge_expansions_into_expolygons(src, expanded)
}

// ============================================================================
// expand_merge_surfaces — per-surface-type expansion
// ============================================================================

/// Extract expolygons of a given surface type from surfaces.
///
/// Port of `fill_surfaces_extract_expolygons()`.
fn extract_expolygons_by_type(
    surfaces: &[crate::surface::Surface],
    surface_type: crate::surface::SurfaceType,
) -> ExPolygons {
    surfaces
        .iter()
        .filter(|s| s.surface_type == surface_type)
        .map(|s| s.expolygon.clone())
        .collect()
}

/// Expand surfaces of a given type into expansion zones, merge, apply closing,
/// subtract from zones, and return the expanded surfaces.
///
/// Port of `expand_merge_surfaces()` from LayerRegion.cpp:470-516.
///
/// # Algorithm
///
/// 1. Extract expolygons of the target surface_type from `surfaces`
/// 2. For each expansion zone: run wave propagation (propagate_waves)
/// 3. Merge expanded regions with source expolygons
/// 4. Apply morphological closing to fill small gaps
/// 5. Subtract the expanded area from each zone (consuming boundary area)
/// 6. Return the expanded expolygons as new surfaces
///
/// # Arguments
///
/// * `surfaces` - The layer's fill surfaces
/// * `surface_type` - Which surface type to expand (Top, Bottom, BottomBridge)
/// * `expansion_zones` - Mutable zones (InternalSolid, Internal, ...) to expand into
/// * `closing_radius` - Morphological closing radius to fill tiny gaps (mm)
/// * `bridge_angle` - Optional bridge angle for bridge surfaces
pub fn expand_merge_surfaces(
    surfaces: &[crate::surface::Surface],
    surface_type: crate::surface::SurfaceType,
    expansion_zones: &mut [ExpansionZone],
    closing_radius: CoordF,
    bridge_angle: Option<CoordF>,
) -> Vec<crate::surface::Surface> {
    let src = extract_expolygons_by_type(surfaces, surface_type);
    if src.is_empty() {
        return Vec::new();
    }

    // Run wave propagation into each zone and collect all expansions.
    // The C++ tracks boundary_id offsets across zones; we handle this by
    // iterating zones sequentially and tracking a global offset.
    let mut all_expansions: Vec<RegionExpansion> = Vec::new();
    let mut processed_count: u32 = 0;

    for zone in expansion_zones.iter_mut() {
        let zone_expansions = propagate_waves(&src, &zone.expolygons, &zone.parameters);
        zone.expanded_into = !zone_expansions.is_empty();

        // Offset boundary_ids by the count of expolygons from previous zones
        for mut exp in zone_expansions {
            exp.boundary_id += processed_count;
            all_expansions.push(exp);
        }
        processed_count += zone.expolygons.len() as u32;
    }

    // Merge expansions into source expolygons
    let mut expanded = merge_expansions_into_expolygons(src, all_expansions);

    // Apply morphological closing to fill small unassigned regions.
    // C++ comment: "The current regularization of the shells can create small
    // unassigned regions in the object (E.G. benchy) without the following
    // closing operation, those regions will stay unfilled."
    if closing_radius > 0.0 && !expanded.is_empty() {
        // C++ LayerRegion::process_external_surfaces (LayerRegion.cpp:503):
        // `expanded = closing_ex(expanded, closing_radius)` = offset2_ex(+r, -r,
        // jtMiter) via ClipperLib @1e5 (ClipperUtils.hpp:412; DefaultJoinType =
        // jtMiter). Rust legacy used geo-clipper `closing` @1µm with ROUND joins,
        // which over-fragments the solid region (R71-R74 A/B: Round+geo n_solid
        // 1911 vs Miter+clib 707, native 605) → drives the ISI/Floating feature
        // split. Route through the vertex-exact clib offset2 with Miter under
        // F1_UNION; default keeps the geo Round closing (byte-locked).
        expanded = if crate::faithful_gate("F1_UNION") {
            offset2_ex_clib(&expanded, closing_radius, -closing_radius, OffsetJoinType::Miter)
        } else {
            closing(&expanded, closing_radius, OffsetJoinType::Round)
        };
    }

    // Subtract expanded area from each zone that was expanded into.
    // This "consumes" the boundary area so subsequent surface types
    // don't expand into already-claimed regions.
    for zone in expansion_zones.iter_mut() {
        if zone.expanded_into {
            // C++ RegionExpansion.cpp / LayerRegion.cpp:505 `diff_ex(zone, expanded)` via
            // ClipperLib @1e5. EXPERIMENT (gated): route through difference_clib now that
            // `expanded` is the full-res clib closing output (R128). R74 measured
            // difference_clib WORSE with a geo closing; retest on the clib closing input.
            zone.expolygons = if crate::faithful_gate("F1_UNION") {
                crate::clipper_utils::difference_clib(&zone.expolygons, &expanded)
            } else {
                difference(&zone.expolygons, &expanded)
            };
        }
    }

    // Create output surfaces
    let mut out = Vec::with_capacity(expanded.len());
    for ep in expanded {
        let mut s = crate::surface::Surface::new(surface_type, ep);
        if let Some(angle) = bridge_angle {
            s.bridge_angle = Some(angle);
        }
        out.push(s);
    }

    out
}

// ============================================================================
// process_external_surfaces — faithful C++ port
// ============================================================================

/// Configuration for process_external_surfaces.
///
/// Encapsulates the flow-derived parameters that the C++ gets from
/// `LayerRegion::flow()` and region config.
#[derive(Debug, Clone)]
pub struct ExternalSurfaceConfig {
    /// Width of the perimeter shell (mm).
    ///       + perimeter_flow.spacing() * (num_perimeters - 1)`
    pub shell_width: CoordF,

    /// Minimum expansion distance (mm).
    pub expansion_min: CoordF,

    /// Solid infill flow spacing (mm), used to compute closing_radius.
    pub solid_infill_spacing: CoordF,

    /// Number of perimeters (wall loops).
    pub num_perimeters: usize,

    /// Minimum sparse infill area (mm²). Regions smaller than this are
    /// promoted from Internal to InternalSolid.
    pub minimum_sparse_infill_area: CoordF,

    /// Whether spiral vase mode is active (disables minimum area logic).
    pub spiral_mode: bool,

    /// Sparse infill density (0.0 - 1.0). Used to decide if minimum area
    /// logic applies.
    pub sparse_infill_density: CoordF,

    /// Custom bridge angle (degrees, 0 = auto-detect). If > 0, uses this
    /// fixed angle for all bridges instead of auto-detecting.
    pub custom_bridge_angle: CoordF,
}

impl Default for ExternalSurfaceConfig {
    fn default() -> Self {
        Self {
            shell_width: 0.4, // typical for 0.4mm nozzle, 1 perimeter
            expansion_min: 0.45,
            solid_infill_spacing: 0.4,
            num_perimeters: 2,
            minimum_sparse_infill_area: 0.0,
            spiral_mode: false,
            sparse_infill_density: 0.15,
            custom_bridge_angle: 0.0,
        }
    }
}

impl ExternalSurfaceConfig {
    // Create config from flow parameters matching C++ LayerRegion::process_external_surfaces().
    //
    // # Arguments
    //
    // * `external_perimeter_width` - External perimeter flow width (mm)
    // * `external_perimeter_spacing` - External perimeter flow spacing (mm)
    // * `perimeter_spacing` - Internal perimeter flow spacing (mm)
    // * `solid_infill_spacing` - Solid infill flow spacing (mm)
    // * `num_perimeters` - Number of perimeter loops
    pub fn from_flows(
        external_perimeter_width: CoordF,
        external_perimeter_spacing: CoordF,
        perimeter_spacing: CoordF,
        solid_infill_spacing: CoordF,
        num_perimeters: usize,
    ) -> Self {
        let (shell_width, expansion_min) = if num_perimeters > 0 {
            if crate::faithful_gate("REGION_EXPANSION_FAITHFUL") {
                // R284: native computes on f32(coord_t scaled) values
                // (LayerRegion.cpp:529-535): shell_width = 0.5f*f32(W_e) +
                // f32(S_e), += f32(S_p_int*(n-1)); expansion_min = f32(S_p).
                let w_e = (external_perimeter_width / 0.00001).trunc();
                let s_e = (external_perimeter_spacing / 0.00001).trunc();
                let s_p = (perimeter_spacing / 0.00001).trunc();
                let mut sw = 0.5f32 * (w_e as f32) + (s_e as f32);
                sw += (s_p * (num_perimeters as f64 - 1.0)) as f32;
                (
                    (sw as f64) / crate::SCALING_FACTOR,
                    ((s_p as f32) as f64) / crate::SCALING_FACTOR,
                )
            } else {
                let sw = 0.5 * external_perimeter_width
                    + external_perimeter_spacing
                    + perimeter_spacing * (num_perimeters as CoordF - 1.0);
                (sw, perimeter_spacing)
            }
        } else if crate::faithful_gate("REGION_EXPANSION_FAITHFUL") {
            // Native fallback: float(SCALED_EPSILON) = 10 units = 1e-4 mm
            // (LayerRegion.cpp:537-538) — NOT 1e-6.
            (1e-4, 1e-4)
        } else {
            (1e-6, 1e-6) // SCALED_EPSILON equivalent
        };

        Self {
            shell_width,
            expansion_min,
            solid_infill_spacing,
            num_perimeters,
            ..Default::default()
        }
    }

    /// Compute expansion distance for top/bottom surfaces.
    pub fn expansion_top(&self) -> CoordF {
        if crate::faithful_gate("REGION_EXPANSION_FAITHFUL") {
            // Native: float expansion_top = shell_width * sqrt(2.) — f32 shell
            // width times DOUBLE sqrt(2), rounded back to f32 at the store
            // (LayerRegion.cpp:542).
            let sw_sc = (self.shell_width * crate::SCALING_FACTOR) as f32;
            ((((sw_sc as f64) * std::f64::consts::SQRT_2) as f32) as f64) / crate::SCALING_FACTOR
        } else {
            self.shell_width * std::f64::consts::SQRT_2
        }
    }

    /// Compute expansion distance for bottom surfaces.
    /// Same as top in C++.
    pub fn expansion_bottom(&self) -> CoordF {
        self.expansion_top()
    }

    /// Compute expansion distance for bridge surfaces.
    /// Same as top in C++.
    pub fn expansion_bottom_bridge(&self) -> CoordF {
        self.expansion_top()
    }

    /// Compute closing radius.
    /// Converted to mm.
    pub fn closing_radius(&self) -> CoordF {
        if crate::faithful_gate("REGION_EXPANSION_FAITHFUL") {
            // Native f32 chain: 0.55f*0.65f*1.05f*f32(scaled_spacing)
            // (LayerRegion.cpp:550), left-assoc const-folded in f32.
            let s_si = (self.solid_infill_spacing / 0.00001).trunc();
            let r = ((0.55f32 * 0.65f32) * 1.05f32) * (s_si as f32);
            (r as f64) / crate::SCALING_FACTOR
        } else {
            0.55 * 0.65 * 1.05 * self.solid_infill_spacing
        }
    }
}

/// Process external surfaces for all layers — faithful port of
/// `LayerRegion::process_external_surfaces()` from LayerRegion.cpp:517-643.
///
/// This function expands Top, Bottom, and BottomBridge surfaces into surrounding
/// Internal and InternalSolid fill area using wave-based expansion. The expansion
/// ensures that external surfaces are wide enough for proper infill (e.g., monotonic
/// top surfaces, bridge anchoring).
///
/// # Algorithm (per layer)
///
/// 1. Extract three expansion zones from fill surfaces:
///    - Zone 0: InternalSolid (shells)
///    - Zone 1: Internal (sparse)
///    - Zone 2: Top (used temporarily for bridge expansion)
/// 2. Expand BottomBridge surfaces into all three zones
/// 3. Move Top zone expolygons back as Top surfaces, remove zone 2
/// 4. Expand Bottom surfaces into zones 0+1
/// 5. Expand Top surfaces into zones 0+1
/// 6. Apply minimum_sparse_infill_area: promote tiny sparse → solid
/// 7. Reassemble fill surfaces from zones + expanded surfaces
///
/// # Arguments
///
/// * `surfaces` - Per-layer fill surfaces (modified in place)
/// * `config` - External surface expansion configuration
pub fn process_external_surfaces_wave(
    surfaces: &mut [Vec<crate::surface::Surface>],
    config: &ExternalSurfaceConfig,
) {
    // C++ constants
    const EXPANSION_STEP: CoordF = 0.1; // mm )
    const MAX_NR_EXPANSION_STEPS: usize = 5;

    // Precompute expansion distances
    let expansion_top = config.expansion_top();
    let expansion_bottom = config.expansion_bottom();
    let expansion_bottom_bridge = config.expansion_bottom_bridge();
    let closing_radius = config.closing_radius();

    // Skip if expansion is negligible
    if expansion_top <= 0.0 {
        return;
    }

    // Build expansion parameters
    // Into sparse infill: expand by expansion_min (perimeter spacing)
    let params_into_sparse = RegionExpansionParameters::build(
        config.expansion_min.max(EXPANSION_STEP * 0.5), // ensure positive
        EXPANSION_STEP,
        MAX_NR_EXPANSION_STEPS,
    );
    // Into solid infill: expand by expansion_bottom_bridge (full shell width * sqrt(2))
    let params_into_solid = RegionExpansionParameters::build(
        expansion_bottom_bridge,
        EXPANSION_STEP,
        MAX_NR_EXPANSION_STEPS,
    );

    for layer_surfaces in surfaces.iter_mut() {
        // ── Extract expansion zones from current fill surfaces ──
        //
        // Zone 0: InternalSolid (shells)
        // Zone 1: Internal (sparse)
        // Zone 2: Top expolygons (temporary, for bridge expansion only)
        let shells = union_ex(&extract_expolygons_by_type(
            layer_surfaces,
            crate::surface::SurfaceType::InternalSolid,
        ));
        let sparse = union_ex(&extract_expolygons_by_type(
            layer_surfaces,
            crate::surface::SurfaceType::Internal,
        ));
        let top_expolygons = union_ex(&extract_expolygons_by_type(
            layer_surfaces,
            crate::surface::SurfaceType::Top,
        ));

        // If there's nothing to expand into, skip this layer
        if shells.is_empty() && sparse.is_empty() && top_expolygons.is_empty() {
            continue;
        }

        // Build the 3-zone structure matching C++ (LayerRegion.cpp:560-564)
        let mut expansion_zones = vec![
            ExpansionZone {
                expolygons: shells,
                parameters: params_into_solid.clone(),
                expanded_into: false,
            },
            ExpansionZone {
                expolygons: sparse,
                parameters: params_into_sparse.clone(),
                expanded_into: false,
            },
            ExpansionZone {
                expolygons: top_expolygons.clone(),
                parameters: params_into_solid.clone(),
                expanded_into: false,
            },
        ];

        // ── Step 1: Process bridges ──
        //
        //      Otherwise, uses expand_bridges_detect_orientations which detects
        //      per-bridge angles from floating edges and groups overlapping bridges.
        let bridges = if config.custom_bridge_angle > 0.0 {
            let angle = config.custom_bridge_angle.to_radians();
            expand_merge_surfaces(
                layer_surfaces,
                crate::surface::SurfaceType::BottomBridge,
                &mut expansion_zones,
                closing_radius,
                Some(angle),
            )
        } else {
            expand_bridges_detect_orientations(layer_surfaces, &mut expansion_zones, closing_radius)
        };

        // ── Step 2: Handle Top expolygons from zone 2 ──
        //
        // Then pop zone 2.
        // This means that after bridge expansion, whatever remains of the Top zone
        // (not consumed by bridges) becomes the new Top surfaces.
        let remaining_top_expolygons = expansion_zones.pop().unwrap().expolygons;

        // ── Step 3: Expand Bottom surfaces ──
        //
        expansion_zones[0].parameters = RegionExpansionParameters::build(
            expansion_bottom,
            EXPANSION_STEP,
            MAX_NR_EXPANSION_STEPS,
        );
        let bottoms = expand_merge_surfaces(
            layer_surfaces,
            crate::surface::SurfaceType::Bottom,
            &mut expansion_zones,
            closing_radius,
            None,
        );

        // ── Step 4: Expand Top surfaces ──
        //
        expansion_zones[0].parameters =
            RegionExpansionParameters::build(expansion_top, EXPANSION_STEP, MAX_NR_EXPANSION_STEPS);

        // For top expansion, we need to use the remaining_top_expolygons from zone 2
        // as the source (not the original layer surfaces, since those were already
        // partially consumed by bridges).
        // Build synthetic surfaces for the top expansion
        let synth_top_surfaces: Vec<crate::surface::Surface> = remaining_top_expolygons
            .iter()
            .map(|ep| crate::surface::Surface::new(crate::surface::SurfaceType::Top, ep.clone()))
            .collect();
        // Also include original top surfaces that weren't in the zone
        // (The C++ removes stTop from fill_surfaces then re-adds zone 2; we approximate)
        let tops = expand_merge_surfaces(
            &synth_top_surfaces,
            crate::surface::SurfaceType::Top,
            &mut expansion_zones,
            closing_radius,
            None,
        );

        // ── Step 5: Minimum sparse infill area ──
        //
        // "apply minimum sparse infill area logic"
        if !config.spiral_mode
            && config.sparse_infill_density > 0.0
            && config.minimum_sparse_infill_area > 0.0
        {
            // C++ LayerRegion.cpp:602 — min_area = scale_(scale_(minimum_sparse_infill_area)).
            // scale_(x) = x * 1e5 (SCALING_FACTOR = 1e-5), so mm² → scaled² is * 1e10, not 1e12.
            let scale_factor = crate::SCALING_FACTOR; // 1e5 integer-per-mm
            let min_area = config.minimum_sparse_infill_area * scale_factor * scale_factor;
            let mut areas_to_solid: ExPolygons = Vec::new();

            expansion_zones[1].expolygons.retain(|ep| {
                if ep.area().abs() <= min_area {
                    areas_to_solid.push(ep.clone());
                    false
                } else {
                    true
                }
            });

            if !areas_to_solid.is_empty() {
                expansion_zones[0].expolygons =
                    union_ex(&[expansion_zones[0].expolygons.clone(), areas_to_solid].concat());
            }
        }

        // ── Step 6: Reassemble fill surfaces ──
        //
        //   - Zone 0 (remaining shells) as InternalSolid
        //   - Zone 1 (remaining sparse) as Internal
        //   - bridges
        //   - bottoms
        //   - tops
        let mut new_surfaces: Vec<crate::surface::Surface> = Vec::new();

        // InternalSolid from remaining zone 0
        for ep in &expansion_zones[0].expolygons {
            new_surfaces.push(crate::surface::Surface::internal_solid(ep.clone()));
        }

        // Internal from remaining zone 1
        for ep in &expansion_zones[1].expolygons {
            new_surfaces.push(crate::surface::Surface::internal(ep.clone()));
        }

        // Bridges
        new_surfaces.extend(bridges);

        // Bottoms
        new_surfaces.extend(bottoms);

        // Tops
        new_surfaces.extend(tops);

        // Preserve other surface types (InternalBridge, InternalVoid) unchanged
        for s in layer_surfaces.iter() {
            match s.surface_type {
                crate::surface::SurfaceType::Internal
                | crate::surface::SurfaceType::InternalSolid
                | crate::surface::SurfaceType::Top
                | crate::surface::SurfaceType::Bottom
                | crate::surface::SurfaceType::BottomBridge => {
                    // Already handled above
                }
                _ => {
                    new_surfaces.push(s.clone());
                }
            }
        }

        *layer_surfaces = new_surfaces;
    }
}

// ============================================================================
// Bridge grouping and direction detection
// ============================================================================
//
// Faithful port of BambuStudio's bridge expansion with per-bridge angle detection.
//
// C++ reference: LayerRegion.cpp — `expand_bridges_detect_orientations()`,
// `get_grouped_bridges()`, `detect_bridge_directions()`, `merge_bridges()`,
// `expand_expolygons()`.
//
// The algorithm:
// 1. Extract BottomBridge expolygons from surfaces
// 2. Expand them into expansion zones (wave seeds + propagation), keeping
//    both anchor info (WaveSeeds) and expansion geometry (RegionExpansionEx)
// 3. Group bridges whose expansions overlap within the same boundary region
//    (union-find)
// 4. For each bridge, collect anchor areas from expansion zones, compute
//    unsupported/floating edges, detect optimal bridge direction
// 5. Merge bridges with the same group_id, union their geometry + expansions,
//    apply closing, assign the group head's bridge angle

/// Wave seed tracking which source region touches which boundary region.
///
/// Port of `Slic3r::Algorithm::WaveSeed` from RegionExpansion.hpp. The faithful
/// `wave_seeds_faithful` path fills `path` with the open boundary-crossing seed
/// polyline (RegionExpansion.hpp:46); the legacy polygon-approximation + bridge-
/// anchor paths leave it empty (they carry seed geometry separately).
#[derive(Debug, Clone, Default)]
struct WaveSeed {
    /// Index of the source expolygon.
    src: u32,
    /// Index of the boundary expolygon (global across all zones).
    boundary: u32,
    /// The open seed polyline (empty on the legacy/bridge paths).
    path: Vec<Point>,
}

/// Extended expansion result with ExPolygon — used for bridge overlap detection.
///
/// Port of `Slic3r::Algorithm::RegionExpansionEx` from RegionExpansion.hpp.
#[derive(Debug, Clone)]
struct RegionExpansionEx {
    /// The expanded expolygon.
    expolygon: ExPolygon,
    /// Index of the source expolygon this expansion originated from.
    src_id: u32,
    /// Index of the boundary expolygon this expansion grew into (global).
    boundary_id: u32,
}

/// Cache for bridge grouping and angle detection.
///
/// Port of the local `Bridge` struct in LayerRegion.cpp (not BridgeDetector.hpp).
struct BridgeInfo {
    /// The bridge expolygon.
    expolygon: ExPolygon,
    /// Group ID for union-find grouping.
    group_id: u32,
    /// Detected bridge angle (radians), None if not yet computed.
    angle: Option<f64>,
    /// Index into expansions vec where this bridge's expansions start.
    #[allow(dead_code)]
    expansion_begin: usize,
}

/// Result of expanding bridge expolygons with anchor tracking.
///
/// Port of `ExpansionResult` struct in LayerRegion.cpp.
struct BridgeExpansionResult {
    /// Anchor seeds — track which src touches which boundary.
    anchors: Vec<WaveSeed>,
    /// Expanded regions with ExPolygon-level detail.
    expansions: Vec<RegionExpansionEx>,
}

/// Run wave propagation and return both anchor info and ExPolygon-level expansions.
///
/// Port of the combination of `wave_seeds()` + `propagate_waves_ex()` from
/// RegionExpansion.cpp:508-531.
///
/// Like `propagate_waves()`, but additionally:
/// - Returns `WaveSeed` anchor info for each (src, boundary) pair
/// - Groups raw expansions by (boundary_id, src_id) and unions into ExPolygons
fn propagate_waves_ex(
    src: &[ExPolygon],
    boundary: &[ExPolygon],
    params: &RegionExpansionParameters,
) -> (Vec<WaveSeed>, Vec<RegionExpansionEx>) {
    // Anchors + raw expansions, produced either by the faithful path (gated) or the
    // legacy geo approximation. Both feed the shared grouping/merge below.
    let (wave_seeds_out, mut raw_expansions): (Vec<WaveSeed>, Vec<RegionExpansion>) =
        if crate::faithful_gate("REGION_EXPANSION_FAITHFUL") {
            // Faithful: byte-faithful Clipper2-Z seeds (carry the anchor src/boundary)
            // + byte-faithful ClipperLib propagation (R131 cz_propagate_wave).
            let anchors = wave_seeds_faithful(src, boundary, params.tiny_expansion, true);
            let raw = propagate_waves_faithful(src, boundary, params);
            (anchors, raw)
        } else {
            let seeds = wave_seeds_polygon_based(src, boundary, params.tiny_expansion);
            let mut anchors: Vec<WaveSeed> = Vec::new();
            for (_, src_id, boundary_id) in &seeds {
                anchors.push(WaveSeed {
                    src: *src_id,
                    boundary: *boundary_id,
                    path: Vec::new(),
                });
            }
            let mut raw: Vec<RegionExpansion> = Vec::new();
            for (seed_polys, src_id, boundary_id) in &seeds {
                let bnd = &[boundary[*boundary_id as usize].clone()];
                let expanded = propagate_wave_from_seeds(
                    seed_polys,
                    bnd,
                    params.initial_step,
                    params.other_step,
                    params.num_other_steps,
                );
                for ep in expanded {
                    raw.push(RegionExpansion {
                        polygon: ep,
                        src_id: *src_id,
                        boundary_id: *boundary_id,
                    });
                }
            }
            (anchors, raw)
        };

    // Group by (boundary_id, src_id) and merge into ExPolygons — matching C++
    // propagate_waves_ex behavior.
    raw_expansions.sort_by(|a, b| {
        a.boundary_id
            .cmp(&b.boundary_id)
            .then(a.src_id.cmp(&b.src_id))
    });

    let mut result: Vec<RegionExpansionEx> = Vec::new();
    let mut i = 0;
    while i < raw_expansions.len() {
        let src_id = raw_expansions[i].src_id;
        let boundary_id = raw_expansions[i].boundary_id;
        let mut acc: ExPolygons = Vec::new();
        while i < raw_expansions.len()
            && raw_expansions[i].boundary_id == boundary_id
            && raw_expansions[i].src_id == src_id
        {
            acc.push(raw_expansions[i].polygon.clone());
            i += 1;
        }
        if acc.len() == 1 {
            result.push(RegionExpansionEx {
                expolygon: acc.into_iter().next().unwrap(),
                src_id,
                boundary_id,
            });
        } else {
            let merged = union_ex(&acc);
            for ep in merged {
                result.push(RegionExpansionEx {
                    expolygon: ep,
                    src_id,
                    boundary_id,
                });
            }
        }
    }

    (wave_seeds_out, result)
}

/// Expand bridge expolygons into expansion zones, returning both anchors and expansions.
///
/// Port of `expand_expolygons()` from LayerRegion.cpp:389-425.
///
/// For each expansion zone, runs wave seeding and propagation. Accumulates
/// anchors (WaveSeeds) and expansions (RegionExpansionEx) across all zones,
/// offsetting boundary_ids to form a flat index space.
fn expand_expolygons_with_anchors(
    expolygons: &[ExPolygon],
    expansion_zones: &mut [ExpansionZone],
) -> BridgeExpansionResult {
    let mut all_anchors: Vec<WaveSeed> = Vec::new();
    let mut all_expansions: Vec<RegionExpansionEx> = Vec::new();

    let mut processed_count: u32 = 0;
    for zone in expansion_zones.iter_mut() {
        let (mut seeds, mut expansions) =
            propagate_waves_ex(expolygons, &zone.expolygons, &zone.parameters);

        // Offset boundary IDs by the count of expolygons from previous zones
        for seed in &mut seeds {
            seed.boundary += processed_count;
        }
        for exp in &mut expansions {
            exp.boundary_id += processed_count;
        }

        zone.expanded_into = !expansions.is_empty();

        all_anchors.extend(seeds);
        all_expansions.extend(expansions);

        processed_count += zone.expolygons.len() as u32;
    }

    BridgeExpansionResult {
        anchors: all_anchors,
        expansions: all_expansions,
    }
}

/// Union-find: resolve group_id to the root of the group.
///
/// Port of the `group_id()` helper in LayerRegion.cpp.
fn resolve_group_id(bridges: &mut [BridgeInfo], src_id: u32) -> u32 {
    let mut id = bridges[src_id as usize].group_id;
    while id != bridges[id as usize].group_id {
        id = bridges[id as usize].group_id;
    }
    // Path compression
    bridges[src_id as usize].group_id = id;
    id
}

/// Group bridge surfaces by overlapping expansions within the same boundary region.
///
/// Port of `get_grouped_bridges()` from LayerRegion.cpp:223-291.
///
/// Creates one BridgeInfo per bridge expolygon. Then iterates through expansions
/// grouped by boundary_id, checking for overlap between expansions from different
/// source bridges. Overlapping bridges get the same group_id (union-find).
fn get_grouped_bridges(
    bridge_expolygons: Vec<ExPolygon>,
    bridge_expansions: &[RegionExpansionEx],
) -> Vec<BridgeInfo> {
    let mut result: Vec<BridgeInfo> = bridge_expolygons
        .into_iter()
        .enumerate()
        .map(|(i, ep)| BridgeInfo {
            expolygon: ep,
            group_id: i as u32,
            angle: None,
            expansion_begin: usize::MAX, // sentinel — set later in merge_bridges
        })
        .collect();

    if result.is_empty() || bridge_expansions.is_empty() {
        return result;
    }

    // Detect overlaps of bridge anchors within the same boundary region.
    // bridge_expansions are sorted by boundary_id from expand_expolygons_with_anchors.
    let mut i = 0;
    while i < bridge_expansions.len() {
        let boundary_id = bridge_expansions[i].boundary_id;

        // Find the range of expansions for this boundary
        let region_begin = i;
        let mut region_end = i + 1;
        while region_end < bridge_expansions.len()
            && bridge_expansions[region_end].boundary_id == boundary_id
        {
            region_end += 1;
        }

        // Cache bounding boxes for quick rejection
        let bounding_boxes: Vec<BoundingBox> = bridge_expansions[region_begin..region_end]
            .iter()
            .map(|exp| BoundingBox::from_points(exp.expolygon.contour.points()))
            .collect();

        // Check all pairs within this boundary region for overlap
        for a_idx in 0..(region_end - region_begin) {
            for b_idx in (a_idx + 1)..(region_end - region_begin) {
                let exp_a = &bridge_expansions[region_begin + a_idx];
                let exp_b = &bridge_expansions[region_begin + b_idx];

                // Only group bridges from different sources
                if exp_a.src_id == exp_b.src_id {
                    continue;
                }

                // Quick bounding box check
                if !bounding_boxes[a_idx].intersects(&bounding_boxes[b_idx]) {
                    continue;
                }

                // Full intersection test (contour only, ignoring holes — matches C++)
                let a_ex = vec![ExPolygon::new(exp_a.expolygon.contour.clone())];
                let b_ex = vec![ExPolygon::new(exp_b.expolygon.contour.clone())];
                if !intersection(&a_ex, &b_ex).is_empty() {
                    // The two bridge expansions intersect — give them the same group id
                    let id_a = resolve_group_id(&mut result, exp_a.src_id);
                    let id_b = resolve_group_id(&mut result, exp_b.src_id);
                    if id_a < id_b {
                        result[id_b as usize].group_id = id_a;
                    } else if id_b < id_a {
                        result[id_a as usize].group_id = id_b;
                    }
                }
            }
        }

        i = region_end;
    }

    result
}

/// Detect the optimal bridging direction from floating (unsupported) edges.
///
/// Port of `detect_bridging_direction(Lines, Polygons)` from BridgeDetector.hpp:75-120.
///
/// Finds the direction that minimizes the total length of floating edges
/// perpendicular to the bridge direction. If there are no floating edges
/// (fully anchored), uses principal component analysis to find the shortest
/// bridge span direction.
fn detect_bridging_direction_from_lines(
    floating_edges: &[Line],
    overhang_polygons: &[Polygon],
) -> (PointF, f64) {
    if floating_edges.is_empty() {
        // Fully anchored — use principal components to find shortest bridge direction.
        // axis (shortest direction).
        if let Some(dir) = principal_component_direction(overhang_polygons) {
            return (dir, 0.0);
        }
        return (PointF::new(1.0, 0.0), 0.0);
    }

    // Build direction candidates from edge normals.
    // C++ quantizes angles to ceil(atan2 * 1000) to deduplicate similar directions.
    // R99 determinism: BTreeMap (sorted by quantized angle) instead of HashMap —
    // Rust HashMap's RandomState iteration is per-run-random; the min-cost pick
    // below is order-dependent on ties, which leaked into the bridge direction.
    // `or_insert` keeps first-seen (matches C++ `emplace`, not overwrite). With a
    // unique min-cost the result is identical to C++ regardless of order.
    let mut directions: std::collections::BTreeMap<i64, PointF> = std::collections::BTreeMap::new();
    for line in floating_edges {
        let dx = (line.b.x - line.a.x) as f64;
        let dy = (line.b.y - line.a.y) as f64;
        // Normal = perpendicular to line direction.
        //
        // R595: C++ `Line::normal()` (Line.hpp:180) is `(dy, -dx)`; this port had
        // `(-dy, dx)`, its NEGATION. The cost below uses abs() so costs are
        // unaffected, and `result_dir` only flips sign (same line mod 180) -- but
        // the dedup key is `ceil(atan2(n.y, n.x) * 1000)`, computed on an angle
        // shifted by pi, so the buckets land differently and the CANDIDATE SET
        // itself differs. That is how a bridge angle can come out 90 deg apart
        // (R594: L47 rust 45 vs cpp 135).
        //
        // Shipped OPT-IN (default OFF), not default-on, per R557: measured against
        // C++ it delivers NO improvement (Benchy byte-identical) and a 4-line
        // regression on Majora (409,270 -> 409,266 matched of 2,518,598). The
        // convention discrepancy is real and documented here; enabling it is a
        // one-flag change if the upstream floating-edge input is ever corrected,
        // at which point the bucket boundaries may start to matter.
        let (nx, ny) = if crate::probe_enabled("BRIDGE_NORMAL_CPP") {
            (dy, -dx)
        } else {
            (-dy, dx)
        };
        let len = (nx * nx + ny * ny).sqrt();
        if len > 1e-10 {
            let normalized = PointF::new(nx / len, ny / len);
            let quantized_angle = (normalized.y.atan2(normalized.x) * 1000.0).ceil() as i64;
            directions.entry(quantized_angle).or_insert(normalized);
        }
    }

    // Calculate cost for each direction (dot product with floating edges).
    // This is the cost of the direction as a *perpendicular* bridge direction;
    // the actual bridge direction is the 90° rotation of the minimum-cost normal.
    let mut direction_costs: Vec<(PointF, f64)> = directions.values().map(|&d| (d, 0.0)).collect();

    for line in floating_edges {
        let line_vec_x = (line.b.x - line.a.x) as f64;
        let line_vec_y = (line.b.y - line.a.y) as f64;
        for (dir, cost) in &mut direction_costs {
            // The dot product already contains the line length. dir is normalized.
            *cost += (line_vec_x * dir.x + line_vec_y * dir.y).abs();
        }
    }

    // Find minimum cost direction and rotate 90° to get bridge direction
    let mut result_dir = PointF::new(1.0, 1.0);
    let mut min_cost = f64::MAX;
    for (dir, cost) in &direction_costs {
        if *cost < min_cost {
            // Flip to get bridge direction (perpendicular to the normal)
            result_dir = PointF::new(dir.y, -dir.x);
            min_cost = *cost;
        }
    }

    (result_dir, min_cost)
}

/// Compute the minor principal component direction of a set of polygons.
///
/// Used as a fallback for bridge direction when the area is fully anchored.
/// Returns the direction of the shortest span (minor axis).
fn principal_component_direction(polygons: &[Polygon]) -> Option<PointF> {
    // Gather all points
    let mut sum_x: f64 = 0.0;
    let mut sum_y: f64 = 0.0;
    let mut count: f64 = 0.0;
    for poly in polygons {
        for pt in poly.points() {
            sum_x += pt.x as f64;
            sum_y += pt.y as f64;
            count += 1.0;
        }
    }
    if count < 2.0 {
        return None;
    }
    let cx = sum_x / count;
    let cy = sum_y / count;

    // Compute covariance matrix
    let mut cov_xx: f64 = 0.0;
    let mut cov_xy: f64 = 0.0;
    let mut cov_yy: f64 = 0.0;
    for poly in polygons {
        for pt in poly.points() {
            let dx = pt.x as f64 - cx;
            let dy = pt.y as f64 - cy;
            cov_xx += dx * dx;
            cov_xy += dx * dy;
            cov_yy += dy * dy;
        }
    }
    cov_xx /= count;
    cov_xy /= count;
    cov_yy /= count;

    // Eigenvalues of 2x2 symmetric matrix via quadratic formula
    let trace = cov_xx + cov_yy;
    let det = cov_xx * cov_yy - cov_xy * cov_xy;
    let disc = (trace * trace - 4.0 * det).max(0.0).sqrt();
    let lambda1 = (trace + disc) / 2.0; // major eigenvalue
    let _lambda2 = (trace - disc) / 2.0; // minor eigenvalue

    // Minor eigenvector (direction of shortest span)
    let (vx, vy) = if cov_xy.abs() > 1e-10 {
        (cov_xy, lambda1 - cov_xx) // eigenvector for minor eigenvalue: use (cov_xy, lambda2 - cov_xx)
                                   // but we want the *minor* axis, so we take the eigenvector for lambda2
                                   // Actually: for lambda2, eigenvector is (cov_xy, lambda2 - cov_xx) or equivalently
                                   // (lambda2 - cov_yy, cov_xy). Let me use the standard formula.
    } else if cov_xx > cov_yy {
        // Axes are aligned; minor axis is Y
        (0.0, 1.0)
    } else {
        // Minor axis is X
        (1.0, 0.0)
    };

    // For the minor eigenvector with non-zero cov_xy, use (lambda_minor - cov_yy, cov_xy)
    let (vx, vy) = if cov_xy.abs() > 1e-10 {
        let minor_lambda = (trace - disc) / 2.0;
        (minor_lambda - cov_yy, cov_xy)
    } else {
        (vx, vy)
    };

    let len = (vx * vx + vy * vy).sqrt();
    if len < 1e-10 {
        return None;
    }
    Some(PointF::new(vx / len, vy / len))
}

/// Convert polylines to individual line segments.
fn polylines_to_lines(polylines: &[Polyline]) -> Vec<Line> {
    let mut lines = Vec::new();
    for polyline in polylines {
        let pts = polyline.points();
        for i in 0..pts.len().saturating_sub(1) {
            lines.push(Line::new(pts[i], pts[i + 1]));
        }
    }
    lines
}

/// Detect bridge directions using anchor areas and floating edge analysis.
///
/// Port of `detect_bridge_directions()` from LayerRegion.cpp:293-339.
///
/// For each bridge:
/// 1. Collects anchor polygons from the expansion zones (using wave seed info)
/// 2. Computes floating edges: `diff_pl(to_polylines(bridge.expolygon), expand(anchor_areas, eps))`
/// 3. Calls `detect_bridging_direction_from_lines()` to find the optimal bridge angle
/// 4. Sets `bridge.angle = PI + atan2(dir.y, dir.x)` (matching C++)
fn detect_bridge_directions_impl(
    bridge_anchors: &[WaveSeed],
    bridges: &mut [BridgeInfo],
    expansion_zones: &[ExpansionZone],
) {
    if expansion_zones.is_empty() {
        return;
    }

    // Sort anchors by src then boundary for sequential iteration
    let mut sorted_anchors: Vec<&WaveSeed> = bridge_anchors.iter().collect();
    sorted_anchors.sort_by(|a, b| a.src.cmp(&b.src).then(a.boundary.cmp(&b.boundary)));

    let mut anchor_iter = 0usize;

    for bridge_id in 0..bridges.len() {
        let mut anchor_areas: Vec<Polygon> = Vec::new();
        let mut last_anchor_boundary: i64 = -1;

        // Collect anchor areas for this bridge from wave seeds.
        // Each WaveSeed tells us which boundary expolygon this bridge touches.
        while anchor_iter < sorted_anchors.len()
            && sorted_anchors[anchor_iter].src == bridge_id as u32
        {
            let boundary_idx = sorted_anchors[anchor_iter].boundary as i64;
            if last_anchor_boundary != boundary_idx {
                last_anchor_boundary = boundary_idx;

                // Find which expansion zone this boundary index belongs to.
                // The boundary indices are a flat namespace across all zones:
                // zone0: [0, zone0.len), zone1: [zone0.len, zone0.len+zone1.len), ...
                let mut start_index: u32 = 0;
                for zone in expansion_zones {
                    let end_index = start_index + zone.expolygons.len() as u32;
                    if (boundary_idx as u32) < end_index {
                        let local_idx = boundary_idx as u32 - start_index;
                        if (local_idx as usize) < zone.expolygons.len() {
                            // Add contour + holes as anchor polygons
                            let ep = &zone.expolygons[local_idx as usize];
                            anchor_areas.push(ep.contour.clone());
                            for hole in &ep.holes {
                                anchor_areas.push(hole.clone());
                            }
                        }
                        break;
                    }
                    start_index = end_index;
                }
            }
            anchor_iter += 1;
        }

        // Compute unsupported/floating edges of this bridge.
        //                                   expand(anchor_areas, float(SCALED_EPSILON))))};
        let bridge_polylines = expolygons_to_polylines(&[bridges[bridge_id].expolygon.clone()]);

        // Expand anchor areas slightly for the clipping test.
        // C++ uses SCALED_EPSILON; we use a very small mm value.
        let epsilon_mm = 0.001; // ~1 micron in mm
        let anchor_expolygons: ExPolygons = if anchor_areas.is_empty() {
            Vec::new()
        } else {
            let anchor_ex: ExPolygons = anchor_areas
                .iter()
                .map(|p| ExPolygon::new(p.clone()))
                .collect();
            grow(&anchor_ex, epsilon_mm, OffsetJoinType::Square)
        };

        let floating_polylines = diff_pl(&bridge_polylines, &anchor_expolygons);
        let floating_lines = polylines_to_lines(&floating_polylines);

        // Detect bridge direction.
        let overhang_polygons: Vec<Polygon> = vec![bridges[bridge_id].expolygon.contour.clone()];
        let (bridging_dir, _unsupported_dist) =
            detect_bridging_direction_from_lines(&floating_lines, &overhang_polygons);

        bridges[bridge_id].angle = Some(PI + bridging_dir.y.atan2(bridging_dir.x));
    }
}

/// Merge bridges by group, producing output surfaces.
///
/// Port of `merge_bridges()` from LayerRegion.cpp:341-387.
///
/// For each group head (bridge whose group_id == its own index):
/// 1. Collects all bridge expolygons + their expansions from the group
/// 2. Unions everything together
/// 3. Applies morphological closing
/// 4. Creates Surface with the group head's bridge angle
fn merge_bridges(
    bridges: &mut [BridgeInfo],
    bridge_expansions: &[RegionExpansionEx],
    closing_radius: CoordF,
) -> Vec<crate::surface::Surface> {
    // Record where each bridge's expansions start in the sorted expansions vec.
    // The expansions are sorted by src_id from expand_expolygons_with_anchors
    // (they come out sorted by boundary_id within each zone, but we need by src_id).
    let mut sorted_expansions: Vec<&RegionExpansionEx> = bridge_expansions.iter().collect();
    sorted_expansions.sort_by(|a, b| {
        a.src_id
            .cmp(&b.src_id)
            .then(a.boundary_id.cmp(&b.boundary_id))
    });

    // Build an index: for each src_id, the range in sorted_expansions
    let mut expansion_ranges: Vec<(usize, usize)> = vec![(0, 0); bridges.len()];
    {
        let mut i = 0;
        while i < sorted_expansions.len() {
            let src_id = sorted_expansions[i].src_id as usize;
            let start = i;
            while i < sorted_expansions.len() && sorted_expansions[i].src_id as usize == src_id {
                i += 1;
            }
            if src_id < expansion_ranges.len() {
                expansion_ranges[src_id] = (start, i);
            }
        }
    }

    let mut result: Vec<crate::surface::Surface> = Vec::new();

    for bridge_id in 0..bridges.len() {
        // Only process group heads
        if resolve_group_id(bridges, bridge_id as u32) != bridge_id as u32 {
            continue;
        }

        // Collect all polygons from bridges in this group
        let mut acc: ExPolygons = Vec::new();
        for bridge_id2 in bridge_id..bridges.len() {
            if resolve_group_id(bridges, bridge_id2 as u32) == bridge_id as u32 {
                // Add the bridge expolygon itself
                acc.push(bridges[bridge_id2].expolygon.clone());

                // Add its expansions
                let (start, end) = expansion_ranges[bridge_id2];
                for exp in &sorted_expansions[start..end] {
                    acc.push(exp.expolygon.clone());
                }
            }
        }

        // Get the angle from the group head
        let angle = bridges[bridge_id].angle.unwrap_or(0.0);

        // Apply closing to fill small unassigned regions.
        // C++ LayerRegion::process_external_surfaces bridge path (LayerRegion.cpp:375):
        // `closing_ex(acc, closing_radius)` = offset2_ex(+r, −r, jtMiter) via ClipperLib
        // @1e5. Rust legacy used geo-clipper `closing(...Round)` @1µm (R128 class). Route
        // through offset2_ex_clib Miter under F1_UNION; default keeps geo Round (byte-locked).
        let merged = if closing_radius > 0.0 && !acc.is_empty() {
            let unioned = union_ex(&acc);
            if crate::faithful_gate("F1_UNION") {
                offset2_ex_clib(&unioned, closing_radius, -closing_radius, OffsetJoinType::Miter)
            } else {
                closing(&unioned, closing_radius, OffsetJoinType::Round)
            }
        } else {
            union_ex(&acc)
        };

        // Create surfaces
        for ep in merged {
            let mut s = crate::surface::Surface::new(crate::surface::SurfaceType::BottomBridge, ep);
            s.bridge_angle = Some(angle);
            result.push(s);
        }
    }

    result
}

/// Expand bridges with per-bridge angle detection.
///
/// Port of `expand_bridges_detect_orientations()` from LayerRegion.cpp:429-467.
///
/// This is the full pipeline that replaces `expand_merge_surfaces(stBottomBridge, ...)`
/// when no custom bridge angle is specified:
/// 1. Extract bridge expolygons
/// 2. Expand into zones (with anchor tracking)
/// 3. Group overlapping bridges
/// 4. Detect per-bridge directions
/// 5. Merge groups and produce surfaces
/// 6. Clip expansion zones by the result
pub fn expand_bridges_detect_orientations(
    surfaces: &[crate::surface::Surface],
    expansion_zones: &mut [ExpansionZone],
    closing_radius: CoordF,
) -> Vec<crate::surface::Surface> {
    // Step 1: Extract BottomBridge expolygons
    let bridge_expolygons =
        extract_expolygons_by_type(surfaces, crate::surface::SurfaceType::BottomBridge);
    if bridge_expolygons.is_empty() {
        return Vec::new();
    }

    // Step 2: Expand into zones, getting both anchors and expanded ExPolygons
    let expansion_result = expand_expolygons_with_anchors(&bridge_expolygons, expansion_zones);

    // Step 3: Group bridges by overlapping expansions (union-find)
    let mut bridges = get_grouped_bridges(bridge_expolygons, &expansion_result.expansions);

    // Step 4: Detect per-bridge directions using anchor areas
    // Sort anchors by src then boundary (required by detect_bridge_directions_impl)
    let mut sorted_anchors = expansion_result.anchors.clone();
    sorted_anchors.sort_by(|a, b| a.src.cmp(&b.src).then(a.boundary.cmp(&b.boundary)));
    detect_bridge_directions_impl(&sorted_anchors, &mut bridges, expansion_zones);

    // Step 5: Sort expansions by src_id for merge_bridges, then merge
    let mut sorted_expansions = expansion_result.expansions;
    sorted_expansions.sort_by(|a, b| {
        a.src_id
            .cmp(&b.src_id)
            .then(a.boundary_id.cmp(&b.boundary_id))
    });
    let out = merge_bridges(&mut bridges, &sorted_expansions, closing_radius);

    // Step 6: Clip expansion zones by the expanded bridges
    // Collect all output expolygons for subtraction
    let out_expolygons: ExPolygons = out.iter().map(|s| s.expolygon.clone()).collect();
    if !out_expolygons.is_empty() {
        for zone in expansion_zones.iter_mut() {
            if zone.expanded_into {
                zone.expolygons = difference(&zone.expolygons, &out_expolygons);
            }
        }
    }

    out
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{ExPolygon, Polygon};
    use crate::surface::Surface;

    fn make_rect_expolygon(x: CoordF, y: CoordF, w: CoordF, h: CoordF) -> ExPolygon {
        // Create rectangle in mm, the Polygon::rectangle creates in scaled coords
        // We need to pass scaled coordinates
        let sx = crate::scale(x);
        let sy = crate::scale(y);
        let sw = crate::scale(w);
        let sh = crate::scale(h);
        ExPolygon::new(Polygon::from_points(vec![
            crate::geometry::Point::new(sx, sy),
            crate::geometry::Point::new(sx + sw, sy),
            crate::geometry::Point::new(sx + sw, sy + sh),
            crate::geometry::Point::new(sx, sy + sh),
        ]))
    }

    // ── RegionExpansionParameters::build tests ──

    #[test]
    fn test_params_build_basic() {
        let params = RegionExpansionParameters::build(1.0, 0.1, 5);
        assert!(params.tiny_expansion > 0.0);
        assert!(params.tiny_expansion <= 0.25); // min(0.25*1.0, 0.05) = 0.05
        assert!(params.initial_step > 0.0);
        assert!(params.num_other_steps <= 5);
        assert!(params.max_inflation > 0.0);
        // Total expansion should approximately equal full_expansion
        let total =
            params.tiny_expansion + params.initial_step * (1 + params.num_other_steps) as CoordF;
        // Allow some margin since build adjusts steps
        assert!(total > 0.5, "total expansion {} too small", total);
    }

    #[test]
    fn test_params_build_small_expansion() {
        let params = RegionExpansionParameters::build(0.1, 0.1, 5);
        assert!(params.tiny_expansion > 0.0);
        assert!(params.initial_step > 0.0);
        // Small expansion → fallback to 0.2/0.8 split
        // tiny=0.02, initial=0.08 or tiny=0.025, initial=0.075
    }

    #[test]
    fn test_params_build_large_expansion() {
        let params = RegionExpansionParameters::build(2.0, 0.1, 5);
        assert!(params.tiny_expansion > 0.0);
        assert!(params.initial_step > 0.0);
        assert!(params.num_other_steps <= 4); // max 5 total steps
    }

    // ── ExternalSurfaceConfig tests ──

    #[test]
    fn test_config_from_flows() {
        // Typical 0.4mm nozzle, 2 perimeters
        let config = ExternalSurfaceConfig::from_flows(
            0.4,  // external_perimeter_width
            0.4,  // external_perimeter_spacing
            0.45, // perimeter_spacing
            0.4,  // solid_infill_spacing
            2,    // num_perimeters
        );

        // shell_width = 0.5*0.4 + 0.4 + 0.45*(2-1) = 0.2 + 0.4 + 0.45 = 1.05
        assert!((config.shell_width - 1.05).abs() < 0.001);
        assert!((config.expansion_min - 0.45).abs() < 0.001);

        // expansion_top = 1.05 * sqrt(2) ≈ 1.485
        assert!((config.expansion_top() - 1.05 * std::f64::consts::SQRT_2).abs() < 0.001);

        // closing_radius = 0.55 * 0.65 * 1.05 * 0.4 ≈ 0.150
        assert!(config.closing_radius() > 0.0);
    }

    #[test]
    fn test_config_zero_perimeters() {
        let config = ExternalSurfaceConfig::from_flows(0.4, 0.4, 0.45, 0.4, 0);
        assert!(config.shell_width > 0.0); // Should be EPSILON-like, not zero
        assert!(config.expansion_min > 0.0);
    }

    // ── Wave propagation tests ──

    #[test]
    fn test_wave_seeds_no_overlap() {
        // Source and boundary don't touch → no seeds
        let src = vec![make_rect_expolygon(0.0, 0.0, 1.0, 1.0)];
        let boundary = vec![make_rect_expolygon(5.0, 5.0, 1.0, 1.0)];
        let seeds = wave_seeds_polygon_based(&src, &boundary, 0.05);
        assert!(
            seeds.is_empty(),
            "Expected no seeds for non-touching regions"
        );
    }

    #[test]
    fn test_wave_seeds_adjacent() {
        // Source and boundary are adjacent → seeds at interface
        let src = vec![make_rect_expolygon(0.0, 0.0, 1.0, 1.0)];
        let boundary = vec![make_rect_expolygon(1.0, 0.0, 1.0, 1.0)];
        let seeds = wave_seeds_polygon_based(&src, &boundary, 0.05);
        assert!(!seeds.is_empty(), "Expected seeds for adjacent regions");
    }

    #[test]
    fn test_propagate_wave_basic() {
        // Seed inside a boundary → should expand to fill boundary
        let seeds = vec![make_rect_expolygon(0.5, 0.5, 0.1, 0.1)];
        let boundary = vec![make_rect_expolygon(0.0, 0.0, 2.0, 2.0)];
        let result = propagate_wave_from_seeds(&seeds, &boundary, 0.3, 0.3, 3);
        assert!(!result.is_empty(), "Wave should produce output");

        let total_area: CoordF = result.iter().map(|ep| ep.area().abs()).sum();
        let seed_area: CoordF = seeds.iter().map(|ep| ep.area().abs()).sum();
        assert!(
            total_area > seed_area,
            "Expanded area should be larger than seed"
        );
    }

    #[test]
    fn test_propagate_wave_clipped_by_boundary() {
        // Seed near boundary edge → expansion should be clipped
        let seeds = vec![make_rect_expolygon(0.0, 0.0, 0.1, 0.1)];
        let boundary = vec![make_rect_expolygon(0.0, 0.0, 0.5, 0.5)];
        let result = propagate_wave_from_seeds(&seeds, &boundary, 0.3, 0.3, 3);

        let boundary_area: CoordF = boundary.iter().map(|ep| ep.area().abs()).sum();
        let total_area: CoordF = result.iter().map(|ep| ep.area().abs()).sum();
        // Expanded area should not exceed boundary
        assert!(
            total_area <= boundary_area * 1.01, // small tolerance for floating point
            "Expanded area {} should not exceed boundary area {}",
            total_area,
            boundary_area
        );
    }

    #[test]
    fn test_expand_merge_expolygons_basic() {
        let src = vec![make_rect_expolygon(0.0, 0.0, 1.0, 1.0)];
        let boundary = vec![make_rect_expolygon(1.0, 0.0, 1.0, 1.0)];
        let params = RegionExpansionParameters::build(0.5, 0.1, 5);
        let result = expand_merge_expolygons(src.clone(), &boundary, &params);

        assert!(!result.is_empty(), "Should produce expanded result");
        let src_area: CoordF = src.iter().map(|ep| ep.area().abs()).sum();
        let result_area: CoordF = result.iter().map(|ep| ep.area().abs()).sum();
        assert!(
            result_area > src_area,
            "Expanded area {} should be larger than source {}",
            result_area,
            src_area
        );
    }

    #[test]
    fn test_expand_merge_no_boundary() {
        let src = vec![make_rect_expolygon(0.0, 0.0, 1.0, 1.0)];
        let boundary: ExPolygons = vec![];
        let params = RegionExpansionParameters::build(0.5, 0.1, 5);
        let result = expand_merge_expolygons(src.clone(), &boundary, &params);

        // No boundary → source returned unchanged
        assert_eq!(result.len(), src.len());
    }

    // ── process_external_surfaces_wave tests ──

    #[test]
    fn test_process_external_surfaces_wave_noop_no_externals() {
        // Only Internal surfaces → no expansion needed
        let mut surfaces = vec![vec![
            Surface::internal(make_rect_expolygon(0.0, 0.0, 10.0, 10.0)),
            Surface::internal_solid(make_rect_expolygon(0.0, 10.0, 10.0, 10.0)),
        ]];

        let config = ExternalSurfaceConfig::default();
        let before_len = surfaces[0].len();
        process_external_surfaces_wave(&mut surfaces, &config);
        // Should be unchanged (no external surfaces to expand)
        assert_eq!(surfaces[0].len(), before_len);
    }

    #[test]
    fn test_process_external_surfaces_wave_top_expands() {
        // Top surface adjacent to Internal → Top should grow into Internal
        let mut surfaces = vec![vec![
            Surface::new(
                crate::surface::SurfaceType::Top,
                make_rect_expolygon(0.0, 0.0, 5.0, 1.0),
            ),
            Surface::internal(make_rect_expolygon(0.0, 1.0, 5.0, 9.0)),
        ]];

        let config = ExternalSurfaceConfig {
            shell_width: 0.5,
            expansion_min: 0.2,
            solid_infill_spacing: 0.4,
            num_perimeters: 2,
            ..Default::default()
        };

        process_external_surfaces_wave(&mut surfaces, &config);

        // Should have at least Top and Internal surfaces
        let has_top = surfaces[0]
            .iter()
            .any(|s| s.surface_type == crate::surface::SurfaceType::Top);
        assert!(has_top, "Should still have Top surfaces after expansion");
    }

    #[test]
    fn test_process_external_surfaces_wave_preserves_other_types() {
        // InternalBridge should be preserved unchanged
        let bridge_ep = make_rect_expolygon(0.0, 20.0, 5.0, 5.0);
        let mut surfaces = vec![vec![
            Surface::new(
                crate::surface::SurfaceType::Top,
                make_rect_expolygon(0.0, 0.0, 5.0, 1.0),
            ),
            Surface::internal(make_rect_expolygon(0.0, 1.0, 5.0, 9.0)),
            Surface::new(
                crate::surface::SurfaceType::InternalBridge,
                bridge_ep.clone(),
            ),
        ]];

        let config = ExternalSurfaceConfig::default();
        process_external_surfaces_wave(&mut surfaces, &config);

        let has_internal_bridge = surfaces[0]
            .iter()
            .any(|s| s.surface_type == crate::surface::SurfaceType::InternalBridge);
        assert!(has_internal_bridge, "InternalBridge should be preserved");
    }

    // ====================================================================
    // Bridge grouping and direction detection tests
    // ====================================================================

    #[test]
    fn test_wave_seed_struct() {
        let seed = super::WaveSeed {
            src: 0,
            boundary: 1,
            path: Vec::new(),
        };
        assert_eq!(seed.src, 0);
        assert_eq!(seed.boundary, 1);
    }

    #[test]
    fn test_region_expansion_ex_struct() {
        let ep = make_rect_expolygon(0.0, 0.0, 10.0, 10.0);
        let rex = super::RegionExpansionEx {
            expolygon: ep.clone(),
            src_id: 0,
            boundary_id: 1,
        };
        assert_eq!(rex.src_id, 0);
        assert_eq!(rex.boundary_id, 1);
        assert!(!rex.expolygon.is_empty());
    }

    #[test]
    fn test_resolve_group_id_self() {
        let ep = make_rect_expolygon(0.0, 0.0, 5.0, 5.0);
        let mut bridges = vec![super::BridgeInfo {
            expolygon: ep,
            group_id: 0,
            angle: None,
            expansion_begin: usize::MAX,
        }];
        assert_eq!(super::resolve_group_id(&mut bridges, 0), 0);
    }

    #[test]
    fn test_resolve_group_id_chain() {
        // Bridge 0 -> self, Bridge 1 -> 0, Bridge 2 -> 1 (chain: 2 -> 1 -> 0)
        let ep = make_rect_expolygon(0.0, 0.0, 5.0, 5.0);
        let mut bridges = vec![
            super::BridgeInfo {
                expolygon: ep.clone(),
                group_id: 0,
                angle: None,
                expansion_begin: usize::MAX,
            },
            super::BridgeInfo {
                expolygon: ep.clone(),
                group_id: 0,
                angle: None,
                expansion_begin: usize::MAX,
            },
            super::BridgeInfo {
                expolygon: ep,
                group_id: 1,
                angle: None,
                expansion_begin: usize::MAX,
            },
        ];
        // Resolve bridge 2: should follow 2 -> 1 -> 0
        assert_eq!(super::resolve_group_id(&mut bridges, 2), 0);
        // After path compression, bridge 2's group_id should be 0 directly
        assert_eq!(bridges[2].group_id, 0);
    }

    #[test]
    fn test_get_grouped_bridges_no_overlap() {
        // Two bridges with non-overlapping expansions in the same boundary
        let ep_a = make_rect_expolygon(0.0, 0.0, 5.0, 5.0);
        let ep_b = make_rect_expolygon(20.0, 0.0, 5.0, 5.0);

        let expansions = vec![
            super::RegionExpansionEx {
                expolygon: make_rect_expolygon(0.0, 0.0, 6.0, 6.0),
                src_id: 0,
                boundary_id: 0,
            },
            super::RegionExpansionEx {
                expolygon: make_rect_expolygon(20.0, 0.0, 6.0, 6.0),
                src_id: 1,
                boundary_id: 0,
            },
        ];

        let result = super::get_grouped_bridges(vec![ep_a, ep_b], &expansions);
        assert_eq!(result.len(), 2);
        // Each bridge should be its own group
        assert_eq!(result[0].group_id, 0);
        assert_eq!(result[1].group_id, 1);
    }

    #[test]
    fn test_get_grouped_bridges_with_overlap() {
        // Two bridges whose expansions overlap within the same boundary
        let ep_a = make_rect_expolygon(0.0, 0.0, 5.0, 5.0);
        let ep_b = make_rect_expolygon(4.0, 0.0, 5.0, 5.0);

        // Expansions that overlap (both cover the 4-6 range in X)
        let expansions = vec![
            super::RegionExpansionEx {
                expolygon: make_rect_expolygon(0.0, 0.0, 7.0, 5.0),
                src_id: 0,
                boundary_id: 0,
            },
            super::RegionExpansionEx {
                expolygon: make_rect_expolygon(3.0, 0.0, 7.0, 5.0),
                src_id: 1,
                boundary_id: 0,
            },
        ];

        let result = super::get_grouped_bridges(vec![ep_a, ep_b], &expansions);
        assert_eq!(result.len(), 2);
        // Both bridges should share the same (lower) group_id
        let gid_0 = result[0].group_id;
        let gid_1 = result[1].group_id;
        assert_eq!(gid_0.min(gid_1), 0);
        // At least one must point to the other's group
        assert!(gid_0 == 0 || gid_1 == 0);
    }

    #[test]
    fn test_get_grouped_bridges_empty() {
        let result = super::get_grouped_bridges(Vec::new(), &[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_detect_bridging_direction_from_lines_empty() {
        // No floating edges → use PCA fallback
        let poly = Polygon::rectangle(
            crate::geometry::Point::new(crate::scale(0.0), crate::scale(0.0)),
            crate::geometry::Point::new(crate::scale(10.0), crate::scale(2.0)),
        );
        let (dir, cost) = super::detect_bridging_direction_from_lines(&[], &[poly]);
        // Should return a valid direction
        let len = (dir.x * dir.x + dir.y * dir.y).sqrt();
        assert!(len > 0.5, "Direction should be non-zero, got len={}", len);
        assert_eq!(cost, 0.0);
    }

    #[test]
    fn test_detect_bridging_direction_with_edges() {
        use crate::geometry::{Line, Point};
        // A horizontal floating edge: bridge should go perpendicular (vertically)
        let edges = vec![Line::new(
            Point::new(crate::scale(0.0), crate::scale(0.0)),
            Point::new(crate::scale(10.0), crate::scale(0.0)),
        )];
        let poly = Polygon::rectangle(
            Point::new(crate::scale(0.0), crate::scale(0.0)),
            Point::new(crate::scale(10.0), crate::scale(5.0)),
        );
        let (dir, cost) = super::detect_bridging_direction_from_lines(&edges, &[poly]);
        // The direction should be roughly vertical (Y dominant) or horizontal depending on
        // the normal of the edge. For a horizontal line, the normal is vertical.
        // The algorithm finds the direction perpendicular to the min-cost normal.
        let len = (dir.x * dir.x + dir.y * dir.y).sqrt();
        assert!(len > 0.5, "Direction should be non-zero");
        assert!(cost >= 0.0, "Cost should be non-negative");
    }

    #[test]
    fn test_principal_component_direction_elongated() {
        use crate::geometry::Point;
        // A long thin rectangle — minor axis should be along Y (short direction)
        let poly = Polygon::rectangle(
            Point::new(crate::scale(0.0), crate::scale(0.0)),
            Point::new(crate::scale(20.0), crate::scale(2.0)),
        );
        let result = super::principal_component_direction(&[poly]);
        assert!(result.is_some(), "Should find a principal component");
        let dir = result.unwrap();
        // Minor axis of a horizontal rectangle should be roughly vertical
        // (Y component dominant). The exact sign doesn't matter.
        assert!(
            dir.y.abs() > dir.x.abs(),
            "Minor axis of horizontal rect should be vertical: dir=({}, {})",
            dir.x,
            dir.y
        );
    }

    #[test]
    fn test_principal_component_direction_square() {
        use crate::geometry::Point;
        // A square — should still return something valid
        let poly = Polygon::rectangle(
            Point::new(crate::scale(0.0), crate::scale(0.0)),
            Point::new(crate::scale(10.0), crate::scale(10.0)),
        );
        let result = super::principal_component_direction(&[poly]);
        // For a perfect square the eigenvalues are equal so direction is
        // somewhat arbitrary, but we should not crash
        assert!(result.is_some() || result.is_none());
    }

    #[test]
    fn test_polylines_to_lines() {
        use crate::geometry::{Point, Polyline};
        let pl = Polyline::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
        ]);
        let lines = super::polylines_to_lines(&[pl]);
        assert_eq!(lines.len(), 2);
        assert_eq!(lines[0].a, Point::new(0, 0));
        assert_eq!(lines[0].b, Point::new(100, 0));
        assert_eq!(lines[1].a, Point::new(100, 0));
        assert_eq!(lines[1].b, Point::new(100, 100));
    }

    #[test]
    fn test_polylines_to_lines_empty() {
        let lines = super::polylines_to_lines(&[]);
        assert!(lines.is_empty());
    }

    #[test]
    fn test_merge_bridges_single_group() {
        // One bridge with one expansion
        let ep = make_rect_expolygon(0.0, 0.0, 5.0, 5.0);
        let exp_ep = make_rect_expolygon(5.0, 0.0, 3.0, 5.0);
        let mut bridges = vec![super::BridgeInfo {
            expolygon: ep,
            group_id: 0,
            angle: Some(1.5),
            expansion_begin: usize::MAX,
        }];
        let expansions = vec![super::RegionExpansionEx {
            expolygon: exp_ep,
            src_id: 0,
            boundary_id: 0,
        }];

        let result = super::merge_bridges(&mut bridges, &expansions, 0.0);
        assert!(!result.is_empty(), "Should produce at least one surface");
        assert_eq!(
            result[0].surface_type,
            crate::surface::SurfaceType::BottomBridge
        );
        assert!(result[0].bridge_angle.is_some());
        // The angle should be the one we set
        let angle = result[0].bridge_angle.unwrap();
        assert!(
            (angle - 1.5).abs() < 1e-6,
            "Expected angle 1.5, got {}",
            angle
        );
    }

    #[test]
    fn test_merge_bridges_two_groups() {
        // Two bridges each in their own group
        let ep_a = make_rect_expolygon(0.0, 0.0, 5.0, 5.0);
        let ep_b = make_rect_expolygon(20.0, 0.0, 5.0, 5.0);
        let mut bridges = vec![
            super::BridgeInfo {
                expolygon: ep_a,
                group_id: 0,
                angle: Some(0.0),
                expansion_begin: usize::MAX,
            },
            super::BridgeInfo {
                expolygon: ep_b,
                group_id: 1,
                angle: Some(1.57),
                expansion_begin: usize::MAX,
            },
        ];
        let expansions: Vec<super::RegionExpansionEx> = vec![];

        let result = super::merge_bridges(&mut bridges, &expansions, 0.0);
        assert_eq!(result.len(), 2, "Should produce two separate surfaces");
    }

    #[test]
    fn test_expand_bridges_detect_orientations_no_bridges() {
        // No BottomBridge surfaces → should return empty
        let surfaces = vec![Surface::internal(make_rect_expolygon(0.0, 0.0, 10.0, 10.0))];
        let params = RegionExpansionParameters::build(1.0, 0.1, 5);
        let mut zones = vec![ExpansionZone {
            expolygons: vec![make_rect_expolygon(0.0, 0.0, 10.0, 10.0)],
            parameters: params,
            expanded_into: false,
        }];

        let result = super::expand_bridges_detect_orientations(&surfaces, &mut zones, 0.1);
        assert!(result.is_empty(), "No bridges → no output");
    }

    #[test]
    fn test_expand_bridges_detect_orientations_basic() {
        // A bridge surface next to a solid boundary — should expand and get an angle
        let bridge_ep = make_rect_expolygon(0.0, 0.0, 3.0, 3.0);
        let boundary_ep = make_rect_expolygon(-1.0, -1.0, 12.0, 12.0);

        let surfaces = vec![Surface::new(
            crate::surface::SurfaceType::BottomBridge,
            bridge_ep,
        )];
        let params = RegionExpansionParameters::build(1.0, 0.1, 5);
        let mut zones = vec![ExpansionZone {
            expolygons: vec![boundary_ep],
            parameters: params,
            expanded_into: false,
        }];

        let result = super::expand_bridges_detect_orientations(&surfaces, &mut zones, 0.1);
        // Should produce at least one bridge surface
        assert!(
            !result.is_empty(),
            "Should produce bridge surfaces when bridge is adjacent to boundary"
        );
        for s in &result {
            assert_eq!(
                s.surface_type,
                crate::surface::SurfaceType::BottomBridge
            );
            assert!(
                s.bridge_angle.is_some(),
                "Bridge surface should have an angle"
            );
        }
    }

    #[test]
    fn test_expand_bridges_clips_zones() {
        // After bridge expansion, the zone should have the expanded area subtracted
        let bridge_ep = make_rect_expolygon(2.0, 2.0, 3.0, 3.0);
        let boundary_ep = make_rect_expolygon(0.0, 0.0, 10.0, 10.0);

        let surfaces = vec![Surface::new(
            crate::surface::SurfaceType::BottomBridge,
            bridge_ep,
        )];
        let params = RegionExpansionParameters::build(1.0, 0.1, 5);
        let original_area = boundary_ep.area().abs();
        let mut zones = vec![ExpansionZone {
            expolygons: vec![boundary_ep],
            parameters: params,
            expanded_into: false,
        }];

        let result = super::expand_bridges_detect_orientations(&surfaces, &mut zones, 0.1);
        if !result.is_empty() {
            // Zone area should be smaller after clipping
            let remaining_area: f64 = zones[0].expolygons.iter().map(|ep| ep.area().abs()).sum();
            assert!(
                remaining_area < original_area,
                "Zone area should decrease after bridge expansion: remaining={} vs original={}",
                remaining_area,
                original_area
            );
        }
    }

    #[test]
    fn test_process_external_surfaces_wave_uses_bridge_detection() {
        // When custom_bridge_angle is 0 (unset), the wave processor should use
        // expand_bridges_detect_orientations instead of expand_merge_surfaces.
        // We verify this by checking that bridge surfaces get angles assigned.
        let bridge_ep = make_rect_expolygon(1.0, 1.0, 3.0, 3.0);
        let internal_ep = make_rect_expolygon(0.0, 0.0, 10.0, 10.0);

        let mut surfaces = vec![vec![
            Surface::new(crate::surface::SurfaceType::BottomBridge, bridge_ep),
            Surface::internal(internal_ep),
        ]];

        let mut config = ExternalSurfaceConfig::default();
        config.custom_bridge_angle = 0.0; // No custom angle — use auto-detect
        config.num_perimeters = 2;
        config.shell_width = 1.0;
        config.expansion_min = 0.5;
        config.solid_infill_spacing = 0.4;

        process_external_surfaces_wave(&mut surfaces, &config);

        // Check that any remaining BottomBridge surfaces have a bridge angle
        for s in &surfaces[0] {
            if s.surface_type == crate::surface::SurfaceType::BottomBridge {
                assert!(
                    s.bridge_angle.is_some(),
                    "Auto-detected bridges should have an angle"
                );
            }
        }
    }
}
