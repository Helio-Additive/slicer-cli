//! Port of `libslic3r/AdaptiveLayerMerge.hpp` and `AdaptiveLayerMerge.cpp`.
//!
//! The layer stack of a print with several nozzle diameters is sliced for the smallest nozzle; a
//! larger nozzle prints a band of those layers as one extrusion. This unit holds the band planner
//! (which layers go into which band) and the helpers that decide what a nozzle can extrude.
//!
//! C++ reference: `references/BambuStudio/src/libslic3r/AdaptiveLayerMerge.{hpp,cpp}`.

use std::sync::Arc;

use crate::clipper_utils::{
    difference_clib, intersection_clib, offset2_ex_clib, offset_expolygons_clib_scaled,
    union_ex_clib, OffsetJoinType,
};
use crate::geometry::{ExPolygon, Polygon};
use crate::libslic3r::EPSILON;
use crate::surface::Surface;

type ExPolygons = Vec<ExPolygon>;
type Polygons = Vec<Polygon>;

/// libslic3r.h:58 — `static constexpr double SCALING_FACTOR = 0.00001;`
const SCALING_FACTOR: f64 = 0.00001;

/// `ClipperLib::pftNonZero`, the default fill type of `union_ex(const Polygons &)`
/// (ClipperUtils.hpp:554), in the numbering `union_ex_clib` takes.
const PFT_NON_ZERO: i32 = 1;

/// AdaptiveLayerMerge.hpp:17-23
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AdaptiveLayerHeightRange {
    pub min_height: f64,
    pub max_height: f64,
}

impl AdaptiveLayerHeightRange {
    /// AdaptiveLayerMerge.cpp:14-17
    pub fn contains(&self, height: f64) -> bool {
        height >= self.min_height - EPSILON && height <= self.max_height + EPSILON
    }
}

/// AdaptiveLayerMerge.hpp:25-65
#[derive(Debug, Clone, Default)]
pub struct AdaptiveMergePatch {
    // A region may carry multiple disjoint patches on one group-top layer. They keep the nozzle's common
    // top_idx but may start at different bottom_idx values after local Z-conflict clipping.
    pub bottom_idx: usize,
    pub top_idx: usize,
    // Only set on the copy owned by the group-top LayerRegion. Layer::id() may be offset against the
    // PrintObject layer index (raft layers, bottom layers dropped by fix_slicing_errors), so the role
    // has to be stored explicitly instead of being derived from top_idx.
    pub group_top: bool,
    pub merged_height: f64,

    // All copies stored across a merge span share the same immutable geometry.
    m_area: Option<Arc<ExPolygons>>,
    m_suppressed_area: Option<Arc<ExPolygons>>,
}

impl AdaptiveMergePatch {
    /// AdaptiveLayerMerge.hpp:37-42
    ///
    /// The retained contour is extruded on the group top and marks the swallowed footprint on lower layers.
    pub fn area(&self) -> &ExPolygons {
        static EMPTY: ExPolygons = Vec::new();
        match &self.m_area {
            Some(area) => area,
            None => &EMPTY,
        }
    }

    /// AdaptiveLayerMerge.hpp:44
    pub fn set_area(&mut self, area: ExPolygons) {
        self.m_area = Some(Arc::new(area));
    }

    /// AdaptiveLayerMerge.hpp:48-52
    ///
    /// Geometry that could not be assigned to a printable slab. It is intentionally separate from area():
    /// the patch must suppress fill in this footprint without generating walls or claiming Z occupancy.
    pub fn suppressed_area(&self) -> &ExPolygons {
        static EMPTY: ExPolygons = Vec::new();
        match &self.m_suppressed_area {
            Some(area) => area,
            None => &EMPTY,
        }
    }

    /// AdaptiveLayerMerge.hpp:54-57
    pub fn set_suppressed_area(&mut self, area: ExPolygons) {
        self.m_suppressed_area = Some(Arc::new(area));
    }

    /// AdaptiveLayerMerge.hpp:59
    pub fn has_suppressed_area(&self) -> bool {
        self.m_suppressed_area.is_some()
    }
}

/// AdaptiveLayerMerge.hpp:67-76
///
/// One band of the layer stack, [bottom_idx, top_idx] inclusive. A nozzle prints the band as a single
/// extrusion of merged_height, placed on the top layer; the layers below it are dropped.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct AdaptiveLayerBand {
    // Ordinary bands are shaped by top_idx. The first band is special: callers use the true first-layer
    // contour and color, and top_idx only selects the physical Z where that snapshot is extruded.
    pub bottom_idx: usize,
    pub top_idx: usize,
    pub merged_height: f64,
}

/// AdaptiveLayerMerge.cpp:19-27
///
/// The printable layer height window of a single nozzle diameter, derived from the diameter itself:
/// 0.2 -> 0.04..0.14, 0.4 -> 0.08..0.28, 0.6 -> 0.12..0.42, 0.8 -> 0.16..0.56.
pub fn adaptive_layer_height_range(nozzle_diameter: f64) -> AdaptiveLayerHeightRange {
    AdaptiveLayerHeightRange { min_height: 0.2 * nozzle_diameter, max_height: 0.7 * nozzle_diameter }
}

/// AdaptiveLayerMerge.cpp:29-37
///
/// Top gap between support and the object above it for a nozzle diameter. Only 0.2 / 0.4 / 0.6 / 0.8 ship;
/// anything else takes the nearest one.
pub fn support_top_z_for_nozzle(nozzle_diameter: f64) -> f32 {
    const TABLE: [(f64, f32); 4] = [(0.2, 0.1), (0.4, 0.2), (0.6, 0.2), (0.8, 0.4)];
    let mut best = &TABLE[0];
    for entry in &TABLE {
        if (entry.0 - nozzle_diameter).abs() < (best.0 - nozzle_diameter).abs() {
            best = entry;
        }
    }
    best.1
}

/// AdaptiveLayerMerge.cpp:39-48
///
/// Keep only the pieces a nozzle of this diameter could really lay a bead into. The area test comes
/// first and the erosion is read as a yes/no answer only: the returned pieces are always the
/// originals, never the output of the offset.
pub fn adaptive_printable_pieces(mut pieces: ExPolygons, nozzle_diameter: f64) -> ExPolygons {
    let min_area: f64 = 0.25 * nozzle_diameter * nozzle_diameter;
    // `scale_` is the bare macro `((val) / SCALING_FACTOR)` here: a double, narrowed by `float()`,
    // with no `coord_t` in between.
    let erosion: f32 = ((0.5 * nozzle_diameter) / SCALING_FACTOR) as f32;
    pieces.retain(|piece| {
        !(piece.area() * SCALING_FACTOR * SCALING_FACTOR < min_area
            || (erosion > 0.0 && shrink_ex(std::slice::from_ref(piece), erosion).is_empty()))
    });
    pieces
}

/// AdaptiveLayerMerge.cpp:50-64
///
/// Like adaptive_printable_pieces(), but also trims every kept piece down to the part a bead of this nozzle
/// fits into.
pub fn adaptive_printable_core(pieces: ExPolygons, nozzle_diameter: f64) -> ExPolygons {
    // The area / erosion test runs first, so degenerate slivers never reach the offset below.
    let pieces = adaptive_printable_pieces(pieces, nozzle_diameter);
    let radius: f32 = ((0.5 * nozzle_diameter) / SCALING_FACTOR) as f32;
    if pieces.is_empty() || radius <= 0.0 {
        return pieces;
    }
    let mut core: ExPolygons = Vec::new();
    for piece in &pieces {
        // Clamp to the original piece: the opening only removes what no bead fits into.
        let opened =
            intersection_clib(&opening_ex(std::slice::from_ref(piece), radius), std::slice::from_ref(piece));
        core.extend(opened);
    }
    adaptive_printable_pieces(core, nozzle_diameter)
}

/// ClipperUtils.hpp:392-393 — `shrink_ex(const ExPolygons &, const float delta)` with the default
/// join type and miter limit: `offset_ex(polygons, -delta, joinType, miterLimit)`.
fn shrink_ex(expolygons: &[ExPolygon], delta: f32) -> ExPolygons {
    offset_expolygons_clib_scaled(expolygons, f64::from(-delta), OffsetJoinType::Miter)
}

/// ClipperUtils.hpp:428-429 — `opening_ex(const ExPolygons &, const float delta)` with the default
/// join type and miter limit: `offset2_ex(polygons, -delta, delta, joinType, miterLimit)`.
fn opening_ex(expolygons: &[ExPolygon], delta: f32) -> ExPolygons {
    // `offset2_ex_clib` takes millimetres and hands ClipperOffset `float(mm / SCALING_FACTOR)`.
    // A float times SCALING_FACTOR, divided by it again, is within a few double ulps of the float
    // and narrows back to it, so ClipperOffset receives `delta` exactly.
    let delta_mm = f64::from(delta) * SCALING_FACTOR;
    assert!((delta_mm / SCALING_FACTOR) as f32 == delta, "opening_ex: delta {delta} does not survive the millimetre round trip");
    offset2_ex_clib(expolygons, -delta_mm, delta_mm, OffsetJoinType::Miter)
}

/// What `adaptive_suppressed_orphan_area` reads of one `LayerRegion` of the layer.
///
/// FIDELITY-NOTE: the C++ takes `const Layer &` and reads `layerm->adaptive_merge_patches` and
/// `layerm->slices.surfaces` of every region (Layer.hpp at the pin). The crate's `LayerRegion`
/// (layer.rs) has no `adaptive_merge_patches` member yet, so the two members travel in this
/// struct; once the member exists the caller builds one of these per region, in region order.
#[derive(Debug, Clone, Copy)]
pub struct AdaptiveSuppressedOrphanRegion<'a> {
    /// `LayerRegion::adaptive_merge_patches`
    pub adaptive_merge_patches: &'a [AdaptiveMergePatch],
    /// `LayerRegion::slices.surfaces`
    pub slices: &'a [Surface],
}

/// AdaptiveLayerMerge.cpp:66-88
///
/// The footprint of a layer whose orphans were suppressed: sliced, but extruded by no region. What rests on it
/// is not supported there.
pub fn adaptive_suppressed_orphan_area(regions: &[AdaptiveSuppressedOrphanRegion<'_>]) -> ExPolygons {
    let mut suppressed: Polygons = Vec::new();
    let mut extruded: Polygons = Vec::new();
    for layerm in regions {
        if layerm.adaptive_merge_patches.is_empty() {
            extruded.extend(crate::surface::to_polygons(layerm.slices));
            continue;
        }
        for patch in layerm.adaptive_merge_patches {
            // Only an orphan carries an explicit footprint. The whole-region marker of a band is left out: the
            // region's material there lies under another extrusion, which is what stands on that step.
            if patch.group_top && patch.area().is_empty() && patch.has_suppressed_area() {
                polygons_append_expolygons(&mut suppressed, patch.suppressed_area());
            } else {
                polygons_append_expolygons(&mut extruded, patch.area());
            }
        }
    }
    if suppressed.is_empty() {
        return Vec::new();
    }
    // A slab spanning this step may still stand where another region lost its orphan.
    if extruded.is_empty() {
        union_ex_clib(&suppressed, PFT_NON_ZERO)
    } else {
        difference_clib(&union_ex_clib(&suppressed, PFT_NON_ZERO), &union_ex_clib(&extruded, PFT_NON_ZERO))
    }
}

/// `polygons_append(dst, to_polygons(const ExPolygons &))` (ExPolygon.hpp): the contour of every
/// expolygon followed by its holes.
fn polygons_append_expolygons(dst: &mut Polygons, src: &[ExPolygon]) {
    for expolygon in src {
        dst.push(expolygon.contour.clone());
        dst.extend(expolygon.holes.iter().cloned());
    }
}

/// AdaptiveLayerMerge.cpp:90-103
///
/// True when the filaments handed in do not all share one nozzle diameter. filament_ids are 0 based; pass
/// the filaments the print actually uses.
///
/// FIDELITY-NOTE: the C++ takes `const PrintConfig &print_config` and calls
/// `nozzle_diameter_for_filament(print_config, filament_id + 1)` (PrintConfig.cpp:454-469). That
/// function reads `filament_diameter_map` and the per-nozzle `nozzle_diameter` vector; the crate's
/// `PrintConfig` has neither (its `nozzle_diameter` is one scalar) and no
/// `nozzle_diameter_for_filament`. The lookup is therefore the first argument: a function of the
/// 1 based filament id that returns the `float` the C++ function returns.
pub fn has_multiple_used_nozzle_diameters(
    nozzle_diameter_for_filament: &dyn Fn(u32) -> f32,
    filament_ids: &[u32],
) -> bool {
    let mut first: f64 = 0.0;
    for &filament_id in filament_ids {
        let diameter: f64 = f64::from(nozzle_diameter_for_filament(filament_id.wrapping_add(1)));
        if diameter <= EPSILON {
            continue;
        }
        if first <= EPSILON {
            first = diameter;
        } else if (diameter - first).abs() > EPSILON {
            return true;
        }
    }
    false
}

/// AdaptiveLayerMerge.cpp:105-122
///
/// How many base layers one nozzle merges into a single extrusion. Returns 1 when the nozzle has to print
/// every layer on its own.
pub fn adaptive_layer_band_size(nozzle_diameter: f64, min_nozzle_diameter: f64, base_layer_height: f64) -> usize {
    if nozzle_diameter <= EPSILON || min_nozzle_diameter <= EPSILON || base_layer_height <= EPSILON {
        return 1;
    }

    let range = adaptive_layer_height_range(nozzle_diameter);
    // The layer stack is sliced for the smallest nozzle, so the diameter ratio is the number of layers a
    // larger nozzle covers with one extrusion.
    let mut size: usize = f64::max(1.0, (nozzle_diameter / min_nozzle_diameter + 0.5).floor()) as usize;
    // A base layer height chosen well below what the smallest nozzle needs leaves the ratio too thin to
    // extrude, so the band grows until the nozzle can lay it down in one go.
    while (size as f64).mul_add(base_layer_height, EPSILON) < range.min_height {
        size += 1;
    }
    // The window wins over the ratio: a band the nozzle cannot extrude is worse than a thinner one.
    while size > 1 && size as f64 * base_layer_height > range.max_height + EPSILON {
        size -= 1;
    }
    if (size as f64).mul_add(base_layer_height, EPSILON) < range.min_height {
        1
    } else {
        size
    }
}

/// AdaptiveLayerMerge.cpp:124-173
///
/// Cover [bottom_idx, top_idx] with consecutive bands, every one of them inside the printable window and as
/// close to band_size layers as the window allows. Returns false when no legal cover exists, which means the
/// span is thinner than a single extrusion of this nozzle.
fn cover_span_with_bands(
    layer_heights: &[f64],
    bottom_idx: usize,
    top_idx: usize,
    band_size: usize,
    height_range: &AdaptiveLayerHeightRange,
    bands: &mut Vec<AdaptiveLayerBand>,
) -> bool {
    bands.clear();
    if bottom_idx > top_idx || top_idx >= layer_heights.len() {
        return false;
    }

    let layer_count = top_idx - bottom_idx + 1;
    let unreachable = usize::MAX;
    // cost[i] is how far the bands covering the first i layers stray from band_size in total.
    let mut cost: Vec<usize> = vec![unreachable; layer_count + 1];
    let mut prev: Vec<usize> = vec![0; layer_count + 1];
    cost[0] = 0;
    for end in 1..=layer_count {
        let mut height: f64 = 0.0;
        let mut begin = end;
        while begin > 0 {
            begin -= 1;
            height += layer_heights[bottom_idx + begin];
            if height > height_range.max_height + EPSILON {
                break;
            }
            if cost[begin] == unreachable || !height_range.contains(height) {
                continue;
            }
            let size = end - begin;
            let penalty = if size > band_size { size - band_size } else { band_size - size };
            // Ties go to the band found last, i.e. to the thickest one, which is also the fastest.
            if cost[begin].wrapping_add(penalty) <= cost[end] {
                cost[end] = cost[begin].wrapping_add(penalty);
                prev[end] = begin;
            }
        }
    }
    if cost[layer_count] == unreachable {
        return false;
    }

    let mut end = layer_count;
    while end > 0 {
        let mut band =
            AdaptiveLayerBand { bottom_idx: bottom_idx + prev[end], top_idx: bottom_idx + end - 1, merged_height: 0.0 };
        for layer_idx in band.bottom_idx..=band.top_idx {
            band.merged_height += layer_heights[layer_idx];
        }
        bands.push(band);
        end = prev[end];
    }
    bands.reverse();
    true
}

/// AdaptiveLayerMerge.cpp:175-227
///
/// Same cost as cover_span_with_bands(), but a span starting at the object base is laid out from the
/// bottom up: on a tie a regular band_size band wins, so the remainder of the stack ends up at the top and
/// the first layer of a coarse nozzle no longer depends on the object height.
fn cover_span_from_base(
    layer_heights: &[f64],
    bottom_idx: usize,
    top_idx: usize,
    band_size: usize,
    height_range: &AdaptiveLayerHeightRange,
    bands: &mut Vec<AdaptiveLayerBand>,
) -> bool {
    if bottom_idx != 0 {
        return cover_span_with_bands(layer_heights, bottom_idx, top_idx, band_size, height_range, bands);
    }

    bands.clear();
    if bottom_idx > top_idx || top_idx >= layer_heights.len() {
        return false;
    }

    let layer_count = top_idx - bottom_idx + 1;
    let unreachable = usize::MAX;
    // cost[i] is how far the bands covering the layers from i up to the span top stray from band_size.
    let mut cost: Vec<usize> = vec![unreachable; layer_count + 1];
    let mut next: Vec<usize> = vec![0; layer_count + 1];
    cost[layer_count] = 0;
    let mut begin = layer_count;
    while begin > 0 {
        begin -= 1;
        let mut height: f64 = 0.0;
        for end in begin + 1..=layer_count {
            height += layer_heights[bottom_idx + end - 1];
            if height > height_range.max_height + EPSILON {
                break;
            }
            if cost[end] == unreachable || !height_range.contains(height) {
                continue;
            }
            let size = end - begin;
            let penalty = if size > band_size { size - band_size } else { band_size - size };
            let total = cost[end].wrapping_add(penalty);
            // Ties keep a regular band, otherwise go to the thickest one, which is also the fastest.
            // `next[begin] - begin` is unsigned in the C++ and wraps while next[begin] is still 0.
            if total < cost[begin] || (total == cost[begin] && next[begin].wrapping_sub(begin) != band_size) {
                cost[begin] = total;
                next[begin] = end;
            }
        }
    }
    if cost[0] == unreachable {
        return false;
    }

    let mut begin = 0;
    while begin < layer_count {
        let mut band =
            AdaptiveLayerBand { bottom_idx: bottom_idx + begin, top_idx: bottom_idx + next[begin] - 1, merged_height: 0.0 };
        for layer_idx in band.bottom_idx..=band.top_idx {
            band.merged_height += layer_heights[layer_idx];
        }
        bands.push(band);
        begin = next[begin];
    }
    true
}

/// AdaptiveLayerMerge.cpp:229-284
///
/// Split the complete layer stack into bands of band_size layers. band_tops optionally marks hard
/// structural boundaries that a band must not straddle; pass an empty slice for the normal fixed grid
/// (the C++ default argument). Returns an empty plan when band_size is 1, i.e. when this nozzle merges
/// nothing.
pub fn build_adaptive_layer_bands(
    layer_heights: &[f64],
    band_size: usize,
    height_range: &AdaptiveLayerHeightRange,
    band_tops: &[u8],
) -> Vec<AdaptiveLayerBand> {
    let mut bands: Vec<AdaptiveLayerBand> = Vec::new();
    // The first band starts at the object base. It is still extruded on its top layer, with a height equal
    // to the complete span, so its lower face stays on the bed or on the object's raft/support substrate.
    // Callers propagate that object-base contact separately from the physical group-top layer id.
    let first_layer_idx: usize = 0;
    if band_size < 2 || layer_heights.len() < 2 {
        return bands;
    }

    let last_idx = layer_heights.len() - 1;
    // Every marked layer closes a segment, and so does the top of the stack. Bands never straddle a
    // segment boundary, which is what keeps a marked layer from being swallowed and decimated away.
    let mut segment_tops: Vec<usize> = Vec::new();
    for layer_idx in first_layer_idx..last_idx {
        if layer_idx < band_tops.len() && band_tops[layer_idx] != 0 {
            segment_tops.push(layer_idx);
        }
    }
    segment_tops.push(last_idx);

    let mut segment_bands: Vec<AdaptiveLayerBand> = Vec::new();
    let mut bottom_idx = first_layer_idx;
    let mut last_bottom_idx = first_layer_idx;
    let mut last_band_count: usize = 0;
    for i in 0..segment_tops.len() {
        let top_idx = segment_tops[i];
        if cover_span_from_base(layer_heights, bottom_idx, top_idx, band_size, height_range, &mut segment_bands) {
            last_bottom_idx = bottom_idx;
            last_band_count = segment_bands.len();
            bands.extend_from_slice(&segment_bands);
            bottom_idx = top_idx + 1;
            continue;
        }
        // Too thin for this nozzle to extrude on its own. Printability outranks the mark, so the segment
        // gives it up and joins the one above by simply not advancing bottom_idx.
        if i + 1 < segment_tops.len() {
            continue;
        }
        // There is nothing above the last segment to join, so take the segment below back and re-cut both
        // at once. This is what keeps the top of the object merged when its tail is shorter than a band.
        if last_band_count > 0
            && cover_span_from_base(layer_heights, last_bottom_idx, top_idx, band_size, height_range, &mut segment_bands)
        {
            bands.truncate(bands.len() - last_band_count);
            bands.extend_from_slice(&segment_bands);
        }
        // Otherwise the whole stack is thinner than one extrusion of this nozzle and nothing merges.
    }
    // A cover may need single-layer groups to bridge a stretch the window cannot span, but such a group is
    // what not merging looks like, so it is not reported as a band.
    bands.retain(|band| !(band.top_idx <= band.bottom_idx));
    bands
}

/// AdaptiveLayerMerge.cpp:286-317
///
/// Pull top_idx down to the last layer in the band that still has material, and recompute merged_height.
/// Returns false when no such layer exists, the leftover is a single step, or the shortened height is
/// outside the printable window. has_material is indexed by layer; it must cover [bottom_idx, top_idx].
pub fn pull_adaptive_band_top_to_material(
    band: &mut AdaptiveLayerBand,
    layer_heights: &[f64],
    height_range: &AdaptiveLayerHeightRange,
    has_material: &[u8],
) -> bool {
    if band.bottom_idx > band.top_idx || band.top_idx >= layer_heights.len() || band.top_idx >= has_material.len() {
        return false;
    }

    // The grid already placed a printable band here. Only the empty-top case needs a shorter top.
    if has_material[band.top_idx] != 0 {
        return true;
    }

    let mut top = band.top_idx;
    while top > band.bottom_idx && has_material[top] == 0 {
        top -= 1;
    }
    if has_material[top] == 0 {
        return false;
    }

    let mut height: f64 = 0.0;
    for layer_idx in band.bottom_idx..=top {
        height += layer_heights[layer_idx];
    }
    // A single leftover step is not a band: it is what not merging looks like, and it is often
    // thinner than this nozzle can extrude. The caller then prints that layer unmerged, if at all.
    if top <= band.bottom_idx || !height_range.contains(height) {
        return false;
    }

    band.top_idx = top;
    band.merged_height = height;
    true
}

/// AdaptiveLayerMerge.cpp:319-363
///
/// Cut a span of layers into consecutive printable groups that stay as close as possible to the nozzle's
/// fixed band size. Returns false when no split works, which means the span cannot be extruded by this nozzle.
pub fn split_adaptive_layer_span(
    layer_heights: &[f64],
    bottom_idx: usize,
    top_idx: usize,
    preferred_band_size: usize,
    height_range: &AdaptiveLayerHeightRange,
    groups: &mut Vec<(usize, usize)>,
) -> bool {
    groups.clear();
    if bottom_idx > top_idx || top_idx >= layer_heights.len() || preferred_band_size == 0 {
        return false;
    }
    let layer_count = top_idx - bottom_idx + 1;
    // cost[i] is the total distance from the preferred band size over the first i layers.
    let unreachable = usize::MAX;
    let mut cost: Vec<usize> = vec![unreachable; layer_count + 1];
    let mut prev: Vec<usize> = vec![0; layer_count + 1];
    cost[0] = 0;
    for end in 1..=layer_count {
        let mut height: f64 = 0.0;
        let mut begin = end;
        while begin > 0 {
            begin -= 1;
            height += layer_heights[bottom_idx + begin];
            if height > height_range.max_height + EPSILON {
                break;
            }
            if cost[begin] == unreachable || !height_range.contains(height) {
                continue;
            }
            let size = end - begin;
            let penalty = if size > preferred_band_size { size - preferred_band_size } else { preferred_band_size - size };
            // Prefer the thicker final band when two covers deviate equally from the fixed grid.
            if cost[begin].wrapping_add(penalty) <= cost[end] {
                cost[end] = cost[begin].wrapping_add(penalty);
                prev[end] = begin;
            }
        }
    }
    if cost[layer_count] == unreachable {
        return false;
    }

    let mut end = layer_count;
    while end > 0 {
        groups.push((bottom_idx + prev[end], bottom_idx + end - 1));
        end = prev[end];
    }
    groups.reverse();
    true
}
