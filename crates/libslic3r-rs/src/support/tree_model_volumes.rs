// Tree supports by Thomas Rahm, losely based on Tree Supports by CuraEngine.
// Original source of Thomas Rahm's tree supports:
// https://github.com/ThomasRahm/CuraEngine
//
// Original CuraEngine copyright:
// Copyright (c) 2021 Ultimaker B.V.
// CuraEngine is released under the terms of the AGPLv3 or higher.
//
//! Faithful 1:1 Rust port of BambuStudio's
//! `src/libslic3r/Support/TreeModelVolumes.{hpp,cpp}` (namespace
//! `Slic3r::TreeSupport3D`).
//!
//! `coord_t` -> [`Coord`] (i64), `coordf_t`/`double` -> [`CoordF`] (f64),
//! `LayerIndex` -> [`LayerIndex`] (i32), `Polygons` -> `Vec<Polygon>`,
//! `ExPolygons` -> `Vec<ExPolygon>`.
//!
//! Divergences from the reference (documented inline at their site):
//!  * TBB `parallel_for` / `task_group` are executed serially. The union/diff
//!    results are order-independent for the cache contents, and the C++ already
//!    sorts the outline indices by size to make the serial-equivalent union
//!    deterministic, so the cached `Polygons` are equivalent.
//!  * The constructor's `PrintObject`/`BuildVolume` slicing dependency
//!    (`TreeModelVolumes(const PrintObject&, ...)`, `TreeModelVolumes.cpp:54`)
//!    and `precalculate`'s `print_object.get_layer(...)->lslices`
//!    (`TreeModelVolumes.cpp:133`) are not threadable here, so layer outlines and
//!    the `TreeSupportMeshGroupSettings`/`TreeSupportSettings` are supplied to the
//!    Rust constructor pre-extracted (see [`TreeModelVolumes::with_layer_outlines`]).
//!  * The crate `clipper_utils::offset_polygons` does not expose a per-call miter
//!    limit (`1.2`) or arc tolerance, so `jtMiter,1.2` / `jtRound,m_min_resolution`
//!    map to [`OffsetJoinType::Miter`] / [`OffsetJoinType::Round`] respectively.
//!  * `BOOST_LOG_TRIVIAL`, `SLIC3R_TREESUPPORTS_PROGRESS`, and the `#if 0`
//!    debug-SVG blocks are omitted.

use crate::clipper_utils::{self, OffsetJoinType};
use crate::geometry::{to_polygons, ExPolygon, ExPolygons, Point, Polygon};
use crate::support::tree_support_common::{
    tree_supports_show_error, LayerIndex, TreeSupportMeshGroupSettings, TreeSupportSettings,
};
use crate::{scale, unscale, Coord, CoordF};
use std::collections::BTreeMap;
use std::sync::Mutex;

// libslic3r::EPSILON (libslic3r.h). Used by calculateAvoidance.
use crate::libslic3r::EPSILON;

// =============================================================================
// TreeModelVolumes.hpp constants (Slic3r::TreeSupport3D namespace)
// =============================================================================

// TreeModelVolumes.hpp:32  static constexpr const double SUPPORT_TREE_EXPONENTIAL_FACTOR = 1.5;
pub const SUPPORT_TREE_EXPONENTIAL_FACTOR: CoordF = 1.5;
// TreeModelVolumes.hpp:33  static constexpr const coord_t SUPPORT_TREE_EXPONENTIAL_THRESHOLD = scaled<coord_t>(1. * SUPPORT_TREE_EXPONENTIAL_FACTOR);
pub const SUPPORT_TREE_EXPONENTIAL_THRESHOLD: Coord = scaled_coord(1.0 * SUPPORT_TREE_EXPONENTIAL_FACTOR);
// TreeModelVolumes.hpp:34  static constexpr const coord_t SUPPORT_TREE_COLLISION_RESOLUTION = scaled<coord_t>(0.5);
pub const SUPPORT_TREE_COLLISION_RESOLUTION: Coord = scaled_coord(0.5);
// TreeModelVolumes.hpp:35  static constexpr const bool SUPPORT_TREE_AVOID_SUPPORT_BLOCKER = true;
pub const SUPPORT_TREE_AVOID_SUPPORT_BLOCKER: bool = true;

// const-fn equivalent of `scaled<coord_t>(v)` (= round(v * SCALING_FACTOR)).
const fn scaled_coord(v: CoordF) -> Coord {
    // SCALING_FACTOR = 100_000.0 (lib.rs:416). All call sites here use exact values.
    (v * crate::SCALING_FACTOR) as Coord
}

// =============================================================================
// AvoidanceType (TreeModelVolumes.hpp:72-78  enum class AvoidanceType : int8_t)
// =============================================================================

/// TreeModelVolumes.hpp:72  `enum class AvoidanceType : int8_t`
///
/// The discriminant order is load-bearing: `calculateAvoidance` computes
/// `AvoidanceType(iter_idx % int(AvoidanceType::Count))`, relying on
/// `Slow = 0, FastSafe = 1, Fast = 2, Count = 3`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(i8)]
pub enum AvoidanceType {
    // TreeModelVolumes.hpp:74
    Slow = 0,
    // TreeModelVolumes.hpp:75
    FastSafe = 1,
    // TreeModelVolumes.hpp:76
    Fast = 2,
    // TreeModelVolumes.hpp:77
    Count = 3,
}

impl AvoidanceType {
    /// `AvoidanceType(int)` reconstruction used by `calculateAvoidance`.
    fn from_i32(v: i32) -> AvoidanceType {
        match v {
            0 => AvoidanceType::Slow,
            1 => AvoidanceType::FastSafe,
            2 => AvoidanceType::Fast,
            _ => AvoidanceType::Count,
        }
    }
}

impl Default for AvoidanceType {
    fn default() -> Self {
        // Matches the prior crate API default; not present in C++.
        Self::Fast
    }
}

// =============================================================================
// Clipper helpers mirroring the ClipperUtils `Polygons`-on-`Polygons` overloads
// used throughout TreeModelVolumes.cpp. These keep the call sites byte-for-byte
// aligned with the C++ (`union_`, `offset`, `diff`, `intersection`).
//
// FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib. All of the
// union_/offset/diff/intersection helpers below route through `clipper_utils`,
// which uses the `geo` crate (geo-clipper, fixed scale 1000) rather than
// ClipperLib at `coord_t` integer precision. The geometry results are therefore
// approximate (and the per-call miter limit `1.2` / arc tolerance from the C++
// `offset(..., jtMiter, 1.2)` / `jtRound, m_min_resolution` calls are not
// expressible), but the algebra/control flow around them mirrors the C++ exactly.
// =============================================================================

#[inline]
fn to_expolys(polygons: &[Polygon]) -> Vec<ExPolygon> {
    polygons.iter().map(|p| ExPolygon::new(p.clone())).collect()
}

// ClipperUtils.hpp  `Polygons union_(const Polygons &subject)`
fn union_(subject: &[Polygon]) -> Vec<Polygon> {
    if subject.is_empty() {
        return Vec::new();
    }
    to_polygons(&clipper_utils::union_polygons_ex(subject))
}

// ClipperUtils.hpp  `Polygons union_(const Polygons &subject, const Polygons &subject2)`
fn union_2(subject: &[Polygon], subject2: &[Polygon]) -> Vec<Polygon> {
    let mut all: Vec<Polygon> = Vec::with_capacity(subject.len() + subject2.len());
    all.extend_from_slice(subject);
    all.extend_from_slice(subject2);
    union_(&all)
}

// ClipperUtils.hpp  `ExPolygons union_ex(const Polygons &subject)`
fn union_ex(subject: &[Polygon]) -> Vec<ExPolygon> {
    clipper_utils::union_polygons_ex(subject)
}

// ClipperUtils.hpp  `Polygons offset(const ExPolygons&, float delta, JoinType, miter)`
fn offset_ex(subject: &[ExPolygon], delta: CoordF, jt: OffsetJoinType) -> Vec<Polygon> {
    to_polygons(&clipper_utils::offset_expolygons(subject, delta, jt))
}

// ClipperUtils.hpp  `Polygons offset(const Polygons&, float delta, JoinType, miter)`
fn offset_polys(subject: &[Polygon], delta: CoordF, jt: OffsetJoinType) -> Vec<Polygon> {
    to_polygons(&clipper_utils::offset_polygons(subject, delta, jt))
}

// ClipperUtils.hpp  `Polygons diff(const Polygons &subject, const Polygons &clip)`
fn diff(subject: &[Polygon], clip: &[Polygon]) -> Vec<Polygon> {
    to_polygons(&clipper_utils::difference(&to_expolys(subject), &to_expolys(clip)))
}

// ClipperUtils.hpp  `Polygons intersection(const Polygons &subject, const Polygons &clip)`
fn intersection(subject: &[Polygon], clip: &[Polygon]) -> Vec<Polygon> {
    to_polygons(&clipper_utils::intersection(&to_expolys(subject), &to_expolys(clip)))
}

// Polygon.cpp  `Polygons polygons_simplify(const Polygons &source, double tolerance)`
fn polygons_simplify(source: &[Polygon], tolerance: Coord) -> Vec<Polygon> {
    crate::geometry::polygons_simplify(source, tolerance as CoordF)
}

// `append(dst, src)` — Slic3r helper, move-append polygons.
#[inline]
fn append(dst: &mut Vec<Polygon>, src: Vec<Polygon>) {
    dst.extend(src);
}

// =============================================================================
// TreeModelVolumes.hpp:173-193  class LayerPolygonCache
// Caching polygons for a contiguous range of layers.
// =============================================================================

#[derive(Debug, Default)]
struct LayerPolygonCache {
    // TreeModelVolumes.hpp:190
    m_polygons: Vec<Vec<Polygon>>,
    // TreeModelVolumes.hpp:191
    m_idx_begin: LayerIndex,
    // TreeModelVolumes.hpp:192
    m_idx_end: LayerIndex,
}

impl LayerPolygonCache {
    // TreeModelVolumes.hpp:175-179
    fn allocate(&mut self, aidx_begin: LayerIndex, aidx_end: LayerIndex) {
        self.m_idx_begin = aidx_begin;
        self.m_idx_end = aidx_end;
        let n = (aidx_end - aidx_begin).max(0) as usize;
        self.m_polygons = vec![Vec::new(); n];
    }

    // TreeModelVolumes.hpp:181
    fn begin(&self) -> LayerIndex {
        self.m_idx_begin
    }
    // TreeModelVolumes.hpp:182
    fn end(&self) -> LayerIndex {
        self.m_idx_end
    }
    // TreeModelVolumes.hpp:183
    fn size(&self) -> usize {
        self.m_polygons.len()
    }

    // TreeModelVolumes.hpp:185
    fn has(&self, idx: LayerIndex) -> bool {
        idx >= self.m_idx_begin && idx < self.m_idx_end
    }
    // TreeModelVolumes.hpp:186  Polygons& operator[](LayerIndex idx)
    fn get(&self, idx: LayerIndex) -> &Vec<Polygon> {
        debug_assert!(idx >= self.m_idx_begin && idx < self.m_idx_end);
        &self.m_polygons[(idx - self.m_idx_begin) as usize]
    }
    fn get_mut(&mut self, idx: LayerIndex) -> &mut Vec<Polygon> {
        debug_assert!(idx >= self.m_idx_begin && idx < self.m_idx_end);
        &mut self.m_polygons[(idx - self.m_idx_begin) as usize]
    }
    // TreeModelVolumes.hpp:187  std::vector<Polygons>& polygons_mutable()
    fn polygons_mutable(&mut self) -> &mut Vec<Vec<Polygon>> {
        &mut self.m_polygons
    }
}

// =============================================================================
// TreeModelVolumes.hpp:198  using RadiusLayerPair = std::pair<coord_t, LayerIndex>;
// =============================================================================

/// `std::pair<coord_t, LayerIndex>`: `.0` = radius, `.1` = layer index.
pub type RadiusLayerPair = (Coord, LayerIndex);

/// Compatibility key kept for the external crate API (re-exported from `lib.rs`).
/// Equivalent to [`RadiusLayerPair`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RadiusLayerKey {
    pub radius: Coord,
    pub layer_idx: usize,
}

impl RadiusLayerKey {
    pub fn new(radius: Coord, layer_idx: usize) -> Self {
        Self { radius, layer_idx }
    }
}

// =============================================================================
// TreeModelVolumes.hpp:199-306  class RadiusLayerPolygonCache
// Vector of layers; at each layer a map from radius to Polygons.
// =============================================================================

/// TreeModelVolumes.hpp:199  `class RadiusLayerPolygonCache`
#[derive(Debug, Default)]
pub struct RadiusLayerPolygonCache {
    // TreeModelVolumes.hpp:204  using Layers = std::vector<LayerData>;
    //                          using LayerData = std::map<coord_t, Polygons>;
    // TreeModelVolumes.hpp:304  Layers m_data;
    // TreeModelVolumes.hpp:305  mutable std::mutex m_mutex;
    m_data: Mutex<Vec<BTreeMap<Coord, Vec<Polygon>>>>,
}

impl RadiusLayerPolygonCache {
    // TreeModelVolumes.hpp:206  RadiusLayerPolygonCache() = default;
    pub fn new() -> Self {
        Self {
            m_data: Mutex::new(Vec::new()),
        }
    }

    // TreeModelVolumes.hpp:882-889  (TreeModelVolumes.cpp:882) allocate_layers
    fn allocate_layers(data: &mut Vec<BTreeMap<Coord, Vec<Polygon>>>, num_layers: usize) {
        if num_layers > data.len() {
            if num_layers > data.capacity() {
                data.reserve(crate::utils::next_highest_power_of_2(num_layers) - data.len());
            }
            data.resize_with(num_layers, BTreeMap::new);
        }
    }

    // TreeModelVolumes.hpp:298-301  get_allocate_layer_data
    fn get_allocate_layer_data(
        data: &mut Vec<BTreeMap<Coord, Vec<Polygon>>>,
        layer_idx: LayerIndex,
    ) -> &mut BTreeMap<Coord, Vec<Polygon>> {
        Self::allocate_layers(data, (layer_idx + 1) as usize);
        &mut data[layer_idx as usize]
    }

    // TreeModelVolumes.hpp:213-217
    // void insert(std::vector<std::pair<RadiusLayerPair, Polygons>> &&in)
    fn insert_pairs(&self, in_data: Vec<(RadiusLayerPair, Vec<Polygon>)>) {
        let mut data = self.m_data.lock().unwrap();
        for (key, polys) in in_data {
            Self::get_allocate_layer_data(&mut data, key.1)
                .entry(key.0)
                .or_insert(polys);
        }
    }

    // TreeModelVolumes.hpp:219-223  by layer
    // void insert(std::vector<std::pair<coord_t, Polygons>> &&in, coord_t radius)
    // Faithful mirror of the C++ overload; not currently called from Rust.
    #[allow(dead_code)]
    fn insert_by_layer(&self, in_data: Vec<(LayerIndex, Vec<Polygon>)>, radius: Coord) {
        let mut data = self.m_data.lock().unwrap();
        for (layer, polys) in in_data {
            Self::get_allocate_layer_data(&mut data, layer)
                .entry(radius)
                .or_insert(polys);
        }
    }

    // TreeModelVolumes.hpp:224-229
    // void insert(std::vector<Polygons> &&in, coord_t first_layer_idx, coord_t radius)
    fn insert_layers(&self, in_data: Vec<Vec<Polygon>>, first_layer_idx: LayerIndex, radius: Coord) {
        let mut data = self.m_data.lock().unwrap();
        let mut first = first_layer_idx;
        Self::allocate_layers(&mut data, (first_layer_idx as usize) + in_data.len());
        for d in in_data {
            data[first as usize].entry(radius).or_insert(d);
            first += 1;
        }
    }

    // TreeModelVolumes.hpp:230-236
    // void insert(LayerPolygonCache &&in, coord_t radius)
    fn insert_cache(&self, mut in_data: LayerPolygonCache, radius: Coord) {
        let mut data = self.m_data.lock().unwrap();
        let mut i = in_data.begin();
        Self::allocate_layers(&mut data, (i as usize) + in_data.size());
        for d in std::mem::take(in_data.polygons_mutable()) {
            data[i as usize].entry(radius).or_insert(d);
            i += 1;
        }
    }

    // TreeModelVolumes.hpp:242-250  getArea
    fn get_area(&self, key: RadiusLayerPair) -> Option<Vec<Polygon>> {
        let data = self.m_data.lock().unwrap();
        if key.1 < 0 || key.1 as usize >= data.len() {
            return None;
        }
        let layer = &data[key.1 as usize];
        layer.get(&key.0).cloned()
    }

    // TreeModelVolumes.hpp:252-266  get_lower_bound_area
    // Get a collision area at a given layer for a radius that is lower or equal to the key radius.
    fn get_lower_bound_area(&self, key: RadiusLayerPair) -> Option<(Coord, Vec<Polygon>)> {
        let data = self.m_data.lock().unwrap();
        if key.1 < 0 || key.1 as usize >= data.len() {
            return None;
        }
        let layer = &data[key.1 as usize];
        if layer.is_empty() {
            return None;
        }
        // std::map::lower_bound(key.first): first element with radius >= key.first.
        match layer.range(key.0..).next() {
            Some((&r, polys)) if r == key.0 => Some((r, polys.clone())),
            // it == end() or it->first != key.first => step back one (--it).
            _ => {
                // The C++ takes the largest element strictly less than key.first.
                match layer.range(..key.0).next_back() {
                    Some((&r, polys)) => Some((r, polys.clone())),
                    // it == begin() => return {}.
                    None => None,
                }
            }
        }
    }

    // TreeModelVolumes.hpp:274-282  getMaxCalculatedLayer
    fn get_max_calculated_layer(&self, radius: Coord) -> LayerIndex {
        let data = self.m_data.lock().unwrap();
        let mut layer_idx = data.len() as LayerIndex - 1;
        while layer_idx > 0 {
            if data[layer_idx as usize].contains_key(&radius) {
                break;
            }
            layer_idx -= 1;
        }
        // The placeable on model areas do not exist on layer 0, as there can not be
        // model below it. As such it may be possible that layer 1 is available, but
        // layer 0 does not exist.
        if layer_idx == 0 {
            -1
        } else {
            layer_idx
        }
    }

    // TreeModelVolumes.cpp:892-902  sorted() — for debugging, sorted by layer then radius.
    #[allow(dead_code)]
    fn sorted(&self) -> Vec<(RadiusLayerPair, Vec<Polygon>)> {
        let data = self.m_data.lock().unwrap();
        let mut out = Vec::new();
        for (layer_idx, layer) in data.iter().enumerate() {
            for (&radius, polys) in layer.iter() {
                out.push(((radius, layer_idx as LayerIndex), polys.clone()));
            }
        }
        out
    }

    // TreeModelVolumes.hpp:287  void clear()
    pub fn clear(&self) {
        let mut data = self.m_data.lock().unwrap();
        data.clear();
    }

    // TreeModelVolumes.hpp:288-295  void clear_all_but_radius0()
    #[allow(dead_code)]
    fn clear_all_but_radius0(&self) {
        let mut data = self.m_data.lock().unwrap();
        for layer in data.iter_mut() {
            // Keep only the first (smallest radius) entry.
            if let Some((&first, _)) = layer.iter().next() {
                layer.retain(|&r, _| r == first);
            }
        }
    }

    // ---- External crate-API compatibility shims (used by lib.rs / tests). ----

    /// Insert polygons for a radius/layer pair (legacy crate API).
    pub fn insert(&self, key: RadiusLayerKey, polygons: Vec<Polygon>) {
        self.insert_pairs(vec![((key.radius, key.layer_idx as LayerIndex), polygons)]);
    }

    /// Get polygons for a radius/layer pair (legacy crate API).
    pub fn get(&self, key: &RadiusLayerKey) -> Option<Vec<Polygon>> {
        self.get_area((key.radius, key.layer_idx as LayerIndex))
    }

    /// Whether a radius/layer pair is present (legacy crate API).
    pub fn contains(&self, key: &RadiusLayerKey) -> bool {
        self.get_area((key.radius, key.layer_idx as LayerIndex)).is_some()
    }

    /// Maximum layer index cached for a radius (legacy crate API).
    pub fn max_layer_for_radius(&self, radius: Coord) -> Option<usize> {
        let data = self.m_data.lock().unwrap();
        let mut best: Option<usize> = None;
        for (layer_idx, layer) in data.iter().enumerate() {
            if layer.contains_key(&radius) {
                best = Some(layer_idx);
            }
        }
        best
    }

    /// Lower-bound area for a radius at a layer (legacy crate API).
    pub fn get_lower_bound(&self, radius: Coord, layer_idx: usize) -> Option<(Coord, Vec<Polygon>)> {
        self.get_lower_bound_area((radius, layer_idx as LayerIndex))
    }
}

// =============================================================================
// TreeModelVolumes.cpp:39-52  calculateMachineBorderCollision
// =============================================================================

// FIXME Machine border is currently ignored.
fn calculate_machine_border_collision(_machine_border: Polygon) -> Vec<Polygon> {
    // Put a border of 1m around the print volume so that we don't collide.
    // #if 1
    //   FIXME just returning no border will let tree support legs collide with print bed boundary
    Vec::new()
    // #else  (offsetting by 1000mm easily overflows int32_t coordinate)
    //   Polygons out = offset(machine_border, scaled<float>(1000.), jtMiter, 1.2);
    //   machine_border.reverse();
    //   out.emplace_back(std::move(machine_border));
    //   return out;
    // #endif
}

// =============================================================================
// TreeModelVolumesConfig — the parameters extracted (in the Rust constructor) from
// `TreeSupportMeshGroupSettings` / `TreeSupportSettings` so that this module does
// not have to thread `PrintObject`. Kept as a separate type for the crate API.
// =============================================================================

/// Configuration extracted from the tree-support settings to drive
/// [`TreeModelVolumes`]. The C++ derives these inside the constructor and
/// `precalculate` from `TreeSupportSettings`/`TreeSupportMeshGroupSettings`.
#[derive(Debug, Clone)]
pub struct TreeModelVolumesConfig {
    /// `m_max_move` precursor (`max_move`, before the `-2` adjustment).
    pub max_move: Coord,
    /// `m_max_move_slow` precursor (`max_move_slow`, before the `-2` adjustment).
    pub max_move_slow: Coord,
    /// `settings.resolution` (becomes `m_min_resolution`).
    pub min_resolution: Coord,
    /// `config.xy_distance`.
    pub xy_distance: Coord,
    /// `config.xy_min_distance`.
    pub xy_min_distance: Coord,
    /// At least one mesh allows support to rest on the model.
    pub support_rests_on_model: bool,
    /// `config.getRadius(0)` (becomes `m_radius_0`).
    pub min_radius: Coord,
    /// `config.increase_radius_until_radius` (becomes `m_increase_until_radius`).
    pub increase_until_radius: Coord,
    /// Per-layer heights (mm), retained for compatibility.
    pub layer_heights: Vec<CoordF>,
    /// Per-layer Z heights (mm), retained for compatibility.
    pub z_heights: Vec<CoordF>,
}

impl Default for TreeModelVolumesConfig {
    fn default() -> Self {
        Self {
            max_move: scale(1.0),
            max_move_slow: scale(0.5),
            min_resolution: scale(0.025),
            xy_distance: scale(0.7),
            xy_min_distance: scale(0.4),
            support_rests_on_model: false,
            min_radius: scale(0.2),
            increase_until_radius: scale(3.0),
            layer_heights: vec![0.2],
            z_heights: vec![0.2],
        }
    }
}

impl TreeModelVolumesConfig {
    /// Create a config with per-layer height/Z arrays (legacy crate API).
    pub fn with_layers(layer_heights: Vec<CoordF>, z_heights: Vec<CoordF>) -> Self {
        Self {
            layer_heights,
            z_heights,
            ..Default::default()
        }
    }

    /// Z height for a layer index.
    pub fn layer_z(&self, layer_idx: usize) -> CoordF {
        if layer_idx < self.z_heights.len() {
            self.z_heights[layer_idx]
        } else if !self.z_heights.is_empty() {
            *self.z_heights.last().unwrap()
        } else {
            0.0
        }
    }

    /// Layer height for a layer index.
    pub fn layer_height(&self, layer_idx: usize) -> CoordF {
        if layer_idx < self.layer_heights.len() {
            self.layer_heights[layer_idx]
        } else if !self.layer_heights.is_empty() {
            *self.layer_heights.last().unwrap()
        } else {
            0.2
        }
    }
}

// =============================================================================
// TreeModelVolumes.hpp:37-550  class TreeModelVolumes
// =============================================================================

/// TreeModelVolumes.hpp:37  `class TreeModelVolumes`
#[derive(Debug)]
pub struct TreeModelVolumes {
    /// TreeModelVolumes.hpp:169  Polygon m_bed_area;
    pub m_bed_area: Polygon,

    // TreeModelVolumes.hpp:420  coord_t m_max_move;
    m_max_move: Coord,
    // TreeModelVolumes.hpp:425  coord_t m_max_move_slow;
    m_max_move_slow: Coord,
    // TreeModelVolumes.hpp:429  coord_t m_min_resolution;
    m_min_resolution: Coord,

    // TreeModelVolumes.hpp:431  bool m_precalculated = false;
    m_precalculated: bool,
    // TreeModelVolumes.hpp:435  size_t m_current_outline_idx;
    m_current_outline_idx: usize,
    // TreeModelVolumes.hpp:439  coord_t m_current_min_xy_dist;
    m_current_min_xy_dist: Coord,
    // TreeModelVolumes.hpp:443  coord_t m_current_min_xy_dist_delta;
    m_current_min_xy_dist_delta: Coord,
    // TreeModelVolumes.hpp:447  bool m_support_rests_on_model;
    m_support_rests_on_model: bool,
    // TreeModelVolumes.hpp:467  coord_t m_increase_until_radius;
    m_increase_until_radius: Coord,

    // TreeModelVolumes.hpp:473  Polygons m_machine_border;
    m_machine_border: Vec<Polygon>,
    // TreeModelVolumes.hpp:477  std::vector<std::pair<TreeSupportMeshGroupSettings, std::vector<Polygons>>> m_layer_outlines;
    m_layer_outlines: Vec<(TreeSupportMeshGroupSettings, Vec<Vec<Polygon>>)>,
    // TreeModelVolumes.hpp:481  std::vector<Polygons> m_anti_overhang;
    m_anti_overhang: Vec<Vec<Polygon>>,
    // TreeModelVolumes.hpp:485  std::vector<coord_t> m_ignorable_radii;
    m_ignorable_radii: Vec<Coord>,
    // TreeModelVolumes.hpp:490  coord_t m_radius_0;
    m_radius_0: Coord,
    // TreeModelVolumes.hpp:493  std::vector<double> m_raft_layers;
    // Populated in C++ from `config.raft_layers`; the `PrintObject` slicing path is
    // not threaded here, so it stays empty (see module-level divergence notes).
    #[allow(dead_code)]
    m_raft_layers: Vec<CoordF>,

    // TreeModelVolumes.hpp:499  RadiusLayerPolygonCache m_collision_cache;
    m_collision_cache: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:500  RadiusLayerPolygonCache m_collision_cache_holefree;
    m_collision_cache_holefree: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:501  RadiusLayerPolygonCache m_avoidance_cache;
    m_avoidance_cache: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:502  RadiusLayerPolygonCache m_avoidance_cache_slow;
    m_avoidance_cache_slow: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:503  RadiusLayerPolygonCache m_avoidance_cache_to_model;
    m_avoidance_cache_to_model: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:504  RadiusLayerPolygonCache m_avoidance_cache_to_model_slow;
    m_avoidance_cache_to_model_slow: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:505  RadiusLayerPolygonCache m_placeable_areas_cache;
    m_placeable_areas_cache: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:511  RadiusLayerPolygonCache m_avoidance_cache_holefree;
    m_avoidance_cache_holefree: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:512  RadiusLayerPolygonCache m_avoidance_cache_holefree_to_model;
    m_avoidance_cache_holefree_to_model: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:540  RadiusLayerPolygonCache m_wall_restrictions_cache;
    m_wall_restrictions_cache: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:545  RadiusLayerPolygonCache m_wall_restrictions_cache_min;
    m_wall_restrictions_cache_min: RadiusLayerPolygonCache,

    // Retained config (the extracted scalar parameters; see TreeModelVolumesConfig).
    config: TreeModelVolumesConfig,
}

impl TreeModelVolumes {
    /// TreeModelVolumes.hpp:40  TreeModelVolumes() = default;
    pub fn new(config: TreeModelVolumesConfig) -> Self {
        // -2 to avoid rounding errors (TreeModelVolumes.cpp:63).
        let m_max_move = (config.max_move - 2).max(0);
        let m_max_move_slow = (config.max_move_slow - 2).max(0);
        Self {
            m_bed_area: Polygon::new(),
            m_max_move,
            m_max_move_slow,
            m_min_resolution: config.min_resolution,
            m_precalculated: false,
            m_current_outline_idx: 0,
            m_current_min_xy_dist: config.xy_min_distance,
            // TreeModelVolumes.cpp:119
            m_current_min_xy_dist_delta: config.xy_distance - config.xy_min_distance,
            m_support_rests_on_model: config.support_rests_on_model,
            m_increase_until_radius: config.increase_until_radius,
            m_machine_border: calculate_machine_border_collision(Polygon::new()),
            m_layer_outlines: Vec::new(),
            m_anti_overhang: Vec::new(),
            m_ignorable_radii: Vec::new(),
            // TreeModelVolumes.cpp:122  m_radius_0 = config.getRadius(0);
            m_radius_0: config.min_radius,
            m_raft_layers: Vec::new(),
            m_collision_cache: RadiusLayerPolygonCache::new(),
            m_collision_cache_holefree: RadiusLayerPolygonCache::new(),
            m_avoidance_cache: RadiusLayerPolygonCache::new(),
            m_avoidance_cache_slow: RadiusLayerPolygonCache::new(),
            m_avoidance_cache_to_model: RadiusLayerPolygonCache::new(),
            m_avoidance_cache_to_model_slow: RadiusLayerPolygonCache::new(),
            m_placeable_areas_cache: RadiusLayerPolygonCache::new(),
            m_avoidance_cache_holefree: RadiusLayerPolygonCache::new(),
            m_avoidance_cache_holefree_to_model: RadiusLayerPolygonCache::new(),
            m_wall_restrictions_cache: RadiusLayerPolygonCache::new(),
            m_wall_restrictions_cache_min: RadiusLayerPolygonCache::new(),
            config,
        }
    }

    /// Rust-side equivalent of `TreeModelVolumes(const PrintObject&, ...)`
    /// (TreeModelVolumes.cpp:54-176). The per-layer model outlines are supplied
    /// pre-extracted (`print_object.get_layer(...)->lslices`), one `ExPolygons`
    /// per object layer; this populates `m_layer_outlines` for a single mesh
    /// group with `TreeSupportMeshGroupSettings::default()`.
    pub fn with_layer_outlines(config: TreeModelVolumesConfig, layer_outlines: Vec<ExPolygons>) -> Self {
        let mut v = Self::new(config);

        // TreeModelVolumes.cpp:124-137  (single mesh group path, the `#else` branch)
        let mesh_settings = TreeSupportMeshGroupSettings::default();
        v.m_current_outline_idx = 0;
        // num_raft_layers = m_raft_layers.size() (= 0 here).
        let outlines: Vec<Vec<Polygon>> = layer_outlines
            .iter()
            .map(|ex| {
                // outlines[layer_idx] = polygons_simplify(to_polygons(lslices), mesh_settings.resolution);
                // outlines[layer_idx].insert(end(), machine_borders.begin(), machine_borders.end());
                let mut o = polygons_simplify(&to_polygons(ex), mesh_settings.resolution);
                // machine_borders is empty here (calculateMachineBorderCollision returns {}).
                o.extend(v.m_machine_border.iter().cloned());
                o
            })
            .collect();
        v.m_layer_outlines.push((mesh_settings, outlines));

        // TreeModelVolumes.cpp:140-145
        v.m_support_rests_on_model = false;
        v.m_min_resolution = Coord::MAX;
        for (settings, _) in &v.m_layer_outlines {
            v.m_support_rests_on_model |= !settings.support_material_buildplate_only;
            v.m_min_resolution = v.m_min_resolution.min(settings.resolution);
        }
        v
    }

    /// Set the bed area (`m_bed_area`).
    pub fn set_bed_area(&mut self, bed_area: Polygon) {
        self.m_bed_area = bed_area;
    }

    /// Set anti-overhang areas (`m_anti_overhang`).
    pub fn set_anti_overhang(&mut self, anti_overhang: Vec<Vec<Polygon>>) {
        self.m_anti_overhang = anti_overhang;
    }

    /// Retained extracted configuration.
    pub fn config(&self) -> &TreeModelVolumesConfig {
        &self.config
    }

    /// Number of layers in the (single) outline group.
    pub fn layer_count(&self) -> usize {
        self.m_layer_outlines
            .first()
            .map(|(_, o)| o.len())
            .unwrap_or(0)
    }

    // TreeModelVolumes.hpp:54-57  void clear()
    pub fn clear(&mut self) {
        self.clear_all_but_object_collision();
        self.m_collision_cache.clear();
    }

    // TreeModelVolumes.hpp:58-70  void clear_all_but_object_collision()
    pub fn clear_all_but_object_collision(&mut self) {
        // m_collision_cache.clear_all_but_radius0();
        self.m_collision_cache_holefree.clear();
        self.m_avoidance_cache.clear();
        self.m_avoidance_cache_slow.clear();
        self.m_avoidance_cache_to_model.clear();
        self.m_avoidance_cache_to_model_slow.clear();
        self.m_placeable_areas_cache.clear();
        self.m_avoidance_cache_holefree.clear();
        self.m_avoidance_cache_holefree_to_model.clear();
        self.m_wall_restrictions_cache.clear();
        self.m_wall_restrictions_cache_min.clear();
    }

    // TreeModelVolumes.hpp:514-535  avoidance_cache(type, to_model)
    fn avoidance_cache(&self, type_: AvoidanceType, to_model: bool) -> &RadiusLayerPolygonCache {
        if to_model {
            match type_ {
                AvoidanceType::Fast => &self.m_avoidance_cache_to_model,
                AvoidanceType::Slow => &self.m_avoidance_cache_to_model_slow,
                AvoidanceType::Count => unreachable!("avoidance_cache(Count)"),
                AvoidanceType::FastSafe => &self.m_avoidance_cache_holefree_to_model,
            }
        } else {
            match type_ {
                AvoidanceType::Fast => &self.m_avoidance_cache,
                AvoidanceType::Slow => &self.m_avoidance_cache_slow,
                AvoidanceType::Count => unreachable!("avoidance_cache(Count)"),
                AvoidanceType::FastSafe => &self.m_avoidance_cache_holefree,
            }
        }
    }

    // =========================================================================
    // TreeModelVolumes.hpp:148-154  ceilRadius(radius, min_xy_dist)
    // =========================================================================
    /// TreeModelVolumes.hpp:148  `coord_t ceilRadius(coord_t radius, bool min_xy_dist) const`
    pub fn ceil_radius_min_xy(&self, radius: Coord, min_xy_dist: bool) -> Coord {
        debug_assert!(radius >= 0);
        if min_xy_dist {
            self.ceil_radius(radius)
        } else if radius > 0 {
            // special case as if a radius 0 is requested it could be to ensure correct xy distance.
            self.ceil_radius(radius + self.m_current_min_xy_dist_delta)
        } else {
            self.m_current_min_xy_dist_delta
        }
    }

    // TreeModelVolumes.hpp:162-167  getRadiusNextCeil(radius, min_xy_dist)
    /// TreeModelVolumes.hpp:162  `coord_t getRadiusNextCeil(coord_t radius, bool min_xy_dist) const`
    pub fn get_radius_next_ceil(&self, radius: Coord, min_xy_dist: bool) -> Coord {
        debug_assert!(radius > 0);
        if min_xy_dist {
            self.ceil_radius(radius)
        } else {
            self.ceil_radius(radius + self.m_current_min_xy_dist_delta) - self.m_current_min_xy_dist_delta
        }
    }

    // =========================================================================
    // TreeModelVolumes.cpp:178-311  precalculate
    // =========================================================================
    /// TreeModelVolumes.cpp:178  `void precalculate(const PrintObject&, coord_t max_layer, throw_on_cancel)`
    ///
    /// `TreeSupportSettings` is supplied directly (extracted from the print
    /// object by the caller) instead of being recomputed from `PrintObject`.
    pub fn precalculate(&mut self, config: &TreeSupportSettings, max_layer: LayerIndex) {
        self.m_precalculated = true;

        // Get the config corresponding to one mesh that is in the current group.
        // (Passed in by the caller.)

        // TreeModelVolumes.cpp:188-203
        {
            // calculate which radius each layer in the tip may have.
            let mut possible_tip_radiis: Vec<Coord> = Vec::new();
            for distance_to_top in 0..=config.tip_layers {
                possible_tip_radiis.push(self.ceil_radius(config.get_radius(distance_to_top, 0.0)));
                possible_tip_radiis.push(self.ceil_radius(
                    config.get_radius(distance_to_top, 0.0) + self.m_current_min_xy_dist_delta,
                ));
            }
            // sort_remove_duplicates(possible_tip_radiis);
            possible_tip_radiis.sort_unstable();
            possible_tip_radiis.dedup();
            // It theoretically may happen in the tip, that the radius can change so much in-between 2 layers,
            // that a ceil step is skipped. As such a radius will not reasonably happen in the tree and it
            // will most likely not be requested, so just skip these.
            let mut radius_eval = self.m_radius_0;
            while radius_eval <= config.branch_radius {
                if possible_tip_radiis.binary_search(&radius_eval).is_err() {
                    self.m_ignorable_radii.push(radius_eval);
                }
                radius_eval = self.ceil_radius(radius_eval + 1);
            }
        }

        // TreeModelVolumes.cpp:210-225
        // it may seem that the required avoidance can be of a smaller radius when going to model, but as for
        // every branch going towards the bp, the to model avoidance is required to check for possible merges
        // with to model branches, this assumption is in-fact wrong.
        let mut radius_until_layer: BTreeMap<Coord, LayerIndex> = BTreeMap::new();
        for distance_to_top in 0..=max_layer {
            let current_layer = max_layer - distance_to_top;
            let update_radius_until_layer = |r: Coord, map: &mut BTreeMap<Coord, LayerIndex>| {
                // emplace only if not present (keeps the first/highest current_layer seen).
                map.entry(r).or_insert(current_layer);
            };
            // regular radius
            update_radius_until_layer(
                self.ceil_radius(config.get_radius(distance_to_top as usize, 0.0) + self.m_current_min_xy_dist_delta),
                &mut radius_until_layer,
            );
            // the maximum radius that the radius with the min_xy_dist can achieve
            update_radius_until_layer(
                self.ceil_radius(config.get_radius(distance_to_top as usize, 0.0)),
                &mut radius_until_layer,
            );
            update_radius_until_layer(
                self.ceil_radius(config.recommended_min_radius(current_layer) + self.m_current_min_xy_dist_delta),
                &mut radius_until_layer,
            );
        }

        // TreeModelVolumes.cpp:231  Copy to deque to use in parallel for later.
        let relevant_avoidance_radiis: Vec<RadiusLayerPair> =
            radius_until_layer.iter().map(|(&r, &l)| (r, l)).collect();

        // TreeModelVolumes.cpp:233-239  Append additional radiis needed for collision.
        radius_until_layer.insert(
            self.ceil_radius_min_xy(self.m_increase_until_radius + self.m_current_min_xy_dist_delta, true),
            max_layer,
        );
        radius_until_layer.insert(0, max_layer);
        if self.m_current_min_xy_dist_delta != 0 {
            radius_until_layer.insert(self.m_current_min_xy_dist_delta, max_layer);
        }

        // TreeModelVolumes.cpp:242
        let relevant_collision_radiis: Vec<RadiusLayerPair> =
            radius_until_layer.iter().map(|(&r, &l)| (r, l)).collect();

        // TreeModelVolumes.cpp:245  Calculate the relevant collisions
        self.calculate_collision_keys(&relevant_collision_radiis);

        // TreeModelVolumes.cpp:248-251  collisions with all holes removed (called safe)
        let mut relevant_hole_collision_radiis: Vec<RadiusLayerPair> = Vec::new();
        for &key in &relevant_avoidance_radiis {
            if key.0 < self.m_increase_until_radius + self.m_current_min_xy_dist_delta {
                relevant_hole_collision_radiis.push(key);
            }
        }

        // TreeModelVolumes.cpp:254  Calculate collisions without holes, built from regular collision
        self.calculate_collision_holefree_keys(&relevant_hole_collision_radiis);
        // TreeModelVolumes.cpp:256-257
        if self.m_support_rests_on_model {
            self.calculate_placeables_keys(&relevant_avoidance_radiis);
        }

        // TreeModelVolumes.cpp:262-267  task_group: avoidance + wall restrictions (serial here).
        self.calculate_avoidance_keys(&relevant_avoidance_radiis, true, self.m_support_rests_on_model);
        self.calculate_wall_restrictions_keys(&relevant_avoidance_radiis);
    }

    // =========================================================================
    // TreeModelVolumes.cpp:313-324  getCollision
    // =========================================================================
    /// TreeModelVolumes.cpp:313  `const Polygons& getCollision(coord_t orig_radius, LayerIndex, bool min_xy_dist) const`
    pub fn get_collision_min_xy(&self, orig_radius: Coord, layer_idx: LayerIndex, min_xy_dist: bool) -> Vec<Polygon> {
        let radius = self.ceil_radius_min_xy(orig_radius, min_xy_dist);
        if let Some(result) = self.m_collision_cache.get_area((radius, layer_idx)) {
            return result;
        }
        if self.m_precalculated {
            // BOOST_LOG_TRIVIAL(error_level_not_in_cache) << "Had to calculate collision ...";
            tree_supports_show_error("Not precalculated Collision requested.", false);
        }
        self.calculate_collision(radius, layer_idx);
        self.get_collision_min_xy(orig_radius, layer_idx, min_xy_dist)
    }

    // TreeModelVolumes.cpp:329-332  get_collision_lower_bound_area
    /// TreeModelVolumes.cpp:329  `get_collision_lower_bound_area(LayerIndex, coord_t max_radius)`
    pub fn get_collision_lower_bound_area(&self, layer_id: LayerIndex, max_radius: Coord) -> Option<(Coord, Vec<Polygon>)> {
        self.m_collision_cache.get_lower_bound_area((max_radius, layer_id))
    }

    // TreeModelVolumes.cpp:335-347  getCollisionHolefree (private)
    /// TreeModelVolumes.cpp:335  `const Polygons& getCollisionHolefree(coord_t radius, LayerIndex) const`
    fn get_collision_holefree(&self, radius: Coord, layer_idx: LayerIndex) -> Vec<Polygon> {
        debug_assert!(radius == self.ceil_radius(radius));
        debug_assert!(radius < self.m_increase_until_radius + self.m_current_min_xy_dist_delta);
        if let Some(result) = self.m_collision_cache_holefree.get_area((radius, layer_idx)) {
            return result;
        }
        if self.m_precalculated {
            tree_supports_show_error("Not precalculated Holefree Collision requested.", false);
        }
        self.calculate_collision_holefree_keys(&[(radius, layer_idx)]);
        self.get_collision_holefree(radius, layer_idx)
    }

    // TreeModelVolumes.cpp:349-376  getAvoidance
    /// TreeModelVolumes.cpp:349  `const Polygons& getAvoidance(coord_t orig_radius, LayerIndex, AvoidanceType, bool to_model, bool min_xy_dist) const`
    pub fn get_avoidance_full(
        &self,
        orig_radius: Coord,
        layer_idx: LayerIndex,
        mut type_: AvoidanceType,
        to_model: bool,
        min_xy_dist: bool,
    ) -> Vec<Polygon> {
        if layer_idx == 0 {
            // What on the layer directly above buildplate do i have to avoid to reach the buildplate ...
            return self.get_collision_min_xy(orig_radius, layer_idx, min_xy_dist);
        }

        let radius = self.ceil_radius_min_xy(orig_radius, min_xy_dist);
        if type_ == AvoidanceType::FastSafe
            && radius >= self.m_increase_until_radius + self.m_current_min_xy_dist_delta
        {
            // no holes anymore by definition at this request
            type_ = AvoidanceType::Fast;
        }

        if let Some(result) = self.avoidance_cache(type_, to_model).get_area((radius, layer_idx)) {
            return result;
        }

        if self.m_precalculated {
            if to_model {
                tree_supports_show_error("Not precalculated Avoidance(to model) requested.", false);
            } else {
                tree_supports_show_error("Not precalculated Avoidance(to buildplate) requested.", false);
            }
        }
        self.calculate_avoidance_keys(&[(radius, layer_idx)], !to_model, to_model);
        // Retrive failed and correct result was calculated. Now it has to be retrived.
        self.get_avoidance_full(orig_radius, layer_idx, type_, to_model, min_xy_dist)
    }

    // TreeModelVolumes.cpp:378-393  getPlaceableAreas
    /// TreeModelVolumes.cpp:378  `const Polygons& getPlaceableAreas(coord_t orig_radius, LayerIndex, throw_on_cancel) const`
    pub fn get_placeable_areas(&self, orig_radius: Coord, layer_idx: LayerIndex) -> Vec<Polygon> {
        let radius = self.ceil_radius(orig_radius);
        if let Some(result) = self.m_placeable_areas_cache.get_area((radius, layer_idx)) {
            return result;
        }
        if self.m_precalculated {
            tree_supports_show_error("Not precalculated Placeable areas requested.", false);
        }
        if orig_radius == 0 {
            // Placable areas for radius 0 are calculated in the general collision code.
            return self.get_collision_min_xy(0, layer_idx, true);
        }
        self.calculate_placeables(radius, layer_idx);
        self.get_placeable_areas(orig_radius, layer_idx)
    }

    // TreeModelVolumes.cpp:395-420  getWallRestriction
    /// TreeModelVolumes.cpp:395  `const Polygons& getWallRestriction(coord_t orig_radius, LayerIndex, bool min_xy_dist) const`
    pub fn get_wall_restriction(&self, orig_radius: Coord, layer_idx: LayerIndex, mut min_xy_dist: bool) -> Vec<Polygon> {
        debug_assert!(layer_idx > 0);
        if layer_idx == 0 {
            // Should never be requested as there will be no going below layer 0 ...,
            // but just to be sure some semi-sane catch.
            return self.get_collision_min_xy(orig_radius, layer_idx, min_xy_dist);
        }

        min_xy_dist &= self.m_current_min_xy_dist_delta > 0;

        let radius = self.ceil_radius(orig_radius);
        let cache = if min_xy_dist {
            &self.m_wall_restrictions_cache_min
        } else {
            &self.m_wall_restrictions_cache
        };
        if let Some(result) = cache.get_area((radius, layer_idx)) {
            return result;
        }
        if self.m_precalculated {
            tree_supports_show_error(
                if min_xy_dist {
                    "Not precalculated Wall restriction of minimum xy distance requested )."
                } else {
                    "Not precalculated Wall restriction requested )."
                },
                false,
            );
        }
        self.calculate_wall_restrictions_keys(&[(radius, layer_idx)]);
        // Retrieve failed and correct result was calculated. Now it has to be retrieved.
        self.get_wall_restriction(orig_radius, layer_idx, min_xy_dist)
    }

    // =========================================================================
    // TreeModelVolumes.cpp:422-433  calculateCollision (keys -> per-key)
    // =========================================================================
    /// TreeModelVolumes.cpp:422  `void calculateCollision(const std::vector<RadiusLayerPair> &keys, throw_on_cancel)`
    fn calculate_collision_keys(&self, keys: &[RadiusLayerPair]) {
        // tbb::parallel_for over keys (serial here). Recursive call to calculateCollision.
        for key in keys {
            let radius = key.0;
            let max_layer_idx = key.1;
            self.calculate_collision(radius, max_layer_idx);
        }
    }

    // =========================================================================
    // TreeModelVolumes.cpp:435-598  calculateCollision (radius, max_layer_idx)
    // =========================================================================
    /// TreeModelVolumes.cpp:435  `void calculateCollision(coord_t radius, LayerIndex max_layer_idx, throw_on_cancel)`
    fn calculate_collision(&self, radius: Coord, max_layer_idx: LayerIndex) {
        // Process the outlines from least layers to most layers so that the final union
        // will run over the longest vector.
        let mut layer_outline_indices: Vec<usize> = (0..self.m_layer_outlines.len()).collect();
        layer_outline_indices.sort_by(|&i, &j| {
            self.m_layer_outlines[i].1.len().cmp(&self.m_layer_outlines[j].1.len())
        });

        // TreeModelVolumes.cpp:445-446
        let mut data = LayerPolygonCache::default();
        data.allocate(
            self.m_collision_cache.get_max_calculated_layer(radius) + 1,
            max_layer_idx + 1,
        );

        // TreeModelVolumes.cpp:448-451
        let calculate_placable = self.m_support_rests_on_model && radius == 0;
        let mut data_placeable = LayerPolygonCache::default();
        if calculate_placable {
            data_placeable.allocate(data.begin(), data.end());
        }

        // TreeModelVolumes.cpp:453
        for &outline_idx in &layer_outline_indices {
            let outlines = &self.m_layer_outlines[outline_idx].1;
            if outlines.is_empty() {
                continue;
            }
            // TreeModelVolumes.cpp:455-466
            let settings = &self.m_layer_outlines[outline_idx].0;
            let layer_height = settings.layer_height;
            let z_distance_bottom_layers =
                ((settings.support_bottom_distance as CoordF / layer_height as CoordF).round()) as i32;
            let z_distance_top_layers =
                ((settings.support_top_distance as CoordF / layer_height as CoordF).round()) as i32;
            let xy_distance = if outline_idx == self.m_current_outline_idx {
                self.m_current_min_xy_dist
            } else {
                // technically this causes collision for the normal xy_distance to be larger by
                // m_current_min_xy_dist_delta for all not currently processing meshes as this delta
                // will be added at request time.
                // FIXME support_xy_distance is not corrected for "soluble" flag.
                settings.support_xy_distance
            };

            // TreeModelVolumes.cpp:468-487  1) Calculate offsets of collision areas.
            let mut collision_areas_offsetted = LayerPolygonCache::default();
            collision_areas_offsetted.allocate(
                (data.begin() - z_distance_bottom_layers).max(0),
                (data.end() + z_distance_top_layers).min(outlines.len() as LayerIndex),
            );
            let offset_value = radius + xy_distance;
            {
                let begin = collision_areas_offsetted.begin();
                let end = collision_areas_offsetted.end();
                for layer_idx in begin..end {
                    let mut collision_areas = self.m_machine_border.clone();
                    append(&mut collision_areas, outlines[layer_idx as usize].clone());
                    // jtRound is not needed here, as the overshoot can not cause errors in the
                    // algorithm, because no assumptions are made about the model.
                    let offsetted = if offset_value == 0 {
                        union_(&collision_areas)
                    } else {
                        offset_ex(&union_ex(&collision_areas), offset_value as CoordF, OffsetJoinType::Miter)
                    };
                    *collision_areas_offsetted.get_mut(layer_idx) = offsetted;
                }
            }

            // TreeModelVolumes.cpp:489-551  2) Sum over top / bottom ranges.
            let processing_last_mesh = outline_idx == layer_outline_indices.len();
            {
                let begin = data.begin();
                let end = data.end();
                for layer_idx in begin..end {
                    let mut collisions: Vec<Polygon> = Vec::new();
                    // bottom layers (i in [-z_distance_bottom_layers, 0])
                    let mut i = -z_distance_bottom_layers;
                    while i <= 0 {
                        let j = layer_idx + i;
                        if collision_areas_offsetted.has(j) {
                            append(&mut collisions, collision_areas_offsetted.get(j).clone());
                        }
                        i += 1;
                    }
                    // top layers (i in [1, z_distance_top_layers])
                    let mut i = 1;
                    while i <= z_distance_top_layers {
                        let j = layer_idx + i;
                        if j < outlines.len() as LayerIndex {
                            let mut collision_areas_original = self.m_machine_border.clone();
                            append(&mut collision_areas_original, outlines[j as usize].clone());
                            // technically the calculation below is off by one layer ... (see C++ comment block).
                            let required_range_x: Coord = (xy_distance as CoordF
                                - ((i as CoordF
                                    - (if z_distance_top_layers == 1 { 0.5 } else { 0.0 }))
                                    * xy_distance as CoordF
                                    / z_distance_top_layers as CoordF))
                                as Coord;
                            // the conditional -0.5 ensures that plastic can never touch on the diagonal
                            // downward when z_distance_top_layers = 1.
                            append(
                                &mut collisions,
                                offset_ex(
                                    &union_ex(&collision_areas_original),
                                    (radius + required_range_x) as CoordF,
                                    OffsetJoinType::Miter,
                                ),
                            );
                        }
                        i += 1;
                    }
                    // TreeModelVolumes.cpp:538-540
                    collisions = if processing_last_mesh && layer_idx < self.m_anti_overhang.len() as LayerIndex {
                        union_2(
                            &collisions,
                            &offset_ex(
                                &union_ex(&self.m_anti_overhang[layer_idx as usize]),
                                radius as CoordF,
                                OffsetJoinType::Miter,
                            ),
                        )
                    } else {
                        union_(&collisions)
                    };
                    // TreeModelVolumes.cpp:541-547
                    let dst = data.get_mut(layer_idx);
                    if processing_last_mesh {
                        if !dst.is_empty() {
                            collisions = union_2(&collisions, dst);
                        }
                        *dst = polygons_simplify(&collisions, self.m_min_resolution);
                    } else {
                        append(dst, collisions);
                    }
                }
            }

            // TreeModelVolumes.cpp:553-582  3) Optionally calculate placables.
            if calculate_placable {
                let begin = (z_distance_bottom_layers + 1).max(data.begin());
                let end = data.end();
                for layer_idx in begin..end {
                    let layer_idx_below = layer_idx - z_distance_bottom_layers - 1;
                    debug_assert!(layer_idx_below >= 0);
                    let current = collision_areas_offsetted.get(layer_idx).clone();
                    let below = outlines[layer_idx_below as usize].clone();
                    // Inflate the surface to sit on by the separation distance to increase chance of a
                    // support being placed on a sloped surface.
                    let clip = if layer_idx_below < self.m_anti_overhang.len() as LayerIndex {
                        union_2(&current, &self.m_anti_overhang[layer_idx_below as usize])
                    } else {
                        current
                    };
                    let mut placable = diff(&offset_polys(&below, xy_distance as CoordF, OffsetJoinType::Miter), &clip);
                    let dst = data_placeable.get_mut(layer_idx);
                    if processing_last_mesh {
                        if !dst.is_empty() {
                            placable = union_2(&placable, dst);
                        }
                        *dst = polygons_simplify(&placable, self.m_min_resolution);
                    } else {
                        append(dst, placable);
                    }
                }
            } else {
                // Calculating just the collision areas.
            }
        }

        // TreeModelVolumes.cpp:595-597
        self.m_collision_cache.insert_cache(data, radius);
        if calculate_placable {
            self.m_placeable_areas_cache.insert_cache(data_placeable, radius);
        }
    }

    // =========================================================================
    // TreeModelVolumes.cpp:600-630  calculateCollisionHolefree
    // =========================================================================
    /// TreeModelVolumes.cpp:600  `void calculateCollisionHolefree(const std::vector<RadiusLayerPair> &keys, throw_on_cancel)`
    fn calculate_collision_holefree_keys(&self, keys: &[RadiusLayerPair]) {
        let mut max_layer: LayerIndex = 0;
        for key in keys {
            max_layer = max_layer.max(key.1);
        }

        // tbb::parallel_for over [0, max_layer+1) (serial here).
        let mut data: Vec<(RadiusLayerPair, Vec<Polygon>)> = Vec::new();
        for layer_idx in 0..(max_layer + 1) {
            for &key in keys {
                if layer_idx <= key.1 {
                    // Logically increase the collision by m_increase_until_radius
                    let radius = key.0;
                    debug_assert!(radius == self.ceil_radius(radius));
                    debug_assert!(radius < self.m_increase_until_radius + self.m_current_min_xy_dist_delta);
                    let increase_radius_ceil = self.ceil_radius_min_xy(self.m_increase_until_radius, false) - radius;
                    debug_assert!(increase_radius_ceil > 0);
                    // this union is important as otherwise holes (in form of lines that will increase to
                    // holes in a later step) can get unioned onto the area.
                    data.push((
                        (radius, layer_idx),
                        polygons_simplify(
                            &offset_ex(
                                &union_ex(&self.get_collision_min_xy(self.m_increase_until_radius, layer_idx, false)),
                                (5 - increase_radius_ceil) as CoordF,
                                OffsetJoinType::Round,
                            ),
                            self.m_min_resolution,
                        ),
                    ));
                }
            }
        }
        self.m_collision_cache_holefree.insert_pairs(data);
    }

    // =========================================================================
    // TreeModelVolumes.cpp:632-732  calculateAvoidance
    // =========================================================================
    /// TreeModelVolumes.cpp:632  `void calculateAvoidance(const std::vector<RadiusLayerPair> &keys, bool to_build_plate, bool to_model, throw_on_cancel)`
    fn calculate_avoidance_keys(&self, keys: &[RadiusLayerPair], to_build_plate: bool, to_model: bool) {
        // TreeModelVolumes.cpp:636-645  struct AvoidanceTask
        struct AvoidanceTask {
            type_: AvoidanceType,
            radius: Coord,
            max_required_layer: LayerIndex,
            to_model: bool,
            start_layer: LayerIndex,
        }
        impl AvoidanceTask {
            fn slow(&self) -> bool {
                self.type_ == AvoidanceType::Slow
            }
            fn holefree(&self) -> bool {
                self.type_ == AvoidanceType::FastSafe
            }
        }

        let mut avoidance_tasks: Vec<AvoidanceTask> = Vec::new();

        // TreeModelVolumes.cpp:650-666
        // for (iter_idx = 0; iter_idx < 2 * keys.size() * Count; ++iter_idx)
        let count = AvoidanceType::Count as i32; // 3
        for iter_idx in 0..(2 * keys.len() as i32 * count) {
            let type_ = AvoidanceType::from_i32(iter_idx % count);
            let radius = keys[(iter_idx / 6) as usize].0;
            let max_required_layer = keys[(iter_idx / 6) as usize].1;
            let task_to_model = ((iter_idx / 3) & 1) != 0;
            // Ensure start_layer is at least 1 (getMaxCalculatedLayer returns -1 if nothing calculated).
            let start_layer = (1 + self.avoidance_cache(type_, task_to_model).get_max_calculated_layer(radius)).max(1);
            let task = AvoidanceTask {
                type_,
                radius,
                max_required_layer,
                to_model: task_to_model,
                start_layer,
            };
            if task.start_layer > task.max_required_layer {
                // BOOST_LOG_TRIVIAL(debug) << "Calculation requested for value already calculated?";
                continue;
            }
            // TreeModelVolumes.cpp:663-665  emplace only when the task is wanted.
            if (if task.to_model { to_model } else { to_build_plate })
                && (!task.holefree()
                    || task.radius < self.m_increase_until_radius + self.m_current_min_xy_dist_delta)
            {
                avoidance_tasks.push(task);
            }
        }

        // tbb::parallel_for over tasks (serial here).
        for task in &avoidance_tasks {
            debug_assert!(!task.holefree() || task.radius < self.m_increase_until_radius + self.m_current_min_xy_dist_delta);
            if task.to_model {
                // ensuring Placeableareas are calculated
                self.get_placeable_areas(task.radius, task.max_required_layer);
            }
            // The following loop propagating avoidance regions bottom up is inherently serial.
            let collision_holefree = (task.slow() || task.holefree())
                && task.radius < self.m_increase_until_radius + self.m_current_min_xy_dist_delta;
            let max_move: CoordF = if task.slow() {
                self.m_max_move_slow as CoordF
            } else {
                self.m_max_move as CoordF
            };
            // Limiting the offset step so that unioning the shrunk latest_avoidance with the current layer
            // collisions will not create gaps in the resulting avoidance region.
            let mut move_step: CoordF = 1.9 * task.radius.max(self.m_current_min_xy_dist) as CoordF;
            // TreeModelVolumes.cpp:686  `if (move_step < EPSILON) return;`
            // In C++ this `return` exits the per-task tbb lambda (grainsize 1), i.e. it
            // skips only the current task. In this serial loop the equivalent is `continue`,
            // NOT `return` (which would wrongly abort all remaining tasks).
            if move_step < EPSILON {
                continue;
            }
            // TreeModelVolumes.cpp:687  `int move_steps = round_up_divide<int>(max_move, move_step);`
            // The explicit `<int>` makes the template's `DataType` = int, so both float
            // operands are truncated toward zero to `int` BEFORE the `(a+b-1)/b` integer
            // divide. `as i64` here truncates toward zero identically; the divide then
            // matches Utils.hpp round_up_divide. FIDELITY-NOTE(F2): C++ uses int32 here;
            // these scaled move distances stay within int32 so the i64 widening is inert.
            let mut move_steps: i32 = crate::utils::round_up_divide(max_move as i64, move_step as i64) as i32;
            debug_assert!(move_steps > 0);
            let mut last_move_step: CoordF = max_move - (move_steps - 1) as CoordF * move_step;
            if last_move_step < scale(0.05) as CoordF {
                // assert(move_steps > 1);
                if move_steps > 1 {
                    // Avoid taking a very short last step, stretch the other steps a bit instead.
                    move_steps -= 1;
                    move_step = max_move / move_steps as CoordF;
                    last_move_step = move_step;
                }
            }
            // minDist as the delta was already added, also avoidance for layer 0 will return the collision.
            let mut latest_avoidance =
                self.get_avoidance_full(task.radius, task.start_layer - 1, task.type_, task.to_model, true);
            let mut data: Vec<(RadiusLayerPair, Vec<Polygon>)> = Vec::new();
            for layer_idx in task.start_layer..=task.max_required_layer {
                // Merge current layer collisions with shrunk last_avoidance.
                let current_layer_collisions = if collision_holefree {
                    self.get_collision_holefree(task.radius, layer_idx)
                } else {
                    self.get_collision_min_xy(task.radius, layer_idx, true)
                };
                // For mildly steep branch angles only one step will be taken.
                for istep in 0..move_steps {
                    let delta = if istep + 1 == move_steps { -last_move_step } else { -move_step };
                    latest_avoidance = union_2(
                        &current_layer_collisions,
                        &offset_polys(&latest_avoidance, delta, OffsetJoinType::Round),
                    );
                }
                if task.to_model {
                    latest_avoidance = diff(&latest_avoidance, &self.get_placeable_areas(task.radius, layer_idx));
                }
                latest_avoidance = polygons_simplify(&latest_avoidance, self.m_min_resolution);
                data.push(((task.radius, layer_idx), latest_avoidance.clone()));
            }
            self.avoidance_cache(task.type_, task.to_model).insert_pairs(data);
        }
    }

    // =========================================================================
    // TreeModelVolumes.cpp:735-742  calculatePlaceables (keys)
    // =========================================================================
    /// TreeModelVolumes.cpp:735  `void calculatePlaceables(const std::vector<RadiusLayerPair> &keys, throw_on_cancel)`
    fn calculate_placeables_keys(&self, keys: &[RadiusLayerPair]) {
        for key in keys {
            self.calculate_placeables(key.0, key.1);
        }
    }

    // =========================================================================
    // TreeModelVolumes.cpp:744-781  calculatePlaceables (radius, max_required_layer)
    // =========================================================================
    /// TreeModelVolumes.cpp:744  `void calculatePlaceables(coord_t radius, LayerIndex max_required_layer, throw_on_cancel)`
    fn calculate_placeables(&self, radius: Coord, max_required_layer: LayerIndex) {
        let start_layer = 1 + self.m_placeable_areas_cache.get_max_calculated_layer(radius);
        if start_layer > max_required_layer {
            // BOOST_LOG_TRIVIAL(debug) << "Requested calculation for value already calculated ?";
            return;
        }

        let mut data: Vec<Vec<Polygon>> = vec![Vec::new(); (max_required_layer + 1 - start_layer) as usize];

        if start_layer == 0 {
            data[0] = diff(&self.m_machine_border, &self.get_collision_min_xy(radius, 0, true));
        }

        // tbb::parallel_for over [max(1, start_layer), max_required_layer+1) (serial here).
        let begin = 1.max(start_layer);
        for layer_idx in begin..(max_required_layer + 1) {
            // As a placeable area is calculated by (collision below) - (collision current) and the collision
            // is offset by xy_distance, it can happen that a small line is considered a flat area; making the
            // area smaller by xy_distance fixes this.
            data[(layer_idx - start_layer) as usize] = offset_ex(
                &union_ex(&self.get_placeable_areas(0, layer_idx)),
                -((radius + self.m_current_min_xy_dist + self.m_current_min_xy_dist_delta) as CoordF),
                OffsetJoinType::Miter,
            );
        }
        self.m_placeable_areas_cache.insert_layers(data, start_layer, radius);
    }

    // =========================================================================
    // TreeModelVolumes.cpp:783-851  calculateWallRestrictions
    // =========================================================================
    /// TreeModelVolumes.cpp:783  `void calculateWallRestrictions(const std::vector<RadiusLayerPair> &keys, throw_on_cancel)`
    fn calculate_wall_restrictions_keys(&self, keys: &[RadiusLayerPair]) {
        // Wall restrictions are mainly important when they represent actual walls that are printed, and not
        // "just" the configured z_distance. (See the C++ ASCII diagrams.)
        for key in keys {
            let radius = key.0;
            let max_required_layer = key.1;
            let min_layer_bottom = 1.max(self.m_wall_restrictions_cache.get_max_calculated_layer(radius));
            let buffer_size = (max_required_layer + 1 - min_layer_bottom) as usize;
            let mut data: Vec<Vec<Polygon>> = vec![Vec::new(); buffer_size];
            let mut data_min: Vec<Vec<Polygon>> = Vec::new();
            if self.m_current_min_xy_dist_delta > 0 {
                data_min = vec![Vec::new(); buffer_size];
            }
            // inner tbb::parallel_for over [min_layer_bottom, max_required_layer+1) (serial here).
            for layer_idx in min_layer_bottom..(max_required_layer + 1) {
                // radius contains m_current_min_xy_dist_delta already if required
                data[(layer_idx - min_layer_bottom) as usize] = polygons_simplify(
                    &intersection(
                        &self.get_collision_min_xy(0, layer_idx, false),
                        &self.get_collision_min_xy(radius, layer_idx - 1, true),
                    ),
                    self.m_min_resolution,
                );
                if !data_min.is_empty() {
                    data_min[(layer_idx - min_layer_bottom) as usize] = polygons_simplify(
                        &intersection(
                            &self.get_collision_min_xy(0, layer_idx, true),
                            &self.get_collision_min_xy(radius, layer_idx - 1, true),
                        ),
                        self.m_min_resolution,
                    );
                }
            }
            self.m_wall_restrictions_cache.insert_layers(data, min_layer_bottom, radius);
            if !data_min.is_empty() {
                self.m_wall_restrictions_cache_min.insert_layers(data_min, min_layer_bottom, radius);
            }
        }
    }

    // =========================================================================
    // TreeModelVolumes.cpp:853-880  ceilRadius (single argument)
    // =========================================================================
    /// TreeModelVolumes.cpp:853  `coord_t ceilRadius(coord_t radius) const`
    pub fn ceil_radius(&self, radius: Coord) -> Coord {
        if radius == 0 {
            return 0;
        }

        let mut out = self.m_radius_0;
        if radius > self.m_radius_0 {
            // generate SUPPORT_TREE_PRE_EXPONENTIAL_STEPS of radiis before starting to exponentially increase.
            let initial_radius_delta = SUPPORT_TREE_EXPONENTIAL_THRESHOLD - self.m_radius_0;
            let ignore = |r: Coord| self.m_ignorable_radii.binary_search(&r).is_ok();
            if initial_radius_delta > SUPPORT_TREE_COLLISION_RESOLUTION {
                let num_steps =
                    crate::utils::round_up_divide(initial_radius_delta, SUPPORT_TREE_EXPONENTIAL_THRESHOLD) as i32;
                let stepsize = initial_radius_delta / num_steps as Coord;
                out += stepsize;
                for _step in 0..num_steps {
                    if out >= radius && !ignore(out) {
                        return out;
                    }
                    out += stepsize;
                }
            } else {
                out += SUPPORT_TREE_COLLISION_RESOLUTION;
            }
            while out < radius || ignore(out) {
                debug_assert!(
                    (out as CoordF * SUPPORT_TREE_EXPONENTIAL_FACTOR)
                        > (out + SUPPORT_TREE_COLLISION_RESOLUTION) as CoordF
                );
                // FIDELITY-NOTE(F2): C++ `out` is `coord_t` (int32); `out * 1.5`
                // truncates back to int32 each iteration and would wrap on overflow.
                // Crate-wide `Coord = i64` widens this; for in-tree radii (< branch_radius)
                // the values stay within int32 so the truncation result is identical.
                out = (out as CoordF * SUPPORT_TREE_EXPONENTIAL_FACTOR) as Coord;
            }
        }
        out
    }

    // =========================================================================
    // External crate-API shims (used by support/mod.rs, tree_support_3d.rs, lib.rs).
    // These delegate to the faithful C++ methods above so the build stays green.
    // =========================================================================

    /// `getCollision(radius, layer_idx, min_xy_dist=true)` (2-arg legacy form).
    pub fn get_collision(&self, radius: Coord, layer_idx: usize) -> Vec<Polygon> {
        self.get_collision_min_xy(radius, layer_idx as LayerIndex, true)
    }

    /// `getCollisionHolefree(radius, layer_idx)` (legacy form).
    pub fn get_collision_holefree_pub(&self, radius: Coord, layer_idx: usize) -> Vec<Polygon> {
        let r = self.ceil_radius(radius);
        self.get_collision_holefree(r, layer_idx as LayerIndex)
    }

    /// `getAvoidance(radius, layer_idx, type, to_model, min_xy_dist=true)` (legacy form).
    pub fn get_avoidance(
        &self,
        radius: Coord,
        layer_idx: usize,
        avoidance_type: AvoidanceType,
        to_model: bool,
    ) -> Vec<Polygon> {
        self.get_avoidance_full(radius, layer_idx as LayerIndex, avoidance_type, to_model, true)
    }

    /// `getPlaceableAreas(radius, layer_idx)` (legacy form).
    pub fn get_placeable(&self, radius: Coord, layer_idx: usize) -> Vec<Polygon> {
        self.get_placeable_areas(radius, layer_idx as LayerIndex)
    }

    /// `getWallRestriction(radius, layer_idx, use_min_distance)` (legacy form).
    pub fn get_wall_restriction_legacy(&self, radius: Coord, layer_idx: usize, use_min_distance: bool) -> Vec<Polygon> {
        self.get_wall_restriction(radius, layer_idx as LayerIndex, use_min_distance)
    }

    /// `getRadiusNextCeil` legacy 1-arg form (min_xy_dist defaulted true).
    pub fn next_ceil_radius(&self, radius: Coord) -> Coord {
        if radius == 0 {
            // C++ asserts radius > 0; preserve a sane value for the legacy callers.
            return self.ceil_radius(0);
        }
        self.get_radius_next_ceil(radius, true)
    }

    /// Whether `precalculate` has been called.
    pub fn is_precalculated(&self) -> bool {
        self.m_precalculated
    }
}

// =============================================================================
// Free helpers retained for the crate API (used by tree_support_3d.rs / lib.rs).
// Not part of the C++ TreeModelVolumes, but depended upon by ported siblings.
// =============================================================================

/// Whether `point` lies inside any of `polygons`.
pub fn point_inside_polygons(point: Point, polygons: &[Polygon]) -> bool {
    for polygon in polygons {
        if polygon.contains_point(&point) {
            return true;
        }
    }
    false
}

/// Whether `point` is outside all collision areas for `radius` at `layer_idx`.
pub fn is_safe_position(volumes: &TreeModelVolumes, point: Point, radius: Coord, layer_idx: usize) -> bool {
    let collision = volumes.get_collision(radius, layer_idx);
    !point_inside_polygons(point, &collision)
}

/// Find the nearest safe position from `point` (expanding-circle search).
pub fn find_nearest_safe_position(
    volumes: &TreeModelVolumes,
    point: Point,
    radius: Coord,
    layer_idx: usize,
    max_search_distance: Coord,
) -> Option<Point> {
    if is_safe_position(volumes, point, radius, layer_idx) {
        return Some(point);
    }

    let collision = volumes.get_collision(radius, layer_idx);

    let search_step = scale(0.1);
    let mut search_radius = search_step;

    while search_radius <= max_search_distance {
        let num_points = (2.0 * std::f64::consts::PI * unscale(search_radius) / 0.1).ceil() as usize;
        let num_points = num_points.max(8);

        for i in 0..num_points {
            let angle = 2.0 * std::f64::consts::PI * (i as f64) / (num_points as f64);
            let dx = (search_radius as f64 * angle.cos()) as Coord;
            let dy = (search_radius as f64 * angle.sin()) as Coord;
            let test_point = Point::new(point.x + dx, point.y + dy);
            if !point_inside_polygons(test_point, &collision) {
                return Some(test_point);
            }
        }
        search_radius += search_step;
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::ExPolygon;

    fn make_square_mm(size: f64) -> Polygon {
        let half = scale(size / 2.0);
        Polygon::from_points(vec![
            Point::new(-half, -half),
            Point::new(half, -half),
            Point::new(half, half),
            Point::new(-half, half),
        ])
    }

    #[test]
    fn test_tree_model_volumes_new() {
        let config = TreeModelVolumesConfig::default();
        let volumes = TreeModelVolumes::new(config);
        assert_eq!(volumes.layer_count(), 0);
        assert!(!volumes.is_precalculated());
    }

    #[test]
    fn test_ceil_radius_zero() {
        let config = TreeModelVolumesConfig::default();
        let volumes = TreeModelVolumes::new(config);
        assert_eq!(volumes.ceil_radius(0), 0);
    }

    #[test]
    fn test_ceil_radius_below_radius0() {
        // For radius <= m_radius_0, ceil_radius returns m_radius_0.
        let config = TreeModelVolumesConfig::default();
        let r0 = config.min_radius;
        let volumes = TreeModelVolumes::new(config);
        assert_eq!(volumes.ceil_radius(1), r0);
        assert_eq!(volumes.ceil_radius(r0), r0);
    }

    #[test]
    fn test_avoidance_type_discriminants() {
        assert_eq!(AvoidanceType::Slow as i32, 0);
        assert_eq!(AvoidanceType::FastSafe as i32, 1);
        assert_eq!(AvoidanceType::Fast as i32, 2);
        assert_eq!(AvoidanceType::Count as i32, 3);
    }

    #[test]
    fn test_collision_cache() {
        let cache = RadiusLayerPolygonCache::new();
        let key = RadiusLayerKey::new(scale(1.0), 5);
        assert!(!cache.contains(&key));
        let polygons = vec![make_square_mm(10.0)];
        cache.insert(key, polygons.clone());
        assert!(cache.contains(&key));
        let retrieved = cache.get(&key).unwrap();
        assert_eq!(retrieved.len(), 1);
    }

    #[test]
    fn test_collision_calculation_empty() {
        let config = TreeModelVolumesConfig::default();
        let volumes = TreeModelVolumes::new(config);
        let collision = volumes.get_collision(scale(1.0), 0);
        assert!(collision.is_empty());
    }

    #[test]
    fn test_cache_lower_bound() {
        let cache = RadiusLayerPolygonCache::new();
        cache.insert(RadiusLayerKey::new(100, 0), vec![make_square_mm(1.0)]);
        cache.insert(RadiusLayerKey::new(200, 0), vec![make_square_mm(2.0)]);
        cache.insert(RadiusLayerKey::new(400, 0), vec![make_square_mm(4.0)]);

        // Request radius 300, should get 200.
        let (radius, _) = cache.get_lower_bound(300, 0).unwrap();
        assert_eq!(radius, 200);
        // Request radius 100, should get 100 (exact match).
        let (radius, _) = cache.get_lower_bound(100, 0).unwrap();
        assert_eq!(radius, 100);
        // Request radius 50, should get nothing.
        assert!(cache.get_lower_bound(50, 0).is_none());
    }

    #[test]
    fn test_point_inside_polygons() {
        let square = make_square_mm(10.0);
        let polygons = vec![square];
        assert!(point_inside_polygons(Point::new(0, 0), &polygons));
        assert!(!point_inside_polygons(
            Point::new(scale(100.0), scale(100.0)),
            &polygons
        ));
    }

    #[test]
    fn test_is_safe_position() {
        let config = TreeModelVolumesConfig::default();
        let square = make_square_mm(10.0);
        let expolygon = ExPolygon::new(square);
        let volumes = TreeModelVolumes::with_layer_outlines(config, vec![vec![expolygon]]);
        // Point inside model should not be safe.
        assert!(!is_safe_position(&volumes, Point::new(0, 0), 0, 0));
        // Point far from model should be safe.
        assert!(is_safe_position(
            &volumes,
            Point::new(scale(100.0), scale(100.0)),
            0,
            0
        ));
    }

    #[test]
    fn test_with_layer_outlines_min_resolution() {
        // m_min_resolution should pick up the mesh-group resolution (0.025 mm).
        let config = TreeModelVolumesConfig::default();
        let expolygon = ExPolygon::new(make_square_mm(10.0));
        let volumes = TreeModelVolumes::with_layer_outlines(config, vec![vec![expolygon]]);
        assert_eq!(volumes.m_min_resolution, scale(0.025));
    }
}
