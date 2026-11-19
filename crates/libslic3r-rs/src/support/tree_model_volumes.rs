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
//! How this file differs from the reference (each noted where it happens):
//!  * TBB `parallel_for` / `task_group` bodies run one after the other. Every one of them fills the
//!    cache entries of its own key from entries looked up by key, so the caches come out the same.
//!  * The getters return a shared pointer to the cache entry (`PolygonsRef`) where the C++ returns a
//!    reference into the cache.
//!  * `TreeModelVolumes::from_print_object` is the constructor of the pin, over the classes of
//!    `crate::print::faithful`. `TreeModelVolumes::new(config)` and `with_layer_outlines` are the
//!    constructors of the earlier interface of the crate (support/mod.rs), which take the outlines
//!    and a few scalars instead of a PrintObject.
//!  * The Clipper operations run over the vendored ClipperLib in scaled units (gates
//!    `TREE_MODEL_VOLUMES_CLIPPER` and `TREE_MODEL_VOLUMES_OFFSET`, each on unless `=0`).
//!  * `BOOST_LOG_TRIVIAL`, `SLIC3R_TREESUPPORTS_PROGRESS`, and the `#if 0` blocks are left out.

use crate::build_volume::BuildVolume;
use crate::clipper_utils::{self, OffsetJoinType};
use crate::geometry::{to_polygons, ExPolygon, ExPolygons, Point, Polygon};
use crate::print::faithful as pf;
use crate::support::tree_support_common::{
    tree_supports_show_error, LayerIndex, TreeSupportMeshGroupSettings, TreeSupportSettings,
};
use crate::{scale, unscale, Coord, CoordF};
use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};

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

// `scaled<coord_t>(v)` (Point.hpp): `coord_t(v / SCALING_FACTOR)`, a division by 0.00001 and a truncation
// (0.5 comes out as 49999, 1.5 as 150000).
const fn scaled_coord(v: CoordF) -> Coord {
    (v / crate::libslic3r::SCALING_FACTOR) as Coord
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
    pub fn from_i32(v: i32) -> AvoidanceType {
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
// The ClipperUtils entry points TreeModelVolumes.cpp calls (`union_`, `union_ex`, `offset`, `diff`,
// `intersection`) and `polygons_simplify`.
//
// Gate TREE_MODEL_VOLUMES_CLIPPER (on unless =0): the operations run over the vendored ClipperLib in
// scaled units, each as the one Clipper operation ClipperUtils.cpp runs for it. =0 keeps the earlier
// arm, which ran them through the `geo` crate on a grid of a micrometre.
// =============================================================================

#[inline]
fn tmv_clipper() -> bool {
    crate::faithful_gate("TREE_MODEL_VOLUMES_CLIPPER")
}

#[inline]
fn to_expolys(polygons: &[Polygon]) -> Vec<ExPolygon> {
    polygons.iter().map(|p| ExPolygon::new(p.clone())).collect()
}

// ClipperUtils.cpp:721-722  `Polygons union_(const Polygons &subject)`:
// `_clipper(ctUnion, PolygonsProvider(subject), EmptyPathsProvider(), ApplySafetyOffset::No)`, one
// union with the non-zero rule, the paths as Clipper puts them out.
fn union_(subject: &[Polygon]) -> Vec<Polygon> {
    if tmv_clipper() {
        return clipper_utils::union_flat_clib(subject);
    }
    if subject.is_empty() {
        return Vec::new();
    }
    to_polygons(&clipper_utils::union_polygons_ex(subject))
}

// ClipperUtils.cpp:727-734  `Polygons union_(const Polygons &subject, const Polygons &subject2)`
fn union_2(subject: &[Polygon], subject2: &[Polygon]) -> Vec<Polygon> {
    let mut polys: Vec<Polygon> = Vec::with_capacity(subject.len() + subject2.len());
    polys.extend_from_slice(subject);
    polys.extend_from_slice(subject2);
    union_(&polys)
}

// ClipperUtils.cpp:813-814  `ExPolygons union_ex(const Polygons &subject, ClipperLib::PolyFillType fill_type = pftNonZero)`
fn union_ex(subject: &[Polygon]) -> Vec<ExPolygon> {
    if tmv_clipper() {
        // ClipperLib::pftNonZero
        return clipper_utils::union_ex_clib(subject, 1);
    }
    clipper_utils::union_polygons_ex(subject)
}

// Every delta this file offsets by is in scaled units (`coord_t`), as in the
// C++; `clipper_utils::offset_expolygons` / `offset_polygons` take millimetres,
// so the earlier arm grew every area by 1e5 times the distance (a 0.4 mm
// collision margin became 40 m and the offset came back empty). Gate
// TREE_MODEL_VOLUMES_OFFSET (on unless =0) offsets in scaled units with the
// call's own miter limit / arc tolerance, through the `float` delta of the C++
// signatures; =0 keeps the earlier arm.
#[inline]
fn tmv_join_code(jt: OffsetJoinType) -> i32 {
    match jt {
        OffsetJoinType::Miter => 0,
        OffsetJoinType::Round => 1,
        OffsetJoinType::Square => 2,
    }
}

// ClipperUtils.cpp:561-562  `Polygons offset(const ExPolygons&, const float delta, JoinType, double miterLimit)`
//   { return to_polygons(expolygons_offset(expolygons, delta, joinType, miterLimit)); }
// `delta` is the `float` of the C++ signature.
fn offset_ex(subject: &[ExPolygon], delta: f32, jt: OffsetJoinType, miter_limit: f64) -> Vec<Polygon> {
    if !crate::faithful_gate("TREE_MODEL_VOLUMES_OFFSET") {
        return to_polygons(&clipper_utils::offset_expolygons(subject, delta as CoordF, jt));
    }
    let rings: Vec<(crate::clipper::Path, crate::clipper::Paths)> = subject
        .iter()
        .map(|e| {
            let ring = |pts: &[Point]| -> crate::clipper::Path {
                pts.iter().map(|q| crate::clipper::IntPoint::new(q.x as i32, q.y as i32)).collect()
            };
            (ring(e.contour.points()), e.holes.iter().map(|h| ring(h.points())).collect())
        })
        .collect();
    clipper_utils::clib_polygons_from_paths(&clipper_utils::expolygons_offset(
        &rings,
        delta as f64,
        clipper_utils::clib_join_type(tmv_join_code(jt)),
        miter_limit,
    ))
}

// ClipperUtils.cpp:413-414  `Polygons offset(const Polygons&, const float delta, JoinType, double miterLimit)`
//   { return to_polygons(offset_paths<ClipperLib::Paths>(PolygonsProvider(polygons), delta, joinType, miterLimit)); }
fn offset_polys(subject: &[Polygon], delta: f32, jt: OffsetJoinType, miter_limit: f64) -> Vec<Polygon> {
    if !crate::faithful_gate("TREE_MODEL_VOLUMES_OFFSET") {
        return to_polygons(&clipper_utils::offset_polygons(subject, delta as CoordF, jt));
    }
    clipper_utils::offset_polygons_clib_ml(subject, delta as f64, jt, miter_limit)
}

// ClipperUtils.cpp:410-411  `Polygons offset(const Polygon &polygon, const float delta, JoinType joinType =
// DefaultJoinType, double miterLimit = DefaultMiterLimit)`
//   { return to_polygons(raw_offset(SinglePathProvider(polygon.points), delta, joinType, miterLimit)); }
// The raw offset of the one path, no union after it.
fn offset_polygon_raw(polygon: &Polygon, delta: f32) -> Vec<Polygon> {
    let path: crate::clipper::Path =
        polygon.points().iter().map(|q| crate::clipper::IntPoint::new(q.x as i32, q.y as i32)).collect();
    clipper_utils::clib_polygons_from_paths(&clipper_utils::raw_offset(
        &vec![path],
        delta as f64,
        crate::clipper::JoinType::jtMiter,
        3.0,
        crate::clipper::EndType::etClosedPolygon,
    ))
}

// ClipperUtils.cpp:677-678  `Polygons diff(const Polygons &subject, const Polygons &clip)`
fn diff(subject: &[Polygon], clip: &[Polygon]) -> Vec<Polygon> {
    if tmv_clipper() {
        return clipper_utils::diff_polygons_raw_clib(subject, clip);
    }
    to_polygons(&clipper_utils::difference(&to_expolys(subject), &to_expolys(clip)))
}

// ClipperUtils.cpp:701-702  `Polygons intersection(const Polygons &subject, const Polygons &clip)`
fn intersection(subject: &[Polygon], clip: &[Polygon]) -> Vec<Polygon> {
    if tmv_clipper() {
        return clipper_utils::intersection_polygons_raw_clib(subject, clip);
    }
    to_polygons(&clipper_utils::intersection(&to_expolys(subject), &to_expolys(clip)))
}

// Polygon.cpp:658-677  `Polygons polygons_simplify(const Polygons &source, double tolerance)`
//
// Gate TREE_MODEL_VOLUMES_CLIPPER (on unless =0): the decimated contour goes through
// `ClipperLib::SimplifyPolygons(SinglePathProvider(simplified), pftNonZero)` (clipper.hpp:558-566: a union
// of the one path, strictly simple, the paths as Clipper puts them out), so a contour that the
// decimation leaves with fewer than three points yields nothing. =0 keeps the earlier arm
// (`crate::geometry::polygons_simplify`, which hands such a contour back as it is).
fn polygons_simplify(source_polygons: &[Polygon], tolerance: Coord) -> Vec<Polygon> {
    if !tmv_clipper() {
        return crate::geometry::polygons_simplify(source_polygons, tolerance as CoordF);
    }
    let mut out: Vec<Polygon> = Vec::with_capacity(source_polygons.len());
    for source_polygon in source_polygons {
        // Run Douglas / Peucker simplification algorithm on an open polyline (by repeating the first point at the end of the polyline),
        // `to_polyline(source_polygon)` (Polygon.hpp): the points and the first of them once more.
        let mut polyline: Vec<Point> = Vec::with_capacity(source_polygon.points.len() + 1);
        polyline.extend_from_slice(&source_polygon.points);
        if let Some(front) = source_polygon.points.first() {
            polyline.push(*front);
        }
        let mut simplified = crate::multi_point::douglas_peucker(&polyline, tolerance as f64);
        // then remove the last (repeated) point.
        simplified.pop();
        // Simplify the decimated contour by ClipperLib.
        let path: crate::clipper::Path =
            simplified.iter().map(|q| crate::clipper::IntPoint::new(q.x as i32, q.y as i32)).collect();
        let ccw = crate::clipper::area(&path) > 0.;
        for mut path in clipper_utils::clib_polygons_from_paths(&crate::clipper::simplify_polygons(
            &vec![path],
            crate::clipper::PolyFillType::pftNonZero,
        )) {
            if !ccw {
                // ClipperLib likely reoriented negative area contours to become positive. Reverse holes back to CW.
                path.points.reverse();
            }
            out.push(path);
        }
    }
    out
}

// `append(dst, src)` — Slic3r helper, move-append polygons.
#[inline]
fn append(dst: &mut Vec<Polygon>, src: Vec<Polygon>) {
    dst.extend(src);
}

// Polygon.cpp:171-186  `Point Polygon::centroid() const`. The result is `Point(Vec2d(...))`, which rounds
// with lrint (Point.hpp:181); `cross2` is one expression, `v1.x() * v2.y() - v2.x() * v1.y()`.
fn polygon_centroid(poly: &Polygon) -> Point {
    let points = poly.points();
    let mut area_sum = 0.0_f64;
    let mut c = (0.0_f64, 0.0_f64);
    if points.len() >= 3 {
        let back = points[points.len() - 1];
        let mut p1 = (back.x as f64, back.y as f64);
        for p in points {
            let p2 = (p.x as f64, p.y as f64);
            let a = p1.0.mul_add(p2.1, -(p2.0 * p1.1));
            area_sum += a;
            c.0 += (p1.0 + p2.0) * a;
            c.1 += (p1.1 + p2.1) * a;
            p1 = p2;
        }
    }
    Point::new(
        (c.0 / (3.0 * area_sum)).round_ties_even() as Coord,
        (c.1 / (3.0 * area_sum)).round_ties_even() as Coord,
    )
}

// BoundingBox.hpp `Polygon BoundingBox::polygon() const`: min, (max.x, min.y), max, (min.x, max.y).
fn bounding_box_polygon(min: Point, max: Point) -> Polygon {
    Polygon::from_points(vec![min, Point::new(max.x, min.y), max, Point::new(min.x, max.y)])
}

// ExPolygon.cpp:382-413  `ExPolygons ExPolygon::split_expoly_with_holes(coord_t gap_width, const ExPolygons&
// collision) const` with an empty `collision`, as the constructor calls it: no hole overlaps the collision,
// so the hole of the largest area is the one the expolygon is split around.
fn split_expoly_with_holes_no_collision(expoly: &ExPolygon, gap_width: Coord) -> ExPolygons {
    let mut sub_overhangs: ExPolygons = Vec::new();
    let mut max_hole = Polygon::new();
    let mut max_area: CoordF = 0.;
    for hole in &expoly.holes {
        // !is_collided && !overlaps({ExPolygon(hole)}, collision) && abs(hole.area()) > max_area
        if hole.area().abs() > max_area {
            max_area = hole.area().abs();
            max_hole = hole.clone();
        }
    }
    if !max_hole.points().is_empty() {
        let overhang_bbx = crate::geometry::get_extents_expoly(expoly);
        let cent = polygon_centroid(&max_hole);
        let this = std::slice::from_ref(expoly);
        let mut part = |min: Point, max: Point| {
            // intersection_ex(ExPolygon(BoundingBox(min, max).polygon()), *this)
            sub_overhangs
                .extend(clipper_utils::intersection_clib(&[ExPolygon::new(bounding_box_polygon(min, max))], this));
        };
        part(overhang_bbx.min, Point::new(cent.x - gap_width, cent.y - gap_width));
        part(Point::new(cent.x + gap_width, cent.y + gap_width), overhang_bbx.max);
        part(Point::new(overhang_bbx.min.x, cent.y + gap_width), Point::new(cent.x - gap_width, overhang_bbx.max.y));
        part(Point::new(cent.x + gap_width, overhang_bbx.min.y), Point::new(overhang_bbx.max.x, cent.y - gap_width));
    }
    sub_overhangs
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

/// The `const Polygons&` the getters of the C++ return: a reference into a cache whose entries are
/// never replaced (`emplace` keeps the first), here a shared pointer to the entry.
pub type PolygonsRef = Arc<Vec<Polygon>>;

/// TreeModelVolumes.hpp:199  `class RadiusLayerPolygonCache`
#[derive(Debug, Default)]
pub struct RadiusLayerPolygonCache {
    // TreeModelVolumes.hpp:204  using Layers = std::vector<LayerData>;
    //                          using LayerData = std::map<coord_t, Polygons>;
    // TreeModelVolumes.hpp:304  Layers m_data;
    // TreeModelVolumes.hpp:305  mutable std::mutex m_mutex;
    m_data: Mutex<Vec<BTreeMap<Coord, PolygonsRef>>>,
}

impl RadiusLayerPolygonCache {
    // TreeModelVolumes.hpp:206  RadiusLayerPolygonCache() = default;
    pub fn new() -> Self {
        Self { m_data: Mutex::new(Vec::new()) }
    }

    // TreeModelVolumes.cpp:891-898  void RadiusLayerPolygonCache::allocate_layers(size_t num_layers)
    fn allocate_layers(data: &mut Vec<BTreeMap<Coord, PolygonsRef>>, num_layers: usize) {
        if num_layers > data.len() {
            if num_layers > data.capacity() {
                data.reserve(crate::utils::next_highest_power_of_2(num_layers) - data.len());
            }
            data.resize_with(num_layers, BTreeMap::new);
        }
    }

    // TreeModelVolumes.hpp:298-301  get_allocate_layer_data
    fn get_allocate_layer_data(
        data: &mut Vec<BTreeMap<Coord, PolygonsRef>>,
        layer_idx: LayerIndex,
    ) -> &mut BTreeMap<Coord, PolygonsRef> {
        Self::allocate_layers(data, (layer_idx + 1) as usize);
        &mut data[layer_idx as usize]
    }

    // TreeModelVolumes.hpp:213-217
    // void insert(std::vector<std::pair<RadiusLayerPair, Polygons>> &&in)
    fn insert_pairs(&self, in_data: Vec<(RadiusLayerPair, Vec<Polygon>)>) {
        let mut data = self.m_data.lock().unwrap();
        for (key, polys) in in_data {
            Self::get_allocate_layer_data(&mut data, key.1).entry(key.0).or_insert_with(|| Arc::new(polys));
        }
    }

    // TreeModelVolumes.hpp:219-223  by layer
    // void insert(std::vector<std::pair<coord_t, Polygons>> &&in, coord_t radius)
    // Nothing of the C++ calls this overload.
    #[allow(dead_code)]
    fn insert_by_layer(&self, in_data: Vec<(LayerIndex, Vec<Polygon>)>, radius: Coord) {
        let mut data = self.m_data.lock().unwrap();
        for (layer, polys) in in_data {
            Self::get_allocate_layer_data(&mut data, layer).entry(radius).or_insert_with(|| Arc::new(polys));
        }
    }

    // TreeModelVolumes.hpp:224-229
    // void insert(std::vector<Polygons> &&in, coord_t first_layer_idx, coord_t radius)
    fn insert_layers(&self, in_data: Vec<Vec<Polygon>>, first_layer_idx: LayerIndex, radius: Coord) {
        let mut data = self.m_data.lock().unwrap();
        let mut first = first_layer_idx;
        Self::allocate_layers(&mut data, (first_layer_idx as usize) + in_data.len());
        for d in in_data {
            data[first as usize].entry(radius).or_insert_with(|| Arc::new(d));
            first += 1;
        }
    }

    // TreeModelVolumes.hpp:230-236
    // void insert(LayerPolygonCache &&in, coord_t radius)
    fn insert_cache(&self, mut in_data: LayerPolygonCache, radius: Coord) {
        let mut data = self.m_data.lock().unwrap();
        let mut i = in_data.begin();
        Self::allocate_layers(&mut data, (i + in_data.size() as LayerIndex) as usize);
        for d in std::mem::take(in_data.polygons_mutable()) {
            data[i as usize].entry(radius).or_insert_with(|| Arc::new(d));
            i += 1;
        }
    }

    // TreeModelVolumes.hpp:242-250  getArea
    // `key.second >= m_data.size()` compares as `size_t`: a negative layer index is past the end.
    fn get_area(&self, key: RadiusLayerPair) -> Option<PolygonsRef> {
        let data = self.m_data.lock().unwrap();
        if key.1 < 0 || key.1 as usize >= data.len() {
            return None;
        }
        let layer = &data[key.1 as usize];
        layer.get(&key.0).cloned()
    }

    // TreeModelVolumes.hpp:252-266  get_lower_bound_area
    // Get a collision area at a given layer for a radius that is lower or equal to the key radius.
    fn get_lower_bound_area(&self, key: RadiusLayerPair) -> Option<(Coord, PolygonsRef)> {
        let data = self.m_data.lock().unwrap();
        if key.1 < 0 || key.1 as usize >= data.len() {
            return None;
        }
        let layer = &data[key.1 as usize];
        if layer.is_empty() {
            return None;
        }
        // auto it = layer.lower_bound(key.first);
        match layer.range(key.0..).next() {
            Some((&r, polys)) if r == key.0 => Some((r, polys.clone())),
            // if (it == layer.end() || it->first != key.first) { if (it == layer.begin()) return {}; -- it; }
            _ => layer.range(..key.0).next_back().map(|(&r, polys)| (r, polys.clone())),
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

    // TreeModelVolumes.cpp:900-911  sorted(): for debugging purposes, sorted by layer index, then by radius.
    pub fn sorted(&self) -> Vec<(RadiusLayerPair, PolygonsRef)> {
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
    // Keeps the first entry of every layer (the one of the smallest radius).
    #[allow(dead_code)]
    fn clear_all_but_radius0(&self) {
        let mut data = self.m_data.lock().unwrap();
        for l in data.iter_mut() {
            // auto begin = l.begin(); if (begin != end && ++ begin != end) l.erase(begin, end);
            if let Some((&first, _)) = l.iter().next() {
                l.retain(|&r, _| r == first);
            }
        }
    }

    // ---- The earlier interface of the crate (lib.rs re-exports this type). ----

    /// Insert polygons for a radius/layer pair.
    pub fn insert(&self, key: RadiusLayerKey, polygons: Vec<Polygon>) {
        self.insert_pairs(vec![((key.radius, key.layer_idx as LayerIndex), polygons)]);
    }

    /// Get polygons for a radius/layer pair.
    pub fn get(&self, key: &RadiusLayerKey) -> Option<Vec<Polygon>> {
        self.get_area((key.radius, key.layer_idx as LayerIndex)).map(|p| (*p).clone())
    }

    /// Whether a radius/layer pair is present.
    pub fn contains(&self, key: &RadiusLayerKey) -> bool {
        self.get_area((key.radius, key.layer_idx as LayerIndex)).is_some()
    }

    /// Maximum layer index cached for a radius.
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

    /// Lower-bound area for a radius at a layer.
    pub fn get_lower_bound(&self, radius: Coord, layer_idx: usize) -> Option<(Coord, Vec<Polygon>)> {
        self.get_lower_bound_area((radius, layer_idx as LayerIndex)).map(|(r, p)| (r, (*p).clone()))
    }
}

// =============================================================================
// TreeModelVolumes.cpp:37-51  calculateMachineBorderCollision
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
// TreeModelVolumesConfig: the parameters of the earlier constructors of the crate
// (`TreeModelVolumes::new(config)`, `with_layer_outlines`), which take no PrintObject.
// The constructor of the pin is `TreeModelVolumes::from_print_object`.
// =============================================================================

/// Configuration of the constructors that take no PrintObject.
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
    /// Create a config with per-layer height/Z arrays.
    pub fn with_layers(layer_heights: Vec<CoordF>, z_heights: Vec<CoordF>) -> Self {
        Self { layer_heights, z_heights, ..Default::default() }
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

/// `std::function<void()> throw_on_cancel`
pub type ThrowOnCancel<'a> = &'a (dyn Fn() + Sync);

/// TreeModelVolumes.hpp:37  `class TreeModelVolumes`
#[derive(Debug)]
pub struct TreeModelVolumes {
    /// TreeModelVolumes.hpp:169  Polygon m_bed_area;
    pub m_bed_area: Polygon,

    // TreeModelVolumes.hpp:420  coord_t m_max_move;
    pub(crate) m_max_move: Coord,
    // TreeModelVolumes.hpp:425  coord_t m_max_move_slow;
    pub(crate) m_max_move_slow: Coord,
    // TreeModelVolumes.hpp:429  coord_t m_min_resolution;
    pub(crate) m_min_resolution: Coord,

    // TreeModelVolumes.hpp:431  bool m_precalculated = false;
    pub(crate) m_precalculated: bool,
    // TreeModelVolumes.hpp:435  size_t m_current_outline_idx;
    pub(crate) m_current_outline_idx: usize,
    // TreeModelVolumes.hpp:439  coord_t m_current_min_xy_dist;
    pub(crate) m_current_min_xy_dist: Coord,
    // TreeModelVolumes.hpp:443  coord_t m_current_min_xy_dist_delta;
    pub(crate) m_current_min_xy_dist_delta: Coord,
    // TreeModelVolumes.hpp:447  bool m_support_rests_on_model;
    pub(crate) m_support_rests_on_model: bool,
    // TreeModelVolumes.hpp:467  coord_t m_increase_until_radius;
    pub(crate) m_increase_until_radius: Coord,

    // TreeModelVolumes.hpp:473  Polygons m_machine_border;
    pub(crate) m_machine_border: Vec<Polygon>,
    // TreeModelVolumes.hpp:477  std::vector<std::pair<TreeSupportMeshGroupSettings, std::vector<Polygons>>> m_layer_outlines;
    pub(crate) m_layer_outlines: Vec<(TreeSupportMeshGroupSettings, Vec<Vec<Polygon>>)>,
    // TreeModelVolumes.hpp:481  std::vector<Polygons> m_anti_overhang;
    pub(crate) m_anti_overhang: Vec<Vec<Polygon>>,
    // TreeModelVolumes.hpp:485  std::vector<coord_t> m_ignorable_radii;
    pub(crate) m_ignorable_radii: Vec<Coord>,
    // TreeModelVolumes.hpp:490  coord_t m_radius_0;
    pub(crate) m_radius_0: Coord,
    // TreeModelVolumes.hpp:493  std::vector<double> m_raft_layers;
    pub(crate) m_raft_layers: Vec<CoordF>,

    // TreeModelVolumes.hpp:499  RadiusLayerPolygonCache m_collision_cache;
    pub(crate) m_collision_cache: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:500  RadiusLayerPolygonCache m_collision_cache_holefree;
    pub(crate) m_collision_cache_holefree: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:501  RadiusLayerPolygonCache m_avoidance_cache;
    pub(crate) m_avoidance_cache: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:502  RadiusLayerPolygonCache m_avoidance_cache_slow;
    pub(crate) m_avoidance_cache_slow: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:503  RadiusLayerPolygonCache m_avoidance_cache_to_model;
    pub(crate) m_avoidance_cache_to_model: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:504  RadiusLayerPolygonCache m_avoidance_cache_to_model_slow;
    pub(crate) m_avoidance_cache_to_model_slow: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:505  RadiusLayerPolygonCache m_placeable_areas_cache;
    pub(crate) m_placeable_areas_cache: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:511  RadiusLayerPolygonCache m_avoidance_cache_holefree;
    pub(crate) m_avoidance_cache_holefree: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:512  RadiusLayerPolygonCache m_avoidance_cache_holefree_to_model;
    pub(crate) m_avoidance_cache_holefree_to_model: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:540  RadiusLayerPolygonCache m_wall_restrictions_cache;
    pub(crate) m_wall_restrictions_cache: RadiusLayerPolygonCache,
    // TreeModelVolumes.hpp:545  RadiusLayerPolygonCache m_wall_restrictions_cache_min;
    pub(crate) m_wall_restrictions_cache_min: RadiusLayerPolygonCache,

    // The parameters of the constructors that take no PrintObject (see TreeModelVolumesConfig).
    config: TreeModelVolumesConfig,
}

/// The members of a TreeModelVolumes other than its caches, for the tests that compare them with the pin.
#[derive(Debug, Clone)]
pub struct TreeModelVolumesMembers {
    pub m_max_move: Coord,
    pub m_max_move_slow: Coord,
    pub m_min_resolution: Coord,
    pub m_precalculated: bool,
    pub m_current_outline_idx: usize,
    pub m_current_min_xy_dist: Coord,
    pub m_current_min_xy_dist_delta: Coord,
    pub m_support_rests_on_model: bool,
    pub m_increase_until_radius: Coord,
    pub m_machine_border: Vec<Polygon>,
    pub m_layer_outlines: Vec<Vec<Vec<Polygon>>>,
    pub m_anti_overhang: Vec<Vec<Polygon>>,
    pub m_ignorable_radii: Vec<Coord>,
    pub m_radius_0: Coord,
    pub m_raft_layers: Vec<CoordF>,
}

/// The caches of a TreeModelVolumes, as `avoidance_cache()` and the members name them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TreeModelVolumesCache {
    Collision,
    CollisionHolefree,
    Avoidance,
    AvoidanceSlow,
    AvoidanceToModel,
    AvoidanceToModelSlow,
    PlaceableAreas,
    AvoidanceHolefree,
    AvoidanceHolefreeToModel,
    WallRestrictions,
    WallRestrictionsMin,
}

impl TreeModelVolumes {
    /// TreeModelVolumes.hpp:40  TreeModelVolumes() = default;
    /// with the members the earlier interface of the crate sets from a TreeModelVolumesConfig.
    pub fn new(config: TreeModelVolumesConfig) -> Self {
        // -2 to avoid rounding errors (TreeModelVolumes.cpp:62).
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
            // TreeModelVolumes.cpp:118
            m_current_min_xy_dist_delta: config.xy_distance - config.xy_min_distance,
            m_support_rests_on_model: config.support_rests_on_model,
            m_increase_until_radius: config.increase_until_radius,
            m_machine_border: calculate_machine_border_collision(Polygon::new()),
            m_layer_outlines: Vec::new(),
            m_anti_overhang: Vec::new(),
            m_ignorable_radii: Vec::new(),
            // TreeModelVolumes.cpp:121  m_radius_0 = config.getRadius(0);
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

    /// TreeModelVolumes.cpp:53-175
    /// `TreeModelVolumes::TreeModelVolumes(const PrintObject &print_object, const BuildVolume &build_volume,
    /// const coord_t max_move, const coord_t max_move_slow, size_t current_mesh_idx,
    /// const std::vector<Polygons>& additional_excluded_areas)`
    ///
    /// Over the classes of `crate::print::faithful`; `print_object.print()` is `print`. An exception of
    /// the C++ (a flow that cannot be made, a canceled slicing of the support blockers) is the `Err`.
    /// `current_mesh_idx` and `additional_excluded_areas` are read only by code the C++ compiles out
    /// (`#if 0`).
    pub fn from_print_object(
        print_object: &pf::PrintObject,
        print: &pf::Print,
        build_volume: &BuildVolume,
        max_move: Coord,
        max_move_slow: Coord,
        _current_mesh_idx: usize,
        _additional_excluded_areas: &[Vec<Polygon>],
    ) -> Result<Self, String> {
        let mut this = Self::new(TreeModelVolumesConfig::default());
        // -2 to avoid rounding errors
        this.m_max_move = std::cmp::max(max_move - 2, 0);
        this.m_max_move_slow = std::cmp::max(max_move_slow - 2, 0);
        this.m_machine_border = calculate_machine_border_collision(build_volume.polygon().clone());

        // TreeModelVolumes.cpp:68-87
        this.m_bed_area = build_volume.polygon().clone();
        let mut machine_borders: Vec<Polygon> = Vec::new();
        if !this.m_bed_area.points().is_empty() {
            let mut hole = this.m_bed_area.clone();
            hole.reverse();
            // ExPolygon machine_outline(offset(m_bed_area, scale_(1000))[0], hole);
            // `scale_(1000)` is a double, the delta of `offset()` a float.
            let delta = (1000. / crate::libslic3r::SCALING_FACTOR) as f32;
            let machine_outline = ExPolygon::with_holes(offset_polygon_raw(&this.m_bed_area, delta)[0].clone(), vec![hole]);
            // ExPolygons outlines = machine_outline.split_expoly_with_holes(scale_(1.), {});
            // `scale_(1.)` is 99999.99999999999, the `coord_t` parameter truncates it.
            let mut outlines =
                split_expoly_with_holes_no_collision(&machine_outline, (1. / crate::libslic3r::SCALING_FACTOR) as Coord);

            if cfg_bool(print.config(), "enable_wrapping_detection") {
                let wrapping_detection_area = cfg_points(print.config(), "wrapping_exclude_area");
                let mut wrapping_poly = Polygon::new();
                let shift = print_object.instances()[0].shift;
                for pt in &wrapping_detection_area {
                    // Point(scale_(pt.x()), scale_(pt.y())) - print_object.instances().front().shift
                    // `Point(double, double)` rounds with lrint (Point.hpp:179).
                    wrapping_poly.points.push(Point::new(
                        (pt.0 / crate::libslic3r::SCALING_FACTOR).round_ties_even() as Coord - shift.x,
                        (pt.1 / crate::libslic3r::SCALING_FACTOR).round_ties_even() as Coord - shift.y,
                    ));
                }
                // outlines = union_ex(outlines, {wrapping_poly});
                // ClipperUtils.cpp:817-821: one union of the paths of both, then the union into a PolyTree.
                let mut paths = to_polygons(&outlines);
                paths.push(wrapping_poly);
                let united = clipper_utils::union_flat_clib(&paths);
                outlines = if united.is_empty() { Vec::new() } else { clipper_utils::polytree_ex_clib(&united) };
            }

            for outline in &outlines {
                machine_borders.push(outline.contour.clone());
            }
        }

        // TreeModelVolumes.cpp:113-136
        {
            // m_anti_overhang = print_object.slice_support_blockers();
            this.m_anti_overhang = print_object
                .slice_support_volumes(print, crate::model::ModelVolumeType::SupportBlocker)
                .map_err(|_| "canceled".to_string())?;
            let mesh_settings = TreeSupportMeshGroupSettings::new(print_object, print).map_err(|e| format!("{e:?}"))?;
            let config = TreeSupportSettings::new(&mesh_settings, print_object.slicing_parameters());
            this.m_current_min_xy_dist = config.xy_min_distance;
            this.m_current_min_xy_dist_delta = config.xy_distance - this.m_current_min_xy_dist;
            debug_assert!(this.m_current_min_xy_dist_delta >= 0);
            this.m_increase_until_radius = config.increase_radius_until_radius;
            this.m_radius_0 = config.get_radius(0, 0.);
            this.m_raft_layers = config.raft_layers.clone();
            this.m_current_outline_idx = 0;
            let num_raft_layers = this.m_raft_layers.len();
            let num_layers = print_object.layer_count() + num_raft_layers;
            let mut outlines: Vec<Vec<Polygon>> = vec![Vec::new(); num_layers];
            // tbb::parallel_for over the object layers: every layer on its own.
            for layer_idx in num_raft_layers..num_layers {
                outlines[layer_idx] = polygons_simplify(
                    &to_polygons(&print_object.get_layer((layer_idx - num_raft_layers) as i32).lslices),
                    mesh_settings.resolution,
                );
                outlines[layer_idx].extend(machine_borders.iter().cloned());
            }
            this.m_layer_outlines.push((mesh_settings, outlines));
        }

        // TreeModelVolumes.cpp:139-144
        this.m_support_rests_on_model = false;
        this.m_min_resolution = Coord::MAX;
        for data_pair in &this.m_layer_outlines {
            this.m_support_rests_on_model |= !data_pair.0.support_material_buildplate_only;
            this.m_min_resolution = std::cmp::min(this.m_min_resolution, data_pair.0.resolution);
        }
        Ok(this)
    }

    /// The constructor of the earlier interface of the crate: the layer outlines are handed in (one
    /// `ExPolygons` per object layer) for a single mesh group with `TreeSupportMeshGroupSettings::default()`
    /// (TreeModelVolumes.cpp:124-144).
    pub fn with_layer_outlines(config: TreeModelVolumesConfig, layer_outlines: Vec<ExPolygons>) -> Self {
        let mut v = Self::new(config);

        let mesh_settings = TreeSupportMeshGroupSettings::default();
        v.m_current_outline_idx = 0;
        let outlines: Vec<Vec<Polygon>> = layer_outlines
            .iter()
            .map(|ex| {
                let mut o = polygons_simplify(&to_polygons(ex), mesh_settings.resolution);
                o.extend(v.m_machine_border.iter().cloned());
                o
            })
            .collect();
        v.m_layer_outlines.push((mesh_settings, outlines));

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

    /// The parameters of the constructors that take no PrintObject.
    pub fn config(&self) -> &TreeModelVolumesConfig {
        &self.config
    }

    /// Number of layers in the (single) outline group.
    pub fn layer_count(&self) -> usize {
        self.m_layer_outlines.first().map(|(_, o)| o.len()).unwrap_or(0)
    }

    /// The members other than the caches.
    pub fn members(&self) -> TreeModelVolumesMembers {
        TreeModelVolumesMembers {
            m_max_move: self.m_max_move,
            m_max_move_slow: self.m_max_move_slow,
            m_min_resolution: self.m_min_resolution,
            m_precalculated: self.m_precalculated,
            m_current_outline_idx: self.m_current_outline_idx,
            m_current_min_xy_dist: self.m_current_min_xy_dist,
            m_current_min_xy_dist_delta: self.m_current_min_xy_dist_delta,
            m_support_rests_on_model: self.m_support_rests_on_model,
            m_increase_until_radius: self.m_increase_until_radius,
            m_machine_border: self.m_machine_border.clone(),
            m_layer_outlines: self.m_layer_outlines.iter().map(|o| o.1.clone()).collect(),
            m_anti_overhang: self.m_anti_overhang.clone(),
            m_ignorable_radii: self.m_ignorable_radii.clone(),
            m_radius_0: self.m_radius_0,
            m_raft_layers: self.m_raft_layers.clone(),
        }
    }

    /// The mesh group settings of the outline group `idx` (`m_layer_outlines[idx].first`).
    pub fn mesh_group_settings(&self, idx: usize) -> &TreeSupportMeshGroupSettings {
        &self.m_layer_outlines[idx].0
    }

    /// A cache, as `RadiusLayerPolygonCache::sorted()` lists it.
    pub fn cache_sorted(&self, cache: TreeModelVolumesCache) -> Vec<(RadiusLayerPair, PolygonsRef)> {
        use TreeModelVolumesCache::*;
        match cache {
            Collision => &self.m_collision_cache,
            CollisionHolefree => &self.m_collision_cache_holefree,
            Avoidance => &self.m_avoidance_cache,
            AvoidanceSlow => &self.m_avoidance_cache_slow,
            AvoidanceToModel => &self.m_avoidance_cache_to_model,
            AvoidanceToModelSlow => &self.m_avoidance_cache_to_model_slow,
            PlaceableAreas => &self.m_placeable_areas_cache,
            AvoidanceHolefree => &self.m_avoidance_cache_holefree,
            AvoidanceHolefreeToModel => &self.m_avoidance_cache_holefree_to_model,
            WallRestrictions => &self.m_wall_restrictions_cache,
            WallRestrictionsMin => &self.m_wall_restrictions_cache_min,
        }
        .sorted()
    }

    // TreeModelVolumes.hpp:54-57  void clear()
    pub fn clear(&mut self) {
        self.clear_all_but_object_collision();
        self.m_collision_cache.clear();
    }

    // TreeModelVolumes.hpp:58-70  void clear_all_but_object_collision()
    pub fn clear_all_but_object_collision(&mut self) {
        //m_collision_cache.clear_all_but_radius0();
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
                // case AvoidanceType::Count: assert(false); falls through to FastSafe
                AvoidanceType::Count | AvoidanceType::FastSafe => &self.m_avoidance_cache_holefree_to_model,
            }
        } else {
            match type_ {
                AvoidanceType::Fast => &self.m_avoidance_cache,
                AvoidanceType::Slow => &self.m_avoidance_cache_slow,
                AvoidanceType::Count | AvoidanceType::FastSafe => &self.m_avoidance_cache_holefree,
            }
        }
    }

    // =========================================================================
    // TreeModelVolumes.hpp:148-154  ceilRadius(radius, min_xy_dist)
    // =========================================================================
    /// TreeModelVolumes.hpp:148  `coord_t ceilRadius(const coord_t radius, const bool min_xy_dist) const`
    pub fn ceil_radius_min_xy(&self, radius: Coord, min_xy_dist: bool) -> Coord {
        debug_assert!(radius >= 0);
        if min_xy_dist {
            self.ceil_radius(radius)
        } else if radius > 0 {
            // special case as if a radius 0 is requested it could be to ensure correct xy distance. As such it is beneficial if the collision is as close to the configured values as possible.
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
    // TreeModelVolumes.cpp:177-320  precalculate
    // =========================================================================
    /// TreeModelVolumes.cpp:177  `void precalculate(const PrintObject& print_object, const coord_t max_layer,
    /// std::function<void()> throw_on_cancel)`
    pub fn precalculate(&mut self, print_object: &pf::PrintObject, max_layer: LayerIndex, throw_on_cancel: ThrowOnCancel) {
        // Get the config corresponding to one mesh that is in the current group. Which one has to be irrelevant.
        // Not the prettiest way to do this, but it ensures some calculations that may be a bit more complex
        // like inital layer diameter are only done in once.
        let config =
            TreeSupportSettings::new(&self.m_layer_outlines[self.m_current_outline_idx].0, print_object.slicing_parameters());
        self.precalculate_with_settings(&config, max_layer, throw_on_cancel);
    }

    /// The body of `precalculate` (TreeModelVolumes.cpp:179-320) after the settings are made.
    pub fn precalculate_with_settings(&mut self, config: &TreeSupportSettings, max_layer: LayerIndex, throw_on_cancel: ThrowOnCancel) {
        self.m_precalculated = true;

        // TreeModelVolumes.cpp:187-206
        {
            // calculate which radius each layer in the tip may have.
            let mut possible_tip_radiis: Vec<Coord> = Vec::new();
            for distance_to_top in 0..=config.tip_layers {
                possible_tip_radiis.push(self.ceil_radius(config.get_radius(distance_to_top, 0.)));
                possible_tip_radiis.push(self.ceil_radius(config.get_radius(distance_to_top, 0.) + self.m_current_min_xy_dist_delta));
                for &xy_min_distance in &config.xy_min_distance_per_region {
                    possible_tip_radiis.push(self.ceil_radius(
                        config.get_radius(distance_to_top, 0.) + std::cmp::max(0, xy_min_distance - config.xy_min_distance),
                    ));
                }
            }
            // sort_remove_duplicates(possible_tip_radiis);
            possible_tip_radiis.sort_unstable();
            possible_tip_radiis.dedup();
            // It theoretically may happen in the tip, that the radius can change so much in-between 2 layers,
            // that a ceil step is skipped (as in there is a radius r so that ceilRadius(radius(dtt))<ceilRadius(r)<ceilRadius(radius(dtt+1))).
            // As such a radius will not reasonable happen in the tree and it will most likely not be requested,
            // there is no need to calculate them. So just skip these.
            let mut radius_eval = self.m_radius_0;
            while radius_eval <= config.branch_radius {
                if possible_tip_radiis.binary_search(&radius_eval).is_err() {
                    self.m_ignorable_radii.push(radius_eval);
                }
                radius_eval = self.ceil_radius(radius_eval + 1);
            }
        }

        throw_on_cancel();

        // TreeModelVolumes.cpp:211-232
        // it may seem that the required avoidance can be of a smaller radius when going to model (no initial layer diameter for to model branches)
        // but as for every branch going towards the bp, the to model avoidance is required to check for possible merges with to model branches, this assumption is in-fact wrong.
        //
        // `std::unordered_map<coord_t, LayerIndex> radius_until_layer`: the order of its keys reaches only
        // the order in which the caches are filled, and every cache entry is computed from its own key
        // and from entries that are looked up by key, so an ordered map gives the same caches.
        let mut radius_until_layer: BTreeMap<Coord, LayerIndex> = BTreeMap::new();
        // while it is possible to calculate, up to which layer the avoidance should be calculated, this simulation is easier to understand, and does not need to be adjusted if something of the radius calculation is changed.
        // Overhead with an assumed worst case of 6600 layers was about 2ms
        for distance_to_top in 0..=max_layer {
            let current_layer = max_layer - distance_to_top;
            let mut update_radius_until_layer = |r: Coord| {
                radius_until_layer.entry(r).or_insert(current_layer);
            };
            // regular radius
            update_radius_until_layer(
                self.ceil_radius(config.get_radius(distance_to_top as usize, 0.) + self.m_current_min_xy_dist_delta),
            );
            // the maximum radius that the radius with the min_xy_dist can achieve
            update_radius_until_layer(self.ceil_radius(config.get_radius(distance_to_top as usize, 0.)));
            for &xy_min_distance in &config.xy_min_distance_per_region {
                update_radius_until_layer(self.ceil_radius(
                    config.get_radius(distance_to_top as usize, 0.) + std::cmp::max(0, xy_min_distance - config.xy_min_distance),
                ));
            }
            update_radius_until_layer(
                self.ceil_radius(config.recommended_min_radius(current_layer) + self.m_current_min_xy_dist_delta),
            );
        }

        throw_on_cancel();

        // TreeModelVolumes.cpp:237-251
        // Copy to deque to use in parallel for later.
        let relevant_avoidance_radiis: Vec<RadiusLayerPair> = radius_until_layer.iter().map(|(&r, &l)| (r, l)).collect();

        // Append additional radiis needed for collision.
        // To calculate collision holefree for every radius, the collision of radius m_increase_until_radius will be required.
        radius_until_layer.insert(self.ceil_radius(self.m_increase_until_radius + self.m_current_min_xy_dist_delta), max_layer);
        // Collision for radius 0 needs to be calculated everywhere, as it will be used to ensure valid xy_distance in drawAreas.
        radius_until_layer.insert(0, max_layer);
        if self.m_current_min_xy_dist_delta != 0 {
            radius_until_layer.insert(self.m_current_min_xy_dist_delta, max_layer);
        }
        for &xy_min_distance in &config.xy_min_distance_per_region {
            radius_until_layer.insert(std::cmp::max(0, xy_min_distance - config.xy_min_distance), max_layer);
        }

        // Now that required_avoidance_limit contains the maximum of ild and regular required radius just copy.
        let relevant_collision_radiis: Vec<RadiusLayerPair> = radius_until_layer.iter().map(|(&r, &l)| (r, l)).collect();

        // TreeModelVolumes.cpp:253-266
        // Calculate the relevant collisions
        self.calculate_collision_keys(&relevant_collision_radiis, throw_on_cancel);

        // calculate a separate Collisions with all holes removed. These are relevant for some avoidances that try to avoid holes (called safe)
        let mut relevant_hole_collision_radiis: Vec<RadiusLayerPair> = Vec::new();
        for &key in &relevant_avoidance_radiis {
            if key.0 < self.m_increase_until_radius + self.m_current_min_xy_dist_delta {
                relevant_hole_collision_radiis.push(key);
            }
        }

        // Calculate collisions without holes, built from regular collision
        self.calculate_collision_holefree(&relevant_hole_collision_radiis, throw_on_cancel);
        // Let placables be calculated from calculateAvoidance() for better parallelization.
        if self.m_support_rests_on_model {
            self.calculate_placeables_keys(&relevant_avoidance_radiis, throw_on_cancel);
        }

        // TreeModelVolumes.cpp:270-276
        // Calculate the relevant avoidances in parallel as far as possible: a task group of the two calls.
        self.calculate_avoidance(&relevant_avoidance_radiis, true, self.m_support_rests_on_model, throw_on_cancel);
        self.calculate_wall_restrictions(&relevant_avoidance_radiis, throw_on_cancel);
    }

    // =========================================================================
    // TreeModelVolumes.cpp:322-333  getCollision
    // =========================================================================
    /// TreeModelVolumes.cpp:322  `const Polygons& getCollision(const coord_t orig_radius, LayerIndex layer_idx, bool min_xy_dist) const`
    pub fn get_collision(&self, orig_radius: Coord, layer_idx: LayerIndex, min_xy_dist: bool) -> PolygonsRef {
        let radius = self.ceil_radius_min_xy(orig_radius, min_xy_dist);
        if let Some(result) = self.m_collision_cache.get_area((radius, layer_idx)) {
            return result;
        }
        if self.m_precalculated {
            // BOOST_LOG_TRIVIAL(error_level_not_in_cache) << "Had to calculate collision ...";
            tree_supports_show_error("Not precalculated Collision requested.", false);
        }
        self.calculate_collision(radius, layer_idx, &|| {});
        self.get_collision(orig_radius, layer_idx, min_xy_dist)
    }

    // TreeModelVolumes.cpp:335-341  get_collision_lower_bound_area
    // Get a collision area at a given layer for a radius that is a lower or equial to the key radius.
    // It is expected that the collision area is precalculated for a given layer at least for the radius zero.
    // Used for pushing tree supports away from object during the final Organic optimization step.
    pub fn get_collision_lower_bound_area(&self, layer_id: LayerIndex, max_radius: Coord) -> Option<(Coord, PolygonsRef)> {
        self.m_collision_cache.get_lower_bound_area((max_radius, layer_id))
    }

    // TreeModelVolumes.cpp:343-356  getCollisionHolefree
    // Private. Only called internally by calculateAvoidance() and calculateAvoidanceToModel(), radius is already snapped to grid.
    fn get_collision_holefree(&self, radius: Coord, layer_idx: LayerIndex) -> PolygonsRef {
        debug_assert!(radius == self.ceil_radius(radius));
        debug_assert!(radius < self.m_increase_until_radius + self.m_current_min_xy_dist_delta);
        if let Some(result) = self.m_collision_cache_holefree.get_area((radius, layer_idx)) {
            return result;
        }
        if self.m_precalculated {
            tree_supports_show_error("Not precalculated Holefree Collision requested.", false);
        }
        // TreeModelVolumes.hpp:357-360  calculateCollisionHolefree(RadiusLayerPair key)
        self.calculate_collision_holefree(&[(radius, layer_idx)], &|| {});
        self.get_collision_holefree(radius, layer_idx)
    }

    // TreeModelVolumes.cpp:358-385  getAvoidance
    /// TreeModelVolumes.cpp:358  `const Polygons& getAvoidance(const coord_t orig_radius, LayerIndex layer_idx,
    /// AvoidanceType type, bool to_model, bool min_xy_dist) const`
    pub fn get_avoidance(
        &self,
        orig_radius: Coord,
        layer_idx: LayerIndex,
        mut type_: AvoidanceType,
        to_model: bool,
        min_xy_dist: bool,
    ) -> PolygonsRef {
        if layer_idx == 0 {
            // What on the layer directly above buildplate do i have to avoid to reach the buildplate ...
            return self.get_collision(orig_radius, layer_idx, min_xy_dist);
        }

        let radius = self.ceil_radius_min_xy(orig_radius, min_xy_dist);
        if type_ == AvoidanceType::FastSafe && radius >= self.m_increase_until_radius + self.m_current_min_xy_dist_delta {
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
        // TreeModelVolumes.hpp:378-381  calculateAvoidance(RadiusLayerPair key, bool to_build_plate, bool to_model)
        self.calculate_avoidance(&[(radius, layer_idx)], !to_model, to_model, &|| {});
        // Retrive failed and correct result was calculated. Now it has to be retrived.
        self.get_avoidance(orig_radius, layer_idx, type_, to_model, min_xy_dist)
    }

    // TreeModelVolumes.cpp:387-402  getPlaceableAreas
    /// TreeModelVolumes.cpp:387  `const Polygons& getPlaceableAreas(const coord_t orig_radius, LayerIndex layer_idx,
    /// std::function<void()> throw_on_cancel) const`
    pub fn get_placeable_areas(&self, orig_radius: Coord, layer_idx: LayerIndex, throw_on_cancel: ThrowOnCancel) -> PolygonsRef {
        let radius = self.ceil_radius(orig_radius);
        if let Some(result) = self.m_placeable_areas_cache.get_area((radius, layer_idx)) {
            return result;
        }
        if self.m_precalculated {
            tree_supports_show_error("Not precalculated Placeable areas requested.", false);
        }
        if orig_radius == 0 {
            // Placable areas for radius 0 are calculated in the general collision code.
            return self.get_collision(0, layer_idx, true);
        }
        self.calculate_placeables(radius, layer_idx, throw_on_cancel);
        self.get_placeable_areas(orig_radius, layer_idx, throw_on_cancel)
    }

    // TreeModelVolumes.cpp:404-429  getWallRestriction
    /// TreeModelVolumes.cpp:404  `const Polygons& getWallRestriction(const coord_t orig_radius, LayerIndex layer_idx, bool min_xy_dist) const`
    pub fn get_wall_restriction(&self, orig_radius: Coord, layer_idx: LayerIndex, mut min_xy_dist: bool) -> PolygonsRef {
        debug_assert!(layer_idx > 0);
        if layer_idx == 0 {
            // Should never be requested as there will be no going below layer 0 ...,
            // but just to be sure some semi-sane catch. Alternative would be empty Polygon.
            return self.get_collision(orig_radius, layer_idx, min_xy_dist);
        }

        min_xy_dist &= self.m_current_min_xy_dist_delta > 0;

        let radius = self.ceil_radius(orig_radius);
        let cache = if min_xy_dist { &self.m_wall_restrictions_cache_min } else { &self.m_wall_restrictions_cache };
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
        // TreeModelVolumes.hpp:412-415  calculateWallRestrictions(RadiusLayerPair key)
        self.calculate_wall_restrictions(&[(radius, layer_idx)], &|| {});
        // Retrieve failed and correct result was calculated. Now it has to be retrieved.
        self.get_wall_restriction(orig_radius, layer_idx, min_xy_dist)
    }

    // =========================================================================
    // TreeModelVolumes.cpp:431-442  calculateCollision (keys)
    // =========================================================================
    /// TreeModelVolumes.cpp:431  `void calculateCollision(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel)`
    fn calculate_collision_keys(&self, keys: &[RadiusLayerPair], throw_on_cancel: ThrowOnCancel) {
        // tbb::parallel_for over the keys: every key fills the cache of its own radius.
        for key in keys {
            // const LayerIndex radius = keys[ikey].first: a `coord_t` narrowed to an `int`.
            let radius = key.0 as LayerIndex as Coord;
            let max_layer_idx = key.1;
            // recursive call to parallel_for.
            self.calculate_collision(radius, max_layer_idx, throw_on_cancel);
        }
    }

    // =========================================================================
    // TreeModelVolumes.cpp:444-607  calculateCollision (radius, max_layer_idx)
    // =========================================================================
    /// TreeModelVolumes.cpp:444  `void calculateCollision(const coord_t radius, const LayerIndex max_layer_idx, std::function<void()> throw_on_cancel)`
    fn calculate_collision(&self, radius: Coord, max_layer_idx: LayerIndex, throw_on_cancel: ThrowOnCancel) {
        // Process the outlines from least layers to most layers so that the final union will run over the longest vector.
        let mut layer_outline_indices: Vec<usize> = (0..self.m_layer_outlines.len()).collect();
        // std::sort over at most a handful of outline groups; there is one group (TreeModelVolumes.cpp:124).
        layer_outline_indices.sort_by(|&i, &j| self.m_layer_outlines[i].1.len().cmp(&self.m_layer_outlines[j].1.len()));

        let mut data = LayerPolygonCache::default();
        data.allocate(self.m_collision_cache.get_max_calculated_layer(radius) + 1, max_layer_idx + 1);

        let calculate_placable = self.m_support_rests_on_model && radius == 0;
        let mut data_placeable = LayerPolygonCache::default();
        if calculate_placable {
            data_placeable.allocate(data.begin(), data.end());
        }

        for &outline_idx in &layer_outline_indices {
            let outlines = &self.m_layer_outlines[outline_idx].1;
            if outlines.is_empty() {
                continue;
            }
            let settings = &self.m_layer_outlines[outline_idx].0;
            let layer_height = settings.layer_height;
            let z_distance_bottom_layers = (settings.support_bottom_distance as f64 / layer_height as f64).round() as i32;
            let z_distance_top_layers = (settings.support_top_distance as f64 / layer_height as f64).round() as i32;
            let xy_distance = if outline_idx == self.m_current_outline_idx {
                self.m_current_min_xy_dist
            } else {
                // technically this causes collision for the normal xy_distance to be larger by m_current_min_xy_dist_delta for all
                // not currently processing meshes as this delta will be added at request time.
                // avoiding this would require saving each collision for each outline_idx separately.
                // and later for each avoidance... But avoidance calculation has to be for the whole scene and can NOT be done for each outline_idx separately and combined later.
                // so avoiding this inaccuracy seems infeasible as it would require 2x the avoidance calculations => 0.5x the performance.
                //FIXME support_xy_distance is not corrected for "soluble" flag, see TreeSupportSettings constructor.
                settings.support_xy_distance
            };

            // 1) Calculate offsets of collision areas in parallel.
            let mut collision_areas_offsetted = LayerPolygonCache::default();
            collision_areas_offsetted.allocate(
                std::cmp::max(0, data.begin() - z_distance_bottom_layers),
                std::cmp::min(outlines.len() as LayerIndex, data.end() + z_distance_top_layers),
            );
            let offset_value = radius + xy_distance;
            for layer_idx in collision_areas_offsetted.begin()..collision_areas_offsetted.end() {
                let mut collision_areas = self.m_machine_border.clone();
                collision_areas.extend(outlines[layer_idx as usize].iter().cloned());
                // jtRound is not needed here, as the overshoot can not cause errors in the algorithm, because no assumptions are made about the model.
                // if a key does not exist when it is accessed it is added!
                *collision_areas_offsetted.get_mut(layer_idx) = if offset_value == 0 {
                    union_(&collision_areas)
                } else {
                    offset_ex(&union_ex(&collision_areas), offset_value as f32, OffsetJoinType::Miter, 1.2)
                };
                throw_on_cancel();
            }

            // 2) Sum over top / bottom ranges.
            // `outline_idx == layer_outline_indices.size()` never holds: the anti overhangs are not
            // added and the layers are not simplified here.
            let processing_last_mesh = outline_idx == layer_outline_indices.len();
            for layer_idx in data.begin()..data.end() {
                let mut collisions: Vec<Polygon> = Vec::new();
                for i in -z_distance_bottom_layers..=0 {
                    let j = layer_idx + i;
                    if collision_areas_offsetted.has(j) {
                        collisions.extend(collision_areas_offsetted.get(j).iter().cloned());
                    }
                }
                for i in 1..=z_distance_top_layers {
                    let j = layer_idx + i;
                    if j < outlines.len() as i32 {
                        let mut collision_areas_original = self.m_machine_border.clone();
                        collision_areas_original.extend(outlines[j as usize].iter().cloned());

                        // If just the collision (including the xy distance) of the layers above is accumulated, it leads to the
                        // following issue:
                        // Example: assuming the z distance is 2 layer
                        // + = xy_distance
                        // - = model
                        // o = overhang of the area two layers above that should result in tips on this layer
                        //
                        //  +-----+
                        //   +-----+
                        //    +-----+
                        //   o +-----+
                        // If just the collision above is accumulated the overhang will get overwritten by the xy_distance of the
                        // layer below the overhang...
                        //
                        // This only causes issues if the overhang area is thinner than xy_distance
                        // Just accumulating areas of the model above without the xy distance is also problematic, as then support
                        // may get closer to the model (on the diagonal downwards) than the user intended. Example (s = support):
                        //  +-----+
                        //   +-----+
                        //   +-----+
                        //   s+-----+

                        // technically the calculation below is off by one layer, as the actual distance between plastic one layer
                        // down is 0 not layer height, as this layer is filled with said plastic. But otherwise a part of the
                        // overhang that is expected to be supported is overwritten by the remaining part of the xy distance of the
                        // layer below the to be supported area.
                        let required_range_x: Coord = (xy_distance as f64
                            - ((i as f64 - (if z_distance_top_layers == 1 { 0.5 } else { 0. })) * xy_distance as f64
                                / z_distance_top_layers as f64)) as Coord;
                        // the conditional -0.5 ensures that plastic can never touch on the diagonal
                        // downward when the z_distance_top_layers = 1. It is assumed to be better to
                        // not support an overhang<90 degree than to risk fusing to it.
                        collisions.extend(offset_ex(
                            &union_ex(&collision_areas_original),
                            (radius + required_range_x) as f32,
                            OffsetJoinType::Miter,
                            1.2,
                        ));
                    }
                }
                collisions = if processing_last_mesh && layer_idx < self.m_anti_overhang.len() as i32 {
                    union_2(
                        &collisions,
                        &offset_ex(&union_ex(&self.m_anti_overhang[layer_idx as usize]), radius as f32, OffsetJoinType::Miter, 1.2),
                    )
                } else {
                    union_(&collisions)
                };
                let dst = data.get_mut(layer_idx);
                if processing_last_mesh {
                    if !dst.is_empty() {
                        collisions = union_2(&collisions, dst);
                    }
                    *dst = polygons_simplify(&collisions, self.m_min_resolution);
                } else {
                    append(dst, collisions);
                }
                throw_on_cancel();
            }

            // 3) Optionally calculate placables.
            if calculate_placable {
                // Calculating both the collision areas and placable areas.
                for layer_idx in std::cmp::max(z_distance_bottom_layers + 1, data.begin())..data.end() {
                    let layer_idx_below = layer_idx - z_distance_bottom_layers - 1;
                    debug_assert!(layer_idx_below >= 0);
                    let current = collision_areas_offsetted.get(layer_idx);
                    let below = &outlines[layer_idx_below as usize];
                    let mut placable = diff(
                        // Inflate the surface to sit on by the separation distance to increase chance of a support being placed on a sloped surface.
                        // `offset(below, xy_distance)`: DefaultJoinType (jtMiter), DefaultMiterLimit (3).
                        &offset_polys(below, xy_distance as f32, OffsetJoinType::Miter, 3.0),
                        &if layer_idx_below < self.m_anti_overhang.len() as i32 {
                            union_2(current, &self.m_anti_overhang[layer_idx_below as usize])
                        } else {
                            current.clone()
                        },
                    );
                    let dst = data_placeable.get_mut(layer_idx);
                    if processing_last_mesh {
                        if !dst.is_empty() {
                            placable = union_2(&placable, dst);
                        }
                        *dst = polygons_simplify(&placable, self.m_min_resolution);
                    } else {
                        append(dst, placable);
                    }
                    throw_on_cancel();
                }
            } else {
                // Calculating just the collision areas.
            }
        }
        throw_on_cancel();
        self.m_collision_cache.insert_cache(data, radius);
        if calculate_placable {
            self.m_placeable_areas_cache.insert_cache(data_placeable, radius);
        }
    }

    // =========================================================================
    // TreeModelVolumes.cpp:609-639  calculateCollisionHolefree
    // =========================================================================
    /// TreeModelVolumes.cpp:609  `void calculateCollisionHolefree(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel)`
    fn calculate_collision_holefree(&self, keys: &[RadiusLayerPair], throw_on_cancel: ThrowOnCancel) {
        let mut max_layer: LayerIndex = 0;
        for key in keys {
            max_layer = std::cmp::max(max_layer, key.1);
        }

        // tbb::parallel_for over blocks of layers; a block inserts what it made, and an entry that is
        // there already is kept, so the blocks may be taken as one.
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
                    // this union is important as otherwise holes(in form of lines that will increase to holes in a later step) can get unioned onto the area.
                    data.push((
                        (radius, layer_idx),
                        polygons_simplify(
                            &offset_ex(
                                &union_ex(&self.get_collision(self.m_increase_until_radius, layer_idx, false)),
                                (5 - increase_radius_ceil) as f32,
                                OffsetJoinType::Round,
                                self.m_min_resolution as f64,
                            ),
                            self.m_min_resolution,
                        ),
                    ));
                    throw_on_cancel();
                }
            }
        }
        self.m_collision_cache_holefree.insert_pairs(data);
    }

    // =========================================================================
    // TreeModelVolumes.cpp:641-741  calculateAvoidance
    // =========================================================================
    /// TreeModelVolumes.cpp:641  `void calculateAvoidance(const std::vector<RadiusLayerPair> &keys, bool to_build_plate, bool to_model, std::function<void()> throw_on_cancel)`
    fn calculate_avoidance(&self, keys: &[RadiusLayerPair], to_build_plate: bool, to_model: bool, throw_on_cancel: ThrowOnCancel) {
        // For every RadiusLayer pair there are 3 avoidances that have to be calculated.
        // Prepare tasks for parallelization.
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

        let count = AvoidanceType::Count as i32;
        for iter_idx in 0..(2 * keys.len() as i32 * count) {
            let mut task = AvoidanceTask {
                type_: AvoidanceType::from_i32(iter_idx % count),
                radius: keys[(iter_idx / 6) as usize].0,
                max_required_layer: keys[(iter_idx / 6) as usize].1,
                to_model: ((iter_idx / 3) & 1) != 0,
                start_layer: 0,
            };
            // Ensure start_layer is at least 1 as if no avoidance was calculated yet getMaxCalculatedLayer() returns -1.
            task.start_layer =
                std::cmp::max(1, 1 + self.avoidance_cache(task.type_, task.to_model).get_max_calculated_layer(task.radius));
            if task.start_layer > task.max_required_layer {
                // BOOST_LOG_TRIVIAL(debug) << "Calculation requested for value already calculated?";
                continue;
            }
            if (if task.to_model { to_model } else { to_build_plate })
                && (!task.holefree() || task.radius < self.m_increase_until_radius + self.m_current_min_xy_dist_delta)
            {
                avoidance_tasks.push(task);
            }
        }

        throw_on_cancel();

        // tbb::parallel_for over the tasks, one task a range; a task fills the cache of its own type and radius.
        for task in &avoidance_tasks {
            debug_assert!(!task.holefree() || task.radius < self.m_increase_until_radius + self.m_current_min_xy_dist_delta);
            if task.to_model {
                // ensuring Placeableareas are calculated
                //FIXME pass throw_on_cancel
                self.get_placeable_areas(task.radius, task.max_required_layer, throw_on_cancel);
            }
            // The following loop propagating avoidance regions bottom up is inherently serial.
            let collision_holefree =
                (task.slow() || task.holefree()) && task.radius < self.m_increase_until_radius + self.m_current_min_xy_dist_delta;
            // `const float max_move`, `float move_step`, `float last_move_step`: single precision.
            let max_move: f32 = if task.slow() { self.m_max_move_slow } else { self.m_max_move } as f32;
            // Limiting the offset step so that unioning the shrunk latest_avoidance with the current layer collisions
            // will not create gaps in the resulting avoidance region letting a tree support branch tunneling through an object wall.
            let mut move_step: f32 = (1.9 * std::cmp::max(task.radius, self.m_current_min_xy_dist) as f64) as f32;
            // `if (move_step < EPSILON) return;` leaves the body of the parallel loop, whose ranges are
            // single tasks (grain size 1): the task is skipped.
            if (move_step as f64) < EPSILON {
                continue;
            }
            // int move_steps = round_up_divide<int>(max_move, move_step): both floats truncated to int.
            let mut move_steps: i32 = {
                let dividend = max_move as i32;
                let divisor = move_step as i32;
                (dividend + divisor - 1) / divisor
            };
            debug_assert!(move_steps > 0);
            // float last_move_step = max_move - (move_steps - 1) * move_step: one expression, the
            // product fused into the subtraction.
            let mut last_move_step: f32 = (-((move_steps - 1) as f32)).mul_add(move_step, max_move);
            // scaled<float>(0.05)
            if last_move_step < (0.05 / crate::libslic3r::SCALING_FACTOR) as f32 {
                //assert(move_steps > 1);
                if move_steps > 1 {
                    // Avoid taking a very short last step, stretch the other steps a bit instead.
                    move_steps -= 1;
                    move_step = max_move / move_steps as f32;
                    last_move_step = move_step;
                }
            }
            // minDist as the delta was already added, also avoidance for layer 0 will return the collision.
            let mut latest_avoidance: Vec<Polygon> =
                (*self.get_avoidance(task.radius, task.start_layer - 1, task.type_, task.to_model, true)).clone();
            let mut data: Vec<(RadiusLayerPair, Vec<Polygon>)> = Vec::new();
            for layer_idx in task.start_layer..=task.max_required_layer {
                // Merge current layer collisions with shrunk last_avoidance.
                let current_layer_collisions = if collision_holefree {
                    self.get_collision_holefree(task.radius, layer_idx)
                } else {
                    self.get_collision(task.radius, layer_idx, true)
                };
                // For mildly steep branch angles only one step will be taken.
                for istep in 0..move_steps {
                    latest_avoidance = union_2(
                        &current_layer_collisions,
                        &offset_polys(
                            &latest_avoidance,
                            if istep + 1 == move_steps { -last_move_step } else { -move_step },
                            OffsetJoinType::Round,
                            self.m_min_resolution as f64,
                        ),
                    );
                }
                if task.to_model {
                    latest_avoidance = diff(&latest_avoidance, &self.get_placeable_areas(task.radius, layer_idx, throw_on_cancel));
                }
                latest_avoidance = polygons_simplify(&latest_avoidance, self.m_min_resolution);
                data.push(((task.radius, layer_idx), latest_avoidance.clone()));
                throw_on_cancel();
            }
            self.avoidance_cache(task.type_, task.to_model).insert_pairs(data);
        }
    }

    // =========================================================================
    // TreeModelVolumes.cpp:744-751  calculatePlaceables (keys)
    // =========================================================================
    /// TreeModelVolumes.cpp:744  `void calculatePlaceables(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel)`
    fn calculate_placeables_keys(&self, keys: &[RadiusLayerPair], throw_on_cancel: ThrowOnCancel) {
        // tbb::parallel_for over the keys: every key fills the cache of its own radius.
        for key in keys {
            self.calculate_placeables(key.0, key.1, throw_on_cancel);
        }
    }

    // =========================================================================
    // TreeModelVolumes.cpp:753-790  calculatePlaceables (radius, max_required_layer)
    // =========================================================================
    /// TreeModelVolumes.cpp:753  `void calculatePlaceables(const coord_t radius, const LayerIndex max_required_layer, std::function<void()> throw_on_cancel)`
    fn calculate_placeables(&self, radius: Coord, max_required_layer: LayerIndex, throw_on_cancel: ThrowOnCancel) {
        let start_layer = 1 + self.m_placeable_areas_cache.get_max_calculated_layer(radius);
        if start_layer > max_required_layer {
            // BOOST_LOG_TRIVIAL(debug) << "Requested calculation for value already calculated ?";
            return;
        }

        let mut data: Vec<Vec<Polygon>> = vec![Vec::new(); (max_required_layer + 1 - start_layer) as usize];

        if start_layer == 0 {
            data[0] = diff(&self.m_machine_border, &self.get_collision(radius, 0, true));
        }

        for layer_idx in std::cmp::max(1, start_layer)..(max_required_layer + 1) {
            data[(layer_idx - start_layer) as usize] = offset_ex(
                &union_ex(&self.get_placeable_areas(0, layer_idx, throw_on_cancel)),
                // As a placeable area is calculated by (collision of the layer below) - (collision of the current layer) and the collision is offset by xy_distance,
                // it can happen that a small line is considered a flat area to place something onto, even though it is mostly
                // xy_distance that cant support it. Making the area smaller by xy_distance fixes this.
                -(radius + self.m_current_min_xy_dist + self.m_current_min_xy_dist_delta) as f32,
                OffsetJoinType::Miter,
                1.2,
            );
            throw_on_cancel();
        }
        self.m_placeable_areas_cache.insert_layers(data, start_layer, radius);
    }

    // =========================================================================
    // TreeModelVolumes.cpp:792-860  calculateWallRestrictions
    // =========================================================================
    /// TreeModelVolumes.cpp:792  `void calculateWallRestrictions(const std::vector<RadiusLayerPair> &keys, std::function<void()> throw_on_cancel)`
    fn calculate_wall_restrictions(&self, keys: &[RadiusLayerPair], throw_on_cancel: ThrowOnCancel) {
        // Wall restrictions are mainly important when they represent actual walls that are printed, and not "just" the configured z_distance, because technically valid placement is no excuse for moving through a wall.
        // As they exist to prevent accidentially moving though a wall at high speed between layers like thie (x = wall,i = influence area,o= empty space,d = blocked area because of z distance) Assume maximum movement distance is two characters and maximum safe movement distance of one character

        /* Potential issue addressed by the wall restrictions: Influence area may lag through a wall
         *  layer z+1:iiiiiiiiiiioooo
         *  layer z+0:xxxxxiiiiiiiooo
         *  layer z-1:ooooixxxxxxxxxx
         */

        // The radius for the upper collission has to be 0 as otherwise one may not enter areas that may be forbidden on layer_idx but not one below (c = not an influence area even though it should ):
        /*
         *  layer z+1:xxxxxiiiiiioo
         *  layer z+0:dddddiiiiiiio
         *  layer z-1:dddocdddddddd
         */
        // Also there can not just the collision of the lower layer be used because if it were:
        /*
         *  layer z+1:dddddiiiiiiiiiio
         *  layer z+0:xxxxxddddddddddc
         *  layer z-1:dddddxxxxxxxxxxc
         */
        // Or of the upper layer be used because if it were:
        /*
         *  layer z+1:dddddiiiiiiiiiio
         *  layer z+0:xxxxcddddddddddc
         *  layer z-1:ddddcxxxxxxxxxxc
         */

        // And just offseting with maximum movement distance (and not in multiple steps) could cause:
        /*
         *  layer z:   oxiiiiiiiiioo
         *  layer z-1: ixiiiiiiiiiii
         */

        // tbb::parallel_for over the keys: every key fills the caches of its own radius.
        for key in keys {
            let radius = key.0;
            let max_required_layer = key.1;
            let min_layer_bottom: Coord = std::cmp::max(1, self.m_wall_restrictions_cache.get_max_calculated_layer(radius)) as Coord;
            let buffer_size = (max_required_layer as Coord + 1 - min_layer_bottom) as usize;
            let mut data: Vec<Vec<Polygon>> = vec![Vec::new(); buffer_size];
            let mut data_min: Vec<Vec<Polygon>> = Vec::new();
            if self.m_current_min_xy_dist_delta > 0 {
                data_min = vec![Vec::new(); buffer_size];
            }
            let min_layer_bottom = min_layer_bottom as LayerIndex;
            for layer_idx in min_layer_bottom..(max_required_layer + 1) {
                data[(layer_idx - min_layer_bottom) as usize] = polygons_simplify(
                    // radius contains m_current_min_xy_dist_delta already if required
                    &intersection(&self.get_collision(0, layer_idx, false), &self.get_collision(radius, layer_idx - 1, true)),
                    self.m_min_resolution,
                );
                if !data_min.is_empty() {
                    data_min[(layer_idx - min_layer_bottom) as usize] = polygons_simplify(
                        &intersection(&self.get_collision(0, layer_idx, true), &self.get_collision(radius, layer_idx - 1, true)),
                        self.m_min_resolution,
                    );
                }
                throw_on_cancel();
            }
            self.m_wall_restrictions_cache.insert_layers(data, min_layer_bottom, radius);
            if !data_min.is_empty() {
                self.m_wall_restrictions_cache_min.insert_layers(data_min, min_layer_bottom, radius);
            }
        }
    }

    // =========================================================================
    // TreeModelVolumes.cpp:862-889  ceilRadius (single argument)
    // =========================================================================
    /// TreeModelVolumes.cpp:862  `coord_t ceilRadius(const coord_t radius) const`
    pub fn ceil_radius(&self, radius: Coord) -> Coord {
        if radius == 0 {
            return 0;
        }

        let mut out = self.m_radius_0;
        if radius > self.m_radius_0 {
            // generate SUPPORT_TREE_PRE_EXPONENTIAL_STEPS of radiis before starting to exponentially increase them.
            let initial_radius_delta = SUPPORT_TREE_EXPONENTIAL_THRESHOLD - self.m_radius_0;
            let ignore = |r: Coord| self.m_ignorable_radii.binary_search(&r).is_ok();
            if initial_radius_delta > SUPPORT_TREE_COLLISION_RESOLUTION {
                // const int num_steps, const int stepsize
                let num_steps = crate::utils::round_up_divide(initial_radius_delta, SUPPORT_TREE_EXPONENTIAL_THRESHOLD) as i32;
                let stepsize = (initial_radius_delta / num_steps as Coord) as i32;
                out += stepsize as Coord;
                for _step in 0..num_steps {
                    if out >= radius && !ignore(out) {
                        return out;
                    }
                    out += stepsize as Coord;
                }
            } else {
                out += SUPPORT_TREE_COLLISION_RESOLUTION;
            }
            while out < radius || ignore(out) {
                debug_assert!(
                    (out as CoordF * SUPPORT_TREE_EXPONENTIAL_FACTOR) > (out + SUPPORT_TREE_COLLISION_RESOLUTION) as CoordF
                );
                out = (out as CoordF * SUPPORT_TREE_EXPONENTIAL_FACTOR) as Coord;
            }
        }
        out
    }

    // =========================================================================
    // The earlier interface of the crate (support/mod.rs and the legacy generator of
    // tree_support_3d.rs), over the methods above.
    // =========================================================================

    /// `getCollision(orig_radius, layer_idx, min_xy_dist)`, the polygons copied.
    pub fn get_collision_min_xy(&self, orig_radius: Coord, layer_idx: LayerIndex, min_xy_dist: bool) -> Vec<Polygon> {
        (*self.get_collision(orig_radius, layer_idx, min_xy_dist)).clone()
    }

    /// `getCollisionHolefree(ceilRadius(radius), layer_idx)`, the polygons copied.
    pub fn get_collision_holefree_pub(&self, radius: Coord, layer_idx: usize) -> Vec<Polygon> {
        let r = self.ceil_radius(radius);
        (*self.get_collision_holefree(r, layer_idx as LayerIndex)).clone()
    }

    /// `getAvoidance(orig_radius, layer_idx, type, to_model, min_xy_dist)`, the polygons copied.
    pub fn get_avoidance_full(
        &self,
        orig_radius: Coord,
        layer_idx: LayerIndex,
        type_: AvoidanceType,
        to_model: bool,
        min_xy_dist: bool,
    ) -> Vec<Polygon> {
        (*self.get_avoidance(orig_radius, layer_idx, type_, to_model, min_xy_dist)).clone()
    }

    /// `getPlaceableAreas(orig_radius, layer_idx, []{})`, the polygons copied.
    pub fn get_placeable_areas_vec(&self, orig_radius: Coord, layer_idx: LayerIndex) -> Vec<Polygon> {
        (*self.get_placeable_areas(orig_radius, layer_idx, &|| {})).clone()
    }

    /// `getWallRestriction(orig_radius, layer_idx, min_xy_dist)`, the polygons copied.
    pub fn get_wall_restriction_vec(&self, orig_radius: Coord, layer_idx: LayerIndex, min_xy_dist: bool) -> Vec<Polygon> {
        (*self.get_wall_restriction(orig_radius, layer_idx, min_xy_dist)).clone()
    }

    /// `getRadiusNextCeil` with `min_xy_dist` true.
    pub fn next_ceil_radius(&self, radius: Coord) -> Coord {
        if radius == 0 {
            // C++ asserts radius > 0.
            return self.ceil_radius(0);
        }
        self.get_radius_next_ceil(radius, true)
    }

    /// Whether `precalculate` has been called.
    pub fn is_precalculated(&self) -> bool {
        self.m_precalculated
    }
}

// `config.<key>.value` of a bool option.
fn cfg_bool(config: &crate::print_config::config_def::DynamicPrintConfig, key: &str) -> bool {
    use crate::config::{ConfigOption, ConfigOptionResolver};
    config.option(key).and_then(|o| o.get_bool().ok()).unwrap_or(false)
}

// `config.<key>.values` of a `ConfigOptionPoints`.
fn cfg_points(config: &crate::print_config::config_def::DynamicPrintConfig, key: &str) -> Vec<(f64, f64)> {
    use crate::config::{ConfigOption, ConfigOptionResolver};
    config
        .option(key)
        .and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionPoints>())
        .map(|o| o.values.iter().map(|v| (v.x, v.y)).collect())
        .unwrap_or_default()
}

// =============================================================================
// Free helpers of the earlier interface of the crate (lib.rs re-exports them).
// Not part of the C++ TreeModelVolumes.
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
    let collision = volumes.get_collision(radius, layer_idx as LayerIndex, true);
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

    let collision = volumes.get_collision(radius, layer_idx as LayerIndex, true);

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
        let collision = volumes.get_collision(scale(1.0), 0, true);
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

    // `is_safe_position` is a crate helper over getCollision; the collision of a
    // 10 mm square at radius 0 is the square grown by m_current_min_xy_dist
    // (TreeModelVolumes.cpp:490-492, offset in scaled units). Before gate
    // TREE_MODEL_VOLUMES_OFFSET the offset took the scaled delta as millimetres
    // and came back empty, so the origin read as safe.
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
