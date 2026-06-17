//! 2D Honeycomb infill pattern.
//!
//! C++ Reference:
//! - Fill/FillHoneycomb.hpp
//! - Fill/FillHoneycomb.cpp
//!
//! Faithful 1:1 port of `Slic3r::FillHoneycomb` (FillHoneycomb.cpp). Generates a
//! 2D honeycomb (hexagonal) infill pattern. The pattern tiles hexagons across the
//! fill region with zigzag polylines that form the honeycomb walls.

// FillHoneycomb.cpp:1-5
//   #include "../ClipperUtils.hpp"
//   #include "../ShortestPath.hpp"
//   #include "../Surface.hpp"
//   #include "FillHoneycomb.hpp"
use super::{connect_infill_expolygon, multiline_fill, FillParams};
use crate::clipper_utils::intersection_pl;
use crate::geometry::{align_to_grid_point, BoundingBox, ExPolygon, Point, Polyline};
use crate::shortest_path::chain_polylines;
use crate::{Coord, CoordF, SCALING_FACTOR};

// FillHoneycomb.cpp:7 — namespace Slic3r

/// Cache key for honeycomb geometry.
///
/// FillHoneycomb.hpp:28-38 — `struct CacheID`.
#[derive(Debug, Clone, Copy)]
pub struct CacheID {
    /// FillHoneycomb.hpp:32 — `float density;`
    pub density: f32,
    /// FillHoneycomb.hpp:33 — `coordf_t spacing;`
    pub spacing: CoordF,
}

impl CacheID {
    /// FillHoneycomb.hpp:30-31 — `CacheID(float adensity, coordf_t aspacing)`.
    pub fn new(adensity: f32, aspacing: CoordF) -> Self {
        Self {
            density: adensity,
            spacing: aspacing,
        }
    }
}

impl PartialEq for CacheID {
    /// FillHoneycomb.hpp:36-37 — `bool operator==(const CacheID &other) const`.
    fn eq(&self, other: &Self) -> bool {
        // return density == other.density && spacing == other.spacing;
        self.density == other.density && self.spacing == other.spacing
    }
}
impl Eq for CacheID {}

impl PartialOrd for CacheID {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for CacheID {
    /// FillHoneycomb.hpp:34-35 — `bool operator<(const CacheID &other) const`.
    /// `(density < other.density) || (density == other.density && spacing < other.spacing)`.
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        use std::cmp::Ordering;
        if self.density < other.density {
            Ordering::Less
        } else if self.density == other.density {
            if self.spacing < other.spacing {
                Ordering::Less
            } else if self.spacing == other.spacing {
                Ordering::Equal
            } else {
                Ordering::Greater
            }
        } else {
            Ordering::Greater
        }
    }
}

/// Cached honeycomb geometry for a given density/spacing pair.
///
/// FillHoneycomb.hpp:39-49 — `struct CacheData`.
#[derive(Debug, Clone, Default)]
pub struct CacheData {
    /// FillHoneycomb.hpp:41 — `coord_t distance;`
    pub distance: Coord,
    /// FillHoneycomb.hpp:42 — `coord_t hex_side;`
    pub hex_side: Coord,
    /// FillHoneycomb.hpp:43 — `coord_t hex_width;`
    pub hex_width: Coord,
    /// FillHoneycomb.hpp:44 — `coord_t pattern_height;`
    pub pattern_height: Coord,
    /// FillHoneycomb.hpp:45 — `coord_t y_short;`
    pub y_short: Coord,
    /// FillHoneycomb.hpp:46 — `coord_t x_offset;`
    pub x_offset: Coord,
    /// FillHoneycomb.hpp:47 — `coord_t y_offset;`
    pub y_offset: Coord,
    /// FillHoneycomb.hpp:48 — `Point hex_center;`
    pub hex_center: Point,
}

/// FillHoneycomb.hpp:50-51 — `typedef std::map<CacheID, CacheData> Cache; Cache cache;`.
///
/// `std::map` is an ordered associative container keyed by `CacheID`; we keep the
/// ordered semantics via a sorted `Vec<(CacheID, CacheData)>`.
pub type Cache = Vec<(CacheID, CacheData)>;

/// FillHoneycomb pattern generator.
///
/// FillHoneycomb.hpp:12-13 — `class FillHoneycomb : public Fill`.
///
/// The base `Slic3r::Fill` members that this filler actually reads (`spacing`)
/// are held here directly, mirroring the inherited C++ fields.
#[derive(Debug, Clone, Default)]
pub struct FillHoneycomb {
    /// Base `Fill::spacing` in unscaled coordinates (FillBase.hpp:115).
    pub spacing: CoordF,
    /// FillHoneycomb.hpp:51 — `Cache cache;`.
    pub cache: Cache,
}

impl FillHoneycomb {
    pub fn new(spacing: CoordF) -> Self {
        Self {
            spacing,
            cache: Cache::new(),
        }
    }

    /// FillHoneycomb.hpp:16 — `bool is_self_crossing() override { return false; }`.
    pub fn is_self_crossing(&self) -> bool {
        false
    }

    /// FillHoneycomb.hpp:53 — `float _layer_angle(size_t idx) const override`.
    /// `{ return float(M_PI/3.) * (idx % 3); }`.
    pub fn layer_angle(&self, idx: usize) -> f32 {
        (std::f64::consts::PI / 3.) as f32 * (idx % 3) as f32
    }

    /// FillHoneycomb.cpp:9-84 — `void FillHoneycomb::_fill_surface_single(...)`.
    pub fn fill_surface_single(
        &mut self,
        params: &FillParams,
        _thickness_layers: u32,
        direction: &(f32, Point),
        expolygon: ExPolygon,
        polylines_out: &mut Vec<Polyline>,
    ) {
        // FillHoneycomb.cpp:16 — cache hexagons math
        // FillHoneycomb.cpp:17
        let cache_id = CacheID::new(params.density as f32, self.spacing);
        // FillHoneycomb.cpp:18-32 — find or insert the cache entry.
        if self
            .cache
            .binary_search_by(|(k, _)| k.cmp(&cache_id))
            .is_err()
        {
            // FillHoneycomb.cpp:20-21
            let mut m = CacheData::default();
            // FillHoneycomb.cpp:22 — coord_t min_spacing = coord_t(scale_(this->spacing)) * params.multiline;
            // scale_(v) == v / SCALING_FACTOR (libslic3r.h:81); coord_t(...) truncates toward zero.
            // The crate's SCALING_FACTOR (1e5) is the reciprocal of C++'s (1e-5), so multiplying
            // matches `scale_(v) = v / 0.00001`. `as Coord` truncates toward zero like `coord_t(...)`.
            // FIDELITY-NOTE(F2): C++ coord_t is int32 (libslic3r.h:40); here Coord = i64, so the
            // intermediate `coord_t(scale_(spacing))` and `min_spacing` do not wrap to int32 as in
            // C++. For realistic honeycomb spacings these values stay well within int32 range.
            let min_spacing: Coord =
                ((self.spacing * SCALING_FACTOR) as Coord) * params.multiline as Coord;
            // FillHoneycomb.cpp:23 — m.distance = coord_t(min_spacing / params.density);
            // params.density is a `float` in C++ (FillBase.hpp:54), so `min_spacing / params.density`
            // is evaluated in single precision before truncating; mirror that with f32 here.
            m.distance = (min_spacing as f32 / params.density as f32) as Coord;
            // FillHoneycomb.cpp:24 — m.hex_side = coord_t(m.distance / (sqrt(3)/2));
            m.hex_side = (m.distance as f64 / (3.0_f64.sqrt() / 2.0)) as Coord;
            // FillHoneycomb.cpp:25 — m.hex_width = m.distance * 2; // == hex_side * sqrt(3)
            m.hex_width = m.distance * 2;
            // FillHoneycomb.cpp:26 — coord_t hex_height = m.hex_side * 2;
            let hex_height: Coord = m.hex_side * 2;
            // FillHoneycomb.cpp:27 — m.pattern_height = hex_height + m.hex_side;
            m.pattern_height = hex_height + m.hex_side;
            // FillHoneycomb.cpp:28 — m.y_short = coord_t(m.distance * sqrt(3)/3);
            m.y_short = (m.distance as f64 * 3.0_f64.sqrt() / 3.0) as Coord;
            // FillHoneycomb.cpp:29 — m.x_offset = min_spacing / 2;
            m.x_offset = min_spacing / 2;
            // FillHoneycomb.cpp:30 — m.y_offset = coord_t(m.x_offset * sqrt(3)/3);
            m.y_offset = (m.x_offset as f64 * 3.0_f64.sqrt() / 3.0) as Coord;
            // FillHoneycomb.cpp:31 — m.hex_center = Point(m.hex_width/2, m.hex_side);
            m.hex_center = Point::new(m.hex_width / 2, m.hex_side);
            // FillHoneycomb.cpp:18-20 — insert keeping the map ordered by CacheID.
            let pos = self
                .cache
                .binary_search_by(|(k, _)| k.cmp(&cache_id))
                .unwrap_or_else(|e| e);
            self.cache.insert(pos, (cache_id, m));
        }
        // FillHoneycomb.cpp:33 — CacheData &m = it_m->second;
        let m = self
            .cache
            .iter()
            .find(|(k, _)| *k == cache_id)
            .map(|(_, v)| v.clone())
            .expect("cache entry was just inserted");

        // FillHoneycomb.cpp:35 — Polylines all_polylines;
        let mut all_polylines: Vec<Polyline> = Vec::new();
        // FillHoneycomb.cpp:36
        {
            // FillHoneycomb.cpp:37-38
            // adjust actual bounding box to the nearest multiple of our hex pattern
            // and align it so that it matches across layers

            // FillHoneycomb.cpp:40 — BoundingBox bounding_box = expolygon.contour.bounding_box();
            let mut bounding_box: BoundingBox = expolygon.contour.bounding_box();
            // FillHoneycomb.cpp:41
            {
                // FillHoneycomb.cpp:42 — rotate bounding box according to infill direction
                // FillHoneycomb.cpp:43 — Polygon bb_polygon = bounding_box.polygon();
                let mut bb_polygon = bounding_box.polygon();
                // FillHoneycomb.cpp:44 — bb_polygon.rotate(direction.first, m.hex_center);
                bb_polygon.rotate_around(direction.0 as CoordF, m.hex_center);
                // FillHoneycomb.cpp:45 — bounding_box = bb_polygon.bounding_box();
                bounding_box = bb_polygon.bounding_box();

                // FillHoneycomb.cpp:47-49
                // extend bounding box so that our pattern will be aligned with other layers
                // $bounding_box->[X1] and [Y1] represent the displacement between new bounding box offset and old one
                // The infill is not aligned to the object bounding box, but to a world coordinate system. Supposedly good enough.
                // FillHoneycomb.cpp:50 — bounding_box.merge(align_to_grid(bounding_box.min, Point(m.hex_width, m.pattern_height)));
                let aligned = align_to_grid_point(
                    bounding_box.min,
                    Point::new(m.hex_width, m.pattern_height),
                );
                bounding_box.merge_point(aligned);
            }

            // FillHoneycomb.cpp:53 — coord_t x = bounding_box.min(0);
            let mut x: Coord = bounding_box.min.x;
            // FillHoneycomb.cpp:54
            while x <= bounding_box.max.x {
                // FillHoneycomb.cpp:55 — Polyline p;
                let mut p = Polyline::new();
                // FillHoneycomb.cpp:56 — coord_t ax[2] = { x + m.x_offset, x + m.distance - m.x_offset };
                let mut ax: [Coord; 2] = [x + m.x_offset, x + m.distance - m.x_offset];
                // FillHoneycomb.cpp:57
                for _i in 0..2usize {
                    // FillHoneycomb.cpp:58 — std::reverse(p.points.begin(), p.points.end()); // turn first half upside down
                    p.points.reverse();
                    // FillHoneycomb.cpp:59 — for (coord_t y = bounding_box.min(1); y <= bounding_box.max(1); y += m.y_short + m.hex_side + m.y_short + m.hex_side)
                    let mut y: Coord = bounding_box.min.y;
                    while y <= bounding_box.max.y {
                        // FillHoneycomb.cpp:60 — p.points.push_back(Point(ax[1], y + m.y_offset));
                        p.points.push(Point::new(ax[1], y + m.y_offset));
                        // FillHoneycomb.cpp:61 — p.points.push_back(Point(ax[0], y + m.y_short - m.y_offset));
                        p.points.push(Point::new(ax[0], y + m.y_short - m.y_offset));
                        // FillHoneycomb.cpp:62 — p.points.push_back(Point(ax[0], y + m.y_short + m.hex_side + m.y_offset));
                        p.points
                            .push(Point::new(ax[0], y + m.y_short + m.hex_side + m.y_offset));
                        // FillHoneycomb.cpp:63 — p.points.push_back(Point(ax[1], y + m.y_short + m.hex_side + m.y_short - m.y_offset));
                        p.points.push(Point::new(
                            ax[1],
                            y + m.y_short + m.hex_side + m.y_short - m.y_offset,
                        ));
                        // FillHoneycomb.cpp:64 — p.points.push_back(Point(ax[1], y + m.y_short + m.hex_side + m.y_short + m.hex_side + m.y_offset));
                        p.points.push(Point::new(
                            ax[1],
                            y + m.y_short + m.hex_side + m.y_short + m.hex_side + m.y_offset,
                        ));
                        // FillHoneycomb.cpp:59 — y += m.y_short + m.hex_side + m.y_short + m.hex_side
                        y += m.y_short + m.hex_side + m.y_short + m.hex_side;
                    }
                    // FillHoneycomb.cpp:66 — ax[0] = ax[0] + m.distance;
                    ax[0] += m.distance;
                    // FillHoneycomb.cpp:67 — ax[1] = ax[1] + m.distance;
                    ax[1] += m.distance;
                    // FillHoneycomb.cpp:68 — std::swap(ax[0], ax[1]); // draw symmetrical pattern
                    ax.swap(0, 1);
                    // FillHoneycomb.cpp:69 — x += m.distance;
                    x += m.distance;
                }
                // FillHoneycomb.cpp:71 — p.rotate(-direction.first, m.hex_center);
                p.rotate_around(-(direction.0 as CoordF), m.hex_center);
                // FillHoneycomb.cpp:72 — p.simplify(5 * spacing); // simplify to 5x line width
                p.simplify(5.0 * self.spacing);
                // FillHoneycomb.cpp:73 — all_polylines.push_back(p);
                all_polylines.push(p);
            }
        }
        // FillHoneycomb.cpp:76 — Apply multiline offset if needed
        // FillHoneycomb.cpp:77 — multiline_fill(all_polylines, params, 1.1 * spacing);
        multiline_fill(&mut all_polylines, params, (1.1 * self.spacing) as f32);

        // FillHoneycomb.cpp:79 — all_polylines = intersection_pl(std::move(all_polylines), expolygon);
        let all_polylines: Vec<Polyline> =
            intersection_pl(&all_polylines, std::slice::from_ref(&expolygon));
        // FillHoneycomb.cpp:80 — if (params.dont_connect() || all_polylines.size() <= 1)
        if params.dont_connect() || all_polylines.len() <= 1 {
            // FillHoneycomb.cpp:81 — append(polylines_out, chain_polylines(std::move(all_polylines)));
            append(polylines_out, chain_polylines(all_polylines, None));
        } else {
            // FillHoneycomb.cpp:83 — connect_infill(std::move(all_polylines), expolygon, polylines_out, this->spacing, params);
            connect_infill_expolygon(all_polylines, &expolygon, self.spacing, params, polylines_out);
        }
    }
}

// FillHoneycomb.cpp:86 — } // namespace Slic3r

/// `append(dst, src)` — Slic3r helper that moves all elements of `src` onto the
/// end of `dst`. Used at FillHoneycomb.cpp:81.
#[inline]
fn append(dst: &mut Vec<Polyline>, mut src: Vec<Polyline>) {
    if dst.is_empty() {
        *dst = src;
    } else {
        dst.append(&mut src);
    }
}
