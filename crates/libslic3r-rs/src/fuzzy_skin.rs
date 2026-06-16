//! Fuzzy skin: adds random texture to perimeters.
//!
//! Faithful 1:1 port of `src/libslic3r/FuzzySkin.cpp` (+ `FuzzySkin.hpp`) from
//! BambuStudio. Functions appear in the same order, with the same names
//! (snake_case), signatures, control flow, constants and rounding as the C++.
//!
//! coord_t -> i64, coordf_t -> f64.
//!
//! NATIVE-DEP NOTE: the procedural noise modules (Perlin / Billow / RidgedMulti
//! / Voronoi) come from the bundled libnoise (`noise.h`), a native C++ library
//! that is not available in this crate and is not wasm-safe. Only the
//! `Classic` / `UniformNoise` path is portable, so `get_noise_module` returns
//! the `UniformNoise` for every `NoiseType`. The non-Classic branches are
//! BLOCKED on a Rust noise backend; their dispatch structure is preserved.

use crate::arachne::utils::extrusion_junction::ExtrusionJunction;
use crate::arachne::utils::extrusion_line::ExtrusionLine;
use crate::geometry::{Point, PointF, Polygon, Polyline};
use crate::region_config::{FuzzySkinDisplacementMode, FuzzySkinType, NoiseType, PrintRegionConfig};
use crate::{unscale, Coord, CoordF};
use std::cell::RefCell;

// FuzzySkin.cpp:18
// Produces a random value between 0 and 1. Thread-safe.
//
// NATIVE-DEP NOTE: C++ uses `std::mt19937` seeded from `std::random_device`.
// That exact bit-stream is not reproducible from Rust's std, so the RNG is a
// divergence by construction (fuzzy skin is intentionally random, so this does
// not change G-code structure, only the random offsets). A thread-local
// generator is used to match the C++ `thread_local` semantics.
thread_local! {
    // FuzzySkin.cpp:21-23
    static FUZZY_RNG: RefCell<Mt19937Like> = RefCell::new(Mt19937Like::new());
}

/// Minimal thread-local PRNG standing in for `std::mt19937` +
/// `std::uniform_real_distribution<double>(0.0, 1.0)`. See the NATIVE-DEP NOTE
/// on `random_value` for why an exact match is not possible.
struct Mt19937Like {
    state: u64,
}

impl Mt19937Like {
    fn new() -> Self {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        use std::thread;
        use std::time::{SystemTime, UNIX_EPOCH};

        // FuzzySkin.cpp:22 — seed from a hash of the thread id (mirrors the C++
        // fallback `std::hash<std::thread::id>()(std::this_thread::get_id())`),
        // combined with time for entropy.
        let mut hasher = DefaultHasher::new();
        thread::current().id().hash(&mut hasher);
        let thread_hash = hasher.finish();
        let time_seed = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0x5DEECE66D);
        let seed = thread_hash ^ time_seed;
        Self {
            state: if seed == 0 { 0x5DEECE66D } else { seed },
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    // uniform_real_distribution<double>(0.0, 1.0): value in [0, 1).
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / ((1u64 << 53) as f64)
    }
}

// FuzzySkin.cpp:19-25
// Produces a random value between 0 and 1. Thread-safe.
fn random_value() -> f64 {
    FUZZY_RNG.with(|rng| rng.borrow_mut().next_f64())
}

// FuzzySkin.cpp:27-33
// Classic uniform random noise for fuzzy skin (backward compatible)
//
// `noise::module::Module` is a libnoise type; we model only the single method
// that FuzzySkin uses, `GetValue(x, y, z) -> f64`.
trait Module {
    // FuzzySkin.cpp:31
    #[allow(dead_code)]
    fn get_source_module_count(&self) -> i32 {
        0
    }
    // noise::module::Module::GetValue
    fn get_value(&self, x: f64, y: f64, z: f64) -> f64;
}

// FuzzySkin.cpp:28-33
struct UniformNoise;

impl Module for UniformNoise {
    // FuzzySkin.cpp:31
    fn get_source_module_count(&self) -> i32 {
        0
    }
    // FuzzySkin.cpp:32
    fn get_value(&self, _x: f64, _y: f64, _z: f64) -> f64 {
        random_value() * 2. - 1.
    }
}

// FuzzySkin.cpp:35-66
fn get_noise_module(cfg: &PrintRegionConfig) -> Box<dyn Module> {
    // FuzzySkin.cpp:37
    let type_ = cfg.fuzzy_skin_noise_type;
    // FuzzySkin.cpp:38
    let _scale = (0.01_f64).max(cfg.fuzzy_skin_scale);
    // FuzzySkin.cpp:39-64
    //
    // NATIVE-DEP NOTE: Perlin / Billow / RidgedMulti / Voronoi require libnoise
    // (`noise::module::*`), which is unavailable and wasm-unsafe. Their dispatch
    // structure is preserved but every branch falls back to `UniformNoise`.
    match type_ {
        // FuzzySkin.cpp:39-45 — Perlin (BLOCKED: needs libnoise)
        NoiseType::Perlin => Box::new(UniformNoise),
        // FuzzySkin.cpp:46-52 — Billow (BLOCKED: needs libnoise)
        NoiseType::Billow => Box::new(UniformNoise),
        // FuzzySkin.cpp:53-58 — RidgedMulti (BLOCKED: needs libnoise)
        NoiseType::RidgedMulti => Box::new(UniformNoise),
        // FuzzySkin.cpp:59-64 — Voronoi (BLOCKED: needs libnoise)
        NoiseType::Voronoi => Box::new(UniformNoise),
        // FuzzySkin.cpp:65
        NoiseType::Classic => Box::new(UniformNoise),
    }
}

/// Eigen `Vec2d::cast<coord_t>()` — truncation toward zero (C++ static_cast).
#[inline]
fn cast_coord(v: PointF) -> Point {
    Point::new(v.x as Coord, v.y as Coord)
}

// FuzzySkin.cpp:68-103
pub fn fuzzy_polyline(poly: &mut Vec<Point>, closed: bool, slice_z: CoordF, config: &PrintRegionConfig) {
    // FuzzySkin.cpp:70
    let thickness = scaled_f(config.fuzzy_skin_thickness);
    // FuzzySkin.cpp:71
    let point_distance = scaled_f(config.fuzzy_skin_point_distance);
    // FuzzySkin.cpp:72
    let min_dist_between_points = point_distance * 3. / 4.;
    // FuzzySkin.cpp:73
    let range_random_point_dist = point_distance / 2.;
    // FuzzySkin.cpp:74
    let mut dist_left_over = random_value() * (min_dist_between_points / 2.);

    // FuzzySkin.cpp:76
    let noise = get_noise_module(config);

    // FuzzySkin.cpp:78-79
    let mut out: Vec<Point> = Vec::with_capacity(poly.len());
    // FuzzySkin.cpp:80
    // Point *p0 = closed ? &poly.back() : &poly.front();
    let mut p0: Point = if closed { *poly.last().unwrap() } else { poly[0] };
    // FuzzySkin.cpp:81
    // for (it_pt1 = closed ? begin() : next(begin()); it_pt1 != end(); ++it_pt1)
    let start = if closed { 0 } else { 1 };
    for i in start..poly.len() {
        // FuzzySkin.cpp:82
        let p1 = poly[i];
        // FuzzySkin.cpp:83
        let p0p1 = PointF::new((p1.x - p0.x) as f64, (p1.y - p0.y) as f64);
        // FuzzySkin.cpp:84
        let p0p1_size = p0p1.length();
        // FuzzySkin.cpp:85
        let mut p0pa_dist = dist_left_over;
        // FuzzySkin.cpp:86
        while p0pa_dist < p0p1_size {
            // FuzzySkin.cpp:87
            let pa = p0 + cast_coord(PointF::new(p0p1.x * (p0pa_dist / p0p1_size), p0p1.y * (p0pa_dist / p0p1_size)));
            // FuzzySkin.cpp:88
            let r = noise.get_value(unscale(pa.x), unscale(pa.y), slice_z) * thickness;
            // FuzzySkin.cpp:89
            let perp_n = p0p1.perp().normalize();
            out.push(pa + cast_coord(PointF::new(perp_n.x * r, perp_n.y * r)));
            // FuzzySkin.cpp:86 (loop increment)
            p0pa_dist += min_dist_between_points + random_value() * range_random_point_dist;
        }
        // FuzzySkin.cpp:91
        dist_left_over = p0pa_dist - p0p1_size;
        // FuzzySkin.cpp:92
        p0 = p1;
    }

    // FuzzySkin.cpp:95-100
    while out.len() < 3 {
        // FuzzySkin.cpp:96 — point_idx is recomputed each iteration, so the
        // C++ `--point_idx` at FuzzySkin.cpp:99 has no observable effect.
        let point_idx = poly.len() - 2;
        // FuzzySkin.cpp:97
        out.push(poly[point_idx]);
        // FuzzySkin.cpp:98
        if point_idx == 0 {
            break;
        }
        // FuzzySkin.cpp:99 (dead `--point_idx`)
    }
    // FuzzySkin.cpp:101-102
    if out.len() >= 3 {
        *poly = out;
    }
}

// FuzzySkin.cpp:105-108
pub fn fuzzy_polygon(polygon: &mut Polygon, slice_z: CoordF, config: &PrintRegionConfig) {
    // FuzzySkin.cpp:107
    fuzzy_polyline(&mut polygon.points, true, slice_z, config);
}

// FuzzySkin.cpp:110-173
pub fn fuzzy_extrusion_line(ext_lines: &mut ExtrusionLine, slice_z: CoordF, config: &PrintRegionConfig) {
    // FuzzySkin.cpp:112
    let thickness = scaled_f(config.fuzzy_skin_thickness);
    // FuzzySkin.cpp:113
    let point_distance = scaled_f(config.fuzzy_skin_point_distance);
    // FuzzySkin.cpp:114
    let min_dist_between_points = point_distance * 3. / 4.;
    // FuzzySkin.cpp:115
    let range_random_point_dist = point_distance / 2.;
    // FuzzySkin.cpp:116
    let min_extrusion_width = scaled_f(0.01); // minimum line width (mm) for Extrusion/Combined
    // FuzzySkin.cpp:117
    let mut dist_left_over = random_value() * (min_dist_between_points / 2.);

    // FuzzySkin.cpp:119
    let noise = get_noise_module(config);
    // FuzzySkin.cpp:120
    let mode = config.fuzzy_skin_displacement_mode;

    // FuzzySkin.cpp:122
    // Arachne::ExtrusionJunction *p0 = &ext_lines.front();
    let mut p0: ExtrusionJunction = ext_lines.junctions[0];
    // FuzzySkin.cpp:123-124
    let mut out: Vec<ExtrusionJunction> = Vec::with_capacity(ext_lines.junctions.len());
    // FuzzySkin.cpp:125
    for idx in 0..ext_lines.junctions.len() {
        let p1 = ext_lines.junctions[idx];
        // FuzzySkin.cpp:126
        if p0.p == p1.p {
            // FuzzySkin.cpp:127
            out.push(ExtrusionJunction::new(p1.p, p1.w, p1.perimeter_index));
            // FuzzySkin.cpp:128
            continue;
        }
        // FuzzySkin.cpp:130
        let p0p1 = PointF::new((p1.p.x - p0.p.x) as f64, (p1.p.y - p0.p.y) as f64);
        // FuzzySkin.cpp:131
        let p0p1_size = p0p1.length();
        // FuzzySkin.cpp:132
        let mut p0pa_dist = dist_left_over;
        // FuzzySkin.cpp:133
        while p0pa_dist < p0p1_size {
            // FuzzySkin.cpp:134
            let pa = p0.p
                + cast_coord(PointF::new(p0p1.x * (p0pa_dist / p0p1_size), p0p1.y * (p0pa_dist / p0p1_size)));
            // FuzzySkin.cpp:135
            let r = noise.get_value(unscale(pa.x), unscale(pa.y), slice_z) * thickness;
            // FuzzySkin.cpp:136
            let perp_n = p0p1.perp().normalize();
            // FuzzySkin.cpp:137
            match mode {
                // FuzzySkin.cpp:138-140
                FuzzySkinDisplacementMode::Displacement => {
                    out.push(ExtrusionJunction::new(
                        pa + cast_coord(PointF::new(perp_n.x * r, perp_n.y * r)),
                        p1.w,
                        p1.perimeter_index,
                    ));
                }
                // FuzzySkin.cpp:141-143
                FuzzySkinDisplacementMode::Extrusion => {
                    out.push(ExtrusionJunction::new(
                        pa,
                        ((p1.w as f64 + r + min_extrusion_width).max(min_extrusion_width)) as Coord,
                        p1.perimeter_index,
                    ));
                }
                // FuzzySkin.cpp:144-148
                FuzzySkinDisplacementMode::Combined => {
                    let rad = (p1.w as f64 + r + min_extrusion_width).max(min_extrusion_width);
                    out.push(ExtrusionJunction::new(
                        pa + cast_coord(PointF::new(
                            perp_n.x * ((rad - p1.w as f64) / 2.),
                            perp_n.y * ((rad - p1.w as f64) / 2.),
                        )),
                        rad as Coord,
                        p1.perimeter_index,
                    ));
                }
            }
            // FuzzySkin.cpp:133 (loop increment)
            p0pa_dist += min_dist_between_points + random_value() * range_random_point_dist;
        }
        // FuzzySkin.cpp:151
        dist_left_over = p0pa_dist - p0p1_size;
        // FuzzySkin.cpp:152
        p0 = p1;
    }

    // FuzzySkin.cpp:155-163
    while out.len() < 3 {
        // FuzzySkin.cpp:156 — point_idx is recomputed each iteration, so the
        // C++ `--point_idx` at FuzzySkin.cpp:162 has no observable effect.
        let point_idx = ext_lines.junctions.len() - 2;
        // FuzzySkin.cpp:157
        let j = ext_lines.junctions[point_idx];
        out.push(ExtrusionJunction::new(j.p, j.w, j.perimeter_index));
        // FuzzySkin.cpp:158-160
        if point_idx == 0 {
            break;
        }
        // FuzzySkin.cpp:162 (dead `--point_idx`)
    }

    // FuzzySkin.cpp:165-168
    if ext_lines.junctions.last().unwrap().p == ext_lines.junctions.first().unwrap().p {
        // FuzzySkin.cpp:166
        let back = *out.last().unwrap();
        out.first_mut().unwrap().p = back.p;
        // FuzzySkin.cpp:167
        out.first_mut().unwrap().w = back.w;
    }

    // FuzzySkin.cpp:170-172
    if out.len() >= 3 {
        ext_lines.junctions = out;
    }
}

// FuzzySkin.cpp:175-188
pub fn should_fuzzify(
    config: &PrintRegionConfig,
    layer_idx: usize,
    perimeter_idx: usize,
    is_contour: bool,
) -> bool {
    // FuzzySkin.cpp:177
    let fuzzy_skin_type = config.fuzzy_skin_type;

    // FuzzySkin.cpp:179-180
    if fuzzy_skin_type == FuzzySkinType::None || fuzzy_skin_type == FuzzySkinType::DisabledFuzzy {
        return false;
    }
    // FuzzySkin.cpp:181-182
    // C++: layer_idx is size_t, so `layer_idx <= 0` is equivalent to `layer_idx == 0`.
    if !config.fuzzy_skin_first_layer && layer_idx == 0 {
        return false;
    }

    // FuzzySkin.cpp:184
    let fuzzify_contours = perimeter_idx == 0 || fuzzy_skin_type == FuzzySkinType::AllWalls;
    // FuzzySkin.cpp:185
    let fuzzify_holes = fuzzify_contours
        && (fuzzy_skin_type == FuzzySkinType::All || fuzzy_skin_type == FuzzySkinType::AllWalls);

    // FuzzySkin.cpp:187
    if is_contour {
        fuzzify_contours
    } else {
        fuzzify_holes
    }
}

// FuzzySkin.cpp:190-234
pub fn apply_fuzzy_skin_polygon(
    polygon: &Polygon,
    base_config: &PrintRegionConfig,
    perimeter_regions: &[PerimeterRegion],
    layer_idx: usize,
    perimeter_idx: usize,
    is_contour: bool,
    slice_z: CoordF,
) -> Polygon {
    // FuzzySkin.cpp:194-201
    let apply_fuzzy_skin_on_polygon = |polygon: &Polygon, config: &PrintRegionConfig| -> Polygon {
        // FuzzySkin.cpp:195
        if should_fuzzify(config, layer_idx, perimeter_idx, is_contour) {
            // FuzzySkin.cpp:196
            let mut fuzzified_polygon = polygon.clone();
            // FuzzySkin.cpp:197
            fuzzy_polygon(&mut fuzzified_polygon, slice_z, config);
            // FuzzySkin.cpp:198
            return fuzzified_polygon;
        }
        // FuzzySkin.cpp:200
        polygon.clone()
    };

    // FuzzySkin.cpp:203-204
    if perimeter_regions.is_empty() {
        return apply_fuzzy_skin_on_polygon(polygon, base_config);
    }

    // FuzzySkin.cpp:206
    let mut segments = polygon_segmentation(polygon, base_config, perimeter_regions);
    // FuzzySkin.cpp:207-208
    if segments.len() == 1 {
        return apply_fuzzy_skin_on_polygon(polygon, &segments[0].config);
    }

    // FuzzySkin.cpp:210
    let mut fuzzified_polygon = Polygon::new();
    // FuzzySkin.cpp:211
    for segment in &mut segments {
        // FuzzySkin.cpp:212
        let config = &segment.config;
        // FuzzySkin.cpp:213-214
        if should_fuzzify(config, layer_idx, perimeter_idx, is_contour) {
            fuzzy_polyline(&mut segment.polyline.points, false, slice_z, config);
        }

        // FuzzySkin.cpp:216
        debug_assert!(!segment.polyline.is_empty());
        // FuzzySkin.cpp:217-222
        if segment.polyline.is_empty() {
            continue;
        } else if !fuzzified_polygon.is_empty()
            && *fuzzified_polygon.points.last().unwrap() == segment.polyline.points[0]
        {
            // Remove the last point to avoid duplicate points.
            fuzzified_polygon.points.pop();
        }

        // FuzzySkin.cpp:224
        append(&mut fuzzified_polygon.points, std::mem::take(&mut segment.polyline.points));
    }

    // FuzzySkin.cpp:227
    debug_assert!(!fuzzified_polygon.is_empty());
    // FuzzySkin.cpp:228-231
    if fuzzified_polygon.points.first() == fuzzified_polygon.points.last() {
        // Remove the last point to avoid duplicity between the first and the last point.
        fuzzified_polygon.points.pop();
    }

    // FuzzySkin.cpp:233
    fuzzified_polygon
}

// FuzzySkin.cpp:236-272
pub fn apply_fuzzy_skin_extrusion(
    extrusion: &ExtrusionLine,
    base_config: &PrintRegionConfig,
    perimeter_regions: &[PerimeterRegion],
    layer_idx: usize,
    perimeter_idx: usize,
    is_contour: bool,
    slice_z: CoordF,
) -> ExtrusionLine {
    // FuzzySkin.cpp:241-248
    if perimeter_regions.is_empty() {
        // FuzzySkin.cpp:242
        if should_fuzzify(base_config, layer_idx, perimeter_idx, is_contour) {
            // FuzzySkin.cpp:243
            let mut fuzzified_extrusion = extrusion.clone();
            // FuzzySkin.cpp:244
            fuzzy_extrusion_line(&mut fuzzified_extrusion, slice_z, base_config);
            // FuzzySkin.cpp:245
            return fuzzified_extrusion;
        }
        // FuzzySkin.cpp:247
        return extrusion.clone();
    }

    // FuzzySkin.cpp:250
    let mut segments = extrusion_segmentation(extrusion, base_config, perimeter_regions);
    // FuzzySkin.cpp:251
    let mut fuzzified_extrusion =
        ExtrusionLine::with_closed(extrusion.inset_idx, extrusion.is_odd, extrusion.is_closed);

    // FuzzySkin.cpp:253
    for segment in &mut segments {
        // FuzzySkin.cpp:254
        let config = &segment.config;
        // FuzzySkin.cpp:255-256
        if should_fuzzify(config, layer_idx, perimeter_idx, is_contour) {
            fuzzy_extrusion_line(&mut segment.extrusion, slice_z, config);
        }

        // FuzzySkin.cpp:258
        debug_assert!(!segment.extrusion.junctions.is_empty());
        // FuzzySkin.cpp:259-264
        if segment.extrusion.junctions.is_empty() {
            continue;
        } else if !fuzzified_extrusion.junctions.is_empty()
            && fuzzified_extrusion.junctions.last().unwrap().p
                == segment.extrusion.junctions[0].p
        {
            // Remove the last point to avoid duplicate points (We don't care if the width of both points is different.).
            fuzzified_extrusion.junctions.pop();
        }

        // FuzzySkin.cpp:266
        append(
            &mut fuzzified_extrusion.junctions,
            std::mem::take(&mut segment.extrusion.junctions),
        );
    }

    // FuzzySkin.cpp:269
    debug_assert!(!fuzzified_extrusion.junctions.is_empty());

    // FuzzySkin.cpp:271
    fuzzified_extrusion
}

/// `scaled<double>(v)` — `v / SCALING_FACTOR` returned as f64 (no rounding).
/// Point.hpp:527-530. In this crate `SCALING_FACTOR` is stored as its inverse
/// (`100_000.0`), so `scaled<double>(v) == v * SCALING_FACTOR_rust`.
#[inline]
fn scaled_f(v: CoordF) -> CoordF {
    v * crate::SCALING_FACTOR
}

/// `Slic3r::append(dst, src)` — move-append `src` onto `dst`.
#[inline]
fn append<T>(dst: &mut Vec<T>, mut src: Vec<T>) {
    dst.append(&mut src);
}

// ------------------------------------------------------------------------
// PerimeterRegion / segmentation support
//
// PerimeterGenerator.hpp:15-30 — `struct PerimeterRegion { const PrintRegion*
// region; ExPolygons expolygons; BoundingBox bbox; }` and
// `using PerimeterRegions = std::vector<PerimeterRegion>;`.
//
// The full PerimeterRegion (with PrintRegion pointer + ExPolygons + BoundingBox)
// and the Clipper-Z based segmentation in
// Algorithm/LineSegmentation/LineSegmentation.cpp are not yet fully ported
// (the Rust `line_segmentation` module returns whole-line single segments).
// We model the minimal config-bearing view FuzzySkin actually consumes: each
// PerimeterRegion carries the PrintRegionConfig used to fuzzify its segment.
// ------------------------------------------------------------------------

/// PerimeterGenerator.hpp:15-28 (config-bearing view).
#[derive(Debug, Clone)]
pub struct PerimeterRegion {
    /// The region config used when fuzzifying segments in this region.
    /// PerimeterGenerator.hpp:17 (`const PrintRegion *region;` -> its config)
    pub config: PrintRegionConfig,
}

/// A polyline segment tagged with the config of the region it falls in.
/// LineSegmentation.hpp:32-38 (PolylineRegionSegment).
struct PolylineRegionSegment {
    polyline: Polyline,
    config: PrintRegionConfig,
}

/// An extrusion segment tagged with the config of the region it falls in.
/// LineSegmentation.hpp:46-52 (ExtrusionRegionSegment).
struct ExtrusionRegionSegment {
    extrusion: ExtrusionLine,
    config: PrintRegionConfig,
}

/// LineSegmentation::polygon_segmentation.
///
/// BLOCKED: the real Clipper-Z segmentation (LineSegmentation.cpp) is not yet
/// ported; the Rust `line_segmentation` module returns the whole subject as a
/// single segment. We mirror that: a single segment carrying `base_config`.
fn polygon_segmentation(
    polygon: &Polygon,
    base_config: &PrintRegionConfig,
    _perimeter_regions: &[PerimeterRegion],
) -> Vec<PolylineRegionSegment> {
    vec![PolylineRegionSegment {
        polyline: polygon.to_polyline(),
        config: base_config.clone(),
    }]
}

/// LineSegmentation::extrusion_segmentation.
///
/// BLOCKED: see `polygon_segmentation`.
fn extrusion_segmentation(
    extrusion: &ExtrusionLine,
    base_config: &PrintRegionConfig,
    _perimeter_regions: &[PerimeterRegion],
) -> Vec<ExtrusionRegionSegment> {
    vec![ExtrusionRegionSegment {
        extrusion: extrusion.clone(),
        config: base_config.clone(),
    }]
}

// ========================================================================
// Rust-side compatibility adapter (NOT part of FuzzySkin.cpp)
//
// `perimeter_generator.rs` drives fuzzy skin through a small `FuzzySkinConfig`
// + `FuzzySkinMode` (None/External/All). These adapters bridge that caller to
// the faithful functions above without changing the C++ logic.
// ========================================================================

use crate::region_config::FuzzySkinMode;

/// Adapter config used by `perimeter_generator.rs`.
#[derive(Debug, Clone)]
pub struct FuzzySkinConfig {
    /// Maximum thickness/displacement from original surface (mm).
    pub thickness: CoordF,
    /// Target distance between points along edges (mm).
    pub point_distance: CoordF,
    /// Which perimeters to fuzzify (None/External/All).
    pub mode: FuzzySkinMode,
}

impl FuzzySkinConfig {
    /// Build a faithful `PrintRegionConfig` view from this adapter config.
    fn to_region_config(&self) -> PrintRegionConfig {
        let mut cfg = PrintRegionConfig::default();
        cfg.fuzzy_skin_thickness = self.thickness;
        cfg.fuzzy_skin_point_distance = self.point_distance;
        cfg.fuzzy_skin_type = match self.mode {
            FuzzySkinMode::None => FuzzySkinType::None,
            FuzzySkinMode::External => FuzzySkinType::External,
            FuzzySkinMode::All => FuzzySkinType::All,
        };
        cfg.fuzzy_skin_mode = self.mode;
        // perimeter_generator applies fuzzy after seam/loop ordering on layer 1+
        // and relies on layer_idx == 0 being skipped, so first-layer fuzz stays
        // off to match the historical adapter behaviour.
        cfg.fuzzy_skin_first_layer = false;
        cfg
    }
}

/// Adapter: fuzzify a polygon in place (slice_z = 0) using the C++ port.
pub fn fuzzy_polygon_params(polygon: &mut Polygon, thickness_mm: CoordF, point_distance_mm: CoordF) {
    let mut cfg = PrintRegionConfig::default();
    cfg.fuzzy_skin_thickness = thickness_mm;
    cfg.fuzzy_skin_point_distance = point_distance_mm;
    fuzzy_polygon(polygon, 0.0, &cfg);
}

/// Adapter: fuzzify an extrusion line in place (slice_z = 0) using the C++ port.
pub fn fuzzy_extrusion_line_params(
    extrusion: &mut ExtrusionLine,
    thickness_mm: CoordF,
    point_distance_mm: CoordF,
) {
    let mut cfg = PrintRegionConfig::default();
    cfg.fuzzy_skin_thickness = thickness_mm;
    cfg.fuzzy_skin_point_distance = point_distance_mm;
    fuzzy_extrusion_line(extrusion, 0.0, &cfg);
}

/// Adapter used by `perimeter_generator.rs` for polygon fuzzification.
pub fn apply_fuzzy_skin_polygon_adapter(
    polygon: &Polygon,
    config: &FuzzySkinConfig,
    layer_idx: usize,
    perimeter_idx: usize,
    is_contour: bool,
) -> Polygon {
    let region_config = config.to_region_config();
    apply_fuzzy_skin_polygon(
        polygon,
        &region_config,
        &[],
        layer_idx,
        perimeter_idx,
        is_contour,
        0.0,
    )
}

/// Adapter used by `perimeter_generator.rs` for extrusion fuzzification.
pub fn apply_fuzzy_skin_extrusion_adapter(
    extrusion: &ExtrusionLine,
    config: &FuzzySkinConfig,
    layer_idx: usize,
    perimeter_idx: usize,
    is_contour: bool,
) -> ExtrusionLine {
    let region_config = config.to_region_config();
    apply_fuzzy_skin_extrusion(
        extrusion,
        &region_config,
        &[],
        layer_idx,
        perimeter_idx,
        is_contour,
        0.0,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scale;

    fn make_square(size_mm: CoordF) -> Polygon {
        let s = scale(size_mm);
        Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(s, 0),
            Point::new(s, s),
            Point::new(0, s),
        ])
    }

    fn external_config() -> PrintRegionConfig {
        let mut cfg = PrintRegionConfig::default();
        cfg.fuzzy_skin_thickness = 0.3;
        cfg.fuzzy_skin_point_distance = 0.8;
        cfg.fuzzy_skin_type = FuzzySkinType::External;
        cfg
    }

    #[test]
    fn test_fuzzy_polygon_adds_points() {
        let original = make_square(10.0);
        let original_count = original.points().len();
        let mut fuzzified = original.clone();
        let cfg = external_config();
        fuzzy_polygon(&mut fuzzified, 0.0, &cfg);
        assert!(fuzzified.points().len() > original_count);
    }

    #[test]
    fn test_should_fuzzify_first_layer() {
        let cfg = external_config();
        // first layer skipped unless fuzzy_skin_first_layer
        assert!(!should_fuzzify(&cfg, 0, 0, true));
        let mut allc = external_config();
        allc.fuzzy_skin_type = FuzzySkinType::All;
        assert!(!should_fuzzify(&allc, 0, 0, true));
    }

    #[test]
    fn test_should_fuzzify_external() {
        let cfg = external_config();
        assert!(should_fuzzify(&cfg, 1, 0, true)); // outer contour
        assert!(!should_fuzzify(&cfg, 1, 0, false)); // hole
        assert!(!should_fuzzify(&cfg, 1, 1, true)); // inner perimeter
    }

    #[test]
    fn test_should_fuzzify_all_walls() {
        let mut cfg = external_config();
        cfg.fuzzy_skin_type = FuzzySkinType::AllWalls;
        assert!(should_fuzzify(&cfg, 1, 0, true));
        assert!(should_fuzzify(&cfg, 1, 1, true));
        assert!(should_fuzzify(&cfg, 1, 1, false)); // holes too
    }

    #[test]
    fn test_should_fuzzify_none_and_disabled() {
        let mut none = external_config();
        none.fuzzy_skin_type = FuzzySkinType::None;
        assert!(!should_fuzzify(&none, 1, 0, true));
        let mut disabled = external_config();
        disabled.fuzzy_skin_type = FuzzySkinType::DisabledFuzzy;
        assert!(!should_fuzzify(&disabled, 1, 0, true));
    }

    #[test]
    fn test_apply_fuzzy_skin_polygon_no_regions() {
        let original = make_square(10.0);
        let cfg = external_config();
        // layer 0: not fuzzified
        let r = apply_fuzzy_skin_polygon(&original, &cfg, &[], 0, 0, true, 0.0);
        assert_eq!(r.points().len(), original.points().len());
        // layer 1 outer contour: fuzzified
        let r = apply_fuzzy_skin_polygon(&original, &cfg, &[], 1, 0, true, 0.0);
        assert!(r.points().len() > original.points().len());
    }

    #[test]
    fn test_fuzzy_extrusion_line_displacement() {
        let mut original = ExtrusionLine::with_closed(0, false, false);
        original.junctions = vec![
            ExtrusionJunction::new(Point::new(0, 0), scale(0.4), 0),
            ExtrusionJunction::new(Point::new(scale(10.0), 0), scale(0.4), 0),
        ];
        let mut fuzzified = original.clone();
        let cfg = external_config();
        fuzzy_extrusion_line(&mut fuzzified, 0.0, &cfg);
        assert!(fuzzified.junctions.len() > original.junctions.len());
    }

    #[test]
    fn test_random_value_range() {
        for _ in 0..100 {
            let v = random_value();
            assert!((0.0..1.0).contains(&v));
        }
    }

    #[test]
    fn test_uniform_noise_range() {
        let m = UniformNoise;
        for _ in 0..100 {
            let v = m.get_value(0.0, 0.0, 0.0);
            assert!((-1.0..1.0).contains(&v));
        }
    }
}
