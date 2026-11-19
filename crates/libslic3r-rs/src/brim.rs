//! Faithful 1:1 port of BambuStudio `src/libslic3r/Brim.cpp`.
//!
//! C++ source: libslic3r/bambustudio/references/BambuStudio/src/libslic3r/Brim.cpp
//! Header:     libslic3r/bambustudio/references/BambuStudio/src/libslic3r/Brim.hpp
//!
//! This file produces brim lines around objects that have brim enabled, plus
//! the auto-brim / by-object brim variants.
//!
//! PORT STATUS: partial.
//!
//! The overwhelming majority of `Brim.cpp` is orchestration over the
//! `Print` / `PrintObject` / `Layer` / `LayerRegion` / `Flow` / `ModelVolume`
//! / `ExtrusionEntityCollection` APIs together with `ClipperLib_Z` (Z-tagged
//! polytree unions), `EdgeGrid::Grid`, and `tbb::parallel_for`. The
//! config-hierarchy wiring (2026-06-12: `PrintObject::config()/print()`,
//! `Layer::object()`, `LayerRegion::region()`) unblocked the bottom-layer /
//! brim-width helpers below, but the following accessors used by the
//! remaining brim code are still missing from the Rust `Print`/`Model`:
//!
//!   Print::      brim_flow, skirt_flow, skirt_first_layer_height, extruders,
//!                has_support_material, has_wipe_tower, get_object,
//!                print_object_ids, get_plate_origin, get_filament_maps,
//!                get_extruder_printable_polygons, get_fake_wipe_tower
//!   PrintObject: instances (PrintInstance::shift_without_plate_offset),
//!                has_brim, has_raft, has_support_material, center_offset,
//!                model_object, firstLayerObjGroups, get_shared_object,
//!                firstLayerObjectBrimBoundingBox
//!   Model::      extruderParamsMap, getThermalLength, findMaxSpeed,
//!                getBedPolygon
//!   Geometry::   segments_intersect (used by connect_brim_lines' EdgeGrid
//!                visitor)
//!
//! Consequently the following symbols are BLOCKED until those dependencies are
//! threaded through (they are NOT stubbed here — porting fake versions would
//! corrupt G-code parity):
//!
//!   append_and_translate (x4 overloads)        — PrintInstance::shift_without_plate_offset
//!   get_top_level_objects_with_brim             — ClipperLib_Z polytree union + ObjectID + instances
//!   top_level_outer_brim_islands                — PrintObject support_layers/instances
//!   top_level_outer_brim_area (x2)              — Print::brim_flow (unported), ObjectID maps
//!   inner_brim_area (x2)                        — same as above + get_bed_shape
//!   getTemperatureFromExtruder                  — per-extruder ConfigOptionInts bed temps +
//!                                                 get_bed_temp_1st_layer_key (Rust PrintConfig
//!                                                 models bed temps as scalars, not vectors)
//!   getadhesionCoeff                            — PrintObject::instances, ModelVolume::extruder_id,
//!                                                 Model::extruderParamsMap
//!   configBrimWidthByVolumes                    — ModelVolume::mesh/transformed_bounding_box
//!   configBrimWidthByVolumeGroups               — Model::getThermalLength
//!   make_brim_ears                              — ModelObject::brim_points, Transformation
//!   outer_inner_brim_area                       — firstLayerObjGroups, get_extruder_printable_polygons
//!   connect_brim_lines                          — EdgeGrid::Grid, Geometry::segments_intersect
//!   make_inner_island_brim (x2)                 — union_pt_chained_outside_in, chain_polylines, tbb
//!   make_inner_brim (x2)                        — inner_brim_area + flow
//!   tryExPolygonOffset                          — Print::brim_flow (unported), ExPolygon::douglas_peucker
//!   makeBrimInfill                              — tbb, chain_polylines, EdgeGrid
//!   make_brim (x2 + auto)                        — full Print pipeline + ClipperLib_Z skirt trimming
//!
//! What IS faithfully ported here:
//!   - max_brim_width(objects)                      Brim.cpp:57
//!   - get_print_object_bottom_layer_expolygons     Brim.cpp:67
//!   - get_print_bottom_layers_expolygons           Brim.cpp:76
//!   - compSecondMoment(Polygon, Vec2d&)            Brim.cpp:630
//!   - struct ExPolyProp                            Brim.cpp:650
//!   - compSecondMoment(ExPolygon, ExPolyProp&)     Brim.cpp:658
//!   - compSecondMoment(ExPolygons, double, double) Brim.cpp:684
//!   - optimize_polylines_by_reversing(Polylines*)  Brim.cpp:1146

use crate::clipper_utils::{closing, OffsetJoinType};
use crate::geometry::{cross2f, ExPolygon, ExPolygons, PointF, Polygon, Polylines};
use crate::libslic3r::EPSILON;
use crate::print::Print;
use crate::print_config::BrimType;
use crate::print_object::PrintObject;

// Brim.cpp:57
// static float max_brim_width(const ConstPrintObjectPtrsAdaptor &objects)
//
// In C++ this iterates the print's objects through the
// `ConstPrintObjectPtrsAdaptor`; the Rust equivalent is the `Print::objects()`
// slice.
pub fn max_brim_width(objects: &[PrintObject]) -> f32 {
    // Brim.cpp:59
    debug_assert!(!objects.is_empty());
    // Brim.cpp:60-63
    objects.iter().fold(0.0_f64, |partial_result, object| {
        partial_result.max(if object.config().brim_type == BrimType::NoBrim {
            0.0
        } else {
            object.config().brim_width
        })
    }) as f32
}

// Brim.cpp:66
// Returns ExPolygons of the bottom layer of the print object after elephant foot compensation.
// Brim.cpp:67
pub fn get_print_object_bottom_layer_expolygons(print_object: &PrintObject) -> ExPolygons {
    // Brim.cpp:69
    let mut ex_polygons: ExPolygons = Vec::new();
    // Brim.cpp:70
    for region in print_object.layers()[0].regions() {
        // Brim.cpp:71
        // C++: Slic3r::append(ex_polygons, closing_ex(region->slices.surfaces, float(SCALED_EPSILON)));
        // `closing_ex(Surfaces, delta)` is `offset2_ex(surfaces, +delta, -delta)`
        // with DefaultJoinType = jtMiter (ClipperUtils.hpp:409-415), i.e. a
        // morphological closing of the surfaces' expolygons. C++ offsets by
        // SCALED_EPSILON = scale_(EPSILON) = 10 scaled units = EPSILON mm; this
        // crate's `closing` wrapper takes its delta in the mm domain (it
        // unscales points before offsetting), so EPSILON is the faithful delta.
        // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib. C++
        // `offset2_ex` is a single ClipperOffset polytree pass at coord_t integer
        // precision; this crate's `closing` is grow-then-shrink via geo-clipper
        // (geo crate, fixed scale 1000) — morphologically equivalent, not
        // byte-identical.
        let surfaces_ex: ExPolygons = region
            .slices
            .surfaces
            .iter()
            .map(|surface| surface.expolygon.clone())
            .collect();
        ex_polygons.extend(closing(&surfaces_ex, EPSILON, OffsetJoinType::Miter));
    }
    // Brim.cpp:72
    ex_polygons
}

// Brim.cpp:75
// Returns ExPolygons of bottom layer for every print object in Print after elephant foot compensation.
// Brim.cpp:76
pub fn get_print_bottom_layers_expolygons(print: &Print) -> Vec<ExPolygons> {
    // Brim.cpp:78-79
    let mut bottom_layers_expolygons: Vec<ExPolygons> = Vec::with_capacity(print.objects().len());
    // Brim.cpp:80-81
    for object in print.objects() {
        bottom_layers_expolygons.push(get_print_object_bottom_layer_expolygons(object));
    }
    // Brim.cpp:83
    bottom_layers_expolygons
}

// Brim.cpp:629
/// Gate `BRIM_SECOND_MOMENT_PIN` (on by default): `compSecondMoment` in the scaled coordinates
/// the C++ computes in (`Point::cast<double>()`), with the contraction of the C++ build, and
/// `Polygon::area()` / `Polygon::centroid()` with the fused `cross2`; checked by
/// tests/brim_native_parity.rs. Off: the earlier arm, which computes in millimetres
/// (`Point::to_f64` unscales) and so returns moments 1e20 times smaller than the C++, unfused.
#[inline]
fn second_moment_pin() -> bool {
    crate::faithful_gate("BRIM_SECOND_MOMENT_PIN")
}

/// Polygon.cpp:49-61 `double Polygon::area(const Points &points)` with Point.hpp:86-88 `cross2`
/// as the C++ build evaluates it (`v1.x() * v2.y() - v1.y() * v2.x()`, the left product fused).
fn pin_area(poly: &Polygon) -> f64 {
    let points = &poly.points;
    let mut a = 0.0f64;
    if points.len() >= 3 {
        let last = points[points.len() - 1];
        let (mut p1x, mut p1y) = (last.x as f64, last.y as f64);
        for p in points {
            let (p2x, p2y) = (p.x as f64, p.y as f64);
            a += p1x.mul_add(p2y, -(p1y * p2x));
            p1x = p2x;
            p1y = p2y;
        }
    }
    0.5 * a
}

/// Polygon.cpp:171-186 `Point Polygon::centroid() const`, then `.cast<double>()`: the centroid
/// rounded to coordinates (`Point(Vec2d)`), as doubles.
fn pin_centroid(poly: &Polygon) -> (f64, f64) {
    let points = &poly.points;
    let mut area_sum = 0.0f64;
    let (mut cx, mut cy) = (0.0f64, 0.0f64);
    if points.len() >= 3 {
        let last = points[points.len() - 1];
        let (mut p1x, mut p1y) = (last.x as f64, last.y as f64);
        for p in points {
            let (p2x, p2y) = (p.x as f64, p.y as f64);
            let a = p1x.mul_add(p2y, -(p1y * p2x));
            area_sum += a;
            // c += (p1 + p2) * a (Eigen packets, unfused)
            cx += (p1x + p2x) * a;
            cy += (p1y + p2y) * a;
            p1x = p2x;
            p1y = p2y;
        }
    }
    // Point(Vec2d(c / (3. * area_sum))): Point.hpp:181 `explicit Point(const Vec2d&)` rounds
    // (`coord_t(lrint(...))`). (`geometry::Polygon::centroid` truncates here.)
    let d = 3.0 * area_sum;
    ((cx / d).round_ties_even() as i32 as f64, (cy / d).round_ties_even() as i32 as f64)
}

// BBS: second moment of area of a polygon
//
// Returns `true` and writes the result into `sm` if the polygon has at least
// three points; otherwise returns `false` and leaves `sm` zeroed.
pub fn comp_second_moment_polygon(poly: &Polygon, sm: &mut PointF) -> bool {
    // Brim.cpp:631-633
    // The C++ takes the polygon by value and conditionally reverses it so the
    // computation runs on a CCW orientation. We mirror that by cloning when the
    // input is clockwise.
    let mut poly_owned;
    let poly: &Polygon = if poly.is_clockwise() {
        poly_owned = poly.clone();
        poly_owned.make_counter_clockwise();
        &poly_owned
    } else {
        poly
    };

    if second_moment_pin() {
        // Brim.cpp:646-656 in the scaled coordinates of the C++ (`cast<double>()`), each scalar
        // expression contracted as clang contracts it.
        *sm = PointF::new(0., 0.);
        if poly.points.len() >= 3 {
            let last = poly.points.last().unwrap();
            let (mut p1x, mut p1y) = (last.x as f64, last.y as f64);
            for p in &poly.points {
                let (p2x, p2y) = (p.x as f64, p.y as f64);
                // Point.hpp:86-88 cross2: `v1.x() * v2.y() - v1.y() * v2.x()`
                let a = p1x.mul_add(p2y, -(p1y * p2x));

                // Brim.cpp:653: Vec2d(...) * a / 12, then `+=` (Eigen packets, unfused)
                let ex = p2y.mul_add(p2y, p1y.mul_add(p1y, p1y * p2y));
                let ey = p2x.mul_add(p2x, p1x.mul_add(p1x, p1x * p2x));
                *sm = PointF::new(sm.x() + ex * a / 12.0, sm.y() + ey * a / 12.0);
                p1x = p2x;
                p1y = p2y;
            }
            return true;
        }
        return false;
    }

    // Brim.cpp:635
    *sm = PointF::new(0., 0.);
    // Brim.cpp:636
    if poly.points.len() >= 3 {
        // Brim.cpp:637
        let mut p1 = poly.points.last().unwrap().to_f64();
        // Brim.cpp:638
        for p in &poly.points {
            // Brim.cpp:639
            let p2 = p.to_f64();
            // Brim.cpp:640
            let a = cross2f(p1, p2);

            // Brim.cpp:642
            *sm = *sm
                + PointF::new(
                    p1.y() * p1.y() + p1.y() * p2.y() + p2.y() * p2.y(),
                    p1.x() * p1.x() + p1.x() * p2.x() + p2.x() * p2.x(),
                ) * a
                    / 12.0;
            // Brim.cpp:643
            p1 = p2;
        }
        // Brim.cpp:645
        return true;
    }
    // Brim.cpp:647
    false
}

// Brim.cpp:649
// BBS: properties of an expolygon
// Brim.cpp:650
#[derive(Clone, Copy, Default)]
pub struct ExPolyProp {
    // Brim.cpp:652
    pub aera: f64,
    // Brim.cpp:653
    pub centroid: PointF,
    // Brim.cpp:654
    pub second_moment_of_area_respect_to_centroid: PointF,
}

// Brim.cpp:657
// BBS: second moment of area of an expolyon
// Brim.cpp:658
pub fn comp_second_moment_expolygon(expoly: &ExPolygon, expoly_prop: &mut ExPolyProp) -> bool {
    if second_moment_pin() {
        // Brim.cpp:672-691 with Polygon::area() and Polygon::centroid() as the C++ build
        // evaluates them (`pin_area`, `pin_centroid`).
        let mut aera = pin_area(&expoly.contour);
        let c = pin_centroid(&expoly.contour);
        let mut cent = PointF::new(c.0 * aera, c.1 * aera);
        let mut sm = PointF::default();
        if !comp_second_moment_polygon(&expoly.contour, &mut sm) {
            return false;
        }

        for hole in &expoly.holes {
            let a = pin_area(hole);
            aera += pin_area(hole);
            let hc = pin_centroid(hole);
            cent = PointF::new(cent.x() + hc.0 * a, cent.y() + hc.1 * a);
            let mut smh = PointF::default();
            if comp_second_moment_polygon(hole, &mut smh) {
                sm = PointF::new(sm.x() + -smh.x(), sm.y() + -smh.y());
            }
        }

        cent = PointF::new(cent.x() / aera, cent.y() / aera);
        sm = PointF::new(sm.x() - (cent.y() * cent.y()) * aera, sm.y() - (cent.x() * cent.x()) * aera);
        expoly_prop.aera = aera;
        expoly_prop.centroid = cent;
        expoly_prop.second_moment_of_area_respect_to_centroid = sm;
        return true;
    }
    // Brim.cpp:660
    let mut aera = expoly.contour.area();
    // Brim.cpp:661
    let mut cent = expoly.contour.centroid().to_f64() * aera;
    // Brim.cpp:662
    let mut sm = PointF::default();
    // Brim.cpp:663-664
    if !comp_second_moment_polygon(&expoly.contour, &mut sm) {
        return false;
    }

    // Brim.cpp:666
    for hole in &expoly.holes {
        // Brim.cpp:668
        let a = hole.area();
        // Brim.cpp:669
        aera += hole.area();
        // Brim.cpp:670
        cent = cent + hole.centroid().to_f64() * a;
        // Brim.cpp:671
        let mut smh = PointF::default();
        // Brim.cpp:672-673
        if comp_second_moment_polygon(hole, &mut smh) {
            sm = sm + (smh * -1.0);
        }
    }

    // Brim.cpp:675
    cent = cent / aera;
    // Brim.cpp:676
    sm = sm - PointF::new(cent.y() * cent.y(), cent.x() * cent.x()) * aera;
    // Brim.cpp:677
    expoly_prop.aera = aera;
    // Brim.cpp:678
    expoly_prop.centroid = cent;
    // Brim.cpp:679
    expoly_prop.second_moment_of_area_respect_to_centroid = sm;
    // Brim.cpp:680
    true
}

// Brim.cpp:683
// BBS: second moment of area of expolygons
// Brim.cpp:684
pub fn comp_second_moment_expolygons(
    expolys: &ExPolygons,
    sm_expolys_x: &mut f64,
    sm_expolys_y: &mut f64,
) -> bool {
    // Brim.cpp:686
    if expolys.is_empty() {
        return false;
    }
    // Brim.cpp:687
    let mut props: Vec<ExPolyProp> = Vec::new();
    // Brim.cpp:688
    for expoly in expolys {
        // Brim.cpp:689
        let mut prop = ExPolyProp::default();
        // Brim.cpp:690-691
        if comp_second_moment_expolygon(expoly, &mut prop) {
            props.push(prop);
        }
    }
    // Brim.cpp:693-694
    if props.is_empty() {
        return false;
    }
    // Brim.cpp:695
    let mut total_area = 0.;
    // Brim.cpp:696
    let mut static_moment = PointF::new(0., 0.);
    // Brim.cpp:697
    for prop in &props {
        // Brim.cpp:698
        total_area += prop.aera;
        // Brim.cpp:699
        static_moment = static_moment + prop.centroid * prop.aera;
    }
    // Brim.cpp:701
    let total_centroid_x = static_moment.x() / total_area;
    // Brim.cpp:702
    let total_centroid_y = static_moment.y() / total_area;

    // Brim.cpp:704
    *sm_expolys_x = 0.;
    // Brim.cpp:705
    *sm_expolys_y = 0.;
    // Brim.cpp:706
    for prop in &props {
        // Brim.cpp:707
        let delta_x = prop.centroid.x() - total_centroid_x;
        // Brim.cpp:708
        let delta_y = prop.centroid.y() - total_centroid_y;
        if second_moment_pin() {
            // Brim.cpp:709-710: `s + (aera * d) * d`, the outer product and the sum fused.
            *sm_expolys_x += (prop.aera * delta_y).mul_add(delta_y, prop.second_moment_of_area_respect_to_centroid.x());
            *sm_expolys_y += (prop.aera * delta_x).mul_add(delta_x, prop.second_moment_of_area_respect_to_centroid.y());
            continue;
        }
        // Brim.cpp:709
        *sm_expolys_x += prop.second_moment_of_area_respect_to_centroid.x() + prop.aera * delta_y * delta_y;
        // Brim.cpp:710
        *sm_expolys_y += prop.second_moment_of_area_respect_to_centroid.y() + prop.aera * delta_x * delta_x;
    }

    // Brim.cpp:712
    true
}

// Brim.cpp:1226-1241
// Flip orientation of open polylines to minimize travel distance.
// C++: `static void optimize_polylines_by_reversing(Polylines *polylines)`
pub fn optimize_polylines_by_reversing(polylines: &mut Polylines) {
    // `(a - b).cast<double>().norm()`: in scaled coordinates, as the C++ compares them. (Until R919 the
    // two distances were taken in millimetres, `Point::to_f64`, where two equal distances can round apart.)
    let norm = |a: crate::geometry::Point, b: crate::geometry::Point| -> f64 {
        let (dx, dy) = ((a.x - b.x) as f64, (a.y - b.y) as f64);
        (dx * dx + dy * dy).sqrt()
    };
    // Brim.cpp:1229
    for poly_idx in 1..polylines.len() {
        // Brim.cpp:1230-1231
        let prev_last = polylines[poly_idx - 1].last_point();

        // Brim.cpp:1233
        if !polylines[poly_idx].is_closed() {
            // Brim.cpp:1234-1235
            let dist_to_start = norm(polylines[poly_idx].first_point(), prev_last);
            let dist_to_end = norm(polylines[poly_idx].last_point(), prev_last);

            // Brim.cpp:1237-1238
            if dist_to_end < dist_to_start {
                polylines[poly_idx].reverse();
            }
        }
    }
}

// ============================================================================
// Brim.cpp:29-557 over the Print of the pin (`crate::print::faithful`): the brim areas.
//
// The functions above take the engine's live `crate::print::Print`; the ones of this module take
// `print::faithful::Print`, the port of Print.hpp at the pin, which carries what Brim.cpp reads
// (`PrintInstance::shift_without_plate_offset`, `brim_flow`, `extruders`, `has_support_material`,
// `ObjectID`s, `support_layers`). Nothing of the engine's slicing path calls them yet: the live
// path prints no brim through Brim.cpp (app_slice.rs).
// ============================================================================

/// Brim.cpp:29-557, the brim areas, over `print::faithful::Print`.
pub mod areas {
    use crate::clipper::{ClipType, JoinType, PolyFillType};
    use crate::clipper_utils as u;
    use crate::clipper_utils::ApplySafetyOffset;
    use crate::config::ConfigOptionResolver;
    use crate::geometry::{polygons_reverse, ExPolygon, ExPolygons, Point, Polygon, Polygons};
    use crate::libslic3r::{EPSILON, SCALING_FACTOR};
    use crate::object_id::ObjectID;
    use crate::print::faithful::{Print, PrintInstance, PrintObject};
    use crate::support::support_common::{cu, cu_expolygon};
    use std::collections::{BTreeMap, HashSet};

    /// PrintConfig.hpp:236-243 `enum BrimType`, the values of the `brim_type` option.
    pub const BT_AUTO_BRIM: i32 = 0;
    pub const BT_BRIM_EARS: i32 = 1;
    pub const BT_OUTER_ONLY: i32 = 2;
    pub const BT_INNER_ONLY: i32 = 3;
    pub const BT_OUTER_AND_INNER: i32 = 4;
    pub const BT_NO_BRIM: i32 = 5;

    /// libslic3r.h:62-64 `RESOLUTION = 0.0125`, `SCALED_RESOLUTION (RESOLUTION / SCALING_FACTOR)`.
    pub const SCALED_RESOLUTION: f64 = 0.0125 / SCALING_FACTOR;

    /// libslic3r.h:81 `#define scale_(val) ((val) / SCALING_FACTOR)`
    #[inline]
    pub(super) fn scale_(val: f64) -> f64 {
        val / SCALING_FACTOR
    }

    /// `object->config().<key>.value` of a float option.
    pub(crate) fn object_config_float(object: &PrintObject, key: &str) -> f64 {
        object.config().option(key).and_then(|o| o.get_float().ok()).unwrap_or(0.0)
    }

    /// `object->config().<key>.value` of an int or enum option.
    pub(crate) fn object_config_int(object: &PrintObject, key: &str) -> i32 {
        object.config().option(key).and_then(|o| o.get_int().ok()).unwrap_or(0)
    }

    /// `print.config().<key>.value` of a float option.
    pub(crate) fn print_config_float(print: &Print, key: &str) -> f64 {
        print.config().option(key).and_then(|o| o.get_float().ok()).unwrap_or(0.0)
    }

    /// Print.hpp:999-1003 `const PrintObject* get_object(ObjectID object_id) const`: the first
    /// object of that ID.
    pub(crate) fn get_object(print: &Print, object_id: ObjectID) -> Option<&PrintObject> {
        print.objects().iter().find(|obj| obj.id() == object_id)
    }

    /// `ExPolygon(const Polygon &contour)` (ExPolygon.hpp:21).
    pub(super) fn expolygon_of_contour(contour: &Polygon) -> ExPolygon {
        ExPolygon { contour: contour.clone(), holes: Vec::new() }
    }

    /// ClipperUtils.cpp:755 `diff_ex(const ExPolygons &, const ExPolygons &, ApplySafetyOffset::No)`
    pub(crate) fn diff_ex_ee(subject: &[ExPolygon], clip: &[ExPolygon]) -> ExPolygons {
        u::_clipper_ex(
            ClipType::ctDifference,
            &cu::expolygons_provider(subject),
            &cu::expolygons_provider(clip),
            ApplySafetyOffset::No,
            PolyFillType::pftNonZero,
        )
    }

    /// ClipperUtils.hpp:358-363 `offset_ex(const Polygon &, float delta, JoinType, double miterLimit)`
    /// (BBS): `offset_ex(Polygons{polygon}, ...)`.
    pub(super) fn offset_ex_polygon(polygon: &Polygon, delta: f32, join_type: JoinType, miter_limit: f64) -> ExPolygons {
        let temp: Polygons = vec![polygon.clone()];
        cu::offset_ex(&temp, delta, join_type, miter_limit)
    }

    /// ClipperUtils.hpp:409-415 `closing_ex(const ExPolygons & / const Surfaces &, float delta)`:
    /// `offset2_ex(polygons, delta, -delta, DefaultJoinType, DefaultMiterLimit)` (ClipperUtils.cpp:581-584).
    fn closing_ex_expolygons(expolygons: &[ExPolygon], delta: f32) -> ExPolygons {
        let mut xy: Vec<i32> = Vec::new();
        let mut lens: Vec<i32> = Vec::new();
        let mut is_hole: Vec<i32> = Vec::new();
        for expoly in expolygons {
            for (k, poly) in std::iter::once(&expoly.contour).chain(expoly.holes.iter()).enumerate() {
                for pt in &poly.points {
                    xy.push(pt.x as i32);
                    xy.push(pt.y as i32);
                }
                lens.push(poly.points.len() as i32);
                is_hole.push(if k == 0 { 0 } else { 1 });
            }
        }
        u::offset2_ex_paths(&xy, &lens, &is_hole, delta as f64, -(delta as f64), cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT)
    }

    /// Brim.cpp:29-36
    /// C++: `static void append_and_translate(ExPolygons &dst, const ExPolygons &src, const PrintInstance &instance)`
    pub fn append_and_translate(dst: &mut ExPolygons, src: &ExPolygons, instance: &PrintInstance, print: &Print) {
        // Brim.cpp:30-31
        let mut dst_idx = dst.len();
        dst.extend(src.iter().cloned());

        // Brim.cpp:33-35
        let instance_shift: Point = instance.shift_without_plate_offset(print);
        while dst_idx < dst.len() {
            dst[dst_idx].translate(instance_shift);
            dst_idx += 1;
        }
    }

    /// Brim.cpp:37-47, the overload of `append_and_translate` that stores the area by object.
    pub mod append_and_translate_by_object {
        use super::*;
        /// Brim.cpp:38-47 (BBS: generate brim area by objs)
        /// C++: `static void append_and_translate(ExPolygons& dst, const ExPolygons& src, const PrintInstance& instance, const Print& print, std::map<ObjectID, ExPolygons>& brimAreaMap)`
        pub fn append_and_translate(
            dst: &mut ExPolygons,
            src: &ExPolygons,
            instance: &PrintInstance,
            print: &Print,
            brim_area_map: &mut BTreeMap<ObjectID, ExPolygons>,
        ) {
            // Brim.cpp:40-43
            let mut src_shifted: ExPolygons = src.clone();
            let instance_shift: Point = instance.shift_without_plate_offset(print);
            for src_idx in 0..src_shifted.len() {
                src_shifted[src_idx].translate(instance_shift);
            }
            // Brim.cpp:44
            src_shifted = diff_ex_ee(&src_shifted, dst);
            //expolygons_append(dst, temp2);
            // Brim.cpp:46 — brimAreaMap[instance.print_object->id()]
            let id = print.objects()[instance.print_object].id();
            brim_area_map.entry(id).or_default().extend(src_shifted);
        }
    }
    pub use append_and_translate_by_object::append_and_translate as append_and_translate_map;

    /// Brim.cpp:49-55, the overload of `append_and_translate` over Polygons.
    pub mod append_and_translate_polygons_overload {
        use super::*;
        /// Brim.cpp:49-55
        /// C++: `static void append_and_translate(Polygons &dst, const Polygons &src, const PrintInstance &instance)`
        pub fn append_and_translate(dst: &mut Polygons, src: &Polygons, instance: &PrintInstance, print: &Print) {
            let mut dst_idx = dst.len();
            dst.extend(src.iter().cloned());
            let instance_shift: Point = instance.shift_without_plate_offset(print);
            while dst_idx < dst.len() {
                dst[dst_idx].translate(instance_shift);
                dst_idx += 1;
            }
        }
    }
    pub use append_and_translate_polygons_overload::append_and_translate as append_and_translate_polygons;

    /// Brim.cpp:57-64
    /// C++: `static float max_brim_width(const ConstPrintObjectPtrsAdaptor &objects)`
    pub fn max_brim_width(objects: &[PrintObject]) -> f32 {
        debug_assert!(!objects.is_empty());
        let mut partial_result = 0.0f64;
        for object in objects {
            let w = if object_config_int(object, "brim_type") == BT_NO_BRIM { 0.0 } else { object_config_float(object, "brim_width") };
            // std::max(partial_result, w)
            partial_result = if partial_result < w { w } else { partial_result };
        }
        partial_result as f32
    }

    /// Brim.cpp:66-85
    /// C++: `static ExPolygons get_print_object_bottom_layer_expolygons(const PrintObject &print_object)`
    ///
    /// Returns ExPolygons of the bottom layer of the print object after elephant foot compensation.
    pub fn get_print_object_bottom_layer_expolygons(print_object: &PrintObject) -> ExPolygons {
        // Brim.cpp:69-70
        let mut ex_polygons: ExPolygons = Vec::new();
        let bottom_layer = &print_object.layers()[0];
        // SCALED_EPSILON as a float (libslic3r.h:84)
        let scaled_epsilon_f = scale_(EPSILON) as f32;
        // Brim.cpp:71-83
        for region in bottom_layer.regions() {
            // Brim.cpp:72-78
            let mut first_band: Option<&crate::adaptive_layer_merge::AdaptiveMergePatch> = None;
            if bottom_layer.bottom_z().abs() <= EPSILON {
                first_band = region.adaptive_merge_patches.iter().find(|patch| patch.bottom_idx == 0);
            }
            // Brim.cpp:79-82
            if let Some(first_band) = first_band {
                ex_polygons.extend(closing_ex_expolygons(first_band.area(), scaled_epsilon_f));
            } else {
                // `ClipperUtils::SurfacesProvider`: the expolygons of the surfaces.
                let surfaces_ex: ExPolygons = region.slices.surfaces.iter().map(|surface| surface.expolygon.clone()).collect();
                ex_polygons.extend(closing_ex_expolygons(&surfaces_ex, scaled_epsilon_f));
            }
        }
        // Brim.cpp:84
        ex_polygons
    }

    /// Brim.cpp:87-96
    /// C++: `static std::vector<ExPolygons> get_print_bottom_layers_expolygons(const Print &print)`
    ///
    /// Returns ExPolygons of bottom layer for every print object in Print after elephant foot compensation.
    pub fn get_print_bottom_layers_expolygons(print: &Print) -> Vec<ExPolygons> {
        let mut bottom_layers_expolygons: Vec<ExPolygons> = Vec::with_capacity(print.objects().len());
        for object in print.objects() {
            bottom_layers_expolygons.push(get_print_object_bottom_layer_expolygons(object));
        }
        bottom_layers_expolygons
    }

    /// Brim.cpp:98-159
    /// C++: `static ConstPrintObjectPtrs get_top_level_objects_with_brim(const Print &print, const std::vector<ExPolygons> &bottom_layers_expolygons)`
    ///
    /// The objects are returned as indices into `print.objects()`.
    pub fn get_top_level_objects_with_brim(print: &Print, bottom_layers_expolygons: &[ExPolygons]) -> Vec<usize> {
        use crate::clipper_lib_z as cz;
        // Brim.cpp:100
        debug_assert!(print.objects().len() == bottom_layers_expolygons.len());
        // Brim.cpp:101-102
        let mut islands: Polygons = Vec::new();
        let mut island_to_object: Vec<usize> = Vec::new();
        // Brim.cpp:103-119
        for print_object_idx in 0..print.objects().len() {
            let object = &print.objects()[print_object_idx];
            let mut islands_object: Polygons = Vec::with_capacity(bottom_layers_expolygons[print_object_idx].len());
            for ex_poly in &bottom_layers_expolygons[print_object_idx] {
                islands_object.push(ex_poly.contour.clone());
            }

            islands.reserve(object.instances().len() * islands_object.len());
            for instance in object.instances() {
                let instance_shift = instance.shift_without_plate_offset(print);
                for poly in &islands_object {
                    islands.push(poly.clone());
                    islands.last_mut().unwrap().translate(instance_shift);
                    island_to_object.push(print_object_idx);
                }
            }
        }
        // Brim.cpp:120
        debug_assert!(islands.len() == island_to_object.len());

        // Brim.cpp:122-132
        let mut islands_clip: cz::Paths = Vec::with_capacity(islands.len());
        for (island_idx, poly) in islands.iter().enumerate() {
            let mut island_clip: cz::Path = Vec::with_capacity(poly.points.len());
            // The Z coordinate carries index of the island used to get the pointer to the object.
            for pt in &poly.points {
                island_clip.push(cz::IntPoint::new_xyz(pt.x as cz::cInt, pt.y as cz::cInt, island_idx as cz::cInt + 1));
            }
            islands_clip.push(island_clip);
        }

        // Brim.cpp:134-140 — Init Clipper
        let mut clipper = cz::Clipper::new(0);
        // Assign the maximum Z from four points. This values is valid index of the island
        clipper.z_fill_function(cz::ZFillCallback::new(
            |e1bot: &cz::IntPoint, e1top: &cz::IntPoint, e2bot: &cz::IntPoint, e2top: &cz::IntPoint, pt: &mut cz::IntPoint| {
                pt.z = e1bot.z.max(e1top.z).max(e2bot.z.max(e2top.z));
            },
        ));
        // Brim.cpp:141-142 — Add islands
        clipper.add_paths(&islands_clip, cz::PolyType::ptSubject, true);
        // Brim.cpp:143-146 — Execute union operation to construct polytree
        let mut islands_polytree = cz::PolyTree::new();
        //FIXME likely pftNonZero or ptfPositive would be better. Why are we using ptfEvenOdd for Unions?
        clipper.execute_poly_tree(cz::ClipType::ctUnion, &mut islands_polytree, cz::PolyFillType::pftEvenOdd, cz::PolyFillType::pftEvenOdd);

        // Brim.cpp:148-158
        let mut processed_objects_idx: HashSet<usize> = HashSet::new();
        let mut top_level_objects_with_brim: Vec<usize> = Vec::new();
        for i in 0..islands_polytree.child_count() {
            let child = islands_polytree.root.childs[i as usize];
            for point in &islands_polytree.node(child).contour {
                if point.z != 0 {
                    let object_idx = island_to_object[(point.z - 1) as usize];
                    let id = print.objects()[object_idx].id().id;
                    if !processed_objects_idx.contains(&id) {
                        top_level_objects_with_brim.push(object_idx);
                        processed_objects_idx.insert(id);
                    }
                }
            }
        }
        top_level_objects_with_brim
    }

    /// `object->support_layers().front()->support_fills.polygons_covered_by_spacing()`
    /// (ExtrusionEntityCollection.hpp:116-117, `scaled_epsilon = 0.f`).
    pub(super) fn first_support_layer_contours(object: &PrintObject) -> Polygons {
        let mut out: Polygons = Vec::new();
        object.support_layers()[0].support_fills.polygons_covered_by_spacing(&mut out, 0.0);
        out
    }

    /// Brim.cpp:161-193
    /// C++: `static Polygons top_level_outer_brim_islands(const ConstPrintObjectPtrs &top_level_objects_with_brim, const double scaled_resolution)`
    pub fn top_level_outer_brim_islands(print: &Print, top_level_objects_with_brim: &[usize], scaled_resolution: f64) -> Polygons {
        let mut islands: Polygons = Vec::new();
        for &object_idx in top_level_objects_with_brim {
            let object = &print.objects()[object_idx];
            // Brim.cpp:165-166
            if !object.has_brim() {
                continue;
            }

            //FIXME how about the brim type?
            // Brim.cpp:169
            let brim_object_gap = scale_(object_config_float(object, "brim_object_gap")) as f32;
            let mut islands_object: Polygons = Vec::new();
            // Brim.cpp:171-177
            for ex_poly in &get_print_object_bottom_layer_expolygons(object) {
                let mut contour_offset = cu::offset_polygon(&ex_poly.contour, brim_object_gap, JoinType::jtSquare, cu::DEFAULT_MITER_LIMIT);
                for poly in &mut contour_offset {
                    poly.douglas_peucker(scaled_resolution);
                }
                islands_object.extend(contour_offset);
            }

            // Brim.cpp:179-187
            if !object.support_layers().is_empty() {
                for support_contour in &first_support_layer_contours(object) {
                    let mut contour_offset = cu::offset_polygon(support_contour, brim_object_gap, JoinType::jtSquare, cu::DEFAULT_MITER_LIMIT);
                    for poly in &mut contour_offset {
                        poly.douglas_peucker(scaled_resolution);
                    }
                    islands_object.extend(contour_offset);
                }
            }

            // Brim.cpp:189-190
            for instance in object.instances() {
                append_and_translate_polygons(&mut islands, &islands_object, instance, print);
            }
        }
        islands
    }

    /// Brim.cpp:219-221: `brimWidthAuto = floor(brimWidthAuto / flowWidth / 2) * flowWidth * 2` with
    /// `flowWidth = print.brim_flow().scaled_spacing() * SCALING_FACTOR`.
    fn brim_width_auto(print: &Print, object: &PrintObject) -> f64 {
        let brim_width_auto = object_config_float(object, "brim_width");
        let flow_width = print.brim_flow().expect("brim flow").scaled_spacing() as f64 * SCALING_FACTOR;
        (brim_width_auto / flow_width / 2.0).floor() * flow_width * 2.0
    }

    /// Brim.cpp:195-268
    /// C++: `static ExPolygons top_level_outer_brim_area(const Print &print, const ConstPrintObjectPtrs &top_level_objects_with_brim, const std::vector<ExPolygons> &bottom_layers_expolygons, const float no_brim_offset, double& brim_width_max, std::map<ObjectID, double>& brim_width_map)`
    pub fn top_level_outer_brim_area(
        print: &Print,
        top_level_objects_with_brim: &[usize],
        bottom_layers_expolygons: &[ExPolygons],
        no_brim_offset: f32,
        // BBS
        brim_width_max: &mut f64,
        brim_width_map: &mut BTreeMap<ObjectID, f64>,
    ) -> ExPolygons {
        // Brim.cpp:204 — scaled<double>(print.config().resolution.value)
        let scaled_resolution = print_config_float(print, "resolution") / SCALING_FACTOR;

        // Brim.cpp:206-210
        debug_assert!(print.objects().len() == bottom_layers_expolygons.len());
        let mut top_level_objects_idx: HashSet<usize> = HashSet::with_capacity(top_level_objects_with_brim.len());
        for &object_idx in top_level_objects_with_brim {
            top_level_objects_idx.insert(print.objects()[object_idx].id().id);
        }

        // Brim.cpp:212-214
        let mut brim_area: ExPolygons = Vec::new();
        let mut no_brim_area: ExPolygons = Vec::new();
        *brim_width_max = 0.0;
        // Brim.cpp:215-265
        for print_object_idx in 0..print.objects().len() {
            let object = &print.objects()[print_object_idx];
            let brim_type = object_config_int(object, "brim_type");
            let brim_object_gap: f32 = scale_(object_config_float(object, "brim_object_gap")) as f32;
            // recording the autoAssigned brimWidth and corresponding objs
            let brim_width_auto = brim_width_auto(print, object);
            // std::map::insert keeps an existing entry
            brim_width_map.entry(object.id()).or_insert(brim_width_auto);
            *brim_width_max = if *brim_width_max < brim_width_auto { brim_width_auto } else { *brim_width_max };
            let brim_width: f32 = scale_(brim_width_auto) as f32;
            let is_top_outer_brim = top_level_objects_idx.contains(&object.id().id);

            let mut brim_area_object: ExPolygons = Vec::new();
            let mut no_brim_area_object: ExPolygons = Vec::new();
            // Brim.cpp:229-248
            for ex_poly in &bottom_layers_expolygons[print_object_idx] {
                if (brim_type == BT_OUTER_ONLY || brim_type == BT_OUTER_AND_INNER || brim_type == BT_AUTO_BRIM) && is_top_outer_brim {
                    brim_area_object.extend(cu::diff_ex(
                        &cu::offset_polygon(&ex_poly.contour, brim_width + brim_object_gap, JoinType::jtRound, scaled_resolution),
                        &cu::offset_polygon(&ex_poly.contour, brim_object_gap, JoinType::jtSquare, cu::DEFAULT_MITER_LIMIT),
                        ApplySafetyOffset::No,
                    ));
                }

                // After 7ff76d07684858fd937ef2f5d863f105a10f798e offset and shrink don't work with CW polygons (holes), so let's make it CCW.
                let mut ex_poly_holes_reversed: Polygons = ex_poly.holes.clone();
                polygons_reverse(&mut ex_poly_holes_reversed);
                if brim_type == BT_OUTER_ONLY || brim_type == BT_NO_BRIM {
                    no_brim_area_object.extend(cu::shrink_ex(&ex_poly_holes_reversed, no_brim_offset, JoinType::jtSquare, cu::DEFAULT_MITER_LIMIT));
                }

                if brim_type == BT_INNER_ONLY || brim_type == BT_NO_BRIM {
                    no_brim_area_object.extend(cu::diff_ex(
                        &cu::offset_polygon(&ex_poly.contour, no_brim_offset, JoinType::jtSquare, cu::DEFAULT_MITER_LIMIT),
                        &ex_poly_holes_reversed,
                        ApplySafetyOffset::No,
                    ));
                }

                if brim_type != BT_NO_BRIM {
                    no_brim_area_object.extend(cu_expolygon::offset_ex(
                        &expolygon_of_contour(&ex_poly.contour),
                        brim_object_gap,
                        JoinType::jtSquare,
                        cu::DEFAULT_MITER_LIMIT,
                    ));
                }

                no_brim_area_object.push(expolygon_of_contour(&ex_poly.contour));
            }

            // Brim.cpp:250-260
            if !object.support_layers().is_empty() {
                for support_contour in &first_support_layer_contours(object) {
                    if (brim_type == BT_OUTER_ONLY || brim_type == BT_OUTER_AND_INNER || brim_type == BT_AUTO_BRIM) && is_top_outer_brim {
                        brim_area_object.extend(cu::diff_ex(
                            &cu::offset_polygon(support_contour, brim_width + brim_object_gap, JoinType::jtRound, scaled_resolution),
                            &cu::offset_polygon(support_contour, brim_object_gap, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                            ApplySafetyOffset::No,
                        ));
                    }

                    if brim_type != BT_NO_BRIM {
                        no_brim_area_object.extend(cu_expolygon::offset_ex(
                            &expolygon_of_contour(support_contour),
                            brim_object_gap,
                            cu::DEFAULT_JOIN_TYPE,
                            cu::DEFAULT_MITER_LIMIT,
                        ));
                    }

                    no_brim_area_object.push(expolygon_of_contour(support_contour));
                }
            }

            // Brim.cpp:262-265
            for instance in object.instances() {
                append_and_translate(&mut brim_area, &brim_area_object, instance, print);
                append_and_translate(&mut no_brim_area, &no_brim_area_object, instance, print);
            }
        }

        // Brim.cpp:267
        diff_ex_ee(&brim_area, &no_brim_area)
    }

    /// `struct brimWritten { bool obj; bool sup; }` (Brim.cpp:293-296, 462-465)
    #[derive(Clone, Copy)]
    pub(super) struct BrimWritten {
        pub(super) obj: bool,
        pub(super) sup: bool,
    }

    /// Brim.cpp:284-289, 447-452: the support filament of the first top-level object when the
    /// print has support material, 1 otherwise.
    pub(super) fn support_material_extruder_of(print: &Print, top_level_objects_with_brim: &[usize]) -> u32 {
        let mut support_material_extruder: u32 = 1;
        if print.has_support_material() {
            let front = &print.objects()[top_level_objects_with_brim[0]];
            debug_assert!(object_config_int(front, "support_filament") >= 0);
            if object_config_int(front, "support_filament") > 0 {
                support_material_extruder = object_config_int(front, "support_filament") as u32;
            }
        }
        support_material_extruder
    }

    /// Brim.cpp:270-378, the overload of `top_level_outer_brim_area` that stores the areas by
    /// object and by extruder.
    pub mod top_level_outer_brim_area_by_object {
        use super::*;
        /// Brim.cpp:271-378 (BBS: the brims of different objs will not overlapped with each other, and are stored by objs and by extruders)
        /// C++: `static ExPolygons top_level_outer_brim_area(const Print& print, const ConstPrintObjectPtrs& top_level_objects_with_brim, const float no_brim_offset, double& brim_width_max, std::map<ObjectID, double>& brim_width_map, std::map<ObjectID, ExPolygons>& brimAreaMap, std::map<ObjectID, ExPolygons>& supportBrimAreaMap, std::vector<std::pair<ObjectID, unsigned int>>& objPrintVec)`
        #[allow(clippy::too_many_arguments)]
        pub fn top_level_outer_brim_area(
            print: &Print,
            top_level_objects_with_brim: &[usize],
            no_brim_offset: f32,
            brim_width_max: &mut f64,
            brim_width_map: &mut BTreeMap<ObjectID, f64>,
            brim_area_map: &mut BTreeMap<ObjectID, ExPolygons>,
            support_brim_area_map: &mut BTreeMap<ObjectID, ExPolygons>,
            obj_print_vec: &[(ObjectID, u32)],
        ) -> ExPolygons {
            // Brim.cpp:276-279
            let mut top_level_objects_idx: HashSet<usize> = HashSet::with_capacity(top_level_objects_with_brim.len());
            for &object_idx in top_level_objects_with_brim {
                top_level_objects_idx.insert(print.objects()[object_idx].id().id);
            }

            // Brim.cpp:281-286
            let support_material_extruder = support_material_extruder_of(print, top_level_objects_with_brim);

            // Brim.cpp:288-298
            let mut brim_area: ExPolygons = Vec::new();
            let mut no_brim_area: ExPolygons = Vec::new();
            *brim_width_max = 0.0;
            let mut brim_to_write: BTreeMap<ObjectID, BrimWritten> = BTreeMap::new();
            for object_with_extruder in obj_print_vec {
                brim_to_write.entry(object_with_extruder.0).or_insert(BrimWritten { obj: true, sup: true });
            }

            // Brim.cpp:300-369
            for extruder_no in print.extruders(false) {
                let extruder_no = extruder_no + 1;
                for object_with_extruder in obj_print_vec {
                    let object = get_object(print, object_with_extruder.0).expect("print object of the ID");
                    let brim_type = object_config_int(object, "brim_type");
                    let brim_offset: f32 = scale_(object_config_float(object, "brim_object_gap")) as f32;
                    // recording the autoAssigned brimWidth and corresponding objs
                    let brim_width_auto = brim_width_auto(print, object);
                    brim_width_map.entry(object.id()).or_insert(brim_width_auto);
                    *brim_width_max = if *brim_width_max < brim_width_auto { brim_width_auto } else { *brim_width_max };
                    let brim_width: f32 = scale_(brim_width_auto) as f32;
                    let is_top_outer_brim = top_level_objects_idx.contains(&object.id().id);

                    // Brim.cpp:314-315
                    brim_area_map.entry(object.id()).or_default();
                    let mut brim_area_object: ExPolygons = Vec::new();
                    let mut brim_area_support: ExPolygons = Vec::new();
                    let mut no_brim_area_object: ExPolygons = Vec::new();
                    let mut no_brim_area_support: ExPolygons = Vec::new();
                    // Brim.cpp:320-344
                    if object_with_extruder.1 == extruder_no && brim_to_write[&object.id()].obj {
                        for ex_poly in &object.layers()[0].lslices {
                            if (brim_type == BT_OUTER_ONLY || brim_type == BT_OUTER_AND_INNER || brim_type == BT_AUTO_BRIM) && is_top_outer_brim {
                                brim_area_object.extend(diff_ex_ee(
                                    &offset_ex_polygon(&ex_poly.contour, brim_width + brim_offset, JoinType::jtRound, SCALED_RESOLUTION),
                                    &offset_ex_polygon(&ex_poly.contour, brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                ));
                            }
                            if brim_type == BT_OUTER_ONLY || brim_type == BT_NO_BRIM {
                                no_brim_area_object.extend(cu::offset_ex(&ex_poly.holes, -no_brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT));
                            }

                            if brim_type == BT_INNER_ONLY || brim_type == BT_NO_BRIM {
                                no_brim_area_object.extend(cu::diff_ex(
                                    &cu::offset_polygon(&ex_poly.contour, no_brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                    &ex_poly.holes,
                                    ApplySafetyOffset::No,
                                ));
                            }

                            if brim_type != BT_NO_BRIM {
                                no_brim_area_object.extend(cu_expolygon::offset_ex(
                                    &expolygon_of_contour(&ex_poly.contour),
                                    brim_offset,
                                    cu::DEFAULT_JOIN_TYPE,
                                    cu::DEFAULT_MITER_LIMIT,
                                ));
                            }

                            no_brim_area_object.push(expolygon_of_contour(&ex_poly.contour));
                        }
                        brim_to_write.get_mut(&object.id()).unwrap().obj = false;
                        for instance in object.instances() {
                            if !brim_area_object.is_empty() {
                                append_and_translate_map(&mut brim_area, &brim_area_object, instance, print, brim_area_map);
                            }
                            append_and_translate(&mut no_brim_area, &no_brim_area_object, instance, print);
                        }
                        if let Some(area) = brim_area_map.get(&object.id()) {
                            brim_area.extend(area.iter().cloned());
                        }
                    }
                    // Brim.cpp:345-367
                    if support_material_extruder == extruder_no && brim_to_write[&object.id()].sup {
                        if !object.support_layers().is_empty() {
                            for support_contour in &first_support_layer_contours(object) {
                                //BBS: no brim offset for supports
                                if (brim_type == BT_OUTER_ONLY || brim_type == BT_OUTER_AND_INNER || brim_type == BT_AUTO_BRIM) && is_top_outer_brim {
                                    brim_area_support.extend(cu::diff_ex(
                                        &cu::offset_polygon(support_contour, brim_width, JoinType::jtRound, SCALED_RESOLUTION),
                                        &cu::offset_polygon(support_contour, 0.0, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                        ApplySafetyOffset::No,
                                    ));
                                }

                                if brim_type != BT_NO_BRIM {
                                    no_brim_area_support.extend(offset_ex_polygon(support_contour, 0.0, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT));
                                }

                                no_brim_area_support.push(expolygon_of_contour(support_contour));
                            }
                        }

                        brim_to_write.get_mut(&object.id()).unwrap().sup = false;
                        for instance in object.instances() {
                            if !brim_area_support.is_empty() {
                                append_and_translate_map(&mut brim_area, &brim_area_support, instance, print, support_brim_area_map);
                            }
                            append_and_translate(&mut no_brim_area, &no_brim_area_support, instance, print);
                        }
                        if let Some(area) = support_brim_area_map.get(&object.id()) {
                            brim_area.extend(area.iter().cloned());
                        }
                    }
                }
            }
            // Brim.cpp:370-375
            for object in print.objects() {
                if let Some(area) = brim_area_map.get_mut(&object.id()) {
                    *area = diff_ex_ee(area, &no_brim_area);
                }
                if let Some(area) = support_brim_area_map.get_mut(&object.id()) {
                    *area = diff_ex_ee(area, &no_brim_area);
                }
            }
            // Brim.cpp:376
            diff_ex_ee(&brim_area, &no_brim_area)
        }
    }
    pub use top_level_outer_brim_area_by_object::top_level_outer_brim_area as top_level_outer_brim_area_map;

    /// ClipperUtils.cpp:788 `intersection_ex(to_polygons(std::move(expolygons)), holes)`.
    fn intersection_ex_of_expolygons_with(expolygons: &[ExPolygon], holes: &[Polygon]) -> ExPolygons {
        cu::intersection_ex(&crate::geometry::to_polygons(expolygons), holes, ApplySafetyOffset::No)
    }

    /// Brim.cpp:379-436
    /// C++: `static ExPolygons inner_brim_area(const Print &print, const ConstPrintObjectPtrs &top_level_objects_with_brim, const std::vector<ExPolygons> &bottom_layers_expolygons, const float no_brim_offset)`
    pub fn inner_brim_area(
        print: &Print,
        top_level_objects_with_brim: &[usize],
        bottom_layers_expolygons: &[ExPolygons],
        no_brim_offset: f32,
    ) -> ExPolygons {
        // Brim.cpp:384-388
        debug_assert!(print.objects().len() == bottom_layers_expolygons.len());
        let mut top_level_objects_idx: HashSet<usize> = HashSet::with_capacity(top_level_objects_with_brim.len());
        for &object_idx in top_level_objects_with_brim {
            top_level_objects_idx.insert(print.objects()[object_idx].id().id);
        }

        // Brim.cpp:390-392
        let mut brim_area: ExPolygons = Vec::new();
        let mut no_brim_area: ExPolygons = Vec::new();
        let mut holes: Polygons = Vec::new();
        // Brim.cpp:393-433
        for print_object_idx in 0..print.objects().len() {
            let object = &print.objects()[print_object_idx];
            let brim_type = object_config_int(object, "brim_type");
            let brim_object_gap: f32 = scale_(object_config_float(object, "brim_object_gap")) as f32;
            let brim_width: f32 = scale_(brim_width_auto(print, object)) as f32;
            let top_outer_brim = top_level_objects_idx.contains(&object.id().id);

            let mut brim_area_object: ExPolygons = Vec::new();
            let mut no_brim_area_object: ExPolygons = Vec::new();
            let mut holes_object: Polygons = Vec::new();
            // Brim.cpp:404-425
            for ex_poly in &bottom_layers_expolygons[print_object_idx] {
                if brim_type == BT_OUTER_ONLY || brim_type == BT_OUTER_AND_INNER || brim_type == BT_AUTO_BRIM {
                    if top_outer_brim {
                        no_brim_area_object.push(ex_poly.clone());
                    } else {
                        brim_area_object.extend(cu::diff_ex(
                            &cu::offset_polygon(&ex_poly.contour, brim_width + brim_object_gap, JoinType::jtSquare, cu::DEFAULT_MITER_LIMIT),
                            &cu::offset_polygon(&ex_poly.contour, brim_object_gap, JoinType::jtSquare, cu::DEFAULT_MITER_LIMIT),
                            ApplySafetyOffset::No,
                        ));
                    }
                }

                // After 7ff76d07684858fd937ef2f5d863f105a10f798e offset and shrink don't work with CW polygons (holes), so let's make it CCW.
                let mut ex_poly_holes_reversed: Polygons = ex_poly.holes.clone();
                polygons_reverse(&mut ex_poly_holes_reversed);
                if brim_type == BT_INNER_ONLY || brim_type == BT_OUTER_AND_INNER || brim_type == BT_AUTO_BRIM {
                    brim_area_object.extend(diff_ex_ee(
                        &cu::shrink_ex(&ex_poly_holes_reversed, brim_object_gap, JoinType::jtSquare, cu::DEFAULT_MITER_LIMIT),
                        &cu::shrink_ex(&ex_poly_holes_reversed, brim_width + brim_object_gap, JoinType::jtSquare, cu::DEFAULT_MITER_LIMIT),
                    ));
                }

                if brim_type == BT_INNER_ONLY || brim_type == BT_NO_BRIM {
                    no_brim_area_object.extend(cu::diff_ex(
                        &cu::offset_polygon(&ex_poly.contour, no_brim_offset, JoinType::jtSquare, cu::DEFAULT_MITER_LIMIT),
                        &ex_poly_holes_reversed,
                        ApplySafetyOffset::No,
                    ));
                }

                if brim_type == BT_OUTER_ONLY || brim_type == BT_NO_BRIM {
                    // ClipperUtils.hpp:480-486 diff_ex(const ExPolygon &, const ExPolygons &)
                    no_brim_area_object.extend(diff_ex_ee(
                        &[expolygon_of_contour(&ex_poly.contour)],
                        &cu::shrink_ex(&ex_poly_holes_reversed, no_brim_offset, JoinType::jtSquare, cu::DEFAULT_MITER_LIMIT),
                    ));
                }

                holes_object.extend(ex_poly_holes_reversed);
            }
            // Brim.cpp:426
            no_brim_area_object.extend(cu::offset_ex_expolygons(
                &bottom_layers_expolygons[print_object_idx],
                brim_object_gap,
                JoinType::jtSquare,
                cu::DEFAULT_MITER_LIMIT,
            ));

            // Brim.cpp:428-432
            for instance in object.instances() {
                append_and_translate(&mut brim_area, &brim_area_object, instance, print);
                append_and_translate(&mut no_brim_area, &no_brim_area_object, instance, print);
                append_and_translate_polygons(&mut holes, &holes_object, instance, print);
            }
        }

        // Brim.cpp:435
        diff_ex_ee(&intersection_ex_of_expolygons_with(&brim_area, &holes), &no_brim_area)
    }

    /// Brim.cpp:438-557, the overload of `inner_brim_area` that stores the areas by object and by
    /// extruder.
    pub mod inner_brim_area_by_object {
        use super::*;
        /// Brim.cpp:439-557 (BBS: the brims of different objs will not overlapped with each other, and are stored by objs and by extruders)
        /// C++: `static ExPolygons inner_brim_area(const Print& print, const ConstPrintObjectPtrs& top_level_objects_with_brim, const float no_brim_offset, std::map<ObjectID, ExPolygons>& brimAreaMap, std::map<ObjectID, ExPolygons>& supportBrimAreaMap, std::vector<std::pair<ObjectID, unsigned int>>& objPrintVec)`
        pub fn inner_brim_area(
            print: &Print,
            top_level_objects_with_brim: &[usize],
            no_brim_offset: f32,
            brim_area_map: &mut BTreeMap<ObjectID, ExPolygons>,
            support_brim_area_map: &mut BTreeMap<ObjectID, ExPolygons>,
            obj_print_vec: &[(ObjectID, u32)],
        ) -> ExPolygons {
            // Brim.cpp:444-447
            let mut top_level_objects_idx: HashSet<usize> = HashSet::with_capacity(top_level_objects_with_brim.len());
            for &object_idx in top_level_objects_with_brim {
                top_level_objects_idx.insert(print.objects()[object_idx].id().id);
            }

            // Brim.cpp:449-454
            let support_material_extruder = support_material_extruder_of(print, top_level_objects_with_brim);

            // Brim.cpp:456-462
            let mut brim_area: ExPolygons = Vec::new();
            let mut no_brim_area: ExPolygons = Vec::new();
            let mut holes: Polygons = Vec::new();
            let bed_shape_points = crate::print_config::config_def::get_bed_shape(print.config(), false);
            let _bed_shape = Polygon::from_points(bed_shape_points.clone());
            holes.push(Polygon::from_points(bed_shape_points));
            let mut inner_brim_area_map: BTreeMap<ObjectID, ExPolygons> = BTreeMap::new();
            let mut inner_support_brim_area_map: BTreeMap<ObjectID, ExPolygons> = BTreeMap::new();

            // Brim.cpp:464-470
            let mut brim_to_write: BTreeMap<ObjectID, BrimWritten> = BTreeMap::new();
            for object_with_extruder in obj_print_vec {
                brim_to_write.entry(object_with_extruder.0).or_insert(BrimWritten { obj: true, sup: true });
            }

            // Brim.cpp:473-535
            for extruder_no in print.extruders(false) {
                let extruder_no = extruder_no + 1;
                for object_with_extruder in obj_print_vec {
                    let object = get_object(print, object_with_extruder.0).expect("print object of the ID");
                    let brim_type = object_config_int(object, "brim_type");
                    let brim_offset: f32 = scale_(object_config_float(object, "brim_object_gap")) as f32;
                    let brim_width: f32 = scale_(brim_width_auto(print, object)) as f32;
                    let top_outer_brim = top_level_objects_idx.contains(&object.id().id);

                    let mut brim_area_object: ExPolygons = Vec::new();
                    let mut no_brim_area_object: ExPolygons = Vec::new();
                    let mut brim_area_support: ExPolygons = Vec::new();
                    let mut no_brim_area_support: ExPolygons = Vec::new();
                    let mut holes_object: Polygons = Vec::new();
                    let holes_support: Polygons = Vec::new();
                    // Brim.cpp:489-513
                    if object_with_extruder.1 == extruder_no && brim_to_write[&object.id()].obj {
                        for ex_poly in &object.layers()[0].lslices {
                            if brim_type == BT_OUTER_ONLY || brim_type == BT_OUTER_AND_INNER || brim_type == BT_AUTO_BRIM {
                                if top_outer_brim {
                                    no_brim_area_object.push(ex_poly.clone());
                                } else {
                                    brim_area_object.extend(diff_ex_ee(
                                        &offset_ex_polygon(&ex_poly.contour, brim_width + brim_offset, JoinType::jtRound, SCALED_RESOLUTION),
                                        &offset_ex_polygon(&ex_poly.contour, brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                    ));
                                }
                            }
                            if brim_type == BT_INNER_ONLY || brim_type == BT_OUTER_AND_INNER {
                                brim_area_object.extend(diff_ex_ee(
                                    &cu::offset_ex(&ex_poly.holes, -brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                    &cu::offset_ex(&ex_poly.holes, -brim_width - brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                ));
                            }
                            if brim_type == BT_INNER_ONLY || brim_type == BT_NO_BRIM {
                                no_brim_area_object.extend(cu::diff_ex(
                                    &cu::offset_polygon(&ex_poly.contour, no_brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                    &ex_poly.holes,
                                    ApplySafetyOffset::No,
                                ));
                            }
                            if brim_type == BT_OUTER_ONLY || brim_type == BT_NO_BRIM {
                                no_brim_area_object.extend(cu::offset_ex(&ex_poly.holes, -no_brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT));
                            }
                            holes_object.extend(ex_poly.holes.iter().cloned());
                        }
                        no_brim_area_object.extend(cu::offset_ex_expolygons(
                            &object.layers()[0].lslices,
                            brim_offset,
                            cu::DEFAULT_JOIN_TYPE,
                            cu::DEFAULT_MITER_LIMIT,
                        ));
                        brim_to_write.get_mut(&object.id()).unwrap().obj = false;
                        for instance in object.instances() {
                            if !brim_area_object.is_empty() {
                                append_and_translate_map(&mut brim_area, &brim_area_object, instance, print, &mut inner_brim_area_map);
                            }
                            append_and_translate(&mut no_brim_area, &no_brim_area_object, instance, print);
                            append_and_translate_polygons(&mut holes, &holes_object, instance, print);
                        }
                        if let Some(area) = inner_brim_area_map.get(&object.id()) {
                            brim_area.extend(area.iter().cloned());
                        }
                    }
                    // Brim.cpp:514-526
                    if support_material_extruder == extruder_no && brim_to_write[&object.id()].sup {
                        if !object.support_layers().is_empty() {
                            for support_contour in &first_support_layer_contours(object) {
                                if brim_type == BT_OUTER_ONLY || brim_type == BT_OUTER_AND_INNER || brim_type == BT_AUTO_BRIM {
                                    if !top_outer_brim {
                                        brim_area_support.extend(diff_ex_ee(
                                            &offset_ex_polygon(support_contour, brim_width + brim_offset, JoinType::jtRound, SCALED_RESOLUTION),
                                            &offset_ex_polygon(support_contour, brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                        ));
                                    }
                                }
                                if brim_type != BT_NO_BRIM {
                                    no_brim_area_support.extend(offset_ex_polygon(support_contour, 0.0, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT));
                                }
                                no_brim_area_support.push(expolygon_of_contour(support_contour));
                            }
                        }
                    }
                    // Brim.cpp:527-535 (outside the `if`, as in the C++)
                    brim_to_write.get_mut(&object.id()).unwrap().sup = false;
                    for instance in object.instances() {
                        if !brim_area_support.is_empty() {
                            append_and_translate_map(&mut brim_area, &brim_area_support, instance, print, &mut inner_support_brim_area_map);
                        }
                        append_and_translate(&mut no_brim_area, &no_brim_area_support, instance, print);
                        append_and_translate_polygons(&mut holes, &holes_support, instance, print);
                    }
                    if let Some(area) = inner_support_brim_area_map.get(&object.id()) {
                        brim_area.extend(area.iter().cloned());
                    }
                }
            }
            // Brim.cpp:537-546
            for object in print.objects() {
                if let Some(area) = inner_brim_area_map.get_mut(&object.id()) {
                    *area = intersection_ex_of_expolygons_with(area, &holes);
                    brim_area_map.entry(object.id()).or_default().extend(area.iter().cloned());
                }
                if let Some(area) = inner_support_brim_area_map.get_mut(&object.id()) {
                    *area = intersection_ex_of_expolygons_with(area, &holes);
                    support_brim_area_map.entry(object.id()).or_default().extend(area.iter().cloned());
                }
            }
            // Brim.cpp:547-552
            for object in print.objects() {
                if let Some(area) = brim_area_map.get_mut(&object.id()) {
                    *area = diff_ex_ee(area, &no_brim_area);
                }
                if let Some(area) = support_brim_area_map.get_mut(&object.id()) {
                    *area = diff_ex_ee(area, &no_brim_area);
                }
            }
            // Brim.cpp:553-555
            brim_area = intersection_ex_of_expolygons_with(&brim_area, &holes);
            no_brim_area.extend(brim_area);
            no_brim_area
        }
    }
    pub use inner_brim_area_by_object::inner_brim_area as inner_brim_area_map;
}

// ============================================================================
// Brim.cpp:560-883 over the Print of the pin: the automatic brim width, the brim ears, the support
// islands on the bed.
// ============================================================================

/// Brim.cpp:642-720, `compSecondMoment` under its C++ name: its three overloads.
pub mod comp_second_moment_of_polygon {
    /// Brim.cpp:642-659 `bool compSecondMoment(Polygon poly, Vec2d& sm)`
    pub fn comp_second_moment(poly: &crate::geometry::Polygon, sm: &mut crate::geometry::PointF) -> bool {
        super::comp_second_moment_polygon(poly, sm)
    }
}
/// Brim.cpp:670-692, the overload of `compSecondMoment` over an ExPolygon.
pub mod comp_second_moment_of_expolygon {
    /// Brim.cpp:670-692 `bool compSecondMoment(const ExPolygon& expoly, ExPolyProp& expolyProp)`
    pub fn comp_second_moment(expoly: &crate::geometry::ExPolygon, expoly_prop: &mut super::ExPolyProp) -> bool {
        super::comp_second_moment_expolygon(expoly, expoly_prop)
    }
}
/// Brim.cpp:696-720, the overload of `compSecondMoment` over ExPolygons.
pub mod comp_second_moment_of_expolygons {
    /// Brim.cpp:696-720 `bool compSecondMoment(const ExPolygons& expolys, double& smExpolysX, double& smExpolysY)`
    pub fn comp_second_moment(expolys: &crate::geometry::ExPolygons, sm_expolys_x: &mut f64, sm_expolys_y: &mut f64) -> bool {
        super::comp_second_moment_expolygons(expolys, sm_expolys_x, sm_expolys_y)
    }
}

/// Brim.cpp:560-883 over `print::faithful::Print`.
pub mod auto_width {
    use super::comp_second_moment_expolygons;
    use crate::config::ConfigOptionResolver;
    use crate::flow::{Flow, FlowRole};
    use crate::geometry::geometry::eigen;
    use crate::geometry::{BoundingBox, ExPolygon, ExPolygons, Point, Polygon};
    use crate::libslic3r::{EPSILON, SCALING_FACTOR};
    use crate::model::{ExtruderParams, Model, ModelObject, ModelVolume};
    use crate::print::faithful::{Print, PrintObject};
    use crate::support::support_common::cu;
    use std::collections::BTreeMap;

    /// libslic3r.h:60 `#define POLY_SIDE_COUNT 24 // for brim ear circle`
    pub const POLY_SIDE_COUNT: usize = 24;

    /// libslic3r.h:81 `#define scale_(val) ((val) / SCALING_FACTOR)`
    #[inline]
    fn scale_(val: f64) -> f64 {
        val / SCALING_FACTOR
    }

    /// Brim.cpp:565-572, 592-600: the extruders of the external perimeters of the regions of the
    /// first layer that have extrusions.
    fn extruders_first_layer(print_object: &PrintObject) -> Vec<usize> {
        let mut extruders_first_layer: Vec<usize> = Vec::new();
        let first_layer_regions = print_object.layers()[0].regions();
        if !first_layer_regions.is_empty() {
            for region_ptr in first_layer_regions {
                if region_ptr.has_extrusions() {
                    extruders_first_layer
                        .push(region_ptr.region().extruder(FlowRole::ExternalPerimeter).expect("extruder of the external perimeter") as usize);
                }
            }
        }
        extruders_first_layer
    }

    /// Brim.cpp:561-585 (BBS maximum temperature difference from print object class)
    /// C++: `double getTemperatureFromExtruder(const PrintObject* printObject)`
    pub fn get_temperature_from_extruder(print: &Print, print_object: &PrintObject) -> f64 {
        // Brim.cpp:563-572
        let extruders_first_layer = extruders_first_layer(print_object);

        // Brim.cpp:574-576
        let config = print.config();
        let curr_bed_type = config.option("curr_bed_type").and_then(|o| o.get_int().ok()).expect("curr_bed_type");
        let key = crate::print_config::config_def::get_bed_temp_1st_layer_key(crate::print_config::enums::BedType::from_i32(curr_bed_type));
        let bed_temp_1st_layer_opt = config
            .option(&key)
            .and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionInts>())
            .expect("the first-layer bed temperatures of the bed type");

        // Brim.cpp:578-583
        let mut max_delta_temp = 0.0f64;
        for extruder_id in extruders_first_layer {
            // ConfigOptionVector::get_at: the first value when the index is out of range.
            let idx = extruder_id.wrapping_sub(1);
            let bed_temp: i32 =
                if idx < bed_temp_1st_layer_opt.values.len() { bed_temp_1st_layer_opt.values[idx] } else { bed_temp_1st_layer_opt.values[0] };
            if bed_temp as f64 > max_delta_temp {
                max_delta_temp = bed_temp as f64;
            }
        }

        max_delta_temp
    }

    /// Brim.cpp:586-639 (BBS adhesion coefficients from print object class)
    /// C++: `double getadhesionCoeff(const PrintObject* printObject)`
    ///
    /// `extruder_params` is the static `Model::extruderParamsMap`.
    pub fn getadhesion_coeff(print: &Print, print_object: &PrintObject, extruder_params: &BTreeMap<usize, ExtruderParams>) -> f64 {
        // Brim.cpp:588-589 — insts[0].model_instance->get_object()->volumes
        let model_object: &ModelObject = print_object.model_object(print);
        let object_volumes = &model_object.volumes;

        // Brim.cpp:591-600
        let extruders_first_layer = extruders_first_layer(print_object);
        // Brim.cpp:601-614
        let mut adhesion_coeff = 1.0f64;
        for model_volume in object_volumes {
            for iter in &extruders_first_layer {
                // `modelVolume->extruder_id() == *iter`: int against size_t, the int converted.
                let extruder_id = model_volume.extruder_id(model_object);
                if extruder_id as isize as usize == *iter {
                    if let Some(p) = extruder_params.get(&(extruder_id as isize as usize)) {
                        if p.material_name == "PETG" || p.material_name == "PCTG" {
                            adhesion_coeff = 2.0;
                        } else if p.material_name == "TPU" || p.material_name == "TPU-AMS" {
                            adhesion_coeff = 0.5;
                        }
                    }
                }
            }
        }

        adhesion_coeff
    }

    /// Brim.cpp:749-754, 800-805: the bounding box of the contours and the thermal length,
    /// `sqrt(bboxX * bboxX + bboxY * bboxY) * SCALING_FACTOR` (the sum of the products fuses).
    fn thermal_length_of(expolys: &ExPolygons) -> (BoundingBox, f64) {
        let mut bbox2 = BoundingBox::new();
        for expoly in expolys {
            bbox2.merge(&crate::geometry::polygon::get_extents(&expoly.contour));
        }
        let bbox_x = bbox2.size().x as i32 as f64;
        let bbox_y = bbox2.size().y as i32 as f64;
        let thermal_length = bbox_x.mul_add(bbox_x, bbox_y * bbox_y).sqrt() * SCALING_FACTOR;
        (bbox2, thermal_length)
    }

    /// Brim.cpp:731-769 (BBS: config brimwidth by volumes)
    /// C++: `double configBrimWidthByVolumes(double deltaT, double adhension, double maxSpeed, const ModelVolume* modelVolumePtr, const ExPolygons& expolys)`
    ///
    /// `object` is the parent of the volume (`modelVolumePtr->get_object()`), `extruder_params`
    /// the static `Model::extruderParamsMap`.
    pub fn config_brim_width_by_volumes(
        _delta_t: f64,
        adhension: f64,
        max_speed: f64,
        model_volume_ptr: &ModelVolume,
        object: &ModelObject,
        extruder_params: &BTreeMap<usize, ExtruderParams>,
        expolys: &ExPolygons,
    ) -> f64 {
        // Brim.cpp:733-740 — height of a volume
        let mut height = 0.0f64;
        if model_volume_ptr.is_model_part() {
            let raw_boundingbox = model_volume_ptr.mesh().transformed_bounding_box(&model_volume_ptr.get_matrix(false, false, false, false));
            let bbox = object.instances[0].transform_bounding_box(&raw_boundingbox, false);
            let bbox_size = bbox.size();
            height = bbox_size.z;
        }

        // Brim.cpp:742-748 — sencond moment of the expolygons of the first layer of the volume
        let (mut ixx, mut iyy) = (-1.0e30f64, -1.0e30f64);
        if !expolys.is_empty() {
            if !comp_second_moment_expolygons(expolys, &mut ixx, &mut iyy) {
                ixx = -1.0e30;
                iyy = -1.0e30;
            }
        }
        ixx = ixx * SCALING_FACTOR * SCALING_FACTOR * SCALING_FACTOR * SCALING_FACTOR;
        iyy = iyy * SCALING_FACTOR * SCALING_FACTOR * SCALING_FACTOR * SCALING_FACTOR;

        // Brim.cpp:750-756 — bounding box of the expolygons of the first layer of the volume
        let (bbox2, thermal_length) = thermal_length_of(expolys);
        let thermal_length_ref = Model::get_thermal_length(model_volume_ptr, object, extruder_params);

        // Brim.cpp:758
        let a = height / ixx * (bbox2.size().y as i32 as f64 * SCALING_FACTOR);
        let b = height / iyy * (bbox2.size().x as i32 as f64 * SCALING_FACTOR);
        let height_to_area = if a < b { b } else { a };
        // Brim.cpp:759
        let h30 = if 30.0 < height { 30.0 } else { height };
        let c1 = height_to_area * max_speed / 24.0;
        let c2 = thermal_length * 8.0 / thermal_length_ref * h30 / 30.0;
        let m1 = if c1 < c2 { c2 } else { c1 };
        let m2 = if 18.0 < m1 { 18.0 } else { m1 };
        let c3 = 1.5 * thermal_length;
        let m3 = if c3 < m2 { c3 } else { m2 };
        let mut brim_width = adhension * m3;
        // Brim.cpp:760-762 — small brims are omitted
        if brim_width < 5.0 && brim_width < 1.5 * thermal_length {
            brim_width = 0.0;
        }
        // Brim.cpp:763-764 — large brims are omitted
        if brim_width > 18.0 {
            brim_width = 18.0;
        }

        brim_width
    }

    /// Brim.cpp:771-818 (BBS: config brimwidth by group of volumes)
    /// C++: `double configBrimWidthByVolumeGroups(double adhension, double maxSpeed, const std::vector<ModelVolume*> modelVolumePtrs, const ExPolygons& expolys, double &groupHeight)`
    ///
    /// `object` is the parent of the volumes, `extruder_params` the static `Model::extruderParamsMap`.
    pub fn config_brim_width_by_volume_groups(
        adhension: f64,
        max_speed: f64,
        model_volume_ptrs: &[&ModelVolume],
        object: &ModelObject,
        extruder_params: &BTreeMap<usize, ExtruderParams>,
        expolys: &ExPolygons,
        group_height: &mut f64,
    ) -> f64 {
        // Brim.cpp:774-787 — height of a group of volumes
        let mut merged_bbx = crate::bounding_box::BoundingBoxf3::new();
        for model_volume_ptr in model_volume_ptrs {
            if model_volume_ptr.is_model_part() {
                let t = if !object.instances.is_empty() {
                    eigen::transform_mul(
                        &object.instances[0].get_matrix(false, false, false, false),
                        &model_volume_ptr.get_matrix(false, false, false, false),
                    )
                } else {
                    model_volume_ptr.get_matrix(false, false, false, false)
                };
                let bbox = model_volume_ptr.mesh().transformed_bounding_box(&t);
                merged_bbx.merge_bb(&bbox);
            }
        }
        // Brim.cpp:788-790
        let bbox_size = merged_bbx.size();
        let height: f64 = bbox_size.z;
        *group_height = height;
        // Brim.cpp:791-798 — second moment of the expolygons of the first layer of the volume group
        let (mut ixx, mut iyy) = (-1.0e30f64, -1.0e30f64);
        if !expolys.is_empty() {
            if !comp_second_moment_expolygons(expolys, &mut ixx, &mut iyy) {
                ixx = -1.0e30;
                iyy = -1.0e30;
            }
        }
        ixx = ixx * SCALING_FACTOR * SCALING_FACTOR * SCALING_FACTOR * SCALING_FACTOR;
        iyy = iyy * SCALING_FACTOR * SCALING_FACTOR * SCALING_FACTOR * SCALING_FACTOR;

        // Brim.cpp:800-806 — bounding box of the expolygons of the first layer of the volume
        let (bbox2, thermal_length) = thermal_length_of(expolys);
        // Model.cpp:4426-4437 `Model::getThermalLength(const std::vector<ModelVolume*> modelVolumePtrs)`:
        // the thermal length of a group is decided by the volume with shortest thermal length
        let mut thermal_length_ref = 1250.0f64;
        for model_volume_ptr in model_volume_ptrs {
            let t = Model::get_thermal_length(model_volume_ptr, object, extruder_params);
            if t < thermal_length_ref {
                thermal_length_ref = t;
            }
        }

        // Brim.cpp:808
        let a = height / ixx * (bbox2.size().y as i32 as f64 * SCALING_FACTOR);
        let b = height / iyy * (bbox2.size().x as i32 as f64 * SCALING_FACTOR);
        let height_to_area = (if a < b { b } else { a }) * height / 1920.0;
        // Brim.cpp:809
        let h30 = if 30.0 < height { 30.0 } else { height };
        let c1 = height_to_area * max_speed;
        let c2 = 0.0 * thermal_length * 8.0 / thermal_length_ref * h30 / 30.0;
        let m1 = if c1 < c2 { c2 } else { c1 };
        let m2 = if 18.0 < m1 { 18.0 } else { m1 };
        let c3 = 1.5 * thermal_length;
        let m3 = if c3 < m2 { c3 } else { m2 };
        let mut brim_width = adhension * m3;
        // Brim.cpp:810-812 — small brims are omitted
        if brim_width < 5.0 && brim_width < 1.5 * thermal_length {
            brim_width = 0.0;
        }
        // Brim.cpp:813-814 — large brims are omitted
        if brim_width > 18.0 {
            brim_width = 18.0;
        }

        brim_width
    }

    /// Brim.cpp:820-860
    /// C++: `static ExPolygons make_brim_ears(const PrintObject* object, const double& flowWidth, float brim_offset, Flow &flow, bool is_outer_brim)`
    pub fn make_brim_ears(print: &Print, object: &PrintObject, flow_width: f64, brim_offset: f32, flow: &Flow, is_outer_brim: bool) -> ExPolygons {
        // Brim.cpp:822-826
        let mut mouse_ears_ex: ExPolygons = Vec::new();
        let model_object = object.model_object(print);
        let brim_ear_points = model_object.brim_points.clone();
        if brim_ear_points.is_empty() {
            return mouse_ears_ex;
        }
        // Brim.cpp:827-830
        let trsf = model_object.instances[0].get_transformation();
        let mut model_trsf = trsf.get_matrix(true, false, false, false);
        let center_offset = object.center_offset();
        // `pretranslate` of an affine transform adds to its translation (Eigen Transform.h).
        model_trsf[(0, 3)] += -(center_offset.x as f64 * SCALING_FACTOR);
        model_trsf[(1, 3)] += -(center_offset.y as f64 * SCALING_FACTOR);
        model_trsf[(2, 3)] += 0.0;
        // Brim.cpp:831-858
        for pt in &brim_ear_points {
            let mut v_trsf = crate::geometry::geometry::Transform3d::identity();
            if pt.volume_idx > -1 {
                // volume_idx may be stale (e.g. volumes were removed or merged after the ear was placed).
                if pt.volume_idx >= model_object.volumes.len() as i32 {
                    continue;
                }
                v_trsf = model_object.volumes[pt.volume_idx as usize].get_matrix(false, false, false, false);
            }
            // Brim.cpp:839-840
            let world_pos = pt.transform(&eigen::transform_mul(&trsf.get_matrix(false, false, false, false), &v_trsf));
            if world_pos.z > 0.0 {
                continue;
            }
            // Brim.cpp:841-846
            let mut point_round = Polygon::default();
            let mut brim_width: f32 = ((scale_(pt.head_front_radius as f64) / flow_width / 2.0).floor() * flow_width * 2.0) as f32;
            if is_outer_brim {
                let flow_width_scale = flow_width / SCALING_FACTOR;
                brim_width = ((brim_width as f64 / flow_width_scale / 2.0).floor() * flow_width_scale * 2.0) as f32;
            }
            // Brim.cpp:847 — `coord_t size_ear = (brim_width - brim_offset - flow.scaled_spacing())`, in float
            let size_ear: i32 = (brim_width - brim_offset - (flow.scaled_spacing() as i32 as f32)) as i32;
            // Brim.cpp:848-851
            for i in 0..POLY_SIDE_COUNT {
                let angle = (2.0 * std::f64::consts::PI * i as f64) / POLY_SIDE_COUNT as f64;
                // Point(double, double) rounds (lrint)
                point_round.points.push(Point::new(
                    (size_ear as f64 * angle.cos()).round_ties_even() as i64,
                    (size_ear as f64 * angle.sin()).round_ties_even() as i64,
                ));
            }
            // Brim.cpp:852-857
            let mut ear = ExPolygon::default();
            ear.contour = point_round;
            let pos = pt.transform(&eigen::transform_mul(&model_trsf, &v_trsf));
            let pt_x: i32 = scale_(pos.x as f64) as i32;
            let pt_y: i32 = scale_(pos.y as f64) as i32;
            ear.contour.translate(Point::new(pt_x as i64, pt_y as i64));
            mouse_ears_ex.push(ear);
        }
        mouse_ears_ex
    }

    /// Brim.cpp:862-881
    /// C++: `static ExPolygons bed_support_islands(const Print& print, const PrintObject& object)`
    ///
    /// Support islands extruded onto the bed. With multiple nozzle diameters a merged band reaches the bed from its
    /// top layer, which need not be the first support layer: the layers it swallowed have no height and no islands.
    pub fn bed_support_islands(print: &Print, object: &PrintObject) -> ExPolygons {
        // Brim.cpp:866-870
        let support_layers = object.support_layers();
        if support_layers.is_empty() {
            return Vec::new();
        }
        if !print.uses_multiple_nozzle_diameters() {
            return support_layers[0].support_islands.clone();
        }

        // Brim.cpp:872-880
        let mut islands: ExPolygons = Vec::new();
        let mut num_layers: usize = 0;
        for support_layer in support_layers {
            let height = support_layer.effective_print_height();
            if height < EPSILON || support_layer.layer.print_z - height > EPSILON {
                continue;
            }
            islands.extend(support_layer.support_islands.iter().cloned());
            num_layers += 1;
        }
        if num_layers > 1 {
            cu::union_ex_expolygons(&islands)
        } else {
            islands
        }
    }
}

// ============================================================================
// Brim.cpp:884-2050 over the Print of the pin: the brim areas of every object by volume groups, the
// brim loops, the two overloads of make_brim.
// ============================================================================

/// Brim.cpp:884-2050 over `print::faithful::Print`.
///
/// `Model::extruderParamsMap` and `Model::printSpeedMap` are statics of the C++ Model; the functions that read
/// them take them as `extruder_params` and `print_speed_map`. Nothing of the engine's slicing path calls this
/// module: the live path prints its brim through the earlier code of print.rs.
pub mod loops {
    use super::areas::{
        append_and_translate, append_and_translate_map, append_and_translate_polygons, diff_ex_ee, expolygon_of_contour, first_support_layer_contours,
        get_object, get_print_bottom_layers_expolygons, get_top_level_objects_with_brim, inner_brim_area, inner_brim_area_map, object_config_float,
        object_config_int, offset_ex_polygon, print_config_float, scale_, support_material_extruder_of, top_level_outer_brim_area,
        top_level_outer_brim_islands, BrimWritten, BT_AUTO_BRIM, BT_BRIM_EARS, BT_INNER_ONLY, BT_NO_BRIM, BT_OUTER_AND_INNER, BT_OUTER_ONLY, SCALED_RESOLUTION,
    };
    use super::auto_width::{bed_support_islands, config_brim_width_by_volume_groups, get_temperature_from_extruder, getadhesion_coeff, make_brim_ears};
    use super::optimize_polylines_by_reversing;
    use crate::clipper::{ClipType, JoinType, PolyFillType, PolyTree};
    use crate::clipper_utils as u;
    use crate::clipper_utils::ApplySafetyOffset;
    use crate::config::ConfigOptionResolver;
    use crate::extrusion_entity::{ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionLoopRole, ExtrusionPath, ExtrusionRole};
    use crate::flow::{Flow, FlowResult};
    use crate::geometry::{polygons_reverse, ExPolygon, ExPolygons, Line, Point, Polygon, Polygons, Polyline, Polylines};
    use crate::libslic3r::{EPSILON, SCALING_FACTOR};
    use crate::model::{ExtruderParams, GlobalSpeedMap, Model, ModelVolume};
    use crate::object_id::ObjectID;
    use crate::print::faithful::{Print, PrintInstance};
    use crate::support::support_common::{cu, cu_expolygon, cu_polygon, get_extents_polygons_bbox, get_extents_polylines_bbox, BBox};
    use std::collections::{BTreeMap, HashSet};

    /// `scaled<double>(print.config().resolution.value)` (Point.hpp:524-530)
    fn scaled_resolution_of(print: &Print) -> f64 {
        print_config_float(print, "resolution") / SCALING_FACTOR
    }

    /// `print.config().<key>.value` of a bool option.
    fn print_config_bool(print: &Print, key: &str) -> bool {
        print.config().option(key).and_then(|o| o.get_bool().ok()).unwrap_or(false)
    }

    /// `print.config().<key>.value` of an int or enum option.
    fn print_config_int(print: &Print, key: &str) -> i32 {
        print.config().option(key).and_then(|o| o.get_int().ok()).unwrap_or(0)
    }

    /// `print.config().<key>.values` of a `ConfigOptionPoints`.
    fn print_config_points(print: &Print, key: &str) -> Vec<crate::geometry::Vec2d> {
        print.config().option(key).and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionPoints>()).map(|o| o.values.clone()).unwrap_or_default()
    }

    /// Polygon.hpp:30-36 `static Polygon new_scale(const std::vector<Vec2d> &points)`: `Point::new_scale`
    /// truncates (Point.hpp:204, `coord_t(scale_(x))`).
    fn polygon_new_scale(points: &[crate::geometry::Vec2d]) -> Polygon {
        Polygon::from_points(points.iter().map(|p| Point::new(scale_(p.x) as i64, scale_(p.y) as i64)).collect())
    }

    /// ExPolygon.cpp:69-74 `void ExPolygon::douglas_peucker(double tolerance)`
    fn expolygon_douglas_peucker(expolygon: &mut ExPolygon, tolerance: f64) {
        expolygon.contour.douglas_peucker(tolerance);
        for poly in &mut expolygon.holes {
            poly.douglas_peucker(tolerance);
        }
    }

    /// ClipperUtils.cpp `diff_ex(const Polygons &subject, const ExPolygons &clip, ApplySafetyOffset::No)`
    fn diff_ex_pe(subject: &[Polygon], clip: &[ExPolygon]) -> ExPolygons {
        u::_clipper_ex(ClipType::ctDifference, &cu::polygons_provider(subject), &cu::expolygons_provider(clip), ApplySafetyOffset::No, PolyFillType::pftNonZero)
    }

    /// ClipperUtils.cpp `intersection_ex(const ExPolygons &subject, const Polygons &clip, ApplySafetyOffset::No)`
    fn intersection_ex_ep(subject: &[ExPolygon], clip: &[Polygon]) -> ExPolygons {
        u::_clipper_ex(ClipType::ctIntersection, &cu::expolygons_provider(subject), &cu::polygons_provider(clip), ApplySafetyOffset::No, PolyFillType::pftNonZero)
    }

    /// ExPolygon.hpp `Polygons to_polygons(const ExPolygons &src)`: of every expolygon its contour, then its holes.
    fn to_polygons(expolygons: &[ExPolygon]) -> Polygons {
        crate::geometry::to_polygons(expolygons)
    }

    /// ClipperUtils.cpp:999-1017
    /// C++: `static void traverse_pt_outside_in(ClipperLib::PolyNodes &&nodes, Polygons *retval)`
    fn traverse_pt_outside_in(tree: &PolyTree, nodes: &[usize], retval: &mut Polygons) {
        // collect ordering points
        let mut ordering_points: Vec<Point> = Vec::with_capacity(nodes.len());
        for &node in nodes {
            let front = &tree.node(node).contour[0];
            ordering_points.push(Point::new(front.x as i64, front.y as i64));
        }

        // Perform the ordering, push results recursively.
        //FIXME pass the last point to chain_clipper_polynodes?
        // ShortestPath.cpp:1975-1978 `chain_clipper_polynodes(points, items)` is `chain_path_items`: the items in
        // the order `chain_segments_greedy` gives their points.
        for idx in crate::shortest_path::chain_points(&ordering_points, None) {
            let node = nodes[idx];
            retval.push(Polygon::from_points(tree.node(node).contour.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect()));
            if tree.is_hole(node) {
                // Orient a hole, which is clockwise oriented, to CCW.
                retval.last_mut().unwrap().reverse();
            }
            // traverse the next depth
            traverse_pt_outside_in(tree, &tree.node(node).childs, retval);
        }
    }

    /// ClipperUtils.cpp:1019-1024 `Polygons union_pt_chained_outside_in(const Polygons &subject)`, with
    /// `union_pt` (ClipperUtils.cpp:962-965): a PolyTree by an even-odd union, which unites nothing.
    pub fn union_pt_chained_outside_in(subject: &[Polygon]) -> Polygons {
        let mut retval: Polygons = Vec::new();
        let tree: PolyTree = u::clipper_do::<PolyTree>(ClipType::ctUnion, &cu::polygons_provider(subject), &crate::clipper::Paths::new(), PolyFillType::pftEvenOdd);
        traverse_pt_outside_in(&tree, &tree.root.childs, &mut retval);
        retval
    }

    /// ExtrusionEntity.hpp:732-744
    /// C++: `inline void extrusion_entities_append_loops(ExtrusionEntitiesPtr &dst, Polygons &&loops, ExtrusionRole role, double mm3_per_mm, float width, float height)`
    fn extrusion_entities_append_loops(dst: &mut Vec<ExtrusionEntityType>, loops: Polygons, role: ExtrusionRole, mm3_per_mm: f64, width: f32, height: f32) {
        dst.reserve(loops.len());
        for poly in loops {
            if poly.is_valid() {
                let mut path = ExtrusionPath::with_params(role, mm3_per_mm, width as f64, height as f64, false);
                path.polyline.points = poly.points;
                let front = path.polyline.points[0];
                path.polyline.points.push(front);
                dst.push(ExtrusionEntityType::Loop(ExtrusionLoop::from_path(path, ExtrusionLoopRole::DEFAULT)));
            }
        }
    }

    /// ExtrusionEntity.hpp:746-763
    /// C++: `inline void extrusion_entities_append_loops_and_paths(ExtrusionEntitiesPtr &dst, Polylines &&polylines, ExtrusionRole role, double mm3_per_mm, float width, float height)`
    fn extrusion_entities_append_loops_and_paths(dst: &mut Vec<ExtrusionEntityType>, polylines: Polylines, role: ExtrusionRole, mm3_per_mm: f64, width: f32, height: f32) {
        dst.reserve(polylines.len());
        for polyline in polylines {
            if polyline.is_valid() {
                if polyline.is_closed() {
                    let mut extrusion_path = ExtrusionPath::with_params(role, mm3_per_mm, width as f64, height as f64, false);
                    extrusion_path.polyline = polyline;
                    dst.push(ExtrusionEntityType::Loop(ExtrusionLoop::from_path(extrusion_path, ExtrusionLoopRole::DEFAULT)));
                } else {
                    let mut extrusion_path = ExtrusionPath::with_params(role, mm3_per_mm, width as f64, height as f64, false);
                    extrusion_path.polyline = polyline;
                    dst.push(ExtrusionEntityType::Path(extrusion_path));
                }
            }
        }
    }

    /// Brim.cpp:906-917, the lambda `save_polygon_if_is_inner_island` of `outer_inner_brim_area`: the index of
    /// the hole the contour lies in, -1 when in none.
    fn inner_island_hole_index(holes_area: &Polygons, contour: &Polygon) -> i32 {
        for i in 0..holes_area.len() {
            let contour_polys: Polygons = vec![contour.clone()];
            if cu::diff_ex(&contour_polys, std::slice::from_ref(&holes_area[i]), ApplySafetyOffset::No).is_empty() {
                // BBS: this is an inner island inside holes_area[i], save
                return i as i32;
            }
        }
        -1
    }

    /// Brim.cpp:884-1225 (BBS: create all brims)
    /// C++: `static ExPolygons outer_inner_brim_area(const Print& print, const float no_brim_offset, std::map<ObjectID, ExPolygons>& brimAreaMap, std::map<ObjectID, ExPolygons>& supportBrimAreaMap, std::vector<std::pair<ObjectID, unsigned int>>& objPrintVec, std::vector<unsigned int>& printExtruders)`
    #[allow(clippy::too_many_arguments)]
    pub fn outer_inner_brim_area(
        print: &Print,
        no_brim_offset: f32,
        brim_area_map: &mut BTreeMap<ObjectID, ExPolygons>,
        support_brim_area_map: &mut BTreeMap<ObjectID, ExPolygons>,
        obj_print_vec: &[(ObjectID, u32)],
        print_extruders: &[u32],
        extruder_params: &BTreeMap<usize, ExtruderParams>,
        print_speed_map: &GlobalSpeedMap,
    ) -> FlowResult<ExPolygons> {
        // Brim.cpp:891
        let mut support_material_extruder: u32 = print_extruders[0] + 1;

        // Brim.cpp:893-895
        let mut brim_area: ExPolygons = Vec::new();
        let mut no_brim_area: ExPolygons = Vec::new();
        let mut holes: Polygons = Vec::new();

        // Brim.cpp:897-903
        let mut brim_to_write: BTreeMap<ObjectID, BrimWritten> = BTreeMap::new();
        for object_with_extruder in obj_print_vec {
            brim_to_write.entry(object_with_extruder.0).or_insert(BrimWritten { obj: true, sup: true });
        }

        // Brim.cpp:905, 918
        let mut object_islands: ExPolygons = Vec::new();
        let scaled_flow_width: f32 = print.brim_flow()?.scaled_spacing() as f32;
        // Brim.cpp:919-1094
        for &extruder_no in print_extruders {
            let extruder_no = extruder_no + 1;
            for object_with_extruder in obj_print_vec {
                let object = get_object(print, object_with_extruder.0).expect("print object of the ID");
                let brim_type = object_config_int(object, "brim_type");
                let brim_offset: f32 = scale_(object_config_float(object, "brim_object_gap")) as f32;
                // The brim width is rounded to whole loops of the flow makeBrimInfill() lays them with, otherwise
                // the leftover opens a gap where the loops offset from both sides of the area meet.
                let object_flow: Flow = print.multi_diameter_brim_flow(object, object_with_extruder.1)?;
                let object_scaled_flow_width: f32 = object_flow.scaled_spacing() as f32;
                let flow_width: f64 = object_flow.scaled_spacing() as f64 * SCALING_FACTOR;
                let mut brim_width: f32 = scale_((object_config_float(object, "brim_width") / flow_width / 2.0).floor() * flow_width * 2.0) as f32;
                let _scaled_additional_brim_width: f32 = scale_((5.0 / flow_width / 2.0).floor() * flow_width * 2.0) as f32;
                let _scaled_half_min_adh_length: f32 = scale_(1.1) as f32;
                let has_brim_auto = brim_type == BT_AUTO_BRIM;
                let use_brim_ears = brim_type == BT_BRIM_EARS;
                // if (object->model_object()->brim_points.size()>0 && has_brim_auto)
                //     use_brim_ears = true;
                let has_inner_brim = brim_type == BT_INNER_ONLY || brim_type == BT_OUTER_AND_INNER || use_brim_ears;
                let has_outer_brim = brim_type == BT_OUTER_ONLY || brim_type == BT_OUTER_AND_INNER || brim_type == BT_AUTO_BRIM || use_brim_ears;

                let mut brim_area_object: ExPolygons = Vec::new();
                let mut no_brim_area_object: ExPolygons = Vec::new();
                let brim_area_support: ExPolygons = Vec::new();
                let mut no_brim_area_support: ExPolygons = Vec::new();
                let mut holes_object: Polygons = Vec::new();
                let holes_support: Polygons = Vec::new();
                // Brim.cpp:946-1066
                if object_with_extruder.1 == extruder_no && brim_to_write[&object.id()].obj {
                    let _delta_t: f64 = get_temperature_from_extruder(print, object);
                    let adhension: f64 = getadhesion_coeff(print, object, extruder_params);
                    let max_speed: f64 = Model::find_max_speed(object.model_object(print), print_speed_map);

                    //BBS: collect holes area which is used to limit the brim of inner island
                    let mut holes_area: Polygons = Vec::new();
                    for ex_poly in &object.layers()[0].lslices {
                        holes_area.extend(ex_poly.holes.iter().cloned());
                    }

                    // BBS: brims are generated by volume groups
                    for volume_group in object.firstLayerObjGroups() {
                        // if this object has raft only update no_brim_area_object
                        if object.has_raft() {
                            continue;
                        }
                        // find volumePtrs included in this group
                        //BBS: support shared object logic
                        let shared_object = match object.get_shared_object() {
                            Some(idx) => &print.objects()[idx],
                            None => object,
                        };
                        let shared_model_object = shared_object.model_object(print);
                        let mut group_volume_ptrs: Vec<&ModelVolume> = Vec::new();
                        for volume_id in &volume_group.volume_ids {
                            let mut current_model_volume_ptr: Option<&ModelVolume> = None;
                            for volume_ptr in &shared_model_object.volumes {
                                if volume_ptr.id() == *volume_id {
                                    current_model_volume_ptr = Some(volume_ptr);
                                    break;
                                }
                            }
                            if let Some(volume_ptr) = current_model_volume_ptr {
                                group_volume_ptrs.push(volume_ptr);
                            }
                        }
                        if group_volume_ptrs.is_empty() {
                            continue;
                        }
                        let mut group_height: f64 = 0.0;
                        // config brim width in auto-brim mode
                        if has_brim_auto {
                            let brim_width_raw = config_brim_width_by_volume_groups(
                                adhension,
                                max_speed,
                                &group_volume_ptrs,
                                shared_model_object,
                                extruder_params,
                                &volume_group.slices,
                                &mut group_height,
                            );
                            brim_width = scale_((brim_width_raw / flow_width / 2.0).floor() * flow_width * 2.0) as f32;
                        }

                        for ex_poly in &volume_group.slices {
                            // BBS: additional brim width will be added if part's adhension area is too small and brim is not generated
                            // (Brim.cpp:987-993: the condition starts with `0 &&`, the width is never widened.)
                            let mut brim_width_mod: f32 = brim_width;
                            //BBS: brim width should be limited to the 1.5*boundingboxSize of a single polygon.
                            if has_brim_auto {
                                let bbox2 = BBox::from_points(&ex_poly.contour.points);
                                let (sx, sy) = (bbox2.max.x - bbox2.min.x, bbox2.max.y - bbox2.min.y);
                                let m = (if sx < sy { sy } else { sx }) as f32;
                                brim_width_mod = if m < brim_width_mod { m } else { brim_width_mod };
                            }
                            brim_width_mod = (brim_width_mod / object_scaled_flow_width / 2.0).floor() * object_scaled_flow_width * 2.0;

                            let mut ex_poly_holes_reversed: Polygons = ex_poly.holes.clone();
                            polygons_reverse(&mut ex_poly_holes_reversed);

                            if has_outer_brim {
                                // BBS: to find whether an island is in a hole of its object
                                let contour_hole_index = inner_island_hole_index(&holes_area, &ex_poly.contour);

                                // BBS: inner and outer boundary are offset from the same polygon incase of round off error.
                                let inner_expoly = offset_ex_polygon(&ex_poly.contour, brim_offset, JoinType::jtRound, SCALED_RESOLUTION);
                                let outer_expoly: ExPolygons = if use_brim_ears {
                                    make_brim_ears(print, object, flow_width, brim_offset, &object_flow, true)
                                    //outerExpoly = offset_ex(outerExpoly, brim_width_mod, jtRound, SCALED_RESOLUTION);
                                } else {
                                    cu::offset_ex_expolygons(&inner_expoly, brim_width_mod, JoinType::jtRound, SCALED_RESOLUTION)
                                };

                                if contour_hole_index < 0 {
                                    brim_area_object.extend(diff_ex_ee(&outer_expoly, &inner_expoly));
                                } else {
                                    let brim_before_clip = diff_ex_ee(&outer_expoly, &inner_expoly);

                                    // BBS: an island's brim should not be outside of its belonging hole
                                    let selected_hole: Polygons = vec![holes_area[contour_hole_index as usize].clone()];
                                    let clipped_brim = intersection_ex_ep(&brim_before_clip, &selected_hole);
                                    brim_area_object.extend(clipped_brim);
                                }
                            }
                            if has_inner_brim {
                                let inner_expoly = cu::offset_ex(&ex_poly_holes_reversed, -brim_width - brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
                                let outer_expoly: ExPolygons = if use_brim_ears {
                                    make_brim_ears(print, object, flow_width, brim_offset, &object_flow, false)
                                } else {
                                    cu::offset_ex(&ex_poly_holes_reversed, -brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT)
                                };
                                brim_area_object.extend(diff_ex_ee(&outer_expoly, &inner_expoly));
                            }
                            if !has_inner_brim {
                                // BBS: brim should be apart from holes
                                no_brim_area_object.extend(diff_ex_pe(
                                    &ex_poly_holes_reversed,
                                    &cu::offset_ex(&ex_poly_holes_reversed, (-scale_(5.0)) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                ));
                            }
                            if !has_outer_brim {
                                no_brim_area_object.extend(cu::diff_ex(
                                    &cu::offset_polygon(&ex_poly.contour, no_brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                    &ex_poly_holes_reversed,
                                    ApplySafetyOffset::No,
                                ));
                            }
                            if !has_inner_brim && !has_outer_brim {
                                no_brim_area_object.extend(diff_ex_pe(
                                    &ex_poly_holes_reversed,
                                    &cu::offset_ex(&ex_poly_holes_reversed, -no_brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                ));
                            }
                            holes_object.extend(ex_poly_holes_reversed);
                        }
                    }
                    // Brim.cpp:1052-1053
                    let object_island = cu::offset_ex_expolygons(&object.layers()[0].lslices, brim_offset, JoinType::jtRound, SCALED_RESOLUTION);
                    no_brim_area_object.extend(object_island.iter().cloned());

                    // Brim.cpp:1055-1065
                    brim_to_write.get_mut(&object.id()).unwrap().obj = false;
                    for instance in object.instances() {
                        if !brim_area_object.is_empty() {
                            append_and_translate_map(&mut brim_area, &brim_area_object, instance, print, brim_area_map);
                        }
                        append_and_translate(&mut no_brim_area, &no_brim_area_object, instance, print);
                        append_and_translate_polygons(&mut holes, &holes_object, instance, print);
                        append_and_translate(&mut object_islands, &object_island, instance, print);
                    }
                    if let Some(area) = brim_area_map.get(&object.id()) {
                        brim_area.extend(area.iter().cloned());
                    }
                }
                // Brim.cpp:1067-1073
                support_material_extruder = object_config_int(object, "support_filament") as u32;
                if support_material_extruder == 0 && object.has_support_material() {
                    if print_config_int(print, "print_sequence") == crate::print_config::enums::PrintSequence::ByObject as i32 {
                        support_material_extruder = object_with_extruder.1;
                    } else {
                        support_material_extruder = print_extruders[0] + 1;
                    }
                }
                // With multiple nozzle diameters a coarse support filament reaches the bed from a band top, so it is not
                // among the first layer filaments looped over here. Only the support islands are collected in this
                // branch, keep them off the brim whatever filament prints them.
                let support_filament_visited = support_material_extruder == extruder_no || print.uses_multiple_nozzle_diameters();
                // Brim.cpp:1079-1092
                if support_filament_visited && brim_to_write[&object.id()].sup {
                    no_brim_area_support.extend(bed_support_islands(print, object));

                    brim_to_write.get_mut(&object.id()).unwrap().sup = false;
                    for instance in object.instances() {
                        if !brim_area_support.is_empty() {
                            append_and_translate_map(&mut brim_area, &brim_area_support, instance, print, support_brim_area_map);
                        }
                        append_and_translate(&mut no_brim_area, &no_brim_area_support, instance, print);
                        append_and_translate_polygons(&mut holes, &holes_support, instance, print);
                    }
                    if let Some(area) = support_brim_area_map.get(&object.id()) {
                        brim_area.extend(area.iter().cloned());
                    }
                }
            }
        }
        let _ = (support_material_extruder, &holes);

        // Brim.cpp:1096-1104
        if print.has_wipe_tower() && !print.get_fake_wipe_tower().outer_wall.is_empty() {
            let tower = print.get_fake_wipe_tower();
            let mut expoly_from_lines: ExPolygons = Vec::new();
            for polyline in tower.outer_wall.iter().next().unwrap().1 {
                let mut polyline: Polyline = polyline.clone();
                crate::multi_point::remove_duplicate_points(&mut polyline.points);
                expoly_from_lines.push(ExPolygon { contour: Polygon::from_points(polyline.points.clone()), holes: Vec::new() });
                // `Point(scale_(pos[0]), scale_(pos[1]))`: Point(double, double) rounds (Point.hpp:181).
                expoly_from_lines
                    .last_mut()
                    .unwrap()
                    .translate(Point::new(scale_(tower.pos[0] as f64).round_ties_even() as i64, scale_(tower.pos[1] as f64).round_ties_even() as i64));
            }
            no_brim_area.extend(expoly_from_lines);
        }

        // Brim.cpp:1106-1116
        {
            let excl_pts = print_config_points(print, "bed_exclude_area");
            let mut exclude_poly = Polygon::default();
            for i in 0..excl_pts.len() {
                exclude_poly.points.push(Point::new(scale_(excl_pts[i].x).round_ties_even() as i64, scale_(excl_pts[i].y).round_ties_even() as i64));
                if i % 4 == 3 {
                    // exclude areas are always rectangles (4 points each)
                    no_brim_area.push(ExPolygon { contour: std::mem::take(&mut exclude_poly), holes: Vec::new() });
                    exclude_poly.points.clear();
                }
            }
        }

        // Brim.cpp:1118-1119
        if print_config_bool(print, "enable_wrapping_detection") && print_config_points(print, "wrapping_exclude_area").len() > 2 {
            no_brim_area.push(expolygon_of_contour(&polygon_new_scale(&print_config_points(print, "wrapping_exclude_area"))));
        }

        // Build per-physical-extruder "outside printable" polygons.
        // Each extruder's brim must not go outside that extruder's own printable area.
        // This is done per-object so that the left head can still brim in its own
        // exclusive zone (which the right head cannot reach), and vice versa.
        // bed_exclude_area and wrapping_exclude_area are already in no_brim_area above
        // and remain global (apply to all heads equally).
        // Brim.cpp:1127-1138
        let extruder_printable_polys: Vec<Polygons> = print.get_extruder_printable_polygons();
        let bed_poly: Polygons = vec![polygon_new_scale(&print_config_points(print, "printable_area"))];

        let nozzle_count = print.config().option("nozzle_diameter").and_then(|o| o.as_vector_base()).map(|v| v.size()).unwrap_or(0) as i32;
        let extruder_count: i32 = if nozzle_count < 1 { 1 } else { nozzle_count };
        let mut outside_printable_area_by_extruder: Vec<ExPolygons> = vec![Vec::new(); extruder_count as usize];
        for physical_ext in 0..extruder_count {
            let printable_area: &Polygons =
                if physical_ext < extruder_printable_polys.len() as i32 && !extruder_printable_polys[physical_ext as usize].is_empty() {
                    &extruder_printable_polys[physical_ext as usize]
                } else {
                    &bed_poly
                };
            if !printable_area.is_empty() {
                outside_printable_area_by_extruder[physical_ext as usize] = cu::diff_ex(
                    &cu::offset(printable_area, scale_(30.0) as f32, JoinType::jtRound, SCALED_RESOLUTION),
                    printable_area,
                    ApplySafetyOffset::No,
                );
            }
        }

        // Map ObjectID -> all 0-based physical extruder IDs using objPrintVec (filament id, 1-based)
        // and the layered nozzle grouping result.
        // Use get_nozzles_for_filament (not get_first_nozzle_for_filament) so that in sequential
        // (ByObject) printing, where the same filament may be routed to different nozzles on different
        // objects/layers, all possible nozzles are considered.  The outside areas of all involved
        // nozzles are unioned, which is equivalent to restricting brim to the intersection of their
        // printable areas – sacrificing some exclusive-zone area to guarantee correctness.
        // Brim.cpp:1147-1158
        let nozzle_group_result = print.get_nozzle_group_result();
        let get_physical_extruders = |filament_id_1based: u32| -> Vec<i32> {
            let filament_id = filament_id_1based as i32 - 1;
            let mut result: Vec<i32> = Vec::new();
            if let Some(group) = &nozzle_group_result {
                for nozzle in group.get_nozzles_for_filament(filament_id) {
                    result.push(nozzle.extruder_id);
                }
            }
            if result.is_empty() {
                result.push(0);
            }
            result
        };

        // Build ObjectID -> physical extruder list lookup from objPrintVec
        // Brim.cpp:1161-1163
        let mut obj_physical_extruders: BTreeMap<ObjectID, Vec<i32>> = BTreeMap::new();
        for pair in obj_print_vec {
            obj_physical_extruders.insert(pair.0, get_physical_extruders(pair.1));
        }

        // Brim.cpp:1165
        no_brim_area = cu::offset2_ex(&no_brim_area, scaled_flow_width, -scaled_flow_width, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);

        // Brim.cpp:1167-1181
        for object in print.objects() {
            let physical_exts: Vec<i32> = match obj_physical_extruders.get(&object.id()) {
                Some(v) => v.clone(),
                None => vec![0],
            };

            // Combine global no_brim with outside areas of all nozzles this filament may use.
            // Union of outside areas == outside of intersection of printable areas.
            let mut obj_no_brim: ExPolygons = no_brim_area.clone();
            for ext in physical_exts {
                if ext >= 0 && ext < outside_printable_area_by_extruder.len() as i32 {
                    obj_no_brim.extend(outside_printable_area_by_extruder[ext as usize].iter().cloned());
                }
            }

            if let Some(area) = brim_area_map.get_mut(&object.id()) {
                *area = diff_ex_ee(area, &obj_no_brim);
            }
        }

        // Brim.cpp:1183-1224
        brim_area.clear();
        let brim_flow_scaled_spacing = print.brim_flow()?.scaled_spacing();
        for object in print.objects() {
            // BBS: brim should be contacted to at least one object's island or brim area
            if brim_area_map.contains_key(&object.id()) {
                // find other objects' brim area
                let mut other_ex_polys: ExPolygons = Vec::new();
                for other_object in print.objects() {
                    if other_object.id() != object.id() {
                        if let Some(area) = brim_area_map.get(&other_object.id()) {
                            other_ex_polys.extend(area.iter().cloned());
                        }
                    }
                }

                let temp_areas: ExPolygons = std::mem::take(brim_area_map.get_mut(&object.id()).unwrap());

                // Brim.cpp:1200-1216: a tbb::parallel_for that pushes the indices of the kept areas under a mutex;
                // on one thread the indices come in ascending order, which is the order ported.
                let mut retained: Vec<usize> = Vec::new();
                for ia in 0..temp_areas.len() {
                    let offseted_ta = cu_expolygon::offset_ex(&temp_areas[ia], (brim_flow_scaled_spacing * 2) as f32, JoinType::jtRound, SCALED_RESOLUTION);
                    if cu_expolygon::overlaps_expolygons(&offseted_ta, &object_islands) || cu_expolygon::overlaps_expolygons(&offseted_ta, &other_ex_polys) {
                        retained.push(ia);
                    }
                }

                for &index in &retained {
                    brim_area_map.get_mut(&object.id()).unwrap().push(temp_areas[index].clone());
                    brim_area.push(temp_areas[index].clone());
                }
            }
        }
        Ok(brim_area)
    }

    /// Brim.cpp:1243-1319
    /// C++: `static Polylines connect_brim_lines(Polylines &&polylines, const Polygons &brim_area, float max_connection_length)`
    pub fn connect_brim_lines(mut polylines: Polylines, brim_area: &[Polygon], max_connection_length: f32) -> Polylines {
        // Brim.cpp:1245-1246
        if polylines.is_empty() {
            return Vec::new();
        }

        // Brim.cpp:1248-1249
        let mut bbox: BBox = get_extents_polylines_bbox(&polylines);
        bbox.merge(&get_extents_polygons_bbox(brim_area));

        // Brim.cpp:1251-1252 — `EdgeGrid::Grid grid(bbox.inflated(SCALED_EPSILON))`: the grid merges the points of
        // its contours into the box it was given (EdgeGrid.cpp:142-157); a box that is not defined starts from them.
        let mut grid = crate::edge_grid::EdgeGrid::new();
        let inflated = bbox.inflated(scale_(EPSILON));
        if inflated.defined {
            grid.set_bbox(inflated.to_bounding_box());
        }
        grid.create_from_mixed(brim_area, &polylines, scale_(10.0) as i64);

        // Brim.cpp:1254-1279, `struct Visitor`: whether the brim line crosses a segment of the cells it runs through.
        let brim_line_intersects = |brim_line: &Line| -> bool {
            let mut intersect = false;
            grid.visit_cells_intersecting_line(brim_line.a, brim_line.b, |iy: usize, ix: usize| -> bool {
                // Called with a row and colum of the grid cell, which is intersected by a line.
                intersect = false;
                for &contour_and_segment in grid.cell_data_range_at(iy, ix) {
                    // End points of the line segment and their vector.
                    let segment = grid.segment(contour_and_segment);
                    if crate::geometry::segments_intersect(segment.a, segment.b, brim_line.a, brim_line.b) {
                        intersect = true;
                        return false;
                    }
                }
                // Continue traversing the grid along the edge.
                true
            });
            intersect
        };

        // Connect successive polylines if they are open, their ends are closer than max_connection_length.
        // Remove empty polylines.
        {
            // Skip initial empty lines.
            // Brim.cpp:1285-1288
            let mut poly_idx: usize = 0;
            while poly_idx < polylines.len() && polylines[poly_idx].points.is_empty() {
                poly_idx += 1;
            }
            poly_idx += 1;
            let mut end: usize = poly_idx;
            // `Slic3r::sqr(max_connection_length)`: the square of a float, as a float.
            let max_connection_length2: f64 = (max_connection_length * max_connection_length) as f64;
            // Brim.cpp:1289-1313
            while poly_idx < polylines.len() {
                if !polylines[poly_idx].points.is_empty() {
                    let mut connect = false;
                    let prev_closed = polylines[end - 1].is_closed();
                    let next_closed = polylines[poly_idx].is_closed();
                    if !prev_closed && !next_closed {
                        let prev_last = polylines[end - 1].last_point();
                        let next_first = polylines[poly_idx].first_point();
                        let (dx, dy) = ((prev_last.x - next_first.x) as f64, (prev_last.y - next_first.y) as f64);
                        let dist2 = dx * dx + dy * dy;
                        if dist2 <= max_connection_length2 {
                            let mut brim_line = Line::new(prev_last, next_first);
                            // Shrink the connection line to avoid collisions with the brim centerlines.
                            brim_line.extend(-scale_(EPSILON));
                            connect = !brim_line_intersects(&brim_line);
                        }
                    }
                    if connect {
                        let next_points = std::mem::take(&mut polylines[poly_idx].points);
                        polylines[end - 1].points.extend(next_points);
                    } else {
                        if end < poly_idx {
                            let next = std::mem::take(&mut polylines[poly_idx]);
                            polylines[end] = next;
                        }
                        end += 1;
                    }
                }
                poly_idx += 1;
            }
            // Brim.cpp:1314-1315
            if end < polylines.len() {
                polylines.truncate(end);
            }
        }

        polylines
    }

    /// Brim.cpp:1328-1339, 1448-1457, the lambda `save_polygon_if_is_inner_island` of the two overloads of
    /// `make_inner_island_brim`: the contour is simplified and filed under the first hole it lies in.
    fn save_polygon_if_is_inner_island(holes_area: &Polygons, contour: &mut Polygon, hole_island_pair: &mut BTreeMap<usize, Polygons>, scaled_resolution: f64) {
        for i in 0..holes_area.len() {
            let contour_polys: Polygons = vec![contour.clone()];
            if cu::diff_ex(&contour_polys, std::slice::from_ref(&holes_area[i]), ApplySafetyOffset::No).is_empty() {
                // BBS: this is an inner island inside holes_area[i], save
                contour.douglas_peucker(scaled_resolution);
                hole_island_pair.entry(i).or_default().push(contour.clone());
                break;
            }
        }
    }

    /// Brim.cpp:1397-1407, 1733-1744, 1863-1873: the loops as polylines, each clipped by `clip` when there is
    /// one and chained, then all of them in one list. The C++ runs the loops through a `tbb::parallel_for`;
    /// every loop writes its own slot.
    fn loops_to_chained_polylines(loops: &Polygons, clip: Option<&Polygons>) -> Polylines {
        let loops_pl: Polylines = cu_polygon::to_polylines(loops);
        let mut loops_pl_by_levels: Vec<Polylines> = Vec::with_capacity(loops_pl.len());
        for loop_pl in loops_pl {
            let single: Polylines = vec![loop_pl];
            loops_pl_by_levels.push(match clip {
                Some(area) => crate::shortest_path::chain_polylines(cu::intersection_pl(&single, area), None),
                None => crate::shortest_path::chain_polylines(single, None),
            });
        }
        // Reduce down to the ordered list of polylines.
        let mut all_loops: Polylines = Vec::new();
        for polylines in loops_pl_by_levels {
            all_loops.extend(polylines);
        }
        all_loops
    }

    /// Brim.cpp:1321-1439
    /// C++: `static void make_inner_island_brim(const Print& print, const ConstPrintObjectPtrs& top_level_objects_with_brim, ExtrusionEntityCollection &brim, ExPolygons &islands_area_ex)`
    ///
    /// BBS: this function is used to generate brim for inner island inside holes.
    /// Collect island + brim area to be minused when generating inner brim for holes.
    pub fn make_inner_island_brim(
        print: &Print,
        top_level_objects_with_brim: &[usize],
        brim: &mut ExtrusionEntityCollection,
        islands_area_ex: &mut ExPolygons,
    ) -> FlowResult<()> {
        // Brim.cpp:1326
        let scaled_resolution = scaled_resolution_of(print);

        // Brim.cpp:1341
        let flow: Flow = print.brim_flow()?;
        let flow_scaled_spacing: f32 = flow.scaled_spacing() as f32;
        for &object_idx in top_level_objects_with_brim {
            let object = &print.objects()[object_idx];
            let brim_type = object_config_int(object, "brim_type");
            // BBS: don't need to handle this object if hasn't enabled outer_brim
            if brim_type == BT_NO_BRIM {
                continue;
            }

            //BBS: 1 collect holes area which is used to limit the brim of inner island
            let mut holes_area: Polygons = Vec::new();
            for ex_poly in &object.layers()[0].lslices {
                holes_area.extend(ex_poly.holes.iter().cloned());
            }

            //BBS: 2 get the island polygons inside holes, saved as map
            let mut hole_island_pair: BTreeMap<usize, Polygons> = BTreeMap::new();
            for ex_poly in &object.layers()[0].lslices {
                let mut counter: Polygon = ex_poly.contour.clone();
                save_polygon_if_is_inner_island(&holes_area, &mut counter, &mut hole_island_pair, scaled_resolution);
            }

            if !object.support_layers().is_empty() {
                for support_contour in &first_support_layer_contours(object) {
                    let mut counter: Polygon = support_contour.clone();
                    save_polygon_if_is_inner_island(&holes_area, &mut counter, &mut hole_island_pair, scaled_resolution);
                }
            }

            //BBS: 3 generate loops, only save part of loop which inside hole
            let brim_offset: f32 = scale_(object_config_float(object, "brim_object_gap")) as f32;
            let brim_width: f32 = scale_(object_config_float(object, "brim_width")) as f32;
            if brim_type == BT_INNER_ONLY {
                // If brim_type is btInnerOnly, we actually doesn't generate loops for inner island.
                // Only update islands_area_ex and return
                for (first, second) in &hole_island_pair {
                    let islands_area_ex_object = cu::intersection_ex(
                        &cu::offset(second, brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                        &cu::offset_polygon(&holes_area[*first], -brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                        ApplySafetyOffset::No,
                    );
                    for instance in object.instances() {
                        append_and_translate(islands_area_ex, &islands_area_ex_object, instance, print);
                    }
                }
            } else {
                // Brim.cpp:1381
                let num_loops: usize = (brim_width / flow_scaled_spacing).floor() as usize;
                for (first, second) in &hole_island_pair {
                    let mut loops: Polygons = Vec::new();
                    let inner_islands: Polygons = cu::offset(second, brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
                    //offset to keep away from hole
                    let brimable_area: Polygons = cu::offset_polygon(&holes_area[*first], -brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
                    let mut contour: Polygons = inner_islands.clone();
                    for _i in 0..num_loops {
                        contour = cu::offset(&contour, flow_scaled_spacing, JoinType::jtSquare, cu::DEFAULT_MITER_LIMIT);
                        for poly in &mut contour {
                            poly.douglas_peucker(scaled_resolution);
                        }
                        loops.extend(cu::offset(&contour, -0.5f32 * flow_scaled_spacing, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT));
                    }
                    // BBS: to be checked.
                    //loops = union_pt_chained_outside_in(loops, false);
                    loops = union_pt_chained_outside_in(&loops);

                    // BBS: Reduce down to the ordered list of polylines.
                    let mut all_loops_object: Polylines = loops_to_chained_polylines(&loops, Some(&brimable_area));

                    optimize_polylines_by_reversing(&mut all_loops_object);
                    all_loops_object = connect_brim_lines(
                        all_loops_object,
                        &cu::offset(&inner_islands, scale_(EPSILON) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                        flow_scaled_spacing * 2.0f32,
                    );

                    // Brim.cpp:1418-1426
                    let mut final_loops: Polylines = Vec::new();
                    for instance in object.instances() {
                        let mut dst_idx = final_loops.len();
                        final_loops.extend(all_loops_object.iter().cloned());
                        let instance_shift: Point = instance.shift_without_plate_offset(print);
                        while dst_idx < final_loops.len() {
                            final_loops[dst_idx].translate(instance_shift);
                            dst_idx += 1;
                        }
                    }
                    extrusion_entities_append_loops_and_paths(
                        &mut brim.entities,
                        final_loops,
                        ExtrusionRole::Brim,
                        flow.mm3_per_mm()? as f32 as f64,
                        flow.width() as f32,
                        print.skirt_first_layer_height() as f32,
                    );

                    //BBS: save all inner island and inner island brim area here, which is necesary if generate inner brim for holes
                    //Inner brim of holes must not occupy this area
                    let islands_area_ex_object = cu::intersection_ex(&contour, &brimable_area, ApplySafetyOffset::No);
                    for instance in object.instances() {
                        append_and_translate(islands_area_ex, &islands_area_ex_object, instance, print);
                    }
                }
            }
        }
        Ok(())
    }

    /// Brim.cpp:1441-1595, the overload of `make_inner_island_brim` that stores the areas by object and by
    /// extruder.
    pub mod make_inner_island_brim_by_object {
        use super::*;
        /// Brim.cpp:1441-1595 (BBS: the brim are generated one by one, and sorted by objs/supports and extruders)
        /// C++: `static void make_inner_island_brim(const Print& print, const ConstPrintObjectPtrs& top_level_objects_with_brim, std::map<ObjectID, ExPolygons>& innerbrimAreaMap, std::map<ObjectID, ExPolygons>& innerSupportBrimAreaMap, ExPolygons& islands_area_ex, ExPolygons& NobrimArea, std::vector<std::pair<ObjectID, unsigned int>>& objPrintVec)`
        #[allow(clippy::too_many_arguments)]
        pub fn make_inner_island_brim(
            print: &Print,
            top_level_objects_with_brim: &[usize],
            innerbrim_area_map: &mut BTreeMap<ObjectID, ExPolygons>,
            inner_support_brim_area_map: &mut BTreeMap<ObjectID, ExPolygons>,
            islands_area_ex: &mut ExPolygons,
            nobrim_area: &ExPolygons,
            obj_print_vec: &[(ObjectID, u32)],
        ) -> FlowResult<()> {
            // Brim.cpp:1459-1464
            let support_material_extruder = support_material_extruder_of(print, top_level_objects_with_brim);

            // Brim.cpp:1466-1469
            let mut top_level_objects_idx: HashSet<usize> = HashSet::with_capacity(top_level_objects_with_brim.len());
            for &object_idx in top_level_objects_with_brim {
                top_level_objects_idx.insert(print.objects()[object_idx].id().id);
            }

            // Brim.cpp:1471-1478
            let mut brim_to_write: BTreeMap<ObjectID, BrimWritten> = BTreeMap::new();
            for object_with_extruder in obj_print_vec {
                if top_level_objects_idx.contains(&object_with_extruder.0.id) {
                    brim_to_write.entry(object_with_extruder.0).or_insert(BrimWritten { obj: true, sup: true });
                }
            }

            // Brim.cpp:1480
            let flow: Flow = print.brim_flow()?;
            let flow_scaled_spacing: i64 = flow.scaled_spacing();
            // Brim.cpp:1481-1587
            for extruder_no in print.extruders(false) {
                let extruder_no = extruder_no + 1;
                for object_with_extruder in obj_print_vec {
                    if top_level_objects_idx.contains(&object_with_extruder.0.id) {
                        let object = get_object(print, object_with_extruder.0).expect("print object of the ID");
                        let brim_type = object_config_int(object, "brim_type");
                        // BBS: don't need to handle this object if hasn't enabled outer_brim
                        if brim_type == BT_NO_BRIM {
                            continue;
                        }

                        //BBS: 1 collect holes area which is used to limit the brim of inner island
                        let mut holes_area: Polygons = Vec::new();
                        for ex_poly in &object.layers()[0].lslices {
                            holes_area.extend(ex_poly.holes.iter().cloned());
                        }

                        //BBS: 2 get the island polygons inside holes, saved as map
                        let mut hole_island_pair: BTreeMap<usize, Polygons> = BTreeMap::new();
                        for ex_poly in &object.layers()[0].lslices {
                            let mut counter: Polygon = ex_poly.contour.clone();
                            save_polygon_if_is_inner_island(&holes_area, &mut counter, &mut hole_island_pair, SCALED_RESOLUTION);
                        }
                        let mut hole_island_pair_supports: BTreeMap<usize, Polygons> = BTreeMap::new();
                        if !object.support_layers().is_empty() {
                            for support_contour in &first_support_layer_contours(object) {
                                let mut counter: Polygon = support_contour.clone();
                                save_polygon_if_is_inner_island(&holes_area, &mut counter, &mut hole_island_pair_supports, SCALED_RESOLUTION);
                            }
                        }

                        //BBS: 3 generate loops, only save part of loop which inside hole
                        // Brim.cpp:1512-1513 — `floor(scale_(brim_width) / 2 / flow.scaled_spacing()) * 2 * flow.scaled_spacing()`
                        // in double, stored in a float.
                        let brim_offset: f32 = scale_(object_config_float(object, "brim_object_gap")) as f32;
                        let brim_width: f32 =
                            ((scale_(object_config_float(object, "brim_width")) / 2.0 / flow_scaled_spacing as f64).floor() * 2.0 * flow_scaled_spacing as f64) as f32;
                        // Brim.cpp:1514-1547
                        if object_with_extruder.1 == extruder_no && brim_to_write[&object.id()].obj {
                            if brim_type == BT_INNER_ONLY {
                                // If brim_type is btInnerOnly, we actually doesn't generate loops for inner island.
                                // Only update islands_area_ex and return
                                for (first, second) in &hole_island_pair {
                                    let islands_area_ex_object = cu::intersection_ex(
                                        &cu::offset(second, brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                        &cu::offset_polygon(&holes_area[*first], -brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                        ApplySafetyOffset::No,
                                    );
                                    for instance in object.instances() {
                                        append_and_translate(islands_area_ex, &islands_area_ex_object, instance, print);
                                    }
                                }
                                brim_to_write.get_mut(&object.id()).unwrap().obj = false;
                            } else {
                                for (first, second) in &hole_island_pair {
                                    let inner_islands: Polygons = cu::offset(second, brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
                                    //offset to keep away from hole
                                    let brimable_area: Polygons = cu::offset_polygon(&holes_area[*first], -brim_offset, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
                                    let mut contour: Polygons = cu::offset(&inner_islands, brim_offset + brim_width, JoinType::jtRound, SCALED_RESOLUTION);
                                    for poly in &mut contour {
                                        poly.douglas_peucker(SCALED_RESOLUTION);
                                    }

                                    //BBS: save all inner island and inner island brim area here, which is necesary if generate inner brim for holes
                                    //Inner brim of holes must not occupy this area
                                    let mut islands_area_ex_object = cu::intersection_ex(&contour, &brimable_area, ApplySafetyOffset::No);
                                    let inner_islands_exp = cu::offset_ex(&inner_islands, 0.0, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
                                    islands_area_ex_object = diff_ex_ee(&islands_area_ex_object, &inner_islands_exp);
                                    for instance in object.instances() {
                                        append_and_translate_map(islands_area_ex, &islands_area_ex_object, instance, print, innerbrim_area_map);
                                    }
                                }
                                brim_to_write.get_mut(&object.id()).unwrap().obj = false;
                            }
                            if let Some(area) = innerbrim_area_map.get(&object.id()) {
                                islands_area_ex.extend(area.iter().cloned());
                            }
                        }

                        // Brim.cpp:1550-1584
                        if support_material_extruder == extruder_no && brim_to_write[&object.id()].sup {
                            if brim_type == BT_INNER_ONLY {
                                // If brim_type is btInnerOnly, we actually doesn't generate loops for inner island.
                                // Only update islands_area_ex and return
                                for (first, second) in &hole_island_pair_supports {
                                    let islands_area_ex_support = cu::intersection_ex(
                                        &cu::offset(second, 0.0, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                        &cu::offset_polygon(&holes_area[*first], 0.0, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
                                        ApplySafetyOffset::No,
                                    );
                                    for instance in object.instances() {
                                        append_and_translate(islands_area_ex, &islands_area_ex_support, instance, print);
                                    }
                                }
                                brim_to_write.get_mut(&object.id()).unwrap().sup = false;
                            } else {
                                for (first, second) in &hole_island_pair_supports {
                                    let inner_islands: Polygons = cu::offset(second, 0.0, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
                                    //offset to keep away from hole
                                    let brimable_area: Polygons =
                                        cu::offset_polygon(&holes_area[*first], -(flow_scaled_spacing as f32), cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
                                    let mut contour: Polygons = cu::offset(&inner_islands, brim_width, JoinType::jtRound, SCALED_RESOLUTION);
                                    for poly in &mut contour {
                                        poly.douglas_peucker(SCALED_RESOLUTION);
                                    }

                                    //BBS: save all inner island and inner island brim area here, which is necesary if generate inner brim for holes
                                    //Inner brim of holes must not occupy this area
                                    let mut islands_area_ex_support = cu::intersection_ex(&contour, &brimable_area, ApplySafetyOffset::No);
                                    let inner_islands_exp = cu::offset_ex(&inner_islands, 0.0, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
                                    islands_area_ex_support = diff_ex_ee(&islands_area_ex_support, &inner_islands_exp);
                                    for instance in object.instances() {
                                        append_and_translate_map(islands_area_ex, &islands_area_ex_support, instance, print, inner_support_brim_area_map);
                                    }
                                }
                                brim_to_write.get_mut(&object.id()).unwrap().sup = false;
                            }
                            if let Some(area) = inner_support_brim_area_map.get(&object.id()) {
                                islands_area_ex.extend(area.iter().cloned());
                            }
                        }
                    }
                }
            }
            // Brim.cpp:1588-1594
            *islands_area_ex = diff_ex_ee(islands_area_ex, nobrim_area);
            for object in print.objects() {
                if let Some(area) = innerbrim_area_map.get_mut(&object.id()) {
                    *area = diff_ex_ee(area, nobrim_area);
                }
                if let Some(area) = inner_support_brim_area_map.get_mut(&object.id()) {
                    *area = diff_ex_ee(area, nobrim_area);
                }
            }
            Ok(())
        }
    }
    pub use make_inner_island_brim_by_object::make_inner_island_brim as make_inner_island_brim_map;

    /// Brim.cpp:1596-1643
    /// C++: `static void make_inner_brim(const Print &print, const ConstPrintObjectPtrs &top_level_objects_with_brim, const std::vector<ExPolygons> &bottom_layers_expolygons, ExtrusionEntityCollection &brim)`
    pub fn make_inner_brim(
        print: &Print,
        top_level_objects_with_brim: &[usize],
        bottom_layers_expolygons: &[ExPolygons],
        brim: &mut ExtrusionEntityCollection,
    ) -> FlowResult<()> {
        // Brim.cpp:1601-1602
        debug_assert!(print.objects().len() == bottom_layers_expolygons.len());
        let scaled_resolution = scaled_resolution_of(print);

        //BBS: generate brim for inner island first
        // Brim.cpp:1605-1606
        let mut inner_islands_ex: ExPolygons = Vec::new();
        make_inner_island_brim(print, top_level_objects_with_brim, brim, &mut inner_islands_ex)?;

        // Brim.cpp:1622-1627
        let flow: Flow = print.brim_flow()?;
        let flow_scaled_spacing: f32 = flow.scaled_spacing() as f32;
        let mut islands_ex: ExPolygons = inner_brim_area(print, top_level_objects_with_brim, bottom_layers_expolygons, flow_scaled_spacing);
        //BBS: brim of hole must not overlap with inner island and inner island brim
        if !inner_islands_ex.is_empty() {
            islands_ex = diff_ex_ee(&islands_ex, &inner_islands_ex);
        }

        // Brim.cpp:1629-1637
        let mut loops: Polygons = Vec::new();
        // jtSquare seems not working when expandign the holes
        islands_ex = cu::offset_ex_expolygons(&islands_ex, -0.5f32 * flow_scaled_spacing, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
        while !islands_ex.is_empty() {
            for poly_ex in &mut islands_ex {
                expolygon_douglas_peucker(poly_ex, scaled_resolution);
            }
            // jtSquare seems not working when expandign the holes
            loops.extend(to_polygons(&islands_ex));
            islands_ex = cu::offset_ex_expolygons(&islands_ex, -1.3f32 * flow_scaled_spacing, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
            islands_ex = cu::offset_ex_expolygons(&islands_ex, 0.3f32 * flow_scaled_spacing, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);
        }

        // Brim.cpp:1639-1642
        loops = union_pt_chained_outside_in(&loops);
        loops.reverse();
        extrusion_entities_append_loops(
            &mut brim.entities,
            loops,
            ExtrusionRole::Brim,
            flow.mm3_per_mm()? as f32 as f64,
            flow.width() as f32,
            print.skirt_first_layer_height() as f32,
        );
        Ok(())
    }

    /// Brim.cpp:1645-1702, the overload of `make_inner_brim` that stores the areas by object.
    pub mod make_inner_brim_by_object {
        use super::*;
        /// Brim.cpp:1645-1702 (BBS: generate inner brim by objs)
        /// C++: `static void make_inner_brim(const Print& print, const ConstPrintObjectPtrs& top_level_objects_with_brim, std::map<ObjectID, ExPolygons>& brimAreaMap, std::map<ObjectID, ExPolygons>& supportBrimAreaMap, std::vector<std::pair<ObjectID, unsigned int>>& objPrintVec)`
        pub fn make_inner_brim(
            print: &Print,
            top_level_objects_with_brim: &[usize],
            brim_area_map: &mut BTreeMap<ObjectID, ExPolygons>,
            support_brim_area_map: &mut BTreeMap<ObjectID, ExPolygons>,
            obj_print_vec: &[(ObjectID, u32)],
        ) -> FlowResult<()> {
            //BBS: generate brim for inner island first
            // Brim.cpp:1667-1669
            let flow: Flow = print.brim_flow()?;
            let _no_brim: ExPolygons =
                inner_brim_area_map(print, top_level_objects_with_brim, flow.scaled_spacing() as f32, brim_area_map, support_brim_area_map, obj_print_vec);

            // Brim.cpp:1671-1675 — the call of make_inner_island_brim is commented out in the C++, the maps stay empty.
            let inner_islands_ex: ExPolygons = Vec::new();
            let inner_brim_area_map_: BTreeMap<ObjectID, ExPolygons> = BTreeMap::new();
            let inner_support_brim_area_map: BTreeMap<ObjectID, ExPolygons> = BTreeMap::new();
            /*make_inner_island_brim(print, top_level_objects_with_brim, innerBrimAreaMap, innerSupportBrimAreaMap,
                inner_islands_ex, NoBrim, objPrintVec);*/

            //BBS: brim of hole must not overlap with inner island and inner island brim
            // Brim.cpp:1678-1701
            if !inner_islands_ex.is_empty() {
                if !brim_area_map.is_empty() {
                    for (_, second) in brim_area_map.iter_mut() {
                        if !second.is_empty() {
                            *second = diff_ex_ee(second, &inner_islands_ex);
                        }
                    }
                }
                if !support_brim_area_map.is_empty() {
                    for (_, second) in support_brim_area_map.iter_mut() {
                        if !second.is_empty() {
                            *second = diff_ex_ee(second, &inner_islands_ex);
                        }
                    }
                }
                for object in print.objects() {
                    if let Some(area) = inner_brim_area_map_.get(&object.id()) {
                        brim_area_map.entry(object.id()).or_default().extend(area.iter().cloned());
                    }
                    if let Some(area) = inner_support_brim_area_map.get(&object.id()) {
                        support_brim_area_map.entry(object.id()).or_default().extend(area.iter().cloned());
                    }
                }
            }
            Ok(())
        }
    }
    pub use make_inner_brim_by_object::make_inner_brim as make_inner_brim_map;

    /// Brim.cpp:1705-1727 (BBS: generate out brim by offseting ExPolygons 'islands_area_ex')
    /// C++: `Polygons tryExPolygonOffset(const ExPolygons islandAreaEx, const Print& print, const Flow& flow)`
    pub fn try_ex_polygon_offset(island_area_ex: &ExPolygons, print: &Print, flow: &Flow) -> Polygons {
        // Brim.cpp:1708-1710
        let _scaled_resolution = scaled_resolution_of(print);
        let mut loops: Polygons = Vec::new();

        // Brim.cpp:1712-1716
        let resolution: f64 = 0.0125 / SCALING_FACTOR;
        let flow_scaled_spacing: f32 = flow.scaled_spacing() as f32;
        let mut islands_ex: ExPolygons = island_area_ex.clone();
        for poly_ex in &mut islands_ex {
            expolygon_douglas_peucker(poly_ex, resolution);
        }
        islands_ex = cu::offset_ex_expolygons(&islands_ex, -0.5f32 * flow_scaled_spacing, JoinType::jtRound, resolution);
        // Brim.cpp:1717-1725
        while !islands_ex.is_empty() {
            for poly_ex in &mut islands_ex {
                expolygon_douglas_peucker(poly_ex, resolution);
            }
            loops.extend(to_polygons(&islands_ex));
            islands_ex = cu::offset_ex_expolygons(&islands_ex, -1.3f32 * flow_scaled_spacing, JoinType::jtRound, resolution);
            for poly_ex in &mut islands_ex {
                expolygon_douglas_peucker(poly_ex, resolution);
            }
            islands_ex = cu::offset_ex_expolygons(&islands_ex, 0.3f32 * flow_scaled_spacing, JoinType::jtRound, resolution);
        }
        loops
    }

    /// Brim.cpp:1728-1766 (BBS: a function creates the ExtrusionEntityCollection from the brim area defined by ExPolygons)
    /// C++: `ExtrusionEntityCollection makeBrimInfill(const ExPolygons& singleBrimArea, const Print& print, const Polygons& islands_area, const Flow& flow)`
    pub fn make_brim_infill(single_brim_area: &ExPolygons, print: &Print, _islands_area: &Polygons, flow: &Flow) -> FlowResult<ExtrusionEntityCollection> {
        // Brim.cpp:1730-1731
        let mut loops: Polygons = try_ex_polygon_offset(single_brim_area, print, flow);
        loops = union_pt_chained_outside_in(&loops);

        // Brim.cpp:1733-1752 — `chain_polylines({ std::move(loops_pl[i]) })`, the loops are not clipped.
        // output
        let mut brim = ExtrusionEntityCollection::new();
        // Reduce down to the ordered list of polylines.
        let mut all_loops: Polylines = loops_to_chained_polylines(&loops, None);

        // Flip orientation of open polylines to minimize travel distance.
        // Brim.cpp:1755-1756
        optimize_polylines_by_reversing(&mut all_loops);
        all_loops = connect_brim_lines(
            all_loops,
            &cu::offset_expolygons(single_brim_area, scale_(EPSILON) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
            flow.scaled_spacing() as f32 * 2.0f32,
        );

        //BBS: finally apply the plate offset which may very large
        // Brim.cpp:1759-1762 — `scaled(plate_offset.x())` truncates (Point.hpp:536-541).
        let plate_offset = print.get_plate_origin();
        let scaled_plate_offset = Point::new(scale_(plate_offset[0]) as i64, scale_(plate_offset[1]) as i64);
        for one_loop in &mut all_loops {
            one_loop.translate(scaled_plate_offset);
        }

        // Brim.cpp:1764
        extrusion_entities_append_loops_and_paths(
            &mut brim.entities,
            all_loops,
            ExtrusionRole::Brim,
            flow.mm3_per_mm()? as f32 as f64,
            flow.width() as f32,
            flow.height() as f32,
        );
        Ok(brim)
    }

    /// Brim.cpp:1768-1841, the overload of `make_brim` that generates the brim by object and by extruder.
    pub mod make_brim_by_object {
        use super::*;
        /// Brim.cpp:1768-1841 (BBS: an overload of the orignal brim generator that generates the brim by obj and by extruders)
        /// C++: `void make_brim(const Print& print, PrintTryCancel try_cancel, Polygons& islands_area, std::map<ObjectID, ExtrusionEntityCollection>& brimMap, std::map<ObjectID, ExtrusionEntityCollection>& supportBrimMap, std::vector<std::pair<ObjectID, unsigned int>> &objPrintVec, std::vector<unsigned int>& printExtruders)`
        ///
        /// The C++ takes the Print by const reference and writes `firstLayerObjectBrimBoundingBox` of its objects
        /// through a `const_cast`; here the Print is mutable.
        #[allow(clippy::too_many_arguments)]
        pub fn make_brim(
            print: &mut Print,
            _try_cancel: Option<&dyn Fn()>,
            islands_area: &mut Polygons,
            brim_map: &mut BTreeMap<ObjectID, ExtrusionEntityCollection>,
            support_brim_map: &mut BTreeMap<ObjectID, ExtrusionEntityCollection>,
            obj_print_vec: &[(ObjectID, u32)],
            print_extruders: &[u32],
            extruder_params: &BTreeMap<usize, ExtruderParams>,
            print_speed_map: &GlobalSpeedMap,
        ) -> FlowResult<()> {
            // Brim.cpp:1776-1783
            let _brim_width_max: f64 = 0.0;
            let mut brim_area_map: BTreeMap<ObjectID, ExPolygons> = BTreeMap::new();
            let mut support_brim_area_map: BTreeMap<ObjectID, ExPolygons> = BTreeMap::new();
            let flow: Flow = print.brim_flow()?;
            let _scaled_resolution = scaled_resolution_of(print);
            let islands_area_ex: ExPolygons = outer_inner_brim_area(
                print,
                flow.scaled_spacing() as f32,
                &mut brim_area_map,
                &mut support_brim_area_map,
                obj_print_vec,
                print_extruders,
                extruder_params,
                print_speed_map,
            )?;

            // BBS: Find boundingbox of the first layer
            // Brim.cpp:1786-1811
            for print_obj_id in print.print_object_ids() {
                let mut bbx = BBox::new();
                let object_idx = print.objects().iter().position(|obj| obj.id() == print_obj_id).expect("print object of the ID");
                {
                    let object = &print.objects()[object_idx];
                    for ex_poly in &object.layers()[0].lslices {
                        for instance in object.instances() {
                            let mut ex_poly_translated = ex_poly.clone();
                            ex_poly_translated.translate(instance.shift_without_plate_offset(print));
                            bbx.merge(&BBox::from_points(&ex_poly_translated.contour.points));
                        }
                    }
                    if !object.support_layers().is_empty() {
                        for support_contour in &first_support_layer_contours(object) {
                            for instance in object.instances() {
                                let mut ex_poly_translated = support_contour.clone();
                                ex_poly_translated.translate(instance.shift_without_plate_offset(print));
                                bbx.merge(&BBox::from_points(&ex_poly_translated.points));
                            }
                        }
                    }
                    if let Some(area) = support_brim_area_map.get(&print_obj_id) {
                        for ex_poly in area {
                            bbx.merge(&BBox::from_points(&ex_poly.contour.points));
                        }
                    }
                    if let Some(area) = brim_area_map.get(&print_obj_id) {
                        for ex_poly in area {
                            bbx.merge(&BBox::from_points(&ex_poly.contour.points));
                        }
                    }
                }
                // `object->firstLayerObjectBrimBoundingBox = bbx`: a box that is not defined stays an empty one.
                print.m_objects[object_idx].firstLayerObjectBrimBoundingBox =
                    if bbx.defined { bbx.to_bounding_box() } else { crate::geometry::BoundingBox::new() };
            }
            let print: &Print = print;

            // Brim.cpp:1813
            *islands_area = to_polygons(&islands_area_ex);

            // BBS: plate offset is applied
            // Brim.cpp:1816-1819
            let plate_offset = print.get_plate_origin();
            let plate_shift = Point::new(scale_(plate_offset[0]) as i64, scale_(plate_offset[1]) as i64);
            for iia in 0..islands_area.len() {
                islands_area[iia].translate(plate_shift);
            }

            // Brim.cpp:1821-1830
            for (first, second) in &brim_area_map {
                if !second.is_empty() {
                    let object = get_object(print, *first);
                    let filament_id: u32 = obj_print_vec.iter().find(|entry| entry.0 == *first).map(|entry| entry.1).unwrap_or(0);
                    let object_flow = match object {
                        Some(object) => print.multi_diameter_brim_flow(object, filament_id)?,
                        None => flow.clone(),
                    };
                    // std::map::insert keeps an existing entry
                    let infill = make_brim_infill(second, print, islands_area, &object_flow)?;
                    brim_map.entry(*first).or_insert(infill);
                }
            }
            // Brim.cpp:1831-1837
            for (first, second) in &support_brim_area_map {
                if !second.is_empty() {
                    let object = get_object(print, *first);
                    let object_flow = match object {
                        Some(object) => print.multi_diameter_support_brim_flow(object)?,
                        None => flow.clone(),
                    };
                    let infill = make_brim_infill(second, print, islands_area, &object_flow)?;
                    support_brim_map.entry(*first).or_insert(infill);
                }
            }

            // Brim.cpp:1839-1840 — `num_loops` is computed for a debug log only.
            Ok(())
        }
    }
    pub use make_brim_by_object::make_brim as make_brim_map;

    /// Brim.cpp:1843-2050
    /// C++: `ExtrusionEntityCollection make_brim(const Print &print, PrintTryCancel try_cancel, Polygons &islands_area)`
    ///
    /// Produce brim lines around those objects, that have the brim enabled.
    /// Collect islands_area to be merged into the final 1st layer convex hull.
    ///
    /// `skirt` is `print.skirt()`: the Print of the pin's port carries no skirt yet.
    pub fn make_brim(print: &Print, _try_cancel: Option<&dyn Fn()>, islands_area: &mut Polygons, skirt: &ExtrusionEntityCollection) -> FlowResult<ExtrusionEntityCollection> {
        use crate::clipper_lib_z as cz;
        // Brim.cpp:1847-1855
        let mut brim_width_max: f64 = 0.0;
        let mut brim_width_map: BTreeMap<ObjectID, f64> = BTreeMap::new();
        let scaled_resolution = scaled_resolution_of(print);
        let flow: Flow = print.brim_flow()?;
        let flow_scaled_spacing: f32 = flow.scaled_spacing() as f32;
        let bottom_layers_expolygons: Vec<ExPolygons> = get_print_bottom_layers_expolygons(print);
        let top_level_objects_with_brim: Vec<usize> = get_top_level_objects_with_brim(print, &bottom_layers_expolygons);
        let _islands: Polygons = top_level_outer_brim_islands(print, &top_level_objects_with_brim, scaled_resolution);
        let islands_area_ex: ExPolygons = top_level_outer_brim_area(
            print,
            &top_level_objects_with_brim,
            &bottom_layers_expolygons,
            flow_scaled_spacing,
            &mut brim_width_max,
            &mut brim_width_map,
        );
        *islands_area = to_polygons(&islands_area_ex);

        // Brim.cpp:1857-1861 — `num_loops` is computed for a debug log only.
        let mut loops: Polygons = try_ex_polygon_offset(&islands_area_ex, print, &flow);
        loops = union_pt_chained_outside_in(&loops);

        // Brim.cpp:1863-1882
        // output
        let mut brim = ExtrusionEntityCollection::new();

        // Reduce down to the ordered list of polylines.
        let mut all_loops: Polylines = loops_to_chained_polylines(&loops, Some(islands_area));

        // Flip orientation of open polylines to minimize travel distance.
        // Brim.cpp:1885
        optimize_polylines_by_reversing(&mut all_loops);

        // Brim.cpp:1899
        all_loops = connect_brim_lines(
            all_loops,
            &cu::offset_expolygons(&islands_area_ex, scale_(EPSILON) as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
            flow_scaled_spacing * 2.0f32,
        );

        // Brim.cpp:1910-1913
        let skirt_distance = print_config_float(print, "skirt_distance");
        let could_brim_intersects_skirt = print.objects().iter().any(|object| {
            let bt = object_config_int(object, "brim_type");
            // `brim_width_map[object->id()]`: 0 for an object the map has no width of.
            (bt == BT_OUTER_ONLY || bt == BT_OUTER_AND_INNER || bt == BT_AUTO_BRIM) && skirt_distance < brim_width_map.get(&object.id()).copied().unwrap_or(0.0)
        });

        // Brim.cpp:1915
        let draft_shield = print_config_int(print, "draft_shield") != crate::print_config::enums::DraftShield::DsDisabled as i32;

        let mm3_per_mm: f64 = flow.mm3_per_mm()? as f32 as f64;
        let width: f32 = flow.width() as f32;
        let height: f32 = print.skirt_first_layer_height() as f32;

        // If there is a possibility that brim intersects skirt, go through loops and split those extrusions
        // The result is either the original Polygon or a list of Polylines
        // Brim.cpp:1920-2046
        if draft_shield && !skirt.entities.is_empty() && could_brim_intersects_skirt {
            // Find the bounding polygons of the skirt
            let loop_polygon = |entity: &ExtrusionEntityType| -> Polygon {
                match entity {
                    ExtrusionEntityType::Loop(l) => l.polygon(),
                    _ => panic!("the skirt of a draft shield is made of loops (Brim.cpp:1923)"),
                }
            };
            let skirt_spacing: f32 = scale_(print.skirt_flow()?.spacing()) as f32;
            let skirt_inners: Polygons =
                cu::offset_polygon(&loop_polygon(skirt.entities.last().unwrap()), -skirt_spacing / 2.0f32, JoinType::jtRound, (scale_(0.1) as f32) as f64);
            let skirt_outers: Polygons =
                cu::offset_polygon(&loop_polygon(&skirt.entities[0]), skirt_spacing / 2.0f32, JoinType::jtRound, (scale_(0.1) as f32) as f64);

            // First calculate the trimming region.
            // Brim.cpp:1933-1958
            let mut trimming: cz::Paths = Vec::new();
            {
                let to_paths = |polys: &Polygons| -> cz::Paths {
                    polys.iter().map(|poly| poly.points.iter().map(|pt| cz::IntPoint::new_xyz(pt.x as cz::cInt, pt.y as cz::cInt, 0)).collect()).collect()
                };
                let input_subject: cz::Paths = to_paths(&skirt_outers);
                let input_clip: cz::Paths = to_paths(&skirt_inners);
                // init Clipper
                let mut clipper = cz::Clipper::new(0);
                // add polygons
                clipper.add_paths(&input_subject, cz::PolyType::ptSubject, true);
                clipper.add_paths(&input_clip, cz::PolyType::ptClip, true);
                // perform operation
                clipper.execute(cz::ClipType::ctDifference, &mut trimming, cz::PolyFillType::pftNonZero, cz::PolyFillType::pftNonZero);
            }

            // Second, trim the extrusion loops with the trimming regions.
            // Brim.cpp:1961-1988
            let mut loops_trimmed: cz::Paths = Vec::new();
            {
                // Produce ClipperLib_Z::Paths from polylines (not necessarily closed).
                let mut input_clip: cz::Paths = Vec::with_capacity(all_loops.len());
                for (loop_idx, loop_pl) in all_loops.iter().enumerate() {
                    // The Z coordinate carries index of the source loop.
                    input_clip.push(loop_pl.points.iter().map(|pt| cz::IntPoint::new_xyz(pt.x as cz::cInt, pt.y as cz::cInt, loop_idx as cz::cInt + 1)).collect());
                }
                // init Clipper
                let mut clipper = cz::Clipper::new(0);
                clipper.z_fill_function(cz::ZFillCallback::new(
                    |e1bot: &cz::IntPoint, e1top: &cz::IntPoint, e2bot: &cz::IntPoint, e2top: &cz::IntPoint, pt: &mut cz::IntPoint| {
                        // Assign a valid input loop identifier. Such an identifier is strictly positive, the next line is safe even in case one side of a segment
                        // hat the Z coordinate not set to the contour coordinate.
                        pt.z = e1bot.z.max(e1top.z).max(e2bot.z.max(e2top.z));
                    },
                ));
                // add polygons
                clipper.add_paths(&input_clip, cz::PolyType::ptSubject, false);
                clipper.add_paths(&trimming, cz::PolyType::ptClip, true);
                // perform operation
                let mut loops_trimmed_tree = cz::PolyTree::new();
                clipper.execute_poly_tree(cz::ClipType::ctDifference, &mut loops_trimmed_tree, cz::PolyFillType::pftNonZero, cz::PolyFillType::pftNonZero);
                cz::poly_tree_to_paths(&loops_trimmed_tree, &mut loops_trimmed);
            }

            // Third, produce the extrusions, sorted by the source loop indices.
            // Brim.cpp:1991-2043
            {
                let mut loops_trimmed_order: Vec<(usize, usize)> = Vec::with_capacity(loops_trimmed.len());
                for (path_idx, path) in loops_trimmed.iter().enumerate() {
                    let mut input_idx: usize = 0;
                    for pt in path {
                        if pt.z > 0 {
                            input_idx = pt.z as usize;
                            break;
                        }
                    }
                    debug_assert!(input_idx != 0);
                    loops_trimmed_order.push((path_idx, input_idx));
                }
                // std::stable_sort
                loops_trimmed_order.sort_by(|l, r| l.1.cmp(&r.1));

                let mut last_pt = Point::new(0, 0);
                let mut i: usize = 0;
                while i < loops_trimmed_order.len() {
                    // Find all pieces that the initial loop was split into.
                    let mut j = i + 1;
                    while j < loops_trimmed_order.len() && loops_trimmed_order[i].1 == loops_trimmed_order[j].1 {
                        j += 1;
                    }
                    let first_path = &loops_trimmed[loops_trimmed_order[i].0];
                    if i + 1 == j
                        && first_path.len() > 3
                        && first_path[0].x == first_path[first_path.len() - 1].x
                        && first_path[0].y == first_path[first_path.len() - 1].y
                    {
                        let mut path = ExtrusionPath::with_params(ExtrusionRole::Brim, mm3_per_mm, width as f64, height as f64, false);
                        path.polyline.points = first_path.iter().map(|pt| Point::new(pt.x as i64, pt.y as i64)).collect();
                        // `new ExtrusionLoop()` (ExtrusionEntity.hpp:500, elrDefault), its only path emplaced.
                        brim.entities.push(ExtrusionEntityType::Loop(ExtrusionLoop::from_path(path, ExtrusionLoopRole::DEFAULT)));
                        i = j;
                    } else {
                        //FIXME The path chaining here may not be optimal.
                        let mut this_loop_trimmed: Vec<ExtrusionEntityType> = Vec::with_capacity(j - i);
                        while i < j {
                            let mut extrusion_path = ExtrusionPath::with_params(ExtrusionRole::Brim, mm3_per_mm, width as f64, height as f64, false);
                            let path = &loops_trimmed[loops_trimmed_order[i].0];
                            extrusion_path.polyline.points = path.iter().map(|pt| Point::new(pt.x as i64, pt.y as i64)).collect();
                            this_loop_trimmed.push(ExtrusionEntityType::Path(extrusion_path));
                            i += 1;
                        }
                        crate::shortest_path::chain_and_reorder_extrusion_entities(&mut this_loop_trimmed, Some(&last_pt));
                        brim.entities.extend(this_loop_trimmed);
                    }
                    last_pt = brim.last_point().expect("the brim has an extrusion");
                }
            }
        } else {
            extrusion_entities_append_loops_and_paths(&mut brim.entities, all_loops, ExtrusionRole::Brim, mm3_per_mm, width, height);
        }

        // Brim.cpp:2048-2049
        make_inner_brim(print, &top_level_objects_with_brim, &bottom_layers_expolygons, &mut brim)?;
        Ok(brim)
    }

    /// `PrintInstance` is part of the signatures of the C++ this module ports through `append_and_translate`.
    #[allow(dead_code)]
    type _PrintInstance = PrintInstance;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polyline};
    use crate::scale;

    fn square(size_mm: f64) -> Polygon {
        let half = scale(size_mm / 2.0);
        Polygon::from_points(vec![
            Point::new(-half, -half),
            Point::new(half, -half),
            Point::new(half, half),
            Point::new(-half, half),
        ])
    }

    #[test]
    fn max_brim_width_ignores_no_brim_objects() {
        // Brim.cpp:62 — btNoBrim objects contribute 0 regardless of brim_width.
        let mut a = PrintObject::new();
        a.config.brim_type = BrimType::NoBrim;
        a.config.brim_width = 50.0;
        let mut b = PrintObject::new();
        b.config.brim_type = BrimType::OuterOnly;
        b.config.brim_width = 5.0;
        assert_eq!(max_brim_width(&[a, b]), 5.0);
    }

    #[test]
    fn comp_second_moment_polygon_degenerate_returns_false() {
        let poly = Polygon::from_points(vec![Point::new(0, 0), Point::new(scale(1.0), 0)]);
        let mut sm = PointF::new(1.0, 1.0);
        assert!(!comp_second_moment_polygon(&poly, &mut sm));
        // sm is zeroed before the size check.
        assert_eq!(sm.x(), 0.0);
        assert_eq!(sm.y(), 0.0);
    }

    #[test]
    fn comp_second_moment_polygon_square_symmetric() {
        let poly = square(10.0);
        let mut sm = PointF::new(0.0, 0.0);
        assert!(comp_second_moment_polygon(&poly, &mut sm));
        // For a centered square the X and Y second moments are equal and positive.
        assert!(sm.x() > 0.0);
        assert!((sm.x() - sm.y()).abs() < 1e-3 * sm.x());
    }

    #[test]
    fn comp_second_moment_expolygon_centroid_origin() {
        let ex = ExPolygon::new(square(10.0));
        let mut prop = ExPolyProp::default();
        assert!(comp_second_moment_expolygon(&ex, &mut prop));
        assert!(prop.aera > 0.0);
        assert!(prop.centroid.x().abs() < 1.0);
        assert!(prop.centroid.y().abs() < 1.0);
    }

    #[test]
    fn comp_second_moment_expolygons_empty_false() {
        let mut x = 0.0;
        let mut y = 0.0;
        assert!(!comp_second_moment_expolygons(&Vec::new(), &mut x, &mut y));
    }

    #[test]
    fn comp_second_moment_expolygons_single() {
        let ex: ExPolygons = vec![ExPolygon::new(square(10.0))];
        let mut x = -1.0;
        let mut y = -1.0;
        assert!(comp_second_moment_expolygons(&ex, &mut x, &mut y));
        assert!(x > 0.0);
        assert!(y > 0.0);
    }

    #[test]
    fn optimize_polylines_reverses_when_closer() {
        // prev ends at (0,0); next is an open line from (10,0) to (0,0).
        // Reversing next puts its start at (0,0), nearer to prev's last point.
        let prev = Polyline::from_points(vec![Point::new(scale(-10.0), 0), Point::new(0, 0)]);
        let next = Polyline::from_points(vec![Point::new(scale(10.0), 0), Point::new(0, 0)]);
        let mut polylines: Polylines = vec![prev, next];
        optimize_polylines_by_reversing(&mut polylines);
        // After reversing, next.first_point() should be (0,0).
        assert_eq!(polylines[1].first_point(), Point::new(0, 0));
    }
}
