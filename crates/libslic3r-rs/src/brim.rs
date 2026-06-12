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
        // morphological closing of the surfaces' expolygons. SCALED_EPSILON is
        // scale_(EPSILON), so the mm-domain closing distance of this crate's
        // clipper wrappers is EPSILON.
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
        // Brim.cpp:709
        *sm_expolys_x += prop.second_moment_of_area_respect_to_centroid.x() + prop.aera * delta_y * delta_y;
        // Brim.cpp:710
        *sm_expolys_y += prop.second_moment_of_area_respect_to_centroid.y() + prop.aera * delta_x * delta_x;
    }

    // Brim.cpp:712
    true
}

// Brim.cpp:1145
// Flip orientation of open polylines to minimize travel distance.
// Brim.cpp:1146
pub fn optimize_polylines_by_reversing(polylines: &mut Polylines) {
    // Brim.cpp:1148
    for poly_idx in 1..polylines.len() {
        // Brim.cpp:1149
        let prev_last = polylines[poly_idx - 1].last_point();
        // Brim.cpp:1150
        // (next is borrowed mutably below)

        // Brim.cpp:1152
        if !polylines[poly_idx].is_closed() {
            // Brim.cpp:1153
            let dist_to_start = (polylines[poly_idx].first_point().to_f64() - prev_last.to_f64()).length();
            // Brim.cpp:1154
            let dist_to_end = (polylines[poly_idx].last_point().to_f64() - prev_last.to_f64()).length();

            // Brim.cpp:1156-1157
            if dist_to_end < dist_to_start {
                polylines[poly_idx].reverse();
            }
        }
    }
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
