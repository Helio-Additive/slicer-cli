// Calculate extents of the extrusions assigned to Print / PrintObject.
// The extents are used for assessing collisions of the print with the priming towers,
// to decide whether to pause the print after the priming towers are extruded
// to let the operator remove them from the print bed.
//
// Faithful 1:1 port of BambuStudio's src/libslic3r/GCode/PrintExtents.cpp.
// coord_t -> i64, coordf_t -> f64.

use crate::extrusion_entity::{
    ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionMultiPath,
    ExtrusionPath,
};
use crate::geometry::{BoundingBox, BoundingBoxF, PointF, Polyline};
use crate::print::Print;
use crate::print_object::PrintObject;
use crate::{scale, Coord};

// PrintExtents.cpp:249-252 (BoundingBox.hpp Slic3r::empty()) —
//   return ! bb.defined || bb.min(0) >= bb.max(0) || bb.min(1) >= bb.max(1);
// BoundingBox::is_empty() only tests `defined`, so we reproduce the full predicate here
// to match the C++ `empty()` free function used below.
#[inline]
fn empty_bbox(bb: &BoundingBox) -> bool {
    !bb.is_defined() || bb.min.x >= bb.max.x || bb.min.y >= bb.max.y
}

// PrintExtents.cpp:17-29
fn extrusion_polyline_extents(polyline: &Polyline, radius: Coord) -> BoundingBox {
    // PrintExtents.cpp:19
    let mut bbox = BoundingBox::new();
    // PrintExtents.cpp:20-21
    if !polyline.points.is_empty() {
        bbox.merge_point(polyline.points[0]);
    }
    // PrintExtents.cpp:22-27
    for pt in &polyline.points {
        bbox.min.x = bbox.min.x.min(pt.x - radius);
        bbox.min.y = bbox.min.y.min(pt.y - radius);
        bbox.max.x = bbox.max.x.max(pt.x + radius);
        bbox.max.y = bbox.max.y.max(pt.y + radius);
    }
    // PrintExtents.cpp:28
    bbox
}

// PrintExtents.cpp:31-41
fn extrusionentity_extents_path(extrusion_path: &ExtrusionPath) -> BoundingBoxF {
    // PrintExtents.cpp:33
    let bbox = extrusion_polyline_extents(
        &extrusion_path.polyline,
        scale(0.5 * extrusion_path.width) as Coord,
    );
    // PrintExtents.cpp:34
    let mut bboxf = BoundingBoxF::new();
    // PrintExtents.cpp:35-39
    if !empty_bbox(&bbox) {
        // bboxf.min = unscale(bbox.min); bboxf.max = unscale(bbox.max); bboxf.defined = true;
        // `from_points_minmax` sets `defined = true`, mirroring the three C++ assignments.
        bboxf = BoundingBoxF::from_points_minmax(bbox.min.to_f64(), bbox.max.to_f64());
    }
    // PrintExtents.cpp:40
    bboxf
}

// PrintExtents.cpp:43-55
fn extrusionentity_extents_loop(extrusion_loop: &ExtrusionLoop) -> BoundingBoxF {
    // PrintExtents.cpp:45
    let mut bbox = BoundingBox::new();
    // PrintExtents.cpp:46-47
    for extrusion_path in &extrusion_loop.paths {
        bbox.merge(&extrusion_polyline_extents(
            &extrusion_path.polyline,
            scale(0.5 * extrusion_path.width) as Coord,
        ));
    }
    // PrintExtents.cpp:48
    let mut bboxf = BoundingBoxF::new();
    // PrintExtents.cpp:49-53
    if !empty_bbox(&bbox) {
        // bboxf.min = unscale(bbox.min); bboxf.max = unscale(bbox.max); bboxf.defined = true;
        // `from_points_minmax` sets `defined = true`, mirroring the three C++ assignments.
        bboxf = BoundingBoxF::from_points_minmax(bbox.min.to_f64(), bbox.max.to_f64());
    }
    // PrintExtents.cpp:54
    bboxf
}

// PrintExtents.cpp:57-69
// Faithfully ported overload. Not reachable from the `ExtrusionEntityType` dispatch because this
// crate's enum has no MultiPath variant (see `extrusionentity_extents_entity`); retained verbatim
// so callers holding an `ExtrusionMultiPath` directly get identical extents.
#[allow(dead_code)]
fn extrusionentity_extents_multi_path(extrusion_multi_path: &ExtrusionMultiPath) -> BoundingBoxF {
    // PrintExtents.cpp:59
    let mut bbox = BoundingBox::new();
    // PrintExtents.cpp:60-61
    for extrusion_path in &extrusion_multi_path.paths {
        bbox.merge(&extrusion_polyline_extents(
            &extrusion_path.polyline,
            scale(0.5 * extrusion_path.width) as Coord,
        ));
    }
    // PrintExtents.cpp:62
    let mut bboxf = BoundingBoxF::new();
    // PrintExtents.cpp:63-67
    if !empty_bbox(&bbox) {
        // bboxf.min = unscale(bbox.min); bboxf.max = unscale(bbox.max); bboxf.defined = true;
        // `from_points_minmax` sets `defined = true`, mirroring the three C++ assignments.
        bboxf = BoundingBoxF::from_points_minmax(bbox.min.to_f64(), bbox.max.to_f64());
    }
    // PrintExtents.cpp:68
    bboxf
}

// PrintExtents.cpp:71 (forward declaration)
// PrintExtents.cpp:73-79
fn extrusionentity_extents_collection(
    extrusion_entity_collection: &ExtrusionEntityCollection,
) -> BoundingBoxF {
    // PrintExtents.cpp:75
    let mut bbox = BoundingBoxF::new();
    // PrintExtents.cpp:76-77
    for extrusion_entity in &extrusion_entity_collection.entities {
        bbox.merge(&extrusionentity_extents_entity(extrusion_entity));
    }
    // PrintExtents.cpp:78
    bbox
}

// PrintExtents.cpp:81-99
//
// C++ dispatches via dynamic_cast over ExtrusionPath / ExtrusionLoop / ExtrusionMultiPath /
// ExtrusionEntityCollection. The Rust `ExtrusionEntityType` enum only models Path / Loop /
// Collection (there is no MultiPath variant in this crate), so the ExtrusionMultiPath branch
// (PrintExtents.cpp:91-93) cannot be reached through the enum dispatch; the dedicated
// `extrusionentity_extents_multi_path` overload above is still ported for completeness and for
// callers holding an `ExtrusionMultiPath` directly.
fn extrusionentity_extents_entity(extrusion_entity: &ExtrusionEntityType) -> BoundingBoxF {
    // PrintExtents.cpp:83-84: the `nullptr` case is impossible for a borrowed enum value.
    match extrusion_entity {
        // PrintExtents.cpp:85-87
        ExtrusionEntityType::Path(extrusion_path) => extrusionentity_extents_path(extrusion_path),
        // PrintExtents.cpp:88-90
        ExtrusionEntityType::Loop(extrusion_loop) => extrusionentity_extents_loop(extrusion_loop),
        // PrintExtents.cpp:94-96
        ExtrusionEntityType::Collection(extrusion_entity_collection) => {
            extrusionentity_extents_collection(extrusion_entity_collection)
        }
    }
    // PrintExtents.cpp:97: throw on unexpected type — unreachable, the match above is exhaustive.
}

// PrintExtents.cpp:101-106
pub fn get_print_extrusions_extents(print: &Print) -> BoundingBoxF {
    //BBS: usage of m_brim are deleted, the bbx of skrit is always larger than that of brim
    // PrintExtents.cpp:104
    let bbox = extrusionentity_extents_collection(print.skirt());
    // PrintExtents.cpp:105
    bbox
}

// PrintExtents.cpp:108-132
pub fn get_print_object_extrusions_extents(
    print_object: &PrintObject,
    max_print_z: f64,
) -> BoundingBoxF {
    // PrintExtents.cpp:110
    let mut bbox = BoundingBoxF::new();
    // PrintExtents.cpp:111
    for layer in print_object.layers() {
        // PrintExtents.cpp:112-113
        if layer.print_z > max_print_z {
            break;
        }
        // PrintExtents.cpp:114
        let mut bbox_this = BoundingBoxF::new();
        // PrintExtents.cpp:115
        for layerm in layer.regions() {
            // PrintExtents.cpp:116
            bbox_this.merge(&extrusionentity_extents_collection(&layerm.perimeters));
            // PrintExtents.cpp:117-119
            for ee in &layerm.fills.entities {
                // fill represents infill extrusions of a single island.
                // C++ does `*dynamic_cast<const ExtrusionEntityCollection*>(ee)`; the Rust
                // enum dispatch in `extrusionentity_extents_entity` reaches the Collection arm,
                // matching the same extents accumulation.
                bbox_this.merge(&extrusionentity_extents_entity(ee));
            }
        }
        // PrintExtents.cpp:121-124
        // C++ tests `dynamic_cast<const SupportLayer*>(layer)`; this crate folds the support
        // layer into `Layer`, exposing the optional `support_fills` collection in its place.
        if let Some(support_fills) = &layer.support_fills {
            for extrusion_entity in &support_fills.entities {
                bbox_this.merge(&extrusionentity_extents_entity(extrusion_entity));
            }
        }
        // PrintExtents.cpp:125-129
        // FIDELITY-NOTE(blocked-dep): C++ iterates `print_object.instances()` and merges
        // `bbox_this` translated by `unscale(instance.shift)` for every PrintInstance. This crate
        // does not model the PrintInstance subsystem on PrintObject (no `instances()`, and
        // `PrintInstance` is an empty stub with no `shift` field), so we emulate the single
        // zero-shift instance case (`bbox_translated = bbox_this`, shift == 0) and merge directly.
        // Restoring exact parity requires adding the print-instance subsystem (cross-cutting).
        bbox.merge(&bbox_this);
    }
    // PrintExtents.cpp:131
    bbox
}

// Faithful port of the per-extrusion extent accumulation shared by
// PrintExtents.cpp:152-162 (general wipe-tower extents) — operates on a single
// `ToolChangeResult`'s extrusion list, applying the wipe-tower placement transform.
//
// `trafo` mirrors the C++ `Transform2d trafo = Translation2d(x, y) * Rotation2Dd(angle)`,
// expressed as (translation, cos, sin) so `trafo * p == rotate(p) + translation`.
#[allow(dead_code)]
fn wipe_tower_toolchange_extents(
    bbox: &mut BoundingBoxF,
    extrusions: &[crate::gcode::wipe_tower::Extrusion],
    translation: PointF,
    cos_a: f64,
    sin_a: f64,
) {
    // PrintExtents.cpp:153
    for i in 1..extrusions.len() {
        // PrintExtents.cpp:154
        let e = &extrusions[i];
        // PrintExtents.cpp:155
        if e.width > 0.0 {
            // PrintExtents.cpp:156: Vec2d delta = 0.5 * Vec2d(e.width, e.width);
            let delta = PointF::new(0.5 * e.width as f64, 0.5 * e.width as f64);
            // PrintExtents.cpp:157-158: p1 = trafo * (&e - 1)->pos; p2 = trafo * e.pos;
            let prev = &extrusions[i - 1];
            let p1 = apply_trafo(translation, cos_a, sin_a, prev.pos.x as f64, prev.pos.y as f64);
            let p2 = apply_trafo(translation, cos_a, sin_a, e.pos.x as f64, e.pos.y as f64);
            // PrintExtents.cpp:159: bbox.merge(p1.cwiseMin(p2) - delta);
            bbox.merge_point(
                PointF::new(p1.x.min(p2.x), p1.y.min(p2.y)) - delta,
            );
            // PrintExtents.cpp:160: bbox.merge(p1.cwiseMax(p2) + delta);
            bbox.merge_point(
                PointF::new(p1.x.max(p2.x), p1.y.max(p2.y)) + delta,
            );
        }
    }
}

// `Transform2d trafo = Translation2d * Rotation2Dd` applied to a point:
// rotate by `[cos -sin; sin cos]` then translate.
#[allow(dead_code)]
#[inline]
fn apply_trafo(translation: PointF, cos_a: f64, sin_a: f64, x: f64, y: f64) -> PointF {
    PointF::new(
        cos_a * x - sin_a * y + translation.x,
        sin_a * x + cos_a * y + translation.y,
    )
}

// PrintExtents.cpp:134-166
// Returns a bounding box of a projection of the wipe tower for the layers <= max_print_z.
// The projection does not contain the priming regions.
pub fn get_wipe_tower_extrusions_extents(_print: &Print, _max_print_z: f64) -> BoundingBoxF {
    // FIDELITY-NOTE(blocked-dep): C++ reads `print.get_plate_index()`,
    // `print.get_plate_origin()`, `print.config().wipe_tower_x/y.get_at(plate_idx)`,
    // `print.config().wipe_tower_rotation_angle`, and `print.wipe_tower_data().tool_changes`
    // (PrintExtents.cpp:140-149) to build the placement transform and walk the per-layer tool
    // changes. This crate's `Print` models none of the WipeTowerData holder / plate-index /
    // plate-origin / wipe_tower_rotation_angle accessors, so the faithful body cannot be wired
    // up per-file. The per-extrusion extent math is ported verbatim in
    // `wipe_tower_toolchange_extents`/`apply_trafo` above; this returns the C++ default
    // (empty BoundingBoxf) until the WipeTowerData subsystem is added (cross-cutting).
    BoundingBoxF::new()
}

// PrintExtents.cpp:168-190
// Returns a bounding box of the wipe tower priming extrusions.
pub fn get_wipe_tower_priming_extrusions_extents(_print: &Print) -> BoundingBoxF {
    // PrintExtents.cpp:171
    let bbox = BoundingBoxF::new();
    // PrintExtents.cpp:172: `if (print.wipe_tower_data().priming != nullptr)`
    // FIDELITY-NOTE(blocked-dep): C++ walks `*print.wipe_tower_data().priming`
    // (PrintExtents.cpp:173-187), merging each priming extrusion's endpoints widened by
    // `0.5 * e.width`. This crate's `Print` exposes no `wipe_tower_data()` accessor (the
    // WipeTowerData holder, incl. its `priming` field, is unmodeled), so the priming list is
    // never available here; this mirrors the `priming == nullptr` branch and returns the empty
    // BoundingBoxf default. Restoring parity requires the WipeTowerData subsystem (cross-cutting).
    // PrintExtents.cpp:189
    bbox
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::extrusion_entity::ExtrusionRole;
    use crate::geometry::{Point, Polyline};

    fn make_path(points: Vec<Point>, width: f64) -> ExtrusionPath {
        let mut path = ExtrusionPath::new(ExtrusionRole::Perimeter);
        path.polyline = Polyline::from_points(points);
        path.width = width;
        path
    }

    #[test]
    fn test_extrusion_polyline_extents_empty() {
        let polyline = Polyline::new();
        let bbox = extrusion_polyline_extents(&polyline, scale(0.5));
        assert!(!bbox.is_defined());
    }

    #[test]
    fn test_extrusion_polyline_extents_single_segment() {
        // Two points; radius adds margin on every side.
        let polyline =
            Polyline::from_points(vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)]);
        let radius = scale(0.2);
        let bbox = extrusion_polyline_extents(&polyline, radius);
        assert!(bbox.is_defined());
        assert_eq!(bbox.min.x, Point::new_scale(0.0, 0.0).x - radius);
        assert_eq!(bbox.min.y, Point::new_scale(0.0, 0.0).y - radius);
        assert_eq!(bbox.max.x, Point::new_scale(10.0, 0.0).x + radius);
        assert_eq!(bbox.max.y, Point::new_scale(0.0, 0.0).y + radius);
    }

    #[test]
    fn test_extrusionentity_extents_path() {
        let path = make_path(
            vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 5.0)],
            0.4,
        );
        let bboxf = extrusionentity_extents_path(&path);
        assert!(bboxf.is_defined());
        // The path width contributes 0.5 * 0.4 = 0.2 mm of radius.
        assert!((bboxf.min.x - (0.0 - 0.2)).abs() < 1e-6);
        assert!((bboxf.max.x - (10.0 + 0.2)).abs() < 1e-6);
    }

    #[test]
    fn test_empty_collection() {
        let collection = ExtrusionEntityCollection::new();
        let bboxf = extrusionentity_extents_collection(&collection);
        assert!(!bboxf.is_defined());
    }

    #[test]
    fn test_print_extrusions_extents_empty_skirt() {
        let print = Print::new();
        let bboxf = get_print_extrusions_extents(&print);
        assert!(!bboxf.is_defined());
    }
}
