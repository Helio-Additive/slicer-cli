//! Port of `GCode/TimelapsePosPicker.{hpp,cpp}` of Bambu Studio.
//!
//! The timelapse position picker selects a safe location for the toolhead to
//! move to while a timelapse photo is taken, avoiding collisions with already
//! printed objects, the camera occlusion zone and the rod clearance area.
//!
//! The class works on `crate::print::faithful::Print`: an object is named by its
//! index in `Print::m_objects` where the C++ holds a `const PrintObject*`.

// TimelapsePosPicker.cpp:1   #include "ClipperUtils.hpp"
// TimelapsePosPicker.cpp:2   #include "TimelapsePosPicker.hpp"
// TimelapsePosPicker.cpp:3   #include "Layer.hpp"
use crate::bounding_box::BoundingBoxf3;
use crate::clipper::{ClipType, JoinType, Path, Paths, PolyFillType, PolyTree};
use crate::clipper_utils::{
    _clipper_ex, clipper_do, clipper_do_polytree, expolygons_offset, intersection_pl_clib, offset_paths, poly_tree_to_ex_polygons,
    raw_offset, ApplySafetyOffset,
};
use crate::config::ConfigOptionResolver;
use crate::gcode::wipe_tower::{cfg_enum, cfg_float, cfg_floats};
use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon, Polyline};
use crate::layer::Layer;
use crate::libslic3r::{EPSILON, SCALING_FACTOR};
use crate::print::faithful::{Print, PrintInstance};
use crate::print_config::enums::{PrintSequence, TimelapseType};
use crate::utils::next_idx_modulo;
use std::collections::HashMap;

// TimelapsePosPicker.hpp:12  const Point DefaultTimelapsePos = Point(0, 0);
pub const DEFAULT_TIMELAPSE_POS: Point = Point { x: 0, y: 0 };
// TimelapsePosPicker.hpp:13  const Point DefaultCameraPos = Point(0, 0);
pub const DEFAULT_CAMERA_POS: Point = Point { x: 0, y: 0 };

// TimelapsePosPicker.cpp:5   constexpr int FILTER_THRESHOLD = 5;
pub(crate) const FILTER_THRESHOLD: i32 = 5;
// TimelapsePosPicker.cpp:6   constexpr int MAX_CANDIDATE_SIZE = 5;
const MAX_CANDIDATE_SIZE: usize = 5;

/// libslic3r.h `scale_(val)`: `((val) / SCALING_FACTOR)`.
#[inline]
fn scale_(val: f64) -> f64 {
    val / SCALING_FACTOR
}

/// libslic3r.h `unscale_(val)`: `((val) * SCALING_FACTOR)`.
#[inline]
fn unscale_(val: f64) -> f64 {
    val * SCALING_FACTOR
}

/// Point.hpp `Point(double x, double y)`: `coord_t(lrint(x)), coord_t(lrint(y))`.
#[inline]
fn point_lrint(x: f64, y: f64) -> Point {
    Point::new(x.round_ties_even() as i64, y.round_ties_even() as i64)
}

/// `ClipperUtils::SinglePathProvider(polygon.points)`
fn polygon_path(polygon: &Polygon) -> Path {
    polygon.points.iter().map(|p| crate::clipper::IntPoint::new(p.x as i32, p.y as i32)).collect()
}

/// `ClipperUtils::PolygonsProvider(polygons)`
fn polygons_paths(polygons: &[Polygon]) -> Paths {
    polygons.iter().map(polygon_path).collect()
}

/// `ClipperUtils::ExPolygonsProvider(expolygons)`: every contour, then its holes.
fn expolygons_paths(expolygons: &[ExPolygon]) -> Paths {
    let mut out = Paths::new();
    for expoly in expolygons {
        out.push(polygon_path(&expoly.contour));
        for hole in &expoly.holes {
            out.push(polygon_path(hole));
        }
    }
    out
}

/// ClipperUtils.cpp `to_polygons(ClipperLib::Paths &&paths)`
fn paths_polygons(paths: &Paths) -> Vec<Polygon> {
    paths.iter().map(|p| Polygon::from_points(p.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect())).collect()
}

/// ClipperUtils.cpp:721-722 `Polygons union_(const Polygons &subject)`:
/// `_clipper(ctUnion, PolygonsProvider(subject), EmptyPathsProvider(), ApplySafetyOffset::No)`.
fn union_(subject: &[Polygon]) -> Vec<Polygon> {
    paths_polygons(&clipper_do::<Paths>(ClipType::ctUnion, &polygons_paths(subject), &Paths::new(), PolyFillType::pftNonZero))
}

/// ClipperUtils.cpp:815-816 `ExPolygons union_ex(const ExPolygons &subject)`
fn union_ex(subject: &[ExPolygon]) -> ExPolygons {
    poly_tree_to_ex_polygons(&clipper_do_polytree(ClipType::ctUnion, &expolygons_paths(subject), &Paths::new(), PolyFillType::pftNonZero))
}

/// ClipperUtils.cpp:817-821 `ExPolygons union_ex(const ExPolygons &subject, const Polygons &subject2)`
fn union_ex_expolygons_polygons(subject: &[ExPolygon], subject2: &[Polygon]) -> ExPolygons {
    poly_tree_to_ex_polygons(&clipper_do_polytree(
        ClipType::ctUnion,
        &expolygons_paths(subject),
        &polygons_paths(subject2),
        PolyFillType::pftNonZero,
    ))
}

/// ClipperUtils.hpp:428-429 `opening_ex(const ExPolygons &polygons, const float delta)`:
/// `offset2_ex(polygons, - delta, delta, DefaultJoinType, DefaultMiterLimit)` (ClipperUtils.cpp:581-584).
fn opening_ex(polygons: &[ExPolygon], delta: f32) -> ExPolygons {
    let expolys: Vec<(Path, Paths)> = polygons.iter().map(|e| (polygon_path(&e.contour), polygons_paths(&e.holes))).collect();
    poly_tree_to_ex_polygons(&offset_paths::<PolyTree>(
        &expolygons_offset(&expolys, -(delta as f64), JoinType::jtMiter, 3.0),
        delta as f64,
        JoinType::jtMiter,
        3.0,
    ))
}

/// TimelapsePosPicker.hpp:18-26 `struct PosPickCtx`
#[derive(Clone)]
pub struct PosPickCtx<'a> {
    pub curr_pos: Point,
    pub curr_layer: &'a Layer,
    /// the extruder id to take picture
    pub picture_extruder_id: i32,
    pub curr_extruder_id: i32,
    /// printed objects, only have value in by object mode
    pub printed_objects: Option<Vec<usize>>,
    /// plate-relative scaled; when set, pick_pos_internal uses farthest-point loss
    pub farthest_point: Option<Point>,
}

/// TimelapsePosPicker.hpp:28-78 `class TimelapsePosPicker`: data are stored without plate offset.
pub struct TimelapsePosPicker<'p> {
    print: Option<&'p Print>,
    /// scaled data
    m_extruder_printable_area: Vec<ExPolygons>,
    /// scaled data
    m_bed_polygon: Polygon,
    /// unscaled data
    m_plate_offset: Point,
    /// unscaled data
    m_plate_height: i32,
    /// unscaled data
    m_plate_width: i32,

    m_print_seq: PrintSequence,
    m_based_on_all_layer: bool,
    m_nozzle_height_to_rod: i32,
    m_nozzle_clearance_radius: i32,
    m_liftable_extruder_id: Option<i32>,
    m_extruder_height_gap: Option<i32>,

    /// `std::unordered_map<const PrintInstance*, BoundingBoxf3> bbox_cache`
    bbox_cache: HashMap<*const PrintInstance, BoundingBoxf3>,

    m_all_layer_pos: Option<Point>,
}

impl<'p> Default for TimelapsePosPicker<'p> {
    /// TimelapsePosPicker.hpp:31 `TimelapsePosPicker() = default;`
    fn default() -> Self {
        TimelapsePosPicker {
            print: None,
            m_extruder_printable_area: Vec::new(),
            m_bed_polygon: Polygon::new(),
            m_plate_offset: Point::new(0, 0),
            m_plate_height: 0,
            m_plate_width: 0,
            m_print_seq: PrintSequence::ByLayer,
            m_based_on_all_layer: false,
            m_nozzle_height_to_rod: 0,
            m_nozzle_clearance_radius: 0,
            m_liftable_extruder_id: None,
            m_extruder_height_gap: None,
            bbox_cache: HashMap::new(),
            m_all_layer_pos: None,
        }
    }
}

impl<'p> TimelapsePosPicker<'p> {
    /// TimelapsePosPicker.cpp:9-30 `void TimelapsePosPicker::init(const Print* print_, const Point& plate_offset)`
    ///
    /// `Print::wipe_tower_data()` writes its estimate through a `const_cast` in the C++; the port of it
    /// takes the print as `&mut`, so `init` does as well and keeps the shared borrow afterwards.
    pub fn init(&mut self, print_: &'p mut Print, plate_offset: &Point) {
        self.reset();
        self.m_plate_offset = *plate_offset;

        self.m_nozzle_height_to_rod = cfg_float(print_.config(), "extruder_clearance_height_to_rod") as i32;
        self.m_nozzle_clearance_radius = cfg_float(print_.config(), "extruder_clearance_max_radius") as i32;
        if cfg_floats(print_.config(), "nozzle_diameter").len() > 1 {
            let extruder_printable_height = cfg_floats(print_.config(), "extruder_printable_height");
            self.m_liftable_extruder_id = Some(if extruder_printable_height[0] < extruder_printable_height[1] { 0 } else { 1 });
            // Only honor the dual-nozzle height gap when more than one extruder is actually used.
            // Otherwise a dual-nozzle machine printing with a single extruder would needlessly
            // retreat to the trash bin for every shot below the gap.
            let nozzle_group = print_.get_nozzle_group_result();
            if let Some(nozzle_group) = nozzle_group {
                if nozzle_group.get_used_extruders().len() > 1 {
                    self.m_extruder_height_gap = Some((extruder_printable_height[0] - extruder_printable_height[1]).abs() as i32);
                }
            }
        }
        self.m_print_seq = if cfg_enum(print_.config(), "print_sequence") == PrintSequence::ByObject as i32 {
            PrintSequence::ByObject
        } else if cfg_enum(print_.config(), "print_sequence") == PrintSequence::ByDefault as i32 {
            PrintSequence::ByDefault
        } else {
            PrintSequence::ByLayer
        };
        self.m_based_on_all_layer = cfg_enum(print_.config(), "timelapse_type") == TimelapseType::TlSmooth as i32;

        self.construct_printable_area_by_printer(print_);
    }

    /// TimelapsePosPicker.cpp:32-46 `void TimelapsePosPicker::reset()`
    pub fn reset(&mut self) {
        self.print = None;
        self.m_bed_polygon.points.clear();
        self.m_extruder_printable_area.clear();
        self.m_all_layer_pos = None;
        self.bbox_cache.clear();

        self.m_print_seq = PrintSequence::ByObject;
        self.m_nozzle_height_to_rod = 0;
        self.m_nozzle_clearance_radius = 0;
        self.m_liftable_extruder_id = None;
        self.m_extruder_height_gap = None;
        self.m_based_on_all_layer = false;
    }

    /// The `print` member, set by `init`.
    fn print(&self) -> &'p Print {
        self.print.expect("TimelapsePosPicker::init")
    }

    /// TimelapsePosPicker.cpp:54-64 `std::vector<const PrintObject*> TimelapsePosPicker::get_object_list(const std::optional<std::vector<const PrintObject*>>& printed_objects)`
    ///
    /// Retrieves a list of print objects based on the provided optional set of printed objects.
    /// If the optional set of printed objects is provided, it converts the set into a vector.
    /// Otherwise, it retrieves all objects from the print instance.
    fn get_object_list(&self, printed_objects: &Option<Vec<usize>>) -> Vec<usize> {
        let object_list: Vec<usize>;
        if let Some(printed_objects) = printed_objects {
            object_list = printed_objects.iter().copied().collect();
        } else {
            object_list = (0..self.print().objects().len()).collect();
        }
        object_list
    }

    /// TimelapsePosPicker.cpp:72-117 `void TimelapsePosPicker::construct_printable_area_by_printer()`
    ///
    /// Constructs the printable area based on printer configuration: initializes the bed polygon,
    /// excludes specific areas, accounts for wipe towers, and calculates the printable area for each
    /// extruder. Sets the `print` member (see `init`).
    fn construct_printable_area_by_printer(&mut self, print: &'p mut Print) {
        let config = print.config().clone();
        let extruder_count = cfg_floats(&config, "nozzle_diameter").len();
        self.m_extruder_printable_area.clear();
        self.m_extruder_printable_area.resize(extruder_count, ExPolygons::new());

        let printable_area_values = cfg_points(&config, "printable_area");
        for idx in 0..printable_area_values.len() {
            self.m_bed_polygon
                .points
                .push(Point::new(scale_(printable_area_values[idx].x) as i64, scale_(printable_area_values[idx].y) as i64));
        }

        let bed_bbox = crate::geometry::polygon::get_extents(&self.m_bed_polygon);
        self.m_plate_height = unscale_(bed_bbox.max.y as f64) as i32;
        self.m_plate_width = unscale_(bed_bbox.max.x as f64) as i32;

        let mut bed_exclude_area = Polygon::new();
        let bed_exclude_area_values = cfg_points(&config, "bed_exclude_area");
        for idx in 0..bed_exclude_area_values.len() {
            bed_exclude_area
                .points
                .push(Point::new(scale_(bed_exclude_area_values[idx].x) as i64, scale_(bed_exclude_area_values[idx].y) as i64));
        }

        let fake_pos = print.get_fake_wipe_tower().pos;
        let mut base_wp_pt = Point::new(fake_pos[0] as i64, fake_pos[1] as i64);
        base_wp_pt = point_lrint(scale_(base_wp_pt.x as f64), scale_(base_wp_pt.y as f64));

        let transform_wt_pt = |pt: Point| -> Point {
            let mut out = pt;
            out.x += base_wp_pt.x;
            out.y += base_wp_pt.y;
            out
        };
        let wt_box = print.wipe_tower_data(0).bbx;
        let print: &'p Print = print;
        self.print = Some(print);
        let (wt_min, wt_max) = wt_box;
        let mut wipe_tower_area = Polygon::from_points(vec![
            transform_wt_pt(point_lrint(scale_(wt_min.0), scale_(wt_min.1))),
            transform_wt_pt(point_lrint(scale_(wt_max.0), scale_(wt_min.1))),
            transform_wt_pt(point_lrint(scale_(wt_max.0), scale_(wt_max.1))),
            transform_wt_pt(point_lrint(scale_(wt_min.0), scale_(wt_max.1))),
        ]);
        wipe_tower_area = self.expand_object_projection(&wipe_tower_area, self.m_print_seq == PrintSequence::ByObject, true);

        let extruder_printable_area_values = cfg_points_groups(&config, "extruder_printable_area");
        for idx in 0..extruder_count {
            // diff(m_bed_polygon, bed_exclude_area): ClipperUtils.cpp:675-676
            let bed = clipper_do::<Paths>(
                ClipType::ctDifference,
                &vec![polygon_path(&self.m_bed_polygon)],
                &vec![polygon_path(&bed_exclude_area)],
                PolyFillType::pftNonZero,
            );
            // diff_ex(const Polygons &subject, const Polygons &clip): ClipperUtils.cpp:741-742
            let mut printable_area: ExPolygons = _clipper_ex(
                ClipType::ctDifference,
                &bed,
                &vec![polygon_path(&wipe_tower_area)],
                ApplySafetyOffset::No,
                PolyFillType::pftNonZero,
            );
            if idx < extruder_printable_area_values.len() {
                let mut extruder_printable_area = Polygon::new();
                for j in 0..extruder_printable_area_values[idx].len() {
                    extruder_printable_area.points.push(Point::new(
                        scale_(extruder_printable_area_values[idx][j].x) as i64,
                        scale_(extruder_printable_area_values[idx][j].y) as i64,
                    ));
                }
                // intersection_ex(const ExPolygons &subject, const Polygons &clip): ClipperUtils.cpp:796-797
                printable_area = _clipper_ex(
                    ClipType::ctIntersection,
                    &expolygons_paths(&printable_area),
                    &vec![polygon_path(&extruder_printable_area)],
                    ApplySafetyOffset::No,
                    PolyFillType::pftNonZero,
                );
            }
            self.m_extruder_printable_area[idx] = printable_area;
        }
    }

    /// TimelapsePosPicker.cpp:128-163 `ExPolygons TimelapsePosPicker::collect_object_slices_data(const Layer* layer, float height_range, const std::vector<const PrintObject*>& object_list, bool by_object)`
    ///
    /// Collects object slice data within a specified height range for a given layer.
    /// `by_object` decides the expand length of polygon.
    fn collect_object_slices_data(&mut self, layer: &Layer, height_range: f32, object_list: &[usize], by_object: bool) -> ExPolygons {
        let range_intersect = |left1: i32, right1: i32, left2: i32, right2: i32| -> bool {
            if left1 <= left2 && left2 <= right1 {
                return true;
            }
            if left2 <= left1 && left1 <= right2 {
                return true;
            }
            false
        };
        let mut ret: ExPolygons = ExPolygons::new();
        let z_target: f32 = layer.print_z as f32;
        let z_low: f32 = if height_range < 0.0 { (layer.print_z + height_range as f64) as f32 } else { layer.print_z as f32 };
        let z_high: f32 = if height_range < 0.0 { layer.print_z as f32 } else { (layer.print_z + height_range as f64) as f32 };
        if z_low <= 0.0 {
            return vec![ExPolygon::new(self.m_bed_polygon.clone())];
        }

        let print = self.print();
        for &obj in object_list {
            for instance in print.objects()[obj].instances() {
                let instance_bbox = self.get_real_instance_bbox(instance);
                let higher_than_curr_layer = if Some(&obj) == object_list.last() { false } else { instance_bbox.max.z > z_target as f64 };
                if range_intersect(instance_bbox.min.z as i32, instance_bbox.max.z as i32, z_low as i32, z_high as i32) {
                    let mut expoly = ExPolygon::default();
                    expoly.contour = Polygon::from_points(vec![
                        point_lrint(scale_(instance_bbox.min.x), scale_(instance_bbox.min.y)),
                        point_lrint(scale_(instance_bbox.max.x), scale_(instance_bbox.min.y)),
                        point_lrint(scale_(instance_bbox.max.x), scale_(instance_bbox.max.y)),
                        point_lrint(scale_(instance_bbox.min.x), scale_(instance_bbox.max.y)),
                    ]);
                    expoly.contour = self.expand_object_projection(&expoly.contour, by_object, higher_than_curr_layer);
                    ret.push(expoly);
                }
            }
        }
        ret = union_ex(&ret);
        ret
    }

    /// TimelapsePosPicker.cpp:166-173 `Polygons TimelapsePosPicker::collect_limit_areas_for_camera(const std::vector<const PrintObject*>& object_list)`
    fn collect_limit_areas_for_camera(&mut self, object_list: &[usize]) -> Vec<Polygon> {
        let mut ret: Vec<Polygon> = Vec::new();
        for &obj in object_list {
            ret.push(self.get_limit_area_for_camera(Some(obj)));
        }
        ret = union_(&ret);
        ret
    }

    /// TimelapsePosPicker.cpp:176-253 `Polygons TimelapsePosPicker::collect_limit_areas_for_rod(const std::vector<const PrintObject*>& object_list, const PosPickCtx& ctx)`
    ///
    /// scaled data
    fn collect_limit_areas_for_rod(&mut self, object_list: &[usize], ctx: &PosPickCtx) -> Vec<Polygon> {
        let print = self.print();
        let rod_limit_height: f64 = self.m_nozzle_height_to_rod as f64 + ctx.curr_layer.print_z;
        let mut rod_collision_candidates: Vec<usize> = Vec::new();
        for &obj in object_list {
            if let Some(printed_objects) = &ctx.printed_objects {
                if Some(&obj) == printed_objects.last() {
                    continue;
                }
            }
            let bbox = self.get_real_instance_bbox(&print.objects()[obj].instances()[0]);
            if bbox.max.z >= rod_limit_height {
                rod_collision_candidates.push(obj);
            }
        }

        if rod_collision_candidates.is_empty() {
            return Vec::new();
        }

        let mut collision_obj_bboxs: Vec<BoundingBoxf3> = Vec::new();
        for &obj in &rod_collision_candidates {
            let bbox = self.get_real_instance_bbox(&print.objects()[obj].instances()[0]);
            collision_obj_bboxs.push(self.expand_object_bbox(&bbox, self.m_print_seq == PrintSequence::ByObject));
        }

        crate::clipper::libcxx_sort(&mut collision_obj_bboxs, |lbbox, rbbox| {
            if lbbox.min.y == rbbox.min.y {
                return lbbox.max.y < rbbox.max.y;
            }
            lbbox.min.y < rbbox.min.y
        });

        let mut object_y_ranges: Vec<(i32, i32)> = vec![(0, 0)];
        for bbox in &collision_obj_bboxs {
            if object_y_ranges.last().unwrap().1 as f64 >= bbox.min.y {
                object_y_ranges.last_mut().unwrap().1 = bbox.max.y as i32;
            } else {
                object_y_ranges.push((bbox.min.y as i32, bbox.max.y as i32));
            }
        }

        if object_y_ranges.last().unwrap().1 < self.m_plate_height {
            object_y_ranges.push((self.m_plate_height, self.m_plate_height));
        }

        let mut lower_y_pos: i32 = -1;
        let mut upper_y_pos: i32 = -1;
        let unscaled_curr_pos = point_lrint(
            unscale_(ctx.curr_pos.x as f64) - self.m_plate_offset.x as f64,
            unscale_(ctx.curr_pos.y as f64) - self.m_plate_offset.y as f64,
        );

        for idx in 1..object_y_ranges.len() {
            if unscaled_curr_pos.y >= object_y_ranges[idx - 1].1 as i64 && unscaled_curr_pos.y <= object_y_ranges[idx].0 as i64 {
                lower_y_pos = object_y_ranges[idx - 1].1;
                upper_y_pos = object_y_ranges[idx].0;
                break;
            }
        }

        if lower_y_pos == -1 && upper_y_pos == -1 {
            return vec![self.m_bed_polygon.clone()];
        }

        let mut ret: Vec<Polygon> = Vec::new();

        ret.push(Polygon::from_points(vec![
            point_lrint(scale_(0.0), scale_(0.0)),
            point_lrint(scale_(self.m_plate_width as f64), scale_(0.0)),
            point_lrint(scale_(self.m_plate_width as f64), scale_(lower_y_pos as f64)),
            point_lrint(scale_(0.0), scale_(lower_y_pos as f64)),
        ]));

        ret.push(Polygon::from_points(vec![
            point_lrint(scale_(0.0), scale_(upper_y_pos as f64)),
            point_lrint(scale_(self.m_plate_width as f64), scale_(upper_y_pos as f64)),
            point_lrint(scale_(self.m_plate_width as f64), scale_(self.m_plate_height as f64)),
            point_lrint(scale_(0.0), scale_(self.m_plate_height as f64)),
        ]));
        ret
    }

    /// TimelapsePosPicker.cpp:258-276 `Polygon TimelapsePosPicker::expand_object_projection(const Polygon &poly, bool by_object, bool higher_than_curr)`
    ///
    /// expand the object expolygon by safe distance, scaled data
    fn expand_object_projection(&self, poly: &Polygon, by_object: bool, higher_than_curr: bool) -> Polygon {
        let radius: f32;
        let extruder_clearance_max_radius = cfg_float(self.print().config(), "extruder_clearance_max_radius");
        if by_object {
            if higher_than_curr {
                radius = scale_(extruder_clearance_max_radius) as f32;
            } else {
                radius = scale_(extruder_clearance_max_radius / 2.0) as f32;
            }
        } else {
            radius = scale_(extruder_clearance_max_radius / 2.0) as f32;
        }

        // the input poly is bounding box, so we get the first offseted polygon is ok
        // offset(const Polygon &polygon, const float delta): ClipperUtils.cpp:410-411
        let ret = paths_polygons(&raw_offset(
            &vec![polygon_path(poly)],
            radius as f64,
            JoinType::jtMiter,
            3.0,
            crate::clipper::EndType::etClosedPolygon,
        ));
        if ret.is_empty() {
            return Polygon::new();
        }
        ret[0].clone()
    }

    /// TimelapsePosPicker.cpp:279-294 `BoundingBoxf3 TimelapsePosPicker::expand_object_bbox(const BoundingBoxf3& bbox, bool by_object)`
    ///
    /// unscaled data
    fn expand_object_bbox(&self, bbox: &BoundingBoxf3, by_object: bool) -> BoundingBoxf3 {
        let radius: f32;
        let extruder_clearance_max_radius = cfg_float(self.print().config(), "extruder_clearance_max_radius");
        if by_object {
            radius = extruder_clearance_max_radius as f32;
        } else {
            radius = (extruder_clearance_max_radius / 2.0) as f32;
        }

        let mut ret = *bbox;
        ret.min.x -= radius as f64;
        ret.min.y -= radius as f64;
        ret.max.x += radius as f64;
        ret.max.y += radius as f64;

        ret
    }

    /// TimelapsePosPicker.cpp:297-319 `double TimelapsePosPicker::get_raft_height(const PrintObject* obj)`
    fn get_raft_height(&self, obj: Option<usize>) -> f64 {
        let print = self.print();
        let obj = match obj {
            Some(obj) if print.objects()[obj].has_raft() => &print.objects()[obj],
            _ => return 0.0,
        };
        let slice_params = obj.slicing_parameters().clone();
        let base_raft_layers: i32 = slice_params.base_raft_layers as i32;
        let base_raft_height: f64 = slice_params.base_raft_layer_height;
        let interface_raft_layers: i32 = slice_params.interface_raft_layers as i32;
        let interface_raft_height: f64 = slice_params.interface_raft_layer_height;
        let contact_raft_layer_height: f64 = slice_params.contact_raft_layer_height;

        // A multi-diameter plate printing with a single diameter lays its first layer at that nozzle's height.
        let mut ret: f64 = if print.single_diameter_layering_nozzle() > EPSILON {
            slice_params.first_print_layer_height
        } else {
            cfg_float(print.config(), "initial_layer_print_height")
        };
        if base_raft_layers - 1 > 0 {
            ret = ((base_raft_layers - 1) as f64).mul_add(base_raft_height, ret);
        }
        if interface_raft_layers - 1 > 0 {
            ret = ((interface_raft_layers - 1) as f64).mul_add(interface_raft_height, ret);
        }
        if obj.config().option("raft_layers").and_then(|o| o.get_int().ok()).unwrap_or(0) > 1 {
            ret += contact_raft_layer_height;
        }

        ret + slice_params.gap_raft_object
    }

    /// TimelapsePosPicker.cpp:322-340 `BoundingBoxf3 TimelapsePosPicker::get_real_instance_bbox(const PrintInstance& instance)`
    ///
    /// get the real instance bounding box, remove the plate offset and add raft height , unscaled data
    fn get_real_instance_bbox(&mut self, instance: &PrintInstance) -> BoundingBoxf3 {
        let key = instance as *const PrintInstance;
        if let Some(found) = self.bbox_cache.get(&key) {
            return *found;
        }

        let mut bbox = instance.get_bounding_box(self.print());
        let raft_height: f64 = self.get_raft_height(Some(instance.print_object));
        bbox.max.z += raft_height;
        // remove plate offset
        bbox.min.x -= self.m_plate_offset.x as f64;
        bbox.max.x -= self.m_plate_offset.x as f64;
        bbox.min.y -= self.m_plate_offset.y as f64;
        bbox.max.y -= self.m_plate_offset.y as f64;

        self.bbox_cache.insert(key, bbox);

        bbox
    }

    /// TimelapsePosPicker.cpp:342-358 `Polygon TimelapsePosPicker::get_limit_area_for_camera(const PrintObject* obj)`
    fn get_limit_area_for_camera(&mut self, obj: Option<usize>) -> Polygon {
        let obj = match obj {
            Some(obj) => obj,
            None => return Polygon::new(),
        };
        let print = self.print();
        let bbox = self.get_real_instance_bbox(&print.objects()[obj].instances()[0]);
        let radius: f32 = (self.m_nozzle_clearance_radius / 2) as f32;

        let offset_bbox = bbox.inflated(2.0f64.sqrt() * radius as f64);
        // Constrain the coordinates to the first quadrant.
        let ret = Polygon::from_points(vec![
            DEFAULT_CAMERA_POS,
            point_lrint(scale_(offset_bbox.max.x).max(0.0), scale_(offset_bbox.min.y).max(0.0)),
            point_lrint(scale_(offset_bbox.max.x).max(0.0), scale_(offset_bbox.max.y).max(0.0)),
            point_lrint(scale_(offset_bbox.min.x).max(0.0), scale_(offset_bbox.max.y).max(0.0)),
        ]);
        ret
    }
}

/// TimelapsePosPicker.cpp:371-453 `Point pick_pos_internal(const Point& curr_pos, const ExPolygons& safe_areas, const ExPolygons& path_collision_area, bool detect_path_collision, const std::optional<Point>& farthest_point = std::nullopt)`
///
/// Selects the nearest position within the given safe areas relative to the
/// current position.
///
/// This function determines the closest point in the safe areas to the provided
/// current position. If the current position is already inside a safe area, it
/// returns the current position. If no safe areas are defined, return default
/// timelapse position.
pub fn pick_pos_internal(
    curr_pos: &Point,
    safe_areas: &ExPolygons,
    path_collision_area: &ExPolygons,
    detect_path_collision: bool,
    farthest_point: &Option<Point>,
) -> Point {
    // struct CandidatePoint { double dist; Point point; bool operator<(other) const { return dist < other.dist; } };

    if safe_areas.iter().any(|p| p.contains(curr_pos, true)) {
        return *curr_pos;
    }

    if safe_areas.is_empty() {
        return DEFAULT_TIMELAPSE_POS;
    }

    // std::priority_queue<CandidatePoint> max_heap;
    let mut max_heap: Vec<CandidatePoint> = Vec::new();

    let candidate_point_segment: f64 = scale_(5.0);
    let weight_of_camera: f64 = 1.0 / 3.0;
    let penalty_func = |curr_post: &Point, camera_pos: &Point, candidatet: &Point| -> f64 {
        if let Some(farthest_point) = farthest_point {
            // Prefer candidate closest to the farthest point (L1 norm)
            return ((farthest_point.x - candidatet.x).abs() + (farthest_point.y - candidatet.y).abs()) as f64;
        }
        // move distance + Camera occlusion penalty function
        let ret_pen: f64 = (-weight_of_camera).mul_add(
            ((camera_pos.x - candidatet.x).abs() + (camera_pos.y - candidatet.y).abs()) as f64,
            ((curr_post.x - candidatet.x).abs() + (curr_post.y - candidatet.y).abs()) as f64,
        );
        ret_pen
    };

    for expoly in safe_areas {
        let polys = expoly.to_polygons();
        for poly in &polys {
            for idx in 0..poly.points.len() {
                let mut best_penalty: f64 = f64::MAX;
                let mut best_candidate: Point = DEFAULT_TIMELAPSE_POS; // the best candidate form current line
                //std::vector<Point> candidate_source;
                let next = poly.points[next_idx_modulo(idx, poly.points.len())];
                if (((poly.points[idx].x - next.x).abs() + (poly.points[idx].y - next.y).abs()) as f64) < candidate_point_segment {
                    best_candidate = poly.points[idx]; // only check the start point if the line is short
                    best_penalty = penalty_func(curr_pos, &DEFAULT_CAMERA_POS, &best_candidate);
                } else {
                    let mut direct_of_line = Point::new(next.x - poly.points[idx].x, next.y - poly.points[idx].y);
                    let length_l1: f64 = (direct_of_line.x.abs() + direct_of_line.y.abs()) as f64;
                    let num_steps: i32 = (length_l1 / candidate_point_segment) as i32; // for long line use 5mm segmentation to check
                    // devide by length_L1 instead of steps, prevent lose accuracy for the step length
                    direct_of_line.x = (direct_of_line.x as f64 * candidate_point_segment / length_l1) as i64;
                    direct_of_line.y = (direct_of_line.y as f64 * candidate_point_segment / length_l1) as i64;
                    for line_seg_i in 0..=num_steps {
                        let candidate = Point::new(
                            poly.points[idx].x + direct_of_line.x * line_seg_i as i64,
                            poly.points[idx].y + direct_of_line.y * line_seg_i as i64,
                        );
                        let dist: f64 = penalty_func(curr_pos, &DEFAULT_CAMERA_POS, &candidate);
                        if dist < best_penalty {
                            best_penalty = dist;
                            best_candidate = candidate;
                        } //only push the best point into heap for the whole line
                    }
                }
                heap_push(&mut max_heap, CandidatePoint { dist: best_penalty, point: best_candidate });
                if max_heap.len() > MAX_CANDIDATE_SIZE {
                    heap_pop(&mut max_heap);
                }
            }
        }
    }

    let mut top_candidates: Vec<Point> = Vec::new();
    while !max_heap.is_empty() {
        top_candidates.push(max_heap[0].point);
        heap_pop(&mut max_heap);
    }
    top_candidates.reverse();

    for p in &top_candidates {
        if !detect_path_collision {
            return *p;
        }

        let path = Polyline::from_points(vec![*curr_pos, *p]);

        if intersection_pl_clib(std::slice::from_ref(&path), path_collision_area).is_empty() {
            return *p;
        }
    }

    DEFAULT_TIMELAPSE_POS
}

/// TimelapsePosPicker.cpp:374-381 `struct CandidatePoint` of `pick_pos_internal`: `operator<` compares
/// `dist`, so `std::priority_queue<CandidatePoint>` is a max-heap on `dist`.
#[derive(Clone, Copy)]
struct CandidatePoint {
    dist: f64,
    point: Point,
}

/// libc++ push_heap.h `__sift_up(first, last, comp, len)` on `v[..last]`, with `operator<` of `CandidatePoint`.
fn heap_sift_up(v: &mut [CandidatePoint], mut last: usize, mut len: usize) {
    if len > 1 {
        len = (len - 2) / 2;
        let mut ptr = len;
        last -= 1;
        if v[ptr].dist < v[last].dist {
            let t = v[last];
            loop {
                v[last] = v[ptr];
                last = ptr;
                if len == 0 {
                    break;
                }
                len = (len - 1) / 2;
                ptr = len;
                if !(v[ptr].dist < t.dist) {
                    break;
                }
            }
            v[last] = t;
        }
    }
}

/// `std::priority_queue::push`: `c.push_back(v); std::push_heap(c.begin(), c.end(), comp);`
fn heap_push(heap: &mut Vec<CandidatePoint>, value: CandidatePoint) {
    heap.push(value);
    let len = heap.len();
    heap_sift_up(heap, len, len);
}

/// `std::priority_queue::pop`: `std::pop_heap(c.begin(), c.end(), comp); c.pop_back();`, with libc++'s
/// pop_heap.h `__pop_heap` (sift_down.h `__floyd_sift_down`, then `__sift_up`).
fn heap_pop(heap: &mut Vec<CandidatePoint>) {
    let len = heap.len();
    if len > 1 {
        let top = heap[0];
        // __floyd_sift_down(first, comp, len)
        let mut hole = 0usize;
        let mut child_i = 0usize;
        let mut child = 0usize;
        loop {
            child_i += child + 1;
            child = 2 * child + 1;
            if child + 1 < len && heap[child_i].dist < heap[child_i + 1].dist {
                child_i += 1;
                child += 1;
            }
            heap[hole] = heap[child_i];
            hole = child_i;
            if child > (len - 2) / 2 {
                break;
            }
        }
        let last = len - 1;
        if hole == last {
            heap[hole] = top;
        } else {
            heap[hole] = heap[last];
            hole += 1;
            heap[last] = top;
            heap_sift_up(heap, hole, hole);
        }
    }
    heap.pop();
}

impl<'p> TimelapsePosPicker<'p> {
    /// TimelapsePosPicker.cpp:455-464 `Point TimelapsePosPicker::pick_pos(const PosPickCtx& ctx)`
    pub fn pick_pos(&mut self, ctx: &PosPickCtx) -> Point {
        let res: Point;
        if self.m_based_on_all_layer {
            res = self.pick_pos_for_all_layer(ctx);
        } else {
            res = self.pick_pos_for_curr_layer(ctx);
        }

        point_lrint(unscale_(res.x as f64), unscale_(res.y as f64))
    }

    /// TimelapsePosPicker.cpp:467-478 `Point TimelapsePosPicker::get_object_center(const PrintObject* obj)`
    ///
    /// get center point of curr object, scaled data
    fn get_object_center(&mut self, obj: Option<usize>) -> Point {
        let obj = match obj {
            Some(obj) => obj,
            None => return Point::new(0, 0),
        };
        let print = self.print();
        // in bambu studio, each object only has one instance
        let instance = &print.objects()[obj].instances()[0];
        let instance_bbox = self.get_real_instance_bbox(instance);
        let min_p = point_lrint(instance_bbox.min.x, instance_bbox.min.y);
        let max_p = point_lrint(instance_bbox.max.x, instance_bbox.max.y);

        point_lrint(scale_(((min_p.x + max_p.x) / 2) as f64), scale_(((min_p.y + max_p.y) / 2) as f64))
    }

    /// TimelapsePosPicker.cpp:481-496 `Point TimelapsePosPicker::pick_nearest_object_center(const Point& curr_pos, const std::vector<const PrintObject*>& object_list)`
    ///
    /// scaled data
    #[allow(dead_code)]
    fn pick_nearest_object_center(&mut self, curr_pos: &Point, object_list: &[usize]) -> Point {
        if object_list.is_empty() {
            return Point::new(0, 0);
        }
        let mut ptr: usize = object_list[0];
        let mut distance: f64 = f64::MAX;
        for &obj in object_list {
            let obj_center = self.get_object_center(Some(obj));
            let dx = (obj_center.x - curr_pos.x) as f64;
            let dy = (obj_center.y - curr_pos.y) as f64;
            // `(obj_center - curr_pos).cast<double>().norm()`: Eigen's packet of two doubles, no contraction.
            let dist: f64 = (dx * dx + dy * dy).sqrt();
            if distance > dist {
                distance = dist;
                ptr = obj;
            }
        }
        self.get_object_center(Some(ptr))
    }

    /// TimelapsePosPicker.cpp:499-538 `Point TimelapsePosPicker::pick_pos_for_curr_layer(const PosPickCtx& ctx)`
    ///
    /// scaled data
    fn pick_pos_for_curr_layer(&mut self, ctx: &PosPickCtx) -> Point {
        let mut height_gap: f32 = 0.0;
        if ctx.curr_extruder_id != ctx.picture_extruder_id {
            if self.m_liftable_extruder_id.is_some()
                && Some(ctx.picture_extruder_id) != self.m_liftable_extruder_id
                && self.m_extruder_height_gap.is_some()
            {
                height_gap = -self.m_extruder_height_gap.unwrap() as f32;
            }
        }

        let by_object = self.m_print_seq == PrintSequence::ByObject;
        let object_list: Vec<usize> = self.get_object_list(&ctx.printed_objects);

        let layer_slices: ExPolygons = self.collect_object_slices_data(ctx.curr_layer, height_gap, &object_list, by_object);
        let camera_limit_areas: Vec<Polygon> = self.collect_limit_areas_for_camera(&object_list);
        let mut rod_limit_areas: Vec<Polygon> = Vec::new();
        if by_object && ctx.printed_objects.as_ref().is_some_and(|p| !p.is_empty()) {
            rod_limit_areas = self.collect_limit_areas_for_rod(&object_list, ctx);
        }
        let unplacable_area: ExPolygons =
            union_ex_expolygons_polygons(&union_ex_expolygons_polygons(&layer_slices, &camera_limit_areas), &rod_limit_areas);
        let extruder_printable_area: ExPolygons = self.m_extruder_printable_area[ctx.curr_extruder_id as usize].clone();

        // diff_ex(const ExPolygons &subject, const ExPolygons &clip): ClipperUtils.cpp:755-756
        let mut safe_area: ExPolygons = _clipper_ex(
            ClipType::ctDifference,
            &expolygons_paths(&extruder_printable_area),
            &expolygons_paths(&unplacable_area),
            ApplySafetyOffset::No,
            PolyFillType::pftNonZero,
        );
        safe_area = opening_ex(&safe_area, scale_(FILTER_THRESHOLD as f64) as f32);

        let center_p: Point;
        if by_object && ctx.printed_objects.as_ref().is_some_and(|p| !p.is_empty()) {
            center_p = self.get_object_center(ctx.printed_objects.as_ref().unwrap().last().copied());
        } else {
            center_p = self.get_objects_center(&object_list);
        }

        let mut path_collision_area: ExPolygons = ExPolygons::new();
        if by_object && ctx.printed_objects.as_ref().is_some_and(|p| !p.is_empty()) {
            let mut object_without_curr = ctx.printed_objects.clone();
            if let Some(object_without_curr) = object_without_curr.as_mut() {
                if !object_without_curr.is_empty() {
                    object_without_curr.pop();
                }
            }

            let object_list_without_curr = self.get_object_list(&object_without_curr);
            let layer_slices_without_curr: ExPolygons =
                self.collect_object_slices_data(ctx.curr_layer, height_gap, &object_list_without_curr, by_object);
            path_collision_area = union_ex_expolygons_polygons(&layer_slices_without_curr, &rod_limit_areas);
        }

        pick_pos_internal(&center_p, &safe_area, &path_collision_area, by_object, &ctx.farthest_point)
    }

    /// TimelapsePosPicker.cpp:549-569 `Point TimelapsePosPicker::get_objects_center(const std::vector<const PrintObject*>& object_list)`
    ///
    /// Calculates the center of multiple objects: the average center of all instances of the provided
    /// objects. Scaled data.
    fn get_objects_center(&mut self, object_list: &[usize]) -> Point {
        if object_list.is_empty() {
            return Point::new(0, 0);
        }
        let mut sum_x: f64 = 0.0;
        let mut sum_y: f64 = 0.0;
        let mut total_instances: usize = 0;
        let print = self.print();
        for &obj in object_list {
            for instance in print.objects()[obj].instances() {
                let bbox = self.get_real_instance_bbox(instance);
                let min_p = point_lrint(bbox.min.x, bbox.min.y);
                let max_p = point_lrint(bbox.max.x, bbox.max.y);
                let center_x: f64 = ((min_p.x + max_p.x) as f32 / 2.0f32) as f64;
                let center_y: f64 = ((min_p.y + max_p.y) as f32 / 2.0f32) as f64;
                sum_x += center_x;
                sum_y += center_y;
                total_instances += 1;
            }
        }
        Point::new(scale_(sum_x / total_instances as f64) as i64, scale_(sum_y / total_instances as f64) as i64)
    }

    /// TimelapsePosPicker.cpp:571-622 `Point TimelapsePosPicker::pick_pos_for_all_layer(const PosPickCtx& ctx)`
    fn pick_pos_for_all_layer(&mut self, ctx: &PosPickCtx) -> Point {
        let by_object = self.m_print_seq == PrintSequence::ByObject;
        if by_object {
            return DEFAULT_TIMELAPSE_POS;
        }

        // In smooth timelapse both nozzles share a single fixed picture position. While the
        // current layer is still below the dual-nozzle height gap, the idle (lower) nozzle can
        // collide with printed parts, so always retreat to the trash bin position regardless of
        // which extruder takes the picture. Traditional (per-layer) mode is handled separately.
        if self.m_extruder_height_gap.is_some() && ctx.curr_layer.print_z <= self.m_extruder_height_gap.unwrap() as f64 {
            return DEFAULT_TIMELAPSE_POS;
        }
        if let Some(pos) = self.m_all_layer_pos {
            return pos;
        }

        let mut object_projections: Vec<Polygon> = Vec::new();

        let object_list = self.get_object_list(&None);

        let print = self.print();
        for &obj in &object_list {
            for instance in print.objects()[obj].instances() {
                let bbox = self.get_real_instance_bbox(instance);
                let min_p = point_lrint(scale_(bbox.min.x), scale_(bbox.min.y));
                let max_p = point_lrint(scale_(bbox.max.x), scale_(bbox.max.y));
                let obj_proj = Polygon::from_points(vec![
                    Point::new(min_p.x, min_p.y),
                    Point::new(max_p.x, min_p.y),
                    Point::new(max_p.x, max_p.y),
                    Point::new(min_p.x, max_p.y),
                ]);
                object_projections.push(self.expand_object_projection(&obj_proj, by_object, true));
            }
        }

        object_projections = union_(&object_projections);
        let camera_limit_areas: Vec<Polygon> = self.collect_limit_areas_for_camera(&object_list);
        // union_(const Polygons &subject, const Polygons &subject2): ClipperUtils.cpp:727-734
        let unplacable_area: Vec<Polygon> = {
            let mut polys = object_projections.clone();
            for poly in &camera_limit_areas {
                polys.push(poly.clone());
            }
            union_(&polys)
        };

        let mut extruder_printable_area: ExPolygons = ExPolygons::new();
        if self.m_extruder_printable_area.len() > 1 {
            // intersection_ex(const ExPolygons &subject, const ExPolygons &clip): ClipperUtils.cpp:802-803
            extruder_printable_area = _clipper_ex(
                ClipType::ctIntersection,
                &expolygons_paths(&self.m_extruder_printable_area[0]),
                &expolygons_paths(&self.m_extruder_printable_area[1]),
                ApplySafetyOffset::No,
                PolyFillType::pftNonZero,
            );
        } else if self.m_extruder_printable_area.len() == 1 {
            extruder_printable_area = self.m_extruder_printable_area[0].clone();
        }

        // diff_ex(const ExPolygons &subject, const Polygons &clip): ClipperUtils.cpp:753-754
        let mut safe_area: ExPolygons = _clipper_ex(
            ClipType::ctDifference,
            &expolygons_paths(&extruder_printable_area),
            &polygons_paths(&unplacable_area),
            ApplySafetyOffset::No,
            PolyFillType::pftNonZero,
        );
        safe_area = opening_ex(&safe_area, scale_(FILTER_THRESHOLD as f64) as f32);

        let starting_pos: Point = self.get_objects_center(&object_list);

        self.m_all_layer_pos = Some(pick_pos_internal(&starting_pos, &safe_area, &ExPolygons::new(), by_object, &None));
        self.m_all_layer_pos.unwrap()
    }

    /// TimelapsePosPicker.cpp:624-665 `bool TimelapsePosPicker::get_is_clear_to_x0(const PosPickCtx &ctx)`
    pub fn get_is_clear_to_x0(&mut self, ctx: &PosPickCtx) -> bool {
        let by_object = self.m_print_seq == PrintSequence::ByObject;
        let object_list: Vec<usize> = self.get_object_list(&ctx.printed_objects);

        let range_intersect = |left1: i32, right1: i32, left2: i32, right2: i32| -> bool {
            if left1 <= left2 && left2 <= right1 {
                return true;
            }
            if left2 <= left1 && left1 <= right2 {
                return true;
            }
            false
        };

        let mut unclear_area: ExPolygons = ExPolygons::new();
        let layer: &Layer = ctx.curr_layer;
        let z_target: f32 = layer.print_z as f32;
        let z_low: f32 = (layer.print_z - 0.5) as f32;
        let z_high: f32 = (layer.print_z + 0.5) as f32;

        let print = self.print();
        for &obj in &object_list {
            for instance in print.objects()[obj].instances() {
                let instance_bbox = self.get_real_instance_bbox(instance);
                let is_curr_obj = (Some(&obj) == object_list.last()) || (!by_object);
                let higher_than_curr_pos = instance_bbox.max.z > z_target as f64;
                if !is_curr_obj && range_intersect(instance_bbox.min.z as i32, instance_bbox.max.z as i32, z_low as i32, z_high as i32) {
                    let mut expoly = ExPolygon::default();
                    expoly.contour = Polygon::from_points(vec![
                        point_lrint(scale_(instance_bbox.min.x), scale_(instance_bbox.min.y)),
                        point_lrint(scale_(instance_bbox.max.x), scale_(instance_bbox.min.y)),
                        point_lrint(scale_(instance_bbox.max.x), scale_(instance_bbox.max.y)),
                        point_lrint(scale_(instance_bbox.min.x), scale_(instance_bbox.max.y)),
                    ]);
                    expoly.contour = self.expand_object_projection(&expoly.contour, by_object, higher_than_curr_pos);
                    unclear_area.push(expoly);
                }
            }
        }

        let curr_pos_in_plate = point_lrint(
            ctx.curr_pos.x as f64 - scale_(self.m_plate_offset.x as f64),
            ctx.curr_pos.y as f64 - scale_(self.m_plate_offset.y as f64),
        );
        for expoly in &unclear_area {
            let bbox = expoly.contour.bounding_box();
            if curr_pos_in_plate.y < bbox.min.y || curr_pos_in_plate.y > bbox.max.y {
                continue;
            }
            if bbox.min.x <= curr_pos_in_plate.x {
                return false;
            }
        }

        true
    }
}

/// `config.<key>.values` of a points option (`Pointfs`).
fn cfg_points(config: &crate::print::faithful::PrintConfig, key: &str) -> Vec<crate::geometry::Vec2d> {
    config.option(key).and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionPoints>()).map(|o| o.values.clone()).unwrap_or_default()
}

/// `config.<key>.values` of a points-groups option (`std::vector<Pointfs>`).
fn cfg_points_groups(config: &crate::print::faithful::PrintConfig, key: &str) -> Vec<Vec<crate::geometry::Vec2d>> {
    config
        .option(key)
        .and_then(|o| o.as_any().downcast_ref::<crate::config::ConfigOptionPointsGroups>())
        .map(|o| o.values.clone())
        .unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{ExPolygon, Polygon};
    use crate::scale;

    /// A point already inside a safe area returns itself (cpp:383-384).
    #[test]
    fn test_pick_pos_internal_inside_returns_curr() {
        let square = Polygon {
            points: vec![
                Point::new(scale(0.0), scale(0.0)),
                Point::new(scale(100.0), scale(0.0)),
                Point::new(scale(100.0), scale(100.0)),
                Point::new(scale(0.0), scale(100.0)),
            ],
        };
        let safe = vec![ExPolygon::new(square)];
        let curr = Point::new(scale(50.0), scale(50.0));
        let res = pick_pos_internal(&curr, &safe, &Vec::new(), false, &None);
        assert_eq!(res, curr);
    }

    /// Empty safe areas yield the default timelapse position (cpp:386-387).
    #[test]
    fn test_pick_pos_internal_empty_returns_default() {
        let curr = Point::new(scale(10.0), scale(10.0));
        let res = pick_pos_internal(&curr, &Vec::new(), &Vec::new(), false, &None);
        assert_eq!(res, DEFAULT_TIMELAPSE_POS);
    }

    /// With a safe area not containing curr and no path-collision detection, a
    /// boundary point of the safe area is returned (cpp:398-440).
    #[test]
    fn test_pick_pos_internal_outside_returns_boundary() {
        let square = Polygon {
            points: vec![
                Point::new(scale(10.0), scale(10.0)),
                Point::new(scale(20.0), scale(10.0)),
                Point::new(scale(20.0), scale(20.0)),
                Point::new(scale(10.0), scale(20.0)),
            ],
        };
        let safe = vec![ExPolygon::new(square)];
        let curr = Point::new(scale(0.0), scale(0.0));
        let res = pick_pos_internal(&curr, &safe, &Vec::new(), false, &None);
        // Must be a real candidate, not the fallback default.
        assert_ne!(res, DEFAULT_TIMELAPSE_POS);
    }
}
