//! Faithful 1:1 line-by-line port of BambuStudio `Support/TreeSupportCommon.hpp`.
//!
//! C++ Reference:
//! - `src/libslic3r/Support/TreeSupportCommon.hpp`
//!
//! This is a header-only C++ unit (`namespace Slic3r::TreeSupport3D`). It defines
//! the tree-support configuration structs (`TreeSupportMeshGroupSettings`,
//! `TreeSupportSettings`), the `InterfacePlacer` helper, the free `layer_*`
//! helpers, and the `InterfacePreference` / `LineStatus` enums.
//!
//! Type mapping: `coord_t` -> `i64` (`Coord`), `coordf_t` -> `f64` (`CoordF`),
//! `LayerIndex` (C++ `int`) -> `i32` here (`LayerIndex`).
//!
//! `scaled<coord_t>(v)` in C++ is `coord_t(v / SCALING_FACTOR)` — a *truncating*
//! conversion (see Point.hpp:537-540). The crate's `scale()` rounds, so this file
//! uses the local `scaled_coord()` truncating helper to stay byte-exact.

use crate::flow::{support_material_flow, support_material_interface_flow, FlowRole};
use crate::geometry::Polygons;
use crate::libslic3r::EPSILON;
use crate::print_config::SupportInterfacePattern;
use crate::print_object::PrintObject;
use crate::slicing::SlicingParams;
use crate::support::support_layer::{
    SupporLayerType, SupportGeneratorLayer, SupportGeneratorLayerStorage, SupportGeneratorLayersPtr,
};
use crate::support::support_parameters::SupportParameters;
use crate::{Coord, CoordF, SCALING_FACTOR};

// TreeSupportCommon.hpp:14
// The number of vertices in each circle.
pub const SUPPORT_TREE_CIRCLE_RESOLUTION: usize = 25;

// TreeSupportCommon.hpp:17  using LayerIndex = int;
pub type LayerIndex = i32;

// PrintConfig.hpp:156-157
//   enum SupportMaterialInterfacePattern {
//       smipAuto, smipRectilinear, smipConcentric, smipRectilinearInterlaced, smipGrid };
// `TreeSupportCommon.hpp` uses this enum (via PrintConfig.hpp) as the type of
// `support_roof_pattern` / `roof_pattern`. The enum's canonical home is the
// not-yet-ported Rust `print_config`; it is mirrored here so this unit can be a
// faithful, self-contained translation. `#[default]` is `smipAuto` (enum value 0),
// matching the `{ smipAuto }` default member initializer (TreeSupportCommon.hpp:168).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(non_camel_case_types)]
pub enum SupportMaterialInterfacePattern {
    #[default]
    smipAuto,
    smipRectilinear,
    smipConcentric,
    smipRectilinearInterlaced,
    smipGrid,
}

// Truncating `scaled<coord_t>(v)` (Point.hpp:537-540): `coord_t(v / SCALING_FACTOR)`.
// SCALING_FACTOR here is the crate's `1/0.00001 = 100000` form, so divide-by becomes
// multiply, then truncate toward zero (C++ static_cast to integer).
#[inline]
fn scaled_coord(v: CoordF) -> Coord {
    (v * SCALING_FACTOR) as Coord
}

// Floating-point `scaled<double>(v)` (Point.hpp:527-530): `double(v / SCALING_FACTOR)`.
#[inline]
fn scaled_f64(v: CoordF) -> CoordF {
    v * SCALING_FACTOR
}

// TreeSupportCommon.hpp:19-26
// enum class InterfacePreference
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InterfacePreference {
    #[default]
    InterfaceAreaOverwritesSupport,
    SupportAreaOverwritesInterface,
    InterfaceLinesOverwriteSupport,
    SupportLinesOverwriteInterface,
    Nothing,
}

// TreeSupportCommon.hpp:28  struct TreeSupportMeshGroupSettings {
//
// The `explicit TreeSupportMeshGroupSettings(const PrintObject&)` constructor
// (TreeSupportCommon.hpp:30-90) is ported as `from_print_object` below.
// The struct itself and all of its default member initializers are also ported.
#[derive(Debug, Clone, PartialEq)]
pub struct TreeSupportMeshGroupSettings {
    /*********************************************************************/
    /* Print parameters, not support specific:                           */
    /*********************************************************************/
    // TreeSupportCommon.hpp:95  coord_t layer_height { scaled<coord_t>(0.15) };
    pub layer_height: Coord,
    // TreeSupportCommon.hpp:100  coord_t resolution { scaled<coord_t>(0.025) };
    pub resolution: Coord,
    // TreeSupportCommon.hpp:104  coord_t min_feature_size { scaled<coord_t>(0.1) };
    pub min_feature_size: Coord,

    /*********************************************************************/
    /* General support parameters:                                       */
    /*********************************************************************/
    // TreeSupportCommon.hpp:112  double support_angle { 50. * M_PI / 180. };
    pub support_angle: f64,
    // TreeSupportCommon.hpp:115  coord_t support_line_width { scaled<coord_t>(0.4) };
    pub support_line_width: Coord,
    // TreeSupportCommon.hpp:117  coord_t support_roof_line_width { scaled<coord_t>(0.4) };
    pub support_roof_line_width: Coord,
    // TreeSupportCommon.hpp:120  bool support_bottom_enable { false };
    pub support_bottom_enable: bool,
    // TreeSupportCommon.hpp:123  coord_t support_bottom_height { scaled<coord_t>(1.) };
    pub support_bottom_height: Coord,
    // TreeSupportCommon.hpp:124  bool support_material_buildplate_only { false };
    pub support_material_buildplate_only: bool,
    // TreeSupportCommon.hpp:128  coord_t support_xy_distance { scaled<coord_t>(0.7) };
    pub support_xy_distance: Coord,
    // TreeSupportCommon.hpp:129  coord_t support_xy_distance_1st_layer { scaled<coord_t>(0.7) };
    pub support_xy_distance_1st_layer: Coord,
    // TreeSupportCommon.hpp:133  coord_t support_xy_distance_overhang { scaled<coord_t>(0.2) };
    pub support_xy_distance_overhang: Coord,
    // TreeSupportCommon.hpp:136  coord_t support_top_distance { scaled<coord_t>(0.1) };
    pub support_top_distance: Coord,
    // TreeSupportCommon.hpp:139  coord_t support_bottom_distance { scaled<coord_t>(0.1) };
    pub support_bottom_distance: Coord,
    // TreeSupportCommon.hpp:143  coord_t support_interface_skip_height { scaled<coord_t>(0.3) };
    pub support_interface_skip_height: Coord,
    // TreeSupportCommon.hpp:151  bool support_roof_enable { false };
    pub support_roof_enable: bool,
    // TreeSupportCommon.hpp:154  coord_t support_roof_layers{ 2 };
    pub support_roof_layers: Coord,
    // TreeSupportCommon.hpp:155  bool support_floor_enable{ false };
    pub support_floor_enable: bool,
    // TreeSupportCommon.hpp:156  coord_t support_floor_layers{ 2 };
    pub support_floor_layers: Coord,
    // TreeSupportCommon.hpp:160  double minimum_roof_area { scaled<double>(scaled<double>(1.)) };
    pub minimum_roof_area: f64,
    // TreeSupportCommon.hpp:165  std::vector<double> support_roof_angles {};
    pub support_roof_angles: Vec<f64>,
    // TreeSupportCommon.hpp:168  SupportMaterialInterfacePattern support_roof_pattern { smipAuto };
    pub support_roof_pattern: SupportMaterialInterfacePattern,
    // TreeSupportCommon.hpp:171  coord_t support_line_spacing { scaled<coord_t>(2.66 - 0.4) };
    pub support_line_spacing: Coord,
    // TreeSupportCommon.hpp:174  coord_t support_bottom_offset { scaled<coord_t>(0.) };
    pub support_bottom_offset: Coord,
    // TreeSupportCommon.hpp:179  int support_wall_count { 1 };
    pub support_wall_count: i32,
    // TreeSupportCommon.hpp:182  coord_t support_roof_line_distance { scaled<coord_t>(0.4) };
    pub support_roof_line_distance: Coord,
    // TreeSupportCommon.hpp:185  coord_t minimum_support_area { scaled<coord_t>(0.) };
    pub minimum_support_area: Coord,
    // TreeSupportCommon.hpp:188  coord_t minimum_bottom_area { scaled<coord_t>(1.0) };
    pub minimum_bottom_area: Coord,
    // TreeSupportCommon.hpp:191  coord_t support_offset { scaled<coord_t>(0.) };
    pub support_offset: Coord,

    /*********************************************************************/
    /* Parameters for the Cura tree supports implementation:             */
    /*********************************************************************/
    // TreeSupportCommon.hpp:200  double support_tree_angle { 60. * M_PI / 180. };
    pub support_tree_angle: f64,
    // TreeSupportCommon.hpp:205  double support_tree_branch_diameter_angle { 5. * M_PI / 180. };
    pub support_tree_branch_diameter_angle: f64,
    // TreeSupportCommon.hpp:209  coord_t support_tree_branch_distance { scaled<coord_t>(1.) };
    pub support_tree_branch_distance: Coord,
    // TreeSupportCommon.hpp:213  coord_t support_tree_branch_diameter { scaled<coord_t>(2.) };
    pub support_tree_branch_diameter: Coord,

    /*********************************************************************/
    /* Parameters new to the Thomas Rahm's tree supports implementation: */
    /*********************************************************************/
    // TreeSupportCommon.hpp:222  double support_tree_angle_slow { 50. * M_PI / 180. };
    pub support_tree_angle_slow: f64,
    // TreeSupportCommon.hpp:227  coord_t support_tree_max_diameter_increase_by_merges_when_support_to_model { scaled<coord_t>(1.0) };
    pub support_tree_max_diameter_increase_by_merges_when_support_to_model: Coord,
    // TreeSupportCommon.hpp:231  coord_t support_tree_min_height_to_model { scaled<coord_t>(1.0) };
    pub support_tree_min_height_to_model: Coord,
    // TreeSupportCommon.hpp:235  coord_t support_tree_bp_diameter { scaled<coord_t>(7.5) };
    pub support_tree_bp_diameter: Coord,
    // TreeSupportCommon.hpp:240  double support_tree_top_rate { 15. };
    pub support_tree_top_rate: f64,
    // TreeSupportCommon.hpp:244  coord_t support_tree_tip_diameter { scaled<coord_t>(0.4) };
    pub support_tree_tip_diameter: Coord,
}

impl Default for TreeSupportMeshGroupSettings {
    // TreeSupportCommon.hpp:29  TreeSupportMeshGroupSettings() = default;
    // The default member initializers (TreeSupportCommon.hpp:95-244) are reproduced
    // exactly. `smipAuto` is the default `SupportMaterialInterfacePattern`.
    fn default() -> Self {
        use std::f64::consts::PI as M_PI;
        Self {
            // TreeSupportCommon.hpp:95
            layer_height: scaled_coord(0.15),
            // TreeSupportCommon.hpp:100
            resolution: scaled_coord(0.025),
            // TreeSupportCommon.hpp:104
            min_feature_size: scaled_coord(0.1),
            // TreeSupportCommon.hpp:112
            support_angle: 50. * M_PI / 180.,
            // TreeSupportCommon.hpp:115
            support_line_width: scaled_coord(0.4),
            // TreeSupportCommon.hpp:117
            support_roof_line_width: scaled_coord(0.4),
            // TreeSupportCommon.hpp:120
            support_bottom_enable: false,
            // TreeSupportCommon.hpp:123
            support_bottom_height: scaled_coord(1.),
            // TreeSupportCommon.hpp:124
            support_material_buildplate_only: false,
            // TreeSupportCommon.hpp:128
            support_xy_distance: scaled_coord(0.7),
            // TreeSupportCommon.hpp:129
            support_xy_distance_1st_layer: scaled_coord(0.7),
            // TreeSupportCommon.hpp:133
            support_xy_distance_overhang: scaled_coord(0.2),
            // TreeSupportCommon.hpp:136
            support_top_distance: scaled_coord(0.1),
            // TreeSupportCommon.hpp:139
            support_bottom_distance: scaled_coord(0.1),
            // TreeSupportCommon.hpp:143
            support_interface_skip_height: scaled_coord(0.3),
            // TreeSupportCommon.hpp:151
            support_roof_enable: false,
            // TreeSupportCommon.hpp:154
            support_roof_layers: 2,
            // TreeSupportCommon.hpp:155
            support_floor_enable: false,
            // TreeSupportCommon.hpp:156
            support_floor_layers: 2,
            // TreeSupportCommon.hpp:160  scaled<double>(scaled<double>(1.))
            minimum_roof_area: scaled_f64(scaled_f64(1.)),
            // TreeSupportCommon.hpp:165
            support_roof_angles: Vec::new(),
            // TreeSupportCommon.hpp:168  smipAuto
            support_roof_pattern: SupportMaterialInterfacePattern::smipAuto,
            // TreeSupportCommon.hpp:171  scaled<coord_t>(2.66 - 0.4)
            support_line_spacing: scaled_coord(2.66 - 0.4),
            // TreeSupportCommon.hpp:174
            support_bottom_offset: scaled_coord(0.),
            // TreeSupportCommon.hpp:179
            support_wall_count: 1,
            // TreeSupportCommon.hpp:182
            support_roof_line_distance: scaled_coord(0.4),
            // TreeSupportCommon.hpp:185
            minimum_support_area: scaled_coord(0.),
            // TreeSupportCommon.hpp:188
            minimum_bottom_area: scaled_coord(1.0),
            // TreeSupportCommon.hpp:191
            support_offset: scaled_coord(0.),
            // TreeSupportCommon.hpp:200
            support_tree_angle: 60. * M_PI / 180.,
            // TreeSupportCommon.hpp:205
            support_tree_branch_diameter_angle: 5. * M_PI / 180.,
            // TreeSupportCommon.hpp:209
            support_tree_branch_distance: scaled_coord(1.),
            // TreeSupportCommon.hpp:213
            support_tree_branch_diameter: scaled_coord(2.),
            // TreeSupportCommon.hpp:222
            support_tree_angle_slow: 50. * M_PI / 180.,
            // TreeSupportCommon.hpp:227
            support_tree_max_diameter_increase_by_merges_when_support_to_model: scaled_coord(1.0),
            // TreeSupportCommon.hpp:231
            support_tree_min_height_to_model: scaled_coord(1.0),
            // TreeSupportCommon.hpp:235
            support_tree_bp_diameter: scaled_coord(7.5),
            // TreeSupportCommon.hpp:240
            support_tree_top_rate: 15.,
            // TreeSupportCommon.hpp:244
            support_tree_tip_diameter: scaled_coord(0.4),
        }
    }
}

impl TreeSupportMeshGroupSettings {
    // TreeSupportCommon.hpp:30-90
    // explicit TreeSupportMeshGroupSettings(const PrintObject &print_object)
    pub fn from_print_object(print_object: &PrintObject) -> Self {
        use std::f64::consts::PI as M_PI;

        // TreeSupportCommon.hpp:32  const PrintConfig &print_config = print_object.print()->config();
        let print_config = print_object.print().config();
        // TreeSupportCommon.hpp:33  const PrintObjectConfig &config = print_object.config();
        let config = print_object.config();
        // TreeSupportCommon.hpp:34  const SlicingParameters &slicing_params = print_object.slicing_parameters();
        let slicing_params = print_object.slicing_parameters();

        // TreeSupportCommon.hpp:42-46
        // Calculate maximum external perimeter width over all printing regions,
        // taking into account the default layer height.
        let mut external_perimeter_width: f64 = 0.;
        for region_id in 0..print_object.num_printing_regions() {
            if let Some(region) = print_object.printing_region(region_id) {
                // TreeSupportCommon.hpp:45  region.flow(print_object, frExternalPerimeter, config.layer_height).width()
                if let Ok(flow) =
                    region.flow(print_object, FlowRole::ExternalPerimeter, config.layer_height, false)
                {
                    external_perimeter_width = external_perimeter_width.max(flow.width());
                }
            }
        }

        // TreeSupportCommon.hpp:48  this->layer_height = scaled<coord_t>(config.layer_height.value);
        let layer_height = scaled_coord(config.layer_height);
        // TreeSupportCommon.hpp:49  this->resolution = scaled<coord_t>(print_config.resolution.value);
        let resolution = scaled_coord(print_config.resolution);
        // TreeSupportCommon.hpp:51  this->min_feature_size = scaled<coord_t>(config.min_feature_size.value);
        // Rust: config.arachne_min_feature_size == C++ config.min_feature_size
        let min_feature_size = scaled_coord(config.arachne_min_feature_size);
        // TreeSupportCommon.hpp:53  this->support_angle = 0.5 * M_PI - std::clamp<double>((config.support_threshold_angle + 1) * M_PI / 180., 0., 0.5 * M_PI);
        let support_angle = 0.5 * M_PI
            - ((config.support_threshold_angle + 1.0) * M_PI / 180.)
                .clamp(0., 0.5 * M_PI);
        // TreeSupportCommon.hpp:54  this->support_line_width = support_material_flow(&print_object, config.layer_height).scaled_width();
        let support_line_width = support_material_flow(
            config.support_line_width,
            config.line_width,
            print_config.nozzle_diameter,
            config.layer_height,
            config.layer_height,
        )
        .map(|f| f.scaled_width())
        .unwrap_or_else(|_| scaled_coord(0.4));
        // TreeSupportCommon.hpp:55  this->support_roof_line_width = support_material_interface_flow(&print_object, config.layer_height).scaled_width();
        let support_roof_line_width = support_material_interface_flow(
            config.support_line_width,
            config.line_width,
            print_config.nozzle_diameter,
            config.layer_height,
            config.layer_height,
        )
        .map(|f| f.scaled_width())
        .unwrap_or_else(|_| scaled_coord(0.4));
        // TreeSupportCommon.hpp:57  this->support_bottom_enable = config.support_interface_top_layers.value > 0 && config.support_interface_bottom_layers.value != 0;
        let support_bottom_enable =
            config.support_interface_top_layers > 0 && config.support_interface_bottom_layers != 0;
        // TreeSupportCommon.hpp:58-62  this->support_bottom_height = ...
        let support_bottom_height: Coord = if support_bottom_enable {
            // TreeSupportCommon.hpp:59  (config.support_interface_bottom_layers.value > 0 ?
            //                             config.support_interface_bottom_layers.value :
            //                             config.support_interface_top_layers.value) * this->layer_height
            let bottom_layers = if config.support_interface_bottom_layers > 0 {
                config.support_interface_bottom_layers as Coord
            } else {
                config.support_interface_top_layers as Coord
            };
            bottom_layers * layer_height
        } else {
            0
        };
        // TreeSupportCommon.hpp:63  this->support_material_buildplate_only = config.support_on_build_plate_only;
        let support_material_buildplate_only = config.support_on_build_plate_only;
        // TreeSupportCommon.hpp:64  this->support_top_distance = scaled<coord_t>(slicing_params.gap_support_object);
        let support_top_distance = scaled_coord(slicing_params.gap_support_object);
        // TreeSupportCommon.hpp:65  this->support_bottom_distance = scaled<coord_t>(slicing_params.gap_object_support);
        let support_bottom_distance = scaled_coord(slicing_params.gap_object_support);
        // TreeSupportCommon.hpp:66  this->support_xy_distance = scaled<coord_t>(std::max(0.01, config.support_object_xy_distance.value));
        let mut support_xy_distance =
            scaled_coord(config.support_object_xy_distance.max(0.01));
        // TreeSupportCommon.hpp:67-68  if (print_config.top_z_overrides_xy_distance) ...
        // Rust: top_z_overrides_xy_distance is on PrintObjectConfig (config), not PrintConfig.
        if config.top_z_overrides_xy_distance {
            // TreeSupportCommon.hpp:68  this->support_xy_distance = std::min(this->support_xy_distance, std::max(this->support_top_distance, coord_t(scale_(0.2))));
            let floor = support_top_distance.max(scaled_coord(0.2));
            support_xy_distance = support_xy_distance.min(floor);
        }
        // TreeSupportCommon.hpp:69  this->support_xy_distance_1st_layer = scaled<coord_t>(config.support_object_first_layer_gap.value);
        let support_xy_distance_1st_layer =
            scaled_coord(config.support_object_first_layer_gap);
        // TreeSupportCommon.hpp:71  this->support_xy_distance_overhang = std::min(this->support_xy_distance, scaled<coord_t>(0.5 * external_perimeter_width));
        let support_xy_distance_overhang =
            support_xy_distance.min(scaled_coord(0.5 * external_perimeter_width));
        // TreeSupportCommon.hpp:72  this->support_roof_enable = config.support_interface_top_layers.value > 0;
        let support_roof_enable = config.support_interface_top_layers > 0;
        // TreeSupportCommon.hpp:73  this->support_roof_layers = config.support_interface_top_layers.value;
        let support_roof_layers = config.support_interface_top_layers as Coord;
        // TreeSupportCommon.hpp:74  this->support_floor_enable = config.support_interface_bottom_layers.value > 0;
        let support_floor_enable = config.support_interface_bottom_layers > 0;
        // TreeSupportCommon.hpp:75  this->support_floor_layers = config.support_interface_bottom_layers.value;
        let support_floor_layers = config.support_interface_bottom_layers as Coord;
        // TreeSupportCommon.hpp:76  this->support_roof_pattern = config.support_interface_pattern;
        // Map Rust SupportInterfacePattern -> SupportMaterialInterfacePattern
        let support_roof_pattern = match config.support_interface_pattern {
            SupportInterfacePattern::Rectilinear => {
                SupportMaterialInterfacePattern::smipRectilinear
            }
            SupportInterfacePattern::Concentric => {
                SupportMaterialInterfacePattern::smipConcentric
            }
            SupportInterfacePattern::Grid => SupportMaterialInterfacePattern::smipGrid,
        };
        // TreeSupportCommon.hpp:77  this->support_line_spacing = scaled<coord_t>(config.support_base_pattern_spacing.value);
        let support_line_spacing = scaled_coord(config.support_base_pattern_spacing);
        // TreeSupportCommon.hpp:78  this->support_wall_count = std::max(1, config.tree_support_wall_count.value);
        let support_wall_count = (config.tree_support_wall_count as i32).max(1);
        // TreeSupportCommon.hpp:79  this->support_roof_line_distance = scaled<coord_t>(config.support_interface_spacing.value) + this->support_roof_line_width;
        let support_roof_line_distance =
            scaled_coord(config.support_interface_spacing) + support_roof_line_width;
        // TreeSupportCommon.hpp:80  double support_tree_angle_slow = 25; // TODO add a setting?
        let support_tree_angle_slow_deg: f64 = 25.;
        // TreeSupportCommon.hpp:81  double tree_support_tip_diameter = 0.8;
        let tree_support_tip_diameter: f64 = 0.8;
        // TreeSupportCommon.hpp:82  this->support_tree_branch_distance = scaled<coord_t>(config.tree_support_branch_distance.value);
        let support_tree_branch_distance = scaled_coord(config.tree_support_branch_distance);
        // TreeSupportCommon.hpp:83  this->support_tree_angle = std::clamp<double>(config.tree_support_branch_angle * M_PI / 180., 0., 0.5 * M_PI - EPSILON);
        let support_tree_angle =
            (config.tree_support_branch_angle * M_PI / 180.).clamp(0., 0.5 * M_PI - EPSILON);
        // TreeSupportCommon.hpp:84  this->support_tree_angle_slow = std::clamp<double>(support_tree_angle_slow * M_PI / 180., 0., this->support_tree_angle - EPSILON);
        let support_tree_angle_slow =
            (support_tree_angle_slow_deg * M_PI / 180.).clamp(0., support_tree_angle - EPSILON);
        // TreeSupportCommon.hpp:85  this->support_tree_branch_diameter = scaled<coord_t>(config.tree_support_branch_diameter.value);
        let support_tree_branch_diameter = scaled_coord(config.tree_support_branch_diameter);
        // TreeSupportCommon.hpp:86  this->support_tree_branch_diameter_angle = std::clamp<double>(config.tree_support_branch_diameter_angle * M_PI / 180., 0., 0.5 * M_PI - EPSILON);
        let support_tree_branch_diameter_angle =
            (config.tree_support_branch_diameter_angle * M_PI / 180.)
                .clamp(0., 0.5 * M_PI - EPSILON);
        // TreeSupportCommon.hpp:87  this->support_tree_top_rate = 30; // percent
        let support_tree_top_rate: f64 = 30.;
        // TreeSupportCommon.hpp:89  this->support_tree_tip_diameter = std::clamp(scaled<coord_t>(tree_support_tip_diameter), 0, this->support_tree_branch_diameter);
        let support_tree_tip_diameter =
            scaled_coord(tree_support_tip_diameter).clamp(0, support_tree_branch_diameter);

        Self {
            layer_height,
            resolution,
            min_feature_size,
            support_angle,
            support_line_width,
            support_roof_line_width,
            support_bottom_enable,
            support_bottom_height,
            support_material_buildplate_only,
            support_xy_distance,
            support_xy_distance_1st_layer,
            support_xy_distance_overhang,
            support_top_distance,
            support_bottom_distance,
            // TreeSupportCommon.hpp:143 default; not set in constructor
            support_interface_skip_height: scaled_coord(0.3),
            support_roof_enable,
            support_roof_layers,
            support_floor_enable,
            support_floor_layers,
            // TreeSupportCommon.hpp:160 default; not set in constructor
            minimum_roof_area: scaled_f64(scaled_f64(1.)),
            // TreeSupportCommon.hpp:165 default; not set in constructor
            support_roof_angles: Vec::new(),
            support_roof_pattern,
            support_line_spacing,
            // TreeSupportCommon.hpp:174 default; not set in constructor
            support_bottom_offset: scaled_coord(0.),
            support_wall_count,
            support_roof_line_distance,
            // TreeSupportCommon.hpp:185 default; not set in constructor
            minimum_support_area: scaled_coord(0.),
            // TreeSupportCommon.hpp:188 default; not set in constructor
            minimum_bottom_area: scaled_coord(1.0),
            // TreeSupportCommon.hpp:191 default; not set in constructor
            support_offset: scaled_coord(0.),
            support_tree_angle,
            support_tree_branch_diameter_angle,
            support_tree_branch_distance,
            support_tree_branch_diameter,
            support_tree_angle_slow,
            // TreeSupportCommon.hpp:227 default; not set in constructor
            support_tree_max_diameter_increase_by_merges_when_support_to_model: scaled_coord(1.0),
            // TreeSupportCommon.hpp:231 default; not set in constructor
            support_tree_min_height_to_model: scaled_coord(1.0),
            // TreeSupportCommon.hpp:235 default; not set in constructor
            support_tree_bp_diameter: scaled_coord(7.5),
            support_tree_top_rate,
            support_tree_tip_diameter,
        }
    }
}

// TreeSupportCommon.hpp:251-254
/// This struct contains settings used in the tree support. Thanks to this most
/// functions do not need to know of meshes etc. Also makes the code shorter.
//
// TreeSupportCommon.hpp:254  struct TreeSupportSettings
#[derive(Debug, Clone)]
pub struct TreeSupportSettings {
    // TreeSupportCommon.hpp:345-347  (private members)
    angle: f64,
    angle_slow: f64,
    known_z: Vec<Coord>,

    // TreeSupportCommon.hpp:352  inline static bool soluble = false;
    // Modeled as an instance field initialized to the static default; the C++
    // `soluble` is a process-wide static, but it is only ever read inside the
    // constructor and toggled externally before construction.
    // TreeSupportCommon.hpp:356  coord_t support_line_width;
    pub support_line_width: Coord,
    // TreeSupportCommon.hpp:360  coord_t layer_height;
    pub layer_height: Coord,
    // TreeSupportCommon.hpp:364  coord_t branch_radius;
    pub branch_radius: Coord,
    // TreeSupportCommon.hpp:368  coord_t min_radius;
    pub min_radius: Coord,
    // TreeSupportCommon.hpp:372  coord_t maximum_move_distance;
    pub maximum_move_distance: Coord,
    // TreeSupportCommon.hpp:376  coord_t maximum_move_distance_slow;
    pub maximum_move_distance_slow: Coord,
    // TreeSupportCommon.hpp:380  size_t support_bottom_layers;
    pub support_bottom_layers: usize,
    // TreeSupportCommon.hpp:384  size_t tip_layers;
    pub tip_layers: usize,
    // TreeSupportCommon.hpp:388  double branch_radius_increase_per_layer;
    pub branch_radius_increase_per_layer: f64,
    // TreeSupportCommon.hpp:392  coord_t max_to_model_radius_increase;
    pub max_to_model_radius_increase: Coord,
    // TreeSupportCommon.hpp:396  size_t min_dtt_to_model;
    pub min_dtt_to_model: usize,
    // TreeSupportCommon.hpp:400  coord_t increase_radius_until_radius;
    pub increase_radius_until_radius: Coord,
    // TreeSupportCommon.hpp:404  size_t increase_radius_until_layer;
    pub increase_radius_until_layer: usize,
    // TreeSupportCommon.hpp:408  bool support_rests_on_model;
    pub support_rests_on_model: bool,
    // TreeSupportCommon.hpp:412  coord_t xy_distance;
    pub xy_distance: Coord,
    // TreeSupportCommon.hpp:416  coord_t bp_radius;
    pub bp_radius: Coord,
    // TreeSupportCommon.hpp:420  LayerIndex layer_start_bp_radius;
    pub layer_start_bp_radius: LayerIndex,
    // TreeSupportCommon.hpp:424  double bp_radius_increase_per_layer;
    pub bp_radius_increase_per_layer: f64,
    // TreeSupportCommon.hpp:428  coord_t xy_min_distance;
    pub xy_min_distance: Coord,
    // TreeSupportCommon.hpp:432  size_t z_distance_top_layers;
    pub z_distance_top_layers: usize,
    // TreeSupportCommon.hpp:436  size_t z_distance_bottom_layers;
    pub z_distance_bottom_layers: usize,
    // TreeSupportCommon.hpp:440  size_t performance_interface_skip_layers;
    pub performance_interface_skip_layers: usize,
    // TreeSupportCommon.hpp:448  std::vector<double> support_roof_angles;
    pub support_roof_angles: Vec<f64>,
    // TreeSupportCommon.hpp:452  SupportMaterialInterfacePattern roof_pattern;
    pub roof_pattern: SupportMaterialInterfacePattern,
    // TreeSupportCommon.hpp:456  SupportMaterialPattern support_pattern;
    pub support_pattern: i32,
    // TreeSupportCommon.hpp:460  coord_t support_roof_line_width;
    pub support_roof_line_width: Coord,
    // TreeSupportCommon.hpp:464  coord_t support_line_spacing;
    pub support_line_spacing: Coord,
    // TreeSupportCommon.hpp:468  coord_t support_bottom_offset;
    pub support_bottom_offset: Coord,
    // TreeSupportCommon.hpp:472  int support_wall_count;
    pub support_wall_count: i32,
    // TreeSupportCommon.hpp:476  coord_t resolution;
    pub resolution: Coord,
    // TreeSupportCommon.hpp:480  coord_t support_roof_line_distance;
    pub support_roof_line_distance: Coord,
    // TreeSupportCommon.hpp:484  InterfacePreference interface_preference;
    pub interface_preference: InterfacePreference,
    // TreeSupportCommon.hpp:489  TreeSupportMeshGroupSettings settings;
    pub settings: TreeSupportMeshGroupSettings,
    // TreeSupportCommon.hpp:494  coord_t min_feature_size;
    pub min_feature_size: Coord,
    // TreeSupportCommon.hpp:497  std::vector<coordf_t> raft_layers;
    pub raft_layers: Vec<CoordF>,
}

// TreeSupportCommon.hpp:352  inline static bool soluble = false;
pub static SOLUBLE: bool = false;

impl Default for TreeSupportSettings {
    // TreeSupportCommon.hpp:256  TreeSupportSettings() = default;
    fn default() -> Self {
        Self {
            angle: 0.,
            angle_slow: 0.,
            known_z: Vec::new(),
            support_line_width: 0,
            layer_height: 0,
            branch_radius: 0,
            min_radius: 0,
            maximum_move_distance: 0,
            maximum_move_distance_slow: 0,
            support_bottom_layers: 0,
            tip_layers: 0,
            branch_radius_increase_per_layer: 0.,
            max_to_model_radius_increase: 0,
            min_dtt_to_model: 0,
            increase_radius_until_radius: 0,
            increase_radius_until_layer: 0,
            support_rests_on_model: false,
            xy_distance: 0,
            bp_radius: 0,
            layer_start_bp_radius: 0,
            bp_radius_increase_per_layer: 0.,
            xy_min_distance: 0,
            z_distance_top_layers: 0,
            z_distance_bottom_layers: 0,
            performance_interface_skip_layers: 0,
            support_roof_angles: Vec::new(),
            roof_pattern: SupportMaterialInterfacePattern::smipAuto,
            support_pattern: 0,
            support_roof_line_width: 0,
            support_line_spacing: 0,
            support_bottom_offset: 0,
            support_wall_count: 0,
            resolution: 0,
            support_roof_line_distance: 0,
            interface_preference: InterfacePreference::default(),
            settings: TreeSupportMeshGroupSettings::default(),
            min_feature_size: 0,
            raft_layers: Vec::new(),
        }
    }
}

impl TreeSupportSettings {
    // TreeSupportCommon.hpp:258-342
    // explicit TreeSupportSettings(const TreeSupportMeshGroupSettings& mesh_group_settings,
    //                              const SlicingParameters &slicing_params)
    pub fn new(
        mesh_group_settings: &TreeSupportMeshGroupSettings,
        slicing_params: &SlicingParams,
    ) -> Self {
        use std::f64::consts::PI as M_PI;

        // TreeSupportCommon.hpp:259  angle(mesh_group_settings.support_tree_angle),
        let angle = mesh_group_settings.support_tree_angle;
        // TreeSupportCommon.hpp:260  angle_slow(mesh_group_settings.support_tree_angle_slow),
        let angle_slow = mesh_group_settings.support_tree_angle_slow;
        // TreeSupportCommon.hpp:261  support_line_width(mesh_group_settings.support_line_width),
        let support_line_width = mesh_group_settings.support_line_width;
        // TreeSupportCommon.hpp:262  layer_height(mesh_group_settings.layer_height),
        let layer_height = mesh_group_settings.layer_height;
        // TreeSupportCommon.hpp:263  branch_radius(mesh_group_settings.support_tree_branch_diameter / 2),
        let branch_radius = mesh_group_settings.support_tree_branch_diameter / 2;
        // TreeSupportCommon.hpp:264  min_radius(mesh_group_settings.support_tree_tip_diameter / 2),
        let min_radius = mesh_group_settings.support_tree_tip_diameter / 2;
        // TreeSupportCommon.hpp:265  maximum_move_distance((angle < M_PI / 2.) ? (coord_t)(tan(angle) * layer_height) : std::numeric_limits<coord_t>::max()),
        let maximum_move_distance = if angle < M_PI / 2. {
            (angle.tan() * layer_height as f64) as Coord
        } else {
            Coord::MAX
        };
        // TreeSupportCommon.hpp:266  maximum_move_distance_slow((angle_slow < M_PI / 2.) ? (coord_t)(tan(angle_slow) * layer_height) : std::numeric_limits<coord_t>::max()),
        let maximum_move_distance_slow = if angle_slow < M_PI / 2. {
            (angle_slow.tan() * layer_height as f64) as Coord
        } else {
            Coord::MAX
        };
        // TreeSupportCommon.hpp:267  support_bottom_layers(mesh_group_settings.support_bottom_enable ? (mesh_group_settings.support_bottom_height + layer_height / 2) / layer_height : 0),
        let support_bottom_layers: usize = if mesh_group_settings.support_bottom_enable {
            ((mesh_group_settings.support_bottom_height + layer_height / 2) / layer_height) as usize
        } else {
            0
        };
        // TreeSupportCommon.hpp:268  tip_layers(std::max((branch_radius - min_radius) / (support_line_width / 3), branch_radius / layer_height)),
        let tip_layers: usize = std::cmp::max(
            (branch_radius - min_radius) / (support_line_width / 3),
            branch_radius / layer_height,
        ) as usize;
        // TreeSupportCommon.hpp:269  branch_radius_increase_per_layer(tan(mesh_group_settings.support_tree_branch_diameter_angle) * layer_height),
        let branch_radius_increase_per_layer =
            mesh_group_settings.support_tree_branch_diameter_angle.tan() * layer_height as f64;
        // TreeSupportCommon.hpp:270  max_to_model_radius_increase(mesh_group_settings.support_tree_max_diameter_increase_by_merges_when_support_to_model / 2),
        let max_to_model_radius_increase = mesh_group_settings
            .support_tree_max_diameter_increase_by_merges_when_support_to_model
            / 2;
        // TreeSupportCommon.hpp:271  min_dtt_to_model(round_up_divide(mesh_group_settings.support_tree_min_height_to_model, layer_height)),
        let min_dtt_to_model = crate::utils::round_up_divide(
            mesh_group_settings.support_tree_min_height_to_model,
            layer_height,
        ) as usize;
        // TreeSupportCommon.hpp:272  increase_radius_until_radius(mesh_group_settings.support_tree_branch_diameter / 2),
        let increase_radius_until_radius = mesh_group_settings.support_tree_branch_diameter / 2;
        // TreeSupportCommon.hpp:273  increase_radius_until_layer(increase_radius_until_radius <= branch_radius ? tip_layers * (increase_radius_until_radius / branch_radius) : (increase_radius_until_radius - branch_radius) / branch_radius_increase_per_layer),
        let increase_radius_until_layer: usize = if increase_radius_until_radius <= branch_radius {
            tip_layers * (increase_radius_until_radius / branch_radius) as usize
        } else {
            ((increase_radius_until_radius - branch_radius) as f64 / branch_radius_increase_per_layer)
                as usize
        };
        // TreeSupportCommon.hpp:274  support_rests_on_model(! mesh_group_settings.support_material_buildplate_only),
        let support_rests_on_model = !mesh_group_settings.support_material_buildplate_only;
        // TreeSupportCommon.hpp:275  xy_distance(mesh_group_settings.support_xy_distance),
        let mut xy_distance = mesh_group_settings.support_xy_distance;
        // TreeSupportCommon.hpp:276  xy_min_distance(std::min(mesh_group_settings.support_xy_distance, mesh_group_settings.support_xy_distance_overhang)),
        let mut xy_min_distance = std::cmp::min(
            mesh_group_settings.support_xy_distance,
            mesh_group_settings.support_xy_distance_overhang,
        );
        // TreeSupportCommon.hpp:277  bp_radius(mesh_group_settings.support_tree_bp_diameter / 2),
        let bp_radius = mesh_group_settings.support_tree_bp_diameter / 2;
        // TreeSupportCommon.hpp:278  bp_radius_increase_per_layer(std::min(tan(0.7) * layer_height, 0.5 * support_line_width)),
        let bp_radius_increase_per_layer = f64::min(
            (0.7_f64).tan() * layer_height as f64,
            0.5 * support_line_width as f64,
        );
        // TreeSupportCommon.hpp:279  z_distance_bottom_layers(size_t(round(double(mesh_group_settings.support_bottom_distance) / double(layer_height)))),
        let z_distance_bottom_layers =
            (mesh_group_settings.support_bottom_distance as f64 / layer_height as f64).round()
                as usize;
        // TreeSupportCommon.hpp:280  z_distance_top_layers(size_t(round(double(mesh_group_settings.support_top_distance) / double(layer_height)))),
        let z_distance_top_layers =
            (mesh_group_settings.support_top_distance as f64 / layer_height as f64).round() as usize;
        // TreeSupportCommon.hpp:282  support_roof_angles(mesh_group_settings.support_roof_angles),
        let support_roof_angles = mesh_group_settings.support_roof_angles.clone();
        // TreeSupportCommon.hpp:283  roof_pattern(mesh_group_settings.support_roof_pattern),
        let roof_pattern = mesh_group_settings.support_roof_pattern;
        // TreeSupportCommon.hpp:284  support_roof_line_width(mesh_group_settings.support_roof_line_width),
        let support_roof_line_width = mesh_group_settings.support_roof_line_width;
        // TreeSupportCommon.hpp:285  support_line_spacing(mesh_group_settings.support_line_spacing),
        let support_line_spacing = mesh_group_settings.support_line_spacing;
        // TreeSupportCommon.hpp:286  support_bottom_offset(mesh_group_settings.support_bottom_offset),
        let support_bottom_offset = mesh_group_settings.support_bottom_offset;
        // TreeSupportCommon.hpp:287  support_wall_count(mesh_group_settings.support_wall_count),
        let support_wall_count = mesh_group_settings.support_wall_count;
        // TreeSupportCommon.hpp:288  resolution(mesh_group_settings.resolution),
        let resolution = mesh_group_settings.resolution;
        // TreeSupportCommon.hpp:289  support_roof_line_distance(mesh_group_settings.support_roof_line_distance),
        let support_roof_line_distance = mesh_group_settings.support_roof_line_distance;
        // TreeSupportCommon.hpp:290  settings(mesh_group_settings),
        let settings = mesh_group_settings.clone();
        // TreeSupportCommon.hpp:291  min_feature_size(mesh_group_settings.min_feature_size)
        let min_feature_size = mesh_group_settings.min_feature_size;

        let mut this = TreeSupportSettings {
            angle,
            angle_slow,
            known_z: Vec::new(),
            support_line_width,
            layer_height,
            branch_radius,
            min_radius,
            maximum_move_distance,
            maximum_move_distance_slow,
            support_bottom_layers,
            tip_layers,
            branch_radius_increase_per_layer,
            max_to_model_radius_increase,
            min_dtt_to_model,
            increase_radius_until_radius,
            increase_radius_until_layer,
            support_rests_on_model,
            xy_distance,
            bp_radius,
            // TreeSupportCommon.hpp:420 default-constructed; set below.
            layer_start_bp_radius: 0,
            bp_radius_increase_per_layer,
            xy_min_distance,
            z_distance_top_layers,
            z_distance_bottom_layers,
            // TreeSupportCommon.hpp:440 performance_interface_skip_layers not set in ctor.
            performance_interface_skip_layers: 0,
            support_roof_angles,
            roof_pattern,
            // TreeSupportCommon.hpp:456 support_pattern not set in ctor.
            support_pattern: 0,
            support_roof_line_width,
            support_line_spacing,
            support_bottom_offset,
            support_wall_count,
            resolution,
            support_roof_line_distance,
            // TreeSupportCommon.hpp:484 interface_preference set in ctor body below.
            interface_preference: InterfacePreference::default(),
            settings,
            min_feature_size,
            raft_layers: Vec::new(),
        };

        // TreeSupportCommon.hpp:293-294
        // At least one tip layer must be defined.
        debug_assert!(this.tip_layers > 0);
        // TreeSupportCommon.hpp:295  layer_start_bp_radius = (bp_radius - branch_radius) / bp_radius_increase_per_layer;
        this.layer_start_bp_radius =
            ((this.bp_radius - this.branch_radius) as f64 / this.bp_radius_increase_per_layer)
                as LayerIndex;

        // TreeSupportCommon.hpp:297  if (TreeSupportSettings::soluble) {
        if SOLUBLE {
            // TreeSupportCommon.hpp:298-300 (comments)
            // safeOffsetInc can only work in steps of the size xy_min_distance in the worst case => xy_min_distance has to be a bit larger than 0 in this worst case and should be large enough for performance to not suffer extremely
            // When for all meshes the z bottom and top distance is more than one layer though the worst case is xy_min_distance + min_feature_size
            // This is not the best solution, but the only one to ensure areas can not lag though walls at high maximum_move_distance.
            // TreeSupportCommon.hpp:301  xy_min_distance = std::max(xy_min_distance, scaled<coord_t>(0.1));
            xy_min_distance = std::cmp::max(xy_min_distance, scaled_coord(0.1));
            // TreeSupportCommon.hpp:302  xy_distance = std::max(xy_distance, xy_min_distance);
            xy_distance = std::cmp::max(xy_distance, xy_min_distance);
            this.xy_min_distance = xy_min_distance;
            this.xy_distance = xy_distance;
        }

        // TreeSupportCommon.hpp:310  interface_preference = InterfacePreference::InterfaceAreaOverwritesSupport;
        this.interface_preference = InterfacePreference::InterfaceAreaOverwritesSupport;

        // TreeSupportCommon.hpp:312  if (slicing_params.raft_layers() > 0) {
        if slicing_params.raft_layers() > 0 {
            // TreeSupportCommon.hpp:313-315
            // Fill in raft_layers with the heights of the layers below the first object layer.
            // First layer
            let mut z = slicing_params.first_print_layer_height;
            // TreeSupportCommon.hpp:316  this->raft_layers.emplace_back(z);
            this.raft_layers.push(z);
            // TreeSupportCommon.hpp:317-318  Raft base layers
            for _i in 1..slicing_params.base_raft_layers {
                // TreeSupportCommon.hpp:319  z += slicing_params.base_raft_layer_height;
                z += slicing_params.base_raft_layer_height;
                // TreeSupportCommon.hpp:320  this->raft_layers.emplace_back(z);
                this.raft_layers.push(z);
            }
            // TreeSupportCommon.hpp:322-323  Raft interface layers
            // for (size_t i = 0; i + 1 < slicing_params.interface_raft_layers; ++ i)
            {
                let mut i: usize = 0;
                while i + 1 < slicing_params.interface_raft_layers {
                    // TreeSupportCommon.hpp:324  z += slicing_params.interface_raft_layer_height;
                    z += slicing_params.interface_raft_layer_height;
                    // TreeSupportCommon.hpp:325  this->raft_layers.emplace_back(z);
                    this.raft_layers.push(z);
                    i += 1;
                }
            }
            // TreeSupportCommon.hpp:327-328  Raft contact layer
            if slicing_params.raft_layers() > 1 {
                // TreeSupportCommon.hpp:329  z = slicing_params.raft_contact_top_z;
                z = slicing_params.raft_contact_top_z;
                // TreeSupportCommon.hpp:330  this->raft_layers.emplace_back(z);
                this.raft_layers.push(z);
            }
            // TreeSupportCommon.hpp:332  if (double dist_to_go = slicing_params.object_print_z_min - z; dist_to_go > EPSILON) {
            let dist_to_go = slicing_params.object_print_z_min - z;
            if dist_to_go > EPSILON {
                // TreeSupportCommon.hpp:333-334  Layers between the raft contacts and bottom of the object.
                // auto nsteps = int(ceil(dist_to_go / slicing_params.max_suport_layer_height));
                let nsteps = (dist_to_go / slicing_params.max_suport_layer_height).ceil() as i32;
                // TreeSupportCommon.hpp:335  double step = dist_to_go / nsteps;
                let step = dist_to_go / nsteps as f64;
                // TreeSupportCommon.hpp:336  for (int i = 0; i < nsteps; ++ i) {
                for _i in 0..nsteps {
                    // TreeSupportCommon.hpp:337  z += step;
                    z += step;
                    // TreeSupportCommon.hpp:338  this->raft_layers.emplace_back(z);
                    this.raft_layers.push(z);
                }
            }
        }

        this
    }

    // TreeSupportCommon.hpp:541-548
    // [[nodiscard]] inline coord_t getRadius(size_t distance_to_top, const double elephant_foot_increases = 0) const
    #[must_use]
    pub fn get_radius(&self, distance_to_top: usize, elephant_foot_increases: f64) -> Coord {
        // TreeSupportCommon.hpp:543-547
        let base = if distance_to_top <= self.tip_layers {
            // tip
            self.min_radius
                + (self.branch_radius - self.min_radius) * distance_to_top as Coord
                    / self.tip_layers as Coord
        } else {
            // base
            self.branch_radius
                // gradual increase
                + ((distance_to_top - self.tip_layers) as f64 * self.branch_radius_increase_per_layer)
                    as Coord
        };
        base + (elephant_foot_increases
            * f64::max(
                self.bp_radius_increase_per_layer - self.branch_radius_increase_per_layer,
                0.0,
            )) as Coord
    }

    // TreeSupportCommon.hpp:555-559
    // [[nodiscard]] inline coord_t recommendedMinRadius(LayerIndex layer_idx) const
    #[must_use]
    pub fn recommended_min_radius(&self, layer_idx: LayerIndex) -> Coord {
        // TreeSupportCommon.hpp:557  double num_layers_widened = layer_start_bp_radius - layer_idx;
        let num_layers_widened = (self.layer_start_bp_radius - layer_idx) as f64;
        // TreeSupportCommon.hpp:558  return num_layers_widened > 0 ? branch_radius + num_layers_widened * bp_radius_increase_per_layer : 0;
        if num_layers_widened > 0. {
            self.branch_radius + (num_layers_widened * self.bp_radius_increase_per_layer) as Coord
        } else {
            0
        }
    }

    // TreeSupportCommon.hpp:566-569
    // [[nodiscard]] inline coord_t getActualZ(LayerIndex layer_idx)
    #[must_use]
    pub fn get_actual_z(&self, layer_idx: LayerIndex) -> Coord {
        // TreeSupportCommon.hpp:568
        // return layer_idx < coord_t(known_z.size()) ? known_z[layer_idx] :
        //        (layer_idx - known_z.size()) * layer_height + known_z.size() ? known_z.back() : 0;
        //
        // NOTE: The C++ expression after the first `:` is faithfully reproduced
        // including its (almost-certainly buggy) operator precedence. `?:` binds
        // looser than `+`, so the condition of the inner ternary is the whole
        // arithmetic expression `(layer_idx - known_z.size()) * layer_height + known_z.size()`,
        // which is then tested for non-zero; if non-zero it yields `known_z.back()`,
        // else `0`. This bug-for-bug port preserves byte-exact behavior.
        if (layer_idx as Coord) < self.known_z.len() as Coord {
            self.known_z[layer_idx as usize]
        } else {
            let cond = (layer_idx as Coord - self.known_z.len() as Coord) * self.layer_height
                + self.known_z.len() as Coord;
            if cond != 0 {
                *self.known_z.last().unwrap()
            } else {
                0
            }
        }
    }

    // TreeSupportCommon.hpp:576-579
    // void setActualZ(std::vector<coord_t>& z)
    pub fn set_actual_z(&mut self, z: Vec<Coord>) {
        // TreeSupportCommon.hpp:578  known_z = z;
        self.known_z = z;
    }
}

impl PartialEq for TreeSupportSettings {
    // TreeSupportCommon.hpp:500-533  bool operator==(const TreeSupportSettings& other) const
    fn eq(&self, other: &Self) -> bool {
        // TreeSupportCommon.hpp:502-512
        self.branch_radius == other.branch_radius
            && self.tip_layers == other.tip_layers
            && self.branch_radius_increase_per_layer == other.branch_radius_increase_per_layer
            && self.layer_start_bp_radius == other.layer_start_bp_radius
            && self.bp_radius == other.bp_radius
            && self.bp_radius_increase_per_layer == other.bp_radius_increase_per_layer
            && self.min_radius == other.min_radius
            && self.xy_min_distance == other.xy_min_distance
            // TreeSupportCommon.hpp:504  if the delta of xy_min_distance and xy_distance is different the collision areas have to be recalculated.
            && self.xy_distance - self.xy_min_distance == other.xy_distance - other.xy_min_distance
            && self.support_rests_on_model == other.support_rests_on_model
            && self.increase_radius_until_layer == other.increase_radius_until_layer
            && self.min_dtt_to_model == other.min_dtt_to_model
            && self.max_to_model_radius_increase == other.max_to_model_radius_increase
            && self.maximum_move_distance == other.maximum_move_distance
            && self.maximum_move_distance_slow == other.maximum_move_distance_slow
            && self.z_distance_bottom_layers == other.z_distance_bottom_layers
            && self.support_line_width == other.support_line_width
            && self.support_line_spacing == other.support_line_spacing
            && self.support_roof_line_width == other.support_roof_line_width
            && self.support_bottom_offset == other.support_bottom_offset
            && self.support_wall_count == other.support_wall_count
            && self.support_pattern == other.support_pattern
            && self.roof_pattern == other.roof_pattern
            && self.support_roof_angles == other.support_roof_angles
            // support_infill_angles == other.support_infill_angles (commented out upstream)
            && self.increase_radius_until_radius == other.increase_radius_until_radius
            && self.support_bottom_layers == other.support_bottom_layers
            && self.layer_height == other.layer_height
            && self.z_distance_top_layers == other.z_distance_top_layers
            && self.resolution == other.resolution
            && self.support_roof_line_distance == other.support_roof_line_distance
            && self.interface_preference == other.interface_preference
            && self.min_feature_size == other.min_feature_size
            // TreeSupportCommon.hpp:514-530 `#if 0` block omitted (disabled upstream).
            && self.raft_layers == other.raft_layers
    }
}

// TreeSupportCommon.hpp:582-595
// inline void tree_supports_show_error(std::string_view message, bool critical)
pub fn tree_supports_show_error(_message: &str, _critical: bool) {
    // todo Remove!  ONLY FOR PUBLIC BETA!!
    // printf("Error: %s, critical: %d\n", message.data(), int(critical));
    // The TREE_SUPPORT_SHOW_ERRORS_WIN32 block (TreeSupportCommon.hpp:585-594)
    // is a Win32 MessageBoxA path and is intentionally not ported (not wasm-safe,
    // and `#ifdef`-guarded off in the reference build).
}

// TreeSupportCommon.hpp:597-602
// inline double layer_z(const SlicingParameters &slicing_params, const TreeSupportSettings &config, const size_t layer_idx)
pub fn layer_z(slicing_params: &SlicingParams, config: &TreeSupportSettings, layer_idx: usize) -> f64 {
    // TreeSupportCommon.hpp:599-601
    if layer_idx >= config.raft_layers.len() {
        slicing_params.object_print_z_min
            + slicing_params.first_object_layer_height
            + (layer_idx - config.raft_layers.len()) as f64 * slicing_params.layer_height
    } else {
        config.raft_layers[layer_idx]
    }
}

// TreeSupportCommon.hpp:603-609
// Lowest collision layer
// inline LayerIndex layer_idx_ceil(const SlicingParameters &slicing_params, const TreeSupportSettings &config, const double z)
pub fn layer_idx_ceil(
    slicing_params: &SlicingParams,
    config: &TreeSupportSettings,
    z: f64,
) -> LayerIndex {
    // TreeSupportCommon.hpp:606-608
    config.raft_layers.len() as LayerIndex
        + std::cmp::max(
            0,
            ((z - slicing_params.object_print_z_min - slicing_params.first_object_layer_height)
                / slicing_params.layer_height)
                .ceil() as LayerIndex,
        )
}

// TreeSupportCommon.hpp:610-616
// Highest collision layer
// inline LayerIndex layer_idx_floor(const SlicingParameters &slicing_params, const TreeSupportSettings &config, const double z)
pub fn layer_idx_floor(
    slicing_params: &SlicingParams,
    config: &TreeSupportSettings,
    z: f64,
) -> LayerIndex {
    // TreeSupportCommon.hpp:613-615
    config.raft_layers.len() as LayerIndex
        + std::cmp::max(
            0,
            ((z - slicing_params.object_print_z_min - slicing_params.first_object_layer_height)
                / slicing_params.layer_height)
                .floor() as LayerIndex,
        )
}

// TreeSupportCommon.hpp:618-628
// inline SupportGeneratorLayer& layer_initialize(SupportGeneratorLayer &layer_new, const SlicingParameters &slicing_params, const TreeSupportSettings &config, const size_t layer_idx)
pub fn layer_initialize<'a>(
    layer_new: &'a mut SupportGeneratorLayer,
    slicing_params: &SlicingParams,
    config: &TreeSupportSettings,
    layer_idx: usize,
) -> &'a mut SupportGeneratorLayer {
    // TreeSupportCommon.hpp:624  layer_new.print_z  = layer_z(slicing_params, config, layer_idx);
    layer_new.print_z = layer_z(slicing_params, config, layer_idx);
    // TreeSupportCommon.hpp:625  layer_new.bottom_z = layer_idx > 0 ? layer_z(slicing_params, config, layer_idx - 1) : 0;
    layer_new.bottom_z = if layer_idx > 0 {
        layer_z(slicing_params, config, layer_idx - 1)
    } else {
        0.
    };
    // TreeSupportCommon.hpp:626  layer_new.height   = layer_new.print_z - layer_new.bottom_z;
    layer_new.height = layer_new.print_z - layer_new.bottom_z;
    // TreeSupportCommon.hpp:627  return layer_new;
    layer_new
}

// TreeSupportCommon.hpp:630-640
// Using the std::deque as an allocator.
// inline SupportGeneratorLayer& layer_allocate_unguarded(SupportGeneratorLayerStorage &layer_storage, SupporLayerType layer_type, const SlicingParameters &slicing_params, const TreeSupportSettings &config, size_t layer_idx)
//
// In the Rust port, `SupportGeneratorLayerStorage::allocate*` hand back the index
// of the freshly allocated layer (mirroring the `Vec<usize>` pointer-vector
// modeling used by `SupportGeneratorLayersPtr`), rather than a borrow that would
// alias the storage. The caller threads the index back into the storage to mutate.
pub fn layer_allocate_unguarded(
    layer_storage: &mut SupportGeneratorLayerStorage,
    layer_type: SupporLayerType,
    slicing_params: &SlicingParams,
    config: &TreeSupportSettings,
    layer_idx: usize,
) -> usize {
    // TreeSupportCommon.hpp:638  SupportGeneratorLayer &layer = layer_storage.allocate_unguarded(layer_type);
    let layer = layer_storage.allocate_unguarded(layer_type);
    // TreeSupportCommon.hpp:639  return layer_initialize(layer, slicing_params, config, layer_idx);
    layer_initialize(layer, slicing_params, config, layer_idx);
    layer_storage.len() - 1
}

// TreeSupportCommon.hpp:642-651
// inline SupportGeneratorLayer& layer_allocate(SupportGeneratorLayerStorage &layer_storage, SupporLayerType layer_type, const SlicingParameters &slicing_params, const TreeSupportSettings &config, size_t layer_idx)
pub fn layer_allocate(
    layer_storage: &mut SupportGeneratorLayerStorage,
    layer_type: SupporLayerType,
    slicing_params: &SlicingParams,
    config: &TreeSupportSettings,
    layer_idx: usize,
) -> usize {
    // TreeSupportCommon.hpp:649  SupportGeneratorLayer &layer = layer_storage.allocate(layer_type);
    let layer = layer_storage.allocate(layer_type);
    // TreeSupportCommon.hpp:650  return layer_initialize(layer, slicing_params, config, layer_idx);
    layer_initialize(layer, slicing_params, config, layer_idx);
    layer_storage.len() - 1
}

// TreeSupportCommon.hpp:653-727
// Used by generate_initial_areas() in parallel by multiple layers.
// class InterfacePlacer
//
// The C++ class borrows the layer storage and the three layer-ptr vectors by
// mutable reference and guards them with a `std::mutex`. The Rust modeling holds
// owned views back into the storage by index (per the `SupportGeneratorLayersPtr =
// Vec<usize>` convention). To match the single-threaded port of
// `SupportGeneratorLayerStorage` (its TBB mutex was dropped), the mutex is omitted.
//
// Because Rust cannot store a `&mut` to the storage alongside `&mut` to the
// pointer vectors simultaneously, the storage and the three pointer vectors are
// threaded into the mutating methods rather than held as fields. The borrowed
// read-only inputs (slicing/support params and config) ARE held as fields,
// matching the C++ member layout.
pub struct InterfacePlacer<'a> {
    // TreeSupportCommon.hpp:673  const SlicingParameters &slicing_parameters;
    pub slicing_parameters: &'a SlicingParams,
    // TreeSupportCommon.hpp:674  const SupportParameters &support_parameters;
    pub support_parameters: &'a SupportParameters,
    // TreeSupportCommon.hpp:675  const TreeSupportSettings &config;
    pub config: &'a TreeSupportSettings,
}

impl<'a> InterfacePlacer<'a> {
    // TreeSupportCommon.hpp:656-667
    // InterfacePlacer(const SlicingParameters&, const SupportParameters&, const TreeSupportSettings&,
    //                 SupportGeneratorLayerStorage&, SupportGeneratorLayersPtr&, SupportGeneratorLayersPtr&, SupportGeneratorLayersPtr&)
    pub fn new(
        slicing_parameters: &'a SlicingParams,
        support_parameters: &'a SupportParameters,
        config: &'a TreeSupportSettings,
    ) -> Self {
        // TreeSupportCommon.hpp:665-666
        Self {
            slicing_parameters,
            support_parameters,
            config,
        }
    }

    // TreeSupportCommon.hpp:680-688
    // Insert the contact layer and some of the inteface and base interface layers below.
    // void add_roofs(std::vector<Polygons> &&new_roofs, const size_t insert_layer_idx)
    #[allow(clippy::too_many_arguments)]
    pub fn add_roofs(
        &mut self,
        new_roofs: Vec<Polygons>,
        insert_layer_idx: usize,
        layer_storage: &mut SupportGeneratorLayerStorage,
        top_contacts: &mut SupportGeneratorLayersPtr,
        top_interfaces: &mut SupportGeneratorLayersPtr,
        top_base_interfaces: &mut SupportGeneratorLayersPtr,
    ) {
        // TreeSupportCommon.hpp:682  if (! new_roofs.empty()) {
        if !new_roofs.is_empty() {
            // TreeSupportCommon.hpp:683  std::lock_guard<std::mutex> lock(m_mutex_layer_storage); (single-threaded; no mutex)
            // TreeSupportCommon.hpp:684  for (size_t idx = 0; idx < new_roofs.size(); ++ idx)
            for (idx, new_roof) in new_roofs.into_iter().enumerate() {
                // TreeSupportCommon.hpp:685  if (! new_roofs[idx].empty())
                if !new_roof.is_empty() {
                    // TreeSupportCommon.hpp:686  add_roof_unguarded(std::move(new_roofs[idx]), insert_layer_idx - idx, idx);
                    self.add_roof_unguarded(
                        new_roof,
                        insert_layer_idx - idx,
                        idx,
                        layer_storage,
                        top_contacts,
                        top_interfaces,
                        top_base_interfaces,
                    );
                }
            }
        }
    }

    // TreeSupportCommon.hpp:690-694
    // void add_roof(Polygons &&new_roof, const size_t insert_layer_idx, const size_t dtt_tip)
    #[allow(clippy::too_many_arguments)]
    pub fn add_roof(
        &mut self,
        new_roof: Polygons,
        insert_layer_idx: usize,
        dtt_tip: usize,
        layer_storage: &mut SupportGeneratorLayerStorage,
        top_contacts: &mut SupportGeneratorLayersPtr,
        top_interfaces: &mut SupportGeneratorLayersPtr,
        top_base_interfaces: &mut SupportGeneratorLayersPtr,
    ) {
        // TreeSupportCommon.hpp:692  std::lock_guard<std::mutex> lock(m_mutex_layer_storage); (single-threaded; no mutex)
        // TreeSupportCommon.hpp:693  add_roof_unguarded(std::move(new_roof), insert_layer_idx, dtt_tip);
        self.add_roof_unguarded(
            new_roof,
            insert_layer_idx,
            dtt_tip,
            layer_storage,
            top_contacts,
            top_interfaces,
            top_base_interfaces,
        );
    }

    // TreeSupportCommon.hpp:696-701
    // called by sample_overhang_area()
    // void add_roof_build_plate(Polygons &&overhang_areas, size_t dtt_roof)
    #[allow(clippy::too_many_arguments)]
    pub fn add_roof_build_plate(
        &mut self,
        overhang_areas: Polygons,
        dtt_roof: usize,
        layer_storage: &mut SupportGeneratorLayerStorage,
        top_contacts: &mut SupportGeneratorLayersPtr,
        top_interfaces: &mut SupportGeneratorLayersPtr,
        top_base_interfaces: &mut SupportGeneratorLayersPtr,
    ) {
        // TreeSupportCommon.hpp:699  std::lock_guard<std::mutex> lock(m_mutex_layer_storage); (single-threaded; no mutex)
        // TreeSupportCommon.hpp:700  this->add_roof_unguarded(std::move(overhang_areas), 0, std::min(dtt_roof, this->support_parameters.num_top_interface_layers));
        self.add_roof_unguarded(
            overhang_areas,
            0,
            std::cmp::min(dtt_roof, self.support_parameters.num_top_interface_layers),
            layer_storage,
            top_contacts,
            top_interfaces,
            top_base_interfaces,
        );
    }

    // TreeSupportCommon.hpp:703-716
    // void add_roof_unguarded(Polygons &&new_roofs, const size_t insert_layer_idx, const size_t dtt_roof)
    #[allow(clippy::too_many_arguments)]
    pub fn add_roof_unguarded(
        &mut self,
        new_roofs: Polygons,
        insert_layer_idx: usize,
        dtt_roof: usize,
        layer_storage: &mut SupportGeneratorLayerStorage,
        top_contacts: &mut SupportGeneratorLayersPtr,
        top_interfaces: &mut SupportGeneratorLayersPtr,
        top_base_interfaces: &mut SupportGeneratorLayersPtr,
    ) {
        // TreeSupportCommon.hpp:705  assert(support_parameters.has_top_contacts);
        debug_assert!(self.support_parameters.has_top_contacts);
        // TreeSupportCommon.hpp:706  assert(dtt_roof <= support_parameters.num_top_interface_layers);
        debug_assert!(dtt_roof <= self.support_parameters.num_top_interface_layers);
        // TreeSupportCommon.hpp:707-709
        // SupportGeneratorLayersPtr &layers =
        //     dtt_roof == 0 ? this->top_contacts :
        //     dtt_roof <= support_parameters.num_top_interface_layers_only() ? this->top_interfaces : this->top_base_interfaces;
        let layers: &mut SupportGeneratorLayersPtr = if dtt_roof == 0 {
            top_contacts
        } else if dtt_roof <= self.support_parameters.num_top_interface_layers_only() {
            top_interfaces
        } else {
            top_base_interfaces
        };
        // TreeSupportCommon.hpp:710  SupportGeneratorLayer*& l = layers[insert_layer_idx];
        // TreeSupportCommon.hpp:711  if (l == nullptr)
        if layers[insert_layer_idx] == SUPPORT_GENERATOR_LAYER_PTR_NULL {
            // TreeSupportCommon.hpp:712-713
            // l = &layer_allocate_unguarded(layer_storage, dtt_roof == 0 ? SupporLayerType::sltTopContact : SupporLayerType::sltTopInterface,
            //         slicing_parameters, config, insert_layer_idx);
            let idx = layer_allocate_unguarded(
                layer_storage,
                if dtt_roof == 0 {
                    SupporLayerType::SltTopContact
                } else {
                    SupporLayerType::SltTopInterface
                },
                self.slicing_parameters,
                self.config,
                insert_layer_idx,
            );
            layers[insert_layer_idx] = idx;
        }
        // TreeSupportCommon.hpp:714-715
        // will be unioned in finalize_interface_and_support_areas()
        // append(l->polygons, std::move(new_roofs));
        let l = layers[insert_layer_idx];
        layer_storage[l].polygons.extend(new_roofs);
    }
}

// Sentinel for a null `SupportGeneratorLayer*` slot in a `SupportGeneratorLayersPtr`
// (which is a `Vec<usize>` of storage indices). `usize::MAX` stands in for nullptr.
pub const SUPPORT_GENERATOR_LAYER_PTR_NULL: usize = usize::MAX;

// TreeSupportCommon.hpp:729-737
// enum class LineStatus
// Variant names preserved verbatim from the C++ enum (SCREAMING_CASE), per the
// 1:1 naming requirement; the camel-case style lint is suppressed accordingly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum LineStatus {
    INVALID,
    TO_MODEL,
    TO_MODEL_GRACIOUS,
    TO_MODEL_GRACIOUS_SAFE,
    TO_BP,
    TO_BP_SAFE,
}
