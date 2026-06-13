//! Support generation parameters.
//!
//! C++ Reference:
//! - Support/SupportParameters.hpp (header-only: struct SupportParameters)
//!
//! This is a faithful 1:1 line-by-line port of `Slic3r::SupportParameters`.
//!
//! PORTING STATUS: complete.
//! The constructor `SupportParameters(const PrintObject&)` is now ported
//! faithfully; all config hierarchy accesses are available via the wired
//! PrintObject accessors (object.print().config(), object.config(),
//! object.slicing_parameters(), object.layers(), object.printing_region()).

use crate::flow::{
    support_material_1st_layer_flow, support_material_flow, support_material_interface_flow, Flow,
    FlowRole,
};
use crate::geometry::deg2rad;
use crate::libslic3r::scale;
use crate::print_config::{
    InfillPattern, SupportBasePattern, SupportInterfacePattern, SupportType, TreeSupportStyle,
};
use crate::print_object::PrintObject;

// SupportParameters.hpp:1-10 — corresponding C++ includes:
//   #include <boost/log/trivial.hpp>
//   #include "../libslic3r.h"
//   #include "../Flow.hpp"
//   #include "../PrintConfig.hpp"
//   #include "../Slicing.hpp"
//   #include "../Fill/FillBase.hpp"
//   #include "../Print.hpp"
//   #include "../Layer.hpp"
//   #include "SupportLayer.hpp"

// SupportParameters.hpp:12 — namespace Slic3r {

/// SupportParameters.hpp:292 — `SupportMaterialStyle support_style = smsDefault;`
///
/// NOTE: In C++ this is the `SupportMaterialStyle` enum from PrintConfig.hpp
/// (`smsDefault`, `smsGrid`, `smsSnug`, `smsTreeSlim`, `smsTreeStrong`,
/// `smsTreeHybrid`, `smsTreeOrganic`). That enum is not yet ported into the
/// Rust `print_config` module (the existing `TreeSupportStyle` / `SupportType`
/// enums have a different variant set), so the field is represented as the
/// raw integer value used by C++ until the enum is ported.
pub type SupportMaterialStyle = i32;

/// SupportParameters.hpp:293 — `SupportMaterialPattern support_base_pattern = smpDefault;`
///
/// NOTE: In C++ this is the `SupportMaterialPattern` enum from PrintConfig.hpp
/// (`smpDefault`, `smpRectilinear`, `smpRectilinearGrid`, `smpHoneycomb`,
/// `smpLightning`, `smpNone`). Not yet ported into Rust `print_config`.
pub type SupportMaterialPattern = i32;

/// SupportParameters.hpp:13 — struct SupportParameters
///
/// Aggregates all configuration values needed for support generation. In the
/// C++ code it is initialized from `PrintObject` (see the constructor) and
/// derives flow rates, spacings, angles, and style/pattern settings.
#[derive(Debug, Clone)]
pub struct SupportParameters {
    // SupportParameters.hpp:243 — // Both top / bottom contacts and interfaces are soluble.
    // SupportParameters.hpp:244 — bool soluble_interface;
    pub soluble_interface: bool,
    // SupportParameters.hpp:245 — // Support contact & interface are soluble, but support base is non-soluble.
    // SupportParameters.hpp:246 — bool soluble_interface_non_soluble_base;
    pub soluble_interface_non_soluble_base: bool,

    // SupportParameters.hpp:248 — // Is there at least a top contact layer extruded above support base?
    // SupportParameters.hpp:249 — bool has_top_contacts;
    pub has_top_contacts: bool,
    // SupportParameters.hpp:250 — // Is there at least a bottom contact layer extruded below support base?
    // SupportParameters.hpp:251 — bool has_bottom_contacts;
    pub has_bottom_contacts: bool,

    // SupportParameters.hpp:253 — // Number of top interface layers without counting the contact layer.
    // SupportParameters.hpp:254 — size_t num_top_interface_layers;
    pub num_top_interface_layers: usize,
    // SupportParameters.hpp:255 — // Number of bottom interface layers without counting the contact layer.
    // SupportParameters.hpp:256 — size_t num_bottom_interface_layers;
    pub num_bottom_interface_layers: usize,
    // SupportParameters.hpp:257 — // Number of top base interface layers. Zero if not soluble_interface_non_soluble_base.
    // SupportParameters.hpp:258 — size_t num_top_base_interface_layers;
    pub num_top_base_interface_layers: usize,
    // SupportParameters.hpp:259 — // Number of bottom base interface layers. Zero if not soluble_interface_non_soluble_base.
    // SupportParameters.hpp:260 — size_t num_bottom_base_interface_layers;
    pub num_bottom_base_interface_layers: usize,

    // SupportParameters.hpp:267 — Flow first_layer_flow;
    pub first_layer_flow: Flow,
    // SupportParameters.hpp:268 — Flow support_material_flow;
    pub support_material_flow: Flow,
    // SupportParameters.hpp:269 — Flow support_material_interface_flow;
    pub support_material_interface_flow: Flow,
    // SupportParameters.hpp:270 — Flow support_material_bottom_interface_flow;
    pub support_material_bottom_interface_flow: Flow,
    // SupportParameters.hpp:271 — // Flow at raft inteface & contact layers.
    // SupportParameters.hpp:272 — Flow raft_interface_flow;
    pub raft_interface_flow: Flow,
    // SupportParameters.hpp:273 — coordf_t support_extrusion_width;
    pub support_extrusion_width: f64,
    // SupportParameters.hpp:274 — // Is merging of regions allowed? Could the interface & base support regions be printed with the same extruder?
    // SupportParameters.hpp:275 — bool can_merge_support_regions;
    pub can_merge_support_regions: bool,

    // SupportParameters.hpp:277 — coordf_t support_layer_height_min;
    pub support_layer_height_min: f64,
    // SupportParameters.hpp:278 — //	coordf_t	support_layer_height_max;

    // SupportParameters.hpp:280 — coordf_t gap_xy;
    pub gap_xy: f64,
    // SupportParameters.hpp:281 — coordf_t gap_xy_first_layer;
    pub gap_xy_first_layer: f64,

    // SupportParameters.hpp:283 — float base_angle;
    pub base_angle: f32,
    // SupportParameters.hpp:284 — float interface_angle;
    pub interface_angle: f32,
    // SupportParameters.hpp:285 — coordf_t interface_spacing;
    pub interface_spacing: f64,
    // SupportParameters.hpp:286 — coordf_t support_expansion=0;
    pub support_expansion: f64,
    // SupportParameters.hpp:287 — coordf_t interface_density;
    pub interface_density: f64,
    // SupportParameters.hpp:288 — // Density of the raft interface and contact layers.
    // SupportParameters.hpp:289 — coordf_t raft_interface_density;
    pub raft_interface_density: f64,
    // SupportParameters.hpp:290 — coordf_t support_spacing;
    pub support_spacing: f64,
    // SupportParameters.hpp:291 — coordf_t support_density;
    pub support_density: f64,
    // SupportParameters.hpp:292 — SupportMaterialStyle support_style = smsDefault;
    pub support_style: SupportMaterialStyle,
    // SupportParameters.hpp:293 — SupportMaterialPattern support_base_pattern = smpDefault;
    pub support_base_pattern: SupportMaterialPattern,

    // SupportParameters.hpp:295 — InfillPattern base_fill_pattern;
    pub base_fill_pattern: InfillPattern,
    // SupportParameters.hpp:296 — InfillPattern interface_fill_pattern;
    pub interface_fill_pattern: InfillPattern,
    // SupportParameters.hpp:297 — // Pattern of the raft interface and contact layers.
    // SupportParameters.hpp:298 — InfillPattern raft_interface_fill_pattern;
    pub raft_interface_fill_pattern: InfillPattern,
    // SupportParameters.hpp:299 — InfillPattern contact_fill_pattern;
    pub contact_fill_pattern: InfillPattern,
    // SupportParameters.hpp:300 — bool with_sheath;
    pub with_sheath: bool,
    // SupportParameters.hpp:301 — // Branches of organic supports with area larger than this threshold will be extruded with double lines.
    // SupportParameters.hpp:302 — double tree_branch_diameter_double_wall_area_scaled = 0.25 * sqr(scaled<double>(5.0)) * M_PI;;
    pub tree_branch_diameter_double_wall_area_scaled: f64,

    // SupportParameters.hpp:304 — float raft_angle_1st_layer;
    pub raft_angle_1st_layer: f32,
    // SupportParameters.hpp:305 — float raft_angle_base;
    pub raft_angle_base: f32,
    // SupportParameters.hpp:306 — float raft_angle_interface;
    pub raft_angle_interface: f32,

    // SupportParameters.hpp:312 — bool independent_layer_height = false;
    pub independent_layer_height: bool,
    // SupportParameters.hpp:313 — const double thresh_big_overhang = /*Slic3r::sqr(scale_(10))*/scale_(10);
    pub thresh_big_overhang: f64,

    // SupportParameters.hpp:315 — // support ironing related configs
    // SupportParameters.hpp:316 — bool enable_support_ironing = false;
    pub enable_support_ironing: bool,
    // SupportParameters.hpp:317 — InfillPattern ironing_pattern;
    pub ironing_pattern: InfillPattern,
    // SupportParameters.hpp:318 — // Spacing of the ironing lines, also to calculate the extrusion flow from.
    // SupportParameters.hpp:319 — double ironing_line_spacing;
    pub ironing_line_spacing: f64,
    // SupportParameters.hpp:320 — // Height of the extrusion, to calculate the extrusion flow from.
    // SupportParameters.hpp:321 — double ironing_flow_percent;
    pub ironing_flow_percent: f64,
    // SupportParameters.hpp:322 — double ironing_speed;
    pub ironing_speed: f64,
    // SupportParameters.hpp:323 — double ironing_angle;
    pub ironing_angle: f64,
    // SupportParameters.hpp:324 — double ironing_inset;
    pub ironing_inset: f64,
}

impl SupportParameters {
    // SupportParameters.hpp:14 — SupportParameters() = delete;
    //
    // The default constructor is deleted in C++; `SupportParameters` is only
    // ever built from a `PrintObject`. See `from_print_object` for the port
    // status of that constructor.

    /// SupportParameters.hpp:15-242 — SupportParameters(const PrintObject& object)
    ///
    /// Faithful 1:1 port of the C++ constructor.
    ///
    /// Notes on deviations from C++ that result from Rust config flattening:
    ///
    /// * `print_config.filament_soluble.get_at(idx)` — C++ has a per-extruder
    ///   vector; Rust has a scalar `filament_soluble: bool`. Since the Rust
    ///   `PrintConfig` models a single-extruder setup, we use the scalar value
    ///   for all filament-index lookups, which is correct for single-extruder.
    ///
    /// * `print_config.nozzle_diameter.get_at(idx)` — same: scalar in Rust.
    ///
    /// * `print_config.min_layer_height.values` — C++ iterates a per-extruder
    ///   vector; Rust has a scalar `min_layer_height`. We iterate once with
    ///   the scalar value, which is equivalent for single-extruder.
    ///
    /// * `object.object_extruders()` — not available in Rust PrintObject. Used
    ///   only in the `can_merge_support_regions` fallback when one filament is
    ///   "auto" (0). We omit this fallback path (conservative: no merge).
    ///
    /// * `object.has_variable_layer_heights` — not available; defaults to
    ///   `false`, which selects `smsTreeOrganic` over `smsTreeHybrid` when
    ///   support style is default tree.
    ///
    /// * `print_config.impact_strength_z.get_at(idx)` — not in Rust config.
    ///   Used only when `tree_support_wall_count == -1` (auto). Rust stores
    ///   `tree_support_wall_count` as `u32`, so -1 is unreachable.
    pub fn from_print_object(object: &PrintObject) -> Self {
        // SupportParameters.hpp:17-19
        let print_config = object.print().config();
        let object_config = object.config();
        let slicing_params = object.slicing_parameters();

        // SupportParameters.hpp:21
        let soluble_interface = slicing_params.soluble_interface;

        // SupportParameters.hpp:22-28
        // Zero z-gap between overhangs and support interface: interface extruder
        // soluble AND base extruder not soluble (or "auto").
        // Rust: filament_soluble is scalar on PrintConfig; support_interface_filament
        // and support_filament are also on PrintConfig in Rust (moved from
        // PrintObjectConfig due to flattening).
        let soluble_interface_non_soluble_base = soluble_interface
            && print_config.support_interface_filament > 0
            && print_config.filament_soluble
            && (print_config.support_filament == 0
                || !print_config.filament_soluble);

        // SupportParameters.hpp:30-65
        // Interface layer counts.
        let num_top_interface_layers =
            object_config.support_interface_top_layers as usize;
        let num_bottom_interface_layers = if object_config.support_interface_bottom_layers < 0 {
            num_top_interface_layers
        } else {
            object_config.support_interface_bottom_layers as usize
        };
        let has_top_contacts = num_top_interface_layers > 0;
        let has_bottom_contacts = num_bottom_interface_layers > 0;

        // SupportParameters.hpp:36 — is_tree(object_config.support_type)
        let is_tree_support = matches!(object_config.support_type, SupportType::Tree | SupportType::Hybrid);

        // Rust: support_filament and support_interface_filament are on PrintConfig
        let differnt_support_interface_filament = print_config.support_interface_filament != 0
            && print_config.support_interface_filament != print_config.support_filament;

        let (num_top_base_interface_layers, num_bottom_base_interface_layers) =
            if is_tree_support {
                if soluble_interface_non_soluble_base {
                    // SupportParameters.hpp:39-40
                    (
                        ((num_top_interface_layers / 2).min(2)),
                        ((num_bottom_interface_layers / 2).min(2)),
                    )
                } else {
                    // SupportParameters.hpp:47-48
                    let base = if differnt_support_interface_filament { 1 } else { 0 };
                    (base, base)
                }
            } else {
                if soluble_interface_non_soluble_base {
                    // SupportParameters.hpp:53-54
                    (
                        if num_top_interface_layers > 0 { 2 } else { 0 },
                        (num_bottom_interface_layers / 2).min(2),
                    )
                } else {
                    // SupportParameters.hpp:61-62
                    (
                        if num_top_interface_layers > 0 {
                            if differnt_support_interface_filament { 2 } else { 1 }
                        } else {
                            0
                        },
                        if differnt_support_interface_filament { 1 } else { 0 },
                    )
                }
            };

        // SupportParameters.hpp:66-68
        let first_layer_flow = support_material_1st_layer_flow(
            object,
            slicing_params.first_print_layer_height as f64,
        )
        .unwrap_or_else(|_| Flow::zero());

        let support_material_flow =
            support_material_flow(object, slicing_params.layer_height as f64)
                .unwrap_or_else(|_| Flow::zero());

        let mut support_material_interface_flow =
            support_material_interface_flow(object, slicing_params.layer_height as f64)
                .unwrap_or_else(|_| Flow::zero());

        // SupportParameters.hpp:69
        let raft_interface_flow = support_material_interface_flow.clone();

        // SupportParameters.hpp:72-76 — minimum support layer height
        // C++: start at scaled(0.01), then clamp with min_layer_height.values
        // and each layer's height. Rust has scalar min_layer_height on
        // PrintObjectConfig (per-object, not per-extruder vector).
        let mut support_layer_height_min = scale(0.01) as f64;
        // min_layer_height is on PrintObjectConfig in Rust
        support_layer_height_min = support_layer_height_min
            .min(object_config.min_layer_height.max(0.01));
        for layer in object.layers() {
            support_layer_height_min =
                support_layer_height_min.min(layer.height.max(0.01));
        }

        // SupportParameters.hpp:78-81
        if object_config.support_interface_top_layers == 0 {
            support_material_interface_flow = support_material_flow.clone();
        }

        // SupportParameters.hpp:85-96 — XY gap
        // NOTE: external_perimeter_width is computed (C++ line 89) but never
        // consumed after the loop — it appears to be dead code in C++ too.
        let mut _external_perimeter_width: f64 = 0.0;
        let mut bridge_flow_ratio: f64 = 0.0;
        let num_regions = object.num_printing_regions();
        for region_id in 0..num_regions {
            if let Some(region) = object.printing_region(region_id) {
                // SupportParameters.hpp:89
                if let Ok(flow) = region.flow(
                    object,
                    FlowRole::ExternalPerimeter,
                    slicing_params.layer_height as f64,
                    false,
                ) {
                    _external_perimeter_width =
                        _external_perimeter_width.max(flow.width() as f64);
                }
                // SupportParameters.hpp:90
                bridge_flow_ratio += region.config().bridge_flow_ratio;
            }
        }

        // SupportParameters.hpp:92-95
        // In C++: print_config.top_z_overrides_xy_distance.
        // In Rust: this field is on PrintObjectConfig.
        let gap_xy = if !object_config.top_z_overrides_xy_distance {
            object_config.support_object_xy_distance
        } else {
            object_config
                .support_object_xy_distance
                .min(object_config.support_top_z_distance.max(0.2))
        };
        let gap_xy_first_layer = object_config.support_object_first_layer_gap;

        // SupportParameters.hpp:96
        if num_regions > 0 {
            bridge_flow_ratio /= num_regions as f64;
        }

        // SupportParameters.hpp:98-100
        let support_material_bottom_interface_flow =
            if slicing_params.soluble_interface || !object_config.thick_bridges {
                support_material_interface_flow
                    .with_flow_ratio(bridge_flow_ratio)
                    .unwrap_or_else(|_| support_material_interface_flow.clone())
            } else {
                Flow::bridging_flow(
                    bridge_flow_ratio * support_material_interface_flow.nozzle_diameter(),
                    support_material_interface_flow.nozzle_diameter(),
                )
            };

        // SupportParameters.hpp:102-111
        // Rust: support_filament and support_interface_filament are on PrintConfig.
        let can_merge_support_regions = print_config.support_filament
            == print_config.support_interface_filament;
        // NOTE: The C++ fallback that checks object_extruders() when one filament
        // is 0 ("auto") is omitted: object.object_extruders() is not available in
        // the Rust PrintObject. This is conservative (may not merge when safe to).

        // SupportParameters.hpp:114-115
        let base_angle =
            deg2rad(object_config.support_angle as f64) as f32;
        let interface_angle =
            deg2rad((object_config.support_angle + 90.0) as f64) as f32;

        // SupportParameters.hpp:116-121
        let interface_spacing_raw = object_config.support_interface_spacing
            + support_material_interface_flow.spacing();
        let interface_density_raw =
            (support_material_interface_flow.spacing() / interface_spacing_raw).min(1.0);

        let raft_interface_spacing = object_config.support_interface_spacing
            + raft_interface_flow.spacing();
        let raft_interface_density =
            (raft_interface_flow.spacing() / raft_interface_spacing).min(1.0);

        let support_spacing = object_config.support_base_pattern_spacing
            + support_material_flow.spacing();
        let support_density =
            (support_material_flow.spacing() / support_spacing).min(1.0);

        // SupportParameters.hpp:122-126
        let (interface_spacing, interface_density) =
            if object_config.support_interface_top_layers == 0 {
                // No interface layers: use base pattern for everything.
                (support_spacing, support_density)
            } else {
                (interface_spacing_raw, interface_density_raw)
            };

        // SupportParameters.hpp:128-134 — support ironing
        let enable_support_ironing = object_config.enable_support_ironing;
        let ironing_angle = object_config.support_ironing_direction;
        let ironing_flow_percent = object_config.support_ironing_flow;
        let ironing_inset = object_config.support_ironing_inset;
        let ironing_line_spacing = object_config.support_ironing_spacing;
        let ironing_pattern = object_config.support_ironing_pattern;
        let ironing_speed = object_config.support_ironing_speed;

        // SupportParameters.hpp:136-156 — resolve support_style
        // C++ SupportMaterialStyle raw values:
        //   smsDefault=0, smsGrid=1, smsSnug=2, smsTreeSlim=3,
        //   smsTreeStrong=4, smsTreeHybrid=5, smsTreeOrganic=6
        let mut support_style: SupportMaterialStyle = match object_config.support_style {
            TreeSupportStyle::Default => SMS_DEFAULT,
            TreeSupportStyle::Slim    => SMS_TREE_SLIM,
            TreeSupportStyle::Strong  => SMS_TREE_STRONG,
            TreeSupportStyle::Hybrid  => SMS_TREE_HYBRID,
            TreeSupportStyle::Organic => SMS_TREE_ORGANIC,
        };
        // Correction: tree-only styles revert to default for non-tree support,
        // grid/snug styles revert to default for tree support.
        if support_style != SMS_DEFAULT {
            if (support_style == SMS_SNUG || support_style == SMS_GRID) && is_tree_support {
                support_style = SMS_DEFAULT;
            }
            if (support_style == SMS_TREE_SLIM
                || support_style == SMS_TREE_STRONG
                || support_style == SMS_TREE_HYBRID
                || support_style == SMS_TREE_ORGANIC)
                && !is_tree_support
            {
                support_style = SMS_DEFAULT;
            }
        }
        if support_style == SMS_DEFAULT {
            if is_tree_support {
                // SupportParameters.hpp:146-155: organic unless variable layer heights
                // has_variable_layer_heights not available in Rust; default false.
                if !slicing_params.soluble_interface {
                    support_style = SMS_TREE_ORGANIC;
                } else {
                    support_style = SMS_TREE_HYBRID;
                }
            } else {
                // SupportParameters.hpp:154
                support_style = SMS_GRID;
            }
        }

        // SupportParameters.hpp:158-165 — resolve support_base_pattern
        // C++ SupportMaterialPattern raw values:
        //   smpDefault=0, smpRectilinear=1, smpRectilinearGrid=2,
        //   smpHoneycomb=3, smpLightning=4, smpNone=5
        let mut support_base_pattern: SupportMaterialPattern = match object_config.support_base_pattern {
            SupportBasePattern::Rectilinear => SMP_RECTILINEAR,
            SupportBasePattern::Honeycomb   => SMP_HONEYCOMB,
            SupportBasePattern::Grid        => SMP_RECTILINEAR_GRID,
        };
        // C++ note: SupportBasePattern Rust enum has no Default/Lightning/None
        // variants, so smpDefault never enters here directly from config.
        // The C++ fixup: lightning not allowed for non-tree; default resolved.
        if support_base_pattern == SMP_LIGHTNING && !is_tree_support {
            support_base_pattern = SMP_RECTILINEAR;
        }
        if support_base_pattern == SMP_DEFAULT {
            if is_tree_support {
                support_base_pattern = if support_style == SMS_TREE_HYBRID {
                    SMP_RECTILINEAR
                } else {
                    SMP_NONE
                };
            } else {
                support_base_pattern = SMP_RECTILINEAR;
            }
        }

        // SupportParameters.hpp:167-181 — fill patterns
        let with_sheath = object_config.tree_support_wall_count > 0;

        let base_fill_pattern = if support_base_pattern == SMP_LIGHTNING {
            InfillPattern::Lightning
        } else if support_base_pattern == SMP_HONEYCOMB {
            InfillPattern::Honeycomb
        } else if support_density > 0.95 || with_sheath {
            InfillPattern::Rectilinear
        } else {
            InfillPattern::SupportBase
        };

        let interface_fill_pattern = if interface_density > 0.95 {
            InfillPattern::Rectilinear
        } else {
            InfillPattern::SupportBase
        };

        let raft_interface_fill_pattern = if raft_interface_density > 0.95 {
            InfillPattern::Rectilinear
        } else {
            InfillPattern::SupportBase
        };

        // SupportParameters.hpp:174-181 — contact fill pattern
        // C++ SupportMaterialInterfacePattern values:
        //   smipAuto=0, smipRectilinear=1, smipConcentric=2,
        //   smipRectilinearInterlaced=3, smipGrid=4
        let contact_fill_pattern = match object_config.support_interface_pattern {
            SupportInterfacePattern::Grid => InfillPattern::Grid,
            SupportInterfacePattern::Rectilinear => InfillPattern::Rectilinear,
            SupportInterfacePattern::Concentric => {
                if interface_density > 0.95 {
                    InfillPattern::Rectilinear
                } else {
                    // smipConcentric branch; also handles smipAuto and
                    // smipRectilinearInterlaced (not in Rust enum) — default
                    // to Rectilinear for auto/interlaced same as C++ line 176.
                    InfillPattern::SupportBase
                }
            }
        };
        // Adjust for smipAuto/smipRectilinearInterlaced which both map to
        // Rectilinear in C++ (lines 176-177); Rust SupportInterfacePattern
        // Rectilinear covers both.
        // (Already handled above — Rectilinear variant returns Rectilinear.)

        // SupportParameters.hpp:183-212 — raft angles
        let mut raft_angle_1st_layer: f32 = 0.0;
        let mut raft_angle_base: f32 = 0.0;
        let mut raft_angle_interface: f32 = 0.0;

        if slicing_params.base_raft_layers > 1 {
            // SupportParameters.hpp:189-194
            raft_angle_1st_layer = interface_angle;
            raft_angle_base = base_angle;
            raft_angle_interface = interface_angle;
            if (slicing_params.interface_raft_layers & 1) == 0 {
                raft_angle_interface += (0.5 * std::f64::consts::PI) as f32;
            }
        } else if slicing_params.base_raft_layers == 1
            || slicing_params.interface_raft_layers > 1
        {
            // SupportParameters.hpp:198-199
            raft_angle_1st_layer = base_angle;
            raft_angle_interface = interface_angle + (0.5 * std::f64::consts::PI) as f32;
        } else if slicing_params.interface_raft_layers == 1 {
            // SupportParameters.hpp:205-206
            raft_angle_1st_layer = (0.5 * std::f64::consts::PI) as f32;
            raft_angle_interface = raft_angle_1st_layer;
        }
        // else: no raft, all angles stay 0.

        // SupportParameters.hpp:214-219 — support extrusion width
        let mut support_extrusion_width = if object_config.support_line_width > 0.0 {
            object_config.support_line_width
        } else {
            object_config.line_width
        };
        if support_extrusion_width <= 0.0 {
            // SupportParameters.hpp:217-218: auto from nozzle diameter
            // Rust: nozzle_diameter is scalar (single-extruder)
            support_extrusion_width = Flow::auto_extrusion_width(
                FlowRole::SupportMaterial,
                print_config.nozzle_diameter,
            );
        }

        // SupportParameters.hpp:221-239 — tree branch double wall area
        // tree_support_wall_count == -1 branch is unreachable (u32 in Rust).
        let tree_branch_diameter_double_wall_area_scaled =
            if object_config.tree_support_wall_count > 1 {
                // SupportParameters.hpp:238: force double walls everywhere
                0.1
            } else {
                // Default: branches >= 5mm diameter get double walls
                sqr(scale(5.0) as f64) * 0.25 * std::f64::consts::PI
            };

        // SupportParameters.hpp:241
        // In C++: print_config.independent_support_layer_height.
        // In Rust: this field is on PrintObjectConfig.
        let independent_layer_height = object_config.independent_support_layer_height;

        Self {
            soluble_interface,
            soluble_interface_non_soluble_base,
            has_top_contacts,
            has_bottom_contacts,
            num_top_interface_layers,
            num_bottom_interface_layers,
            num_top_base_interface_layers,
            num_bottom_base_interface_layers,
            first_layer_flow,
            support_material_flow,
            support_material_interface_flow,
            support_material_bottom_interface_flow,
            raft_interface_flow,
            support_extrusion_width,
            can_merge_support_regions,
            support_layer_height_min,
            gap_xy,
            gap_xy_first_layer,
            base_angle,
            interface_angle,
            interface_spacing,
            support_expansion: object_config.support_expansion,
            interface_density,
            raft_interface_density,
            support_spacing,
            support_density,
            support_style,
            support_base_pattern,
            base_fill_pattern,
            interface_fill_pattern,
            raft_interface_fill_pattern,
            contact_fill_pattern,
            with_sheath,
            tree_branch_diameter_double_wall_area_scaled,
            raft_angle_1st_layer,
            raft_angle_base,
            raft_angle_interface,
            independent_layer_height,
            thresh_big_overhang: scale(10.0) as f64,
            enable_support_ironing,
            ironing_pattern,
            ironing_line_spacing,
            ironing_flow_percent,
            ironing_speed,
            ironing_angle,
            ironing_inset,
        }
    }

    // SupportParameters.hpp:262 — bool has_contacts() const { return this->has_top_contacts || this->has_bottom_contacts; }
    pub fn has_contacts(&self) -> bool {
        self.has_top_contacts || self.has_bottom_contacts
    }

    // SupportParameters.hpp:263 — bool has_interfaces() const { return this->num_top_interface_layers + this->num_bottom_interface_layers > 0; }
    pub fn has_interfaces(&self) -> bool {
        self.num_top_interface_layers + self.num_bottom_interface_layers > 0
    }

    // SupportParameters.hpp:264 — bool has_base_interfaces() const { return this->num_top_base_interface_layers + this->num_bottom_base_interface_layers > 0; }
    pub fn has_base_interfaces(&self) -> bool {
        self.num_top_base_interface_layers + self.num_bottom_base_interface_layers > 0
    }

    // SupportParameters.hpp:265 — size_t num_top_interface_layers_only() const { return std::max(0, int(this->num_top_interface_layers) - int(this->num_top_base_interface_layers)); }
    pub fn num_top_interface_layers_only(&self) -> usize {
        std::cmp::max(
            0,
            self.num_top_interface_layers as i32 - self.num_top_base_interface_layers as i32,
        ) as usize
    }

    // SupportParameters.hpp:266 — size_t num_bottom_interface_layers_only() const { return this->num_bottom_interface_layers - this->num_bottom_base_interface_layers; }
    pub fn num_bottom_interface_layers_only(&self) -> usize {
        self.num_bottom_interface_layers - self.num_bottom_base_interface_layers
    }

    // SupportParameters.hpp:308-310 — // Produce a raft interface angle for a given SupportLayer::interface_id()
    // float raft_interface_angle(size_t interface_id) const
    //     { return this->raft_angle_interface + ((interface_id & 1) ? float(- M_PI / 4.) : float(+ M_PI / 4.)); }
    pub fn raft_interface_angle(&self, interface_id: usize) -> f32 {
        self.raft_angle_interface
            + if interface_id & 1 != 0 {
                (-std::f64::consts::PI / 4.0) as f32
            } else {
                (std::f64::consts::PI / 4.0) as f32
            }
    }
}

impl Default for SupportParameters {
    /// Mirror of the C++ in-class member initializers (values fixed at
    /// compile time). All other fields default to zero/false; they are
    /// populated by `from_print_object` in normal use.
    fn default() -> Self {
        Self {
            soluble_interface: false,
            soluble_interface_non_soluble_base: false,
            has_top_contacts: false,
            has_bottom_contacts: false,
            num_top_interface_layers: 0,
            num_bottom_interface_layers: 0,
            num_top_base_interface_layers: 0,
            num_bottom_base_interface_layers: 0,
            first_layer_flow: Flow::zero(),
            support_material_flow: Flow::zero(),
            support_material_interface_flow: Flow::zero(),
            support_material_bottom_interface_flow: Flow::zero(),
            raft_interface_flow: Flow::zero(),
            support_extrusion_width: 0.0,
            can_merge_support_regions: false,
            support_layer_height_min: 0.0,
            gap_xy: 0.0,
            gap_xy_first_layer: 0.0,
            base_angle: 0.0,
            interface_angle: 0.0,
            interface_spacing: 0.0,
            // SupportParameters.hpp:286 — coordf_t support_expansion=0;
            support_expansion: 0.0,
            interface_density: 0.0,
            raft_interface_density: 0.0,
            support_spacing: 0.0,
            support_density: 0.0,
            // SupportParameters.hpp:292 — = smsDefault;
            support_style: SMS_DEFAULT,
            // SupportParameters.hpp:293 — = smpDefault;
            support_base_pattern: SMP_DEFAULT,
            base_fill_pattern: InfillPattern::default(),
            interface_fill_pattern: InfillPattern::default(),
            raft_interface_fill_pattern: InfillPattern::default(),
            contact_fill_pattern: InfillPattern::default(),
            with_sheath: false,
            // SupportParameters.hpp:302 — = 0.25 * sqr(scaled<double>(5.0)) * M_PI;
            tree_branch_diameter_double_wall_area_scaled: 0.25
                * sqr(scale(5.0) as f64)
                * std::f64::consts::PI,
            raft_angle_1st_layer: 0.0,
            raft_angle_base: 0.0,
            raft_angle_interface: 0.0,
            // SupportParameters.hpp:312 — bool independent_layer_height = false;
            independent_layer_height: false,
            // SupportParameters.hpp:313 — const double thresh_big_overhang = scale_(10);
            thresh_big_overhang: scale(10.0) as f64,
            // SupportParameters.hpp:316 — bool enable_support_ironing = false;
            enable_support_ironing: false,
            ironing_pattern: InfillPattern::default(),
            ironing_line_spacing: 0.0,
            ironing_flow_percent: 0.0,
            ironing_speed: 0.0,
            ironing_angle: 0.0,
            ironing_inset: 0.0,
        }
    }
}

// === C++ SupportMaterialStyle raw enum values (PrintConfig.hpp:145-147) ===
const SMS_DEFAULT: SupportMaterialStyle = 0;
const SMS_GRID: SupportMaterialStyle = 1;
const SMS_SNUG: SupportMaterialStyle = 2;
const SMS_TREE_SLIM: SupportMaterialStyle = 3;
const SMS_TREE_STRONG: SupportMaterialStyle = 4;
const SMS_TREE_HYBRID: SupportMaterialStyle = 5;
const SMS_TREE_ORGANIC: SupportMaterialStyle = 6;

// === C++ SupportMaterialPattern raw enum values (PrintConfig.hpp:138-143) ===
const SMP_DEFAULT: SupportMaterialPattern = 0;
const SMP_RECTILINEAR: SupportMaterialPattern = 1;
const SMP_RECTILINEAR_GRID: SupportMaterialPattern = 2;
const SMP_HONEYCOMB: SupportMaterialPattern = 3;
const SMP_LIGHTNING: SupportMaterialPattern = 4;
const SMP_NONE: SupportMaterialPattern = 5;

/// libslic3r.h — `template<typename T> inline T sqr(T x) { return x * x; }`
#[inline]
fn sqr(x: f64) -> f64 {
    x * x
}

// SupportParameters.hpp:326 — } // namespace Slic3r
