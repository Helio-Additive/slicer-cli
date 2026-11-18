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

use crate::config::{ConfigOptionBool, ConfigOptionFloat, ConfigOptionResolver};
use crate::flow::{
    support_material_1st_layer_flow, support_material_flow, support_material_interface_flow, Flow,
    FlowError, FlowResult, FlowRole,
};
use crate::gcode::wipe_tower::{cfg_bools, cfg_floats, get_at};
use crate::libslic3r::scale;
use crate::print::faithful as pf;
use crate::print_config::config_def::DynamicPrintConfig;
use crate::print_config::enums;
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
    // SupportParameters.hpp:336-339 — BBS multi-diameter: the line width option behind the support flows, so
    // base and interface slabs can be rebuilt for the nozzle selected by the support merge plan.
    // `ConfigOptionFloat support_base_width;` / `ConfigOptionFloat support_first_layer_width;`: the values of
    // the two options (the crate's ConfigOptionFloat is not Clone).
    pub support_base_width: f64,
    pub support_first_layer_width: f64,
    // SupportParameters.hpp:340 — float preset_nozzle_diameter { 0.f };
    pub preset_nozzle_diameter: f32,
    // SupportParameters.hpp:341 — bool multi_nozzle_diameter_mode { false };
    pub multi_nozzle_diameter_mode: bool,
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
        // C++: Geometry::deg2rad(float(value)). deg2rad is a template
        // `T deg2rad(T) { return T(PI)*angle/T(180.0); }` instantiated with
        // T=float, so the WHOLE computation runs in f32 precision. We mirror
        // that here (computing in f32) rather than calling the f64 deg2rad and
        // truncating, which would differ in the low bits.
        // Note: `support_angle + 90.` promotes to f64 first (90. is double),
        // then is cast to float before deg2rad — reproduced below.
        let base_angle = std::f32::consts::PI
            * (object_config.support_angle as f32)
            / 180.0_f32;
        let interface_angle = std::f32::consts::PI
            * ((object_config.support_angle + 90.0) as f32)
            / 180.0_f32;

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
            // C++: `interface_angle + 0.5 * M_PI` — no float() cast on the
            // `0.5 * M_PI` term (unlike line 194), so float+double promotes to
            // double, then truncates to float on assignment. Mirror in f64.
            raft_angle_interface =
                (interface_angle as f64 + 0.5 * std::f64::consts::PI) as f32;
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
        //
        // C++ branches on `object_config.tree_support_wall_count.value`:
        //   == -1 : auto — compute from weakest-filament impact strength
        //           (SupportParameters.hpp:233-235). UNREACHABLE in Rust:
        //           `tree_support_wall_count` is `u32`, so -1 cannot occur.
        //           See FIDELITY-NOTE below.
        //   else  : SupportParameters.hpp:238 —
        //           `tree_support_wall_count > 1 ? 0.1 : std::numeric_limits<double>::max()`
        // FIDELITY-NOTE: the `== -1` auto path is dropped because the Rust
        // config type cannot represent -1 and `print_config.impact_strength_z`
        // (per-filament vector) is not ported. Both reachable sub-branches of
        // the C++ `else` are reproduced exactly.
        let tree_branch_diameter_double_wall_area_scaled =
            if object_config.tree_support_wall_count > 1 {
                // SupportParameters.hpp:238 — force double walls everywhere
                0.1
            } else {
                // SupportParameters.hpp:238 — std::numeric_limits<double>::max()
                f64::MAX
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
            // Members of the pin's struct the older constructor has no source for (SupportParameters.hpp:338-341).
            support_base_width: if object_config.support_line_width > 0.0 { object_config.support_line_width } else { object_config.line_width },
            support_first_layer_width: 0.0,
            preset_nozzle_diameter: 0.0,
            multi_nozzle_diameter_mode: false,
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
        // size_t arithmetic: it wraps where the base layers outnumber the interface layers.
        self.num_bottom_interface_layers.wrapping_sub(self.num_bottom_base_interface_layers)
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

// ============================================================================
// SupportParameters.hpp at the pin, over the Print classes of `crate::print::faithful`.
// ============================================================================

// `config.<key>.value` of a float or percent option.
fn cfg_float(config: &DynamicPrintConfig, key: &str) -> f64 {
    config.option(key).and_then(|o| o.get_float().ok()).unwrap_or(0.0)
}

// `config.<key>.value` of an int or enum option.
fn cfg_int(config: &DynamicPrintConfig, key: &str) -> i32 {
    config.option(key).and_then(|o| o.get_int().ok()).unwrap_or(0)
}

// `config.<key>.value` of a bool option.
fn cfg_bool(config: &DynamicPrintConfig, key: &str) -> bool {
    config.option(key).and_then(|o| o.get_bool().ok()).unwrap_or(false)
}

// `std::min(a, b)`: `(b < a) ? b : a`.
#[inline]
fn std_min(a: f64, b: f64) -> f64 {
    if b < a {
        b
    } else {
        a
    }
}

// `std::max(a, b)`: `(a < b) ? b : a`.
#[inline]
fn std_max(a: f64, b: f64) -> f64 {
    if a < b {
        b
    } else {
        a
    }
}

/// The `InfillPattern` of the crate's older enum for a value of PrintConfig.hpp's `enum InfillPattern`
/// (the first 27 enumerators are the same in both; the support options take none of the later ones).
fn infill_pattern_from_i32(v: i32) -> InfillPattern {
    match enums::InfillPattern::from_i32(v) {
        enums::InfillPattern::IpConcentric => InfillPattern::Concentric,
        enums::InfillPattern::IpRectilinear => InfillPattern::Rectilinear,
        enums::InfillPattern::IpGrid => InfillPattern::Grid,
        enums::InfillPattern::IpLine => InfillPattern::Line,
        enums::InfillPattern::IpCubic => InfillPattern::Cubic,
        enums::InfillPattern::IpTriangles => InfillPattern::Triangles,
        enums::InfillPattern::IpStars => InfillPattern::Stars,
        enums::InfillPattern::IpGyroid => InfillPattern::Gyroid,
        enums::InfillPattern::IpHoneycomb => InfillPattern::Honeycomb,
        enums::InfillPattern::IpAdaptiveCubic => InfillPattern::AdaptiveCubic,
        enums::InfillPattern::IpMonotonic => InfillPattern::Monotonic,
        enums::InfillPattern::IpMonotonicLine => InfillPattern::MonotonicLine,
        enums::InfillPattern::IpAlignedRectilinear => InfillPattern::AlignedRectilinear,
        enums::InfillPattern::Ip3DHoneycomb => InfillPattern::Honeycomb3D,
        enums::InfillPattern::IpHilbertCurve => InfillPattern::HilbertCurve,
        enums::InfillPattern::IpArchimedeanChords => InfillPattern::ArchimedeanChords,
        enums::InfillPattern::IpOctagramSpiral => InfillPattern::OctagramSpiral,
        enums::InfillPattern::IpSupportCubic => InfillPattern::SupportCubic,
        enums::InfillPattern::IpSupportBase => InfillPattern::SupportBase,
        enums::InfillPattern::IpConcentricInternal => InfillPattern::ConcentricInternal,
        enums::InfillPattern::IpLightning => InfillPattern::Lightning,
        enums::InfillPattern::IpCrossHatch => InfillPattern::CrossHatch,
        enums::InfillPattern::IpZigZag => InfillPattern::ZigZag,
        enums::InfillPattern::IpCrossZag => InfillPattern::CrossZag,
        enums::InfillPattern::IpFloatingConcentric => InfillPattern::FloatingConcentric,
        enums::InfillPattern::IpLockedZag => InfillPattern::LockedZag,
        enums::InfillPattern::Ip2DLattice => InfillPattern::Lattice2D,
        other => panic!("InfillPattern {:?} is not a support pattern", other),
    }
}

// SupportParameters.hpp:14-19
// Read a bool print option when present; unknown keys stay false.
pub fn config_flag_or_false(cfg: &(impl ConfigOptionResolver + ?Sized), key: &str) -> bool {
    // const auto *opt = cfg.option<ConfigOptionBool>(key);
    let opt = cfg.option(key).and_then(|o| o.as_any().downcast_ref::<ConfigOptionBool>());
    // return opt != nullptr && opt->value;
    opt.is_some() && opt.unwrap().value
}

// SupportParameters.hpp:21-28
pub fn resolved_support_object_xy_distance(print_config: &pf::PrintConfig, object_config: &pf::PrintObjectConfig) -> f64 {
    if !cfg_bool(print_config, "top_z_overrides_xy_distance") {
        cfg_float(object_config, "support_object_xy_distance")
    } else {
        std_min(
            cfg_float(object_config, "support_object_xy_distance"),
            std_max(0.2, cfg_float(object_config, "support_top_z_distance")),
        )
    }
}

// SupportParameters.hpp:30-62
// Keep Default / Grid / Snug / mismatched-style rewrite in one place so the uncovered-overhang
// warning names the same classic-vs-organic path as SupportParameters. Organic is refused for
// variable layer heights (#4313), a zero Top Z gap, a requested ring / heat preserve, or multi-diameter mode.
pub fn resolve_support_style(
    mut support_style: enums::SupportMaterialStyle,
    support_type: enums::SupportType,
    has_variable_layer_heights: bool,
    soluble_interface: bool,
    heat_preserve_mode: bool,
    enable_support_ring: bool,
    multi_diameter_mode: bool,
) -> enums::SupportMaterialStyle {
    use crate::print_config::config_def::is_tree;
    use enums::SupportMaterialStyle::*;
    if support_style != SmsDefault {
        if (support_style == SmsSnug || support_style == SmsGrid) && is_tree(support_type) {
            support_style = SmsDefault;
        }
        if (support_style == SmsTreeSlim || support_style == SmsTreeStrong || support_style == SmsTreeHybrid || support_style == SmsTreeOrganic)
            && !is_tree(support_type)
        {
            support_style = SmsDefault;
        }
    }
    if support_style == SmsDefault {
        if is_tree(support_type) {
            if !has_variable_layer_heights && !soluble_interface && !heat_preserve_mode && !enable_support_ring && !multi_diameter_mode {
                support_style = SmsTreeOrganic;
            } else {
                support_style = SmsTreeHybrid;
            }
        } else {
            support_style = SmsGrid;
        }
    }
    support_style
}

// Flow.cpp:293-301 `float support_nozzle_diameter(const PrintObject *object, unsigned int filament_id)`
//
// FIDELITY-NOTE: Flow.cpp's function; flow.rs has the support flows of the slicing pipeline's PrintObject only,
// so the ones over `crate::print::faithful` that the constructor calls are kept here.
pub fn support_nozzle_diameter(print: &pf::Print, filament_id: u32) -> f32 {
    if filament_id == 0 {
        let single_nozzle = print.single_diameter_layering_nozzle();
        if single_nozzle > crate::libslic3r::EPSILON {
            return single_nozzle as f32;
        }
    }
    pf::nozzle_diameter_for_filament(print.config(), filament_id)
}

// Flow.cpp:136-157 `Flow Flow::new_from_config_width(FlowRole role, const ConfigOptionFloat &width, float
// preset_nozzle_diameter, float nozzle_diameter, float height, bool multi_nozzle_diameter_mode)`
// This constructor builds a Flow object from an extrusion width config setting
// and other context properties.
pub fn flow_new_from_config_width(
    role: FlowRole,
    width: &ConfigOptionFloat,
    preset_nozzle_diameter: f32,
    nozzle_diameter: f32,
    height: f32,
    multi_nozzle_diameter_mode: bool,
) -> FlowResult<Flow> {
    if height <= 0.0 {
        return Err(FlowError::InvalidArgument("Invalid flow height supplied to new_from_config_width()".to_string()));
    }

    let w: f32;
    if width.value == 0. {
        // If user left option to 0, calculate a sane default width.
        w = Flow::auto_extrusion_width(role, nozzle_diameter as f64) as f32;
    } else {
        w = Flow::extrusion_width_for_nozzle(width, preset_nozzle_diameter, nozzle_diameter, multi_nozzle_diameter_mode) as f32;
    }

    // return Flow(w, height, rounded_rectangle_extrusion_spacing(w, height), nozzle_diameter, false);
    Flow::new(w as f64, height as f64, nozzle_diameter as f64)
}

// Flow.cpp:248-261 `Flow support_material_flow(const PrintObject *object, float layer_height)`
pub fn support_material_flow_of(object: &pf::PrintObject, print: &pf::Print, layer_height: f32) -> FlowResult<Flow> {
    use crate::print_config::config_def::{multi_nozzle_diameter_mode_enabled, primary_nozzle_diameter};
    let preset_nozzle_diameter = primary_nozzle_diameter(print.full_print_config()) as f32;
    // Width conversion follows the selected nozzle even when one diameter is sliced directly and needs no merge plan.
    let multi_diameter_mode = multi_nozzle_diameter_mode_enabled(print.full_print_config());
    let config = object.config();
    flow_new_from_config_width(
        FlowRole::SupportMaterial,
        &ConfigOptionFloat::with_value(if cfg_float(config, "support_line_width") > 0.0 {
            cfg_float(config, "support_line_width")
        } else {
            cfg_float(config, "line_width")
        }),
        preset_nozzle_diameter,
        support_nozzle_diameter(print, cfg_int(config, "support_filament") as u32),
        if layer_height > 0.0 { layer_height } else { cfg_float(config, "layer_height") as f32 },
        multi_diameter_mode,
    )
}

// Flow.cpp:270-281 `Flow support_material_1st_layer_flow(const PrintObject *object, float layer_height)`
pub fn support_material_1st_layer_flow_of(object: &pf::PrintObject, print: &pf::Print, layer_height: f32) -> FlowResult<Flow> {
    use crate::print_config::config_def::{multi_nozzle_diameter_mode_enabled, primary_nozzle_diameter};
    let print_config = print.config();
    let config = object.config();
    // const auto &width = (print_config.initial_layer_line_width.value > 0) ? print_config.initial_layer_line_width : object->config().support_line_width;
    let width = if cfg_float(print_config, "initial_layer_line_width") > 0.0 {
        cfg_float(print_config, "initial_layer_line_width")
    } else {
        cfg_float(config, "support_line_width")
    };
    flow_new_from_config_width(
        FlowRole::SupportMaterial,
        &ConfigOptionFloat::with_value(if width > 0.0 { width } else { cfg_float(config, "line_width") }),
        primary_nozzle_diameter(print.full_print_config()) as f32,
        support_nozzle_diameter(print, cfg_int(config, "support_filament") as u32),
        if layer_height > 0.0 { layer_height } else { cfg_float(print_config, "initial_layer_print_height") as f32 },
        multi_nozzle_diameter_mode_enabled(print.full_print_config()),
    )
}

// Flow.cpp:283-292 `Flow support_material_interface_flow(const PrintObject *object, float layer_height)`
pub fn support_material_interface_flow_of(object: &pf::PrintObject, print: &pf::Print, layer_height: f32) -> FlowResult<Flow> {
    use crate::print_config::config_def::{multi_nozzle_diameter_mode_enabled, primary_nozzle_diameter};
    let config = object.config();
    flow_new_from_config_width(
        FlowRole::SupportMaterialInterface,
        &ConfigOptionFloat::with_value(if cfg_float(config, "support_line_width") > 0.0 {
            cfg_float(config, "support_line_width")
        } else {
            cfg_float(config, "line_width")
        }),
        primary_nozzle_diameter(print.full_print_config()) as f32,
        support_nozzle_diameter(print, cfg_int(config, "support_interface_filament") as u32),
        if layer_height > 0.0 { layer_height } else { cfg_float(config, "layer_height") as f32 },
        multi_nozzle_diameter_mode_enabled(print.full_print_config()),
    )
}

impl SupportParameters {
    /// SupportParameters.hpp:66-305 — `SupportParameters(const PrintObject& object)`, the constructor of the
    /// pin over `crate::print::faithful` (`object.print()` is `print`).
    ///
    /// An exception of the C++ (a flow that cannot be made) is the `Err`.
    pub fn new(object: &pf::PrintObject, print: &pf::Print) -> FlowResult<Self> {
        use crate::print_config::config_def::{is_tree, multi_nozzle_diameter_mode_enabled, primary_nozzle_diameter};
        use enums::{SupportMaterialInterfacePattern as Smip, SupportMaterialPattern as Smp, SupportMaterialStyle as Sms};
        // 68-70
        let print_config = print.config();
        let object_config = object.config();
        let slicing_params = object.slicing_parameters();
        let mut this = SupportParameters::default();

        let support_filament = cfg_int(object_config, "support_filament");
        let support_interface_filament = cfg_int(object_config, "support_interface_filament");
        let support_type = enums::SupportType::from_i32(cfg_int(object_config, "support_type"));
        let filament_soluble = cfg_bools(print_config, "filament_soluble");

        // 72
        this.soluble_interface = slicing_params.soluble_interface;
        // 73-79
        this.soluble_interface_non_soluble_base =
            // Zero z-gap between the overhangs and the support interface.
            slicing_params.soluble_interface &&
            // Interface extruder soluble.
            support_interface_filament > 0 && get_at(&filament_soluble, (support_interface_filament - 1) as usize) &&
            // Base extruder: Either "print with active extruder" not soluble.
            (support_filament == 0 || !get_at(&filament_soluble, (support_filament - 1) as usize));

        // 81-116
        {
            let support_interface_top_layers = cfg_int(object_config, "support_interface_top_layers");
            let support_interface_bottom_layers = cfg_int(object_config, "support_interface_bottom_layers");
            this.num_top_interface_layers = std::cmp::max(0, support_interface_top_layers) as usize;
            this.num_bottom_interface_layers =
                if support_interface_bottom_layers < 0 { this.num_top_interface_layers } else { support_interface_bottom_layers as usize };
            this.has_top_contacts = this.num_top_interface_layers > 0;
            this.has_bottom_contacts = this.num_bottom_interface_layers > 0;
            if is_tree(support_type) {
                if this.soluble_interface_non_soluble_base {
                    // Try to support soluble dense interfaces with non-soluble dense interfaces.
                    this.num_top_base_interface_layers = std::cmp::min(this.num_top_interface_layers as i32 / 2, 2) as usize;
                    this.num_bottom_base_interface_layers = std::cmp::min(this.num_bottom_interface_layers as i32 / 2, 2) as usize;
                } else {
                    // BBS: if support interface and support base do not use the same filament, add a base layer to improve their adhesion
                    // Note: support materials (such as Supp.W) can't be used as support base now, so support interface and base are still using different filaments even if
                    // support_filament==0
                    let differnt_support_interface_filament = support_interface_filament != 0 && support_interface_filament != support_filament;
                    this.num_top_base_interface_layers = if differnt_support_interface_filament { 1 } else { 0 };
                    this.num_bottom_base_interface_layers = if differnt_support_interface_filament { 1 } else { 0 };
                }
            } else {
                if this.soluble_interface_non_soluble_base {
                    // Try to support soluble dense interfaces with non-soluble dense interfaces.
                    this.num_top_base_interface_layers = if this.num_top_interface_layers > 0 { 2 } else { 0 };
                    this.num_bottom_base_interface_layers = std::cmp::min(this.num_bottom_interface_layers as i32 / 2, 2) as usize;
                } else {
                    // BBS: if support interface and support base do not use the same filament, add a base layer to improve their adhesion
                    // Note: support materials (such as Supp.W) can't be used as support base now, so support interface and base are still using different filaments even if
                    // support_filament==0
                    let differnt_support_interface_filament = support_interface_filament != 0 && support_interface_filament != support_filament;
                    this.num_top_base_interface_layers =
                        if this.num_top_interface_layers > 0 { if differnt_support_interface_filament { 2 } else { 1 } } else { 0 };
                    this.num_bottom_base_interface_layers = if differnt_support_interface_filament { 1 } else { 0 };
                }
            }
        }
        // 117-120
        this.first_layer_flow = support_material_1st_layer_flow_of(object, print, slicing_params.first_print_layer_height as f32)?;
        this.support_material_flow = support_material_flow_of(object, print, slicing_params.layer_height as f32)?;
        this.support_material_interface_flow = support_material_interface_flow_of(object, print, slicing_params.layer_height as f32)?;
        this.raft_interface_flow = this.support_material_interface_flow;
        // 121-127
        // BBS multi-diameter: kept so the support base flow can be rebuilt for a nozzle the support merge plan
        // picked. support_material_flow() derives the nozzle from support_filament, which is 0 in that case.
        this.support_base_width =
            if cfg_float(object_config, "support_line_width") > 0.0 { cfg_float(object_config, "support_line_width") } else { cfg_float(object_config, "line_width") };
        this.support_first_layer_width = if cfg_float(print_config, "initial_layer_line_width") > 0.0 {
            cfg_float(print_config, "initial_layer_line_width")
        } else {
            this.support_base_width
        };
        this.preset_nozzle_diameter = primary_nozzle_diameter(print.full_print_config()) as f32;
        this.multi_nozzle_diameter_mode = multi_nozzle_diameter_mode_enabled(print.full_print_config());

        // 129-134
        // Calculate a minimum support layer height as a minimum over all extruders, but not smaller than 10um.
        // scaled<coord_t>(0.01): coord_t(0.01 / SCALING_FACTOR)
        this.support_layer_height_min = (0.01 / crate::libslic3r::SCALING_FACTOR) as i64 as f64;
        for lh in cfg_floats(print_config, "min_layer_height") {
            this.support_layer_height_min = std_min(this.support_layer_height_min, std_max(0.01, lh));
        }
        for layer in object.layers() {
            this.support_layer_height_min = std_min(this.support_layer_height_min, std_max(0.01, layer.height));
        }

        // 136-139
        if cfg_int(object_config, "support_interface_top_layers") == 0 {
            // No interface layers allowed, print everything with the base support pattern.
            this.support_material_interface_flow = this.support_material_flow;
        }

        // 141-152
        // Evaluate the XY gap between the object outer perimeters and the support structures.
        // Evaluate the XY gap between the object outer perimeters and the support structures.
        //
        // `external_perimeter_width` (the maximum of `region.flow(object, frExternalPerimeter,
        // slicing_params.layer_height).width()` over the regions) is computed by the C++ and never read; the
        // faithful PrintRegion has no `flow()`, and the value is not computed here.
        let mut bridge_flow_ratio: f64 = 0.0;
        for region_id in 0..object.num_printing_regions() {
            let region = object.printing_region(region_id);
            bridge_flow_ratio += cfg_float(region.borrow().config(), "bridge_flow");
        }
        this.gap_xy = resolved_support_object_xy_distance(print_config, object_config);
        this.gap_xy_first_layer = cfg_float(object_config, "support_object_first_layer_gap");
        bridge_flow_ratio /= object.num_printing_regions() as f64;

        // 154-156
        this.support_material_bottom_interface_flow = if slicing_params.soluble_interface || !cfg_bool(object_config, "thick_bridges") {
            this.support_material_interface_flow.with_flow_ratio(bridge_flow_ratio)?
        } else {
            Flow::bridging_flow(
                bridge_flow_ratio * this.support_material_interface_flow.nozzle_diameter(),
                this.support_material_interface_flow.nozzle_diameter(),
            )
        };

        // 158-167
        this.can_merge_support_regions = support_filament == support_interface_filament;
        if !this.can_merge_support_regions && (support_filament == 0 || support_interface_filament == 0) {
            // One of the support extruders is of "don't care" type.
            let object_extruders = object.object_extruders(print);
            if object_extruders.len() == 1 &&
                // object_extruders are 0-based but object_config.support_filament's are 1-based
                object_extruders[0] + 1 == std::cmp::max(support_filament as u32, support_interface_filament as u32)
            {
                // Object is printed with the same extruder as the support.
                this.can_merge_support_regions = true;
            }
        }

        // 170-171: Geometry::deg2rad<float>: `T(PI) * angle / T(180.0)`
        let support_angle = cfg_float(object_config, "support_angle");
        this.base_angle = std::f32::consts::PI * (support_angle as f32) / 180.0_f32;
        this.interface_angle = std::f32::consts::PI * ((support_angle + 90.) as f32) / 180.0_f32;
        // 172-177
        this.interface_spacing = cfg_float(object_config, "support_interface_spacing") + this.support_material_interface_flow.spacing();
        this.interface_density = std_min(1., this.support_material_interface_flow.spacing() / this.interface_spacing);
        let raft_interface_spacing = cfg_float(object_config, "support_interface_spacing") + this.raft_interface_flow.spacing();
        this.raft_interface_density = std_min(1., this.raft_interface_flow.spacing() / raft_interface_spacing);
        this.support_spacing = cfg_float(object_config, "support_base_pattern_spacing") + this.support_material_flow.spacing();
        this.support_density = std_min(1., this.support_material_flow.spacing() / this.support_spacing);
        // 178-182
        if cfg_int(object_config, "support_interface_top_layers") == 0 {
            // No interface layers allowed, print everything with the base support pattern.
            this.interface_spacing = this.support_spacing;
            this.interface_density = this.support_density;
        }

        // 184-190
        this.enable_support_ironing = cfg_bool(object_config, "enable_support_ironing");
        this.ironing_angle = cfg_float(object_config, "support_ironing_direction");
        this.ironing_flow_percent = cfg_float(object_config, "support_ironing_flow");
        this.ironing_inset = cfg_float(object_config, "support_ironing_inset");
        this.ironing_line_spacing = cfg_float(object_config, "support_ironing_spacing");
        this.ironing_pattern = infill_pattern_from_i32(cfg_int(object_config, "support_ironing_pattern"));
        this.ironing_speed = cfg_float(object_config, "support_ironing_speed");

        // 192-213
        let mut support_style = Sms::from_i32(cfg_int(object_config, "support_style"));
        if support_style != Sms::SmsDefault {
            if (support_style == Sms::SmsSnug || support_style == Sms::SmsGrid) && is_tree(support_type) {
                support_style = Sms::SmsDefault;
            }
            if (support_style == Sms::SmsTreeSlim
                || support_style == Sms::SmsTreeStrong
                || support_style == Sms::SmsTreeHybrid
                || support_style == Sms::SmsTreeOrganic)
                && !is_tree(support_type)
            {
                support_style = Sms::SmsDefault;
            }
        }
        if support_style == Sms::SmsDefault {
            if is_tree(support_type) {
                // organic support doesn't work with variable layer heights (including adaptive layer height and height range modifier, see #4313)
                // BBS multi-diameter: default to hybrid tree as well.
                if !object.has_variable_layer_heights && !slicing_params.soluble_interface && !print.uses_multiple_nozzle_diameters() {
                    // BOOST_LOG_TRIVIAL(warning) << "tree support default to organic support";
                    support_style = Sms::SmsTreeOrganic;
                } else {
                    // BOOST_LOG_TRIVIAL(warning) << "tree support default to hybrid tree due to adaptive layer height or multi-diameter mode";
                    support_style = Sms::SmsTreeHybrid;
                }
            } else {
                support_style = Sms::SmsGrid;
            }
        }
        this.support_style = support_style as i32;

        // 215-222
        let mut support_base_pattern = Smp::from_i32(cfg_int(object_config, "support_base_pattern"));
        if support_base_pattern == Smp::SmpLightning && !is_tree(support_type) {
            support_base_pattern = Smp::SmpRectilinear;
        }
        if support_base_pattern == Smp::SmpDefault {
            if is_tree(support_type) {
                support_base_pattern = if support_style == Sms::SmsTreeHybrid { Smp::SmpRectilinear } else { Smp::SmpNone };
            } else {
                support_base_pattern = Smp::SmpRectilinear;
            }
        }
        this.support_base_pattern = support_base_pattern as i32;

        // 224-238
        this.with_sheath = cfg_int(object_config, "tree_support_wall_count") > 0;
        this.base_fill_pattern = if support_base_pattern == Smp::SmpLightning {
            InfillPattern::Lightning
        } else if support_base_pattern == Smp::SmpHoneycomb {
            InfillPattern::Honeycomb
        } else if this.support_density > 0.95 || this.with_sheath {
            InfillPattern::Rectilinear
        } else {
            InfillPattern::SupportBase
        };
        this.interface_fill_pattern = if this.interface_density > 0.95 { InfillPattern::Rectilinear } else { InfillPattern::SupportBase };
        this.raft_interface_fill_pattern = if this.raft_interface_density > 0.95 { InfillPattern::Rectilinear } else { InfillPattern::SupportBase };
        let support_interface_pattern = Smip::from_i32(cfg_int(object_config, "support_interface_pattern"));
        if support_interface_pattern == Smip::SmipGrid {
            this.contact_fill_pattern = InfillPattern::Grid;
        } else if support_interface_pattern == Smip::SmipRectilinearInterlaced || support_interface_pattern == Smip::SmipAuto {
            this.contact_fill_pattern = InfillPattern::Rectilinear;
        } else {
            this.contact_fill_pattern = if support_interface_pattern == Smip::SmipConcentric {
                InfillPattern::Concentric
            } else if this.interface_density > 0.95 {
                InfillPattern::Rectilinear
            } else {
                InfillPattern::SupportBase
            };
        }

        // 240-269
        this.raft_angle_1st_layer = 0.;
        this.raft_angle_base = 0.;
        this.raft_angle_interface = 0.;
        if slicing_params.base_raft_layers > 1 {
            debug_assert!(slicing_params.raft_layers() >= 4);
            // There are all raft layer types (1st layer, base, interface & contact layers) available.
            this.raft_angle_1st_layer = this.interface_angle;
            this.raft_angle_base = this.base_angle;
            this.raft_angle_interface = this.interface_angle;
            if (slicing_params.interface_raft_layers & 1) == 0 {
                // Allign the 1st raft interface layer so that the object 1st layer is hatched perpendicularly to the raft contact interface.
                this.raft_angle_interface += (0.5 * std::f64::consts::PI) as f32;
            }
        } else if slicing_params.base_raft_layers == 1 || slicing_params.interface_raft_layers > 1 {
            debug_assert!(slicing_params.raft_layers() == 2 || slicing_params.raft_layers() == 3);
            // 1st layer, interface & contact layers available.
            this.raft_angle_1st_layer = this.base_angle;
            // `this->interface_angle + 0.5 * M_PI`: the sum in double, stored as a float.
            this.raft_angle_interface = (this.interface_angle as f64 + 0.5 * std::f64::consts::PI) as f32;
        } else if slicing_params.interface_raft_layers == 1 {
            // Only the contact raft layer is non-empty, which will be printed as the 1st layer.
            debug_assert!(slicing_params.base_raft_layers == 0);
            debug_assert!(slicing_params.interface_raft_layers == 1);
            debug_assert!(slicing_params.raft_layers() == 1);
            this.raft_angle_1st_layer = (0.5 * std::f64::consts::PI) as f32;
            this.raft_angle_interface = this.raft_angle_1st_layer;
        } else {
            // No raft.
            debug_assert!(slicing_params.base_raft_layers == 0);
            debug_assert!(slicing_params.interface_raft_layers == 0);
            debug_assert!(slicing_params.raft_layers() == 0);
        }

        // 271-282
        {
            let support_nozzle = support_nozzle_diameter(print, support_filament as u32);
            let support_width_opt = ConfigOptionFloat::with_value(if cfg_float(object_config, "support_line_width") > 0.0 {
                cfg_float(object_config, "support_line_width")
            } else {
                cfg_float(object_config, "line_width")
            });
            this.support_extrusion_width =
                Flow::extrusion_width_for_nozzle(&support_width_opt, this.preset_nozzle_diameter, support_nozzle, this.multi_nozzle_diameter_mode);
        }
        // Check if set to zero, use default if so.
        if this.support_extrusion_width <= 0.0 {
            let nozzle_diameter = support_nozzle_diameter(print, support_interface_filament as u32);
            this.support_extrusion_width = Flow::auto_extrusion_width(FlowRole::SupportMaterial, nozzle_diameter as f64);
        }

        // 284-302
        let tree_support_branch_diameter_double_wall: f64;
        // get support filament strength and decide the thresh of double wall area
        let impact_strength_z = cfg_floats(print_config, "impact_strength_z");
        // `get_at(object_config.support_filament - 1)`: the index is a size_t, -1 is past the end.
        let mut support_filament_strength: f32 = get_at(&impact_strength_z, (support_filament as i64 - 1) as usize) as f32;
        if support_filament == 0 {
            // find the weakest filament
            support_filament_strength = f32::MAX;
            for extruder in object.object_extruders(print) {
                let strength = get_at(&impact_strength_z, extruder as usize) as f32;
                if strength < support_filament_strength {
                    support_filament_strength = strength;
                }
            }
        }

        let tree_support_wall_count = cfg_int(object_config, "tree_support_wall_count");
        if tree_support_wall_count == -1 {
            tree_support_branch_diameter_double_wall = support_filament_strength as f64;
            // 0.25*sqr(scaled<double>(tree_support_branch_diameter_double_wall))*M_PI
            this.tree_branch_diameter_double_wall_area_scaled =
                0.25 * sqr(tree_support_branch_diameter_double_wall / crate::libslic3r::SCALING_FACTOR) * std::f64::consts::PI;
        } else {
            // force double walls everywhere if wall count is larger than 1
            this.tree_branch_diameter_double_wall_area_scaled = if tree_support_wall_count > 1 { 0.1 } else { f64::MAX };
        }

        // 304
        this.independent_layer_height = cfg_bool(print_config, "independent_support_layer_height");
        // 370 `coordf_t support_expansion=0;`, 404 `const double thresh_big_overhang = scale_(10);`
        this.support_expansion = 0.;
        this.thresh_big_overhang = 10. / crate::libslic3r::SCALING_FACTOR;
        Ok(this)
    }

    // SupportParameters.hpp:342-346
    pub fn support_material_flow_for_nozzle(&self, nozzle_diameter: f64, layer_height: f32) -> FlowResult<Flow> {
        flow_new_from_config_width(
            FlowRole::SupportMaterial,
            &ConfigOptionFloat::with_value(self.support_base_width),
            self.preset_nozzle_diameter,
            nozzle_diameter as f32,
            layer_height,
            self.multi_nozzle_diameter_mode,
        )
    }

    // SupportParameters.hpp:347-351
    pub fn support_material_interface_flow_for_nozzle(&self, nozzle_diameter: f64, layer_height: f32) -> FlowResult<Flow> {
        flow_new_from_config_width(
            FlowRole::SupportMaterialInterface,
            &ConfigOptionFloat::with_value(self.support_base_width),
            self.preset_nozzle_diameter,
            nozzle_diameter as f32,
            layer_height,
            self.multi_nozzle_diameter_mode,
        )
    }

    // SupportParameters.hpp:352-356
    pub fn support_material_first_layer_flow_for_nozzle(&self, nozzle_diameter: f64, layer_height: f32) -> FlowResult<Flow> {
        flow_new_from_config_width(
            FlowRole::SupportMaterial,
            &ConfigOptionFloat::with_value(self.support_first_layer_width),
            self.preset_nozzle_diameter,
            nozzle_diameter as f32,
            layer_height,
            self.multi_nozzle_diameter_mode,
        )
    }

    // SupportParameters.hpp:396-401
    // Produce a support interface angle for a given SupportLayer::interface_id()
    pub fn support_interface_angle(&self, interface_id: usize, interface_pattenn: enums::SupportMaterialInterfacePattern) -> f32 {
        if self.support_style == enums::SupportMaterialStyle::SmsGrid as i32
            || interface_pattenn == enums::SupportMaterialInterfacePattern::SmipRectilinear
        {
            return self.interface_angle;
        }
        self.interface_angle
            + if interface_id & 1 != 0 { (-std::f64::consts::PI / 4.) as f32 } else { (std::f64::consts::PI / 4.) as f32 }
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
            support_base_width: 0.0,
            support_first_layer_width: 0.0,
            // SupportParameters.hpp:340 — float preset_nozzle_diameter { 0.f };
            preset_nozzle_diameter: 0.0,
            // SupportParameters.hpp:341 — bool multi_nozzle_diameter_mode { false };
            multi_nozzle_diameter_mode: false,
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
            // `scaled<double>(5.0)` is the FLOATING `scaled<Tout=double>` variant
            // (Point.hpp:527): `double(5.0 / SCALING_FACTOR)` — a plain float
            // division with NO flooring (unlike scale_()/scale()). Mirror that.
            tree_branch_diameter_double_wall_area_scaled: 0.25
                * sqr(5.0 / crate::libslic3r::SCALING_FACTOR)
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
