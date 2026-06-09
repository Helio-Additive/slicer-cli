//! Support generation parameters.
//!
//! C++ Reference:
//! - Support/SupportParameters.hpp (header-only: struct SupportParameters)
//!
//! This is a faithful 1:1 line-by-line port of `Slic3r::SupportParameters`.
//!
//! PORTING STATUS: partial.
//! The data members and the const accessor methods are ported faithfully.
//! The constructor `SupportParameters(const PrintObject&)`
//! (SupportParameters.hpp:15-242) is BLOCKED and intentionally not ported:
//! it reads from config surfaces that are not threaded through the current
//! Rust `PrintObject`. See the doc comment on [`SupportParameters::from_print_object`]
//! for the exhaustive list of missing dependencies. Once those are available,
//! the body can be filled in by following the C++ line refs preserved below.

use crate::flow::Flow;
use crate::libslic3r::scale;
use crate::print_config::InfillPattern;

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
    /// BLOCKED — not ported. The C++ constructor body reads from config
    /// surfaces that are not threaded through the current Rust `PrintObject`:
    ///
    /// 1. `object.print()->config()` (the global `PrintConfig`): the Rust
    ///    `PrintObject` has no `print()` link and no `PrintConfig` accessor.
    ///    Needed for `filament_soluble.get_at(..)`, `nozzle_diameter.get_at(..)`,
    ///    `min_layer_height.values`, `impact_strength_z.get_at(..)`,
    ///    `top_z_overrides_xy_distance`, `independent_support_layer_height`.
    /// 2. The Rust `PrintConfig` is flattened: `nozzle_diameter: f64` and
    ///    `filament_soluble: bool` are not per-extruder/per-filament vectors
    ///    with `.get_at(idx - 1)`, and `impact_strength_z` does not exist.
    /// 3. `object.slicing_parameters()` — no accessor (the `slicing_params`
    ///    field on the Rust `PrintObject` is private).
    /// 4. `object.object_extruders()` — does not exist on the Rust `PrintObject`.
    /// 5. `object.has_variable_layer_heights` — does not exist.
    /// 6. The `SupportMaterialStyle` / `SupportMaterialPattern` /
    ///    `SupportMaterialInterfacePattern` enums and the `is_tree()` helper
    ///    are not ported into the Rust `print_config` module.
    ///
    /// Until those are available, attempting to construct this faithfully
    /// would require fabricated config plumbing, which would break G-code
    /// parity. The C++ line refs are preserved on each field above so the
    /// body can be filled in line-by-line once the dependencies land.
    pub fn from_print_object() {
        unimplemented!(
            "SupportParameters(const PrintObject&) is blocked: requires \
             object.print()->config() (global PrintConfig), \
             object.slicing_parameters(), object.object_extruders(), \
             object.has_variable_layer_heights, and the \
             SupportMaterialStyle/Pattern enums to be threaded through the \
             Rust PrintObject. See SupportParameters.hpp:15-242."
        )
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
    /// Mirror of the C++ in-class member initializers (the only values fixed
    /// at construction time independent of the deleted-then-`PrintObject`
    /// constructor). All other fields default to zero/false; they are
    /// populated by the (currently blocked) `from_print_object` constructor.
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

/// C++ `smsDefault` (SupportMaterialStyle, PrintConfig.hpp). Raw enum value 0.
const SMS_DEFAULT: SupportMaterialStyle = 0;
/// C++ `smpDefault` (SupportMaterialPattern, PrintConfig.hpp). Raw enum value 0.
const SMP_DEFAULT: SupportMaterialPattern = 0;

/// libslic3r.h — `template<typename T> inline T sqr(T x) { return x * x; }`
#[inline]
fn sqr(x: f64) -> f64 {
    x * x
}

// SupportParameters.hpp:326 — } // namespace Slic3r
