// Faithful 1:1 port of libslic3r/Preset.cpp (+ Preset.hpp) from BambuStudio.
//
// SCOPE NOTE: The bulk of Preset.cpp is built on top of `DynamicPrintConfig`
// (a dynamic key->ConfigOption map with .option()/.has()/.keys()/.erase()/
// .diff()/.apply()/.load_from_json(), typed ConfigOptionFloats/Strings/
// VectorBase, coStrings, etc.), `boost::property_tree`, `boost::filesystem`,
// `nlohmann::json`, `AppConfig`, `PresetBundle`, `print_config_def` and
// `FullPrintConfig::defaults()`. None of that dynamic-config infrastructure
// exists yet in the Rust crate (the Rust `PrintConfig` in print_config.rs is a
// static hand-rolled struct, not a dynamic key->option config). Every function
// that touches those is therefore BLOCKED and listed in the porter report; it
// is NOT stubbed here.
//
// This module ports the self-contained, pure pieces faithfully, line by line:
// the enums, the VendorProfile / PrinterModel / PrinterVariant structs and
// their pure methods, the Preset struct + its pure static helpers, the static
// option-name vectors, and the pure PhysicalPrinter helpers.

use crate::config::PrinterTechnology;
use crate::semver::Semver;
use std::collections::BTreeMap;
use std::sync::RwLock;

// ============================================================================
// Preset.hpp constants (Preset.hpp:18-28)
// ============================================================================

//BBS: change system directories
pub const PRESET_SYSTEM_DIR: &str = "system"; // Preset.hpp:18
pub const PRESET_USER_DIR: &str = "user"; // Preset.hpp:19
pub const PRESET_FILAMENT_NAME: &str = "filament"; // Preset.hpp:20
pub const PRESET_PRINT_NAME: &str = "process"; // Preset.hpp:21
pub const PRESET_PRINTER_NAME: &str = "machine"; // Preset.hpp:22
pub const PRESET_SLA_PRINT_NAME: &str = "sla_print"; // Preset.hpp:23
pub const PRESET_SLA_MATERIALS_NAME: &str = "sla_materials"; // Preset.hpp:24
pub const PRESET_PROFILES_DIR: &str = "profiles"; // Preset.hpp:25
pub const PRESET_PROFILES_TEMOLATE_DIR: &str = "profiles_template"; // Preset.hpp:26
pub const PRESET_TEMPLATE_DIR: &str = "Template"; // Preset.hpp:27
pub const PRESET_CUSTOM_VENDOR: &str = "Custom"; // Preset.hpp:28

//BBS: iot preset type strings
pub const PRESET_IOT_PRINTER_TYPE: &str = "printer"; // Preset.hpp:31
pub const PRESET_IOT_FILAMENT_TYPE: &str = "filament"; // Preset.hpp:32
pub const PRESET_IOT_PRINT_TYPE: &str = "print"; // Preset.hpp:33

//BBS: add json support
pub const BBL_JSON_KEY_VERSION: &str = "version"; // Preset.hpp:37

// ============================================================================
// ConfigFileType (Preset.hpp:86-92)
// ============================================================================

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ConfigFileType {
    ConfigFileTypeUnknown,
    ConfigFileTypeAppConfig,
    ConfigFileTypeConfig,
    ConfigFileTypeConfigBundle,
}

// ============================================================================
// VendorProfile (Preset.hpp:103-172)
// ============================================================================

// Preset.hpp:112-116
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PrinterVariant {
    pub name: String,
}

impl PrinterVariant {
    // Preset.hpp:113
    pub fn new() -> Self {
        Self::default()
    }
    // Preset.hpp:114
    pub fn with_name(name: &str) -> Self {
        Self {
            name: name.to_string(),
        }
    }
}

// Preset.hpp:118-151
#[derive(Clone, Debug)]
pub struct PrinterModel {
    pub id: String,
    pub name: String,
    //BBS: this is internal id for the printer. Currently only used for searching in database
    pub model_id: String,
    pub technology: PrinterTechnology,
    pub family: String,
    pub variants: Vec<PrinterVariant>,
    pub default_materials: Vec<String>,
    pub not_support_bed_types: Vec<String>,
    // Vendor & Printer Model specific print bed model & texture.
    pub bed_model: String,
    pub bed_texture: String,
    pub use_rect_grid: String,
    pub image_bed_type: String,
    pub default_bed_type: String,
    pub bottom_texture_end_name: String,
    pub use_double_extruder_default_texture: String,
    pub bottom_texture_rect: String,
    pub bottom_texture_rect_longer: String,
    pub middle_texture_rect: String,
    pub right_icon_offset_bed: String,
    pub hotend_model: String,
}

impl Default for PrinterModel {
    // Preset.hpp:119  PrinterModel() {}
    fn default() -> Self {
        Self {
            id: String::new(),
            name: String::new(),
            model_id: String::new(),
            technology: PrinterTechnology::PtFFF,
            family: String::new(),
            variants: Vec::new(),
            default_materials: Vec::new(),
            not_support_bed_types: Vec::new(),
            bed_model: String::new(),
            bed_texture: String::new(),
            use_rect_grid: String::new(),
            image_bed_type: String::new(),
            default_bed_type: String::new(),
            bottom_texture_end_name: String::new(),
            use_double_extruder_default_texture: String::new(),
            bottom_texture_rect: String::new(),
            bottom_texture_rect_longer: String::new(),
            middle_texture_rect: String::new(),
            right_icon_offset_bed: String::new(),
            hotend_model: String::new(),
        }
    }
}

impl PrinterModel {
    // Preset.hpp:142-147
    pub fn variant_mut(&mut self, name: &str) -> Option<&mut PrinterVariant> {
        for v in self.variants.iter_mut() {
            if v.name == name {
                return Some(v);
            }
        }
        None
    }

    // Preset.hpp:149
    pub fn variant(&self, name: &str) -> Option<&PrinterVariant> {
        self.variants.iter().find(|v| v.name == name)
    }

    // Preset.cpp:213  std::map<std::string, std::string> VendorProfile::PrinterModel::get_bed_texture_maps() const
    pub fn get_bed_texture_maps(&self) -> BTreeMap<String, String> {
        let mut maps: BTreeMap<String, String> = BTreeMap::new();
        // Preset.cpp:216
        if !self.use_double_extruder_default_texture.is_empty() {
            maps.insert(
                "use_double_extruder_default_texture".to_string(),
                self.use_double_extruder_default_texture.clone(),
            );
        }
        // Preset.cpp:217
        if !self.bottom_texture_end_name.is_empty() {
            maps.insert(
                "bottom_texture_end_name".to_string(),
                self.bottom_texture_end_name.clone(),
            );
        }
        // Preset.cpp:218
        if !self.bottom_texture_rect.is_empty() {
            maps.insert(
                "bottom_texture_rect".to_string(),
                self.bottom_texture_rect.clone(),
            );
        }
        // Preset.cpp:219
        if !self.bottom_texture_rect_longer.is_empty() {
            maps.insert(
                "bottom_texture_rect_longer".to_string(),
                self.bottom_texture_rect_longer.clone(),
            );
        }
        // Preset.cpp:220
        if !self.middle_texture_rect.is_empty() {
            maps.insert(
                "middle_texture_rect".to_string(),
                self.middle_texture_rect.clone(),
            );
        }
        maps
    }
}

// Preset.hpp:103-172
#[derive(Clone, Debug, Default)]
pub struct VendorProfile {
    pub name: String,
    pub id: String,
    pub config_version: Semver,
    pub config_update_url: String,
    pub changelog_url: String,
    pub models: Vec<PrinterModel>,
    pub default_filaments: std::collections::BTreeSet<String>,
    pub default_sla_materials: std::collections::BTreeSet<String>,
}

impl VendorProfile {
    // Preset.hpp:157  VendorProfile() {}
    pub fn new() -> Self {
        Self::default()
    }

    // Preset.hpp:158  VendorProfile(std::string id) : id(std::move(id)) {}
    pub fn with_id(id: String) -> Self {
        Self {
            id,
            ..Self::default()
        }
    }

    // Preset.hpp:160  bool valid() const { return ! name.empty() && ! id.empty() && config_version.valid(); }
    pub fn valid(&self) -> bool {
        !self.name.is_empty() && !self.id.is_empty() && self.config_version.valid()
    }

    // Preset.hpp:167  size_t num_variants() const { size_t n = 0; for (auto &model : models) n += model.variants.size(); return n; }
    pub fn num_variants(&self) -> usize {
        let mut n: usize = 0;
        for model in &self.models {
            n += model.variants.len();
        }
        n
    }

    // Preset.cpp:351  std::vector<std::string> VendorProfile::families() const
    pub fn families(&self) -> Vec<String> {
        let mut res: Vec<String> = Vec::new();
        let mut _num_familiies: u32 = 0;

        for model in &self.models {
            if !res.iter().any(|f| *f == model.family) {
                res.push(model.family.clone());
                _num_familiies += 1;
            }
        }

        res
    }
}

// Preset.hpp:170  bool operator< (const VendorProfile &rhs) const { return this->id <  rhs.id; }
// Preset.hpp:171  bool operator==(const VendorProfile &rhs) const { return this->id == rhs.id; }
impl PartialEq for VendorProfile {
    fn eq(&self, rhs: &Self) -> bool {
        self.id == rhs.id
    }
}
impl Eq for VendorProfile {}
impl PartialOrd for VendorProfile {
    fn partial_cmp(&self, rhs: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(rhs))
    }
}
impl Ord for VendorProfile {
    fn cmp(&self, rhs: &Self) -> std::cmp::Ordering {
        self.id.cmp(&rhs.id)
    }
}

// ============================================================================
// Preset (Preset.hpp:191-383)
// ============================================================================

// Preset.hpp:194-210
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Type {
    TypeInvalid,
    TypePrint,
    TypeSlaPrint,
    TypeFilament,
    TypeSlaMaterial,
    TypePrinter,
    TypeCount,
    // This type is here to support PresetConfigSubstitutions for physical printers, however it does not belong to the Preset class,
    // PhysicalPrinter class is used instead.
    TypePhysicalPrinter,
    // BBS: plate config
    TypePlate,
    // BBS: model config
    TypeModel,
}

// Suffix to be added to a modified preset name in the combo box.
// Preset.cpp:367  static std::string g_suffix_modified = " (modified)";
static G_SUFFIX_MODIFIED: RwLock<Option<String>> = RwLock::new(None);

fn g_suffix_modified_get() -> String {
    let guard = G_SUFFIX_MODIFIED.read().unwrap();
    match guard.as_ref() {
        Some(s) => s.clone(),
        None => " (modified)".to_string(),
    }
}

impl Type {
    // Preset.cpp:443  std::string Preset::get_type_string(Preset::Type type)
    pub fn get_type_string(ty: Type) -> String {
        match ty {
            Type::TypeFilament => PRESET_FILAMENT_NAME.to_string(),
            Type::TypePrint => PRESET_PRINT_NAME.to_string(),
            Type::TypePrinter => PRESET_PRINTER_NAME.to_string(),
            Type::TypePhysicalPrinter => "physical_printer".to_string(),
            Type::TypeInvalid => "invalid".to_string(),
            _ => "invalid".to_string(),
        }
    }

    // Preset.cpp:461  std::string Preset::get_iot_type_string(Preset::Type type)
    pub fn get_iot_type_string(ty: Type) -> String {
        match ty {
            Type::TypeFilament => PRESET_IOT_FILAMENT_TYPE.to_string(),
            Type::TypePrint => PRESET_IOT_PRINT_TYPE.to_string(),
            Type::TypePrinter => PRESET_IOT_PRINTER_TYPE.to_string(),
            _ => "invalid".to_string(),
        }
    }

    //make the type string compatibility with local and iot type string
    // Preset.cpp:477  Preset::Type Preset::get_type_from_string(std::string type_str)
    pub fn get_type_from_string(type_str: &str) -> Type {
        if type_str == PRESET_PRINT_NAME || type_str == PRESET_IOT_PRINT_TYPE {
            Type::TypePrint
        } else if type_str == PRESET_FILAMENT_NAME || type_str == PRESET_IOT_FILAMENT_TYPE {
            Type::TypeFilament
        } else if type_str == PRESET_PRINTER_NAME || type_str == PRESET_IOT_PRINTER_TYPE {
            Type::TypePrinter
        } else {
            Type::TypeInvalid
        }
    }
}

// Preset.hpp:191-383
//
// NOTE: `config: DynamicPrintConfig` and `vendor: const VendorProfile*` are
// represented faithfully in spirit but the dynamic config type is not yet
// ported; config-dependent methods are blocked (see module header). Fields are
// kept 1:1 with the C++ declaration order and defaults.
#[derive(Clone, Debug)]
pub struct Preset {
    // Preset.hpp:212
    pub r#type: Type,
    // Preset.hpp:216
    pub is_default: bool,
    // Preset.hpp:219
    pub is_external: bool,
    // Preset.hpp:221
    pub is_system: bool,
    // Preset.hpp:225
    pub is_visible: bool,
    // Preset.hpp:227
    pub is_dirty: bool,
    // Preset.hpp:229
    pub is_compatible: bool,
    //BBS: add type for project-embedded
    // Preset.hpp:232
    pub is_project_embedded: bool,
    // Preset.hpp:238  Name of the preset, usually derived form the file name.
    pub name: String,
    // Preset.hpp:242  File name of the preset.
    pub file: String,
    // Preset.hpp:247  Has this profile been loaded?
    pub loaded: bool,
    // Preset.hpp:253  Alias of the preset
    pub alias: String,
    // Preset.hpp:257  List of profile names, from which this profile was renamed at some point of time.
    pub renamed_from: Vec<String>,
    //BBS
    // Preset.hpp:260  version of preset
    pub version: Semver,
    // Preset.hpp:261  ini string of preset
    pub ini_str: String,
    // Preset.hpp:262  setting id in cloud database
    pub setting_id: String,
    // Preset.hpp:263  setting id in cloud database
    pub filament_id: String,
    // Preset.hpp:264  preset user_id
    pub user_id: String,
    // Preset.hpp:265  base id of preset
    pub base_id: String,
    // Preset.hpp:266  enum: "delete", "create", "update", ""
    pub sync_info: String,
    // Preset.hpp:267
    pub description: String,
    // Preset.hpp:268  last updated time
    pub updated_time: i64,
    // Preset.hpp:269
    pub key_values: BTreeMap<String, String>,
}

impl Preset {
    // BBS: move constructor to public
    // Preset.hpp:376  Preset(Type type, const std::string &name, bool is_default = false) : type(type), is_default(is_default), name(name) {}
    pub fn new(ty: Type, name: &str, is_default: bool) -> Self {
        Self {
            r#type: ty,
            is_default,
            is_external: false,
            is_system: false,
            is_visible: true,
            is_dirty: false,
            is_compatible: true,
            is_project_embedded: false,
            name: name.to_string(),
            file: String::new(),
            loaded: false,
            alias: String::new(),
            renamed_from: Vec::new(),
            version: Semver::default(),
            ini_str: String::new(),
            setting_id: String::new(),
            filament_id: String::new(),
            user_id: String::new(),
            base_id: String::new(),
            sync_info: String::new(),
            description: String::new(),
            updated_time: 0,
            key_values: BTreeMap::new(),
        }
    }

    // Preset.hpp:234  bool is_user() const { return ! this->is_default && ! this->is_system && ! this->is_project_embedded; }
    pub fn is_user(&self) -> bool {
        !self.is_default && !self.is_system && !self.is_project_embedded
    }

    // Preset.cpp:368  const std::string& Preset::suffix_modified()
    pub fn suffix_modified() -> String {
        g_suffix_modified_get()
    }

    // Preset.cpp:373  void Preset::update_suffix_modified(const std::string& new_suffix_modified)
    pub fn update_suffix_modified(new_suffix_modified: &str) {
        let mut guard = G_SUFFIX_MODIFIED.write().unwrap();
        *guard = Some(new_suffix_modified.to_string());
    }

    // Remove an optional "(modified)" suffix from a name.
    // This converts a UI name to a unique preset identifier.
    // Preset.cpp:379  std::string Preset::remove_suffix_modified(const std::string &name)
    pub fn remove_suffix_modified(name: &str) -> String {
        let g_suffix_modified = g_suffix_modified_get();
        if name.starts_with(&g_suffix_modified) {
            name[g_suffix_modified.len()..].to_string()
        } else {
            name.to_string()
        }
    }

    // Return a label of this preset, consisting of a name and a "(modified)" suffix, if this preset is dirty.
    // Preset.cpp:665  std::string Preset::label(bool no_alias) const
    pub fn label(&self, no_alias: bool) -> String {
        let prefix = if self.is_dirty {
            g_suffix_modified_get()
        } else {
            String::new()
        };
        let body = if no_alias || self.alias.is_empty() {
            self.name.clone()
        } else {
            self.alias.clone()
        };
        prefix + &body
    }

    // ------------------------------------------------------------------
    // Static option-name vectors (Preset.cpp:954-1210)
    // ------------------------------------------------------------------

    // Preset.cpp:1202  const std::vector<std::string>& Preset::print_options()
    pub fn print_options() -> &'static [&'static str] {
        &S_PRESET_PRINT_OPTIONS
    }

    // Preset.cpp:1203  const std::vector<std::string>& Preset::filament_options()
    pub fn filament_options() -> &'static [&'static str] {
        &S_PRESET_FILAMENT_OPTIONS
    }

    // Preset.cpp:1204  const std::vector<std::string>& Preset::machine_limits_options()
    pub fn machine_limits_options() -> &'static [&'static str] {
        &S_PRESET_MACHINE_LIMITS_OPTIONS
    }

    // Preset.cpp:1208  const std::vector<std::string>& Preset::sla_print_options()
    pub fn sla_print_options() -> &'static [&'static str] {
        &S_PRESET_SLA_PRINT_OPTIONS
    }

    // Preset.cpp:1209  const std::vector<std::string>& Preset::sla_material_options()
    pub fn sla_material_options() -> &'static [&'static str] {
        &S_PRESET_SLA_MATERIAL_OPTIONS
    }

    // Preset.cpp:1210  const std::vector<std::string>& Preset::sla_printer_options()
    pub fn sla_printer_options() -> &'static [&'static str] {
        &S_PRESET_SLA_PRINTER_OPTIONS
    }
}

// Preset.cpp:954  static std::vector<std::string> s_Preset_print_options
static S_PRESET_PRINT_OPTIONS: &[&str] = &[
    "layer_height", "initial_layer_print_height", "wall_loops", "slice_closing_radius", "spiral_mode", "spiral_mode_smooth", "spiral_mode_max_xy_smoothing", "slicing_mode",
    "top_shell_layers", "top_shell_thickness", "bottom_shell_layers", "bottom_shell_thickness", "ensure_vertical_shell_thickness", "reduce_crossing_wall", "detect_thin_wall",
    "detect_overhang_wall", "top_color_penetration_layers", "bottom_color_penetration_layers",
    "infill_instead_top_bottom_surfaces",
    "smooth_speed_discontinuity_area", "smooth_coefficient", "seam_position", "seam_placement_away_from_overhangs", "wall_sequence", "is_infill_first", "sparse_infill_density", "fill_multiline",
    "sparse_infill_pattern", "sparse_infill_anchor", "sparse_infill_anchor_max", "top_surface_pattern", "monotonic_travel_into_wall",
    "locked_skin_infill_pattern", "locked_skeleton_infill_pattern",
    "bottom_surface_pattern", "internal_solid_infill_pattern", "infill_direction", "bridge_angle", "infill_shift_step", "skeleton_infill_density", "infill_lock_depth", "skin_infill_depth", "skin_infill_density",
    "infill_rotate_step", "top_surface_density", "bottom_surface_density",
    "symmetric_infill_y_axis", "sparse_infill_lattice_angle_1", "sparse_infill_lattice_angle_2",
    "minimum_sparse_infill_area", "reduce_infill_retraction_mode", "ironing_pattern", "ironing_type",
    "ironing_flow", "ironing_speed", "ironing_spacing", "ironing_direction", "ironing_inset",
    "enable_support_ironing", "support_ironing_pattern", "support_ironing_speed",
    "support_ironing_flow", "support_ironing_spacing", "support_ironing_inset", "support_ironing_direction",
    "max_travel_detour_distance", "avoid_crossing_wall_includes_support",
    "fuzzy_skin", "fuzzy_skin_thickness", "fuzzy_skin_point_distance",
    "fuzzy_skin_first_layer", "fuzzy_skin_noise_type", "fuzzy_skin_scale", "fuzzy_skin_octaves", "fuzzy_skin_persistence", "fuzzy_skin_mode",
    // HAS_PRESSURE_EQUALIZER not defined: "max_volumetric_extrusion_rate_slope_positive", "max_volumetric_extrusion_rate_slope_negative",
    "inner_wall_speed", "outer_wall_speed", "sparse_infill_speed", "internal_solid_infill_speed",
    "top_surface_speed", "support_speed", "support_object_xy_distance", "support_object_first_layer_gap", "support_interface_speed",
    "bridge_speed", "gap_infill_speed", "travel_speed", "travel_speed_z", "initial_layer_speed", "outer_wall_acceleration",
    "initial_layer_acceleration", "top_surface_acceleration", "default_acceleration", "travel_acceleration", "travel_short_distance_acceleration", "initial_layer_travel_acceleration", "inner_wall_acceleration", "sparse_infill_acceleration",
    "accel_to_decel_enable", "accel_to_decel_factor", "skirt_loops", "skirt_distance",
    "skirt_height", "draft_shield",
    "brim_width", "brim_object_gap", "brim_type", "enable_support", "support_type", "support_threshold_angle", "enforce_support_layers",
    "raft_layers", "raft_first_layer_density", "raft_first_layer_expansion", "raft_contact_distance", "raft_expansion",
    "support_base_pattern", "support_base_pattern_spacing", "support_expansion", "support_style",
    // BBS
    "print_extruder_id", "print_extruder_variant", "independent_support_layer_height",
    "top_z_overrides_xy_distance",
    "support_angle", "support_interface_top_layers", "support_interface_bottom_layers",
    "support_interface_pattern", "support_interface_spacing", "support_interface_loop_pattern",
    "support_top_z_distance", "support_on_build_plate_only", "support_critical_regions_only", "support_remove_small_overhang",
    "bridge_no_support", "thick_bridges", "max_bridge_length", "print_sequence",
    "filename_format", "wall_filament", "support_bottom_z_distance",
    "sparse_infill_filament", "solid_infill_filament", "support_filament", "support_interface_filament", "support_interface_not_for_body",
    "ooze_prevention", "standby_temperature_delta", "interface_shells", "line_width", "initial_layer_line_width", "inner_wall_line_width",
    "outer_wall_line_width", "sparse_infill_line_width", "internal_solid_infill_line_width",
    "skin_infill_line_width", "skeleton_infill_line_width",
    "top_surface_line_width", "support_line_width", "infill_wall_overlap", "bridge_flow",
    "elefant_foot_compensation", "xy_contour_compensation", "xy_hole_compensation", "resolution", "enable_prime_tower", "prime_tower_enable_framework",
    "prime_tower_width", "prime_tower_brim_width", "prime_tower_skip_points", "prime_tower_max_speed", "enable_tower_interface_features",
    "prime_tower_rib_wall", "prime_tower_extra_rib_length", "prime_tower_rib_width", "prime_tower_fillet_wall", "prime_tower_infill_gap", "prime_tower_lift_speed", "prime_tower_lift_height",
    "prime_tower_flat_ironing", "enable_circle_compensation", "circle_compensation_manual_offset", "apply_scarf_seam_on_circles",
    "wipe_tower_no_sparse_layers", "compatible_printers", "compatible_printers_condition", "inherits",
    "flush_into_infill", "flush_into_objects", "flush_into_support", "process_notes", "enable_mixed_color_sublayer",
    // BBS
    "tree_support_branch_angle", "tree_support_wall_count", "tree_support_branch_distance", "tree_support_branch_diameter",
    "tree_support_branch_diameter_angle",
    "detect_narrow_internal_solid_infill",
    "gcode_add_line_number", "enable_arc_fitting", "precise_z_height", "infill_combination", /*"adaptive_layer_height",*/
    "support_bottom_interface_spacing", "enable_overhang_speed", "overhang_1_4_speed", "overhang_2_4_speed", "overhang_3_4_speed", "overhang_4_4_speed", "overhang_totally_speed",
    "enable_height_slowdown", "slowdown_start_height", "slowdown_start_speed", "slowdown_start_acc", "slowdown_end_height", "slowdown_end_speed", "slowdown_end_acc",
    "initial_layer_infill_speed", "top_one_wall_type", "top_area_threshold", "only_one_wall_first_layer",
    "timelapse_type", "internal_bridge_support_thickness",
    "wall_generator", "wall_transition_length", "wall_transition_filter_deviation", "wall_transition_angle",
    "wall_distribution_count", "min_feature_size", "min_bead_width", "post_process",
    "seam_gap", "wipe_speed", "top_solid_infill_flow_ratio", "initial_layer_flow_ratio",
    "default_jerk", "outer_wall_jerk", "inner_wall_jerk", "infill_jerk", "top_surface_jerk", "initial_layer_jerk", "travel_jerk",
    "filter_out_gap_fill", "mmu_segmented_region_max_width", "mmu_segmented_region_interlocking_depth",
    "small_perimeter_speed", "small_perimeter_threshold", "z_direction_outwall_speed_continuous",
    "vertical_shell_speed", "detect_floating_vertical_shell", "enable_wrapping_detection",
    // calib
    "print_flow_ratio",
    //Orca
    "exclude_object", "override_filament_scarf_seam_setting", "seam_slope_type", "seam_slope_conditional", "scarf_angle_threshold",
    "seam_slope_start_height", "seam_slope_entire_loop", "seam_slope_min_length",
    "seam_slope_steps", "seam_slope_inner_walls", "role_base_wipe_speed", "seam_slope_gap", "precise_outer_wall",
    "interlocking_beam", "interlocking_orientation", "interlocking_beam_layer_count", "interlocking_depth", "interlocking_boundary_avoidance", "interlocking_beam_width", "embedding_wall_into_infill",
];

// Preset.cpp:1027  static std::vector<std::string> s_Preset_filament_options
static S_PRESET_FILAMENT_OPTIONS: &[&str] = &[
    /*"filament_colour", */ "default_filament_colour", "required_nozzle_HRC", "filament_diameter", "volumetric_speed_coefficients", "filament_type",
    "filament_soluble", "filament_is_support", "filament_printable", "filament_extruder_compatibility", "filament_scarf_seam_type", "filament_scarf_height",
    "filament_scarf_gap", "filament_scarf_length",
    "filament_max_volumetric_speed", "impact_strength_z", "filament_ramming_volumetric_speed", "filament_ramming_volumetric_speed_nc", "filament_adaptive_volumetric_speed",
    "filament_flow_ratio", "filament_density", "filament_adhesiveness_category", "filament_metal_stickiness", "filament_cost", "filament_minimal_purge_on_wipe_tower",
    "nozzle_temperature", "nozzle_temperature_initial_layer",
    // BBS
    "cool_plate_temp", "eng_plate_temp", "hot_plate_temp", "textured_plate_temp", "cool_plate_temp_initial_layer", "eng_plate_temp_initial_layer", "hot_plate_temp_initial_layer", "textured_plate_temp_initial_layer",
    "supertack_plate_temp_initial_layer", "supertack_plate_temp",
    "circle_compensation_speed", "counter_coef_1", "counter_coef_2", "counter_coef_3", "hole_coef_1", "hole_coef_2", "hole_coef_3",
    "counter_limit_min", "counter_limit_max", "hole_limit_min", "hole_limit_max", "diameter_limit",
    // "bed_type",
    //BBS:temperature_vitrification
    "temperature_vitrification", "reduce_fan_stop_start_freq", "slow_down_for_layer_cooling", "no_slow_down_for_cooling_on_outwalls", "cooling_slowdown_logic", "cooling_perimeter_transition_distance", "fan_min_speed", "filament_ramming_travel_time", "filament_pre_cooling_temperature", "filament_ramming_travel_time_nc", "filament_pre_cooling_temperature_nc",
    "fan_max_speed", "enable_overhang_bridge_fan", "overhang_fan_speed", "ironing_fan_speed", "pre_start_fan_time", "overhang_fan_threshold", "overhang_threshold_participating_cooling", "close_fan_the_first_x_layers", "first_x_layer_part_fan_speed", "close_additional_fan_first_x_layers", "first_x_layer_fan_speed", "full_fan_speed_layer", "additional_fan_full_speed_layer", "fan_cooling_layer_time", "slow_down_layer_time", "slow_down_min_speed",
    "filament_start_gcode", "filament_end_gcode",
    //exhaust fan control
    "activate_air_filtration", "during_print_exhaust_fan_speed", "complete_print_exhaust_fan_speed",
    // Retract overrides
    "filament_retraction_length", "filament_z_hop", "filament_z_hop_types", "filament_retraction_speed", "filament_deretraction_speed", "filament_retract_length_nc", "filament_retract_restart_extra", "filament_retraction_minimum_travel",
    "filament_retract_when_changing_layer", "filament_wipe", "filament_retract_before_wipe",
    // Profile compatibility
    "filament_vendor", "compatible_prints", "compatible_prints_condition", "compatible_printers", "compatible_printers_condition", "inherits",
    //BBS
    "filament_wipe_distance", "additional_cooling_fan_speed",
    "nozzle_temperature_range_low", "nozzle_temperature_range_high",
    "filament_extruder_variant",
    //OrcaSlicer
    "enable_pressure_advance", "pressure_advance", "chamber_temperatures", "filament_notes",
    "filament_long_retractions_when_cut", "filament_retraction_distances_when_cut", "filament_shrink", "filament_velocity_adaptation_factor",
    //BBS filament change length while the extruder color
    "filament_change_length", "filament_change_length_nc", "filament_prime_volume", "filament_prime_volume_nc", "filament_flush_volumetric_speed", "filament_flush_temp",
    "long_retractions_when_ec", "retraction_distances_when_ec",
    "filament_enable_overhang_speed",
    "filament_bridge_speed",
    "filament_overhang_1_4_speed",
    "filament_overhang_2_4_speed",
    "filament_overhang_3_4_speed",
    "filament_overhang_4_4_speed",
    "filament_overhang_totally_speed",
    "override_process_overhang_speed",
    "filament_cooling_before_tower",
    "filament_tower_interface_pre_extrusion_dist",
    "filament_tower_interface_pre_extrusion_length",
    "filament_tower_ironing_area",
    "filament_tower_interface_purge_volume",
    "filament_tower_interface_print_temp",
    //ams chamber
    "filament_dev_ams_drying_ams_limitations", "filament_dev_ams_drying_temperature", "filament_dev_ams_drying_time", "filament_dev_ams_drying_heat_distortion_temperature",
    "filament_dev_chamber_drying_bed_temperature", "filament_dev_chamber_drying_time",
    "filament_dev_drying_softening_temperature", "filament_dev_drying_cooling_temperature",
];

// Preset.cpp:1081  static std::vector<std::string> s_Preset_machine_limits_options
static S_PRESET_MACHINE_LIMITS_OPTIONS: &[&str] = &[
    "machine_max_acceleration_extruding", "machine_max_acceleration_retracting", "machine_max_acceleration_travel",
    "machine_max_acceleration_x", "machine_max_acceleration_y", "machine_max_acceleration_z", "machine_max_acceleration_e",
    "machine_max_speed_x", "machine_max_speed_y", "machine_max_speed_z", "machine_max_speed_e",
    "machine_min_extruding_rate", "machine_min_travel_rate",
    "machine_max_jerk_x", "machine_max_jerk_y", "machine_max_jerk_z", "machine_max_jerk_e",
];

// Preset.cpp:1089  static std::vector<std::string> s_Preset_printer_options
// (Used by Preset::printer_options(), which is BLOCKED because it appends
//  nozzle_options() == print_config_def.extruder_option_keys() — see report.)
pub static S_PRESET_PRINTER_OPTIONS: &[&str] = &[
    "printer_technology",
    "printable_area", "extruder_printable_area", "bed_exclude_area", "bed_custom_texture", "bed_custom_model", "gcode_flavor",
    "single_extruder_multi_material", "machine_start_gcode", "machine_end_gcode", "printing_by_object_gcode", "before_layer_change_gcode", "layer_change_gcode", "time_lapse_gcode", "wrapping_detection_gcode", "change_filament_gcode",
    "printer_model", "printer_variant", "printer_extruder_id", "printer_extruder_variant", "extruder_variant_list", "default_nozzle_volume_type",
    "printable_height", "extruder_printable_height", "extruder_clearance_dist_to_rod", "extruder_clearance_max_radius", "extruder_clearance_height_to_lid", "extruder_clearance_height_to_rod",
    "nozzle_height", "master_extruder_id",
    "default_print_profile", "inherits",
    "silent_mode",
    // BBS
    "scan_first_layer", "wrapping_detection_layers", "wrapping_exclude_area", "machine_load_filament_time", "machine_unload_filament_time", "machine_pause_gcode", "template_custom_gcode", "machine_hotend_change_time",
    "nozzle_type", "auxiliary_fan", "fan_direction", "nozzle_volume", "upward_compatible_machine", "z_hop_types", "support_chamber_temp_control", "support_air_filtration", "support_cooling_filter", "cooling_filter_enabled", "printer_structure", "thumbnail_size",
    "best_object_pos", "head_wrap_detect_zone", "printer_notes", "print_in_clockwise",
    "enable_long_retraction_when_cut", "long_retractions_when_cut", "retraction_distances_when_cut",
    //OrcaSlicer
    "host_type", "print_host", "printhost_apikey",
    "print_host_webui",
    "printhost_cafile", "printhost_port", "printhost_authorization_type",
    "printhost_user", "printhost_password", "printhost_ssl_ignore_revoke",
    "use_relative_e_distances", "extruder_type", "use_firmware_retraction",
    "grab_length", "machine_switch_extruder_time", "hotend_cooling_rate", "hotend_heating_rate", "enable_pre_heating", "support_object_skip_flush", "physical_extruder_map",
    "bed_temperature_formula", "machine_prepare_compensation_time", "nozzle_flush_dataset",
    "group_algo_with_time", "extruder_max_nozzle_count",
];

// Preset.cpp:1114  static std::vector<std::string> s_Preset_sla_print_options
static S_PRESET_SLA_PRINT_OPTIONS: &[&str] = &[
    "layer_height",
    "faded_layers",
    "supports_enable",
    "support_head_front_diameter",
    "support_head_penetration",
    "support_head_width",
    "support_pillar_diameter",
    "support_small_pillar_diameter_percent",
    "support_max_bridges_on_pillar",
    "support_pillar_connection_mode",
    "support_buildplate_only",
    "support_pillar_widening_factor",
    "support_base_diameter",
    "support_base_height",
    "support_base_safety_distance",
    "support_critical_angle",
    "support_max_bridge_length",
    "support_max_pillar_link_distance",
    "support_object_elevation",
    "support_points_density_relative",
    "support_points_minimal_distance",
    "slice_closing_radius",
    "pad_enable",
    "pad_wall_thickness",
    "pad_wall_height",
    "pad_brim_size",
    "pad_max_merge_distance",
    // "pad_edge_radius",
    "pad_wall_slope",
    "pad_object_gap",
    "pad_around_object",
    "pad_around_object_everywhere",
    "pad_object_connector_stride",
    "pad_object_connector_width",
    "pad_object_connector_penetration",
    "hollowing_enable",
    "hollowing_min_thickness",
    "hollowing_quality",
    "hollowing_closing_distance",
    "filename_format",
    "default_sla_print_profile",
    "compatible_printers",
    "compatible_printers_condition",
    "inherits",
];

// Preset.cpp:1161  static std::vector<std::string> s_Preset_sla_material_options
static S_PRESET_SLA_MATERIAL_OPTIONS: &[&str] = &[
    "material_colour",
    "material_type",
    "initial_layer_height",
    "bottle_cost",
    "bottle_volume",
    "bottle_weight",
    "material_density",
    "exposure_time",
    "initial_exposure_time",
    "material_correction",
    "material_correction_x",
    "material_correction_y",
    "material_correction_z",
    "material_vendor",
    "material_print_speed",
    "default_sla_material_profile",
    "compatible_prints", "compatible_prints_condition",
    "compatible_printers", "compatible_printers_condition", "inherits",
];

// Preset.cpp:1182  static std::vector<std::string> s_Preset_sla_printer_options
static S_PRESET_SLA_PRINTER_OPTIONS: &[&str] = &[
    "printer_technology",
    "printable_area", "bed_custom_texture", "bed_custom_model", "printable_height",
    "display_width", "display_height", "display_pixels_x", "display_pixels_y",
    "display_mirror_x", "display_mirror_y",
    "display_orientation",
    "fast_tilt_time", "slow_tilt_time", "area_fill",
    "relative_correction",
    "relative_correction_x",
    "relative_correction_y",
    "relative_correction_z",
    "absolute_correction",
    "elefant_foot_compensation",
    "elefant_foot_min_width",
    "gamma_correction",
    "min_exposure_time", "max_exposure_time",
    "min_initial_exposure_time", "max_initial_exposure_time",
    "inherits",
];

// ============================================================================
// PresetSelectCompatibleType (Preset.hpp:389-396)
// ============================================================================

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PresetSelectCompatibleType {
    // Never select a compatible preset if the newly selected profile is not compatible.
    Never,
    // Only select a compatible preset if the active profile used to be compatible, but it is no more.
    OnlyIfWasCompatible,
    // Always select a compatible preset if the active profile is no more compatible.
    Always,
}

// ============================================================================
// PresetConfigSubstitutions::Source (Preset.hpp:404-410)
// ============================================================================

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PresetConfigSubstitutionsSource {
    UserFile,
    ConfigBundle,
    //BBS: add cloud and project type
    UserCloud,
    ProjectFile,
}

// ============================================================================
// PresetCollection::LoadAndSelect (Preset.hpp:509-516)
// ============================================================================

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum LoadAndSelect {
    // Never select
    Never,
    // Always select
    Always,
    // Select a profile only if it was modified.
    OnlyIfModified,
}

// ============================================================================
// PhysicalPrinter (Preset.hpp:854-920) — pure helpers only
// ============================================================================

// Preset.cpp:3479  static std::vector<std::string> s_PhysicalPrinter_opts
static S_PHYSICAL_PRINTER_OPTS: &[&str] = &[
    "preset_name", // temporary option to compatibility with older Slicer
    "preset_names",
    "printer_technology",
    "host_type",
    "print_host",
    "printhost_apikey",
    "printhost_cafile",
    "printhost_port",
    "printhost_authorization_type",
    // HTTP digest authentization (RFC 2617)
    "printhost_user",
    "printhost_password",
    "printhost_ssl_ignore_revoke",
];

#[derive(Clone, Debug, Default)]
pub struct PhysicalPrinter {
    // Preset.hpp:862  Name of the Physical Printer, usually derived form the file name.
    pub name: String,
    // Preset.hpp:864  File name of the Physical Printer.
    pub file: String,
    // Preset.hpp:868  set of presets used with this physical printer
    pub preset_names: std::collections::BTreeSet<String>,
    // Preset.hpp:871  Has this profile been loaded?
    pub loaded: bool,
}

impl PhysicalPrinter {
    // Preset.cpp:3474  std::string PhysicalPrinter::separator()
    pub fn separator() -> String {
        " * ".to_string()
    }

    // Preset.cpp:3495  const std::vector<std::string>& PhysicalPrinter::printer_options()
    pub fn printer_options() -> &'static [&'static str] {
        S_PHYSICAL_PRINTER_OPTS
    }

    // Preset.cpp:3510  bool PhysicalPrinter::has_print_host_information(const DynamicPrintConfig& config)
    // (config arg dropped: the C++ body unconditionally returns false.)
    pub fn has_print_host_information() -> bool {
        false
    }

    // Preset.cpp:3515  const std::set<std::string>& PhysicalPrinter::get_preset_names() const
    pub fn get_preset_names(&self) -> &std::collections::BTreeSet<String> {
        &self.preset_names
    }

    // Preset.cpp:3578  void PhysicalPrinter::reset_presets()
    pub fn reset_presets(&mut self) {
        self.preset_names.clear();
    }

    // Preset.cpp:3583  bool PhysicalPrinter::add_preset(const std::string& preset_name)
    pub fn add_preset(&mut self, preset_name: &str) -> bool {
        self.preset_names.insert(preset_name.to_string())
    }

    // Preset.cpp:3588  bool PhysicalPrinter::delete_preset(const std::string& preset_name)
    pub fn delete_preset(&mut self, preset_name: &str) -> bool {
        self.preset_names.remove(preset_name)
    }

    // Preset.cpp:3605  void PhysicalPrinter::set_name(const std::string& name)
    pub fn set_name(&mut self, name: &str) {
        self.name = name.to_string();
    }

    // Preset.cpp:3610  std::string PhysicalPrinter::get_full_name(std::string preset_name) const
    pub fn get_full_name(&self, preset_name: &str) -> String {
        self.name.clone() + &PhysicalPrinter::separator() + preset_name
    }

    // Preset.cpp:3615  std::string PhysicalPrinter::get_short_name(std::string full_name)
    pub fn get_short_name(full_name: &str) -> String {
        let mut full_name = full_name.to_string();
        // int pos = full_name.find(separator());
        let sep = PhysicalPrinter::separator();
        let pos: i64 = match full_name.find(&sep) {
            Some(p) => p as i64,
            None => -1,
        };
        // if (pos > 0) boost::erase_tail(full_name, full_name.length() - pos);
        if pos > 0 {
            // boost::erase_tail removes the last `n` chars; keep the first `pos`.
            full_name.truncate(pos as usize);
        }
        full_name
    }

    // Preset.cpp:3623  std::string PhysicalPrinter::get_preset_name(std::string name)
    pub fn get_preset_name(name: &str) -> String {
        let mut name = name.to_string();
        // int pos = name.find(separator());
        let sep = PhysicalPrinter::separator();
        let pos: i64 = match name.find(&sep) {
            Some(p) => p as i64,
            None => -1,
        };
        // boost::erase_head(name, pos + 3); -> remove the first (pos + 3) chars
        let head = pos + 3;
        if head > 0 {
            let head = head as usize;
            name = if head >= name.len() {
                String::new()
            } else {
                name[head..].to_string()
            };
        }
        Preset::remove_suffix_modified(&name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_type_string() {
        assert_eq!(Type::get_type_string(Type::TypeFilament), "filament");
        assert_eq!(Type::get_type_string(Type::TypePrint), "process");
        assert_eq!(Type::get_type_string(Type::TypePrinter), "machine");
        assert_eq!(
            Type::get_type_string(Type::TypePhysicalPrinter),
            "physical_printer"
        );
        assert_eq!(Type::get_type_string(Type::TypeInvalid), "invalid");
        assert_eq!(Type::get_type_string(Type::TypePlate), "invalid");
    }

    #[test]
    fn test_get_type_from_string() {
        assert_eq!(Type::get_type_from_string("process"), Type::TypePrint);
        assert_eq!(Type::get_type_from_string("print"), Type::TypePrint);
        assert_eq!(Type::get_type_from_string("filament"), Type::TypeFilament);
        assert_eq!(Type::get_type_from_string("machine"), Type::TypePrinter);
        assert_eq!(Type::get_type_from_string("printer"), Type::TypePrinter);
        assert_eq!(Type::get_type_from_string("bogus"), Type::TypeInvalid);
    }

    #[test]
    fn test_remove_suffix_modified() {
        assert_eq!(Preset::remove_suffix_modified(" (modified)foo"), "foo");
        assert_eq!(Preset::remove_suffix_modified("foo"), "foo");
    }

    #[test]
    fn test_label() {
        let mut p = Preset::new(Type::TypePrint, "MyPreset", false);
        assert_eq!(p.label(true), "MyPreset");
        p.alias = "Alias".to_string();
        assert_eq!(p.label(false), "Alias");
        assert_eq!(p.label(true), "MyPreset");
        p.is_dirty = true;
        assert_eq!(p.label(true), " (modified)MyPreset");
    }

    #[test]
    fn test_physical_printer_names() {
        let p = PhysicalPrinter {
            name: "Printer".to_string(),
            ..Default::default()
        };
        assert_eq!(p.get_full_name("Preset"), "Printer * Preset");
        assert_eq!(PhysicalPrinter::get_short_name("Printer * Preset"), "Printer");
        assert_eq!(PhysicalPrinter::get_preset_name("Printer * Preset"), "Preset");
    }
}
