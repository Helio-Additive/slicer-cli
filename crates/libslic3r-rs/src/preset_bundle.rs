//! Bundle of Print + Filament + Printer presets.
//!
//! C++ Reference:
//! - PresetBundle.hpp
//! - PresetBundle.cpp
//!
//! PORT STATUS: PARTIAL — only the dependency-free constants and static tables
//! of `PresetBundle.cpp` are ported faithfully here. The C++ `PresetBundle`
//! class (~60 methods, 6031 lines) is a thin orchestrator over three core
//! abstractions that are **not yet present** in the Rust crate:
//!
//!   * `PresetCollection` / `PrinterPresetCollection` / `PhysicalPrinterCollection`
//!     (Preset.hpp:423/822/928) — the `prints`, `sla_prints`, `filaments`,
//!     `sla_materials`, `printers`, `physical_printers` members. `preset.rs`
//!     ports `Preset` but NOT these collections.
//!   * `Preset::config` (a `DynamicPrintConfig`) — the Rust `Preset` in
//!     `preset.rs` deliberately omits the `config` field (see its module docs),
//!     so there is no per-preset config to merge.
//!   * A dynamic, name-indexed `DynamicPrintConfig` with typed options
//!     (`option<ConfigOptionInts>("filament_map")`,
//!     `dynamic_cast<ConfigOptionFloatsNullable*>(out.option("nozzle_diameter"))`,
//!     `apply`, `apply_only`, `has`, `opt_int`, …) plus `FullPrintConfig::defaults()`.
//!     The Rust `print_config::PrintConfig` is a fixed-field struct with
//!     `apply_from`; it does not support the by-name typed-option queries that
//!     `construct_full_config`/`full_fff_config`/`full_config` require.
//!
//! Because every `PresetBundle` method reads/writes those types, none of them
//! can be ported as a faithful 1:1 translation today. See the BLOCKED list at
//! the bottom of this file. This module intentionally contains NO stubs or
//! fakes for those methods — the previous revision of this file fabricated
//! types and logic (e.g. a `PresetBundle { print_presets: Vec<PresetEntry> }`,
//! a hand-invented HRC lookup table, a JSON `load_config_file`) that bore no
//! relation to the C++ and have been removed per the parity rules.

// PresetBundle.hpp:15-16
pub const DEFAULT_USER_FOLDER_NAME: &str = "default";
pub const BUNDLE_STRUCTURE_JSON_NAME: &str = "bundle_structure.json";

// PresetBundle.hpp:18-21
pub const VALIDATE_PRESETS_SUCCESS: i32 = 0;
pub const VALIDATE_PRESETS_PRINTER_NOT_FOUND: i32 = 1;
pub const VALIDATE_PRESETS_FILAMENTS_NOT_FOUND: i32 = 2;
pub const VALIDATE_PRESETS_MODIFIED_GCODES: i32 = 3;

// PresetBundle.hpp:33-46
//
// AMS combo UI information. Mirrors `struct AMSComboInfo`.
#[derive(Debug, Clone, Default)]
pub struct AMSComboInfo {
    // PresetBundle.hpp:34
    pub ams_filament_colors: Vec<String>,
    // PresetBundle.hpp:35
    pub ams_multi_color_filment: Vec<Vec<String>>,
    // PresetBundle.hpp:36
    pub ams_filament_presets: Vec<String>,
    // PresetBundle.hpp:37
    pub ams_names: Vec<String>,
}

impl AMSComboInfo {
    // PresetBundle.hpp:38-43
    pub fn clear(&mut self) {
        self.ams_filament_colors.clear();
        self.ams_multi_color_filment.clear();
        self.ams_filament_presets.clear();
        self.ams_names.clear();
    }

    // PresetBundle.hpp:44-46
    pub fn empty(&self) -> bool {
        self.ams_names.is_empty()
    }
}

// PresetBundle.hpp:26-31
//
// `struct AMSMapInfo` — for new ams mapping (from struct FilamentInfo).
#[derive(Debug, Clone, Default)]
pub struct AMSMapInfo {
    // PresetBundle.hpp:29
    pub ams_id: String,
    // PresetBundle.hpp:30
    pub slot_id: String,
}

// PresetBundle.hpp:48-51
//
// `struct MergeFilamentInfo`.
#[derive(Debug, Clone, Default)]
pub struct MergeFilamentInfo {
    // PresetBundle.hpp:49
    pub merges: Vec<Vec<i32>>,
}

impl MergeFilamentInfo {
    // PresetBundle.hpp:50
    pub fn is_empty(&self) -> bool {
        self.merges.is_empty()
    }
}

// PresetBundle.hpp:349-358
//
// `enum LoadConfigBundleAttribute` — flags used by load_*_configs_from_json.
// NOTE: the C++ values differ entirely from the prior (fabricated) Rust enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoadConfigBundleAttribute {
    // PresetBundle.hpp:351  Save the profiles, which have been loaded.
    SaveImported,
    // PresetBundle.hpp:353  Delete all old config profiles before loading.
    ResetUserProfile,
    // PresetBundle.hpp:355  Load a system config bundle.
    LoadSystem,
    // PresetBundle.hpp:356
    LoadVendorOnly,
    // PresetBundle.hpp:357
    LoadFilamentOnly,
}

// PresetBundle.cpp:73-76
//BBS: add BBL as default
pub const BBL_BUNDLE: &str = "BBL";
pub const BBL_DEFAULT_PRINTER_MODEL: &str = "Bambu Lab X1 Carbon";
pub const BBL_DEFAULT_PRINTER_VARIANT: &str = "0.4";
pub const BBL_DEFAULT_FILAMENT: &str = "Generic PLA";

// PresetBundle.cpp:44-70
//
// The project configuration values are kept separated from the
// print/filament/printer preset; this is the canonical list of option keys
// that `full_fff_config`/`construct_full_config` treat as project-level
// (applied via `out.apply(project_config)`).
pub static S_PROJECT_OPTIONS: &[&str] = &[
    "flush_volumes_vector",
    "flush_volumes_matrix",
    // BBS
    "filament_colour",
    "filament_colour_type",
    "filament_multi_colour",
    "wipe_tower_x",
    "wipe_tower_y",
    "wipe_tower_rotation_angle",
    "curr_bed_type",
    "flush_multiplier",
    "nozzle_volume_type",
    "filament_map_mode",
    "filament_map",
    "filament_volume_map",
    "filament_nozzle_map",
    "extruder_nozzle_stats",
    "prime_volume_mode",
    "enable_filament_dynamic_map",
    "filament_is_mixed",
    "filament_mixed_components",
    "filament_mixed_sublayer_ratios",
    "filament_mixed_gradient",
    "filament_mixed_gradient_range",
    "has_filament_switcher",
];

// PresetBundle.cpp:1347-1348
//
// The set of custom-G-code option keys checked by `validate_presets` when
// comparing a config against the active presets (reported via
// VALIDATE_PRESETS_MODIFIED_GCODES).
pub static GCODES_KEY_SET: &[&str] = &[
    "filament_end_gcode",
    "filament_start_gcode",
    "change_filament_gcode",
    "layer_change_gcode",
    "machine_end_gcode",
    "machine_pause_gcode",
    "machine_start_gcode",
    "template_custom_gcode",
    "printing_by_object_gcode",
    "before_layer_change_gcode",
    "time_lapse_gcode",
    "wrapping_detection_gcode",
];

// ===========================================================================
// BLOCKED SYMBOLS (cannot be faithfully ported until the listed deps land).
// All require `PresetCollection`/`PrinterPresetCollection`/
// `PhysicalPrinterCollection`, `Preset::config` (DynamicPrintConfig), and/or a
// dynamic name-indexed `DynamicPrintConfig` with typed `ConfigOption*`.
//
//   construct_full_config                  PresetBundle.cpp:78
//   PresetBundle (ctor) / operator= / copy PresetBundle.cpp:434
//   reset                                  PresetBundle.cpp:458
//   setup_directories                      PresetBundle.cpp:478
//   copy_files                             PresetBundle.cpp:527
//   load_presets                           PresetBundle.cpp:548
//   get_preset_differed_for_save           PresetBundle.cpp:587
//   get_differed_values_to_update          PresetBundle.cpp:609
//   get_vendor_profile_version             PresetBundle.cpp:632
//   get_filament_by_filament_id            PresetBundle.cpp:644
//   load_project_embedded_presets          PresetBundle.cpp:695
//   get_current_project_embedded_presets   PresetBundle.cpp:730
//   reset_project_embedded_presets         PresetBundle.cpp:748
//   get_texture_for_printer_model          PresetBundle.cpp:806
//   get_stl_model_for_printer_model        PresetBundle.cpp:834
//   get_hotend_model_for_printer_model     PresetBundle.cpp:861
//   load_user_presets (x2)                 PresetBundle.cpp:888,933
//   import_presets                         PresetBundle.cpp:1012
//   import_json_presets                    PresetBundle.cpp:1099
//   save_user_presets                      PresetBundle.cpp:1203
//   update_user_presets_directory          PresetBundle.cpp:1228
//   remove_user_presets_directory          PresetBundle.cpp:1249
//   update_system_preset_setting_ids       PresetBundle.cpp:1264
//   get_full_flush_matrix                  PresetBundle.cpp:1315
//   validate_presets                       PresetBundle.cpp:1349
//   remove_users_preset                    PresetBundle.cpp:1423
//   load_system_presets_from_json          PresetBundle.cpp:1584
//   load_system_models_from_json           PresetBundle.cpp:1647
//   load_system_filaments_json             PresetBundle.cpp:1681
//   get_custom_vendor_models               PresetBundle.cpp:1732
//   merge_presets                          PresetBundle.cpp:1759
//   update_system_maps                     PresetBundle.cpp:1779
//   load_installed_printers                PresetBundle.cpp:1804
//   get_preset_name_by_alias               PresetBundle.cpp:1811
//   get_required_hrc_by_filament_id        PresetBundle.cpp:1826
//   save_changes_for_preset                PresetBundle.cpp:1836
//   load_installed_filaments               PresetBundle.cpp:1864
//   quick_fix_for_filaments_due_to_upgrade PresetBundle.cpp:1926
//   load_installed_sla_materials           PresetBundle.cpp:1970
//   load_selections                        PresetBundle.cpp:1995
//   export_selections                      PresetBundle.cpp:2217
//   set_num_filaments                      PresetBundle.cpp:2292
//   update_num_filaments                   PresetBundle.cpp:2341
//   is_mixed_filament                      PresetBundle.cpp:2414
//   physical_filament_config_indices       PresetBundle.cpp:2420
//   get_ams_cobox_infos                    PresetBundle.cpp:2429
//   sync_ams_list                          PresetBundle.cpp:2492
//   update_filament_multi_color            PresetBundle.cpp:2829
//   get_used_tpu_filaments                 PresetBundle.cpp:2850
//   set_calibrate_printer                  PresetBundle.cpp:2874
//   get_extruder_filament_info             PresetBundle.cpp:2897
//   get_printer_names_by_printer_type_and_nozzle              PresetBundle.cpp:2914
//   check_filament_temp_equation_by_printer_type_and_nozzle_for_mas_tray  PresetBundle.cpp:2950
//   get_similar_printer_preset             PresetBundle.cpp:3012
//   is_the_only_edited_filament            PresetBundle.cpp:3046
//   reset_default_nozzle_volume_type       PresetBundle.cpp:3073
//   get_printer_extruder_count             PresetBundle.cpp:3082
//   support_different_extruders            PresetBundle.cpp:3091
//   get_printer_nozzle_volume_list         PresetBundle.cpp:3100
//   get_default_nozzle_volume_types_for_filaments  PresetBundle.cpp:3116
//   full_config                            PresetBundle.cpp:3132
//   full_config_secure                     PresetBundle.cpp:3139
//   full_fff_config                        PresetBundle.cpp:3157
//   full_sla_config                        PresetBundle.cpp:3456
//   load_config_file                       PresetBundle.cpp:3511
//   load_config_file_config                PresetBundle.cpp:3631
//   load_vendor_configs_from_json (x2)     PresetBundle.cpp:4522,5335
//   on_extruders_count_changed             PresetBundle.cpp:5434
//   update_multi_material_filament_presets PresetBundle.cpp:5443
//   update_compatible                      PresetBundle.cpp:5507
//   export_current_configs                 PresetBundle.cpp:5860
//   set_filament_preset                    PresetBundle.cpp:5893
//   set_default_suppressed                 PresetBundle.cpp:5902
//   load_support_recommended_params        PresetBundle.cpp:5911
//   get_support_recommended_params         PresetBundle.cpp:6021
//   convert_filament_preset_name (free fn) PresetBundle.cpp (declared hpp:451)
//   ExtruderNozzleStat::* methods          PresetBundle.hpp:121-146
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bbl_constants() {
        // PresetBundle.cpp:73-76
        assert_eq!(BBL_BUNDLE, "BBL");
        assert_eq!(BBL_DEFAULT_PRINTER_MODEL, "Bambu Lab X1 Carbon");
        assert_eq!(BBL_DEFAULT_PRINTER_VARIANT, "0.4");
        assert_eq!(BBL_DEFAULT_FILAMENT, "Generic PLA");
    }

    #[test]
    fn test_validate_preset_codes() {
        // PresetBundle.hpp:18-21
        assert_eq!(VALIDATE_PRESETS_SUCCESS, 0);
        assert_eq!(VALIDATE_PRESETS_MODIFIED_GCODES, 3);
    }

    #[test]
    fn test_project_options_first_last() {
        // PresetBundle.cpp:45,69
        assert_eq!(S_PROJECT_OPTIONS.first().copied(), Some("flush_volumes_vector"));
        assert_eq!(S_PROJECT_OPTIONS.last().copied(), Some("has_filament_switcher"));
        assert_eq!(S_PROJECT_OPTIONS.len(), 24);
    }

    #[test]
    fn test_gcodes_key_set() {
        // PresetBundle.cpp:1347-1348
        assert_eq!(GCODES_KEY_SET.len(), 12);
        assert!(GCODES_KEY_SET.contains(&"machine_start_gcode"));
        assert!(GCODES_KEY_SET.contains(&"wrapping_detection_gcode"));
    }

    #[test]
    fn test_ams_combo_info_empty() {
        // PresetBundle.hpp:44-46
        let mut info = AMSComboInfo::default();
        assert!(info.empty());
        info.ams_names.push("AMS-1".to_string());
        assert!(!info.empty());
        info.clear();
        assert!(info.empty());
    }

    #[test]
    fn test_merge_filament_info_empty() {
        // PresetBundle.hpp:50
        let info = MergeFilamentInfo::default();
        assert!(info.is_empty());
    }
}
