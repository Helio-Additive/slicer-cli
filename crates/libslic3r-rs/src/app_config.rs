//! 1:1 port of `AppConfig.{hpp,cpp}` from BambuStudio's libslic3r.
//!
//! C++ source:
//!   - `src/libslic3r/AppConfig.hpp`
//!   - `src/libslic3r/AppConfig.cpp`
//!
//! Faithfulness notes (see RULES):
//!   * The active translation unit compiles with `USE_JSON_CONFIG` defined
//!     (AppConfig.cpp:36) and with `BBL_RELEASE_TO_PUBLIC` truthy in the public
//!     build. The non-JSON `#else` `load()`/`save()` (AppConfig.cpp:951-1155)
//!     and the WIN32-only MD5 helpers (AppConfig.cpp:518-562) are dead code in
//!     this configuration; they are documented but not emitted, matching what
//!     the compiler produces.
//!   * `escape_strings_cstyle` / `unescape_strings_cstyle` live in `Config.cpp`
//!     (not yet ported). Their string-only implementations are reproduced here
//!     as private helpers with `Config.cpp:NNN` refs so this file is
//!     self-contained until `config.rs` exists.
//!   * `slicer_uuid` (AppConfig.cpp:294-297) needs a random UUID. `boost::uuids`
//!     is a native dep; we synthesize a v4-shaped UUID from `std`'s
//!     non-cryptographic entropy sources (wasm-safe). This value never enters
//!     G-code, so byte-exact parity is unaffected.

use std::collections::{BTreeMap, HashMap, HashSet};

use crate::calib::{BedType, CaliPresetInfo, FlowRatioCalibrationType, NozzleVolumeType, PrinterCaliInfo};
use crate::locales_utils::{float_to_string_decimal_point, string_to_double_decimal_point};
use crate::log_sink::data_dir;
use crate::semver::Semver;
use crate::utils::{
    copy_file, get_current_pid, header_gcodeviewer_generated, header_slic3r_generated, is_shapes_dir, rename_file,
    CopyFileResult,
};

// AppConfig.hpp:16-19
#[allow(dead_code)]
pub const ENV_DEV_HOST: &str = "0";
#[allow(dead_code)]
pub const ENV_QAT_HOST: &str = "1";
#[allow(dead_code)]
pub const ENV_PRE_HOST: &str = "2";
#[allow(dead_code)]
pub const ENV_PRODUCT_HOST: &str = "3";

// libslic3r.h / version.inc
// SLIC3R_APP_KEY = "BambuStudio"; GCODEVIEWER_APP_KEY = "BambuStudioGcodeViewer".
const SLIC3R_APP_KEY: &str = "BambuStudio";
const GCODEVIEWER_APP_KEY: &str = "BambuStudioGcodeViewer";

// AppConfig.cpp:42-43
static VERSION_CHECK_URL: &str = "";
static MODELS_STR: &str = "models";

// Preset.hpp:20-22
const PRESET_FILAMENT_NAME: &str = "filament";
const PRESET_PRINT_NAME: &str = "process";
const PRESET_PRINTER_NAME: &str = "machine";

// PrintConfig.cpp:498 — ConfigOptionEnum<FilamentMapMode>::get_enum_names()[fmmAutoForFlush].
const FMM_AUTO_FOR_FLUSH_NAME: &str = "Auto For Flush";

// AppConfig.cpp:69 / :443 etc. — public release build.
const BBL_RELEASE_TO_PUBLIC: bool = true;

/// Application mode enum (Editor vs GCodeViewer)
/// AppConfig.hpp:33-37
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EAppMode {
    // AppConfig.hpp:35
    Editor,
    // AppConfig.hpp:36
    GCodeViewer,
}

// AppConfig.hpp:167  typedef std::map<std::string, std::map<std::string, std::set<std::string>>> VendorMap;
pub type VendorMap = BTreeMap<String, BTreeMap<String, HashSet<String>>>;

/// Application configuration manager, stores section->key->value settings.
/// AppConfig.hpp:30-299
#[derive(Debug)]
pub struct AppConfig {
    /// Type of application: Editor or GCodeViewer
    /// AppConfig.hpp:278
    m_mode: EAppMode,
    /// Map of section, name -> value
    /// AppConfig.hpp:280
    ///
    /// `BTreeMap` mirrors `std::map`'s ordered iteration, which `save()` relies
    /// on for deterministic output.
    m_storage: BTreeMap<String, BTreeMap<String, String>>,
    /// Map of enabled vendors / models / variants
    /// AppConfig.hpp:283
    m_vendors: VendorMap,
    /// Has any value been modified since the config.ini has been last saved or loaded?
    /// AppConfig.hpp:285
    m_dirty: bool,
    /// Original version found in the ini file before it was overwritten
    /// AppConfig.hpp:287
    m_orig_version: Semver,
    /// Whether the existing version is before system profiles & configuration updating
    /// AppConfig.hpp:289
    m_legacy_datadir: bool,
    /// AppConfig.hpp:291
    m_loading_path: String,
    /// AppConfig.hpp:293
    m_filament_presets: Vec<String>,
    /// AppConfig.hpp:294
    m_filament_colors: Vec<String>,
    /// AppConfig.hpp:295
    m_filament_multi_colors: Vec<String>,
    /// AppConfig.hpp:296
    m_filament_color_types: Vec<String>,
    /// AppConfig.hpp:298
    m_printer_cali_infos: Vec<PrinterCaliInfo>,
}

/// Default implementation for AppConfig, delegates to the constructor.
/// AppConfig.hpp:40-47
impl Default for AppConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl AppConfig {
    // AppConfig.hpp:258-260  static const std::string SECTION_FILAMENTS / SECTION_MATERIALS / SECTION_EMBOSS_STYLE
    // AppConfig.cpp:45-47
    pub const SECTION_FILAMENTS: &'static str = "filaments";
    pub const SECTION_MATERIALS: &'static str = "sla_materials";
    pub const SECTION_EMBOSS_STYLE: &'static str = "font";

    // AppConfig.hpp:40-47  explicit AppConfig() : m_dirty(false), m_orig_version(Semver::invalid()),
    //                                             m_mode(EAppMode::Editor), m_legacy_datadir(false) { this->reset(); }
    pub fn new() -> Self {
        // AppConfig.hpp:41-44
        let mut config = Self {
            m_mode: EAppMode::Editor,
            m_storage: BTreeMap::new(),
            m_vendors: BTreeMap::new(),
            m_dirty: false,
            m_orig_version: Semver::invalid(),
            m_legacy_datadir: false,
            m_loading_path: String::new(),
            m_filament_presets: Vec::new(),
            m_filament_colors: Vec::new(),
            m_filament_multi_colors: Vec::new(),
            m_filament_color_types: Vec::new(),
            m_printer_cali_infos: Vec::new(),
        };
        // AppConfig.hpp:46  this->reset();
        config.reset();
        config
    }

    // AppConfig.cpp:48-63  std::string AppConfig::get_language_code()
    pub fn get_language_code(&self) -> String {
        // AppConfig.cpp:50  std::string get_lang = get("language");
        let mut get_lang = self.get_key("language");
        // AppConfig.cpp:51  if (get_lang.empty()) return "";
        if get_lang.is_empty() {
            return String::new();
        }

        // AppConfig.cpp:53-60
        if get_lang == "zh_CN" {
            // AppConfig.cpp:55
            get_lang = "zh-cn".to_string();
        } else {
            // AppConfig.cpp:59  if (get_lang.length() >= 2) { get_lang = get_lang.substr(0, 2); }
            if get_lang.len() >= 2 {
                get_lang = get_lang[0..2].to_string();
            }
        }

        // AppConfig.cpp:62
        get_lang
    }

    // AppConfig.cpp:65-82  std::string AppConfig::get_hms_host()
    pub fn get_hms_host(&self) -> String {
        // AppConfig.cpp:67  std::string sel = get("iot_environment");
        let _sel = self.get_key("iot_environment");
        // AppConfig.cpp:68  std::string host = "";
        // AppConfig.cpp:69-81  #if !BBL_RELEASE_TO_PUBLIC ... #else return "e.bambulab.com"; #endif
        // BBL_RELEASE_TO_PUBLIC is true, so the dev/qa/pre branches are compiled out.
        // AppConfig.cpp:80
        "e.bambulab.com".to_string()
    }

    // AppConfig.cpp:84-88  void AppConfig::reset()
    pub fn reset(&mut self) {
        // AppConfig.cpp:86  m_storage.clear();
        self.m_storage.clear();
        // AppConfig.cpp:87  set_defaults();
        self.set_defaults();
    }

    // AppConfig.cpp:90-516  void AppConfig::set_defaults()
    // Override missing or keys with their defaults.
    pub fn set_defaults(&mut self) {
        // AppConfig.cpp:93  if (m_mode == EAppMode::Editor) {
        if self.m_mode == EAppMode::Editor {
            // AppConfig.cpp:94-98  #ifdef SUPPORT_AUTO_CENTER ... (not defined)
            // AppConfig.cpp:100-104  #ifdef SUPPORT_BACKGROUND_PROCESSING ... (not defined)
            // AppConfig.cpp:106-109  #ifdef SUPPORT_SHOW_DROP_PROJECT ... (not defined)

            // AppConfig.cpp:111-112
            if self.get_key("drop_project_action").is_empty() {
                self.set_app_bool("drop_project_action", true);
            }

            // AppConfig.cpp:114-122  #ifdef _WIN32 associate_3mf/stl/step (not on this platform)

            // AppConfig.cpp:124-126  remove old 'use_legacy_opengl' parameter from this config, if present
            if !self.get_key("use_legacy_opengl").is_empty() {
                self.erase("app", "use_legacy_opengl");
            }

            // AppConfig.cpp:128-131  #ifdef __APPLE__ use_retina_opengl
            #[cfg(target_os = "macos")]
            {
                if self.get_key("use_retina_opengl").is_empty() {
                    self.set_app_bool("use_retina_opengl", true);
                }
            }

            // AppConfig.cpp:133-134
            if self.get_key("single_instance").is_empty() {
                self.set_app_bool("single_instance", false);
            }
            // AppConfig.cpp:135-136
            if self.get_key("import_3mf_as_project").is_empty() {
                self.set_app_bool("import_3mf_as_project", true);
            }
            // AppConfig.cpp:137-143  #ifdef SUPPORT_REMEMBER_OUTPUT_PATH ... (not defined)
            // AppConfig.cpp:144-145
            if self.get_key("toolkit_size").is_empty() {
                self.set_app("toolkit_size", "100");
            }

            // AppConfig.cpp:147-150  #if ENABLE_ENVIRONMENT_MAP ... (not enabled)

            // AppConfig.cpp:152-153
            if self.get_key("use_inches").is_empty() {
                self.set_app("use_inches", "0");
            }
        } else {
            // AppConfig.cpp:156-159  #ifdef _WIN32 associate_gcode (not on this platform)
        }

        // AppConfig.cpp:162-163
        if self.get_key("use_perspective_camera").is_empty() {
            self.set_app_bool("use_perspective_camera", true);
        }

        // AppConfig.cpp:165-168  #ifdef SUPPORT_FREE_CAMERA ... (not defined)
        // AppConfig.cpp:170-173  #ifdef SUPPORT_REVERSE_MOUSE_ZOOM ... (not defined)

        // AppConfig.cpp:174-175
        if self.get_key("enable_append_color_by_sync_ams").is_empty() {
            self.set_app_bool("enable_append_color_by_sync_ams", true);
        }
        // AppConfig.cpp:176-177
        if self.get_key("enable_merge_color_by_sync_ams").is_empty() {
            self.set_app_bool("enable_merge_color_by_sync_ams", false);
        }
        // AppConfig.cpp:178-179
        if self.get_key("ams_sync_match_full_use_color_dist").is_empty() {
            self.set_app_bool("ams_sync_match_full_use_color_dist", false);
        }
        // AppConfig.cpp:180-181
        if self.get_key("enable_sidebar_floatable").is_empty() {
            self.set_app_bool("enable_sidebar_floatable", false);
        }

        // AppConfig.cpp:183-184
        if self.get_key("export_sources_full_pathnames").is_empty() {
            self.set_app_bool("export_sources_full_pathnames", false);
        }

        // AppConfig.cpp:186-187
        if self.get_key("zoom_to_mouse").is_empty() {
            self.set_app_bool("zoom_to_mouse", false);
        }
        // AppConfig.cpp:188-189
        if self.get_key("show_shells_in_preview").is_empty() {
            self.set_app_bool("show_shells_in_preview", true);
        }
        // AppConfig.cpp:190-191
        if self.get_key("enable_text_styles").is_empty() {
            self.set_app_bool("enable_text_styles", false);
        }
        // AppConfig.cpp:192-193
        if self.get_key("use_last_fold_state_gcodeview_option_panel").is_empty() {
            self.set_app_bool("use_last_fold_state_gcodeview_option_panel", true);
        }
        // AppConfig.cpp:194-195
        if self.get_key("enable_lod").is_empty() {
            self.set_app_bool("enable_lod", true);
        }
        // AppConfig.cpp:196-197
        if self.get_key("enable_assemble_view_preview").is_empty() {
            self.set_app("enable_assemble_view_preview", "Auto");
        }
        // AppConfig.cpp:198-199
        if self.get_key("enable_bvh").is_empty() {
            self.set_app_bool("enable_bvh", true);
        }
        // AppConfig.cpp:200-201
        if self.get_key("show_assembly_bvh_bounds").is_empty() {
            self.set_app_bool("show_assembly_bvh_bounds", false);
        }
        // AppConfig.cpp:202-203
        if self.get_key("gamma_correct_in_import_obj").is_empty() {
            self.set_app_bool("gamma_correct_in_import_obj", false);
        }
        // AppConfig.cpp:204-205
        if self.get_key("enable_opengl_multi_instance").is_empty() {
            self.set_app_bool("enable_opengl_multi_instance", true);
        }
        // AppConfig.cpp:206-207
        if self.get_key("import_single_svg_and_split").is_empty() {
            self.set_app_bool("import_single_svg_and_split", true);
        }
        // AppConfig.cpp:208-209
        if self.get_key("user_bed_type").is_empty() {
            self.set_app_bool("user_bed_type", true);
        }
        // AppConfig.cpp:210-211
        if self.get_key("grabber_size_factor").is_empty() {
            self.set_app("grabber_size_factor", "1.0");
        }
        // AppConfig.cpp:212-213
        if self.get_key("3d_middle_tooltip_offset_x").is_empty() {
            self.set_app("3d_middle_tooltip_offset_x", "0.0");
        }
        // AppConfig.cpp:214-215
        if self.get_key("3d_middle_tooltip_offset_y").is_empty() {
            self.set_app("3d_middle_tooltip_offset_y", "0.0");
        }
        // AppConfig.cpp:216-217
        if self.get_key("cancel_glmultidraw").is_empty() {
            self.set_app_bool("cancel_glmultidraw", false);
        }
        // AppConfig.cpp:218-221  //#ifdef SUPPORT_SHOW_HINTS
        if self.get_key("show_hints").is_empty() {
            self.set_app_bool("show_hints", false);
        }
        // AppConfig.cpp:222-223
        if self.get_key("support_backup_fonts").is_empty() {
            self.set_app_bool("support_backup_fonts", true);
        }
        // AppConfig.cpp:224-225
        if self.get_key("custom_back_font_name").is_empty() {
            self.set_app("custom_back_font_name", "");
        }
        // AppConfig.cpp:226-227
        if self.get_key("enable_multi_machine").is_empty() {
            self.set_app_bool("enable_multi_machine", false);
        }

        // AppConfig.cpp:229-230
        if self.get_key("enable_record_gcodeviewer_option_item").is_empty() {
            self.set_app_bool("enable_record_gcodeviewer_option_item", false);
        }
        // AppConfig.cpp:231-232
        if self.get_key("prefer_to_use_dgpu").is_empty() {
            self.set_app_bool("prefer_to_use_dgpu", false);
        }

        // AppConfig.cpp:234-235
        if self.get_key("msaa_type").is_empty() {
            self.set_app("msaa_type", "X4");
        }

        // AppConfig.cpp:237-238
        if self.get_key("enable_advanced_antialiasing").is_empty() {
            self.set_app_bool("enable_advanced_antialiasing", false);
        }

        // AppConfig.cpp:240-241
        if self.get_key("enable_advanced_gcode_viewer_").is_empty() {
            self.set_app_bool("enable_advanced_gcode_viewer_", true);
        }

        // AppConfig.cpp:243-244
        if self.get_key("gizmo_keep_screen_size").is_empty() {
            self.set_app_bool("gizmo_keep_screen_size", true);
        }

        // AppConfig.cpp:246-247
        if self.get_key("show_3d_navigator").is_empty() {
            self.set_app_bool("show_3d_navigator", true);
        }

        // AppConfig.cpp:249-265  #ifdef _WIN32 use_legacy_3DConnexion / dark_color_mode / sys_menu_enabled (not on this platform)

        // AppConfig.cpp:267-269  BBS /* 3mf_include_gcode (commented out) */

        // AppConfig.cpp:271-272
        if self.get_key("developer_mode").is_empty() {
            self.set_app_bool("developer_mode", false);
        }

        // AppConfig.cpp:274-275
        if self.get_key("enable_ssl_for_mqtt").is_empty() {
            self.set_app_bool("enable_ssl_for_mqtt", true);
        }

        // AppConfig.cpp:277-278
        if self.get_key("enable_ssl_for_ftp").is_empty() {
            self.set_app_bool("enable_ssl_for_ftp", true);
        }

        // AppConfig.cpp:280-281
        if self.get_key("severity_level").is_empty() {
            self.set_app("severity_level", "info");
        }

        // AppConfig.cpp:283-284
        if self.get_key("internal_developer_mode").is_empty() {
            self.set_app_bool("internal_developer_mode", false);
        }

        // AppConfig.cpp:286-287
        if self.get_key("disable_auto_flow_cali_tips").is_empty() {
            self.set_app_bool("disable_auto_flow_cali_tips", false);
        }

        // AppConfig.cpp:289-291  BBS
        if self.get_key("preset_folder").is_empty() {
            self.set_app("preset_folder", "");
        }

        // AppConfig.cpp:293-297  BBS
        if self.get_key("slicer_uuid").is_empty() {
            // boost::uuids::uuid uuid = boost::uuids::random_generator()();
            let uuid = random_uuid_string();
            // set("slicer_uuid", to_string(uuid));
            self.set_app("slicer_uuid", &uuid);
        }

        // AppConfig.cpp:299-301
        if self.get_key("show_model_mesh").is_empty() {
            self.set_app_bool("show_model_mesh", false);
        }

        // AppConfig.cpp:303-305
        if self.get_key("show_model_shadow").is_empty() {
            self.set_app_bool("show_model_shadow", true);
        }

        // AppConfig.cpp:307-309  (NB: C++ checks "show_build_edges" but sets "show_build_edgets" — preserved verbatim)
        if self.get_key("show_build_edges").is_empty() {
            self.set_app_bool("show_build_edgets", false);
        }

        // AppConfig.cpp:311-313
        if self.get_key("show_daily_tips").is_empty() {
            self.set_app_bool("show_daily_tips", true);
        }

        // AppConfig.cpp:315-317
        if self.get_key("auto_calculate_flush").is_empty() {
            self.set_app("auto_calculate_flush", "all");
        }

        // AppConfig.cpp:319-321
        if self.get_key("enable_high_low_temp_mixed_printing").is_empty() {
            self.set_app_bool("enable_high_low_temp_mixed_printing", false);
        }

        // AppConfig.cpp:323-325
        if self.get_key("ignore_ext_filament_in_filament_map").is_empty() {
            self.set_app_bool("ignore_ext_filament_in_filament_map", false);
        }

        // AppConfig.cpp:327-329
        if self.get_key("pop_up_filament_map_dialog").is_empty() {
            self.set_app_bool("pop_up_filament_map_dialog", false);
        }

        // AppConfig.cpp:331-333  set("prefered_filament_map_mode", get_enum_names()[fmmAutoForFlush]);
        if self.get_key("prefered_filament_map_mode").is_empty() {
            self.set_app("prefered_filament_map_mode", FMM_AUTO_FOR_FLUSH_NAME);
        }

        // AppConfig.cpp:335-337
        if self.get_key("show_home_page").is_empty() {
            self.set_app_bool("show_home_page", true);
        }

        // AppConfig.cpp:339-341
        if self.get_key("show_print_history").is_empty() {
            self.set_app_bool("show_print_history", true);
        }

        // AppConfig.cpp:343-345
        if self.get_key("show_printable_box").is_empty() {
            self.set_app_bool("show_printable_box", true);
        }

        // AppConfig.cpp:347-349
        if self.get_key("units").is_empty() {
            self.set_app("units", "0");
        }

        // AppConfig.cpp:351-353
        if self.get_key("auto_transfer_when_switch_preset").is_empty() {
            self.set_app("auto_transfer_when_switch_preset", "true");
        }

        // AppConfig.cpp:355-357
        if self.get_key("sync_user_preset").is_empty() {
            self.set_app_bool("sync_user_preset", false);
        }

        // AppConfig.cpp:359-361
        if self.get_key("keyboard_supported").is_empty() {
            self.set_app("keyboard_supported", "none/alt/control/shift");
        }

        // AppConfig.cpp:363-365
        if self.get_key("mouse_supported").is_empty() {
            self.set_app("mouse_supported", "mouse left/mouse middle/mouse right");
        }

        // AppConfig.cpp:367-369
        if self.get_key("privacy_version").is_empty() {
            self.set_app("privacy_version", "00.00.00.00");
        }

        // AppConfig.cpp:371-373
        if self.get_key("rotate_view").is_empty() {
            self.set_app("rotate_view", "none/mouse left");
        }

        // AppConfig.cpp:375-377
        if self.get_key("move_view").is_empty() {
            self.set_app("move_view", "none/mouse left");
        }

        // AppConfig.cpp:379-381
        if self.get_key("zoom_view").is_empty() {
            self.set_app("zoom_view", "none/mouse left");
        }

        // AppConfig.cpp:383-385
        if self.get_key("precise_control").is_empty() {
            self.set_app("precise_control", "none/mouse left");
        }

        // AppConfig.cpp:387-389
        if self.get_key("download_path").is_empty() {
            self.set_app("download_path", "");
        }

        // AppConfig.cpp:391-392
        if self.get_key("mouse_wheel").is_empty() {
            self.set_app("mouse_wheel", "0");
        }

        // AppConfig.cpp:394-397  helio options
        if self.get_key("helio_enable").is_empty() {
            self.set_app_bool("helio_enable", false);
        }

        // AppConfig.cpp:399-401
        if self.get_key("helio_api_china").is_empty() {
            self.set_app("helio_api_china", "https://api.helioam.cn/graphql");
        }

        // AppConfig.cpp:403-405
        if self.get_key("helio_api_other").is_empty() {
            self.set_app("helio_api_other", "https://api.helioadditive.com/graphql");
        }

        // AppConfig.cpp:407-409
        if self.get_key("max_recent_count").is_empty() {
            self.set_app("max_recent_count", "18");
        }

        // AppConfig.cpp:411-413
        if self.get_key("staff_pick_switch").is_empty() {
            self.set_app_bool("staff_pick_switch", true);
        }

        // AppConfig.cpp:415-417
        if self.get_key("sync_system_preset").is_empty() {
            self.set_app_bool("sync_system_preset", true);
        }

        // AppConfig.cpp:419-421  (string compare, not semver)
        if self.get_key("backup_switch").is_empty() || self.get_key("version").as_str() < "01.06.00.00" {
            self.set_app_bool("backup_switch", true);
        }

        // AppConfig.cpp:423-425
        if self.get("liveview", "auto_stop_liveview").is_empty() {
            self.set_bool("liveview", "auto_stop_liveview", true);
        }

        // AppConfig.cpp:427-429
        if self.get_key("backup_interval").is_empty() {
            self.set_app("backup_interval", "10");
        }

        // AppConfig.cpp:431-433
        if self.get_key("curr_bed_type").is_empty() {
            self.set_app("curr_bed_type", "1");
        }

        // AppConfig.cpp:435-437
        if self.get_key("sending_interval").is_empty() {
            self.set_app("sending_interval", "5");
        }

        // AppConfig.cpp:439-441
        if self.get_key("max_send").is_empty() {
            self.set_app("max_send", "3");
        }

        // AppConfig.cpp:443-451  #if BBL_RELEASE_TO_PUBLIC iot_environment="3" #else "2"
        if BBL_RELEASE_TO_PUBLIC {
            // AppConfig.cpp:444-446
            if self.get_key("iot_environment").is_empty() {
                self.set_app("iot_environment", "3");
            }
        } else {
            // AppConfig.cpp:448-450
            if self.get_key("iot_environment").is_empty() {
                self.set_app("iot_environment", "2");
            }
        }

        // AppConfig.cpp:453-455
        if self.get("print", "bed_leveling").is_empty() {
            self.set_str("print", "bed_leveling", "1");
        }
        // AppConfig.cpp:456-458
        if self.get("print", "flow_cali").is_empty() {
            self.set_str("print", "flow_cali", "1");
        }
        // AppConfig.cpp:459-461
        if self.get("print", "timelapse").is_empty() {
            self.set_str("print", "timelapse", "1");
        }

        // AppConfig.cpp:463-465
        if self.get_key("enable_step_mesh_setting").is_empty() {
            self.set_app_bool("enable_step_mesh_setting", true);
        }
        // AppConfig.cpp:466-468
        if self.get_key("enable_beta_version_update").is_empty() {
            self.set_app_bool("enable_beta_version_update", true);
        }
        // AppConfig.cpp:469-471
        if self.get_key("linear_defletion").is_empty() {
            self.set_app("linear_defletion", "0.003");
        }
        // AppConfig.cpp:472-474
        if self.get_key("angle_defletion").is_empty() {
            self.set_app("angle_defletion", "0.5");
        }
        // AppConfig.cpp:475-477
        if self.get_key("is_split_compound").is_empty() {
            self.set_app_bool("is_split_compound", false);
        }
        // AppConfig.cpp:478-480
        if self.get_key("play_slicing_video").is_empty() {
            self.set_app_bool("play_slicing_video", true);
        }
        // AppConfig.cpp:481-483
        if self.get_key("show_fila_switch_tips").is_empty() {
            self.set_app_bool("show_fila_switch_tips", true);
        }
        // AppConfig.cpp:484-486
        if self.get_key("play_tpu_printing_video").is_empty() {
            self.set_app_bool("play_tpu_printing_video", true);
        }
        // AppConfig.cpp:487-489
        if self.get_key("show_wrapping_detect_dialog").is_empty() {
            self.set_app_bool("show_wrapping_detect_dialog", true);
        }
        // AppConfig.cpp:490-492
        if self.get_key("show_support_recommend_dialog").is_empty() {
            self.set_app_bool("show_support_recommend_dialog", true);
        }
        // AppConfig.cpp:493-495
        if self.get_key("ignore_module_cert").is_empty() {
            self.set_app_bool("ignore_module_cert", false);
        }
        // AppConfig.cpp:496-498
        if self.get_key("webview_auto_fill").is_empty() {
            self.set_app_bool("webview_auto_fill", true);
        }
        // AppConfig.cpp:499
        self.erase("app", "item_webview_auto_fill");

        // AppConfig.cpp:501-503
        if self.get_key("prompt_for_brittle_filaments").is_empty() {
            self.set_app_bool("prompt_for_brittle_filaments", true);
        }

        // AppConfig.cpp:505-507
        if self.get_key("use_12h_time_format").is_empty() {
            self.set_app_bool("use_12h_time_format", false);
        }

        // AppConfig.cpp:509-515  Remove legacy window positions/sizes
        self.erase("app", "main_frame_maximized");
        self.erase("app", "main_frame_pos");
        self.erase("app", "main_frame_size");
        self.erase("app", "object_settings_maximized");
        self.erase("app", "object_settings_pos");
        self.erase("app", "object_settings_size");
    }

    // AppConfig.cpp:567-789  std::string AppConfig::load()  (USE_JSON_CONFIG branch)
    //
    // 1) Read the complete config file and parse it as JSON.
    // The WIN32-only backup/MD5 recovery path (AppConfig.cpp:585-643) is dead
    // code off-Windows; this port follows the non-WIN32 `ifs >> j;` path
    // (AppConfig.cpp:607).
    pub fn load(&mut self) -> String {
        // AppConfig.cpp:569  json j;
        // AppConfig.cpp:578-609  try { open + parse }
        let path = self.loading_path();
        let contents = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(_) => {
                // AppConfig.cpp:581-584  if (!ifs.is_open()) return "Line break format may be incorrect.";
                return "Line break format may be incorrect.".to_string();
            }
        };
        // AppConfig.cpp:607  ifs >> j;
        let j: serde_json::Value = match serde_json::from_str(&contents) {
            Ok(v) => v,
            // AppConfig.cpp:610-650  catch(parse_error) — off-Windows just logs and returns err.what().
            Err(err) => {
                return err.to_string();
            }
        };

        // AppConfig.cpp:652-760  try { iterate over the JSON object }
        if let Some(obj) = j.as_object() {
            for (key, value) in obj.iter() {
                // AppConfig.cpp:654  if (it.key() == MODELS_STR)
                if key == MODELS_STR {
                    // AppConfig.cpp:655-665
                    if let Some(arr) = value.as_array() {
                        for j_model in arr {
                            // AppConfig.cpp:657  const auto vendor_name = j_model["vendor"].get<std::string>();
                            let vendor_name = json_get_str(j_model, "vendor");
                            // AppConfig.cpp:658  auto& vendor = m_vendors[vendor_name];
                            let vendor = self.m_vendors.entry(vendor_name).or_default();
                            // AppConfig.cpp:659  const auto model_name = j_model["model"].get<std::string>();
                            let model_name = json_get_str(j_model, "model");
                            // AppConfig.cpp:660  std::vector<std::string> variants;
                            let mut variants: Vec<String> = Vec::new();
                            // AppConfig.cpp:661  if (!unescape_strings_cstyle(j_model["nozzle_diameter"], variants)) continue;
                            let nozzle_diameter = json_get_str(j_model, "nozzle_diameter");
                            if !unescape_strings_cstyle(&nozzle_diameter, &mut variants) {
                                continue;
                            }
                            // AppConfig.cpp:662-664
                            for variant in variants {
                                vendor.entry(model_name.clone()).or_default().insert(variant);
                            }
                        }
                    }
                } else if key == Self::SECTION_FILAMENTS {
                    // AppConfig.cpp:666-670
                    if let Some(arr) = value.as_array() {
                        for element in arr {
                            if let Some(s) = element.as_str() {
                                self.m_storage
                                    .entry(key.clone())
                                    .or_default()
                                    .insert(s.to_string(), "true".to_string());
                            }
                        }
                    }
                } else if key == "presets" {
                    // AppConfig.cpp:671-688
                    if let Some(presets_obj) = value.as_object() {
                        for (iter_key, iter_value) in presets_obj.iter() {
                            // AppConfig.cpp:673  if (iter.key() == "filaments")
                            if iter_key == "filaments" {
                                // AppConfig.cpp:674-684
                                let mut idx = 0i32;
                                if let Some(arr) = iter_value.as_array() {
                                    for element in arr {
                                        let element_s = element.as_str().unwrap_or("").to_string();
                                        if idx == 0 {
                                            // AppConfig.cpp:677
                                            self.m_storage
                                                .entry(key.clone())
                                                .or_default()
                                                .insert("filament".to_string(), element_s);
                                        } else {
                                            // AppConfig.cpp:679-681
                                            let mut n = idx.to_string();
                                            if n.len() == 1 {
                                                n = format!("0{}", n);
                                            }
                                            self.m_storage
                                                .entry(key.clone())
                                                .or_default()
                                                .insert(format!("filament_{}", n), element_s);
                                        }
                                        idx += 1;
                                    }
                                }
                            } else {
                                // AppConfig.cpp:686  m_storage[it.key()][iter.key()] = iter.value().get<std::string>();
                                let v = iter_value.as_str().unwrap_or("").to_string();
                                self.m_storage
                                    .entry(key.clone())
                                    .or_default()
                                    .insert(iter_key.clone(), v);
                            }
                        }
                    }
                } else if key == "calis" {
                    // AppConfig.cpp:689-727
                    if let Some(arr) = value.as_array() {
                        for calis_j in arr {
                            // AppConfig.cpp:691  PrinterCaliInfo cali_info;
                            let mut cali_info = PrinterCaliInfo::default();
                            // AppConfig.cpp:692-693
                            if let Some(v) = calis_j.get("dev_id") {
                                cali_info.dev_id = v.as_str().unwrap_or("").to_string();
                            }
                            // AppConfig.cpp:694-695  cali_info.cali_finished = bool(int);
                            if let Some(v) = calis_j.get("cali_finished") {
                                cali_info.cali_finished = json_as_i64(v) != 0;
                            }
                            // AppConfig.cpp:696-697  cali_info.cache_flow_ratio = float;
                            if let Some(v) = calis_j.get("flow_ratio") {
                                cali_info.cache_flow_ratio = json_as_f64(v) as f32;
                            }
                            // AppConfig.cpp:698-699  cache_flow_rate_calibration_type = FlowRatioCalibrationType(int);
                            if let Some(v) = calis_j.get("cache_flow_rate_calibration_type") {
                                cali_info.cache_flow_rate_calibration_type =
                                    flow_ratio_calibration_type_from_int(json_as_i64(v) as i32);
                            }
                            // AppConfig.cpp:700-725
                            if let Some(presets_v) = calis_j.get("presets") {
                                // AppConfig.cpp:701  cali_info.selected_presets.clear();
                                cali_info.selected_presets.clear();
                                if let Some(presets_arr) = presets_v.as_array() {
                                    for cali_it in presets_arr {
                                        // AppConfig.cpp:703  CaliPresetInfo preset_info;
                                        let mut preset_info = CaliPresetInfo::default();
                                        // AppConfig.cpp:704
                                        preset_info.tray_id = json_get_i64(cali_it, "tray_id") as i32;
                                        // AppConfig.cpp:705
                                        preset_info.nozzle_diameter = json_get_f64(cali_it, "nozzle_diameter") as f32;
                                        // AppConfig.cpp:706
                                        preset_info.filament_id = json_get_str(cali_it, "filament_id");
                                        // AppConfig.cpp:707
                                        preset_info.setting_id = json_get_str(cali_it, "setting_id");
                                        // AppConfig.cpp:708
                                        preset_info.name = json_get_str(cali_it, "name");
                                        // AppConfig.cpp:709-710
                                        if let Some(v) = cali_it.get("extruder_id") {
                                            preset_info.extruder_id = json_as_i64(v) as usize;
                                        }
                                        // AppConfig.cpp:711-712  NozzleVolumeType(int)
                                        if let Some(v) = cali_it.get("nozzle_volume_type") {
                                            preset_info.nozzle_volume_type =
                                                nozzle_volume_type_from_int(json_as_i64(v) as i32);
                                        }
                                        // AppConfig.cpp:713-714  BedType(int)
                                        if let Some(v) = cali_it.get("bed_type") {
                                            preset_info.bed_type = bed_type_from_int(json_as_i64(v) as i32);
                                        }
                                        // AppConfig.cpp:715-716
                                        if let Some(v) = cali_it.get("nozzle_pos_id") {
                                            preset_info.nozzle_pos_id = json_as_i64(v) as i32;
                                        }
                                        // AppConfig.cpp:717-718
                                        if let Some(v) = cali_it.get("nozzle_sn") {
                                            preset_info.nozzle_sn = v.as_str().unwrap_or("").to_string();
                                        }
                                        // AppConfig.cpp:719
                                        preset_info.nozzle_diameter = json_get_f64(cali_it, "nozzle_diameter") as f32;
                                        // AppConfig.cpp:720
                                        preset_info.filament_id = json_get_str(cali_it, "filament_id");
                                        // AppConfig.cpp:721
                                        preset_info.setting_id = json_get_str(cali_it, "setting_id");
                                        // AppConfig.cpp:722
                                        preset_info.name = json_get_str(cali_it, "name");
                                        // AppConfig.cpp:723
                                        cali_info.selected_presets.push(preset_info);
                                    }
                                }
                            }
                            // AppConfig.cpp:726  m_printer_cali_infos.emplace_back(cali_info);
                            self.m_printer_cali_infos.push(cali_info);
                        }
                    }
                } else {
                    // AppConfig.cpp:728-754  else { if (it.value().is_object()) ... }
                    if let Some(inner) = value.as_object() {
                        for (iter_key, iter_value) in inner.iter() {
                            // AppConfig.cpp:731-736  is_boolean
                            if let Some(b) = iter_value.as_bool() {
                                if b {
                                    self.m_storage
                                        .entry(key.clone())
                                        .or_default()
                                        .insert(iter_key.clone(), "true".to_string());
                                } else {
                                    self.m_storage
                                        .entry(key.clone())
                                        .or_default()
                                        .insert(iter_key.clone(), "false".to_string());
                                }
                            } else if iter_key == "filament_presets" {
                                // AppConfig.cpp:737-738
                                self.m_filament_presets = json_string_vec(iter_value);
                            } else if iter_key == "filament_colors" {
                                // AppConfig.cpp:739-740
                                self.m_filament_colors = json_string_vec(iter_value);
                            } else if iter_key == "filament_multi_colors" {
                                // AppConfig.cpp:741-742
                                self.m_filament_multi_colors = json_string_vec(iter_value);
                            } else if iter_key == "filament_color_types" {
                                // AppConfig.cpp:743-744
                                self.m_filament_color_types = json_string_vec(iter_value);
                            } else {
                                // AppConfig.cpp:745-751
                                if let Some(s) = iter_value.as_str() {
                                    self.m_storage
                                        .entry(key.clone())
                                        .or_default()
                                        .insert(iter_key.clone(), s.to_string());
                                } else {
                                    // AppConfig.cpp:749  BOOST_LOG_TRIVIAL(warning) << "load config warning...";
                                }
                            }
                        }
                    }
                }
            }
        }

        // AppConfig.cpp:762-769  Figure out if datadir has legacy presets
        let ini_ver = Semver::parse(&self.get_key("version"));
        // AppConfig.cpp:764  m_legacy_datadir = false;
        self.m_legacy_datadir = false;
        // AppConfig.cpp:765-769  if (ini_ver) { ... }
        if let Some(mut ini_ver) = ini_ver {
            // AppConfig.cpp:767-768
            ini_ver.set_metadata(None);
            ini_ver.set_prerelease(None);
            // AppConfig.cpp:766  m_orig_version = *ini_ver;  (note: C++ assigns the unmodified version)
            self.m_orig_version = ini_ver;
        }

        // AppConfig.cpp:771-783  Legacy conversion
        if self.m_mode == EAppMode::Editor {
            // AppConfig.cpp:775  if (auto it_section = m_storage.find("extras"); ...)
            if self.m_storage.contains_key("extras") {
                // AppConfig.cpp:776-779
                let physical_printer = self
                    .m_storage
                    .get("extras")
                    .and_then(|s| s.get("physical_printer"))
                    .cloned();
                if let Some(pp) = physical_printer {
                    // AppConfig.cpp:777  m_storage["presets"]["physical_printer"] = it_physical_printer->second;
                    self.m_storage
                        .entry("presets".to_string())
                        .or_default()
                        .insert("physical_printer".to_string(), pp);
                    // AppConfig.cpp:778  it_section->second.erase(it_physical_printer);
                    if let Some(extras) = self.m_storage.get_mut("extras") {
                        extras.remove("physical_printer");
                    }
                }
                // AppConfig.cpp:780-781  if (it_section->second.empty()) m_storage.erase(it_section);
                if self.m_storage.get("extras").map(|s| s.is_empty()).unwrap_or(false) {
                    self.m_storage.remove("extras");
                }
            }
        }

        // AppConfig.cpp:785-788
        // Override missing or keys with their defaults.
        self.set_defaults();
        self.m_dirty = false;
        String::new()
    }

    // AppConfig.cpp:791-949  void AppConfig::save()  (USE_JSON_CONFIG branch)
    //
    // Native dependencies (`is_main_thread_active`, `boost::nowide::ofstream`,
    // WIN32 MD5/backup) are off the wasm/parity path; this builds the same JSON
    // document and writes it to a PID-suffixed temp file, then renames it.
    pub fn save(&mut self) {
        // AppConfig.cpp:793-794  if (!is_main_thread_active()) throw CriticalException(...)
        // (not modeled; CLI runs save() on its main thread)

        // AppConfig.cpp:798  const auto path = config_path();
        let path = self.config_path();
        // AppConfig.cpp:799  std::string path_pid = (boost::format("%1%.%2%") % path % get_current_pid()).str();
        let path_pid = format!("{}.{}", path, get_current_pid());

        // AppConfig.cpp:801  json j;
        let mut j = serde_json::Map::new();

        // AppConfig.cpp:804-807
        if self.m_mode == EAppMode::Editor {
            j.insert("header".to_string(), serde_json::Value::String(header_slic3r_generated()));
        } else {
            j.insert(
                "header".to_string(),
                serde_json::Value::String(header_gcodeviewer_generated()),
            );
        }

        // AppConfig.cpp:809-820  Make sure the "no" category is written first.  ("app" section)
        {
            let mut app_obj = serde_json::Map::new();
            if let Some(app) = self.m_storage.get("app") {
                for (k, v) in app.iter() {
                    if v == "true" {
                        app_obj.insert(k.clone(), serde_json::Value::Bool(true));
                        continue;
                    }
                    if v == "false" {
                        app_obj.insert(k.clone(), serde_json::Value::Bool(false));
                        continue;
                    }
                    app_obj.insert(k.clone(), serde_json::Value::String(v.clone()));
                }
            }
            // AppConfig.cpp:822-824  filament_presets
            if !self.m_filament_presets.is_empty() {
                app_obj.insert("filament_presets".to_string(), str_vec_to_json(&self.m_filament_presets));
            }
            // AppConfig.cpp:826-828  filament_colors
            if !self.m_filament_colors.is_empty() {
                app_obj.insert("filament_colors".to_string(), str_vec_to_json(&self.m_filament_colors));
            }
            // AppConfig.cpp:829-831  filament_multi_colors
            if !self.m_filament_multi_colors.is_empty() {
                app_obj.insert(
                    "filament_multi_colors".to_string(),
                    str_vec_to_json(&self.m_filament_multi_colors),
                );
            }
            // AppConfig.cpp:833-835  filament_color_types
            if !self.m_filament_color_types.is_empty() {
                app_obj.insert(
                    "filament_color_types".to_string(),
                    str_vec_to_json(&self.m_filament_color_types),
                );
            }
            j.insert("app".to_string(), serde_json::Value::Object(app_obj));
        }

        // AppConfig.cpp:837-858  calis
        if !self.m_printer_cali_infos.is_empty() {
            let mut calis_arr: Vec<serde_json::Value> = Vec::new();
            for cali_info in &self.m_printer_cali_infos {
                // AppConfig.cpp:838  json cali_json;
                let mut cali_json = serde_json::Map::new();
                // AppConfig.cpp:839
                cali_json.insert("dev_id".to_string(), serde_json::Value::String(cali_info.dev_id.clone()));
                // AppConfig.cpp:840
                cali_json.insert("flow_ratio".to_string(), json_f32(cali_info.cache_flow_ratio));
                // AppConfig.cpp:841  cali_info.cali_finished ? 1 : 0
                cali_json.insert(
                    "cali_finished".to_string(),
                    serde_json::Value::from(if cali_info.cali_finished { 1 } else { 0 }),
                );
                // AppConfig.cpp:842
                cali_json.insert(
                    "cache_flow_rate_calibration_type".to_string(),
                    serde_json::Value::from(flow_ratio_calibration_type_to_int(
                        cali_info.cache_flow_rate_calibration_type,
                    )),
                );
                // AppConfig.cpp:843-856
                let mut presets_arr: Vec<serde_json::Value> = Vec::new();
                for filament_preset in &cali_info.selected_presets {
                    // AppConfig.cpp:844  json preset_json;
                    let mut preset_json = serde_json::Map::new();
                    // AppConfig.cpp:845
                    preset_json.insert("tray_id".to_string(), serde_json::Value::from(filament_preset.tray_id));
                    // AppConfig.cpp:846
                    preset_json.insert(
                        "extruder_id".to_string(),
                        serde_json::Value::from(filament_preset.extruder_id),
                    );
                    // AppConfig.cpp:847  int(nozzle_volume_type)
                    preset_json.insert(
                        "nozzle_volume_type".to_string(),
                        serde_json::Value::from(nozzle_volume_type_to_int(filament_preset.nozzle_volume_type)),
                    );
                    // AppConfig.cpp:848  int(bed_type)
                    preset_json.insert(
                        "bed_type".to_string(),
                        serde_json::Value::from(bed_type_to_int(filament_preset.bed_type)),
                    );
                    // AppConfig.cpp:849
                    preset_json.insert(
                        "nozzle_diameter".to_string(),
                        json_f32(filament_preset.nozzle_diameter),
                    );
                    // AppConfig.cpp:850
                    preset_json.insert(
                        "nozzle_pos_id".to_string(),
                        serde_json::Value::from(filament_preset.nozzle_pos_id),
                    );
                    // AppConfig.cpp:851
                    preset_json.insert(
                        "nozzle_sn".to_string(),
                        serde_json::Value::String(filament_preset.nozzle_sn.clone()),
                    );
                    // AppConfig.cpp:852
                    preset_json.insert(
                        "filament_id".to_string(),
                        serde_json::Value::String(filament_preset.filament_id.clone()),
                    );
                    // AppConfig.cpp:853
                    preset_json.insert(
                        "setting_id".to_string(),
                        serde_json::Value::String(filament_preset.setting_id.clone()),
                    );
                    // AppConfig.cpp:854
                    preset_json.insert("name".to_string(), serde_json::Value::String(filament_preset.name.clone()));
                    // AppConfig.cpp:855  cali_json["presets"].push_back(preset_json);
                    presets_arr.push(serde_json::Value::Object(preset_json));
                }
                cali_json.insert("presets".to_string(), serde_json::Value::Array(presets_arr));
                // AppConfig.cpp:857  j["calis"].push_back(cali_json);
                calis_arr.push(serde_json::Value::Object(cali_json));
            }
            j.insert("calis".to_string(), serde_json::Value::Array(calis_arr));
        }

        // AppConfig.cpp:860-901  Write the other categories.
        for (cat_name, category) in self.m_storage.iter() {
            // AppConfig.cpp:862-863  if (category.first.empty()) continue;
            if cat_name.is_empty() {
                continue;
            }
            // The "app" section was already written above.
            if cat_name == "app" {
                continue;
            }
            // AppConfig.cpp:864-870  SECTION_FILAMENTS
            if cat_name == Self::SECTION_FILAMENTS {
                // AppConfig.cpp:865-868
                let mut j_filaments: Vec<serde_json::Value> = Vec::new();
                for (k, _v) in category.iter() {
                    j_filaments.push(serde_json::Value::String(k.clone()));
                }
                // AppConfig.cpp:869
                j.insert(cat_name.clone(), serde_json::Value::Array(j_filaments));
                continue;
            } else if cat_name == "presets" {
                // AppConfig.cpp:871-889
                let mut presets_obj = serde_json::Map::new();
                let mut j_filament_array: Vec<serde_json::Value> = Vec::new();
                for (k, v) in category.iter() {
                    // AppConfig.cpp:872-878  is_filament_preset_key
                    if is_filament_preset_key(k) {
                        // AppConfig.cpp:881
                        j_filament_array.push(serde_json::Value::String(v.clone()));
                    } else {
                        // AppConfig.cpp:883-884
                        presets_obj.insert(k.clone(), serde_json::Value::String(v.clone()));
                    }
                }
                // AppConfig.cpp:887  j["presets"]["filaments"] = j_filament_array;
                presets_obj.insert("filaments".to_string(), serde_json::Value::Array(j_filament_array));
                j.insert(cat_name.clone(), serde_json::Value::Object(presets_obj));
                continue;
            }
            // AppConfig.cpp:890-900
            let mut cat_obj = serde_json::Map::new();
            for (k, v) in category.iter() {
                if v == "true" {
                    cat_obj.insert(k.clone(), serde_json::Value::Bool(true));
                    continue;
                }
                if v == "false" {
                    cat_obj.insert(k.clone(), serde_json::Value::Bool(false));
                    continue;
                }
                cat_obj.insert(k.clone(), serde_json::Value::String(v.clone()));
            }
            j.insert(cat_name.clone(), serde_json::Value::Object(cat_obj));
        }

        // AppConfig.cpp:903-920  Write vendor sections
        {
            let mut models_arr: Vec<serde_json::Value> = Vec::new();
            for (vendor_name, vendor) in self.m_vendors.iter() {
                // AppConfig.cpp:905-907
                let mut size_sum = 0usize;
                for (_model_name, model) in vendor.iter() {
                    size_sum += model.len();
                }
                if size_sum == 0 {
                    continue;
                }
                // AppConfig.cpp:909-919
                for (model_name, model) in vendor.iter() {
                    // AppConfig.cpp:910  if (model.second.empty()) continue;
                    if model.is_empty() {
                        continue;
                    }
                    // AppConfig.cpp:911  const std::vector<std::string> variants(model.second.begin(), model.second.end());
                    // std::set iterates in sorted order; reproduce it.
                    let mut variants: Vec<String> = model.iter().cloned().collect();
                    variants.sort();
                    // AppConfig.cpp:912  const auto escaped = escape_strings_cstyle(variants);
                    let escaped = escape_strings_cstyle(&variants);
                    // AppConfig.cpp:914-918
                    let mut j_model = serde_json::Map::new();
                    j_model.insert("vendor".to_string(), serde_json::Value::String(vendor_name.clone()));
                    j_model.insert("model".to_string(), serde_json::Value::String(model_name.clone()));
                    j_model.insert("nozzle_diameter".to_string(), serde_json::Value::String(escaped));
                    models_arr.push(serde_json::Value::Object(j_model));
                }
            }
            // Mirror nlohmann's behaviour where j[MODELS_STR] is only created when pushed to.
            if !models_arr.is_empty() {
                j.insert(MODELS_STR.to_string(), serde_json::Value::Array(models_arr));
            }
        }

        // AppConfig.cpp:922-924  c << std::setw(4) << j << std::endl;
        let value = serde_json::Value::Object(j);
        let dumped = serde_json::to_string_pretty(&value).unwrap_or_default();
        let out = format!("{}\n", dumped);

        // AppConfig.cpp:926-931  #ifdef WIN32 MD5 line (off-platform)
        // AppConfig.cpp:933  c.close();
        if std::fs::write(&path_pid, out.as_bytes()).is_err() {
            return;
        }

        // AppConfig.cpp:935-942  #ifdef WIN32 backup (off-platform)
        // AppConfig.cpp:947  rename_file(path_pid, path);
        let _ = rename_file(&path_pid, &path);
        // AppConfig.cpp:948  m_dirty = false;
        self.m_dirty = false;
    }

    // ---- AppConfig.hpp inline accessors -------------------------------------

    // AppConfig.hpp:64  bool dirty() const { return m_dirty; }
    pub fn dirty(&self) -> bool {
        self.m_dirty
    }

    // AppConfig.hpp:67  void set_dirty() { m_dirty = true; }
    pub fn set_dirty(&mut self) {
        self.m_dirty = true;
    }

    // AppConfig.hpp:70-81  bool get(section, key, value) const  (returns Option, modeling the bool + out-param)
    pub fn get_opt(&self, section: &str, key: &str) -> Option<&String> {
        // AppConfig.hpp:73-78
        self.m_storage.get(section).and_then(|s| s.get(key))
    }

    // AppConfig.hpp:82-83  std::string get(section, key) const { std::string value; this->get(section, key, value); return value; }
    pub fn get(&self, section: &str, key: &str) -> String {
        self.get_opt(section, key).cloned().unwrap_or_default()
    }

    // AppConfig.hpp:84-85  std::string get(key) const { return get("app", key); }
    pub fn get_key(&self, key: &str) -> String {
        self.get("app", key)
    }

    // AppConfig.hpp:86  bool get_bool(key) const { return get(key) == "true" || get(key) == "1"; }
    pub fn get_bool(&self, key: &str) -> bool {
        let v = self.get_key(key);
        v == "true" || v == "1"
    }

    // AppConfig.hpp:87-102  void set(section, key, value)
    pub fn set(&mut self, section: &str, key: &str, value: &str) {
        // AppConfig.hpp:89-95  #ifndef NDEBUG trim assertions (debug-only)
        // AppConfig.hpp:97  std::string &old = m_storage[section][key];
        let entry = self.m_storage.entry(section.to_string()).or_default();
        // AppConfig.hpp:98-101  if (old != value) { old = value; m_dirty = true; }
        match entry.get(key) {
            Some(old) if old == value => {}
            _ => {
                entry.insert(key.to_string(), value.to_string());
                self.m_dirty = true;
            }
        }
    }

    // AppConfig.hpp:104-119  void set_str(section, key, value)  (identical body to set)
    pub fn set_str(&mut self, section: &str, key: &str, value: &str) {
        // AppConfig.hpp:106-112  #ifndef NDEBUG trim assertions (debug-only)
        // AppConfig.hpp:114  std::string& old = m_storage[section][key];
        let entry = self.m_storage.entry(section.to_string()).or_default();
        // AppConfig.hpp:115-118
        match entry.get(key) {
            Some(old) if old == value => {}
            _ => {
                entry.insert(key.to_string(), value.to_string());
                self.m_dirty = true;
            }
        }
    }

    // AppConfig.hpp:121-128  void set(section, key, bool value)
    pub fn set_bool(&mut self, section: &str, key: &str, value: bool) {
        // AppConfig.hpp:123-127
        if value {
            self.set(section, key, "true");
        } else {
            self.set(section, key, "false");
        }
    }

    // AppConfig.hpp:131-132  void set(key, value) { set("app", key, value); }
    pub fn set_app(&mut self, key: &str, value: &str) {
        self.set("app", key, value);
    }

    // AppConfig.hpp:134-137  void set_bool(key, value) { set("app", key, value); }
    pub fn set_app_bool(&mut self, key: &str, value: bool) {
        self.set_bool("app", key, value);
    }

    // AppConfig.hpp:139-146  bool has(section, key) const
    pub fn has(&self, section: &str, key: &str) -> bool {
        // AppConfig.hpp:141-145  return it2 != end && !it2->second.empty();
        self.m_storage
            .get(section)
            .and_then(|s| s.get(key))
            .map(|v| !v.is_empty())
            .unwrap_or(false)
    }

    // AppConfig.hpp:147-148  bool has(key) const { return has("app", key); }
    pub fn has_key(&self, key: &str) -> bool {
        self.has("app", key)
    }

    // AppConfig.hpp:150-156  void erase(section, key)
    pub fn erase(&mut self, section: &str, key: &str) {
        // AppConfig.hpp:152-155
        if let Some(s) = self.m_storage.get_mut(section) {
            s.remove(key);
        }
    }

    // AppConfig.hpp:158-159  bool has_section(section) const
    pub fn has_section(&self, section: &str) -> bool {
        self.m_storage.contains_key(section)
    }

    // AppConfig.hpp:160-161  const std::map<std::string, std::string>& get_section(section) const
    pub fn get_section(&self, section: &str) -> &BTreeMap<String, String> {
        self.m_storage.get(section).unwrap()
    }

    // AppConfig.hpp:162-163  void set_section(section, data)
    pub fn set_section(&mut self, section: &str, data: BTreeMap<String, String>) {
        self.m_storage.insert(section.to_string(), data);
    }

    // AppConfig.hpp:164-165  void clear_section(section) { m_storage[section].clear(); }
    pub fn clear_section(&mut self, section: &str) {
        self.m_storage.entry(section.to_string()).or_default().clear();
    }

    // AppConfig.hpp:168 / AppConfig.cpp:1157-1163  bool get_variant(vendor, model, variant) const
    pub fn get_variant(&self, vendor: &str, model: &str, variant: &str) -> bool {
        // AppConfig.cpp:1159-1162
        match self.m_vendors.get(vendor) {
            None => false,
            Some(it_v) => match it_v.get(model) {
                None => false,
                Some(it_m) => it_m.contains(variant),
            },
        }
    }

    // AppConfig.hpp:169 / AppConfig.cpp:1165-1181  void set_variant(vendor, model, variant, enable)
    pub fn set_variant(&mut self, vendor: &str, model: &str, variant: &str, enable: bool) {
        // AppConfig.cpp:1167-1178
        if enable {
            // AppConfig.cpp:1168  if (get_variant(...)) return;
            if self.get_variant(vendor, model, variant) {
                return;
            }
            // AppConfig.cpp:1169
            self.m_vendors
                .entry(vendor.to_string())
                .or_default()
                .entry(model.to_string())
                .or_default()
                .insert(variant.to_string());
        } else {
            // AppConfig.cpp:1171-1177
            let it_v = match self.m_vendors.get_mut(vendor) {
                None => return,
                Some(v) => v,
            };
            let it_m = match it_v.get_mut(model) {
                None => return,
                Some(m) => m,
            };
            if !it_m.contains(variant) {
                return;
            }
            it_m.remove(variant);
        }
        // AppConfig.cpp:1180  m_dirty = true;
        self.m_dirty = true;
    }

    // AppConfig.hpp:170 / AppConfig.cpp:1183-1187  void set_vendors(const AppConfig &from)
    pub fn set_vendors_from(&mut self, from: &AppConfig) {
        // AppConfig.cpp:1185-1186
        self.m_vendors = from.m_vendors.clone();
        self.m_dirty = true;
    }

    // AppConfig.hpp:171-172  void set_vendors(const VendorMap&) / (VendorMap&&)
    pub fn set_vendors(&mut self, vendors: VendorMap) {
        self.m_vendors = vendors;
        self.m_dirty = true;
    }

    // AppConfig.hpp:173  const VendorMap& vendors() const { return m_vendors; }
    pub fn vendors(&self) -> &VendorMap {
        &self.m_vendors
    }

    // AppConfig.hpp:175  const std::vector<std::string>& get_filament_presets() const
    pub fn get_filament_presets(&self) -> &Vec<String> {
        &self.m_filament_presets
    }

    // AppConfig.hpp:176-179  void set_filament_presets(...)
    pub fn set_filament_presets(&mut self, filament_presets: Vec<String>) {
        self.m_filament_presets = filament_presets;
        self.m_dirty = true;
    }

    // AppConfig.hpp:180  const std::vector<std::string>& get_filament_colors() const
    pub fn get_filament_colors(&self) -> &Vec<String> {
        &self.m_filament_colors
    }

    // AppConfig.hpp:181-184  void set_filament_colors(...)
    pub fn set_filament_colors(&mut self, filament_colors: Vec<String>) {
        self.m_filament_colors = filament_colors;
        self.m_dirty = true;
    }

    // AppConfig.hpp:186  const std::vector<PrinterCaliInfo>& get_printer_cali_infos() const
    pub fn get_printer_cali_infos(&self) -> &Vec<PrinterCaliInfo> {
        &self.m_printer_cali_infos
    }

    // AppConfig.hpp:187 / AppConfig.cpp:1189-1206  void save_printer_cali_infos(cali_info, need_change_status = true)
    pub fn save_printer_cali_infos(&mut self, cali_info: &PrinterCaliInfo, need_change_status: bool) {
        // AppConfig.cpp:1191-1193  find_if by dev_id
        let pos = self
            .m_printer_cali_infos
            .iter()
            .position(|item| item.dev_id == cali_info.dev_id);

        match pos {
            // AppConfig.cpp:1195-1196  not found -> emplace_back
            None => {
                self.m_printer_cali_infos.push(cali_info.clone());
            }
            Some(idx) => {
                let iter = &mut self.m_printer_cali_infos[idx];
                // AppConfig.cpp:1198-1200  if (need_change_status) (*iter).cali_finished = ...;
                if need_change_status {
                    iter.cali_finished = cali_info.cali_finished;
                }
                // AppConfig.cpp:1201-1203
                iter.cache_flow_ratio = cali_info.cache_flow_ratio;
                iter.selected_presets = cali_info.selected_presets.clone();
                iter.cache_flow_rate_calibration_type = cali_info.cache_flow_rate_calibration_type;
            }
        }
        // AppConfig.cpp:1205  m_dirty = true;
        self.m_dirty = true;
    }

    // AppConfig.hpp:190 / AppConfig.cpp:1208-1224  std::string get_last_dir() const
    pub fn get_last_dir(&self) -> String {
        // AppConfig.cpp:1210-1222
        if let Some(it) = self.m_storage.get("recent") {
            // AppConfig.cpp:1212-1215
            if let Some(it2) = it.get("last_opened_folder") {
                if !it2.is_empty() {
                    return it2.clone();
                }
            }
            // AppConfig.cpp:1217-1220
            if let Some(it2) = it.get("settings_folder") {
                if !it2.is_empty() {
                    return it2.clone();
                }
            }
        }
        // AppConfig.cpp:1223
        String::new()
    }

    // AppConfig.hpp:240 / AppConfig.cpp:1226-1238  std::vector<std::string> get_recent_projects() const
    pub fn get_recent_projects(&self) -> Vec<String> {
        // AppConfig.cpp:1228  std::vector<std::string> ret;
        let mut ret: Vec<String> = Vec::new();
        // AppConfig.cpp:1229-1237
        if let Some(it) = self.m_storage.get("recent_projects") {
            for (_k, v) in it.iter() {
                ret.push(v.clone());
            }
        }
        ret
    }

    // AppConfig.hpp:241 / AppConfig.cpp:1240-1254  void set_recent_projects(const std::vector<std::string>&)
    pub fn set_recent_projects(&mut self, recent_projects: &[String]) {
        // AppConfig.cpp:1242-1245  ensure "recent_projects" section, clear it
        let it = self.m_storage.entry("recent_projects".to_string()).or_default();
        // AppConfig.cpp:1246  it->second.clear();
        it.clear();
        // AppConfig.cpp:1247-1253
        for i in 0..recent_projects.len() {
            let mut n = (i + 1).to_string();
            // AppConfig.cpp:1250-1251
            if n.len() == 1 {
                n = format!("00{}", n);
            } else if n.len() == 2 {
                n = format!("0{}", n);
            }
            // AppConfig.cpp:1252
            it.insert(n, recent_projects[i].clone());
        }
    }

    // AppConfig.hpp:243 / AppConfig.cpp:1256-1271  void set_mouse_device(...)
    pub fn set_mouse_device(
        &mut self,
        name: &str,
        translation_speed: f64,
        translation_deadzone: f64,
        rotation_speed: f32,
        rotation_deadzone: f32,
        zoom_speed: f64,
        swap_yz: bool,
    ) {
        // AppConfig.cpp:1259  std::string key = std::string("mouse_device:") + name;
        let key = format!("mouse_device:{}", name);
        // AppConfig.cpp:1260-1264  ensure section, clear it
        let it = self.m_storage.entry(key).or_default();
        it.clear();
        // AppConfig.cpp:1265-1270  float_to_string_decimal_point(value) — default precision -1
        it.insert(
            "translation_speed".to_string(),
            float_to_string_decimal_point(translation_speed, -1),
        );
        it.insert(
            "translation_deadzone".to_string(),
            float_to_string_decimal_point(translation_deadzone, -1),
        );
        it.insert(
            "rotation_speed".to_string(),
            float_to_string_decimal_point(rotation_speed as f64, -1),
        );
        it.insert(
            "rotation_deadzone".to_string(),
            float_to_string_decimal_point(rotation_deadzone as f64, -1),
        );
        it.insert(
            "zoom_speed".to_string(),
            float_to_string_decimal_point(zoom_speed, -1),
        );
        it.insert("swap_yz".to_string(), if swap_yz { "1".to_string() } else { "0".to_string() });
    }

    // AppConfig.hpp:244 / AppConfig.cpp:1273-1282  std::vector<std::string> get_mouse_device_names() const
    pub fn get_mouse_device_names(&self) -> Vec<String> {
        // AppConfig.cpp:1275-1276
        const PREFIX: &str = "mouse_device:";
        let prefix_len = PREFIX.len();
        // AppConfig.cpp:1277  std::vector<std::string> out;
        let mut out: Vec<String> = Vec::new();
        // AppConfig.cpp:1278-1280
        for (k, _v) in self.m_storage.iter() {
            if k.starts_with(PREFIX) && k.len() > prefix_len {
                out.push(k[prefix_len..].to_string());
            }
        }
        out
    }

    // AppConfig.hpp:245-256  get_mouse_device_* helpers delegate to get_3dmouse_device_numeric_value
    // AppConfig.hpp:245-246  translation_speed
    pub fn get_mouse_device_translation_speed(&self, name: &str) -> Option<f64> {
        self.get_3dmouse_device_numeric_value(name, "translation_speed")
    }
    // AppConfig.hpp:247-248  translation_deadzone
    pub fn get_mouse_device_translation_deadzone(&self, name: &str) -> Option<f64> {
        self.get_3dmouse_device_numeric_value(name, "translation_deadzone")
    }
    // AppConfig.hpp:249-250  rotation_speed (float)
    pub fn get_mouse_device_rotation_speed(&self, name: &str) -> Option<f32> {
        self.get_3dmouse_device_numeric_value(name, "rotation_speed").map(|v| v as f32)
    }
    // AppConfig.hpp:251-252  rotation_deadzone (float)
    pub fn get_mouse_device_rotation_deadzone(&self, name: &str) -> Option<f32> {
        self.get_3dmouse_device_numeric_value(name, "rotation_deadzone").map(|v| v as f32)
    }
    // AppConfig.hpp:253-254  zoom_speed
    pub fn get_mouse_device_zoom_speed(&self, name: &str) -> Option<f64> {
        self.get_3dmouse_device_numeric_value(name, "zoom_speed")
    }
    // AppConfig.hpp:255-256  swap_yz (bool, via numeric value)
    pub fn get_mouse_device_swap_yz(&self, name: &str) -> Option<bool> {
        self.get_3dmouse_device_numeric_value(name, "swap_yz").map(|v| v != 0.0)
    }

    // AppConfig.hpp:263-275  template<typename T> bool get_3dmouse_device_numeric_value(...)
    fn get_3dmouse_device_numeric_value(&self, device_name: &str, parameter_name: &str) -> Option<f64> {
        // AppConfig.hpp:266  std::string key = std::string("mouse_device:") + device_name;
        let key = format!("mouse_device:{}", device_name);
        // AppConfig.hpp:267-269
        let it = self.m_storage.get(&key)?;
        // AppConfig.hpp:270-272
        let it_val = it.get(parameter_name)?;
        // AppConfig.hpp:273  out = T(string_to_double_decimal_point(it_val->second));
        let (value, _consumed) = string_to_double_decimal_point(it_val);
        Some(value)
    }

    // AppConfig.hpp:191 / AppConfig.cpp:1284-1287  void update_config_dir(const std::string&)
    pub fn update_config_dir(&mut self, dir: &str) {
        // AppConfig.cpp:1286
        self.set("recent", "settings_folder", dir);
    }

    // AppConfig.hpp:192 / AppConfig.cpp:1289-1294  void update_skein_dir(const std::string&)
    pub fn update_skein_dir(&mut self, dir: &str) {
        // AppConfig.cpp:1291-1292  if (is_shapes_dir(dir)) return;
        if is_shapes_dir(dir) {
            return;
        }
        // AppConfig.cpp:1293
        self.set("recent", "last_opened_folder", dir);
    }

    // AppConfig.hpp:196 / AppConfig.cpp:1314-1324  std::string get_last_output_dir(const std::string& alt, const bool removable = false) const
    pub fn get_last_output_dir(&self, alt: &str, _removable: bool) -> String {
        // AppConfig.cpp:1316  std::string s1 = ("last_export_path");
        let s1 = "last_export_path";
        // AppConfig.cpp:1317-1322
        if let Some(it) = self.m_storage.get("app") {
            if let Some(it2) = it.get(s1) {
                if !it2.is_empty() {
                    return it2.clone();
                }
            }
        }
        // AppConfig.cpp:1323  return is_shapes_dir(alt) ? get_last_dir() : alt;
        if is_shapes_dir(alt) {
            self.get_last_dir()
        } else {
            alt.to_string()
        }
    }

    // AppConfig.hpp:197 / AppConfig.cpp:1326-1329  void update_last_output_dir(const std::string& dir, const bool removable = false)
    pub fn update_last_output_dir(&mut self, dir: &str, _removable: bool) {
        // AppConfig.cpp:1328
        self.set("app", "last_export_path", dir);
    }

    // AppConfig.hpp:200 / AppConfig.cpp:1332-1341  std::string get_last_backup_dir() const
    pub fn get_last_backup_dir(&self) -> String {
        // AppConfig.cpp:1334-1339
        if let Some(it) = self.m_storage.get("app") {
            if let Some(it2) = it.get("last_backup_path") {
                return it2.clone();
            }
        }
        // AppConfig.cpp:1340
        String::new()
    }

    // AppConfig.hpp:201 / AppConfig.cpp:1343-1348  void update_last_backup_dir(const std::string& dir)
    pub fn update_last_backup_dir(&mut self, dir: &str) {
        // AppConfig.cpp:1346
        self.set("app", "last_backup_path", dir);
        // AppConfig.cpp:1347
        self.save();
    }

    // AppConfig.hpp:203 / AppConfig.cpp:1350-1367  std::string get_region()
    pub fn get_region(&self) -> String {
        // AppConfig.cpp:1352-1353  #if BBL_RELEASE_TO_PUBLIC return this->get("region");
        if BBL_RELEASE_TO_PUBLIC {
            self.get_key("region")
        } else {
            // AppConfig.cpp:1355-1366  dev/qa/pre region remapping (compiled out in public build)
            let sel = self.get_key("iot_environment");
            let mut region = String::new();
            if sel == ENV_DEV_HOST {
                region = "NEW_ENV_DEV_HOST".to_string();
            } else if sel == ENV_QAT_HOST {
                region = "NEW_ENV_QAT_HOST".to_string();
            } else if sel == ENV_PRE_HOST {
                region = "NEW_ENV_PRE_HOST".to_string();
            }
            if region.is_empty() {
                return self.get_key("region");
            }
            region
        }
    }

    // AppConfig.hpp:204 / AppConfig.cpp:1369-1389  std::string get_country_code()
    pub fn get_country_code(&self) -> String {
        // AppConfig.cpp:1371  std::string region = get_region();
        let region = self.get_region();
        // AppConfig.cpp:1372-1374  #if !BBL_RELEASE_TO_PUBLIC if (is_engineering_region()) return region;
        if !BBL_RELEASE_TO_PUBLIC && self.is_engineering_region() {
            return region;
        }
        // AppConfig.cpp:1375-1386
        if region == "CHN" || region == "China" {
            "CN".to_string()
        } else if region == "USA" {
            "US".to_string()
        } else if region == "Asia-Pacific" {
            "Others".to_string()
        } else if region == "Europe" {
            "US".to_string()
        } else if region == "North America" {
            "US".to_string()
        } else {
            "Others".to_string()
        }
    }

    // AppConfig.hpp:205 / AppConfig.cpp:1391-1399  bool is_engineering_region()
    pub fn is_engineering_region(&self) -> bool {
        // AppConfig.cpp:1392  std::string sel = get("iot_environment");
        let sel = self.get_key("iot_environment");
        // AppConfig.cpp:1394-1397
        if sel == ENV_DEV_HOST || sel == ENV_QAT_HOST || sel == ENV_PRE_HOST {
            return true;
        }
        // AppConfig.cpp:1398
        false
    }

    // AppConfig.hpp:207 / AppConfig.cpp:1401-1420  void save_custom_color_to_config(const std::vector<std::string>& colors)
    pub fn save_custom_color_to_config(&mut self, colors: &[String]) {
        // AppConfig.cpp:1403-1407  set_colors lambda: data[to_string(10 + i)] = colors[i];
        fn set_colors(data: &mut BTreeMap<String, String>, colors: &[String]) {
            for (i, c) in colors.iter().enumerate() {
                data.insert((10 + i).to_string(), c.clone());
            }
        }
        // AppConfig.cpp:1408-1419
        if !colors.is_empty() {
            if !self.has_section("custom_color_list") {
                // AppConfig.cpp:1410-1412
                let mut data: BTreeMap<String, String> = BTreeMap::new();
                set_colors(&mut data, colors);
                self.set_section("custom_color_list", data);
            } else {
                // AppConfig.cpp:1414-1417  copy existing, modify, set back
                let mut data = self.get_section("custom_color_list").clone();
                set_colors(&mut data, colors);
                self.set_section("custom_color_list", data);
            }
        }
    }

    // AppConfig.hpp:208 / AppConfig.cpp:1422-1432  std::vector<std::string> get_custom_color_from_config()
    pub fn get_custom_color_from_config(&self) -> Vec<String> {
        // AppConfig.cpp:1424  std::vector<std::string> colors;
        let mut colors: Vec<String> = Vec::new();
        // AppConfig.cpp:1425-1430
        if self.has_section("custom_color_list") {
            let data = self.get_section("custom_color_list");
            for (_k, v) in data.iter() {
                colors.push(v.clone());
            }
        }
        colors
    }

    // AppConfig.hpp:210 / AppConfig.cpp:1434-1446  void save_nozzle_volume_types_to_config(printer_name, nozzle_volume_types)
    pub fn save_nozzle_volume_types_to_config(&mut self, printer_name: &str, nozzle_volume_types: &str) {
        // AppConfig.cpp:1436-1445
        if !self.has_section("nozzle_volume_types") {
            // AppConfig.cpp:1437-1439
            let mut data: BTreeMap<String, String> = BTreeMap::new();
            data.insert(printer_name.to_string(), nozzle_volume_types.to_string());
            self.set_section("nozzle_volume_types", data);
        } else {
            // AppConfig.cpp:1441-1444
            let mut data = self.get_section("nozzle_volume_types").clone();
            data.insert(printer_name.to_string(), nozzle_volume_types.to_string());
            self.set_section("nozzle_volume_types", data);
        }
    }

    // AppConfig.hpp:211 / AppConfig.cpp:1448-1458  std::string get_nozzle_volume_types_from_config(printer_name)
    pub fn get_nozzle_volume_types_from_config(&self, printer_name: &str) -> String {
        // AppConfig.cpp:1450  std::string nozzle_volume_types;
        let mut nozzle_volume_types = String::new();
        // AppConfig.cpp:1451-1455
        if self.has_section("nozzle_volume_types") {
            let data = self.get_section("nozzle_volume_types");
            if let Some(v) = data.get(printer_name) {
                nozzle_volume_types = v.clone();
            }
        }
        nozzle_volume_types
    }

    // AppConfig.hpp:216 / AppConfig.cpp:1460-1472  void reset_selections()
    pub fn reset_selections(&mut self) {
        // AppConfig.cpp:1462-1471
        if let Some(it) = self.m_storage.get_mut("presets") {
            // AppConfig.cpp:1464-1469
            it.remove(PRESET_PRINT_NAME);
            it.remove(PRESET_FILAMENT_NAME);
            it.remove("sla_print");
            it.remove("sla_material");
            it.remove(PRESET_PRINTER_NAME);
            it.remove("physical_printer");
            // AppConfig.cpp:1470  m_dirty = true;
            self.m_dirty = true;
        }
    }

    // AppConfig.hpp:219  std::string config_path() { return config_path(m_mode); }
    pub fn config_path(&self) -> String {
        Self::config_path_mode(self.m_mode)
    }

    // AppConfig.hpp:220 / AppConfig.cpp:1474-1487  static std::string config_path(EAppMode mode)
    pub fn config_path_mode(mode: EAppMode) -> String {
        // AppConfig.cpp:1476-1484  #ifdef USE_JSON_CONFIG ".conf" #else ".ini"
        // USE_JSON_CONFIG is defined: use ".conf".
        let app_key = if mode == EAppMode::Editor {
            SLIC3R_APP_KEY
        } else {
            GCODEVIEWER_APP_KEY
        };
        // (boost::filesystem::path(data_dir()) / (app_key ".conf")).make_preferred().string()
        let mut path = std::path::PathBuf::from(data_dir());
        path.push(format!("{}.conf", app_key));
        path.to_string_lossy().into_owned()
    }

    // AppConfig.hpp:223  bool legacy_datadir() const { return m_legacy_datadir; }
    pub fn legacy_datadir(&self) -> bool {
        self.m_legacy_datadir
    }

    // AppConfig.hpp:224  void set_legacy_datadir(bool value) { m_legacy_datadir = value; }
    pub fn set_legacy_datadir(&mut self, value: bool) {
        self.m_legacy_datadir = value;
    }

    // AppConfig.hpp:228 / AppConfig.cpp:1489-1493  std::string version_check_url() const
    pub fn version_check_url(&self) -> String {
        // AppConfig.cpp:1491  auto from_settings = get("version_check_url");
        let from_settings = self.get_key("version_check_url");
        // AppConfig.cpp:1492  return from_settings.empty() ? VERSION_CHECK_URL : from_settings;
        if from_settings.is_empty() {
            VERSION_CHECK_URL.to_string()
        } else {
            from_settings
        }
    }

    // AppConfig.hpp:232  Semver orig_version() const { return m_orig_version; }
    pub fn orig_version(&self) -> &Semver {
        &self.m_orig_version
    }

    // AppConfig.hpp:235 / AppConfig.cpp:1495-1498  bool exists()
    pub fn exists(&self) -> bool {
        // AppConfig.cpp:1497
        std::path::Path::new(&self.config_path()).exists()
    }

    // AppConfig.hpp:237  void set_loading_path(const std::string& path) { m_loading_path = path; }
    pub fn set_loading_path(&mut self, path: &str) {
        self.m_loading_path = path.to_string();
    }

    // AppConfig.hpp:238  std::string loading_path() { return (m_loading_path.empty() ? config_path() : m_loading_path); }
    pub fn loading_path(&self) -> String {
        if self.m_loading_path.is_empty() {
            self.config_path()
        } else {
            self.m_loading_path.clone()
        }
    }
}

// ===========================================================================
// File-local helpers
// ===========================================================================

// AppConfig.cpp:872-878  lambda is_filament_preset_key
fn is_filament_preset_key(key: &str) -> bool {
    // AppConfig.cpp:873  if (key == "filament") return true;
    if key == "filament" {
        return true;
    }
    // AppConfig.cpp:874-876  key.size() > 9 && key.substr(0,9) == "filament_" && all digits after
    if key.len() > 9 && &key[0..9] == "filament_" {
        return key[9..].chars().all(|c| c.is_ascii_digit());
    }
    // AppConfig.cpp:877
    false
}

// ---- serde_json access helpers (model nlohmann's get<T>() coercions) -------

fn json_get_str(v: &serde_json::Value, key: &str) -> String {
    v.get(key).and_then(|x| x.as_str()).unwrap_or("").to_string()
}

fn json_get_i64(v: &serde_json::Value, key: &str) -> i64 {
    v.get(key).map(json_as_i64).unwrap_or(0)
}

fn json_get_f64(v: &serde_json::Value, key: &str) -> f64 {
    v.get(key).map(json_as_f64).unwrap_or(0.0)
}

fn json_as_i64(v: &serde_json::Value) -> i64 {
    if let Some(i) = v.as_i64() {
        i
    } else if let Some(f) = v.as_f64() {
        f as i64
    } else {
        0
    }
}

fn json_as_f64(v: &serde_json::Value) -> f64 {
    v.as_f64().unwrap_or(0.0)
}

fn json_string_vec(v: &serde_json::Value) -> Vec<String> {
    match v.as_array() {
        Some(arr) => arr
            .iter()
            .map(|x| x.as_str().unwrap_or("").to_string())
            .collect(),
        None => Vec::new(),
    }
}

fn str_vec_to_json(v: &[String]) -> serde_json::Value {
    serde_json::Value::Array(v.iter().map(|s| serde_json::Value::String(s.clone())).collect())
}

fn json_f32(v: f32) -> serde_json::Value {
    serde_json::Number::from_f64(v as f64)
        .map(serde_json::Value::Number)
        .unwrap_or(serde_json::Value::Null)
}

// ---- calib enum <-> int mappings (PrintConfig.hpp enum order) --------------
//
// The Rust `calib` enums currently model only a subset of the C++ variants
// (Standard=0, HighFlow=1; CoolPlate=0..). We map by discriminant order to keep
// the JSON round-trip faithful for the values that exist; unknown integers fall
// back to the C++ default (variant 0).

fn nozzle_volume_type_from_int(i: i32) -> NozzleVolumeType {
    // PrintConfig.hpp:346-351  nvtStandard = 0, nvtHighFlow, ...
    match i {
        1 => NozzleVolumeType::HighFlow,
        _ => NozzleVolumeType::Standard,
    }
}

fn nozzle_volume_type_to_int(t: NozzleVolumeType) -> i32 {
    match t {
        NozzleVolumeType::Standard => 0,
        NozzleVolumeType::HighFlow => 1,
    }
}

fn bed_type_from_int(i: i32) -> BedType {
    // calib::BedType variant order: CoolPlate, EngineeringPlate, HighTempPlate, TexturedPEI
    match i {
        1 => BedType::EngineeringPlate,
        2 => BedType::HighTempPlate,
        3 => BedType::TexturedPEI,
        _ => BedType::CoolPlate,
    }
}

fn bed_type_to_int(t: BedType) -> i32 {
    match t {
        BedType::CoolPlate => 0,
        BedType::EngineeringPlate => 1,
        BedType::HighTempPlate => 2,
        BedType::TexturedPEI => 3,
    }
}

fn flow_ratio_calibration_type_from_int(i: i32) -> FlowRatioCalibrationType {
    // calib.rs:159  CompleteCalibration = 0, FineCalibration
    match i {
        1 => FlowRatioCalibrationType::FineCalibration,
        _ => FlowRatioCalibrationType::CompleteCalibration,
    }
}

fn flow_ratio_calibration_type_to_int(t: FlowRatioCalibrationType) -> i32 {
    match t {
        FlowRatioCalibrationType::CompleteCalibration => 0,
        FlowRatioCalibrationType::FineCalibration => 1,
    }
}

// ---- c-style string (un)escaping, ported from Config.cpp -------------------
// These live in `crate::config` (the faithful port of Config.cpp). Re-aliased
// here so the existing AppConfig.cpp call sites keep working unchanged.
use crate::config::{escape_strings_cstyle, unescape_strings_cstyle};

// ---- slicer_uuid generation (replaces boost::uuids::random_generator) ------
//
// boost::uuids::random_generator is a native dep; we synthesize a v4-shaped
// UUID string from std entropy. This value never enters G-code, so byte-exact
// parity is unaffected.
fn random_uuid_string() -> String {
    use std::collections::hash_map::RandomState;
    use std::hash::{BuildHasher, Hasher};
    use std::time::{SystemTime, UNIX_EPOCH};

    let mut bytes = [0u8; 16];
    let mut fill = |seed: u64, off: usize| {
        let mut h = RandomState::new().build_hasher();
        h.write_u64(seed);
        let v = h.finish();
        bytes[off..off + 8].copy_from_slice(&v.to_le_bytes());
    };
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);
    fill(now ^ (std::process::id() as u64), 0);
    fill(now.wrapping_mul(0x9E37_79B9_7F4A_7C15), 8);

    // Set version (4) and variant (RFC 4122) bits.
    bytes[6] = (bytes[6] & 0x0F) | 0x40;
    bytes[8] = (bytes[8] & 0x3F) | 0x80;

    format!(
        "{:02x}{:02x}{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}-{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
        bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7], bytes[8], bytes[9],
        bytes[10], bytes[11], bytes[12], bytes[13], bytes[14], bytes[15]
    )
}

// Suppress unused warnings for items kept for fidelity / future callers.
#[allow(dead_code)]
fn _faithful_keepalive() {
    let _ = (CopyFileResult::Success, copy_file as fn(&str, &str, &mut String, bool) -> CopyFileResult);
    let _: HashMap<String, String> = HashMap::new();
}
