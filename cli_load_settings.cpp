// cli_load_settings.cpp — see cli_load_settings.hpp.
//
// A line-by-line port. "B nnnn" cites BambuStudio.cpp at 5873b5f and
// "O nnnn" OrcaSlicer.cpp at 31f6803; ENGINE_ORCA blocks are where the two
// official CLIs differ.
#include "cli_load_settings.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>
#include <nlohmann/json.hpp>

#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/FlushVolCalc.hpp"
#include "libslic3r/LocalesUtils.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Utils.hpp"

#include "cli_events.hpp"
#include "desktop_presets.hpp"

namespace slicer_cli {
namespace {

using namespace Slic3r;
using json = nlohmann::json;

#ifdef ENGINE_ORCA
using FloatsN = ConfigOptionFloats;   // OrcaSlicer's nozzle_diameter and friends are not nullable
using BoolsN  = ConfigOptionBools;
#else
using FloatsN = ConfigOptionFloatsNullable;
using BoolsN  = ConfigOptionBoolsNullable;
#endif

// B 682-683 / O 545-546.
const std::set<std::string> gcodes_key_set = {"filament_end_gcode", "filament_start_gcode", "change_filament_gcode",
    "layer_change_gcode", "machine_end_gcode", "machine_pause_gcode", "machine_start_gcode", "template_custom_gcode",
    "printing_by_object_gcode", "before_layer_change_gcode", "time_lapse_gcode", "wrapping_detection_gcode"};

/// load_default_gcodes_to_config (B 685-745; O 548-604): every G-code key
/// present, empty when the file has none.
void load_default_gcodes_to_config(DynamicPrintConfig& config, Preset::Type type) {
    if (config.size() == 0)
        return;
    if (type == Preset::TYPE_PRINTER) {
        for (const char* key : {"change_filament_gcode", "layer_change_gcode", "machine_end_gcode", "machine_pause_gcode",
                                "machine_start_gcode", "template_custom_gcode", "printing_by_object_gcode",
                                "before_layer_change_gcode", "time_lapse_gcode", "wrapping_detection_gcode"})
            config.option<ConfigOptionString>(key, true);
    } else if (type == Preset::TYPE_FILAMENT) {
        for (const char* key : {"filament_start_gcode", "filament_end_gcode"}) {
            std::vector<std::string>& values = config.option<ConfigOptionStrings>(key, true)->values;
            if (values.empty())
                values.resize(1, std::string());
        }
    }
}

/// load_key_values_from_json (B 646-680): the model_id and name of a JSON file.
void load_key_values_from_json(const std::string& file, std::map<std::string, std::string>& key_values) {
    try {
        CNumericLocalesSetter locales_setter;
        json j;
        boost::nowide::ifstream ifs(file);
        ifs >> j;
        for (auto it = j.begin(); it != j.end(); ++it) {
            if (boost::iequals(it.key(), BBL_JSON_KEY_MODEL_ID))
                key_values.emplace(BBL_JSON_KEY_MODEL_ID, it.value());
            else if (boost::iequals(it.key(), BBL_JSON_KEY_NAME))
                key_values.emplace(BBL_JSON_KEY_NAME, it.value());
        }
    } catch (...) {
    }
}

/// The lambda load_config_file (B 2248-2315; O 1880-1947): a settings file
/// with its type, name, filament id and origin; "system" or "User" only.
int load_config_file(const std::string& file, DynamicPrintConfig& config, std::string& config_type,
                     std::string& config_name, std::string& filament_id, std::string& config_from, std::string& why) {
    if (!boost::filesystem::exists(file)) {
        why = "cannot find the settings file " + file;
        return CLI_FILE_NOTFOUND;
    }
    try {
        std::map<std::string, std::string> key_values;
        std::string reason;
        config.load_from_json(file, ForwardCompatibilitySubstitutionRule::Enable, key_values, reason);
        if (!reason.empty()) {
            why = "cannot load settings from " + file + ": " + reason;
            return CLI_CONFIG_FILE_ERROR;
        }
        config_name = key_values[BBL_JSON_KEY_NAME];
        if (auto from_iter = key_values.find(BBL_JSON_KEY_FROM); from_iter != key_values.end())
            config_from = from_iter->second;
        if (config_from != "system" && config_from != "User" && config_from != "user") {
            why = file + " says it is from '" + config_from + "'; a settings file must be from \"system\" or \"User\"";
            return CLI_CONFIG_FILE_ERROR;
        }
        if (auto type_iter = key_values.find(BBL_JSON_KEY_TYPE); type_iter != key_values.end())
            config_type = type_iter->second;
        if (config_type == "filament") {
            if (auto id_iter = key_values.find(BBL_JSON_KEY_FILAMENT_ID); id_iter != key_values.end())
                filament_id = id_iter->second;
        } else if (config_type != "machine" && config_type != "process") {
            why = file + " is of type '" + config_type + "'; a settings file is a machine, process or filament";
            return CLI_CONFIG_FILE_ERROR;
        }
        config.normalize_fdm();
    } catch (const std::exception& ex) {
        why = "loading " + file + " failed: " + ex.what();
        return CLI_CONFIG_FILE_ERROR;
    }
    return 0;
}

std::string full_preset_path(const char* folder, const std::string& name) {
    return resources_dir() + "/profiles/BBL/" + folder + "/" + name + ".json";
}

/// The printer model id of a machine file's printer_model, from
/// machine_full (B 2345-2359).
std::string printer_model_id_for(const std::string& printer_model) {
    if (printer_model.empty())
        return {};
    const std::string path = full_preset_path("machine_full", printer_model);
    if (!boost::filesystem::exists(path))
        return {};
    std::map<std::string, std::string> key_values;
    load_key_values_from_json(path, key_values);
    auto it = key_values.find(BBL_JSON_KEY_MODEL_ID);
    return it == key_values.end() ? std::string() : it->second;
}

/// The lambda compute_variant_index (B 3098-3138; O 2634-2675).
void compute_variant_index(DynamicPrintConfig& full_config, const DynamicPrintConfig& new_config,
                           const std::string& id_name, const std::string& variant_name,
                           std::vector<int>& new_index, bool& count_changed) {
    auto curr_variant_opt = dynamic_cast<const ConfigOptionStrings*>(full_config.option(variant_name));
    auto new_variant_opt  = dynamic_cast<const ConfigOptionStrings*>(new_config.option(variant_name));
    auto curr_id_opt      = dynamic_cast<const ConfigOptionInts*>(full_config.option(id_name));
    auto new_id_opt       = dynamic_cast<const ConfigOptionInts*>(new_config.option(id_name));
    if (!new_variant_opt || !new_id_opt) {
        count_changed = false;
        return;
    }
    const int new_variant_count = int(new_variant_opt->size());
    int curr_variant_count = 0;
    new_index.clear();
    new_index.resize(new_variant_count, -1);
    if (curr_variant_opt && curr_id_opt) {
        if (curr_variant_opt->size() != curr_id_opt->size()) {
            count_changed = false;
            return;
        }
        curr_variant_count = int(curr_variant_opt->size());
        count_changed = (curr_variant_count != new_variant_count);
    } else
        count_changed = (new_variant_count != 1);
    for (int i = 0; i < new_variant_count; i++) {
        if (curr_variant_count > 0) {
            for (int j = 0; j < curr_variant_count; j++)
                if (curr_variant_opt->values[j] == new_variant_opt->values[i] && curr_id_opt->values[j] == new_id_opt->values[i]) {
                    new_index[i] = j;
                    break;
                }
        } else if (new_variant_opt->values[i] == "Direct Drive Standard" && new_id_opt->values[i] == 1)
            new_index[i] = 0;
    }
}

/// The lambda update_full_config (B 3141-3205; O 2677-2740).
int update_full_config(DynamicPrintConfig& full_config, const DynamicPrintConfig& config, std::set<std::string>& diff_key_sets,
                       bool variant_count_changed, const std::set<std::string>& key_set_1, const std::set<std::string>& key_set_2,
                       std::vector<int> variant_index, bool update_all = false, bool skip_gcodes = false) {
    for (const t_config_option_key& opt_key : config.keys()) {
        if (!update_all && !diff_key_sets.empty()) {
            auto iter = diff_key_sets.find(opt_key);
            if (iter != diff_key_sets.end()) {
                if (skip_gcodes && gcodes_key_set.count(opt_key)) {
                    diff_key_sets.erase(iter);
                } else {
                    if (variant_count_changed) {
                        int stride = 0;
                        if (key_set_1.count(opt_key) > 0)
                            stride = 1;
                        else if (key_set_2.count(opt_key) > 0)
                            stride = 2;
                        if (stride > 0) {
                            const ConfigOption* source_opt = config.option(opt_key);
                            ConfigOption* dest_opt = full_config.option(opt_key, true);
                            auto* opt_vec_src  = static_cast<const ConfigOptionVectorBase*>(source_opt);
                            auto* opt_vec_dest = static_cast<ConfigOptionVectorBase*>(dest_opt);
                            opt_vec_dest->set_with_restore(opt_vec_src, variant_index, stride);
#ifndef ENGINE_ORCA
                            continue;   // B 3175; the OrcaSlicer CLI falls through and also copies the key (O 2709-2713)
#endif
                        } else
                            continue;
                    } else
                        continue;
                }
            }
        }
        const ConfigOption* source_opt = config.option(opt_key);
        if (source_opt == nullptr)
            return CLI_CONFIG_FILE_ERROR;
        if (opt_key == "compatible_prints" || opt_key == "compatible_printers" || opt_key == "model_id" || opt_key == "inherits" ||
            opt_key == "dev_model_name" || opt_key == "name" || opt_key == "from" || opt_key == "type" || opt_key == "version" ||
            opt_key == "setting_id" || opt_key == "instantiation")
            continue;
        ConfigOption* dest_opt = full_config.option(opt_key, true);
        if (dest_opt == nullptr)
            return CLI_CONFIG_FILE_ERROR;
        dest_opt->set(source_opt);
    }
    return 0;
}

/// BitmapCache::parse_color4 (BambuStudio slic3r/GUI/BitmapCache.cpp
/// 555-569; the same at 31f6803): "#RRGGBB" or "#RRGGBBAA".
bool parse_color4(const std::string& scolor, unsigned char* rgba_out) {
    auto hex_digit = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
    };
    rgba_out[0] = rgba_out[1] = rgba_out[2] = 0;
    rgba_out[3] = 255;
    if ((scolor.size() != 7 && scolor.size() != 9) || scolor.front() != '#')
        return false;
    const char* c = scolor.data() + 1;
    for (size_t i = 0; i < scolor.size() / 2; ++i) {
        const int d1 = hex_digit(*c++);
        const int d2 = hex_digit(*c++);
        if (d1 == -1 || d2 == -1)
            return false;
        rgba_out[i] = (unsigned char)(d1 * 16 + d2);
    }
    return true;
}

/// get_min_flush_volumes (BambuStudio slic3r/GUI/Plater.cpp 1121-1185;
/// OrcaSlicer slic3r/GUI/Plater.cpp 752-816): the nozzle volume less what a
/// long retraction when cutting already removed, per filament.
std::vector<int> get_min_flush_volumes(const DynamicPrintConfig& full_config, size_t nozzle_id) {
    std::vector<int> extra_flush_volumes;
    const auto* nozzle_volume_opt = full_config.option<ConfigOptionFloatsNullable>("nozzle_volume");
    const int nozzle_volume_val = nozzle_volume_opt ? (int)nozzle_volume_opt->get_at(nozzle_id) : 0;
    int machine_enabled_level = 0;
    if (const auto* opt = full_config.option<ConfigOptionInt>("enable_long_retraction_when_cut"))
        machine_enabled_level = opt->value;
    bool machine_activated = false;
    if (const auto* opt = full_config.option<BoolsN>("long_retractions_when_cut"))
        machine_activated = opt->values[nozzle_id] == 1;
    const size_t filament_size = full_config.option<ConfigOptionFloats>("filament_diameter")->values.size();
    std::vector<double> filament_retraction_distance_when_cut(filament_size, 18.0f), printer_retraction_distance_when_cut(filament_size, 18.0f);
    std::vector<unsigned char> filament_long_retractions_when_cut(filament_size, 0);
    if (const auto* opt = full_config.option<FloatsN>("filament_retraction_distances_when_cut"))
        filament_retraction_distance_when_cut = opt->values;
    if (const auto* opt = full_config.option<FloatsN>("retraction_distances_when_cut"))
        printer_retraction_distance_when_cut = opt->values;
    if (const auto* opt = full_config.option<BoolsN>("filament_long_retractions_when_cut"))
        filament_long_retractions_when_cut = opt->values;
    for (size_t idx = 0; idx < filament_size; ++idx) {
        int extra_flush_volume = nozzle_volume_val;
        int retract_length = machine_enabled_level && machine_activated ? printer_retraction_distance_when_cut[nozzle_id] : 0;
        const unsigned char filament_activated = filament_long_retractions_when_cut[idx];
        const double filament_retract_length = filament_retraction_distance_when_cut[idx];
        if (filament_activated == 0)
            retract_length = 0;
        else if (filament_activated == 1 && machine_enabled_level == LongRectrationLevel::EnableFilament) {
            if (!std::isnan(filament_retract_length))
                retract_length = (int)filament_retraction_distance_when_cut[idx];
            else
                retract_length = printer_retraction_distance_when_cut[nozzle_id];
        }
        extra_flush_volume -= PI * 1.75 * 1.75 / 4 * retract_length;
        extra_flush_volumes.emplace_back(extra_flush_volume);
    }
    return extra_flush_volumes;
}

StepResult fail(int code, const std::string& message) {
    StepResult r;
    r.code    = code;
    r.message = message;
    return r;
}

} // namespace

void read_project_facts(const DynamicPrintConfig& config, ProjectFacts& f) {
    f.is_bbl_3mf = true;
    f.current_printer_name  = config.option<ConfigOptionString>("printer_settings_id") ? config.opt_string("printer_settings_id") : "";
    f.current_process_name  = config.option<ConfigOptionString>("print_settings_id") ? config.opt_string("print_settings_id") : "";
    f.current_printer_model = config.has("printer_model") ? config.opt_string("printer_model") : "";
    if (const auto* fs = config.option<ConfigOptionStrings>("filament_settings_id"))
        f.current_filaments_name = fs->values;
    if (const auto* nd = config.option<FloatsN>("nozzle_diameter"))
        f.current_extruder_count = int(nd->values.size());
    if (const auto* v = config.option<ConfigOptionStrings>("printer_extruder_variant"))
        f.current_printer_variant_count = int(v->values.size());
    if (const auto* v = config.option<ConfigOptionStrings>("print_extruder_variant"))
        f.current_print_variant_count = int(v->values.size());
    f.current_is_multi_extruder = f.current_extruder_count > 1;
    const auto* opt_extruder_type       = dynamic_cast<const ConfigOptionEnumsGeneric*>(config.option("extruder_type"));
    const auto* opt_nozzle_volume_type  = dynamic_cast<const ConfigOptionEnumsGeneric*>(config.option("nozzle_volume_type"));
    if (opt_nozzle_volume_type)
        f.current_nozzle_volume_type = opt_nozzle_volume_type->values;
    f.current_extruder_variants.assign(f.current_extruder_count, "");
    for (int e = 0; e < f.current_extruder_count; e++) {
        const ExtruderType extruder_type = opt_extruder_type ? (ExtruderType)(opt_extruder_type->get_at(e)) : etDirectDrive;
        const NozzleVolumeType nvt = opt_nozzle_volume_type ? (NozzleVolumeType)(opt_nozzle_volume_type->get_at(e)) : nvtStandard;
        f.current_extruder_variants[e] = get_extruder_variant_string(extruder_type, nvt);
    }
    if (const auto* option_strings = config.option<ConfigOptionStrings>("inherits_group");
        option_strings && option_strings->values.size() >= 2) {
        f.current_inherits_group = option_strings->values;
        const size_t size = f.current_inherits_group.size();
        f.current_printer_system_name = f.current_inherits_group[size - 1].empty() ? f.current_printer_name : f.current_inherits_group[size - 1];
        f.current_process_system_name = f.current_inherits_group[0].empty() ? f.current_process_name : f.current_inherits_group[0];
        f.current_filaments_system_name.resize(size - 2);
        for (size_t index = 1; index < size - 1; index++)
            f.current_filaments_system_name[index - 1] =
                f.current_inherits_group[index].empty() && index - 1 < f.current_filaments_name.size()
                    ? f.current_filaments_name[index - 1] : f.current_inherits_group[index];
    } else {
        f.current_printer_system_name   = f.current_printer_name;
        f.current_process_system_name   = f.current_process_name;
        f.current_filaments_system_name = f.current_filaments_name;
    }
    f.filament_count = int(f.current_filaments_name.size());
    f.converted_filaments_system_name = f.current_filaments_system_name;
#ifndef ENGINE_ORCA
    // B 2042-2046: the system filament names converted to the printer's naming.
    for (int i = 0; i < f.filament_count && i < int(f.converted_filaments_system_name.size()); i++) {
        std::string machine = f.current_printer_system_name;
        convert_filament_preset_name(machine, f.converted_filaments_system_name[i]);
    }
#endif
    if (const auto* v = config.option<ConfigOptionStrings>("upward_compatible_machine"))
        f.upward_compatible_printers = v->values;
    if (const auto* v = config.option<ConfigOptionStrings>("print_compatible_printers"))
        f.current_print_compatible_printers = v->values;
    if (const auto* v = config.option<ConfigOptionStrings>("different_settings_to_system"))
        f.current_different_settings = v->values;
    if (const auto* exclude = config.option<ConfigOptionPoints>("bed_exclude_area")) {
        f.old_exclude_area_empty = exclude->values.empty();
        f.old_exclude_area = exclude->values;
    }
    if (const auto* area = config.option<ConfigOptionPoints>("printable_area"); area && area->values.size() >= 4) {
        f.old_printable_width = (int)(area->values[2].x() - area->values[0].x());
        f.old_printable_depth = (int)(area->values[2].y() - area->values[0].y());
    }
    if (config.has("printable_height"))
        f.old_printable_height = (int)config.opt_float("printable_height");
    if (config.option<ConfigOptionFloat>("extruder_clearance_height_to_rod"))
        f.old_height_to_rod = float(config.opt_float("extruder_clearance_height_to_rod"));
    if (config.option<ConfigOptionFloat>("extruder_clearance_height_to_lid"))
        f.old_height_to_lid = float(config.opt_float("extruder_clearance_height_to_lid"));
#ifdef ENGINE_ORCA
    if (config.option<ConfigOptionFloat>("extruder_clearance_radius"))
        f.old_max_radius = float(config.opt_float("extruder_clearance_radius"));
#else
    if (config.option<ConfigOptionFloat>("extruder_clearance_max_radius"))
        f.old_max_radius = float(config.opt_float("extruder_clearance_max_radius"));
    if (config.option<ConfigOptionFloat>("extruder_clearance_dist_to_rod"))
        f.old_distance_to_rod = float(config.opt_float("extruder_clearance_dist_to_rod"));
#endif
}

bool wants_settings_merge(const CliOptions& o) {
    auto has = [&](const char* key) { return o.given_flag(key); };
    return has("load_settings") || has("load_filaments") || has("uptodate") || has("uptodate_settings") ||
           has("uptodate_filaments") || has("load_defaultfila") || o.extra_config.has("filament_colour")
#ifndef ENGINE_ORCA
           || (has("estimate_mode") && o.cli.opt_bool("estimate_mode"))
#endif
        ;
}

/// The refusal for --uptodate when this package ships none of the "full"
/// system presets the step reads. The official step takes the project's own
/// machine, process and filament presets from machine_full/, process_full/ and
/// filament_full/ (BambuStudio.cpp 2723-2786 at 5873b5f; OrcaSlicer.cpp
/// 2350-2420 at 31f6803) and, when a file is not there, leaves those settings
/// alone: exit 0, nothing updated. Saying so beats letting a run believe the
/// project was brought up to date, and the flags that do the same thing by
/// hand are named.
static std::string uptodate_system_presets_missing() {
    return "--uptodate: this engine ships no machine_full, process_full or filament_full system presets ("
           + resources_dir() + "/profiles), so the project's own presets cannot be read and nothing is updated."
           " Give the presets instead: --uptodate-settings <the project's machine file>;<the project's process file>,"
           " and --uptodate-filaments <one file per filament, in the project's order>; or load other settings with"
           " --load-settings/--load-filaments.";
}

StepResult merge_loaded_settings(const CliOptions& o, const ProjectFacts& facts_in, const std::string& profiles_dir,
                                 DynamicPrintConfig& m_print_config, DynamicPrintConfig& m_extra_config,
                                 const std::vector<Preset*>& project_presets, SettingsMerge& out) {
    ProjectFacts facts = facts_in;
    const auto& cfg = o.cli;
    const std::vector<std::string>& load_configs       = cfg.option<ConfigOptionStrings>("load_settings")->values;
    const std::vector<std::string>& uptodate_configs   = cfg.option<ConfigOptionStrings>("uptodate_settings")->values;
    const std::vector<std::string>& uptodate_filaments = cfg.option<ConfigOptionStrings>("uptodate_filaments")->values;
    std::vector<std::string> load_filaments            = cfg.option<ConfigOptionStrings>("load_filaments")->values;
    const bool up_config_to_date        = o.given_flag("uptodate") && cfg.opt_bool("uptodate");
    bool use_first_fila_as_default      = o.given_flag("load_defaultfila") && cfg.opt_bool("load_defaultfila");
    const bool skip_modified_gcodes     = o.given_flag("skip_modified_gcodes") && cfg.opt_bool("skip_modified_gcodes");
#ifndef ENGINE_ORCA
    const bool estimate_mode            = o.given_flag("estimate_mode") && cfg.opt_bool("estimate_mode");
#else
    const bool estimate_mode            = false;
#endif
    const bool is_bbl_3mf = facts.is_bbl_3mf;
    std::string why;

    std::string new_printer_name, new_printer_system_name, new_process_name, new_process_system_name, printer_model,
        printer_model_id, new_default_process_name, different_process_setting;
    bool new_printer_config_is_system = true, new_process_config_is_system = true;
    std::vector<std::string> new_print_compatible_printers;
    DynamicPrintConfig load_process_config, load_machine_config;
    std::vector<std::string> upward_compatible_printers        = facts.upward_compatible_printers;
    std::vector<std::string> current_print_compatible_printers = facts.current_print_compatible_printers;
    int filament_count = facts.filament_count;

    // --load-settings: one machine and one process at most (B 2316-2406; O 1948-2023).
    for (const std::string& file : load_configs) {
        DynamicPrintConfig config;
        std::string config_type, config_name, filament_id, config_from;
        if (int ret = load_config_file(file, config, config_type, config_name, filament_id, config_from, why); ret)
            return fail(ret, why);
        if (config_type == "machine") {
            if (!new_printer_name.empty())
                return fail(CLI_CONFIG_FILE_ERROR, "--load-settings names two machine files; " + file +
                                                       " is the second; give one machine file");
            new_printer_name = config_name;
            if (config_from == "system") {
                new_printer_system_name = new_printer_name;
                new_printer_config_is_system = true;
            } else {
                new_printer_system_name = config.option<ConfigOptionString>("inherits", true)->value;
                new_printer_config_is_system = false;
            }
            config.set("printer_settings_id", new_printer_name, true);
            printer_model = config.option<ConfigOptionString>("printer_model", true)->value;
            printer_model_id = printer_model_id_for(printer_model);
            if (config.option<ConfigOptionString>("default_print_profile"))
                new_default_process_name = config.option<ConfigOptionString>("default_print_profile")->value;
            load_machine_config = std::move(config);
        } else if (config_type == "process") {
            if (!new_process_name.empty())
                return fail(CLI_CONFIG_FILE_ERROR, "--load-settings names two process files; " + file +
                                                       " is the second; give one process file");
            new_process_name = config_name;
            if (config_from == "system") {
                new_process_system_name = new_process_name;
                new_process_config_is_system = true;
            } else {
                new_process_system_name = config.option<ConfigOptionString>("inherits", true)->value;
                new_process_config_is_system = false;
            }
            config.set("print_settings_id", new_process_name, true);
            new_print_compatible_printers = config.option<ConfigOptionStrings>("compatible_printers", true)->values;
            if (!is_bbl_3mf && config.option<ConfigOptionStrings>("different_settings_to_system")) {
                const std::vector<std::string> diff_settings = config.option<ConfigOptionStrings>("different_settings_to_system")->values;
                if (!diff_settings.empty())
                    different_process_setting = diff_settings[0];
#ifndef ENGINE_ORCA
                config.erase("different_settings_to_system");   // B 2390
#endif
            }
            load_process_config = std::move(config);
        } else {
            return fail(CLI_CONFIG_FILE_ERROR, file + " is a filament file; give it with --load-filaments");
        }
    }

#ifndef ENGINE_ORCA
    // --estimate-mode with no --load-filaments: each slot's filament for the
    // new machine from filament_full (B 2408-2444). Without that folder no
    // preset is found, so the step refuses as the official does.
    if (estimate_mode && load_filaments.empty()) {
        std::string new_machine_bbl_tag;
        if (auto* fp = load_machine_config.option<ConfigOptionStrings>("default_filament_profile"); fp && !fp->values.empty()) {
            const std::string& def = fp->values[0];
            if (auto pos = def.find(" @BBL "); pos != std::string::npos)
                new_machine_bbl_tag = def.substr(pos + 6);
        }
        if (new_machine_bbl_tag.empty())
            return fail(CLI_INVALID_PARAMS, "--estimate-mode needs --load-settings with a Bambu Lab machine file that names its default filament");
        const std::string filament_full_dir = resources_dir() + "/profiles/BBL/filament_full/";
        for (size_t i = 0; i < facts.converted_filaments_system_name.size(); i++) {
            const std::string candidate = facts.converted_filaments_system_name[i];
            const std::string path = filament_full_dir + candidate + ".json";
            load_filaments.push_back(boost::filesystem::exists(path) ? path : std::string());
        }
        use_first_fila_as_default = true;
    }
#endif

    // --load-filaments (B 2446-2571; O 2024-2111).
    int load_filament_count = int(load_filaments.size());
    std::vector<int> load_filaments_index;
    std::set<std::string> load_filaments_set;
    bool disable_wipe_tower_after_mapping = false;
    std::vector<DynamicPrintConfig> load_filaments_config;
    std::vector<std::string> load_filaments_id, load_filaments_name, load_filaments_inherit;
    int current_index = 0;
    std::string default_load_fila_name, default_load_fila_id, default_filament_file, default_filament_inherit;
    DynamicPrintConfig default_load_fila_config;
    if (use_first_fila_as_default) {
        for (int index = 0; index < load_filament_count; index++) {
            const std::string& file = load_filaments[index];
            if (default_filament_file.empty() && !file.empty()) {
                DynamicPrintConfig config;
                std::string config_type, config_name, filament_id, config_from;
                if (int ret = load_config_file(file, config, config_type, config_name, filament_id, config_from, why); ret)
                    return fail(ret, why);
                if (config_type != "filament")
                    return fail(CLI_CONFIG_FILE_ERROR, file + " is not a filament file; give it with --load-filaments");
                if (config_from == "User" || config_from == "user")
                    default_filament_inherit = config.option<ConfigOptionString>("inherits", true)->value;
                default_filament_file    = file;
                default_load_fila_name   = config_name;
                default_load_fila_id     = filament_id;
                default_load_fila_config = std::move(config);
                break;
            }
        }
        if (load_filament_count > 0 && default_filament_file.empty()) {
            // The flags the user gave decide the sentence: --load-defaultfila
            // never was given when --estimate-mode filled this list itself
            // (BambuStudio.cpp 2408-2444 takes each slot from filament_full), so
            // blaming that flag would name a flag nobody gave. The cause there
            // is this package's missing filament_full folder.
            if (!o.given_flag("load_defaultfila")) {
                std::string machine;
                if (const auto* fp = load_machine_config.option<ConfigOptionStrings>("default_filament_profile");
                    fp && !fp->values.empty())
                    machine = " (" + fp->values[0] + ")";
                return fail(CLI_CONFIG_FILE_ERROR,
                            "--estimate-mode: this engine ships no filament_full profiles for the machine's default"
                            " filament" + machine + ", so the temperatures cannot be estimated. Give the filaments"
                            " with --load-filaments.");
            }
            return fail(CLI_CONFIG_FILE_ERROR, "--load-defaultfila: none of the --load-filaments files could be loaded; "
                                               "check the paths given to --load-filaments");
        }
    }
    for (int index = 0; index < load_filament_count; index++) {
        const std::string& file = load_filaments[index];
        current_index++;
        if (!file.empty()) {
            DynamicPrintConfig config;
            std::string config_type, config_name, filament_id, config_from;
            if (int ret = load_config_file(file, config, config_type, config_name, filament_id, config_from, why); ret)
                return fail(ret, why);
            if (config_type != "filament")
                return fail(CLI_CONFIG_FILE_ERROR, file + " is not a filament file; give it with --load-filaments");
            std::string inherits;
            if (config_from == "User" || config_from == "user") {
                inherits = config.option<ConfigOptionString>("inherits", true)->value;
#ifndef ENGINE_ORCA
                // B 2526-2546: the per-variant values a user filament leaves
                // out come from its system parent in filament_full.
                if (!inherits.empty()) {
                    const std::string parent_path = full_preset_path("filament_full", inherits);
                    if (boost::filesystem::exists(parent_path)) {
                        DynamicPrintConfig parent_config;
                        std::string ptype, pname, pid, pfrom, pwhy;
                        if (!load_config_file(parent_path, parent_config, ptype, pname, pid, pfrom, pwhy) && ptype == "filament") {
                            for (const auto& opt_key : filament_options_with_variant) {
                                ConfigOption* opt = config.option(opt_key);
                                ConfigOption* parent_opt = parent_config.option(opt_key);
                                if (opt && parent_opt)
                                    static_cast<ConfigOptionVectorBase*>(opt)->set_with_default(static_cast<const ConfigOptionVectorBase*>(parent_opt));
                            }
                        }
                    }
                }
#endif
            }
            load_filaments_inherit.push_back(inherits);
            load_filaments_id.push_back(filament_id);
            load_filaments_name.push_back(config_name);
            load_filaments_config.push_back(std::move(config));
            load_filaments_index.push_back(current_index);
            load_filaments_set.emplace(config_name);
        } else if (use_first_fila_as_default) {
            load_filaments_id.push_back(default_load_fila_id);
            load_filaments_name.push_back(default_load_fila_name);
            load_filaments_config.push_back(default_load_fila_config);
            load_filaments_index.push_back(current_index);
            load_filaments_inherit.push_back(default_filament_inherit);
            load_filaments_set.emplace(default_load_fila_name);
        }
    }

#ifndef ENGINE_ORCA
    // The colours of an assemble list's OBJ files (B 2573-2620): one filament
    // per colour, copies of the first loaded filament where too few were
    // given, each coloured as the OBJ is. OrcaSlicer's CLI leaves them out.
    if (!out.input_obj_colours.empty()) {
        const int input_color_count = int(out.input_obj_colours.size());
        if (load_filament_count == 0 || load_filaments_config.empty())
            return fail(CLI_INVALID_PARAMS, "The assemble list's OBJ files carry colours, which become filaments: "
                                            "give the filaments with --load-filaments.");
        if (m_extra_config.option<ConfigOptionStrings>("filament_colour"))
            return fail(CLI_INVALID_PARAMS, "The assemble list's OBJ files set the filament colours; "
                                            "--filament-colour cannot be given with them.");
        if (load_filament_count < input_color_count) {
            const int delta = input_color_count - load_filament_count;
            for (int index = 0; index < delta; index++) {
                load_filaments_id.push_back(load_filaments_id[0]);
                load_filaments_name.push_back(load_filaments_name[0]);
                load_filaments_config.push_back(load_filaments_config[0]);
                load_filaments_index.push_back(index + 1 + load_filament_count);
                load_filaments_inherit.push_back(load_filaments_inherit[0]);
            }
            load_filament_count = input_color_count;
        }
        std::vector<std::string>& filament_colors =
            m_extra_config.option<ConfigOptionStrings>("filament_colour", true)->values;
        filament_colors.resize(input_color_count);
        for (int index = 0; index < input_color_count; index++) {
            std::ostringstream stream;
            for (int c = 0; c < 4; c++)
                stream << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
                       << std::clamp((int) (out.input_obj_colours[index][c] * 255.f), 0, 255);
            filament_colors[index] = "#" + stream.str();
        }
    }
#endif
    if (filament_count == 0)
        filament_count = load_filament_count;
    if (is_bbl_3mf && load_filament_count > 0 && load_filaments_set.size() == 1 && !estimate_mode)
        disable_wipe_tower_after_mapping = true;   // B 2630-2634; O 2252-2256

    // --uptodate (B 2636-2870; O 2258-2420).
    bool fetch_compatible_values = false, fetch_upward_values = false;
    if (up_config_to_date && !is_bbl_3mf)
        // The official step is inside its own `is_bbl_3mf` test
        // (BambuStudio.cpp 2638 at 5873b5f, OrcaSlicer.cpp 2290 at 31f6803): it
        // updates a Bambu Studio project to this engine's own presets. On
        // anything else there is nothing to update, and saying so is the only
        // honest answer.
        return fail(CLI_INVALID_PARAMS,
                    "--uptodate updates a Bambu Studio project to this engine's own system presets, and this input is"
                    " not one. Give the settings to load instead: --load-settings and --load-filaments, or the"
                    " project's own presets with --uptodate-settings and --uptodate-filaments.");
    if (is_bbl_3mf && up_config_to_date) {
        // How many of the project's own system preset files this package has:
        // machine_full, process_full and filament_full. With none of them the
        // official step updates nothing at all (B 2723-2786; O 2350-2420) and
        // says nothing, and the refusal below says so instead.
        int system_files_read = 0;
        if (!uptodate_configs.empty()) {
            for (const std::string& file : uptodate_configs) {
                DynamicPrintConfig config;
                std::string config_type, config_name, filament_id, config_from;
                if (int ret = load_config_file(file, config, config_type, config_name, filament_id, config_from, why); ret)
                    return fail(ret, why);
                if (config_type == "machine") {
                    if (config_name != facts.current_printer_system_name)
                        return fail(CLI_CONFIG_FILE_ERROR, "--uptodate-settings: " + file + " is machine '" + config_name +
                                                           "', not the project's '" + facts.current_printer_system_name +
                                                           "'; give the project's own machine file");
                    upward_compatible_printers = config.option<ConfigOptionStrings>("upward_compatible_machine", true)->values;
                    if (new_printer_name.empty() && !facts.current_printer_system_name.empty()) {
                        config.set("printer_settings_id", config_name, true);
                        printer_model = config.option<ConfigOptionString>("printer_model", true)->value;
                        printer_model_id = printer_model_id_for(printer_model);
                        int orig_w = 0, orig_d = 0, orig_h = 0;
                        const Pointfs orig_area = config.option<ConfigOptionPoints>("printable_area", true)->values;
                        if (orig_area.size() >= 4) {
                            orig_w = (int)(orig_area[2].x() - orig_area[0].x());
                            orig_d = (int)(orig_area[2].y() - orig_area[0].y());
                        }
                        orig_h = (int)config.opt_float("printable_height");
                        if (orig_w > 0 && orig_d > 0 && orig_h > 0 &&
                            (facts.old_printable_width > orig_w || facts.old_printable_depth > orig_d || facts.old_printable_height > orig_h))
                            return fail(CLI_MODIFIED_PARAMS_TO_PRINTER,
                                        "Printer Settings: the printable size {" + std::to_string(facts.old_printable_width) + ", " +
                                        std::to_string(facts.old_printable_depth) + ", " + std::to_string(facts.old_printable_height) +
                                        "} exceeds the default size.");
                        load_machine_config = std::move(config);
                    }
                } else if (config_type == "process") {
                    if (config_name != facts.current_process_system_name)
                        return fail(CLI_CONFIG_FILE_ERROR, "--uptodate-settings: " + file + " is process '" + config_name +
                                                           "', not the project's '" + facts.current_process_system_name +
                                                           "'; give the project's own process file");
                    current_print_compatible_printers = config.option<ConfigOptionStrings>("compatible_printers", true)->values;
                    if (new_process_name.empty() && !facts.current_process_system_name.empty()) {
                        config.set("print_settings_id", config_name, true);
                        load_process_config = std::move(config);
                    }
                } else
                    return fail(CLI_CONFIG_FILE_ERROR, "--uptodate-settings: " + file +
                                                           " is not a machine or process file; give a machine or process file");
            }
        } else {
            // The project's own system presets from machine_full / process_full
            // (B 2723-2786); missing folders keep the file's settings.
            if (new_printer_name.empty() && !facts.current_printer_system_name.empty()) {
                const std::string path = full_preset_path("machine_full", facts.current_printer_system_name);
                if (boost::filesystem::exists(path)) {
                    DynamicPrintConfig config;
                    std::string config_type, config_name, filament_id, config_from;
                    if (int ret = load_config_file(path, config, config_type, config_name, filament_id, config_from, why); ret)
                        return fail(ret, why);
                    upward_compatible_printers = config.option<ConfigOptionStrings>("upward_compatible_machine", true)->values;
                    config.set("printer_settings_id", config_name, true);
                    printer_model = config.option<ConfigOptionString>("printer_model", true)->value;
                    printer_model_id = printer_model_id_for(printer_model);
                    load_machine_config = std::move(config);
                    ++system_files_read;
                } else
                    emit({{"event", "preset_warning"}, {"tag", "SystemPresetFileMissing"}, {"path", path},
                          {"message", "--uptodate: no system file for printer '" + facts.current_printer_system_name +
                                      "'; the project's printer settings are kept"}});
            } else
                fetch_upward_values = true;
            if (new_process_name.empty() && !facts.current_process_system_name.empty()) {
                const std::string path = full_preset_path("process_full", facts.current_process_system_name);
                if (boost::filesystem::exists(path)) {
                    DynamicPrintConfig config;
                    std::string config_type, config_name, filament_id, config_from;
                    if (int ret = load_config_file(path, config, config_type, config_name, filament_id, config_from, why); ret)
                        return fail(ret, why);
                    current_print_compatible_printers = config.option<ConfigOptionStrings>("compatible_printers", true)->values;
                    config.set("print_settings_id", config_name, true);
                    load_process_config = std::move(config);
                    ++system_files_read;
                } else
                    emit({{"event", "preset_warning"}, {"tag", "SystemPresetFileMissing"}, {"path", path},
                          {"message", "--uptodate: no system file for process '" + facts.current_process_system_name +
                                      "'; the project's process settings are kept"}});
            } else
                fetch_compatible_values = true;
        }
        if (load_filament_count == 0) {
            if (!uptodate_filaments.empty()) {
                if (uptodate_filaments.size() != (size_t)filament_count)
                    return fail(CLI_INVALID_PARAMS, "--uptodate-filaments gives " + std::to_string(uptodate_filaments.size()) +
                                                    " file(s) for " + std::to_string(filament_count) +
                                                    " filament(s); give one file per filament, in the project's order");
                for (int index = 0; index < filament_count; index++) {
                    const std::string& file = uptodate_filaments[index];
                    DynamicPrintConfig config;
                    std::string config_type, config_name, filament_id, config_from;
                    if (int ret = load_config_file(file, config, config_type, config_name, filament_id, config_from, why); ret)
                        return fail(ret, why);
                    if (config_type != "filament")
                        return fail(CLI_CONFIG_FILE_ERROR, file + " is not a filament file; give it with --load-filaments");
                    const bool matches = index < int(facts.current_filaments_system_name.size()) &&
                                         (config_name == facts.current_filaments_system_name[index]
#ifndef ENGINE_ORCA
                                          || config_name == facts.converted_filaments_system_name[index]
#endif
                                         );
                    if (!matches)
                        return fail(CLI_CONFIG_FILE_ERROR, "--uptodate-filaments: " + file + " is filament '" + config_name +
                                                           "', not the project's filament " + std::to_string(index + 1) +
                                                           "; give the project's own filament files, in the project's order");
                    load_filaments_id.push_back(filament_id);
                    load_filaments_name.push_back(config_name);
                    load_filaments_config.push_back(std::move(config));
                    load_filaments_index.push_back(index + 1);
                    load_filaments_inherit.push_back(config_name);
                }
            } else {
                current_index = 0;
#ifdef ENGINE_ORCA
                const std::vector<std::string>& names = facts.current_filaments_system_name;
#else
                const std::vector<std::string>& names = facts.converted_filaments_system_name;
#endif
                for (size_t index = 0; index < names.size(); index++) {
                    const std::string path = full_preset_path("filament_full", names[index]);
                    current_index++;
                    if (!boost::filesystem::exists(path))
                        continue;
                    DynamicPrintConfig config;
                    std::string config_type, config_name, filament_id, config_from;
                    if (int ret = load_config_file(path, config, config_type, config_name, filament_id, config_from, why); ret)
                        return fail(ret, why);
                    if (config_type != "filament")
                        return fail(CLI_CONFIG_FILE_ERROR, path + " is not a filament file; give it with --load-filaments");
                    load_filaments_id.push_back(filament_id);
                    load_filaments_name.push_back(config_name);
                    load_filaments_config.push_back(std::move(config));
                    load_filaments_index.push_back(current_index);
                    load_filaments_inherit.push_back(config_name);
                    ++system_files_read;
                }
            }
        }
        // Nothing at all could be read from the presets this package ships
        // (machine_full, process_full, filament_full), and no file was given
        // for the step to read: the official step then updates nothing and
        // exits 0, which reads as "the project is up to date". Say what is
        // missing and what to give instead. A run that named its own files
        // never gets here — the checks above refuse with their own sentences.
        if (uptodate_configs.empty() && uptodate_filaments.empty() && system_files_read == 0)
            return fail(CLI_INVALID_PARAMS, uptodate_system_presets_missing());
    } else if (is_bbl_3mf) {
        fetch_upward_values = true;
        fetch_compatible_values = true;
    }
    // B 2872-2914; O 2422-2486.
    if (fetch_upward_values && !facts.current_printer_system_name.empty()) {
        const std::string path = full_preset_path("machine_full", facts.current_printer_system_name);
        if (boost::filesystem::exists(path)) {
            DynamicPrintConfig config;
            std::string config_type, config_name, filament_id, config_from;
            if (int ret = load_config_file(path, config, config_type, config_name, filament_id, config_from, why); ret)
                return fail(ret, why);
            upward_compatible_printers = config.option<ConfigOptionStrings>("upward_compatible_machine", true)->values;
        }
    }
    if (fetch_compatible_values && !facts.current_process_system_name.empty()) {
        const std::string path = full_preset_path("process_full", facts.current_process_system_name);
        if (boost::filesystem::exists(path)) {
            DynamicPrintConfig config;
            std::string config_type, config_name, filament_id, config_from;
            if (int ret = load_config_file(path, config, config_type, config_name, filament_id, config_from, why); ret)
                return fail(ret, why);
            current_print_compatible_printers = config.option<ConfigOptionStrings>("compatible_printers", true)->values;
        }
    }

    // The process must suit the printer (B 2916-3010; O 2488-2584).
    bool process_compatible = false, machine_upwards = false, machine_switch = false;
    auto contains = [](const std::vector<std::string>& list, const std::string& name) {
        return std::find(list.begin(), list.end(), name) != list.end();
    };
    if (!new_printer_name.empty()) {
        process_compatible = !new_process_name.empty() ? contains(new_print_compatible_printers, new_printer_system_name)
                                                       : contains(current_print_compatible_printers, new_printer_system_name);
    } else if (!new_process_name.empty()) {
        process_compatible = contains(new_print_compatible_printers, facts.current_printer_system_name);
    } else {
        process_compatible = contains(current_print_compatible_printers, facts.current_printer_system_name);
        if (!process_compatible && current_print_compatible_printers.empty())
            process_compatible = true;   // an old 3MF lists no compatible printers
    }
    if (!process_compatible && !new_printer_name.empty() && !facts.current_printer_name.empty() &&
        new_printer_name != facts.current_printer_name) {
        if (new_process_name.empty())
            process_compatible = true;
        machine_switch = true;
        if (!upward_compatible_printers.empty()) {
            if (contains(upward_compatible_printers, new_printer_system_name)) {
                process_compatible = true;
                machine_upwards = true;
            }
            if (!process_compatible)
                return fail(CLI_3MF_NEW_MACHINE_NOT_SUPPORTED,
                            "The project cannot move to printer '" + new_printer_name +
                                "'; give a process that printer suits (--load-settings or --process-preset), "
                                "or keep the project's own printer");
        }
    }
    if (!process_compatible)
        return fail(CLI_PROCESS_NOT_COMPATIBLE,
                    "The process does not suit the printer; give a process that printer suits "
                    "(--load-settings or --process-preset)");
#ifndef ENGINE_ORCA
    if (estimate_mode && (new_printer_name.empty() || facts.current_printer_name.empty() || new_printer_name == facts.current_printer_name))
        return fail(CLI_INVALID_PARAMS, "--estimate-mode needs --load-settings with a different printer");   // B 3041-3045
#endif
    out.upward_machines = upward_compatible_printers;

    // The "(auto)" process the exported project carries after a printer change
    // (B 3049-3095; O 2585-2632).
    if (is_bbl_3mf && machine_switch) {
        const Preset* current_preset = nullptr;
        for (const Preset* p : project_presets)
            if (p && p->name == facts.current_process_name) {
                current_preset = p;
                break;
            }
        auto new_preset = std::make_shared<Preset>(Preset::TYPE_PRINT, facts.current_process_name);
        if (current_preset) {
            *new_preset = *current_preset;
            std::vector<std::string>& compatible = new_preset->config.option<ConfigOptionStrings>("compatible_printers", true)->values;
            if (!contains(compatible, new_printer_system_name))
                compatible.push_back(new_printer_system_name);
        } else {
            new_preset->config.apply_only(m_print_config, Preset::print_options());
            std::vector<std::string>& compatible = new_preset->config.option<ConfigOptionStrings>("compatible_printers", true)->values;
            compatible = current_print_compatible_printers;
            compatible.push_back(new_printer_system_name);
            if (facts.current_process_system_name != facts.current_process_name)
                new_preset->config.option<ConfigOptionString>("inherits", true)->value = facts.current_process_system_name;
            new_preset->is_project_embedded = true;
            new_preset->type = Preset::TYPE_PRINT;
        }
        new_preset->name += "(auto)";
        new_preset->config.set("print_settings_id", new_preset->name, true);
        m_print_config.set("print_settings_id", new_preset->name, true);
        out.new_preset = new_preset;
    }

    // The two lists carry one entry per preset in the project's order — process,
    // filament 1 .. filament N, printer (B 3208-3210). The desktop collects them
    // the same way and writes the key only when some entry is non-empty
    // (PresetBundle.cpp 3490-3523, add_if_some_non_empty).
    //
    // filament_count is the count the merge works from: the project 3MF's own
    // filaments, or the files --load-filaments brings (B 2627-2628). It stays 0
    // when the roster comes from the named presets (--filament-preset) and no
    // settings file names the filaments, which is this command line's whole
    // preset path: the lists were then sized (0 + 2) and carried two empty
    // entries. The official CLI reads them as N + 2 entries, sizes the filament
    // system names to size - 2 (B 2001-2040) and then reads
    // converted_filaments_system_name[f_index] for every filament (B 2043-2046):
    // a two-entry list is an out-of-bounds read, SIGSEGV (rc 139) on every
    // --slice N of the exported project. Size them from the roster the settings
    // already carry instead, exactly as the file-fed path already did.
    int inherits_count = filament_count;
    if (inherits_count == 0)
        for (const char* key : {"filament_colour", "filament_settings_id", "filament_ids",
                                "filament_type", "filament_diameter"})
            if (const auto* vec = dynamic_cast<const ConfigOptionVectorBase*>(m_print_config.option(key, false)))
                inherits_count = std::max(inherits_count, int(vec->size()));

    std::vector<std::string>& different_settings = m_print_config.option<ConfigOptionStrings>("different_settings_to_system", true)->values;
    std::vector<std::string>& inherits_group     = m_print_config.option<ConfigOptionStrings>("inherits_group", true)->values;
    inherits_group.resize(inherits_count + 2, std::string());
    different_settings.resize(inherits_count + 2, std::string());
    if (!is_bbl_3mf && !different_process_setting.empty())
        different_settings[0] = different_process_setting;

    std::vector<int> new_variant_index;
    bool variant_count_changed = false;
    // The machine into the print settings (B 3216-3338; O 2752-2870).
    if (!new_printer_name.empty() || up_config_to_date) {
        std::vector<std::string> different_keys;
        if (new_printer_name.empty()) {
            if (!different_settings.empty())
                unescape_strings_cstyle(different_settings[inherits_count + 1], different_keys);
        } else {
            different_settings[inherits_count + 1] = "";
            inherits_group[inherits_count + 1] = new_printer_config_is_system ? "" : new_printer_system_name;
        }
        std::set<std::string> different_keys_set(different_keys.begin(), different_keys.end());
        int ret = 0;
        load_default_gcodes_to_config(load_machine_config, Preset::TYPE_PRINTER);
        if (new_printer_name.empty()) {
            const size_t diff_keys_size = different_keys_set.size();
            compute_variant_index(m_print_config, load_machine_config, "printer_extruder_id", "printer_extruder_variant",
                                  new_variant_index, variant_count_changed);
            ret = update_full_config(m_print_config, load_machine_config, different_keys_set, variant_count_changed,
                                     printer_options_with_variant_1, printer_options_with_variant_2, new_variant_index, false,
                                     skip_modified_gcodes);
            if (diff_keys_size != different_keys_set.size())
                different_settings[inherits_count + 1] =
                    escape_strings_cstyle(std::vector<std::string>(different_keys_set.begin(), different_keys_set.end()));
        } else {
            ret = update_full_config(m_print_config, load_machine_config, different_keys_set, variant_count_changed,
                                     printer_options_with_variant_1, printer_options_with_variant_2, new_variant_index, true);
            if (new_printer_name != facts.current_printer_name) {
                // The printer's safe machine limits from cli_config.json (B 3263-3331).
                const std::string cli_config_file = resources_dir() + "/profiles/BBL/cli_config.json";
                if (boost::filesystem::exists(cli_config_file)) {
                    try {
                        json root_json;
                        boost::nowide::ifstream ifs(cli_config_file);
                        ifs >> root_json;
                        if (root_json.contains("printer") && !printer_model.empty() && root_json["printer"].contains(printer_model) &&
                            root_json["printer"][printer_model].contains("machine_limits")) {
                            const auto params = root_json["printer"][printer_model]["machine_limits"].get<std::map<std::string, std::string>>();
                            for (const auto& [param, value] : params) {
                                std::string key = param;
                                key.replace(0, 8, "machine_max");
                                if (auto* option = m_print_config.option<ConfigOptionFloats>(key)) {
                                    ConfigOptionFloats new_option;
                                    new_option.deserialize(value);
                                    for (size_t i = 0; i < option->size(); i++)
                                        if (i < new_option.size() && new_option.values[i] != 0.f && new_option.values[i] < option->values[i])
                                            option->values[i] = new_option.values[i];
                                }
                            }
                        }
                    } catch (...) {
                    }
                }
            }
        }
        if (ret)
            return fail(ret, "Merging the machine settings failed.");
    }

    int new_extruder_count = 1, new_printer_variant_count = 1;
    bool new_is_multi_extruder = false;
    std::vector<std::string> new_printer_extruder_variants;
#ifdef ENGINE_ORCA
    // O 995-1000 of the block: the extruder count from the merged nozzles.
    if (m_print_config.option<ConfigOptionFloats>("nozzle_diameter")) {
        new_extruder_count = int(m_print_config.option<ConfigOptionFloats>("nozzle_diameter")->values.size());
        new_is_multi_extruder = new_extruder_count > 1;
        new_printer_extruder_variants = m_print_config.option<ConfigOptionStrings>("printer_extruder_variant", true)->values;
        new_printer_variant_count = int(new_printer_extruder_variants.size());
    }
#endif

    // The process into the print settings (B 3340-3406; O 2881-2945).
    int current_print_variant_count = facts.current_print_variant_count;
    std::vector<std::string>& print_compatible_printers = m_print_config.option<ConfigOptionStrings>("print_compatible_printers", true)->values;
    if (!new_process_name.empty() || up_config_to_date) {
        std::vector<std::string> different_keys;
        if (new_process_name.empty()) {
            if (!different_settings.empty()) {
                unescape_strings_cstyle(different_settings[0], different_keys);
                auto it = std::find(different_keys.begin(), different_keys.end(), "compatible_printers");
                if (it != different_keys.end()) {
                    *it = different_keys.back();
                    different_keys.pop_back();
                    different_settings[0] = escape_strings_cstyle(different_keys);
                }
            }
            print_compatible_printers = current_print_compatible_printers;
        } else {
#ifdef ENGINE_ORCA
            different_settings[0] = "";
#else
            different_settings[0] = different_process_setting;
#endif
            inherits_group[0] = new_process_config_is_system ? "" : new_process_system_name;
            print_compatible_printers = new_print_compatible_printers;
        }
        std::set<std::string> different_keys_set(different_keys.begin(), different_keys.end());
        int ret = 0;
        load_default_gcodes_to_config(load_process_config, Preset::TYPE_PRINT);
        if (new_process_name.empty()) {
            const size_t diff_keys_size = different_keys_set.size();
            compute_variant_index(m_print_config, load_process_config, "print_extruder_id", "print_extruder_variant",
                                  new_variant_index, variant_count_changed);
            ret = update_full_config(m_print_config, load_process_config, different_keys_set, variant_count_changed,
                                     print_options_with_variant, empty_options, new_variant_index, false, skip_modified_gcodes);
            if (diff_keys_size != different_keys_set.size())
                different_settings[0] = escape_strings_cstyle(std::vector<std::string>(different_keys_set.begin(), different_keys_set.end()));
        } else {
            ret = update_full_config(m_print_config, load_process_config, different_keys_set, variant_count_changed,
                                     print_options_with_variant, empty_options, new_variant_index, true);
        }
        current_print_variant_count = int(m_print_config.option<ConfigOptionStrings>("print_extruder_variant", true)->values.size());
        if (ret)
            return fail(ret, "Merging the process settings failed.");
    }

    // Nozzle volume types of the new setup (B 3408-3458; O 2947-2972).
    std::vector<NozzleVolumeType> new_nozzle_volume_type;
#ifndef ENGINE_ORCA
    sync_nozzle_volume_type_to_extruder_count(m_print_config, m_extra_config.has("nozzle_volume_type"));
    const bool different_extruder = m_print_config.support_different_extruders(new_extruder_count);
    (void)different_extruder;
    new_is_multi_extruder = new_extruder_count > 1;
    new_printer_extruder_variants = m_print_config.option<ConfigOptionStrings>("printer_extruder_variant", true)->values;
    new_printer_variant_count = int(new_printer_extruder_variants.size());
#endif
    if (m_extra_config.has("nozzle_volume_type")) {
        if (auto* opt = dynamic_cast<const ConfigOptionEnumsGeneric*>(m_extra_config.option("nozzle_volume_type")))
            for (int v : opt->values)
                new_nozzle_volume_type.push_back(NozzleVolumeType(v));
    } else {
#ifndef ENGINE_ORCA
        auto* opt_print_nvt = dynamic_cast<const ConfigOptionEnumsGeneric*>(m_print_config.option("nozzle_volume_type"));
        if (opt_print_nvt && opt_print_nvt->values.size() >= size_t(new_extruder_count)) {
            for (int i = 0; i < new_extruder_count; i++)
                new_nozzle_volume_type.push_back(NozzleVolumeType(opt_print_nvt->values[i]));
        } else {
            if (!machine_switch)
                for (int v : facts.current_nozzle_volume_type)
                    new_nozzle_volume_type.push_back(NozzleVolumeType(v));
            new_nozzle_volume_type.resize(new_extruder_count, nvtStandard);
        }
#else
        new_nozzle_volume_type.resize(new_extruder_count, nvtStandard);
#endif
    }
    new_nozzle_volume_type.resize(std::max<size_t>(new_nozzle_volume_type.size(), size_t(new_extruder_count)), nvtStandard);
    std::vector<std::string> new_extruder_variants(new_extruder_count, "");
    const auto* opt_extruder_type = dynamic_cast<const ConfigOptionEnumsGeneric*>(m_print_config.option("extruder_type"));
    for (int e = 0; e < new_extruder_count; e++) {
        const ExtruderType extruder_type = opt_extruder_type ? (ExtruderType)(opt_extruder_type->get_at(e)) : etDirectDrive;
        new_extruder_variants[e] = get_extruder_variant_string(extruder_type, new_nozzle_volume_type[e]);
    }
    std::vector<NozzleVolumeType> current_nozzle_volume_type;
    for (int v : facts.current_nozzle_volume_type)
        current_nozzle_volume_type.push_back(NozzleVolumeType(v));
    if (current_nozzle_volume_type.empty())
        current_nozzle_volume_type.resize(facts.current_extruder_count, nvtStandard);
    out.current_print_extruder_variants = m_print_config.option<ConfigOptionStrings>("print_extruder_variant", true)->values;

    if (machine_switch) {
        print_compatible_printers.push_back(new_printer_system_name);
        const std::string old_setting = different_settings[0];
        if (old_setting.empty())
            different_settings[0] = "compatible_printers";
        else {
            std::vector<std::string> keys;
            unescape_strings_cstyle(old_setting, keys);
            if (std::find(keys.begin(), keys.end(), "compatible_printers") == keys.end())
                different_settings[0] = old_setting + ";compatible_printers";
        }
        // Another extruder count: the process values follow the new
        // extruders, from the printer's default process (B 3480-3521).
#ifdef ENGINE_ORCA
        // OrcaSlicer ships no machine_full/process_full folder, and the branch
        // the official CLI takes here reads the new printer's default process
        // from one (OrcaSlicer.cpp 3003-3011: resources/orca/profiles/BBL/
        // process_full/<default_print_profile>.json), so its own 2.4.0-alpha
        // build refuses this printer change with "cannot find the settings
        // file". The desktop app switches printers from the presets its
        // package ships: PresetBundle::update_compatible keeps the current
        // process while the new printer is one it suits and otherwise selects
        // the best compatible system preset, the printer's
        // default_print_profile first (OrcaSlicer PresetBundle.cpp 5295-5330,
        // Preset.hpp 686-709, Tab.cpp 6130-6140 at 31f6803), whose settings
        // then are the process's (PresetBundle::full_config, PresetBundle.cpp
        // 3858-3862). BambuStudio keeps the official branch below, as its
        // package ships process_full.
        if (new_process_name.empty()) {
            // The process list to search is the NEW printer's vendor's, not
            // the project's: the desktop tags every loaded preset with the
            // vendor bundle it came from (PresetBundle.cpp 4992) and a printer
            // model resolves to its vendor by that bundle's machine_list
            // (PresetBundle.cpp 617-622). A Snapmaker machine preset switched
            // into a BBL project therefore reads the Snapmaker process list
            // (PresetBundle::find_preset_vendor, OrcaSlicer PresetBundle.cpp
            // 244-318 at 31f6803). With no bundle naming the preset the
            // project's vendor list is searched, as before.
            std::string new_vendor = desktop_printer_vendor(profiles_dir, new_printer_system_name);
            if (new_vendor.empty())
                new_vendor = "BBL";
            const DesktopProcessSwitch picked = desktop_printer_switch_process(
                profiles_dir, new_vendor, new_printer_system_name, new_default_process_name, m_print_config,
                current_print_compatible_printers, facts.current_process_system_name);
            if (picked.replaced) {
                // The whole preset, less its bookkeeping key.
                t_config_option_keys keys;
                for (const t_config_option_key& key : picked.config.keys())
                    if (key != "print_settings_id")
                        keys.push_back(key);
                m_print_config.apply_only(picked.config, keys, true);
                // The process is now the selected preset: it names itself, as
                // the desktop's full config does (PresetBundle.cpp 4106).
                m_print_config.option<ConfigOptionString>("print_settings_id", true)->value = picked.name;
            }
        }
#else
        if (new_process_name.empty() && (facts.current_extruder_count != new_extruder_count ||
                                         current_print_variant_count != new_printer_variant_count)) {
            if (new_default_process_name.empty())
                return fail(CLI_CONFIG_FILE_ERROR,
                            "The new printer names no default process; give a process with --load-settings "
                            "or --process-preset");
            const std::string file_path = full_preset_path("process_full", new_default_process_name);
            DynamicPrintConfig config;
            std::string config_type, config_name, filament_id, config_from;
            if (int ret = load_config_file(file_path, config, config_type, config_name, filament_id, config_from, why); ret)
                return fail(ret, why);
            if (config_type != "process" || config_from != "system")
                return fail(CLI_CONFIG_FILE_ERROR, file_path + " is not a system process file; give a system "
                                                               "process preset from the package's profiles");
            int ret = 0;
            std::set<std::string> keys = print_options_with_variant;
            if (!facts.current_is_multi_extruder && new_is_multi_extruder && current_print_variant_count == 1)
                ret = m_print_config.update_values_from_single_to_multi(config, keys, "print_extruder_id", "print_extruder_variant");
            else
                ret = m_print_config.update_values_from_multi_to_multi(config, keys, "print_extruder_id", "print_extruder_variant",
                                                                       new_extruder_variants);
            if (ret)
                return fail(CLI_CONFIG_FILE_ERROR, "Moving the process values to the new extruders failed.");
        }
#endif
    }

    // The filaments into the print settings (B 3524-3757; O 3041-3264).
    if (load_filament_count > 0 || up_config_to_date) {
        std::vector<int> old_start_indice(filament_count, 0);
        std::vector<int> old_variant_counts(filament_count, 1), new_variant_counts;
        ConfigOptionInts* filament_self_index_opt = m_print_config.option<ConfigOptionInts>("filament_self_index");
        if (!filament_self_index_opt) {
            filament_self_index_opt = m_print_config.option<ConfigOptionInts>("filament_self_index", true);
            std::vector<int>& indice = filament_self_index_opt->values;
            indice.resize(filament_count);
            for (int i = 0; i < filament_count; i++)
                indice[i] = i + 1;
        }
        const std::vector<int> old_self_indice = filament_self_index_opt->values;
        int k = -1, current_filament = 0;
        for (size_t i = 0; i < old_self_indice.size(); i++) {
            if (old_self_indice[i] > current_filament) {
                current_filament = old_self_indice[i];
                if (k + 1 < filament_count) {
                    old_start_indice[++k] = int(i);
                    old_variant_counts[k] = 1;
                }
            } else if (k >= 0)
                old_variant_counts[k] = old_variant_counts[k] + 1;
        }
        new_variant_counts = old_variant_counts;
        for (size_t index = 0; index < load_filaments_config.size(); index++) {
            DynamicPrintConfig& config = load_filaments_config[index];
            const int filament_index = load_filaments_index[index];
            if (filament_index < 1 || filament_index > filament_count)
                continue;
            std::vector<std::string> different_keys;
            load_default_gcodes_to_config(config, Preset::TYPE_FILAMENT);
            if (load_filament_count > 0) {
                auto* opt_filament_settings = static_cast<ConfigOptionStrings*>(m_print_config.option("filament_settings_id", true));
                ConfigOptionString filament_name_setting(load_filaments_name[index]);
                if (int(opt_filament_settings->size()) < filament_count)
                    opt_filament_settings->resize(filament_count, &filament_name_setting);
                opt_filament_settings->set_at(&filament_name_setting, filament_index - 1, 0);
                config.erase("filament_settings_id");
#ifdef ENGINE_ORCA
                different_settings[filament_index] = "";   // O 3082-3083
#else
                if (config.option("different_settings_to_system")) {
                    const std::vector<std::string> fds = config.option<ConfigOptionStrings>("different_settings_to_system", true)->values;
                    different_settings[filament_index] = fds.empty() ? "" : fds[0];
                    config.erase("different_settings_to_system");
                } else
                    different_settings[filament_index] = "";
#endif
                inherits_group[filament_index] = load_filaments_inherit[index];
            } else if (!different_settings.empty()) {
                unescape_strings_cstyle(different_settings[filament_index], different_keys);
            }
            {
                auto* opt_filament_ids = static_cast<ConfigOptionStrings*>(m_print_config.option("filament_ids", true));
                ConfigOptionString filament_id_setting(load_filaments_id[index]);
                if (int(opt_filament_ids->size()) < filament_count)
                    opt_filament_ids->resize(filament_count, &filament_id_setting);
                opt_filament_ids->set_at(&filament_id_setting, filament_index - 1, 0);
            }
            ConfigOptionStrings* curr_variant_opt = m_print_config.option<ConfigOptionStrings>("filament_extruder_variant");
            if (!curr_variant_opt) {
                curr_variant_opt = m_print_config.option<ConfigOptionStrings>("filament_extruder_variant", true);
                curr_variant_opt->values.resize(filament_count, get_extruder_variant_string(etDirectDrive, nvtStandard));
            }
            const auto* new_variant_opt = dynamic_cast<const ConfigOptionStrings*>(config.option("filament_extruder_variant", true));
            const int new_variant_count = int(new_variant_opt->size());
            const int old_variant_count = old_variant_counts[filament_index - 1];
            std::vector<int> new_variant_indice(new_variant_count, -1);
            for (int i = 0; i < new_variant_count; i++)
                for (int j = old_start_indice[filament_index - 1]; j < old_start_indice[filament_index - 1] + old_variant_count &&
                                                                   j < int(curr_variant_opt->values.size()); j++)
                    if (curr_variant_opt->values[j] == new_variant_opt->values[i]) {
                        new_variant_indice[i] = j;
                        break;
                    }
            std::set<std::string> different_keys_set(different_keys.begin(), different_keys.end());
            const size_t diff_keys_size = different_keys_set.size();
            for (const t_config_option_key& opt_key : config.keys()) {
                const ConfigOption* source_opt = config.option(opt_key);
                if (source_opt == nullptr)
                    return fail(CLI_CONFIG_FILE_ERROR, "Filament " + std::to_string(filament_index) + " has no value for " + opt_key + ".");
                if (load_filament_count == 0 && !different_keys_set.empty()) {
                    auto iter = different_keys_set.find(opt_key);
                    if (iter != different_keys_set.end()) {
                        if (skip_modified_gcodes && gcodes_key_set.count(opt_key)) {
                            different_keys_set.erase(iter);
                        } else {
                            if (filament_options_with_variant.count(opt_key)) {
                                ConfigOption* opt = m_print_config.option(opt_key);
                                if (opt == nullptr)
                                    return fail(CLI_CONFIG_FILE_ERROR, "The project has no value for " + opt_key + ".");
                                auto* opt_vec_dst = static_cast<ConfigOptionVectorBase*>(opt);
                                auto* opt_vec_src = static_cast<const ConfigOptionVectorBase*>(source_opt);
#ifdef ENGINE_ORCA
                                opt_vec_dst->set_with_restore_2(opt_vec_src, new_variant_indice, old_start_indice[filament_index - 1], old_variant_count);
#else
                                opt_vec_dst->set_with_restore_2(opt_key, opt_vec_src, new_variant_indice, old_start_indice[filament_index - 1], old_variant_count);
#endif
                            }
                            continue;
                        }
                    }
                }
                if (source_opt->is_scalar()) {
                    if (opt_key == "compatible_printers_condition") {
                        auto* dst = static_cast<ConfigOptionStrings*>(m_print_config.option("compatible_machine_expression_group", true));
                        ConfigOptionString empty;
                        if (dst->size() == 0)
                            dst->resize(filament_count + 2, &empty);
                        dst->set_at(source_opt, filament_index, 0);
                    } else if (opt_key == "compatible_prints_condition") {
                        auto* dst = static_cast<ConfigOptionStrings*>(m_print_config.option("compatible_process_expression_group", true));
                        ConfigOptionString empty;
                        if (dst->size() == 0)
                            dst->resize(filament_count, &empty);
                        dst->set_at(source_opt, filament_index - 1, 0);
                    }
                    continue;
                }
                if (opt_key == "compatible_prints" || opt_key == "compatible_printers" || opt_key == "model_id" ||
                    opt_key == "dev_model_name" || opt_key == "filament_settings_id")
                    continue;
                ConfigOption* opt = m_print_config.option(opt_key, true);
                if (opt == nullptr)
                    return fail(CLI_CONFIG_FILE_ERROR, "Cannot take " + opt_key + " from filament " + std::to_string(filament_index) + ".");
                auto* opt_vec_dst = static_cast<ConfigOptionVectorBase*>(opt);
                auto* opt_vec_src = static_cast<const ConfigOptionVectorBase*>(source_opt);
                if (filament_options_with_variant.count(opt_key)) {
                    std::vector<int> temp_variant_indice(new_variant_count, -1);
#ifdef ENGINE_ORCA
                    opt_vec_dst->set_with_restore_2(opt_vec_src, temp_variant_indice, old_start_indice[filament_index - 1], old_variant_count, true);
#else
                    opt_vec_dst->set_with_restore_2(opt_key, opt_vec_src, temp_variant_indice, old_start_indice[filament_index - 1], old_variant_count, true);
#endif
                    if (opt_key == "filament_extruder_variant")
                        new_variant_counts[filament_index - 1] = int(opt_vec_src->size());
                } else
                    opt_vec_dst->set_at(opt_vec_src, filament_index - 1, 0);
            }
            if (old_variant_count != new_variant_count)
                for (int i = int(index) + 1; i < filament_count; i++)
                    old_start_indice[i] += new_variant_count - old_variant_count;
            if (diff_keys_size != different_keys_set.size())
                different_settings[filament_index] =
                    escape_strings_cstyle(std::vector<std::string>(different_keys_set.begin(), different_keys_set.end()));
        }
        if (m_print_config.option<ConfigOptionStrings>("filament_extruder_variant")) {
            std::vector<int>& indice = m_print_config.option<ConfigOptionInts>("filament_self_index", true)->values;
            const int index_size = int(m_print_config.option<ConfigOptionStrings>("filament_extruder_variant")->size());
            indice.resize(index_size, 1);
            int n = 0;
            for (int i = 0; i < filament_count; i++)
                for (int j = 0; j < new_variant_counts[i] && n < index_size; j++)
                    indice[n++] = i + 1;
        }
    }

    // The flush volumes, when the colours, the nozzles or the matrix change
    // (B 3759-3941; O 3266-3420).
    ConfigOptionStrings* selected_colors_option = m_extra_config.option<ConfigOptionStrings>("filament_colour");
    // The project's colours are the loaded settings' own only: the engine
    // default this command line seeds first is not one the official
    // m_print_config holds here (see SettingsMerge::project_has_filament_colour).
    // Deliberate difference: with more than one filament and no colours from
    // anywhere (neither loaded nor given with --filament-colour), the
    // official CLI goes on with one colour for several filaments and crashes
    // in automatic grouping (SIGSEGV, OrcaSlicer 2.4.0-alpha and BambuStudio
    // CLIs, H2D with two settings-file filaments); here the seeded colour is
    // kept, and the check below refuses the run (CLI_CONFIG_FILE_ERROR) as
    // before.
    const bool colours_loaded =
        out.project_has_filament_colour || (filament_count > 1 && selected_colors_option == nullptr);
    ConfigOptionStrings* project_colors_option =
        colours_loaded ? m_print_config.option<ConfigOptionStrings>("filament_colour") : nullptr;
    if ((!project_colors_option || project_colors_option->values.empty()) && selected_colors_option) {
        project_colors_option = m_print_config.option<ConfigOptionStrings>("filament_colour", true);
        // Created empty, then sized (B 3765-3767; O 3272-3274).
        project_colors_option->values.assign(filament_count, "#FFFFFF");
    }
    bool filament_color_changed = false;
    if (project_colors_option &&
        (selected_colors_option || !m_print_config.option<ConfigOptionFloats>("flush_volumes_matrix") ||
         facts.current_extruder_count != new_extruder_count || new_nozzle_volume_type != current_nozzle_volume_type)) {
        std::vector<std::string> selected_colors;
        if (selected_colors_option) {
            selected_colors = selected_colors_option->values;
            m_extra_config.erase("filament_colour");
            if (disable_wipe_tower_after_mapping) {
                std::set<std::string> color_set;
                for (const std::string& c : selected_colors)
                    if (!c.empty())
                        color_set.emplace(c);
                if (color_set.size() > 1)
                    disable_wipe_tower_after_mapping = false;
            }
        }
        std::vector<std::string>& project_colors = project_colors_option->values;
        const size_t project_filament_count = project_colors.size();
        if (project_filament_count > 0) {
            for (size_t index = 0; index < project_filament_count; index++) {
                if (selected_colors.size() > index && !selected_colors[index].empty()) {
                    unsigned char ori[4] = {}, neu[4] = {};
                    parse_color4(project_colors[index], ori);
                    parse_color4(selected_colors[index], neu);
                    if (ori[0] != neu[0] || ori[1] != neu[1] || ori[2] != neu[2] || ori[3] != neu[3])
                        filament_color_changed = true;
                    project_colors[index] = selected_colors[index];
                }
            }
            ConfigOptionBools* filament_is_support = m_print_config.option<ConfigOptionBools>("filament_is_support", true);
            const std::vector<int> min_flush_volumes = get_min_flush_volumes(m_print_config, 0);
            if (filament_is_support->size() != project_filament_count)
                return fail(CLI_CONFIG_FILE_ERROR, "filament_is_support has " + std::to_string(filament_is_support->size()) +
                                                   " values for " + std::to_string(project_filament_count) + " filament colours.");
            std::vector<double>& flush_vol_matrix = m_print_config.option<ConfigOptionFloats>("flush_volumes_matrix", true)->values;
            flush_vol_matrix.resize(project_filament_count * project_filament_count * new_extruder_count, 0.f);
#ifndef ENGINE_ORCA
            const std::vector<std::string>& flush_filament_ids = m_print_config.option<ConfigOptionStrings>("filament_ids", true)->values;
            auto get_flush_filament_id = [&flush_filament_ids](int idx) -> std::string {
                return (idx >= 0 && idx < (int)flush_filament_ids.size()) ? flush_filament_ids[idx] : std::string();
            };
#endif
            m_print_config.option<ConfigOptionFloats>("flush_multiplier", true)->values.resize(new_extruder_count, 1.f);
#ifndef ENGINE_ORCA
            m_print_config.option<ConfigOptionFloats>("flush_multiplier_fast", true)->values.resize(new_extruder_count, 1.2f);
#endif
            std::vector<int> nozzle_flush_dataset(new_extruder_count, 0);
            {
                std::vector<int> nozzle_flush_dataset_full = m_print_config.option<ConfigOptionIntsNullable>("nozzle_flush_dataset", true)->values;
                if (m_print_config.has("printer_extruder_variant"))
                    nozzle_flush_dataset_full.resize(new_printer_variant_count, 0);
                else
                    nozzle_flush_dataset_full.resize(1, 0);
                std::vector<int> extruders;
                if (m_print_config.has("extruder_type"))
                    extruders = m_print_config.option<ConfigOptionEnumsGeneric>("extruder_type")->values;
                else
                    extruders.resize(1, int(ExtruderType::etDirectDrive));
                std::vector<int> volume_types;
                if (m_print_config.has("nozzle_volume_type"))
                    volume_types = m_print_config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type")->values;
                else
                    volume_types.resize(1, int(NozzleVolumeType::nvtStandard));
                if (m_extra_config.has("nozzle_volume_type"))
                    volume_types = m_extra_config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type")->values;
                extruders.resize(std::max<size_t>(extruders.size(), size_t(new_extruder_count)), extruders.empty() ? 0 : extruders.back());
                volume_types.resize(std::max<size_t>(volume_types.size(), size_t(new_extruder_count)), volume_types.empty() ? 0 : volume_types.back());
                for (int eidx = 0; eidx < new_extruder_count; ++eidx) {
                    int idx = 0;
                    if (m_print_config.has("printer_extruder_id") && m_print_config.has("printer_extruder_variant"))
                        idx = m_print_config.get_index_for_extruder(eidx + 1, "printer_extruder_id", ExtruderType(extruders[eidx]),
                                                                     NozzleVolumeType(volume_types[eidx]), "printer_extruder_variant");
                    if (idx >= 0 && idx < int(nozzle_flush_dataset_full.size()))
                        nozzle_flush_dataset[eidx] = nozzle_flush_dataset_full[idx];
                }
            }
            for (size_t nozzle_id = 0; nozzle_id < size_t(new_extruder_count); ++nozzle_id) {
                std::vector<double> flush_vol_mtx = get_flush_volumes_matrix(flush_vol_matrix, nozzle_id, new_extruder_count);
                for (size_t from_idx = 0; from_idx < project_filament_count; from_idx++) {
                    unsigned char from_rgb[4] = {};
                    parse_color4(project_colors[from_idx], from_rgb);
                    const bool is_from_support = filament_is_support->get_at(from_idx);
                    for (size_t to_idx = 0; to_idx < project_filament_count; to_idx++) {
                        const bool is_to_support = filament_is_support->get_at(to_idx);
                        if (from_idx == to_idx) {
                            flush_vol_mtx[project_filament_count * from_idx + to_idx] = 0.f;
                            continue;
                        }
                        int flushing_volume = 0;
                        if (is_to_support) {
                            flushing_volume = g_flush_volume_to_support;
                        } else {
                            unsigned char to_rgb[4] = {};
                            parse_color4(project_colors[to_idx], to_rgb);
                            FlushVolCalculator calculator(min_flush_volumes[from_idx], g_max_flush_volume, nozzle_flush_dataset[nozzle_id]);
#ifdef ENGINE_ORCA
                            flushing_volume = calculator.calc_flush_vol(from_rgb[3], from_rgb[0], from_rgb[1], from_rgb[2], to_rgb[3], to_rgb[0], to_rgb[1], to_rgb[2]);
#else
                            flushing_volume = calculator.calc_flush_vol(get_flush_filament_id(int(from_idx)), get_flush_filament_id(int(to_idx)),
                                                                        from_rgb[3], from_rgb[0], from_rgb[1], from_rgb[2], to_rgb[3], to_rgb[0], to_rgb[1], to_rgb[2]);
#endif
                            if (is_from_support)
                                flushing_volume = std::max(g_min_flush_volume_from_support, flushing_volume);
                        }
                        flush_vol_mtx[project_filament_count * from_idx + to_idx] = flushing_volume;
                    }
                }
                set_flush_volumes_matrix(flush_vol_matrix, flush_vol_mtx, nozzle_id, new_extruder_count);
            }
        }
    }

    out.machine_switch                   = machine_switch;
    out.machine_upwards                  = machine_upwards;
    out.disable_wipe_tower_after_mapping = disable_wipe_tower_after_mapping;
    out.filament_color_changed           = filament_color_changed;
    out.filament_count                   = filament_count;
    out.new_extruder_count               = new_extruder_count;
    out.new_printer_variant_count        = new_printer_variant_count;
    out.new_is_multi_extruder            = new_is_multi_extruder;
    out.new_printer_name                 = new_printer_name;
    out.new_printer_system_name          = new_printer_system_name;
    out.new_process_name                 = new_process_name;
    out.printer_model                    = printer_model;
    out.printer_model_id                 = printer_model_id;
    out.new_printer_extruder_variants    = new_printer_extruder_variants;
    out.load_process_config              = load_process_config;
    emit({{"event", "config_normalized"}, {"tag", "SettingsFilesMerged"},
          {"printer", new_printer_name}, {"process", new_process_name}, {"filaments", load_filaments_name},
          {"machine_switch", machine_switch},
          {"message", "Merged the settings files into the project's settings" +
                          std::string(machine_switch ? " (printer changed to " + new_printer_name + ")" : "")}});
    return StepResult();
}

bool disable_tower_after_mapping(const CliOptions& o, const SettingsMerge& merge, DynamicPrintConfig& print_config,
                                 const DynamicPrintConfig& extra_config) {
    if (!merge.disable_wipe_tower_after_mapping)
        return false;
    // The settings as the official reads them here: the flags laid over.
    DynamicPrintConfig view = print_config;
    view.apply(extra_config, true);
    const auto* wrapping_opt = view.option<ConfigOptionBool>("enable_wrapping_detection");
    const bool enable_wrapping_detect = wrapping_opt && wrapping_opt->value;
    const auto* wrapping_area = view.option<ConfigOptionPoints>("wrapping_exclude_area");
    if (enable_wrapping_detect && wrapping_area && !wrapping_area->values.empty())
        return false;
    const bool enable_timelapse = o.given_flag("enable_timelapse") && o.cli.opt_bool("enable_timelapse");
    const ConfigOption* timelapse_type_opt = view.option("timelapse_type");
    if (enable_timelapse && timelapse_type_opt && timelapse_type_opt->getInt() == TimelapseType::tlSmooth)
        return false;
    print_config.option<ConfigOptionBool>("enable_prime_tower", true)->value = false;
    // The process's list of settings that differ from its system preset
    // gains enable_prime_tower (different_settings_to_system[0]).
    std::vector<std::string>& different_settings =
        print_config.option<ConfigOptionStrings>("different_settings_to_system", true)->values;
    if (different_settings.empty())
        different_settings.resize(1);
    const std::string diff_settings = different_settings[0];
    if (diff_settings.empty())
        different_settings[0] = "enable_prime_tower";
    else {
        std::vector<std::string> different_keys;
        Slic3r::unescape_strings_cstyle(diff_settings, different_keys);
        if (std::find(different_keys.begin(), different_keys.end(), "enable_prime_tower") == different_keys.end())
            different_settings[0] = diff_settings + ";enable_prime_tower";
    }
    return true;
}

void update_object_configs_after_switch(const ProjectFacts& facts, const SettingsMerge& merge,
                                        const DynamicPrintConfig& print_config, Model& model) {
    if (!merge.machine_switch)
        return;
    const int current_print_variant_count = int(merge.current_print_extruder_variants.size());
    if (facts.current_extruder_count == merge.new_extruder_count && current_print_variant_count == merge.new_printer_variant_count)
        return;
    auto update = [&](DynamicPrintConfig& c) {
        c.update_values_from_multi_to_multi_2(merge.current_print_extruder_variants, merge.new_printer_extruder_variants,
                                              print_config, print_options_with_variant);
    };
    for (ModelObject* object : model.objects) {
        DynamicPrintConfig object_config = object->config.get();
        if (!object_config.empty()) {
            update(object_config);
            object->config.assign_config(std::move(object_config));
        }
        for (ModelVolume* v : object->volumes)
            if (v->is_model_part() || v->is_modifier()) {
                DynamicPrintConfig volume_config = v->config.get();
                if (!volume_config.empty()) {
                    update(volume_config);
                    v->config.assign_config(std::move(volume_config));
                }
            }
        for (auto& range : object->layer_config_ranges) {
            DynamicPrintConfig layer_config = range.second.get();
            if (!layer_config.empty()) {
                update(layer_config);
                range.second.assign_config(std::move(layer_config));
            }
        }
    }
}

StepResult apply_custom_gcodes(const CliOptions& o, int plate_to_slice, Model& model) {
    if (o.given_flag("load_custom_gcodes")) {
        const std::string file = o.cli.opt_string("load_custom_gcodes");
        if (!boost::filesystem::exists(file))
            return fail(CLI_FILE_NOTFOUND, "--load-custom-gcodes: cannot find " + file + "; check the file's path");
        try {
            json jj;
            boost::nowide::ifstream ifs(file);
            ifs >> jj;
            const int plate_id = plate_to_slice == 0 ? 0 : plate_to_slice - 1;
            CustomGCode::Info info;
            info.from_json(jj);
            std::map<int, CustomGCode::Info> custom_gcodes_map;
            custom_gcodes_map.emplace(plate_id, info);
            model.plates_custom_gcodes = custom_gcodes_map;
        } catch (const std::exception& ex) {
            return fail(CLI_CONFIG_FILE_ERROR, "--load-custom-gcodes: reading " + file + " failed: " + ex.what());
        }
    }
    if (o.given_flag("skip_modified_gcodes") && o.cli.opt_bool("skip_modified_gcodes")) {
        for (auto& [plate, gcode_info] : model.plates_custom_gcodes) {
            (void)plate;
            auto it = gcode_info.gcodes.begin();
            while (it != gcode_info.gcodes.end())
                it = it->type == CustomGCode::Custom ? gcode_info.gcodes.erase(it) : std::next(it);
        }
    }
    return StepResult();
}

namespace {

// The shared area of a multi-extruder printer: the bed box cut by every
// extruder's printable area, and the lowest extruder height
// (BambuStudio.cpp 4728-4756, 4258-4280; OrcaSlicer.cpp 4067-4093).
bool shared_extruder_box(const DynamicPrintConfig& config, int width, int depth, double height,
                         double& shared_width, double& shared_depth, double& shared_height) {
    const auto* areas   = config.option<ConfigOptionPointsGroups>("extruder_printable_area");
    const auto* heights = config.option<ConfigOptionFloatsNullable>("extruder_printable_height");
    if (!areas || !heights || areas->values.empty() || heights->values.size() != areas->values.size())
        return false;
    BoundingBox current_bbox({0, 0}, {width, depth});
    double temp_extruder_height = height;
    for (size_t i = 0; i < areas->values.size(); i++) {
        BoundingBox temp_bbox;
        for (const Vec2d& pt : areas->values[i])
            temp_bbox.merge({pt.x(), pt.y()});
        if (current_bbox.min.x() < temp_bbox.min.x()) current_bbox.min.x() = temp_bbox.min.x();
        if (current_bbox.min.y() < temp_bbox.min.y()) current_bbox.min.y() = temp_bbox.min.y();
        if (current_bbox.max.x() > temp_bbox.max.x()) current_bbox.max.x() = temp_bbox.max.x();
        if (current_bbox.max.y() > temp_bbox.max.y()) current_bbox.max.y() = temp_bbox.max.y();
        if (temp_extruder_height > heights->values[i]) temp_extruder_height = heights->values[i];
    }
    shared_width  = current_bbox.size().x();
    shared_depth  = current_bbox.size().y();
    shared_height = temp_extruder_height;
    return true;
}

// load_downward_settings_list_from_config (BambuStudio.cpp 1440-1488;
// OrcaSlicer.cpp 1137-1185): the printer's list in cli_config.json.
std::vector<std::string> downward_list_from_cli_config(const std::string& printer_name, const std::string& printer_model) {
    std::vector<std::string> list;
    const std::string file = resources_dir() + "/profiles/BBL/cli_config.json";
    if (!boost::filesystem::exists(file))
        return list;
    try {
        nlohmann::json root;
        boost::nowide::ifstream ifs(file);
        ifs >> root;
        if (root.contains("printer") && !printer_model.empty() && root["printer"].contains(printer_model)) {
            const nlohmann::json& model_json = root["printer"][printer_model];
            if (model_json.contains("downward_check") && model_json["downward_check"].contains(printer_name))
                list = model_json["downward_check"][printer_name].get<std::vector<std::string>>();
        }
    } catch (const std::exception&) {
        list.clear();
    }
    return list;
}

} // namespace

StepResult load_downward_printers(const CliOptions& o, const ProjectFacts& facts, std::vector<DownwardPrinter>& printers) {
    StepResult r;
    std::vector<std::string> files;
    if (const auto* given = o.cli.option<ConfigOptionStrings>("downward_settings"))
        files = given->values;
    const bool use_default = files.empty();
    if (use_default)
        files = downward_list_from_cli_config(facts.current_printer_system_name, facts.current_printer_model);
    for (const std::string& file : files) {
        const std::string path = use_default ? full_preset_path("machine_full", file) : file;
        if (use_default && !boost::filesystem::exists(path)) {
            emit({{"event", "preset_warning"}, {"tag", "SystemPresetFileMissing"}, {"path", path},
                  {"message", "--downward-check: no system file for printer '" + file + "'; it is left out of the check"}});
            continue;
        }
        DynamicPrintConfig config;
        std::string config_type, config_name, filament_id, config_from, why;
        if (int ret = load_config_file(path, config, config_type, config_name, filament_id, config_from, why); ret) {
            r.code    = ret;
            r.message = "--downward-settings: " + why;
            return r;
        }
        if (config_type != "machine" || config_from != "system") {
            r.code    = CLI_CONFIG_FILE_ERROR;
            r.message = "--downward-settings: " + path + " is not a system machine file; "
                                                       "give a system machine preset from the package's profiles";
            return r;
        }
        DownwardPrinter printer;
        printer.name = config_name;
        const Pointfs printable_area = config.option<ConfigOptionPoints>("printable_area", true)->values;
        const Pointfs exclude_area   = config.option<ConfigOptionPoints>("bed_exclude_area", true)->values;
        const Pointfs wrapping_area  = config.option<ConfigOptionPoints>("wrapping_exclude_area", true)->values;
        if (printable_area.size() >= 4) {
            printer.printable_width  = (int)(printable_area[2].x() - printable_area[0].x());
            printer.printable_depth  = (int)(printable_area[2].y() - printable_area[0].y());
            printer.printable_height = (int)(config.opt_float("printable_height"));
        }
        if (exclude_area.size() >= 4) {
            printer.exclude_width = (int)(exclude_area[2].x() - exclude_area[0].x());
            printer.exclude_depth = (int)(exclude_area[2].y() - exclude_area[0].y());
        }
        if (wrapping_area.size() >= 4) {
            printer.wrapping_width = (int)(wrapping_area[2].x() - wrapping_area[0].x());
            printer.wrapping_depth = (int)(wrapping_area[2].y() - wrapping_area[0].y());
        }
        shared_extruder_box(config, printer.printable_width, printer.printable_depth, printer.printable_height,
                            printer.shared_width, printer.shared_depth, printer.shared_height);
#ifndef ENGINE_ORCA
        // The clearances, for print-by-object plates (BambuStudio.cpp 4765-4775).
        if (config.option<ConfigOptionFloat>("extruder_clearance_height_to_lid"))
            printer.height_to_lid = config.opt_float("extruder_clearance_height_to_lid");
        if (config.option<ConfigOptionFloat>("extruder_clearance_height_to_rod"))
            printer.height_to_rod = config.opt_float("extruder_clearance_height_to_rod");
        if (config.option<ConfigOptionFloat>("extruder_clearance_max_radius"))
            printer.cleareance_radius = config.opt_float("extruder_clearance_max_radius");
        if (config.option<ConfigOptionFloat>("extruder_clearance_dist_to_rod"))
            printer.distance_to_rod = config.opt_float("extruder_clearance_dist_to_rod");
#endif
        printers.push_back(std::move(printer));
    }
    return r;
}

std::vector<std::string> downward_failures(const std::vector<DownwardPrinter>& printers, const Vec3d& size,
                                           bool is_sequence, const DynamicPrintConfig& config,
                                           const ProjectFacts& facts, bool has_support) {
    std::vector<std::string> failed;
    const auto* wrapping_opt = config.option<ConfigOptionBool>("enable_wrapping_detection");
    const bool enable_wrapping_detect = wrapping_opt != nullptr && wrapping_opt->value;
#ifndef ENGINE_ORCA
    // This printer's clearances and shared area (BambuStudio.cpp 4227-4280).
    auto opt_or_zero = [&](const char* key) { return config.has(key) ? config.opt_float(key) : 0.; };
    const double height_to_lid     = opt_or_zero("extruder_clearance_height_to_lid");
    const double height_to_rod     = opt_or_zero("extruder_clearance_height_to_rod");
    const double cleareance_radius = opt_or_zero("extruder_clearance_max_radius");
    const double distance_to_rod   = opt_or_zero("extruder_clearance_dist_to_rod");
    int current_width = 0, current_depth = 0;
    if (const auto* area = config.option<ConfigOptionPoints>("printable_area"); area && area->values.size() >= 4) {
        current_width = (int)(area->values[2].x() - area->values[0].x());
        current_depth = (int)(area->values[2].y() - area->values[0].y());
    }
    double shared_printable_width = 0., unused_depth = 0., unused_height = 0.;
    shared_extruder_box(config, current_width, current_depth, opt_or_zero("printable_height"),
                        shared_printable_width, unused_depth, unused_height);
    // The file's printable size, else this printer's (BambuStudio.cpp 4252-4255).
    const int old_printable_width = facts.old_printable_width > 0 ? facts.old_printable_width : current_width;
    const int old_printable_depth = facts.old_printable_depth > 0 ? facts.old_printable_depth : current_depth;
    constexpr int DOWNWARD_CHECK_MARGIN = 10;   // BambuStudio.hpp 20
#else
    (void)facts;
    (void)has_support;
    (void)is_sequence;
#endif
    for (const DownwardPrinter& plate_info : printers) {
#ifndef ENGINE_ORCA
        // BambuStudio.cpp 4819-4836.
        if (is_sequence) {
            if (plate_info.cleareance_radius > 0.f && plate_info.height_to_rod > 0.f &&
                plate_info.height_to_lid > 0.f && plate_info.distance_to_rod > 0.f) {
                if (cleareance_radius < plate_info.cleareance_radius || height_to_rod > plate_info.height_to_rod ||
                    height_to_lid > plate_info.height_to_lid || distance_to_rod < plate_info.distance_to_rod) {
                    failed.push_back(plate_info.name);
                    continue;
                }
            } else {
                failed.push_back(plate_info.name);
                continue;
            }
        }
#endif
        // BambuStudio.cpp 4830-4837; OrcaSlicer.cpp 4125-4134.
        if (size.z() > plate_info.printable_height || size.y() > plate_info.printable_depth ||
            size.x() > plate_info.printable_width) {
            failed.push_back(plate_info.name);
            continue;
        }
#ifndef ENGINE_ORCA
        // Single to multiple extruders (BambuStudio.cpp 4838-4847).
        if (shared_printable_width == 0 && plate_info.shared_width > 0 && plate_info.shared_depth > 0 &&
            plate_info.shared_height > 0 &&
            (size.z() > plate_info.shared_height || size.y() > plate_info.shared_depth || size.x() > plate_info.shared_width)) {
            failed.push_back(plate_info.name);
            continue;
        }
#endif
        if (plate_info.exclude_width > 0) {
            int real_width = plate_info.printable_width - plate_info.exclude_width;
            int real_depth = plate_info.printable_depth - plate_info.exclude_depth;
#ifndef ENGINE_ORCA
            // BambuStudio.cpp 4848-4862: either side over, with a margin for
            // a supported plate from a bigger bed without an excluded area.
            if (has_support && old_printable_width > real_width && old_printable_depth > real_depth &&
                facts.old_exclude_area_empty) {
                real_width -= DOWNWARD_CHECK_MARGIN;
                real_depth -= DOWNWARD_CHECK_MARGIN;
            }
            const bool over = size.x() > real_width || size.y() > real_depth;
#else
            // OrcaSlicer.cpp 4135-4145: both sides over.
            const bool over = size.x() > real_width && size.y() > real_depth;
#endif
            if (over) {
                failed.push_back(plate_info.name);
                continue;
            }
        }
        // BambuStudio.cpp 4863-4875; OrcaSlicer.cpp 4146-4157.
        if (enable_wrapping_detect && plate_info.wrapping_width > 0) {
            int real_depth = plate_info.printable_depth - plate_info.wrapping_depth;
            if (size.y() > real_depth) {
                failed.push_back(plate_info.name);
                continue;
            }
        }
    }
    return failed;
}

} // namespace slicer_cli

// libslic3r's FlushVolCalc.cpp calls RGB2HSV, which both apps define in
// their GUI library (slic3r/Utils/ColorSpaceConvert.cpp 113-140 in
// BambuStudio at 5873b5f and in OrcaSlicer at 31f6803, the same function in
// both; that file also needs wxWidgets). The function itself is plain
// arithmetic; this is that function, unchanged.
void RGB2HSV(float r, float g, float b, float* h, float* s, float* v)
{
    float Cmax = std::max(std::max(r, g), b);
    float Cmin = std::min(std::min(r, g), b);
    float delta = Cmax - Cmin;

    if (std::abs(delta) < 0.001) {
        *h = 0.f;
    }
    else if (Cmax == r) {
        *h = 60.f * fmod((g - b) / delta, 6.f);
    }
    else if (Cmax == g) {
        *h = 60.f * ((b - r) / delta + 2);
    }
    else {
        *h = 60.f * ((r - g) / delta + 4);
    }

    if (std::abs(Cmax) < 0.001) {
        *s = 0.f;
    }
    else {
        *s = delta / Cmax;
    }

    *v = Cmax;
}
