// Thin shim over libslic3r exposing three entry points to Rust:
//   slicer_run          — drive a full slice job from a JobSpec JSON blob
//   slicer_list_presets — list bundled preset names by category
//   slicer_get_preset   — fetch a named preset as JSON

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <unordered_set>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Format/STL.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/miniz_extension.hpp"

#include <nlohmann/json.hpp>
#include <boost/filesystem.hpp>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#include <climits>
#endif

#include "libslic3r/bambustudio/shim.hpp"

using json = nlohmann::json;

// ─── SliceArgs ────────────────────────────────────────────────────────────────

struct SliceArgs {
    std::string input_path;
    std::string output_path;

    // Config presets as inline JSON objects (null = not provided)
    nlohmann::json bundle_json;
    nlohmann::json machine_json;
    std::vector<nlohmann::json> filament_jsons;
    nlohmann::json process_json;

    // Optional CLI-style overrides, e.g. {"layer_height": "0.2"}
    std::map<std::string, std::string> overrides;

    bool verbose  = false;
    int  plate_id = 0;   // 0 = all plates, >0 = specific plate

    // Hint for finding resources/profiles when resolving 3MF presets.
    // Leave empty when calling from Rust (the runtime will use
    // _NSGetExecutablePath / /proc/self/exe instead).
    std::string exe_hint;
};

// ─── Structured event emission ───────────────────────────────────────────────

namespace {

constexpr const char* SLICER_EVENT_PREFIX = "[[SLICER_EVENT]] ";

const char* slicing_notification_tag(int t) {
    using NT = Slic3r::PrintStateBase::SlicingNotificationType;
    switch (static_cast<NT>(t)) {
        case NT::SlicingDefaultNotification:    return "SlicingDefaultNotification";
        case NT::SlicingReplaceInitEmptyLayers: return "SlicingReplaceInitEmptyLayers";
        case NT::SlicingNeedSupportOn:          return "SlicingNeedSupportOn";
        case NT::SlicingEmptyGcodeLayers:       return "SlicingEmptyGcodeLayers";
        case NT::SlicingGcodeOverlap:           return "SlicingGcodeOverlap";
    }
    return "SlicingUnknown";
}

const char* warning_level_tag(Slic3r::PrintStateBase::WarningLevel l) {
    return l == Slic3r::PrintStateBase::WarningLevel::CRITICAL ? "critical" : "non_critical";
}

const char* string_exception_tag(Slic3r::StringExceptionType t) {
    switch (t) {
        case Slic3r::STRING_EXCEPT_NOT_DEFINED:                     return "STRING_EXCEPT_NOT_DEFINED";
        case Slic3r::STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE:     return "STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE";
        case Slic3r::STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP:        return "STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP";
        case Slic3r::STRING_EXCEPT_OBJECT_COLLISION_IN_SEQ_PRINT:   return "STRING_EXCEPT_OBJECT_COLLISION_IN_SEQ_PRINT";
        case Slic3r::STRING_EXCEPT_OBJECT_COLLISION_IN_LAYER_PRINT: return "STRING_EXCEPT_OBJECT_COLLISION_IN_LAYER_PRINT";
        case Slic3r::STRING_EXCEPT_LAYER_HEIGHT_EXCEEDS_LIMIT:      return "STRING_EXCEPT_LAYER_HEIGHT_EXCEEDS_LIMIT";
        case Slic3r::STRING_EXCEPT_COUNT:                           return "STRING_EXCEPT_COUNT";
    }
    return "STRING_EXCEPT_UNKNOWN";
}

void emit_event(const json& payload) {
    std::cout << SLICER_EVENT_PREFIX << payload.dump() << '\n';
    std::cout.flush();
}

void emit_status_warning(const Slic3r::PrintBase::SlicingStatus& s) {
    using FB = Slic3r::PrintBase::SlicingStatus::FlagBits;
    const bool is_warning =
        (s.flags & (FB::UPDATE_PRINT_STEP_WARNINGS | FB::UPDATE_PRINT_OBJECT_STEP_WARNINGS)) != 0;
    if (!is_warning) return;
    json e;
    e["event"]   = "warning";
    e["tag"]     = slicing_notification_tag(static_cast<int>(s.message_type));
    e["level"]   = warning_level_tag(s.warning_level);
    e["message"] = s.text;
    e["step"]    = s.warning_step;
    e["scope"]   = (s.flags & FB::UPDATE_PRINT_OBJECT_STEP_WARNINGS) ? "object" : "print";
    emit_event(e);
}

void emit_validation_event(const Slic3r::StringObjectException& v) {
    json e;
    e["event"]   = v.is_warning ? "validation_warning" : "validation_error";
    e["tag"]     = string_exception_tag(v.type);
    e["message"] = v.string;
    if (!v.opt_key.empty())  e["opt_key"]   = v.opt_key;
    if (!v.params.empty())   e["params"]    = v.params;
    if (!v.hypetext.empty()) e["hypertext"] = v.hypetext;
    emit_event(e);
}

} // namespace

// ─── Config loading ───────────────────────────────────────────────────────────

static bool load_json_config_from_value(const json& j,
                                        Slic3r::DynamicPrintConfig& config,
                                        bool verbose = false) {
    try {
        Slic3r::ConfigSubstitutionContext substitution_context(
            Slic3r::ForwardCompatibilitySubstitutionRule::Enable);

        for (auto& [key, value] : j.items()) {
            if (key == "type" || key == "name" || key == "inherits" ||
                key == "from" || key == "setting_id" || key == "instantiation" ||
                key == "description" || key == "compatible_printers" ||
                key == "compatible_prints" || key == "include" ||
                key == "upward_compatible_machine" || key == "printer_model" ||
                key == "printer_variant" || key == "default_filament_profile" ||
                key == "default_print_profile") {
                continue;
            }

            try {
                std::string value_str;
                if (value.is_array()) {
                    std::vector<std::string> parts;
                    for (auto& v : value) {
                        if (v.is_string())       parts.push_back(v.get<std::string>());
                        else if (v.is_number())  parts.push_back(std::to_string(v.get<double>()));
                    }
                    for (size_t i = 0; i < parts.size(); i++) {
                        if (i > 0) value_str += ",";
                        value_str += parts[i];
                    }
                } else if (value.is_string()) {
                    value_str = value.get<std::string>();
                } else if (value.is_number_float()) {
                    value_str = std::to_string(value.get<double>());
                } else if (value.is_number_integer()) {
                    value_str = std::to_string(value.get<int>());
                } else if (value.is_boolean()) {
                    value_str = value.get<bool>() ? "1" : "0";
                }

                if (!value_str.empty() && value_str != "nil") {
                    config.set_deserialize(key, value_str, substitution_context);
                }
            } catch (const std::exception& e) {
                if (verbose) {
                    std::cerr << "Warning: Failed to set config key '" << key
                              << "': " << e.what() << "\n";
                }
            }
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing JSON config: " << e.what() << "\n";
        return false;
    }
}

bool load_json_config(const std::string& filepath,
                      Slic3r::DynamicPrintConfig& config,
                      bool verbose) {
    if (verbose) std::cout << "Loading config: " << filepath << "\n";
    std::ifstream f(filepath);
    if (!f.is_open()) {
        std::cerr << "Error: Cannot open config file: " << filepath << "\n";
        return false;
    }
    try {
        json j = json::parse(f);
        if (verbose) std::cout << "  Loaded successfully\n";
        return load_json_config_from_value(j, config, verbose);
    } catch (const std::exception& e) {
        std::cerr << "Error parsing JSON config: " << e.what() << "\n";
        return false;
    }
}

// ─── Nozzle mapping ───────────────────────────────────────────────────────────

static bool apply_explicit_nozzle_mapping(Slic3r::DynamicPrintConfig& config) {
    {
        auto* mode_opt = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>(
            "filament_map_mode", false);
        if (mode_opt && mode_opt->value == Slic3r::FilamentMapMode::fmmNozzleManual)
            return false;
    }

    auto* filament_map          = config.option<Slic3r::ConfigOptionInts>("filament_map", false);
    auto* filament_nozzle_map   = config.option<Slic3r::ConfigOptionInts>("filament_nozzle_map", false);
    auto* physical_extruder_map = config.option<Slic3r::ConfigOptionInts>("physical_extruder_map", false);
    auto* nozzle_diameter       = config.option<Slic3r::ConfigOptionFloatsNullable>("nozzle_diameter", false);
    if (!filament_map || !filament_nozzle_map || !physical_extruder_map || !nozzle_diameter)
        return false;

    const size_t filament_count = filament_map->values.size();
    const size_t extruder_count = nozzle_diameter->values.size();
    if (filament_count < 2 || extruder_count < 2)
        return false;
    if (filament_nozzle_map->values.size() < filament_count ||
        physical_extruder_map->values.size() < extruder_count)
        return false;

    std::map<int, int> physical_to_logical;
    for (size_t logical_idx = 0; logical_idx < extruder_count; ++logical_idx) {
        physical_to_logical[physical_extruder_map->values[logical_idx]] =
            static_cast<int>(logical_idx) + 1;
    }
    if (physical_to_logical.size() < extruder_count) return false;

    std::vector<int> derived_map = filament_map->values;
    for (size_t filament_idx = 0; filament_idx < filament_count; ++filament_idx) {
        auto it = physical_to_logical.find(filament_nozzle_map->values[filament_idx]);
        if (it == physical_to_logical.end()) return false;
        derived_map[filament_idx] = it->second;
    }

    const bool uses_multiple_logical_extruders =
        std::adjacent_find(derived_map.begin(), derived_map.end(),
                           std::not_equal_to<int>()) != derived_map.end();
    if (!uses_multiple_logical_extruders) return false;

    filament_map->values = derived_map;

    auto* filament_map_2 = config.option<Slic3r::ConfigOptionInts>("filament_map_2", true);
    filament_map_2->values.resize(derived_map.size());
    for (size_t i = 0; i < derived_map.size(); ++i)
        filament_map_2->values[i] = derived_map[i] - 1;

    {
        Slic3r::ConfigSubstitutionContext substitution_context(
            Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
        config.set_deserialize("filament_map_mode", "Nozzle Manual", substitution_context);
    }
    return true;
}

static void reassign_objects_to_master_nozzle(Slic3r::Model& model,
                                               const Slic3r::DynamicPrintConfig& config) {
    const auto* filament_map          = config.option<Slic3r::ConfigOptionInts>("filament_map");
    const auto* physical_extruder_map = config.option<Slic3r::ConfigOptionInts>("physical_extruder_map");
    if (!filament_map || !physical_extruder_map) return;

    const size_t extruder_count = physical_extruder_map->values.size();
    if (extruder_count < 2) return;

    int master_logical_idx = -1;
    for (size_t i = 0; i < extruder_count; ++i) {
        if (physical_extruder_map->values[i] == 0) {
            master_logical_idx = static_cast<int>(i);
            break;
        }
    }
    if (master_logical_idx < 0) return;

    int master_extruder_1based = master_logical_idx + 1;
    int master_filament_slot = -1;
    for (size_t i = 0; i < filament_map->values.size(); ++i) {
        if (filament_map->values[i] == master_extruder_1based) {
            master_filament_slot = static_cast<int>(i) + 1;
            break;
        }
    }
    if (master_filament_slot < 0) return;

    for (auto* obj : model.objects) {
        int cur = obj->config.extruder();
        if (cur != master_filament_slot)
            obj->config.set_key_value("extruder",
                new Slic3r::ConfigOptionInt(master_filament_slot));
        for (auto* vol : obj->volumes) {
            const Slic3r::ConfigOption* vopt = vol->config.option("extruder");
            if (vopt && vopt->getInt() != 0 && vopt->getInt() != master_filament_slot)
                vol->config.set_key_value("extruder",
                    new Slic3r::ConfigOptionInt(master_filament_slot));
        }
    }
}

// ─── Default config ───────────────────────────────────────────────────────────

static void set_default_config(Slic3r::DynamicPrintConfig& config) {
    config.apply(Slic3r::FullPrintConfig::defaults(), true);

    // BambuStudio defaults to a multi-nozzle config (nozzle_diameter has 2+ entries).
    // Force single-extruder as the baseline so extruder_count=1 until a machine
    // profile overrides nozzle_diameter with the actual hardware count.
    // nozzle_diameter is declared as ConfigOptionFloatsNullable in FullPrintConfig;
    // using ConfigOptionFloats breaks the dynamic_cast in support_different_extruders().
    config.set_key_value("nozzle_diameter",    new Slic3r::ConfigOptionFloatsNullable({0.4}));
    config.set_key_value("extruder_type",      new Slic3r::ConfigOptionEnumsGeneric({0})); // etDirectDrive
    config.set_key_value("nozzle_volume_type", new Slic3r::ConfigOptionEnumsGeneric({0})); // nvtStandard

    config.set_key_value("print_settings_id",    new Slic3r::ConfigOptionString(""));
    config.set_key_value("filament_settings_id", new Slic3r::ConfigOptionStrings({""}));
    config.set_key_value("printer_settings_id",  new Slic3r::ConfigOptionString(""));

    size_t num_filaments = 1;
    config.option<Slic3r::ConfigOptionInts>("filament_map", true)->values =
        std::vector<int>(num_filaments, 1);
    config.option<Slic3r::ConfigOptionInts>("filament_volume_map", true)->values =
        std::vector<int>(num_filaments, 0);

    auto filament_variant = config.option<Slic3r::ConfigOptionStrings>("filament_extruder_variant", true);
    if (filament_variant) {
        int index_size = filament_variant->values.size();
        if (index_size == 0) {
            index_size = 1;
            filament_variant->values.resize(1, "Direct Drive Standard");
        }
        config.option<Slic3r::ConfigOptionInts>("filament_self_index", true)->values
            .resize(index_size, 1);
    }

    auto support_fil = config.option<Slic3r::ConfigOptionInt>("support_filament");
    if (support_fil)
        support_fil->value = std::max(0, std::min(support_fil->value, (int)num_filaments));
    auto support_iface = config.option<Slic3r::ConfigOptionInt>("support_interface_filament");
    if (support_iface)
        support_iface->value = std::max(0, std::min(support_iface->value, (int)num_filaments));

    auto filament_map_opt = config.option<Slic3r::ConfigOptionInts>("filament_map", true);
    if (filament_map_opt->values.empty() || filament_map_opt->values[0] != 1)
        filament_map_opt->values = {1};

    auto filament_volume_map_opt = config.option<Slic3r::ConfigOptionInts>("filament_volume_map", true);
    if (filament_volume_map_opt->values.empty() || filament_volume_map_opt->values[0] != 0)
        filament_volume_map_opt->values = {0};

    auto filament_nozzle_map_opt = config.option<Slic3r::ConfigOptionInts>("filament_nozzle_map", true);
    if (filament_nozzle_map_opt->values.empty())
        filament_nozzle_map_opt->values = {1};

    auto filament_map_2_opt = config.option<Slic3r::ConfigOptionInts>("filament_map_2", true);
    if (filament_map_2_opt->values.empty())
        filament_map_2_opt->values = {0};

    auto print_extruder_variant_opt =
        config.option<Slic3r::ConfigOptionStrings>("print_extruder_variant", true);
    if (!print_extruder_variant_opt || print_extruder_variant_opt->values.empty())
        config.set_key_value("print_extruder_variant",
            new Slic3r::ConfigOptionStrings({"Direct Drive Standard"}));

    auto print_extruder_id_opt = config.option<Slic3r::ConfigOptionInts>("print_extruder_id", true);
    if (!print_extruder_id_opt || print_extruder_id_opt->values.empty())
        config.set_key_value("print_extruder_id", new Slic3r::ConfigOptionInts({1}));

    auto init_float   = [&](const std::string& k, double v) {
        auto o = config.option<Slic3r::ConfigOptionFloats>(k, true);
        if (!o || o->values.empty())
            config.set_key_value(k, new Slic3r::ConfigOptionFloats({v}));
    };
    auto init_int     = [&](const std::string& k, int v) {
        auto o = config.option<Slic3r::ConfigOptionInts>(k, true);
        if (!o || o->values.empty())
            config.set_key_value(k, new Slic3r::ConfigOptionInts({v}));
    };
    auto init_percent = [&](const std::string& k, double v) {
        auto o = config.option<Slic3r::ConfigOptionPercents>(k, true);
        if (!o || o->values.empty())
            config.set_key_value(k, new Slic3r::ConfigOptionPercents({v}));
    };
    auto init_bool    = [&](const std::string& k, bool v) {
        auto opt_n = config.option<Slic3r::ConfigOptionBoolsNullable>(k, false);
        if (opt_n) { if (opt_n->values.empty()) opt_n->values.push_back((unsigned char)v); return; }
        auto opt   = config.option<Slic3r::ConfigOptionBools>(k, false);
        if (opt)   { if (opt->values.empty())   opt->values.push_back((unsigned char)v);   return; }
        const auto* def = Slic3r::print_config_def.get(k);
        if (def && def->nullable)
            config.set_key_value(k, new Slic3r::ConfigOptionBoolsNullable({(unsigned char)v}));
        else
            config.set_key_value(k, new Slic3r::ConfigOptionBools({v}));
    };

    init_float("filament_flow_ratio",                     1.0);
    init_float("filament_max_volumetric_speed",           0.0);
    init_float("filament_ramming_volumetric_speed",       0.0);
    init_int  ("filament_pre_cooling_temperature",        0);
    init_float("filament_ramming_travel_time",            0.0);
    init_float("filament_ramming_volumetric_speed_nc",    0.0);
    init_int  ("filament_pre_cooling_temperature_nc",     0);
    init_float("filament_ramming_travel_time_nc",         0.0);
    init_float("filament_retraction_length",              0.8);
    init_float("filament_retract_length_nc",              0.0);
    init_float("filament_z_hop",                          0.0);
    init_float("filament_retract_restart_extra",          0.0);
    init_float("filament_retraction_speed",               20.0);
    init_float("filament_deretraction_speed",             20.0);
    init_float("filament_retraction_minimum_travel",      0.0);
    init_bool ("filament_retract_when_changing_layer",    false);
    init_bool ("filament_wipe",                           false);
    init_float("filament_wipe_distance",                  0.0);
    init_percent("filament_retract_before_wipe",          0.0);
    init_bool ("filament_long_retractions_when_cut",      false);
    init_float("filament_retraction_distances_when_cut",  0.0);
    init_bool ("long_retractions_when_ec",                false);
    init_float("retraction_distances_when_ec",            0.0);
    init_float("filament_flush_volumetric_speed",         0.0);
    init_bool ("override_process_overhang_speed",         false);
    init_bool ("filament_enable_overhang_speed",          false);
    init_bool ("filament_adaptive_volumetric_speed",      false);
    init_float("filament_bridge_speed",                   0.0);
    init_float("filament_overhang_1_4_speed",             0.0);
    init_float("filament_overhang_2_4_speed",             0.0);
    init_float("filament_overhang_3_4_speed",             0.0);
    init_float("filament_overhang_4_4_speed",             0.0);
    init_float("filament_overhang_totally_speed",         0.0);

    config.option<Slic3r::ConfigOptionEnumGeneric>("printer_technology", true)->value =
        Slic3r::ptFFF;

    if (!config.has("machine_start_gcode") || config.opt_string("machine_start_gcode").empty())
        config.set_key_value("machine_start_gcode", new Slic3r::ConfigOptionString(
            "; Minimal start G-code\nG28 ; home all axes\nG1 Z5 F5000 ; lift nozzle\n"));
    if (!config.has("machine_end_gcode") || config.opt_string("machine_end_gcode").empty())
        config.set_key_value("machine_end_gcode", new Slic3r::ConfigOptionString(
            "; Minimal end G-code\nG1 E-1 F300 ; retract\nG28 X0 Y0 ; home X Y\nM84 ; disable motors\n"));

    auto ensure_temp = [&](const std::string& k, int v) {
        if (!config.has(k)) config.set_key_value(k, new Slic3r::ConfigOptionInts({v}));
    };
    ensure_temp("nozzle_temperature",              200);
    ensure_temp("nozzle_temperature_initial_layer",200);
    ensure_temp("bed_temperature",                 60);
    ensure_temp("bed_temperature_initial_layer",   60);
}

// ─── Vector size enforcement ──────────────────────────────────────────────────

static void ensure_vector_config_sizes(Slic3r::DynamicPrintConfig& config) {
    auto ensure = [&](const std::string& key, size_t min_size,
                      const std::string& default_val = "") {
        auto* opt = config.option(key, true);
        if (!opt) return;
#define PAD(Type, zero_val) \
        if (auto v = dynamic_cast<Slic3r::Type*>(opt)) { \
            while (v->values.size() < min_size) \
                v->values.push_back(v->values.empty() ? (zero_val) : v->values[0]); \
        } else
        PAD(ConfigOptionBoolsNullable,          (unsigned char)0)
        PAD(ConfigOptionBools,                  (unsigned char)0)
        PAD(ConfigOptionIntsNullable,           0)
        PAD(ConfigOptionInts,                   0)
        PAD(ConfigOptionFloatsNullable,         0.0)
        PAD(ConfigOptionFloats,                 0.0)
        PAD(ConfigOptionPercentsNullable,       0.0)
        PAD(ConfigOptionPercents,               0.0)
        PAD(ConfigOptionFloatsOrPercentsNullable,(Slic3r::FloatOrPercent{0.0,false}))
        PAD(ConfigOptionFloatsOrPercents,       (Slic3r::FloatOrPercent{0.0,false}))
        PAD(ConfigOptionStrings,                default_val)
        PAD(ConfigOptionEnumsGenericNullable,   0)
        PAD(ConfigOptionEnumsGeneric,           0)
        { /* points, groups — leave alone */ }
#undef PAD
    };

    ensure("nozzle_diameter",     1);
    ensure("filament_diameter",   1);
    ensure("filament_type",       1, "PLA");
    ensure("filament_colour",     1, "#FFFFFF");

    if (!config.has("extruder_type") ||
        config.option<Slic3r::ConfigOptionEnumsGeneric>("extruder_type")->values.empty())
        config.set_key_value("extruder_type",   new Slic3r::ConfigOptionEnumsGeneric({0}));
    if (!config.has("nozzle_volume_type") ||
        config.option<Slic3r::ConfigOptionEnumsGeneric>("nozzle_volume_type")->values.empty())
        config.set_key_value("nozzle_volume_type", new Slic3r::ConfigOptionEnumsGeneric({0}));

    {
        size_t n = 1;
        if (auto* nd = config.option<Slic3r::ConfigOptionFloatsNullable>("nozzle_diameter"))
            n = std::max(n, nd->values.size());
        ensure("extruder_type",             n);
        ensure("nozzle_volume_type",        n);
        ensure("extruder_max_nozzle_count", n);
    }

    ensure("filament_map",                 1);
    ensure("filament_volume_map",          1);
    ensure("filament_nozzle_map",          1);
    {
        auto* o = config.option<Slic3r::ConfigOptionInts>("filament_map_2", true);
        if (o && o->values.empty()) o->values.push_back(0);
    }
    ensure("print_extruder_variant",       1, "Direct Drive Standard");
    ensure("print_extruder_id",            1);
    ensure("filament_extruder_variant",    1, "Direct Drive Standard");
    ensure("filament_self_index",          1);
    ensure("filament_max_volumetric_speed",1);
    ensure("filament_flow_ratio",          1);
    ensure("printer_extruder_variant",     1, "0.4");
    ensure("retract_length",               1);
    ensure("retract_lift",                 1);
    ensure("temperature",                  1);
    ensure("nozzle_temperature",           1);
    ensure("override_process_overhang_speed",   1);
    ensure("filament_enable_overhang_speed",    1);
    ensure("filament_adaptive_volumetric_speed",1);
    ensure("filament_bridge_speed",             1);
    ensure("filament_overhang_1_4_speed",       1);
    ensure("filament_overhang_2_4_speed",       1);
    ensure("filament_overhang_3_4_speed",       1);
    ensure("filament_overhang_4_4_speed",       1);
    ensure("filament_overhang_totally_speed",   1);
    ensure("enable_overhang_speed",             1);
    ensure("enable_height_slowdown",            1);
    ensure("long_retractions_when_cut",         1);
    ensure("long_retractions_when_ec",          1);
    ensure("retract_before_wipe",               1);
    ensure("retraction_length",                 1);
    ensure("retraction_speed",                  1);
    ensure("deretraction_speed",                1);
    ensure("z_hop",                             1);
    ensure("travel_speed",                      1);
    ensure("travel_speed_z",                    1);
    ensure("outer_wall_speed",                  1);
    ensure("inner_wall_speed",                  1);
    ensure("sparse_infill_speed",               1);
    ensure("internal_solid_infill_speed",       1);
    ensure("top_surface_speed",                 1);
    ensure("gap_infill_speed",                  1);
    ensure("support_speed",                     1);
    ensure("support_interface_speed",           1);
    ensure("bridge_speed",                      1);
    ensure("overhang_totally_speed",            1);
    ensure("overhang_1_4_speed",                1);
    ensure("overhang_2_4_speed",                1);
    ensure("overhang_3_4_speed",                1);
    ensure("overhang_4_4_speed",                1);
    ensure("filament_ramming_volumetric_speed", 1);
    ensure("filament_flush_volumetric_speed",   1);

    {
        auto* fp = config.option<Slic3r::ConfigOptionInts>("filament_printable", true);
        if (!fp || fp->values.empty())
            config.set_key_value("filament_printable",
                new Slic3r::ConfigOptionInts({std::numeric_limits<int>::max()}));
        else
            for (auto& v : fp->values) v = std::numeric_limits<int>::max();
    }
}

// ─── PresetBundle loader ──────────────────────────────────────────────────────
// Reads printer_settings_id / print_settings_id / filament_settings_id from
// `config`, resolves them via PresetBundle, and replaces `config` with the
// fully-merged full_config().  Returns true on success.
static bool try_load_presets_via_bundle(Slic3r::DynamicPrintConfig& config,
                                        const std::string& exe_hint,
                                        bool verbose) {
    boost::filesystem::path exe_dir;
#ifdef __APPLE__
    {
        char pathbuf[PATH_MAX];
        uint32_t size = sizeof(pathbuf);
        if (_NSGetExecutablePath(pathbuf, &size) == 0) {
            try { exe_dir = boost::filesystem::canonical(pathbuf).parent_path(); } catch (...) {}
        }
    }
#else
    try { exe_dir = boost::filesystem::canonical("/proc/self/exe").parent_path(); } catch (...) {}
#endif
    if (exe_dir.empty() && !exe_hint.empty()) {
        try { exe_dir = boost::filesystem::canonical(exe_hint).parent_path(); } catch (...) {}
    }

    boost::filesystem::path profiles_dir;
    for (const auto& p : std::vector<boost::filesystem::path>{
        exe_dir / ".." / ".." / "references" / "BambuStudio" / "resources" / "profiles",
        exe_dir / ".." / "resources" / "profiles",
        boost::filesystem::path("/home/user/slicer/references/BambuStudio/resources/profiles"),
    }) {
        if (boost::filesystem::exists(p) && boost::filesystem::is_directory(p)) {
            profiles_dir = boost::filesystem::canonical(p);
            break;
        }
    }

    if (profiles_dir.empty()) return false;

    try {
        auto tmpdir = boost::filesystem::temp_directory_path() / "slicer_cli_presets";
        auto sysdir = tmpdir / "system";
        boost::filesystem::remove_all(tmpdir);
        boost::filesystem::create_directories(sysdir);
        for (auto& entry : boost::filesystem::directory_iterator(profiles_dir)) {
            auto dst = sysdir / entry.path().filename();
            if (boost::filesystem::is_directory(entry.path()))
                boost::filesystem::create_directory_symlink(
                    boost::filesystem::canonical(entry.path()), dst);
            else if (entry.path().extension() == ".json")
                boost::filesystem::copy_file(entry.path(), dst);
        }
        Slic3r::set_data_dir(tmpdir.string());
        Slic3r::set_resources_dir((profiles_dir / "..").string());

        Slic3r::PresetBundle preset_bundle;
        bool first_vendor = true;
        for (auto& dir_entry : boost::filesystem::directory_iterator(sysdir)) {
            if (dir_entry.path().extension() != ".json") continue;
            std::string vname = dir_entry.path().stem().string();
            if (vname == "blacklist") continue;
            try {
                if (first_vendor) {
                    preset_bundle.load_vendor_configs_from_json(
                        sysdir.string(), vname,
                        Slic3r::PresetBundle::LoadSystem,
                        Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
                    first_vendor = false;
                } else {
                    preset_bundle.load_vendor_configs_from_json(
                        sysdir.string(), vname,
                        Slic3r::PresetBundle::LoadConfigBundleAttributes(),
                        Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
                }
            } catch (...) {}
        }

        std::string printer_name  = config.opt_string("printer_settings_id");
        std::string print_name    = config.opt_string("print_settings_id");
        std::string filament_name;
        if (auto* fsi = config.option<Slic3r::ConfigOptionStrings>("filament_settings_id"))
            if (!fsi->values.empty()) filament_name = fsi->values[0];

        if (verbose)
            std::cout << "  Preset lookup: printer='" << printer_name
                      << "' print='" << print_name
                      << "' filament='" << filament_name << "'\n";

        if (!printer_name.empty() && !print_name.empty() && !filament_name.empty()) {
            bool ok_printer  = preset_bundle.printers.select_preset_by_name(printer_name, true);
            bool ok_print    = preset_bundle.prints.select_preset_by_name(print_name, true);
            bool ok_filament = preset_bundle.filaments.select_preset_by_name(filament_name, true);

            if (ok_printer && ok_print && ok_filament
                && preset_bundle.printers.get_edited_preset().name  == printer_name
                && preset_bundle.prints.get_edited_preset().name    == print_name
                && preset_bundle.filaments.get_edited_preset().name == filament_name)
            {
                Slic3r::DynamicPrintConfig base_config = preset_bundle.full_config();
                base_config.apply(config, true);
                config = std::move(base_config);
                if (verbose)
                    std::cout << "  Presets loaded: " << printer_name
                              << " / " << print_name
                              << " / " << filament_name << "\n";
                return true;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "  PresetBundle exception: " << e.what() << "\n";
    }
    return false;
}

// ─── Main implementation ──────────────────────────────────────────────────────

static int run_slice_job(const SliceArgs& args) {
    Slic3r::set_logging_level(args.verbose ? 5 : 3);

    const bool verbose  = args.verbose;
    const int  plate_id = args.plate_id;

    std::cout << "libslic3r_standalone - Standalone slicing tool\n";
    std::cout << "Based on BambuStudio libslic3r\n\n";

    try {
        std::cout << "\nConfiguring print settings...\n";
        Slic3r::DynamicPrintConfig config;
        set_default_config(config);

        std::cout << "Loading model: " << args.input_path << "\n";
        Slic3r::Model model;
        model.set_backup_path(
            boost::filesystem::temp_directory_path().string() + "/slicer_cli_backup");
        bool is_bbl_3mf = false;
        bool bundle_preset_loaded = false;
        Slic3r::PlateDataPtrs plate_data;

        const std::string& input_file = args.input_path;

        if (input_file.find(".stl") != std::string::npos ||
            input_file.find(".STL") != std::string::npos) {
            if (!Slic3r::load_stl(input_file.c_str(), &model)) {
                std::cerr << "Failed to load STL file\n";
                return 1;
            }
            // Inject preset names from the resolved JSON so PresetBundle can
            // look them up and produce a correct full_config().
            auto extract_name = [](const json& j) -> std::string {
                auto it = j.find("name");
                if (it != j.end() && it->is_string()) return it->get<std::string>();
                return {};
            };
            if (!args.machine_json.is_null() && args.machine_json.is_object()) {
                std::string n = extract_name(args.machine_json);
                if (!n.empty())
                    config.set_key_value("printer_settings_id",
                        new Slic3r::ConfigOptionString(n));
            }
            if (!args.process_json.is_null() && args.process_json.is_object()) {
                std::string n = extract_name(args.process_json);
                if (!n.empty())
                    config.set_key_value("print_settings_id",
                        new Slic3r::ConfigOptionString(n));
            }
            if (!args.filament_jsons.empty() && args.filament_jsons[0].is_object()) {
                std::string n = extract_name(args.filament_jsons[0]);
                if (!n.empty())
                    config.option<Slic3r::ConfigOptionStrings>("filament_settings_id", true)
                        ->values = {n};
            }
            bool preset_loaded = try_load_presets_via_bundle(config, args.exe_hint, verbose);
            if (!preset_loaded)
                std::cout << "  WARNING: PresetBundle lookup failed — using flat JSON config\n";
        } else if (input_file.find(".3mf") != std::string::npos ||
                   input_file.find(".3MF") != std::string::npos) {
            Slic3r::ConfigSubstitutionContext config_subst(
                Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
            std::vector<Slic3r::Preset*> presets;
            Slic3r::Semver file_version;

            auto strategy = Slic3r::LoadStrategy::LoadModel
                          | Slic3r::LoadStrategy::LoadConfig
                          | Slic3r::LoadStrategy::AddDefaultInstances;
            if (!Slic3r::load_bbs_3mf(input_file.c_str(), &config, &config_subst,
                                       &model, &plate_data, &presets, &is_bbl_3mf,
                                       &file_version, nullptr, strategy, nullptr, plate_id)) {
                std::cerr << "Failed to load 3MF file\n";
                return 1;
            }

            if (plate_id > 0 && (int)plate_data.size() < plate_id) {
                std::cerr << "Error: --plate " << plate_id
                          << " but 3MF only has " << plate_data.size() << " plate(s)\n";
                return 1;
            }

            // Multi-plate coordinate translation
            if (plate_id > 0 && is_bbl_3mf && !model.objects.empty()) {
                Slic3r::BoundingBoxf3 actual_bbox;
                for (auto* obj : model.objects)
                    for (size_t i = 0; i < obj->instances.size(); i++)
                        actual_bbox.merge(obj->instance_bounding_box(i));

                double expected_min_x = 0.0, expected_min_y = 0.0;
                bool is_seq_print_plate = false;

                mz_zip_archive zip;
                mz_zip_zero_struct(&zip);
                if (Slic3r::open_zip_reader(&zip, input_file)) {
                    std::string plate_json_path = "Metadata/plate_" + std::to_string(plate_id) + ".json";
                    int file_idx = mz_zip_reader_locate_file(&zip, plate_json_path.c_str(), nullptr, 0);
                    if (file_idx >= 0) {
                        mz_zip_archive_file_stat stat;
                        if (mz_zip_reader_file_stat(&zip, file_idx, &stat)) {
                            std::string content(stat.m_uncomp_size, '\0');
                            mz_zip_reader_extract_to_mem(&zip, file_idx, content.data(), content.size(), 0);
                            try {
                                auto plate_json = json::parse(content);
                                if (plate_json.contains("bbox_all") && plate_json["bbox_all"].size() >= 4) {
                                    expected_min_x = plate_json["bbox_all"][0].get<double>();
                                    expected_min_y = plate_json["bbox_all"][1].get<double>();
                                }
                                if (plate_json.contains("is_seq_print"))
                                    is_seq_print_plate = plate_json["is_seq_print"].get<bool>();
                            } catch (...) {}
                        }
                    }
                    mz_zip_reader_end(&zip);
                }

                double offset_x = expected_min_x - actual_bbox.min.x();
                double offset_y = expected_min_y - actual_bbox.min.y();
                if (std::abs(offset_x) > 1.0 || std::abs(offset_y) > 1.0) {
                    if (verbose)
                        std::cout << "Plate " << plate_id << " coord translation: ("
                                  << offset_x << ", " << offset_y << ")\n";
                    for (auto* obj : model.objects)
                        for (auto* inst : obj->instances) {
                            Slic3r::Vec3d off = inst->get_offset();
                            inst->set_offset(Slic3r::Vec3d(off.x()+offset_x, off.y()+offset_y, off.z()));
                        }
                }
                if (is_seq_print_plate) {
                    Slic3r::ConfigSubstitutionContext seq_subst(
                        Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
                    config.set_deserialize("print_sequence", "by object", seq_subst);
                    if (verbose) std::cout << "Plate " << plate_id << " uses sequential printing\n";
                }
            }

            // Rebuild config via PresetBundle for 3MF inputs
            {
                bool preset_loaded = try_load_presets_via_bundle(config, args.exe_hint, verbose);
                if (!preset_loaded)
                    std::cout << "  WARNING: Using flat 3MF config (presets not resolved)\n";
            }
        } else {
            std::cerr << "Unsupported file format. Use .stl or .3mf\n";
            return 1;
        }

        if (model.objects.empty()) {
            std::cerr << "No objects found in model\n";
            return 1;
        }

        for (auto* obj : model.objects) {
            if (obj->instances.empty()) obj->add_instance();
            for (auto* inst : obj->instances)
                inst->use_loaded_id_for_label = true;
        }

        std::cout << "Model loaded successfully:\n";
        for (const auto* obj : model.objects)
            std::cout << "  - " << obj->name << " (" << obj->volumes.size()
                      << " volumes, " << obj->instances.size() << " instances)\n";

        // Apply inline config presets (args take precedence over 3MF embedded config)
        if (!args.bundle_json.is_null() && args.bundle_json.is_object())
            load_json_config_from_value(args.bundle_json,  config, verbose);
        if (!args.machine_json.is_null() && args.machine_json.is_object())
            load_json_config_from_value(args.machine_json, config, verbose);
        if (!args.process_json.is_null() && args.process_json.is_object())
            load_json_config_from_value(args.process_json, config, verbose);
        if (!args.filament_jsons.empty() && !args.filament_jsons[0].is_null())
            load_json_config_from_value(args.filament_jsons[0], config, verbose);

        // Vector padding
        {
            size_t extruder_count = 1;
            if (auto* nd = config.option<Slic3r::ConfigOptionFloatsNullable>("nozzle_diameter", false))
                if (!nd->values.empty()) extruder_count = nd->values.size();

            static const std::unordered_set<std::string> skip_pad = {
                "printable_area", "bed_exclude_area", "thumbnails", "extruder_printable_area"
            };

            for (const auto& key : config.keys()) {
                if (skip_pad.count(key)) continue;
                auto* opt = config.option(key, false);
                if (!opt) continue;
#define NORM_VEC(Type, zero_val) \
                if (auto v = dynamic_cast<Slic3r::Type*>(opt)) { \
                    while (v->values.size() < extruder_count) \
                        v->values.push_back(zero_val); \
                } else
                NORM_VEC(ConfigOptionBoolsNullable,             (unsigned char)0)
                NORM_VEC(ConfigOptionBools,                     (unsigned char)0)
                NORM_VEC(ConfigOptionIntsNullable,              0)
                NORM_VEC(ConfigOptionInts,                      0)
                NORM_VEC(ConfigOptionFloatsNullable,            0.0)
                NORM_VEC(ConfigOptionFloats,                    0.0)
                NORM_VEC(ConfigOptionPercentsNullable,          0.0)
                NORM_VEC(ConfigOptionPercents,                  0.0)
                NORM_VEC(ConfigOptionFloatsOrPercentsNullable,  (Slic3r::FloatOrPercent{0.0,false}))
                NORM_VEC(ConfigOptionFloatsOrPercents,          (Slic3r::FloatOrPercent{0.0,false}))
                NORM_VEC(ConfigOptionStrings,                   std::string{})
                NORM_VEC(ConfigOptionEnumsGenericNullable,      0)
                NORM_VEC(ConfigOptionEnumsGeneric,              0)
                { /* points / groups — leave alone */ }
#undef NORM_VEC
            }

            if (auto* mid = config.option<Slic3r::ConfigOptionInt>("master_extruder_id", false)) {
                if (mid->value < 1) mid->value = 1;
                if (mid->value > (int)extruder_count) mid->value = (int)extruder_count;
            }
        }

        ensure_vector_config_sizes(config);

        // Plate filament mapping
        int plate_data_idx = (plate_id > 0 && (int)plate_data.size() >= plate_id) ? plate_id-1 : 0;
        if (!plate_data.empty() && plate_data[plate_data_idx] != nullptr) {
            const auto& pm = plate_data[plate_data_idx]->filament_maps;
            bool has_diverse = pm.size() >= 2 &&
                std::adjacent_find(pm.begin(), pm.end(), std::not_equal_to<int>()) != pm.end();
            bool is_manual = false;
            {
                auto* mode_opt = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>(
                    "filament_map_mode", false);
                if (mode_opt && mode_opt->value == Slic3r::FilamentMapMode::fmmNozzleManual)
                    is_manual = true;
            }
            if (pm.size() >= 2 && (has_diverse || is_manual)) {
                auto* fm  = config.option<Slic3r::ConfigOptionInts>("filament_map",   true);
                auto* fm2 = config.option<Slic3r::ConfigOptionInts>("filament_map_2", true);
                fm->values = pm;
                fm2->values.resize(pm.size());
                for (size_t i = 0; i < pm.size(); ++i) fm2->values[i] = pm[i] - 1;
                Slic3r::ConfigSubstitutionContext subst(
                    Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
                config.set_deserialize("filament_map_mode", "Nozzle Manual", subst);
            }
        }

        bool nozzle_mapping_derived = apply_explicit_nozzle_mapping(config);
        if (nozzle_mapping_derived)
            reassign_objects_to_master_nozzle(model, config);

        // Disable prime tower when no multi-material printing
        {
            auto* ept = config.option<Slic3r::ConfigOptionBool>("enable_prime_tower", false);
            if (ept && ept->value) {
                bool disable = false;
                auto* fd = config.option<Slic3r::ConfigOptionFloats>("filament_diameter", false);
                if (fd && fd->values.size() <= 1) disable = true;
                if (!disable) {
                    auto* fc = config.option<Slic3r::ConfigOptionStrings>("filament_colour", false);
                    auto* fm = config.option<Slic3r::ConfigOptionInts>("filament_map", false);
                    bool all_same_color = fc && !fc->values.empty() &&
                        std::all_of(fc->values.begin(), fc->values.end(),
                                    [&](const std::string& c){ return c == fc->values[0]; });
                    bool all_same_extruder = fm && !fm->values.empty() &&
                        std::all_of(fm->values.begin(), fm->values.end(),
                                    [&](int v){ return v == fm->values[0]; });
                    if (all_same_color && all_same_extruder) disable = true;
                }
                if (disable) ept->value = false;
            }
        }

        // Command-line overrides
        for (const auto& [key, value] : args.overrides) {
            try {
                if (key == "layer_height") {
                    config.set_key_value(key, new Slic3r::ConfigOptionFloat(std::stof(value)));
                } else if (key == "nozzle_diameter") {
                    config.set_key_value(key, new Slic3r::ConfigOptionFloatsNullable({std::stod(value)}));
                } else if (key == "fill_density") {
                    config.set_key_value(key, new Slic3r::ConfigOptionPercent(std::stoi(value)));
                    config.set_key_value("sparse_infill_density",
                        new Slic3r::ConfigOptionPercent(std::stoi(value)));
                } else if (key == "perimeters") {
                    config.set_key_value(key, new Slic3r::ConfigOptionInt(std::stoi(value)));
                } else if (key == "nozzle_temperature" || key == "bed_temperature") {
                    config.set_key_value(key, new Slic3r::ConfigOptionInts({std::stoi(value)}));
                }
            } catch (const std::exception& e) {
                std::cerr << "Warning: Invalid value for " << key << ": " << value << "\n";
            }
        }

        std::cout << "\nActive print settings:\n";
        if (config.has("layer_height"))
            std::cout << "  Layer height: " << config.opt_float("layer_height") << "mm\n";
        if (config.has("perimeters"))
            std::cout << "  Perimeters: " << config.opt_int("perimeters") << "\n";
        if (config.has("sparse_infill_density")) {
            auto p = config.option<Slic3r::ConfigOptionPercent>("sparse_infill_density");
            if (p) std::cout << "  Infill: " << p->value << "%\n";
        }
        if (config.has("nozzle_diameter")) {
            auto n = config.option<Slic3r::ConfigOptionFloatsNullable>("nozzle_diameter");
            if (n && !n->values.empty())
                std::cout << "  Nozzle: " << n->values[0] << "mm\n";
        }

        // Center STL objects on the printable bed. 3MF objects are already positioned.
        if (!is_bbl_3mf) {
            auto* pa = config.option<Slic3r::ConfigOptionPoints>("printable_area");
            if (pa && !pa->values.empty()) {
                Slic3r::BoundingBoxf bed_bbox;
                for (const auto& pt : pa->values) bed_bbox.merge(pt);
                Slic3r::Vec2d bed_center = bed_bbox.center();

                Slic3r::BoundingBoxf3 model_bbox;
                for (auto* obj : model.objects)
                    for (size_t i = 0; i < obj->instances.size(); ++i)
                        model_bbox.merge(obj->instance_bounding_box(i));

                if (model_bbox.defined) {
                    double dx = bed_center.x() - (model_bbox.min.x() + model_bbox.max.x()) / 2.0;
                    double dy = bed_center.y() - (model_bbox.min.y() + model_bbox.max.y()) / 2.0;
                    for (auto* obj : model.objects)
                        for (auto* inst : obj->instances) {
                            Slic3r::Vec3d off = inst->get_offset();
                            inst->set_offset(Slic3r::Vec3d(off.x() + dx, off.y() + dy, off.z()));
                        }
                    if (verbose)
                        std::cout << "  Centered model on bed: offset (" << dx << ", " << dy << ")\n";
                }
            }
        }

        std::cout << "\nInitializing print...\n";
        Slic3r::Print print;
        if (is_bbl_3mf) print.set_BBL_Printer(true);
        print.set_plate_origin(Slic3r::Vec3d(0.0, 0.0, 0.0));

        try {
            std::cout << "Applying configuration...\n";
            print.apply(model, config);
            print.set_status_callback(emit_status_warning);

            std::cout << "Validating...\n";
            Slic3r::StringObjectException validation_warning;
            Slic3r::StringObjectException validation_result = print.validate(&validation_warning);
            if (!validation_warning.string.empty()) {
                emit_validation_event(validation_warning);
                std::cout << "Validation warning: " << validation_warning.string << "\n";
            }
            if (!validation_result.string.empty()) {
                emit_validation_event(validation_result);
                std::cerr << "Validation error: " << validation_result.string << "\n";
                return 1;
            }

            std::cout << "Slicing...\n";
            print.process();
            std::cout << "\n✓ Slicing complete!\n";

            std::cout << "\nExporting G-code to: " << args.output_path << "\n";
            try {
                Slic3r::GCodeProcessorResult gcode_result;
                print.export_gcode(args.output_path, &gcode_result, nullptr);
                std::cout << "✓ G-code export complete!\n";
                std::cout << "\nOutput file: " << args.output_path << "\n";
            } catch (const Slic3r::RuntimeError& e) {
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind","RuntimeError"},{"message",std::string(e.what())}});
                std::cerr << "\n❌ G-code export failed: " << e.what() << "\n";
                return 1;
            } catch (const std::exception& e) {
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind",typeid(e).name()},{"message",std::string(e.what())}});
                std::cerr << "\n❌ G-code export failed: " << e.what() << "\n";
                return 1;
            }
        } catch (const std::exception& e) {
            emit_event({{"event","slicing_error"},{"phase","process"},{"kind",typeid(e).name()},{"message",std::string(e.what())}});
            std::cerr << "Slicing error: " << e.what() << "\n";
            return 1;
        }

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}

// ─── FFI entry points ─────────────────────────────────────────────────────────

int32_t slicer_run(::rust::Str job_json, EventSink& sink) {
    (void)sink;
    try {
        json job = json::parse(std::string_view(job_json.data(), job_json.size()));

        SliceArgs args;

        // input
        const auto& input = job.at("input");
        if (input.at("kind") == "path") {
            args.input_path = input.at("path").get<std::string>();
        } else {
            return 2; // URL inputs not yet supported
        }

        // output
        const auto& output = job.at("output");
        if (output.at("kind") == "path") {
            args.output_path = output.at("path").get<std::string>();
        } else {
            return 2; // S3 / HTTP PUT not yet supported
        }

        // config presets (inline JSON objects from the JobSpec)
        if (job.contains("machine"))  args.machine_json  = job["machine"];
        if (job.contains("process"))  args.process_json  = job["process"];
        if (job.contains("filament") && job["filament"].is_array())
            for (const auto& f : job["filament"])
                args.filament_jsons.push_back(f);

        return static_cast<int32_t>(run_slice_job(args));

    } catch (const std::exception&) {
        return 1;
    }
}

::rust::String slicer_list_presets(::rust::Str kind) {
    // TODO: load PresetBundle::load_system_presets() from resources/profiles/,
    // collect names from bundle.printers / bundle.filaments / bundle.prints
    // depending on `kind`, return as JSON array.
    (void)kind;
    return ::rust::String("[]");
}

::rust::String slicer_get_preset(::rust::Str kind, ::rust::Str name) {
    // TODO: load PresetBundle::load_system_presets() from resources/profiles/,
    // find preset by name in the appropriate collection, return as JSON object.
    (void)kind;
    (void)name;
    return ::rust::String("null");
}
