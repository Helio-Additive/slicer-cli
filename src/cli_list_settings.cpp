#include "cli_list_settings.hpp"

#include "cli_project_archive.hpp"
#include "cli_flag_support.hpp"
#include "libslic3r/Preset.hpp"
#include <nlohmann/json.hpp>
#include <cfloat>
#include <iostream>

namespace slicer_cli {

using json = nlohmann::json;

/// True when the engine's own writer can give this option's default. An enum
/// default is written through the key map the *option* carries, and the engines
/// build some of them from a bare value (an initializer list), which leaves
/// that map unset while their writer walks it without a check
/// (ConfigOptionEnumsGenericTempl::serialize_single_value): such a default has
/// no name to report. Unset is not always null: the Bambu and Snapmaker
/// constructor initialises the member from itself (Config.hpp 2135 at
/// v02.09.01.70; 1679 at bac6fd5), so the map is trusted only when it is the
/// one the setting's own definition holds.
static bool default_is_writable(const Slic3r::ConfigOptionDef& def) {
    const Slic3r::ConfigOption* option = def.default_value.get();
    if (option == nullptr)
        return false;
    const auto trusted = [&def](const Slic3r::t_config_enum_values* map) {
        return map != nullptr && map == def.enum_keys_map;
    };
    if (const auto* one = dynamic_cast<const Slic3r::ConfigOptionEnumGeneric*>(option))
        return trusted(one->keys_map);
    if (const auto* many = dynamic_cast<const Slic3r::ConfigOptionEnumsGeneric*>(option))
        return trusted(many->keys_map);
    if (const auto* nullable = dynamic_cast<const Slic3r::ConfigOptionEnumsGenericNullable*>(option))
        return trusted(nullable->keys_map);
    return true;
}

/// --list-settings: this engine's settings table (PrintConfigDef) as one JSON
/// document — every key with what it means, its type, its default, its bounds
/// and the values it takes — so an agent driving this binary can find a
/// setting ("infill" -> sparse_infill_density, 0-100 %) without reading the
/// engine's source. Nothing is loaded, so nothing can warn: stdout is the
/// document, exit 0.
int run_list_settings() {
    using namespace Slic3r;
    // The type's own name, and whether it is the plural (vector) one: the two
    // group types are defined with the vector bit already set (10 +
    // coVectorType, 11 + coVectorType), so they are matched on that value.
    const auto type_text = [](ConfigOptionType type, bool& vector) {
        switch (int(type)) {
        case coFloat:            vector = false; return std::string("float");
        case coFloats:           vector = true;  return std::string("floats");
        case coInt:              vector = false; return std::string("int");
        case coInts:             vector = true;  return std::string("ints");
        case coString:           vector = false; return std::string("string");
        case coStrings:          vector = true;  return std::string("strings");
        case coPercent:          vector = false; return std::string("percent");
        case coPercents:         vector = true;  return std::string("percents");
        case coFloatOrPercent:   vector = false; return std::string("float_or_percent");
        case coFloatsOrPercents: vector = true;  return std::string("floats_or_percents");
        case coPoint:            vector = false; return std::string("point");
        case coPoints:           vector = true;  return std::string("points");
        case coPoint3:           vector = false; return std::string("point3");
        case coBool:             vector = false; return std::string("bool");
        case coBools:            vector = true;  return std::string("bools");
        case coEnum:             vector = false; return std::string("enum");
        case coEnums:            vector = true;  return std::string("enums");
#ifndef ENGINE_SNAPMAKER
        // ConfigOptionPointsGroups / ConfigOptionIntsGroups are OrcaSlicer 2.4
        // only (no such types at bac6fd5).
        case coPointsGroups:     vector = true;  return std::string("points_groups");
        case coIntsGroups:       vector = true;  return std::string("ints_groups");
#endif
        default:                 vector = false; return std::string("none");
        }
    };
    const auto mode_text = [](ConfigOptionMode mode) {
        switch (mode) {
        case comSimple:   return std::string("simple");
        case comAdvanced: return std::string("advanced");
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
        // ConfigOptionMode::comExpert is OrcaSlicer 2.4 only (absent at bac6fd5).
        case comExpert:   return std::string("expert");
#endif
        case comDevelop:  return std::string("develop");
        }
        return std::string("simple");
    };
    // Which preset holds the key: the engine's own lists (Preset.cpp). The
    // meta keys they share ("inherits", "compatible_printers") are named the
    // way the engines name them, by the last list that holds them: the lists
    // are laid down printer, filament, process, and the process list has the
    // last word — a key in several of them is reported as process.
    static const std::map<std::string, std::string> scopes = [] {
        std::map<std::string, std::string> out;
        for (const std::string& key : Preset::printer_options())  out[key] = "printer";
        for (const std::string& key : Preset::filament_options()) out[key] = "filament";
        for (const std::string& key : Preset::print_options())    out[key] = "process";
        return out;
    }();
    json settings = json::array();
    for (const auto& option : print_config_def.options) {
        const std::string& key = option.first;
        const ConfigOptionDef& def = option.second;
        bool vector = false;
        json entry;
        entry["key"]    = key;
        entry["type"]   = type_text(def.type, vector);
        entry["vector"] = vector;
        entry["mode"]   = mode_text(def.mode);
        if (!def.label.empty())      entry["label"] = def.label;
        if (!def.full_label.empty()) entry["full_label"] = def.full_label;
        if (!def.category.empty())   entry["category"] = def.category;
        if (!def.tooltip.empty())    entry["tooltip"] = def.tooltip;
        if (!def.sidetext.empty())   entry["unit"] = def.sidetext;
        // The default as the engine writes it, where the engine's own writer
        // can write it (see default_is_writable).
        if (default_is_writable(def))
            entry["default"] = def.default_value->serialize();
        // Only a real bound: the engines mark "none" with a sentinel
        // (min_default/max_default in BambuStudio, -/+FLT_MAX in OrcaSlicer).
#ifdef ENGINE_ORCA
        const float min_default = -FLT_MAX, max_default = FLT_MAX;
#else
        const double min_default = ConfigOptionDef::min_default, max_default = ConfigOptionDef::max_default;
#endif
        if (def.min > min_default) entry["min"] = def.min;
        if (def.max < max_default) entry["max"] = def.max;
        if (const auto scope = scopes.find(key); scope != scopes.end())
            entry["scope"] = scope->second;
        if (!def.enum_values.empty()) {
            json values = json::array();
            for (size_t i = 0; i < def.enum_values.size(); ++i) {
                const bool labelled = i < def.enum_labels.size() && !def.enum_labels[i].empty();
                values.push_back(json{{"value", def.enum_values[i]},
                                      {"label", labelled ? def.enum_labels[i] : def.enum_values[i]}});
            }
            entry["enum"] = std::move(values);
        }
        // A setting this binary refuses as a flag says so, with the sentence
        // the refusal gives (cli_flag_support.cpp).
        if (const std::string why = slicer_cli::setting_flag_refusal(key); !why.empty())
            entry["refused_as_flag"] = why;
        settings.push_back(std::move(entry));
    }
    json out;
#ifdef ENGINE_SNAPMAKER
    out["engine"] = "snapmaker";
#elif defined(ENGINE_ORCA)
    out["engine"] = "orcaslicer";
#else
    out["engine"] = "bambustudio";
#endif
    out["engine_version"] = engine_version_text();
    out["settings"]       = std::move(settings);
    std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
    return 0;
}

} // namespace slicer_cli
