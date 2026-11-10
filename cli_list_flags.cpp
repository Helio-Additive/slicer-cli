// cli_list_flags.cpp — see cli_list_flags.hpp.
#include "cli_list_flags.hpp"

#include <algorithm>
#include <iostream>
#include <nlohmann/json.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "cli_command_line.hpp"
#include "cli_flag_support.hpp"

namespace slicer_cli {
namespace {

using json = nlohmann::json;

// The value type of an official CLI flag, in the words --list-settings uses.
std::string type_text(Slic3r::ConfigOptionType type) {
    switch (int(type)) {
    case Slic3r::coFloat:   return "float";
    case Slic3r::coFloats:  return "floats";
    case Slic3r::coInt:     return "int";
    case Slic3r::coInts:    return "ints";
    case Slic3r::coString:  return "string";
    case Slic3r::coStrings: return "strings";
    case Slic3r::coPercent: return "percent";
    case Slic3r::coPoint:   return "point";
    case Slic3r::coPoints:  return "points";
    case Slic3r::coBool:    return "bool";
    case Slic3r::coBools:   return "bools";
    case Slic3r::coEnum:    return "enum";
    default:                return "other";
    }
}

std::string dash(const std::string& name) {
    return (name.size() == 1 ? "-" : "--") + name;
}

// An official flag of this engine's CLI definitions (CLIActionsConfigDef,
// CLITransformConfigDef, CLIMiscConfigDef), as --help lists it.
json official_entry(const std::string& key, const Slic3r::ConfigOptionDef& def) {
    const std::vector<std::string> names = def.cli_args(key);
    json entry;
    entry["flag"]        = dash(names.front());
    entry["source"]      = "official";
    entry["key"]         = key;
    entry["takes_value"] = def.type != Slic3r::coBool;
    if (names.size() > 1) {
        json aliases = json::array();
        for (size_t i = 1; i < names.size(); ++i) aliases.push_back(dash(names[i]));
        entry["aliases"] = std::move(aliases);
    }
    if (!def.cli_params.empty()) entry["value"] = def.cli_params;
    entry["type"] = type_text(def.type);
    if (!def.enum_values.empty()) entry["values"] = def.enum_values;
    if (def.default_value) {
        try { entry["default"] = def.default_value->serialize(); } catch (...) {}
    }
    entry["help"]     = def.tooltip;
    entry["works_in"] = works_in(key);
    const std::string refusal = official_flag_refusal(key);
    entry["supported"] = refusal.empty();
    if (!refusal.empty()) entry["refusal"] = refusal;
    if (key == "datadir") entry["note"] = "accepted; no effect: slicer-cli keeps no settings between runs";
    return entry;
}

} // namespace

int run_list_flags(const std::string& engine, const std::string& engine_version) {
    std::vector<json> flags;
    for (const OwnFlag& f : own_flags()) {
        if (is_official_flag_name(f.name)) continue;   // the official definition wins at parse time
        json entry;
        entry["flag"]        = dash(f.name);
        entry["source"]      = "slicer-cli";
        entry["takes_value"] = f.takes_value;
        if (f.takes_value) entry["value"] = f.value_name;
        entry["help"]      = f.help;
        entry["works_in"]  = f.works_in;
        entry["supported"] = f.refusal.empty();
        if (!f.refusal.empty()) entry["refusal"] = f.refusal;
        flags.push_back(std::move(entry));
    }
    // The modes --help explains in prose rather than in its tables.
    {
        json help;
        help["flag"] = "--help"; help["aliases"] = json::array({"-h"}); help["source"] = "slicer-cli";
        help["takes_value"] = false; help["help"] = "The flags of this program"; help["works_in"] = all_binaries_label();
        help["supported"] = true;
        flags.push_back(std::move(help));
        json timing;
        timing["flag"] = "--gcode-timing"; timing["source"] = "slicer-cli"; timing["takes_value"] = true;
        timing["value"] = "ORIGINAL OPTIMIZED";
        timing["help"] = "Import two existing G-code files with this binary's native processor and print their normal-mode estimates as one JSON document; never slices or rewrites either input";
        timing["works_in"] = all_binaries_label(); timing["supported"] = true;
        flags.push_back(std::move(timing));
#ifdef ENGINE_ORCA
        json vendor;
        vendor["flag"] = "--printer-vendor-bbl"; vendor["source"] = "slicer-cli"; vendor["takes_value"] = true;
        vendor["value"] = "true|false";
        vendor["help"] = "Required with --gcode-timing: whether the original printer preset belongs to the BBL vendor";
        vendor["works_in"] = "slicer_cli-orcaslicer and slicer_cli-snapmaker"; vendor["supported"] = true;
        flags.push_back(std::move(vendor));
#endif
    }
    for (const Slic3r::ConfigDef* d : {static_cast<const Slic3r::ConfigDef*>(&Slic3r::cli_misc_config_def),
                                       static_cast<const Slic3r::ConfigDef*>(&Slic3r::cli_transform_config_def),
                                       static_cast<const Slic3r::ConfigDef*>(&Slic3r::cli_actions_config_def)}) {
        for (const auto& kv : d->options) {
            if (kv.second.cli_args(kv.first).empty()) continue;
            json entry = official_entry(kv.first, kv.second);
            // The Snapmaker engine defines downward_check in two of its CLI
            // tables; one flag, listed once.
            const bool seen = std::any_of(flags.begin(), flags.end(),
                                          [&](const json& f) { return f["flag"] == entry["flag"]; });
            if (!seen) flags.push_back(std::move(entry));
        }
    }
    // Flags only the Bambu Studio build defines, named from the other builds
    // so a caller sees the whole package's contract from any one binary.
    for (const std::string& name : other_engine_only_flags()) {
        json entry;
        entry["flag"]        = dash(name);
        entry["source"]      = "official";
        std::string key = name;
        std::replace(key.begin(), key.end(), '-', '_');
        entry["key"]         = key;
        entry["takes_value"] = false;
        entry["help"]        = "";
        entry["works_in"]    = "slicer_cli only (Bambu Studio engine)";
        entry["supported"]   = false;
        entry["refusal"]     = foreign_flag_refusal(name);
        flags.push_back(std::move(entry));
    }
    std::sort(flags.begin(), flags.end(), [](const json& a, const json& b) {
        return a["flag"].get<std::string>() < b["flag"].get<std::string>();
    });
    json out;
    out["engine"]         = engine;
    out["engine_version"] = engine_version;
    out["flags"]          = json(flags);
    std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
    return 0;
}

} // namespace slicer_cli
