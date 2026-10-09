// cli_command_line.cpp — see cli_command_line.hpp.
//
// The token walk is the official DynamicConfig::read_cli (Config.cpp 1660-1782
// at BambuStudio 5873b5f; 1591-1713 at OrcaSlicer 31f6803), done one token at
// a time so that a flag slicer-cli does not have refuses with its own name and
// the reason, instead of read_cli's bare "Invalid option". Every value is
// stored exactly the way read_cli stores it, into this engine's own
// DynamicPrintAndCLIConfig; CLI::setup's split into actions and transforms,
// its validate(true) and its defaults follow (BambuStudio.cpp 8338-8367,
// OrcaSlicer.cpp 7109-7138).
#include "cli_command_line.hpp"

#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>

#include <boost/algorithm/string/predicate.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Utils.hpp"

#include "cli_flag_support.hpp"

namespace slicer_cli {
namespace {

// The engine this binary is built on, in the official app's own name.
#ifdef ENGINE_ORCA
const char* const kThisApp  = "OrcaSlicer";
const char* const kOtherApp = "BambuStudio";
#else
const char* const kThisApp  = "BambuStudio";
const char* const kOtherApp = "OrcaSlicer";
#endif

// ── slicer-cli's own flags ───────────────────────────────────────────────
// Kept from before the official flags; an official flag of the same name
// takes precedence (the walk asks the official table first), so a legacy
// entry here only ever names a flag the official CLIs do not have.
enum class Legacy {
    Verbose, Output, Machine, Filament, Process, Config,
    Infill, Perimeters, Nozzle, Temp, BedTemp,
    Plate, Input, NoNormalizeLegacyGcode,
    CalibMode, CalibStart, CalibEnd, CalibStep, CalibExtruderId, CalibNoNumbers,
    Layout, LayoutPlan, Progress,
    PrinterPreset, ProcessPreset, FilamentPreset, ListPresets, Printer,
    AllowSubstitution, EngineInfo, LayerHeight, SingleInstance,
};

struct LegacyFlag {
    const char* name;      // without leading dashes
    Legacy      id;
    bool        takes_value;
    const char* help_value;
    const char* help;
};

const std::vector<LegacyFlag>& legacy_flags() {
    static const std::vector<LegacyFlag> flags = {
        {"v",                         Legacy::Verbose,           false, "",        "Verbose engine log (same as --verbose)"},
        {"verbose",                   Legacy::Verbose,           false, "",        "Verbose engine log"},
        {"o",                         Legacy::Output,            true,  "FILE",    "Output G-code file for the default call (default: output.gcode)"},
        {"output",                    Legacy::Output,            true,  "FILE",    "Same as -o"},
        {"machine",                   Legacy::Machine,           true,  "FILE",    "Printer settings file (JSON)"},
        {"filament",                  Legacy::Filament,          true,  "FILE",    "Filament settings file (JSON)"},
        {"process",                   Legacy::Process,           true,  "FILE",    "Process settings file (JSON)"},
        {"config",                    Legacy::Config,            true,  "FILE",    "Settings bundle (JSON)"},
        {"layer-height",              Legacy::LayerHeight,       true,  "MM",      "Layer height (also the official setting flag of the same name)"},
        {"infill",                    Legacy::Infill,            true,  "PERCENT", "Sparse infill density 0-100 (sets sparse_infill_density)"},
        {"perimeters",                Legacy::Perimeters,        true,  "N",       "Wall count (sets wall_loops)"},
        {"nozzle",                    Legacy::Nozzle,            true,  "MM",      "Nozzle diameter (sets nozzle_diameter)"},
        {"temp",                      Legacy::Temp,              true,  "C",       "Nozzle temperature (sets nozzle_temperature)"},
        {"bed-temp",                  Legacy::BedTemp,           true,  "C",       "Bed temperature (sets bed_temperature)"},
        {"plate",                     Legacy::Plate,             true,  "N",       "Default call: slice only plate N of a 3MF (1-based)"},
        {"input",                     Legacy::Input,             true,  "FILE",    "Input file (layout modes; or the model file)"},
        {"no-normalize-legacy-gcode", Legacy::NoNormalizeLegacyGcode, false, "",   "Do not alias unbound legacy placeholders in custom G-code"},
        {"calib-mode",                Legacy::CalibMode,         true,  "MODE",    "Calibration test: temp_tower, retraction_tower, pressure_advance_line, pressure_advance_pattern, pressure_advance_tower"},
        {"calib-start",               Legacy::CalibStart,        true,  "N",       "Calibration sweep start"},
        {"calib-end",                 Legacy::CalibEnd,          true,  "N",       "Calibration sweep end"},
        {"calib-step",                Legacy::CalibStep,         true,  "N",       "Calibration sweep step (> 0)"},
        {"calib-extruder-id",         Legacy::CalibExtruderId,   true,  "N",       "Calibration extruder (default 0)"},
        {"calib-no-numbers",          Legacy::CalibNoNumbers,    false, "",        "No numeric labels (pressure_advance_line)"},
        {"layout",                    Legacy::Layout,            true,  "FILE",    "Headless arrange from an older JSON form"},
        {"layout-plan",               Legacy::LayoutPlan,        false, "",        "Headless arrange, versioned JSON contract (stdin or --input)"},
        {"progress",                  Legacy::Progress,          false, "",        "Progress events; on with --slice"},
        {"printer-preset",            Legacy::PrinterPreset,     true,  "NAME",    "Printer system preset by name, every parent applied"},
        {"process-preset",            Legacy::ProcessPreset,     true,  "NAME",    "Process system preset by name (default: the printer's)"},
        {"filament-preset",           Legacy::FilamentPreset,    true,  "NAME",    "Filament system preset by name; repeat for more filaments"},
        {"list-presets",              Legacy::ListPresets,       false, "",        "This engine's system presets as JSON"},
        {"printer",                   Legacy::Printer,           true,  "NAME",    "With --list-presets: only this printer's presets"},
        {"allow-substitution",        Legacy::AllowSubstitution, false, "",        "With --slice: slice a 3MF whose values this engine substitutes"},
        {"engine-info",               Legacy::EngineInfo,        true,  "FILE",    "What the file is and which engine binary fits it, as JSON"},
        {"single-instance",           Legacy::SingleInstance,    false, "",        "Accepted and ignored (the official app's one-window switch; commented out of its CLI)"},
    };
    return flags;
}

const LegacyFlag* find_legacy(const std::string& name) {
    for (const LegacyFlag& f : legacy_flags())
        if (name == f.name)
            return &f;
    return nullptr;
}

// The official option table: CLI name -> key (read_cli's cache, Config.cpp 1663-1666).
const std::map<std::string, std::string>& official_names() {
    static const std::map<std::string, std::string> names = [] {
        std::map<std::string, std::string> out;
        Slic3r::DynamicPrintAndCLIConfig cfg;
        for (const auto& kv : cfg.def()->options)
            for (const std::string& t : kv.second.cli_args(kv.first))
                out[t] = kv.first;
        return out;
    }();
    return names;
}

bool is_cli_key(const std::string& key) {
    return Slic3r::cli_actions_config_def.has(key) || Slic3r::cli_transform_config_def.has(key) ||
           Slic3r::cli_misc_config_def.has(key);
}

std::string dashed(const std::string& key) {
    std::string out = key;
    std::replace(out.begin(), out.end(), '_', '-');
    return out;
}

bool parse_int_text(const std::string& text, int& out) {
    try {
        size_t pos = 0;
        const long v = std::stol(text, &pos);
        if (pos != text.size() || v < INT32_MIN || v > INT32_MAX)
            return false;
        out = int(v);
        return true;
    } catch (...) {
        return false;
    }
}

bool parse_double_text(const std::string& text, double& out) {
    try {
        size_t pos = 0;
        out = std::stod(text, &pos);
        return pos == text.size();
    } catch (...) {
        return false;
    }
}

std::string join(const std::vector<std::string>& items, const char* sep) {
    std::string out;
    for (size_t i = 0; i < items.size(); ++i) {
        if (i) out += sep;
        out += items[i];
    }
    return out;
}

// Why a value did not take, in words a user can act on.
std::string bad_value_sentence(const std::string& flag, const Slic3r::ConfigOptionDef& def,
                               const std::string& value) {
    std::string s = "--" + flag + ": '" + value + "' is not a value " + kThisApp + " takes for this flag";
    if (!def.enum_values.empty())
        return s + "; use one of: " + join(def.enum_values, ", ");
    switch (def.type) {
    case Slic3r::coInt:
    case Slic3r::coInts:     return s + "; it takes a whole number" + (def.type == Slic3r::coInts ? "s, comma separated" : "");
    case Slic3r::coFloat:
    case Slic3r::coFloats:   return s + "; it takes a number" + std::string(def.type == Slic3r::coFloats ? "s, comma separated" : "");
    case Slic3r::coPercent:
    case Slic3r::coPercents: return s + "; it takes a percentage such as 15%";
    case Slic3r::coFloatOrPercent:
    case Slic3r::coFloatsOrPercents: return s + "; it takes a number or a percentage";
    case Slic3r::coBool:
    case Slic3r::coBools:    return s + "; it takes 1 or 0";
    case Slic3r::coPoint:
    case Slic3r::coPoints:   return s + "; it takes points as XxY";
    default:                 return s;
    }
}

// read_cli's store step (Config.cpp 1741-1779 at 5873b5f), returning false
// where read_cli would print "Invalid value" (and, stricter, where a vector
// or bool value does not parse, which read_cli would store half-read).
bool store_official(Slic3r::DynamicPrintAndCLIConfig& cfg, const std::string& opt_key,
                    const std::string& value, std::vector<std::string>& order) {
    const bool existing = cfg.has(opt_key);
    if (!existing)
        order.push_back(opt_key);
    Slic3r::ConfigOption* opt_base = cfg.option(opt_key, true);
    auto* opt_vector = opt_base->is_vector() ? static_cast<Slic3r::ConfigOptionVectorBase*>(opt_base) : nullptr;
    if (opt_vector) {
        if (!existing)
            opt_vector->clear();
        if (opt_base->type() == Slic3r::coBools && value.empty()) {
            static_cast<Slic3r::ConfigOptionBools*>(opt_base)->values.push_back(true);
            return true;
        }
        // Enum vectors need the key's map; set_deserialize_nothrow carries it.
        if (opt_base->type() == Slic3r::coEnums) {
            if (existing) {
                // Repeated enum-vector flag: append like read_cli.
                Slic3r::ConfigSubstitutionContext ctx(Slic3r::ForwardCompatibilitySubstitutionRule::Disable);
                return cfg.set_deserialize_nothrow(opt_key, value, ctx, true);
            }
            Slic3r::ConfigSubstitutionContext ctx(Slic3r::ForwardCompatibilitySubstitutionRule::Disable);
            return cfg.set_deserialize_nothrow(opt_key, value, ctx, false);
        }
        return opt_vector->deserialize(value, true);
    }
    if (opt_base->type() == Slic3r::coBool) {
        if (value.empty()) {
            static_cast<Slic3r::ConfigOptionBool*>(opt_base)->value = true;
            return true;
        }
        return opt_base->deserialize(value);
    }
    if (opt_base->type() == Slic3r::coString) {
        static_cast<Slic3r::ConfigOptionString*>(opt_base)->value = value;
        return true;
    }
    Slic3r::ConfigSubstitutionContext context(Slic3r::ForwardCompatibilitySubstitutionRule::Disable);
    return cfg.set_deserialize_nothrow(opt_key, value, context, false);
}

ParseRefusal refuse(const std::string& message) {
    ParseRefusal r;
    r.code    = CLI_INVALID_PARAMS;
    r.message = message;
    return r;
}

// The number refusals slicer-cli's own flags and --slice/--arrange have
// always given (sentence, usage, exit 1).
ParseRefusal refuse_number(const std::string& flag, const std::string& value, bool whole) {
    ParseRefusal r = refuse(flag + (whole ? " expects an integer, got '" : " expects a number, got '") + value + "'");
    r.with_usage = true;
    return r;
}

} // namespace

bool parse_command_line(int argc, char** argv, CliOptions& o, ModeArgs& m, ParseRefusal& refusal) {
    o.argv0 = argc > 0 ? argv[0] : "slicer_cli";
    Slic3r::DynamicPrintAndCLIConfig& cfg = o.cli;
    const auto& names = official_names();
    std::vector<std::string> order;
    std::vector<std::string> positional;

    // First pass for the two values a refusal needs: --slice (write
    // result.json) and --outputdir (where), read before any refusal. The
    // tokens are read as the parse below reads them: nothing after "--" is
    // a flag, and a flag that takes a value (an official non-bool option, a
    // slicer-cli flag with a value) consumes the next token unless it has an
    // inline =value, so a value spelled like a flag is never read as one.
    for (int i = 1; i < argc; ++i) {
        std::string t = argv[i];
        if (t == "--")
            break;
        if (!boost::starts_with(t, "-") || t == "-")
            continue;
        t.erase(t.begin(), t.begin() + (boost::starts_with(t, "--") ? 2 : 1));
        std::string v;
        bool inline_value = false;
        const size_t eq = t.find('=');
        if (eq != std::string::npos) { v = t.substr(eq + 1); t.erase(eq); inline_value = true; }
        std::string opt_key;
        bool takes_value = false;
        if (const auto it = names.find(t); it != names.end()) {
            opt_key = it->second;
            const Slic3r::ConfigOptionDef& def = cfg.def()->options.at(opt_key);
            takes_value = def.type != Slic3r::coBool && def.type != Slic3r::coBools;
        } else if (const LegacyFlag* lf = find_legacy(t)) {
            takes_value = lf->takes_value;
        }
        if (takes_value && !inline_value && i + 1 < argc)
            v = argv[++i];
        if (opt_key == "slice") {
            o.slice_given = true;
            int n = 0;
            if (parse_int_text(v, n)) { o.slice_mode = true; o.slice_plate = n; }
        } else if (opt_key == "outputdir") {
            o.outputdir = v;
        }
    }

    bool parse_options = true;
    for (int i = 1; i < argc; ++i) {
        std::string token = argv[i];
        if (!parse_options || !boost::starts_with(token, "-") || token == "-") {
            positional.push_back(token);
            continue;
        }
#ifdef __APPLE__
        if (boost::starts_with(token, "-psn_"))
            continue;   // read_cli, Config.cpp 1676-1681
#endif
        if (token == "--") {
            parse_options = false;
            continue;
        }
        const std::string as_typed = token;
        token.erase(token.begin(), token.begin() + (boost::starts_with(token, "--") ? 2 : 1));
        std::string value;
        bool has_inline_value = false;
        {
            const size_t eq = token.find('=');
            if (eq != std::string::npos) {
                value = token.substr(eq + 1);
                token.erase(eq);
                has_inline_value = true;
            }
        }

        // 1. An official flag of this binary.
        auto it = names.find(token);
        if (it != names.end()) {
            const std::string& opt_key = it->second;
            const Slic3r::ConfigOptionDef& def = cfg.def()->options.at(opt_key);
            if (!has_inline_value && def.type != Slic3r::coBool && def.type != Slic3r::coBools) {
                if (i == argc - 1) {
                    refusal = refuse("--" + token + " needs a value");
                    return false;
                }
                value = argv[++i];
            }
            const std::string why = official_flag_refusal(opt_key);
            if (!why.empty()) {
                refusal = refuse(why);
                return false;
            }
            int whole = 0;
            if ((opt_key == "slice" || opt_key == "arrange") && !parse_int_text(value, whole)) {
                refusal = refuse_number("--" + token, value, true);
                return false;
            }
            if (!store_official(cfg, opt_key, value, order)) {
                refusal = refuse(bad_value_sentence(token, def, value));
                return false;
            }
            // An enum value the engine's key map knows but does not list as a
            // choice ("Default Plate" for curr_bed_type, btDefault: a plate's
            // "same as the project" marker, PrintConfig.cpp 437-445 at 5873b5f,
            // 467-475 at 31f6803) is not a value for the setting itself; the
            // slice crashes on it. Only the listed values are taken.
            if (def.type == Slic3r::coEnum && !def.enum_values.empty()) {
                const std::string stored = cfg.option(opt_key)->serialize();
                if (std::find(def.enum_values.begin(), def.enum_values.end(), stored) == def.enum_values.end()) {
                    refusal = refuse(bad_value_sentence(token, def, value));
                    return false;
                }
            }
            continue;
        }

        // 2. slicer-cli's own flag.
        if (const LegacyFlag* lf = find_legacy(token)) {
            if (lf->takes_value && !has_inline_value) {
                if (i == argc - 1) {
                    refusal = refuse("--" + token + " needs a value");
                    return false;
                }
                value = argv[++i];
            }
            switch (lf->id) {
            case Legacy::Verbose:   o.verbose = true; break;
            case Legacy::Output:    o.output_file = value; break;
            case Legacy::Machine:   o.machine_config = value; break;
            case Legacy::Filament:  o.filament_config = value; break;
            case Legacy::Process:   o.process_config = value; break;
            case Legacy::Config:    o.bundle_config = value; break;
            case Legacy::LayerHeight: o.overrides["layer_height"] = value; break;
            case Legacy::Infill:    o.overrides["fill_density"] = value; break;
            case Legacy::Perimeters: o.overrides["perimeters"] = value; break;
            case Legacy::Nozzle:    o.overrides["nozzle_diameter"] = value; break;
            case Legacy::Temp:      o.overrides["nozzle_temperature"] = value; break;
            case Legacy::BedTemp:   o.overrides["bed_temperature"] = value; break;
            case Legacy::Plate:
                if (!parse_int_text(value, o.plate_id) || o.plate_id < 0) {
                    refusal = refuse("--plate: '" + value + "' is not a plate number; "
                                     "give 0 for every plate, or a plate number");
                    return false;
                }
                break;
            case Legacy::Input:     positional.insert(positional.begin(), value); break;
            case Legacy::NoNormalizeLegacyGcode: o.normalize_legacy_gcode = false; break;
            case Legacy::CalibMode: m.calib.mode = value; break;
            case Legacy::CalibStart:
            case Legacy::CalibEnd:
            case Legacy::CalibStep: {
                double d = 0;
                if (!parse_double_text(value, d)) {
                    refusal = refuse_number("--" + token, value, false);
                    return false;
                }
                if (lf->id == Legacy::CalibStart) { m.calib.start = d; m.calib.has_start = true; }
                if (lf->id == Legacy::CalibEnd)   { m.calib.end = d;   m.calib.has_end = true; }
                if (lf->id == Legacy::CalibStep)  { m.calib.step = d;  m.calib.has_step = true; }
                break;
            }
            case Legacy::CalibExtruderId:
                if (!parse_int_text(value, m.calib.extruder_id)) {
                    refusal = refuse_number("--calib-extruder-id", value, true);
                    return false;
                }
                break;
            case Legacy::CalibNoNumbers: m.calib.print_numbers = false; break;
            case Legacy::Layout:     m.layout_json_file = value; break;
            case Legacy::LayoutPlan: m.layout_plan_mode = true; break;
            case Legacy::Progress:   o.progress = true; break;
            case Legacy::PrinterPreset:  o.printer_preset = value; break;
            case Legacy::ProcessPreset:  o.process_preset = value; break;
            case Legacy::FilamentPreset: o.filament_presets.push_back(value); break;
            case Legacy::ListPresets:    m.list_presets = true; break;
            case Legacy::Printer:        m.list_printer = value; break;
            case Legacy::AllowSubstitution: o.allow_substitution = true; break;
            case Legacy::EngineInfo:     m.engine_info_file = value; break;
            case Legacy::SingleInstance: break;
            }
            continue;
        }

        // 3. Known names that are not flags of this binary: say which.
        const std::string why = foreign_flag_refusal(token);
        refusal = refuse(!why.empty() ? why
            : as_typed + " is not a flag of this binary and not a setting of its " + kThisApp +
              " engine; run --help for the flags");
        return false;
    }

    // CLI::setup: actions and transforms in command-line order (BambuStudio.cpp 8346-8351).
    for (const std::string& key : order) {
        o.given.insert(key);
        if (Slic3r::cli_actions_config_def.has(key))
            o.actions.push_back(key);
        else if (Slic3r::cli_transform_config_def.has(key))
            o.transforms.push_back(key);
    }

    // CLI::setup validates the given values as a full print config (8354, 8362-8367).
    {
        std::map<std::string, std::string> validity = cfg.validate(true);
        if (!validity.empty()) {
            std::string msg = "Params in command line error:";
            for (const auto& kv : validity)
                msg += " " + kv.first + ": " + kv.second + ";";
            msg.pop_back();
            refusal = refuse(msg);
            return false;
        }
    }
    // ... then every CLI key at its default (8357-8359).
    for (const Slic3r::t_optiondef_map* options : {&Slic3r::cli_actions_config_def.options,
                                                   &Slic3r::cli_transform_config_def.options,
                                                   &Slic3r::cli_misc_config_def.options})
        for (const auto& optdef : *options)
            cfg.option(optdef.first, true);

    // m_extra_config: every print setting given (BambuStudio.cpp 1563-1564,
    // OrcaSlicer.cpp 1258-1259).
    o.extra_config.apply(cfg, true);
    o.extra_config.normalize_fdm();
    if (const std::string why = extra_config_refusal(o.extra_config); !why.empty()) {
        refusal = refuse(why);
        return false;
    }

    // The official flags slicer-cli keeps in its own fields.
    o.input_files = positional;
    o.input_file  = positional.empty() ? std::string() : positional.front();
    if (o.given_flag("slice")) {
        o.slice_mode  = true;
        o.slice_plate = cfg.opt_int("slice");
        if (o.slice_plate < 0) {
            refusal = refuse("--slice expects 0 (every plate) or a plate number");
            refusal.with_usage = true;
            return false;
        }
    } else {
        o.slice_mode = false;
    }
    // Only what was given: some CLI definitions carry a default value
    // (export_3mf's is "output.3mf") that means nothing until the flag is used.
    o.outputdir        = o.given_flag("outputdir") ? cfg.opt_string("outputdir") : std::string();
    o.export_3mf       = o.given_flag("export_3mf") ? cfg.opt_string("export_3mf") : std::string();
    o.allow_newer_file = o.given_flag("allow_newer_file") && cfg.opt_bool("allow_newer_file");
    if (o.given_flag("arrange"))
        o.arrange = cfg.opt_int("arrange");
    if (o.given_flag("help"))
        m.help = true;
    if (o.given_flag("debug") && cfg.opt_int("debug") >= 4)
        o.verbose = true;

    // metadata_name and metadata_value pair up (BambuStudio.cpp 1691-1699).
    if (cfg.option<Slic3r::ConfigOptionStrings>("metadata_name")->values.size() !=
        cfg.option<Slic3r::ConfigOptionStrings>("metadata_value")->values.size()) {
        refusal = refuse("--metadata-name and --metadata-value must come in pairs");
        return false;
    }
    return true;
}

namespace {
// One flag line: the flag in a 34-wide column, or on its own line when wider.
void help_row(std::ostream& out, const std::string& lhs, std::string text) {
    for (char& c : text)
        if (c == '\n') c = ' ';
    while (!text.empty() && text.back() == ' ')
        text.pop_back();
    if (lhs.size() < 34)
        out << "  " << std::left << std::setw(34) << lhs << text << "\n";
    else
        out << "  " << lhs << "\n" << std::string(36, ' ') << text << "\n";
}
} // namespace

void print_help(std::ostream& out, const char* prog) {
    out << "Usage: " << prog << " [OPTIONS] [model files: .3mf .stl .obj .amf .step .stp"
#ifndef ENGINE_ORCA
        << " .glb .gltf .fbx"
#else
        << " .svg"
#endif
        << " ...]\n"
        << "\nEngine of this binary: " << kThisApp << ". The other slicer-cli binary runs " << kOtherApp << ".\n"
        << "\nThe default call:\n"
        << "  " << prog << " model.3mf --plate 1 -o plate1.gcode\n"
        << "The official call:\n"
        << "  " << prog << " model.3mf --slice 1 --outputdir out\n";

    out << "\nFlags that share a name with Bambu Studio's or OrcaSlicer's own command line behave the same way.\n"
        << "\"Works in\" names the binaries that take the flag: slicer_cli runs the Bambu Studio engine,\n"
        << "slicer_cli-orcaslicer the OrcaSlicer engine.\n";

    out << "\n=== Files, presets and modes ===\n";
    for (const auto& f : legacy_flags()) {
        if (official_names().count(f.name)) continue;   // listed with the shared names below
        std::string lhs = (std::string(f.name).size() == 1 ? "-" : "--") + std::string(f.name);
        if (f.takes_value) lhs += std::string(" ") + f.help_value;
        help_row(out, lhs, std::string(f.help) + "  [works in: both binaries]");
    }

    out << "\n=== Slicing, transforms and exports ===\n";
    for (const Slic3r::ConfigDef* d : {static_cast<const Slic3r::ConfigDef*>(&Slic3r::cli_misc_config_def),
                                       static_cast<const Slic3r::ConfigDef*>(&Slic3r::cli_transform_config_def),
                                       static_cast<const Slic3r::ConfigDef*>(&Slic3r::cli_actions_config_def)}) {
        for (const auto& kv : d->options) {
            const auto& def = kv.second;
            std::vector<std::string> args = def.cli_args(kv.first);
            if (args.empty()) continue;
            std::string lhs;
            for (const auto& a : args) lhs += (lhs.empty() ? "" : "|") + std::string(a.size() == 1 ? "-" : "--") + a;
            if (!def.cli_params.empty()) lhs += " " + def.cli_params;
            const std::string why = official_flag_refusal(kv.first);
            std::string text = def.tooltip + "  [works in: " + works_in(kv.first) + "]";
            if (!why.empty())
                text += "  [not supported: refuses with the reason]";
            else if (kv.first == "datadir")
                text += "  [accepted; no effect: slicer-cli keeps no settings between runs]";
            help_row(out, lhs, text);
        }
    }
    for (const std::string& name : other_engine_only_flags())
        help_row(out, "--" + name, "[works in: slicer_cli only (Bambu Studio engine)]");
    {
        std::vector<std::string> shared;
        for (const auto& f : legacy_flags())
            if (official_names().count(f.name))
                shared.push_back(std::string("--") + f.name);
        if (!shared.empty())
            out << "  Also: " << join(shared, ", ") << " (a setting flag, see below).\n";
    }

    out << "\n=== Every print setting ===\n"
        << "  Any setting of this binary's engine is a flag: --<setting-key with dashes> VALUE,\n"
        << "  for example --curr-bed-type \"Textured PEI Plate\" or --sparse-infill-density 15%.\n"
        << "  These win over the file's and the presets' values, for every plate.\n";
    {
        const Slic3r::ConfigOptionDef* bed = Slic3r::print_config_def.get("curr_bed_type");
        if (bed)
            out << "  --curr-bed-type values: " << join(bed->enum_values, ", ") << "\n";
    }
    out << "\nPrint settings priorities:\n"
        << "\t1) setting values from the command line (highest priority)\n"
        << "\t2) setting values loaded with --load-settings and --load-filaments\n"
        << "\t3) setting values loaded from 3mf (lowest priority)\n";
}

} // namespace slicer_cli
