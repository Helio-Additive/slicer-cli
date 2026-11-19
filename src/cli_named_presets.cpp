#include "cli_named_presets.hpp"
#include <boost/nowide/fstream.hpp>

#include "cli_run_context.hpp"
#include "cli_options.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/AppConfig.hpp"
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include "cli_engine_events.hpp"
#include "cli_preset_staging.hpp"
#include "cli_printer_catalog.hpp"
#include "cli_snapmaker.hpp"
#include "desktop_presets.hpp"

namespace slicer_cli {

using json = nlohmann::json;

// ── Presets by name (desktop-app inheritance) ────────────────────────────────
// The desktop app loads every vendor's system presets with
// PresetBundle::load_system_presets_from_json (BambuStudio PresetBundle.cpp
// 1594 at 5873b5f; OrcaSlicer PresetBundle.cpp 2163 at 31f6803), which
// flattens each preset over its "inherits" chain while it loads
// (load_vendor_configs_from_json), and builds the print's settings with
// PresetBundle::full_config(): defaults, then process, printer and filament
// presets. Neither official CLI does this for --load-settings files: they
// read the one file given and stop at its parent. This path is the desktop's.

/// This engine's profiles tree: Bambu's for the Bambu build, Orca's for Orca.
boost::filesystem::path this_engine_profiles_dir(const std::string& argv0) {
    const boost::filesystem::path exe_dir = engine_executable_dir(argv0.c_str());
#ifdef ENGINE_SNAPMAKER
    return slicer_cli::snapmaker_profiles_dir(exe_dir);
#elif defined(ENGINE_ORCA)
    return orca_profiles_dir(exe_dir);
#else
    return engine_profiles_dir(exe_dir);
#endif
}

/// --machine, --process, --filament and --config take any settings file,
/// including the presets this engine ships, which hold only their differences
/// from their parents (load_vendor_configs_from_json: `config = *default_config;
/// config.apply(config_src)`, OrcaSlicer PresetBundle.cpp 4894, BambuStudio
/// 5087). Read as given, such a file leaves every key its parents define at
/// the engine's default — a 74-key machine preset slices as a mixture of two
/// machines — so the chain is resolved out of this engine's bundled profiles
/// and applied over the file's own keys. A file whose parent this engine does
/// not ship keeps its own keys: these flags accept a person's own file, and
/// the desktop reads one over its system parent only when its tree holds it
/// (PresetCollection::load_external_preset, OrcaSlicer Preset.cpp 2445-2456).
void apply_file_inherits(const std::string& argv0, const std::string& file, const char* kind,
                                Slic3r::DynamicPrintConfig& config, bool verbose) {
    std::string from, parent, stated_type;
    try {
        boost::nowide::ifstream f(file);
        if (!f.is_open())
            return;
        const json j = json::parse(f);
        from        = j.value("from", std::string());
        parent      = j.value("inherits", std::string());
        stated_type = j.value("type", std::string());
    } catch (...) {
        return;   // load_json_config reports a file that cannot be read
    }
    // The file's own "type" first: --config takes a whole settings file, and a
    // shipped preset says which kind it is. The flag's own kind is the fallback
    // (--machine, --process, --filament), and --config's "bundle" is no kind at
    // all, so a bundle file without a "type" is left as it is.
    std::string type = stated_type;
    if (type != "machine" && type != "process" && type != "filament")
        type = std::string(kind) == "bundle" ? std::string() : std::string(kind);
    if (type != "machine" && type != "process" && type != "filament")
        return;
    if (parent.empty())
        return;
    Slic3r::DynamicPrintConfig own;
    std::map<std::string, std::string> key_values;
    std::string reason;
    try {
#ifdef ENGINE_SNAPMAKER
        // This file's own values are laid over `config` again below, so the
        // flow-type words its first read converted are converted here too:
        // the substitutions are recorded (EnableSilent records none,
        // Config.cpp 683-684 at bac6fd5) and nothing reports them.
        Slic3r::ConfigSubstitutionContext substitutions(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
        own.load_from_json(file, substitutions, /*load_inherits_in_config=*/false, key_values, reason);
        slicer_cli::convert_flow_type_spelling(own, substitutions, file, /*announce=*/false);
#else
        Slic3r::ConfigSubstitutionContext substitutions(Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
        own.load_from_json(file, substitutions, /*load_inherits_in_config=*/false, key_values, reason);
#endif
    } catch (...) {
        return;
    }
    if (!reason.empty())
        return;
    // load_from_json takes "inherits" into its key-values and not into the
    // config; the chain is resolved from the config's own key, so put the name
    // back (the CLI's loader keeps it: it reads the substitution-context form).
    own.set("inherits", parent, true);
    std::string resolved_parent;
    const slicer_cli::InheritsResolution resolved = slicer_cli::apply_inherited_settings(
        this_engine_profiles_dir(argv0).string(), type, from.empty() ? std::string("system") : from, own,
        resolved_parent, file);
    if (resolved == slicer_cli::InheritsResolution::Resolved) {
        config.apply(own, /*ignore_nonexistent=*/true);
        if (verbose)
            std::cout << "Applied " << resolved_parent << ", the preset " << file << " inherits\n";
    }
}

/// A string setting of `config`, empty when the key is not there. The project's
/// preset ids are read this way: a 3MF that states none is not a project this
/// command line rebases. Bambu reads printer_variant the same way for the
/// nozzle statistics.
std::string config_id_text(const Slic3r::DynamicPrintConfig& config, const char* key) {
    const auto* opt = config.option<Slic3r::ConfigOptionString>(key);
    return opt != nullptr ? opt->value : std::string();
}

#ifdef ENGINE_ORCA
/// Any setting's value as the engine writes it (`serialize`), empty when the key
/// is not there: a coEnum such as support_type has no ConfigOptionString behind
/// it, so the text a decision reads is the serialized one. Takes the config
/// base, so an object's own settings (ModelConfigObject) read the same way as a
/// project's.
std::string config_value_text(const Slic3r::ConfigBase& config, const char* key) {
    const Slic3r::ConfigOption* opt = config.option(key);
    return opt != nullptr ? opt->serialize() : std::string();
}

#endif
/// True when this engine ships the system presets the project names. The
/// desktop loads the project over the preset it names and only warns when the
/// preset is missing (Preset.cpp 2446-2457 load_external_preset runs with
/// found = false); this command line keeps the flat file config for the whole
/// project instead, and says so in an event.
bool project_presets_shipped(Slic3r::PresetBundle& bundle, const std::string& printer, const std::string& process) {
#ifdef ENGINE_SNAPMAKER
    // A project saved under a preset's former name names that preset: the
    // fork's load_external_preset matches the original name against the
    // renamed_from map (Preset.cpp 2114-2120 at bac6fd5), though its
    // find_preset, unlike upstream's, does not (Preset.cpp 2776-2783).
    return !printer.empty() && !process.empty() && slicer_cli::preset_shipped_or_renamed(bundle.printers, printer) &&
           slicer_cli::preset_shipped_or_renamed(bundle.prints, process);
#else
    return !printer.empty() && !process.empty() &&
           bundle.printers.find_preset(printer, false) != nullptr &&
           bundle.prints.find_preset(process, false) != nullptr;
#endif
}

/// Loads this engine's system presets the way the desktop app does: the
/// profiles tree staged as <data_dir>/system, then
/// load_system_presets_from_json. `staging` is removed by the caller's guard.
bool load_system_presets(const std::string& argv0, Slic3r::PresetBundle& bundle,
                                const boost::filesystem::path& staging, std::string& error,
                                bool document_mode) {
    const boost::filesystem::path profiles_dir = this_engine_profiles_dir(argv0);
    if (profiles_dir.empty()) {
        error = "No profiles tree was found beside this binary.";
        return false;
    }
    const boost::filesystem::path sysdir = staging / "system";
    boost::filesystem::create_directories(sysdir);
    for (auto& entry : boost::filesystem::directory_iterator(profiles_dir)) {
        const auto dst = sysdir / entry.path().filename();
        if (boost::filesystem::is_directory(entry.path()))
            stage_vendor_dir(entry.path(), dst);
        else if (entry.path().extension() == ".json" && entry.path().stem() != "blacklist")
            stage_file_copy(entry.path(), dst);
    }
    Slic3r::set_data_dir(staging.string());
    // The desktop app's own entry point (PresetBundle::load_presets, Bambu
    // PresetBundle.cpp 551 / Orca 513): system presets through
    // load_system_presets_from_json, then the (empty) user folder, then the
    // compatibility pass. A fresh AppConfig is what a first launch has. Since
    // Bambu Studio 2.9.1 the bundle keeps a pointer to it: load_presets ->
    // load_selections stores it as the nozzle inventory's history
    // (PresetBundle.cpp 580, 2382 at v02.09.01.70) and every later
    // select_printer reads it (NozzleInventory.cpp 478-494), so it must
    // outlive every bundle, as the desktop's one app config does.
    static Slic3r::AppConfig app_config;
#ifdef ENGINE_ORCA
    bundle.load_presets(app_config, Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
#else
    auto [substitutions, errors] =
        bundle.load_presets(app_config, Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
    (void)substitutions;
    if (!errors.empty()) {
        // --list-presets owns stdout as one JSON document: warnings go to stderr there.
        if (document_mode)
            std::cerr << "Warning: " << errors << "\n";
        else
            emit_event({{"event","preset_warning"}, {"tag","SystemPresetLoadErrors"}, {"message", errors}});
    }
#endif
    return true;
}

/// A vendor preset a person can pick. Templates (instantiation "false") never
/// become presets at all (load_vendor_configs_from_json keeps them only as
/// parents), and the built-in "Default" presets are not system presets.
static bool is_listed_preset(const Slic3r::Preset& preset) {
    return preset.is_system && !preset.is_default;
}

/// Names of presets similar `wanted` (same words), for a refusal that points on.
static std::string near_preset_names(const Slic3r::PresetCollection& collection, const std::string& wanted) {
    std::vector<std::string> words;
    boost::algorithm::split(words, wanted, boost::is_any_of(" @"), boost::token_compress_on);
    std::vector<std::pair<int, std::string>> scored;
    for (const Slic3r::Preset& preset : collection) {
        if (!is_listed_preset(preset)) continue;
        int score = 0;
        for (const std::string& w : words)
            if (!w.empty() && boost::algorithm::icontains(preset.name, w)) ++score;
        if (score > 0) scored.emplace_back(-score, preset.name);
    }
    std::sort(scored.begin(), scored.end());
    std::string out;
    for (size_t i = 0; i < scored.size() && i < 5; ++i)
        out += (i ? "; " : "") + scored[i].second;
    return out;
}

/// Selects the named presets (printer first, then its compatible process and
/// filaments; an omitted process or filament takes the printer's own
/// default_print_profile / default_filament_profile, as the desktop app does
/// when a printer is picked) and returns full_config(). Refuses an unknown
/// name with close matches, and a process or filament the printer cannot use
/// (official CLI_PROCESS_NOT_COMPATIBLE, BambuStudio.cpp at 5873b5f).
/// A preset of this name exists and suits the selected printer.
static bool preset_usable(const Slic3r::PresetCollection& collection, const std::string& name) {
    const Slic3r::Preset* p = name.empty() ? nullptr : collection.find_preset(name, false);
    return p && p->name == name && p->is_compatible;
}

/// The first compatible, listed preset with the highest match quality, the
/// desktop's PresetCollection::first_compatible_idx (Preset.hpp 630 at
/// 5873b5f, 686-709 at 31f6803): ties keep the earlier preset in collection
/// order. The desktop counts only visible presets; here a listed preset is
/// visible, as naming the printer installs its presets, the way the setup
/// wizard does.
template<class Quality>
static std::string first_compatible_name(const Slic3r::PresetCollection& collection, Quality quality) {
    std::string best;
    int best_quality = -1;
    for (const Slic3r::Preset& p : collection) {
        if (!is_listed_preset(p) || !p.is_compatible) continue;
        const int q = quality(p);
        if (q > best_quality) { best_quality = q; best = p.name; }
    }
    return best;
}

/// The process the desktop selects for a printer picked with no process
/// selected yet: PresetBundle::update_compatible re-selects through
/// first_compatible_idx with PreferedPrintProfileMatch(nullptr,
/// default_print_profile), which scores the printer's default name 3 and
/// any other visible preset 2 (PresetBundle.cpp 5617-5640 at 5873b5f,
/// 5215-5240 at 31f6803). A default this engine does not ship (the Orca
/// pin's Snapmaker U1 0.4 names "0.20mm Standard @Snapmaker") therefore
/// gives the first compatible process.
///
/// With no compatible process at all, first_compatible_idx returns index 0,
/// the built-in default process (Preset.hpp 686-709 at 31f6803: "No
/// compatible preset found, return the default preset"; the same at
/// 5873b5f), and the desktop slices with that.

static std::string desktop_default_process(const Slic3r::PresetCollection& prints, const std::string& declared) {
#ifdef ENGINE_SNAPMAKER
    if (const std::string stated = slicer_cli::snapmaker_stated_default_process(prints, declared, is_listed_preset);
        !stated.empty())
        return stated;
#endif
    const std::string picked =
        first_compatible_name(prints, [&](const Slic3r::Preset& p) { return (p.name == declared ? 1 : 0) + 2; });
    return picked.empty() ? prints.default_preset().name : picked;
}

/// The filament the desktop selects for a printer picked on a fresh install
/// (PresetBundle::load_selections, PresetBundle.cpp 2740-2753 and 2850-2857
/// at 31f6803): the printer's first default filament when it is a visible,
/// compatible preset, else the placeholder, which the last step replaces with
/// filaments.first_compatible() in every slot. With no installed-filaments
/// section every system filament is visible (Preset::set_visible_from_appconfig,
/// Preset.cpp 830-843), so the pick is the first compatible filament in
/// collection order: on the Snapmaker U1 "Generic ABS @System", as the
/// OrcaSlicer 2.4.2 app selects. (update_compatible's PLA preference sets
/// only the editor's preset, which this step overrides.)
static std::string desktop_default_filament(const Slic3r::PresetCollection& filaments,
                                            const std::vector<std::string>& declared) {
    if (!declared.empty() && preset_usable(filaments, declared.front()))
        return declared.front();
#ifdef ENGINE_SNAPMAKER
    if (const std::string material = slicer_cli::snapmaker_default_material(filaments, declared, is_listed_preset);
        !material.empty())
        return material;
#endif
    return first_compatible_name(filaments, [](const Slic3r::Preset&) { return 0; });
}

/// Why the desktop's pick is not the printer's first stated default, or
/// empty when it is.
static std::string default_replaced_note(const char* kind, const std::string& picked, const std::string& declared) {
    if (picked.empty() || picked == declared) return {};
#ifdef ENGINE_SNAPMAKER
    // A process the tree names without the "mm" its presets spell
    // (desktop_default_process) is the printer's own default, not a
    // replacement of it.
    if (picked == slicer_cli::declared_with_mm_unit(declared)) return {};
#endif
    std::string why;
    if (declared.empty())
        why = "the printer names no default";
    else
        why = "the printer's default '" + declared + "' is not available";
    return std::string(kind) + " '" + picked + "' (" + why + ")";
}


/// The named system preset of `collection`, made visible the way naming a
/// preset installs it — a fresh AppConfig has no printer models installed, so
/// system presets load invisible, and select_preset_by_name skips an invisible
/// preset (Preset.cpp 3206 at 5873b5f / 3493 at 31f6803) — or a refusal naming
/// the close matches. Only the presets --list-presets offers are taken, never
/// a built-in "- default -", which `allow_default` takes solely for the
/// desktop's own fallback pick.
static bool find_named_preset(Slic3r::PresetCollection& collection, const std::string& name, const char* kind,
                              bool allow_default, const std::string& app, int& code, std::string& error) {
    Slic3r::Preset* preset = name.empty() ? nullptr : collection.find_preset(name, false);
    if (preset && preset->name == name && (is_listed_preset(*preset) || (allow_default && preset->is_default))) {
        preset->is_visible = true;
        return true;
    }
    const std::string similar = near_preset_names(collection, name);
    error = app + " has no " + kind + " preset named '" + name + "'." +
            (similar.empty() ? std::string() : " Close names: " + similar + ".") + " See --list-presets.";
    code = CLI_CONFIG_FILE_ERROR;
    return false;
}

/// One named system preset written out as a settings file the official merge
/// reads: its settings with every parent already applied (Preset::config holds
/// them — the vendor loader builds each preset over its parents,
/// load_vendor_configs_from_json), plus the keys that make it a system preset
/// of its own type. save_to_json is the engine's own writer, so the values
/// keep their encoding; it writes no "type", "instantiation" or "filament_id",
/// which load_config_file requires, so they are added after it.
static bool write_named_preset_file(const std::string& dir, const Slic3r::Preset& preset, const char* type,
                                    const std::string& stem, std::string& path, int& code, std::string& error) {
    path = dir + "/" + stem + ".json";
    try {
        preset.config.save_to_json(path, preset.name, "system", preset.version.to_string());
        json j;
        {
            boost::nowide::ifstream in(path);
            in >> j;
        }
        j["type"]          = type;
        j["instantiation"] = "true";
        if (std::string(type) == "filament")
            j["filament_id"] = preset.filament_id;
        boost::nowide::ofstream out(path, std::ios::out | std::ios::trunc);
        out << j.dump(1, '\t') << std::endl;
        if (!out.good()) {
            error = "Writing the settings file for preset '" + preset.name + "' failed";
            code = CLI_ENVIRONMENT_ERROR;
            return false;
        }
    } catch (const std::exception& e) {
        error = "Writing the settings file for preset '" + preset.name + "' failed: " + e.what();
        code = CLI_ENVIRONMENT_ERROR;
        return false;
    }
    return true;
}

/// --printer-preset, with --process-preset / --filament-preset, on a project
/// 3MF: the desktop app's printer switch. Naming a printer is what the
/// desktop's printer list does — and what the shipped presets cannot do by
/// path, since each holds only its differences from its parents — so the named
/// presets are written out in full into `dir` and given to the same merge
/// --load-settings runs. The switch then goes through the desktop's own code:
/// the process the new printer takes (desktop_printer_switch_process), the
/// plate moved onto the new bed (MovedToNewBed) and the printer's own plate
/// (desktop_bed_type).
bool stage_named_presets_for_project(CliOptions& o, const std::string& dir, int& code, std::string& error) {
    namespace fs = boost::filesystem;
    if (o.printer_preset.empty()) {
        error = "--process-preset and --filament-preset need --printer-preset.";
        code = CLI_INVALID_PARAMS;
        return false;
    }
    struct Cleanup { fs::path dir; ~Cleanup() { remove_owned_temp_dir(dir); } } cleanup;
    Slic3r::PresetBundle bundle;
    try {
        const fs::path staging = make_owned_temp_dir("slicer_cli_named-");
        cleanup.dir = staging;
        if (!load_system_presets(o.argv0, bundle, staging, error)) { code = CLI_ENVIRONMENT_ERROR; return false; }
    } catch (const std::exception& e) {
        error = std::string("Loading the system presets failed: ") + e.what();
        code = CLI_ENVIRONMENT_ERROR;
        return false;
    }
#ifdef ENGINE_ORCA
    slicer_cli::apply_preset_renames(o.printer_preset, o.process_preset, o.filament_presets, bundle);
    const std::string app = "OrcaSlicer";
#else
    const std::string app = "BambuStudio";
#endif
    const auto add_settings_file = [&o](const std::string& path) {
        o.cli.option<Slic3r::ConfigOptionStrings>("load_settings", true)->values.push_back(path);
    };
    std::string path;
    if (!find_named_preset(bundle.printers, o.printer_preset, "printer", /*allow_default=*/false, app, code, error))
        return false;
    if (!write_named_preset_file(dir, *bundle.printers.find_preset(o.printer_preset, false), "machine", "machine",
                                 path, code, error))
        return false;
    add_settings_file(path);
    // The process is named only when it is given: with none the merge takes
    // the one the desktop selects for the new printer, or keeps the project's
    // while that printer is one it suits.
    if (!o.process_preset.empty()) {
        if (!find_named_preset(bundle.prints, o.process_preset, "process", /*allow_default=*/false, app, code, error))
            return false;
        if (!write_named_preset_file(dir, *bundle.prints.find_preset(o.process_preset, false), "process", "process",
                                     path, code, error))
            return false;
        add_settings_file(path);
    }
    for (size_t i = 0; i < o.filament_presets.size(); ++i) {
        const std::string& name = o.filament_presets[i];
        if (!find_named_preset(bundle.filaments, name, "filament", /*allow_default=*/false, app, code, error))
            return false;
        if (!write_named_preset_file(dir, *bundle.filaments.find_preset(name, false), "filament",
                                     "filament" + std::to_string(i + 1), path, code, error))
            return false;
        o.cli.option<Slic3r::ConfigOptionStrings>("load_filaments", true)->values.push_back(path);
    }
    o.preset_switch = true;
    emit_event({{"event","presets_resolved"},
                {"tag","ProjectPrinterSwitch"},
                {"printer", o.printer_preset},
                {"process", o.process_preset},
                {"filaments", o.filament_presets},
                {"message","The project moves to printer '" + o.printer_preset +
                           "' by name, as the desktop app's printer list does; its presets are read from this "
                           "engine's own profiles, every parent applied"}});
    return true;
}

/// The plate type the desktop app selects when the printer preset
/// `printer_name` is picked, for a run whose printer comes from settings files
/// rather than from --printer-preset: the same rule the named-preset path
/// applies (desktop_bed_type). False when this engine ships no printer preset
/// of that name, and nothing is then changed.
bool desktop_plate_type_for_printer(const CliOptions& o, const std::string& printer_name,
                                           Slic3r::BedType& out, std::string& why) {
    if (printer_name.empty()) return false;
    namespace fs = boost::filesystem;
    struct Cleanup { fs::path dir; ~Cleanup() { remove_owned_temp_dir(dir); } } cleanup;
    Slic3r::PresetBundle bundle;
    std::string ignored;
    try {
        const fs::path staging = make_owned_temp_dir("slicer_cli_plate-");
        cleanup.dir = staging;
        if (!load_system_presets(o.argv0, bundle, staging, ignored)) return false;
    } catch (const std::exception&) {
        return false;
    }
    Slic3r::Preset* preset = bundle.printers.find_preset(printer_name, false);
    if (preset == nullptr || preset->name != printer_name || !is_listed_preset(*preset)) return false;
    preset->is_visible = true;
    if (!bundle.printers.select_preset_by_name(printer_name, true)) return false;
    if (bundle.printers.get_edited_preset().name != printer_name) return false;
    const slicer_cli::DesktopBedType bed = slicer_cli::desktop_bed_type(bundle);
    out = bed.type;
    why = bed.why;
    return true;
}

bool resolve_named_presets(CliRunContext& context, const CliOptions& o, Slic3r::DynamicPrintConfig& out,
                                  int& code, std::string& error) {
    namespace fs = boost::filesystem;
    // The temp folder lookup throws on a bad TMPDIR/TMP; it sits inside the
    // try so that is a refusal with a sentence, never an uncaught exception.
    struct Cleanup { fs::path dir; ~Cleanup() { remove_owned_temp_dir(dir); } } cleanup;
    Slic3r::PresetBundle bundle;
    try {
        const fs::path staging = make_owned_temp_dir("slicer_cli_named-");
        cleanup.dir = staging;
        if (!load_system_presets(o.argv0, bundle, staging, error)) { code = CLI_ENVIRONMENT_ERROR; return false; }
    } catch (const std::exception& e) {
        error = std::string("Loading the system presets failed: ") + e.what();
        code = CLI_ENVIRONMENT_ERROR;
        return false;
    }
#ifdef ENGINE_ORCA
    // main's options object is not const; the names are rewritten once here,
    // after the presets that know the renames have loaded.
    {
        CliOptions& named = const_cast<CliOptions&>(o);
        slicer_cli::apply_preset_renames(named.printer_preset, named.process_preset, named.filament_presets, bundle);
    }
    const std::string app = "OrcaSlicer";
#else
    const std::string app = "BambuStudio";
#endif
    auto find = [&](Slic3r::PresetCollection& collection, const std::string& name, const char* kind) -> bool {
        // The built-in default process is taken only as the desktop's own
        // fallback, never by name on the command line.
        const bool desktop_fallback = kind == std::string("process") && o.process_preset.empty();
        if (find_named_preset(collection, name, kind, desktop_fallback, app, code, error))
            return true;
        // A printer preset the other engine of this package ships is refused
        // naming that binary, as the 3MF engine-fit refusal does
        // (engine_mismatch_sentence): the name exists, this binary has no such
        // printer, and the sibling does.
        if (kind == std::string("printer")) {
            const boost::filesystem::path exe_dir = engine_executable_dir(o.argv0.c_str());
#ifdef ENGINE_SNAPMAKER
            const boost::filesystem::path other_dir = orca_profiles_dir(exe_dir);
            const std::string other_app = "OrcaSlicer", other_binary = "slicer_cli-orcaslicer";
#elif defined(ENGINE_ORCA)
            const boost::filesystem::path other_dir = engine_profiles_dir(exe_dir);
            const std::string other_app = "BambuStudio", other_binary = "slicer_cli";
#else
            const boost::filesystem::path other_dir = orca_profiles_dir(exe_dir);
            const std::string other_app = "OrcaSlicer", other_binary = "slicer_cli-orcaslicer";
#endif
            if (!preset_printer_model(other_dir, name).empty())
                error += " This printer is in " + other_binary + " (" + other_app + "); use that binary.";
        }
        return false;
    };

    if (o.printer_preset.empty()) {
        error = "--process-preset and --filament-preset need --printer-preset.";
        code = CLI_INVALID_PARAMS;
        return false;
    }
    if (!find(bundle.printers, o.printer_preset, "printer")) return false;
    bundle.printers.select_preset_by_name(o.printer_preset, true);
    if (bundle.printers.get_edited_preset().name != o.printer_preset) {
        error = "Printer preset '" + o.printer_preset + "' could not be selected.";
        code = CLI_CONFIG_FILE_ERROR;
        return false;
    }
    // Compatibility flags against this printer, keeping the selections.
    bundle.update_compatible(Slic3r::PresetSelectCompatibleType::Never);
    const Slic3r::DynamicPrintConfig& printer = bundle.printers.get_edited_preset().config;

    // An omitted process or filament takes what the desktop selects when
    // the printer is picked: its default when this engine ships it, else
    // the desktop's own fallback (desktop_default_process/_filament).
    std::vector<std::string> substituted_defaults;
    std::string process = o.process_preset;
    if (process.empty()) {
        const std::string declared = printer.has("default_print_profile") ? printer.opt_string("default_print_profile") : std::string();
        process = desktop_default_process(bundle.prints, declared);
        const std::string note = default_replaced_note("process", process, declared);
        if (!note.empty()) substituted_defaults.push_back(note);
    }
    if (process.empty()) {
        error = "Printer preset '" + o.printer_preset + "' has no compatible process; give --process-preset.";
        code = CLI_INVALID_PARAMS;
        return false;
    }
    if (!find(bundle.prints, process, "process")) return false;
    if (!bundle.prints.find_preset(process, false)->is_compatible) {
        error = "Process preset '" + process + "' is not for printer '" + o.printer_preset +
                "'. See --list-presets --printer \"" + o.printer_preset + "\".";
        code = CLI_PROCESS_NOT_COMPATIBLE;
        return false;
    }
    bundle.prints.select_preset_by_name(process, true);
    if (bundle.prints.get_edited_preset().name != process) {
        error = "Process preset '" + process + "' could not be selected.";
        code = CLI_CONFIG_FILE_ERROR;
        return false;
    }

    std::vector<std::string> filaments = o.filament_presets;
    if (filaments.empty()) {
        std::vector<std::string> declared;
        if (const auto* d = printer.option<Slic3r::ConfigOptionStrings>("default_filament_profile"))
            declared = d->values;
        const std::string wanted = desktop_default_filament(bundle.filaments, declared);
        const std::string note = default_replaced_note("filament", wanted,
                                                       declared.empty() ? std::string() : declared.front());
        if (!note.empty()) substituted_defaults.push_back(note);
        if (!wanted.empty()) filaments.push_back(wanted);
    }
    if (filaments.empty()) {
        error = "Printer preset '" + o.printer_preset + "' has no compatible filament; give --filament-preset.";
        code = CLI_INVALID_PARAMS;
        return false;
    }
    for (const std::string& f : filaments) {
        if (!find(bundle.filaments, f, "filament")) return false;
        if (!bundle.filaments.find_preset(f, false)->is_compatible) {
            error = "Filament preset '" + f + "' is not for printer '" + o.printer_preset +
                    "'. See --list-presets --printer \"" + o.printer_preset + "\".";
            code = CLI_PROCESS_NOT_COMPATIBLE;
            return false;
        }
    }
    bundle.filaments.select_preset_by_name(filaments.front(), true);
    if (bundle.filaments.get_edited_preset().name != filaments.front()) {
        error = "Filament preset '" + filaments.front() + "' could not be selected.";
        code = CLI_CONFIG_FILE_ERROR;
        return false;
    }
    // The desktop adds each filament slot with PresetBundle::set_num_filaments
    // (Sidebar::add_custom_filament: BambuStudio Plater.cpp 4198, PresetBundle.cpp
    // 2343-2393; OrcaSlicer Plater.cpp 3243, PresetBundle.cpp 2993-3027),
    // which sizes the project's per-filament lists (filament_colour,
    // filament_multi_colour, filament_colour_type, filament_map, and on
    // BambuStudio the nozzle and volume maps) to the slot count, each new slot
    // taking the last slot's value. No colour is picked here, as no one picks
    // one on the command line.
    bundle.set_num_filaments(unsigned(filaments.size()), std::string());
    bundle.filament_presets = filaments;
    // The desktop app runs this whenever the filament list changes: it sizes
    // the project's flush_volumes_matrix to filaments x filaments per nozzle
    // and flush_multiplier to the nozzle count (PresetBundle.cpp at both
    // pins). Without it a two-nozzle printer (X2D, H2D) reads past the
    // matrix in ToolOrdering::reorder_extruders_for_minimum_flush_volume.
    bundle.update_multi_material_filament_presets();
#ifdef ENGINE_ORCA
    // OrcaSlicer's update_multi_material_filament_presets gives a printer with
    // more extruders than filaments one filament per extruder (PresetBundle.cpp
    // 5120-5131), single_extruder_multi_material or not, and leaves the colour
    // lists as they were. The desktop never keeps them apart: it sizes
    // filament_colour, filament_multi_colour and filament_colour_type to the
    // filament count when it loads the printer (PresetBundle::load_selections
    // 2780-2795, update_selections 2636-2651, new slots "#26A69A" and type
    // "1"), and the official CLI refuses a project where they differ (-5,
    // "filament_is_support's count ... not equal to filament_colour's size").
    // A slice with them apart reads past the shorter lists. BambuStudio's
    // update_multi_material_filament_presets adds no filament (PresetBundle.cpp
    // 5527-5538), so its lists already match.
    // set_num_filaments sizes the lists but gives the new slots no colour of
    // their own here (the filament count has already grown, so its
    // new-colour branch is skipped, and the lists fill from the first slot);
    // the new slots then take load_selections' "#26A69A" and type "1".
    if (const auto* colours = bundle.project_config.option<Slic3r::ConfigOptionStrings>("filament_colour");
        colours && colours->values.size() != bundle.filament_presets.size()) {
        const size_t old_count = colours->values.size();
        const size_t count = bundle.filament_presets.size();
        bundle.set_num_filaments(unsigned(count), std::string());
        auto& colour      = bundle.project_config.option<Slic3r::ConfigOptionStrings>("filament_colour")->values;
        auto& multi       = bundle.project_config.option<Slic3r::ConfigOptionStrings>("filament_multi_colour")->values;
        auto& colour_type = bundle.project_config.option<Slic3r::ConfigOptionStrings>("filament_colour_type")->values;
        for (size_t i = old_count; i < count; ++i) {
            if (i < colour.size()) colour[i] = "#26A69A";
            if (i < multi.size()) multi[i] = "#26A69A";
            if (i < colour_type.size()) colour_type[i] = "1";
        }
    }
#endif
    filaments = bundle.filament_presets;

    // What the desktop app sets in the project when the printer is picked:
    // the plate type (desktop_bed_type) and, on the BambuStudio build, the
    // nozzle statistics. Both are project settings full_config() applies
    // (construct_full_config: BambuStudio PresetBundle.cpp 97, OrcaSlicer 82).
    const slicer_cli::DesktopBedType bed = slicer_cli::desktop_bed_type(bundle);
    bundle.project_config.set_key_value("curr_bed_type", new Slic3r::ConfigOptionEnum<Slic3r::BedType>(bed.type));
    context.g_preset_printer_model_id = bundle.printers.get_edited_preset().get_printer_type(&bundle);
    slicer_cli::apply_printer_pick(bundle);
#ifdef ENGINE_ORCA
    // The settings the desktop slices with: full_config(false), every
    // filament's own values for each extruder variant with
    // filament_self_index (OrcaSlicer Plater.cpp 7959-7963), which the Print
    // maps onto the extruders. full_config(true) maps them already and
    // writes no filament_self_index (PresetBundle.cpp 4005-4060 at 31f6803),
    // so on a printer with several extruders the Print mapped every
    // filament to filament 1's values (H2D, 4 filaments: nozzle_temperature
    // 220,220,220,220 for 220,220,245,270). The official CLI's own merge of
    // --load-filaments files writes some of these lists differently (one
    // value, or the first filament's value followed by 0s); named presets
    // follow the desktop's full_fff_config, one value per filament from
    // each filament's own preset.
#ifdef ENGINE_SNAPMAKER
    // Item 3b: on a switch to a printer that is not a Snapmaker U1 the desktop
    // trims the filament count to the printer's extruder count, while a U1
    // keeps the count it had (references/SnapmakerOrca/src/slic3r/GUI/Tab.cpp:2247-2270
    // at bac6fd5; set_num_filaments is its own call, Tab.cpp:2270).
    if (!o.printer_preset.empty() && o.printer_preset.find("Snapmaker U1") == std::string::npos &&
        slicer_cli::trim_named_filaments_for_switch(bundle))
        context.g_filament_count_changed = true;
    // The remap the switch built, applied to the volumes once the model is loaded.
    context.g_filament_id_remap = bundle.consume_last_filament_id_remap();
#endif
#ifdef ENGINE_SNAPMAKER
    // Snapmaker Orca's PresetBundle::full_config() takes no argument and is
    // full_fff_config() for FFF (references/SnapmakerOrca/src/libslic3r/PresetBundle.cpp:2491-2496
    // at bac6fd5): the same settings OrcaSlicer's full_config(false) returns.
    out = bundle.full_config();
    // The filament total the switch's remap maps into.
    context.g_filament_id_total = slicer_cli::snapmaker_filament_id_total(out);
#else
    out = bundle.full_config(false);
#endif
#else
    // BambuStudio's full_config(true) keeps each filament's own values and
    // its filament_self_index (PresetBundle.cpp 3400-3460 at 5873b5f), and
    // the Print maps them as for full_config(false); false would also write
    // the AMS drying lists per variant, unlike a desktop project.
    out = bundle.full_config();
#endif
#ifdef ENGINE_SNAPMAKER
    // The desktop's rules for the filaments and process picked by name: the
    // volumetric-speed floor and the process's mixed and dithering keys.
    slicer_cli::apply_named_preset_rules(out, bundle);
#endif
    // full_config() writes each filament's own colour; a slot without one
    // keeps the engine default.
    emit_event({{"event","presets_resolved"},
                {"tag","NamedPresetsResolved"},
                {"printer", o.printer_preset},
                {"process", process},
                {"filaments", filaments},
                {"curr_bed_type", slicer_cli::bed_type_name(bed.type)},
                {"curr_bed_type_reason", bed.why},
                {"defaults_replaced", substituted_defaults},
                {"message","Settings built from " + app + " system presets with every parent applied" +
                           (substituted_defaults.empty() ? std::string() : "; used " + boost::algorithm::join(substituted_defaults, ", "))}});
    return true;
}

/// --list-presets [--printer NAME]: the system presets of this engine, as one
/// JSON document. With a printer: the processes and filaments it can use,
/// and its defaults.
int run_list_presets(const CliOptions& o, const std::string& printer_name) {
    namespace fs = boost::filesystem;
    struct Cleanup { fs::path dir; ~Cleanup() { remove_owned_temp_dir(dir); } } cleanup;
    Slic3r::PresetBundle bundle;
    std::string error;
    json out;
#ifdef ENGINE_SNAPMAKER
    out["engine"] = "snapmaker";
#elif defined(ENGINE_ORCA)
    out["engine"] = "orcaslicer";
#else
    out["engine"] = "bambustudio";
#endif
    try {
        // Inside the try: the temp folder lookup throws on a bad TMPDIR/TMP.
        const fs::path staging = make_owned_temp_dir("slicer_cli_list-");
        cleanup.dir = staging;
        if (!load_system_presets(o.argv0, bundle, staging, error, /*document_mode=*/true)) {
            out["error"] = error;
            std::cout << out.dump(2) << std::endl;
            return CLI_ENVIRONMENT_ERROR;
        }
    } catch (const std::exception& e) {
        out["error"] = std::string("Loading the system presets failed: ") + e.what();
        std::cout << out.dump(2) << std::endl;
        return CLI_ENVIRONMENT_ERROR;
    }
    if (printer_name.empty()) {
        json printers = json::array();
        for (const Slic3r::Preset& p : bundle.printers)
            if (is_listed_preset(p))
                printers.push_back(json{{"name", p.name},
                                        {"printer_model", p.config.has("printer_model") ? p.config.opt_string("printer_model") : std::string()}});
        out["printers"] = printers;
    } else {
#ifdef ENGINE_ORCA
        // An old printer name lists its current preset (apply_preset_renames);
        // the name is main's own argument, so it is rewritten in place.
        if (const std::string old = slicer_cli::rename_to_current(bundle.printers, const_cast<std::string&>(printer_name),
                                                      "printer", /*announce=*/false);
            !old.empty())
            out["renamed_from"] = old;
#endif
        const Slic3r::Preset* printer = bundle.printers.find_preset(printer_name, false);
        if (!printer || printer->name != printer_name || !is_listed_preset(*printer)) {
            out["error"] = "No printer preset named '" + printer_name + "'.";
            const std::string similar = near_preset_names(bundle.printers, printer_name);
            if (!similar.empty()) out["close_names"] = similar;
            std::cout << out.dump(2) << std::endl;
            return CLI_CONFIG_FILE_ERROR;
        }
        const_cast<Slic3r::Preset*>(printer)->is_visible = true;   // see resolve_named_presets
        bundle.printers.select_preset_by_name(printer_name, true);
        bundle.update_compatible(Slic3r::PresetSelectCompatibleType::Never);
        const Slic3r::DynamicPrintConfig& cfg = bundle.printers.get_edited_preset().config;
        out["printer"] = printer_name;
        // The process and filament a slice with only --printer-preset takes:
        // the desktop's pick (desktop_default_process/_filament), so a listed
        // default is always one --process-preset/--filament-preset accepts.
        json defaults_replaced = json::array();
        const std::string declared_process = cfg.has("default_print_profile") ? cfg.opt_string("default_print_profile") : std::string();
        const std::string process = desktop_default_process(bundle.prints, declared_process);
        out["default_process"] = process;
        if (const std::string note = default_replaced_note("process", process, declared_process); !note.empty())
            defaults_replaced.push_back(note);
        std::vector<std::string> declared_filaments;
        if (const auto* d = cfg.option<Slic3r::ConfigOptionStrings>("default_filament_profile"))
            declared_filaments = d->values;
        const std::string filament = desktop_default_filament(bundle.filaments, declared_filaments);
        json default_filaments = json::array();
        if (!filament.empty()) default_filaments.push_back(filament);
        out["default_filaments"] = default_filaments;
        if (const std::string note = default_replaced_note("filament", filament,
                declared_filaments.empty() ? std::string() : declared_filaments.front()); !note.empty())
            defaults_replaced.push_back(note);
        out["defaults_replaced"] = defaults_replaced;
        json processes = json::array(), filaments = json::array();
        for (const Slic3r::Preset& p : bundle.prints)
            if (is_listed_preset(p) && p.is_compatible) processes.push_back(p.name);
        for (const Slic3r::Preset& p : bundle.filaments)
            if (is_listed_preset(p) && p.is_compatible) filaments.push_back(p.name);
        out["processes"] = processes;
        out["filaments"] = filaments;
        // The plate type a slice with this printer takes (desktop_bed_type).
        const slicer_cli::DesktopBedType bed = slicer_cli::desktop_bed_type(bundle);
        out["default_bed_type"] = slicer_cli::bed_type_name(bed.type);
    }
    std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
    return 0;
}

} // namespace slicer_cli
