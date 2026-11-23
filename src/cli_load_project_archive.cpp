#include "cli_load_project_archive.hpp"
#include "cli_events.hpp"
#include "cli_plate_arrange.hpp"
#include "cli_load_slice_inputs.hpp"
#include "cli_run_context.hpp"
#include "cli_slice_state.hpp"
#include "cli_options.hpp"
#include "cli_engine_events.hpp"
#include "cli_json_config.hpp"
#include "cli_filament_config.hpp"
#include "cli_load_settings.hpp"
#include "cli_named_presets.hpp"
#include "cli_percent_widths.hpp"
#include "cli_plate_plan.hpp"
#include "cli_preset_staging.hpp"
#include "cli_printer_catalog.hpp"
#include "cli_project_archive.hpp"
#include "cli_project_load.hpp"
#include "cli_project_settings.hpp"
#include "cli_project_values.hpp"
#include "cli_result_json.hpp"
#include "cli_run_steps.hpp"
#include "cli_snapmaker.hpp"
#include "desktop_presets.hpp"
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r_version.h"
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>
#include <iostream>

namespace slicer_cli {

using json = nlohmann::json;

struct ProjectArchiveLoad {
    Slic3r::ConfigSubstitutionContext config_subst{Slic3r::ForwardCompatibilitySubstitutionRule::Enable};
    std::vector<Slic3r::Preset*> presets;
    Slic3r::Semver file_version;
#ifdef ENGINE_ORCA
    bool is_orca_3mf = false;
#endif

    ~ProjectArchiveLoad() {
        for (Slic3r::Preset* p : presets)
            delete p;
        presets.clear();
    }
};

static int read_3mf_archive(const CliOptions& o, PlateOutcome& outcome, int load_plate_id,
            Slic3r::DynamicPrintConfig& config, ProjectArchiveLoad& archive,
            const std::string& load_path, LoadedProject& loaded_project) {
    const auto& input_file = o.input_file;
    auto& model = loaded_project.model;
    auto& is_bbl_3mf = loaded_project.is_bbl_3mf;
    auto& plate_data = loaded_project.plate_data;

    // Pass &config so load_bbs_3mf extracts project_settings.config from the
    // 3MF archive and applies it via config.load_from_json().  Previously this
    // was nullptr which caused load_bbs_3mf to return false immediately
    // (bbs_3mf.cpp:8966 checks config == nullptr).
    //
    // LoadStrategy must include LoadModel (parse geometry) and LoadConfig
    // (parse Metadata/project_settings.config).  AddDefaultInstances ensures
    // each object gets at least one instance.
    auto strategy = Slic3r::LoadStrategy::LoadModel
                  | Slic3r::LoadStrategy::LoadConfig
                  | Slic3r::LoadStrategy::AddDefaultInstances;
#ifdef ENGINE_ORCA
#ifdef ENGINE_SNAPMAKER
    // Snapmaker's load_bbs_3mf has no is_orca_3mf flag (Format/bbs_3mf.hpp
    // at bac6fd5), so this uses the BambuStudio-shaped call.
    bool result = Slic3r::load_bbs_3mf(
        load_path.c_str(),
        &config,
        &archive.config_subst,
        &model,
        &plate_data,
        &archive.presets,
        &is_bbl_3mf,
        &archive.file_version,
        nullptr,   // proFn (progress callback)
        strategy,
        nullptr,   // BBLProject
        load_plate_id   // 0 = all plates, >0 = specific plate
    );
#else
    // OrcaSlicer's load_bbs_3mf inserts a bool* is_orca_3mf between is_bbl_3mf
    // and file_version (and drops the two trailing Bambu-only params).
    bool result = Slic3r::load_bbs_3mf(
        load_path.c_str(),
        &config,
        &archive.config_subst,
        &model,
        &plate_data,
        &archive.presets,
        &is_bbl_3mf,
        &archive.is_orca_3mf,
        &archive.file_version,
        nullptr,   // proFn (progress callback)
        strategy,
        nullptr,   // BBLProject
        load_plate_id   // 0 = all plates, >0 = specific plate
    );
#endif // ENGINE_SNAPMAKER
#else
    bool result = Slic3r::load_bbs_3mf(
        load_path.c_str(),
        &config,
        &archive.config_subst,
        &model,
        &plate_data,
        &archive.presets,
        &is_bbl_3mf,
        &archive.file_version,
        nullptr,   // proFn (progress callback)
        strategy,
        nullptr,   // BBLProject
        load_plate_id   // 0 = all plates, >0 = specific plate
    );
#endif
    // Surface what the engine silently changed or ignored while
    // deserializing Metadata/project_settings.config. This is the
    // channel the GUI renders as "incompatible settings were
    // substituted", and it is also where an unknown key (a setting the
    // host wrote that this engine has no definition for) is recorded.
#ifdef ENGINE_SNAPMAKER
    slicer_cli::convert_flow_type_spelling(config, archive.config_subst, "3mf:project_settings.config");
#endif
    emit_config_substitutions(archive.config_subst, "3mf:project_settings.config");
    outcome.unknown_settings = archive.config_subst.unrecogized_keys;   // sic — upstream spelling
#ifdef ENGINE_SNAPMAKER
    if (result)
        slicer_cli::warn_newer_project_schema(config);
#endif

    if (!result) {
        emit_event({{"event","load_error"},
                    {"tag","ThreeMfLoadFailed"},
                    {"path", input_file},
                    {"message","Failed to load 3MF file"}});
        std::cerr << "Failed to load 3MF file\n";
        set_outcome_failure(outcome, CLI_DATA_FILE_ERROR);
        return 1;
    }
    // --normative-check, on by default in the official CLI for a
    // Bambu-made 3MF (BambuStudio.cpp 1948-1965; OrcaSlicer.cpp
    // 1609-1619). Part of --slice here, or asked for by name, so the
    // default call slices the same files it always has.
    if (is_bbl_3mf && (o.slice_mode || o.given_flag("normative_check"))) {
        const slicer_cli::StepResult nc = slicer_cli::normative_check(o, config);
        if (nc.code != 0) {
            emit_event({{"event","config_refused"}, {"tag","NormativeCheckFailed"}, {"message", nc.message}});
            std::cerr << "Error: " << nc.message << "\n";
            set_outcome_failure(outcome, nc.code, nc.message);
            return 1;
        }
    }
    return 0;
}

static void load_desktop_project_presets(const CliOptions& o, LoadedProject& loaded_project,
            PlateOutcome& outcome, Slic3r::DynamicPrintConfig& config, const Slic3r::Semver& file_version,
            std::vector<Slic3r::Preset*>& presets, bool& project_file_config_kept,
            Slic3r::DynamicPrintConfig& project_file_config, bool bambu_made_project) {
    const auto& input_file = o.input_file;

    if (bambu_made_project && !o.project_settings_as_saved) {
        // The settings as the file states them, for read_project_facts
        // below: the facts describe the file (its printer, bed, plate
        // membership and the maker's own changed-key list), not the
        // preset-rebased settings this run slices with.
        project_file_config      = config;
        project_file_config_kept = true;
        Slic3r::DynamicPrintConfig rebased;
        std::string                unavailable;
        bool                       rebase = true;
        try {
            // The profiles tree staged as <data_dir>/system, the same
            // staging the named-preset path uses (load_system_presets):
            // the bundle reads system presets from there and writes
            // nothing back (load_external_preset saves no file).
            const boost::filesystem::path staging = make_owned_temp_dir("slicer_cli_project-");
            struct StagingCleanup {
                boost::filesystem::path dir;
                ~StagingCleanup() { remove_owned_temp_dir(dir); }
            } staging_cleanup;
            staging_cleanup.dir = staging;
            Slic3r::PresetBundle bundle;
            if (!load_system_presets(o.argv0, bundle, staging, unavailable))
                rebase = false;
            else {
                // The names the FILE states, not the ones the config
                // carries: this engine's own 3MF loader substitutes a
                // preset name it does not know with its default before
                // the front end sees the config, and a project whose
                // preset is not shipped is the one the rebase must skip
                // (loading it over the default is a different print, and
                // a project whose process preset the bundle cannot
                // resolve is loaded as an external preset that
                // Print::apply then aborts on).
                const json  file_settings  = project_settings_json(input_file);
                std::string file_printer   = json_scalar_text(file_settings, "printer_settings_id");
                std::string file_process   = json_scalar_text(file_settings, "print_settings_id");
                if (file_printer.empty())
                    file_printer = config_id_text(config, "printer_settings_id");
                if (file_process.empty())
                    file_process = config_id_text(config, "print_settings_id");
#ifdef ENGINE_BAMBU
                // The presets saved inside the project and the file's
                // own nozzle rack, as the desktop keeps them
                // (desktop_presets.cpp).
                const Slic3r::DynamicPrintConfig file_nozzle_keys =
                    slicer_cli::load_project_embedded_presets(bundle, presets, config);
#endif
                if (!project_presets_shipped(bundle, file_printer, file_process)) {
                    unavailable = "this engine ships no '" + file_printer + "' / '" + file_process +
                                  "' system preset";
                    rebase = false;
                } else {
                // The tower's plate positions, as the desktop keeps
                // them across the load (Plater.cpp 6380-6389).
                Slic3r::ConfigOptionFloats file_wipe_tower_x, file_wipe_tower_y;
                bool                       has_wipe_tower_x = false, has_wipe_tower_y = false;
                if (const auto* opt = config.option<Slic3r::ConfigOptionFloats>("wipe_tower_x")) {
                    file_wipe_tower_x = *opt;
                    has_wipe_tower_x  = true;
                }
                if (const auto* opt = config.option<Slic3r::ConfigOptionFloats>("wipe_tower_y")) {
                    file_wipe_tower_y = *opt;
                    has_wipe_tower_y  = true;
                }
                // load_config_model takes the config by value and
                // consumes it: the bundle keeps what it parsed. Handed
                // the caller's own config, an exception part-way (a
                // preset this engine cannot resolve) would leave the
                // flat path with a moved-from config, and Print::apply
                // then reads settings that are no longer there. So the
                // load gets a copy, and `config` is replaced only when
                // the rebase itself succeeded.
                Slic3r::DynamicPrintConfig file_config_for_bundle = config;
                bundle.load_config_model(input_file, std::move(file_config_for_bundle), file_version);
#ifdef ENGINE_BAMBU
                // The file's nozzles win over the selected printer's
                // (NozzleInventory::open_project, Plater.cpp 12160).
                bundle.nozzle_inventory.open_project(bundle.printers, file_nozzle_keys);
#endif
#ifdef ENGINE_SNAPMAKER
                rebased = bundle.full_config();   // see above: full_fff_config at bac6fd5
#else
                rebased = bundle.full_config(false);
#endif
#ifdef ENGINE_BAMBU
                // extruder_ams_count: a newly started app takes the
                // file's (PresetBundle.cpp 5362-5366: the bundle's own
                // counts are empty then), and so does this run.
#endif
                // A project whose preset name this bundle cannot
                // resolve is loaded as an external preset built from the
                // file, and the config that comes back can describe a
                // different filament roster than the file does;
                // Print::apply then aborts on the mismatch (it derives
                // its extruder count from filament_diameter,
                // PrintApply.cpp:1504). The rebase is kept only when it
                // describes the roster the file states.
                const size_t file_filaments    = project_filament_count(config);
                const size_t rebased_filaments = project_filament_count(rebased);
                if (file_filaments != 0 && rebased_filaments != file_filaments) {
                    unavailable = "the system presets the project names describe " +
                                  std::to_string(rebased_filaments) + " filament(s) where the file states " +
                                  std::to_string(file_filaments);
                    rebase = false;
                }
                // The same for the extruders: a file that states
                // another number of extruders than the printer preset
                // it names is not that preset's project, and its
                // settings are sliced as the file states them.
                const auto extruder_count = [](const Slic3r::DynamicPrintConfig& c) {
                    const auto* d = c.option<Slic3r::ConfigOptionFloats>("nozzle_diameter");
                    return d ? d->values.size() : size_t(0);
                };
                if (rebase && extruder_count(config) != 0 && extruder_count(rebased) != extruder_count(config)) {
                    unavailable = "the printer preset the project names has " +
                                  std::to_string(extruder_count(rebased)) + " extruder(s) where the file states " +
                                  std::to_string(extruder_count(config));
                    rebase = false;
                }
                if (has_wipe_tower_x)
                    rebased.set_key_value("wipe_tower_x", new Slic3r::ConfigOptionFloats(file_wipe_tower_x));
                if (has_wipe_tower_y)
                    rebased.set_key_value("wipe_tower_y", new Slic3r::ConfigOptionFloats(file_wipe_tower_y));
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
                // The nozzle rack the project's printer had (extruder_nozzle_stats).
                // It is no preset setting: the app writes it into the printer
                // it has open after a device sync (MultiNozzleSync.cpp 234 at
                // f92a7c0), and OrcaSlicer's command line reads it from the
                // project for "Nozzle Manual" (OrcaSlicer.cpp 6865;
                // ToolOrdering.cpp 1822-1838). No system preset states it, so
                // the file's value is kept across this load.
                if (const auto* stats = config.option<Slic3r::ConfigOptionStrings>("extruder_nozzle_stats");
                    stats != nullptr && !stats->values.empty())
                    rebased.set_key_value("extruder_nozzle_stats", stats->clone());
#endif
                }
            }
        } catch (const std::exception& e) {
            unavailable = e.what();
            rebase      = false;
            // A filament setting the preset split cannot read. The
            // desktop calls the same load_config_model (Plater.cpp 6391
            // at 31f6803) inside a catch that shows the error and skips
            // the file (6639-6642), so the project is not widened here:
            // the reason is named, and any refusal of the plate says
            // which key it is.
#ifdef ENGINE_ORCA
            const auto empty = filament_keys_split_reads_empty(config, project_settings_json(input_file));
            if (!empty.empty()) {
                std::string names;
                bool        single = true;
                for (const auto& key : empty) {
                    names += (names.empty() ? "" : ", ") + key.first;
                    single = single && key.second;
                }
                const bool  one  = empty.size() == 1;
                const char* what = single ? (one ? " is a single value" : " are single values")
                                          : (one ? " has no entries" : " have no entries");
                unavailable += ": the file's " + names + what + " where OrcaSlicer expects one per filament";
                outcome.refusal_note =
                    "This project's " + names + what +
                    " where OrcaSlicer expects one per filament, so its settings could not be matched to the "
                    "desktop presets; slice it with slicer_cli (BambuStudio), or re-save it with one " +
                    names + " entry per filament.";
            }
#endif
        }
        if (rebase) {
            outcome.settings_replaced_by_presets = slicer_cli::settings_replaced_by_presets(config, rebased);
            // Only what the file states: a key it leaves out was
            // never the file's value.
            const json file_settings = project_settings_json(input_file);
            for (const std::string& key : outcome.settings_replaced_by_presets)
                if (file_settings.contains(key)) {
                    auto& values = outcome.replaced_file_values[key];
                    values.push_back(config.opt_serialize(key));
                    if (file_settings[key].is_string())
                        values.push_back(file_settings[key].get<std::string>());
                }
            config = std::move(rebased);
            loaded_project.project_loaded_like_desktop = true;
            emit_event({{"event","config_normalized"},
                        {"tag","ProjectRebasedOnSystemPresets"},
                        {"printer_settings_id", config_id_text(config, "printer_settings_id")},
                        {"print_settings_id", config_id_text(config, "print_settings_id")},
                        {"replaced_settings", outcome.settings_replaced_by_presets},
                        {"message","The project was loaded over this engine's system presets, as the desktop loads it: "
                                   "every key the maker did not change takes the preset's value"}});
        } else {
            // The engine ships no preset to load the project over, so
            // the file's own settings are sliced, as before.
            project_file_config.clear();
            project_file_config_kept = false;
            emit_event({{"event","config_normalized"},
                        {"tag","ProjectPresetRebaseSkipped"},
                        {"reason", unavailable},
                        {"message","The project keeps the values the file states: " + unavailable}});
        }
    }
}

#if defined(ENGINE_ORCA)
static int convert_project_object_values(CliRunContext& context, PlateOutcome& outcome,
            const std::string& input_file, Slic3r::DynamicPrintConfig& config, Slic3r::Model& model,
            Slic3r::ConfigSubstitutionContext& config_subst, Slic3r::DynamicPrintConfig& project_file_config) {
    // ── The file's own words, in this engine's values ───────────────
    // The engine's own legacy pass (PrintConfigDef::handle_legacy,
    // PrintConfig.cpp 7867-8151 at 31f6803) runs per key while a config
    // loads: it cannot see another key's value, and it knows only some
    // of BambuStudio's words. The rest are converted here, on the
    // settings the run slices with, from the words the file states --
    // for the project and for every object's and part's own settings,
    // which the file keeps in Metadata/model_settings.config and which
    // bbs_3mf loads over the same legacy pass (bbs_3mf.cpp 2130, 5115,
    // 5263 at 31f6803). One set of rules (bambu_value_actions) decides
    // for all three scopes.
    {
        const json project_settings = project_settings_json(input_file);
        RawValues  project_raw;
        for (const char* key : {"tree_support_wall_count", "raft_first_layer_expansion", "top_one_wall_type",
                                "ensure_vertical_shell_thickness"})
            project_raw[key] = json_scalar_text(project_settings, key);
        std::vector<BambuValueAction> conversions;
        std::vector<BambuValueAction> warnings;
        std::string                   refusal;
        if (!bambu_value_actions(context, project_raw, config_value_text(config, "support_type"),
                                 maker_changed_key(project_file_config, "raft_first_layer_expansion"),
                                 conversions, warnings, refusal)) {
            emit_event({{"event","config_refused"},
                        {"tag","BambuValueNotCarryable"},
                        {"opt_key", project_raw.count("raft_first_layer_expansion") ? "raft_first_layer_expansion"
                                                                                   : "ensure_vertical_shell_thickness"},
                        {"message", refusal}});
            std::cerr << "Error: " << refusal << "\n";
            set_outcome_failure(outcome, CLI_INVALID_VALUES_IN_3MF, refusal);
            return 1;
        }
        // A word this pass converted is not a substitution: the engine's
        // record of not knowing it would make --slice mode refuse an
        // encoding difference. The loader keeps one substitution context
        // for the whole file, object and part settings included, so the
        // record names the key, not the scope it came from.
        const auto drop_substitution = [&](const char* key) {
            config_subst.substitutions.erase(
                std::remove_if(config_subst.substitutions.begin(), config_subst.substitutions.end(),
                               [key](const Slic3r::ConfigSubstitution& s) {
                                   return s.opt_def != nullptr && s.opt_def->opt_key == key;
                               }),
                config_subst.substitutions.end());
        };
        apply_bambu_actions(
            conversions, warnings,
            [&](const char* key, const std::string& value) {
                Slic3r::ConfigSubstitutionContext context(
                    Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
                config.set_deserialize(key, value, context);
            },
            drop_substitution, "scope", std::string());

        // Each object's and each part's own settings, matched to the
        // loaded model by the names the file states (the file's numeric
        // ids are not kept on the loaded objects). An object's settings
        // are the maker's own words by definition, so the "the maker
        // changed this key" test the project needs does not apply here.
        const std::vector<FileObjectSettings>          file_objects = project_object_settings(input_file);
        std::map<std::string, std::vector<const FileObjectSettings*>> by_name;
        for (const FileObjectSettings& settings : file_objects)
            by_name[settings.name].push_back(&settings);
        std::map<std::string, size_t> taken;
        for (Slic3r::ModelObject* object : model.objects) {
            if (object == nullptr)
                continue;
            const auto& candidates = by_name[object->name];
            const size_t index     = taken[object->name]++;
            if (index >= candidates.size())
                continue;
            const FileObjectSettings& raw_object = *candidates[index];
            const std::string project_support = config_value_text(config, "support_type");
            const std::string object_support  = object->config.option("support_type") != nullptr
                                                    ? object->config.option("support_type")->serialize()
                                                    : project_support;
            auto convert_scope = [&](const RawValues&                            raw,
                                     const std::string&                          support_type,
                                     const std::string&                          scope,
                                     const std::string&                          scope_name,
                                     const std::function<void(const char*, const std::string&)>& write) {
                std::vector<BambuValueAction> scope_conversions;
                std::vector<BambuValueAction> scope_warnings;
                std::string                   scope_refusal;
                if (!bambu_value_actions(context, raw, support_type, /*maker_set_raft=*/true, scope_conversions,
                                         scope_warnings, scope_refusal)) {
                    const std::string sentence = scope_refusal + " (in " + scope + " \"" + scope_name + "\")";
                    emit_event({{"event","config_refused"},
                                {"tag","BambuValueNotCarryable"},
                                {scope, scope_name},
                                {"message", sentence}});
                    std::cerr << "Error: " << sentence << "\n";
                    set_outcome_failure(outcome, CLI_INVALID_VALUES_IN_3MF, sentence);
                    return false;
                }
                apply_bambu_actions(scope_conversions, scope_warnings, write, drop_substitution, scope,
                                    scope_name);
                return true;
            };
            const bool object_ok = convert_scope(
                raw_object.values, object_support, "object", object->name,
                [&](const char* key, const std::string& value) {
                    Slic3r::ConfigSubstitutionContext context(
                        Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
                    object->config.set_deserialize(key, value, context);
                });
            if (!object_ok)
                return 1;
            std::map<std::string, size_t> parts_taken;
            for (Slic3r::ModelVolume* volume : object->volumes) {
                if (volume == nullptr)
                    continue;
                size_t part_index = 0;
                bool   matched    = false;
                for (const auto& part : raw_object.parts) {
                    if (part.first != volume->name)
                        continue;
                    if (part_index++ < parts_taken[volume->name])
                        continue;
                    parts_taken[volume->name]++;
                    const std::string part_support = volume->config.option("support_type") != nullptr
                                                         ? volume->config.option("support_type")->serialize()
                                                         : object_support;
                    if (!convert_scope(part.second, part_support, "part", volume->name,
                                       [&](const char* key, const std::string& value) {
                                           Slic3r::ConfigSubstitutionContext context(
                                               Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
                                           volume->config.set_deserialize(key, value, context);
                                       }))
                        return 1;
                    matched = true;
                    break;
                }
                (void)matched;
            }
        }
    }
    // OrcaSlicer v2.4.2 numbers every feature-filament index from 0,
    // where 0 is "Default" — the active object's or part's own
    // filament (outer_wall_filament_id, PrintConfig.cpp 5011-5018, min
    // 0, default 0) — and both apps' project files now carry those
    // keys by that name: handle_legacy renames the old 1-based
    // wall_filament / sparse_infill_filament / solid_infill_filament
    // to their *_id spelling and maps a legacy "1" to "0"
    // (PrintConfig.cpp 8038-8058). A Bambu Studio file's own 0 is
    // therefore already a value this engine reads, and nothing has to
    // be moved for it. Before the bump this block carried that 0 into
    // this engine's 1-based range; v2.4.2 makes that half redundant.
    //
    // What the desktop still does on load, and this run now does on
    // the settings it slices with, before the value check below: an
    // index outside [0, N] falls back to 0 for the six feature
    // filaments (PresetBundle.cpp 4127-4136), and support_filament /
    // support_interface_filament / wipe_tower_filament clamp to
    // [0, N] (PresetBundle.cpp 4119-4125), with N the filament count.
    // Any other out-of-range value is left as it is and still
    // refuses, exactly as the desktop leaves it (it does not clamp
    // them either, e.g. a BambuStudio -1, its "auto").
    return 0;
}
#endif

#if defined(ENGINE_ORCA)
static void normalize_project_support_filaments(Slic3r::DynamicPrintConfig& config) {
    {
        const size_t filament_count = std::max<size_t>(1, project_filament_count(config));
        json reset = json::object();
        json clamped = json::object();
        for (const char* key : {"outer_wall_filament_id", "inner_wall_filament_id",
                                "sparse_infill_filament_id", "internal_solid_filament_id",
                                "top_surface_filament_id", "bottom_surface_filament_id"}) {
            auto* opt = dynamic_cast<Slic3r::ConfigOptionInt*>(config.option(key, false));
            if (opt == nullptr || (opt->value >= 0 && opt->value <= int(filament_count)))
                continue;
            reset[key] = {{"from", opt->value}, {"to", 0}};
            opt->value = 0;
        }
        for (const char* key : {"support_filament", "support_interface_filament", "wipe_tower_filament"}) {
            auto* opt = dynamic_cast<Slic3r::ConfigOptionInt*>(config.option(key, false));
            if (opt == nullptr)
                continue;
            const int value = std::min(std::max(opt->value, 0), int(filament_count));
            if (value == opt->value)
                continue;
            clamped[key] = {{"from", opt->value}, {"to", value}};
            opt->value = value;
        }
        if (!reset.empty() || !clamped.empty()) {
            std::string names;
            for (const auto& [key, change] : reset.items())
                names += (names.empty() ? "" : ", ") + key + " " + change["from"].dump() + " -> " +
                         change["to"].dump();
            for (const auto& [key, change] : clamped.items())
                names += (names.empty() ? "" : ", ") + key + " " + change["from"].dump() + " -> " +
                         change["to"].dump();
            emit_event({{"event","config_normalized"},
                        {"tag","FilamentIndexOutOfRangeReset"},
                        {"reset", reset},
                        {"clamped", clamped},
                        {"filament_count", int(filament_count)},
                        {"message","The 3mf states filament index value(s) outside this engine's range for its " +
                                   std::to_string(filament_count) + " filament(s): " + names +
                                   " (0 is Default; the desktop resets the same keys on load)"}});
        }
    }
}
#endif

static int check_project_version(const CliOptions& o, PlateOutcome& outcome,
            const Slic3r::DynamicPrintConfig& config, const Slic3r::Semver& file_version) {
    const auto& input_file = o.input_file;

    // Engine fit: a project for a printer this engine does not have is
    // refused before anything is sliced. Without the printer's presets
    // the engine slices the flat file against its own defaults (a
    // 200 x 200 bed for a U1 project on the Bambu build) and reports
    // success. Keyed on printer_model, not on the app that made the
    // file: OrcaSlicer ships Bambu Lab printers too, so a Bambu
    // printer's project is valid on both builds.
    // printer_model as the file states it: a loader that does not take
    // the file for its own app's project (the Bambu loader with an
    // "OrcaSlicer-" Application) leaves the setting at the default.
    std::string file_printer_model;
    {
        std::string text;
        if (read_zip_member(input_file, "Metadata/project_settings.config", text)) {
            try {
                const json settings = json::parse(text);
                if (settings.is_object() && settings.contains("printer_model") &&
                    settings["printer_model"].is_string())
                    file_printer_model = settings["printer_model"].get<std::string>();
            } catch (...) {
            }
        }
        if (file_printer_model.empty() && config.has("printer_model"))
            file_printer_model = config.opt_string("printer_model");
    }
    {
        const std::string& printer_model = file_printer_model;
        if (!printer_model.empty()) {
            const EngineFit fit = engine_fit_for(o.argv0, printer_model);
            if (fit.this_catalog_found && !fit.this_has) {
                const std::string sentence = engine_mismatch_sentence(fit, printer_model);
                json fits = json::array();
                if (fit.other_has) fits.push_back(fit.other_binary);
                emit_event({{"event","engine_mismatch"},
                            {"tag","PrinterModelNotInEngine"},
                            {"printer_model", printer_model},
                            {"engine", fit.this_binary},
                            {"fits", fits},
                            {"message", sentence}});
                std::cerr << "Error: " << sentence << "\n";
                set_outcome_failure(outcome, CLI_3MF_NOT_SUPPORT_MACHINE_CHANGE, sentence);
                return 1;
            }
        }
    }
    // The official version gate (BambuStudio.cpp 1906-1911 at 5873b5f,
    // OrcaSlicer.cpp 1806-1810 at f92a7c0): a file saved by a newer
    // major.minor than this engine is refused unless --allow-newer-file.
    // On the Orca build the maker decides which number the file's
    // version is in (G2, see below): a Bambu Studio project is only
    // warned about, an OrcaSlicer project is refused.
    // The refusal sentence adds what the file says about its maker: a
    // Bambu Studio 02.07 project on the Orca 2.4 build is newer only in
    // the other app's numbering, and the binary to use is the maker's.
    if (!o.allow_newer_file && file_version.maj() + file_version.min() > 0) {
        std::string model_xml;
        std::map<std::string, std::string> meta;
        if (read_zip_member(input_file, "3D/3dmodel.model", model_xml))
            meta = model_metadata(model_xml);
        const auto app = meta.find("Application");
        // G2 (file_version_verdict): a Bambu Studio project on the
        // Orca build is compared with the Bambu base and only warned
        // about -- a BambuStudio 02.05 project is not newer than
        // OrcaSlicer 2.4, it is newer than the base both apps share,
        // and the desktop loads it.
        const FileVersionVerdict verdict = file_version_verdict(file_version, meta);
        if (verdict == FileVersionVerdict::WarnedNewerThanBase)
            emit_event({{"event","warning"},
                        {"tag","FileNewerThanEngineBase"},
                        {"file_version", file_version.to_string()},
                        {"application", app != meta.end() ? json(app->second) : json(nullptr)},
                        {"base_version", SLIC3R_VERSION},
                        {"unknown_keys", outcome.unknown_settings},
                        {"message","The file is version " + file_version.to_string() +
                                   (app != meta.end() ? " (" + app->second + ")" : std::string()) +
                                   ", newer than the " + SLIC3R_VERSION +
                                   " Bambu base this engine is built on. It is sliced as it is; the "
                                   "setting(s) this engine has no definition for are ignored"}});
        if (verdict == FileVersionVerdict::Refused) {
            std::string detail = "The file is version " + file_version.to_string() +
                                 (app != meta.end() ? " (" + app->second + ")" : std::string()) +
                                 "; this engine is " + engine_version_text() + "." +
                                 " Pass --allow-newer-file to slice it anyway; the settings this engine"
                                 " does not know are then ignored.";
            emit_event({{"event","config_refused"},
                        {"tag","FileVersionNewerThanEngine"},
                        {"file_version", file_version.to_string()},
                        {"engine_version", engine_version_text()},
                        {"message", cli_error_sentence(CLI_FILE_VERSION_NOT_SUPPORTED) + " " + detail}});
            std::cerr << "Error: " << cli_error_sentence(CLI_FILE_VERSION_NOT_SUPPORTED) << " " << detail << "\n";
            set_outcome_failure(outcome, CLI_FILE_VERSION_NOT_SUPPORTED, detail);
            return 1;
            }
    }
    // A setting whose value this engine has no meaning for is a
    return 0;
}

static void check_project_substitutions(const CliOptions& o, std::vector<std::string>& unknown_values,
            json& unknown_items, const Slic3r::ConfigSubstitutionContext& config_subst) {
    // different print from the one the file states, so it is refused,
    // naming the setting, the value and the values this engine has.
    // Only enum
    // substitutions: those are the ones where the engine swapped in
    // its own default for a word it does not know. --allow-substitution
    // keeps the old behaviour (slice with the engine's substitute).
    // A project moved from OrcaSlicer to a Bambu printer can still
    // carry Orca words (ensure_all, rectilinear) that the default call
    // has always substituted; the default call keeps doing so, and the
    // refusal is part of the --slice mode only.
    if (o.slice_mode && !o.allow_substitution) {
        std::vector<std::string>& refused = unknown_values;
        json& items = unknown_items;
        for (const auto& sub : config_subst.substitutions) {
            if (!sub.opt_def) continue;
            const auto type = sub.opt_def->type;
            if (type != Slic3r::coEnum && type != Slic3r::coEnums) continue;
            std::string allowed;
            if (sub.opt_def->enum_keys_map)
                for (const auto& [name, value] : *sub.opt_def->enum_keys_map)
                    allowed += (allowed.empty() ? "" : ", ") + name;
            // BambuStudio's "partial" skips one shell pass (BambuStudio
            // PrintObject.cpp 1944, 1975 at 5873b5f) and this engine has
            // no shell pass to skip, so the value is refused with the
            // choice to make rather than the engine's generic sentence.
            // "enabled"/"disabled" are the same meaning in another
            // encoding and are converted by handle_legacy
            // (libslic3r/orcaslicer/libslic3r/PrintConfig.cpp).
            const bool partial_shell =
                sub.opt_def->opt_key == "ensure_vertical_shell_thickness" && sub.old_value == "partial";
            refused.push_back(partial_shell
                ? "'ensure_vertical_shell_thickness' is 'partial', BambuStudio's skip-one-shell-pass mode, "
                  "which this engine has no value for (its values: " + allowed +
                  "); pick the shell coverage you want, e.g. ensure_all for BambuStudio's 'enabled'"
                : "'" + sub.opt_def->opt_key + "' is '" + sub.old_value +
                  "', which this engine does not have (its values: " + allowed + ")");
            items.push_back(json{{"opt_key", sub.opt_def->opt_key}, {"value", sub.old_value},
                                 {"allowed", allowed}});
        }
    }
}

static int read_loaded_plate_layout(LoadedProject& loaded_project, RunPlatePlan& plate_plan,
            PlateOutcome& outcome, bool verbose, int plate_id, Slic3r::DynamicPrintConfig& config,
            const Slic3r::Semver& file_version, const PlateLoadInputs& inputs,
            const std::vector<Slic3r::Preset*>& presets, bool& project_file_config_kept,
            const Slic3r::DynamicPrintConfig& project_file_config) {
    auto& model = loaded_project.model;
    auto& is_bbl_3mf = loaded_project.is_bbl_3mf;
    auto& plate_data = loaded_project.plate_data;
    auto& plate_origins = loaded_project.plate_origins;
    auto& project_facts = loaded_project.project_facts;
    auto& project_presets_kept = loaded_project.project_presets_kept;
    auto& plan_file_config = plate_plan.plan_file_config;
    auto& plan_file_version = plate_plan.plan_file_version;
    auto& plan_file_filaments = plate_plan.plan_file_filaments;

    if (is_bbl_3mf)
        // The file's own settings when the rebase kept them apart: the
        // facts describe the file (its printer, bed, plate membership
        // and the maker's changed-key list), and the printer change
        // reads them to find the system preset the file's process
        // inherits from. full_fff_config erases
        // different_settings_to_system and writes a new inherits_group,
        // so the rebased settings would answer a different question.
        slicer_cli::read_project_facts(project_file_config_kept ? project_file_config : config,
                                       project_facts);
    // is_bbl_3mf ends false once a model file follows the project
    // (BambuStudio.cpp 1871; OrcaSlicer.cpp 1553): the settings steps
    // for a Bambu-made project do not run.
    if (inputs.trailing_port)
        project_facts.is_bbl_3mf = false;
    loaded_project.file_version = file_version;
    for (const Slic3r::Preset* p : presets)
        if (p) project_presets_kept.push_back(*p);
    // Validate --plate against actual plate count
    if (plate_id > 0 && (int)plate_data.size() < plate_id) {
        emit_event({{"event","input_error"},
                    {"tag","PlateOutOfRange"},
                    {"requested_plate", plate_id},
                    {"plate_count", (int)plate_data.size()},
                    {"message","--plate " + std::to_string(plate_id) +
                               " but the 3MF has only " +
                               std::to_string(plate_data.size()) + " plate(s)"}});
        std::cerr << "Error: --plate " << plate_id
                  << " but 3MF only has " << plate_data.size() << " plate(s)\n";
        set_outcome_failure(outcome, CLI_INVALID_PARAMS);
        return 1;
    }

    // ── Multi-plate coordinate translation ───────────────────────
    // A Bambu/Orca 3MF stores every object at its scene position: plate
    // i (0-based) sits at the grid origin the desktop's PartPlateList
    // gives it, and the CLI slices that plate with its print origin set
    // there.  Port of that rule (pins 5873b5f / 31f6803):
    //   - plate size = (int)(printable_area[2] - printable_area[0]) from
    //     the FILE's config (BambuStudio.cpp 2054-2056), plus
    //     bed3d_ax3s_default_tip_radius (2.5 * 0.5 mm) for files older
    //     than 1.5.9 (BambuStudio.cpp 1911-1914, 4316-4318; OrcaSlicer.cpp
    //     3727-3729);
    //   - cols = compute_colum_count(plate count) (PartPlate.hpp:38);
    //   - origin(i) = (col * w * 1.2, -row * d * 1.2) with
    //     LOGICAL_PART_PLATE_GAP = 1/5 (PartPlate.cpp:55, 4343-4355, 5240-5254).
    // The objects stay at their scene places in the one model, and
    // each plate is sliced in its own frame (the plate's instances
    // moved by minus its origin, PlateScope; see the Print's origin at
    // print.apply). The objects stay where the maker put them on their
    // plate: no snap to the bed
    // corner, no plate_N.json bbox guess (both moved parts, and on beds
    // with an exclusion area the corner snap pushed them into it).
    if (is_bbl_3mf)
        for (int index = 0; index < (int)plate_data.size(); ++index)
            plate_origins.push_back(plate_grid_origin(config, file_version, index, (int)plate_data.size()));
    // The plates of the run: the one plate loaded (every instance of
    // the model is on it), or with every plate loaded each plate's
    // instances as the file states them (PlateData::obj_inst_map's
    // identify ids, else the first plate an instance's box meets:
    // PartPlateList::reload_all_objects -> PartPlate::intersect_instance,
    // PartPlate.cpp 5668-5710, 2651-2677).
    if (is_bbl_3mf && plate_id > 0) {
        plate_plan.plates.assign(plate_data.size(), RunPlate());
        for (int index = 0; index < (int)plate_data.size(); ++index) {
            plate_plan.plates[index].origin = plate_origins[index];
            plate_plan.plates[index].all = index == plate_id - 1;
        }
    } else if (is_bbl_3mf && plate_data.size() > 1 && !inputs.trailing_port) {
        plate_plan.plates.assign(plate_data.size(), RunPlate());
        std::set<const Slic3r::ModelInstance*> placed;
        for (int index = 0; index < (int)plate_data.size(); ++index) {
            plate_plan.plates[index].origin = plate_origins[index];
            plate_plan.plates[index].all = false;
            if (plate_data[index] == nullptr) continue;
            std::set<int> identify_ids;
            for (const auto& entry : plate_data[index]->obj_inst_map)
                if (entry.second.second > 0) identify_ids.insert(entry.second.second);
            for (Slic3r::ModelObject* object : model.objects)
                for (Slic3r::ModelInstance* inst : object->instances)
                    if (inst->loaded_id > 0 && identify_ids.count(int(inst->loaded_id)) && placed.insert(inst).second)
                        plate_plan.plates[index].members.push_back(inst);
        }
        for (Slic3r::ModelObject* object : model.objects)
            for (size_t ii = 0; ii < object->instances.size(); ++ii) {
                if (placed.count(object->instances[ii])) continue;
                const Slic3r::BoundingBoxf3 box = object->instance_convex_hull_bounding_box(ii);
                for (int index = 0; index < (int)plate_data.size(); ++index)
                    if (plate_scene_box(config, file_version, index, (int)plate_data.size()).intersects(box)) {
                        plate_plan.plates[index].members.push_back(object->instances[ii]);
                        break;
                    }
            }
    }
    if (inputs.trailing_port || plate_id == 0) {
        plan_file_config = config;
        plan_file_version = file_version;
        if (const auto* ids = config.option<Slic3r::ConfigOptionStrings>("filament_settings_id"))
            plan_file_filaments = (int)ids->values.size();
    }
    // A plate prints by object when its own print_sequence says so,
    // else when the project's does (get_print_sequence,
    // BambuStudio.cpp 4403-4416; the plate's own settings go over the
    // project's per plate, 6902-6904). plate_N.json's is_seq_print is
    // what the desktop's last slice found, an output, not a setting.
    if (plate_id > 0 && is_bbl_3mf && !model.objects.empty()) {
        const Slic3r::Vec2d origin = plate_grid_origin(config, file_version, plate_id - 1, (int)plate_data.size());
        if (verbose)
            std::cout << "Plate " << plate_id << " of " << plate_data.size()
                      << ": grid origin (" << origin.x() << ", " << origin.y() << ")\n";
    }
    return 0;
}

#if defined(ENGINE_BAMBU)
static int resolve_bambu_project_profiles(const CliOptions& o, PlateOutcome& outcome,
            Slic3r::DynamicPrintConfig& config) {
    const auto& verbose = o.verbose;

    // ── Rebuild config via PresetBundle ──────────────────────────
    // The 3MF's project_settings.config is a merged flat file with
    // multi-element arrays from the printer profile's variant support.
    // BambuStudio desktop builds config from individual presets via
    // PresetBundle::full_config(), producing correctly-sized vectors.
    // Replicate that here: load system presets → select by name →
    // full_config() → overlay any 3MF-only keys.
    //
    // If this fails (profiles not found, names don't match, etc.)
    // we silently fall back to the flat 3MF config from load_bbs_3mf.
    {
        // Same executable dir and profiles tree that
        // configure_engine_resources() resolved at startup, including
        // the package-root layout. exe_dir is kept for the
        // preset_resolution_failed diagnostic below.
        const boost::filesystem::path exe_dir = engine_executable_dir(o.argv0.c_str());
        const boost::filesystem::path profiles_dir = engine_profiles_dir(exe_dir);

        bool preset_loaded = false;
        if (!profiles_dir.empty()) {
            try {
                // Prepare data_dir/system/ with symlinks to vendor dirs.
                // Always start fresh to avoid stale cached copies of
                // vendor JSON files that can cause extruder variant
                // lookup failures (see update_values_to_printer_extruders).
                // One staging directory per process: two packaged slices
                // running at once must not delete each other's vendor
                // files mid-load (a shared fixed path with remove_all did).
                //
                // The cleanup guard lives outside the fatal try below so
                // the staged tree survives the PresetBundle load; it gets
                // its path before create_directories so a half-created
                // staging dir is still removed.
                boost::filesystem::path tmpdir;
                boost::filesystem::path sysdir;
                struct StagingCleanup {
                    boost::filesystem::path dir;
                    ~StagingCleanup() { remove_owned_temp_dir(dir); }
                } staging_cleanup;
                // A staging step that fails — creating the staging dir
                // or a copy that even the stream-copy fallback cannot
                // complete — is fatal: the same event + exit-code shape
                // as the other preset failures below, except the slice
                // stops here instead of running on the flat 3MF config
                // with no presets at all.
                try {
                    tmpdir = make_owned_temp_dir("slicer_cli_presets-");
                    sysdir = tmpdir / "system";
                    staging_cleanup.dir = tmpdir;
                    boost::filesystem::create_directories(sysdir);
                    for (auto& entry : boost::filesystem::directory_iterator(profiles_dir)) {
                        auto dst = sysdir / entry.path().filename();
                        if (boost::filesystem::is_directory(entry.path()))
                            stage_vendor_dir(entry.path(), dst);
                        else if (entry.path().extension() == ".json")
                            stage_file_copy(entry.path(), dst);
                    }
                } catch (const std::exception& e) {
                    emit_event({{"event","preset_error"},
                                {"tag","PresetStagingFailed"},
                                {"exe_dir", exe_dir.empty() ? std::string{} : exe_dir.string()},
                                {"profiles_dir", profiles_dir.string()},
                                {"message", std::string("Preset staging failed: ") + e.what()}});
                    std::cerr << "  Preset staging failed: " << e.what() << "\n";
                    set_outcome_failure(outcome, CLI_ENVIRONMENT_ERROR, e.what());
                    return 1;
                }
                // resources_dir() already points at profiles_dir/.. — set
                // once for both engines by configure_engine_resources().
                Slic3r::set_data_dir(tmpdir.string());

                Slic3r::PresetBundle preset_bundle;

                // Load ALL vendors (BBL, Creality, Qidi, …)
                // First vendor uses LoadSystem (resets bundle),
                // subsequent vendors just load without reset.
                bool first_vendor = true;
                for (auto& dir_entry : boost::filesystem::directory_iterator(sysdir)) {
                    if (dir_entry.path().extension() != ".json")
                        continue;
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
                            // load_vendor_configs_from_json without LoadSystem
                            // won't reset — it appends to existing presets
                            preset_bundle.load_vendor_configs_from_json(
                                sysdir.string(), vname,
                                Slic3r::PresetBundle::LoadConfigBundleAttributes(),
                                Slic3r::ForwardCompatibilitySubstitutionRule::EnableSilent);
                        }
                    } catch (...) {
                        // Skip vendors that fail to load
                    }
                }

                // Read preset names from the 3MF config
                std::string printer_name = config.opt_string("printer_settings_id");
                std::string print_name   = config.opt_string("print_settings_id");
                std::string filament_name;
                if (auto* fsi = config.option<Slic3r::ConfigOptionStrings>("filament_settings_id"))
                    if (!fsi->values.empty())
                        filament_name = fsi->values[0];

                std::cout << "  Preset lookup: printer='" << printer_name
                          << "' print='" << print_name
                          << "' filament='" << filament_name << "'\n";

                if (!printer_name.empty() && !print_name.empty() && !filament_name.empty()) {
                    bool ok_printer  = preset_bundle.printers.select_preset_by_name(printer_name, true);
                    bool ok_print    = preset_bundle.prints.select_preset_by_name(print_name, true);
                    bool ok_filament = preset_bundle.filaments.select_preset_by_name(filament_name, true);

                    std::cout << "  Preset match: printer=" << ok_printer
                              << " (resolved='" << preset_bundle.printers.get_edited_preset().name << "')"
                              << " print=" << ok_print
                              << " (resolved='" << preset_bundle.prints.get_edited_preset().name << "')"
                              << " filament=" << ok_filament
                              << " (resolved='" << preset_bundle.filaments.get_edited_preset().name << "')\n";

                    // Only use PresetBundle config if ALL three presets were found
                    // (not just the default fallbacks)
                    if (ok_printer && ok_print && ok_filament
                        && preset_bundle.printers.get_edited_preset().name == printer_name
                        && preset_bundle.prints.get_edited_preset().name == print_name
                        && preset_bundle.filaments.get_edited_preset().name == filament_name)
                    {
                        // Same flow as BambuStudio desktop:
                        // full_config() builds defaults → printer → process → filament
                        // then apply project_settings.config on top.
                        Slic3r::DynamicPrintConfig base_config = preset_bundle.full_config();
                        base_config.apply(config, /*ignore_nonexistent=*/true);
                        config = std::move(base_config);
                        preset_loaded = true;

                        if (verbose)
                            std::cout << "  Presets: " << printer_name
                                      << " / " << print_name
                                      << " / " << filament_name << "\n";
                    }
                }
            } catch (const std::exception& e) {
                emit_event({{"event","preset_error"},
                            {"tag","PresetBundleException"},
                            {"message", std::string("PresetBundle exception: ") + e.what()}});
                std::cerr << "  PresetBundle exception: " << e.what() << "\n";
            }
        }
        if (!preset_loaded) {
            // The slice is about to run on the flat, merged 3MF config
            // instead of a preset-resolved one. The desktop app can
            // never be in this state, so nothing downstream expects it;
            // it must not stay a plain stdout line.
            emit_event({{"event","preset_resolution_failed"},
                        {"tag","FlatThreeMfConfigFallback"},
                        {"exe_dir", exe_dir.empty() ? std::string{} : exe_dir.string()},
                        {"profiles_dir", profiles_dir.empty() ? std::string{} : profiles_dir.string()},
                        {"message","Presets were not resolved; slicing from the flat 3MF config"}});
            std::cout << "  WARNING: Using flat 3MF config (presets not resolved)\n";
            if (!exe_dir.empty())
                std::cout << "  exe_dir: " << exe_dir << "\n";
            else
                std::cout << "  exe_dir: <empty — could not determine executable path>\n";
            if (!profiles_dir.empty())
                std::cout << "  profiles_dir: " << profiles_dir << "\n";
            else
                std::cout << "  profiles_dir: <not found>\n";
        }
    }
    return 0;
}
#endif

int load_project_archive(CliRunContext& context, const CliOptions& o, LoadedProject& loaded_project,
            RunPlatePlan& plate_plan, PlateOutcome& outcome, int plate_id, PlateLoadSettings& settings,
            const PlateLoadInputs& inputs) {
    // The plate filter of the one load (Model::read_from_file(...,
    // plate_to_slice), BambuStudio.cpp 1889; OrcaSlicer.cpp 1789): the
    // plate the run slices, or 0 for every plate.
    const int load_plate_id = plate_id;
    auto& config = settings.config;
    const auto& input_file = o.input_file;
    const auto& verbose = o.verbose;
    auto& model = loaded_project.model;
    auto& is_bbl_3mf = loaded_project.is_bbl_3mf;

#ifdef ENGINE_BAMBU
    Slic3r::ConfigOptionInts accepted_3mf_nozzle_map(
        config.option<Slic3r::ConfigOptionInts>("filament_nozzle_map", true)->values);
    // The file's own filament_nozzle_map is what an earlier slice grouped
    // (written back from Print::get_filament_nozzle_maps, BambuStudio.cpp
    // 7107-7108), not a placement anyone asked for: the official CLI
    // reads a nozzle map only from its command line, and only in a
    // manual mode (BambuStudio.cpp 6789, 6836-6838). So it is kept as
    // the file's value but never makes the mapping explicit; only a
    // --config/--machine/--process/--filament overlay can.
    bbs_3mf_config_contains_nozzle_map(input_file, accepted_3mf_nozzle_map);
#endif
    // The path the engine's 3MF loader reads: the input itself, or on
    // the Bambu build a converted copy when the file states line widths
    // as percentages (see rewrite_percent_line_widths).
    std::string load_path = input_file;
#ifdef ENGINE_BAMBU
    PercentRewrite percent_rewrite;
    rewrite_percent_line_widths(input_file, percent_rewrite);
    if (!percent_rewrite.refusal.empty()) {
        emit_event({{"event","config_refused"},
                    {"tag","PercentLineWidthUnusable"},
                    {"message", percent_rewrite.refusal}});
        std::cerr << "Error: " << percent_rewrite.refusal << "\n";
        set_outcome_failure(outcome, CLI_INVALID_VALUES_IN_3MF, percent_rewrite.refusal);
        return 1;
    }
    if (percent_rewrite.changed) {
        load_path = percent_rewrite.temp_path;
        emit_event({{"event","config_normalized"},
                    {"tag","PercentLineWidthConverted"},
                    {"nozzle_diameter", percent_rewrite.nozzle_mm},
                    {"converted", percent_rewrite.converted},
                    {"message","Converted " + std::to_string(percent_rewrite.converted.size()) +
                               " line width(s) stated as a percentage of the " +
                               mm_text(percent_rewrite.nozzle_mm) + " mm nozzle into mm"}});
    }
#endif
    ProjectArchiveLoad archive;
    auto& config_subst = archive.config_subst;
    auto& presets = archive.presets;
    auto& file_version = archive.file_version;
#ifdef ENGINE_ORCA
    auto& is_orca_3mf = archive.is_orca_3mf;
#endif

    if (read_3mf_archive(o, outcome, load_plate_id, config, archive, load_path, loaded_project) != 0)
        return 1;
    // The settings exactly as the file states them, and whether they
    // were kept: the rebase below (Orca build) replaces `config` with
    // the preset-rebased settings, and the steps that describe the FILE
    // read these instead (read_project_facts).
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
    // OrcaSlicer from 2.5 (OrcaSlicer.cpp 4244-4259 at f92a7c0): a mixed
    // filament slot is virtual but needs a filament entry of its own. Checked
    // on the file's own lists, before the load below sizes them to the
    // filament count: without the slot's entry, the objects that name it
    // would be put on the first filament and print in one colour, with no
    // word about it.
    if (const auto* is_mixed = config.option<Slic3r::ConfigOptionBools>("filament_is_mixed")) {
        const auto*  file_filaments = config.option<Slic3r::ConfigOptionStrings>("filament_settings_id");
        const size_t filament_count = file_filaments != nullptr ? file_filaments->values.size() : 0;
        for (size_t slot = filament_count; filament_count > 0 && slot < is_mixed->values.size(); ++slot)
            if (is_mixed->values[slot]) {
                const std::string refusal = "Mixed filament slot " + std::to_string(slot + 1) +
                                            " has no filament of its own: the project states " + std::to_string(filament_count) +
                                            " filament(s). Save the project with one filament per slot, including each mixed one.";
                emit_event({{"event", "config_refused"}, {"tag", "MixedFilamentWithoutSlot"}, {"message", refusal}});
                std::cerr << "Error: " << refusal << "\n";
                set_outcome_failure(outcome, CLI_MIXED_FILAMENT_INVALID, refusal);
                return 1;
            }
    }
#endif
    Slic3r::DynamicPrintConfig project_file_config;
    bool                       project_file_config_kept = false;
#ifdef ENGINE_ORCA
    // ── A Bambu Studio project is rebased onto its system presets ────
    // The desktop does not slice a project's settings as the file
    // states them. It loads this engine's system presets
    // (PresetBundle::load_presets, Plater.cpp 6040-6160 at 31f6803),
    // hands the file's settings to PresetBundle::load_config_model
    // (PresetBundle.hpp 388 -> PresetBundle.cpp 4239-4567), which loads
    // the project's printer, process and filament presets over the ones
    // this engine ships and, for every key the file does not list in
    // different_settings_to_system, takes the system preset's value
    // (Preset.cpp 2446-2457 load_external_preset ->
    // update_non_diff_values_to_base_config), and slices
    // PresetBundle::full_config(false) (Plater.cpp 7959-7963). Every
    // Bambu-only encoding of a key the maker did not touch -- the
    // 0-based filament indices, tree_support_wall_count -1,
    // raft_first_layer_expansion -1, ensure_vertical_shell_thickness
    // "enabled" -- therefore never reaches the engine. Upstream main's
    // CLI rebases the same way for its --uptodate path (OrcaSlicer
    // 94266c28, OrcaSlicer.cpp 3075-3108 and 2697-2726 on main).
    // wipe_tower_x/y are the plate's own positions and are put back
    // after the load (Plater.cpp 6380-6389).
    // This scope test is the one place the scope is decided.
#ifdef ENGINE_SNAPMAKER
    // Snapmaker's loader reports no "made by OrcaSlicer" flag (Format/bbs_3mf.hpp
    // at bac6fd5), so the maker is read from the file's own tags: a Snapmaker Orca
    // project (Application "Snapmaker_Orca-<version>", Format/bbs_3mf.cpp:6645-6647)
    // or an OrcaSlicer one. The maker decides only the Bambu Studio walls
    // warning below; the rebase covers every project.
    {
        std::string model_xml;
        if (read_zip_member(input_file, "3D/3dmodel.model", model_xml))
            is_orca_3mf = slicer_cli::snapmaker_engine_made(model_metadata(model_xml));
    }
#endif
    // Every project one of the desktops made is rebased, this engine's
    // own too: the desktop's load has no test on who made the file
    // (OrcaSlicer Plater.cpp 6063-6110 lets a From_Orca project load
    // its config, 6415 load_config_model, Preset.cpp 2516-2535;
    // Snapmaker_Orca Plater.cpp 12103, Preset.cpp 2121-2148).
    const bool bambu_made_project = is_bbl_3mf;
#if defined(ENGINE_SNAPMAKER) || defined(ENGINE_ORCA)
    context.g_file_made_by_bambustudio = is_bbl_3mf && !is_orca_3mf;
#endif
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
    // The loader's own is_orca_3mf flag does not recognise every
    // OrcaSlicer-made file (it is false for an "OrcaSlicer-0.1.0-dev"
    // project), so the walls warning reads the maker from the file's own
    // tags, the way --engine-info reads it. The rebase scope above stays
    // the loader's flag: it is the OrcaSlicer build's behaviour on main,
    // which this change leaves as it is.
    {
        std::string maker_xml;
        if (read_zip_member(input_file, "3D/3dmodel.model", maker_xml)) {
            const auto maker_meta = model_metadata(maker_xml);
            const auto maker_app  = maker_meta.find("Application");
            const bool orca_made  = maker_meta.count("OrcaSlicer") != 0 ||
                                    (maker_app != maker_meta.end() &&
                                     boost::starts_with(maker_app->second, "OrcaSlicer-"));
            context.g_file_made_by_bambustudio = is_bbl_3mf && !orca_made;
        }
    }
#endif
#else
    // The BambuStudio build loads every Bambu Studio project the same
    // way: Bambu Studio's own Plater does (load_config_model,
    // Plater.cpp 12155 at v02.09.01.70 -> load_config_file_config,
    // PresetBundle.cpp 5265 -> load_external_preset, Preset.cpp:
    // update_non_diff_values_to_base_config for every key the file
    // does not list in different_settings_to_system). A project saved
    // by an older version then slices with today's start G-code, and a
    // setting the maker never changed takes today's preset value, as
    // it does on the desktop. --project-settings-as-saved keeps the
    // file's own values, which is what the official command line does.
    const bool bambu_made_project = is_bbl_3mf;
#endif
    load_desktop_project_presets(o, loaded_project, outcome, config, file_version, presets,
            project_file_config_kept, project_file_config, bambu_made_project);
#ifdef ENGINE_ORCA
    if (convert_project_object_values(context, outcome, input_file, config, model,
            config_subst, project_file_config) != 0)
        return 1;
    normalize_project_support_filaments(config);
#endif
#ifdef ENGINE_SNAPMAKER
    // The desktop's rule for filament indices out of range on load,
    // after the switch's paint remap (cli_snapmaker.hpp).
    slicer_cli::fix_out_of_range_filaments(model, config, project_filament_count(config), context.g_filament_id_remap,
                                           context.g_filament_id_total, context.g_filament_count_changed);
#endif
    if (check_project_version(o, outcome, config, file_version) != 0)
        return 1;
    check_project_substitutions(o, settings.unknown_values, settings.unknown_items, config_subst);
#ifdef ENGINE_BAMBU
    // The importer may partially mutate a vector before rejecting it.
    // Retain the last usable input map, or the original driver default
    // when no usable map was supplied. This does not establish provenance.
    config.set_key_value("filament_nozzle_map", accepted_3mf_nozzle_map.clone());
#endif
    if (read_loaded_plate_layout(loaded_project, plate_plan, outcome, verbose,
            plate_id, config, file_version, inputs, presets,
            project_file_config_kept, project_file_config) != 0)
        return 1;

#ifdef ENGINE_BAMBU
    if (resolve_bambu_project_profiles(o, outcome, config) != 0)
        return 1;
#endif // ENGINE_BAMBU — PresetBundle preset-resolution + staging symlinks.
    return 0;
}

} // namespace slicer_cli
