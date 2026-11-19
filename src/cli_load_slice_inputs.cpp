#include "cli_load_slice_inputs.hpp"
#include "libslic3r_version.h"

#include "cli_run_context.hpp"
#include "cli_slice_state.hpp"
#include "cli_plate_scope.hpp"
#include "cli_options.hpp"
#include "cli_assemble.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/BuildVolume.hpp"
#include <set>
#include <functional>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/nowide/fstream.hpp>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <limits>
#include "cli_arrange_geometry.hpp"
#include "cli_engine_events.hpp"
#include "cli_failure_reason.hpp"
#include "cli_filament_config.hpp"
#include "cli_json_config.hpp"
#include "cli_load_settings.hpp"
#include "cli_model_load.hpp"
#include "cli_named_presets.hpp"
#include "cli_nozzle_options.hpp"
#include "cli_percent_widths.hpp"
#include "cli_plate_arrange.hpp"
#include "cli_plate_plan.hpp"
#include "cli_preset_staging.hpp"
#include "cli_printer_catalog.hpp"
#include "cli_project_archive.hpp"
#include "cli_project_load.hpp"
#include "cli_project_settings.hpp"
#include "cli_project_values.hpp"
#include "cli_result_json.hpp"
#include "cli_run_cleanup.hpp"
#include "cli_run_steps.hpp"
#include "cli_setting_overrides.hpp"
#include "cli_snapmaker.hpp"
#include "desktop_presets.hpp"
#include "libslic3r/PresetBundle.hpp"
#ifdef ENGINE_BAMBU
#include "libslic3r/FilamentMixer.hpp"
#endif

namespace slicer_cli {

using json = nlohmann::json;

int load_slice_inputs(CliRunContext& context, const CliOptions& o, RunState& rs, int plate_id,
                      const PlateLoadInputs& inputs, PlateLoadSettings& settings, PlateOutcome& outcome) {
    const std::string& input_file      = o.input_file;
    const std::string& machine_config  = o.machine_config;
    const std::string& filament_config = o.filament_config;
    const std::string& process_config  = o.process_config;
    const std::string& bundle_config   = o.bundle_config;
    const bool verbose                 = o.verbose;
    const auto& overrides = o.overrides;
    Slic3r::Model& model = rs.project.model;
    bool& is_bbl_3mf = rs.project.is_bbl_3mf;
    Slic3r::PlateDataPtrs& plate_data = rs.project.plate_data;
    std::vector<Slic3r::Vec2d>& plate_origins = rs.project.plate_origins;
    std::optional<Slic3r::DynamicPrintConfig>& plan_file_config = rs.layout.plan_file_config;
    Slic3r::Semver& plan_file_version = rs.layout.plan_file_version;
    int& plan_file_filaments = rs.layout.plan_file_filaments;
    slicer_cli::ProjectFacts& project_facts = rs.project.project_facts;
    std::vector<Slic3r::Preset>& project_presets_kept = rs.project.project_presets_kept;
    Slic3r::DynamicPrintConfig& extra = rs.settings.extra;
#ifdef ENGINE_BAMBU
    bool& explicit_config_supplied_nozzle_map = rs.settings.explicit_config_supplied_nozzle_map;
#endif
    int& shrink_to_new_bed = rs.settings.shrink_to_new_bed;
    slicer_cli::SettingsMerge& settings_merge = rs.settings.settings_merge;
    std::string& merged_print_sequence = rs.settings.merged_print_sequence;
#ifdef ENGINE_BAMBU
    size_t& project_filaments = rs.settings.project_filaments;
#endif
    // The plate filter of the one load (Model::read_from_file(...,
    // plate_to_slice), BambuStudio.cpp 1889; OrcaSlicer.cpp 1571): the
    // plate the run slices, or 0 for every plate.
    const int load_plate_id = plate_id;
    auto& config = settings.config;
    auto& unknown_values = settings.unknown_values;
    auto& unknown_items = settings.unknown_items;

    std::cout << "\nConfiguring print settings...\n";
#ifdef ENGINE_BAMBU
    // Start with BambuStudio's full defaults (ensures all keys exist with correct
    // types) PLUS the BBS-specific extruder-variant normalization that coaxes the
    // Bambu engine into accepting a non-Bambu (e.g. Snapmaker U1) printer config.
    set_default_config(config);
#else
    // OrcaSlicer handles non-Bambu printers (U1/Prusa/Voron/…) natively, so none of
    // the BBS variant-array normalization is needed.  Just seed every key with the
    // engine's defaults; load_bbs_3mf (next) overlays the 3MF's project_settings.config.
    config.apply(Slic3r::FullPrintConfig::defaults(), true);
#endif
    extra = o.extra_config;

    // Load model
    std::cout << "Loading model: " << input_file << "\n";
    // Pre-set backup_path to a writable temp dir of this run's own so the
    // backup manager never touches the read-only /bamboo_model network
    // path, and parallel runs never share the loader's _temp_N.config
    // files (run_backup_path), with the input's own directories in it (the
    // OrcaSlicer loader does not create them: create_archive_dirs).
    set_run_backup_path(context, model);
    create_archive_dirs(input_file, run_backup_path(context));
    // The run's model is detached when this block ends, so no copy of it
    // (each plate's own, or any temporary) can take the folder down while
    // a later load still writes into it (detach_run_backup_path).
    const RunBackupDetach detach_run_model{model};
    if (inputs.assemble_input) {
        // The list's plates the run slices: plate N, or every plate. Each
        // plate's objects go to its place in the plate grid once the bed
        // is known (below).
        if (plate_id > (int)context.g_assemble->plates.size() || plate_id < 0) {
            set_outcome_failure(outcome, CLI_INVALID_PARAMS, "The assemble list has no plate " + std::to_string(plate_id) + ".");
            return 1;
        }
        rs.layout.plates.assign(context.g_assemble->plates.size(), RunPlate());
        for (size_t k = 0; k < context.g_assemble->plates.size(); ++k) {
            rs.layout.plates[k].all = false;
            if (plate_id > 0 && int(k) != plate_id - 1)
                continue;
            const size_t before = model.objects.size();
            for (const Slic3r::ModelObject* object : context.g_assemble->plates[k].loaded_obj_list)
                model.add_object(*object);
            for (size_t oi = before; oi < model.objects.size(); ++oi)
                for (Slic3r::ModelInstance* inst : model.objects[oi]->instances)
                    rs.layout.plates[k].members.push_back(inst);
            emit_event({{"event","model_loaded"}, {"tag","AssemblePlateBuilt"}, {"plate_id", int(k) + 1},
                        {"objects", model.objects.size() - before},
                        {"message","Plate " + std::to_string(k + 1) + " of the assemble list holds " +
                                   std::to_string(model.objects.size() - before) + " object(s)"}});
        }
        for (size_t k = 0; k < context.g_assemble->plates.size(); ++k) {
            auto* pd = new Slic3r::PlateData();
            pd->plate_index = int(k);
            pd->plate_name  = context.g_assemble->plates[k].plate_name;
            pd->config      = context.g_assemble->plate_configs[k];
            plate_data.push_back(pd);
        }
    } else if (inputs.geometry_input) {
        const slicer_cli::ModelLoadResult loaded = slicer_cli::load_geometry_files(o, o.input_files, 0, model);
        if (loaded.code != 0 && o.input_files.size() == 1 && input_is_stl(input_file) &&
            (loaded.code == CLI_DATA_FILE_ERROR || loaded.code == CLI_FILE_NOTFOUND)) {
            // One STL that cannot be read: the refusal slicer-cli has
            // always given for it.
            std::cerr << "Failed to load STL file\n";
            set_outcome_failure(outcome, CLI_DATA_FILE_ERROR);
            return 1;
        }
        slicer_cli::add_failure_reason(outcome.error_reason, loaded.failure_reason);
        if (loaded.code != 0) {
            emit_event({{"event","load_error"}, {"tag","ModelLoadFailed"}, {"path", input_file},
                        {"message", loaded.message}});
            std::cerr << "Error: " << loaded.message << "\n";
            set_outcome_failure(outcome, loaded.code, loaded.message);
            return 1;
        }
    } else if (input_is_3mf(input_file)) {
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
        Slic3r::ConfigSubstitutionContext config_subst(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
        std::vector<Slic3r::Preset*> presets;
        struct PresetsRelease {
            std::vector<Slic3r::Preset*>& presets;
            ~PresetsRelease() { for (Slic3r::Preset* p : presets) delete p; presets.clear(); }
        } presets_release{presets};
        Slic3r::Semver file_version;

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
            &config_subst,
            &model,
            &plate_data,
            &presets,
            &is_bbl_3mf,
            &file_version,
            nullptr,   // proFn (progress callback)
            strategy,
            nullptr,   // BBLProject
            load_plate_id   // 0 = all plates, >0 = specific plate
        );
#else
        // OrcaSlicer's load_bbs_3mf inserts a bool* is_orca_3mf between is_bbl_3mf
        // and file_version (and drops the two trailing Bambu-only params).
        bool is_orca_3mf = false;
        bool result = Slic3r::load_bbs_3mf(
            load_path.c_str(),
            &config,
            &config_subst,
            &model,
            &plate_data,
            &presets,
            &is_bbl_3mf,
            &is_orca_3mf,
            &file_version,
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
            &config_subst,
            &model,
            &plate_data,
            &presets,
            &is_bbl_3mf,
            &file_version,
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
        slicer_cli::convert_flow_type_spelling(config, config_subst, "3mf:project_settings.config");
#endif
        emit_config_substitutions(config_subst, "3mf:project_settings.config");
        outcome.unknown_settings = config_subst.unrecogized_keys;   // sic — upstream spelling
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
        // The settings exactly as the file states them, and whether they
        // were kept: the rebase below (Orca build) replaces `config` with
        // the preset-rebased settings, and the steps that describe the FILE
        // read these instead (read_project_facts).
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
        bool is_orca_3mf = false;
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
                rs.project.project_loaded_like_desktop = true;
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
#ifdef ENGINE_ORCA
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
#endif
#ifdef ENGINE_SNAPMAKER
        // The desktop's rule for filament indices out of range on load,
        // after the switch's paint remap (cli_snapmaker.hpp).
        slicer_cli::fix_out_of_range_filaments(model, config, project_filament_count(config), context.g_filament_id_remap,
                                               context.g_filament_id_total, context.g_filament_count_changed);
#endif
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
        // OrcaSlicer.cpp 1588-1592 at 31f6803): a file saved by a newer
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
#ifdef ENGINE_BAMBU
        // The importer may partially mutate a vector before rejecting it.
        // Retain the last usable input map, or the original driver default
        // when no usable map was supplied. This does not establish provenance.
        config.set_key_value("filament_nozzle_map", accepted_3mf_nozzle_map.clone());
#endif
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
        rs.project.file_version = file_version;
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
            rs.layout.plates.assign(plate_data.size(), RunPlate());
            for (int index = 0; index < (int)plate_data.size(); ++index) {
                rs.layout.plates[index].origin = plate_origins[index];
                rs.layout.plates[index].all = index == plate_id - 1;
            }
        } else if (is_bbl_3mf && plate_data.size() > 1 && !inputs.trailing_port) {
            rs.layout.plates.assign(plate_data.size(), RunPlate());
            std::set<const Slic3r::ModelInstance*> placed;
            for (int index = 0; index < (int)plate_data.size(); ++index) {
                rs.layout.plates[index].origin = plate_origins[index];
                rs.layout.plates[index].all = false;
                if (plate_data[index] == nullptr) continue;
                std::set<int> identify_ids;
                for (const auto& entry : plate_data[index]->obj_inst_map)
                    if (entry.second.second > 0) identify_ids.insert(entry.second.second);
                for (Slic3r::ModelObject* object : model.objects)
                    for (Slic3r::ModelInstance* inst : object->instances)
                        if (inst->loaded_id > 0 && identify_ids.count(int(inst->loaded_id)) && placed.insert(inst).second)
                            rs.layout.plates[index].members.push_back(inst);
            }
            for (Slic3r::ModelObject* object : model.objects)
                for (size_t ii = 0; ii < object->instances.size(); ++ii) {
                    if (placed.count(object->instances[ii])) continue;
                    const Slic3r::BoundingBoxf3 box = object->instance_convex_hull_bounding_box(ii);
                    for (int index = 0; index < (int)plate_data.size(); ++index)
                        if (plate_scene_box(config, file_version, index, (int)plate_data.size()).intersects(box)) {
                            rs.layout.plates[index].members.push_back(object->instances[ii]);
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

#ifdef ENGINE_BAMBU
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
#endif // ENGINE_BAMBU — PresetBundle preset-resolution + staging symlinks.
   // OrcaSlicer slices directly from the flat 3MF project_settings.config.
    } else if (inputs.calib_self_geometry && input_file.empty()) {
        std::cout << "No --input: pressure_advance_pattern will synthesize a handle cube.\n";
    } else {
        // The kinds this engine's loaders read (cli_model_load.cpp).
        std::cerr << "Unsupported file format. This engine loads: "
                  << slicer_cli::loadable_model_extensions() << "\n";
        set_outcome_failure(outcome, CLI_DATA_FILE_ERROR);
        return 1;
    }

    // For pressure_advance_pattern the model is generated later by
    // apply_pa_pattern (before print.apply), so an empty model here is fine.
    if (model.objects.empty() && !inputs.calib_self_geometry) {
        std::cerr << "No objects found in model\n";
        set_outcome_failure(outcome, CLI_NO_SUITABLE_OBJECTS);
        return 1;
    }

    // Model files after a project 3MF: loaded the same way as geometry
    // (the official CLI reads them after the first file, BambuStudio.cpp
    // 1855-2163) and placed further down with the other new objects.
    if (!inputs.geometry_input && o.input_files.size() > 1) {
        const std::vector<std::string> rest(o.input_files.begin() + 1, o.input_files.end());
        const slicer_cli::ModelLoadResult loaded = slicer_cli::load_geometry_files(o, rest, 1, model);
        slicer_cli::add_failure_reason(outcome.error_reason, loaded.failure_reason);
        if (loaded.code != 0) {
            emit_event({{"event","load_error"}, {"tag","ModelLoadFailed"}, {"path", rest.front()},
                        {"message", loaded.message}});
            std::cerr << "Error: " << loaded.message << "\n";
            set_outcome_failure(outcome, loaded.code, loaded.message);
            return 1;
        }
    }
    // A run of one plate (model files, a 3MF sliced as one plate): every
    // instance is on it, at the origin.
    if (rs.layout.plates.empty())
        rs.layout.plates.assign(1, RunPlate());
#ifdef ENGINE_BAMBU
    // Bambu Studio puts each plate's saved tower back inside the plate
    // when it opens a project (cli_project_load.hpp). Here, before the
    // flags are laid on the settings: a position a flag gives stays as
    // given, and the project's settings, which the exports write, hold
    // the corrected one. Every plate whose objects are loaded (one plate
    // with --slice N or --plate N). --arrange places the tower itself.
    if (rs.project.project_loaded_like_desktop && !o.arrange_forced())
        if (const auto* area = config.option<Slic3r::ConfigOptionPoints>("printable_area");
            area != nullptr && area->values.size() >= 3) {
            const Slic3r::BoundingBoxf reach = tower_reach_box(config, *area);
            const double height = config.has("printable_height") ? config.opt_float("printable_height") : 0.;
            const Slic3r::BoundingBoxf3 plate_box(Slic3r::Vec3d(reach.min.x(), reach.min.y(), 0.),
                                                  Slic3r::Vec3d(reach.max.x(), reach.max.y(), height));
            const TowerWipeParams wipe = tower_wipe_params(config);
            const auto exclude = arrange_exclude_boxes(config);
            for (int index = 0; index < (int)rs.layout.plates.size(); ++index) {
                if (plate_id > 0 && index != plate_id - 1)
                    continue;
                PlateScope scope(model, rs.layout.plates[index]);
                slicer_cli::project_load_tower_inside_plate(
                    config, index, (int)rs.layout.plates.size(), reach,
                    (int)arrange_plate_extruders(model, config, index).size(),
                    [&](int filaments) {
                        return arrange_estimate_wipe_tower_size(model, config, wipe.width, wipe.volume,
                                                                wipe.extruder_count, filaments, wipe.wrapping,
                                                                plate_box, exclude, nullptr,
                                                                /*legacy_behavior=*/false);
                    });
            }
        }
#else
    // OrcaSlicer and Snapmaker Orca keep a project's saved tower position
    // when they open it (cli_project_load.hpp), and their slice reads a
    // plate the list holds no position for as the first plate's
    // (get_at, Config.hpp 627 at 8500fcd; 438 at bac6fd5).
    if (rs.project.project_loaded_like_desktop && !o.arrange_forced())
        slicer_cli::project_load_tower_positions_per_plate(config, (int)rs.layout.plates.size());
#endif
    const bool has_unplaced_objects = std::any_of(model.objects.begin(), model.objects.end(),
        [](const Slic3r::ModelObject* obj) { return obj->instances.empty(); });

    // Ensure all objects have at least one instance
    // Also set use_loaded_id_for_label so that the identify_id from
    // model_settings.config is used for OBJECT_ID labels in G-code
    // (matches BambuStudio desktop behavior at BambuStudio.cpp:6196).
    // Geometry still to be placed gets its instance from the placement.
    for (auto* obj : model.objects) {
        if (obj->instances.empty() && !has_unplaced_objects) {
            obj->add_instance();
        }
        for (auto* inst : obj->instances) {
            inst->use_loaded_id_for_label = true;
        }
    }

    std::cout << "Model loaded successfully:\n";
    for (const auto* obj : model.objects) {
        std::cout << "  - " << obj->name << " (" << obj->volumes.size()
                  << " volumes, " << obj->instances.size() << " instances)\n";
    }

    // Mesh auto-repair census. The GUI puts a warning icon beside every
    // object whose mesh admesh had to repair, with a "(Repair)" hyperlink
    // (GUI_ObjectList.cpp:516/521). The CLI repairs silently, so an agent
    // cannot tell a clean input from one that was rewritten under it.
    for (auto* obj : model.objects) {
        const int repaired = obj->get_repaired_errors_count();
        if (repaired <= 0) continue;
        emit_event({{"event","mesh_repaired"},
                    {"tag","RepairedMeshErrors"},
                    {"object", obj->name},
                    {"repaired_error_count", repaired},
                    {"message","Mesh errors were auto-repaired on load for '" + obj->name + "'"}});
    }

    // Load config files in order (later ones can override earlier ones)
    // A profile that fails to load leaves the slice running on whatever was
    // already in `config` — silently, with only a stderr line. The agent
    // that chose that profile must be told it did not take effect.
    auto load_profile = [&](const std::string& path, const char* kind) {
        if (path.empty()) return;
        if (load_json_config(path, config, verbose,
                             std::string("profile:") + kind + ":" + path,
#ifdef ENGINE_BAMBU
                             &explicit_config_supplied_nozzle_map
#else
                             nullptr
#endif
                             )) {
            // A shipped preset holds only its differences from its
            // parents: read the rest of the chain in, so the file means
            // the same here as it does in the desktop app.
            apply_file_inherits(o.argv0, path, kind, config, verbose);
            return;
        }
        emit_event({{"event","config_load_failed"},
                    {"tag","ProfileLoadFailed"},
                    {"kind", kind},
                    {"path", path},
                    {"message", std::string("Failed to load ") + kind +
                                " config; slicing continues with the previously resolved settings"}});
        std::cerr << "Warning: Failed to load " << kind << " config\n";
        slicer_cli::add_failure_reason(outcome.error_reason, std::string("The ") + kind + " file " + path +
                                                             " could not be read, so its settings were not used.");
    };
    // Presets by name: the desktop app's full_config() for the named
    // printer, process and filaments, over the engine defaults.
    if (context.g_preset_config)
        config.apply(*context.g_preset_config);
    // Capture preset names before raw settings files can replace their
    // id fields. A settings file is configuration input, not proof that
    // its arbitrary ids were selected from this engine's preset catalog.
    // Project inputs, including named project switches, are deliberately
    // omitted: an embedded id can be arbitrary and switch defaults are
    // decided later in the settings merge.
    if (context.g_preset_config) {
        if (const auto* id = config.option<Slic3r::ConfigOptionString>("printer_settings_id"))
            outcome.printer_preset = id->value;
        if (const auto* id = config.option<Slic3r::ConfigOptionString>("print_settings_id"))
            outcome.process_preset = id->value;
        if (const auto* ids = config.option<Slic3r::ConfigOptionStrings>("filament_settings_id"))
            outcome.filament_presets = ids->values;
    }
    load_profile(bundle_config,   "bundle");
    load_profile(machine_config,  "machine");
    load_profile(process_config,  "process");
    load_profile(filament_config, "filament");
    // --load-settings, --load-filaments, --uptodate*: the official
    // settings merge (cli_load_settings.cpp), on the settings loaded so far.
    // (rs.settings.settings_merge; rs.settings.merged_print_sequence: the project's
    // print_sequence after the merge, the official m_print_config's.)
#ifdef ENGINE_BAMBU
    // An assemble list's OBJ colours become filaments in the merge, which
    // needs the filaments loaded (BambuStudio.cpp 2574-2580).
    if (inputs.assemble_input && !context.g_assemble->colours.empty()) {
        settings_merge.input_obj_colours.assign(context.g_assemble->colours.begin(), context.g_assemble->colours.end());
        if (!slicer_cli::wants_settings_merge(o)) {
            const std::string why = "The assemble list's OBJ files carry colours, which become filaments: "
                                    "give the filaments with --load-filaments.";
            std::cerr << "Error: " << why << "\n";
            set_outcome_failure(outcome, CLI_INVALID_PARAMS, why);
            return 1;
        }
    }
#endif
    if (slicer_cli::wants_settings_merge(o)) {
        // Whether the settings loaded so far carry filament_colour (the
        // official m_print_config holds no engine defaults yet; see
        // SettingsMerge::project_has_filament_colour): a project 3MF's
        // own settings, the named presets' full config, or a profile file
        // that names it.
        bool has_colours = (!input_file.empty() && input_is_3mf(input_file) && !inputs.geometry_input && !inputs.assemble_input) ||
                           context.g_preset_config != nullptr;
        for (const std::string* path : {&bundle_config, &machine_config, &process_config, &filament_config})
            has_colours = has_colours || json_file_has_key(*path, "filament_colour");
        settings_merge.project_has_filament_colour = has_colours;
        std::vector<Slic3r::Preset*> preset_ptrs;
        for (Slic3r::Preset& p : project_presets_kept)
            preset_ptrs.push_back(&p);
        // The official merges into m_print_config, which holds the
        // project's print_sequence, not a plate's own (get_print_sequence
        // reads the plate's first, BambuStudio.cpp 4403-4416).
        const slicer_cli::StepResult merged =
            slicer_cli::merge_loaded_settings(o, project_facts, this_engine_profiles_dir(o.argv0).string(), config, extra,
                                              preset_ptrs, settings_merge);
        if (const Slic3r::ConfigOption* seq = config.option("print_sequence"))
            merged_print_sequence = seq->serialize();
        if (merged.code != 0) {
            emit_event({{"event","config_refused"}, {"tag","SettingsFilesRefused"}, {"message", merged.message}});
            std::cerr << "Error: " << merged.message << "\n";
            set_outcome_failure(outcome, merged.code, merged.message);
            return 1;
        }
        slicer_cli::update_object_configs_after_switch(project_facts, settings_merge, config, model);
        outcome.settings_merge = std::make_shared<slicer_cli::SettingsMerge>(settings_merge);
        if (slicer_cli::disable_tower_after_mapping(o, settings_merge, config, extra))
            emit_event({{"event","config_normalized"}, {"tag","PrimeTowerOffOneFilament"},
                        {"message","Every filament slot holds the same filament: the prime tower is off"}});
    }
    // --load-custom-gcodes, --skip-modified-gcodes (cli_load_settings.cpp).
    {
        const slicer_cli::StepResult g = slicer_cli::apply_custom_gcodes(o, o.slice_mode ? o.slice_plate : plate_id, model);
        if (g.code != 0) {
            std::cerr << "Error: " << g.message << "\n";
            set_outcome_failure(outcome, g.code, g.message);
            return 1;
        }
    }
    // Every setting given as a flag, over the loaded settings: the
    // official m_print_config.apply(m_extra_config, true)
    // (BambuStudio.cpp 4091 at 5873b5f; OrcaSlicer.cpp 3542 at 31f6803).
    // The settings before any flag: the settings check's desktop baseline.
    const Slic3r::DynamicPrintConfig settings_before_flags = config;
#ifdef ENGINE_BAMBU
    // The filament switcher and the smart filament assignment that needs
    // it are off unless a flag states them, whatever the project or its
    // plate saved: the official CLI's rule (BambuStudio.cpp 4436-4439 at
    // v02.09.01.70). They describe hardware fitted to the maker's
    // printer, which a headless run cannot see.
    for (const char* key : {"enable_filament_dynamic_map", "has_filament_switcher"})
        if (!extra.has(key))
            extra.set_key_value(key, new Slic3r::ConfigOptionBool(false));
    // --estimate-mode slices with the automatic grouping and a full
    // Standard rack (BambuStudio.cpp 4441-4474 at v02.09.01.70).
    if (o.given_flag("estimate_mode") && o.cli.opt_bool("estimate_mode")) {
        const std::string why = slicer_cli::estimate_mode_nozzle_settings(extra, config);
        if (!why.empty()) {
            std::cerr << "Error: " << why << "\n";
            set_outcome_failure(outcome, CLI_INVALID_PARAMS, why);
            return 1;
        }
    }
#endif
    if (!extra.empty())
        config.apply(extra, true);

    // Two inputs the desktop app decides by choosing a printer preset, not
    // by editing a value: the nozzle and the plate type. Both are settled
    // here, once, where the settings files and the flags have landed and
    // before any plate is sliced.
    {
        // A string setting as the file states it, empty when the key is not
        // there (the engine's own printer_model / printer_variant /
        // printer_settings_id are plain strings in both engines).
        const auto id_text = [](const Slic3r::DynamicPrintConfig& c, const char* key) {
            const auto* opt = c.option<Slic3r::ConfigOptionString>(key);
            return opt != nullptr ? opt->value : std::string();
        };
        const std::string printer_model   = id_text(config, "printer_model");
        const std::string printer_variant = id_text(config, "printer_variant");
        const bool nozzle_given = overrides.count("nozzle_diameter") > 0;
        // The plate type the desktop app sets when a printer is picked
        // (Sidebar::update_all_preset_comboboxes -> set_bed_type_accord_combox,
        // BambuStudio Plater.cpp 3340-3414 and 3703-3715 at 5873b5f;
        // OrcaSlicer Plater.cpp 2527-2556 and 2798-2806 at 31f6803), for a
        // run whose printer comes from settings files: those state no plate
        // type, so the printer preset's own is taken. Without this the
        // engine's own default is used (curr_bed_type's ConfigDef default,
        // btPC "Cool Plate": BambuStudio PrintConfig.cpp 1162 at 5873b5f),
        // which is never the printer's. The named-preset path already sets
        // it (resolve_named_presets -> desktop_bed_type).
        bool plate_chosen = o.given_flag("curr_bed_type");
        for (const std::string* path : {&bundle_config, &machine_config, &process_config, &filament_config})
            plate_chosen = plate_chosen || (!path->empty() && json_file_has_key(*path, "curr_bed_type"));
        const bool plate_needed = !plate_chosen && context.g_preset_config == nullptr &&
                                  (!input_is_3mf(input_file) || inputs.geometry_only_3mf);
        if (nozzle_given || plate_needed) {
            // The printer preset this run's settings belong to: the name
            // the settings state, else the preset this engine ships for the
            // same printer model and nozzle variant. A settings file the
            // desktop exported (Export current configs) carries no
            // printer_settings_id at all, and a shipped leaf file carries
            // one that is empty.
            const std::vector<ShippedPrinterPreset> shipped =
                shipped_printer_presets(this_engine_profiles_dir(o.argv0), printer_model);
            const std::string printer_settings = id_text(config, "printer_settings_id");
            std::string       printer_preset   = printer_settings;
            if (printer_preset.empty() ||
                std::none_of(shipped.begin(), shipped.end(),
                             [&](const ShippedPrinterPreset& p) { return p.name == printer_preset; }))
                if (const std::string by_variant = shipped_printer_preset_of_variant(shipped, printer_variant);
                    !by_variant.empty())
                    printer_preset = by_variant;

            // ── --nozzle ────────────────────────────────────────────────
            // The desktop app changes a printer's nozzle by choosing the
            // printer variant: the printer list is rebuilt per nozzle
            // variant and the process, the line widths and the start G-code
            // come from the chosen preset
            // (Sidebar::update_all_preset_comboboxes, BambuStudio Plater.cpp
            // 3340-3414 at 5873b5f; OrcaSlicer Plater.cpp 2527-2556 at
            // 31f6803). Editing nozzle_diameter alone leaves the 0.4 line
            // widths and the 0.4 process in place, so a value the printer
            // preset is not built for is refused, naming the preset to
            // choose instead; a value it already has is what it already is.
            if (nozzle_given) {
                const std::string& given = overrides.find("nozzle_diameter")->second;
                std::vector<double> current;
                if (const auto* nd = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter"))
                    current = nd->values;
                double wanted = 0.;
                bool   number = false;
                try {
                    size_t used = 0;
                    wanted = std::stod(given, &used);
                    number = used == given.size() && std::isfinite(wanted);
                } catch (...) {
                }
                if (number && !current.empty() &&
                    !std::all_of(current.begin(), current.end(),
                                 [&](double v) { return std::abs(v - wanted) < 1e-6; })) {
                    std::ostringstream has;
                    for (size_t i = 0; i < current.size(); ++i) has << (i ? ", " : "") << current[i];
                    std::ostringstream variant;
                    variant << wanted;
                    const std::string wanted_preset =
                        shipped_printer_preset_of_variant(shipped, variant.str());
                    std::string instead;
                    if (!wanted_preset.empty())
                        instead = "--printer-preset \"" + wanted_preset + "\"";
                    else if (!printer_model.empty())
                        instead = "--printer-preset \"" + printer_model + " " + variant.str() + " nozzle\"";
                    else
                        instead = "the printer preset of a " + variant.str() + " mm nozzle";
                    std::string sentence =
                        "--nozzle " + given + " does not change the printer: " +
                        (printer_preset.empty() ? std::string("this printer") : "'" + printer_preset + "'") +
                        " is built for a " + has.str() + " mm nozzle. The desktop app changes the nozzle by "
                        "choosing the printer variant, so give the printer preset of that nozzle instead: " +
                        instead + ".";
                    if (!shipped.empty())
                        sentence += " This engine ships " + preset_name_list(shipped, 4) + ".";
                    emit_event({{"event","config_refused"}, {"tag","NozzleFlagNotAPrinterVariant"},
                                {"nozzle", given}, {"printer_variant", printer_variant},
                                {"printer_preset", printer_preset}, {"message", sentence}});
                    std::cerr << "Error: " << sentence << "\n";
                    set_outcome_failure(outcome, CLI_INVALID_PARAMS, sentence);
                    return 1;
                }
            }

            // ── the plate type ──────────────────────────────────────────
            // The same desktop_bed_type the named-preset path applies, so a
            // run whose printer comes from settings files cannot land on a
            // different plate than --printer-preset does. The rule it
            // ports: the desktop sets the plate when a printer is picked
            // (OrcaSlicer Plater.cpp 2527-2563 at 31f6803, both sides of
            // its is_bbl_vendor || support_multi_bed_types test, through
            // Preset::get_default_bed_type, Preset.cpp 931-958 at 31f6803
            // — for the Snapmaker U1 the file's stated NAME "Textured PEI
            // Plate" reads as no number, which gives btPEI, the High Temp
            // Plate — and BambuStudio Plater.cpp 3340-3414, 3703-3715 at
            // 5873b5f). Without this the engine's own default is used
            // (curr_bed_type's ConfigDef default, btPC "Cool Plate":
            // BambuStudio PrintConfig.cpp 1162 at 5873b5f), which is no
            // printer's — and on the U1 it is a plate whose bed
            // temperature the shipped filament chain leaves at 0, so the
            // run is refused as unprintable at the first layer.
            if (plate_needed && !printer_preset.empty()) {
                Slic3r::BedType bed = Slic3r::btPC;
                std::string     why;
                if (desktop_plate_type_for_printer(o, printer_preset, bed, why)) {
                    config.set_key_value("curr_bed_type", new Slic3r::ConfigOptionEnum<Slic3r::BedType>(bed));
                    emit_event({{"event","presets_resolved"}, {"tag","PrinterDefaultPlate"},
                                {"printer", printer_preset},
                                {"curr_bed_type", slicer_cli::bed_type_name(bed)},
                                {"curr_bed_type_reason", why},
                                {"message","The plate type is the printer preset's own: '" +
                                               slicer_cli::bed_type_name(bed) + "' (" + why +
                                               "); give --curr-bed-type to choose another"}});
                }
            }
        }
    }

    // The official value check (OrcaSlicer.cpp 3574-3581 at 31f6803,
    // BambuStudio.cpp 4134-4141 at 5873b5f): DynamicPrintConfig::validate(true)
    // over the settings as the file and profiles state them, before this
    // driver pads any vector. Out-of-range values are named with the
    // engine's own sentence ("tree_support_wall_count: -1 not in range
    // [0,2]"); which of them refuse and which only warn is below.
    // The official check runs after the command line is applied
    // (m_print_config.apply(m_extra_config) at BambuStudio.cpp 4091,
    // OrcaSlicer.cpp 3542), so a command-line override is range-checked
    // too. The overrides themselves still land where they always have
    // (further down); here they go onto a copy for the check only, and
    // without overrides the check reads `config` as before.
    if (!inputs.calib_self_geometry) {
        // The plate's own settings are checked the same way, in the order
        // they are applied below (plate, then command line). The official
        // CLI checks m_print_config (BambuStudio.cpp 4134) before it lays
        // the plate over it (6903), so it never range-checks a plate
        // value; checking the settings a plate is sliced with is stricter
        // and changes nothing for a plate that states none.
        // Every plate the run slices: plate N, or each plate for --slice 0.
        std::vector<int> checked_plates;
        if (plate_id > 0 || plate_data.size() <= 1)
            checked_plates.push_back(plate_id);
        else
            for (int k = 1; k <= (int)plate_data.size(); ++k)
                checked_plates.push_back(k);
        for (const int checked : checked_plates) {
        const Slic3r::DynamicPrintConfig plate_settings = plate_own_settings(plate_data, checked);
        std::unique_ptr<Slic3r::DynamicPrintConfig> with_overrides;
        if (!overrides.empty() || !plate_settings.empty()) {
            with_overrides = std::make_unique<Slic3r::DynamicPrintConfig>(config);
            with_overrides->apply(plate_settings, true);
            with_overrides->apply(extra, true);
            apply_command_line_overrides(*with_overrides, overrides, /*report_rejections=*/false);
        }
        const std::map<std::string, std::string> found =
            (with_overrides ? *with_overrides : config).validate(true);
        // The desktop app runs the same check on the settings a 3MF
        // brings (config.validate(), not under the CLI) and only shows a
        // notice, "Invalid values found in the 3MF" (OrcaSlicer
        // Plater.cpp 6283-6297 at v2.4.2; BambuStudio Plater.cpp
        // 8627-8640 at 926a719); its slice is gated by Print::validate,
        // which checks each region against its own nozzle (OrcaSlicer
        // Print.cpp 1676-1690). Those findings are a warning here too, and
        // the slice's own Print::validate below stays the gate. Still
        // refused: what only the CLI check finds (the spiral-vase checks,
        // under_cli only: OrcaSlicer PrintConfig.cpp 10456-10458,
        // BambuStudio PrintConfig.cpp 9574-9576; the desktop asks the user
        // there), and what a command-line value brings (the desktop's
        // own fields refuse a value out of range as it is typed). The
        // baseline is the settings before any flag or override, with the
        // plate's own laid over: a finding there is the file's and only
        // warned; a finding the command line brings (on the flag's own
        // setting or another, as --nozzle-diameter makes a file's
        // bridge_line_width too wide) is refused.
        std::map<std::string, std::string> validity;   // refused
        std::map<std::string, std::string> noted;      // warned, as the desktop notice
        if (!found.empty()) {
            Slic3r::DynamicPrintConfig file_only(settings_before_flags);
            file_only.apply(plate_settings, true);
            const std::map<std::string, std::string> desktop = file_only.validate(false);
            for (const auto& [key, why] : found) {
                const auto it = desktop.find(key);
                // A setting given on the command line is the command
                // line's value even when the file's own was out of range
                // the same way: refused, as the desktop's field refuses it.
                const bool given_as_flag = extra.has(key) || overrides.count(key) > 0;
                (!given_as_flag && it != desktop.end() && it->second == why ? noted : validity)[key] = why;
            }
        }
        // Each plate's findings are its own: recorded once per plate
        // (every call of a --slice 0 run checks the plates again) and
        // given to that plate's result below.
        const int noted_plate = checked > 0 ? checked : 1;
        const bool noted_first_time = rs.warnings.noted.count(noted_plate) == 0;
        json& noted_warnings = rs.warnings.noted[noted_plate];
        noted_warnings = json::array();
        // Noted only when the run goes on: a refusal below says why it
        // does not, and the warning's "the run slices" would not hold.
        if (!noted.empty() && validity.empty() && unknown_values.empty()) {
            std::string sentence = "Invalid values found in the 3MF/config: ";
            json items = json::object();
            std::string flags;
            for (const auto& [key, why] : noted) {
                sentence += (items.empty() ? "" : "; ") + key + ": " + why;
                items[key] = why;
                std::string flag = "--" + key;
                std::replace(flag.begin(), flag.end(), '_', '-');
                flags += (flags.empty() ? "" : ", ") + flag;
            }
            sentence += ". The run slices with them as they are, and the slice's own check of each "
                        "object against its nozzle and plate still applies. Give " + flags +
                        " a value in range to change it.";
            if (noted_first_time) {
                const json announcement = {{"event","warning"},
                                           {"tag","InvalidValuesNoted"},
                                           {"plate_id", noted_plate},
                                           {"settings", items},
                                           {"message", sentence}};
                // Held while the check pass runs every plate: it is
                // announced once they all passed (announce_noted_warning).
                if (rs.warnings.hold_noted)
                    rs.warnings.noted_held.push_back(announcement);
                else
                    announce_noted_warning(announcement);
            }
            noted_warnings.push_back(json{{"message", sentence},
                                          {"level", "warning"},
                                          {"tag", "InvalidValuesNoted"}});
        }
        if (!validity.empty() || !unknown_values.empty()) {
            // One refusal naming every value at once: unknown enum
            // values first, then the engine's range findings.
#ifdef ENGINE_SNAPMAKER
            std::string sentence = "Snapmaker Orca cannot print this file as it is: ";
#elif defined(ENGINE_ORCA)
            std::string sentence = "OrcaSlicer cannot print this file as it is: ";
#else
            std::string sentence = "BambuStudio cannot print this file as it is: ";
#endif
            bool first = true;
            for (const std::string& u : unknown_values) {
                sentence += (first ? "" : "; ") + u;
                first = false;
            }
            json items = json::object();
            for (const auto& [key, why] : validity) {
                sentence += (first ? "" : "; ") + key + ": " + why;
                first = false;
                items[key] = why;
            }
            sentence += ".";
            // The official sentence above stays the first part of the
            // message, so anything that matches on it still matches; the
            // caller is told how to get past the refusal (a command-line
            // override is applied on top of the file's value).
            if (!items.empty()) {
                std::string flags;
                for (auto it = items.begin(); it != items.end(); ++it) {
                    std::string flag = "--" + it.key();
                    std::replace(flag.begin(), flag.end(), '_', '-');
                    flags += (flags.empty() ? "" : ", ") + flag;
                }
                sentence += " Give " + flags + " a value in range to override it.";
            }
            sentence += " --allow-substitution lets the engine use its own default for a value it does"
                        " not know instead.";
            emit_event({{"event","config_refused"},
                        {"tag", unknown_values.empty() ? "InvalidValues" : "InvalidOrUnknownValues"},
                        {"settings", items},
                        {"unknown_values", unknown_items},
                        {"message", sentence}});
            std::cerr << "Param values in 3mf/config error: " << sentence << "\n";
            set_outcome_failure(outcome, CLI_INVALID_VALUES_IN_3MF, sentence);
            return 1;
        }
        }
        // This result's plate gets its own findings, once.
        take_noted_warnings(outcome, rs.warnings.noted, plate_id);
    }

    // A project on a printer with another bed: each plate's objects and
    // tower move onto it (move_to_new_bed; translate_models runs on every
    // plate, BambuStudio.cpp 4497-4659; OrcaSlicer.cpp 3885-3980), after
    // the settings given as flags apply (BambuStudio.cpp 4091; OrcaSlicer.cpp
    // 3542), so a flag's tower position or prime tower moves too. Each
    // plate moves in its own frame (PlateScope).
    if (project_facts.is_bbl_3mf && !inputs.assemble_input && !model.objects.empty()) {
        const int first = plate_id > 0 ? plate_id : 1;
        const int last  = plate_id > 0 ? plate_id : (int)rs.layout.plates.size();
        for (int moved_plate = first; moved_plate <= last; ++moved_plate) {
            const int index = moved_plate - 1;
            RunPlate& run_plate = rs.layout.plate(moved_plate);
            // The plate's own print sequence, else the project's
            // (get_print_sequence, BambuStudio.cpp 4403-4416).
            bool is_sequence = false;
            if (const auto* seq = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence"))
                is_sequence = seq->value == Slic3r::PrintSequence::ByObject;
            if (index < (int)plate_data.size() && plate_data[index] != nullptr)
                if (const auto* own = plate_data[index]->config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                    own && own->value != Slic3r::PrintSequence::ByDefault)
                    is_sequence = own->value == Slic3r::PrintSequence::ByObject;
            Slic3r::Vec3d moved;
            {
                PlateScope scope(model, run_plate);
                shrink_to_new_bed = move_to_new_bed(context, project_facts, settings_merge.machine_switch, model, config,
                                                    index, is_sequence, moved);
            }
            if (moved.x() != 0. || moved.y() != 0.) {
                std::ostringstream text;
                text << std::fixed << std::setprecision(2) << "The plate's objects and prime tower moved by (" << moved.x()
                     << ", " << moved.y() << ") mm onto this printer's bed";
                emit_event({{"event","arranged"}, {"tag","MovedToNewBed"}, {"shrink_to_new_bed", shrink_to_new_bed},
                            {"message", text.str()}});
                // The moved tower is the project's (the official moves it in
                // m_print_config, which it exports, BambuStudio.cpp 4659,
                // 8156-8157); the flags laid over the plate below
                // (6902-6904) reach only the slice.
            }
        }
        // A plate moved onto the new printer also moves to its place in
        // the grid of the new bed size (compute_origin_using_new_size,
        // BambuStudio.cpp 4534; OrcaSlicer.cpp 3919; then reset_size):
        // translate_models moves every plate when the printer changes
        // (BambuStudio.cpp 4516) or, on Orca, when the bed changes
        // (OrcaSlicer.cpp 3902). The plate keeps the same place on itself.
#ifdef ENGINE_ORCA
        const bool translated = shrink_to_new_bed > 0;
#else
        const bool translated = settings_merge.machine_switch;
#endif
        const auto* new_area = config.option<Slic3r::ConfigOptionPoints>("printable_area");
        if (translated && new_area && new_area->values.size() >= 4 && !plate_data.empty())
            for (int index = 0; index < (int)rs.layout.plates.size() && index < (int)plate_data.size(); ++index) {
                RunPlate& run_plate = rs.layout.plates[index];
                const Slic3r::Vec2d new_origin =
                    plate_grid_origin(config, Slic3r::Semver(), index, (int)plate_data.size());
                const Slic3r::Vec3d delta(new_origin.x() - run_plate.origin.x(), new_origin.y() - run_plate.origin.y(), 0.);
                if (delta == Slic3r::Vec3d::Zero())
                    continue;
                for (Slic3r::ModelObject* object : model.objects)
                    for (Slic3r::ModelInstance* inst : object->instances)
                        if (run_plate.is_member(inst))
                            inst->set_offset(inst->get_offset() + delta);
                run_plate.origin = new_origin;
            }
    }

#ifdef ENGINE_BAMBU
    // Filament roster the project declares, captured before the BBS
    // normalization below reshapes any vector option.  It is taken from the
    // five identity keys alone, and it is the only count that stays the
    // project's: the plate overlay and the nozzle-map derivation further down
    // can rewrite filament_map, and the final alignment grows the per-filament
    // arrays to this count, so asking one of those arrays for its own length
    // later answers a different question.  The identity keys are themselves
    // never padded to the machine's nozzle count
    // (is_per_filament_config_key), so this is also the count the engine
    // derives once the config is applied.
    project_filaments = project_filament_count(config);

    // extruder_nozzle_stats: how many nozzles of each flow type each
    // extruder holds; automatic grouping reads it. The desktop writes it
    // into every project (NozzleInventory::export_to, NozzleInventory.cpp
    // 785-808 at v02.09.01.70, through full_config, PresetBundle.cpp 5061;
    // re-resolved on a printer change, select_printer 642-646), and a
    // named preset carries it (desktop_presets.cpp). Input without it
    // (settings files, an STL with --machine/--process/--filament) gets
    // the official CLI's: the printer's extruder_max_nozzle_count nozzles
    // of the nozzle_volume_type flow per extruder, a Hybrid slot taking
    // the preset's per-slot defaults (on_printer_model_change_cli,
    // BambuStudio.cpp 4536-4553), then each extruder switched to the
    // plate's flow type, the per-slot nozzle keys derived from the counts,
    // and the stats written to m_print_config on a printer with several
    // extruders or nozzles (BambuStudio.cpp 7088, 7170-7191;
    // new_nozzle_volume_type from 3804-3825). Stats the input carries are
    // read as they are (extruder_nozzle_stats_new first, 4496, 4555-4559)
    // unless the printer changed, when the official recomputes them from
    // the new printer (4530). A 2.8.x project carries the stats but none
    // of the per-slot keys, so the per-slot derivation below runs for
    // carried stats too (7173-7189).
    {
        const auto has_values = [](const Slic3r::ConfigOptionStrings* opt) {
            return opt != nullptr && std::any_of(opt->values.begin(), opt->values.end(),
                                                 [](const std::string& s) { return !s.empty(); });
        };
        const auto* stats_new = config.option<Slic3r::ConfigOptionStrings>("extruder_nozzle_stats_new");
        const auto* stats     = config.option<Slic3r::ConfigOptionStrings>("extruder_nozzle_stats");
        const Slic3r::ConfigOptionStrings* carried =
            rs.settings.settings_merge.machine_switch ? nullptr : has_values(stats_new) ? stats_new : has_values(stats) ? stats : nullptr;
        const auto* max_opt = config.option<Slic3r::ConfigOptionIntsNullable>("extruder_max_nozzle_count");
        const auto* nozzles = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter");
        if (max_opt != nullptr && !max_opt->values.empty() && nozzles != nullptr) {
            const std::vector<int> max_nozzle_count = max_opt->values;
            const size_t slot_count = max_nozzle_count.size();
            const int new_extruder_count = int(nozzles->values.size());
            const auto* opt_nvt = dynamic_cast<const Slic3r::ConfigOptionEnumsGeneric*>(config.option("nozzle_volume_type"));
            Slic3r::ExtruderNozzleStat nozzle_stats_obj;
            if (carried != nullptr) {
                // BambuStudio.cpp 4555-4559.
                nozzle_stats_obj.set_raw_stat(Slic3r::get_extruder_nozzle_stats(carried->values));
            } else {
                // BambuStudio.cpp 4536-4553.
                std::vector<int> curr_volume_map_value(slot_count, static_cast<int>(Slic3r::NozzleVolumeType::nvtStandard));
                if (opt_nvt && !opt_nvt->values.empty())
                    for (size_t idx = 0; idx < slot_count; ++idx)
                        curr_volume_map_value[idx] = idx < opt_nvt->values.size() ? opt_nvt->values[idx] : opt_nvt->values.back();
                nozzle_stats_obj.on_printer_model_change_cli(curr_volume_map_value, max_nozzle_count);
                // Hybrid only summarises mixed slots, so take the real mix
                // from the preset's per-slot defaults (4544-4553).
                for (size_t eid = 0; eid < slot_count; ++eid) {
                    if (curr_volume_map_value[eid] != int(Slic3r::NozzleVolumeType::nvtHybrid))
                        continue;
                    bool first = true;
                    for (const auto& [vt, cnt] : Slic3r::count_default_nozzle_volume_types(config, int(eid), max_nozzle_count[eid])) {
                        nozzle_stats_obj.set_extruder_nozzle_count(int(eid), vt, cnt, first);
                        first = false;
                    }
                }
            }
            // BambuStudio.cpp 3804-3825: the flow type per extruder.
            std::vector<Slic3r::NozzleVolumeType> new_nozzle_volume_type;
            if (opt_nvt && opt_nvt->values.size() >= static_cast<size_t>(new_extruder_count)) {
                for (int i = 0; i < new_extruder_count; i++)
                    new_nozzle_volume_type.push_back(Slic3r::NozzleVolumeType(opt_nvt->values[i]));
            } else {
                if (!rs.settings.settings_merge.machine_switch)
                    for (int v : project_facts.current_nozzle_volume_type)
                        new_nozzle_volume_type.push_back(Slic3r::NozzleVolumeType(v));
                new_nozzle_volume_type.resize(new_extruder_count, Slic3r::NozzleVolumeType::nvtStandard);
            }
            // BambuStudio.cpp 3804 and 7088, 7170-7191.
            const bool support_multi_nozzle = std::any_of(max_nozzle_count.begin(), max_nozzle_count.end(),
                                                          [](int v) { return v > 1; });
            if (new_extruder_count > 1 || support_multi_nozzle) {
                // An input without the full per-slot layout (extruder_nozzle_count,
                // _volume_type, _diameter, _slot) gets it from the counts: a
                // single-extruder nozzle rack keeps its count of the target
                // flow type, any other extruder keeps its total and takes the
                // plate's flow type (7173-7189).
                // The rack the command line describes wins over the input's
                // (7103-7172, cli_nozzle_options.cpp).
                const bool from_flags = slicer_cli::command_line_nozzle_rack(extra, config, new_extruder_count, nozzle_stats_obj);
                slicer_cli::sync_ams_count_with_command_line(extra, config);
                // A project moved to another printer keeps its nozzles
                // only where they hold the printer's diameter, as the
                // app keeps them (cli_nozzle_options.hpp). The official
                // command line keeps a complete per-slot layout from the
                // file as it is (BambuStudio.cpp 7173-7174). The
                // printer changes whenever the run names another one;
                // machine_switch is set only when the project's process
                // does not suit it.
                const bool printer_changed = !rs.settings_merge.new_printer_name.empty() &&
                                             rs.settings_merge.new_printer_name != project_facts.current_printer_name;
                if (!from_flags && printer_changed &&
                    slicer_cli::fit_project_nozzles_to_printer(config))
                    emit_event({{"event","config_normalized"}, {"tag","ProjectNozzlesFitToPrinter"},
                                {"message","The project's nozzles do not hold the new printer's diameter; "
                                           "those extruders take the printer's default nozzles, as the app does"}});
                const bool derived = !from_flags && !Slic3r::per_slot_nozzle_options_consistent(config);
                if (derived) {
                    for (size_t eid = 0; eid < new_nozzle_volume_type.size(); ++eid) {
                        if (Slic3r::is_single_ext_multi_nozzle(config))
                            nozzle_stats_obj.on_volume_type_switch(int(eid), new_nozzle_volume_type[eid]);
                        else if (new_nozzle_volume_type[eid] != Slic3r::NozzleVolumeType::nvtHybrid)
                            nozzle_stats_obj.set_extruder_nozzle_count(int(eid), new_nozzle_volume_type[eid],
                                                                       nozzle_stats_obj.get_extruder_nozzle_count(int(eid), std::nullopt), true);
                    }
                    const Slic3r::ExtruderFullStat full_stat = Slic3r::build_extruder_full_stat_from_nozzle_stat(
                        nozzle_stats_obj, max_nozzle_count, config_id_text(config, "printer_variant"));
                    Slic3r::sync_per_slot_nozzle_options_from_full_stat(config, full_stat);
                }
                // The official re-serialises the stats every time (7191), so
                // a project whose newer extruder_nozzle_stats_new differs
                // from its older key (a rack with a TPU nozzle) slices with
                // the newer one.
                config.option<Slic3r::ConfigOptionStrings>("extruder_nozzle_stats", true)->values =
                    Slic3r::save_extruder_nozzle_stats_to_string(nozzle_stats_obj.get_raw_stat());
            }
        }
    }
#endif

#ifdef ENGINE_BAMBU
    // Provenance is accumulated while each input/profile layer is parsed,
    // before normalization pads vector options. A supplied [1] is explicit
    // even when it equals the driver's seed value.
    if (verbose)
        std::cout << "Nozzle-map provenance: explicit config map="
                  << (explicit_config_supplied_nozzle_map ? "yes" : "no") << "\n";

    // ── BBS toolchanger / per-extruder normalizations ───────────────────
    // Everything from here to the matching #endif exists to make the Bambu
    // engine accept a non-Bambu printer config (vector-array padding/collapse,
    // explicit nozzle-map derivation, master-nozzle reassignment, prime-tower
    // disable heuristics).  OrcaSlicer handles all of this natively, so the
    // Orca driver skips the whole region and slices from the flat 3MF config.
    //
    // Pad empty vector config options and preserve 3MF multi-element values.
    //
    // The 3MF project_settings.config contains per-extruder arrays from the
    // printer profile (e.g. machine_max_acceleration_e = ["5000","5000"]).
    // BambuStudio desktop keeps these intact through Print::apply() which
    // processes them via update_values_to_printer_extruders.  The CLI must
    // NOT truncate them — the pipeline handles multi-element arrays via
    // get_at() which clamps to the last element for single-filament prints.
    //
    // Pad empty vectors to extruder_count (from nozzle_diameter size).
    {
        // Determine actual extruder count from nozzle_diameter vector.
        size_t extruder_count = 1;
        if (auto* nd = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter", false)) {
            if (!nd->values.empty())
                extruder_count = nd->values.size();
        }

        // Keys that must NOT be touched (polygon/group semantics)
        static const std::unordered_set<std::string> skip_pad = {
            "printable_area",           // bed shape polygon (4+ points)
            "bed_exclude_area",         // exclusion zones
            "thumbnails",
            "extruder_printable_area",  // per-extruder bed polygons

            // The two per-filament INDEX maps.  is_per_filament_config_key()
            // returns false for them — they are the "special" spellings that
            // get their own alignment, not plain value arrays — so the
            // predicate below does not reach them on its own.  They are per
            // filament all the same and must not be padded to the nozzle
            // count: padding is exactly how a one-filament project on a
            // two-nozzle machine grows filament_map to [1, 0], whose LENGTH
            // the engine then reads as two filaments and fans every
            // per-filament array out to (PrintConfig.cpp:8134).
            // align_per_filament_maps() extends and repairs them against the
            // roster instead.  (filament_nozzle_map, the third index map, is a
            // plain per-filament key, so the predicate already covers it.)
            "filament_map",
            "filament_map_2",

            // A list of script paths, not a value per extruder. Padding it
            // names an empty script, and a project exported with that entry
            // is refused by the official command line's normative check
            // (BambuStudio.cpp 1948-1956).
            "post_process",

            // Not one value per extruder either; the official CLI leaves
            // each as the settings give it, and a padded 0 is read:
            //  - wipe_tower_x / wipe_tower_y: one entry per plate, read
            //    with get_at(plate index) (Print.cpp 998, 2213, 2662),
            //    which falls back to the first entry (Config.hpp 681-685). A 0 put plate
            //    2's tower at (0, 0).
            //  - flush_multiplier / flush_multiplier_fast: one per nozzle;
            //    when the official sizes them it fills 1.0 / 1.2
            //    (BambuStudio.cpp 3864-3868; OrcaSlicer.cpp 3360-3361),
            //    and get_at() gives a short list's first value. A 0 is no
            //    flush on the second nozzle.
            //  - first_layer_print_sequence / other_layers_print_sequence:
            //    filament orders; more than one entry is read as a custom
            //    first-layer order (ToolOrdering.cpp 467-468, 597-610).
            "wipe_tower_x",
            "wipe_tower_y",
            "flush_multiplier",
            "flush_multiplier_fast",
            "first_layer_print_sequence",
            "other_layers_print_sequence",
        };

        // Per-filament arrays are excluded from this padding altogether: the
        // plain ones by the same predicate the final alignment uses, the two
        // index maps by skip_pad above.  Their LENGTH is the
        // engine's filament count, not the head count: Print::apply() reads its
        // num_extruders from filament_diameter (PrintApply.cpp:1438),
        // ToolOrdering sizes its filament loops and the per-nozzle flush
        // geometry from filament_colour / filament_diameter (ToolOrdering.cpp:509,
        // 2503; prepare_flush_matrices:1421), Print::has_wipe_tower()
        // (Print.cpp:3113) tests filament_diameter.size() > 1 directly, and the
        // multi-filament fan-out reads filament_map's length
        // (PrintConfig.cpp:8134).  Padding a one-filament project's
        // per-filament arrays up to a two-nozzle machine's nozzle count
        // invents a filament slot the project never declared — with a zero
        // diameter, no colour and no type — and everything sized from that
        // count downstream (wipe tower, per-nozzle flush matrix, G-code
        // metadata) is then sized for it.  Left at their declared length, the
        // engine sees the roster the project actually loaded, and the final
        // central alignment extends them to exactly that count.
        //
        // A zero spliced in here would also be sticky: the alignment only ever
        // duplicates the FRONT value, so a padded [22, 0] becomes [22, 0, 22]
        // for a three-filament roster instead of [22, 22, 22].
        //
        // The variant-slot arrays (filament_extruder_variant,
        // filament_self_index) are not per-filament and keep the extruder-count
        // padding they have always had — they belong to the printer-side slot
        // table (PrintConfig.cpp:7592).
        //
        // Only pad empty vectors to extruder_count; do NOT truncate.
        // Variant-expanded arrays (retraction_length, machine_max_*, speed, etc.)
        // are collapsed by PrintApply.cpp's update_values_to_printer_extruders()
        // when (extruder_count > 1) || different_extruder.
        // support_different_extruders() returns true when extruder_variant_list
        // has multiple comma-separated variants (e.g. "Direct Drive Standard,
        // Direct Drive High Flow"), which is the normal case for BBL printers.
        // Truncating here would bypass that collapsing and produce wrong sizes.
        for (const auto& key : config.keys()) {
            if (skip_pad.count(key))
                continue;

            // Every filament setting (is_filament_setting_key, which holds
            // all the keys the final alignment treats as a roster) is left
            // alone, so this pass can never pad an array the alignment then
            // treats as a roster.  The index maps the predicate excludes are
            // in skip_pad above.
            if (is_filament_setting_key(key))
                continue;

            auto* opt = config.option(key, false);
            if (!opt) continue;

#define NORM_VEC(Type, zero_val) \
            if (auto v = dynamic_cast<Slic3r::Type*>(opt)) { \
                while (v->values.size() < extruder_count) v->values.push_back(zero_val); \
            } else
            NORM_VEC(ConfigOptionBoolsNullable,          (unsigned char)0)
            NORM_VEC(ConfigOptionBools,                  (unsigned char)0)
            NORM_VEC(ConfigOptionIntsNullable,           0)
            NORM_VEC(ConfigOptionInts,                   0)
            NORM_VEC(ConfigOptionFloatsNullable,         0.0)
            NORM_VEC(ConfigOptionFloats,                 0.0)
            NORM_VEC(ConfigOptionPercentsNullable,       0.0)
            NORM_VEC(ConfigOptionPercents,               0.0)
            NORM_VEC(ConfigOptionFloatsOrPercentsNullable, (Slic3r::FloatOrPercent{0.0, false}))
            NORM_VEC(ConfigOptionFloatsOrPercents,       (Slic3r::FloatOrPercent{0.0, false}))
            NORM_VEC(ConfigOptionStrings,                std::string{})
            NORM_VEC(ConfigOptionEnumsGenericNullable,   0)
            NORM_VEC(ConfigOptionEnumsGeneric,           0)
            { /* ConfigOptionPoints, ConfigOptionPointsGroups, etc. — leave alone */ }
#undef NORM_VEC
        }

        // extruder_variant_list contains comma-separated variant names per slot
        // (e.g. "Direct Drive Standard,Direct Drive High Flow").
        // support_different_extruders() splits these and detects multiple variants,
        // triggering update_values_to_printer_extruders() in PrintApply.cpp.
        // Do NOT truncate this list — it must remain as-is for correct collapsing.

        // Clamp master_extruder_id to extruder_count (1-indexed).
        // ToolOrdering::get_recommended_filament_maps() uses master_extruder_id-1
        // as an index into nozzle_list, which has extruder_count entries.
        if (auto* mid = config.option<Slic3r::ConfigOptionInt>("master_extruder_id", false)) {
            if (mid->value < 1) mid->value = 1;
            if (mid->value > (int)extruder_count) mid->value = (int)extruder_count;
        }
    }

    // Ensure vector options after loading JSON configs and forcing single-extruder
    // JSON configs may have left some vectors empty or missing
    ensure_vector_config_sizes(config);
#endif // ENGINE_BAMBU — the run's settings base

    // The run's settings base, m_print_config (see RunState): everything
    // above runs once per run.
    rs.settings.base = config;
    rs.project.loaded = true;
    return 0;
}

} // namespace slicer_cli
