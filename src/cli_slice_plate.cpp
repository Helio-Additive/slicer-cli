#include "cli_slice_plate.hpp"

#include "cli_run_context.hpp"
#include "cli_slice_state.hpp"
#include "cli_load_slice_inputs.hpp"
#include "cli_plate_scope.hpp"
#include "cli_options.hpp"
#include "cli_assemble.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Format/STL.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/GCode/WipeTower.hpp"
#include "libslic3r/BuildVolume.hpp"
#include <set>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/nowide/fstream.hpp>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <limits>
#include "calib_args.hpp"
#include "cli_arrange_geometry.hpp"
#include "cli_engine_events.hpp"
#include "cli_engine_info.hpp"
#include "cli_failure_reason.hpp"
#include "cli_filament_config.hpp"
#include "cli_filament_map.hpp"
#include "cli_layer_ranges.hpp"
#include "cli_legacy_gcode.hpp"
#include "cli_load_settings.hpp"
#include "cli_model_load.hpp"
#include "cli_named_presets.hpp"
#include "cli_nozzle_diameter.hpp"
#include "cli_nozzle_options.hpp"
#include "cli_percent_widths.hpp"
#include "cli_pipe.hpp"
#include "cli_plate_arrange.hpp"
#include "cli_plate_plan.hpp"
#include "cli_plate_report.hpp"
#include "cli_project_archive.hpp"
#include "cli_project_load.hpp"
#include "cli_repetitions.hpp"
#include "cli_result_json.hpp"
#include "cli_run_cleanup.hpp"
#include "cli_run_steps.hpp"
#include "cli_setting_overrides.hpp"
#include "cli_simplify.hpp"
#include "cli_slice_export.hpp"
#include "cli_snapmaker.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "diagnostic_output.hpp"
#ifdef ENGINE_BAMBU
#include "libslic3r/FilamentMixer.hpp"
#endif

namespace slicer_cli {

using json = nlohmann::json;

/// One plate's objects, loaded alone from the project 3MF, for
/// --downward-check: the official sizes every plate of the project
/// (check_plate_wipe_tower per plate, BambuStudio.cpp 4645-4658; OrcaSlicer.cpp
/// 3986-3996), whichever plate is sliced. Plate 0 is the whole project;
/// `plate_members`, when given, receives each plate's (object, instance)
/// pairs in the loaded model (PlateData::objects_and_instances), and
/// `file_config` the project's settings as the file holds them.
static bool load_plate_objects(CliRunContext& context, const std::string& input_file, int plate_id, Slic3r::Model& model,
                               std::vector<std::vector<std::pair<int, int>>>* plate_members = nullptr,
                               Slic3r::DynamicPrintConfig* file_config = nullptr) {
    using namespace Slic3r;
    DynamicPrintConfig scratch;
    scratch.apply(FullPrintConfig::defaults(), true);
    ConfigSubstitutionContext subst(ForwardCompatibilitySubstitutionRule::Enable);
    PlateDataPtrs plates;
    std::vector<Preset*> presets;
    bool is_bbl = false;
    Semver version;
    std::string load_path = input_file;
#ifdef ENGINE_BAMBU
    PercentRewrite percent_rewrite;
    rewrite_percent_line_widths(input_file, percent_rewrite);
    if (!percent_rewrite.refusal.empty())
        return false;
    if (percent_rewrite.changed)
        load_path = percent_rewrite.temp_path;
#endif
    const auto strategy = LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::AddDefaultInstances;
    // A writable backup folder of this run's own, as the slice's own load
    // sets (never the loader's default /bamboo_model, and never one shared
    // with another run: run_backup_path), with the archive's directories in it
    // so the extraction finds them, and detached again when this function
    // ends: this model is a temporary one, and its destruction must not remove
    // the folder the run's own load extracts into (detach_run_backup_path).
    set_run_backup_path(context, model);
    create_archive_dirs(load_path, run_backup_path(context));
    const RunBackupDetach detach_model{model};
    bool loaded = false;
    try {
#ifdef ENGINE_ORCA
#ifdef ENGINE_SNAPMAKER
        // Snapmaker's load_bbs_3mf takes no is_orca_3mf flag (Format/bbs_3mf.hpp
        // at bac6fd5), like BambuStudio's.
        loaded = load_bbs_3mf(load_path.c_str(), &scratch, &subst, &model, &plates, &presets, &is_bbl,
                              &version, nullptr, strategy, nullptr, plate_id);
#else
        bool is_orca = false;
        loaded = load_bbs_3mf(load_path.c_str(), &scratch, &subst, &model, &plates, &presets, &is_bbl, &is_orca,
                              &version, nullptr, strategy, nullptr, plate_id);
#endif
#else
        loaded = load_bbs_3mf(load_path.c_str(), &scratch, &subst, &model, &plates, &presets, &is_bbl,
                              &version, nullptr, strategy, nullptr, plate_id);
#endif
    } catch (const std::exception&) {
        loaded = false;
    }
    if (plate_members) {
        // The loader keeps a plate's instances only as obj_inst_map and
        // stamps each loaded instance with its identify_id (bbs_3mf.cpp
        // 2411-2445 at 5873b5f); an instance tied to no plate goes to the
        // first plate its box meets (PartPlateList::reload_all_objects ->
        // PartPlate::intersect_instance, PartPlate.cpp 5668-5710, 2651-2677),
        // as export_sliced_3mf finds them.
        plate_members->assign(plates.size(), {});
        std::set<std::pair<int, int>> placed;
        for (size_t k = 0; loaded && k < plates.size(); ++k) {
            if (plates[k] == nullptr) continue;
            std::set<int> identify_ids;
            for (const auto& entry : plates[k]->obj_inst_map)
                if (entry.second.second > 0) identify_ids.insert(entry.second.second);
            for (size_t oi = 0; oi < model.objects.size(); ++oi)
                for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii) {
                    const int id = model.objects[oi]->instances[ii]->loaded_id;
                    if (id > 0 && identify_ids.count(id) && placed.insert({int(oi), int(ii)}).second)
                        (*plate_members)[k].emplace_back(int(oi), int(ii));
                }
        }
        for (size_t oi = 0; loaded && oi < model.objects.size(); ++oi)
            for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii) {
                if (placed.count({int(oi), int(ii)})) continue;
                const BoundingBoxf3 box = model.objects[oi]->instance_convex_hull_bounding_box(ii);
                for (size_t k = 0; k < plates.size(); ++k)
                    if (plate_scene_box(scratch, version, int(k), int(plates.size())).intersects(box)) {
                        (*plate_members)[k].emplace_back(int(oi), int(ii));
                        break;
                    }
            }
    }
    if (file_config)
        *file_config = scratch;
    release_PlateData_list(plates);
    for (Preset* p : presets)
        delete p;
    return loaded;
}

/// Events held while it lives (diagnostics::held_events): dropped when the
/// step they come from passes (drop()), sent when it stops early.
class HeldEventsScope {
public:
    explicit HeldEventsScope(bool active) : m_active(active) {
        if (!m_active) return;
        m_prev = slicer_cli::diagnostics::held_events;
        slicer_cli::diagnostics::held_events = &m_held;
    }
    void drop() { m_held.clear(); }
    ~HeldEventsScope() {
        if (!m_active) return;
        slicer_cli::diagnostics::held_events = m_prev;
        for (const std::string& line : m_held)
            slicer_cli::diagnostics::write_event(line);
    }
    HeldEventsScope(const HeldEventsScope&) = delete;
    HeldEventsScope& operator=(const HeldEventsScope&) = delete;
private:
    bool m_active = false;
    std::vector<std::string>* m_prev = nullptr;
    std::vector<std::string> m_held;
};

static void pipe_files_loaded(RunState& rs) {
    if (!slicer_cli::pipe_started() || rs.pipe_files_loaded_sent)
        return;
    rs.pipe_files_loaded_sent = true;
    slicer_cli::pipe_update(2, "Loading files finished");
}
static void pipe_prepare_slicing(RunState& rs) {
    pipe_files_loaded(rs);
    if (!slicer_cli::pipe_started() || rs.pipe_prepare_sent)
        return;
    rs.pipe_prepare_sent = true;
    slicer_cli::pipe_update(3, "Prepare slicing");
}

/// --skip-objects on the placed objects (BambuStudio.cpp 6541-6581,
/// CLI_NO_SUITABLE_OBJECTS_AFTER_SKIP when nothing is left; OrcaSlicer.cpp
/// 5645-5720): inside the slice action, after the model actions. Its caller
/// runs --ensure-on-bed first, before the actions (BambuStudio.cpp 6188-6194
/// then the action loop at 6336; OrcaSlicer.cpp 5443-5455 then 5470).
/// --mtcpp follows the bed check (plate_triangle_limit).
static bool plate_object_steps(const CliOptions& o, Slic3r::Model& model, int plate, PlateOutcome& outcome) {
    std::vector<int> skipped;
    if (slicer_cli::apply_skip_objects(o, model, skipped) < 0) {
        const std::string why = "Every object on plate " + std::to_string(plate) + " is in --skip-objects.";
        std::cerr << "Error: " << why << "\n";
        set_outcome_failure(outcome, CLI_NO_SUITABLE_OBJECTS_AFTER_SKIP, why);
        return false;
    }
    outcome.skipped_objects = skipped;
    return true;
}

/// --mtcpp, once the bed check has marked each instance inside or not
/// (BambuStudio.cpp 6541-6597; OrcaSlicer.cpp 5657-5720).
static bool plate_triangle_limit(const CliOptions& o, const Slic3r::Model& model, int plate, PlateOutcome& outcome) {
    const slicer_cli::StepResult tri = slicer_cli::check_triangle_limit(o, model, plate);
    if (tri.code != 0) {
        std::cerr << "Error: " << tri.message << "\n";
        set_outcome_failure(outcome, tri.code, tri.message);
        return false;
    }
    return true;
}

#ifndef ENGINE_SNAPMAKER
/// --filament-map-mode and --filament-map: the command line's mode and map win
/// over the plate's and the project's, as in the official CLIs: the mode from
/// the command line before the plate's (BambuStudio.cpp 7095-7099 at
/// v02.09.01.70; OrcaSlicer.cpp 5851-5854 at 8500fcd), the map set on the plate
/// (7208-7249; 5889-5929) and laid over the print config (7434; 5989). In
/// Manual each engine prints a filament with the extruder its map names (Bambu
/// ToolOrdering.cpp 2653-2680; Orca ToolOrdering.cpp 1288-1305, which regroups
/// only in an automatic mode). --filament-map with no mode means Manual: the
/// official CLI would keep the plate's mode and, in an automatic one, regroup
/// over the map. On the Bambu build a manual mode also runs the official
/// rack checks and fills (cli_nozzle_options.cpp), and on a printer with one
/// extruder and a nozzle rack the map is Bambu Studio's dialog grouping.
/// Snapmaker Orca's plate carries no map at bac6fd5, so that build refuses
/// both flags (cli_flag_support.cpp).
/// ModeOnly: the mode is set and the plate's own map still applies.
enum class FilamentMapFlag { NotGiven, Applied, ModeOnly, Refused };
static FilamentMapFlag apply_filament_map_flag(const CliOptions& o, Slic3r::DynamicPrintConfig& extra,
                                               Slic3r::DynamicPrintConfig& config, RunState& rs,
                                               std::optional<Slic3r::DynamicPrintConfig>& project_settings,
                                               size_t project_filaments, int plate_id, PlateOutcome& outcome) {
    // The checks and the maps are cli_filament_map.cpp's; this applies what
    // they decided to the run's own records.
    const slicer_cli::FilamentMapFlagStep step =
        slicer_cli::filament_map_flag_step(extra, config, project_filaments, !(o.slice_mode && o.slice_plate == 0));
    using Kind = slicer_cli::FilamentMapFlagStep::Kind;
    // The exported plate states the volume and nozzle maps the flags gave (or
    // the step filled), not the ones the plate was saved with.
    if (step.kind != Kind::Refused && o.slice_mode && !o.export_3mf.empty())
        for (const char* key : {"filament_volume_map", "filament_nozzle_map"})
            if (const auto* map = extra.option<Slic3r::ConfigOptionInts>(key))
                outcome.flag_plate_maps[key] = map->values;
    if (step.kind == Kind::NotGiven)
        return FilamentMapFlag::NotGiven;
    if (step.kind == Kind::Refused) {
        std::cerr << "Error: " << step.refusal << "\n";
        set_outcome_failure(outcome, CLI_INVALID_PARAMS, step.refusal);
        return FilamentMapFlag::Refused;
    }
    // The project's settings state the mode with the map they already carry:
    // --export-settings writes them (BambuStudio.cpp 6366-6370; OrcaSlicer.cpp
    // 5499-5503), and a map saved with an automatic mode is regrouped on the
    // next slice.
    Slic3r::ConfigSubstitutionContext subst(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
    // With the mode go the maps the slice runs with: the map as the step
    // normalised it (a nozzle rack's grouping is extruder 1 for every
    // filament) and the nozzle and volume maps it filled.
    const auto state_maps = [&](Slic3r::DynamicPrintConfig& settings) {
        settings.set_deserialize("filament_map_mode", step.mode_name, subst);
        if (step.kind != Kind::Applied || !step.manual)
            return;
        settings.option<Slic3r::ConfigOptionInts>("filament_map", true)->values = step.map;
        for (const char* key : {"filament_volume_map", "filament_nozzle_map"})
            if (const Slic3r::ConfigOption* map = extra.option(key))
                settings.set_key_value(key, map->clone());
    };
    state_maps(rs.base);
    if (project_settings)
        state_maps(*project_settings);
    // The exported project's plate states the mode, and in a manual mode the
    // map, the slice used (store_bbs_3mf writes a plate's mode and map,
    // bbs_3mf.cpp).
    if (o.slice_mode && !o.export_3mf.empty()) {
        outcome.derived_filament_map_mode = step.mode_name;
        if (step.kind == Kind::Applied && step.manual)
            outcome.derived_filament_map = step.map;
        else
            outcome.derived_filament_map.clear();
    }
    if (step.kind == Kind::ModeOnly) {
        emit_event({{"event","config_normalized"}, {"tag","FilamentMapModeFromCommandLine"},
                    {"plate_id", plate_id}, {"filament_map_mode", step.mode_name}, {"message", step.message}});
        return FilamentMapFlag::ModeOnly;
    }
    emit_event({{"event","config_normalized"}, {"tag","FilamentMapFromCommandLine"},
                {"plate_id", plate_id}, {"filament_map", step.map}, {"filament_map_mode", step.mode_name},
                {"message", step.message}});
    return FilamentMapFlag::Applied;
}
#endif // ENGINE_SNAPMAKER

#ifdef ENGINE_BAMBU
/// The official check of a manual map against filament_printable
/// (cli_nozzle_options.cpp), on the objects the model holds now: the caller
/// scopes it to the plate and runs it after the object exclusions.
static bool manual_map_extruders_fit(const Slic3r::DynamicPrintConfig& config, const Slic3r::Model& model, int plate_id,
                                     PlateOutcome& outcome) {
    const std::string why =
        slicer_cli::manual_map_extruder_refusal(config, arrange_plate_extruders(model, config, std::max(0, plate_id - 1)));
    if (why.empty())
        return true;
    std::cerr << "Error: " << why << "\n";
    set_outcome_failure(outcome, CLI_FILAMENTS_NOT_SUPPORTED_BY_EXTRUDER, why);
    return false;
}
#endif

/// One slice of one plate from the run's one model and settings (RunState):
/// the first call of a run loads the inputs and builds the settings base
/// (`rs.loaded`) and runs the steps the official CLI runs once before its
/// plate loop (`rs.prepared`: the transforms, the arrange, the actions before
/// --slice); every call then lays the plate's own settings over a copy of the
/// base and slices the plate with its origin, as the official plate loop does
/// (BambuStudio.cpp 6437-7260; OrcaSlicer.cpp 5561-6200). The default call
/// (`file [--plate N] -o out.gcode`) is one call; `--slice` makes a prepare
/// call (outcome.prepare_pass, which stops before the plate checks) and then
/// one call per plate. The return value is the process exit code of the
/// default call; `outcome` carries what `--slice` mode reports on top.
int slice_one_plate(CliRunContext& context, const CliOptions& o, Slic3r::Calib_Params& calib_params, RunState& rs,
                           int plate_id, const std::string& output_file, PlateOutcome& outcome) {
    const std::string& input_file      = o.input_file;
    const bool verbose                 = o.verbose;
    const bool normalize_legacy_gcode  = o.normalize_legacy_gcode;
    const auto& overrides              = o.overrides;
#ifdef ENGINE_SNAPMAKER
    // The engine writes debug SVGs into data_dir()/"SVG" on some slicing paths
    // (references/SnapmakerOrca/src/libslic3r/utils.cpp:302-320 at bac6fd5, where
    // create_directory throws if the parent is gone). The data directory can be
    // the named-preset staging folder, which is removed once staging ends, so a
    // run that switched printer stopped there. Give such a run a data directory
    // that exists for as long as the slice does.
    slicer_cli::snapmaker_keep_data_dir(run_backup_path(context));
#endif
    const bool calib_enabled       = calib_params.mode != Slic3r::CalibMode::Calib_None;
    const bool calib_self_geometry = slicer_cli::calib_mode_generates_geometry(calib_params.mode);
    const auto plate_started = std::chrono::steady_clock::now();
    // The first call of the run loads and prepares; the others slice only.
    const bool first_call = !rs.prepared;
    if (!first_call) {
        // What the load and the run-level steps recorded, for this plate's result.
        PlateOutcome carried = rs.carry;
        carried.plate_id    = outcome.plate_id;
        carried.plate_index = outcome.plate_index;
        carried.plate_count = outcome.plate_count;
        carried.gcode_path  = outcome.gcode_path;
        carried.pre_check   = outcome.pre_check;
        if (plate_id != rs.prepared_plate) {
            // Another plate than the one the run-level steps ran for: its own
            // move to the new bed and none of that plate's arrange results.
            carried.derived_filament_map.clear();
            carried.derived_filament_map_mode.clear();
            carried.flag_plate_maps.clear();
        }
        outcome = carried;
        // The carry is the prepare call's outcome, with that plate's settings
        // warnings: this plate gets its own instead.
        take_noted_warnings(outcome, rs.noted, plate_id);
    }
    try {
        // Create configuration BEFORE model loading so load_bbs_3mf can populate it
        // from the embedded Metadata/project_settings.config JSON.
        PlateLoadSettings load_settings;
        auto& config = load_settings.config;
        Slic3r::Model& model = rs.model;
        bool& is_bbl_3mf = rs.is_bbl_3mf;
        Slic3r::PlateDataPtrs& plate_data = rs.plate_data;
        // Each plate's grid origin in the project (plate_grid_origin), for
        // the plates --downward-check loads alone; empty when not a project.
        std::vector<Slic3r::Vec2d>& plate_origins = rs.plate_origins;
        // A project 3MF with model files after it (ProjectPlan): the file's own
        // settings and version, for the plate list's size, and its filament
        // count, the used filaments of the official's tower step.
        std::optional<Slic3r::DynamicPrintConfig>& plan_file_config = rs.plan_file_config;
        Slic3r::Semver& plan_file_version = rs.plan_file_version;
        int& plan_file_filaments = rs.plan_file_filaments;
        // What the official CLI reads from a Bambu-made 3MF, and its embedded
        // presets, for --load-settings and friends (cli_load_settings.cpp).
        slicer_cli::ProjectFacts& project_facts = rs.project_facts;
        // The settings given as flags; the settings merge takes filament_colour
        // out of them once it has used it, as the official does.
        Slic3r::DynamicPrintConfig& extra = rs.extra;
#ifdef ENGINE_BAMBU
        bool& explicit_config_supplied_nozzle_map = rs.explicit_config_supplied_nozzle_map;
#endif

        // Geometry files (every model file this engine reads that is not a
        // project 3MF) load the official CLI's way, placed further down once
        // the bed is known. Any other file keeps the unsupported-format
        // refusal below.
        const bool geometry_input = !input_file.empty() && slicer_cli::is_loadable_model_file(input_file) &&
            (!input_is_3mf(input_file) ||
             slicer_cli::classify_3mf(input_file) == slicer_cli::ThreeMfKind::GeometryOnly);
        const bool geometry_only_3mf = geometry_input && input_is_3mf(input_file);
        // A project 3MF with model files after it, run as the official CLI
        // runs it (ProjectPlan): the run loads every plate (plate_id 0) and
        // the model files into the one model and arranges them; each plate's
        // slice then takes its instances from the plan.
        const bool trailing_port = !geometry_input && context.g_assemble == nullptr && trailing_models_port(o);
        // --load-assemble-list: the plates' objects from the list built
        // before the plate loop, and every plate's own settings (plate_params)
        // as its PlateData config, where the official keeps them
        // (BambuStudio.cpp 1066-1078; OrcaSlicer.cpp 877-889).
        const bool assemble_input = context.g_assemble != nullptr;
        // The settings base the plate's own are laid over (below), and the
        // run-level part of the settings and of the model: once per run.
        std::optional<Slic3r::DynamicPrintConfig> project_settings;
        int& shrink_to_new_bed = rs.shrink_to_new_bed;
        slicer_cli::SettingsMerge& settings_merge = rs.settings_merge;
        std::string& merged_print_sequence = rs.merged_print_sequence;
#ifdef ENGINE_BAMBU
        size_t& project_filaments = rs.project_filaments;
#endif
        if (!rs.loaded) {
            const PlateLoadInputs inputs{geometry_input, geometry_only_3mf, trailing_port, assemble_input, calib_self_geometry};
            const int rc = load_slice_inputs(context, o, rs, plate_id, inputs, load_settings, outcome);
            if (rc != 0)
                return rc;
        }

        // This plate's settings: a copy of the base with the plate's own and
        // the flags laid on top (BambuStudio.cpp 6902-6904; OrcaSlicer.cpp
        // 5905-5907). The plate the run-level steps ran for keeps the settings
        // those steps read and changed (the arrange's tower).
        const bool fresh_plate_config = !(rs.prepared && plate_id == rs.prepared_plate);
        config = fresh_plate_config ? rs.base : rs.run_config;
        // --export-settings writes the project's settings, not this plate's:
        // the official action saves m_print_config before the plate loop
        // applies *part_plate->config() (BambuStudio.cpp 6366-6370 and
        // 6902-6904; OrcaSlicer.cpp 5499-5503 and 5905-5907).
        if (slicer_cli::has_model_actions(o))
            project_settings = rs.base;
        if (fresh_plate_config) {
        // The prepare call of a run of several plates lays no plate's own
        // settings: its settings serve the run-level steps only, and each
        // plate reports its own when it is sliced.
        HeldEventsScope run_config_events(outcome.prepare_pass && plate_id == 0 &&
                                          (trailing_port || plate_data.size() > 1));
#ifdef ENGINE_BAMBU
        // If the fully-overlaid config carries an explicit per-filament nozzle assignment,
        // apply it directly to config's filament_map.  This reproduces the
        // BambuStudio desktop behaviour where the user has already constrained the
        // mapping in the GUI ("Nozzle Manual" mode) and saved it in the plate metadata.
        //
        // Explicit is detected when EITHER:
        //   (a) plate filament_maps have multiple distinct values (e.g. [2,1]), OR
        //   (b) the project config says "Nozzle Manual" (handles uniform maps like [2,2])
        //
        // When the maps are uniform AND mode is "Auto For Flush" (e.g. [1,1]),
        // we fall through to apply_explicit_nozzle_mapping() which re-derives the map
        // from filament_nozzle_map using the physical_extruder_map inverse.
        int plate_data_idx = (plate_id > 0 && (int)plate_data.size() >= plate_id) ? plate_id - 1 : 0;
        bool explicit_plate_mapping_applied = false;
        // --filament-map-mode and --filament-map (apply_filament_map_flag).
        // --filament-map alone is Manual, not Nozzle Manual, on the H2C as
        // well: Manual groups by filament_map and checks the result against it
        // (ToolOrdering.cpp 2653-2680 at v02.09.01.70), Nozzle Manual reads
        // filament_nozzle_map instead (2565-2623).
        bool cli_mode_given = false;
        switch (apply_filament_map_flag(o, extra, config, rs, project_settings, project_filaments, plate_id, outcome)) {
        case FilamentMapFlag::Refused:
            return 1;
        case FilamentMapFlag::Applied: {
            // filament_map_2, the 0-based mirror, follows the map.
            const std::vector<int>& pm = config.option<Slic3r::ConfigOptionInts>("filament_map")->values;
            auto* fm2 = config.option<Slic3r::ConfigOptionInts>("filament_map_2", true);
            fm2->values.resize(pm.size());
            for (size_t i = 0; i < pm.size(); ++i)
                fm2->values[i] = pm[i] - 1;
            explicit_plate_mapping_applied = true;
            cli_mode_given = true;
            break;
        }
        case FilamentMapFlag::ModeOnly:
            cli_mode_given = true;
            break;
        case FilamentMapFlag::NotGiven:
            break;
        }
        if (!explicit_plate_mapping_applied && !plate_data.empty() && plate_data[plate_data_idx] != nullptr) {
            const auto& pm = plate_data[plate_data_idx]->filament_maps;
            // The plate's own mode, else the project's
            // (PartPlate::get_real_filament_map_mode, PartPlate.cpp 278-289;
            // BambuStudio.cpp 6661-6665), applied over the print config as the
            // official CLI applies the plate config (BambuStudio.cpp 6905-6907).
            // The stored plate map is used only in a manual mode: under an
            // automatic mode the engine groups the filaments itself
            // (ToolOrdering.cpp 1897-1914), whatever map an earlier slice saved.
            Slic3r::FilamentMapMode real_mode = Slic3r::FilamentMapMode::fmmAutoForFlush;
            if (auto* mode_opt = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode", false))
                real_mode = mode_opt->value;
            if (auto* plate_mode = plate_data[plate_data_idx]->config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode");
                !cli_mode_given && plate_mode && plate_mode->value != Slic3r::FilamentMapMode::fmmDefault) {
                real_mode = plate_mode->value;
                config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode", true)->value = real_mode;
            }
            const bool is_manual_mode = real_mode == Slic3r::FilamentMapMode::fmmManual ||
                                        real_mode == Slic3r::FilamentMapMode::fmmNozzleManual;
            // Any saved map in a manual mode, one slot included: the official
            // CLI slices a manual plate with get_real_filament_maps(), the
            // plate's own map whenever it has one (BambuStudio.cpp 6751;
            // PartPlate.cpp 266-276), whatever its length.
            bool has_explicit = !pm.empty() && is_manual_mode;
            if (has_explicit) {
                auto* fm = config.option<Slic3r::ConfigOptionInts>("filament_map", true);
                fm->values = pm;  // already 1-based per PlateData docs
                // The plate's nozzle-volume map with it, as the official
                // plate config carries both (bbs_3mf.cpp 4612-4624).
                if (const auto* pvm = plate_data[plate_data_idx]->config.option<Slic3r::ConfigOptionInts>("filament_volume_map");
                    pvm && pvm->values.size() == pm.size())
                    config.option<Slic3r::ConfigOptionInts>("filament_volume_map", true)->values = pvm->values;

                // Also sync filament_map_2 (0-based mirror used by some code paths)
                auto* fm2 = config.option<Slic3r::ConfigOptionInts>("filament_map_2", true);
                fm2->values.resize(pm.size());
                for (size_t i = 0; i < pm.size(); ++i)
                    fm2->values[i] = pm[i] - 1;

                // `Nozzle Manual` drives the multi-nozzle grouping branch.  H2D is
                // instead a multi-extruder machine (one nozzle per extruder), where
                // `Manual` is the branch that preserves this already-resolved plate
                // map through ToolOrdering.  Derive the mode from the configured
                // topology rather than a printer identity.
                bool has_multiple_nozzles_per_extruder = false;
                if (auto* counts = config.option<Slic3r::ConfigOptionInts>("extruder_max_nozzle_count", false)) {
                    has_multiple_nozzles_per_extruder = std::any_of(
                        counts->values.begin(), counts->values.end(),
                        [](int count) { return count > 1; });
                }
                // The command line's mode stays as given.
                Slic3r::ConfigSubstitutionContext subst(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
                if (!cli_mode_given)
                    config.set_deserialize(
                        "filament_map_mode",
                        has_multiple_nozzles_per_extruder ? "Nozzle Manual" : "Manual",
                        subst);
                explicit_plate_mapping_applied = true;
            }
        }

        // Fix the cardinality of the per-filament maps BEFORE the derivation reads
        // them.  apply_explicit_nozzle_mapping() takes filament_map's own length as
        // the filament count, so a map still holding the driver's single seed entry
        // — a project whose filament_map is a placeholder while its
        // filament_nozzle_map carries the real cross-nozzle split — would look like
        // a one-filament project and skip a routing the file declares.  This is the
        // same helper the final alignment calls, which is what keeps the maps
        // cardinality-correct after the plate overlay above and the command-line
        // overrides below have had their chance to rewrite them.
        align_per_filament_maps(config, project_filaments);

        // The selected plate already expresses physical routing when it carries
        // a diverse map. Do not re-derive that routing from the general
        // filament_nozzle_map fallback, which is for an otherwise-auto mapping.
        // Nor when the command line names an automatic mode: the engine
        // groups the filaments itself, and a manual map derived here would
        // be the one an exported project states.
        bool cli_automatic_mode = false;
        if (cli_mode_given)
            if (const auto* mode = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode"))
                cli_automatic_mode = mode->value != Slic3r::FilamentMapMode::fmmManual &&
                                     mode->value != Slic3r::FilamentMapMode::fmmNozzleManual;
        bool nozzle_mapping_derived =
            (explicit_plate_mapping_applied || cli_automatic_mode) ? false : apply_explicit_nozzle_mapping(
                config, explicit_config_supplied_nozzle_map, verbose);

        // The derived routing is the plate's (filament_map + its mode); every
        // object keeps its own filament, as in Bambu Studio and OrcaSlicer,
        // which never move an object to another filament for routing.
        if (nozzle_mapping_derived) {
            // --export-3mf reloads the file, which knows nothing of this
            // routing: keep it so the plate states what the slice used.
            if (o.slice_mode && !o.export_3mf.empty()) {
                outcome.derived_filament_map = config.option<Slic3r::ConfigOptionInts>("filament_map", true)->values;
                outcome.derived_filament_map_mode = config.opt_serialize("filament_map_mode");
            }
        } else if (verbose) {
            const auto* mode = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>(
                "filament_map_mode", false);
            const std::string mode_name = mode ? mode->serialize() : "<unset>";
            std::cout << "Nozzle-map derivation: skipped mode=" << mode_name << "\n";
        }

        // Disable prime tower when there is no actual multi-material printing.
        //
        // BambuStudio (BambuStudio.cpp:3885-3913) disables prime tower when
        // all loaded filament presets are the same AND all filament colors are
        // the same.  The GUI also disables it when all filament slots map to the
        // same physical extruder (filament_map all equal) and all colors match.
        //
        // A prime tower with no tool changes (ToolOrdering::has_wipe_tower()==false)
        // causes Print::has_wipe_tower()==true to take a different code path in
        // psWipeTower: it builds m_wipe_tower_data.tool_ordering but leaves
        // m_tool_ordering empty.  psSkirtBrim then reads the empty m_tool_ordering,
        // producing initial_extruder_id==-1 and wrong path ordering.  Disabling
        // enable_prime_tower forces the correct code path.
        {
            auto* ept = config.option<Slic3r::ConfigOptionBool>("enable_prime_tower", false);
            if (ept && ept->value) {
                bool disable = false;

                // Single filament slot: trivially no multi-material.  This asks
                // the roster the project declared (filament_colour /
                // settings_id / ids / type / diameter), in the snapshot taken
                // from project_filament_count() before any normalization ran —
                // not one array's own current length, which the alignment below
                // has already grown to that same count.  A project that declares
                // no identity key at all still reads as zero here, not as one.
                if (project_filaments <= 1)
                    disable = true;

                // Multiple slots: disable when all filament colors are the same
                // AND all filament_map values point to the same extruder.
                // This covers: same-preset AMS prints (tests 20-23, 18-19),
                // same-extruder different-preset prints (tests 24-25), etc.
                // Multi-color AMS prints (test 10) have different colors → kept enabled.
                if (!disable) {
                    auto* fc = config.option<Slic3r::ConfigOptionStrings>("filament_colour", false);
                    auto* fm = config.option<Slic3r::ConfigOptionInts>("filament_map", false);
                    bool all_same_color = fc && !fc->values.empty() &&
                        std::all_of(fc->values.begin(), fc->values.end(),
                                    [&](const std::string& c){ return c == fc->values[0]; });
                    bool all_same_extruder = fm && !fm->values.empty() &&
                        std::all_of(fm->values.begin(), fm->values.end(),
                                    [&](int v){ return v == fm->values[0]; });
                    if (all_same_color && all_same_extruder)
                        disable = true;
                }

                if (disable)
                    ept->value = false;
            }
        }
#endif // ENGINE_BAMBU — BBS toolchanger / per-extruder normalizations

        // Toolchanger filament-map (U1/Prusa-XL class) on the Orca build: the
        // flat 3MF stores filament_map_mode="Auto For Flush" with a placeholder
        // filament_map. With is_BBL_printer() initialized deterministically (see
        // below, next to print construction) the engine's native Auto-For-Flush
        // resolution computes the per-filament→tool map from the per-volume
        // extruders we already load, matching golden B — so no driver-side
        // filament_map injection is needed here. Model per-extruder static
        // tables (setExtruderParams/setPrintSpeedTable) are still set after
        // apply(), exactly as Orca's own headless CLI does.

        // The selected plate's own settings over the project's, then the
        // command line over both: the official CLI slices each plate with
        // new_print_config = m_print_config; apply(*part_plate->config());
        // apply(m_extra_config) (BambuStudio.cpp 6902-6904 at 5873b5f;
        // OrcaSlicer.cpp 5905-5907 at 31f6803). A plate states bed type,
        // print sequence, first/other layer order, spiral mode and the
        // filament-switcher flags (bbs_3mf.cpp 4563-4728 / Orca 4419-4563).
        // The filament-map keys are not taken here: on the Bambu build the
        // plate block above applies the plate's mode and, in a manual mode,
        // its maps (an automatic mode regroups whatever map was saved,
        // ToolOrdering.cpp 1897-1914); on the Orca build the block just
        // below does the same. Only with a plate named (--plate N or
        // --slice): a whole-file load is not one plate.
        // The plate wins over the --config/--machine/--process/--filament
        // files: they are the official --load_settings/--load_filaments,
        // which merge into m_print_config (update_full_config, BambuStudio.cpp
        // 3246, 3384; OrcaSlicer.cpp 2781, 2925) before the plate is laid over
        // it. The command-line overrides (m_extra_config) come last.
#ifdef ENGINE_ORCA
        // The plate's filament-map keys on the Orca build. The official Orca
        // CLI lays the whole plate config over the project's
        // (OrcaSlicer.cpp 5905-5907), mode and map included. Here: the
        // plate's mode when it states one (PartPlate::get_real_filament_map_mode,
        // fmmDefault defers to the project), and in the manual mode the
        // plate's own map (get_real_filament_maps, the map whenever the plate
        // has one) and nozzle-volume map. An automatic mode keeps the
        // engine's native Auto-For-Flush resolution with the project's map,
        // as before (the U1 toolchanger case).
        bool cli_filament_map_applied = false, cli_mode_given = false;
#ifndef ENGINE_SNAPMAKER
        // --filament-map-mode and --filament-map win over the plate's mode and
        // map (apply_filament_map_flag).
        switch (apply_filament_map_flag(o, extra, config, rs, project_settings, project_filament_count(config), plate_id,
                                        outcome)) {
        case FilamentMapFlag::Refused:  return 1;
        case FilamentMapFlag::Applied:  cli_filament_map_applied = cli_mode_given = true; break;
        case FilamentMapFlag::ModeOnly: cli_mode_given = true; break;
        case FilamentMapFlag::NotGiven: break;
        }
#endif
        if (!cli_filament_map_applied && plate_id > 0 && (int)plate_data.size() >= plate_id && plate_data[plate_id - 1] != nullptr) {
            const Slic3r::PlateData& plate = *plate_data[plate_id - 1];
            Slic3r::FilamentMapMode real_mode = Slic3r::FilamentMapMode::fmmAutoForFlush;
            if (auto* mode_opt = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode", false))
                real_mode = mode_opt->value;
            if (auto* plate_mode = plate.config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode");
                !cli_mode_given && plate_mode && plate_mode->value != Slic3r::FilamentMapMode::fmmDefault) {
                real_mode = plate_mode->value;
                config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode", true)->value = real_mode;
            }
#ifndef ENGINE_SNAPMAKER
            // PlateData::filament_maps is OrcaSlicer 2.4 only; Snapmaker's
            // PlateData has no per-plate map at bac6fd5, so a plate's own map
            // cannot be applied.
            if (real_mode == Slic3r::FilamentMapMode::fmmManual && !plate.filament_maps.empty()) {
                config.option<Slic3r::ConfigOptionInts>("filament_map", true)->values = plate.filament_maps;
                if (const auto* pvm = plate.config.option<Slic3r::ConfigOptionInts>("filament_volume_map");
                    pvm && pvm->values.size() == plate.filament_maps.size())
                    config.option<Slic3r::ConfigOptionInts>("filament_volume_map", true)->values = pvm->values;
                emit_event({{"event","config_normalized"}, {"tag","PlateFilamentMapApplied"},
                            {"plate_id", plate_id}, {"filament_map", plate.filament_maps},
                            {"message","Plate " + std::to_string(plate_id) +
                                       " is in Manual mode: its own filament map applies"}});
            }
#endif // ENGINE_SNAPMAKER
        }
#endif
        {
            const Slic3r::DynamicPrintConfig plate_config = plate_own_settings(plate_data, plate_id);
            if (!plate_config.empty()) {
                config.apply(plate_config, true);
                emit_event({{"event","config_normalized"}, {"tag","PlateSettingsApplied"},
                            {"plate_id", plate_id}, {"keys", plate_config.keys()},
                            {"message","Applied plate " + std::to_string(plate_id) + "'s own settings: " +
                                       boost::algorithm::join(plate_config.keys(), ", ")}});
            }
        }

        // The settings given as flags over the plate's own (the official
        // apply(m_extra_config) after apply(*part_plate->config()),
        // BambuStudio.cpp 6902-6904, OrcaSlicer.cpp 5905-5907).
        if (!extra.empty())
            config.apply(extra, true);
        // Apply command-line overrides
        apply_command_line_overrides(config, overrides, /*report_rejections=*/true);
#ifdef ENGINE_BAMBU
        // A manual map, the command line's or the plate's, that puts a filament
        // on an extruder it cannot print with (BambuStudio.cpp 7357-7400).
        // A --slice run checks it after the plate's object exclusions
        // (--skip-objects), with the bed checks below.
        if (!o.slice_mode && !manual_map_extruders_fit(config, model, plate_id, outcome))
            return 1;
#endif
#ifdef ENGINE_ORCA
        // A Bambu Studio project's own start G-code can carry the maker's
        // placeholders, and this engine binds no value for two of them: its
        // PlaceholderParser sets initial_extruder and reads nozzle_diameter
        // from the config, while BambuStudio also sets
        // nozzle_diameter_at_nozzle_id and initial_nozzle_id (BambuStudio
        // GCode.cpp 2656-2660 at 5873b5f; upstream main binds them with its
        // multi-nozzle engine, GCode.cpp 3334-3335 on main). Unbound, the
        // parser throws at export, after the whole run has sliced
        // ("Not a variable name"): refused here, before slicing, naming the
        // placeholder and the value to use instead. The settings are the ones
        // the run slices with, overrides and all.
        {
            std::string       placeholder_key;
            const std::string placeholder = bambu_only_gcode_placeholder(config, placeholder_key);
            if (!placeholder.empty()) {
                const std::string instead = placeholder == "initial_nozzle_id"
                                                ? "{initial_extruder}"
                                                : "{nozzle_diameter[initial_extruder]}";
                const std::string sentence =
                    "The project's " + placeholder_key + " uses {" + placeholder +
                    "}, which this engine has no value for; use " + instead + " instead (OrcaSlicer binds "
                    "the initial extruder as {initial_extruder} and the nozzle diameters as {nozzle_diameter}).";
                emit_event({{"event","config_refused"},
                            {"tag","BambuOnlyGcodePlaceholder"},
                            {"opt_key", placeholder_key},
                            {"placeholder", placeholder},
                            {"replace_with", instead},
                            {"message", sentence}});
                std::cerr << "Error: " << sentence << "\n";
                set_outcome_failure(outcome, CLI_INVALID_VALUES_IN_3MF, sentence);
                return 1;
            }
        }
#endif

        // Display active settings
        std::cout << "\nActive print settings:\n";
        if (config.has("layer_height")) {
            std::cout << "  Layer height: " << config.opt_float("layer_height") << "mm\n";
        }
        if (config.has("perimeters")) {
            std::cout << "  Perimeters: " << config.opt_int("perimeters") << "\n";
        }
        if (config.has("sparse_infill_density")) {
            auto percent_opt = config.option<Slic3r::ConfigOptionPercent>("sparse_infill_density");
            if (percent_opt) {
                std::cout << "  Infill: " << percent_opt->value << "%\n";
            }
        }
        if (config.has("nozzle_diameter")) {
            auto nozzles = config.option<Slic3r::ConfigOptionFloats>("nozzle_diameter");
            if (nozzles && !nozzles->values.empty()) {
                std::cout << "  Nozzle: " << nozzles->values[0] << "mm\n";
            }
        }
        run_config_events.drop();
        } // fresh_plate_config
        // A plate sliced after the run-level steps ran for every plate: the
        // towers those steps placed (the official's m_print_config after its
        // arrange, BambuStudio.cpp 5693-5700, 5856; OrcaSlicer.cpp 4954-4961).
        if (fresh_plate_config && rs.prepared)
            for (const char* key : {"wipe_tower_x", "wipe_tower_y"})
                if (const Slic3r::ConfigOption* opt = rs.run_config.option(key))
                    config.set_key_value(key, opt->clone());

        // A plate's print sequence: its own, or else the project's
        // (get_print_sequence, BambuStudio.cpp 4403-4416), for the plates
        // other than this one (--downward-check, and the whole project of
        // the model actions for --slice 0).
        std::optional<bool> project_sequence_read;
        const auto project_sequence = [&]() {
            if (project_sequence_read)
                return *project_sequence_read;
            std::string project_seq;
            if (const Slic3r::ConfigOption* opt = extra.option("print_sequence"))
                project_seq = opt->serialize();
            else if (!merged_print_sequence.empty())
                // --load-settings and friends: the merged settings
                // (m_print_config, BambuStudio.cpp 4785).
                project_seq = merged_print_sequence;
            else if (assemble_input) {
                if (const auto* seq = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence"))
                    project_seq = seq->serialize();
            } else {
                std::string text;
                if (read_zip_member(input_file, "Metadata/project_settings.config", text)) {
                    try {
                        const json settings = json::parse(text);
                        if (settings.contains("print_sequence") && settings["print_sequence"].is_string())
                            project_seq = settings["print_sequence"].get<std::string>();
                    } catch (...) {
                    }
                }
            }
            project_sequence_read = project_seq == "by object";
            return *project_sequence_read;
        };
        const auto plate_sequence = [&](int index) {
            if (index < (int)plate_data.size() && plate_data[index] != nullptr)
                if (const auto* seq = plate_data[index]->config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                    seq && seq->value != Slic3r::PrintSequence::ByDefault)
                    return seq->value == Slic3r::PrintSequence::ByObject;
            return project_sequence();
        };

        // --downward-check, before the transforms as the official runs it
        // (BambuStudio.cpp 4669-4914; OrcaSlicer.cpp 4006-4175). The
        // official checks the plates of a project (plate_obj_size_infos
        // exists only then, BambuStudio.cpp 4646-4658), every one of them.
        if (o.given_flag("downward_check") && o.cli.opt_bool("downward_check") && context.g_downward_run.done) {
            // Every plate was checked by the run's first call.
            outcome.downward_checked = true;
            outcome.downward_printers = context.g_downward_run.printers;
            outcome.downward_failed = context.g_downward_run.failed;
            outcome.sequence_plate = context.g_downward_run.sequence_plate;
            outcome.multi_nozzle_diameters = context.g_downward_run.multi_nozzle_diameters;
        } else if (o.given_flag("downward_check") && o.cli.opt_bool("downward_check")) {
            if (plate_data.empty()) {
                const std::string why = "--downward-check checks the plates of a 3MF project; this input has none";
                std::cerr << "Error: " << why << "\n";
                set_outcome_failure(outcome, CLI_INVALID_PARAMS, why);
                return 1;
            }
            std::vector<slicer_cli::DownwardPrinter> printers;
            const slicer_cli::StepResult loaded =
                slicer_cli::load_downward_printers(o, project_facts, this_engine_profiles_dir(o.argv0).string(), printers);
            if (loaded.code != 0) {
                std::cerr << "Error: " << loaded.message << "\n";
                set_outcome_failure(outcome, loaded.code, loaded.message);
                return 1;
            }
            // Every plate of the project, whichever is sliced (BambuStudio.cpp
            // 4779-4790; OrcaSlicer.cpp 4104-4117): each plate's objects, and
            // its own print sequence or else the project's (get_print_sequence,
            // BambuStudio.cpp 4403-4416). A plate in the run's model is read
            // from it, in its own frame (PlateScope); a plate the run did not
            // load (--plate N / --slice N load one plate) from the project.
            struct DownwardPlate {
                int index = 0;
                std::unique_ptr<Slic3r::Model> loaded;
                const Slic3r::Model* model = nullptr;
                bool is_sequence = false;
            };
            std::vector<DownwardPlate> downward_plates;
            const int project_plates = (int)plate_data.size();
            for (int index = 0; index < project_plates; ++index) {
                DownwardPlate plate;
                plate.index = index;
                const bool in_model = index < (int)rs.plates.size() &&
                                      (rs.plates[index].all || !rs.plates[index].members.empty());
                if (project_plates == 1 || index == plate_id - 1 || in_model) {
                    {
                        PlateScope scope(model, rs.plate(index + 1));
                        plate.loaded = std::make_unique<Slic3r::Model>(model);
                    }
                    plate.model = plate.loaded.get();
                    if (project_plates == 1 || index == plate_id - 1) {
                        bool is_sequence = false;
                        if (const auto* seq = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence"))
                            is_sequence = seq->value == Slic3r::PrintSequence::ByObject;
                        plate.is_sequence = is_sequence;
                    } else {
                        plate.is_sequence = plate_sequence(index);
                    }
                } else {
                    plate.loaded = std::make_unique<Slic3r::Model>();
                    if (assemble_input) {
                        for (const Slic3r::ModelObject* object : context.g_assemble->plates[index].loaded_obj_list)
                            plate.loaded->add_object(*object);
                    } else if (!load_plate_objects(context, input_file, index + 1, *plate.loaded)) {
                        const std::string why = "--downward-check could not read plate " + std::to_string(index + 1) +
                                                " of " + input_file + ".";
                        std::cerr << "Error: " << why << "\n";
                        set_outcome_failure(outcome, CLI_DATA_FILE_ERROR, why);
                        return 1;
                    } else if (verbose) {
                        std::cout << "--downward-check: read plate " << index + 1 << " from the project\n";
                    }
                    if (!assemble_input && index < (int)plate_origins.size()) {
                        // The plate at its own origin, as the sliced plate is:
                        // the official adds the plate's origin to its tower
                        // position instead (plate->get_origin(), BambuStudio.cpp
                        // 4486-4489; OrcaSlicer.cpp 3875-3878).
                        const Slic3r::Vec2d origin = plate_origins[index];
                        for (Slic3r::ModelObject* object : plate.loaded->objects)
                            for (Slic3r::ModelInstance* inst : object->instances) {
                                const Slic3r::Vec3d off = inst->get_offset();
                                inst->set_offset(Slic3r::Vec3d(off.x() - origin.x(), off.y() - origin.y(), off.z()));
                            }
                    }
                    plate.model = plate.loaded.get();
                    plate.is_sequence = plate_sequence(index);
                }
                downward_plates.push_back(std::move(plate));
            }
            // A Bambu-made 3MF with support on anywhere (BambuStudio.cpp 2197-2210).
            bool has_support = false;
            if (is_bbl_3mf && !trailing_port) {
                const auto* support = config.option<Slic3r::ConfigOptionBool>("enable_support");
                has_support = support && support->value;
                for (const DownwardPlate& plate : downward_plates)
                    for (const Slic3r::ModelObject* object : plate.model->objects) {
                        const auto* obj_support = object->config.get().option<Slic3r::ConfigOptionBool>("enable_support");
                        has_support |= obj_support && obj_support->value;
                    }
            }
            outcome.downward_checked = true;
            for (const slicer_cli::DownwardPrinter& p : printers)
                outcome.downward_printers.push_back(p.name);
#ifdef ENGINE_BAMBU
            // Mixed nozzle diameters (BambuStudio.cpp 5218-5258 at v02.09.01.70):
            // every diameter the project's filaments select needs a nozzle of
            // its own, so a printer that mounts fewer nozzles than that fails.
            // With no plate using a filament, the whole map counts.
            if (Slic3r::multi_nozzle_diameter_mode_enabled(config)) {
                std::vector<std::vector<int>> plates_filaments;
                for (const DownwardPlate& plate : downward_plates)
                    plates_filaments.push_back(arrange_plate_extruders(*plate.model, config, plate.index));
                const std::set<std::string> used_diameters = slicer_cli::project_nozzle_diameters(config, plates_filaments);
                outcome.multi_nozzle_diameters = used_diameters.size() > 1;
                if (used_diameters.size() > 1)
                    for (const slicer_cli::DownwardPrinter& p : printers)
                        if ((int)used_diameters.size() > p.max_nozzle_count &&
                            std::find(outcome.downward_failed.begin(), outcome.downward_failed.end(), p.name) ==
                                outcome.downward_failed.end()) {
                            emit_event({{"event","warning"},
                                        {"tag","PrinterNozzleCountTooSmall"},
                                        {"printer", p.name},
                                        {"max_nozzle_count", p.max_nozzle_count},
                                        {"nozzle_diameters", std::vector<std::string>(used_diameters.begin(), used_diameters.end())},
                                        {"message", p.name + " mounts " + std::to_string(p.max_nozzle_count) +
                                                    " nozzle(s); the project uses " + std::to_string(used_diameters.size()) +
                                                    " nozzle diameters"}});
                            outcome.downward_failed.push_back(p.name);
                        }
            }
#endif
            for (const DownwardPlate& plate : downward_plates) {
                outcome.sequence_plate |= plate.is_sequence;
                if (printers.empty())
                    continue;
                const std::vector<std::string> failed = slicer_cli::downward_failures(
                    printers, downward_plate_size(context, *plate.model, config, plate.index, plate.is_sequence),
                    plate.is_sequence, config, project_facts, has_support);
                for (const std::string& name : failed)
                    if (std::find(outcome.downward_failed.begin(), outcome.downward_failed.end(), name) ==
                        outcome.downward_failed.end())
                        outcome.downward_failed.push_back(name);
            }
            context.g_downward_run.done = true;
            context.g_downward_run.printers = outcome.downward_printers;
            context.g_downward_run.failed = outcome.downward_failed;
            context.g_downward_run.sequence_plate = outcome.sequence_plate;
            context.g_downward_run.multi_nozzle_diameters = outcome.multi_nozzle_diameters;
        }

        // --pipe: the files are loaded (BambuStudio.cpp 4923-4926;
        // OrcaSlicer.cpp 4185), before the transforms.
        pipe_files_loaded(rs);

        // The transforms on the command line, in its order, then orient
        // (cli_run_steps.cpp). Before any placement, as the official CLI
        // runs them before its arrange.
        // A plate of the plan has its objects transformed already (the plan
        // pass ran the transforms on every object, as the official does).
        // The plates of an assemble list at their places in the plate grid of
        // this printer's bed (PartPlateList::compute_origin, PartPlate.cpp
        // 4306-4341), now that the bed is known.
        if (first_call && assemble_input) {
            const int count = (int)context.g_assemble->plates.size();
            for (int k = 0; k < count && k < (int)rs.plates.size(); ++k) {
                const Slic3r::Vec2d origin = plate_grid_origin(config, Slic3r::Semver(), k, count);
                rs.plates[k].origin = origin;
                for (Slic3r::ModelInstance* inst : rs.plates[k].members)
                    inst->set_offset(inst->get_offset() + Slic3r::Vec3d(origin.x(), origin.y(), 0.));
            }
        }
        if (first_call && !o.transforms.empty()) {
            int duplicate_count = 0;
            const int plate_count = std::max<int>(1, (int)plate_data.size());
            const slicer_cli::StepResult t = slicer_cli::apply_transforms(
                o, model, &settings_merge.load_process_config, o.slice_mode ? o.slice_plate : plate_id, plate_count,
                duplicate_count);
            if (t.code != 0) {
                std::cerr << "Error: " << t.message << "\n";
                set_outcome_failure(outcome, t.code, t.message);
                return 1;
            }
            outcome.duplicate_count = duplicate_count;
        }
        // --simplify: the desktop Simplify tool, after the transforms and
        // before the objects are placed, so the placement, the bed check,
        // --mtcpp, every export and the slice see the reduced meshes. The
        // desktop's "more than 1M triangles" suggestion follows, on the
        // meshes as they now stand (cli_simplify.cpp).
        if (first_call) {
            // The ids --simplify-objects and --skip-objects may name
            // (cli_engine_info.hpp).
            const std::set<int> known_object_ids = slicer_cli::known_object_ids(
                o.input_files, model,
                [](const std::string& archive, const std::string& member, std::string& out) {
                    return read_zip_member(archive, member, out);
                });
            // --skip-objects keeps the official meaning (an unknown id is
            // ignored, BambuStudio.cpp 6541-6581), so the caller is told once
            // by an event, where --simplify-objects, a new flag, refuses.
            if (const std::string why = slicer_cli::unknown_object_ids_refusal(
                    "--skip-objects", o.cli.option<Slic3r::ConfigOptionInts>("skip_objects")->values, known_object_ids);
                !why.empty())
                emit_event({{"event", "warning"},
                            {"tag", "SkipObjectsUnknownIds"},
                            {"message", why + ". The id is ignored and the slice goes on, as the official command line does"}});
            const slicer_cli::StepResult sm = slicer_cli::simplify_meshes(o, model, known_object_ids);
            if (sm.code != 0) {
                std::cerr << "Error: " << sm.message << "\n";
                set_outcome_failure(outcome, sm.code, sm.message);
                return 1;
            }
            slicer_cli::warn_large_meshes(model);
        }
        {
            const slicer_cli::StepResult sd = slicer_cli::check_slicedata_flags(o, outcome.duplicate_count);
            if (sd.code != 0) {
                std::cerr << "Error: " << sd.message << "\n";
                set_outcome_failure(outcome, sd.code, sd.message);
                return 1;
            }
        }
        const bool needs_placement = first_call && (geometry_input ||
            std::any_of(model.objects.begin(), model.objects.end(),
                        [](const Slic3r::ModelObject* obj) { return obj->instances.empty(); }));

        // Geometry placement, with the bed now known. --arrange 1 or 0: the
        // official CLI's default instances (read_from_file with
        // AddDefaultInstances, BambuStudio.cpp 1884), then --arrange 1
        // arranges below. Otherwise the desktop's placement on load
        // (cli_model_load.cpp). A project 3MF's own objects keep their places.
        if (first_call && geometry_only_3mf && o.arrange_auto())
            slicer_cli::desktop_center_geometry_3mf(model, config);
        if (needs_placement) {
            if (!o.arrange_auto())
                model.add_default_instances();
            else
                slicer_cli::desktop_place_on_bed(model, config);
            for (auto* obj : model.objects)
                for (auto* inst : obj->instances)
                    inst->use_loaded_id_for_label = true;
        }

        // Actions that work on the loaded model and settings (--info,
        // --export-settings, --export-stl, --export-stls). The official runs
        // every action in one loop in command-line order, the first that fails
        // stopping the rest (BambuStudio.cpp 6336, 6366-6401; OrcaSlicer.cpp
        // 5470, 5499-5534), --slice among them (6437; 5561) with each plate's
        // bed check and slice inside it. So with --slice the actions given
        // before it run before the first plate's bed check, and those given
        // after it once the last plate has sliced; without --slice they are the
        // whole run. A failure is recorded with plate 0, as the official records
        // these (record_exit_reson(..., CLI_EXPORT_STL_ERROR, 0, ...),
        // BambuStudio.cpp 6391, 6399).
        // A project 3MF with model files after it: the plan (ProjectPlan) once
        // the transforms ran, before any action, as the official arranges
        // before its action loop; the actions then see every object at its
        // arranged place.
        // --slice 0 --arrange 1 on a project of several plates: the official
        // arranges every plate's objects together across the plates (the
        // global arrange, BambuStudio.cpp 5628-5728, 5886-6006; OrcaSlicer.cpp
        // 4887-4990, 5142-5262: duplicate_count 0 and plate_to_slice 0), adding
        // or dropping plates, as for model files after the project.
        // A run of model actions only is a run without --slice, whose
        // plate_to_slice stays 0 (BambuStudio.cpp 1677; OrcaSlicer.cpp 1368):
        // the official arranges every plate for it too.
        const bool global_arrange = (o.slice_mode ? o.slice_plate == 0 : slicer_cli::model_actions_only(o)) &&
                                    o.arrange_forced() && is_bbl_3mf &&
                                    !assemble_input && !trailing_port && plate_data.size() > 1;
        if (first_call && (trailing_port || global_arrange) && !calib_self_geometry) {
            std::set<int> used_filaments;
#ifndef ENGINE_ORCA
            for (int f = 1; f <= plan_file_filaments; ++f)
                used_filaments.insert(f);
#endif
            const auto& filament_ids = o.cli.option<Slic3r::ConfigOptionInts>("load_filament_ids")->values;
            for (size_t index = 1; index < o.input_files.size() && index < filament_ids.size(); ++index)
                if (filament_ids[index] > 0)
                    used_filaments.insert(filament_ids[index]);
            ProjectPlan& plan = rs.plan;
            const Slic3r::DynamicPrintConfig& file_config = plan_file_config ? *plan_file_config : config;
            if (!plan_project_plates(context, model, config, file_config, plan_file_version, plate_data, (int)used_filaments.size(),
                                     global_arrange || o.arrange != 0, plan, outcome)) {
                std::cerr << "Error: " << outcome.error_string << "\n";
                return 1;
            }
            // The plates as the plan left them; a plate the arrange added is a
            // fresh plate with no settings of its own (PartPlateList::create_plate).
            rs.plates.assign(plan.plate_count, RunPlate());
            for (int k = 0; k < plan.plate_count; ++k) {
                rs.plates[k].origin  = plan.origins[k];
                rs.plates[k].all     = false;
                rs.plates[k].members = plan.members[k];
            }
            while ((int)plate_data.size() < plan.plate_count) {
                auto* pd = new Slic3r::PlateData();
                pd->plate_index = int(plate_data.size());
                plate_data.push_back(pd);
            }
            emit_project_plan_event(plan, trailing_port);
        }

        // The actions read the run's one model, every object at its place in
        // the scene (the official's m_models, BambuStudio.cpp 6371-6401;
        // OrcaSlicer.cpp 5504-5534), and --export-settings the project's
        // settings (m_print_config, BambuStudio.cpp 6366-6370; OrcaSlicer.cpp
        // 5499-5503).
        const auto run_model_action_step = [&](slicer_cli::ActionPhase phase) -> bool {
            slicer_cli::StepResult a;
            // --export-settings writes the project's settings, not this
            // plate's (see project_settings above). The command line is
            // already in them; the setting flags go on as on `config`.
            std::optional<Slic3r::DynamicPrintConfig> settings_to_export = project_settings;
            if (settings_to_export)
                apply_command_line_overrides(*settings_to_export, overrides, /*report_rejections=*/false);
#ifdef ENGINE_SNAPMAKER
            // The switch's filament list, as each plate slices with it.
            if (settings_to_export && slicer_cli::switch_trims_filaments(o.preset_switch, o.printer_preset))
                slicer_cli::trim_project_roster_for_switch(*settings_to_export);
#endif
            {
                // The actions run once per run, also from the check pass:
                // their events are not held.
                std::vector<std::string>* const held = slicer_cli::diagnostics::held_events;
                slicer_cli::diagnostics::held_events = nullptr;
                a = slicer_cli::run_model_actions(o, model, settings_to_export ? *settings_to_export : config, nullptr,
                                                  phase);
                slicer_cli::diagnostics::held_events = held;
            }
            if (a.code != 0) {
                std::cerr << "Error: " << a.message << "\n";
                set_outcome_failure(outcome, a.code, a.message);
                outcome.run_step_failed = true;
                return false;
            }
            return true;
        };
        // Every output of the run, checked once the model is loaded (the
        // object STL names are known) and before the first write: the
        // actions, then the slice (check_run_outputs). The plates are the
        // ones already known; --slice checks them all again once its plan is
        // final (run_slice_mode).
        const auto outputs_ok = [&](bool after_slice_actions_only = false) -> bool {
            if (rs.outputs_checked)
                return true;
            rs.outputs_checked = true;
            // Every plate the run could write: plate N for --slice N, else
            // every plate of the plan, or of the input when there is no plan
            // yet (a superset is fine: the check after the plan stays).
            std::vector<int> known_plates;
            if (o.slice_mode) {
                if (o.slice_plate > 0)
                    known_plates.push_back(o.slice_plate);
                const int count = rs.plan.done ? std::max(rs.plan.plate_count, rs.file_plate_count)
                                               : rs.file_plate_count;
                if (o.slice_plate <= 0)
                    for (int p = 1; p <= std::max(1, count); ++p)
                        known_plates.push_back(p);
            }
            std::string detail;
            if (const int refused = check_run_outputs(context, o, known_plates, &model, &detail, after_slice_actions_only,
                                                      after_slice_actions_only ? &rs.reserved_outputs : nullptr,
                                                      after_slice_actions_only ? nullptr : &rs.reserved_outputs)) {
                set_outcome_failure(outcome, refused, detail);
                outcome.run_step_failed = true;
                return false;
            }
            return true;
        };

        // --slice: the official per-plate gate before apply (BambuStudio.cpp
        // 6527-6567 at 5873b5f; OrcaSlicer.cpp 5645-5697 at 31f6803). An
        // object partly over the bed edge is refused, and a plate with no
        // object fully inside is refused; objects wholly outside are left
        // out of the print, as the official CLI's apply() leaves them out.
        // --slice, once per run before the plate loop: the arrange, the
        // --ensure-on-bed lift and the actions given before --slice
        // (BambuStudio.cpp 5237-6194, 6336; OrcaSlicer.cpp 4498-5455, 5470),
        // for the plate the run names (in its own frame, PlateScope) or for
        // every plate. A run of model actions only takes the same steps: the
        // official arranges before its action loop with or without --slice
        // (need_arrange for model files, BambuStudio.cpp 2077, 5558, then
        // 6336; OrcaSlicer.cpp 1724, 4817, then 5470).
        const bool actions_only = slicer_cli::model_actions_only(o);
        if ((o.slice_mode || actions_only) && !calib_self_geometry && first_call) {
            // The plate's towers before the arrange: only the entries the
            // arrange changes go into the project below. The rest are the
            // flags laid over this plate (BambuStudio.cpp 6902-6904), which
            // reach only the slice.
            std::map<std::string, std::vector<double>> tower_before;
            for (const char* key : {"wipe_tower_x", "wipe_tower_y"})
                if (const auto* opt = config.option<Slic3r::ConfigOptionFloats>(key))
                    tower_before[key] = opt->values;
            // The plate's filament count from its last slice (slice_info), as
            // the official reads plate_data_src[plate]->slice_filaments_info
            // (BambuStudio.cpp 5794; OrcaSlicer.cpp 5050); 0 for an STL.
            size_t sliced_filament_count = 0;
            if (plate_id > 0 && (int)plate_data.size() >= plate_id && plate_data[plate_id - 1] != nullptr)
                sliced_filament_count = plate_data[plate_id - 1]->slice_filaments_info.size();
            const int arrange_plate_index = plate_id > 0 ? plate_id - 1 : 0;
            // Several objects from model files and --arrange automatic (not 0
            // or 1, or not given): the official
            // CLI arranges model files (need_arrange is set for them,
            // BambuStudio.cpp 2077 / OrcaSlicer.cpp 1724, and --arrange only
            // overrides it, 5066-5081 / 4327-4342), where the desktop's
            // placement on load could stack them. One object keeps the
            // desktop's placement (the bed centre either way).
            int model_file_instances = 0;
            if (geometry_input)
                for (const Slic3r::ModelObject* object : model.objects)
                    for (const Slic3r::ModelInstance* inst : object->instances)
                        model_file_instances += inst->printable ? 1 : 0;
            const bool auto_arrange_model_files = o.arrange_auto() && model_file_instances > 1;
            // A printer change that changes the extruder clearances re-arranges
            // a plate printed by object (BambuStudio.cpp 5275-5288: only on a
            // change of printer, the distance to the rod counted; OrcaSlicer.cpp
            // 4538-4549: any change of the clearances), unless the bed changed
            // (shrink_to_new_bed) or the plate is arranged anyway.
            bool clearance_arrange = false;
            {
                const auto* seq = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                const bool is_seq_print = seq && seq->value == Slic3r::PrintSequence::ByObject;
                const bool need_arrange = o.arrange_forced() || auto_arrange_model_files || outcome.duplicate_count > 0 ||
                                          assemble_input;
                auto opt_or_zero = [&](const char* key) {
                    return config.option<Slic3r::ConfigOptionFloat>(key) ? float(config.opt_float(key)) : 0.f;
                };
                const float height_to_rod = opt_or_zero("extruder_clearance_height_to_rod");
                const float height_to_lid = opt_or_zero("extruder_clearance_height_to_lid");
#ifdef ENGINE_ORCA
                const float clearance_radius = opt_or_zero("extruder_clearance_radius");
                const bool printer_changed = true;
                const bool rod_distance_changed = false;
#else
                const float clearance_radius = opt_or_zero("extruder_clearance_max_radius");
                const bool printer_changed = !settings_merge.new_printer_system_name.empty() &&
                                             settings_merge.new_printer_system_name != project_facts.current_printer_system_name;
                const float distance_to_rod = opt_or_zero("extruder_clearance_dist_to_rod");
                const bool rod_distance_changed = project_facts.old_distance_to_rod != 0.f &&
                                                  project_facts.old_distance_to_rod != distance_to_rod;
#endif
                const bool clearances_changed =
                    (project_facts.old_height_to_rod != 0.f && project_facts.old_height_to_rod != height_to_rod) ||
                    (project_facts.old_height_to_lid != 0.f && project_facts.old_height_to_lid != height_to_lid) ||
                    (project_facts.old_max_radius != 0.f && project_facts.old_max_radius != clearance_radius) ||
                    rod_distance_changed;
                // plate_to_slice > 0: one plate named, never --slice 0; 0
                // without --slice (BambuStudio.cpp 1677).
                const int plate_to_slice = o.slice_mode ? o.slice_plate : 0;
                clearance_arrange = !need_arrange && is_bbl_3mf && !trailing_port && shrink_to_new_bed == 0 && plate_to_slice > 0 &&
                                    printer_changed && clearances_changed && is_seq_print;
                if (clearance_arrange)
                    emit_event({{"event","arranged"}, {"tag","ClearanceArrange"},
                                {"message","The printer's extruder clearances differ from the project's: the by-object plate is arranged again"}});
            }
            // --repetitions: a spiral vase is copied only when printing by
            // object (BambuStudio.cpp 5259-5269; OrcaSlicer.cpp 4519-4529).
            if (!assemble_input && outcome.duplicate_count > 0) {
                const auto* spiral = config.option<Slic3r::ConfigOptionBool>("spiral_mode");
                const auto* seq    = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                if (spiral && spiral->value && !(seq && seq->value == Slic3r::PrintSequence::ByObject)) {
                    emit_event({{"event","arranged"}, {"tag","RepetitionsSkippedSpiral"},
                                {"message","--repetitions: a spiral vase is copied only when printing by object; the plate is printed once"}});
                    outcome.duplicate_count = 0;
                }
            }
            // --repetitions arranges the copies and sets the tower in `config`
            // (arrange_repetitions); the project keeps that tower.
            if (assemble_input) {
                // An assemble list's plates take the list's own arrange in
                // place of --arrange and --repetitions (BambuStudio.cpp
                // 5326-5557, before the general arrange's else-branch;
                // OrcaSlicer.cpp 4582-4816), every plate of the run in its
                // own frame.
                for (int arranged_plate = plate_id > 0 ? plate_id : 1;
                     arranged_plate <= (plate_id > 0 ? plate_id : (int)context.g_assemble->plates.size()); ++arranged_plate) {
                const int arrange_plate_index = arranged_plate - 1;
                const slicer_cli::AssemblePlate& plate = context.g_assemble->plates[arrange_plate_index];
                // A plate of --slice 0 reads its own settings (plate_params)
                // over the run's, as its own slice does.
                Slic3r::DynamicPrintConfig plate_settings_copy;
                if (plate_id == 0) {
                    plate_settings_copy = config;
                    plate_settings_copy.apply(plate_own_settings(plate_data, arranged_plate), true);
                    if (!extra.empty())
                        plate_settings_copy.apply(extra, true);
                    apply_command_line_overrides(plate_settings_copy, overrides, /*report_rejections=*/false);
                }
                Slic3r::DynamicPrintConfig& config_k = plate_id == 0 ? plate_settings_copy : config;
                PlateScope scope(model, rs.plate(arranged_plate));
                if (plate.need_arrange) {
                    if (!arrange_assemble_plate(context, model, config_k, arrange_plate_index, plate.filaments_count,
                                                context.g_assemble->model, outcome)) {
                        std::cerr << "Error: " << outcome.error_string << "\n";
                        return 1;
                    }
                } else {
                    // The objects keep the list's positions; a plate of several
                    // filaments gets the tower's default corner unless a
                    // position was given (has_wipe_tower_position, BambuStudio.cpp
                    // 4098: wipe_tower_x/y are in no preset, so only a settings
                    // file or a flag states them for a run without a 3MF). The
                    // corner is the desktop's own: clamped inside the plate and
                    // its extruder areas, as the desktop clamps a new plate's
                    // (set_default_wipe_tower_pos_for_plate, Orca PartPlate.cpp
                    // 4115-4190; Bambu PartPlate.cpp 4615-4702) — the official
                    // CLI's corner alone put the tower off the bed and refused
                    // this run -104.
                    const auto* seq = config_k.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                    const bool is_seq_print = seq && seq->value == Slic3r::PrintSequence::ByObject;
                    const auto states_tower = [](const Slic3r::DynamicPrintConfig& c) {
                        return c.has("wipe_tower_x") && c.has("wipe_tower_y");
                    };
                    const bool has_wipe_tower_position =
                        states_tower(settings_merge.load_process_config) || states_tower(o.extra_config);
                    if (!is_seq_print && plate.filaments_count > 1 && !has_wipe_tower_position)
                        assemble_tower_inside_plate(context, config_k, model, arrange_plate_index, /*keep_existing=*/false);
                }
                // The plate's tower entry goes into the run's settings.
                if (plate_id == 0)
                    for (const char* key : {"wipe_tower_x", "wipe_tower_y"})
                        if (const auto* placed = config_k.option<Slic3r::ConfigOptionFloats>(key);
                            placed && (int)placed->values.size() > arrange_plate_index) {
                            Slic3r::ConfigOptionFloat entry(placed->get_at(arrange_plate_index));
                            config.option<Slic3r::ConfigOptionFloats>(key, true)->set_at(&entry, arrange_plate_index, 0);
                        }
                }
            } else if (outcome.duplicate_count > 0) {
                PlateScope scope(model, rs.plate(plate_id));
                std::set<int> skip_ids;
                for (int id : o.cli.option<Slic3r::ConfigOptionInts>("skip_objects")->values)
                    skip_ids.insert(id);
                model.add_default_instances();
                const slicer_cli::RepetitionsResult reps = slicer_cli::arrange_repetitions(
                    model, config, arrange_plate_index, outcome.duplicate_count, skip_ids,
                    [&](Slic3r::Model& m, Slic3r::DynamicPrintConfig& c, int& landed) {
                        PlateOutcome scratch = outcome;
                        return arrange_on_bed(context, m, c, arrange_plate_index, sliced_filament_count, scratch, &landed);
                    });
                // Copies the loop left off the plate stand wholly outside it
                // and are left out by the inside check below.
                outcome.duplicate_count = reps.copies_kept;
            } else if ((o.arrange_forced() || auto_arrange_model_files || clearance_arrange) && !rs.plan.done) {
                PlateScope scope(model, rs.plate(plate_id));
                if (!arrange_on_bed(context, model, config, arrange_plate_index, sliced_filament_count, outcome)) {
                    std::cerr << "Error: " << outcome.error_string << "\n";
                    return 1;
                }
            }
            // --ensure-on-bed before the actions given before --slice, as the
            // official lifts the objects before its action loop
            // (BambuStudio.cpp 6188-6194 then 6336; OrcaSlicer.cpp 5443-5455
            // then 5470); --skip-objects stays in the slice step after them.
            slicer_cli::ensure_on_bed_if_asked(o, model);
            // The towers the arrange placed are the project's from here on:
            // the official arrange writes them into m_print_config
            // (BambuStudio.cpp 5693-5700, 5856, 6075; OrcaSlicer.cpp
            // 4954-4961, 5112, 5335), which --export-settings, result.json and
            // --export-3mf then read (6366-6370, 6905-6910, 8156-8157).
            // Only the entries the arrange set: a tower given as a flag stays
            // the plate's, and the project keeps its own (moved) value.
            for (const char* key : {"wipe_tower_x", "wipe_tower_y"}) {
                const auto* placed = config.option<Slic3r::ConfigOptionFloats>(key);
                if (placed == nullptr)
                    continue;
                const std::vector<double>& before = tower_before[key];
                auto* project_tower = rs.base.option<Slic3r::ConfigOptionFloats>(key);
                if (project_tower == nullptr || project_tower->values.empty()) {
                    rs.base.set_key_value(key, placed->clone());
                    continue;
                }
                for (size_t k = 0; k < placed->values.size(); ++k) {
                    if (k < before.size() && before[k] == placed->values[k])
                        continue;
                    Slic3r::ConfigOptionFloat entry(placed->values[k]);
                    project_tower->set_at(&entry, k, 0);
                }
            }
            if (slicer_cli::has_model_actions(o))
                project_settings = rs.base;
            if (!outputs_ok())
                return 1;
            if (slicer_cli::has_model_actions(o) && !rs.actions_done) {
                rs.actions_done = true;
                if (!run_model_action_step(o.slice_mode ? slicer_cli::ActionPhase::BeforeSlice
                                                        : slicer_cli::ActionPhase::All))
                    return 1;
            }
            // Model actions only: the actions were the whole run.
            if (actions_only) {
                outcome.actions_only = true;
                return 0;
            }
            // --pipe: the slice action starts, before the first plate's
            // checks (BambuStudio.cpp 6455-6458; OrcaSlicer.cpp 5580).
            pipe_prepare_slicing(rs);
        }
        // The run-level steps are done: the prepare call stops here, before
        // any plate is checked.
        if (first_call && o.slice_mode) {
            rs.prepared = true;
            rs.prepared_plate = plate_id;
            rs.run_config = config;
            rs.carry = outcome;
            if (outcome.prepare_pass)
                return 0;
        }

        // --slice: the official per-plate gate before apply (BambuStudio.cpp
        // 6527-6567 at 5873b5f; OrcaSlicer.cpp 5645-5697 at 31f6803). An
        // object partly over the bed edge is refused, and a plate with no
        // object fully inside is refused; objects wholly outside are left
        // out of the print, as the official CLI's apply() leaves them out.
        if (o.slice_mode && !calib_self_geometry) {
            RunPlate& run_plate = rs.plate(plate_id);
            pipe_prepare_slicing(rs);
            // A plate with none of the run's instances: the refusal a load of
            // that plate alone gave (above, "No objects found in model").
            if (!run_plate.all && run_plate.members.empty()) {
                std::cerr << "No objects found in model\n";
                set_outcome_failure(outcome, CLI_NO_SUITABLE_OBJECTS);
                return 1;
            }
            const int step_plate = plate_id > 0 ? plate_id : std::max(1, o.slice_plate);
            {
                // The plate's instances in the plate's own frame: the bed check
                // reads the plate's shape (PartPlate::get_shape, the bed at the
                // plate's origin, BambuStudio.cpp 6527).
                PlateScope scope(model, run_plate);
                if (!plate_object_steps(o, model, step_plate, outcome))
                    return 1;
#ifdef ENGINE_BAMBU
                if (!manual_map_extruders_fit(config, model, step_plate, outcome))
                    return 1;
#endif
                if (!check_objects_inside_bed(model, config, outcome, o.arrange_forced())) {
                    std::cerr << "Error: " << outcome.error_string << "\n";
                    return 1;
                }
                if (!plate_triangle_limit(o, model, step_plate, outcome))
                    return 1;
            }
            // The other plates' instances are outside this plate, so this
            // plate's Print leaves them out (ModelInstance::is_printable,
            // Model.hpp 1466 / 1344; PrintApply.cpp 145-155).
            for (Slic3r::ModelObject* object : model.objects)
                for (Slic3r::ModelInstance* inst : object->instances)
                    if (!run_plate.is_member(inst))
                        inst->print_volume_state = Slic3r::ModelInstancePVS_Fully_Outside;
        } else if (!calib_self_geometry) {
            if (first_call)
                slicer_cli::ensure_on_bed_if_asked(o, model);
            // --skip-objects and --mtcpp belong to the slice action
            // (BambuStudio.cpp 6541-6597; OrcaSlicer.cpp 5645-5720): a run of
            // model actions only never slices, so it skips nothing.
            if (!slicer_cli::model_actions_only(o) &&
                (!plate_object_steps(o, model, std::max(1, plate_id), outcome) ||
                 !plate_triangle_limit(o, model, std::max(1, plate_id), outcome)))
                return 1;
        }

        // Without --slice, or for the calibration prints that make their own
        // model, the actions run here; with --slice those before it ran before
        // the bed check above. The outputs are checked first (without --slice
        // this is the check before the actions and the -o G-code).
        if (!outputs_ok())
            return 1;
        if (slicer_cli::has_model_actions(o) && !rs.actions_done) {
            rs.actions_done = true;
            if (!run_model_action_step(o.slice_mode ? slicer_cli::ActionPhase::BeforeSlice : slicer_cli::ActionPhase::All))
                return 1;
            if (slicer_cli::model_actions_only(o)) {
                outcome.actions_only = true;
                return 0;
            }
        }

        // Initialize print
        std::cout << "\nInitializing print...\n";
        Slic3r::Print print;

        // The plate's own per-layer custom G-code (pauses, colour changes,
        // custom lines): the loader keys them by plate (plate_id - 1,
        // bbs_3mf.cpp 3446/3474), --load-custom-gcodes by the plate sliced,
        // and the Print reads the entry of the model's current plate
        // (Print.cpp 517-518; PrintApply.cpp 1558-1569 copies the index), so
        // the official CLI sets it per plate before slicing it
        // (BambuStudio.cpp 6493; OrcaSlicer.cpp 5617). Left at 0, every plate
        // took plate 1's. The pressure-advance pattern writes its entry at
        // the same index (calib.cpp 655).
        model.curr_plate_index = std::max(0, plate_id - 1);

        // Enable BBL printer features (M981 spaghetti detector, M1003 powerlost
        // recovery, etc.) for a printer of the Bambu Lab vendor.
#ifdef ENGINE_BAMBU
        // set_BBL_Printer is a BambuStudio-only Print method (enables M981/M1003
        // BBL printer features).  OrcaSlicer's Print has no such method.
        // The desktop decides from the printer preset's vendor
        // (BackgroundSlicingProcess.cpp:205, Preset::is_bbl_vendor_preset), and
        // the official CLI from the plate config's printer_model, else the
        // loaded or the project's printer name (BambuStudio.cpp 7055-7070 at
        // 5873b5f): the same rule for a 3MF, a named preset or settings files.
        {
            const std::string printer_model_string = config.opt_string("printer_model", true);
            bool is_bbl_vendor_preset = false;
            if (!printer_model_string.empty())
                is_bbl_vendor_preset = printer_model_string.compare(0, 9, "Bambu Lab") == 0;
            else if (!rs.settings_merge.new_printer_name.empty())
                is_bbl_vendor_preset = rs.settings_merge.new_printer_name.compare(0, 9, "Bambu Lab") == 0;
            else if (!rs.project_facts.current_printer_system_name.empty())
                is_bbl_vendor_preset = rs.project_facts.current_printer_system_name.compare(0, 9, "Bambu Lab") == 0;
            print.set_BBL_Printer(is_bbl_vendor_preset);
        }
#endif
#ifdef ENGINE_ORCA
        // Orca's Print::m_isBBLPrinter (Print.hpp:1143) has NO default initializer and
        // no set_BBL_Printer() method. Left uninitialized it is read with an
        // indeterminate value (UB) across the slice/export phases — driving both the
        // wipe_tower_type() Type1/Type2 choice (Print.hpp:1072), the GCode dialect
        // (GCode.cpp:2046), and the ;Z_HEIGHT vs ;Z: comment (GCode.cpp:4486). The
        // inconsistent reads make process()'s wipe-tower tool ordering disagree with the
        // sequence read at export → "append_tcr ... toolchange it didn't expect".
        // Mirror OrcaSlicer's own headless CLI (OrcaSlicer.cpp:5972-5986): the flag is
        // true iff the printer_model preset begins with "Bambu Lab" — false for the
        // Snapmaker U1, which selects Type2 + the Orca/Prusa-style gcode dialect (golden B).
        {
            const std::string pm = config.opt_string("printer_model", true);
            print.is_BBL_printer() = (pm.rfind("Bambu Lab", 0) == 0);
        }
#endif

        // The plate's Print in the plate's own frame. The official CLI slices
        // each plate of the scene model with a Print whose origin is the
        // plate's (PartPlate::get_print and the plate's BuildVolume,
        // BambuStudio.cpp 6506-6529; OrcaSlicer.cpp 5623-5646; the origin from
        // set_plate_origin, PartPlate.cpp 2376/2517 at 5873b5f, 2311/2433 at
        // 31f6803). Here the Print's model is the plate's instances moved by
        // minus the plate's origin (PlateScope at print.apply below) and the
        // Print's origin stays 0: the same placement, and the G-code this
        // command line has always written, which the product relies on. The
        // two differ in bytes on a plate past the first: the engine fits arcs
        // on the Print's own coordinates (arc centres round differently) and
        // starts the first layer's moves from (0,0) of that frame (the first
        // moves and the object order change). Measured on the product corpus
        // for PR #35 with the scene-frame build kept as pkg-origin: object
        // order changed on 11 of 50 plates past the first, moves on 16 beyond
        // the engine's run-to-run noise; extents and settings unchanged.
        const Slic3r::Vec2d plate_origin = Slic3r::Vec2d::Zero();   // the Print's origin
        print.set_plate_origin(Slic3r::Vec3d(0.0, 0.0, 0.0));
        // The plate's own per-plate values (wipe_tower_x/y are one entry per
        // plate, read with get_at(m_plate_index): Print.cpp 2213, GCode.cpp 5006)
        // need the plate's index, as the desktop sets it (PartPlate.cpp:2299).
        // Left at 0, plate N's prime tower stood where plate 1's does.
        if (plate_id > 0)
            print.set_plate_index(plate_id - 1);

        // Alias the unbound `initial_no_support_filament_id` placeholder to the
        // engine-bound `initial_no_support_extruder` across custom-gcode keys, BEFORE
        // print.apply snapshots the config. Without this the PlaceholderParser throws at
        // export when a 3MF carries the legacy token in its custom gcode. Always on;
        // suppressed with --no-normalize-legacy-gcode.
        if (normalize_legacy_gcode) {
            normalize_legacy_gcode_tokens(config);
        }

        // Calibration: whether the active printer speaks the Bambu G-code dialect.
        // Mirrors the is_BBL_printer() derivation used below for the engine.
        const bool calib_is_bbl_machine =
            config.opt_string("printer_model", true).rfind("Bambu Lab", 0) == 0;

        // Calibration: pressure_advance_pattern generates its own geometry + per-layer
        // custom G-code. This MUST run before print.apply (apply snapshots the
        // model + config and reads plates_custom_gcodes). `calib_params` is a
        // main-scope local, so the reference held by model.calib_pa_pattern
        // stays valid through the whole slice.
        if (calib_params.mode == Slic3r::CalibMode::Calib_PA_Pattern) {
            std::cout << "Generating pressure-advance pattern geometry...\n";
            slicer_cli::apply_pa_pattern(calib_params, config, model, calib_is_bbl_machine);
            // The pattern replaced the model, so the object STL files an
            // --export-stl after --slice writes carry its object's name
            // (obj_1_pa_pattern_handle.stl): the outputs still to be written
            // are checked again. One given before --slice has written the
            // loaded model's files already.
            rs.outputs_checked = false;
            if (!outputs_ok(/*after_slice_actions_only=*/true))
                return 1;
        }

#ifdef ENGINE_BAMBU
        // Align every per-filament array with the filament roster the project
        // loaded (see align_per_filament_config_vectors above), using the count
        // captured before the normalization pass padded vector options.  This is
        // the one and only alignment, and it is the last config write before
        // print.apply() snapshots the config, so every layer that can reshape a
        // per-filament array is already in place: the plate map overlay and the
        // nozzle-map derivation, the legacy-gcode token aliasing, the calibration
        // pattern, and the command-line overrides — an override such as --temp
        // rewrites nozzle_temperature down to a single value, so aligning any
        // earlier would leave that array short again.
        align_per_filament_config_vectors(config, project_filaments, verbose);
#endif // ENGINE_BAMBU

        // Automatic filament grouping on a machine with several extruders sizes
        // each nozzle's group from extruder_ams_count (FilamentGroupUtils.cpp
        // calc_max_group_size, both engines). The desktop app takes that count
        // from the printer it is connected to, else from the project it opened,
        // and builds no AMS slots without a printer (BambuStudio PresetBundle.cpp
        // 3826-3831 and 3547, Plater.cpp 10810-10826 at 02.08.02.61; OrcaSlicer
        // PresetBundle.cpp 4375-4379 and 4162 at 2.4.2). Headless there is no
        // printer, so the input's own count stands, and input without one groups
        // as the desktop does with an empty count. The official CLIs instead
        // give every nozzle an estimated 4-slot AMS (BambuStudio.cpp 6911-6950,
        // OrcaSlicer.cpp 5914-5951), which regroups a project the maker sliced
        // with another count.

        try {
            std::cout << "Applying configuration...\n";
#ifdef ENGINE_ORCA
            // Last write before Print::apply snapshots the config: size every
            // filament vector to the roster the project declares, the way the
            // desktop's Preset::normalize does. Print::get_physical_unprintable_
            // filaments reads filament_printable unchecked
            // (Print.cpp:3229 at v2.4.2); the Bambu build's own alignment
            // (align_per_filament_config_vectors above) already ran on its path.
            if (size_t grown = normalize_filament_config_vectors(config); grown > 0 && verbose)
                std::cout << "Filament vectors: " << grown << " array(s) sized to the roster\n";
#endif
#ifdef ENGINE_SNAPMAKER
            // The plate's default tower corner for a run from a model file
            // (cli_snapmaker.hpp: when nothing else placed the tower).
            // wipe_tower_x/y are official setting flags, so a value from the
            // command line shows up in the given-flag set, not in `overrides`.
            const bool user_set_tower = o.given_flag("wipe_tower_x") || o.given_flag("wipe_tower_y");
            if (slicer_cli::snapmaker_tower_default_applies(config, !boost::algorithm::iends_with(input_file, ".3mf"),
                                                            user_set_tower, o.given_flag("arrange"))) {
                // The same writer the assemble and fixed-plate paths use, so the
                // corner and its margin live in one place.
                assemble_tower_default_corner(context, config, std::max(0, plate_id - 1),
                                              WIPE_TOWER_MARGIN + arrange_opt(config, "prime_tower_width")->getFloat());
            }
#endif
#ifdef ENGINE_SNAPMAKER
            // A switch to a printer that is not a Snapmaker U1 trims the filament
            // list to that printer's extruder count and leaves the painted ids
            // clamped to it (references/SnapmakerOrca/src/slic3r/GUI/Tab.cpp:2247-2270
            // and Plater.cpp:22281-22290 at bac6fd5: on_filaments_change runs
            // remap_extruder_ids, else update_extruder_count). The staged 3MF path
            // never reaches resolve_named_presets' setters, so the trim happens
            // here, on the settings this plate slices with, with the same rule.
            if (slicer_cli::switch_trims_filaments(o.preset_switch, o.printer_preset))
                slicer_cli::trim_filaments_for_switch(config, model, o.printer_preset, plate_id, rs.switch_paint_remapped);
#endif
#ifdef ENGINE_SNAPMAKER
            // The desktop's flow-type sync and the U1's plate types change the
            // settings the slice runs with, so they come before Print::apply,
            // which snapshots the config.
            {
                // The filaments this plate's objects use, as the dialog lists
                // them (FilamentGroupDialog.cpp 352-372).
                std::vector<int> plate_filaments;
                {
                    PlateScope scope(model, rs.plate(plate_id));
                    plate_filaments = arrange_plate_extruders(model, config, std::max(0, plate_id - 1));
                }
                const std::string why = slicer_cli::sync_filament_volume_types(
                    config, plate_id, std::set<int>(plate_filaments.begin(), plate_filaments.end()),
                    extra.has("filament_volume_type"));
                if (!why.empty()) {
                    std::cerr << "Error: " << why << "\n";
                    set_outcome_failure(outcome, CLI_INVALID_PARAMS, why);
                    return 1;
                }
            }
            slicer_cli::force_u1_bed_type(config, plate_id);
#endif
            {
                // The plate's instances only, on the plate (see the Print's
                // origin above). Print::apply copies the model it is given
                // (PrintApply.cpp 1553 / 1298), so the run's model stays in
                // the scene.
                PlateScope scope(model, rs.plate(plate_id));
                // Every height range carries a layer height before the engine
                // builds the layer height profile: it reads one out of each range
                // without a check (Slicing.cpp layer_height_profile_from_ranges,
                // `it_range->second.option("layer_height")->getFloat()`), and a
                // range from an assemble list or a 3MF that states
                // only its settings would crash the slice. A range that states none
                // keeps the object's height, so nothing about the print changes.
                // Filled for this plate's objects with this plate's settings, and
                // taken out again when the scope ends: Print::apply copies the
                // model, so the print keeps the heights, while the run's model
                // stays as it was read for the next plate and the next pass.
                struct RangeFill {
                    std::vector<slicer_cli::FilledRangeHeight> filled;
                    ~RangeFill() { slicer_cli::clear_filled_range_layer_heights(filled); }
                } range_fill{slicer_cli::fill_range_layer_heights(model, config)};
                const auto mm = [](double v) { std::ostringstream s; s << v; return s.str(); };
                for (const slicer_cli::FilledRangeHeight& filled : range_fill.filled)
                    emit_event({{"event", "config_normalized"}, {"tag", "RangeLayerHeightFilled"},
                                {"object", filled.object},
                                {"min_z", filled.min_z}, {"max_z", filled.max_z},
                                {"layer_height", filled.layer_height},
                                {"message", "The height range " + mm(filled.min_z) + "-" + mm(filled.max_z) +
                                                " mm of '" + filled.object +
                                                "' states no layer height; it keeps the object's " +
                                                mm(filled.layer_height) + " mm."}});
                print.apply(model, config);
#ifdef ENGINE_ORCA
                // The Orca desktop applies again before Slice (Plater.cpp 15855/15893 at v2.4.2).
                // On Bambu a second apply moves the output away from the desktop's own G-code (7
                // corpus files with the desktop's G-code keep enable_prime_tower=1), so not there.
                // Applied again, as the desktop does before every slice: its
                // background-process timer applies (OrcaSlicer Plater.cpp
                // 5035-5038, 7983 at v2.4.2), then Slice applies once more
                // (reslice, 15855 -> 15893). Print::apply counts the used
                // filaments (PrintApply.cpp 1131, 1620) before it builds the
                // regions (1730), so on the first apply a filament used only
                // through a feature setting (top_surface_filament_id and the
                // like, PrintRegion.cpp 89-90) is not counted and
                // normalize_fdm_2 turns the prime tower off (PrintConfig.cpp
                // 8692); on the second the regions exist and the tower stays
                // as the settings state it. The official CLI applies once
                // (OrcaSlicer.cpp 6040) and drops the tower. Nothing else of
                // the model or the settings changes between the two calls.
                // What the second call changes in the settings the print runs
                // with is told (PrintReapplied); when it changes nothing, the
                // print is the one the first call made.
                const Slic3r::PrintConfig first_config = print.config();
                print.apply(model, config);
                const Slic3r::t_config_option_keys reapplied = first_config.diff(print.config());
                if (!reapplied.empty()) {
                    json values = json::object();
                    for (const std::string& key : reapplied)
                        values[key] = print.config().opt_serialize(key);
                    emit_event({{"event","config_normalized"}, {"tag","PrintReapplied"},
                                {"plate_id", plate_id}, {"settings", values},
                                {"message","The settings applied a second time before slicing, as the desktop "
                                           "app does, changed " + boost::algorithm::join(reapplied, ", ") +
                                           ": the plate's filaments are counted with its regions"}});
                }
#endif
                // What the load replaced and the print, after its last apply,
                // used other than the file's value (cli_project_load.hpp). The
                // setting flags count by the settings they write: a short flag
                // writes other keys than its own name (--infill writes
                // sparse_infill_density, --bed-temp the plate type's keys).
                if (!outcome.settings_replaced_by_presets.empty()) {
                    std::set<std::string> command_line;
                    for (const std::string& key : rs.extra.keys())
                        command_line.insert(key);
                    Slic3r::DynamicPrintConfig written;
                    if (const Slic3r::ConfigOption* bed = config.option("curr_bed_type"))
                        written.set_key_value("curr_bed_type", bed->clone());
                    apply_command_line_overrides(written, o.overrides, /*report_rejections=*/false);
                    for (const std::string& key : written.keys())
                        if (key != "curr_bed_type" || o.overrides.count(key) != 0)
                            command_line.insert(key);
                    for (const auto& override_value : o.overrides)
                        command_line.insert(override_value.first);
                    slicer_cli::keep_replaced_printed_differently(outcome.settings_replaced_by_presets,
                                                                  outcome.replaced_file_values,
                                                                  print.full_print_config(), command_line);
                }
            }
#ifdef ENGINE_ORCA
            // OrcaSlicer's WipeTowerData::height is read on a path that never
            // writes it, and a value out of Clipper's range then reaches the
            // skirt. Nothing in the engine at 31f6803 initialises it: the struct
            // declares `float height;` with no initialiser (Print.hpp:763) and
            // its constructor, `WipeTowerData(ToolOrdering &) : tool_ordering(...)
            // { clear(); }` (Print.hpp:784), calls a clear() that resets depth,
            // brim_width and rib_offset but not height (Print.hpp:767-778); the
            // only writer, Print::_make_wipe_tower, sets it in its WipeTower2
            // (Type-2) branch alone (Print.cpp:3449), and a Bambu Lab printer
            // always takes the Type-1 branch (wipe_tower_type() is
            // is_BBL_printer() ? Type1 : m_config.wipe_tower_type.value,
            // Print.hpp:1072; is_wipe_tower_type2 at Print.cpp:3196, the
            // `if (!is_wipe_tower_type2)` branch at 3242, which sets depth,
            // brim_width, bbx and rib_offset at 3338-3342 and nothing else). No
            // other writer exists: OrcaSlicer.cpp never touches it, and the
            // desktop only reads it (GLCanvas3D.cpp:2874, 2894).
            // 0 is the value upstream itself gives this field: OrcaSlicer main's
            // WipeTowerData::clear() sets `height = 0.f;` beside the other
            // floats, which is exactly what our pin's clear() is missing. Set
            // here because the engine submodule cannot be patched.
            // Print::first_layer_wipe_tower_corners() reads it unconditionally
            // (Print.cpp:2838) as the height of the stabilization cone — a
            // WipeTower2-only feature, whose base radius is
            // tan(deg2rad(wipe_tower_cone_angle/2)) * height
            // (WipeTower2.cpp:2161-2175) — and adds the ring of points at that
            // radius to the skirt's hull (Print.cpp:2840-2844, appended at
            // 2647), so an indeterminate height puts a point past Clipper's
            // range and Print::_make_skirt's offset() throws
            // clipperException "Coordinate outside allowed range"
            // (Print.cpp:2695, ClipperBase::AddPathInternal). The Type-1 tower
            // (GCode/WipeTower.cpp) has no cone at all, so 0 is also the radius
            // its path means: the ring collapses onto the tower's centre and the
            // tower's own bbox, already in the hull, is unchanged. Measured on
            // the packaged macOS Orca engine of run 37538132874: the E4 project's
            // plate 2 either slices or dies with that exception run to run, and
            // setting wipe_tower_cone_angle to 0 makes every failing input slice.
#ifdef ENGINE_SNAPMAKER
            // Snapmaker Orca has no WipeTowerType/wipe_tower_type() (absent at
            // bac6fd5) and always builds the Type-2 tower
            // (`bUseWipeTower2 = is_BBL_printer() ? false : true`,
            // references/SnapmakerOrca/src/libslic3r/Print.cpp:3221), whose
            // branch writes WipeTowerData::height — so the workaround below,
            // which is for the Type-1 path, does not apply.
#else
            if (print.has_wipe_tower() && print.wipe_tower_type() == Slic3r::WipeTowerType::Type1)
                const_cast<Slic3r::WipeTowerData&>(print.wipe_tower_data()).height = 0.f;
#endif
#endif

            // Calibration: install the calibration params after apply (which resets print
            // state) and before validate/process so the engine's per-layer calib
            // emission (GCode.cpp) and PA-line/pattern paths see the mode.
            if (calib_enabled) {
                print.set_calib_params(calib_params);
                std::cout << "Calibration mode: "
                          << slicer_cli::calib_mode_name(calib_params.mode)
                          << " [start=" << calib_params.start
                          << " end=" << calib_params.end
                          << " step=" << calib_params.step << "]\n";
            }

            // Install the structured-warning emitter BEFORE validate() / process()
            // so the TS host sees pre-slice diagnostics in the same JSON-line
            // stream as slicing-time warnings. Pure stdout side-effect — does
            // not change print state, exit codes, or G-code output.
            // --pipe: the steps before the first plate (BambuStudio.cpp
            // 1765-1767, 4923-4926, 6455-6458: 1 at the start, 2 once the
            // files are loaded, 3 before the plates), then this plate
            // (7034-7046: its index and the run's plate count, 4 to begin),
            // and every status the slice reports (cli_status_callback).
            const bool piped = slicer_cli::pipe_started();
            // The check pass sends no plate step: the official reports a plate
            // only once it slices it (BambuStudio.cpp 7034-7046, after the
            // check pass's continue at 7027).
            if (piped && !outcome.pre_check) {
                pipe_prepare_slicing(rs);   // once per run: here only for a run that skipped the plate checks
                slicer_cli::pipe_set_plate_info(std::max(1, outcome.plate_id), std::max(1, outcome.plate_count));
                slicer_cli::pipe_update(4, "Slicing begins");
            }
            if (o.progress || piped) {
                print.set_status_callback([&outcome, &o, piped](const Slic3r::PrintBase::SlicingStatus& status) {
                    if (piped)
                        slicer_cli::pipe_update(status.percent, status.text, status.warning_step);
                    emit_status_warning(status);
                    if (!o.progress)
                        return;
                    using FB = Slic3r::PrintBase::SlicingStatus::FlagBits;
                    const bool is_warning =
                        (status.flags & (FB::UPDATE_PRINT_STEP_WARNINGS | FB::UPDATE_PRINT_OBJECT_STEP_WARNINGS)) != 0;
                    if (status.warning_step != -1 || is_warning) {
                        // The official CLI keeps every warning (g_slicing_warnings,
                        // cli_status_callback) and reports the last one per plate.
                        if (!status.text.empty() &&
                            status.message_type != Slic3r::PrintStateBase::SlicingDefaultNotification) {
                            outcome.warning_message = status.text;
                            outcome.warnings.push_back(json{{"message", status.text},
                                                            {"level", warning_level_tag(status.warning_level)},
                                                            {"tag", slicing_notification_tag(static_cast<int>(status.message_type))}});
                        }
                        return;
                    }
                    if (status.percent >= 0)
                        emit_progress(outcome, status.percent, status.text);
                });
            } else {
                print.set_status_callback(emit_status_warning);
            }

            // The per-filament and speed tables the brim width reads
            // (Brim.cpp, through Model::extruderParamsMap and
            // printSpeedMap): both official CLIs fill them between apply()
            // and process() (BambuStudio.cpp 7074-7075 at 5873b5f;
            // OrcaSlicer.cpp 6065-6066 at 31f6803). Left empty, the brim
            // takes no account of the filament (TPU narrower, PETG/PCTG
            // wider) or of the print speed.
            {
                int filament_count = 1;
                if (auto* fc = config.option<Slic3r::ConfigOptionStrings>("filament_colour", false))
                    filament_count = std::max<int>(1, (int)fc->values.size());
                Slic3r::Model::setExtruderParams(config, filament_count);
                Slic3r::Model::setPrintSpeedTable(config, print.config());
#ifdef ENGINE_ORCA
                std::cout << "  [orca] setExtruderParams/setPrintSpeedTable (filaments="
                          << filament_count << ")\n";
#endif
            }
            // --no-check and --allow-mix-temp (BambuStudio.cpp 6979-6982;
            // OrcaSlicer.cpp 5968-5971); only when given, so the default
            // stays the engine's own.
            const slicer_cli::PlateLoopSwitches switches = slicer_cli::plate_loop_switches(o);
            outcome.no_check = switches.no_check;
            if (o.given_flag("no_check"))
                print.set_no_check_flag(switches.no_check);
#ifdef ENGINE_SNAPMAKER
            // Snapmaker Orca has no setter for this check (no
            // set_check_multi_filaments_compatibility in its Print at
            // bac6fd5, Print.hpp:1020), so --allow_mix_temp has no engine hook.
#elif defined(ENGINE_BAMBU)
            // Bambu 2.9.1 dropped the plate-level high/low temperature mix check and
            // its setter (no set_check_multi_filaments_compatibility in Print at
            // v02.09.01.70; the official CLI parses allow_mix_temp and no longer uses
            // it). The flag is kept for callers that pass it; the run says so once,
            // at its start (AllowMixTempIgnored, run_cli_slice).
#else
            if (o.given_flag("allow_mix_temp"))
                print.set_check_multi_filaments_compatibility(!switches.allow_mix_temp);
#endif
#ifdef ENGINE_SNAPMAKER
            // The desktop's notes after apply and its PLA/PETG warning; neither
            // stops the slice.
            slicer_cli::post_apply_filament_notes(print, config, plate_id);
#endif
#ifdef ENGINE_SNAPMAKER
            // The Snapmaker desktop's own pre-slice blockers
            // (references/SnapmakerOrca/src/slic3r/GUI/Plater.cpp:21485-21530 at
            // bac6fd5): the high/low-temperature mix, the Cool Steel Plate, then
            // the zero flow ratio. They replace the OrcaSlicer CLI's type-based
            // Print::check_multi_filaments_compatibility, which the desktop does
            // not use. --allow-mix-temp lifts the temperature rule only.
            {
                int blocker_code = 0;
                std::string blocker_message;
                // Only this plate's objects count, as the desktop's
                // collect_used_filament_slots_on_plate counts them.
                int blocked = 0;
                {
                    PlateScope scope(model, rs.plate(plate_id));
                    blocked = slicer_cli::snapmaker_pre_slice_check(model, config, plate_id, switches.allow_mix_temp,
                                                                    blocker_code, blocker_message);
                }
                if (blocked != 0) {
                    std::cerr << blocker_message << "\n";
                    set_outcome_failure(outcome, blocker_code, blocker_message);
                    return 1;
                }
            }
#endif

#ifdef ENGINE_BAMBU
            // Mixed nozzle diameters (BambuStudio.cpp 5066-5086 at v02.09.01.70):
            // with multi_nozzle_diameter_mode on (the project's, or the command
            // line's), a plate whose filaments all
            // print with one nozzle diameter is refused (the single-diameter
            // map was refused in the settings merge).
            if (Slic3r::multi_nozzle_diameter_mode_enabled(config)) {
                // Only this plate's objects count, and the plate's tool changes
                // are keyed 0-based (plates_custom_gcodes, bbs_3mf.cpp 3872).
                // A model file has one plate, numbered 1 (plate_id 0 here, as
                // curr_plate_index = max(0, plate_id - 1) below).
                const int plate_number = std::max(1, plate_id);
                std::vector<int> plate_filaments;
                {
                    PlateScope scope(model, rs.plate(plate_id));
                    plate_filaments = arrange_plate_extruders(model, config, plate_number - 1);
                }
                const std::string single_diameter = slicer_cli::plate_single_nozzle_diameter(config, plate_filaments);
                if (!single_diameter.empty()) {
                    const std::string detail = "Plate " + std::to_string(plate_number) + " uses only nozzle diameter " +
                                               single_diameter + " with multi_nozzle_diameter_mode on.";
                    emit_event({{"event","config_refused"},
                                {"tag","PlateSingleNozzleDiameter"},
                                {"plate", plate_number},
                                {"nozzle_diameter", single_diameter},
                                {"message", cli_error_sentence(CLI_MULTI_NOZZLE_DIAMETER_INVALID) + " " + detail}});
                    std::cerr << "Error: " << cli_error_sentence(CLI_MULTI_NOZZLE_DIAMETER_INVALID) << " " << detail << "\n";
                    set_outcome_failure(outcome, CLI_MULTI_NOZZLE_DIAMETER_INVALID, detail);
                    return 1;
                }
            }
#endif
            std::cout << "Validating...\n";
            // Option C — pass a warning out-pointer so BBS routes is_warning-
            // flagged exceptions there instead of into the return value
            // (mirrors `Plater.cpp:9888`'s call signature). Hard validation
            // errors still come back as the return value and remain fatal.
            // Soft warnings get emitted as JSON events and slicing proceeds —
            // matching what BBS GUI does in this case.
            Slic3r::StringObjectException validation_warning;
            // The GUI calls validate() with all three out-params
            // (Plater.cpp:10835) and paints the returned hulls on the plate;
            // a human sees the collision as geometry, with no text anywhere.
            // Passing them here is the only way an agent can learn WHERE a
            // sequential-print clearance violation is, rather than just that
            // one happened.
            Slic3r::Polygons collision_polygons;
            std::vector<std::pair<Slic3r::Polygon, float>> height_polygons;
#ifdef ENGINE_BAMBU
            // Bambu 2.9.1 validate() returns every warning (Print.hpp 948 at
            // v02.09.01.70); the official CLI joins them with newlines (BambuStudio.cpp 7531-7536).
            std::vector<Slic3r::StringObjectException> validation_warnings;
            Slic3r::StringObjectException validation_result =
                print.validate(&validation_warnings, &collision_polygons, &height_polygons);
            for (const Slic3r::StringObjectException& item : validation_warnings) {
                if (validation_warning.string.empty()) validation_warning = item;
                else {
                    if (validation_warning.string.back() != '\n') validation_warning.string += "\n";
                    validation_warning.string += item.string;
                }
            }
#else
            Slic3r::StringObjectException validation_result =
                print.validate(&validation_warning, &collision_polygons, &height_polygons);
#endif
            if (!collision_polygons.empty() || !height_polygons.empty()) {
                json e;
                e["event"] = "clearance_violation";
                e["tag"]   = "SequentialPrintClearance";
                e["collision_hull_count"] = collision_polygons.size();
                e["height_hull_count"]    = height_polygons.size();
                json hulls = json::array();
                for (const auto& poly : collision_polygons) {
                    json points = json::array();
                    for (const auto& p : poly.points)
                        points.push_back(json{{"x_mm", Slic3r::unscaled<double>(p.x()) - plate_origin.x()},
                                              {"y_mm", Slic3r::unscaled<double>(p.y()) - plate_origin.y()}});
                    hulls.push_back(points);
                }
                if (!hulls.empty()) e["collision_hulls"] = hulls;
                json heights = json::array();
                for (const auto& [poly, height] : height_polygons) {
                    json points = json::array();
                    for (const auto& p : poly.points)
                        points.push_back(json{{"x_mm", Slic3r::unscaled<double>(p.x()) - plate_origin.x()},
                                              {"y_mm", Slic3r::unscaled<double>(p.y()) - plate_origin.y()}});
                    heights.push_back(json{{"height_mm", height},
                                           {"point_count", poly.points.size()},
                                           {"points", points}});
                }
                if (!heights.empty()) e["height_hulls"] = heights;
                e["message"] = "Object clearance hulls overlap; the GUI would paint these regions red on the plate";
                emit_event(e);
            }
            if (!validation_warning.string.empty()) {
                emit_validation_event(validation_warning);
                std::cout << "Validation warning: " << validation_warning.string << "\n";
            }
            // --no-check lets a layer height over the printer's limit pass
            // as a warning (BambuStudio.cpp 6985-6987; OrcaSlicer.cpp 5992-5994).
            if (!validation_result.string.empty() && switches.no_check &&
                validation_result.type == Slic3r::STRING_EXCEPT_LAYER_HEIGHT_EXCEEDS_LIMIT) {
                emit_validation_event(validation_result);
                std::cout << "Validation warning: " << validation_result.string << "\n";
                validation_result.string.clear();
            }
            if (!validation_result.string.empty()) {
                emit_validation_event(validation_result);
                std::cerr << "Validation error: " << validation_result.string << "\n";
                std::cerr << "\nTip: You may need to specify proper config files:\n";
                std::cerr << "  --machine, --filament, and --process options\n";
                // The official CLI's mapping of validate() errors to result
                // codes (BambuStudio.cpp 7094-7111 at 5873b5f).
                int validate_error = CLI_VALIDATE_ERROR;
                switch (validation_result.type) {
                    case Slic3r::STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE:      validate_error = CLI_FILAMENT_NOT_MATCH_BED_TYPE; break;
#ifndef ENGINE_SNAPMAKER
                    case Slic3r::STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP:         validate_error = CLI_FILAMENTS_DIFFERENT_TEMP; break;
#else
                    // Snapmaker Orca calls it STRING_EXCEPT_FILAMENTS_MIXING_TEMP
                    // (references/SnapmakerOrca/src/libslic3r/PrintBase.hpp:1-12
                    // at bac6fd5); the same situation, the same code.
                    case Slic3r::STRING_EXCEPT_FILAMENTS_MIXING_TEMP:            validate_error = CLI_FILAMENTS_DIFFERENT_TEMP; break;
#endif
                    case Slic3r::STRING_EXCEPT_OBJECT_COLLISION_IN_SEQ_PRINT:    validate_error = CLI_OBJECT_COLLISION_IN_SEQ_PRINT; break;
                    case Slic3r::STRING_EXCEPT_OBJECT_COLLISION_IN_LAYER_PRINT:  validate_error = CLI_OBJECT_COLLISION_IN_LAYER_PRINT; break;
                    default: break;
                }
                // The engine's own sentence names the symptom ("Add \"G92 E0\"
                // to layer_gcode"). When the run has no layer change G-code at
                // all, the cause is that one: the settings files state none, so
                // the merge wrote the key empty (load_default_gcodes_to_config,
                // BambuStudio.cpp 685-745 at 5873b5f / OrcaSlicer.cpp 548-604 at
                // 31f6803 makes every G-code key the file leaves out an empty
                // string). Name it, so the refusal says what to give instead of
                // what to edit in a file the user does not have.
                std::string message = validation_result.string;
                if (message.rfind("Relative extruder addressing", 0) == 0) {
                    const auto* layer_gcode = config.option<Slic3r::ConfigOptionString>("layer_change_gcode");
                    if (layer_gcode == nullptr || layer_gcode->value.empty())
                        message += " This run has no layer change G-code at all: the machine settings given state no"
                                   " layer_change_gcode, so the settings merge wrote it empty. Give a machine settings"
                                   " file that states its layer_change_gcode, or the printer preset"
                                   " (--printer-preset).";
                }
                // The engine's own sentence names the plate and the filament
                // and stops there ("Plate 1: Cool Plate does not support
                // filament 2"). Its check reads the plate's own bed temperature
                // key with get_at and reads 0 as "not this filament's plate", for every
                // filament the run prints with (Print.cpp 1700-1726 at 31f6803;
                // Print.cpp 1650-1672 at 5873b5f; the filament list is
                // Print::extruders(), Print.cpp 1265 / 1307; get_bed_temp_key,
                // OrcaSlicer PrintConfig.hpp 466-490 / BambuStudio
                // PrintConfig.hpp 438-456). Name the plates whose key is not 0
                // for every one of them — the plates this run can actually
                // print on — so the refusal says what to give instead.
                if (validation_result.type == Slic3r::STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE) {
                    const std::vector<unsigned int> used = print.extruders();
                    const Slic3r::ConfigOptionDef*    bed_def = Slic3r::print_config_def.get("curr_bed_type");
                    std::vector<std::string>          plates;
                    if (!used.empty() && bed_def != nullptr && bed_def->enum_keys_map != nullptr) {
                        // enum_values is the plate list in the definition's own
                        // order (PrintConfig.cpp 1049-1053 at 31f6803,
                        // 1150-1154 at 5873b5f), so the list is stable.
                        for (const std::string& plate : bed_def->enum_values) {
                            const auto named = bed_def->enum_keys_map->find(plate);
                            if (named == bed_def->enum_keys_map->end())
                                continue;
                            const std::string temp_key =
                                Slic3r::get_bed_temp_key(static_cast<Slic3r::BedType>(named->second));
                            if (temp_key.empty())
                                continue;
                            const auto* temps = config.option<Slic3r::ConfigOptionInts>(temp_key);
                            if (temps == nullptr || temps->values.empty())
                                continue;
                            // Read as the check reads it: get_at, so a list with
                            // one value is that value for every filament.
                            bool every_filament = true;
                            for (unsigned int filament : used)
                                if (temps->get_at(filament) == 0) {
                                    every_filament = false;
                                    break;
                                }
                            if (every_filament)
                                plates.push_back(plate);
                        }
                    }
                    if (plates.empty())
                        message += " No plate this engine knows gives every filament of this run a bed temperature:"
                                   " check the plate temperatures in the filament presets (--filament-preset).";
                    else {
                        message += " Pick a plate this filament supports: --curr-bed-type \"" + plates.front() + "\"";
                        if (plates.size() > 1) {
                            message += " (or";
                            for (size_t i = 1; i < plates.size(); i++)
                                message += (i > 1 ? ", " : " ") + std::string("\"") + plates[i] + "\"";
                            message += ")";
                        }
                        message += ".";
                    }
                }
                set_outcome_failure(outcome, validate_error, message);
                return 1;
            }
            // --slice 0 on several plates: the check pass ends here for this
            // plate, once its bed check, apply and validate passed
            // (BambuStudio.cpp 7027-7028; OrcaSlicer.cpp 6034-6035).
            if (outcome.pre_check)
                return 0;

            // --load-slicedata: the plate's saved slicing, else a normal
            // slice (BambuStudio.cpp 7076-7099; OrcaSlicer.cpp 6067-6090).
            const int cache_plate = plate_id > 0 ? plate_id : 1;
            const std::string load_dir = slicer_cli::slicedata_dir(o, "load_slicedata", cache_plate);
#ifdef ENGINE_SNAPMAKER
            // Again here: the project rebase and the desktop plate type load
            // presets after the call at the top of this function, and each
            // points data_dir() at a staging folder its scope then removes.
            slicer_cli::snapmaker_keep_data_dir(run_backup_path(context));
#endif
            std::cout << "Slicing...\n";
            if (!load_dir.empty() && print.load_cached_data(load_dir) == 0) {
                emit_event({{"event","model_loaded"}, {"tag","SliceDataLoaded"}, {"path", load_dir},
                            {"message","Loaded the saved slicing of plate " + std::to_string(cache_plate)}});
                slicer_cli::pipe_update(69, "Cache data loaded");   // BambuStudio.cpp 7087-7090
                print.process(nullptr, true);
            } else {
                if (!load_dir.empty())
                    emit_event({{"event","model_loaded"}, {"tag","SliceDataNotLoaded"}, {"path", load_dir},
                                {"message","No usable saved slicing for plate " + std::to_string(cache_plate) + "; slicing it normally"}});
                print.process();
            }

            std::cout << "\n✓ Slicing complete!\n";

            // --slice: toolpath conflicts stop the plate before any G-code
            // is written, as the official CLI checks them right after
            // process() and exits before export_gcode (BambuStudio.cpp
            // 7111-7116 at 5873b5f). Print::process() sets the conflict
            // result in both engines (Print.cpp 2396 / 2553).
            if (o.slice_mode) {
                const std::string conflict = print.get_conflict_string();
                if (!conflict.empty()) {
                    set_outcome_failure(outcome, CLI_GCODE_PATH_CONFLICTS, conflict + ".");
                    emit_event({{"event","plate_error"}, {"tag","GcodePathConflicts"}, {"message", conflict}});
                    return 1;
                }
            }

            // Attempt G-code export
            std::cout << "\nExporting G-code to: " << output_file << "\n";

            try {
                /// Export G-code with a result object (must not be nullptr — GCode.cpp:1738 dereferences it)
                /// Print.hpp:848
                /// C++: std::string export_gcode(const std::string &path, GCodeProcessorResult* result, ThumbnailsGeneratorCallback thumbnail_cb);
                Slic3r::GCodeProcessorResult gcode_result;
                // GCode::do_export makes the file's folder when it is missing and
                // throws on a bare file name, whose folder is "" (GCode.cpp
                // 1793-1797 at 5873b5f): the current folder is named for it.
                const std::string export_path = boost::filesystem::path(output_file).has_parent_path()
                                                    ? output_file : (boost::filesystem::path(".") / output_file).string();
                print.export_gcode(export_path, &gcode_result, nullptr);

                // The result object is fully populated by the export. Drain
                // every check the GUI would show a human before reporting
                // success — a run can exit 0 and still carry warnings,
                // toolpath conflicts or printable-area failures.
                emit_gcode_result_diagnostics(gcode_result);
                if (o.slice_mode) {
                    record_plate_statistics(print, model, config, rs.base, gcode_result, outcome);
                    outcome.sliced_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - plate_started).count();
                    // --mstpp: a plate that took longer ends the run, listed with
                    // its note (BambuStudio.cpp 7364-7373, a limit in seconds;
                    // OrcaSlicer.cpp 6202-6211 compares the milliseconds with the
                    // value as given).
                    if (switches.max_slicing_time_s != 0) {
#ifdef ENGINE_ORCA
                        const long long limit = switches.max_slicing_time_s;
#else
                        const long long limit = (long long)switches.max_slicing_time_s * 1000;
#endif
                        if (outcome.sliced_time_ms > limit) {
                            outcome.warning_message = "plate " + std::to_string(cache_plate) + "'s slice time " +
                                                      std::to_string(outcome.sliced_time_ms) + " exceeds the limit " +
                                                      std::to_string(limit) + ", return error.";
                            outcome.exported = true;
                            set_outcome_failure(outcome, CLI_SLICING_TIME_EXCEEDS_LIMIT);
                            return 1;
                        }
                    }
                    if (!post_slice_checks(print, model, gcode_result, outcome))
                        return 1;
                    outcome.exported = true;
                    if (!o.export_3mf.empty())
                        record_plate_for_export(context, print, model, config, gcode_result, output_file, outcome,
                                                plate_origin);
                    if (o.progress)
                        emit_progress(outcome, 100, "Slicing finished");
                    slicer_cli::pipe_update(100, "Slicing finished");   // BambuStudio.cpp 7242-7245
                }
                {
                    // --export-slicedata (BambuStudio.cpp 7247-7262; OrcaSlicer.cpp 6183-6199).
                    if (const std::string export_dir = slicer_cli::slicedata_dir(o, "export_slicedata", cache_plate);
                        !export_dir.empty()) {
                        const bool with_space = Slic3r::get_logging_level() >= 4;
                        // Print::export_cached_data makes the plate's folder
                        // only, so the folder given on the command line is made
                        // first (the official fails with -53 when it is missing).
                        {
                            boost::system::error_code mk;
                            boost::filesystem::create_directories(boost::filesystem::path(export_dir).parent_path(), mk);
                        }
#ifdef ENGINE_ORCA
                        const int ret = print.export_cached_data(export_dir, with_space);
#else
                        int obj_cached_cnt = 0;
                        const int ret = print.export_cached_data(export_dir, obj_cached_cnt, with_space);
#endif
                        if (ret != 0) {
                            boost::system::error_code rm;
                            boost::filesystem::remove_all(export_dir, rm);
                            set_outcome_failure(outcome, ret, "Could not write the slicing data to " + export_dir + ".");
                            return 1;
                        }
                        emit_event({{"event","export"}, {"tag","SliceDataExported"}, {"path", export_dir},
                                    {"message","Wrote the slicing data of plate " + std::to_string(cache_plate) + " to " + export_dir}});
                    }
                }

                std::cout << "✓ G-code export complete!\n";
                std::cout << "\nOutput file: " << output_file << "\n";

                // The actions given after --slice, once the last plate has
                // sliced (see run_model_action_step).
                if (o.slice_mode && outcome.plate_index == outcome.plate_count &&
                    slicer_cli::has_model_actions(o, slicer_cli::ActionPhase::AfterSlice) &&
                    !run_model_action_step(slicer_cli::ActionPhase::AfterSlice))
                    return 1;

            } catch (const std::bad_alloc&) {
                // Out of memory while slicing or exporting. The official CLI has
                // this outcome of its own -- CLI_OUT_OF_MEMORY (-14, Utils.hpp:39
                // at 31f6803) with the sentence that says what to do
                // (OrcaSlicer.cpp 126, BambuStudio.cpp 132) -- and it is not a
                // slicing failure: a run that ran out of memory must never
                // report CLI_SLICING_ERROR (-100).
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind","bad_alloc"},
                            {"message","std::bad_alloc"}});
                std::cerr << "\nOut of memory during slicing.\n";
                set_outcome_failure(outcome, CLI_OUT_OF_MEMORY);
                return 1;
            } catch (const Slic3r::RuntimeError& e) {
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind","RuntimeError"},{"message",std::string(e.what())}});
                slicer_cli::add_engine_reason(outcome.error_reason, e.what());
                std::cerr << "\n❌ G-code export failed (RuntimeError): " << e.what() << "\n";
                std::cerr << "\nSlicing succeeded, but export needs additional configuration.\n";
                return 1;
            } catch (const std::length_error& e) {
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind","length_error"},{"message",std::string(e.what())}});
                slicer_cli::add_engine_reason(outcome.error_reason, e.what());
                std::cerr << "\n❌ G-code export failed (length_error): " << e.what() << "\n";
                return 1;
            } catch (const std::exception& e) {
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind",typeid(e).name()},{"message",std::string(e.what())}});
                slicer_cli::add_engine_reason(outcome.error_reason, e.what());
                std::cerr << "\n❌ G-code export failed (" << typeid(e).name() << "): " << e.what() << "\n";
                std::cerr << "\nSlicing succeeded, but export needs additional configuration.\n";
                return 1;
            } catch (...) {
                emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind","unknown"},{"message","unknown exception"}});
                std::cerr << "\n❌ G-code export failed (unknown exception)\n";
                return 1;
            }

        } catch (const std::bad_alloc&) {
            // The slicing step ran out of memory (see the export step above):
            // this engine's own CLI has CLI_OUT_OF_MEMORY for it, never -100.
            emit_event({{"event","slicing_error"},{"phase","process"},{"kind","bad_alloc"},
                        {"message","std::bad_alloc"}});
            std::cerr << "\nOut of memory during slicing.\n";
            set_outcome_failure(outcome, CLI_OUT_OF_MEMORY);
            return 1;
        } catch (const Slic3r::RuntimeError& e) {
            emit_event({{"event","slicing_error"},{"phase","process"},{"kind","RuntimeError"},{"message",std::string(e.what())}});
            slicer_cli::add_engine_reason(outcome.error_reason, e.what());
            std::cerr << "Slicing RuntimeError: " << e.what() << "\n";
            std::cerr << "\nThis may indicate missing or incomplete configuration.\n";
            std::cerr << "Try using BambuStudio config files with --machine, --filament, --process\n";
            return 1;
        } catch (const std::length_error& e) {
            emit_event({{"event","slicing_error"},{"phase","process"},{"kind","length_error"},{"message",std::string(e.what())}});
            slicer_cli::add_engine_reason(outcome.error_reason, e.what());
            std::cerr << "Slicing length_error: " << e.what() << "\n";
            return 1;
        } catch (const std::exception& e) {
            emit_event({{"event","slicing_error"},{"phase","process"},{"kind",typeid(e).name()},{"message",std::string(e.what())}});
            slicer_cli::add_engine_reason(outcome.error_reason, e.what());
            std::cerr << "Slicing error (" << typeid(e).name() << "): " << e.what() << "\n";
            std::cerr << "\nThis may indicate missing or incomplete configuration.\n";
            std::cerr << "Try using BambuStudio config files with --machine, --filament, --process\n";
            return 1;
        } catch (...) {
            emit_event({{"event","slicing_error"},{"phase","process"},{"kind","unknown"},{"message","unknown exception"}});
            std::cerr << "Slicing error: unknown exception\n";
            return 1;
        }

        return 0;

    } catch (const std::bad_alloc&) {
        // Out of memory before or around the slicing steps (the load of a large
        // project, the arrange, the plate cleanup). The official outcome is
        // CLI_OUT_OF_MEMORY with its sentence (Utils.hpp:39, OrcaSlicer.cpp 126
        // at 31f6803), never CLI_SLICING_ERROR (-100): this is the run that must
        // tell the user to lower the geometry resolution, not report a slicing
        // failure.
        emit_event({{"event","slicing_error"},{"phase","load_or_prepare"},{"kind","bad_alloc"},
                    {"message","std::bad_alloc"}});
        std::cerr << "\nOut of memory during slicing.\n";
        set_outcome_failure(outcome, CLI_OUT_OF_MEMORY);
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}

} // namespace slicer_cli
