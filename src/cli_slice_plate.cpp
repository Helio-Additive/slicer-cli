#include "cli_slice_plate.hpp"
#include "cli_plate_settings.hpp"
#include "cli_plate_print.hpp"

#include "cli_run_context.hpp"
#include "cli_slice_state.hpp"
#include "cli_load_slice_inputs.hpp"
#include "cli_plate_scope.hpp"
#include "cli_options.hpp"
#include "cli_assemble.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/ModelArrange.hpp"
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
#include "cli_events.hpp"
#include "cli_engine_info.hpp"
#include "cli_filament_config.hpp"
#include "cli_filament_map.hpp"
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

static void carry_plate_outcome(const PlatePreparation& preparation, const PlateWarnings& plate_warnings,
            PlateOutcome& outcome, bool first_call, int plate_id) {
    if (!first_call) {
        // What the load and the run-level steps recorded, for this plate's result.
        PlateOutcome carried = preparation.carry;
        carried.plate_id    = outcome.plate_id;
        carried.plate_index = outcome.plate_index;
        carried.plate_count = outcome.plate_count;
        carried.gcode_path  = outcome.gcode_path;
        carried.pre_check   = outcome.pre_check;
        if (plate_id != preparation.prepared_plate) {
            // Another plate than the one the run-level steps ran for: its own
            // move to the new bed and none of that plate's arrange results.
            carried.derived_filament_map.clear();
            carried.derived_filament_map_mode.clear();
            carried.flag_plate_maps.clear();
        }
        outcome = carried;
        // The carry is the prepare call's outcome, with that plate's settings
        // warnings: this plate gets its own instead.
        take_noted_warnings(outcome, plate_warnings.noted, plate_id);
    }
}

static bool read_project_print_sequence(const Slic3r::DynamicPrintConfig& extra, const std::string& merged_print_sequence,
                                        bool assemble_input, const Slic3r::DynamicPrintConfig& config,
                                        const std::string& input_file, std::optional<bool>& project_sequence_read) {
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
}

static int check_downward_printers(CliRunContext& context, const CliOptions& o, RunPlatePlan& plate_plan,
            PlateOutcome& outcome, bool trailing_port, bool assemble_input, int plate_id,
            Slic3r::DynamicPrintConfig& config, LoadedProject& loaded_project, RunSettings& run_settings) {
    const auto& input_file = o.input_file;
    const auto& verbose = o.verbose;
    auto& model = loaded_project.model;
    auto& is_bbl_3mf = loaded_project.is_bbl_3mf;
    auto& plate_data = loaded_project.plate_data;
    auto& plate_origins = loaded_project.plate_origins;
    auto& project_facts = loaded_project.project_facts;
    auto& extra = run_settings.extra;
    auto& merged_print_sequence = run_settings.merged_print_sequence;

    // A plate's print sequence: its own, or else the project's
    // (get_print_sequence, BambuStudio.cpp 4403-4416), for the plates
    // other than this one (--downward-check, and the whole project of
    // the model actions for --slice 0).
    std::optional<bool> project_sequence_read;
    const auto project_sequence = [&]() {
        return read_project_print_sequence(extra, merged_print_sequence, assemble_input, config, input_file, project_sequence_read);
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
            const bool in_model = index < (int)plate_plan.plates.size() &&
                                  (plate_plan.plates[index].all || !plate_plan.plates[index].members.empty());
            if (project_plates == 1 || index == plate_id - 1 || in_model) {
                {
                    PlateScope scope(model, plate_plan.plate(index + 1));
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
    return 0;
}

static int transform_loaded_objects(CliRunContext& context, const CliOptions& o, RunPlatePlan& plate_plan,
            PlateOutcome& outcome, bool first_call, bool assemble_input, int plate_id,
            Slic3r::DynamicPrintConfig& config, SettingsMerge& settings_merge, LoadedProject& loaded_project) {
    auto& model = loaded_project.model;
    auto& plate_data = loaded_project.plate_data;

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
        for (int k = 0; k < count && k < (int)plate_plan.plates.size(); ++k) {
            const Slic3r::Vec2d origin = plate_grid_origin(config, Slic3r::Semver(), k, count);
            plate_plan.plates[k].origin = origin;
            for (Slic3r::ModelInstance* inst : plate_plan.plates[k].members)
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
    return 0;
}

static int simplify_and_check_models(const CliOptions& o, PlateOutcome& outcome, bool first_call,
            Slic3r::Model& model) {
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
    return 0;
}

static void place_geometry_inputs(const CliOptions& o, bool first_call, bool geometry_input,
            bool geometry_only_3mf, Slic3r::DynamicPrintConfig& config, Slic3r::Model& model) {
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


}

static int build_plate_arrangement_plan(CliRunContext& context, const CliOptions& o, RunPlatePlan& plate_plan,
            PlateOutcome& outcome, bool calib_self_geometry, bool first_call, bool trailing_port,
            bool assemble_input, Slic3r::DynamicPrintConfig& config, LoadedProject& loaded_project) {
    auto& model = loaded_project.model;
    auto& is_bbl_3mf = loaded_project.is_bbl_3mf;
    auto& plate_data = loaded_project.plate_data;
    auto& plan_file_config = plate_plan.plan_file_config;
    auto& plan_file_version = plate_plan.plan_file_version;
    auto& plan_file_filaments = plate_plan.plan_file_filaments;

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
        ProjectPlan& plan = plate_plan.plan;
        const Slic3r::DynamicPrintConfig& file_config = plan_file_config ? *plan_file_config : config;
        if (!plan_project_plates(context, model, config, file_config, plan_file_version, plate_data, (int)used_filaments.size(),
                                 global_arrange || o.arrange != 0, plan, outcome)) {
            std::cerr << "Error: " << outcome.error_string << "\n";
            return 1;
        }
        // The plates as the plan left them; a plate the arrange added is a
        // fresh plate with no settings of its own (PartPlateList::create_plate).
        plate_plan.plates.assign(plan.plate_count, RunPlate());
        for (int k = 0; k < plan.plate_count; ++k) {
            plate_plan.plates[k].origin  = plan.origins[k];
            plate_plan.plates[k].all     = false;
            plate_plan.plates[k].members = plan.members[k];
        }
        while ((int)plate_data.size() < plan.plate_count) {
            auto* pd = new Slic3r::PlateData();
            pd->plate_index = int(plate_data.size());
            plate_data.push_back(pd);
        }
        emit_project_plan_event(plan, trailing_port);
    }
    return 0;
}

static int arrange_plate_instances(CliRunContext& context, const CliOptions& o, RunPlatePlan& plate_plan,
            PlateOutcome& outcome, bool assemble_input, int plate_id, Slic3r::DynamicPrintConfig& config,
            bool auto_arrange_model_files, bool clearance_arrange, int arrange_plate_index,
            size_t sliced_filament_count, LoadedProject& loaded_project, RunSettings& run_settings) {
    const auto& overrides = o.overrides;
    auto& model = loaded_project.model;
    auto& plate_data = loaded_project.plate_data;
    auto& extra = run_settings.extra;
    auto& settings_merge = run_settings.settings_merge;

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
        PlateScope scope(model, plate_plan.plate(arranged_plate));
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
        PlateScope scope(model, plate_plan.plate(plate_id));
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
    } else if ((o.arrange_forced() || auto_arrange_model_files || clearance_arrange) && !plate_plan.plan.done) {
        PlateScope scope(model, plate_plan.plate(plate_id));
        if (!arrange_on_bed(context, model, config, arrange_plate_index, sliced_filament_count, outcome)) {
            std::cerr << "Error: " << outcome.error_string << "\n";
            return 1;
        }
    }
    return 0;
}

static std::optional<int> finish_run_arrangement(CliRunContext& context, const CliOptions& o,
            LoadedProject& loaded_project, RunPlatePlan& plate_plan, RunSettings& run_settings,
            RunPipeProgress& pipe_progress, RunActionOutputs& action_outputs, PlateOutcome& outcome,
            const PlateLoadInputs& inputs, bool first_call, int plate_id, Slic3r::DynamicPrintConfig& config,
            std::optional<Slic3r::DynamicPrintConfig>& project_settings, PreparedPlate& prepared) {
    auto& model = loaded_project.model;
    auto& is_bbl_3mf = loaded_project.is_bbl_3mf;
    auto& plate_data = loaded_project.plate_data;
    auto& project_facts = loaded_project.project_facts;
    auto& shrink_to_new_bed = run_settings.shrink_to_new_bed;
    auto& settings_merge = run_settings.settings_merge;

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
    if ((o.slice_mode || actions_only) && !inputs.calib_self_geometry && first_call) {
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
        if (inputs.geometry_input)
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
                                      inputs.assemble_input;
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
            clearance_arrange = !need_arrange && is_bbl_3mf && !inputs.trailing_port && shrink_to_new_bed == 0 && plate_to_slice > 0 &&
                                printer_changed && clearances_changed && is_seq_print;
            if (clearance_arrange)
                emit_event({{"event","arranged"}, {"tag","ClearanceArrange"},
                            {"message","The printer's extruder clearances differ from the project's: the by-object plate is arranged again"}});
        }
        // --repetitions: a spiral vase is copied only when printing by
        // object (BambuStudio.cpp 5259-5269; OrcaSlicer.cpp 4519-4529).
        if (!inputs.assemble_input && outcome.duplicate_count > 0) {
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
        if (arrange_plate_instances(context, o, plate_plan, outcome,
                inputs.assemble_input, plate_id, config, auto_arrange_model_files,
                clearance_arrange, arrange_plate_index, sliced_filament_count,
                loaded_project, run_settings) != 0)
            return 1;
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
            auto* project_tower = run_settings.base.option<Slic3r::ConfigOptionFloats>(key);
            if (project_tower == nullptr || project_tower->values.empty()) {
                run_settings.base.set_key_value(key, placed->clone());
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
            project_settings = run_settings.base;
        if (!prepared.outputs_ok())
            return 1;
        if (slicer_cli::has_model_actions(o) && !action_outputs.actions_done) {
            action_outputs.actions_done = true;
            if (!prepared.run_model_action_step(o.slice_mode ? slicer_cli::ActionPhase::BeforeSlice
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
        pipe_prepare_slicing(pipe_progress);
    }
    return std::nullopt;
}

static int check_slice_plate_objects(const CliOptions& o, RunPlatePlan& plate_plan,
            RunPipeProgress& pipe_progress, PlateOutcome& outcome, bool calib_self_geometry, bool first_call,
            int plate_id, Slic3r::DynamicPrintConfig& config, Slic3r::Model& model) {
    // --slice: the official per-plate gate before apply (BambuStudio.cpp
    // 6527-6567 at 5873b5f; OrcaSlicer.cpp 5645-5697 at 31f6803). An
    // object partly over the bed edge is refused, and a plate with no
    // object fully inside is refused; objects wholly outside are left
    // out of the print, as the official CLI's apply() leaves them out.
    if (o.slice_mode && !calib_self_geometry) {
        RunPlate& run_plate = plate_plan.plate(plate_id);
        pipe_prepare_slicing(pipe_progress);
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
    return 0;
}

static std::optional<int> finish_model_actions(const CliOptions& o, RunActionOutputs& action_outputs,
            PlateOutcome& outcome, PreparedPlate& prepared) {
    // Without --slice, or for the calibration prints that make their own
    // model, the actions run here; with --slice those before it ran before
    // the bed check above. The outputs are checked first (without --slice
    // this is the check before the actions and the -o G-code).
    if (!prepared.outputs_ok())
        return 1;
    if (slicer_cli::has_model_actions(o) && !action_outputs.actions_done) {
        action_outputs.actions_done = true;
        if (!prepared.run_model_action_step(o.slice_mode ? slicer_cli::ActionPhase::BeforeSlice : slicer_cli::ActionPhase::All))
            return 1;
        if (slicer_cli::model_actions_only(o)) {
            outcome.actions_only = true;
            return 0;
        }
    }
    return std::nullopt;
}

static PlateLoadInputs classify_plate_inputs(const CliOptions& o, const CliRunContext& context, bool calib_self_geometry) {
    const std::string& input_file = o.input_file;
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
    return {geometry_input, geometry_only_3mf, trailing_port, assemble_input, calib_self_geometry};
}

/// One slice of one plate from the run's one model and settings (RunState):
/// the first call of a run loads the inputs and builds the settings base
/// (`rs.project.loaded`) and runs the steps the official CLI runs once before its
/// plate loop (`rs.preparation.prepared`: the transforms, the arrange, the actions before
/// --slice); every call then lays the plate's own settings over a copy of the
/// base and slices the plate with its origin, as the official plate loop does
/// (BambuStudio.cpp 6437-7260; OrcaSlicer.cpp 5561-6200). The default call
/// (`file [--plate N] -o out.gcode`) is one call; `--slice` makes a prepare
/// call (outcome.prepare_pass, which stops before the plate checks) and then
/// one call per plate. The return value is the process exit code of the
/// default call; `outcome` carries what `--slice` mode reports on top.
int slice_one_plate(CliRunContext& context, const CliOptions& o, Slic3r::Calib_Params& calib_params, RunState& rs,
                           int plate_id, const std::string& output_file, PlateOutcome& outcome) {
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
    const bool first_call = !rs.preparation.prepared;
    carry_plate_outcome(rs.preparation, rs.warnings, outcome, first_call, plate_id);
    try {
        // Create configuration BEFORE model loading so load_bbs_3mf can populate it
        // from the embedded Metadata/project_settings.config JSON.
        PlateLoadSettings load_settings;
        auto& config = load_settings.config;
        Slic3r::Model& model = rs.project.model;


        const PlateLoadInputs inputs = classify_plate_inputs(o, context, calib_self_geometry);
        // The settings base the plate's own are laid over (below), and the
        // run-level part of the settings and of the model: once per run.
        std::optional<Slic3r::DynamicPrintConfig> project_settings;
        slicer_cli::SettingsMerge& settings_merge = rs.settings.settings_merge;
        if (!rs.project.loaded) {
            const int rc = load_slice_inputs(context, o, rs, plate_id, inputs, load_settings, outcome);
            if (rc != 0)
                return rc;
        }

        if (prepare_plate_configuration(o, rs.project, rs.settings, rs.preparation, outcome, inputs.trailing_port,
                                        plate_id, config, project_settings) != 0)
            return 1;
        if (check_downward_printers(context, o, rs.layout, outcome, inputs.trailing_port,
                inputs.assemble_input, plate_id, config, rs.project, rs.settings) != 0)
            return 1;

        // --pipe: the files are loaded (BambuStudio.cpp 4923-4926;
        // OrcaSlicer.cpp 4185), before the transforms.
        pipe_files_loaded(rs.progress);

        if (transform_loaded_objects(context, o, rs.layout, outcome, first_call,
                inputs.assemble_input, plate_id, config, settings_merge, rs.project) != 0)
            return 1;
        if (simplify_and_check_models(o, outcome, first_call, model) != 0)
            return 1;
        place_geometry_inputs(o, first_call, inputs.geometry_input, inputs.geometry_only_3mf, config, model);
        if (build_plate_arrangement_plan(context, o, rs.layout, outcome,
                calib_self_geometry, first_call, inputs.trailing_port, inputs.assemble_input,
                config, rs.project) != 0)
            return 1;
        PreparedPlate prepared{context, o, rs.project, rs.settings, rs.layout, rs.progress, rs.outputs,
                               config, project_settings, outcome};

        if (const auto result = finish_run_arrangement(context, o, rs.project,
                rs.layout, rs.settings, rs.progress, rs.outputs, outcome,
                inputs, first_call, plate_id, config, project_settings, prepared))
            return *result;
        // The run-level steps are done: the prepare call stops here, before
        // any plate is checked.
        if (first_call && o.slice_mode) {
            rs.preparation.prepared = true;
            rs.preparation.prepared_plate = plate_id;
            rs.preparation.run_config = config;
            rs.preparation.carry = outcome;
            if (outcome.prepare_pass)
                return 0;
        }

        if (check_slice_plate_objects(o, rs.layout, rs.progress, outcome,
                calib_self_geometry, first_call, plate_id, config, model) != 0)
            return 1;

        if (const auto result = finish_model_actions(o, rs.outputs, outcome,
                prepared))
            return *result;
        return prepared.print_plate(calib_params, plate_id, output_file, plate_started, calib_enabled);

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
