#include "cli_check_downward.hpp"
#include "cli_assemble.hpp"
#include "cli_events.hpp"
#include "cli_load_settings.hpp"
#include "cli_nozzle_diameter.hpp"
#include "cli_options.hpp"
#include "cli_plate_arrange.hpp"
#include "cli_plate_plan.hpp"
#include "cli_project_archive.hpp"
#include "cli_run_cleanup.hpp"
#include "cli_run_context.hpp"
#include "cli_slice_state.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <set>

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

int check_downward_printers(CliRunContext& context, const CliOptions& o, RunPlatePlan& plate_plan,
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

} // namespace slicer_cli
