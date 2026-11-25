#include "cli_load_slice_inputs.hpp"
#include "cli_events.hpp"
#include "cli_load_project_archive.hpp"
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

static int load_input_geometry(CliRunContext& context, const CliOptions& o, LoadedProject& loaded_project,
            RunPlatePlan& plate_plan, PlateOutcome& outcome, int plate_id, PlateLoadSettings& settings,
            const PlateLoadInputs& inputs) {
    const auto& input_file = o.input_file;
    auto& model = loaded_project.model;
    auto& plate_data = loaded_project.plate_data;

    if (inputs.assemble_input) {
        // The list's plates the run slices: plate N, or every plate. Each
        // plate's objects go to its place in the plate grid once the bed
        // is known (below).
        if (plate_id > (int)context.g_assemble->plates.size() || plate_id < 0) {
            set_outcome_failure(outcome, CLI_INVALID_PARAMS, "The assemble list has no plate " + std::to_string(plate_id) + ".");
            return 1;
        }
        plate_plan.plates.assign(context.g_assemble->plates.size(), RunPlate());
        for (size_t k = 0; k < context.g_assemble->plates.size(); ++k) {
            plate_plan.plates[k].all = false;
            if (plate_id > 0 && int(k) != plate_id - 1)
                continue;
            const size_t before = model.objects.size();
            for (const Slic3r::ModelObject* object : context.g_assemble->plates[k].loaded_obj_list)
                model.add_object(*object);
            for (size_t oi = before; oi < model.objects.size(); ++oi)
                for (Slic3r::ModelInstance* inst : model.objects[oi]->instances)
                    plate_plan.plates[k].members.push_back(inst);
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
        if (load_project_archive(context, o, loaded_project, plate_plan, outcome, plate_id,
                settings, inputs) != 0)
            return 1;
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
    return 0;
}

static int finish_loaded_geometry(const CliOptions& o, LoadedProject& loaded_project, RunPlatePlan& plate_plan,
            PlateOutcome& outcome, int plate_id, Slic3r::DynamicPrintConfig& config,
            const PlateLoadInputs& inputs) {
    auto& model = loaded_project.model;

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
    if (plate_plan.plates.empty())
        plate_plan.plates.assign(1, RunPlate());
#ifdef ENGINE_BAMBU
    // Bambu Studio puts each plate's saved tower back inside the plate
    // when it opens a project (cli_project_load.hpp). Here, before the
    // flags are laid on the settings: a position a flag gives stays as
    // given, and the project's settings, which the exports write, hold
    // the corrected one. Every plate whose objects are loaded (one plate
    // with --slice N or --plate N). --arrange places the tower itself.
    if (loaded_project.project_loaded_like_desktop && !o.arrange_forced())
        if (const auto* area = config.option<Slic3r::ConfigOptionPoints>("printable_area");
            area != nullptr && area->values.size() >= 3) {
            const Slic3r::BoundingBoxf reach = tower_reach_box(config, *area);
            const double height = config.has("printable_height") ? config.opt_float("printable_height") : 0.;
            const Slic3r::BoundingBoxf3 plate_box(Slic3r::Vec3d(reach.min.x(), reach.min.y(), 0.),
                                                  Slic3r::Vec3d(reach.max.x(), reach.max.y(), height));
            const TowerWipeParams wipe = tower_wipe_params(config);
            const auto exclude = arrange_exclude_boxes(config);
            for (int index = 0; index < (int)plate_plan.plates.size(); ++index) {
                if (plate_id > 0 && index != plate_id - 1)
                    continue;
                PlateScope scope(model, plate_plan.plates[index]);
                slicer_cli::project_load_tower_inside_plate(
                    config, index, (int)plate_plan.plates.size(), reach,
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
    // (get_at, Config.hpp 679 at f92a7c0; 438 at bac6fd5).
    if (loaded_project.project_loaded_like_desktop && !o.arrange_forced())
        slicer_cli::project_load_tower_positions_per_plate(config, (int)plate_plan.plates.size());
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
    return 0;
}

static void load_named_and_file_profiles(CliRunContext& context, const CliOptions& o, PlateOutcome& outcome,
            Slic3r::DynamicPrintConfig& config, bool& explicit_config_supplied_nozzle_map) {
    const auto& machine_config = o.machine_config;
    const auto& filament_config = o.filament_config;
    const auto& process_config = o.process_config;
    const auto& bundle_config = o.bundle_config;
    const auto& verbose = o.verbose;

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
}

static int merge_slice_settings(CliRunContext& context, const CliOptions& o, PlateOutcome& outcome,
            int plate_id, Slic3r::DynamicPrintConfig& config, const PlateLoadInputs& inputs,
            LoadedProject& loaded_project, RunSettings& run_settings) {
    const auto& input_file = o.input_file;
    const auto& machine_config = o.machine_config;
    const auto& filament_config = o.filament_config;
    const auto& process_config = o.process_config;
    const auto& bundle_config = o.bundle_config;
    auto& model = loaded_project.model;
    const auto& project_facts = loaded_project.project_facts;
    auto& project_presets_kept = loaded_project.project_presets_kept;
    auto& extra = run_settings.extra;
    auto& settings_merge = run_settings.settings_merge;
    auto& merged_print_sequence = run_settings.merged_print_sequence;

    // --load-settings, --load-filaments, --uptodate*: the official
    // settings merge (cli_load_settings.cpp), on the settings loaded so far.
    // (run_settings.settings_merge; run_settings.merged_print_sequence: the project's
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
    // An assemble list's filament number past the filaments the run has now,
    // with every preset and settings file in (cli_assemble.hpp).
    if (inputs.assemble_input) {
        const std::string why = slicer_cli::assemble_filament_refusal(context.g_assemble->plates, project_filament_count(config));
        if (!why.empty()) {
            std::cerr << "Error: " << why << "\n";
            set_outcome_failure(outcome, CLI_INVALID_PARAMS, why);
            return 1;
        }
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
    return 0;
}

static int apply_slice_setting_flags(const CliOptions& o, PlateOutcome& outcome,
            Slic3r::DynamicPrintConfig& config, Slic3r::DynamicPrintConfig& extra) {
#ifndef ENGINE_SNAPMAKER
    // The filament switcher and the smart filament assignment that needs
    // it are off unless a flag states them, whatever the project or its
    // plate saved: the official CLI's rule (BambuStudio.cpp 4436-4439 at
    // v02.09.01.70; OrcaSlicer.cpp 1374-1380 at f92a7c0). They describe
    // hardware fitted to the maker's printer, which a headless run cannot see.
    for (const char* key : {"enable_filament_dynamic_map", "has_filament_switcher"})
        if (!extra.has(key))
            extra.set_key_value(key, new Slic3r::ConfigOptionBool(false));
#endif
#ifdef ENGINE_BAMBU
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
    return 0;
}

static int select_nozzle_and_plate(CliRunContext& context, const CliOptions& o, PlateOutcome& outcome,
            Slic3r::DynamicPrintConfig& config, const PlateLoadInputs& inputs) {
    const auto& input_file = o.input_file;
    const auto& machine_config = o.machine_config;
    const auto& filament_config = o.filament_config;
    const auto& process_config = o.process_config;
    const auto& bundle_config = o.bundle_config;
    const auto& overrides = o.overrides;


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
        // OrcaSlicer Plater.cpp 3880-3909 and 4233-4241 at f92a7c0), for a
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
            // 3340-3414 at 5873b5f; OrcaSlicer Plater.cpp 3880-3909 at
            // f92a7c0). Editing nozzle_diameter alone leaves the 0.4 line
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
            // (OrcaSlicer Plater.cpp 3880-3910 at f92a7c0, both sides of
            // its is_bbl_vendor || support_multi_bed_types test, through
            // Preset::get_default_bed_type, which reads a stated plate NAME
            // from OrcaSlicer 2.5 on and a number only in Snapmaker Orca;
            // desktop_presets.cpp desktop_bed_type has both rules — and
            // BambuStudio Plater.cpp 3340-3414, 3703-3715 at
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
    return 0;
}

static int validate_loaded_plate_settings(PlateWarnings& plate_warnings, PlateOutcome& outcome, int plate_id,
            const std::map<std::string, std::string>& overrides, PlateLoadSettings& settings,
            const Slic3r::DynamicPrintConfig& extra, const Slic3r::PlateDataPtrs& plate_data, const PlateLoadInputs& inputs,
            const Slic3r::DynamicPrintConfig& settings_before_flags) {
    auto& config = settings.config;
    const auto& unknown_values = settings.unknown_values;
    const auto& unknown_items = settings.unknown_items;

    // The official value check (OrcaSlicer.cpp 4298-4305 at f92a7c0,
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
        const bool noted_first_time = plate_warnings.noted.count(noted_plate) == 0;
        json& noted_warnings = plate_warnings.noted[noted_plate];
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
                if (plate_warnings.hold_noted)
                    plate_warnings.noted_held.push_back(announcement);
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
        take_noted_warnings(outcome, plate_warnings.noted, plate_id);
    }
    return 0;
}

static void move_loaded_project_to_bed(CliRunContext& context, RunPlatePlan& plate_plan, int plate_id,
            Slic3r::DynamicPrintConfig& config, const PlateLoadInputs& inputs, LoadedProject& loaded_project,
            RunSettings& run_settings) {
    auto& model = loaded_project.model;
    auto& plate_data = loaded_project.plate_data;
    auto& project_facts = loaded_project.project_facts;
    auto& shrink_to_new_bed = run_settings.shrink_to_new_bed;
    auto& settings_merge = run_settings.settings_merge;

    // A project on a printer with another bed: each plate's objects and
    // tower move onto it (move_to_new_bed; translate_models runs on every
    // plate, BambuStudio.cpp 4497-4659; OrcaSlicer.cpp 3885-3980), after
    // the settings given as flags apply (BambuStudio.cpp 4091; OrcaSlicer.cpp
    // 3542), so a flag's tower position or prime tower moves too. Each
    // plate moves in its own frame (PlateScope).
    if (project_facts.is_bbl_3mf && !inputs.assemble_input && !model.objects.empty()) {
        const int first = plate_id > 0 ? plate_id : 1;
        const int last  = plate_id > 0 ? plate_id : (int)plate_plan.plates.size();
        for (int moved_plate = first; moved_plate <= last; ++moved_plate) {
            const int index = moved_plate - 1;
            RunPlate& run_plate = plate_plan.plate(moved_plate);
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
            for (int index = 0; index < (int)plate_plan.plates.size() && index < (int)plate_data.size(); ++index) {
                RunPlate& run_plate = plate_plan.plates[index];
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
}

#if defined(ENGINE_BAMBU)
static void initialize_extruder_nozzle_stats(RunSettings& run_settings, Slic3r::DynamicPrintConfig& config,
            ProjectFacts& project_facts) {
    auto& extra = run_settings.extra;
    auto& project_filaments = run_settings.project_filaments;

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
            run_settings.settings_merge.machine_switch ? nullptr : has_values(stats_new) ? stats_new : has_values(stats) ? stats : nullptr;
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
                if (!run_settings.settings_merge.machine_switch)
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
                const bool printer_changed = !run_settings.settings_merge.new_printer_name.empty() &&
                                             run_settings.settings_merge.new_printer_name != project_facts.current_printer_name;
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
}
#endif

#if defined(ENGINE_BAMBU)
static void normalize_toolchanger_vectors(bool verbose, Slic3r::DynamicPrintConfig& config,
            bool& explicit_config_supplied_nozzle_map) {
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
}
#endif

int load_slice_inputs(CliRunContext& context, const CliOptions& o, LoadedProject& loaded_project, RunSettings& run_settings,
                      RunPlatePlan& plate_plan, PlateWarnings& plate_warnings, int plate_id,
                      const PlateLoadInputs& inputs, PlateLoadSettings& settings, PlateOutcome& outcome) {
    const std::string& input_file      = o.input_file;
    const bool verbose                 = o.verbose;
    const auto& overrides = o.overrides;
    Slic3r::Model& model = loaded_project.model;
    Slic3r::PlateDataPtrs& plate_data = loaded_project.plate_data;
    slicer_cli::ProjectFacts& project_facts = loaded_project.project_facts;
    Slic3r::DynamicPrintConfig& extra = run_settings.extra;
    bool& explicit_config_supplied_nozzle_map = run_settings.explicit_config_supplied_nozzle_map;
    auto& config = settings.config;

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
    if (load_input_geometry(context, o, loaded_project, plate_plan, outcome, plate_id, settings, inputs) != 0)
        return 1;

    if (finish_loaded_geometry(o, loaded_project, plate_plan, outcome, plate_id, config,
            inputs) != 0)
        return 1;

    load_named_and_file_profiles(context, o, outcome, config, explicit_config_supplied_nozzle_map);
    if (merge_slice_settings(context, o, outcome, plate_id, config, inputs, loaded_project,
            run_settings) != 0)
        return 1;
    // Every setting given as a flag, over the loaded settings: the
    // official m_print_config.apply(m_extra_config, true)
    // (BambuStudio.cpp 4091 at 5873b5f; OrcaSlicer.cpp 4189 at f92a7c0).
    // The settings before any flag: the settings check's desktop baseline.
    const Slic3r::DynamicPrintConfig settings_before_flags = config;
    if (apply_slice_setting_flags(o, outcome, config, extra) != 0)
        return 1;
    if (select_nozzle_and_plate(context, o, outcome, config, inputs) != 0)
        return 1;

    if (validate_loaded_plate_settings(plate_warnings, outcome, plate_id, overrides, settings,
            extra, plate_data, inputs,
            settings_before_flags) != 0)
        return 1;

    move_loaded_project_to_bed(context, plate_plan, plate_id, config, inputs, loaded_project, run_settings);

#ifdef ENGINE_BAMBU
    initialize_extruder_nozzle_stats(run_settings, config, project_facts);
#endif

#ifdef ENGINE_BAMBU
    normalize_toolchanger_vectors(verbose, config, explicit_config_supplied_nozzle_map);
#endif // ENGINE_BAMBU — the run's settings base

    // The run's settings base, m_print_config (see RunState): everything
    // above runs once per run.
    run_settings.base = config;
    loaded_project.loaded = true;
    return 0;
}

} // namespace slicer_cli
