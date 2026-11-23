#include "cli_plate_print.hpp"
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
#include "libslic3r/GCode/GCodeStatistics.hpp"
#endif

#include "calib_args.hpp"
#include "cli_arrange_geometry.hpp"
#include "cli_engine_events.hpp"
#include "cli_failure_reason.hpp"
#include "cli_filament_config.hpp"
#include "cli_layer_ranges.hpp"
#include "cli_legacy_gcode.hpp"
#include "cli_nozzle_diameter.hpp"
#include "cli_options.hpp"
#include "cli_pipe.hpp"
#include "cli_plate_arrange.hpp"
#include "cli_plate_report.hpp"
#include "cli_plate_scope.hpp"
#include "cli_project_load.hpp"
#include "cli_result_json.hpp"
#include "cli_run_cleanup.hpp"
#include "cli_run_context.hpp"
#include "cli_run_steps.hpp"
#include "cli_setting_overrides.hpp"
#include "cli_slice_export.hpp"
#include "cli_slice_state.hpp"
#include "cli_snapmaker.hpp"
#include "diagnostic_output.hpp"
#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCode/WipeTower.hpp"
#include "libslic3r/Print.hpp"

#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <sstream>
#include <iostream>
#include <set>

namespace slicer_cli {

using json = nlohmann::json;

void pipe_files_loaded(RunPipeProgress& progress) {
    if (!slicer_cli::pipe_started() || progress.pipe_files_loaded_sent)
        return;
    progress.pipe_files_loaded_sent = true;
    slicer_cli::pipe_update(2, "Loading files finished");
}
void pipe_prepare_slicing(RunPipeProgress& progress) {
    pipe_files_loaded(progress);
    if (!slicer_cli::pipe_started() || progress.pipe_prepare_sent)
        return;
    progress.pipe_prepare_sent = true;
    slicer_cli::pipe_update(3, "Prepare slicing");
}

// The actions read the run's one model, every object at its place in
// the scene (the official's m_models, BambuStudio.cpp 6371-6401;
// OrcaSlicer.cpp 5504-5534), and --export-settings the project's
// settings (m_print_config, BambuStudio.cpp 6366-6370; OrcaSlicer.cpp
// 5499-5503).
bool PreparedPlate::run_model_action_step(slicer_cli::ActionPhase phase) {
    auto& model = loaded_project.model;
    const auto& overrides = o.overrides;
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
}

// Every output of the run, checked once the model is loaded (the
// object STL names are known) and before the first write: the
// actions, then the slice (check_run_outputs). The plates are the
// ones already known; --slice checks them all again once its plan is
// final (run_slice_mode).
bool PreparedPlate::outputs_ok(bool after_slice_actions_only) {
    auto& model = loaded_project.model;
    if (action_outputs.outputs_checked)
        return true;
    action_outputs.outputs_checked = true;
    // Every plate the run could write: plate N for --slice N, else
    // every plate of the plan, or of the input when there is no plan
    // yet (a superset is fine: the check after the plan stays).
    std::vector<int> known_plates;
    if (o.slice_mode) {
        if (o.slice_plate > 0)
            known_plates.push_back(o.slice_plate);
        const int count = plate_plan.plan.done ? std::max(plate_plan.plan.plate_count, plate_plan.file_plate_count)
                                       : plate_plan.file_plate_count;
        if (o.slice_plate <= 0)
            for (int p = 1; p <= std::max(1, count); ++p)
                known_plates.push_back(p);
    }
    std::string detail;
    if (const int refused = check_run_outputs(context, o, known_plates, &model, &detail, after_slice_actions_only,
                                              after_slice_actions_only ? &action_outputs.reserved_outputs : nullptr,
                                              after_slice_actions_only ? nullptr : &action_outputs.reserved_outputs)) {
        set_outcome_failure(outcome, refused, detail);
        outcome.run_step_failed = true;
        return false;
    }
    return true;
}

static void set_plate_printer_vendor(LoadedProject& loaded_project, const SettingsMerge& settings_merge, int plate_id,
            Slic3r::DynamicPrintConfig& config, Slic3r::Print& print) {
    auto& model = loaded_project.model;


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
        else if (!settings_merge.new_printer_name.empty())
            is_bbl_vendor_preset = settings_merge.new_printer_name.compare(0, 9, "Bambu Lab") == 0;
        else if (!loaded_project.project_facts.current_printer_system_name.empty())
            is_bbl_vendor_preset = loaded_project.project_facts.current_printer_system_name.compare(0, 9, "Bambu Lab") == 0;
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
}

static int normalize_plate_before_apply(CliRunContext& context, const CliOptions& o,
            LoadedProject& loaded_project, RunPlatePlan& plate_plan, PlateOutcome& outcome, int plate_id,
            Slic3r::DynamicPrintConfig& config, Slic3r::DynamicPrintConfig& extra) {
    const auto& input_file = o.input_file;
    const auto& verbose = o.verbose;
    auto& model = loaded_project.model;

    // Automatic filament grouping on a machine with several extruders sizes
    // each nozzle's group from extruder_ams_count (FilamentGroupUtils.cpp
    // calc_max_group_size, both engines). The desktop app takes that count
    // from the printer it is connected to, else from the project it opened,
    // and builds no AMS slots without a printer (BambuStudio PresetBundle.cpp
    // 3826-3831 and 3547, Plater.cpp 10810-10826 at 02.08.02.61; OrcaSlicer
    // PresetBundle.cpp 6272-6276 and 5842 at 2.5.2). Headless there is no
    // printer, so the input's own count stands, and input without one groups
    // as the desktop does with an empty count. The official CLIs instead
    // give every nozzle an estimated 4-slot AMS (BambuStudio.cpp 6911-6950,
    // OrcaSlicer.cpp 6954-6991), which regroups a project the maker sliced
    // with another count.

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
#ifndef ENGINE_SNAPMAKER
    // OrcaSlicer from 2.5: the mixed-filament lists hold one entry per loaded
    // filament (OrcaSlicer.cpp 4260-4261 at f92a7c0). A mixed slot past the
    // loaded filaments is refused with the plate's other filament checks.
    if (const auto* presets = config.option<Slic3r::ConfigOptionStrings>("filament_settings_id");
        presets != nullptr && !presets->values.empty())
        if (const auto* is_mixed = config.option<Slic3r::ConfigOptionBools>("filament_is_mixed");
            is_mixed == nullptr || is_mixed->values.size() <= presets->values.size())
            Slic3r::resize_mixed_filament_metadata(config, presets->values.size(), presets->values.size());
#endif
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
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
    // OrcaSlicer from 2.5: the stored (or default) tower position may not fit
    // the tower this plate needs, so the official command line fits every
    // plate's tower into the plate before it slices (OrcaSlicer.cpp 6495-6526
    // at f92a7c0), also a position a flag gave. The engine then refuses a
    // tower outside the area every extruder reaches (Print.cpp 1734-1750).
    {
        // The plate's own print sequence, else the project's (get_print_sequence).
        bool by_object = false;
        if (const auto* seq = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence"))
            by_object = seq->value == Slic3r::PrintSequence::ByObject;
        const int index = std::max(0, plate_id - 1);
        if (index < (int)loaded_project.plate_data.size() && loaded_project.plate_data[index] != nullptr)
            if (const auto* own = loaded_project.plate_data[index]->config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
                own && own->value != Slic3r::PrintSequence::ByDefault)
                by_object = own->value == Slic3r::PrintSequence::ByObject;
        PlateTowerFit fit;
        {
            PlateScope scope(model, plate_plan.plate(plate_id));
            fit = fit_plate_tower_before_slice(model, config, index, by_object);
        }
        if (fit.moved) {
            std::ostringstream msg;
            msg << "Plate " << (index + 1) << ": the prime tower no longer fits at its position (" << fit.saved_x << ", "
                << fit.saved_y << ") and was moved to (" << fit.x << ", " << fit.y << "), "
                << (fit.overlaps_objects ? "onto an object, as no free space is left" : "to free space")
                << ". Give --wipe-tower-x and --wipe-tower-y for another place.";
            emit_event({{"event", "config_normalized"}, {"tag", "PrimeTowerMoved"}, {"plate_id", index + 1},
                        {"from", {fit.saved_x, fit.saved_y}}, {"to", {fit.x, fit.y}},
                        {"overlaps_objects", fit.overlaps_objects}, {"message", msg.str()}});
        }
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
        slicer_cli::trim_filaments_for_switch(config, model, o.printer_preset, plate_id, loaded_project.switch_paint_remapped);
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
            PlateScope scope(model, plate_plan.plate(plate_id));
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
    return 0;
}

static void apply_plate_to_print(const CliOptions& o, RunPlatePlan& plate_plan, const Slic3r::DynamicPrintConfig& extra,
            PlateOutcome& outcome, int plate_id, Slic3r::DynamicPrintConfig& config, Slic3r::Model& model,
            Slic3r::Print& print) {
    {
        // The plate's instances only, on the plate (see the Print's
        // origin above). Print::apply copies the model it is given
        // (PrintApply.cpp 1553 / 1298), so the run's model stays in
        // the scene.
        PlateScope scope(model, plate_plan.plate(plate_id));
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
            for (const std::string& key : extra.keys())
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
}

#if defined(ENGINE_ORCA)
static void initialize_print_wipe_tower(Slic3r::Print& print) {
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
}
#endif

static void apply_print_calibration(bool calib_enabled, Slic3r::Calib_Params& calib_params,
            Slic3r::Print& print) {
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
static void install_plate_progress(const CliOptions& o, RunPipeProgress& pipe_progress, PlateOutcome& outcome,
            Slic3r::Print& print, bool piped) {
    // The check pass sends no plate step: the official reports a plate
    // only once it slices it (BambuStudio.cpp 7034-7046, after the
    // check pass's continue at 7027).
    if (piped && !outcome.pre_check) {
        pipe_prepare_slicing(pipe_progress);   // once per run: here only for a run that skipped the plate checks
        slicer_cli::pipe_set_plate_info(std::max(1, outcome.plate_id), std::max(1, outcome.plate_count));
        slicer_cli::pipe_update(4, "Slicing begins");
    }
    // --strict acts on the warnings this callback keeps, so it is installed
    // for it also when no progress was asked for.
    const bool strict = slicer_cli::plate_loop_switches(o).strict;
    if (o.progress || piped || strict) {
        print.set_status_callback([&outcome, &o, piped, strict](const Slic3r::PrintBase::SlicingStatus& status) {
            if (piped)
                slicer_cli::pipe_update(status.percent, status.text, status.warning_step);
            emit_status_warning(status);
            if (!o.progress && !strict)
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
            if (o.progress && status.percent >= 0)
                emit_progress(outcome, status.percent, status.text);
        });
    } else {
        print.set_status_callback(emit_status_warning);
    }
}

static int configure_plate_checks(const CliOptions& o, RunPlatePlan& plate_plan, PlateOutcome& outcome,
            int plate_id, Slic3r::DynamicPrintConfig& config, Slic3r::Model& model, Slic3r::Print& print,
            const PlateLoopSwitches& switches) {
    outcome.no_check = switches.no_check;
    outcome.strict   = switches.strict;
    // --no-check skips the check behind the only non-critical warning --strict
    // acts on, so OrcaSlicer refuses the pair (OrcaSlicer.cpp 1422-1429 at f92a7c0).
    if (switches.strict && switches.no_check) {
        const std::string why = "--strict cannot be combined with --no-check";
        std::cerr << "Error: " << why << "\n";
        set_outcome_failure(outcome, CLI_INVALID_PARAMS, why);
        return 1;
    }
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
            PlateScope scope(model, plate_plan.plate(plate_id));
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
            PlateScope scope(model, plate_plan.plate(plate_id));
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
    return 0;
}

static int report_plate_validation_error(PlateOutcome& outcome, Slic3r::DynamicPrintConfig& config,
            Slic3r::Print& print, Slic3r::StringObjectException& validation_result) {
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
        // BambuStudio.cpp 685-745 at 5873b5f / OrcaSlicer.cpp 686-742 at
        // f92a7c0 makes every G-code key the file leaves out an empty
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
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
        // OrcaSlicer from 2.5 checks the prime tower before it slices: partly
        // outside the printable area, on an exclusion area or on the clumping
        // detection area (Print.cpp 1723-1745 at f92a7c0). The sentence says
        // what is wrong; add the flags that move the tower or turn it off,
        // as the refusal after the slice does (cli_plate_report.cpp).
        if (message.rfind("Prime tower is", 0) == 0) {
            while (!message.empty() && (message.back() == '\n' || message.back() == ' '))
                message.pop_back();
            message += " Move the prime tower with --wipe-tower-x and --wipe-tower-y, or turn it off with"
                       " --enable-prime-tower=0.";
        }
#endif
        // The engine's own sentence names the plate and the filament
        // and stops there ("Plate 1: Cool Plate does not support
        // filament 2"). Its check reads the plate's own bed temperature
        // key with get_at and reads 0 as "not this filament's plate", for every
        // filament the run prints with (Print.cpp 2814-2840 at f92a7c0;
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
                // order (PrintConfig.cpp 1455-1459 at f92a7c0,
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
    return 0;
}

static int validate_plate_print(PlateOutcome& outcome, Slic3r::DynamicPrintConfig& config, Slic3r::Print& print,
            const Slic3r::Vec2d& plate_origin, const PlateLoopSwitches& switches) {
    std::cout << "Validating...\n";
    // Option C — pass a warning out-pointer so BBS routes is_warning-
    // flagged exceptions there instead of into the return value
    // (mirrors `Plater.cpp:9888`'s call signature). Hard validation
    // errors still come back as the return value and remain fatal.
    // Soft warnings get emitted as JSON events and slicing proceeds —
    // matching what BBS GUI does in this case.
    // BambuStudio PrintBase.hpp:35 defaults the warning type to zero. OrcaSlicer PrintBase.hpp:35 at 8500fcd leaves it unset.
    Slic3r::StringObjectException validation_warning{};
    // The GUI calls validate() with all three out-params
    // (Plater.cpp:10835) and paints the returned hulls on the plate;
    // a human sees the collision as geometry, with no text anywhere.
    // Passing them here is the only way an agent can learn WHERE a
    // sequential-print clearance violation is, rather than just that
    // one happened.
    Slic3r::Polygons collision_polygons;
    std::vector<std::pair<Slic3r::Polygon, float>> height_polygons;
#ifndef ENGINE_SNAPMAKER
    // Bambu 2.9.1 and OrcaSlicer 2.5.2 validate() return every warning (Print.hpp 948
    // at v02.09.01.70, Print.hpp 1116 at v2.5.2); the official CLIs join them with
    // newlines (BambuStudio.cpp 7531-7536, OrcaSlicer.cpp 7086-7092). Snapmaker Orca
    // 2.4.1 returns one.
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
    if (report_plate_validation_error(outcome, config, print, validation_result) != 0)
        return 1;
    return 0;
}

static int process_plate_toolpath(CliRunContext& context, const CliOptions& o, PlateOutcome& outcome,
            Slic3r::Print& print, int cache_plate) {
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
    return 0;
}

static int export_plate_gcode(CliRunContext& context, const CliOptions& o, const Slic3r::DynamicPrintConfig& base,
            PlateOutcome& outcome, const std::string& output_file, Slic3r::DynamicPrintConfig& config,
            Slic3r::Model& model, std::chrono::steady_clock::time_point plate_started, Slic3r::Print& print,
            const Slic3r::Vec2d& plate_origin, const PlateLoopSwitches& switches, int cache_plate,
            PreparedPlate& actions) {
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
            record_plate_statistics(print, model, config, base, gcode_result, outcome);
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
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
            // --export-statistics (OrcaSlicer from 2.5; OrcaSlicer.cpp 7228-7237 at
            // f92a7c0): the plate's statistics as JSON next to its G-code. The
            // official skips it without an output directory, where its G-code is
            // a temporary file. Here a --slice run always keeps its G-code (in
            // --outputdir, or in the current folder without it), so the
            // statistics are written next to it in both cases; the name is
            // checked with the run's other outputs (collect_run_outputs).
            if (o.given_flag("export_statistics") && o.cli.opt_bool("export_statistics")) {
                try {
                    Slic3r::export_gcode_statistics(gcode_result, Slic3r::gcode_statistics_path(output_file));
                } catch (const std::exception& ex) {
                    set_outcome_failure(outcome, CLI_EXPORT_STATISTICS_ERROR, ex.what());
                    return 1;
                }
            }
#endif
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
            !actions.run_model_action_step(slicer_cli::ActionPhase::AfterSlice))
            return 1;

    } catch (const std::bad_alloc&) {
        // Out of memory while slicing or exporting. The official CLI has
        // this outcome of its own -- CLI_OUT_OF_MEMORY (-14, Utils.hpp:39
        // at 31f6803) with the sentence that says what to do
        // (OrcaSlicer.cpp 208, BambuStudio.cpp 132) -- and it is not a
        // slicing failure: a run that ran out of memory must never
        // report CLI_SLICING_ERROR (-100).
        emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind","bad_alloc"},
                    {"message","std::bad_alloc"}});
        std::cerr << "\nOut of memory during slicing.\n";
        set_outcome_failure(outcome, CLI_OUT_OF_MEMORY);
        return 1;
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
    } catch (const Slic3r::SlicingErrors& exs) {
        // OrcaSlicer from 2.5 collects the errors of several objects; the
        // engine words them, one line per object (OrcaSlicer.cpp 7295-7300,
        // Print.cpp 1964-1978 at f92a7c0).
        const std::string message = print.slicing_errors_message(exs);
        emit_event({{"event","slicing_error"},{"phase","export_gcode"},{"kind","SlicingErrors"},{"message",message}});
        slicer_cli::add_engine_reason(outcome.error_reason, message.c_str());
        std::cerr << "\n" << message << "\n";
        return 1;
#endif
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
    return 0;
}

static Slic3r::Vec2d set_print_plate_frame(Slic3r::Print& print, int plate_id) {
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
    return plate_origin;
}

bool PreparedPlate::prepare_calibration_geometry(Slic3r::Calib_Params& calib_params) {
    // Alias the unbound `initial_no_support_filament_id` placeholder to the
    // engine-bound `initial_no_support_extruder` across custom-gcode keys, BEFORE
    // print.apply snapshots the config. Without this the PlaceholderParser throws at
    // export when a 3MF carries the legacy token in its custom gcode. Always on;
    // suppressed with --no-normalize-legacy-gcode.
    if (o.normalize_legacy_gcode) {
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
        slicer_cli::apply_pa_pattern(calib_params, config, loaded_project.model, calib_is_bbl_machine);
        // The pattern replaced the model, so the object STL files an
        // --export-stl after --slice writes carry its object's name
        // (obj_1_pa_pattern_handle.stl): the outputs still to be written
        // are checked again. One given before --slice has written the
        // loaded model's files already.
        action_outputs.outputs_checked = false;
        if (!outputs_ok(/*after_slice_actions_only=*/true))
            return false;
    }
    return true;
}

static void set_brim_tables(Slic3r::DynamicPrintConfig& config, Slic3r::Print& print) {
    // The per-filament and speed tables the brim width reads
    // (Brim.cpp, through Model::extruderParamsMap and
    // printSpeedMap): both official CLIs fill them between apply()
    // and process() (BambuStudio.cpp 7074-7075 at 5873b5f;
    // OrcaSlicer.cpp 7112-7113 at f92a7c0). Left empty, the brim
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
}

int PreparedPlate::print_plate(Slic3r::Calib_Params& calib_params, int plate_id, const std::string& output_file,
                               std::chrono::steady_clock::time_point plate_started, bool calib_enabled) {
    auto& model = loaded_project.model;
    auto& extra = run_settings.extra;

    // Initialize print
    std::cout << "\nInitializing print...\n";
    Slic3r::Print print;
    set_plate_printer_vendor(loaded_project, run_settings.settings_merge, plate_id, config, print);

    const Slic3r::Vec2d plate_origin = set_print_plate_frame(print, plate_id);

    if (!prepare_calibration_geometry(calib_params))
        return 1;

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
    align_per_filament_config_vectors(config, run_settings.project_filaments, o.verbose);
#endif // ENGINE_BAMBU

    try {
        if (normalize_plate_before_apply(context, o, loaded_project, plate_plan, outcome,
                plate_id, config, extra) != 0)
            return 1;
        apply_plate_to_print(o, plate_plan, extra, outcome, plate_id, config, model, print);
#ifdef ENGINE_ORCA
        initialize_print_wipe_tower(print);
#endif

        apply_print_calibration(calib_enabled, calib_params, print);

        const bool piped = slicer_cli::pipe_started();
        install_plate_progress(o, pipe_progress, outcome, print, piped);

        set_brim_tables(config, print);

        // --no-check and --allow-mix-temp (BambuStudio.cpp 6979-6982;
        // OrcaSlicer.cpp 5968-5971); only when given, so the default
        // stays the engine's own.
        const slicer_cli::PlateLoopSwitches switches = slicer_cli::plate_loop_switches(o);
        if (configure_plate_checks(o, plate_plan, outcome, plate_id, config, model,
                print, switches) != 0)
            return 1;
        if (validate_plate_print(outcome, config, print, plate_origin, switches) != 0)
            return 1;
        // --slice 0 on several plates: the check pass ends here for this
        // plate, once its bed check, apply and validate passed
        // (BambuStudio.cpp 7027-7028; OrcaSlicer.cpp 6034-6035).
        if (outcome.pre_check)
            return 0;

        // --load-slicedata: the plate's saved slicing, else a normal
        // slice (BambuStudio.cpp 7076-7099; OrcaSlicer.cpp 6067-6090).
        const int cache_plate = plate_id > 0 ? plate_id : 1;
        if (process_plate_toolpath(context, o, outcome, print, cache_plate) != 0)
            return 1;

        if (export_plate_gcode(context, o, run_settings.base, outcome, output_file, config,
                model, plate_started, print, plate_origin, switches, cache_plate,
                *this) != 0)
            return 1;

    } catch (const std::bad_alloc&) {
        // The slicing step ran out of memory (see the export step above):
        // this engine's own CLI has CLI_OUT_OF_MEMORY for it, never -100.
        emit_event({{"event","slicing_error"},{"phase","process"},{"kind","bad_alloc"},
                    {"message","std::bad_alloc"}});
        std::cerr << "\nOut of memory during slicing.\n";
        set_outcome_failure(outcome, CLI_OUT_OF_MEMORY);
        return 1;
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
    } catch (const Slic3r::SlicingErrors& exs) {
        // See export_plate_gcode.
        const std::string message = print.slicing_errors_message(exs);
        emit_event({{"event","slicing_error"},{"phase","process"},{"kind","SlicingErrors"},{"message",message}});
        slicer_cli::add_engine_reason(outcome.error_reason, message.c_str());
        std::cerr << message << "\n";
        return 1;
#endif
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
}

} // namespace slicer_cli
