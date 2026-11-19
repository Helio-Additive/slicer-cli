#include "cli_plate_settings.hpp"
#include "cli_arrange_geometry.hpp"
#include "cli_engine_events.hpp"
#include "cli_filament_config.hpp"
#include "cli_filament_map.hpp"
#include "cli_legacy_gcode.hpp"
#include "cli_nozzle_options.hpp"
#include "cli_nozzle_diameter.hpp"
#include "cli_load_settings.hpp"
#include "cli_run_steps.hpp"
#include "cli_options.hpp"
#include "cli_result_json.hpp"
#include "cli_setting_overrides.hpp"
#include "cli_slice_state.hpp"
#include "cli_snapmaker.hpp"
#include "diagnostic_output.hpp"
#include "libslic3r/PresetBundle.hpp"
#include <boost/algorithm/string.hpp>
#include <iostream>
#ifdef ENGINE_BAMBU
#include "libslic3r/FilamentMixer.hpp"
#endif

namespace slicer_cli {

using json = nlohmann::json;

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
                                               Slic3r::DynamicPrintConfig& config, Slic3r::DynamicPrintConfig& base,
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
    state_maps(base);
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
bool manual_map_extruders_fit(const Slic3r::DynamicPrintConfig& config, const Slic3r::Model& model, int plate_id,
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

#if defined(ENGINE_BAMBU)
static int apply_bambu_plate_filament_map(const CliOptions& o, RunSettings& run_settings, PlateOutcome& outcome,
            int plate_id, Slic3r::DynamicPrintConfig& config, Slic3r::PlateDataPtrs& plate_data,
            std::optional<Slic3r::DynamicPrintConfig>& project_settings) {
    const auto& verbose = o.verbose;
    auto& extra = run_settings.extra;
    auto& explicit_config_supplied_nozzle_map = run_settings.explicit_config_supplied_nozzle_map;
    auto& project_filaments = run_settings.project_filaments;

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
    switch (apply_filament_map_flag(o, extra, config, run_settings.base, project_settings, project_filaments, plate_id, outcome)) {
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
    return 0;
}
#endif

static int apply_plate_overrides(const CliOptions& o, RunSettings& run_settings, PlateOutcome& outcome,
            int plate_id, Slic3r::DynamicPrintConfig& config,
            std::optional<Slic3r::DynamicPrintConfig>& project_settings, LoadedProject& loaded_project) {
    const auto& overrides = o.overrides;
    auto& model = loaded_project.model;
    auto& plate_data = loaded_project.plate_data;
    auto& extra = run_settings.extra;


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
    switch (apply_filament_map_flag(o, extra, config, run_settings.base, project_settings, project_filament_count(config), plate_id,
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
    return 0;
}

static void report_active_plate_settings(const Slic3r::DynamicPrintConfig& config) {

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
}

static int overlay_fresh_plate_configuration(const CliOptions& o, LoadedProject& loaded_project,
            RunSettings& run_settings, PlateOutcome& outcome, bool trailing_port, bool fresh_plate_config,
            int plate_id, Slic3r::DynamicPrintConfig& config,
            std::optional<Slic3r::DynamicPrintConfig>& project_settings) {
    auto& plate_data = loaded_project.plate_data;

    if (fresh_plate_config) {
        // The prepare call of a run of several plates lays no plate's own
        // settings: its settings serve the run-level steps only, and each
        // plate reports its own when it is sliced.
        HeldEventsScope run_config_events(outcome.prepare_pass && plate_id == 0 &&
                                          (trailing_port || plate_data.size() > 1));
#ifdef ENGINE_BAMBU
        if (apply_bambu_plate_filament_map(o, run_settings, outcome, plate_id, config,
                plate_data, project_settings) != 0)
            return 1;
#endif // ENGINE_BAMBU — BBS toolchanger / per-extruder normalizations
        if (apply_plate_overrides(o, run_settings, outcome, plate_id, config,
                project_settings, loaded_project) != 0)
            return 1;
        report_active_plate_settings(config);
        run_config_events.drop();
    } // fresh_plate_config
    return 0;
}

int prepare_plate_configuration(const CliOptions& o, LoadedProject& loaded_project, RunSettings& run_settings,
                                       const PlatePreparation& preparation, PlateOutcome& outcome, bool trailing_port,
                                       int plate_id, Slic3r::DynamicPrintConfig& config,
                                       std::optional<Slic3r::DynamicPrintConfig>& project_settings) {
    // This plate's settings: a copy of the base with the plate's own and
    // the flags laid on top (BambuStudio.cpp 6902-6904; OrcaSlicer.cpp
    // 5905-5907). The plate the run-level steps ran for keeps the settings
    // those steps read and changed (the arrange's tower).
    const bool fresh_plate_config = !(preparation.prepared && plate_id == preparation.prepared_plate);
    config = fresh_plate_config ? run_settings.base : preparation.run_config;
    // --export-settings writes the project's settings, not this plate's:
    // the official action saves m_print_config before the plate loop
    // applies *part_plate->config() (BambuStudio.cpp 6366-6370 and
    // 6902-6904; OrcaSlicer.cpp 5499-5503 and 5905-5907).
    if (slicer_cli::has_model_actions(o))
        project_settings = run_settings.base;
    if (overlay_fresh_plate_configuration(o, loaded_project, run_settings, outcome,
            trailing_port, fresh_plate_config, plate_id, config, project_settings) != 0)
        return 1;
    // A plate sliced after the run-level steps ran for every plate: the
    // towers those steps placed (the official's m_print_config after its
    // arrange, BambuStudio.cpp 5693-5700, 5856; OrcaSlicer.cpp 4954-4961).
    if (fresh_plate_config && preparation.prepared)
        for (const char* key : {"wipe_tower_x", "wipe_tower_y"})
            if (const Slic3r::ConfigOption* opt = preparation.run_config.option(key))
                config.set_key_value(key, opt->clone());
    return 0;
}

} // namespace slicer_cli
