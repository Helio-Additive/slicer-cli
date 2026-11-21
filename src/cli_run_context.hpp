#pragma once

#include "cli_staging_folders.hpp"

#include <memory>
#include <string>
#include <vector>

struct CliOptions;
namespace Slic3r { class DynamicPrintConfig; }

namespace slicer_cli {

struct AssembleList;
struct ModeArgs;

/// The official arrange switches given on the command line (cli_options.hpp).
struct ArrangeSwitches {
    bool enable_timelapse            = false;   // --enable-timelapse
    bool allow_rotations             = false;   // --allow-rotations (see below)
    bool allow_multicolor_oneplate   = true;    // --allow-multicolor-oneplate
    bool avoid_extrusion_cali_region = false;   // --avoid-extrusion-cali-region
};

/// --downward-check's result: the official checks every plate once per run
/// (BambuStudio.cpp 4645-4658, 4779-4790), so a --slice 0 run checks on its
/// first pass and every later pass reports the same.
struct DownwardRun {
    bool done = false;
    std::vector<std::string> printers, failed;
    bool sequence_plate = false;
    bool multi_nozzle_diameters = false;
};

struct CliRunContext {
    // The settings the named presets resolve to, computed once per run (loading a
    // profiles tree takes seconds) and applied to every plate.
    std::unique_ptr<Slic3r::DynamicPrintConfig> g_preset_config;

    // --load-assemble-list: the list's plates and objects, built once per run
    // before the plate loop (as BambuStudio.cpp 2165-2188 builds them before
    // its own); each plate's slice copies its objects out.
    std::unique_ptr<slicer_cli::AssembleList> g_assemble;

    /// The parts of an assemble list the run is building — the ones its loaders
    /// have read, in the order they read them — or, if the build turned the list
    /// down, those same parts: they were read while it was built, and a refusal
    /// then writes result.json too, from run_slice_mode or from its caller. Those
    /// parts and the files their loaders read beside them are the run's own inputs
    /// by then, so the write is held back for them as it is for a built list's. A
    /// part no loader opened is not here: nothing read it, and reading it beside
    /// the input list can hold the run (a part that is a pipe). Empty while no
    /// assemble list is in hand.
    std::vector<std::string> g_assemble_pending_sources;

    /// --skip-useless-pick (BambuStudio only), read where the export fills the plate.
    bool g_skip_useless_pick = false;

    /// The printer model id the desktop app stores per plate for presets picked
    /// by name (Preset::get_printer_type, Plater.cpp 22261 at 5873b5f).
    std::string g_preset_printer_model_id;

    /// The run's command line, for result.json never to write over a file it
    /// reads (set once parsing starts), and its words as typed: a refused command
    /// line may stop parsing before it reaches the flag that names the file.
    const CliOptions* g_result_json_options = nullptr;
    /// The modes' own files (--layout JSON, --engine-info FILE): named to the run
    /// to read like the settings files, and not held by CliOptions.
    const slicer_cli::ModeArgs* g_result_json_mode = nullptr;
    std::vector<std::string> g_command_line_words;
    bool g_command_line_parsed = false;   // the whole command line was read

    ArrangeSwitches g_arrange_switches;

    DownwardRun g_downward_run;

#ifdef ENGINE_SNAPMAKER
    /// The filament-id remap a printer switch leaves behind, and the filament total
    /// it maps into. The desktop applies it to every volume's painted facets when
    /// the filament count changes (references/SnapmakerOrca/src/slic3r/GUI/Plater.cpp:22180-22290
    /// at bac6fd5); the remap is built by
    /// PresetBundle::update_multi_material_filament_presets (PresetBundle.cpp:3983-4094).
    std::vector<unsigned int> g_filament_id_remap;
    size_t                    g_filament_id_total = 0;
    // The desktop remaps only when the filament count changes (on_filaments_change);
    // a run without a printer switch that changes it leaves the paint alone.
    bool                      g_filament_count_changed = false;

#endif

#if defined(ENGINE_SNAPMAKER) || defined(ENGINE_ORCA)
    /// Whether this file was made by BambuStudio, so BambuStudio's own meaning of a
    /// value applies to it and no one else's. Set from the maker check.
    bool g_file_made_by_bambustudio = false;
#endif
    RunStagingFolders run_staging;
};

} // namespace slicer_cli
