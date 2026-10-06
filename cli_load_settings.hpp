// cli_load_settings.hpp — --load-settings, --load-filaments and the
// --uptodate family: the official CLI's settings merge.
//
// The official CLI builds the print settings from the files it loaded
// (m_print_config) and then merges the setting files given on the command
// line into them, key by key: a machine and a process with --load-settings,
// filaments with --load-filaments, or, with --uptodate, the newest system
// presets of the same names. It checks that the process suits the printer
// (and refuses a printer change the project does not allow), records what
// changed for the exported project, and recomputes the flush volumes when the
// filament colours or the nozzles change.
//   BambuStudio.cpp 2248-3941 at 5873b5f; OrcaSlicer.cpp 1880-3410 at 31f6803.
// The *_full preset folders the official reads system presets from
// (resources/profiles/BBL/machine_full, process_full and filament_full) are
// not part of the open-source apps (resources/profiles/BBL/cli_config.json
// is); where a file is missing this port carries on with the project's own
// settings and says so in a preset_warning event.
#pragma once

#include <array>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "libslic3r/Preset.hpp"
#include "libslic3r/PrintConfig.hpp"

#include "cli_options.hpp"
#include "cli_run_steps.hpp"

namespace slicer_cli {

/// What the official CLI reads from a Bambu-made 3MF's settings when it loads
/// it (BambuStudio.cpp 1973-2073; OrcaSlicer.cpp 1627-1720).
struct ProjectFacts {
    bool is_bbl_3mf = false;
    std::string current_printer_name, current_process_name, current_printer_model;
    std::string current_printer_system_name, current_process_system_name;
    std::vector<std::string> current_filaments_name, current_filaments_system_name, converted_filaments_system_name;
    std::vector<std::string> current_inherits_group;
    int  current_extruder_count = 1, current_printer_variant_count = 1, current_print_variant_count = 1;
    bool current_is_multi_extruder = false;
    std::vector<int> current_nozzle_volume_type;   // NozzleVolumeType values
    std::vector<std::string> current_extruder_variants;
    std::vector<std::string> upward_compatible_printers, current_print_compatible_printers, current_different_settings;
    int old_printable_width = 0, old_printable_depth = 0, old_printable_height = 0;
    float old_height_to_rod = 0.f, old_height_to_lid = 0.f, old_max_radius = 0.f, old_distance_to_rod = 0.f;
    bool old_exclude_area_empty = true;     // the file's bed_exclude_area (B 2053)
    std::vector<Slic3r::Vec2d> old_exclude_area;   // the same, its points (B 2053; O 1702)
    int filament_count = 0;
};

/// Reads the facts from a Bambu-made 3MF's own settings.
void read_project_facts(const Slic3r::DynamicPrintConfig& file_config, ProjectFacts& facts);

/// What the merge decided, for the steps after it.
struct SettingsMerge {
    bool machine_switch = false;
    bool machine_upwards = false;
    bool disable_wipe_tower_after_mapping = false;
    // Kept for parity: the official reads it only to clear the plate
    // thumbnails (BambuStudio.cpp 5297-5322; OrcaSlicer.cpp
    // 4553-4578), and slicer-cli makes no pictures.
    bool filament_color_changed = false;
    int  filament_count = 0;
    int  new_extruder_count = 1;
    int  new_printer_variant_count = 1;
    bool new_is_multi_extruder = false;
    std::string new_printer_name, new_printer_system_name, new_process_name, printer_model, printer_model_id;
    std::vector<std::string> current_print_extruder_variants, new_printer_extruder_variants;
    std::vector<std::string> upward_machines;
    Slic3r::DynamicPrintConfig load_process_config;   // the orient step reads it
    // The process the project saves after a printer change: a copy of the
    // project's process, marked compatible with the new printer, named
    // "<name>(auto)" (BambuStudio.cpp 3049-3095).
    std::shared_ptr<Slic3r::Preset> new_preset;
    // Input, set by the caller: the colours of an assemble list's OBJ files,
    // which become the filaments (BambuStudio only, BambuStudio.cpp 2573-2620).
    std::vector<std::array<float, 4>> input_obj_colours;
    // Input, set by the caller: whether the settings loaded before the merge
    // (a project's own settings, named presets, --machine/--process/--filament
    // files) carry filament_colour. The official m_print_config holds only
    // what was loaded at that point (the engine defaults come later,
    // BambuStudio.cpp 4115; OrcaSlicer.cpp 3555), so a run of model files
    // with settings files has no filament_colour when the flush volumes are
    // considered, and they are not recomputed (BambuStudio.cpp 3760-3771;
    // OrcaSlicer.cpp 3267-3277). This command line seeds the engine defaults
    // first, so the option's presence alone does not say it.
    bool project_has_filament_colour = true;
};

/// True when the command line asks for any of the merge's inputs.
bool wants_settings_merge(const CliOptions& o);

/// The merge itself, on `print_config` (the official m_print_config), with
/// `extra_config` the settings given as flags (filament_colour is read and
/// taken out here, as the official does). `project_presets` are the 3MF's
/// embedded presets, read to build the "(auto)" process. `profiles_dir` is
/// this engine's own profiles tree (the one --printer-preset resolves names
/// in): the Orca build reads the new printer's process from it, as the desktop
/// does, since its package ships no process_full folder.
StepResult merge_loaded_settings(const CliOptions& o, const ProjectFacts& facts, const std::string& profiles_dir,
                                 Slic3r::DynamicPrintConfig& print_config, Slic3r::DynamicPrintConfig& extra_config,
                                 const std::vector<Slic3r::Preset*>& project_presets, SettingsMerge& merge);

/// After a printer change with a different extruder count, the objects',
/// volumes' and height ranges' own settings follow the new extruders
/// (BambuStudio.cpp 3968-4011; OrcaSlicer.cpp 3462-3505).
void update_object_configs_after_switch(const ProjectFacts& facts, const SettingsMerge& merge,
                                        const Slic3r::DynamicPrintConfig& print_config, Slic3r::Model& model);

/// Every filament slot loaded with --load-filaments is one filament: the
/// prime tower goes off, unless wrapping detection with an exclusion area or
/// a smooth timelapse needs it (BambuStudio.cpp 4176-4215; OrcaSlicer.cpp
/// 3588-3625). `extra_config` is the settings given as flags, which the
/// official has laid over the settings by then (BambuStudio.cpp 4091).
/// True when the tower was turned off.
bool disable_tower_after_mapping(const CliOptions& o, const SettingsMerge& merge,
                                 Slic3r::DynamicPrintConfig& print_config,
                                 const Slic3r::DynamicPrintConfig& extra_config);

/// --load-custom-gcodes and --skip-modified-gcodes on the model's per-plate
/// custom G-code (BambuStudio.cpp 2213-2246, 4013-4044; OrcaSlicer.cpp
/// 1845-1878, 3507-3538).
StepResult apply_custom_gcodes(const CliOptions& o, int plate_to_slice, Slic3r::Model& model);

/// --downward-check: the other printers a plate also fits on (BambuStudio.cpp
/// 4669-4914; OrcaSlicer.cpp 4006-4175). One per machine file
/// (printer_plate_info_t, BambuStudio.hpp 82-105).
struct DownwardPrinter {
    std::string name;
    int    printable_width = 0, printable_depth = 0, printable_height = 0;
    int    exclude_width = 0, exclude_depth = 0;
    int    wrapping_width = 0, wrapping_depth = 0;
    double shared_width = 0., shared_depth = 0., shared_height = 0.;
    float  height_to_lid = 0.f, height_to_rod = 0.f, cleareance_radius = 0.f, distance_to_rod = 0.f;
};

/// The machine files of --downward-settings, or with none the printer's list
/// in resources/profiles/BBL/cli_config.json read from the machine_full folder
/// (BambuStudio.cpp 1440-1488, 4673-4779; OrcaSlicer.cpp 4009-4100). A file
/// that is not a system machine file refuses the run with
/// CLI_CONFIG_FILE_ERROR; a missing machine_full file is skipped with a
/// preset_warning event (the folder is not part of the open-source apps).
StepResult load_downward_printers(const CliOptions& o, const ProjectFacts& facts, std::vector<DownwardPrinter>& printers);

/// The printers `printers` that one plate does not fit: `plate_size` is the
/// plate's objects with its prime tower, `is_sequence` its print-by-object.
std::vector<std::string> downward_failures(const std::vector<DownwardPrinter>& printers, const Slic3r::Vec3d& plate_size,
                                           bool is_sequence, const Slic3r::DynamicPrintConfig& config,
                                           const ProjectFacts& facts, bool has_support);

} // namespace slicer_cli
