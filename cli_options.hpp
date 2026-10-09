// cli_options.hpp — what one slicer_cli run was asked to do.
//
// Front end only: the parsed command line, no engine calls. The parser that
// fills it is cli_command_line.cpp; main.cpp and the engine-side files read it.
#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "libslic3r/PrintConfig.hpp"

/// One run's options. The default call (file, --plate, -o, --machine/--process/
/// --filament, --layout-plan) is what existing callers use; every flag added
/// after it only changes behaviour when it is passed.
struct CliOptions {
    std::string argv0;
    std::string input_file;                 // the first model file
    std::vector<std::string> input_files;   // every model file, in command-line order
    std::string output_file = "output.gcode";
    std::string machine_config;
    std::string filament_config;
    std::string process_config;
    std::string bundle_config;
    bool verbose = false;
    int  plate_id = 0;  // 0 = all plates (default); >0 = slice only that plate
    bool normalize_legacy_gcode = true;
    // slicer-cli's own short setting flags (--infill, --perimeters, --nozzle,
    // --temp, --bed-temp), applied as they always were.
    std::map<std::string, std::string> overrides;
    // Every print setting given as an official flag (--curr-bed-type,
    // --layer-height, ...): the official CLI's m_extra_config
    // (BambuStudio.cpp 1563-1564 at 5873b5f; OrcaSlicer.cpp 1258-1259 at
    // 31f6803), laid over the loaded settings like the official does.
    Slic3r::DynamicPrintConfig extra_config;
    // The official flags as the official CLI holds them (m_config after
    // CLI::setup, BambuStudio.cpp 8338-8359 at 5873b5f; OrcaSlicer.cpp
    // 7109-7130 at 31f6803): every CLI key present, the given ones with their
    // value, the rest at their defaults.
    Slic3r::DynamicPrintAndCLIConfig cli;
    // The official flags that were given, in command-line order, split the way
    // CLI::setup splits them (BambuStudio.cpp 8346-8351).
    std::vector<std::string> actions;
    std::vector<std::string> transforms;
    std::set<std::string>    given;   // every official key on the command line
    bool given_flag(const std::string& key) const { return given.count(key) != 0; }

    // --slice / --outputdir: the official CLI's plate loop and output folder.
    bool        slice_mode  = false;
    // --slice is on the command line, whatever its value (the first pass
    // of parse_command_line): a refusal of the value still writes result.json.
    bool        slice_given = false;
    int         slice_plate = 0;   // 0 = every plate
    std::string outputdir;
    // Progress events: on with --slice, or asked for with --progress.
    bool        progress    = false;
    // Official overrides of the refusals below (BambuStudio.cpp: allow_newer_file).
    bool        allow_newer_file   = false;
    bool        allow_substitution = false;
    // --arrange N, official meaning (BambuStudio.cpp 5066-5081 at 5873b5f):
    // 0 = do not arrange, 1 = arrange, any other value = automatic. -1 = not given.
    int         arrange = -1;
    bool arrange_forced() const { return arrange == 1; }
    // --arrange: 0 off, 1 forced, any other value (or none given) automatic,
    // which keeps each input's own need_arrange (BambuStudio.cpp 5066-5081;
    // OrcaSlicer.cpp 4327-4342; "0-disable, 1-enable, others-auto",
    // PrintConfig.cpp 9674).
    bool arrange_auto() const { return arrange != 0 && arrange != 1; }
    // Presets by name, resolved like the desktop app (every parent applied).
    std::string printer_preset;
    std::string process_preset;
    std::vector<std::string> filament_presets;
    // --printer-preset ... on a project 3MF: the named presets are staged as
    // settings files, so the official merge runs for them exactly as it does
    // for --load-settings.
    bool preset_switch = false;
    // --export-3mf NAME: the sliced project, written into --outputdir.
    std::string export_3mf;
    bool uses_presets() const {
        return !printer_preset.empty() || !process_preset.empty() || !filament_presets.empty();
    }
};
