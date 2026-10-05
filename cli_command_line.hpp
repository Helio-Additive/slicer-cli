// cli_command_line.hpp — reading slicer_cli's command line.
//
// Two flag families, read in one pass:
//   - the official CLIs' flags, parsed by this engine's own
//     DynamicPrintAndCLIConfig::read_cli (Config.cpp 1660-1782 at 5873b5f,
//     1591-1713 at 31f6803) over its own CLI definitions (CLIActionsConfigDef,
//     CLITransformConfigDef, CLIMiscConfigDef in PrintConfig.cpp) and every
//     print setting, spelled as the key with dashes (ConfigOptionDef::cli_args,
//     Config.cpp 242-259 / 238-255);
//   - slicer-cli's own flags (presets by name, --engine-info, the legacy short
//     setting flags, layout and calibration), from a small table beside it.
// An official flag slicer-cli does not support refuses by name; it never falls
// through to "Unknown option".
#pragma once

#include <iosfwd>
#include <set>
#include <string>
#include <vector>

#include "calib_args.hpp"
#include "cli_options.hpp"

namespace slicer_cli {

/// The flags that pick a non-slicing mode (layout, presets list, file report).
struct ModeArgs {
    std::string layout_json_file;   // --layout
    bool        layout_plan_mode = false;
    std::string engine_info_file;   // --engine-info FILE
    bool        list_presets = false;
    std::string list_printer;       // --printer (with --list-presets)
    bool        help = false;
    CalibOptions calib;
};

/// A command line that cannot run: the official result code and why.
struct ParseRefusal {
    int         code = 0;
    std::string message;
    // A refusal slicer-cli has always made this way: the sentence, then the
    // usage text, exit 1 and no result.json.
    bool        with_usage = false;
};

/// Reads argv into `o` and `m`. Returns false with `refusal` filled when the
/// run must stop; `o.slice_mode` and `o.outputdir` are filled even then, so
/// the caller can write result.json the way --slice always does.
bool parse_command_line(int argc, char** argv, CliOptions& o, ModeArgs& m, ParseRefusal& refusal);

/// --help: usage, slicer-cli's own flags, and every official flag with where
/// it comes from (BambuStudio, OrcaSlicer, both) and whether this build has it.
void print_help(std::ostream& out, const char* prog);

} // namespace slicer_cli
