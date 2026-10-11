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
    bool        list_settings = false;   // --list-settings
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

/// One official flag whose value names files the run reads.
struct FileOption {
    const char* key;
    bool        vector;   // a list of files (the rest hold one)
};

/// The official flags that read a file (settings to merge, the assemble list,
/// the custom G-codes, --downward-settings): the run opens these, so
/// result.json's guard (named_input_files, main.cpp) and the walk below name
/// the same set.
const std::vector<FileOption>& file_options();

/// The files a command line that was refused before it was fully read names
/// the run to read, read token by token as the parser reads it: a positional
/// word names a model, and a value counts only when the flag in front of it
/// reads a file. A word that is any other flag's value (-o NAME, a
/// --metadata-value) is not a file of the run's, whatever it is spelled like.
struct CommandLineFiles {
    std::vector<std::string> models;   // model files: positional words, --input
    std::vector<std::string> named;    // the value of a flag that reads a file
};
CommandLineFiles command_line_files(const std::vector<std::string>& words);

/// Reads argv into `o` and `m`. Returns false with `refusal` filled when the
/// run must stop; `o.slice_mode` and `o.outputdir` are filled even then, so
/// the caller can write result.json the way --slice always does.
bool parse_command_line(int argc, char** argv, CliOptions& o, ModeArgs& m, ParseRefusal& refusal);

/// --help: usage, slicer-cli's own flags, and every official flag with where
/// it comes from (BambuStudio, OrcaSlicer, both) and whether this build has it.
void print_help(std::ostream& out, const char* prog);

} // namespace slicer_cli
