// cli_gcode_timing.hpp -- isolated existing-G-code timing command.
#pragma once

namespace slicer_cli {

// gcode_timing_requested() lives in cli_command_line.hpp: it applies the
// parser's own value-consumption rules to tell the option from a value.

/// Runs --gcode-timing: prints the command's one JSON response document on
/// stdout and returns the process exit status (0 timed, 64 bad arguments,
/// 1 unreadable input, environment or importer failure).
int run_gcode_timing_command(int argc, char **argv);

/// Prints the schema-valid UNAVAILABLE response for an environment failure
/// before the command runs (resource discovery threw) and returns its exit
/// status, 1.
int write_gcode_timing_unavailable();

} // namespace slicer_cli
