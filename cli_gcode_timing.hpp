// cli_gcode_timing.hpp -- isolated existing-G-code timing command.
#pragma once

namespace slicer_cli {

/// Runs --gcode-timing when present.  Returns true when it printed the command's
/// one JSON response and placed its process status in `exit_code`.
bool run_gcode_timing_command(int argc, char **argv, int &exit_code);

/// Writes the timing command's schema-valid unavailable response.
void write_gcode_timing_unavailable(int &exit_code);

} // namespace slicer_cli
