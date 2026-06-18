// calib_args.hpp — driver-side calibration CLI support (slicer-cli #5).
//
// Parses the `--calib-*` flag family into the engine's Calib_Params and drives
// Print::set_calib_params() so the engine's per-layer calibration emission
// (GCode.cpp) runs headlessly. For the geometry-generating pressure_advance_
// pattern mode it also ports the GUI-free CalibUtils::calib_pa_pattern template
// (no engine edits — the driver only calls public engine APIs).
#pragma once

#include <string>

// The calib header is `Calib.hpp` (capital) in BambuStudio but `calib.hpp`
// (lowercase) in OrcaSlicer. macOS's case-insensitive FS hides the difference,
// but a case-sensitive Linux CI build needs the exact case per engine.
#ifdef ENGINE_ORCA
#include "libslic3r/calib.hpp"
#else
#include "libslic3r/Calib.hpp"
#endif
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"

namespace slicer_cli {

// Raw options collected from the CLI flags, before validation / defaulting.
struct CalibOptions {
    std::string mode;            // raw --calib-mode string ("" => no calibration)
    double      start = 0.0;
    double      end   = 0.0;
    double      step  = 0.0;
    int         extruder_id = 0;
    bool        print_numbers = true;   // PA-line numbering (--calib-no-numbers => false; the pattern always labels)
    bool        has_start = false;
    bool        has_end   = false;
    bool        has_step  = false;
};

// Map a --calib-mode token to the engine enum. Empty string => Calib_None.
// Throws std::invalid_argument for an unknown non-empty token.
Slic3r::CalibMode parse_calib_mode(const std::string& s);

// Canonical (lower-snake) name for a CalibMode, for usage/help and messages.
std::string calib_mode_name(Slic3r::CalibMode mode);

// True for the only mode that generates its own geometry and so may run without
// a user-supplied --input model (pressure_advance_pattern).
bool calib_mode_generates_geometry(Slic3r::CalibMode mode);

// Build & validate engine Calib_Params from CLI options. Fills canonical sweep
// defaults for any of start/end/step the user omitted. Throws
// std::invalid_argument on an unknown mode or an insane range (step <= 0,
// non-finite values, or a span smaller than one step).
Slic3r::Calib_Params build_calib_params(const CalibOptions& opts);

// GUI-free port of CalibUtils::calib_pa_pattern (verified pure-libslic3r).
// Applies the SuggestedConfigCalibPAPattern keys, sets outer_wall_speed via
// find_optimal_PA_speed, synthesizes a handle cube when `model` has no objects,
// constructs the pattern, centers it on the printable area and stores
// model.calib_pa_pattern. MUST be called BEFORE Print::apply (apply snapshots
// model + config and reads plates_custom_gcodes).
void apply_pa_pattern(const Slic3r::Calib_Params& params,
                      Slic3r::DynamicPrintConfig& config,
                      Slic3r::Model& model,
                      bool is_bbl_machine);

} // namespace slicer_cli
