// --filament-map (slicer_cli): the command line's filament-to-nozzle map on a
// two-nozzle Bambu printer. The check runs before slicing; main.cpp's plate
// block applies the map with Manual mode.
#pragma once

#include <string>
#include <vector>

#include "libslic3r/PrintConfig.hpp"

namespace slicer_cli {

/// --filament-map: why the command line's filament-to-nozzle map cannot be
/// used on this printer and project, or empty when it can. The map is the
/// official CLI's --filament_map (one 1-based logical extruder per filament,
/// BambuStudio.cpp 7208-7249 at v02.09.01.70), which Manual mode then holds
/// to: ToolOrdering throws "Group error in manual mode" when the grouping it
/// finds differs (ToolOrdering.cpp 2653-2680). Extruder 1 is the left nozzle,
/// 2 the right, as Bambu Studio numbers them (FilamentMapPanel.cpp 488).
std::string filament_map_flag_refusal(const std::vector<int>& map,
                                      const Slic3r::DynamicPrintConfig& config,
                                      size_t project_filaments);

} // namespace slicer_cli
