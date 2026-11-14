// --filament-map (slicer_cli): the command line's filament-to-nozzle map on a
// two-nozzle Bambu printer. The check runs before slicing; main.cpp's plate
// block applies the map with Manual mode.
#pragma once

#include <string>
#include <vector>

#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/Model.hpp"
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

/// What --filament-map-mode and --filament-map decided for one plate.
struct FilamentMapFlagStep {
    enum class Kind { NotGiven, Applied, ModeOnly, Refused };
    Kind             kind = Kind::NotGiven;
    std::string      refusal;     // Refused: the sentence
    std::string      mode_name;   // the mode the plate slices with
    std::vector<int> map;         // Applied: the map
    bool             manual = false;
    std::string      message;     // the event's sentence
};

/// --filament-map-mode and --filament-map on one plate: checks them, fills
/// the maps a manual mode needs into `extra`, and sets the mode and the map
/// on `config`. `one_plate` is false when the run slices every plate. Not for
/// the Snapmaker build, whose plate carries no map.
FilamentMapFlagStep filament_map_flag_step(Slic3r::DynamicPrintConfig& extra, Slic3r::DynamicPrintConfig& config,
                                           size_t project_filaments, bool one_plate);

/// The official reach check before the slice: why a filament cannot print
/// where its object is placed, or empty. `blocked` gets the filament. Not for
/// the Snapmaker build, which has no per-extruder areas.
std::string extruder_reach_refusal(const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                   const Slic3r::BuildVolume& build_volume, size_t extruder_count, int& blocked);

} // namespace slicer_cli
