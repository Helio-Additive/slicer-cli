// What a desktop app does to a project's settings when it opens the project,
// beyond loading them over the presets (main.cpp's project block): the prime
// tower Bambu Studio puts back inside the plate, and the list of the settings
// the load replaced.
#pragma once

#include <functional>
#include <string>
#include <vector>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/PrintConfig.hpp"

namespace slicer_cli {

#ifdef ENGINE_BAMBU
/// Opening a project, Bambu Studio puts each plate's saved tower position back
/// inside the plate (Plater.cpp 12270-12276 at v02.09.01.70:
/// set_default_wipe_tower_pos_for_plate with keep_existing, PartPlate.cpp
/// 5141-5221). The list gets one position per plate first, a plate without
/// one taking the first plate's (5146-5148). The tower is sized for the filaments the plate's objects use,
/// and for two when that size is zero (5183-5187); its corner is then kept
/// WIPE_TOWER_MARGIN inside the area every nozzle reaches, pinned to the low
/// edge when the tower is larger than that area (wipe_tower_legal_corner_range
/// and wipe_tower_pullback_then_avoid, WipeTowerPlacement.cpp 86-97, 403-417;
/// a kept position has no forbidden regions, PartPlate.cpp 5194-5196).
/// `reach` is that area at the plate's own origin, `tower_size` the size the
/// desktop estimates for a number of filaments on this plate. OrcaSlicer and
/// Snapmaker Orca keep the saved position when they open a project: their
/// Plater calls this only after a printer change (OrcaSlicer Plater.cpp
/// 10903-10907; Snapmaker_Orca PartPlate.cpp 3799, 4277, 4686).
void project_load_tower_inside_plate(Slic3r::DynamicPrintConfig& config, int plate_index, int plate_count,
                                     const Slic3r::BoundingBoxf& reach, int plate_filaments,
                                     const std::function<Slic3r::Vec3d(int)>& tower_size);
#endif

/// One tower position per plate, a plate the list holds none for taking the
/// first plate's, as the desktop apps read it (Bambu Studio at load,
/// PartPlate.cpp 5146-5148; all three at slice, ConfigOptionVector::get_at).
/// --project-settings-as-saved does not call it, so such a list keeps the
/// official command line's reading.
void project_load_tower_positions_per_plate(Slic3r::DynamicPrintConfig& config, int plate_count);

/// The settings whose value in the project file the presets replaced when the
/// project was loaded the way its desktop app loads it: the desktop drops them
/// without a word, and a caller that edited the file must be able to read what
/// was not kept. The two lists the load itself rewrites are not settings, and
/// a list that only changed its length (one value where the loaded presets
/// hold the same value once per extruder or variant) holds the same setting.
std::vector<std::string> settings_replaced_by_presets(const Slic3r::DynamicPrintConfig& file,
                                                      const Slic3r::DynamicPrintConfig& loaded);

} // namespace slicer_cli
