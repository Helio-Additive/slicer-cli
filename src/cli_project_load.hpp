// What a desktop app does to a project's settings when it opens the project,
// beyond loading them over the presets (cli_load_slice_inputs.cpp): the prime
// tower Bambu Studio puts back inside the plate, and the list of the settings
// the load replaced.
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "libslic3r/BoundingBox.hpp"

namespace Slic3r { class DynamicPrintConfig; }

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

/// The keys of `replaced` that printed with a value other than the file's:
/// the load's list holds every key whose loaded value differs from the file's,
/// and on a printer with extruder variants the presets hold one value per
/// variant, which the print narrows to the plate's own (Print::apply), so a
/// key can differ at load and print with the file's value. A key the command
/// line set (`command_line`: the setting flags and the short flags) is its own change, not the presets', and a key
/// the print does not hold (a preset's list of compatible printers and the
/// like) prints nothing, the keys that index the per-variant lists are not
/// settings, neither is a key the app does not show (no label, or developer
/// mode only), and a project-level setting is the project's, not a preset's.
/// A key's file values are its value as the engine read it and, for a text
/// value, the text the file writes. `file_values` holds the value of each key the file
/// states; a key it does not state was never the file's value.
void keep_replaced_printed_differently(std::vector<std::string>& replaced,
                                       const std::map<std::string, std::vector<std::string>>& file_values,
                                       const Slic3r::DynamicPrintConfig& printed,
                                       const std::set<std::string>& command_line);

} // namespace slicer_cli
