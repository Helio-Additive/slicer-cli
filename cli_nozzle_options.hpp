// The nozzle and filament-grouping settings the official command lines read
// from the command line, and the Bambu Studio dialog's grouping on a printer
// with one extruder and a nozzle rack. main.cpp calls these where the official
// CLI runs the matching block; cli_filament_map.cpp checks --filament-map.
#pragma once

#include <string>
#include <vector>

#include "libslic3r/PrintConfig.hpp"
#ifndef ENGINE_ORCA
#include "libslic3r/PresetBundle.hpp"   // ExtruderNozzleStat
#endif

namespace slicer_cli {

/// --filament-map values of -1 take the first value given, else 1
/// (BambuStudio.cpp 7208-7249 at v02.09.01.70; OrcaSlicer.cpp 5889-5927 at
/// 8500fcd).
std::vector<int> sanitized_filament_map(std::vector<int> map);

#ifndef ENGINE_ORCA
/// --estimate-mode on a printer with several extruders or nozzles
/// (BambuStudio.cpp 4441-4474): a mode other than Auto For Flush is refused
/// (the sentence is returned), and the run slices with Auto For Flush, the
/// Standard flow type on every extruder and a full rack of Standard nozzles.
std::string estimate_mode_nozzle_settings(Slic3r::DynamicPrintConfig& extra, const Slic3r::DynamicPrintConfig& config);

/// --extruder-nozzle-count with --extruder-nozzle-volume-type: the rack as the
/// command line describes it (BambuStudio.cpp 7103-7172). Fills `stats` and
/// the per-slot keys of `config`; false when the command line gave no rack.
bool command_line_nozzle_rack(Slic3r::DynamicPrintConfig& extra, Slic3r::DynamicPrintConfig& config,
                              int new_extruder_count, Slic3r::ExtruderNozzleStat& stats);

/// --ams-nozzle-idx on a printer with one extruder and a nozzle rack: the
/// plate's extruder_ams_count follows it, an AMS when the index names a
/// nozzle and none when it is -1 (BambuStudio.cpp 7448-7455). The engine
/// reads the AMS nozzle only with a count above zero
/// (get_effective_ams_nozzle_idx, PrintConfig.cpp 1176-1195). Without the
/// flag the input's own count stands, as in the app with no printer connected.
void sync_ams_count_with_command_line(const Slic3r::DynamicPrintConfig& extra, Slic3r::DynamicPrintConfig& config);

/// "Auto For Quality" groups by the difference between a printer's extruders,
/// so the engine and the app offer it only when their types differ
/// (Print::get_available_filament_map_modes, Print.cpp 3427-3447;
/// FilamentMapDialog.cpp 105-112). Returns a refusal sentence, or empty.
std::string quality_mode_refusal(const Slic3r::DynamicPrintConfig& config);

/// One extruder with a nozzle rack (the H3C) in Manual: the Bambu Studio
/// dialog's two groups, 1 = the nozzle the AMS feeds and 2 = the fixed
/// nozzles (build_multi_nozzle_manual_maps, FilamentMapDialog.cpp 144-170).
/// `map` becomes the engine's filament_map (all 1) and `nozzle_map` the
/// group's nozzle per filament; the engine picks the fixed nozzle that fits
/// (ToolOrdering.cpp 2537-2551). Returns a refusal sentence, or empty.
std::string rack_group_maps(std::vector<int>& map, std::vector<int>& nozzle_map,
                            const Slic3r::DynamicPrintConfig& config, size_t project_filaments);

/// A manual mode from the command line (BambuStudio.cpp 7284-7345): Nozzle
/// Manual needs a nozzle rack and all five of its settings, and Manual on a
/// rack printer takes each filament's flow type from its extruder unless the
/// extruder mixes types. Writes filament_volume_map into `extra`. Returns a
/// refusal sentence, or empty.
std::string manual_mode_nozzle_maps(Slic3r::DynamicPrintConfig& extra, const Slic3r::DynamicPrintConfig& config,
                                    Slic3r::FilamentMapMode mode, size_t filament_count, bool one_plate);

/// A manual mode on a printer with several extruders or nozzles: a filament
/// the plate uses whose filament_printable does not allow the extruder its
/// map names (BambuStudio.cpp 7357-7400; the official CLI ends
/// CLI_FILAMENTS_NOT_SUPPORTED_BY_EXTRUDER). `plate_filaments` are the
/// 1-based filaments the plate's objects use. Returns a refusal sentence, or
/// empty.
std::string manual_map_extruder_refusal(const Slic3r::DynamicPrintConfig& config, const std::vector<int>& plate_filaments);
#endif

} // namespace slicer_cli
