// The nozzle and filament-grouping settings the official command lines read
// from the command line, and the Bambu Studio dialog's grouping on a printer
// with one extruder and a nozzle rack. cli_slice_plate.cpp calls these where the official
// CLI runs the matching block; cli_filament_map.cpp checks --filament-map.
#pragma once

#include <string>
#include <vector>

#include "libslic3r/PrintConfig.hpp"
#ifdef ENGINE_BAMBU
#include "libslic3r/PresetBundle.hpp"   // ExtruderNozzleStat
#endif

namespace slicer_cli {

/// --filament-map values of -1 take the first value given, else 1
/// (BambuStudio.cpp 7208-7249 at v02.09.01.70; OrcaSlicer.cpp 6784-6822 at
/// f92a7c0).
std::vector<int> sanitized_filament_map(std::vector<int> map);

#ifdef ENGINE_BAMBU
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
#endif

#ifndef ENGINE_SNAPMAKER
/// A manual mode from the command line (BambuStudio.cpp 7284-7345; OrcaSlicer.cpp
/// 6840-6899 at f92a7c0): Nozzle Manual needs a nozzle rack and its settings
/// (five in Bambu Studio; the three maps in OrcaSlicer, which has no
/// command-line rack flags), and Manual on a rack printer takes each
/// filament's flow type from its extruder unless the extruder mixes types.
/// Writes filament_volume_map into `extra`. Returns a refusal sentence, or
/// empty.
std::string manual_mode_nozzle_maps(Slic3r::DynamicPrintConfig& extra, const Slic3r::DynamicPrintConfig& config,
                                    Slic3r::FilamentMapMode mode, size_t filament_count, bool one_plate);

/// A manual mode on a printer with several extruders or nozzles: a filament
/// the plate uses whose filament_printable does not allow the extruder its
/// map names (BambuStudio.cpp 7357-7400; OrcaSlicer.cpp 6912-6933 at f92a7c0; the official CLI ends
/// CLI_FILAMENTS_NOT_SUPPORTED_BY_EXTRUDER). `plate_filaments` are the
/// 1-based filaments the plate's objects use. Returns a refusal sentence, or
/// empty.
std::string manual_map_extruder_refusal(const Slic3r::DynamicPrintConfig& config, const std::vector<int>& plate_filaments);
#endif

#ifdef ENGINE_BAMBU
/// A project moved to another printer (a printer settings file): the Bambu
/// Studio app keeps the file's nozzles on an extruder only when they hold the
/// new printer's diameter (on a nozzle rack, when any nozzle is installed);
/// another extruder takes the printer's default nozzles (NozzleInventory
/// resolve and holds_selected_diameter, NozzleInventory.cpp 69-76, 523-541,
/// read_selected_printer 128-155, at v02.09.01.70). Rewrites the per-slot
/// nozzle keys of `config` when an extruder's do not hold it; returns whether
/// it did. `config` already holds the new printer's settings.
bool fit_project_nozzles_to_printer(Slic3r::DynamicPrintConfig& config);
#endif

} // namespace slicer_cli
