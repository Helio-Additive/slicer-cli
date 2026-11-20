// Nozzle diameters per filament (Bambu Studio 2.9.1's mixed nozzle diameter
// flow): the helpers the official CLI uses to fill and check
// filament_diameter_map, ported from BambuStudio.cpp 1425-1531 at
// v02.09.01.70. Each filament slot names the nozzle diameter it prints with
// ("0.4"), or lists the diameters it supports ("0.2;0.4") when none is
// selected. multi_nozzle_diameter_mode (a project setting, written to the
// 3MF) turns the checks on.
#pragma once

#include <set>
#include <string>
#include <vector>

#include "libslic3r/PrintConfig.hpp"

namespace slicer_cli {

/// "Bambu Lab A1 0.4 nozzle" -> "0.4"; empty when the name carries no
/// diameter (diameter_from_compatible_printer_name, BambuStudio.cpp 1425-1440).
std::string diameter_from_compatible_printer_name(const std::string& printer_name);

/// The diameters a filament's compatible_printers names carry
/// (filament_supported_diameters, 1442-1452).
std::set<std::string> filament_supported_diameters(const std::vector<std::string>& compatible_printers);

/// A filament preset's own diameter: from its name, else the first one its
/// compatible_printers name (filament_preset_nozzle_diameter, 1454-1470).
std::string filament_preset_nozzle_diameter(const std::string& preset_name, const Slic3r::DynamicPrintConfig& config);

/// The machine preset's diameter: printer_variant, else the first positive
/// nozzle_diameter, else the preset name (default_filament_nozzle_diameter,
/// 1472-1491).
std::string default_filament_nozzle_diameter(const std::string& machine_preset_name, const Slic3r::DynamicPrintConfig& machine_config);

/// The diameters the given filaments (1-based) select in filament_diameter_map;
/// a slot listing several ("0.2;0.4") selects none
/// (collect_selected_filament_diameters, 1493-1512).
void collect_selected_filament_diameters(const Slic3r::DynamicPrintConfig& config, const std::vector<int>& filament_ids,
                                         std::set<std::string>& diameters);

/// Every diameter the map selects (selected_filament_map_diameters, 1514-1527).
std::set<std::string> selected_filament_map_diameters(const Slic3r::DynamicPrintConfig& config);

/// How many nozzles the machine mounts at once: the sum of
/// extruder_max_nozzle_count, else the positive nozzle_diameter entries, else 1
/// (machine_max_nozzle_count, 1529-1549).
int machine_max_nozzle_count(const Slic3r::DynamicPrintConfig& config);

/// With multi_nozzle_diameter_mode on, a plate whose filaments all select one
/// nozzle diameter cannot be uploaded: that diameter, else empty (a plate with
/// no filament, or with two or more diameters, passes; BambuStudio.cpp
/// 5066-5086).
std::string plate_single_nozzle_diameter(const Slic3r::DynamicPrintConfig& config, const std::vector<int>& plate_filaments);

/// The diameters the project's plates select, one list of filaments per
/// plate; when no plate uses a filament, every map slot counts
/// (BambuStudio.cpp 5218-5236).
std::set<std::string> project_nozzle_diameters(const Slic3r::DynamicPrintConfig& config,
                                               const std::vector<std::vector<int>>& plates_filaments);

} // namespace slicer_cli
