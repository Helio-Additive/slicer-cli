#include "cli_filament_map.hpp"

#include <algorithm>
#include <cstdlib>

namespace slicer_cli {

std::string filament_map_flag_refusal(const std::vector<int>& map,
                                      const Slic3r::DynamicPrintConfig& config,
                                      size_t project_filaments)
{
    size_t extruders = 1;
    if (const auto* nozzles = dynamic_cast<const Slic3r::ConfigOptionVectorBase*>(config.option("nozzle_diameter")))
        extruders = std::max<size_t>(1, nozzles->size());
    std::string printer;
    if (const auto* name = config.option<Slic3r::ConfigOptionString>("printer_settings_id"); name && !name->value.empty())
        printer = " (" + name->value + ")";
#ifdef ENGINE_ORCA
    // This engine groups filaments on a Bambu Lab printer with two extruders
    // only; on any other printer with several extruders filament N prints
    // with extruder N whatever the map says
    // (ToolOrdering::get_recommended_filament_maps, ToolOrdering.cpp 1154 and
    // 1208-1210 at 8500fcd; the G-code's tool is the filament's own). main.cpp
    // tells the engine a Bambu Lab printer by the same printer_model rule.
    if (const auto* model = config.option<Slic3r::ConfigOptionString>("printer_model");
        extruders >= 2 && (model == nullptr || model->value.rfind("Bambu Lab", 0) != 0))
        return "--filament-map chooses between a Bambu Lab printer's left and right extruder; on this printer" + printer +
               " every filament has its own extruder (filament 1 prints with extruder 1, and so on), so there is "
               "nothing to choose. Drop the flag to slice.";
#endif
    // One extruder that changes between several nozzles (the H3C): the engine
    // picks the nozzle, filament_map stays all 1 (ToolOrdering.cpp 2670-2672).
    bool nozzle_rack = false;
    if (const auto* counts = dynamic_cast<const Slic3r::ConfigOptionVectorBase*>(config.option("extruder_max_nozzle_count")))
        for (const std::string& count : counts->vserialize())
            nozzle_rack = nozzle_rack || std::atoi(count.c_str()) > 1;
    if (extruders < 2 && nozzle_rack) {
        // With --filament-nozzle-map the map is the engine's own, all 1; the
        // dialog's 1/2 grouping is rack_group_maps (cli_nozzle_options.cpp).
        for (int extruder : map)
            if (extruder != 1)
                return "--filament-map with --filament-nozzle-map: this printer" + printer + " has one extruder, so every "
                       "--filament-map value is 1 and --filament-nozzle-map names each filament's nozzle.";
        return {};
    }
    if (extruders < 2)
        return "--filament-map chooses which nozzle each filament prints with on a two-nozzle printer (such as the "
               "H2D or H2C); this printer" + printer + " has one nozzle. Drop the flag to slice.";
    const size_t filaments = std::max<size_t>(1, project_filaments);
    if (map.size() != filaments)
        return "--filament-map gives " + std::to_string(map.size()) + " value" + (map.size() == 1 ? "" : "s") +
               " but the project has " + std::to_string(filaments) + " filament" + (filaments == 1 ? "" : "s") +
               ": give one nozzle per filament, in filament order (1 = left, 2 = right), e.g. --filament-map " +
               (filaments == 1 ? std::string("1") : std::string("1,2")) + ".";
    for (size_t i = 0; i < map.size(); ++i)
        if (map[i] < 1 || map[i] > (int)extruders)
            return "--filament-map gives nozzle " + std::to_string(map[i]) + " for filament " + std::to_string(i + 1) +
                   ", but this printer" + printer + " has nozzles 1 to " + std::to_string(extruders) +
                   " (1 = left, 2 = right).";
    return {};
}

} // namespace slicer_cli
