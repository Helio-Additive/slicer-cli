// See cli_nozzle_diameter.hpp. Every function is the official CLI's
// (BambuStudio.cpp 1425-1549 at v02.09.01.70), with Slic3r's
// format_diameter_to_str (Utils.hpp 102) spelling every diameter the way the
// presets do ("0.4"). The Orca engines have no mixed-diameter flow and no
// format_diameter_to_str; the functions are compiled for the Bambu engine only.
#include "cli_nozzle_diameter.hpp"

#ifndef ENGINE_ORCA

#include <cstdlib>

#include <boost/algorithm/string/predicate.hpp>

#include "libslic3r/Utils.hpp"

namespace slicer_cli {

using namespace Slic3r;

// Extract nozzle diameter from a compatible_printers entry like "Bambu Lab A1 0.4 nozzle".
std::string diameter_from_compatible_printer_name(const std::string& printer_name)
{
    static const std::string nozzle_suffix = " nozzle";
    if (printer_name.size() <= nozzle_suffix.size() ||
        !boost::algorithm::ends_with(printer_name, nozzle_suffix))
        return {};
    const size_t nozzle_pos = printer_name.size() - nozzle_suffix.size();
    const size_t prev = printer_name.rfind(' ', nozzle_pos > 0 ? nozzle_pos - 1 : 0);
    if (prev == std::string::npos || prev + 1 >= nozzle_pos)
        return {};
    const double diameter = std::atof(printer_name.substr(prev + 1, nozzle_pos - prev - 1).c_str());
    if (diameter <= 0.)
        return {};
    return format_diameter_to_str(diameter);
}

// Collect supported nozzle diameters from filament compatible_printers names
// (e.g. "Bambu Lab H2C 0.4 nozzle" -> "0.4").
std::set<std::string> filament_supported_diameters(const std::vector<std::string>& compatible_printers)
{
    std::set<std::string> result;
    for (const std::string& printer_name : compatible_printers) {
        const std::string diameter = diameter_from_compatible_printer_name(printer_name);
        if (!diameter.empty())
            result.insert(diameter);
    }
    return result;
}

// Nozzle diameter of a filament preset. A name such as "Generic PLA @BBL H2C 0.4 nozzle"
// carries one diameter. Otherwise the first diameter found in compatible_printers is used.
// Only for empty --load-filaments slots when the CLI did pass filament_diameter_map.
std::string filament_preset_nozzle_diameter(const std::string& preset_name, const DynamicPrintConfig& config)
{
    const std::string from_name = diameter_from_compatible_printer_name(preset_name);
    if (!from_name.empty())
        return from_name;

    const auto* compatible = config.option<ConfigOptionStrings>("compatible_printers");
    if (compatible == nullptr)
        return {};

    const std::set<std::string> supported = filament_supported_diameters(compatible->values);
    if (supported.size() >= 1)
        return *supported.begin();
    return {};
}

// Nozzle diameter of the machine preset, used to fill filament_diameter_map when the CLI
// does not pass one. printer_variant is the preset's diameter (same source as
// primary_nozzle_diameter). Otherwise the first positive nozzle_diameter, then a
// "0.4 nozzle" suffix on the preset name.
std::string default_filament_nozzle_diameter(const std::string& machine_preset_name, const DynamicPrintConfig& machine_config)
{
    if (const auto* variant = machine_config.option<ConfigOptionString>("printer_variant")) {
        const double diameter = std::atof(variant->value.c_str());
        if (diameter > 0.)
            return format_diameter_to_str(diameter);
    }

    if (const auto* diameters = machine_config.option<ConfigOptionFloatsNullable>("nozzle_diameter")) {
        for (double diameter : diameters->values) {
            if (diameter > 0.)
                return format_diameter_to_str(diameter);
        }
    }

    return diameter_from_compatible_printer_name(machine_preset_name);
}

// A filament_diameter_map slot is a selected diameter only when it holds one number.
// "0.2;0.4" lists supported diameters and is not a selection.
void collect_selected_filament_diameters(const DynamicPrintConfig& config, const std::vector<int>& filament_ids,
                                         std::set<std::string>& diameters)
{
    const auto* dia_map = config.option<ConfigOptionStrings>("filament_diameter_map");
    if (dia_map == nullptr)
        return;

    for (int filament_id : filament_ids) {
        if (filament_id <= 0)
            continue;
        const size_t idx = size_t(filament_id - 1);
        if (idx >= dia_map->values.size())
            continue;
        const std::string& raw = dia_map->values[idx];
        if (raw.empty() || raw.find(';') != std::string::npos)
            continue;
        const double diameter = std::atof(raw.c_str());
        if (diameter > 0.)
            diameters.insert(format_diameter_to_str(diameter));
    }
}

// Distinct diameters actually selected in filament_diameter_map. Semicolon lists are ignored.
std::set<std::string> selected_filament_map_diameters(const DynamicPrintConfig& config)
{
    std::set<std::string> diameters;
    const auto* dia_map = config.option<ConfigOptionStrings>("filament_diameter_map");
    if (dia_map == nullptr)
        return diameters;

    std::vector<int> filament_ids;
    filament_ids.reserve(dia_map->values.size());
    for (size_t i = 0; i < dia_map->values.size(); ++i)
        filament_ids.push_back(int(i + 1));
    collect_selected_filament_diameters(config, filament_ids, diameters);
    return diameters;
}

// Nozzles the machine can mount together. Single-head profiles sum to 1, dual-head to 2,
// and a rack (H2C / H3C) sums every slot in extruder_max_nozzle_count.
int machine_max_nozzle_count(const DynamicPrintConfig& config)
{
    int total = 0;
    if (const auto* opt = config.option<ConfigOptionIntsNullable>("extruder_max_nozzle_count")) {
        for (size_t i = 0; i < opt->values.size(); ++i) {
            if (opt->is_nil(i))
                continue;
            if (opt->values[i] > 0)
                total += opt->values[i];
        }
    }
    if (total > 0)
        return total;

    if (const auto* diameters = config.option<ConfigOptionFloatsNullable>("nozzle_diameter")) {
        for (double diameter : diameters->values)
            if (diameter > 0.)
                ++total;
    }
    return total > 0 ? total : 1;
}

std::string plate_single_nozzle_diameter(const DynamicPrintConfig& config, const std::vector<int>& plate_filaments)
{
    if (plate_filaments.empty())
        return {};
    std::set<std::string> plate_diameters;
    collect_selected_filament_diameters(config, plate_filaments, plate_diameters);
    return plate_diameters.size() == 1 ? *plate_diameters.begin() : std::string();
}

std::set<std::string> project_nozzle_diameters(const DynamicPrintConfig& config,
                                               const std::vector<std::vector<int>>& plates_filaments)
{
    std::set<std::string> used_diameters;
    bool found_used_filament = false;
    for (const std::vector<int>& filaments : plates_filaments) {
        found_used_filament |= !filaments.empty();
        collect_selected_filament_diameters(config, filaments, used_diameters);
    }
    if (!found_used_filament)
        used_diameters = selected_filament_map_diameters(config);
    return used_diameters;
}

} // namespace slicer_cli

#endif // ENGINE_ORCA
