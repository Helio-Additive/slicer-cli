#include "cli_filament_map.hpp"
#include "cli_nozzle_options.hpp"

#include <algorithm>
#include <iterator>
#include <set>
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
        const size_t rack_filaments = std::max<size_t>(1, project_filaments);
        if (map.size() != rack_filaments)
            return "--filament-map gives " + std::to_string(map.size()) + " value" + (map.size() == 1 ? "" : "s") +
                   " but the project has " + std::to_string(rack_filaments) + " filament" +
                   (rack_filaments == 1 ? "" : "s") + ": give 1 for every filament, in filament order.";
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

#ifndef ENGINE_SNAPMAKER
FilamentMapFlagStep filament_map_flag_step(Slic3r::DynamicPrintConfig& extra, Slic3r::DynamicPrintConfig& config,
                                           size_t project_filaments, bool one_plate) {
    using Slic3r::FilamentMapMode;
    FilamentMapFlagStep step;
    const auto* cli_mode = extra.option<Slic3r::ConfigOptionEnum<FilamentMapMode>>("filament_map_mode");
    auto* cli_map = extra.option<Slic3r::ConfigOptionInts>("filament_map");
    if (cli_mode == nullptr && cli_map == nullptr)
        return step;
    const FilamentMapMode mode = cli_mode ? cli_mode->value : FilamentMapMode::fmmManual;
#ifdef ENGINE_BAMBU
    step.manual = mode == FilamentMapMode::fmmManual || mode == FilamentMapMode::fmmNozzleManual;
    bool rack_groups = false;
#else
    step.manual = mode == FilamentMapMode::fmmManual;
    (void)one_plate;
#endif
    const auto refuse = [&](const std::string& why) {
        step.kind    = FilamentMapFlagStep::Kind::Refused;
        step.refusal = why;
        return step;
    };
#ifdef ENGINE_BAMBU
    if (mode == FilamentMapMode::fmmAutoForQuality)
        if (const std::string why = quality_mode_refusal(config); !why.empty())
            return refuse(why);
#endif
    if (cli_map != nullptr) {
        cli_map->values = sanitized_filament_map(cli_map->values);
        std::string why;
#ifdef ENGINE_BAMBU
        if (mode == FilamentMapMode::fmmManual && Slic3r::is_single_ext_multi_nozzle(config) &&
            !extra.has("filament_nozzle_map")) {
            std::vector<int> nozzle_map;
            why = rack_group_maps(cli_map->values, nozzle_map, config, project_filaments);
            if (why.empty())
                extra.option<Slic3r::ConfigOptionInts>("filament_nozzle_map", true)->values = nozzle_map;
            rack_groups = true;
        } else
#endif
        if (step.manual)
            why = filament_map_flag_refusal(cli_map->values, config, project_filaments);
        if (!why.empty())
            return refuse(why);
    }
#ifdef ENGINE_BAMBU
    if (step.manual) {
        const std::string why =
            manual_mode_nozzle_maps(extra, config, mode, std::max<size_t>(1, project_filaments), one_plate);
        if (!why.empty())
            return refuse(why);
        // The maps this step wrote, over the plate's (BambuStudio.cpp 7332-7339).
        for (const char* key : {"filament_volume_map", "filament_nozzle_map"})
            if (const Slic3r::ConfigOption* opt = extra.option(key))
                config.set_key_value(key, opt->clone());
    }
#endif
    step.mode_name = Slic3r::ConfigOptionEnum<FilamentMapMode>(mode).serialize();
    Slic3r::ConfigSubstitutionContext subst(Slic3r::ForwardCompatibilitySubstitutionRule::Enable);
    config.set_deserialize("filament_map_mode", step.mode_name, subst);
    if (cli_map == nullptr) {
        step.kind    = FilamentMapFlagStep::Kind::ModeOnly;
        step.message = "--filament-map-mode sets the filament grouping mode to " + step.mode_name;
        return step;
    }
    step.kind = FilamentMapFlagStep::Kind::Applied;
    step.map  = cli_map->values;
    config.option<Slic3r::ConfigOptionInts>("filament_map", true)->values = step.map;
    step.message =
#ifdef ENGINE_BAMBU
        rack_groups ? "--filament-map sets each filament's nozzle group (1 = the nozzle the AMS feeds, 2 = the "
                      "fixed nozzles) in Manual mode; filament_map itself is 1 for every filament" :
#endif
        step.manual ? "--filament-map sets each filament's nozzle (1 = left, 2 = right) in " + step.mode_name + " mode"
                    : "--filament-map is given with the automatic mode " + step.mode_name +
                      ": the engine groups the filaments itself";
    return step;
}
#endif // ENGINE_SNAPMAKER

#ifndef ENGINE_SNAPMAKER
/// Nozzles that each reach only part of the bed: a one-filament object in
/// an area some extruders do not reach makes its filament unprintable on
/// those (BambuStudio.cpp 7026-7052 at v02.09.01.70; OrcaSlicer.cpp
/// 5795-5806 at 8500fcd). In an automatic mode a filament no extruder can
/// print ends the run, in a manual mode a filament whose map names an
/// extruder that cannot reach it (7254-7281, 7347-7355; OrcaSlicer.cpp
/// 5937), both CLI_FILAMENT_CAN_NOT_MAP, before the slice.
std::string extruder_reach_refusal(const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                   const Slic3r::BuildVolume& build_volume, size_t extruder_count, int& blocked) {
    blocked = 0;
    std::vector<std::set<int>> unprintable(extruder_count);
#ifndef ENGINE_ORCA
    const Slic3r::GlobalFeatureFilaments global_features = Slic3r::GlobalFeatureFilaments::from_full_config(config);
#endif
    const auto volume_filaments = [&](const Slic3r::ModelVolume* vol) {
#ifdef ENGINE_ORCA
        return vol->get_extruders();
#else
        return vol->get_extruders(global_features);
#endif
    };
    for (const Slic3r::ModelObject* object : model.objects) {
        std::set<int> object_filaments;
        for (const Slic3r::ModelVolume* vol : object->volumes) {
            const std::vector<int> filaments = volume_filaments(vol);
            object_filaments.insert(filaments.begin(), filaments.end());
        }
        if (object_filaments.size() != 1)
            continue;   // only a single-colour object is checked
        for (const Slic3r::ModelInstance* inst : object->instances) {
            if (inst->print_volume_state != Slic3r::ModelInstancePVS_Inside)
                continue;
            const Slic3r::Transform3d& inst_matrix = inst->get_transformation().get_matrix();
            for (const Slic3r::ModelVolume* vol : object->volumes) {
                if (!vol->is_model_part())
                    continue;
                const Slic3r::BoundingBoxf3 bbox = vol->get_convex_hull().transformed_bounding_box(inst_matrix * vol->get_matrix());
                std::vector<bool> inside_extruders;
                if (build_volume.check_volume_bbox_state_with_extruder_areas(bbox, inside_extruders) !=
                    Slic3r::BuildVolume::ObjectState::Limited)
                    continue;
                const std::vector<int> filaments = volume_filaments(vol);
                for (size_t j = 0; j < inside_extruders.size() && j < unprintable.size(); ++j)
                    if (!inside_extruders[j])
                        unprintable[j].insert(filaments.begin(), filaments.end());
            }
        }
    }
    Slic3r::FilamentMapMode mode = Slic3r::FilamentMapMode::fmmAutoForFlush;
    if (const auto* mode_opt = config.option<Slic3r::ConfigOptionEnum<Slic3r::FilamentMapMode>>("filament_map_mode"))
        mode = mode_opt->value;
    bool manual = mode == Slic3r::FilamentMapMode::fmmManual;
#ifndef ENGINE_ORCA
    manual = manual || mode == Slic3r::FilamentMapMode::fmmNozzleManual;
#endif
    int blocked_extruder = 0;
    if (manual) {
        if (const auto* maps = config.option<Slic3r::ConfigOptionInts>("filament_map"))
            for (size_t index = 0; index < maps->values.size() && blocked == 0; ++index) {
                const int extruder = maps->values[index];
                if (extruder >= 1 && extruder <= (int)unprintable.size() && unprintable[extruder - 1].count(int(index) + 1)) {
                    blocked = int(index) + 1;
                    blocked_extruder = extruder;
                }
            }
    } else {
        std::set<int> everywhere = unprintable.front();
        for (size_t j = 1; j < unprintable.size(); ++j) {
            std::set<int> both;
            std::set_intersection(everywhere.begin(), everywhere.end(), unprintable[j].begin(), unprintable[j].end(),
                                  std::inserter(both, both.begin()));
            everywhere = std::move(both);
        }
        if (!everywhere.empty())
            blocked = *everywhere.begin();
    }
    if (blocked == 0)
        return {};
    return manual
            ? "Filament " + std::to_string(blocked) + " prints an object placed where extruder " + std::to_string(blocked_extruder) +
              (unprintable.size() == 2 ? (blocked_extruder == 1 ? " (left)" : " (right)") : "") +
              " cannot reach, and the filament map names that extruder for it. Give it the other extruder in "
              "--filament-map, or move the object to where both nozzles reach (--arrange 1)."
            : "Filament " + std::to_string(blocked) + " prints objects placed where no single extruder reaches all of "
              "them. Move the objects to where both nozzles reach (--arrange 1).";
}
#endif // ENGINE_SNAPMAKER

} // namespace slicer_cli
