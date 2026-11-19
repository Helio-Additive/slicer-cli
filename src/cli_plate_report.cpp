#include "cli_plate_report.hpp"
#include "cli_load_settings.hpp"

#include "cli_run_context.hpp"
#include "cli_result_json.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include <boost/filesystem.hpp>
#include <iostream>
#include <set>
#include "cli_plate_arrange.hpp"
#include "cli_plate_arrange.hpp"

namespace slicer_cli {

using json = nlohmann::json;

/// Per-plate figures the official CLI records after export (BambuStudio.cpp
/// 7262-7356 at 5873b5f): print time, filament per slot in grams, and each
/// object with its bounding box. Orca's statistics carry no per-role times
/// and name the change count total_filament_changes.
void record_plate_statistics(const Slic3r::Print& print, const Slic3r::Model& model,
                                    const Slic3r::DynamicPrintConfig& config, const Slic3r::DynamicPrintConfig& project,
                                    const Slic3r::GCodeProcessorResult& result, PlateOutcome& outcome) {
    using Slic3r::PrintEstimatedStatistics;
    const PrintEstimatedStatistics& stat = result.print_statistics;
    const PrintEstimatedStatistics::Mode& mode =
        stat.modes[static_cast<size_t>(PrintEstimatedStatistics::ETimeMode::Normal)];
    outcome.total_predication = mode.time;
    outcome.main_predication  = mode.time - mode.prepare_time;
#ifdef ENGINE_BAMBU
    outcome.filament_change_times = int(stat.total_flush_filament_changes);
    for (const auto& [role, t] : mode.roles_times) {
        if (role == Slic3r::ExtrusionRole::erWipeTower || role == Slic3r::ExtrusionRole::erFlush)
            outcome.main_predication -= t;
        if (t > 0.f)
            outcome.feature_type_times.emplace_back(Slic3r::ExtrusionEntity::role_to_string(role), t);
    }
    for (const auto& [move, t] : mode.moves_times)
        if (move == Slic3r::EMoveType::Travel) {
            if (t > 0.f) outcome.feature_type_times.emplace_back("Travel", t);
            break;
        }
#else
#ifdef ENGINE_SNAPMAKER
    // Snapmaker Orca names the counter total_filamentchanges
    // (references/SnapmakerOrca/src/libslic3r/GCode/GCodeProcessor.hpp at bac6fd5).
    outcome.filament_change_times = int(stat.total_filamentchanges);
#else
    outcome.filament_change_times = int(stat.total_filament_changes);
#endif
#endif
    const auto* filament_ids = dynamic_cast<const Slic3r::ConfigOptionStrings*>(config.option("filament_ids"));
    for (const auto& [extruder, volume] : stat.total_volumes_per_extruder) {
        const double density = extruder < result.filament_densities.size() ? result.filament_densities[extruder] : 1.;
        double main_volume = 0.;
        if (auto it = stat.model_volumes_per_extruder.find(extruder); it != stat.model_volumes_per_extruder.end())
            main_volume += it->second;
        if (auto it = stat.support_volumes_per_extruder.find(extruder); it != stat.support_volumes_per_extruder.end())
            main_volume += it->second;
        outcome.filaments.push_back(json{
            {"id", extruder + 1},
            {"filament_id", filament_ids && extruder < filament_ids->values.size() ? filament_ids->values[extruder] : std::string("unknown")},
            {"total_used_g", 0.001 * volume * density},
            {"main_used_g",  0.001 * main_volume * density}});
    }
#ifndef ENGINE_SNAPMAKER
    // A printer with more than one nozzle: the nozzle each filament printed
    // with (1 = left, 2 = right), the map the export writes to the plate's
    // slice_info filament_maps (record_plate_for_export), whether the
    // command line chose it (--filament-map, Manual) or the engine grouped
    // the filaments itself (the automatic modes write their grouping back to
    // the plate, BambuStudio.cpp 7660-7662 at v02.09.01.70). The project's
    // own filament_map is not this: an automatic mode leaves it as saved.
    // Snapmaker Orca's Print has no filament map at bac6fd5.
    if (const auto* nozzles = dynamic_cast<const Slic3r::ConfigOptionVectorBase*>(config.option("nozzle_diameter"));
        nozzles && nozzles->size() > 1) {
        outcome.filament_maps = print.get_filament_maps();
        if (const auto* mode = config.option("filament_map_mode"))
            outcome.filament_map_mode = mode->serialize();
        for (json& f : outcome.filaments) {
            const int id = f.value("id", 0);
            if (id >= 1 && id <= (int)outcome.filament_maps.size())
                f["extruder"] = outcome.filament_maps[id - 1];
        }
    }
#endif
#ifdef ENGINE_BAMBU
    // A nozzle rack (the H2C's right extruder, the H3C): the rack nozzle each
    // filament printed with, the map the grouping wrote back
    // (Print::get_filament_nozzle_maps; the G-code's filament_nozzle_map).
    if (const auto* counts = config.option<Slic3r::ConfigOptionIntsNullable>("extruder_max_nozzle_count");
        counts && std::any_of(counts->values.begin(), counts->values.end(), [](int v) { return v > 1; })) {
        outcome.filament_nozzle_map = print.get_filament_nozzle_maps();
        if (const auto* mode = config.option("filament_map_mode"))
            outcome.filament_map_mode = mode->serialize();
    }
#endif
    for (const Slic3r::ModelObject* object : model.objects) {
        bool printed = false;
        for (const Slic3r::ModelInstance* inst : object->instances)
            printed = printed || inst->print_volume_state != Slic3r::ModelInstancePVS_Fully_Outside;
        if (!printed) continue;
        const Slic3r::BoundingBoxf3 box = object_world_bbox(object);
        const size_t triangles = object->facets_count();
        outcome.triangle_count += triangles;
        outcome.objects.push_back(json{
            {"id", object->id().id},
            {"name", object->name},
            {"triangle_count", triangles},
            {"bbox", {{"x", box.min.x()}, {"y", box.min.y()}, {"z", box.min.z()},
                      {"width", box.size().x()}, {"depth", box.size().y()}, {"height", box.size().z()}}}});
    }
    // The run's summary reads the project's settings, m_print_config, not
    // the plate's copy (BambuStudio.cpp 6905-6910; OrcaSlicer.cpp 5908-5913).
    if (project.has("layer_height"))          outcome.layer_height = float(project.opt_float("layer_height"));
    if (project.has("wall_loops"))            outcome.wall_loops   = project.opt_int("wall_loops");
    if (const auto* d = project.option<Slic3r::ConfigOptionPercent>("sparse_infill_density"))
        outcome.sparse_infill_density = float(d->value);
    (void)print;
}


/// The slice result of one plate in the shape the GUI stores it into a
/// project (PartPlateList::store_to_3mf_structure, slic3r/GUI/PartPlate.cpp
/// at 5873b5f): G-code file, predicted time, weight, filaments used per slot
/// (PlateData::parse_filament_info), and the processor's per-plate facts.
void record_plate_for_export(CliRunContext& context, const Slic3r::Print& print, const Slic3r::Model& model,
                                    const Slic3r::DynamicPrintConfig& config,
                                    Slic3r::GCodeProcessorResult& result, const std::string& gcode_file,
                                    PlateOutcome& outcome, const Slic3r::Vec2d& plate_origin) {
    auto pd = std::make_shared<Slic3r::PlateData>();
    pd->plate_index      = std::max(0, outcome.plate_id - 1);
    pd->gcode_file       = gcode_file;
    pd->is_sliced_valid  = true;
    pd->gcode_prediction = std::to_string(int(result.print_statistics
        .modes[static_cast<size_t>(Slic3r::PrintEstimatedStatistics::ETimeMode::Normal)].time));
    pd->toolpath_outside         = result.toolpath_outside;
    pd->timelapse_warning_code   = result.timelapse_warning_code;
    pd->is_label_object_enabled  = result.label_object_enabled;
#ifndef ENGINE_SNAPMAKER
    // OrcaSlicer 2.4 writes these per-plate filament-map facts into the 3MF;
    // Snapmaker's PlateData has no such fields at bac6fd5, so its plates carry
    // none of them.
    pd->limit_filament_maps      = result.limit_filament_maps;
    pd->layer_filaments          = result.layer_filaments;
    pd->filament_change_sequence = result.filament_change_sequence;
    pd->nozzle_change_sequence   = result.nozzle_change_sequence;
    pd->optimal_assignment       = result.optimal_assignment;
    pd->filament_maps            = print.get_filament_maps();
#endif
    const double weight = print.print_statistics().total_weight;
    if (weight != 0.) {
        char text[32];
        std::snprintf(text, sizeof text, "%.2f", weight);
        pd->gcode_weight = text;
    }
    pd->is_support_used = print.is_support_used();
    pd->parse_filament_info(&result);
    // What the official export adds per plate (BambuStudio.cpp 7514-7532;
    // OrcaSlicer.cpp 6323-6343): the skipped objects, the printer model id,
    // the nozzle diameters, and each used filament's type, colour and id.
    pd->skipped_objects.assign(outcome.skipped_objects.begin(), outcome.skipped_objects.end());
    if (outcome.settings_merge && !outcome.settings_merge->printer_model_id.empty())
        pd->printer_model_id = outcome.settings_merge->printer_model_id;
    else if (!context.g_preset_printer_model_id.empty())
        pd->printer_model_id = context.g_preset_printer_model_id;
    if (const auto* nozzles = config.option("nozzle_diameter"))
        pd->nozzle_diameters = nozzles->serialize();
    {
        Slic3r::DynamicPrintConfig types = config;
        const auto* colours        = config.option<Slic3r::ConfigOptionStrings>("filament_colour");
        const auto* ids            = config.option<Slic3r::ConfigOptionStrings>("filament_ids");
        const auto* filament_types = config.option<Slic3r::ConfigOptionStrings>("filament_type");
        // An entry is read at the filament's index only while the list HAS one.
        // filament_ids' own default is an empty list (PrintConfig.cpp 2851-2852
        // at 5873b5f, `new ConfigOptionStrings()`), and BambuStudio saves every
        // project without it — it is a nocli key — so a project that states no
        // ids carries the option with no entry at all, and get_at() then reads
        // values.front() of an empty vector (Config.hpp 681-684), a null
        // dereference. The desktop never meets that shape because its
        // PresetBundle fans every per-filament array out to the filament count
        // (PresetBundle::full_fff_config), so filament_ids always has one entry
        // per filament there. Take upstream's own fallbacks for the empty case:
        // the empty tray id, "#FFFFFF", and get_filament_type's own empty return
        // for a project without a filament list of its own.
        for (auto& info : pd->slice_filaments_info) {
            std::string display_type;
            info.type        = filament_types && !filament_types->values.empty()
                                   ? types.get_filament_type(display_type, info.id) : std::string();
            info.color       = colours && !colours->values.empty() ? colours->get_at(info.id) : "#FFFFFF";
            info.filament_id = ids && !ids->values.empty() ? ids->get_at(info.id) : "";
        }
    }
#ifdef ENGINE_BAMBU
    // --skip-useless-pick: a plate of one object needs no object labels
    // (BambuStudio.cpp 7486-7498).
    if (context.g_skip_useless_pick) {
        int printable = 0;
        for (const Slic3r::ModelObject* object : model.objects)
            for (const Slic3r::ModelInstance* inst : object->instances)
                printable += inst->printable && inst->print_volume_state == Slic3r::ModelInstancePVS_Inside ? 1 : 0;
        if (printable == 1)
            pd->is_label_object_enabled = false;
    }
#endif
    outcome.plate_data = pd;
    // The plate's first-layer boxes, Metadata/plate_N.json (BambuStudio.cpp
    // 8021-8125; OrcaSlicer.cpp 6833-6935), on the plate: the objects' boxes
    // come without the plate offset (PrintObject::get_first_layer_bbox,
    // shift_without_plate_offset), and the tower's corners take the plate's
    // origin off (BambuStudio.cpp 8107-8108; OrcaSlicer.cpp 6938-6939).
    {
        auto bbox = std::make_shared<Slic3r::PlateBBoxData>();
        const auto* seq = config.option<Slic3r::ConfigOptionEnum<Slic3r::PrintSequence>>("print_sequence");
        bbox->is_seq_print = seq && seq->value == Slic3r::PrintSequence::ByObject;
        if (const auto* bed = config.option<Slic3r::ConfigOptionEnum<Slic3r::BedType>>("curr_bed_type"))
            bbox->bed_type = Slic3r::bed_type_to_gcode_string(bed->value);
        bbox->first_extruder = print.get_tool_ordering().first_extruder();
#ifdef ENGINE_BAMBU
        if (const auto* nozzles = dynamic_cast<const Slic3r::ConfigOptionFloatsNullable*>(config.option("nozzle_diameter")))
#else
        if (const auto* nozzles = dynamic_cast<const Slic3r::ConfigOptionFloats*>(config.option("nozzle_diameter")))
#endif
            bbox->nozzle_diameter = float(nozzles->get_at(bbox->first_extruder));
        Slic3r::BoundingBoxf bbox_all;
        for (Slic3r::PrintObject* obj : const_cast<Slic3r::Print&>(print).objects()) {
            Slic3r::BBoxData data;
            const auto bb = Slic3r::unscaled(obj->get_first_layer_bbox(data.area, data.layer_height, data.name));
            bbox_all.merge(bb);
            data.area *= float(SCALING_FACTOR * SCALING_FACTOR);
            data.id   = int(obj->id().id);
            data.bbox = {bb.min.x(), bb.min.y(), bb.max.x(), bb.max.y()};
            bbox->bbox_objs.emplace_back(std::move(data));
        }
        if (print.has_wipe_tower()) {
            const auto corners = print.first_layer_wipe_tower_corners();
            if (!corners.empty()) {
                auto bb = Slic3r::unscaled(Slic3r::BoundingBox(corners[0], corners[2]));
                bb.min -= plate_origin;
                bb.max -= plate_origin;
                bbox_all.merge(bb);
                Slic3r::BBoxData data;
                data.name = "wipe_tower";
                data.id   = std::max(0, outcome.plate_id - 1) + 1000;
                data.bbox = {bb.min.x(), bb.min.y(), bb.max.x(), bb.max.y()};
                bbox->bbox_objs.emplace_back(std::move(data));
            }
        }
        bbox->bbox_all = {bbox_all.min.x(), bbox_all.min.y(), bbox_all.max.x(), bbox_all.max.y()};
        for (const auto& info : pd->slice_filaments_info) {
            bbox->filament_ids.push_back(info.id);
            bbox->filament_colors.push_back(info.color);
        }
        outcome.plate_bbox = bbox;
    }
}

/// The objects a G-code check names: each instance's label id is the one the
/// G-code carries (ModelInstance::get_labeled_id, Model.hpp in both engines);
/// -1 is what the processor records for moves outside any object.
static std::string label_names(const Slic3r::Model& model, const std::set<int>& labels) {
    std::string out;
    for (int label : labels) {
        std::string name;
        if (label < 0) {
            name = "the prime tower, skirt or brim";
        } else {
            for (const Slic3r::ModelObject* object : model.objects)
                for (const Slic3r::ModelInstance* inst : object->instances)
                    if (int(inst->get_labeled_id()) == label) name = "'" + object->name + "'";
            if (name.empty()) name = "object " + std::to_string(label);
        }
        if (out.find(name) != std::string::npos) continue;
        out += (out.empty() ? "" : ", ") + name;
    }
    return out;
}

/// The official checks after export (BambuStudio.cpp 7024-7047, 7187-7243,
/// 7297-7326 at 5873b5f): toolpath conflicts, the critical slicing warnings
/// the official CLI refuses, the G-code check bits and unprintable filaments.
/// Each refusal names what it found. Orca's CLI maps every G-code check bit to
/// CLI_GCODE_PATH_IN_UNPRINTABLE_AREA (OrcaSlicer.cpp 6145-6155); both builds
/// use the finer Bambu mapping here.
bool post_slice_checks(const Slic3r::Print& print, const Slic3r::Model& model,
                              const Slic3r::GCodeProcessorResult& result, PlateOutcome& outcome) {
    (void)print;
    if (result.conflict_result.has_value()) {
        const auto& c = *result.conflict_result;
        set_outcome_failure(outcome, CLI_GCODE_PATH_CONFLICTS,
            "Toolpaths of '" + c._objName1 + "' and '" + c._objName2 + "' meet at height " +
            mm_text(c._height) + " mm.");
        return false;
    }
    // With --no-check the official CLI does not stop on these warnings
    // (BambuStudio.cpp 7132-7142; OrcaSlicer.cpp 6113-6123).
    for (const json& w : outcome.no_check ? json::array() : outcome.warnings) {
        const std::string tag = w.value("tag", "");
        if (w.value("level", "") == "critical" &&
            (tag == "SlicingEmptyGcodeLayers" || tag == "SlicingGcodeOverlap")) {
            set_outcome_failure(outcome, CLI_SLICING_ERROR, w.value("message", ""));
            return false;
        }
    }
#ifndef ENGINE_SNAPMAKER
    // The post-slice G-code check (gcode_check_result) and the unprintable
    // filament result are OrcaSlicer 2.4 only; the Snapmaker fork's
    // GCodeProcessorResult carries neither at bac6fd5, so its CLI has nothing
    // to check here (and no CLI_GCODE_PATH_IN_UNPRINTABLE_AREA code either).
    const int code = result.gcode_check_result.error_code;
    if (code != 0 && code != (1 << 11)) {
        std::set<int> labels;
        for (const auto& [extruder, entries] : result.gcode_check_result.print_area_error_infos)
            for (const auto& [filament, label] : entries) labels.insert(label);
        for (const auto& [extruder, entries] : result.gcode_check_result.print_height_error_infos)
            for (const auto& [filament, label] : entries) labels.insert(label);
        std::string where = label_names(model, labels);
        if (code & 0b1100) {
            std::string detail = where.empty()
                ? "The objects themselves are inside the bed, so the cause is the prime tower, a skirt, a brim or support."
                : "It comes from " + where + ".";
            // A negative label is the prime tower, skirt or brim (label_names):
            // say what moves it, after the existing text.
            if (!labels.empty() && *labels.begin() < 0)
                detail += " Move the prime tower with --wipe-tower-x and --wipe-tower-y, or turn it off with --enable-prime-tower=0.";
            set_outcome_failure(outcome, CLI_GCODE_PATH_OUTSIDE, detail);
        } else if (code & 0b10000) {
            set_outcome_failure(outcome, CLI_GCODE_IN_WRAPPING_DETECT_AREA,
                                where.empty() ? std::string() : "It comes from " + where + ".");
        } else {
            set_outcome_failure(outcome, CLI_GCODE_PATH_IN_UNPRINTABLE_AREA,
                                where.empty() ? std::string() : "It comes from " + where + ".");
        }
        return false;
    }
    if (!result.filament_printable_reuslt.conflict_filament.empty()) {
        std::string slots;
        for (int f : result.filament_printable_reuslt.conflict_filament)
            slots += (slots.empty() ? "" : ", ") + std::to_string(f + 1);
        set_outcome_failure(outcome, CLI_FILAMENT_UNPRINTABLE_ON_FIRST_LAYER,
                            "Filament slot(s) " + slots + " cannot print on the " +
                            result.filament_printable_reuslt.plate_name + " plate.");
        return false;
    }
#endif // ENGINE_SNAPMAKER
    return true;
}

} // namespace slicer_cli
