#include "cli_plate_arrange.hpp"

#include "cli_run_context.hpp"
#include "cli_result_json.hpp"
#include "cli_options.hpp"
#include "cli_assemble.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/GCode/WipeTower.hpp"
#include <boost/algorithm/string.hpp>
#include <iostream>
#include "cli_arrange_geometry.hpp"
#include "cli_engine_events.hpp"
#include "cli_filament_map.hpp"
#include "cli_load_settings.hpp"
#include "cli_percent_widths.hpp"
#include "cli_snapmaker.hpp"

namespace slicer_cli {

using json = nlohmann::json;

/// Formats a length in mm the way a person reads it: no trailing zeros.
std::string mm_text(double v) {
    std::ostringstream s;
    s.setf(std::ios::fixed);
    s.precision(1);
    s << v;
    std::string t = s.str();
    if (t.size() > 2 && t.compare(t.size() - 2, 2, ".0") == 0) t.resize(t.size() - 2);
    return t;
}

/// The bed as a person reads it: "180 x 180 x 180 mm".
static std::string bed_size_text(const Slic3r::DynamicPrintConfig& config) {
    const auto* area = config.option<Slic3r::ConfigOptionPoints>("printable_area");
    if (!area || area->values.empty()) return "unknown";
    Slic3r::BoundingBoxf bed;
    for (const Slic3r::Vec2d& p : area->values) bed.merge(p);
    std::string text = mm_text(bed.size().x()) + " x " + mm_text(bed.size().y());
    if (config.has("printable_height"))
        text += " x " + mm_text(config.opt_float("printable_height"));
    return text + " mm";
}

Slic3r::BoundingBoxf3 object_world_bbox(const Slic3r::ModelObject* object) {
    Slic3r::BoundingBoxf3 box;
    for (size_t i = 0; i < object->instances.size(); ++i)
        box.merge(object->instance_bounding_box(i));
    return box;
}

static std::string object_size_text(const Slic3r::BoundingBoxf3& box) {
    const Slic3r::Vec3d size = box.size();
    return mm_text(size.x()) + " x " + mm_text(size.y()) + " x " + mm_text(size.z()) + " mm";
}

/// The official per-plate gate before apply (BambuStudio.cpp 6527-6567 at
/// 5873b5f; OrcaSlicer.cpp 5645-5697 at 31f6803): Model::update_print_volume_state
/// against the printer's bed, then refuse an object partly over the edge
/// (CLI_OBJECTS_PARTLY_INSIDE) and a plate with nothing fully inside
/// (CLI_NO_SUITABLE_OBJECTS). The sentence names the object and its size
/// against the bed, which the official result leaves out. `arranged` is the
/// run's own --arrange 1: it has already re-arranged this plate, so the
/// refusal cannot offer that as the way out.
bool check_objects_inside_bed(Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                     PlateOutcome& outcome, bool arranged) {
    const auto* area = config.option<Slic3r::ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 3)
        return true;   // no bed to check against
    double height = config.has("printable_height") ? config.opt_float("printable_height") : 0.;
    if (height <= 0.) height = 1e6;
    // Per-extruder areas only when the printer states them completely; a
    // padded or partial table would shrink the bed the engine itself uses.
    std::vector<std::vector<Slic3r::Vec2d>> extruder_areas;
    std::vector<double> extruder_heights;
#ifndef ENGINE_SNAPMAKER
    const auto* groups  = config.option<Slic3r::ConfigOptionPointsGroups>("extruder_printable_area");
    const auto* heights = config.option<Slic3r::ConfigOptionFloatsNullable>("extruder_printable_height");
    if (groups && heights && groups->values.size() > 1 && groups->values.size() == heights->values.size()) {
        bool complete = true;
        for (size_t i = 0; i < groups->values.size(); ++i)
            complete = complete && groups->values[i].size() >= 3 && heights->values[i] > 0.;
        if (complete) {
            extruder_areas = groups->values;
            extruder_heights.assign(heights->values.begin(), heights->values.end());
        }
    }
    Slic3r::BuildVolume build_volume(area->values, height, extruder_areas, extruder_heights);
#else
    // Snapmaker Orca has neither ConfigOptionPointsGroups nor the per-extruder
    // BuildVolume constructor (references/SnapmakerOrca/src/libslic3r/BuildVolume.hpp:36
    // at bac6fd5): its bed is the single printable area, as its own CLI uses.
    Slic3r::BuildVolume build_volume(area->values, height);
#endif
    const unsigned int inside = model.update_print_volume_state(build_volume);

    for (const Slic3r::ModelObject* object : model.objects) {
        for (size_t i = 0; i < object->instances.size(); ++i) {
            if (object->instances[i]->print_volume_state != Slic3r::ModelInstancePVS_Partly_Outside)
                continue;
            const std::string detail = "Object '" + object->name + "' (" +
                object_size_text(object->instance_bounding_box(i)) +
                ") crosses the edge of the " + bed_size_text(config) + " bed."
                // The verdict says what is wrong; this says what to do with it.
                // --arrange 1 re-arranges the plate on this bed (--arrange N,
                // official meaning, BambuStudio.cpp 5066-5081 at 5873b5f /
                // OrcaSlicer.cpp 4327-4342 at 31f6803, run before this check,
                // BambuStudio.cpp 5936-5945 / OrcaSlicer.cpp 5196-5205), and
                // the object as it stands fits a bigger printer's bed. A run
                // that was given --arrange 1 has taken that advice already, so
                // it gets the way out that is left.
                + (arranged
                       ? std::string(" Use a printer with a bigger bed.")
                       : std::string(" Give --arrange 1 to re-arrange the plate on this bed, or use a printer with a bigger bed."));
            set_outcome_failure(outcome, CLI_OBJECTS_PARTLY_INSIDE, detail);
            emit_event({{"event","plate_error"}, {"tag","ObjectPartlyOutsideBed"},
                        {"object", object->name}, {"message", detail}});
            return false;
        }
    }
    if (inside == 0) {
        std::string detail;
        for (const Slic3r::ModelObject* object : model.objects) {
            const Slic3r::BoundingBoxf3 box = object_world_bbox(object);
            detail += (detail.empty() ? "" : " ") + std::string("Object '") + object->name + "' is " +
                      object_size_text(box) + ";";
        }
        detail += " the bed is " + bed_size_text(config) + ".";
        set_outcome_failure(outcome, CLI_NO_SUITABLE_OBJECTS, detail);
        emit_event({{"event","plate_error"}, {"tag","NoObjectInsideBed"}, {"message", detail}});
        return false;
    }
#ifndef ENGINE_SNAPMAKER
    // The official reach check before the slice (cli_filament_map.cpp).
    if (extruder_areas.size() > 1) {
        int blocked = 0;
        const std::string detail = slicer_cli::extruder_reach_refusal(model, config, build_volume, extruder_areas.size(), blocked);
        if (!detail.empty()) {
            set_outcome_failure(outcome, CLI_FILAMENT_CAN_NOT_MAP, detail);
            emit_event({{"event","plate_error"}, {"tag","FilamentCannotMapToExtruder"}, {"filament", blocked},
                        {"message", detail}});
            return false;
        }
    }
#endif
    return true;
}



/// The box of the plate's objects with its prime tower, as the lambda
/// check_plate_wipe_tower measures it (BambuStudio.cpp 4418-4495; OrcaSlicer.cpp
/// 3819-3884), for --downward-check and the move to a new bed. The model holds
/// only this plate's objects, at the plate's own origin.
static Slic3r::BoundingBoxf3 downward_plate_bbox(CliRunContext& context, const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                                 int plate_index, bool is_sequence, bool* has_wipe_tower = nullptr) {
    using namespace Slic3r;
    if (has_wipe_tower)
        *has_wipe_tower = false;
    BoundingBoxf3 obj_bbox;
    for (const ModelObject* object : model.objects)
        for (size_t i = 0; i < object->instances.size(); ++i)
            obj_bbox.merge(object->instance_bounding_box(i));
    if (!config.has("wipe_tower_x"))
        return obj_bbox;
    const auto* enable_tower = config.option<ConfigOptionBool>("enable_prime_tower");
    if (!enable_tower || !enable_tower->value)
        return obj_bbox;
    int valid_count = 0;
    for (const ModelObject* object : model.objects)
        for (const ModelInstance* inst : object->instances)
            valid_count += inst->printable ? 1 : 0;
    if (valid_count <= 0 || (is_sequence && valid_count > 1))
        return obj_bbox;
    const auto* timelapse_type_opt = config.option("timelapse_type");
    const bool is_smooth_timelapse = context.g_arrange_switches.enable_timelapse && timelapse_type_opt &&
                                     timelapse_type_opt->getInt() == TimelapseType::tlSmooth;
    const unsigned int filaments_cnt = (unsigned int)arrange_plate_extruders(model, config, plate_index).size();
    if (filaments_cnt <= 1 && !is_smooth_timelapse)
        return obj_bbox;

#ifdef ENGINE_SNAPMAKER
    // Missing or empty tower coordinates, or any missing tower scalar, means
    // no tower rather than a crash: Snapmaker's check_plate_wipe_tower returns
    // with has_wipe_tower false (references/SnapmakerOrca/src/Snapmaker_Orca.cpp:3186-3220
    // at bac6fd5).
    if (!slicer_cli::snapmaker_tower_settings_present(config))
        return obj_bbox;
#endif

    const float wipe_x = (float)dynamic_cast<const ConfigOptionFloats*>(config.option("wipe_tower_x"))->get_at(plate_index);
    const float wipe_y = (float)dynamic_cast<const ConfigOptionFloats*>(arrange_opt(config, "wipe_tower_y"))->get_at(plate_index);
    const double width = arrange_opt(config, "prime_tower_width")->getFloat();
    float brim_width = (float)arrange_opt(config, "prime_tower_brim_width")->getFloat();
#ifdef ENGINE_SNAPMAKER
    // No WipeTower::get_auto_brim_by_height at bac6fd5: this engine has no
    // automatic prime-tower brim, so a negative (automatic) width adds none.
    if (brim_width < 0) brim_width = 0.f;
#else
    if (brim_width < 0)
        brim_width = WipeTower::get_auto_brim_by_height((float)obj_bbox.max.z());
#endif
#ifdef ENGINE_ORCA
    const double wipe_volume = arrange_opt(config, "prime_volume")->getFloat();
#else
    const std::vector<double> volumes = dynamic_cast<const ConfigOptionFloats*>(arrange_opt(config, "filament_prime_volume"))->values;
    const double wipe_volume = volumes.empty() ? 0. : *std::max_element(volumes.begin(), volumes.end());
#endif
    const auto* wrapping_detection = config.option<ConfigOptionBool>("enable_wrapping_detection");
    const bool enable_wrapping = wrapping_detection != nullptr && wrapping_detection->value;
    int extruder_count = 1;
    if (const auto* nozzles = dynamic_cast<const ConfigOptionVectorBase*>(config.option("nozzle_diameter")))
        extruder_count = std::max(1, (int)nozzles->size());
    BoundingBoxf bed;
    if (const auto* area = config.option<ConfigOptionPoints>("printable_area"))
        for (const Vec2d& pt : area->values) bed.merge(pt);
    const double bed_height = config.has("printable_height") ? config.opt_float("printable_height") : 0.;
    const BoundingBoxf3 plate_box(Vec3d(bed.min.x(), bed.min.y(), 0.), Vec3d(bed.max.x(), bed.max.y(), bed_height));
    const Vec3d tower = arrange_estimate_wipe_tower_size(model, config, width, wipe_volume, extruder_count, (int)filaments_cnt,
                                                         enable_wrapping, plate_box, arrange_exclude_boxes(config));
#ifdef ENGINE_ORCA
    // OrcaSlicer.cpp 3876-3879: the brim on the sides only.
    obj_bbox.merge(Vec3d(wipe_x - brim_width, wipe_y, 0.f));
    obj_bbox.merge(Vec3d(wipe_x + tower(0) + brim_width, wipe_y + tower(1), 0.f));
#else
    obj_bbox.merge(Vec3d(wipe_x - brim_width, wipe_y - brim_width, 0.f));
    obj_bbox.merge(Vec3d(wipe_x + tower(0) + brim_width, wipe_y + tower(1) + brim_width, 0.f));
#endif
    if (has_wipe_tower)
        *has_wipe_tower = true;
    return obj_bbox;
}

Slic3r::Vec3d downward_plate_size(CliRunContext& context, const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                         int plate_index, bool is_sequence) {
    return downward_plate_bbox(context, model, config, plate_index, is_sequence).size();
}

/// shrink_to_new_bed alone (BambuStudio.cpp 4280-4310; OrcaSlicer.cpp
/// 3693-3715): 0 the same bed (or no bed known), 1 a larger one, 2 a smaller
/// one or another exclusion area.
static int new_bed_shrink(const slicer_cli::ProjectFacts& facts, const Slic3r::DynamicPrintConfig& config) {
    using namespace Slic3r;
    if (!facts.is_bbl_3mf)
        return 0;
    const auto* area = config.option<ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 4)
        return 0;
    const int current_width  = (int)(area->values[2].x() - area->values[0].x());
    const int current_depth  = (int)(area->values[2].y() - area->values[0].y());
    const int current_height = config.has("printable_height") ? (int)config.opt_float("printable_height") : 0;
    const int old_width  = facts.old_printable_width > 0 ? facts.old_printable_width : current_width;
    const int old_depth  = facts.old_printable_depth > 0 ? facts.old_printable_depth : current_depth;
    const int old_height = facts.old_printable_height > 0 ? facts.old_printable_height : current_height;
    std::vector<Vec2d> current_exclude_area;
    if (const auto* exclude = config.option<ConfigOptionPoints>("bed_exclude_area"))
        current_exclude_area = exclude->values;
    int shrink_to_new_bed = 0;
    if (old_width > 0 && old_depth > 0 && old_height > 0) {
        if (old_width > current_width || old_depth > current_depth || old_height > current_height)
            shrink_to_new_bed = 2;
        else if (old_width < current_width || old_depth < current_depth)
            shrink_to_new_bed = 1;
        else if (!current_exclude_area.empty() && current_exclude_area != facts.old_exclude_area)
            shrink_to_new_bed = 2;
    }
    return shrink_to_new_bed;
}

/// A project moved onto a printer with another bed: translate_models
/// (BambuStudio.cpp 4497-4644, called at 4659; OrcaSlicer.cpp 3885-3980), with
/// shrink_to_new_bed from the file's bed against this printer's
/// (BambuStudio.cpp 4280-4310; OrcaSlicer.cpp 3693-3715). 1: the new bed is
/// larger, so the plate keeps its place around the bed centre; 2: it is smaller
/// (or only its exclusion area differs), so the objects and tower are centred
/// on the bed, or on the shared area of a two-nozzle printer. BambuStudio moves
/// only on a printer change through --load-settings, and on such a change with
/// the same bed it centres a plate whose objects and tower fit the shared area
/// there; OrcaSlicer moves whenever the beds differ. The model holds this
/// plate's objects at the plate's own origin, so each move is the official one
/// with both origins taken out, and the tower moves with them. Returns
/// shrink_to_new_bed; `moved` is the offset applied.
int move_to_new_bed(CliRunContext& context, const slicer_cli::ProjectFacts& facts, bool machine_switch, Slic3r::Model& model,
                           Slic3r::DynamicPrintConfig& config, int plate_index, bool is_sequence, Slic3r::Vec3d& moved) {
    using namespace Slic3r;
    moved = Vec3d::Zero();
    if (!facts.is_bbl_3mf)
        return 0;
    const auto* area = config.option<ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 4)
        return 0;
    const int current_width  = (int)(area->values[2].x() - area->values[0].x());
    const int current_depth  = (int)(area->values[2].y() - area->values[0].y());
    const int old_width  = facts.old_printable_width > 0 ? facts.old_printable_width : current_width;
    const int old_depth  = facts.old_printable_depth > 0 ? facts.old_printable_depth : current_depth;
    std::vector<Vec2d> current_exclude_area;
    if (const auto* exclude = config.option<ConfigOptionPoints>("bed_exclude_area"))
        current_exclude_area = exclude->values;
    const int shrink_to_new_bed = new_bed_shrink(facts, config);
#ifdef ENGINE_ORCA
    (void)machine_switch;
    if (shrink_to_new_bed == 0)
        return 0;
#else
    if (!machine_switch)
        return shrink_to_new_bed;
#endif
    // The area every nozzle reaches (BambuStudio.cpp 4250-4277; OrcaSlicer.cpp
    // 3662-3689): an integer box, as the official's.
    int shared_center_x = 0, shared_center_y = 0, shared_width = 0, shared_depth = 0;
#ifndef ENGINE_SNAPMAKER
    // No ConfigOptionPointsGroups at bac6fd5: a Snapmaker printer states no
    // per-nozzle area, so the shared box is the whole bed (the zeros above).
    {
        const auto* areas   = config.option<ConfigOptionPointsGroups>("extruder_printable_area");
        const auto* heights = config.option<ConfigOptionFloatsNullable>("extruder_printable_height");
        if (areas && heights && !areas->values.empty() && areas->values.size() == heights->values.size()) {
            BoundingBox current_bbox({0, 0}, {current_width, current_depth});
            for (const auto& shape : areas->values) {
                BoundingBox temp_bbox;
                for (const Vec2d& pt : shape)
                    temp_bbox.merge({pt.x(), pt.y()});
                if (current_bbox.min.x() < temp_bbox.min.x()) current_bbox.min.x() = temp_bbox.min.x();
                if (current_bbox.min.y() < temp_bbox.min.y()) current_bbox.min.y() = temp_bbox.min.y();
                if (current_bbox.max.x() > temp_bbox.max.x()) current_bbox.max.x() = temp_bbox.max.x();
                if (current_bbox.max.y() > temp_bbox.max.y()) current_bbox.max.y() = temp_bbox.max.y();
            }
            shared_width    = (int)current_bbox.size().x();
            shared_depth    = (int)current_bbox.size().y();
            shared_center_x = (int)current_bbox.center().x();
            shared_center_y = (int)current_bbox.center().y();
        }
    }
#endif // ENGINE_SNAPMAKER
    double exclude_width = 0., exclude_depth = 0.;
    if (current_exclude_area.size() >= 4) {
        exclude_width = current_exclude_area[2].x() - current_exclude_area[0].x();
        exclude_depth = current_exclude_area[2].y() - current_exclude_area[0].y();
    }
    bool has_wipe_tower = false;
    const BoundingBoxf3 bbox = downward_plate_bbox(context, model, config, plate_index, is_sequence, &has_wipe_tower);
    Vec3d offset = Vec3d::Zero();
    if (shrink_to_new_bed == 1) {
        offset = Vec3d(double(current_width) / 2 - double(old_width) / 2, double(current_depth) / 2 - double(old_depth) / 2, 0.);
    } else if (shrink_to_new_bed == 2) {
        Vec3d new_center_offset((double(current_width) + exclude_width) / 2, (double(current_depth) + exclude_depth) / 2, 0.);
        const Vec3d size = bbox.size();
#ifndef ENGINE_ORCA
        constexpr int DOWNWARD_CHECK_MARGIN = 10;   // BambuStudio.hpp 20
#endif
        if (shared_center_x != 0)
            new_center_offset(0) = (double)shared_center_x;
        else if (exclude_width > 0) {
            if (size.x() > (current_width - exclude_width))
                new_center_offset(0) = double(current_width) / 2;
#ifndef ENGINE_ORCA
            else if (size.x() <= (current_width - exclude_width - DOWNWARD_CHECK_MARGIN))
                new_center_offset(0) = new_center_offset(0) - DOWNWARD_CHECK_MARGIN / 2;
#endif
        }
        if (shared_center_y != 0)
            new_center_offset(1) = (double)shared_center_y;
        else if (exclude_depth > 0) {
            if (size.y() > (current_depth - exclude_depth))
                new_center_offset(1) = double(current_depth) / 2;
#ifndef ENGINE_ORCA
            else if (size.y() <= (current_depth - exclude_depth - DOWNWARD_CHECK_MARGIN))
                new_center_offset(1) = new_center_offset(1) - DOWNWARD_CHECK_MARGIN / 2;
#endif
        }
        offset = new_center_offset - bbox.center();
    }
#ifndef ENGINE_ORCA
    else if (shared_center_x != 0 && shared_width > 0 && shared_depth > 0 && has_wipe_tower) {
        // The same bed: a plate whose objects and tower fit the shared area
        // is centred on it (BambuStudio.cpp 4584-4620).
        const Vec3d bbox_size = bbox.size();
        if (bbox_size.x() <= shared_width && bbox_size.y() <= shared_depth)
            offset = Vec3d(shared_center_x, shared_center_y, 0.) - bbox.center();
    }
#endif
    offset(2) = 0.;
    if (offset.x() == 0. && offset.y() == 0.)
        return shrink_to_new_bed;
    for (ModelObject* object : model.objects)
        for (ModelInstance* inst : object->instances)
            inst->set_offset(inst->get_offset() + offset);
    for (const char* key : {"wipe_tower_x", "wipe_tower_y"}) {
        auto* opt = config.option<ConfigOptionFloats>(key);
        if (!opt || opt->values.empty())
            continue;
        // A short list grows with copies of its first value, as the
        // official set_at does (Config.hpp 437) and as get_at reads it.
        if ((int)opt->values.size() <= plate_index)
            opt->values.resize(plate_index + 1, opt->values.front());
        opt->values[plate_index] += key[11] == 'x' ? offset.x() : offset.y();
    }
    moved = offset;
    return shrink_to_new_bed;
}


/// The plate's bounding box, narrowed by every extruder's own area
/// (PartPlate::get_extruder_areas), at the plate's own origin: where every
/// nozzle reaches.
Slic3r::BoundingBoxf tower_reach_box(const Slic3r::DynamicPrintConfig& config,
                                            const Slic3r::ConfigOptionPoints& area) {
    using namespace Slic3r;
    BoundingBoxf plate_bbox;
    for (const Vec2d& pt : area.values) plate_bbox.merge(pt);
#ifndef ENGINE_SNAPMAKER
    // No ConfigOptionPointsGroups at bac6fd5: the plate box is the printable
    // area itself, with no per-nozzle narrowing.
    if (const auto* extruder_areas = config.option<ConfigOptionPointsGroups>("extruder_printable_area"))
        for (const std::vector<Vec2d>& points : extruder_areas->values) {
            BoundingBoxf boxf;
            for (const Vec2d& pt : points) boxf.merge(pt);
            // The area every extruder reaches: the bounding box of the shared
            // area, as PartPlate::get_build_volume takes it (get_shared_poly,
            // PartPlate.cpp 3711-3733 at v02.09.01.70).
            plate_bbox.min = plate_bbox.min.cwiseMax(boxf.min);
            plate_bbox.max = plate_bbox.max.cwiseMin(boxf.max);
        }
#endif
    return plate_bbox;
}

/// --arrange 1: place this plate's objects on the bed the way the official CLI
/// arranges one plate (`--arrange 1` with a plate to slice; BambuStudio.cpp
/// 5730-5952 at 5873b5f, OrcaSlicer.cpp 4990-5210 at 31f6803):
///   - each printable instance is an item from get_instance_arrange_poly
///     (libslic3r/ModelArrange.cpp 119, both engines), which carries its
///     height, temperatures, name and a brim width from its support settings;
///     unprintable instances are arranged on their own;
///   - the prime tower, when the plate needs one, is a fixed item where the
///     plate's wipe_tower_x/y put it, clamped inside the bed, and the clamped
///     position is written back to the plate's wipe_tower_x/y entry;
///   - the bed exclusion boxes (and the wrapping-detection area when it is on)
///     are fixed items and excluded regions;
///   - the arrange settings are the official ones (min_obj_distance 0 so the
///     items' own inflation applies, print-by-object, I3 alignment, the
///     printer's clearances), then update_arrange_params,
///     update_selected_items_inflation, update_unselected_items_inflation,
///     (Orca) update_selected_items_axis_align and get_shrink_bedpts.
/// An object larger than the bed is refused first, with the official -50
/// sentence plus its size against the bed; objects that do not all land on
/// the bed are refused with the official arrange sentence (-21,
/// BambuStudio.cpp 5938-5944; OrcaSlicer.cpp 5198-5204).
/// `landed_out`, for --repetitions: how many printable items landed on the
/// plate; the arrange is applied even when some did not, and those are
/// left off the plate (not printed), as the official copies loop does.
bool arrange_on_bed(CliRunContext& context, Slic3r::Model& model, Slic3r::DynamicPrintConfig& config,
                           int plate_index, size_t sliced_filament_count, PlateOutcome& outcome,
                           int* landed_out) {
    using namespace Slic3r;
    using namespace Slic3r::arrangement;
    const auto* area = config.option<ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 3) {
        set_outcome_failure(outcome, CLI_OBJECT_ARRANGE_FAILED, "The printer states no bed shape (printable_area).");
        return false;
    }
    BoundingBoxf bed;
    for (const Vec2d& pt : area->values) bed.merge(pt);
    const double bed_height = config.has("printable_height") ? config.opt_float("printable_height") : 0.;
    // Each instance on its own: the arrange places every instance as its own
    // item, so copies that start far apart are not one object wider than the bed.
    // A part counts as larger than the bed only when it fits in neither
    // orientation. Whether one that fits only turned 90 degrees is turned is
    // the arrange's call, and the engines differ: Bambu's arrange offers a
    // too-big item its minimum-area rotation even with rotations off
    // (Arrange.cpp 937-941 at 5873b5f), Orca's only with rotations on
    // (Arrange.cpp 995-1004 at 31f6803), and both turn a long part onto Y on
    // I3 printers. When the arrange leaves it off the bed, -21 below says why.
    for (ModelObject* object : model.objects) {
        object->ensure_on_bed();
        for (size_t i = 0; i < object->instances.size(); ++i) {
            const BoundingBoxf3 box = object->instance_bounding_box(i);
            const Vec3d size = box.size();
            const bool fits_as_is = size.x() <= bed.size().x() + EPSILON && size.y() <= bed.size().y() + EPSILON;
            const bool fits_turned = size.y() <= bed.size().x() + EPSILON && size.x() <= bed.size().y() + EPSILON;
            const bool too_wide = !fits_as_is && !fits_turned;
            const bool too_tall = bed_height > 0. && size.z() > bed_height + EPSILON;
            if (too_wide || too_tall) {
                const std::string detail = "Object '" + object->name + "' is " + object_size_text(box) +
                                           "; the bed is " + bed_size_text(config) +
                                           ". Scale the object down, split it across plates, or use a bigger printer.";
                set_outcome_failure(outcome, CLI_NO_SUITABLE_OBJECTS, detail);
                emit_event({{"event","plate_error"}, {"tag","ObjectLargerThanBed"},
                            {"object", object->name}, {"message", detail}});
                return false;
            }
        }
    }

    // A fresh set of arrange settings (BambuStudio.cpp 5579; OrcaSlicer.cpp
    // 4839), with the progress callback quiet: its default prints to stdout.
    ArrangeParams params;
    params.progressind = [](unsigned, std::string) {};
    // Print by object, from the plate's own sequence or else the project's
    // (get_print_sequence, BambuStudio.cpp 4403-4416, called at 5624;
    // OrcaSlicer.cpp 4883). The caller has already laid the plate's own
    // print_sequence (plate_N.json is_seq_print, the plate's settings) over
    // the config.
    if (const auto* seq = config.option<ConfigOptionEnum<PrintSequence>>("print_sequence"))
        params.is_seq_print = seq->value == PrintSequence::ByObject;

    // Step 1: the items (BambuStudio.cpp 5734-5767; OrcaSlicer.cpp 4994-5027).
    ArrangePolygons selected, unselected, unprintable;
    ModelInstancePtrs selected_instances, unprintable_instances;
    for (ModelObject* mo : model.objects) {
        for (ModelInstance* minst : mo->instances) {
            ArrangePolygon ap = get_instance_arrange_poly(minst, config);
            ArrangePolygons& cont = minst->printable ? selected : unprintable;
            ModelInstancePtrs& owners = minst->printable ? selected_instances : unprintable_instances;
            ap.itemid = cont.size();
            cont.emplace_back(std::move(ap));
            owners.emplace_back(minst);
        }
    }

    const BoundingBoxf3 plate_box(Vec3d(bed.min.x(), bed.min.y(), 0.), Vec3d(bed.max.x(), bed.max.y(), bed_height));
    const std::vector<BoundingBoxf> exclude_boxes = arrange_exclude_boxes(config);
    const auto* wrapping_opt = config.option<ConfigOptionBool>("enable_wrapping_detection");
    const bool enable_wrapping_detect = wrapping_opt != nullptr && wrapping_opt->value;
    const auto* wrapping_area = config.option<ConfigOptionPoints>("wrapping_exclude_area");
    const bool wrapping_area_empty = wrapping_area == nullptr || wrapping_area->values.empty();
    // Smooth timelapse: --enable-timelapse with a smooth timelapse_type
    // (BambuStudio.cpp 4183-4187; OrcaSlicer.cpp 3596-3599).
    const auto* timelapse_type_opt = config.option("timelapse_type");
    const bool is_smooth_timelapse = context.g_arrange_switches.enable_timelapse && timelapse_type_opt &&
                                     timelapse_type_opt->getInt() == TimelapseType::tlSmooth;

    // The prime tower as a fixed item (BambuStudio.cpp 5770-5878; OrcaSlicer.cpp
    // 5030-5134). No duplicate copies in slicer-cli, so the tower starts where
    // the plate's wipe_tower_x/y put it ("keep the original").
    if (config.has("wipe_tower_x") && (is_smooth_timelapse || !params.is_seq_print || (selected.size() <= 1))) {
        float x = dynamic_cast<const ConfigOptionFloats*>(config.option("wipe_tower_x"))->get_at(plate_index);
        float y = dynamic_cast<const ConfigOptionFloats*>(arrange_opt(config, "wipe_tower_y"))->get_at(plate_index);
        float w = arrange_opt(config, "prime_tower_width")->getFloat();
        float a = arrange_opt(config, "wipe_tower_rotation_angle")->getFloat();
#ifdef ENGINE_ORCA
        float v = arrange_opt(config, "prime_volume")->getFloat();
#else
        std::vector<double> volumes = dynamic_cast<const ConfigOptionFloats*>(arrange_opt(config, "filament_prime_volume"))->values;
        if (config.option<ConfigOptionEnum<PrimeVolumeMode>>("prime_volume_mode", true)->value == pvmSaving) {
            for (auto& val : volumes)
                val = 15.f;
        }
        const double v = volumes.empty() ? 0. : *std::max_element(volumes.begin(), volumes.end());
#endif
        // The plate's filament count from its last slice (slice_info), else the
        // filaments its objects use (PartPlate::get_extruders_under_cli).
        unsigned int filaments_cnt = (unsigned int)sliced_filament_count;
        if (filaments_cnt == 0)
            filaments_cnt = (unsigned int)arrange_plate_extruders(model, config, plate_index).size();

        if ((filaments_cnt <= 1) && !is_smooth_timelapse && (!enable_wrapping_detect || wrapping_area_empty)) {
            // Not a multi-colour plate: no tower to keep clear of.
        } else {
            // The printer's extruder count: one per nozzle_diameter entry
            // (DynamicPrintConfig::support_different_extruders, Bambu
            // PrintConfig.cpp 7776-7783; OrcaSlicer.cpp 2874-2875).
            int extruder_count = 1;
            if (const auto* nozzles = dynamic_cast<const ConfigOptionVectorBase*>(config.option("nozzle_diameter")))
                extruder_count = std::max(1, (int)nozzles->size());
            Vec3d wipe_tower_size = arrange_estimate_wipe_tower_size(model, config, w, v, extruder_count, (int)filaments_cnt,
                                                                     enable_wrapping_detect, plate_box, exclude_boxes);
            // PartPlateList's plate size: the bed's whole-mm width and depth
            // (BambuStudio.cpp 2054-2056, 4242-4247; reset_size 4313-4320).
            const int plate_width = area->values.size() >= 4 ? (int)(area->values[2].x() - area->values[0].x()) : (int)bed.size().x();
            const int plate_depth = area->values.size() >= 4 ? (int)(area->values[2].y() - area->values[0].y()) : (int)bed.size().y();
            float depth = wipe_tower_size(1);
            float margin = 15.f, wp_brim_width = 0.f;
            if (const ConfigOption* wipe_tower_brim_width_opt = config.option("prime_tower_brim_width")) {
                wp_brim_width = wipe_tower_brim_width_opt->getFloat();
#ifdef ENGINE_SNAPMAKER
                if (wp_brim_width < 0) wp_brim_width = 0.f;   // no automatic brim at bac6fd5
#else
                if (wp_brim_width < 0) wp_brim_width = WipeTower::get_auto_brim_by_height((float) wipe_tower_size.z());
#endif
            }
            w = wipe_tower_size(0);
#ifndef ENGINE_SNAPMAKER
            // Nozzles that each reach only part of the bed (H2D, H2C): the
            // tower starts inside the area every nozzle reaches, as the
            // desktop puts a loaded project's tower back inside it
            // (set_default_wipe_tower_pos_for_plate with keep_existing, Bambu
            // PartPlate.cpp 5141-5154 and Plater.cpp 12275 at v02.09.01.70).
            // A model with no saved tower carries the setting's default
            // corner, which one nozzle cannot reach: the slice then ends -102.
            // Only a tower that does not lie inside that area moves, by its
            // estimated size: one inside it stays where the project put it.
            const BoundingBoxf reach = tower_reach_box(config, *area);
            if (const auto* areas = config.option<ConfigOptionPointsGroups>("extruder_printable_area");
                areas && areas->values.size() > 1 &&
                (x < reach.min.x() || x + w > reach.max.x() || y < reach.min.y() || y + depth > reach.max.y())) {
                assemble_tower_inside_plate(context, config, model, plate_index, /*keep_existing=*/true);
                x = (float)config.option<ConfigOptionFloats>("wipe_tower_x")->get_at(plate_index);
                y = (float)config.option<ConfigOptionFloats>("wipe_tower_y")->get_at(plate_index);
            }
#endif
            if ((y + depth + margin + wp_brim_width) > (float)plate_depth)
                y = (float)plate_depth - depth - margin - wp_brim_width;
            if ((x + w + margin + wp_brim_width) > (float)plate_width)
                x = (float)plate_width - w - margin - wp_brim_width;
            if (x < margin)
                x = margin;
            if (y < margin)
                y = margin;
            // The clamped position is the plate's tower position from here on:
            // print.set_plate_index makes the slice read this entry.
            ConfigOptionFloat wt_x_opt(x);
            ConfigOptionFloat wt_y_opt(y);
            config.option<ConfigOptionFloats>("wipe_tower_x", true)->set_at(&wt_x_opt, plate_index, 0);
            config.option<ConfigOptionFloats>("wipe_tower_y", true)->set_at(&wt_y_opt, plate_index, 0);

            ArrangePolygon wipe_tower_ap;
            Slic3r::Polygon ap({
                {scaled(x - wp_brim_width), scaled(y - wp_brim_width)},
                {scaled(x + w + wp_brim_width), scaled(y - wp_brim_width)},
                {scaled(x + w + wp_brim_width), scaled(y + depth + wp_brim_width)},
                {scaled(x - wp_brim_width), scaled(y + depth + wp_brim_width)}
            });
            wipe_tower_ap.bed_idx = 0;
            wipe_tower_ap.setter = NULL; // do not move wipe tower
            wipe_tower_ap.poly.contour = std::move(ap);
            wipe_tower_ap.translation  = {scaled(0.f), scaled(0.f)};
            wipe_tower_ap.rotation     = a;
            wipe_tower_ap.name = "WipeTower";
            wipe_tower_ap.is_virt_object = true;
            wipe_tower_ap.is_wipe_tower = true;
            ++wipe_tower_ap.priority;
            unselected.emplace_back(std::move(wipe_tower_ap));
        }
    }

    // The exclusion areas as fixed items (BambuStudio.cpp 5882; OrcaSlicer.cpp 5138).
    arrange_add_exclude_areas(unselected, config, 0.);

    // Step 2: the arrange settings (BambuStudio.cpp 5886-5909; OrcaSlicer.cpp
    // 5142-5166), with the official CLI's option values: allow_multicolor_oneplate
    // true and avoid_extrusion_cali_region false (their defaults, PrintConfig.cpp
    // 9896-9909 at 5873b5f, 10743-10756 at 31f6803); the clearances and the
    // printable height from the printer (BambuStudio.cpp 4226-4230;
    // OrcaSlicer.cpp 3638-3642).
    //
    // Rotations stay off. This is a deliberate choice: it follows the desktop
    // app's Arrange (GLCanvas3D.hpp ArrangeSettings enable_rotation = false,
    // line 564 at 5873b5f, 493 at 31f6803), not the official CLI, whose
    // allow_rotations option defaults to true (PrintConfig.cpp 9901-9904 at
    // 5873b5f, 10748-10751 at 31f6803). On an I3 printer align_to_y_axis
    // still turns a long part to the Y axis, as the official does (Bambu
    // Arrange.cpp 946-959, long side > 1.1 x short side; Orca
    // update_selected_items_axis_align, Arrange.cpp 155-254).
    // --allow-rotations, --allow-multicolor-oneplate and
    // --avoid-extrusion-cali-region set them as the official CLI does
    // (BambuStudio.cpp 5886-5888; OrcaSlicer.cpp 5143-5145); rotations stay
    // off unless --allow-rotations is given.
    params.allow_rotations                     = context.g_arrange_switches.allow_rotations;
    params.allow_multi_materials_on_same_plate = context.g_arrange_switches.allow_multicolor_oneplate;
    params.avoid_extrusion_cali_region         = context.g_arrange_switches.avoid_extrusion_cali_region;
    params.clearance_height_to_rod             = arrange_opt(config, "extruder_clearance_height_to_rod")->getFloat();
    params.clearance_height_to_lid             = arrange_opt(config, "extruder_clearance_height_to_lid")->getFloat();
#ifdef ENGINE_ORCA
    params.clearance_radius                    = arrange_opt(config, "extruder_clearance_radius")->getFloat();
#else
    params.cleareance_radius                   = arrange_opt(config, "extruder_clearance_max_radius")->getFloat();
#endif
    params.printable_height                    = arrange_opt(config, "printable_height")->getFloat();
    params.min_obj_distance = 0;
    if (params.is_seq_print) {
        // BED_SHRINK_SEQ_PRINT: 0 on Bambu, 5 on Orca (each engine's Arrange.hpp).
        params.bed_shrink_x = BED_SHRINK_SEQ_PRINT;
        params.bed_shrink_y = BED_SHRINK_SEQ_PRINT;
    }
    if (auto printer_structure_opt = config.option<ConfigOptionEnum<PrinterStructure>>("printer_structure"))
        params.align_to_y_axis = (printer_structure_opt->value == PrinterStructure::psI3);

#ifdef ENGINE_ORCA
    update_arrange_params(params, &config, selected);
    update_selected_items_inflation(selected, &config, params);
    update_unselected_items_inflation(unselected, &config, params);
    update_selected_items_axis_align(selected, &config, params);
    Points beds = get_shrink_bedpts(&config, params);
#else
    update_arrange_params(params, config, selected);
    update_selected_items_inflation(selected, config, params);
    update_unselected_items_inflation(unselected, config, params);
    Points beds = get_shrink_bedpts(config, params);
#endif
    arrange_add_exclude_areas(params.excluded_regions, config, scale_(1));

    // Step 3 (BambuStudio.cpp 5925-5926; OrcaSlicer.cpp 5185-5186).
    arrangement::arrange(selected, unselected, beds, params);
    arrangement::arrange(unprintable, {}, beds, params);

    // Every selected item must land on this plate (BambuStudio.cpp 5936-5945;
    // OrcaSlicer.cpp 5196-5205).
    std::string off_bed, only_turned;
    if (landed_out != nullptr) {
        int landed = 0;
        for (const ArrangePolygon& ap : selected)
            landed += ap.bed_idx == 0 ? 1 : 0;
        *landed_out = landed;
        apply_arrange_polys(selected, selected_instances, [](ArrangePolygon&) {});
        apply_arrange_polys(unprintable, unprintable_instances, [](ArrangePolygon&) {});
        for (ModelObject* object : model.objects)
            object->ensure_on_bed();
        return landed == int(selected.size());
    }
    for (size_t k = 0; k < selected.size(); ++k) {
        if (selected[k].bed_idx == 0)
            continue;
        off_bed += (off_bed.empty() ? "'" : ", '") + selected[k].name + "'";
        // A part that fits the bed only turned 90 degrees, which this
        // engine's arrange did not do.
        const ModelInstance* inst = selected_instances[k];
        const ModelObject* object = inst->get_object();
        const auto it = std::find(object->instances.begin(), object->instances.end(), inst);
        const Vec3d size = object->instance_bounding_box(size_t(it - object->instances.begin())).size();
        if (!(size.x() <= bed.size().x() + EPSILON && size.y() <= bed.size().y() + EPSILON))
            only_turned += (only_turned.empty() ? "'" : ", '") + object->name + "'";
    }
    if (!off_bed.empty()) {
        std::string detail = "These objects do not fit on the " + bed_size_text(config) +
                             " bed together: " + off_bed + "."
                             // The verdict says what is wrong; this says what to
                             // do with it: the plate's objects can be spread
                             // over the file's other plates (the official
                             // command line arranges one plate per --slice,
                             // BambuStudio.cpp 5936-5945 at 5873b5f /
                             // OrcaSlicer.cpp 5196-5205 at 31f6803), or a
                             // bigger printer's bed takes them together.
                             " Move some objects to another plate in the file, or use a printer with a bigger bed.";
        if (!only_turned.empty())
            detail += " " + only_turned + " fit" + (only_turned.find(',') == std::string::npos ? "s" : "") +
                      " the bed only turned 90 degrees, which this arrange does not do; turn it in the file.";
        set_outcome_failure(outcome, CLI_OBJECT_ARRANGE_FAILED, detail);
        emit_event({{"event","plate_error"}, {"tag","ArrangeFailed"}, {"message", detail}});
        return false;
    }
    apply_arrange_polys(selected, selected_instances, [](ArrangePolygon&) {});
    // Unprintable instances: the official moves them on to a virtual bed past
    // the last plate (BambuStudio.cpp 5989-5997); a one-plate slice has no such
    // bed, so they keep their own arrange result on this bed. They are not printed.
    apply_arrange_polys(unprintable, unprintable_instances, [](ArrangePolygon&) {});
    for (ModelObject* object : model.objects)
        object->ensure_on_bed();
    json placed = json::array();
    for (const ModelObject* object : model.objects) {
        const BoundingBoxf3 box = object_world_bbox(object);
        placed.push_back(json{{"object", object->name},
                              {"center_x_mm", box.center().x()}, {"center_y_mm", box.center().y()}});
    }
    emit_event({{"event","arranged"}, {"tag","ObjectsArranged"}, {"objects", placed},
                {"message","Placed " + std::to_string(model.objects.size()) + " object(s) on the " +
                           bed_size_text(config) + " bed"}});
    return true;
}

/// The desktop's default prime-tower corner: the top left of the bed, the I3
/// printers' own on a psI3 structure (BambuStudio.cpp 5382-5395; OrcaSlicer.cpp
/// 4635-4649; the defaults are PartPlate.cpp 68-74 / 66-70 in each engine, and
/// set_default_wipe_tower_pos_for_plate reads the same ones: Orca PartPlate.cpp
/// 4128-4135, Bambu 4625-4635). Bambu's A2L keeps its own y (model id "N9",
/// PartPlate.cpp 4637-4642).
Slic3r::Vec2d tower_default_corner(CliRunContext& context, const Slic3r::DynamicPrintConfig& config) {
    using namespace Slic3r;
#ifdef ENGINE_SNAPMAKER
    return slicer_cli::snapmaker_tower_default_corner(config);
#else
    Vec2d corner(165., 250.);   // WIPE_TOWER_DEFAULT_X_POS / _Y_POS
    if (const auto* structure = config.option<ConfigOptionEnum<PrinterStructure>>("printer_structure");
        structure && structure->value == PrinterStructure::psI3)
        corner = Vec2d(0., 250.);   // I3_WIPE_TOWER_DEFAULT_X_POS / _Y_POS
#ifdef ENGINE_BAMBU
    if (context.g_preset_printer_model_id == "N9")   // N9_WIPE_TOWER_DEFAULT_Y_POS
        corner.y() = 160.;
#endif
    return corner;
#endif // ENGINE_SNAPMAKER
}

/// The prime tower's starting corner on an assemble-list plate, kept a margin
/// from the edges (BambuStudio.cpp 5382-5395, 5522-5535; OrcaSlicer.cpp
/// 4635-4649, 4776-4789). The Orca arrange keeps it 1 mm plus
/// prime_tower_width from the edges, as its block reads it (OrcaSlicer.cpp
/// 4636-4637); the Orca fixed plate and both Bambu blocks use
/// WIPE_TOWER_MARGIN (15 mm on Bambu, 1 mm on Orca).
void assemble_tower_default_corner(CliRunContext& context, Slic3r::DynamicPrintConfig& config, int plate_index, float margin) {
    using namespace Slic3r;
    const Vec2d corner = tower_default_corner(context, config);
    float x = (float)corner.x(), y = (float)corner.y();
    if (x < margin) x = margin;
    if (y < margin) y = margin;
    ConfigOptionFloat wt_x_opt(x), wt_y_opt(y);
    config.option<ConfigOptionFloats>("wipe_tower_x", true)->set_at(&wt_x_opt, plate_index, 0);
    config.option<ConfigOptionFloats>("wipe_tower_y", true)->set_at(&wt_y_opt, plate_index, 0);
}



TowerWipeParams tower_wipe_params(const Slic3r::DynamicPrintConfig& config) {
    using namespace Slic3r;
    TowerWipeParams p;
    p.width = arrange_opt(config, "prime_tower_width")->getFloat();
#ifdef ENGINE_ORCA
    p.volume = arrange_opt(config, "prime_volume")->getFloat();
#else
    std::vector<double> volumes = dynamic_cast<const ConfigOptionFloats*>(arrange_opt(config, "filament_prime_volume"))->values;
    if (const auto* pvm = config.option<ConfigOptionEnum<PrimeVolumeMode>>("prime_volume_mode"); pvm && pvm->value == pvmSaving)
        for (auto& val : volumes)
            val = 15.f;
    p.volume = volumes.empty() ? 0. : *std::max_element(volumes.begin(), volumes.end());
#endif
    if (const auto* nozzles = dynamic_cast<const ConfigOptionVectorBase*>(config.option("nozzle_diameter")))
        p.extruder_count = std::max(1, (int)nozzles->size());
    const auto* wrapping_opt = config.option<ConfigOptionBool>("enable_wrapping_detection");
    p.wrapping = wrapping_opt != nullptr && wrapping_opt->value;
    return p;
}

/// PartPlateList::set_default_wipe_tower_pos_for_plate (Orca PartPlate.cpp
/// 4115-4190; Bambu PartPlate.cpp 4615-4702): the desktop's default corner,
/// clamped inside the plate's bounding box intersected with the extruder
/// areas, for a tower sized over the plate's own objects. A new plate sizes it
/// for two extruders (init_pos: Orca PartPlate.cpp 4300, 4374; Bambu 4814,
/// 4893), which is why the desktop's own fixed-plate assemble run lands a
/// tower inside the bed and a copy of the official CLI's corner does not.
/// `keep_existing` keeps the plate's saved position and only clamps it back
/// inside the bed, as the 3MF load does (Bambu PartPlate.hpp 840-844,
/// Plater.cpp 8774).
void assemble_tower_inside_plate(CliRunContext& context, Slic3r::DynamicPrintConfig& config, const Slic3r::Model& model,
                                        int plate_index, bool keep_existing) {
    using namespace Slic3r;
    auto* wipe_x = config.option<ConfigOptionFloats>("wipe_tower_x", true);
    auto* wipe_y = config.option<ConfigOptionFloats>("wipe_tower_y", true);
    const Vec2d corner = tower_default_corner(context, config);
    float x = (float)corner.x(), y = (float)corner.y();
    if (keep_existing && plate_index >= 0 && (int)wipe_x->values.size() > plate_index &&
        (int)wipe_y->values.size() > plate_index) {
        x = (float)wipe_x->get_at(plate_index);
        y = (float)wipe_y->get_at(plate_index);
    }
    const auto* area = config.option<ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 3)
        return;
    BoundingBoxf plate_bbox = tower_reach_box(config, *area);
    const double height = config.has("printable_height") ? config.opt_float("printable_height") : 0.;
    const BoundingBoxf3 plate_box(Vec3d(plate_bbox.min.x(), plate_bbox.min.y(), 0.),
                                  Vec3d(plate_bbox.max.x(), plate_bbox.max.y(), height));
    const TowerWipeParams wipe = tower_wipe_params(config);
    const Vec3d tower = arrange_estimate_wipe_tower_size(model, config, wipe.width, wipe.volume, wipe.extruder_count,
                                                         /*plate_extruder_size=*/2, wipe.wrapping, plate_box,
                                                         arrange_exclude_boxes(config));
    float margin = WIPE_TOWER_MARGIN;
#ifdef ENGINE_ORCA
    // Orca's margin carries the tower's brim, the height rule when negative
    // (Orca PartPlate.cpp 4162-4169).
    float brim_width = arrange_opt(config, "prime_tower_brim_width")->getFloat();
#ifdef ENGINE_SNAPMAKER
    if (brim_width < 0) brim_width = 0.f;   // no automatic brim at bac6fd5
#else
    if (brim_width < 0) brim_width = WipeTower::get_auto_brim_by_height((float) tower.z());
#endif
    margin += brim_width;
#endif
    if (x + margin + (float)tower(0) > (float)plate_bbox.max.x())
        x = (float)plate_bbox.max.x() - (float)tower(0) - margin;
    else if (x < margin + (float)plate_bbox.min.x())
        x = margin + (float)plate_bbox.min.x();
    if (y + margin + (float)tower(1) > (float)plate_bbox.max.y())
        y = (float)plate_bbox.max.y() - (float)tower(1) - margin;
    else if (y < margin)
        y = margin;
    ConfigOptionFloat wt_x_opt(x), wt_y_opt(y);
    wipe_x->set_at(&wt_x_opt, plate_index, 0);
    wipe_y->set_at(&wt_y_opt, plate_index, 0);
}

/// PartPlate::estimate_wipe_tower_polygon with use_global_objects (Bambu
/// PartPlate.cpp 2236-2281; Orca 2174-2220): plate_index's tower position
/// clamped inside a plate of plate_width x plate_depth (the plate list's
/// size), sized for `filaments_count` filaments and the tallest object of
/// `global_model`. Returns the fixed tower item (bed 0) and the clamped
/// position in `pos`; the caller writes it back, as the official callers do.
Slic3r::arrangement::ArrangePolygon estimate_tower_polygon_global(const Slic3r::Model& global_model,
                                                                        const Slic3r::DynamicPrintConfig& config,
                                                                        int plate_index, int filaments_count,
                                                                        int plate_width, int plate_depth,
                                                                        Slic3r::Vec2d& pos) {
    using namespace Slic3r;
    using namespace Slic3r::arrangement;
    float x = dynamic_cast<const ConfigOptionFloats*>(config.option("wipe_tower_x"))->get_at(plate_index);
    float y = dynamic_cast<const ConfigOptionFloats*>(config.option("wipe_tower_y"))->get_at(plate_index);
    const TowerWipeParams wipe = tower_wipe_params(config);
    const float w = wipe.width;
    const double v = wipe.volume;
    const int extruder_count = wipe.extruder_count;
    const bool enable_wrapping_detect = wipe.wrapping;
#ifdef ENGINE_ORCA
    const float margin = WIPE_TOWER_MARGIN + arrange_opt(config, "prime_tower_brim_width")->getFloat();
#else
    const float margin = WIPE_TOWER_MARGIN;
#endif
    // use_global_objects: every object, so the plate box and the exclusion
    // boxes are not read.
    const Vec3d wt_size = arrange_estimate_wipe_tower_size(global_model, config, w, v, extruder_count, filaments_count,
                                                           enable_wrapping_detect, BoundingBoxf3(), {}, &global_model);
    const float depth = wt_size(1);
    float wp_brim_width = 0.f;
    if (const ConfigOption* brim_opt = config.option("prime_tower_brim_width")) {
        wp_brim_width = brim_opt->getFloat();
#ifdef ENGINE_SNAPMAKER
        if (wp_brim_width < 0) wp_brim_width = 0.f;   // no automatic brim at bac6fd5
#else
        if (wp_brim_width < 0) wp_brim_width = WipeTower::get_auto_brim_by_height((float) wt_size.z());
#endif
    }
    // std::clamp(x, margin, plate - w - margin - brim), spelled out.
    const auto clamp = [](float value, float lo, float hi) { return value < lo ? lo : (hi < value ? hi : value); };
    x = clamp(x, margin, (float)plate_width - w - margin - wp_brim_width);
    y = clamp(y, margin, (float)plate_depth - depth - margin - wp_brim_width);
    pos = Vec2d(x, y);

    ArrangePolygon wipe_tower_ap;
    Slic3r::Polygon ap({
        {scaled(x - wp_brim_width), scaled(y - wp_brim_width)},
        {scaled(x + w + wp_brim_width), scaled(y - wp_brim_width)},
        {scaled(x + w + wp_brim_width), scaled(y + depth + wp_brim_width)},
        {scaled(x - wp_brim_width), scaled(y + depth + wp_brim_width)}
    });
    wipe_tower_ap.bed_idx = plate_index;
    wipe_tower_ap.setter = NULL; // do not move wipe tower
    wipe_tower_ap.poly.contour = std::move(ap);
    wipe_tower_ap.translation = {scaled(0.f), scaled(0.f)};
    wipe_tower_ap.name = "WipeTower";
    wipe_tower_ap.is_virt_object = true;
    wipe_tower_ap.is_wipe_tower = true;
    return wipe_tower_ap;
}

/// One --load-assemble-list plate with need_arrange (BambuStudio.cpp
/// 5350-5502; OrcaSlicer.cpp 4606-4760). Unlike the general arrange, the
/// tower starts at the default corner and is sized for the filaments the
/// plate's objects use, measured over every object of the list
/// (estimate_wipe_tower_polygon with use_global_objects, Bambu PartPlate.cpp
/// 2236-2281, Orca 2174-2220), and the objects are only moved, never dropped
/// to the bed. An object the arrange leaves off the plate is
/// CLI_OBJECT_ARRANGE_FAILED.
bool arrange_assemble_plate(CliRunContext& context, Slic3r::Model& model, Slic3r::DynamicPrintConfig& config, int plate_index,
                                   int filaments_count, const Slic3r::Model& global_model, PlateOutcome& outcome) {
    using namespace Slic3r;
    using namespace Slic3r::arrangement;
    const auto* area = config.option<ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 3) {
        set_outcome_failure(outcome, CLI_OBJECT_ARRANGE_FAILED, "The printer states no bed shape (printable_area).");
        return false;
    }
    BoundingBoxf bed;
    for (const Vec2d& pt : area->values) bed.merge(pt);
    const double bed_height = config.has("printable_height") ? config.opt_float("printable_height") : 0.;

    ArrangeParams params;
    params.progressind = [](unsigned, std::string) {};
    if (const auto* seq = config.option<ConfigOptionEnum<PrintSequence>>("print_sequence"))
        params.is_seq_print = seq->value == PrintSequence::ByObject;

    // Step 1: every instance of the plate's objects.
    ArrangePolygons selected, unselected;
    ModelInstancePtrs selected_instances;
    for (ModelObject* mo : model.objects)
        for (ModelInstance* minst : mo->instances) {
            ArrangePolygon ap = get_instance_arrange_poly(minst, config);
            ap.itemid = selected.size();
            selected.emplace_back(std::move(ap));
            selected_instances.emplace_back(minst);
        }

    const auto* wrapping_opt = config.option<ConfigOptionBool>("enable_wrapping_detection");
    const bool enable_wrapping_detect = wrapping_opt != nullptr && wrapping_opt->value;
    const auto* wrapping_area = config.option<ConfigOptionPoints>("wrapping_exclude_area");
    const bool wrapping_area_empty = wrapping_area == nullptr || wrapping_area->values.empty();
    if ((!params.is_seq_print && filaments_count > 1) || (enable_wrapping_detect && !wrapping_area_empty)) {
#ifdef ENGINE_ORCA
        const float corner_margin = WIPE_TOWER_MARGIN + arrange_opt(config, "prime_tower_width")->getFloat();
#else
        const float corner_margin = WIPE_TOWER_MARGIN;
#endif
        assemble_tower_default_corner(context, config, plate_index, corner_margin);
        const int plate_width = area->values.size() >= 4 ? (int)(area->values[2].x() - area->values[0].x()) : (int)bed.size().x();
        const int plate_depth = area->values.size() >= 4 ? (int)(area->values[2].y() - area->values[0].y()) : (int)bed.size().y();
        Vec2d pos;
        ArrangePolygon wipe_tower_ap = estimate_tower_polygon_global(global_model, config, plate_index, filaments_count,
                                                                     plate_width, plate_depth, pos);
        ConfigOptionFloat wt_x_opt(pos.x()), wt_y_opt(pos.y());
        config.option<ConfigOptionFloats>("wipe_tower_x", true)->set_at(&wt_x_opt, plate_index, 0);
        config.option<ConfigOptionFloats>("wipe_tower_y", true)->set_at(&wt_y_opt, plate_index, 0);
        wipe_tower_ap.bed_idx = 0;
        unselected.emplace_back(std::move(wipe_tower_ap));
    }
    arrange_add_exclude_areas(unselected, config, 0.);

    // Step 2: the same settings as the general arrange (see arrange_on_bed).
    params.allow_rotations                     = context.g_arrange_switches.allow_rotations;
    params.allow_multi_materials_on_same_plate = context.g_arrange_switches.allow_multicolor_oneplate;
    params.avoid_extrusion_cali_region         = context.g_arrange_switches.avoid_extrusion_cali_region;
    params.clearance_height_to_rod             = arrange_opt(config, "extruder_clearance_height_to_rod")->getFloat();
    params.clearance_height_to_lid             = arrange_opt(config, "extruder_clearance_height_to_lid")->getFloat();
#ifdef ENGINE_ORCA
    params.clearance_radius                    = arrange_opt(config, "extruder_clearance_radius")->getFloat();
#else
    params.cleareance_radius                   = arrange_opt(config, "extruder_clearance_max_radius")->getFloat();
#endif
    params.printable_height                    = arrange_opt(config, "printable_height")->getFloat();
    params.min_obj_distance = 0;
    if (params.is_seq_print) {
        params.bed_shrink_x = BED_SHRINK_SEQ_PRINT;
        params.bed_shrink_y = BED_SHRINK_SEQ_PRINT;
    }
    if (auto printer_structure_opt = config.option<ConfigOptionEnum<PrinterStructure>>("printer_structure"))
        params.align_to_y_axis = (printer_structure_opt->value == PrinterStructure::psI3);
#ifdef ENGINE_ORCA
    update_arrange_params(params, &config, selected);
    update_selected_items_inflation(selected, &config, params);
    update_unselected_items_inflation(unselected, &config, params);
    update_selected_items_axis_align(selected, &config, params);
    Points beds = get_shrink_bedpts(&config, params);
#else
    update_arrange_params(params, config, selected);
    update_selected_items_inflation(selected, config, params);
    update_unselected_items_inflation(unselected, config, params);
    Points beds = get_shrink_bedpts(config, params);
#endif
    arrange_add_exclude_areas(params.excluded_regions, config, scale_(1));

    // Step 3 and the landing check (BambuStudio.cpp 5476-5499; OrcaSlicer.cpp 4733-4756).
    arrangement::arrange(selected, unselected, beds, params);
    std::string off_bed;
    for (const ArrangePolygon& ap : selected)
        if (ap.bed_idx != 0)
            off_bed += (off_bed.empty() ? "'" : ", '") + ap.name + "'";
    if (!off_bed.empty()) {
        const std::string detail = "These objects of the assemble list's plate " + std::to_string(plate_index + 1) +
                                   " do not fit on the " + bed_size_text(config) + " bed together: " + off_bed +
                                   ". Give the plate fewer objects or smaller copies, or use a bigger printer.";
        set_outcome_failure(outcome, CLI_OBJECT_ARRANGE_FAILED, detail);
        emit_event({{"event","plate_error"}, {"tag","ArrangeFailed"}, {"message", detail}});
        return false;
    }
    apply_arrange_polys(selected, selected_instances, [](ArrangePolygon&) {});
    json placed = json::array();
    for (const ModelObject* object : model.objects) {
        const BoundingBoxf3 box = object_world_bbox(object);
        placed.push_back(json{{"object", object->name},
                              {"center_x_mm", box.center().x()}, {"center_y_mm", box.center().y()}});
    }
    emit_event({{"event","arranged"}, {"tag","ObjectsArranged"}, {"objects", placed},
                {"message","Placed " + std::to_string(model.objects.size()) + " object(s) on the " +
                           bed_size_text(config) + " bed"}});
    return true;
}

} // namespace slicer_cli
