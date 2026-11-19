#include "cli_plate_plan.hpp"

#include "cli_run_context.hpp"
#include "cli_options.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/BuildVolume.hpp"
#include <iostream>
#include "cli_arrange_geometry.hpp"
#include "cli_events.hpp"
#include "cli_model_load.hpp"
#include "cli_plate_arrange.hpp"
#include "cli_result_json.hpp"
#include "cli_run_steps.hpp"

namespace slicer_cli {

using json = nlohmann::json;

/// A plate's logical width and depth, as the desktop's PartPlateList sizes
/// it from the printable area: whole millimetres, plus the axes tip for files
/// older than 1.5.9 (through reset_size(int, int, ...)). Zero without an area.
static Slic3r::Vec2d plate_logical_size(const Slic3r::DynamicPrintConfig& file_config, const Slic3r::Semver& file_version)
{
    const auto* area = file_config.option<Slic3r::ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 4)
        return Slic3r::Vec2d::Zero();
    double width = (int)(area->values[2].x() - area->values[0].x());
    double depth = (int)(area->values[2].y() - area->values[0].y());
    if (file_version.maj() + file_version.min() + file_version.patch() > 0 &&
        file_version < Slic3r::Semver(1, 5, 9)) {
        // + bed3d_ax3s_default_tip_radius, through reset_size(int, int, ...).
        width = (int)(width + 2.5f * 0.5f);
        depth = (int)(depth + 2.5f * 0.5f);
    }
    return Slic3r::Vec2d(width, depth);
}

/// The scene origin of plate `index` (0-based) in a project of `plate_count`
/// plates, as the desktop's PartPlateList lays them out (see the call site).
Slic3r::Vec2d plate_grid_origin(const Slic3r::DynamicPrintConfig& file_config,
                                       const Slic3r::Semver& file_version, int index, int plate_count)
{
    if (index <= 0 || plate_count <= 1)
        return Slic3r::Vec2d::Zero();
    const Slic3r::Vec2d size = plate_logical_size(file_config, file_version);
    if (size.x() <= 0. || size.y() <= 0.)
        return Slic3r::Vec2d::Zero();
    // compute_colum_count (PartPlate.hpp:38).
    const float value = std::sqrt((float)plate_count);
    const float round_value = std::round(value);
    const int cols = value > round_value ? (int)round_value + 1 : (int)round_value;
    const int row = index / cols, col = index % cols;
    const double gap = 1. / 5.;   // LOGICAL_PART_PLATE_GAP
    return Slic3r::Vec2d(col * size.x() * (1. + gap), -row * size.y() * (1. + gap));
}

/// Plate `index`'s box in the scene, PartPlate::get_build_volume (PartPlate.cpp
/// 3263-3285 at 5873b5f): its origin plus the first printable_area point, its
/// logical size and the printable height, widened by SceneEpsilon.
Slic3r::BoundingBoxf3 plate_scene_box(const Slic3r::DynamicPrintConfig& file_config,
                                             const Slic3r::Semver& file_version, int index, int plate_count)
{
    const Slic3r::Vec2d origin = plate_grid_origin(file_config, file_version, index, plate_count);
    const Slic3r::Vec2d size = plate_logical_size(file_config, file_version);
    const auto* area = file_config.option<Slic3r::ConfigOptionPoints>("printable_area");
    const Slic3r::Vec2d first = area && !area->values.empty() ? area->values.front() : Slic3r::Vec2d::Zero();
    const double height = file_config.has("printable_height") ? file_config.opt_float("printable_height") : 0.;
    const double eps = Slic3r::BuildVolume::SceneEpsilon;
    return Slic3r::BoundingBoxf3(
        Slic3r::Vec3d(origin.x() + first.x() - eps, origin.y() + first.y() - eps, -eps),
        Slic3r::Vec3d(origin.x() + first.x() + size.x() + eps, origin.y() + first.y() + size.y() + eps, height + eps));
}


/// A project 3MF with model files after it, under --slice or for model
/// actions only: the run the official CLI makes of them (see ProjectPlan).
/// The product's default call (`file --plate N -o`) keeps loading the model
/// files into the plate it slices.
bool trailing_models_port(const CliOptions& o) {
    return o.input_files.size() > 1 && slicer_cli::classify_3mf(o.input_file) == slicer_cli::ThreeMfKind::Project &&
           (o.slice_mode || slicer_cli::model_actions_only(o));
}

/// PartPlateList's plate grid for the plan: plate `index` of a list with
/// `cols` columns sits at (col * width * 1.2, -row * depth * 1.2)
/// (compute_origin, Bambu PartPlate.cpp 4306-4341; plate_stride_x/y with
/// LOGICAL_PART_PLATE_GAP 1/5). Unlike plate_grid_origin it also places a bed
/// past the last plate, as postprocess_arrange_polygon does.
static Slic3r::Vec2d plan_plate_origin(const Slic3r::Vec2d& plate_size, int index, int cols) {
    const double gap = 1. / 5.;
    const int row = index / cols, col = index % cols;
    return Slic3r::Vec2d(col * plate_size.x() * (1. + gap), -row * plate_size.y() * (1. + gap));
}

/// compute_colum_count (PartPlate.hpp:38).
static int plan_plate_cols(int count) {
    const float value = std::sqrt((float)std::max(1, count));
    const float round_value = std::round(value);
    return value > round_value ? (int)round_value + 1 : (int)round_value;
}

/// The plan of a project 3MF with model files after it (see ProjectPlan).
/// `model` holds every plate's objects at their scene places followed by the
/// model files' objects, and the arrange moves them in place; `config` is the
/// run's settings at the arrange (the setting flags applied), whose
/// wipe_tower_x / wipe_tower_y the arrange updates; `file_config` and
/// `file_version` are the project's own, for the plate list's size (the old
/// printer's, as the plate list keeps it without the Bambu-project resize,
/// BambuStudio.cpp 4313-4320; OrcaSlicer.cpp 3727-3729).
bool plan_project_plates(CliRunContext& context, Slic3r::Model& model, Slic3r::DynamicPrintConfig& config,
                                const Slic3r::DynamicPrintConfig& file_config, const Slic3r::Semver& file_version,
                                const Slic3r::PlateDataPtrs& plate_data, int filament_count, bool need_arrange,
                                ProjectPlan& plan, PlateOutcome& outcome) {
    using namespace Slic3r;
    using namespace Slic3r::arrangement;
    const int file_plates = std::max(1, (int)plate_data.size());
    const Vec2d plate_size = plate_logical_size(file_config, file_version);
    const auto* area = config.option<ConfigOptionPoints>("printable_area");
    if (!area || area->values.size() < 3 || plate_size.x() <= 0. || plate_size.y() <= 0.) {
        set_outcome_failure(outcome, CLI_OBJECT_ARRANGE_FAILED, "The printer states no bed shape (printable_area).");
        return false;
    }
    const Vec2d first_point = area->values.front();
    const double plate_height = file_config.has("printable_height") ? file_config.opt_float("printable_height") : 0.;
    // PartPlate::get_build_volume (Bambu PartPlate.cpp 3263-3285): the
    // plate's origin plus the bed's first point, the plate list's size.
    const auto plate_box = [&](int index, int count) {
        const Vec2d origin = plan_plate_origin(plate_size, index, plan_plate_cols(count));
        const double eps = BuildVolume::SceneEpsilon;
        return BoundingBoxf3(Vec3d(origin.x() + first_point.x() - eps, origin.y() + first_point.y() - eps, -eps),
                             Vec3d(origin.x() + first_point.x() + plate_size.x() + eps,
                                   origin.y() + first_point.y() + plate_size.y() + eps, plate_height + eps));
    };

    // The plate each instance is on, as the file states it: the plate's
    // identify ids, else the first plate its box meets (load_plate_objects).
    std::map<std::pair<int, int>, int> on_plate;
    for (size_t k = 0; k < plate_data.size(); ++k) {
        if (plate_data[k] == nullptr) continue;
        std::set<int> identify_ids;
        for (const auto& entry : plate_data[k]->obj_inst_map)
            if (entry.second.second > 0) identify_ids.insert(entry.second.second);
        for (size_t oi = 0; oi < model.objects.size(); ++oi)
            for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii) {
                const int id = model.objects[oi]->instances[ii]->loaded_id;
                if (id > 0 && identify_ids.count(id))
                    on_plate.emplace(std::make_pair(int(oi), int(ii)), int(k));
            }
    }
    for (size_t oi = 0; oi < model.objects.size(); ++oi)
        for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii) {
            if (on_plate.count({int(oi), int(ii)})) continue;
            const BoundingBoxf3 box = model.objects[oi]->instance_convex_hull_bounding_box(ii);
            for (int k = 0; k < file_plates; ++k)
                if (plate_box(k, file_plates).intersects(box)) {
                    on_plate.emplace(std::make_pair(int(oi), int(ii)), k);
                    break;
                }
        }
    std::vector<bool> locked(file_plates, false);
    for (int k = 0; k < (int)plate_data.size(); ++k)
        locked[k] = plate_data[k] != nullptr && plate_data[k]->locked;

    int plate_count = file_plates;
    if (need_arrange) {
        // Step 1, the global arrange (BambuStudio.cpp 5627-5722; OrcaSlicer.cpp
        // 4887-4983): every instance, those of a locked plate kept where they
        // are (PartPlateList::preprocess_arrange_polygon, Bambu PartPlate.cpp
        // 5831-5871).
        ArrangePolygons selected, unselected, unprintable, locked_aps;
        const int file_cols = plan_plate_cols(file_plates);
        for (size_t oi = 0; oi < model.objects.size(); ++oi) {
            ModelObject* mo = model.objects[oi];
            for (size_t ii = 0; ii < mo->instances.size(); ++ii) {
                ModelInstance* minst = mo->instances[ii];
                ArrangePolygon ap = get_instance_arrange_poly(minst, config);
                const auto it = on_plate.find({int(oi), int(ii)});
                if (it != on_plate.end() && locked[it->second]) {
                    const int k = it->second;
                    ap.bed_idx = k;
                    ap.row = k / file_cols;
                    ap.col = k % file_cols;
                    const Vec2d origin = plan_plate_origin(plate_size, k, file_cols);
                    ap.translation(X) -= scaled<double>(origin.x());
                    ap.translation(Y) -= scaled<double>(origin.y());
                    ap.itemid = locked_aps.size();
                    locked_aps.emplace_back(ap);
                } else {
                    // The official numbers the unprintable items in the
                    // selected list too.
                    ap.itemid = selected.size();
                    if (minst->printable)
                        selected.emplace_back(ap);
                    else
                        unprintable.emplace_back(ap);
                }
            }
        }
        ArrangeParams params;
        params.progressind = [](unsigned, std::string) {};
        if (const auto* seq = config.option<ConfigOptionEnum<PrintSequence>>("print_sequence"))
            params.is_seq_print = seq->value == PrintSequence::ByObject;
        // The exclusion areas on 16 beds (preprocess_exclude_areas' default).
        arrange_add_exclude_areas(unselected, config, 0., 16);
        // The tower on every bed (BambuStudio.cpp 5668-5722; OrcaSlicer.cpp
        // 4927-4981) when filaments are in use: the default corner on each file
        // plate, clamped by estimate_wipe_tower_polygon with every object
        // counted, and the last file plate's tower on the beds after it. The
        // used filaments: on Bambu the project's and --load-filament-ids'
        // (BambuStudio.cpp 2041-2046, 2083-2088); on Orca only
        // --load-filament-ids' (OrcaSlicer.cpp 1734).
        if (filament_count > 0) {
#ifdef ENGINE_SNAPMAKER
            // The one corner function, so this path and the assemble ones cannot
            // drift apart (Snapmaker's x = 13 / y = depth/2, Snapmaker_Orca.cpp:
            // 4206-4213 and 4296-4303 at bac6fd5).
            const Vec2d snap_corner = tower_default_corner(context, config);
            float x = (float) snap_corner.x();
            float y = (float) snap_corner.y();
#else
            float x = 165.f, y = 250.f;   // WIPE_TOWER_DEFAULT_X_POS / _Y_POS
            if (const auto* structure = config.option<ConfigOptionEnum<PrinterStructure>>("printer_structure");
                structure && structure->value == PrinterStructure::psI3) {
                x = 0.f;     // I3_WIPE_TOWER_DEFAULT_X_POS
                y = 250.f;   // I3_WIPE_TOWER_DEFAULT_Y_POS
            }
#endif
#ifdef ENGINE_ORCA
            const float tower_margin = WIPE_TOWER_MARGIN + arrange_opt(config, "prime_tower_width")->getFloat();
#else
            const float tower_margin = WIPE_TOWER_MARGIN;
#endif
            if (x < tower_margin) x = tower_margin;
            if (y < tower_margin) y = tower_margin;
            ConfigOptionFloat wt_x_opt(x), wt_y_opt(y);
            auto* wipe_x_option = config.option<ConfigOptionFloats>("wipe_tower_x", true);
            auto* wipe_y_option = config.option<ConfigOptionFloats>("wipe_tower_y", true);
            for (int bedid = 0; bedid < 36; bedid++) {   // MAX_PLATE_COUNT
                const int plate_index_valid = std::min(bedid, file_plates - 1);
                if (bedid < file_plates) {
                    wipe_x_option->set_at(&wt_x_opt, plate_index_valid, 0);
                    wipe_y_option->set_at(&wt_y_opt, plate_index_valid, 0);
                }
                Vec2d pos;
                ArrangePolygon wipe_tower_ap = estimate_tower_polygon_global(
                    model, config, plate_index_valid, filament_count, (int)plate_size.x(), (int)plate_size.y(), pos);
                if (bedid < file_plates) {
                    wt_x_opt.value = pos.x();
                    wt_y_opt.value = pos.y();
                    wipe_x_option->set_at(&wt_x_opt, plate_index_valid, 0);
                    wipe_y_option->set_at(&wt_y_opt, plate_index_valid, 0);
                }
                wipe_tower_ap.bed_idx = bedid;
                unselected.emplace_back(std::move(wipe_tower_ap));
            }
        }

        // Step 2 (BambuStudio.cpp 5886-5915; OrcaSlicer.cpp 5142-5172): the
        // same settings as the one-plate arrange (arrange_on_bed).
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

        // Step 3.
        arrangement::arrange(selected, unselected, beds, params);
        arrangement::arrange(unprintable, {}, beds, params);

        // Step 4 (BambuStudio.cpp 5946-6006; OrcaSlicer.cpp 5189-5262): each
        // selected item's bed past the locked plates, adding plates while the
        // bed is past the last one (postprocess_bed_index_for_selected and
        // create_plate, at most MAX_PLATE_COUNT, Bambu PartPlate.cpp 6009-6051,
        // 4834-4842).
        const auto create_plate = [&]() -> int {
            if (plate_count >= 36)
                return -1;
            locked.push_back(false);
            return plate_count++;
        };
        int bed_idx_max = 0;
        for (ArrangePolygon& ap : selected) {
            if (ap.bed_idx != -1) {
                bool placed = false;
                for (int i = 0; i < plate_count; ++i) {
                    if (locked[i]) {
                        ap.bed_idx += 1;
                    } else if (ap.bed_idx <= i) {
                        placed = true;
                        break;
                    }
                }
                if (!placed) {
                    int plate_index = create_plate();
                    while (plate_index != -1) {
                        if (ap.bed_idx <= plate_index)
                            break;
                        plate_index = create_plate();
                    }
                }
            }
            bed_idx_max = std::max(ap.bed_idx, bed_idx_max);
        }
        // postprocess_arrange_polygon (Bambu PartPlate.cpp 6107-6129): onto
        // its plate in the grid of the plates there are now; an item the
        // arrange could not place goes past the last plate.
        const int cols = plan_plate_cols(plate_count);
        const auto to_grid = [&](ArrangePolygon& ap) {
            const Vec2d origin = plan_plate_origin(plate_size, ap.bed_idx, cols);
            ap.translation(X) += scaled<double>(origin.x());
            ap.translation(Y) += scaled<double>(origin.y());
        };
        for (ArrangePolygon& ap : locked_aps) {
            bed_idx_max = std::max(ap.bed_idx, bed_idx_max);
            to_grid(ap);
            ap.apply();
        }
        for (ArrangePolygon& ap : selected) {
            if (ap.bed_idx == -1) {
                ap.bed_idx = plate_count;
                const BoundingBox apbox = get_extents(ap.transformed_poly());
                const auto apbox_size = apbox.size();
                ap.translation(X) = 0.5 * apbox_size[0];
                ap.translation(Y) = scaled<double>(static_cast<double>((int)plate_size.y())) - 0.5 * apbox_size[1];
            }
            to_grid(ap);
            ap.apply();
        }
        for (ArrangePolygon& ap : unprintable) {
            ap.bed_idx = bed_idx_max + 1;
            to_grid(ap);
            ap.apply();
        }
        // rebuild_plates_after_arrangement (Bambu PartPlate.cpp 6803-6860;
        // Orca 6005-6037): the objects in arrange order (stable here, where the
        // official's std::sort leaves equal orders in any order).
        std::stable_sort(model.objects.begin(), model.objects.end(), [](const ModelObject* a, const ModelObject* b) {
            const int ao = a->instances.empty() ? 0 : a->instances[0]->arrange_order;
            const int bo = b->instances.empty() ? 0 : b->instances[0]->arrange_order;
            return ao < bo;
        });
        plan.arranged = true;
    }

    // reload_all_objects (Bambu PartPlate.cpp 5668-5710): each instance on
    // the first plate its box meets.
    std::vector<std::vector<std::pair<int, int>>> members(plate_count);
    int off_plates = 0;
    for (size_t oi = 0; oi < model.objects.size(); ++oi)
        for (size_t ii = 0; ii < model.objects[oi]->instances.size(); ++ii) {
            const BoundingBoxf3 box = model.objects[oi]->instance_convex_hull_bounding_box(ii);
            int k = 0;
            for (; k < plate_count; ++k)
                if (plate_box(k, plate_count).intersects(box)) {
                    members[k].emplace_back(int(oi), int(ii));
                    break;
                }
            if (k == plate_count)
                ++off_plates;
        }
    // Each instance's place on its plate, which a plate keeps when the plates
    // before it go (delete_plate moves a plate with its objects).
    std::vector<std::vector<Vec3d>> on_plate_offset(plate_count);
    for (int k = 0; k < plate_count; ++k) {
        const Vec2d origin = plan_plate_origin(plate_size, k, plan_plate_cols(plate_count));
        for (const auto& [oi, ii] : members[k])
            on_plate_offset[k].push_back(model.objects[oi]->instances[ii]->get_offset() - Vec3d(origin.x(), origin.y(), 0.));
    }
    if (need_arrange) {
        // Recycle the empty plates (Bambu PartPlate.cpp 6837-6858: every empty
        // plate after the first; Orca 6018-6037: from the last plate back to the
        // first that holds a printable object, past locked ones).
        const auto printable_on = [&](int k) {
            for (const auto& [oi, ii] : members[k])
                if (model.objects[oi]->instances[ii]->printable)
                    return true;
            return false;
        };
        for (int i = plate_count - 1; i > 0; --i) {
            if (members[i].empty() || !printable_on(i)) {
                off_plates += (int)members[i].size();
                members.erase(members.begin() + i);
                on_plate_offset.erase(on_plate_offset.begin() + i);
                locked.erase(locked.begin() + i);
            } else if (locked[i]) {
                continue;
            } else {
#ifdef ENGINE_ORCA
                break;
#else
                continue;
#endif
            }
        }
        plate_count = (int)members.size();
    }
    plan.origins.clear();
    plan.members.assign(plate_count, {});
    const int final_cols = plan_plate_cols(plate_count);
    for (int k = 0; k < plate_count; ++k) {
        const Vec2d origin = plan_plate_origin(plate_size, k, final_cols);
        plan.origins.push_back(origin);
        for (size_t m = 0; m < members[k].size(); ++m) {
            const auto& [oi, ii] = members[k][m];
            ModelInstance* inst = model.objects[oi]->instances[ii];
            inst->set_offset(on_plate_offset[k][m] + Vec3d(origin.x(), origin.y(), 0.));
            plan.members[k].push_back(inst);
        }
    }
    plan.file_plate_count = file_plates;
    plan.plate_count = plate_count;
    plan.off_plates = off_plates;
    if (plan.arranged && filament_count > 0) {
        if (const auto* wx = config.option<ConfigOptionFloats>("wipe_tower_x")) plan.tower_x = wx->values;
        if (const auto* wy = config.option<ConfigOptionFloats>("wipe_tower_y")) plan.tower_y = wy->values;
    }
    plan.done = true;
    return true;
}

/// The event for a project 3MF's plan: the plates the run slices and the
/// objects on each.
void emit_project_plan_event(const ProjectPlan& plan, bool model_files) {
    json plates = json::array();
    for (int k = 0; k < plan.plate_count; ++k) {
        json objects = json::array();
        for (const Slic3r::ModelInstance* inst : plan.members[k])
            objects.push_back(inst->get_object()->name);
        plates.push_back(json{{"plate_id", k + 1}, {"objects", objects}});
    }
    std::string message = plan.arranged && !model_files
        ? "The project's plates were arranged together over " + std::to_string(plan.plate_count) +
              " plate(s), as the official command line arranges them for --slice 0 --arrange 1"
        : plan.arranged
        ? "The project and the model files after it were arranged together over " + std::to_string(plan.plate_count) +
              " plate(s), as the official command line arranges them"
        : "The model files after the project keep their places (--arrange 0) over " +
              std::to_string(plan.plate_count) + " plate(s)";
    if (plan.off_plates > 0)
        message += "; " + std::to_string(plan.off_plates) + " object(s) are on no plate and are not printed";
    emit_event({{"event","arranged"}, {"tag", plan.arranged ? "ProjectArrangedAcrossPlates" : "ProjectPlatesByPosition"},
                {"plate_count", plan.plate_count}, {"file_plate_count", plan.file_plate_count},
                {"off_plates", plan.off_plates}, {"plates", plates}, {"message", message}});
}

} // namespace slicer_cli
