#include "cli_arrange_geometry.hpp"

#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/GCode/WipeTower.hpp"
#include "cli_snapmaker.hpp"

namespace slicer_cli {

// ── --arrange 1: the official per-plate arrange ─────────────────────────
// Port of the official CLI's "arrange this plate only" branch (BambuStudio.cpp
// 5730-5952 at 5873b5f; OrcaSlicer.cpp 4990-5210 at 31f6803). The official
// keeps its plates in slic3r/GUI/PartPlate.cpp, which slicer-cli does not
// link; the few plate steps it needs are ported here, each with its source.
//
// Coordinates: the caller has already moved this plate's objects by minus the
// plate's grid origin (plate_grid_origin, the multi-plate translation in
// slice_one_plate), and the print origin stays at 0. Objects, the bed
// (printable_area), the exclusion boxes (bed_exclude_area) and the prime
// tower (wipe_tower_x/y, per plate) are therefore all in the same plate-local
// bed frame; the official reads the boxes from plate 0, whose origin is
// (0, 0), and the tower from the plate's own wipe_tower_x/y entry, both in
// that frame. Nothing is translated here.

/// A print setting from `config`, or the engine's default when the config
/// does not hold it (FullPrintConfig::defaults(), the values a fresh Print
/// starts with).
const Slic3r::ConfigOption* arrange_opt(const Slic3r::DynamicPrintConfig& config, const std::string& key) {
    if (const Slic3r::ConfigOption* opt = config.option(key))
        return opt;
#ifdef ENGINE_SNAPMAKER
    // A run whose settings come from bare model files can reach the arrange
    // with a setting this engine has no default for; a null here used to crash
    // in arrange_plate_extruders. Zero is the engine's own "Default"/none.
    if (Slic3r::FullPrintConfig::defaults().option(key) == nullptr) {
        static const Slic3r::ConfigOptionInt kZero(0);
        return &kZero;
    }
#endif
    return Slic3r::FullPrintConfig::defaults().option(key);
}

/// The printer's bed exclusion boxes, as PartPlate::calc_bounding_boxes makes
/// them from bed_exclude_area: every 4 points are one box (Bambu
/// slic3r/GUI/PartPlate.cpp 375-392; Orca PartPlate.cpp 418-435). Plate 0's
/// position is (0, 0) (PartPlate::set_shape adds it, Bambu PartPlate.cpp
/// 3185-3189), so the boxes are plate-local bed coordinates.
std::vector<Slic3r::BoundingBoxf> arrange_exclude_boxes(const Slic3r::DynamicPrintConfig& config) {
    std::vector<Slic3r::BoundingBoxf> boxes;
    const auto* area = config.option<Slic3r::ConfigOptionPoints>("bed_exclude_area");
    if (area == nullptr)
        return boxes;
    Slic3r::BoundingBoxf box;
    for (size_t index = 0; index < area->values.size(); ++index) {
        if (index % 4 == 0)
            box = Slic3r::BoundingBoxf();
        box.merge(area->values[index]);
        if (index % 4 == 3)
            boxes.push_back(box);
    }
    return boxes;
}

/// PartPlateList::preprocess_exclude_areas (Bambu PartPlate.cpp 5906-5972;
/// Orca PartPlate.cpp 5363-5429, the same code): the wrapping-detection area
/// when enable_wrapping_detection is on (BambuStudio.cpp 4175-4176;
/// OrcaSlicer.cpp 3588), then one fixed virtual item per exclusion box. The
/// official adds a copy for every bed up to the sliced plate (num_plates,
/// default 16); a one-plate arrange places items on bed 0 only, so it asks
/// for bed 0's copy alone, and the project-wide arrange for the default 16.
/// Also used by --layout-plan (layout_plan.cpp), for a v3 request.
void arrange_add_exclude_areas(Slic3r::arrangement::ArrangePolygons& out,
                               const Slic3r::DynamicPrintConfig& config, double inflation,
                               int num_beds) {
    using namespace Slic3r;
    const auto* wrapping = config.option<ConfigOptionBool>("enable_wrapping_detection");
    if (wrapping != nullptr && wrapping->value) {
        const auto* area = config.option<ConfigOptionPoints>("wrapping_exclude_area");
        if (area != nullptr && !area->values.empty()) {
            Slic3r::Polygon ap{};
            for (const Vec2d& p : area->values)
                ap.append({scale_(p(0)), scale_(p(1))});
            for (int j = 0; j < num_beds; ++j) {
                arrangement::ArrangePolygon ret;
                ret.poly.contour   = ap;
                ret.translation    = Vec2crd(0, 0);
                ret.rotation       = 0.0f;
                ret.is_virt_object = true;
                ret.bed_idx        = j;
                ret.height         = 1;
                ret.name           = "WrappingRegion";
                ret.inflation      = inflation;
                out.emplace_back(std::move(ret));
            }
        }
    }
    const std::vector<BoundingBoxf> boxes = arrange_exclude_boxes(config);
    for (size_t index = 0; index < boxes.size(); ++index) {
        const BoundingBoxf& box = boxes[index];
        Slic3r::Polygon ap({
            {scaled(box.min.x()), scaled(box.min.y())},
            {scaled(box.max.x()), scaled(box.min.y())},
            {scaled(box.max.x()), scaled(box.max.y())},
            {scaled(box.min.x()), scaled(box.max.y())}
        });
        for (int j = 0; j < num_beds; ++j) {
            arrangement::ArrangePolygon ret;
            ret.poly.contour   = ap;
            ret.translation    = Vec2crd(0, 0);
            ret.rotation       = 0.0f;
            ret.is_virt_object = true;
            ret.bed_idx        = j;
            ret.height         = 1;
            ret.name           = "ExcludedRegion" + std::to_string(index);
            ret.inflation      = inflation;
            out.emplace_back(std::move(ret));
        }
    }
}

/// The filaments this plate's objects use, as PartPlate::get_extruders_under_cli
/// (true, config) lists them (Bambu PartPlate.cpp 1275-1428; Orca PartPlate.cpp
/// 1625-1739): each printable instance's volumes, its layer ranges, its support
/// filaments when it has support, the plate's tool-change G-codes, sorted and
/// unique. The model holds only this plate's objects (load_bbs_3mf with the
/// plate id), so every instance is the plate's. Bambu (v02.09.01.70): the
/// global wall / sparse-infill / solid-infill filaments count for every
/// volume (GlobalFeatureFilaments::from_full_config, Model.cpp 3409-3413;
/// PartPlate.cpp 1321, 1336) and a layer range may name them too (1343).
std::vector<int> arrange_plate_extruders(const Slic3r::Model& model,
                                                const Slic3r::DynamicPrintConfig& full_config, int plate_index) {
    using namespace Slic3r;
    std::vector<int> plate_extruders;
    const int glb_support_intf_extr = arrange_opt(full_config, "support_interface_filament")->getInt();
    const int glb_support_extr      = arrange_opt(full_config, "support_filament")->getInt();
#ifndef ENGINE_ORCA
    const GlobalFeatureFilaments global_features = GlobalFeatureFilaments::from_full_config(full_config);
    static const char* const range_keys[] = {"extruder", "wall_filament", "sparse_infill_filament", "solid_infill_filament"};
#else
    static const char* const range_keys[] = {"extruder"};
#endif
#ifdef ENGINE_SNAPMAKER
    // The fork keeps the 1-based feature-filament keys and filament 1 is its
    // default, so only other values count for the tower
    // (references/SnapmakerOrca/src/slic3r/GUI/PartPlate.cpp:1899-1937, 2042-2081
    // at bac6fd5).
    const int glb_wall_extr          = arrange_opt(full_config, "wall_filament")->getInt();
    const int glb_sparse_infill_extr = arrange_opt(full_config, "sparse_infill_filament")->getInt();
    const int glb_solid_infill_extr  = arrange_opt(full_config, "solid_infill_filament")->getInt();
#endif
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
    // OrcaSlicer v2.4.2 renamed the feature-filament keys to `*_id`, where 0
    // means "Default" (PrintConfig.cpp handle_legacy, 8038-8058), and adds the
    // surface and inner-wall keys (PartPlate.cpp 1671-1681). The wall pair
    // falls back to each other, and top/bottom surface to internal solid.
    int glb_outer_wall_extr = arrange_opt(full_config, "outer_wall_filament_id")->getInt();
    int glb_inner_wall_extr = arrange_opt(full_config, "inner_wall_filament_id")->getInt();
    if (glb_outer_wall_extr == 0) glb_outer_wall_extr = glb_inner_wall_extr;
    if (glb_inner_wall_extr == 0) glb_inner_wall_extr = glb_outer_wall_extr;
    const int glb_sparse_infill_extr  = arrange_opt(full_config, "sparse_infill_filament_id")->getInt();
    const int glb_internal_solid_extr = arrange_opt(full_config, "internal_solid_filament_id")->getInt();
    int glb_top_surface_extr    = arrange_opt(full_config, "top_surface_filament_id")->getInt();
    int glb_bottom_surface_extr = arrange_opt(full_config, "bottom_surface_filament_id")->getInt();
    if (glb_top_surface_extr == 0)    glb_top_surface_extr    = glb_internal_solid_extr;
    if (glb_bottom_surface_extr == 0) glb_bottom_surface_extr = glb_internal_solid_extr;
#endif
#ifdef ENGINE_ORCA
    bool glb_support = arrange_opt(full_config, "enable_support")->getBool();
    glb_support |= arrange_opt(full_config, "raft_layers")->getInt() > 0;
#else
    const bool glb_enable_support = arrange_opt(full_config, "enable_support")->getBool();
    const int  glb_raft_layers    = arrange_opt(full_config, "raft_layers")->getInt();
#endif

    for (const ModelObject* object : model.objects) {
        for (const ModelInstance* instance : object->instances) {
            if (!instance->printable)
                continue;
            for (const ModelVolume* mv : object->volumes) {
#ifdef ENGINE_ORCA
                const std::vector<int> volume_extruders = mv->get_extruders();
#else
                const std::vector<int> volume_extruders = mv->get_extruders(global_features);
#endif
                plate_extruders.insert(plate_extruders.end(), volume_extruders.begin(), volume_extruders.end());
            }
            // layer range
            for (const auto& layer_range : object->layer_config_ranges) {
                for (const char* key : range_keys) {
                    if (layer_range.second.has(key)) {
                        if (auto id = layer_range.second.option(key)->getInt(); id > 0)
                            plate_extruders.push_back(id);
                    }
                }
            }
            bool obj_support = false;
            const ConfigOption* obj_support_opt = object->config.option("enable_support");
            const ConfigOption* obj_raft_opt    = object->config.option("raft_layers");
#ifndef ENGINE_ORCA
            // Bambu keeps the two overrides independent (PartPlate.cpp
            // 1351-1363 at v02.09.01.70): an object overriding only raft_layers
            // keeps the global enable_support, and the other way round.
            bool obj_enable_support = glb_enable_support;
            int  obj_raft_layers    = glb_raft_layers;
            if (obj_support_opt != nullptr)
                obj_enable_support = obj_support_opt->getBool();
            if (obj_raft_opt != nullptr)
                obj_raft_layers = obj_raft_opt->getInt();
            obj_support = obj_enable_support || obj_raft_layers > 0;
#else
            if (obj_support_opt != nullptr || obj_raft_opt != nullptr) {
                if (obj_support_opt != nullptr)
                    obj_support = obj_support_opt->getBool();
                if (obj_raft_opt != nullptr)
                    obj_support |= obj_raft_opt->getInt() > 0;
            } else
                obj_support = glb_support;
#endif
            if (obj_support) {
                int obj_support_intf_extr = 0;
                if (const ConfigOption* opt = object->config.option("support_interface_filament"))
                    obj_support_intf_extr = opt->getInt();
                if (obj_support_intf_extr != 0)
                    plate_extruders.push_back(obj_support_intf_extr);
                else if (glb_support_intf_extr != 0)
                    plate_extruders.push_back(glb_support_intf_extr);

                int obj_support_extr = 0;
                if (const ConfigOption* opt = object->config.option("support_filament"))
                    obj_support_extr = opt->getInt();
                if (obj_support_extr != 0)
                    plate_extruders.push_back(obj_support_extr);
                else if (glb_support_extr != 0)
                    plate_extruders.push_back(glb_support_extr);
            }
#ifdef ENGINE_SNAPMAKER
            slicer_cli::snapmaker_object_feature_filaments(*object, glb_wall_extr, glb_sparse_infill_extr,
                                                           glb_solid_infill_extr, plate_extruders);
#endif
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
            // Orca also lists wall / infill / surface filaments, with or
            // without support: the desktop's PartPlate::get_extruders, which
            // decides the plate's wipe tower (GLCanvas3D.cpp 2880), lists them
            // after its `if (obj_support)` block (PartPlate.cpp 1563-1640,
            // v2.4.2). The official CLI's get_extruders_under_cli puts them
            // behind `if (!obj_support) continue;` (1723-1801), so an object
            // without support would count one filament for the tower and the
            // arrange. Every key is `*_id` and 0 is "Default".
            int obj_outer_wall_extr = 0;
            if (const ConfigOption* opt = object->config.option("outer_wall_filament_id"))
                obj_outer_wall_extr = opt->getInt();
            if (obj_outer_wall_extr == 0)
                if (const ConfigOption* opt = object->config.option("inner_wall_filament_id"))
                    obj_outer_wall_extr = opt->getInt();
            if (obj_outer_wall_extr != 0)
                plate_extruders.push_back(obj_outer_wall_extr);
            else if (glb_outer_wall_extr != 0)
                plate_extruders.push_back(glb_outer_wall_extr);

            int obj_inner_wall_extr = 0;
            if (const ConfigOption* opt = object->config.option("inner_wall_filament_id"))
                obj_inner_wall_extr = opt->getInt();
            if (obj_inner_wall_extr == 0)
                if (const ConfigOption* opt = object->config.option("outer_wall_filament_id"))
                    obj_inner_wall_extr = opt->getInt();
            if (obj_inner_wall_extr != 0)
                plate_extruders.push_back(obj_inner_wall_extr);
            else if (glb_inner_wall_extr != 0)
                plate_extruders.push_back(glb_inner_wall_extr);

            int obj_sparse_infill_extr = 0;
            if (const ConfigOption* opt = object->config.option("sparse_infill_filament_id"))
                obj_sparse_infill_extr = opt->getInt();
            if (obj_sparse_infill_extr != 0)
                plate_extruders.push_back(obj_sparse_infill_extr);
            else if (glb_sparse_infill_extr != 0)
                plate_extruders.push_back(glb_sparse_infill_extr);

            int obj_internal_solid_extr = 0;
            if (const ConfigOption* opt = object->config.option("internal_solid_filament_id"))
                obj_internal_solid_extr = opt->getInt();
            if (obj_internal_solid_extr != 0)
                plate_extruders.push_back(obj_internal_solid_extr);
            else if (glb_internal_solid_extr != 0)
                plate_extruders.push_back(glb_internal_solid_extr);

            int obj_top_surface_extr = 0;
            if (const ConfigOption* opt = object->config.option("top_surface_filament_id"))
                obj_top_surface_extr = opt->getInt();
            if (obj_top_surface_extr == 0)
                obj_top_surface_extr = obj_internal_solid_extr;
            if (obj_top_surface_extr != 0)
                plate_extruders.push_back(obj_top_surface_extr);
            else if (glb_top_surface_extr != 0)
                plate_extruders.push_back(glb_top_surface_extr);

            int obj_bottom_surface_extr = 0;
            if (const ConfigOption* opt = object->config.option("bottom_surface_filament_id"))
                obj_bottom_surface_extr = opt->getInt();
            if (obj_bottom_surface_extr == 0)
                obj_bottom_surface_extr = obj_internal_solid_extr;
            if (obj_bottom_surface_extr != 0)
                plate_extruders.push_back(obj_bottom_surface_extr);
            else if (glb_bottom_surface_extr != 0)
                plate_extruders.push_back(glb_bottom_surface_extr);
#endif
        }
    }

    // conside_custom_gcode: the plate's tool changes.
#ifndef ENGINE_SNAPMAKER
    // Snapmaker's CLI passes conside_custom_gcode = false and only logs that it
    // skipped the scan, so its plate filament list never gains a tool-change
    // filament (references/SnapmakerOrca/src/slic3r/GUI/PartPlate.cpp:2083-2087
    // at bac6fd5).
    if (const auto* color_option = dynamic_cast<const ConfigOptionStrings*>(full_config.option("filament_colour"))) {
        const int nums_extruders = (int)color_option->values.size();
        auto it = model.plates_custom_gcodes.find(plate_index);
        if (it != model.plates_custom_gcodes.end()) {
            for (const auto& item : it->second.gcodes)
                if (item.type == CustomGCode::Type::ToolChange && item.extruder <= nums_extruders)
                    plate_extruders.push_back(item.extruder);
        }
    }
#endif

    std::sort(plate_extruders.begin(), plate_extruders.end());
    auto it_end = std::unique(plate_extruders.begin(), plate_extruders.end());
    plate_extruders.resize(std::distance(plate_extruders.begin(), it_end));

#ifdef ENGINE_BAMBU
    // Mixed filament slots count as their physical parts (PartPlate.cpp 1414-1426).
    if (auto* is_mixed_opt = full_config.option<ConfigOptionBools>("filament_is_mixed")) {
        if (auto* comp_strs_opt = full_config.option<ConfigOptionStrings>("filament_mixed_components")) {
            if (has_any_mixed_filament(is_mixed_opt->values)) {
                std::vector<unsigned int> ext_0based;
                for (int e : plate_extruders)
                    if (e >= 1) ext_0based.push_back((unsigned int)(e - 1));
                auto expanded = expand_mixed_filaments(ext_0based, is_mixed_opt->values, comp_strs_opt->values);
                plate_extruders.clear();
                for (unsigned int e : expanded)
                    plate_extruders.push_back((int)(e + 1));
            }
        }
    }
#endif
    return plate_extruders;
}

/// PartPlate::contain_instance_totally(obj, 0) before the arrange: the
/// instance is on the plate and PartPlate::check_outside finds it inside the
/// plate box and clear of every exclusion box (Bambu PartPlate.cpp 2597-2648;
/// Orca PartPlate.cpp 2529+). The official tests the instance's convex hull
/// against each exclusion box; this uses the hull's bounding box.
static bool arrange_instance_totally_inside(const Slic3r::ModelObject* object, const Slic3r::BoundingBoxf3& plate_box_in,
                                            const std::vector<Slic3r::BoundingBoxf>& exclude_boxes) {
    using namespace Slic3r;
    if (object->instances.empty())
        return false;
    const BoundingBoxf3 instance_box = object->instance_convex_hull_bounding_box(size_t(0));
    BoundingBoxf3 plate_box = plate_box_in;
    if (instance_box.max.z() > plate_box.min.z())
        plate_box.min.z() += instance_box.min.z(); // not considering outsize if sinking
    if (!plate_box.contains(instance_box))
        return false;
    for (const BoundingBoxf& box : exclude_boxes)
        if (box.min.x() < instance_box.max.x() && instance_box.min.x() < box.max.x() &&
            box.min.y() < instance_box.max.y() && instance_box.min.y() < box.max.y())
            return false;
    return true;
}

/// PartPlate::estimate_wipe_tower_size (Bambu PartPlate.cpp 2350-2440 at
/// v02.09.01.70, where the official command line passes legacy_behavior and
/// the desktop does not, BambuStudio.cpp 4883, 6246; Orca
/// PartPlate.cpp 2099-2165). The two engines differ in the tallest-object
/// measure (Bambu: every instance's top; Orca: the object's exact height) and
/// in the rib-wall settings (Bambu prime_tower_rib_wall / prime_tower_rib_width
/// / prime_tower_extra_rib_length; Orca wipe_tower_wall_type == Rib /
/// wipe_tower_rib_width / wipe_tower_extra_rib_length). The official reads the
/// filament-change length, filament diameter and extra rib length from the
/// plate's Print, which at arrange time is a fresh `new Print()` (PartPlateList
/// init, Bambu PartPlate.cpp 4024) holding the engine defaults; so do we.
Slic3r::Vec3d arrange_estimate_wipe_tower_size(const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                                      const double w, const double wipe_volume, int extruder_count,
                                                      int plate_extruder_size, bool enable_wrapping_detection,
                                                      const Slic3r::BoundingBoxf3& plate_box,
                                                      const std::vector<Slic3r::BoundingBoxf>& exclude_boxes,
                                                      const Slic3r::Model* global_model,
                                                      bool legacy_behavior) {
    using namespace Slic3r;
    Vec3d wipe_tower_size;
    double layer_height = 0.08f; // hard code layer height
    double max_height = 0.f;
    wipe_tower_size.setZero();
#ifdef ENGINE_SNAPMAKER
    // Snapmaker starts the width at the wipe volume, kept when no branch below
    // sets it (references/SnapmakerOrca/src/slic3r/GUI/PartPlate.cpp:2184 at bac6fd5).
    wipe_tower_size(0) = wipe_volume;
#endif

    if (const ConfigOption* layer_height_opt = config.option("layer_height"))
        layer_height = layer_height_opt->getFloat();

    // The official counts the plate's filaments again for an empty count
    // (get_extruders(true)); the caller's count already is that list.
    if (plate_extruder_size == 0)
        return wipe_tower_size;

    // use_global_objects (the assemble-list arrange): every object of the
    // run, on any plate, without the inside check.
    for (const ModelObject* mo : (global_model ? global_model->objects : model.objects)) {
        if (!global_model && !arrange_instance_totally_inside(mo, plate_box, exclude_boxes))
            continue;
#ifdef ENGINE_ORCA
        BoundingBoxf3 bbox = mo->bounding_box_exact();
        max_height = std::max(bbox.size().z(), max_height);
#else
        for (size_t i = 0; i < mo->instances.size(); ++i) {
            BoundingBoxf3 bbox = mo->instance_bounding_box(i);
            max_height         = std::max(bbox.max.z(), max_height);
        }
#endif
    }
    wipe_tower_size(2) = max_height;
#ifdef ENGINE_SNAPMAKER
    // Snapmaker rewrote this sizing (cli_snapmaker.hpp).
    const Vec2d footprint = slicer_cli::snapmaker_wipe_tower_footprint(
        config, w, wipe_volume, plate_extruder_size, layer_height,
        arrange_opt(config, "wipe_tower_rib_width")->getFloat());
    wipe_tower_size(0) = footprint.x();
    wipe_tower_size(1) = footprint.y();
    (void) extruder_count;
    (void) enable_wrapping_detection;
    return wipe_tower_size;
#else
    auto timelapse_type  = config.option<ConfigOptionEnum<TimelapseType>>("timelapse_type");
    bool need_wipe_tower = (timelapse_type ? (timelapse_type->value == TimelapseType::tlSmooth) : false) | enable_wrapping_detection;
    double extra_spacing = arrange_opt(config, "prime_tower_infill_gap")->getFloat() / 100.;
    const FullPrintConfig& print_defaults = FullPrintConfig::defaults();
#ifdef ENGINE_ORCA
    const auto* use_rib_wall_opt = config.option<ConfigOptionEnum<WipeTowerWallType>>("wipe_tower_wall_type");
    bool use_rib_wall = use_rib_wall_opt ? use_rib_wall_opt->value == WipeTowerWallType::wtwRib : false;
    double rib_width = arrange_opt(config, "wipe_tower_rib_width")->getFloat();
    const double extra_rib_length = print_defaults.wipe_tower_extra_rib_length.value;
#else
    const ConfigOptionBool* use_rib_wall_opt = config.option<ConfigOptionBool>("prime_tower_rib_wall");
    bool use_rib_wall = use_rib_wall_opt ? use_rib_wall_opt->value : true;
    double rib_width = arrange_opt(config, "prime_tower_rib_width")->getFloat();
    const double extra_rib_length = print_defaults.prime_tower_extra_rib_length.value;
#endif
    double depth;
    double filament_change_volume = 0.;
    {
        const std::vector<double>& filament_change_lengths = print_defaults.filament_change_length.values;
        double length = filament_change_lengths.empty() ? 0 : *std::max_element(filament_change_lengths.begin(), filament_change_lengths.end());
        double diameter = 1.75;
        const std::vector<double>& diameters = print_defaults.filament_diameter.values;
        diameter = diameters.empty() ? diameter : *std::max_element(diameters.begin(), diameters.end());
        filament_change_volume = length * PI * diameter * diameter / 4.;
    }
#ifdef ENGINE_ORCA
    (void) legacy_behavior;
    double volume = wipe_volume * (extruder_count == 2 ? plate_extruder_size : (plate_extruder_size - 1));
    if (extruder_count == 2) volume += filament_change_volume * (int) (plate_extruder_size / 2);
#else
    // Bambu PartPlate.cpp 2396-2409: the desktop sizes a one-colour plate with
    // smooth timelapse for one filament, and a printer whose nozzles count as
    // extruders rams at every nozzle change.
    const bool smooth_timelapse = timelapse_type ? (timelapse_type->value == TimelapseType::tlSmooth) : false;
    int filament_depth_count = extruder_count == 2 ? plate_extruder_size : (plate_extruder_size - 1);
    if (!legacy_behavior && plate_extruder_size == 1 && smooth_timelapse) filament_depth_count = 1;
    double volume = wipe_volume * filament_depth_count;
    if (extruder_count == 2) volume += filament_change_volume * (int) (plate_extruder_size / 2);
    if (const auto* hotend_as_ext = config.option<ConfigOptionBool>("handle_hotend_as_extruder");
        hotend_as_ext && hotend_as_ext->value && plate_extruder_size >= 2 && filament_change_volume > 0)
        volume += filament_change_volume * (plate_extruder_size - 1);
#endif
    if (use_rib_wall) {
        depth = std::sqrt(volume / layer_height * extra_spacing);
        if (need_wipe_tower || plate_extruder_size > 1) {
            float min_wipe_tower_depth = WipeTower::get_limit_depth_by_height(max_height);
            double volume_depth        = depth;
            depth = std::max((double) min_wipe_tower_depth, depth);
#ifndef ENGINE_ORCA
            // Slicing forces a wrapping tower to this depth (Bambu PartPlate.cpp 2420-2422).
            if (enable_wrapping_detection)
                depth = std::max(depth, WipeTower::wrapping_wipe_tower_depth);
#endif
            rib_width = std::min(rib_width, depth / 2);
            depth = rib_width / std::sqrt(2) + std::max(depth + extra_rib_length, volume_depth);
            wipe_tower_size(0) = wipe_tower_size(1) = depth;
        }
    } else {
        depth = volume / (layer_height * w) * extra_spacing;
        if (need_wipe_tower || depth > EPSILON) {
            float min_wipe_tower_depth = WipeTower::get_limit_depth_by_height(max_height);
            depth = std::max((double) min_wipe_tower_depth, depth);
#ifndef ENGINE_ORCA
            if (enable_wrapping_detection)   // Bambu PartPlate.cpp 2432-2434
                depth = std::max(depth, WipeTower::wrapping_wipe_tower_depth);
#endif
        }
        wipe_tower_size(0) = w;
        wipe_tower_size(1) = depth;
    }
    return wipe_tower_size;
#endif // ENGINE_SNAPMAKER
}

} // namespace slicer_cli
