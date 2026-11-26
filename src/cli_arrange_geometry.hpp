#pragma once

#include "libslic3r/Arrange.hpp"
#include "libslic3r/BoundingBox.hpp"
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
#include "libslic3r/GCode/WipeTowerEstimate.hpp"
#endif

namespace Slic3r { class Model; }

namespace slicer_cli {

const Slic3r::ConfigOption* arrange_opt(const Slic3r::DynamicPrintConfig& config, const std::string& key);
std::vector<Slic3r::BoundingBoxf> arrange_exclude_boxes(const Slic3r::DynamicPrintConfig& config);
void arrange_add_exclude_areas(Slic3r::arrangement::ArrangePolygons& out,
                               const Slic3r::DynamicPrintConfig& config, double inflation,
                               int num_beds = 1);
/// `expand_mixed_slots` false lists a mixed filament as its own slot, not as
/// its physical parts (get_extruders_under_cli's third argument at f92a7c0).
std::vector<int> arrange_plate_extruders(const Slic3r::Model& model,
                                                const Slic3r::DynamicPrintConfig& full_config, int plate_index,
                                                bool expand_mixed_slots = true);
Slic3r::Vec3d arrange_estimate_wipe_tower_size(const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                                      const double w, const double wipe_volume, int extruder_count,
                                                      int plate_extruder_size, bool enable_wrapping_detection,
                                                      const Slic3r::BoundingBoxf3& plate_box,
                                                      const std::vector<Slic3r::BoundingBoxf>& exclude_boxes,
                                                      const Slic3r::Model* global_model = nullptr,
                                                      bool legacy_behavior = true);
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
/// OrcaSlicer from 2.5: the prime tower a plate needs before it is sliced
/// (PartPlate::estimate_wipe_tower_footprint, PartPlate.cpp 3974-4026 at
/// f92a7c0). `plate_extruders` are the plate's filaments (1-based);
/// `plate_extruder_size` is a floor: the lowest ids not on the plate are added
/// up to that count. `global_model` (the assemble-list arrange) sizes for every
/// object of the run without the inside check.
Slic3r::WipeTowerFootprint plate_tower_footprint(const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                                 std::vector<int> plate_extruders, int plate_extruder_size,
                                                 const Slic3r::BoundingBoxf3& plate_box,
                                                 const std::vector<Slic3r::BoundingBoxf>& exclude_boxes,
                                                 const Slic3r::Model* global_model = nullptr);
#endif

} // namespace slicer_cli
