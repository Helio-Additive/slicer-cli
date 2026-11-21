#pragma once

#include "libslic3r/Arrange.hpp"
#include "libslic3r/BoundingBox.hpp"

namespace Slic3r { class Model; }

namespace slicer_cli {

const Slic3r::ConfigOption* arrange_opt(const Slic3r::DynamicPrintConfig& config, const std::string& key);
std::vector<Slic3r::BoundingBoxf> arrange_exclude_boxes(const Slic3r::DynamicPrintConfig& config);
void arrange_add_exclude_areas(Slic3r::arrangement::ArrangePolygons& out,
                               const Slic3r::DynamicPrintConfig& config, double inflation,
                               int num_beds = 1);
std::vector<int> arrange_plate_extruders(const Slic3r::Model& model,
                                                const Slic3r::DynamicPrintConfig& full_config, int plate_index);
Slic3r::Vec3d arrange_estimate_wipe_tower_size(const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                                      const double w, const double wipe_volume, int extruder_count,
                                                      int plate_extruder_size, bool enable_wrapping_detection,
                                                      const Slic3r::BoundingBoxf3& plate_box,
                                                      const std::vector<Slic3r::BoundingBoxf>& exclude_boxes,
                                                      const Slic3r::Model* global_model = nullptr,
                                                      bool legacy_behavior = true);

} // namespace slicer_cli
