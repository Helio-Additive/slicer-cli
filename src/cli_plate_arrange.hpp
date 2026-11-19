#pragma once

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Arrange.hpp"
#include "libslic3r/PrintConfig.hpp"
#include <string>

struct CliOptions;
namespace Slic3r { class DynamicPrintConfig; class Model; class ModelObject; }

namespace slicer_cli {

struct CliRunContext;
struct PlateOutcome;
struct ProjectFacts;
std::string mm_text(double v);
Slic3r::BoundingBoxf3 object_world_bbox(const Slic3r::ModelObject* object);
bool check_objects_inside_bed(Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                     PlateOutcome& outcome, bool arranged = false);
Slic3r::Vec3d downward_plate_size(CliRunContext& context, const Slic3r::Model& model, const Slic3r::DynamicPrintConfig& config,
                                         int plate_index, bool is_sequence);
int move_to_new_bed(CliRunContext& context, const slicer_cli::ProjectFacts& facts, bool machine_switch, Slic3r::Model& model,
                           Slic3r::DynamicPrintConfig& config, int plate_index, bool is_sequence, Slic3r::Vec3d& moved);
Slic3r::BoundingBoxf tower_reach_box(const Slic3r::DynamicPrintConfig& config,
                                            const Slic3r::ConfigOptionPoints& area);
bool arrange_on_bed(CliRunContext& context, Slic3r::Model& model, Slic3r::DynamicPrintConfig& config,
                           int plate_index, size_t sliced_filament_count, PlateOutcome& outcome,
                           int* landed_out = nullptr);
Slic3r::Vec2d tower_default_corner(CliRunContext& context, const Slic3r::DynamicPrintConfig& config);
void assemble_tower_default_corner(CliRunContext& context, Slic3r::DynamicPrintConfig& config, int plate_index, float margin);
/// The values the tower sizing reads: the tower width, the wipe volume, the
/// printer's nozzle count and whether wrapping detection is on. Orca takes the
/// volume from prime_volume; Bambu the largest filament_prime_volume, 15 when
/// the mode saves (Orca PartPlate.cpp 4150-4155; Bambu PartPlate.cpp
/// 4666-4673).
struct TowerWipeParams {
    float  width          = 0.f;   // prime_tower_width
    double volume         = 0.;    // prime_volume / max filament_prime_volume
    int    extruder_count = 1;     // one per nozzle_diameter entry
    bool   wrapping       = false; // enable_wrapping_detection
};

TowerWipeParams tower_wipe_params(const Slic3r::DynamicPrintConfig& config);
void assemble_tower_inside_plate(CliRunContext& context, Slic3r::DynamicPrintConfig& config, const Slic3r::Model& model,
                                        int plate_index, bool keep_existing);
Slic3r::arrangement::ArrangePolygon estimate_tower_polygon_global(const Slic3r::Model& global_model,
                                                                        const Slic3r::DynamicPrintConfig& config,
                                                                        int plate_index, int filaments_count,
                                                                        int plate_width, int plate_depth,
                                                                        Slic3r::Vec2d& pos);
bool arrange_assemble_plate(CliRunContext& context, Slic3r::Model& model, Slic3r::DynamicPrintConfig& config, int plate_index,
                                   int filaments_count, const Slic3r::Model& global_model, PlateOutcome& outcome);

} // namespace slicer_cli
