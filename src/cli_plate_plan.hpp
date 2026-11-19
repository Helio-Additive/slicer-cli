#pragma once

#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/BoundingBox.hpp"

struct CliOptions;

namespace slicer_cli {

struct CliRunContext;
/// A project 3MF followed by model files (`project.3mf part.stl`): what the
/// official CLI does with them, for both engines.
///  - Each file loads into its own model, and is_bbl_3mf is reset for every
///    file, so it ends false (BambuStudio.cpp 1871; OrcaSlicer.cpp 1553):
///    --slice N becomes --slice 0 (2192-2196; 1839-1843), and none of the
///    Bambu-project steps after the load run (the move to a new bed, the
///    clearance re-arrange, the project's settings rules).
///  - A model file sets need_arrange (BambuStudio.cpp 2077; OrcaSlicer.cpp
///    1724), which only --arrange 0 turns off (5066-5081; 4327-4342).
///  - The models merge into one (BambuStudio.cpp 3947-3963; OrcaSlicer.cpp
///    3441-3457) and, with need_arrange, every object of every plate is
///    arranged across the plates (the global arrange, BambuStudio.cpp
///    5627-5722, 5886-5926, 5946-6006; OrcaSlicer.cpp 4887-4983, 5142-5186,
///    5189-5262), which adds plates when they are full and drops empty ones
///    (PartPlateList::rebuild_plates_after_arrangement).
/// The plan is computed once on the run's one model (RunState), before the
/// plates are sliced, and each plate's slice takes its instances from it.
struct ProjectPlan {
    bool done = false;
    int file_plate_count = 0;
    int plate_count = 0;
    // Per plate: its instances in the run's model (every object stays at its
    // place in the scene, in the order the arrange left them), and its grid
    // origin.
    std::vector<std::vector<Slic3r::ModelInstance*>> members;
    std::vector<Slic3r::Vec2d> origins;
    // wipe_tower_x / wipe_tower_y as the arrange left them (empty when the
    // arrange did not set them).
    std::vector<double> tower_x, tower_y;
    // Instances on no plate (left off every plate, or not printable).
    int off_plates = 0;
    bool arranged = false;
};

Slic3r::Vec2d plate_grid_origin(const Slic3r::DynamicPrintConfig& file_config,
                                       const Slic3r::Semver& file_version, int index, int plate_count);
Slic3r::BoundingBoxf3 plate_scene_box(const Slic3r::DynamicPrintConfig& file_config,
                                             const Slic3r::Semver& file_version, int index, int plate_count);
bool trailing_models_port(const CliOptions& o);
bool plan_project_plates(CliRunContext& context, Slic3r::Model& model, Slic3r::DynamicPrintConfig& config,
                                const Slic3r::DynamicPrintConfig& file_config, const Slic3r::Semver& file_version,
                                const Slic3r::PlateDataPtrs& plate_data, int filament_count, bool need_arrange,
                                ProjectPlan& plan, PlateOutcome& outcome);
void emit_project_plan_event(const ProjectPlan& plan, bool model_files = true);

} // namespace slicer_cli
