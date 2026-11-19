#pragma once

#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Point.hpp"
#include <map>
#include <set>
#include <vector>

namespace Slic3r { class ModelInstance; }

namespace slicer_cli {

struct RunPlate;

/// One plate of the run's model in its own frame, for the steps the official
/// CLI runs on one plate in the plate's coordinates (the move to a new bed,
/// BambuStudio.cpp 4487-4533; the one-plate arrange, 5730-6176, with
/// PartPlateList::preprocess_arrange_polygon taking the plate origin off and
/// postprocess_arrange_polygon putting it back, PartPlate.cpp 5831-5871,
/// 6107-6129; the bed check against the plate's shape, 6527): while the
/// scope lives the model holds only this plate's instances, each moved by
/// minus the plate's origin. On exit every instance goes back to the scene
/// with the move the step made (an instance the step did not move keeps its
/// exact offset), the other instances come back, and instances or objects
/// the step added (--repetitions) are kept, on the plate. Instances are known
/// by their ObjectID, which a copy of the model keeps (Model::assign_copy), so
/// a step that puts back a saved copy of the model finds its instances again.
/// Only a scope that holds the whole model may let its step replace the model's
/// objects: nothing this model reaches through a scope may replace them (the
/// repetitions search runs on a copy of the model for that reason,
/// cli_repetitions.cpp).
class PlateScope {
public:
    PlateScope(Slic3r::Model& model, const RunPlate& plate);
    ~PlateScope();
    PlateScope(const PlateScope&) = delete;
    PlateScope& operator=(const PlateScope&) = delete;

private:
    Slic3r::Model& m_model;
    Slic3r::Vec2d m_origin;
    bool m_whole = true;
    std::vector<Slic3r::ModelObject*> m_all_objects;
    std::map<Slic3r::ModelObject*, std::vector<Slic3r::ModelInstance*>> m_instances;
    std::set<const Slic3r::ModelInstance*> m_members;
    std::map<size_t, Slic3r::Vec3d> m_scene, m_local;
};

Slic3r::DynamicPrintConfig plate_own_settings(const Slic3r::PlateDataPtrs& plate_data, int plate_id);

} // namespace slicer_cli
