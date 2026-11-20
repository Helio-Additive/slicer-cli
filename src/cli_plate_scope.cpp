#include "cli_plate_scope.hpp"

#include "cli_slice_state.hpp"

namespace slicer_cli {

PlateScope::PlateScope(Slic3r::Model& model, const RunPlate& plate) : m_model(model), m_origin(plate.origin) {
    m_whole = plate.all;
    if (!m_whole) {
        m_whole = true;
        for (const Slic3r::ModelObject* object : model.objects)
            for (const Slic3r::ModelInstance* inst : object->instances)
                m_whole = m_whole && plate.is_member(inst);
    }
    if (!m_whole) {
        m_all_objects = model.objects;
        std::vector<Slic3r::ModelObject*> kept;
        for (Slic3r::ModelObject* object : model.objects) {
            std::vector<Slic3r::ModelInstance*> mine;
            for (Slic3r::ModelInstance* inst : object->instances)
                if (plate.is_member(inst))
                    mine.push_back(inst);
            if (mine.empty())
                continue;
            m_instances[object] = object->instances;
            object->instances = mine;
            kept.push_back(object);
        }
        model.objects = kept;
    }
    for (Slic3r::ModelObject* object : model.objects)
        for (Slic3r::ModelInstance* inst : object->instances) {
            const Slic3r::Vec3d scene = inst->get_offset();
            const Slic3r::Vec3d local = scene - Slic3r::Vec3d(m_origin.x(), m_origin.y(), 0.);
            m_scene[inst->id().id] = scene;
            m_local[inst->id().id] = local;
            m_members.insert(inst);
            if (m_origin.x() != 0. || m_origin.y() != 0.)
                inst->set_offset(local);
        }
}
PlateScope::~PlateScope() {
    const Slic3r::Vec3d origin(m_origin.x(), m_origin.y(), 0.);
    for (Slic3r::ModelObject* object : m_model.objects) {
        for (Slic3r::ModelInstance* inst : object->instances) {
            const auto it = m_scene.find(inst->id().id);
            if (it == m_scene.end())
                inst->set_offset(inst->get_offset() + origin);   // added in the scope
            else if (inst->get_offset() != m_local[it->first])
                inst->set_offset(it->second + (inst->get_offset() - m_local[it->first]));
            else
                inst->set_offset(it->second);
        }
        object->invalidate_bounding_box();
    }
    if (m_whole)
        return;
    // The other plates' instances back in their places, the plate's own
    // (and any the step added) where they were.
    for (auto& [object, all] : m_instances) {
        std::vector<Slic3r::ModelInstance*> now = object->instances;
        std::vector<Slic3r::ModelInstance*> restored;
        for (Slic3r::ModelInstance* inst : all)
            if (!m_members.count(inst) || std::find(now.begin(), now.end(), inst) != now.end())
                restored.push_back(inst);
        for (Slic3r::ModelInstance* inst : now)
            if (std::find(all.begin(), all.end(), inst) == all.end())
                restored.push_back(inst);
        object->instances = restored;
    }
    std::vector<Slic3r::ModelObject*> now = m_model.objects;
    std::vector<Slic3r::ModelObject*> restored = m_all_objects;
    for (Slic3r::ModelObject* object : now)
        if (std::find(m_all_objects.begin(), m_all_objects.end(), object) == m_all_objects.end())
            restored.push_back(object);
    m_model.objects = restored;
}

/// The selected plate's own settings, as the official CLI lays them over the
/// project's per plate (see the call site): the plate's PlateData::config
/// without the filament-map keys (handled by the plate filament-map blocks
/// of each build: mode always, map in a manual mode) and without keys this
/// engine does not define. Empty
/// without a named plate (plate_id 0 is a whole-file load, not one plate).
Slic3r::DynamicPrintConfig plate_own_settings(const Slic3r::PlateDataPtrs& plate_data, int plate_id) {
    Slic3r::DynamicPrintConfig plate_config;
    if (plate_id <= 0 || (int)plate_data.size() < plate_id || plate_data[plate_id - 1] == nullptr)
        return plate_config;
    plate_config = plate_data[plate_id - 1]->config;
    for (const char* key : {"filament_map_mode", "filament_map", "filament_volume_map"})
        plate_config.erase(key);
    for (const std::string& key : plate_config.keys())
        if (Slic3r::print_config_def.get(key) == nullptr)
            plate_config.erase(key);
    return plate_config;
}

} // namespace slicer_cli
