// cli_engine_info.hpp — the objects of a project 3MF, for --engine-info.
//
// --skip-objects and --simplify-objects take the id the engine's loader gives
// each placed object: the plate's model_instance identify_id in
// Metadata/model_settings.config (bbs_3mf.cpp 2443 at 56a57d0 sets
// ModelInstance::loaded_id from it). Nothing else prints that id, so
// --engine-info lists it beside the object's name and plate, and a caller can
// name an object before it slices.
#pragma once

#include <string>
#include <nlohmann/json.hpp>

namespace slicer_cli {

/// The placed objects a project's Metadata/model_settings.config declares,
/// in plate order: {id, object, instance, plate, name}. `id` is null when the
/// file states no identify_id (the loader then gives the instance no id, and
/// the object flags cannot name it).
nlohmann::json project_objects(const std::string& model_settings_xml);

} // namespace slicer_cli
