// cli_engine_info.hpp — the objects of a project 3MF, for --engine-info.
//
// --skip-objects and --simplify-objects take the id the engine's loader gives
// each placed object: the plate's model_instance identify_id in
// Metadata/model_settings.config (bbs_3mf.cpp 2443 at 56a57d0 sets
// ModelInstance::loaded_id from it). Nothing else prints that id, so
// --engine-info lists it beside the object's name and plate, and a caller can
// name an object before it slices.
#pragma once

#include <set>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace slicer_cli {

/// The placed objects a project's Metadata/model_settings.config declares,
/// in plate order: {id, object, instance, plate, name}. `id` is null when the
/// file states no identify_id (the loader then gives the instance no id, and
/// the object flags cannot name it).
nlohmann::json project_objects(const std::string& model_settings_xml);

/// The ids of every placed object the project declares, on every plate: the
/// set --skip-objects and --simplify-objects are checked against, so an id on
/// another plate than the one being sliced is a no-op, not a mistake.
std::set<int> project_object_ids(const std::string& model_settings_xml);

/// The sentence that refuses `flag` when one of `wanted` is an id `known` has
/// not (CLI_INVALID_PARAMS), or "" when every id is known. `known` is the
/// file's ids (project_object_ids) for a project 3MF, else the loaded model's.
std::string unknown_object_ids_refusal(const char* flag, const std::vector<int>& wanted,
                                       const std::set<int>& known);

} // namespace slicer_cli
