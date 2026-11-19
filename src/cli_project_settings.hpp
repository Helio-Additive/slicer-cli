#pragma once

#include <map>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace Slic3r { class DynamicPrintConfig; }

namespace slicer_cli {

#ifdef ENGINE_ORCA
/// The raw settings one scope of a project states: the project itself, an
/// object or a part. Values keep the file's own words (json strings).
using RawValues = std::map<std::string, std::string>;

/// The settings one object of a project 3MF states, and each of its parts', in
/// the order the file lists them. bbs_3mf reads the same member into the
/// model's object and volume configs (bbs_3mf.cpp 2130, 5115, 5263 at 31f6803),
/// but the 3MF's numeric ids are not kept on the loaded ModelObject or
/// ModelVolume, so the file's own names are what the loaded model is matched
/// by (they are the same strings: each metadata "name" is assigned to the
/// object and to its volume).
struct FileObjectSettings {
    std::string                                         name;
    RawValues                                           values;
    std::vector<std::pair<std::string, RawValues>>      parts;
};

/// Every object's settings of a project 3MF, in the order the file lists them.
std::vector<FileObjectSettings> project_object_settings(const std::string& path);

bool maker_changed_key(const Slic3r::DynamicPrintConfig& file_config, const char* key);
bool file_states_number(const nlohmann::json& settings, const char* key, double value);
bool file_states_text(const nlohmann::json& settings, const char* key, const char* text);
#endif

nlohmann::json project_settings_json(const std::string& path);

} // namespace slicer_cli
