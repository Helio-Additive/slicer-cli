#include "cli_project_settings.hpp"

#include "cli_project_archive.hpp"
#include "libslic3r/PrintConfig.hpp"

namespace slicer_cli {

using json = nlohmann::json;

#ifdef ENGINE_ORCA
/// One attribute's value in an XML start tag ("key=\"value\"").
static std::string xml_attr(const std::string& tag, const std::string& name) {
    const std::string needle = name + "=\"";
    const size_t      at     = tag.find(needle);
    if (at == std::string::npos)
        return {};
    const size_t start = at + needle.size();
    const size_t end   = tag.find('"', start);
    return end == std::string::npos ? std::string() : tag.substr(start, end - start);
}

/// Metadata/model_settings.config of a project 3MF, by object and part id. The
/// member states each object's and each part's own settings as
/// <metadata key="…" value="…"/> inside <object id="…"> and <part id="…">
/// (bbs_3mf.cpp parses the same tags at 2130, 5115, 5263 at 31f6803).
std::vector<FileObjectSettings> project_object_settings(const std::string& path) {
    std::vector<FileObjectSettings> out;
    std::string                     text;
    if (!read_zip_member(path, "Metadata/model_settings.config", text))
        return out;
    size_t pos = 0;
    while (pos < text.size()) {
        const size_t lt = text.find('<', pos);
        if (lt == std::string::npos)
            break;
        const size_t gt = text.find('>', lt);
        if (gt == std::string::npos)
            break;
        const std::string tag = text.substr(lt + 1, gt - lt - 1);
        pos                   = gt + 1;
        if (tag.rfind("object", 0) == 0) {
            out.emplace_back();
        } else if (tag.rfind("/object", 0) == 0) {
            if (!out.empty())
                out.back().parts.shrink_to_fit();
        } else if (tag.rfind("part", 0) == 0) {
            if (!out.empty())
                out.back().parts.emplace_back();
        } else if (tag.rfind("metadata", 0) == 0 && !out.empty()) {
            const std::string key   = xml_attr(tag, "key");
            const std::string value = xml_attr(tag, "value");
            if (key.empty())
                continue;
            if (!out.back().parts.empty()) {
                auto& part = out.back().parts.back();
                if (key == "name")
                    part.first = value;
                else
                    part.second[key] = value;
            } else if (key == "name") {
                out.back().name = value;
            } else {
                out.back().values[key] = value;
            }
        }
    }
    return out;
}

#endif
/// Metadata/project_settings.config of a project 3MF, parsed. A null json when
/// the file has none or it cannot be read.
json project_settings_json(const std::string& path) {
    std::string text;
    if (!read_zip_member(path, "Metadata/project_settings.config", text))
        return json();
    try {
        const json settings = json::parse(text);
        return settings.is_object() ? settings : json();
    } catch (...) {
        return json();
    }
}
#ifdef ENGINE_ORCA

/// True when the maker listed `key` among the keys it changed. The desktop (and
/// this command line) keep those keys' file values and take the rest from the
/// system preset (Preset.cpp 2446-2457), so this is the test for "the file
/// states this value on purpose".
bool maker_changed_key(const Slic3r::DynamicPrintConfig& file_config, const char* key) {
    const auto* diff = file_config.option<Slic3r::ConfigOptionStrings>("different_settings_to_system");
    if (diff == nullptr || diff->values.empty())
        return false;
    std::vector<std::string> keys;
    Slic3r::unescape_strings_cstyle(diff->values.front(), keys);
    return std::find(keys.begin(), keys.end(), key) != keys.end();
}

/// True when the file states `value` for `key` (as a string or a number).
bool file_states_number(const json& settings, const char* key, double value) {
    const auto it = settings.find(key);
    if (it == settings.end())
        return false;
    if (it->is_string())
        return it->get<std::string>() == std::to_string(int(value));
    if (it->is_number())
        return it->get<double>() == value;
    return false;
}

/// True when the file states `text` for `key` (a setting's own word).
bool file_states_text(const json& settings, const char* key, const char* text) {
    const auto it = settings.find(key);
    return it != settings.end() && it->is_string() && it->get<std::string>() == text;
}
#endif

} // namespace slicer_cli
