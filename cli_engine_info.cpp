// cli_engine_info.cpp — see cli_engine_info.hpp.
#include "cli_engine_info.hpp"

#include <map>

namespace slicer_cli {
namespace {

using json = nlohmann::json;

// The value of attribute `name` in the text of one XML tag, or "".
std::string attr(const std::string& tag, const std::string& name) {
    const std::string needle = " " + name + "=\"";
    const size_t at = tag.find(needle);
    if (at == std::string::npos) return {};
    const size_t start = at + needle.size();
    const size_t end   = tag.find('"', start);
    return end == std::string::npos ? std::string() : tag.substr(start, end - start);
}

json integer_or_null(const std::string& text) {
    if (text.empty()) return nullptr;
    try {
        size_t pos = 0;
        const long long v = std::stoll(text, &pos);
        return pos == text.size() ? json(v) : json(nullptr);
    } catch (...) {
        return nullptr;
    }
}

} // namespace

json project_objects(const std::string& xml) {
    // Pass one: every <object id> and its name (the object's own
    // <metadata key="name">, not a part's). Pass two: each <plate>'s
    // <model_instance> entries, in file order.
    std::map<std::string, std::string> names;
    json objects = json::array();
    std::string current_object;
    bool in_part = false, in_instance = false;
    int plate = 0;
    std::string object_id, instance_id, identify_id;
    size_t pos = 0;
    while (pos < xml.size()) {
        const size_t lt = xml.find('<', pos);
        if (lt == std::string::npos) break;
        if (xml.compare(lt, 4, "<!--") == 0) {
            const size_t end = xml.find("-->", lt + 4);
            if (end == std::string::npos) break;
            pos = end + 3;
            continue;
        }
        const size_t gt = xml.find('>', lt);
        if (gt == std::string::npos) break;
        const std::string tag = xml.substr(lt + 1, gt - lt - 1);
        pos = gt + 1;
        const auto is = [&](const char* name) {
            const size_t n = std::string(name).size();
            return tag.compare(0, n, name) == 0 &&
                   (tag.size() == n || tag[n] == ' ' || tag[n] == '/' || tag[n] == '\t' || tag[n] == '\n' || tag[n] == '\r');
        };
        if (is("object"))              { current_object = attr(tag, "id"); in_part = false; }
        else if (is("/object"))        { current_object.clear(); }
        else if (is("part"))           { in_part = true; }
        else if (is("/part"))          { in_part = false; }
        else if (is("plate"))          { ++plate; }
        else if (is("model_instance")) { in_instance = true; object_id.clear(); instance_id.clear(); identify_id.clear(); }
        else if (is("/model_instance")) {
            in_instance = false;
            objects.push_back(json{{"id", integer_or_null(identify_id)},
                                   {"object", integer_or_null(object_id)},
                                   {"instance", integer_or_null(instance_id)},
                                   {"plate", plate},
                                   {"name", object_id}});   // replaced by the name below
        } else if (is("metadata")) {
            const std::string key = attr(tag, "key"), value = attr(tag, "value");
            if (in_instance) {
                if (key == "object_id")        object_id   = value;
                else if (key == "instance_id") instance_id = value;
                else if (key == "identify_id") identify_id = value;
            } else if (!current_object.empty() && !in_part && key == "name") {
                names[current_object] = value;
            }
        }
    }
    for (json& o : objects) {
        const auto it = names.find(o["name"].get<std::string>());
        o["name"] = it == names.end() ? json(nullptr) : json(it->second);
    }
    return objects;
}

} // namespace slicer_cli
