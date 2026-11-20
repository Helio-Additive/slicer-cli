// cli_engine_info.cpp — see cli_engine_info.hpp.
#include "cli_engine_info.hpp"

#include <algorithm>
#include <cctype>
#include <map>

#include "libslic3r/Model.hpp"

namespace slicer_cli {
namespace {

using json = nlohmann::json;

// The five XML entities and the numeric references, decoded; anything else
// is left as written (the file's own spelling is better than a guess).
std::string decode_entities(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (size_t i = 0; i < text.size(); ++i) {
        if (text[i] != '&') { out += text[i]; continue; }
        const size_t semi = text.find(';', i);
        if (semi == std::string::npos) { out += text.substr(i); break; }
        const std::string entity = text.substr(i + 1, semi - i - 1);
        if      (entity == "amp")  out += '&';
        else if (entity == "lt")   out += '<';
        else if (entity == "gt")   out += '>';
        else if (entity == "quot") out += '"';
        else if (entity == "apos") out += '\'';
        else if (entity.size() > 1 && entity[0] == '#') {
            try {
                const bool hex = entity[1] == 'x' || entity[1] == 'X';
                size_t pos = 0;
                const unsigned long code = std::stoul(entity.substr(hex ? 2 : 1), &pos, hex ? 16 : 10);
                if (pos != entity.size() - (hex ? 2 : 1) || code > 0x10FFFF) { out += text.substr(i, semi - i + 1); i = semi; continue; }
                // UTF-8
                if (code < 0x80) out += char(code);
                else if (code < 0x800) { out += char(0xC0 | (code >> 6)); out += char(0x80 | (code & 0x3F)); }
                else if (code < 0x10000) { out += char(0xE0 | (code >> 12)); out += char(0x80 | ((code >> 6) & 0x3F)); out += char(0x80 | (code & 0x3F)); }
                else { out += char(0xF0 | (code >> 18)); out += char(0x80 | ((code >> 12) & 0x3F)); out += char(0x80 | ((code >> 6) & 0x3F)); out += char(0x80 | (code & 0x3F)); }
            } catch (...) { out += text.substr(i, semi - i + 1); }
        } else out += text.substr(i, semi - i + 1);
        i = semi;
    }
    return out;
}

bool xml_space(char c) { return c == ' ' || c == '\t' || c == '\n' || c == '\r'; }

// The value of attribute `name` in the text of one XML tag, decoded, or "".
// The tag's attributes are walked as XML spells them (whitespace of any kind,
// either quote, whitespace around '='), so text inside a quoted value never
// passes for an attribute name.
std::string attr(const std::string& tag, const std::string& name) {
    size_t pos = 0;
    while (pos < tag.size() && !xml_space(tag[pos])) ++pos;   // the tag name
    while (pos < tag.size()) {
        while (pos < tag.size() && xml_space(tag[pos])) ++pos;
        const size_t name_start = pos;
        while (pos < tag.size() && !xml_space(tag[pos]) && tag[pos] != '=' && tag[pos] != '/' && tag[pos] != '>') ++pos;
        const std::string attribute = tag.substr(name_start, pos - name_start);
        while (pos < tag.size() && xml_space(tag[pos])) ++pos;
        if (pos >= tag.size() || tag[pos] != '=') { if (attribute.empty()) ++pos; continue; }
        ++pos;
        while (pos < tag.size() && xml_space(tag[pos])) ++pos;
        if (pos >= tag.size() || (tag[pos] != '"' && tag[pos] != '\'')) return {};
        const char   quote = tag[pos];
        const size_t end   = tag.find(quote, pos + 1);
        if (end == std::string::npos) return {};
        if (attribute == name) return decode_entities(tag.substr(pos + 1, end - pos - 1));
        pos = end + 1;
    }
    return {};
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
        // The tag ends at the first '>' outside a quoted attribute value: a
        // literal '>' is valid inside one ("Left > Right").
        size_t gt = lt + 1;
        for (char quote = 0; gt < xml.size(); ++gt) {
            const char c = xml[gt];
            if (quote) { if (c == quote) quote = 0; }
            else if (c == '"' || c == '\'') quote = c;
            else if (c == '>') break;
        }
        if (gt >= xml.size()) break;
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

std::set<int> project_object_ids(const std::string& xml) {
    std::set<int> ids;
    for (const json& o : project_objects(xml))
        if (o["id"].is_number_integer())
            ids.insert(o["id"].get<int>());
    return ids;
}

std::string unknown_object_ids_refusal(const char* flag, const std::vector<int>& wanted,
                                       const std::set<int>& known) {
    std::set<int> missing;
    for (int id : wanted)
        if (!known.count(id))
            missing.insert(id);
    if (missing.empty())
        return {};
    const auto list = [](const std::set<int>& values) {
        std::string out;
        for (int v : values) out += (out.empty() ? "" : ", ") + std::to_string(v);
        return out;
    };
    // A plain model file gives its objects no id (loaded_id 0): the flags
    // cannot name them, and the sentence says so instead of listing "0".
    const bool no_ids = known.empty() || (known.size() == 1 && *known.begin() == 0);
    if (no_ids)
        return std::string(flag) + ": no object has id " + list(missing) +
               "; this file gives its objects no ids (a plain model file). The flag takes the ids --engine-info "
               "lists for a project 3MF";
    return std::string(flag) + ": no object has id " + list(missing) + "; the file's object ids are " + list(known);
}

std::set<int> known_object_ids(const std::vector<std::string>& inputs, const Slic3r::Model& model,
                               const ZipMemberReader& read_member) {
    std::set<int> known;
    for (const std::string& input : inputs) {
        std::string extension = input.size() >= 4 ? input.substr(input.size() - 4) : std::string();
        for (char& c : extension) c = char(std::tolower(static_cast<unsigned char>(c)));
        if (std::string model_settings; extension == ".3mf" && read_member(input, "Metadata/model_settings.config", model_settings))
            for (int id : project_object_ids(model_settings))
                known.insert(id);
    }
    // The loaded objects too (a model file beside the project, a 3MF without
    // the member); the loader's 0 for "no id" only when the run has no project
    // ids at all, so a plain model file is told so.
    const bool project_ids = !known.empty();
    for (const Slic3r::ModelObject* obj : model.objects)
        for (const Slic3r::ModelInstance* inst : obj->instances)
            if (inst->loaded_id > 0 || !project_ids)
                known.insert(int(inst->loaded_id));
    return known;
}

} // namespace slicer_cli
