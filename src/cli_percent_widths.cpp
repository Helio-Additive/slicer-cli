#include "cli_percent_widths.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/miniz_extension.hpp"
#include "cli_project_archive.hpp"

namespace slicer_cli {

using json = nlohmann::json;

#ifdef ENGINE_BAMBU
// ── Percent line widths on the Bambu build (cross-engine, same meaning) ──────
// OrcaSlicer declares the ten line-width options coFloatOrPercent with
// ratio_over = "nozzle_diameter" (Orca PrintConfig.cpp at 31f6803) and writes
// them as "100%"; BambuStudio declares the same keys plain coFloat, and its
// ConfigOptionFloat::deserialize reads "100%" as 100 mm, which the engine then
// rejects as "Too large line width" or prints with. The meaning is the same,
// only the unit differs, so the Bambu build converts: the percentage of the
// file's one nozzle diameter, in mm. A percentage that is not one number,
// or a project with more than one nozzle diameter, is refused naming the
// setting, never guessed.

static bool percent_line_width_is_engine_foreign(const std::string& key) {
    static const std::string suffix = "_line_width";
    const bool line_width = key == "line_width" ||
        (key.size() > suffix.size() && key.compare(key.size() - suffix.size(), suffix.size(), suffix) == 0);
    if (!line_width) return false;
    const Slic3r::ConfigOptionDef* def = Slic3r::print_config_def.get(key);
    return def != nullptr && def->type == Slic3r::coFloat;
}

enum class PercentShape { Plain, Percent, Malformed };

static PercentShape percent_text_shape(const std::string& text, double& ratio) {
    std::string t = boost::algorithm::trim_copy(text);
    if (t.empty() || t.back() != '%') return PercentShape::Plain;
    t.pop_back();
    boost::algorithm::trim(t);
    try {
        size_t used = 0;
        const double v = std::stod(t, &used);
        if (used != t.size() || !std::isfinite(v)) return PercentShape::Malformed;
        ratio = v;
        return PercentShape::Percent;
    } catch (...) {
        return PercentShape::Malformed;
    }
}

/// A JSON line-width value: a string, or an array of strings that must agree.
static PercentShape percent_json_shape(const json& value, double& ratio) {
    std::vector<json> entries;
    if (value.is_array()) entries.assign(value.begin(), value.end());
    else entries.push_back(value);
    bool any_percent = false, any_plain = false;
    boost::optional<double> agreed;
    for (const json& e : entries) {
        double r = 0.;
        const PercentShape shape = e.is_string() ? percent_text_shape(e.get<std::string>(), r) : PercentShape::Plain;
        if (shape == PercentShape::Malformed) return PercentShape::Malformed;
        if (shape == PercentShape::Plain) { any_plain = true; continue; }
        any_percent = true;
        if (agreed && *agreed != r) return PercentShape::Malformed;
        agreed = r;
    }
    if (!any_percent) return PercentShape::Plain;
    if (any_plain) return PercentShape::Malformed;
    ratio = *agreed;
    return PercentShape::Percent;
}

/// The one nozzle diameter a project states (blank entries skipped), or none
/// when it states none or two different ones (bound_nozzle_diameter_mm).
static boost::optional<double> bound_nozzle_diameter_mm(const json& settings) {
    if (!settings.contains("nozzle_diameter")) return boost::none;
    const json& stated = settings["nozzle_diameter"];
    std::vector<json> entries;
    if (stated.is_array()) entries.assign(stated.begin(), stated.end());
    else entries.push_back(stated);
    boost::optional<double> diameter;
    for (const json& e : entries) {
        double v = 0.;
        if (e.is_null()) continue;
        if (e.is_number()) v = e.get<double>();
        else if (e.is_string()) {
            const std::string t = boost::algorithm::trim_copy(e.get<std::string>());
            if (t.empty()) continue;
            try { size_t used = 0; v = std::stod(t, &used); if (used != t.size()) return boost::none; }
            catch (...) { return boost::none; }
        } else return boost::none;
        if (!std::isfinite(v) || v <= 0.) return boost::none;
        if (diameter && *diameter != v) return boost::none;
        diameter = v;
    }
    return diameter;
}

static std::string width_text(double width) {
    std::ostringstream s;
    s.precision(10);
    s << width;
    return s.str();
}

/// One `<metadata key="K" value="V"/>` per line-width key in
/// model_settings.config (per-object and per-part overrides): the value
/// attribute is rewritten, every other byte is kept.
static bool rewrite_metadata_percents(std::string& xml, double nozzle_mm, json& converted,
                                      std::string& refusal, bool apply) {
    bool found = false;
    size_t pos = 0;
    while ((pos = xml.find("<metadata", pos)) != std::string::npos) {
        const size_t end = xml.find('>', pos);
        if (end == std::string::npos) break;
        const size_t k = xml.find("key=\"", pos);
        if (k == std::string::npos || k > end) { pos = end; continue; }
        const size_t k_end = xml.find('"', k + 5);
        const std::string key = xml.substr(k + 5, k_end - k - 5);
        const size_t v = xml.find("value=\"", pos);
        if (v == std::string::npos || v > end || !percent_line_width_is_engine_foreign(key)) { pos = end; continue; }
        const size_t v_start = v + 7;
        const size_t v_end = xml.find('"', v_start);
        if (v_end == std::string::npos || v_end > end) { pos = end; continue; }
        const std::string value = xml.substr(v_start, v_end - v_start);
        double ratio = 0.;
        const PercentShape shape = percent_text_shape(value, ratio);
        if (shape == PercentShape::Plain) { pos = end; continue; }
        if (shape == PercentShape::Malformed) {
            refusal = "Setting '" + key + "' is '" + value + "', which is not one line width.";
            return found;
        }
        found = true;
        if (apply) {
            const double width = ratio / 100. * nozzle_mm;
            if (!std::isfinite(width) || width <= 0.) {
                refusal = "Setting '" + key + "' is '" + value + "', which is not a printable line width.";
                return found;
            }
            const std::string written = width_text(width);
            converted.push_back(json{{"key", key}, {"scope", "object"}, {"from", value}, {"to_mm", width}});
            xml.replace(v_start, v_end - v_start, written);
            pos = v_start + written.size();
        } else {
            pos = end;
        }
    }
    return found;
}

/// Converts the percent line widths of a 3MF into mm in a temporary copy
/// (same file name, so [input_filename_base] in custom G-code is unchanged).
/// A file without percentages is not copied at all.
void rewrite_percent_line_widths(const std::string& input, PercentRewrite& out) {
    std::string settings_text, model_settings;
    if (!read_zip_member(input, "Metadata/project_settings.config", settings_text))
        return;
    json settings;
    try { settings = json::parse(settings_text); } catch (...) { return; }
    if (!settings.is_object()) return;
    const bool has_model_settings = read_zip_member(input, "Metadata/model_settings.config", model_settings);

    std::vector<std::pair<std::string, double>> globals;
    for (auto& [key, value] : settings.items()) {
        if (!percent_line_width_is_engine_foreign(key)) continue;
        double ratio = 0.;
        const PercentShape shape = percent_json_shape(value, ratio);
        if (shape == PercentShape::Malformed) {
            out.refusal = "Setting '" + key + "' is " + value.dump() + ", which is not one line width.";
            return;
        }
        if (shape == PercentShape::Percent) globals.emplace_back(key, ratio);
    }
    json scratch = json::array();
    bool object_percents = false;
    if (has_model_settings) {
        object_percents = rewrite_metadata_percents(model_settings, 0., scratch, out.refusal, false);
        if (!out.refusal.empty()) return;
    }
    if (globals.empty() && !object_percents) return;

    const boost::optional<double> nozzle = bound_nozzle_diameter_mm(settings);
    const std::string first_key = globals.empty() ? std::string("a per-object line width") : "'" + globals.front().first + "'";
    if (!nozzle) {
        out.refusal = "Setting " + first_key + " is a percentage of the nozzle, and this project states " +
            (settings.contains("nozzle_diameter") ? "more than one nozzle diameter (" + settings["nozzle_diameter"].dump() + ")"
                                                 : std::string("no nozzle diameter")) +
            "; a percentage cannot be turned into one width.";
        return;
    }
    out.nozzle_mm = *nozzle;
    for (const auto& [key, ratio] : globals) {
        const double width = ratio / 100. * *nozzle;
        if (!std::isfinite(width) || width <= 0.) {
            out.refusal = "Setting '" + key + "' is " + settings[key].dump() + ", which is not a printable line width.";
            return;
        }
        out.converted.push_back(json{{"key", key}, {"scope", "project"}, {"from", settings[key]}, {"to_mm", width}});
        settings[key] = width_text(width);
    }
    if (has_model_settings) {
        rewrite_metadata_percents(model_settings, *nozzle, out.converted, out.refusal, true);
        if (!out.refusal.empty()) return;
    }

    // Write the copy: every member as stored, the two settings members rewritten.
    out.temp_dir = make_owned_temp_dir("slicer_cli_percent-");
    out.temp_path = (out.temp_dir / boost::filesystem::path(input).filename()).string();
    const std::string new_settings = settings.dump(4);
    mz_zip_archive reader, writer;
    mz_zip_zero_struct(&reader);
    mz_zip_zero_struct(&writer);
    if (!Slic3r::open_zip_reader(&reader, input))
        throw std::runtime_error("cannot reopen " + input);
    if (!Slic3r::open_zip_writer(&writer, out.temp_path)) {
        Slic3r::close_zip_reader(&reader);
        throw std::runtime_error("cannot write " + out.temp_path);
    }
    bool ok = true;
    const mz_uint count = mz_zip_reader_get_num_files(&reader);
    for (mz_uint i = 0; i < count && ok; ++i) {
        mz_zip_archive_file_stat stat;
        if (!mz_zip_reader_file_stat(&reader, i, &stat)) { ok = false; break; }
        std::string name(stat.m_filename);
        std::string norm = name;
        std::replace(norm.begin(), norm.end(), '\\', '/');
        if (boost::algorithm::iequals(norm, "Metadata/project_settings.config"))
            ok = mz_zip_writer_add_mem(&writer, name.c_str(), new_settings.data(), new_settings.size(), MZ_DEFAULT_COMPRESSION);
        else if (has_model_settings && boost::algorithm::iequals(norm, "Metadata/model_settings.config"))
            ok = mz_zip_writer_add_mem(&writer, name.c_str(), model_settings.data(), model_settings.size(), MZ_DEFAULT_COMPRESSION);
        else
            ok = mz_zip_writer_add_from_zip_reader(&writer, &reader, i);
    }
    ok = ok && mz_zip_writer_finalize_archive(&writer);
    Slic3r::close_zip_writer(&writer);
    Slic3r::close_zip_reader(&reader);
    if (!ok)
        throw std::runtime_error("cannot write the converted copy " + out.temp_path);
    out.changed = true;
}
#endif // ENGINE_BAMBU

} // namespace slicer_cli
