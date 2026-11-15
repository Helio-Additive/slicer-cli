// The environment NozzleInventory.cpp is compiled in by gen.cpp: the few types and functions of
// libslic3r it uses, reduced to what it reads. The functions marked "verbatim" are copied from
// Bambu Studio at the pin (the file and lines are given), so the reference output follows the
// engine's own code; the config and preset classes are minimal stand-ins with the same members.
#pragma once

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <boost/algorithm/string.hpp>

#include "fast_float/fast_float.h"

namespace Slic3r {

// ---- Log.hpp / Log.cpp: the records are captured instead of written ----

struct LogRecord { std::string level, message; };
inline std::vector<LogRecord> g_log_records;

struct LogCapture
{
    std::string        level;
    std::ostringstream buf;
    explicit LogCapture(const char *lvl) : level(lvl) { buf.setf(std::ios_base::boolalpha); }
    ~LogCapture() { g_log_records.push_back({level, buf.str()}); }
    template<class T> LogCapture &operator<<(const T &value) { buf << value; return *this; }
};
#define BOOST_LOG_TRIVIAL(lvl) ::Slic3r::LogCapture(#lvl)

namespace log_key { inline constexpr std::string_view extruder_id = "extruder_id"; }

// Log.cpp:214-234, verbatim
inline std::string quoted_if_needed(const std::string& value)
{
    if (!value.empty() && value.find_first_of(" =\"\t\r\n") == std::string::npos)
        return value;

    std::string out;
    out.reserve(value.size() + 2);
    out += '"';
    for (char c : value) {
        switch (c) {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n";  break;
        case '\r': out += "\\r";  break;
        case '\t': out += "\\t";  break;
        default:   out += c;
        }
    }
    out += '"';
    return out;
}

// Log.hpp:203-215 with Log.cpp:237-244; extruder_id is not a masked key
template<class T> std::string KV(std::string_view key, const T &value)
{
    std::ostringstream s;
    s << std::boolalpha << value;
    std::string out(key);
    out += '=';
    out += quoted_if_needed(s.str());
    out += ' ';
    return out;
}

// ---- LocalesUtils.cpp:54-63, verbatim but for the optional pos ----

inline double string_to_double_decimal_point(const std::string_view str)
{
    double out = 0.;
    fast_float::from_chars(str.data(), str.data() + str.size(), out);
    return out;
}

// ---- Utils.cpp:1325-1334, verbatim ----

inline std::string format_diameter_to_str(double diameter, int precision = 1)
{
    double candidates[] = {0.2, 0.4, 0.6, 0.8};
    double best = *std::min_element(std::begin(candidates), std::end(candidates), [diameter](double a, double b) { return std::abs(a - diameter) < std::abs(b - diameter); });
    std::ostringstream oss;
    // Diameters are used as keys ("0.4"), never as UI text, so they must not follow the UI locale.
    oss.imbue(std::locale::classic());
    oss << std::fixed << std::setprecision(precision) << best;
    return oss.str();
}

// ---- PrintConfig.hpp:382-390, verbatim ----

enum NozzleVolumeType {
    nvtStandard = 0,
    nvtHighFlow,
    nvtHybrid,
    nvtTPUHighFlow,
    nvtTPU285,
    nvtE3DHighFlow,
    nvtCount, // Always put this at the end
};

// ---- Config.hpp: the option classes, reduced to their values ----

struct ConfigOption { virtual ~ConfigOption() = default; };
struct ConfigOptionString : ConfigOption { std::string value; };
struct ConfigOptionBool : ConfigOption { bool value = false; };
struct ConfigOptionStrings : ConfigOption { std::vector<std::string> values; };
struct ConfigOptionInts : ConfigOption { std::vector<int> values; };
struct ConfigOptionEnumsGeneric : ConfigOption { std::vector<int> values; };
struct ConfigOptionFloats : ConfigOption { std::vector<double> values; };
struct ConfigOptionFloatsNullable : ConfigOption
{
    std::vector<double> values;
    size_t size() const { return values.size(); }
};
struct ConfigOptionIntsNullable : ConfigOption
{
    std::vector<int> values;
    // Config.hpp: nil_value() is std::numeric_limits<int>::max()
    bool is_nil(size_t idx) const { return values[idx] == std::numeric_limits<int>::max(); }
};

struct DynamicPrintConfig
{
    std::map<std::string, std::shared_ptr<ConfigOption>> options;

    const ConfigOption *option(const std::string &key) const
    {
        auto it = options.find(key);
        return it == options.end() ? nullptr : it->second.get();
    }
    template<class T> const T *option(const std::string &key) const { return dynamic_cast<const T *>(option(key)); }
    template<class T> T *option(const std::string &key, bool create = false)
    {
        auto it = options.find(key);
        if (it != options.end())
            return dynamic_cast<T *>(it->second.get());
        if (!create)
            return nullptr;
        auto opt = std::make_shared<T>();
        options[key] = opt;
        return opt.get();
    }
    bool               has(const std::string &key) const { return options.count(key) > 0; }
    const std::string &opt_string(const std::string &key) const { return option<ConfigOptionString>(key)->value; }
};
using ConfigBase           = DynamicPrintConfig;
using ConfigOptionResolver = DynamicPrintConfig;

// ---- PrintConfig.cpp ----

// PrintConfig.cpp:897-904, as a name table
inline const std::map<std::string, int> s_keys_map_NozzleVolumeType = {
    { "Standard",  nvtStandard },
    { "High Flow", nvtHighFlow },
    { "TPU High Flow", nvtTPUHighFlow },
    { "TPU285", nvtTPU285},
    { "Hybrid", nvtHybrid},
    { "E3D High Flow", nvtE3DHighFlow }
};
// PrintConfig.cpp:905: the names by enum value
inline const char *const s_keys_names_NozzleVolumeType[] = {"Standard", "High Flow", "Hybrid", "TPU High Flow", "TPU285", "E3D High Flow"};

// PrintConfig.cpp:139-143, verbatim
inline bool multi_nozzle_diameter_mode_enabled(const ConfigOptionResolver &config)
{
    const auto *opt = config.option<ConfigOptionBool>("multi_nozzle_diameter_mode");
    return opt != nullptr && opt->value;
}

// PrintConfig.cpp:1023-1030, verbatim but for boost::format
inline std::string get_nozzle_volume_type_string(NozzleVolumeType nozzle_volume_type)
{
    if (int(nozzle_volume_type) < 0 || nozzle_volume_type >= nvtCount) {
        BOOST_LOG_TRIVIAL(error) << "unsupported NozzleVolumeType=" << int(nozzle_volume_type);
        return "";
    }
    return s_keys_names_NozzleVolumeType[nozzle_volume_type];
}

// PrintConfig.cpp:1199-1210, verbatim
inline bool is_single_ext_multi_nozzle(const ConfigBase& config)
{
    const auto* hotend_as_extruder = config.option<ConfigOptionBool>("handle_hotend_as_extruder");
    if (!hotend_as_extruder || !hotend_as_extruder->value)
        return false;
    const auto* nozzle_diameters = config.option<ConfigOptionFloatsNullable>("nozzle_diameter");
    if (!nozzle_diameters || nozzle_diameters->size() != 1)
        return false;
    const auto* max_nozzle_count = config.option<ConfigOptionIntsNullable>("extruder_max_nozzle_count");
    return max_nozzle_count && std::any_of(max_nozzle_count->values.begin(), max_nozzle_count->values.end(),
                                           [](int v) { return v > 1; });
}

// PrintConfig.cpp:1212-1242, verbatim but for boost::format
inline std::vector<std::map<NozzleVolumeType,int>> get_extruder_nozzle_stats(const std::vector<std::string>& strs)
{
    std::vector<std::map<NozzleVolumeType,int>> extruder_nozzle_counts;
    for (const std::string& str : strs) {
        std::map<NozzleVolumeType,int> nozzle_count_map;
        if(str.empty()){
            extruder_nozzle_counts.emplace_back(nozzle_count_map);
            continue;
        }
        std::vector<std::string> nozzle_infos;
        boost::algorithm::split(nozzle_infos, str, boost::is_any_of("|"));
        for (auto& nozzle_info : nozzle_infos) {
            std::vector<std::string> attr;
            boost::algorithm::split(attr, nozzle_info, boost::is_any_of("#"));
            if (attr.size() < 2) {
                BOOST_LOG_TRIVIAL(error) << "malformed nozzle stat entry \"" << nozzle_info << "\", skipped";
                continue;
            }

            NozzleVolumeType volume_type = nvtStandard;
            auto             type_iter   = s_keys_map_NozzleVolumeType.find(attr[0]);
            if (type_iter != s_keys_map_NozzleVolumeType.end())
                volume_type = NozzleVolumeType(type_iter->second);
            else
                BOOST_LOG_TRIVIAL(error) << "unknown NozzleVolumeType \"" << attr[0] << "\", fall back to Standard";
            int nozzle_count = std::atoi(attr[1].c_str());
            // Accumulate so that several unknown types collapsing onto Standard keep the total count.
            nozzle_count_map[volume_type] += nozzle_count;
        }
        extruder_nozzle_counts.emplace_back(nozzle_count_map);
    }
    return extruder_nozzle_counts;
}

// PrintConfig.cpp:1264-1270, verbatim
inline NozzleVolumeType nozzle_volume_type_from_string(const std::string &name)
{
    auto iter = s_keys_map_NozzleVolumeType.find(name);
    if (iter != s_keys_map_NozzleVolumeType.end())
        return NozzleVolumeType(iter->second);
    return nvtStandard;
}

// PrintConfig.cpp:1275-1310, verbatim but for boost::format
inline std::vector<NozzleVolumeType> default_nozzle_volume_slots(const ConfigBase &config, int extruder_id)
{
    const auto *diameters      = dynamic_cast<const ConfigOptionFloatsNullable *>(config.option("nozzle_diameter"));
    const auto *max_counts     = dynamic_cast<const ConfigOptionIntsNullable *>(config.option("extruder_max_nozzle_count"));
    const int   extruder_count = diameters ? int(diameters->values.size()) : 1;
    if (extruder_id < 0 || extruder_id >= extruder_count) return {};

    auto slot_count = [max_counts](int eid) {
        if (!max_counts || eid >= int(max_counts->values.size()) || max_counts->is_nil(eid)) return 1;
        return std::max(1, max_counts->values[eid]);
    };

    std::vector<NozzleVolumeType> slots(slot_count(extruder_id), nvtStandard);
    const auto *opt   = dynamic_cast<const ConfigOptionEnumsGeneric *>(config.option("default_nozzle_volume_type"));
    const int   count = opt ? int(opt->values.size()) : 0;
    if (count == 0) return slots;

    if (count <= extruder_count) {
        std::fill(slots.begin(), slots.end(), NozzleVolumeType(opt->values[std::min(extruder_id, count - 1)]));
        return slots;
    }

    int offset = 0, total = 0;
    for (int eid = 0; eid < extruder_count; ++eid) {
        if (eid == extruder_id) offset = total;
        total += slot_count(eid);
    }
    if (count != total) {
        const auto *model = dynamic_cast<const ConfigOptionString *>(config.option("printer_model"));
        BOOST_LOG_TRIVIAL(warning) << "default_nozzle_volume_type has " << count << " values, expected 1, " << extruder_count << " or " << total
                                   << ", machine: " << (model ? model->value : std::string("unknown"));
        return slots;
    }
    for (size_t slot = 0; slot < slots.size(); ++slot)
        slots[slot] = NozzleVolumeType(opt->values[offset + slot]);
    return slots;
}

// PrintConfig.cpp:1323-1327, verbatim
inline NozzleVolumeType get_default_nozzle_volume_type(const ConfigBase &config, int extruder_id, int nozzle_id)
{
    const std::vector<NozzleVolumeType> slots = default_nozzle_volume_slots(config, extruder_id);
    return nozzle_id >= 0 && nozzle_id < int(slots.size()) ? slots[nozzle_id] : nvtStandard;
}

// PrintConfig.cpp:1443-1457, verbatim
inline std::vector<std::string> save_extruder_nozzle_stats_to_string(const std::vector<std::map<NozzleVolumeType,int>>& extruder_nozzle_stats)
{
    std::vector<std::string> extruder_nozzle_count_str;
    for (size_t idx = 0; idx < extruder_nozzle_stats.size(); ++idx) {
        std::ostringstream oss;
        const auto& item = extruder_nozzle_stats[idx];
        for (auto it = item.begin(); it != item.end(); ++it) {
            oss << get_nozzle_volume_type_string(it->first) << "#" << it->second;
            if (std::next(it) != item.end())
                oss << "|";
        }
        extruder_nozzle_count_str.emplace_back(oss.str());
    }
    return extruder_nozzle_count_str;
}

// PrintConfig.hpp:2229-2236, verbatim
inline bool nozzle_variant_exists(const std::string &diameter, NozzleVolumeType volume_type)
{
    if (volume_type == NozzleVolumeType::nvtE3DHighFlow && diameter != "0.4" && diameter != "0.6")
        return false;
    if (diameter == "0.2" && volume_type != NozzleVolumeType::nvtStandard)
        return false;
    return true;
}

// ---- PresetBundle.hpp / PresetBundle.cpp ----

// PresetBundle.hpp:113-148, the members NozzleInventory.cpp and the functions below use, verbatim
struct ExtruderFullStat
{
public:
    ExtruderFullStat() = default;
    const std::vector<std::vector<std::pair<std::string, std::string>>> &get_raw_stat() const { return extruder_nozzle_stats; }
    void resize(size_t extruder_count) { extruder_nozzle_stats.resize(extruder_count); }
    void set_extruder_stats(size_t extruder_id, const std::vector<std::pair<std::string, std::string>> &stats)
    {
        if (extruder_id >= extruder_nozzle_stats.size()) extruder_nozzle_stats.resize(extruder_id + 1);
        extruder_nozzle_stats[extruder_id] = stats;
    }
private:
    std::vector<std::vector<std::pair<std::string, std::string>>> extruder_nozzle_stats;
};

// PresetBundle.cpp:1778-1790, verbatim
inline std::string GetNozzleVolumeTypeStr(const NozzleVolumeType &type)
{
    switch (type) {
    case NozzleVolumeType::nvtStandard: return "Standard";
    case NozzleVolumeType::nvtHighFlow: return "High Flow";
    case NozzleVolumeType::nvtTPUHighFlow: return "TPU High Flow";
    case NozzleVolumeType::nvtTPU285: return "TPU285";
    case NozzleVolumeType::nvtE3DHighFlow: return "E3D High Flow";
    case NozzleVolumeType::nvtHybrid: return "Hybrid";
    default: return std::string();
    }
}

// PresetBundle.cpp:1822-1843, verbatim
inline bool per_slot_nozzle_options_consistent(const DynamicPrintConfig &config)
{
    const ConfigOptionInts*         count_opt = config.option<ConfigOptionInts>("extruder_nozzle_count");
    const ConfigOptionEnumsGeneric* vtype_opt = config.option<ConfigOptionEnumsGeneric>("extruder_nozzle_volume_type");
    const ConfigOptionFloats*       diam_opt  = config.option<ConfigOptionFloats>("extruder_nozzle_diameter");
    const ConfigOptionInts*         slot_opt  = config.option<ConfigOptionInts>("extruder_nozzle_slot");
    const std::vector<int> counts = count_opt ? count_opt->values : std::vector<int>{};
    const std::vector<int> vtypes = vtype_opt ? vtype_opt->values : std::vector<int>{};
    const std::vector<double> diams = diam_opt ? diam_opt->values : std::vector<double>{};
    const std::vector<int> slots = slot_opt ? slot_opt->values : std::vector<int>{};

    int total = 0;
    for (int c : counts) {
        if (c < 0)
            return false;
        total += c;
    }
    return total > 0
        && static_cast<int>(vtypes.size()) == total
        && static_cast<int>(diams.size()) == total
        && static_cast<int>(slots.size()) == total;
}

// PresetBundle.cpp:1845-1870, verbatim
inline void sync_per_slot_nozzle_options_from_full_stat(DynamicPrintConfig &config, const ExtruderFullStat &full_stat)
{
    const auto raw = full_stat.get_raw_stat();
    std::vector<int>    nozzle_counts;
    std::vector<int>    nozzle_volume_types;
    std::vector<double> nozzle_diameters;
    std::vector<int>    nozzle_slots;
    nozzle_counts.reserve(raw.size());
    int global_slot = 0;
    for (const auto &ext : raw) {
        int installed = 0;
        for (const auto &slot : ext) {
            if (!slot.second.empty()) {
                nozzle_volume_types.push_back(static_cast<int>(nozzle_volume_type_from_string(slot.second)));
                nozzle_diameters.push_back(slot.first.empty() ? 0.0 : string_to_double_decimal_point(slot.first.c_str()));
                nozzle_slots.push_back(global_slot);
                ++installed;
            }
            ++global_slot;
        }
        nozzle_counts.push_back(installed);
    }
    config.option<ConfigOptionInts>("extruder_nozzle_count", true)->values              = std::move(nozzle_counts);
    config.option<ConfigOptionEnumsGeneric>("extruder_nozzle_volume_type", true)->values = std::move(nozzle_volume_types);
    config.option<ConfigOptionFloats>("extruder_nozzle_diameter", true)->values         = std::move(nozzle_diameters);
    config.option<ConfigOptionInts>("extruder_nozzle_slot", true)->values               = std::move(nozzle_slots);
}

// ---- AppConfig.hpp / AppConfig.cpp: the storage and the members NozzleInventory.cpp calls ----

class AppConfig
{
public:
    std::map<std::string, std::map<std::string, std::string>> m_storage;

    // AppConfig.hpp:70-83, verbatim
    bool get(const std::string &section, const std::string &key, std::string &value) const
    {
        value.clear();
        auto it = m_storage.find(section);
        if (it == m_storage.end())
            return false;
        auto it2 = it->second.find(key);
        if (it2 == it->second.end())
            return false;
        value = it2->second;
        return true;
    }
    std::string get(const std::string &section, const std::string &key) const
    { std::string value; this->get(section, key, value); return value; }
    // AppConfig.hpp:87-102 without the dirty flag
    void set(const std::string &section, const std::string &key, const std::string &value)
    {
        std::string &old = m_storage[section][key];
        if (old != value)
            old = value;
    }
    bool has_section(const std::string &section) const { return m_storage.find(section) != m_storage.end(); }
    const std::map<std::string, std::string> &get_section(const std::string &section) const { return m_storage.find(section)->second; }
    void set_section(const std::string &section, const std::map<std::string, std::string> &data) { m_storage[section] = data; }

    // AppConfig.cpp:1477-1489, verbatim
    void save_nozzle_volume_types_to_config(const std::string& printer_name, const std::string& nozzle_volume_types)
    {
        if (!has_section("nozzle_volume_types")) {
            std::map<std::string, std::string> data;
            data[printer_name] = nozzle_volume_types;
            set_section("nozzle_volume_types", data);
        } else {
            auto data        = get_section("nozzle_volume_types");
            auto data_modify = const_cast<std::map<std::string, std::string>&>(data);
            data_modify[printer_name] = nozzle_volume_types;
            set_section("nozzle_volume_types", data_modify);
        }
    }

    // AppConfig.cpp:1491-1501, verbatim
    std::string get_nozzle_volume_types_from_config(const std::string& printer_name)
    {
        std::string nozzle_volume_types;
        if (has_section("nozzle_volume_types")) {
            auto data        = get_section("nozzle_volume_types");
            if (data.find(printer_name) != data.end())
                nozzle_volume_types = data[printer_name];
        }

        return nozzle_volume_types;
    }
};

// ---- Preset.hpp: the selected printer preset, its base and the edited config ----

struct Preset
{
    std::string        name;
    DynamicPrintConfig config;
};

class PresetCollection
{
public:
    Preset        selected;
    Preset        edited;
    const Preset *base = nullptr; ///< what get_preset_base() gives for the selected preset; may be null

    const Preset &get_selected_preset() const { return selected; }
    const Preset &get_edited_preset() const { return edited; }
    const Preset *get_preset_base(const Preset &) const { return base; }
};

} // namespace Slic3r
