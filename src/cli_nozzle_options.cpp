#include "cli_nozzle_options.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#ifdef ENGINE_BAMBU
#include "libslic3r/LocalesUtils.hpp"   // string_to_double_decimal_point
#include "libslic3r/Utils.hpp"          // format_diameter_to_str
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
#include "libslic3r/MultiNozzleUtils.hpp"
#endif
#endif

namespace slicer_cli {

using namespace Slic3r;

std::vector<int> sanitized_filament_map(std::vector<int> map) {
    int default_value = -1;
    for (int v : map)
        if (v != -1) { default_value = v; break; }
    for (int& v : map)
        if (v == -1)
            v = default_value != -1 ? default_value : 1;
    return map;
}

#ifndef ENGINE_SNAPMAKER
namespace {

std::string printer_name(const DynamicPrintConfig& config) {
    const auto* name = config.option<ConfigOptionString>("printer_settings_id");
    return name && !name->value.empty() ? " (" + name->value + ")" : std::string();
}

std::vector<int> max_nozzle_counts(const DynamicPrintConfig& config) {
    const auto* opt = config.option<ConfigOptionIntsNullable>("extruder_max_nozzle_count");
    return opt ? opt->values : std::vector<int>{};
}

bool has_nozzle_rack(const DynamicPrintConfig& config) {
    const std::vector<int> counts = max_nozzle_counts(config);
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
    // OrcaSlicer skips an unset entry: a nullable int's nil is INT_MAX
    // (OrcaSlicer.cpp 6840-6845 at f92a7c0).
    return std::any_of(counts.begin(), counts.end(),
                       [](int v) { return v > 1 && v != ConfigOptionIntsNullable::nil_value(); });
#else
    return std::any_of(counts.begin(), counts.end(), [](int v) { return v > 1; });
#endif
}

int extruder_count(const DynamicPrintConfig& config) {
    const auto* nozzles = dynamic_cast<const ConfigOptionVectorBase*>(config.option("nozzle_diameter"));
    return nozzles ? std::max(1, (int)nozzles->size()) : 1;
}

} // namespace

#ifdef ENGINE_BAMBU
std::string estimate_mode_nozzle_settings(DynamicPrintConfig& extra, const DynamicPrintConfig& config) {
    const int new_extruder_count = extruder_count(config);
    const std::vector<int> extruder_max_nozzle_count = max_nozzle_counts(config);
    const bool support_multi_nozzle = has_nozzle_rack(config);
    const bool new_is_multi_extruder = new_extruder_count > 1;
    if (extra.has("filament_map_mode")) {
        const auto* opt_fmm = extra.option<ConfigOptionEnum<FilamentMapMode>>("filament_map_mode");
        if (!opt_fmm || opt_fmm->value != fmmAutoForFlush)
            return "--estimate-mode slices with the Auto For Flush filament grouping only; drop --filament-map-mode or "
                   "give \"Auto For Flush\".";
    } else if (new_is_multi_extruder || support_multi_nozzle) {
        extra.option<ConfigOptionEnum<FilamentMapMode>>("filament_map_mode", true)->value = fmmAutoForFlush;
    }
    if (new_is_multi_extruder || support_multi_nozzle)
        extra.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true)->values.assign(new_extruder_count, nvtStandard);
    if (support_multi_nozzle) {
        extra.option<ConfigOptionInts>("extruder_nozzle_count", true)->values = extruder_max_nozzle_count;
        int total_sub_nozzles = 0;
        for (int v : extruder_max_nozzle_count)
            total_sub_nozzles += (v > 0 ? v : 1);
        extra.option<ConfigOptionEnumsGeneric>("extruder_nozzle_volume_type", true)->values.assign(total_sub_nozzles, nvtStandard);
    }
    return {};
}

bool command_line_nozzle_rack(DynamicPrintConfig& extra, DynamicPrintConfig& config, int new_extruder_count,
                              ExtruderNozzleStat& stats) {
    if (!extra.has("extruder_nozzle_count") || !extra.has("extruder_nozzle_volume_type"))
        return false;
    std::vector<std::map<NozzleVolumeType, int>> extruder_nozzle_stats_maps(new_extruder_count);
    std::vector<int> extruder_nozzle_counts = extra.option<ConfigOptionInts>("extruder_nozzle_count")->values;
    std::vector<int> extruder_nozzle_volume_types = extra.option<ConfigOptionEnumsGeneric>("extruder_nozzle_volume_type")->values;
    // The nozzles' diameters: the command line's, else the printer's own on
    // every slot (7107-7116).
    std::vector<double> extruder_nozzle_diameters;
    if (extra.has("extruder_nozzle_diameter"))
        extruder_nozzle_diameters = extra.option<ConfigOptionFloats>("extruder_nozzle_diameter")->values;
    else {
        const auto* nd_opt = config.option<ConfigOptionFloatsNullable>("nozzle_diameter");
        const double fallback = (nd_opt && !nd_opt->values.empty()) ? nd_opt->values.front() : 0.4;
        extruder_nozzle_diameters.assign(extruder_nozzle_volume_types.size(), fallback);
    }
    // An extruder given 0 nozzles gets one Standard nozzle of a diameter the
    // list does not hold yet: 0.2 first, then 0.4, 0.6, 0.8 (7117-7150).
    {
        static const double k_pad_diameters[] = {0.2, 0.4, 0.6, 0.8};
        const auto next_unused_diameter = [&extruder_nozzle_diameters]() {
            for (double candidate : k_pad_diameters) {
                bool used = false;
                for (double existing : extruder_nozzle_diameters)
                    if (std::abs(existing - candidate) < 1e-3) { used = true; break; }
                if (!used)
                    return candidate;
            }
            return k_pad_diameters[0];
        };
        int insert_at = 0;
        const int extruder_n = std::min(new_extruder_count, (int)extruder_nozzle_counts.size());
        for (int e_index = 0; e_index < extruder_n; ++e_index) {
            if (extruder_nozzle_counts[e_index] > 0) {
                insert_at += extruder_nozzle_counts[e_index];
                continue;
            }
            extruder_nozzle_counts[e_index] = 1;
            const double pad_diameter = next_unused_diameter();
            const int volume_at = std::min(insert_at, (int)extruder_nozzle_volume_types.size());
            const int diameter_at = std::min(insert_at, (int)extruder_nozzle_diameters.size());
            extruder_nozzle_volume_types.insert(extruder_nozzle_volume_types.begin() + volume_at, (int)nvtStandard);
            extruder_nozzle_diameters.insert(extruder_nozzle_diameters.begin() + diameter_at, pad_diameter);
            insert_at += 1;
        }
    }
    // The official loop reads past a list shorter than the counts state; a
    // short list ends the count here.
    size_t nozzle_index = 0;
    for (int e_index = 0; e_index < new_extruder_count && e_index < (int)extruder_nozzle_counts.size(); e_index++) {
        std::map<NozzleVolumeType, int> nozzle_volume_type_maps;
        for (int sub_index = 0; sub_index < extruder_nozzle_counts[e_index] && nozzle_index < extruder_nozzle_volume_types.size();
             sub_index++, nozzle_index++)
            nozzle_volume_type_maps[NozzleVolumeType(extruder_nozzle_volume_types[nozzle_index])]++;
        extruder_nozzle_stats_maps[e_index] = std::move(nozzle_volume_type_maps);
    }
    stats.set_raw_stat(extruder_nozzle_stats_maps);
    config.option<ConfigOptionInts>("extruder_nozzle_count", true)->values = extruder_nozzle_counts;
    config.option<ConfigOptionEnumsGeneric>("extruder_nozzle_volume_type", true)->values = extruder_nozzle_volume_types;
    // The input's slots go: the command line's types are slots 0..K-1, the
    // ones ams_nozzle_idx counts in (7164-7166).
    config.option<ConfigOptionInts>("extruder_nozzle_slot", true)->values.clear();
    // No AMS nozzle unless the command line names one (7167-7169).
    if (!extra.has("ams_nozzle_idx"))
        config.option<ConfigOptionInts>("ams_nozzle_idx", true)->values = {-1};
    config.option<ConfigOptionFloats>("extruder_nozzle_diameter", true)->values = extruder_nozzle_diameters;
    // The flags are laid over the settings once more before the slice: they
    // carry the lists as normalised here, not as typed.
    extra.option<ConfigOptionInts>("extruder_nozzle_count", true)->values = extruder_nozzle_counts;
    extra.option<ConfigOptionEnumsGeneric>("extruder_nozzle_volume_type", true)->values = extruder_nozzle_volume_types;
    if (extra.has("extruder_nozzle_diameter"))
        extra.option<ConfigOptionFloats>("extruder_nozzle_diameter", true)->values = extruder_nozzle_diameters;
    return true;
}

void sync_ams_count_with_command_line(const DynamicPrintConfig& extra, DynamicPrintConfig& config) {
    if (!extra.has("ams_nozzle_idx") || !is_single_ext_multi_nozzle(config))
        return;
    const auto* ams_idx_opt = config.option<ConfigOptionInts>("ams_nozzle_idx");
    const int ams_idx = (ams_idx_opt && !ams_idx_opt->values.empty()) ? ams_idx_opt->get_at(0) : -1;
    config.option<ConfigOptionStrings>("extruder_ams_count", true)->values.assign(extruder_count(config),
                                                                                  ams_idx >= 0 ? "1#0|4#1" : "0#0");
}

std::string quality_mode_refusal(const DynamicPrintConfig& config) {
    const auto* types = dynamic_cast<const ConfigOptionEnumsGeneric*>(config.option("extruder_type"));
    if (types && types->values.size() > 1)
        for (size_t i = 1; i < types->values.size(); ++i)
            if (types->values[i] != types->values[0])
                return {};
    return "--filament-map-mode \"Auto For Quality\" groups the filaments by the difference between a printer's "
           "extruders; this printer" + printer_name(config) + " has no two extruders of different types. Give "
           "\"Auto For Flush\" or \"Auto For Match\".";
}

std::string rack_group_maps(std::vector<int>& map, std::vector<int>& nozzle_map, const DynamicPrintConfig& config,
                            size_t project_filaments) {
    const std::string printer = printer_name(config);
    const size_t filaments = std::max<size_t>(1, project_filaments);
    if (map.size() != filaments)
        return "--filament-map gives " + std::to_string(map.size()) + " value" + (map.size() == 1 ? "" : "s") +
               " but the project has " + std::to_string(filaments) + " filament" + (filaments == 1 ? "" : "s") +
               ": give one value per filament, in filament order.";
    for (size_t i = 0; i < map.size(); ++i)
        if (map[i] != 1 && map[i] != 2)
            return "--filament-map gives " + std::to_string(map[i]) + " for filament " + std::to_string(i + 1) +
                   ", but this printer" + printer + " has one extruder with a nozzle rack: give 1 for the nozzle the "
                   "AMS feeds or 2 for the fixed nozzles, as Bambu Studio's filament grouping does. The engine picks "
                   "the fixed nozzle; --filament-map-mode \"Nozzle Manual\" with --filament-nozzle-map names it.";
    const std::vector<int> counts = max_nozzle_counts(config);
    const int total_nozzles = counts.empty() ? 0 : counts[0];
    // The nozzle the AMS feeds, as the engine reads it: none without an AMS
    // (get_effective_ams_nozzle_idx, PrintConfig.cpp 1176-1195), when the
    // dialog shows one group of fixed nozzles only.
    const int ams_idx = get_effective_ams_nozzle_idx(config);
    if (ams_idx < 0)
        for (size_t i = 0; i < map.size(); ++i)
            if (map[i] == 1)
                return "--filament-map gives 1 (the nozzle the AMS feeds) for filament " + std::to_string(i + 1) +
                       ", but the settings state no AMS on this printer" + printer +
                       ": every nozzle is a fixed nozzle. Give --ams-nozzle-idx with the rack nozzle the AMS feeds "
                       "(0 is the first), or 2 for every filament.";
    nozzle_map.assign(map.size(), 0);
    if (ams_idx >= 0) {
        int fixed_group_nozzle_idx = 0;
        for (int nozzle = 0; nozzle < total_nozzles; ++nozzle)
            if (nozzle != ams_idx) { fixed_group_nozzle_idx = nozzle; break; }
        for (size_t i = 0; i < map.size(); ++i)
            nozzle_map[i] = map[i] == 1 ? ams_idx : fixed_group_nozzle_idx;
    }
    map.assign(map.size(), 1);
    return {};
}
#endif // ENGINE_BAMBU

std::string manual_mode_nozzle_maps(DynamicPrintConfig& extra, const DynamicPrintConfig& config, FilamentMapMode mode,
                                    size_t filament_count, bool one_plate) {
    const bool support_multi_nozzle = has_nozzle_rack(config);
    if (!support_multi_nozzle && mode == fmmNozzleManual)
        return "--filament-map-mode \"Nozzle Manual\" names the nozzle of a nozzle rack for each filament; this printer" +
               printer_name(config) + " has no nozzle rack. Give \"Manual\" with --filament-map.";
    if (!support_multi_nozzle || !one_plate)
        return {};
    if (mode == fmmNozzleManual) {
        std::string missing;
#ifdef ENGINE_BAMBU
        const std::vector<const char*> needed = {"filament_map", "filament_volume_map", "filament_nozzle_map",
                                                 "extruder_nozzle_count", "extruder_nozzle_volume_type"};
#else
        // OrcaSlicer rebuilds the grouping from the three maps alone; it has no
        // command-line rack flags (OrcaSlicer.cpp 6847-6857 at f92a7c0).
        const std::vector<const char*> needed = {"filament_map", "filament_volume_map", "filament_nozzle_map"};
#endif
        for (const char* key : needed)
            if (!extra.has(key)) {
                std::string flag = key;
                std::replace(flag.begin(), flag.end(), '_', '-');
                missing += (missing.empty() ? "--" : ", --") + flag;
            }
        if (!missing.empty())
            return "--filament-map-mode \"Nozzle Manual\" needs the rack and every filament's place in it: " + missing +
                   (missing.find(',') == std::string::npos ? " is" : " are") + " not given.";
    }
    if (mode == fmmManual && extra.has("filament_map")) {
        // Each filament prints with its extruder's flow type; an extruder
        // whose rack mixes types leaves the filament's own value (7307-7322).
        std::vector<int> nozzle_volume_type;
        if (const auto* opt = dynamic_cast<const ConfigOptionEnumsGeneric*>(config.option("nozzle_volume_type")))
            nozzle_volume_type = opt->values;
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
        // OrcaSlicer reads one flow type per nozzle: the settings' own when
        // they hold any, else the printer's defaults; a nozzle a short list
        // leaves out takes its first entry (nozzle_volume_types_per_nozzle,
        // OrcaSlicer.cpp 485-498, used at 3446 and 6863 at f92a7c0).
        if (nozzle_volume_type.empty())
            if (const auto* opt = dynamic_cast<const ConfigOptionEnumsGeneric*>(config.option("default_nozzle_volume_type")))
                nozzle_volume_type = opt->values;
        if (!nozzle_volume_type.empty())
            nozzle_volume_type.resize(std::max<size_t>(nozzle_volume_type.size(), extruder_count(config)), nozzle_volume_type.front());
#endif
        std::vector<std::map<NozzleVolumeType, int>> stats;
        if (const auto* opt = config.option<ConfigOptionStrings>("extruder_nozzle_stats"))
            stats = get_extruder_nozzle_stats(opt->values);
        const std::vector<int>& filament_maps = extra.option<ConfigOptionInts>("filament_map")->values;
        std::vector<int>& filament_volume_maps = extra.option<ConfigOptionInts>("filament_volume_map", true)->values;
        filament_volume_maps.resize(filament_count, (int)nvtStandard);
        for (size_t f_index = 0; f_index < filament_count && f_index < filament_maps.size(); f_index++) {
            int e = filament_maps[f_index] - 1;
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
            // As Print::apply reads it, a nozzle outside [1, nozzle count]
            // means nozzle 1 (OrcaSlicer.cpp 6829-6833 at f92a7c0).
            if (e < 0 || e >= extruder_count(config))
                e = 0;
#endif
            if (e < 0)
                continue;
            const bool hybrid = (size_t)e < stats.size() && stats[e].size() > 1;
            const int type = (size_t)e < nozzle_volume_type.size() ? nozzle_volume_type[e] : (int)nvtStandard;
            if (!hybrid && type != (int)nvtHybrid)
                filament_volume_maps[f_index] = type;
        }
    }
    return {};
}

std::string manual_map_extruder_refusal(const DynamicPrintConfig& config, const std::vector<int>& plate_filaments) {
    const auto* mode = config.option<ConfigOptionEnum<FilamentMapMode>>("filament_map_mode");
    if (mode == nullptr || (mode->value != fmmManual && mode->value != fmmNozzleManual))
        return {};
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
    // Nozzle Manual: OrcaSlicer rebuilds its nozzle grouping from the three maps
    // and the rack the settings describe (extruder_nozzle_stats), with this
    // same call (ToolOrdering.cpp 1822-1838 at f92a7c0). When that fails the
    // engine only logs it and goes on with an empty grouping, and the export
    // then fails inside the printer's start G-code. The rack comes from a
    // project the app saved: OrcaSlicer's command line has no flag for it.
    if (mode->value == fmmNozzleManual) {
        const auto* fmap  = config.option<ConfigOptionInts>("filament_map");
        const auto* vmap  = config.option<ConfigOptionInts>("filament_volume_map");
        const auto* nmap  = config.option<ConfigOptionInts>("filament_nozzle_map");
        const auto* stats = config.option<ConfigOptionStrings>("extruder_nozzle_stats");
        const auto* diam  = config.option<ConfigOptionFloats>("nozzle_diameter");
        bool rebuilt = fmap && vmap && nmap && vmap->values.size() == fmap->values.size() &&
                       nmap->values.size() == fmap->values.size();
        if (rebuilt) {
            std::vector<int> zero_based = fmap->values;
            for (int& v : zero_based) --v;
            std::vector<unsigned int> used;
            for (const int fid : plate_filaments)
                if (fid >= 1 && (size_t)fid <= zero_based.size())
                    used.push_back((unsigned int)(fid - 1));
            rebuilt = MultiNozzleUtils::LayeredNozzleGroupResult::create(
                          used, zero_based, vmap->values, nmap->values,
                          get_extruder_nozzle_stats(stats ? stats->values : std::vector<std::string>()),
                          diam == nullptr || diam->values.empty() ? 0.4f : (float)diam->values.front()).has_value();
        }
        if (!rebuilt)
            return "--filament-map-mode \"Nozzle Manual\": the nozzle rack these settings describe (extruder_nozzle_stats" +
                   std::string(stats == nullptr || stats->values.empty() ? ", not stated" : " = " + stats->serialize()) +
                   ") has no nozzle for every filament's place in --filament-map, --filament-volume-map and "
                   "--filament-nozzle-map. OrcaSlicer's command line has no flag that describes the rack: slice a "
                   "project saved by OrcaSlicer that states it, or give \"Manual\" with --filament-map.";
    }
#endif
    const int extruders = extruder_count(config);
    if (extruders < 2 && !has_nozzle_rack(config))
        return {};
    const auto* maps = config.option<ConfigOptionInts>("filament_map");
    const auto* printable = dynamic_cast<const ConfigOptionInts*>(config.option("filament_printable"));
    const auto* types = config.option<ConfigOptionStrings>("filament_type");
    if (maps == nullptr || printable == nullptr)
        return {};
    for (const int fid : plate_filaments) {
        if (fid < 1 || (size_t)fid > maps->values.size() || (size_t)fid > printable->values.size())
            continue;
        int extruder = maps->values[fid - 1];
#if defined(ENGINE_ORCA) && !defined(ENGINE_SNAPMAKER)
        // As Print::apply reads it, a nozzle outside [1, nozzle count] means
        // nozzle 1 (OrcaSlicer.cpp 6829-6833 at f92a7c0).
        if (extruder < 1 || extruder > extruders)
            extruder = 1;
#endif
        if (extruder < 1 || extruder > extruders)
            continue;
        if (!((printable->values[fid - 1] >> (extruder - 1)) & 1)) {
            const std::string type = types && (size_t)fid <= types->values.size() ? types->values[fid - 1] : std::string();
            return "Filament " + std::to_string(fid) + (type.empty() ? "" : " (" + type + ")") + " cannot print with extruder " +
                   std::to_string(extruder) + (extruders == 2 ? (extruder == 1 ? " (left)" : " (right)") : "") +
                   " of this printer" + printer_name(config) + ", which the filament map names for it. Give it another "
                   "extruder in --filament-map, or an automatic --filament-map-mode.";
        }
    }
    return {};
}
#endif // ENGINE_SNAPMAKER

#ifdef ENGINE_BAMBU
bool fit_project_nozzles_to_printer(Slic3r::DynamicPrintConfig& config) {
    using namespace Slic3r;
    if (!per_slot_nozzle_options_consistent(config) || !config.has("printer_variant"))
        return false;
    const auto* nozzle_diameters = config.option<ConfigOptionFloats>("nozzle_diameter");
    const auto* max_counts       = config.option<ConfigOptionIntsNullable>("extruder_max_nozzle_count");
    const size_t extruder_count  = nozzle_diameters ? nozzle_diameters->values.size() : 1;
    // read_selected_printer: the diameter, each extruder's slot count, the
    // default volume type of each slot, and whether an extruder has a rack.
    const std::string diameter = format_diameter_to_str(string_to_double_decimal_point(config.opt_string("printer_variant")));
    std::vector<int> slot_counts;
    bool has_rack = false;
    for (size_t eid = 0; eid < extruder_count; ++eid) {
        int slot_count = 1;
        if (max_counts && eid < max_counts->values.size() && !max_counts->is_nil(eid) && max_counts->values[eid] > 0)
            slot_count = max_counts->values[eid];
        slot_counts.push_back(slot_count);
        has_rack = has_rack || slot_count > 1;
    }

    // The file's slots per extruder: extruder_nozzle_count nozzles each, in
    // order, at the global slot extruder_nozzle_slot names.
    const std::vector<int>    counts = config.option<ConfigOptionInts>("extruder_nozzle_count")->values;
    const std::vector<int>    types  = config.option<ConfigOptionEnumsGeneric>("extruder_nozzle_volume_type")->values;
    const std::vector<double> sizes  = config.option<ConfigOptionFloats>("extruder_nozzle_diameter")->values;
    const std::vector<int>    slots  = config.option<ConfigOptionInts>("extruder_nozzle_slot")->values;
    if (counts.size() != extruder_count)
        return false;
    using Slot = std::pair<std::string, std::string>;   // diameter, volume type; empty when no nozzle
    std::vector<std::vector<Slot>> file(extruder_count);
    size_t nozzle = 0;
    int    offset = 0;
    for (size_t eid = 0; eid < extruder_count; ++eid) {
        file[eid].assign(size_t(slot_counts[eid]), Slot{});
        for (int i = 0; i < counts[eid]; ++i, ++nozzle) {
            const int at = slots[nozzle] - offset;
            if (at < 0 || at >= slot_counts[eid])
                return false;
            file[eid][size_t(at)] = {format_diameter_to_str(sizes[nozzle]),
                                     GetNozzleVolumeTypeStr(NozzleVolumeType(types[nozzle]))};
        }
        offset += slot_counts[eid];
    }

    // holds_selected_diameter, then resolve(): the file's slots, or the
    // printer's defaults (uniform_slots) with a type the diameter is not
    // made in replaced by Standard (drop_unmade_volume_types).
    bool changed = false;
    ExtruderFullStat full_stat;
    full_stat.resize(extruder_count);
    for (size_t eid = 0; eid < extruder_count; ++eid) {
        const bool holds = has_rack
            ? std::any_of(file[eid].begin(), file[eid].end(), [](const Slot& s) { return !s.second.empty(); })
            : std::any_of(file[eid].begin(), file[eid].end(), [&diameter](const Slot& s) { return s.first == diameter; });
        std::vector<Slot> result = file[eid];
        if (!holds) {
            changed = true;
            result.clear();
            for (int slot = 0; slot < slot_counts[eid]; ++slot) {
                NozzleVolumeType type = get_default_nozzle_volume_type(config, int(eid), slot);
                if (!nozzle_variant_exists(diameter, type))
                    type = nvtStandard;
                result.emplace_back(diameter, GetNozzleVolumeTypeStr(type));
            }
        }
        full_stat.set_extruder_stats(eid, result);
    }
    if (changed)
        sync_per_slot_nozzle_options_from_full_stat(config, full_stat);
    return changed;
}
#endif

} // namespace slicer_cli
