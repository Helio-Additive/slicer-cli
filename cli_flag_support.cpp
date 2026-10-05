// cli_flag_support.cpp — see cli_flag_support.hpp.
#include "cli_flag_support.hpp"

#include <map>
#include <set>

namespace slicer_cli {
namespace {

#ifdef ENGINE_ORCA
const char* const kThisApp  = "OrcaSlicer";
const char* const kOtherApp = "BambuStudio";
#else
const char* const kThisApp  = "BambuStudio";
const char* const kOtherApp = "OrcaSlicer";
#endif

// Official CLI flags only BambuStudio has: its CLI defs at 5873b5f
// (PrintConfig.cpp export_png 9549, estimate_mode 9867, skip_useless_pick
// 9917, camera_view 9959) have no counterpart in OrcaSlicer's at 31f6803
// (PrintConfig.cpp 10329-10800). OrcaSlicer's CLI has no flag BambuStudio lacks.
const std::set<std::string>& bambu_only_keys() {
    static const std::set<std::string> keys = {"export_png", "estimate_mode", "skip_useless_pick", "camera_view"};
    return keys;
}

// Official flags this binary has but slicer-cli does not run, with the reason.
const std::map<std::string, std::string>& not_supported() {
    static const std::map<std::string, std::string> why = {
        // The official CLI renders plate pictures with OpenGL (GUI code);
        // slicer-cli makes no pictures.
        {"export_png",  "slicer-cli makes no pictures (the official CLI draws them with its GUI renderer)"},
        {"camera_view", "slicer-cli makes no pictures (the official CLI draws them with its GUI renderer)"},
#if !defined(__linux__) && !defined(__LINUX__)
        // The official pipe works on Linux only (BambuStudio.cpp 244-437,
        // OrcaSlicer.cpp 220-410; cli_pipe.cpp ports it there). Elsewhere
        // progress comes as events on stdout instead (--progress).
        {"pipe",        "the official pipe exists on Linux only; slicer-cli reports progress on every OS as events on stdout (--progress)"},
#endif
    };
    return why;
}

// Names the official CLIs define only inside comments at our pins
// (BambuStudio PrintConfig.cpp 9473-9964, OrcaSlicer 10329-10800).
const std::set<std::string>& commented_out_names() {
    static const std::set<std::string> names = {
        "export-obj", "export-svg", "export-sla", "sla", "export-amf", "export-gcode", "gcode", "g",
        "help-fff", "help-sla", "align-xy", "cut", "cut-grid", "cut-x", "cut-y", "center", "copy",
        "duplicate-grid", "repair", "split", "scale-to-fit", "ignore-nonexistent-config",
        "config-compatibility", "load", "autosave", "gcodeviewer", "sw-renderer",
    };
    return names;
}

std::string dashed(std::string key) {
    for (char& c : key)
        if (c == '_') c = '-';
    return key;
}

} // namespace

std::string official_flag_refusal(const std::string& key) {
    auto it = not_supported().find(key);
    if (it == not_supported().end())
        return {};
    return "--" + dashed(key) + " is a flag of " +
           (bambu_only_keys().count(key) ? std::string("Bambu Studio's") : std::string("Bambu Studio's and OrcaSlicer's")) +
           " own command line that slicer-cli does not support: " + it->second;
}

std::string works_in(const std::string& key) {
    return bambu_only_keys().count(key) ? "slicer_cli only (Bambu Studio engine)" : "both binaries";
}

std::string foreign_flag_refusal(const std::string& cli_name) {
#ifdef ENGINE_ORCA
    for (const std::string& key : bambu_only_keys())
        if (dashed(key) == cli_name)
            return "--" + cli_name + " works only in slicer_cli (the Bambu Studio engine); "
                   "slicer_cli-orcaslicer runs the OrcaSlicer engine, which has no such flag.";
#endif
    if (commented_out_names().count(cli_name))
        return "--" + cli_name + " is not a flag of slicer-cli: Bambu Studio's and OrcaSlicer's own "
               "command lines do not have it either (it is commented out in both)";
    (void)kOtherApp;
    return {};
}

std::vector<std::string> other_engine_only_flags() {
    std::vector<std::string> out;
#ifdef ENGINE_ORCA
    for (const std::string& key : bambu_only_keys())
        out.push_back(dashed(key));
#endif
    return out;
}

std::string extra_config_refusal(const Slic3r::DynamicPrintConfig& extra) {
    // Settings the official CLI does more with than lay them over the loaded
    // config. slicer-cli refuses them by name until it does the same:
    //   BambuStudio.cpp at 5873b5f: nozzle_volume_type 3408-3420, 3889-3890; filament_map_mode,
    //   extruder_nozzle_count, extruder_nozzle_volume_type 4050-4087;
    //   filament_map, filament_volume_map, filament_nozzle_map 6662-6837.
    //   OrcaSlicer.cpp at 31f6803: nozzle_volume_type 2950-2951, 3383-3384; filament_map_mode,
    //   filament_map 5778-5854.
    static const std::map<std::string, const char*> special = {
        {"nozzle_volume_type",          "the official CLI re-derives the nozzle setup from it"},
        {"filament_map_mode",           "the official CLI re-derives the filament-to-nozzle grouping from it"},
        {"filament_map",                "the official CLI re-derives the filament-to-nozzle grouping from it"},
        {"filament_volume_map",         "the official CLI re-derives the filament-to-nozzle grouping from it"},
        {"filament_nozzle_map",         "the official CLI re-derives the filament-to-nozzle grouping from it"},
        {"extruder_nozzle_count",       "the official CLI re-derives the nozzle setup from it"},
        {"extruder_nozzle_volume_type", "the official CLI re-derives the nozzle setup from it"},
    };
    for (const std::string& key : extra.keys()) {
        // Neither official CLI runs post-processing scripts (the call is
        // commented out: BambuStudio.cpp 7238, OrcaSlicer.cpp 6175), so the
        // setting would be taken and silently do nothing.
        if (key == "post_process")
            return "--post-process: slicer-cli does not run post-processing scripts, as the official command "
                   "lines do not; run the script on the G-code yourself.";
        auto it = special.find(key);
        if (it != special.end())
            return "--" + dashed(key) + " is a " + kThisApp + " setting that slicer-cli does not take on the "
                   "command line yet: " + it->second + ". Set it in the 3MF or the presets instead.";
    }
    return {};
}

} // namespace slicer_cli
