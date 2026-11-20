#include "cli_printer_catalog.hpp"

#include <set>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>
#include <nlohmann/json.hpp>
#include "cli_snapmaker.hpp"
#ifdef _WIN32
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#else
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#endif
#ifdef __APPLE__
#include <mach-o/dyld.h>   // _NSGetExecutablePath
#include <climits>         // PATH_MAX
#endif

namespace slicer_cli {

using json = nlohmann::json;

// ── Engine resource roots ────────────────────────────────────────────────
// Both engines read folders that sit beside the profiles tree at slice time:
// Bambu Print.cpp:2687/2722/2755 (info/*.json), FlushVolPredictor.cpp:413 and
// :320 (flush/), ColorDecomposeRecipe.cpp:103 (filament_mixing/); Orca
// Print.cpp:2883/2916 (info/*.json). resources_dir() is an empty global in both
// engines until set_resources_dir() is called, so without the call below both
// binaries open "info/…" relative to the working directory and silently fall
// back to their hardcoded filament/nozzle tables.

/// Directory holding the running executable.
boost::filesystem::path engine_executable_dir(const char* argv0) {
    boost::filesystem::path exe_dir;
#if defined(_WIN32)
    {
        // The image path, wherever the process was launched from (a PATH lookup
        // by basename leaves argv[0] without a directory).
        wchar_t pathbuf[MAX_PATH + 1];
        DWORD length = GetModuleFileNameW(nullptr, pathbuf, MAX_PATH + 1);
        if (length > 0 && length <= MAX_PATH) {
            try { exe_dir = boost::filesystem::canonical(boost::filesystem::path(pathbuf)).parent_path(); } catch (...) {}
        }
    }
#elif defined(__APPLE__)
    {
        char pathbuf[PATH_MAX];
        uint32_t size = sizeof(pathbuf);
        if (_NSGetExecutablePath(pathbuf, &size) == 0) {
            try { exe_dir = boost::filesystem::canonical(pathbuf).parent_path(); } catch (...) {}
        }
    }
#else
    try { exe_dir = boost::filesystem::canonical("/proc/self/exe").parent_path(); } catch (...) {}
#endif
    // Final fallback: derive from argv[0]
    if (exe_dir.empty()) {
        try { exe_dir = boost::filesystem::canonical(argv0).parent_path(); } catch (...) {}
    }
    return exe_dir;
}

/// The profiles tree the resource folders sit beside, or empty when no known
/// layout is present. Ordered most-specific first: a checkout's references tree
/// (from build/ or build/<config>/),
/// the package layouts — Linux puts the binaries in bin/ with resources/ as its
/// sibling, macOS and Windows put them at the package root next to resources/.
boost::filesystem::path engine_profiles_dir(const boost::filesystem::path& exe_dir) {
    for (const auto& p : std::vector<boost::filesystem::path>{
        // checkout: build/slicer_cli (single-config) and build/Release/slicer_cli
        // (multi-config generators), both under the repo root
        exe_dir / ".." / "references" / "BambuStudio" / "resources" / "profiles",
        exe_dir / ".." / ".." / "references" / "BambuStudio" / "resources" / "profiles",
        exe_dir / ".." / "resources" / "profiles",
        exe_dir / "resources" / "profiles",
        boost::filesystem::path("/home/user/slicer/references/BambuStudio/resources/profiles"),
    }) {
        if (boost::filesystem::exists(p) && boost::filesystem::is_directory(p))
            return boost::filesystem::canonical(p);
    }
    return boost::filesystem::path();
}

/// The OrcaSlicer profiles tree beside this binary, or empty: the package
/// ships it as resources/profiles-orca (Linux: bin/../resources; macOS and
/// Windows: next to the binaries), a checkout has references/OrcaSlicer.
boost::filesystem::path orca_profiles_dir(const boost::filesystem::path& exe_dir) {
    for (const auto& p : std::vector<boost::filesystem::path>{
        exe_dir / ".." / "references" / "OrcaSlicer" / "resources" / "profiles",
        exe_dir / ".." / ".." / "references" / "OrcaSlicer" / "resources" / "profiles",
        exe_dir / ".." / "resources" / "profiles-orca",
        exe_dir / "resources" / "profiles-orca",
    }) {
        if (boost::filesystem::exists(p) && boost::filesystem::is_directory(p))
            return boost::filesystem::canonical(p);
    }
    return boost::filesystem::path();
}


/// The printer models a profiles tree offers: every vendor file's
/// machine_model_list (the list PresetBundle::load_vendor_configs_from_json
/// reads its printer models from, PresetBundle.cpp in both engines).
static std::set<std::string> catalog_printer_models(const boost::filesystem::path& profiles_dir) {
    std::set<std::string> models;
    if (profiles_dir.empty()) return models;
    try {
        for (auto& entry : boost::filesystem::directory_iterator(profiles_dir)) {
            if (entry.path().extension() != ".json") continue;
            // The native path: on Windows a narrowed path loses characters
            // outside the ANSI code page (as stage_file_copy's streams).
            boost::filesystem::ifstream f(entry.path());
            if (!f.is_open()) continue;
            try {
                const json vendor = json::parse(f);
                if (!vendor.is_object() || !vendor.contains("machine_model_list")) continue;
                for (const auto& m : vendor["machine_model_list"])
                    if (m.is_object() && m.contains("name") && m["name"].is_string())
                        models.insert(m["name"].get<std::string>());
            } catch (...) {
            }
        }
    } catch (...) {
    }
    return models;
}

/// The printer model of a printer system preset, read from the profiles tree:
/// <vendor>/machine/<name>.json, following "inherits" to the file that states
/// printer_model. Empty when the tree has no such preset.
std::string preset_printer_model(const boost::filesystem::path& profiles_dir, const std::string& name) {
    if (profiles_dir.empty() || name.empty()) return {};
    try {
        for (auto& vendor : boost::filesystem::directory_iterator(profiles_dir)) {
            if (!boost::filesystem::is_directory(vendor.path())) continue;
            std::string current = name;
            for (int depth = 0; depth < 16 && !current.empty(); ++depth) {
                const boost::filesystem::path file = vendor.path() / "machine" / (current + ".json");
                if (!boost::filesystem::exists(file)) break;
                boost::filesystem::ifstream f(file);
                json preset;
                try { preset = json::parse(f); } catch (...) { break; }
                if (preset.contains("printer_model") && preset["printer_model"].is_string() &&
                    !preset["printer_model"].get<std::string>().empty())
                    return preset["printer_model"].get<std::string>();
                current = preset.contains("inherits") && preset["inherits"].is_string()
                              ? preset["inherits"].get<std::string>() : std::string();
            }
        }
    } catch (...) {
    }
    return {};
}

EngineFit engine_fit_for(const std::string& argv0, const std::string& printer_model) {
    const boost::filesystem::path exe_dir = engine_executable_dir(argv0.c_str());
#ifdef ENGINE_SNAPMAKER
    const boost::filesystem::path this_dir  = slicer_cli::snapmaker_profiles_dir(exe_dir);
    // A printer this engine does not have belongs to one of the engines the
    // package ships beside it; the OrcaSlicer one is the one named.
    const boost::filesystem::path other_dir = orca_profiles_dir(exe_dir);
    EngineFit fit{"Snapmaker Orca", "slicer_cli-snapmaker", "OrcaSlicer", "slicer_cli-orcaslicer"};
    fit.third_app = "BambuStudio";
    fit.third_binary = "slicer_cli";
    fit.third_dir = engine_profiles_dir(exe_dir);
#elif defined(ENGINE_ORCA)
    const boost::filesystem::path this_dir  = orca_profiles_dir(exe_dir);
    const boost::filesystem::path other_dir = engine_profiles_dir(exe_dir);
    EngineFit fit{"OrcaSlicer", "slicer_cli-orcaslicer", "BambuStudio", "slicer_cli"};
#else
    const boost::filesystem::path this_dir  = engine_profiles_dir(exe_dir);
    const boost::filesystem::path other_dir = orca_profiles_dir(exe_dir);
    EngineFit fit{"BambuStudio", "slicer_cli", "OrcaSlicer", "slicer_cli-orcaslicer"};
#endif
#ifndef ENGINE_SNAPMAKER
    fit.third_app = "Snapmaker Orca";
    fit.third_binary = "slicer_cli-snapmaker";
    fit.third_dir = slicer_cli::snapmaker_profiles_dir(exe_dir);
#endif
    const std::set<std::string> third_models = catalog_printer_models(fit.third_dir);
    fit.third_catalog_found = !third_models.empty();
    fit.third_has = third_models.count(printer_model) > 0;
    // The Snapmaker Orca tree ships every vendor's printers, as OrcaSlicer's
    // does; Snapmaker's own are the machine_model_list of its Snapmaker
    // vendor file (the list catalog_printer_models reads).
    const boost::filesystem::path snapmaker_dir = slicer_cli::snapmaker_profiles_dir(exe_dir);
    if (!snapmaker_dir.empty() && !printer_model.empty()) {
        try {
            boost::filesystem::ifstream f(snapmaker_dir / "Snapmaker.json");
            const json vendor = json::parse(f);
            if (vendor.contains("machine_model_list"))
                for (const auto& m : vendor["machine_model_list"])
                    if (m.is_object() && m.contains("name") && m["name"] == printer_model)
                        fit.snapmaker_own_printer = true;
        } catch (...) {
        }
    }
    const std::set<std::string> this_models  = catalog_printer_models(this_dir);
    const std::set<std::string> other_models = catalog_printer_models(other_dir);
    fit.this_catalog_found  = !this_models.empty();
    fit.other_catalog_found = !other_models.empty();
    fit.this_has  = this_models.count(printer_model) > 0;
    fit.other_has = other_models.count(printer_model) > 0;
    return fit;
}

/// The sentence for a printer this engine does not have, naming the binary
/// that has it: a file for one engine sent to the other is refused, never
/// substituted, and the refusal names the engine that fits.
std::string engine_mismatch_sentence(const EngineFit& fit, const std::string& printer_model) {
    std::string s = fit.this_app + " has no " + printer_model + " printer; ";
    if (fit.other_has)
        s += "use " + fit.other_binary + " (" + fit.other_app + ").";
    else
        s += "neither engine in this package has it.";
    return s;
}

/// The printer presets of `printer_model` in a profiles tree: every
/// <vendor>/machine/<name>.json that states that printer_model. The vendor's
/// own <vendor>.json machine_list names each preset's sub_path, and the model
/// file beside them (type "machine_model") is no preset
/// (load_vendor_configs_from_json, PresetBundle.cpp in both engines).
std::vector<ShippedPrinterPreset> shipped_printer_presets(const boost::filesystem::path& profiles_dir,
                                                                 const std::string& printer_model) {
    std::vector<ShippedPrinterPreset> out;
    if (profiles_dir.empty() || printer_model.empty()) return out;
    try {
        for (auto& vendor : boost::filesystem::directory_iterator(profiles_dir)) {
            if (!boost::filesystem::is_directory(vendor.path())) continue;
            const boost::filesystem::path machine_dir = vendor.path() / "machine";
            if (!boost::filesystem::exists(machine_dir)) continue;
            for (auto& entry : boost::filesystem::directory_iterator(machine_dir)) {
                if (entry.path().extension() != ".json") continue;
                boost::filesystem::ifstream f(entry.path());
                if (!f.is_open()) continue;
                json preset;
                try { f >> preset; } catch (...) { continue; }
                if (!preset.is_object() || preset.value("type", std::string()) == "machine_model") continue;
                if (preset.value("printer_model", std::string()) != printer_model) continue;
                ShippedPrinterPreset p;
                p.name    = entry.path().stem().string();
                p.variant = preset.value("printer_variant", std::string());
                out.push_back(std::move(p));
            }
        }
    } catch (...) {
    }
    std::sort(out.begin(), out.end(), [](const ShippedPrinterPreset& a, const ShippedPrinterPreset& b) {
        return a.name < b.name;
    });
    return out;
}

/// The shipped printer preset of `printer_model` built for `variant` ("0.4"),
/// or empty: the preset a nozzle change would have to choose.
std::string shipped_printer_preset_of_variant(const std::vector<ShippedPrinterPreset>& presets,
                                                     const std::string& variant) {
    if (variant.empty()) return {};
    for (const ShippedPrinterPreset& p : presets)
        if (p.variant == variant) return p.name;
    return {};
}

/// "a, b and c" of the shipped preset names of a printer model, for a refusal
/// that names the variants this engine has.
std::string preset_name_list(const std::vector<ShippedPrinterPreset>& presets, size_t max_names) {
    std::string out;
    const size_t shown = std::min(presets.size(), max_names);
    for (size_t i = 0; i < shown; ++i) {
        if (i > 0) out += i + 1 == shown ? " and " : ", ";
        out += presets[i].name;
    }
    return out;
}

} // namespace slicer_cli
