#include "cli_file_info.hpp"

#include "libslic3r/PresetBundle.hpp"
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include "cli_engine_info.hpp"
#include "cli_events.hpp"
#include "cli_model_load.hpp"
#include "cli_named_presets.hpp"
#include "cli_options.hpp"
#include "cli_percent_widths.hpp"
#include "cli_printer_catalog.hpp"
#include "cli_project_archive.hpp"
#include "cli_result_json.hpp"
#include "cli_snapmaker.hpp"
#include "cli_staging_folders.hpp"

namespace slicer_cli {

using json = nlohmann::json;

/// --info FILE: what the file is and which binary of this package slices it,
/// as one JSON document on stdout. The plugin calls this first to pick the
/// binary. The app that made the file is reported, and is the tie-break when
/// both engines have the printer; it never refuses anything by itself.
int run_info(const std::string& argv0, const std::string& path, const std::string& printer_preset) {
    json out;
    out["file"] = path;
    // The non-throwing check: a path that cannot be queried (permissions, a
    // failing mount) is still one JSON document, never an uncaught exception.
    boost::system::error_code exists_ec;
    const bool exists = boost::filesystem::exists(path, exists_ec);
    if (exists_ec && exists_ec != boost::system::errc::no_such_file_or_directory) {
        out["error"] = cli_error_sentence(CLI_ENVIRONMENT_ERROR) + " Cannot check " + path + ": " + exists_ec.message();
        std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
        return CLI_ENVIRONMENT_ERROR;
    }
    if (!exists) {
        out["error"] = cli_error_sentence(CLI_FILE_NOTFOUND);
        std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
        return CLI_FILE_NOTFOUND;
    }
    const bool is_3mf = input_is_3mf(path);
    // The model file kinds this engine reads (cli_model_load.cpp), by extension.
    std::string kind = "unknown";
    if (slicer_cli::is_loadable_model_file(path)) {
        kind = boost::filesystem::path(path).extension().string().substr(1);
        boost::algorithm::to_lower(kind);
    }
    out["kind"] = kind;

    std::string printer_model;
    std::string maker_app;
    std::string route_app;
    // A 3MF the slicer cannot open is not inspected: the archive must open and
    // its model part (3D/3dmodel.model, the part load_bbs_3mf reads the
    // objects from) must decompress intact, as the slice path would refuse it (-2).
    // An STL is loaded with the slice path's own loader (Slic3r::load_stl), so
    // a folder named .stl or an empty or malformed file is refused here as the
    // slice would refuse it (-2).
    if (input_is_stl(path)) {
        Slic3r::Model stl_model;
        bool loaded = false;
        try {
            loaded = Slic3r::load_stl(path.c_str(), &stl_model) && !stl_model.objects.empty();
        } catch (...) {
            loaded = false;
        }
        if (!loaded) {
            out["kind"] = "stl";
            out["error"] = cli_error_sentence(CLI_DATA_FILE_ERROR) + " " + path + " is not a readable STL file.";
            std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
            return CLI_DATA_FILE_ERROR;
        }
    }
    // Any other model file: read with the slice path's loader.
    if (!is_3mf && !input_is_stl(path) && kind != "unknown") {
        const bool loaded = slicer_cli::model_file_has_objects(path);
        if (!loaded) {
            out["error"] = cli_error_sentence(CLI_DATA_FILE_ERROR) + " " + path + " is not a readable " + kind + " file.";
            std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
            return CLI_DATA_FILE_ERROR;
        }
    }
    if (is_3mf && !zip_member_readable(path, "3D/3dmodel.model")) {
        out["error"] = cli_error_sentence(CLI_DATA_FILE_ERROR) + " " + path +
                       " is not a readable 3MF archive with a 3D/3dmodel.model part.";
        std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
        return CLI_DATA_FILE_ERROR;
    }
    if (is_3mf) {
        std::string settings_text, model_xml;
        json settings = json::object();
        if (read_zip_member(path, "Metadata/project_settings.config", settings_text)) {
            try { settings = json::parse(settings_text); } catch (...) {}
        }
        auto text = [&](const char* key) -> json {
            return settings.contains(key) ? settings[key] : json(nullptr);
        };
        if (settings.contains("printer_model") && settings["printer_model"].is_string())
            printer_model = settings["printer_model"].get<std::string>();
        out["printer_model"]   = printer_model.empty() ? json(nullptr) : json(printer_model);
        out["printer_preset"]  = text("printer_settings_id");
        out["process_preset"]  = text("print_settings_id");
        out["filament_presets"] = text("filament_settings_id");
        out["nozzle_diameter"] = text("nozzle_diameter");
        // A geometry-only 3MF is one plate, as the slice takes it.
        out["plates"] = slicer_cli::classify_3mf(path) == slicer_cli::ThreeMfKind::GeometryOnly
                            ? 1 : std::max(1, count_3mf_plates(path));
        // The placed objects with the id --skip-objects and --simplify-objects
        // take (cli_engine_info.hpp). A geometry-only 3MF declares none.
        if (std::string model_settings; read_zip_member(path, "Metadata/model_settings.config", model_settings))
            out["objects"] = slicer_cli::project_objects(model_settings);

        std::map<std::string, std::string> meta;
        if (read_zip_member(path, "3D/3dmodel.model", model_xml))
            meta = model_metadata(model_xml);
        json maker = json::object();
        const auto app = meta.find("Application");
        const auto orca = meta.find("OrcaSlicer");
        maker["application"] = app != meta.end() ? json(app->second) : json(nullptr);
        maker["orcaslicer"]  = orca != meta.end() ? json(orca->second) : json(nullptr);
        // OrcaSlicer writes "BambuStudio-<its Bambu base>" as Application and
        // its own version in the OrcaSlicer tag (bbs_3mf.cpp 6837 at 31f6803).
#ifdef ENGINE_SNAPMAKER
        // Snapmaker Orca writes "Snapmaker_Orca-<version>" as Application
        // (Format/bbs_3mf.cpp:6645-6646 at bac6fd5); its app is this engine's,
        // named as engine_fit_for names it. The other builds keep the name they
        // print today (AGENTS.md: printed output is added to, never changed).
        if (app != meta.end() && boost::starts_with(app->second, "Snapmaker_Orca-"))
            maker_app = "Snapmaker Orca";
        else
#endif
        if (orca != meta.end() || (app != meta.end() && boost::starts_with(app->second, "OrcaSlicer-")))
            maker_app = "OrcaSlicer";
        else if (app != meta.end() && boost::starts_with(app->second, "BambuStudio-"))
            maker_app = "BambuStudio";
        else if (app != meta.end())
            maker_app = app->second.substr(0, app->second.find('-'));
        maker["app"] = maker_app.empty() ? json(nullptr) : json(maker_app);
        out["maker"] = maker;
        // The engine whose app made the file, for "recommended": every build
        // routes a Snapmaker Orca project to slicer_cli-snapmaker, while
        // maker.app keeps the name each build printed before.
        route_app = app != meta.end() && boost::starts_with(app->second, "Snapmaker_Orca-") ? "Snapmaker Orca" : maker_app;

        json this_engine = {{"version", engine_version_text()}};
        if (const auto v = engine_file_version(meta)) {
            this_engine["file_version"] = v->to_string();
            // True when the slice refuses the file (the slice gate's rule): a
            // Bambu Studio project newer than the Orca build's Bambu base is
            // sliced with a warning, so it is not.
            this_engine["file_newer_than_engine"] =
                file_version_verdict(*v, meta) == FileVersionVerdict::Refused;
        }
        out["this_engine_reads"] = this_engine;
    } else {
        out["plates"] = 1;
    }
    // A model file with --printer-preset: the printer is the preset's model,
    // from either engine's profiles tree.
    if (printer_model.empty() && !printer_preset.empty()) {
        const boost::filesystem::path exe_dir = engine_executable_dir(argv0.c_str());
        printer_model = preset_printer_model(this_engine_profiles_dir(argv0), printer_preset);
        if (printer_model.empty())
#ifdef ENGINE_SNAPMAKER
            // The other engine engine_fit_for names: OrcaSlicer.
            printer_model = preset_printer_model(orca_profiles_dir(exe_dir), printer_preset);
#elif defined(ENGINE_ORCA)
            printer_model = preset_printer_model(engine_profiles_dir(exe_dir), printer_preset);
#else
            printer_model = preset_printer_model(orca_profiles_dir(exe_dir), printer_preset);
#endif
        if (printer_model.empty())
            printer_model = preset_printer_model(engine_fit_for(argv0, {}).third_dir, printer_preset);
        out["printer_preset"] = printer_preset;
        out["printer_model"]  = printer_model.empty() ? json(nullptr) : json(printer_model);
    }

    EngineFit fit = engine_fit_for(argv0, printer_model);
    json engines = json::array();
    json fits = json::array();
    auto add = [&](const std::string& app, const std::string& binary, bool found, bool has) {
        const bool fits_printer = printer_model.empty() ? found : has;
        engines.push_back(json{{"app", app}, {"binary", binary}, {"catalog_found", found},
                               {"has_printer", printer_model.empty() ? json(nullptr) : json(has)}});
        if (fits_printer) fits.push_back(binary);
    };
    add(fit.this_app, fit.this_binary, fit.this_catalog_found, fit.this_has);
    add(fit.other_app, fit.other_binary, fit.other_catalog_found, fit.other_has);
    // The package's third binary comes last, so the first two entries stay
    // as each build printed them (AGENTS.md: printed output is added to).
    add(fit.third_app, fit.third_binary, fit.third_catalog_found, fit.third_has);
    out["engines"] = engines;
    out["fits"] = fits;
    const auto fitting = [&](const std::string& binary) {
        return std::find(fits.begin(), fits.end(), binary) != fits.end();
    };
    json recommended = nullptr;
    for (const auto& e : engines)
        if (fitting(e["binary"]) && e["app"] == route_app)
            recommended = e["binary"];
    // No fitting engine made the file and the printer is one of Snapmaker's
    // own: the Snapmaker engine, the printer maker's own app.
    if (recommended.is_null() && fit.snapmaker_own_printer && fitting("slicer_cli-snapmaker"))
        recommended = "slicer_cli-snapmaker";
    // Otherwise the first that fits, the Snapmaker engine only when no other
    // does: the other two builds list it last, so the Snapmaker build answers
    // as they do.
    if (recommended.is_null() && !fits.empty() && !printer_model.empty()) {
        recommended = fits.front();
        for (const auto& b : fits)
            if (b != "slicer_cli-snapmaker") { recommended = b; break; }
    }
    out["recommended"] = recommended;
    std::cout << out.dump(2, ' ', false, json::error_handler_t::replace) << std::endl;
    return 0;
}

} // namespace slicer_cli
