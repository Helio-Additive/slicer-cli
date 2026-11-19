#include "cli_layout_command.hpp"
#include "cli_json_config.hpp"
#include "cli_named_presets.hpp"
#include "layout_plan.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/Arrange.hpp"

#include <boost/nowide/convert.hpp>
#include <boost/nowide/fstream.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fcntl.h>
#ifdef _WIN32
#include <io.h>
#include <windows.h>
#else
#include <unistd.h>
#include <cerrno>
#endif

namespace slicer_cli {

using json = nlohmann::json;

// Read all of an fd into a string, polling the cancellation flag between reads
// so SIGINT/Ctrl+C during a slow stream (stdin, FIFO, or pipe) is honoured
// with a bounded exit. Returns false if cancelled.
// 0 = ok, 1 = cancelled, 2 = hard read error (not a cancel)
static int read_all_cancellable(int fd, std::string& out) {
    char buf[4096];
    while (true) {
        if (layout_plan::is_cancelled()) return 1;
#ifdef _WIN32
        // Windows: a redirected pipe can block in _read even after the CRT
        // handler runs. PeekNamedPipe before each read; when the pipe is empty,
        // wait a bounded interval and re-check the flag.
        HANDLE h = reinterpret_cast<HANDLE>(_get_osfhandle(fd));
        if (h != INVALID_HANDLE_VALUE && GetFileType(h) == FILE_TYPE_PIPE) {
            DWORD avail = 0;
            if (PeekNamedPipe(h, nullptr, 0, nullptr, &avail, nullptr) && avail == 0) {
                Sleep(50);
                continue;
            }
        }
        int n = ::_read(fd, buf, static_cast<unsigned>(sizeof buf));
#else
        ssize_t n = ::read(fd, buf, sizeof buf);
        if (n < 0 && errno == EINTR) continue;  // sigaction has no SA_RESTART
#endif
        if (n < 0) return 2;       // hard error (not a cancel)
        if (n == 0) return 0;      // EOF, ok
        out.append(buf, static_cast<size_t>(n));
    }
}

int run_layout_plan_command(const std::string& input_file) {
    layout_plan::install_cancellation_handler();  // before input read: honor SIGINT during parse
    json raw;
    std::string input_data;
    int fd = -1;
    if (!input_file.empty()) {
        // opening a FIFO blocks until a writer connects; loop on EINTR and
        // treat it as a cancellation check point
        bool cancelled = false;
        for (;;) {
            if (layout_plan::is_cancelled()) { cancelled = true; break; }
#ifdef _WIN32
            // The path is UTF-8 (boost::nowide::args): open it by its wide form.
            fd = ::_wopen(boost::nowide::widen(input_file).c_str(), _O_RDONLY | _O_BINARY);  // binary: no Ctrl+Z EOF, no newline translation
#else
            fd = ::open(input_file.c_str(), O_RDONLY);
            if (fd < 0 && errno == EINTR) {
                if (layout_plan::is_cancelled()) { cancelled = true; break; }
                continue;
            }
#endif
            break;
        }
        if (cancelled) {
            std::cerr << json{{"schemaVersion",1},{"error",{{"code","CANCELLED"},{"message","cancelled during input open"}}}}.dump() << std::endl;
            return 5;
        }
        if (fd < 0) {
            std::cerr << json{{"schemaVersion",1},{"error",{{"code","INVALID_INPUT"},{"message","cannot open --input file"}}}}.dump() << std::endl;
            return 3;
        }
    } else {
        fd = 0;  // stdin
    }
    // route --input through the same cancellable fd read loop as stdin so
    // FIFOs/slow streams observe SIGINT/Ctrl+C with a bounded exit
    int rrc = read_all_cancellable(fd, input_data);
    if (fd > 0) {
#ifdef _WIN32
        ::_close(fd);
#else
        ::close(fd);
#endif
    }
    if (rrc == 1) {
        std::cerr << json{{"schemaVersion",1},{"error",{{"code","CANCELLED"},{"message","cancelled during input read"}}}}.dump() << std::endl;
        return 5;
    }
    if (rrc == 2) {
        std::cerr << json{{"schemaVersion",1},{"error",{{"code","INVALID_INPUT"},{"message","failed to read input stream"}}}}.dump() << std::endl;
        return 3;
    }
    try { raw = json::parse(input_data); } catch (const std::exception& e) {
        if (layout_plan::is_cancelled()) {  // SIGINT during the parse → cancel, not parse-error
            std::cerr << json{{"schemaVersion",1},{"error",{{"code","CANCELLED"},{"message","cancelled during input read"}}}}.dump() << std::endl;
            return 5;
        }
        std::cerr << json{{"schemaVersion",1},{"error",{{"code","INVALID_INPUT"},{"message",std::string("JSON parse error: ")+e.what()}}}}.dump() << std::endl;
        return 3;
    }
    if (layout_plan::is_cancelled()) {  // SIGINT during a large parse → CANCELLED, no continued work
        std::cerr << json{{"schemaVersion",1},{"error",{{"code","CANCELLED"},{"message","cancelled during input read"}}}}.dump() << std::endl;
        return 5;
    }
    layout_plan::LayoutProblemV1 problem;
    layout_plan::LayoutErrorV1   parse_err;
    if (!layout_plan::parse_input(raw, problem, parse_err)) {
        json err_json = {
            {"schemaVersion", parse_err.SCHEMA_VERSION},
            {"error", {
                {"code",    parse_err.error.code},
                {"message", parse_err.error.message}
            }}
        };
        if (!parse_err.error.object_ids.empty())
            err_json["error"]["object_ids"] = parse_err.error.object_ids;
        std::cerr << err_json.dump() << std::endl;
        return parse_err.error.code == "CANCELLED" ? 5 : 3;
    }
    boost::log::core::get()->set_logging_enabled(false);
    return layout_plan::run_layout_plan(problem);
}

int run_layout_command(const std::string& layout_json_file, const char* argv0) {
    boost::nowide::ifstream lf(layout_json_file);
    if (!lf.is_open()) { std::cerr << "Cannot open layout JSON: " << layout_json_file << "\n"; return 1; }
    json lj;
    try { lj = json::parse(lf); } catch (const std::exception& e) {
        std::cerr << "Failed to parse layout JSON: " << e.what() << "\n"; return 1;
    }
    std::string profiles_dir = lj.value("profilesDir", "");
    while (!profiles_dir.empty() && profiles_dir.back() == '/') profiles_dir.pop_back();

    Slic3r::DynamicPrintConfig cfg;
    if (lj.contains("profiles")) {
        if (profiles_dir.empty()) {
            std::cerr << "profilesDir is required when profiles is specified\n";
            return 1;
        }
        for (auto& [kind, path] : lj["profiles"].items()) {
            if (!load_json_config(profiles_dir + "/" + path.get<std::string>(), cfg)) {
                std::cerr << "Failed to load profile: " << path.get<std::string>() << "\n";
                return 1;
            }
            // The profile keeps its own "inherits" chain: the arrange reads
            // the printer's clearances, which a shipped machine preset
            // leaves to its parents.
            apply_file_inherits(argv0 ? argv0 : "slicer_cli", profiles_dir + "/" + path.get<std::string>(),
                                kind.c_str(), cfg, false);
        }
    }
    using namespace Slic3r;
    using namespace Slic3r::arrangement;

    ArrangeParams params;
#ifdef ENGINE_ORCA
    params.clearance_radius = cfg.has("extruder_clearance_max_radius") ? cfg.opt_float("extruder_clearance_max_radius") : 1.0f;
    if (params.clearance_radius < 1.0f) params.clearance_radius = lj.value("clearanceRadiusMm", 68.0f);
#else
    params.cleareance_radius = cfg.has("extruder_clearance_max_radius") ? cfg.opt_float("extruder_clearance_max_radius") : 1.0f;
    if (params.cleareance_radius < 1.0f) params.cleareance_radius = lj.value("clearanceRadiusMm", 68.0f);
#endif

    // Load all STLs into a single Model (one ModelObject per STL)
    Model model;
    if (!lj.contains("objects") || !lj["objects"].is_array()) {
        std::cerr << "objects must be a JSON array\n";
        return 1;
    }
    for (auto& obj : lj["objects"]) {
        if (!obj.is_object()) {
            std::cerr << "each object entry must be a JSON object\n";
            return 1;
        }
        std::string stl_path = obj.value("stl", "");
        if (stl_path.empty()) continue;
        try {
            Model m = Model::read_from_file(stl_path);
            for (ModelObject* mo : m.objects) {
                ModelObject* new_obj = model.add_object(*mo);
                if (new_obj->instances.empty())
                    new_obj->add_instance();
            }
        } catch (const std::exception& e) {
            std::cerr << "Failed to load " << stl_path << ": " << e.what() << "\n";
            return 1;
        }
    }
    params.min_obj_distance = scaled<coord_t>(lj.value("spacingMm", 10.0));
    params.allow_rotations = lj.value("allowRotations", true);
    params.do_final_align = lj.value("doFinalAlign", true);

    // Use get_arrange_polys -> arrange pipeline (same as GUI ArrangeJob)
    ModelInstancePtrs instances;
    auto input = get_arrange_polys(model, instances);
    // get_arrange_polygon leaves bed_idx at its UNARRANGED (-1) default,
    // which the nester skips as BIN_ID_UNFIT. Seed bed 0 as the desktop
    // does (ModelArrange.cpp:98, `ap.bed_idx = 0;` in get_arrange_poly;
    // same in Orca v2.4.2 src/libslic3r/ModelArrange.cpp:25).
    for (auto& ap : input) ap.bed_idx = 0;

#ifdef ENGINE_ORCA
    update_arrange_params(params, &cfg, input);
    update_selected_items_inflation(input, &cfg, params);
    Points bed_pts = get_shrink_bedpts(&cfg, params);
#else
    update_arrange_params(params, cfg, input);
    update_selected_items_inflation(input, cfg, params);
    Points bed_pts = get_shrink_bedpts(cfg, params);
#endif
    arrangement::arrange(input, {}, bed_pts, params);

    // Apply results back to model instances
    apply_arrange_polys(input, instances, [](ArrangePolygon&) {});

    json out;
    out["engine"] =
    #ifdef ENGINE_SNAPMAKER
        "snapmaker";
    #elif defined(ENGINE_ORCA)
        "orca";
    #else
        "bambu";
    #endif
    out["placements"] = json::array();
    for (auto& ap : input) {
        json p;
        p["name"] = ap.name;
        p["bed_idx"] = ap.bed_idx;
        p["x_mm"] = unscaled<double>(ap.translation.x());
        p["y_mm"] = unscaled<double>(ap.translation.y());
        p["rotation_deg"] = ap.rotation * 180.0 / M_PI;
        BoundingBox bb = ap.transformed_poly().contour.bounding_box();
        p["cx_mm"] = unscaled<double>(bb.min.x() + bb.max.x()) / 2.0;
        p["cy_mm"] = unscaled<double>(bb.min.y() + bb.max.y()) / 2.0;
        out["placements"].push_back(p);
    }
    std::cout << out.dump() << std::endl;
    return 0;
}

} // namespace slicer_cli
