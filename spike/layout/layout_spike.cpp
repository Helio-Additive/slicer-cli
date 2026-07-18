// spike/layout/layout_spike.cpp
//
// DISPOSABLE confirmation-spike harness for slicer-cli issue #7.
// Calls the pinned libslic3r arrangement kernel (Slic3r::arrangement::arrange)
// directly on raw 2D polygons — no Model, no 3MF, no GUI code.
// The fixture/output JSON formats here are explicitly NOT LayoutProblemV1 /
// PlacementCandidateV1; they exist only to measure kernel behavior.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Arrange.hpp"
#include "libslic3r/Point.hpp"

#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <nlopt.h>
#include <boost/log/core.hpp>

using json = nlohmann::json;

namespace {

[[noreturn]] void die(const std::string &msg, int code = 2)
{
    std::cerr << "layout_spike: error: " << msg << std::endl;
    std::exit(code);
}

coord_t mm(double v) { return Slic3r::scaled<coord_t>(v); }
double  to_mm(coord_t v) { return Slic3r::unscale<double>(v); }

std::string sha256_hex(const std::string &data)
{
    unsigned char out[EVP_MAX_MD_SIZE];
    size_t        outlen = 0;
    if (!EVP_Q_digest(nullptr, "SHA256", nullptr, data.data(), data.size(), out, &outlen))
        die("SHA256 failed", 4);
    std::ostringstream oss;
    for (unsigned int i = 0; i < outlen; ++i) {
        char buf[3];
        std::snprintf(buf, sizeof(buf), "%02x", out[i]);
        oss << buf;
    }
    return oss.str();
}

Slic3r::Polygon parse_ring(const json &ring, const std::string &what)
{
    if (!ring.is_array() || ring.size() < 3)
        die(what + " must be an array of >=3 [x,y] points");
    Slic3r::Points pts;
    pts.reserve(ring.size());
    for (const auto &p : ring) {
        if (!p.is_array() || p.size() != 2 || !p[0].is_number() || !p[1].is_number())
            die(what + " point must be [x,y] numbers");
        pts.emplace_back(mm(p[0].get<double>()), mm(p[1].get<double>()));
    }
    return Slic3r::Polygon(std::move(pts));
}

struct ItemIn {
    std::string id;
    bool        locked = false;
    double      translation_mm[2] = {0.0, 0.0};
    double      rotation_deg = 0.0;
};

} // namespace

int main(int argc, char **argv)
{
    boost::log::core::get()->set_logging_enabled(false); // keep stdout JSON-only
    if (argc < 2) {
        std::cerr << "usage: layout_spike <fixture.json> [--parallel 0|1] [--seed N] "
                     "[--time-budget-ms N] [--accuracy F]\n";
        return 2;
    }

    std::string fixture_path = argv[1];
    // CLI overrides; -1 / empty means "take from fixture".
    long   cli_seed = -1, cli_budget = -1;
    int    cli_parallel = -1;
    double cli_accuracy = -1.0;
    for (int i = 2; i < argc; ++i) {
        std::string a = argv[i];
        auto need = [&](const char *name) -> std::string {
            if (i + 1 >= argc) die(std::string("missing value for ") + name);
            return argv[++i];
        };
        if (a == "--parallel")       cli_parallel = std::stoi(need("--parallel"));
        else if (a == "--seed")      cli_seed = std::stol(need("--seed"));
        else if (a == "--time-budget-ms") cli_budget = std::stol(need("--time-budget-ms"));
        else if (a == "--accuracy")  cli_accuracy = std::stod(need("--accuracy"));
        else die("unknown argument: " + a);
    }

    std::ifstream in(fixture_path);
    if (!in) die("cannot open fixture: " + fixture_path);
    std::ostringstream raw;
    raw << in.rdbuf();
    const std::string raw_text = raw.str();

    json f = json::parse(raw_text, nullptr, false);
    if (f.is_discarded()) die("malformed JSON in " + fixture_path);
    if (!f.is_object()) die("fixture root must be an object");

    const std::set<std::string> known_keys = {
        "bed", "items", "exclusions", "spacing", "accuracy", "parallel",
        "seed", "time_budget_ms", "sequential", "comment"};
    for (auto it = f.begin(); it != f.end(); ++it)
        if (!known_keys.count(it.key()))
            std::cerr << "layout_spike: warning: ignoring unknown fixture key '" << it.key() << "'\n";

    // ---- bed ----
    if (!f.contains("bed") || !f["bed"].is_object() || !f["bed"].contains("polygon"))
        die("fixture missing required key 'bed.polygon'");
    Slic3r::Polygon bed_poly = parse_ring(f["bed"]["polygon"], "bed.polygon");
    Slic3r::Points  bed_pts  = bed_poly.points;

    // ---- params ----
    Slic3r::arrangement::ArrangeParams params;
    double spacing_mm = f.value("spacing", 0.0);
    params.min_obj_distance = mm(spacing_mm);
    double accuracy = cli_accuracy >= 0.0 ? cli_accuracy : f.value("accuracy", 1.0);
    params.accuracy = static_cast<float>(accuracy);
    params.parallel = cli_parallel >= 0 ? (cli_parallel != 0) : f.value("parallel", true);
    long   seed     = cli_seed >= 0 ? cli_seed : f.value("seed", 0);
    long   budget   = cli_budget >= 0 ? cli_budget : f.value("time_budget_ms", 0);
    // The kernel's default progressind prints to stdout; redirect to stderr.
    params.progressind = [](unsigned remaining, std::string msg) {
        std::cerr << "layout_spike: packed, remaining=" << remaining << " " << msg << "\n";
    };

    json warnings = json::array();

    // ---- sequential (sequential-print clearance) block ----
    if (f.contains("sequential")) {
        const json &s = f["sequential"];
        if (!s.is_object()) die("'sequential' must be an object");
        params.is_seq_print           = true;
        params.clearance_height_to_rod = s.value("clearance_height_to_rod", 0.0f);
        params.clearance_height_to_lid = s.value("clearance_height_to_lid", 0.0f);
        params.printable_height        = s.value("printable_height", 256.0f);
#ifdef ENGINE_BAMBU
        params.cleareance_radius = s.value("clearance_radius", 0.0f); // sic: Bambu spelling
#else
        params.clearance_radius = s.value("clearance_radius", 0.0f);
#endif
    }

    // ---- items ----
    if (!f.contains("items") || !f["items"].is_array() || f["items"].empty())
        die("fixture missing required non-empty array 'items'");

    Slic3r::arrangement::ArrangePolygons items, excludes;
    std::vector<ItemIn> item_meta;   // parallel to items (free items only)
    json locked_out = json::array();
    std::set<std::string> seen_ids;

    auto add_exclusion = [&](const Slic3r::Polygon &poly, double tx_mm, double ty_mm, double rot_deg) {
        Slic3r::arrangement::ArrangePolygon ap;
        ap.poly.contour = poly;
        ap.translation  = Slic3r::Vec2crd(mm(tx_mm), mm(ty_mm));
        ap.rotation     = rot_deg * PI / 180.0;
        ap.bed_idx      = 0; // fixed in logical bed 0; UNARRANGED binId is silently ignored (markAsFixedInBin)
        excludes.emplace_back(std::move(ap));
    };

    for (const auto &ji : f["items"]) {
        if (!ji.is_object()) die("each item must be an object");
        if (!ji.contains("id") || !ji["id"].is_string()) die("item missing required key 'id' (string)");
        if (!ji.contains("footprint")) die("item '" + ji["id"].get<std::string>() + "' missing 'footprint'");
        const std::string id = ji["id"].get<std::string>();
        if (!seen_ids.insert(id).second) die("duplicate item id: " + id);

        Slic3r::arrangement::ArrangePolygon ap;
        ap.poly.contour = parse_ring(ji["footprint"], "item '" + id + "' footprint");
        if (ji.contains("holes")) {
            if (!ji["holes"].is_array()) die("item '" + id + "' holes must be an array of rings");
            for (const auto &h : ji["holes"])
                ap.poly.holes.push_back(parse_ring(h, "item '" + id + "' hole"));
        }
        if (ji.contains("translation")) {
            const json &t = ji["translation"];
            if (!t.is_array() || t.size() != 2) die("item '" + id + "' translation must be [x,y]");
            ap.translation = Slic3r::Vec2crd(mm(t[0].get<double>()), mm(t[1].get<double>()));
        }
        ap.rotation = ji.value("rotation_deg", 0.0) * PI / 180.0;
        ap.inflation = mm(ji.value("spacing", 0.0));
        ap.priority  = ji.value("priority", 0);
        ap.height    = ji.value("height", 0.0);
        ap.name      = id;
        ap.bed_idx   = 0; // target logical bed 0 (UNARRANGED default = do-not-pack; see ModelArrange.cpp:98)
#ifdef ENGINE_ORCA
        // Orca's seq-print sortfunc unconditionally calls extrude_ids.front()
        // (Arrange.cpp:815) — UB on empty. The GUI always populates this.
        ap.extrude_ids = {ji.value("extruder_id", 1)};
#endif
        if (ji.contains("allowed_rotations_deg")) {
            if (!ji["allowed_rotations_deg"].is_array() || ji["allowed_rotations_deg"].empty())
                die("item '" + id + "' allowed_rotations_deg must be a non-empty array");
            ap.allowed_rotations.clear();
            for (const auto &r : ji["allowed_rotations_deg"])
                ap.allowed_rotations.push_back(r.get<double>() * PI / 180.0);
            if (ap.allowed_rotations.size() > 1) params.allow_rotations = true;
        }

        const bool locked = ji.value("locked", false);
        if (locked) {
            locked_out.push_back({{"id", id},
                                  {"x_mm", to_mm(ap.translation.x())},
                                  {"y_mm", to_mm(ap.translation.y())},
                                  {"yaw_deg", ji.value("rotation_deg", 0.0)}});
            excludes.emplace_back(std::move(ap)); // native fixed-item path
        } else {
            ItemIn meta;
            meta.id = id;
            meta.translation_mm[0] = to_mm(ap.translation.x());
            meta.translation_mm[1] = to_mm(ap.translation.y());
            meta.rotation_deg = ji.value("rotation_deg", 0.0);
            item_meta.push_back(meta);
            items.emplace_back(std::move(ap));
        }
    }

    // ---- exclusions ----
    if (f.contains("exclusions")) {
        if (!f["exclusions"].is_array()) die("'exclusions' must be an array");
        for (const auto &je : f["exclusions"]) {
            if (!je.is_object() || !je.contains("polygon"))
                die("each exclusion needs a 'polygon'");
            Slic3r::Polygon poly = parse_ring(je["polygon"], "exclusion polygon");
            double tx = 0.0, ty = 0.0, rot = 0.0;
            if (je.contains("translation")) {
                const json &t = je["translation"];
                if (!t.is_array() || t.size() != 2) die("exclusion translation must be [x,y]");
                tx = t[0].get<double>(); ty = t[1].get<double>();
            }
            rot = je.value("rotation_deg", 0.0);
            add_exclusion(poly, tx, ty, rot);
        }
    }

    // ---- run ----
    nlopt_srand(static_cast<unsigned long>(seed)); // only affects the unused genetic optimizer

    const auto t0 = std::chrono::steady_clock::now();
    bool stopped = false;
    if (budget > 0) {
        params.stopcondition = [t0, budget, &stopped]() {
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - t0).count() >= budget) {
                stopped = true;
                return true;
            }
            return false;
        };
    }

    Slic3r::arrangement::arrange(items, excludes, bed_pts, params);

    const long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0).count();

    // ---- readback ----
    json placements = json::array(), unplaced = json::array();
    std::ostringstream canonical;
    std::vector<std::string> canon_lines;
    for (size_t i = 0; i < items.size(); ++i) {
        const auto &ap = items[i];
        const std::string &id = item_meta[i].id;
        if (ap.bed_idx == Slic3r::arrangement::UNARRANGED) {
            unplaced.push_back({{"id", id}, {"reason", stopped ? "time_budget" : "no_fit"}});
        } else {
            const double x = to_mm(ap.translation.x());
            const double y = to_mm(ap.translation.y());
            const double yaw = ap.rotation * 180.0 / PI;
            placements.push_back({{"id", id}, {"bed_id", ap.bed_idx},
                                  {"x_mm", x}, {"y_mm", y}, {"yaw_deg", yaw}});
            char line[256];
            std::snprintf(line, sizeof(line), "%s|%d|%.4f|%.4f|%.4f", id.c_str(), ap.bed_idx, x, y, yaw);
            canon_lines.emplace_back(line);
        }
    }
    std::sort(canon_lines.begin(), canon_lines.end());
    for (const auto &l : canon_lines) canonical << l << "\n";

    // every input id accounted for exactly once
    if (placements.size() + unplaced.size() + locked_out.size() != seen_ids.size()) {
        std::cerr << "layout_spike: FATAL: output does not account for every input id\n";
        return 3;
    }

#ifdef ENGINE_BAMBU
    const char *engine = "bambu";
#else
    const char *engine = "orca";
#endif

    json out = {
        {"engine", engine},
        {"seed", seed},
        {"elapsed_ms", elapsed},
        {"termination", stopped ? "time_budget" : "completed"},
        {"placements", placements},
        {"unplaced", unplaced},
        {"locked", locked_out},
        {"warnings", warnings},
        {"input_sha256", sha256_hex(raw_text)},
        {"output_sha256", sha256_hex(canonical.str())},
    };
    std::cout << out.dump(2) << std::endl;
    return 0;
}
