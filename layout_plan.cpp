// layout_plan.cpp — headless layout-plan executor for issue #7
//
// Reads LayoutProblemV1, runs arrange on the selected engine,
// emits PlacementCandidateV1 on stdout or LayoutErrorV1 on stderr.
// Also: `layout capabilities --json` for capability detection.
#include "layout_plan.hpp"

#include "libslic3r/Arrange.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <random>

namespace layout_plan {

using json = nlohmann::json;
using namespace Slic3r;
using namespace Slic3r::arrangement;

// ─── Cancellation signal ─────────────────────────────────────────────────────
// SIGINT sets this; the stopcondition on ArrangeParams polls it.
static std::atomic<bool> g_cancelled{false};

static void cancellation_handler(int) {
    g_cancelled.store(true, std::memory_order_release);
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

static bool load_profile(const std::string& fp, DynamicPrintConfig& cfg) {
    std::ifstream f(fp);
    if (!f.is_open()) return false;
    try {
        json j = json::parse(f);
        ConfigSubstitutionContext ctx(ForwardCompatibilitySubstitutionRule::Enable);
        for (auto& [k, v] : j.items()) {
            if (k == "type" || k == "name" || k == "inherits" || k == "from" ||
                k == "setting_id" || k == "instantiation" || k == "description" ||
                k == "compatible_printers" || k == "compatible_prints" ||
                k == "include" || k == "upward_compatible_machine" ||
                k == "printer_model" || k == "printer_variant" ||
                k == "default_filament_profile" || k == "default_print_profile")
                continue;
            try {
                std::string vs;
                if (v.is_array()) {
                    std::vector<std::string> ps;
                    for (auto& e : v) {
                        if (e.is_string()) ps.push_back(e.get<std::string>());
                        else if (e.is_number()) ps.push_back(std::to_string(e.get<double>()));
                    }
                    for (size_t i = 0; i < ps.size(); i++) {
                        if (i > 0) vs += ",";
                        vs += ps[i];
                    }
                } else if (v.is_string())
                    vs = v.get<std::string>();
                else if (v.is_number_float())
                    vs = std::to_string(v.get<double>());
                else if (v.is_number_integer())
                    vs = std::to_string(v.get<int>());
                else if (v.is_boolean())
                    vs = v.get<bool>() ? "1" : "0";
                if (!vs.empty() && vs != "nil") cfg.set_deserialize(k, vs, ctx);
            } catch (...) {}
        }
        return true;
    } catch (...) { return false; }
}

static std::string strip_trailing_slash(const std::string& s) {
    std::string r = s;
    while (!r.empty() && r.back() == '/') r.pop_back();
    return r;
}

static double safe_opt_float(const DynamicPrintConfig& cfg, const char* key, double fallback) {
    return cfg.has(key) ? cfg.opt_float(key) : fallback;
}

// ─── JSON serialisers ────────────────────────────────────────────────────────

static json to_json(const PlacedModel& pm) {
    json j;
    j["id"]           = pm.id;
    j["bed_idx"]      = pm.bed_idx;
    if (pm.bed_idx >= 0) {
        j["x_mm"]         = pm.x_mm;
        j["y_mm"]         = pm.y_mm;
        j["rotation_rad"] = pm.rotation_rad;
        j["bb_cx_mm"]     = pm.bb_cx_mm;
        j["bb_cy_mm"]     = pm.bb_cy_mm;
    }
    return j;
}

static json to_json(const PlacementCandidateV1& pc) {
    json arr = json::array();
    for (auto& p : pc.placements) arr.push_back(to_json(p));
    return {
        {"schemaVersion", pc.SCHEMA_VERSION},
        {"engine",        pc.engine},
        {"placements",    arr}
    };
}

static json to_json(const LayoutErrorV1::Detail& d) {
    json j = {{"code", d.code}, {"message", d.message}};
    if (!d.object_ids.empty()) j["object_ids"] = d.object_ids;
    return j;
}

static json to_json(const LayoutErrorV1& e) {
    return {{"schemaVersion", e.SCHEMA_VERSION}, {"error", to_json(e.error)}};
}

static json to_json(const CapabilitiesV1& c) {
    return {
        {"schemaVersion",        c.SCHEMA_VERSION},
        {"engine",               c.engine},
        {"engine_commit",        c.engine_commit},
        {"engine_version",       c.engine_version},
        {"min_schema_version",   c.min_schema_version},
        {"max_schema_version",   c.max_schema_version},
        {"capabilities", {
            {"within_plate",         c.within_plate},
            {"cross_plate",          c.cross_plate},
            {"non_rectangular_beds", c.non_rectangular_beds},
            {"locks",                c.locks},
            {"rotation_constraints", c.rotation_constraints},
            {"seeded_determinism",   c.seeded_determinism},
            {"cancellation",         c.cancellation},
            {"progress",             c.progress}
        }}
    };
}

// ─── Capabilities command ────────────────────────────────────────────────────

int run_capabilities() {
    CapabilitiesV1 caps;
#ifdef ENGINE_ORCA
    caps.engine = "orca";
#else
    caps.engine = "bambu";
#endif
    caps.engine_commit  = SLIC3R_BUILD_ID;
    caps.engine_version = SLIC3R_VERSION;
    std::cout << to_json(caps).dump() << std::endl;
    return 0;
}

// ─── Schema version rejection ────────────────────────────────────────────────

static bool check_schema_version(int sv, LayoutErrorV1& err) {
    if (sv < LayoutProblemV1::VERSION_MIN || sv > LayoutProblemV1::VERSION_MAX) {
        err.error.code    = "INVALID_INPUT";
        err.error.message = "unsupported schemaVersion " + std::to_string(sv) +
                            " (supported range: " +
                            std::to_string(LayoutProblemV1::VERSION_MIN) + "–" +
                            std::to_string(LayoutProblemV1::VERSION_MAX) + ")";
        return false;
    }
    return true;
}

// ─── Parser ──────────────────────────────────────────────────────────────────

bool parse_input(const json& raw, LayoutProblemV1& out, LayoutErrorV1& err) {
    if (!raw.is_object()) {
        err.error.code    = "INVALID_INPUT";
        err.error.message = "root must be a JSON object";
        return false;
    }

    int sv = raw.value("schemaVersion", 0);
    if (!check_schema_version(sv, err)) return false;

    out.engine = raw.value("engine", "");
    if (out.engine != "orca" && out.engine != "bambu") {
        err.error.code    = "INVALID_INPUT";
        err.error.message = "unknown engine '" + out.engine + "' (expected 'orca' or 'bambu')";
        return false;
    }

    out.profiles_dir = strip_trailing_slash(raw.value("profilesDir", ""));
    if (out.profiles_dir.empty()) {
        err.error.code    = "INVALID_INPUT";
        err.error.message = "profilesDir is required";
        return false;
    }
    if (!raw.contains("profiles") || !raw["profiles"].is_object()) {
        err.error.code    = "INVALID_INPUT";
        err.error.message = "profiles must be a JSON object";
        return false;
    }
    const json& prof_j = raw["profiles"];
    out.profiles.machine  = prof_j.value("machine", "");
    out.profiles.process  = prof_j.value("process", "");
    out.profiles.filament = prof_j.value("filament", "");
    if (out.profiles.machine.empty()) {
        err.error.code    = "INVALID_INPUT";
        err.error.message = "profiles.machine is required";
        return false;
    }

    const json& sp_j = raw.value("spacing", json::object());
    out.spacing.min_object_distance_mm = sp_j.value("minObjectDistanceMm", 10.0);
    out.spacing.clearance_radius_mm    = sp_j.value("clearanceRadiusMm", 0.0);
    out.spacing.allow_rotations        = sp_j.value("allowRotations", true);

    // seed: 0 = nondeterministic
    out.seed = raw.value("seed", uint64_t(0));

    if (!raw.contains("models") || !raw["models"].is_array()) {
        err.error.code    = "INVALID_INPUT";
        err.error.message = "models must be a JSON array";
        return false;
    }
    const json& mods_j = raw["models"];
    if (mods_j.empty()) {
        err.error.code    = "INVALID_INPUT";
        err.error.message = "at least one model is required";
        return false;
    }
    for (const auto& m : mods_j) {
        if (!m.is_object()) {
            err.error.code    = "INVALID_INPUT";
            err.error.message = "each model entry must be a JSON object";
            return false;
        // Validate field types
        if (!m["id"].is_string()) {
            err.error.code    = "INVALID_INPUT";
            err.error.message = "model entry 'id' must be a string";
            return false;
        }
        if (!m["path"].is_string()) {
            err.error.code    = "INVALID_INPUT";
            err.error.message = "model entry 'path' must be a string";
            return false;
        }
        ModelRef ref;
        ref.id        = m.value("id", "");
        ref.path      = m.value("path", "");
        ref.locked    = m.value("locked", false);
        if (ref.id.empty()) {
            err.error.code    = "INVALID_INPUT";
            err.error.message = "model entry missing 'id'";
            return false;
        }
        // Reject duplicate model ids
        for (auto& existing : out.models) {
            if (existing.id == ref.id) {
                err.error.code       = "INVALID_INPUT";
                err.error.message    = "duplicate model id '" + ref.id + "'";
                err.error.object_ids = {ref.id};
                return false;
            }
        }
        if (ref.path.empty()) {
            err.error.code    = "INVALID_INPUT";
            err.error.message = "model '" + ref.id + "' missing 'path'";
            err.error.object_ids = {ref.id};
            return false;
        }
        const json& tx_j = m.value("transform", json::object());
        ref.x_mm      = tx_j.value("x", 0.0);
        ref.y_mm      = tx_j.value("y", 0.0);
        ref.z_mm      = tx_j.value("z", 0.0);
        ref.rot_z_rad = tx_j.value("rotationZ", 0.0);

        // allowed_rotations: empty = free, non-empty = exact list in radians
        if (m.contains("allowed_rotations")) {
            if (!m["allowed_rotations"].is_array()) {
                err.error.code    = "INVALID_INPUT";
                err.error.message = "model '" + ref.id + "' allowed_rotations must be an array";
                err.error.object_ids = {ref.id};
                return false;
            }
            for (auto& r : m["allowed_rotations"]) {
                if (!r.is_number()) {
                    err.error.code    = "INVALID_INPUT";
                    err.error.message = "model '" + ref.id + "' allowed_rotations entries must be numbers";
                    err.error.object_ids = {ref.id};
                    return false;
                }
                ref.allowed_rotations.push_back(r.get<double>());
            }
        }

        out.models.push_back(ref);
    return true;
}

// ─── Runner ──────────────────────────────────────────────────────────────────

int run_layout_plan(const LayoutProblemV1& problem) {
    // --- Engine match ---
    const char* build_engine =
#ifdef ENGINE_ORCA
        "orca";
#else
        "bambu";
#endif
    if (problem.engine != build_engine) {
        LayoutErrorV1 err;
        err.error.code    = "ENGINE_MISMATCH";
    // --- Load profiles: semantic order matters (machine first, for inheritance) ---
    std::string dir = problem.profiles_dir;
    DynamicPrintConfig cfg;
    // Load common machine base if present
    {
        std::string common_path = dir + "/BBL/machine/fdm_bbl_3dp_001_common.json";
        std::ifstream cf(common_path);
        if (cf.good()) { cf.close(); load_profile(common_path, cfg); }
    }
    // Load in dependency order: machine → process → filament
    auto load_req = [&](const std::string& relative) -> bool {
        if (relative.empty()) return true;
        std::string fp = dir + "/" + relative;
        if (!load_profile(fp, cfg)) {
            LayoutErrorV1 err;
            err.error.code    = "INVALID_INPUT";
            err.error.message = "failed to load profile: " + relative;
            std::cerr << to_json(err).dump() << std::endl;
            return false;
        }
        return true;
    };
    if (!load_req(problem.profiles.machine))  return 3;
    if (!load_req(problem.profiles.process))  return 3;
    if (!load_req(problem.profiles.filament)) return 3;
    // --- Install SIGINT handler for cancellation ---
    g_cancelled.store(false);
    std::signal(SIGINT, cancellation_handler);

    // --- Load profiles ---
    std::string dir = problem.profiles_dir;
    DynamicPrintConfig cfg;
    std::string common_path = dir + "/BBL/machine/fdm_bbl_3dp_001_common.json";
    std::ifstream cf(common_path);
    if (cf.good()) { cf.close(); load_profile(common_path, cfg); }

    auto load_req = [&](const std::string& relative) -> bool {
        if (relative.empty()) return true;
        std::string fp = dir + "/" + relative;
        if (!load_profile(fp, cfg)) {
            LayoutErrorV1 err;
            err.error.code    = "INVALID_INPUT";
            err.error.message = "failed to load profile: " + relative;
            std::cerr << to_json(err).dump() << std::endl;
            return false;
        }
        return true;
    };
    if (!load_req(problem.profiles.machine))  return 3;
    if (!load_req(problem.profiles.process))  return 3;
    if (!load_req(problem.profiles.filament)) return 3;

    // --- Seed RNG if requested ---
    std::mt19937_64 rng;
    uint64_t effective_seed = problem.seed;
    bool seeded = (effective_seed != 0);
    if (seeded) {
        rng.seed(effective_seed);
    } else {
        std::random_device rd;
        effective_seed = (uint64_t(rd()) << 32) | rd();
        rng.seed(effective_seed);
    }

    // --- Load models; split locked vs unlocked ---
    Model unlocked_model, locked_model;
    for (auto& ref : problem.models) {
        try {
            Model m = Model::read_from_file(ref.path);
            for (ModelObject* mo : m.objects) {
                Model* target = ref.locked ? &locked_model : &unlocked_model;
                ModelObject* new_obj = target->add_object(*mo);
                if (new_obj->instances.empty())
                    new_obj->add_instance();
                for (auto* inst : new_obj->instances) {
                    inst->set_offset(Vec3d(ref.x_mm, ref.y_mm, ref.z_mm));
                    inst->set_rotation(Vec3d(0, 0, Geometry::rad2deg(ref.rot_z_rad)));
                }
            }
        } catch (const std::exception& e) {
            LayoutErrorV1 err;
            err.error.code       = "INVALID_INPUT";
            err.error.message    = std::string("failed to load model '") + ref.id + "': " + e.what();
            err.error.object_ids = {ref.id};
            std::cerr << to_json(err).dump() << std::endl;
            return 3;
        }
    }

    // --- Build arrange params ---
    ArrangeParams params;
    // Clearance radius: use profile value if present AND > 0, else use problem override,
    // else fall back to 1.0mm (safe default — zero means objects can touch).
    double prof_clearance = 1.0; // safe default fallback
    if (cfg.has("extruder_clearance_max_radius")) {
        double v = cfg.opt_float("extruder_clearance_max_radius");
        if (v > 0.0) prof_clearance = v;
    }
#ifdef ENGINE_ORCA
    params.clearance_radius = prof_clearance;
#else
    params.cleareance_radius = prof_clearance;
#endif
    if (problem.spacing.clearance_radius_mm > 0.0) {
#ifdef ENGINE_ORCA
        params.clearance_radius = problem.spacing.clearance_radius_mm;
#else
        params.cleareance_radius = problem.spacing.clearance_radius_mm;
#endif
    }
    params.progressind     = [](unsigned,std::string){};
    params.min_obj_distance = scaled<coord_t>(problem.spacing.min_object_distance_mm);
    params.allow_rotations  = problem.spacing.allow_rotations;
    // --- Extract arrange polygons for unlocked items ---
    ModelInstancePtrs unlocked_instances, locked_instances;
    auto unlocked_input = get_arrange_polys(unlocked_model, unlocked_instances);
    auto locked_input   = get_arrange_polys(locked_model, locked_instances);

    // --- Collect excluded polygons from locked items ---
    ArrangePolygons locked_as_excludes;
    for (auto& ap : locked_input) {
        locked_as_excludes.push_back(ap);
    }

    // --- Apply per-model rotation constraints ---
    // Map unlocked_input back to problem.models (filter locked)
    size_t ui = 0;
    for (auto& ref : problem.models) {
        if (ref.locked) continue;
        if (ui >= unlocked_input.size()) break;
        if (!ref.allowed_rotations.empty()) {
            // Restrict to the caller's list: find the best-fit rotation among them.
            // Strategy: try each allowed rotation, pick the one whose min-area
            // bounding-box rotation is closest.
            auto& ap = unlocked_input[ui];
            double best_rot = ref.allowed_rotations[0];
            double min_diff = std::numeric_limits<double>::max();
            // Compute the item's natural min-area-bbox rotation once
            double natural = min_area_boundingbox_rotation(ap.transformed_poly());
            for (double r : ref.allowed_rotations) {
                double diff = std::abs(std::remainder(r - natural, 2.0 * M_PI));
                if (diff < min_diff) { min_diff = diff; best_rot = r; }
            }
            // Apply: set the ArrangePolygon's allowed_rotations AND set initial rotation
            ap.allowed_rotations = ref.allowed_rotations;
            ap.rotation = best_rot;
        }
        ui++;
    }

    // --- Configure and run arrange ---
#ifdef ENGINE_ORCA
    update_arrange_params(params, &cfg, unlocked_input);
    update_selected_items_inflation(unlocked_input, &cfg, params);
    Points bed_pts = get_shrink_bedpts(&cfg, params);
#else
    update_arrange_params(params, cfg, unlocked_input);
    update_selected_items_inflation(unlocked_input, cfg, params);
    Points bed_pts = get_shrink_bedpts(cfg, params);
#endif

    // Pass locked items as excluded (obstacles)
    arrangement::arrange(unlocked_input, locked_as_excludes, bed_pts, params);

    // --- Check cancellation ---
    if (g_cancelled.load(std::memory_order_acquire)) {
        LayoutErrorV1 err;
        err.error.code    = "CANCELLED";
        err.error.message = "arrangement cancelled by signal";
        std::cerr << to_json(err).dump() << std::endl;
        // NO partial candidate on stdout
        return 5;
    }

    // --- Build result ---
    PlacementCandidateV1 result;
    result.engine = build_engine;

    // Build full result: locked items come first (in input order), then unlocked items
    Polygon bed_poly(bed_pts);
    std::vector<std::string> unfittable;
    size_t ui2 = 0;

    for (auto& ref : problem.models) {
        PlacedModel pm;
        pm.id = ref.id;

        if (ref.locked) {
            // Locked item: report its input transform, never moves
            pm.bed_idx      = 0;
            pm.x_mm         = ref.x_mm;
            pm.y_mm         = ref.y_mm;
            pm.rotation_rad = ref.rot_z_rad;
            // Compute BB centre from the locked polygon
            if (locked_input.size() > 0) {
                // Find the corresponding locked polygon
                for (auto& ap : locked_input) {
                    if (ap.name.find(ref.id) == 0 || locked_input.size() == 1) {
                        // Best-effort: use first matching
                        BoundingBox bb = ap.transformed_poly().contour.bounding_box();
                        pm.bb_cx_mm = unscaled<double>((double)bb.min.x() + (double)bb.max.x()) / 2.0;
                        pm.bb_cy_mm = unscaled<double>((double)bb.min.y() + (double)bb.max.y()) / 2.0;
                        break;
                    }
                }
            }
            result.placements.push_back(pm);
            continue;
        }

        // Unlocked item
        if (ui2 >= unlocked_input.size()) break;
        auto& ap = unlocked_input[ui2++];

        BoundingBox bb = ap.transformed_poly().contour.bounding_box();
        double cx = unscaled<double>((double)bb.min.x() + (double)bb.max.x()) / 2.0;
        double cy = unscaled<double>((double)bb.min.y() + (double)bb.max.y()) / 2.0;

        bool is_unfittable = false;

        if (ap.bed_idx != 0) {
            is_unfittable = true;
        }

        if (!is_unfittable) {
            Point corners[] = {
                Point(bb.min.x(), bb.min.y()),
                Point(bb.max.x(), bb.min.y()),
                Point(bb.max.x(), bb.max.y()),
                Point(bb.min.x(), bb.max.y())
            };
            for (auto& c : corners) {
                if (!bed_poly.contains(c)) {
                    is_unfittable = true;
                    break;
                }
            }
        }

        if (is_unfittable) {
            pm.bed_idx = -1;
            unfittable.push_back(ref.id);
        } else {
            pm.bed_idx      = ap.bed_idx;
            pm.x_mm         = cx;
            pm.y_mm         = cy;
            pm.rotation_rad = ap.rotation;
            pm.bb_cx_mm     = cx;
            pm.bb_cy_mm     = cy;
        }
        result.placements.push_back(pm);
    }

    if (!unfittable.empty()) {
        LayoutErrorV1 err;
        err.error.code       = "UNFITTABLE";
        err.error.message    = "some objects could not be placed on any bed";
        err.error.object_ids = unfittable;
        std::cerr << to_json(err).dump() << std::endl;
        std::cout << to_json(result).dump() << std::endl;
        return 4;
    }

    std::cout << to_json(result).dump() << std::endl;
    return 0;
}

}  // namespace layout_plan
