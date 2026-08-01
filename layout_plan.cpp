// layout_plan.cpp — headless layout-plan executor for issue #7
//
// Reads LayoutProblemV1, runs arrange on the selected engine,
// emits PlacementCandidateV1 on stdout or LayoutErrorV1 on stderr.
#include "layout_plan.hpp"

#include "libslic3r/Arrange.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/PrintConfig.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <algorithm>

namespace layout_plan {

using json = nlohmann::json;
using namespace Slic3r;
using namespace Slic3r::arrangement;

// ─── Helpers ─────────────────────────────────────────────────────────────────

// Load a BambuStudio/OrcaSlicer profile JSON file into a DynamicPrintConfig.
// (Mirrors the load_json_config in main.cpp / arrange_harness.cpp.)
static bool load_profile(const std::string& fp, DynamicPrintConfig& cfg) {
    std::ifstream f(fp);
    if (!f.is_open()) return false;
    try {
        json j = json::parse(f);
        ConfigSubstitutionContext ctx(ForwardCompatibilitySubstitutionRule::Enable);
        for (auto& [k, v] : j.items()) {
            // skip metadata keys that aren't config options
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

// Strip trailing slashes from a path string.
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
    return {
        {"id",           pm.id},
        {"bed_idx",      pm.bed_idx},
        {"x_mm",         pm.x_mm},
        {"y_mm",         pm.y_mm},
        {"rotation_rad", pm.rotation_rad},
        {"bb_cx_mm",     pm.bb_cx_mm},
        {"bb_cy_mm",     pm.bb_cy_mm}
    };
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
    json j = {
        {"code",    d.code},
        {"message", d.message}
    };
    if (!d.object_ids.empty()) j["object_ids"] = d.object_ids;
    return j;
}

static json to_json(const LayoutErrorV1& e) {
    return {
        {"schemaVersion", e.SCHEMA_VERSION},
        {"error",         to_json(e.error)}
    };
}

// ─── Parser ──────────────────────────────────────────────────────────────────
bool parse_input(const json& raw, LayoutProblemV1& out, LayoutErrorV1& err) {
    // Guard against non-object root
    if (!raw.is_object()) {
        err.error.code    = "INVALID_INPUT";
        err.error.message = "root must be a JSON object";
        return false;
    }

    // schema version check
    int sv = raw.value("schemaVersion", 0);
    if (sv != LayoutProblemV1::SCHEMA_VERSION) {
        err.error.code    = "INVALID_INPUT";
        err.error.message = "unsupported schemaVersion " + std::to_string(sv) +
                            " (expected " + std::to_string(LayoutProblemV1::SCHEMA_VERSION) + ")";
        return false;
    }

    // engine
    out.engine = raw.value("engine", "");
    if (out.engine != "orca" && out.engine != "bambu") {
        err.error.code    = "INVALID_INPUT";
        err.error.message = "unknown engine '" + out.engine + "' (expected 'orca' or 'bambu')";
        return false;
    }

    // profiles
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

    // spacing
    const json& sp_j = raw.value("spacing", json::object());
    out.spacing.min_object_distance_mm = sp_j.value("minObjectDistanceMm", 10.0);
    out.spacing.clearance_radius_mm    = sp_j.value("clearanceRadiusMm", 0.0);
    out.spacing.allow_rotations        = sp_j.value("allowRotations", true);

    // models
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
        }
        ModelRef ref;
        ref.id        = m.value("id", "");
        ref.path      = m.value("path", "");
        if (ref.id.empty()) {
            err.error.code    = "INVALID_INPUT";
            err.error.message = "model entry missing 'id'";
            return false;
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
        out.models.push_back(ref);
    }

    return true;
}

// ─── Runner ──────────────────────────────────────────────────────────────────

int run_layout_plan(const LayoutProblemV1& problem) {
    // --- Verify engine match at runtime ---
    const char* build_engine =
#ifdef ENGINE_ORCA
        "orca";
#else
        "bambu";
#endif
    if (problem.engine != build_engine) {
        LayoutErrorV1 err;
        err.error.code    = "ENGINE_MISMATCH";
        err.error.message = std::string("requested engine '") + problem.engine +
                            "' but binary is built for '" + build_engine + "'";
        std::cerr << to_json(err).dump() << std::endl;
        return 2;
    }

    // --- Load profiles ---
    std::string dir = problem.profiles_dir;
    DynamicPrintConfig cfg;

    // Load common machine base if present
    std::string common_path = dir + "/BBL/machine/fdm_bbl_3dp_001_common.json";
    std::ifstream cf(common_path);
    if (cf.good()) { cf.close(); load_profile(common_path, cfg); }

    auto load_req = [&](const std::string& relative) -> bool {
        if (relative.empty()) return true;  // optional
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

    // --- Load models ---
    Model model;
    for (auto& ref : problem.models) {
        try {
            Model m = Model::read_from_file(ref.path);
            for (ModelObject* mo : m.objects) {
                ModelObject* new_obj = model.add_object(*mo);
                if (new_obj->instances.empty())
                    new_obj->add_instance();
                // Apply caller-supplied initial transform, if any
                if (ref.x_mm != 0.0 || ref.y_mm != 0.0 || ref.z_mm != 0.0 || ref.rot_z_rad != 0.0) {
                    for (auto* inst : new_obj->instances) {
                        inst->set_offset(Vec3d(ref.x_mm, ref.y_mm, ref.z_mm));
                        inst->set_rotation(Vec3d(0, 0, Geometry::rad2deg(ref.rot_z_rad)));
                    }
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
#ifdef ENGINE_ORCA
    params.clearance_radius = safe_opt_float(cfg, "extruder_clearance_max_radius", 1.0f);
#else
    params.cleareance_radius = safe_opt_float(cfg, "extruder_clearance_max_radius", 1.0f);
#endif
    if (problem.spacing.clearance_radius_mm > 0.0) {
#ifdef ENGINE_ORCA
        params.clearance_radius = problem.spacing.clearance_radius_mm;
#else
        params.cleareance_radius = problem.spacing.clearance_radius_mm;
#endif
    }
    params.progressind = [](unsigned,std::string){};  // suppress default stdout output
    params.min_obj_distance = scaled<coord_t>(problem.spacing.min_object_distance_mm);
    params.allow_rotations  = problem.spacing.allow_rotations;
    params.do_final_align   = true;

    // --- Extract arrange polygons ---
    ModelInstancePtrs instances;
    auto input = get_arrange_polys(model, instances);

    // --- Configure and run arrange ---
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

    // --- Build result ---
    PlacementCandidateV1 result;
    result.engine = build_engine;

    // Contract: every input model must produce exactly one arrange output.
    // If get_arrange_polys returns fewer items than problem.models, models
    // were silently dropped (multi-object 3MF expansion, zero-area skip).
    if (input.size() != problem.models.size()) {
        LayoutErrorV1 err;
        err.error.code    = "INVALID_INPUT";
        err.error.message = "arrange input count mismatch: " +
                            std::to_string(problem.models.size()) + " models but " +
                            std::to_string(input.size()) + " arrange polygons produced";
        std::cerr << to_json(err).dump() << std::endl;
        return 3;
    }

    // Validate: every placed item must have its bounding box entirely inside
    // the bed polygon (not just its AABB, for non-rectangular beds).
    Polygon bed_poly(bed_pts);
    BoundingBox bed_bounds = bed_poly.bounding_box();
    std::vector<std::string> unfittable;
    for (size_t i = 0; i < input.size(); ++i) {
        auto& ap  = input[i];
        auto& ref = problem.models[i];

        // Report the BOUNDING-BOX CENTRE as the canonical placement position.
        BoundingBox bb = ap.transformed_poly().contour.bounding_box();
        double cx = unscaled<double>(bb.min.x() + bb.max.x()) / 2.0;
        double cy = unscaled<double>(bb.min.y() + bb.max.y()) / 2.0;

        bool is_unfittable = false;

        // v1 single-bed: anything not on bed 0 (virtual, unarranged, unfittable)
        // is UNFITTABLE — checked BEFORE containment so an unplaced item that
        // happens to have its default bbox inside the bed is still caught.
        if (ap.bed_idx != 0) {
            is_unfittable = true;
        }

        // Point-in-polygon check: all four corners of the object's bounding box
        // must lie inside the bed polygon.
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

        PlacedModel pm;
        pm.id           = ref.id;
        if (is_unfittable) {
            pm.bed_idx = -1;
            unfittable.push_back(ref.id);
            // Unfittable items: emit bed_idx=-1 with no coordinates (contract)
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
