// layout_plan.cpp — headless layout-plan executor for issue #7
//
// Reads LayoutProblemV1, runs arrange, emits PlacementCandidateV1 on stdout.
// Also: `layout capabilities --json`.
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
#include <functional>
#include <iostream>

namespace layout_plan {

using json = nlohmann::json;
using namespace Slic3r;
using namespace Slic3r::arrangement;

// ─── Cancellation ────────────────────────────────────────────────────────────
static std::atomic<bool> g_cancelled{false};
static void cancellation_handler(int) { g_cancelled.store(true); }

// ─── Helpers ─────────────────────────────────────────────────────────────────

static bool load_profile(const std::string& fp, DynamicPrintConfig& cfg) {
    std::ifstream f(fp); if (!f.is_open()) return false;
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
                if (v.is_array()) { std::vector<std::string> ps; for (auto& e : v) { if (e.is_string()) ps.push_back(e.get<std::string>()); else if (e.is_number()) ps.push_back(std::to_string(e.get<double>())); } for (size_t i = 0; i < ps.size(); i++) { if (i > 0) vs += ","; vs += ps[i]; } }
                else if (v.is_string()) vs = v.get<std::string>();
                else if (v.is_number_float()) vs = std::to_string(v.get<double>());
                else if (v.is_number_integer()) vs = std::to_string(v.get<int>());
                else if (v.is_boolean()) vs = v.get<bool>() ? "1" : "0";
                if (!vs.empty() && vs != "nil") cfg.set_deserialize(k, vs, ctx);
            } catch (...) {}
        }
        return true;
    } catch (...) { return false; }
}

static void load_with_inherits(DynamicPrintConfig& cfg, const std::string& fp) {
    std::ifstream f(fp); if (!f.is_open()) return;
    json j; try { j = json::parse(f); } catch (...) { return; }
    if (j.contains("inherits")) {
        std::string v;
        auto& iv = j["inherits"];
        if (iv.is_string()) v = iv.get<std::string>();
        else if (iv.is_array() && !iv.empty() && iv[0].is_string()) v = iv[0].get<std::string>();
        if (!v.empty()) {
            std::string pp = fp.substr(0, fp.find_last_of('/')) + "/" + v;
            if (pp.size() <= 5 || pp.compare(pp.size()-5, 5, ".json") != 0) pp += ".json";
            load_with_inherits(cfg, pp);
        }
    }
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
            if (v.is_array()) { std::vector<std::string> ps; for (auto& e : v) { if (e.is_string()) ps.push_back(e.get<std::string>()); else if (e.is_number()) ps.push_back(std::to_string(e.get<double>())); } for (size_t i = 0; i < ps.size(); i++) { if (i > 0) vs += ","; vs += ps[i]; } }
            else if (v.is_string()) vs = v.get<std::string>();
            else if (v.is_number_float()) vs = std::to_string(v.get<double>());
            else if (v.is_number_integer()) vs = std::to_string(v.get<int>());
            else if (v.is_boolean()) vs = v.get<bool>() ? "1" : "0";
            if (!vs.empty() && vs != "nil") cfg.set_deserialize(k, vs, ctx);
        } catch (...) {}
    }
}

static std::string strip_dir(const std::string& s) {
    std::string r = s; while (!r.empty() && r.back() == '/') r.pop_back(); return r;
}

// ─── Serialisers ─────────────────────────────────────────────────────────────

static json to_json(const PlacedModel& pm) {
    json j; j["id"] = pm.id; j["bed_idx"] = pm.bed_idx;
    if (pm.bed_idx >= 0) {
        j["x_mm"] = pm.x_mm; j["y_mm"] = pm.y_mm;
        j["rotation_rad"] = pm.rotation_rad; j["bb_cx_mm"] = pm.bb_cx_mm; j["bb_cy_mm"] = pm.bb_cy_mm;
    }
    return j;
}
static json to_json(const PlacementCandidateV1& pc) {
    json arr = json::array(); for (auto& p : pc.placements) arr.push_back(to_json(p));
    return {{"schemaVersion",pc.SCHEMA_VERSION},{"engine",pc.engine},{"placements",arr}};
}
static json to_json(const LayoutErrorV1::Detail& d) {
    json j = {{"code",d.code},{"message",d.message}};
    if (!d.object_ids.empty()) j["object_ids"] = d.object_ids;
    return j;
}
static json to_json(const LayoutErrorV1& e) {
    return {{"schemaVersion",e.SCHEMA_VERSION},{"error",to_json(e.error)}};
}
static json to_json(const CapabilitiesV1& c) {
    return {{"schemaVersion",c.SCHEMA_VERSION},{"engine",c.engine},{"engine_commit",c.engine_commit},
        {"engine_version",c.engine_version},{"min_schema_version",c.min_schema_version},
        {"max_schema_version",c.max_schema_version},
        {"capabilities",{{"within_plate",c.within_plate},{"cross_plate",c.cross_plate},
         {"non_rectangular_beds",c.non_rectangular_beds},{"locks",c.locks},
         {"rotation_constraints",c.rotation_constraints},{"seeded_determinism",c.seeded_determinism},
         {"cancellation",c.cancellation},{"progress",c.progress}}}};
}

// ─── Capabilities ────────────────────────────────────────────────────────────

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

// ─── Parser ──────────────────────────────────────────────────────────────────

static bool check_schema(int sv, LayoutErrorV1& err) {
    if (sv < LayoutProblemV1::VERSION_MIN || sv > LayoutProblemV1::VERSION_MAX) {
        err.error.code="INVALID_INPUT";
        err.error.message="unsupported schemaVersion "+std::to_string(sv)+
            " (supported range: "+std::to_string(LayoutProblemV1::VERSION_MIN)+
            "\u2013"+std::to_string(LayoutProblemV1::VERSION_MAX)+")";
        return false;
    }
    return true;
}

bool parse_input(const json& raw, LayoutProblemV1& out, LayoutErrorV1& err) {
    if (!raw.is_object()) { err.error.code="INVALID_INPUT"; err.error.message="root must be JSON object"; return false; }
    int sv = raw.value("schemaVersion", 0);
    if (!check_schema(sv, err)) return false;
    out.engine = raw.value("engine", "");
    if (out.engine != "orca" && out.engine != "bambu") { err.error.code="INVALID_INPUT"; err.error.message="unknown engine"; return false; }
    out.profiles_dir = strip_dir(raw.value("profilesDir", ""));
    if (out.profiles_dir.empty()) { err.error.code="INVALID_INPUT"; err.error.message="profilesDir required"; return false; }
    if (!raw.contains("profiles") || !raw["profiles"].is_object()) { err.error.code="INVALID_INPUT"; err.error.message="profiles must be object"; return false; }
    const json& prof_j = raw["profiles"];
    out.profiles.machine  = prof_j.value("machine", "");
    out.profiles.process  = prof_j.value("process", "");
    out.profiles.filament = prof_j.value("filament", "");
    if (out.profiles.machine.empty()) { err.error.code="INVALID_INPUT"; err.error.message="profiles.machine required"; return false; }
    const json& sp_j = raw.value("spacing", json::object());
    out.spacing.min_object_distance_mm = sp_j.value("minObjectDistanceMm", sp_j.value("min_mm", 10.0));
    out.spacing.clearance_radius_mm    = sp_j.value("clearanceRadiusMm", 0.0);
    out.spacing.allow_rotations        = sp_j.value("allowRotations", true);
    { uint64_t s = raw.value("seed", uint64_t(0)); if (s != 0) { err.error.code="INVALID_INPUT"; err.error.message="seed not supported (capabilities.seeded_determinism=false)"; return false; } }

    if (!raw.contains("models") || !raw["models"].is_array()) { err.error.code="INVALID_INPUT"; err.error.message="models must be array"; return false; }
    const json& mods_j = raw["models"];
    if (mods_j.empty()) { err.error.code="INVALID_INPUT"; err.error.message="at least one model required"; return false; }
    for (const auto& m : mods_j) {
        if (!m.is_object()) { err.error.code="INVALID_INPUT"; err.error.message="model entry must be object"; return false; }
        if (!m.contains("id") || !m["id"].is_string()) { err.error.code="INVALID_INPUT"; err.error.message="model missing/non-string id"; return false; }
        if (!m.contains("path") || !m["path"].is_string()) { err.error.code="INVALID_INPUT"; err.error.message="model missing/non-string path"; return false; }
        ModelRef ref;
        ref.id     = m.value("id", "");
        ref.path   = m.value("path", "");
        ref.locked = m.value("locked", false);
        if (ref.id.empty()) { err.error.code="INVALID_INPUT"; err.error.message="model entry missing id"; return false; }
        for (auto& ex : out.models) { if (ex.id == ref.id) { err.error.code="INVALID_INPUT"; err.error.message="duplicate id '"+ref.id+"'"; err.error.object_ids={ref.id}; return false; } }
        if (ref.path.empty()) { err.error.code="INVALID_INPUT"; err.error.message="model '"+ref.id+"' missing path"; err.error.object_ids={ref.id}; return false; }
        const json& tx_j = m.contains("transform") ? m["transform"] : m;
        ref.x_mm = tx_j.value("x", tx_j.value("x_mm", 0.0));
        ref.y_mm = tx_j.value("y", tx_j.value("y_mm", 0.0));
        ref.z_mm = tx_j.value("z", tx_j.value("z_mm", 0.0));
        ref.rot_z_rad = tx_j.value("rotationZ", tx_j.value("rotation_rad", tx_j.value("rot_z_rad", 0.0)));
        if (m.contains("allowed_rotations") && m["allowed_rotations"].is_array() && !m["allowed_rotations"].empty()) {
            err.error.code="INVALID_INPUT"; err.error.message="allowed_rotations not supported (capabilities.rotation_constraints=false)";
            err.error.object_ids={ref.id}; return false;
        }
        out.models.push_back(ref);
    }
    return true;
}

// ─── Runner ──────────────────────────────────────────────────────────────────

int run_layout_plan(const LayoutProblemV1& problem) {
    const char* be =
#ifdef ENGINE_ORCA
        "orca";
#else
        "bambu";
#endif
    if (problem.engine != be) {
        LayoutErrorV1 err; err.error.code="ENGINE_MISMATCH";
        err.error.message = std::string("requested '")+problem.engine+"' but built for '"+be+"'";
        std::cerr << to_json(err).dump() << std::endl; return 2;
    }
    g_cancelled.store(false);
    std::signal(SIGINT, cancellation_handler);

    // Load profiles
    std::string dir = problem.profiles_dir;
    DynamicPrintConfig cfg;
    { std::string cp = dir + "/BBL/machine/fdm_bbl_3dp_001_common.json"; std::ifstream cf(cp); if (cf.good()) { cf.close(); load_with_inherits(cfg, cp); } }
    { std::string fp = dir + "/" + problem.profiles.machine; load_with_inherits(cfg, fp); }

    auto load_req = [&](const std::string& rel, bool req = true) -> bool {
        if (rel.empty()) return true;
        std::string fp = dir + "/" + rel;
        std::ifstream t(fp);
        if (!t.good()) {
            if (req) { LayoutErrorV1 err; err.error.code="INVALID_INPUT"; err.error.message="failed to open: "+rel; std::cerr<<to_json(err).dump()<<std::endl; return false; }
            return true;
        }
        t.close();
        if (!load_profile(fp, cfg)) {
            LayoutErrorV1 err; err.error.code="INVALID_INPUT"; err.error.message="failed to load: "+rel;
            std::cerr << to_json(err).dump() << std::endl; return false;
        }
        return true;
    };
    if (!load_req(problem.profiles.machine)) return 3;
    if (!load_req(problem.profiles.process, false)) return 3;
    if (!load_req(problem.profiles.filament, false)) return 3;

    // Load models
    Model unlocked_model, locked_model;
    for (auto& ref : problem.models) {
        try {
            Model m = Model::read_from_file(ref.path);
            for (ModelObject* mo : m.objects) {
                Model* tgt = ref.locked ? &locked_model : &unlocked_model;
                ModelObject* no = tgt->add_object(*mo);
                if (no->instances.empty()) no->add_instance();
                for (auto* inst : no->instances) {
                    inst->set_offset(Vec3d(ref.x_mm, ref.y_mm, ref.z_mm));
                    inst->set_rotation(Vec3d(0, 0, Geometry::rad2deg(ref.rot_z_rad)));
                }
            }
        } catch (const std::exception& e) {
            LayoutErrorV1 err; err.error.code="INVALID_INPUT";
            err.error.message = std::string("failed to load '")+ref.id+"': "+e.what();
            err.error.object_ids={ref.id}; std::cerr << to_json(err).dump() << std::endl; return 3;
        }
    }

    // Build params
    ArrangeParams params;
    double cl = 1.0;
    if (cfg.has("extruder_clearance_max_radius")) { double v = cfg.opt_float("extruder_clearance_max_radius"); if (v > 0) cl = v; }
#ifdef ENGINE_ORCA
    params.clearance_radius = cl;
#else
    params.cleareance_radius = cl;
#endif
    if (problem.spacing.clearance_radius_mm > 0) {
#ifdef ENGINE_ORCA
        params.clearance_radius = problem.spacing.clearance_radius_mm;
#else
        params.cleareance_radius = problem.spacing.clearance_radius_mm;
#endif
    }
    params.progressind = [](unsigned,std::string){};
    params.min_obj_distance = scaled<coord_t>(problem.spacing.min_object_distance_mm);
    params.allow_rotations  = problem.spacing.allow_rotations;
    params.do_final_align   = true;
    params.stopcondition    = [](){ return g_cancelled.load(); };

    // Extract polygons
    ModelInstancePtrs ui, li;
    auto unlocked_input = get_arrange_polys(unlocked_model, ui);
    auto locked_input   = get_arrange_polys(locked_model, li);

    // Arrange
#ifdef ENGINE_ORCA
    update_arrange_params(params, &cfg, unlocked_input);
    update_selected_items_inflation(unlocked_input, &cfg, params);
    update_selected_items_inflation(locked_input, &cfg, params);
    Points bed_pts = get_shrink_bedpts(&cfg, params);
#else
    update_arrange_params(params, cfg, unlocked_input);
    update_selected_items_inflation(unlocked_input, cfg, params);
    update_selected_items_inflation(locked_input, cfg, params);
    Points bed_pts = get_shrink_bedpts(cfg, params);
#endif
    if (bed_pts.size() < 3) {
        LayoutErrorV1 err; err.error.code="INVALID_INPUT";
        err.error.message="bed degenerate"; std::cerr << to_json(err).dump() << std::endl; return 3;
    }
    { BoundingBox bb(bed_pts); if (bb.size().x()<=0 || bb.size().y()<=0) {
        LayoutErrorV1 err; err.error.code="INVALID_INPUT"; err.error.message="bed zero dims";
        std::cerr << to_json(err).dump() << std::endl; return 3;
    }}
    arrangement::arrange(unlocked_input, locked_input, bed_pts, params);

    if (unlocked_input.size() != unlocked_model.objects.size()) {
        LayoutErrorV1 err; err.error.code="INVALID_INPUT";
        err.error.message="count mismatch: "+std::to_string(unlocked_model.objects.size())+" models, "+std::to_string(unlocked_input.size())+" polys";
        std::cerr << to_json(err).dump() << std::endl; return 3;
    }
    if (g_cancelled.load()) {
        LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled";
        std::cerr << to_json(err).dump() << std::endl; return 5;
    }

    // Build result
    PlacementCandidateV1 result;
    result.engine = be;
    Polygon bed_poly(bed_pts);
    std::vector<std::string> unfittable;
    size_t ui2 = 0;

    for (auto& ref : problem.models) {
        if (!ref.locked) continue;
        PlacedModel pm; pm.id = ref.id;
        pm.bed_idx = 0; pm.x_mm = ref.x_mm; pm.y_mm = ref.y_mm; pm.rotation_rad = ref.rot_z_rad;
        { static size_t _li = 0;
          if (_li < locked_input.size()) {
              BoundingBox bb = locked_input[_li].transformed_poly().contour.bounding_box();
              pm.bb_cx_mm = unscaled<double>((double)bb.min.x()+(double)bb.max.x())/2.0;
              pm.bb_cy_mm = unscaled<double>((double)bb.min.y()+(double)bb.max.y())/2.0;
              _li++;
          }
        }
        result.placements.push_back(pm);
    }
    for (auto& ref : problem.models) {
        if (ref.locked) continue;
        if (ui2 >= unlocked_input.size()) break;
        auto& ap = unlocked_input[ui2++];
        BoundingBox bb = ap.transformed_poly().contour.bounding_box();
        double cx = unscaled<double>((double)bb.min.x()+(double)bb.max.x())/2.0;
        double cy = unscaled<double>((double)bb.min.y()+(double)bb.max.y())/2.0;
        PlacedModel pm; pm.id = ref.id;
        bool bad = (ap.bed_idx != 0);
        if (!bad) {
            Point corners[] = {Point(bb.min.x(),bb.min.y()),Point(bb.max.x(),bb.min.y()),
                              Point(bb.max.x(),bb.max.y()),Point(bb.min.x(),bb.max.y())};
            for (auto& c : corners) { if (!bed_poly.contains(c)) { bad=true; break; } }
        }
        if (bad) { pm.bed_idx = -1; unfittable.push_back(ref.id); }
        else { pm.bed_idx=ap.bed_idx; pm.x_mm=cx; pm.y_mm=cy; pm.rotation_rad=ap.rotation; pm.bb_cx_mm=cx; pm.bb_cy_mm=cy; }
        result.placements.push_back(pm);
    }
    if (!unfittable.empty()) {
        LayoutErrorV1 err; err.error.code="UNFITTABLE";
        err.error.message="some objects could not be placed"; err.error.object_ids=unfittable;
        std::cerr << to_json(err).dump() << std::endl;
        std::cout << to_json(result).dump() << std::endl; return 4;
    }
    std::cout << to_json(result).dump() << std::endl;
    return 0;
}

}  // namespace layout_plan
