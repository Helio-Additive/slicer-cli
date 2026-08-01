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
#include "libslic3r/ClipperUtils.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <unordered_set>

namespace layout_plan {

using json = nlohmann::json;
using namespace Slic3r;
using namespace Slic3r::arrangement;

// ─── Cancellation ────────────────────────────────────────────────────────────
static std::atomic<bool> g_cancelled{false};
static void cancellation_handler(int) { g_cancelled.store(true); }

void install_cancellation_handler() {
    std::signal(SIGINT, cancellation_handler);
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

static constexpr int kMaxInheritsDepth = 16;

// return: 0 = ok, 1 = inheritance depth limit exceeded (incomplete chain), cycle = clean stop
static int load_profile_json(const std::string& fp, DynamicPrintConfig& cfg,
                             std::unordered_set<std::string>& visited, int depth) {
    if (visited.count(fp)) return 0;               // cycle → already loaded ancestors, clean stop
    if (depth > kMaxInheritsDepth) return 1;       // non-cyclic deep chain → truncated, error
    visited.insert(fp);
    std::ifstream f(fp); if (!f.is_open()) return 1;
    json j; try { j = json::parse(f); } catch (...) { return 1; }
    // resolve inherits chain first (parent overrides child's keys below)
    if (j.contains("inherits")) {
        std::string v;
        auto& iv = j["inherits"];
        if (iv.is_string()) v = iv.get<std::string>();
        else if (iv.is_array() && !iv.empty() && iv[0].is_string()) v = iv[0].get<std::string>();
        if (!v.empty()) {
            size_t pos = fp.find_last_of("/\\");  // native separators on Windows
            std::string pp = fp.substr(0, pos) + "/" + v;
            if (pp.size() <= 5 || pp.compare(pp.size()-5, 5, ".json") != 0) pp += ".json";
            int rc = load_profile_json(pp, cfg, visited, depth + 1);
            if (rc != 0) return rc;               // propagate depth error
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
    return 0;
}

static int load_with_inherits(DynamicPrintConfig& cfg, const std::string& fp) {
    std::unordered_set<std::string> visited;
    return load_profile_json(fp, cfg, visited, 0);
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
    // ENGINE_COMMIT is baked by CMake from the engine submodule git SHA.
    caps.engine_commit  =
#ifdef ENGINE_COMMIT
        ENGINE_COMMIT;
#else
        "unknown";
#endif
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
    try {
        if (!raw.is_object()) { err.error.code="INVALID_INPUT"; err.error.message="root must be JSON object"; return false; }
        if (!raw.contains("schemaVersion") || !raw["schemaVersion"].is_number_integer()) {
            err.error.code="INVALID_INPUT"; err.error.message="schemaVersion must be an integer"; return false;
        }
        int sv = raw["schemaVersion"].get<int>();
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
        if (out.spacing.min_object_distance_mm < 0) {
            err.error.code="INVALID_INPUT"; err.error.message="minObjectDistanceMm must be >= 0"; return false;
        }
        out.spacing.clearance_radius_mm    = sp_j.value("clearanceRadiusMm", 0.0);
        if (out.spacing.clearance_radius_mm < 0) {
            err.error.code="INVALID_INPUT"; err.error.message="clearanceRadiusMm must be >= 0"; return false;
        }
        out.spacing.allow_rotations = sp_j.value("allowRotations", true);
        if (raw.contains("seed")) {
            if (!raw["seed"].is_number_unsigned()) { err.error.code="INVALID_INPUT"; err.error.message="seed must be a non-negative integer"; return false; }
            out.seed = raw["seed"].get<uint64_t>(); // accepted-and-recorded; engine is inherently deterministic
        }
        if (!raw.contains("models") || !raw["models"].is_array()) { err.error.code="INVALID_INPUT"; err.error.message="models must be array"; return false; }
        const json& mods_j = raw["models"];
        if (mods_j.empty()) { err.error.code="INVALID_INPUT"; err.error.message="at least one model required"; return false; }
        for (const auto& m : mods_j) {
            if (!m.is_object()) { err.error.code="INVALID_INPUT"; err.error.message="model entry must be object"; return false; }
            if (!m.contains("id") || !m["id"].is_string()) { err.error.code="INVALID_INPUT"; err.error.message="model missing/non-string id"; return false; }
            if (!m.contains("path") || !m["path"].is_string()) { err.error.code="INVALID_INPUT"; err.error.message="model missing/non-string path"; return false; }
            // refuse allowed_rotations on PRESENCE (any type)
            if (m.contains("allowed_rotations")) {
                err.error.code="INVALID_INPUT"; err.error.message="allowed_rotations not supported (capabilities.rotation_constraints=false)";
                err.error.object_ids={m.value("id","")}; return false;
            }
            ModelRef ref;
            ref.id     = m.value("id", "");
            ref.path   = m.value("path", "");
            if (m.contains("locked")) {
                if (!m["locked"].is_boolean()) { err.error.code="INVALID_INPUT"; err.error.message="model '"+ref.id+"' locked must be boolean"; err.error.object_ids={ref.id}; return false; }
                ref.locked = m["locked"].get<bool>();
            }
            if (ref.id.empty()) { err.error.code="INVALID_INPUT"; err.error.message="model entry missing id"; return false; }
            for (auto& ex : out.models) { if (ex.id == ref.id) { err.error.code="INVALID_INPUT"; err.error.message="duplicate id '"+ref.id+"'"; err.error.object_ids={ref.id}; return false; } }
            if (ref.path.empty()) { err.error.code="INVALID_INPUT"; err.error.message="model '"+ref.id+"' missing path"; err.error.object_ids={ref.id}; return false; }
            // transform: nested object or flat top-level fields; track presence to preserve embedded transforms
            bool has_override = false;
            if (m.contains("transform")) {
                if (!m["transform"].is_object()) { err.error.code="INVALID_INPUT"; err.error.message="model '"+ref.id+"' transform must be object"; err.error.object_ids={ref.id}; return false; }
                const json& tx = m["transform"];
                ref.has_x = tx.contains("x");   ref.x_mm = tx.value("x", 0.0);
                ref.has_y = tx.contains("y");   ref.y_mm = tx.value("y", 0.0);
                ref.has_z = tx.contains("z");   ref.z_mm = tx.value("z", 0.0);
                ref.has_rot = tx.contains("rotationZ") || tx.contains("rotation_rad") || tx.contains("rot_z_rad");
                ref.rot_z_rad = tx.value("rotationZ", tx.value("rotation_rad", tx.value("rot_z_rad", 0.0)));
                has_override = ref.has_x || ref.has_y || ref.has_z || ref.has_rot;
            } else {
                ref.has_x = m.contains("x_mm");  ref.x_mm = m.value("x_mm", 0.0);
                ref.has_y = m.contains("y_mm");  ref.y_mm = m.value("y_mm", 0.0);
                ref.has_z = m.contains("z_mm");  ref.z_mm = m.value("z_mm", 0.0);
                ref.has_rot = m.contains("rotationZ") || m.contains("rotation_rad") || m.contains("rot_z_rad");
                ref.rot_z_rad = m.value("rotationZ", m.value("rotation_rad", m.value("rot_z_rad", 0.0)));
                has_override = ref.has_x || ref.has_y || ref.has_z || ref.has_rot;
            }
            ref.has_override = has_override;
            out.models.push_back(ref);
        }
        return true;
    } catch (const std::exception& e) {
        err.error.code="INVALID_INPUT";
        err.error.message=std::string("malformed input: ")+e.what();
        return false;
    }
}

// ─── Runner ──────────────────────────────────────────────────────────────────

// Edge-aware bed containment: clip the object's contour against the bed polygon
// and compare area. Vertex-in-polygon alone misses edges crossing a recess of a
// concave (e.g. U-shaped) bed; clipping detects every crossing edge.
static bool fully_inside_bed(const ExPolygon& obj, const Polygon& bed_poly) {
    Slic3r::Polygons obj_polys = to_polygons(obj);
    if (obj_polys.empty()) return false;
    double obj_area = std::abs(Polygon(obj_polys.front().points).area());
    Slic3r::Polygons clipped = intersection(obj_polys.front(), bed_poly);
    double clip_area = 0.0;
    for (auto& p : clipped) clip_area += std::abs(p.area());
    return (obj_area - clip_area) <= scaled<double>(0.01) * scaled<double>(0.01);
}

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
    if (g_cancelled.load()) {
        LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled";
        std::cerr << to_json(err).dump() << std::endl; return 5;
    }

    // seed is accepted-and-recorded: the arrange pipeline (subplex + firstfit,
    // per-index parallel writes) consumes no randomness, so output is
    // inherently deterministic — same input+seed → byte-identical output.
    (void)problem.seed;

    // Load profiles
    std::string dir = problem.profiles_dir;
    DynamicPrintConfig cfg;
    { std::string cp = dir + "/BBL/machine/fdm_bbl_3dp_001_common.json"; std::ifstream cf(cp); if (cf.good()) { cf.close(); load_with_inherits(cfg, cp); } }
    { std::string fp = dir + "/" + problem.profiles.machine;
      std::ifstream mf(fp);
      if (!mf.good()) {
          LayoutErrorV1 err; err.error.code="INVALID_INPUT"; err.error.message="failed to open machine profile: "+problem.profiles.machine;
          std::cerr << to_json(err).dump() << std::endl; return 3;
      }
      mf.close();
      int rc = load_with_inherits(cfg, fp);
      if (rc != 0) {
          LayoutErrorV1 err; err.error.code="INVALID_INPUT";
          err.error.message = (rc == 1) ? "inheritance chain for machine profile exceeds depth limit: "+problem.profiles.machine
                                        : "failed to load machine profile: "+problem.profiles.machine;
          std::cerr << to_json(err).dump() << std::endl; return 3;
      }
    }

    auto load_opt = [&](const std::string& rel) -> bool {
        if (rel.empty()) return true;  // empty field = explicitly skipped
        std::string fp = dir + "/" + rel;
        std::ifstream t(fp);
        if (!t.good()) {
            // supplied-but-absent is a typed error, same as supplied-but-invalid
            LayoutErrorV1 err; err.error.code="INVALID_INPUT"; err.error.message="failed to open profile: "+rel;
            std::cerr << to_json(err).dump() << std::endl; return false;
        }
        t.close();
        int rc = load_with_inherits(cfg, fp);
        if (rc != 0) {
            LayoutErrorV1 err; err.error.code="INVALID_INPUT";
            err.error.message = (rc == 1) ? "inheritance chain for profile exceeds depth limit: "+rel
                                          : "failed to load profile: "+rel;
            std::cerr << to_json(err).dump() << std::endl; return false;
        }
        return true;
    };
    if (!load_opt(problem.profiles.process)) return 3;
    if (!load_opt(problem.profiles.filament)) return 3;

    // Load models; count polygons per ref for identity mapping
    Model unlocked_model, locked_model;
    struct RefCount { std::string id; size_t count; double rot = 0.0; };
    std::vector<RefCount> unlocked_counts, locked_counts;
    for (auto& ref : problem.models) {
        if (g_cancelled.load()) {
            LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during model load";
            std::cerr << to_json(err).dump() << std::endl; return 5;
        }
        try {
            Model m = Model::read_from_file(ref.path);
            size_t n = 0;
            for (ModelObject* mo : m.objects) {
                Model* tgt = ref.locked ? &locked_model : &unlocked_model;
                ModelObject* no = tgt->add_object(*mo);
                if (no->instances.empty()) no->add_instance();
                for (auto* inst : no->instances) {
                    if (ref.has_override) {
                        Vec3d off = inst->get_offset();
                        if (ref.has_x) off.x() = ref.x_mm;
                        if (ref.has_y) off.y() = ref.y_mm;
                        if (ref.has_z) off.z() = ref.z_mm;
                        inst->set_offset(off);
                        if (ref.has_rot) {
                            Vec3d rot = inst->get_rotation();
                            rot.z() = ref.rot_z_rad; // radians
                            inst->set_rotation(rot);
                        }
                    } // else: preserve embedded instance transform
                    n++;
                }
            }
            if (ref.locked) {
                RefCount rc{ref.id, n, ref.rot_z_rad};
                // if no override, read the preserved embedded rotation
                if (!ref.has_rot) {
                    for (auto& mo2 : locked_model.objects) {
                        for (auto* inst : mo2->instances) {
                            Vec3d r = inst->get_rotation();
                            rc.rot = r.z();
                        }
                    }
                }
                locked_counts.push_back(rc);
            } else {
                unlocked_counts.push_back({ref.id, n});
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

    // F3: total per-class polygon count must match ref sums (zero-area/dropped
    // polygons are typed errors, not silent drops)
    {
        size_t exp_u = 0; for (auto& rc : unlocked_counts) exp_u += rc.count;
        size_t exp_l = 0; for (auto& rc : locked_counts)   exp_l += rc.count;
        if (unlocked_input.size() != exp_u) {
            LayoutErrorV1 err; err.error.code="INVALID_INPUT";
            err.error.message="unlocked polygon count mismatch: expected "+std::to_string(exp_u)+", got "+std::to_string(unlocked_input.size())+" (zero-area model?)";
            std::cerr << to_json(err).dump() << std::endl; return 3;
        }
        if (locked_input.size() != exp_l) {
            LayoutErrorV1 err; err.error.code="INVALID_INPUT";
            err.error.message="locked polygon count mismatch: expected "+std::to_string(exp_l)+", got "+std::to_string(locked_input.size())+" (zero-area model?)";
            std::cerr << to_json(err).dump() << std::endl; return 3;
        }
        // zero-footprint polygons (degenerate/flat meshes) are typed errors
        size_t li3 = 0;
        for (auto& rc : locked_counts) {
            if (li3 < locked_input.size() &&
                std::abs(locked_input[li3].transformed_poly().contour.area()) < scaled<double>(0.01) * scaled<double>(0.01)) {
                LayoutErrorV1 err; err.error.code="INVALID_INPUT";
                err.error.message="locked model '"+rc.id+"' has zero footprint";
                err.error.object_ids={rc.id}; std::cerr << to_json(err).dump() << std::endl; return 3;
            }
            li3++;
        }
        size_t ui3 = 0;
        for (auto& rc : unlocked_counts) {
            if (ui3 < unlocked_input.size() &&
                std::abs(unlocked_input[ui3].transformed_poly().contour.area()) < scaled<double>(0.01) * scaled<double>(0.01)) {
                LayoutErrorV1 err; err.error.code="INVALID_INPUT";
                err.error.message="model '"+rc.id+"' has zero footprint";
                err.error.object_ids={rc.id}; std::cerr << to_json(err).dump() << std::endl; return 3;
            }
            ui3++;
        }
    }

    // Identity mapping: each ref must produce exactly one polygon (multi-polygon files refused)
    for (auto& rc : unlocked_counts)
        if (rc.count != 1) {
            LayoutErrorV1 err; err.error.code="INVALID_INPUT";
            err.error.message="model '"+rc.id+"' expands to "+std::to_string(rc.count)+" polygons (expected 1); multi-object files unsupported";
            err.error.object_ids={rc.id}; std::cerr << to_json(err).dump() << std::endl; return 3;
        }
    for (auto& rc : locked_counts)
        if (rc.count != 1) {
            LayoutErrorV1 err; err.error.code="INVALID_INPUT";
            err.error.message="locked model '"+rc.id+"' expands to "+std::to_string(rc.count)+" polygons (expected 1)";
            err.error.object_ids={rc.id}; std::cerr << to_json(err).dump() << std::endl; return 3;
        }

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
    Polygon bed_poly(bed_pts);
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
    std::vector<std::string> unfittable;
    struct LockedRec { std::string id; ExPolygon poly; coord_t inflation; };
    std::vector<LockedRec> locked_placed;
    size_t ui2 = 0, li2 = 0;

    for (auto& ref : problem.models) {
        PlacedModel pm; pm.id = ref.id;
        if (ref.locked) {
            // locked: identity-mapped to its own polygon
            if (li2 >= locked_input.size()) { pm.bed_idx=-1; unfittable.push_back(ref.id); result.placements.push_back(pm); continue; }
            double lrot = (li2 < locked_counts.size()) ? locked_counts[li2].rot : 0.0;
            auto& ap = locked_input[li2++];
            BoundingBox bb = ap.transformed_poly().contour.bounding_box();
            double cx = unscaled<double>((double)bb.min.x()+(double)bb.max.x())/2.0;
            double cy = unscaled<double>((double)bb.min.y()+(double)bb.max.y())/2.0;
            // edge-aware containment on the INFLATED footprint (brim/profile clearance):
            // a locked object whose inflation extends past the bed edge is UNFITTABLE
            {
                ExPolygon inflated = ap.transformed_poly();
                Slic3r::Polygons off = offset(to_polygons(inflated).front(), float(ap.inflation));
                if (!off.empty())
                    inflated = ExPolygon(off.front());
                if (!fully_inside_bed(inflated, bed_poly)) {
                    pm.bed_idx=-1; unfittable.push_back(ref.id); result.placements.push_back(pm); continue;
                }
            }
            locked_placed.push_back({ref.id, ap.transformed_poly(), ap.inflation});
            pm.bed_idx = 0;
            // effective transform: request override or preserved embedded
            pm.x_mm = unscaled<double>(ap.translation.x());
            pm.y_mm = unscaled<double>(ap.translation.y());
            pm.rotation_rad = lrot;
            pm.bb_cx_mm = cx; pm.bb_cy_mm = cy;
            result.placements.push_back(pm);
        } else {
            if (ui2 >= unlocked_input.size()) { pm.bed_idx=-1; unfittable.push_back(ref.id); result.placements.push_back(pm); continue; }
            auto& ap = unlocked_input[ui2++];
            BoundingBox bb = ap.transformed_poly().contour.bounding_box();
            double cx = unscaled<double>((double)bb.min.x()+(double)bb.max.x())/2.0;
            double cy = unscaled<double>((double)bb.min.y()+(double)bb.max.y())/2.0;
            bool bad = (ap.bed_idx != 0);
            if (!bad) bad = !fully_inside_bed(ap.transformed_poly(), bed_poly);
            if (bad) { pm.bed_idx=-1; unfittable.push_back(ref.id); }
            else { pm.bed_idx=ap.bed_idx; pm.x_mm=cx; pm.y_mm=cy; pm.rotation_rad=ap.rotation; pm.bb_cx_mm=cx; pm.bb_cy_mm=cy; }
            result.placements.push_back(pm);
        }
    }
    // F4: honour late SIGINT during validation — no partial stdout
    if (g_cancelled.load()) {
        LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during validation";
        std::cerr << to_json(err).dump() << std::endl; return 5;
    }

    // F1: locked-vs-locked clearance — inflate each locked contour by half the
    // minimum spacing; overlapping INFLATED contours mean the pair violates
    // minObjectDistanceMm (or overlaps outright) → UNFITTABLE naming both.
    {
        double spacing_infl = scaled<double>(problem.spacing.min_object_distance_mm) / 2.0;
        std::vector<Polygon> inflated;
        for (auto& rec : locked_placed) {
            if (g_cancelled.load()) {
                LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during validation";
                std::cerr << to_json(err).dump() << std::endl; return 5;
            }
            // actual inflation = max of spacing-derived and profile-derived (brim/clearance)
            double eff = std::max(spacing_infl, double(rec.inflation));
            Slic3r::Polygons off = offset(to_polygons(rec.poly).front(), float(eff));
            inflated.emplace_back(off.empty() ? Polygon(rec.poly.contour.points) : off.front());
            if (g_cancelled.load()) {  // after the last offset call (single-locked-model path)
                LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during validation";
                std::cerr << to_json(err).dump() << std::endl; return 5;
            }
        }
        for (size_t a = 0; a < inflated.size(); ++a)
            for (size_t b = a+1; b < inflated.size(); ++b) {
                if (g_cancelled.load()) {
                    LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during validation";
                    std::cerr << to_json(err).dump() << std::endl; return 5;
                }
                Slic3r::Polygons ia = intersection(inflated[a], inflated[b]);
                if (g_cancelled.load()) {  // intersection may straddle SIGINT
                    LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during validation";
                    std::cerr << to_json(err).dump() << std::endl; return 5;
                }
                if (!ia.empty()) {
                    if (std::find(unfittable.begin(), unfittable.end(), locked_placed[a].id) == unfittable.end())
                        unfittable.push_back(locked_placed[a].id);
                    if (std::find(unfittable.begin(), unfittable.end(), locked_placed[b].id) == unfittable.end())
                        unfittable.push_back(locked_placed[b].id);
                }
            }
    }

    // F2: entries for overlapping/too-close locked models must be unplaced
    // (bed_idx=-1, no coordinates) in the emitted candidate
    for (auto& pm : result.placements)
        if (std::find(unfittable.begin(), unfittable.end(), pm.id) != unfittable.end())
            pm.bed_idx = -1;

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
