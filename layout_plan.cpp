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
#ifdef _WIN32
#include <windows.h>
#include <io.h>
#include <fcntl.h>
#else
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#endif
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
#ifdef _WIN32
    // Windows delivers console Ctrl+C on a separate thread; blocking reads are
    // not restarted the same way, so the plain handler suffices.
    std::signal(SIGINT, cancellation_handler);
#else
    // no SA_RESTART: an interrupted read returns EINTR so the stdin poll loop
    // can observe the cancellation flag promptly instead of staying blocked
    struct sigaction sa{};
    sa.sa_handler = cancellation_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    // a closed stdout pipe must surface as a write error (checked after
    // emission), not as a SIGPIPE death that skips the typed error
    signal(SIGPIPE, SIG_IGN);
#endif
}

bool is_cancelled() {
    return g_cancelled.load();
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

static constexpr int kMaxInheritsDepth = 16;

static int load_profile_json(const std::string& fp, DynamicPrintConfig& cfg,
                             std::unordered_set<std::string>& visited, int depth);

// Slurp a profile through a cancellable reader. 0=ok, 1=cancelled, 2=hard error.
// Windows: PeekNamedPipe polling (same pattern as the stdin reader) so a
// stalled named-pipe profile observes Ctrl+C instead of blocking in _read.
// POSIX: EINTR loop (sigaction has no SA_RESTART). Single open — the slurp
// IS the existence check; no separate probe open that could consume a stream.
static int slurp_profile_cancellable(const std::string& fp, std::string& out) {
#ifdef _WIN32
    int fd = ::_open(fp.c_str(), _O_RDONLY | _O_BINARY);
    if (fd < 0) return 2;
    char buf[4096];
    for (;;) {
        if (g_cancelled.load()) { ::_close(fd); return 1; }
        HANDLE h = reinterpret_cast<HANDLE>(_get_osfhandle(fd));
        if (h != INVALID_HANDLE_VALUE && GetFileType(h) == FILE_TYPE_PIPE) {
            DWORD avail = 0;
            if (PeekNamedPipe(h, nullptr, 0, nullptr, &avail, nullptr) && avail == 0) {
                Sleep(50);
                continue;
            }
        }
        int n = ::_read(fd, buf, static_cast<unsigned>(sizeof buf));
        if (n < 0) { ::_close(fd); return 2; }
        if (n == 0) { ::_close(fd); return 0; }
        out.append(buf, static_cast<size_t>(n));
    }
#else
    // opening a FIFO blocks until a writer connects; loop on EINTR and treat
    // it as a cancellation check point (same pattern as the --input open)
    int fd = -1;
    for (;;) {
        if (g_cancelled.load()) return 1;
        fd = ::open(fp.c_str(), O_RDONLY);
        if (fd < 0 && errno == EINTR) {
            if (g_cancelled.load()) return 1;
            continue;
        }
        break;
    }
    if (fd < 0) return 2;
    char buf[4096];
    for (;;) {
        if (g_cancelled.load()) { ::close(fd); return 1; }
        ssize_t n = ::read(fd, buf, sizeof buf);
        if (n < 0 && errno == EINTR) continue;   // SIGINT → recheck flag next
        if (n < 0) { ::close(fd); return 2; }
        if (n == 0) { ::close(fd); return 0; }
        out.append(buf, static_cast<size_t>(n));
    }
#endif
}

// return: 0 = ok, 1 = inheritance depth limit exceeded (incomplete chain), cycle = clean stop
// parses the slurped content; the root file was opened ONCE by slurp_profile_cancellable
static int load_profile_json_impl(const std::string& content, const std::string& fp, DynamicPrintConfig& cfg,
                                  std::unordered_set<std::string>& visited, int depth) {
    if (g_cancelled.load()) return 3;              // honour SIGINT during slow chains
    if (visited.count(fp)) return 0;               // cycle → already loaded ancestors, clean stop
    if (depth > kMaxInheritsDepth) return 1;       // non-cyclic deep chain → truncated, error
    visited.insert(fp);
    json j; try { j = json::parse(content); } catch (...) { if (g_cancelled.load()) return 3; return 2; }
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
            if (rc != 0) return rc;               // propagate depth/bad-file/cancel
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

// path variant: slurps the file once (open-failure IS the existence signal)
// then delegates; the slurp reader is cancellable on both platforms
static int load_profile_json(const std::string& fp, DynamicPrintConfig& cfg,
                             std::unordered_set<std::string>& visited, int depth) {
    std::string content;
    int rr = slurp_profile_cancellable(fp, content);
    if (rr == 1) return 3;                       // cancelled
    if (rr == 2) return 2;                       // bad file (missing/malformed)
    return load_profile_json_impl(content, fp, cfg, visited, depth);
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
    if (!std::cout.good()) {  // hard write failure (e.g. /dev/full)
        if (g_cancelled.load()) {  // cancel takes precedence over the write failure
            LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled";
            std::cerr << to_json(err).dump() << std::endl; return 5;
        }
        LayoutErrorV1 err; err.error.code="WRITE_FAILED"; err.error.message="failed to write capabilities to stdout";
        std::cerr << to_json(err).dump() << std::endl; return 6;
    }
    if (g_cancelled.load()) {  // final recheck per convention
        LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled";
        std::cerr << to_json(err).dump() << std::endl; return 5;
    }
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
        // numeric range guard: finite and sane-bounded — huge/NaN/inf spacing
        // would overflow scaled<coord_t> downstream (1e100 is finite but
        // already overflows; NaN/inf corrupt the arrange geometry)
        constexpr double kMaxSpacingMm = 10000.0;
        out.spacing.min_object_distance_mm = sp_j.value("minObjectDistanceMm", sp_j.value("min_mm", 10.0));
        if (!std::isfinite(out.spacing.min_object_distance_mm) || out.spacing.min_object_distance_mm < 0 ||
            out.spacing.min_object_distance_mm > kMaxSpacingMm) {
            err.error.code="INVALID_INPUT"; err.error.message="minObjectDistanceMm must be finite and in [0, 10000] mm"; return false;
        }
        out.spacing.clearance_radius_mm    = sp_j.value("clearanceRadiusMm", 0.0);
        if (!std::isfinite(out.spacing.clearance_radius_mm) || out.spacing.clearance_radius_mm < 0 ||
            out.spacing.clearance_radius_mm > kMaxSpacingMm) {
            err.error.code="INVALID_INPUT"; err.error.message="clearanceRadiusMm must be finite and in [0, 10000] mm"; return false;
        }
        out.spacing.allow_rotations = sp_j.value("allowRotations", true);
        if (raw.contains("seed")) {
            if (sv < 2) { err.error.code="INVALID_INPUT"; err.error.message="field 'seed' requires schemaVersion 2"; return false; }
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
                if (sv < 2) { err.error.code="INVALID_INPUT"; err.error.message="model '"+ref.id+"' locked requires schemaVersion 2"; err.error.object_ids={ref.id}; return false; }
                if (!m["locked"].is_boolean()) { err.error.code="INVALID_INPUT"; err.error.message="model '"+ref.id+"' locked must be boolean"; err.error.object_ids={ref.id}; return false; }
                ref.locked = m["locked"].get<bool>();
            }
            if (ref.id.empty()) { err.error.code="INVALID_INPUT"; err.error.message="model entry missing id"; return false; }
            for (auto& ex : out.models) {  // O(n^2) duplicate scan — poll cancel inside
                if (g_cancelled.load()) {
                    err.error.code="CANCELLED"; err.error.message="cancelled during input validation";
                    return false;
                }
                if (ex.id == ref.id) { err.error.code="INVALID_INPUT"; err.error.message="duplicate id '"+ref.id+"'"; err.error.object_ids={ref.id}; return false; }
            }
            if (ref.path.empty()) { err.error.code="INVALID_INPUT"; err.error.message="model '"+ref.id+"' missing path"; err.error.object_ids={ref.id}; return false; }
            // transform: nested object or flat top-level fields; track presence to preserve embedded transforms
            bool has_override = false;
            if (m.contains("transform")) {
                if (!m["transform"].is_object()) { err.error.code="INVALID_INPUT"; err.error.message="model '"+ref.id+"' transform must be object"; err.error.object_ids={ref.id}; return false; }
                const json& tx = m["transform"];
                // both spellings accepted in both branches (nested and flat):
                // x_mm/y_mm/z_mm primary (flat convention), x/y/z legacy fallback
                ref.has_x = tx.contains("x_mm") || tx.contains("x");   ref.x_mm = tx.value("x_mm", tx.value("x", 0.0));
                ref.has_y = tx.contains("y_mm") || tx.contains("y");   ref.y_mm = tx.value("y_mm", tx.value("y", 0.0));
                ref.has_z = tx.contains("z_mm") || tx.contains("z");   ref.z_mm = tx.value("z_mm", tx.value("z", 0.0));
                ref.has_rot = tx.contains("rotationZ") || tx.contains("rotation_rad") || tx.contains("rot_z_rad");
                ref.rot_z_rad = tx.value("rotationZ", tx.value("rotation_rad", tx.value("rot_z_rad", 0.0)));
                has_override = ref.has_x || ref.has_y || ref.has_z || ref.has_rot;
            } else {
                ref.has_x = m.contains("x_mm") || m.contains("x");  ref.x_mm = m.value("x_mm", m.value("x", 0.0));
                ref.has_y = m.contains("y_mm") || m.contains("y");  ref.y_mm = m.value("y_mm", m.value("y", 0.0));
                ref.has_z = m.contains("z_mm") || m.contains("z");  ref.z_mm = m.value("z_mm", m.value("z", 0.0));
                ref.has_rot = m.contains("rotationZ") || m.contains("rotation_rad") || m.contains("rot_z_rad");
                ref.rot_z_rad = m.value("rotationZ", m.value("rotation_rad", m.value("rot_z_rad", 0.0)));
                has_override = ref.has_x || ref.has_y || ref.has_z || ref.has_rot;
            }
            // numeric range guard (mirrors spacing): positions must be finite
            // and |v| <= 10000 mm (huge values overflow scaled<coord_t>);
            // rotation must be finite (NaN/inf corrupt the geometry)
            constexpr double kMaxCoordMm = 10000.0;
            auto bad_val = [](double v, bool is_pos) {
                if (!std::isfinite(v)) return true;
                return is_pos && (v < -kMaxCoordMm || v > kMaxCoordMm);
            };
            if ((ref.has_x && bad_val(ref.x_mm, true)) || (ref.has_y && bad_val(ref.y_mm, true)) ||
                (ref.has_z && bad_val(ref.z_mm, true)) || (ref.has_rot && bad_val(ref.rot_z_rad, false))) {
                err.error.code="INVALID_INPUT"; err.error.message="model '"+ref.id+"' transform coordinates must be finite (|x_mm/y_mm/z_mm| <= 10000, rotation finite)";
                err.error.object_ids={ref.id}; return false;
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
    // hole-aware: clip the full ExPolygon (outer + holes) against the bed and
    // compare solid areas — holes are preserved, not dropped via to_polygons().front()
    ExPolygon bed_exp(bed_poly);
    double obj_area = std::abs(obj.area());
    Slic3r::ExPolygons clipped = intersection_ex(obj, bed_exp);
    double clip_area = 0.0;
    for (auto& e : clipped) clip_area += std::abs(e.area());
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
    { std::string cp = dir + "/BBL/machine/fdm_bbl_3dp_001_common.json";
      std::string content; if (slurp_profile_cancellable(cp, content) == 0) { std::unordered_set<std::string> vis; load_profile_json_impl(content, cp, cfg, vis, 0); } }
    { std::string fp = dir + "/" + problem.profiles.machine;
      std::string content;
      int rr = slurp_profile_cancellable(fp, content);  // single open; slurp failure IS the existence signal
      if (rr == 2) {
          if (g_cancelled.load()) {  // SIGINT-interrupted open → cancel, not bad-file
              LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during profile load";
              std::cerr << to_json(err).dump() << std::endl; return 5;
          }
          LayoutErrorV1 err; err.error.code="INVALID_INPUT"; err.error.message="failed to open machine profile: "+problem.profiles.machine;
          std::cerr << to_json(err).dump() << std::endl; return 3;
      }
      if (rr == 1) {
          LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during profile load";
          std::cerr << to_json(err).dump() << std::endl; return 5;
      }
      std::unordered_set<std::string> vis;
      int rc = load_profile_json_impl(content, fp, cfg, vis, 0);
      if (rc == 3) {
          LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during profile load";
          std::cerr << to_json(err).dump() << std::endl; return 5;
      }
      if (rc != 0) {
          LayoutErrorV1 err; err.error.code="INVALID_INPUT";
          err.error.message = (rc == 1) ? "inheritance chain for machine profile exceeds depth limit: "+problem.profiles.machine
                                        : "failed to load machine profile: "+problem.profiles.machine;
          std::cerr << to_json(err).dump() << std::endl; return 3;
      }
    }

    // returns 0 ok, 1 bad (typed INVALID_INPUT), 2 cancelled (exit 5)
    auto load_opt = [&](const std::string& rel) -> int {
        if (rel.empty()) return 0;  // empty field = explicitly skipped
        std::string fp = dir + "/" + rel;
        std::string content;
        int rr = slurp_profile_cancellable(fp, content);  // single open; slurp failure IS the existence signal
        if (rr == 2) {
            if (g_cancelled.load()) {  // SIGINT-interrupted open → cancel, not bad-file
                LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during profile load";
                std::cerr << to_json(err).dump() << std::endl; return 2;
            }
            // supplied-but-absent is a typed error, same as supplied-but-invalid
            LayoutErrorV1 err; err.error.code="INVALID_INPUT"; err.error.message="failed to open profile: "+rel;
            std::cerr << to_json(err).dump() << std::endl; return 1;
        }
        if (rr == 1) {
            LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during profile load";
            std::cerr << to_json(err).dump() << std::endl; return 2;
        }
        std::unordered_set<std::string> vis;
        int rc = load_profile_json_impl(content, fp, cfg, vis, 0);
        if (rc == 3) {
            LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during profile load";
            std::cerr << to_json(err).dump() << std::endl; return 2;
        }
        if (rc != 0) {
            LayoutErrorV1 err; err.error.code="INVALID_INPUT";
            err.error.message = (rc == 1) ? "inheritance chain for profile exceeds depth limit: "+rel
                                          : "failed to load profile: "+rel;
            std::cerr << to_json(err).dump() << std::endl; return 1;
        }
        return 0;
    };
    { int r = load_opt(problem.profiles.process); if (r == 2) return 5; if (r == 1) return 3; }
    { int r = load_opt(problem.profiles.filament); if (r == 2) return 5; if (r == 1) return 3; }

    // Load models; count polygons per ref for identity mapping
    Model unlocked_model, locked_model;
    struct RefCount { std::string id; size_t count; double rot = 0.0; };
    std::vector<RefCount> unlocked_counts, locked_counts;
    for (auto& ref : problem.models) {
        // honest limitation: g_cancelled is re-checked immediately before each
        // Model::read_from_file call. A read blocked INSIDE the engine loader
        // (e.g. a stalled network path) cannot be interrupted mid-call on any
        // platform — the engine's own file IO has no cancellation hook, and we
        // do not pretend otherwise. The flag is honoured at the next
        // checkpoint: before the next model, and at every post-load loop.
        if (g_cancelled.load()) {
            LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during model load";
            std::cerr << to_json(err).dump() << std::endl; return 5;
        }
        try {
            Model m = Model::read_from_file(ref.path);
            if (g_cancelled.load()) {  // SIGINT may land during the load; recheck right after it returns
                LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during model load";
                std::cerr << to_json(err).dump() << std::endl; return 5;
            }
            size_t n = 0;
            for (ModelObject* mo : m.objects) {
                if (g_cancelled.load()) {  // clone loop may be long for many-object models
                    LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during model load";
                    std::cerr << to_json(err).dump() << std::endl; return 5;
                }
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
            // the read may have failed because SIGINT interrupted it
            if (g_cancelled.load()) {
                LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during model load";
                std::cerr << to_json(err).dump() << std::endl; return 5;
            }
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
    if (g_cancelled.load()) {  // extraction may be long for many-object models
        LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during validation";
        std::cerr << to_json(err).dump() << std::endl; return 5;
    }
    auto locked_input   = get_arrange_polys(locked_model, li);
    if (g_cancelled.load()) {  // recheck after the second extraction too
        LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during validation";
        std::cerr << to_json(err).dump() << std::endl; return 5;
    }

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
    if (g_cancelled.load()) {  // params/inflation/shrink block may be long — recheck before bed validation
        LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during validation";
        std::cerr << to_json(err).dump() << std::endl; return 5;
    }
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
                Slic3r::ExPolygons off = offset_ex(inflated, float(ap.inflation));  // holes survive inflation
                if (!off.empty())
                    inflated = off.front();
                if (!fully_inside_bed(inflated, bed_poly)) {
                    pm.bed_idx=-1; unfittable.push_back(ref.id); result.placements.push_back(pm); continue;
                }
            }
            locked_placed.push_back({ref.id, ap.transformed_poly(), ap.inflation});
            pm.bed_idx = 0;
            // x_mm/y_mm = bounding-box centre (candidate convention, same as unlocked);
            // rotation_rad = effective rotation (request override or preserved embedded)
            pm.x_mm = cx; pm.y_mm = cy;
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
        std::vector<ExPolygon> inflated;
        for (auto& rec : locked_placed) {
            if (g_cancelled.load()) {
                LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during validation";
                std::cerr << to_json(err).dump() << std::endl; return 5;
            }
            // actual inflation = max of spacing-derived and profile-derived (brim/clearance)
            double eff = std::max(spacing_infl, double(rec.inflation));
            Slic3r::ExPolygons off = offset_ex(rec.poly, float(eff));  // hole-aware inflate
            inflated.emplace_back(off.empty() ? rec.poly : off.front());
            if (g_cancelled.load()) {  // after the last offset call (single-locked-model path)
                LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during validation";
                std::cerr << to_json(err).dump() << std::endl; return 5;
            }
        }
        // O(1) dedup of failure ids — the pair loop is O(n^2) in pairs already;
        // keep a set so repeated violations of one id don't add linear scans
        std::unordered_set<std::string> fail_ids;
        for (size_t a = 0; a < inflated.size(); ++a)
            for (size_t b = a+1; b < inflated.size(); ++b) {
                if (g_cancelled.load()) {
                    LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during validation";
                    std::cerr << to_json(err).dump() << std::endl; return 5;
                }
                Slic3r::ExPolygons ia = intersection_ex(inflated[a], inflated[b]);  // holes preserved
                if (g_cancelled.load()) {  // intersection may straddle SIGINT
                    LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during validation";
                    std::cerr << to_json(err).dump() << std::endl; return 5;
                }
                if (!ia.empty()) {
                    if (fail_ids.insert(locked_placed[a].id).second)
                        unfittable.push_back(locked_placed[a].id);
                    if (fail_ids.insert(locked_placed[b].id).second)
                        unfittable.push_back(locked_placed[b].id);
                }
            }
    }

    // F2: entries for overlapping/too-close locked models must be unplaced
    // (bed_idx=-1, no coordinates) in the emitted candidate
    for (auto& pm : result.placements) {
        if (g_cancelled.load()) {
            LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during validation";
            std::cerr << to_json(err).dump() << std::endl; return 5;
        }
        if (std::find(unfittable.begin(), unfittable.end(), pm.id) != unfittable.end())
            pm.bed_idx = -1;
        if (g_cancelled.load()) {  // checkpoint after the lookup
            LayoutErrorV1 err; err.error.code="CANCELLED"; err.error.message="cancelled during validation";
            std::cerr << to_json(err).dump() << std::endl; return 5;
        }
    }

    // write the serialized candidate to stdout in chunks, checking
    // cancellation between chunks. On a backpressured stdout pipe the write
    // blocks in the kernel; SIGINT interrupts it (EINTR, no SA_RESTART) and
    // the next chunk-boundary check exits 5 CANCELLED promptly. A partially
    // written candidate on a killed pipe is unavoidable (the consumer already
    // saw bytes), but our side stops promptly and emits no further data.
    auto emit_cancellable = [&](const std::string& out) -> int {
        const size_t kChunk = 1 << 16;  // 64 KiB per write
        size_t i = 0;
        // Honest limitation (all platforms): a blocked stdout write is
        // observed only at the next chunk boundary. On POSIX, SIGINT
        // interrupts the write (EINTR, no SA_RESTART) and the chunk check
        // exits promptly; on Windows, ordinary inherited/anonymous pipes are
        // NOT overlapped-capable (they lack FILE_FLAG_OVERLAPPED), so async
        // I/O would fail for the common case — a SIGINT during a blocked
        // write is not observed until the write returns, same as synchronous
        // _open on stalled UNC paths. Untestable async I/O (no Windows host
        // available) would be riskier than the limitation. The chunk
        // boundary bounds each blocking write; the flag is honoured between
        // chunks and after the final flush.
        while (i < out.size()) {
            if (g_cancelled.load()) {
                LayoutErrorV1 cerr2; cerr2.error.code="CANCELLED"; cerr2.error.message="cancelled during validation";
                std::cerr << to_json(cerr2).dump() << std::endl; return 5;
            }
            size_t n = std::min(kChunk, out.size() - i);
            std::cout.write(out.data() + static_cast<std::streamsize>(i), static_cast<std::streamsize>(n));
            std::cout.flush();  // bound each blocking write to one chunk
            if (!std::cout.good()) {  // hard write failure (e.g. closed stdout pipe)
                if (g_cancelled.load()) {  // cancel takes precedence over the write failure
                    LayoutErrorV1 cerr2; cerr2.error.code="CANCELLED"; cerr2.error.message="cancelled during validation";
                    std::cerr << to_json(cerr2).dump() << std::endl; return 5;
                }
                LayoutErrorV1 cerr2; cerr2.error.code="WRITE_FAILED"; cerr2.error.message="failed to write candidate to stdout";
                std::cerr << to_json(cerr2).dump() << std::endl; return 6;
            }
            i += n;
        }
        std::cout << '\n';
        std::cout.flush();
        if (!std::cout.good()) {  // final flush failed (e.g. consumer closed the pipe)
            if (g_cancelled.load()) {  // cancel takes precedence over the write failure
                LayoutErrorV1 cerr2; cerr2.error.code="CANCELLED"; cerr2.error.message="cancelled during validation";
                std::cerr << to_json(cerr2).dump() << std::endl; return 5;
            }
            LayoutErrorV1 cerr2; cerr2.error.code="WRITE_FAILED"; cerr2.error.message="failed to write candidate to stdout";
            std::cerr << to_json(cerr2).dump() << std::endl; return 6;
        }
        if (g_cancelled.load()) {  // final recheck after the last flush (per convention)
            LayoutErrorV1 cerr2; cerr2.error.code="CANCELLED"; cerr2.error.message="cancelled during validation";
            std::cerr << to_json(cerr2).dump() << std::endl; return 5;
        }
        return 0;
    };

    if (!unfittable.empty()) {
        std::string out = to_json(result).dump();
        if (g_cancelled.load()) {  // cancel wins over emitting a large result
            LayoutErrorV1 cerr2; cerr2.error.code="CANCELLED"; cerr2.error.message="cancelled during validation";
            std::cerr << to_json(cerr2).dump() << std::endl; return 5;
        }
        int w = emit_cancellable(out);  // emit first — UNFITTABLE goes out only after the candidate emits fully
        if (w == 5) return 5;           // cancel during emission: CANCELLED already printed, suppress UNFITTABLE
        if (w == 6) return 6;           // hard write failure: WRITE_FAILED already printed
        LayoutErrorV1 err; err.error.code="UNFITTABLE";
        err.error.message="some objects could not be placed on any bed";
        err.error.object_ids=unfittable;
        std::cerr << to_json(err).dump() << std::endl;
        return 4;
    }
    std::string out = to_json(result).dump();
    if (g_cancelled.load()) {  // cancel wins over emitting a large result
        LayoutErrorV1 cerr2; cerr2.error.code="CANCELLED"; cerr2.error.message="cancelled during validation";
        std::cerr << to_json(cerr2).dump() << std::endl; return 5;
    }
    int w = emit_cancellable(out);
    return w == 5 ? 5 : (w == 6 ? 6 : 0);
}

}  // namespace layout_plan
