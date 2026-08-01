// layout_plan.hpp — versioned headless layout-plan contract for issue #7
//
// LayoutProblemV1   (stdin / --input)   → slicer_cli --layout-plan
// CapabilitiesV1    (--json)            ← slicer_cli layout capabilities
// PlacementCandidateV1 (stdout)         ← arrange result
// LayoutErrorV1     (stderr)            ← on failure
//
// Helio owns the schema; all keys versioned.

#ifndef LAYOUT_PLAN_HPP
#define LAYOUT_PLAN_HPP

#include <string>
#include <vector>
#include <set>
#include <nlohmann/json.hpp>

namespace layout_plan {

// ─── Input ───────────────────────────────────────────────────────────────────

struct ModelRef {
    std::string id;             // caller-chosen stable key
    std::string path;           // absolute or relative file path (STL/OBJ/3MF)
    double     x_mm   = 0.0;   // current instance offset (X), mm
    double     y_mm   = 0.0;   // current instance offset (Y), mm
    double     z_mm   = 0.0;   // current instance offset (Z), mm
    double     rot_z_rad = 0.0;// current instance Z-rotation, radians
    bool       locked   = false; // v1: locked instances never move (obstacles)
    std::vector<double> allowed_rotations; // empty = free rotation; non-empty = yaw list in radians
};

struct SpacingPolicy {
    double min_object_distance_mm = 10.0;
    double clearance_radius_mm    = 0.0;   // 0 → use profile value
    bool   allow_rotations        = true;
};

struct ProfileRef {
    std::string machine;
    std::string process;
    std::string filament;
};

struct LayoutProblemV1 {
    static constexpr int  SCHEMA_VERSION = 1;
    static constexpr int  VERSION_MIN    = 1;
    static constexpr int  VERSION_MAX    = 1;

    std::string            engine;       // "orca" or "bambu"
    std::string            profiles_dir; // root of BBL/ tree
    ProfileRef             profiles;
    SpacingPolicy          spacing;
    std::vector<ModelRef>  models;
    uint64_t               seed = 0;     // 0 = nondeterministic; non-zero = seeded
};

// ─── Output ──────────────────────────────────────────────────────────────────

struct PlacedModel {
    std::string id;
    int         bed_idx      = -1;
    double      x_mm         = 0.0;
    double      y_mm         = 0.0;
    double      rotation_rad = 0.0;
    double      bb_cx_mm     = 0.0;
    double      bb_cy_mm     = 0.0;
};

struct PlacementCandidateV1 {
    static constexpr int SCHEMA_VERSION = 1;

    std::string              engine;
    std::vector<PlacedModel> placements;
};

struct LayoutErrorV1 {
    static constexpr int SCHEMA_VERSION = 1;

    struct Detail {
        std::string              code;
        std::string              message;
        std::vector<std::string> object_ids;
    };

    Detail error;
};

// ─── Capabilities ────────────────────────────────────────────────────────────

struct CapabilitiesV1 {
    static constexpr int SCHEMA_VERSION = 1;

    // Engine identity
    std::string engine;              // "orca" | "bambu"
    std::string engine_commit;       // upstream git SHA
    std::string engine_version;      // human-readable version string

    // Contract version range this binary speaks
    int         min_schema_version = LayoutProblemV1::VERSION_MIN;
    int         max_schema_version = LayoutProblemV1::VERSION_MAX;

    // Honest capability booleans — false means "this build cannot do it"
    bool within_plate          = true;   // single-plate arrange
    bool cross_plate           = false;  // v1 non-goal
    bool non_rectangular_beds  = true;   // point-in-polygon gate
    bool locks                 = true;   // locked model instances
    bool rotation_constraints  = true;   // per-model allowed_rotations
    bool seeded_determinism    = true;   // seed → byte-identical output
    bool cancellation          = true;   // SIGINT → CANCELLED
    bool progress              = false;  // no progress streaming in v1
};

// ─── Parsers & Runners ───────────────────────────────────────────────────────

bool parse_input(const nlohmann::json& raw, LayoutProblemV1& out, LayoutErrorV1& err);
int  run_layout_plan(const LayoutProblemV1& problem);
int  run_capabilities();

}  // namespace layout_plan

#endif
