// layout_plan.hpp — versioned headless layout-plan contract for issue #7
//
// LayoutProblemV1  (stdin / --input)  → slicer_cli --layout-plan
// PlacementCandidateV1 (stdout)       ← arrange result
// LayoutErrorV1     (stderr)          ← on failure
//
// Helio owns the schema; all keys versioned.  Never add fields
// without bumping the schema version.

#ifndef LAYOUT_PLAN_HPP
#define LAYOUT_PLAN_HPP

#include <string>
#include <vector>
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
    static constexpr int SCHEMA_VERSION = 1;

    std::string            engine;       // "orca" or "bambu"
    std::string            profiles_dir; // root of BBL/ tree
    ProfileRef             profiles;
    SpacingPolicy          spacing;
    std::vector<ModelRef>  models;
};

// ─── Output ──────────────────────────────────────────────────────────────────

struct PlacedModel {
    std::string id;
    int         bed_idx      = -1;
    double      x_mm         = 0.0;
    double      y_mm         = 0.0;
    double      rotation_rad = 0.0;
    double      bb_cx_mm     = 0.0;  // bounding-box centre X (post-arrange)
    double      bb_cy_mm     = 0.0;  // bounding-box centre Y
};

struct PlacementCandidateV1 {
    static constexpr int SCHEMA_VERSION = 1;

    std::string              engine;
    std::vector<PlacedModel> placements;
};

struct LayoutErrorV1 {
    static constexpr int SCHEMA_VERSION = 1;

    struct Detail {
        std::string              code;        // e.g. UNFITTABLE, INVALID_INPUT, ENGINE_MISMATCH
        std::string              message;
        std::vector<std::string> object_ids;  // which objects (empty = global)
    };

    Detail error;
};

// ─── Parsers ─────────────────────────────────────────────────────────────────

// Returns false on parse failure; populates `out` on success and writes
// the typed error to `err`.
bool parse_input(const nlohmann::json& raw, LayoutProblemV1& out, LayoutErrorV1& err);

// ─── Runner ──────────────────────────────────────────────────────────────────

// Runs arrange given a parsed problem.  Returns 0 on success (writes
// PlacementCandidateV1 to stdout), non-zero on failure (writes
// LayoutErrorV1 to stderr).
int run_layout_plan(const LayoutProblemV1& problem);

}  // namespace layout_plan

#endif
