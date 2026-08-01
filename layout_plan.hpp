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
#include <cstdint>
#include <nlohmann/json.hpp>

namespace layout_plan {

struct ModelRef {
    std::string id;
    std::string path;
    double     x_mm   = 0.0;
    double     y_mm   = 0.0;
    double     z_mm   = 0.0;
    double     rot_z_rad = 0.0;
    bool       locked   = false;
    bool       has_x = false, has_y = false, has_z = false, has_rot = false;
    bool       has_override = false; // any transform component supplied
};

struct SpacingPolicy {
    double min_object_distance_mm = 10.0;
    double clearance_radius_mm    = 0.0;
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
    std::string            engine;
    std::string            profiles_dir;
    ProfileRef             profiles;
    SpacingPolicy          spacing;
    std::vector<ModelRef>  models;
    uint64_t               seed = 0;   // accepted-and-recorded; engine is inherently deterministic
};

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

struct CapabilitiesV1 {
    static constexpr int SCHEMA_VERSION = 1;
    std::string engine;
    std::string engine_commit;
    std::string engine_version;
    int         min_schema_version = LayoutProblemV1::VERSION_MIN;
    int         max_schema_version = LayoutProblemV1::VERSION_MAX;
    bool within_plate          = true;
    bool cross_plate           = false;
    bool non_rectangular_beds  = true;
    bool locks                 = true;
    bool rotation_constraints  = false; // v2: engine computes rotations internally, caller sets unsupported
    bool seeded_determinism    = true;  // inherent: arrange pipeline uses no RNG
    bool cancellation          = true;
    bool progress              = false;
};

bool parse_input(const nlohmann::json& raw, LayoutProblemV1& out, LayoutErrorV1& err);
int  run_layout_plan(const LayoutProblemV1& problem);
int  run_capabilities();

}  // namespace layout_plan

#endif
