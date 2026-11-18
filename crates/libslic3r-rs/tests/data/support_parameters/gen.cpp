// Generator of the vectors of tests/support_parameters_native_parity.rs: libslic3r's
// Support/SupportParameters.hpp at the pin (references/BambuStudio/src/libslic3r; the override layer has
// no copy of it).
//
// Build, with REPO = this repository, BUILD = a built tree of the Bambu Studio engine
// (docs/building.md, -DENGINE=bambu) and DATA = this directory: compile with the compile line the
// build uses for a libslic3r source (its defines and include directories, the override layer
// $REPO/libslic3r/bambustudio before the pin, -std=c++17 -O3 -DNDEBUG and the forced include of
// cgal_54_compat.hpp), then link with the build's libslic3r_core and the libraries it links:
//
//   c++ <the build's defines and include directories> -std=c++17 -O3 -DNDEBUG \
//       -include $REPO/libslic3r/bambustudio/cgal_54_compat.hpp -c $DATA/gen.cpp -o gen.o
//   c++ gen.o $BUILD/libslic3r_core.a <the libraries libslic3r_core links> -o gen
//   ./gen vectors.txt
//
// Every scenario is a Print of one box-shaped object made by `Print::apply` with a few keys of the
// configuration changed, whose object layers are filled in by hand (their heights only), and the
// SupportParameters of the object: every member, the flows for a nozzle, the interface angles. The free
// functions of the header run on their own arguments.
//
// Lines that start with "in" are the inputs the Rust test reads back; the others are results it has to
// reproduce. Tokens are separated by one space; a double is its 16 hexadecimal digits, a float its 8,
// an integer decimal. The vectors were made on macOS arm64 with Apple clang and Eigen 5.0.1.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// Before the access specifiers are redefined: tbb declares nested classes ahead of their definition.
#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_unordered_set.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/spin_mutex.h>

#define private public
#define protected public
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#undef protected
#undef private
#include "libslic3r/Model.hpp"
#include "libslic3r/Support/SupportParameters.hpp"
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;

static FILE *g_out = nullptr;

static std::string hD(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, 8);
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long) u);
    return b;
}
static std::string hF(float v)
{
    uint32_t u;
    std::memcpy(&u, &v, 4);
    char b[16];
    std::snprintf(b, sizeof(b), "%08x", u);
    return b;
}
static std::string hI(long long v) { return std::to_string(v); }
static std::string hB(bool v) { return v ? "1" : "0"; }
static void out(const std::string &s) { std::fprintf(g_out, "%s\n", s.c_str()); }
static std::string FLOW(const Flow &f)
{
    return hF(f.width()) + " " + hF(f.height()) + " " + hF(f.spacing()) + " " + hF(f.nozzle_diameter()) + " " + hB(f.bridge());
}

typedef std::vector<std::pair<std::string, std::string>> Cfg;

static std::vector<Cfg> scenarios()
{
    std::vector<Cfg> s;
    // 0: the defaults with normal supports.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(auto)"}});
    // 1: snug, interface layers, a raft with every kind of layer.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(manual)"}, {"support_style", "snug"}, {"support_interface_top_layers", "3"},
                 {"support_interface_bottom_layers", "2"}, {"raft_layers", "5"}, {"support_angle", "30"}, {"support_interface_spacing", "0.3"},
                 {"support_base_pattern_spacing", "1.7"}, {"support_line_width", "0.38"}});
    // 2: no interface layers, a raft of two layers, a grid interface, a honeycomb base.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_interface_top_layers", "0"}, {"raft_layers", "2"},
                 {"support_interface_pattern", "grid"}, {"support_base_pattern", "honeycomb"}, {"support_angle", "45"},
                 {"top_z_overrides_xy_distance", "1"}, {"support_top_z_distance", "0.12"}, {"support_object_xy_distance", "0.5"}});
    // 3: a raft of one layer, a concentric interface, a dense base, thick bridges, separate support filaments.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"raft_layers", "1"}, {"support_interface_pattern", "concentric"},
                 {"support_base_pattern_spacing", "0"}, {"thick_bridges", "1"}, {"support_filament", "2"}, {"support_interface_filament", "3"},
                 {"bridge_flow", "0.9"}, {"support_interface_bottom_layers", "-1"}, {"support_interface_top_layers", "4"}});
    // 4: tree supports with the default style, the double wall threshold of the filament strength.
    s.push_back({{"enable_support", "1"}, {"support_type", "tree(auto)"}, {"tree_support_wall_count", "-1"}, {"impact_strength_z", "13.5,7.25,21"},
                 {"support_interface_filament", "2"}, {"layer_height", "0.16"}, {"support_ironing_pattern", "concentric"}, {"enable_support_ironing", "1"},
                 {"support_ironing_flow", "12.5%"}, {"support_ironing_spacing", "0.15"}, {"support_ironing_inset", "0.05"}, {"support_ironing_speed", "33"},
                 {"support_ironing_direction", "17"}});
    // 5: tree supports asked for with a style of the normal supports, a lightning base, two walls, a soluble interface.
    s.push_back({{"enable_support", "1"}, {"support_type", "tree(manual)"}, {"support_style", "grid"}, {"support_base_pattern", "lightning"},
                 {"tree_support_wall_count", "2"}, {"support_top_z_distance", "0"}, {"support_interface_filament", "3"}, {"filament_soluble", "0,0,1"},
                 {"support_filament", "1"}, {"support_interface_top_layers", "5"}, {"support_interface_bottom_layers", "5"},
                 {"independent_support_layer_height", "0"}});
    // 6: normal supports asked for with a style of the tree supports, a soluble interface and base, a wide first layer.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_style", "tree_hybrid"}, {"support_base_pattern", "lightning"},
                 {"support_top_z_distance", "0"}, {"support_interface_filament", "3"}, {"support_filament", "3"}, {"filament_soluble", "0,0,1"},
                 {"initial_layer_line_width", "0.62"}, {"line_width", "0.44"}, {"support_line_width", "0"}, {"support_interface_top_layers", "2"},
                 {"support_interface_bottom_layers", "1"}, {"support_object_first_layer_gap", "0.35"}, {"raft_layers", "3"},
                 {"min_layer_height", "0.04,0.08,0.06"}});
    // 7: a soluble interface over a base that is not, normal supports; an interlaced interface.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_top_z_distance", "0"}, {"support_interface_filament", "3"},
                 {"support_filament", "0"}, {"filament_soluble", "0,0,1"}, {"support_interface_top_layers", "3"}, {"support_interface_bottom_layers", "5"},
                 {"support_interface_pattern", "rectilinear_interlaced"}, {"support_interface_spacing", "0"}, {"raft_layers", "4"}});
    // 8: tree supports, strong; the supports of filament 2 and the interface with the active one.
    s.push_back({{"enable_support", "1"}, {"support_type", "tree(auto)"}, {"support_style", "tree_strong"}, {"support_filament", "2"},
                 {"support_interface_filament", "0"}, {"tree_support_wall_count", "1"}, {"support_interface_pattern", "rectilinear"},
                 {"support_base_pattern", "rectilinear-grid"}});
    return s;
}

static void run_scenario(int scenario, const Cfg &cfg)
{
    const std::string S = "s" + hI(scenario);
    Model        model;
    ModelObject *o = model.add_object();
    o->name = "box";
    indexed_triangle_set its = its_make_cube(20, 20, 10);
    for (Vec3f &v : its.vertices) v += Vec3f(-10.f, -10.f, 0.f);
    o->add_volume(TriangleMesh(its));
    o->add_instance()->set_offset(Vec3d(100., 100., 0.));
    out("in " + S + " model 20 20 10 100 100");

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(3);
    config.option<ConfigOptionStrings>("filament_colour", true)->values = {"#FF0000", "#00FF00", "#0000FF"};
    config.option<ConfigOptionInts>("filament_map", true)->values = {1, 1, 1};
    for (const auto &kv : cfg) {
        config.set_deserialize_strict(kv.first, kv.second);
        out("in " + S + " cfg " + kv.first + " " + kv.second);
    }
    Print print;
    print.apply(model, config);
    PrintObject *obj = print.m_objects.front();
    obj->update_slicing_parameters();

    // The object layers: their heights only (the constructor reads nothing else of them).
    const int    n_layers = 6;
    const double heights[n_layers] = {0.2, 0.2, 0.12, 0.2, 0.07 + scenario * 0.011, 0.2};
    double       z = 0.;
    for (int i = 0; i < n_layers; ++i) {
        z += heights[i];
        Layer *layer = new Layer(i, obj, heights[i], z, z - heights[i] / 2.);
        obj->m_layers.emplace_back(layer);
        out("in " + S + " layer " + hD(z) + " " + hD(heights[i]) + " " + hD(layer->slice_z));
    }
    obj->has_variable_layer_heights = (scenario == 4);
    out("in " + S + " variable " + hB(obj->has_variable_layer_heights));

    const SlicingParameters &sp = obj->slicing_parameters();
    out(S + " slicing " + hD(sp.layer_height) + " " + hD(sp.first_print_layer_height) + " " + hI((long long) sp.base_raft_layers) + " " +
        hI((long long) sp.interface_raft_layers) + " " + hB(sp.soluble_interface));

    const SupportParameters p(*obj);
    out(S + " flags " + hB(p.soluble_interface) + " " + hB(p.soluble_interface_non_soluble_base) + " " + hB(p.has_top_contacts) + " " + hB(p.has_bottom_contacts) + " " +
        hI((long long) p.num_top_interface_layers) + " " + hI((long long) p.num_bottom_interface_layers) + " " + hI((long long) p.num_top_base_interface_layers) + " " +
        hI((long long) p.num_bottom_base_interface_layers) + " " + hB(p.has_contacts()) + " " + hB(p.has_interfaces()) + " " + hB(p.has_base_interfaces()) + " " +
        hI((long long) p.num_top_interface_layers_only()) + " " + hI((long long) p.num_bottom_interface_layers_only()));
    out(S + " flows " + FLOW(p.first_layer_flow) + " " + FLOW(p.support_material_flow) + " " + FLOW(p.support_material_interface_flow) + " " +
        FLOW(p.support_material_bottom_interface_flow) + " " + FLOW(p.raft_interface_flow));
    out(S + " widths " + hD(p.support_base_width.value) + " " + hD(p.support_first_layer_width.value) + " " + hF(p.preset_nozzle_diameter) + " " +
        hB(p.multi_nozzle_diameter_mode) + " " + hD(p.support_extrusion_width) + " " + hB(p.can_merge_support_regions) + " " + hD(p.support_layer_height_min) + " " +
        hD(p.gap_xy) + " " + hD(p.gap_xy_first_layer));
    out(S + " angles " + hF(p.base_angle) + " " + hF(p.interface_angle) + " " + hD(p.interface_spacing) + " " + hD(p.support_expansion) + " " + hD(p.interface_density) + " " +
        hD(p.raft_interface_density) + " " + hD(p.support_spacing) + " " + hD(p.support_density) + " " + hI((int) p.support_style) + " " + hI((int) p.support_base_pattern));
    out(S + " patterns " + hI((int) p.base_fill_pattern) + " " + hI((int) p.interface_fill_pattern) + " " + hI((int) p.raft_interface_fill_pattern) + " " +
        hI((int) p.contact_fill_pattern) + " " + hB(p.with_sheath) + " " + hD(p.tree_branch_diameter_double_wall_area_scaled) + " " + hF(p.raft_angle_1st_layer) + " " +
        hF(p.raft_angle_base) + " " + hF(p.raft_angle_interface) + " " + hB(p.independent_layer_height) + " " + hD(p.thresh_big_overhang));
    out(S + " ironing " + hB(p.enable_support_ironing) + " " + hI((int) p.ironing_pattern) + " " + hD(p.ironing_line_spacing) + " " + hD(p.ironing_flow_percent) + " " +
        hD(p.ironing_speed) + " " + hD(p.ironing_angle) + " " + hD(p.ironing_inset));
    for (size_t interface_id : {size_t(0), size_t(1), size_t(2), size_t(7)}) {
        std::string line = S + " interface_angle " + hI((long long) interface_id) + " " + hF(p.raft_interface_angle(interface_id));
        for (int pattern = 0; pattern < 5; ++pattern) line += " " + hF(p.support_interface_angle(interface_id, SupportMaterialInterfacePattern(pattern)));
        out(line);
    }
    for (double nozzle : {0.2, 0.4, 0.6, 0.8})
        for (float lh : {0.08f, 0.2f, 0.28f})
            out(S + " flow_for_nozzle " + hD(nozzle) + " " + hF(lh) + " " + FLOW(p.support_material_flow_for_nozzle(nozzle, lh)) + " " +
                FLOW(p.support_material_interface_flow_for_nozzle(nozzle, lh)) + " " + FLOW(p.support_material_first_layer_flow_for_nozzle(nozzle, lh)));
    out(S + " xy_distance " + hD(resolved_support_object_xy_distance(print.config(), obj->config())));
    out(S + " flag " + hB(config_flag_or_false(print.full_print_config(), "enable_support")) + " " + hB(config_flag_or_false(print.full_print_config(), "thick_bridges")) + " " +
        hB(config_flag_or_false(print.full_print_config(), "no_such_key")) + " " + hB(config_flag_or_false(print.full_print_config(), "layer_height")));
}

int main(int argc, char **argv)
{
    g_out = std::fopen(argc > 1 ? argv[1] : "vectors.txt", "w");
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism, 1);
    // resolve_support_style over every combination of its arguments.
    for (int style = 0; style <= 6; ++style)
        for (int type = 0; type <= 3; ++type) {
            std::string line = "resolve_support_style " + hI(style) + " " + hI(type);
            for (int bits = 0; bits < 32; ++bits)
                line += " " + hI((int) resolve_support_style(SupportMaterialStyle(style), SupportType(type), bits & 1, bits & 2, bits & 4, bits & 8, bits & 16));
            out(line);
        }
    const std::vector<Cfg> s = scenarios();
    for (size_t i = 0; i < s.size(); ++i) run_scenario(int(i), s[i]);
    std::fclose(g_out);
    return 0;
}
