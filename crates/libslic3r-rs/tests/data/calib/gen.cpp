// Generator of the vectors of tests/calib_native_parity.rs: libslic3r's Calib.hpp / Calib.cpp at
// the pin (references/BambuStudio/src/libslic3r; the override layer has no copy of them): the
// configuration and geometry members of CalibPressureAdvancePattern, the arithmetic members of
// CalibPressureAdvance, find_optimal_PA_speed and SuggestedConfigCalibPAPattern.
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
//   ./gen > $DATA/calib.txt
//
// Made on macOS arm64 with Apple clang (-ffp-contract=on). The protected and private members are
// reached by including the libslic3r headers with `private` and `protected` defined as `public` (the inline
// members are compiled here from the same header with the same flags; the others run from
// libslic3r_core).
//
// Every case is a configuration (option values set from text, as the Rust test sets them), a
// model of one box with one instance, and Calib_Params; the case line `case <n>` is followed by
// one line per member: its name, then the result (doubles as 16 hexadecimal digits, ints and
// floats' bits as decimal / 8 hexadecimal digits, texts as the hexadecimal of their bytes).

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

// Model.hpp includes Calib.hpp: the access keywords are redefined before the libslic3r headers.
#define private public
#define protected public
#include "libslic3r/Calib.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Flow.hpp"
#undef private
#undef protected

using namespace Slic3r;

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
static std::string hT(const std::string &s)
{
    if (s.empty()) return "-";
    std::string o;
    char b[4];
    for (unsigned char c : s) {
        std::snprintf(b, sizeof(b), "%02x", c);
        o += b;
    }
    return o;
}
static void out(const std::string &name, const std::string &v) { std::printf("%s %s\n", name.c_str(), v.c_str()); }

struct Case
{
    std::vector<std::pair<std::string, std::string>> opts;
    std::vector<std::pair<std::string, std::string>> object_opts;
    double start, end, step;
    double box[3];
    double offset[3];
};

int main()
{
    std::vector<Case> cases = {
        {{{"nozzle_diameter", "0.4"}, {"line_width", "0.45"}, {"initial_layer_line_width", "0.5"}, {"wall_loops", "3"},
          {"initial_layer_print_height", "0.25"}, {"layer_height", "0.2"}, {"initial_layer_speed", "30"}, {"outer_wall_speed", "200"},
          {"filament_max_volumetric_speed", "21"}},
         {},
         0.0, 0.1, 0.002, {20, 30, 1}, {100, 120, 0}},
        {{{"nozzle_diameter", "0.6"}, {"line_width", "0.62"}, {"initial_layer_line_width", "0.75"}, {"wall_loops", "2"},
          {"initial_layer_print_height", "0.3"}, {"layer_height", "0.3"}, {"initial_layer_speed", "25"}, {"outer_wall_speed", "60"},
          {"filament_max_volumetric_speed", "12"}},
         {{"wall_loops", "4"}},
         0.01, 0.05, 0.005, {35.5, 12.25, 2}, {-10.5, 33.3, 0}},
        {{{"nozzle_diameter", "0.2"}, {"line_width", "0.22"}, {"initial_layer_line_width", "0.28"}, {"wall_loops", "3"},
          {"initial_layer_print_height", "0.1"}, {"layer_height", "0.08"}, {"initial_layer_speed", "20"}, {"outer_wall_speed", "150"},
          {"filament_max_volumetric_speed", "3.5"}},
         {},
         0.0, 0.04, 0.004, {10, 10, 0.5}, {50, 50, 0}},
        {{{"nozzle_diameter", "0.4"}, {"line_width", "0.42"}, {"initial_layer_line_width", "0.5"}, {"wall_loops", "1"},
          {"initial_layer_print_height", "0.2"}, {"layer_height", "0.16"}, {"initial_layer_speed", "50"}, {"outer_wall_speed", "300"},
          {"filament_max_volumetric_speed", "40"}},
         {{"layer_height", "0.12"}},
         0.02, 0.08, 0.0025, {60, 40, 3}, {0, 0, 0}},
        {{{"nozzle_diameter", "0.8"}, {"line_width", "0.9"}, {"initial_layer_line_width", "1"}, {"wall_loops", "5"},
          {"initial_layer_print_height", "0.4"}, {"layer_height", "0.4"}, {"initial_layer_speed", "35"}, {"outer_wall_speed", "80"},
          {"filament_max_volumetric_speed", "30"}},
         {},
         0.1, 1.0, 0.1, {80, 25, 1.5}, {12.75, -8.5, 0}},
        {{{"nozzle_diameter", "0.4"}, {"line_width", "0.45"}, {"initial_layer_line_width", "0.5"}, {"wall_loops", "3"},
          {"initial_layer_print_height", "0.25"}, {"layer_height", "0.2"}, {"initial_layer_speed", "30"}, {"outer_wall_speed", "200"},
          {"filament_max_volumetric_speed", "21"}},
         {},
         0.0, 0.0333, 0.001111, {20, 30, 1}, {100, 120, 0}},
    };

    for (size_t n = 0; n < cases.size(); ++n) {
        const Case &c = cases[n];
        std::printf("case %zu\n", n);
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        for (auto &kv : c.opts) config.set_deserialize_strict(kv.first, kv.second);
        Model model;
        ModelObject *obj = model.add_object();
        obj->add_volume(TriangleMesh(its_make_cube(c.box[0], c.box[1], c.box[2])));
        for (auto &kv : c.object_opts) {
            ConfigSubstitutionContext ctxt(ForwardCompatibilitySubstitutionRule::Disable);
            obj->config.set_deserialize(kv.first, kv.second, ctxt);
        }
        obj->add_instance()->set_offset(Vec3d(c.offset[0], c.offset[1], c.offset[2]));
        Calib_Params params;
        params.start = c.start;
        params.end   = c.end;
        params.step  = c.step;
        params.mode  = CalibMode::Calib_PA_Pattern;
        CalibPressureAdvancePattern p(params, config, false, model, Vec3d(5, 7, 0));

        out("handle_xy_size", hD(p.handle_xy_size()));
        out("handle_spacing", hD(p.handle_spacing()));
        Vec3d so = p.get_start_offset();
        out("get_start_offset", hD(so.x()) + " " + hD(so.y()) + " " + hD(so.z()));
        out("speed_first_layer", hD(p.speed_first_layer()));
        out("speed_perimeter", hD(p.speed_perimeter()));
        out("line_width_first_layer", hD(p.line_width_first_layer()));
        out("line_width", hD(p.line_width()));
        out("wall_count", std::to_string(p.wall_count()));
        out("height_first_layer", hD(p.height_first_layer()));
        out("height_layer", hD(p.height_layer()));
        out("get_num_patterns", std::to_string(p.get_num_patterns()));
        out("line_spacing", hD(p.line_spacing()));
        out("line_spacing_first_layer", hD(p.line_spacing_first_layer()));
        out("line_spacing_angle", hD(p.line_spacing_angle()));
        out("object_size_x", hD(p.object_size_x()));
        out("object_size_y", hD(p.object_size_y()));
        out("frame_size_y", hD(p.frame_size_y()));
        std::string g;
        for (int i = 0; i < 6; ++i) g += (i ? " " : "") + hD(p.glyph_start_x(i));
        out("glyph_start_x", g);
        out("glyph_length_x", hD(p.glyph_length_x()));
        out("glyph_tab_max_x", hD(p.glyph_tab_max_x()));
        out("max_numbering_height", hD(p.max_numbering_height()));
        out("pattern_shift", hD(p.pattern_shift()));
        out("print_size_x", hD(p.print_size_x()));
        out("print_size_y", hD(p.print_size_y()));
        out("max_layer_z", hD(p.max_layer_z()));
        p.set_start_offset(Vec3d(-3.5, 4.25, 0.5));
        so = p.get_start_offset();
        out("set_start_offset", hD(so.x()) + " " + hD(so.y()) + " " + hD(so.z()) + " " + hD(p.glyph_tab_max_x()));

        out("find_optimal_PA_speed", hF(CalibPressureAdvance::find_optimal_PA_speed(config, p.line_width(), p.height_layer(), 0, 0)));
        out("e_per_mm", hD(p.e_per_mm(p.line_width(), p.height_layer(), float(config.option<ConfigOptionFloatsNullable>("nozzle_diameter")->get_at(0)),
                                      float(config.option<ConfigOptionFloats>("filament_diameter")->get_at(0)),
                                      float(config.option<ConfigOptionFloatsNullable>("filament_flow_ratio")->get_at(0)))));
        out("get_distance", hD(p.get_distance(Vec2d(c.offset[0], c.offset[1]), Vec2d(c.box[0], c.box[1]))));
        out("to_radians", hD(p.to_radians(c.box[0])));
        out("number_spacing", hD(p.number_spacing()));
    }

    std::printf("case strings\n");
    CalibPressureAdvancePattern *none = nullptr;
    (void) none;
    const double nums[] = {0, 0.002, 0.1, 0.035, 1.0 / 3.0, 12.5, 100, -0.004, 0.0333, 1e-7, 2.0000005, 123456.789, -0.0};
    for (double v : nums) {
        // convert_number_to_string reads no member: called through a null object of the base class
        std::string s = static_cast<CalibPressureAdvance *>(nullptr)->convert_number_to_string(v);
        out("convert_number_to_string", hD(v) + " " + hT(s));
    }
    SuggestedConfigCalibPAPattern sc;
    for (double d : {0.2, 0.4, 0.2000001, 0.1999995, 0.6}) {
        auto fp = sc.float_pairs(d);
        out("float_pairs", hD(d) + " " + hD(sc.initial_layer_print_height(d)) + " " + hT(fp[0].first) + " " + hD(fp[0].second) + " " +
                               hT(fp[1].first) + " " + hD(fp[1].second));
    }
    return 0;
}
