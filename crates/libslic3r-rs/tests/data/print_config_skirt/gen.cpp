// Generator of the vectors of tests/print_config_skirt_native_parity.rs: the skirt helpers and the bed
// shape with its excluded areas of PrintConfig.cpp at the pin (references/BambuStudio/src/libslic3r):
// has_skirt (PrintConfig.cpp:11558-11565), the three get_real_skirt_dist overloads (:11637-11674) and,
// through them, the static skirt_expansion_params (:11566-11588), skirt_nozzle_diameter_for_filament
// (:11590-11613) and skirt_flow_from_config (:11615-11635); get_bed_shape of a PrintConfig (:11508-11525) and
// get_bed_shape_with_excluded_area (:11529-11556); get_extruder_index (:110-117), nozzle_diameter_for_filament
// (:454-468), get_filament_config_idx (:485-492), get_process_config_idx (:494-502).
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
//   ./gen > vectors.txt
//
// Every scenario starts from the defaults of every option (DynamicPrintConfig::full_print_config()) and
// sets its keys by name. Line format: tokens separated by one space; a float is its 8 hexadecimal digits,
// an integer decimal. The vectors were made on macOS arm64 with Apple clang (-ffp-contract=on).

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/Polygon.hpp"

using namespace Slic3r;

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
static void out(const std::string &s) { std::printf("%s\n", s.c_str()); std::fflush(stdout); }
static std::string POLY(const Polygon &p)
{
    std::string o = hI((long long) p.points.size());
    for (const Point &pt : p.points) o += " " + hI(pt.x()) + " " + hI(pt.y());
    return o;
}

static void deser(DynamicPrintConfig &config, const char *key, const char *value)
{
    ConfigSubstitutionContext ctxt{ForwardCompatibilitySubstitutionRule::Disable};
    config.set_deserialize(key, value, ctxt);
}

static const int N_SCENARIOS = 10;

static DynamicPrintConfig scenario_config(int s)
{
    DynamicPrintConfig c = DynamicPrintConfig::full_print_config();
    switch (s) {
    case 0: break;
    case 1:
        deser(c, "skirt_per_object", "1"); deser(c, "skirt_loops", "2"); deser(c, "skirt_height", "1"); deser(c, "skirt_distance", "3.5");
        break;
    case 2:
        deser(c, "skirt_per_object", "1"); deser(c, "skirt_loops", "0"); deser(c, "draft_shield", "enabled"); deser(c, "skirt_distance", "2");
        break;
    case 3:
        deser(c, "skirt_per_object", "1"); deser(c, "skirt_loops", "3"); deser(c, "skirt_height", "2");
        deser(c, "initial_layer_line_width", "0"); deser(c, "line_width", "0.45"); deser(c, "initial_layer_print_height", "0.25");
        deser(c, "nozzle_diameter", "0.4,0.6"); deser(c, "filament_diameter_map", "\"\";\"0.8\";\"0.4;0.6\"");
        deser(c, "support_filament", "2");
        break;
    case 4:
        deser(c, "skirt_per_object", "1"); deser(c, "skirt_loops", "1"); deser(c, "skirt_height", "0");
        break;
    case 5:
        deser(c, "skirt_per_object", "1"); deser(c, "skirt_loops", "2"); deser(c, "skirt_height", "1");
        deser(c, "initial_layer_line_width", "0"); deser(c, "line_width", "0"); deser(c, "nozzle_diameter", "0.2,0.8");
        deser(c, "multi_nozzle_diameter_mode", "1");
        break;
    case 6:
        deser(c, "skirt_per_object", "1"); deser(c, "skirt_loops", "4"); deser(c, "skirt_height", "1"); deser(c, "skirt_distance", "0.7");
        deser(c, "initial_layer_line_width", "120%"); deser(c, "nozzle_diameter", "0.6"); deser(c, "filament_diameter_map", "\"0.6\";\"0\"");
        deser(c, "printable_area", "0x0,180x0,180x180,0x180");
        deser(c, "bed_exclude_area", "0x0,20x0,20x30,0x30,160x0,180x0,180x15,160x15");
        break;
    case 7:
        deser(c, "printable_area", "0x0,256x0,256x256,0x256");
        deser(c, "bed_exclude_area", "0x0,18x0,18x28,0x28");
        deser(c, "enable_wrapping_detection", "1");
        deser(c, "wrapping_exclude_area", "240.5x240.25,256x240.25,256x256,240.5x256");
        break;
    case 8:
        deser(c, "printable_area", "0x0,350x0,350x320,0x320");
        deser(c, "extruder_printable_area", "0x0,325x0,325x320,0x320#25x0,350x0,350x320,25x320");
        deser(c, "bed_exclude_area", "30x30,60x30,60x60,30x60");
        break;
    case 9:
        // extruder_type keeps its default: an enum vector set by name into a clone of full_print_config()
        // dereferences the clone's null keys_map at the pin (StaticCache::finalize, PrintConfig.hpp:880-897).
        c.set_num_filaments(4);
        deser(c, "nozzle_diameter", "0.4,0.6");
        deser(c, "filament_map", "1,2,0,2");
        deser(c, "filament_volume_map", "0,1,0,1");
        deser(c, "printer_extruder_id", "1,1,2,2");
        deser(c, "printer_extruder_variant", "Direct Drive Standard,Direct Drive High Flow,Bowden Standard,Bowden High Flow");
        deser(c, "filament_self_index", "1,1,2,2,3,4,4");
        deser(c, "filament_extruder_variant",
              "Direct Drive Standard,Direct Drive High Flow,Bowden Standard,Bowden High Flow,Direct Drive Standard,Bowden Standard,Bowden High Flow");
        deser(c, "filament_diameter_map", "\"\";\"0.8\";\"0.4;0.6\";\"0\"");
        break;
    }
    return c;
}

int main()
{
    for (int s = 0; s < N_SCENARIOS; ++s) {
        DynamicPrintConfig c = scenario_config(s);
        std::string sc = "k" + std::to_string(s);
        out(sc + " has_skirt " + hB(has_skirt(c)));
        for (float w : {0.42f, 0.5f})
            for (float h : {0.2f, 0.28f}) {
                Flow flow(w, h, 0.4f);
                out(sc + " dist_flow " + hF(w) + " " + hF(h) + " " + hF(get_real_skirt_dist(c, flow)));
            }
        std::vector<std::vector<unsigned int>> ids = {{}, {1}, {2}, {3}, {1, 2, 3}, {2, 1}};
        for (auto &v : ids) {
            std::string l = sc + " dist_ids " + hI((long long) v.size());
            for (unsigned int f : v) l += " " + hI(f);
            out(l + " " + hF(get_real_skirt_dist(c, v)));
        }
        out(sc + " dist " + hF(get_real_skirt_dist(c)));
        PrintConfig pc;
        pc.apply(c, true);
        // nozzle_diameter_for_filament and get_extruder_index on every configuration (both stay inside their
        // vectors); the two config indices only where filament_volume_map, extruder_type and the variant lists
        // are set for every filament (get_at of an empty vector is undefined).
        std::string ix = sc + " idx";
        for (unsigned int f = 0; f < 6; ++f)
            ix += " " + hI((long long) get_extruder_index(pc, f)) + " " + hF(nozzle_diameter_for_filament(pc, f));
        if (s == 9)
            for (unsigned int f = 0; f < 4; ++f)
                ix += " " + hI((long long) get_filament_config_idx(pc, f)) + " " + hI((long long) get_process_config_idx(pc, f));
        out(ix);
        std::string bs = sc + " bed_shape";
        for (bool share : {false, true}) {
            Points pts = get_bed_shape(pc, share);
            bs += " " + hI((long long) pts.size());
            for (const Point &pt : pts) bs += " " + hI(pt.x()) + " " + hI(pt.y());
        }
        out(bs);
        out(sc + " bed_excluded " + POLY(get_bed_shape_with_excluded_area(pc, false)) + " share " + POLY(get_bed_shape_with_excluded_area(pc, true)));
    }
    return 0;
}
