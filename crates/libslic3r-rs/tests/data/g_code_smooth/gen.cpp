// Generator of the vectors of tests/g_code_smooth_native_parity.rs: the pin's speed smoothing of
// discontinuity areas (GCode.cpp:7157-7170 mapping_speed / get_speed_coor_x, 7212-7359
// split_and_mapping_speed, 7402-7497 set_speed_transition), with GCode.cpp included so the private
// members are reachable, over LCG-made wall paths.
//
// Build it with target/parity-tools/build.sh (the engine's compile and link line), run it with the
// output directory, then
//     gzip -9 -n -c <out>/g_code_smooth.txt > tests/data/g_code_smooth/g_code_smooth.txt.gz
//
// Output lines (tokens separated by one blank; f32 as `%08x` of its bits, f64 as `%016llx`):
//   map <coeff f32> <dist f64> <mapping_speed f64> <speed f64> <get_speed_coor_x f64>
//   st <coeff f32> <ngroups> {<npaths> {PATH}} | <nout> {PATH}                 set_speed_transition
//   sm <coeff f32> <other_v f64> <final_v f64> <max_len f64> <left 0|1> <n> {PATH} | <ninterp> {PATH} | <nleft> {PATH}
//   mg <coeff f32> <n> {<role> <overhang_degree f64> <mm3_per_mm f64> <get_path_speed f64> <npoints> x y ...}
//      | <ngroups> {<npaths> {PATH}} | <nout> {PATH}           merge_same_speed_paths, smooth_speed_discontinuity_area
// where PATH is `<role 0 perimeter|1 external|2 overhang> <smooth_speed f64> <npoints> x y ...`.

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <array>
#include <regex>
#include <memory>
#include <mutex>
#include <thread>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <climits>
#include <numeric>
#include <chrono>
#include <atomic>
#include <list>
#include <deque>
#include <queue>
#include <stack>
#include <tuple>
#include <variant>
#include <any>
#include <bitset>
#include <random>
#include <condition_variable>
#include <future>
#include <boost/algorithm/string.hpp>
#include <boost/log/trivial.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/nowide/cstdio.hpp>
#include <boost/nowide/iostream.hpp>
#include <boost/nowide/cstdlib.hpp>
#include <boost/filesystem.hpp>
#include <boost/beast/core/detail/base64.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/container/small_vector.hpp>
#include <tbb/parallel_for.h>
#include <tbb/parallel_pipeline.h>
#include <tbb/spin_mutex.h>
#include <tbb/concurrent_vector.h>
#include <Eigen/Geometry>

#define private public
#define protected public
#include "libslic3r/Print.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode.cpp"
#undef private
#undef protected

using namespace Slic3r;


static FILE *g_out = nullptr;

static std::string f32h(float v)
{
    uint32_t b;
    std::memcpy(&b, &v, 4);
    char buf[16];
    snprintf(buf, sizeof(buf), "%08x", b);
    return buf;
}

static std::string f64h(double v)
{
    uint64_t b;
    std::memcpy(&b, &v, 8);
    char buf[24];
    snprintf(buf, sizeof(buf), "%016llx", (unsigned long long) b);
    return buf;
}

// 64-bit LCG (Knuth MMIX constants), deterministic on every platform.
static uint64_t g_seed = 0x2545F4914F6CDD1DULL;
static uint32_t lcg()
{
    g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL;
    return uint32_t(g_seed >> 33);
}
static double lcg_unit() { return double(lcg()) / 2147483648.0; } // [0, 1)
static int    lcg_int(int n) { return int(lcg() % uint32_t(n)); }

static int role_code(ExtrusionRole r) { return r == erPerimeter ? 0 : r == erExternalPerimeter ? 1 : 2; }
static ExtrusionRole role_of(int c) { return c == 0 ? erPerimeter : c == 1 ? erExternalPerimeter : erOverhangPerimeter; }

static void put_path(const ExtrusionPath &p)
{
    fprintf(g_out, " %d %s %zu", role_code(p.role()), f64h(p.smooth_speed).c_str(), p.polyline.points.size());
    for (const Point &pt : p.polyline.points)
        fprintf(g_out, " %d %d", pt.x(), pt.y());
}
static void put_paths(const ExtrusionPaths &ps)
{
    fprintf(g_out, " %zu", ps.size());
    for (const ExtrusionPath &p : ps)
        put_path(p);
}

// A wall path continuing from `cur`: 2..6 points, each step a random direction and a length that is
// short (below min_step_length * 1.5 = 0.6 mm) or long, so both branches of length_enough are taken.
static ExtrusionPath make_path(Point &cur, ExtrusionRole role, double speed)
{
    ExtrusionPath p(role, 0.05, 0.45f, 0.2f);
    int np = 2 + lcg_int(5);
    p.polyline.points.push_back(cur);
    for (int i = 1; i < np; ++i) {
        double ang = lcg_unit() * 6.283185307179586;
        int    k   = lcg_int(4);
        double len = k == 0 ? 0.005 + 0.1 * lcg_unit() : k == 1 ? 0.1 + 0.6 * lcg_unit() : k == 2 ? 0.55 + 0.1 * lcg_unit() : 0.6 + 6.0 * lcg_unit();
        Point  nx(cur.x() + coord_t(std::lround(len * 100000. * std::cos(ang))), cur.y() + coord_t(std::lround(len * 100000. * std::sin(ang))));
        if (nx == cur)
            nx.x() += 7;
        p.polyline.points.push_back(nx);
        cur = nx;
    }
    p.smooth_speed = speed;
    return p;
}

static const double k_wall_speeds[] = {30., 50., 60., 80., 120., 150., 200., 250., 300.};
static const double k_over_speeds[] = {10., 15., 20., 25., 40.};
static const float  k_coeffs[]      = {4.f, 80.f, 150.f, 2.7f, 0.35f};

static void mapping_cases(GCode &g)
{
    for (float c : k_coeffs) {
        g.m_smooth_coefficient = c;
        for (int i = 0; i < 40; ++i) {
            double dist  = i == 0 ? 0. : i == 1 ? -1. : 0.01 + 12. * lcg_unit();
            double speed = i == 0 ? 0. : 1. + 400. * lcg_unit();
            fprintf(g_out, "map %s %s %s %s %s\n", f32h(c).c_str(), f64h(dist).c_str(), f64h(g.mapping_speed(dist)).c_str(), f64h(speed).c_str(),
                    f64h(g.get_speed_coor_x(speed)).c_str());
        }
    }
}

static void transition_cases(GCode &g, int n)
{
    for (int cs = 0; cs < n; ++cs) {
        float c = k_coeffs[lcg_int(5)];
        g.m_smooth_coefficient = c;
        int    ng  = 2 + lcg_int(6);
        Point  cur(coord_t(lcg_int(2000000)) - 1000000, coord_t(lcg_int(2000000)) - 1000000);
        std::vector<ExtrusionPaths> groups;
        for (int gi = 0; gi < ng; ++gi) {
            ExtrusionPaths grp;
            if (lcg_int(4) == 0) {
                grp.push_back(make_path(cur, erOverhangPerimeter, k_over_speeds[lcg_int(5)]));
            } else {
                ExtrusionRole r  = lcg_int(2) ? erPerimeter : erExternalPerimeter;
                double        v  = k_wall_speeds[lcg_int(9)];
                int           np = 1 + lcg_int(3);
                for (int k = 0; k < np; ++k)
                    grp.push_back(make_path(cur, r, v));
            }
            groups.push_back(std::move(grp));
        }
        fprintf(g_out, "st %s %zu", f32h(c).c_str(), groups.size());
        for (const ExtrusionPaths &grp : groups)
            put_paths(grp);
        ExtrusionPaths out = g.set_speed_transition(groups);
        fprintf(g_out, " |");
        put_paths(out);
        fprintf(g_out, "\n");
    }
}

static void split_cases(GCode &g, int n)
{
    for (int cs = 0; cs < n; ++cs) {
        float c = k_coeffs[lcg_int(5)];
        g.m_smooth_coefficient = c;
        Point          cur(coord_t(lcg_int(2000000)) - 1000000, coord_t(lcg_int(2000000)) - 1000000);
        ExtrusionRole  r  = lcg_int(2) ? erPerimeter : erExternalPerimeter;
        double         fv = k_wall_speeds[lcg_int(9)];
        double         ov = k_over_speeds[lcg_int(5)];
        int            np = 1 + lcg_int(4);
        ExtrusionPaths ps;
        for (int k = 0; k < np; ++k)
            ps.push_back(make_path(cur, r, fv));
        double total = 0.;
        for (const ExtrusionPath &p : ps)
            total += p.length();
        int    mk     = lcg_int(4);
        double maxlen = mk == 0 ? total : mk == 1 ? total / 2 : mk == 2 ? 10000. + 200000. * lcg_unit() : 0.;
        bool   left   = lcg_int(2) == 0;
        if (!left) {
            std::reverse(ps.begin(), ps.end());
        }
        fprintf(g_out, "sm %s %s %s %s %d", f32h(c).c_str(), f64h(ov).c_str(), f64h(fv).c_str(), f64h(maxlen).c_str(), int(left));
        put_paths(ps);
        ExtrusionPaths interp;
        g.split_and_mapping_speed(ov, fv, ps, maxlen, interp, left);
        fprintf(g_out, " |");
        put_paths(interp);
        fprintf(g_out, " |");
        put_paths(ps);
        fprintf(g_out, "\n");
    }
}

// GCode::merge_same_speed_paths (GCode.cpp:7362-7400) and GCode::smooth_speed_discontinuity_area
// (GCode.cpp:7499-7512) with the pin's get_path_speed (GCode.cpp:6615-6662) over the default config, one
// filament and no layer (m_print and m_layer null: the config indices are 0, no path touches the bed).
static void merge_cases(GCode &g, int n)
{
    static const double k_mm3[]    = {0.05, 0.08, 0.02};
    static const double k_degree[] = {0., 0., 0., 1., 2., 2.5, 3.75, 4., 5., 0.3};
    for (int cs = 0; cs < n; ++cs) {
        float c = k_coeffs[lcg_int(5)];
        g.m_smooth_coefficient = c;
        Point          cur(coord_t(lcg_int(2000000)) - 1000000, coord_t(lcg_int(2000000)) - 1000000);
        int            np = 2 + lcg_int(9);
        ExtrusionPaths ps;
        ExtrusionRole  r   = lcg_int(2) ? erPerimeter : erExternalPerimeter;
        double         mm3 = k_mm3[lcg_int(3)];
        for (int k = 0; k < np; ++k) {
            int           kind = lcg_int(5);
            ExtrusionRole pr   = kind == 0 ? erOverhangPerimeter : r;
            if (kind == 1)
                mm3 = k_mm3[lcg_int(3)];
            ExtrusionPath p    = make_path(cur, pr, 0.);
            p.mm3_per_mm       = mm3;
            int           kd   = lcg_int(13);
            p.overhang_degree  = pr == erOverhangPerimeter ? (lcg_int(2) ? 5. : 4.) : kd < 10 ? k_degree[kd] : 4. * (lcg_unit() + lcg_unit() / 2147483648.); // full-mantissa degrees
            ps.push_back(p);
        }
        fprintf(g_out, "mg %s %zu", f32h(c).c_str(), ps.size());
        for (const ExtrusionPath &p : ps) {
            fprintf(g_out, " %d %s %s %s %zu", role_code(p.role()), f64h(p.overhang_degree).c_str(), f64h(p.mm3_per_mm).c_str(),
                    f64h(g.get_path_speed(p)).c_str(), p.polyline.points.size());
            for (const Point &pt : p.polyline.points)
                fprintf(g_out, " %d %d", pt.x(), pt.y());
        }
        std::vector<ExtrusionPaths> groups = g.merge_same_speed_paths(ps);
        fprintf(g_out, " | %zu", groups.size());
        for (const ExtrusionPaths &grp : groups)
            put_paths(grp);
        g.smooth_speed_discontinuity_area(ps);
        fprintf(g_out, " |");
        put_paths(ps);
        fprintf(g_out, "\n");
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: gen <out dir>\n");
        return 1;
    }
    std::string dir = argv[1];
    g_out           = fopen((dir + "/g_code_smooth.txt").c_str(), "wb");
    GCode g;
    mapping_cases(g);
    transition_cases(g, 1500);
    split_cases(g, 1500);
    g.m_writer.set_extruders({0});
    g.m_writer.toolchange(0, 0);
    {
        // Speeds that differ per overhang degree, so the merge and the ramps see several speeds.
        DynamicPrintConfig cfg;
        for (const auto &kv : std::vector<std::pair<std::string, std::string>>{
                 {"filament_max_volumetric_speed", "40"}, {"inner_wall_speed", "200"}, {"outer_wall_speed", "120"},
                 {"overhang_1_4_speed", "100"}, {"overhang_2_4_speed", "70"}, {"overhang_3_4_speed", "40"},
                 {"overhang_4_4_speed", "15"}, {"overhang_totally_speed", "10"}, {"bridge_speed", "25"},
                 {"enable_overhang_speed", "1"}})
            cfg.set_deserialize_strict(kv.first, kv.second);
        g.m_config.apply(cfg, true);
    }
    merge_cases(g, 1000);
    fclose(g_out);
    return 0;
}
