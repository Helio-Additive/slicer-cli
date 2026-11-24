// Generator of the vectors of tests/g_code_native_parity.rs: the pin's GCode.cpp (included, so its
// file-static functions and the private members are reachable) over standalone inputs.
//
// Build it against a Bambu Studio build tree of this repository (the include paths and libraries of
// libslic3r), run it with the output directory, then
//     gzip -9 -n -c <out>/g_code.txt > tests/data/g_code/g_code.txt.gz
//
// Output lines (tokens separated by one blank; floats as the hex of their bits, f32 `%08x`, f64
// `%016llx`; strings as hex of their bytes, `-` when empty):
//   cgst <hex gcode> <dont_wait> <and_wait> <g10> <ret> <temp>     custom_gcode_sets_temperature
//   b64 <u64 hex> <text>                                           encodeBase64
//   infer <hex gcode> <safe x> <safe y> <z f32> <has> <x> <y> <z>  infer_timelapse_final_plate_pos
//   eol <hex in> <hex out>                                         check_add_eol
//   cgct <hex gcode> <hex prefix> <next> <ret>                     custom_gcode_changes_tool
//   tg <hex gcode> <pos x> <pos y> <tr x> <tr y> <angle> <hex out> transform_gcode (f32 inputs)
//   wapx <min x> <min y> <max x> <max y> <offset> <out>            get_wipe_avoid_pos_x (f32)
//   ovs <slope f64> <n> (<len_units> <degree f64>)... <ret>        has_overhang_path_on_slope over a
//                                                                  loop of straight paths along X
//   cmvs <lh f64> <lw f64> <hex co_str> <res f64>                  GCode::calc_max_volumetric_speed
//   label <n ids> <ids...> <n query> <query...> <ok> <hex out_or_error>   _encode_label_ids_to_base64
//   remain <n ids> <ids...> <n clear> <clear...> <hex text>       _init_remaining_objects_bitmap,
//                                                                  _clear_remaining_object, _remaining_objects_gcode
//   origin <last x> <last y> <n wipe> (<x> <y>)... <ox f64> <oy f64> <nx f64> <ny f64> <out last x> <out last y> (<x> <y>)...
//                                                                  set_origin from (ox, oy) to (nx, ny)
//   afc <n keys> (<key> <hex value>)... <hex text>                 GCode::append_full_config
//   vcg <n keys> (<key> <hex value>)... <n> (<hex source> <hex tag>)...   DoExport::validate_custom_gcode
//   mass <weight f64> <n keys> (<key> <hex value>)... <y limit f64> <mass f64>   mass_load_limited_machine_acceleration
//   env <extruder_id> <n keys> (<key> <hex value>)... <hex text>   print_machine_envelope
//   skirt <fn> <loops> <has_skirt> <has_inf> <seq> <min_len f32> <n keys> (<key> <hex value>)...
//         <print_z f64> <has_skirt lt> <n ext> <ext>... <n done> <done f64>... <last_filament>
//         -> <n out> (<ext> <first> <last>)... <n done> <done f64>... <last_filament>
//                                                                  Skirt::make_skirt_loops_* (fn 0: all
//                                                                  printing, 1: multi diameter, 2: 1st
//                                                                  layer, 3: other layers)

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

static std::string hex(const std::string &s)
{
    if (s.empty())
        return "-";
    static const char *d = "0123456789abcdef";
    std::string o;
    o.reserve(s.size() * 2);
    for (unsigned char c : s) {
        o += d[c >> 4];
        o += d[c & 15];
    }
    return o;
}

static std::string f32(float v)
{
    uint32_t b;
    std::memcpy(&b, &v, 4);
    char buf[16];
    snprintf(buf, sizeof(buf), "%08x", b);
    return buf;
}

static std::string f64(double v)
{
    uint64_t b;
    std::memcpy(&b, &v, 8);
    char buf[24];
    snprintf(buf, sizeof(buf), "%016llx", (unsigned long long) b);
    return buf;
}

using KVs = std::vector<std::pair<std::string, std::string>>;

static std::string kv_str(const KVs &kv)
{
    std::string s = std::to_string(kv.size());
    for (const auto &p : kv)
        s += " " + p.first + " " + hex(p.second);
    return s;
}

static DynamicPrintConfig kv_cfg(const KVs &kv)
{
    DynamicPrintConfig c;
    for (const auto &p : kv)
        c.set_deserialize_strict(p.first, p.second);
    return c;
}

static void free_functions()
{
    // custom_gcode_sets_temperature
    std::vector<std::string> gcodes = {
        "",
        "M104 S200\n",
        "M109 S215 ; wait\n",
        "  M104 T1 S180\nG1 X1\n",
        "M140 S60\nM190 S65\n",
        "G10 S205\n",
        "G10 P0 R150 S210\n",
        "M104\n",
        "M104 S\n",
        "M1040 S1\n",
        "M10 S5\n",
        "M104 Sabc\n",
        "M104 S 230\r\nM109 S240\n",
        "M104S200\n",
        "G1 X1\nM109 S-5\n",
        "M109 S99999999999\n",
        "M190 S70;comment\n",
        "\tM104\tS190\n",
        "G10\n",
        "G100 S1\n",
        "M104 X1 S2 Y3\n",
        "; M104 S200\n",
        "M+104 S1\n",
        "M 104 S1\n",
        "M104 S1.5\n",
    };
    for (const std::string &g : gcodes) {
        for (auto [a, b] : {std::pair<int, int>{104, 109}, std::pair<int, int>{140, 190}}) {
            for (bool g10 : {false, true}) {
                int  temp = 0;
                bool r    = custom_gcode_sets_temperature(g, a, b, g10, temp);
                fprintf(g_out, "cgst %s %d %d %d %d %d\n", hex(g).c_str(), a, b, int(g10), int(r), temp);
            }
        }
    }
    // encodeBase64
    for (uint64_t v : {uint64_t(0), uint64_t(1), uint64_t(3), uint64_t(0xff), uint64_t(0x100), uint64_t(0x8000000000000000ull),
                       uint64_t(0xffffffffffffffffull), uint64_t(0x0123456789abcdefull), uint64_t(0x7f)})
        fprintf(g_out, "b64 %016llx %s\n", (unsigned long long) v, encodeBase64(v).c_str());
    // infer_timelapse_final_plate_pos
    std::vector<std::string> tl = {
        "",
        "G1 X10 Y20\n",
        "M9711 X5 Y6 Z7\n",
        "M9711 U5 V6\nG1 X1 Y2\n",
        "M9711 X5\nG1 Z3\nM9711 Y9\nG1 X4\n",
        "M9711 U1 X2 V3 Y4\nG0 X8 Y9 Z1.5\nM400\n",
        "G1 X3\nM9711\n",
    };
    for (const std::string &g : tl) {
        Point safe(scaled<coord_t>(10.), scaled<coord_t>(20.));
        auto  r = infer_timelapse_final_plate_pos(g, safe, 0.4f);
        fprintf(g_out, "infer %s %lld %lld %s %d %s %s %s\n", hex(g).c_str(), (long long) safe.x(), (long long) safe.y(), f32(0.4f).c_str(),
                int(bool(r)), f32(r ? r->x() : 0.f).c_str(), f32(r ? r->y() : 0.f).c_str(), f32(r ? r->z() : 0.f).c_str());
    }
    // check_add_eol
    for (std::string s : {std::string(""), std::string("a"), std::string("a\n"), std::string("\n"), std::string("a\r")}) {
        std::string o = s;
        check_add_eol(o);
        fprintf(g_out, "eol %s %s\n", hex(s).c_str(), hex(o).c_str());
    }
    // custom_gcode_changes_tool
    for (std::string g : {std::string("T1\n"), std::string("G1\nT2\n"), std::string("  T1"), std::string("xT1\n"), std::string("T"),
                          std::string("T1\nT3\n"), std::string(";T1\n"), std::string("M620 S1\nT1 ; tool\n"), std::string("\tT01\n")}) {
        for (unsigned next : {1u, 2u, 3u})
            fprintf(g_out, "cgct %s %s %u %d\n", hex(g).c_str(), hex("T").c_str(), next, int(custom_gcode_changes_tool(g, "T", next)));
    }
    // transform_gcode
    std::vector<std::string> tgs = {
        "G1 X1 Y2 E0.5\nG1 X3\nM104 S200\nG1 Y4 F1200\n",
        "G1 X0 Y0\nG1 X0 Y0\nG1 X1 Y0\n",
        std::string("G1 X1 Y1") + WipeTower::never_skip_tag() + "\nG1 X1 Y1\n",
        "G0 X1\nG1 E1\n",
    };
    for (const std::string &g : tgs) {
        for (float angle : {0.f, 90.f, 30.f}) {
            Vec2f pos(1.5f, -2.f), tr(100.f, 50.25f);
            std::string out = transform_gcode(g, pos, tr, float(angle * M_PI / 180.));
            fprintf(g_out, "tg %s %s %s %s %s %s %s\n", hex(g).c_str(), f32(pos.x()).c_str(), f32(pos.y()).c_str(), f32(tr.x()).c_str(),
                    f32(tr.y()).c_str(), f32(float(angle * M_PI / 180.)).c_str(), hex(out).c_str());
        }
    }
    // get_wipe_avoid_pos_x
    for (auto box : std::vector<std::array<float, 5>>{{10, 10, 50, 50, 5}, {120, 10, 160, 50, 5}, {200, 10, 260, 50, 10}, {90, 0, 99, 1, 2}, {240, 0, 300, 1, 2}}) {
        float r = get_wipe_avoid_pos_x(Vec2f(box[0], box[1]), Vec2f(box[2], box[3]), box[4]);
        fprintf(g_out, "wapx %s %s %s %s %s %s\n", f32(box[0]).c_str(), f32(box[1]).c_str(), f32(box[2]).c_str(), f32(box[3]).c_str(),
                f32(box[4]).c_str(), f32(r).c_str());
    }
    // has_overhang_path_on_slope
    std::vector<std::pair<double, std::vector<std::pair<long, double>>>> loops = {
        {1e6, {{100000, 0.}, {200000, 0.5}, {300000, 2.}}},
        {250000., {{100000, 0.}, {200000, 0.5}, {300000, 2.}}},
        {50000., {{100000, 1.5}}},
        {0., {{100000, 0.}, {100000, 3.}}},
        {300000., {{100000, 1.}, {100000, 1.}, {100000, 1.}, {100000, 1.01}}},
        {1e6, {}},
    };
    for (const auto &l : loops) {
        ExtrusionPaths paths;
        coord_t        x = 0;
        for (const auto &p : l.second) {
            ExtrusionPath path(erExternalPerimeter);
            path.polyline.points = {Point(x, 0), Point(x + p.first, 0)};
            x += p.first;
            path.overhang_degree = p.second;
            paths.push_back(path);
        }
        ExtrusionLoop loop(paths);
        bool          r = has_overhang_path_on_slope(loop, l.first);
        fprintf(g_out, "ovs %s %zu", f64(l.first).c_str(), l.second.size());
        for (const auto &p : l.second)
            fprintf(g_out, " %ld %s", p.first, f64(p.second).c_str());
        fprintf(g_out, " %d\n", int(r));
    }
}

static void gcode_methods()
{
    GCode g;
    // calc_max_volumetric_speed
    for (std::string co : {std::string("1 2 3 4 5 6"), std::string("0 0 0 0 0 0"), std::string("1 2 3"), std::string("1  2 3 4 5 6"),
                           std::string("-1.5 2.25 0 4e-1 5 6 "), std::string("a 2 3 4 5 6"), std::string(""), std::string("1 2 3 4 5 6 7"),
                           std::string("0.5 0.25 0.125 1 2 3"), std::string("1x 2 3 4 5 6")}) {
        for (auto lw : std::vector<std::pair<double, double>>{{0.2, 0.42}, {0.08, 0.6}}) {
            double r = g.calc_max_volumetric_speed(lw.first, lw.second, co);
            fprintf(g_out, "cmvs %s %s %s %s\n", f64(lw.first).c_str(), f64(lw.second).c_str(), hex(co).c_str(), f64(r).c_str());
        }
    }
    // _encode_label_ids_to_base64
    std::vector<std::pair<std::vector<size_t>, std::vector<size_t>>> lab = {
        {{3, 7, 9}, {7}}, {{3, 7, 9}, {3, 9}}, {{3, 7, 9}, {}}, {{3, 7, 9}, {4}}, {{1, 2, 3, 4, 5, 6, 7, 8, 9, 10}, {10, 1}},
    };
    for (const auto &l : lab) {
        GCode gg;
        gg.m_label_objects_ids = l.first;
        std::string out;
        bool        ok = true;
        try {
            out = gg._encode_label_ids_to_base64(l.second);
        } catch (const std::exception &e) {
            ok  = false;
            out = e.what();
        }
        fprintf(g_out, "label %zu", l.first.size());
        for (size_t v : l.first)
            fprintf(g_out, " %zu", v);
        fprintf(g_out, " %zu", l.second.size());
        for (size_t v : l.second)
            fprintf(g_out, " %zu", v);
        fprintf(g_out, " %d %s\n", int(ok), hex(out).c_str());
    }
    // remaining objects
    std::vector<std::pair<std::vector<size_t>, std::vector<size_t>>> rem = {
        {{3, 7, 9}, {}}, {{3, 7, 9}, {7}}, {{3, 7, 9}, {3, 9, 7}}, {{3, 7, 9}, {5}}, {{}, {}},
    };
    {
        std::vector<size_t> many;
        for (size_t i = 0; i < 64; ++i)
            many.push_back(i * 2);
        rem.push_back({many, {0, 126}});
    }
    for (const auto &l : rem) {
        GCode gg;
        gg.m_label_objects_ids = l.first;
        gg._init_remaining_objects_bitmap();
        for (size_t c : l.second)
            gg._clear_remaining_object(c);
        std::string out = gg._remaining_objects_gcode();
        fprintf(g_out, "remain %zu", l.first.size());
        for (size_t v : l.first)
            fprintf(g_out, " %zu", v);
        fprintf(g_out, " %zu", l.second.size());
        for (size_t v : l.second)
            fprintf(g_out, " %zu", v);
        fprintf(g_out, " %s\n", hex(out).c_str());
    }
    // set_origin
    std::vector<std::array<double, 4>> origins = {{0, 0, 10.5, -3.25}, {100, 50, 100, 50}, {1.000004, 2.000006, 0, 0}, {-5, 7, 3.3333333, 9.9999999}};
    for (const auto &o : origins) {
        GCode gg;
        gg.m_origin = Vec2d(o[0], o[1]);
        gg.set_last_pos(Point(123456, -654321));
        gg.m_wipe.path.points = {Point(0, 0), Point(1000000, 5), Point(-7, 33)};
        gg.set_origin(Vec2d(o[2], o[3]));
        fprintf(g_out, "origin 123456 -654321 3 0 0 1000000 5 -7 33 %s %s %s %s %lld %lld", f64(o[0]).c_str(), f64(o[1]).c_str(),
                f64(o[2]).c_str(), f64(o[3]).c_str(), (long long) gg.last_pos().x(), (long long) gg.last_pos().y());
        for (const Point &p : gg.m_wipe.path.points)
            fprintf(g_out, " %lld %lld", (long long) p.x(), (long long) p.y());
        fprintf(g_out, "\n");
    }
    // append_full_config
    std::vector<KVs> afcs = {
        {{"layer_height", "0.2"}, {"compatible_printers", "a;b"}, {"filament_type", "PLA;PETG"}, {"print_host", "x"}, {"wall_loops", "3"}},
        {{"machine_start_gcode", "G28\nG1 Z5"}, {"nozzle_diameter", "0.4,0.6"}, {"printhost_port", "1"}},
        {},
    };
    for (const KVs &kv : afcs) {
        std::string s;
        GCode::append_full_config(kv_cfg(kv), s);
        fprintf(g_out, "afc %s %s\n", kv_str(kv).c_str(), hex(s).c_str());
    }
    // validate_custom_gcode
    std::vector<KVs> vcgs = {
        {{"machine_start_gcode", "G28\n; FEATURE: x\n"}, {"machine_end_gcode", ";LAYER_CHANGE\n"}},
        {{"machine_start_gcode", "; FEATURE: a\n; WIPE_START\n; WIPE_END\n"}, {"layer_change_gcode", "; CHANGE_LAYER\n;HEIGHT:1\n; LAYER_HEIGHT: 1\n"},
         {"filament_start_gcode", "; LINE_WIDTH: 1\n;\"x\""}},
        {{"filament_start_gcode", "; FEATURE: a\n\";\";\"; WIPE_END\n\""}, {"filament_end_gcode", "\"; PAUSE_PRINT\n\";\"\""}},
        {},
    };
    for (const KVs &kv : vcgs) {
        Print print;
        print.m_config.apply(kv_cfg(kv), true);
        auto r = DoExport::validate_custom_gcode(print);
        fprintf(g_out, "vcg %s %zu", kv_str(kv).c_str(), r.size());
        for (const auto &p : r)
            fprintf(g_out, " %s %s", hex(p.first).c_str(), hex(p.second).c_str());
        fprintf(g_out, "\n");
    }
    // mass_load_limited_machine_acceleration
    std::vector<std::pair<double, KVs>> masses = {
        {0., {{"machine_max_acceleration_y", "20000,9000"}}},
        {250.5, {{"machine_max_acceleration_y", "20000,9000"}, {"machine_max_force_Y", "200"}, {"machine_bed_mass_Y", "1500"}}},
        {0., {{"machine_max_acceleration_y", "20000,9000"}, {"machine_max_force_Y", "200"}, {"machine_bed_mass_Y", "1500"}}},
        {1000., {{"machine_max_acceleration_y", "500"}, {"machine_max_force_Y", "50"}, {"machine_bed_mass_Y", "100"}}},
    };
    for (const auto &m : masses) {
        Print print;
        print.m_config.apply(kv_cfg(m.second), true);
        PrintStatistics st;
        st.total_weight = m.first;
        double y = -1, mass = -1;
        g.mass_load_limited_machine_acceleration(st, print, y, mass);
        fprintf(g_out, "mass %s %s %s %s\n", f64(m.first).c_str(), kv_str(m.second).c_str(), f64(y).c_str(), f64(mass).c_str());
    }
    // print_machine_envelope
    std::vector<std::pair<int, KVs>> envs = {
        {0, {{"gcode_flavor", "marlin2"}}},
        {0, {{"gcode_flavor", "marlin"}, {"machine_max_jerk_x", "8.125,7"}, {"machine_max_speed_e", "29.5,20"}}},
        {1, {{"gcode_flavor", "marlin2"}, {"filament_map", "1,2"}, {"machine_max_speed_x", "500,400,300,200"}, {"machine_max_acceleration_x", "100,200,300,400"},
             {"machine_max_acceleration_y", "1,2,3,4"}, {"machine_max_acceleration_z", "1,2,3,4"}, {"machine_max_acceleration_e", "1,2,3,4"},
             {"machine_max_speed_y", "1,2,3,4"}, {"machine_max_speed_z", "1,2,3,4"}, {"machine_max_speed_e", "1,2,3,4"}}},
        {0, {{"gcode_flavor", "klipper"}}},
        {0, {{"gcode_flavor", "marlin2"}, {"machine_max_jerk_y", "0.125,1"}, {"machine_max_jerk_z", "0.375,1"}, {"machine_max_jerk_e", "2.675,1"}}},
    };
    for (const auto &e : envs) {
        Print print;
        print.m_config.apply(kv_cfg(e.second), true);
        GCode gg;
        gg.m_config.apply(print.m_config);
        GCodeProcessor proc;
        FILE *f = tmpfile();
        {
            GCode::GCodeOutputStream file(f, proc);
            gg.print_machine_envelope(file, print, e.first);
            file.flush();
            std::string text;
            fseek(f, 0, SEEK_SET);
            char buf[4096];
            size_t n;
            while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
                text.append(buf, n);
            fprintf(g_out, "env %d %s %s\n", e.first, kv_str(e.second).c_str(), hex(text).c_str());
        }
    }
}

static void skirt()
{
    struct SC
    {
        int                       fn;
        size_t                    loops;
        bool                      has_skirt, has_inf, seq;
        float                     min_len;
        KVs                        kv;
        double                    print_z;
        bool                      lt_has_skirt;
        std::vector<unsigned int> ext;
        std::vector<double>       done;
        unsigned int              last;
    };
    KVs base = {{"skirt_height", "2"}, {"skirt_per_object", "0"}, {"nozzle_diameter", "0.4,0.2"}, {"filament_map", "1,2,1"}};
    std::vector<SC> cs;
    for (int fn = 0; fn <= 3; ++fn) {
        for (float min_len : {0.f, 5.f}) {
            cs.push_back({fn, 3, true, false, false, min_len, base, 0.2, true, {0, 1}, {}, 0});
            cs.push_back({fn, 3, true, false, false, min_len, base, 0.4, true, {1, 0, 2}, {0.2}, 0});
            cs.push_back({fn, 5, true, false, false, min_len, base, 0.6, true, {2}, {0.2, 0.4}, 0});
            cs.push_back({fn, 5, true, true, false, min_len, base, 0.6, true, {2, 0}, {0.2, 0.4}, 2});
            cs.push_back({fn, 3, true, false, false, min_len, base, 0.4, false, {0}, {0.2}, 0});
            cs.push_back({fn, 3, false, false, false, min_len, base, 0.2, true, {0}, {}, 0});
            cs.push_back({fn, 0, true, false, false, min_len, base, 0.2, true, {0}, {}, 0});
            cs.push_back({fn, 3, true, false, true, min_len, KVs{{"skirt_height", "2"}, {"skirt_per_object", "1"}}, 0.2, true, {0}, {}, 0});
            cs.push_back({fn, 4, true, false, false, min_len, base, 0.3, true, {1, 0}, {0.2}, 1});
            cs.push_back({fn, 4, true, false, false, min_len, base, 0.25, true, {0, 1}, {0.2}, 1});
            cs.push_back({fn, 4, true, false, false, min_len, base, 1.2, true, {0, 1}, {0.2}, 1});
            cs.push_back({fn, 4, true, false, false, min_len, base, 0.2, true, {0, 1}, {0.2}, 1});
        }
    }
    for (const SC &c : cs) {
        Print print;
        print.m_config.apply(kv_cfg(c.kv), true);
        // The skirt loops: what make_skirt_loops_* reads of them is their count.
        for (size_t i = 0; i < c.loops; ++i)
            print.m_skirt.entities.push_back(new ExtrusionLoop());
        Print::min_skirt_length = c.min_len;
        // has_skirt(), has_infinite_skirt() and is_sequential_print() come from the configuration; the
        // values the functions see are written beside the case for the port's view of the Print.
        bool has_skirt = print.has_skirt(), has_inf = print.has_infinite_skirt(), seq = print.is_sequential_print();
        (void) c.has_skirt;
        (void) c.has_inf;
        (void) c.seq;
        LayerTools lt(c.print_z);
        lt.has_skirt = c.lt_has_skirt;
        lt.extruders = c.ext;
        std::vector<coordf_t>                             done = c.done;
        unsigned int                                      last = c.last;
        std::map<unsigned int, std::pair<size_t, size_t>> out;
        switch (c.fn) {
        case 0: Skirt::skirt_loops_per_extruder_all_printing(print, lt, out); break;
        case 1: out = Skirt::make_skirt_loops_multi_diameter(print, lt, done, last); break;
        case 2: out = Skirt::make_skirt_loops_per_extruder_1st_layer(print, lt, done); break;
        default: out = Skirt::make_skirt_loops_per_extruder_other_layers(print, lt, done); break;
        }
        fprintf(g_out, "skirt %d %zu %d %d %d %s %s %s %d %zu", c.fn, c.loops, int(has_skirt), int(has_inf), int(seq), f32(c.min_len).c_str(),
                kv_str(c.kv).c_str(), f64(c.print_z).c_str(), int(c.lt_has_skirt), c.ext.size());
        for (unsigned e : c.ext)
            fprintf(g_out, " %u", e);
        fprintf(g_out, " %zu", c.done.size());
        for (double d : c.done)
            fprintf(g_out, " %s", f64(d).c_str());
        fprintf(g_out, " %u -> %zu", c.last, out.size());
        for (const auto &o : out)
            fprintf(g_out, " %u %zu %zu", o.first, o.second.first, o.second.second);
        fprintf(g_out, " %zu", done.size());
        for (double d : done)
            fprintf(g_out, " %s", f64(d).c_str());
        fprintf(g_out, " %u\n", last);
        Print::min_skirt_length = 0;
    }
}

// The free functions of namespace DoExport and the members of GCode over the print's statistics,
// configuration and first layer.
static void export_functions()
{
    // update_print_estimated_stats: a processor result's times and volumes, the extruders of a writer
    struct Est
    {
        float                    normal, stealth;
        bool                     stealth_enabled;
        std::map<size_t, double> total, model;
        KVs                      kv;
    };
    std::vector<Est> ests = {
        {3725.5f, 4100.25f, false, {{0, 1234.5}, {2, 77.25}, {5, 10.}}, {{0, 1000.}, {2, 50.}}, {}},
        {59.f, 61.f, true, {{1, 0.125}}, {{1, 0.1}}, {{"filament_diameter", "1.75,2.85"}, {"filament_density", "1.24,1.04"}, {"filament_cost", "20,35.5"}}},
        {90061.f, 0.f, true, {}, {}, {}},
        {0.f, 7.5f, false, {{0, 25000.}, {1, 3.75}, {2, 1.}}, {{2, 1.}}, {{"filament_diameter", "1.75,1.75,2.85"}, {"filament_density", "1.27"}, {"filament_cost", "0,12,30"}}},
    };
    for (const Est &e : ests) {
        GCodeProcessor proc;
        proc.m_result.print_statistics.modes[0].time = e.normal;
        proc.m_result.print_statistics.modes[1].time = e.stealth;
        proc.m_result.print_statistics.total_volumes_per_extruder = e.total;
        proc.m_result.print_statistics.model_volumes_per_extruder = e.model;
        proc.enable_stealth_time_estimator(e.stealth_enabled);
        DynamicPrintConfig dc = DynamicPrintConfig::full_print_config();
        for (const auto &p : e.kv)
            dc.set_deserialize_strict(p.first, p.second);
        PrintConfig pc;
        pc.apply(dc, true);
        GCodeWriter w;
        w.apply_print_config(pc);
        w.set_extruders({0, 1, 2});
        PrintStatistics ps;
        DoExport::update_print_estimated_stats(proc, w.extruders(), ps);
        fprintf(g_out, "est %s %s %d %s %s %s %s %s %s %s %s %zu", f32(e.normal).c_str(), f32(e.stealth).c_str(), int(e.stealth_enabled),
                kv_str(e.kv).c_str(), hex(ps.estimated_normal_print_time).c_str(), hex(ps.estimated_silent_print_time).c_str(),
                f64(ps.total_extruded_volume).c_str(), f64(ps.total_used_filament).c_str(), f64(ps.total_weight).c_str(),
                f64(ps.total_cost).c_str(), "|", e.total.size());
        for (const auto &v : e.total)
            fprintf(g_out, " %zu %s", v.first, f64(v.second).c_str());
        fprintf(g_out, " %zu", ps.filament_stats.size());
        for (const auto &v : ps.filament_stats)
            fprintf(g_out, " %zu %s", v.first, f64(v.second).c_str());
        fprintf(g_out, "\n");
    }
    // update_print_stats_and_format_filament_stats: the extruders of a writer after some extrusion
    struct Stat
    {
        bool               has_wipe_tower;
        int                toolchanges;
        std::vector<float> used;
        KVs                kv;
        std::vector<double> extrude;
    };
    std::vector<Stat> stats = {
        {false, -1, {}, {}, {1.5, 2.25, 0.125}},
        {true, 7, {10.5f, 0.f, 3.25f}, {{"filament_density", "1.24,1.04,1.3"}, {"filament_cost", "20,35.5,0"}}, {12.5, 0., 4.}},
        {true, 0, {0.f, 0.f, 0.f}, {{"filament_density", "0,1,1"}, {"filament_cost", "1,0,1"}}, {3., 2., 1.}},
        {false, 3, {1.f, 1.f, 1.f}, {{"use_relative_e_distances", "1"}, {"filament_diameter", "2.85"}}, {100., 0.001, 7.}},
    };
    for (const Stat &s : stats) {
        DynamicPrintConfig dc = DynamicPrintConfig::full_print_config();
        for (const auto &p : s.kv)
            dc.set_deserialize_strict(p.first, p.second);
        PrintConfig pc;
        pc.apply(dc, true);
        GCodeWriter w;
        w.apply_print_config(pc);
        w.set_extruders({0, 1, 2});
        for (size_t i = 0; i < s.extrude.size(); ++i) {
            w.toolchange(unsigned(i), 0);
            w.extrude_to_xy(Vec2d(10. * i, 5.), s.extrude[i]);
            w.retract();
        }
        Print print;
        print.m_wipe_tower_data.number_of_toolchanges = s.toolchanges;
        print.m_wipe_tower_data.used_filament         = s.used;
        PrintStatistics ps;
        ps.total_weight = 99.;
        std::string out = DoExport::update_print_stats_and_format_filament_stats(s.has_wipe_tower, print.m_wipe_tower_data, w.extruders(), ps);
        fprintf(g_out, "stats %d %d %zu", int(s.has_wipe_tower), s.toolchanges, s.used.size());
        for (float u : s.used)
            fprintf(g_out, " %s", f32(u).c_str());
        fprintf(g_out, " %s %zu", kv_str(s.kv).c_str(), s.extrude.size());
        for (double x : s.extrude)
            fprintf(g_out, " %s", f64(x).c_str());
        fprintf(g_out, " %s %d %s %s %s %s %s %s\n", hex(out).c_str(), ps.total_toolchanges, f64(ps.total_weight).c_str(), f64(ps.total_cost).c_str(),
                f64(ps.total_used_filament).c_str(), f64(ps.total_extruded_volume).c_str(), f64(ps.total_wipe_tower_filament).c_str(),
                f64(ps.total_wipe_tower_cost).c_str());
    }
    // polygon_to_string
    std::vector<std::pair<Vec3d, Points>> polys = {
        {Vec3d(0, 0, 0), {Point(0, 0), Point(1000000, 0), Point(1000000, 2000000)}},
        {Vec3d(128.5, -3.25, 0), {Point(12345678, -987654), Point(-1, 1), Point(33333333, 7)}},
        {Vec3d(0.1, 0.2, 0), {Point(123456789012, 5)}},
    };
    for (const auto &pp : polys) {
        Print print;
        print.set_plate_origin(pp.first);
        Polygon poly(pp.second);
        fprintf(g_out, "pts %s %s %zu", f64(pp.first.x()).c_str(), f64(pp.first.y()).c_str(), pp.second.size());
        for (const Point &q : pp.second)
            fprintf(g_out, " %lld %lld", (long long) q.x(), (long long) q.y());
        fprintf(g_out, " %s\n", hex(polygon_to_string(poly, &print)).c_str());
    }
    // GCode::apply_print_config
    std::vector<KVs> apcs = {
        {},
        {{"resolution", "0.0125"}, {"gcode_flavor", "klipper"}, {"z_hop", "0.4,0.2"}, {"machine_max_acceleration_extruding", "8000"}},
        {{"resolution", "0.3"}, {"gcode_flavor", "marlin2"}, {"machine_max_jerk_x", "7.5"}, {"machine_max_jerk_y", "6.4"},
         {"printable_area", "0x0,300x0,300x250,0x250"}, {"single_extruder_multi_material", "1"}},
    };
    for (const KVs &kv : apcs) {
        DynamicPrintConfig dc = DynamicPrintConfig::full_print_config();
        for (const auto &p : kv)
            dc.set_deserialize_strict(p.first, p.second);
        PrintConfig pc;
        pc.apply(dc, true);
        GCode gg;
        gg.apply_print_config(pc);
        fprintf(g_out, "apc %s %s %u %s %d %d %s %s %s\n", kv_str(kv).c_str(), f64(gg.m_scaled_resolution).c_str(), gg.m_writer.m_max_acceleration,
                f64(gg.m_writer.m_max_jerk).c_str(), int(gg.m_writer.m_single_extruder_multi_material), int(gg.m_writer.config.gcode_flavor.value),
                hex(gg.m_config.opt_serialize("z_hop")).c_str(), hex(gg.m_config.opt_serialize("resolution")).c_str(),
                hex(gg.m_config.opt_serialize("printable_area")).c_str());
    }
    // GCode() (GCode.hpp:175-203, :544 `FullPrintConfig m_config`): the object and region options at
    // their defaults, before and after apply_print_config (which writes the PrintConfig members only).
    {
        const char *ctor_keys[] = {"wipe_speed", "role_base_wipe_speed", "wall_loops", "layer_height", "sparse_infill_density",
                                   "seam_gap", "wall_sequence", "travel_speed", "resolution"};
        for (int applied = 0; applied < 2; ++applied) {
            GCode gg;
            if (applied) {
                DynamicPrintConfig dc = DynamicPrintConfig::full_print_config();
                dc.set_deserialize_strict("travel_speed", "321");
                dc.set_deserialize_strict("resolution", "0.02");
                PrintConfig pc;
                pc.apply(dc, true);
                gg.apply_print_config(pc);
            }
            fprintf(g_out, "ctor %d %zu", applied, gg.m_config.keys().size());
            for (const char *k : ctor_keys)
                fprintf(g_out, " %s %s", k, hex(gg.m_config.opt_serialize(k)).c_str());
            fprintf(g_out, "\n");
        }
    }
    // GCode::first_layer_projection of a print without objects
    std::vector<Points> hulls = {
        {Point(0, 0), Point(1000000, 0), Point(1000000, 2000000)},
        {Point(-5000000, 300), Point(-5000000, 300)},
        {Point(123456789, 987654321), Point(-123456789, 5), Point(0, -1)},
        {},
    };
    for (const Points &h : hulls) {
        Print print;
        print.m_first_layer_convex_hull = Polygon(h);
        GCode gg;
        fprintf(g_out, "flp %zu", h.size());
        for (const Point &q : h)
            fprintf(g_out, " %lld %lld", (long long) q.x(), (long long) q.y());
        try {
            BoundingBoxf b = gg.first_layer_projection(print);
            fprintf(g_out, " %d %s %s %s %s\n", int(b.defined), f64(b.min.x()).c_str(), f64(b.min.y()).c_str(), f64(b.max.x()).c_str(),
                    f64(b.max.y()).c_str());
        } catch (const std::exception &) {
            fprintf(g_out, " throw\n");
        }
    }
}

// Wipe::wipe (GCode.cpp:440-525), not a tool change, over a GCode after apply_print_config with
// one filament: the speed (role-based from the current speed, else travel_speed * wipe_speed %),
// the clipped path and the retraction per segment.
static void wipe_cases()
{
    std::vector<std::tuple<KVs, double, Points>> cases = {
        {{}, 0., {Point(10000000, 10000000), Point(0, 10000000)}},
        {{}, 4591.28, {Point(10000000, 10000000), Point(9000000, 10000000), Point(9000000, 9500000)}},
        {{{"role_base_wipe_speed", "0"}}, 3000., {Point(10000000, 10000000), Point(0, 10000000)}},
        {{{"wipe_speed", "50%"}, {"travel_speed", "400"}}, 0., {Point(0, 0), Point(300000, 400000), Point(-500000, 2000000)}},
        {{{"wipe_distance", "0.5"}}, 7058.823, {Point(123456, 654321), Point(223456, 654321)}},
        // Retraction during the wipe (dE per segment, GCode.cpp:456-511): retract_before_wipe below
        // 100 % leaves length * (1 - retract_before_wipe) to the wipe; the paths run over several
        // segments, the last clipped (cases 1, 2) or shorter than wipe_distance (cases 3, 4).
        {{{"retract_before_wipe", "0%"}, {"retraction_length", "0.8"}}, 4591.28,
         {Point(10000000, 10000000), Point(9950000, 10000000), Point(9950000, 9920000), Point(9870000, 9890000)}},
        {{{"retract_before_wipe", "30%"}, {"retraction_length", "1.2"}, {"wipe_distance", "3"}}, 0.,
         {Point(0, 0), Point(70000, 0), Point(70000, 90000), Point(-10000, 170000), Point(-130000, 165000)}},
        {{{"retract_before_wipe", "0%"}, {"retraction_length", "2"}}, 3000.,
         {Point(5000000, 5000000), Point(5030000, 5010000), Point(5040000, 5050000)}},
        {{{"retract_before_wipe", "70%"}, {"retraction_length", "0.45"}, {"wipe_distance", "1.7"}, {"role_base_wipe_speed", "0"}},
         2400., {Point(-200000, 30000), Point(-190000, 30000), Point(-190000, 110000), Point(-200000, 110000)}},
    };
    for (const auto &c : cases) {
        const KVs &kv = std::get<0>(c);
        DynamicPrintConfig dc = DynamicPrintConfig::full_print_config();
        for (const auto &p : kv)
            dc.set_deserialize_strict(p.first, p.second);
        PrintConfig pc;
        pc.apply(dc, true);
        GCode gg;
        gg.apply_print_config(pc);
        gg.m_writer.set_extruders({0});
        gg.m_writer.toolchange(0, 0);
        double cur = std::get<1>(c);
        if (cur > 0.)
            gg.m_writer.set_speed(cur);
        const Points &pts = std::get<2>(c);
        gg.set_last_pos(pts.front());
        gg.m_wipe.path = Polyline(pts);
        std::string out = gg.m_wipe.wipe(gg, false, false);
        fprintf(g_out, "wipe %s %s %zu", kv_str(kv).c_str(), f64(cur).c_str(), pts.size());
        for (const Point &q : pts)
            fprintf(g_out, " %lld %lld", (long long) q.x(), (long long) q.y());
        fprintf(g_out, " %s %lld %lld %d\n", hex(out).c_str(), (long long) gg.last_pos().x(), (long long) gg.last_pos().y(),
                int(gg.m_wipe.has_path()));
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: gen <out dir>\n");
        return 1;
    }
    std::string dir = argv[1];
    g_out           = fopen((dir + "/g_code.txt").c_str(), "wb");
    free_functions();
    gcode_methods();
    skirt();
    export_functions();
    wipe_cases();
    fclose(g_out);
    return 0;
}
