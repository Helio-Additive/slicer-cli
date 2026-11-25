// Generator of the vectors of tests/g_code_processor_native_parity.rs: the pin's
// GCode/GCodeProcessor.cpp (included, so its file-static helpers and the private members are
// reachable) over synthetic G-code programs and helper inputs.
//
// Build it against a Bambu Studio build tree of this repository (the include paths and libraries of
// libslic3r), run it with the output directory, then
//     gzip -9 -n -c <out>/g_code_processor.txt > tests/data/g_code_processor/g_code_processor.txt.gz
//
// Output lines (tokens separated by one blank; floats as the hex of their bits, f32 `%08x`, f64
// `%016llx`; strings as hex of their bytes, `-` when empty):
//   eols <hex text> <end>                      remove_eols over the whole text
//   pnf <hex text> <ok> <f32>                  parse_number<float> (out starts at 0)
//   pni <hex text> <ok> <int>                  parse_number<int> (out starts at 0)
//   optv <id> <value f32> <n values> <get before> <values after set...>   set/get_option_value
//   helio <hex comment> <is_helio> <min> <max> <mean>                      parse_helioadditive_comment
//   lastpos <hex text> <in x> <in y> <in z> <ret> <x> <y> <z>              get_last_position_from_gcode
//   lastz <hex text> <ret> <z f64>                                         get_last_z_from_gcode
//   lastfil <hex text> <filament>                                          get_gcode_last_filament
//   tag <hex text> <ret> <hex found>                                       contains_reserved_tag
//   tags <hex text> <max> <ret> <n> <hex found>...                         contains_reserved_tags
//   producer <id> <hex comment> <ret> <role> <layer_id> <forced w> <forced h> <flavor> <seams active>
//                                              process_producers_tags with m_producer = id, from a
//                                              fresh processor per comment
//   case <name>                                a program: the lines up to `end`
//     cfg <key> <hex value>                    the configuration applied (set_deserialize_strict)
//     stealth <0|1>                            enable_stealth_time_estimator
//     envelope <0|1>                           enable_machine_envelope_processing
//     prog <hex program>
//     time <mode> <f32>   prepare <mode> <f32>
//     roles <mode> <n> (<role> <f32>)...       get_roles_time
//     mtimes <mode> <n> (<type> <f32>)...      get_moves_time
//     layers <mode> <n> <f32>...               get_layers_time
//     cgt <mode> <n> (<type> <f32> <f32>)...   get_custom_gcode_times(mode, true)
//     vol <kind> <n> (<id> <f64>)...           print_statistics volumes per extruder
//     moves <n>
//     m <type> <role> <ext> <cp> <gcode_id> <x> <y> <z> <de> <feedrate> <width> <height> <mm3> <fan>
//       <temp> <layer_duration> <t0> <t1> <additional_fan_speed>
//     post <hex text>                          the file after finalize(true) (TimeProcessor::post_process)
//   The moves and times are those of finalize(false); the text is that of a second run with
//   finalize(true).
//   end

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
#include <boost/algorithm/string.hpp>
#include <boost/log/trivial.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/nowide/cstdio.hpp>
#include <boost/filesystem.hpp>
#include <tbb/parallel_for.h>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/Extruder.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/MultiNozzleUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"

#define private public
#define protected public
#include "libslic3r/GCode/GCodeProcessor.cpp"
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

static void helpers()
{
    // remove_eols
    for (std::string t : {std::string(""), std::string("abc"), std::string("abc\n"), std::string("abc\r\n"), std::string("\r\n\n"),
                          std::string("a\nb\n\r"), std::string("\n")}) {
        const char *e = remove_eols(t.data(), t.data() + t.size());
        fprintf(g_out, "eols %s %d\n", hex(t).c_str(), int(e - t.data()));
    }
    // parse_number
    for (std::string t : {"0", "1", "-1", "+1", "1.5", "-0.25", ".5", "5.", "1e3", "1E-2", "abc", "", "12a", " 1", "1 ", "nan",
                          "inf", "-inf", "0x10", "3.4028236e38", "1e-50", "007", "2147483647", "2147483648", "-2147483648"}) {
        float  vf = 0.f;
        bool   okf = parse_number(std::string_view(t), vf);
        fprintf(g_out, "pnf %s %d %s\n", hex(t).c_str(), int(okf), f32(vf).c_str());
        int  vi = 0;
        bool oki = parse_number(std::string_view(t), vi);
        fprintf(g_out, "pni %s %d %d\n", hex(t).c_str(), int(oki), vi);
    }
    // set_option_value / get_option_value
    for (size_t n : {0u, 1u, 2u, 3u}) {
        for (size_t id : {0u, 1u, 2u, 5u}) {
            ConfigOptionFloats opt;
            for (size_t i = 0; i < n; ++i)
                opt.values.push_back(10.0 * double(i + 1) + 0.25);
            float before = get_option_value(opt, id);
            set_option_value(opt, id, 7.125f + float(id));
            fprintf(g_out, "optv %zu %s %zu %s", id, f32(7.125f + float(id)).c_str(), n, f32(before).c_str());
            for (double v : opt.values)
                fprintf(g_out, " %s", f64(v).c_str());
            fprintf(g_out, "\n");
        }
    }
    // parse_helioadditive_comment
    for (std::string c : {std::string(";helioadditive=ti.max=0.5,ti.min=0.1,ti.mean=0.25"),
                          std::string("; something ;helioadditive=xti.max=1,ti.min=2,ti.mean=3"),
                          std::string(";helioadditive= ti.max=-1.5,ti.min=.25,ti.mean=7"),
                          std::string(";helioadditive=ti.max=1.,ti.min=2,ti.mean=3"),
                          std::string(";helioadditive=ti.max=12,ti.min=0.001,ti.mean=99a"),
                          std::string(";helioadditive=ti.max=12,ti.min=0.001,ti.mean=99 end"),
                          std::string(";helioadditive=ti.max=12,ti.min=0.001,ti.mean=99.5.5"),
                          std::string(";helioadditive=ti.max=1,ti.min=2"),
                          std::string("; no helio here ti.max=1,ti.min=2,ti.mean=3"),
                          std::string(";helioadditive=ti.max=--1,ti.min=2,ti.mean=3 ti.max=4,ti.min=5,ti.mean=6")}) {
        bool is_helio = false;
        GCodeProcessor::ThermalIndex ti = GCodeProcessor::parse_helioadditive_comment(c, is_helio);
        fprintf(g_out, "helio %s %d %s %s %s\n", hex(c).c_str(), int(is_helio), f32(ti.min).c_str(), f32(ti.max).c_str(), f32(ti.mean).c_str());
    }
    // get_last_position_from_gcode / get_last_z_from_gcode / get_gcode_last_filament
    std::vector<std::string> texts = {
        "G1 X1 Y2 Z0.2\nG1 X3 Y4 Z0.4\n",
        "G1 X10\n",
        "G1 Y5.5 F3000\nG0 Z1.25\nM104 S200\n",
        "G387 X1 Y2 J1\nG387 X0.5 Y3 J1\n",
        "G387 X1 Y2 J-1\nG387 X0.5 Y3 J-1\n",
        "  G1 X7 ;comment\nG1 Z3;;\n",
        "G1 X1\nG1 X2",
        "G2 X1 Y1 I1 J0\nG3 X2 Y2 I0 J1 Z0.6\n",
        "M620 S1A\nT1\nM621 S1A\n",
        "T3\nM104\nT255\nT1000\n",
        "G1 X123456789.5\n",
        "G1 Xabc Y1\n",
        "",
    };
    for (const std::string &t : texts) {
        Vec3f p(9.f, 8.f, 7.f);
        bool  r = GCodeProcessor::get_last_position_from_gcode(t, p);
        fprintf(g_out, "lastpos %s %s %s %s %d %s %s %s\n", hex(t).c_str(), f32(9.f).c_str(), f32(8.f).c_str(), f32(7.f).c_str(), int(r),
                f32(p.x()).c_str(), f32(p.y()).c_str(), f32(p.z()).c_str());
        double z  = -1.;
        bool   rz = GCodeProcessor::get_last_z_from_gcode(t, z);
        fprintf(g_out, "lastz %s %d %s\n", hex(t).c_str(), int(rz), f64(rz ? z : -1.).c_str());
        fprintf(g_out, "lastfil %s %d\n", hex(t).c_str(), GCodeProcessor::get_gcode_last_filament(t));
    }
    // contains_reserved_tag(s)
    std::vector<std::string> tagtexts = {
        "G1 X1\n; FEATURE: Outer wall\n",
        ";LAYER_CHANGE\n",
        "; CHANGE_LAYER\n;HEIGHT:0.2\n; WIPE_START\n; WIPE_END\n",
        "G1 X1 ; FEATURE: x\n",
        "  ; FEATURE: x\n",
        "; PAUSE_PRINT\n; CUSTOM_GCODE\n; COLOR_CHANGE\n; LAYER_HEIGHT: 0.2\n; LINE_WIDTH: 0.4\n",
        "; no tags here\nG1 X0\n",
        "",
    };
    for (const std::string &t : tagtexts) {
        std::string found;
        bool        r = GCodeProcessor::contains_reserved_tag(t, found);
        fprintf(g_out, "tag %s %d %s\n", hex(t).c_str(), int(r), hex(found).c_str());
        for (unsigned int max : {1u, 3u, 10u}) {
            std::vector<std::string> fs;
            bool                     rr = GCodeProcessor::contains_reserved_tags(t, max, fs);
            fprintf(g_out, "tags %s %u %d %zu", hex(t).c_str(), max, int(rr), fs.size());
            for (const std::string &f : fs)
                fprintf(g_out, " %s", hex(f).c_str());
            fprintf(g_out, "\n");
        }
    }
}

static void producers()
{
    std::vector<std::string> comments = {
        "TYPE:SKIRT", "TYPE:WALL-OUTER", "TYPE:WALL-INNER", "TYPE:SKIN", "TYPE:FILL", "TYPE:SUPPORT", "TYPE:SUPPORT-INTERFACE",
        "TYPE:PRIME-TOWER", "TYPE:WHAT", "TYPE:RAFT", "TYPE:SOLID-FILL", "TYPE:BRIDGE", " xTYPE:FILL", "FLAVOR:BFB", "FLAVOR:Mach3",
        "FLAVOR:Makerbot", "FLAVOR:UltiGCode", "FLAVOR:Marlin(Volumetric)", "FLAVOR:Griffin", "FLAVOR:Repetier", "FLAVOR:RepRap",
        "FLAVOR:Marlin", "FLAVOR:Other", "LAYER:3", " LAYER:3", " feature skirt", " skirt", " outer perimeter", " inner perimeter",
        " gap fill", " infill", " solid layer", " bridge", " support", " dense support", " prime pillar", " ooze shield", " raft",
        " internal single extrusion", " tool H0.2 W0.45", " tool W0.5", " tool H0.3", " tool Hx W0.4", " feature tool H0.1 W0.2",
        " layer 3", " layer end", " layer", "segType:Skirt", "segType:Perimeter", "segType:HShell", "segType:InnerHair",
        "segType:Loop", "segType:Infill", "segType:Raft", "segType:Support", "segType:SupportTouch", "segType:SoftSupport",
        "segType:Pillar", "segType:Other", " Layer #2", "Layer #2", "WIDTH:0.42", "HEIGHT:0.2", "WIDTH:x", "HEIGHT:", " 'Raft Path'",
        " 'Support Interface Path'", " 'Travel/Ironing Path'", " 'Support (may Stack) Path'", " 'Perimeter Path'", " 'Pillar Path'",
        " 'Destring/Wipe/Jump Path'", " 'Prime Pillar Path'", " 'Loop Path'", " 'Crown Path'", " 'Solid Path'",
        " 'Stacked Sparse Infill Path'", " 'Sparse Infill Path'", " BEGIN_LAYER_OBJECT z=0.4", "BEGIN_LAYER_", " nothing",
    };
    for (int id = 0; id <= 9; ++id) {
        for (const std::string &c : comments) {
            GCodeProcessor p;
            p.m_producer      = GCodeProcessor::EProducer(id);
            p.m_forced_width  = 0.f;
            p.m_forced_height = 0.f;
            bool r = p.process_producers_tags(c);
            fprintf(g_out, "producer %d %s %d %d %u %s %s %d %d\n", id, hex(c).c_str(), int(r), int(p.m_extrusion_role), p.m_layer_id,
                    f32(p.m_forced_width).c_str(), f32(p.m_forced_height).c_str(), int(p.m_flavor), int(p.m_seams_detector.is_active()));
        }
    }
}

struct Case
{
    std::string                                      name;
    std::vector<std::pair<std::string, std::string>> cfg;
    bool                                             stealth;
    std::string                                      prog;
    bool                                             envelope = false;
};

static void run_case(const Case &c, const std::string &dir)
{
    fprintf(g_out, "case %s\n", c.name.c_str());
    DynamicPrintConfig dyn;
    for (const auto &kv : c.cfg) {
        dyn.set_deserialize_strict(kv.first, kv.second);
        fprintf(g_out, "cfg %s %s\n", kv.first.c_str(), hex(kv.second).c_str());
    }
    fprintf(g_out, "stealth %d\n", int(c.stealth));
    fprintf(g_out, "envelope %d\n", int(c.envelope));
    fprintf(g_out, "prog %s\n", hex(c.prog).c_str());
    PrintConfig config;
    config.apply(dyn, true);

    std::string path = dir + "/" + c.name + ".gcode";
    {
        std::ofstream f(path, std::ios::binary);
        f << c.prog;
    }
    // The moves and times come from finalize(false): TimeProcessor::post_process (finalize(true))
    // also renumbers the moves' gcode_id for the lines it inserts. The post-processed text comes
    // from a second run with finalize(true).
    GCodeProcessor p;
    p.reset();
    p.apply_config(config);
    p.enable_stealth_time_estimator(c.stealth);
    p.enable_machine_envelope_processing(c.envelope);
    p.initialize(path);
    p.process_buffer(c.prog);
    p.finalize(false);
    {
        GCodeProcessor q;
        q.reset();
        q.apply_config(config);
        q.enable_stealth_time_estimator(c.stealth);
        q.enable_machine_envelope_processing(c.envelope);
        q.initialize(path);
        q.process_buffer(c.prog);
        q.finalize(true);
    }

    using ETM = PrintEstimatedStatistics::ETimeMode;
    for (int mode = 0; mode < 2; ++mode) {
        ETM m = ETM(mode);
        fprintf(g_out, "time %d %s\n", mode, f32(p.get_time(m)).c_str());
        fprintf(g_out, "prepare %d %s\n", mode, f32(p.get_prepare_time(m)).c_str());
        auto roles = p.get_roles_time(m);
        fprintf(g_out, "roles %d %zu", mode, roles.size());
        for (auto &r : roles)
            fprintf(g_out, " %d %s", int(r.first), f32(r.second).c_str());
        fprintf(g_out, "\n");
        auto mt = p.get_moves_time(m);
        fprintf(g_out, "mtimes %d %zu", mode, mt.size());
        for (auto &r : mt)
            fprintf(g_out, " %d %s", int(r.first), f32(r.second).c_str());
        fprintf(g_out, "\n");
        auto lt = p.get_layers_time(m);
        fprintf(g_out, "layers %d %zu", mode, lt.size());
        for (float t : lt)
            fprintf(g_out, " %s", f32(t).c_str());
        fprintf(g_out, "\n");
        auto cg = p.get_custom_gcode_times(m, true);
        fprintf(g_out, "cgt %d %zu", mode, cg.size());
        for (auto &r : cg)
            fprintf(g_out, " %d %s %s", int(r.first), f32(r.second.first).c_str(), f32(r.second.second).c_str());
        fprintf(g_out, "\n");
    }
    const auto &ps   = p.get_result().print_statistics;
    auto        vols = [&](const char *kind, const std::map<size_t, double> &m) {
        fprintf(g_out, "vol %s %zu", kind, m.size());
        for (auto &kv : m)
            fprintf(g_out, " %zu %s", kv.first, f64(kv.second).c_str());
        fprintf(g_out, "\n");
    };
    vols("total", ps.total_volumes_per_extruder);
    vols("model", ps.model_volumes_per_extruder);
    vols("support", ps.support_volumes_per_extruder);
    vols("wipe", ps.wipe_tower_volumes_per_extruder);
    vols("flush", ps.flush_per_filament);
    const auto &moves = p.get_result().moves;
    fprintf(g_out, "moves %zu\n", moves.size());
    for (const auto &mv : moves) {
        fprintf(g_out, "m %d %d %d %d %u %s %s %s %s %s %s %s %s %s %s %s %s %s %s\n", int(mv.type), int(mv.extrusion_role), int(mv.extruder_id),
                int(mv.cp_color_id), mv.gcode_id, f32(mv.position.x()).c_str(), f32(mv.position.y()).c_str(), f32(mv.position.z()).c_str(),
                f32(mv.delta_extruder).c_str(), f32(mv.feedrate).c_str(), f32(mv.width).c_str(), f32(mv.height).c_str(),
                f32(mv.mm3_per_mm).c_str(), f32(mv.fan_speed).c_str(), f32(mv.temperature).c_str(), f32(mv.layer_duration).c_str(),
                f32(mv.time[0]).c_str(), f32(mv.time[1]).c_str(), f32(mv.additional_fan_speed).c_str());
    }
    std::string post;
    {
        std::ifstream f(path, std::ios::binary);
        std::stringstream ss;
        ss << f.rdbuf();
        post = ss.str();
    }
    fprintf(g_out, "post %s\n", hex(post).c_str());
    fprintf(g_out, "end\n");
}

// The machine limits of a BBL printer, the same for both modes.
static std::vector<std::pair<std::string, std::string>> limits(const std::string &flavor)
{
    return {
        {"gcode_flavor", flavor},
        {"machine_max_acceleration_x", "20000,20000"},
        {"machine_max_acceleration_y", "20000,20000"},
        {"machine_max_acceleration_z", "500,500"},
        {"machine_max_acceleration_e", "5000,5000"},
        {"machine_max_acceleration_extruding", "20000,20000"},
        {"machine_max_acceleration_retracting", "5000,5000"},
        {"machine_max_acceleration_travel", "9000,9000"},
        {"machine_max_speed_x", "500,500"},
        {"machine_max_speed_y", "500,500"},
        {"machine_max_speed_z", "20,20"},
        {"machine_max_speed_e", "30,30"},
        {"machine_max_jerk_x", "9,9"},
        {"machine_max_jerk_y", "9,9"},
        {"machine_max_jerk_z", "3,3"},
        {"machine_max_jerk_e", "2.5,2.5"},
        {"machine_min_extruding_rate", "0,0"},
        {"machine_min_travel_rate", "0,0"},
        {"filament_diameter", "1.75"},
        {"filament_density", "1.26"},
        {"nozzle_volume", "107"},
        {"machine_load_filament_time", "0"},
        {"machine_unload_filament_time", "0"},
        {"machine_switch_extruder_time", "0"},
        {"machine_hotend_change_time", "0"},
        {"machine_prepare_compensation_time", "0"},
        {"extruder_offset", "0x0"},
        {"filament_map", "1"},
        {"initial_layer_print_height", "0.2"},
    };
}

static const char *PROG_BASIC = "G28\n"
                                "G90\n"
                                "M83\n"
                                "M104 S210\n"
                                "M140 S60\n"
                                "M190 S60\n"
                                "M109 S210\n"
                                "M106 S255\n"
                                "G1 Z0.2 F600\n"
                                "G1 X10 Y10 F12000\n"
                                "; FEATURE: Outer wall\n"
                                "; LINE_WIDTH: 0.42\n"
                                "; LAYER_HEIGHT: 0.2\n"
                                "G1 X60 Y10 E2.5 F3000\n"
                                "G1 X60 Y60 E2.5\n"
                                "G1 X10 Y60 E2.5 F1800\n"
                                "G1 X10 Y10 E2.5\n"
                                "G1 E-0.8 F1800\n"
                                "G1 X30 Y30 F12000\n"
                                "G1 E0.8 F1800\n"
                                "; FEATURE: Sparse infill\n"
                                "G1 X40 Y30 E0.5 F6000\n"
                                "G1 X40 Y40 E0.5\n"
                                "G1 X30.5 Y40.25 E0.48\n"
                                "M107\n"
                                "G4 P500\n"
                                "G4 S1\n"
                                "G1 Z0.4 F600\n"
                                "G1 X50 Y50 F9000\n"
                                "G1 X51 Y50 E0.05 F300\n";

static const char *PROG_ACCEL = "G28\n"
                                "M83\n"
                                "M204 S5000\n"
                                "G1 Z0.2 F600\n"
                                "G1 X10 Y10 F12000\n"
                                "G1 X80 Y10 E3 F6000\n"
                                "M204 P2000 T4000\n"
                                "G1 X80 Y80 E3\n"
                                "M204 R1000\n"
                                "G1 E-1 F2400\n"
                                "G1 X10 Y80 F15000\n"
                                "G1 E1 F2400\n"
                                "M201 X1000 Y1500 Z100 E2000\n"
                                "G1 X10 Y10 E3 F6000\n"
                                "M203 X100 Y120 Z10 E20\n"
                                "G1 X50 Y10 E1.5 F9000\n"
                                "M205 X5 Y6 Z0.5 E1.5\n"
                                "G1 X50 Y50 E1.5 F9000\n"
                                "M221 S90\n"
                                "G1 X10 Y50 E1.5 F9000\n"
                                "M221 S100\n"
                                "G92 E0\n"
                                "M82\n"
                                "G1 X10 Y20 E1 F3000\n"
                                "G1 X20 Y20 E2\n"
                                "G91\n"
                                "G1 X5 Y5 E0.5\n"
                                "G90\n"
                                "G1 X0 Y0 F12000\n";

static const char *PROG_ARCS = "G28\n"
                               "M83\n"
                               "G1 Z0.3 F600\n"
                               "G1 X20 Y20 F12000\n"
                               "; FEATURE: Inner wall\n"
                               "G2 X40 Y20 I10 J0 E1.2 F3000\n"
                               "G3 X20 Y20 I-10 J0 E1.2\n"
                               "G2 X30 Y30 R10 E0.8\n"
                               "G3 X20 Y20 R-10 E0.8\n"
                               "G2 X20 Y20 I5 J5 E2\n"
                               "G1 X25 Y25 F12000\n";

static const char *PROG_LAYERS = "G28\n"
                                 "M83\n"
                                 "; FEATURE: Custom\n"
                                 "G1 Z5 F600\n"
                                 "G1 X100 Y100 F12000\n"
                                 "; MACHINE_START_GCODE_END\n"
                                 "; CHANGE_LAYER\n"
                                 "; Z_HEIGHT: 0.2\n"
                                 "; LAYER_HEIGHT: 0.2\n"
                                 "G1 Z0.2 F600\n"
                                 "; FEATURE: Outer wall\n"
                                 "; LINE_WIDTH: 0.45\n"
                                 "G1 X120 Y100 E1 F3000\n"
                                 "G1 X120 Y120 E1\n"
                                 "; CHANGE_LAYER\n"
                                 "; Z_HEIGHT: 0.4\n"
                                 "; LAYER_HEIGHT: 0.2\n"
                                 "G1 Z0.4 F600\n"
                                 "G1 X100 Y120 E1 F3000\n"
                                 "; WIPE_START\n"
                                 "G1 X102 Y120 E-0.2 F6000\n"
                                 "; WIPE_END\n"
                                 "; CHANGE_LAYER\n"
                                 "; Z_HEIGHT: 0.6\n"
                                 "; LAYER_HEIGHT: 0.2\n"
                                 "G1 Z0.6 F600\n"
                                 "; FEATURE: Top surface\n"
                                 "G1 X100 Y100 E1 F3000\n"
                                 "; MACHINE_END_GCODE_START\n"
                                 "G1 Z10 F600\n";

static const char *PROG_VIRTUAL = "G28\n"
                                  "M83\n"
                                  "G1 Z0.2 F600\n"
                                  "G1 X10 Y10 F12000\n"
                                  ";VM104 S180\n"
                                  "G1 X20 Y10 E1 F3000\n"
                                  ";VM109 S190\n"
                                  ";vm104 S170\n"
                                  "G1 X20 Y20 E1\n"
                                  ";VT0 H-1\n"
                                  "G1 X10 Y20 E1\n";

static const char *PROG_UNITS = "G28\n"
                                "G21\n"
                                "M83\n"
                                "G1 Z0.2 F600\n"
                                "G1 X10 Y10 F12000\n"
                                "G20\n"
                                "G1 X1 Y1 F300\n"
                                "G1 X2 Y1 E0.04 F60\n"
                                "G21\n"
                                "G1 X0 Y0 F12000\n"
                                "G10\n"
                                "G11\n"
                                "G22\n"
                                "G23\n"
                                "M400\n"
                                "M400 S1\n"
                                "M400 P250\n"
                                "G29\n"
                                "M1\n";

static const char *PROG_KLIPPER = "G28\n"
                                  "M83\n"
                                  "G1 Z0.2 F600\n"
                                  "G1 X10 Y10 F12000\n"
                                  "SET_VELOCITY_LIMIT ACCEL=3000 ACCEL_TO_DECEL=1500 SQUARE_CORNER_VELOCITY=5\n"
                                  "G1 X60 Y10 E2 F6000\n"
                                  "SET_VELOCITY_LIMIT VELOCITY=200\n"
                                  "G1 X60 Y60 E2\n"
                                  "M204 S4000\n"
                                  "G1 X10 Y60 E2\n";

static const char *PROG_FANS = "G28\n"
                               "M83\n"
                               "G1 Z0.2 F600\n"
                               "M106 S128\n"
                               "G1 X10 Y10 E0.5 F3000\n"
                               "M106 P1 S64\n"
                               "G1 X20 Y10 E0.5\n"
                               "M106 P2 S200\n"
                               "G1 X20 Y20 E0.5\n"
                               "M106 P10\n"
                               "G1 X10 Y20 E0.5\n"
                               "M107 P2\n"
                               "G1 X10 Y10 E0.5\n"
                               "M106 P3 S10\n"
                               "M107 P3\n"
                               "G1 X15 Y15 E0.5\n"
                               "M107\n"
                               "G1 X16 Y15 E0.5\n";

static const char *PROG_DWELL = "G28\n"
                                "M83\n"
                                "G1 Z0.2 F600\n"
                                "G1 X10 Y10 E0.5 F3000\n"
                                "M400 S1 P250\n"
                                "G1 X20 Y10 E0.5\n"
                                "M400 P125\n"
                                "G1 X20 Y20 E0.5\n"
                                "M400 U1\n"
                                "M400 S0.3\n"
                                "G4 P333\n"
                                "G1 X10 Y20 E0.5\n";

static const char *PROG_REPETIER = "G28\n"
                                   "M83\n"
                                   "G1 Z0.2 F600\n"
                                   "G1 X10 Y10 F6000\n"
                                   "M401\n"
                                   "G1 X50 Y40 E1 F3000\n"
                                   "M402\n"
                                   "G1 X60 Y40 E0.5\n"
                                   "M401\n"
                                   "G1 X70 Y70 F9000\n"
                                   "M402 X F1200\n"
                                   "G1 X80 Y70 E0.5\n"
                                   "M203 X50 Y50\n"
                                   "G1 X10 Y10 E2\n";

static const char *PROG_TOOLS = "G28\n"
                                "M83\n"
                                "G1 Z0.2 F600\n"
                                "M108 T0\n"
                                "M135 T0\n"
                                "G1 X10 Y10 E0.5 F3000\n"
                                "M108 T9\n"
                                "M135 T0 ; tool\n"
                                "G1 X20 Y10 E0.5\n";

static const char *PROG_LIMITS = "G28\n"
                                 "M83\n"
                                 "G1 Z0.2 F600\n"
                                 "G1 X10 Y10 F12000\n"
                                 "M201 X800 Y900 Z50 E1500\n"
                                 "G1 X80 Y10 E3 F9000\n"
                                 "M203 X150 Y160 Z8 E25\n"
                                 "G1 X80 Y80 E3\n"
                                 "M205 X4 Y5 Z0.3 E1.2 S5 T7\n"
                                 "G1 X10 Y80 E3\n"
                                 "M566 X600 Y480 Z12 E120\n"
                                 "G1 X10 Y10 E3\n"
                                 "M204 S3000 T800\n"
                                 "G1 X40 Y40 F15000\n"
                                 "G1 E-1 F2400\n";

static const char *PROG_VG1 = "G28\n"
                              "M83\n"
                              "G1 Z0.2 F600\n"
                              "G1 X10 Y10 F12000\n"
                              "; FEATURE: Outer wall\n"
                              ";VG1 X40 Y10 E1.5 F3000\n"
                              ";VG1 X40 Y40 E1.5\n"
                              "G1 X10 Y40 E1.5 F1800\n"
                              ";VG1 X12 Y38 E0.1 F600\n"
                              ";VG1 E-0.8 F1800\n"
                              ";VG1 X30 Y30 F12000\n"
                              "; VFLUSH_START\n"
                              ";VG1 E2 F300\n"
                              ";VG1 E1.5\n"
                              "; VFLUSH_END\n"
                              ";vg1 X31 Y30 E0.05 F600\n"
                              "; FLUSH_START\n"
                              "G1 E3 F300\n"
                              "; FLUSH_END\n"
                              ";VG1 X60 Y30 E2 F9000\n"
                              ";VG1 X60 Y31 E0.04 F9000\n"
                              ";VG1 X20 Y31 E2.5 F9000\n"
                              ";VG1 Z0.6 F600\n"
                              ";VG1 X20 Y60 E1 F6000\n";

// GCodeProcessor.cpp:2038-2040, 6596-6610. Keep the rows deliberately out
// of extruder order to distinguish an exact match, an owner's fallback, and 0.
static void machine_rows()
{
    using namespace MultiNozzleUtils;
    PrintConfig config;
    config.printer_extruder_id.values = {2, 1, 1, 2};
    config.printer_extruder_variant.values = {"Bowden Standard", "Direct Drive Standard",
                                               "Direct Drive High Flow", "Bowden High Flow"};
    config.extruder_type.values = {etDirectDrive, etBowden};
    const std::vector<NozzleInfo> nozzles = {
        {"0.4", nvtStandard, 0, 0}, {"0.4", nvtHighFlow, 0, 1},
        {"0.4", nvtHighFlow, 1, 2}, {"0.4", nvtTPUHighFlow, 0, 3}
    };
    auto group = LayeredNozzleGroupResult::create(std::vector<int>{0, 1, 2, 3}, nozzles, {0, 1, 2, 3});
    if (!group) std::abort();
    for (int grouped : {0, 1})
        for (int extruder : {-1, 0, 1, 2})
            for (int nozzle : {-1, 0, 1, 2, 3, 4}) {
                GCodeProcessor p;
                p.apply_config(config);
                p.m_nozzle_group_result = grouped ? std::make_shared<LayeredNozzleGroupResult>(*group) : nullptr;
                p.m_extruder_id = static_cast<unsigned char>(extruder);
                if (nozzle >= 0)
                    p.m_nozzle_status_recorder.set_nozzle_status(nozzle, 0, p.get_extruder_id());
                for (int filament : {-1, 0, 3})
                    std::printf("%d %d %d %d %d\n", grouped, extruder, nozzle, filament,
                                p.get_machine_config_idx(filament));
            }
}

// PrintConfig.hpp:1312-1342 and GCodeProcessor.cpp:2030-2036, 6264-6303.
// Exercise both modes of three nozzle rows. A short option repeats its last
// element, an empty one has no limit, and a nullable slot stays nil.
static void machine_axis_limits()
{
    using Mode = PrintEstimatedStatistics::ETimeMode;
    for (const char *kind : {"speed", "acceleration", "jerk"})
        for (int axis = X; axis <= E; ++axis)
            for (const char *values : {"", "38.125", "500,200", "42,11,24,7", "nil,123.75,60,nil"}) {
                const std::string key = std::string("machine_max_") + kind + "_" + "xyze"[axis];
                PrintConfig cfg;
                cfg.gcode_flavor.value = gcfMarlinFirmware;
                cfg.set_deserialize_strict(key, values);
                GCodeProcessor p;
                p.apply_config(cfg);
                std::printf("rows %s %s", key.c_str(), *values ? values : "-");
                for (int row = 0; row < 3; ++row)
                    for (Mode mode : {Mode::Normal, Mode::Stealth}) {
                        const float v = std::string(kind) == "speed" ? p.get_axis_max_feedrate(mode, Axis(axis), row)
                                      : std::string(kind) == "acceleration" ? p.get_axis_max_acceleration(mode, Axis(axis), row)
                                      : p.get_axis_max_jerk(mode, Axis(axis));
                        std::printf(" %s", std::isnan(v) ? "nil" : f32(v).c_str());
                    }
                std::putchar('\n');
            }
    for (const char *speeds : {"500,500", "500,200", "500,20"}) {
        PrintConfig cfg;
        cfg.gcode_flavor.value = gcfMarlinLegacy;
        cfg.set_deserialize_strict("machine_max_speed_x", speeds);
        cfg.set_deserialize_strict("machine_max_acceleration_x", "9000,9000");
        cfg.set_deserialize_strict("machine_max_acceleration_travel", "9000,9000");
        cfg.set_deserialize_strict("machine_max_acceleration_extruding", "9000,9000");
        cfg.set_deserialize_strict("machine_max_jerk_x", "9,9");
        cfg.set_deserialize_strict("machine_min_travel_rate", "0,0");
        GCodeProcessor p;
        p.reset();
        p.apply_config(cfg);
        p.enable_stealth_time_estimator(true);
        p.initialize("axis.gcode");
        p.process_buffer("G90\nG1 X1000 F30000\nG1 X0 F30000\n");
        p.finalize(false);
        std::printf("time %s %s %s\n", speeds, f32(p.get_time(Mode::Normal)).c_str(), f32(p.get_time(Mode::Stealth)).c_str());
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: gen <out dir>\n");
        return 1;
    }
    if (std::string(argv[1]) == "--axis-limits") {
        machine_axis_limits();
        return 0;
    }
    if (std::string(argv[1]) == "--machine-rows") {
        machine_rows();
        return 0;
    }
    std::string dir = argv[1];
    g_out           = fopen((dir + "/g_code_processor.txt").c_str(), "wb");
    helpers();
    producers();

    std::vector<Case> cases;
    for (const char *flavor : {"marlin2", "marlin", "klipper", "reprapfirmware"}) {
        for (bool stealth : {false, true}) {
            std::string fl(flavor);
            cases.push_back({"basic_" + fl + (stealth ? "_s" : ""), limits(fl), stealth, PROG_BASIC});
        }
        std::string fl(flavor);
        cases.push_back({"accel_" + fl, limits(fl), false, PROG_ACCEL});
        cases.push_back({"arcs_" + fl, limits(fl), false, PROG_ARCS});
        cases.push_back({"layers_" + fl, limits(fl), false, PROG_LAYERS});
        cases.push_back({"virtual_" + fl, limits(fl), false, PROG_VIRTUAL});
        cases.push_back({"units_" + fl, limits(fl), false, PROG_UNITS});
    }
    cases.push_back({"klipper_svl", limits("klipper"), false, PROG_KLIPPER});
    for (const char *flavor : {"marlin2", "marlin", "klipper", "reprapfirmware", "reprap", "repetier", "sailfish", "makerware", "smoothie"}) {
        std::string fl(flavor);
        cases.push_back({"fans_" + fl, limits(fl), false, PROG_FANS});
        cases.push_back({"dwell_" + fl, limits(fl), true, PROG_DWELL});
        cases.push_back({"repetier_" + fl, limits(fl), false, PROG_REPETIER});
        cases.push_back({"tools_" + fl, limits(fl), false, PROG_TOOLS});
        cases.push_back({"limits_" + fl, limits(fl), true, PROG_LIMITS, false});
        cases.push_back({"limits_env_" + fl, limits(fl), true, PROG_LIMITS, true});
        cases.push_back({"basic_env_" + fl, limits(fl), true, PROG_BASIC, true});
        cases.push_back({"vg1_" + fl, limits(fl), true, PROG_VG1, false});
    }
    {
        auto cfg = limits("marlin2");
        for (auto &kv : cfg) {
            if (kv.first == "machine_max_acceleration_travel") kv.second = "0,0";
            if (kv.first == "machine_max_acceleration_extruding") kv.second = "0,0";
            if (kv.first == "machine_max_acceleration_retracting") kv.second = "0,0";
        }
        cases.push_back({"basic_noaccel", cfg, false, PROG_BASIC});
    }
    for (const Case &c : cases)
        run_case(c, dir);
    fclose(g_out);
    return 0;
}
