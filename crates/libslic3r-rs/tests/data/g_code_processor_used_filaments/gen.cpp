// Generator of the vectors of tests/g_code_processor_helpers_native_parity.rs: the file-static helpers
// of the pin's GCodeProcessor.cpp (included): get_object_label_id (152-165), get_z_height (167-177),
// skip_whitespaces (2649-2652) and remove_eols (2654-2657), over fixed and LCG-made comment strings.
//
// Build it with target/parity-tools/build.sh, run it with the output directory, then
//     gzip -9 -n -c <out>/g_code_processor_helpers.txt > tests/data/g_code_processor_helpers/g_code_processor_helpers.txt.gz
//
// Output lines (strings as the hex of their bytes, `-` when empty; f32 as `%08x` of its bits):
//   lbl <hex comment> <get_object_label_id>
//   zh <hex comment> <get_z_height f32>
//   sw <hex text> <skip_whitespaces offset from begin> <remove_eols offset of end from begin>

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

// Native vectors for GCodeProcessor::UsedFilaments (GCodeProcessor.cpp:1380-1534): reset, the three
// increase_*_caches, process_color_change_cache, process_model_cache, process_wipe_tower_cache,
// process_support_cache, process_total_volume_cache, update_flush_per_filament, process_role_cache and
// process_caches, driven on a real GCodeProcessor (m_used_filaments) by LCG sequences. After every
// operation the whole UsedFilaments state is printed with doubles as their 64-bit patterns.
// usage: gen <out dir>

using namespace Slic3r;

static FILE *g_out = nullptr;

static uint64_t g_seed = 0x2545F4914F6CDD1DULL;
static uint32_t lcg()
{
    g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL;
    return uint32_t(g_seed >> 33);
}

static std::string d64(double v)
{
    uint64_t b;
    std::memcpy(&b, &v, 8);
    char buf[24];
    snprintf(buf, sizeof(buf), "%016llx", (unsigned long long) b);
    return buf;
}

static std::string f32h(float v)
{
    uint32_t b;
    std::memcpy(&b, &v, 4);
    char buf[16];
    snprintf(buf, sizeof(buf), "%08x", b);
    return buf;
}

static double rnd_volume()
{
    switch (lcg() % 6) {
    case 0: return 0.0;
    case 1: return double(lcg() % 1000) / 100.;
    case 2: return -double(lcg() % 1000) / 1000.;
    default: {
        // full-mantissa positive double in [0, 64)
        uint64_t m = (uint64_t(lcg()) << 32) | lcg();
        return double(m >> 11) * (1.0 / 9007199254740992.0) * 64.0;
    }
    }
}

static float rnd_float_volume()
{
    switch (lcg() % 4) {
    case 0: return 0.f;
    case 1: return float(lcg() % 1000) / 10.f;
    default: return float(double(lcg()) / 4294967296.0 * 300.0);
    }
}

static void dump_map(std::string &o, const char *name, const std::map<size_t, double> &m)
{
    o += " ";
    o += name;
    o += "=";
    if (m.empty()) o += "-";
    bool first = true;
    for (const auto &kv : m) {
        if (!first) o += ",";
        first = false;
        o += std::to_string(kv.first) + ":" + d64(kv.second);
    }
}

static void dump(GCodeProcessor &p)
{
    const auto &u = p.m_used_filaments;
    std::string o = "S";
    o += " cc=" + d64(u.color_change_cache) + " mc=" + d64(u.model_extrude_cache) + " wc=" + d64(u.wipe_tower_cache) +
         " sc=" + d64(u.support_volume_cache) + " tc=" + d64(u.total_volume_cache) + " rc=" + d64(u.role_cache);
    o += " vpcc=";
    if (u.volumes_per_color_change.empty()) o += "-";
    for (size_t i = 0; i < u.volumes_per_color_change.size(); ++i) {
        if (i) o += ",";
        o += d64(u.volumes_per_color_change[i]);
    }
    dump_map(o, "mv", u.model_volumes_per_filament);
    dump_map(o, "wv", u.wipe_tower_volumes_per_filament);
    dump_map(o, "sv", u.support_volumes_per_filament);
    dump_map(o, "fl", u.flush_per_filament);
    dump_map(o, "tv", u.total_volumes_per_filament);
    o += " fr=";
    if (u.filaments_per_role.empty()) o += "-";
    bool first = true;
    for (const auto &kv : u.filaments_per_role) {
        if (!first) o += ",";
        first = false;
        o += std::to_string(int(kv.first)) + ":" + d64(kv.second.first) + ":" + d64(kv.second.second);
    }
    fprintf(g_out, "%s\n", o.c_str());
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: gen <out dir>\n");
        return 1;
    }
    g_out = fopen((std::string(argv[1]) + "/g_code_processor_used_filaments.txt").c_str(), "wb");
    const float diameters[] = {1.75f, 2.85f, 1.7f, 3.0f, 1.8f, 2.9f, 1.75f, 2.2f};
    const float densities[] = {1.24f, 1.27f, 1.04f, 1.25f, 1.31f, 0.99f, 1.2f, 1.26f};
    for (int c = 0; c < 400; ++c) {
        GCodeProcessor p;
        p.m_used_filaments.reset();
        const size_t n_fil = 8;
        p.m_result.filament_diameters.assign(n_fil, 1.75f);
        p.m_result.filament_densities.assign(n_fil, 1.24f);
        for (size_t i = 0; i < n_fil; ++i) {
            if (lcg() % 3 == 0)
                p.m_result.filament_diameters[i] = float(1.5 + double(lcg()) / 4294967296.0 * 1.5);
            else
                p.m_result.filament_diameters[i] = diameters[lcg() % 8];
            p.m_result.filament_densities[i] = (lcg() % 3 == 0) ? float(0.9 + double(lcg()) / 4294967296.0 * 0.6) : densities[lcg() % 8];
        }
        p.m_extruder_id = (unsigned char) (-1);
        p.m_filament_id.assign(1 + lcg() % 2, (unsigned char) (-1));
        p.m_extrusion_role = erNone;
        std::string fils = "F";
        for (size_t i = 0; i < n_fil; ++i) fils += " " + f32h(p.m_result.filament_diameters[i]) + ":" + f32h(p.m_result.filament_densities[i]);
        fprintf(g_out, "C %d %zu\n%s\n", c, p.m_filament_id.size(), fils.c_str());
        int n_ops = 5 + lcg() % 40;
        for (int k = 0; k < n_ops; ++k) {
            int op = lcg() % 14;
            // the inputs the processor hands over: get_filament_id() (forced) and get_filament_id(false)
            int fid  = p.get_filament_id();
            int fid0 = p.get_filament_id(false);
            switch (op) {
            case 0: { double v = rnd_volume(); p.m_used_filaments.increase_model_caches(v); fprintf(g_out, "O %d %s\n", op, d64(v).c_str()); break; }
            case 1: { double v = rnd_volume(); p.m_used_filaments.increase_support_caches(v); fprintf(g_out, "O %d %s\n", op, d64(v).c_str()); break; }
            case 2: { double v = rnd_volume(); p.m_used_filaments.increase_wipe_tower_caches(v); fprintf(g_out, "O %d %s\n", op, d64(v).c_str()); break; }
            case 3: p.m_used_filaments.process_color_change_cache(); fprintf(g_out, "O %d\n", op); break;
            case 4: p.m_used_filaments.process_model_cache(&p); fprintf(g_out, "O %d %d\n", op, fid); break;
            case 5: p.m_used_filaments.process_wipe_tower_cache(&p); fprintf(g_out, "O %d %d\n", op, fid); break;
            case 6: p.m_used_filaments.process_support_cache(&p); fprintf(g_out, "O %d %d\n", op, fid0); break;
            case 7: p.m_used_filaments.process_total_volume_cache(&p); fprintf(g_out, "O %d %d\n", op, fid); break;
            case 8: { size_t id = lcg() % n_fil; float v = rnd_float_volume(); p.m_used_filaments.update_flush_per_filament(id, v); fprintf(g_out, "O %d %zu %s\n", op, id, f32h(v).c_str()); break; }
            case 9: p.m_used_filaments.process_role_cache(&p); fprintf(g_out, "O %d %d %d\n", op, fid, int(p.m_extrusion_role)); break;
            case 10: p.m_used_filaments.process_caches(&p); fprintf(g_out, "O %d %d %d %d\n", op, fid, fid0, int(p.m_extrusion_role)); break;
            case 11: {
                // select an extruder / filament (or leave them unset)
                unsigned r = lcg() % 6;
                if (r == 0) p.m_extruder_id = (unsigned char) (-1);
                else {
                    p.m_extruder_id = (unsigned char) (lcg() % p.m_filament_id.size());
                    p.m_filament_id[p.m_extruder_id] = (r == 1) ? (unsigned char) (-1) : (unsigned char) (lcg() % n_fil);
                }
                fprintf(g_out, "O %d\n", op);
                break;
            }
            case 12: p.m_extrusion_role = ExtrusionRole(lcg() % erCount); fprintf(g_out, "O %d %d\n", op, int(p.m_extrusion_role)); break;
            default: p.m_used_filaments.reset(); fprintf(g_out, "O %d\n", op); break;
            }
            dump(p);
        }
    }
    fclose(g_out);

    // GCodeProcessor::get_time_dhm(mode) (GCodeProcessor.cpp:2898-2901): short_time(get_time_dhms(time)) of the
    // mode's machine; times as float bits (zero, sub-second, minutes, hours, days, rounding edges, random).
    FILE *t_out = fopen((std::string(argv[1]) + "/g_code_processor_time_dhm.txt").c_str(), "wb");
    std::vector<float> times = {0.f, 0.4f, 0.5f, 0.6f, 1.f, 59.f, 59.4f, 59.5f, 59.6f, 60.f, 61.f, 3599.f, 3599.5f, 3600.f,
                                3661.f, 86399.f, 86399.6f, 86400.f, 90061.f, 499681.3f, 499352.f, 1e7f, -1.f, -60.f};
    for (int i = 0; i < 300; ++i) {
        uint32_t k = lcg() % 3;
        float    v = k == 0 ? float(lcg() % 200000) : (k == 1 ? float(double(lcg()) / 4294967296.0 * 1e6) : float(lcg() % 4000) + 0.5f);
        times.push_back(v);
    }
    for (float v : times)
        for (int mode = 0; mode < 2; ++mode) {
            GCodeProcessor p;
            p.m_time_processor.machines[mode].time = v;
            std::string s = p.get_time_dhm(PrintEstimatedStatistics::ETimeMode(mode));
            fprintf(t_out, "T %d %s %s\n", mode, f32h(v).c_str(), s.c_str());
        }
    fclose(t_out);
    return 0;
}
