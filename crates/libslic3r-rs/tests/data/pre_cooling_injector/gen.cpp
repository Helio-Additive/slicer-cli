// Generator for tests/pre_cooling_injector_native_parity.rs: the pin's
// GCodeProcessor::TimeProcessor::post_process (GCodeProcessor.cpp:571-1300) with pre-heating on,
// that is the first pass's block collection (T / NOZZLE_CHANGE_START,END / CP_TOOLCHANGE_WIPE /
// CHANGE_LAYER / SKIPPABLE / MACHINE_START_GCODE_END, MACHINE_END_GCODE_START), the moves' time fill,
// the remaining filament changes and the PreCoolingInjector (GCodeProcessor.cpp:6611-7202), on
// generated two-extruder G-code with generated move times (LCG, fixed seeds).
// Usage: gen <vectors out> <scratch dir>
// Output per scenario:
//   S <id> <filaments> <handle_hotend_as_extruder>
//   fmap <filament -> nozzle>...      nozzle k is extruder k, group k
//   nt <nozzle_temp>...  pct <pre_cooling_temp>...  pdelta <%a>...  minec <%a>...
//   ftypes <type>...  phe <0|1>...  cool <%a> <%a>  rack <%a> <%a>  heat <%a> <%a>
//   gcode <hex>
//   moves <n> (<gcode_id> <%a time0> <%a time1>)...
//   out <hex>                         the file after post_process
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

static uint32_t g_lcg = 1;
static uint32_t lcg() { g_lcg = g_lcg * 1664525u + 1013904223u; return g_lcg >> 8; }

static std::string hex(const std::string &s)
{
    static const char *d = "0123456789abcdef";
    std::string out;
    for (unsigned char c : s) { out += d[c >> 4]; out += d[c & 15]; }
    return out.empty() ? "-" : out;
}

int main(int argc, char **argv)
{
    if (argc < 3) return 1;
    FILE *out = fopen(argv[1], "w");
    std::string dir = argv[2];
    for (int sc = 0; sc < 48; ++sc) {
        g_lcg = 1234u + 7919u * unsigned(sc);
        const int  F      = 2 + int(lcg() % 5);
        const bool hotend = (sc % 4) == 3;
        std::vector<MultiNozzleUtils::NozzleInfo> nozzles(2);
        for (int k = 0; k < 2; ++k) { nozzles[k].diameter = "0.4"; nozzles[k].volume_type = nvtStandard; nozzles[k].extruder_id = k; nozzles[k].group_id = k; }
        std::vector<int> fmap(F);
        for (int f = 0; f < F; ++f) fmap[f] = int(lcg() % 2);
        std::vector<unsigned int> used(F);
        for (int f = 0; f < F; ++f) used[f] = unsigned(f);
        auto ngr = MultiNozzleUtils::LayeredNozzleGroupResult::create(fmap, nozzles, used);
        if (!ngr) { fprintf(out, "S %d nogroup\n", sc); continue; }

        GCodeProcessor::FilamentVariantRows rows;
        std::vector<std::string> ftypes(F);
        std::vector<unsigned char> phe(F);
        std::vector<double> minec(F);
        for (int f = 0; f < F; ++f) {
            int t = 190 + int(lcg() % 70);
            rows.nozzle_temp.push_back(t);
            rows.pre_cooling_temp.push_back((lcg() % 3) == 0 ? 0 : t - 10 - int(lcg() % 60));
            rows.preheat_temperature_delta.push_back(double(lcg() % 25));
            rows.default_row.push_back(f);
            ftypes[f] = (lcg() % 5) == 0 ? "PVA" : "PLA";
            phe[f] = (lcg() % 6) == 0 ? 0 : 1;
            minec[f] = (lcg() % 3) == 0 ? -1.0 : double(40 + lcg() % 120);
        }
        std::vector<double> cool = {2.0 + (lcg() % 10) * 0.25, 1.5 + (lcg() % 10) * 0.25};
        std::vector<double> rack = {1.0 + (lcg() % 10) * 0.25, 2.0 + (lcg() % 10) * 0.25};
        std::vector<double> heat = {3.6, 2.5 + (lcg() % 10) * 0.3};

        // the G-code and its moves
        std::string g;
        unsigned int line = 0;
        std::vector<GCodeProcessorResult::MoveVertex> moves;
        float t = 0.f;
        auto add = [&](const std::string &s) { g += s + "\n"; return ++line; };
        auto mv = [&](bool zero_time) {
            unsigned int id = add("G1 X" + std::to_string(lcg() % 200) + " Y" + std::to_string(lcg() % 200) + " E0.1");
            GCodeProcessorResult::MoveVertex m;
            m.gcode_id = id;
            if (zero_time) { m.time[0] = 0.f; m.time[1] = 0.f; }
            else { t += float(lcg() % 4000) / 100.f + 0.25f; m.time[0] = t; m.time[1] = t * 1.5f; }
            moves.push_back(m);
        };
        add("; HEADER_BLOCK_START");
        mv(false);
        add("T0 H-1");
        mv(false);
        add("; MACHINE_START_GCODE_END");
        int cur = int(lcg() % F);
        int layer = 0;
        add("T" + std::to_string(cur) + " H-1");
        const int changes = 6 + int(lcg() % 14);
        for (int c = 0; c < changes; ++c) {
            int nmoves = 1 + int(lcg() % 6);
            for (int k = 0; k < nmoves; ++k) mv((lcg() % 7) == 0);
            if ((lcg() % 3) == 0) { add("; CHANGE_LAYER"); ++layer; mv(false); }
            int next = int(lcg() % F);
            if (next == cur) next = (cur + 1) % F;
            bool skippable = (lcg() % 4) == 0;
            if (skippable) add("; SKIPPABLE_START");
            if (hotend && (lcg() % 2) == 0) add("; FilamentChangeGCodeStart");
            if (fmap[cur] != fmap[next]) {
                std::string tail = " OF" + std::to_string(cur) + " NF" + std::to_string(next) + " ON" + std::to_string(fmap[cur]) + " NN" + std::to_string(fmap[next]);
                add("; NOZZLE_CHANGE_START" + tail);
                int r = 1 + int(lcg() % 3);
                for (int k = 0; k < r; ++k) mv(false);
                add("; NOZZLE_CHANGE_END" + tail);
            }
            if ((lcg() % 3) == 0) add("; CP_TOOLCHANGE_WIPE CT" + std::to_string(lcg() % 2) + ((lcg() % 2) ? " FL1" : " FL0"));
            add("T" + std::to_string(next) + ((lcg() % 4) == 0 ? " H" + std::to_string(fmap[next]) : std::string(" H-1")));
            if (hotend) add("; FilamentChangeGCodeEnd");
            if (skippable) { mv(false); add("; SKIPPABLE_END"); }
            cur = next;
        }
        mv(false);
        add("; MACHINE_END_GCODE_START");
        mv(false);
        add("M400");

        GCodeProcessor::UsedFilaments uf;
        std::vector<Extruder> filament_lists;
        std::vector<int> physical_extruder_map = {1, 0};
        std::vector<int> max_nozzle = {1, 1};
        std::vector<ExtruderType> ext_types = {etDirectDrive, etDirectDrive};
        std::vector<double> nozzle_diameter = {0.4, 0.4};
        MultiNozzleUtils::LayeredNozzleGroupResult group = *ngr;
        GCodeProcessor::TimeProcessContext ctx(uf, filament_lists, group, ftypes, phe, physical_extruder_map, size_t(layer), cool, rack, heat,
                                               minec, 0.f, true, hotend, false, max_nozzle, ext_types, nozzle_diameter, rows);

        std::string path = dir + "/pci_" + std::to_string(sc) + ".gcode";
        { std::ofstream f(path, std::ios::binary); f << g; }
        GCodeProcessor::TimeProcessor tp;
        tp.reset();
        std::vector<GCodeProcessorResult::MoveVertex> moves_copy = moves;
        std::vector<size_t> lines_ends;
        tp.post_process(path, moves_copy, lines_ends, ctx);
        std::string res;
        { std::ifstream f(path, std::ios::binary); std::stringstream ss; ss << f.rdbuf(); res = ss.str(); }
        boost::nowide::remove(path.c_str());

        fprintf(out, "S %d %d %d\n", sc, F, int(hotend));
        fprintf(out, "fmap"); for (int v : fmap) fprintf(out, " %d", v); fprintf(out, "\n");
        fprintf(out, "nt"); for (int v : rows.nozzle_temp) fprintf(out, " %d", v); fprintf(out, "\n");
        fprintf(out, "pct"); for (int v : rows.pre_cooling_temp) fprintf(out, " %d", v); fprintf(out, "\n");
        fprintf(out, "pdelta"); for (double v : rows.preheat_temperature_delta) fprintf(out, " %a", v); fprintf(out, "\n");
        fprintf(out, "minec"); for (double v : minec) fprintf(out, " %a", v); fprintf(out, "\n");
        fprintf(out, "ftypes"); for (auto &v : ftypes) fprintf(out, " %s", v.c_str()); fprintf(out, "\n");
        fprintf(out, "phe"); for (auto v : phe) fprintf(out, " %d", int(v)); fprintf(out, "\n");
        fprintf(out, "cool %a %a\nrack %a %a\nheat %a %a\n", cool[0], cool[1], rack[0], rack[1], heat[0], heat[1]);
        fprintf(out, "gcode %s\n", hex(g).c_str());
        fprintf(out, "moves %zu", moves.size());
        for (auto &m : moves) fprintf(out, " %u %a %a", m.gcode_id, double(m.time[0]), double(m.time[1]));
        fprintf(out, "\n");
        fprintf(out, "out %s\n", hex(res).c_str());
    }
    fclose(out);
    return 0;
}
