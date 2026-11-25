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
#undef protected

using namespace Slic3r;

static FILE *g_out = nullptr;

static std::string hex(const std::string &s)
{
    if (s.empty())
        return "-";
    static const char *d = "0123456789abcdef";
    std::string o;
    for (unsigned char c : s) {
        o += d[c >> 4];
        o += d[c & 15];
    }
    return o;
}

static std::string f32h(float v)
{
    uint32_t b;
    std::memcpy(&b, &v, 4);
    char buf[16];
    snprintf(buf, sizeof(buf), "%08x", b);
    return buf;
}

static uint64_t g_seed = 0x9E3779B97F4A7C15ULL;
static uint32_t lcg()
{
    g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL;
    return uint32_t(g_seed >> 33);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: gen <out dir>\n");
        return 1;
    }
    g_out = fopen((std::string(argv[1]) + "/g_code_processor_helpers.txt").c_str(), "wb");
    std::vector<std::string> fixed = {
        " OBJECT_ID: 12", " OBJECT_ID:12", "OBJECT_ID: -3", " OBJECT_ID: +7", " OBJECT_ID: 0012", " OBJECT_ID: ", " OBJECT_ID:",
        " OBJECT_ID: abc", " OBJECT_ID: 12abc", " OBJECT_ID: 99999999999", " OBJECT_ID: 2147483647", " OBJECT_ID: 2147483648",
        " OBJECT_ID: -2147483648", " OBJECT_ID: -2147483649", "12", "", ":", "::5", " OBJECT_ID: \t 42", " OBJECT_ID: 4 2",
        " Z_HEIGHT: 0.2", " Z_HEIGHT: 196.6", " Z_HEIGHT:1.2", " Z_HEIGHT: .4", " Z_HEIGHT: 1e-3", " Z_HEIGHT: 1E3x", " Z_HEIGHT: -0",
        " Z_HEIGHT: inf", " Z_HEIGHT: -INF", " Z_HEIGHT: nan", " Z_HEIGHT: 0x1p3", " Z_HEIGHT: 0x.8", " Z_HEIGHT: 1e40",
        " Z_HEIGHT: 1e-50", " Z_HEIGHT: ", " Z_HEIGHT: +", " Z_HEIGHT: 0.299995", " Z_HEIGHT: 3.4028236e38", " Z_HEIGHT: 12.5mm",
        " Z_HEIGHT: \t0.8", "0.8", " Z_HEIGHT: 0.30000001192092896", " Z_HEIGHT: 1.17549435e-38", " Z_HEIGHT: 7.006e-46",
        "  \t ; x\r\n", "\r\n", "\t\t", "G1 X1\n", "a\r", " ", "abc",
    };
    for (int i = 0; i < 400; ++i) {
        // LCG layer heights / ids as the G-code writer prints them, with random padding.
        char buf[64];
        int  k = lcg() % 4;
        if (k == 0)
            snprintf(buf, sizeof(buf), " Z_HEIGHT: %.*f", int(lcg() % 7), double(lcg() % 3000000) / 10000.);
        else if (k == 1)
            snprintf(buf, sizeof(buf), " Z_HEIGHT: %g", double(lcg()) / double(lcg() % 1000 + 1) * 1e-3);
        else if (k == 2)
            snprintf(buf, sizeof(buf), " OBJECT_ID: %d", int(lcg() % 20000) - 100);
        else
            snprintf(buf, sizeof(buf), "%s%s%u%s", (lcg() & 1) ? " " : "\t", (lcg() & 1) ? "LAYER:" : "", lcg() % 1000, (lcg() & 1) ? "\r\n" : " ");
        fixed.push_back(buf);
    }
    for (const std::string &s : fixed) {
        fprintf(g_out, "lbl %s %d\n", hex(s).c_str(), get_object_label_id(s));
        fprintf(g_out, "zh %s %s\n", hex(s).c_str(), f32h(get_z_height(s)).c_str());
        const char *b = s.data(), *e = s.data() + s.size();
        fprintf(g_out, "sw %s %d %d\n", hex(s).c_str(), int(skip_whitespaces(b, e) - b), int(remove_eols(b, e) - b));
    }
    fclose(g_out);
    return 0;
}
