// Vectors for tests/ffc_interpolate_width_native_parity.rs: the pin's interpolate_width
// (override layer libslic3r/bambustudio/libslic3r/Fill/FillFloatingConcentric.cpp:220-258)
// on LCG inputs.
// Usage: gen <cases>   (writes to stdout)
// One line per case:
//   C <n> <m> <subject_idx_range> <default_width> <idx> <x y>*n <width bits>*(2n-2) <x y z>*m <result bits>
// Doubles are the 16 hex digits of their bits.
#include "libslic3r/Fill/FillFloatingConcentric.cpp"
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
using namespace Slic3r;
static uint64_t g_seed = 0x2545F4914F6CDD1DULL;
static uint32_t lcg() { g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL; return uint32_t(g_seed >> 33); }
static long long rnd(long long lo, long long hi) { return lo + (long long)(lcg() % uint32_t(hi - lo + 1)); }
static unsigned long long bits(double d) { unsigned long long u; std::memcpy(&u, &d, 8); return u; }
int main(int argc, char **argv)
{
    int cases = argc > 1 ? atoi(argv[1]) : 1000;
    for (int c = 0; c < cases; ++c) {
        int n = int(rnd(2, 9));
        int m = int(rnd(1, 12));
        int sir = n;
        int dw = int(rnd(20000, 60000));
        size_t idx = size_t(rnd(0, m - 1));
        ThickPolyline line;
        for (int i = 0; i < n; ++i) line.points.emplace_back(coord_t(rnd(-2000000, 2000000)), coord_t(rnd(-2000000, 2000000)));
        for (int i = 0; i < 2 * n - 2; ++i) line.width.push_back(double(rnd(20000, 60000)) + double(lcg()) / 4294967296.0);
        ZPath path;
        long long x = rnd(-2000000, 2000000), y = rnd(-2000000, 2000000);
        for (int i = 0; i < m; ++i) {
            x += rnd(-300000, 300000); y += rnd(-300000, 300000);
            long long r = rnd(0, 9), z;
            if (r < 2) z = -1; else if (r < 3) z = sir + rnd(0, 5); else z = rnd(0, n - 1);
            path.emplace_back(ClipperLib_Z::cInt(x), ClipperLib_Z::cInt(y), ClipperLib_Z::cInt(z));
        }
        double res = interpolate_width(path, line, sir, dw, idx);
        printf("C %d %d %d %d %zu", n, m, sir, dw, idx);
        for (const Point &p : line.points) printf(" %lld %lld", (long long) p.x(), (long long) p.y());
        for (double w : line.width) printf(" %016llx", bits(w));
        for (const auto &p : path) printf(" %lld %lld %lld", (long long) p.x(), (long long) p.y(), (long long) p.z());
        printf(" %016llx\n", bits(res));
    }
    return 0;
}
