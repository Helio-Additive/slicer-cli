// Vectors for merge_segmented_layers (MultiMaterialSegmentation.cpp:2039-2079): random per-layer side
// segmentations and top/bottom regions, the pin's function on one thread, its output with every point.
// Usage: gen_merge <out.txt>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_unordered_set.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/spin_mutex.h>

#include "libslic3r/MultiMaterialSegmentation.cpp"

using namespace Slic3r;

static uint64_t g_seed = 0x2545f4914f6cdd1dULL;
static uint32_t lcg()
{
    g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL;
    return uint32_t(g_seed >> 33);
}
static int rnd(int n) { return int(lcg() % uint32_t(n)); }

static Polygon random_polygon(double cx, double cy, double r0, bool ccw)
{
    Polygon poly;
    if (rnd(3) == 0) {
        // An axis-aligned rectangle (shared edges between regions make the Clipper cases with collinear edges).
        coord_t hx = coord_t(r0 * (0.5 + 0.5 * double(rnd(1000)) / 1000.)), hy = coord_t(r0 * (0.5 + 0.5 * double(rnd(1000)) / 1000.));
        coord_t x = coord_t(std::lrint(cx)) / 50000 * 50000, y = coord_t(std::lrint(cy)) / 50000 * 50000;
        hx = hx / 50000 * 50000 + 50000; hy = hy / 50000 * 50000 + 50000;
        poly.points = {Point(x - hx, y - hy), Point(x + hx, y - hy), Point(x + hx, y + hy), Point(x - hx, y + hy)};
    } else {
        const int n = 3 + rnd(9);
        for (int i = 0; i < n; ++i) {
            double a = 2. * M_PI * double(i) / double(n);
            double r = r0 * (0.6 + 0.4 * double(rnd(1000)) / 1000.);
            poly.points.emplace_back(coord_t(std::lrint(cx + r * std::cos(a))), coord_t(std::lrint(cy + r * std::sin(a))));
        }
    }
    if (!ccw) poly.reverse();
    return poly;
}

static ExPolygons random_expolygons()
{
    ExPolygons out;
    const int k = rnd(4) == 0 ? 0 : 1 + rnd(3);
    for (int i = 0; i < k; ++i) {
        const double cx = double(rnd(3000000)), cy = double(rnd(3000000)), r0 = 200000. + double(rnd(1500000));
        ExPolygon ex;
        ex.contour = random_polygon(cx, cy, r0, true);
        if (rnd(3) == 0) ex.holes.push_back(random_polygon(cx, cy, r0 * 0.3, false));
        out.push_back(std::move(ex));
    }
    return out;
}

static void print_expolygons(FILE *out, const ExPolygons &expolys)
{
    std::fprintf(out, " %zu", expolys.size());
    for (const ExPolygon &ex : expolys) {
        std::fprintf(out, " %zu", ex.holes.size() + 1);
        auto pp = [out](const Polygon &p) {
            std::fprintf(out, " %zu", p.points.size());
            for (const Point &pt : p.points) std::fprintf(out, " %lld %lld", (long long) pt.x(), (long long) pt.y());
        };
        pp(ex.contour);
        for (const Polygon &h : ex.holes) pp(h);
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: gen_merge <out.txt>\n");
        return 2;
    }
    tbb::global_control one_thread(tbb::global_control::max_allowed_parallelism, 1);
    FILE *out = std::fopen(argv[1], "w");
    const int num_cases = 600;
    for (int c = 0; c < num_cases; ++c) {
        const size_t num_layers = 1 + rnd(3), num_extruders = 1 + rnd(4);
        std::vector<std::vector<ExPolygons>> segmented(num_layers, std::vector<ExPolygons>(num_extruders + 1));
        std::vector<std::vector<ExPolygons>> tb(num_extruders + 1, std::vector<ExPolygons>(num_layers));
        for (size_t l = 0; l < num_layers; ++l)
            for (size_t e = 0; e <= num_extruders; ++e) segmented[l][e] = random_expolygons();
        for (size_t e = 0; e <= num_extruders; ++e)
            for (size_t l = 0; l < num_layers; ++l) tb[e][l] = rnd(2) == 0 ? ExPolygons() : random_expolygons();
        std::fprintf(out, "ms %zu %zu", num_layers, num_extruders);
        for (size_t l = 0; l < num_layers; ++l)
            for (size_t e = 0; e <= num_extruders; ++e) print_expolygons(out, segmented[l][e]);
        std::fprintf(out, " ;");
        for (size_t e = 0; e <= num_extruders; ++e)
            for (size_t l = 0; l < num_layers; ++l) print_expolygons(out, tb[e][l]);
        std::vector<std::vector<ExPolygons>> merged = merge_segmented_layers(segmented, std::move(tb), num_extruders, []() {});
        std::fprintf(out, " =>");
        for (size_t l = 0; l < num_layers; ++l)
            for (size_t e = 0; e < num_extruders; ++e) print_expolygons(out, merged[l][e]);
        std::fprintf(out, "\n");
    }
    std::fclose(out);
    return 0;
}
