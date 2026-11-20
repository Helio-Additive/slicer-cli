// Generator of the vectors of tests/support_material_native_parity.rs: libslic3r's
// Support/SupportMaterial.cpp at the pin (references/BambuStudio/src/libslic3r; the override layer has no
// copy of it).
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
//   ./gen vectors.txt
//
// SupportMaterial.cpp is compiled into this file (most of its functions are static or members of classes the
// file keeps to itself), with the private members of the classes made public. tbb is limited to one thread.
//
// The first part ("grid") is the support grid: rasterize_polygons, contours_simplified, SupportGridParams and
// SupportGridPattern (the constructor, extract_support, dilate_trimming_region, seed_fill_block,
// island_samples) on polygons and rasters made from a fixed linear congruential generator.
//
// Lines that start with "in" are the inputs the Rust test reads back; the others are results it has to
// reproduce. Tokens are separated by one space; a double is its 16 hexadecimal digits, an integer decimal, a
// raster the hexadecimal digits of its bytes ("-" when it is empty). The vectors were made on macOS arm64
// with Apple clang and Eigen 5.0.1.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

// Before the access specifiers are redefined: tbb declares nested classes ahead of their definition.
#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_unordered_set.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/scalable_allocator.h>
#include <tbb/spin_mutex.h>
#include <tbb/task_group.h>

#include <boost/container/static_vector.hpp>

#define private public
#define protected public
#include "libslic3r/Support/SupportMaterial.cpp"
#undef protected
#undef private

using namespace Slic3r;

static FILE *g_out = nullptr;

static std::string hD(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, 8);
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long) u);
    return b;
}
static std::string hI(long long v) { return std::to_string(v); }
static std::string hB(bool v) { return v ? "1" : "0"; }
static void out(const std::string &s) { std::fprintf(g_out, "%s\n", s.c_str()); }
static std::string PT(const Point &p) { return hI(p.x()) + " " + hI(p.y()); }
static std::string PTS(const Points &pts)
{
    std::string o = hI((long long) pts.size());
    for (const Point &pt : pts) o += " " + PT(pt);
    return o;
}
static std::string POLY(const Polygon &p) { return PTS(p.points); }
static std::string POLYS(const Polygons &ps)
{
    std::string o = hI((long long) ps.size());
    for (const Polygon &p : ps) o += " " + POLY(p);
    return o;
}
static std::string EXPOLY(const ExPolygon &e)
{
    std::string o = POLY(e.contour) + " " + hI((long long) e.holes.size());
    for (const Polygon &h : e.holes) o += " " + POLY(h);
    return o;
}
static std::string EXPOLYS(const ExPolygons &es)
{
    std::string o = hI((long long) es.size());
    for (const ExPolygon &e : es) o += " " + EXPOLY(e);
    return o;
}
static std::string HEX(const std::vector<unsigned char> &d)
{
    if (d.empty()) return "-";
    static const char *digits = "0123456789abcdef";
    std::string o;
    o.reserve(d.size() * 2);
    for (unsigned char c : d) {
        o += digits[c >> 4];
        o += digits[c & 15];
    }
    return o;
}

// The generator of the inputs.
static uint64_t g_seed = 0x5DEECE66DULL;
static uint32_t lcg()
{
    g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL;
    return uint32_t(g_seed >> 33);
}
static int irand(int lo, int hi) { return lo + int(lcg() % uint32_t(hi - lo + 1)); }

static Polygon ccw(Polygon p)
{
    if (! p.is_counter_clockwise()) p.reverse();
    return p;
}
static Polygon rect(coord_t x0, coord_t y0, coord_t x1, coord_t y1)
{
    Polygon p;
    p.points = { Point(x0, y0), Point(x1, y0), Point(x1, y1), Point(x0, y1) };
    return p;
}
// A shape with its lower left corner near (ox, oy), not larger than `ext`.
static Polygon rnd_shape(coord_t ox, coord_t oy, coord_t ext)
{
    const coord_t w = irand(int(ext / 5), int(ext));
    const coord_t h = irand(int(ext / 5), int(ext));
    Polygon p;
    switch (irand(0, 4)) {
    case 0: p = rect(ox, oy, ox + w, oy + h); break;
    case 1: p.points = { Point(ox, oy), Point(ox + w, oy + irand(0, int(h / 3))), Point(ox + irand(0, int(w)), oy + h) }; break;
    case 2: p.points = { Point(ox + w / 2, oy), Point(ox + w, oy + h / 2), Point(ox + w / 2, oy + h), Point(ox, oy + h / 2) }; break;
    case 3: p.points = { Point(ox, oy), Point(ox + w, oy), Point(ox + w, oy + h / 3), Point(ox + w / 3, oy + h / 3), Point(ox + w / 3, oy + h), Point(ox, oy + h) }; break;
    default: {
        // An irregular polygon around a centre, by rational directions (no trigonometry).
        static const int dirs[8][2] = { { 4, 0 }, { 3, 3 }, { 0, 4 }, { -3, 3 }, { -4, 0 }, { -3, -3 }, { 0, -4 }, { 3, -3 } };
        for (int i = 0; i < 8; ++ i) {
            const int r = irand(40, 100);
            p.points.emplace_back(Point(ox + w / 2 + coord_t(int64_t(w) * dirs[i][0] * r / 800), oy + h / 2 + coord_t(int64_t(h) * dirs[i][1] * r / 800)));
        }
    }
    }
    return ccw(p);
}
static Polygons rnd_shapes(int n, coord_t area, coord_t ext)
{
    Polygons out;
    for (int i = 0; i < n; ++ i)
        out.emplace_back(rnd_shape(irand(0, int(area)), irand(0, int(area)), ext));
    return out;
}
// A raster of 0 and `on` with unset boundary pixels.
static std::vector<unsigned char> rnd_grid(int gx, int gy, int percent, bool mixed)
{
    std::vector<unsigned char> g(size_t(gx * gy), 0);
    for (int r = 1; r + 1 < gy; ++ r)
        for (int c = 1; c + 1 < gx; ++ c)
            if (irand(0, 99) < percent)
                g[size_t(r * gx + c)] = mixed ? (unsigned char) irand(1, 255) : 1;
    return g;
}

static std::string PARAMS(const SupportGridParams &p)
{
    return hI(int(p.style)) + " " + hD(p.grid_resolution) + " " + hD(p.support_angle) + " " + hD(p.extrusion_width) + " " + hD(p.support_closing_radius) + " " +
           hI(p.expansion_to_slice) + " " + hI(p.expansion_to_propagate);
}

static void run_grid()
{
    // rasterize_polygons
    for (int i = 0; i < 24; ++ i) {
        const Vec2i  grid_size(irand(4, 40), irand(4, 40));
        const double pixel_size = (i % 3 == 0) ? 45021. : (i % 3 == 1) ? double(irand(30000, 90000)) + 0.5 : double(irand(200000, 400000)) / 3.;
        const Point  left_bottom(irand(-3000000, 3000000), irand(-3000000, 3000000));
        const coord_t span = coord_t(std::max(grid_size.x(), grid_size.y()) * pixel_size);
        Polygons polygons = rnd_shapes(irand(1, 5), span, span / 2 + 10);
        // Some of them stick out of the raster, to the left and to the bottom as well.
        for (Polygon &p : polygons)
            p.translate(left_bottom - Point(i % 4 == 3 ? span / 8 : 0, i % 4 == 2 ? span / 8 : 0));
        if (i % 5 == 4) {
            // with holes, as a difference gives them
            polygons = diff(polygons, rnd_shapes(2, span, span / 4 + 10).front().points.empty() ? Polygons() : Polygons{ rect(left_bottom.x() + span / 4, left_bottom.y() + span / 4, left_bottom.x() + span / 3, left_bottom.y() + span / 3) });
        }
        out("in rast " + hI(grid_size.x()) + " " + hI(grid_size.y()) + " " + hD(pixel_size) + " " + PT(left_bottom) + " " + POLYS(polygons));
        out("rast " + HEX(rasterize_polygons(grid_size, pixel_size, left_bottom, polygons)));
    }
    // dilate_trimming_region
    for (int i = 0; i < 16; ++ i) {
        const Vec2i grid_size(irand(3, 30), irand(3, 30));
        const std::vector<unsigned char> trimming = rnd_grid(grid_size.x(), grid_size.y(), 55 + 3 * i, i % 2 == 1);
        out("in dil " + hI(grid_size.x()) + " " + hI(grid_size.y()) + " " + HEX(trimming));
        out("dil " + HEX(SupportGridPattern::dilate_trimming_region(trimming, grid_size)));
    }
    // seed_fill_block
    for (int i = 0; i < 24; ++ i) {
        const int   oversampling = 1 + i % 8;
        const Vec2i grid_blocks(irand(1, 6), irand(1, 6));
        const Vec2i grid_size = grid_blocks * oversampling + Vec2i(2, 2);
        std::vector<unsigned char> grid = rnd_grid(grid_size.x(), grid_size.y(), 8 + i, i % 3 == 2);
        const std::vector<unsigned char> trimming = rnd_grid(grid_size.x(), grid_size.y(), 5 + 2 * i, i % 2 == 1);
        out("in seed " + hI(grid_size.x()) + " " + hI(grid_size.y()) + " " + hI(grid_blocks.x()) + " " + hI(grid_blocks.y()) + " " + hI(oversampling) + " " + HEX(grid) + " " + HEX(trimming));
        SupportGridPattern::seed_fill_block(grid, grid_size, trimming, grid_blocks, oversampling);
        out("seed " + HEX(grid));
    }
    // contours_simplified
    for (int i = 0; i < 40; ++ i) {
        const Vec2i  grid_size(irand(3, 26), irand(3, 26));
        const double pixel_size = (i % 3 == 0) ? 45021. : (i % 3 == 1) ? double(irand(30000, 90000)) + 0.5 : double(irand(200000, 400000)) / 3.;
        const Point  left_bottom(irand(-3000000, 3000000), irand(-3000000, 3000000));
        const std::vector<unsigned char> grid = rnd_grid(grid_size.x(), grid_size.y(), 20 + (i * 7) % 60, i % 4 == 1);
        const coord_t offset = (i % 5 == 0) ? 0 : (i % 5 == 1) ? -3 : (i % 5 == 2) ? 22505 - 5 - 6 : irand(-9000, 9000);
        const bool    fill_holes = (i / 2) % 2 == 0;
        out("in cs " + hI(grid_size.x()) + " " + hI(grid_size.y()) + " " + hD(pixel_size) + " " + PT(left_bottom) + " " + hI(offset) + " " + hB(fill_holes) + " " + HEX(grid));
        out("cs " + POLYS(contours_simplified(grid_size, pixel_size, left_bottom, grid, offset, fill_holes)));
    }
    // island_samples
    for (int i = 0; i < 16; ++ i) {
        ExPolygons expolygons = union_ex(rnd_shapes(irand(1, 6), 4000000, 1500000));
        if (i % 4 == 1)
            expolygons = diff_ex(rnd_shapes(irand(2, 6), 4000000, 2500000), rnd_shapes(irand(1, 4), 4000000, 800000));
        if (i % 4 == 2) {
            // an island too thin for the offset, a contour of two points
            expolygons.emplace_back(ExPolygon(rect(9000000, 0, 9000030, 500000)));
            ExPolygon two;
            two.contour.points = { Point(0, 0), Point(100, 100) };
            expolygons.emplace_back(two);
        }
        out("in isl " + EXPOLYS(expolygons));
        out("isl " + PTS(SupportGridPattern::island_samples(expolygons)));
    }
    // SupportGridParams
    {
        static const SupportMaterialStyle styles[] = { smsDefault, smsGrid, smsSnug, smsTreeSlim, smsTreeStrong, smsTreeHybrid, smsTreeOrganic };
        static const float widths[]  = { 0.42f, 0.45f, 0.5f, 0.62f, 0.82f, 0.38f };
        static const float heights[] = { 0.2f, 0.12f, 0.28f, 0.3f, 0.4f, 0.08f };
        static const float nozzles[] = { 0.4f, 0.4f, 0.4f, 0.6f, 0.8f, 0.2f };
        static const double spacings[] = { 2.5, 0., 1., 3.3, 0.2, 10. };
        static const double angles[]   = { 0., 15., 45., -30., 90., 37.5 };
        for (int i = 0; i < 14; ++ i) {
            PrintObjectConfig object_config;
            object_config.support_style.value = styles[i % 7];
            object_config.support_base_pattern_spacing.value = spacings[i % 6];
            object_config.support_angle.value = angles[(i / 2) % 6];
            const Flow flow(widths[i % 6], heights[i % 6], nozzles[i % 6]);
            out("in par " + hI(int(object_config.support_style.value)) + " " + hD(object_config.support_base_pattern_spacing.value) + " " + hD(object_config.support_angle.value) + " " +
                hD(double(flow.width())) + " " + hD(double(flow.height())) + " " + hD(double(flow.nozzle_diameter())));
            out("par " + PARAMS(SupportGridParams(object_config, flow)));
        }
    }
    // SupportGridPattern: the constructor and extract_support
    {
        static const SupportMaterialStyle styles[] = { smsGrid, smsDefault, smsSnug, smsGrid, smsTreeHybrid, smsSnug, smsGrid, smsTreeOrganic };
        static const double angles[]   = { 0., 0., 0., 15., 45., 30., -30., 90. };
        static const double spacings[] = { 2.5, 0.2, 2.5, 1., 3.3, 2.5, 0., 10. };
        static const float widths[]  = { 0.42f, 0.45f, 0.5f, 0.62f, 0.82f, 0.38f };
        static const float heights[] = { 0.2f, 0.12f, 0.28f, 0.3f, 0.4f, 0.08f };
        static const float nozzles[] = { 0.4f, 0.4f, 0.4f, 0.6f, 0.8f, 0.2f };
        for (int i = 0; i < 48; ++ i) {
            PrintObjectConfig object_config;
            object_config.support_style.value = styles[i % 8];
            object_config.support_base_pattern_spacing.value = spacings[(i / 8 + i) % 8];
            object_config.support_angle.value = angles[i % 8];
            const Flow flow(widths[i % 6], heights[i % 6], nozzles[i % 6]);
            const SupportGridParams params(object_config, flow);
            // The object and the support around it, trimmed by the object with a gap.
            const coord_t area = irand(1500000, 6000000);
            Polygons trimming_polygons = union_(rnd_shapes(irand(1, 3), area, area / 2));
            Polygons support_polygons  = diff(rnd_shapes(irand(1, 5), area, area / 2 + area / 4), offset(trimming_polygons, float(irand(10000, 60000))));
            if (i == 7)
                support_polygons.clear();
            if (i == 13)
                trimming_polygons.clear();
            out("in pat " + PARAMS(params) + " " + POLYS(support_polygons) + " " + POLYS(trimming_polygons));
            SupportGridPattern pattern(&support_polygons, &trimming_polygons, params);
            if (pattern.m_style == smsGrid)
                out("pat " + hI(pattern.m_grid_size.x()) + " " + hI(pattern.m_grid_size.y()) + " " + hD(pattern.m_pixel_size) + " " + PT(pattern.m_bbox.min) + " " + PT(pattern.m_bbox.max) + " " + HEX(pattern.m_grid2));
            else
                out("pat " + hI(int(pattern.m_style)));
            const coord_t offsets[4] = { params.expansion_to_slice, params.expansion_to_propagate, 0, coord_t(irand(-2000, 2000)) };
            for (int k = 0; k < 4; ++ k) {
                const bool fill_holes = (k + i) % 2 == 0;
                out("in ext " + hI(offsets[k]) + " " + hB(fill_holes));
                out("ext " + POLYS(pattern.extract_support(offsets[k], fill_holes)));
            }
        }
    }
}

int main(int argc, char **argv)
{
    g_out = std::fopen(argc > 1 ? argv[1] : "vectors.txt", "w");
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism, 1);
    run_grid();
    std::fclose(g_out);
    return 0;
}
