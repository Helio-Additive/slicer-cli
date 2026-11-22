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

// ---------------------------------------------------------------------------------------------------------
// The stages of PrintObjectSupportMaterial on the object of a Print (the "s" scenarios).
// ---------------------------------------------------------------------------------------------------------

static std::string hF(float v)
{
    uint32_t u;
    std::memcpy(&u, &v, 4);
    char b[16];
    std::snprintf(b, sizeof(b), "%08x", (unsigned) u);
    return b;
}
static std::string FLOATS(const std::vector<float> &v)
{
    std::string o = hI((long long) v.size());
    for (float f : v) o += " " + hF(f);
    return o;
}
static std::string PLS(const Polylines &ps)
{
    std::string o = hI((long long) ps.size());
    for (const Polyline &p : ps) o += " " + PTS(p.points);
    return o;
}
static std::string PATH(const ExtrusionPath &p)
{
    return "P " + hI(int(p.role())) + " " + hD(p.mm3_per_mm) + " " + hF(p.width) + " " + hF(p.height) + " " + PTS(p.polyline.points);
}
static std::string PATHS(const ExtrusionPaths &paths)
{
    std::string o = hI((long long) paths.size());
    for (const ExtrusionPath &p : paths) o += " " + PATH(p);
    return o;
}
static std::string ENT(const ExtrusionEntity *e)
{
    if (const auto *c = dynamic_cast<const ExtrusionEntityCollection *>(e)) {
        std::string o = "C " + hI((long long) c->entities.size());
        for (const ExtrusionEntity *ee : c->entities) o += " " + ENT(ee);
        return o;
    }
    if (const auto *l = dynamic_cast<const ExtrusionLoop *>(e)) return "L " + PATHS(l->paths);
    if (const auto *m = dynamic_cast<const ExtrusionMultiPath *>(e)) return "M " + PATHS(m->paths);
    if (const auto *p = dynamic_cast<const ExtrusionPath *>(e)) return PATH(*p);
    return "X";
}
static std::string ENTS(const ExtrusionEntitiesPtr &v)
{
    std::string o = hI((long long) v.size());
    for (const ExtrusionEntity *e : v) o += " " + ENT(e);
    return o;
}
// A surface: its type, whether its bridge angle is set (it is -1 when not), the angle, the expolygon.
static std::string SURFS(const Surfaces &v)
{
    std::string o = hI((long long) v.size());
    for (const Surface &s : v) o += " " + hI(int(s.surface_type)) + " " + hB(s.bridge_angle != -1) + " " + hD(s.bridge_angle) + " " + EXPOLY(s.expolygon);
    return o;
}

static std::map<const SupportGeneratorLayer *, int> g_index;
static void reindex(SupportGeneratorLayerStorage &storage)
{
    g_index.clear();
    int i = 0;
    for (const SupportGeneratorLayer &l : storage.m_storage) g_index[&l] = i++;
}
static std::string OPT(const std::unique_ptr<Polygons> &p) { return p ? "1 " + POLYS(*p) : std::string("0"); }
static std::string LAYER(const SupportGeneratorLayer &l)
{
    return hI((int) l.layer_type) + " " + hD(l.print_z) + " " + hD(l.bottom_z) + " " + hD(l.height) + " " + hI((long long) l.idx_object_layer_above) + " " +
           hI((long long) l.idx_object_layer_below) + " " + hB(l.bridging) + " " + hB(l.up) + " " + POLYS(l.polygons) + " " + OPT(l.contact_polygons) + " " +
           OPT(l.overhang_polygons) + " " + OPT(l.enforcer_polygons);
}
static std::string LIST(const SupportGeneratorLayersPtr &v)
{
    std::string o = hI((long long) v.size());
    for (const SupportGeneratorLayer *l : v) o += " " + hI(l == nullptr ? -1 : g_index.at(l));
    return o;
}
static void dump_storage(const std::string &tag, SupportGeneratorLayerStorage &storage)
{
    reindex(storage);
    out(tag + " storage " + hI((long long) storage.m_storage.size()));
    for (const SupportGeneratorLayer &l : storage.m_storage) out(tag + " L " + LAYER(l));
}
static std::string POLYSV(const std::vector<Polygons> &v)
{
    std::string o = hI((long long) v.size());
    for (const Polygons &p : v) o += " " + POLYS(p);
    return o;
}

static coord_t  sc(double mm) { return coord_t(std::lrint(mm / SCALING_FACTOR)); }
static Polygon  rectmm(double x0, double y0, double x1, double y1) { return rect(sc(x0), sc(y0), sc(x1), sc(y1)); }
static Polyline plmm(std::initializer_list<std::pair<double, double>> pts)
{
    Polyline pl;
    for (const auto &p : pts) pl.points.emplace_back(Point(sc(p.first), sc(p.second)));
    return pl;
}
static ExtrusionPath pathmm(ExtrusionRole role, float width, float height, std::initializer_list<std::pair<double, double>> pts)
{
    ExtrusionPath p(role, 0.04, width, height);
    p.polyline = plmm(pts);
    return p;
}

typedef std::vector<std::pair<std::string, std::string>> Cfg;

// ---------------------------------------------------------------------------------------------------------
// The helpers the scenarios do not reach with every branch (the "misc" group): layers_append,
// collect_region_slices_by_type and collect_slices_outer on a layer of two regions whose surfaces have
// holes, OverhangCluster (insert, height, intersects) and add_overhang.
// ---------------------------------------------------------------------------------------------------------

// A rectangle with, when `hole`, a rectangular hole inside.
static ExPolygon rnd_expoly(coord_t area, coord_t ext, bool hole)
{
    const coord_t ox = irand(0, int(area)), oy = irand(0, int(area));
    const coord_t w = irand(int(ext / 3), int(ext)), h = irand(int(ext / 3), int(ext));
    ExPolygon e(rect(ox, oy, ox + w, oy + h));
    if (hole) {
        Polygon hl = rect(ox + w / 4, oy + h / 4, ox + w / 4 + w / 3, oy + h / 4 + h / 3);
        hl.reverse();
        e.holes.emplace_back(hl);
    }
    return e;
}

static void run_misc()
{
    // layers_append
    for (int k = 0; k < 6; ++k) {
        SupportGeneratorLayerStorage storage;
        const int n = irand(1, 9);
        for (int i = 0; i < n; ++i) storage.allocate(SupporLayerType(irand(1, 6)));
        reindex(storage);
        std::vector<SupportGeneratorLayer *> all;
        for (SupportGeneratorLayer &l : storage.m_storage) all.emplace_back(&l);
        SupportGeneratorLayersPtr dst, src;
        const int nd = irand(0, 5), ns = irand(0, 5);
        for (int i = 0; i < nd; ++i) dst.emplace_back(all[size_t(irand(0, n - 1))]);
        for (int i = 0; i < ns; ++i) src.emplace_back(all[size_t(irand(0, n - 1))]);
        out("in misc la " + hI(n) + " " + LIST(dst) + " " + LIST(src));
        layers_append(dst, src);
        out("misc la " + LIST(dst));
    }
    // collect_region_slices_by_type, collect_slices_outer
    for (int k = 0; k < 8; ++k) {
        Layer layer(0, nullptr, 0.2, 0.2, 0.1);
        const int nr = irand(1, 3);
        std::string in = "in misc crs " + hI(nr);
        for (int r = 0; r < nr; ++r) {
            LayerRegion *lr = layer.add_region(nullptr);
            const int    ns = irand(0, 5);
            for (int i = 0; i < ns; ++i) {
                static const SurfaceType types[] = {stTop, stBottom, stBottomBridge, stInternal};
                lr->slices.surfaces.emplace_back(types[irand(0, 3)], rnd_expoly(4000000, 3000000, irand(0, 2) == 0));
            }
            in += " " + SURFS(lr->slices.surfaces);
        }
        const int nl = irand(0, 4);
        for (int i = 0; i < nl; ++i) layer.lslices.emplace_back(rnd_expoly(4000000, 3000000, irand(0, 1) == 0));
        out(in + " " + EXPOLYS(layer.lslices));
        out("misc crs " + POLYS(collect_region_slices_by_type(layer, stTop)) + " " + POLYS(collect_region_slices_by_type(layer, stBottom)) + " " +
            POLYS(collect_region_slices_by_type(layer, stBottomBridge)) + " " + POLYS(collect_region_slices_by_type(layer, stInternal)) + " " + POLYS(collect_slices_outer(layer)));
    }
    // OverhangCluster and add_overhang: overhangs of consecutive layers go into the clusters one by one.
    for (int k = 0; k < 16; ++k) {
        static const double offsets[] = {0., 1e-5, 200000., 420000., 650000., 1000000.};
        const double offset_scaled = offsets[k % 6];
        const int    n             = irand(3, 14);
        ExPolygons   overhangs;
        std::vector<int> layer_nrs;
        int          layer_nr = irand(0, 2);
        for (int i = 0; i < n; ++i) {
            overhangs.emplace_back(rnd_expoly(6000000, 2500000, irand(0, 3) == 0));
            layer_nrs.emplace_back(layer_nr);
            layer_nr += irand(0, 2) == 0 ? irand(0, 3) : irand(0, 1);
        }
        std::string in = "in misc oc " + hD(offset_scaled) + " " + hI(n);
        for (int i = 0; i < n; ++i) in += " " + hI(layer_nrs[size_t(i)]) + " " + EXPOLY(overhangs[size_t(i)]);
        out(in);
        std::vector<OverhangCluster> clusters;
        clusters.reserve(size_t(n));
        std::string added = "misc oc added";
        for (int i = 0; i < n; ++i) {
            // What every cluster answers before the overhang is added.
            std::string hits;
            for (OverhangCluster &c : clusters) hits += hB(c.intersects(overhangs[size_t(i)], layer_nrs[size_t(i)]));
            OverhangCluster *c = add_overhang(clusters, &overhangs[size_t(i)], layer_nrs[size_t(i)], offset_scaled);
            added += " " + hI((long long) (c - clusters.data())) + ":" + (hits.empty() ? std::string("-") : hits);
        }
        out(added);
        out("misc oc clusters " + hI((long long) clusters.size()));
        for (OverhangCluster &c : clusters) {
            std::string o = "misc oc C " + hI(c.min_layer) + " " + hI(c.max_layer) + " " + hI(c.height()) + " " + hD(c.offset_scaled) + " " + hI((long long) c.layer_overhangs.size());
            for (const auto &kv : c.layer_overhangs) {
                o += " " + hI(kv.first) + " " + hI((long long) kv.second.size());
                for (const ExPolygon *e : kv.second) o += " " + hI((long long) (e - overhangs.data()));
            }
            out(o + " " + EXPOLYS(c.merged_overhangs_dilated));
        }
    }
}

struct Scen
{
    Cfg cfg;
    // 0: a T, a bridge between two pillars, a block under the right arm of the T and an island that starts
    //    in the air; 1: the same without the block and the island, the first slab layer without bridging
    //    extrusions.
    int shape;
    // Volumes besides the box: 1 a support enforcer over the left arm of the T, 2 a support blocker over a part
    // of its right arm.
    int volumes = 0;
};

static std::vector<Scen> scenarios()
{
    std::vector<Scen> s;
    // 0: automatic normal supports on a grid.
    s.push_back({{{"enable_support", "1"}, {"support_type", "normal(auto)"}}, 0});
    // 1: snug, the layers of the supports tied to the object's.
    s.push_back({{{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_style", "snug"}, {"independent_support_layer_height", "0"}}, 0});
    // 2: a raft.
    s.push_back({{{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"raft_layers", "3"}}, 0});
    // 3: on the build plate only.
    s.push_back({{{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_on_build_plate_only", "1"}}, 0});
    // 4: no gap above the supports (a soluble interface), thick bridges.
    s.push_back({{{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_top_z_distance", "0"}, {"support_bottom_z_distance", "0"}, {"thick_bridges", "1"},
                  {"independent_support_layer_height", "0"}},
                 0});
    // 5: thick bridges, no supports under bridges, the areas expanded.
    s.push_back({{{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"thick_bridges", "1"}, {"bridge_no_support", "1"},
                  {"support_expansion", "0.8"}},
                 0});
    // 6: the areas shrunk, small overhangs removed, a threshold angle, the grid turned, tied layers and thick bridges.
    s.push_back({{{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_expansion", "-0.3"}, {"support_remove_small_overhang", "1"}, {"support_threshold_angle", "40"},
                  {"support_angle", "30"}, {"thick_bridges", "1"}, {"bridge_no_support", "1"}, {"independent_support_layer_height", "0"}},
                 0});
    // 7: manual supports with the first layers enforced, a raft of one layer.
    s.push_back({{{"enable_support", "1"}, {"support_type", "normal(manual)"}, {"enforce_support_layers", "14"}, {"raft_layers", "1"}, {"raft_expansion", "1.5"}}, 1});
    // 8: a raft only.
    s.push_back({{{"enable_support", "0"}, {"raft_layers", "2"}}, 1});
    // 9: tree supports: no normal supports.
    s.push_back({{{"enable_support", "1"}, {"support_type", "tree(auto)"}}, 1});
    // 10: no threshold angle, small overhangs removed, a wider gap to the object.
    // 11: a raft with tied layers, on the build plate only, a gap below the supports of two layers.
    s.push_back({{{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_threshold_angle", "0"}, {"support_remove_small_overhang", "1"}, {"support_object_xy_distance", "0.6"},
                  {"support_base_pattern_spacing", "1.2"}},
                 0});
    s.push_back({{{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"raft_layers", "2"}, {"independent_support_layer_height", "0"}, {"support_on_build_plate_only", "1"},
                  {"support_bottom_z_distance", "0.4"}, {"support_top_z_distance", "0.3"}},
                 0});
    // 12: manual supports under a support enforcer.
    s.push_back({{{"enable_support", "1"}, {"support_type", "normal(manual)"}}, 0, 1});
    // 13: an enforcer and a blocker, on the build plate only.
    s.push_back({{{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_on_build_plate_only", "1"}}, 0, 3});
    // 14: a blocker, snug, tied layers.
    s.push_back({{{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_style", "snug"}, {"independent_support_layer_height", "0"}}, 0, 2});
    // 15: two nozzle diameters, the supports of the coarse one.
    s.push_back({{{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"multi_nozzle_diameter_mode", "1"}, {"extruder_nozzle_count", "2"}, {"extruder_nozzle_diameter", "0.4,0.8"},
                  {"filament_diameter", "1.75,1.75,1.75"}, {"filament_diameter_map", "0.4;0.8;0.4"}, {"support_filament", "2"}, {"support_interface_filament", "2"},
                  {"layer_height", "0.1"}},
                 0});
    // 16: two nozzle diameters, the base of the object's filament, the interface of the coarse nozzle, tied layers.
    s.push_back({{{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"multi_nozzle_diameter_mode", "1"}, {"extruder_nozzle_count", "2"}, {"extruder_nozzle_diameter", "0.4,0.8"},
                  {"filament_diameter", "1.75,1.75,1.75"}, {"filament_diameter_map", "0.4;0.8;0.4"}, {"support_filament", "1"}, {"support_interface_filament", "2"},
                  {"layer_height", "0.1"}, {"independent_support_layer_height", "0"}},
                 1});
    return s;
}

// `max_stage`: 1 the helpers and top_contact_layers, 2 the stages one by one, 3 generate() as well.
static void run_object(int scenario, const Scen &scen, int max_stage)
{
    const std::string S = "s" + hI(scenario);
    Model        model;
    ModelObject *o = model.add_object();
    o->name = "box";
    indexed_triangle_set its = its_make_cube(20, 20, 10);
    for (Vec3f &v : its.vertices) v += Vec3f(-10.f, -10.f, 0.f);
    o->add_volume(TriangleMesh(its));
    if (scen.volumes & 1) {
        // x -10..-6, y -7..7, z 2.35..3.3: the slab layers over the left arm of the T.
        indexed_triangle_set e = its_make_cube(4, 14, 0.95);
        for (Vec3f &v : e.vertices) v += Vec3f(-10.f, -7.f, 2.35f);
        o->add_volume(TriangleMesh(e))->set_type(ModelVolumeType::SUPPORT_ENFORCER);
    }
    if (scen.volumes & 2) {
        // x 6..10, y -7..0, z 2.35..3.3
        indexed_triangle_set e = its_make_cube(4, 7, 0.95);
        for (Vec3f &v : e.vertices) v += Vec3f(6.f, -7.f, 2.35f);
        o->add_volume(TriangleMesh(e))->set_type(ModelVolumeType::SUPPORT_BLOCKER);
    }
    o->add_instance()->set_offset(Vec3d(100., 100., 0.));
    out("in " + S + " model 20 20 10 100 100 " + hI(scen.volumes));

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(3);
    config.option<ConfigOptionStrings>("filament_colour", true)->values = {"#FF0000", "#00FF00", "#0000FF"};
    config.option<ConfigOptionInts>("filament_map", true)->values = {1, 1, 1};
    for (const auto &kv : scen.cfg) {
        config.set_deserialize_strict(kv.first, kv.second);
        out("in " + S + " cfg " + kv.first + " " + kv.second);
    }
    Print print;
    print.apply(model, config);
    PrintObject *obj = print.m_objects.front();
    obj->update_slicing_parameters();
    const SlicingParameters &sp = obj->slicing_parameters();
    out(S + " slicing " + hD(sp.layer_height) + " " + hD(sp.first_print_layer_height) + " " + hD(sp.first_object_layer_height) + " " + hI((long long) sp.raft_layers()) + " " +
        hD(sp.raft_interface_top_z) + " " + hD(sp.raft_contact_top_z) + " " + hD(sp.contact_raft_layer_height) + " " + hD(sp.object_print_z_min) + " " + hB(sp.soluble_interface) + " " +
        hD(sp.gap_support_object) + " " + hD(sp.gap_object_support) + " " + hD(sp.max_suport_layer_height));

    // The object layers, written by hand; one region.
    const int    n_pillar = 12, n_layers = 16;
    const size_t raft     = sp.raft_layers();
    double       z        = sp.object_print_z_min;
    for (int i = 0; i < n_layers; ++i) {
        const double h = i == 0 ? sp.first_object_layer_height : sp.layer_height;
        z += h;
        Layer       *layer = new Layer(raft + i, obj, h, z, z - h / 2.);
        LayerRegion *lr    = layer->add_region(&obj->printing_region(0));
        Surfaces    &surfs = lr->slices.surfaces;
        const float  fh    = float(h);
        if (i < n_pillar) {
            surfs.emplace_back(stInternal, ExPolygon(rectmm(-5, -5, 5, 5)));    // the pillar of the T
            surfs.emplace_back(stInternal, ExPolygon(rectmm(-15, 25, -9, 35))); // the pillars of the bridge
            surfs.emplace_back(stInternal, ExPolygon(rectmm(9, 25, 15, 35)));
            if (scen.shape == 0) {
                // a block under the right arm, wider from its third layer on: an overhang next to the first layer
                if (i < 5) surfs.emplace_back(i == 4 ? stTop : stInternal, ExPolygon(i < 2 ? rectmm(8, -6, 14, 6) : rectmm(8, -6, 17, 6)));
                if (i >= 5 && i < 10) surfs.emplace_back(i == 9 ? stTop : (i == 5 ? stBottom : stInternal), ExPolygon(rectmm(-22, -1, -20.5, 0.5))); // an island in the air
            }
            for (const Surface &s : surfs) layer->lslices.emplace_back(s.expolygon);
        } else if (i == n_pillar) {
            // The first layer of the slabs: the bottoms of the arms of the T, the bridge between the pillars.
            surfs.emplace_back(stInternal, ExPolygon(rectmm(-5, -8, 5, 8)));
            surfs.emplace_back(stBottom, ExPolygon(rectmm(-15, -8, -5, 8)));
            surfs.emplace_back(stBottom, ExPolygon(rectmm(5, -8, 15, 8)));
            surfs.emplace_back(stInternal, ExPolygon(rectmm(-15, 25, -9, 35)));
            surfs.emplace_back(stBottomBridge, ExPolygon(rectmm(-9, 25, 9, 35)));
            surfs.back().bridge_angle = 0.;
            surfs.emplace_back(stInternal, ExPolygon(rectmm(9, 25, 15, 35)));
            layer->lslices = {ExPolygon(rectmm(-15, -8, 15, 8)), ExPolygon(rectmm(-15, 25, 15, 35))};
            if (scen.shape == 0) {
                // Perimeters: a collection with a loop of an outer wall and a loop with an overhang wall; a loop of an
                // open overhang wall; walls across the bridge (straight and supported at both ends, bent, short of
                // the far pillar).
                ExtrusionEntityCollection *c = new ExtrusionEntityCollection();
                c->entities.emplace_back(new ExtrusionLoop(pathmm(erExternalPerimeter, 0.42f, fh, {{-14.8, -7.8}, {14.8, -7.8}, {14.8, 7.8}, {-14.8, 7.8}, {-14.8, -7.8}})));
                {
                    ExtrusionPaths paths;
                    paths.emplace_back(pathmm(erPerimeter, 0.45f, fh, {{-14.3, -7.3}, {-5, -7.3}}));
                    paths.emplace_back(pathmm(erOverhangPerimeter, 0.45f, fh, {{-5, -7.3}, {-5, 7.3}, {-14.3, 7.3}, {-14.3, -7.3}}));
                    c->entities.emplace_back(new ExtrusionLoop(std::move(paths)));
                }
                c->entities.emplace_back(new ExtrusionLoop(pathmm(erOverhangPerimeter, 0.45f, fh, {{6, -7}, {14, -7}, {14, 7}, {6, 7}, {6, -7}})));
                lr->perimeters.entities.emplace_back(c);
                lr->perimeters.entities.emplace_back(new ExtrusionLoop(pathmm(erOverhangPerimeter, 0.5f, fh, {{-8.7, 34.6}, {8.7, 34.6}})));
                lr->perimeters.entities.emplace_back(new ExtrusionPath(pathmm(erPerimeter, 0.45f, fh, {{-9.6, 30}, {9.6, 30}})));
                lr->perimeters.entities.emplace_back(new ExtrusionPath(pathmm(erPerimeter, 0.45f, fh, {{-9.6, 27}, {0, 27}, {0, 28}, {9.6, 28}})));
                lr->perimeters.entities.emplace_back(new ExtrusionPath(pathmm(erPerimeter, 0.45f, fh, {{-9.6, 32}, {7, 32}})));
                // Fills: a collection of bridge infill, a collection of solid infill.
                ExtrusionEntityCollection *f0 = new ExtrusionEntityCollection();
                f0->entities.emplace_back(new ExtrusionPath(pathmm(erBridgeInfill, 0.4f, 0.4f, {{-9, 26}, {9, 26}})));
                f0->entities.emplace_back(new ExtrusionPath(pathmm(erBridgeInfill, 0.4f, 0.4f, {{9, 26.5}, {-9, 26.5}})));
                ExtrusionEntityCollection *f1 = new ExtrusionEntityCollection();
                f1->entities.emplace_back(new ExtrusionPath(pathmm(erSolidInfill, 0.42f, fh, {{-4, -7}, {4, -7}})));
                lr->fills.entities.emplace_back(f1);
                lr->fills.entities.emplace_back(f0);
                lr->unsupported_bridge_edges = {plmm({{-3, 25}, {3, 25}}), plmm({{-2, 35}, {2, 35}, {2.5, 35}})};
            }
        } else {
            const SurfaceType t = i + 1 == n_layers ? stTop : stInternal;
            if (scen.shape == 0) {
                // The slab of the T a millimetre wider than its first layer (an overhang right above the one of the
                // first slab layer), from its third layer on with a small tab at its side (a small overhang).
                Polygon slab = rectmm(-16, -8, 16, 8);
                if (i >= n_pillar + 2)
                    slab.points = {Point(sc(-16), sc(-8)), Point(sc(16), sc(-8)), Point(sc(16), sc(-0.4)), Point(sc(16.6), sc(-0.4)),
                                   Point(sc(16.6), sc(0.4)), Point(sc(16), sc(0.4)),  Point(sc(16), sc(8)),    Point(sc(-16), sc(8))};
                surfs.emplace_back(t, ExPolygon(slab));
            } else
                surfs.emplace_back(t, ExPolygon(rectmm(-15, -8, 15, 8)));
            surfs.emplace_back(t, ExPolygon(rectmm(-15, 25, 15, 35)));
            for (const Surface &s : surfs) layer->lslices.emplace_back(s.expolygon);
            if (i == n_pillar + 1 && scen.shape == 0) {
                lr->perimeters.entities.emplace_back(new ExtrusionLoop(pathmm(erExternalPerimeter, 0.42f, fh, {{-15.8, -7.8}, {15.8, -7.8}, {15.8, 7.8}, {-15.8, 7.8}, {-15.8, -7.8}})));
                lr->perimeters.entities.emplace_back(new ExtrusionLoop(pathmm(erOverhangPerimeter, 0.45f, fh, {{15.3, -7.3}, {15.3, 7.3}, {15.7, 7.3}, {15.7, -7.3}, {15.3, -7.3}})));
            }
        }
        lr->raw_slices      = layer->lslices;
        lr->fill_surfaces.surfaces = surfs;
        for (const ExPolygon &e : layer->lslices) layer->lslices_bboxes.emplace_back(get_extents(e));
        if (! obj->m_layers.empty()) {
            layer->lower_layer = obj->m_layers.back();
            obj->m_layers.back()->upper_layer = layer;
        }
        obj->m_layers.emplace_back(layer);
        out("in " + S + " layer " + hI((long long) layer->id()) + " " + hD(layer->print_z) + " " + hD(layer->height) + " " + hD(layer->slice_z) + " " + EXPOLYS(layer->lslices));
        out("in " + S + " region " + SURFS(lr->slices.surfaces) + " " + EXPOLYS(lr->raw_slices) + " " + SURFS(lr->fill_surfaces.surfaces) + " " + ENTS(lr->perimeters.entities) + " " +
            ENTS(lr->fills.entities) + " " + PLS(lr->unsupported_bridge_edges));
    }
    auto layer_index = [obj](const Layer *l) -> long long {
        if (l == nullptr) return -1;
        for (size_t i = 0; i < obj->m_layers.size(); ++i)
            if (obj->m_layers[i] == l) return (long long) i;
        return -2;
    };

    PrintObjectSupportMaterial psm(obj, sp);
    out(S + " psm " + hB(psm.has_raft()) + " " + hB(psm.has_support()) + " " + hB(psm.build_plate_only()) + " " + hB(psm.synchronize_layers()) + " " + hB(psm.has_contact_loops()) + " " +
        hD(psm.m_support_params.gap_xy) + " " + hD(psm.m_support_params.support_layer_height_min));

    // SupportMaterialInternal
    for (int i = 0; i < n_layers; ++i) {
        const Layer       &layer = *obj->m_layers[i];
        const LayerRegion &lr    = *layer.regions().front();
        Polygons           areas;
        SupportMaterialInternal::collect_bridging_perimeter_areas(lr.perimeters, float(scale_(psm.m_support_params.gap_xy)), areas);
        out(S + " internal " + hI(i) + " " + hB(SupportMaterialInternal::has_bridging_extrusions(layer)) + " " + hB(SupportMaterialInternal::has_bridging_perimeters(lr.perimeters)) + " " +
            hB(SupportMaterialInternal::has_bridging_fills(lr.fills)) + " " + POLYS(areas));
        if (i >= n_pillar && i <= n_pillar + 1) {
            const Layer &lower = *layer.lower_layer;
            Polygons     lower_polygons = to_polygons(lower.lslices);
            Polygons     contact        = diff(to_polygons(lr.slices.surfaces), lower_polygons);
            const float  fw             = float(lr.flow(frExternalPerimeter).scaled_width());
            SupportMaterialInternal::remove_bridges_from_contacts(print.config(), lower, lower_polygons, lr, fw, contact);
            out(S + " rbfc " + hI(i) + " " + hF(fw) + " " + POLYS(contact));
        }
    }
    // sync_gap_with_object_layer
    {
        static const double gaps[] = {0., 0.05, 0.1, 0.2, 0.25, 0.3, 0.45, 0.61};
        for (int i : {1, 2, 5, n_layers - 2})
            for (double gap : gaps)
                out(S + " sync " + hI(i) + " " + hD(gap) + " " + hI(layer_index(sync_gap_with_object_layer(*obj->m_layers[i], gap, true))) + " " +
                    hI(layer_index(sync_gap_with_object_layer(*obj->m_layers[i], gap, false))));
    }

    std::vector<Polygons> buildplate_covered = psm.buildplate_covered(*obj);
    out(S + " bp " + POLYSV(buildplate_covered));

    // detect_overhangs layer by layer; the members of the layer it writes are put back.
    {
        SupportAnnotations annotations(*obj, buildplate_covered);
        out(S + " annotations " + POLYSV(annotations.enforcers_layers) + " " + POLYSV(annotations.blockers_layers));
        const size_t num_layers     = psm.has_support() ? obj->layer_count() : 1;
        const size_t layer_id_start = psm.has_raft() ? 0 : 1;
        for (size_t layer_id = layer_id_start; layer_id < num_layers; ++layer_id) {
            const Layer &layer = *obj->m_layers[layer_id];
            Polygons     lower_layer_polygons = layer_id == 0 ? Polygons() : to_polygons(obj->m_layers[layer_id - 1]->lslices);
            const ExPolygons         sharp_tails        = layer.sharp_tails;
            const std::vector<float> sharp_tails_height = layer.sharp_tails_height;
            const ExPolygons         cantilevers        = layer.cantilevers;
            const ExPolygons overhangs = detect_overhangs(layer, layer_id, lower_layer_polygons, print.config(), obj->config(), annotations, psm.m_support_params.gap_xy);
            out(S + " ov " + hI((long long) layer_id) + " " + EXPOLYS(overhangs) + " " + EXPOLYS(layer.sharp_tails) + " " + FLOATS(layer.sharp_tails_height) + " " + EXPOLYS(layer.cantilevers));
            layer.sharp_tails        = sharp_tails;
            layer.sharp_tails_height = sharp_tails_height;
            layer.cantilevers        = cantilevers;
        }
    }

    SupportGeneratorLayerStorage storage;
    SupportGeneratorLayersPtr    top_contacts = psm.top_contact_layers(*obj, buildplate_covered, storage);
    dump_storage(S + " top", storage);
    out(S + " top list " + LIST(top_contacts));
    for (int i = 0; i < n_layers; ++i) {
        const Layer &layer = *obj->m_layers[i];
        out(S + " tails " + hI(i) + " " + EXPOLYS(layer.sharp_tails) + " " + FLOATS(layer.sharp_tails_height) + " " + EXPOLYS(layer.cantilevers));
    }
    if (max_stage < 2) return;

    std::vector<Polygons>     layer_support_areas;
    SupportGeneratorLayersPtr bottom_contacts = psm.bottom_contact_layers_and_layer_support_areas(*obj, top_contacts, buildplate_covered, storage, layer_support_areas);
    dump_storage(S + " bottom", storage);
    out(S + " bottom list " + LIST(bottom_contacts));
    out(S + " bottom areas " + POLYSV(layer_support_areas));
    out(S + " bottom bp " + POLYSV(buildplate_covered));

    SupportGeneratorLayersPtr intermediate_layers = psm.raft_and_intermediate_support_layers(*obj, bottom_contacts, top_contacts, storage);
    dump_storage(S + " inter", storage);
    out(S + " inter list " + LIST(intermediate_layers));

    psm.trim_support_layers_by_object(*obj, top_contacts, sp.gap_support_object, sp.gap_object_support, psm.m_support_params.gap_xy);
    dump_storage(S + " trim", storage);

    psm.generate_base_layers(*obj, bottom_contacts, top_contacts, intermediate_layers, layer_support_areas);
    dump_storage(S + " base", storage);

    psm.trim_top_contacts_by_bottom_contacts(*obj, bottom_contacts, top_contacts);
    dump_storage(S + " ttb", storage);
    if (max_stage < 3) return;

    // generate(): the support layers it leaves on the object, without their extrusions (they are the fillers').
    // The sharp tails and cantilevers of the layers are those of the stages above and of this run.
    psm.generate(*obj);
    out(S + " gen " + hI((long long) obj->support_layers().size()));
    for (const SupportLayer *sl : obj->support_layers())
        out(S + " gen L " + hI((long long) sl->id()) + " " + hI((long long) sl->interface_id()) + " " + hD(sl->print_z) + " " + hD(sl->height) + " " + hD(sl->slice_z) + " " +
            hI(sl->planned_filament_id) + " " + hD(sl->planned_nozzle_diameter) + " " + hI(sl->planned_interface_filament_id) + " " + hD(sl->planned_interface_nozzle_diameter) + " " +
            hD(sl->base_slab_height) + " " + hD(sl->interface_slab_height) + " " + EXPOLYS(sl->support_islands));
    for (int i = 0; i < n_layers; ++i) {
        const Layer &layer = *obj->m_layers[i];
        out(S + " gen tails " + hI(i) + " " + EXPOLYS(layer.sharp_tails) + " " + FLOATS(layer.sharp_tails_height) + " " + EXPOLYS(layer.cantilevers));
    }
}

int main(int argc, char **argv)
{
    g_out = std::fopen(argc > 1 ? argv[1] : "vectors.txt", "w");
    // The stages of the object scenarios to write: 1 up to top_contact_layers, 2 the stages one by one, 3 (default)
    // generate() as well.
    const int max_stage = argc > 2 ? std::atoi(argv[2]) : 3;
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism, 1);
    run_grid();
    run_misc();
    const std::vector<Scen> s = scenarios();
    for (size_t i = 0; i < s.size(); ++i) run_object(int(i), s[i], max_stage);
    std::fclose(g_out);
    return 0;
}
