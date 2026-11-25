// Generator of the vectors of tests/perimeter_generator_native_parity.rs: libslic3r's PerimeterGenerator.cpp
// at the pin (references/BambuStudio/src/libslic3r; the override layer has no copy of it).
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
// PerimeterGenerator.cpp is compiled into this file (most of its helpers are static), with the private
// members of PerimeterGenerator and LayerRegion made public.
//
// Every line but the configuration lines is "<what> <inputs> => <results>": the test reads the inputs
// back, calls the port and has to print the same results. Tokens are separated by one space; a double
// is its 16 hexadecimal digits, a float its 8, an integer decimal. A configuration line is
// "cfg<TAB>multi_nozzle_diameter_mode<TAB>key=value..." (the keys set over the defaults; the lines after
// it are computed with it) or "rcfg<TAB>index<TAB>key=value..." (the config of a PrintRegion).
// The vectors were made on macOS arm64 with Apple clang and Eigen 5.0.1.

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

// Before the access specifiers are redefined: tbb declares nested classes ahead of their definition.
#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_unordered_set.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/spin_mutex.h>

#define private public
#define protected public
#include "libslic3r/Print.hpp"
#include "libslic3r/PerimeterGenerator.cpp"
#undef private
#undef protected

#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"

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
static std::string hF(float v)
{
    uint32_t u;
    std::memcpy(&u, &v, 4);
    char b[16];
    std::snprintf(b, sizeof(b), "%08x", u);
    return b;
}
static std::string hI(long long v) { return std::to_string(v); }
static std::string hB(bool v) { return v ? "1" : "0"; }
static void out(const std::string &s) { std::fprintf(g_out, "%s\n", s.c_str()); }
static std::string PT(const Point &p) { return hI(p.x()) + " " + hI(p.y()); }
static std::string POLY(const Polygon &p)
{
    std::string o = hI((long long) p.points.size());
    for (const Point &pt : p.points) o += " " + PT(pt);
    return o;
}
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
// The corners of a box that is defined, "0" alone of one that is not.
static std::string BBOX(const BoundingBox &b) { return b.defined ? "1 " + PT(b.min) + " " + PT(b.max) : std::string("0"); }
static std::string PATHS(const ExtrusionPaths &paths)
{
    std::string o = hI((long long) paths.size());
    for (const ExtrusionPath &p : paths) {
        o += " " + hI((int) p.role()) + " " + hD(p.overhang_degree) + " " + hI((long long) p.polyline.points.size());
        for (const Point &pt : p.polyline.points) o += " " + PT(pt);
    }
    return o;
}

struct Lcg
{
    uint64_t s;
    explicit Lcg(uint64_t seed) : s(seed) {}
    uint32_t next()
    {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return uint32_t(s >> 33);
    }
    // An integer in [lo, hi].
    long long range(long long lo, long long hi) { return lo + (long long) (next() % uint32_t(hi - lo + 1)); }
};

static Polygon rect(coord_t x0, coord_t y0, coord_t x1, coord_t y1)
{
    Polygon p;
    p.points = {Point(x0, y0), Point(x1, y0), Point(x1, y1), Point(x0, y1)};
    return p;
}
// A convex polygon of eight corners around (cx, cy), counter clockwise, with integer corners that
// depend on no trigonometric function: a rectangle cut at its corners.
static Polygon blob(Lcg &g, coord_t cx, coord_t cy, coord_t rx, coord_t ry)
{
    coord_t cut_x = g.range(rx / 8, rx / 3), cut_y = g.range(ry / 8, ry / 3);
    Polygon p;
    p.points = {Point(cx - rx + cut_x, cy - ry), Point(cx + rx - cut_x, cy - ry), Point(cx + rx, cy - ry + cut_y), Point(cx + rx, cy + ry - cut_y),
                Point(cx + rx - cut_x, cy + ry), Point(cx - rx + cut_x, cy + ry), Point(cx - rx, cy + ry - cut_y), Point(cx - rx, cy - ry + cut_y)};
    return p;
}
// Islands of one to four millimetres, some with a hole, some overlapping, now and then a sliver that
// a negative offset removes.
static ExPolygons islands(Lcg &g, int n)
{
    ExPolygons es;
    for (int i = 0; i < n; ++i) {
        coord_t cx = g.range(-600000, 600000), cy = g.range(-600000, 600000);
        if (g.range(0, 5) == 0) {
            es.emplace_back(rect(cx, cy, cx + g.range(100000, 500000), cy + g.range(8000, 30000)));
            continue;
        }
        coord_t rx = g.range(100000, 400000), ry = g.range(100000, 400000);
        ExPolygon e(blob(g, cx, cy, rx, ry));
        if (g.range(0, 1) == 0) {
            Polygon hole = blob(g, cx + g.range(-rx / 6, rx / 6), cy + g.range(-ry / 6, ry / 6), rx / 3, ry / 3);
            hole.reverse();
            e.holes.emplace_back(hole);
        }
        es.emplace_back(e);
    }
    return es;
}

using KeyValues = std::vector<std::pair<std::string, std::string>>;

static std::string KV(const KeyValues &kvs)
{
    std::string o;
    for (const auto &kv : kvs) o += "\t" + kv.first + "=" + kv.second;
    return o;
}

// The keys of a case over the defaults. extruder_type is set by value: the copy of the default that
// full_print_config() holds has no map of names, and deserialize() of it reads through a null pointer.
static void set_keys(DynamicPrintConfig &full, const KeyValues &kvs)
{
    for (const auto &kv : kvs) {
        if (kv.first == "extruder_type") {
            ConfigOptionEnumsGeneric *opt = full.option<ConfigOptionEnumsGeneric>(kv.first, true);
            opt->values.clear();
            size_t at = 0;
            while (at <= kv.second.size()) {
                size_t end = kv.second.find(',', at);
                if (end == std::string::npos) end = kv.second.size();
                opt->values.push_back(kv.second.substr(at, end - at) == "Bowden" ? int(etBowden) : int(etDirectDrive));
                at = end + 1;
            }
            continue;
        }
        full.set_deserialize_strict(kv.first, kv.second);
    }
}

// ---------------------------------------------------------------------------------------------
// The two filters of the overhang degrees
// ---------------------------------------------------------------------------------------------

static ExtrusionPaths random_paths(Lcg &g)
{
    static const ExtrusionRole roles[] = {erPerimeter, erExternalPerimeter, erPerimeter, erExternalPerimeter,
                                          erPerimeter, erOverhangPerimeter, erInternalInfill, erSupportMaterial};
    const int      n = (int) g.range(0, 9);
    ExtrusionPaths paths;
    Point          cur(g.range(-2000000, 2000000), g.range(-2000000, 2000000));
    // Runs of one degree, as the detection leaves them, in half of the cases.
    const bool runs   = g.range(0, 1) == 0;
    double     degree = 0.;
    for (int i = 0; i < n; ++i) {
        ExtrusionPath p(roles[g.range(0, 7)]);
        if (!runs || i == 0 || g.range(0, 2) == 0) {
            const long long k = g.range(0, 13);
            degree            = k <= 10 ? double(k) : (k == 11 ? 2.5 : (k == 12 ? 7.9 : 0.3));
        }
        p.overhang_degree = degree;
        if (g.range(0, 3) == 0) cur = Point(g.range(-2000000, 2000000), g.range(-2000000, 2000000));
        const int np = (int) g.range(2, 4);
        p.polyline.points.push_back(cur);
        for (int j = 1; j < np; ++j) {
            const coord_t span = g.range(0, 2) != 0 ? g.range(500, 40000) : g.range(60000, 400000);
            coord_t       dx = g.range(-span, span), dy = g.range(-span, span);
            if (dx == 0 && dy == 0) dx = span;
            cur = Point(cur.x() + dx, cur.y() + dy);
            p.polyline.points.push_back(cur);
        }
        paths.push_back(p);
    }
    return paths;
}

static void run_filters()
{
    Lcg g(0x9e3779b97f4a7c15ULL);
    for (int c = 0; c < 400; ++c) {
        ExtrusionPaths    paths = random_paths(g);
        const std::string in    = PATHS(paths);
        lowpass_filter_by_paths_overhang_degree(paths);
        out("lowpass " + in + " => " + PATHS(paths));
    }
    for (int c = 0; c < 400; ++c) {
        ExtrusionPaths    paths = random_paths(g);
        const std::string in    = PATHS(paths);
        smooth_overhang_level(paths);
        out("smooth " + in + " => " + PATHS(paths));
    }

    // A short open wall can wrap repeatedly inside the 6.5 mm filter window
    // (PerimeterGenerator.cpp:548-580). Coordinates are relative to its start.
    const Points points{Point(0, 0), Point(-1980, 1806), Point(-4399, 4011), Point(-6818, 6217), Point(-8836, 8058)};
    ExtrusionPaths paths;
    for (size_t i = 1; i < points.size(); ++i) {
        ExtrusionPath path(erExternalPerimeter);
        path.overhang_degree = 3.0 + 0.25 * double(i - 1);
        path.polyline.points = {points[i - 1], points[i]};
        paths.push_back(path);
    }
    const std::string in = PATHS(paths);
    smooth_overhang_level(paths);
    out("smooth " + in + " => " + PATHS(paths));
}

// ---------------------------------------------------------------------------------------------
// to_polygons_with_flag, PerimeterGeneratorLoop
// ---------------------------------------------------------------------------------------------

static void run_flags()
{
    Lcg g(0x2545f4914f6cdd1dULL);
    for (int c = 0; c < 60; ++c) {
        const coord_t rx = g.range(100000, 400000), ry = g.range(100000, 400000);
        ExPolygon     e(blob(g, g.range(-600000, 600000), g.range(-600000, 600000), rx, ry));
        const int     nh = (int) g.range(0, 3);
        for (int i = 0; i < nh; ++i) {
            Polygon hole = rect(-rx / 8, -ry / 8, rx / 8, ry / 8);
            hole.reverse();
            e.holes.emplace_back(hole);
        }
        const bool       contour_flag = g.range(0, 1) == 0;
        std::vector<int> holes_flag, flags_out;
        const int        nf = (int) g.range(0, 4);
        for (int i = 0; i < nf; ++i) holes_flag.push_back((int) g.range(-1, 5));
        const int npre = (int) g.range(0, 2);
        for (int i = 0; i < npre; ++i) flags_out.push_back((int) g.range(7, 9));
        std::string in = EXPOLY(e) + " " + hB(contour_flag) + " " + hI(nf);
        for (int f : holes_flag) in += " " + hI(f);
        in += " " + hI(npre);
        for (int f : flags_out) in += " " + hI(f);
        const Polygons polygons = to_polygons_with_flag(e, contour_flag, holes_flag, flags_out);
        std::string    res      = POLYS(polygons) + " " + hI((long long) flags_out.size());
        for (int f : flags_out) res += " " + hI(f);
        out("flags " + in + " => " + res);
    }
    for (int c = 0; c < 40; ++c) {
        const unsigned short   depth      = (unsigned short) g.range(0, 3);
        const bool             is_contour = g.range(0, 2) != 0;
        PerimeterGeneratorLoop loop(rect(0, 0, 1000, 1000), depth, is_contour);
        const int              nc = (int) g.range(0, 3);
        std::string            in = hI(depth) + " " + hB(is_contour) + " " + hI(nc);
        for (int i = 0; i < nc; ++i) {
            const bool child_is_contour = g.range(0, 3) == 0;
            loop.children.emplace_back(rect(0, 0, 500, 500), (unsigned short) (depth + 1), child_is_contour);
            in += " " + hB(child_is_contour);
        }
        out("loop " + in + " => " + hB(loop.is_external()) + " " + hB(loop.is_internal_contour()));
    }
}

// ---------------------------------------------------------------------------------------------
// What reads the configs of a PerimeterGenerator
// ---------------------------------------------------------------------------------------------

struct CfgCase
{
    bool      multi_nozzle_diameter_mode;
    KeyValues kvs;
};

static const std::vector<CfgCase> &cfg_cases()
{
    static const std::string two_variants  = "Direct Drive Standard;Direct Drive High Flow";
    static const std::string four_variants = "Direct Drive Standard;Direct Drive High Flow;Bowden Standard;Bowden High Flow";
    static const std::vector<CfgCase> cases = {
        {false, {}},
        {false, {{"wall_filament", "2"}, {"nozzle_diameter", "0.4,0.6"}, {"wall_loops", "3"}}},
        {true, {{"wall_filament", "2"}, {"nozzle_diameter", "0.4,0.6"}, {"wall_loops", "3"}}},
        {true, {{"wall_filament", "1"}, {"nozzle_diameter", "0.2"}, {"wall_loops", "3"}}},
        {true, {{"wall_filament", "3"}, {"nozzle_diameter", "0.2,0.4,0.8"}, {"filament_diameter_map", "0.4;0.2;0.5"}, {"wall_loops", "6"}}},
        {true, {{"wall_filament", "2"}, {"nozzle_diameter", "0.2,0.4,0.8"}, {"filament_diameter_map", "0.4;0.8;0.2"}, {"wall_loops", "7"}}},
        {false, {{"wall_filament", "2"}, {"nozzle_diameter", "0.4,0.6"}, {"filament_diameter_map", "0.2;;0.8"}}},
        {true, {{"wall_filament", "0"}, {"nozzle_diameter", "0.6,0.4"}, {"wall_loops", "1"}}},
        {false, {{"enable_overhang_speed", "0"}}},
        {false,
         {{"wall_filament", "2"}, {"filament_volume_map", "0,1"}, {"printer_extruder_id", "1,1"}, {"printer_extruder_variant", two_variants}, {"enable_overhang_speed", "1,0"}}},
        {false,
         {{"wall_filament", "1"}, {"filament_volume_map", "0,1"}, {"printer_extruder_id", "1,1"}, {"printer_extruder_variant", two_variants}, {"enable_overhang_speed", "1,0"}}},
        {false,
         {{"wall_filament", "3"},
          {"filament_map", "1,2,2"},
          {"extruder_type", "Direct Drive,Bowden"},
          {"filament_volume_map", "0,0,1"},
          {"printer_extruder_id", "1,1,2,2"},
          {"printer_extruder_variant", four_variants},
          {"enable_overhang_speed", "0,0,0,1"}}},
        {false,
         {{"wall_filament", "2"},
          {"filament_map", "1,2,2"},
          {"extruder_type", "Direct Drive,Bowden"},
          {"filament_volume_map", "0,0,1"},
          {"printer_extruder_id", "1,1,2,2"},
          {"printer_extruder_variant", four_variants},
          {"enable_overhang_speed", "0,0,0,1"}}},
        // The variant of the filament is not among those of the process: the first row of its extruder.
        {false,
         {{"wall_filament", "2"},
          {"filament_map", "1,2"},
          {"extruder_type", "Direct Drive,Bowden"},
          {"filament_volume_map", "0,1"},
          {"printer_extruder_id", "1,2"},
          {"printer_extruder_variant", "Direct Drive Standard;Bowden Standard"},
          {"enable_overhang_speed", "0,1"}}},
        {false, {{"wall_filament", "2"}, {"override_process_overhang_speed", "0,1"}, {"filament_enable_overhang_speed", "1,0"}, {"enable_overhang_speed", "1"}}},
        {false, {{"wall_filament", "1"}, {"override_process_overhang_speed", "1,0"}, {"filament_enable_overhang_speed", "1,0"}, {"enable_overhang_speed", "0"}}},
        {false, {{"wall_filament", "2"}, {"override_process_overhang_speed", "0,0"}, {"filament_enable_overhang_speed", "0,0"}, {"enable_overhang_speed", "1"}}},
        // A filament mapped to no extruder prints with the first one.
        {false,
         {{"wall_filament", "2"}, {"filament_map", "1,0"}, {"filament_volume_map", "0,1"}, {"printer_extruder_id", "1,1"}, {"printer_extruder_variant", two_variants}, {"enable_overhang_speed", "1,0"}}},
        {false, {{"fuzzy_skin", "none"}}},
        {false, {{"fuzzy_skin", "external"}}},
        {false, {{"fuzzy_skin", "all"}}},
        {false, {{"fuzzy_skin", "allwalls"}}},
        {false, {{"fuzzy_skin", "disabled_fuzzy"}}},
    };
    return cases;
}

static void run_configs()
{
    Lcg g(0xd1b54a32d192ed03ULL);
    static const double dist_widths[]  = {0.42, 0.45, 0.4, 0.5, 0.62, 0.8, 1.0 / 3.0, double(0.45f)};
    static const float  lower_widths[] = {0.42f, 0.45f, 0.62f};
    for (const CfgCase &cc : cfg_cases()) {
        out("cfg\t" + hB(cc.multi_nozzle_diameter_mode) + KV(cc.kvs));
        DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
        set_keys(full, cc.kvs);
        PrintConfig       print_config;
        PrintRegionConfig region_config;
        PrintObjectConfig object_config;
        print_config.apply(full, true);
        region_config.apply(full, true);
        object_config.apply(full, true);

        SurfaceCollection         slices, fill_surfaces;
        ExtrusionEntityCollection loops, gap_fill;
        ExPolygons                fill_no_overlap;
        std::vector<LoopNode>     loop_nodes;
        PerimeterGenerator pg(&slices, 0.2, Flow(0.45f, 0.2f, 0.4f), &region_config, &object_config, &print_config, 0.4f, cc.multi_nozzle_diameter_mode, false, &loops,
                              &gap_fill, &fill_surfaces, &fill_no_overlap, &loop_nodes);

        out("overhang_speed => " + hB(is_enable_overhang_speed(pg)));

        // fuzzy_skin_allows_overhang_slowdown reads whether there are perimeter regions.
        PrintRegion      a_region(region_config);
        LayerRegion      a_layer_region(nullptr, &a_region);
        PerimeterRegions none, one;
        one.emplace_back(a_layer_region);
        pg.perimeter_regions = &none;
        out("fuzzy_slowdown 1 => " + hB(fuzzy_skin_allows_overhang_slowdown(pg)));
        pg.perimeter_regions = &one;
        out("fuzzy_slowdown 0 => " + hB(fuzzy_skin_allows_overhang_slowdown(pg)));

        out("wall_loops => " + hI(pg.effective_wall_loops()));
        for (double width : dist_widths) {
            const std::pair<double, double> b = pg.dist_boundary(width);
            out("dist_boundary " + hD(width) + " => " + hD(b.first) + " " + hD(b.second));
        }
        for (int i = 0; i < 4; ++i) {
            const float      width = lower_widths[i % 3];
            const ExPolygons lower = islands(g, (int) g.range(1, 4));
            pg.lower_slices        = i == 3 ? nullptr : &lower;
            const std::vector<Polygons> series = pg.generate_lower_polygons_series(width);
            std::string                 res    = hI((long long) series.size());
            for (const Polygons &ps : series) res += " " + POLYS(ps);
            out("lower_series " + hB(i != 3) + " " + hF(width) + " " + EXPOLYS(lower) + " => " + res);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// PerimeterRegion
// ---------------------------------------------------------------------------------------------

static void run_regions()
{
    Lcg g(0x94d049bb133111ebULL);
    // The configs of the regions: the nine keys has_compatible_perimeter_regions compares, one changed
    // in each, and one that changes other keys only.
    static const std::vector<KeyValues> variants = {
        {},
        {{"fuzzy_skin", "external"}},
        {{"fuzzy_skin_thickness", "0.5"}},
        {{"fuzzy_skin_point_distance", "0.6"}},
        {{"fuzzy_skin_first_layer", "1"}},
        {{"fuzzy_skin_noise_type", "perlin"}},
        {{"fuzzy_skin_scale", "2.5"}},
        {{"fuzzy_skin_octaves", "7"}},
        {{"fuzzy_skin_persistence", "0.25"}},
        {{"fuzzy_skin_mode", "combined"}},
        {{"wall_loops", "5"}, {"sparse_infill_density", "33%"}},
        {{"fuzzy_skin", "external"}, {"wall_loops", "4"}},
    };
    std::vector<PrintRegionConfig> configs;
    for (size_t i = 0; i < variants.size(); ++i) {
        out("rcfg\t" + hI((long long) i) + KV(variants[i]));
        DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
        set_keys(full, variants[i]);
        PrintRegionConfig region_config;
        region_config.apply(full, true);
        configs.emplace_back(region_config);
    }
    std::vector<std::unique_ptr<PrintRegion>> regions;
    for (const PrintRegionConfig &c : configs) regions.emplace_back(new PrintRegion(c));
    auto region_index = [&regions](const PrintRegion *r) -> long long {
        for (size_t i = 0; i < regions.size(); ++i)
            if (regions[i].get() == r) return (long long) i;
        return -1;
    };

    for (size_t i = 0; i < configs.size(); ++i)
        for (size_t j = 0; j < configs.size(); ++j)
            out("compat " + hI((long long) i) + " " + hI((long long) j) + " => " + hB(PerimeterRegion::has_compatible_perimeter_regions(configs[i], configs[j])));

    // The constructor: the expolygons of the slices of a LayerRegion and their box. A contour of no
    // area has no box.
    for (int c = 0; c < 40; ++c) {
        ExPolygons es = islands(g, (int) g.range(0, 3));
        if (c % 5 == 1) {
            ExPolygon flat;
            flat.contour.points = {Point(-900000, 0), Point(900000, 0)};
            es.insert(es.begin() + g.range(0, (long long) es.size()), flat);
        }
        if (c % 7 == 3) es.emplace_back();
        const size_t r = (size_t) g.range(0, (long long) regions.size() - 1);
        LayerRegion  layer_region(nullptr, regions[r].get());
        for (const ExPolygon &e : es) layer_region.slices.surfaces.emplace_back(stInternal, e);
        const PerimeterRegion pr(layer_region);
        out("region_new " + hI((long long) r) + " " + EXPOLYS(es) + " => " + hI(region_index(pr.region)) + " " + EXPOLYS(pr.expolygons) + " " + BBOX(pr.bbox));
    }

    // merge_compatible_perimeter_regions over runs of regions that are compatible or not.
    static const size_t picks[] = {0, 10, 0, 10, 1, 11, 1, 2, 0, 5};
    for (int c = 0; c < 80; ++c) {
        const int        n = (int) g.range(0, 7);
        PerimeterRegions prs;
        std::string      in = hI(n);
        for (int i = 0; i < n; ++i) {
            const size_t     r  = picks[g.range(0, 9)];
            const ExPolygons es = islands(g, (int) g.range(0, 3));
            LayerRegion      layer_region(nullptr, regions[r].get());
            for (const ExPolygon &e : es) layer_region.slices.surfaces.emplace_back(stInternal, e);
            prs.emplace_back(layer_region);
            in += " " + hI((long long) r) + " " + EXPOLYS(es);
        }
        PerimeterRegion::merge_compatible_perimeter_regions(prs);
        std::string res = hI((long long) prs.size());
        for (const PerimeterRegion &pr : prs) res += " " + hI(region_index(pr.region)) + " " + EXPOLYS(pr.expolygons) + " " + BBOX(pr.bbox);
        out("merge " + in + " => " + res);
    }
}

// ---------------------------------------------------------------------------------------------
// detect_brigde_wall_arachne
// ---------------------------------------------------------------------------------------------

// The paths with everything thick_polyline_to_multi_path sets.
static std::string XPATHS(const ExtrusionPaths &paths)
{
    std::string o = hI((long long) paths.size());
    for (const ExtrusionPath &p : paths) {
        o += " " + hI((int) p.role()) + " " + hD(p.mm3_per_mm) + " " + hF(p.width) + " " + hF(p.height) + " " + hD(p.overhang_degree) + " " +
             hI((long long) p.polyline.points.size());
        for (const Point &pt : p.polyline.points) o += " " + PT(pt);
    }
    return o;
}
static std::string ZPATHS(const ClipperLib_Z::Paths &paths)
{
    std::string o = hI((long long) paths.size());
    for (const ClipperLib_Z::Path &path : paths) {
        o += " " + hI((long long) path.size());
        for (const ClipperLib_Z::IntPoint &pt : path) o += " " + hI(pt.x()) + " " + hI(pt.y()) + " " + hI(pt.z());
    }
    return o;
}

// A flow is written as what makes it: "1 dmr nozzle" for Flow::bridging_flow(dmr, nozzle),
// "0 width height nozzle" for Flow(width, height, nozzle).
struct FlowCase
{
    Flow        flow;
    std::string text;
};
static FlowCase random_flow(Lcg &g, bool bridge)
{
    static const float widths[]  = {0.42f, 0.45f, 0.62f, 0.4f};
    static const float heights[] = {0.2f, 0.12f, 0.28f};
    static const float nozzles[] = {0.4f, 0.6f, 0.2f};
    const float nozzle = nozzles[g.range(0, 2)];
    if (bridge) return {Flow::bridging_flow(nozzle, nozzle), "1 " + hF(nozzle) + " " + hF(nozzle)};
    const float width = widths[g.range(0, 3)], height = heights[g.range(0, 2)];
    return {Flow(width, height, nozzle), "0 " + hF(width) + " " + hF(height) + " " + hF(nozzle)};
}

// Open paths of two to six points with a width at every point: exactly straight ones (the bridges),
// bent ones, now and then a segment of no length or a step in the width.
static ClipperLib_Z::Paths random_zpaths(Lcg &g, int n)
{
    ClipperLib_Z::Paths paths;
    for (int i = 0; i < n; ++i) {
        ClipperLib_Z::Path path;
        const int          np       = (int) g.range(2, 6);
        const bool         straight = g.range(0, 2) == 0;
        coord_t            x = g.range(-2000000, 2000000), y = g.range(-2000000, 2000000);
        const coord_t      sx = g.range(-30000, 30000), sy = g.range(-30000, 30000);
        coord_t            w = g.range(15000, 90000);
        for (int j = 0; j < np; ++j) {
            const long long k = g.range(0, 5);
            if (k == 0)
                w = g.range(15000, 90000);
            else if (k == 1)
                w += g.range(-3000, 3000);
            if (w < 15000) w = 15000;
            path.emplace_back(x, y, w);
            if (g.range(0, 9) == 0) continue;
            if (straight) {
                const coord_t m = g.range(1, 5);
                x += m * sx;
                y += m * sy;
            } else {
                x += g.range(-60000, 60000);
                y += g.range(-60000, 60000);
            }
        }
        paths.emplace_back(std::move(path));
    }
    return paths;
}

static void run_bridge_walls()
{
    Lcg g(0xbf58476d1ce4e5b9ULL);
    DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
    PrintConfig        print_config;
    PrintRegionConfig  region_config;
    PrintObjectConfig  object_config;
    print_config.apply(full, true);
    region_config.apply(full, true);
    object_config.apply(full, true);
    SurfaceCollection         slices, fill_surfaces;
    ExtrusionEntityCollection loops, gap_fill;
    ExPolygons                fill_no_overlap;
    std::vector<LoopNode>     loop_nodes;
    PerimeterGenerator pg(&slices, 0.2, Flow(0.45f, 0.2f, 0.4f), &region_config, &object_config, &print_config, 0.4f, false, false, &loops, &gap_fill, &fill_surfaces,
                          &fill_no_overlap, &loop_nodes);
    for (int c = 0; c < 300; ++c) {
        const bool          bridge = g.range(0, 2) == 0;
        const FlowCase      fc     = random_flow(g, bridge);
        // A flow of a bridge is taken as it is for an overhang perimeter only.
        const ExtrusionRole role          = bridge || g.range(0, 3) != 0 ? erOverhangPerimeter : erPerimeter;
        ClipperLib_Z::Paths path_overhang = random_zpaths(g, (int) g.range(0, 4));
        ExtrusionPaths      paths;
        detect_brigde_wall_arachne(pg, paths, path_overhang, role, fc.flow);
        out("bridge_wall " + fc.text + " " + hI((int) role) + " " + ZPATHS(path_overhang) + " => " + XPATHS(paths));
    }
}

// ---------------------------------------------------------------------------------------------
// traverse_extrusions
// ---------------------------------------------------------------------------------------------

// The collection: per entity "L" (a loop: its role, its flag) or "M" (a multi path), then its paths,
// each with its flag; the range of the loop nodes.
static std::string COLL(const ExtrusionEntityCollection &coll)
{
    auto paths_of = [](const ExtrusionPaths &paths) {
        std::string o = XPATHS(paths);
        for (const ExtrusionPath &p : paths) o += " " + hI((int) p.get_customize_flag());
        return o;
    };
    std::string o = hI((long long) coll.entities.size());
    for (const ExtrusionEntity *e : coll.entities) {
        if (const ExtrusionLoop *l = dynamic_cast<const ExtrusionLoop *>(e))
            o += " L " + hI((int) l->loop_role()) + " " + hI((int) l->get_customize_flag()) + " " + paths_of(l->paths);
        else if (const ExtrusionMultiPath *m = dynamic_cast<const ExtrusionMultiPath *>(e))
            o += " M " + paths_of(m->paths);
        else
            o += " ?";
    }
    return o + " " + hI((long long) coll.loop_node_range.first) + " " + hI((long long) coll.loop_node_range.second);
}
static std::string NODES(const std::vector<LoopNode> &nodes)
{
    std::string o = hI((long long) nodes.size());
    for (const LoopNode &n : nodes) {
        o += " " + hI(n.node_id) + " " + hI(n.loop_id) + " " + BBOX(n.bbox) + " " + hB(n.node_contour.is_loop) + " " + hI((long long) n.node_contour.pts.size());
        for (size_t i = 0; i < n.node_contour.pts.size(); ++i) o += " " + PT(n.node_contour.pts[i]) + " " + hI((long long) n.node_contour.widths[i]);
    }
    return o;
}

static void run_traverse()
{
    Lcg g(0x6a09e667f3bcc909ULL);
    // Fuzzy skin stays off (its random source cannot be repeated) and there are no perimeter regions.
    static const std::vector<CfgCase> cases = {
        {false, {}},
        {false, {{"z_direction_outwall_speed_continuous", "1"}}},
        {false, {{"enable_overhang_speed", "0"}}},
        {false, {{"detect_overhang_wall", "0"}, {"z_direction_outwall_speed_continuous", "1"}}},
        {false, {{"raft_layers", "3"}}},
        {false, {{"fuzzy_skin", "disabled_fuzzy"}, {"z_direction_outwall_speed_continuous", "1"}, {"outer_wall_line_width", "1.3"}}},
        {true, {{"z_direction_outwall_speed_continuous", "1"}, {"outer_wall_line_width", "0.42"}, {"nozzle_diameter", "0.8"}}},
    };
    for (const CfgCase &cc : cases) {
        out("cfg\t" + hB(cc.multi_nozzle_diameter_mode) + KV(cc.kvs));
        DynamicPrintConfig full = DynamicPrintConfig::full_print_config();
        set_keys(full, cc.kvs);
        PrintConfig       print_config;
        PrintRegionConfig region_config;
        PrintObjectConfig object_config;
        print_config.apply(full, true);
        region_config.apply(full, true);
        object_config.apply(full, true);
        const PerimeterRegions none;

        for (int c = 0; c < 40; ++c) {
            SurfaceCollection         slices, fill_surfaces;
            ExtrusionEntityCollection loops, gap_fill;
            ExPolygons                fill_no_overlap;
            std::vector<LoopNode>     loop_nodes;
            // Nodes of the islands before this one.
            const int nodes_before = (int) g.range(0, 2);
            for (int i = 0; i < nodes_before; ++i) {
                LoopNode node;
                node.node_id              = i;
                node.loop_id              = i;
                node.node_contour.is_loop = false;
                loop_nodes.push_back(node);
            }
            const FlowCase perimeter_flow = random_flow(g, false), ext_perimeter_flow = random_flow(g, false);
            const FlowCase overhang_flow  = random_flow(g, g.range(0, 3) != 0);
            PerimeterGenerator pg(&slices, 0.2, perimeter_flow.flow, &region_config, &object_config, &print_config, 0.4f, cc.multi_nozzle_diameter_mode, false, &loops,
                                  &gap_fill, &fill_surfaces, &fill_no_overlap, &loop_nodes);
            pg.ext_perimeter_flow    = ext_perimeter_flow.flow;
            pg.overhang_flow         = overhang_flow.flow;
            pg.perimeter_regions     = &none;
            pg.layer_id              = (int) g.range(0, 6);
            pg.starts_at_object_base = g.range(0, 7) == 0;
            pg.slice_z               = 0.2 * (pg.layer_id + 1);
            // The grown lower slices: islands that the walls below cross.
            pg.m_lower_slices_polygons.clear();
            const int n_lower = (int) g.range(0, 3);
            for (int i = 0; i < n_lower; ++i) {
                const coord_t rx = g.range(150000, 500000), ry = g.range(150000, 500000);
                pg.m_lower_slices_polygons.emplace_back(blob(g, g.range(-400000, 400000), g.range(-400000, 400000), rx, ry));
                if (g.range(0, 3) == 0) {
                    Polygon hole = blob(g, pg.m_lower_slices_polygons.back().points.front().x() + rx, pg.m_lower_slices_polygons.back().points.front().y() + ry, rx / 3, ry / 3);
                    hole.reverse();
                    pg.m_lower_slices_polygons.emplace_back(hole);
                }
            }

            // The extrusions: closed walls around the origin, in either direction, and open ones.
            const int                          n = (int) g.range(1, 4);
            std::vector<Arachne::ExtrusionLine> lines;
            std::vector<bool>                   contours;
            for (int i = 0; i < n; ++i) {
                const size_t inset_idx = (size_t) g.range(0, 2);
                const bool   closed    = g.range(0, 3) != 0;
                Arachne::ExtrusionLine line(inset_idx, !closed && g.range(0, 1) == 0, closed);
                const bool   flagged = g.range(0, 5) == 0;
                coord_t      w       = g.range(30000, 60000);
                Points       pts;
                if (closed) {
                    Polygon ring = blob(g, g.range(-300000, 300000), g.range(-300000, 300000), g.range(100000, 450000), g.range(100000, 450000));
                    if (g.range(0, 1) == 0) ring.reverse();
                    pts = ring.points;
                    pts.push_back(pts.front());
                } else {
                    Point cur(g.range(-600000, 600000), g.range(-600000, 600000));
                    const int np = (int) g.range(2, 7);
                    for (int j = 0; j < np; ++j) {
                        pts.push_back(cur);
                        cur = Point(cur.x() + g.range(-250000, 250000), cur.y() + g.range(-250000, 250000));
                    }
                }
                for (size_t j = 0; j < pts.size(); ++j) {
                    if (g.range(0, 3) == 0) w = g.range(30000, 60000);
                    // A closed line ends with the junction it starts with.
                    const coord_t wj = closed && j + 1 == pts.size() ? line.junctions.front().w : w;
                    line.junctions.emplace_back(pts[j], wj, inset_idx, flagged && g.range(0, 5) != 0);
                }
                lines.emplace_back(line);
                contours.push_back(g.range(0, 1) == 0);
            }
            std::vector<PerimeterGeneratorArachneExtrusion> pg_extrusions;
            for (size_t i = 0; i < lines.size(); ++i) pg_extrusions.push_back({&lines[i], bool(contours[i]), false});

            std::string in = hI(pg.layer_id) + " " + hB(pg.starts_at_object_base) + " " + hD(pg.slice_z) + " " + perimeter_flow.text + " " + ext_perimeter_flow.text + " " +
                             overhang_flow.text + " " + hI(nodes_before) + " " + POLYS(pg.m_lower_slices_polygons) + " " + hI((long long) lines.size());
            for (size_t i = 0; i < lines.size(); ++i) {
                in += " " + hI((long long) lines[i].inset_idx) + " " + hB(lines[i].is_odd) + " " + hB(lines[i].is_closed) + " " + hB(contours[i]) + " " +
                      hI((long long) lines[i].junctions.size());
                for (const Arachne::ExtrusionJunction &j : lines[i].junctions) in += " " + PT(j.p) + " " + hI(j.w) + " " + hB(j.hole_compensation_flag);
            }
            const ExtrusionEntityCollection coll = traverse_extrusions(pg, pg_extrusions);
            out("traverse " + in + " => " + COLL(coll) + " " + NODES(loop_nodes));
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The whole of process_classic and process_arachne: LayerRegion::make_perimeters of a layer
// ---------------------------------------------------------------------------------------------

// Every long result of a "proc" line in full (the third argument "full"), or "#" and its hash.
static bool g_full = false;
static std::string fnv(const std::string &s)
{
    uint64_t h = 1469598103934665603ULL;
    for (unsigned char c : s) {
        h ^= c;
        h *= 1099511628211ULL;
    }
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long) h);
    return b;
}
static std::string part(const std::string &body) { return g_full ? body : "#" + fnv(body); }

// A path of a wall with what the G-code reads of it.
static std::string WPATHS(const ExtrusionPaths &paths)
{
    std::string o = hI((long long) paths.size());
    for (const ExtrusionPath &p : paths) {
        o += " " + hI((int) p.role()) + " " + hD(p.mm3_per_mm) + " " + hF(p.width) + " " + hF(p.height) + " " + hD(p.overhang_degree) + " " + hI(p.curve_degree) + " " +
             hI((int) p.get_customize_flag()) + " " + hI((long long) p.polyline.points.size());
        for (const Point &pt : p.polyline.points) o += " " + PT(pt);
    }
    return o;
}
// The entities of a collection and of the collections in it, in order: "L role flag paths" (a loop),
// "M paths" (a multi path), "P paths" (a path).
static void leaves(const ExtrusionEntityCollection &coll, std::string &o, long long &n)
{
    for (const ExtrusionEntity *e : coll.entities) {
        if (const ExtrusionEntityCollection *c = dynamic_cast<const ExtrusionEntityCollection *>(e))
            leaves(*c, o, n);
        else if (const ExtrusionLoop *l = dynamic_cast<const ExtrusionLoop *>(e)) {
            o += " L " + hI((int) l->loop_role()) + " " + hI((int) l->get_customize_flag()) + " " + WPATHS(l->paths);
            ++n;
        } else if (const ExtrusionMultiPath *m = dynamic_cast<const ExtrusionMultiPath *>(e)) {
            o += " M " + WPATHS(m->paths);
            ++n;
        } else if (const ExtrusionPath *p = dynamic_cast<const ExtrusionPath *>(e)) {
            o += " P " + WPATHS({*p});
            ++n;
        } else
            o += " ?";
    }
}
static std::string LEAVES(const ExtrusionEntityCollection &coll)
{
    std::string o;
    long long   n = 0;
    leaves(coll, o, n);
    return hI(n) + o;
}
// The islands of the walls: per collection in `loops` the number of its entities and the range of its
// loop nodes.
static std::string ISLANDS(const ExtrusionEntityCollection &coll)
{
    std::string o = hI((long long) coll.entities.size());
    for (const ExtrusionEntity *e : coll.entities) {
        if (const ExtrusionEntityCollection *c = dynamic_cast<const ExtrusionEntityCollection *>(e)) {
            std::string t;
            long long   n = 0;
            leaves(*c, t, n);
            o += " " + hI(n) + " " + hI((long long) c->loop_node_range.first) + " " + hI((long long) c->loop_node_range.second);
        } else
            o += " 1 0 0";
    }
    return o;
}

// The loop nodes with their points and their widths apart: a loop of the classic walls has one width.
// The loop of a node is written as its index among the walls of all the islands of `loops`: the index in
// the collection of its island (what the pin stores) plus the number of the walls of the islands before.
static std::string PNODES(const std::vector<LoopNode> &nodes, const ExtrusionEntityCollection &loops)
{
    std::vector<long long> base(nodes.size(), 0);
    long long              before = 0;
    for (const ExtrusionEntity *e : loops.entities) {
        if (const ExtrusionEntityCollection *c = dynamic_cast<const ExtrusionEntityCollection *>(e)) {
            for (size_t k = c->loop_node_range.first; k < c->loop_node_range.second && k < nodes.size(); ++k) base[k] = before;
            std::string t;
            long long   n = 0;
            leaves(*c, t, n);
            before += n;
        } else
            ++before;
    }
    std::string o = hI((long long) nodes.size());
    for (size_t k = 0; k < nodes.size(); ++k) {
        const LoopNode &n = nodes[k];
        o += " " + hI(n.node_id) + " " + hI(n.loop_id + base[k]) + " " + BBOX(n.bbox) + " " + hB(n.node_contour.is_loop) + " " + hI((long long) n.node_contour.pts.size());
        for (const Point &pt : n.node_contour.pts) o += " " + PT(pt);
        o += " " + hI((long long) n.node_contour.widths.size());
        for (const coord_t w : n.node_contour.widths) o += " " + hI((long long) w);
    }
    return o;
}

// The keys the walls of a layer read, written for every case as the pin holds them after Print::apply.
static const char *const proc_keys[] = {
    // PrintConfig
    "nozzle_diameter", "resolution", "enable_arc_fitting", "z_direction_outwall_speed_continuous", "spiral_mode", "initial_layer_line_width",
    "initial_layer_infill_line_width", "enable_overhang_speed", "initial_layer_print_height",
    // PrintObjectConfig
    "layer_height", "line_width", "wall_generator", "top_one_wall_type", "top_area_threshold", "only_one_wall_first_layer", "brim_type", "brim_width", "min_feature_size",
    "min_bead_width", "wall_transition_length", "wall_transition_angle", "wall_transition_filter_deviation", "wall_distribution_count", "thick_bridges", "raft_layers",
    // PrintRegionConfig
    "wall_loops", "wall_filament", "sparse_infill_filament", "solid_infill_filament", "outer_wall_line_width", "inner_wall_line_width", "sparse_infill_line_width",
    "internal_solid_infill_line_width", "top_surface_line_width", "wall_sequence", "precise_outer_wall", "gap_infill_speed", "filter_out_gap_fill", "alternate_extra_wall",
    "detect_thin_wall", "detect_overhang_wall", "fuzzy_skin", "sparse_infill_density", "infill_wall_overlap", "counterbore_hole_bridging", "bridge_angle",
    "bridge_flow", "bottom_shell_layers", "bottom_shell_thickness", "top_shell_layers", "smooth_speed_discontinuity_area", "smooth_coefficient",
};

static Polygon circle(coord_t cx, coord_t cy, double r, int n, double phase)
{
    Polygon p;
    for (int i = 0; i < n; ++i) {
        const double a = phase + 2. * M_PI * i / n;
        p.points.emplace_back(Point(cx + coord_t(std::lround(r * std::cos(a))), cy + coord_t(std::lround(r * std::sin(a)))));
    }
    return p;
}
static ExPolygon holed(Polygon contour, Polygon hole)
{
    ExPolygon e(contour);
    hole.make_clockwise();
    e.holes.emplace_back(hole);
    return e;
}
static coord_t mm(Lcg &g, double lo, double hi) { return g.range((long long) (lo * 100000.), (long long) (hi * 100000.)); }

// One island of a kind, around (cx, cy).
static ExPolygons island(Lcg &g, int kind, coord_t cx, coord_t cy)
{
    ExPolygons es;
    switch (kind) {
    case 0: { // a rectangle
        const coord_t w = mm(g, 8., 25.), h = mm(g, 5., 20.);
        es.emplace_back(rect(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2));
        break;
    }
    case 1: { // a rectangle with a round hole
        const coord_t w = mm(g, 12., 25.), h = mm(g, 12., 20.);
        es.emplace_back(holed(rect(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2),
                              circle(cx + mm(g, -1., 1.), cy + mm(g, -1., 1.), double(mm(g, 1.2, 4.)), (int) g.range(16, 40), 0.1 * g.range(0, 30))));
        break;
    }
    case 2: { // a ring, down to a wall that two loops do not fit into
        const double r = double(mm(g, 5., 11.));
        const double t = double(mm(g, 0.5, 3.));
        const int    n = (int) g.range(24, 64);
        es.emplace_back(holed(circle(cx, cy, r, n, 0.05 * g.range(0, 60)), circle(cx + mm(g, -0.2, 0.2), cy, r - t, n - (int) g.range(0, 8), 0.05 * g.range(0, 60))));
        break;
    }
    case 3: { // an L
        const coord_t w = mm(g, 10., 22.), h = mm(g, 10., 22.), a = mm(g, 1.5, 6.), b = mm(g, 1.5, 6.);
        Polygon       p;
        p.points = {Point(cx - w / 2, cy - h / 2), Point(cx + w / 2, cy - h / 2), Point(cx + w / 2, cy - h / 2 + b), Point(cx - w / 2 + a, cy - h / 2 + b),
                    Point(cx - w / 2 + a, cy + h / 2), Point(cx - w / 2, cy + h / 2)};
        es.emplace_back(p);
        break;
    }
    case 4: { // a strip, from thinner than a wall to four walls
        static const double hs[] = {0.25, 0.38, 0.5, 0.7, 0.86, 0.95, 1.2, 1.5, 1.74, 2.1};
        const coord_t w = mm(g, 6., 20.), h = coord_t(hs[g.range(0, 9)] * 100000.) + g.range(-900, 900);
        es.emplace_back(rect(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2));
        break;
    }
    case 5: { // a wedge with a sharp tip
        const coord_t l = mm(g, 10., 22.), b = mm(g, 1.5, 5.);
        Polygon       p;
        p.points = {Point(cx - l / 2, cy - b / 2), Point(cx + l / 2, cy + coord_t(g.range(-3000, 3000))), Point(cx - l / 2, cy + b / 2)};
        es.emplace_back(p);
        break;
    }
    case 6: { // two plates joined by a neck
        const coord_t w = mm(g, 5., 9.), h = mm(g, 6., 12.), nl = mm(g, 2., 6.), nh = mm(g, 0.3, 1.6) / 2;
        Polygon       p;
        p.points = {Point(cx - nl / 2 - w, cy - h / 2), Point(cx - nl / 2, cy - h / 2), Point(cx - nl / 2, cy - nh), Point(cx + nl / 2, cy - nh),
                    Point(cx + nl / 2, cy - h / 2), Point(cx + nl / 2 + w, cy - h / 2), Point(cx + nl / 2 + w, cy + h / 2), Point(cx + nl / 2, cy + h / 2),
                    Point(cx + nl / 2, cy + nh),    Point(cx - nl / 2, cy + nh),         Point(cx - nl / 2, cy + h / 2),     Point(cx - nl / 2 - w, cy + h / 2)};
        es.emplace_back(p);
        break;
    }
    case 7: { // a blob with a blob in it
        const coord_t rx = mm(g, 4., 10.), ry = mm(g, 4., 10.);
        ExPolygon     e(blob(g, cx, cy, rx, ry));
        if (g.range(0, 1) == 0) {
            Polygon hole = blob(g, cx + g.range(-rx / 6, rx / 6), cy + g.range(-ry / 6, ry / 6), rx / 3, ry / 3);
            hole.reverse();
            e.holes.emplace_back(hole);
        }
        es.emplace_back(e);
        break;
    }
    case 8: { // a star
        const int    n  = 2 * (int) g.range(4, 9);
        const double ro = double(mm(g, 6., 11.)), ri = ro * (0.35 + 0.05 * g.range(0, 6)), ph = 0.07 * g.range(0, 40);
        Polygon      p;
        for (int i = 0; i < n; ++i) {
            const double a = ph + 2. * M_PI * i / n, r = i % 2 == 0 ? ro : ri;
            p.points.emplace_back(Point(cx + coord_t(std::lround(r * std::cos(a))), cy + coord_t(std::lround(r * std::sin(a)))));
        }
        es.emplace_back(p);
        break;
    }
    default: { // a disc
        es.emplace_back(circle(cx, cy, double(mm(g, 2., 9.)), (int) g.range(12, 72), 0.05 * g.range(0, 60)));
        break;
    }
    }
    return es;
}

// The slices of the layer below or above: `how` 0 the same, 1 smaller all around, 2 moved, 3 one half,
// 4 larger all around, 5 none.
static ExPolygons neighbour(Lcg &g, const ExPolygons &slices, int how)
{
    switch (how) {
    case 0: return slices;
    case 1: return offset_ex(slices, -float(mm(g, 0.1, 1.6)));
    case 2: {
        ExPolygons es = slices;
        const Point d(mm(g, -1.5, 1.5), mm(g, -1.5, 1.5));
        for (ExPolygon &e : es) e.translate(d);
        return es;
    }
    case 3: {
        const BoundingBox bb = get_extents(slices);
        const coord_t     x  = bb.min.x() + (bb.max.x() - bb.min.x()) * g.range(30, 70) / 100;
        return intersection_ex(slices, ExPolygons{ExPolygon(rect(bb.min.x() - 1000, bb.min.y() - 1000, x, bb.max.y() + 1000))});
    }
    case 4: return offset_ex(slices, float(mm(g, 0.1, 1.2)));
    case 7: { // pockets under the islands (a counterbore): a round one, a slot across the island, or both
        Polygons pockets;
        for (const ExPolygon &e : slices) {
            const BoundingBox bb   = get_extents(e);
            const Point       c    = bb.center();
            const int         what = (int) g.range(0, 2);
            if (what != 1)
                pockets.emplace_back(circle(c.x() + mm(g, -2., 2.), c.y() + mm(g, -2., 2.), double(mm(g, 1.5, 4.5)), (int) g.range(12, 48), 0.1 * g.range(0, 30)));
            if (what != 0) {
                const coord_t w = mm(g, 1., 5.);
                const coord_t x = c.x() + mm(g, -3., 3.);
                pockets.emplace_back(rect(x - w / 2, bb.min.y() - 100000, x + w / 2, bb.max.y() + 100000));
            }
        }
        return diff_ex(slices, pockets);
    }
    default: return ExPolygons();
    }
}

struct ProcCase
{
    const char *tag;
    KeyValues   kvs;
};

static void run_process()
{
    // What the cases share: the keys of the process of the fixtures (a 0.16 mm layer, two walls of
    // 0.45 and 0.42 mm, a 0.4 mm nozzle).
    static const KeyValues base = {{"layer_height", "0.16"},
                                   {"initial_layer_print_height", "0.2"},
                                   {"wall_loops", "2"},
                                   {"outer_wall_line_width", "0.42"},
                                   {"inner_wall_line_width", "0.45"},
                                   {"line_width", "0.42"},
                                   {"sparse_infill_line_width", "0.45"},
                                   {"internal_solid_infill_line_width", "0.42"},
                                   {"top_surface_line_width", "0.42"},
                                   {"initial_layer_line_width", "0.5"},
                                   {"initial_layer_infill_line_width", "0.5"},
                                   {"resolution", "0.012"},
                                   {"enable_arc_fitting", "0"},
                                   {"z_direction_outwall_speed_continuous", "1"},
                                   {"sparse_infill_density", "15%"},
                                   {"infill_wall_overlap", "15%"},
                                   {"detect_overhang_wall", "1"},
                                   {"gap_infill_speed", "250"},
                                   {"bottom_shell_layers", "4"},
                                   {"top_shell_layers", "6"},
                                   {"brim_type", "no_brim"},
                                   {"wall_generator", "classic"}};
    static const std::vector<ProcCase> cases = {
        {"classic", {}},
        {"classic_1wall", {{"wall_loops", "1"}}},
        {"classic_3walls", {{"wall_loops", "3"}}},
        {"classic_4walls_outer_inner", {{"wall_loops", "4"}, {"wall_sequence", "outer wall/inner wall"}}},
        {"classic_3walls_inner_outer_inner", {{"wall_loops", "3"}, {"wall_sequence", "inner-outer-inner wall"}}},
        {"classic_thin_walls", {{"detect_thin_wall", "1"}}},
        {"classic_no_overhang", {{"detect_overhang_wall", "0"}}},
        {"classic_top_none", {{"top_one_wall_type", "not apply"}}},
        {"classic_topmost", {{"top_one_wall_type", "topmost"}}},
        {"classic_arc", {{"enable_arc_fitting", "1"}}},
        {"classic_no_z_continuity", {{"z_direction_outwall_speed_continuous", "0"}}},
        {"classic_no_gap_fill", {{"gap_infill_speed", "0"}}},
        {"classic_no_infill", {{"sparse_infill_density", "0%"}}},
        {"classic_precise", {{"precise_outer_wall", "1"}}},
        {"classic_spiral", {{"spiral_mode", "1"}, {"wall_loops", "1"}, {"top_shell_layers", "0"}, {"sparse_infill_density", "0%"}, {"bottom_shell_layers", "2"}}},
        {"classic_alternate", {{"alternate_extra_wall", "1"}}},
        {"classic_first_layer_1wall", {{"only_one_wall_first_layer", "1"}}},
        {"classic_no_overhang_speed", {{"enable_overhang_speed", "0"}}},
        {"classic_same_widths", {{"layer_height", "0.2"}, {"outer_wall_line_width", "0.45"}, {"inner_wall_line_width", "0.45"}}},
        {"classic_filter_gap", {{"filter_out_gap_fill", "1.5"}}},
        {"classic_brim", {{"brim_type", "outer_only"}, {"brim_width", "5"}}},
        {"arachne", {{"wall_generator", "arachne"}}},
        {"arachne_1wall", {{"wall_generator", "arachne"}, {"wall_loops", "1"}}},
        {"arachne_3walls", {{"wall_generator", "arachne"}, {"wall_loops", "3"}}},
        {"arachne_4walls_outer_inner", {{"wall_generator", "arachne"}, {"wall_loops", "4"}, {"wall_sequence", "outer wall/inner wall"}}},
        {"arachne_3walls_inner_outer_inner", {{"wall_generator", "arachne"}, {"wall_loops", "3"}, {"wall_sequence", "inner-outer-inner wall"}}},
        {"arachne_no_overhang", {{"wall_generator", "arachne"}, {"detect_overhang_wall", "0"}}},
        {"arachne_top_none", {{"wall_generator", "arachne"}, {"top_one_wall_type", "not apply"}}},
        {"arachne_topmost", {{"wall_generator", "arachne"}, {"top_one_wall_type", "topmost"}}},
        {"arachne_no_z_continuity", {{"wall_generator", "arachne"}, {"z_direction_outwall_speed_continuous", "0"}}},
        {"arachne_precise", {{"wall_generator", "arachne"}, {"precise_outer_wall", "1"}}},
        {"arachne_arc", {{"wall_generator", "arachne"}, {"enable_arc_fitting", "1"}}},
        {"arachne_first_layer_1wall", {{"wall_generator", "arachne"}, {"only_one_wall_first_layer", "1"}}},
        {"arachne_beads", {{"wall_generator", "arachne"}, {"min_bead_width", "60%"}, {"min_feature_size", "15%"}, {"wall_distribution_count", "2"}, {"wall_loops", "3"}}},
        {"arachne_no_overhang_speed", {{"wall_generator", "arachne"}, {"enable_overhang_speed", "0"}}},
        // PerimeterGenerator::process_no_bridge: the counterbore hole bridging, with a lower layer in every
        // case but the first layer's (`counterbore` in the tag makes the lower slices holed or cut).
        {"classic_counterbore_bridges", {{"counterbore_hole_bridging", "partiallybridge"}}},
        {"classic_counterbore_filled", {{"counterbore_hole_bridging", "sacrificiallayer"}}},
        {"classic_counterbore_filled_angle", {{"counterbore_hole_bridging", "sacrificiallayer"}, {"bridge_angle", "30"}, {"wall_loops", "3"}}},
        {"arachne_counterbore_bridges", {{"wall_generator", "arachne"}, {"counterbore_hole_bridging", "partiallybridge"}}},
        {"arachne_counterbore_filled", {{"wall_generator", "arachne"}, {"counterbore_hole_bridging", "sacrificiallayer"}}},
        {"arachne_counterbore_bridges_angle", {{"wall_generator", "arachne"}, {"counterbore_hole_bridging", "partiallybridge"}, {"bridge_angle", "45"}, {"wall_loops", "3"}}},
    };
    Lcg g(0x3c6ef372fe94f82bULL);
    for (const ProcCase &pc : cases) {
        Model        model;
        ModelObject *o = model.add_object();
        o->add_volume(TriangleMesh(its_make_cube(20., 20., 6.)));
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        set_keys(config, base);
        set_keys(config, pc.kvs);
        Print print;
        print.set_status_callback([](const PrintBase::SlicingStatus &) {});
        print.apply(model, config);
        PrintObject       *obj    = print.m_objects.front();
        const PrintRegion &region = *obj->m_shared_regions->all_regions.front();
        std::string        cfg    = std::string("pcfg\t") + pc.tag;
        for (const char *key : proc_keys) {
            const ConfigBase *holder = region.config().has(key) ? static_cast<const ConfigBase *>(&region.config()) :
                                       obj->config().has(key)   ? static_cast<const ConfigBase *>(&obj->config()) :
                                       print.config().has(key)  ? static_cast<const ConfigBase *>(&print.config()) :
                                                                  nullptr;
            cfg += std::string("\t") + key + "=" + (holder == nullptr ? std::string("?") : holder->opt_serialize(key));
        }
        out(cfg);
        const double layer_height       = obj->config().layer_height.value;
        const double first_layer_height = print.config().initial_layer_print_height.value;

        const int n_layers = 12;
        for (int c = 0; c < n_layers; ++c) {
            // The layer: the first one in two cases of twelve, an odd and an even one among the rest.
            const size_t layer_id = c < 2 ? 0 : (size_t) g.range(1, 9);
            const double height   = layer_id == 0 ? first_layer_height : layer_height;
            const double print_z  = first_layer_height + layer_height * double(layer_id);
            const double slice_z  = print_z - 0.5 * height;
            // The cases of the counterbore hole bridging: pockets in the layer below, and in two layers of
            // twelve a thick ring with a disc in its hole over a smaller layer below (the unsupported band
            // around the hole surrounds another island).
            const bool counterbore = std::strstr(pc.tag, "counterbore") != nullptr;
            const bool nested      = counterbore && (c == 6 || c == 10);
            // One to three islands, apart from each other.
            const int             n_islands = nested ? 2 : c % 4 == 3 ? (int) g.range(2, 3) : 1;
            ExPolygons            slices;
            static const coord_t  xs[] = {-3000000, 0, 3000000};
            if (nested) {
                const coord_t cx = g.range(-200000, 200000), cy = g.range(-200000, 200000);
                const double  ro = double(mm(g, 9., 11.)), ri = double(mm(g, 4., 5.5));
                slices.emplace_back(holed(circle(cx, cy, ro, (int) g.range(32, 64), 0.05 * g.range(0, 60)), circle(cx, cy, ri, (int) g.range(24, 48), 0.05 * g.range(0, 60))));
                slices.emplace_back(circle(cx + mm(g, -0.5, 0.5), cy + mm(g, -0.5, 0.5), double(mm(g, 1.5, 3.)), (int) g.range(12, 32), 0.05 * g.range(0, 60)));
            } else
                for (int i = 0; i < n_islands; ++i) {
                    // The kinds in turn, so every case has every kind.
                    const int kind = n_islands == 1 ? (c + int(&pc - &cases[0])) % 10 : (int) g.range(0, 9);
                    append(slices, island(g, kind, (n_islands == 1 ? 0 : xs[i]) + g.range(-200000, 200000), g.range(-200000, 200000)));
                }
            std::vector<unsigned short> extra;
            for (size_t i = 0; i < slices.size(); ++i) extra.push_back(g.range(0, 7) == 0 ? 1 : 0);

            const int  lower_how = layer_id == 0 ? 6 : nested ? 1 : counterbore && c % 3 != 2 ? 7 : (int) g.range(0, 5);
            const int  upper_how = (int) g.range(0, 6);
            const bool has_lower = lower_how != 6, has_upper = upper_how != 6;
            Layer      lower(layer_id == 0 ? 0 : layer_id - 1, obj, layer_height, print_z - height, slice_z - height);
            Layer      upper(layer_id + 1, obj, layer_height, print_z + layer_height, slice_z + layer_height);
            Layer      layer(layer_id, obj, height, print_z, slice_z);
            if (has_lower) {
                lower.lslices     = neighbour(g, slices, lower_how);
                layer.lower_layer = &lower;
            }
            if (has_upper) {
                upper.lslices     = neighbour(g, slices, upper_how);
                layer.upper_layer = &upper;
            }
            LayerRegion *layerm = layer.add_region(&region);
            for (size_t i = 0; i < slices.size(); ++i) {
                Surface s(stInternal, slices[i]);
                s.extra_perimeters = extra[i];
                layerm->slices.surfaces.emplace_back(s);
            }
            SurfaceCollection     fill_surfaces;
            ExPolygons            fill_no_overlap;
            std::vector<LoopNode> loop_nodes;
            const PerimeterRegions perimeter_regions;
            layerm->make_perimeters(layerm->slices, perimeter_regions, &fill_surfaces, &fill_no_overlap, loop_nodes);

            std::string in = hI((long long) layer_id) + " " + hD(height) + " " + hD(print_z) + " " + hD(slice_z) + " " + hI((long long) slices.size());
            for (size_t i = 0; i < slices.size(); ++i) in += " " + hI(extra[i]) + " " + EXPOLY(slices[i]);
            in += " " + hB(has_lower) + " " + EXPOLYS(lower.lslices) + " " + hB(has_upper) + " " + EXPOLYS(upper.lslices);
            ExPolygons fill_expolygons;
            for (const Surface &s : fill_surfaces.surfaces) fill_expolygons.emplace_back(s.expolygon);
            // The flows the walls were made with, to tell a difference of the flows from one of the walls.
            const Flow  pf = layerm->flow(frPerimeter), ef = layerm->flow(frExternalPerimeter), sf = layerm->flow(frSolidInfill);
            const Flow  of = layerm->bridging_flow(frPerimeter, obj->config().thick_bridges);
            std::string flows;
            for (const Flow *f : {&pf, &ef, &sf, &of}) flows += std::string(flows.empty() ? "" : " ") + hF(f->width()) + " " + hF(f->height()) + " " + hF(f->spacing()) + " " + hD(f->mm3_per_mm());
            out("proc " + in + " => " + flows + " | " + part(LEAVES(layerm->perimeters)) + " | " + part(LEAVES(layerm->thin_fills)) + " | " + part(EXPOLYS(fill_expolygons)) + " | " +
                part(EXPOLYS(fill_no_overlap)) + " | " + part(PNODES(loop_nodes, layerm->perimeters)) + " | " + part(ISLANDS(layerm->perimeters)));
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Layer::make_perimeters on a layer with several regions of a real Print
// ---------------------------------------------------------------------------------------------
// The object has PARAMETER_MODIFIER volumes with options of their own; Print::apply makes a region of
// each distinct config (PrintApply.cpp). A line "lcfg<TAB>tag<TAB>region<TAB>key=value..." holds the
// options of one region as the pin holds them, a line "lgrp <layer> <height> <print_z> <slice_z>
// <regions> (<islands> (<extra> <expolygon>)...)... <has lower> <lower> <has upper> <upper> =>
// (<perimeter entities> <thin fill entities> | <perimeters> | <thin fills> | <fill expolygons> |
// <fill_no_overlap>) per region" one call of the pin's Layer::make_perimeters (Layer.cpp:202-408).
static const char *const group_keys[] = {
    "inner_wall_speed", "outer_wall_speed", "bridge_speed", "override_filament_scarf_seam_setting", "seam_slope_type", "seam_slope_start_height",
    "seam_slope_gap", "seam_slope_min_length", "seam_slope_conditional", "seam_slope_entire_loop", "seam_slope_steps", "seam_slope_inner_walls",
    "initial_layer_flow_ratio", "fuzzy_skin_thickness", "fuzzy_skin_point_distance",
};

struct GroupCase
{
    const char            *tag;
    KeyValues              kvs;
    std::vector<KeyValues> mods;
};

// R935 'gen patches <file>': Layer::make_perimeters (Layer.cpp:201-400) on the layers of an object whose
// regions carry adaptive merge patches (AdaptiveLayerMerge.hpp:25-60), set here by hand in the shapes
// PrintObject::compute_adaptive_merge_groups gives them (PrintObject.cpp:4312-4960): the layers bottom..top-1
// of a band swallowed (group_top false), the top one the group top with the band's height (bands of two layers). Three objects per
// case: a band from layer 1, a band from object layer 0, and a band from layer 0 whose top layer has no
// slices of its own (has_first_band_source, Layer.cpp:233-239). One line 'lobj <K>' per object, then K
// lines 'lpat <layer inputs> => <results per region as lgrp>'.
static bool g_patches = false;
static void run_layer_patches(PrintObject *obj, const std::vector<const PrintRegion *> &regions, const std::string &tag)
{
    Lcg          g(0x9e3779b97f4a7c15ULL ^ (uint64_t) std::hash<std::string>()(tag));
    const size_t K                  = 4;
    const double layer_height       = obj->config().layer_height.value;
    const double first_layer_height = obj->print()->config().initial_layer_print_height.value;
    for (int kind = 0; kind < 3; ++kind) {
        for (Layer *l : obj->m_layers) delete l;
        obj->m_layers.clear();
        // The regions' islands, one shape per region for the whole object (one region in six without any).
        std::vector<ExPolygons> shapes(regions.size());
        for (size_t r = 0; r < regions.size(); ++r)
            if (g.range(0, 5) != 0)
                shapes[r] = island(g, (int) g.range(0, 9), coord_t(3200000) * coord_t(r) + g.range(-200000, 200000), g.range(-200000, 200000));
        // Bands of two layers: a merged height above the wall width trips Flow::with_cross_section's assert
        // (Flow.cpp:161; compiled out at the pin with NDEBUG, a debug_assert in this crate).
        const size_t bottom = kind == 0 ? 1 : 0, top = bottom + 1;
        double merged_height = 0.;
        for (size_t k = 0; k < K; ++k) {
            const double height  = k == 0 ? first_layer_height : layer_height;
            const double print_z = first_layer_height + layer_height * double(k);
            Layer *layer = new Layer(k, obj, height, print_z, print_z - 0.5 * height);
            if (k >= bottom && k <= top) merged_height += height;
            if (! obj->m_layers.empty()) {
                layer->lower_layer = obj->m_layers.back();
                obj->m_layers.back()->upper_layer = layer;
            }
            obj->m_layers.push_back(layer);
            ExPolygons all;
            for (size_t r = 0; r < regions.size(); ++r) {
                LayerRegion *layerm = layer->add_region(regions[r]);
                if (kind == 2 && k == top) continue;
                for (const ExPolygon &e : shapes[r]) layerm->slices.surfaces.emplace_back(Surface(stInternal, e));
                append(all, shapes[r]);
            }
            layer->lslices = union_ex(all);
        }
        for (size_t k = bottom; k <= top; ++k)
            for (size_t r = 0; r < regions.size(); ++r) {
                if (shapes[r].empty()) continue;
                AdaptiveMergePatch patch;
                patch.bottom_idx    = bottom;
                patch.top_idx       = top;
                patch.group_top     = k == top;
                patch.merged_height = merged_height;
                patch.set_area(shapes[r]);
                obj->m_layers[k]->get_region(int(r))->adaptive_merge_patches.push_back(patch);
            }
        std::vector<std::string> ins;
        for (Layer *layer : obj->m_layers) {
            std::string in = hI((long long) layer->id()) + " " + hD(layer->height) + " " + hD(layer->print_z) + " " + hD(layer->slice_z) + " " + hI((long long) regions.size());
            for (const LayerRegion *layerm : layer->regions()) {
                in += " " + hI((long long) layerm->slices.surfaces.size());
                for (const Surface &s : layerm->slices.surfaces) in += " " + hI(s.extra_perimeters) + " " + EXPOLY(s.expolygon);
                in += " " + hI((long long) layerm->adaptive_merge_patches.size());
                for (const AdaptiveMergePatch &patch : layerm->adaptive_merge_patches)
                    in += " " + hI((long long) patch.bottom_idx) + " " + hI((long long) patch.top_idx) + " " + hB(patch.group_top) + " " + hD(patch.merged_height) + " " + EXPOLYS(patch.area());
            }
            in += " " + EXPOLYS(layer->lslices);
            ins.push_back(in);
        }
        out("lobj " + hI((long long) K));
        for (size_t k = 0; k < K; ++k) {
            Layer *layer = obj->m_layers[k];
            // Layer.cpp:202-408
            layer->make_perimeters();
            std::string res;
            for (const LayerRegion *layerm : layer->regions()) {
                res += std::string(res.empty() ? "" : " | ") + hI((long long) layerm->perimeters.entities.size()) + " " + hI((long long) layerm->thin_fills.entities.size()) + " | " +
                       part(LEAVES(layerm->perimeters)) + " | " + part(LEAVES(layerm->thin_fills)) + " | " + part(EXPOLYS(layerm->fill_expolygons)) + " | " +
                       part(EXPOLYS(layerm->fill_no_overlap_expolygons));
            }
            out("lpat " + ins[k] + " => " + res);
        }
        for (Layer *l : obj->m_layers) delete l;
        obj->m_layers.clear();
    }
}

// R935 'gen simplify <file>': Layer::simplify_support_entity_collection (Layer.cpp:622-685) on collections
// of a path, a multi path of two paths, a loop of two paths and a nested collection of one path, with
// spiral_mode / enable_arc_fitting / resolution of the print config (Layer.cpp:640-643). The polylines are
// noisy arcs and straight runs. One line 'lsim <spiral> <arc> <resolution> <paths> => <paths>' per case, a path
// being '<n> <x y>...' and in the results '<n> <x y>... <m> <start end path_type length>...'.
static bool g_simplify = false;
static std::string sim_path(const Polyline &pl, bool with_fitting)
{
    std::string s = hI((long long) pl.points.size());
    for (const Point &p : pl.points) s += " " + hI((long long) p.x()) + " " + hI((long long) p.y());
    if (with_fitting) {
        s += " " + hI((long long) pl.fitting_result.size());
        for (const PathFittingData &f : pl.fitting_result)
            s += " " + hI((long long) f.start_point_index) + " " + hI((long long) f.end_point_index) + " " + hI((long long) int(f.path_type)) + " " + hD(f.arc_data.length);
    }
    return s;
}
static void run_layer_simplify(PrintObject *obj)
{
    Lcg   g(0x5151deadbeefULL);
    Layer layer(1, obj, 0.2, 0.4, 0.3);
    PrintConfig &pc = const_cast<PrintConfig &>(obj->print()->config());
    auto make_polyline = [&g]() {
        Polyline pl;
        const int    n  = (int) g.range(2, 60);
        const double cx = double(g.range(-50000000, 50000000)), cy = double(g.range(-50000000, 50000000));
        if (g.range(0, 3) == 0) {
            for (int i = 0; i < n; ++i) pl.points.emplace_back(coord_t(cx + 400000. * i + g.range(-3000, 3000)), coord_t(cy + 150000. * i + g.range(-3000, 3000)));
        } else {
            const double r = double(g.range(1000000, 20000000)), a0 = double(g.range(0, 6283)) / 1000., span = double(g.range(300, 6000)) / 1000.;
            const long long noise = g.range(0, 4) == 0 ? 0 : g.range(0, 4000);
            for (int i = 0; i < n; ++i) {
                const double a = a0 + span * double(i) / double(n - 1);
                pl.points.emplace_back(coord_t(cx + r * std::cos(a)) + (noise ? g.range(-noise, noise) : 0), coord_t(cy + r * std::sin(a)) + (noise ? g.range(-noise, noise) : 0));
            }
        }
        return pl;
    };
    auto make_path = [&](const Polyline &pl) { ExtrusionPath p(erSupportMaterial, 0.05, 0.4, 0.2); p.polyline = pl; return p; };
    for (int c = 0; c < 64; ++c) {
        const bool   spiral = c % 4 == 3, arc = c % 4 != 1;
        const double resolution = (c / 4) % 2 == 0 ? 0.01 : double(g.range(1, 120)) / 1000.;
        pc.spiral_mode.value        = spiral;
        pc.enable_arc_fitting.value = arc;
        pc.resolution.value         = resolution;
        std::vector<Polyline> pls;
        for (int i = 0; i < 6; ++i) pls.push_back(make_polyline());
        ExtrusionEntityCollection coll;
        coll.append(make_path(pls[0]));
        ExtrusionMultiPath mp;
        mp.paths.push_back(make_path(pls[1]));
        mp.paths.push_back(make_path(pls[2]));
        coll.append(mp);
        ExtrusionPaths loop_paths{make_path(pls[3]), make_path(pls[4])};
        coll.append(ExtrusionLoop(std::move(loop_paths)));
        ExtrusionEntityCollection nested;
        nested.append(make_path(pls[5]));
        coll.append(nested);
        std::string in = hB(spiral) + " " + hB(arc) + " " + hD(resolution);
        for (const Polyline &pl : pls) in += " " + sim_path(pl, false);
        const ExtrusionEntityCollection orig = coll;
        // Layer.cpp:622-636
        layer.simplify_support_entity_collection(&coll);
        std::vector<const Polyline *> res_pls;
        res_pls.push_back(&static_cast<const ExtrusionPath *>(coll.entities[0])->polyline);
        for (const ExtrusionPath &p : static_cast<const ExtrusionMultiPath *>(coll.entities[1])->paths) res_pls.push_back(&p.polyline);
        for (const ExtrusionPath &p : static_cast<const ExtrusionLoop *>(coll.entities[2])->paths) res_pls.push_back(&p.polyline);
        res_pls.push_back(&static_cast<const ExtrusionPath *>(static_cast<const ExtrusionEntityCollection *>(coll.entities[3])->entities[0])->polyline);
        std::string res;
        for (const Polyline *pl : res_pls) res += std::string(res.empty() ? "" : " ") + sim_path(*pl, true);
        out("lsim " + in + " => " + res);
        // Layer.hpp:266-267 simplify_wall_extrusion_path / simplify_infill_extrusion_path -> LayerRegion.cpp:1037-1110
        // on the same collection with the roles below (erInternalInfill fits arcs at SCALED_SPARSE_INFILL_RESOLUTION).
        // One line 'lsrg <roles> => <perimeters paths> <fills paths>' after each lsim line.
        static const ExtrusionRole roles[6] = {erInternalInfill, erPerimeter, erInternalInfill, erExternalPerimeter, erInternalInfill, erSolidInfill};
        ExtrusionEntityCollection with_roles = orig;
        std::vector<ExtrusionPath *> rp;
        rp.push_back(static_cast<ExtrusionPath *>(with_roles.entities[0]));
        for (ExtrusionPath &p : static_cast<ExtrusionMultiPath *>(with_roles.entities[1])->paths) rp.push_back(&p);
        for (ExtrusionPath &p : static_cast<ExtrusionLoop *>(with_roles.entities[2])->paths) rp.push_back(&p);
        rp.push_back(static_cast<ExtrusionPath *>(static_cast<ExtrusionEntityCollection *>(with_roles.entities[3])->entities[0]));
        std::string rin;
        for (size_t i = 0; i < rp.size(); ++i) {
            rp[i]->set_extrusion_role(roles[(i + size_t(c)) % 6]);
            rin += std::string(rin.empty() ? "" : " ") + hI((long long) int(roles[(i + size_t(c)) % 6]));
        }
        if (layer.regions().empty()) layer.add_region(obj->m_shared_regions->all_regions.front().get());
        LayerRegion *layerm = layer.get_region(0);
        layerm->perimeters  = with_roles;
        layerm->fills       = with_roles;
        layer.simplify_wall_extrusion_path();
        layer.simplify_infill_extrusion_path();
        std::string rres;
        for (const ExtrusionEntityCollection *ec : {&layerm->perimeters, &layerm->fills}) {
            rres += std::string(rres.empty() ? "" : " ") + sim_path(static_cast<const ExtrusionPath *>(ec->entities[0])->polyline, true);
            for (const ExtrusionPath &p : static_cast<const ExtrusionMultiPath *>(ec->entities[1])->paths) rres += " " + sim_path(p.polyline, true);
            for (const ExtrusionPath &p : static_cast<const ExtrusionLoop *>(ec->entities[2])->paths) rres += " " + sim_path(p.polyline, true);
            rres += " " + sim_path(static_cast<const ExtrusionPath *>(static_cast<const ExtrusionEntityCollection *>(ec->entities[3])->entities[0])->polyline, true);
        }
        out("lsrg " + rin + " => " + rres);
    }
}

static void run_layer_groups()
{
    static const KeyValues base = {{"layer_height", "0.16"},
                                   {"initial_layer_print_height", "0.2"},
                                   {"wall_loops", "2"},
                                   {"outer_wall_line_width", "0.42"},
                                   {"inner_wall_line_width", "0.45"},
                                   {"line_width", "0.42"},
                                   {"sparse_infill_line_width", "0.45"},
                                   {"internal_solid_infill_line_width", "0.42"},
                                   {"top_surface_line_width", "0.42"},
                                   {"initial_layer_line_width", "0.5"},
                                   {"initial_layer_infill_line_width", "0.5"},
                                   {"resolution", "0.012"},
                                   {"enable_arc_fitting", "0"},
                                   {"z_direction_outwall_speed_continuous", "1"},
                                   {"sparse_infill_density", "15%"},
                                   {"infill_wall_overlap", "15%"},
                                   {"detect_overhang_wall", "1"},
                                   {"gap_infill_speed", "250"},
                                   {"bottom_shell_layers", "4"},
                                   {"top_shell_layers", "6"},
                                   {"brim_type", "no_brim"},
                                   {"wall_generator", "classic"}};
    static const std::vector<GroupCase> cases = {
        {"compatible", {}, {{{"sparse_infill_density", "30%"}}, {{"top_shell_layers", "3"}}}},
        {"walls", {}, {{{"wall_loops", "3"}}, {{"sparse_infill_density", "25%"}}}},
        {"widths", {}, {{{"inner_wall_line_width", "0.5"}}, {{"outer_wall_line_width", "0.45"}}, {{"sparse_infill_density", "35%"}}}},
        {"speeds", {}, {{{"inner_wall_speed", "100"}}, {{"outer_wall_speed", "60"}}, {{"gap_infill_speed", "40"}}}},
        {"mixed", {}, {{{"wall_loops", "3"}}, {{"wall_loops", "3"}, {"sparse_infill_density", "40%"}}, {{"detect_thin_wall", "1"}}}},
        {"sequence", {}, {{{"wall_sequence", "outer wall/inner wall"}}, {{"bridge_speed", "20"}}, {{"top_shell_layers", "2"}}}},
        {"arachne", {{"wall_generator", "arachne"}}, {{{"wall_loops", "3"}}, {{"sparse_infill_density", "30%"}}, {{"inner_wall_line_width", "0.5"}}}},
        // Two filaments of one nozzle diameter: a region with the walls of the second filament (Layer.cpp:175) and
        // one with its infill only.
        {"filaments", {{"nozzle_diameter", "0.4,0.4"}, {"filament_diameter", "1.75,1.75"}}, {{{"wall_filament", "2"}}, {{"sparse_infill_filament", "2"}}, {{"solid_infill_filament", "2"}}}},
    };
    Lcg g(0x9e3779b97f4a7c15ULL);
    for (const GroupCase &gc : cases) {
        Model        model;
        ModelObject *o = model.add_object();
        o->add_volume(TriangleMesh(its_make_cube(20., 20., 6.)));
        for (size_t m = 0; m < gc.mods.size(); ++m) {
            ModelVolume *v = o->add_volume(TriangleMesh(its_make_cube(4., 4., 6.)), ModelVolumeType::PARAMETER_MODIFIER);
            v->set_offset(Vec3d(2. + 5. * double(m), 2., 0.));
            ConfigSubstitutionContext ctxt(ForwardCompatibilitySubstitutionRule::Disable);
            for (const auto &kv : gc.mods[m]) v->config.set_deserialize(kv.first, kv.second, ctxt);
        }
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        set_keys(config, base);
        set_keys(config, gc.kvs);
        Print print;
        print.set_status_callback([](const PrintBase::SlicingStatus &) {});
        print.apply(model, config);
        PrintObject *obj = print.m_objects.front();
        std::vector<const PrintRegion *> regions;
        for (const auto &r : obj->m_shared_regions->all_regions) regions.push_back(r.get());
        for (size_t r = 0; r < regions.size(); ++r) {
            if (g_simplify) continue;
            std::string cfg = std::string("lcfg\t") + gc.tag + "\t" + hI((long long) r);
            auto put = [&](const char *key) {
                const ConfigBase *holder = regions[r]->config().has(key) ? static_cast<const ConfigBase *>(&regions[r]->config()) :
                                           obj->config().has(key)        ? static_cast<const ConfigBase *>(&obj->config()) :
                                           print.config().has(key)       ? static_cast<const ConfigBase *>(&print.config()) :
                                                                           nullptr;
                cfg += std::string("\t") + key + "=" + (holder == nullptr ? std::string("?") : holder->opt_serialize(key));
            };
            for (const char *key : proc_keys) put(key);
            for (const char *key : group_keys) put(key);
            out(cfg);
        }
        const double layer_height       = obj->config().layer_height.value;
        const double first_layer_height = print.config().initial_layer_print_height.value;
        for (int c = 0; c < 10; ++c) {
            const size_t layer_id = c < 2 ? 0 : (size_t) g.range(1, 9);
            const double height   = layer_id == 0 ? first_layer_height : layer_height;
            const double print_z  = first_layer_height + layer_height * double(layer_id);
            const double slice_z  = print_z - 0.5 * height;
            Layer        lower(layer_id == 0 ? 0 : layer_id - 1, obj, layer_height, print_z - height, slice_z - height);
            Layer        upper(layer_id + 1, obj, layer_height, print_z + layer_height, slice_z + layer_height);
            Layer        layer(layer_id, obj, height, print_z, slice_z);
            std::string  in = hI((long long) layer_id) + " " + hD(height) + " " + hD(print_z) + " " + hD(slice_z) + " " + hI((long long) regions.size());
            ExPolygons   all;
            for (size_t r = 0; r < regions.size(); ++r) {
                LayerRegion *layerm = layer.add_region(regions[r]);
                // One region in six without slices; the regions' islands 32 mm apart.
                ExPolygons slices;
                if (g.range(0, 5) != 0)
                    append(slices, island(g, (int) g.range(0, 9), coord_t(3200000) * coord_t(r) + g.range(-200000, 200000), g.range(-200000, 200000)));
                in += " " + hI((long long) slices.size());
                for (const ExPolygon &e : slices) {
                    Surface s(stInternal, e);
                    s.extra_perimeters = g.range(0, 7) == 0 ? 1 : 0;
                    layerm->slices.surfaces.emplace_back(s);
                    in += " " + hI(s.extra_perimeters) + " " + EXPOLY(e);
                }
                append(all, slices);
            }
            const int  lower_how = layer_id == 0 ? 6 : (int) g.range(0, 5);
            const int  upper_how = (int) g.range(0, 6);
            const bool has_lower = lower_how != 6, has_upper = upper_how != 6;
            if (has_lower) {
                lower.lslices     = neighbour(g, all, lower_how);
                layer.lower_layer = &lower;
            }
            if (has_upper) {
                upper.lslices     = neighbour(g, all, upper_how);
                layer.upper_layer = &upper;
            }
            in += " " + hB(has_lower) + " " + EXPOLYS(lower.lslices) + " " + hB(has_upper) + " " + EXPOLYS(upper.lslices);
            // Layer.cpp:202-408
            layer.make_perimeters();
            std::string res;
            for (const LayerRegion *layerm : layer.regions()) {
                res += std::string(res.empty() ? "" : " | ") + hI((long long) layerm->perimeters.entities.size()) + " " + hI((long long) layerm->thin_fills.entities.size()) + " | " +
                       part(LEAVES(layerm->perimeters)) + " | " + part(LEAVES(layerm->thin_fills)) + " | " + part(EXPOLYS(layerm->fill_expolygons)) + " | " +
                       part(EXPOLYS(layerm->fill_no_overlap_expolygons));
            }
            if (! g_patches && ! g_simplify)
                out("lgrp " + in + " => " + res);
        }
        if (g_patches)
            run_layer_patches(obj, regions, gc.tag);
        if (g_simplify) {
            run_layer_simplify(obj);
            return;
        }
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: gen <vectors.txt> [<process_vectors.txt> [full]]\n");
        return 1;
    }
    tbb::global_control one_thread(tbb::global_control::max_allowed_parallelism, 1);
    // gen groups <layer_groups_vectors.txt> [full]: Layer::make_perimeters on several regions.
    if (std::strcmp(argv[1], "simplify") == 0) {
        if (argc < 3) return 1;
        g_simplify = true;
        g_out      = std::fopen(argv[2], "w");
        if (g_out == nullptr) return 1;
        run_layer_groups();
        std::fclose(g_out);
        return 0;
    }
    if (std::strcmp(argv[1], "patches") == 0) {
        if (argc < 3) return 1;
        g_patches = true;
        g_out     = std::fopen(argv[2], "w");
        if (g_out == nullptr) return 1;
        run_layer_groups();
        std::fclose(g_out);
        return 0;
    }
    if (std::strcmp(argv[1], "groups") == 0) {
        if (argc < 3) return 1;
        g_full = argc >= 4 && std::strcmp(argv[3], "full") == 0;
        g_out  = std::fopen(argv[2], "w");
        if (g_out == nullptr) return 1;
        run_layer_groups();
        std::fclose(g_out);
        return 0;
    }
    if (std::strcmp(argv[1], "-") != 0) {
        g_out = std::fopen(argv[1], "w");
        if (g_out == nullptr) return 1;
        run_filters();
        run_flags();
        run_configs();
        run_regions();
        run_bridge_walls();
        run_traverse();
        std::fclose(g_out);
    }
    // The whole of the walls of a layer goes to a file of its own.
    if (argc >= 3) {
        g_full = argc >= 4 && std::strcmp(argv[3], "full") == 0;
        g_out  = std::fopen(argv[2], "w");
        if (g_out == nullptr) return 1;
        run_process();
        std::fclose(g_out);
    }
    return 0;
}
