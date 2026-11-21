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
#include "libslic3r/PerimeterGenerator.cpp"
#undef private
#undef protected

#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"

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

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: gen <vectors.txt>\n");
        return 1;
    }
    g_out = std::fopen(argv[1], "w");
    if (g_out == nullptr) return 1;
    tbb::global_control one_thread(tbb::global_control::max_allowed_parallelism, 1);
    run_filters();
    run_flags();
    run_configs();
    run_regions();
    run_bridge_walls();
    run_traverse();
    std::fclose(g_out);
    return 0;
}
