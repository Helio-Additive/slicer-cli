// Generator of the vectors of tests/brim_native_parity.rs: libslic3r's Brim.cpp at the pin
// (references/BambuStudio/src/libslic3r, the override layer has no copy of it).
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
//   ./gen $DATA/vectors.txt $DATA/vectors_print.txt     (then gzip -9n vectors_print.txt)
//
// Brim.cpp is compiled into this file (most of its functions are static), with the private members of the
// classes made public. tbb is limited to one thread. Made on macOS arm64 with Apple clang and libc++.
//
// vectors.txt: the three overloads of compSecondMoment, the second moment of area the automatic brim width
// is computed from. Lines (doubles as 16 hexadecimal digits, a polygon as its number of points and the
// points, scaled):
//   poly <polygon> -> <ok> <sm.x> <sm.y>
//   expoly <number of holes> <contour> <holes> -> <ok> <aera> <centroid> <secondMomentOfAreaRespectToCentroid>
//   expolys <n> { <number of holes> <contour> <holes> } -> <ok> <smExpolysX> <smExpolysY>
//
// vectors_print.txt: the rest of Brim.cpp. First connect_brim_lines, tryExPolygonOffset and makeBrimInfill on
// polygons. Then, per scenario, a Print of boxes made with `Print::apply`; the first layer of every object
// (its slices, the slices of its regions, its volume groups) and its first support layer are written by
// hand, as the slicing steps would leave them, and the brim functions run on it: the areas
// (get_print_bottom_layers_expolygons, get_top_level_objects_with_brim, top_level_outer_brim_islands, the two
// overloads of top_level_outer_brim_area and of inner_brim_area, outer_inner_brim_area), the two overloads of
// make_inner_island_brim and of make_inner_brim, and the two overloads of make_brim. The regions of the
// first scenario carry no extrusions (getTemperatureFromExtruder and getadhesionCoeff see no filament of the
// first layer), those of the others one path; the two functions are also called on every object.
// Lines that start with "in" are the inputs the Rust test reads back; the others are results it has to
// reproduce. Tokens are separated by one space; a double is its 16 hexadecimal digits, a float its 8, an
// integer decimal; an object is named by its index in the Print.

#include <algorithm>
#include <cmath>
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

#include <boost/container/static_vector.hpp>

#define private public
#define protected public
#include "libslic3r/Brim.cpp"
#undef protected
#undef private
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;

static FILE *g_mom = nullptr;
static FILE *g_out = nullptr;

static std::string hD(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, 8);
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long) u);
    return b;
}
static std::string N(long long v) { return std::to_string(v); }
static std::string POLY(const Polygon &p)
{
    std::string s = N(p.points.size());
    for (const Point &q : p.points) s += " " + N(q.x()) + " " + N(q.y());
    return s;
}
static std::string EX(const ExPolygon &e)
{
    std::string s = N(e.holes.size()) + " " + POLY(e.contour);
    for (const Polygon &h : e.holes) s += " " + POLY(h);
    return s;
}

struct Rng
{
    uint64_t s;
    double   next()
    {
        s = s * 6364136223846793005ull + 1442695040888963407ull;
        return double((s >> 33) & 0xFFFFFF) / double(0x1000000);
    }
    double range(double a, double b) { return a + (b - a) * next(); }
};

// A star-shaped polygon around (cx, cy), counter-clockwise.
static Polygon star(Rng &r, double cx, double cy, double rmin, double rmax, int n)
{
    Polygon p;
    for (int i = 0; i < n; ++i) {
        const double a = 2. * M_PI * (i + r.range(-0.3, 0.3)) / n;
        const double d = r.range(rmin, rmax);
        p.points.emplace_back(coord_t(std::lround(cx + d * std::cos(a))), coord_t(std::lround(cy + d * std::sin(a))));
    }
    return p;
}

static ExPolygon make_ex(Rng &r, double cx, double cy, double size, int holes)
{
    ExPolygon e;
    e.contour = star(r, cx, cy, 0.6 * size, size, 5 + int(r.range(0, 12)));
    for (int h = 0; h < holes; ++h) {
        const double a = 2. * M_PI * h / holes;
        Polygon      hole = star(r, cx + 0.3 * size * std::cos(a), cy + 0.3 * size * std::sin(a), 0.03 * size, 0.08 * size, 4 + int(r.range(0, 5)));
        hole.reverse();
        e.holes.emplace_back(hole);
    }
    return e;
}


static void moments()
{
    Rng r{777ull};
    // Polygons: counter-clockwise, clockwise, degenerate.
    for (int i = 0; i < 40; ++i) {
        Polygon p = star(r, r.range(-2e7, 2e7), r.range(-2e7, 2e7), 1e5 * (1 + i), 3e5 * (1 + i), 3 + i % 17);
        if (i % 3 == 1) p.reverse();
        Vec2d sm(1., 2.);
        bool  ok = compSecondMoment(p, sm);
        std::fprintf(g_mom, "poly %s -> %d %s %s\n", POLY(p).c_str(), ok ? 1 : 0, hD(sm.x()).c_str(), hD(sm.y()).c_str());
    }
    {
        Polygon two;
        two.points = {Point(0, 0), Point(1000000, 500000)};
        Vec2d sm(1., 2.);
        bool  ok = compSecondMoment(two, sm);
        std::fprintf(g_mom, "poly %s -> %d %s %s\n", POLY(two).c_str(), ok ? 1 : 0, hD(sm.x()).c_str(), hD(sm.y()).c_str());
        Polygon rect;
        rect.points = {Point(0, 0), Point(20000000, 0), Point(20000000, 1000000), Point(0, 1000000)};
        ok = compSecondMoment(rect, sm);
        std::fprintf(g_mom, "poly %s -> %d %s %s\n", POLY(rect).c_str(), ok ? 1 : 0, hD(sm.x()).c_str(), hD(sm.y()).c_str());
    }
    // ExPolygons.
    ExPolygons all;
    for (int i = 0; i < 30; ++i) {
        ExPolygon   e = make_ex(r, r.range(-1e7, 1e7), r.range(-1e7, 1e7), 5e5 * (1 + i), i % 4);
        ExPolyProp  prop;
        prop.centroid = Vec2d(0., 0.);
        prop.secondMomentOfAreaRespectToCentroid = Vec2d(0., 0.);
        bool ok = compSecondMoment(e, prop);
        std::fprintf(g_mom, "expoly %s -> %d %s %s %s %s %s\n", EX(e).c_str(), ok ? 1 : 0, hD(prop.aera).c_str(), hD(prop.centroid.x()).c_str(), hD(prop.centroid.y()).c_str(),
                    hD(prop.secondMomentOfAreaRespectToCentroid.x()).c_str(), hD(prop.secondMomentOfAreaRespectToCentroid.y()).c_str());
        all.emplace_back(e);
    }
    // Groups of ExPolygons.
    for (int i = 0; i < 20; ++i) {
        ExPolygons  group;
        std::string s;
        const int   n = i % 6;
        for (int k = 0; k < n; ++k) {
            ExPolygon e = all[(i * 7 + k * 3) % all.size()];
            e.translate(Point(coord_t(k * 30000000), coord_t(i * 1000000)));
            s += " " + EX(e);
            group.emplace_back(e);
        }
        if (i == 13) {
            ExPolygon e;
            e.contour.points = {Point(0, 0), Point(5, 5)};
            s += " " + EX(e);
            group.emplace_back(e);
        }
        double ix = -1., iy = -2.;
        bool   ok = compSecondMoment(group, ix, iy);
        std::fprintf(g_mom, "expolys %d%s -> %d %s %s\n", int(group.size()), s.c_str(), ok ? 1 : 0, hD(ix).c_str(), hD(iy).c_str());
    }
    }

// ---------------------------------------------------------------------------------------------------------
// The rest of Brim.cpp.
// ---------------------------------------------------------------------------------------------------------

static std::string hF(float v)
{
    uint32_t u;
    std::memcpy(&u, &v, 4);
    char b[32];
    std::snprintf(b, sizeof(b), "%08x", (unsigned) u);
    return b;
}
static std::string hB(bool v) { return v ? "1" : "0"; }
static void out(const std::string &s) { std::fprintf(g_out, "%s\n", s.c_str()); }
static std::string PTS(const Points &pts)
{
    std::string o = N((long long) pts.size());
    for (const Point &pt : pts) o += " " + N(pt.x()) + " " + N(pt.y());
    return o;
}
static std::string POLYS(const Polygons &ps)
{
    std::string o = N((long long) ps.size());
    for (const Polygon &p : ps) o += " " + PTS(p.points);
    return o;
}
static std::string PLS(const Polylines &ps)
{
    std::string o = N((long long) ps.size());
    for (const Polyline &p : ps) o += " " + PTS(p.points);
    return o;
}
// An expolygon as the shared helpers of the Rust tests read it: the contour, the number of holes, the holes.
static std::string EXPOLY(const ExPolygon &e)
{
    std::string o = PTS(e.contour.points) + " " + N((long long) e.holes.size());
    for (const Polygon &h : e.holes) o += " " + PTS(h.points);
    return o;
}
static std::string EXPOLYS(const ExPolygons &es)
{
    std::string o = N((long long) es.size());
    for (const ExPolygon &e : es) o += " " + EXPOLY(e);
    return o;
}
static std::string PATH(const ExtrusionPath &p)
{
    return N(int(p.role())) + " " + hD(p.mm3_per_mm) + " " + hF(p.width) + " " + hF(p.height) + " " + PTS(p.polyline.points);
}
static std::string ENTS(const ExtrusionEntitiesPtr &v)
{
    std::string o = N((long long) v.size());
    for (const ExtrusionEntity *e : v) {
        if (const auto *l = dynamic_cast<const ExtrusionLoop *>(e)) {
            o += " L " + N((long long) l->paths.size());
            for (const ExtrusionPath &p : l->paths) o += " " + PATH(p);
        } else if (const auto *p = dynamic_cast<const ExtrusionPath *>(e))
            o += " P " + PATH(*p);
        else
            o += " X";
    }
    return o;
}

static Polygon rect(double x0, double y0, double x1, double y1)
{
    Polygon p;
    p.points = {Point(scale_(x0), scale_(y0)), Point(scale_(x1), scale_(y0)), Point(scale_(x1), scale_(y1)), Point(scale_(x0), scale_(y1))};
    return p;
}
// A polygon around a center; the vertices come from polynomials, not from sin and cos, so that the inputs do
// not depend on the math library.
static Polygon ngon(double cx, double cy, double r, int n)
{
    Polygon p;
    for (int i = 0; i < n; ++i) {
        double a = 2. * 3.14159265358979323846 * i / n;
        if (a > 3.14159265358979323846) a -= 2. * 3.14159265358979323846;
        double c = 1. - a * a / 2. + a * a * a * a / 24. - a * a * a * a * a * a / 720. + a * a * a * a * a * a * a * a / 40320.;
        double s = a - a * a * a / 6. + a * a * a * a * a / 120. - a * a * a * a * a * a * a / 5040. + a * a * a * a * a * a * a * a * a / 362880.;
        p.points.emplace_back(Point(scale_(cx + r * c), scale_(cy + r * s)));
    }
    return p;
}
static ExPolygon with_hole(const Polygon &contour, Polygon hole)
{
    ExPolygon e(contour);
    if (hole.is_counter_clockwise()) hole.reverse();
    e.holes.emplace_back(hole);
    return e;
}

typedef std::vector<std::pair<std::string, std::string>> Cfg;

static void set_common(DynamicPrintConfig &config)
{
    config.set_num_filaments(3);
    config.option<ConfigOptionStrings>("filament_colour", true)->values = {"#FF0000", "#00FF00", "#0000FF"};
    config.option<ConfigOptionInts>("filament_map", true)->values = {1, 1, 1};
}

// connect_brim_lines, tryExPolygonOffset, makeBrimInfill on polygons.
static void run_helpers()
{
    // connect_brim_lines: open pieces of rings that end near each other, with and without a wall between.
    {
        Rng r{4242ull};
        for (int k = 0; k < 24; ++k) {
            Polylines pls;
            const int n = 2 + k % 5;
            double    x = r.range(-20., 20.), y = r.range(-20., 20.);
            for (int i = 0; i < n; ++i) {
                Polyline pl;
                if (i == 2 && k % 4 == 1) {
                    pls.emplace_back(pl); // an empty one
                    continue;
                }
                const int m = 2 + int(r.range(0., 4.));
                for (int j = 0; j < m; ++j) {
                    pl.points.emplace_back(Point(scale_(x), scale_(y)));
                    x += r.range(0.5, 4.);
                    y += r.range(-2., 2.);
                }
                if (i == 1 && k % 6 == 3) pl.points.emplace_back(pl.points.front()); // a closed one
                pls.emplace_back(pl);
                // The gap to the next one: short, or longer than the connection length.
                x += (k % 3 == 2 && i == 0) ? 3. : r.range(0.1, 0.9);
                y += r.range(-0.3, 0.3);
            }
            Polygons area;
            if (k % 2 == 0) area.emplace_back(rect(-25, -25, 60, 25));
            if (k % 5 == 2) {
                // A wall across the first gap.
                const Point a = pls.front().points.back();
                area.emplace_back(rect(unscale_(a.x()) + 0.05, unscale_(a.y()) - 5., unscale_(a.x()) + 0.08, unscale_(a.y()) + 5.));
            }
            const float max_len = float(scale_(0.9));
            out("in cbl " + PLS(pls) + " " + POLYS(area) + " " + hF(max_len));
            Polylines res = connect_brim_lines(std::move(pls), area, max_len);
            out("cbl " + PLS(res));
        }
    }
    // tryExPolygonOffset and makeBrimInfill: rings and shapes with holes; a plate origin.
    {
        Model        model;
        ModelObject *o = model.add_object();
        o->name = "box";
        indexed_triangle_set its = its_make_cube(20, 20, 10);
        o->add_volume(TriangleMesh(its));
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        set_common(config);
        Print print;
        print.apply(model, config);
        const Flow flow = print.brim_flow();
        out("helpers flow " + hF(flow.width()) + " " + hF(flow.height()) + " " + hF(flow.spacing()) + " " + hF(flow.nozzle_diameter()) + " " + hD(flow.mm3_per_mm()) + " " +
            hD(print.skirt_first_layer_height()));
        std::vector<ExPolygons> areas;
        areas.push_back({with_hole(rect(-15, -15, 15, 15), rect(-10, -10, 10, 10))});
        areas.push_back({with_hole(ngon(0, 0, 14, 40), ngon(0, 0, 9.5, 32))});
        areas.push_back({ExPolygon(rect(0, 0, 3, 30)), ExPolygon(rect(10, 0, 10.6, 12))});
        areas.push_back({with_hole(rect(-20, -8, 20, 8), ngon(-9, 0, 3.3, 12)), with_hole(ngon(40, 3, 11, 9), rect(36, 0, 43, 5))});
        areas.push_back({ExPolygon(rect(0, 0, 0.3, 5))});
        areas.push_back({});
        for (size_t k = 0; k < areas.size(); ++k) {
            if (k == 3) print.set_plate_origin(Vec3d(180.5, -20.25, 0.));
            const Vec3d origin = print.get_plate_origin();
            out("in bi " + EXPOLYS(areas[k]) + " " + hD(origin.x()) + " " + hD(origin.y()));
            out("bi offset " + POLYS(tryExPolygonOffset(areas[k], print, flow)));
            Polygons                  islands;
            ExtrusionEntityCollection c = makeBrimInfill(areas[k], print, islands, flow);
            out("bi infill " + ENTS(c.entities));
        }
    }
}

struct ObjSpec
{
    double                                 sx, sy, sz;
    std::vector<std::pair<double, double>> instances;
    Cfg                                    cfg;
    // The slices of the first layer: 0 the footprint, 1 the footprint with a hole and an island in the hole,
    // 2 two islands, one with a round hole, in two volume groups, 3 the footprint with two holes.
    int  kind;
    int  extruder;
    bool support;
    int  ears;
};
struct Scen
{
    Cfg                  cfg;
    std::vector<ObjSpec> objs;
    double               ox, oy;
    int                  extra; // 1: a wipe tower wall, 2: a skirt of two loops
    std::string          m1 = "PLA", m2 = "ABS";
};

static ExPolygons first_slices(const ObjSpec &o)
{
    const double hx = o.sx / 2., hy = o.sy / 2.;
    switch (o.kind) {
    case 1: return {with_hole(rect(-hx, -hy, hx, hy), rect(-hx * 0.6, -hy * 0.6, hx * 0.6, hy * 0.6)), ExPolygon(ngon(0.5, -0.3, std::min(hx, hy) * 0.25, 14))};
    case 2: return {with_hole(rect(-hx, -hy, -hx * 0.1, hy), ngon(-hx * 0.55, 0.2, std::min(hx, hy) * 0.3, 10)), ExPolygon(rect(hx * 0.2, -hy * 0.7, hx, hy * 0.5))};
    case 3: return {[&]() {
        ExPolygon e = with_hole(rect(-hx, -hy, hx, hy), rect(-hx * 0.8, -hy * 0.5, -hx * 0.2, hy * 0.5));
        Polygon   h = ngon(hx * 0.5, 0., std::min(hx, hy) * 0.35, 18);
        h.reverse();
        e.holes.emplace_back(h);
        return e;
    }()};
    default: return {ExPolygon(rect(-hx, -hy, hx, hy))};
    }
}

static std::string MAP(const Print &print, const std::map<ObjectID, ExPolygons> &m)
{
    std::string o = N((long long) m.size());
    for (const auto &kv : m) {
        long long idx = -1;
        for (size_t i = 0; i < print.objects().size(); ++i)
            if (print.objects()[i]->id() == kv.first) idx = (long long) i;
        o += " " + N(idx) + " " + EXPOLYS(kv.second);
    }
    return o;
}
static std::string EMAP(const Print &print, const std::map<ObjectID, ExtrusionEntityCollection> &m)
{
    std::string o = N((long long) m.size());
    for (const auto &kv : m) {
        long long idx = -1;
        for (size_t i = 0; i < print.objects().size(); ++i)
            if (print.objects()[i]->id() == kv.first) idx = (long long) i;
        o += " " + N(idx) + " " + ENTS(kv.second.entities);
    }
    return o;
}
static std::string OBJS(const Print &print, const ConstPrintObjectPtrs &v)
{
    std::string o = N((long long) v.size());
    for (const PrintObject *p : v)
        for (size_t i = 0; i < print.objects().size(); ++i)
            if (print.objects()[i] == p) o += " " + N((long long) i);
    return o;
}

static void run_scenario(int k, const Scen &sc)
{
    const std::string S = "b" + N(k);
    Model model;
    for (const ObjSpec &os : sc.objs) {
        ModelObject *o = model.add_object();
        o->name = "box";
        indexed_triangle_set its = its_make_cube(os.sx, os.sy, os.sz);
        for (Vec3f &v : its.vertices) v += Vec3f(float(-os.sx / 2.), float(-os.sy / 2.), 0.f);
        o->add_volume(TriangleMesh(its));
        std::string line = "in " + S + " object " + hD(os.sx) + " " + hD(os.sy) + " " + hD(os.sz) + " " + N(os.kind) + " " + N(os.extruder) + " " + hB(os.support) + " " + N(os.ears) + " " +
                           N((long long) os.instances.size());
        for (const auto &in : os.instances) {
            o->add_instance()->set_offset(Vec3d(in.first, in.second, 0.));
            line += " " + hD(in.first) + " " + hD(in.second);
        }
        line += " " + N((long long) os.cfg.size());
        for (const auto &kv : os.cfg) {
            ConfigSubstitutionContext ctxt(ForwardCompatibilitySubstitutionRule::Disable);
            o->config.set_deserialize(kv.first, kv.second, ctxt);
            line += " " + kv.first + " " + kv.second;
        }
        if (os.extruder != 1) o->config.set("extruder", os.extruder);
        for (int e = 0; e < os.ears; ++e)
            o->brim_points.emplace_back(BrimPoint(float(-os.sx / 2. + e * os.sx), float(os.sy / 2. - e * 0.5 * os.sy), 0.f, 4.f + e));
        out(line);
    }
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    set_common(config);
    for (const auto &kv : sc.cfg) {
        config.set_deserialize_strict(kv.first, kv.second);
        out("in " + S + " cfg " + kv.first + " " + kv.second);
    }
    Print print;
    print.apply(model, config);
    print.set_plate_origin(Vec3d(sc.ox, sc.oy, 0.));
    // The materials of the first two filaments (Model::extruderParamsMap).
    Model::extruderParamsMap[1].materialName = sc.m1;
    Model::extruderParamsMap[2].materialName = sc.m2;
    out("in " + S + " print " + hD(sc.ox) + " " + hD(sc.oy) + " " + N(sc.extra) + " " + sc.m1 + " " + sc.m2);

    std::vector<std::pair<ObjectID, unsigned int>> objPrintVec;
    std::vector<unsigned int>                      printExtruders;
    for (size_t i = 0; i < print.m_objects.size(); ++i) {
        PrintObject   *obj = print.m_objects[i];
        const ObjSpec &os  = sc.objs[i];
        obj->update_slicing_parameters();
        const double h = obj->slicing_parameters().first_print_layer_height;
        const double z = obj->slicing_parameters().object_print_z_min + h;
        Layer       *layer = new Layer(0, obj, h, z, z - h / 2.);
        layer->lslices = first_slices(os);
        // The regions: the slices, the last expolygon in a region of its own when there are several.
        const size_t n_regions = layer->lslices.size() > 1 ? 2 : 1;
        for (size_t r = 0; r < n_regions; ++r) {
            // The region of the object (it has one volume, so one region); both layer regions print with it.
            LayerRegion *lr = layer->add_region(obj->m_shared_regions->all_regions.front().get());
            for (size_t e = 0; e < layer->lslices.size(); ++e)
                if ((n_regions == 1) || ((e + 1 == layer->lslices.size()) == (r == 1))) lr->slices.surfaces.emplace_back(stInternal, layer->lslices[e]);
            if (k != 0 && !lr->slices.surfaces.empty()) {
                // An extrusion: the region counts for the filaments of the first layer.
                ExtrusionPath *p = new ExtrusionPath(erExternalPerimeter, 0.04, 0.42f, float(h));
                const Point    c = lr->slices.surfaces.front().expolygon.contour.points.front();
                p->polyline.points = {c, Point(c.x() + coord_t(100000), c.y())};
                lr->perimeters.entities.emplace_back(p);
            }
        }
        obj->m_layers.emplace_back(layer);
        std::string line = "in " + S + " layer " + N((long long) i) + " " + hD(z) + " " + hD(h) + " " + hD(layer->slice_z) + " " + EXPOLYS(layer->lslices) + " " + N((long long) n_regions);
        for (const LayerRegion *lr : layer->regions()) {
            ExPolygons es;
            for (const Surface &s : lr->slices.surfaces) es.emplace_back(s.expolygon);
            line += " " + EXPOLYS(es) + " " + ENTS(lr->perimeters.entities);
        }
        out(line);
        // The volume groups of the first layer.
        const ObjectID vid = obj->model_object()->volumes.front()->id();
        if (os.kind == 2) {
            for (size_t e = 0; e < layer->lslices.size(); ++e) {
                groupedVolumeSlices g;
                g.groupId    = int(e);
                g.volume_ids = {vid};
                g.slices     = {layer->lslices[e]};
                obj->firstLayerObjSliceByGroups.emplace_back(g);
            }
        } else {
            groupedVolumeSlices g;
            g.groupId    = 0;
            g.volume_ids = {vid};
            g.slices     = layer->lslices;
            obj->firstLayerObjSliceByGroups.emplace_back(g);
        }
        line = "in " + S + " groups " + N((long long) i) + " " + N((long long) obj->firstLayerObjSliceByGroups.size());
        for (const groupedVolumeSlices &g : obj->firstLayerObjSliceByGroups) line += " " + N(g.groupId) + " " + EXPOLYS(g.slices);
        out(line);
        // The first support layer: an island beside the object and one inside its first hole, two lines each.
        if (os.support) {
            SupportLayer *sl = new SupportLayer(0, 0, obj, h, z, z - h / 2.);
            const double  hx = os.sx / 2., hy = os.sy / 2.;
            sl->support_islands = {ExPolygon(rect(hx + 1., -hy, hx + 5., hy * 0.5))};
            auto add_path = [&](double x0, double y0, double x1, double y1) {
                ExtrusionPath *p = new ExtrusionPath(erSupportMaterial, 0.05, 0.42f, float(h));
                p->polyline.points = {Point(scale_(x0), scale_(y0)), Point(scale_(x1), scale_(y1))};
                sl->support_fills.entities.emplace_back(p);
            };
            add_path(hx + 1.5, -hy + 0.5, hx + 1.5, hy * 0.5 - 0.5);
            add_path(hx + 4.5, -hy + 0.5, hx + 4.5, hy * 0.5 - 0.5);
            if (os.kind == 3) {
                sl->support_islands.emplace_back(ExPolygon(rect(-hx * 0.7, -hy * 0.3, -hx * 0.3, hy * 0.3)));
                add_path(-hx * 0.6, -hy * 0.2, -hx * 0.6, hy * 0.2);
            }
            obj->m_support_layers.emplace_back(sl);
            out("in " + S + " support " + N((long long) i) + " " + hD(z) + " " + hD(h) + " " + hD(sl->slice_z) + " " + EXPOLYS(sl->support_islands) + " " + ENTS(sl->support_fills.entities));
        }
        objPrintVec.emplace_back(obj->id(), (unsigned int) os.extruder);
        if (std::find(printExtruders.begin(), printExtruders.end(), (unsigned int) (os.extruder - 1)) == printExtruders.end()) printExtruders.emplace_back((unsigned int) (os.extruder - 1));
    }
    std::sort(printExtruders.begin(), printExtruders.end());
    if (sc.extra == 1) {
        // The outer wall of a wipe tower.
        Polyline wall;
        wall.points = {Point(scale_(0.), scale_(0.)), Point(scale_(0.), scale_(0.)), Point(scale_(12.), scale_(0.)), Point(scale_(12.), scale_(9.)), Point(scale_(0.), scale_(9.))};
        print.m_fake_wipe_tower.outer_wall[0.2f] = {wall};
        print.m_fake_wipe_tower.pos               = Vec2f(30.25f, 61.5f);
        out("in " + S + " tower " + hF(0.2f) + " " + PTS(wall.points) + " " + hF(print.m_fake_wipe_tower.pos.x()) + " " + hF(print.m_fake_wipe_tower.pos.y()));
    }
    if (sc.extra == 2) {
        // A skirt of two loops across the brim of the first object.
        const Point c = print.m_objects.front()->instances().front().shift_without_plate_offset();
        // The first loop is the outer one, as Print::_make_skirt leaves them.
        for (double r : {14.5, 14.}) {
            Polygon       p = ngon(unscale_(c.x()), unscale_(c.y()), r, 48);
            ExtrusionPath path(erSkirt, 0.05, 0.5f, 0.2f);
            path.polyline.points = p.points;
            path.polyline.points.emplace_back(p.points.front());
            print.m_skirt.entities.emplace_back(new ExtrusionLoop(std::move(path)));
        }
        out("in " + S + " skirt " + ENTS(print.m_skirt.entities));
    }
    {
        std::string line = S + " setup " + hB(print.has_wipe_tower()) + " " + hB(print.has_support_material()) + " " + N((long long) print.extruders().size());
        for (const PrintObject *obj : print.objects()) {
            line += " " + hB(obj->has_brim()) + " " + N((long long) obj->instances().size());
            for (const PrintInstance &in : obj->instances()) line += " " + N(in.shift_without_plate_offset().x()) + " " + N(in.shift_without_plate_offset().y());
        }
        out(line);
    }

    {
        std::string line = S + " adhesion";
        for (const PrintObject *obj : print.objects()) line += " " + hD(getTemperatureFromExtruder(obj)) + " " + hD(getadhesionCoeff(obj));
        out(line);
    }

    const Flow  flow = print.brim_flow();
    const float fs   = float(flow.scaled_spacing());
    out(S + " flow " + hF(flow.width()) + " " + hF(flow.height()) + " " + hF(flow.spacing()) + " " + hD(flow.mm3_per_mm()) + " " + hD(print.skirt_first_layer_height()));

    // The areas.
    std::vector<ExPolygons> bottom = get_print_bottom_layers_expolygons(print);
    {
        std::string line = S + " bottom " + N((long long) bottom.size());
        for (const ExPolygons &e : bottom) line += " " + EXPOLYS(e);
        out(line);
    }
    ConstPrintObjectPtrs top = get_top_level_objects_with_brim(print, bottom);
    out(S + " top " + OBJS(print, top));
    const double scaled_resolution = scaled<double>(print.config().resolution.value);
    if (!top.empty()) {
        out(S + " islands " + POLYS(top_level_outer_brim_islands(top, scaled_resolution)));
        {
            double                     wmax = 0;
            std::map<ObjectID, double> wmap;
            ExPolygons                 a = top_level_outer_brim_area(print, top, bottom, fs, wmax, wmap);
            std::string                line = S + " outer_area " + EXPOLYS(a) + " " + hD(wmax) + " " + N((long long) wmap.size());
            for (const auto &kv : wmap) line += " " + hD(kv.second);
            out(line);
        }
        {
            double                         wmax = 0;
            std::map<ObjectID, double>     wmap;
            std::map<ObjectID, ExPolygons> am, sm;
            ExPolygons                     a = top_level_outer_brim_area(print, top, fs, wmax, wmap, am, sm, objPrintVec);
            out(S + " outer_area_map " + EXPOLYS(a) + " " + hD(wmax) + " | " + MAP(print, am) + " | " + MAP(print, sm));
        }
        out(S + " inner_area " + EXPOLYS(inner_brim_area(print, top, bottom, fs)));
        {
            std::map<ObjectID, ExPolygons> am, sm;
            ExPolygons                     a = inner_brim_area(print, top, fs, am, sm, objPrintVec);
            out(S + " inner_area_map " + EXPOLYS(a) + " | " + MAP(print, am) + " | " + MAP(print, sm));
        }
        // The inner islands.
        {
            ExtrusionEntityCollection brim;
            ExPolygons                islands;
            make_inner_island_brim(print, top, brim, islands);
            out(S + " inner_island " + ENTS(brim.entities) + " | " + EXPOLYS(islands));
        }
        {
            std::map<ObjectID, ExPolygons> am, sm;
            ExPolygons                     islands;
            ExPolygons                     nobrim = {ExPolygon(rect(-1000., -1000., -999., -999.))};
            if (k % 2 == 1) nobrim.emplace_back(ExPolygon(rect(sc.objs.front().instances.front().first - sc.ox - 0.7, -500., sc.objs.front().instances.front().first - sc.ox + 0.9, 500.)));
            out("in " + S + " nobrim " + EXPOLYS(nobrim));
            make_inner_island_brim(print, top, am, sm, islands, nobrim, objPrintVec);
            out(S + " inner_island_map " + EXPOLYS(islands) + " | " + MAP(print, am) + " | " + MAP(print, sm));
        }
        {
            ExtrusionEntityCollection brim;
            make_inner_brim(print, top, bottom, brim);
            out(S + " inner_brim " + ENTS(brim.entities));
        }
        {
            std::map<ObjectID, ExPolygons> am, sm;
            make_inner_brim(print, top, am, sm, objPrintVec);
            out(S + " inner_brim_map " + MAP(print, am) + " | " + MAP(print, sm));
        }
    }
    // The brims of all objects by volume groups.
    {
        std::map<ObjectID, ExPolygons> am, sm;
        ExPolygons                     a = outer_inner_brim_area(print, fs, am, sm, objPrintVec, printExtruders);
        out(S + " oiba " + EXPOLYS(a) + " | " + MAP(print, am) + " | " + MAP(print, sm));
    }
    {
        Polygons                                      islands_area;
        std::map<ObjectID, ExtrusionEntityCollection> bm, sbm;
        make_brim(print, print.make_try_cancel(), islands_area, bm, sbm, objPrintVec, printExtruders);
        out(S + " make_brim_map " + POLYS(islands_area) + " | " + EMAP(print, bm) + " | " + EMAP(print, sbm));
        std::string line = S + " bbox";
        for (const PrintObject *obj : print.objects()) {
            const BoundingBox &bb = obj->firstLayerObjectBrimBoundingBox;
            line += " " + hB(bb.defined);
            if (bb.defined) line += " " + N(bb.min.x()) + " " + N(bb.min.y()) + " " + N(bb.max.x()) + " " + N(bb.max.y());
        }
        out(line);
    }
    // The brim around all objects.
    if (!top.empty()) {
        Polygons                  islands_area;
        ExtrusionEntityCollection brim = make_brim(print, print.make_try_cancel(), islands_area);
        out(S + " make_brim " + POLYS(islands_area) + " | " + ENTS(brim.entities));
    }
}

static std::vector<Scen> scenarios()
{
    std::vector<Scen> s;
    // 0: one box, the default brim (auto), one instance.
    s.push_back({{}, {{20, 20, 10, {{100, 100}}, {}, 0, 1, false, 0}}, 0, 0, 0});
    // 1: outer brim; a hole with an island in it; two instances.
    s.push_back({{{"brim_type", "outer_only"}, {"brim_width", "5"}}, {{30, 24, 10, {{90, 100}, {150, 104.5}}, {}, 1, 1, false, 0}}, 0, 0, 0});
    // 2: inner brim; a gap to the object.
    s.push_back({{{"brim_type", "inner_only"}, {"brim_width", "4"}, {"brim_object_gap", "0.15"}}, {{30, 24, 10, {{90, 100}}, {}, 1, 1, false, 0}}, 0, 0, 0});
    // 3: outer and inner brim, two holes, a support layer; a plate origin.
    s.push_back({{{"brim_type", "outer_and_inner"}, {"brim_width", "3.3"}, {"enable_support", "1"}}, {{34, 26, 10, {{110, 120}}, {}, 3, 1, true, 0}}, 256., 0., 0});
    // 4: three objects: an outer brim, no brim, an inner brim of another filament; the first two close to each other.
    s.push_back({{{"brim_width", "6"}},
                 {{20, 20, 10, {{80, 80}}, {{"brim_type", "outer_only"}}, 0, 1, false, 0},
                  {16, 30, 5, {{104, 82}}, {{"brim_type", "no_brim"}}, 3, 1, false, 0},
                  {30, 30, 8, {{150, 150}, {60, 160}}, {{"brim_type", "outer_and_inner"}, {"brim_width", "2.5"}}, 2, 2, false, 0}},
                 0, 0, 0});
    // 5: the automatic brim of a tall thin box and of a flat one; an exclude area of the bed.
    s.push_back({{{"brim_type", "auto_brim"}, {"bed_exclude_area", "0x0,18x0,18x28,0x28"}},
                 {{4, 4, 120, {{60, 60}}, {}, 0, 1, false, 0}, {50, 6, 60, {{140, 70}}, {}, 0, 2, false, 0}, {40, 40, 2, {{120, 150}}, {}, 1, 1, false, 0}},
                 0, 0, 0});
    // 6: brim ears.
    s.push_back({{{"brim_type", "brim_ears"}, {"brim_width", "5"}}, {{24, 18, 10, {{100, 100}}, {}, 3, 1, false, 2}}, 0, 0, 0});
    // 7: a raft: no brim of the object; supports.
    s.push_back({{{"brim_type", "outer_only"}, {"brim_width", "5"}, {"raft_layers", "2"}, {"enable_support", "1"}}, {{20, 20, 10, {{100, 100}}, {}, 0, 1, true, 0}}, 0, 0, 0});
    // 8: a wipe tower beside two objects of two filaments; supports of the object filament.
    s.push_back({{{"brim_type", "outer_only"}, {"brim_width", "8"}, {"enable_prime_tower", "1"}, {"enable_support", "1"}},
                 {{20, 20, 10, {{50, 80}}, {}, 0, 1, true, 0}, {20, 20, 10, {{100, 80}}, {}, 2, 2, true, 0}},
                 0, 0, 1});
    // 9: a draft shield whose skirt crosses the brim.
    s.push_back({{{"brim_type", "outer_only"}, {"brim_width", "10"}, {"draft_shield", "enabled"}, {"skirt_distance", "2"}, {"skirt_loops", "2"}},
                 {{16, 16, 10, {{100, 100}}, {}, 0, 1, false, 0}},
                 0, 0, 2});
    // 10: by object: the support filament of the object.
    s.push_back({{{"brim_type", "outer_and_inner"}, {"brim_width", "4"}, {"print_sequence", "by object"}, {"enable_support", "1"}, {"support_filament", "0"}},
                 {{24, 24, 10, {{70, 70}}, {}, 3, 2, true, 0}, {18, 18, 10, {{150, 150}}, {{"brim_type", "inner_only"}}, 1, 1, true, 0}},
                 0, 0, 0});
    // 11: the automatic brim of PETG and PCTG (twice the width); the bed temperatures of another plate.
    s.push_back({{{"brim_type", "auto_brim"}, {"curr_bed_type", "High Temp Plate"}, {"hot_plate_temp_initial_layer", "55,80,65"}, {"cool_plate_temp_initial_layer", "35,40,45"}},
                 {{4, 4, 120, {{60, 60}}, {}, 0, 1, false, 0}, {50, 6, 60, {{140, 70}}, {}, 0, 2, false, 0}, {8, 8, 100, {{110, 150}}, {}, 1, 2, false, 0}},
                 0, 0, 0, "PETG", "PCTG"});
    // 12: the automatic brim of TPU (half the width).
    s.push_back({{{"brim_type", "auto_brim"}, {"cool_plate_temp_initial_layer", "35,40,45"}},
                 {{4, 4, 120, {{60, 60}}, {}, 0, 1, false, 0}, {50, 6, 60, {{140, 70}}, {}, 0, 2, false, 0}, {5, 30, 150, {{110, 150}, {30, 150}}, {}, 0, 3, false, 0}},
                 0, 0, 0, "TPU", "TPU-AMS"});
    // 13: PCTG and TPU-AMS by their other names; a plate origin.
    s.push_back({{{"brim_type", "auto_brim"}},
                 {{6, 5, 140, {{70, 60}}, {}, 0, 1, false, 0}, {3, 40, 90, {{150, 90}}, {}, 0, 1, false, 0}},
                 0., 256., 0, "PCTG", "PLA"});
    s.push_back({{{"brim_type", "auto_brim"}},
                 {{6, 5, 140, {{70, 60}}, {}, 0, 1, false, 0}, {3, 40, 90, {{150, 90}}, {}, 0, 1, false, 0}},
                 0, 0, 0, "TPU-AMS", "PLA"});
    return s;
}

int main(int argc, char **argv)
{
    if (argc < 3) return 1;
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism, 1);
    g_mom = std::fopen(argv[1], "w");
    g_out = std::fopen(argv[2], "w");
    if (!g_mom || !g_out) return 1;
    moments();
    std::fclose(g_mom);
    // The statics of Model the brim reads.
    Model::printSpeedMap.perimeterSpeed         = 200.;
    Model::printSpeedMap.externalPerimeterSpeed = 150.;
    Model::printSpeedMap.infillSpeed            = 270.;
    Model::printSpeedMap.solidInfillSpeed       = 250.;
    Model::printSpeedMap.topSolidInfillSpeed    = 200.;
    Model::printSpeedMap.supportSpeed           = 150.;
    Model::printSpeedMap.smallPerimeterSpeed    = 75.;
    Model::printSpeedMap.maxSpeed               = 270.;
    Model::extruderParamsMap[1]                 = {"PLA", 55, 220.};
    Model::extruderParamsMap[2]                 = {"ABS", 100, 260.};
    out("in statics " + hD(200.) + " " + hD(150.) + " " + hD(270.) + " " + hD(250.) + " " + hD(200.) + " " + hD(150.) + " " + hD(75.) + " " + hD(270.) + " 3 0 - 0 " + hD(0.) +
        " 1 PLA 55 " + hD(220.) + " 2 ABS 100 " + hD(260.));
    run_helpers();
    const std::vector<Scen> s = scenarios();
    for (size_t i = 0; i < s.size(); ++i) run_scenario(int(i), s[i]);
    std::fclose(g_out);
    return 0;
}
