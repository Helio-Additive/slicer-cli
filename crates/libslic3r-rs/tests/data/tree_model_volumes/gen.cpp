// Generator of the vectors of tests/tree_model_volumes_native_parity.rs: libslic3r's
// Support/TreeModelVolumes.cpp/.hpp and Support/TreeSupportCommon.hpp at the pin
// (references/BambuStudio/src/libslic3r; the override layer has no copy of them).
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
// The headers are read with the private members of TreeModelVolumes, TreeSupportSettings, PrintObject
// and Layer made public. The file is compiled with the compiler's default floating point contraction,
// as the engine is.
//
// Every scenario is a Print of one box-shaped object made by `Print::apply`, whose object layers are
// then filled in by hand (a stem with a wide top, plates, islands), so the classes run without the
// slicing steps, with tbb limited to one thread:
//   * TreeSupportMeshGroupSettings(print_object) and TreeSupportSettings(mesh settings, slicing
//     parameters): every member, getRadius, recommendedMinRadius, getActualZ, layer_z, layer_idx_ceil,
//     layer_idx_floor, layer_mid_z, layer_idx_mid_ceil, layer_idx_mid_floor;
//   * InterfacePlacer: add_roof, add_roofs, add_roof_build_plate;
//   * TreeModelVolumes: the constructor (every member), ceilRadius, getRadiusNextCeil, the getters on
//     a volume that was not precalculated (getCollision, getAvoidance, getPlaceableAreas,
//     getWallRestriction, get_collision_lower_bound_area, each computing what it needs), and
//     precalculate() with every cache it leaves.
//
// Lines that start with "in" are the inputs the Rust test reads back (the configuration keys, the
// layers, the arguments); the others are results it has to reproduce. Tokens are separated by one
// space; a double is its 16 hexadecimal digits, an integer decimal. The long lines (areas) are
// written as "#" and the FNV-1a hash of the text the line would carry, which keeps the vectors small;
// `./gen vectors.txt full` writes every line in full, for finding the first token that differs (the
// test reads such a file when TREE_MODEL_VOLUMES_PARITY_VECTORS names it).
// The vectors were made on macOS arm64 with Apple clang and Eigen 5.0.1.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
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
#include <tbb/task_group.h>

#include <boost/container/small_vector.hpp>
#include <boost/container/static_vector.hpp>
#include <boost/functional/hash.hpp>

#define private public
#define protected public
#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Support/SupportCommon.hpp"
#include "libslic3r/Support/SupportParameters.hpp"
#include "libslic3r/Support/TreeSupportCommon.hpp"
#include "libslic3r/Support/TreeModelVolumes.hpp"
#undef protected
#undef private

using namespace Slic3r;
using namespace Slic3r::TreeSupport3D;

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
static void        out(const std::string &s) { std::fprintf(g_out, "%s\n", s.c_str()); }
// Every line in full (the second argument "full").
static bool        g_full = false;
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
// A line of a head and a long body: the body, or "#" and its hash.
static void        outb(const std::string &head, const std::string &body) { out(! g_full ? head + " #" + fnv(body) : head + " " + body); }
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
static std::string COORDS(const std::vector<coord_t> &v)
{
    std::string o = hI((long long) v.size());
    for (coord_t c : v) o += " " + hI(c);
    return o;
}
static std::string DOUBLES(const std::vector<double> &v)
{
    std::string o = hI((long long) v.size());
    for (double c : v) o += " " + hD(c);
    return o;
}

static Polygon rect(double x0, double y0, double x1, double y1)
{
    Polygon p;
    p.points = {Point(scale_(x0), scale_(y0)), Point(scale_(x1), scale_(y0)), Point(scale_(x1), scale_(y1)), Point(scale_(x0), scale_(y1))};
    return p;
}
// The corners of a regular polygon, from a table of sines so that the vectors do not depend on how the
// compiler evaluates sin and cos of one argument.
static Polygon ngon(double cx, double cy, double r, int n, double phase = 0.)
{
    Polygon p;
    for (int i = 0; i < n; ++i) {
        const double a = phase + 2. * M_PI * i / n;
        const double c = std::round(std::cos(a) * 1e6) / 1e6, s = std::round(std::sin(a) * 1e6) / 1e6;
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

static indexed_triangle_set box_its(double sx, double sy, double sz, float ox, float oy, float oz)
{
    indexed_triangle_set its = its_make_cube(sx, sy, sz);
    for (Vec3f &v : its.vertices) v += Vec3f(ox, oy, oz);
    return its;
}

struct LayerIn
{
    double     print_z, height;
    ExPolygons lslices;
};

static void add_layers(std::vector<LayerIn> &layers, int n, double lh, const ExPolygons &lslices)
{
    for (int i = 0; i < n; ++i) {
        LayerIn l;
        l.height  = lh;
        l.print_z = (layers.empty() ? 0. : layers.back().print_z) + lh;
        l.lslices = lslices;
        layers.emplace_back(std::move(l));
    }
}
// An island that starts at the layer `from` and is `n` layers tall.
static void add_island(std::vector<LayerIn> &layers, size_t from, size_t n, const ExPolygon &island)
{
    for (size_t i = from; i < from + n && i < layers.size(); ++i) layers[i].lslices.emplace_back(island);
}

// A linear congruential generator (Knuth's MMIX constants), for the scenarios from FIRST_RANDOM on.
struct Lcg
{
    uint64_t state;
    uint32_t next()
    {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        return uint32_t(state >> 33);
    }
    // In [a, b), rounded to a hundredth of a millimetre.
    double uni(double a, double b) { return std::round((a + (b - a) * (double(next()) / 2147483648.)) * 100.) / 100.; }
    int    pick(int n) { return int(next() % uint32_t(n)); }
    bool   chance(int percent) { return pick(100) < percent; }
    template<class T> T of(std::initializer_list<T> l) { return *(l.begin() + pick(int(l.size()))); }
};

static const int FIRST_RANDOM  = 7;
static const int NUM_RANDOM    = 9;
static const int NUM_SCENARIOS = FIRST_RANDOM + NUM_RANDOM;

struct Scenario
{
    std::vector<std::pair<std::string, std::string>> config;
    std::vector<LayerIn>                              layers;
    // A support blocker (a box: size and the corner of its minimum), or no size.
    Vec3d blocker_size{0., 0., 0.};
    Vec3f blocker_min{0.f, 0.f, 0.f};
    // TreeSupportSettings::soluble, as group_meshes() leaves it for the object.
    bool  soluble{false};
    // The layer precalculate() is asked for.
    int   max_layer{0};
};

static std::string num(double v)
{
    char b[32];
    std::snprintf(b, sizeof(b), "%g", v);
    return std::string(b);
}

static Scenario make_scenario(int scenario)
{
    Scenario s;
    s.config = {{"enable_support", "1"}, {"support_type", "tree(auto)"}, {"support_style", "tree_organic"}};
    double lh = 0.2;
    switch (scenario) {
    case 0:
        // A stem with a wide top.
        add_layers(s.layers, 30, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_layers(s.layers, 8, lh, {ExPolygon(rect(-18, -6, 18, 6))});
        s.max_layer = 29;
        break;
    case 1:
        // Supports on the build plate only, no top gap (the soluble contract), thick slanted branches.
        s.config.emplace_back("support_on_build_plate_only", "1");
        s.config.emplace_back("support_top_z_distance", "0");
        s.config.emplace_back("tree_support_branch_diameter", "4");
        s.config.emplace_back("tree_support_branch_angle", "30");
        s.config.emplace_back("tree_support_branch_diameter_angle", "8");
        s.soluble = true;
        add_layers(s.layers, 8, lh, {ExPolygon(rect(-4, -4, 4, 4)), ExPolygon(rect(6, -12, 19, 12))});
        add_layers(s.layers, 20, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_layers(s.layers, 6, lh, {ExPolygon(rect(-18, -8, 18, 8))});
        s.max_layer = 28;
        break;
    case 2:
        // A plate under the right arm, a top with a hole: the branches rest on the object. A raft, a wide tip.
        s.config.emplace_back("raft_layers", "2");
        s.config.emplace_back("tree_support_tip_diameter", "1.2");
        s.config.emplace_back("support_object_xy_distance", "0.5");
        add_layers(s.layers, 8, lh, {ExPolygon(rect(-4, -4, 4, 4)), ExPolygon(rect(6, -12, 19, 12))});
        add_layers(s.layers, 18, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_layers(s.layers, 6, lh, {with_hole(rect(-18, -8, 18, 8), ngon(11, 0, 3, 16))});
        s.max_layer = 27;
        break;
    case 3:
        // Thin layers, gaps of two layers above and below, bottom interfaces.
        lh = 0.16;
        s.config.emplace_back("layer_height", "0.16");
        s.config.emplace_back("initial_layer_print_height", "0.16");
        s.config.emplace_back("support_top_z_distance", "0.32");
        s.config.emplace_back("support_bottom_z_distance", "0.32");
        s.config.emplace_back("support_interface_bottom_layers", "2");
        s.config.emplace_back("tree_support_branch_angle", "50");
        s.config.emplace_back("tree_support_wall_count", "2");
        add_layers(s.layers, 10, lh, {ExPolygon(rect(-19, -12, 2, 12))});
        add_layers(s.layers, 22, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_layers(s.layers, 8, lh, {ExPolygon(rect(-18, -6, 18, 6))});
        s.max_layer = 30;
        break;
    case 4:
        // The wrapping detection area joins the machine borders.
        s.config.emplace_back("enable_wrapping_detection", "1");
        s.config.emplace_back("wrapping_exclude_area", "110x90,160x90,160x135,110x135");
        add_layers(s.layers, 20, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_layers(s.layers, 6, lh, {ExPolygon(rect(-18, -6, 18, 6))});
        s.max_layer = 19;
        break;
    case 5:
        // A support blocker over the right arm.
        add_layers(s.layers, 12, lh, {ExPolygon(rect(-19, -12, 19, 12))});
        add_layers(s.layers, 14, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_layers(s.layers, 6, lh, {ExPolygon(rect(-18, -6, 18, 6))});
        s.blocker_size = Vec3d(9., 8., 4.);
        s.blocker_min  = Vec3f(7.f, -4.f, 1.f);
        s.max_layer    = 25;
        break;
    case 6:
        // The top gap gives way to the distance to the object in the plane; a thin object.
        s.config.emplace_back("top_z_overrides_xy_distance", "1");
        s.config.emplace_back("support_object_xy_distance", "0.8");
        s.config.emplace_back("support_interface_top_layers", "0");
        s.config.emplace_back("tree_support_branch_diameter", "1");
        s.config.emplace_back("tree_support_branch_distance", "2");
        add_layers(s.layers, 24, lh, {ExPolygon(rect(-2, -2, 2, 2))});
        add_layers(s.layers, 5, lh, {ExPolygon(ngon(0, 0, 11, 40))});
        s.max_layer = 23;
        break;
    default: {
        // Drawn from the generator seeded with the number of the scenario: a post in a corner, now and then a
        // block on the bed, one to four islands that start in the air, a support configuration.
        Lcg g{uint64_t(scenario) * 0x9E3779B97F4A7C15ULL + 4711};
        for (int i = 0; i < 4; ++i) g.next();
        s.config.emplace_back("tree_support_branch_diameter", num(g.of({1., 2., 3., 5.})));
        s.config.emplace_back("tree_support_branch_diameter_angle", num(g.of({0., 5., 8., 15.})));
        s.config.emplace_back("tree_support_branch_angle", num(g.of({20., 40., 50., 60.})));
        s.config.emplace_back("support_interface_top_layers", num(g.of({0., 2., 3.})));
        if (g.chance(40)) s.config.emplace_back("support_on_build_plate_only", "1");
        if (g.chance(30)) s.config.emplace_back("tree_support_tip_diameter", num(g.of({0.4, 1.2, 2.})));
        if (g.chance(30)) s.config.emplace_back("support_object_xy_distance", num(g.of({0.1, 0.5, 1.})));
        if (g.chance(20)) s.config.emplace_back("support_top_z_distance", num(g.of({0.1, 0.4, 0.6})));
        if (g.chance(20)) s.config.emplace_back("support_bottom_z_distance", num(g.of({0., 0.4})));
        if (g.chance(20)) s.config.emplace_back("raft_layers", num(g.of({1., 3.})));
        if (g.chance(20)) s.config.emplace_back("resolution", num(g.of({0.005, 0.05})));
        if (g.chance(20)) s.config.emplace_back("top_z_overrides_xy_distance", "1");
        if (g.chance(20)) s.config.emplace_back("support_line_width", num(g.of({0.3, 0.6})));
        if (g.chance(20)) s.config.emplace_back("outer_wall_line_width", num(g.of({0.3, 0.6})));
        if (g.chance(15)) {
            lh = g.of({0.12, 0.28});
            s.config.emplace_back("layer_height", num(lh));
            s.config.emplace_back("initial_layer_print_height", num(lh));
        }
        s.soluble = g.chance(25);
        const int nl = g.of({24, 32, 40});
        add_layers(s.layers, nl, lh, {ExPolygon(rect(-20, -14, -18, -12))});
        if (g.chance(50)) {
            const int    n  = g.of({4, 8, 12});
            const double x0 = g.uni(-10, 0), y0 = g.uni(-8, 0), x1 = g.uni(1, 10), y1 = g.uni(1, 8);
            for (int i = 0; i < n; ++i) s.layers[i].lslices.emplace_back(ExPolygon(rect(x0, y0, x1, y1)));
        }
        const int islands = 1 + g.pick(4);
        int       top     = 0;
        for (int i = 0; i < islands; ++i) {
            const int    from = 10 + g.pick(nl - 6 - 10);
            const double cx = g.uni(-12, 12), cy = g.uni(-8, 8);
            top = std::max(top, from);
            switch (g.pick(3)) {
            case 0: {
                const double w = g.uni(1, 9), h = g.uni(1, 7);
                add_island(s.layers, from, 6, ExPolygon(rect(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)));
                break;
            }
            case 1: add_island(s.layers, from, 6, ExPolygon(ngon(cx, cy, g.uni(1, 6), g.of({3, 5, 8, 24})))); break;
            default: {
                const double radius = g.uni(3, 9);
                const int    n      = g.of({12, 24, 36});
                add_island(s.layers, from, 6, with_hole(ngon(cx, cy, radius, n), ngon(cx, cy, std::round(radius * g.uni(0.4, 0.8) * 100.) / 100., g.of({8, 12}))));
                break;
            }
            }
        }
        s.max_layer = top;
        break;
    }
    }
    return s;
}

static const char *cache_names[] = {"collision",          "collision_holefree", "avoidance",         "avoidance_slow",          "avoidance_to_model", "avoidance_to_model_slow",
                                    "placeable_areas",    "avoidance_holefree", "avoidance_holefree_to_model", "wall_restrictions", "wall_restrictions_min"};

// Every cache of the volumes: per layer, the radii and the areas.
static void dump_caches(const std::string &tag, const TreeModelVolumes &v)
{
    const TreeModelVolumes::RadiusLayerPolygonCache *caches[] = {&v.m_collision_cache,          &v.m_collision_cache_holefree,   &v.m_avoidance_cache,
                                                                 &v.m_avoidance_cache_slow,     &v.m_avoidance_cache_to_model,   &v.m_avoidance_cache_to_model_slow,
                                                                 &v.m_placeable_areas_cache,    &v.m_avoidance_cache_holefree,   &v.m_avoidance_cache_holefree_to_model,
                                                                 &v.m_wall_restrictions_cache,  &v.m_wall_restrictions_cache_min};
    for (size_t c = 0; c < 11; ++c) {
        const auto sorted = caches[c]->sorted();
        out(tag + " cache " + cache_names[c] + " " + hI((long long) sorted.size()));
        for (size_t i = 0; i < sorted.size();) {
            size_t j = i;
            for (++j; j < sorted.size() && sorted[i].first.second == sorted[j].first.second; ++j) ;
            std::string head = tag + " cache " + cache_names[c] + " layer " + hI(sorted[i].first.second) + " " + hI((long long) (j - i));
            for (size_t k = i; k < j; ++k) head += " " + hI(sorted[k].first.first);
            if (g_full) {
                out(head);
                for (size_t k = i; k < j; ++k)
                    out(tag + " cache " + cache_names[c] + " area " + hI(sorted[i].first.second) + " " + hI(sorted[k].first.first) + " " + POLYS(sorted[k].second.get()));
            } else {
                std::string body;
                for (size_t k = i; k < j; ++k) body += POLYS(sorted[k].second.get()) + "\n";
                out(head + " #" + fnv(body));
            }
            i = j;
        }
    }
}

static std::string MEMBERS(const TreeModelVolumes &v)
{
    return hI(v.m_max_move) + " " + hI(v.m_max_move_slow) + " " + hI(v.m_min_resolution) + " " + hB(v.m_precalculated) + " " + hI((long long) v.m_current_outline_idx) + " " +
           hI(v.m_current_min_xy_dist) + " " + hI(v.m_current_min_xy_dist_delta) + " " + hB(v.m_support_rests_on_model) + " " + hI(v.m_increase_until_radius) + " " +
           hI(v.m_radius_0) + " " + DOUBLES(v.m_raft_layers) + " " + COORDS(v.m_ignorable_radii);
}

static void run_scenario(int scenario)
{
    const std::string S  = "s" + hI(scenario);
    const Scenario    sc = make_scenario(scenario);
    Model             model;
    ModelObject      *o = model.add_object();
    o->name = "tee";
    o->add_volume(TriangleMesh(box_its(40, 28, 12, -20.f, -14.f, 0.f)));
    const bool has_blocker = sc.blocker_size.x() > 0.;
    if (has_blocker) {
        ModelVolume *b = o->add_volume(TriangleMesh(box_its(sc.blocker_size.x(), sc.blocker_size.y(), sc.blocker_size.z(), sc.blocker_min.x(), sc.blocker_min.y(), sc.blocker_min.z())));
        b->set_type(ModelVolumeType::SUPPORT_BLOCKER);
    }
    o->add_instance()->set_offset(Vec3d(100., 100., 0.));
    out("in " + S + " model 40 28 12 100 100 " + hB(has_blocker) + " " + hD(sc.blocker_size.x()) + " " + hD(sc.blocker_size.y()) + " " + hD(sc.blocker_size.z()) + " " +
        hD(sc.blocker_min.x()) + " " + hD(sc.blocker_min.y()) + " " + hD(sc.blocker_min.z()));

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(3);
    config.option<ConfigOptionStrings>("filament_colour", true)->values = {"#FF0000", "#00FF00", "#0000FF"};
    config.option<ConfigOptionInts>("filament_map", true)->values = {1, 1, 1};
    for (const auto &kv : sc.config) {
        config.set_deserialize_strict(kv.first, kv.second);
        out("in " + S + " cfg " + kv.first + " " + kv.second);
    }
    Print print;
    print.apply(model, config);
    PrintObject *obj = print.m_objects.front();
    obj->update_slicing_parameters();

    for (size_t i = 0; i < sc.layers.size(); ++i) {
        const LayerIn &l     = sc.layers[i];
        Layer         *layer = new Layer(i, obj, l.height, l.print_z, l.print_z - l.height / 2.);
        layer->lslices            = l.lslices;
        layer->lslices_extrudable = l.lslices;
        if (i > 0) {
            layer->lower_layer                = obj->m_layers.back();
            obj->m_layers.back()->upper_layer = layer;
        }
        obj->m_layers.emplace_back(layer);
        out("in " + S + " layer " + hD(l.print_z) + " " + hD(l.height) + " " + hD(layer->slice_z) + " " + EXPOLYS(l.lslices));
    }

    TreeSupportSettings::soluble = sc.soluble;
    out("in " + S + " soluble " + hB(sc.soluble));

    const SlicingParameters &sp = obj->slicing_parameters();

    // TreeSupportMeshGroupSettings
    const TreeSupportMeshGroupSettings m(*obj);
    out(S + " mesh_group_settings " + hI(m.layer_height) + " " + hI(m.resolution) + " " + hI(m.min_feature_size) + " " + hD(m.support_angle) + " " + hI(m.support_line_width) + " " +
        hI(m.support_roof_line_width) + " " + hB(m.support_bottom_enable) + " " + hI(m.support_bottom_height) + " " + hB(m.support_material_buildplate_only) + " " +
        hI(m.support_xy_distance) + " " + hI(m.support_xy_distance_1st_layer) + " " + hI(m.support_xy_distance_overhang) + " " +
        COORDS(m.support_xy_distance_overhang_per_region) + " " + hI(m.support_top_distance) + " " + hI(m.support_bottom_distance) + " " + hI(m.support_interface_skip_height) +
        " " + hB(m.support_roof_enable) + " " + hI(m.support_roof_layers) + " " + hB(m.support_floor_enable) + " " + hI(m.support_floor_layers) + " " + hD(m.minimum_roof_area) +
        " " + DOUBLES(m.support_roof_angles) + " " + hI(int(m.support_roof_pattern)) + " " + hI(m.support_line_spacing) + " " + hI(m.support_bottom_offset) + " " +
        hI(m.support_wall_count) + " " + hI(m.support_roof_line_distance) + " " + hI(m.minimum_support_area) + " " + hI(m.minimum_bottom_area) + " " + hI(m.support_offset) + " " +
        hD(m.support_tree_angle) + " " + hD(m.support_tree_branch_diameter_angle) + " " + hI(m.support_tree_branch_distance) + " " + hI(m.support_tree_branch_diameter) + " " +
        hD(m.support_tree_angle_slow) + " " + hI(m.support_tree_max_diameter_increase_by_merges_when_support_to_model) + " " + hI(m.support_tree_min_height_to_model) + " " +
        hI(m.support_tree_bp_diameter) + " " + hD(m.support_tree_top_rate) + " " + hI(m.support_tree_tip_diameter));

    // TreeSupportSettings
    TreeSupportSettings c(m, sp);
    out(S + " settings " + hI(c.support_line_width) + " " + hI(c.layer_height) + " " + hI(c.branch_radius) + " " + hI(c.min_radius) + " " + hI(c.maximum_move_distance) + " " +
        hI(c.maximum_move_distance_slow) + " " + hI((long long) c.support_bottom_layers) + " " + hI((long long) c.tip_layers) + " " + hD(c.branch_radius_increase_per_layer) + " " +
        hI(c.max_to_model_radius_increase) + " " + hI((long long) c.min_dtt_to_model) + " " + hI(c.increase_radius_until_radius) + " " +
        hI((long long) c.increase_radius_until_layer) + " " + hB(c.support_rests_on_model) + " " + hI(c.xy_distance) + " " + hI(c.bp_radius) + " " + hI(c.layer_start_bp_radius) +
        " " + hD(c.bp_radius_increase_per_layer) + " " + hI(c.xy_min_distance) + " " + COORDS(c.xy_min_distance_per_region) + " " + hI((long long) c.z_distance_top_layers) + " " +
        hI((long long) c.z_distance_bottom_layers) + " " + DOUBLES(c.support_roof_angles) + " " + hI(int(c.roof_pattern)) + " " + hI(c.support_roof_line_width) + " " +
        hI(c.support_line_spacing) + " " + hI(c.support_bottom_offset) + " " + hI(c.support_wall_count) + " " + hI(c.resolution) + " " + hI(c.support_roof_line_distance) + " " +
        hI(int(c.interface_preference)) + " " + hI(c.min_feature_size) + " " + DOUBLES(c.raft_layers) + " " + hB(c == c));
    {
        std::string line = S + " get_radius";
        for (size_t dtt : {size_t(0), size_t(1), size_t(2), size_t(5), c.tip_layers, c.tip_layers + 1, size_t(40), size_t(333)})
            for (double e : {0., 1., 2.5, 40.}) line += " " + hI(c.getRadius(dtt, e));
        out(line);
        line = S + " recommended_min_radius";
        for (LayerIndex l : {0, 1, 2, 7, c.layer_start_bp_radius - 1, c.layer_start_bp_radius, c.layer_start_bp_radius + 1, 400}) line += " " + hI(c.recommendedMinRadius(l));
        out(line);
        const int num_layers = int(sc.layers.size() + c.raft_layers.size());
        line = S + " layer_z";
        for (size_t l : {size_t(0), size_t(1), size_t(2), size_t(3), size_t(4), size_t(11), size_t(num_layers - 1), size_t(num_layers + 3)})
            line += " " + hD(layer_z(sp, c, l)) + " " + hD(layer_mid_z(sp, c, LayerIndex(l)));
        out(line);
        line = S + " layer_idx";
        for (double z : {-1., 0., 0.05, 0.1, 0.2, 0.3, 0.30000000000000004, 0.4, 0.56, 1.1, 1.2, 3.7, 3.8, 3.9000000000000004, 7.123, 30.})
            line += " " + hI(layer_idx_ceil(sp, c, z)) + " " + hI(layer_idx_floor(sp, c, z)) + " " + hI(layer_idx_mid_ceil(sp, c, z, num_layers)) + " " +
                    hI(layer_idx_mid_floor(sp, c, z, num_layers)) + " " + hI(layer_idx_mid_ceil(sp, c, z, 0)) + " " + hI(layer_idx_mid_floor(sp, c, z, 0));
        out(line);
        TreeSupportSettings c2 = c;
        std::vector<coord_t> known_z{100, 250, 400};
        c2.setActualZ(known_z);
        out(S + " get_actual_z " + hI(c2.getActualZ(0)) + " " + hI(c2.getActualZ(2)) + " " + hI(c2.getActualZ(3)) + " " + hI(c2.getActualZ(7)));
    }

    // InterfacePlacer
    {
        SupportParameters support_params(*obj);
        const size_t      num_layers = sc.layers.size() + c.raft_layers.size();
        SupportGeneratorLayerStorage layer_storage;
        SupportGeneratorLayersPtr    top_contacts(num_layers, nullptr), top_interfaces(num_layers, nullptr), top_base_interfaces(num_layers, nullptr);
        if (support_params.has_top_contacts) {
            InterfacePlacer placer(sp, support_params, c, layer_storage, top_contacts, top_interfaces, top_base_interfaces);
            const size_t    ntop = support_params.num_top_interface_layers;
            placer.add_roof(Polygons{rect(0, 0, 3, 3)}, 12, 0);
            placer.add_roof(Polygons{rect(1, 1, 4, 4)}, 12, 0);
            placer.add_roof(Polygons{rect(5, 0, 8, 3)}, 11, std::min<size_t>(1, ntop));
            placer.add_roof(Polygons{rect(5, 5, 8, 8)}, 10, ntop);
            std::vector<Polygons> roofs;
            for (size_t i = 0; i <= ntop; ++i) roofs.emplace_back(i == 1 ? Polygons{} : Polygons{rect(-9, double(i), -6, double(i) + 2)});
            placer.add_roofs(std::move(roofs), 9);
            placer.add_roof_build_plate(Polygons{rect(-3, -3, -1, -1)}, 1);
            placer.add_roof_build_plate(Polygons{rect(-3, 3, -1, 5)}, 7);
        }
        out(S + " interface_placer " + hB(support_params.has_top_contacts) + " " + hI((long long) support_params.num_top_interface_layers) + " " +
            hI((long long) support_params.num_top_interface_layers_only()));
        int which = 0;
        for (const SupportGeneratorLayersPtr *layers : {&top_contacts, &top_interfaces, &top_base_interfaces}) {
            for (size_t i = 0; i < layers->size(); ++i)
                if (const SupportGeneratorLayer *l = (*layers)[i]; l != nullptr)
                    out(S + " interface_placer layer " + hI(which) + " " + hI((long long) i) + " " + hI(int(l->layer_type)) + " " + hD(l->print_z) + " " + hD(l->bottom_z) + " " +
                        hD(l->height) + " " + POLYS(l->polygons));
            ++which;
        }
    }

    // The build volume, as generate_tree_support_3D() makes it: the bed in the coordinates of the object.
    Polygon bed = get_bed_shape_with_excluded_area(print.config());
    {
        const Vec3d plate_offset = print.get_plate_origin();
        bed.translate(Point(scale_(plate_offset(0)), scale_(plate_offset(1))) - obj->instances().front().shift);
    }
    out("in " + S + " bed " + hD(print.config().printable_height.value) + " " + POLY(bed));
    Pointfs bedptsf;
    std::transform(bed.points.begin(), bed.points.end(), std::back_inserter(bedptsf), [](const Point &p) { return unscale(p); });
    const BuildVolume build_volume{bedptsf, print.config().printable_height, {}, {}};

    // The constructor.
    {
        TreeModelVolumes v{*obj, build_volume, c.maximum_move_distance, c.maximum_move_distance_slow, 0, {}};
        out(S + " ctor " + MEMBERS(v) + " " + POLY(v.m_bed_area) + " " + POLYS(v.m_machine_border) + " " + hI((long long) v.m_layer_outlines.size()) + " " +
            hI((long long) v.m_layer_outlines.front().second.size()) + " " + hI((long long) v.m_anti_overhang.size()));
        for (size_t l = 0; l < v.m_layer_outlines.front().second.size(); ++l) outb(S + " ctor outline " + hI((long long) l), POLYS(v.m_layer_outlines.front().second[l]));
        for (size_t l = 0; l < v.m_anti_overhang.size(); ++l) outb(S + " ctor anti_overhang " + hI((long long) l), POLYS(v.m_anti_overhang[l]));

        // ceilRadius, getRadiusNextCeil
        std::string line = S + " ceil_radius";
        for (coord_t r : {coord_t(0), coord_t(1), v.m_radius_0 - 1, v.m_radius_0, v.m_radius_0 + 1, coord_t(49999), coord_t(50000), coord_t(99999), coord_t(100000), coord_t(149999),
                          coord_t(150000), coord_t(150001), coord_t(224999), coord_t(225000), coord_t(225001), coord_t(400000), coord_t(1234567)})
            line += " " + hI(v.ceilRadius(r)) + " " + hI(v.ceilRadius(r, true)) + " " + hI(v.ceilRadius(r, false)) + " " + (r > 0 ? hI(v.getRadiusNextCeil(r, true)) : "-") + " " +
                    (r > 0 ? hI(v.getRadiusNextCeil(r, false)) : "-");
        out(line);

        // The getters on a volume that was not precalculated: each computes what it needs.
        const LayerIndex nl  = LayerIndex(v.m_layer_outlines.front().second.size());
        const LayerIndex top = std::min<LayerIndex>(nl - 1, sc.max_layer + LayerIndex(c.raft_layers.size()));
        using AT = TreeModelVolumes::AvoidanceType;
        outb(S + " lazy placeable0 " + hI(top / 2), POLYS(v.getPlaceableAreas(0, top / 2, [] {})));
        outb(S + " lazy placeable0 again " + hI(top / 2), POLYS(v.getPlaceableAreas(0, top / 2, [] {})));
        for (coord_t r : {coord_t(0), c.min_radius, c.branch_radius})
            for (LayerIndex l : {0, 3, top})
                for (bool b : {true, false}) outb(S + " lazy collision " + hI(r) + " " + hI(l) + " " + hB(b), POLYS(v.getCollision(r, l, b)));
        for (coord_t r : {c.min_radius, coord_t(c.branch_radius + 30000)})
            for (LayerIndex l : {0, 2, top - 1})
                for (AT t : {AT::Slow, AT::FastSafe, AT::Fast})
                    for (bool to_model : {false, true})
                        outb(S + " lazy avoidance " + hI(r) + " " + hI(l) + " " + hI(int(t)) + " " + hB(to_model), POLYS(v.getAvoidance(r, l, t, to_model, to_model)));
        for (coord_t r : {coord_t(0), c.min_radius, c.branch_radius})
            for (LayerIndex l : {1, 4, top}) outb(S + " lazy placeable " + hI(r) + " " + hI(l), POLYS(v.getPlaceableAreas(r, l, [] {})));
        for (coord_t r : {c.min_radius, c.branch_radius})
            for (LayerIndex l : {1, 5, top})
                for (bool b : {true, false}) outb(S + " lazy wall_restriction " + hI(r) + " " + hI(l) + " " + hB(b), POLYS(v.getWallRestriction(r, l, b)));
        for (coord_t r : {coord_t(0), coord_t(10), c.min_radius, c.branch_radius, coord_t(100000000)})
            for (LayerIndex l : {0, 3, top, nl + 5}) {
                const auto a = v.get_collision_lower_bound_area(l, r);
                outb(S + " lazy lower_bound " + hI(r) + " " + hI(l) + " " + hB(bool(a)) + " " + hI(a ? a->first : -1), a ? POLYS(a->second.get()) : std::string("-"));
            }
        dump_caches(S + " lazy", v);
        v.clear_all_but_object_collision();
        dump_caches(S + " cleared", v);
        v.clear();
        out(S + " cleared_all " + hI((long long) v.m_collision_cache.sorted().size()));
    }

    // precalculate()
    {
        TreeModelVolumes v{*obj, build_volume, c.maximum_move_distance, c.maximum_move_distance_slow, 0, {}};
        const LayerIndex max_layer = sc.max_layer + LayerIndex(c.raft_layers.size());
        out("in " + S + " precalculate " + hI(max_layer));
        v.precalculate(*obj, max_layer, [] {});
        out(S + " precalculated " + MEMBERS(v));
        std::string line = S + " ceil_radius";
        for (coord_t r = 1; r < 1300000; r += 9973) line += " " + hI(v.ceilRadius(r, true)) + " " + hI(v.ceilRadius(r, false));
        out(line);
        dump_caches(S + " pre", v);
    }
}

int main(int argc, char **argv)
{
    g_out  = std::fopen(argc > 1 ? argv[1] : "vectors.txt", "w");
    g_full = argc > 2 && std::string(argv[2]) == "full";
    // Optionally one scenario only (a diagnostic).
    const int only = argc > 3 ? std::atoi(argv[3]) : -1;
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism, 1);
    out(std::string("in mode ") + (g_full ? "full" : "hash"));
    if (only >= 0)
        run_scenario(only);
    else
        for (int scenario = 0; scenario < NUM_SCENARIOS; ++scenario) run_scenario(scenario);
    std::fclose(g_out);
    return 0;
}
