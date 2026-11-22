// Generator of the vectors of tests/tree_support_3d_native_parity.rs: libslic3r's Support/TreeSupport3D.cpp
// and Support/TreeSupport3D.hpp at the pin (references/BambuStudio/src/libslic3r; the override layer has no
// copy of them).
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
// TreeSupport3D.cpp is compiled into this file (its helpers are static), with the private members of the
// classes it uses made public. The file is compiled with the compiler's default floating point
// contraction, as the engine is.
//
// The helpers that take no volumes run on polylines, polygons and points drawn from a linear congruential
// generator. Every scenario is a Print of one box-shaped object made by `Print::apply`, whose object layers
// and overhangs are filled in by hand, so that the tree support runs without the slicing steps and without
// the overhang detection of TreeSupport, with tbb limited to one thread: group_meshes(), the constructor of
// TreeModelVolumes, precalculate(), the helpers that query the volumes, generate_support_infill_lines(),
// generate_initial_areas() (with generate_raft_contact(), sample_overhang_area(), RichInterfacePlacer and
// finalize_raft_contact()), create_layer_pathing(), create_nodes_from_area(), generate_branch_areas(),
// smooth_branch_areas() and draw_areas().
//
// Lines that start with "in" are the inputs the Rust test reads back; the others are results it has to
// reproduce. Tokens are separated by one space; a double is its 16 hexadecimal digits, a float its 8, an
// integer decimal. The long lines of the scenarios are written as "#" and the FNV-1a hash of the text the
// line would carry, which keeps the vectors small; `./gen vectors.txt full` writes every line in full, for
// finding the first token that differs (the test reads such a file when TREE_SUPPORT_3D_PARITY_VECTORS
// names it).
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
#include <tbb/task_arena.h>
#include <tbb/task_group.h>

#include <boost/container/small_vector.hpp>
#include <boost/container/static_vector.hpp>
#include <boost/functional/hash.hpp>

#define private public
#define protected public
// What TreeSupport3D.cpp includes, ahead of it: the headers are read with `Fill` meaning the class of
// FillBase.hpp.
#include "libslic3r/Support/TreeSupport3D.hpp"
#include "libslic3r/AABBTreeIndirect.hpp"
#include "libslic3r/AABBTreeLines.hpp"
#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/EdgeGrid.hpp"
#include "libslic3r/Fill/Fill.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/Fill/Lightning/Generator.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/MultiPoint.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/MutablePolygon.hpp"
#include "libslic3r/ShortestPath.hpp"
#include "libslic3r/Support/SupportCommon.hpp"
#include "libslic3r/Time.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"
#include "libslic3r/Support/TreeSupport.hpp"
#include "libslic3r/I18N.hpp"
#include "libslic3r/Log.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"

namespace Slic3r {
// The filler of the generator: TreeSupport3D.cpp is compiled with `Fill` standing for `RecFill`, a filler that
// writes down what it is asked (its pattern, the members generate_support_infill_lines() sets, the parameters
// and the surface) and answers with lines across the surface that it clips with the surface. The Rust test
// has the same filler, so that everything around the fillers is compared, not the fillers of the engine.
struct RecFill : public Fill
{
    InfillPattern   pattern{ipRectilinear};
    static RecFill *new_from_type(const InfillPattern type)
    {
        RecFill *f = new RecFill();
        f->pattern = type;
        return f;
    }
    Fill     *clone() const override { return new RecFill(*this); }
    bool      is_self_crossing() override { return false; }
    Polylines fill_surface(const Surface *surface, const FillParams &params) override;
};
} // namespace Slic3r

#define Fill RecFill
#include "libslic3r/Support/TreeSupport3D.cpp"
#undef Fill
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
static std::string PLS(const Polylines &ps)
{
    std::string o = hI((long long) ps.size());
    for (const Polyline &p : ps) o += " " + PTS(p.points);
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
static std::string LINES(const LineInformations &lines)
{
    std::string o = hI((long long) lines.size());
    for (const LineInformation &l : lines) {
        o += " " + hI((long long) l.size());
        for (const auto &p : l) o += " " + PT(p.first) + " " + hI(int(p.second));
    }
    return o;
}

// What the fillers were asked since the list was cleared.
static std::vector<std::string> g_fills;

// The filler of the generator. Its lines: at every `step` of the height of the surface, slanted unless the
// pattern is the rectilinear one, clipped with the surface. The concentric one fails on an odd number of lines.
Polylines Slic3r::RecFill::fill_surface(const Surface *surface, const FillParams &params)
{
    g_fills.emplace_back(hI(int(this->pattern)) + " " + hF(this->angle) + " " + hD(this->spacing) + " " + hI((long long) this->layer_id) + " " + hF(params.density) + " " +
                         hB(params.dont_adjust) + " " + EXPOLY(surface->expolygon));
    const Points &pts = surface->expolygon.contour.points;
    if (pts.empty()) return {};
    Point lo = pts.front(), hi = pts.front();
    for (const Point &q : pts) {
        lo = Point(std::min(lo.x(), q.x()), std::min(lo.y(), q.y()));
        hi = Point(std::max(hi.x(), q.x()), std::max(hi.y(), q.y()));
    }
    const double d    = params.density > 0.f ? double(params.density) : 1.;
    coord_t      step = coord_t(scale_(this->spacing) / d);
    if (step < 20000) step = 20000;
    const coord_t slant = this->pattern == ipRectilinear ? 0 : 7770;
    Polylines     lines;
    for (coord_t y = lo.y() + step / 2; y < hi.y(); y += step) lines.emplace_back(Polyline(Point(lo.x() - 1000, y), Point(hi.x() + 1000, y + slant)));
    if (this->pattern == ipConcentric && lines.size() % 2 == 1) throw InfillFailedException();
    return intersection_pl(lines, surface->expolygon);
}

static Polygon rect(double x0, double y0, double x1, double y1)
{
    Polygon p;
    p.points = {Point(scale_(x0), scale_(y0)), Point(scale_(x1), scale_(y0)), Point(scale_(x1), scale_(y1)), Point(scale_(x0), scale_(y1))};
    return p;
}
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
    Polygons   overhangs;
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
// An island that starts at the layer `from` and is `n` layers tall, its first layer an overhang.
static void add_island(std::vector<LayerIn> &layers, size_t from, size_t n, const ExPolygon &island)
{
    for (size_t i = from; i < from + n && i < layers.size(); ++i) layers[i].lslices.emplace_back(island);
    append(layers[from].overhangs, to_polygons(island));
}

// A linear congruential generator (Knuth's MMIX constants).
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
    Point  pt(double r) { return Point(scale_(uni(-r, r)), scale_(uni(-r, r))); }
};

static const int FIRST_RANDOM  = 8;
static const int NUM_RANDOM    = 10;
static const int NUM_SCENARIOS = FIRST_RANDOM + NUM_RANDOM;

struct Scenario
{
    std::vector<std::pair<std::string, std::string>> config;
    std::vector<LayerIn>                              layers;
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
        s.layers[30].overhangs = {rect(-18, -6, -4.3, 6), rect(4.3, -6, 18, 6)};
        break;
    case 1:
        // Supports on the build plate only, no top gap (the soluble contract), thick slanted branches.
        s.config.emplace_back("support_on_build_plate_only", "1");
        s.config.emplace_back("support_top_z_distance", "0");
        s.config.emplace_back("tree_support_branch_diameter", "4");
        s.config.emplace_back("tree_support_branch_angle", "30");
        s.config.emplace_back("tree_support_branch_diameter_angle", "8");
        add_layers(s.layers, 8, lh, {ExPolygon(rect(-4, -4, 4, 4)), ExPolygon(rect(6, -12, 19, 12))});
        add_layers(s.layers, 20, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_layers(s.layers, 6, lh, {ExPolygon(rect(-18, -8, 18, 8))});
        s.layers[28].overhangs = {rect(-18, -8, -4.3, 8), rect(4.3, -8, 18, 8)};
        break;
    case 2:
        // A plate under the right arm, a top with a hole: the branches rest on the object. A raft, a wide tip
        // (the tips become roofs).
        s.config.emplace_back("raft_layers", "2");
        s.config.emplace_back("tree_support_tip_diameter", "1.2");
        s.config.emplace_back("support_object_xy_distance", "0.5");
        add_layers(s.layers, 8, lh, {ExPolygon(rect(-4, -4, 4, 4)), ExPolygon(rect(6, -12, 19, 12))});
        add_layers(s.layers, 18, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_layers(s.layers, 6, lh, {with_hole(rect(-18, -8, 18, 8), ngon(11, 0, 3, 16))});
        s.layers[26].overhangs = {rect(-18, -8, -4.3, 8)};
        append(s.layers[26].overhangs, to_polygons(with_hole(rect(4.3, -8, 18, 8), ngon(11, 0, 3, 16))));
        break;
    case 3:
        // Thin layers, gaps of two layers above and below, bottom interfaces, no top interfaces.
        lh = 0.16;
        s.config.emplace_back("layer_height", "0.16");
        s.config.emplace_back("initial_layer_print_height", "0.16");
        s.config.emplace_back("support_top_z_distance", "0.32");
        s.config.emplace_back("support_bottom_z_distance", "0.32");
        s.config.emplace_back("support_interface_top_layers", "0");
        s.config.emplace_back("tree_support_branch_angle", "50");
        s.config.emplace_back("tree_support_wall_count", "2");
        add_layers(s.layers, 10, lh, {ExPolygon(rect(-19, -12, 2, 12))});
        add_layers(s.layers, 22, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_layers(s.layers, 8, lh, {ExPolygon(rect(-18, -6, 18, 6))});
        s.layers[32].overhangs = {rect(-18, -6, -4.3, 6), rect(4.3, -6, 18, 6)};
        break;
    case 4:
        // A raft with an expansion, overhangs low over the raft and high above it.
        s.config.emplace_back("raft_layers", "3");
        s.config.emplace_back("raft_expansion", "2");
        add_layers(s.layers, 30, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_island(s.layers, 2, 4, ExPolygon(rect(4, -3, 7, 3)));
        add_island(s.layers, 3, 4, ExPolygon(rect(-12, -3, -4, 3)));
        add_island(s.layers, 22, 6, ExPolygon(rect(4, -6, 16, 6)));
        break;
    case 5:
        // Small islands in the air: thin, tiny and round overhangs, tips closer together.
        s.config.emplace_back("tree_support_branch_distance", "2");
        s.config.emplace_back("support_interface_top_layers", "2");
        add_layers(s.layers, 40, lh, {ExPolygon(rect(-3, -3, 3, 3))});
        add_island(s.layers, 12, 9, ExPolygon(rect(6, -0.6, 7.2, 0.6)));
        add_island(s.layers, 18, 7, with_hole(ngon(-9, 0, 3, 24), ngon(-9, 0, 1.6, 12)));
        add_island(s.layers, 26, 5, ExPolygon(rect(5, 5, 7.5, 6.2)));
        add_island(s.layers, 32, 4, ExPolygon(rect(-8, -9, -7.2, -8.4)));
        add_island(s.layers, 35, 4, ExPolygon(ngon(9, -8, 2.2, 5, 0.4)));
        break;
    case 6:
        // The top gap gives way to the distance to the object in the plane; a thin object, thin branches.
        s.config.emplace_back("top_z_overrides_xy_distance", "1");
        s.config.emplace_back("support_object_xy_distance", "0.8");
        s.config.emplace_back("support_interface_top_layers", "0");
        s.config.emplace_back("tree_support_branch_diameter", "1");
        s.config.emplace_back("tree_support_branch_distance", "2");
        add_layers(s.layers, 24, lh, {ExPolygon(rect(-2, -2, 2, 2))});
        add_layers(s.layers, 5, lh, {ExPolygon(ngon(0, 0, 11, 40))});
        append(s.layers[24].overhangs, to_polygons(with_hole(ngon(0, 0, 11, 40), rect(-2.3, -2.3, 2.3, 2.3))));
        break;
    case 7:
        // A ring in the air whose branches close up below it, a small island under its middle, a block on the bed.
        s.config.emplace_back("tree_support_branch_diameter", "4");
        s.config.emplace_back("tree_support_branch_diameter_angle", "12");
        s.config.emplace_back("support_interface_top_layers", "2");
        add_layers(s.layers, 46, lh, {ExPolygon(rect(-20, -14, -18, -12))});
        for (int i = 0; i < 10; ++i) s.layers[i].lslices.emplace_back(ExPolygon(rect(2, -5, 12, 5)));
        add_island(s.layers, 40, 6, with_hole(ngon(0, 0, 9, 36), ngon(0, 0, 6, 24)));
        add_island(s.layers, 30, 4, ExPolygon(rect(-1, -1, 1, 1)));
        break;
    default: {
        // Drawn from the generator seeded with the number of the scenario: a post in a corner, now and then a
        // block on the bed, one to four islands that start in the air, a support configuration.
        Lcg g{uint64_t(scenario) * 0x9E3779B97F4A7C15ULL + 2024};
        for (int i = 0; i < 4; ++i) g.next();
        s.config.emplace_back("tree_support_branch_diameter", num(g.of({1., 2., 3., 5.})));
        s.config.emplace_back("tree_support_branch_diameter_angle", num(g.of({0., 5., 8., 15.})));
        s.config.emplace_back("tree_support_branch_angle", num(g.of({20., 40., 50., 60.})));
        s.config.emplace_back("tree_support_branch_distance", num(g.of({2., 3., 5.})));
        s.config.emplace_back("support_interface_top_layers", num(g.of({0., 2., 3.})));
        if (g.chance(40)) s.config.emplace_back("support_on_build_plate_only", "1");
        if (g.chance(30)) s.config.emplace_back("tree_support_tip_diameter", num(g.of({0.4, 1.2, 2.})));
        if (g.chance(30)) s.config.emplace_back("support_object_xy_distance", num(g.of({0.1, 0.5, 1.})));
        if (g.chance(20)) s.config.emplace_back("support_top_z_distance", num(g.of({0., 0.1, 0.4, 0.6})));
        if (g.chance(20)) s.config.emplace_back("support_bottom_z_distance", num(g.of({0., 0.4})));
        if (g.chance(20)) s.config.emplace_back("raft_layers", num(g.of({1., 3.})));
        if (g.chance(20)) s.config.emplace_back("top_z_overrides_xy_distance", "1");
        if (g.chance(20)) s.config.emplace_back("support_line_width", num(g.of({0.3, 0.6})));
        if (g.chance(15)) {
            lh = g.of({0.12, 0.28});
            s.config.emplace_back("layer_height", num(lh));
            s.config.emplace_back("initial_layer_print_height", num(lh));
        }
        const int nl = g.of({24, 32, 40});
        add_layers(s.layers, nl, lh, {ExPolygon(rect(-20, -14, -18, -12))});
        if (g.chance(50)) {
            const int    n  = g.of({4, 8, 12});
            const double x0 = g.uni(-10, 0), y0 = g.uni(-8, 0), x1 = g.uni(1, 10), y1 = g.uni(1, 8);
            for (int i = 0; i < n; ++i) s.layers[i].lslices.emplace_back(ExPolygon(rect(x0, y0, x1, y1)));
        }
        const int islands = 1 + g.pick(4);
        for (int i = 0; i < islands; ++i) {
            const int    from = 10 + g.pick(nl - 6 - 10);
            const double cx = g.uni(-12, 12), cy = g.uni(-8, 8);
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
        break;
    }
    }
    return s;
}

static std::string STATE(const SupportElementState &s)
{
    return hB(s.to_buildplate) + hB(s.to_model_gracious) + hB(s.use_min_xy_dist) + hB(s.supports_roof) + hB(s.can_use_safe_radius) + hB(s.skip_ovalisation) + hB(s.lost) +
           hB(s.verylost) + hB(s.deleted) + hB(s.marked) + " " + hI(s.type) + " " + hD(s.radius) + " " + hF(s.print_z) + " " + hI(s.xy_min_distance) + " " + hI(s.target_height) +
           " " + PT(s.target_position) + " " + PT(s.next_position) + " " + hI(s.layer_idx) + " " + hI(s.effective_radius_height) + " " + hI(s.distance_to_top) + " " +
           PT(s.result_on_layer) + " " + hI(s.increased_to_model_radius) + " " + hD(s.elephant_foot_increases) + " " + hI(s.dont_move_until) + " " +
           hI(int(s.last_area_increase.type)) + " " + hI(s.last_area_increase.increase_speed) + " " + hB(s.last_area_increase.increase_radius) +
           hB(s.last_area_increase.no_error) + hB(s.last_area_increase.use_min_distance) + hB(s.last_area_increase.move) + " " + hI(s.missing_roof_layers);
}
static std::string ELEMENT(const SupportElement &e)
{
    std::string o = STATE(e.state) + " p " + hI((long long) e.parents.size());
    for (int32_t p : e.parents) o += " " + hI(p);
    return o + " a " + POLYS(e.influence_area);
}

// The elements of every layer, in their order.
static void dump_move_bounds(const std::string &tag, const std::vector<SupportElements> &move_bounds)
{
    for (size_t layer_idx = 0; layer_idx < move_bounds.size(); ++layer_idx) {
        if (move_bounds[layer_idx].empty()) continue;
        if (g_full) {
            out(tag + " layer " + hI((long long) layer_idx) + " " + hI((long long) move_bounds[layer_idx].size()));
            for (const SupportElement &e : move_bounds[layer_idx]) out(tag + " element " + hI((long long) layer_idx) + " " + ELEMENT(e));
        } else {
            std::string all;
            for (const SupportElement &e : move_bounds[layer_idx]) all += ELEMENT(e) + "\n";
            out(tag + " layer " + hI((long long) layer_idx) + " " + hI((long long) move_bounds[layer_idx].size()) + " #" + fnv(all));
        }
    }
}

static void dump_generator_layers(const std::string &tag, const SupportGeneratorLayersPtr &layers)
{
    out(tag + " " + hI((long long) layers.size()));
    for (size_t i = 0; i < layers.size(); ++i)
        if (const SupportGeneratorLayer *l = layers[i]; l != nullptr)
            outb(tag + " layer " + hI((long long) i) + " " + hI(int(l->layer_type)) + " " + hD(l->print_z) + " " + hD(l->bottom_z) + " " + hD(l->height), POLYS(l->polygons));
}

// The areas of draw_areas() by layer: of every area the index of its child in the layer below (-1 for none), the
// mark of its element and its polygons.
static void dump_draw_areas(const std::string &tag, const std::vector<SupportElements> &move_bounds, const std::vector<DrawArea> &linear_data, const std::vector<size_t> &linear_data_layers)
{
    for (size_t layer_idx = 0; layer_idx + 1 < linear_data_layers.size(); ++layer_idx) {
        const size_t begin = linear_data_layers[layer_idx], end = linear_data_layers[layer_idx + 1];
        if (begin == end) continue;
        std::string all;
        for (size_t j = begin; j < end; ++j) {
            const DrawArea &a     = linear_data[j];
            long long       child = -1;
            if (a.child_element != nullptr)
                for (size_t k = 0; k < move_bounds[layer_idx - 1].size(); ++k)
                    if (&move_bounds[layer_idx - 1][k] == a.child_element) child = (long long) k;
            const std::string line = hI(child) + " " + hB(a.element->state.marked) + " " + POLYS(a.polygons);
            if (g_full) out(tag + " area " + hI((long long) layer_idx) + " " + hI((long long) (j - begin)) + " " + line);
            all += line + "\n";
        }
        out(tag + " layer " + hI((long long) layer_idx) + " " + hI((long long) (end - begin)) + " #" + fnv(all));
    }
}

// A mesh: its vertices and its triangles.
static std::string ITS_S(const indexed_triangle_set &its)
{
    std::string o = hI((long long) its.vertices.size());
    for (const Vec3f &v : its.vertices) o += " " + hF(v.x()) + " " + hF(v.y()) + " " + hF(v.z());
    o += " " + hI((long long) its.indices.size());
    for (const auto &f : its.indices) o += " " + hI(f(0)) + " " + hI(f(1)) + " " + hI(f(2));
    return o;
}

// A copy of generator layers into a storage of their own (the layers of a scenario are shared by its stages).
static SupportGeneratorLayersPtr copy_generator_layers(const SupportGeneratorLayersPtr &src, SupportGeneratorLayerStorage &storage)
{
    SupportGeneratorLayersPtr dst(src.size(), nullptr);
    for (size_t i = 0; i < src.size(); ++i)
        if (const SupportGeneratorLayer *l = src[i]; l != nullptr) {
            SupportGeneratorLayer &n = storage.allocate(l->layer_type);
            n.print_z                = l->print_z;
            n.bottom_z               = l->bottom_z;
            n.height                 = l->height;
            n.idx_object_layer_above = l->idx_object_layer_above;
            n.idx_object_layer_below = l->idx_object_layer_below;
            n.bridging               = l->bridging;
            n.up                     = l->up;
            n.polygons               = l->polygons;
            dst[i]                   = &n;
        }
    return dst;
}

static void flush_fills(const std::string &tag)
{
    if (g_full)
        for (const std::string &l : g_fills) out(tag + " fill " + l);
    std::string body;
    for (const std::string &l : g_fills) body += l + "\n";
    out(tag + " fills " + hI((long long) g_fills.size()) + " #" + fnv(body));
    g_fills.clear();
}

static void run_scenario(int scenario)
{
    const std::string S  = "s" + hI(scenario);
    const Scenario    sc = make_scenario(scenario);
    Model             model;
    ModelObject      *o = model.add_object();
    o->name = "tee";
    o->add_volume(TriangleMesh(box_its(40, 28, 12, -20.f, -14.f, 0.f)));
    o->add_instance()->set_offset(Vec3d(100., 100., 0.));
    out("in " + S + " model 40 28 12 100 100");

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
        out("in " + S + " layer " + hD(l.print_z) + " " + hD(l.height) + " " + hD(layer->slice_z) + " " + EXPOLYS(l.lslices) + " " + POLYS(l.overhangs));
    }

    // group_meshes()
    TreeSupportSettings::soluble = false;
    std::vector<std::pair<TreeSupportSettings, std::vector<size_t>>> grouped_meshes = group_meshes(print, {0});
    TreeSupportSettings                                             &c              = grouped_meshes.front().first;
    out(S + " group_meshes " + hI((long long) grouped_meshes.size()) + " " + hI((long long) grouped_meshes.front().second.size()) + " " +
        hI((long long) grouped_meshes.front().second.front()) + " " + hB(TreeSupportSettings::soluble) + " " + hI(c.min_radius) + " " + hI(c.branch_radius) + " " +
        hI(c.xy_distance) + " " + hI(c.xy_min_distance) + " " + hI((long long) c.z_distance_top_layers) + " " + hI((long long) c.raft_layers.size()));

    // The build volume and the volumes, as generate_support_areas() makes them.
    Polygon bed = get_bed_shape_with_excluded_area(print.config());
    {
        const Vec3d plate_offset = print.get_plate_origin();
        bed.translate(Point(scale_(plate_offset(0)), scale_(plate_offset(1))) - obj->instances().front().shift);
    }
    out("in " + S + " bed " + hD(print.config().printable_height.value) + " " + POLY(bed));
    Pointfs bedptsf;
    std::transform(bed.points.begin(), bed.points.end(), std::back_inserter(bedptsf), [](const Point &p) { return unscale(p); });
    const BuildVolume build_volume{bedptsf, print.config().printable_height, {}, {}};
    TreeModelVolumes  volumes{*obj, build_volume, c.maximum_move_distance, c.maximum_move_distance_slow, 0, {}};

    // The overhangs, by the index of the support layer.
    const int             num_raft_layers = int(c.raft_layers.size());
    const int             num_layers      = int(obj->layer_count()) + num_raft_layers;
    std::vector<Polygons> overhangs(num_layers);
    for (size_t i = 0; i < sc.layers.size(); ++i) overhangs[i + num_raft_layers] = sc.layers[i].overhangs;

    // precalculate()
    size_t num_support_layers = precalculate(print, overhangs, c, {0}, volumes, [] {});
    out(S + " precalculate " + hI((long long) num_support_layers) + " " + hB(volumes.m_precalculated));
    const bool has_support = num_support_layers > 0;
    const bool has_raft    = c.raft_layers.size() > 0;
    num_support_layers     = std::max(num_support_layers, c.raft_layers.size());
    if (num_support_layers == 0) return;

    SupportParameters support_params(*obj);
    support_params.with_sheath = true;

    // The helpers that query the volumes.
    {
        const LayerIndex top = LayerIndex(num_support_layers) - 1;
        Lcg              g{uint64_t(scenario) * 77 + 5};
        Points           pts;
        for (int i = 0; i < 24; ++i) pts.emplace_back(g.pt(i < 12 ? 8. : 22.));
        std::string line = S + " get_avoidance_status";
        for (const Point &p : pts)
            for (coord_t r : {c.min_radius, c.branch_radius})
                for (LayerIndex l : {LayerIndex(1), top / 2, top})
                    for (coord_t xy : {coord_t(-1), coord_t(0), coord_t(c.xy_min_distance + 5000)}) line += " " + hI(int(get_avoidance_status(p, r, l, volumes, c, xy)));
        out("in " + S + " points " + PTS(pts));
        out(line);
        Polylines polylines;
        for (int i = 0; i < 6; ++i) {
            Polyline pl;
            const int n = 4 + g.pick(8);
            for (int k = 0; k < n; ++k) pl.points.emplace_back(g.pt(i < 3 ? 9. : 21.));
            polylines.emplace_back(std::move(pl));
        }
        out("in " + S + " polylines " + PLS(polylines));
        for (LayerIndex l : {LayerIndex(1), top / 2, top})
            for (coord_t xy : {coord_t(-1), coord_t(c.xy_min_distance + 5000)}) {
                const LineInformations lines = convert_lines_to_internal(volumes, c, polylines, l, xy);
                outb(S + " convert_lines_to_internal " + hI(l) + " " + hI(xy), LINES(lines));
                std::string ev;
                for (const LineInformation &li : lines)
                    for (const auto &p : li) ev += hB(evaluate_point_for_next_layer_function(volumes, c, size_t(l), p, xy));
                outb(S + " evaluate_point_for_next_layer_function " + hI(l) + " " + hI(xy), ev);
                const auto split = split_lines(lines, [&](const std::pair<Point, LineStatus> &p) { return evaluate_point_for_next_layer_function(volumes, c, size_t(l), p, xy); });
                outb(S + " split_lines " + hI(l) + " " + hI(xy), LINES(split.first) + " | " + LINES(split.second));
            }
        // generate_support_infill_lines()
        g_fills.clear();
        const Polygons areas[2] = {{rect(-9, -5, 3, 4), rect(5, 5, 9, 9)}, to_polygons(with_hole(ngon(1, 2, 10, 30), rect(-2, -2, 3, 3)))};
        for (int a = 0; a < 2; ++a) out("in " + S + " area " + POLYS(areas[a]));
        for (int a = 0; a < 2; ++a)
            for (bool roof : {false, true})
                for (LayerIndex l : {LayerIndex(4), LayerIndex(5)})
                    for (coord_t dist : {coord_t(100000), coord_t(254321)})
                        outb(S + " generate_support_infill_lines " + hI(a) + " " + hB(roof) + " " + hI(l) + " " + hI(dist), PLS(generate_support_infill_lines(areas[a], support_params, roof, l, dist)));
        flush_fills(S + " generate_support_infill_lines");
    }

    // generate_initial_areas(), as generate_support_areas() calls it.
    SupportGeneratorLayerStorage layer_storage;
    SupportGeneratorLayersPtr    top_contacts;
    SupportGeneratorLayersPtr    interface_layers;
    SupportGeneratorLayersPtr    base_interface_layers;
    if (support_params.has_top_contacts || has_raft) top_contacts.assign(num_support_layers, nullptr);
    if (support_params.has_interfaces() || has_raft) interface_layers.assign(num_support_layers, nullptr);
    if (support_params.has_base_interfaces() || has_raft) base_interface_layers.assign(num_support_layers, nullptr);
    InterfacePlacer interface_placer{obj->slicing_parameters(), support_params, c, layer_storage, top_contacts, interface_layers, base_interface_layers};
    out(S + " support_params " + hB(support_params.has_top_contacts) + " " + hB(support_params.has_interfaces()) + " " + hB(support_params.has_base_interfaces()) + " " +
        hB(support_params.soluble_interface) + " " + hI((long long) support_params.num_top_interface_layers) + " " + hB(has_support) + " " + hB(has_raft) + " " +
        hI((long long) num_support_layers));
    if (! has_support) return;
    std::vector<SupportElements> move_bounds(num_support_layers);
    g_fills.clear();
    generate_initial_areas(*obj, volumes, c, overhangs, move_bounds, interface_placer, [] {});
    flush_fills(S + " initial");
    dump_move_bounds(S + " initial", move_bounds);
    dump_generator_layers(S + " initial top_contacts", top_contacts);
    dump_generator_layers(S + " initial interface_layers", interface_layers);
    dump_generator_layers(S + " initial base_interface_layers", base_interface_layers);

    // create_layer_pathing() (with increase_areas_one_layer(), increase_single_area() and merge_influence_areas()).
    // merge_influence_areas() sizes its first buckets by tbb::this_task_arena::max_concurrency(), which is the
    // number of threads of the machine: the scenarios run it in an arena of one slot and of eight slots in
    // turn (still on one thread), so that the vectors do not depend on the machine they were made on.
    {
        tbb::task_arena arena(scenario % 2 == 0 ? 1 : 8);
        arena.execute([&] {
            out("in " + S + " threads " + hI(tbb::this_task_arena::max_concurrency()));
            create_layer_pathing(volumes, c, move_bounds, [] {});
        });
    }
    dump_move_bounds(S + " pathing", move_bounds);

    // create_nodes_from_area() (with set_points_on_areas(), set_to_model_contact_simple(),
    // set_to_model_contact_to_model_gracious() and remove_deleted_elements()).
    create_nodes_from_area(volumes, c, move_bounds, [] {});
    dump_move_bounds(S + " nodes", move_bounds);

    // The areas of the branches: generate_branch_areas() and smooth_branch_areas() on a copy of the elements, their
    // areas put together as draw_areas() does it (TreeSupport3D.cpp:3296-3326), then draw_areas() itself (with
    // drop_non_gracious_areas() and finalize_interface_and_support_areas()).
    {
        std::vector<SupportElements> mb = move_bounds;
        std::vector<DrawArea>        linear_data;
        std::vector<size_t>          linear_data_layers;
        {
            std::vector<std::pair<SupportElement *, SupportElement *>> map_downwards_old;
            std::vector<std::pair<SupportElement *, SupportElement *>> map_downwards_new;
            for (LayerIndex layer_idx = 0; layer_idx < LayerIndex(mb.size()); ++layer_idx) {
                SupportElements *layer_above = layer_idx + 1 < LayerIndex(mb.size()) ? &mb[layer_idx + 1] : nullptr;
                map_downwards_new.clear();
                linear_data_layers.emplace_back(linear_data.size());
                std::sort(map_downwards_old.begin(), map_downwards_old.end(), [](auto &l, auto &r) { return l.first < r.first; });
                for (SupportElement &elem : mb[layer_idx]) {
                    SupportElement *child = nullptr;
                    if (layer_idx > 0) {
                        auto it = std::lower_bound(map_downwards_old.begin(), map_downwards_old.end(), &elem, [](auto &l, const SupportElement *r) { return l.first < r; });
                        if (it != map_downwards_old.end() && it->first == &elem) child = it->second;
                    }
                    for (int32_t parent_idx : elem.parents) {
                        SupportElement &parent = (*layer_above)[parent_idx];
                        if (parent.state.result_on_layer_is_set()) map_downwards_new.emplace_back(&parent, &elem);
                    }
                    linear_data.push_back({&elem, child});
                }
                std::swap(map_downwards_old, map_downwards_new);
            }
            linear_data_layers.emplace_back(linear_data.size());
        }
        generate_branch_areas(volumes, c, mb, linear_data, [] {});
        dump_draw_areas(S + " branch", mb, linear_data, linear_data_layers);
        smooth_branch_areas(c, mb, linear_data, linear_data_layers, [] {});
        dump_draw_areas(S + " smooth", mb, linear_data, linear_data_layers);
        // The union of the areas of every layer and smooth_outward() of it (MutablePolygon.cpp), the first two
        // steps finalize_interface_and_support_areas() takes with the areas of a layer: lines of their own, to tell
        // the smoothing from the rest of that function.
        for (size_t layer_idx = 0; layer_idx + 1 < linear_data_layers.size(); ++layer_idx) {
            Polygons all;
            for (size_t j = linear_data_layers[layer_idx]; j < linear_data_layers[layer_idx + 1]; ++j) append(all, linear_data[j].polygons);
            if (all.empty()) continue;
            const Polygons u = union_(all);
            outb(S + " layer_union " + hI((long long) layer_idx), POLYS(u));
            outb(S + " smooth_outward " + hI((long long) layer_idx), POLYS(smooth_outward(u, c.support_line_width)));
        }
    }
    // The organic part starts from the elements and the top contacts as they are here, before draw_areas().
    const std::vector<SupportElements> move_bounds_nodes = move_bounds;
    SupportGeneratorLayerStorage       layer_storage_nodes;
    const SupportGeneratorLayersPtr    top_contacts_nodes = copy_generator_layers(top_contacts, layer_storage_nodes);

    SupportGeneratorLayersPtr bottom_contacts;
    SupportGeneratorLayersPtr intermediate_layers(num_support_layers, nullptr);
    if (support_params.has_bottom_contacts) bottom_contacts.assign(num_support_layers, nullptr);
    draw_areas(*obj, volumes, c, overhangs, move_bounds, bottom_contacts, top_contacts, intermediate_layers, layer_storage, [] {});
    dump_generator_layers(S + " draw bottom_contacts", bottom_contacts);
    dump_generator_layers(S + " draw top_contacts", top_contacts);
    dump_generator_layers(S + " draw intermediate_layers", intermediate_layers);

    // draw_branches(): organic_smooth_branches_avoid_collisions() moves the points of the elements, then the mesh
    // of all branches (extrude_branch() of every path between two bifurcations).
    {
        std::vector<SupportElements> mb   = move_bounds_nodes;
        const indexed_triangle_set   mesh = draw_branches(*obj, volumes, c, mb, [] {});
        dump_move_bounds(S + " organic_smooth", mb);
        outb(S + " draw_branches " + hI((long long) mesh.vertices.size()) + " " + hI((long long) mesh.indices.size()), ITS_S(mesh));

        // The slicer on that mesh at the middle of every layer, as organic_draw_branches() slices the mesh of a
        // branch: slice_mesh() in the positive mode, and slice_mesh_ex() with the closing radius of the object as
        // slice_branches() calls it.
        {
            std::vector<float> zs;
            for (size_t l = 0; l < mb.size(); ++l) zs.emplace_back(float(layer_mid_z(obj->slicing_parameters(), c, LayerIndex(l))));
            MeshSlicingParams mp;
            mp.mode = MeshSlicingParams::SlicingMode::Positive;
            const std::vector<Polygons> sl = slice_mesh(mesh, zs, mp, [] {});
            for (size_t l = 0; l < sl.size(); ++l)
                if (! sl[l].empty()) outb(S + " slice_mesh " + hI((long long) l) + " " + hF(zs[l]), POLYS(sl[l]));
            MeshSlicingParamsEx mpe;
            mpe.closing_radius = float(obj->config().slice_closing_radius.value);
            mpe.mode           = MeshSlicingParams::SlicingMode::Positive;
            const std::vector<ExPolygons> sle = slice_mesh_ex(mesh, zs, mpe, [] {});
            for (size_t l = 0; l < sle.size(); ++l)
                if (! sle[l].empty()) outb(S + " slice_mesh_ex " + hI((long long) l) + " " + hF(zs[l]), EXPOLYS(sle[l]));
        }

        // slice_branches() of that mesh (the pin does not call it; with finalize_interface_and_support_areas()).
        SupportGeneratorLayerStorage storage;
        SupportGeneratorLayersPtr    top = copy_generator_layers(top_contacts_nodes, storage);
        SupportGeneratorLayersPtr    bottom;
        SupportGeneratorLayersPtr    intermediate(num_support_layers, nullptr);
        if (support_params.has_bottom_contacts) bottom.assign(num_support_layers, nullptr);
        slice_branches(*obj, volumes, c, overhangs, mb, mesh, bottom, top, intermediate, storage, [] {});
        dump_generator_layers(S + " slice_branches bottom_contacts", bottom);
        dump_generator_layers(S + " slice_branches top_contacts", top);
        dump_generator_layers(S + " slice_branches intermediate_layers", intermediate);
    }

    // organic_draw_branches(), what generate_support_areas() draws the branches with: the elements (their points
    // and marks), the cooldown areas, the bottom contacts, the top contacts and the intermediate layers.
    {
        std::vector<SupportElements> mb = move_bounds_nodes;
        SupportGeneratorLayerStorage storage;
        SupportGeneratorLayersPtr    top = copy_generator_layers(top_contacts_nodes, storage);
        SupportGeneratorLayersPtr    top_interfaces(interface_layers.size(), nullptr);
        SupportGeneratorLayersPtr    top_base_interfaces(base_interface_layers.size(), nullptr);
        SupportGeneratorLayersPtr    bottom;
        SupportGeneratorLayersPtr    intermediate(num_support_layers, nullptr);
        if (support_params.has_bottom_contacts) bottom.assign(num_support_layers, nullptr);
        InterfacePlacer         placer{obj->slicing_parameters(), support_params, c, storage, top, top_interfaces, top_base_interfaces};
        std::vector<ExPolygons> cooldown_areas(num_support_layers);
        organic_draw_branches(*obj, volumes, c, mb, bottom, top, placer, intermediate, storage, cooldown_areas, [] {});
        dump_move_bounds(S + " organic elements", mb);
        for (size_t i = 0; i < cooldown_areas.size(); ++i)
            if (! cooldown_areas[i].empty()) outb(S + " organic cooldown " + hI((long long) i) + " " + hI((long long) cooldown_areas[i].size()), EXPOLYS(cooldown_areas[i]));
        dump_generator_layers(S + " organic bottom_contacts", bottom);
        dump_generator_layers(S + " organic top_contacts", top);
        dump_generator_layers(S + " organic intermediate_layers", intermediate);
        // organic_lightning_infill() (TreeSupport3D.cpp:4388-4792) on the layers organic_draw_branches() left:
        // the size of the result, the number of layers with lines, and the chained lines of each such layer.
        {
            std::vector<Polylines> lightning_infill_lines;
            organic_lightning_infill(*obj, volumes, c, bottom, top, intermediate, lightning_infill_lines, [] {});
            size_t n = 0;
            for (const Polylines &pl : lightning_infill_lines)
                if (! pl.empty()) ++n;
            out(S + " lightning " + hI((long long) lightning_infill_lines.size()) + " " + hI((long long) n));
            for (size_t i = 0; i < lightning_infill_lines.size(); ++i)
                if (! lightning_infill_lines[i].empty()) outb(S + " lightning layer " + hI((long long) i), PLS(lightning_infill_lines[i]));
        }
    }

    // generate_overhangs() (TreeSupport3D.cpp:226-345) over the layers of the scenario: the overhangs by the index
    // of the support layer and the extrudable slices it leaves in the layers. Last, as it rewrites
    // lslices_extrudable.
    {
        const std::vector<Polygons> go = generate_overhangs(c, *obj, [] {});
        out(S + " generate_overhangs " + hI((long long) go.size()));
        for (size_t i = 0; i < go.size(); ++i)
            if (! go[i].empty()) outb(S + " generate_overhangs layer " + hI((long long) i), POLYS(go[i]));
        for (size_t i = 0; i < obj->layer_count(); ++i)
            outb(S + " generate_overhangs extrudable " + hI((long long) i), EXPOLYS(obj->get_layer(int(i))->lslices_extrudable));
    }
}

// The helpers that take no volumes.
static void run_helpers()
{
    Lcg g{20240919};
    // polyline_sample_next_point_at_distance(), ensure_maximum_distance_polyline()
    for (int i = 0; i < 60; ++i) {
        Polyline pl;
        const int n = 2 + g.pick(11);
        Point     prev = g.pt(15.);
        for (int k = 0; k < n; ++k) {
            // Now and then a very short segment, a repeated point.
            const int kind = g.pick(10);
            Point     p    = kind == 0 ? prev + Point(g.pick(900) - 450, g.pick(900) - 450) : kind == 1 ? prev : g.pt(15.);
            pl.points.emplace_back(p);
            prev = p;
        }
        if (g.chance(40)) pl.points.emplace_back(pl.points.front());
        out("in polyline " + PTS(pl.points));
        std::string line = "polyline_sample_next_point_at_distance " + hI(i);
        for (size_t start_idx = 0; start_idx + 1 < pl.points.size(); start_idx += 2)
            for (double dist : {30000., 150000.5, 799999.25, 2500000.}) {
                const Point start_pt = start_idx % 4 == 0 ? pl.points[start_idx] : Point((pl.points[start_idx] + pl.points[start_idx + 1]) / 2);
                const auto  r        = polyline_sample_next_point_at_distance(pl.points, start_pt, start_idx, dist);
                line += r ? " " + PT(r->first) + " " + hI((long long) r->second) : std::string(" -");
            }
        out(line);
        for (double distance : {50000., 200000., 612345.5})
            for (size_t min_points : {size_t(0), size_t(1), size_t(2), size_t(3), size_t(5)})
                out("ensure_maximum_distance_polyline " + hI(i) + " " + hD(distance) + " " + hI((long long) min_points) + " " +
                    PLS(ensure_maximum_distance_polyline({pl, Polyline(), pl}, distance, min_points)));
    }
    // move_inside(), move_inside_if_outside()
    for (int i = 0; i < 40; ++i) {
        Polygons polygons;
        const int np = 1 + g.pick(3);
        for (int k = 0; k < np; ++k) {
            const double cx = g.uni(-10, 10), cy = g.uni(-10, 10);
            switch (g.pick(4)) {
            case 0: polygons.emplace_back(rect(cx - g.uni(1, 6), cy - g.uni(1, 6), cx + g.uni(1, 6), cy + g.uni(1, 6))); break;
            case 1: polygons.emplace_back(ngon(cx, cy, g.uni(1, 8), g.of({3, 5, 12, 25}), g.uni(0, 3))); break;
            case 2: {
                Polygon star;
                const int    n  = 2 * (3 + g.pick(5));
                const double r0 = g.uni(1, 4), r1 = g.uni(4, 9);
                for (int j = 0; j < n; ++j) {
                    const double a = 2. * M_PI * j / n, r = (j % 2) ? r0 : r1;
                    star.points.emplace_back(Point(scale_(cx + r * std::round(std::cos(a) * 1e6) / 1e6), scale_(cy + r * std::round(std::sin(a) * 1e6) / 1e6)));
                }
                polygons.emplace_back(star);
                break;
            }
            default: {
                // A clockwise polygon (a hole), with a repeated point.
                Polygon p = ngon(cx, cy, g.uni(1, 5), 8);
                p.points.emplace_back(p.points.back());
                p.reverse();
                polygons.emplace_back(p);
            }
            }
        }
        if (i % 9 == 8) polygons.emplace_back(Polygon{Points{g.pt(5.)}});
        out("in polygons " + POLYS(polygons));
        std::string line = "move_inside " + hI(i);
        for (int k = 0; k < 12; ++k) {
            const Point from0 = g.pt(14.);
            for (int distance : {0, 500, -500, 30000, -30000})
                for (int64_t max_dist2 : {std::numeric_limits<int64_t>::max(), int64_t(200000) * 200000}) {
                    Point              from = from0;
                    const unsigned int r    = move_inside(polygons, from, distance, max_dist2);
                    line += " " + hI((long long) r) + " " + PT(from);
                }
            line += " " + PT(move_inside_if_outside(polygons, from0));
            out("in from " + PT(from0));
        }
        out(line);
    }
    // getEffectiveDTT(), support_element_radius(), support_element_collision_radius(),
    // support_element_model_collision_radius(), SupportElementState::propagate_down()
    {
        TreeSupportMeshGroupSettings m;
        m.support_tree_branch_diameter_angle = 0.14;
        m.support_tree_tip_diameter          = 60000;
        m.support_xy_distance_overhang       = 15000;
        SlicingParameters sp;
        sp.layer_height = 0.2;
        TreeSupportSettings c(m, sp);
        out("in settings " + hI(c.min_radius) + " " + hI(c.branch_radius) + " " + hI((long long) c.tip_layers) + " " + hI((long long) c.increase_radius_until_layer) + " " +
            hD(c.branch_radius_increase_per_layer) + " " + hD(c.bp_radius_increase_per_layer) + " " + hI(c.xy_min_distance));
        for (int i = 0; i < 40; ++i) {
            SupportElementState s;
            s.target_height           = g.pick(60);
            s.target_position         = g.pt(10.);
            s.next_position           = g.pt(10.);
            s.layer_idx               = 1 + g.pick(50);
            s.effective_radius_height = uint32_t(g.pick(40));
            s.distance_to_top         = uint32_t(g.pick(40));
            s.result_on_layer         = g.chance(50) ? g.pt(10.) : s.result_on_layer;
            s.increased_to_model_radius = g.pick(40000);
            s.elephant_foot_increases = g.pick(4) == 0 ? 0. : g.uni(0, 9);
            s.dont_move_until         = uint32_t(g.pick(6));
            s.missing_roof_layers     = uint32_t(g.pick(4));
            s.use_min_xy_dist         = g.chance(50);
            s.skip_ovalisation        = g.chance(50);
            s.xy_min_distance         = g.pick(30000);
            out("in state " + STATE(s));
            SupportElement e(s, Polygons{});
            const SupportElementState d = SupportElementState::propagate_down(s);
            out("support_element " + hI(i) + " " + hI((long long) getEffectiveDTT(c, s)) + " " + hI(support_element_radius(c, s)) + " " + hI(support_element_collision_radius(c, s)) +
                " " + hI(support_element_model_collision_radius(c, s)) + " " + hI(support_element_radius(c, e)) + " " + hI(support_element_collision_radius(c, e)) + " " +
                hB(s.result_on_layer_is_set()) + " " + STATE(d));
        }
    }
    out("tiny_area_threshold " + hD(tiny_area_threshold));
    // circles_intersect()
    {
        auto D3 = [](const Vec3d &v) { return hD(v.x()) + " " + hD(v.y()) + " " + hD(v.z()); };
        std::string line = "circles_intersect ";
        for (int i = 0; i < 300; ++i) {
            const Vec3d p1(g.uni(-5, 5), g.uni(-5, 5), g.uni(0, 5));
            const Vec3d p2 = p1 + Vec3d(g.uni(-2, 2), g.uni(-2, 2), g.uni(0, 2));
            const Vec3d n1 = Vec3d(g.uni(-1, 1), g.uni(-1, 1), g.uni(0.2, 1)).normalized();
            // Now and then the same plane normal.
            Vec3d       n2 = Vec3d(g.uni(-1, 1), g.uni(-1, 1), g.uni(0.2, 1)).normalized();
            if (i % 37 == 5) n2 = n1;
            if (n1.dot(n2) < 0) n2 = -n2;
            const double r1 = g.uni(0.2, 3), r2 = g.uni(0.2, 3);
            out("in circles " + D3(p1) + " " + D3(n1) + " " + hD(r1) + " " + D3(p2) + " " + D3(n2) + " " + hD(r2));
            line += circles_intersect(p1, n1, r1, p2, n2, r2) ? "1" : "0";
        }
        out(line);
    }
    // discretize_circle(), triangulate_fan(), triangulate_strip(): two circles, a fan on each and the strip
    // between them.
    {
        auto F3 = [](const Vec3f &v) { return hF(v.x()) + " " + hF(v.y()) + " " + hF(v.z()); };
        for (int i = 0; i < 60; ++i) {
            const Vec3f c1(float(g.uni(-5, 5)), float(g.uni(-5, 5)), float(g.uni(0, 5)));
            const Vec3f c2 = c1 + Vec3f(float(g.uni(-1, 1)), float(g.uni(-1, 1)), float(g.uni(0.1, 1)));
            Vec3f       n1 = Vec3f(float(g.uni(-1, 1)), float(g.uni(-1, 1)), float(g.uni(0.2, 1))).normalized();
            if (i % 11 == 4) n1 = Vec3f(0.f, -1.f, 0.f);
            Vec3f       n2 = Vec3f(float(g.uni(-1, 1)), float(g.uni(-1, 1)), float(g.uni(0.2, 1))).normalized();
            if (i % 5 == 0) n2 = Vec3f(0.f, 0.f, 1.f);
            const float r1 = float(g.uni(0.3, 4)), r2 = float(g.uni(0.3, 4));
            const float eps = g.of({0.015f, 0.05f, 0.2f});
            out("in mesh " + F3(c1) + " " + F3(n1) + " " + hF(r1) + " " + F3(c2) + " " + F3(n2) + " " + hF(r2) + " " + hF(eps));
            indexed_triangle_set its;
            its.vertices.emplace_back(c1);
            const std::pair<int, int> a = discretize_circle(c1, n1, r1, eps, its.vertices);
            const std::pair<int, int> b = discretize_circle(c2, n2, r2, eps, its.vertices);
            triangulate_fan<false>(its, 0, a.first, a.second);
            triangulate_strip(its, a.first, a.second, b.first, b.second);
            its.vertices.emplace_back(c2);
            triangulate_fan<true>(its, int(its.vertices.size()) - 1, b.first, b.second);
            std::string line = "mesh " + hI(i) + " " + hI(a.first) + " " + hI(a.second) + " " + hI(b.first) + " " + hI(b.second) + " " + hI((long long) its.vertices.size());
            for (const Vec3f &v : its.vertices) line += " " + F3(v);
            line += " " + hI((long long) its.indices.size());
            for (const auto &f : its.indices) line += " " + hI(f(0)) + " " + hI(f(1)) + " " + hI(f(2));
            out(line);
        }
    }
    // discretize_polygon()
    for (int i = 0; i < 12; ++i) {
        const Polygons polys{ngon(g.uni(-5, 5), g.uni(-5, 5), g.uni(0.5, 4), g.of({3, 7, 20}), g.uni(0, 3)), rect(0, 0, 1, 1)};
        const Vec3f    center(float(g.uni(-5, 5)), float(g.uni(-5, 5)), float(g.uni(0, 9)));
        out("in dpoly " + hF(center.z()) + " " + POLYS(polys));
        std::vector<Vec3f> pts{center};
        const std::pair<int, int> r = discretize_polygon(center, polys, pts);
        std::string line = "discretize_polygon " + hI(i) + " " + hI(r.first) + " " + hI(r.second);
        // Not the centre, of which the function reads the height only.
        for (size_t k = 1; k < pts.size(); ++k) line += " " + hF(pts[k].x()) + " " + hF(pts[k].y()) + " " + hF(pts[k].z());
        out(line);
    }
    // section_circles_intersect()
    {
        auto D3 = [](const Vec3d &v) { return hD(v.x()) + " " + hD(v.y()) + " " + hD(v.z()); };
        std::string line = "section_circles_intersect ";
        for (int i = 0; i < 600; ++i) {
            const Vec3d p1(g.uni(-5, 5), g.uni(-5, 5), g.uni(0, 5));
            const Vec3d p2 = p1 + Vec3d(g.uni(-1.5, 1.5), g.uni(-1.5, 1.5), g.uni(0, 0.5));
            const Vec3d n1 = Vec3d(g.uni(-1, 1), g.uni(-1, 1), g.uni(0.2, 1)).normalized();
            Vec3d       n2 = Vec3d(g.uni(-1, 1), g.uni(-1, 1), g.uni(0.2, 1)).normalized();
            // The same normal, normals a tilt of about 1e-4 apart (the threshold of the test) and a small tilt.
            if (i % 9 == 3) n2 = n1;
            if (i % 9 == 5) n2 = (n1 + Vec3d(g.uni(-2, 2) * 1e-4, g.uni(-2, 2) * 1e-4, 0.)).normalized();
            if (i % 9 == 7) n2 = (n1 + Vec3d(g.uni(-0.3, 0.3), g.uni(-0.3, 0.3), 0.)).normalized();
            const double r1 = g.uni(0.2, 3), r2 = g.uni(0.2, 3);
            out("in sections " + D3(p1) + " " + D3(n1) + " " + hD(r1) + " " + D3(p2) + " " + D3(n2) + " " + hD(r2));
            line += section_circles_intersect(p1, n1, r1, p2, n2, r2) ? "1" : "0";
        }
        out(line);
    }
    // extrude_branch_tube() with the four pairs of end caps, extrude_branch() with and without the descriptors of
    // the split caps: paths of elements one layer apart, every fourth over two raft layers.
    {
        auto D3 = [](const Vec3d &v) { return hD(v.x()) + " " + hD(v.y()) + " " + hD(v.z()); };
        auto ITS = [](const indexed_triangle_set &its) {
            std::string o = hI((long long) its.vertices.size());
            for (const Vec3f &v : its.vertices) o += " " + hF(v.x()) + " " + hF(v.y()) + " " + hF(v.z());
            o += " " + hI((long long) its.indices.size());
            for (const auto &f : its.indices) o += " " + hI(f(0)) + " " + hI(f(1)) + " " + hI(f(2));
            return o;
        };
        TreeSupportMeshGroupSettings m;
        m.support_tree_branch_diameter_angle = 0.14;
        m.support_tree_tip_diameter          = 60000;
        m.support_xy_distance_overhang       = 15000;
        SlicingParameters sp;
        sp.layer_height = 0.2;
        const TreeSupportSettings c(m, sp);
        TreeSupportSettings       c_raft(m, sp);
        c_raft.raft_layers = {0.1, 0.2};
        // What layer_z() reads of the slicing parameters.
        SlicingParameters spz;
        spz.layer_height              = 0.2;
        spz.first_object_layer_height = 0.25;
        spz.object_print_z_min        = 0.;
        SlicingParameters spz_raft = spz;
        spz_raft.object_print_z_min = 0.3;
        const std::vector<SupportElements> no_move_bounds;
        for (int i = 0; i < 80; ++i) {
            const int    n    = 2 + g.pick(7);
            const bool   raft = i % 4 == 3;
            const double step = g.of({0.05, 0.3, 1., 2.5});
            std::vector<SupportElement> elems;
            Point          pos   = g.pt(20.);
            const int      layer = raft ? g.pick(2) : 1 + g.pick(30);
            const uint32_t dtt   = uint32_t(n + g.pick(40));
            for (int k = 0; k < n; ++k) {
                SupportElementState s;
                s.layer_idx               = layer + k;
                s.result_on_layer         = pos;
                s.target_position         = pos;
                s.next_position           = pos;
                s.distance_to_top         = dtt - uint32_t(k);
                s.effective_radius_height = uint32_t(g.pick(30));
                s.elephant_foot_increases = g.pick(3) == 0 ? g.uni(0, 6) : 0.;
                elems.emplace_back(s, Polygons{});
                pos += Point(scale_(g.uni(-step, step)), scale_(g.uni(-step, step)));
            }
            out("in path " + hI(n) + " " + hB(raft));
            std::vector<const SupportElement*> path;
            for (const SupportElement &e : elems) {
                out("in pstate " + STATE(e.state));
                path.emplace_back(&e);
            }
            const TreeSupportSettings &cc = raft ? c_raft : c;
            const SlicingParameters   &ss = raft ? spz_raft : spz;
            for (int caps = 0; caps < 4; ++caps) {
                indexed_triangle_set its;
                // Now and then a mesh that is not empty: the tube is appended.
                if (caps == 3) its.vertices.emplace_back(Vec3f(1.f, 2.f, 3.f));
                const std::pair<float, float> span = extrude_branch_tube(path, cc, ss, no_move_bounds, its, (caps & 1) ? TubeEndCap::FlatDisk : TubeEndCap::HemisphereMesh,
                                                                         (caps & 2) ? TubeEndCap::FlatDisk : TubeEndCap::HemisphereMesh);
                outb("extrude_branch_tube " + hI(i) + " " + hI(caps) + " " + hF(span.first) + " " + hF(span.second) + " " + hI((long long) its.vertices.size()), ITS(its));
            }
            {
                indexed_triangle_set         its;
                std::vector<ExtrudeSplitCap> split_caps;
                const std::pair<float, float> span = extrude_branch(path, cc, ss, no_move_bounds, its, &split_caps);
                std::string body = ITS(its);
                for (const ExtrudeSplitCap &cap : split_caps) body += " " + D3(cap.center) + " " + D3(cap.normal) + " " + hD(cap.radius) + " " + hB(cap.is_bottom);
                outb("extrude_branch " + hI(i) + " " + hF(span.first) + " " + hF(span.second) + " " + hI((long long) its.vertices.size()) + " " + hI((long long) split_caps.size()), body);
                indexed_triangle_set its2;
                const std::pair<float, float> span2 = extrude_branch(path, cc, ss, no_move_bounds, its2);
                outb("extrude_branch_plain " + hI(i) + " " + hF(span2.first) + " " + hF(span2.second), ITS(its2));
            }
        }
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
    else {
        run_helpers();
        for (int scenario = 0; scenario < NUM_SCENARIOS; ++scenario) run_scenario(scenario);
    }
    std::fclose(g_out);
    return 0;
}
