// Generator of the vectors of tests/tree_support_native_parity.rs: libslic3r's Support/TreeSupport.cpp
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
// TreeSupport.cpp is compiled into this file (its helpers are static), with the private members of
// TreeSupport, TreeSupportData, PrintObject and Layer made public. The file is compiled with the
// compiler's default floating point contraction, as the engine is.
//
// Every scenario is a Print of one box-shaped object made by `Print::apply`, whose object layers are
// then filled in by hand (a stem with a wide top, the overhangs of the top with their types), so the
// tree support runs without the slicing steps: the constructor, generate_contact_points(),
// plan_layer_heights(), drop_nodes() and smooth_nodes(), with tbb limited to one thread.
//
// Lines that start with "in" are the inputs the Rust test reads back (the configuration keys, the
// layers, the arguments of the helpers); the others are results it has to reproduce. Tokens are
// separated by one space; a double is its 16 hexadecimal digits, a float its 8, an integer decimal.
// The vectors were made on macOS arm64 with Apple clang and Eigen 5.0.1.
//
// The scenarios from 4 on write the long lines (areas, the nodes of a layer) as "#" and the FNV-1a
// hash of the text the line would carry, which keeps the vectors small; `./gen vectors.txt full`
// writes every line in full, for finding the first token that differs (the test reads such a file
// when TREE_SUPPORT_PARITY_VECTORS names it).

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
#include <tbb/spin_mutex.h>

#define private public
#define protected public
#include "libslic3r/Support/TreeSupport.cpp"
#undef protected
#undef private
#include "libslic3r/Model.hpp"
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
// Every line in full (the second argument "full").
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
// A line of a head and a long body: the body, or "#" and its hash where the scenario hashes.
static void outb(const std::string &head, const std::string &body, bool hashed) { out(hashed && ! g_full ? head + " #" + fnv(body) : head + " " + body); }
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
        double a = phase + 2. * M_PI * i / n;
        p.points.emplace_back(Point(scale_(cx + r * std::cos(a)), scale_(cy + r * std::sin(a))));
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

static std::string node_ref(const SupportNode *n)
{
    if (n == nullptr) return "-";
    return PT(n->position) + " " + hD(n->print_z);
}

static std::string NODE(const SupportNode &n)
{
    std::string o = PT(n.position) + " " + hD(n.print_z) + " " + hD(n.height) + " " + hD(n.radius) + " " + hD(n.dist_mm_to_top) + " " +
                    hI(n.distance_to_top) + " " + hI(n.support_roof_layers_below) + " " + hI(n.obj_layer_nr) + " " + hB(n.to_buildplate) + " " +
                    hB(n.valid) + " " + hI((int) n.type) + " " + hB(n.is_corner) + " " + hB(n.is_sharp_tail) + " " + hB(n.fading) + " " +
                    hB(n.need_extra_wall) + " " + hB(n.is_processed) + " " + hF(n.carried_mass) + " " + hD(n.max_move_dist) + " " + PT(n.movement) + " " +
                    PT(n.orig_pos) + " " + hD(n.target_radius) + " " + hD(n.overhang_degree) + " " + PT(n.skin_direction) + " p " +
                    node_ref(n.parent) + " c " + node_ref(n.child) + " " + hI((long long) n.parents.size()) + " " +
                    hI((long long) n.merged_neighbours.size());
    if (n.type == ePolygon) o += " o " + EXPOLY(n.overhang) + " " + hD(n.origin_area);
    return o;
}

static void dump_nodes(const std::string &tag, const std::vector<std::vector<SupportNode *>> &contact_nodes, bool hashed = false)
{
    for (size_t layer_nr = 0; layer_nr < contact_nodes.size(); ++layer_nr) {
        if (contact_nodes[layer_nr].empty()) continue;
        std::vector<std::string> lines;
        for (const SupportNode *n : contact_nodes[layer_nr]) lines.emplace_back(NODE(*n));
        std::sort(lines.begin(), lines.end());
        if (hashed && ! g_full) {
            std::string all;
            for (const std::string &l : lines) all += l + "\n";
            out(tag + " layer " + hI((long long) layer_nr) + " " + hI((long long) lines.size()) + " #" + fnv(all));
            continue;
        }
        out(tag + " layer " + hI((long long) layer_nr) + " " + hI((long long) lines.size()));
        for (const std::string &l : lines) out(tag + " node " + l);
    }
}

static std::string BBOX(const BoundingBox &b) { return hB(b.defined) + " " + PT(b.min) + " " + PT(b.max); }

// The area groups of a support layer; the area of a group is its position in the vector its type names.
static std::string AREA_GROUPS(const SupportLayer &l)
{
    std::string o = hI((long long) l.area_groups.size());
    for (const SupportLayer::AreaGroup &g : l.area_groups) {
        const ExPolygons &v = g.type == SupportLayer::BaseType ? l.base_areas :
                              g.type == SupportLayer::RoofType ? l.roof_areas : g.type == SupportLayer::FloorType ? l.floor_areas : l.roof_1st_layer;
        o += " " + hI(g.type) + " " + hI((long long) (g.area - v.data())) + " " + hI(g.interface_id) + " " + hD(g.dist_to_top) + " " + hB(g.need_infill) + " " +
             hB(g.need_extra_wall) + " " + hB(g.need_cooling) + " " + hD(g.line_width);
    }
    return o;
}

// The support layers of the object with the areas of the tree support. The long part of a line is hashed
// unless every line is written in full.
static void dump_support_layers(const std::string &tag, const PrintObject &obj)
{
    out(tag + " " + hI((long long) obj.support_layers().size()));
    size_t i = 0;
    for (const SupportLayer *sl : obj.support_layers()) {
        std::string body = "base " + EXPOLYS(sl->base_areas) + " roof " + EXPOLYS(sl->roof_areas) + " roof1 " + EXPOLYS(sl->roof_1st_layer) + " floor " +
                           EXPOLYS(sl->floor_areas) + " gap " + EXPOLYS(sl->roof_gap_areas) + " islands " + EXPOLYS(sl->support_islands) + " bboxes " +
                           hI((long long) sl->lslices_bboxes.size());
        for (const BoundingBox &b : sl->lslices_bboxes) body += " " + BBOX(b);
        body += " groups " + AREA_GROUPS(*sl);
        outb(tag + " layer " + hI((long long) i) + " " + hI((long long) sl->id()) + " " + hI((long long) sl->interface_id()) + " " + hD(sl->print_z) + " " +
                 hD(sl->height) + " " + hD(sl->slice_z) + " " + hI(sl->lower_layer ? (long long) sl->lower_layer->id() : -1) + " " +
                 hI(sl->upper_layer ? (long long) sl->upper_layer->id() : -1) + " " + hD(sl->base_slab_height) + " " + hD(sl->interface_slab_height),
             body, true);
        ++i;
    }
}

static indexed_triangle_set box_its(double sx, double sy, double sz, float ox, float oy, float oz)
{
    indexed_triangle_set its = its_make_cube(sx, sy, sz);
    for (Vec3f &v : its.vertices) v += Vec3f(ox, oy, oz);
    return its;
}

struct LayerIn
{
    double                              print_z, height;
    ExPolygons                          lslices;
    std::vector<std::pair<ExPolygon, int>> overhangs;
};

// The number of scenarios, and the first whose long lines are hashed.
static const int FIRST_HASHED     = 4;
// The first of the scenarios drawn from a random number generator, and their number.
static const int FIRST_RANDOM     = 15;
static const int NUM_RANDOM       = 30;
static const int NUM_SCENARIOS    = FIRST_RANDOM + NUM_RANDOM;

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
// An island that starts at the layer `from` and is `n` layers tall, its first layer an overhang of the type.
static void add_island(std::vector<LayerIn> &layers, size_t from, size_t n, const ExPolygon &island, int type)
{
    for (size_t i = from; i < from + n && i < layers.size(); ++i) layers[i].lslices.emplace_back(island);
    layers[from].overhangs.emplace_back(island, type);
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

struct RandomScenario
{
    std::vector<std::pair<std::string, std::string>> config;
    std::vector<LayerIn>                              layers;
    std::vector<std::pair<Vec3f, Vec3f>>              enforcer_points;
};

// A scenario drawn from the generator seeded with its number: a post in a corner, now and then a block on the
// bed, one to five islands that start in the air (rectangles, polygons, rings; of every overhang type), a
// support configuration.
static const RandomScenario &random_scenario(int scenario)
{
    static std::map<int, RandomScenario> cache;
    auto                                 it = cache.find(scenario);
    if (it != cache.end()) return it->second;
    RandomScenario &r = cache[scenario];
    Lcg             g{uint64_t(scenario) * 0x9E3779B97F4A7C15ULL + 12345};
    for (int i = 0; i < 4; ++i) g.next();
    auto num = [](double v) {
        char b[32];
        std::snprintf(b, sizeof(b), "%g", v);
        return std::string(b);
    };
    r.config.emplace_back("support_style", g.of<const char *>({"tree_slim", "tree_slim", "tree_strong", "tree_hybrid", "default"}));
    r.config.emplace_back("tree_support_branch_diameter", num(g.of({2., 3., 4., 5.})));
    r.config.emplace_back("tree_support_branch_distance", num(g.of({2., 3., 4., 5.})));
    r.config.emplace_back("tree_support_branch_diameter_angle", num(g.of({5., 8., 12., 15.})));
    r.config.emplace_back("support_interface_top_layers", num(g.of({0., 2., 3.})));
    r.config.emplace_back("tree_support_branch_angle", num(g.of({30., 40., 50.})));
    if (g.chance(30)) r.config.emplace_back("support_on_build_plate_only", "1");
    if (g.chance(30)) r.config.emplace_back("tree_support_wall_count", num(g.of({0., 1., 2.})));
    if (g.chance(15)) r.config.emplace_back("support_base_pattern", g.of<const char *>({"lightning", "rectilinear", "hollow"}));
    if (g.chance(10)) r.config.emplace_back("support_top_z_distance", "0");
    if (g.chance(10)) r.config.emplace_back("raft_layers", "2");
    if (g.chance(20)) r.config.emplace_back("independent_support_layer_height", "0");
    if (g.chance(10)) r.config.emplace_back("support_expansion", "1");
    if (g.chance(15)) r.config.emplace_back("raft_first_layer_expansion", "3");
    if (g.chance(15)) r.config.emplace_back("tree_support_tip_diameter", num(g.of({0.4, 1.2, 2.})));

    const double lh = 0.2;
    const int    nl = g.of({40, 55, 70});
    add_layers(r.layers, nl, lh, {ExPolygon(rect(-20, -14, -18, -12))});
    if (g.chance(40)) {
        const int    n  = g.of({4, 8, 12});
        const double x0 = g.uni(-10, 0), y0 = g.uni(-8, 0), x1 = g.uni(1, 10), y1 = g.uni(1, 8);
        for (int i = 0; i < n; ++i) r.layers[i].lslices.emplace_back(ExPolygon(rect(x0, y0, x1, y1)));
    }
    const int islands = 1 + g.pick(5);
    for (int i = 0; i < islands; ++i) {
        const int    from = 15 + g.pick(nl - 8 - 15);
        const int    type = g.of({0, 0, 0, int(TreeSupport::SharpTail), int(TreeSupport::BigFlat), int(TreeSupport::ThinPlate), int(TreeSupport::Cantilever),
                                  int(TreeSupport::Small)});
        const double cx = g.uni(-12, 12), cy = g.uni(-8, 8);
        switch (g.pick(4)) {
        case 0: {
            const double w = g.uni(1, 9), h = g.uni(1, 7);
            add_island(r.layers, from, 6, ExPolygon(rect(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)), type);
            break;
        }
        case 1: add_island(r.layers, from, 6, ExPolygon(ngon(cx, cy, g.uni(1, 6), g.of({3, 5, 8, 24, 120}))), type); break;
        default: {
            const double radius = g.uni(3, 9);
            const int    n      = g.of({24, 36, 150});
            add_island(r.layers, from, 6, with_hole(ngon(cx, cy, radius, n), ngon(cx, cy, std::round(radius * g.uni(0.4, 0.8) * 100.) / 100., g.of({12, 24}))), type);
            break;
        }
        }
    }
    if (g.chance(20))
        for (int i = 0; i < 3; ++i) {
            const double a = g.uni(0, 6.28);
            r.enforcer_points.emplace_back(Vec3f(float(scale_(g.uni(-15, 15))), float(scale_(g.uni(-10, 10))), float(g.uni(1, nl * lh))),
                                           Vec3f(float(std::cos(a)), float(std::sin(a)), 0.f));
        }
    return r;
}

// The layers of the scenarios from 4 on.
static std::vector<LayerIn> build_layers_more(int scenario)
{
    if (scenario >= FIRST_RANDOM) return random_scenario(scenario).layers;
    std::vector<LayerIn> layers;
    const double         lh = 0.2;
    switch (scenario) {
    case 4:
    case 5: {
        // A plate under the arms (4: under both, 5: under the left one), a stem, a wide top: the branches land on the object.
        add_layers(layers, 8, lh, {ExPolygon(scenario == 4 ? rect(-19, -12, 19, 12) : rect(-19, -12, 2, 12))});
        add_layers(layers, 22, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_layers(layers, 8, lh, {ExPolygon(rect(-18, -6, 18, 6))});
        layers[30].overhangs.emplace_back(ExPolygon(rect(-18, -6, -4.3, 6)), 0);
        layers[30].overhangs.emplace_back(ExPolygon(rect(4.3, -6, 18, 6)), 0);
        break;
    }
    case 6: {
        // A plate under the right arm, a top with a hole in the right arm.
        add_layers(layers, 8, lh, {ExPolygon(rect(-4, -4, 4, 4)), ExPolygon(rect(6, -12, 19, 12))});
        add_layers(layers, 22, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_layers(layers, 8, lh, {with_hole(rect(-18, -8, 18, 8), ngon(11, 0, 3, 16))});
        layers[30].overhangs.emplace_back(ExPolygon(rect(-18, -8, -4.3, 8)), 0);
        layers[30].overhangs.emplace_back(with_hole(rect(4.3, -8, 18, 8), ngon(11, 0, 3, 16)), 0);
        break;
    }
    case 7: {
        // Big flat overhangs: one of 353 mm2, two with a gap of 1.2 mm between them.
        add_layers(layers, 30, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_layers(layers, 6, lh, {ExPolygon(rect(-19, -12, 19, 12))});
        layers[30].overhangs.emplace_back(ExPolygon(rect(-19, -12, -4.3, 12)), int(TreeSupport::BigFlat));
        layers[30].overhangs.emplace_back(ExPolygon(rect(4.3, -12, 19, -0.6)), int(TreeSupport::BigFlat));
        layers[30].overhangs.emplace_back(ExPolygon(rect(4.3, 0.6, 19, 12)), int(TreeSupport::BigFlat));
        break;
    }
    case 8: {
        // A thin plate and a normal overhang side by side, a big flat one over a low block; the supports are expanded.
        add_layers(layers, 6, lh, {ExPolygon(rect(-4, -4, 4, 4)), ExPolygon(rect(8, -3, 14, 3))});
        add_layers(layers, 18, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_layers(layers, 6, lh, {ExPolygon(rect(-19, -12, 19, 12))});
        layers[24].overhangs.emplace_back(ExPolygon(rect(-19, -12, -4.3, -1)), int(TreeSupport::ThinPlate));
        layers[24].overhangs.emplace_back(ExPolygon(rect(-19, 1, -4.3, 12)), 0);
        layers[24].overhangs.emplace_back(ExPolygon(rect(4.3, -12, 19, 12)), int(TreeSupport::BigFlat));
        break;
    }
    case 9: {
        // Small islands that start in the air: sharp tails (one a ring), a small one, a tiny cantilever.
        add_layers(layers, 40, lh, {ExPolygon(rect(-3, -3, 3, 3))});
        add_island(layers, 12, 9, ExPolygon(rect(6, -0.6, 7.2, 0.6)), int(TreeSupport::SharpTail));
        add_island(layers, 18, 7, with_hole(ngon(-9, 0, 3, 24), ngon(-9, 0, 1.6, 12)), int(TreeSupport::SharpTail));
        add_island(layers, 26, 5, ExPolygon(rect(5, 5, 7.5, 6.2)), 0);
        add_island(layers, 32, 4, ExPolygon(rect(-8, -9, -7.2, -8.4)), int(TreeSupport::Cantilever));
        add_island(layers, 35, 4, ExPolygon(ngon(9, -8, 2.2, 5, 0.4)), int(TreeSupport::Small));
        break;
    }
    case 10: {
        // More than a hundred layers under a small overhang.
        add_layers(layers, 112, lh, {ExPolygon(rect(-2, -2, 2, 2))});
        add_layers(layers, 5, lh, {ExPolygon(rect(-2, -2, 7, 2))});
        layers[112].overhangs.emplace_back(ExPolygon(rect(2.3, -2, 7, 2)), 0);
        break;
    }
    case 12: {
        // The lightning pattern in the base of the supports.
        add_layers(layers, 30, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_layers(layers, 6, lh, {ExPolygon(rect(-19, -12, 19, 12))});
        layers[30].overhangs.emplace_back(ExPolygon(rect(-19, -12, -4.3, 12)), 0);
        layers[30].overhangs.emplace_back(ExPolygon(rect(4.3, -12, 19, 12)), int(TreeSupport::BigFlat));
        break;
    }
    case 13: {
        // More than 200 nodes in a layer (square supports): thirty small islands over a post in a corner.
        add_layers(layers, 26, lh, {ExPolygon(rect(-20, -14, -18.5, -12.5))});
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 5; ++j) {
                if (i == 0 && j == 0) continue;
                add_island(layers, 20, 6, ExPolygon(rect(-17 + 6 * i, -12 + 6 * j, -15 + 6 * i, -10 + 6 * j)), 0);
            }
        break;
    }
    case 14: {
        // A ring in the air whose branches close up below it, a small island under its middle: holes in the base areas.
        add_layers(layers, 46, lh, {ExPolygon(rect(-20, -14, -18, -12))});
        add_island(layers, 40, 6, with_hole(ngon(0, 0, 9, 36), ngon(0, 0, 6, 24)), 0);
        add_island(layers, 30, 4, ExPolygon(rect(-1, -1, 1, 1)), 0);
        break;
    }
    case 11: {
        // A raft, no top gap, enforcer points on a vertical wall.
        add_layers(layers, 30, lh, {ExPolygon(rect(-4, -4, 4, 4))});
        add_layers(layers, 6, lh, {ExPolygon(rect(-12, -4, 12, 4))});
        layers[30].overhangs.emplace_back(ExPolygon(rect(-12, -4, -4.3, 4)), 0);
        break;
    }
    }
    return layers;
}

// The points of the support enforcers on vertical walls, as PrintObject::project_and_append_custom_facets()
// hands them over: the position (x and y scaled, z in mm) and the normal.
static std::vector<std::pair<Vec3f, Vec3f>> scenario_enforcer_points(int scenario)
{
    if (scenario >= FIRST_RANDOM) return random_scenario(scenario).enforcer_points;
    if (scenario != 11) return {};
    return {{Vec3f(float(scale_(6.)), 0.f, 3.1f), Vec3f(1.f, 0.f, 0.f)},
            {Vec3f(float(scale_(7.5)), float(scale_(1.2)), 4.53f), Vec3f(0.6f, 0.8f, 0.f)},
            {Vec3f(float(scale_(7.25)), float(scale_(-1.7)), 4.47f), Vec3f(0.28f, -0.96f, 0.f)},
            {Vec3f(float(scale_(5.)), float(scale_(-2.)), 0.05f), Vec3f(1.f, 0.f, 0.f)},
            {Vec3f(float(scale_(5.)), float(scale_(2.)), 50.f), Vec3f(1.f, 0.f, 0.f)}};
}

// The layers of a scenario: a stem, then a wide top; the overhangs of the first layer of the top.
static std::vector<LayerIn> build_layers(int scenario)
{
    if (scenario >= FIRST_HASHED) return build_layers_more(scenario);
    std::vector<LayerIn> layers;
    const double lh       = scenario == 3 ? 0.16 : 0.2;
    const int    n_stem   = scenario == 2 ? 30 : 40;
    const int    n_top    = 12;
    double       z        = 0.;
    for (int i = 0; i < n_stem + n_top; ++i) {
        LayerIn l;
        l.height = lh;
        z += lh;
        l.print_z = z;
        if (i < n_stem) {
            l.lslices = {ExPolygon(rect(-5, -5, 5, 5))};
            if (scenario == 1 && i < 10) l.lslices.emplace_back(ExPolygon(rect(12, -4, 18, 4))); // a low block under the right arm
        } else if (scenario == 2)
            l.lslices = {with_hole(ngon(0, 0, 14, 48), ngon(0, 0, 3, 16))};
        else
            l.lslices = {ExPolygon(rect(-20, -5, 20, 5))};
        if (i == n_stem) {
            if (scenario == 2)
                l.overhangs.emplace_back(with_hole(ngon(0, 0, 14, 48), ngon(0, 0, 7.2, 24)), 0);
            else {
                l.overhangs.emplace_back(ExPolygon(rect(-20, -5, -5.3, 5)), scenario == 3 ? int(TreeSupport::SharpTail) : 0);
                l.overhangs.emplace_back(ExPolygon(rect(5.3, -5, 20, 5)), scenario == 1 ? int(TreeSupport::BigFlat) : 0);
            }
        }
        layers.emplace_back(std::move(l));
    }
    return layers;
}

static std::vector<std::pair<std::string, std::string>> scenario_config(int scenario)
{
    std::vector<std::pair<std::string, std::string>> c = {
        {"enable_support", "1"}, {"support_type", "tree(auto)"}, {"layer_height", scenario == 3 ? "0.16" : "0.2"}, {"initial_layer_print_height", scenario == 3 ? "0.16" : "0.2"}};
    if (scenario == 0) {
        c.emplace_back("support_style", "tree_slim");
    }
    if (scenario == 1) {
        c.emplace_back("support_style", "tree_hybrid");
        c.emplace_back("tree_support_branch_angle", "35");
        c.emplace_back("tree_support_branch_distance", "3.5");
        c.emplace_back("support_interface_top_layers", "3");
    }
    if (scenario == 2) {
        c.emplace_back("support_style", "tree_strong");
        c.emplace_back("tree_support_wall_count", "2");
        c.emplace_back("tree_support_branch_diameter", "4");
        c.emplace_back("tree_support_branch_diameter_angle", "8");
        c.emplace_back("support_interface_top_layers", "0");
        c.emplace_back("support_on_build_plate_only", "1");
    }
    if (scenario == 3) {
        c.emplace_back("support_style", "tree_slim");
        c.emplace_back("tree_support_tip_diameter", "1.2");
        c.emplace_back("support_top_z_distance", "0.32");
        c.emplace_back("raft_layers", "3");
    }
    if (scenario >= FIRST_RANDOM)
        for (const auto &kv : random_scenario(scenario).config) c.emplace_back(kv);
    if (scenario == 4) {
        c.emplace_back("support_style", "tree_slim");
    }
    if (scenario == 5) {
        c.emplace_back("support_on_build_plate_only", "1");
        c.emplace_back("tree_support_branch_distance", "4");
    }
    if (scenario == 6) {
        c.emplace_back("support_style", "tree_strong");
        c.emplace_back("tree_support_branch_angle", "50");
        c.emplace_back("tree_support_branch_diameter", "3");
        c.emplace_back("tree_support_wall_count", "0");
        c.emplace_back("support_interface_top_layers", "2");
    }
    if (scenario == 7) {
        c.emplace_back("support_style", "tree_hybrid");
        c.emplace_back("tree_support_wall_count", "2");
        c.emplace_back("support_interface_top_layers", "2");
    }
    if (scenario == 8) {
        c.emplace_back("support_style", "tree_hybrid");
        c.emplace_back("support_expansion", "1.5");
        c.emplace_back("support_threshold_angle", "45");
    }
    if (scenario == 9) {
        c.emplace_back("support_style", "tree_slim");
        c.emplace_back("independent_support_layer_height", "0");
        c.emplace_back("support_interface_top_layers", "2");
        c.emplace_back("tree_support_branch_distance", "2");
    }
    if (scenario == 10) {
        c.emplace_back("support_style", "tree_slim");
        c.emplace_back("tree_support_branch_diameter", "2");
    }
    if (scenario == 12) {
        c.emplace_back("support_style", "tree_strong");
        c.emplace_back("support_base_pattern", "lightning");
        c.emplace_back("support_interface_top_layers", "2");
    }
    if (scenario == 13) {
        c.emplace_back("support_style", "tree_slim");
        c.emplace_back("tree_support_branch_diameter", "1");
        c.emplace_back("tree_support_branch_distance", "1");
    }
    if (scenario == 14) {
        c.emplace_back("support_style", "tree_strong");
        c.emplace_back("tree_support_branch_diameter", "4");
        c.emplace_back("tree_support_branch_diameter_angle", "12");
        c.emplace_back("support_interface_top_layers", "2");
    }
    if (scenario == 11) {
        c.emplace_back("support_style", "tree_strong");
        c.emplace_back("raft_layers", "2");
        c.emplace_back("support_top_z_distance", "0");
        c.emplace_back("support_interface_top_layers", "1");
    }
    return c;
}

static void run_scenario(int scenario)
{
    const std::string S = "s" + hI(scenario);
    // The long lines of this scenario are hashed.
    const bool   H  = scenario >= FIRST_HASHED;
    const int    sz = scenario == 10 || scenario >= FIRST_RANDOM ? 24 : 12;
    Model        model;
    ModelObject *o = model.add_object();
    o->name = "tee";
    o->add_volume(TriangleMesh(box_its(40, 28, sz, -20.f, -14.f, 0.f)));
    o->add_instance()->set_offset(Vec3d(100., 100., 0.));
    out("in " + S + " model 40 28 " + hI(sz) + " 100 100");

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(3);
    config.option<ConfigOptionStrings>("filament_colour", true)->values = {"#FF0000", "#00FF00", "#0000FF"};
    config.option<ConfigOptionInts>("filament_map", true)->values = {1, 1, 1};
    for (const auto &kv : scenario_config(scenario)) {
        config.set_deserialize_strict(kv.first, kv.second);
        out("in " + S + " cfg " + kv.first + " " + kv.second);
    }
    Print print;
    print.apply(model, config);
    PrintObject *obj = print.m_objects.front();
    obj->update_slicing_parameters();

    std::vector<LayerIn> layers = build_layers(scenario);
    for (size_t i = 0; i < layers.size(); ++i) {
        const LayerIn &l     = layers[i];
        Layer         *layer = new Layer(i, obj, l.height, l.print_z, l.print_z - l.height / 2.);
        layer->lslices            = l.lslices;
        layer->lslices_extrudable = l.lslices;
        for (const auto &ov : l.overhangs) add_overhang(layer, ov.first, ov.second);
        if (i > 0) {
            layer->lower_layer             = obj->m_layers.back();
            obj->m_layers.back()->upper_layer = layer;
        }
        obj->m_layers.emplace_back(layer);
        std::string line = "in " + S + " layer " + hD(l.print_z) + " " + hD(l.height) + " " + hD(layer->slice_z) + " " + EXPOLYS(l.lslices) + " " +
                           hI((long long) l.overhangs.size());
        for (const auto &ov : l.overhangs) line += " " + hI(ov.second) + " " + EXPOLY(ov.first);
        out(line);
    }

    const SlicingParameters &sp = obj->slicing_parameters();
    out("in " + S + " slicing " + hD(sp.layer_height) + " " + hD(sp.min_layer_height) + " " + hD(sp.max_layer_height) + " " + hD(sp.max_suport_layer_height) + " " +
        hI((long long) sp.base_raft_layers) + " " + hI((long long) sp.interface_raft_layers) + " " + hD(sp.base_raft_layer_height) + " " +
        hD(sp.interface_raft_layer_height) + " " + hD(sp.first_print_layer_height) + " " + hD(sp.object_print_z_min) + " " + hB(sp.soluble_interface) + " " +
        hD(sp.gap_object_support));
    out("in " + S + " bed " + POLY(get_bed_shape_with_excluded_area(print.config())));

    TreeSupport ts(*obj, sp);
    const SupportParameters &p = ts.m_support_params;
    out("in " + S + " params " + hI((int) p.support_style) + " " + hI((int) p.support_base_pattern) + " " + hI((long long) p.num_top_interface_layers) + " " +
        hD(p.support_extrusion_width) + " " + hB(p.independent_layer_height) + " " + hB(p.soluble_interface) + " " + hD(p.thresh_big_overhang));

    // The constructor.
    out(S + " ctor " + hI((long long) ts.m_raft_layers) + " " + hI((int) ts.support_type) + " " + hD(ts.diameter_angle_scale_factor) + " " + hB(ts.is_slim) + " " +
        hB(ts.is_strong) + " " + hD(ts.tip_radius) + " " + hD(ts.base_radius) + " " + hB(ts.with_infill) + " " + hF(ts.top_z_distance) + " " + hF(ts.m_top_z_max) + " " +
        hI((long long) ts.m_top_z_outlines.size()) + " " + EXPOLY(ts.m_machine_border));

    // TreeSupportData: the constructor, ceil_radius, the contours, the collision and avoidance areas.
    TreeSupportData &d = *ts.m_ts_data;
    out(S + " data " + hD(d.m_xy_distance) + " " + hD(d.m_radius_sample_resolution) + " " + hD(d.branch_scale_factor) + " " + hI((long long) d.m_layer_outlines.size()));
    for (size_t l : {size_t(0), size_t(9), size_t(10), size_t(20), layers.size() - 12, layers.size() - 1})
        outb(S + " outline " + hI((long long) l) + " " + hD(d.m_max_move_distances[l]),
             EXPOLYS(d.m_layer_outlines[l]) + " " + EXPOLYS(d.m_layer_outlines_below[l]) + " " + POLYS(d.get_contours(l)) + " " + POLYS(d.get_contours_with_holes(l)), H);
    for (double r : {0., 0.1, 0.2, 0.25, 0.4, 0.45000000000000001, 1.3, 2.0000001, 5.55})
        out(S + " ceil_radius " + hD(r) + " " + hD(d.ceil_radius(r)));
    for (double r : {0., 0.4, 1.3})
        for (size_t l : {size_t(0), size_t(3), size_t(11), layers.size() - 13, layers.size() - 12, layers.size() - 2})
            outb(S + " collision " + hD(r) + " " + hI((long long) l), EXPOLYS(d.get_collision(r, l)), H);
    for (double r : {0.4, 2.})
        for (size_t l : {size_t(0), size_t(5), layers.size() - 13, layers.size() - 12})
            outb(S + " avoidance " + hD(r) + " " + hI((long long) l), EXPOLYS(d.get_avoidance(r, l)) + " " + EXPOLYS(ts.get_avoidance(r, l)), H);
    outb(S + " collision_polys", POLYS(ts.get_collision_polys(0.4, 3)) + " " + EXPOLYS(ts.get_collision(0.4, 3)), H);

    // The radii of a branch.
    for (double mm : {0., 0.1, 0.39, 0.8, 1.5, 2.5, 7.3, 30., 200.})
        out(S + " calc_radius " + hD(mm) + " " + hD(ts.calc_radius(mm)) + " " + hD(ts.calc_branch_radius(1.7, mm, 0.11, true)) + " " +
            hD(ts.calc_branch_radius(ts.base_radius, mm, ts.diameter_angle_scale_factor, false)));
    for (size_t n : {size_t(0), size_t(1), size_t(4), size_t(9), size_t(10), size_t(21), size_t(80), size_t(400)})
        out(S + " calc_branch_radius_layers " + hI((long long) n) + " " + hD(ts.calc_branch_radius(ts.base_radius, n, size_t(10), ts.diameter_angle_scale_factor)) + " " +
            hD(ts.calc_branch_radius(1.3, n, size_t(4), 0.2)));

    // The top gap.
    for (size_t l : {size_t(0), size_t(7), layers.size() - 12})
        out(S + " top_z " + hI((long long) l) + " " + hF(ts.top_z_at(l)) + " " + hF(ts.top_z_for_overhang(l, ExPolygon(rect(-20, -5, -5.3, 5)))));
    for (size_t l : {layers.size() - 14, layers.size() - 13, size_t(3)})
        for (bool clamp : {false, true}) {
            ExPolygons collision = ts.get_collision(0., l);
            ts.union_outlines_within_top_z(collision, l, clamp);
            outb(S + " union_outlines_within_top_z " + hI((long long) l) + " " + hB(clamp), EXPOLYS(collision), H);
        }
    out(S + " support_flare_step_width " + hD(ts.support_flare_step_width(3.2, 0.2)) + " " + hD(ts.support_flare_step_width(0.2, 0.2)));

    // The pipeline of TreeSupport::generate() after the overhang detection.
    ts.create_tree_support_layers();
    {
        std::string line = S + " raft_layers " + hI((long long) ts.m_raft_layers) + " " + hI((long long) obj->support_layers().size());
        for (const SupportLayer *sl : obj->support_layers()) line += " " + hI((long long) sl->id()) + " " + hD(sl->print_z) + " " + hD(sl->height) + " " + hD(sl->slice_z);
        out(line);
    }
    ts.m_ts_data->is_slim = ts.is_slim;
    {
        // The enforcer points detect_overhangs() collects.
        ts.m_vertical_enforcer_points = scenario_enforcer_points(scenario);
        if (H) {
            std::string line = "in " + S + " enforcer_points " + hI((long long) ts.m_vertical_enforcer_points.size());
            for (const auto &pn : ts.m_vertical_enforcer_points)
                for (const Vec3f &v : {pn.first, pn.second}) line += " " + hF(v.x()) + " " + hF(v.y()) + " " + hF(v.z());
            out(line);
        }
    }
    ts.generate_contact_points();
    out(S + " contact_points " + hI((long long) ts.contact_nodes.size()) + " " + hI(ts.avg_node_per_layer) + " " + hI((long long) d.contact_nodes.size()) +
        (H ? " " + hF(ts.nodes_angle) : std::string()));
    dump_nodes(S + " contact", ts.contact_nodes, H);

    d.layer_heights = ts.plan_layer_heights();
    {
        std::string line = S + " layer_heights " + hI((long long) d.layer_heights.size());
        for (const LayerHeightData &lh : d.layer_heights) line += " " + hD(lh.print_z) + " " + hD(lh.height) + " " + hI((long long) lh.obj_layer_nr);
        out(line);
        line = S + " support_layers " + hI((long long) obj->support_layers().size());
        for (const SupportLayer *sl : obj->support_layers())
            line += " " + hI((long long) sl->id()) + " " + hD(sl->print_z) + " " + hD(sl->height) + " " + hD(sl->slice_z) + " " +
                    hI(sl->lower_layer ? (long long) sl->lower_layer->id() : -1) + " " + hI(sl->upper_layer ? (long long) sl->upper_layer->id() : -1);
        out(line);
    }
    dump_nodes(S + " planned", ts.contact_nodes, H);

    ts.drop_nodes();
    out(S + " dropped " + hI((long long) d.contact_nodes.size()) + " " + hD(SupportNode::diameter_angle_scale_factor));
    dump_nodes(S + " drop", ts.contact_nodes, H);

    ts.smooth_nodes();
    dump_nodes(S + " smooth", ts.contact_nodes, H);

    // insert_dropped_node: a node already in the layer, and one that is not.
    for (size_t layer_nr = 0; layer_nr + 1 < ts.contact_nodes.size(); ++layer_nr)
        if (! ts.contact_nodes[layer_nr].empty() && ! ts.contact_nodes[layer_nr + 1].empty()) {
            std::vector<SupportNode *> nodes_layer = ts.contact_nodes[layer_nr];
            ts.insert_dropped_node(nodes_layer, nodes_layer.front());
            const size_t before = nodes_layer.size();
            ts.insert_dropped_node(nodes_layer, ts.contact_nodes[layer_nr + 1].front());
            out(S + " insert_dropped_node " + hI((long long) layer_nr) + " " + hI((long long) before) + " " + hI((long long) nodes_layer.size()) + " " +
                node_ref(nodes_layer.back()));
            break;
        }

    // tree_support_node_disk over the nodes of a few layers.
    {
        const coordf_t branch_radius        = obj->config().tree_support_branch_diameter.value / 2;
        const coordf_t branch_radius_scaled = scale_(branch_radius);
        const Polygon  unit_circle          = make_circle(branch_radius_scaled, SUPPORT_TREE_CIRCLE_RESOLUTION);
        out("in " + S + " disk " + hD(branch_radius) + " " + hD(branch_radius_scaled) + " " + POLY(unit_circle));
        size_t done = 0;
        for (size_t layer_nr = 1; layer_nr < ts.contact_nodes.size() && done < 4; layer_nr += 7) {
            if (ts.contact_nodes[layer_nr].empty()) continue;
            std::vector<std::string> lines;
            for (const SupportNode *n : ts.contact_nodes[layer_nr])
                lines.emplace_back(NODE(*n) + " d " + EXPOLY(tree_support_node_disk(*n, unit_circle, branch_radius, branch_radius_scaled, false)) + " " +
                                   EXPOLY(tree_support_node_disk(*n, unit_circle, branch_radius, branch_radius_scaled, true, 0.9)));
            std::sort(lines.begin(), lines.end());
            if (H && ! g_full) {
                std::string all;
                for (const std::string &l : lines) all += l + "\n";
                out(S + " disk " + hI((long long) layer_nr) + " " + hI((long long) lines.size()) + " #" + fnv(all));
            } else
                for (const std::string &l : lines) out(S + " disk " + hI((long long) layer_nr) + " " + l);
            ++done;
        }
    }

    // The support areas: the steps of TreeSupport::generate() after smooth_nodes().
    ts.fold_nodes_multi_diameter();
    ts.smooth_line_width_multi_diameter();
    ts.draw_circles();
    dump_support_layers(S + " draw_circles", *obj);
    {
        // What draw_circles() left for the lightning infill of the support: the layer of every print_z.
        std::vector<std::pair<double, size_t>> zs(ts.printZ_to_lightninglayer.begin(), ts.printZ_to_lightninglayer.end());
        std::sort(zs.begin(), zs.end());
        std::string line = S + " lightning " + hI((long long) zs.size());
        for (const auto &z : zs) line += " " + hD(z.first) + " " + hI((long long) z.second);
        out(line);
        if (ts.generator != nullptr) {
            // The generator of the lightning infill: the radius it derived from the density, the overhangs it was handed.
            out("in " + S + " lightning_width " + hF(ts.generator->m_infill_extrusion_width));
            std::string body = hI((long long) ts.generator->m_supporting_radius) + " " + hI((long long) ts.generator->m_overhang_per_layer.size());
            for (const Polygons &polys : ts.generator->m_overhang_per_layer) body += " " + POLYS(polys);
            outb(S + " lightning_generator", body, true);
        }
    }
}

// The helpers that take no TreeSupport.
static void run_helpers()
{
    std::vector<Polygon> polys = {
        rect(0, 0, 30, 20), rect(-3, -2, 4, 9), ngon(1.5, -2.25, 12, 7), ngon(100, 50, 25.5, 40, 0.3), ngon(0, 0, 3, 5, 1.1),
        ngon(-40.123, 12.5, 60, 96, 0.01), ngon(5, 5, 0.8, 3)};
    {
        Polygon star;
        for (int i = 0; i < 14; ++i) {
            double a = 2. * M_PI * i / 14, r = (i % 2) ? 9.37 : 21.9;
            star.points.emplace_back(Point(scale_(3.3 + r * std::cos(a)), scale_(-7.1 + r * std::sin(a))));
        }
        polys.emplace_back(star);
    }
    for (size_t i = 0; i < polys.size(); ++i) {
        const Polygon &p = polys[i];
        out("in poly " + POLY(p));
        out("centroid " + hI((long long) i) + " " + PT(p.centroid()));
        for (double max_move : {0.05, 0.3, 2.}) {
            std::unordered_map<Point, Point, PointHash> movements;
            smooth_filter(p, 2, movements, max_move);
            std::string line = "smooth_filter " + hI((long long) i) + " " + hD(max_move);
            for (const Point &pt : p.points) line += " " + PT(movements.at(pt));
            out(line);
        }
        for (double max_edge : {2., 0.7, 11.3}) {
            Polygon q = p;
            densify_polygon(q, max_edge);
            out("densify_polygon " + hI((long long) i) + " " + hD(max_edge) + " " + POLY(q));
        }
    }
    // libnest2d's EdgeCache, as generate_contact_points() walks it.
    std::vector<ExPolygon> expolys = {ExPolygon(rect(0, 0, 30, 20)), with_hole(ngon(0, 0, 14, 48), ngon(0, 0, 7.2, 24)), with_hole(rect(-10, -8, 22, 9), rect(-4, -3, 5, 2)),
                                      ExPolygon(ngon(3, 4, 6.5, 9, 0.7))};
    for (size_t i = 0; i < expolys.size(); ++i) {
        out("in expoly " + EXPOLY(expolys[i]));
        libnest2d::placers::EdgeCache<ExPolygon> edge_cache(expolys[i]);
        std::string line = "edge_cache " + hI((long long) i) + " " + hI((long long) edge_cache.holeCount()) + " " + hD(edge_cache.circumference());
        for (size_t h = 0; h < edge_cache.holeCount(); ++h) line += " " + hD(edge_cache.circumference(h));
        for (size_t h = 0; h < edge_cache.holeCount() + 1; ++h) {
            double step = scale_(3.) / (h == 0 ? edge_cache.circumference() : edge_cache.circumference(h - 1));
            for (double distance = 0; distance < 1; distance += step) line += " " + PT(h == 0 ? edge_cache.coords(distance) : edge_cache.coords(h - 1, distance));
        }
        out(line);
    }
    // carried_area / carried_area_ratio
    std::vector<ExPolygons> carriers = {
        {ExPolygon(rect(0, 0, 10, 10)), ExPolygon(rect(20, 0, 30, 10)), with_hole(rect(-30, -30, -5, -5), rect(-20, -20, -10, -10))},
        {with_hole(ngon(0, 0, 14, 48), ngon(0, 0, 7.2, 24))},
        {}};
    std::vector<ExPolygon> regions = {ExPolygon(rect(5, 5, 25, 8)), ExPolygon(rect(-25, -25, -8, -8)), ExPolygon(ngon(0, 0, 9, 32)), ExPolygon(rect(100, 100, 110, 110)),
                                      with_hole(rect(-12, -12, 12, 12), rect(-2, -2, 2, 2)), ExPolygon(rect(8, 2, 22, 3))};
    for (size_t c = 0; c < carriers.size(); ++c) {
        out("in carrier " + EXPOLYS(carriers[c]));
        const std::vector<BoundingBox> bboxes = get_extents_vector(carriers[c]);
        for (size_t r = 0; r < regions.size(); ++r) {
            if (c == 0) out("in region " + EXPOLY(regions[r]));
            out("carried_area " + hI((long long) c) + " " + hI((long long) r) + " " + hD(carried_area(carriers[c], bboxes, regions[r])) + " " +
                hD(carried_area_ratio(carriers[c], bboxes, regions[r])));
        }
    }
}

int main(int argc, char **argv)
{
    g_out  = std::fopen(argc > 1 ? argv[1] : "vectors.txt", "w");
    g_full = argc > 2 && std::string(argv[2]) == "full";
    // Optionally one scenario only (a diagnostic); any number from FIRST_RANDOM on is a random scenario.
    const int only = argc > 3 ? std::atoi(argv[3]) : -1;
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism, 1);
    out(std::string("in mode ") + (g_full ? "full" : "hash"));
    if (only < 0) run_helpers();
    if (only >= 0)
        run_scenario(only);
    else
        for (int scenario = 0; scenario < NUM_SCENARIOS; ++scenario) run_scenario(scenario);
    std::fclose(g_out);
    return 0;
}
