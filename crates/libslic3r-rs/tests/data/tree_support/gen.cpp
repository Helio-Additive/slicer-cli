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

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
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

static void dump_nodes(const std::string &tag, const std::vector<std::vector<SupportNode *>> &contact_nodes)
{
    for (size_t layer_nr = 0; layer_nr < contact_nodes.size(); ++layer_nr) {
        if (contact_nodes[layer_nr].empty()) continue;
        std::vector<std::string> lines;
        for (const SupportNode *n : contact_nodes[layer_nr]) lines.emplace_back(NODE(*n));
        std::sort(lines.begin(), lines.end());
        out(tag + " layer " + hI((long long) layer_nr) + " " + hI((long long) lines.size()));
        for (const std::string &l : lines) out(tag + " node " + l);
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

// The layers of a scenario: a stem, then a wide top; the overhangs of the first layer of the top.
static std::vector<LayerIn> build_layers(int scenario)
{
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
    return c;
}

static void run_scenario(int scenario)
{
    const std::string S = "s" + hI(scenario);
    Model        model;
    ModelObject *o = model.add_object();
    o->name = "tee";
    o->add_volume(TriangleMesh(box_its(40, 28, 12, -20.f, -14.f, 0.f)));
    o->add_instance()->set_offset(Vec3d(100., 100., 0.));
    out("in " + S + " model 40 28 12 100 100");

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
        out(S + " outline " + hI((long long) l) + " " + hD(d.m_max_move_distances[l]) + " " + EXPOLYS(d.m_layer_outlines[l]) + " " + EXPOLYS(d.m_layer_outlines_below[l]) +
            " " + POLYS(d.get_contours(l)) + " " + POLYS(d.get_contours_with_holes(l)));
    for (double r : {0., 0.1, 0.2, 0.25, 0.4, 0.45000000000000001, 1.3, 2.0000001, 5.55})
        out(S + " ceil_radius " + hD(r) + " " + hD(d.ceil_radius(r)));
    for (double r : {0., 0.4, 1.3})
        for (size_t l : {size_t(0), size_t(3), size_t(11), layers.size() - 13, layers.size() - 12, layers.size() - 2})
            out(S + " collision " + hD(r) + " " + hI((long long) l) + " " + EXPOLYS(d.get_collision(r, l)));
    for (double r : {0.4, 2.})
        for (size_t l : {size_t(0), size_t(5), layers.size() - 13, layers.size() - 12})
            out(S + " avoidance " + hD(r) + " " + hI((long long) l) + " " + EXPOLYS(d.get_avoidance(r, l)) + " " + EXPOLYS(ts.get_avoidance(r, l)));
    out(S + " collision_polys " + POLYS(ts.get_collision_polys(0.4, 3)) + " " + EXPOLYS(ts.get_collision(0.4, 3)));

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
            out(S + " union_outlines_within_top_z " + hI((long long) l) + " " + hB(clamp) + " " + EXPOLYS(collision));
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
    ts.generate_contact_points();
    out(S + " contact_points " + hI((long long) ts.contact_nodes.size()) + " " + hI(ts.avg_node_per_layer) + " " + hI((long long) d.contact_nodes.size()));
    dump_nodes(S + " contact", ts.contact_nodes);

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
    dump_nodes(S + " planned", ts.contact_nodes);

    ts.drop_nodes();
    out(S + " dropped " + hI((long long) d.contact_nodes.size()) + " " + hD(SupportNode::diameter_angle_scale_factor));
    dump_nodes(S + " drop", ts.contact_nodes);

    ts.smooth_nodes();
    dump_nodes(S + " smooth", ts.contact_nodes);

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
            for (const std::string &l : lines) out(S + " disk " + hI((long long) layer_nr) + " " + l);
            ++done;
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
    g_out = std::fopen(argc > 1 ? argv[1] : "vectors.txt", "w");
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism, 1);
    run_helpers();
    for (int scenario = 0; scenario < 4; ++scenario) run_scenario(scenario);
    std::fclose(g_out);
    return 0;
}
