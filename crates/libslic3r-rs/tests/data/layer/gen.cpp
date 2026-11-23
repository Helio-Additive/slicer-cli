// Generator of the vectors of tests/layer_native_parity.rs: libslic3r's Layer.cpp at the pin
// (references/BambuStudio/src/libslic3r; the override layer has no copy of it).
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
// Layer.cpp is compiled into this file (ContinuitiousDistancer is local to it), with the protected
// constructor of Layer made public.
//
// Every line is "<what> <inputs> => <results>": the test reads the inputs back, calls the port and
// has to print the same results. Tokens are separated by one space; a float is its 8 hexadecimal
// digits, an integer decimal.
// The vectors were made on macOS arm64 with Apple clang and Eigen 5.0.1.

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <thread>
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
#include "libslic3r/Layer.cpp"
#undef private
#undef protected

using namespace Slic3r;

static FILE *g_out = nullptr;

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

// A wall of a layer: a ring of eight corners with points in between (closed: the first point is
// repeated at the end) or an open run of points.
static Points wall(Lcg &g, coord_t cx, coord_t cy, coord_t rx, coord_t ry, bool closed)
{
    Points corners;
    if (closed) {
        const coord_t cut_x = g.range(rx / 8, rx / 3), cut_y = g.range(ry / 8, ry / 3);
        corners = {Point(cx - rx + cut_x, cy - ry), Point(cx + rx - cut_x, cy - ry), Point(cx + rx, cy - ry + cut_y), Point(cx + rx, cy + ry - cut_y),
                   Point(cx + rx - cut_x, cy + ry), Point(cx - rx + cut_x, cy + ry), Point(cx - rx, cy + ry - cut_y), Point(cx - rx, cy - ry + cut_y)};
        corners.push_back(corners.front());
    } else {
        Point cur(cx - rx, cy - ry);
        const int n = (int) g.range(2, 6);
        for (int i = 0; i < n; ++i) {
            corners.push_back(cur);
            cur = Point(cur.x() + g.range(0, 2 * rx / n + 1), cur.y() + g.range(-ry / 2, ry / 2));
        }
    }
    // Points in between, so that runs of short segments add up.
    Points pts;
    for (size_t i = 0; i + 1 < corners.size(); ++i) {
        pts.push_back(corners[i]);
        const int k = (int) g.range(0, 3);
        for (int j = 1; j <= k; ++j)
            pts.emplace_back(corners[i].x() + (corners[i + 1].x() - corners[i].x()) * j / (k + 1), corners[i].y() + (corners[i + 1].y() - corners[i].y()) * j / (k + 1));
    }
    pts.push_back(corners.back());
    return pts;
}

// ---------------------------------------------------------------------------------------------
// ContinuitiousDistancer
// ---------------------------------------------------------------------------------------------

static void run_distancer()
{
    Lcg g(0x9e3779b97f4a7c15ULL);
    for (int c = 0; c < 300; ++c) {
        Points pts;
        const long long kind = g.range(0, 9);
        if (kind == 0) {
            // No line at all: no point or one.
            if (g.range(0, 1) == 0) pts.emplace_back(g.range(-500000, 500000), g.range(-500000, 500000));
        } else {
            pts = wall(g, g.range(-3000000, 3000000), g.range(-3000000, 3000000), g.range(20000, 900000), g.range(20000, 900000), kind < 6);
            // A point twice in a row: a line of no length.
            if (kind == 9) pts.insert(pts.begin() + 1, pts.front());
        }
        ContinuitiousDistancer distancer(pts);
        std::string            in = hI((long long) pts.size());
        for (const Point &p : pts) in += " " + PT(p);
        const int   nq = 12;
        std::string res;
        in += " " + hI(nq);
        for (int q = 0; q < nq; ++q) {
            Point base(g.range(-4000000, 4000000), g.range(-4000000, 4000000));
            if (!pts.empty() && g.range(0, 3) != 0) {
                // Near the wall, on it now and then.
                const Point &at = pts[g.range(0, (long long) pts.size() - 1)];
                const coord_t d  = g.range(0, 3) == 0 ? 0 : g.range(1, 80000);
                base             = Point(at.x() + g.range(-d, d), at.y() + g.range(-d, d));
            }
            const Vec2f point = base.cast<float>();
            in += " " + hF(point.x()) + " " + hF(point.y());
            res += (q == 0 ? "" : " ") + hF(distancer.distance_from_perimeter(point));
        }
        out("dist " + in + " => " + res);
    }
}

// ---------------------------------------------------------------------------------------------
// Layer::calculate_perimeter_continuity
// ---------------------------------------------------------------------------------------------

static std::string NODE(const LoopNode &n)
{
    std::string o = hI(n.node_id) + " " + PT(n.bbox.min) + " " + PT(n.bbox.max) + " " + hB(n.node_contour.is_loop) + " " + hI((long long) n.node_contour.pts.size());
    for (const Point &p : n.node_contour.pts) o += " " + PT(p);
    o += " " + hI((long long) n.node_contour.widths.size());
    for (coord_t w : n.node_contour.widths) o += " " + hI(w);
    return o;
}

// A node of a wall. The box is that of the points grown by a random margin, as the walls' boxes are
// grown by half a line width.
static LoopNode make_node(Lcg &g, int id, const Points &pts, bool is_loop)
{
    LoopNode node;
    node.node_id              = id;
    node.loop_id              = id;
    node.node_contour.pts     = pts;
    node.node_contour.is_loop = is_loop;
    coord_t w                 = g.range(30000, 60000);
    // The widths: one per point, or fewer (the first one stands in for the missing ones).
    const size_t nw = g.range(0, 5) == 0 ? (size_t) g.range(1, (long long) std::max<size_t>(1, pts.size())) : std::max<size_t>(1, pts.size());
    for (size_t i = 0; i < nw; ++i) {
        if (g.range(0, 3) == 0) w = g.range(30000, 60000);
        node.node_contour.widths.push_back(w);
    }
    BoundingBox bbox;
    for (const Point &p : pts) bbox.merge(p);
    if (pts.empty()) bbox = BoundingBox(Point(0, 0), Point(0, 0));
    const coord_t margin = g.range(0, 30000);
    node.bbox            = BoundingBox(Point(bbox.min.x() - margin, bbox.min.y() - margin), Point(bbox.max.x() + margin, bbox.max.y() + margin));
    return node;
}

static void run_continuity()
{
    Lcg g(0x2545f4914f6cdd1dULL);
    for (int c = 0; c < 300; ++c) {
        Layer layer(1, nullptr, 0.2, 0.4, 0.3);
        std::vector<LoopNode> prev_nodes;
        const int             n_walls = (int) g.range(1, 4);
        int                   prev_id = 0;
        for (int i = 0; i < n_walls; ++i) {
            const coord_t cx = g.range(-1500000, 1500000), cy = g.range(-1500000, 1500000);
            const coord_t rx = g.range(20000, 700000), ry = g.range(20000, 700000);
            const bool    closed = g.range(0, 2) != 0;
            const Points  pts    = wall(g, cx, cy, rx, ry, closed);
            layer.loop_nodes.emplace_back(make_node(g, i, pts, closed));
            // The walls of the layer below: this wall moved by less than a line width, by about one,
            // or by more; cut short now and then; or another wall altogether.
            const int n_prev = (int) g.range(0, 2);
            for (int j = 0; j < n_prev; ++j) {
                const long long kind = g.range(0, 5);
                Points          below;
                bool            below_closed = closed;
                if (kind == 5) {
                    below_closed = g.range(0, 1) == 0;
                    below        = wall(g, cx + g.range(-rx, rx), cy + g.range(-ry, ry), g.range(20000, 700000), g.range(20000, 700000), below_closed);
                } else {
                    const coord_t reach = kind < 2 ? 20000 : (kind < 4 ? 50000 : 150000);
                    const coord_t dx = g.range(-reach, reach), dy = g.range(-reach, reach);
                    for (const Point &p : pts) {
                        const coord_t jitter = g.range(0, 4) == 0 ? g.range(-40000, 40000) : 0;
                        below.emplace_back(p.x() + dx + jitter, p.y() + dy + jitter);
                    }
                    if (g.range(0, 4) == 0 && below.size() > 3) {
                        below.resize(below.size() - (size_t) g.range(1, 2));
                        below_closed = g.range(0, 1) == 0;
                    }
                }
                prev_nodes.emplace_back(make_node(g, prev_id++, below, below_closed));
            }
        }
        // A wall below with one point, and one with none that is not a loop.
        if (c % 23 == 5) prev_nodes.emplace_back(make_node(g, prev_id++, Points{layer.loop_nodes.front().node_contour.pts.front()}, c % 2 == 0));
        if (c % 29 == 7) {
            LoopNode empty = make_node(g, prev_id++, Points(), false);
            empty.bbox     = layer.loop_nodes.front().bbox;
            prev_nodes.emplace_back(empty);
        }

        std::string in = hI((long long) layer.loop_nodes.size());
        for (const LoopNode &n : layer.loop_nodes) in += " " + NODE(n);
        in += " " + hI((long long) prev_nodes.size());
        for (const LoopNode &n : prev_nodes) in += " " + NODE(n);

        layer.calculate_perimeter_continuity(prev_nodes);

        std::string res;
        for (const LoopNode &n : layer.loop_nodes) {
            res += (res.empty() ? "" : " ") + hI((long long) n.lower_node_id.size());
            for (int id : n.lower_node_id) res += " " + hI(id);
        }
        for (const LoopNode &n : prev_nodes) {
            res += " " + hI((long long) n.upper_node_id.size());
            for (int id : n.upper_node_id) res += " " + hI(id);
        }
        out("continuity " + in + " => " + res);
    }
}

// ---------------------------------------------------------------------------------------------
// Layer::make_slices, get_extents, backup_untyped_slices / restore_untyped_slices*
// ---------------------------------------------------------------------------------------------
static std::string EX(const ExPolygon &ex)
{
    std::string s = hI((long long) ex.contour.points.size());
    for (const Point &p : ex.contour.points) s += " " + PT(p);
    s += " " + hI((long long) ex.holes.size());
    for (const Polygon &h : ex.holes) {
        s += " " + hI((long long) h.points.size());
        for (const Point &p : h.points) s += " " + PT(p);
    }
    return s;
}

static std::string BB(const BoundingBox &bb) { return hB(bb.defined) + " " + PT(bb.min) + " " + PT(bb.max); }

static std::string SLICES(const Layer &layer)
{
    std::string s;
    for (const LayerRegion *layerm : layer.m_regions) {
        s += " " + hI((long long) layerm->slices.surfaces.size());
        for (const Surface &surface : layerm->slices.surfaces) s += " " + hB(surface.surface_type == stInternal) + " " + EX(surface.expolygon);
    }
    return s;
}

// The regions of a layer on a grid of cells of 4 mm: whole cells (neighbours of different regions
// share an edge, as the parts of a modifier do), cells with a margin, octagons, some with a hole.
static void run_slices()
{
    Lcg g(0x51CE5ULL);
    const coord_t cell = 400000;
    for (int c = 0; c < 400; ++c) {
        Layer layer(1, nullptr, 0.2, 0.4, 0.3);
        const int nreg = c % 4 == 0 ? 1 : (int) g.range(2, 4);
        std::string in = hI(nreg);
        for (int r = 0; r < nreg; ++r) {
            LayerRegion *layerm = layer.add_region(nullptr);
            const int nsurf = (int) g.range(r == 0 ? 1 : 0, 4);
            in += " " + hI(nsurf);
            for (int k = 0; k < nsurf; ++k) {
                const coord_t x0 = g.range(0, 5) * cell, y0 = g.range(0, 5) * cell;
                const int kind = (int) g.range(0, 3);
                ExPolygon ex;
                if (kind == 0) {
                    ex.contour.points = {Point(x0, y0), Point(x0 + cell, y0), Point(x0 + cell, y0 + cell), Point(x0, y0 + cell)};
                } else if (kind == 1) {
                    const coord_t mx = g.range(1, cell / 3), my = g.range(1, cell / 3);
                    ex.contour.points = {Point(x0 + mx, y0 + my), Point(x0 + cell - mx, y0 + my), Point(x0 + cell - mx, y0 + cell - my), Point(x0 + mx, y0 + cell - my)};
                } else {
                    Points pts = wall(g, x0 + cell / 2, y0 + cell / 2, g.range(cell / 4, cell / 2 + cell / 8), g.range(cell / 4, cell / 2 + cell / 8), true);
                    pts.pop_back();
                    ex.contour.points = pts;
                }
                if (g.range(0, 3) == 0) {
                    const coord_t hx = x0 + cell / 2, hy = y0 + cell / 2, hr = g.range(cell / 40, cell / 10);
                    Polygon hole;
                    hole.points = {Point(hx - hr, hy - hr), Point(hx - hr, hy + hr), Point(hx + hr, hy + hr), Point(hx + hr, hy - hr)};
                    ex.holes.push_back(hole);
                }
                in += " " + EX(ex);
                layerm->slices.surfaces.emplace_back(stInternal, ex);
            }
        }
        // Layer.cpp:47-75
        layer.make_slices();
        std::string res = hI((long long) layer.lslices.size());
        for (const ExPolygon &ex : layer.lslices) res += " " + EX(ex);
        // Layer.cpp:750-770
        for (const LayerRegion *layerm : layer.m_regions) res += " " + BB(get_extents(*layerm));
        res += " " + BB(get_extents(layer.m_regions));
        // Layer.cpp:84-148: the slices get a type and lose a surface between the backup and the restore.
        layer.backup_untyped_slices();
        for (LayerRegion *layerm : layer.m_regions) {
            for (Surface &surface : layerm->slices.surfaces) surface.surface_type = stTop;
            if (layerm->slices.surfaces.size() > 1) layerm->slices.surfaces.pop_back();
        }
        layer.restore_untyped_slices();
        res += SLICES(layer);
        for (LayerRegion *layerm : layer.m_regions) {
            for (Surface &surface : layerm->slices.surfaces) surface.surface_type = stBottom;
            if (!layerm->slices.surfaces.empty()) layerm->slices.surfaces.erase(layerm->slices.surfaces.begin());
        }
        layer.restore_untyped_slices_no_extra_perimeters();
        res += SLICES(layer);
        out("slices " + in + " => " + res);
    }
}

// ---------------------------------------------------------------------------------------------
// Layer::merged
// ---------------------------------------------------------------------------------------------
// The regions of run_slices with a region config each: a region without shells, infill and walls is
// left out of the union (Layer.cpp:163).
static void run_merged()
{
    Lcg g(0x3E26EDULL);
    const coord_t cell = 400000;
    const float offsets[4] = {0.f, 1000.f, 20000.f, 45000.f};
    for (int c = 0; c < 200; ++c) {
        Layer layer(1, nullptr, 0.2, 0.4, 0.3);
        std::vector<std::unique_ptr<PrintRegion>> print_regions;
        const float offset_scaled = offsets[c % 4];
        const int nreg = (int) g.range(1, 4);
        std::string in = hF(offset_scaled) + " " + hI(nreg);
        for (int r = 0; r < nreg; ++r) {
            PrintRegionConfig cfg;
            const bool empty = g.range(0, 4) == 0;
            cfg.bottom_shell_layers.value   = empty ? 0 : (int) g.range(0, 3);
            cfg.top_shell_layers.value      = empty ? 0 : (int) g.range(0, 3);
            cfg.sparse_infill_density.value = empty ? 0. : double(g.range(0, 1) * 15);
            cfg.wall_loops.value            = empty ? 0 : (int) g.range(0, 2);
            in += " " + hI(cfg.bottom_shell_layers.value) + " " + hI(cfg.top_shell_layers.value) + " " + hI((long long) cfg.sparse_infill_density.value) + " " + hI(cfg.wall_loops.value);
            print_regions.emplace_back(new PrintRegion(cfg));
            LayerRegion *layerm = layer.add_region(print_regions.back().get());
            const int nsurf = (int) g.range(r == 0 ? 1 : 0, 4);
            in += " " + hI(nsurf);
            for (int k = 0; k < nsurf; ++k) {
                const coord_t x0 = g.range(0, 5) * cell, y0 = g.range(0, 5) * cell;
                const int kind = (int) g.range(0, 3);
                ExPolygon ex;
                if (kind == 0) {
                    ex.contour.points = {Point(x0, y0), Point(x0 + cell, y0), Point(x0 + cell, y0 + cell), Point(x0, y0 + cell)};
                } else if (kind == 1) {
                    const coord_t mx = g.range(1, cell / 3), my = g.range(1, cell / 3);
                    ex.contour.points = {Point(x0 + mx, y0 + my), Point(x0 + cell - mx, y0 + my), Point(x0 + cell - mx, y0 + cell - my), Point(x0 + mx, y0 + cell - my)};
                } else {
                    Points pts = wall(g, x0 + cell / 2, y0 + cell / 2, g.range(cell / 4, cell / 2 + cell / 8), g.range(cell / 4, cell / 2 + cell / 8), true);
                    pts.pop_back();
                    ex.contour.points = pts;
                }
                if (g.range(0, 3) == 0) {
                    const coord_t hx = x0 + cell / 2, hy = y0 + cell / 2, hr = g.range(cell / 40, cell / 10);
                    Polygon hole;
                    hole.points = {Point(hx - hr, hy - hr), Point(hx - hr, hy + hr), Point(hx + hr, hy + hr), Point(hx + hr, hy - hr)};
                    ex.holes.push_back(hole);
                }
                in += " " + EX(ex);
                layerm->slices.surfaces.emplace_back(stInternal, ex);
            }
        }
        // Layer.cpp:150-170
        const ExPolygons merged = layer.merged(offset_scaled);
        std::string res = hI((long long) merged.size());
        for (const ExPolygon &ex : merged) res += " " + EX(ex);
        out("merged " + in + " => " + res);
    }
}

// ---------------------------------------------------------------------------------------------
// Layer::has_compatible_layer_regions
// ---------------------------------------------------------------------------------------------
// One option of the second region's config differs from the first's (key 0: none; keys 24 and 25
// are options the function does not read), on the first layer and above it, in both orders of the
// arguments. The layer belongs to an object of a real Print: Layer.cpp:178-181 ask the Print for the
// process index of the wall filament.
static void set_compat_key(PrintRegionConfig &cfg, int key)
{
    switch (key) {
    case 1: cfg.wall_filament.value = 2; break;
    case 2: cfg.wall_loops.value = 3; break;
    case 3: cfg.wall_sequence.value = WallSequence::OuterInner; break;
    case 4: cfg.inner_wall_speed.values = {55.}; break;
    case 5: cfg.outer_wall_speed.values = {35.}; break;
    case 6: cfg.gap_infill_speed.values = {45.}; break;
    case 7: cfg.bridge_speed.values = {20.}; break;
    case 8: cfg.detect_overhang_wall.value = false; break;
    case 9: cfg.filter_out_gap_fill.value = 0.5; break;
    case 10: cfg.inner_wall_line_width.value = 0.5; break;
    case 11: cfg.outer_wall_line_width.value = 0.45; break;
    case 12: cfg.detect_thin_wall.value = true; break;
    case 13: cfg.infill_wall_overlap.value = 25.; break;
    case 14: cfg.override_filament_scarf_seam_setting.value = true; break;
    case 15: cfg.seam_slope_type.value = SeamScarfType::External; break;
    case 16: cfg.seam_slope_start_height.value = 20.; break;
    case 17: cfg.seam_slope_gap.value = 0.1; break;
    case 18: cfg.seam_slope_min_length.value = 20.; break;
    case 19: cfg.seam_slope_conditional.value = true; break;
    case 20: cfg.seam_slope_entire_loop.value = true; break;
    case 21: cfg.seam_slope_steps.value = 12; break;
    case 22: cfg.seam_slope_inner_walls.value = true; break;
    case 23: cfg.initial_layer_flow_ratio.value = 0.9; break;
    case 24: cfg.sparse_infill_density.value = 30.; break;
    case 25: cfg.top_shell_layers.value = 7; break;
    case 26: cfg.seam_slope_start_height.percent = true; break;
    default: break;
    }
}

static void run_compat()
{
    Model        model;
    ModelObject *o = model.add_object();
    o->add_volume(TriangleMesh(its_make_cube(20., 20., 6.)));
    o->add_instance()->set_offset(Vec3d(100., 100., 0.));
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    Print print;
    print.set_status_callback([](const PrintBase::SlicingStatus &) {});
    print.apply(model, config);
    PrintObject *obj = print.m_objects.front();

    PrintRegionConfig base;
    base.wall_filament.value                        = 1;
    base.wall_loops.value                           = 2;
    base.wall_sequence.value                        = WallSequence::InnerOuter;
    base.inner_wall_speed.values                    = {60.};
    base.outer_wall_speed.values                    = {30.};
    base.gap_infill_speed.values                    = {50.};
    base.bridge_speed.values                        = {25.};
    base.detect_overhang_wall.value                 = true;
    base.filter_out_gap_fill.value                  = 0.;
    base.inner_wall_line_width.value                = 0.45;
    base.outer_wall_line_width.value                = 0.42;
    base.detect_thin_wall.value                     = false;
    base.infill_wall_overlap.value                  = 15.;
    base.override_filament_scarf_seam_setting.value = false;
    base.seam_slope_type.value                      = SeamScarfType::None;
    base.seam_slope_start_height.value              = 10.;
    base.seam_slope_start_height.percent            = false;
    base.seam_slope_gap.value                       = 0.;
    base.seam_slope_gap.percent                     = false;
    base.seam_slope_min_length.value                = 10.;
    base.seam_slope_conditional.value               = false;
    base.seam_slope_entire_loop.value               = false;
    base.seam_slope_steps.value                     = 10;
    base.seam_slope_inner_walls.value               = false;
    base.initial_layer_flow_ratio.value             = 1.;
    base.sparse_infill_density.value                = 15.;
    base.top_shell_layers.value                     = 3;

    const int layer_ids[3] = {0, 1, 7};
    for (int layer_id : layer_ids) {
        Layer layer(size_t(layer_id), obj, 0.2, 0.2 * (layer_id + 1), 0.2 * layer_id + 0.1);
        for (int key = 0; key <= 26; ++key)
            for (int order = 0; order < 2; ++order) {
                PrintRegionConfig other = base;
                set_compat_key(other, key);
                // Layer.cpp:173-198
                const bool res = order == 0 ? layer.has_compatible_layer_regions(base, other) : layer.has_compatible_layer_regions(other, base);
                out("compat " + hI(layer_id) + " " + hI(key) + " " + hI(order) + " => " + hB(res));
            }
    }
}

// ---------------------------------------------------------------------------------------------
// The merge of the slices of compatible regions in Layer::make_perimeters
// ---------------------------------------------------------------------------------------------
// Layer.cpp:379-380 merge the surfaces of one number of extra perimeters with
// offset_ex(Surfaces, ClipperSafetyOffset). The surfaces are those of run_slices.
static void run_group()
{
    Lcg g(0x6E0F5ULL);
    const coord_t cell = 400000;
    for (int c = 0; c < 200; ++c) {
        Surfaces  surfaces;
        const int nsurf = (int) g.range(1, 6);
        std::string in = hI(nsurf);
        for (int k = 0; k < nsurf; ++k) {
            const coord_t x0 = g.range(0, 5) * cell, y0 = g.range(0, 5) * cell;
            const int kind = (int) g.range(0, 3);
            ExPolygon ex;
            if (kind == 0) {
                ex.contour.points = {Point(x0, y0), Point(x0 + cell, y0), Point(x0 + cell, y0 + cell), Point(x0, y0 + cell)};
            } else if (kind == 1) {
                const coord_t mx = g.range(1, cell / 3), my = g.range(1, cell / 3);
                ex.contour.points = {Point(x0 + mx, y0 + my), Point(x0 + cell - mx, y0 + my), Point(x0 + cell - mx, y0 + cell - my), Point(x0 + mx, y0 + cell - my)};
            } else {
                Points pts = wall(g, x0 + cell / 2, y0 + cell / 2, g.range(cell / 4, cell / 2 + cell / 8), g.range(cell / 4, cell / 2 + cell / 8), true);
                pts.pop_back();
                ex.contour.points = pts;
            }
            if (g.range(0, 3) == 0) {
                const coord_t hx = x0 + cell / 2, hy = y0 + cell / 2, hr = g.range(cell / 40, cell / 10);
                Polygon hole;
                hole.points = {Point(hx - hr, hy - hr), Point(hx - hr, hy + hr), Point(hx + hr, hy + hr), Point(hx + hr, hy - hr)};
                ex.holes.push_back(hole);
            }
            in += " " + EX(ex);
            surfaces.emplace_back(stInternal, ex);
        }
        // Layer.cpp:380
        const ExPolygons merged = offset_ex(surfaces, ClipperSafetyOffset);
        std::string res = hI((long long) merged.size());
        for (const ExPolygon &ex : merged) res += " " + EX(ex);
        out("group " + in + " => " + res);
    }
}

// ---------------------------------------------------------------------------------------------
// The fill surfaces of a shared perimeter pass split back among the regions
// ---------------------------------------------------------------------------------------------
// Layer.cpp:395-403: intersection_ex(fill surfaces, a region's slices) and
// intersection_ex(a region's slices, fill_no_overlap). The fill area is the merged slices
// (Layer.cpp:380) shrunk by the walls, fill_no_overlap the same shrunk a little more.
static void run_split()
{
    Lcg g(0x5B117ULL);
    const coord_t cell = 400000;
    for (int c = 0; c < 200; ++c) {
        const int nreg = (int) g.range(2, 4);
        std::vector<Surfaces> region_slices(nreg);
        Surfaces all;
        std::string in = hI(nreg);
        for (int r = 0; r < nreg; ++r) {
            const int nsurf = (int) g.range(1, 3);
            in += " " + hI(nsurf);
            for (int k = 0; k < nsurf; ++k) {
                const coord_t x0 = g.range(0, 5) * cell, y0 = g.range(0, 5) * cell;
                const int kind = (int) g.range(0, 3);
                ExPolygon ex;
                if (kind == 0) {
                    ex.contour.points = {Point(x0, y0), Point(x0 + cell, y0), Point(x0 + cell, y0 + cell), Point(x0, y0 + cell)};
                } else if (kind == 1) {
                    const coord_t mx = g.range(1, cell / 3), my = g.range(1, cell / 3);
                    ex.contour.points = {Point(x0 + mx, y0 + my), Point(x0 + cell - mx, y0 + my), Point(x0 + cell - mx, y0 + cell - my), Point(x0 + mx, y0 + cell - my)};
                } else {
                    Points pts = wall(g, x0 + cell / 2, y0 + cell / 2, g.range(cell / 4, cell / 2 + cell / 8), g.range(cell / 4, cell / 2 + cell / 8), true);
                    pts.pop_back();
                    ex.contour.points = pts;
                }
                in += " " + EX(ex);
                region_slices[r].emplace_back(stInternal, ex);
                all.emplace_back(stInternal, ex);
            }
        }
        const coord_t walls = g.range(20000, 90000);
        const ExPolygons fill_area       = offset_ex(offset_ex(all, ClipperSafetyOffset), -float(walls));
        const ExPolygons fill_no_overlap = offset_ex(fill_area, -float(walls / 4));
        Surfaces fill_surfaces;
        for (const ExPolygon &ex : fill_area) fill_surfaces.emplace_back(stInternal, ex);
        in += " " + hI((long long) fill_area.size());
        for (const ExPolygon &ex : fill_area) in += " " + EX(ex);
        in += " " + hI((long long) fill_no_overlap.size());
        for (const ExPolygon &ex : fill_no_overlap) in += " " + EX(ex);
        std::string res;
        for (int r = 0; r < nreg; ++r) {
            // Layer.cpp:398
            const ExPolygons expp = intersection_ex(fill_surfaces, region_slices[r]);
            // Layer.cpp:402
            const ExPolygons no_overlap = intersection_ex(region_slices[r], fill_no_overlap);
            res += (res.empty() ? "" : " ") + hI((long long) expp.size());
            for (const ExPolygon &ex : expp) res += " " + EX(ex);
            res += " " + hI((long long) no_overlap.size());
            for (const ExPolygon &ex : no_overlap) res += " " + EX(ex);
        }
        out("split " + in + " => " + res);
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
    run_distancer();
    run_continuity();
    run_slices();
    run_merged();
    run_compat();
    run_group();
    run_split();
    std::fclose(g_out);
    return 0;
}
