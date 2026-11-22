// Generator of the vectors of tests/layer_region_native_parity.rs: the member functions of LayerRegion
// of libslic3r's LayerRegion.cpp at the pin (references/BambuStudio/src/libslic3r; the override layer has
// no copy of it), called on the LayerRegion of a Layer of a real Print.
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
// A line "lcfg<TAB>tag<TAB>key=value..." holds the options the functions read as the pin's Print holds
// them after Print::apply; the lines after it are computed with it. Every other line is
// "<what> <inputs> => <results>": the test reads the inputs back, calls the port and has to print the
// same results. Tokens are separated by one space; a double is its 16 hexadecimal digits, a float its 8,
// an integer decimal. A surface is "type thickness thickness_layers bridge_angle expolygon".
// The vectors were made on macOS arm64 with Apple clang and Eigen 5.0.1.

#include <algorithm>
#include <cassert>
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
#include <tbb/spin_mutex.h>

#define private public
#define protected public
#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#undef private
#undef protected

#include "libslic3r/ClipperUtils.hpp"
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
static std::string SURFACES(const Surfaces &ss)
{
    std::string o = hI((long long) ss.size());
    for (const Surface &s : ss)
        o += " " + hI((int) s.surface_type) + " " + hD(s.thickness) + " " + hI(s.thickness_layers) + " " + hD(s.bridge_angle) + " " + EXPOLY(s.expolygon);
    return o;
}
static std::string FLOW(const Flow &f)
{
    return hF(f.width()) + " " + hF(f.height()) + " " + hF(f.spacing()) + " " + hF(f.nozzle_diameter()) + " " + hB(f.bridge()) + " " + hD(f.mm3_per_mm());
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
// A polygon of n corners on an ellipse, counter clockwise.
static Polygon round(coord_t cx, coord_t cy, double rx, double ry, int n, double phase)
{
    Polygon p;
    for (int i = 0; i < n; ++i) {
        const double a = phase + 2. * M_PI * i / n;
        p.points.emplace_back(Point(cx + coord_t(std::lround(rx * std::cos(a))), cy + coord_t(std::lround(ry * std::sin(a)))));
    }
    return p;
}
static coord_t mm(Lcg &g, double lo, double hi) { return g.range((long long) (lo * 100000.), (long long) (hi * 100000.)); }

// An island of a layer: a plate with zero to two holes.
static ExPolygon island(Lcg &g, coord_t cx, coord_t cy)
{
    const coord_t w = mm(g, 14., 30.), h = mm(g, 12., 24.);
    ExPolygon     e(g.range(0, 2) == 0 ? round(cx, cy, w / 2., h / 2., (int) g.range(16, 48), 0.05 * g.range(0, 60)) : rect(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2));
    const int     nh = (int) g.range(0, 2);
    for (int i = 0; i < nh; ++i) {
        Polygon hole = round(cx + (i == 0 ? -1 : 1) * w / 5 + mm(g, -0.5, 0.5), cy + mm(g, -1., 1.), double(mm(g, 0.8, 2.2)), double(mm(g, 0.8, 2.2)), (int) g.range(8, 24), 0.1 * g.range(0, 30));
        hole.make_clockwise();
        e.holes.emplace_back(hole);
    }
    return e;
}
// A patch inside the box of an island: a rectangle or a round one, of a few millimetres.
static Polygon patch(Lcg &g, const BoundingBox &bb)
{
    const coord_t cx = g.range(bb.min.x(), bb.max.x()), cy = g.range(bb.min.y(), bb.max.y());
    const coord_t w = mm(g, 1.5, 9.), h = mm(g, 1.5, 9.);
    return g.range(0, 1) == 0 ? rect(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2) : round(cx, cy, w / 2., h / 2., (int) g.range(8, 32), 0.1 * g.range(0, 30));
}

using KeyValues = std::vector<std::pair<std::string, std::string>>;

static void set_keys(DynamicPrintConfig &full, const KeyValues &kvs)
{
    for (const auto &kv : kvs) full.set_deserialize_strict(kv.first, kv.second);
}

// The keys the functions read, written for every case as the pin holds them after Print::apply.
static const char *const keys[] = {
    // PrintConfig
    "nozzle_diameter", "resolution", "enable_arc_fitting", "spiral_mode", "initial_layer_line_width", "initial_layer_infill_line_width", "initial_layer_print_height",
    // PrintObjectConfig
    "layer_height", "line_width", "thick_bridges",
    // PrintRegionConfig
    "wall_loops", "wall_filament", "sparse_infill_filament", "solid_infill_filament", "outer_wall_line_width", "inner_wall_line_width", "sparse_infill_line_width",
    "internal_solid_infill_line_width", "top_surface_line_width", "sparse_infill_density", "minimum_sparse_infill_area", "bridge_angle", "bridge_flow", "bottom_shell_layers",
    "top_shell_layers",
};

struct Case
{
    const char *tag;
    KeyValues   kvs;
};

// The fill surfaces of a layer for process_external_surfaces: per island the area inside its walls, cut
// into top, bottom and bridge patches, a solid band and the sparse rest. `kind` 0 all types, 1 no bridges,
// 2 bridges only, 3 a top that covers the island, 4 a bottom that covers it, 5 a bridge that covers it,
// 6 bridges in a bottom (4 to 6: a layer without a top, a solid or a sparse area).
static Surfaces typed_surfaces(Lcg &g, const ExPolygons &islands, double thickness, int kind)
{
    Surfaces out_surfaces;
    for (const ExPolygon &isl : islands) {
        ExPolygons        rest = offset_ex(ExPolygons{isl}, -float(mm(g, 0.6, 1.3)));
        const BoundingBox bb   = get_extents(isl);
        auto take = [&](SurfaceType type, int n) {
            for (int i = 0; i < n; ++i) {
                Polygon          p    = patch(g, bb);
                const ExPolygons part = intersection_ex(rest, ExPolygons{ExPolygon(p)});
                if (part.empty()) continue;
                rest = diff_ex(rest, ExPolygons{ExPolygon(p)});
                for (const ExPolygon &e : part) {
                    Surface s(type, e);
                    s.thickness = thickness;
                    out_surfaces.emplace_back(s);
                }
            }
        };
        if (kind == 3 || kind == 4 || kind == 5) {
            for (const ExPolygon &e : rest) {
                Surface s(kind == 3 ? stTop : kind == 4 ? stBottom : stBottomBridge, e);
                s.thickness = thickness;
                out_surfaces.emplace_back(s);
            }
            continue;
        }
        if (kind == 6) {
            take(stBottomBridge, (int) g.range(1, 2));
            for (const ExPolygon &e : rest) {
                Surface s(stBottom, e);
                s.thickness = thickness;
                out_surfaces.emplace_back(s);
            }
            continue;
        }
        if (kind != 2) take(stTop, (int) g.range(0, 2));
        if (kind != 2) take(stBottom, (int) g.range(0, 2));
        if (kind != 1) take(stBottomBridge, (int) g.range(kind == 2 ? 1 : 0, 2));
        take(stInternalSolid, (int) g.range(0, 2));
        for (const ExPolygon &e : rest) {
            Surface s(stInternal, e);
            s.thickness = thickness;
            out_surfaces.emplace_back(s);
        }
    }
    return out_surfaces;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: gen <vectors.txt>\n");
        return 1;
    }
    tbb::global_control one_thread(tbb::global_control::max_allowed_parallelism, 1);
    g_out = std::fopen(argv[1], "w");
    if (g_out == nullptr) return 1;

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
                                   {"sparse_infill_density", "15%"},
                                   {"minimum_sparse_infill_area", "15"},
                                   {"bottom_shell_layers", "4"},
                                   {"top_shell_layers", "6"}};
    static const std::vector<Case> cases = {
        {"base", {}},
        {"1wall", {{"wall_loops", "1"}}},
        {"4walls", {{"wall_loops", "4"}}},
        {"no_walls", {{"wall_loops", "0"}}},
        {"thick_bridges", {{"thick_bridges", "1"}}},
        {"bridge_flow", {{"bridge_flow", "0.85"}}},
        {"thick_bridge_flow", {{"thick_bridges", "1"}, {"bridge_flow", "1.2"}}},
        {"bridge_angle", {{"bridge_angle", "30"}}},
        {"no_infill", {{"sparse_infill_density", "0%"}}},
        {"small_min_area", {{"minimum_sparse_infill_area", "2"}}},
        {"large_min_area", {{"minimum_sparse_infill_area", "60"}}},
        {"spiral", {{"spiral_mode", "1"}, {"wall_loops", "1"}, {"top_shell_layers", "0"}, {"sparse_infill_density", "0%"}}},
        {"widths", {{"layer_height", "0.2"}, {"outer_wall_line_width", "0.45"}, {"inner_wall_line_width", "0.5"}, {"internal_solid_infill_line_width", "0.45"}, {"top_surface_line_width", "0.4"}}},
        {"auto_widths", {{"outer_wall_line_width", "0"}, {"inner_wall_line_width", "0"}, {"sparse_infill_line_width", "0"}, {"internal_solid_infill_line_width", "0"}, {"top_surface_line_width", "0"}, {"line_width", "0"}}},
    };
    Lcg g(0x51ed270b3c6ef372ULL);
    // The stream of the "pesz" lines, apart, so that the lines above them stay what they were.
    Lcg g2(0x2b992ddfa23249d6ULL);
    // The stream of the "pes" lines with a surface of another type beside, apart for the same reason.
    Lcg g3(0x7c0f1e5ab3d4960bULL);
    for (const Case &c : cases) {
        Model        model;
        ModelObject *o = model.add_object();
        o->add_volume(TriangleMesh(its_make_cube(20., 20., 6.)));
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        set_keys(config, base);
        set_keys(config, c.kvs);
        Print print;
        print.set_status_callback([](const PrintBase::SlicingStatus &) {});
        print.apply(model, config);
        PrintObject       *obj    = print.m_objects.front();
        const PrintRegion &region = *obj->m_shared_regions->all_regions.front();
        std::string        cfg    = std::string("lcfg\t") + c.tag;
        for (const char *key : keys) {
            const ConfigBase *holder = region.config().has(key) ? static_cast<const ConfigBase *>(&region.config()) :
                                       obj->config().has(key)   ? static_cast<const ConfigBase *>(&obj->config()) :
                                       print.config().has(key)  ? static_cast<const ConfigBase *>(&print.config()) :
                                                                  nullptr;
            cfg += std::string("\t") + key + "=" + (holder == nullptr ? std::string("?") : holder->opt_serialize(key));
        }
        out(cfg);
        const double layer_height       = obj->config().layer_height.value;
        const double first_layer_height = print.config().initial_layer_print_height.value;

        for (int k = 0; k < 10; ++k) {
            const size_t layer_id = k < 2 ? 0 : (size_t) g.range(1, 9);
            const double height   = layer_id == 0 ? first_layer_height : layer_height;
            const double print_z  = first_layer_height + layer_height * double(layer_id);
            const double slice_z  = print_z - 0.5 * height;
            const std::string at  = hI((long long) layer_id) + " " + hD(height);
            Layer        layer(layer_id, obj, height, print_z, slice_z);
            LayerRegion *layerm = layer.add_region(&region);

            // LayerRegion::flow, bridging_flow, infill_area_threshold
            if (k < 3) {
                std::string r;
                for (FlowRole role : {frExternalPerimeter, frPerimeter, frInfill, frSolidInfill, frTopSolidInfill}) r += FLOW(layerm->flow(role)) + " ";
                for (FlowRole role : {frExternalPerimeter, frPerimeter, frInfill, frSolidInfill, frTopSolidInfill})
                    for (bool thick : {false, true}) r += FLOW(layerm->bridging_flow(role, thick)) + " ";
                r += FLOW(layerm->flow(frPerimeter, 0.28)) + " " + hD(layerm->infill_area_threshold());
                out("flow " + at + " => " + r);
            }

            // One or two islands.
            ExPolygons islands;
            const int  n_islands = k % 3 == 2 ? 2 : 1;
            for (int i = 0; i < n_islands; ++i) islands.emplace_back(island(g, (n_islands == 1 ? 0 : (i == 0 ? -2500000 : 2500000)) + g.range(-200000, 200000), g.range(-200000, 200000)));

            // LayerRegion::slices_to_fill_surfaces_clipped: the slices of the islands by type, clipped by the
            // areas inside their walls.
            {
                layerm->slices.surfaces = typed_surfaces(g, islands, height, (int) g.range(0, 1));
                // The slices reach to the outline of the islands; the fill areas are what the walls leave.
                layerm->fill_expolygons = offset_ex(islands, -float(mm(g, 0.3, 1.5)));
                if (g.range(0, 3) == 0) layerm->fill_expolygons.clear();
                const std::string in = SURFACES(layerm->slices.surfaces) + " " + EXPOLYS(layerm->fill_expolygons);
                layerm->slices_to_fill_surfaces_clipped();
                out("clip " + at + " " + in + " => " + SURFACES(layerm->fill_surfaces.surfaces));
            }

            // LayerRegion::prepare_fill_surfaces: sparse areas below the smallest become solid.
            {
                layerm->fill_surfaces.surfaces = typed_surfaces(g, islands, height, (int) g.range(0, 1));
                // Small sparse areas beside: around the threshold of 15 mm2.
                for (int i = 0; i < 4; ++i) {
                    const coord_t w = mm(g, 1., 9.), h = mm(g, 1., 9.), x = 6000000 + i * 1200000, y = g.range(-200000, 200000);
                    Surface       s(i == 3 ? stInternalSolid : stInternal, ExPolygon(rect(x, y, x + w, y + h)));
                    s.thickness = height;
                    layerm->fill_surfaces.surfaces.emplace_back(s);
                }
                const std::string in = SURFACES(layerm->fill_surfaces.surfaces);
                layerm->prepare_fill_surfaces();
                out("prep " + at + " " + in + " => " + SURFACES(layerm->fill_surfaces.surfaces));
            }

            // LayerRegion::trim_surfaces and elephant_foot_compensation_step: the untyped slices of the
            // islands and the polygons that trim them (the islands shrunk, as the compensation makes them).
            {
                const Polygons trimming = to_polygons(offset_ex(islands, -float(mm(g, 0.05, 0.6))));
                layerm->slices.surfaces.clear();
                for (const ExPolygon &e : islands) layerm->slices.surfaces.emplace_back(Surface(stInternal, e));
                std::string in = SURFACES(layerm->slices.surfaces) + " " + POLYS(trimming);
                layerm->trim_surfaces(trimming);
                out("trim " + at + " " + in + " => " + SURFACES(layerm->slices.surfaces));

                layerm->slices.surfaces.clear();
                for (const ExPolygon &e : islands) layerm->slices.surfaces.emplace_back(Surface(stInternal, e));
                const float step = float(mm(g, 0.1, 0.9));
                in               = hF(step) + " " + SURFACES(layerm->slices.surfaces) + " " + POLYS(trimming);
                layerm->elephant_foot_compensation_step(step, trimming);
                out("efc " + at + " " + in + " => " + SURFACES(layerm->slices.surfaces));
            }

            // LayerRegion::process_external_surfaces
            for (int rep = 0; rep < 2; ++rep) {
                layerm->fill_surfaces.surfaces = typed_surfaces(g, islands, height, rep == 0 ? 0 : (int) g.range(0, 3));
                const std::string in           = SURFACES(layerm->fill_surfaces.surfaces);
                layerm->process_external_surfaces(nullptr, nullptr);
                out("pes " + at + " " + in + " => " + SURFACES(layerm->fill_surfaces.surfaces));
            }
        }

        // LayerRegion::process_external_surfaces on a layer without a top, a solid or a sparse area
        // (LayerRegion.cpp:821-862 has no exit for it): all bottom, all bridge, bridges in a bottom.
        for (int k = 0; k < 9; ++k) {
            const size_t layer_id = k < 3 ? 0 : (size_t) g2.range(1, 9);
            const double height   = layer_id == 0 ? first_layer_height : layer_height;
            const double print_z  = first_layer_height + layer_height * double(layer_id);
            const double slice_z  = print_z - 0.5 * height;
            const std::string at  = hI((long long) layer_id) + " " + hD(height);
            Layer        layer(layer_id, obj, height, print_z, slice_z);
            LayerRegion *layerm = layer.add_region(&region);
            ExPolygons   islands;
            const int    n_islands = k >= 6 ? 2 : 1;
            for (int i = 0; i < n_islands; ++i) islands.emplace_back(island(g2, (n_islands == 1 ? 0 : (i == 0 ? -2500000 : 2500000)) + g2.range(-200000, 200000), g2.range(-200000, 200000)));
            layerm->fill_surfaces.surfaces = typed_surfaces(g2, islands, height, 4 + k % 3);
            const std::string in           = SURFACES(layerm->fill_surfaces.surfaces);
            layerm->process_external_surfaces(nullptr, nullptr);
            out("pesz " + at + " " + in + " => " + SURFACES(layerm->fill_surfaces.surfaces));
        }

        // LayerRegion::process_external_surfaces with a surface of a type it does not expand beside the
        // others: the function clears the fill surfaces before it stores its own (LayerRegion.cpp:885).
        for (int k = 0; k < 3; ++k) {
            const size_t layer_id = (size_t) g3.range(1, 9);
            const double height   = layer_height;
            const double print_z  = first_layer_height + layer_height * double(layer_id);
            const double slice_z  = print_z - 0.5 * height;
            const std::string at  = hI((long long) layer_id) + " " + hD(height);
            Layer        layer(layer_id, obj, height, print_z, slice_z);
            LayerRegion *layerm = layer.add_region(&region);
            ExPolygons   islands;
            islands.emplace_back(island(g3, g3.range(-200000, 200000), g3.range(-200000, 200000)));
            layerm->fill_surfaces.surfaces = typed_surfaces(g3, islands, height, k);
            {
                const coord_t w = mm(g3, 1., 9.), h = mm(g3, 1., 9.), x = 6000000, y = g3.range(-200000, 200000);
                Surface       s(k == 1 ? stInternalVoid : stInternalBridge, ExPolygon(rect(x, y, x + w, y + h)));
                s.thickness = height;
                layerm->fill_surfaces.surfaces.emplace_back(s);
            }
            const std::string in = SURFACES(layerm->fill_surfaces.surfaces);
            layerm->process_external_surfaces(nullptr, nullptr);
            out("pes " + at + " " + in + " => " + SURFACES(layerm->fill_surfaces.surfaces));
        }
    }
    std::fclose(g_out);
    return 0;
}
