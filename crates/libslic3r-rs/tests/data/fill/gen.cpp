// Generator of the vectors of tests/fill_native_parity.rs: `group_fills` of libslic3r's Fill/Fill.cpp at
// the pin (references/BambuStudio/src/libslic3r; the override layer has no copy of Fill.cpp), on a real
// Print made by `Print::apply` and layers whose fill surfaces are set by hand.
//
// Build as tests/data/perimeter_generator/gen.cpp says (the build's compile line of a libslic3r source,
// linked with the build's libslic3r_core and the libraries it links), then `./gen vectors.txt`.
// Fill.cpp is compiled into this file (SurfaceFillParams and SurfaceFill are defined in it), with the
// private members of Print and Layer made public.
//
// "gcfg<TAB>tag<TAB>key=value..." holds the options as the pin's Print holds them after `Print::apply`.
// "gf <layer> <surfaces> <lower layer> => <the SurfaceFill list>": tokens separated by one space; a
// double is its 16 hexadecimal digits, a float its 8, an integer decimal.
// "mf <object box> <regions> R <entities>...": what the pin's Layer::make_fills leaves in the `fills`
// of each region of the layer of the "gf" line before it, in order.
// `./gen vectors.txt vectors_ironing.txt` also writes the Layer::make_ironing lines ("ic", "il"; see
// `run_ironing`) to the second file; the first file is the same with or without it.
// `./gen vectors.txt vectors_ironing.txt vectors_anchoring.txt` also writes the
// Layer::generate_sparse_infill_polylines_for_anchoring lines ("acfg", "ga"; see `run_anchoring`) to the
// third file; the first two are the same with or without it.
// The vectors were made on macOS arm64 with Apple clang and Eigen 5.0.1.

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
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
#include "libslic3r/Fill/Fill.cpp"
#undef private
#undef protected

#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;

typedef std::vector<std::pair<std::string, std::string>> KeyValues;

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

static void set_keys(DynamicPrintConfig &full, const KeyValues &kvs)
{
    for (const auto &kv : kvs) full.set_deserialize_strict(kv.first, kv.second);
}

// The options of the "gcfg" line: what group_fills reads.
static const char *const fill_keys[] = {
    // PrintConfig
    "nozzle_diameter", "initial_layer_line_width", "initial_layer_infill_line_width", "initial_layer_print_height",
    // PrintObjectConfig
    "layer_height", "line_width", "thick_bridges", "raft_layers", "detect_narrow_internal_solid_infill",
    // PrintRegionConfig
    "wall_loops", "wall_filament", "sparse_infill_filament", "solid_infill_filament", "outer_wall_line_width", "inner_wall_line_width", "sparse_infill_line_width",
    "internal_solid_infill_line_width", "top_surface_line_width", "sparse_infill_density", "sparse_infill_pattern", "top_surface_pattern", "bottom_surface_pattern",
    "internal_solid_infill_pattern", "infill_direction", "infill_wall_overlap", "infill_combination", "bridge_angle", "bridge_flow", "bottom_shell_layers", "top_shell_layers",
    "minimum_sparse_infill_area", "ironing_type",
};

struct FillCase
{
    const char *tag;
    KeyValues   kvs;
};

// The list of SurfaceFill of group_fills, in its order.
static std::string FILLS(const std::vector<SurfaceFill> &fills)
{
    std::string o = hI((long long) fills.size());
    for (const SurfaceFill &f : fills) {
        const SurfaceFillParams &p = f.params;
        o += " F " + hI(f.region_id == size_t(-1) ? -1 : (long long) f.region_id) + " " + hI((int) f.surface.surface_type) + " " + hD(f.surface.thickness) + " " + hI(p.extruder) + " " +
             hI((int) p.pattern) + " " + hD(p.spacing) + " " + hD(p.overlap) + " " + hF(p.angle) + " " + hB(p.bridge) + " " + hF(p.bridge_angle) + " " + hF(p.density) + " " +
             hI(p.multiline) + " " + hF(p.anchor_length) + " " + hF(p.anchor_length_max) + " " + hF(p.flow.width()) + " " + hF(p.flow.height()) + " " + hF(p.flow.nozzle_diameter()) +
             " " + hF(p.flow.spacing()) + " " + hB(p.flow.bridge()) + " " + hI((int) p.extrusion_role) + " " + hI((long long) f.region_id_group.size());
        for (size_t r : f.region_id_group) o += " " + hI((long long) r);
        o += " " + EXPOLYS(f.expolygons) + " " + EXPOLYS(f.no_overlap_expolygons);
    }
    return o;
}

// One extrusion entity of a region's `fills`, nested collections with their no_sort flag: "C" a
// collection, "L" a loop, "M" a multi path, "P" a path (role by name, width, height, mm3_per_mm, points).
static void ENT(const ExtrusionEntity *e, std::string &o)
{
    if (const ExtrusionEntityCollection *c = dynamic_cast<const ExtrusionEntityCollection *>(e)) {
        o += " C " + hB(c->no_sort) + " " + hI((long long) c->entities.size());
        for (const ExtrusionEntity *x : c->entities) ENT(x, o);
    } else if (const ExtrusionLoop *l = dynamic_cast<const ExtrusionLoop *>(e)) {
        o += " L " + hI((long long) l->paths.size());
        for (const ExtrusionPath &p : l->paths) ENT(&p, o);
    } else if (const ExtrusionMultiPath *m = dynamic_cast<const ExtrusionMultiPath *>(e)) {
        o += " M " + hI((long long) m->paths.size());
        for (const ExtrusionPath &p : m->paths) ENT(&p, o);
    } else if (const ExtrusionPath *p = dynamic_cast<const ExtrusionPath *>(e)) {
        std::string role = ExtrusionEntity::role_to_string(p->role());
        std::replace(role.begin(), role.end(), ' ', '_');
        o += " P " + role + " " + hF(p->width) + " " + hF(p->height) + " " + hD(p->mm3_per_mm) + " " + hI((long long) p->polyline.points.size());
        for (const Point &pt : p->polyline.points) o += " " + PT(pt);
    } else {
        o += " ?";
    }
}

static void run_group_fills()
{
    // What the cases share: the keys of the process of the fixtures (a 0.16 mm layer, a 0.4 mm nozzle).
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
                                   {"sparse_infill_density", "15%"},
                                   {"infill_wall_overlap", "15%"},
                                   {"bottom_shell_layers", "4"},
                                   {"top_shell_layers", "6"}};
    static const std::vector<FillCase> cases = {
        {"base", {}},
        {"grid", {{"sparse_infill_pattern", "grid"}}},
        {"gyroid_25", {{"sparse_infill_pattern", "gyroid"}, {"sparse_infill_density", "25%"}}},
        {"line_40", {{"sparse_infill_pattern", "line"}, {"sparse_infill_density", "40%"}}},
        {"dense_100", {{"sparse_infill_density", "100%"}}},
        {"no_infill", {{"sparse_infill_density", "0%"}}},
        {"top_concentric", {{"top_surface_pattern", "concentric"}}},
        {"top_monotonicline", {{"top_surface_pattern", "monotonicline"}}},
        {"bottom_concentric", {{"bottom_surface_pattern", "concentric"}}},
        {"solid_concentric", {{"internal_solid_infill_pattern", "concentric"}}},
        {"solid_rectilinear", {{"internal_solid_infill_pattern", "zig-zag"}}},
        {"direction_30", {{"infill_direction", "30"}}},
        {"no_narrow", {{"detect_narrow_internal_solid_infill", "0"}}},
        {"thick_bridges", {{"thick_bridges", "1"}}},
        {"bridge_flow_09", {{"bridge_flow", "0.9"}}},
        {"layer_02", {{"layer_height", "0.2"}}},
        {"widths", {{"sparse_infill_line_width", "0.6"}, {"internal_solid_infill_line_width", "0.5"}, {"top_surface_line_width", "0.4"}}},
        {"overlap_30", {{"infill_wall_overlap", "30%"}}},
        {"bridge_angle_45", {{"bridge_angle", "45"}}},
        {"honeycomb", {{"sparse_infill_pattern", "honeycomb"}}},
        // The last case (the lines of the cases before it stay as they were): sub-top surfaces, narrow
        // and wide, for Fill.cpp:556-558 and 570.
        {"sub_top", {}},
    };
    static const SurfaceType types_sub_top[] = {stSubTop, stInternalSolid, stInternal, stSubTop, stTop, stSubTop, stInternalSolid};
    static const SurfaceType types[] = {stTop, stBottom, stBottomBridge, stInternal, stInternalSolid, stInternalBridge, stInternalVoid};
    Lcg g(0x9e3779b97f4a7c15ULL);
    for (const FillCase &fc : cases) {
        Model        model;
        ModelObject *o = model.add_object();
        o->add_volume(TriangleMesh(its_make_cube(20., 20., 6.)));
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        set_keys(config, base);
        set_keys(config, fc.kvs);
        Print print;
        print.set_status_callback([](const PrintBase::SlicingStatus &) {});
        print.apply(model, config);
        PrintObject       *obj    = print.m_objects.front();
        const PrintRegion &region = *obj->m_shared_regions->all_regions.front();
        std::string        cfg    = std::string("gcfg\t") + fc.tag;
        for (const char *key : fill_keys) {
            const ConfigBase *holder = region.config().has(key) ? static_cast<const ConfigBase *>(&region.config()) :
                                       obj->config().has(key)   ? static_cast<const ConfigBase *>(&obj->config()) :
                                       print.config().has(key)  ? static_cast<const ConfigBase *>(&print.config()) :
                                                                  nullptr;
            cfg += std::string("\t") + key + "=" + (holder == nullptr ? std::string("?") : holder->opt_serialize(key));
        }
        out(cfg);
        const double layer_height       = obj->config().layer_height.value;
        const double first_layer_height = print.config().initial_layer_print_height.value;
        for (int c = 0; c < 12; ++c) {
            // The first layer in two cases of twelve.
            const size_t layer_id = c < 2 ? 0 : (size_t) g.range(1, 9);
            const double height   = layer_id == 0 ? first_layer_height : layer_height;
            const double print_z  = first_layer_height + layer_height * double(layer_id);
            const double slice_z  = print_z - 0.5 * height;
            Layer        lower(layer_id == 0 ? 0 : layer_id - 1, obj, layer_height, print_z - height, slice_z - height);
            Layer        layer(layer_id, obj, height, print_z, slice_z);
            LayerRegion *layerm = layer.add_region(&region);
            // Two to six rectangles apart from each other, from narrow (under the 3 mm of
            // NARROW_INFILL_AREA_THRESHOLD on a side) to wide, every surface type in turn.
            const int   n  = (int) g.range(2, 6);
            std::string in = hI((long long) layer_id) + " " + hD(height) + " " + hD(print_z) + " " + hD(slice_z) + " " + hI(n);
            ExPolygons  rects;
            for (int i = 0; i < n; ++i) {
                const coord_t     x0 = coord_t(i) * 2000000 + g.range(-100000, 100000), y0 = g.range(-100000, 100000);
                const coord_t     w = g.range(0, 2) == 0 ? g.range(50000, 500000) : g.range(700000, 1500000), h = g.range(0, 2) == 0 ? g.range(50000, 500000) : g.range(700000, 1500000);
                const bool        sub_top = std::strcmp(fc.tag, "sub_top") == 0;
                const SurfaceType type    = c == 0 ? (i % 2 == 0 ? stBottom : stInternalSolid) : (sub_top ? types_sub_top : types)[g.range(0, 6)];
                Surface           s(type, ExPolygon(rect(x0, y0, x0 + w, y0 + h)));
                if (g.range(0, 5) == 0) s.thickness = 2. * layer_height;
                if (s.is_bridge() && g.range(0, 1) == 0) s.bridge_angle = 0.1 * double(g.range(0, 31));
                layerm->fill_surfaces.surfaces.emplace_back(s);
                rects.emplace_back(s.expolygon);
                in += " " + hI((int) type) + " " + hD(s.thickness) + " " + hD(s.bridge_angle) + " " + EXPOLY(s.expolygon);
            }
            // The layer below: sparse infill under some of the rectangles (Fill.cpp, the narrow solid areas).
            const bool has_lower = layer_id > 0 && g.range(0, 3) != 0;
            ExPolygons lower_internal;
            if (has_lower) {
                LayerRegion *lm = lower.add_region(&region);
                for (int i = 0; i < n; ++i)
                    if (g.range(0, 1) == 0) {
                        lm->fill_surfaces.surfaces.emplace_back(Surface(stInternal, rects[i]));
                        lower_internal.emplace_back(rects[i]);
                    }
                layer.lower_layer = &lower;
            }
            in += " " + hB(has_lower) + " " + EXPOLYS(lower_internal);
            LockRegionParam          lock_param;
            std::vector<SurfaceFill> fills = group_fills(layer, lock_param);
            out("gf " + in + " => " + FILLS(fills));
            // The pin's Layer::make_fills on the same layer (Fill.cpp:678-867; one thread, the real
            // fillers), the object holding one first layer whose bottom is at 0 for
            // `object()->get_layer(0)` (so the base of Fill.cpp:228-229 is the 0. of the line above).
            // "mf <the object's box> <regions> { R <entities of fills> ... }".
            {
                Layer base(0, obj, first_layer_height, first_layer_height, 0.5 * first_layer_height);
                obj->m_layers.emplace_back(&base);
                layer.make_fills(nullptr, nullptr, nullptr);
                obj->m_layers.pop_back();
                std::string mf = BBOX(obj->bounding_box()) + " " + hI((long long) layer.m_regions.size());
                for (const LayerRegion *lr : layer.m_regions) {
                    mf += " R " + hI((long long) lr->fills.entities.size());
                    for (const ExtrusionEntity *e : lr->fills.entities) ENT(e, mf);
                }
                out("mf " + mf);
                std::fflush(g_out);
            }
        }
    }
}

// The options of the "ic" line: what Layer::make_ironing reads (Fill.cpp:1059-1093, 1131-1196).
static const char *const ironing_keys[] = {
    "nozzle_diameter", "layer_height", "wall_loops", "wall_filament", "solid_infill_filament", "sparse_infill_density", "infill_direction", "top_shell_layers",
    "bottom_shell_layers", "ironing_type", "ironing_pattern", "ironing_flow", "ironing_spacing", "ironing_inset", "ironing_speed", "ironing_direction",
};

// Layer::make_ironing (Fill.cpp:983-1250) on layers whose slices, fill surfaces, lslices and upper layer
// are set by hand: "ic<TAB>tag<TAB>key=value..." the options, then per layer
// "il <layer> <height> <print_z> <slice_z> <has upper> <slices> <fill surfaces> <lslices> => <box> <regions> R <entities>...":
// what make_ironing appends to the `fills` (empty before) of each region.
static void run_ironing(FILE *fo)
{
    static const KeyValues base = {{"layer_height", "0.16"}, {"initial_layer_print_height", "0.2"}, {"wall_loops", "2"}, {"sparse_infill_density", "15%"},
                                   {"top_shell_layers", "6"}, {"bottom_shell_layers", "4"}, {"ironing_type", "top"}};
    static const std::vector<FillCase> cases = {
        {"iron_top", {}},
        {"iron_topmost", {{"ironing_type", "topmost"}}},
        {"iron_solid", {{"ironing_type", "solid"}}},
        {"iron_solid_dense", {{"ironing_type", "solid"}, {"sparse_infill_density", "100%"}}},
        {"iron_spacing_flow", {{"ironing_spacing", "0.2"}, {"ironing_flow", "15%"}}},
        {"iron_inset", {{"ironing_inset", "0.3"}}},
        {"iron_direction", {{"ironing_direction", "30"}, {"infill_direction", "100"}}},
        {"iron_concentric", {{"ironing_pattern", "concentric"}}},
        {"iron_none", {{"ironing_type", "no ironing"}}},
        {"iron_no_top_shells", {{"top_shell_layers", "0"}}},
        {"iron_solid_no_bottom", {{"ironing_type", "solid"}, {"bottom_shell_layers", "0"}}},
    };
    static const SurfaceType slice_types[] = {stTop, stBottom, stBottomBridge, stInternal, stTop};
    static const SurfaceType fill_types[] = {stTop, stInternalSolid, stSubTop, stInternal, stInternalBridge, stInternalVoid, stBottom};
    Lcg g(0x2545f4914f6cdd1dULL);
    for (const FillCase &fc : cases) {
        Model        model;
        ModelObject *o = model.add_object();
        o->add_volume(TriangleMesh(its_make_cube(20., 20., 6.)));
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        set_keys(config, base);
        set_keys(config, fc.kvs);
        Print print;
        print.set_status_callback([](const PrintBase::SlicingStatus &) {});
        print.apply(model, config);
        PrintObject       *obj    = print.m_objects.front();
        const PrintRegion &region = *obj->m_shared_regions->all_regions.front();
        std::string        cfg    = std::string("ic\t") + fc.tag;
        for (const char *key : ironing_keys) {
            const ConfigBase *holder = region.config().has(key) ? static_cast<const ConfigBase *>(&region.config()) :
                                       obj->config().has(key)   ? static_cast<const ConfigBase *>(&obj->config()) :
                                       print.config().has(key)  ? static_cast<const ConfigBase *>(&print.config()) :
                                                                  nullptr;
            cfg += std::string("\t") + key + "=" + (holder == nullptr ? std::string("?") : holder->opt_serialize(key));
        }
        std::fprintf(fo, "%s\n", cfg.c_str());
        const double layer_height = obj->config().layer_height.value;
        const double first_layer  = print.config().initial_layer_print_height.value;
        for (int c = 0; c < 8; ++c) {
            const size_t layer_id = (size_t) g.range(1, 9);
            const double print_z  = first_layer + layer_height * double(layer_id);
            const double slice_z  = print_z - 0.5 * layer_height;
            Layer        upper(layer_id + 1, obj, layer_height, print_z + layer_height, slice_z + layer_height);
            Layer        layer(layer_id, obj, layer_height, print_z, slice_z);
            const bool   has_upper = c % 3 != 0;
            if (has_upper) layer.upper_layer = &upper;
            LayerRegion *layerm = layer.add_region(&region);
            // Two to four islands apart from each other, 1.5 to 6 mm on a side; per island one slice
            // surface and the fill surfaces inside it, 0.8 mm in from its sides, in one or two parts.
            const int   n  = (int) g.range(2, 4);
            std::string in = hI((long long) layer_id) + " " + hD(layer_height) + " " + hD(print_z) + " " + hD(slice_z) + " " + hB(has_upper);
            ExPolygons  islands;
            Surfaces    fills;
            std::string sl = hI(n);
            for (int i = 0; i < n; ++i) {
                const coord_t     x0 = coord_t(i) * 8000000 + g.range(-200000, 200000), y0 = g.range(-200000, 200000);
                const coord_t     w = g.range(1500000, 6000000), h = g.range(1500000, 6000000);
                const SurfaceType type = slice_types[g.range(0, 4)];
                Surface           s(type, ExPolygon(rect(x0, y0, x0 + w, y0 + h)));
                layerm->slices.surfaces.emplace_back(s);
                islands.emplace_back(s.expolygon);
                sl += " " + hI((int) type) + " " + EXPOLY(s.expolygon);
                const coord_t in0 = 800000;
                if (g.range(0, 1) == 0) {
                    fills.emplace_back(Surface(fill_types[g.range(0, 6)], ExPolygon(rect(x0 + in0, y0 + in0, x0 + w - in0, y0 + h - in0))));
                } else {
                    const coord_t xm = x0 + w / 2;
                    fills.emplace_back(Surface(fill_types[g.range(0, 6)], ExPolygon(rect(x0 + in0, y0 + in0, xm, y0 + h - in0))));
                    fills.emplace_back(Surface(fill_types[g.range(0, 6)], ExPolygon(rect(xm, y0 + in0, x0 + w - in0, y0 + h - in0))));
                }
            }
            std::string fl = hI((long long) fills.size());
            for (const Surface &s : fills) {
                layerm->fill_surfaces.surfaces.emplace_back(s);
                fl += " " + hI((int) s.surface_type) + " " + EXPOLY(s.expolygon);
            }
            layer.lslices = islands;
            in += " " + sl + " " + fl + " " + EXPOLYS(islands);
            layer.make_ironing();
            std::string mi = BBOX(obj->bounding_box()) + " " + hI((long long) layer.m_regions.size());
            for (const LayerRegion *lr : layer.m_regions) {
                mi += " R " + hI((long long) lr->fills.entities.size());
                for (const ExtrusionEntity *e : lr->fills.entities) ENT(e, mi);
            }
            std::fprintf(fo, "il %s => %s\n", in.c_str(), mi.c_str());
            std::fflush(fo);
        }
    }
}

// Layer::generate_sparse_infill_polylines_for_anchoring (Fill.cpp:869-980) with no octrees and no
// lightning generator, as Fill.cpp:760 calls it, on layers whose fill surfaces are set by hand:
// "acfg<TAB>tag<TAB>key=value..." the options (those of the "gcfg" lines), then per layer
// "ga <layer> <height> <print_z> <slice_z> <surfaces> => <object box> <polylines>": the polylines in
// order, each "<points> x y...". The object holds one first layer whose bottom is at 0, as for "mf".
static void run_anchoring(FILE *fo)
{
    static const KeyValues base = {{"layer_height", "0.16"}, {"initial_layer_print_height", "0.2"}, {"wall_loops", "2"}, {"sparse_infill_line_width", "0.45"},
                                   {"sparse_infill_density", "15%"}, {"top_shell_layers", "6"}, {"bottom_shell_layers", "4"}};
    static const std::vector<FillCase> cases = {
        {"anchor_cubic", {{"sparse_infill_pattern", "cubic"}}},
        {"anchor_triangles", {{"sparse_infill_pattern", "triangles"}}},
        {"anchor_stars", {{"sparse_infill_pattern", "tri-hexagon"}}},
        {"anchor_cubic_30", {{"sparse_infill_pattern", "cubic"}, {"sparse_infill_density", "30%"}, {"infill_direction", "30"}}},
        {"anchor_triangles_dense", {{"sparse_infill_pattern", "triangles"}, {"sparse_infill_density", "85%"}}},
        {"anchor_grid", {{"sparse_infill_pattern", "grid"}}},
        {"anchor_zigzag", {{"sparse_infill_pattern", "zig-zag"}}},
        // R940: the gyroid arm of generate_sparse_infill_polylines_for_anchoring (Layer.cpp), appended so the
        // cases above keep their inputs.
        {"anchor_gyroid", {{"sparse_infill_pattern", "gyroid"}}},
        {"anchor_gyroid_30", {{"sparse_infill_pattern", "gyroid"}, {"sparse_infill_density", "30%"}, {"infill_direction", "30"}}},
        {"anchor_gyroid_dense", {{"sparse_infill_pattern", "gyroid"}, {"sparse_infill_density", "60%"}}},
    };
    static const SurfaceType types[] = {stInternal, stInternal, stInternal, stInternalSolid, stTop};
    Lcg g(0x2545f4914f6cdd1dULL);
    for (const FillCase &fc : cases) {
        Model        model;
        ModelObject *o = model.add_object();
        o->add_volume(TriangleMesh(its_make_cube(20., 20., 6.)));
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        set_keys(config, base);
        set_keys(config, fc.kvs);
        Print print;
        print.set_status_callback([](const PrintBase::SlicingStatus &) {});
        print.apply(model, config);
        PrintObject       *obj    = print.m_objects.front();
        const PrintRegion &region = *obj->m_shared_regions->all_regions.front();
        std::string        cfg    = std::string("acfg\t") + fc.tag;
        for (const char *key : fill_keys) {
            const ConfigBase *holder = region.config().has(key) ? static_cast<const ConfigBase *>(&region.config()) :
                                       obj->config().has(key)   ? static_cast<const ConfigBase *>(&obj->config()) :
                                       print.config().has(key)  ? static_cast<const ConfigBase *>(&print.config()) :
                                                                  nullptr;
            cfg += std::string("\t") + key + "=" + (holder == nullptr ? std::string("?") : holder->opt_serialize(key));
        }
        std::fprintf(fo, "%s\n", cfg.c_str());
        const double layer_height       = obj->config().layer_height.value;
        const double first_layer_height = print.config().initial_layer_print_height.value;
        for (int c = 0; c < 8; ++c) {
            const size_t layer_id = (size_t) g.range(1, 9);
            // R940: every other gyroid layer is thinner than the object's layer height, so the sparse
            // surface's spacing (the flow at the object's layer height, Fill.cpp:305-328) differs from the
            // layer's flow spacing; the earlier configurations keep their inputs.
            const bool   thin     = std::strncmp(fc.tag, "anchor_gyroid", 13) == 0 && (c % 2) == 1;
            const double height   = thin ? 0.08 : layer_height;
            const double print_z  = first_layer_height + layer_height * double(layer_id);
            const double slice_z  = print_z - 0.5 * height;
            Layer        layer(layer_id, obj, height, print_z, slice_z);
            LayerRegion *layerm = layer.add_region(&region);
            // One to four rectangles apart from each other, wide enough for sparse infill.
            const int   n  = (int) g.range(1, 4);
            std::string in = hI((long long) layer_id) + " " + hD(height) + " " + hD(print_z) + " " + hD(slice_z) + " " + hI(n);
            for (int i = 0; i < n; ++i) {
                const coord_t     x0 = coord_t(i) * 2500000 + g.range(-100000, 100000), y0 = g.range(-100000, 100000);
                const coord_t     w = g.range(400000, 2200000), h = g.range(400000, 2200000);
                const SurfaceType type = types[g.range(0, 4)];
                Surface           s(type, ExPolygon(rect(x0, y0, x0 + w, y0 + h)));
                layerm->fill_surfaces.surfaces.emplace_back(s);
                in += " " + hI((int) type) + " " + hD(s.thickness) + " " + hD(s.bridge_angle) + " " + EXPOLY(s.expolygon);
            }
            Layer base(0, obj, first_layer_height, first_layer_height, 0.5 * first_layer_height);
            obj->m_layers.emplace_back(&base);
            Polylines pls = layer.generate_sparse_infill_polylines_for_anchoring(nullptr, nullptr, nullptr);
            obj->m_layers.pop_back();
            std::string res = BBOX(obj->bounding_box()) + " " + hI((long long) pls.size());
            for (const Polyline &pl : pls) {
                res += " " + hI((long long) pl.points.size());
                for (const Point &p : pl.points) res += " " + PT(p);
            }
            std::fprintf(fo, "ga %s => %s\n", in.c_str(), res.c_str());
            std::fflush(fo);
        }
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: gen <vectors.txt> [<vectors_ironing.txt> [<vectors_anchoring.txt>]]\n");
        return 2;
    }
    tbb::global_control one_thread(tbb::global_control::max_allowed_parallelism, 1);
    g_out = std::fopen(argv[1], "w");
    if (g_out == nullptr) return 1;
    run_group_fills();
    std::fclose(g_out);
    // `gen vectors.txt vectors_ironing.txt`: the second file holds the make_ironing lines.
    if (argc >= 3) {
        FILE *fo = std::fopen(argv[2], "w");
        if (fo == nullptr) return 1;
        run_ironing(fo);
        std::fclose(fo);
    }
    // `gen vectors.txt vectors_ironing.txt vectors_anchoring.txt`: the third file holds the
    // generate_sparse_infill_polylines_for_anchoring lines.
    if (argc >= 4) {
        FILE *fo = std::fopen(argv[3], "w");
        if (fo == nullptr) return 1;
        run_anchoring(fo);
        std::fclose(fo);
    }
    return 0;
}
