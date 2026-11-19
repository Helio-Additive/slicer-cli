// Generator of the vectors of tests/print_object_slice_native_parity.rs: libslic3r's PrintObjectSlice.cpp
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
//   ./gen vectors.txt          (./gen vectors.txt full   writes every line in full;
//                               ./gen vectors.txt full 3 7   the scenarios 3 and 7 alone)
//
// PrintObjectSlice.cpp is compiled into this file (its helpers are static), with the private members
// of Print, PrintObject and Layer made public. The two segmentations by painting it calls are routed
// through a hook that writes what they return as input lines, so the test feeds the same segmentation
// to the port. tbb runs on one thread.
//
// Every scenario is a Model with a full print config put through `Print::apply`; its objects are
// sliced with `PrintObject::slice()` and the layers written out (lslices, the surfaces of every
// region, the boxes, the raw slices, the slices by volume and by group of the first layer), after
// the direct calls of the helpers on the same object (new_layers, slice_volumes_inner, slice_volume
// with and without ranges, slices_to_regions) and before those that read the sliced object
// (slice_support_volumes, support_shrinkage_scale, should_skip_filament_shrink). The remaining
// helpers run on synthetic inputs, which are in the vectors as lines that start with "in".
//
// Tokens are separated by one space; a double is its 16 hexadecimal digits, a float its 8, an
// integer decimal. The long part of a line is "#" and the FNV-1a hash of its text unless the
// generator is run with "full". The vectors were made on macOS arm64 with Apple clang and Eigen 5.0.1.

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
#include "libslic3r/AdaptiveLayerMerge.hpp"
#include "libslic3r/ElephantFootCompensation.hpp"
#include "libslic3r/I18N.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/MultiMaterialSegmentation.hpp"
namespace Slic3r {
static std::vector<std::vector<ExPolygons>> gen_mms_hook(const PrintObject &print_object, const std::function<void()> &throw_on_cancel_callback);
static std::vector<std::vector<ExPolygons>> gen_fuzzy_hook(const PrintObject &print_object, const std::function<void()> &throw_on_cancel_callback);
} // namespace Slic3r
#define multi_material_segmentation_by_painting gen_mms_hook
#define fuzzy_skin_segmentation_by_painting gen_fuzzy_hook
#include "libslic3r/PrintObjectSlice.cpp"
#undef multi_material_segmentation_by_painting
#undef fuzzy_skin_segmentation_by_painting
#undef protected
#undef private
#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;

static FILE *g_out = nullptr;
// Every line in full (the second argument "full").
static bool g_full = false;
// The tag of the object the hooks write for.
static std::string g_hook_tag;

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
static void outb(const std::string &head, const std::string &body) { out(g_full ? head + " " + body : head + " #" + fnv(body)); }
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
static std::string BBOX(const BoundingBox &b) { return hB(b.defined) + " " + PT(b.min) + " " + PT(b.max); }
static std::string SURFACES(const Surfaces &ss)
{
    std::string o = hI((long long) ss.size());
    for (const Surface &s : ss) o += " " + hI((int) s.surface_type) + " " + EXPOLY(s.expolygon);
    return o;
}

// The same generator as the test has for the synthetic inputs.
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
// A convex polygon of n corners around (cx, cy), counter clockwise, with integer corners that depend
// on no trigonometric function: the corners of a rectangle cut at its corners.
static Polygon blob(Lcg &g, coord_t cx, coord_t cy, coord_t rx, coord_t ry)
{
    coord_t  cut_x = g.range(rx / 8, rx / 3), cut_y = g.range(ry / 8, ry / 3);
    Polygon p;
    p.points = {Point(cx - rx + cut_x, cy - ry), Point(cx + rx - cut_x, cy - ry), Point(cx + rx, cy - ry + cut_y), Point(cx + rx, cy + ry - cut_y),
                Point(cx + rx - cut_x, cy + ry), Point(cx - rx + cut_x, cy + ry), Point(cx - rx, cy + ry - cut_y), Point(cx - rx, cy - ry + cut_y)};
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
static indexed_triangle_set moved(indexed_triangle_set its, float ox, float oy, float oz)
{
    for (Vec3f &v : its.vertices) v += Vec3f(ox, oy, oz);
    return its;
}

// ---------------------------------------------------------------------------------------------
// The scenarios
// ---------------------------------------------------------------------------------------------

static const int N_SCENARIOS = 26;

// The keys a scenario sets over the defaults of every option and the three filaments.
static std::vector<std::pair<std::string, std::string>> scenario_config(int scenario)
{
    std::vector<std::pair<std::string, std::string>> c;
    switch (scenario) {
    case 3: c = {{"enable_order_independent_overlap_carving", "1"}}; break;
    case 5: c = {{"xy_hole_compensation", "0.1"}, {"xy_contour_compensation", "-0.05"}, {"elefant_foot_compensation", "0.15"}}; break;
    case 6: c = {{"xy_hole_compensation", "-0.08"}, {"xy_contour_compensation", "0.12"}, {"elefant_foot_compensation", "0"}}; break;
    case 7: c = {{"xy_hole_compensation", "0.1"}, {"xy_contour_compensation", "0.07"}, {"elefant_foot_compensation", "0.2"}}; break;
    case 8: c = {{"xy_hole_compensation", "-0.05"}, {"xy_contour_compensation", "0.1"}, {"raft_layers", "2"}}; break;
    case 12: c = {{"filament_shrink", "98%,97%,100%"}}; break;
    case 15: c = {{"spiral_mode", "1"}}; break;
    case 16: c = {{"slicing_mode", "even_odd"}}; break;
    case 17: c = {{"slicing_mode", "close_holes"}}; break;
    case 18: c = {{"resolution", "0.0005"}, {"slice_closing_radius", "0.02"}, {"layer_height", "0.12"}}; break;
    case 20: c = {{"xy_hole_compensation", "-0.1"}, {"xy_contour_compensation", "-0.07"}, {"elefant_foot_compensation", "0.1"}}; break;
    case 22: c = {{"xy_hole_compensation", "0.1"}, {"xy_contour_compensation", "0.05"}, {"elefant_foot_compensation", "0"}}; break;
    case 23: c = {{"filament_shrink", "98%,97%,100%"}}; break;
    // One extruder with a rack of three nozzle diameters, and the diameter every filament prints with.
    case 24:
        c = {{"multi_nozzle_diameter_mode", "1"}, {"extruder_nozzle_count", "3"}, {"extruder_nozzle_diameter", "0.2,0.4,0.8"}, {"filament_diameter_map", "0.2;0.8;0.4"}};
        break;
    case 25:
        c = {{"multi_nozzle_diameter_mode", "1"}, {"extruder_nozzle_count", "3"}, {"extruder_nozzle_diameter", "0.2,0.4,0.8"}, {"filament_diameter_map", "0.4;0.2;0.8"}};
        break;
    default: break;
    }
    return c;
}

// The parts of the object of the scenarios 2, 3, 7, 8, 12 and 20: four parts of three filaments that
// overlap, two of them of one region, and a negative volume.
static void add_multi(ModelObject *o)
{
    o->add_volume(TriangleMesh(box_its(10, 10, 6, -5.f, -5.f, 0.f)));
    o->add_volume(TriangleMesh(box_its(10, 10, 6, 0.f, -2.f, 0.f)))->config.set("extruder", 2);
    o->add_volume(TriangleMesh(box_its(6, 6, 4, -8.f, -8.f, 0.f)));
    o->add_volume(TriangleMesh(box_its(4, 4, 6, 2.f, 2.f, 0.f)))->config.set("extruder", 3);
    o->add_volume(TriangleMesh(box_its(3, 3, 3, -1.f, -1.f, 1.5f)), ModelVolumeType::NEGATIVE_VOLUME);
}
// The plate with a round hole of the scenarios 5, 6, 15 and 17.
static void add_holed(ModelObject *o)
{
    o->add_volume(TriangleMesh(box_its(20, 20, 4, -10.f, -10.f, 0.f)));
    o->add_volume(TriangleMesh(moved(its_make_cylinder(3., 10., 2. * M_PI / 24.), 1.f, 0.5f, -1.f)), ModelVolumeType::NEGATIVE_VOLUME);
}

static Model build_model(int scenario)
{
    Model model;
    auto  object = [&model](const char *name) {
        ModelObject *o = model.add_object();
        o->name        = name;
        return o;
    };
    switch (scenario) {
    case 0: {
        ModelObject *o = object("cube");
        o->add_volume(TriangleMesh(box_its(20, 20, 6, -10.f, -10.f, 0.f)));
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        break;
    }
    case 1:
    case 14: {
        // A part with two modifiers (one of them outside of it) and a negative volume; two instances
        // that differ by a rotation, so two PrintObjects. Scenario 14 mirrors the instances.
        ModelObject *o = object("mods");
        o->config.set("extruder", 2);
        o->add_volume(TriangleMesh(box_its(10, 12, 6, -5.f, -6.f, 0.f)))->config.set("wall_filament", 3);
        ModelVolume *v1 = o->add_volume(TriangleMesh(box_its(4, 4, 4, 0.f, 0.f, 2.f)), ModelVolumeType::PARAMETER_MODIFIER);
        v1->config.set("sparse_infill_filament", 1);
        v1->set_offset(Vec3d(1., -1., 0.5));
        o->add_volume(TriangleMesh(box_its(3, 3, 3, 20.f, 20.f, 0.f)), ModelVolumeType::PARAMETER_MODIFIER)->config.set("wall_loops", 5);
        o->add_volume(TriangleMesh(box_its(2, 2, 10, -1.f, -1.f, 0.f)), ModelVolumeType::NEGATIVE_VOLUME);
        ModelInstance *i0 = o->add_instance();
        i0->set_offset(Vec3d(60., 60., 0.));
        ModelInstance *i1 = o->add_instance();
        i1->set_offset(Vec3d(80., 40., 0.));
        i1->set_rotation(Vec3d(0., 0., 0.5));
        if (scenario == 14) {
            i0->set_mirror(Vec3d(-1., 1., 1.));
            i1->set_mirror(Vec3d(1., -1., 1.));
        }
        break;
    }
    case 2:
    case 3:
    case 7:
    case 8:
    case 20: {
        ModelObject *o = object("multi");
        add_multi(o);
        o->add_instance()->set_offset(Vec3d(120., 80., 0.));
        break;
    }
    case 4:
    case 18: {
        // Sloped and round shapes, turned about Z.
        ModelObject *o = object("prism");
        o->add_volume(TriangleMesh(its_make_prism(20.f, 10.f, 8.f)));
        ModelInstance *i = o->add_instance();
        i->set_offset(Vec3d(70., 70., 0.));
        i->set_rotation(Vec3d(0., 0., 0.3));
        ModelObject *o2 = object("cylinder");
        o2->add_volume(TriangleMesh(its_make_cylinder(4., 5., 2. * M_PI / 20.)));
        o2->add_instance()->set_offset(Vec3d(110., 70., 0.));
        ModelObject *o3 = object("sphere");
        o3->add_volume(TriangleMesh(moved(its_make_sphere(5., 2. * M_PI / 16.), 0.f, 0.f, 5.f)));
        ModelInstance *i3 = o3->add_instance();
        i3->set_offset(Vec3d(70., 110., 0.));
        i3->set_rotation(Vec3d(0., 0., 1.1));
        ModelObject *o4 = object("pyramid");
        o4->add_volume(TriangleMesh(its_make_pyramid(12.f, 7.f)));
        o4->add_instance()->set_offset(Vec3d(110., 110., 0.));
        break;
    }
    case 5:
    case 6:
    case 15:
    case 17: {
        ModelObject *o = object("holed");
        add_holed(o);
        o->add_instance()->set_offset(Vec3d(90., 90., 0.));
        break;
    }
    case 9: {
        // A needle under a block: the layers of the needle are thinner than half an outer wall.
        ModelObject *o = object("needle");
        o->add_volume(TriangleMesh(box_its(0.1, 0.1, 0.6, -0.05f, -0.05f, 0.f)));
        o->add_volume(TriangleMesh(box_its(10, 10, 3, -5.f, -5.f, 0.6f)));
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        break;
    }
    case 10: {
        // A block that floats above the bed: empty layers at the bottom.
        ModelObject *o = object("floating");
        o->add_volume(TriangleMesh(box_its(10, 10, 3, -5.f, -5.f, 1.5f)));
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        break;
    }
    case 11: {
        ModelObject *o = object("enforced");
        o->add_volume(TriangleMesh(box_its(10, 10, 4, -5.f, -5.f, 0.f)));
        o->add_volume(TriangleMesh(box_its(4, 4, 2, -2.f, -2.f, 0.f)), ModelVolumeType::SUPPORT_ENFORCER);
        o->add_volume(TriangleMesh(box_its(4, 4, 2, 0.f, 0.f, 1.f)), ModelVolumeType::SUPPORT_ENFORCER);
        o->add_volume(TriangleMesh(box_its(3, 3, 3, 3.f, 3.f, 0.f)), ModelVolumeType::SUPPORT_BLOCKER);
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        break;
    }
    case 12: {
        ModelObject *o = object("shrink");
        add_multi(o);
        o->add_volume(TriangleMesh(box_its(4, 4, 2, -2.f, -2.f, 0.f)), ModelVolumeType::SUPPORT_ENFORCER);
        o->add_volume(TriangleMesh(box_its(4, 4, 2, 0.f, 0.f, 1.f)), ModelVolumeType::SUPPORT_ENFORCER);
        o->add_instance()->set_offset(Vec3d(120., 80., 0.));
        break;
    }
    case 13: {
        // Layer ranges with their own configuration, a second part and a modifier across them.
        ModelObject *o = object("ranges");
        o->add_volume(TriangleMesh(box_its(15, 15, 6, -7.5f, -7.5f, 0.f)));
        o->add_volume(TriangleMesh(box_its(6, 6, 5, 4.f, 4.f, 0.f)))->config.set("extruder", 2);
        o->add_volume(TriangleMesh(box_its(5, 5, 6, -6.f, -6.f, 0.f)), ModelVolumeType::PARAMETER_MODIFIER)->config.set("wall_loops", 4);
        o->layer_config_ranges[{1.0, 2.0}].set("extruder", 3);
        o->layer_config_ranges[{1.0, 2.0}].set("layer_height", 0.2);
        o->layer_config_ranges[{3.0, 4.0}].set("wall_filament", 2);
        o->layer_config_ranges[{3.0, 4.0}].set("layer_height", 0.2);
        o->add_instance()->set_offset(Vec3d(150., 50., 0.));
        break;
    }
    case 16: {
        // Two parts of one region that overlap, and a part inside a part.
        ModelObject *o = object("evenodd");
        o->add_volume(TriangleMesh(box_its(12, 12, 4, -6.f, -6.f, 0.f)));
        o->add_volume(TriangleMesh(box_its(12, 12, 4, 0.f, 0.f, 0.f)));
        o->add_volume(TriangleMesh(box_its(4, 4, 4, -4.f, -4.f, 0.f)));
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        break;
    }
    case 24:
    case 25: {
        // Two parts of the filaments 1 and 2, faces painted with the filaments 2 and 3, and the three
        // filaments on nozzles of three diameters.
        ModelObject *o = object("diameters");
        ModelVolume *v = o->add_volume(TriangleMesh(box_its(10, 10, 4, -5.f, -5.f, 0.f)));
        v->mmu_segmentation_facets.set_triangle_from_string(2, "8");
        v->mmu_segmentation_facets.set_triangle_from_string(3, "8");
        v->mmu_segmentation_facets.set_triangle_from_string(4, "0C");
        v->mmu_segmentation_facets.set_triangle_from_string(7, "8");
        v->mmu_segmentation_facets.set_triangle_from_string(10, "0C");
        ModelVolume *v2 = o->add_volume(TriangleMesh(box_its(6, 6, 3, 2.f, 2.f, 0.f)));
        v2->config.set("extruder", 2);
        v2->mmu_segmentation_facets.set_triangle_from_string(5, "0C");
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        break;
    }
    case 19: {
        // Faces painted with the filaments 2 and 3.
        ModelObject *o  = object("painted");
        ModelVolume *v = o->add_volume(TriangleMesh(box_its(10, 10, 4, -5.f, -5.f, 0.f)));
        v->mmu_segmentation_facets.set_triangle_from_string(2, "8");
        v->mmu_segmentation_facets.set_triangle_from_string(3, "8");
        v->mmu_segmentation_facets.set_triangle_from_string(4, "C");
        v->mmu_segmentation_facets.set_triangle_from_string(7, "8");
        v->mmu_segmentation_facets.set_triangle_from_string(10, "C");
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        break;
    }
    case 21: {
        // Faces painted with fuzzy skin.
        ModelObject *o = object("fuzzy");
        ModelVolume *v = o->add_volume(TriangleMesh(box_its(10, 10, 4, -5.f, -5.f, 0.f)));
        v->fuzzy_skin_facets.set_triangle_from_string(4, "4");
        v->fuzzy_skin_facets.set_triangle_from_string(5, "4");
        v->fuzzy_skin_facets.set_triangle_from_string(8, "4");
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        break;
    }
    case 22:
    case 23: {
        // Two parts of two filaments, painted with filaments and with fuzzy skin: with the XY size
        // compensation (22) and with a filament that shrinks (23), both of which painting switches off.
        ModelObject *o = object("painted2");
        ModelVolume *v = o->add_volume(TriangleMesh(box_its(10, 10, 4, -5.f, -5.f, 0.f)));
        v->mmu_segmentation_facets.set_triangle_from_string(2, "8");
        v->mmu_segmentation_facets.set_triangle_from_string(3, "8");
        v->mmu_segmentation_facets.set_triangle_from_string(6, "4");
        v->mmu_segmentation_facets.set_triangle_from_string(9, "C");
        v->fuzzy_skin_facets.set_triangle_from_string(4, "4");
        v->fuzzy_skin_facets.set_triangle_from_string(7, "4");
        ModelVolume *v2 = o->add_volume(TriangleMesh(box_its(6, 6, 3, 2.f, 2.f, 0.f)));
        v2->config.set("extruder", 2);
        v2->mmu_segmentation_facets.set_triangle_from_string(5, "4");
        v2->fuzzy_skin_facets.set_triangle_from_string(10, "4");
        o->add_instance()->set_offset(Vec3d(100., 100., 0.));
        break;
    }
    }
    return model;
}

// ---------------------------------------------------------------------------------------------
// Dumps
// ---------------------------------------------------------------------------------------------

// The index of the volume of the object that has the id.
static long long volume_index(const ModelObject &mo, const ObjectID id)
{
    for (size_t i = 0; i < mo.volumes.size(); ++i)
        if (mo.volumes[i]->id() == id) return (long long) i;
    return -1;
}

static void dump_volume_slices(const std::string &tag, const ModelObject &mo, const std::vector<VolumeSlices> &vss)
{
    out(tag + " " + hI((long long) vss.size()));
    for (const VolumeSlices &vs : vss) {
        std::string body;
        for (const ExPolygons &e : vs.slices) body += (body.empty() ? "" : " ") + EXPOLYS(e);
        outb(tag + " volume " + hI(volume_index(mo, vs.volume_id)) + " " + hI((long long) vs.slices.size()), body);
    }
}

static void dump_by_layer(const std::string &tag, const std::vector<ExPolygons> &layers)
{
    std::string body;
    for (const ExPolygons &e : layers) body += (body.empty() ? "" : " ") + EXPOLYS(e);
    outb(tag + " " + hI((long long) layers.size()), body);
}

static void dump_layers(const std::string &tag, const LayerPtrs &layers)
{
    out(tag + " layers " + hI((long long) layers.size()));
    for (size_t i = 0; i < layers.size(); ++i) {
        const Layer *l    = layers[i];
        std::string  body = "lslices " + EXPOLYS(l->lslices) + " bboxes " + hI((long long) l->lslices_bboxes.size());
        for (const BoundingBox &b : l->lslices_bboxes) body += " " + BBOX(b);
        body += " regions " + hI((long long) l->m_regions.size());
        for (const LayerRegion *r : l->m_regions) body += " slices " + SURFACES(r->slices.surfaces) + " raw " + EXPOLYS(r->raw_slices);
        outb(tag + " layer " + hI((long long) i) + " " + hI((long long) l->id()) + " " + hD(l->print_z) + " " + hD(l->height) + " " + hD(l->slice_z) + " " +
                 hB(l->slicing_errors) + " " + hI(l->lower_layer ? (long long) l->lower_layer->id() : -1) + " " +
                 hI(l->upper_layer ? (long long) l->upper_layer->id() : -1),
             body);
    }
}

static void dump_groups(const std::string &tag, const ModelObject &mo, const std::vector<groupedVolumeSlices> &groups)
{
    out(tag + " " + hI((long long) groups.size()));
    for (const groupedVolumeSlices &g : groups) {
        std::string head = tag + " group " + hI(g.groupId) + " " + hI((long long) g.volume_ids.size());
        for (const ObjectID &id : g.volume_ids) head += " " + hI(volume_index(mo, id));
        outb(head, EXPOLYS(g.slices));
    }
}

namespace Slic3r {
static void dump_segmentation(const std::string &what, const std::vector<std::vector<ExPolygons>> &segmentation)
{
    out("in " + g_hook_tag + " " + what + " " + hI((long long) segmentation.size()));
    for (const std::vector<ExPolygons> &layer : segmentation) {
        std::string line = "in " + g_hook_tag + " " + what + "_layer " + hI((long long) layer.size());
        for (const ExPolygons &e : layer) line += " " + EXPOLYS(e);
        out(line);
    }
}
static std::vector<std::vector<ExPolygons>> gen_mms_hook(const PrintObject &print_object, const std::function<void()> &throw_on_cancel_callback)
{
    std::vector<std::vector<ExPolygons>> segmentation = multi_material_segmentation_by_painting(print_object, throw_on_cancel_callback);
    dump_segmentation("mms", segmentation);
    return segmentation;
}
static std::vector<std::vector<ExPolygons>> gen_fuzzy_hook(const PrintObject &print_object, const std::function<void()> &throw_on_cancel_callback)
{
    std::vector<std::vector<ExPolygons>> segmentation = fuzzy_skin_segmentation_by_painting(print_object, throw_on_cancel_callback);
    dump_segmentation("fuzzy", segmentation);
    return segmentation;
}
} // namespace Slic3r

static void run_scenario(int scenario)
{
    const std::string S = "s" + hI(scenario);
    Model             model = build_model(scenario);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    // Three filaments. (`set_num_filaments` resizes nothing at the pin: the list of the filament keys is
    // never filled in, PrintConfig.cpp:7624.)
    config.option<ConfigOptionFloats>("filament_diameter", true)->values = {1.75, 1.75, 1.75};
    config.option<ConfigOptionStrings>("filament_colour", true)->values  = {"#FF0000", "#00FF00", "#0000FF"};
    config.option<ConfigOptionInts>("filament_map", true)->values        = {1, 1, 1};
    for (const auto &kv : scenario_config(scenario)) config.set_deserialize_strict(kv.first, kv.second);
    Print print;
    print.set_status_callback([](const PrintBase::SlicingStatus &) {});
    print.apply(model, config);
    out(S + " objects " + hI((long long) print.m_objects.size()) + " " + hB(print.uses_multiple_nozzle_diameters()));
    const std::function<void()> no_cancel = []() {};

    for (size_t obj_idx = 0; obj_idx < print.m_objects.size(); ++obj_idx) {
        PrintObject        *obj = print.m_objects[obj_idx];
        const ModelObject  &mo  = *obj->model_object();
        const std::string   T   = S + " o" + hI((long long) obj_idx);
        g_hook_tag              = T;

        // new_layers over the layers `PrintObject::slice()` makes.
        std::vector<coordf_t> layer_height_profile;
        bool                  nozzle_range_reset = false;
        obj->update_layer_height_profile(mo, obj->m_slicing_params, layer_height_profile, nozzle_range_reset);
        LayerPtrs layers = new_layers(obj, generate_object_layers(obj->m_slicing_params, layer_height_profile, obj->m_config.precise_z_height.value));
        {
            std::string body;
            for (const Layer *l : layers)
                body += " " + hI((long long) l->id()) + " " + hD(l->print_z) + " " + hD(l->height) + " " + hD(l->slice_z) + " " +
                        hI(l->lower_layer ? (long long) l->lower_layer->id() : -1) + " " + hI(l->upper_layer ? (long long) l->upper_layer->id() : -1);
            outb(T + " new_layers " + hI((long long) layers.size()), body);
        }
        std::vector<float> zs = zs_from_layers(layers);
        for (Layer *l : layers) delete l;
        layers.clear();

        // slice_volumes_inner, with and without the print.
        std::vector<VolumeSlices> by_volume =
            slice_volumes_inner(print.config(), obj->config(), obj->trafo_centered(), mo.volumes, obj->m_shared_regions->layer_ranges, zs, no_cancel, &print);
        dump_volume_slices(T + " slice_volumes_inner", mo, by_volume);

        // slice_volume of every volume: all of the layers, every third layer, and layer ranges.
        {
            MeshSlicingParamsEx params;
            params.trafo          = obj->trafo_centered();
            params.closing_radius = 0.049f;
            std::vector<float> zs3;
            for (size_t i = 0; i < zs.size(); i += 3) zs3.emplace_back(zs[i]);
            const double                      top = zs.empty() ? 0. : double(zs.back());
            std::vector<t_layer_height_range> ranges1{{0., top + 1.}};
            std::vector<t_layer_height_range> ranges2{{0.3, 0.9}, {1.1, 1.1 + 0.25 * top}, {0.75 * top, top}};
            std::vector<t_layer_height_range> ranges3{{top + 5., top + 6.}};
            for (size_t iv = 0; iv < mo.volumes.size(); ++iv) {
                const ModelVolume &mv = *mo.volumes[iv];
                const std::string  V  = T + " slice_volume " + hI((long long) iv);
                out(V + " needs_slicing " + hB(model_volume_needs_slicing(mv)));
                dump_by_layer(V + " all", slice_volume(mv, zs, params, no_cancel));
                dump_by_layer(V + " third", slice_volume(mv, zs3, params, no_cancel));
                dump_by_layer(V + " ranges1", slice_volume(mv, zs, ranges1, params, no_cancel));
                dump_by_layer(V + " ranges2", slice_volume(mv, zs, ranges2, params, no_cancel));
                dump_by_layer(V + " ranges3", slice_volume(mv, zs, ranges3, params, no_cancel));
            }
        }

        // slices_to_regions: the four combinations of its two flags.
        for (int flags = 0; flags < 4; ++flags) {
            std::vector<std::vector<ExPolygons>> by_region =
                slices_to_regions(mo.volumes, *obj->m_shared_regions, zs, std::vector<VolumeSlices>(by_volume), (flags & 1) != 0, (flags & 2) != 0, no_cancel);
            out(T + " slices_to_regions " + hI(flags) + " " + hI((long long) by_region.size()));
            for (size_t r = 0; r < by_region.size(); ++r) dump_by_layer(T + " slices_to_regions " + hI(flags) + " region " + hI((long long) r), by_region[r]);
        }

        // The step.
        std::string error;
        try {
            obj->slice();
        } catch (const std::exception &e) {
            error = e.what();
        }
        out(T + " slice " + hB(error.empty()) + " " + hB(obj->is_step_done(posSlice)));
        dump_layers(T, obj->m_layers);
        dump_volume_slices(T + " first_layer_by_volume", mo, obj->firstLayerObjSliceByVolume);
        dump_groups(T + " first_layer_groups", mo, obj->firstLayerObjSliceByGroups);

        // What reads the sliced object.
        out(T + " shrink " + hD(obj->support_shrinkage_scale()) + " " + hB(should_skip_filament_shrink(*obj)));
        for (ModelVolumeType type : {ModelVolumeType::SUPPORT_ENFORCER, ModelVolumeType::SUPPORT_BLOCKER}) {
            std::vector<Polygons> slices = obj->slice_support_volumes(type);
            std::string           body;
            for (const Polygons &p : slices) body += (body.empty() ? "" : " ") + POLYS(p);
            outb(T + " slice_support_volumes " + hI((int) type) + " " + hI((long long) slices.size()), body);
        }

        // normalize_mm_segmentation_to_layer_bands on the sliced object of a print of several nozzle
        // diameters, over synthetic segmentations: strokes of a filament held over a run of layers.
        if (scenario == 24 || scenario == 25) {
            Lcg          g(1000 + uint64_t(scenario));
            const size_t nl = obj->layer_count();
            for (int c = 0; c < 8; ++c) {
                std::vector<std::vector<ExPolygons>> seg(nl, std::vector<ExPolygons>(3));
                const int                            strokes = int(g.range(3, 10));
                for (int k = 0; k < strokes; ++k) {
                    const int     e  = int(g.range(c == 0 ? 1 : 0, 2));
                    const coord_t cx = g.range(-3, 5) * 100000, cy = g.range(-3, 5) * 100000;
                    const coord_t rx = g.range(60000, 200000), ry = g.range(60000, 200000);
                    const Polygon p  = blob(g, cx, cy, rx, ry);
                    const int     l0 = int(g.range(0, (long long) nl - 1));
                    const int     l1 = std::min(int(nl) - 1, l0 + int(g.range(0, 11)));
                    for (int l = l0; l <= l1; ++l) {
                        // The filaments of a layer do not overlap: a stroke takes its area from the others.
                        for (int o = 0; o < 3; ++o)
                            if (o != e) seg[l][o] = diff_ex(seg[l][o], ExPolygons{ExPolygon(p)});
                        Polygons all = to_polygons(seg[l][e]);
                        all.emplace_back(p);
                        seg[l][e] = union_ex(all);
                    }
                }
                dump_segmentation("normalize", seg);
                normalize_mm_segmentation_to_layer_bands(*obj, seg, no_cancel);
                for (size_t l = 0; l < seg.size(); ++l)
                    outb(T + " normalized " + hI(c) + " layer " + hI((long long) l), EXPOLYS(seg[l][0]) + " " + EXPOLYS(seg[l][1]) + " " + EXPOLYS(seg[l][2]));
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// The helpers on synthetic inputs
// ---------------------------------------------------------------------------------------------

static std::string F3(const Vec3f &v) { return hF(v.x()) + " " + hF(v.y()) + " " + hF(v.z()); }

// Layers of blobs on a grid of 2 mm: neighbours touch, overlap or stay 0.1 mm and more apart.
static std::vector<ExPolygons> blob_layers(Lcg &g, int nl)
{
    std::vector<ExPolygons> layers(nl);
    for (ExPolygons &layer : layers) {
        const int k = int(g.range(0, 2));
        for (int b = 0; b < k; ++b) {
            const coord_t cx = g.range(0, 5) * 200000, cy = g.range(0, 3) * 200000;
            const coord_t rx = g.range(85000, 110000), ry = g.range(85000, 110000);
            const Polygon p  = blob(g, cx, cy, rx, ry);
            if (g.range(0, 3) == 0)
                layer.emplace_back(with_hole(p, blob(g, cx, cy, rx / 3, ry / 3)));
            else
                layer.emplace_back(p);
        }
    }
    return layers;
}

static void run_synthetic()
{
    Lcg g(20261119);
    const std::function<void()> no_cancel = []() {};

    // overlap_in_xy
    for (int i = 0; i < 80; ++i) {
        auto box = [&g]() {
            const Vec3f a(float(g.range(-4, 4)), float(g.range(-4, 4)), float(g.range(-4, 4)));
            const Vec3f sz(float(g.range(0, 3)) * 0.5f, float(g.range(0, 3)) * 0.5f, float(g.range(0, 3)) * 0.5f);
            return PrintObjectRegions::BoundingBox(a, a + sz);
        };
        const PrintObjectRegions::BoundingBox l = box(), r = box();
        out("in overlap_in_xy " + F3(l.min()) + " " + F3(l.max()) + " " + F3(r.min()) + " " + F3(r.max()));
        out("overlap_in_xy " + hB(overlap_in_xy(l, r)));
    }

    // layer_range_first, layer_range_next: contiguous ranges, and the heights the assertions of the two
    // functions admit (a height closer than EPSILON to a boundary without being on it is not one).
    for (int c = 0; c < 8; ++c) {
        std::vector<PrintObjectRegions::LayerRangeRegions> ranges;
        const int   n    = int(g.range(1, 5));
        double      z    = 0.;
        std::string line = "in layer_ranges " + hI(n);
        for (int i = 0; i < n; ++i) {
            PrintObjectRegions::LayerRangeRegions lr;
            lr.layer_height_range = {z, z + 0.25 * double(g.range(1, 8))};
            z                     = lr.layer_height_range.second;
            line += " " + hD(lr.layer_height_range.first) + " " + hD(lr.layer_height_range.second);
            ranges.emplace_back(std::move(lr));
        }
        out(line);
        for (int i = 0; i < n; ++i) {
            const double lo = ranges[i].layer_height_range.first, hi = ranges[i].layer_height_range.second;
            std::vector<double> first{lo, 0.5 * (lo + hi), hi, hi - 5e-5, lo + 2e-4, lo + 0.1, hi - 0.1};
            for (double zz : first) {
                out("in layer_range_first " + hD(zz));
                out("layer_range_first " + hI((long long) (layer_range_first(ranges, zz) - ranges.begin())));
            }
            std::vector<double> next{lo, 0.5 * (lo + hi), lo + 2e-4, hi - 2e-4, lo + 0.1};
            for (double zz : next)
                for (int from = 0; from <= i; ++from) {
                    out("in layer_range_next " + hI(from) + " " + hD(zz));
                    out("layer_range_next " + hI((long long) (layer_range_next(ranges, ranges.begin() + from, zz) - ranges.begin())));
                }
        }
    }

    // groupingVolumes
    auto dump_synthetic_groups = [](const std::string &tag, const std::vector<groupedVolumeSlices> &groups) {
        out(tag + " " + hI((long long) groups.size()));
        for (const groupedVolumeSlices &gr : groups) {
            std::string head = tag + " group " + hI(gr.groupId) + " " + hI((long long) gr.volume_ids.size());
            for (const ObjectID &id : gr.volume_ids) head += " " + hI((long long) id.id);
            outb(head, EXPOLYS(gr.slices));
        }
    };
    for (int c = 0; c < 16; ++c) {
        const int                 n = int(g.range(2, 8)), nl = int(g.range(1, 5));
        std::vector<VolumeSlices> vss(n);
        const double              resolution = (c % 3 == 0) ? 0. : (c % 3 == 1 ? 1250. : 500.);
        const int                 replaced   = c % 2;
        out("in grouping " + hI(n) + " " + hD(resolution) + " " + hI(replaced));
        for (int i = 0; i < n; ++i) {
            vss[i].volume_id = ObjectID(size_t(100 + 3 * i));
            // A volume with another number of layers than the others connects to none of them.
            vss[i].slices = blob_layers(g, (c == 5 && i == 1) ? nl + 1 : nl);
            std::string line = "in grouping_volume " + hI((long long) vss[i].volume_id.id) + " " + hI((long long) vss[i].slices.size());
            for (const ExPolygons &e : vss[i].slices) line += " " + EXPOLYS(e);
            out(line);
        }
        std::vector<groupedVolumeSlices> groups;
        const bool                       ok = groupingVolumes(vss, groups, resolution, replaced);
        out("grouping ok " + hB(ok));
        dump_synthetic_groups("grouping", groups);
    }

    // findPartVolumes, groupingVolumes, applyNegtiveVolumes and reGroupingLayerPolygons, as
    // groupingVolumesForBrim chains them, over the volumes of an object.
    {
        Model        model;
        ModelObject *o = model.add_object();
        const std::vector<ModelVolumeType> types{ModelVolumeType::MODEL_PART,      ModelVolumeType::NEGATIVE_VOLUME, ModelVolumeType::MODEL_PART,
                                                 ModelVolumeType::PARAMETER_MODIFIER, ModelVolumeType::MODEL_PART,      ModelVolumeType::NEGATIVE_VOLUME,
                                                 ModelVolumeType::SUPPORT_ENFORCER,   ModelVolumeType::MODEL_PART};
        for (ModelVolumeType type : types) o->add_volume(TriangleMesh(box_its(1, 1, 1, 0.f, 0.f, 0.f)), type);
        for (int c = 0; c < 12; ++c) {
            const int                 nl         = int(g.range(1, 4));
            const double              resolution = (c % 3 == 0) ? 0. : (c % 3 == 1 ? 1250. : 500.);
            const int                 replaced   = (c % 4 == 3) ? 1 : 0;
            std::vector<VolumeSlices> vss;
            for (size_t i = 0; i < types.size(); ++i) {
                if (g.range(0, 6) == 0) continue;
                VolumeSlices vs;
                vs.volume_id = o->volumes[i]->id();
                vs.slices    = blob_layers(g, nl);
                vss.emplace_back(std::move(vs));
            }
            out("in brim_groups " + hI((long long) vss.size()) + " " + hD(resolution) + " " + hI(replaced));
            for (const VolumeSlices &vs : vss) {
                std::string line = "in brim_groups_volume " + hI(volume_index(*o, vs.volume_id)) + " " + hI((long long) vs.slices.size());
                for (const ExPolygons &e : vs.slices) line += " " + EXPOLYS(e);
                out(line);
            }
            auto dump = [o](const std::string &tag, const std::vector<groupedVolumeSlices> &groups) { dump_groups(tag, *o, groups); };
            const std::vector<VolumeSlices> parts = findPartVolumes(vss, o->volumes);
            {
                std::string line = "brim_groups parts " + hI((long long) parts.size());
                for (const VolumeSlices &vs : parts) line += " " + hI(volume_index(*o, vs.volume_id));
                out(line);
            }
            std::vector<groupedVolumeSlices> groups;
            groupingVolumes(parts, groups, resolution, replaced);
            dump("brim_groups grouped", groups);
            applyNegtiveVolumes(o->volumes, vss, groups, resolution);
            dump("brim_groups negative", groups);
            // The islands of a layer: the parts of the layer the groups are of, united.
            Polygons all;
            for (const VolumeSlices &vs : parts)
                if (! vs.slices.empty()) polygons_append(all, to_polygons(vs.slices[std::min(size_t(replaced), vs.slices.size() - 1)]));
            ExPolygons eps = union_ex(all);
            out("in brim_groups_eps " + EXPOLYS(eps));
            reGroupingLayerPolygons(groups, eps, resolution);
            dump("brim_groups regrouped", groups);
        }
    }

    // PrintObject::_shrink_contour_holes, on the object of the first scenario.
    {
        Model              model  = build_model(0);
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        Print              print;
        print.set_status_callback([](const PrintBase::SlicingStatus &) {});
        print.apply(model, config);
        const PrintObject *obj = print.m_objects.front();
        for (int c = 0; c < 40; ++c) {
            ExPolygons    polys;
            const int     n = int(g.range(1, 3));
            for (int i = 0; i < n; ++i) {
                const coord_t cx = coord_t(i) * 320000 + g.range(-10000, 10000), cy = g.range(-10000, 10000);
                const coord_t rx = g.range(100000, 150000), ry = g.range(100000, 150000);
                const Polygon p  = blob(g, cx, cy, rx, ry);
                if (g.range(0, 2) != 0)
                    polys.emplace_back(with_hole(p, blob(g, cx + g.range(-20000, 20000), cy + g.range(-20000, 20000), rx / 3, ry / 3)));
                else
                    polys.emplace_back(p);
            }
            // The deltas are floats in the callers.
            const double contour_delta = double(float(g.range(-4, 4) * 2500)), hole_delta = double(float(g.range(-4, 4) * 2500));
            out("in shrink_contour_holes " + hD(contour_delta) + " " + hD(hole_delta) + " " + EXPOLYS(polys));
            outb("shrink_contour_holes", EXPOLYS(obj->_shrink_contour_holes(contour_delta, hole_delta, polys)));
        }
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: gen <vectors.txt> [full|hashed [scenario ...]]\n");
        return 1;
    }
    g_out  = std::fopen(argv[1], "w");
    g_full = argc > 2 && std::string(argv[2]) == "full";
    if (g_out == nullptr) return 1;
    tbb::global_control one_thread(tbb::global_control::max_allowed_parallelism, 1);
    out(std::string("in mode ") + (g_full ? "full" : "hashed"));
    // A diagnostic: the scenarios named after the mode, alone (tests/print_object_slice_native_parity.rs
    // reads such a file with PRINT_OBJECT_SLICE_PARITY_ONLY).
    if (argc > 3) {
        for (int i = 3; i < argc; ++i) run_scenario(std::atoi(argv[i]));
    } else {
        for (int scenario = 0; scenario < N_SCENARIOS; ++scenario) run_scenario(scenario);
        run_synthetic();
    }
    std::fclose(g_out);
    return 0;
}
