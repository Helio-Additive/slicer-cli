// Generator of the vectors of tests/support_common_native_parity.rs: libslic3r's
// Support/SupportCommon.cpp and SupportCommon.hpp at the pin (references/BambuStudio/src/libslic3r; the
// override layer has no copy of them).
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
// SupportCommon.cpp is compiled into this file (several of its functions are static), with the private
// members of the classes made public. tbb is limited to one thread.
//
// The fillers: SupportCommon.cpp is compiled with `Fill` standing for `RecFill`, a filler that writes down
// what it is asked ("fill_surface" lines: its pattern, the members the support code sets, the parameters
// and the surface) and answers with lines across the surface that it clips with the surface. The Rust test
// has the same filler, so that everything around the fillers is compared, not the fillers of the engine.
//
// The first part runs the helpers on polygons: safe_union, safe_offset_inc, draw_perimeters,
// keep_polylines_near_bbox. The second builds, per scenario, a Print of one box with `Print::apply`, a
// SupportParameters of its object and a storage of support generator layers (a column of intermediate
// layers, top and bottom contacts), and runs the steps PrintObjectSupportMaterial::generate() runs after
// the layers exist: generate_interface_layers, generate_raft_base, generate_support_layers,
// generate_support_toolpaths. A support generator layer is named by its index in the storage. The first
// scenario also runs the path helpers one by one: tree_supports_generate_paths,
// fill_expolygons_with_sheath_generate_paths, LoopInterfaceProcessor::generate,
// SupportGeneratorLayerExtruded::merge and modulate_extrusion_by_overlapping_layers.
//
// Lines that start with "in" are the inputs the Rust test reads back; the others are results it has to
// reproduce. Tokens are separated by one space; a double is its 16 hexadecimal digits, a float its 8,
// an integer decimal. The vectors were made on macOS arm64 with Apple clang and Eigen 5.0.1.

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
#include <tbb/scalable_allocator.h>
#include <tbb/spin_mutex.h>

#include <boost/container/static_vector.hpp>

#define private public
#define protected public
// What SupportCommon.cpp includes, ahead of it: the headers are read with `Fill` meaning the class of
// FillBase.hpp.
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Fill/FillBase.hpp"
#include "libslic3r/MutablePolygon.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Point.hpp"
#include "clipper/clipper_z.hpp"
#include "libslic3r/Support/SupportCommon.hpp"
#include "libslic3r/Support/SupportLayer.hpp"
#include "libslic3r/Support/SupportParameters.hpp"
#include "libslic3r/Log.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"

namespace Slic3r {
// The filler of the generator: see the head of this file.
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
#include "libslic3r/Support/SupportCommon.cpp"
#undef Fill
#undef protected
#undef private

namespace Slic3r {
// The function SupportCommon.hpp declares, for the objects of the library that call it: not reached here.
void fill_expolygons_with_sheath_generate_paths(ExtrusionEntitiesPtr &, const Polygons &, Fill *, float, ExtrusionRole, const Flow &, const SupportParameters &, bool, bool)
{
    std::abort();
}
} // namespace Slic3r

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
static std::string hI(long long v) { return std::to_string(v); }
static std::string hB(bool v) { return v ? "1" : "0"; }
static void out(const std::string &s) { std::fprintf(g_out, "%s\n", s.c_str()); }
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

static std::string hF(float v)
{
    uint32_t u;
    std::memcpy(&u, &v, 4);
    char b[32];
    std::snprintf(b, sizeof(b), "%08x", (unsigned) u);
    return b;
}
static std::string FLOW(const Flow &f) { return hF(f.width()) + " " + hF(f.height()) + " " + hF(f.nozzle_diameter()); }
static std::string PATH(const ExtrusionPath &p)
{
    return "P " + hI(int(p.role())) + " " + hD(p.mm3_per_mm) + " " + hF(p.width) + " " + hF(p.height) + " " + hB(p.can_reverse()) + " " + hD(p.overhang_degree) + " " +
           PTS(p.polyline.points);
}
static std::string ENT(const ExtrusionEntity *e)
{
    if (const auto *c = dynamic_cast<const ExtrusionEntityCollection *>(e)) {
        std::string o = "C " + hB(c->no_sort) + " " + hI((long long) c->entities.size());
        for (const ExtrusionEntity *ee : c->entities) o += " " + ENT(ee);
        return o;
    }
    if (const auto *m = dynamic_cast<const ExtrusionMultiPath *>(e)) {
        std::string o = "M " + hB(m->can_reverse()) + " " + hI((long long) m->paths.size());
        for (const ExtrusionPath &p : m->paths) o += " " + PATH(p);
        return o;
    }
    if (const auto *p = dynamic_cast<const ExtrusionPath *>(e)) return PATH(*p);
    return "X";
}
static std::string ENTS(const ExtrusionEntitiesPtr &v)
{
    std::string o = hI((long long) v.size());
    for (const ExtrusionEntity *e : v) o += " " + ENT(e);
    return o;
}

// The filler of the generator. Its lines: at every `step` of the height of the surface, slanted unless the
// pattern is the rectilinear one, clipped with the surface. The concentric one fails on an odd number of lines.
Polylines Slic3r::RecFill::fill_surface(const Surface *surface, const FillParams &params)
{
    out("fill_surface " + hI(int(this->pattern)) + " " + hF(this->angle) + " " + hD(this->spacing) + " " + hI(this->link_max_length) + " " + PT(this->bounding_box.min) + " " +
        PT(this->bounding_box.max) + " " + hI((long long) this->layer_id) + " " + hD(this->z) + " " + hD(this->overlap) + " " + hF(params.density) + " " +
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
// A regular polygon; the vertices are computed from a table of integers, not from sin and cos, so that
// the inputs do not depend on the math library.
static Polygon ngon(double cx, double cy, double r, int n)
{
    Polygon p;
    for (int i = 0; i < n; ++i) {
        // A point of the unit circle by a rational parametrization: t = tan(a / 2) is not needed, the
        // vertices only have to lie around the center.
        double a  = 2. * 3.14159265358979323846 * i / n;
        double c = 1. - a * a / 2. + a * a * a * a / 24. - a * a * a * a * a * a / 720. + a * a * a * a * a * a * a * a / 40320.;
        double s = a - a * a * a / 6. + a * a * a * a * a / 120. - a * a * a * a * a * a * a / 5040. + a * a * a * a * a * a * a * a * a / 362880.;
        if (a > 3.14159265358979323846) {
            double b = a - 2. * 3.14159265358979323846;
            c = 1. - b * b / 2. + b * b * b * b / 24. - b * b * b * b * b * b / 720. + b * b * b * b * b * b * b * b / 40320.;
            s = b - b * b * b / 6. + b * b * b * b * b / 120. - b * b * b * b * b * b * b / 5040. + b * b * b * b * b * b * b * b * b / 362880.;
        }
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

// ---------------------------------------------------------------------------------------------------------
// The helpers.
// ---------------------------------------------------------------------------------------------------------

static void run_helpers()
{
    std::vector<Polygons> sets;
    sets.push_back({rect(0, 0, 10, 6)});
    sets.push_back({rect(0, 0, 10, 6), rect(8, 2, 16, 9)});
    sets.push_back({ngon(3, 4, 5.5, 24), rect(20, -3, 26, 3)});
    sets.push_back({ngon(0, 0, 9, 40), ngon(0, 0, 4, 12)});
    {
        // The sliver of the comment of safe_union(): its union is empty after a few rounds.
        Polygon sliver;
        sliver.points = {Point(120410, 83599), Point(120384, 83643), Point(120399, 83618), Point(120414, 83591), Point(120423, 83570), Point(120419, 83580)};
        sets.push_back({sliver});
        // A degenerate one: a line there and back.
        Polygon line;
        line.points = {Point(0, 0), Point(250000, 130000), Point(500000, 260000)};
        sets.push_back({line});
    }
    sets.push_back({});
    {
        Polygon star;
        for (int i = 0; i < 14; ++i) {
            const Polygon c = ngon(3.3, -7.1, (i % 2) ? 9.37 : 21.9, 14);
            star.points.emplace_back(c.points[i]);
        }
        sets.push_back({star});
    }
    for (size_t i = 0; i < sets.size(); ++i) out("in set " + POLYS(sets[i]));
    for (size_t i = 0; i < sets.size(); ++i)
        for (size_t j = 0; j < sets.size(); ++j) {
            if (j != i && j != (i + 1) % sets.size() && j != 6) continue;
            out("safe_union " + hI((long long) i) + " " + hI((long long) j) + " " + POLYS(safe_union(sets[i], sets[j])));
            ExPolygons a = union_ex(sets[i]), b = union_ex(sets[j]);
            // The ExPolygons overload on what union_ex makes of the sets, and on the raw contours.
            out("safe_union_ex " + hI((long long) i) + " " + hI((long long) j) + " " + EXPOLYS(safe_union(a, b)));
            ExPolygons ra, rb;
            for (const Polygon &p : sets[i]) ra.emplace_back(ExPolygon(p));
            for (const Polygon &p : sets[j]) rb.emplace_back(ExPolygon(p));
            out("safe_union_raw " + hI((long long) i) + " " + hI((long long) j) + " " + EXPOLYS(safe_union(ra, rb)));
        }
    std::vector<Polygons> collisions = {{}, {rect(11, -2, 13, 12)}, {rect(-4, 5.2, 30, 8), ngon(-3, -2, 2.5, 9)}};
    for (size_t c = 0; c < collisions.size(); ++c) out("in collision " + POLYS(collisions[c]));
    struct Args { coord_t distance, safe_step_size, last_step; size_t min_amount; };
    const std::vector<Args> args = {{0, 20000, 0, 0},      {0, 20000, 30000, 1},  {30000, 20000, 0, 1},   {30000, 20000, 50000, 1},   {250000, 20000, 10000, 1},
                                    {250000, 60000, 0, 3}, {70000, 40000, 20000, 12}, {90000, 100000, 100000, 5}, {45000, -1, 0, 1}, {123457, 31000, 7000, 2},
                                    {400000, 21000, 15000, 30}};
    for (size_t i = 0; i < sets.size(); ++i)
        for (size_t c = 0; c < collisions.size(); ++c)
            for (size_t a = 0; a < args.size(); ++a) {
                if ((i + c + a) % 3 != 0 && a > 1) continue;
                const Args &g = args[a];
                const std::string id = hI((long long) i) + " " + hI((long long) c) + " " + hI(g.distance) + " " + hI(g.safe_step_size) + " " + hI(g.last_step) + " " + hI((long long) g.min_amount);
                out("safe_offset_inc " + id + " " + POLYS(safe_offset_inc(sets[i], g.distance, collisions[c], g.safe_step_size, g.last_step, g.min_amount)));
                out("safe_offset_inc_ex " + id + " " + EXPOLYS(safe_offset_inc(union_ex(sets[i]), g.distance, collisions[c], g.safe_step_size, g.last_step, g.min_amount)));
            }

    // draw_perimeters
    std::vector<ExPolygon> expolys = {ExPolygon(rect(0, 0, 30, 20)), with_hole(ngon(0, 0, 14, 48), ngon(0, 0, 7.2, 24)), with_hole(rect(-10, -8, 22, 9), rect(-4, -3, 5, 2)),
                                      ExPolygon(ngon(3, 4, 6.5, 9))};
    for (size_t i = 0; i < expolys.size(); ++i) {
        out("in expoly " + EXPOLY(expolys[i]));
        for (double clip : {0., 6300., 123456.7, 2500000., 1e9})
            out("draw_perimeters " + hI((long long) i) + " " + hD(clip) + " " + PLS(draw_perimeters(expolys[i], clip)));
    }

    // keep_polylines_near_bbox
    Polylines lines;
    for (const ExPolygon &e : expolys)
        for (const Polyline &pl : draw_perimeters(e, 100.)) lines.emplace_back(pl);
    lines.emplace_back(Polyline());
    lines.emplace_back(Polyline(Points{Point(5, 5)}));
    {
        Polyline zigzag;
        for (int i = 0; i < 40; ++i) zigzag.points.emplace_back(Point(scale_(-20. + i * 1.3), scale_((i % 2) ? 3. : -3. + i * 0.2)));
        lines.emplace_back(zigzag);
    }
    out("in lines " + PLS(lines));
    std::vector<BoundingBox> boxes = {BoundingBox(Point(scale_(-2.), scale_(-2.)), Point(scale_(4.), scale_(5.))), BoundingBox(Point(scale_(100.), scale_(100.)), Point(scale_(110.), scale_(110.))),
                                      BoundingBox(Point(scale_(-50.), scale_(-50.)), Point(scale_(50.), scale_(50.))), BoundingBox(Point(scale_(10.), scale_(-1.)), Point(scale_(10.5), scale_(0.))),
                                      BoundingBox(Point(scale_(29.), scale_(19.)), Point(scale_(31.), scale_(21.)))};
    for (const BoundingBox &bb : boxes) {
        Polylines l = lines;
        keep_polylines_near_bbox(l, bb);
        out("keep_polylines_near_bbox " + PT(bb.min) + " " + PT(bb.max) + " " + PLS(l));
    }
}

// ---------------------------------------------------------------------------------------------------------
// The layers.
// ---------------------------------------------------------------------------------------------------------

static std::map<const SupportGeneratorLayer *, int> g_index;
static void reindex(SupportGeneratorLayerStorage &storage)
{
    g_index.clear();
    int i = 0;
    for (const SupportGeneratorLayer &l : storage.m_storage) g_index[&l] = i++;
}
static std::string OPT(const std::unique_ptr<Polygons> &p) { return p ? "1 " + POLYS(*p) : std::string("0"); }
static std::string LAYER(const SupportGeneratorLayer &l)
{
    return hI((int) l.layer_type) + " " + hD(l.print_z) + " " + hD(l.bottom_z) + " " + hD(l.height) + " " + hI((long long) l.idx_object_layer_above) + " " +
           hI((long long) l.idx_object_layer_below) + " " + hB(l.bridging) + " " + hB(l.up) + " " + POLYS(l.polygons) + " " + OPT(l.contact_polygons) + " " +
           OPT(l.overhang_polygons) + " " + OPT(l.enforcer_polygons);
}
static std::string LIST(const SupportGeneratorLayersPtr &v)
{
    std::string o = hI((long long) v.size());
    for (const SupportGeneratorLayer *l : v) o += " " + hI(l == nullptr ? -1 : g_index.at(l));
    return o;
}
static void dump_storage(const std::string &tag, SupportGeneratorLayerStorage &storage)
{
    reindex(storage);
    out(tag + " storage " + hI((long long) storage.m_storage.size()));
    for (const SupportGeneratorLayer &l : storage.m_storage) out(tag + " L " + LAYER(l));
}

// ---------------------------------------------------------------------------------------------------------
// The path helpers (first scenario).
// ---------------------------------------------------------------------------------------------------------

static void clear(ExtrusionEntitiesPtr &v)
{
    for (ExtrusionEntity *e : v) delete e;
    v.clear();
}
static Polygon reversed(Polygon p)
{
    p.reverse();
    return p;
}

static void run_path_helpers(const SupportParameters &p0)
{
    std::vector<Polygons> shapes;
    shapes.push_back({rect(0, 0, 12, 9)});
    shapes.push_back({rect(0, 0, 30, 22), reversed(rect(8, 6, 20, 15))});
    shapes.push_back({ngon(0, 0, 9, 40), reversed(ngon(0, 0, 4, 12))});
    shapes.push_back({ngon(3, 4, 2.2, 16)});
    shapes.push_back({rect(0, 0, 1.1, 14)});
    shapes.push_back({rect(0, 0, 10, 6), rect(8, 2, 16, 9), ngon(30, 0, 5, 20)});
    shapes.push_back({rect(0, 0, 40, 40), reversed(rect(3, 3, 37, 37))});
    shapes.push_back({});
    for (const Polygons &sh : shapes) out("in shape " + POLYS(sh));
    const std::vector<Flow> flows = {Flow(0.42f, 0.2f, 0.4f), Flow(0.45f, 0.28f, 0.4f), Flow(0.82f, 0.4f, 0.8f)};
    for (const Flow &f : flows) out("in flow " + FLOW(f));

    // tree_supports_generate_paths
    const double areas[4] = {p0.tree_branch_diameter_double_wall_area_scaled, 0., 0.1, 3e12};
    for (size_t i = 0; i < shapes.size(); ++i)
        for (size_t f = 0; f < flows.size(); ++f)
            for (size_t a = 0; a < 4; ++a) {
                if ((i + f + a) % 2 == 1) continue;
                SupportParameters p                            = p0;
                p.tree_branch_diameter_double_wall_area_scaled = areas[a];
                p.multi_nozzle_diameter_mode                   = (i + a) % 3 == 0;
                ExtrusionEntitiesPtr dst;
                tree_supports_generate_paths(dst, shapes[i], flows[f], p);
                out("tsgp " + hI((long long) i) + " " + hI((long long) f) + " " + hD(areas[a]) + " " + hB(p.multi_nozzle_diameter_mode) + " " + ENTS(dst));
                clear(dst);
            }

    // fill_expolygons_with_sheath_generate_paths
    struct F
    {
        float         density;
        bool          with_sheath, no_sort;
        InfillPattern pattern;
        ExtrusionRole role;
    };
    const std::vector<F> fs = {{0.f, true, false, ipRectilinear, erSupportMaterial},   {0.25f, true, false, ipRectilinear, erSupportMaterial},
                               {0.4f, true, true, ipSupportBase, erSupportMaterial},   {0.5f, false, false, ipConcentric, erSupportMaterialInterface},
                               {1.f, false, true, ipRectilinear, erSupportTransition}, {0.f, false, false, ipHoneycomb, erSupportMaterial}};
    for (size_t i = 0; i < shapes.size(); ++i)
        for (size_t f = 0; f < flows.size(); ++f)
            for (size_t k = 0; k < fs.size(); ++k) {
                if ((i + f + k) % 2 == 0) continue;
                std::unique_ptr<RecFill> filler(RecFill::new_from_type(fs[k].pattern));
                filler->angle           = 0.25f * float(k + 1);
                filler->spacing         = flows[f].spacing();
                filler->link_max_length = coord_t(100 * k);
                filler->set_bounding_box(BoundingBox(Point(-5, -7), Point(11, 13)));
                ExtrusionEntitiesPtr dst;
                fill_expolygons_with_sheath_generate_paths(dst, shapes[i], filler.get(), fs[k].density, fs[k].role, flows[f], p0, fs[k].with_sheath, fs[k].no_sort);
                out("fesg " + hI((long long) i) + " " + hI((long long) f) + " " + hI((long long) k) + " " + ENTS(dst));
                clear(dst);
            }

    // LoopInterfaceProcessor::generate
    for (int c = 0; c < 6; ++c) {
        SupportGeneratorLayerStorage storage;
        SupportGeneratorLayer       &l = storage.allocate(sltTopContact);
        l.print_z                      = 2.0;
        l.height                       = c % 2 ? 0.2 : 0.15;
        l.bottom_z                     = l.print_z - l.height;
        l.polygons                     = shapes[c % 3 == 0 ? 1 : c % 3 == 1 ? 2 : 5];
        if (c != 4) l.overhang_polygons = std::make_unique<Polygons>(c % 2 ? Polygons{rect(-2, -2, 16, 12)} : Polygons{rect(5, -3, 35, 30), ngon(0, 0, 3, 12)});
        const Flow            &flow = flows[c % 3];
        LoopInterfaceProcessor proc(1.5 * flow.scaled_width());
        proc.n_contact_loops = 1 + c % 3;
        out("in lip " + hD(proc.circle_radius) + " " + hI(proc.n_contact_loops) + " " + hI(c % 3) + " " + LAYER(l));
        SupportGeneratorLayerExtruded ex;
        ex.layer = &l;
        proc.generate(ex, flow);
        out("lip " + POLY(proc.circle) + " " + ENTS(ex.extrusions) + " " + POLYS(ex.polygons_to_extrude()) + " " + LAYER(l));
        clear(ex.extrusions);
    }

    // SupportGeneratorLayerExtruded: could_merge, merge, polygons_append
    for (int c = 0; c < 6; ++c) {
        SupportGeneratorLayerStorage storage;
        SupportGeneratorLayer       &a = storage.allocate(sltBase);
        a.print_z                      = 1.;
        a.height                       = 0.2;
        a.polygons                     = {rect(0, 0, 10, 6)};
        SupportGeneratorLayer &b       = storage.allocate(sltTopContact);
        b.print_z                      = 1.;
        b.height                       = c >= 4 ? 0.25 : 0.2;
        b.polygons                     = {rect(8, 2, 16, 9)};
        SupportGeneratorLayerExtruded ea, eb;
        ea.layer = &a;
        eb.layer = &b;
        if (c & 1) ea.set_polygons_to_extrude({rect(1, 1, 9, 5)});
        if (c & 2) eb.set_polygons_to_extrude({rect(9, 3, 15, 8)});
        const bool cm = ea.could_merge(eb);
        if (cm) ea.merge(std::move(eb));
        Polygons app;
        ea.polygons_append(app);
        out("extruded_merge " + hI(c) + " " + hB(cm) + " " + POLYS(ea.polygons_to_extrude()) + " " + POLYS(a.polygons) + " " + POLYS(b.polygons) + " " + hB(eb.empty()) + " " +
            POLYS(app));
    }

    // modulate_extrusion_by_overlapping_layers
    for (int c = 0; c < 4; ++c) {
        SupportGeneratorLayerStorage storage;
        SupportGeneratorLayer       &t = storage.allocate(c % 2 ? sltBottomContact : sltTopContact);
        t.print_z                      = 1.0;
        t.height                       = 0.3;
        t.bottom_z                     = 0.7;
        t.polygons                     = {rect(0, 0, 30, 20)};
        SupportGeneratorLayer &o1      = storage.allocate(sltIntermediate);
        o1.print_z                     = 0.8;
        o1.height                      = 0.2;
        o1.polygons                    = {rect(-5, -5, 12, 9)};
        SupportGeneratorLayer &o2      = storage.allocate(sltTopContact);
        o2.print_z                     = 0.9;
        o2.height                      = 0.2;
        o2.polygons                    = {ngon(20, 10, 6, 24), rect(10, 14, 16, 30)};
        SupportGeneratorLayersPtr overlapping = {&o1, &o2};
        if (c == 3) overlapping = {&o2};
        const Flow flow = flows[c % 3].with_height(0.3f);
        Polylines  lines;
        // c == 2: paths parallel to an axis
        for (int i = 0; i < 9; ++i)
            lines.emplace_back(Polyline(Point(scale_(-1.), scale_(1. + 2. * i)), Point(scale_(31.), scale_(1. + 2. * i + (c == 2 ? 0. : 0.7)))));
        lines.emplace_back(Polyline(Points{Point(scale_(2.), scale_(-2.)), Point(scale_(14.), scale_(12.)), Point(scale_(26.), scale_(3.)), Point(scale_(29.), scale_(19.))}));
        ExtrusionEntitiesPtr ents;
        extrusion_entities_append_paths(ents, Polylines(lines.begin(), lines.begin() + 4), erSupportMaterialInterface, flow.mm3_per_mm(), flow.width(), flow.height());
        ExtrusionEntityCollection *coll = new ExtrusionEntityCollection();
        coll->no_sort                   = true;
        extrusion_entities_append_paths(coll->entities, Polylines(lines.begin() + 4, lines.begin() + 7), erSupportMaterial, flow.mm3_per_mm(), flow.width(), flow.height(), false);
        ents.emplace_back(coll);
        ExtrusionMultiPath *mp = new ExtrusionMultiPath();
        mp->paths.emplace_back(ExtrusionPath(erSupportMaterial, flow.mm3_per_mm(), flow.width(), flow.height()));
        mp->paths.back().polyline = lines[7];
        ents.emplace_back(mp);
        extrusion_entities_append_paths(ents, Polylines(lines.begin() + 8, lines.end()), erSupportMaterial, flow.mm3_per_mm(), flow.width(), flow.height());
        std::string in = "in mod " + LAYER(t) + " | " + hI((long long) overlapping.size());
        for (const SupportGeneratorLayer *o : overlapping) in += " | " + LAYER(*o);
        out(in);
        out("in mod_ents " + ENTS(ents));
        modulate_extrusion_by_overlapping_layers(ents, t, overlapping);
        out("mod " + ENTS(ents));
        clear(ents);
    }
}

typedef std::vector<std::pair<std::string, std::string>> Cfg;

static std::vector<Cfg> scenarios()
{
    std::vector<Cfg> s;
    // 0: grid, three top interface layers, the brim of the automatic type.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(auto)"}});
    // 1: snug, top and bottom interfaces, a raft with every kind of layer.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(manual)"}, {"support_style", "snug"}, {"support_interface_top_layers", "3"},
                 {"support_interface_bottom_layers", "2"}, {"raft_layers", "5"}, {"raft_first_layer_expansion", "1.5"}});
    // 2: no interface layers, a raft of two layers, the automatic expansion of its first layer.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_interface_top_layers", "0"}, {"raft_layers", "2"}});
    // 3: a soluble interface over a base that is not; an outer and inner brim with a gap.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_top_z_distance", "0"}, {"support_interface_filament", "3"},
                 {"support_filament", "1"}, {"filament_soluble", "0,0,1"}, {"support_interface_top_layers", "3"}, {"support_interface_bottom_layers", "4"},
                 {"brim_type", "outer_and_inner"}, {"brim_width", "4"}, {"brim_object_gap", "0.1"}, {"raft_first_layer_expansion", "0.6"}});
    // 4: tree supports, the interface of another filament; an outer brim with a gap; no expansion of the first layer.
    s.push_back({{"enable_support", "1"}, {"support_type", "tree(auto)"}, {"support_style", "tree_hybrid"}, {"support_interface_filament", "2"},
                 {"support_interface_top_layers", "3"}, {"support_interface_bottom_layers", "2"}, {"brim_type", "outer_only"}, {"brim_width", "3"},
                 {"brim_object_gap", "0.2"}, {"raft_first_layer_expansion", "0"}});
    // 5: normal supports, the interface of another filament (two base interface layers); an inner brim.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_interface_filament", "2"}, {"support_interface_top_layers", "2"},
                 {"support_interface_bottom_layers", "1"}, {"brim_type", "inner_only"}, {"brim_width", "3"}, {"brim_object_gap", "0.15"},
                 {"raft_first_layer_expansion", "3"}});
    // 6: a raft of one layer, snug.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_style", "snug"}, {"raft_layers", "1"}, {"raft_first_layer_expansion", "3"},
                 {"support_interface_bottom_layers", "-1"}});
    // 7: tree supports with a raft of three layers and the interfaces of the organic supports handed in.
    s.push_back({{"enable_support", "1"}, {"support_type", "tree(auto)"}, {"raft_layers", "3"}, {"support_interface_top_layers", "2"},
                 {"support_interface_bottom_layers", "0"}, {"raft_first_layer_expansion", "-1"}, {"support_interface_filament", "2"}});
    // 8: no brim, no raft, a small expansion below the fine inflation.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"brim_type", "no_brim"}, {"raft_first_layer_expansion", "0.0005"},
                 {"support_interface_top_layers", "1"}, {"support_interface_bottom_layers", "1"}, {"support_object_first_layer_gap", "0.45"}});
    // 9: the loops of the contact layers.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_interface_loop_pattern", "1"}, {"support_interface_top_layers", "2"},
                 {"support_interface_bottom_layers", "2"}});
    // 10: no interface layers: the contacts are printed as the base; a raft; the grid of the base.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"support_interface_top_layers", "0"}, {"support_interface_bottom_layers", "0"},
                 {"raft_layers", "3"}, {"support_base_pattern", "rectilinear-grid"}});
    // 11: organic trees: the walls of the branches, lightning lines and cooldown areas handed in.
    s.push_back({{"enable_support", "1"}, {"support_type", "tree(auto)"}, {"support_style", "tree_organic"}, {"support_interface_top_layers", "2"}});
    // 12: ironing of the interfaces, concentric interfaces without a spacing, the nozzles of a merge plan.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(auto)"}, {"enable_support_ironing", "1"}, {"support_ironing_inset", "0.2"},
                 {"support_interface_pattern", "concentric"}, {"support_interface_spacing", "0"}});
    // 13: snug, a honeycomb base, interlaced interfaces of another filament, a raft of four layers.
    s.push_back({{"enable_support", "1"}, {"support_type", "normal(manual)"}, {"support_style", "snug"}, {"support_base_pattern", "honeycomb"},
                 {"support_interface_pattern", "rectilinear_interlaced"}, {"support_interface_filament", "2"}, {"support_interface_top_layers", "3"},
                 {"raft_layers", "4"}, {"support_interface_bottom_layers", "0"}});
    return s;
}

static SupportGeneratorLayer &add_layer(SupportGeneratorLayerStorage &storage, SupporLayerType type, double print_z, double height, double bottom_z, Polygons polygons)
{
    SupportGeneratorLayer &l = storage.allocate(type);
    l.print_z  = print_z;
    l.height   = height;
    l.bottom_z = bottom_z;
    l.polygons = std::move(polygons);
    return l;
}

static void run_scenario(int scenario, const Cfg &cfg)
{
    const std::string S = "s" + hI(scenario);
    Model        model;
    ModelObject *o = model.add_object();
    o->name = "box";
    indexed_triangle_set its = its_make_cube(20, 20, 10);
    for (Vec3f &v : its.vertices) v += Vec3f(-10.f, -10.f, 0.f);
    o->add_volume(TriangleMesh(its));
    o->add_instance()->set_offset(Vec3d(100., 100., 0.));
    out("in " + S + " model 20 20 10 100 100");

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(3);
    config.option<ConfigOptionStrings>("filament_colour", true)->values = {"#FF0000", "#00FF00", "#0000FF"};
    config.option<ConfigOptionInts>("filament_map", true)->values = {1, 1, 1};
    for (const auto &kv : cfg) {
        config.set_deserialize_strict(kv.first, kv.second);
        out("in " + S + " cfg " + kv.first + " " + kv.second);
    }
    Print print;
    print.apply(model, config);
    PrintObject *obj = print.m_objects.front();
    obj->update_slicing_parameters();
    const SlicingParameters &sp = obj->slicing_parameters();
    out(S + " slicing " + hD(sp.layer_height) + " " + hD(sp.first_print_layer_height) + " " + hI((long long) sp.base_raft_layers) + " " +
        hI((long long) sp.interface_raft_layers) + " " + hD(sp.base_raft_layer_height) + " " + hD(sp.interface_raft_layer_height) + " " +
        hD(sp.contact_raft_layer_height) + " " + hD(sp.raft_base_top_z) + " " + hD(sp.raft_interface_top_z) + " " + hD(sp.raft_contact_top_z) + " " +
        hD(sp.object_print_z_min) + " " + hB(sp.soluble_interface) + " " + hI((long long) sp.raft_layers()));

    // The object layers: the first one carries the slices of the object, a square with a hole and an island.
    const int n_object_layers = 4;
    for (int i = 0; i < n_object_layers; ++i) {
        const double z = sp.object_print_z_min + 0.2 * (i + 1);
        Layer *layer = new Layer(i, obj, 0.2, z, z - 0.1);
        if (i == 0) layer->lslices = {with_hole(rect(-10, -10, 10, 10), rect(-3, -3, 3, 3)), ExPolygon(ngon(18.5, 0, 2.2, 16))};
        obj->m_layers.emplace_back(layer);
        out("in " + S + " layer " + hD(z) + " " + hD(0.2) + " " + hD(layer->slice_z) + " " + EXPOLYS(layer->lslices));
    }

    const SupportParameters p(*obj);
    if (scenario == 0) run_path_helpers(p);

    // The support generator layers: a column A of intermediate layers under an overhang, a shorter column B.
    SupportGeneratorLayerStorage storage;
    SupportGeneratorLayersPtr    bottom_contacts, top_contacts, intermediate_layers, top_interface_layers, top_base_interface_layers;
    const double z0  = sp.raft_layers() > 0 ? sp.raft_interface_top_z : 0.;
    const int    n   = 12;
    const double gap = sp.soluble_interface ? 0. : 0.15;
    if (sp.raft_layers() > 0 && scenario != 7) {
        // The raft contact layer.
        SupportGeneratorLayer &l = add_layer(storage, sltTopContact, sp.raft_contact_top_z, sp.contact_raft_layer_height, sp.raft_contact_top_z - sp.contact_raft_layer_height,
                                             {rect(-11, -11, 11, 11), ngon(18.5, 0, 3., 16)});
        l.overhang_polygons = std::make_unique<Polygons>(Polygons{rect(-10, -10, 10, 10)});
        top_contacts.emplace_back(&l);
    }
    for (int i = 0; i < n; ++i) {
        const double h = (i == 0 && z0 == 0.) ? sp.first_print_layer_height : 0.2;
        const double z = (i == 0) ? z0 + h : intermediate_layers.back()->print_z + h;
        Polygons     polys = {rect(20 + 0.05 * i, -8, 35, 8 - 0.07 * i)};
        if (i < 6) polys.emplace_back(rect(-35, -6 + 0.11 * i, -22 - 0.03 * i, 6));
        if (i >= 3 && i < 9) polys.emplace_back(ngon(0, 24, 4.5 - 0.1 * i, 20));
        intermediate_layers.emplace_back(&add_layer(storage, sltIntermediate, z, h, z - h, polys));
    }
    {
        // The top contact of column B, above the sixth intermediate layer; of the disc; of column A.
        const double zb = intermediate_layers[5]->print_z;
        SupportGeneratorLayer &b = add_layer(storage, sltTopContact, zb + 0.2 - gap, 0.2 - gap > 0.05 ? 0.2 - gap : 0.2, zb, {rect(-35.6, -6.4, -21.5, 6.5)});
        b.overhang_polygons = std::make_unique<Polygons>(Polygons{rect(-36.5, -7, -21, 7)});
        b.idx_object_layer_above = 7;
        top_contacts.emplace_back(&b);
        const double zd = intermediate_layers[8]->print_z;
        SupportGeneratorLayer &d = add_layer(storage, sltTopContact, zd + 0.2, 0.2, zd, {ngon(0, 24, 5.2, 20)});
        d.overhang_polygons = std::make_unique<Polygons>(Polygons{ngon(0, 24, 5.8, 20)});
        d.bridging = (scenario == 5);
        top_contacts.emplace_back(&d);
        const double za = intermediate_layers[n - 1]->print_z;
        SupportGeneratorLayer &a = add_layer(storage, sltTopContact, za + 0.2 - gap, 0.2 - gap > 0.05 ? 0.2 - gap : 0.2, za, {rect(19.4, -8.6, 35.5, 8.2), rect(40, -2, 44, 2)});
        a.overhang_polygons = std::make_unique<Polygons>(Polygons{rect(18.5, -9.5, 36.5, 9)});
        a.idx_object_layer_above = 13;
        top_contacts.emplace_back(&a);
    }
    {
        // Bottom contacts: on top of an object part below column A, and one below the disc.
        const double z1 = intermediate_layers[1]->print_z;
        SupportGeneratorLayer &a = add_layer(storage, sltBottomContact, z1, 0.2, z1 - 0.2, {rect(24, -5, 31, 4)});
        a.idx_object_layer_below = 1;
        a.bridging = (scenario == 3);
        bottom_contacts.emplace_back(&a);
        const double z2 = intermediate_layers[3]->print_z - 0.05;
        SupportGeneratorLayer &d = add_layer(storage, sltBottomContact, z2, 0.15, z2 - 0.15, {ngon(0.5, 24, 3.6, 12)});
        d.idx_object_layer_below = 3;
        bottom_contacts.emplace_back(&d);
    }
    if (scenario == 7) {
        // The interfaces of the organic supports: at some of the intermediate layers, one of them empty, one null.
        for (int i : {9, 10, 11}) {
            const SupportGeneratorLayer *il = intermediate_layers[i];
            top_interface_layers.emplace_back(&add_layer(storage, sltTopInterface, il->print_z, il->height, il->bottom_z, i == 10 ? Polygons{} : Polygons{rect(18, -9, 37, 9.5)}));
        }
        top_interface_layers.insert(top_interface_layers.begin() + 1, nullptr);
        for (int i : {7, 8}) {
            const SupportGeneratorLayer *il = intermediate_layers[i];
            top_base_interface_layers.emplace_back(&add_layer(storage, sltBase, il->print_z, il->height, il->bottom_z, {rect(18.5, -8.5, 36, 9)}));
        }
    }
    reindex(storage);
    out("in " + S + " storage " + hI((long long) storage.m_storage.size()));
    for (const SupportGeneratorLayer &l : storage.m_storage) out("in " + S + " L " + LAYER(l));
    out("in " + S + " lists " + LIST(bottom_contacts) + " " + LIST(top_contacts) + " " + LIST(intermediate_layers) + " " + LIST(top_interface_layers) + " " +
        LIST(top_base_interface_layers));

    // compact_layers on a copy of the lists.
    {
        SupportGeneratorLayersPtr v = top_interface_layers;
        v.insert(v.end(), top_contacts.begin(), top_contacts.end());
        v.emplace_back(nullptr);
        compact_layers(v);
        out(S + " compact_layers " + LIST(v));
    }

    // With no intermediate layers the interfaces of the organic supports pass through (on copies).
    {
        SupportGeneratorLayersPtr none, ti = top_interface_layers, tb = top_base_interface_layers;
        if (scenario == 7) ti.erase(ti.begin() + 1);
        auto r = generate_interface_layers(obj->config(), p, bottom_contacts, top_contacts, ti, tb, none, storage);
        out(S + " interface_layers_no_base " + LIST(r.first) + " " + LIST(r.second) + " " + LIST(ti) + " " + LIST(tb));
    }
    if (scenario == 7) top_interface_layers.erase(top_interface_layers.begin() + 1);

    auto [interface_layers, base_interface_layers] =
        generate_interface_layers(obj->config(), p, bottom_contacts, top_contacts, top_interface_layers, top_base_interface_layers, intermediate_layers, storage);
    dump_storage(S + " interface", storage);
    out(S + " interface lists " + LIST(interface_layers) + " " + LIST(base_interface_layers) + " " + LIST(top_interface_layers) + " " + LIST(top_base_interface_layers));

    SupportGeneratorLayersPtr raft_layers = generate_raft_base(*obj, p, sp, top_contacts, interface_layers, base_interface_layers, intermediate_layers, storage);
    dump_storage(S + " raft", storage);
    out(S + " raft lists " + LIST(raft_layers));

    SupportGeneratorLayersPtr layers_sorted = generate_support_layers(*obj, raft_layers, bottom_contacts, top_contacts, intermediate_layers, interface_layers, base_interface_layers);
    dump_storage(S + " support_layers", storage);
    {
        // The layers that compare equal come out in the order of their addresses: print them by their index.
        bool sorted = true;
        for (size_t i = 1; i < layers_sorted.size(); ++i)
            if (*layers_sorted[i] < *layers_sorted[i - 1]) sorted = false;
        std::vector<int> idx;
        for (const SupportGeneratorLayer *l : layers_sorted) idx.emplace_back(g_index.at(l));
        std::sort(idx.begin(), idx.end());
        std::string line = S + " layers_sorted " + hB(sorted) + " " + hI((long long) idx.size());
        for (int i : idx) line += " " + hI(i);
        out(line);
        line = S + " object_support_layers " + hI((long long) obj->support_layers().size());
        for (const SupportLayer *sl : obj->support_layers())
            line += " " + hI((long long) sl->id()) + " " + hI((long long) sl->interface_id()) + " " + hD(sl->print_z) + " " + hD(sl->height) + " " + hD(sl->slice_z);
        out(line);
    }

    // The toolpaths.
    {
        std::vector<ExPolygons> cooldown_areas;
        std::vector<Polylines>  lightning;
        if (scenario == 4 || scenario == 11) {
            // Cooldown areas at every third support layer.
            cooldown_areas.resize(obj->support_layers().size());
            for (size_t i = 0; i < cooldown_areas.size(); i += 3) cooldown_areas[i] = {ExPolygon(rect(25, -4, 33, 6))};
        }
        if (scenario == 11) {
            lightning.resize(intermediate_layers.size());
            for (size_t i = 0; i < lightning.size(); i += 2)
                lightning[i] = {Polyline(Point(scale_(21.), scale_(-7.)), Point(scale_(34.), scale_(6.5))),
                                Polyline(Points{Point(scale_(22.), scale_(5.)), Point(scale_(27.), scale_(1.)), Point(scale_(27.4), scale_(-6.))}),
                                Polyline(Point(scale_(30.), scale_(0.)), Point(scale_(30.5), scale_(0.3)))};
        }
        if (scenario == 12)
            for (SupportLayer *sl : obj->m_support_layers) {
                sl->planned_nozzle_diameter = 0.6;
                if (sl->id() % 2) sl->planned_interface_nozzle_diameter = 0.6;
                if (sl->id() % 3 == 0) sl->interface_slab_height = 0.3;
            }
        std::string in = "in " + S + " toolpaths " + hI((long long) cooldown_areas.size());
        for (const ExPolygons &a : cooldown_areas) in += " " + EXPOLYS(a);
        in += " " + hI((long long) lightning.size());
        for (const Polylines &l : lightning) in += " " + PLS(l);
        in += " " + hI((long long) obj->support_layers().size());
        for (const SupportLayer *sl : obj->support_layers())
            in += " " + hD(sl->planned_nozzle_diameter) + " " + hD(sl->planned_interface_nozzle_diameter) + " " + hD(sl->interface_slab_height);
        out(in);
        generate_support_toolpaths(obj->m_support_layers, obj->config(), p, sp, raft_layers, bottom_contacts, top_contacts, intermediate_layers, interface_layers,
                                   base_interface_layers, cooldown_areas, scenario == 11 ? &lightning : nullptr, {});
        dump_storage(S + " toolpaths", storage);
        for (const SupportLayer *sl : obj->support_layers())
            out(S + " toolpaths fills " + hI((long long) sl->id()) + " " + ENTS(sl->support_fills.entities) + " | " + EXPOLYS(sl->support_islands));
        std::string line = S + " toolpaths lightning " + hI((long long) lightning.size());
        for (const Polylines &l : lightning) line += " " + PLS(l);
        out(line);
    }
}

int main(int argc, char **argv)
{
    g_out = std::fopen(argc > 1 ? argv[1] : "vectors.txt", "w");
    tbb::global_control gc(tbb::global_control::max_allowed_parallelism, 1);
    run_helpers();
    const std::vector<Cfg> s = scenarios();
    for (size_t i = 0; i < s.size(); ++i) run_scenario(int(i), s[i]);
    std::fclose(g_out);
    return 0;
}
