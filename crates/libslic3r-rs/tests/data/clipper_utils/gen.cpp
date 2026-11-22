// Generator of the vectors of tests/clipper_utils_cpp_native_parity.rs: every overload of ClipperUtils.hpp of
// libslic3r on deterministic inputs, with the results `clipper_utils::cpp` of the Rust port has to reproduce.
//
// Build with the compile line the build uses for a libslic3r source (its defines and include directories,
// -std=c++17 -O3 -DNDEBUG), link with the engine's libraries and run `./gen vectors.txt`.
//
// A case is an `in` line and one line per call. All tokens are integers.
//   in jt ml*100 et delta*2 delta_b*2 sgn A B bbox(4)
//     A and B (subject and clip): polygons, expolygons (the union of the polygons), polylines, lines.
//     jt, et: ClipperLib::JoinType / EndType; the offsets are delta (positive), od = sgn * delta,
//     od2 = -sgn * delta_b, lw = delta + 2 (a line width); bbox is the bounding box of the first clip polygon.
//   <id> result
//     polygons / polylines: n (m (x y)*m)*n; expolygons: n (contour holes)*n, holes: h polygon*h;
//     lines: n (ax ay bx by)*n; a point list: m (x y)*m; a PolyTree: for every node, depth first, its contour
//     and the number of its children, the root first (its contour is empty).
// The id of a line names the function, the types of its arguments (Pg Polygon, Pgs Polygons, Ex ExPolygon,
// Exs ExPolygons, Sf Surfaces, Sp SurfacesPtr, Pl Polyline, Pls Polylines, Ln Line, Lns Lines) and, where the
// function takes one, the safety offset (0 no, 1 yes).
//
// This file is written by a script from the table of overloads; the calls below are that table.
// The vectors were made on macOS arm64 with Apple clang.

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Surface.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

// Overloads ClipperUtils.cpp defines and ClipperUtils.hpp does not declare.
namespace Slic3r {
// ClipperUtils.cpp:574
ExPolygons offset_ex(const SurfacesPtr &surfaces, const float delta, ClipperLib::JoinType joinType, double miterLimit);
// ClipperUtils.cpp:604
ExPolygons closing_ex(const Surfaces &surfaces, const float delta1, const float delta2, ClipperLib::JoinType joinType, double miterLimit);
// ClipperUtils.cpp:770
ExPolygons diff_ex(const Polygon &subject, const Polygons &clip, ApplySafetyOffset do_safety_offset);
// ClipperUtils.cpp:778
ExPolygons diff_ex(const Polygon &subject, const Polygon &clip, ApplySafetyOffset do_safety_offset);
} // namespace Slic3r

using namespace Slic3r;

struct Lcg {
    unsigned long long s;
    explicit Lcg(unsigned long long seed) : s(seed) {}
    unsigned next()
    {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return unsigned(s >> 33);
    }
    // lo <= result < hi
    int range(int lo, int hi) { return lo + int(next() % unsigned(hi - lo)); }
};

static FILE *g_out = nullptr;

static void put(const Point &p) { std::fprintf(g_out, " %lld %lld", (long long) p.x(), (long long) p.y()); }
static void put(const Points &pts)
{
    std::fprintf(g_out, " %zu", pts.size());
    for (const Point &p : pts)
        put(p);
}
static void put(const Polygon &p) { put(p.points); }
static void put(const Polygons &v)
{
    std::fprintf(g_out, " %zu", v.size());
    for (const Polygon &p : v)
        put(p.points);
}
static void put(const Polylines &v)
{
    std::fprintf(g_out, " %zu", v.size());
    for (const Polyline &p : v)
        put(p.points);
}
static void put(const ExPolygons &v)
{
    std::fprintf(g_out, " %zu", v.size());
    for (const ExPolygon &e : v) {
        put(e.contour.points);
        put(e.holes);
    }
}
static void put(const Lines &v)
{
    std::fprintf(g_out, " %zu", v.size());
    for (const Line &l : v) {
        put(l.a);
        put(l.b);
    }
}
static void put_node(const ClipperLib::PolyNode &n)
{
    put(n.Contour);
    std::fprintf(g_out, " %d", n.ChildCount());
    for (const ClipperLib::PolyNode *c : n.Childs)
        put_node(*c);
}
static void put(const ClipperLib::PolyTree &t) { put_node(t); }

#define R(id, expr)                    \
    do {                               \
        std::fprintf(g_out, "%s", id); \
        put(expr);                     \
        std::fputc('\n', g_out);       \
    } while (0)

struct Shapes {
    Polygon     pg;
    Polygons    pgs;
    ExPolygon   ex;
    ExPolygons  exs;
    Surfaces    sf;
    SurfacesPtr sp;
    Polyline    pl;
    Polylines   pls;
    Line        ln;
    Lines       lns;
};

// A polygon around c: the angles go round once, the radii are drawn. `grid` snaps the points to a grid, which
// makes collinear and repeated points and edges shared with the other polygons of the case.
static Polygon gen_polygon(Lcg &r, const Point &c, int rmax, bool grid)
{
    int     n = r.range(3, 10);
    Polygon p;
    for (int i = 0; i < n; ++i) {
        double  ang = 2. * M_PI * (double(i) + double(r.range(0, 100)) / 200.) / double(n);
        double  rad = double(rmax) * (0.35 + double(r.range(0, 65)) / 100.);
        coord_t x   = c.x() + coord_t(rad * std::cos(ang));
        coord_t y   = c.y() + coord_t(rad * std::sin(ang));
        if (grid) {
            x = (x / 250000) * 250000;
            y = (y / 250000) * 250000;
        }
        p.points.emplace_back(x, y);
    }
    return p;
}

static void gen_shapes(Lcg &r, Shapes &s)
{
    for (;;) {
        s = Shapes();
        bool grid = r.next() % 3 == 0;
        int  k    = r.range(1, 4);
        for (int i = 0; i < k; ++i)
            s.pgs.push_back(gen_polygon(r, Point(r.range(-2000000, 2000000), r.range(-2000000, 2000000)), r.range(1000000, 3000000), grid));
        if (r.next() % 3 == 0) {
            // A hole in the first polygon: a small one around its first point's side, clockwise.
            const Point &f = s.pgs.front().points.front();
            Point        c((f.x() + s.pgs.front().points[1].x()) / 3, (f.y() + s.pgs.front().points[1].y()) / 3);
            Polygon      hole = gen_polygon(r, c, r.range(100000, 400000), grid);
            hole.reverse();
            s.pgs.push_back(hole);
        }
        s.exs = union_ex(s.pgs);
        if (!s.exs.empty())
            break;
    }
    s.pg = s.pgs.front();
    s.ex = s.exs.front();
    int k = r.range(1, 4);
    for (int i = 0; i < k; ++i) {
        Polyline pl;
        int      m = r.range(2, 6);
        for (int j = 0; j < m; ++j)
            pl.points.emplace_back(r.range(-4000000, 4000000), r.range(-4000000, 4000000));
        s.pls.push_back(pl);
    }
    s.pl = s.pls.front();
    k    = r.range(1, 5);
    for (int i = 0; i < k; ++i)
        s.lns.emplace_back(Point(r.range(-4000000, 4000000), r.range(-4000000, 4000000)), Point(r.range(-4000000, 4000000), r.range(-4000000, 4000000)));
    s.ln = s.lns.front();
}

// The surfaces of the expolygons; the pointers point into `s.sf`, so this runs on the object the calls take.
static void set_surfaces(Shapes &s)
{
    for (const ExPolygon &e : s.exs)
        s.sf.emplace_back(stInternal, e);
    for (Surface &sf : s.sf)
        s.sp.push_back(&sf);
}

static void put_shapes(const Shapes &s)
{
    put(s.pgs);
    put(s.exs);
    put(s.pls);
    put(s.lns);
}

int main(int argc, char **argv)
{
    g_out = argc > 1 ? std::fopen(argv[1], "w") : stdout;
    if (g_out == nullptr)
        return 1;
    const int num_cases = argc > 2 ? std::atoi(argv[2]) : 48;
    for (int icase = 0; icase < num_cases; ++icase) {
        Lcg    r(20241122ULL + (unsigned long long) icase);
        Shapes a, b;
        gen_shapes(r, a);
        gen_shapes(r, b);
        set_surfaces(a);
        set_surfaces(b);
        const int                  jti  = icase % 3;
        const ClipperLib::JoinType jt   = jti == 0 ? ClipperLib::jtMiter : jti == 1 ? ClipperLib::jtRound : ClipperLib::jtSquare;
        const int                  ml100 = jti == 0 ? (icase % 2 == 0 ? 300 : 150) : jti == 1 ? 125000 : 0;
        const double               ml   = double(ml100) / 100.;
        const int                  eti  = (icase / 3) % 3;
        const ClipperLib::EndType  et   = eti == 0 ? ClipperLib::etOpenButt : eti == 1 ? ClipperLib::etOpenSquare : ClipperLib::etOpenRound;
        const int                  delta2  = r.range(1, 400) * 1001;
        const int                  deltab2 = r.range(1, 400) * 1001;
        const int                  sgn     = r.next() % 2 == 0 ? 1 : -1;
        const float                delta   = float(delta2) / 2.f;
        const float                delta_b = float(deltab2) / 2.f;
        const float                od      = float(sgn) * delta;
        const float                od2     = -float(sgn) * delta_b;
        const float                lw      = delta + 2.f;
        const BoundingBox          bbox    = get_extents(b.pg);
        std::fprintf(g_out, "in %d %d %d %d %d %d", int(jt), ml100, int(et), delta2, deltab2, sgn);
        put_shapes(a);
        put_shapes(b);
        put(bbox.min);
        put(bbox.max);
        std::fputc('\n', g_out);
        R("diff:Pg:Pg:0", diff(a.pg, b.pg, ApplySafetyOffset::No));
        R("diff:Pg:Pg:1", diff(a.pg, b.pg, ApplySafetyOffset::Yes));
        R("diff:Pgs:Pgs:0", diff(a.pgs, b.pgs, ApplySafetyOffset::No));
        R("diff:Pgs:Pgs:1", diff(a.pgs, b.pgs, ApplySafetyOffset::Yes));
        R("diff:Pgs:Exs:0", diff(a.pgs, b.exs, ApplySafetyOffset::No));
        R("diff:Pgs:Exs:1", diff(a.pgs, b.exs, ApplySafetyOffset::Yes));
        R("diff:Exs:Pgs:0", diff(a.exs, b.pgs, ApplySafetyOffset::No));
        R("diff:Exs:Pgs:1", diff(a.exs, b.pgs, ApplySafetyOffset::Yes));
        R("diff:Exs:Exs:0", diff(a.exs, b.exs, ApplySafetyOffset::No));
        R("diff:Exs:Exs:1", diff(a.exs, b.exs, ApplySafetyOffset::Yes));
        R("diff:Sf:Pgs:0", diff(a.sf, b.pgs, ApplySafetyOffset::No));
        R("diff:Sf:Pgs:1", diff(a.sf, b.pgs, ApplySafetyOffset::Yes));
        R("intersection:Pg:Pg:0", intersection(a.pg, b.pg, ApplySafetyOffset::No));
        R("intersection:Pg:Pg:1", intersection(a.pg, b.pg, ApplySafetyOffset::Yes));
        R("intersection:Pgs:Ex:0", intersection(a.pgs, b.ex, ApplySafetyOffset::No));
        R("intersection:Pgs:Ex:1", intersection(a.pgs, b.ex, ApplySafetyOffset::Yes));
        R("intersection:Pgs:Pgs:0", intersection(a.pgs, b.pgs, ApplySafetyOffset::No));
        R("intersection:Pgs:Pgs:1", intersection(a.pgs, b.pgs, ApplySafetyOffset::Yes));
        R("intersection:Ex:Ex:0", intersection(a.ex, b.ex, ApplySafetyOffset::No));
        R("intersection:Ex:Ex:1", intersection(a.ex, b.ex, ApplySafetyOffset::Yes));
        R("intersection:Exs:Pgs:0", intersection(a.exs, b.pgs, ApplySafetyOffset::No));
        R("intersection:Exs:Pgs:1", intersection(a.exs, b.pgs, ApplySafetyOffset::Yes));
        R("intersection:Exs:Exs:0", intersection(a.exs, b.exs, ApplySafetyOffset::No));
        R("intersection:Exs:Exs:1", intersection(a.exs, b.exs, ApplySafetyOffset::Yes));
        R("intersection:Sf:Pgs:0", intersection(a.sf, b.pgs, ApplySafetyOffset::No));
        R("intersection:Sf:Pgs:1", intersection(a.sf, b.pgs, ApplySafetyOffset::Yes));
        R("intersection:Sf:Exs:0", intersection(a.sf, b.exs, ApplySafetyOffset::No));
        R("intersection:Sf:Exs:1", intersection(a.sf, b.exs, ApplySafetyOffset::Yes));
        R("intersection:Pgs:Pg:0", intersection(a.pgs, b.pg, ApplySafetyOffset::No));
        R("intersection:Pgs:Pg:1", intersection(a.pgs, b.pg, ApplySafetyOffset::Yes));
        R("diff_ex:Pgs:Pgs:0", diff_ex(a.pgs, b.pgs, ApplySafetyOffset::No));
        R("diff_ex:Pgs:Pgs:1", diff_ex(a.pgs, b.pgs, ApplySafetyOffset::Yes));
        R("diff_ex:Pgs:Sf:0", diff_ex(a.pgs, b.sf, ApplySafetyOffset::No));
        R("diff_ex:Pgs:Sf:1", diff_ex(a.pgs, b.sf, ApplySafetyOffset::Yes));
        R("diff_ex:Pg:Exs:0", diff_ex(a.pg, b.exs, ApplySafetyOffset::No));
        R("diff_ex:Pg:Exs:1", diff_ex(a.pg, b.exs, ApplySafetyOffset::Yes));
        R("diff_ex:Pgs:Exs:0", diff_ex(a.pgs, b.exs, ApplySafetyOffset::No));
        R("diff_ex:Pgs:Exs:1", diff_ex(a.pgs, b.exs, ApplySafetyOffset::Yes));
        R("diff_ex:Ex:Pg:0", diff_ex(a.ex, b.pg, ApplySafetyOffset::No));
        R("diff_ex:Ex:Pg:1", diff_ex(a.ex, b.pg, ApplySafetyOffset::Yes));
        R("diff_ex:Ex:Pgs:0", diff_ex(a.ex, b.pgs, ApplySafetyOffset::No));
        R("diff_ex:Ex:Pgs:1", diff_ex(a.ex, b.pgs, ApplySafetyOffset::Yes));
        R("diff_ex:Exs:Pgs:0", diff_ex(a.exs, b.pgs, ApplySafetyOffset::No));
        R("diff_ex:Exs:Pgs:1", diff_ex(a.exs, b.pgs, ApplySafetyOffset::Yes));
        R("diff_ex:Exs:Exs:0", diff_ex(a.exs, b.exs, ApplySafetyOffset::No));
        R("diff_ex:Exs:Exs:1", diff_ex(a.exs, b.exs, ApplySafetyOffset::Yes));
        R("diff_ex:Sf:Pgs:0", diff_ex(a.sf, b.pgs, ApplySafetyOffset::No));
        R("diff_ex:Sf:Pgs:1", diff_ex(a.sf, b.pgs, ApplySafetyOffset::Yes));
        R("diff_ex:Sf:Exs:0", diff_ex(a.sf, b.exs, ApplySafetyOffset::No));
        R("diff_ex:Sf:Exs:1", diff_ex(a.sf, b.exs, ApplySafetyOffset::Yes));
        R("diff_ex:Exs:Sf:0", diff_ex(a.exs, b.sf, ApplySafetyOffset::No));
        R("diff_ex:Exs:Sf:1", diff_ex(a.exs, b.sf, ApplySafetyOffset::Yes));
        R("diff_ex:Sf:Sf:0", diff_ex(a.sf, b.sf, ApplySafetyOffset::No));
        R("diff_ex:Sf:Sf:1", diff_ex(a.sf, b.sf, ApplySafetyOffset::Yes));
        R("diff_ex:Sp:Pgs:0", diff_ex(a.sp, b.pgs, ApplySafetyOffset::No));
        R("diff_ex:Sp:Pgs:1", diff_ex(a.sp, b.pgs, ApplySafetyOffset::Yes));
        R("diff_ex:Sp:Exs:0", diff_ex(a.sp, b.exs, ApplySafetyOffset::No));
        R("diff_ex:Sp:Exs:1", diff_ex(a.sp, b.exs, ApplySafetyOffset::Yes));
        R("diff_ex:Pg:Pgs:0", diff_ex(a.pg, b.pgs, ApplySafetyOffset::No));
        R("diff_ex:Pg:Pgs:1", diff_ex(a.pg, b.pgs, ApplySafetyOffset::Yes));
        R("diff_ex:Pg:Pg:0", diff_ex(a.pg, b.pg, ApplySafetyOffset::No));
        R("diff_ex:Pg:Pg:1", diff_ex(a.pg, b.pg, ApplySafetyOffset::Yes));
        R("diff_ex:Ex:Ex:0", diff_ex(a.ex, b.ex, ApplySafetyOffset::No));
        R("diff_ex:Ex:Ex:1", diff_ex(a.ex, b.ex, ApplySafetyOffset::Yes));
        R("diff_ex:Ex:Exs:0", diff_ex(a.ex, b.exs, ApplySafetyOffset::No));
        R("diff_ex:Ex:Exs:1", diff_ex(a.ex, b.exs, ApplySafetyOffset::Yes));
        R("diff_ex:Exs:Ex:0", diff_ex(a.exs, b.ex, ApplySafetyOffset::No));
        R("diff_ex:Exs:Ex:1", diff_ex(a.exs, b.ex, ApplySafetyOffset::Yes));
        R("intersection_ex:Pgs:Pgs:0", intersection_ex(a.pgs, b.pgs, ApplySafetyOffset::No));
        R("intersection_ex:Pgs:Pgs:1", intersection_ex(a.pgs, b.pgs, ApplySafetyOffset::Yes));
        R("intersection_ex:Ex:Pgs:0", intersection_ex(a.ex, b.pgs, ApplySafetyOffset::No));
        R("intersection_ex:Ex:Pgs:1", intersection_ex(a.ex, b.pgs, ApplySafetyOffset::Yes));
        R("intersection_ex:Ex:Ex:0", intersection_ex(a.ex, b.ex, ApplySafetyOffset::No));
        R("intersection_ex:Ex:Ex:1", intersection_ex(a.ex, b.ex, ApplySafetyOffset::Yes));
        R("intersection_ex:Pgs:Exs:0", intersection_ex(a.pgs, b.exs, ApplySafetyOffset::No));
        R("intersection_ex:Pgs:Exs:1", intersection_ex(a.pgs, b.exs, ApplySafetyOffset::Yes));
        R("intersection_ex:Exs:Pgs:0", intersection_ex(a.exs, b.pgs, ApplySafetyOffset::No));
        R("intersection_ex:Exs:Pgs:1", intersection_ex(a.exs, b.pgs, ApplySafetyOffset::Yes));
        R("intersection_ex:Exs:Ex:0", intersection_ex(a.exs, b.ex, ApplySafetyOffset::No));
        R("intersection_ex:Exs:Ex:1", intersection_ex(a.exs, b.ex, ApplySafetyOffset::Yes));
        R("intersection_ex:Ex:Exs:0", intersection_ex(a.ex, b.exs, ApplySafetyOffset::No));
        R("intersection_ex:Ex:Exs:1", intersection_ex(a.ex, b.exs, ApplySafetyOffset::Yes));
        R("intersection_ex:Exs:Exs:0", intersection_ex(a.exs, b.exs, ApplySafetyOffset::No));
        R("intersection_ex:Exs:Exs:1", intersection_ex(a.exs, b.exs, ApplySafetyOffset::Yes));
        R("intersection_ex:Sf:Pgs:0", intersection_ex(a.sf, b.pgs, ApplySafetyOffset::No));
        R("intersection_ex:Sf:Pgs:1", intersection_ex(a.sf, b.pgs, ApplySafetyOffset::Yes));
        R("intersection_ex:Sf:Exs:0", intersection_ex(a.sf, b.exs, ApplySafetyOffset::No));
        R("intersection_ex:Sf:Exs:1", intersection_ex(a.sf, b.exs, ApplySafetyOffset::Yes));
        R("intersection_ex:Sf:Sf:0", intersection_ex(a.sf, b.sf, ApplySafetyOffset::No));
        R("intersection_ex:Sf:Sf:1", intersection_ex(a.sf, b.sf, ApplySafetyOffset::Yes));
        R("intersection_ex:Sp:Exs:0", intersection_ex(a.sp, b.exs, ApplySafetyOffset::No));
        R("intersection_ex:Sp:Exs:1", intersection_ex(a.sp, b.exs, ApplySafetyOffset::Yes));
        R("xor_ex:Exs:Ex:0", xor_ex(a.exs, b.ex, ApplySafetyOffset::No));
        R("xor_ex:Exs:Ex:1", xor_ex(a.exs, b.ex, ApplySafetyOffset::Yes));
        R("xor_ex:Exs:Exs:0", xor_ex(a.exs, b.exs, ApplySafetyOffset::No));
        R("xor_ex:Exs:Exs:1", xor_ex(a.exs, b.exs, ApplySafetyOffset::Yes));
        R("diff_pl:Pl:Pgs", diff_pl(a.pl, b.pgs));
        R("diff_pl:Pls:Pgs", diff_pl(a.pls, b.pgs));
        R("diff_pl:Pl:Ex", diff_pl(a.pl, b.ex));
        R("diff_pl:Pl:Exs", diff_pl(a.pl, b.exs));
        R("diff_pl:Pls:Ex", diff_pl(a.pls, b.ex));
        R("diff_pl:Pls:Exs", diff_pl(a.pls, b.exs));
        R("diff_pl:Pgs:Pgs", diff_pl(a.pgs, b.pgs));
        R("intersection_pl:Pls:Pg", intersection_pl(a.pls, b.pg));
        R("intersection_pl:Pl:Ex", intersection_pl(a.pl, b.ex));
        R("intersection_pl:Pls:Ex", intersection_pl(a.pls, b.ex));
        R("intersection_pl:Pl:Pgs", intersection_pl(a.pl, b.pgs));
        R("intersection_pl:Pl:Exs", intersection_pl(a.pl, b.exs));
        R("intersection_pl:Pls:Pgs", intersection_pl(a.pls, b.pgs));
        R("intersection_pl:Pls:Exs", intersection_pl(a.pls, b.exs));
        R("intersection_pl:Pgs:Pgs", intersection_pl(a.pgs, b.pgs));
        R("offset:Offset:Pg", offset(a.pg, od, jt, ml));
        R("offset:Offset:Pgs", offset(a.pgs, od, jt, ml));
        R("offset:Offset:Ex", offset(a.ex, od, jt, ml));
        R("offset:Offset:Exs", offset(a.exs, od, jt, ml));
        R("offset:Offset:Sf", offset(a.sf, od, jt, ml));
        R("offset:Offset:Sp", offset(a.sp, od, jt, ml));
        R("offset:OffsetPolyline:Pl", offset(a.pl, delta, jt, ml, et));
        R("offset:OffsetPolyline:Pls", offset(a.pls, delta, jt, ml, et));
        R("offset_ex:OffsetEx:Pgs", offset_ex(a.pgs, od, jt, ml));
        R("offset_ex:OffsetEx:Ex", offset_ex(a.ex, od, jt, ml));
        R("offset_ex:OffsetEx:Exs", offset_ex(a.exs, od, jt, ml));
        R("offset_ex:OffsetEx:Sf", offset_ex(a.sf, od, jt, ml));
        R("offset_ex:OffsetEx:Sp", offset_ex(a.sp, od, jt, ml));
        R("offset_ex:OffsetEx:Pg", offset_ex(a.pg, od, jt, ml));
        R("contour_to_polygons:ContourToPolygons:Pg", contour_to_polygons(a.pg, lw, jt, ml));
        R("contour_to_polygons:ContourToPolygons:Pgs", contour_to_polygons(a.pgs, lw, jt, ml));
        R("expand:Expand:Pg", expand(a.pg, delta, jt, ml));
        R("expand:Expand:Pgs", expand(a.pgs, delta, jt, ml));
        R("shrink_ex:ShrinkEx:Pgs", shrink_ex(a.pgs, delta, jt, ml));
        R("shrink_ex:ShrinkEx:Exs", shrink_ex(a.exs, delta, jt, ml));
        R("offset2_ex:Offset2Ex:Exs", offset2_ex(a.exs, od, od2, jt, ml));
        R("offset2_ex:Offset2Ex:Sf", offset2_ex(a.sf, od, od2, jt, ml));
        R("closing:Pgs:2", closing(a.pgs, delta, delta_b, jt, ml));
        R("closing:Pgs:1", closing(a.pgs, delta, jt, ml));
        R("closing_ex:Pgs:2", closing_ex(a.pgs, delta, delta_b, jt, ml));
        R("closing_ex:Sf:2", closing_ex(a.sf, delta, delta_b, jt, ml));
        R("closing_ex:Pgs:1", closing_ex(a.pgs, delta, jt, ml));
        R("closing_ex:Exs:1", closing_ex(a.exs, delta, jt, ml));
        R("closing_ex:Sf:1", closing_ex(a.sf, delta, jt, ml));
        R("opening:Pgs:2", opening(a.pgs, delta, delta_b, jt, ml));
        R("opening:Exs:2", opening(a.exs, delta, delta_b, jt, ml));
        R("opening:Sf:2", opening(a.sf, delta, delta_b, jt, ml));
        R("opening:Pgs:1", opening(a.pgs, delta, jt, ml));
        R("opening:Exs:1", opening(a.exs, delta, jt, ml));
        R("opening:Sf:1", opening(a.sf, delta, jt, ml));
        R("opening_ex:OpeningEx:Exs", opening_ex(a.exs, delta, jt, ml));
        R("opening_ex:OpeningEx:Sf", opening_ex(a.sf, delta, jt, ml));
        R("union_safety_offset:Pgs", union_safety_offset(a.pgs));
        R("union_safety_offset_ex:Pgs", union_safety_offset_ex(a.pgs));
        R("union_safety_offset:Exs", union_safety_offset(a.exs));
        R("union_safety_offset_ex:Exs", union_safety_offset_ex(a.exs));
        R("expand_ex", expand_ex(a.pgs, delta, jt, ml));
        R("shrink", shrink(a.pgs, delta, jt, ml));
        R("offset2", offset2(a.exs, od, od2, jt, ml));
        R("diff_clipped:Pgs:Pgs:0", diff_clipped(a.pgs, b.pgs, ApplySafetyOffset::No));
        R("diff_clipped:Exs:Pgs:0", diff_clipped(a.exs, b.pgs, ApplySafetyOffset::No));
        R("diff_clipped:Exs:Exs:0", diff_clipped(a.exs, b.exs, ApplySafetyOffset::No));
        R("intersection_clipped:0", intersection_clipped(a.pgs, b.pgs, ApplySafetyOffset::No));
        R("diff_clipped:Pgs:Pgs:1", diff_clipped(a.pgs, b.pgs, ApplySafetyOffset::Yes));
        R("diff_clipped:Exs:Pgs:1", diff_clipped(a.exs, b.pgs, ApplySafetyOffset::Yes));
        R("diff_clipped:Exs:Exs:1", diff_clipped(a.exs, b.exs, ApplySafetyOffset::Yes));
        R("intersection_clipped:1", intersection_clipped(a.pgs, b.pgs, ApplySafetyOffset::Yes));
        R("union_:Pgs", union_(a.pgs));
        R("union_:Exs", union_(a.exs));
        R("union_:Pgs:Pgs", union_(a.pgs, b.pgs));
        R("union_:Pgs:Ex", union_(a.pgs, b.ex));
        R("union_ex:Exs", union_ex(a.exs));
        R("union_ex:Exs:Pgs", union_ex(a.exs, b.pgs));
        R("union_ex:Sf", union_ex(a.sf));
        R("union_ex:Exs:Exs:0", union_ex(a.exs, b.exs, false));
        R("union_ex:Exs:Exs:1", union_ex(a.exs, b.exs, true));
        R("_clipper_ln:i", _clipper_ln(ClipperLib::ctIntersection, a.lns, b.pgs));
        R("_clipper_ln:d", _clipper_ln(ClipperLib::ctDifference, a.lns, b.pgs));
        R("diff_ln", diff_ln(a.lns, b.pgs));
        R("intersection_ln:Lns", intersection_ln(a.lns, b.pgs));
        R("intersection_ln:Ln", intersection_ln(a.ln, b.pgs));
        R("union_pt:Pgs", union_pt(a.pgs));
        R("union_pt:Exs", union_pt(a.exs));
        R("union_pt_chained_outside_in", union_pt_chained_outside_in(a.pgs));
        R("simplify_polygons:0", simplify_polygons(a.pgs, false));
        R("simplify_polygons:1", simplify_polygons(a.pgs, true));
        R("simplify_polygons_ex:0", simplify_polygons_ex(a.pgs, false));
        R("simplify_polygons_ex:1", simplify_polygons_ex(a.pgs, true));
        R("top_level_islands", top_level_islands(a.pgs));
        R("ccp:Points", ClipperUtils::clip_clipper_polygon_with_subject_bbox(a.pg.points, bbox));
        R("ccp:Polygon:0", ClipperUtils::clip_clipper_polygon_with_subject_bbox(a.pg, bbox, false));
        R("ccp:Polygon:1", ClipperUtils::clip_clipper_polygon_with_subject_bbox(a.pg, bbox, true));
        R("ccps:Pgs", ClipperUtils::clip_clipper_polygons_with_subject_bbox(a.pgs, bbox));
        R("union_:Pgs:pftEvenOdd", union_(a.pgs, ClipperLib::pftEvenOdd));
        R("union_ex:Pgs:pftEvenOdd", union_ex(a.pgs, ClipperLib::pftEvenOdd));
        R("union_:Pgs:pftNonZero", union_(a.pgs, ClipperLib::pftNonZero));
        R("union_ex:Pgs:pftNonZero", union_ex(a.pgs, ClipperLib::pftNonZero));
        R("union_:Pgs:pftPositive", union_(a.pgs, ClipperLib::pftPositive));
        R("union_ex:Pgs:pftPositive", union_ex(a.pgs, ClipperLib::pftPositive));
        R("union_:Pgs:pftNegative", union_(a.pgs, ClipperLib::pftNegative));
        R("union_ex:Pgs:pftNegative", union_ex(a.pgs, ClipperLib::pftNegative));
        R("ccps:Ex:0", ClipperUtils::clip_clipper_polygons_with_subject_bbox(a.ex, bbox, false));
        R("ccps:Exs:0", ClipperUtils::clip_clipper_polygons_with_subject_bbox(a.exs, bbox, false));
        R("ccps:Ex:1", ClipperUtils::clip_clipper_polygons_with_subject_bbox(a.ex, bbox, true));
        R("ccps:Exs:1", ClipperUtils::clip_clipper_polygons_with_subject_bbox(a.exs, bbox, true));
    }
    if (g_out != stdout)
        std::fclose(g_out);
    return 0;
}
