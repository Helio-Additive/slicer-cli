// Generator of tests/data/ex_polygon/vectors.txt: the pin's ExPolygon (ExPolygon.hpp / ExPolygon.cpp) member
// and free functions on deterministic inputs (an LCG, no rand()).
//
// Build with the engine's own compile and link line, run without arguments, output on stdout.
//
// One line per case, tokens separated by one space. Integers are decimal, doubles are the 16 hex digits of
// their bits. A point is `x y`, a point list `n` and n points, an expolygon its contour's point list, the
// number of holes and each hole's point list, an expolygon list `n` and n expolygons. A line is
//   c <ex> <other> <p> <line.a> <line.b> <polyline: point list> <tolerance> <eps> <factor> <v>
//     -> <area> <is_valid> <contains(p, true)> <contains(p, false)> <on_boundary(p, eps)>
//        <point_projection(p)> <contains(line)> <contains(polyline)> <contains({polyline, line})>
//        <overlaps(other)> <simplify_p(tolerance): n, n point lists> <simplify(tolerance): expolygon list>
//        <douglas_peucker(tolerance): expolygon> <lines(): n, a b each> <translate(v): expolygon>
//        <scale(factor): expolygon> <remove_colinear_points(): removed, expolygon>
//        <count_points(ex)> <count_points({ex, other})> <number_polygons({ex, other})>
//        <to_lines({ex, other}): n, a b each> <to_points({ex, other}): point list>
//        <to_polygons({ex, other}): n, n point lists> <get_extents(ex): min max defined>
//        <get_extents({ex, other}): min max defined> <area({ex, other})>
//        <expolygons_simplify({ex, other}, tolerance): expolygon list>
//        <expolygons_contain({other, ex}, p, true)> <expolygons_contain({other, ex}, p, false)>
//        <overlaps({other, ex}, ex)> <has_duplicate_points(ex)> <get_expolygons_area({ex, other})>
//        <symmetric_y(v.y): expolygon> <projection_onto({ex, other}, p)> <has_duplicate_points({ex, other})>
//        <scale(factor, 2.25 - factor): expolygon>
//        <split_expoly_with_holes(range / 20, {other}): expolygon list>
//        <get_extents_vector({ex, other}): n, min max defined each>
//        <remove_small_and_small_holes({ex, other}, 0.02 * range^2): modified, expolygon list>
//        <keep_largest_contour_only({other, ex}): expolygon list>
// with range one of 60, 3000, 400000, 2000000 by the case number modulo 4.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <algorithm>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"

using namespace Slic3r;

static uint64_t g_state = 0x9E3779B97F4A7C15ull;
static uint32_t lcg()
{
    g_state = g_state * 6364136223846793005ull + 1442695040888963407ull;
    return (uint32_t)(g_state >> 33);
}
// An integer in [-range, range].
static int32_t rnd(int32_t range) { return (int32_t)(lcg() % (uint32_t)(2 * range + 1)) - range; }
static double  unit() { return (double)lcg() / 2147483648.0; }
static void pd(double v)
{
    uint64_t b;
    std::memcpy(&b, &v, 8);
    std::printf(" %016llx", (unsigned long long)b);
}
static void pp(const Point &p) { std::printf(" %d %d", (int)p.x(), (int)p.y()); }
static void pps(const Points &pts)
{
    std::printf(" %d", (int)pts.size());
    for (const Point &p : pts) pp(p);
}
static void ppolys(const Polygons &polys)
{
    std::printf(" %d", (int)polys.size());
    for (const Polygon &p : polys) pps(p.points);
}
static void pex(const ExPolygon &ex)
{
    pps(ex.contour.points);
    ppolys(ex.holes);
}
static void pexs(const ExPolygons &exs)
{
    std::printf(" %d", (int)exs.size());
    for (const ExPolygon &ex : exs) pex(ex);
}
static void plines(const Lines &lines)
{
    std::printf(" %d", (int)lines.size());
    for (const Line &l : lines) { pp(l.a); pp(l.b); }
}
static void pbb(const BoundingBox &bb)
{
    pp(bb.min); pp(bb.max);
    std::printf(" %d", bb.defined ? 1 : 0);
}

// A star-shaped ring around `c`: angles in order, radii in [rmin, rmax]; counter-clockwise unless `cw`.
static Points star(const Point &c, int n, double rmin, double rmax, bool cw)
{
    Points pts;
    for (int i = 0; i < n; ++i) {
        const double a = 2. * M_PI * (double(i) + 0.8 * unit()) / double(n);
        const double r = rmin + (rmax - rmin) * unit();
        pts.emplace_back(c.x() + int32_t(r * std::cos(a)), c.y() + int32_t(r * std::sin(a)));
    }
    if (cw) std::reverse(pts.begin(), pts.end());
    return pts;
}

static ExPolygon make_ex(int icase, int32_t range, const Point &c)
{
    ExPolygon ex;
    if (icase % 5 == 4) {
        // Any ring, self-intersecting or not, either orientation, no holes.
        const int n = 3 + int(lcg() % 6);
        for (int i = 0; i < n; ++i) ex.contour.points.emplace_back(c.x() + rnd(range), c.y() + rnd(range));
        return ex;
    }
    ex.contour.points = star(c, 5 + int(lcg() % 8), 0.6 * double(range), double(range), false);
    const int nholes = int(lcg() % 3);
    for (int h = 0; h < nholes; ++h) {
        const Point hc(c.x() + (h == 0 ? 1 : -1) * int32_t(0.22 * double(range)), c.y() + rnd(range / 10));
        Polygon hole;
        hole.points = star(hc, 3 + int(lcg() % 5), 0.05 * double(range), 0.16 * double(range), true);
        ex.holes.emplace_back(std::move(hole));
    }
    return ex;
}

int main()
{
    const int32_t ranges[4] = { 60, 3000, 400000, 2000000 };
    for (int icase = 0; icase < 2000; ++icase) {
        const int32_t range = ranges[icase % 4];
        const ExPolygon ex    = make_ex(icase, range, Point(0, 0));
        const ExPolygon other = make_ex(icase + 1, range / 2 + 1, Point(rnd(range), rnd(range)));
        Point p(rnd(range), rnd(range));
        Line  line(Point(rnd(range), rnd(range)), Point(rnd(range), rnd(range)));
        Polyline polyline;
        for (int i = 0; i < 3; ++i) polyline.points.emplace_back(rnd(range / 3), rnd(range / 3));
        const double tolerance = unit() * 0.1 * double(range);
        const double eps       = 1. + unit() * 0.02 * double(range);
        const double factor    = 0.25 + 2. * unit();
        const Point  v(rnd(range), rnd(range));
        const size_t nc = ex.contour.points.size();
        if (icase % 7 == 3) p = ex.contour.points[nc / 2];
        if (icase % 7 == 5) p = Point((ex.contour.points[0].x() + ex.contour.points[1].x()) / 2, (ex.contour.points[0].y() + ex.contour.points[1].y()) / 2);
        if (icase % 7 == 6 && ! ex.holes.empty()) p = ex.holes.front().points[1];
        if (icase % 3 == 1) line = Line(Point(rnd(range / 4), rnd(range / 4)), Point(rnd(range / 4), rnd(range / 4)));

        std::printf("c");
        pex(ex); pex(other); pp(p); pp(line.a); pp(line.b); pps(polyline.points);
        pd(tolerance); pd(eps); pd(factor); pp(v);
        std::printf(" ->");

        pd(ex.area());
        std::printf(" %d", ex.is_valid() ? 1 : 0);
        std::printf(" %d %d", ex.contains(p, true) ? 1 : 0, ex.contains(p, false) ? 1 : 0);
        std::printf(" %d", ex.on_boundary(p, eps) ? 1 : 0);
        pp(ex.point_projection(p));
        std::printf(" %d", ex.contains(line) ? 1 : 0);
        std::printf(" %d", ex.contains(polyline) ? 1 : 0);
        {
            Polylines pls;
            pls.emplace_back(polyline);
            pls.emplace_back(Polyline(line.a, line.b));
            std::printf(" %d", ex.contains(pls) ? 1 : 0);
        }
        std::printf(" %d", ex.overlaps(other) ? 1 : 0);
        ppolys(ex.simplify_p(tolerance));
        pexs(ex.simplify(tolerance));
        {
            ExPolygon dp(ex);
            dp.douglas_peucker(tolerance);
            pex(dp);
        }
        plines(ex.lines());
        {
            ExPolygon t(ex);
            t.translate(v);
            pex(t);
        }
        {
            ExPolygon s(ex);
            s.scale(factor);
            pex(s);
        }
        {
            ExPolygon r(ex);
            const bool removed = r.remove_colinear_points();
            std::printf(" %d", removed ? 1 : 0);
            pex(r);
        }
        ExPolygons exs;
        exs.emplace_back(ex);
        exs.emplace_back(other);
        ExPolygons sxe;
        sxe.emplace_back(other);
        sxe.emplace_back(ex);
        std::printf(" %d %d %d", (int)count_points(ex), (int)count_points(exs), (int)number_polygons(exs));
        plines(to_lines(exs));
        pps(to_points(exs));
        ppolys(to_polygons(exs));
        pbb(get_extents(ex));
        pbb(get_extents(exs));
        pd(area(exs));
        pexs(expolygons_simplify(exs, tolerance));
        std::printf(" %d %d", expolygons_contain(sxe, p, true) ? 1 : 0, expolygons_contain(sxe, p, false) ? 1 : 0);
        std::printf(" %d", overlaps(sxe, ex) ? 1 : 0);
        std::printf(" %d", has_duplicate_points(ex) ? 1 : 0);
        pd(get_expolygons_area(exs));
        {
            ExPolygon s(ex);
            const coord_t y_axis = v.y();
            s.symmetric_y(y_axis);
            pex(s);
        }
        pp(projection_onto(exs, p));
        std::printf(" %d", has_duplicate_points(exs) ? 1 : 0);
        {
            ExPolygon s(ex);
            s.scale(factor, 2.25 - factor);
            pex(s);
        }
        {
            ExPolygons collision;
            collision.emplace_back(other);
            pexs(ex.split_expoly_with_holes(coord_t(range / 20), collision));
        }
        {
            const std::vector<BoundingBox> bbs = get_extents_vector(exs);
            std::printf(" %d", (int)bbs.size());
            for (const BoundingBox &bb : bbs) pbb(bb);
        }
        {
            ExPolygons r(exs);
            const bool modified = remove_small_and_small_holes(r, 0.02 * double(range) * double(range));
            std::printf(" %d", modified ? 1 : 0);
            pexs(r);
        }
        {
            ExPolygons k(sxe);
            keep_largest_contour_only(k);
            pexs(k);
        }
        std::printf("\n");
    }
    return 0;
}
