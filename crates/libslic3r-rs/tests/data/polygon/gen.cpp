// Generator of tests/data/polygon/vectors.txt: the pin's Polygon (Polygon.hpp / Polygon.cpp) member
// functions on deterministic inputs (an LCG, no rand()).
//
// Build with the engine's own compile and link line, run without arguments, output on stdout.
//
// One line per case, tokens separated by one space. Integers are decimal, doubles are the 16 hex digits of
// their bits. A line is
//   c <n> <n points> <line.a> <line.b> <p> <threshold> <tolerance> <spacing> <max_deviation> <max_variance>
//     -> <length> <area> <is_counter_clockwise> <is_clockwise> <centroid> <lines: n, a b each>
//        <first_intersection(line): found x y> <intersections(line): found n points>
//        <convex_points(threshold): n points> <concave_points(threshold): n points> <point_projection(p)>
//        <triangulate_convex: n polygons, each n points> <douglas_peucker(tolerance): n points>
//        <make_counter_clockwise: reversed, n points> <make_clockwise: reversed, n points>
//        <equally_spaced_points(spacing): n points>
//        <polygon_is_convex> <remove_sticks: removed, n points> <remove_collinear: n points>
//        <is_approx_circle(max_deviation, max_variance): ok [center diameter]>   (R931)
// where a point is `x y`.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Polygon.hpp"

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

int main()
{
    const int32_t ranges[4] = { 20, 3000, 400000, 2000000 };
    for (int icase = 0; icase < 3000; ++icase) {
        const int32_t range = ranges[icase % 4];
        const int n = 3 + int(lcg() % 8);
        Points pts;
        if (icase % 3 == 0) {
            // A star-shaped polygon around the origin: angles in order, random radii.
            for (int i = 0; i < n; ++i) {
                const double a = 2. * M_PI * (double(i) + 0.8 * unit()) / double(n);
                const double r = double(range) * (0.3 + 0.7 * unit());
                pts.emplace_back(int32_t(r * std::cos(a)), int32_t(r * std::sin(a)));
            }
            if (icase % 2 == 0) std::reverse(pts.begin(), pts.end());
        } else {
            for (int i = 0; i < n; ++i) pts.emplace_back(rnd(range), rnd(range));
        }
        Line  line(Point(rnd(range), rnd(range)), Point(rnd(range), rnd(range)));
        Point p(rnd(range), rnd(range));
        const double threshold = (0.25 + 1.5 * unit()) * M_PI;
        const double tolerance = unit() * 0.3 * double(range);
        if (icase % 7 == 3) line = Line(pts[0], pts[n / 2]);
        if (icase % 7 == 5) p = pts[n - 1];

        std::printf("c");
        pps(pts);
        const double spacing = tolerance + 0.25 * double(range);
        const double max_deviation = (0.01 + 0.2 * unit()) * double(range);
        const double max_variance  = (0.01 + 0.2 * unit()) * double(range);
        pp(line.a); pp(line.b); pp(p); pd(threshold); pd(tolerance); pd(spacing); pd(max_deviation); pd(max_variance);
        std::printf(" ->");

        const Polygon poly(pts);
        pd(poly.length());
        pd(poly.area());
        std::printf(" %d %d", poly.is_counter_clockwise() ? 1 : 0, poly.is_clockwise() ? 1 : 0);
        pp(poly.centroid());
        {
            const Lines lines = poly.lines();
            std::printf(" %d", (int)lines.size());
            for (const Line &l : lines) { pp(l.a); pp(l.b); }
        }
        {
            Point x(0, 0);
            const bool ok = poly.first_intersection(line, &x);
            std::printf(" %d", ok ? 1 : 0);
            pp(x);
        }
        {
            Points xs;
            const bool ok = poly.intersections(line, &xs);
            std::printf(" %d", ok ? 1 : 0);
            pps(xs);
        }
        pps(poly.convex_points(threshold));
        pps(poly.concave_points(threshold));
        pp(poly.point_projection(p));
        {
            Polygons tris;
            poly.triangulate_convex(&tris);
            std::printf(" %d", (int)tris.size());
            for (const Polygon &t : tris) pps(t.points);
        }
        {
            Polygon dp(poly);
            dp.douglas_peucker(tolerance);
            pps(dp.points);
        }
        {
            Polygon ccw(poly);
            const bool flipped = ccw.make_counter_clockwise();
            std::printf(" %d", flipped ? 1 : 0);
            pps(ccw.points);
            Polygon cw(poly);
            const bool flipped2 = cw.make_clockwise();
            std::printf(" %d", flipped2 ? 1 : 0);
            pps(cw.points);
        }
        pps(poly.equally_spaced_points(spacing));
        // R931: the free functions of Polygon.cpp and Polygon::is_approx_circle.
        std::printf(" %d", polygon_is_convex(poly) ? 1 : 0);
        {
            Polygon st(poly);
            const bool removed = remove_sticks(st);
            std::printf(" %d", removed ? 1 : 0);
            pps(st.points);
            Polygon co(poly);
            remove_collinear(co);
            pps(co.points);
        }
        {
            Point  center(0, 0);
            double diameter = 0.;
            const bool ok = poly.is_approx_circle(max_deviation, max_variance, center, diameter);
            std::printf(" %d", ok ? 1 : 0);
            if (ok) { pp(center); pd(diameter); }
        }
        std::printf("\n");
    }
    return 0;
}
