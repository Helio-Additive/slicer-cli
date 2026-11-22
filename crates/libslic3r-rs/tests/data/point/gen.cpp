// Generator of tests/data/point/vectors.txt: the pin's Point.hpp / Point.cpp member functions and a few
// free functions of the same files on deterministic inputs (an LCG, no rand()).
//
// Build with the engine's own compile and link line, run without arguments, output on stdout.
//
// One line per case, tokens separated by one space. Integers are decimal, doubles are the 16 hex digits of
// their bits. A line is
//   c <p> <a> <b> <angle> <cos> <sin> <sx> <sy> <factor> <n> <n points>
//     -> <rotated(angle, a)> <rotated(cos, sin)> <rotated(angle)> <rotate_90_degree_ccw>
//        <nearest_point_index> <nearest_point: found x y> <ccw(a, b)> <ccw(Line(a, b))> <ccw_angle(a, b)>
//        <projection_onto(Line(a, b))> <projection_onto(Polyline(points))> <projection_onto(Polygon(points))>
//        <is_in_lines(points)> <new_scale(sx, sy)> <both_comp > <> <any_comp > <> <any_comp(a.x) > <>
//        <p < a> <member p * factor> <free p * factor> <has_duplicate_points> <collect_duplicates: n points>
//        <int128::orient(p, a, b)> <int128::cross(a, b)>
// where a point is `x y`.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"

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

int main()
{
    const int32_t ranges[4] = { 20, 3000, 400000, 20000000 };
    for (int icase = 0; icase < 4000; ++icase) {
        const int32_t range = ranges[icase % 4];
        Point p(rnd(range), rnd(range));
        Point a(rnd(range), rnd(range));
        Point b(rnd(range), rnd(range));
        if (icase % 37 == 5) b = a;
        const double angle  = (unit() - 0.5) * 4. * M_PI;
        // Read the cosine and the sine from two different arguments' worth of calls, so that the values the
        // line carries do not depend on how the compiler pairs them.
        volatile double angle_c = angle;
        const double cos_a  = std::cos(angle_c);
        volatile double angle_s = angle;
        const double sin_a  = std::sin(angle_s);
        const double sx     = (unit() - 0.5) * 400.;
        const double sy     = (unit() - 0.5) * 400.;
        const double factor = (unit() - 0.5) * 6.;
        const int    n      = 2 + int(lcg() % 9);
        Points pts;
        for (int i = 0; i < n; ++i)
            pts.emplace_back(rnd(range), rnd(range));
        // Variants that reach the branches of is_in_lines, nearest_point_index and the duplicate tests.
        const int k = int(lcg() % (n - 1));
        switch (icase % 8) {
        case 1: p = pts[k]; break;                                                   // on an end point
        case 2: pts[k + 1].x() = pts[k].x(); p.x() = pts[k].x(); break;              // on or beside a vertical segment
        case 3: pts[k + 1].y() = pts[k].y(); p.y() = pts[k].y(); break;              // on or beside a horizontal one
        case 4: p = Point((pts[k].x() + pts[k + 1].x()) / 2, (pts[k].y() + pts[k + 1].y()) / 2); break; // a middle
        case 5: pts[n - 1] = pts[0]; break;                                          // a duplicate
        case 6: pts[k + 1] = pts[k]; p = Point((pts[0].x() + pts[n - 1].x()) / 2, (pts[0].y() + pts[n - 1].y()) / 2); break;
        default: break;
        }

        std::printf("c");
        pp(p); pp(a); pp(b); pd(angle); pd(cos_a); pd(sin_a); pd(sx); pd(sy); pd(factor);
        std::printf(" %d", n);
        for (const Point &q : pts) pp(q);
        std::printf(" ->");

        pp(p.rotated(angle, a));
        pp(p.rotated(cos_a, sin_a));
        pp(p.rotated(angle));
        pp(p.rotate_90_degree_ccw());
        std::printf(" %d", p.nearest_point_index(pts));
        {
            Point found(0, 0);
            const bool ok = p.nearest_point(pts, &found);
            std::printf(" %d", ok ? 1 : 0);
            pp(found);
        }
        pd(p.ccw(a, b));
        pd(p.ccw(Line(a, b)));
        pd(p.ccw_angle(a, b));
        pp(p.projection_onto(Line(a, b)));
        pp(p.projection_onto(Polyline(pts)));
        pp(p.projection_onto(Polygon(pts)));
        std::printf(" %d", p.is_in_lines(pts) ? 1 : 0);
        pp(Point::new_scale(sx, sy));
        {
            Point q(p);
            std::printf(" %d %d %d %d %d %d", q.both_comp(a, ">") ? 1 : 0, q.both_comp(a, "<") ? 1 : 0, q.any_comp(a, ">") ? 1 : 0,
                        q.any_comp(a, "<") ? 1 : 0, q.any_comp(a.x(), ">") ? 1 : 0, q.any_comp(a.x(), "<") ? 1 : 0);
            std::printf(" %d", (p < a) ? 1 : 0);
            pp(q * factor);                                  // the member, Point.hpp:200
            pp(operator*(static_cast<const Point &>(p), factor)); // the free function, Point.hpp:255-258
        }
        std::printf(" %d", has_duplicate_points(Points(pts)) ? 1 : 0);
        {
            const Points dups = collect_duplicates(pts);
            std::printf(" %d", (int)dups.size());
            for (const Point &q : dups) pp(q);
        }
        std::printf(" %d %d", int128::orient(p, a, b), int128::cross(a, b));
        std::printf("\n");
    }
    return 0;
}
