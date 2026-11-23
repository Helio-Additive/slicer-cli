// Generator of tests/data/polygon_rest/vectors.txt: the rest of the pin's Polygon (Polygon.hpp /
// Polygon.cpp) not in tests/data/polygon, on deterministic inputs (an LCG, no rand()).
//
// Build with the engine's own compile and link line, run without arguments, output on stdout.
//
// One line per case, tokens separated by one space. Integers are decimal, doubles are the 16 hex digits of
// their bits, floats the 8 hex digits of theirs. A line is
//   e <poly: n points> <other: n points> <line.a> <line.b> <p> <eps> <tolerance> <angle> <min_length>
//     <min_area> <radius> <error> <segments>
//     -> <split_at_vertex(poly[case % n]): n points> <split_at_index(k): n points> <split_at_first_point: n points>
//        <is_valid> <contains(p)> <on_boundary(p, eps)> <simplify(tolerance): n polygons, n points each>
//        <intersection(line): found x y> <overlaps({other})> <parameter_by_length: n floats>
//        <densify(min_length): n points, n lengths (floats)> <has_duplicate_points({poly, other})>
//        <remove_same_neighbor(poly): removed n points> <remove_degenerate({poly, other}): removed n polygons>
//        <remove_small({poly, other}, min_area): removed n polygons> <polygons_simplify({poly, other}): ...>
//        <polygons_match(poly, other)> <overlaps({poly}, {other})> <get_extents_rotated(poly, angle): box>
//        <make_circle(radius, error): n points> <make_circle_num_segments(radius, segments): n points>
//        <total_length({poly, other})> <area({poly, other})>
// where a point is `x y`, a box `defined min max`, and k = n / 2 (poly).
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/BoundingBox.hpp"

using namespace Slic3r;

static uint64_t g_state = 0x2545F4914F6CDD1Dull;
static uint32_t lcg()
{
    g_state = g_state * 6364136223846793005ull + 1442695040888963407ull;
    return (uint32_t)(g_state >> 33);
}
static int32_t rnd(int32_t range) { return (int32_t)(lcg() % (uint32_t)(2 * range + 1)) - range; }
static double  unit() { return (double)lcg() / 2147483648.0; }
static void pd(double v)
{
    uint64_t b;
    std::memcpy(&b, &v, 8);
    std::printf(" %016llx", (unsigned long long)b);
}
static void pf(float v)
{
    uint32_t b;
    std::memcpy(&b, &v, 4);
    std::printf(" %08x", (unsigned)b);
}
static void pp(const Point &p) { std::printf(" %d %d", (int)p.x(), (int)p.y()); }
static void pps(const Points &pts)
{
    std::printf(" %d", (int)pts.size());
    for (const Point &p : pts) pp(p);
}
static void ppolys(const Polygons &ps)
{
    std::printf(" %d", (int)ps.size());
    for (const Polygon &p : ps) pps(p.points);
}
static Points make_poly(int32_t range, int kind)
{
    const int n = 3 + int(lcg() % 9);
    Points pts;
    if (kind == 0) {
        for (int i = 0; i < n; ++i) {
            const double a = 2. * M_PI * (double(i) + 0.8 * unit()) / double(n);
            const double r = double(range) * (0.3 + 0.7 * unit());
            pts.emplace_back(int32_t(r * std::cos(a)), int32_t(r * std::sin(a)));
        }
    } else {
        for (int i = 0; i < n; ++i) pts.emplace_back(rnd(range), rnd(range));
    }
    // Repeated neighbours and a repeated first point for remove_same_neighbor / has_duplicate_points.
    if (lcg() % 4 == 0) pts.insert(pts.begin() + (lcg() % pts.size()), pts[lcg() % pts.size()]);
    if (lcg() % 5 == 0) { const size_t k = lcg() % pts.size(); pts.insert(pts.begin() + k, pts[k]); }
    if (lcg() % 6 == 0) pts.push_back(pts.front());
    return pts;
}

int main()
{
    const int32_t ranges[4] = { 20, 3000, 400000, 2000000 };
    for (int icase = 0; icase < 2000; ++icase) {
        const int32_t range = ranges[icase % 4];
        Polygon poly(make_poly(range, icase % 3 == 0 ? 0 : 1));
        Polygon other(make_poly(range, icase % 2));
        if (icase % 11 == 4) other = poly;
        if (icase % 11 == 7 && poly.size() > 1) { other.points = poly.points; std::rotate(other.points.begin(), other.points.begin() + 1, other.points.end()); }
        Line   line(Point(rnd(range), rnd(range)), Point(rnd(range), rnd(range)));
        Point  p(rnd(range), rnd(range));
        if (icase % 7 == 3) line = Line(poly.points[0], poly.points[poly.size() / 2]);
        if (icase % 7 == 5) p = poly.points[poly.size() - 1];
        if (icase % 7 == 6) p = (poly.points[0] + poly.points[1]) / 2;
        const double eps        = unit() * 0.05 * double(range);
        const double tolerance  = unit() * 0.3 * double(range);
        const double angle      = (unit() - 0.5) * 4. * M_PI;
        const float  min_length = float((0.1 + 0.5 * unit()) * double(range));
        const double min_area   = unit() * unit() * double(range) * double(range);
        const double radius     = (0.1 + unit()) * double(range);
        const double error      = (0.01 + 0.1 * unit()) * radius;
        const size_t segments   = 3 + lcg() % 60;

        std::printf("e");
        pps(poly.points); pps(other.points); pp(line.a); pp(line.b); pp(p);
        pd(eps); pd(tolerance); pd(angle); pf(min_length); pd(min_area); pd(radius); pd(error);
        std::printf(" %d ->", (int)segments);

        pps(poly.split_at_vertex(poly.points[size_t(icase) % poly.size()]).points);
        pps(poly.split_at_index(int(poly.size() / 2)).points);
        pps(poly.split_at_first_point().points);
        std::printf(" %d %d %d", poly.is_valid() ? 1 : 0, poly.contains(p) ? 1 : 0, poly.on_boundary(p, eps) ? 1 : 0);
        ppolys(poly.simplify(tolerance));
        {
            Point x(0, 0);
            const bool ok = poly.intersection(line, &x);
            std::printf(" %d", ok ? 1 : 0);
            pp(x);
        }
        std::printf(" %d", poly.overlaps(Polygons{ other }) ? 1 : 0);
        {
            const std::vector<float> l = poly.parameter_by_length();
            std::printf(" %d", (int)l.size());
            for (float v : l) pf(v);
        }
        {
            Polygon d(poly);
            // densify reads the parametrization it is given (Polygon.cpp:357-382): pass the polygon's own.
            std::vector<float> l = d.parameter_by_length();
            d.densify(min_length, &l);
            pps(d.points);
            std::printf(" %d", (int)l.size());
            for (float v : l) pf(v);
        }
        std::printf(" %d", has_duplicate_points(Polygons{ poly, other }) ? 1 : 0);
        {
            Polygon r(poly);
            const bool removed = remove_same_neighbor(r);
            std::printf(" %d", removed ? 1 : 0);
            pps(r.points);
        }
        {
            Polygons ps{ poly, other };
            if (icase % 5 == 2) ps.emplace_back(Points{ poly.points.front(), poly.points.back() });
            const bool removed = remove_degenerate(ps);
            std::printf(" %d", removed ? 1 : 0);
            ppolys(ps);
        }
        {
            Polygons ps{ poly, other };
            const bool removed = remove_small(ps, min_area);
            std::printf(" %d", removed ? 1 : 0);
            ppolys(ps);
        }
        ppolys(polygons_simplify(Polygons{ poly, other }, tolerance));
        std::printf(" %d %d", polygons_match(poly, other) ? 1 : 0, overlaps(Polygons{ poly }, Polygons{ other }) ? 1 : 0);
        {
            const BoundingBox b = get_extents_rotated(poly, angle);
            std::printf(" %d", b.defined ? 1 : 0);
            pp(b.min); pp(b.max);
        }
        pps(make_circle(radius, error).points);
        pps(make_circle_num_segments(radius, segments).points);
        pd(total_length(Polygons{ poly, other }));
        pd(area(Polygons{ poly, other }));
        std::printf("\n");
    }
    return 0;
}
