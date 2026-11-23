// Generator of tests/data/line/vectors.txt: the pin's Line.hpp / Line.cpp member functions and the free
// get_extents(Lines) on deterministic inputs (an LCG, no rand()).
//
// Build with the engine's own compile and link line, run without arguments, output on stdout.
//
// One line per case, tokens separated by one space. Integers are decimal, doubles are the 16 hex digits of
// their bits. A line is
//   c <l.a> <l.b> <m.a> <m.b> <p> <angle> <offset> <bbox.min> <bbox.max>
//     -> <length> <midpoint> <atan2_> <orientation> <direction> <vector> <normal>
//        <distance_to_squared(p)> <distance_to(p)> <distance_to_squared(p, a, b)> <distance_to(p, a, b)>
//        <distance_to_infinite_squared(p, a, b)> <distance_to_infinite(p, a, b)> <perp_distance_to(p)>
//        <parallel_to(m)> <perpendicular_to(m)> <parallel_to(angle)> <perpendicular_to(angle)>
//        <intersection(m): found x y> <intersection_infinite(m): found x y> <overlap(m): found length>
//        <clip_with_bbox: found a b> <extend(offset): a b> <get_extents({l, m}): defined min max>
// where a point is `x y`. A point or a length the pin leaves untouched is printed as it was initialised (zero).
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/BoundingBox.hpp"

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
        Line  l(Point(rnd(range), rnd(range)), Point(rnd(range), rnd(range)));
        Line  m(Point(rnd(range), rnd(range)), Point(rnd(range), rnd(range)));
        Point p(rnd(range), rnd(range));
        double angle  = (unit() - 0.5) * 2. * M_PI;
        const double offset = (unit() - 0.25) * double(range);
        Point bmin(rnd(range), rnd(range));
        Point bmax(bmin.x() + 1 + int32_t(lcg() % uint32_t(range)), bmin.y() + 1 + int32_t(lcg() % uint32_t(range)));
        const int k = 1 + int(lcg() % 4);
        // Variants that reach the branches: parallel, collinear, perpendicular and degenerate lines, a point on
        // the line, an angle equal to the line's.
        switch (icase % 12) {
        case 1: m = Line(m.a, m.a + (l.b - l.a)); break;                                   // parallel
        case 2: m = Line(l.a + (l.b - l.a) * k, l.b + (l.b - l.a) * (k + 1)); break;       // collinear, beside
        case 3: m = Line(l.a, l.a + (l.b - l.a) * k); break;                               // collinear, overlapping
        case 4: m = Line(m.a, m.a + Point(-(l.b.y() - l.a.y()), l.b.x() - l.a.x())); break; // perpendicular
        case 5: l.b = l.a; break;                                                          // a point
        case 6: p = l.a + (l.b - l.a) * k; break;                                          // on the infinite line
        case 7: p = Point((l.a.x() + l.b.x()) / 2, (l.a.y() + l.b.y()) / 2); break;        // near the middle
        case 8: angle = l.direction(); break;
        case 9: angle = l.direction() + 0.5 * M_PI; break;
        case 10: l.b.y() = l.a.y(); m = Line(Point(l.a.x() + k, l.a.y()), Point(l.b.x() + k, l.b.y())); break; // horizontal, overlapping
        case 11: l.b.x() = l.a.x(); m.b.x() = m.a.x() = l.a.x(); break;                    // vertical, collinear
        default: break;
        }
        const BoundingBox bbox(bmin, bmax);

        std::printf("c");
        pp(l.a); pp(l.b); pp(m.a); pp(m.b); pp(p); pd(angle); pd(offset); pp(bbox.min); pp(bbox.max);
        std::printf(" ->");

        pd(l.length());
        pp(l.midpoint());
        pd(l.atan2_());
        pd(l.orientation());
        pd(l.direction());
        pp(l.vector());
        pp(l.normal());
        pd(l.distance_to_squared(p));
        pd(l.distance_to(p));
        pd(Line::distance_to_squared(p, l.a, l.b));
        pd(Line::distance_to(p, l.a, l.b));
        pd(Line::distance_to_infinite_squared(p, l.a, l.b));
        pd(Line::distance_to_infinite(p, l.a, l.b));
        pd(l.perp_distance_to(p));
        std::printf(" %d %d %d %d", l.parallel_to(m) ? 1 : 0, l.perpendicular_to(m) ? 1 : 0, l.parallel_to(angle) ? 1 : 0,
                    l.perpendicular_to(angle) ? 1 : 0);
        {
            Point x(0, 0);
            const bool ok = l.intersection(m, &x);
            std::printf(" %d", ok ? 1 : 0);
            pp(ok ? x : Point(0, 0));
        }
        {
            Point x(0, 0);
            const bool ok = l.intersection_infinite(m, &x);
            std::printf(" %d", ok ? 1 : 0);
            pp(ok ? x : Point(0, 0));
        }
        {
            double len = 0.;
            const bool ok = l.overlap(m, len);
            std::printf(" %d", ok ? 1 : 0);
            pd(ok ? len : 0.);
        }
        {
            Line c(l);
            const bool ok = c.clip_with_bbox(bbox);
            std::printf(" %d", ok ? 1 : 0);
            pp(c.a); pp(c.b);
        }
        {
            Line e(l);
            e.extend(offset);
            pp(e.a); pp(e.b);
        }
        {
            Lines lines;
            lines.push_back(l);
            lines.push_back(m);
            const BoundingBox bb = get_extents(lines);
            std::printf(" %d", bb.defined ? 1 : 0);
            pp(bb.min); pp(bb.max);
        }
        std::printf("\n");
    }
    return 0;
}
