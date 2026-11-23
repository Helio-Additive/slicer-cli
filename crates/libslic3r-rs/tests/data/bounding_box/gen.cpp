// Generator of tests/data/bounding_box/vectors.txt: the pin's BoundingBox (BoundingBox.hpp / BoundingBox.cpp,
// the 2D integer box) on deterministic inputs (an LCG, no rand()).
//
// Build with the engine's own compile and link line, run without arguments, output on stdout.
//
// One line per case, tokens separated by one space. Integers are decimal, doubles are the 16 hex digits of
// their bits. A line is
//   c <n> <n points> <m> <m points> <q> <angle> <center> <factor> <delta> <cell>
//     -> <BoundingBox(points): box> <BoundingBox(points2): box> <size> <radius> <area> <center>
//        <contains(q)> <contains(box2)> <overlap(box2)> <merge(q): box> <merge(box2): box>
//        <offset(delta): min max> <inflated(delta): min max> <scale(factor): min max> <scaled(factor): min max>
//        <rotated(angle): box> <rotated(angle, center): box> <polygon(): 4 points>
//        <align_to_grid(cell): min max> <operator[] 0..3: 4 points>
// where a point is `x y` and a box is `defined min max`.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
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
static void pb(const BoundingBox &b)
{
    std::printf(" %d", b.defined ? 1 : 0);
    pp(b.min); pp(b.max);
}

int main()
{
    const int32_t ranges[4] = { 20, 3000, 400000, 20000000 };
    for (int icase = 0; icase < 4000; ++icase) {
        const int32_t range = ranges[icase % 4];
        const int n = 1 + int(lcg() % 7);
        const int m = 1 + int(lcg() % 5);
        Points pts, pts2;
        for (int i = 0; i < n; ++i) pts.emplace_back(rnd(range), rnd(range));
        for (int i = 0; i < m; ++i) pts2.emplace_back(rnd(range), rnd(range));
        Point q(rnd(range), rnd(range));
        const double angle  = (unit() - 0.5) * 4. * M_PI;
        const Point  center(rnd(range), rnd(range));
        const double factor = (unit() - 0.5) * 6.;
        double       delta  = (unit() - 0.5) * double(range);
        const coord_t cell  = 1 + coord_t(lcg() % uint32_t(range));
        // Variants: boxes without an area (not defined), a point on the boundary, a box inside the other,
        // a delta on a half.
        switch (icase % 9) {
        case 1: for (Point &p : pts) p.x() = pts.front().x(); break;
        case 2: for (Point &p : pts2) p.y() = pts2.front().y(); break;
        case 3: q = Point(pts.front().x(), q.y()); break;
        case 4: q = pts.back(); break;
        case 5: pts2.assign(pts.begin(), pts.begin() + (n + 1) / 2); break;
        case 6: delta = std::floor(delta) + 0.5; break;
        case 7: pts2 = pts; pts2.emplace_back(q); break;
        default: break;
        }

        std::printf("c %d", (int)pts.size());
        for (const Point &p : pts) pp(p);
        std::printf(" %d", (int)pts2.size());
        for (const Point &p : pts2) pp(p);
        pp(q); pd(angle); pp(center); pd(factor); pd(delta);
        std::printf(" %d ->", (int)cell);

        const BoundingBox bb(pts);
        const BoundingBox bb2(pts2);
        pb(bb);
        pb(bb2);
        pp(bb.size());
        pd(bb.radius());
        pd(bb.area());
        pp(bb.center());
        std::printf(" %d %d %d", bb.contains(q) ? 1 : 0, bb.contains(bb2) ? 1 : 0, bb.overlap(bb2) ? 1 : 0);
        { BoundingBox b(bb); b.merge(q); pb(b); }
        { BoundingBox b(bb); b.merge(bb2); pb(b); }
        { BoundingBox b(bb); b.offset(delta); pp(b.min); pp(b.max); }
        { const BoundingBox b = bb.inflated(delta); pp(b.min); pp(b.max); }
        { BoundingBox b(bb); b.scale(factor); pp(b.min); pp(b.max); }
        { const BoundingBox b = bb.scaled(factor); pp(b.min); pp(b.max); }
        pb(bb.rotated(angle));
        pb(bb.rotated(angle, center));
        { const Polygon poly = bb.polygon(); for (const Point &p : poly.points) pp(p); }
        { BoundingBox b(bb); b.align_to_grid(cell); pp(b.min); pp(b.max); }
        for (size_t i = 0; i < 4; ++i) pp(bb[i]);
        std::printf("\n");
    }
    return 0;
}
