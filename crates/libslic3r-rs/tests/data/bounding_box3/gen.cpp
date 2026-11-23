// Generator of tests/data/bounding_box3/vectors.txt: the pin's BoundingBoxf3 (BoundingBox3Base<Vec3d>,
// BoundingBox.hpp:105-161, 240-258 and BoundingBox.cpp:117-255) on deterministic inputs (an LCG, no rand()).
//
// Build with the engine's own compile and link line, run without arguments, output on stdout.
//
// One line per case, tokens separated by one space. Integers are decimal, doubles are the 16 hex digits of
// their bits. A line is
//   d <kind> <n> <n points> <m> <m points> <q> <delta> <v>
//     -> <bb: box> <bb2: box> <size(bb)> <radius> <center> <max_size> <volume> <empty(bb)>
//        <contains(q)> <contains(bb2)> <intersects(bb2)> <merge(q): box> <merge(bb2): box>
//        <merge(points2): box> <offset(delta): min max> <inflated(delta): min max> <translate(v): min max>
//        <polygon(false): 4 points> <polygon(true): 4 points>
// where a point is `x y z` (doubles), a polygon point is `x y` (integers) and a box is `defined min max`.
// kind (case % 8): 0 bb, bb2 from the points; 1 the points of bb share z (not defined); 2 bb2 from
// (pmin, pmax) = (points2 front, points2 back), maybe inverted; 3 bb2 = BoundingBoxf3(); 4 bb = BoundingBoxf3();
// 5 q = bb.max; 6 points2 = the first half of points; 7 points2 = points + q.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/BoundingBox.hpp"

using namespace Slic3r;

static uint64_t g_state = 0xD1B54A32D192ED03ull;
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
static void p3(const Vec3d &p) { pd(p.x()); pd(p.y()); pd(p.z()); }
static void pp(const Point &p) { std::printf(" %lld %lld", (long long)p.x(), (long long)p.y()); }
static void pb(const BoundingBoxf3 &b)
{
    std::printf(" %d", b.defined ? 1 : 0);
    p3(b.min); p3(b.max);
}
// A coordinate with an integer part in [-range, range] and a fraction off any grid, so that products are
// rounded and the fma shape of radius() shows (no negative zero: the fraction is >= 0).
static double crd(int32_t range) { return double(rnd(range)) + unit() * 0.999; }

int main()
{
    const int32_t ranges[4] = { 3, 200, 5000, 300000 };
    for (int icase = 0; icase < 4000; ++icase) {
        const int32_t range = ranges[icase % 4];
        const int     kind  = icase % 8;
        const int n = 1 + int(lcg() % 7);
        const int m = 1 + int(lcg() % 5);
        Pointf3s pts, pts2;
        for (int i = 0; i < n; ++i) pts.emplace_back(crd(range), crd(range), crd(range));
        for (int i = 0; i < m; ++i) pts2.emplace_back(crd(range), crd(range), crd(range));
        Vec3d q(crd(range), crd(range), crd(range));
        double delta = (unit() - 0.5) * double(range);
        const Vec3d v(crd(range), crd(range), crd(range));
        if (kind == 1) for (Vec3d &p : pts) p.z() = pts.front().z();
        if (kind == 6) pts2.assign(pts.begin(), pts.begin() + (n + 1) / 2);
        if (kind == 7) { pts2 = pts; pts2.emplace_back(q); }
        BoundingBoxf3 bb  = kind == 4 ? BoundingBoxf3() : BoundingBoxf3(pts);
        BoundingBoxf3 bb2 = kind == 2 ? BoundingBoxf3(pts2.front(), pts2.back()) :
                            kind == 3 ? BoundingBoxf3() : BoundingBoxf3(pts2);
        if (kind == 5) q = bb.max;

        std::printf("d %d %d", kind, (int)pts.size());
        for (const Vec3d &p : pts) p3(p);
        std::printf(" %d", (int)pts2.size());
        for (const Vec3d &p : pts2) p3(p);
        p3(q); pd(delta); p3(v);
        std::printf(" ->");

        pb(bb);
        pb(bb2);
        p3(bb.size());
        pd(bb.radius());
        p3(bb.center());
        pd(bb.max_size());
        pd(bb.volume());
        std::printf(" %d %d %d %d", empty(bb) ? 1 : 0, bb.contains(q) ? 1 : 0, bb.contains(bb2) ? 1 : 0,
                    bb.intersects(bb2) ? 1 : 0);
        { BoundingBoxf3 b(bb); b.merge(q); pb(b); }
        { BoundingBoxf3 b(bb); b.merge(bb2); pb(b); }
        { BoundingBoxf3 b(bb); b.merge(pts2); pb(b); }
        { BoundingBoxf3 b(bb); b.offset(delta); p3(b.min); p3(b.max); }
        { const BoundingBox3Base<Vec3d> b = bb.inflated(delta); p3(b.min); p3(b.max); }
        { BoundingBoxf3 b(bb); b.translate(v); p3(b.min); p3(b.max); }
        { const Polygon poly = bb.polygon(false); for (const Point &p : poly.points) pp(p); }
        { const Polygon poly = bb.polygon(true); for (const Point &p : poly.points) pp(p); }
        std::printf("\n");
    }
    return 0;
}
