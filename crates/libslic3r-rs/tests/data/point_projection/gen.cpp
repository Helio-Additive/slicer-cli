// Native vectors for tests/point_projection_native_parity.rs: the pin's Point::projection_onto(const Line&)
// (Point.cpp:157-180), Point::projection_onto(const MultiPoint&) (141-155) on Polygons and Polylines, and
// get_extents<false/true>(const Points&) / get_extents<false/true>(const VecOfPoints&) (251-272).
// Deterministic LCG inputs.
#include "libslic3r/Point.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/BoundingBox.hpp"
#include <cstdio>
#include <cstdint>

using namespace Slic3r;

static uint64_t s = 9521;
static uint32_t rnd() { s = s * 6364136223846793005ULL + 1442695040888963407ULL; return uint32_t(s >> 33); }
static int ri(int lo, int hi) { return lo + int(rnd() % uint32_t(hi - lo + 1)); }

static void pts(const char *tag, const Points &p)
{
    printf("%s %zu", tag, p.size());
    for (const Point &q : p) printf(" %d %d", q.x(), q.y());
    printf("\n");
}
static void bb(const BoundingBox &b) { printf("BB %d %d %d %d %d\n", b.defined ? 1 : 0, b.min.x(), b.min.y(), b.max.x(), b.max.y()); }

int main()
{
    for (int i = 0; i < 4000; ++i) {
        int R = (i % 3 == 0) ? 50 : ((i % 3 == 1) ? 100000 : 20000000);
        Point p(ri(-R, R), ri(-R, R));
        Line l(Point(ri(-R, R), ri(-R, R)), Point(ri(-R, R), ri(-R, R)));
        if (i % 17 == 0) l.b = l.a;
        Point pl = p.projection_onto(l);
        printf("PL %d %d %d %d %d %d %d %d\n", p.x(), p.y(), l.a.x(), l.a.y(), l.b.x(), l.b.y(), pl.x(), pl.y());
        int n = ri(1, 9);
        Points q;
        for (int k = 0; k < n; ++k) q.emplace_back(ri(-R, R), ri(-R, R));
        if (i % 11 == 0 && n > 2) q[2] = q[1];
        Polygon pg(q);
        Polyline py; py.points = q;
        Point a = p.projection_onto(pg), b = p.projection_onto(py);
        pts("Q", q);
        printf("PM %d %d %d %d %d %d\n", p.x(), p.y(), a.x(), a.y(), b.x(), b.y());
        bb(get_extents<false>(q));
        bb(get_extents<true>(q));
        VecOfPoints vp;
        int m = ri(0, 3);
        for (int k = 0; k < m; ++k) {
            Points r;
            int c = ri(0, 4);
            for (int j = 0; j < c; ++j) r.emplace_back(ri(-R, R), ri(-R, R));
            if (c == 2 && i % 2) r[1] = Point(r[0].x(), r[1].y());
            vp.push_back(r);
            pts("W", r);
        }
        printf("WE %d\n", m);
        bb(get_extents<false>(vp));
        bb(get_extents<true>(vp));
    }
    return 0;
}
