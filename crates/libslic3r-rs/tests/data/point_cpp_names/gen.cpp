// Native vectors for tests/point_cpp_names_native_parity.rs: Point.hpp / Point.cpp functions under
// their C++ names (align_to_grid x4, scaled x4, unscaled x3, has_duplicate_points(const&),
// Point::rotate x3, Point::rotated x3, Point::ccw x2, Point::new_scale x3, Point::any_comp x2,
// Point::nearest_point_index x3). Doubles are printed as their 64-bit patterns, floats as 32-bit.
#include "libslic3r/Point.hpp"
#include "libslic3r/Line.hpp"
#include <cstdio>
#include <cstring>
#include <cstdint>

using namespace Slic3r;

static uint64_t g_s = 0x2545F4914F6CDD1DULL;
static uint32_t rnd() { g_s = g_s * 6364136223846793005ULL + 1442695040888963407ULL; return uint32_t(g_s >> 33); }
static int32_t rint_(int32_t lo, int32_t hi) { return lo + int32_t(rnd() % uint32_t(hi - lo + 1)); }
static double rdbl(double lo, double hi) { return lo + (hi - lo) * (double(rnd()) / 2147483648.0); }
static unsigned long long db(double d) { unsigned long long u; std::memcpy(&u, &d, 8); return u; }
static unsigned fb(float f) { unsigned u; std::memcpy(&u, &f, 4); return u; }

int main()
{
    // align_to_grid
    for (int i = 0; i < 4000; ++i) {
        coord_t c = rint_(-2000000, 2000000), s = rint_(1, 50000), b = rint_(-300000, 300000);
        coord_t c2 = rint_(-2000000, 2000000), s2 = rint_(1, 50000), b2 = rint_(-300000, 300000);
        Point p1 = align_to_grid(Point(c, c2), Point(s, s2));
        Point p2 = align_to_grid(Point(c, c2), Point(s, s2), Point(b, b2));
        printf("AG %d %d %d %d %d %d %d %d %d %d %d %d\n", c, s, b, c2, s2, b2,
               align_to_grid(c, s), align_to_grid(c, s, b), p1.x(), p1.y(), p2.x(), p2.y());
    }
    // scaled / unscaled
    const double lits[] = {0.5, 1., 0.005, 0.01, 0.02, 2., 10., 0.1, 0.2, 0.4, 0.42, 0.45, 1e-4, 3., 0.3, 0.15};
    for (int i = 0; i < 4000; ++i) {
        double v = i < 16 ? lits[i] : (i % 3 == 0 ? double(rint_(-200000, 200000)) / 1000. : rdbl(-500., 500.));
        double w = rdbl(-500., 500.);
        float vf = float(v);
        double sd = scaled<double>(v);
        float sf = scaled<float>(vf);
        coord_t sc = scaled<coord_t>(v);
        Point sp = scaled<coord_t>(Vec2d(v, w));
        printf("SC %016llx %016llx %016llx %08x %d %d %d\n", db(v), db(w), db(sd), fb(sf), sc, sp.x(), sp.y());
        coord_t k = rint_(-2000000000, 2000000000), k2 = rint_(-200000000, 200000000);
        double ud = unscaled<double>(k);
        float uf = unscaled<float>(k);
        Vec2d uv = unscaled(Point(k, k2));
        printf("US %d %d %016llx %08x %016llx %016llx\n", k, k2, db(ud), fb(uf), db(uv.x()), db(uv.y()));
        Point n1 = Point::new_scale(v, w);
        Point n2 = Point::new_scale(Vec2d(w, v));
        Point n3 = Point::new_scale(Vec2f(float(v), float(w)));
        printf("NS %016llx %016llx %d %d %d %d %d %d\n", db(v), db(w), n1.x(), n1.y(), n2.x(), n2.y(), n3.x(), n3.y());
    }
    // has_duplicate_points(const&)
    for (int i = 0; i < 2000; ++i) {
        int n = rint_(0, 12);
        Points pts;
        for (int j = 0; j < n; ++j)
            pts.emplace_back(rint_(-6, 6), rint_(-6, 6));
        const Points &cpts = pts;
        printf("HD %d", n);
        for (const Point &p : pts) printf(" %d %d", p.x(), p.y());
        printf(" %d\n", int(has_duplicate_points(cpts)));
    }
    // rotate / rotated / ccw / any_comp
    for (int i = 0; i < 6000; ++i) {
        Point p(rint_(-30000000, 30000000), rint_(-30000000, 30000000));
        Point c(rint_(-30000000, 30000000), rint_(-30000000, 30000000));
        Point q(rint_(-30000000, 30000000), rint_(-30000000, 30000000));
        double a = rdbl(-7., 7.);
        double ca = std::cos(a), sa = std::sin(a);
        Point r1 = p; r1.rotate(a);
        Point r2 = p; r2.rotate(ca, sa);
        Point r3 = p; r3.rotate(a, c);
        Point d1 = p.rotated(a), d2 = p.rotated(ca, sa), d3 = p.rotated(a, c);
        double cc = p.ccw(c, q);
        double cl = p.ccw(Line(c, q));
        coord_t val = rint_(-30000000, 30000000);
        printf("RO %d %d %d %d %d %d %016llx %016llx %016llx %d %d %d %d %d %d %d %d %d %d %d %d %016llx %016llx %d %d %d %d %d\n",
               p.x(), p.y(), c.x(), c.y(), q.x(), q.y(), db(a), db(ca), db(sa),
               r1.x(), r1.y(), r2.x(), r2.y(), r3.x(), r3.y(), d1.x(), d1.y(), d2.x(), d2.y(), d3.x(), d3.y(),
               db(cc), db(cl), val,
               int(p.any_comp(q, ">")), int(p.any_comp(q, "<")), int(p.any_comp(val, ">")), int(p.any_comp(val, "<")));
    }
    // nearest_point_index
    for (int i = 0; i < 3000; ++i) {
        int n = rint_(0, 20);
        Point p(rint_(-1000, 1000), rint_(-1000, 1000));
        Points pts;
        for (int j = 0; j < n; ++j)
            pts.emplace_back(rint_(-1000, 1000), rint_(-1000, 1000));
        if (n > 3 && i % 5 == 0) pts[n / 2] = p;
        PointConstPtrs cp; PointPtrs pp;
        for (Point &q : pts) { cp.push_back(&q); pp.push_back(&q); }
        printf("NP %d %d %d", p.x(), p.y(), n);
        for (const Point &q : pts) printf(" %d %d", q.x(), q.y());
        printf(" %d %d %d\n", p.nearest_point_index(pts), p.nearest_point_index(cp), p.nearest_point_index(pp));
    }
    return 0;
}
