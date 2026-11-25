// Native vectors for tests/multi_point_cpp_names_native_parity.rs: the members of class MultiPoint
// (MultiPoint.hpp:17-115, MultiPoint.cpp:6-177) on Polygons and Polylines: scale x2, translate x2,
// rotate x3, reverse, size, empty, is_valid, closest_point_index, closest_point, distance_to, clear,
// append x4, intersection, first_intersection, intersections. Doubles as 64-bit patterns.
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Line.hpp"
#include <cstdio>
#include <cstring>
#include <cstdint>

using namespace Slic3r;

static uint64_t g_s = 0x9E3779B97F4A7C15ULL;
static uint32_t rnd() { g_s = g_s * 6364136223846793005ULL + 1442695040888963407ULL; return uint32_t(g_s >> 33); }
static int32_t ri(int32_t lo, int32_t hi) { return lo + int32_t(rnd() % uint32_t(hi - lo + 1)); }
static double rd(double lo, double hi) { return lo + (hi - lo) * (double(rnd()) / 2147483648.0); }
static unsigned long long db(double d) { unsigned long long u; std::memcpy(&u, &d, 8); return u; }

static void pr(const char *tag, const MultiPoint &mp)
{
    printf(" %s %zu", tag, mp.points.size());
    for (const Point &p : mp.points) printf(" %d %d", p.x(), p.y());
}

static void run(MultiPoint &mp, int kind)
{
    Point q(ri(-6000000, 6000000), ri(-6000000, 6000000));
    Line ln(Point(ri(-6000000, 6000000), ri(-6000000, 6000000)), Point(ri(-6000000, 6000000), ri(-6000000, 6000000)));
    double f = rd(-3., 3.), fx = rd(-3., 3.), fy = rd(-3., 3.), tx = rd(-1e6, 1e6), ty = rd(-1e6, 1e6), a = rd(-7., 7.);
    double ca = cos(a), sa = sin(a);
    Point c(ri(-6000000, 6000000), ri(-6000000, 6000000));
    printf("MP %d", kind);
    pr("in", mp);
    printf(" q %d %d ln %d %d %d %d f %016llx %016llx %016llx %016llx %016llx %016llx %016llx %016llx c %d %d",
           q.x(), q.y(), ln.a.x(), ln.a.y(), ln.b.x(), ln.b.y(), db(f), db(fx), db(fy), db(tx), db(ty), db(a), db(ca), db(sa), c.x(), c.y());
    printf(" | %zu %d %d %d", mp.size(), int(mp.empty()), int(mp.is_valid()), mp.closest_point_index(q));
    const Point *cl = mp.closest_point(q);
    printf(" %d %d %016llx", cl ? cl->x() : 0, cl ? cl->y() : 0, db(mp.empty() ? 0. : mp.distance_to(q)));
    Point ip(0, 0), fp(0, 0);
    Points ips;
    bool b1 = mp.intersection(ln, &ip), b2 = mp.first_intersection(ln, &fp), b3 = mp.intersections(ln, &ips);
    printf(" %d %d %d %d %d %d %d %zu", int(b1), ip.x(), ip.y(), int(b2), fp.x(), fp.y(), int(b3), ips.size());
    for (const Point &p : ips) printf(" %d %d", p.x(), p.y());
    Points orig = mp.points;
    mp.scale(f);                 pr("s1", mp); mp.points = orig;
    mp.scale(fx, fy);            pr("s2", mp); mp.points = orig;
    mp.translate(tx, ty);        pr("t1", mp); mp.points = orig;
    mp.translate(c);             pr("t2", mp); mp.points = orig;
    mp.rotate(a);                pr("r1", mp); mp.points = orig;
    mp.rotate(ca, sa);           pr("r2", mp); mp.points = orig;
    mp.rotate(a, c);             pr("r3", mp); mp.points = orig;
    mp.reverse();                pr("rv", mp); mp.points = orig;
    mp.append(q);                pr("a1", mp); mp.points = orig;
    Points src = { c, q };
    mp.append(src);              pr("a2", mp); mp.points = orig;
    mp.append(src.cbegin() + 1, src.cend()); pr("a3", mp); mp.points = orig;
    mp.append(Points(src));      pr("a4", mp); mp.points = orig;
    mp.clear();                  pr("cl", mp); mp.points = orig;
    printf("\n");
}

int main()
{
    for (int i = 0; i < 3000; ++i) {
        int n = (i % 10 == 0) ? ri(0, 2) : ri(3, 12);
        Points pts;
        for (int j = 0; j < n; ++j) pts.emplace_back(ri(-6000000, 6000000), ri(-6000000, 6000000));
        if (i % 2 == 0) { Polygon pg(pts); run(pg, 0); }
        else            { Polyline pl; pl.points = pts; run(pl, 1); }
    }
    return 0;
}
