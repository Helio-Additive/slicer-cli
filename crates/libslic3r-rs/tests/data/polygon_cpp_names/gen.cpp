// Native vectors for tests/polygon_cpp_names_native_parity.rs: Polygon.hpp / Polygon.cpp overloads under
// the C++ names: contains(Polygon) / contains(Polygons), area(Polygons), polygons_append x2,
// to_lines(Polygons), to_polylines(Polygons&&), to_polygons(paths) x2, remove_same_neighbor(Polygon) /
// (Polygons), has_duplicate_points(const Polygon&).
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include <cstdio>
#include <cstring>
#include <cstdint>

using namespace Slic3r;

static uint64_t g_s = 0xD1B54A32D192ED03ULL;
static uint32_t rnd() { g_s = g_s * 6364136223846793005ULL + 1442695040888963407ULL; return uint32_t(g_s >> 33); }
static int32_t ri(int32_t lo, int32_t hi) { return lo + int32_t(rnd() % uint32_t(hi - lo + 1)); }
static unsigned long long db(double d) { unsigned long long u; std::memcpy(&u, &d, 8); return u; }

static void pp(const Points &pts) { printf(" %zu", pts.size()); for (const Point &p : pts) printf(" %d %d", p.x(), p.y()); }
static void pps(const char *tag, const Polygons &ps) { printf(" %s %zu", tag, ps.size()); for (const Polygon &p : ps) pp(p.points); }

int main()
{
    for (int i = 0; i < 3000; ++i) {
        int np = ri(1, 4);
        Polygons ps;
        for (int k = 0; k < np; ++k) {
            int n = ri(3, 9);
            Polygon pg;
            int cx = ri(-500, 500), cy = ri(-500, 500);
            for (int j = 0; j < n; ++j) {
                if (j > 0 && ri(0, 5) == 0) pg.points.push_back(pg.points.back());
                else pg.points.emplace_back(cx + ri(-300, 300), cy + ri(-300, 300));
            }
            ps.push_back(pg);
        }
        Point q(ri(-800, 800), ri(-800, 800));
        if (i % 4 == 0) q = ps[0].points[0];
        printf("PG");
        pps("in", ps);
        printf(" q %d %d | %d %d %d %d %016llx %d", q.x(), q.y(), int(contains(ps[0], q, false)), int(contains(ps[0], q, true)),
               int(contains(ps, q, false)), int(contains(ps, q, true)), db(area(ps)), int(has_duplicate_points(static_cast<const Polygon&>(ps[0]))));
        Lines ls = to_lines(ps);
        printf(" L %zu", ls.size());
        for (const Line &l : ls) printf(" %d %d %d %d", l.a.x(), l.a.y(), l.b.x(), l.b.y());
        Polylines pls = to_polylines(Polygons(ps));
        printf(" PL %zu", pls.size());
        for (const Polyline &pl : pls) pp(pl.points);
        std::vector<Points> paths;
        for (const Polygon &p : ps) paths.push_back(p.points);
        if (i % 3 == 0) paths.push_back(Points());
        const std::vector<Points> &cpaths = paths;
        pps("TP1", to_polygons(cpaths));
        pps("TP2", to_polygons(std::vector<Points>(paths)));
        Polygons dst1; if (i % 2) dst1.push_back(ps.back());
        Polygons dst2 = dst1;
        polygons_append(dst1, static_cast<const Polygons&>(ps)); pps("A1", dst1);
        polygons_append(dst2, Polygons(ps)); pps("A2", dst2);
        Polygon one = ps[0]; bool r1 = remove_same_neighbor(one);
        printf(" R1 %d", int(r1)); pp(one.points);
        Polygons all = ps; bool r2 = remove_same_neighbor(all);
        printf(" R2 %d", int(r2)); pps("RS", all);
        printf("\n");
    }
    return 0;
}
