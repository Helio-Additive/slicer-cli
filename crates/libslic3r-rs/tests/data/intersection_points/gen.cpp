// Reference vectors for crates/libslic3r-rs/src/intersection_points.rs, taken from
// IntersectionPoints.cpp at the pin.
//
// The program links the engine's libslic3r and calls the five get_intersections overloads on
// generated polygons (build it as a libslic3r test program of a configured Bambu Studio engine
// build tree), then:
//   ./intersection_points_gen | gzip -9n > crates/libslic3r-rs/tests/data/intersection_points/vectors.txt.gz
//
// One case per line: the input polygons, then the result of each overload on them.
//   CASE k  n1 x y x y ...  n2 x y ...  ->  L m (i j x y)*  P m ...  PS m ...  E m ...  ES m ...
// where L is get_intersections(Lines) on to_lines of all polygons, P get_intersections(Polygon)
// on the first, PS get_intersections(Polygons) on all, E get_intersections(ExPolygon) with the
// first polygon as contour and the others as holes, ES get_intersections(ExPolygons) with one
// expolygon per polygon. Each result is its count m, then per intersection the two line indices
// and the point's doubles as the hex of their bits.
// The last cases also check has_self_intersection of Triangulation.cpp (an inline function of its
// private namespace, `!get_intersections(lines).empty()` over the constrained edges):
//   SELF n x y ...  e  i j ...  -> 0|1

#include "libslic3r/IntersectionPoints.hpp"
#include "libslic3r/ExPolygon.hpp"

#include <cstdio>
#include <cstring>

using namespace Slic3r;

static uint64_t s = 0x94D049BB133111EBull;
static uint64_t rnd() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
static unsigned long long db(double d) { unsigned long long u; memcpy(&u, &d, 8); return u; }

static void res(const char *tag, const IntersectionsLines &r)
{
    printf(" %s %zu", tag, r.size());
    for (const IntersectionLines &i : r)
        printf(" %u %u %016llx %016llx", i.line_index1, i.line_index2, db(i.intersection.x()), db(i.intersection.y()));
}

int main()
{
    for (int c = 0; c < 400; ++ c) {
        // Coordinates on a coarse grid so that end points coincide and segments overlap now and then.
        // Odd grids make line centroids fall on halves, which the tree truncates.
        int64_t grid  = (c % 4 == 0) ? 10 : (c % 4 == 1) ? 1000 : (c % 4 == 2) ? 100000 : 7;
        int     cells = 6 + int(rnd() % 10);
        Polygons polys(1 + rnd() % 3);
        for (Polygon &p : polys) {
            size_t n = (grid == 7) ? 6 + rnd() % 10 : 3 + rnd() % 8;
            for (size_t i = 0; i < n; ++ i)
                p.points.emplace_back(coord_t(int64_t(rnd() % cells) * grid), coord_t(int64_t(rnd() % cells) * grid));
        }
        printf("CASE %zu", polys.size());
        for (const Polygon &p : polys) {
            printf(" %zu", p.points.size());
            for (const Point &pt : p.points)
                printf(" %d %d", pt.x(), pt.y());
        }
        printf(" ->");
        res("L", get_intersections(to_lines(polys)));
        res("P", get_intersections(polys.front()));
        res("PS", get_intersections(polys));
        ExPolygon ex;
        ex.contour = polys.front();
        for (size_t i = 1; i < polys.size(); ++ i)
            ex.holes.push_back(polys[i]);
        res("E", get_intersections(ex));
        ExPolygons exs;
        for (const Polygon &p : polys)
            exs.emplace_back(p);
        res("ES", get_intersections(exs));
        printf("\n");
    }
    for (int c = 0; c < 200; ++ c) {
        size_t n = 3 + rnd() % 8;
        Points pts;
        for (size_t i = 0; i < n; ++ i)
            pts.emplace_back(coord_t(rnd() % 8) * 100, coord_t(rnd() % 8) * 100);
        size_t e = 2 + rnd() % 6;
        std::vector<std::pair<uint32_t, uint32_t>> edges;
        for (size_t i = 0; i < e; ++ i)
            edges.emplace_back(uint32_t(rnd() % n), uint32_t(rnd() % n));
        Lines lines;
        for (const auto &he : edges)
            lines.emplace_back(pts[he.first], pts[he.second]);
        printf("SELF %zu", n);
        for (const Point &p : pts)
            printf(" %d %d", p.x(), p.y());
        printf(" %zu", e);
        for (const auto &he : edges)
            printf(" %u %u", he.first, he.second);
        printf(" -> %d\n", int(! get_intersections(lines).empty()));
    }
    return 0;
}
