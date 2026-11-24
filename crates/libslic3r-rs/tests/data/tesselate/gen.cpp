// Native parity vectors for Tesselate.cpp (glu-libtess). R947.
// Usage: gen <out.txt>
// Per case: "case k mode z flip nex", per ExPolygon "ex nholes", then "c n x y x y ..." (contour) and
// "h n x y ..." per hole (scaled coordinates), then the pin's results:
// "r3 n" + n lines "%a %a %a" for triangulate_expolygons_3d(all, z, flip),
// "r1 n" + n lines for triangulate_expolygon_3d(first, z, flip),
// "r2 n" + n lines "%a %a" for triangulate_expolygon_2d(first, flip),
// "r4 n" + n lines "%a %a" for triangulate_expolygons_2d(all, flip),
// "r5 n" / "r6 n" + n lines "%a %a" (floats promoted) for triangulate_expolygon_2f(first) / triangulate_expolygons_2f(all).
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>
#include <algorithm>
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Tesselate.hpp"

using namespace Slic3r;

static uint64_t g_seed = 0x9E3779B97F4A7C15ULL;
static double rnd() { g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL; return double(g_seed >> 11) / double(1ULL << 53); }

// Star-shaped polygon around (cx, cy): n vertices at sorted angles with radii in [rmin, rmax].
static Polygon star(double cx, double cy, int n, double rmin, double rmax, int mode)
{
    std::vector<double> ang(n);
    for (int i = 0; i < n; ++i) ang[i] = rnd() * 2. * M_PI;
    std::sort(ang.begin(), ang.end());
    Polygon p;
    for (int i = 0; i < n; ++i) {
        double r = rmin + (rmax - rmin) * rnd();
        double x = cx + r * std::cos(ang[i]);
        double y = cy + r * std::sin(ang[i]);
        if (mode == 1) { x = std::round(x / 200000.) * 200000.; y = std::round(y / 200000.) * 200000.; }
        p.points.emplace_back(coord_t(x), coord_t(y));
    }
    return p;
}

static void dump_poly(FILE *f, const char *tag, const Polygon &p)
{
    std::fprintf(f, "%s %zu", tag, p.points.size());
    for (const Point &pt : p.points) std::fprintf(f, " %d %d", int(pt.x()), int(pt.y()));
    std::fprintf(f, "\n");
}

int main(int argc, char **argv)
{
    if (argc < 2) { std::fprintf(stderr, "usage: gen <out.txt>\n"); return 2; }
    FILE *f = std::fopen(argv[1], "w");
    const int ncases = 500;
    for (int k = 0; k < ncases; ++k) {
        // mode 0: random star polygons with holes; 1: snapped to a 2 mm grid (collinear / duplicate
        // points); 2: rectangles and overlapping holes (intersections, combine callback).
        int mode = k % 3;
        int nex = 1 + int(rnd() * 3);
        double z = (rnd() - 0.5) * 20.;
        bool flip = rnd() < 0.5;
        ExPolygons exs;
        for (int e = 0; e < nex; ++e) {
            double cx = (rnd() - 0.5) * 2e7, cy = (rnd() - 0.5) * 2e7;
            ExPolygon ex;
            int n = 3 + int(rnd() * (rnd() < 0.15 ? 150 : 30));
            if (mode == 2 && rnd() < 0.5) {
                double w = 1e6 + rnd() * 5e6, h = 1e6 + rnd() * 5e6;
                ex.contour.points = { Point(coord_t(cx - w), coord_t(cy - h)), Point(coord_t(cx + w), coord_t(cy - h)),
                                      Point(coord_t(cx + w), coord_t(cy + h)), Point(coord_t(cx - w), coord_t(cy + h)) };
            } else
                ex.contour = star(cx, cy, n, 1e6, 5e6, mode);
            int nh = int(rnd() * 4);
            for (int h = 0; h < nh; ++h) {
                double hx = cx + (rnd() - 0.5) * 2e6, hy = cy + (rnd() - 0.5) * 2e6;
                Polygon hole = star(hx, hy, 3 + int(rnd() * 12), mode == 2 ? 3e5 : 1e5, mode == 2 ? 1.5e6 : 6e5, mode);
                hole.reverse();
                ex.holes.push_back(hole);
            }
            exs.push_back(ex);
        }
        std::fprintf(f, "case %d %d %a %d %d\n", k, mode, z, int(flip), nex);
        for (const ExPolygon &ex : exs) {
            std::fprintf(f, "ex %zu\n", ex.holes.size());
            dump_poly(f, "c", ex.contour);
            for (const Polygon &h : ex.holes) dump_poly(f, "h", h);
        }
        std::vector<Vec3d> r3 = triangulate_expolygons_3d(exs, z, flip);
        std::fprintf(f, "r3 %zu\n", r3.size());
        for (const Vec3d &v : r3) std::fprintf(f, "%a %a %a\n", v.x(), v.y(), v.z());
        std::vector<Vec3d> r1 = triangulate_expolygon_3d(exs.front(), z, flip);
        std::fprintf(f, "r1 %zu\n", r1.size());
        for (const Vec3d &v : r1) std::fprintf(f, "%a %a %a\n", v.x(), v.y(), v.z());
        std::vector<Vec2d> r2 = triangulate_expolygon_2d(exs.front(), flip);
        std::fprintf(f, "r2 %zu\n", r2.size());
        for (const Vec2d &v : r2) std::fprintf(f, "%a %a\n", v.x(), v.y());
        std::vector<Vec2d> r4 = triangulate_expolygons_2d(exs, flip);
        std::fprintf(f, "r4 %zu\n", r4.size());
        for (const Vec2d &v : r4) std::fprintf(f, "%a %a\n", v.x(), v.y());
        std::vector<Vec2f> r5 = triangulate_expolygon_2f(exs.front(), flip);
        std::fprintf(f, "r5 %zu\n", r5.size());
        for (const Vec2f &v : r5) std::fprintf(f, "%a %a\n", double(v.x()), double(v.y()));
        std::vector<Vec2f> r6 = triangulate_expolygons_2f(exs, flip);
        std::fprintf(f, "r6 %zu\n", r6.size());
        for (const Vec2f &v : r6) std::fprintf(f, "%a %a\n", double(v.x()), double(v.y()));
    }
    std::fclose(f);
    return 0;
}
