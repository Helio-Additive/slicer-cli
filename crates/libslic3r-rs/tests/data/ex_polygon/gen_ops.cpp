// Native parity vectors for ExPolygon (R947): ExPolygon::overlaps (ExPolygon.cpp:158-173),
// ExPolygon::contains(Line) / contains(Polyline) (76-90), get_extents_rotated(ExPolygon) (506-509),
// has_duplicate_points(ExPolygon) (531-...), to_unscaled_linesf(ExPolygons) (ExPolygon.hpp:180-198).
// Usage: gen_ops <out.txt> (tests/data/ex_polygon/ops_vectors.txt.gz, read by tests/ex_polygon_ops_native_parity.rs)
// Per case: "case k", "ex nholes" + "c n x y ..." + "h n x y ..." for A and B, "pl n x y ..." (a polyline),
// "ln x y x y", "ang %a", then "ov a_b b_a", "cl b", "cp b", "bb minx miny maxx maxy", "dup b",
// "lf n" + n lines "%a %a %a %a" (to_unscaled_linesf of {A, B}),
// then "pt x y", "mw %a", "x2 ovs ovsx narrow proj_x proj_y match_rot match_ab bbx minx miny maxx maxy":
// overlaps(ExPolygons{A}, ExPolygons{B}), overlaps(ExPolygons{B}, A), is_narrow_expolygon(A, mw) (default
// min_area / ratio, ExPolygon.cpp:462-475), projection_onto({A, B}, pt) (206-), expolygons_match(A, A with its
// contour rotated) and (A, B) (480-488), get_extents_rotated({A, B}, ang) (511-);
// then "ar %a" (area(ExPolygons{A, B}), ExPolygon.hpp:488), "rsn b k" + k ExPolygons (remove_same_neighbor on
// {A with a doubled point, B}, ExPolygon.cpp), "klc k" + k ExPolygons (keep_largest_contour_only({A, B}),
// 623-), "rs b" + 1 ExPolygon (remove_sticks(A), 597-).
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>
#include <algorithm>
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/BoundingBox.hpp"

using namespace Slic3r;

static uint64_t g_seed = 0x2545F4914F6CDD1DULL;
static double rnd() { g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL; return double(g_seed >> 11) / double(1ULL << 53); }

static Polygon star(double cx, double cy, int n, double rmin, double rmax, bool snap)
{
    std::vector<double> ang(n);
    for (int i = 0; i < n; ++i) ang[i] = rnd() * 2. * M_PI;
    std::sort(ang.begin(), ang.end());
    Polygon p;
    for (int i = 0; i < n; ++i) {
        double r = rmin + (rmax - rmin) * rnd();
        double x = cx + r * std::cos(ang[i]), y = cy + r * std::sin(ang[i]);
        if (snap) { x = std::round(x / 500000.) * 500000.; y = std::round(y / 500000.) * 500000.; }
        p.points.emplace_back(coord_t(x), coord_t(y));
    }
    return p;
}

static ExPolygon random_ex(bool snap)
{
    double cx = (rnd() - 0.5) * 4e6, cy = (rnd() - 0.5) * 4e6;
    ExPolygon ex;
    ex.contour = star(cx, cy, 3 + int(rnd() * 20), 1e6, 4e6, snap);
    int nh = int(rnd() * 3);
    for (int h = 0; h < nh; ++h) {
        Polygon hole = star(cx + (rnd() - 0.5) * 1e6, cy + (rnd() - 0.5) * 1e6, 3 + int(rnd() * 8), 1e5, 5e5, snap);
        hole.reverse();
        ex.holes.push_back(hole);
    }
    return ex;
}

static void dump_poly(FILE *f, const char *tag, const Points &pts)
{
    std::fprintf(f, "%s %zu", tag, pts.size());
    for (const Point &pt : pts) std::fprintf(f, " %d %d", int(pt.x()), int(pt.y()));
    std::fprintf(f, "\n");
}

static void dump_ex(FILE *f, const ExPolygon &ex)
{
    std::fprintf(f, "ex %zu\n", ex.holes.size());
    dump_poly(f, "c", ex.contour.points);
    for (const Polygon &h : ex.holes) dump_poly(f, "h", h.points);
}

int main(int argc, char **argv)
{
    if (argc < 2) { std::fprintf(stderr, "usage: gen <out.txt>\n"); return 2; }
    FILE *f = std::fopen(argv[1], "w");
    for (int k = 0; k < 1500; ++k) {
        bool snap = (k % 4) == 3;
        ExPolygon a = random_ex(snap), b = random_ex(snap);
        if (k % 5 == 4 && !a.contour.points.empty()) a.contour.points.push_back(a.contour.points[int(rnd() * a.contour.points.size())]);
        Polyline pl;
        int npl = 2 + int(rnd() * 6);
        for (int i = 0; i < npl; ++i) pl.points.emplace_back(coord_t((rnd() - 0.5) * 8e6), coord_t((rnd() - 0.5) * 8e6));
        Line ln(Point(coord_t((rnd() - 0.5) * 6e6), coord_t((rnd() - 0.5) * 6e6)), Point(coord_t((rnd() - 0.5) * 6e6), coord_t((rnd() - 0.5) * 6e6)));
        double ang = (rnd() - 0.5) * 7.;
        std::fprintf(f, "case %d\n", k);
        dump_ex(f, a);
        dump_ex(f, b);
        dump_poly(f, "pl", pl.points);
        std::fprintf(f, "ln %d %d %d %d\n", int(ln.a.x()), int(ln.a.y()), int(ln.b.x()), int(ln.b.y()));
        std::fprintf(f, "ang %a\n", ang);
        std::fprintf(f, "ov %d %d\n", int(a.overlaps(b)), int(b.overlaps(a)));
        std::fprintf(f, "cl %d\n", int(a.contains(ln)));
        std::fprintf(f, "cp %d\n", int(a.contains(pl)));
        BoundingBox bb = get_extents_rotated(a, ang);
        std::fprintf(f, "bb %d %d %d %d\n", int(bb.min.x()), int(bb.min.y()), int(bb.max.x()), int(bb.max.y()));
        std::fprintf(f, "dup %d\n", int(has_duplicate_points(a)));
        Linesf lf = to_unscaled_linesf(ExPolygons{ a, b });
        std::fprintf(f, "lf %zu\n", lf.size());
        for (const Linef &l : lf) std::fprintf(f, "%a %a %a %a\n", l.a.x(), l.a.y(), l.b.x(), l.b.y());
        Point pt(coord_t((rnd() - 0.5) * 8e6), coord_t((rnd() - 0.5) * 8e6));
        double mw = scale_(0.2 + rnd() * 3.);
        std::fprintf(f, "pt %d %d\n", int(pt.x()), int(pt.y()));
        std::fprintf(f, "mw %a\n", mw);
        ExPolygon ar = a;
        int rot = int(rnd() * ar.contour.points.size());
        std::rotate(ar.contour.points.begin(), ar.contour.points.begin() + rot, ar.contour.points.end());
        std::fprintf(f, "rot %d\n", rot);
        Point proj = projection_onto(ExPolygons{ a, b }, pt);
        BoundingBox bbx = get_extents_rotated(ExPolygons{ a, b }, ang);
        std::fprintf(f, "x2 %d %d %d %d %d %d %d %d %d %d %d\n", int(overlaps(ExPolygons{ a }, ExPolygons{ b })), int(overlaps(ExPolygons{ b }, a)),
            int(is_narrow_expolygon(a, mw)), int(proj.x()), int(proj.y()), int(expolygons_match(a, ar)), int(expolygons_match(a, b)),
            int(bbx.min.x()), int(bbx.min.y()), int(bbx.max.x()), int(bbx.max.y()));
        std::fprintf(f, "ar %a\n", area(ExPolygons{ a, b }));
        {
            ExPolygons v{ a, b };
            if (v[0].contour.points.size() > 1) v[0].contour.points.insert(v[0].contour.points.begin() + 1, v[0].contour.points[1]);
            bool r = remove_same_neighbor(v);
            std::fprintf(f, "rsn %d %zu\n", int(r), v.size());
            for (const ExPolygon &e : v) dump_ex(f, e);
        }
        {
            ExPolygons v{ a, b };
            keep_largest_contour_only(v);
            std::fprintf(f, "klc %zu\n", v.size());
            for (const ExPolygon &e : v) dump_ex(f, e);
        }
        {
            ExPolygon e = a;
            bool r = remove_sticks(e);
            std::fprintf(f, "rs %d\n", int(r));
            dump_ex(f, e);
        }
    }
    std::fclose(f);
    return 0;
}
