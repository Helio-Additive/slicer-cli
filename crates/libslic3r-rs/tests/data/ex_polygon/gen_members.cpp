// R951: the ExPolygon members and free-function overloads under their C++ names (ExPolygon.hpp:38-86,
// 150-178, 214-435, ExPolygon.cpp:16-67, 225-261, 490-520) on random star-shaped ExPolygons with holes.
// Usage: gen <out.txt>. Output: tests/data/ex_polygon/members_vectors.txt.gz (gzip -9n).
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>
#include <algorithm>
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/BoundingBox.hpp"

using namespace Slic3r;

static uint64_t g_seed = 0x9E3779B97F4A7C15ULL;
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

static ExPolygon random_ex(bool snap, int k)
{
    double cx = (rnd() - 0.5) * 4e6, cy = (rnd() - 0.5) * 4e6;
    ExPolygon ex;
    ex.contour = star(cx, cy, 3 + int(rnd() * 20), 1e6, 4e6, snap);
    int nh = int(rnd() * 3);
    for (int h = 0; h < nh; ++h) {
        Polygon hole = star(cx + (rnd() - 0.5) * 1e6, cy + (rnd() - 0.5) * 1e6, 3 + int(rnd() * 8), 1e5, 5e5, snap);
        if (k % 7 != 6) hole.reverse();   // every seventh keeps a CCW hole (is_valid false)
        ex.holes.push_back(hole);
    }
    if (k % 13 == 12) ex.contour.reverse(); // a CW contour (is_valid false)
    return ex;
}

static void pts(FILE *f, const Points &p) { std::fprintf(f, " %zu", p.size()); for (const Point &q : p) std::fprintf(f, " %d %d", int(q.x()), int(q.y())); }
static void ex1(FILE *f, const ExPolygon &e) { std::fprintf(f, " %zu", e.holes.size()); pts(f, e.contour.points); for (const Polygon &h : e.holes) pts(f, h.points); }
static void exs(FILE *f, const char *tag, const ExPolygons &v) { std::fprintf(f, "%s %zu", tag, v.size()); for (const ExPolygon &e : v) ex1(f, e); std::fprintf(f, "\n"); }
static void exo(FILE *f, const char *tag, const ExPolygon &e) { std::fprintf(f, "%s", tag); ex1(f, e); std::fprintf(f, "\n"); }
static void pgs(FILE *f, const char *tag, const Polygons &v) { std::fprintf(f, "%s %zu", tag, v.size()); for (const Polygon &p : v) pts(f, p.points); std::fprintf(f, "\n"); }
static void pls(FILE *f, const char *tag, const Polylines &v) { std::fprintf(f, "%s %zu", tag, v.size()); for (const Polyline &p : v) pts(f, p.points); std::fprintf(f, "\n"); }
static void bbo(FILE *f, const char *tag, const BoundingBox &b) { std::fprintf(f, "%s %d %d %d %d %d\n", tag, int(b.defined), int(b.min.x()), int(b.min.y()), int(b.max.x()), int(b.max.y())); }

int main(int argc, char **argv)
{
    if (argc < 2) { std::fprintf(stderr, "usage: gen <out.txt>\n"); return 2; }
    FILE *f = std::fopen(argv[1], "w");
    for (int k = 0; k < 1000; ++k) {
        bool snap = (k % 4) == 3;
        ExPolygon a = random_ex(snap, k), b = random_ex(snap, k + 1);
        double fac = 0.25 + rnd() * 3., fx = 0.25 + rnd() * 3., fy = 0.25 + rnd() * 3.;
        double tx = (rnd() - 0.5) * 2e6, ty = (rnd() - 0.5) * 2e6;
        double ang = (rnd() - 0.5) * 7.;
        Point ctr(coord_t((rnd() - 0.5) * 4e6), coord_t((rnd() - 0.5) * 4e6));
        double tol = 1000. + rnd() * 60000.;
        std::fprintf(f, "case %d\n", k);
        exo(f, "a", a);
        exo(f, "b", b);
        std::fprintf(f, "prm %a %a %a %a %a %a %d %d %a\n", fac, fx, fy, tx, ty, ang, int(ctr.x()), int(ctr.y()), tol);
        { ExPolygon e = a; e.scale(fac); exo(f, "sc1", e); }
        { ExPolygon e = a; e.scale(fx, fy); exo(f, "sc2", e); }
        { ExPolygon e = a; e.translate(ctr); exo(f, "tr1", e); }
        { ExPolygon e = a; e.translate(tx, ty); exo(f, "tr2", e); }
        { ExPolygon e = a; e.rotate(ang); exo(f, "ro1", e); }
        { ExPolygon e = a; e.rotate(ang, ctr); exo(f, "ro2", e); }
        { ExPolygon e = a; bool e0 = e.empty(); size_t nc = e.num_contours(); bool v = e.is_valid(); e.clear(); std::fprintf(f, "mem %d %zu %d %d %zu\n", int(e0), nc, int(v), int(e.empty()), e.holes.size()); }
        pgs(f, "sp1", a.simplify_p(tol));
        { Polygons pp = to_polygons(b); a.simplify_p(tol, &pp); pgs(f, "sp2", pp); }
        exs(f, "si1", a.simplify(tol));
        { ExPolygons v{ b }; a.simplify(tol, &v); exs(f, "si2", v); }
        { ExPolygons v; a.simplify(tol, &v); exs(f, "si3", v); }
        ExPolygons ab{ a, b };
        pls(f, "pl1", to_polylines(a));
        pls(f, "pl2", to_polylines(ab));
        pls(f, "pl3", to_polylines(ExPolygon(a)));
        pls(f, "pl4", to_polylines(ExPolygons(ab)));
        pgs(f, "pg1", to_polygons(a));
        pgs(f, "pg2", to_polygons(ab));
        pgs(f, "pg3", to_polygons(ExPolygon(a)));
        pgs(f, "pg4", to_polygons(ExPolygons(ab)));
        { Polygons src = to_polygons(ab); exs(f, "te1", to_expolygons(src)); exs(f, "te2", to_expolygons(Polygons(src))); }
        { Polygons d = to_polygons(b); polygons_append(d, a); pgs(f, "pa1", d); }
        { Polygons d = to_polygons(b); polygons_append(d, ab); pgs(f, "pa2", d); }
        { Polygons d = to_polygons(b); polygons_append(d, ExPolygon(a)); pgs(f, "pa3", d); }
        { Polygons d = to_polygons(b); polygons_append(d, ExPolygons(ab)); pgs(f, "pa4", d); }
        { ExPolygons d{ b }; expolygons_append(d, ab); exs(f, "ea1", d); }
        { ExPolygons d; expolygons_append(d, ExPolygons(ab)); exs(f, "ea2", d); }
        { ExPolygons d{ b }; expolygons_append(d, ExPolygons(ab)); exs(f, "ea3", d); }
        bbo(f, "ge1", get_extents(a));
        bbo(f, "ge2", get_extents(ab));
        bbo(f, "gr1", get_extents_rotated(a, ang));
        bbo(f, "gr2", get_extents_rotated(ab, ang));
        {
            Linesf l0 = to_linesf(ab), l1 = to_linesf(ab, uint32_t(count_points(ab)));
            std::fprintf(f, "lf %zu %zu", l0.size(), l1.size());
            for (const Linef &l : l0) std::fprintf(f, " %.1f %.1f %.1f %.1f", l.a.x(), l.a.y(), l.b.x(), l.b.y());
            bool same = l0.size() == l1.size();
            for (size_t i = 0; same && i < l0.size(); ++i) same = l0[i].a == l1[i].a && l0[i].b == l1[i].b;
            std::fprintf(f, " %d\n", int(same));
        }
    }
    std::fclose(f);
    return 0;
}
