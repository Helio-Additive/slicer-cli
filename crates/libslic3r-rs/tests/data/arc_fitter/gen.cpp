// Vectors for tests/arc_fitter_native_parity.rs: the pin's ArcFitter::do_arc_fitting_and_simplify
// (ArcFitter.cpp:9-152, with Circle.cpp's try_create_circle / try_create_arc) on deterministic
// polylines made of arcs and straight runs with a few scaled units of noise (1,500 cases; the
// second half with the radii and steps of infill and wall curves).
// Output per case: "C tol n x0 y0 ... -> m x0 y0 ... | k (s e type [cx cy radius length angle ps pe dir])..."
// then per fitted arc "S arc px py -> inside clip_start(ok arc) clip_end(ok arc) reverse(ok arc) split_at(ok [arc arc])",
// and "T x1 y1 x2 y2 x3 y3 max_radius -> ok [cx cy radius]" for Circle::try_create_circle of three of its points;
// at the end 2,000 "R start end centre ccw -> radian radius length" (float bits) for ArcSegment::calc_arc_*.
#include "libslic3r/ArcFitter.hpp"
#include "libslic3r/Point.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

using namespace Slic3r;

static uint64_t s = 0x2545F4914F6CDD1Dull;
static uint64_t rnd() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
static double   unit() { return double(rnd() >> 11) / double(1ull << 53); }
static double   uni(double a, double b) { return a + (b - a) * unit(); }
static int      irange(int a, int b) { return a + int(rnd() % uint64_t(b - a + 1)); }
static unsigned long long db(double d) { unsigned long long u; memcpy(&u, &d, 8); return u; }

static void arc_out(const ArcSegment &a)
{
    printf(" %d %lld %lld %016llx %lld %lld %lld %lld %d %016llx %016llx %016llx %016llx", int(a.is_arc), (long long)a.center.x(),
           (long long)a.center.y(), db(a.radius), (long long)a.start_point.x(), (long long)a.start_point.y(), (long long)a.end_point.x(),
           (long long)a.end_point.y(), int(a.direction), db(a.length), db(a.angle_radians), db(a.polar_start_theta), db(a.polar_end_theta));
}

int main()
{
    const double tols[] = { 1000., 2500., 5000., 10000. };
    for (int c = 0; c < 1500; ++c) {
        Points pts;
        double x = uni(-1e7, 1e7), y = uni(-1e7, 1e7), dir = uni(0, 2 * M_PI);
        int pieces = irange(1, 4);
        long long noise = irange(0, 3) == 0 ? 0 : irange(1, 40);
        pts.emplace_back(coord_t(x), coord_t(y));
        for (int p = 0; p < pieces; ++p) {
            if (rnd() % 3 == 0) {
                int    n    = irange(1, 6);
                double step = uni(2e4, 4e5);
                for (int i = 0; i < n; ++i) {
                    x += step * cos(dir); y += step * sin(dir);
                    pts.emplace_back(coord_t(x) + (noise ? (long long)(rnd() % (2 * noise + 1)) - noise : 0),
                                     coord_t(y) + (noise ? (long long)(rnd() % (2 * noise + 1)) - noise : 0));
                }
            } else {
                // the second half: radii and steps of infill and wall curves
                double r    = c < 750 ? uni(2e5, 3e7) : uni(5e4, 5e6);
                double sgn  = (rnd() & 1) ? 1. : -1.;
                double cx   = x - sgn * r * sin(dir), cy = y + sgn * r * cos(dir);
                double a0   = atan2(y - cy, x - cx);
                int    n    = irange(3, 40);
                double dang = sgn * (c < 750 ? uni(0.02, 0.3) : uni(0.01, 0.2));
                for (int i = 1; i <= n; ++i) {
                    double a = a0 + dang * i;
                    x = cx + r * cos(a); y = cy + r * sin(a);
                    pts.emplace_back(coord_t(x) + (noise ? (long long)(rnd() % (2 * noise + 1)) - noise : 0),
                                     coord_t(y) + (noise ? (long long)(rnd() % (2 * noise + 1)) - noise : 0));
                }
                dir += dang * n;
            }
        }
        double tol = tols[rnd() % 4];
        printf("C %.0f %zu", tol, pts.size());
        for (const Point &p : pts) printf(" %lld %lld", (long long)p.x(), (long long)p.y());
        Points work = pts;
        std::vector<PathFittingData> result;
        ArcFitter::do_arc_fitting_and_simplify(work, result, tol);
        printf(" -> %zu", work.size());
        for (const Point &p : work) printf(" %lld %lld", (long long)p.x(), (long long)p.y());
        printf(" | %zu", result.size());
        for (const PathFittingData &d : result) {
            printf(" %zu %zu %d", d.start_point_index, d.end_point_index, int(d.path_type));
            if (d.path_type == EMovePathType::Arc_move_cw || d.path_type == EMovePathType::Arc_move_ccw) {
                const ArcSegment &a = d.arc_data;
                printf(" %lld %lld %016llx %016llx %016llx %016llx %016llx %d", (long long)a.center.x(), (long long)a.center.y(),
                       db(a.radius), db(a.length), db(a.angle_radians), db(a.polar_start_theta), db(a.polar_end_theta), int(a.direction));
            }
        }
        printf("\n");
        // ArcSegment::is_point_inside / clip_start / clip_end / reverse / split_at (Circle.cpp:203-254)
        // on every fitted arc, at a point on the circle inside or outside the arc.
        for (const PathFittingData &d : result) {
            if (!(d.path_type == EMovePathType::Arc_move_cw || d.path_type == EMovePathType::Arc_move_ccw)) continue;
            const ArcSegment &a  = d.arc_data;
            double            t  = uni(-0.2, 1.2);
            double            th = a.polar_start_theta + a.angle_radians * t;
            Point             p(coord_t(lrint(a.center.x() + a.radius * cos(th))), coord_t(lrint(a.center.y() + a.radius * sin(th))));
            if (rnd() % 8 == 0) p = a.center;
            printf("S");
            arc_out(a);
            printf(" %lld %lld -> %d", (long long)p.x(), (long long)p.y(), int(a.is_point_inside(p)));
            ArcSegment cs = a; bool ok = cs.clip_start(p); printf(" %d", int(ok)); arc_out(cs);
            ArcSegment ce = a; ok = ce.clip_end(p); printf(" %d", int(ok)); arc_out(ce);
            ArcSegment rv = a; ok = rv.reverse(); printf(" %d", int(ok)); arc_out(rv);
            ArcSegment sp = a, s1, s2; ok = sp.split_at(p, s1, s2); printf(" %d", int(ok));
            if (ok) { arc_out(s1); arc_out(s2); }
            printf("\n");
        }
        // Circle::try_create_circle(p1, p2, p3, max_radius, c) (Circle.cpp:15-55) on three of the points.
        if (pts.size() >= 3) {
            const double maxr[] = { 1e6, 1e7, 1e9 };
            double       mr     = maxr[rnd() % 3];
            Circle       circ;
            size_t       i1 = rnd() % pts.size(), i2 = rnd() % pts.size(), i3 = rnd() % pts.size();
            bool         ok = Circle::try_create_circle(pts[i1], pts[i2], pts[i3], mr, circ);
            printf("T %lld %lld %lld %lld %lld %lld %016llx -> %d", (long long)pts[i1].x(), (long long)pts[i1].y(), (long long)pts[i2].x(),
                   (long long)pts[i2].y(), (long long)pts[i3].x(), (long long)pts[i3].y(), db(mr), int(ok));
            if (ok) printf(" %lld %lld %016llx", (long long)circ.center.x(), (long long)circ.center.y(), db(circ.radius));
            printf("\n");
        }
    }
    // ArcSegment::calc_arc_radian / calc_arc_radius / calc_arc_length (Circle.cpp:479-514), float.
    for (int c = 0; c < 2000; ++c) {
        Vec3f st(float(uni(-200, 200)), float(uni(-200, 200)), float(uni(0, 50)));
        Vec3f ce(float(uni(-200, 200)), float(uni(-200, 200)), float(uni(0, 50)));
        float r  = (st - ce).head<2>().norm();
        float th = float(uni(0, 2 * M_PI));
        Vec3f en = (c % 4 == 0) ? Vec3f(float(uni(-200, 200)), float(uni(-200, 200)), float(uni(0, 50))) :
                                  Vec3f(ce.x() + r * float(cos(double(th))), ce.y() + r * float(sin(double(th))), st.z());
        if (c % 50 == 0) en = st;
        bool  ccw = rnd() & 1;
        float a = ArcSegment::calc_arc_radian(st, en, ce, ccw), rr = ArcSegment::calc_arc_radius(st, ce),
              l = ArcSegment::calc_arc_length(st, en, ce, ccw);
        unsigned int ua, ur, ul, v[9];
        memcpy(&ua, &a, 4); memcpy(&ur, &rr, 4); memcpy(&ul, &l, 4);
        float in[9] = { st.x(), st.y(), st.z(), en.x(), en.y(), en.z(), ce.x(), ce.y(), ce.z() };
        memcpy(v, in, 36);
        printf("R");
        for (int i = 0; i < 9; ++i) printf(" %08x", v[i]);
        printf(" %d -> %08x %08x %08x\n", int(ccw), ua, ur, ul);
    }
    return 0;
}
