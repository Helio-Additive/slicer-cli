// Vectors for tests/polyline_split_native_parity.rs: the pin's Polyline::split_at (Polyline.cpp:223-265),
// split_at_index (268-298), split_at_length (300-344) with the fitting result they carry
// (split_fitting_result_before_index / after_index), Polyline::extend_end (103-109) and the free
// remove_same_neighbor(Polyline&) / remove_same_neighbor(Polylines&) (534-556), on deterministic
// polylines (a xorshift generator; arcs and straight runs, a few scaled units of noise, repeated points)
// after ArcFitter::do_arc_fitting_and_simplify (Polyline::simplify_by_fitting_arc, Polyline.cpp:152-156)
// for three quarters of the cases.
// Output: "P n x y ... | k (s e type [is_arc sx sy ex ey cx cy radius length angle polar_start polar_end (bits) dir])..."
// for a polyline; per case "C <fitted polyline>", "R <input points>", then the queries:
//   "SP qx qy -> px py <p1> <p2>"  split_at(point) (the point is written back)
//   "SI idx -> ok <p1> <p2>"        split_at_index
//   "SL len-bits -> ok <p1> <p2>"   split_at_length
//   "EE dist-bits -> <polyline>"    extend_end (on the plain points)
//   "RN -> erased <polyline>"       remove_same_neighbor(Polyline&)
//   "CE/CS dist-bits -> <polyline>" clip_end / clip_start, "RV -> <polyline>" reverse,
//   "AP <p1> <p2> -> <polyline>"    p1.append(p2), "AM <p2> <p1> -> <polyline>" p2.append(std::move(p1)),
//   "ES dist-bits -> <polyline>"    extend_start (on the plain points)
//   "PM dst_first src_first <dst> <src> -> <points>" polylines_merge on the points
// and "RNS n <polyline>... -> erased m <polyline>..." for remove_same_neighbor(Polylines&).
#include "libslic3r/ArcFitter.hpp"
#include "libslic3r/Polyline.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

using namespace Slic3r;

static uint64_t s = 0x9E3779B97F4A7C15ull;
static uint64_t rnd() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
static double   unit() { return double(rnd() >> 11) / double(1ull << 53); }
static double   uni(double a, double b) { return a + (b - a) * unit(); }
static int      irange(int a, int b) { return a + int(rnd() % uint64_t(b - a + 1)); }
static unsigned long long db(double d) { unsigned long long u; memcpy(&u, &d, 8); return u; }

static void out_pl(const Polyline &pl)
{
    printf(" P %zu", pl.points.size());
    for (const Point &p : pl.points) printf(" %d %d", p.x(), p.y());
    printf(" | %zu", pl.fitting_result.size());
    for (const PathFittingData &d : pl.fitting_result) {
        printf(" %zu %zu %d", d.start_point_index, d.end_point_index, int(d.path_type));
        if (d.path_type == EMovePathType::Arc_move_cw || d.path_type == EMovePathType::Arc_move_ccw) {
            const ArcSegment &a = d.arc_data;
            printf(" %d %d %d %d %d %d %d %016llx %016llx %016llx %016llx %016llx %d", int(a.is_arc), a.start_point.x(), a.start_point.y(),
                   a.end_point.x(), a.end_point.y(), a.center.x(), a.center.y(), db(a.radius), db(a.length), db(a.angle_radians),
                   db(a.polar_start_theta), db(a.polar_end_theta), int(a.direction));
        }
    }
}

static Polyline make_polyline(int c)
{
    Points pts;
    double x = uni(-1e7, 1e7), y = uni(-1e7, 1e7), dir = uni(0, 2 * M_PI);
    int pieces = irange(1, 4);
    long long noise = irange(0, 3) == 0 ? 0 : irange(1, 30);
    pts.emplace_back(coord_t(x), coord_t(y));
    for (int k = 0; k < pieces; ++k) {
        if (irange(0, 2) != 0) {
            // arc
            double r = c % 2 ? uni(2e4, 4e5) : uni(2e5, 5e6);
            double sweep = uni(0.3, 2.5) * (irange(0, 1) ? 1 : -1);
            int    n = irange(4, 30);
            double cx = x - r * sin(dir) * (sweep > 0 ? -1 : 1), cy = y + r * cos(dir) * (sweep > 0 ? -1 : 1);
            double a0 = atan2(y - cy, x - cx);
            for (int i = 1; i <= n; ++i) {
                double a = a0 + sweep * i / n;
                x = cx + r * cos(a); y = cy + r * sin(a);
                long long nx = noise ? irange(-int(noise), int(noise)) : 0, ny = noise ? irange(-int(noise), int(noise)) : 0;
                pts.emplace_back(coord_t(x) + coord_t(nx), coord_t(y) + coord_t(ny));
            }
            dir += sweep;
        } else {
            int n = irange(1, 6);
            double step = uni(1e4, 4e5);
            for (int i = 0; i < n; ++i) {
                x += step * cos(dir); y += step * sin(dir);
                pts.emplace_back(coord_t(x), coord_t(y));
                if (irange(0, 9) == 0) pts.emplace_back(pts.back()); // repeated point
            }
            dir += uni(-1.2, 1.2);
        }
    }
    Polyline pl(pts);
    return pl;
}

int main()
{
    const double tols[] = { 1000., 2500., 5000., 10000. };
    for (int c = 0; c < 1500; ++c) {
        Polyline pl = make_polyline(c);
        Polyline raw = pl;
        if (c % 4 != 3) pl.simplify_by_fitting_arc(tols[c % 4]);
        printf("C");
        out_pl(pl);
        printf("\nR");
        out_pl(Polyline(raw.points));
        printf("\n");
        // split_at(point): a vertex, a point near a segment, a point near an arc, a far point
        for (int q = 0; q < 4 && !pl.points.empty(); ++q) {
            Point qp;
            if (q == 0) qp = pl.points[irange(0, int(pl.points.size()) - 1)];
            else if (q == 3) qp = Point(coord_t(uni(-1.2e7, 1.2e7)), coord_t(uni(-1.2e7, 1.2e7)));
            else {
                const Point &a = raw.points[irange(0, int(raw.points.size()) - 1)];
                qp = Point(a.x() + irange(-3000, 3000), a.y() + irange(-3000, 3000));
            }
            Point io = qp;
            Polyline p1, p2;
            pl.split_at(io, &p1, &p2);
            printf("SP %d %d -> %d %d", qp.x(), qp.y(), io.x(), io.y());
            out_pl(p1); out_pl(p2);
            printf("\n");
        }
        // split_at_index
        for (int q = 0; q < 3 && !pl.points.empty(); ++q) {
            size_t idx = q == 0 ? 0 : (q == 1 ? pl.points.size() - 1 : size_t(irange(0, int(pl.points.size()) - 1)));
            Polyline p1, p2;
            bool ok = pl.split_at_index(idx, &p1, &p2);
            printf("SI %zu -> %d", idx, int(ok));
            out_pl(p1); out_pl(p2);
            printf("\n");
        }
        // split_at_length
        double L = pl.length();
        for (int q = 0; q < 4; ++q) {
            double len = q == 0 ? 0. : (q == 1 ? L : (q == 2 ? L * uni(0, 1) : L * uni(0, 1) + 0.25));
            Polyline p1, p2;
            bool ok = pl.split_at_length(len, &p1, &p2);
            printf("SL %016llx -> %d", db(len), int(ok));
            out_pl(p1); out_pl(p2);
            printf("\n");
        }
        // clip_end / clip_start (Polyline.cpp:45-100) and reverse (Polyline.cpp:34-50) with the fitting result
        for (int q = 0; q < 2; ++q) {
            double dist = q == 0 ? L * uni(0, 0.6) : uni(0, 4e4);
            Polyline ce = pl; ce.clip_end(dist);
            printf("CE %016llx ->", db(dist)); out_pl(ce); printf("\n");
            Polyline cs = pl; cs.clip_start(dist);
            printf("CS %016llx ->", db(dist)); out_pl(cs); printf("\n");
        }
        { Polyline rv = pl; rv.reverse(); printf("RV ->"); out_pl(rv); printf("\n"); }
        // append(const Polyline&) and append(Polyline&&) (Polyline.cpp:358-404) of the two halves of a split
        if (!pl.points.empty()) {
            Polyline p1, p2;
            pl.split_at_length(L * uni(0, 1), &p1, &p2);
            Polyline a = p1; a.append(p2);
            printf("AP"); out_pl(p1); out_pl(p2); printf(" ->"); out_pl(a); printf("\n");
            Polyline m = p2; Polyline src = p1; m.append(std::move(src));
            printf("AM"); out_pl(p2); out_pl(p1); printf(" ->"); out_pl(m); printf("\n");
        }
        // extend_start on the plain points (Polyline.cpp:111-116)
        if (raw.points.size() >= 2) {
            Polyline e(raw.points);
            double dist = uni(-2e5, 2e5);
            e.extend_start(dist);
            printf("ES %016llx ->", db(dist));
            out_pl(Polyline(e.points));
            printf("\n");
        }
        // extend_end on the plain points (needs two points)
        if (raw.points.size() >= 2) {
            Polyline e(raw.points);
            double dist = uni(-2e5, 2e5);
            e.extend_end(dist);
            printf("EE %016llx ->", db(dist));
            out_pl(Polyline(e.points));
            printf("\n");
        }
        // remove_same_neighbor(Polyline&)
        {
            Polyline r(raw.points);
            bool erased = remove_same_neighbor(r);
            printf("RN -> %d", int(erased));
            out_pl(Polyline(r.points));
            printf("\n");
        }
    }
    // remove_same_neighbor(Polylines&)
    for (int c = 0; c < 300; ++c) {
        Polylines pls;
        int n = irange(0, 5);
        for (int k = 0; k < n; ++k) {
            Points pts;
            int m = irange(0, 6);
            Point p(irange(-50, 50), irange(-50, 50));
            for (int i = 0; i < m; ++i) {
                if (irange(0, 2) != 0) p = Point(p.x() + irange(-2, 2), p.y() + irange(-2, 2));
                pts.push_back(p);
            }
            pls.emplace_back(pts);
        }
        // polylines_merge (Polyline.hpp:230-247) of the first two vectors, all four flag combinations
        if (pls.size() >= 2) {
            for (int f = 0; f < 4; ++f) {
                Points dst = pls[0].points, src = pls[1].points;
                polylines_merge(dst, (f & 1) != 0, std::move(src), (f & 2) != 0);
                printf("PM %d %d", f & 1, (f >> 1) & 1); out_pl(pls[0]); out_pl(pls[1]); printf(" ->"); out_pl(Polyline(dst)); printf("\n");
            }
        }
        printf("RNS %zu", pls.size());
        for (const Polyline &p : pls) out_pl(p);
        bool erased = remove_same_neighbor(pls);
        printf(" -> %d %zu", int(erased), pls.size());
        for (const Polyline &p : pls) out_pl(p);
        printf("\n");
    }
}
