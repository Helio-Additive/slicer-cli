// Vectors for the route of FillMonotonicLineWGapFill: plan_monotonic_route (FillRectilinear.cpp:3247-3590),
// a static function, reached by including the pin's FillRectilinear.cpp.
// Build: target/parity-tools/build.sh <this file> <out binary>; run: <out binary> > vectors.txt
#include "libslic3r/Fill/FillRectilinear.cpp"
#include <cstdio>
#include <cstdint>
#include <cmath>

using namespace Slic3r;

static uint64_t g_state = 0x2545F4914F6CDD1DULL;
static uint32_t lcg() { g_state = g_state * 6364136223846793005ULL + 1442695040888963407ULL; return uint32_t(g_state >> 33); }
static int64_t rnd(int64_t lo, int64_t hi) { return lo + int64_t(lcg() % uint64_t(hi - lo + 1)); }

int main()
{
    const int n_cases = 200;
    for (int c = 0; c < n_cases; ++ c) {
        // the scan angle
        float scan_angle;
        switch (c % 4) {
        case 0: scan_angle = 0.f; break;
        case 1: scan_angle = float(M_PI / 4.); break;
        case 2: scan_angle = float(M_PI / 2.); break;
        default: scan_angle = float(double(rnd(0, 6283)) / 1000.); break;
        }
        const coord_t spacing = coord_t(rnd(30000, 50000));
        const double  link_max = 3. * double(spacing) * (double(rnd(80, 120)) / 100.);
        // islands of columns in the scan frame, lines alternating up and down
        const int n_islands = int(rnd(1, 3));
        Polylines lines;
        std::vector<size_t> island_end;
        coord_t x = coord_t(rnd(-2000000, 2000000));
        for (int isl = 0; isl < n_islands; ++ isl) {
            const int n_cols = int(rnd(0, 12));
            const coord_t ybase = coord_t(rnd(-1000000, 1000000));
            for (int k = 0; k < n_cols; ++ k) {
                const coord_t y0 = ybase + coord_t(rnd(-60000, 60000));
                const coord_t y1 = ybase + coord_t(rnd(400000, 1500000));
                Polyline pl;
                Point a(x, y0), b(x, y1);
                if (k % 2) std::swap(a, b);
                pl.points.push_back(a);
                if (rnd(0, 3) == 0) pl.points.push_back(Point(x, (y0 + y1) / 2));
                pl.points.push_back(b);
                // back to the world frame
                for (Point &p : pl.points) p.rotate(double(scan_angle));
                lines.push_back(pl);
                x += spacing;
            }
            island_end.push_back(lines.size());
            x += coord_t(rnd(2, 6)) * spacing;
        }
        if (rnd(0, 9) == 0) island_end.clear();
        // gaps near the ends of the lines (or anywhere)
        const int n_gaps = int(rnd(0, 7));
        ThickPolylines gaps;
        for (int g = 0; g < n_gaps; ++ g) {
            Point base;
            if (! lines.empty() && rnd(0, 4) != 0) {
                const Polyline &l = lines[size_t(rnd(0, int64_t(lines.size()) - 1))];
                base = rnd(0, 1) ? l.first_point() : l.last_point();
            } else
                base = Point(coord_t(rnd(-2000000, 2000000)), coord_t(rnd(-2000000, 2000000)));
            ThickPolyline tp;
            const int npts = int(rnd(1, 5));
            Point p = base + Point(coord_t(rnd(-3, 3)) * spacing / 2, coord_t(rnd(-3, 3)) * spacing / 2);
            for (int i = 0; i < npts; ++ i) {
                tp.points.push_back(p);
                p += Point(coord_t(rnd(-spacing, spacing)), coord_t(rnd(-spacing, spacing)));
            }
            if (npts >= 3 && rnd(0, 3) == 0) tp.points.push_back(tp.points.front());
            if (tp.points.size() >= 2) {
                tp.width.assign(2 * (tp.points.size() - 1), 0.);
            }
            tp.endpoints = std::make_pair(rnd(0, 1) == 1, rnd(0, 1) == 1);
            gaps.push_back(tp);
        }
        printf("case %d %zu %zu %a %a %zu", c, lines.size(), gaps.size(), link_max, double(scan_angle), island_end.size());
        for (size_t e : island_end) printf(" %zu", e);
        printf("\n");
        for (const Polyline &l : lines) {
            printf("L %zu", l.points.size());
            for (const Point &p : l.points) printf(" %lld %lld", (long long)p.x(), (long long)p.y());
            printf("\n");
        }
        for (const ThickPolyline &g : gaps) {
            printf("G %zu", g.points.size());
            for (const Point &p : g.points) printf(" %lld %lld", (long long)p.x(), (long long)p.y());
            printf("\n");
        }
        MonotonicRoute route = plan_monotonic_route(lines, gaps, link_max, scan_angle, island_end);
        for (size_t i = 0; i < route.gaps_at.size(); ++ i) {
            if (route.gaps_at[i].empty()) continue;
            printf("S %zu %zu", i, route.gaps_at[i].size());
            for (size_t g : route.gaps_at[i]) printf(" %zu", g);
            printf("\n");
        }
        printf("O %zu", route.leftover.size());
        for (size_t g : route.leftover) printf(" %zu", g);
        printf("\n");
        for (const ThickPolyline &g : gaps) {
            printf("P %zu", g.points.size());
            for (const Point &p : g.points) printf(" %lld %lld", (long long)p.x(), (long long)p.y());
            printf("\n");
        }
        printf("E\n");
    }
    return 0;
}
