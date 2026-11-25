// Native vectors for EdgeGrid::Grid (R948): the create overloads (EdgeGrid.cpp:28-140, EdgeGrid.hpp:111) with
// create_from_m_contours (141-335) and visit_intersect_line_impl (EdgeGrid.hpp:172-289) through the cells and
// the cell data they leave, intersecting_edges (1414-1450), calculate_sdf (672-980), cell_inside_or_crossing
// (EdgeGrid.hpp:420-431) and closest_point_signed_distance (1047-1176) on random contours, a part of them
// snapped to the grid so that lines run through cell corners and along cell edges.
// Usage: gen <output file>
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <vector>
#include <algorithm>
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/EdgeGrid.hpp"

using namespace Slic3r;

static uint64_t g_seed = 0x2545F4914F6CDD1DULL;
static uint64_t rnd_u() { g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL; return g_seed >> 11; }
static double rnd() { return double(rnd_u()) / double(1ULL << 53); }
static int rnd_i(int n) { return int(rnd_u() % uint64_t(n)); }

static uint64_t fnv(uint64_t h, uint64_t v) { h ^= v; h *= 0x100000001b3ULL; return h; }

// Access to the protected members of the pin's Grid.
struct G : public EdgeGrid::Grid {
    uint64_t hash_cells() const { uint64_t h = 0xcbf29ce484222325ULL; for (const Cell &c : m_cells) { h = fnv(h, c.begin); h = fnv(h, c.end); } return h; }
    uint64_t hash_data() const { uint64_t h = 0xcbf29ce484222325ULL; for (const auto &d : m_cell_data) { h = fnv(h, d.first); h = fnv(h, d.second); } return h; }
    uint64_t hash_sdf() const { uint64_t h = 0xcbf29ce484222325ULL; for (float f : m_signed_distance_field) { uint32_t u; memcpy(&u, &f, 4); h = fnv(h, u); } return h; }
    size_t ncells() const { return m_cells.size(); }
    size_t ndata() const { return m_cell_data.size(); }
    size_t nsdf() const { return m_signed_distance_field.size(); }
    bool cic(int r, int c) const { return cell_inside_or_crossing(r, c); }
    size_t nrows() const { return m_rows; }
    size_t ncols() const { return m_cols; }
    const BoundingBox &bb() const { return m_bbox; }
    size_t contour_index(const EdgeGrid::Contour *c) const { return size_t(c - m_contours.data()); }
};

static Points star(double cx, double cy, int n, double rmin, double rmax, int snap, coord_t res)
{
    std::vector<double> ang(n);
    for (int i = 0; i < n; ++i) ang[i] = rnd() * 2. * M_PI;
    std::sort(ang.begin(), ang.end());
    Points p;
    for (int i = 0; i < n; ++i) {
        double r = rmin + (rmax - rmin) * rnd();
        double x = cx + r * std::cos(ang[i]);
        double y = cy + r * std::sin(ang[i]);
        if (snap) { double q = snap == 1 ? double(res) : double(res) / 2.; x = std::round(x / q) * q; y = std::round(y / q) * q; }
        p.emplace_back(coord_t(x), coord_t(y));
    }
    if (n > 3 && rnd_i(6) == 0) p.insert(p.begin() + 1, p[1]);  // a repeated point
    return p;
}

int main(int argc, char **argv)
{
    if (argc < 2) return 1;
    FILE *f = fopen(argv[1], "w");
    const int NCASES = 700;
    const coord_t resolutions[] = { 2000, 5000, 7919, 20000, 33333, 50000 };
    for (int k = 0; k < NCASES; ++k) {
        int mode = k % 7;
        coord_t res = resolutions[rnd_i(6)];
        int snap = rnd_i(3);
        bool preset = rnd_i(4) == 0;
        // lists: (kind, points). kind: closed/open hint (modes 2-4) or 0 = contour / 1 = hole (modes 5, 6)
        std::vector<std::pair<int, Points>> lists;
        int nl = 1 + rnd_i(4);
        for (int i = 0; i < nl; ++i) {
            double cx = (rnd() - 0.5) * 300000., cy = (rnd() - 0.5) * 300000.;
            double rmax = 10000. + rnd() * 120000.;
            int n = 3 + rnd_i(14);
            Points pts = star(cx, cy, n, rmax * 0.3, rmax, snap, res);
            int kind = 0;
            if (mode == 2 || mode == 4) { kind = rnd_i(2); if (rnd_i(3) == 0) pts.push_back(pts.front()); }
            if (mode == 5 || mode == 6) {
                kind = (i == 0 || (mode == 6 && rnd_i(3) == 0)) ? 0 : 1;
                if (kind == 1) { Polygon h(star(cx, cy, n, rmax * 0.05, rmax * 0.2, snap, res)); h.make_clockwise(); pts = h.points; }
                else { Polygon c(pts); c.make_counter_clockwise(); pts = c.points; }
            }
            if (mode == 0 && rnd_i(10) == 0) pts.resize(1);  // a one-point polygon
            lists.emplace_back(kind, pts);
        }
        if (mode == 0 && rnd_i(12) == 0) lists.clear();
        fprintf(f, "case %d mode %d res %d preset %d\n", k, mode, int(res), int(preset));
        G g;
        BoundingBox pre;
        if (preset) {
            pre = BoundingBox(Point(coord_t(-200000 - rnd_i(50000)), coord_t(-200000 - rnd_i(50000))), Point(coord_t(200000 + rnd_i(50000)), coord_t(200000 + rnd_i(50000))));
            fprintf(f, "bbox %d %d %d %d\n", int(pre.min.x()), int(pre.min.y()), int(pre.max.x()), int(pre.max.y()));
            g.set_bbox(pre);
        }
        fprintf(f, "lists %d\n", int(lists.size()));
        for (auto &l : lists) {
            fprintf(f, "pl %d %d", l.first, int(l.second.size()));
            for (auto &p : l.second) fprintf(f, " %d %d", int(p.x()), int(p.y()));
            fprintf(f, "\n");
        }
        Polygons polygons;
        std::vector<const Polygon*> pptrs;
        std::vector<Points> vpts;
        Polylines polylines;
        ExPolygon expoly;
        ExPolygons expolys;
        switch (mode) {
        case 0: for (auto &l : lists) polygons.emplace_back(l.second); g.create(polygons, res); break;
        case 1: for (auto &l : lists) polygons.emplace_back(l.second); for (auto &p : polygons) pptrs.push_back(&p); g.create(pptrs, res); break;
        case 2: for (auto &l : lists) vpts.push_back(l.second); g.create(vpts, res, true); break;
        case 3: for (auto &l : lists) vpts.push_back(l.second); g.create(vpts, res); break;
        case 4: for (auto &l : lists) { if (l.first) polylines.emplace_back(l.second); else polygons.emplace_back(l.second); } g.create(polygons, polylines, res); break;
        case 5: for (auto &l : lists) { if (l.first == 0) expoly.contour.points = l.second; else expoly.holes.emplace_back(l.second); } g.create(expoly, res); break;
        case 6: for (auto &l : lists) { if (l.first == 0 || expolys.empty()) { expolys.emplace_back(); expolys.back().contour.points = l.second; } else expolys.back().holes.emplace_back(l.second); } g.create(expolys, res); break;
        }
        const BoundingBox &bb = g.bb();
        fprintf(f, "grid %d %d %d %d %d %d %d %d %llu %llu\n", int(bb.min.x()), int(bb.min.y()), int(bb.max.x()), int(bb.max.y()),
            int(g.nrows()), int(g.ncols()), int(g.ncells()), int(g.ndata()), (unsigned long long)g.hash_cells(), (unsigned long long)g.hash_data());
        {
            auto ie = g.intersecting_edges();
            uint64_t h = 0xcbf29ce484222325ULL;
            for (auto &e : ie) { h = fnv(h, g.contour_index(e.first.first)); h = fnv(h, e.first.second); h = fnv(h, g.contour_index(e.second.first)); h = fnv(h, e.second.second); }
            fprintf(f, "ie %d %llu\n", int(ie.size()), (unsigned long long)h);
        }
        // cell_inside_or_crossing before the distance field
        auto dump_cic = [&](const char *tag) {
            fprintf(f, "%s", tag);
            for (int q = 0; q < 12; ++q) {
                int r = rnd_i(int(g.nrows()) + 4) - 2, c = rnd_i(int(g.ncols()) + 4) - 2;
                fprintf(f, " %d %d %d", r, c, int(g.cic(r, c)));
            }
            fprintf(f, "\n");
        };
        dump_cic("cic0");
        bool closed = (mode == 0 || mode == 1 || mode == 5 || mode == 6) && ! lists.empty() && g.ncells() <= 40000;
        if (closed) {
            g.calculate_sdf();
            fprintf(f, "sdf %d %llu\n", int(g.nsdf()), (unsigned long long)g.hash_sdf());
            dump_cic("cic1");
            fprintf(f, "cp");
            for (int q = 0; q < 8; ++q) {
                Point pt(coord_t(bb.min.x() + 16 + rnd() * double(bb.max.x() - bb.min.x() - 32)), coord_t(bb.min.y() + 16 + rnd() * double(bb.max.y() - bb.min.y() - 32)));
                coord_t radius = coord_t(res / 2 + rnd_i(int(res) * 3));
                auto r = g.closest_point_signed_distance(pt, radius);
                fprintf(f, " %d %d %d %lld %lld %a %a", int(pt.x()), int(pt.y()), int(radius), (long long)(r.valid() ? r.contour_idx : -1), (long long)(r.valid() ? r.start_point_idx : -1), r.distance, r.t);
            }
            fprintf(f, "\n");
        }
    }
    fclose(f);
    return 0;
}
