// Generator of the vectors of tests/layer_native_parity.rs: libslic3r's Layer.cpp at the pin
// (references/BambuStudio/src/libslic3r; the override layer has no copy of it).
//
// Build, with REPO = this repository, BUILD = a built tree of the Bambu Studio engine
// (docs/building.md, -DENGINE=bambu) and DATA = this directory: compile with the compile line the
// build uses for a libslic3r source (its defines and include directories, the override layer
// $REPO/libslic3r/bambustudio before the pin, -std=c++17 -O3 -DNDEBUG and the forced include of
// cgal_54_compat.hpp), then link with the build's libslic3r_core and the libraries it links:
//
//   c++ <the build's defines and include directories> -std=c++17 -O3 -DNDEBUG \
//       -include $REPO/libslic3r/bambustudio/cgal_54_compat.hpp -c $DATA/gen.cpp -o gen.o
//   c++ gen.o $BUILD/libslic3r_core.a <the libraries libslic3r_core links> -o gen
//   ./gen vectors.txt
//
// Layer.cpp is compiled into this file (ContinuitiousDistancer is local to it), with the protected
// constructor of Layer made public.
//
// Every line is "<what> <inputs> => <results>": the test reads the inputs back, calls the port and
// has to print the same results. Tokens are separated by one space; a float is its 8 hexadecimal
// digits, an integer decimal.
// The vectors were made on macOS arm64 with Apple clang and Eigen 5.0.1.

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

// Before the access specifiers are redefined: tbb declares nested classes ahead of their definition.
#include <tbb/blocked_range.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_unordered_set.h>
#include <tbb/concurrent_vector.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/spin_mutex.h>

#define private public
#define protected public
#include "libslic3r/Layer.cpp"
#undef private
#undef protected

using namespace Slic3r;

static FILE *g_out = nullptr;

static std::string hF(float v)
{
    uint32_t u;
    std::memcpy(&u, &v, 4);
    char b[16];
    std::snprintf(b, sizeof(b), "%08x", u);
    return b;
}
static std::string hI(long long v) { return std::to_string(v); }
static std::string hB(bool v) { return v ? "1" : "0"; }
static void out(const std::string &s) { std::fprintf(g_out, "%s\n", s.c_str()); }
static std::string PT(const Point &p) { return hI(p.x()) + " " + hI(p.y()); }

struct Lcg
{
    uint64_t s;
    explicit Lcg(uint64_t seed) : s(seed) {}
    uint32_t next()
    {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return uint32_t(s >> 33);
    }
    // An integer in [lo, hi].
    long long range(long long lo, long long hi) { return lo + (long long) (next() % uint32_t(hi - lo + 1)); }
};

// A wall of a layer: a ring of eight corners with points in between (closed: the first point is
// repeated at the end) or an open run of points.
static Points wall(Lcg &g, coord_t cx, coord_t cy, coord_t rx, coord_t ry, bool closed)
{
    Points corners;
    if (closed) {
        const coord_t cut_x = g.range(rx / 8, rx / 3), cut_y = g.range(ry / 8, ry / 3);
        corners = {Point(cx - rx + cut_x, cy - ry), Point(cx + rx - cut_x, cy - ry), Point(cx + rx, cy - ry + cut_y), Point(cx + rx, cy + ry - cut_y),
                   Point(cx + rx - cut_x, cy + ry), Point(cx - rx + cut_x, cy + ry), Point(cx - rx, cy + ry - cut_y), Point(cx - rx, cy - ry + cut_y)};
        corners.push_back(corners.front());
    } else {
        Point cur(cx - rx, cy - ry);
        const int n = (int) g.range(2, 6);
        for (int i = 0; i < n; ++i) {
            corners.push_back(cur);
            cur = Point(cur.x() + g.range(0, 2 * rx / n + 1), cur.y() + g.range(-ry / 2, ry / 2));
        }
    }
    // Points in between, so that runs of short segments add up.
    Points pts;
    for (size_t i = 0; i + 1 < corners.size(); ++i) {
        pts.push_back(corners[i]);
        const int k = (int) g.range(0, 3);
        for (int j = 1; j <= k; ++j)
            pts.emplace_back(corners[i].x() + (corners[i + 1].x() - corners[i].x()) * j / (k + 1), corners[i].y() + (corners[i + 1].y() - corners[i].y()) * j / (k + 1));
    }
    pts.push_back(corners.back());
    return pts;
}

// ---------------------------------------------------------------------------------------------
// ContinuitiousDistancer
// ---------------------------------------------------------------------------------------------

static void run_distancer()
{
    Lcg g(0x9e3779b97f4a7c15ULL);
    for (int c = 0; c < 300; ++c) {
        Points pts;
        const long long kind = g.range(0, 9);
        if (kind == 0) {
            // No line at all: no point or one.
            if (g.range(0, 1) == 0) pts.emplace_back(g.range(-500000, 500000), g.range(-500000, 500000));
        } else {
            pts = wall(g, g.range(-3000000, 3000000), g.range(-3000000, 3000000), g.range(20000, 900000), g.range(20000, 900000), kind < 6);
            // A point twice in a row: a line of no length.
            if (kind == 9) pts.insert(pts.begin() + 1, pts.front());
        }
        ContinuitiousDistancer distancer(pts);
        std::string            in = hI((long long) pts.size());
        for (const Point &p : pts) in += " " + PT(p);
        const int   nq = 12;
        std::string res;
        in += " " + hI(nq);
        for (int q = 0; q < nq; ++q) {
            Point base(g.range(-4000000, 4000000), g.range(-4000000, 4000000));
            if (!pts.empty() && g.range(0, 3) != 0) {
                // Near the wall, on it now and then.
                const Point &at = pts[g.range(0, (long long) pts.size() - 1)];
                const coord_t d  = g.range(0, 3) == 0 ? 0 : g.range(1, 80000);
                base             = Point(at.x() + g.range(-d, d), at.y() + g.range(-d, d));
            }
            const Vec2f point = base.cast<float>();
            in += " " + hF(point.x()) + " " + hF(point.y());
            res += (q == 0 ? "" : " ") + hF(distancer.distance_from_perimeter(point));
        }
        out("dist " + in + " => " + res);
    }
}

// ---------------------------------------------------------------------------------------------
// Layer::calculate_perimeter_continuity
// ---------------------------------------------------------------------------------------------

static std::string NODE(const LoopNode &n)
{
    std::string o = hI(n.node_id) + " " + PT(n.bbox.min) + " " + PT(n.bbox.max) + " " + hB(n.node_contour.is_loop) + " " + hI((long long) n.node_contour.pts.size());
    for (const Point &p : n.node_contour.pts) o += " " + PT(p);
    o += " " + hI((long long) n.node_contour.widths.size());
    for (coord_t w : n.node_contour.widths) o += " " + hI(w);
    return o;
}

// A node of a wall. The box is that of the points grown by a random margin, as the walls' boxes are
// grown by half a line width.
static LoopNode make_node(Lcg &g, int id, const Points &pts, bool is_loop)
{
    LoopNode node;
    node.node_id              = id;
    node.loop_id              = id;
    node.node_contour.pts     = pts;
    node.node_contour.is_loop = is_loop;
    coord_t w                 = g.range(30000, 60000);
    // The widths: one per point, or fewer (the first one stands in for the missing ones).
    const size_t nw = g.range(0, 5) == 0 ? (size_t) g.range(1, (long long) std::max<size_t>(1, pts.size())) : std::max<size_t>(1, pts.size());
    for (size_t i = 0; i < nw; ++i) {
        if (g.range(0, 3) == 0) w = g.range(30000, 60000);
        node.node_contour.widths.push_back(w);
    }
    BoundingBox bbox;
    for (const Point &p : pts) bbox.merge(p);
    if (pts.empty()) bbox = BoundingBox(Point(0, 0), Point(0, 0));
    const coord_t margin = g.range(0, 30000);
    node.bbox            = BoundingBox(Point(bbox.min.x() - margin, bbox.min.y() - margin), Point(bbox.max.x() + margin, bbox.max.y() + margin));
    return node;
}

static void run_continuity()
{
    Lcg g(0x2545f4914f6cdd1dULL);
    for (int c = 0; c < 300; ++c) {
        Layer layer(1, nullptr, 0.2, 0.4, 0.3);
        std::vector<LoopNode> prev_nodes;
        const int             n_walls = (int) g.range(1, 4);
        int                   prev_id = 0;
        for (int i = 0; i < n_walls; ++i) {
            const coord_t cx = g.range(-1500000, 1500000), cy = g.range(-1500000, 1500000);
            const coord_t rx = g.range(20000, 700000), ry = g.range(20000, 700000);
            const bool    closed = g.range(0, 2) != 0;
            const Points  pts    = wall(g, cx, cy, rx, ry, closed);
            layer.loop_nodes.emplace_back(make_node(g, i, pts, closed));
            // The walls of the layer below: this wall moved by less than a line width, by about one,
            // or by more; cut short now and then; or another wall altogether.
            const int n_prev = (int) g.range(0, 2);
            for (int j = 0; j < n_prev; ++j) {
                const long long kind = g.range(0, 5);
                Points          below;
                bool            below_closed = closed;
                if (kind == 5) {
                    below_closed = g.range(0, 1) == 0;
                    below        = wall(g, cx + g.range(-rx, rx), cy + g.range(-ry, ry), g.range(20000, 700000), g.range(20000, 700000), below_closed);
                } else {
                    const coord_t reach = kind < 2 ? 20000 : (kind < 4 ? 50000 : 150000);
                    const coord_t dx = g.range(-reach, reach), dy = g.range(-reach, reach);
                    for (const Point &p : pts) {
                        const coord_t jitter = g.range(0, 4) == 0 ? g.range(-40000, 40000) : 0;
                        below.emplace_back(p.x() + dx + jitter, p.y() + dy + jitter);
                    }
                    if (g.range(0, 4) == 0 && below.size() > 3) {
                        below.resize(below.size() - (size_t) g.range(1, 2));
                        below_closed = g.range(0, 1) == 0;
                    }
                }
                prev_nodes.emplace_back(make_node(g, prev_id++, below, below_closed));
            }
        }
        // A wall below with one point, and one with none that is not a loop.
        if (c % 23 == 5) prev_nodes.emplace_back(make_node(g, prev_id++, Points{layer.loop_nodes.front().node_contour.pts.front()}, c % 2 == 0));
        if (c % 29 == 7) {
            LoopNode empty = make_node(g, prev_id++, Points(), false);
            empty.bbox     = layer.loop_nodes.front().bbox;
            prev_nodes.emplace_back(empty);
        }

        std::string in = hI((long long) layer.loop_nodes.size());
        for (const LoopNode &n : layer.loop_nodes) in += " " + NODE(n);
        in += " " + hI((long long) prev_nodes.size());
        for (const LoopNode &n : prev_nodes) in += " " + NODE(n);

        layer.calculate_perimeter_continuity(prev_nodes);

        std::string res;
        for (const LoopNode &n : layer.loop_nodes) {
            res += (res.empty() ? "" : " ") + hI((long long) n.lower_node_id.size());
            for (int id : n.lower_node_id) res += " " + hI(id);
        }
        for (const LoopNode &n : prev_nodes) {
            res += " " + hI((long long) n.upper_node_id.size());
            for (int id : n.upper_node_id) res += " " + hI(id);
        }
        out("continuity " + in + " => " + res);
    }
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: gen <vectors.txt>\n");
        return 1;
    }
    g_out = std::fopen(argv[1], "w");
    if (g_out == nullptr) return 1;
    tbb::global_control one_thread(tbb::global_control::max_allowed_parallelism, 1);
    run_distancer();
    run_continuity();
    std::fclose(g_out);
    return 0;
}
