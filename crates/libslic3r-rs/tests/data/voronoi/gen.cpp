// Native vectors for tests/voronoi_native_parity.rs: the pin's Geometry::VoronoiDiagram
// (Geometry/Voronoi.hpp, Voronoi.cpp) compiled with the engine's flags, on deterministic inputs:
// closed polygons (random star outlines, thin slivers, grids of squares), sets with a few crossing or
// overlapping segments (which the issue detection flags and the repair by rotation handles), points,
// and points with segments. Per diagram: issue type, state, is_valid, num_vertices / num_edges /
// num_cells, every vertex (x, y, color), every edge (vertex0, twin, next, cell, primary, linear) and
// every cell (source index, source category); then encode_input_segment_endpoint and
// decode_input_segment_endpoint and segment_traits<Segment>::get.
//
// Build: target/parity-tools/build.sh <this file> <output binary>; run: <binary> | gzip -9n > vectors.txt.gz
// Format: `s <n> <segments: x0 y0 x1 y1 ...>` (segment diagram, repair on), `p <n> <points>` (points),
// `ps <np> <points> <ns> <segments>` (points and segments), each followed by `d` lines:
//   d <issue> <state> <valid> <nv> <ne> <nc> V <x y color, hex doubles> E <v0 twin next cell primary linear> C <idx cat>
// and `e <idx> <dir> <color> <decoded x y> <get x y>` lines for the endpoint coding.
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <boost/log/core.hpp>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Line.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polyline.hpp"
#include <functional>
#include "boost/polygon/voronoi.hpp"
// VoronoiDiagram::Segment is private; only the class sees this (its own includes are already in).
#define private public
#include "libslic3r/Geometry/Voronoi.hpp"
#undef private
#include "libslic3r/Geometry/Voronoi.cpp" // encode/decode_input_segment_endpoint are inline there

using namespace Slic3r;
using VD = Geometry::VoronoiDiagram;

static uint64_t g_state = 0x8BADF00D12345679ULL;
static uint64_t xs() {
    g_state ^= g_state << 13;
    g_state ^= g_state >> 7;
    g_state ^= g_state << 17;
    return g_state;
}
static double unit() { return double(xs() >> 11) / 9007199254740992.0; }
static double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
static void pd(double v) { uint64_t u; std::memcpy(&u, &v, 8); std::printf(" %016" PRIx64, u); }
static long long idx_or(const void *p, const void *base, size_t sz) { return p ? (long long)(((const char *)p - (const char *)base) / sz) : -1; }

static void dump(const VD &vd) {
    std::printf("d %d %d %d %zu %zu %zu V", int(vd.get_issue_type()), int(vd.get_state()), vd.is_valid() ? 1 : 0, vd.num_vertices(), vd.num_edges(), vd.num_cells());
    const auto &vs = vd.vertices();
    const auto &es = vd.edges();
    const auto &cs = vd.cells();
    for (const auto &v : vs) { pd(v.x()); pd(v.y()); std::printf(" %zu", size_t(v.color())); }
    std::printf(" E");
    for (const auto &e : es)
        std::printf(" %lld %lld %lld %lld %d %d", idx_or(e.vertex0(), vs.data(), sizeof(vs[0])), idx_or(e.twin(), es.data(), sizeof(es[0])),
                    idx_or(e.next(), es.data(), sizeof(es[0])), idx_or(e.cell(), cs.data(), sizeof(cs[0])), e.is_primary() ? 1 : 0, e.is_linear() ? 1 : 0);
    std::printf(" C");
    for (const auto &c : cs) std::printf(" %zu %d", size_t(c.source_index()), int(c.source_category()));
    std::printf("\n");
}

static Polygon star(double cx, double cy, double r0, double r1, int n) {
    Polygon p;
    for (int i = 0; i < n; ++i) {
        double a = 2. * M_PI * i / n + range(-0.1, 0.1), r = range(r0, r1);
        p.points.emplace_back(coord_t(cx + r * cos(a)), coord_t(cy + r * sin(a)));
    }
    return p;
}

int main() {
    // Voronoi.cpp logs the issues it finds (BOOST_LOG_TRIVIAL); keep them out of the vectors.
    boost::log::core::get()->set_logging_enabled(false);
    for (int tc = 0; tc < 400; ++tc) {
        Lines lines;
        const int kind = tc % 5;
        if (kind == 0 || kind == 1) {
            Polygon p = star(range(-1e6, 1e6), range(-1e6, 1e6), kind == 0 ? 2e5 : 1e3, kind == 0 ? 1e6 : 5e5, 3 + int(xs() % 30));
            for (const Line &l : p.lines()) lines.push_back(l);
            if (xs() % 2) { Polygon h = star(p.points[0].x() * 0 + range(-1e4, 1e4), range(-1e4, 1e4), 1e3, 5e4, 3 + int(xs() % 8)); (void)h; }
        } else if (kind == 2) {
            // thin sliver
            coord_t x0 = coord_t(range(-1e6, 1e6)), y0 = coord_t(range(-1e6, 1e6)), len = coord_t(range(1e3, 1e6)), w = coord_t(range(1, 50));
            Polygon p({ Point(x0, y0), Point(x0 + len, y0 + coord_t(range(-3, 3))), Point(x0 + len, y0 + w), Point(x0, y0 + w) });
            for (const Line &l : p.lines()) lines.push_back(l);
        } else if (kind == 3) {
            // grid of squares
            int n = 1 + int(xs() % 4);
            coord_t s = coord_t(range(1e3, 1e5));
            for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) {
                Polygon p({ Point(i * 3 * s, j * 3 * s), Point(i * 3 * s + s, j * 3 * s), Point(i * 3 * s + s, j * 3 * s + s), Point(i * 3 * s, j * 3 * s + s) });
                for (const Line &l : p.lines()) lines.push_back(l);
            }
        } else {
            // a polygon plus a few segments crossing it or each other (invalid input boost does not expect)
            Polygon p = star(0, 0, 1e5, 4e5, 4 + int(xs() % 10));
            for (const Line &l : p.lines()) lines.push_back(l);
            int extra = 1 + int(xs() % 2);
            for (int k = 0; k < extra; ++k)
                lines.emplace_back(Point(coord_t(range(-5e5, 5e5)), coord_t(range(-5e5, 5e5))), Point(coord_t(range(-5e5, 5e5)), coord_t(range(-5e5, 5e5))));
        }
        std::printf("s %zu", lines.size());
        for (const Line &l : lines) std::printf(" %d %d %d %d", l.a.x(), l.a.y(), l.b.x(), l.b.y());
        std::printf("\n");
        VD vd;
        vd.construct_voronoi(lines.begin(), lines.end());
        dump(vd);
        // endpoint coding over these segments
        for (int k = 0; k < 4; ++k) {
            size_t i = xs() % lines.size();
            boost::polygon::direction_1d dir = (xs() % 2) ? boost::polygon::HIGH : boost::polygon::LOW;
            auto color = Geometry::encode_input_segment_endpoint(i, dir);
            Point dec = Geometry::decode_input_segment_endpoint(color, lines.begin(), lines.end());
            VD::Segment seg(lines[i].a, lines[i].b);
            Point g = boost::polygon::segment_traits<VD::Segment>::get(seg, dir);
            std::printf("e %zu %d %zu %d %d %d %d\n", i, dir.to_int() ? 1 : 0, size_t(color), dec.x(), dec.y(), g.x(), g.y());
        }
    }
    for (int tc = 0; tc < 100; ++tc) {
        Points pts;
        int n = 1 + int(xs() % 40);
        for (int i = 0; i < n; ++i) pts.emplace_back(coord_t(range(-1e6, 1e6)), coord_t(range(-1e6, 1e6)));
        std::printf("p %zu", pts.size());
        for (const Point &p : pts) std::printf(" %d %d", p.x(), p.y());
        std::printf("\n");
        VD vd;
        vd.construct_voronoi(pts.begin(), pts.end());
        dump(vd);
        Polygon poly = star(range(-1e5, 1e5), range(-1e5, 1e5), 2e6, 3e6, 3 + int(xs() % 12));
        Lines lines = poly.lines();
        std::printf("ps %zu", pts.size());
        for (const Point &p : pts) std::printf(" %d %d", p.x(), p.y());
        std::printf(" %zu", lines.size());
        for (const Line &l : lines) std::printf(" %d %d %d %d", l.a.x(), l.a.y(), l.b.x(), l.b.y());
        std::printf("\n");
        VD vd2;
        vd2.construct_voronoi(pts.begin(), pts.end(), lines.begin(), lines.end());
        dump(vd2);
    }
    return 0;
}
