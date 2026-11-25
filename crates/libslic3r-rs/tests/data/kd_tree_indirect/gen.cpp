// Native vectors for tests/kd_tree_indirect_native_parity.rs: the pin's KDTreeIndirect
// (KDTreeIndirect.hpp: build, find_closest_points<K>, find_closest_point, find_nearby_points by radius
// and by box), compiled with the engine's flags (clang, arm64, -O3, -ffp-contract=on), on the tree
// types the pin instantiates: KDTreeIndirect<2, double> (ShortestPath.cpp:105, 516) and
// KDTreeIndirect<3, float> (SeamPlacer.cpp:279, 612; SeamPlacer.hpp:115). Queries include midpoints
// of point pairs, where the two points are equidistant in exact arithmetic and the rounding of
// `dist += d * d` (KDTreeIndirect.hpp:226-227, 301-302) decides.
//
// Build: target/parity-tools/build.sh <this file> <output binary>; run: <binary> | gzip -9n > vectors.txt.gz
// Format (every double as its 16 hex digits, a float as its 8):
//   tree <d2|f3> <n> <coordinates, point by point> nodes <m> <m node indices>
//   q <coordinates of the query> <radius> <filter 0|1> -> c1 <idx> c3 <i0> <i1> <i2> nr <count> <idx...> nb <count> <idx...>
// c1 = find_closest_point, c3 = find_closest_points<3>, nr = find_nearby_points(center, radius),
// nb = find_nearby_points(center - radius, center + radius) (the box overload); with filter 1 every
// query takes the filter `idx % 5 != 0`.
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <limits>
#include <vector>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Utils.hpp"
// m_nodes is private; the std and libslic3r headers are already in, so only the tree class sees this.
#define private public
#include "libslic3r/KDTreeIndirect.hpp"
#undef private

using namespace Slic3r;

static uint64_t g_state = 0x2545F4914F6CDD1DULL;
static uint64_t xs() {
    g_state ^= g_state << 13;
    g_state ^= g_state >> 7;
    g_state ^= g_state << 17;
    return g_state;
}
static double unit() { return double(xs() >> 11) / 9007199254740992.0; }
static double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
static void pd(double v) { uint64_t u; std::memcpy(&u, &v, 8); std::printf(" %016" PRIx64, u); }
static void pf(float v) { uint32_t u; std::memcpy(&u, &v, 4); std::printf(" %08x", u); }
static void pz(size_t v) { std::printf(" %" PRId64, v == size_t(-1) ? int64_t(-1) : int64_t(v)); }
static void plist(const char *tag, const std::vector<size_t> &v) {
    std::printf(" %s %zu", tag, v.size());
    for (size_t i : v) pz(i);
}

static void run_d2(int tc) {
    const int kind = tc % 3;
    const size_t n = 1 + xs() % (tc % 7 == 0 ? 8 : 160);
    std::vector<Vec2d> pts(n);
    for (auto &p : pts) {
        if (kind == 0) p = Vec2d(range(-1000, 1000), range(-1000, 1000));
        else if (kind == 1) p = Vec2d(double(int64_t(xs() % 20000001) - 10000000), double(int64_t(xs() % 20000001) - 10000000)); // scaled integer points
        else p = Vec2d(range(-1, 1) * 3e7, range(-1, 1) * 3e7);
    }
    if (n > 4 && xs() % 4 == 0) pts[1] = pts[0]; // duplicates
    auto coordinate_fn = [&pts](size_t idx, size_t dimension) -> double { return pts[idx](dimension); };
    KDTreeIndirect<2, double, decltype(coordinate_fn)> tree(coordinate_fn, n);
    std::printf("tree d2 %zu", n);
    for (auto &p : pts) { pd(p.x()); pd(p.y()); }
    std::printf(" nodes %zu", tree.m_nodes.size());
    for (size_t v : tree.m_nodes) pz(v);
    std::printf("\n");
    for (int qi = 0; qi < 24; ++qi) {
        Vec2d q;
        if (qi % 3 == 0) q = Vec2d(range(-1100, 1100), range(-1100, 1100)) * (kind == 0 ? 1. : 3e4);
        else {
            const Vec2d &a = pts[xs() % n], &b = pts[xs() % n];
            q = 0.5 * (a + b);
            if (qi % 3 == 2) q += Vec2d(range(-1e-9, 1e-9), range(-1e-9, 1e-9)) * q.norm();
        }
        const double r = range(0, kind == 0 ? 300 : 9e6);
        const bool filt = qi % 4 == 3;
        auto filter = [filt](size_t idx) { return !filt || idx % 5 != 0; };
        size_t c1 = find_closest_point(tree, q, filter);
        auto c3 = find_closest_points<3>(tree, q, filter);
        auto nr = find_nearby_points(tree, q, r, filter);
        Vec2d bmin = q - Vec2d(r, r), bmax = q + Vec2d(r, r);
        auto nb = find_nearby_points(tree, bmin, bmax, filter);
        std::printf("q");
        pd(q.x()); pd(q.y()); pd(r);
        std::printf(" %d ->", filt ? 1 : 0);
        std::printf(" c1"); pz(c1);
        std::printf(" c3"); pz(c3[0]); pz(c3[1]); pz(c3[2]);
        plist("nr", nr);
        plist("nb", nb);
        std::printf("\n");
    }
}

static void run_f3(int tc) {
    const int kind = tc % 2;
    const size_t n = 1 + xs() % (tc % 7 == 0 ? 8 : 200);
    std::vector<Vec3f> pts(n);
    for (auto &p : pts) {
        if (kind == 0) p = Vec3f(float(range(-120, 120)), float(range(-120, 120)), float(range(0, 200)));
        else p = Vec3f(float(range(-5, 5)), float(range(-5, 5)), float(range(0, 1))); // a dense patch, as seam candidates
    }
    if (n > 4 && xs() % 4 == 0) pts[2] = pts[1];
    auto coordinate_fn = [&pts](size_t idx, size_t dimension) -> float { return pts[idx](dimension); };
    KDTreeIndirect<3, float, decltype(coordinate_fn)> tree(coordinate_fn, n);
    std::printf("tree f3 %zu", n);
    for (auto &p : pts) { pf(p.x()); pf(p.y()); pf(p.z()); }
    std::printf(" nodes %zu", tree.m_nodes.size());
    for (size_t v : tree.m_nodes) pz(v);
    std::printf("\n");
    for (int qi = 0; qi < 24; ++qi) {
        Vec3f q;
        if (qi % 3 == 0) q = kind == 0 ? Vec3f(float(range(-130, 130)), float(range(-130, 130)), float(range(-10, 210)))
                                       : Vec3f(float(range(-6, 6)), float(range(-6, 6)), float(range(-0.5, 1.5)));
        else {
            const Vec3f &a = pts[xs() % n], &b = pts[xs() % n];
            q = 0.5f * (a + b);
        }
        const float r = float(range(0, kind == 0 ? 40 : 1.5));
        const bool filt = qi % 4 == 3;
        auto filter = [filt](size_t idx) { return !filt || idx % 5 != 0; };
        size_t c1 = find_closest_point(tree, q, filter);
        auto c3 = find_closest_points<3>(tree, q, filter);
        auto nr = find_nearby_points(tree, q, r, filter);
        Vec3f bmin = q - Vec3f(r, r, r), bmax = q + Vec3f(r, r, r);
        auto nb = find_nearby_points(tree, bmin, bmax, filter);
        std::printf("q");
        pf(q.x()); pf(q.y()); pf(q.z()); pf(r);
        std::printf(" %d ->", filt ? 1 : 0);
        std::printf(" c1"); pz(c1);
        std::printf(" c3"); pz(c3[0]); pz(c3[1]); pz(c3[2]);
        plist("nr", nr);
        plist("nb", nb);
        std::printf("\n");
    }
}

int main() {
    for (int tc = 0; tc < 300; ++tc) run_d2(tc);
    for (int tc = 0; tc < 300; ++tc) run_f3(tc);
    return 0;
}
