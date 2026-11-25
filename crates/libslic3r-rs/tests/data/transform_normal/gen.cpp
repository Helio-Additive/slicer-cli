// Native vectors for tests/transform_normal_native_parity.rs: the pin's Geometry::transform_normal
// (Geometry.hpp:382-393, both overloads: Vec3d and Vec3f normals), compiled with the engine's flags
// (clang, arm64, -O3, Eigen), on deterministic transforms (Geometry::assemble_transform with
// rotations, non-uniform scales and mirrors, raw random linear parts, and degenerate ones).
//
// Build: target/parity-tools/build.sh <this file> <output binary>; run: <binary> > vectors.txt
// Format, one case per line, every double as its 16 hex digits, a float as its 8:
//   d <16 doubles: the 4x4 matrix row by row> <3 doubles: normal> -> <3 doubles: n> <3 doubles: result>
//   f <16 doubles> <3 floats: normal> -> <3 doubles: result>
// where n is `transform.linear().inverse().transpose() * normal` evaluated as Geometry.hpp:384 does.
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Geometry.hpp"

using namespace Slic3r;

static uint64_t g_state = 0x9E3779B97F4A7C15ULL;
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

static Transform3d rnd_transform(int i) {
    const int kind = i % 4;
    if (kind == 3) {
        Transform3d t = Transform3d::Identity();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c) t.matrix()(r, c) = range(-3, 3);
        if (xs() % 10 == 0) t.matrix()(2, 0) = t.matrix()(0, 0), t.matrix()(2, 1) = t.matrix()(0, 1), t.matrix()(2, 2) = t.matrix()(0, 2); // singular
        return t;
    }
    Vec3d rot(range(-3.2, 3.2), range(-3.2, 3.2), range(-3.2, 3.2));
    Vec3d sc = kind == 0 ? Vec3d::Constant(range(0.1, 5)) : Vec3d(range(0.01, 5), range(0.01, 5), range(0.01, 5));
    Vec3d mirror(xs() % 2 ? -1 : 1, xs() % 3 ? 1 : -1, 1);
    return Geometry::assemble_transform(Vec3d(range(-100, 100), range(-100, 100), range(-100, 100)), rot, sc, mirror);
}

int main() {
    for (int i = 0; i < 4000; ++i) {
        Transform3d t = rnd_transform(i);
        Vec3d normal(range(-1, 1), range(-1, 1), range(-1, 1));
        if (i % 7 == 0) normal = normal.normalized();
        if (i % 50 == 0) normal = Vec3d(range(-1e-6, 1e-6), range(-1e-6, 1e-6), range(-1e-6, 1e-6)); // len < EPSILON
        const Vec3d n = t.linear().inverse().transpose() * normal;
        const Vec3d res = Geometry::transform_normal(t, normal);
        std::printf("d");
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) pd(t.matrix()(r, c));
        pd(normal.x()); pd(normal.y()); pd(normal.z());
        std::printf(" ->");
        pd(n.x()); pd(n.y()); pd(n.z()); pd(res.x()); pd(res.y()); pd(res.z());
        std::printf("\n");
        const Vec3f nf = normal.cast<float>();
        const Vec3d resf = Geometry::transform_normal(t, nf);
        std::printf("f");
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) pd(t.matrix()(r, c));
        pf(nf.x()); pf(nf.y()); pf(nf.z());
        std::printf(" ->");
        pd(resf.x()); pd(resf.y()); pd(resf.z());
        std::printf("\n");
    }
    return 0;
}
