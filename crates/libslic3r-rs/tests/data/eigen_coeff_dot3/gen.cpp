// Native vectors for tests/eigen_coeff_dot3_native_parity.rs: the Eigen expressions behind every call
// site of the port's `geometry::eigen::coeff_dot3` (the third row of a lazy product evaluated alone,
// `e0 + (e1 + e2)`), and the transposed product it does not fit, compiled with the engine's flags
// (clang, arm64, -O3, -ffp-contract=on, Eigen 5.0.1) on random matrices with no structure (every
// coefficient random, so the order of the three-term sums shows):
//   Matrix3d * Matrix3d, Matrix3d * Vec3d, Transform3d * Transform3d, Transform3d::inverse(),
//   Transform3d::translate(Vec3d), Transform3d::rotate(Matrix3d), transform(Linef3, Transform3d)
//   (Line.cpp:10-20), BoundingBoxf3::transformed (BoundingBox.cpp:265-294), Matrix3d.transpose() * Vec3d.
//
// Build: target/parity-tools/build.sh <this file> <output binary>; run: <binary> | gzip -9n > vectors.txt.gz
// Format, one case per line, every double as its 16 hex digits:
//   c <a: 9, row by row> <b: 9> <v: 3> <w: 3> <t: 16, row by row> <u: 16> ->
//     <a*b: 9> <a*v: 3> <t*u: 16> <t.inverse(): 16> <t.translate(v): 16> <t.rotate(a): 16>
//     <transform(Linef3(v, w), t): 6> <BoundingBoxf3(min(v,w), max(v,w)).transformed(t): 6> <a.transpose()*v: 3>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/BoundingBox.hpp"

using namespace Slic3r;

static uint64_t g_state = 0xD1B54A32D192ED03ULL;
static uint64_t xs() {
    g_state ^= g_state << 13;
    g_state ^= g_state >> 7;
    g_state ^= g_state << 17;
    return g_state;
}
static double unit() { return double(xs() >> 11) / 9007199254740992.0; }
static double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
static void pd(double v) { uint64_t u; std::memcpy(&u, &v, 8); std::printf(" %016" PRIx64, u); }
static void pm3(const Matrix3d &m) { for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) pd(m(r, c)); }
static void pv3(const Vec3d &v) { pd(v.x()); pd(v.y()); pd(v.z()); }
static void pt(const Transform3d &t) { for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) pd(t.matrix()(r, c)); }

static Matrix3d rnd_m3(double s) {
    Matrix3d m;
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) m(r, c) = range(-s, s);
    return m;
}
static Transform3d rnd_t(double s) {
    Transform3d t = Transform3d::Identity();
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) t.matrix()(r, c) = range(-s, s);
    for (int r = 0; r < 3; ++r) t.matrix()(r, 3) = range(-200, 200);
    return t;
}

int main() {
    for (int i = 0; i < 1200; ++i) {
        const double s = i % 3 == 0 ? 1. : (i % 3 == 1 ? 10. : 0.1);
        const Matrix3d a = rnd_m3(s), b = rnd_m3(1.);
        const Vec3d v(range(-100, 100), range(-100, 100), range(-100, 100));
        const Vec3d w(range(-100, 100), range(-100, 100), range(-100, 100));
        const Transform3d t = rnd_t(s), u = rnd_t(1.);
        std::printf("c");
        pm3(a); pm3(b); pv3(v); pv3(w); pt(t); pt(u);
        std::printf(" ->");
        const Matrix3d ab = a * b;
        pm3(ab);
        const Vec3d av = a * v;
        pv3(av);
        const Transform3d tu = t * u;
        pt(tu);
        const Transform3d ti = t.inverse();
        pt(ti);
        Transform3d tt = t;
        tt.translate(v);
        pt(tt);
        Transform3d tr = t;
        tr.rotate(a);
        pt(tr);
        const Linef3 l = transform(Linef3(v, w), t);
        pv3(l.a); pv3(l.b);
        const BoundingBoxf3 bb(v.cwiseMin(w), v.cwiseMax(w));
        const BoundingBoxf3 tb = bb.transformed(t);
        pv3(tb.min); pv3(tb.max);
        const Vec3d atv = a.transpose() * v;
        pv3(atv);
        std::printf("\n");
    }
    return 0;
}
