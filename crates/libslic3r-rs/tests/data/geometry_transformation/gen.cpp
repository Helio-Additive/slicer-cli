// Generator of the vectors of tests/geometry_transformation_native_parity.rs: libslic3r's
// Geometry.cpp transformation functions (assemble_transform, extract_euler_angles,
// rotation_transform, Transformation, TransformationSVD, volume_to_bed_transformation,
// rotation_xyz_diff, mat_around_a_point_rotate, generate_transform), BoundingBoxf3::transformed
// (BoundingBox.cpp) and the Eigen operations they are made of, at the pin
// (references/BambuStudio/src/libslic3r; the override layer has no copy of these files).
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
//   ./gen | gzip -9 -n > $DATA/vectors.txt.gz
//
// The vectors in this directory were made on macOS arm64 with Apple clang (its default
// -ffp-contract=on) and Eigen 5.0.1. The functions of Geometry.cpp and BoundingBox.cpp run from
// libslic3r_core as slicer_cli runs them; the Eigen operations of the "q" lines are instantiated
// here, from the same headers with the same flags.
//
// The arithmetic of this file (building the inputs) is compiled with
// `#pragma clang fp contract(off)` (after the includes), so its own expressions are evaluated as
// the Rust test writes them.
//
// Line format: `<tag> <case> <payload>`. The payload is the result: doubles as their 16
// hexadecimal digits, booleans 0/1, separated by one space. For the first FULL cases of every
// tag the payload is written out; for the others it is `#` and the 64-bit FNV-1a hash of the
// payload text, which the Rust test computes from its own payload. Run with FULL=1 in the
// environment to write every payload out (for finding a difference; not committed).
//
// Tags: the matrices of the cases come from nine families, made by `make_case` below (random
// affine with mirrors, pure rotations, rotations from Euler angles including multiples of 90
// degrees, assembled transforms, pure scales, near-singular, general (skewed), signed permutations
// times scales, rotations by tiny angles); the tags of their lines end in the family's name
// (`a.aff`, ...). For every matrix a line `m` with the matrix (rows 0-2, columns 0-3) and four
// lines:
//   a  decomposition: JacobiSVD<Matrix3d> U V singular values; computeRotationScaling R S;
//      get_rotation, get_scaling_factor, get_mirror, get_rotation_by_quaternion, has_skew,
//      is_left_handed, is_scaling_uniform; extract_euler_angles(Transform3d) and of R;
//      TransformationSVD u s v and flags; get_rotation_matrix, get_scaling_factor_matrix,
//      get_mirror_matrix
//   b  setters and builders: set_rotation, set_scaling_factor (vector, axis), set_mirror (vector,
//      axis), set_rotation_matrix, rotation_transform, assemble_transform, translation_transform,
//      scale_transform (vector, double)
//   c  resets: reset_rotation, reset_scaling_factor, reset_skew, reset_mirror, the 15 get_matrix
//      flag combinations other than all false, get_matrix_no_offset, get_matrix_no_scaling_factor
//   d  products and others: Transformation * Transformation, volume_to_bed_transformation (of the
//      identity when the decomposition has a zero or NaN scale or mirror, which its
//      set_scaling_factor asserts against),
//      rotation_transform(get_rotation()), BoundingBoxf3::transformed, Transform3d * Vec3d,
//      Transform3d::inverse, linear().determinant(), mat_around_a_point_rotate
// and lines of vectors:
//   v  rotation_from_two_vectors (axis, angle, matrix) of random, near-parallel pairs and pairs
//      with (0, 0, 1); rotation_xyz_diff, rotation_diff_z; generate_transform. The same line is
//      tagged `va` when the pair is nearly antiparallel (Quaternion::setFromTwoVectors' branch
//      of a 2x3 JacobiSVD); lines `vx`: rotation_from_two_vectors of exactly antiparallel pairs.
//   q  Eigen operations: Quaterniond(AngleAxisd), quaternion product, toRotationMatrix,
//      Quaterniond(Matrix3d) and normalized, AngleAxisd(Matrix3d), AngleAxisd::toRotationMatrix,
//      eulerAngles(2, 1, 0), squaredNorm, norm, normalized, dot, cross, Matrix3d * Matrix3d,
//      Matrix3d * Vec3d, determinant

#include "libslic3r/Geometry.hpp"
#include "libslic3r/BoundingBox.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

using namespace Slic3r;

#pragma clang fp contract(off)

static bool g_full_all = false;
static const int FULL = 4;

static std::string hD(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, 8);
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long) u);
    return b;
}
static std::string hB(bool v) { return v ? "1" : "0"; }

struct Out
{
    std::string s;
    void d(double v) { s += (s.empty() ? "" : " ") + hD(v); }
    void b(bool v) { s += (s.empty() ? "" : " ") + hB(v); }
    void v3(const Vec3d &v) { d(v.x()); d(v.y()); d(v.z()); }
    void m3(const Matrix3d &m)
    {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) d(m(r, c));
    }
    void m4(const Transform3d &t)
    {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c) d(t.matrix()(r, c));
    }
    void q(const Eigen::Quaterniond &q) { d(q.x()); d(q.y()); d(q.z()); d(q.w()); }
};

static void emit(const std::string &tag, int k, const Out &p)
{
    if (g_full_all || k < FULL) {
        std::printf("%s %d %s\n", tag.c_str(), k, p.s.c_str());
    } else {
        uint64_t h = 1469598103934665603ULL;
        for (unsigned char c : p.s) {
            h ^= c;
            h *= 1099511628211ULL;
        }
        std::printf("%s %d #%016llx\n", tag.c_str(), k, (unsigned long long) h);
    }
}

// splitmix64
struct Rng
{
    uint64_t s;
    explicit Rng(uint64_t seed) : s(seed) {}
    uint64_t next()
    {
        s += 0x9e3779b97f4a7c15ULL;
        uint64_t z = s;
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        return z ^ (z >> 31);
    }
    double uni() { return double(next() >> 11) * (1.0 / 9007199254740992.0); }
    double range(double a, double b) { return a + (b - a) * uni(); }
    int below(int n) { return int(next() % uint64_t(n)); }
};

static const double PI_ = 3.14159265358979323846;

// Rotation matrix of the unit quaternion of four uniform numbers.
static Matrix3d rand_rot(Rng &r)
{
    double w = r.range(-1, 1), x = r.range(-1, 1), y = r.range(-1, 1), z = r.range(-1, 1);
    double n = std::sqrt(w * w + x * x + y * y + z * z);
    w /= n; x /= n; y /= n; z /= n;
    Matrix3d m;
    m(0, 0) = 1 - 2 * (y * y + z * z); m(0, 1) = 2 * (x * y - z * w);     m(0, 2) = 2 * (x * z + y * w);
    m(1, 0) = 2 * (x * y + z * w);     m(1, 1) = 1 - 2 * (x * x + z * z); m(1, 2) = 2 * (y * z - x * w);
    m(2, 0) = 2 * (x * z - y * w);     m(2, 1) = 2 * (y * z + x * w);     m(2, 2) = 1 - 2 * (x * x + y * y);
    return m;
}

// Rz(c) * Ry(b) * Rx(a), plain products.
static Matrix3d euler_rot(double a, double b, double c)
{
    double ca = std::cos(a), sa = std::sin(a), cb = std::cos(b), sb = std::sin(b), cc = std::cos(c), sc = std::sin(c);
    Matrix3d rx, ry, rz;
    rx << 1, 0, 0, 0, ca, -sa, 0, sa, ca;
    ry << cb, 0, sb, 0, 1, 0, -sb, 0, cb;
    rz << cc, -sc, 0, sc, cc, 0, 0, 0, 1;
    Matrix3d t, m;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) t(i, j) = rz(i, 0) * ry(0, j) + rz(i, 1) * ry(1, j) + rz(i, 2) * ry(2, j);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m(i, j) = t(i, 0) * rx(0, j) + t(i, 1) * rx(1, j) + t(i, 2) * rx(2, j);
    return m;
}

// An Euler angle: random, a multiple of 90 degrees, or a multiple of 90 degrees and a little.
static double rand_angle(Rng &r)
{
    int kind = r.below(4);
    if (kind == 0) return double(r.below(9) - 4) * (0.5 * PI_);
    if (kind == 1) {
        double k = double(r.below(9) - 4);
        double e = r.range(-1e-9, 1e-9);
        return k * (0.5 * PI_) + e;
    }
    return r.range(-PI_, PI_);
}

static Transform3d affine(const Matrix3d &lin, const Vec3d &t)
{
    Transform3d m = Transform3d::Identity();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) m.matrix()(i, j) = lin(i, j);
        m.matrix()(i, 3) = t(i);
    }
    return m;
}

static Matrix3d times_diag(const Matrix3d &m, const Vec3d &s)
{
    Matrix3d o;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) o(i, j) = m(i, j) * s(j);
    return o;
}

// Function arguments are not sequenced in C++: every draw is a statement of its own.
static Vec3d rand_vec(Rng &r, double a, double b)
{
    double x = r.range(a, b);
    double y = r.range(a, b);
    double z = r.range(a, b);
    return Vec3d(x, y, z);
}

static const int NFAM = 9;
static const char *FAM_NAMES[NFAM] = {"aff", "rot", "rte", "asm", "sca", "sing", "gen", "int", "tiny"};
static const int FAM_COUNT[NFAM] = {600, 250, 250, 400, 150, 200, 250, 150, 100};

static Transform3d make_case(int fam, Rng &r)
{
    switch (fam) {
    case 0: { // random affine: rotation * scale, a mirror on one axis 3 times in 4
        Matrix3d rot = rand_rot(r);
        Vec3d s = rand_vec(r, 0.5, 2.0);
        int mi = r.below(4);
        if (mi < 3) s(mi) = -s(mi);
        Matrix3d m = times_diag(rot, s);
        return affine(m, rand_vec(r, -50, 50));
    }
    case 1: { // pure rotation
        Matrix3d m = rand_rot(r);
        return affine(m, rand_vec(r, -50, 50));
    }
    case 2: { // rotation from Euler angles, with multiples of 90 degrees
        double a = rand_angle(r), b = rand_angle(r), c = rand_angle(r);
        Matrix3d m = euler_rot(a, b, c);
        return affine(m, rand_vec(r, -50, 50));
    }
    case 3: { // assembled: translation * rotation * scale * mirror
        double a = rand_angle(r), b = rand_angle(r), c = rand_angle(r);
        Vec3d s = rand_vec(r, 0.1, 10.0);
        for (int i = 0; i < 3; ++i)
            if (r.below(4) == 0) s(i) = -s(i);
        if (r.below(3) == 0) s(1) = s(2) = s(0);
        Matrix3d m = times_diag(euler_rot(a, b, c), s);
        return affine(m, rand_vec(r, -100, 100));
    }
    case 4: { // pure scale
        Vec3d s = rand_vec(r, 0.01, 100.0);
        int kind = r.below(3);
        if (kind == 0) s(1) = s(2) = s(0);
        if (kind == 1) s(r.below(3)) *= -1.0;
        Matrix3d m = times_diag(Matrix3d::Identity(), s);
        return affine(m, rand_vec(r, -50, 50));
    }
    case 5: { // near-singular
        Matrix3d rot = rand_rot(r);
        Vec3d s = rand_vec(r, 0.5, 2.0);
        int k = r.below(5);
        double tiny = k == 0 ? 1e-3 : k == 1 ? 1e-6 : k == 2 ? 1e-9 : k == 3 ? 1e-12 : 0.0;
        int axis = r.below(3);
        s(axis) = tiny * s(axis);
        if (r.below(4) == 0) s((axis + 1) % 3) = tiny * 0.5;
        Matrix3d m = times_diag(rot, s);
        return affine(m, rand_vec(r, -50, 50));
    }
    case 6: { // general, skewed
        Matrix3d m;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) m(i, j) = r.range(-2.0, 2.0);
        return affine(m, rand_vec(r, -50, 50));
    }
    case 7: { // signed permutation times a scale of 1, 2, 0.5 or 3 per axis
        static const int perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
        static const double scales[4] = {1.0, 2.0, 0.5, 3.0};
        int p = r.below(6);
        Matrix3d m = Matrix3d::Zero();
        for (int i = 0; i < 3; ++i) m(i, perms[p][i]) = r.below(2) ? -1.0 : 1.0;
        double s0 = scales[r.below(4)];
        double s1 = scales[r.below(4)];
        double s2 = scales[r.below(4)];
        double t0 = double(r.below(21) - 10);
        double t1 = double(r.below(21) - 10);
        return affine(times_diag(m, Vec3d(s0, s1, s2)), Vec3d(t0, t1, 0.0));
    }
    default: { // rotation by a tiny angle about a random axis
        Vec3d ax = rand_vec(r, -1, 1);
        double n = std::sqrt(ax.x() * ax.x() + ax.y() * ax.y() + ax.z() * ax.z());
        ax = Vec3d(ax.x() / n, ax.y() / n, ax.z() / n);
        double ang = std::pow(10.0, r.range(-12.0, -2.0));
        double c = std::cos(ang), s = std::sin(ang), t = 1 - c;
        Matrix3d m;
        m(0, 0) = t * ax.x() * ax.x() + c;          m(0, 1) = t * ax.x() * ax.y() - s * ax.z(); m(0, 2) = t * ax.x() * ax.z() + s * ax.y();
        m(1, 0) = t * ax.x() * ax.y() + s * ax.z(); m(1, 1) = t * ax.y() * ax.y() + c;          m(1, 2) = t * ax.y() * ax.z() - s * ax.x();
        m(2, 0) = t * ax.x() * ax.z() - s * ax.y(); m(2, 1) = t * ax.y() * ax.z() + s * ax.x(); m(2, 2) = t * ax.z() * ax.z() + c;
        Vec3d sc = Vec3d::Ones();
        if (r.below(2)) sc = Vec3d(1.5, 1.5, 1.5);
        Matrix3d ms = times_diag(m, sc);
        return affine(ms, rand_vec(r, -50, 50));
    }
    }
}

// A mirror request: -1, 1, 0 or another value.
static double rand_mirror(Rng &r)
{
    int k = r.below(5);
    return k == 0 ? -1.0 : k == 1 ? 1.0 : k == 2 ? 0.0 : k == 3 ? -2.5 : 0.5;
}

static void line_a(const std::string &fam, int k, const Transform3d &T)
{
    Out p;
    Eigen::JacobiSVD<Matrix3d> svd(T.linear(), Eigen::ComputeFullU | Eigen::ComputeFullV);
    p.m3(svd.matrixU());
    p.m3(svd.matrixV());
    p.v3(svd.singularValues());
    Matrix3d R, S;
    T.computeRotationScaling(&R, &S);
    p.m3(R);
    p.m3(S);
    Geometry::Transformation tr(T);
    p.v3(tr.get_rotation());
    p.v3(tr.get_scaling_factor());
    p.v3(tr.get_mirror());
    p.v3(tr.get_rotation_by_quaternion());
    p.b(tr.has_skew());
    p.b(tr.is_left_handed());
    p.b(tr.is_scaling_uniform());
    p.v3(Geometry::extract_euler_angles(T));
    p.v3(Geometry::extract_euler_angles(Eigen::Matrix<double, 3, 3, Eigen::DontAlign>(R)));
    Geometry::TransformationSVD ts(T);
    p.m3(ts.u);
    p.m3(ts.s);
    p.m3(ts.v);
    p.b(ts.mirror);
    p.b(ts.scale);
    p.b(ts.anisotropic_scale);
    p.b(ts.rotation);
    p.b(ts.rotation_90_degrees);
    p.b(ts.skew);
    p.m4(tr.get_rotation_matrix());
    p.m4(tr.get_scaling_factor_matrix());
    p.m4(tr.get_mirror_matrix());
    emit("a." + fam, k, p);
}

static void line_b(const std::string &fam, int k, const Transform3d &T, Rng &r)
{
    Out p;
    Geometry::Transformation tr(T);
    double r0 = rand_angle(r);
    double r1 = rand_angle(r);
    double r2 = rand_angle(r);
    Vec3d rot(r0, r1, r2);
    Vec3d sc = rand_vec(r, 0.2, 3.0);
    int axis = r.below(3);
    double sa = r.range(0.2, 3.0);
    double m0 = rand_mirror(r);
    double m1 = rand_mirror(r);
    double m2 = rand_mirror(r);
    Vec3d mir(m0, m1, m2);
    double ma = rand_mirror(r);
    Vec3d tv = rand_vec(r, -100, 100);
    double su = r.range(0.2, 3.0);
    {
        Geometry::Transformation c(tr);
        c.set_rotation(rot);
        p.m4(c.get_matrix());
    }
    {
        Geometry::Transformation c(tr);
        c.set_scaling_factor(sc);
        p.m4(c.get_matrix());
    }
    {
        Geometry::Transformation c(tr);
        c.set_scaling_factor(Axis(axis), sa);
        p.m4(c.get_matrix());
    }
    {
        Geometry::Transformation c(tr);
        c.set_mirror(mir);
        p.m4(c.get_matrix());
    }
    {
        Geometry::Transformation c(tr);
        c.set_mirror(Axis(axis), ma);
        p.m4(c.get_matrix());
    }
    {
        Geometry::Transformation c(tr);
        c.set_rotation_matrix(Geometry::rotation_transform(rot));
        p.m4(c.get_matrix());
    }
    p.m4(Geometry::rotation_transform(rot));
    Vec3d m3(mir.x() < 0 ? -1.0 : 1.0, mir.y() < 0 ? -1.0 : 1.0, mir.z() < 0 ? -1.0 : 1.0);
    p.m4(Geometry::assemble_transform(tv, rot, sc, m3));
    p.m4(Geometry::translation_transform(tv));
    p.m4(Geometry::scale_transform(sc));
    p.m4(Geometry::scale_transform(su));
    emit("b." + fam, k, p);
}

static void line_c(const std::string &fam, int k, const Transform3d &T)
{
    Out p;
    Geometry::Transformation tr(T);
    {
        Geometry::Transformation c(tr);
        c.reset_rotation();
        p.m4(c.get_matrix());
    }
    {
        Geometry::Transformation c(tr);
        c.reset_scaling_factor();
        p.m4(c.get_matrix());
    }
    {
        Geometry::Transformation c(tr);
        c.reset_skew();
        p.m4(c.get_matrix());
    }
    {
        Geometry::Transformation c(tr);
        c.reset_mirror();
        p.m4(c.get_matrix());
    }
    for (int f = 1; f < 16; ++f) p.m4(tr.get_matrix((f & 1) != 0, (f & 2) != 0, (f & 4) != 0, (f & 8) != 0));
    p.m4(tr.get_matrix_no_offset());
    p.m4(tr.get_matrix_no_scaling_factor());
    emit("c." + fam, k, p);
}

static void line_d(const std::string &fam, int k, const Transform3d &T, Rng &r)
{
    Out p;
    Geometry::Transformation tr(T);
    Transform3d T2 = make_case(0, r);
    Vec3d bmin = rand_vec(r, -20, 0);
    Vec3d bmax = rand_vec(r, 0, 20);
    BoundingBoxf3 bb(bmin, bmax);
    Vec3d v = rand_vec(r, -30, 30);
    Vec3d pt = rand_vec(r, -30, 30);
    Vec3d ax = rand_vec(r, -1, 1);
    float theta = float(r.range(-PI_, PI_));
    p.m4((tr * Geometry::Transformation(T2)).get_matrix());
    // volume_to_bed_transformation asserts positive scaling factors (set_scaling_factor): a matrix
    // whose decomposition has a zero or NaN scale or mirror (signed permutations with two equal
    // scales and a mirror) takes the identity instead.
    const Vec3d sf = tr.get_scaling_factor(), mf = tr.get_mirror();
    bool v2b_ok = true;
    for (int i = 0; i < 3; ++i)
        if (!(sf(i) > 0.0) || !std::isfinite(sf(i)) || !std::isfinite(mf(i))) v2b_ok = false;
    p.m4(Geometry::Transformation::volume_to_bed_transformation(v2b_ok ? tr : Geometry::Transformation(), bb).get_matrix());
    p.m4(Geometry::rotation_transform(tr.get_rotation()));
    BoundingBoxf3 tb = bb.transformed(T);
    p.b(tb.defined);
    p.v3(tb.min);
    p.v3(tb.max);
    p.v3(T * v);
    p.m4(T.inverse());
    p.d(T.linear().determinant());
    p.m4(Geometry::mat_around_a_point_rotate(tr, pt, ax, theta).get_matrix());
    emit("d." + fam, k, p);
}

static void lines_v(Rng &r)
{
    const int N = 400;
    for (int k = 0; k < N; ++k) {
        Out p;
        Vec3d from = rand_vec(r, -2, 2), to;
        int kind = r.below(6);
        if (kind == 0 || kind == 1) {
            double f = r.range(0.5, 2.0);
            if (kind == 1) f = -f;
            Vec3d e = rand_vec(r, -1e-7, 1e-7);
            to = Vec3d(from.x() * f + e.x(), from.y() * f + e.y(), from.z() * f + e.z());
        }
        else if (kind == 2) to = Vec3d(0, 0, 1);
        else if (kind == 3) {
            double fx = r.range(-1e-3, 1e-3);
            double fy = r.range(-1e-3, 1e-3);
            double fz = r.below(2) ? 1.0 : -1.0;
            from = Vec3d(fx, fy, fz);
            to = Vec3d(0, 0, 1);
        }
        else to = rand_vec(r, -2, 2);
        Vec3d axis;
        double phi;
        Matrix3d m;
        Geometry::rotation_from_two_vectors(from, to, axis, phi, &m);
        p.v3(axis);
        p.d(phi);
        p.m3(m);
        double f0 = rand_angle(r);
        double f1 = rand_angle(r);
        double f2 = rand_angle(r);
        double t0 = rand_angle(r);
        double t1 = rand_angle(r);
        double t2 = rand_angle(r);
        Vec3d r_from(f0, f1, f2);
        Vec3d r_to(t0, t1, t2);
        if (r.below(2)) {
            double z = r.range(-PI_, PI_);
            r_to = Vec3d(r_from.x(), r_from.y(), z);
        }
        p.q(Geometry::rotation_xyz_diff(r_from, r_to));
        p.d(Geometry::rotation_diff_z(r_from, r_to));
        Vec3d o = rand_vec(r, -50, 50);
        Matrix3d b = rand_rot(r);
        Vec3d sx = rand_vec(r, 0.5, 2);
        Vec3d gx(b(0, 0) * sx.x(), b(1, 0) * sx.x(), b(2, 0) * sx.x());
        Vec3d gy(b(0, 1) * sx.y(), b(1, 1) * sx.y(), b(2, 1) * sx.y());
        Vec3d gz(b(0, 2) * sx.z(), b(1, 2) * sx.z(), b(2, 2) * sx.z());
        p.m4(Geometry::generate_transform(gx, gy, gz, o).get_matrix());
        emit(kind == 1 ? "va" : "v", k, p);
    }
    // exactly antiparallel pairs (Orient.cpp turns a face normal (0, 0, -1) to (0, 0, 1))
    const double exact[][6] = {{0, 0, -1, 0, 0, 1}, {1, 0, 0, -2, 0, 0}, {0.6, 0, -0.8, -0.6, 0, 0.8}, {0, 1, 0, 0, -1, 0}, {0, 0, 1, 0, 0, -1}};
    for (int k = 0; k < 5; ++k) {
        Out p;
        Vec3d axis;
        double phi;
        Matrix3d m;
        Geometry::rotation_from_two_vectors(Vec3d(exact[k][0], exact[k][1], exact[k][2]), Vec3d(exact[k][3], exact[k][4], exact[k][5]), axis, phi, &m);
        p.v3(axis);
        p.d(phi);
        p.m3(m);
        emit("vx", k, p);
    }
}

static void lines_q(Rng &r)
{
    const int N = 600;
    for (int k = 0; k < N; ++k) {
        Out p;
        double a1 = rand_angle(r), a2 = r.range(-10.0, 10.0);
        Vec3d ax1 = rand_vec(r, -1, 1);
        Vec3d ax2 = rand_vec(r, -1, 1);
        double n1 = std::sqrt(ax1.x() * ax1.x() + ax1.y() * ax1.y() + ax1.z() * ax1.z());
        double n2 = std::sqrt(ax2.x() * ax2.x() + ax2.y() * ax2.y() + ax2.z() * ax2.z());
        ax1 = Vec3d(ax1.x() / n1, ax1.y() / n1, ax1.z() / n1);
        ax2 = Vec3d(ax2.x() / n2, ax2.y() / n2, ax2.z() / n2);
        if (k % 7 == 0) ax1 = Vec3d::UnitZ();
        if (k % 11 == 0) ax2 = Vec3d::UnitX();
        Eigen::Quaterniond q1(Eigen::AngleAxisd(a1, ax1));
        Eigen::Quaterniond q2(Eigen::AngleAxisd(a2, ax2));
        p.q(q1);
        p.q(q2);
        Eigen::Quaterniond q12 = q1 * q2;
        p.q(q12);
        Matrix3d m12 = q12.toRotationMatrix();
        p.m3(m12);
        Matrix3d rr = (k % 5 == 0) ? Matrix3d(times_diag(rand_rot(r), Vec3d(1.0, 1.0, 1.0))) : m12;
        Eigen::Quaterniond qm(rr);
        p.q(qm);
        Eigen::Quaterniond qn = qm;
        qn.normalize();
        p.q(qn);
        Eigen::AngleAxisd aa(rr);
        p.v3(aa.axis());
        p.d(aa.angle());
        p.m3(Eigen::AngleAxisd(a2, ax2).toRotationMatrix());
        p.v3(rr.eulerAngles(2, 1, 0));
        Vec3d v = rand_vec(r, -10, 10);
        Vec3d w = rand_vec(r, -10, 10);
        p.d(v.squaredNorm());
        p.d(v.norm());
        p.v3(v.normalized());
        p.d(v.dot(w));
        p.v3(v.cross(w));
        Matrix3d A, B;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                A(i, j) = r.range(-3, 3);
                B(i, j) = r.range(-3, 3);
            }
        Matrix3d AB = A * B;
        p.m3(AB);
        Vec3d Av = A * v;
        p.v3(Av);
        p.d(A.determinant());
        Matrix3d ABt = A * B.transpose();
        p.m3(ABt);
        emit("q", k, p);
    }
}

int main()
{
    g_full_all = std::getenv("FULL") != nullptr;
    Rng r(0x5eed5eedULL);
    for (int fam = 0; fam < NFAM; ++fam) {
        for (int i = 0; i < FAM_COUNT[fam]; ++i) {
            Transform3d T = make_case(fam, r);
            const std::string name = FAM_NAMES[fam];
            Out m;
            m.m4(T);
            emit("m." + name, i, m);
            line_a(name, i, T);
            line_b(name, i, T, r);
            line_c(name, i, T);
            line_d(name, i, T, r);
        }
    }
    lines_v(r);
    lines_q(r);
    return 0;
}
