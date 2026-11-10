// Implementation: call the REAL Eigen with the EXACT sequence libslic3r uses
// (TriangleMeshSlicer.cpp:1827-1862). Bit-exact to C++ by construction (same
// Eigen headers, same -O3 arm64 codegen forced in build.rs).
#include "eigen_transform_shim.h"

#include <cstdio>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <Eigen/Geometry>

using Transform3d = Eigen::Transform<double, 3, Eigen::Affine>;
using Transform3f = Eigen::Transform<float, 3, Eigen::Affine>;
using Vec3d = Eigen::Matrix<double, 3, 1>;
using stl_vertex = Eigen::Matrix<float, 3, 1>;

// R87 frame-unification: apply C++'s EXACT slice transform to verts that are in
// rust's PLACED frame (rust_raw). C++ slices `prescale · params2.trafo · volume`,
// where volume = rust_raw − volume_offset (rust's placed mesh is the C++ volume
// shifted by volume_offset = volume.get_matrix translation). So:
//   slice_vert = prescale · params2.trafo · (rust_raw − volume_offset)
// `trafo16` = the f64 params2.trafo (row-major 4x4, 16 doubles). `voff_{x,y,z}` =
// the volume offset (subtracted from each input vert before the trafo, in f64 to
// match C++'s f32 centered-volume store — see caller). All f32 matmul via real
// Eigen (bit-exact). Z handled by the trafo (the +24 round-trip = C++'s R65 floor
// mechanism), so the caller must NOT pre-bake quantize_f32_center_roundtrip.
extern "C" void eigen_transform_verts_unified(const double *trafo16, double scaling_factor,
                                              double voff_x, double voff_y, double voff_z,
                                              const float *verts_in, float *verts_out,
                                              int32_t n) {
    Transform3d trafo;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            trafo.matrix()(r, c) = trafo16[r * 4 + c];

    const double s = 1.0 / scaling_factor;
    Transform3d t = trafo;
    t.prescale(Vec3d(s, s, 1.0));
    Transform3f tf = t.cast<float>();

    // volume offset as f32 (C++ stores the centered volume in f32).
    const float ox = (float) voff_x, oy = (float) voff_y, oz = (float) voff_z;
    for (int32_t i = 0; i < n; ++i) {
        stl_vertex v(verts_in[3 * i] - ox, verts_in[3 * i + 1] - oy, verts_in[3 * i + 2] - oz);
        v = tf * v;
        verts_out[3 * i] = v.x();
        verts_out[3 * i + 1] = v.y();
        verts_out[3 * i + 2] = v.z();
    }
}

// R786 — plain f32 affine apply: `out = Transform3f(trafo16) * v` per vertex,
// NO prescale, NO voff. Replicates MultiMaterialSegmentation.cpp:2303's
// `facet[p] = tr * vertices[idx]` (tr = (trafo() * get_matrix()).cast<float>())
// with the exact Eigen f32 codegen (rotation matmul = the R85 1-ULP wall).
extern "C" void eigen_transform_verts_affine_f32(const double *trafo16,
                                                 const float *verts_in, float *verts_out,
                                                 int32_t n) {
    Transform3d trafo;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            trafo.matrix()(r, c) = trafo16[r * 4 + c];
    Transform3f tf = trafo.cast<float>();
    for (int32_t i = 0; i < n; ++i) {
        stl_vertex v(verts_in[3 * i], verts_in[3 * i + 1], verts_in[3 * i + 2]);
        v = tf * v;
        verts_out[3 * i] = v.x();
        verts_out[3 * i + 1] = v.y();
        verts_out[3 * i + 2] = v.z();
    }
}

// R787 — native MMS.cpp:2303 composes the paint transform IN F32:
//   Transform3f tr = trafo().cast<float>() * get_matrix().cast<float>();
// (f32 x f32 matrix product, NOT an f64 compose then cast). a16 = trafo(),
// b16 = volume matrix, both f64 row-major 4x4; per vertex out = tr * v.
extern "C" void eigen_transform_verts_affine_f32_pair(const double *a16, const double *b16,
                                                      const float *verts_in, float *verts_out,
                                                      int32_t n) {
    Transform3d a, b;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) {
            a.matrix()(r, c) = a16[r * 4 + c];
            b.matrix()(r, c) = b16[r * 4 + c];
        }
    Transform3f tr = a.cast<float>() * b.cast<float>();
    for (int32_t i = 0; i < n; ++i) {
        stl_vertex v(verts_in[3 * i], verts_in[3 * i + 1], verts_in[3 * i + 2]);
        v = tr * v;
        verts_out[3 * i] = v.x();
        verts_out[3 * i + 1] = v.y();
        verts_out[3 * i + 2] = v.z();
    }
}

extern "C" void eigen_transform_verts_for_slicing(double scaling_factor, double cx,
                                                  double cy, const float *verts_in,
                                                  float *verts_out, int32_t n) {
    // trafo_centered = trafo().pretranslate(Vec3d(-cx, -cy, 0)) — trafo() is the
    // PrintObject m_trafo, which for slicer_cli STL (add_instance @ offset 0, no
    // rotation/scale) is Identity (Print.hpp:375-376, PrintApply.cpp:151-152 reset
    // XY then the trafo is stored without translation). So here:
    Transform3d trafo = Transform3d::Identity();
    trafo.pretranslate(Vec3d(-cx, -cy, 0.0));

    // make_trafo_for_slicing (TriangleMeshSlicer.cpp:1827-1833):
    //   auto t = trafo; t.prescale(Vec3d(s, s, 1.)); return t.cast<float>();
    const double s = 1.0 / scaling_factor;
    Transform3d t = trafo;
    t.prescale(Vec3d(s, s, 1.0));
    Transform3f tf = t.cast<float>();


    // transform_mesh_vertices_for_slicing non-identity path (line 1858-1859):
    //   for (stl_vertex &v : out) v = tf * v;
    // Use a CONCRETE stl_vertex (not a Map) + the in-place `v = tf * v` form,
    // matching C++'s codegen exactly (Map<> can change the product code path).
    for (int32_t i = 0; i < n; ++i) {
        stl_vertex v(verts_in[3 * i], verts_in[3 * i + 1], verts_in[3 * i + 2]);
        v = tf * v;
        verts_out[3 * i] = v.x();
        verts_out[3 * i + 1] = v.y();
        verts_out[3 * i + 2] = v.z();
    }
}

// R806 — MultiMaterialSegmentation.cpp:493-513 `project_line_on_line`, run through
// the REAL Eigen with the native expression order (so the compiler's FMA
// contraction of the 2D dot products matches the libslic3r build). Points are
// integer coord_t; the projection endpoints are `a + (t * v1).cast<coord_t>()`
// (truncation toward zero). Returns 0 when the projection line is degenerate.
extern "C" int32_t eigen_project_line_on_line(int64_t pax, int64_t pay, int64_t pbx, int64_t pby,
                                             int64_t qax, int64_t qay, int64_t qbx, int64_t qby,
                                             int64_t *out4) {
    using Vec2d = Eigen::Matrix<double, 2, 1>;
    using Vec2i = Eigen::Matrix<int64_t, 2, 1>;
    const Vec2i pa(pax, pay), pb(pbx, pby), qa(qax, qay), qb(qbx, qby);
    const Vec2d  v1 = (pb - pa).cast<double>();
    const Vec2d  va = (qa - pa).cast<double>();
    const Vec2d  vb = (qb - pa).cast<double>();
    const double l2 = v1.squaredNorm();
    if (l2 == 0.0)
        return 0;
    double t1 = va.dot(v1) / l2;
    double t2 = vb.dot(v1) / l2;
    t1 = std::clamp(t1, 0., 1.);
    t2 = std::clamp(t2, 0., 1.);
    const Vec2i p1 = pa + (t1 * v1).cast<int64_t>();
    const Vec2i p2 = pa + (t2 * v1).cast<int64_t>();
    out4[0] = p1.x(); out4[1] = p1.y(); out4[2] = p2.x(); out4[3] = p2.y();
    return 1;
}

// R806 — MultiPoint.cpp:179-230 `_douglas_peucker` with Line.hpp:40-69
// `line_alg::distance_to_squared`, run through the REAL Eigen with the native
// expression order so the compiler's FMA contraction matches libslic3r. Marks
// keep[i]=1 for every retained point of the open polyline xy[0..n).
extern "C" void eigen_douglas_peucker(const int64_t *xy, int32_t n, double tolerance, uint8_t *keep) {
    using Vec2d = Eigen::Matrix<double, 2, 1>;
    using Vec2i = Eigen::Matrix<int64_t, 2, 1>;
    for (int32_t i = 0; i < n; ++i) keep[i] = 0;
    if (n <= 0) return;
    auto P = [&](int32_t i) { return Vec2i(xy[2 * i], xy[2 * i + 1]); };
    auto dist2 = [&](int32_t i, int32_t a, int32_t b) -> double {
        const Vec2i pa = P(a), pb = P(b), pt = P(i);
        const Vec2d  v  = (pb - pa).cast<double>();
        const Vec2d  va = (pt - pa).cast<double>();
        const double l2 = v.squaredNorm();
        if (l2 == 0.0) return va.squaredNorm();
        const double t = va.dot(v) / l2;
        if (t <= 0.0) return va.squaredNorm();
        else if (t >= 1.0) return (pt - pb).cast<double>().squaredNorm();
        return (t * v - va).squaredNorm();
    };
    const double tolerance_sq = tolerance * tolerance;
    int32_t anchor_idx = 0, floater_idx = n - 1;
    keep[0] = 1;
    if (anchor_idx != floater_idx) {
        std::vector<int32_t> dpStack; dpStack.reserve(n); dpStack.push_back(floater_idx);
        for (;;) {
            double max_dist_sq = 0.0; int32_t furthest_idx = anchor_idx;
            for (int32_t i = anchor_idx + 1; i < floater_idx; ++i) {
                double d = dist2(i, anchor_idx, floater_idx);
                if (d > max_dist_sq) { max_dist_sq = d; furthest_idx = i; }
            }
            if (max_dist_sq <= tolerance_sq) {
                keep[floater_idx] = 1;
                anchor_idx = floater_idx;
                dpStack.pop_back();
                if (dpStack.empty()) break;
                floater_idx = dpStack.back();
            } else {
                floater_idx = furthest_idx;
                dpStack.push_back(floater_idx);
            }
        }
    }
}
