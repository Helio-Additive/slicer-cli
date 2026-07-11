// Implementation: call the REAL Eigen with the EXACT sequence libslic3r uses
// (TriangleMeshSlicer.cpp:1827-1862). Bit-exact to C++ by construction (same
// Eigen headers, same -O3 arm64 codegen forced in build.rs).
#include "eigen_transform_shim.h"

#include <cstdio>
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

// ---------------------------------------------------------------------------
// R187: exact-native min_vertex_dot_product kernel (ShortEdgeCollapse.cpp:43-55)
// Copied verbatim from the reference tree so the Eigen f32 codegen matches the
// native binary (same compiler/flags): TriangleMesh.hpp:330-334 face normals
// (note the DOUBLE normalize) + NormalUtils.cpp:5-15/96-126 nelson-weighted
// vertex normals + the min-dot loop.
// ---------------------------------------------------------------------------
#include <vector>
#include <Eigen/Geometry>

extern "C" void secol_min_vertex_dots(const float *verts, int64_t n_verts,
                                      const int32_t *indices, int64_t n_tris,
                                      float *out)
{
    using Vec3f = Eigen::Matrix<float, 3, 1>;
    using Vec3i = Eigen::Matrix<int, 3, 1>;

    std::vector<Vec3f> vertices(n_verts);
    for (int64_t i = 0; i < n_verts; ++i)
        vertices[i] = Vec3f(verts[3 * i], verts[3 * i + 1], verts[3 * i + 2]);
    std::vector<Vec3i> faces(n_tris);
    for (int64_t i = 0; i < n_tris; ++i)
        faces[i] = Vec3i(indices[3 * i], indices[3 * i + 1], indices[3 * i + 2]);

    // its_face_normals (TriangleMesh.hpp:331-334): face_normal = normalized
    // cross, then face_normal_normalized normalizes AGAIN (native double
    // normalize, copied exactly).
    std::vector<Vec3f> face_normals;
    face_normals.reserve(n_tris);
    for (const Vec3i &face : faces) {
        const Vec3f v[3]{vertices[face[0]], vertices[face[1]], vertices[face[2]]};
        Vec3f fn = (v[1] - v[0]).cross(v[2] - v[1]).normalized();
        face_normals.push_back(fn.normalized());
    }

    // NormalUtils::create_normals_nelson_weighted (NormalUtils.cpp:96-126).
    std::vector<Vec3f> normals(n_verts, Vec3f(0.f, 0.f, 0.f));
    std::vector<float> count(n_verts, 0.f);
    for (const Vec3i &indice : faces) {
        // create_triangle_normal (NormalUtils.cpp:5-15)
        const Vec3f &v0 = vertices[indice[0]];
        const Vec3f &v1 = vertices[indice[1]];
        const Vec3f &v2 = vertices[indice[2]];
        Vec3f normal = (v1 - v0).cross(v2 - v0);
        normal.normalize();

        float e0 = (v0 - v1).norm();
        float e1 = (v1 - v2).norm();
        float e2 = (v2 - v0).norm();

        Vec3f coefs(e0 * e2, e0 * e1, e1 * e2);
        for (int i = 0; i < 3; ++i) {
            const float &weight = coefs[i];
            normals[indice[i]] += normal * weight;
            count[indice[i]] += weight;
        }
    }
    for (int64_t i = 0; i < n_verts; ++i)
        normals[i] /= count[i];

    // ShortEdgeCollapse.cpp:43-55 — the min-dot loop.
    for (int64_t i = 0; i < n_verts; ++i)
        out[i] = 1.0f;
    for (int64_t face_idx = 0; face_idx < n_tris; ++face_idx) {
        const Vec3i &t = faces[face_idx];
        const Vec3f &n = face_normals[face_idx];
        out[t[0]] = std::min(out[t[0]], n.dot(normals[t[0]]));
        out[t[1]] = std::min(out[t[1]], n.dot(normals[t[1]]));
        out[t[2]] = std::min(out[t[2]], n.dot(normals[t[2]]));
    }
}
