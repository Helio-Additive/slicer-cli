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
