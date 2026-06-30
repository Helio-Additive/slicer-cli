// C ABI shim around Eigen, exposing the EXACT vertex transform libslic3r uses for
// slicing (make_trafo_for_slicing + transform_mesh_vertices_for_slicing's
// non-identity path, TriangleMeshSlicer.cpp:1827-1862). By calling the real Eigen
// (the same header-only lib + the same -O3 arm64 codegen as the C++ build) the
// f32 matmul is BIT-EXACT to C++ by construction — sidestepping the pure-rust
// Eigen-reproduction wall (R85: 74180/112569 verts off by 1 ULP).
#ifndef EIGEN_TRANSFORM_SHIM_H
#define EIGEN_TRANSFORM_SHIM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Transform `n` input vertices (verts_in: 3*n floats, x,y,z interleaved) into the
// slice-time frame and write to verts_out (3*n floats). Replicates EXACTLY:
//   Transform3d t = trafo_centered;            // trafo (Identity for slicer_cli
//                                              //   STL) then pretranslate(-cx,-cy,0)
//   t.prescale(Vec3d(s, s, 1.)); s = 1/scaling_factor
//   Transform3f tf = t.cast<float>();
//   for each v: out = tf * v;                  // Eigen Affine3f * Vector3f
//
// `scaling_factor` = libslic3r SCALING_FACTOR (1e-5). `cx`/`cy` = the unscaled
// (mm) center_offset = unscale(m_center_offset). For the identity (no centering)
// case pass cx=cy=0 (then this equals v.x*=s, v.y*=s with Z unchanged — but the
// caller uses the pure-rust path for that; this shim is for the centered case).
void eigen_transform_verts_for_slicing(double scaling_factor, double cx, double cy,
                                       const float *verts_in, float *verts_out,
                                       int32_t n);

// R87 frame-unification: slice_vert = prescale·params2.trafo·(rust_raw − voff).
// `trafo16` = f64 params2.trafo (row-major 4x4). `voff_*` = volume.get_matrix
// translation (rust placed frame = C++ volume + voff). Bit-exact via real Eigen.
void eigen_transform_verts_unified(const double *trafo16, double scaling_factor,
                                   double voff_x, double voff_y, double voff_z,
                                   const float *verts_in, float *verts_out, int32_t n);

#ifdef __cplusplus
}
#endif

#endif // EIGEN_TRANSFORM_SHIM_H
