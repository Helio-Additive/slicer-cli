//! Low-level FFI binding to the Eigen transform shim (see
//! `shim/eigen_transform_shim.{h,cpp}`).
//!
//! The shim calls the REAL Eigen with the exact `make_trafo_for_slicing` sequence
//! libslic3r uses, so the f32 vertex transform is bit-exact to the C++ slicer
//! (sidesteps the pure-rust Eigen-reproduction 1-ULP wall, R85). Only runtime
//! dependency is the C++ standard library (statically linked).

extern "C" {
    /// Transform `n` vertices (`verts_in`: 3*n interleaved x,y,z f32) into the
    /// slice-time centered frame, writing `verts_out` (3*n f32). Replicates
    /// `make_trafo_for_slicing` + `transform_mesh_vertices_for_slicing` exactly:
    /// `tf = (Identity.pretranslate(-cx,-cy,0).prescale(1/sf,1/sf,1)).cast<float>()`,
    /// then per vertex `out = tf * v` (Eigen `Affine3f * Vector3f`). `scaling_factor`
    /// = SCALING_FACTOR (1e-5); `cx`/`cy` = unscaled (mm) center_offset.
    pub fn eigen_transform_verts_for_slicing(
        scaling_factor: f64,
        cx: f64,
        cy: f64,
        verts_in: *const f32,
        verts_out: *mut f32,
        n: i32,
    );
}

/// Safe wrapper: transform `verts` (slice of [x,y,z] f32 triples flattened) into
/// the slice-time centered frame, returning a new flat Vec. See
/// [`eigen_transform_verts_for_slicing`].
pub fn transform_verts_for_slicing(
    scaling_factor: f64,
    cx: f64,
    cy: f64,
    verts_in: &[f32],
) -> Vec<f32> {
    let n = (verts_in.len() / 3) as i32;
    let mut out = vec![0.0f32; verts_in.len()];
    // SAFETY: verts_in has 3*n f32; out is allocated 3*n; the shim only reads
    // verts_in and writes verts_out within [0, 3*n).
    unsafe {
        eigen_transform_verts_for_slicing(
            scaling_factor,
            cx,
            cy,
            verts_in.as_ptr(),
            out.as_mut_ptr(),
            n,
        );
    }
    out
}
