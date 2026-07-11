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

    /// R87 frame-unification: `slice_vert = prescale·params2.trafo·(rust_raw − voff)`.
    /// `trafo16` = f64 params2.trafo (row-major 4x4). Bit-exact via real Eigen.
    pub fn eigen_transform_verts_unified(
        trafo16: *const f64,
        scaling_factor: f64,
        voff_x: f64,
        voff_y: f64,
        voff_z: f64,
        verts_in: *const f32,
        verts_out: *mut f32,
        n: i32,
    );
}

/// Safe wrapper for [`eigen_transform_verts_unified`].
pub fn transform_verts_unified(
    trafo16: &[f64; 16],
    scaling_factor: f64,
    voff: (f64, f64, f64),
    verts_in: &[f32],
) -> Vec<f32> {
    let n = (verts_in.len() / 3) as i32;
    let mut out = vec![0.0f32; verts_in.len()];
    unsafe {
        eigen_transform_verts_unified(
            trafo16.as_ptr(),
            scaling_factor,
            voff.0,
            voff.1,
            voff.2,
            verts_in.as_ptr(),
            out.as_mut_ptr(),
            n,
        );
    }
    out
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

unsafe extern "C" {
    fn secol_min_vertex_dots(
        verts: *const f32,
        n_verts: i64,
        indices: *const i32,
        n_tris: i64,
        out: *mut f32,
    );
}

/// R187: exact-native min_vertex_dot_product kernel for the short-edge
/// collapse (ShortEdgeCollapse.cpp:43-55) — Eigen f32 pipeline compiled with
/// the same toolchain as the native binary.
pub fn min_vertex_dots(verts: &[f32], indices: &[i32]) -> Vec<f32> {
    assert!(verts.len() % 3 == 0 && indices.len() % 3 == 0);
    let n_verts = (verts.len() / 3) as i64;
    let n_tris = (indices.len() / 3) as i64;
    let mut out = vec![0.0f32; n_verts as usize];
    unsafe {
        secol_min_vertex_dots(
            verts.as_ptr(),
            n_verts,
            indices.as_ptr(),
            n_tris,
            out.as_mut_ptr(),
        );
    }
    out
}
