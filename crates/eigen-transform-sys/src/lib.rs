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

    fn secol_collapse(
        verts: *const f32,
        n_verts: i64,
        indices: *const i32,
        n_tris: i64,
        target_triangle_count: i64,
        out_verts: *mut f32,
        out_n_verts: *mut i64,
        out_indices: *mut i32,
        out_n_tris: *mut i64,
    ) -> i64;

    fn secol_raycast_visibility(
        verts: *const f32,
        n_verts: i64,
        indices: *const i32,
        n_tris: i64,
        sample_positions: *const f32,
        sample_normals: *const f32,
        n_samples: i64,
        sqr_rays_per_sample_point: i64,
        out_visibility: *mut f32,
    );

    fn secol_sample_uniform(
        verts: *const f32,
        n_verts: i64,
        indices: *const i32,
        n_tris: i64,
        samples_count: i64,
        out_positions: *mut f32,
        out_normals: *mut f32,
        out_tri_idx: *mut i64,
        out_total_area: *mut f32,
    );
}

/// R188b: native `sample_its_uniform_parallel` (TriangleSetSampling.cpp:9-68)
/// — the f32 cross/norm triangle areas and sample interpolation run through
/// the native Eigen codegen; RNG is the real libc++ mt19937_64 +
/// uniform_real_distribution. Returns (positions, normals, tri_indices, total_area).
pub fn sample_uniform(
    verts: &[f32],
    indices: &[i32],
    samples_count: usize,
) -> (Vec<f32>, Vec<f32>, Vec<i64>, f32) {
    assert!(verts.len() % 3 == 0 && indices.len() % 3 == 0);
    let mut positions = vec![0.0f32; 3 * samples_count];
    let mut normals = vec![0.0f32; 3 * samples_count];
    let mut tri_idx = vec![0i64; samples_count];
    let mut total_area = 0.0f32;
    unsafe {
        secol_sample_uniform(
            verts.as_ptr(),
            (verts.len() / 3) as i64,
            indices.as_ptr(),
            (indices.len() / 3) as i64,
            samples_count as i64,
            positions.as_mut_ptr(),
            normals.as_mut_ptr(),
            tri_idx.as_mut_ptr(),
            &mut total_area,
        );
    }
    (positions, normals, tri_idx, total_area)
}

/// R188: FULL native its_short_edge_collpase (ShortEdgeCollapse.cpp:11-185)
/// compiled with the native toolchain — every float decision (min dots, edge
/// squaredNorm threshold, edge_len evolution) plus libc++ std::shuffle come
/// from the same codegen as the reference binary. Returns (verts, indices).
pub fn short_edge_collapse(verts: &[f32], indices: &[i32], target: usize) -> (Vec<f32>, Vec<i32>) {
    assert!(verts.len() % 3 == 0 && indices.len() % 3 == 0);
    let n_verts = (verts.len() / 3) as i64;
    let n_tris = (indices.len() / 3) as i64;
    let mut out_verts = vec![0.0f32; verts.len()];
    let mut out_indices = vec![0i32; indices.len()];
    let mut out_n_verts: i64 = 0;
    let mut out_n_tris: i64 = 0;
    let rc = unsafe {
        secol_collapse(
            verts.as_ptr(),
            n_verts,
            indices.as_ptr(),
            n_tris,
            target as i64,
            out_verts.as_mut_ptr(),
            &mut out_n_verts,
            out_indices.as_mut_ptr(),
            &mut out_n_tris,
        )
    };
    assert_eq!(rc, 0, "secol_collapse output exceeded input-sized buffers");
    out_verts.truncate((out_n_verts * 3) as usize);
    out_indices.truncate((out_n_tris * 3) as usize);
    (out_verts, out_indices)
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

/// R190: native `raycast_visibility` (SeamPlacer.cpp:135-214, no-negative-volumes
/// branch) — AABBTreeIndirect build + f64-ray first-hit + Frame/hemisphere all in
/// native code. 112/750k edge-grazing ray decisions differed in the rust port.
pub fn raycast_visibility_native(
    verts: &[f32],
    indices: &[i32],
    sample_positions: &[f32],
    sample_normals: &[f32],
    sqr_rays_per_sample_point: usize,
) -> Vec<f32> {
    assert!(verts.len() % 3 == 0 && indices.len() % 3 == 0);
    assert_eq!(sample_positions.len(), sample_normals.len());
    let n_samples = sample_positions.len() / 3;
    let mut out = vec![0.0f32; n_samples];
    unsafe {
        secol_raycast_visibility(
            verts.as_ptr(),
            (verts.len() / 3) as i64,
            indices.as_ptr(),
            (indices.len() / 3) as i64,
            sample_positions.as_ptr(),
            sample_normals.as_ptr(),
            n_samples as i64,
            sqr_rays_per_sample_point as i64,
            out.as_mut_ptr(),
        );
    }
    out
}
