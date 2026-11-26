//! The affine Eigen operations used by BambuStudio 56a57d0 in
//! TriangleMeshSlicer.cpp 1945–1951, 2007–2008 and MultiMaterialSegmentation.cpp 2303.
#![forbid(unsafe_code)]

/// An `Eigen::Transform<float, 3, Affine>`: the three rows of its 3x4 affine part.
pub type Transform3f = [[f32; 4]; 3];

/// `Transform3d::cast<float>()` of a row-major 4x4 of doubles.
pub fn transform3d_cast_float(trafo16: &[f64; 16]) -> Transform3f {
    let mut tf = [[0f32; 4]; 3];
    for r in 0..3 {
        for c in 0..4 {
            tf[r][c] = trafo16[r * 4 + c] as f32;
        }
    }
    tf
}

/// `tf * v` for an affine `Transform3f` and a `Vec3f`, as the C++ build
/// evaluates it: the columns of the linear part are accumulated with fused
/// multiply-adds, `fma(m2, v2, fma(m1, v1, m0 * v0))`, and the translation is
/// added last.
#[inline]
pub fn transform3f_mul_vec3f(tf: &Transform3f, v: [f32; 3]) -> [f32; 3] {
    let mut out = [0f32; 3];
    for r in 0..3 {
        let m = &tf[r];
        out[r] = m[2].mul_add(v[2], m[1].mul_add(v[1], m[0] * v[0])) + m[3];
    }
    out
}

/// `a * b` for two affine `Transform3f`: the affine part of `a` times the 4x4
/// of `b`. The C++ build sums the four products of an element without fusing
/// them, in the order `((p1 + p2) + p0) + p3`.
pub fn transform3f_mul_transform3f(a: &Transform3f, b: &Transform3f) -> Transform3f {
    let mut out = [[0f32; 4]; 3];
    let b_row3 = [0f32, 0.0, 0.0, 1.0];
    for r in 0..3 {
        for c in 0..4 {
            let p0 = a[r][0] * b[0][c];
            let p1 = a[r][1] * b[1][c];
            let p2 = a[r][2] * b[2][c];
            let p3 = a[r][3] * b_row3[c];
            out[r][c] = ((p1 + p2) + p0) + p3;
        }
    }
    out
}
