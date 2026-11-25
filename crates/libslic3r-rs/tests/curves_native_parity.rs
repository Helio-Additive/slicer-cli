//! Native parity of Geometry/Bicubic.hpp `CubicKernelWrapper<...<float>>::kernel` (Bicubic.hpp:198-213)
//! and Geometry/Curves.hpp `fit_cubic_bspline` / `get_fitted_value` (Curves.hpp:66-170, Dimension 3, float),
//! against vectors printed by `tests/data/curves/gen.cpp` (the pin's own templates).
//!
//! The kernel is asserted bit-equal (`cubic_kernel_f32`, R938). The fit is asserted bit-equal since
//! R951: the port solves with `FullPivHouseholderQrF32`, Eigen's `fullPivHouseholderQr` as the pin's
//! build runs it (Curves.hpp:168-173; gates CURVES_FULLPIV_QR and CURVES_KERNEL_F32, both faithful).

use slicer::geometry::bicubic::{CubicBSplineKernel, CubicCatmulRomKernel, LinearKernel};
use slicer::geometry::curves::{cubic_kernel_f32, fit_cubic_bspline};

/// Parses C's `%a` output (a double; the generator prints floats promoted to double).
fn hexf(s: &str) -> f64 {
    let (neg, s) = match s.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, s),
    };
    let s = s.strip_prefix("0x").expect("hex float");
    let (mant, exp) = s.split_once('p').expect("exponent");
    let exp: i32 = exp.parse().expect("exp");
    let (int_part, frac_part) = mant.split_once('.').unwrap_or((mant, ""));
    let digits = format!("{int_part}{frac_part}");
    let m = u64::from_str_radix(&digits, 16).expect("mantissa");
    let v = (m as f64) * 2f64.powi(exp - 4 * frac_part.len() as i32);
    if neg {
        -v
    } else {
        v
    }
}

fn hf(s: &str) -> f32 {
    hexf(s) as f32
}

fn vectors() -> String {
    let path = std::env::var("CURVES_PARITY_VECTORS").unwrap_or_else(|_| {
        format!("{}/tests/data/curves/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"))
    });
    let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    if path.ends_with(".gz") {
        use std::io::Read;
        let mut s = String::new();
        flate2::read::GzDecoder::new(&bytes[..]).read_to_string(&mut s).expect("gunzip");
        s
    } else {
        String::from_utf8(bytes).expect("utf8")
    }
}

#[test]
fn curves_native_parity() {
    let text = vectors();
    let (mut k_cases, mut k_bad) = (0usize, 0usize);
    let (mut fit_cases, mut fit_coef_bad, mut fit_value_bad) = (0usize, 0usize, 0usize);
    for line in text.lines() {
        let (lhs, rhs) = line.split_once(" => ").expect("=>");
        let l: Vec<&str> = lhs.split_whitespace().collect();
        match l[0] {
            "k" => {
                let x = hf(l[2]);
                let want = hf(rhs.trim());
                let got = match l[1] {
                    "b" => cubic_kernel_f32::<CubicBSplineKernel>(x),
                    "c" => cubic_kernel_f32::<CubicCatmulRomKernel>(x),
                    "l" => cubic_kernel_f32::<LinearKernel>(x),
                    k => panic!("kernel kind {k}"),
                };
                k_cases += 1;
                if got.to_bits() != want.to_bits() {
                    k_bad += 1;
                    if k_bad <= 5 {
                        eprintln!("kernel {} x={x:e}: rust {got:e} pin {want:e}", l[1]);
                    }
                }
            }
            "fit" => {
                let n: usize = l[1].parse().unwrap();
                let segments: usize = l[2].parse().unwrap();
                let mut obs = Vec::new();
                let mut pts = Vec::new();
                let mut ws = Vec::new();
                for i in 0..n {
                    let b = 3 + 5 * i;
                    obs.push(vec![hf(l[b]), hf(l[b + 1]), hf(l[b + 2])]);
                    pts.push(hf(l[b + 3]));
                    ws.push(hf(l[b + 4]));
                }
                let (coefs, values) = rhs.split_once(" |").expect("|");
                let c: Vec<&str> = coefs.split_whitespace().collect();
                let cols: usize = c[0].parse().unwrap();
                let curve = fit_cubic_bspline(&obs, &pts, &ws, segments, 0, 3);
                fit_cases += 1;
                let mut coef_equal = curve.coefficients.ncols() == cols
                    && curve.start.to_bits() == hf(c[1 + 3 * cols]).to_bits()
                    && curve.segment_size.to_bits() == hf(c[2 + 3 * cols]).to_bits();
                if coef_equal {
                    for r in 0..3 {
                        for col in 0..cols {
                            if curve.coefficients[(r, col)].to_bits() != hf(c[1 + r * cols + col]).to_bits() {
                                coef_equal = false;
                            }
                        }
                    }
                }
                if !coef_equal {
                    fit_coef_bad += 1;
                }
                let v: Vec<&str> = values.split_whitespace().collect();
                let mut value_equal = true;
                for i in 0..n {
                    let got = curve.get_fitted_value::<CubicBSplineKernel>(pts[i]);
                    for d in 0..3 {
                        if got[d].to_bits() != hf(v[3 * i + d]).to_bits() {
                            value_equal = false;
                        }
                    }
                }
                if !value_equal {
                    fit_value_bad += 1;
                }
            }
            k => panic!("line kind {k}"),
        }
    }
    eprintln!(
        "curves: kernel {k_cases} cases, {k_bad} differ; fit {fit_cases} cases: coefficients differ in {fit_coef_bad}, fitted values differ in {fit_value_bad} "
    );
    assert!(k_cases > 0 && fit_cases > 0);
    assert_eq!(k_bad, 0, "cubic_kernel_f32 differs from the pin");
    // R951: Curves.hpp:168-173 solved as Eigen's FullPivHouseholderQR (FullPivHouseholderQrF32).
    assert_eq!(fit_coef_bad, 0, "fit_cubic_bspline coefficients differ from the pin");
    assert_eq!(fit_value_bad, 0, "get_fitted_value differs from the pin");
}
