//! R954: `crate::sla::bicubic` (SLA/bicubic.h) against the pin's arithmetic. The pin's header does
//! not compile as it stands (bicubic.h:96, 152, 176; nothing includes it), so the generator
//! (tests/data/sla_bicubic/gen.cpp) carries bicubic.h:12-129 verbatim but for line 96 and compiles it
//! with the engine's flags: the 3 kernels' 16 coefficients for float and double, `CubicKernel::kernel`
//! and `CubicKernel::interpolate` on LCG inputs (300 + 300 per kernel and type), and
//! `BicubicInternal::clamp` (40 int cases). Results must be bit-equal.

use slicer::sla::bicubic::bicubic_internal::{clamp, CubicBSplineKernel, CubicCatmulRomKernel, LinearKernel};
use slicer::sla::bicubic::{BicubicFloat, CubicKernel, KernelCoeffs};
use std::io::Read;

/// `%a` of a double (glibc / Apple libc form: `[-]0x1.hhhp[+-]d`, `0x0p+0`).
fn parse_hexf(s: &str) -> f64 {
    let (neg, s) = match s.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, s),
    };
    let s = s.strip_prefix("0x").expect("0x");
    let (mant, exp) = s.split_once('p').expect("p");
    let (int_part, frac_part) = mant.split_once('.').unwrap_or((mant, ""));
    let mut m: u64 = u64::from_str_radix(int_part, 16).unwrap();
    for c in frac_part.chars() {
        m = (m << 4) | c.to_digit(16).unwrap() as u64;
    }
    let e: i32 = exp.parse::<i32>().unwrap() - 4 * frac_part.len() as i32;
    let v = m as f64 * 2f64.powi(e);
    if neg {
        -v
    } else {
        v
    }
}

fn coeffs<K: KernelCoeffs>() -> Vec<f64>
where
    K::FloatType: Into<f64>,
{
    vec![
        K::k00(), K::k01(), K::k02(), K::k03(), K::k10(), K::k11(), K::k12(), K::k13(),
        K::k20(), K::k21(), K::k22(), K::k23(), K::k30(), K::k31(), K::k32(), K::k33(),
    ]
    .into_iter()
    .map(Into::into)
    .collect()
}

/// One kernel and type: (coefficients, kernel(x), interpolate(f0..f3, x)) as doubles.
fn eval<K: KernelCoeffs>(kind: char, args: &[f64]) -> f64
where
    K::FloatType: Into<f64>,
{
    let t = |v: f64| <K::FloatType as BicubicFloat>::from_f64(v);
    match kind {
        'K' => CubicKernel::<K>::kernel(t(args[0])).into(),
        'I' => CubicKernel::<K>::interpolate(t(args[0]), t(args[1]), t(args[2]), t(args[3]), t(args[4])).into(),
        _ => unreachable!(),
    }
}

#[test]
fn sla_bicubic_kernels_match_the_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/sla_bicubic/vectors.txt.gz");
    let mut text = String::new();
    flate2::read::GzDecoder::new(std::fs::File::open(path).expect("vectors")).read_to_string(&mut text).expect("gzip");
    let (mut n_c, mut n_k, mut n_i, mut n_m) = (0, 0, 0, 0);
    let mut bad = Vec::new();
    for line in text.lines() {
        let w: Vec<&str> = line.split(' ').collect();
        match w[0] {
            "C" => {
                let want: Vec<u64> = w[3..].iter().map(|s| parse_hexf(s).to_bits()).collect();
                let got: Vec<u64> = match (w[1], w[2]) {
                    ("Linear", "float") => coeffs::<LinearKernel<f32>>(),
                    ("Linear", "double") => coeffs::<LinearKernel<f64>>(),
                    ("CatmulRom", "float") => coeffs::<CubicCatmulRomKernel<f32>>(),
                    ("CatmulRom", "double") => coeffs::<CubicCatmulRomKernel<f64>>(),
                    ("BSpline", "float") => coeffs::<CubicBSplineKernel<f32>>(),
                    ("BSpline", "double") => coeffs::<CubicBSplineKernel<f64>>(),
                    _ => unreachable!(),
                }
                .into_iter()
                .map(f64::to_bits)
                .collect();
                if got != want {
                    bad.push(line.to_string());
                }
                n_c += 1;
            }
            "K" | "I" => {
                let kind = w[0].chars().next().unwrap();
                let vals: Vec<f64> = w[3..].iter().map(|s| parse_hexf(s)).collect();
                let (args, want) = vals.split_at(vals.len() - 1);
                let got = match (w[1], w[2]) {
                    ("Linear", "float") => eval::<LinearKernel<f32>>(kind, args),
                    ("Linear", "double") => eval::<LinearKernel<f64>>(kind, args),
                    ("CatmulRom", "float") => eval::<CubicCatmulRomKernel<f32>>(kind, args),
                    ("CatmulRom", "double") => eval::<CubicCatmulRomKernel<f64>>(kind, args),
                    ("BSpline", "float") => eval::<CubicBSplineKernel<f32>>(kind, args),
                    ("BSpline", "double") => eval::<CubicBSplineKernel<f64>>(kind, args),
                    _ => unreachable!(),
                };
                if got.to_bits() != want[0].to_bits() {
                    bad.push(format!("{line} -> got {got:e}"));
                }
                if kind == 'K' {
                    n_k += 1;
                } else {
                    n_i += 1;
                }
            }
            "M" => {
                let v: Vec<i32> = w[1..].iter().map(|s| s.parse().unwrap()).collect();
                if clamp(v[0], v[1], v[2]) != v[3] {
                    bad.push(line.to_string());
                }
                n_m += 1;
            }
            _ => panic!("unknown line {line}"),
        }
    }
    assert_eq!((n_c, n_k, n_i, n_m), (6, 1800, 1800, 40));
    assert!(bad.is_empty(), "{} lines differ:\n{}", bad.len(), bad[..bad.len().min(10)].join("\n"));
}
