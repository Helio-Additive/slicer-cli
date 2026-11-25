//! Native parity of `FullPivHouseholderQrF32` (Eigen 5.0.1 `FullPivHouseholderQR<MatrixXf>` as the
//! pin's build runs it, Curves.hpp:168-173) and of `fit_cubic_bspline` with Dimension 2 as
//! SeamPlacer.cpp:1340-1341 calls it. Vectors: `tests/data/curves_fullpiv_qr/gen.cpp`
//! (`gen 400 | gzip -9n > vectors.txt.gz`). Every float is compared by its bits.

use nalgebra::DMatrix;
use slicer::geometry::curves::{fit_cubic_bspline, FullPivHouseholderQrF32};

fn vectors() -> String {
    let path = std::env::var("CURVES_FULLPIV_QR_VECTORS").unwrap_or_else(|_| {
        format!("{}/tests/data/curves_fullpiv_qr/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"))
    });
    let bytes = std::fs::read(&path).expect("vectors");
    if path.ends_with(".gz") {
        use std::io::Read;
        let mut s = String::new();
        flate2::read::GzDecoder::new(&bytes[..]).read_to_string(&mut s).expect("gunzip");
        s
    } else {
        String::from_utf8(bytes).expect("utf8")
    }
}

fn f(s: &str) -> f32 {
    f32::from_bits(u32::from_str_radix(s, 16).expect("hex bits"))
}

#[test]
fn full_piv_householder_qr_and_fit_match_the_pin() {
    let text = vectors();
    let (mut q_cases, mut q_bad, mut rank_bad, mut f_cases, mut f_bad) = (0usize, 0usize, 0usize, 0usize, 0usize);
    for line in text.lines() {
        let mut t = line.split_whitespace();
        match t.next().expect("kind") {
            "q" => {
                let rows: usize = t.next().unwrap().parse().unwrap();
                let cols: usize = t.next().unwrap().parse().unwrap();
                let data: Vec<f32> = (0..rows * cols).map(|_| f(t.next().unwrap())).collect();
                let b: Vec<f32> = (0..rows).map(|_| f(t.next().unwrap())).collect();
                assert_eq!(t.next(), Some("=>"));
                let rank: usize = t.next().unwrap().parse().unwrap();
                let expected: Vec<f32> = (0..cols).map(|_| f(t.next().unwrap())).collect();
                let m = DMatrix::<f32>::from_column_slice(rows, cols, &data);
                let qr = FullPivHouseholderQrF32::new(&m);
                let x = qr.solve(&b);
                q_cases += 1;
                if qr.rank() != rank {
                    rank_bad += 1;
                }
                let bad = x.iter().zip(&expected).filter(|(a, e)| a.to_bits() != e.to_bits()).count();
                if bad > 0 {
                    if q_bad < 5 {
                        eprintln!("q case {q_cases} ({rows}x{cols}): {bad} of {cols} differ, e.g. {:?} vs {:?}", &x[..cols.min(3)], &expected[..cols.min(3)]);
                    }
                    q_bad += 1;
                }
            }
            "f" => {
                let n: usize = t.next().unwrap().parse().unwrap();
                let segments: usize = t.next().unwrap().parse().unwrap();
                let (mut obs, mut pts, mut ws) = (Vec::new(), Vec::new(), Vec::new());
                for _ in 0..n {
                    let x = f(t.next().unwrap());
                    let y = f(t.next().unwrap());
                    obs.push(vec![x, y]);
                    pts.push(f(t.next().unwrap()));
                    ws.push(f(t.next().unwrap()));
                }
                assert_eq!(t.next(), Some("=>"));
                let cols: usize = t.next().unwrap().parse().unwrap();
                let expected: Vec<f32> = (0..2 * cols).map(|_| f(t.next().unwrap())).collect();
                let curve = fit_cubic_bspline(&obs, &pts, &ws, segments, 0, 2);
                f_cases += 1;
                assert_eq!(curve.coefficients.ncols(), cols);
                let mut bad = 0;
                for r in 0..2 {
                    for c in 0..cols {
                        if curve.coefficients[(r, c)].to_bits() != expected[r * cols + c].to_bits() {
                            bad += 1;
                        }
                    }
                }
                if bad > 0 {
                    if f_bad < 5 {
                        eprintln!("f case {f_cases} (n {n}, {segments} segments): {bad} of {} coefficients differ", 2 * cols);
                    }
                    f_bad += 1;
                }
            }
            k => panic!("unknown line kind {k}"),
        }
    }
    eprintln!(
        "curves_fullpiv_qr: {q_cases} solves ({q_bad} differ, {rank_bad} ranks differ); {f_cases} fits ({f_bad} differ)"
    );
    assert!(q_cases > 0 && f_cases > 0);
    assert_eq!(rank_bad, 0, "FullPivHouseholderQrF32::rank differs from the pin");
    assert_eq!(q_bad, 0, "FullPivHouseholderQrF32::solve differs from the pin");
    assert_eq!(f_bad, 0, "fit_cubic_bspline differs from the pin");
}
