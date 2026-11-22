//! What Fill/Lightning of libslic3r takes from the C++ library, against vectors of the oracle's toolchain
//! (tests/data/lightning/gen.cpp): the stream of `rand()` (TreeNode.cpp:365) and the order `std::stable_sort`
//! leaves with the comparator of DistanceField.cpp:74-79, which is not a strict weak ordering.

use slicer::fill::lightning::distance_field::libcxx_stable_sort;
use slicer::fill::lightning::tree_node::{c_rand, c_srand};

#[test]
fn lightning_native_parity() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/lightning/vectors.txt");
    let text = std::fs::read_to_string(path).expect("vectors.txt");
    let (mut rands, mut sorts) = (0usize, 0usize);
    for line in text.lines() {
        let mut t = line.split(' ');
        match t.next() {
            Some("traits") => {
                // the pin's coord_t is an int; a Point is not trivially copy assignable
                assert_eq!(line, "traits 4 0");
            }
            Some("rand") => {
                let seed: u32 = t.next().unwrap().parse().unwrap();
                c_srand(seed);
                for (i, v) in t.enumerate() {
                    let expected: i32 = v.parse().unwrap();
                    assert_eq!(c_rand(), expected, "rand() number {i} after srand({seed})");
                    rands += 1;
                }
            }
            Some("ss") => {
                let n: usize = t.next().unwrap().parse().unwrap();
                let kind: u32 = t.next().unwrap().parse().unwrap();
                let expected: Vec<i32> = t.map(|v| v.parse().unwrap()).collect();
                assert_eq!(expected.len(), 2 * n);
                let mut v: Vec<(i32, i32)> = Vec::new();
                let mut s: u64 = 12345u64 + (n as u64) * 2 + kind as u64;
                for _ in 0..n {
                    s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                    let a = ((s >> 33) % 50) as i32;
                    s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                    let b = ((s >> 33) % 50) as i32;
                    v.push((a, b));
                }
                if kind == 0 {
                    libcxx_stable_sort(&mut v, |a, b| if (b.0 - a.0).abs() > 10 { a.0 < b.0 } else { (a.1 % 7) < (b.1 % 7) });
                } else {
                    libcxx_stable_sort(&mut v, |a, b| a.0 / 5 < b.0 / 5);
                }
                let got: Vec<i32> = v.iter().flat_map(|p| [p.0, p.1]).collect();
                assert_eq!(got, expected, "stable_sort of {n} elements, comparator {kind}");
                sorts += 1;
            }
            _ => panic!("unknown line: {line}"),
        }
    }
    eprintln!("{rands} values of rand(), {sorts} stable sorts: equal");
    assert_eq!((rands, sorts), (56, 42));
}
