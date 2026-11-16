//! `PathSanitizer`, `split_string` and `bbl_calc_md5` against utils.cpp / Utils.hpp at the pin.
//!
//! `tests/data/utils/vectors.txt.gz` is the output of `gen.cpp` beside it, which links the engine's
//! libslic3r and calls the three on generated inputs (the header comment there says how, and
//! describes the lines). The sanitizer is run for three user profiles (HOME), with paths under the
//! profile and its user folder, the profile with a capitalised name, other paths, cut UTF-8
//! characters; its output is compared byte for byte. The md5 cases cover the empty file and the
//! 64 KiB read buffer's edges.

use slicer::utils::{bbl_calc_md5, split_string, PathSanitizer, UserPathLayout};
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/utils/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn unhex(hex: &str) -> Vec<u8> {
    if hex == "-" {
        return Vec::new();
    }
    (0..hex.len() / 2).map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).expect("hex")).collect()
}

fn hex(b: &[u8]) -> String {
    if b.is_empty() {
        return "-".to_string();
    }
    b.iter().map(|c| format!("{:02x}", c)).collect()
}

#[test]
fn utils_match_the_pin() {
    let text = read_vectors();
    // The macOS branch of utils.cpp:952-958, the reference build's.
    let suffix_len = "/Library/Application Support/BambuStudio/user".len();
    let scratch = std::env::temp_dir().join(format!("utils_native_parity_{}", std::process::id()));
    std::fs::create_dir_all(&scratch).unwrap();
    let (mut counts, mut bad) = ([0usize; 3], 0);
    for line in text.lines() {
        let (lhs, want) = line.split_once(" -> ").expect("arrow");
        let f: Vec<&str> = lhs.split_whitespace().collect();
        let got = match f[0] {
            "SAN" => {
                counts[0] += 1;
                let home = String::from_utf8(unhex(f[1])).expect("utf-8 home");
                let layout = UserPathLayout::from_profile(Some(&home), suffix_len);
                hex(&PathSanitizer::sanitize_with_layout(unhex(f[2]), &layout))
            }
            "SPLIT" => {
                counts[1] += 1;
                let delim = u8::from_str_radix(f[1], 16).unwrap() as char;
                let s = String::from_utf8(unhex(f[2])).expect("utf-8");
                let parts = split_string(&s, delim);
                let mut out = parts.len().to_string();
                for p in &parts {
                    out += " ";
                    out += &hex(p.as_bytes());
                }
                out
            }
            "MD5" => {
                counts[2] += 1;
                let (i, n): (u64, u64) = (f[1].parse().unwrap(), f[2].parse().unwrap());
                let content: Vec<u8> =
                    (0..n).map(|k| ((k.wrapping_mul(2654435761).wrapping_add(i * 97)) >> 13) as u32 as u8).collect();
                let path = scratch.join(format!("md5_{i}.bin"));
                std::fs::write(&path, &content).unwrap();
                let mut md5 = String::new();
                assert!(bbl_calc_md5(path.to_str().unwrap(), &mut md5).unwrap());
                md5
            }
            tag => panic!("unknown tag {tag}"),
        };
        if got != want {
            bad += 1;
            if bad <= 10 {
                eprintln!("{lhs}\n   want {want}\n   got  {got}");
            }
        }
    }
    let _ = std::fs::remove_dir_all(&scratch);
    assert_eq!(counts, [1500, 300, 60]);
    assert_eq!(bad, 0);
    // A missing file: boost::filesystem::file_size throws in the C++.
    assert!(bbl_calc_md5("/nonexistent/utils_native_parity", &mut String::new()).is_err());
}
