//! Format/3mf's PrusaFileParser (3mf.hpp:7-27, 3mf.cpp:274-358) against the pin: the check the
//! pin's `Model::read_from_archive` runs on every .3mf before reading it (Model.cpp:494-510).
//! Vectors from `tests/data/three_mf/gen.cpp`: 600 generated model documents parsed by expat with
//! the parser's own handlers (3mf.cpp:301-316), and `check_3mf_from_prusa` on 3MF files (the
//! crafted ones here, the repository's 3MF fixtures, and the Majora fixture when it is present).
use slicer::format::three_mf::PrusaFileParser;
use std::io::Read;

fn vectors() -> String {
    let path = format!("{}/tests/data/three_mf/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).expect("vectors");
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn unhex(s: &str) -> Vec<u8> {
    if s == "-" {
        return Vec::new();
    }
    (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
}

#[test]
fn prusa_file_parser_handlers_match_the_pin() {
    let text = vectors();
    let (mut n, mut bad) = (0, Vec::new());
    for line in text.lines().filter(|l| l.starts_with("xml ")) {
        let f: Vec<&str> = line.split(' ').collect();
        let doc = unhex(f[1]);
        let mut p = PrusaFileParser::new();
        p.parse_prusa_buffer(&doc);
        let got = (p.m_from_prusa as i32, p.m_is_application_key as i32);
        let want = (f[2].parse::<i32>().unwrap(), f[3].parse::<i32>().unwrap());
        if got != want {
            bad.push(format!("{:?} got {:?} want {:?}", String::from_utf8_lossy(&doc), got, want));
        }
        n += 1;
    }
    assert_eq!(n, 600);
    assert!(bad.is_empty(), "{} of {} documents differ, first: {}", bad.len(), n, bad[0]);
}

#[test]
fn check_3mf_from_prusa_matches_the_pin() {
    let text = vectors();
    let repo = format!("{}/../..", env!("CARGO_MANIFEST_DIR"));
    let (mut n, mut skipped) = (0, 0);
    for line in text.lines().filter(|l| l.starts_with("file ")) {
        let f: Vec<&str> = line.split(' ').collect();
        let path = format!("{}/{}", repo, f[1]);
        // The Majora fixture is not in the repository (.gitignore); its vector is checked where it is.
        if f[1].starts_with("tests/rust-engine/fixtures/") && !std::path::Path::new(&path).exists() {
            skipped += 1;
            continue;
        }
        let mut p = PrusaFileParser::new();
        assert_eq!(p.check_3mf_from_prusa(&path) as i32, f[2].parse::<i32>().unwrap(), "{}", f[1]);
        n += 1;
    }
    assert!(n >= 12, "{} files checked, {} skipped", n, skipped);
}
