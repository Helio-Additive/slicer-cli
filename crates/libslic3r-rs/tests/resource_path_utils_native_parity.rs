//! `format::resource_path_utils` against Format/ResourcePathUtils.hpp at the pin.
//!
//! `tests/data/resource_path_utils/vectors.txt` is the output of `gen.cpp` beside it, which includes
//! the C++ header itself and runs it with Boost.Filesystem (the header comment there says how, and
//! describes the lines). Compared byte for byte: the string functions, the decomposition of a path
//! the port has to share with `boost::filesystem::path` (POSIX rules; skipped on Windows), and the
//! functions that look at the file system, on a copy of the tree the generator made.
//!
//! The vectors ask the file system only for names in their own case or for names that exist in no
//! case, so that they hold on a file system that ignores case and on one that does not;
//! `case_insensitive_resolution` covers the other requests, with the answer each kind of file
//! system gives according to ResourcePathUtils.hpp:138-181.

use std::path::{Path, PathBuf};

use slicer::format::resource_path_utils::*;

const VECTORS: &str = include_str!("data/resource_path_utils/vectors.txt");

fn unhex(hex: &str) -> Vec<u8> {
    if hex == "-" {
        return Vec::new();
    }
    (0..hex.len() / 2).map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).expect("hex")).collect()
}

fn text(hex: &str) -> String {
    String::from_utf8(unhex(hex)).expect("utf-8")
}

/// The fields of every line tagged `tag`, without the tag and the arrow.
fn lines(tag: &str) -> Vec<Vec<&'static str>> {
    VECTORS
        .lines()
        .filter_map(|line| {
            let mut fields = line.split_whitespace();
            (fields.next() == Some(tag)).then(|| fields.filter(|f| *f != "->").collect())
        })
        .collect()
}

#[test]
fn hex_digits() {
    let cases = lines("HEX");
    assert_eq!(cases.len(), 1);
    assert_eq!(cases[0].len(), 256);
    for (c, want) in cases[0].iter().enumerate() {
        assert_eq!(hex_digit_value(c as u8), want.parse::<i32>().unwrap(), "byte {c}");
    }
}

#[test]
fn string_functions() {
    let cases = lines("STR");
    assert_eq!(cases.len(), 700);
    let (mut stripped, mut remote, mut decoded) = (0, 0, 0);
    for f in &cases {
        let input = text(f[0]);
        assert_eq!(ascii_lower_copy(&input).as_bytes(), unhex(f[1]), "ascii_lower_copy({input:?})");
        assert_eq!(percent_decode_copy(&input), unhex(f[2]), "percent_decode_copy({input:?})");
        assert_eq!(strip_file_uri_prefix_copy(&input).as_bytes(), unhex(f[3]), "strip_file_uri_prefix_copy({input:?})");
        assert_eq!(file_uri_has_remote_authority(&input), f[4] == "1", "file_uri_has_remote_authority({input:?})");
        stripped += usize::from(unhex(f[3]) != input.as_bytes());
        remote += usize::from(f[4] == "1");
        decoded += usize::from(unhex(f[2]) != input.as_bytes());
    }
    assert!(stripped > 30 && remote > 15 && decoded > 100, "{stripped} stripped, {remote} remote, {decoded} decoded");
}

#[cfg(unix)]
#[test]
fn path_decomposition() {
    let cases = lines("PATH");
    assert_eq!(cases.len(), 500);
    for f in &cases {
        let input = unhex(f[0]);
        let at = String::from_utf8_lossy(&input).into_owned();
        assert_eq!(boost_path::root_name(&input), unhex(f[1]), "root_name({at:?})");
        assert_eq!(boost_path::root_directory(&input), unhex(f[2]), "root_directory({at:?})");
        assert_eq!(boost_path::is_absolute(&input), f[3] == "1", "is_absolute({at:?})");
        assert_eq!(boost_path::filename(&input), unhex(f[4]), "filename({at:?})");
        let n: usize = f[5].parse().unwrap();
        let want: Vec<Vec<u8>> = f[6..6 + n].iter().map(|h| unhex(h)).collect();
        assert_eq!(boost_path::elements(&input), want, "elements({at:?})");
        let path = boost_path::path(&input);
        assert_eq!(boost_path::string(&portable_path_copy(&path)), unhex(f[6 + n]), "portable_path_copy({at:?})");
        assert_eq!(looks_like_windows_absolute_path(&path), f[7 + n] == "1", "looks_like_windows_absolute_path({at:?})");
        assert_eq!(boost_path::string(&filename_from_portable_path(&path)), unhex(f[8 + n]), "filename_from_portable_path({at:?})");
    }
    let cases = lines("JOIN");
    assert_eq!(cases.len(), 200);
    for f in &cases {
        let (a, b) = (unhex(f[0]), unhex(f[1]));
        assert_eq!(boost_path::append(&a, &b), unhex(f[2]), "{:?} / {:?}", String::from_utf8_lossy(&a), String::from_utf8_lossy(&b));
    }
}

/// A fresh directory under the system's temporary directory, as the operating system names it.
fn fresh_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("{name}_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir.canonicalize().unwrap()
}

/// The file system functions. This is the one test that changes the current directory, which the
/// relative cases need; no other test of this file depends on it.
#[cfg(unix)]
#[test]
fn file_system_functions() {
    let root = fresh_dir("resource_path_utils_vectors");
    let root_text = root.to_str().unwrap().to_string();
    for f in lines("DIR") {
        std::fs::create_dir_all(root.join(text(f[0]))).unwrap();
    }
    for f in lines("FILE") {
        std::fs::write(root.join(text(f[0])), "x").unwrap();
    }
    std::env::set_current_dir(&root).unwrap();

    let rooted = |hex: &str| text(hex).replace("$ROOT", &root_text);
    let shown = |path: &Path| path.to_str().unwrap().to_string();

    let cases = lines("CHILD");
    assert_eq!(cases.len(), 187);
    let mut found = 0;
    for f in &cases {
        let (directory, name, want) = (rooted(f[0]), text(f[1]), rooted(f[2]));
        let got = find_child_case_insensitive(Path::new(&directory), Path::new(&name), "test");
        assert_eq!(shown(&got), want, "find_child_case_insensitive({directory:?}, {name:?})");
        found += usize::from(!want.is_empty());
    }
    assert!(found >= 25, "{found}");

    let cases = lines("EXIST");
    assert_eq!(cases.len(), 47);
    for f in &cases {
        let (path, want) = (rooted(f[0]), rooted(f[1]));
        let got = resolve_existing_path_case_insensitive(Path::new(&path), "test");
        assert_eq!(shown(&got), want, "resolve_existing_path_case_insensitive({path:?})");
    }

    let cases = lines("REL");
    assert_eq!(cases.len(), 88);
    for f in &cases {
        let (base, path, want) = (rooted(f[0]), rooted(f[1]), rooted(f[2]));
        let got = resolve_existing_relative_path_case_insensitive(Path::new(&base), Path::new(&path), "test");
        assert_eq!(shown(&got), want, "resolve_existing_relative_path_case_insensitive({base:?}, {path:?})");
    }

    let cases = lines("EXT");
    assert_eq!(cases.len(), 396);
    let mut found = 0;
    for f in &cases {
        let (base, raw, fallback, want) = (rooted(f[0]), rooted(f[1]), f[2] == "1", rooted(f[3]));
        let got = resolve_external_resource_path(Path::new(&base), &raw, "test", fallback);
        assert_eq!(shown(&got), want, "resolve_external_resource_path({base:?}, {raw:?}, {fallback})");
        found += usize::from(!want.is_empty());
    }
    assert!(found >= 100, "{found}");

    std::env::set_current_dir(std::env::temp_dir()).unwrap();
    std::fs::remove_dir_all(&root).unwrap();
}

/// Requests in another case than the names have. No C++ reference: the answer depends on the file
/// system, and the vectors were made on one that ignores case. Read off
/// ResourcePathUtils.hpp:138-181: where the file system finds the path as it was asked for, the
/// path comes back as asked; where it does not, every element is looked up in its directory.
#[test]
fn case_insensitive_resolution() {
    let root = fresh_dir("resource_path_utils_case");
    std::fs::create_dir_all(root.join("Models/Textures")).unwrap();
    std::fs::write(root.join("Models/Textures/Wood.PNG"), "x").unwrap();
    let ignores_case = root.join("models").exists();

    let asked = root.join("models/TEXTURES/wood.png");
    let got = resolve_existing_path_case_insensitive(&asked, "test");
    let want = if ignores_case { asked.clone() } else { root.join("Models/Textures/Wood.PNG") };
    assert_eq!(got, want);
    assert!(got.exists());

    // Relative to a base directory, with backslashes, percent encoding and a file URI.
    let got = resolve_existing_relative_path_case_insensitive(&root.join("Models"), Path::new("textures\\WOOD.png"), "test");
    let want =
        if ignores_case { root.join("Models/textures/WOOD.png") } else { root.join("Models/Textures/Wood.PNG") };
    assert_eq!(got, want);
    let uri = format!("file://{}/MODELS/textures/wood%2Epng", root.to_str().unwrap());
    let got = resolve_external_resource_path(Path::new("unused"), &uri, "test", true);
    let want =
        if ignores_case { root.join("MODELS/textures/wood.png") } else { root.join("Models/Textures/Wood.PNG") };
    assert_eq!(got, want);

    // The directory is gone: only the name is left to go by, when that is allowed.
    let base = root.join("Models/Textures");
    let got = resolve_external_resource_path(&base, "D:\\work\\maps\\WOOD.png", "test", true);
    let want = if ignores_case { base.join("WOOD.png") } else { base.join("Wood.PNG") };
    assert_eq!(got, want);
    assert_eq!(resolve_external_resource_path(&base, "D:\\work\\maps\\WOOD.png", "test", false), PathBuf::new());
    // Not for a file on another host.
    assert_eq!(resolve_external_resource_path(&base, "file://server/maps/Wood.PNG", "test", true), PathBuf::new());

    // Two names that differ only in case: no answer (a file system that ignores case cannot hold both).
    if !ignores_case {
        std::fs::write(root.join("Models/Textures/wood.png"), "x").unwrap();
        assert_eq!(find_child_case_insensitive(&base, Path::new("WOOD.PNG"), "test"), PathBuf::new());
        assert_eq!(resolve_existing_path_case_insensitive(&base.join("WOOD.PNG"), "test"), PathBuf::new());
    }
    std::fs::remove_dir_all(&root).unwrap();
}
