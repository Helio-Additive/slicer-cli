//! is_plain_file, is_ini_file, is_idx_file and is_gallery_file(directory_entry, ".png")
//! (utils.cpp:867-904) against the pin's over the directory `tests/data/utils_file_kinds/gen.cpp` lays
//! out: a symlink to a file is a plain file (`directory_entry::status()` follows it), a dot file
//! ".ini" has the extension ".ini" (boost::filesystem v3).

#[cfg(unix)]
#[test]
fn file_kinds_match_the_pin() {
    use slicer::utils::{is_gallery_file_entry, is_idx_file, is_ini_file, is_plain_file};
    use std::collections::BTreeMap;
    let expected: BTreeMap<String, String> =
        std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/utils_file_kinds/vectors.txt"))
            .unwrap()
            .lines()
            .map(|l| {
                let (name, rest) = l.split_once(' ').unwrap();
                (name.to_string(), rest.to_string())
            })
            .collect();
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("utils_file_kinds_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    for f in ["a.ini", "b.INI", "c.Idx", "d.txt", ".ini", "e.ini.bak", "f.", "g.png", "h.PNG", "noext", "x.y.idx"] {
        std::fs::write(dir.join(f), "x").unwrap();
    }
    std::fs::create_dir(dir.join("sub.ini")).unwrap();
    std::os::unix::fs::symlink("a.ini", dir.join("link.ini")).unwrap();
    std::os::unix::fs::symlink("missing.ini", dir.join("broken.ini")).unwrap();
    std::os::unix::fs::symlink("sub.ini", dir.join("dirlink.ini")).unwrap();
    let mut got: BTreeMap<String, String> = BTreeMap::new();
    for e in std::fs::read_dir(&dir).unwrap() {
        let e = e.unwrap();
        got.insert(
            e.file_name().to_string_lossy().into_owned(),
            format!(
                "{} {} {} {}",
                is_plain_file(&e) as i32,
                is_ini_file(&e) as i32,
                is_idx_file(&e) as i32,
                is_gallery_file_entry(&e, ".png") as i32
            ),
        );
    }
    let _ = std::fs::remove_dir_all(&dir);
    assert_eq!(got, expected);
    assert_eq!(expected.len(), 15);
}
