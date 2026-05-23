use std::collections::HashSet;
use std::path::Path;

fn main() {
    let manifest = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    let build_dir = format!("{manifest}/build");

    // Force-load the thin shim so the linker always includes slicer_run,
    // slicer_list_presets, slicer_get_preset even in a one-pass link.
    let shim = format!("{build_dir}/libslicer_shim.a");
    if cfg!(target_os = "macos") {
        println!("cargo:rustc-link-arg=-Wl,-force_load,{shim}");
    } else {
        println!("cargo:rustc-link-arg=-Wl,--whole-archive");
        println!("cargo:rustc-link-arg={shim}");
        println!("cargo:rustc-link-arg=-Wl,--no-whole-archive");
    }

    // All remaining libraries come from cmake's slicer_cli link.txt.
    emit_cmake_link_flags(&build_dir);

    cxx_build::bridge("src/ffi.rs")
        .include(".")
        .include(&build_dir)
        .flag_if_supported("-std=c++17")
        .compile("slicer_bridge");

    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=libslic3r/bambustudio/shim.cpp");
    println!("cargo:rerun-if-changed=libslic3r/bambustudio/shim.hpp");
    println!("cargo:rerun-if-changed=build/CMakeFiles/slicer_cli.dir/link.txt");
}

fn emit_cmake_link_flags(build_dir: &str) {
    let link_txt = format!("{build_dir}/CMakeFiles/slicer_cli.dir/link.txt");
    let content = std::fs::read_to_string(&link_txt)
        .unwrap_or_else(|_| panic!("cmake link.txt not found — run `just configure` first: {link_txt}"));

    let mut seen_dirs: HashSet<String> = HashSet::new();
    let mut seen_libs: HashSet<String> = HashSet::new();

    for token in content.split_whitespace() {
        if let Some(path) = token.strip_prefix("-L") {
            emit_search(path, &mut seen_dirs);
        } else if let Some(lib) = token.strip_prefix("-l") {
            if lib != "m" && seen_libs.insert(lib.to_string()) {
                println!("cargo:rustc-link-lib={lib}");
            }
        } else if token.ends_with(".a") || token.ends_with(".dylib") || token.ends_with(".so") {
            let p = if Path::new(token).is_absolute() {
                token.to_string()
            } else {
                format!("{build_dir}/{token}")
            };
            let path = Path::new(&p);
            if let (Some(dir), Some(fname)) = (path.parent(), path.file_name()) {
                let fname = fname.to_string_lossy();
                let name = fname
                    .strip_prefix("lib")
                    .and_then(|n| n.rsplit_once('.').map(|(base, _)| base))
                    .unwrap_or(&fname);
                // Strip version segments from dylib names (libFoo.1.2.3.dylib → Foo).
                let name = name.split('.').next().unwrap_or(name);
                let kind = if p.ends_with(".a") { "static" } else { "dylib" };
                let key = format!("{kind}={name}");
                emit_search(dir.to_str().unwrap(), &mut seen_dirs);
                if seen_libs.insert(key.clone()) {
                    println!("cargo:rustc-link-lib={key}");
                }
            }
        }
    }

    if cfg!(target_os = "macos") {
        println!("cargo:rustc-link-lib=c++");
    } else {
        println!("cargo:rustc-link-lib=stdc++");
    }
}

fn emit_search(dir: &str, seen: &mut HashSet<String>) {
    if !dir.is_empty() && seen.insert(dir.to_string()) {
        println!("cargo:rustc-link-search=native={dir}");
    }
}
