// Builds the Eigen transform C ABI shim into a static library. The shim calls the
// REAL header-only Eigen (vendored in the BambuStudio references tree, same copy
// libslic3r uses) with the exact make_trafo_for_slicing sequence, so the f32
// matmul is bit-exact to the C++ slicer.
//
// CRITICAL for bit-exactness: the C++ libslic3r build is `-O3 -DNDEBUG` on arm64.
// Eigen's f32 Matrix*Vector codegen (NEON / FMA / reduction order) depends on the
// optimization level, so we FORCE -O3 here regardless of the cargo profile (a
// debug -O0 build would change the f32 rounding and reintroduce the 1-ULP drift).

use std::path::PathBuf;
use std::process::Command;

fn eigen_include_dir() -> PathBuf {
    // 1. Try pkg-config eigen3 (devbox provides it) — but prefer the SAME vendored
    //    Eigen libslic3r compiles against (version match guarantees identical codegen).
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let vendored = manifest.join("../../libslic3r/bambustudio/references/BambuStudio/src/eigen");
    if vendored.join("Eigen/Geometry").exists() {
        return vendored;
    }
    if let Ok(out) = Command::new("pkg-config")
        .args(["--cflags-only-I", "eigen3"])
        .output()
    {
        if out.status.success() {
            let s = String::from_utf8_lossy(&out.stdout);
            for tok in s.split_whitespace() {
                if let Some(path) = tok.strip_prefix("-I") {
                    let p = PathBuf::from(path);
                    if p.join("Eigen/Geometry").exists() {
                        return p;
                    }
                }
            }
        }
    }
    panic!(
        "eigen-transform-sys: could not locate Eigen headers (tried the vendored \
         references copy at {} and `pkg-config eigen3`). Build must run inside the \
         devbox shell.",
        vendored.display()
    );
}

fn main() {
    let eigen = eigen_include_dir();
    println!("cargo:rerun-if-changed=shim/eigen_transform_shim.cpp");
    println!("cargo:rerun-if-changed=shim/eigen_transform_shim.h");

    let mut shim = cc::Build::new();
    shim.cpp(true)
        .std("c++17")
        .include("shim")
        .include(&eigen)
        .flag_if_supported("-w")
        .define("NDEBUG", None)
        // Force -O3 to match the C++ release build's Eigen f32 codegen (bit-exact).
        .opt_level(3)
        .file("shim/eigen_transform_shim.cpp");
    shim.compile("eigen_transform_shim");
}
