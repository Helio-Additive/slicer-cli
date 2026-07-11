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

// The Eigen the NATIVE binary compiles against (nix eigen3 via pkg-config —
// build.ninja shows `-isystem .devbox/nix/profile/default/include/eigen3` FIRST,
// i.e. 3.4.0, NOT the vendored 3.3.7). The ShortEdgeCollapse kernels must use
// this one; 3.3.7-vs-3.4.0 f32 normalized()/norm codegen differs by ulps (R188).
fn native_eigen_include_dir() -> PathBuf {
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
        "eigen-transform-sys: could not locate the nix Eigen headers via \
         `pkg-config eigen3`. Build must run inside the devbox shell."
    );
}

fn main() {
    let eigen = eigen_include_dir();
    println!("cargo:rerun-if-changed=shim/eigen_transform_shim.cpp");
    println!("cargo:rerun-if-changed=shim/eigen_transform_shim.h");
    println!("cargo:rerun-if-changed=shim/secol_shim.cpp");

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

    // Separate TU + separate Eigen: the transform shims above are byte-locked
    // against the vendored 3.3.7; the collapse kernels must match the native
    // binary's 3.4.0 (see native_eigen_include_dir).
    let mut secol = cc::Build::new();
    secol
        .cpp(true)
        .std("c++17")
        .include(&native_eigen_include_dir())
        .flag_if_supported("-w")
        .define("NDEBUG", None)
        .opt_level(3)
        .file("shim/secol_shim.cpp");
    secol.compile("secol_shim");
}
