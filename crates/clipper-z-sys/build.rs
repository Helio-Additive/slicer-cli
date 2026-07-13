// Builds the vendored BambuStudio ClipperLib (non-Z) and ClipperLib_Z (XYZ)
// translation units plus the C ABI shim, into static libraries linked into the
// crate. Portable binary: C++ is statically linked; the only residual runtime
// dependency is the C++ standard library (fine on macOS/Linux).
//
// clipper.cpp is compiled TWICE from the same source file:
//   1. normal      -> namespace ClipperLib       (2D IntPoint)
//   2. -DCLIPPERLIB_USE_XYZ -> namespace ClipperLib_Z (3D IntPoint, Z tags)
// The two TUs live in different C++ namespaces, so their symbols do not collide.
//
// clipper.hpp uses Eigen for IntPoint (Eigen::Matrix<cInt, 2or3, 1>). Eigen is
// header-only; we locate its include dir via `pkg-config eigen3` (always present
// in the devbox shell) with a fallback to the BambuStudio-vendored copy under
// references/. No dynamic/system library dependency is introduced.

use std::path::PathBuf;
use std::process::Command;

fn eigen_include_dir() -> PathBuf {
    // 1. Try pkg-config eigen3 (devbox provides it).
    if let Ok(out) = Command::new("pkg-config")
        .args(["--cflags-only-I", "eigen3"])
        .output()
    {
        if out.status.success() {
            let s = String::from_utf8_lossy(&out.stdout);
            // Output looks like: -I/nix/store/.../include/eigen3
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

    // 2. Fallback: the Eigen vendored inside the BambuStudio references tree.
    //    build.rs cwd is the crate root (crates/clipper-z-sys).
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let vendored = manifest
        .join("../../libslic3r/bambustudio/references/BambuStudio/src/eigen");
    if vendored.join("Eigen/Geometry").exists() {
        return vendored;
    }

    panic!(
        "clipper-z-sys: could not locate Eigen headers (tried `pkg-config eigen3` \
         and the vendored references copy at {}). Build must run inside the devbox \
         shell (devbox run -- cargo build).",
        vendored.display()
    );
}

fn main() {
    let eigen = eigen_include_dir();
    println!("cargo:rerun-if-changed=shim/medial_axis_shim.cpp");
    println!("cargo:rerun-if-changed=vendor/clipper.cpp");
    println!("cargo:rerun-if-changed=vendor/clipper.hpp");
    println!("cargo:rerun-if-changed=vendor/clipper_z.hpp");
    println!("cargo:rerun-if-changed=vendor/Int128.hpp");
    println!("cargo:rerun-if-changed=shim/clipper_z_shim.cpp");
    println!("cargo:rerun-if-changed=shim/clipper_z_shim.h");

    let common = |b: &mut cc::Build| {
        b.cpp(true)
            .std("c++17")
            // R324: match native libslic3r flags (-O3, clang default
            // -ffp-contract=on) so the EFC smooth shim's FMA contraction is
            // bit-identical to the reference binary.
            .opt_level(3)
            .include("vendor")
            .include("shim")
            .include(&eigen)
            // Eigen + the vendored clipper trip a lot of warnings; keep the log quiet.
            .flag_if_supported("-w")
            // NDEBUG kills the original clip_extrusion asserts (matches release builds).
            .define("NDEBUG", None)
            // Wrap the vendored ClipperLib / ClipperLib_Z in a unique outer
            // namespace so its mangled symbols become `ClipperZSys::ClipperLib::…`.
            // WITHOUT this, the int32 (CLIPPERLIB_INT32) ClipperLib here collides
            // at link time with geo-clipper's `clipper-sys` int64 `ClipperLib`
            // (same mangled names, incompatible IntPoint layout) — an ODR
            // violation that corrupts the heap inside `ClipperOffset::DoOffset`
            // and segfaults the bridges wave_seeds path. clipper.hpp/.cpp already
            // support CLIPPERLIB_NAMESPACE_PREFIX (it wraps every namespace block).
            .define("CLIPPERLIB_NAMESPACE_PREFIX", "ClipperZSys");
    };

    // TU 1: non-Z ClipperLib.
    let mut lib = cc::Build::new();
    common(&mut lib);
    lib.file("vendor/clipper.cpp");
    lib.compile("clipper_nonz");

    // TU 2: ClipperLib_Z (XYZ). Same source, different define + object dir so
    // cc does not collide the object files.
    let mut lib_z = cc::Build::new();
    common(&mut lib_z);
    lib_z.define("CLIPPERLIB_USE_XYZ", None);
    lib_z.file("vendor/clipper.cpp");
    lib_z.compile("clipper_z");

    // TU 3: the C ABI shim (includes clipper_z.hpp then clipper.hpp).
    let mut shim = cc::Build::new();
    common(&mut shim);
    shim.file("shim/clipper_z_shim.cpp");
    // R269: medial-axis shim — boost::polygon voronoi from the SAME nix boost
    // (1.87.0, devbox profile) the native binary uses, for vertex-exactness.
    shim.file("shim/medial_axis_shim.cpp");
    if let Ok(out) = std::process::Command::new("pkg-config")
        .args(["--cflags-only-I", "boost"])
        .output()
    {
        let sout = String::from_utf8_lossy(&out.stdout);
        for tok in sout.split_whitespace() {
            if let Some(path) = tok.strip_prefix("-I") {
                shim.include(path);
            }
        }
    }
    shim.compile("clipper_z_shim");
}
