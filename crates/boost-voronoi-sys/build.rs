// Builds the boost::polygon Voronoi C ABI shim. CRITICAL: uses the SAME boost
// headers the native engine compiles against (nix boost, see the CMake cache)
// and FORCES -O3, so the f64 vertex constructions round identically to the C++
// slicer (the whole point — the rust `boostvoronoi` port drifts by ULPs, which
// welds/gaps piece boundaries and shifts every arachne junction).

use std::path::PathBuf;

fn boost_include_dir() -> PathBuf {
    // 1. Explicit override.
    if let Ok(p) = std::env::var("BOOST_INCLUDE_DIR") {
        let p = PathBuf::from(p);
        if p.join("boost/polygon/voronoi.hpp").exists() {
            return p;
        }
    }
    let manifest = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    // 2. The Boost the NATIVE binary compiled against (CMake cache of the bambu build).
    let cache = manifest.join("../../libslic3r/bambustudio/build/CMakeCache.txt");
    if let Ok(s) = std::fs::read_to_string(&cache) {
        for line in s.lines() {
            if let Some(rest) = line.strip_prefix("Boost_INCLUDE_DIR:PATH=") {
                let p = PathBuf::from(rest.trim());
                if p.join("boost/polygon/voronoi.hpp").exists() {
                    return p;
                }
            }
        }
    }
    // 3. Common fallbacks.
    for cand in ["/usr/local/include", "/usr/include", "/opt/homebrew/include"] {
        let p = PathBuf::from(cand);
        if p.join("boost/polygon/voronoi.hpp").exists() {
            return p;
        }
    }
    panic!(
        "boost-voronoi-sys: could not locate boost/polygon/voronoi.hpp (tried \
         $BOOST_INCLUDE_DIR, the bambu CMake cache, and system prefixes). Build \
         inside the devbox shell after `devbox run bambu:build` has configured CMake."
    );
}

fn main() {
    let boost = boost_include_dir();
    println!("cargo:rerun-if-changed=shim/boost_voronoi_shim.cpp");
    println!("cargo:rerun-if-env-changed=BOOST_INCLUDE_DIR");
    let mut build = cc::Build::new();
    build
        .cpp(true)
        .file("shim/boost_voronoi_shim.cpp")
        .include(&boost)
        .flag_if_supported("-std=c++17")
        // Match the native -O3 -DNDEBUG codegen regardless of cargo profile.
        .opt_level(3)
        .define("NDEBUG", None)
        .flag_if_supported("-fno-fast-math")
        .warnings(false);
    build.compile("boost_voronoi_shim");
    // C++ standard library (cc links it for .cpp(true), keep explicit for clarity).
    let target = std::env::var("TARGET").unwrap_or_default();
    if target.contains("apple") {
        println!("cargo:rustc-link-lib=c++");
    } else {
        println!("cargo:rustc-link-lib=stdc++");
    }
}
