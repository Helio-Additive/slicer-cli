// Build the vendored Clipper2 (v1.5.2, the engine's pin) WITH `-DUSINGZ` so Point64 carries a `z`
// field and the Clipper64 / ClipperOffset Z-callback path is compiled in, then
// the C-ABI shim on top. This is the engine BambuStudio's RegionExpansion.cpp
// `wave_seeds` uses (Clipper2Lib_Z).
//
// ODR ISOLATION (critical): `clipper2c-sys` already links Clipper2 WITHOUT Z, so
// its `Clipper2Lib::Point64` has a DIFFERENT layout than the USINGZ one. To avoid a
// silent One-Definition-Rule collision when both are linked into the final binary,
// the vendored copy here is renamed `namespace Clipper2Lib -> Clipper2ZSys` (a
// text substitution applied to vendor/*.{cpp,h}); the two Clipper2 builds therefore
// never share a symbol. Mirrors clipper-z-sys's `CLIPPERLIB_NAMESPACE_PREFIX`.

fn main() {
    println!("cargo:rerun-if-changed=vendor/clipper.engine.cpp");
    println!("cargo:rerun-if-changed=vendor/clipper.offset.cpp");
    println!("cargo:rerun-if-changed=vendor/clipper.rectclip.cpp");
    println!("cargo:rerun-if-changed=shim/clipper2_z_shim.cpp");
    println!("cargo:rerun-if-changed=shim/clipper2_z_shim.h");
    println!("cargo:rerun-if-changed=shim/clipper2_ref_shim.cpp");
    println!("cargo:rerun-if-changed=shim/clipper2_ref_shim.h");
    println!("cargo:rerun-if-changed=vendor/clipper2/clipper.core.h");
    println!("cargo:rerun-if-changed=vendor/clipper2/clipper.engine.h");

    let common = |b: &mut cc::Build| {
        b.cpp(true)
            .std("c++17")
            .include("vendor")
            .include("shim")
            .flag_if_supported("-w")
            .define("NDEBUG", None)
            // Compile the Z path: Point64 gains `z`, ClipperBase/ClipperOffset get
            // the ZCallback64 members and SetZCallback().
            .define("USINGZ", None);
    };

    let mut lib = cc::Build::new();
    common(&mut lib);
    lib.file("vendor/clipper.engine.cpp");
    lib.file("vendor/clipper.offset.cpp");
    lib.file("vendor/clipper.rectclip.cpp");
    lib.compile("clipper2z_core");

    let mut shim = cc::Build::new();
    common(&mut shim);
    shim.file("shim/clipper2_z_shim.cpp");
    shim.compile("clipper2_z_shim");

    // The references of `clipper2_engine_native_parity` and `clipper2_engine_z_native_parity`
    // of `crates/libslic3r-rs` (test-only; `cz2_ref_xy_*` and `cz2_ref_z_*`, see
    // shim/clipper2_ref_shim.cpp): clipper.engine.cpp of the ENGINE'S PIN,
    // `references/BambuStudio/src/clipper2/Clipper2Lib`, compiled twice as the engine does
    // (CMakeLists.txt of that directory: clipper.engine.cpp without `USINGZ`, and
    // clipper2_z.cpp, which defines the macro and includes the same file). The pin names
    // its namespace `Clipper2Lib` without the macro and `Clipper2Lib_Z` with it; each is
    // renamed by a macro of that name, so that neither build shares a symbol with the
    // un-renamed Clipper2 `clipper2c-sys` links or with the `Clipper2ZSys` above.
    //
    // Where the submodule is not checked out the vendored copy stands in. It is the pin's
    // sources with the namespace renamed; the one other difference is an `_M_ARM64EC` guard
    // of an MSVC-only `#if` (clipper.engine.cpp:16-19 of the pin).
    let pin = std::path::Path::new("../../references/BambuStudio/src/clipper2/Clipper2Lib");
    let pin_engine = pin.join("src/clipper.engine.cpp");
    let pin_include = pin.join("include");
    let have_pin = pin_engine.is_file() && pin_include.join("clipper2/clipper.engine.h").is_file();
    println!("cargo:rerun-if-changed={}", pin_engine.display());
    println!("cargo:rerun-if-changed={}", pin_include.join("clipper2/clipper.core.h").display());
    println!("cargo:rerun-if-changed={}", pin_include.join("clipper2/clipper.engine.h").display());
    for (usingz, namespace, lib) in [(true, "Clipper2RefZ", "clipper2_ref_z"), (false, "Clipper2RefXY", "clipper2_ref_xy")] {
        let mut b = cc::Build::new();
        b.cpp(true).std("c++17").include("shim").flag_if_supported("-w").define("NDEBUG", None);
        if usingz {
            b.define("USINGZ", None);
        }
        b.define("CZ2_REF_NAMESPACE", namespace);
        if have_pin {
            b.include(&pin_include);
            b.define(if usingz { "Clipper2Lib_Z" } else { "Clipper2Lib" }, namespace);
            b.define("CZ2_REF_FROM_PIN", "1");
            b.file(&pin_engine);
        } else {
            println!("cargo:warning=clipper2-z-sys: references/BambuStudio is not checked out; the test-only Clipper2 reference is built from the vendored copy");
            b.include("vendor");
            b.define("Clipper2ZSys", namespace);
            b.define("CZ2_REF_FROM_PIN", "0");
            b.file("vendor/clipper.engine.cpp");
        }
        b.file("shim/clipper2_ref_shim.cpp");
        b.compile(lib);
    }
}
