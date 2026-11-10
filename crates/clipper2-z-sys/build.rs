// Build the vendored Clipper2 (v1.5.4) WITH `-DUSINGZ` so Point64 carries a `z`
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
}
