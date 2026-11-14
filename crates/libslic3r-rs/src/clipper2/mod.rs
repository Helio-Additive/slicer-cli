//! Clipper2 (`src/clipper2/Clipper2Lib` of the engine, version 1.5.2 as the engine pins it),
//! ported file by file: `clipper_core.rs` is `include/clipper2/clipper.core.h`,
//! `clipper_engine.rs` is `include/clipper2/clipper.engine.h` and `src/clipper.engine.cpp`.
//!
//! The engine compiles the library twice: as `namespace Clipper2Lib`, and through
//! `clipper2_z.cpp` with `USINGZ` as `namespace Clipper2Lib_Z`, where a point has a third
//! member `z` that the clipper carries from the input to the output and a callback sets at
//! intersections. There is one body of code here too: this module is `Clipper2Lib`, built
//! with `build_xy.rs`; `clipper2_z` compiles the same two files a second time with
//! `build_xyz.rs`. As in the C++, the types of one build are not the types of the other.
//!
//! `tests/clipper2_engine_native_parity.rs` and `tests/clipper2_engine_z_native_parity.rs`
//! hold `Clipper64::execute` and the helpers of clipper.core.h equal, bit for bit, to the
//! C++ built from the same sources: the test-only exports `cz2_ref_xy_*` and `cz2_ref_z_*`
//! of `clipper2-z-sys`, compiled from the engine's pin.
//!
//! Not ported yet: `ClipperD`, `PolyPathD`, `ReuseableDataContainer64` and the `PointD`
//! templates of clipper.core.h, clipper.offset.cpp, clipper.rectclip.cpp. Nothing in the
//! engine calls this module yet.

#[path = "build_xy.rs"]
mod clipper2_build;

pub mod clipper_core;
pub mod clipper_engine;

pub mod clipper2_z;

pub use clipper_core::*;
pub use clipper_engine::*;
