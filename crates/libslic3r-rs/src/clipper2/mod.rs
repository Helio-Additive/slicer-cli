//! Clipper2 (`src/clipper2/Clipper2Lib` of the engine, version 1.5.2 as the engine pins it),
//! ported file by file: `clipper_core.rs` is `include/clipper2/clipper.core.h`,
//! `clipper_engine.rs` is `include/clipper2/clipper.engine.h` and `src/clipper.engine.cpp`,
//! `clipper_offset.rs` is `include/clipper2/clipper.offset.h` and `src/clipper.offset.cpp`,
//! `clipper_h.rs` is `include/clipper2/clipper.h`.
//!
//! The engine compiles the library twice: as `namespace Clipper2Lib`, and through
//! `clipper2_z.cpp` with `USINGZ` as `namespace Clipper2Lib_Z`, where a point has a third
//! member `z` that the clipper carries from the input to the output and a callback sets at
//! intersections. There is one body of code here too: this module is `Clipper2Lib`, built
//! with `build_xy.rs`; `clipper2_z` compiles the same files a second time with
//! `build_xyz.rs`. As in the C++, the types of one build are not the types of the other.
//!
//! The tests `clipper2_engine_native_parity`, `clipper2_engine_z_native_parity`,
//! `clipper2_offset_native_parity` and `clipper2_offset_z_native_parity` hold
//! `Clipper64::execute`, `ClipperOffset::execute` and the helpers of clipper.core.h and
//! clipper.h equal, bit for bit, to the C++ built from the same sources: the test-only
//! exports `cz2_ref_xy_*` and `cz2_ref_z_*` of `clipper2-z-sys`, compiled from the engine's
//! pin.
//!
//! Not ported: `ClipperD`, `PolyPathD`, `ReuseableDataContainer64`, the `PathsD` overloads
//! of clipper.h and the templates of clipper.core.h that serve them, clipper.rectclip.cpp,
//! clipper.minkowski.h. Nothing the engine calls reaches them. The engine's callers are
//! `crate::clipper2_utils` and `crate::clipper2_z`.

#[path = "build_xy.rs"]
mod clipper2_build;

pub mod clipper_core;
pub mod clipper_engine;
pub mod clipper_h;
pub mod clipper_offset;

pub mod clipper2_z;

pub use clipper_core::*;
pub use clipper_engine::*;
pub use clipper_h::*;
pub use clipper_offset::*;
