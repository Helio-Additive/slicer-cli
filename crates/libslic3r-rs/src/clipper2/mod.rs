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
//! `clipper_rectclip.rs` is `include/clipper2/clipper.rectclip.h` and
//! `src/clipper.rectclip.cpp`, `clipper_minkowski.rs` is `include/clipper2/clipper.minkowski.h`.
//! `clipper2_rectclip_native_parity`, `clipper2_d_native_parity` and their `_z` twins hold
//! those, `ClipperD`, `ReuseableDataContainer64` and the `PathsD` interface equal to the same
//! reference.
//!
//! Not ported: `include/clipper2/clipper.export.h` (a C interface for other languages; no
//! file of the engine includes it and the engine's build does not compile it) and the
//! stream operators. The engine's callers are `crate::clipper2_utils` and
//! `crate::clipper2_z`; they reach `Clipper64`, `ClipperOffset` and clipper.h on `Path64`.

#[path = "build_xy.rs"]
mod clipper2_build;

pub mod clipper_core;
pub mod clipper_engine;
pub mod clipper_h;
pub mod clipper_minkowski;
pub mod clipper_offset;
pub mod clipper_rectclip;

pub mod clipper2_z;

pub use clipper_core::*;
pub use clipper_engine::*;
pub use clipper_h::*;
pub use clipper_minkowski::*;
pub use clipper_offset::*;
pub use clipper_rectclip::*;
