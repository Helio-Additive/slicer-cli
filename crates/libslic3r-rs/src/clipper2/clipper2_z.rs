//! `namespace Clipper2Lib_Z`: Clipper2 compiled with `USINGZ` (`src/clipper2_z.cpp` and
//! `include/clipper2/clipper2_z.hpp` of the engine's Clipper2Lib, which define the macro and
//! include the sources). `clipper_core.rs`, `clipper_engine.rs`, `clipper_offset.rs` and
//! `clipper_h.rs` are compiled here a second time, with `build_xyz.rs` in the place of
//! `build_xy.rs`. `tests/clipper2_engine_z_native_parity.rs` and
//! `tests/clipper2_offset_z_native_parity.rs` hold it equal to the library.

#[path = "build_xyz.rs"]
mod clipper2_build;

#[path = "clipper_core.rs"]
pub mod clipper_core;
#[path = "clipper_engine.rs"]
pub mod clipper_engine;
#[path = "clipper_h.rs"]
pub mod clipper_h;
#[path = "clipper_offset.rs"]
pub mod clipper_offset;

pub use clipper_core::*;
pub use clipper_engine::*;
pub use clipper_h::*;
pub use clipper_offset::*;
