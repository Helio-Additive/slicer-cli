//! `namespace Clipper2Lib_Z`: Clipper2 compiled with `USINGZ` (`src/clipper2_z.cpp` and
//! `include/clipper2/clipper2_z.hpp` of the engine's Clipper2Lib, which define the macro and
//! include the sources). `clipper_core.rs` and `clipper_engine.rs` are compiled here a second
//! time, with `build_xyz.rs` in the place of `build_xy.rs`.
//! `tests/clipper2_engine_z_native_parity.rs` holds it equal to the library.

#[path = "build_xyz.rs"]
mod clipper2_build;

#[path = "clipper_core.rs"]
pub mod clipper_core;
#[path = "clipper_engine.rs"]
pub mod clipper_engine;

pub use clipper_core::*;
pub use clipper_engine::*;
