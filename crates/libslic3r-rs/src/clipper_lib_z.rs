//! `namespace ClipperLib_Z`: ClipperLib compiled with `CLIPPERLIB_USE_XYZ`
//! (`src/clipper/clipper_z.cpp` and `clipper_z.hpp` of the engine, which define the macro
//! and include clipper.cpp and clipper.hpp). A point has a third coordinate, Z, which the
//! clipper carries from the input to the output, and `Clipper` takes a callback that sets
//! the Z of a point where two edges intersect.
//!
//! There is one body of code, `clipper.rs`: it is compiled here a second time, as the C++
//! compiles clipper.cpp a second time, with `clipper_build_xyz.rs` in the place of
//! `clipper_build_xy.rs`. `crate::clipper` is `ClipperLib`, this module is `ClipperLib_Z`,
//! and as in the C++ the types of one are not the types of the other.
//! `tests/clipper_z_native_parity.rs` holds it equal to the library.
//!
//! The engine's own routines on top of it are in `clipper_z.rs` and `clipper_z_utils.rs`.

#[path = "clipper_build_xyz.rs"]
mod clipperlib_build;

#[path = "clipper.rs"]
mod clipper;

pub use clipper::*;

impl Clipper {
    /// clipper.hpp:414-417
    /// C++: void ZFillFunction(ZFillCallback zFillFunc) { m_ZFill = zFillFunc; }
    /// set the callback function for z value filling on intersections (otherwise Z is 0)
    pub fn z_fill_function(&mut self, z_fill_func: ZFillCallback) {
        self.m_z_fill = z_fill_func;
    }
}
