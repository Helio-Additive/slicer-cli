//! Port of the pin's `src/glu-libtess` (SGI GLU libtess, as Bambu Studio
//! vendors it), one Rust file per C file. Tesselate.cpp triangulates with it.

pub mod dict;
pub mod geom;
pub mod mesh;
pub mod normal;
pub mod priorityq;
pub mod priorityq_heap;
pub mod render;
pub mod sweep;
pub mod tess;
pub mod tessmono;

pub use tess::{GluTesselator, TessOut};
