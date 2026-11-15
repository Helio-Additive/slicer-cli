//! `src/clipper2/clipper_rectclip.rs` (clipper.rectclip.h and clipper.rectclip.cpp) of Clipper2 without Z, `namespace Clipper2Lib` (`slicer::clipper2`), against the
//! library itself: `cz2_ref_xy_ext` of `clipper2-z-sys`, which is the sources the engine pins
//! (`references/BambuStudio/src/clipper2/Clipper2Lib`, version 1.5.2) compiled without
//! `USINGZ`, as the engine compiles them.
//!
//! `RectClip64::Execute` and `RectClipLines64::Execute` (clipper.rectclip.cpp) and the
//! `RectClip` / `RectClipLines` wrappers of clipper.h, on `Paths64`, one `Path64`, and on
//! doubles for every precision -8..8 and some outside it (the exception). The answers have to
//! be equal: the same paths in the same order, each from the same start vertex. The paths
//! are made about the rectangle: on its edges and corners, inside, outside, around it, along
//! an edge, across it; the rectangle is small, of one unit, empty, inverted or huge.
//! `GetSegmentIntersection` and `StartLocsAreClockwise` are compared on their own.
//!
//! The generators, the operations on both sides and the tests are in
//! `clipper2_ext_parity/mod.rs`, shared with the `_z` twin of this file.

mod clipper2_ext_parity;

use clipper2_ext_parity as common;
use slicer::clipper2 as c2;

const Z: bool = false;

fn mk(q: &[i64; 3]) -> c2::Point64 {
    c2::Point64::new(q[0], q[1])
}

fn zof(_p: &c2::Point64) -> i64 {
    0
}

/// `SetZCallback` exists under `USINGZ` only.
#[allow(dead_code)]
fn install_zcb_d(_c: &mut c2::ClipperD, _state: std::rc::Rc<std::cell::RefCell<(u64, u64)>>) {}

rectclip_parity_tests!();
