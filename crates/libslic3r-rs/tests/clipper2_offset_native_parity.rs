//! `src/clipper2/clipper_offset.rs` (clipper.offset.h and clipper.offset.cpp) and
//! `src/clipper2/clipper_h.rs` (clipper.h) of Clipper2 without Z, `namespace Clipper2Lib`,
//! against the library itself: `cz2_ref_xy_*` of `clipper2-z-sys`, which are the sources the
//! engine pins (`references/BambuStudio/src/clipper2/Clipper2Lib`, version 1.5.2) compiled
//! without `USINGZ`, as the engine compiles them.
//!
//! `ClipperOffset::Execute` has to give the same answer bit for bit: the same paths in the
//! same order, each from the same start vertex with the same coordinates; with a
//! `PolyTree64` the same nodes in the same pre-order with the same parent, `IsHole()` and
//! polygon. Every case takes one of the 40 combinations of JoinType, EndType and
//! paths-or-tree in turn; the delta (zero, insignificant, small to twice the extent of the
//! input, both signs), the miter limit, the arc tolerance, PreserveCollinear, ReverseSolution
//! and how the paths go in are drawn. The helpers of clipper.h are compared one by one
//! (`path_helpers`, `boolean_helpers`).
//!
//! The generator, the comparison and the tests are in `clipper2_offset_parity/mod.rs`,
//! shared with `clipper2_offset_z_native_parity.rs`.

mod clipper2_offset_parity;

use clipper2_offset_parity as common;
use slicer::clipper2 as c2;

const Z: bool = false;

fn mk(q: &[i64; 3]) -> c2::Point64 {
    c2::Point64::new(q[0], q[1])
}

fn zof(_p: &c2::Point64) -> i64 {
    0
}

/// `SetZCallback` exists under `USINGZ` only.
fn install_zcb(_co: &mut c2::ClipperOffset, _kind: i32, _state: std::rc::Rc<std::cell::RefCell<(u64, u64)>>) {}

offset_parity_tests!();
