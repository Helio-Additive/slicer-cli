//! The double-precision half of Clipper2 without Z, `namespace Clipper2Lib` (`slicer::clipper2`), against the
//! library itself: `cz2_ref_xy_ext` of `clipper2-z-sys`, which is the sources the engine pins
//! (`references/BambuStudio/src/clipper2/Clipper2Lib`, version 1.5.2) compiled without
//! `USINGZ`, as the engine compiles them.
//!
//! `ClipperD` and `PolyPathD` (clipper.engine.h, clipper.engine.cpp), the `PathsD`
//! interface of clipper.h (`BooleanOp` and its wrappers, `InflatePaths`, `TrimCollinear`,
//! `TranslatePath`, `SimplifyPath`, `RamerDouglasPeucker`, `Ellipse`, `MakePath`, the outline
//! of a PolyTree), clipper.minkowski.h, `ReuseableDataContainer64` with
//! `ClipperBase::AddReuseableData`, and the templates of clipper.core.h on doubles
//! (`ScalePath(s)`, `TransformPath(s)`, `ScaleRect`, `Rect<double>`, `GetBounds`,
//! `StripNearEqual`, `CheckPrecisionRange`, `PointInPolygon`, `Area` and the point
//! functions). Every double of an answer has to be equal bit for bit. The precisions are
//! -8..8 and some outside (the library throws: the number of the exception is compared, and
//! `ErrorCode()` after it).
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

d_parity_tests!();
