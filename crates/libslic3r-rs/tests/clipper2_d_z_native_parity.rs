//! The double-precision half of Clipper2 with `USINGZ`, `namespace Clipper2Lib_Z`
//! (`slicer::clipper2::clipper2_z`), against the library itself: `cz2_ref_z_ext` of
//! `clipper2-z-sys`, which is the sources the engine pins
//! (`references/BambuStudio/src/clipper2/Clipper2Lib`, version 1.5.2) compiled with `USINGZ`,
//! as `clipper2_z.cpp` of the engine compiles them. The z of every point has to be equal too.
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
//! `clipper2_ext_parity/mod.rs`.

mod clipper2_ext_parity;

use clipper2_ext_parity as common;
use slicer::clipper2::clipper2_z as c2;

const Z: bool = true;

fn mk(q: &[i64; 3]) -> c2::Point64 {
    c2::Point64::new_xyz(q[0], q[1], q[2])
}

fn zof(p: &c2::Point64) -> i64 {
    p.z
}

/// Every argument and the number of the call go into the z that comes out (as in op 20 of
/// clipper2_ref_ext_shim.cpp).
#[allow(dead_code)]
fn install_zcb_d(c: &mut c2::ClipperD, state: std::rc::Rc<std::cell::RefCell<(u64, u64)>>) {
    c.set_z_callback(c2::ZCallbackD::new(move |e1bot: &c2::PointD, e1top: &c2::PointD, e2bot: &c2::PointD, e2top: &c2::PointD, pt: &mut c2::PointD| {
        let mut st = state.borrow_mut();
        st.0 += 1;
        let mut h = HASH_START;
        mix(&mut h, st.0 as i64);
        for p in [*e1bot, *e1top, *e2bot, *e2top, *pt] {
            mix(&mut h, p.x.to_bits() as i64);
            mix(&mut h, p.y.to_bits() as i64);
            mix(&mut h, p.z);
        }
        pt.z = (h >> 1) as i64;
        mix(&mut st.1, pt.z);
    }));
}

d_parity_tests!();
