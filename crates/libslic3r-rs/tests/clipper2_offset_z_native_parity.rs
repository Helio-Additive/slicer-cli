//! `src/clipper2/clipper_offset.rs` (clipper.offset.h and clipper.offset.cpp) and
//! `src/clipper2/clipper_h.rs` (clipper.h) of Clipper2 with `USINGZ`,
//! `namespace Clipper2Lib_Z` (`slicer::clipper2::clipper2_z`), against the library itself:
//! `cz2_ref_z_*` of `clipper2-z-sys`, which are the sources the engine pins
//! (`references/BambuStudio/src/clipper2/Clipper2Lib`, version 1.5.2) compiled with `USINGZ`,
//! as `clipper2_z.cpp` of the engine compiles them.
//!
//! As `clipper2_offset_native_parity.rs`, and the z of every point has to be equal too: the
//! z of the input vertex on every point an offset makes from it, what
//! `ClipperOffset::ZCB` gives the intersections of the finishing union, and what the user's
//! callback gives where `ZCB` comes down to it (the number of its calls and a hash of all
//! its arguments are compared). The input z is all zero, one per vertex, one per path, a few
//! small values with zero among them, or anything.
//!
//! The generator, the comparison and the tests are in `clipper2_offset_parity/mod.rs`.

mod clipper2_offset_parity;

use clipper2_offset_parity as common;
use slicer::clipper2::clipper2_z as c2;

const Z: bool = true;

fn mk(q: &[i64; 3]) -> c2::Point64 {
    c2::Point64::new_xyz(q[0], q[1], q[2])
}

fn zof(p: &c2::Point64) -> i64 {
    p.z
}

/// Kind 2: every argument, the z the clipper gave the point before the call, and the number
/// of the call go into the z that comes out (as in `cz2_ref_z_offset`).
fn install_zcb(co: &mut c2::ClipperOffset, kind: i32, state: std::rc::Rc<std::cell::RefCell<(u64, u64)>>) {
    if kind != 2 {
        return;
    }
    co.set_z_callback(c2::ZCallback64::new(move |e1bot: &c2::Point64, e1top: &c2::Point64, e2bot: &c2::Point64, e2top: &c2::Point64, pt: &mut c2::Point64| {
        let mut st = state.borrow_mut();
        st.0 += 1;
        let mut h = common::HASH_START;
        common::mix(&mut h, st.0 as i64);
        for p in [*e1bot, *e1top, *e2bot, *e2top, *pt] {
            common::mix(&mut h, p.x);
            common::mix(&mut h, p.y);
            common::mix(&mut h, p.z);
        }
        pt.z = (h >> 1) as i64;
        common::mix(&mut st.1, pt.z);
    }));
}

offset_parity_tests!();
