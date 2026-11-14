//! `src/clipper2/clipper2_z.rs` (Clipper2 with `USINGZ`: `clipper_core.rs` and
//! `clipper_engine.rs` compiled a second time, `namespace Clipper2Lib_Z`) against the library
//! itself: `cz2_ref_z_*` of `clipper2-z-sys`, which are the sources the engine pins
//! (`references/BambuStudio/src/clipper2/Clipper2Lib`, version 1.5.2) compiled with `USINGZ`,
//! as the engine's clipper2_z.cpp compiles them.
//!
//! Everything `clipper2_engine_native_parity.rs` holds equal is held equal here too, and with
//! it the z of every point that comes out, how often the Z callback ran and what it was
//! given. The input gets its z in six ways in turn (all zero; every vertex its own; the
//! number of its path, as the engine marks a contour; a drawn number per path; a few small
//! values; anything), and the clipper one of three callbacks: none, the engine's
//! (`Clipper2ZIntersectionVisitor`, Clipper2ZUtils.hpp, as `cz2_intersect_open_z` of
//! clipper2_z_shim.cpp has it: the two source z of an intersection go into a table and the
//! point gets minus the size of the table), and one that hashes the number of the call, its
//! four edge end points and the point it is given (with the z the clipper put there before
//! the call) into the z it returns. `DefaultZ` is drawn.
//!
//! The generator and the comparison are in `clipper2_engine_parity/mod.rs`.

mod clipper2_engine_parity;

use clipper2_engine_parity as common;
use common::{Case, CoreOut, Flat, Stats};
use slicer::clipper2::clipper2_z::{
    area, cross_product, distance_sqr, dot_product, get_closest_point_on_segment, get_segment_intersect_pt, is_collinear,
    mid_point, perpendic_dist_from_line_sqrd, point_in_polygon, products_128_count, products_are_equal, segments_intersect,
    ClipType, Clipper64, FillRule, Path64, Paths64, Point64, PointInPolygonResult, PolyPath64, PolyTree64, ZCallback64,
};
use std::cell::RefCell;
use std::rc::Rc;

const Z: bool = true;

fn to_path(p: &common::Path) -> Path64 {
    p.iter().map(|q| Point64::new_xyz(q[0], q[1], q[2])).collect()
}

fn to_paths(ps: &common::Paths) -> Paths64 {
    ps.iter().map(to_path).collect()
}

fn from_path(p: &Path64) -> common::Path {
    p.iter().map(|q| [q.x, q.y, q.z]).collect()
}

fn walk(node: &PolyPath64, parent: i32, flat: &mut Flat) {
    for i in 0..node.count() {
        let child = node.child(i);
        let me = flat.parents.len() as i32;
        flat.parents.push(parent);
        flat.holes.push(child.is_hole() as i32);
        flat.add(&from_path(child.polygon()));
        walk(child, me, flat);
    }
}

/// What the callbacks keep: the number of calls, the hash, the table of intersections.
#[derive(Default)]
struct CallbackState {
    calls: u64,
    hash: u64,
    intersections: Vec<(i64, i64)>,
}

/// The case through the Rust port, as `cz2_ref_z_execute` (clipper2_ref_shim.cpp) runs it
/// through the library.
fn rust_execute(case: &Case) -> (Flat, Stats) {
    let products_before = products_128_count();
    let mut clipper = Clipper64::new();
    clipper.set_preserve_collinear(case.preserve_collinear);
    clipper.set_reverse_solution(case.reverse_solution);

    clipper.default_z = case.default_z;
    let state = Rc::new(RefCell::new(CallbackState { calls: 0, hash: common::HASH_START, intersections: Vec::new() }));
    if case.z_callback == 1 {
        // Clipper2ZIntersectionVisitor::operator() (Clipper2ZUtils.hpp), as in
        // cz2_intersect_open_z of clipper2_z_shim.cpp, without the assert.
        let st = state.clone();
        clipper.set_z_callback(ZCallback64::new(move |e1bot: &Point64, e1top: &Point64, e2bot: &Point64, e2top: &Point64, pt: &mut Point64| {
            let mut st = st.borrow_mut();
            st.calls += 1;
            let mut srcs = vec![e1bot.z, e1top.z, e2bot.z, e2top.z];
            srcs.sort();
            srcs.dedup();
            if srcs.len() == 1 {
                pt.z = srcs[0];
            } else if srcs.len() == 2 {
                st.intersections.push((srcs[0], srcs[1]));
                pt.z = -(st.intersections.len() as i64);
            }
        }));
    } else if case.z_callback == 2 {
        // Every argument, the z the clipper gave the point before the call, and the number
        // of the call go into the z that comes out.
        let st = state.clone();
        clipper.set_z_callback(ZCallback64::new(move |e1bot: &Point64, e1top: &Point64, e2bot: &Point64, e2top: &Point64, pt: &mut Point64| {
            let mut st = st.borrow_mut();
            st.calls += 1;
            let mut h = common::HASH_START;
            common::mix(&mut h, st.calls as i64);
            for p in [*e1bot, *e1top, *e2bot, *e2top, *pt] {
                common::mix(&mut h, p.x);
                common::mix(&mut h, p.y);
                common::mix(&mut h, p.z);
            }
            pt.z = (h >> 1) as i64;
            common::mix(&mut st.hash, pt.z);
        }));
    }

    let add = |clipper: &mut Clipper64, paths: &common::Paths, kind: i32| {
        let mut one = |ps: &Paths64| match kind {
            0 => clipper.add_subject(ps),
            1 => clipper.add_open_subject(ps),
            _ => clipper.add_clip(ps),
        };
        if case.add_order & 4 != 0 {
            for p in paths {
                one(&vec![to_path(p)]);
            }
        } else {
            one(&to_paths(paths));
        }
    };
    if case.add_order & 1 == 0 {
        add(&mut clipper, &case.subj, 0);
        add(&mut clipper, &case.open, 1);
        add(&mut clipper, &case.clip, 2);
    } else {
        add(&mut clipper, &case.clip, 2);
        add(&mut clipper, &case.open, 1);
        add(&mut clipper, &case.subj, 0);
    }
    if case.add_order & 2 != 0 {
        let (mut dropped_closed, mut dropped_open) = (Paths64::new(), Paths64::new());
        clipper.execute_open(ClipType::Xor, FillRule::EvenOdd, &mut dropped_closed, &mut dropped_open);
    }

    let clip_type = [ClipType::NoClip, ClipType::Intersection, ClipType::Union, ClipType::Difference, ClipType::Xor][case.clip_type as usize];
    let fill_rule = [FillRule::EvenOdd, FillRule::NonZero, FillRule::Positive, FillRule::Negative][case.fill_rule as usize];
    let mut flat = Flat { cb_hash: common::HASH_START, ..Flat::default() };
    let mut open_paths = Paths64::new();
    if case.use_tree {
        let mut tree = PolyTree64::new();
        flat.succeeded = clipper.execute_tree_open(clip_type, fill_rule, &mut tree, &mut open_paths);
        walk(&tree, -1, &mut flat);
        flat.num_closed = flat.parents.len() as i32;
    } else {
        let mut closed = Paths64::new();
        flat.succeeded = clipper.execute_open(clip_type, fill_rule, &mut closed, &mut open_paths);
        for p in &closed {
            flat.add(&from_path(p));
        }
        flat.num_closed = closed.len() as i32;
    }
    for p in &open_paths {
        flat.add(&from_path(p));
    }
    flat.num_open = open_paths.len() as i32;
    {
        let mut st = state.borrow_mut();
        let intersections = std::mem::take(&mut st.intersections);
        for (a, b) in intersections {
            common::mix(&mut st.hash, a);
            common::mix(&mut st.hash, b);
        }
        flat.cb_calls = st.calls;
        flat.cb_hash = st.hash;
    }

    let st = clipper.stats;
    let stats: Stats = [
        st.horz_joins,
        st.horz_join_splits,
        st.split_ops,
        st.split_new_outrecs,
        st.edge_splits,
        st.edge_joins,
        st.intersect_corrections,
        st.path1_inside_path2_midpoint,
        st.split_owners_found,
        st.failures,
        st.collinear_removed,
        st.splits_changed_in_loop,
        st.intersect_list_max,
        st.intersect_reorders,
        st.horizontals,
        products_128_count() - products_before,
    ];
    (flat, stats)
}

/// The helpers of clipper.core.h on four points, as `cz2_ref_z_core` calls them: with z, the
/// intersection point starts as (0, 0, z -7) and takes the z of an end point where it is
/// one, the closest point is a new point (z 0) or the end of a segment without length, the
/// midpoint has z 0, and `==` looks at x and y only.
fn rust_core(p: &[i64; 12]) -> CoreOut {
    let a = Point64::new_xyz(p[0], p[1], p[2]);
    let b = Point64::new_xyz(p[3], p[4], p[5]);
    let c = Point64::new_xyz(p[6], p[7], p[8]);
    let d = Point64::new_xyz(p[9], p[10], p[11]);
    let mut ip = Point64::new_xyz(0, 0, -7);
    let hit = get_segment_intersect_pt(&a, &b, &c, &d, &mut ip);
    let cp = get_closest_point_on_segment(&a, &b, &c);
    let mp = mid_point(&a, &b);
    CoreOut {
        ints: [ip.x, ip.y, ip.z, cp.x, cp.y, cp.z, mp.x, mp.y, mp.z],
        doubles: [cross_product(&a, &b, &c), dot_product(&a, &b, &c), perpendic_dist_from_line_sqrd(&a, &b, &c), distance_sqr(a, b)]
            .map(f64::to_bits),
        bools: [
            hit,
            segments_intersect(&a, &b, &c, &d, false),
            segments_intersect(&a, &b, &c, &d, true),
            is_collinear(&a, &b, &c),
            products_are_equal(a.x, a.y, b.x, b.y),
            a == b,
        ],
    }
}

#[test]
fn reference_is_clipper2_of_the_pin() {
    let (from_pin, version) = common::cpp_source(Z);
    eprintln!("reference: Clipper2 {version}, {}", if from_pin == 1 { "built from references/BambuStudio" } else { "built from the vendored copy" });
    assert_eq!(version, "1.5.2");
}

#[test]
fn core_helpers() {
    const N: u64 = 400_000;
    let mut differ = 0u64;
    let mut first = String::new();
    let mut hits = [0u64; 6];
    let before = products_128_count();
    for index in 0..N {
        let mut s = common::case_seed(201, index);
        let p = common::core_input(index, &mut s);
        let rust = rust_core(&p);
        let cpp = common::cpp_core(&p, Z);
        for (i, b) in cpp.bools.iter().enumerate() {
            hits[i] += *b as u64;
        }
        if rust != cpp {
            differ += 1;
            if first.is_empty() {
                first = format!("input {index} {p:?}:\n  Rust {rust:?}\n  C++  {cpp:?}");
            }
        }
    }
    eprintln!(
        "core_helpers: {N} inputs; true: GetSegmentIntersectPt {} SegmentsIntersect {} inclusive {} IsCollinear {} ProductsAreEqual {} equal points {}; 128-bit products {}",
        hits[0],
        hits[1],
        hits[2],
        hits[3],
        hits[4],
        hits[5],
        products_128_count() - before
    );
    assert!(differ == 0, "{differ} of {N} inputs differ. The first: {first}");
    // Both answers of every predicate were met, and so was the 128-bit multiply.
    for (i, h) in hits.iter().enumerate() {
        assert!(*h > 0 && *h < N, "predicate {i} gave one answer only");
    }
    assert!(products_128_count() - before > 10_000);
}

#[test]
fn point_in_polygon_helper() {
    const N: u64 = 30_000;
    let mut differ = 0u64;
    let mut first = String::new();
    let mut seen = [0u64; 3];
    let mut asked = 0u64;
    for index in 0..N {
        let mut s = common::case_seed(202, index);
        let (poly, pts) = common::point_in_polygon_input(index, &mut s);
        let path = to_path(&poly);
        for (x, y) in pts {
            let rust = match point_in_polygon(&Point64::new(x, y), &path) {
                PointInPolygonResult::IsOn => 0,
                PointInPolygonResult::IsInside => 1,
                PointInPolygonResult::IsOutside => 2,
            };
            let cpp = common::cpp_point_in_polygon(x, y, &poly, Z);
            asked += 1;
            seen[cpp as usize] += 1;
            if rust != cpp {
                differ += 1;
                if first.is_empty() {
                    first = format!("input {index}, point ({x}, {y}) in {poly:?}: Rust {rust}, C++ {cpp}");
                }
            }
        }
    }
    eprintln!("point_in_polygon: {asked} points in {N} polygons; IsOn {} IsInside {} IsOutside {}", seen[0], seen[1], seen[2]);
    assert!(differ == 0, "{differ} of {asked} points differ. The first: {first}");
    assert!(seen.iter().all(|n| *n > 1000));
}

#[test]
fn area_helper() {
    const N: u64 = 100_000;
    let mut differ = 0u64;
    let mut first = String::new();
    for index in 0..N {
        let mut s = common::case_seed(203, index);
        let poly = if index % 2 == 0 {
            common::point_in_polygon_input(index / 2, &mut s).0
        } else {
            // any number of points, odd and even, at any magnitude
            let mag = [10i64, 100_000, 1_000_000_000, 1 << 45, common::MAX_COORD / 2][(index / 2 % 5) as usize];
            (0..common::between(0, 12, &mut s)).map(|_| [common::r(mag, &mut s), common::r(mag, &mut s), 0]).collect()
        };
        let rust = area(&to_path(&poly)).to_bits();
        let cpp = common::cpp_area(&poly, Z);
        if rust != cpp {
            differ += 1;
            if first.is_empty() {
                first = format!("input {index} {poly:?}: Rust {}, C++ {}", f64::from_bits(rust), f64::from_bits(cpp));
            }
        }
    }
    assert!(differ == 0, "{differ} of {N} polygons differ. The first: {first}");
}

/// Every way of giving the input its z met every callback, the callbacks ran, and z came out.
fn z_was_exercised(t: &common::Totals) {
    assert_eq!(t.z_pairs.len(), 18, "not every pair of z assignment and callback was run");
    assert!(t.cb_calls > 10_000, "the Z callback hardly ran");
    assert!(t.nonzero_z_out > 1_000, "hardly any z came out");
}

#[test]
fn random_polygons() {
    let t = common::run_category("random_polygons", 101, 20_000, common::gen_random, Z, rust_execute);
    assert!(t.nonempty > 10_000);
    assert!(t.stat("split_ops") > 0, "no self-intersection was fixed");
    assert!(t.stat("intersect_reorders") > 0);
    z_was_exercised(&t);
}

#[test]
fn huge_coordinates() {
    let t = common::run_category("huge_coordinates", 102, 12_800, common::gen_huge, Z, rust_execute);
    assert!(t.nonempty > 5_000);
    assert!(t.stat("products_128") > 10_000, "the 128-bit multiply was not reached");
}

#[test]
fn coincident_and_collinear() {
    let t = common::run_category("coincident_and_collinear", 103, 16_000, common::gen_coincident, Z, rust_execute);
    assert!(t.stat("collinear_removed") > 10_000);
    assert!(t.stat("edge_joins") > 0 && t.stat("edge_splits") > 0, "no edges were joined and split");
    assert!(t.stat("horz_joins") > 0);
}

#[test]
fn horizontal_edges() {
    let t = common::run_category("horizontal_edges", 104, 12_000, common::gen_horizontal, Z, rust_execute);
    assert!(t.stat("horizontals") > 50_000);
    assert!(t.stat("horz_joins") > 1_000, "no horizontal joins");
    assert!(t.stat("horz_join_splits") > 0, "no horizontal join split a path");
}

#[test]
fn touching_and_overlapping() {
    let t = common::run_category("touching_and_overlapping", 105, 10_000, common::gen_touching, Z, rust_execute);
    assert!(t.stat("edge_joins") > 0);
}

#[test]
fn self_intersecting() {
    let t = common::run_category("self_intersecting", 106, 10_000, common::gen_self_intersecting, Z, rust_execute);
    assert!(t.stat("split_ops") > 100, "no self-intersection was fixed");
    assert!(t.stat("split_new_outrecs") > 0, "no split made a new path");
    assert!(t.stat("intersect_list_max") > 24, "the intersection sort never left its insertion sort");
}

#[test]
fn open_subjects() {
    let t = common::run_category("open_subjects", 107, 14_000, common::gen_open, Z, rust_execute);
    assert!(t.open_paths > 5_000, "no open paths came out");
    z_was_exercised(&t);
}

#[test]
fn nested() {
    let t = common::run_category("nested", 108, 8_000, common::gen_nested, Z, rust_execute);
    assert!(t.tree_holes > 1_000 && t.tree_deepest >= 4, "the trees are flat");
    assert!(t.stat("path1_inside_path2_midpoint") > 0);
}

#[test]
fn dense_intersections() {
    let t = common::run_category("dense_intersections", 109, 3_072, common::gen_dense, Z, rust_execute);
    assert!(t.stat("intersect_list_max") > 128, "the intersection sort never took its ninther");
    assert!(t.stat("intersect_reorders") > 0);
}

#[test]
fn nearly_parallel() {
    let t = common::run_category("nearly_parallel", 110, 6_016, common::gen_nearly_parallel, Z, rust_execute);
    assert!(t.stat("intersect_corrections") > 0, "no intersection point had to be brought back into its scanbeam");
}
