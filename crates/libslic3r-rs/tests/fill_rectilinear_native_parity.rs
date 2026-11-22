//! Native parity of the Fill/FillRectilinear.cpp port (`slicer::fill::fill_rectilinear`) against vectors made
//! by the C++ at the pin (tests/data/fill_rectilinear/gen.cpp).
//!
//! Groups: "fs" (`fill_surface` of FillRectilinear, FillMonotonic, FillMonotonicLine,
//! FillAlignedRectilinear, FillGrid, FillTriangles, FillStars, FillCubic, FillSupportBase, FillZigZag and
//! FillCrossZag: the spacing the filler is left with and the polylines in order), "sl" (the stages of
//! `fill_surface_by_lines` one at a time: the offset contours, the vertical lines, their links, the
//! polylines of the graph traversal, the pinched lines, the monotonic regions, their chain and its
//! polylines), "mfl" (`make_fill_lines`), "sgp" (`sample_grid_pattern`) and "wg"
//! (`FillMonotonicLineWGapFill::fill_surface_by_lines`, also of FillGlobalMonotonicLine). The lines of the vectors that
//! start with "in" are the inputs, which this test reads back; every other line it has to reproduce bit for
//! bit.

mod support_shared;

use slicer::fill::fill_base::Fill;
use slicer::fill::fill_rectilinear::{
    connect_segment_intersections_by_contours, make_fill_lines, monotonic_stages, sample_grid_pattern, slice_region_by_vertical_lines,
    traverse_graph_generate_polylines, ExPolygonWithOffset, FillAlignedRectilinear, FillCrossZag, FillCubic, FillGrid, FillMonotonic,
    FillGlobalMonotonicLine, FillMonotonicLine, FillMonotonicLineWGapFill, FillRectilinear, FillRectilinearParams, FillStars, FillSupportBase, FillTriangles, FillZigZag, SegmentedIntersectionLine,
};
use slicer::fill::FillParams;
use slicer::geometry::{BoundingBox, Point, Polyline};
use slicer::print_config::InfillPattern;
use slicer::surface::{Surface, SurfaceType};
use support_shared::*;

/// gen.cpp `PLS`
fn pls_s(ps: &[Polyline]) -> String {
    let mut o = format!("{}", ps.len());
    for p in ps {
        o += &format!(" {}", p.points.len());
        for q in &p.points {
            o += &format!(" {}", pt(q));
        }
    }
    o
}

const TYPE_NAMES: [&str; 11] =
    ["FillRectilinear", "FillMonotonic", "FillMonotonicLine", "FillAlignedRectilinear", "FillGrid", "FillTriangles", "FillStars", "FillCubic", "FillSupportBase", "FillZigZag", "FillCrossZag"];

fn run_fs(r: &mut Run) {
    while let Some(line) = r.input("in fs") {
        let mut t = Tok::new(&line);
        let ty = t.i();
        let layer_id = t.i() as usize;
        let z = t.d();
        let spacing = t.d();
        let overlap = t.d();
        let angle_deg = t.d();
        let link_max_length = t.i();
        let (bmin, bmax) = (t.pt(), t.pt());
        let bridge_angle = t.d();
        let thickness_layers = t.i() as usize;
        let mut params = FillParams::new();
        params.density = t.f() as f64;
        params.anchor_length = t.f() as f64;
        params.anchor_length_max = t.f() as f64;
        params.multiline = t.i() as i32;
        params.dont_adjust = t.b();
        params.pattern = match ty {
            0 => InfillPattern::Rectilinear,
            1 => InfillPattern::Monotonic,
            2 => InfillPattern::MonotonicLine,
            3 => InfillPattern::AlignedRectilinear,
            4 => InfillPattern::Grid,
            5 => InfillPattern::Triangles,
            6 => InfillPattern::Stars,
            7 => InfillPattern::Cubic,
            8 => InfillPattern::SupportBase,
            9 => InfillPattern::ZigZag,
            _ => InfillPattern::CrossZag,
        };
        let mut surface = Surface::new(SurfaceType::Internal, t.expoly());
        // Surface.hpp:41: negative means undefined.
        surface.bridge_angle = if bridge_angle >= 0. { Some(bridge_angle) } else { None };
        surface.thickness_layers = thickness_layers;
        let setup = |f: &mut Fill| {
            f.layer_id = layer_id;
            f.z = z;
            f.spacing = spacing;
            f.overlap = overlap;
            // float(Geometry::deg2rad(angle_deg))
            f.angle = (std::f64::consts::PI * angle_deg / 180.0) as f32;
            f.link_max_length = link_max_length;
            // BoundingBox(Point(0, 0), Point(-1, -1)) is the undefined box of the constructor.
            if !(bmin == Point::new(0, 0) && bmax == Point::new(-1, -1)) {
                f.set_bounding_box(&BoundingBox::from_points_minmax(bmin, bmax));
            }
        };
        macro_rules! run {
            ($filler:ident, $base:ident) => {{
                let mut f = <$filler>::new();
                {
                    let $base = &mut f;
                    setup(base_of!($filler, $base));
                }
                let result = f.fill_surface(&surface, &params);
                let $base = &mut f;
                (base_of!($filler, $base).spacing, result)
            }};
        }
        macro_rules! base_of {
            (FillRectilinear, $f:ident) => {
                &mut $f.base
            };
            ($other:ident, $f:ident) => {
                &mut $f.0.base
            };
        }
        let (spacing_after, result) = match ty {
            0 => run!(FillRectilinear, f),
            1 => run!(FillMonotonic, f),
            2 => run!(FillMonotonicLine, f),
            3 => run!(FillAlignedRectilinear, f),
            4 => run!(FillGrid, f),
            5 => run!(FillTriangles, f),
            6 => run!(FillStars, f),
            7 => run!(FillCubic, f),
            8 => run!(FillSupportBase, f),
            9 => run!(FillZigZag, f),
            _ => run!(FillCrossZag, f),
        };
        r.out(format!("fs {} {}", hd(spacing_after), pls_s(&result)));
    }
}

/// gen.cpp `SEGS`
fn segs_s(segs: &[SegmentedIntersectionLine]) -> String {
    let mut o = format!("{}", segs.len());
    for il in segs {
        o += &format!(" {} {} {}", il.idx, il.pos, il.intersections.len());
        for it in &il.intersections {
            o += &format!(
                " {} {} {} {} {} {} {} {} {} {} {} {}",
                it.i_contour as i64,
                it.i_segment as i64,
                it.pos_p,
                it.pos_q,
                it.itype as i32,
                it.prev_on_contour,
                it.prev_on_contour_type as i32,
                it.prev_on_contour_quality as i32,
                it.next_on_contour,
                it.next_on_contour_type as i32,
                it.next_on_contour_quality as i32,
                it.prev_idx as i64
            );
        }
    }
    o
}

fn run_sl(r: &mut Run) {
    while let Some(line) = r.input("in sl") {
        let mut t = Tok::new(&line);
        let angle = t.f();
        let aoffset1 = t.i();
        let aoffset2 = t.i();
        let line_spacing = t.i();
        let full = t.b();
        let anchor_length_max = t.f();
        let link_max_length = t.i();
        let expolygon = t.expoly();
        let poly_with_offset = ExPolygonWithOffset::new_pin(&expolygon, -angle, aoffset1, aoffset2);
        let mut o = format!("pwo {} {} {}", poly_with_offset.n_contours_outer, poly_with_offset.n_contours_inner, poly_with_offset.n_contours);
        for p in &poly_with_offset.polygons_outer {
            o += &format!(" {}", poly_s(p));
        }
        for p in &poly_with_offset.polygons_inner {
            o += &format!(" {}", poly_s(p));
        }
        for k in 0..poly_with_offset.n_contours {
            o += &format!(" {}", hb(poly_with_offset.is_contour_ccw(k)));
        }
        r.out(o);
        if poly_with_offset.n_contours == 0 {
            continue;
        }
        let bounding_box = poly_with_offset.bounding_box_src_pin();
        r.out(format!("bbs {} {}", pt(&bounding_box.min), pt(&bounding_box.max)));
        let n_vlines = ((bounding_box.max.x - bounding_box.min.x + line_spacing - 1) / line_spacing) as usize;
        let mut x0 = bounding_box.min.x;
        if full {
            x0 += (line_spacing + 10) / 2;
        }
        let mut segs = slice_region_by_vertical_lines(&poly_with_offset, n_vlines, x0, line_spacing);
        r.out(format!("sl {}", segs_s(&segs)));
        if aoffset2 == 0 {
            continue;
        }
        let mut params = FillRectilinearParams {
            density: if full { 1.0 } else { 0.3f32 as f64 },
            monotonic: false,
            dont_connect: anchor_length_max < 0.05,
            link_max_length,
            full_infill: full,
            dont_adjust: true,
            consistent_pattern: false,
        };
        connect_segment_intersections_by_contours(&poly_with_offset, &mut segs, &params, link_max_length);
        r.out(format!("csi {}", segs_s(&segs)));
        {
            // The flag of the traversal is in the vectors: "tg <flag> ...".
            let expected_flag = r.expected.get(r.lines.len()).map(|l| l.starts_with("tg 1")).unwrap_or(false);
            params.consistent_pattern = expected_flag;
            let mut segs2 = segs.clone();
            let mut pls = Vec::new();
            traverse_graph_generate_polylines(&poly_with_offset, &params, &mut segs2, &mut pls);
            r.out(format!("tg {} {}", hb(expected_flag), pls_s(&pls)));
            params.consistent_pattern = false;
        }
        {
            let st = monotonic_stages(&poly_with_offset, &segs);
            r.out(format!("pinch {}", segs_s(&st.segs)));
            let mut o = format!("mr {}", st.regions.len());
            for (b, len1, len2, flips, left, right) in &st.regions {
                o += &format!(" {} {} {} {} {} {} {} {} {} {}", b[0], b[1], b[2], b[3], b[4], b[5], hf(*len1), hf(*len2), hb(*flips), left.len());
                for nb in left {
                    o += &format!(" {nb}");
                }
                o += &format!(" {}", right.len());
                for nb in right {
                    o += &format!(" {nb}");
                }
            }
            r.out(o);
            if !st.regions.is_empty() {
                let mut o = format!("chain {}", st.chain.len());
                for (region, flipped) in &st.chain {
                    o += &format!(" {} {}", region, hb(*flipped));
                }
                r.out(o);
                r.out(format!("pfp {}", pls_s(&st.polylines)));
            }
        }
    }
}

fn run_mfl(r: &mut Run) {
    while let Some(line) = r.input("in mfl") {
        let mut t = Tok::new(&line);
        let angle = t.d();
        let aoffset1 = t.i();
        let line_spacing = t.i();
        let x_margin = t.i();
        let pattern_shift = t.i();
        let refpt = t.pt();
        let expolygon = t.expoly();
        let base = ExPolygonWithOffset::new_pin(&expolygon, 0., aoffset1, 0);
        let mut fill_lines = Vec::new();
        if base.n_contours > 0 {
            make_fill_lines(&ExPolygonWithOffset::new_rotated(&base, (-angle) as f32), refpt, angle, x_margin, line_spacing, pattern_shift, &mut fill_lines);
        }
        r.out(format!("mfl {}", pls_s(&fill_lines)));
    }
}

fn run_sgp(r: &mut Run) {
    while let Some(line) = r.input("in sgp") {
        let mut t = Tok::new(&line);
        let spacing = t.i();
        let (min, max) = (t.pt(), t.pt());
        let expolygon = t.expoly();
        let pts = sample_grid_pattern(&expolygon, spacing, &BoundingBox::from_points_minmax(min, max));
        let mut o = format!("sgp {}", pts.len());
        for q in &pts {
            o += &format!(" {}", pt(q));
        }
        r.out(o);
    }
}

fn run_wg(r: &mut Run) {
    while let Some(line) = r.input("in wg") {
        let mut t = Tok::new(&line);
        let global = t.b();
        let mut f = if global { FillGlobalMonotonicLine::new().0 } else { FillMonotonicLineWGapFill::new() };
        f.base.layer_id = t.i() as usize;
        f.base.spacing = t.d();
        f.base.overlap = 0.;
        let angle_deg = t.d();
        f.base.angle = (std::f64::consts::PI * angle_deg / 180.0) as f32;
        f.base.link_max_length = 0;
        let (bmin, bmax) = (t.pt(), t.pt());
        f.base.set_bounding_box(&BoundingBox::from_points_minmax(bmin, bmax));
        let bridge_angle = t.d();
        let mut params = FillParams::new();
        params.density = 1.0;
        params.monotonic = true;
        params.anchor_length_max = 0.0;
        params.dont_adjust = t.b();
        let flow_spacing = t.f();
        let mut surface = Surface::new(SurfaceType::Top, t.expoly());
        surface.bridge_angle = if bridge_angle >= 0. { Some(bridge_angle) } else { None };
        let mut result = vec![Polyline::from_points(vec![Point::new(7, 8), Point::new(9, 10)])];
        f.fill_surface_by_lines_flow_spacing(&surface, &params, flow_spacing, &mut result);
        r.out(format!("wg {} {}", hd(f.base.spacing), pls_s(&result)));
    }
}

#[test]
fn fill_rectilinear_native_parity() {
    let mut r = Run::load("fill_rectilinear");
    run_fs(&mut r);
    run_sl(&mut r);
    run_mfl(&mut r);
    run_sgp(&mut r);
    run_wg(&mut r);
    let n = r.expected.iter().filter(|l| !l.starts_with("in ")).count();
    // "fs" by filler.
    let mut per_type = [(0usize, 0usize); 11];
    for (i, e) in r.expected.iter().enumerate() {
        if let Some(rest) = e.strip_prefix("in fs ") {
            let ty: usize = rest.split_whitespace().next().unwrap().parse().unwrap();
            per_type[ty].0 += 1;
            if r.lines.get(i + 1) == r.expected.get(i + 1) {
                per_type[ty].1 += 1;
            }
        }
    }
    for (ty, (total, ok)) in per_type.iter().enumerate() {
        eprintln!("fill_rectilinear: fs {}: {ok} of {total}", TYPE_NAMES[ty]);
    }
    let mut not_equal: Vec<String> = Vec::new();
    for tag in ["fs ", "pwo ", "bbs ", "sl ", "csi ", "tg ", "pinch ", "mr ", "chain ", "pfp ", "mfl ", "sgp ", "wg "] {
        let (total, ok) = r.tally(tag);
        eprintln!("fill_rectilinear: {tag}: {ok} of {total}");
        if ok != total {
            not_equal.push(format!("{tag}{ok}/{total}"));
        }
    }
    // FILL_RECTILINEAR_SHOW=<tag>: the first lines of that tag that differ, token by token.
    if let Ok(tag) = std::env::var("FILL_RECTILINEAR_SHOW") {
        let mut shown = 0;
        for (i, e) in r.expected.iter().enumerate() {
            if e.starts_with(&format!("{tag} ")) && r.lines.get(i) != Some(e) && shown < 6 {
                shown += 1;
                let got = r.lines.get(i).cloned().unwrap_or_default();
                let (et, gt): (Vec<&str>, Vec<&str>) = (e.split(' ').collect(), got.split(' ').collect());
                let first = et.iter().zip(gt.iter()).position(|(a, b)| a != b).unwrap_or(et.len().min(gt.len()));
                let lo = first.saturating_sub(12);
                eprintln!(
                    "line {} ({} tokens, got {}), first difference at token {first}:\n  expected ...{}\n  got      ...{}",
                    i + 1,
                    et.len(),
                    gt.len(),
                    et[lo..et.len().min(first + 14)].join(" "),
                    gt[lo..gt.len().min(first + 14)].join(" ")
                );
            }
        }
    }
    let bad = r.finish();
    eprintln!("fill_rectilinear: {} result lines, {} differ", n, bad);
    assert_eq!(bad, 0, "not equal: {not_equal:?}");
}
