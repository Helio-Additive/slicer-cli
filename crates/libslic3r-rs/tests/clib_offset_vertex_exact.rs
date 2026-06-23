//! M1 verification: the ClipperLib-backed perimeter offset
//! (`clipper_utils::offset_expolygons_clib`) is VERTEX-EXACT relative to the
//! input contour, whereas the geo-clipper offset densifies dense (sub-|delta|)
//! contours. This is the inner-wall density fix (see /tmp/perimoffset_findings.md
//! and /tmp/offsetvtx_findings.md). The swap is area-invariant: both backends
//! must agree on the offset area.

use slicer::clipper_utils::{offset_expolygons, offset_expolygons_clib, OffsetJoinType};
use slicer::geometry::{ExPolygon, ExPolygons, Point, Polygon};
use slicer::{scale, CoordF};

/// Build a closed ellipse contour sampled at `n` points (mm-space, scaled).
fn ellipse_mm(cx: f64, cy: f64, rx: f64, ry: f64, n: usize) -> ExPolygon {
    let pts: Vec<Point> = (0..n)
        .map(|i| {
            let t = std::f64::consts::TAU * (i as f64) / (n as f64);
            Point::new(scale(cx + rx * t.cos()), scale(cy + ry * t.sin()))
        })
        .collect();
    Polygon::from_points(pts).into()
}

fn vtx_count(xs: &ExPolygons) -> usize {
    xs.iter()
        .map(|p| {
            p.contour.points().len() + p.holes.iter().map(|h| h.points().len()).sum::<usize>()
        })
        .sum()
}

#[test]
fn clib_offset_is_vertex_exact_geo_densifies() {
    // 200-pt ellipse (vertex spacing ~0.51mm) shrunk by 0.45mm (inner-wall
    // perimeter spacing). This is exactly the regime where geo-clipper inserts
    // extra miter vertices and ClipperLib does not.
    let n = 200usize;
    let e = ellipse_mm(50.0, 50.0, 20.0, 12.0, n);
    let delta = -0.45_f64;

    let geo = offset_expolygons(&[e.clone()], delta, OffsetJoinType::Miter);
    let clib = offset_expolygons_clib(&[e.clone()], delta, OffsetJoinType::Miter);

    assert!(!geo.is_empty() && !clib.is_empty(), "both offsets nonempty");

    let geo_pts = vtx_count(&geo);
    let clib_pts = vtx_count(&clib);
    println!("ellipse_200pt shrink -0.45mm: input={n} geo={geo_pts} clib={clib_pts}");

    // ClipperLib preserves the input vertex count (vertex-exact).
    assert!(
        clib_pts <= n + 2,
        "ClipperLib offset must be ~vertex-exact (<= {n} input pts), got {clib_pts}"
    );
    // geo-clipper over-emits on this dense contour.
    assert!(
        clib_pts < geo_pts,
        "ClipperLib ({clib_pts}) must emit fewer vertices than geo-clipper ({geo_pts})"
    );

    // Area parity: area-invariant swap (material must stay at parity).
    let geo_area: CoordF = geo.iter().map(|p| p.area().abs()).sum();
    let clib_area: CoordF = clib.iter().map(|p| p.area().abs()).sum();
    let rel = (geo_area - clib_area).abs() / geo_area.max(1.0);
    println!("areas: geo={geo_area} clib={clib_area} rel={rel}");
    assert!(rel < 0.005, "offset areas must agree (area-invariant): rel={rel}");
}

#[test]
fn clib_offset_coarse_contour_matches_geo() {
    // On a coarse contour (edges >> |delta|) both backends are vertex-exact and
    // agree exactly. A 12-pt ellipse shrunk by 0.45mm.
    let e = ellipse_mm(50.0, 50.0, 20.0, 12.0, 12);
    let delta = -0.45_f64;

    let geo = offset_expolygons(&[e.clone()], delta, OffsetJoinType::Miter);
    let clib = offset_expolygons_clib(&[e.clone()], delta, OffsetJoinType::Miter);

    assert!(!geo.is_empty() && !clib.is_empty());
    let geo_pts = vtx_count(&geo);
    let clib_pts = vtx_count(&clib);
    println!("ellipse_12pt shrink -0.45mm: geo={geo_pts} clib={clib_pts}");
    // Coarse contour: ClipperLib at most the geo count (typically equal).
    assert!(clib_pts <= geo_pts + 1);
}
