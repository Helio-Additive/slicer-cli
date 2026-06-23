//! ASSESS-ONLY (offset-vtx-assess branch): compare offset-output vertex density
//! between the live geo-clipper path (clipper_utils::offset_polygon) and the
//! vendored BambuStudio ClipperLib (clipper-z-sys cz_offset_closed, faithful to
//! libslic3r raw_offset). NOT a production primitive; quantifies whether the
//! offset backend is the source of the inner-wall over-arc vertex divergence.

use slicer::clipper_utils::{offset_polygon, OffsetJoinType};
use slicer::geometry::{Point, Polygon};
use slicer::{scale, unscale, Coord};

/// Build a closed contour from mm-space (x,y) points (scaled to coord space).
fn poly_mm(pts: &[(f64, f64)]) -> Polygon {
    Polygon::from_points(
        pts.iter()
            .map(|&(x, y)| Point::new(scale(x), scale(y)))
            .collect(),
    )
}

/// geo-clipper offset: returns total output vertex count across all loops.
fn geo_offset_vtx(poly: &Polygon, delta_mm: f64, jt: OffsetJoinType) -> (usize, usize) {
    let out = offset_polygon(poly, delta_mm, jt);
    let loops = out.iter().map(|e| 1 + e.holes.len()).sum::<usize>();
    let pts = out
        .iter()
        .map(|e| e.contour.points().len() + e.holes.iter().map(|h| h.points().len()).sum::<usize>())
        .sum::<usize>();
    (loops, pts)
}

/// ClipperLib offset via the shim, faithful to raw_offset. Input is the same
/// libslic3r-scaled (1e5/mm) coords used in production; ClipperLib here is
/// CLIPPERLIB_INT32, and the bed-scale coords fit in i32. delta is scaled too.
fn clib_offset_vtx(poly: &Polygon, delta_mm: f64, jt_code: i32) -> (usize, usize) {
    let xy: Vec<i32> = poly
        .points()
        .iter()
        .flat_map(|p| {
            [
                i32::try_from(p.x).expect("x in i32 range"),
                i32::try_from(p.y).expect("y in i32 range"),
            ]
        })
        .collect();
    let n = poly.points().len() as i32;
    let delta_scaled = delta_mm * slicer::SCALING_FACTOR;
    let raw = unsafe { clipper_z_sys::cz_offset_closed(xy.as_ptr(), n, delta_scaled, jt_code) };
    let loops = raw.num_paths as usize;
    let pts = raw.total_points as usize;
    unsafe { clipper_z_sys::cz_free_zpaths(raw) };
    (loops, pts)
}

/// A many-vertex rounded contour resembling a Benchy hull cross-section:
/// an ellipse sampled at N points (so every vertex is a "corner"), which is the
/// worst case for offset densification differences.
fn ellipse(cx: f64, cy: f64, rx: f64, ry: f64, n: usize) -> Polygon {
    let pts: Vec<(f64, f64)> = (0..n)
        .map(|i| {
            let t = std::f64::consts::TAU * (i as f64) / (n as f64);
            (cx + rx * t.cos(), cy + ry * t.sin())
        })
        .collect();
    poly_mm(&pts)
}

#[test]
fn offset_vertex_density_comparison() {
    let perim_spacing = 0.45_f64; // inner_wall_line_width from the cmp gcode
    let delta = -perim_spacing; // inward (shrink), as inner-wall offset does

    println!("\n=== OFFSET VERTEX DENSITY: geo-clipper vs ClipperLib (miter) ===");
    println!(
        "{:<28} {:>6} {:>8} {:>8} {:>8} {:>8}",
        "contour", "inPts", "geoPts", "clibPts", "geo/in", "clib/in"
    );

    // Inputs spanning the regimes that matter: a smooth high-vertex ellipse
    // (densification worst case), a coarse polygon (few corners), and a
    // rectangle (axis-aligned, exact).
    let cases: Vec<(&str, Polygon)> = vec![
        ("ellipse_60pt", ellipse(50.0, 50.0, 20.0, 12.0, 60)),
        ("ellipse_200pt", ellipse(50.0, 50.0, 20.0, 12.0, 200)),
        ("ellipse_12pt", ellipse(50.0, 50.0, 20.0, 12.0, 12)),
        ("rect_20x10", poly_mm(&[(10.0, 10.0), (30.0, 10.0), (30.0, 20.0), (10.0, 20.0)])),
        (
            "L_shape",
            poly_mm(&[
                (0.0, 0.0),
                (30.0, 0.0),
                (30.0, 10.0),
                (10.0, 10.0),
                (10.0, 30.0),
                (0.0, 30.0),
            ]),
        ),
    ];

    for (name, poly) in &cases {
        let in_pts = poly.points().len();
        let (_gl, gpts) = geo_offset_vtx(poly, delta, OffsetJoinType::Miter);
        let (_cl, cpts) = clib_offset_vtx(poly, delta, 0); // 0 = jtMiter
        println!(
            "{:<28} {:>6} {:>8} {:>8} {:>8.2} {:>8.2}",
            name,
            in_pts,
            gpts,
            cpts,
            gpts as f64 / in_pts as f64,
            cpts as f64 / in_pts as f64,
        );
    }

    // Also show ROUND join (jtRound) for the ellipse — this is where the two
    // backends' arc densification could diverge the most.
    println!("\n=== ROUND join (jtRound) — densification regime ===");
    println!(
        "{:<28} {:>6} {:>8} {:>8}",
        "contour", "inPts", "geoPts", "clibPts"
    );
    let e = ellipse(50.0, 50.0, 20.0, 12.0, 60);
    let (_gl, gpts) = geo_offset_vtx(&e, delta, OffsetJoinType::Round);
    let (_cl, cpts) = clib_offset_vtx(&e, delta, 1); // 1 = jtRound
    println!("{:<28} {:>6} {:>8} {:>8}", "ellipse_60pt", e.points().len(), gpts, cpts);

    // Characterize geo-clipper miter densification vs vertex spacing: sample the
    // same ellipse at increasing point counts and see where geo-clipper starts
    // inserting extra vertices that ClipperLib does not.
    println!("\n=== geo-clipper miter densification vs input spacing (ellipse 20x12mm) ===");
    println!(
        "{:<10} {:>10} {:>6} {:>8} {:>8} {:>8}",
        "inPts", "spacing_mm", "geo", "clib", "geo/in", "clib/in"
    );
    for &n in &[20usize, 40, 80, 120, 160, 200, 300, 500] {
        let e = ellipse(50.0, 50.0, 20.0, 12.0, n);
        // approx vertex spacing = perimeter / n; ellipse perim ~ Ramanujan
        let (a, b) = (20.0_f64, 12.0_f64);
        let h = ((a - b) / (a + b)).powi(2);
        let perim = std::f64::consts::PI * (a + b) * (1.0 + 3.0 * h / (10.0 + (4.0 - 3.0 * h).sqrt()));
        let spacing = perim / n as f64;
        let (_g, gpts) = geo_offset_vtx(&e, delta, OffsetJoinType::Miter);
        let (_c, cpts) = clib_offset_vtx(&e, delta, 0);
        println!(
            "{:<10} {:>10.4} {:>6} {:>8} {:>8.2} {:>8.2}",
            n,
            spacing,
            gpts,
            cpts,
            gpts as f64 / n as f64,
            cpts as f64 / n as f64
        );
    }

    // Sanity: a rectangle miter-offset inward by 0.45mm yields exactly 4 pts on
    // both backends (no densification). This is the floor-parity claim.
    let rect = poly_mm(&[(10.0, 10.0), (30.0, 10.0), (30.0, 20.0), (10.0, 20.0)]);
    let (_g, grect) = geo_offset_vtx(&rect, delta, OffsetJoinType::Miter);
    let (_c, crect) = clib_offset_vtx(&rect, delta, 0);
    println!(
        "\nRECT miter floor: geo={} clib={} (both should be 4)",
        grect, crect
    );

    // Report the first few output coords of the rect for positional comparison.
    let geo_rect = offset_polygon(&rect, delta, OffsetJoinType::Miter);
    if let Some(e0) = geo_rect.first() {
        let cs: Vec<(Coord, Coord)> =
            e0.contour.points().iter().map(|p| (p.x, p.y)).collect();
        println!("geo rect coords (scaled): {:?}", cs);
        println!(
            "geo rect coords (mm): {:?}",
            cs.iter()
                .map(|(x, y)| (unscale(*x), unscale(*y)))
                .collect::<Vec<_>>()
        );
    }
}
