// R769: run the mm-seg input prep on the REAL dumped lid-0 slices.
fn main() {
    use slicer::geometry::{ExPolygon, Point, Polygon};
    let path = std::env::args().nth(1).expect("dump path");
    let mut exs: Vec<ExPolygon> = Vec::new();
    for line in std::fs::read_to_string(&path).unwrap().lines() {
        let mut it = line.splitn(4, ' ');
        let tag = it.next().unwrap();
        let idx: usize = it.next().unwrap().parse().unwrap();
        let _sub: usize = it.next().unwrap().parse().unwrap();
        let pts: Vec<Point> = it
            .next()
            .unwrap()
            .split(';')
            .map(|p| {
                let (x, y) = p.split_once(',').unwrap();
                Point::new(x.parse().unwrap(), y.parse().unwrap())
            })
            .collect();
        let poly = Polygon::from_points(pts);
        if tag == "contour" {
            assert_eq!(idx, exs.len());
            exs.push(ExPolygon { contour: poly, holes: vec![] });
        } else {
            exs[idx].holes.push(poly);
        }
    }
    let stat = |tag: &str, exs: &[ExPolygon]| {
        let nh: usize = exs.iter().map(|e| e.holes.len()).sum();
        let a: f64 = exs.iter().map(|e| e.area()).sum();
        println!("{tag}: n={} holes={} area={:.0}", exs.len(), nh, a);
    };
    for (i, e) in exs.iter().enumerate() {
        println!(
            "ex{}: contour {} pts signed={:.0} ccw={}; holes: {:?}",
            i,
            e.contour.points().len(),
            e.contour.signed_area(),
            e.contour.is_counter_clockwise(),
            e.holes
                .iter()
                .map(|h| (h.points().len(), h.signed_area() as i64, h.is_clockwise()))
                .collect::<Vec<_>>()
        );
    }
    stat("input", &exs);

    let grow_mm = (10.0 * slicer::libslic3r::SCALED_EPSILON) / slicer::SCALING_FACTOR;
    let grown = slicer::clipper_utils::offset_expolygons_clib(
        &exs,
        grow_mm,
        slicer::clipper_utils::OffsetJoinType::Miter,
    );
    stat("grown", &grown);
    let mut unioned =
        slicer::clipper_utils::union_ex_clib(&slicer::geometry::to_polygons(&grown), 1);
    stat("unioned", &unioned);
    let min_area = {
        let s = 0.1f64 / slicer::libslic3r::SCALING_FACTOR;
        s * s
    };
    slicer::ex_polygon::remove_small_and_small_holes(&mut unioned, min_area);
    stat("rmsmall", &unioned);
    let shrunk = slicer::clipper_utils::offset_expolygons_clib(
        &unioned,
        -grow_mm,
        slicer::clipper_utils::OffsetJoinType::Miter,
    );
    stat("shrunk", &shrunk);
    let simplified = {
        let tol_mm = 5.0 * slicer::libslic3r::SCALED_EPSILON / slicer::SCALING_FACTOR;
        let mut out: Vec<ExPolygon> = Vec::new();
        for e in &shrunk {
            let rings = e.simplify_p_dp_rings(tol_mm);
            println!("  rings: {:?}", rings.iter().map(|r| (r.points().len(), r.signed_area() as i64)).collect::<Vec<_>>());
            let cleaned = slicer::clipper_utils::simplify_polygons_clib(&rings, 1);
            println!("  cleaned: {:?}", cleaned.iter().map(|r| (r.points().len(), r.signed_area() as i64)).collect::<Vec<_>>());
            let u = slicer::clipper_utils::union_ex_clib(&cleaned, 1);
            println!("  unioned: n={} holes={}", u.len(), u.iter().map(|e| e.holes.len()).sum::<usize>());
            out.extend(u);
        }
        out
    };
    stat("simplified", &simplified);
    let deduped = slicer::mutable_polygon::remove_duplicates_expolygons(
        simplified,
        (0.01f64 / slicer::libslic3r::SCALING_FACTOR) as i64,
        std::f64::consts::PI / 6.0,
    );
    stat("deduped", &deduped);
}
