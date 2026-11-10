// R772 — A/B the rust boostvoronoi crate vs the boost::polygon exact shim on
// the REAL lid-0 mm-seg input (the TBDUMP file): quantify the vertex ULP drift
// that AWALL/TBPROBE8 measured downstream.
fn main() {
    use slicer::geometry::{Line, Point, Polygon};
    let path = std::env::args().nth(1).expect("dump path");
    let mut rings: Vec<Polygon> = Vec::new();
    for line in std::fs::read_to_string(&path).unwrap().lines() {
        let mut it = line.splitn(4, ' ');
        let _tag = it.next().unwrap();
        let _i = it.next().unwrap();
        let _s = it.next().unwrap();
        let pts: Vec<Point> = it
            .next()
            .unwrap()
            .split(';')
            .map(|p| {
                let (x, y) = p.split_once(',').unwrap();
                Point::new(x.parse().unwrap(), y.parse().unwrap())
            })
            .collect();
        rings.push(Polygon::from_points(pts));
    }
    let mut lines: Vec<Line> = Vec::new();
    let mut segs: Vec<[i32; 4]> = Vec::new();
    for r in &rings {
        let pts = r.points();
        for i in 0..pts.len() {
            let a = pts[i];
            let b = pts[(i + 1) % pts.len()];
            lines.push(Line { a, b });
            segs.push([a.x as i32, a.y as i32, b.x as i32, b.y as i32]);
        }
    }
    println!("segments: {}", segs.len());

    // Backend A: rust boostvoronoi via the crate wrapper.
    let mut vd = slicer::geometry::voronoi_diagram::VoronoiDiagram::new();
    vd.construct_voronoi(&lines, false).expect("bv build");
    let da = vd.diagram();
    let a_verts: Vec<(f64, f64)> = da.vertices().iter().map(|v| (v.x(), v.y())).collect();

    // Backend B: the exact C++ shim.
    let db = boost_voronoi_sys::construct_segments(&segs);

    println!(
        "A(rust bv): v={} e={} c={}   B(shim): v={} e={} c={}",
        a_verts.len(), da.edges().len(), da.cells().len(),
        db.vertices.len(), db.edges.len(), db.cells.len()
    );

    // Match vertices by nearest neighbour on sorted keys; report drift stats.
    let mut a_sorted = a_verts.clone();
    a_sorted.sort_by(|p: &(f64, f64), q: &(f64, f64)| p.partial_cmp(q).unwrap());
    let mut b_sorted = db.vertices.clone();
    b_sorted.sort_by(|p: &(f64, f64), q: &(f64, f64)| p.partial_cmp(q).unwrap());
    if a_sorted.len() == b_sorted.len() {
        let (mut exact, mut ulp, mut round_flips, mut big) = (0u64, 0u64, 0u64, 0u64);
        let mut max_d = 0.0f64;
        for (pa, pb) in a_sorted.iter().zip(b_sorted.iter()) {
            let dx = (pa.0 - pb.0).abs();
            let dy = (pa.1 - pb.1).abs();
            let d = dx.max(dy);
            max_d = max_d.max(d);
            if pa == pb {
                exact += 1;
            } else if d < 1e-6 {
                ulp += 1;
            } else if d < 0.5 {
                big += 1;
            } else {
                big += 1;
            }
            if pa.0.round() != pb.0.round() || pa.1.round() != pb.1.round() {
                round_flips += 1;
            }
        }
        println!(
            "vertex pairs: exact={} ulp(<1e-6)={} larger={} llround-FLIPS={} max|d|={:.6}",
            exact, ulp, big, round_flips, max_d
        );
    } else {
        println!("TOPOLOGY DIFFERS: vertex counts unequal — drift is structural");
    }
}
