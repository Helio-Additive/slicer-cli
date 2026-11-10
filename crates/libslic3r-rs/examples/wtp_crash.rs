// R775 — standalone repro of the ST missing-twin panic (AWINPANIC capture).
// Usage: wtp_crash <awinpanic.txt>
fn main() {
    use slicer::arachne::wall_tool_paths::{WallToolPaths, WallToolPathsParams};
    use slicer::geometry::{Point, Polygon};
    let path = std::env::args().nth(1).expect("awinpanic file");
    let line = std::fs::read_to_string(&path).unwrap();
    let line = line.trim().strip_prefix("AWINPANIC ").unwrap();
    let mut toks = line.split(' ');
    let mut bw0 = 0i64;
    let mut bwx = 0i64;
    let mut ic = 0usize;
    let mut w0i = 0i64;
    let mut lh = 0.0f64;
    let mut outline: Vec<Polygon> = Vec::new();
    for t in toks.by_ref() {
        if let Some(v) = t.strip_prefix("bw0=") {
            bw0 = v.parse().unwrap();
        } else if let Some(v) = t.strip_prefix("bwx=") {
            bwx = v.parse().unwrap();
        } else if let Some(v) = t.strip_prefix("ic=") {
            ic = v.parse().unwrap();
        } else if let Some(v) = t.strip_prefix("w0i=") {
            w0i = v.parse().unwrap();
        } else if let Some(v) = t.strip_prefix("lh=") {
            lh = v.parse().unwrap();
        } else {
            let pts: Vec<Point> = t
                .split(';')
                .map(|p| {
                    let (x, y) = p.split_once(',').unwrap();
                    Point::new(x.parse().unwrap(), y.parse().unwrap())
                })
                .collect();
            outline.push(Polygon::from_points(pts));
        }
    }
    println!("outline: {} polys, ic={} bw0={} lh={}", outline.len(), ic, bw0, lh);
    let params = WallToolPathsParams {
        min_bead_width: 0.34,
        min_feature_size: 0.1,
        wall_transition_length: 0.4,
        wall_transition_angle: 10.0,
        wall_transition_filter_deviation: 0.1,
        wall_distribution_count: 1,
    };
    let mut wtp = WallToolPaths::new(outline, bw0, bwx, ic, w0i, lh, params);
    let paths = wtp.generate();
    println!("ok: {} wall sets", paths.len());
}
