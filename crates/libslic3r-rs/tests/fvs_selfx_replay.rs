// R800 — replay native FVSDUMP3 loop points through rust's EdgeGrid
// has_intersecting_edges (env FVSDUMP_FILE; ignored without it).
use slicer::edge_grid::EdgeGrid;
use slicer::geometry::{BoundingBox, Point, Polyline};

#[test]
fn replay_native_selfx_loops() {
    let path = match std::env::var("FVSDUMP_FILE") {
        Ok(p) => p,
        Err(_) => return,
    };
    std::env::set_var("FVS_SELFX", "1");
    let data = std::fs::read_to_string(&path).unwrap();
    let mut fired = 0usize;
    let mut total = 0usize;
    for ln in data.lines() {
        if !ln.starts_with("FVSDUMP3") {
            continue;
        }
        let closed = ln.contains("closed=1");
        let pts_str = ln.split("pts=").nth(1).unwrap().trim().trim_end_matches(';');
        let pts: Vec<Point> = pts_str
            .split(';')
            .map(|p| {
                let mut it = p.split(',');
                Point::new(
                    it.next().unwrap().parse::<i64>().unwrap(),
                    it.next().unwrap().parse::<i64>().unwrap(),
                )
            })
            .collect();
        let polyline = Polyline::from_points(pts);
        let bbox = BoundingBox::from_points(&polyline.points);
        let mut grid = EdgeGrid::new();
        grid.set_bbox(bbox);
        grid.create_from_polylines_flag(
            std::slice::from_ref(&polyline),
            slicer::scale(10.0),
            !closed,
        );
        total += 1;
        if grid.has_intersecting_edges() {
            fired += 1;
        }
    }
    eprintln!("REPLAY: {fired}/{total} native selfx loops fire in rust");
}

#[test]
fn bowtie_fires() {
    let pts = vec![
        Point::new(0, 0),
        Point::new(100000, 100000),
        Point::new(100000, 0),
        Point::new(0, 100000),
    ];
    let polyline = Polyline::from_points(pts);
    let bbox = BoundingBox::from_points(&polyline.points);
    let mut grid = EdgeGrid::new();
    grid.set_bbox(bbox);
    grid.create_from_polylines_flag(std::slice::from_ref(&polyline), slicer::scale(10.0), true);
    eprintln!("BOWTIE fires={}", grid.has_intersecting_edges());
    assert!(grid.has_intersecting_edges());
}
