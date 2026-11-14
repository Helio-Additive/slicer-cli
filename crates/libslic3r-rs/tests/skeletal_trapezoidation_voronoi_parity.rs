//! Arachne's graph construction (`SkeletalTrapezoidation::construct_from_polygons`,
//! SkeletalTrapezoidation.cpp:391-504) over `Geometry::VoronoiDiagram` on the port of
//! boost::polygon's builder (`geometry::voronoi_diagram::boost_polygon::VoronoiDiagram`),
//! against `boost::polygon::construct_voronoi` itself, compiled in `boost-voronoi-sys`:
//!
//! - the diagram the walk consumes is boost's, element for element and bit for bit;
//! - the half-edge graph built from it is the same on every run, and the same as the one
//!   built from the `boostvoronoi` crate's diagram (`ARACHNE_VORONOI_PORT=0`), which is why
//!   the gate does not move the fixtures' G-code;
//! - the repair by rotation (Voronoi.cpp:276-361) gives boost's diagram of the rotated
//!   input with every vertex rotated back or put on the input endpoint it stands for.

use slicer::arachne::beading_strategy::BeadingStrategyFactory;
use slicer::arachne::skeletal_trapezoidation::SkeletalTrapezoidation;
use slicer::arachne::skeletal_trapezoidation_graph::NodePtr;
use slicer::geometry::voronoi_diagram::boost_polygon::VoronoiDiagram;
use slicer::geometry::voronoi_diagram::{IssueType, RepairState};
use slicer::geometry::{Line, Point, Polygon, Polygons};

struct Rng(u64);

impl Rng {
    fn for_case(case: u64) -> Rng {
        let mut z = case.wrapping_add(0x9E37_79B9_7F4A_7C15).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z ^= z >> 30;
        z = z.wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        Rng(z | 1)
    }

    fn xorshift(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    /// In `lo..=hi`.
    fn range(&mut self, lo: i64, hi: i64) -> i64 {
        lo + (self.xorshift() % (hi - lo + 1) as u64) as i64
    }

    /// In `[0, 1)`.
    fn unit(&mut self) -> f64 {
        (self.xorshift() >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// A star-shaped ring around `(cx, cy)`: `n` points at increasing angles, each at a
/// radius in `r_min..r_max`. Counter-clockwise, and simple, since the angles are monotone.
fn star(rng: &mut Rng, cx: i64, cy: i64, n: usize, r_min: f64, r_max: f64) -> Vec<Point> {
    let mut pts: Vec<Point> = Vec::with_capacity(n);
    for i in 0..n {
        let a = (i as f64 + 0.1 + 0.8 * rng.unit()) / n as f64 * std::f64::consts::TAU;
        let r = r_min + (r_max - r_min) * rng.unit();
        let p = Point::new(cx + (r * a.cos()).round() as i64, cy + (r * a.sin()).round() as i64);
        if pts.last() != Some(&p) {
            pts.push(p);
        }
    }
    pts
}

/// What Arachne is given: an outer contour (counter-clockwise) and, for two cases in
/// three, a hole (clockwise) inside it. Even cases are star-shaped, in the engine's scaled
/// units (1 mm = 100,000); odd ones are axis-parallel frames, whose diagrams are full of
/// cocircular sites.
fn generate(case: u64) -> Polygons {
    let mut rng = Rng::for_case(case);
    let (cx, cy) = (rng.range(-8_000_000, 8_000_000), rng.range(-8_000_000, 8_000_000));
    let mut polys: Polygons = Vec::new();
    if case % 2 == 0 {
        let n = rng.range(5, 48) as usize;
        let r_out = 400_000.0 + 2_600_000.0 * rng.unit();
        polys.push(Polygon::from_points(star(&mut rng, cx, cy, n, r_out, r_out * 1.6)));
        if case % 3 != 0 {
            let m = rng.range(4, 24) as usize;
            let mut hole = star(&mut rng, cx, cy, m, r_out * 0.35, r_out * 0.8);
            hole.reverse();
            polys.push(Polygon::from_points(hole));
        }
    } else {
        let (w, h) = (rng.range(200_000, 3_000_000), rng.range(200_000, 3_000_000));
        polys.push(Polygon::from_points(vec![
            Point::new(cx - w, cy - h),
            Point::new(cx + w, cy - h),
            Point::new(cx + w, cy + h),
            Point::new(cx - w, cy + h),
        ]));
        if case % 3 != 0 {
            let t = rng.range(20_000, 150_000);
            let (iw, ih) = (w - t, h - rng.range(20_000, 150_000));
            polys.push(Polygon::from_points(vec![
                Point::new(cx - iw, cy - ih),
                Point::new(cx - iw, cy + ih),
                Point::new(cx + iw, cy + ih),
                Point::new(cx + iw, cy - ih),
            ]));
        }
    }
    polys
}

/// The segments of `polys` in the order `constructFromPolygons` makes them
/// (SkeletalTrapezoidation.cpp:410-413): per polygon, from each point to the next.
fn lines_of(polys: &Polygons) -> Vec<Line> {
    let mut lines = Vec::new();
    for poly in polys {
        let n = poly.points.len();
        for i in 0..n {
            lines.push(Line::new(poly.points[i], poly.points[(i + 1) % n]));
        }
    }
    lines
}

fn segs_of(lines: &[Line]) -> Vec<[i32; 4]> {
    lines.iter().map(|l| [l.a.x as i32, l.a.y as i32, l.b.x as i32, l.b.y as i32]).collect()
}

fn idx(i: i32) -> Option<usize> {
    (i >= 0).then_some(i as usize)
}

/// The structure of the port's diagram equal to boost's: every half-edge with its links
/// and flags, every cell.
fn same_structure(vd: &slicer::boost_polygon_voronoi::VoronoiDiagram, boost: &boost_voronoi_sys::Diagram, what: &str) {
    assert_eq!(vd.num_vertices(), boost.vertices.len(), "{what}: vertices");
    assert_eq!(vd.num_edges(), boost.edges.len(), "{what}: edges");
    assert_eq!(vd.num_cells(), boost.cells.len(), "{what}: cells");
    for (i, (e, b)) in vd.edges().iter().zip(&boost.edges).enumerate() {
        assert_eq!(e.vertex0(), idx(b.vertex0), "{what}: edge {i} vertex0");
        assert_eq!(vd.vertex1(i), idx(b.vertex1), "{what}: edge {i} vertex1");
        assert_eq!(e.twin(), idx(b.twin), "{what}: edge {i} twin");
        assert_eq!(e.next(), idx(b.next), "{what}: edge {i} next");
        assert_eq!(e.prev(), idx(b.prev), "{what}: edge {i} prev");
        assert_eq!(e.cell(), idx(b.cell), "{what}: edge {i} cell");
        assert_eq!(e.is_primary(), b.is_primary, "{what}: edge {i} is_primary");
        assert_eq!(e.is_curved(), b.is_curved, "{what}: edge {i} is_curved");
    }
    for (i, (c, b)) in vd.cells().iter().zip(&boost.cells).enumerate() {
        assert_eq!(c.source_index(), b.source_index, "{what}: cell {i} source_index");
        assert_eq!(c.incident_edge(), idx(b.incident_edge), "{what}: cell {i} incident_edge");
        assert_eq!(c.contains_point(), b.contains_point, "{what}: cell {i} contains_point");
        assert_eq!(c.contains_segment(), b.contains_segment, "{what}: cell {i} contains_segment");
    }
}

/// The graph `construct_from_polygons` leaves: the nodes in list order, and every
/// half-edge in list order with its two ends, its twin's ends and whether it has a
/// neighbour on either side.
#[derive(Debug, PartialEq)]
struct Graph {
    nodes: Vec<(i64, i64, i64)>,
    edges: Vec<[(i64, i64); 4]>,
    links: Vec<(bool, bool, bool)>,
}

fn graph_of(polys: &Polygons, voronoi_port: bool) -> Graph {
    // The strategy WallToolPaths makes for a 0.42 mm line; the graph construction reads
    // its transitioning angle only.
    let strategy = BeadingStrategyFactory::make_strategy(
        42_000, 42_000, 40_000, std::f64::consts::PI / 18.0, true, 35_700, 10_500, 0.7, 0.85, 4, 0, 1, 0.5,
    );
    let mut st = SkeletalTrapezoidation::new(&*strategy, strategy.get_transitioning_angle(), 80_000, 10_000_000, 2_500, 40_000, false, Vec::new());
    st.construct_from_polygons_with(polys, voronoi_port);
    let nodes = st.graph.nodes.iter().map(|n| (n.base.p.x, n.base.p.y, n.base.data.distance_to_boundary)).collect();
    let mut edges = Vec::new();
    let mut links = Vec::new();
    // The ends of a half-edge are pointers into `st.graph.nodes`, alive as long as `st`.
    let end = |n: Option<NodePtr>| -> (i64, i64) {
        let n = unsafe { n.unwrap().as_ref() };
        (n.p.x, n.p.y)
    };
    for e in st.graph.edges.iter() {
        let e = &e.base;
        let twin = match e.twin {
            Some(t) => {
                let t = unsafe { t.as_ref() };
                (end(t.from), end(t.to))
            }
            None => ((i64::MIN, 0), (i64::MIN, 0)),
        };
        edges.push([end(e.from), end(e.to), twin.0, twin.1]);
        links.push((e.twin.is_some(), e.prev.is_some(), e.next.is_some()));
    }
    Graph { nodes, edges, links }
}

const CASES: u64 = 300;

#[test]
fn the_diagram_arachne_walks_is_boosts() {
    for case in 0..CASES {
        let polys = generate(case);
        let lines = lines_of(&polys);
        let boost = boost_voronoi_sys::construct_segments(&segs_of(&lines));

        let mut vd = VoronoiDiagram::new();
        vd.construct_voronoi(&lines, true);
        assert_eq!(vd.get_state(), RepairState::RepairNotNeeded, "case {case}");
        assert_eq!(vd.get_issue_type(), IssueType::NoIssue, "case {case}");

        let what = format!("case {case}");
        same_structure(vd.diagram(), &boost, &what);
        for (i, (v, b)) in vd.diagram().vertices().iter().zip(&boost.vertices).enumerate() {
            assert_eq!((v.x().to_bits(), v.y().to_bits()), (b.0.to_bits(), b.1.to_bits()), "{what}: vertex {i}");
        }
    }
}

#[test]
fn the_graph_is_the_same_on_every_run_and_over_both_diagrams() {
    let (mut nodes, mut edges) = (0, 0);
    for case in 0..CASES {
        let polys = generate(case);
        let port = graph_of(&polys, true);
        assert!(!port.nodes.is_empty() && !port.edges.is_empty(), "case {case}: an empty graph");
        assert_eq!(port, graph_of(&polys, true), "case {case}: two runs over the port differ");
        // Not a law: the two diagrams' vertices differ in their last places, and a vertex
        // within that of a half-integer would round to another unit. None of these does.
        assert_eq!(port, graph_of(&polys, false), "case {case}: the port and the boostvoronoi crate differ");
        nodes += port.nodes.len();
        edges += port.edges.len();
    }
    println!("{CASES} polygon sets: {nodes} graph nodes, {edges} half-edges, equal over both diagrams");
}

/// `Point::rotated(angle)` (Point.hpp:225-231, 234) as the reference build computes it.
fn rotated(p: Point, angle: f64) -> Point {
    let (c, s) = (angle.cos(), angle.sin());
    let (x, y) = (p.x as f64, p.y as f64);
    Point::new(c.mul_add(x, -(s * y)).round() as i64, c.mul_add(y, s * x).round() as i64)
}

#[test]
fn the_repair_by_rotation_gives_boosts_diagram_of_the_rotated_input() {
    use std::f64::consts::PI;
    for case in 0..CASES {
        let polys = generate(case);
        let lines = lines_of(&polys);
        let what = format!("case {case}");

        let mut vd = VoronoiDiagram::new();
        vd.construct_voronoi(&lines, true);
        let unrotated = vd.diagram().clone();

        // Voronoi.cpp:303-304: on the first angle `detect_known_issues(*this, ...)` still
        // reads boost's diagram of the input as it was (`m_is_modified` is set at the end
        // of the function) and holds it against the rotated segments.
        let first = vd.try_to_repair_degenerated_voronoi_diagram_by_rotation(&lines, PI / 6.0);
        assert_ne!(first, IssueType::NoIssue, "{what}: the first angle examines the diagram of the input as it was");

        // From the second angle on it is the diagram of the rotated input.
        let angle = PI / 5.0;
        let second = vd.try_to_repair_degenerated_voronoi_diagram_by_rotation(&lines, angle);
        assert_eq!(second, IssueType::NoIssue, "{what}");

        let lines_rotated: Vec<Line> = lines.iter().map(|l| Line::new(rotated(l.a, angle), rotated(l.b, angle))).collect();
        let boost = boost_voronoi_sys::construct_segments(&segs_of(&lines_rotated));
        let repaired = vd.diagram();
        same_structure(repaired, &boost, &what);

        // Every vertex is boost's, rotated back (VoronoiUtils.cpp:270-283), or exactly the
        // input endpoint it stands for (Voronoi.cpp:341-347); none keeps a colour.
        let (c, s) = ((-angle).cos(), (-angle).sin());
        let endpoints: std::collections::HashSet<(i64, i64)> = lines.iter().map(|l| (l.a.x, l.a.y)).collect();
        let mut on_endpoint = 0;
        for (i, (v, b)) in repaired.vertices().iter().zip(&boost.vertices).enumerate() {
            assert_eq!(v.color(), 0, "{what}: vertex {i} colour");
            let back = (c.mul_add(b.0, -(s * b.1)), c.mul_add(b.1, s * b.0));
            if (v.x().to_bits(), v.y().to_bits()) != (back.0.to_bits(), back.1.to_bits()) {
                assert!(
                    v.x().fract() == 0.0 && v.y().fract() == 0.0 && endpoints.contains(&(v.x() as i64, v.y() as i64)),
                    "{what}: vertex {i} ({}, {}) is neither boost's rotated back ({}, {}) nor an input endpoint",
                    v.x(),
                    v.y(),
                    back.0,
                    back.1
                );
                assert!((v.x() - back.0).abs() <= 2.0 && (v.y() - back.1).abs() <= 2.0, "{what}: vertex {i} far from its endpoint");
                on_endpoint += 1;
            }
        }
        // Every input endpoint is a vertex of the diagram of a closed contour.
        assert!(on_endpoint >= endpoints.len(), "{what}: {on_endpoint} vertices on {} endpoints", endpoints.len());

        // The repaired diagram is the unrotated one up to the rounding of the rotation.
        assert_eq!(repaired.num_cells(), unrotated.num_cells(), "{what}: cells");
    }
}
