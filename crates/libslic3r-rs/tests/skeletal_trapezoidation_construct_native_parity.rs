//! The graph `SkeletalTrapezoidation`'s constructor builds (`constructFromPolygons`,
//! SkeletalTrapezoidation.cpp:374-516: the Voronoi diagram, computePointCellRange /
//! compute_segment_cell_range, transferEdge, makeNode, makeRib, separatePointyQuadEndNodes,
//! collapseSmallEdges and the incident edges) and `SkeletalTrapezoidation::discretize`
//! (SkeletalTrapezoidation.cpp:218-328) against the pin.
//!
//! `tests/data/skeletal_trapezoidation_construct/vectors.txt.gz` is the output of `gen.cpp` beside
//! it, which links the engine's libslic3r and runs the pin's constructor on generated polygon sets
//! (stars with and without holes, axis-aligned rectangles on a grid, thin strips), with and without
//! hole compensation, and the pin's `discretize` on the finite Voronoi edges inside the polygons'
//! bounding box. Every node (position, distance to the boundary, bead count, incident edge) and
//! every half-edge (ends, twin, prev, next, type, hole compensation flag, central state) is compared
//! in list order, as indices into the two lists.

use slicer::arachne::beading_strategy::BeadingStrategyFactory;
use slicer::arachne::skeletal_trapezoidation::SkeletalTrapezoidation;
use slicer::arachne::utils::polygons_segment_index::PolygonsSegmentIndex;
use slicer::geometry::voronoi_diagram::boost_polygon::VoronoiDiagram;
use slicer::geometry::{Point, Polygon, Polygons};
use std::collections::HashMap;
use std::io::Read;

fn vectors() -> String {
    let path = std::env::var("SKELETAL_TRAPEZOIDATION_CONSTRUCT_PARITY_VECTORS").unwrap_or_else(|_| {
        format!("{}/tests/data/skeletal_trapezoidation_construct/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"))
    });
    let file = std::fs::File::open(&path).expect("vectors");
    let mut text = String::new();
    if path.ends_with(".gz") {
        flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    } else {
        std::io::BufReader::new(file).read_to_string(&mut text).expect("text");
    }
    text
}

/// The graph of `st` in the generator's format (`g`, `n`, `e` lines).
fn dump_graph(st: &SkeletalTrapezoidation) -> Vec<String> {
    let mut ni: HashMap<usize, i64> = HashMap::new();
    let mut ei: HashMap<usize, i64> = HashMap::new();
    for (k, n) in st.graph.nodes.iter().enumerate() {
        ni.insert(&n.base as *const _ as usize, k as i64);
        ni.insert(&**n as *const _ as usize, k as i64);
    }
    for (k, e) in st.graph.edges.iter().enumerate() {
        ei.insert(&e.base as *const _ as usize, k as i64);
        ei.insert(&**e as *const _ as usize, k as i64);
    }
    let nidx = |p: Option<usize>| p.map_or(-1, |p| *ni.get(&p).expect("node in the list"));
    let eidx = |p: Option<usize>| p.map_or(-1, |p| *ei.get(&p).expect("edge in the list"));
    let mut out = vec![format!("g {} {}", st.graph.nodes.len(), st.graph.edges.len())];
    for n in st.graph.nodes.iter() {
        let n = &n.base;
        out.push(format!(
            "n {} {} {} {} {}",
            n.p.x,
            n.p.y,
            n.data.distance_to_boundary,
            n.data.bead_count,
            eidx(n.incident_edge.map(|p| p.as_ptr() as usize))
        ));
    }
    for e in st.graph.edges.iter() {
        let e = &e.base;
        // SkeletalTrapezoidationEdge.hpp:19 enum class Central { UNKNOWN = -1, NO, YES }
        let central = if !e.data.central_is_set() { -1 } else if e.data.is_central() { 1 } else { 0 };
        out.push(format!(
            "e {} {} {} {} {} {} {} {}",
            nidx(e.from.map(|p| p.as_ptr() as usize)),
            nidx(e.to.map(|p| p.as_ptr() as usize)),
            eidx(e.twin.map(|p| p.as_ptr() as usize)),
            eidx(e.prev.map(|p| p.as_ptr() as usize)),
            eidx(e.next.map(|p| p.as_ptr() as usize)),
            e.data.edge_type as i32,
            e.data.get_hole_compensation_flag() as i32,
            central
        ));
    }
    out
}

fn nums(line: &str) -> Vec<i64> {
    line.split_whitespace().skip(1).map(|t| t.parse().unwrap()).collect()
}

#[test]
fn construct_from_polygons_and_discretize_match_the_pin() {
    let text = vectors();
    let lines: Vec<&str> = text.lines().collect();
    // The strategy WallToolPaths makes for a 0.42 mm line (gen.cpp); the construction reads only its
    // transitioning angle.
    let strategy = BeadingStrategyFactory::make_strategy(
        42_000,
        42_000,
        40_000,
        // float transitioning_angle (BeadingStrategyFactory.hpp:21): gen.cpp passes float(M_PI / 18.0).
        (std::f64::consts::PI / 18.0) as f32 as f64,
        true,
        35_700,
        10_500,
        0.7,
        0.85,
        4,
        0,
        1,
        0.5,
    );
    let (mut cases, mut graph_lines, mut disc, mut graph_differ, mut disc_differ) = (0, 0, 0, 0, 0);
    let mut i = 0;
    while i < lines.len() {
        let c = nums(lines[i]);
        assert!(lines[i].starts_with("c "), "line {i}: {}", lines[i]);
        let (case, npolys, hc, nholes) = (c[0], c[1] as usize, c[2] != 0, c[3] as usize);
        let holes: Vec<i32> = c[4..4 + nholes].iter().map(|&h| h as i32).collect();
        let step = c[4 + nholes];
        i += 1;
        let mut polys: Polygons = Vec::new();
        for _ in 0..npolys {
            let v = nums(lines[i]);
            let pts = (0..v[0] as usize).map(|k| Point::new(v[1 + 2 * k], v[2 + 2 * k])).collect();
            polys.push(Polygon::from_points(pts));
            i += 1;
        }
        let mut st = SkeletalTrapezoidation::new(&*strategy, strategy.get_transitioning_angle(), step, 10_000_000, 2_500, 40_000, hc, holes);
        st.construct_from_polygons(&polys);
        let ours = dump_graph(&st);
        let g = nums(lines[i]);
        let n = 1 + g[0] as usize + g[1] as usize;
        let pin = &lines[i..i + n];
        graph_lines += n;
        if ours.len() != n || ours.iter().zip(pin).any(|(a, b)| a != b) {
            graph_differ += 1;
            if graph_differ <= 3 {
                let k = ours.iter().zip(pin).position(|(a, b)| a != b).unwrap_or(ours.len().min(n));
                eprintln!(
                    "case {case}: graph differs at line {k} of {n} (ours {}): pin {:?} ours {:?}",
                    ours.len(),
                    pin.get(k),
                    ours.get(k)
                );
            }
        }
        i += n;

        // SkeletalTrapezoidation.cpp:419-424: the segments and the diagram constructFromPolygons builds.
        let mut segments: Vec<PolygonsSegmentIndex> = Vec::new();
        for poly_idx in 0..polys.len() {
            for point_idx in 0..polys[poly_idx].points.len() {
                segments.push(PolygonsSegmentIndex::with_indices(&polys, poly_idx, point_idx));
            }
        }
        let mut vd = VoronoiDiagram::new();
        vd.construct_voronoi(&segments, true);
        while i < lines.len() && lines[i].starts_with("d ") {
            let d = nums(lines[i]);
            let pts = st.discretize(vd.diagram(), d[0] as usize, &segments);
            let mut ours = format!("d {} {}", d[0], pts.len());
            for q in &pts {
                ours.push_str(&format!(" {} {}", q.x, q.y));
            }
            disc += 1;
            if ours != lines[i] {
                disc_differ += 1;
                if disc_differ <= 3 {
                    eprintln!("case {case}: discretize differs\n  pin  {}\n  ours {ours}", lines[i]);
                }
            }
            i += 1;
        }
        cases += 1;
    }
    println!(
        "{cases} polygon sets: {graph_lines} graph lines ({graph_differ} sets differ), {disc} discretized edges ({disc_differ} differ)"
    );
    assert!(cases > 0);
    assert_eq!(graph_differ, 0, "graphs that differ from the pin");
    assert_eq!(disc_differ, 0, "discretized edges that differ from the pin");
}
