//! Integration test for the ported `build_graph` Voronoi pipeline in
//! `multi_material_segmentation` (MultiMaterialSegmentation.cpp:1670-1885).
//!
//! The crate's `--lib` test target is pre-existingly broken, so this integration test
//! exercises the newly-ported path end to end:
//!   build_graph -> remove_multiple_edges_in_vertices -> remove_nodes_with_one_arc
//!   -> extract_colored_segments
//!
//! Input: a single CCW square contour of 4 ColoredLines where two adjacent sides are
//! colour 1 and the other two are colour 2 (a real colour boundary that the graph must
//! resolve). The assertion is that the whole pipeline runs without panicking and builds a
//! non-trivial graph.

use slicer::geometry::{Line, Point};
use slicer::multi_material_segmentation::{
    build_graph, extract_colored_segments, remove_multiple_edges_in_vertices, ColoredLine,
};

fn colored_line(a: Point, b: Point, color: i32, local_line_idx: i32) -> ColoredLine {
    let mut cl = ColoredLine::new(Line::new(a, b), color);
    cl.poly_idx = 0;
    cl.local_line_idx = local_line_idx;
    cl
}

#[test]
fn build_graph_two_color_square() {
    // Closed CCW square, coords ~ +-5_000_000 scaled units.
    let p0 = Point::new(-5_000_000, -5_000_000);
    let p1 = Point::new(5_000_000, -5_000_000);
    let p2 = Point::new(5_000_000, 5_000_000);
    let p3 = Point::new(-5_000_000, 5_000_000);

    // Two adjacent sides colour 1 (bottom + right), two adjacent sides colour 2 (top + left).
    let contour = vec![
        colored_line(p0, p1, 1, 0),
        colored_line(p1, p2, 1, 1),
        colored_line(p2, p3, 2, 2),
        colored_line(p3, p0, 2, 3),
    ];
    let color_poly = vec![contour];

    // Build the graph — this drives the full Voronoi construction + append_voronoi_vertices
    // + the two-pass edge classification.
    let mut graph = build_graph(0, &color_poly);

    // The four contour points must always survive as border nodes.
    assert!(
        graph.all_border_points == 4,
        "expected 4 border points, got {}",
        graph.all_border_points
    );
    assert!(
        graph.nodes_count() >= 4,
        "graph should retain at least the 4 contour nodes, got {}",
        graph.nodes_count()
    );

    // Post-process exactly as the C++ colorize path does.
    remove_multiple_edges_in_vertices(&mut graph, &color_poly);
    graph.remove_nodes_with_one_arc();

    // extract_colored_segments must not panic and returns one bucket per extruder + 1.
    let num_extruders = 2;
    let segments = extract_colored_segments(&graph, num_extruders);
    assert_eq!(
        segments.len(),
        num_extruders + 1,
        "extract_colored_segments should return num_extruders + 1 buckets"
    );

    // The two-colour square has a real colour boundary, so the graph should produce at
    // least one interior (non-border) arc and at least one extracted colour segment.
    let interior_arc = graph
        .arcs
        .iter()
        .any(|a| a.r#type == slicer::multi_material_segmentation::ArcType::NonBorder);
    let any_segment = segments.iter().any(|bucket| !bucket.is_empty());
    assert!(
        interior_arc || any_segment,
        "two-colour square should yield an interior arc or a colour segment"
    );
}
