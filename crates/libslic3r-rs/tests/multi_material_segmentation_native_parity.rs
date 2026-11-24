//! Native parity of the functions of MultiMaterialSegmentation.cpp that work on an annotated Voronoi
//! diagram (`slicer::multi_material_segmentation`: non_deleted_edge_count, can_vertex_be_deleted,
//! delete_vertex_deep, edge_length, calc_total_edge_length, remove_multiple_edges_in_vertex) against
//! vectors made by the C++ at the pin (tests/data/multi_material_segmentation/gen.cpp).
//!
//! A line of the vectors is "vd <inputs> => <A> | <B> | <C>"; the generator says what the three parts
//! are. This test builds the diagram of the same lines with the port of boost's builder (which
//! tests/boost_polygon_voronoi_native_parity.rs holds equal to the library, element for element),
//! annotates it as the generator did and has to print the same results.
//! `MULTI_MATERIAL_SEGMENTATION_PARITY_VECTORS` names a file of the generator to read instead of the
//! committed one.

use slicer::boost_polygon_voronoi::{construct_voronoi, VoronoiDiagram};
use slicer::geometry::voronoi_utils::VdColors;
use slicer::multi_material_segmentation::{
    calc_total_edge_length, can_vertex_be_deleted, delete_vertex_deep, edge_length, non_deleted_edge_count, remove_multiple_edges_in_vertex, VD_ANNOTATION,
};
use std::io::Read;

/// gen.cpp `COLORS(vd)`
fn colors(vd: &VoronoiDiagram) -> String {
    let mut o: Vec<String> = Vec::new();
    for v in 0..vd.num_vertices() {
        o.push(vd.vertex_color(v).to_string());
    }
    for e in 0..vd.num_edges() {
        o.push(vd.edge_color(e).to_string());
    }
    o.join(" ")
}

fn read_vectors() -> Vec<String> {
    if let Ok(path) = std::env::var("MULTI_MATERIAL_SEGMENTATION_PARITY_VECTORS") {
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
        return text.lines().map(|l| l.to_string()).collect();
    }
    let path = format!("{}/tests/data/multi_material_segmentation/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).unwrap_or_else(|e| panic!("{path}: {e}"));
    text.lines().map(|l| l.to_string()).collect()
}

#[test]
fn multi_material_segmentation_native_parity() {
    let expected = read_vectors();
    let mut mismatches = [0usize; 3];
    let mut vertices = 0usize;
    let mut edges = 0usize;
    for (k, line) in expected.iter().enumerate() {
        let (head, want) = line.split_once(" => ").unwrap_or_else(|| panic!("no result in {line}"));
        let mut t = head.split_whitespace();
        assert_eq!(t.next(), Some("vd"));
        let mut int = || -> i64 { t.next().expect("token").parse().expect("integer") };
        let n = int();
        let mut segments: Vec<[i32; 4]> = Vec::new();
        for _ in 0..n {
            segments.push([int() as i32, int() as i32, int() as i32, int() as i32]);
        }
        let mut list = |int: &mut dyn FnMut() -> i64| -> Vec<usize> {
            let n = int();
            (0..n).map(|_| int() as usize).collect()
        };
        let on_contour = list(&mut int);
        let deleted = list(&mut int);
        let tried = list(&mut int);

        let mut vd = VoronoiDiagram::new();
        construct_voronoi(&segments, &mut vd);
        vertices += vd.num_vertices();
        edges += vd.num_edges();

        // gen.cpp: the infinite edges are deleted, then the marks.
        for e in 0..vd.num_edges() {
            if vd.is_infinite(e) {
                let twin = vd.edges()[e].twin().expect("twin");
                vd.set_edge_color(e, VD_ANNOTATION::DELETED);
                vd.set_edge_color(twin, VD_ANNOTATION::DELETED);
            }
        }
        for &v in &on_contour {
            vd.set_vertex_color(v, VD_ANNOTATION::VERTEX_ON_CONTOUR);
        }
        for &e in &deleted {
            let twin = vd.edges()[e].twin().expect("twin");
            vd.set_edge_color(e, VD_ANNOTATION::DELETED);
            vd.set_edge_color(twin, VD_ANNOTATION::DELETED);
        }

        // A
        let mut a: Vec<String> = Vec::new();
        for v in 0..vd.num_vertices() {
            a.push(non_deleted_edge_count(&vd, v).to_string());
            a.push(if can_vertex_be_deleted(&vd, v) { "1".into() } else { "0".into() });
        }
        for e in 0..vd.num_edges() {
            if vd.edge_color(e) == VD_ANNOTATION::DELETED {
                a.push("-".into());
            } else {
                a.push(format!("{:016x}", edge_length(&vd, e).to_bits()));
                a.push(format!("{:016x}", calc_total_edge_length(&vd, e).to_bits()));
            }
        }
        // B
        for &v in &tried {
            if can_vertex_be_deleted(&vd, v) {
                delete_vertex_deep(&mut vd, v);
            }
        }
        let b = colors(&vd);
        // C
        for v in 0..vd.num_vertices() {
            if vd.vertex_color(v) == VD_ANNOTATION::VERTEX_ON_CONTOUR {
                remove_multiple_edges_in_vertex(&mut vd, v);
            }
        }
        let c = colors(&vd);

        let got = [a.join(" "), b, c];
        let want: Vec<&str> = want.split(" | ").collect();
        assert_eq!(want.len(), 3, "line {}", k + 1);
        for part in 0..3 {
            if got[part] != want[part] {
                mismatches[part] += 1;
                if mismatches.iter().sum::<usize>() <= 6 {
                    let wt: Vec<&str> = want[part].split(' ').collect();
                    let gt: Vec<&str> = got[part].split(' ').collect();
                    let i = (0..wt.len().min(gt.len())).find(|&i| wt[i] != gt[i]).unwrap_or(wt.len().min(gt.len()));
                    let lo = i.saturating_sub(4);
                    eprintln!(
                        "line {} part {}: first difference at token {} of {} / {}\n  C++:  {}\n  Rust: {}",
                        k + 1,
                        ["A", "B", "C"][part],
                        i,
                        wt.len(),
                        gt.len(),
                        wt[lo..(i + 5).min(wt.len())].join(" "),
                        gt[lo..(i + 5).min(gt.len())].join(" ")
                    );
                }
            }
        }
    }
    eprintln!(
        "multi_material_segmentation parity: {} diagrams ({} vertices, {} edges); lines that differ: A {}, B {}, C {}",
        expected.len(),
        vertices,
        edges,
        mismatches[0],
        mismatches[1],
        mismatches[2]
    );
    assert_eq!(mismatches, [0, 0, 0], "of {} diagrams", expected.len());
}

/// Vectors of tests/data/multi_material_segmentation/gen_painted.cpp; `MMS_PAINTED_PARITY_VECTORS`
/// names a file of the generator to read instead of the committed one.
fn read_painted_vectors() -> Vec<String> {
    if let Ok(path) = std::env::var("MMS_PAINTED_PARITY_VECTORS") {
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
        return text.lines().map(|l| l.to_string()).collect();
    }
    let path = format!("{}/tests/data/multi_material_segmentation/painted_vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).unwrap_or_else(|e| panic!("{path}: {e}"));
    text.lines().map(|l| l.to_string()).collect()
}

/// post_process_painted_lines (with filter_painted_lines; MultiMaterialSegmentation.cpp:642-719) and
/// colorize_contours (with colorize_contour, colorize_line, filter_colorized_polygon; 731-959) against
/// the pin on synthetic contours whose painted lines tie in the sort's comparator (gen_painted.cpp).
#[test]
fn painted_lines_native_parity() {
    use slicer::edge_grid::Contour;
    use slicer::geometry::{Line, Point};
    use slicer::multi_material_segmentation::{
        build_graph, colorize_contours, extract_colored_segments, has_layer_only_one_color, post_process_painted_lines, remove_multiple_edges_in_vertices, PaintedLine,
    };
    let lines = read_painted_vectors();
    assert!(!lines.is_empty());
    let (mut bad_pp, mut bad_col, mut n_painted) = (0usize, 0usize, 0usize);
    let (mut bad_seg, mut n_graph, mut n_graph_panic) = (0usize, 0usize, 0usize);
    let default_hook = std::panic::take_hook();
    // Diagnostic: MMS_PAINTED_PANIC_LOC=1 prints where a graph-stage panic happened.
    std::panic::set_hook(Box::new(|info| {
        if std::env::var_os("MMS_PAINTED_PANIC_LOC").is_some() {
            eprintln!("PANIC {}", info);
        }
    }));
    for (k, l) in lines.iter().enumerate() {
        let (inp, res) = l.split_once(" => ").expect("=>");
        let (want_pp, rest) = res.split_once(" | ").expect("|");
        let (want_col, want_seg) = rest.split_once(" # ").expect("#");
        let t: Vec<i64> = inp[3..].split(' ').filter(|s| *s != ";").map(|s| s.parse().unwrap()).collect();
        let mut i = 0usize;
        let nc = t[i] as usize;
        i += 1;
        let mut contours = Vec::with_capacity(nc);
        for _ in 0..nc {
            let n = t[i] as usize;
            i += 1;
            let pts: Vec<Point> = (0..n).map(|j| Point::new(t[i + 2 * j], t[i + 2 * j + 1])).collect();
            i += 2 * n;
            contours.push(Contour::new_closed(pts));
        }
        let m = t[i] as usize;
        i += 1;
        let mut painted = Vec::with_capacity(m);
        for _ in 0..m {
            painted.push(PaintedLine {
                contour_idx: t[i] as usize,
                line_idx: t[i + 1] as usize,
                projected_line: Line { a: Point::new(t[i + 2], t[i + 3]), b: Point::new(t[i + 4], t[i + 5]) },
                color: t[i + 6] as i32,
            });
            i += 7;
        }
        n_painted += m;
        let mut post = post_process_painted_lines(&contours, painted);
        if post.is_empty() {
            // gen_painted.cpp: no painted line gives no colorization input; the generator passes empty
            // vectors per contour.
            post = vec![Vec::new(); nc];
        }
        let mut got_pp: Vec<String> = Vec::new();
        for v in &post {
            got_pp.push(v.len().to_string());
            for pl in v {
                let pl: &PaintedLine = pl;
                got_pp.push(format!(
                    "{} {} {} {} {} {} {}",
                    pl.contour_idx, pl.line_idx, pl.projected_line.a.x, pl.projected_line.a.y, pl.projected_line.b.x, pl.projected_line.b.y, pl.color
                ));
            }
        }
        let col = colorize_contours(&contours, &post);
        let mut got_col: Vec<String> = Vec::new();
        for v in &col {
            got_col.push(v.len().to_string());
            for cl in v {
                got_col.push(format!(
                    "{} {} {} {} {} {} {}",
                    cl.line.a.x, cl.line.a.y, cl.line.b.x, cl.line.b.y, cl.color, cl.poly_idx, cl.local_line_idx
                ));
            }
        }
        // MultiMaterialSegmentation.cpp:2420-2428: the graph stage of a layer.
        let got_seg = if has_layer_only_one_color(&col) {
            "one".to_string()
        } else {
            n_graph += 1;
            // A diagram on which the pin's Voronoi.cpp logs "missing Voronoi vertex even after the
            // rotation of input" can carry edges with extreme coordinates: Line::intersection's integer
            // subtraction then overflows (a panic in a debug build; the pin wraps). Counted apart.
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            // Diagnostic only: MMS_PAINTED_DUMP_CASE=<1-based case> prints the graph after each step
            // in the format of a probe copy of gen_painted.cpp (G/N/A lines).
            let dump = std::env::var("MMS_PAINTED_DUMP_CASE").ok().and_then(|v| v.parse::<usize>().ok()) == Some(k + 1);
            let dumpg = |st: &str, g: &slicer::multi_material_segmentation::MmuGraph| {
                if !dump {
                    return;
                }
                eprintln!("G {} nodes {} arcs {}", st, g.nodes.len(), g.arcs.len());
                for (i, n) in g.nodes.iter().enumerate() {
                    let a: Vec<String> = n.arc_idxs.iter().map(|x| x.to_string()).collect();
                    eprintln!("N {} {} {} {} : {}", st, i, n.point.x, n.point.y, a.join(" "));
                }
                for (i, a) in g.arcs.iter().enumerate() {
                    eprintln!("A {} {} {} {} {} {:?}", st, i, a.from_idx, a.to_idx, a.color, a.r#type);
                }
            };
            let mut graph = build_graph(0, &col);
            dumpg("b", &graph);
            remove_multiple_edges_in_vertices(&mut graph, &col);
            dumpg("m", &graph);
            graph.remove_nodes_with_one_arc();
            dumpg("r", &graph);
            let segs = extract_colored_segments(&graph, 3);
            let mut o: Vec<String> = vec![segs.len().to_string()];
            for v in &segs {
                o.push(v.len().to_string());
                for poly in v {
                    o.push(format!("0 {}", poly.points.len()));
                    for p in &poly.points {
                        o.push(format!("{} {}", p.x, p.y));
                    }
                }
            }
            o.join(" ")
            }))
            .unwrap_or_else(|_| "PANIC".to_string())
        };
        if got_seg == "PANIC" {
            n_graph_panic += 1;
        } else if got_seg != want_seg {
            bad_seg += 1;
            if bad_seg <= 3 {
                eprintln!("case {}: graph stage differs\n  C++:  {}\n  Rust: {}", k + 1, want_seg, got_seg);
            }
        }
        let (gp, gc) = (got_pp.join(" "), got_col.join(" "));
        if gp != want_pp {
            bad_pp += 1;
            if bad_pp <= 3 {
                eprintln!("case {}: post_process_painted_lines differs\n  C++:  {}\n  Rust: {}", k + 1, want_pp, gp);
            }
        }
        if gc != want_col {
            bad_col += 1;
            if bad_col <= 3 {
                eprintln!("case {}: colorize_contours differs\n  C++:  {}\n  Rust: {}", k + 1, want_col, gc);
            }
        }
    }
    std::panic::set_hook(default_hook);
    eprintln!(
        "painted lines parity: {} cases, {} painted lines, {} graphs; differ: post_process_painted_lines {}, colorize_contours {}, graph stage {} (+ {} graphs that panic in Rust)",
        lines.len(),
        n_painted,
        n_graph,
        bad_pp,
        bad_col,
        bad_seg,
        n_graph_panic
    );
    // The graph stage (build_graph, remove_multiple_edges_in_vertices, remove_nodes_with_one_arc,
    // extract_colored_segments) is asserted since R944: case 578 needed the pin's int32 coord_t in
    // clip_finite_voronoi_edge (MMS_CLIP_EDGE_I32_PIN), the 7 that panicked the wrapping subtraction
    // of Line::intersection (LINE_INTERSECTION_WRAP_PIN) and the pin's NDEBUG build of points_inside
    // (MMS_NDEBUG_ASSERTS_PIN).
    assert_eq!((bad_pp, bad_col), (0, 0), "of {} cases", lines.len());
    assert_eq!((bad_seg, n_graph_panic), (0, 0), "graph stage, of {} graphs", n_graph);
}

/// merge_segmented_layers (MultiMaterialSegmentation.cpp:2039-2079) against
/// tests/data/multi_material_segmentation/merge_vectors.txt.gz (gen_merge.cpp, the pin on one thread): a line
/// is "ms <layers> <extruders> <segmented[layer][0..=extruders]> ; <top_and_bottom[0..=extruders][layer]> =>
/// <merged[layer][0..extruders]>", every ExPolygons as "<n> (<rings> (<points> (x y)*)*)*".
/// `MMS_MERGE_PARITY_VECTORS` names an uncompressed file of the generator to read instead.
#[test]
fn merge_segmented_layers_native_parity() {
    use slicer::geometry::{ExPolygon, ExPolygons, Point, Polygon};
    use slicer::multi_material_segmentation::merge_segmented_layers;
    let text = match std::env::var("MMS_MERGE_PARITY_VECTORS") {
        Ok(p) => std::fs::read_to_string(p).expect("vectors"),
        Err(_) => {
            let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/multi_material_segmentation/merge_vectors.txt.gz");
            let mut s = String::new();
            flate2::read::GzDecoder::new(std::fs::File::open(path).expect("merge_vectors.txt.gz")).read_to_string(&mut s).expect("gunzip");
            s
        }
    };
    fn read_expolys<'a>(it: &mut impl Iterator<Item = &'a str>) -> ExPolygons {
        let mut next = || it.next().expect("token").parse::<i64>().expect("int");
        let n = next() as usize;
        let mut out = ExPolygons::new();
        for _ in 0..n {
            let rings = next() as usize;
            let mut polys: Vec<Polygon> = Vec::new();
            for _ in 0..rings {
                let np = next() as usize;
                let mut pts = Vec::with_capacity(np);
                for _ in 0..np {
                    let x = next();
                    let y = next();
                    pts.push(Point::new(x, y));
                }
                polys.push(Polygon::from_points(pts));
            }
            let contour = polys.remove(0);
            out.push(ExPolygon { contour, holes: polys });
        }
        out
    }
    fn print_expolys(o: &mut Vec<String>, e: &ExPolygons) {
        o.push(e.len().to_string());
        for ex in e {
            o.push((ex.holes.len() + 1).to_string());
            for p in std::iter::once(&ex.contour).chain(ex.holes.iter()) {
                o.push(p.points.len().to_string());
                for pt in &p.points {
                    o.push(format!("{} {}", pt.x, pt.y));
                }
            }
        }
    }
    let (mut n, mut bad) = (0usize, 0usize);
    for line in text.lines() {
        let (inp, want) = line.split_once(" => ").expect("=>");
        let mut it = inp.split_whitespace();
        assert_eq!(it.next(), Some("ms"));
        let num_layers: usize = it.next().unwrap().parse().unwrap();
        let num_extruders: usize = it.next().unwrap().parse().unwrap();
        let mut segmented: Vec<Vec<ExPolygons>> = Vec::new();
        for _ in 0..num_layers {
            let mut v = Vec::new();
            for _ in 0..=num_extruders {
                v.push(read_expolys(&mut it));
            }
            segmented.push(v);
        }
        assert_eq!(it.next(), Some(";"));
        let mut tb: Vec<Vec<ExPolygons>> = Vec::new();
        for _ in 0..=num_extruders {
            let mut v = Vec::new();
            for _ in 0..num_layers {
                v.push(read_expolys(&mut it));
            }
            tb.push(v);
        }
        let merged = merge_segmented_layers(&segmented, tb, num_extruders);
        let mut o: Vec<String> = Vec::new();
        for l in 0..num_layers {
            for e in 0..num_extruders {
                print_expolys(&mut o, &merged[l][e]);
            }
        }
        let got = o.join(" ");
        n += 1;
        if got != want.trim() {
            bad += 1;
            if bad <= 3 {
                eprintln!("case {}: merge_segmented_layers differs\n  C++:  {}\n  Rust: {}", n, want.trim(), got);
            }
        }
    }
    eprintln!("merge_segmented_layers parity: {} cases, {} differ", n, bad);
    assert!(n > 0);
    assert_eq!(bad, 0, "merge_segmented_layers differs from the pin");
}
