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
