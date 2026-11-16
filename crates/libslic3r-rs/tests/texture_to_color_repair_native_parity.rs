//! `texture_to_color/repair.rs` against Bambu Studio's `TextureToColor/Repair.hpp` at the pin.
//!
//! `ComputeBoundaryEdgeStats` is compared with the built C++ over the meshes of
//! `tests/data/texture_to_color_cgal_utils/vectors_mesh.txt` (written by `gen.cpp` of that
//! directory). `RepairMesh` and `CloseBoundariesAndRepairManifoldness` are chains of CGAL
//! algorithms the crate has no port of; no reference is possible for them, so their tests give
//! them a recording stand-in for the algorithms and require the calls, the progress reports and
//! the points of cancellation Repair.hpp has, read off the C++.

use std::cell::RefCell;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use slicer::texture_to_color::callbacks::{AlgoCancelCallback, AlgoProgress, AlgoProgressCallback};
use slicer::texture_to_color::cgal_utils::*;
use slicer::texture_to_color::repair::*;
use slicer::texture_to_color::tri_mesh::TriMesh;
use slicer::triangle_set_sampling::{Vec3f, Vec3i};

#[test]
fn compute_boundary_edge_stats_matches_the_pin() {
    let text = include_str!("data/texture_to_color_cgal_utils/vectors_mesh.txt");
    let mut mesh = TriMesh::default();
    let mut cases = 0;
    for (lineno, line) in text.lines().enumerate() {
        let mut t = line.split_whitespace();
        match t.next().expect("tag") {
            "mesh" => {
                let nv: usize = t.next().unwrap().parse().unwrap();
                let mut f = || f32::from_bits(u32::from_str_radix(t.next().unwrap(), 16).unwrap());
                let vertices: Vec<Vec3f> = (0..nv).map(|_| Vec3f::new(f(), f(), f())).collect();
                let nf: usize = t.next().unwrap().parse().unwrap();
                let mut i = || t.next().unwrap().parse::<i32>().unwrap();
                let indices: Vec<Vec3i> = (0..nf).map(|_| Vec3i::new(i(), i(), i())).collect();
                mesh = TriMesh::new(indices, vertices);
            }
            "ComputeBoundaryEdgeStats" => {
                assert_eq!(t.next(), Some("->"));
                let mut u = || t.next().unwrap().parse::<usize>().unwrap();
                let want = BoundaryEdgeStats { total_boundary_edges: u(), max_cycle_edges: u(), cycle_count: u() };
                assert_eq!(ComputeBoundaryEdgeStats(&trimesh_to_cgal(&mesh)), want, "line {}", lineno + 1);
                cases += 1;
            }
            _ => {}
        }
    }
    assert_eq!(cases, 32);
}

#[test]
fn repair_setting_defaults() {
    // Repair.hpp:24-32
    assert_eq!(MAX_REPAIRABLE_MESH_HOLE_EDGES, 500);
    assert_eq!(MAX_REPAIRABLE_MESH_BOUNDARY_EDGES, 5000);
    assert_eq!(RepairSetting::default(), RepairSetting { max_hole_edges: 500, max_boundary_edges: 5000 });
    assert_eq!(BoundaryEdgeStats::default(), BoundaryEdgeStats { total_boundary_edges: 0, max_cycle_edges: 0, cycle_count: 0 });
}

/// Records the calls; `polygon_soup_to_polygon_mesh` builds the mesh with `add_face`, the other
/// algorithms change nothing.
#[derive(Default)]
struct Recorder {
    calls: RefCell<Vec<String>>,
}

impl PolygonMeshProcessing for Recorder {
    fn does_self_intersect(&self, _mesh: &CGALMesh) -> bool {
        self.calls.borrow_mut().push("does_self_intersect".into());
        false
    }
    fn remove_self_intersections(&self, _mesh: &mut CGALMesh) -> bool {
        self.calls.borrow_mut().push("remove_self_intersections".into());
        true
    }
    fn isotropic_remeshing(&self, _mesh: &mut CGALMesh, _l: f64, _n: usize, _p: bool, _e: &[bool], _v: &[bool], _c: bool) {
        self.calls.borrow_mut().push("isotropic_remeshing".into());
    }
    fn repair_polygon_soup(&self, points: &mut Vec<Point_3>, polygons: &mut Vec<Vec<usize>>) {
        self.calls.borrow_mut().push(format!("repair_polygon_soup {} {}", points.len(), polygons.len()));
    }
    fn orient_polygon_soup(&self, _points: &mut Vec<Point_3>, _polygons: &mut Vec<Vec<usize>>) -> bool {
        self.calls.borrow_mut().push("orient_polygon_soup".into());
        true
    }
    fn polygon_soup_to_polygon_mesh(&self, points: &[Point_3], polygons: &[Vec<usize>], out: &mut CGALMesh) {
        self.calls.borrow_mut().push("polygon_soup_to_polygon_mesh".into());
        let vs: Vec<SM_Vertex_index> = points.iter().map(|p| out.add_vertex(*p)).collect();
        for p in polygons.iter() {
            out.add_face(vs[p[0]], vs[p[1]], vs[p[2]]);
        }
    }
    fn remove_degenerate_faces(&self, _mesh: &mut CGALMesh) -> bool {
        self.calls.borrow_mut().push("remove_degenerate_faces".into());
        true
    }
    fn stitch_borders(&self, _mesh: &mut CGALMesh) -> usize {
        self.calls.borrow_mut().push("stitch_borders".into());
        0
    }
    fn duplicate_non_manifold_vertices(&self, _mesh: &mut CGALMesh) -> usize {
        self.calls.borrow_mut().push("duplicate_non_manifold_vertices".into());
        0
    }
    fn triangulate_hole(&self, _mesh: &mut CGALMesh, border_halfedge: SM_Halfedge_index, _patch_faces: &mut Vec<SM_Face_index>) {
        self.calls.borrow_mut().push(format!("triangulate_hole {}", border_halfedge.0));
    }
}

/// Two triangles over a square: one border cycle of four edges.
fn square() -> TriMesh {
    TriMesh::new(
        vec![Vec3i::new(0, 1, 2), Vec3i::new(0, 2, 3)],
        vec![Vec3f::new(0.0, 0.0, 0.0), Vec3f::new(1.0, 0.0, 0.0), Vec3f::new(1.0, 1.0, 0.0), Vec3f::new(0.0, 1.0, 0.0)],
    )
}

fn progress_recorder() -> (AlgoProgressCallback, Arc<Mutex<Vec<(i32, String)>>>) {
    let seen = Arc::new(Mutex::new(Vec::new()));
    let sink = seen.clone();
    (Some(Arc::new(move |p: AlgoProgress| sink.lock().unwrap().push((p.percent, p.message)))), seen)
}

#[test]
fn repair_mesh_runs_the_stages_of_the_pin_in_order() {
    // Repair.hpp:85-250
    let pmp = Recorder::default();
    let (progress, seen) = progress_recorder();
    let mut out: Option<TriMesh> = None;
    assert!(RepairMesh(&square(), &mut out, &progress, &None, &RepairSetting::default(), &pmp));
    let border = extract_boundary_cycles(&trimesh_to_cgal(&square()));
    assert_eq!(border.len(), 1);
    assert_eq!(
        *pmp.calls.borrow(),
        vec![
            "repair_polygon_soup 4 2".to_string(),
            "orient_polygon_soup".into(),
            "polygon_soup_to_polygon_mesh".into(),
            "remove_degenerate_faces".into(),
            "stitch_borders".into(),
            "duplicate_non_manifold_vertices".into(),
            // CloseBoundariesAndRepairManifoldness, Repair.hpp:64-83
            "stitch_borders".into(),
            "duplicate_non_manifold_vertices".into(),
            format!("triangulate_hole {}", border[0].0),
            "remove_degenerate_faces".into(),
            "duplicate_non_manifold_vertices".into(),
        ]
    );
    let seen: Vec<(i32, String)> = seen.lock().unwrap().clone();
    assert_eq!(
        seen.iter().map(|(p, m)| (*p, m.as_str())).collect::<Vec<_>>(),
        vec![
            (30, "Repairing polygon soup"),
            (50, "Orienting polygon soup"),
            (70, "Converting to CGAL mesh"),
            (80, "Closing mesh boundaries"),
            (85, "Converting from CGAL mesh"),
            (100, "Done")
        ]
    );
    let out = out.expect("a mesh");
    assert_eq!(out.vertices, square().vertices);
    // cgal_to_trimesh starts every face at the target of its halfedge
    assert_eq!(out.facets_count(), 2);
}

#[test]
fn repair_mesh_leaves_holes_that_are_too_large() {
    // Repair.hpp:186-205: the holes are closed when the border edges are at most
    // max_boundary_edges and the longest cycle at most max_hole_edges
    for (setting, closes) in [
        (RepairSetting { max_hole_edges: 4, max_boundary_edges: 4 }, true),
        (RepairSetting { max_hole_edges: 3, max_boundary_edges: 4 }, false),
        (RepairSetting { max_hole_edges: 4, max_boundary_edges: 3 }, false),
    ] {
        let pmp = Recorder::default();
        let mut out: Option<TriMesh> = None;
        assert!(RepairMesh(&square(), &mut out, &None, &None, &setting, &pmp));
        let closed = pmp.calls.borrow().iter().any(|c| c.starts_with("triangulate_hole"));
        assert_eq!(closed, closes, "{setting:?}");
        assert_eq!(pmp.calls.borrow().len(), if closes { 11 } else { 6 });
    }
}

#[test]
fn repair_mesh_stops_where_the_pin_asks_for_cancellation() {
    // Repair.hpp:119, 133, 147, 169, 215: the cancel callback is asked five times, after the
    // progress reports 30, 50, 70, 80 and 85; `out_mesh` stays as it was
    let calls_before = [0usize, 1, 2, 4, 11];
    for stop_at in 1..=5usize {
        let pmp = Recorder::default();
        let (progress, seen) = progress_recorder();
        let asked = Arc::new(AtomicUsize::new(0));
        let counter = asked.clone();
        let cancel: AlgoCancelCallback = Some(Arc::new(move || counter.fetch_add(1, Ordering::SeqCst) + 1 >= stop_at));
        let mut out: Option<TriMesh> = None;
        assert!(!RepairMesh(&square(), &mut out, &progress, &cancel, &RepairSetting::default(), &pmp));
        assert!(out.is_none());
        assert_eq!(asked.load(Ordering::SeqCst), stop_at);
        assert_eq!(seen.lock().unwrap().len(), stop_at);
        assert_eq!(pmp.calls.borrow().len(), calls_before[stop_at - 1], "cancelled at the {stop_at}. question");
    }
}
