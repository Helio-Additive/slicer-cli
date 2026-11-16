//! Port of `TextureToColor/Repair.hpp`: making a triangle mesh one the tex2color algorithms can
//! work on (a halfedge mesh without holes).
//!
//! C++ Reference:
//! - src/libslic3r/TextureToColor/Repair.hpp
//!
//! FIDELITY-NOTE: `CloseBoundariesAndRepairManifoldness` and `RepairMesh` are chains of CGAL
//! algorithms (`PMP::repair_polygon_soup`, `orient_polygon_soup`, `polygon_soup_to_polygon_mesh`,
//! `remove_degenerate_faces`, `stitch_borders`, `duplicate_non_manifold_vertices`,
//! `triangulate_hole`) the crate has no port of. They take the algorithms as
//! `cgal_utils::PolygonMeshProcessing` and are the port of what is between the calls: the order,
//! the progress and cancel callbacks, the limits. `ComputeBoundaryEdgeStats` is whole.
//!
//! The durations C++ logs between the stages are not measured.
//!
//! Nothing in the slicing path calls this unit. In C++ `RepairMesh` is called by
//! `repair_cluster_smooth` of TextureToColor.cpp.

#![allow(non_snake_case)]

use crate::texture_to_color::callbacks::{AlgoCancelCallback, AlgoProgress, AlgoProgressCallback};
use crate::texture_to_color::cgal_utils::{self, CGALMesh, Point_3, PolygonMeshProcessing, SM_Face_index};
use crate::texture_to_color::tri_mesh::TriMesh;

/// Repair.hpp:24
pub const MAX_REPAIRABLE_MESH_HOLE_EDGES: usize = 500;

/// Repair.hpp:26
pub const MAX_REPAIRABLE_MESH_BOUNDARY_EDGES: usize = 5000;

/// Repair.hpp:28-32
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RepairSetting {
    pub max_hole_edges: usize,
    pub max_boundary_edges: usize,
}

impl Default for RepairSetting {
    /// Repair.hpp:30-31, the member initializers
    fn default() -> Self {
        RepairSetting { max_hole_edges: MAX_REPAIRABLE_MESH_HOLE_EDGES, max_boundary_edges: MAX_REPAIRABLE_MESH_BOUNDARY_EDGES }
    }
}

/// Repair.hpp:34-39
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BoundaryEdgeStats {
    pub total_boundary_edges: usize,
    pub max_cycle_edges: usize,
    pub cycle_count: usize,
}

/// Repair.hpp:41-62: the number of border cycles of the mesh, the edges of the longest and of all
pub fn ComputeBoundaryEdgeStats(cgal_mesh: &CGALMesh) -> BoundaryEdgeStats {
    let border_cycles = cgal_utils::extract_boundary_cycles(cgal_mesh);

    let mut stats = BoundaryEdgeStats::default();
    stats.cycle_count = border_cycles.len();
    for &h0 in border_cycles.iter() {
        let mut len: usize = 0;
        let mut h = h0;
        loop {
            len += 1;
            h = cgal_mesh.next(h);
            if h == h0 {
                break;
            }
        }
        stats.max_cycle_edges = stats.max_cycle_edges.max(len);
        stats.total_boundary_edges += len;
    }
    stats
}

/// Repair.hpp:64-83: borders stitched, every hole triangulated, degenerate faces removed. See the
/// FIDELITY-NOTE of the file.
pub fn CloseBoundariesAndRepairManifoldness(cgal_mesh: &mut CGALMesh, pmp: &dyn PolygonMeshProcessing) {
    pmp.stitch_borders(cgal_mesh);
    pmp.duplicate_non_manifold_vertices(cgal_mesh);

    let border_cycles = cgal_utils::extract_boundary_cycles(cgal_mesh);

    for &h in border_cycles.iter() {
        let mut patch_faces: Vec<SM_Face_index> = Vec::new();
        pmp.triangulate_hole(cgal_mesh, h, &mut patch_faces);
    }

    pmp.remove_degenerate_faces(cgal_mesh);
    pmp.duplicate_non_manifold_vertices(cgal_mesh);
}

/// Repair.hpp:85-250: the mesh as a polygon soup, repaired, oriented and made a mesh again, its
/// holes closed when they are few and small enough. `out_mesh` is set on success only. In C++
/// `progress_callback` and `cancel_callback` default to none and `setting` to `RepairSetting{}`.
/// See the FIDELITY-NOTE of the file.
pub fn RepairMesh(
    mesh: &TriMesh,
    out_mesh: &mut Option<TriMesh>,
    progress_callback: &AlgoProgressCallback,
    cancel_callback: &AlgoCancelCallback,
    setting: &RepairSetting,
    pmp: &dyn PolygonMeshProcessing,
) -> bool {
    let progress = |percent: i32, message: &str| {
        if let Some(cb) = progress_callback {
            cb(AlgoProgress::new(percent, message));
        }
    };
    let cancelled = || -> bool {
        match cancel_callback {
            Some(cb) => cb(),
            None => false,
        }
    };

    let mut soup_points: Vec<Point_3> = Vec::with_capacity(mesh.vertices.len());
    let mut soup_triangles: Vec<Vec<usize>> = Vec::with_capacity(mesh.indices.len());

    for v in mesh.vertices.iter() {
        soup_points.push([v.x as f64, v.y as f64, v.z as f64]);
    }

    for f in mesh.indices.iter() {
        // static_cast<std::size_t>(f[i])
        soup_triangles.push(vec![f[0] as i64 as usize, f[1] as i64 as usize, f[2] as i64 as usize]);
    }

    progress(30, "Repairing polygon soup");
    if cancelled() {
        return false;
    }

    pmp.repair_polygon_soup(&mut soup_points, &mut soup_triangles);

    progress(50, "Orienting polygon soup");
    if cancelled() {
        return false;
    }

    pmp.orient_polygon_soup(&mut soup_points, &mut soup_triangles);

    progress(70, "Converting to CGAL mesh");
    if cancelled() {
        return false;
    }

    let mut cgal_mesh = CGALMesh::new();
    pmp.polygon_soup_to_polygon_mesh(&soup_points, &soup_triangles, &mut cgal_mesh);

    pmp.remove_degenerate_faces(&mut cgal_mesh);

    progress(80, "Closing mesh boundaries");
    if cancelled() {
        return false;
    }

    pmp.stitch_borders(&mut cgal_mesh);
    pmp.duplicate_non_manifold_vertices(&mut cgal_mesh);
    let stats = ComputeBoundaryEdgeStats(&cgal_mesh);

    let can_repair_holes = stats.total_boundary_edges <= setting.max_boundary_edges && stats.max_cycle_edges <= setting.max_hole_edges;

    if can_repair_holes {
        CloseBoundariesAndRepairManifoldness(&mut cgal_mesh, pmp);
    }

    progress(85, "Converting from CGAL mesh");
    if cancelled() {
        return false;
    }

    let out = cgal_utils::cgal_to_trimesh(&cgal_mesh);

    *out_mesh = Some(out);
    progress(100, "Done");

    true
}
