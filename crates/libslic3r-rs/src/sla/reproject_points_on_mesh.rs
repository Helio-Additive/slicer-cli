//! Reprojection of SLA support points and drain holes onto a mesh surface.
//!
//! C++ Reference:
//! - SLA/ReprojectPointsOnMesh.hpp (header-only; there is no .cpp)
//!
//! Faithful 1:1 port. The C++ header pulls in:
//!   ReprojectPointsOnMesh.hpp:4  #include "libslic3r/Point.hpp"
//!   ReprojectPointsOnMesh.hpp:5  #include "SupportPoint.hpp"
//!   ReprojectPointsOnMesh.hpp:6  #include "Hollowing.hpp"
//!   ReprojectPointsOnMesh.hpp:7  #include "IndexedMesh.hpp"
//!   ReprojectPointsOnMesh.hpp:8  #include "libslic3r/Model.hpp"
//!   ReprojectPointsOnMesh.hpp:10 #include <tbb/parallel_for.h>
//!
//! NOTE on the mesh type: the C++ function takes `const sla::IndexedMesh &`.
//! `sla::IndexedMesh::squared_distance(const Vec3d &p, int &i, Vec3d &c)`
//! (IndexedMesh.cpp:308-323) delegates to
//! `AABBTreeIndirect::squared_distance_to_indexed_triangle_set`. The crate now
//! ports `sla::IndexedMesh` (crate::sla::indexed_mesh::IndexedMesh) with the
//! identical out-parameter signature, so this port takes that type directly,
//! matching the C++ header exactly.

// namespace Slic3r { namespace sla {  // ReprojectPointsOnMesh.hpp:12
use crate::geometry::Vec3;
use crate::sla::indexed_mesh::{IndexedMesh, Vec3d};
use rayon::prelude::*;

/// Accessor pair for point-like types carrying a `pos` member (`Vec3f pos`).
///
/// In C++ these are two overloaded free-function templates, instantiated for
/// `sla::SupportPoint` (SupportPoint.hpp:18) and `sla::DrainHole`
/// (Hollowing.hpp:33). Rust expresses the same duck-typed getter/setter pair
/// as a trait bound on the point type.
///
/// ReprojectPointsOnMesh.hpp:14-15
pub trait Pos {
    /// C++: `template<class Pt> Vec3d pos(const Pt &p) { return p.pos.template cast<double>(); }`
    /// ReprojectPointsOnMesh.hpp:14
    fn pos(&self) -> Vec3;

    /// C++: `template<class Pt> void pos(Pt &p, const Vec3d &pp) { p.pos = pp.cast<float>(); }`
    ///
    /// Implementations must mirror the C++ narrowing cast: store the `f64`
    /// coordinates back into the point's `f32` `pos` member (`pp.cast<float>()`).
    /// ReprojectPointsOnMesh.hpp:15
    fn set_pos(&mut self, pp: &Vec3);
}

/// Project every point of `pts` onto the closest point of `mesh`.
///
/// C++: `template<class PointType>`
/// C++: `void reproject_support_points(const IndexedMesh &mesh, std::vector<PointType> &pts)`
/// ReprojectPointsOnMesh.hpp:17-26
pub fn reproject_support_points<PointType: Pos + Send>(
    mesh: &IndexedMesh,
    pts: &mut Vec<PointType>,
) {
    // C++: tbb::parallel_for(size_t(0), pts.size(), [&mesh, &pts](size_t idx) {
    // ReprojectPointsOnMesh.hpp:20
    pts.par_iter_mut().for_each(|pt| {
        // C++: int junk;
        // ReprojectPointsOnMesh.hpp:21
        let mut junk: i32 = 0;
        // C++: Vec3d new_pos;
        // ReprojectPointsOnMesh.hpp:22
        let mut new_pos = Vec3d::zeros();
        // C++: mesh.squared_distance(pos(pts[idx]), junk, new_pos);
        // ReprojectPointsOnMesh.hpp:23
        //
        // `IndexedMesh::squared_distance` is defined over nalgebra `Vec3d`
        // (Eigen `Matrix<double,1,3>`), while the `Pos` accessors return the
        // crate's geometry `Vec3`. Both losslessly represent the same Eigen
        // `Vec3d`; bridge them coordinate-wise (no precision change).
        let p = pt.pos();
        let p_na = Vec3d::new(p.x, p.y, p.z);
        mesh.squared_distance(&p_na, &mut junk, &mut new_pos);
        // C++: pos(pts[idx], new_pos);
        // ReprojectPointsOnMesh.hpp:24
        pt.set_pos(&Vec3::new(new_pos.x, new_pos.y, new_pos.z));
    });
}

// BLOCKED — `inline void reproject_points_and_holes(ModelObject *object)`
// ReprojectPointsOnMesh.hpp:28-43
//
// Not portable yet because the `ModelObject` inputs it touches are missing from
// the crate's (simplified) `ModelObject` (src/model.rs, Model.hpp:344-460 port):
//   - `ModelObject` has no `sla_support_points` (Model.hpp) nor `sla_drain_holes`
//     fields (ReprojectPointsOnMesh.hpp:30-31).
//   - `ModelObject` has no `raw_mesh()` method (ReprojectPointsOnMesh.hpp:35); the
//     crate stores `mesh: TriangleMesh` directly instead of a volume list.
//   - The crate's `TriangleMesh` is a documented divergent struct (f64 vertices,
//     no `its` member — see triangle_mesh.rs "DIVERGENCE" note), so even the
//     `IndexedMesh emesh{rmsh}` step (ReprojectPointsOnMesh.hpp:36) has no
//     faithful equivalent until `IndexedMesh::from_triangle_mesh` lands
//     (see indexed_mesh.rs BLOCKED note on `IndexedMesh(const TriangleMesh&)`).
//
// For reference, the C++ body to port once those land:
//   ReprojectPointsOnMesh.hpp:30  bool has_sppoints = !object->sla_support_points.empty();
//   ReprojectPointsOnMesh.hpp:31  bool has_holes    = !object->sla_drain_holes.empty();
//   ReprojectPointsOnMesh.hpp:33  if (!object || (!has_holes && !has_sppoints)) return;
//   ReprojectPointsOnMesh.hpp:35  TriangleMesh rmsh = object->raw_mesh();
//   ReprojectPointsOnMesh.hpp:36  IndexedMesh emesh{rmsh};
//   ReprojectPointsOnMesh.hpp:38-39  if (has_sppoints) reproject_support_points(emesh, object->sla_support_points);
//   ReprojectPointsOnMesh.hpp:41-42  if (has_holes)    reproject_support_points(emesh, object->sla_drain_holes);

// }} // namespace Slic3r::sla  // ReprojectPointsOnMesh.hpp:45

#[cfg(test)]
mod tests {
    use super::*;
    use crate::normal_utils::indexed_triangle_set;
    use crate::triangle_mesh::{Vec3f, Vec3i};

    /// Minimal stand-in for a point type with a `Vec3f pos` member, mirroring
    /// the layout of sla::SupportPoint (SupportPoint.hpp:18).
    struct TestPoint {
        pos: [f32; 3],
    }

    impl Pos for TestPoint {
        // ReprojectPointsOnMesh.hpp:14 — p.pos.template cast<double>()
        fn pos(&self) -> Vec3 {
            Vec3::new(self.pos[0] as f64, self.pos[1] as f64, self.pos[2] as f64)
        }

        // ReprojectPointsOnMesh.hpp:15 — p.pos = pp.cast<float>()
        fn set_pos(&mut self, pp: &Vec3) {
            self.pos = [pp.x as f32, pp.y as f32, pp.z as f32];
        }
    }

    #[test]
    fn reprojects_point_onto_triangle() {
        // One triangle in the z = 0 plane.
        let its = indexed_triangle_set {
            vertices: vec![
                Vec3f::new(0.0, 0.0, 0.0),
                Vec3f::new(10.0, 0.0, 0.0),
                Vec3f::new(0.0, 10.0, 0.0),
            ],
            indices: vec![Vec3i::new(0, 1, 2)],
        };
        let mesh = IndexedMesh::new(&its, false);

        let mut pts = vec![TestPoint { pos: [1.0, 1.0, 5.0] }];
        reproject_support_points(&mesh, &mut pts);

        // Closest point on the triangle to (1, 1, 5) is (1, 1, 0).
        assert!((pts[0].pos[0] - 1.0).abs() < 1e-6);
        assert!((pts[0].pos[1] - 1.0).abs() < 1e-6);
        assert!(pts[0].pos[2].abs() < 1e-6);
    }
}
