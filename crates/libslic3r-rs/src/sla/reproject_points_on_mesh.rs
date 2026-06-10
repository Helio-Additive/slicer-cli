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
//! `AABBTreeIndirect::squared_distance_to_indexed_triangle_set`, which is
//! exactly what the already-ported `crate::aabb_mesh::AABBMesh::squared_distance`
//! (AABBMesh.cpp:313-323) computes. Since `crate::sla::indexed_mesh` is not yet
//! ported, we take `AABBMesh` here — the same query with identical math.

// namespace Slic3r { namespace sla {  // ReprojectPointsOnMesh.hpp:12
use crate::aabb_mesh::AABBMesh;
use crate::geometry::Vec3;
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
    mesh: &AABBMesh,
    pts: &mut Vec<PointType>,
) {
    // C++: tbb::parallel_for(size_t(0), pts.size(), [&mesh, &pts](size_t idx) {
    // ReprojectPointsOnMesh.hpp:20
    pts.par_iter_mut().for_each(|pt| {
        // C++: int junk;
        // ReprojectPointsOnMesh.hpp:21
        // C++: Vec3d new_pos;
        // ReprojectPointsOnMesh.hpp:22
        // C++: mesh.squared_distance(pos(pts[idx]), junk, new_pos);
        // ReprojectPointsOnMesh.hpp:23
        let (_sqdst, _junk, new_pos) = mesh.squared_distance(pt.pos());
        // C++: pos(pts[idx], new_pos);
        // ReprojectPointsOnMesh.hpp:24
        pt.set_pos(&new_pos);
    });
}

// BLOCKED — `inline void reproject_points_and_holes(ModelObject *object)`
// ReprojectPointsOnMesh.hpp:28-43
//
// Not portable yet because every input it touches is missing from the crate:
//   - `ModelObject` (src/model.rs, Model.hpp:344-460 port) has no
//     `sla_support_points` (Model.hpp) nor `sla_drain_holes` fields and no
//     `raw_mesh()` method (ReprojectPointsOnMesh.hpp:30-31, 35).
//   - `sla::SupportPoint` (crate::sla::support_point) is still a placeholder
//     without its `Vec3f pos` field (SupportPoint.hpp:18), so the `Pos` impl
//     for it (instantiation of ReprojectPointsOnMesh.hpp:14-15) is blocked.
//   - `sla::DrainHole` (crate::sla::hollowing) is still a placeholder without
//     its `Vec3f pos` field (Hollowing.hpp:33), likewise blocking its `Pos`
//     impl.
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
    use crate::aabb_mesh::IndexedTriangleSet;
    use crate::geometry::Point3F;

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
        let its = IndexedTriangleSet::from_parts(
            vec![
                Point3F::new(0.0, 0.0, 0.0),
                Point3F::new(10.0, 0.0, 0.0),
                Point3F::new(0.0, 10.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        let mesh = AABBMesh::new(its, false);

        let mut pts = vec![TestPoint { pos: [1.0, 1.0, 5.0] }];
        reproject_support_points(&mesh, &mut pts);

        // Closest point on the triangle to (1, 1, 5) is (1, 1, 0).
        assert!((pts[0].pos[0] - 1.0).abs() < 1e-6);
        assert!((pts[0].pos[1] - 1.0).abs() < 1e-6);
        assert!(pts[0].pos[2].abs() < 1e-6);
    }
}
