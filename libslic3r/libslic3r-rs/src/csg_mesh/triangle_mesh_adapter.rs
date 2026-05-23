//! Triangle mesh adapter for CSG operations.
//!
//! C++ Reference:
//! - CSGMesh/TriangleMeshAdapter.hpp
//!
//! Provides default overloads so that a plain TriangleMesh (or indexed_triangle_set)
//! can be used as a CSG part with an implicit Union operation.
//! In C++, these are template overloads; in Rust, we implement them as
//! conversion traits and free functions.

use super::csg_mesh::{CSGPart, CSGStackOp, CSGType, MeshPtr};
use crate::geometry::Transform3D;
use crate::triangle_mesh::TriangleMesh;

/// Get the CSG operation for a plain triangle mesh (always Union).
///
/// TriangleMeshAdapter.hpp:13-15
#[inline]
pub fn get_operation(_mesh: &TriangleMesh) -> CSGType {
    CSGType::Union
}

/// Get the stack operation for a plain triangle mesh (always Continue).
///
/// TriangleMeshAdapter.hpp:18-20
#[inline]
pub fn get_stack_operation(_mesh: &TriangleMesh) -> CSGStackOp {
    CSGStackOp::Continue
}

/// Get the transformation for a plain triangle mesh (always Identity).
///
/// TriangleMeshAdapter.hpp:28-30
#[inline]
pub fn get_transform(_mesh: &TriangleMesh) -> Transform3D {
    Transform3D::identity()
}

/// Convert a TriangleMesh into a CSGPart with implicit Union operation.
///
/// This is the Rust equivalent of the C++ template overloads that allow
/// a plain TriangleMesh to be used wherever a CSGPartT is expected.
impl From<TriangleMesh> for CSGPart {
    fn from(mesh: TriangleMesh) -> Self {
        CSGPart::from_parts(
            MeshPtr::from_owned(mesh),
            CSGType::Union,
            Transform3D::identity(),
        )
    }
}

impl From<&TriangleMesh> for CSGPart {
    fn from(mesh: &TriangleMesh) -> Self {
        CSGPart::from_parts(
            MeshPtr::from_owned(mesh.clone()),
            CSGType::Union,
            Transform3D::identity(),
        )
    }
}

/// Convert a vector of TriangleMeshes into CSGParts.
pub fn meshes_to_csg_parts(meshes: Vec<TriangleMesh>) -> Vec<CSGPart> {
    meshes.into_iter().map(CSGPart::from).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_operation() {
        let mesh = TriangleMesh::new();
        assert_eq!(get_operation(&mesh), CSGType::Union);
    }

    #[test]
    fn test_get_stack_operation() {
        let mesh = TriangleMesh::new();
        assert_eq!(get_stack_operation(&mesh), CSGStackOp::Continue);
    }

    #[test]
    fn test_get_transform() {
        let mesh = TriangleMesh::new();
        assert_eq!(get_transform(&mesh), Transform3D::identity());
    }

    #[test]
    fn test_mesh_to_csg_part() {
        let mesh = TriangleMesh::new();
        let part: CSGPart = mesh.into();
        assert_eq!(part.operation, CSGType::Union);
        assert_eq!(part.stack_operation, CSGStackOp::Continue);
    }

    #[test]
    fn test_meshes_to_csg_parts() {
        let meshes = vec![TriangleMesh::new(), TriangleMesh::new()];
        let parts = meshes_to_csg_parts(meshes);
        assert_eq!(parts.len(), 2);
        for part in &parts {
            assert_eq!(part.operation, CSGType::Union);
        }
    }
}
