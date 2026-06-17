//! Triangle mesh adapter for CSG operations.
//!
//! C++ Reference:
//! - CSGMesh/TriangleMeshAdapter.hpp
//!
//! Provides default overloads so that a plain `indexed_triangle_set` or a
//! `TriangleMesh` can be used as a `CSGPart` with an implicit `Union`
//! operation, a `Continue` stack operation, and an `Identity` transform.
//!
//! In C++ these are free-function overloads selected by argument type. The
//! header declares four accessors (`get_operation`, `get_stack_operation`,
//! `get_mesh`, `get_transform`) for each of four argument forms:
//!   * `const indexed_triangle_set &`            (TriangleMeshAdapter.hpp:13-31)
//!   * `const indexed_triangle_set * const`      (TriangleMeshAdapter.hpp:33-51)
//!   * `const TriangleMesh &`                    (TriangleMeshAdapter.hpp:53-71)
//!   * `const TriangleMesh * const`              (TriangleMeshAdapter.hpp:73-91)
//!
//! In Rust a shared reference `&T` already covers both the `const T&` and the
//! `const T * const` C++ overload families (a borrow is the idiomatic spelling
//! of "non-owning const access"), so the by-pointer overloads collapse onto the
//! by-reference forms below — they carry no distinct logic in the C++ source
//! (each pointer overload simply forwards the same body as its reference twin).

use super::csg_mesh::{CSGPart, CSGStackOp, CSGType, MeshPtr};
use crate::geometry::Transform3D;
use crate::normal_utils::{indexed_triangle_set, StlTriangleVertexIndices, StlVertex};
use crate::triangle_mesh::TriangleMesh;

// --- indexed_triangle_set overloads -------------------------------------------------

/// Get the CSG operation for a plain `indexed_triangle_set` (always Union).
///
/// TriangleMeshAdapter.hpp:13-16
/// TriangleMeshAdapter.hpp:33-36 (the `* const` overload, identical body)
#[inline]
pub fn get_operation_its(_part: &indexed_triangle_set) -> CSGType {
    CSGType::Union
}

/// Get the stack operation for a plain `indexed_triangle_set` (always Continue).
///
/// TriangleMeshAdapter.hpp:18-21
/// TriangleMeshAdapter.hpp:38-41 (the `* const` overload, identical body)
#[inline]
pub fn get_stack_operation_its(_part: &indexed_triangle_set) -> CSGStackOp {
    CSGStackOp::Continue
}

/// Get the mesh for a plain `indexed_triangle_set` (the part itself).
///
/// C++: `inline const indexed_triangle_set * get_mesh(const indexed_triangle_set &part) { return &part; }`
///
/// TriangleMeshAdapter.hpp:23-26
/// TriangleMeshAdapter.hpp:43-46 (the `* const` overload returns `part`)
#[inline]
pub fn get_mesh_its(part: &indexed_triangle_set) -> &indexed_triangle_set {
    part
}

/// Get the transformation for a plain `indexed_triangle_set` (always Identity).
///
/// TriangleMeshAdapter.hpp:28-31
/// TriangleMeshAdapter.hpp:48-51 (the `* const` overload, identical body)
#[inline]
pub fn get_transform_its(_part: &indexed_triangle_set) -> Transform3D {
    Transform3D::identity()
}

// --- TriangleMesh overloads ---------------------------------------------------------

/// Get the CSG operation for a plain `TriangleMesh` (always Union).
///
/// TriangleMeshAdapter.hpp:53-56
/// TriangleMeshAdapter.hpp:73-76 (the `* const` overload, identical body)
#[inline]
pub fn get_operation(_part: &TriangleMesh) -> CSGType {
    CSGType::Union
}

/// Get the stack operation for a plain `TriangleMesh` (always Continue).
///
/// TriangleMeshAdapter.hpp:58-61
/// TriangleMeshAdapter.hpp:78-81 (the `* const` overload, identical body)
#[inline]
pub fn get_stack_operation(_part: &TriangleMesh) -> CSGStackOp {
    CSGStackOp::Continue
}

/// Get the mesh (indexed triangle set) for a plain `TriangleMesh`.
///
/// C++: `inline const indexed_triangle_set * get_mesh(const TriangleMesh &part) { return &part.its; }`
///
/// TriangleMeshAdapter.hpp:63-66
/// TriangleMeshAdapter.hpp:83-86 (the `* const` overload returns `&part->its`)
///
/// FIDELITY-NOTE: In C++ `TriangleMesh::its` is an embedded `indexed_triangle_set`
/// member, so `get_mesh` returns a *borrow* into the mesh at zero cost. The crate's
/// `TriangleMesh` (triangle_mesh.rs:2041) instead stores `vertices`/`indices`
/// directly and has no embedded `indexed_triangle_set`, so a borrow is impossible.
/// We therefore reconstruct the `indexed_triangle_set` by value here — same data,
/// same field meaning — mirroring the existing `triangle_mesh_to_its` helper in
/// model_to_csg_mesh.rs:91. Adding a true embedded `its` member to `TriangleMesh`
/// is a cross-cutting structural change, out of per-file scope.
#[inline]
pub fn get_mesh(part: &TriangleMesh) -> indexed_triangle_set {
    let mut its = indexed_triangle_set::default();
    its.vertices.reserve(part.vertices().len());
    for v in part.vertices() {
        its.vertices
            .push(StlVertex::new(v.x as f32, v.y as f32, v.z as f32));
    }
    its.indices.reserve(part.indices().len());
    for tri in part.indices() {
        its.indices.push(StlTriangleVertexIndices::new(
            tri.indices[0] as i32,
            tri.indices[1] as i32,
            tri.indices[2] as i32,
        ));
    }
    its
}

/// Get the transformation for a plain `TriangleMesh` (always Identity).
///
/// TriangleMeshAdapter.hpp:68-71
/// TriangleMeshAdapter.hpp:88-91 (the `* const` overload, identical body)
#[inline]
pub fn get_transform(_part: &TriangleMesh) -> Transform3D {
    Transform3D::identity()
}

// --- Rust-only conveniences (no C++ counterpart) ------------------------------------

/// Convert a `TriangleMesh` into a `CSGPart` with implicit Union operation.
///
/// This is the Rust equivalent of the C++ template overloads that allow a plain
/// `TriangleMesh` to be used wherever a `CSGPartT` is expected. It composes the
/// adapter accessors above (`get_operation`, `get_stack_operation`,
/// `get_transform`), all of which return the implicit defaults.
impl From<TriangleMesh> for CSGPart {
    fn from(mesh: TriangleMesh) -> Self {
        let operation = get_operation(&mesh);
        let transform = get_transform(&mesh);
        CSGPart::from_parts(MeshPtr::from_owned(mesh), operation, transform)
    }
}

impl From<&TriangleMesh> for CSGPart {
    fn from(mesh: &TriangleMesh) -> Self {
        let operation = get_operation(mesh);
        let transform = get_transform(mesh);
        CSGPart::from_parts(MeshPtr::from_owned(mesh.clone()), operation, transform)
    }
}

/// Convert a vector of `TriangleMesh`es into `CSGPart`s.
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
    fn test_its_overloads() {
        let its = indexed_triangle_set::default();
        assert_eq!(get_operation_its(&its), CSGType::Union);
        assert_eq!(get_stack_operation_its(&its), CSGStackOp::Continue);
        assert_eq!(get_transform_its(&its), Transform3D::identity());
        // get_mesh on an ITS returns the part itself.
        assert!(std::ptr::eq(get_mesh_its(&its), &its));
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
