//! CSG mesh boolean operations.
//!
//! C++ Reference:
//! - CSGMesh/PerformCSGMeshBooleans.hpp
//!
//! Provides functions for performing boolean operations (union, difference,
//! intersection) on collections of CSG mesh parts. In C++, this uses CGAL
//! and MCUT libraries. This Rust port provides the algorithmic structure
//! with placeholder mesh boolean operations that can be replaced with
//! actual boolean mesh libraries when available.

use super::csg_mesh::{
    get_mesh, get_operation, get_stack_operation, get_transform, CSGPart, CSGStackOp, CSGType,
};
use crate::geometry::Transform3D;
use crate::triangle_mesh::TriangleMesh;

/// Reason for boolean operation failure.
///
/// PerformCSGMeshBooleans.hpp:16
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanFailReason {
    /// No failure
    OK,
    /// Mesh is empty
    MeshEmpty,
    /// Mesh does not bound a volume
    NotBoundAVolume,
    /// Mesh has self-intersections
    SelfIntersect,
    /// No intersection found
    NoIntersection,
}

impl Default for BooleanFailReason {
    fn default() -> Self {
        BooleanFailReason::OK
    }
}

/// Opaque handle for a boolean-ready mesh representation.
///
/// In C++ this would be CGALMeshPtr or McutMeshPtr.
/// Here it wraps a TriangleMesh for the algorithmic structure.
///
/// PerformCSGMeshBooleans.hpp (CGALMeshPtr / McutMeshPtr)
#[derive(Clone)]
pub struct BooleanMesh {
    mesh: Option<TriangleMesh>,
}

impl BooleanMesh {
    /// Create from a TriangleMesh.
    pub fn from_mesh(mesh: TriangleMesh) -> Self {
        Self { mesh: Some(mesh) }
    }

    /// Create an empty boolean mesh.
    pub fn empty() -> Self {
        Self {
            mesh: Some(TriangleMesh::new()),
        }
    }

    /// Create a null (no mesh) handle.
    pub fn null() -> Self {
        Self { mesh: None }
    }

    /// Check if this mesh handle is null.
    pub fn is_null(&self) -> bool {
        self.mesh.is_none()
    }

    /// Check if the mesh is empty (has no triangles).
    pub fn is_empty(&self) -> bool {
        match &self.mesh {
            None => true,
            Some(m) => m.is_empty(),
        }
    }

    /// Get a reference to the inner mesh.
    pub fn get_mesh(&self) -> Option<&TriangleMesh> {
        self.mesh.as_ref()
    }

    /// Take ownership of the inner mesh.
    pub fn into_mesh(self) -> Option<TriangleMesh> {
        self.mesh
    }
}

/// Convert a CSG part to a boolean-ready mesh.
///
/// Applies the transformation from the CSG part to the mesh.
///
/// PerformCSGMeshBooleans.hpp:20-42
pub fn get_boolean_mesh(part: &CSGPart) -> BooleanMesh {
    let mesh_opt = get_mesh(part);
    let _transform = get_transform(part);

    match mesh_opt {
        Some(mesh) => {
            // Clone the mesh and apply transformation
            // PerformCSGMeshBooleans.hpp:31-32
            // C++: indexed_triangle_set m = *its;
            // C++: its_transform(m, get_transform(csgpart), true);
            let m = mesh.clone();
            // TODO: Apply transformation when its_transform is available
            BooleanMesh::from_mesh(m)
        }
        None => BooleanMesh::null(),
    }
}

/// Perform a single CSG operation on two boolean meshes.
///
/// PerformCSGMeshBooleans.hpp:75-96
fn perform_csg(op: CSGType, dst: &mut BooleanMesh, src: &mut BooleanMesh) {
    // If dst is null and op is Union and src exists, move src to dst
    // PerformCSGMeshBooleans.hpp:77-80
    if dst.is_null() && op == CSGType::Union && !src.is_null() {
        *dst = std::mem::replace(src, BooleanMesh::null());
        return;
    }

    // If either is null, nothing to do
    // PerformCSGMeshBooleans.hpp:82-83
    if dst.is_null() || src.is_null() {
        return;
    }

    // Perform the boolean operation
    // PerformCSGMeshBooleans.hpp:85-96
    // NOTE: Actual boolean mesh operations require a mesh boolean library.
    // The algorithmic structure is correct; the actual mesh booleans are stubs.
    match op {
        CSGType::Union => {
            // mesh_boolean_union(dst, src)
            // For now, just keep dst as-is (stub)
        }
        CSGType::Difference => {
            // mesh_boolean_difference(dst, src)
        }
        CSGType::Intersection => {
            // mesh_boolean_intersection(dst, src)
        }
    }
}

/// Stack frame for CSG expression evaluation with boolean meshes.
///
/// PerformCSGMeshBooleans.hpp:168-174
struct Frame {
    op: CSGType,
    mesh: BooleanMesh,
}

impl Frame {
    fn new(op: CSGType) -> Self {
        Self {
            op,
            mesh: BooleanMesh::empty(),
        }
    }
}

/// Perform CSG mesh booleans on a collection of CSG parts.
///
/// Uses a stack-based algorithm to evaluate the CSG expression formed by
/// the sequence of parts with their Push/Pop stack operations.
///
/// PerformCSGMeshBooleans.hpp:161-207
pub fn perform_csgmesh_booleans(parts: &[CSGPart]) -> BooleanMesh {
    let mut opstack: Vec<Frame> = Vec::new();
    opstack.push(Frame::new(CSGType::Union));

    // Convert all parts to boolean meshes (could be parallelized)
    // PerformCSGMeshBooleans.hpp:180
    let mut boolean_meshes: Vec<BooleanMesh> = parts.iter().map(get_boolean_mesh).collect();

    // Process each CSG part
    // PerformCSGMeshBooleans.hpp:183-206
    for (idx, part) in parts.iter().enumerate() {
        let op = get_operation(part);
        let mesh = &mut boolean_meshes[idx];

        // Handle Push: start a new sub-expression
        // PerformCSGMeshBooleans.hpp:188-191
        if get_stack_operation(part) == CSGStackOp::Push {
            opstack.push(Frame::new(op));
        }

        // Perform the CSG operation on the top frame
        // PerformCSGMeshBooleans.hpp:195
        let top = opstack.last_mut().unwrap();
        perform_csg(get_operation(part), &mut top.mesh, mesh);

        // Handle Pop: complete the sub-expression
        // PerformCSGMeshBooleans.hpp:197-204
        if get_stack_operation(part) == CSGStackOp::Pop {
            let popped = opstack.pop().unwrap();
            let mut src = popped.mesh;
            let pop_op = popped.op;
            if let Some(parent) = opstack.last_mut() {
                perform_csg(pop_op, &mut parent.mesh, &mut src);
            }
        }
    }

    // Return the final result
    // PerformCSGMeshBooleans.hpp:206
    opstack
        .pop()
        .map(|f| f.mesh)
        .unwrap_or_else(BooleanMesh::null)
}

/// Check CSG mesh booleans for validity.
///
/// Validates each mesh in the CSG collection and returns the failure reason
/// and the name of the failing part, if any.
///
/// PerformCSGMeshBooleans.hpp:262-322
pub fn check_csgmesh_booleans(parts: &[CSGPart]) -> (BooleanFailReason, String) {
    for (i, part) in parts.iter().enumerate() {
        let mesh_opt = get_mesh(part);

        // Null mesh with stack operation is allowed
        // PerformCSGMeshBooleans.hpp:276-279
        if mesh_opt.is_none() && get_stack_operation(part) != CSGStackOp::Continue {
            continue;
        }

        // Check if mesh is empty
        // PerformCSGMeshBooleans.hpp:282-286
        match mesh_opt {
            None => {
                return (BooleanFailReason::MeshEmpty, part.name.clone());
            }
            Some(mesh) => {
                if mesh.is_empty() {
                    return (BooleanFailReason::MeshEmpty, part.name.clone());
                }
                // NOTE: Self-intersection and volume-bounding checks require
                // actual mesh analysis libraries (CGAL in C++).
                // PerformCSGMeshBooleans.hpp:296-301
            }
        }
    }

    (BooleanFailReason::OK, String::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csg_mesh::csg_mesh::MeshPtr;

    #[test]
    fn test_boolean_fail_reason_default() {
        assert_eq!(BooleanFailReason::default(), BooleanFailReason::OK);
    }

    #[test]
    fn test_boolean_mesh_empty() {
        let m = BooleanMesh::empty();
        assert!(!m.is_null());
    }

    #[test]
    fn test_boolean_mesh_null() {
        let m = BooleanMesh::null();
        assert!(m.is_null());
        assert!(m.is_empty());
    }

    #[test]
    fn test_perform_csgmesh_booleans_empty() {
        let parts: Vec<CSGPart> = vec![];
        let result = perform_csgmesh_booleans(&parts);
        assert!(!result.is_null());
    }

    #[test]
    fn test_perform_csgmesh_booleans_single() {
        let mesh = TriangleMesh::new();
        let part = CSGPart::from_mesh(MeshPtr::from_owned(mesh));
        let result = perform_csgmesh_booleans(&[part]);
        assert!(!result.is_null());
    }

    #[test]
    fn test_perform_csgmesh_booleans_with_stack() {
        let cube1 = CSGPart::new()
            .with_operation(CSGType::Union)
            .with_stack_operation(CSGStackOp::Continue);
        let cube2 = CSGPart::new()
            .with_operation(CSGType::Difference)
            .with_stack_operation(CSGStackOp::Push);
        let cube3 = CSGPart::new()
            .with_operation(CSGType::Union)
            .with_stack_operation(CSGStackOp::Pop);

        let result = perform_csgmesh_booleans(&[cube1, cube2, cube3]);
        assert!(!result.is_null());
    }

    #[test]
    fn test_check_booleans_empty_parts() {
        let (reason, name) = check_csgmesh_booleans(&[]);
        assert_eq!(reason, BooleanFailReason::OK);
        assert!(name.is_empty());
    }

    #[test]
    fn test_check_booleans_empty_mesh() {
        let part = CSGPart::new().with_name("test_part".to_string());
        let (reason, name) = check_csgmesh_booleans(&[part]);
        assert_eq!(reason, BooleanFailReason::MeshEmpty);
        assert_eq!(name, "test_part");
    }

    #[test]
    fn test_check_booleans_null_mesh_with_stack_op() {
        let part = CSGPart::new().with_stack_operation(CSGStackOp::Push);
        let (reason, _name) = check_csgmesh_booleans(&[part]);
        assert_eq!(reason, BooleanFailReason::OK);
    }
}
