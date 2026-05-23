//! CSG mesh copy operations.
//!
//! C++ Reference:
//! - CSGMesh/CSGMeshCopy.hpp
//!
//! Provides functions to copy CSG part collections either shallowly
//! (sharing mesh pointers) or deeply (cloning mesh data), and to
//! compare two CSG ranges for equality.

use super::csg_mesh::{
    get_mesh, get_operation, get_stack_operation, get_transform, CSGPart,
    MeshPtr,
};
use crate::triangle_mesh::TriangleMesh;
use std::sync::Arc;

/// Copy a CSG range shallowly: mesh pointers are shared, not cloned.
///
/// If the source part has a shared (Arc) mesh pointer, the reference count
/// is incremented. Otherwise, we store a raw pointer reference as an Arc.
///
/// CSGMeshCopy.hpp:12-33
pub fn copy_csgrange_shallow(parts: &[CSGPart]) -> Vec<CSGPart> {
    let mut out = Vec::with_capacity(parts.len());

    for part in parts {
        let mut cpy = CSGPart::from_parts(MeshPtr::None, get_operation(part), get_transform(part));
        cpy.stack_operation = get_stack_operation(part);

        // Try to share the mesh pointer (equivalent to get_shared_cpy in C++)
        // CSGMeshCopy.hpp:21-25
        match &part.mesh {
            MeshPtr::Shared(arc) => {
                cpy.mesh = MeshPtr::Shared(Arc::clone(arc));
            }
            MeshPtr::Owned(boxed) => {
                // Create a shared reference from owned data
                let mesh_clone = (**boxed).clone();
                cpy.mesh = MeshPtr::Shared(Arc::new(mesh_clone));
            }
            MeshPtr::Rc(rc) => {
                let mesh_clone = (**rc).clone();
                cpy.mesh = MeshPtr::Shared(Arc::new(mesh_clone));
            }
            MeshPtr::None => {
                cpy.mesh = MeshPtr::None;
            }
        }

        out.push(cpy);
    }

    out
}

/// Copy a CSG range deeply: new mesh data is allocated for each part.
///
/// CSGMeshCopy.hpp:36-52
pub fn copy_csgrange_deep(parts: &[CSGPart]) -> Vec<CSGPart> {
    let mut out = Vec::with_capacity(parts.len());

    for part in parts {
        let mut cpy = CSGPart::from_parts(MeshPtr::None, get_operation(part), get_transform(part));

        // Deep clone the mesh
        // CSGMeshCopy.hpp:42-44
        if let Some(mesh) = get_mesh(part) {
            cpy.mesh = MeshPtr::Owned(Box::new(mesh.clone()));
        }

        cpy.stack_operation = get_stack_operation(part);
        out.push(cpy);
    }

    out
}

/// Check if two CSG ranges represent the same CSG expression.
///
/// Compares mesh pointers (identity), operations, stack operations,
/// and transformations (approximate equality).
///
/// CSGMeshCopy.hpp:54-76
pub fn is_same(a: &[CSGPart], b: &[CSGPart]) -> bool {
    if a.len() != b.len() {
        return false;
    }

    for (part_a, part_b) in a.iter().zip(b.iter()) {
        // Compare mesh pointers (identity check)
        // CSGMeshCopy.hpp:69
        let mesh_a = get_mesh(part_a).map(|m| m as *const TriangleMesh);
        let mesh_b = get_mesh(part_b).map(|m| m as *const TriangleMesh);
        if mesh_a != mesh_b {
            return false;
        }

        // Compare operations
        // CSGMeshCopy.hpp:70
        if get_operation(part_a) != get_operation(part_b) {
            return false;
        }

        // Compare stack operations
        // CSGMeshCopy.hpp:71
        if get_stack_operation(part_a) != get_stack_operation(part_b) {
            return false;
        }

        // Compare transformations (approximate)
        // CSGMeshCopy.hpp:72
        if get_transform(part_a) != get_transform(part_b) {
            return false;
        }
    }

    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_copy_shallow_empty() {
        let parts: Vec<CSGPart> = vec![];
        let copy = copy_csgrange_shallow(&parts);
        assert!(copy.is_empty());
    }

    #[test]
    fn test_copy_shallow_preserves_operations() {
        let parts = vec![
            CSGPart::new().with_operation(CSGType::Union),
            CSGPart::new().with_operation(CSGType::Difference),
        ];
        let copy = copy_csgrange_shallow(&parts);
        assert_eq!(copy.len(), 2);
        assert_eq!(get_operation(&copy[0]), CSGType::Union);
        assert_eq!(get_operation(&copy[1]), CSGType::Difference);
    }

    #[test]
    fn test_copy_deep_clones_mesh() {
        let mesh = TriangleMesh::new();
        let part = CSGPart::from_mesh(MeshPtr::from_owned(mesh));
        let parts = vec![part];
        let copy = copy_csgrange_deep(&parts);
        assert_eq!(copy.len(), 1);
        // Deep copy: different pointers
        let orig_ptr = get_mesh(&parts[0]).map(|m| m as *const _);
        let copy_ptr = get_mesh(&copy[0]).map(|m| m as *const _);
        assert_ne!(orig_ptr, copy_ptr);
    }

    #[test]
    fn test_is_same_empty() {
        let a: Vec<CSGPart> = vec![];
        let b: Vec<CSGPart> = vec![];
        assert!(is_same(&a, &b));
    }

    #[test]
    fn test_is_same_different_lengths() {
        let a = vec![CSGPart::new()];
        let b: Vec<CSGPart> = vec![];
        assert!(!is_same(&a, &b));
    }

    #[test]
    fn test_is_same_different_operations() {
        let a = vec![CSGPart::new().with_operation(CSGType::Union)];
        let b = vec![CSGPart::new().with_operation(CSGType::Difference)];
        assert!(!is_same(&a, &b));
    }

    #[test]
    fn test_is_same_identical() {
        let mesh = Arc::new(TriangleMesh::new());
        let a = vec![CSGPart::from_mesh(MeshPtr::from_arc(mesh.clone()))];
        let b = vec![CSGPart::from_mesh(MeshPtr::from_arc(mesh.clone()))];
        assert!(is_same(&a, &b));
    }
}
