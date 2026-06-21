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
use std::rc::Rc;
use std::sync::Arc;

/// Copy a csg range but for the meshes, only copy the pointers. If the copy
/// is made from a CSGPart compatible object, and the pointer is a shared one,
/// it will be copied with reference counting.
///
/// CSGMeshCopy.hpp:8-33
pub fn copy_csgrange_shallow(parts: &[CSGPart]) -> Vec<CSGPart> {
    let mut out = Vec::with_capacity(parts.len());

    // CSGMeshCopy.hpp:14  for (const auto &part : csgrange) {
    for part in parts {
        // CSGMeshCopy.hpp:15-17  CSGPart cpy{{}, get_operation(part), get_transform(part)};
        // The mesh pointer (its_ptr) is value-initialized to empty here.
        let mut cpy = CSGPart::from_parts(MeshPtr::None, get_operation(part), get_transform(part));

        // CSGMeshCopy.hpp:19  cpy.stack_operation = get_stack_operation(part);
        cpy.stack_operation = get_stack_operation(part);

        // CSGMeshCopy.hpp:21-25
        //   if constexpr (std::is_convertible_v<decltype(part), const CSGPart&>) {
        //       if (auto shptr = part.its_ptr.get_shared_cpy()) {
        //           cpy.its_ptr = shptr;
        //       }
        //   }
        // `get_shared_cpy()` only yields a non-empty pointer when the source
        // `AnyPtr` holds a `shared_ptr` (AnyPtr.hpp:123-130); for a raw pointer
        // or a `unique_ptr` it returns an empty shared_ptr. Both `MeshPtr::Shared`
        // (Arc) and `MeshPtr::Rc` (Rc) model a reference-counted shared pointer,
        // so both take the reference-counting copy path (refcount bump, no mesh
        // data clone), mirroring `shared_ptr`'s copy.
        match &part.mesh {
            MeshPtr::Shared(arc) => {
                cpy.mesh = MeshPtr::Shared(Arc::clone(arc));
            }
            MeshPtr::Rc(rc) => {
                cpy.mesh = MeshPtr::Rc(Rc::clone(rc));
            }
            // `MeshPtr::Owned` (unique_ptr) and `MeshPtr::None` (empty/raw):
            // `get_shared_cpy()` returns empty, so `cpy.its_ptr` stays null and
            // we fall through to the raw-pointer wrap below (CSGMeshCopy.hpp:27-28).
            MeshPtr::Owned(_) | MeshPtr::None => {}
        }

        // CSGMeshCopy.hpp:27-28
        //   if (!cpy.its_ptr)
        //       cpy.its_ptr = AnyPtr<const indexed_triangle_set>{get_mesh(part)};
        // In C++ this wraps the raw, NON-OWNING pointer returned by
        // `get_mesh(part)` (no mesh-data copy). `MeshPtr` has no borrowing
        // raw-pointer variant tied to the source lifetime, and this function
        // returns an owned `Vec<CSGPart>`, so the non-owning borrow cannot be
        // represented; the closest faithful behaviour is an owned clone of the
        // source mesh for the `Owned` case (`None` stays empty).
        // FIDELITY-NOTE: MeshPtr lacks a non-owning raw-pointer variant; C++
        // stores a borrow here, the Rust port deep-clones the Owned mesh.
        if cpy.mesh.is_empty() {
            if let Some(mesh) = get_mesh(part) {
                cpy.mesh = MeshPtr::Owned(Box::new(mesh.clone()));
            }
        }

        // CSGMeshCopy.hpp:30-31  *out = std::move(cpy); ++out;
        out.push(cpy);
    }

    out
}

/// Copy the csg range, allocating new meshes.
///
/// CSGMeshCopy.hpp:35-52
pub fn copy_csgrange_deep(parts: &[CSGPart]) -> Vec<CSGPart> {
    let mut out = Vec::with_capacity(parts.len());

    // CSGMeshCopy.hpp:39  for (const auto &part : csgrange) {
    for part in parts {
        // CSGMeshCopy.hpp:41  CSGPart cpy{{}, get_operation(part), get_transform(part)};
        let mut cpy = CSGPart::from_parts(MeshPtr::None, get_operation(part), get_transform(part));

        // CSGMeshCopy.hpp:43-45
        //   if (auto meshptr = get_mesh(part))
        //       cpy.its_ptr = std::make_unique<const indexed_triangle_set>(*meshptr);
        if let Some(meshptr) = get_mesh(part) {
            cpy.mesh = MeshPtr::Owned(Box::new(meshptr.clone()));
        }

        // CSGMeshCopy.hpp:47  cpy.stack_operation = get_stack_operation(part);
        cpy.stack_operation = get_stack_operation(part);

        // CSGMeshCopy.hpp:49-50  *out = std::move(cpy); ++out;
        out.push(cpy);
    }

    out
}

/// Check if two CSG ranges represent the same CSG expression.
///
/// Compares mesh pointers (identity), operations, stack operations,
/// and transformations (Eigen `isApprox` approximate equality).
///
/// CSGMeshCopy.hpp:54-76
pub fn is_same(a: &[CSGPart], b: &[CSGPart]) -> bool {
    // CSGMeshCopy.hpp:57  bool ret = true;
    let mut ret = true;

    // CSGMeshCopy.hpp:59  size_t s = A.size();
    let s = a.len();

    // CSGMeshCopy.hpp:61-62  if (B.size() != s) ret = false;
    if b.len() != s {
        ret = false;
    }

    // CSGMeshCopy.hpp:64-73
    //   size_t i = 0;
    //   auto itA = A.begin();
    //   auto itB = B.begin();
    //   for (; ret && i < s; ++itA, ++itB, ++i) {
    //       ret = ret && get_mesh(*itA) == get_mesh(*itB)
    //                 && get_operation(*itA) == get_operation(*itB)
    //                 && get_stack_operation(*itA) == get_stack_operation(*itB)
    //                 && get_transform(*itA).isApprox(get_transform(*itB));
    //   }
    // When the lengths differ `ret` is already false, so the loop guard
    // (`ret && i < s`) short-circuits and the body never runs, exactly as in C++.
    let mut i = 0;
    while ret && i < s {
        let part_a = &a[i];
        let part_b = &b[i];

        // CSGMeshCopy.hpp:69  get_mesh(*itA) == get_mesh(*itB) (raw pointer identity)
        let mesh_a = get_mesh(part_a).map(|m| m as *const TriangleMesh);
        let mesh_b = get_mesh(part_b).map(|m| m as *const TriangleMesh);

        ret = ret
            && mesh_a == mesh_b
            // CSGMeshCopy.hpp:70  get_operation(*itA) == get_operation(*itB)
            && get_operation(part_a) == get_operation(part_b)
            // CSGMeshCopy.hpp:71  get_stack_operation(*itA) == get_stack_operation(*itB)
            && get_stack_operation(part_a) == get_stack_operation(part_b)
            // CSGMeshCopy.hpp:72  get_transform(*itA).isApprox(get_transform(*itB))
            && get_transform(part_a).is_approx(&get_transform(part_b));

        i += 1;
    }

    // CSGMeshCopy.hpp:75  return ret;
    ret
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
