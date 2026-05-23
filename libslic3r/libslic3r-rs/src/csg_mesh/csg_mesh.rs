//! CSG (Constructive Solid Geometry) mesh representation and operations.
//!
//! This module provides types and functions for representing CSG operations on meshes.
//! A collection of CSGPart objects can be interpreted as one model and used in various
//! contexts (assembled with CGAL or OpenVDB, rendered with OpenCSG, or provided to ray-tracers).
//!
//! C++ Reference:
//! - CSGMesh/CSGMesh.hpp
//!
//! Key Concepts:
//! - CSGPart: A mesh + transformation + CSG operation
//! - CSGType: Union, Difference, or Intersection
//! - CSGStackOp: Push/Pop operations for expression evaluation (parentheses)
//!
//! Example CSG Expression:
//! ```text
//! CUBE1 - (CUBE2 + CUBE3)
//! ```
//! Represented as:
//! ```text
//! [
//!   CSGPart { mesh: cube1, op: Union,      stack_op: Continue },
//!   CSGPart { mesh: cube2, op: Difference, stack_op: Push },
//!   CSGPart { mesh: cube3, op: Union,      stack_op: Pop },
//! ]
//! ```

use crate::geometry::Transform3D;
use crate::triangle_mesh::TriangleMesh;
use std::rc::Rc;
use std::sync::Arc;

/// Supported CSG operation types
/// CSGMesh.hpp:18
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CSGType {
    /// Union operation (A ∪ B) - combines two meshes
    /// CSGMesh.hpp:18
    Union,

    /// Difference operation (A - B) - subtracts B from A
    /// CSGMesh.hpp:18
    Difference,

    /// Intersection operation (A ∩ B) - only the overlapping volume
    /// CSGMesh.hpp:18
    Intersection,
}

impl Default for CSGType {
    /// Default CSG operation is Union
    /// CSGMesh.hpp:60
    fn default() -> Self {
        CSGType::Union
    }
}

/// Stack operation for CSG expression evaluation.
///
/// A CSG part can instruct the processing to push the sub-result onto a stack
/// until a new CSG part with a pop instruction appears. This implements
/// parentheses in a CSG expression represented by a collection of CSG parts.
///
/// When a CSG part contains a Push instruction, the CSG operation it contains
/// refers to the whole collection spanning to the nearest part with a Pop instruction.
///
/// CSGMesh.hpp:25-37
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CSGStackOp {
    /// Push the current result onto the stack
    /// CSGMesh.hpp:25
    Push,

    /// Continue without stack operations (default)
    /// CSGMesh.hpp:25
    Continue,

    /// Pop from the stack and apply the operation
    /// CSGMesh.hpp:25
    Pop,
}

impl Default for CSGStackOp {
    /// Default stack operation is Continue
    /// CSGMesh.hpp:64
    fn default() -> Self {
        CSGStackOp::Continue
    }
}

/// A pointer type that can hold either owned or borrowed mesh data.
///
/// This is the Rust equivalent of C++'s AnyPtr<const indexed_triangle_set>.
/// It supports multiple ownership patterns to work with different contexts.
///
/// CSGMesh.hpp:9 (AnyPtr)
#[derive(Debug, Clone)]
pub enum MeshPtr {
    /// No mesh (empty/null)
    None,

    /// Owned mesh data
    Owned(Box<TriangleMesh>),

    /// Reference-counted shared mesh (thread-safe)
    Shared(Arc<TriangleMesh>),

    /// Reference-counted shared mesh (single-threaded)
    Rc(Rc<TriangleMesh>),
}

impl MeshPtr {
    /// Create an empty MeshPtr
    pub fn new() -> Self {
        MeshPtr::None
    }

    /// Create a MeshPtr from an owned mesh
    pub fn from_owned(mesh: TriangleMesh) -> Self {
        MeshPtr::Owned(Box::new(mesh))
    }

    /// Create a MeshPtr from a boxed mesh
    pub fn from_box(mesh: Box<TriangleMesh>) -> Self {
        MeshPtr::Owned(mesh)
    }

    /// Create a MeshPtr from an Arc
    pub fn from_arc(mesh: Arc<TriangleMesh>) -> Self {
        MeshPtr::Shared(mesh)
    }

    /// Create a MeshPtr from an Rc
    pub fn from_rc(mesh: Rc<TriangleMesh>) -> Self {
        MeshPtr::Rc(mesh)
    }

    /// Get a reference to the mesh, if available
    pub fn get(&self) -> Option<&TriangleMesh> {
        match self {
            MeshPtr::None => None,
            MeshPtr::Owned(mesh) => Some(mesh.as_ref()),
            MeshPtr::Shared(mesh) => Some(mesh.as_ref()),
            MeshPtr::Rc(mesh) => Some(mesh.as_ref()),
        }
    }

    /// Check if the pointer is empty (None)
    pub fn is_empty(&self) -> bool {
        matches!(self, MeshPtr::None)
    }
}

impl Default for MeshPtr {
    fn default() -> Self {
        MeshPtr::None
    }
}

impl From<TriangleMesh> for MeshPtr {
    fn from(mesh: TriangleMesh) -> Self {
        MeshPtr::from_owned(mesh)
    }
}

impl From<Box<TriangleMesh>> for MeshPtr {
    fn from(mesh: Box<TriangleMesh>) -> Self {
        MeshPtr::from_box(mesh)
    }
}

impl From<Arc<TriangleMesh>> for MeshPtr {
    fn from(mesh: Arc<TriangleMesh>) -> Self {
        MeshPtr::from_arc(mesh)
    }
}

impl From<Rc<TriangleMesh>> for MeshPtr {
    fn from(mesh: Rc<TriangleMesh>) -> Self {
        MeshPtr::from_rc(mesh)
    }
}

/// A CSG part: mesh + transformation + CSG operation.
///
/// Default implementation of a CSGPartT object that implements the necessary
/// interface to be usable in CSG contexts. A CSG part cannot contain another
/// CSG collection, only a mesh - this is why stack operations are used instead
/// of recursion in the data definition.
///
/// CSGMesh.hpp:57-68
#[derive(Debug, Clone)]
pub struct CSGPart {
    /// Pointer to the indexed triangle set (mesh)
    /// CSGMesh.hpp:58
    pub mesh: MeshPtr,

    /// Transformation matrix associated with the mesh
    /// CSGMesh.hpp:59
    pub transform: Transform3D,

    /// CSG operation type (Union, Difference, Intersection)
    /// CSGMesh.hpp:60
    pub operation: CSGType,

    /// Stack operation for expression evaluation
    /// CSGMesh.hpp:61
    pub stack_operation: CSGStackOp,

    /// Optional name for debugging/identification
    /// CSGMesh.hpp:62
    pub name: String,
}

impl CSGPart {
    /// Create a new CSG part with default values
    /// CSGMesh.hpp:64-68
    pub fn new() -> Self {
        Self {
            mesh: MeshPtr::None,
            transform: Transform3D::identity(),
            operation: CSGType::Union,
            stack_operation: CSGStackOp::Continue,
            name: String::new(),
        }
    }

    /// Create a CSG part from a mesh pointer
    /// CSGMesh.hpp:64-68
    pub fn from_mesh(mesh: MeshPtr) -> Self {
        Self {
            mesh,
            transform: Transform3D::identity(),
            operation: CSGType::Union,
            stack_operation: CSGStackOp::Continue,
            name: String::new(),
        }
    }

    /// Create a CSG part with mesh, operation, and transform
    /// CSGMesh.hpp:64-68
    pub fn from_parts(mesh: MeshPtr, operation: CSGType, transform: Transform3D) -> Self {
        Self {
            mesh,
            transform,
            operation,
            stack_operation: CSGStackOp::Continue,
            name: String::new(),
        }
    }

    /// Create a CSG part with all fields specified
    pub fn with_all_fields(
        mesh: MeshPtr,
        operation: CSGType,
        transform: Transform3D,
        stack_operation: CSGStackOp,
        name: String,
    ) -> Self {
        Self {
            mesh,
            transform,
            operation,
            stack_operation,
            name,
        }
    }

    /// Set the CSG operation
    pub fn with_operation(mut self, operation: CSGType) -> Self {
        self.operation = operation;
        self
    }

    /// Set the transformation
    pub fn with_transform(mut self, transform: Transform3D) -> Self {
        self.transform = transform;
        self
    }

    /// Set the stack operation
    pub fn with_stack_operation(mut self, stack_op: CSGStackOp) -> Self {
        self.stack_operation = stack_op;
        self
    }

    /// Set the name
    pub fn with_name(mut self, name: String) -> Self {
        self.name = name;
        self
    }

    /// Get a reference to the mesh, if available
    pub fn get_mesh(&self) -> Option<&TriangleMesh> {
        self.mesh.get()
    }
}

impl Default for CSGPart {
    /// Create a default CSG part
    /// CSGMesh.hpp:64-68
    fn default() -> Self {
        Self::new()
    }
}

/// Get the CSG operation of a part.
///
/// This is a generic function that can be overridden for custom types.
/// For CSGPart, it simply returns the operation field.
///
/// CSGMesh.hpp:40-43
#[inline]
pub fn get_operation(part: &CSGPart) -> CSGType {
    // CSGMesh.hpp:42
    part.operation
}

/// Get the stack operation required by the CSG part.
///
/// This is a generic function that can be overridden for custom types.
/// For CSGPart, it simply returns the stack_operation field.
///
/// CSGMesh.hpp:45-48
#[inline]
pub fn get_stack_operation(part: &CSGPart) -> CSGStackOp {
    // CSGMesh.hpp:47
    part.stack_operation
}

/// Get the mesh for the part.
///
/// This is a generic function that can be overridden for custom types.
/// For CSGPart, it returns a reference to the mesh if available.
///
/// CSGMesh.hpp:50-53
#[inline]
pub fn get_mesh(part: &CSGPart) -> Option<&TriangleMesh> {
    // CSGMesh.hpp:52
    part.mesh.get()
}

/// Get the transformation associated with the mesh inside a CSGPart object.
///
/// This is a generic function that can be overridden for custom types.
/// For CSGPart, it returns a copy of the transformation matrix.
///
/// CSGMesh.hpp:55-58
#[inline]
pub fn get_transform(part: &CSGPart) -> Transform3D {
    // CSGMesh.hpp:57
    part.transform
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_csg_type_default() {
        assert_eq!(CSGType::default(), CSGType::Union);
    }

    #[test]
    fn test_csg_stack_op_default() {
        assert_eq!(CSGStackOp::default(), CSGStackOp::Continue);
    }

    #[test]
    fn test_mesh_ptr_empty() {
        let ptr = MeshPtr::new();
        assert!(ptr.is_empty());
        assert!(ptr.get().is_none());
    }

    #[test]
    fn test_mesh_ptr_owned() {
        let mesh = TriangleMesh::new();
        let ptr = MeshPtr::from_owned(mesh);
        assert!(!ptr.is_empty());
        assert!(ptr.get().is_some());
    }

    #[test]
    fn test_mesh_ptr_arc() {
        let mesh = Arc::new(TriangleMesh::new());
        let ptr = MeshPtr::from_arc(mesh);
        assert!(!ptr.is_empty());
        assert!(ptr.get().is_some());
    }

    #[test]
    fn test_mesh_ptr_rc() {
        let mesh = Rc::new(TriangleMesh::new());
        let ptr = MeshPtr::from_rc(mesh);
        assert!(!ptr.is_empty());
        assert!(ptr.get().is_some());
    }

    #[test]
    fn test_csg_part_default() {
        let part = CSGPart::new();
        assert_eq!(part.operation, CSGType::Union);
        assert_eq!(part.stack_operation, CSGStackOp::Continue);
        assert!(part.mesh.is_empty());
        assert_eq!(part.name, "");
    }

    #[test]
    fn test_csg_part_from_mesh() {
        let mesh = TriangleMesh::new();
        let part = CSGPart::from_mesh(MeshPtr::from_owned(mesh));
        assert_eq!(part.operation, CSGType::Union);
        assert_eq!(part.stack_operation, CSGStackOp::Continue);
        assert!(!part.mesh.is_empty());
    }

    #[test]
    fn test_csg_part_builder() {
        let mesh = TriangleMesh::new();
        let part = CSGPart::from_mesh(MeshPtr::from_owned(mesh))
            .with_operation(CSGType::Difference)
            .with_stack_operation(CSGStackOp::Push)
            .with_name("test_part".to_string());

        assert_eq!(part.operation, CSGType::Difference);
        assert_eq!(part.stack_operation, CSGStackOp::Push);
        assert_eq!(part.name, "test_part");
        assert!(!part.mesh.is_empty());
    }

    #[test]
    fn test_get_operation() {
        let part = CSGPart::new().with_operation(CSGType::Intersection);
        assert_eq!(get_operation(&part), CSGType::Intersection);
    }

    #[test]
    fn test_get_stack_operation() {
        let part = CSGPart::new().with_stack_operation(CSGStackOp::Pop);
        assert_eq!(get_stack_operation(&part), CSGStackOp::Pop);
    }

    #[test]
    fn test_get_mesh_empty() {
        let part = CSGPart::new();
        assert!(get_mesh(&part).is_none());
    }

    #[test]
    fn test_get_mesh_present() {
        let mesh = TriangleMesh::new();
        let part = CSGPart::from_mesh(MeshPtr::from_owned(mesh));
        assert!(get_mesh(&part).is_some());
    }

    #[test]
    fn test_get_transform() {
        let part = CSGPart::new();
        let transform = get_transform(&part);
        // Should be identity matrix
        assert_eq!(transform, Transform3D::identity());
    }

    #[test]
    fn test_csg_expression_example() {
        // Test the example from the module documentation:
        // CUBE1 - (CUBE2 + CUBE3)
        let cube1 = CSGPart::from_mesh(MeshPtr::from_owned(TriangleMesh::new()))
            .with_operation(CSGType::Union)
            .with_stack_operation(CSGStackOp::Continue)
            .with_name("CUBE1".to_string());

        let cube2 = CSGPart::from_mesh(MeshPtr::from_owned(TriangleMesh::new()))
            .with_operation(CSGType::Difference)
            .with_stack_operation(CSGStackOp::Push)
            .with_name("CUBE2".to_string());

        let cube3 = CSGPart::from_mesh(MeshPtr::from_owned(TriangleMesh::new()))
            .with_operation(CSGType::Union)
            .with_stack_operation(CSGStackOp::Pop)
            .with_name("CUBE3".to_string());

        let parts = vec![cube1, cube2, cube3];

        // Verify the structure
        assert_eq!(parts[0].operation, CSGType::Union);
        assert_eq!(parts[0].stack_operation, CSGStackOp::Continue);

        assert_eq!(parts[1].operation, CSGType::Difference);
        assert_eq!(parts[1].stack_operation, CSGStackOp::Push);

        assert_eq!(parts[2].operation, CSGType::Union);
        assert_eq!(parts[2].stack_operation, CSGStackOp::Pop);
    }

    #[test]
    fn test_csg_part_with_all_fields() {
        let mesh = TriangleMesh::new();
        let transform = Transform3D::identity();
        let part = CSGPart::with_all_fields(
            MeshPtr::from_owned(mesh),
            CSGType::Intersection,
            transform,
            CSGStackOp::Push,
            "complex_part".to_string(),
        );

        assert_eq!(part.operation, CSGType::Intersection);
        assert_eq!(part.stack_operation, CSGStackOp::Push);
        assert_eq!(part.name, "complex_part");
        assert!(!part.mesh.is_empty());
    }
}
