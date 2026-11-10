//! Faithful 1:1 port of BambuStudio `src/libslic3r/CSGMesh/CSGMesh.hpp`.
//!
//! Header-only file; there is no `.cpp` translation unit, so every definition
//! lives in the header and is reproduced here line-by-line. Line references
//! point into `CSGMesh.hpp`.
//!
//! Divergence note: the C++ `CSGPart` stores `AnyPtr<const indexed_triangle_set>`
//! (see `crate::any_ptr::AnyPtr`) and a `Transform3f` (Eigen
//! `Transform<float,3,Affine>`). This crate's CSGMesh module
//! (`csg_mesh_copy`, `model_to_csg_mesh`, `slice_csg_mesh`,
//! `perform_csg_mesh_booleans`, `voxelize_csg_mesh`, ...) was built around
//! `crate::triangle_mesh::TriangleMesh` and `crate::geometry::Transform3D`
//! through a local `MeshPtr` enum that mirrors `AnyPtr`'s three alternatives
//! (raw/owned/shared, here None/Owned/Shared plus an extra Rc). Those
//! higher-level types are retained so the whole module stays consistent; the
//! field semantics and the four accessor templates below match the C++ exactly.

use crate::geometry::Transform3D;
use crate::triangle_mesh::TriangleMesh;
use std::rc::Rc;
use std::sync::Arc;

// CSGMesh.hpp:7  namespace Slic3r { namespace csg {

// CSGMesh.hpp:9-17
// A CSGPartT should be an object that can provide at least a mesh + trafo and an
// associated csg operation. A collection of CSGPartT objects can then
// be interpreted as one model and used in various contexts. It can be assembled
// with CGAL or OpenVDB, rendered with OpenCSG or provided to a ray-tracer to
// deal with various parts of it according to the supported CSG types...
//
// A few simple templated interface functions are provided here and a default
// CSGPart class that implements the necessary means to be usable as a
// CSGPartT object.

// CSGMesh.hpp:19  Supported CSG operation types
// CSGMesh.hpp:20  enum class CSGType { Union, Difference, Intersection };
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CSGType {
    Union,
    Difference,
    Intersection,
}

impl Default for CSGType {
    // The C++ `CSGPart` constructor defaults `op` to `CSGType::Union`
    // (CSGMesh.hpp:76).
    fn default() -> Self {
        CSGType::Union
    }
}

// CSGMesh.hpp:22-37
// A CSG part can instruct the processing to push the sub-result until a new
// csg part with a pop instruction appears. This can be used to implement
// parentheses in a CSG expression represented by the collection of csg parts.
// A CSG part can not contain another CSG collection, only a mesh, this is why
// its easier to do this stacking instead of recursion in the data definition.
// CSGStackOp::Continue means no stack operation required.
// When a CSG part contains a Push instruction, it is expected that the CSG
// operation it contains refers to the whole collection spanning to the nearest
// part with a Pop instruction.
// e.g.:
// {
//      CUBE1: { mesh: cube, op: Union, stack op: Continue },
//      CUBE2: { mesh: cube, op: Difference, stack op: Push},
//      CUBE3: { mesh: cube, op: Union, stack op: Pop}
// }
// is a collection of csg parts representing the expression CUBE1 - (CUBE2 + CUBE3)
// CSGMesh.hpp:38  enum class CSGStackOp { Push, Continue, Pop };
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CSGStackOp {
    Push,
    Continue,
    Pop,
}

impl Default for CSGStackOp {
    // The C++ `CSGPart` constructor always sets `stack_operation` to
    // `CSGStackOp::Continue` (CSGMesh.hpp:80).
    fn default() -> Self {
        CSGStackOp::Continue
    }
}

/// A pointer type that can hold either owned or borrowed mesh data.
///
/// This is the Rust equivalent of C++'s `AnyPtr<const indexed_triangle_set>`.
/// It supports multiple ownership patterns to work with different contexts.
///
/// CSGMesh.hpp:69 (`AnyPtr<const indexed_triangle_set> its_ptr;`)
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

// CSGMesh.hpp:67  Default implementation
/// A CSG part: mesh + transformation + CSG operation.
///
/// Default implementation of a `CSGPartT` object that implements the necessary
/// means to be usable as a `CSGPartT` object. A CSG part can not contain
/// another CSG collection, only a mesh — this is why stack operations are used
/// instead of recursion in the data definition.
///
/// CSGMesh.hpp:68-83  struct CSGPart { ... };
#[derive(Debug, Clone)]
pub struct CSGPart {
    /// CSGMesh.hpp:69  AnyPtr<const indexed_triangle_set> its_ptr;
    pub mesh: MeshPtr,

    /// CSGMesh.hpp:70  Transform3f trafo;
    pub transform: Transform3D,

    /// CSGMesh.hpp:71  CSGType operation;
    pub operation: CSGType,

    /// CSGMesh.hpp:72  CSGStackOp stack_operation;
    pub stack_operation: CSGStackOp,

    /// CSGMesh.hpp:73  std::string name;
    pub name: String,
}

impl CSGPart {
    // CSGMesh.hpp:75-82
    // CSGPart(AnyPtr<const indexed_triangle_set> ptr = {},
    //         CSGType                            op  = CSGType::Union,
    //         const Transform3f                 &tr  = Transform3f::Identity())
    //     : its_ptr{std::move(ptr)}
    //     , operation{op}
    //     , stack_operation{CSGStackOp::Continue}
    //     , trafo{tr}
    // {}
    /// Create a new CSG part with the C++ default arguments
    /// (empty `its_ptr`, `CSGType::Union`, identity transform).
    pub fn new() -> Self {
        Self {
            mesh: MeshPtr::None,
            transform: Transform3D::identity(),
            operation: CSGType::Union,
            stack_operation: CSGStackOp::Continue,
            name: String::new(),
        }
    }

    /// Create a CSG part from a mesh pointer (C++ ctor with `ptr` supplied,
    /// `op`/`tr` defaulted).
    /// CSGMesh.hpp:75-82
    pub fn from_mesh(mesh: MeshPtr) -> Self {
        Self {
            mesh,
            transform: Transform3D::identity(),
            operation: CSGType::Union,
            stack_operation: CSGStackOp::Continue,
            name: String::new(),
        }
    }

    /// Create a CSG part with mesh, operation, and transform (C++ ctor with all
    /// three positional arguments supplied).
    /// CSGMesh.hpp:75-82
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
    /// Create a default CSG part (C++ default-constructed `CSGPart`).
    /// CSGMesh.hpp:75-82
    fn default() -> Self {
        Self::new()
    }
}

// CSGMesh.hpp:40  Get the CSG operation of the part. Can be overriden for any type
// CSGMesh.hpp:41-44
// template<class CSGPartT> CSGType get_operation(const CSGPartT &part)
// {
//     return part.operation;
// }
#[inline]
pub fn get_operation(part: &CSGPart) -> CSGType {
    // CSGMesh.hpp:43  return part.operation;
    part.operation
}

// CSGMesh.hpp:46  Get the stack operation required by the CSG part.
// CSGMesh.hpp:47-50
// template<class CSGPartT> CSGStackOp get_stack_operation(const CSGPartT &part)
// {
//     return part.stack_operation;
// }
#[inline]
pub fn get_stack_operation(part: &CSGPart) -> CSGStackOp {
    // CSGMesh.hpp:49  return part.stack_operation;
    part.stack_operation
}

// CSGMesh.hpp:52  Get the mesh for the part. Can be overriden for any type
// CSGMesh.hpp:53-57
// template<class CSGPartT>
// const indexed_triangle_set *get_mesh(const CSGPartT &part)
// {
//     return part.its_ptr.get();
// }
#[inline]
pub fn get_mesh(part: &CSGPart) -> Option<&TriangleMesh> {
    // CSGMesh.hpp:56  return part.its_ptr.get();
    part.mesh.get()
}

// CSGMesh.hpp:59-60  Get the transformation associated with the mesh inside a
// CSGPartT object. Can be overriden for any type.
// CSGMesh.hpp:61-65
// template<class CSGPartT>
// Transform3f get_transform(const CSGPartT &part)
// {
//     return part.trafo;
// }
#[inline]
pub fn get_transform(part: &CSGPart) -> Transform3D {
    // CSGMesh.hpp:64  return part.trafo;
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
