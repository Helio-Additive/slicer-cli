//! CSG (Constructive Solid Geometry) mesh module
//!
//! This module provides types and functions for CSG operations on triangle meshes.
//! CSG allows combining meshes using boolean operations (union, difference, intersection).
//!
//! C++ Reference:
//! - CSGMesh/CSGMesh.hpp
//! - CSGMesh/*.cpp files

pub mod csg_mesh;
pub mod csg_mesh_copy;
pub mod model_to_csg_mesh;
pub mod perform_csg_mesh_booleans;
pub mod slice_csg_mesh;
pub mod triangle_mesh_adapter;
pub mod voxelize_csg_mesh;

// Re-export key types
pub use csg_mesh::{
    get_mesh, get_operation, get_stack_operation, get_transform, CSGPart, CSGStackOp, CSGType,
    MeshPtr,
};
pub use voxelize_csg_mesh::{
    get_voxelgrid, voxelize_csgmesh, StatusCallback, VoxelGrid, VoxelizeParams,
};
