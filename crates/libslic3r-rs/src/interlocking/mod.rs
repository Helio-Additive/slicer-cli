//! Interlocking structure generation module.
//!
//! Provides voxel-based interlocking structure generation between adjacent
//! mesh regions with different extruders.
//!
//! C++ Reference: Interlocking/VoxelUtils.hpp, InterlockingGenerator.hpp

pub mod interlocking_generator;
pub mod voxel_utils;

// Re-export key types
pub use interlocking_generator::InterlockingGenerator;
pub use voxel_utils::{DilationKernel, DilationKernelType, GridPoint3, VoxelUtils};
