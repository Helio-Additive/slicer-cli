//! Arachne module - Variable-width perimeter generation
//!
//! This module implements the Arachne algorithm for generating variable-width
//! perimeters that adapt to local geometry. This improves print quality for
//! thin walls and narrow features.
//!
//! # Overview
//!
//! Arachne uses a skeletal trapezoidation approach to:
//! - Decompose polygons into medial axis graphs
//! - Calculate optimal bead widths at each point
//! - Generate toolpaths with variable extrusion widths
//!
//! # C++ Reference
//!
//! Corresponds to BambuStudio's `src/libslic3r/Arachne/` directory:
//! - `WallToolPaths.cpp/hpp` - Main entry point
//! - `SkeletalTrapezoidation.cpp/hpp` - Core algorithm
//! - `BeadingStrategy/` - Width calculation strategies
//! - `utils/` - Supporting utilities

// Main Arachne implementation
pub mod arachne;
pub mod beading_strategy;
pub mod skeletal_trapezoidation;
pub mod skeletal_trapezoidation_edge;
pub mod skeletal_trapezoidation_graph;
pub mod skeletal_trapezoidation_joint;
pub mod utils;
pub mod wall_tool_paths;

// Re-export main types from arachne/arachne/mod.rs
pub use arachne::{
    generate_arachne_walls, generate_arachne_walls_with_width, ArachneConfig, ArachneGenerator,
    ArachneResult, BeadingCalculator, BeadingResult, BeadingStrategy, ExtrusionJunction,
    ExtrusionLine, VariableWidthLines,
};

// Re-export wall tool paths
pub use wall_tool_paths::WallToolPaths;

// Re-export beading strategies (only those that exist)
pub use beading_strategy::{
    BeadingStrategyFactory, DistributedBeadingStrategy, LimitedBeadingStrategy,
    OuterWallInsetBeadingStrategy, RedistributeBeadingStrategy, WideningBeadingStrategy,
};
// TODO: Port CenterDeviationBeadingStrategy and InwardDistributedBeadingStrategy from C++

// Re-export skeletal trapezoidation
pub use skeletal_trapezoidation::SkeletalTrapezoidation;
pub use skeletal_trapezoidation_edge::SkeletalTrapezoidationEdge;
pub use skeletal_trapezoidation_graph::SkeletalTrapezoidationGraph;
pub use skeletal_trapezoidation_joint::SkeletalTrapezoidationJoint;

// Re-export utils (only those that exist in utils/mod.rs)
// TODO: Port compute_medial_axis, PolygonUtils, VoronoiUtils from C++
// ExtrusionJunction and ExtrusionLine are already exported from arachne module above
