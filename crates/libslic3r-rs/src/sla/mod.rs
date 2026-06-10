//! sla module
//!
//! Auto-generated module declaration for sla

pub mod agg;
pub mod agg_raster;
pub mod bicubic;
pub mod boost_adapter;
pub mod clustering;
pub mod concave_hull;
pub mod concurrency;
pub mod hollowing;
pub mod indexed_mesh;
pub mod job_controller;
pub mod pad;
pub mod raster_base;
pub mod raster_to_polygons;
pub mod reproject_points_on_mesh;
pub mod rotfinder;
pub mod spat_index;
pub mod support_point;
pub mod support_point_generator;
pub mod support_tree;
pub mod support_tree_builder;
pub mod support_tree_buildsteps;
pub mod support_tree_mesher;

// Re-export key types
// SLA/Concurrency.hpp:64-66 places `ccr`, `ccr_seq`, `ccr_par` directly in
// the Slic3r::sla namespace.
pub use concurrency::{ccr, ccr_par, ccr_seq, USE_FULL_CONCURRENCY};
