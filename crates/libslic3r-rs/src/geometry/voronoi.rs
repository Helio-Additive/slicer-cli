//! Geometry/Voronoi.cpp + Voronoi.hpp — `Slic3r::Geometry::VoronoiDiagram`.
//!
//! C++ `Geometry/Voronoi.cpp` is a thin wrapper around `boost::polygon`'s
//! Voronoi diagram (`voronoi_diagram<double>`) that adds degeneracy detection
//! (`detect_known_issues` / `has_finite_edge_with_non_finite_vertex` /
//! `detect_known_voronoi_cell_issues`) and repair-by-rotation
//! (`try_to_repair_degenerated_voronoi_diagram*`).
//!
//! That class is ported faithfully — backed by the `boostvoronoi` crate, which
//! is the Rust port of `boost::polygon` (wasm-safe, no native deps) — in
//! [`crate::geometry::voronoi_diagram`]. This module re-exports it so that the
//! historical path `crate::geometry::VoronoiDiagram` and the dedicated
//! `voronoi_diagram::VoronoiDiagram` resolve to the same faithful type.
//!
//! Previously this file held a brute-force point-sampling "Voronoi" generator
//! (`from_points` / `compute_cell_brute_force` / `find_closest_site` /
//! `site_distance` / `VoronoiCell`). None of that corresponds to anything in
//! `Geometry/Voronoi.cpp` — C++ never point-samples cells, never computes a
//! per-cell bounding box, and has no `find_closest_site`/`site_distance` free
//! functions. It was a divergent placeholder and has been removed in favor of
//! the faithful `boostvoronoi`-backed port.

// Voronoi.hpp:21  class VoronoiDiagram
// Voronoi.cpp:26  VoronoiDiagram::construct_voronoi(...)
pub use crate::geometry::voronoi_diagram::VoronoiDiagram;
