//! Constrained Delaunay triangulation for 2D polygons
//!
//! 1:1 line-by-line port of `Triangulation.cpp` / `Triangulation.hpp`.
//!
//! C++ Reference:
//! - Triangulation.hpp (71 lines)
//! - Triangulation.cpp (329 lines)
//!
//! BLOCKED SYMBOL (native CGAL backend; byte-exactness, not just wasm):
//! The core `triangulate(points, half_edges)` (Triangulation.cpp:86-206) is
//! implemented in C++ entirely on top of CGAL's
//! `Constrained_Delaunay_triangulation_2<K, Tds, Exact_predicates_tag>`
//! (kernel `Exact_predicates_inexact_constructions_kernel`), `CGAL::spatial_sort`,
//! and the CGAL triangulation data structure with vertex info. There is no CGAL
//! FFI in this crate and no pure-Rust constrained-Delaunay backend is wired in
//! (no `spade`/`cdt`/`delaunator` in Cargo.toml).
//!
//! Two reasons this stays blocked rather than swapping in a Rust CDT crate:
//!   1. CGAL is a native C++ library and is NOT wasm-safe, so per the porting
//!      rules we do not add it as a system/dylib dep.
//!   2. Even a *wasm-safe* pure-Rust CDT (e.g. `spade`) could NOT reproduce this
//!      result byte-for-byte: the emitted triangle set depends on CGAL's exact
//!      predicates, on the specific `CGAL::spatial_sort` insertion order (which
//!      drives the Delaunay tie-breaking), on CGAL's internal face/vertex
//!      ordering, and on the constrained-edge flood-fill traversal order over
//!      `cdt.finite_face_handles()`. A different backend yields a different (even
//!      if geometrically valid) `Vec3i32` index list, breaking G-code/mesh
//!      parity. Faking a result here would violate the no-stubs rule.
//!
//! Everything that does NOT depend on the CGAL kernel is ported faithfully below
//! (all edge assembly, change-map/duplicate handling, and the precondition
//! checks); the CGAL kernel body itself is left returning an empty index set and
//! documented as blocked.
//!
//! Sole on-path C++ callers of the blocked symbol: `WipeTower.cpp` (rib-tower /
//! rib-brim cap meshes via `its_make_rib_tower` / `its_make_rib_brim`) and
//! `Emboss.cpp` (text-on-mesh, off the slicing path). The wipe-tower extrusion
//! G-code itself comes from the 2D fill logic, not from these triangulated caps.

use crate::geometry::{
    collect_duplicates, count_points, to_points, ExPolygon, ExPolygons, Point, Points, Polygon,
    Polygons,
};
use crate::intersection_points::get_intersections_lines;
use crate::Result;

/// Half-edge definition: oriented connection of 2 vertices (by index)
/// Triangulation.hpp:18
pub type HalfEdge = (u32, u32);

/// Vector of half-edges
/// Triangulation.hpp:19
pub type HalfEdges = Vec<HalfEdge>;

/// Triangle indices (3 vertex indices per triangle)
/// `Vec3i32` in C++.
/// Triangulation.hpp:20
pub type Indices = Vec<[i32; 3]>;

/// Map for convert original index to set without duplication
///              from_index<to_index>
/// Triangulation.hpp:40
pub type Changes = Vec<u32>;

// ============================================================================
// namespace priv { ... }  (Triangulation.cpp:10-68)
// ============================================================================

/// Triangulation.cpp:11-24
#[inline]
fn insert_edges_with_changes(
    edges: &mut HalfEdges,
    offset: &mut u32,
    polygon: &Polygon,
    changes: &Changes,
) {
    // Triangulation.cpp:12
    let pts = &polygon.points;
    // Triangulation.cpp:13
    let size = pts.len() as u32;
    // Triangulation.cpp:14
    let last_index = *offset + size - 1;
    // Triangulation.cpp:15
    let mut prev_index = changes[last_index as usize];
    // Triangulation.cpp:16
    for i in 0..size {
        // Triangulation.cpp:17
        let index = changes[(*offset + i) as usize];
        // when duplicit points are neighbor
        // Triangulation.cpp:19
        if prev_index == index {
            continue;
        }
        // Triangulation.cpp:20
        edges.push((prev_index, index));
        // Triangulation.cpp:21
        prev_index = index;
    }
    // Triangulation.cpp:23
    *offset += size;
}

/// Triangulation.cpp:26-36
#[inline]
fn insert_edges(edges: &mut HalfEdges, offset: &mut u32, polygon: &Polygon) {
    // Triangulation.cpp:27
    let pts = &polygon.points;
    // Triangulation.cpp:28
    let size = pts.len() as u32;
    // Triangulation.cpp:29
    let mut prev_index = *offset + size - 1;
    // Triangulation.cpp:30
    for i in 0..size {
        // Triangulation.cpp:31
        let index = *offset + i;
        // Triangulation.cpp:32
        edges.push((prev_index, index));
        // Triangulation.cpp:33
        prev_index = index;
    }
    // Triangulation.cpp:35
    *offset += size;
}

/// Triangulation.cpp:38-48
#[inline]
fn has_bidirectional_constrained(constrained: &HalfEdges) -> bool {
    // Triangulation.cpp:41
    for c in constrained {
        // Triangulation.cpp:42
        let key = (c.1, c.0);
        // Triangulation.cpp:43-44: std::lower_bound on the sorted vector
        let idx = constrained.partition_point(|&e| e < key);
        // Triangulation.cpp:45
        if idx != constrained.len() && constrained[idx] == key {
            return true;
        }
    }
    // Triangulation.cpp:47
    false
}

/// Triangulation.cpp:50-55
#[inline]
fn is_unique(points: &Points) -> bool {
    // Triangulation.cpp:51: Points pts = points; // copy
    let mut pts = points.clone();
    // Triangulation.cpp:52
    pts.sort();
    // Triangulation.cpp:53: auto it = std::adjacent_find(pts.begin(), pts.end());
    // Triangulation.cpp:54: return it == pts.end();
    for i in 1..pts.len() {
        if pts[i - 1] == pts[i] {
            return false;
        }
    }
    true
}

/// Triangulation.cpp:57-66
#[inline]
fn has_self_intersection(points: &Points, constrained_half_edges: &HalfEdges) -> bool {
    // Triangulation.cpp:61
    let mut lines = crate::geometry::Lines::new();
    // Triangulation.cpp:62
    lines.reserve(constrained_half_edges.len());
    // Triangulation.cpp:63-64
    for he in constrained_half_edges {
        lines.push(crate::geometry::Line::new(
            points[he.0 as usize],
            points[he.1 as usize],
        ));
    }
    // Triangulation.cpp:65
    !get_intersections_lines(&lines).is_empty()
}

// ============================================================================
// class Triangulation  (Triangulation.cpp:86-329)
// ============================================================================

/// Connect points by triangulation to create filled surface by triangles.
/// Input points have to be unique.
///
/// Triangulation.cpp:86-206
///
/// NOTE (blocked): the CGAL Constrained Delaunay backend is a native,
/// non-wasm-safe dependency and is not wired into this crate, so the actual
/// triangulation step (Triangulation.cpp:101-205) is unavailable. The faithful
/// pre-conditions (assertions) are preserved below. See module docs.
pub fn triangulate(points: &Points, constrained_half_edges: &HalfEdges) -> Result<Indices> {
    // Triangulation.cpp:89
    assert!(!points.is_empty());
    // Triangulation.cpp:90
    assert!(!constrained_half_edges.is_empty());
    // constrained must be sorted
    // Triangulation.cpp:92-93
    assert!(constrained_half_edges.windows(2).all(|w| w[0] <= w[1]));
    // check that there is no duplicit constrained edge
    // Triangulation.cpp:95
    assert!(constrained_half_edges
        .windows(2)
        .all(|w| w[0] != w[1]));
    // edges can NOT contain bidirectional constrained
    // Triangulation.cpp:97
    assert!(!has_bidirectional_constrained(constrained_half_edges));
    // check that there is only unique poistion of points
    // Triangulation.cpp:99
    assert!(is_unique(points));
    // Triangulation.cpp:100
    assert!(!has_self_intersection(points, constrained_half_edges));

    // --- BLOCKED: CGAL Constrained_Delaunay_triangulation_2 backend ---
    // Triangulation.cpp:101-205
    //
    // The C++ body builds a CGAL CDT (Exact_predicates_inexact_constructions
    // kernel + Exact_predicates_tag), spatial-sorts the points, inserts the
    // vertices carrying their original index as `info`, inserts the constrained
    // half-edges, then flood-fills face constraints to keep only the faces that
    // lie inside the constrained region and emits them as `Vec3i32` triangles.
    //
    // None of CGAL is available here (native, non-wasm), and no pure-Rust
    // constrained-Delaunay crate is wired in, so this step cannot be reproduced
    // byte-exactly. Returning an empty index set rather than a fake result.
    // Triangulation.cpp:205: return indices;
    let _ = (points, constrained_half_edges);
    Ok(Vec::new())
}

/// Triangulation.cpp:208-217
pub fn triangulate_polygon(polygon: &Polygon) -> Result<Indices> {
    // Triangulation.cpp:210
    let pts = &polygon.points;
    // Triangulation.cpp:211
    let mut edges: HalfEdges = HalfEdges::new();
    // Triangulation.cpp:212
    edges.reserve(pts.len());
    // Triangulation.cpp:213
    let mut offset: u32 = 0;
    // Triangulation.cpp:214
    insert_edges(&mut edges, &mut offset, polygon);
    // Triangulation.cpp:215
    edges.sort();
    // Triangulation.cpp:216
    triangulate(pts, &edges)
}

/// Triangulation.cpp:219-236
pub fn triangulate_polygons(polygons: &Polygons) -> Result<Indices> {
    // Triangulation.cpp:221: size_t count = count_points(polygons);
    let count: usize = polygons.iter().map(|p| p.points.len()).sum();
    // Triangulation.cpp:222
    let mut points: Points = Points::new();
    // Triangulation.cpp:223
    points.reserve(count);

    // Triangulation.cpp:225
    let mut edges: HalfEdges = HalfEdges::new();
    // Triangulation.cpp:226
    edges.reserve(count);
    // Triangulation.cpp:227
    let mut offset: u32 = 0;

    // Triangulation.cpp:229
    for polygon in polygons {
        // Triangulation.cpp:230: Slic3r::append(points, polygon.points);
        points.extend_from_slice(&polygon.points);
        // Triangulation.cpp:231
        insert_edges(&mut edges, &mut offset, polygon);
    }

    // Triangulation.cpp:234
    edges.sort();
    // Triangulation.cpp:235
    triangulate(&points, &edges)
}

/// Triangulation.cpp:238-241
pub fn triangulate_expolygon(expolygon: &ExPolygon) -> Result<Indices> {
    // Triangulation.cpp:239: ExPolygons expolys({expolygon});
    let expolys: ExPolygons = vec![expolygon.clone()];
    // Triangulation.cpp:240
    triangulate_expolygons(&expolys)
}

/// Triangulation.cpp:243-260
pub fn triangulate_expolygons(expolygons: &ExPolygons) -> Result<Indices> {
    // Triangulation.cpp:244
    let pts: Points = to_points(expolygons);
    // Triangulation.cpp:245
    let d_pts: Points = collect_duplicates(pts.clone());
    // Triangulation.cpp:246
    if d_pts.is_empty() {
        return triangulate_expolygons_with_points(expolygons, &pts);
    }

    // Triangulation.cpp:248
    let changes: Changes = create_changes(&pts, &d_pts);
    // Triangulation.cpp:249
    let mut indices: Indices = triangulate_expolygons_with_changes(expolygons, &pts, &changes)?;
    // reverse map for changes
    // Triangulation.cpp:251: Changes changes2(changes.size(), numeric_limits<uint32_t>::max());
    let mut changes2: Changes = vec![u32::MAX; changes.len()];
    // Triangulation.cpp:252-253
    for i in 0..changes.len() {
        changes2[changes[i] as usize] = i as u32;
    }

    // convert indices into expolygons indicies
    // Triangulation.cpp:256-257
    for t in indices.iter_mut() {
        for ti in 0..3 {
            t[ti] = changes2[t[ti] as usize] as i32;
        }
    }

    // Triangulation.cpp:259
    Ok(indices)
}

/// Triangulation.cpp:262-278
pub fn triangulate_expolygons_with_points(
    expolygons: &ExPolygons,
    points: &Points,
) -> Result<Indices> {
    // Triangulation.cpp:264
    assert!(count_points(expolygons) == points.len());
    // when contain duplicit coordinate in points will not work properly
    // Triangulation.cpp:266
    assert!(collect_duplicates(points.clone()).is_empty());

    // Triangulation.cpp:268
    let mut edges: HalfEdges = HalfEdges::new();
    // Triangulation.cpp:269
    edges.reserve(points.len());
    // Triangulation.cpp:270
    let mut offset: u32 = 0;
    // Triangulation.cpp:271
    for expolygon in expolygons {
        // Triangulation.cpp:272
        insert_edges(&mut edges, &mut offset, &expolygon.contour);
        // Triangulation.cpp:273
        for hole in &expolygon.holes {
            insert_edges(&mut edges, &mut offset, hole);
        }
    }
    // Triangulation.cpp:276
    edges.sort();
    // Triangulation.cpp:277
    triangulate(points, &edges)
}

/// Triangulation.cpp:280-302
pub fn triangulate_expolygons_with_changes(
    expolygons: &ExPolygons,
    points: &Points,
    changes: &Changes,
) -> Result<Indices> {
    // Triangulation.cpp:282
    assert!(!points.is_empty());
    // Triangulation.cpp:283
    assert!(count_points(expolygons) == points.len());
    // Triangulation.cpp:284
    assert!(changes.len() == points.len());
    // IMPROVE: search from end and somehow distiquish that value is not a change
    // Triangulation.cpp:286: uint32_t count_points = *std::max_element(changes...)+1;
    let count_points: u32 = *changes.iter().max().unwrap() + 1;
    // Triangulation.cpp:287: Points pts(count_points);
    let mut pts: Points = vec![Point::new(0, 0); count_points as usize];
    // Triangulation.cpp:288-289
    for i in 0..changes.len() {
        pts[changes[i] as usize] = points[i];
    }

    // Triangulation.cpp:291
    let mut edges: HalfEdges = HalfEdges::new();
    // Triangulation.cpp:292
    edges.reserve(points.len());
    // Triangulation.cpp:293
    let mut offset: u32 = 0;
    // Triangulation.cpp:294
    for expolygon in expolygons {
        // Triangulation.cpp:295
        insert_edges_with_changes(&mut edges, &mut offset, &expolygon.contour, changes);
        // Triangulation.cpp:296
        for hole in &expolygon.holes {
            insert_edges_with_changes(&mut edges, &mut offset, hole, changes);
        }
    }

    // Triangulation.cpp:300
    edges.sort();
    // Triangulation.cpp:301
    triangulate(&pts, &edges)
}

/// Triangulation.cpp:304-329
pub fn create_changes(points: &Points, duplicits: &Points) -> Changes {
    // Triangulation.cpp:306
    assert!(!duplicits.is_empty());
    // Triangulation.cpp:307
    assert!(duplicits.len() < points.len() / 2);
    // Triangulation.cpp:308: duplicit_indices(duplicits.size(), numeric_limits<uint32_t>::max())
    let mut duplicit_indices: Vec<u32> = vec![u32::MAX; duplicits.len()];
    // Triangulation.cpp:309
    let mut changes: Changes = Changes::new();
    // Triangulation.cpp:310
    changes.reserve(points.len());
    // Triangulation.cpp:311
    let mut index: u32 = 0;
    // Triangulation.cpp:312
    for p in points {
        // Triangulation.cpp:313: std::lower_bound(duplicits.begin(), duplicits.end(), p)
        let it = duplicits.partition_point(|d| d < p);
        // Triangulation.cpp:314
        if it == duplicits.len() || duplicits[it] != *p {
            // Triangulation.cpp:315
            changes.push(index);
            // Triangulation.cpp:316
            index += 1;
            // Triangulation.cpp:317
            continue;
        }
        // Triangulation.cpp:319: uint32_t &d_index = duplicit_indices[it - duplicits.begin()];
        let d_index = &mut duplicit_indices[it];
        // Triangulation.cpp:320
        if *d_index == u32::MAX {
            // Triangulation.cpp:321
            *d_index = index;
            // Triangulation.cpp:322
            changes.push(index);
            // Triangulation.cpp:323
            index += 1;
        } else {
            // Triangulation.cpp:325
            changes.push(*d_index);
        }
    }
    // Triangulation.cpp:328
    changes
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_halfedge_type() {
        let edge: HalfEdge = (0, 1);
        assert_eq!(edge.0, 0);
        assert_eq!(edge.1, 1);
    }

    #[test]
    fn test_has_bidirectional_constrained() {
        // sorted lexicographically (precondition of the function)
        let edges = vec![(0u32, 1u32), (1, 2), (2, 0)];
        assert!(!has_bidirectional_constrained(&edges));

        let edges_with_bidir = vec![(0u32, 1u32), (1, 0), (1, 2)];
        assert!(has_bidirectional_constrained(&edges_with_bidir));
    }

    #[test]
    fn test_is_unique() {
        let points = vec![Point::new(0, 0), Point::new(1, 1), Point::new(2, 2)];
        assert!(is_unique(&points));

        let points_with_dup = vec![Point::new(0, 0), Point::new(1, 1), Point::new(0, 0)];
        assert!(!is_unique(&points_with_dup));
    }

    #[test]
    fn test_has_self_intersection() {
        // Two crossing constrained edges: (0,0)-(2,2) and (0,2)-(2,0)
        let points = vec![
            Point::new(0, 0),
            Point::new(2, 2),
            Point::new(0, 2),
            Point::new(2, 0),
        ];
        let edges = vec![(0u32, 1u32), (2, 3)];
        assert!(has_self_intersection(&points, &edges));

        // Non-crossing edges
        let edges_ok = vec![(0u32, 3u32), (1, 2)];
        assert!(!has_self_intersection(&points, &edges_ok));
    }

    #[test]
    fn test_create_changes_no_remap_for_unique() {
        // points with one duplicate coordinate (index 0 and 3 both (0,0))
        let points = vec![
            Point::new(0, 0),
            Point::new(10, 0),
            Point::new(0, 10),
            Point::new(0, 0),
            Point::new(5, 5),
            Point::new(7, 7),
        ];
        let duplicits = collect_duplicates(points.clone());
        assert_eq!(duplicits, vec![Point::new(0, 0)]);
        let changes = create_changes(&points, &duplicits);
        // index 3 must map back to the first occurrence's new index (0)
        assert_eq!(changes[0], changes[3]);
        assert_eq!(changes.len(), points.len());
    }
}
