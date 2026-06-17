//! Faithful 1:1 port of `GCode/RetractWhenCrossingPerimeters.{hpp,cpp}`
//! (BambuStudio / libslic3r).
//!
//! Decides whether the nozzle should retract while crossing perimeter walls,
//! using AABB tree searches over internal island boundaries.
//!
//! C++ Reference:
//! - GCode/RetractWhenCrossingPerimeters.hpp
//! - GCode/RetractWhenCrossingPerimeters.cpp

// RetractWhenCrossingPerimeters.cpp:1-10
// #include "../ClipperUtils.hpp"
// #include "../Layer.hpp"
// #include "../Polyline.hpp"
// #include "../BoundingBox.hpp"
// #include "../ExPolygon.hpp"
// #include "../Polygon.hpp"
// #include "./Utils.hpp"
// #include <vector>
// #include "RetractWhenCrossingPerimeters.hpp"
use crate::aabb_tree_lines::tree2d;
use crate::aabb_tree_lines::LinesDistancer;
use crate::clipper_utils;
use crate::geometry::{BoundingBox, ExPolygon, Line, Polyline};
use crate::layer::Layer;
use crate::libslic3r::SCALED_EPSILON;

// RetractWhenCrossingPerimeters.cpp:12
// #define RETRACT_WHEN_CROSSING_PERIMETERS_DEBUG

// RetractWhenCrossingPerimeters.hpp:38 — `using AABBTree = AABBTreeIndirect::Tree<2, coord_t>;`
type AABBTree = tree2d::Tree;

/// RetractWhenCrossingPerimeters.hpp:17-40 `class RetractWhenCrossingPerimeters`
pub struct RetractWhenCrossingPerimeters {
    // RetractWhenCrossingPerimeters.hpp:28
    // Last object layer visited, for which a cache of internal islands was created.
    //
    // The C++ stores a `const Layer *` and compares it by pointer identity
    // (`m_layer != &layer`). Rust cannot hold that borrow across calls, so we cache
    // the layer's stable `id()` instead, which is one-to-one with the layer object
    // within a print and produces identical cache-(in)validation behavior.
    m_layer: Option<usize>,
    // RetractWhenCrossingPerimeters.hpp:30
    // Search structure over internal islands.
    #[allow(dead_code)]
    m_internal_islands_bbox: BoundingBox,
    // RetractWhenCrossingPerimeters.hpp:31
    m_aabbtree_lines_distancer: LinesDistancer,
    // RetractWhenCrossingPerimeters.hpp:32
    m_internal_islands_lines: Vec<Line>,
    // RetractWhenCrossingPerimeters.hpp:33
    cross_perimeters_flag: bool,

    // RetractWhenCrossingPerimeters.hpp:36
    // Internal islands only, referencing data owned by m_layer->regions()->surfaces().
    m_internal_islands: Vec<ExPolygon>,
    // RetractWhenCrossingPerimeters.hpp:39
    // Search structure over internal islands.
    m_aabbtree_internal_islands: AABBTree,
}

impl RetractWhenCrossingPerimeters {
    /// Default-constructed state (RetractWhenCrossingPerimeters.hpp:17-40, in-class
    /// member initializers + value-initialized members).
    pub fn new() -> Self {
        Self {
            m_layer: None,
            m_internal_islands_bbox: BoundingBox::new(),
            m_aabbtree_lines_distancer: LinesDistancer::default(),
            m_internal_islands_lines: Vec::new(),
            // RetractWhenCrossingPerimeters.hpp:33 — `bool cross_perimeters_flag = false;`
            cross_perimeters_flag: false,
            m_internal_islands: Vec::new(),
            m_aabbtree_internal_islands: AABBTree::new(),
        }
    }

    // RetractWhenCrossingPerimeters.cpp:16-69
    fn travel_cross_perimeters(&mut self, layer: &Layer, travel: &Polyline) -> bool {
        // RetractWhenCrossingPerimeters.cpp:18
        if !self.cross_perimeters_flag {
            // RetractWhenCrossingPerimeters.cpp:19
            self.cross_perimeters_flag = true;
            // RetractWhenCrossingPerimeters.cpp:20-21
            // get all the external perimeters and internal perimeters
            self.m_internal_islands_lines.clear();
            // RetractWhenCrossingPerimeters.cpp:22
            let mut perimeters_polylines: Vec<Polyline> = Vec::new();
            // RetractWhenCrossingPerimeters.cpp:23
            for layer_region_ptr in layer.regions() {
                // RetractWhenCrossingPerimeters.cpp:24
                let mut is_internal = false;
                // RetractWhenCrossingPerimeters.cpp:25
                for surface in &layer_region_ptr.get_slices().surfaces {
                    // RetractWhenCrossingPerimeters.cpp:26
                    if surface.is_internal() {
                        // RetractWhenCrossingPerimeters.cpp:27
                        is_internal = true;
                        // RetractWhenCrossingPerimeters.cpp:28
                        let lines = surface.expolygon.lines();
                        // RetractWhenCrossingPerimeters.cpp:29
                        self.m_internal_islands_lines.extend(lines);
                    }
                }
                // RetractWhenCrossingPerimeters.cpp:31
                if is_internal {
                    let polylines = layer_region_ptr.perimeters.collect_polylines();
                    perimeters_polylines.extend(polylines);
                }
            }
            // RetractWhenCrossingPerimeters.cpp:33
            for perimeter_polyline in &perimeters_polylines {
                // RetractWhenCrossingPerimeters.cpp:34
                // Convert Polyline to Lines and add to m_internal_islands_lines.
                // RetractWhenCrossingPerimeters.cpp:35
                let lines = perimeter_polyline.lines();
                // RetractWhenCrossingPerimeters.cpp:36
                self.m_internal_islands_lines.extend(lines);
            }

            // RetractWhenCrossingPerimeters.cpp:39
            // AABBTreeLines::LinesDistancer<Line>{std::move(m_internal_islands_lines)};
            self.m_aabbtree_lines_distancer =
                LinesDistancer::new(std::mem::take(&mut self.m_internal_islands_lines));

            // RetractWhenCrossingPerimeters.cpp:41-48
            // #ifdef RETRACT_WHEN_CROSSING_PERIMETERS_DEBUG ... bbox merge ... #endif
        }

        // RetractWhenCrossingPerimeters.cpp:51-56
        // #ifdef RETRACT_WHEN_CROSSING_PERIMETERS_DEBUG ... SVG output ... #endif

        // RetractWhenCrossingPerimeters.cpp:58
        let mut has_intersection = false;
        // RetractWhenCrossingPerimeters.cpp:59
        for line in &travel.lines() {
            // RetractWhenCrossingPerimeters.cpp:60
            // Check if the travel line intersects with any of the internal islands.
            // RetractWhenCrossingPerimeters.cpp:61
            let intersections = self
                .m_aabbtree_lines_distancer
                .intersections_with_line::<false>(line);
            // RetractWhenCrossingPerimeters.cpp:62
            if !intersections.is_empty() {
                // RetractWhenCrossingPerimeters.cpp:63
                has_intersection = true;
                // RetractWhenCrossingPerimeters.cpp:64
                break;
            }
        }

        // RetractWhenCrossingPerimeters.cpp:68
        has_intersection
    }

    // RetractWhenCrossingPerimeters.cpp:71-111
    fn travel_inside_internal_regions(&mut self, layer: &Layer, travel: &Polyline) -> bool {
        // RetractWhenCrossingPerimeters.cpp:73
        if self.m_layer != Some(layer.id()) {
            // RetractWhenCrossingPerimeters.cpp:74
            self.cross_perimeters_flag = false;
            // RetractWhenCrossingPerimeters.cpp:75-76
            // Update cache.
            self.m_layer = Some(layer.id());
            // RetractWhenCrossingPerimeters.cpp:77
            self.m_internal_islands.clear();
            // RetractWhenCrossingPerimeters.cpp:78
            self.m_aabbtree_internal_islands.clear();
            // RetractWhenCrossingPerimeters.cpp:79
            // Collect expolygons of internal slices.
            // RetractWhenCrossingPerimeters.cpp:80-82
            for layerm in layer.regions() {
                for surface in &layerm.get_slices().surfaces {
                    if surface.is_internal() {
                        // C++ stores `&surface.expolygon` (a pointer into layer data).
                        // We clone the ExPolygon since Rust cannot hold those borrows
                        // for the lifetime of the cache; semantics are identical.
                        self.m_internal_islands.push(surface.expolygon.clone());
                    }
                }
            }
            // RetractWhenCrossingPerimeters.cpp:83-84
            // Calculate bounding boxes of internal slices.
            let mut bboxes: Vec<tree2d::BoundingBoxWrapper> = Vec::new();
            // RetractWhenCrossingPerimeters.cpp:85
            bboxes.reserve(self.m_internal_islands.len());
            // RetractWhenCrossingPerimeters.cpp:86
            for i in 0..self.m_internal_islands.len() {
                bboxes.push(tree2d::BoundingBoxWrapper::new(
                    i,
                    &crate::geometry::get_extents_expoly(&self.m_internal_islands[i]),
                ));
            }
            // RetractWhenCrossingPerimeters.cpp:87-88
            // Build AABB tree over bounding boxes of internal slices.
            self.m_aabbtree_internal_islands.build_modify_input(&mut bboxes);
        }

        // RetractWhenCrossingPerimeters.cpp:91
        // BoundingBox get_extents(const Polyline &polyline) { return polyline.bounding_box(); }
        let mut bbox_travel = travel.bounding_box();
        // RetractWhenCrossingPerimeters.cpp:92
        // AABBTree::BoundingBox bbox_travel_eigen{bbox_travel.min, bbox_travel.max};
        let bbox_travel_eigen = tree2d::BoundingBox {
            min: [bbox_travel.min.x as f64, bbox_travel.min.y as f64],
            max: [bbox_travel.max.x as f64, bbox_travel.max.y as f64],
        };
        // RetractWhenCrossingPerimeters.cpp:93
        let mut result: i32 = -1;
        // RetractWhenCrossingPerimeters.cpp:94
        // `BoundingBox::offset(coordf_t)` adds Point(delta, delta) to each side
        // (delta = SCALED_EPSILON = 10.0, truncated to coord_t). `expand` is the
        // crate's faithful equivalent for the (defined) travel bounding box.
        bbox_travel.expand(SCALED_EPSILON as i64);
        // RetractWhenCrossingPerimeters.cpp:95-109
        let islands = &self.m_internal_islands;
        tree2d::traverse(
            &self.m_aabbtree_internal_islands,
            // RetractWhenCrossingPerimeters.cpp:96
            // [&bbox_travel_eigen](const Node &node){ return bbox_travel_eigen.intersects(node.bbox); }
            |node: &tree2d::Node| bbox_travel_eigen.intersects(&node.bbox),
            // RetractWhenCrossingPerimeters.cpp:97-108
            |node: &tree2d::Node| {
                // RetractWhenCrossingPerimeters.cpp:98-99
                debug_assert!(node.is_leaf());
                debug_assert!(node.is_valid());
                // RetractWhenCrossingPerimeters.cpp:100
                // Polygons clipped = ClipperUtils::clip_clipper_polygons_with_subject_bbox(*islands[node.idx], bbox_travel);
                // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib —
                // clip_clipper_polygons_with_subject_bbox routes through the `geo`
                // crate (fixed scale 1000) rather than ClipperLib at coord_t integer
                // precision, so the clipped polygons may differ at the sub-bbox edges.
                let clipped = clipper_utils::clip_clipper_polygons_with_subject_bbox_expolygon(
                    &islands[node.idx],
                    &bbox_travel,
                    false,
                );
                // RetractWhenCrossingPerimeters.cpp:101
                // if (diff_pl(travel, clipped).empty()) {
                // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib —
                // diff_pl (inside diff_pl_polygons) uses the geo crate, not ClipperLib;
                // the "travel is fully inside the island" test can differ by a hair near
                // boundaries. Logic/control flow is faithful; the primitive is foundational.
                if diff_pl_polygons(travel, &clipped).is_empty() {
                    // RetractWhenCrossingPerimeters.cpp:102
                    // Travel path is completely inside an "internal" island. Don't retract.
                    // RetractWhenCrossingPerimeters.cpp:103
                    result = node.idx as i32;
                    // RetractWhenCrossingPerimeters.cpp:104-105
                    // Stop traversal.
                    return false;
                }
                // RetractWhenCrossingPerimeters.cpp:107-108
                // Continue traversal.
                true
            },
        );
        // RetractWhenCrossingPerimeters.cpp:110
        result != -1
    }

    // RetractWhenCrossingPerimeters.cpp:113-117
    pub fn travel_inside_internal_regions_no_wall_crossing(
        &mut self,
        layer: &Layer,
        travel: &Polyline,
    ) -> bool {
        // RetractWhenCrossingPerimeters.cpp:115
        if !self.travel_inside_internal_regions(layer, travel) {
            return false;
        }
        // RetractWhenCrossingPerimeters.cpp:116
        !self.travel_cross_perimeters(layer, travel)
    }
}

impl Default for RetractWhenCrossingPerimeters {
    fn default() -> Self {
        Self::new()
    }
}

/// `diff_pl(const Polyline &subject, const Polygons &clip)` (ClipperUtils.cpp:909-910):
/// returns the portions of `travel` lying OUTSIDE the region described by `clip`.
///
/// `clip` is the contour+holes `Polygons` produced by
/// `clip_clipper_polygons_with_subject_bbox`, so the first polygon (if any) is the
/// outer contour and the rest are holes; we reassemble that single island into an
/// `ExPolygon` and reuse `clipper_utils::diff_pl` (which clips against ExPolygons),
/// matching the C++ `PolygonsProvider` winding semantics for one island.
fn diff_pl_polygons(travel: &Polyline, clip: &[crate::geometry::Polygon]) -> Vec<Polyline> {
    if clip.is_empty() {
        // ClipperLib difference against an empty clip set returns the subject.
        return vec![travel.clone()];
    }
    let contour = clip[0].clone();
    let holes = clip[1..].to_vec();
    let expoly = ExPolygon::with_holes(contour, holes);
    clipper_utils::diff_pl(std::slice::from_ref(travel), std::slice::from_ref(&expoly))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let checker = RetractWhenCrossingPerimeters::new();
        assert!(checker.m_layer.is_none());
        assert!(checker.m_internal_islands.is_empty());
        assert!(!checker.cross_perimeters_flag);
    }
}
