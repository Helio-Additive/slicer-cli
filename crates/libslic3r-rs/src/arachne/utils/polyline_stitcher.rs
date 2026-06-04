//! Polyline stitcher for Arachne - stitches polylines into longer polylines or polygons
//!
//! C++ Reference:
//! - Arachne/utils/PolylineStitcher.hpp
//! - Arachne/utils/PolylineStitcher.cpp
//!
//! **STATUS:** ✅ COMPLETE - Full implementation with C++ parity

use super::polygons_point_index::{PathsPointIndex, PathsPointIndexLocator};
use super::sparse_point_grid::SparsePointGrid;

use crate::geometry::{Point, Polygon, Polygons};
use crate::scaled;

/// Class for stitching polylines into longer polylines or into polygons
///
/// C++ Reference: Arachne/utils/PolylineStitcher.hpp:17-207
/// C++: template<typename Paths, typename Path, typename Junction>
/// C++: class PolylineStitcher
/// C++: {
/// C++: public:
/// C++:     static void stitch(const Paths& lines, Paths& result_lines, Paths& result_polygons,
/// C++:                        coord_t max_stitch_distance = scaled<coord_t>(0.1),
/// C++:                        coord_t snap_distance = scaled<coord_t>(0.01))
/// C++:     // ... implementation ...
/// C++: };
pub struct PolylineStitcher;

impl PolylineStitcher {
    /// Stitch together the separate lines into result_lines and if they can be closed into result_polygons
    ///
    /// Only introduce new segments shorter than max_stitch_distance, and larger than snap_distance
    /// but always try to take the shortest connection possible.
    ///
    /// Only stitch polylines into closed polygons if they are larger than 3 * max_stitch_distance,
    /// in order to prevent small segments to accidentally get closed into a polygon.
    ///
    /// **Warning:** Tiny polylines (smaller than 3 * max_stitch_distance) will not be closed into polygons.
    ///
    /// **Note:** Resulting polylines and polygons are added onto the existing containers.
    ///
    /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:51-191
    /// C++: static void stitch(const Paths& lines, Paths& result_lines, Paths& result_polygons,
    /// C++:                    coord_t max_stitch_distance = scaled<coord_t>(0.1),
    /// C++:                    coord_t snap_distance = scaled<coord_t>(0.01))
    pub fn stitch_polygons(
        lines: &Polygons,
        result_lines: &mut Polygons,
        result_polygons: &mut Polygons,
        max_stitch_distance: i64,
        snap_distance: i64,
    ) {
        if lines.is_empty() {
            return;
        }

        /// Create spatial grid for efficient nearest neighbor queries
        /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:57
        /// C++: SparsePointGrid<PathsPointIndex<Paths>, PathsPointIndexLocator<Paths>> grid(max_stitch_distance, lines.size() * 2);
        let mut grid = SparsePointGrid::<PathsPointIndex, PathsPointIndexLocator>::new(
            max_stitch_distance,
            lines.len() * 2,
            1.0,
        );

        /// Populate grid with start and end points of each line
        /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:59-64
        /// C++: for (size_t line_idx = 0; line_idx < lines.size(); line_idx++)
        /// C++: {
        /// C++:     const auto line = lines[line_idx];
        /// C++:     grid.insert(PathsPointIndex<Paths>(&lines, line_idx, 0));
        /// C++:     grid.insert(PathsPointIndex<Paths>(&lines, line_idx, line.size() - 1));
        /// C++: }
        for line_idx in 0..lines.len() {
            let line = &lines[line_idx];
            if !line.points.is_empty() {
                grid.insert(PathsPointIndex::with_indices(lines, line_idx, 0));
                grid.insert(PathsPointIndex::with_indices(
                    lines,
                    line_idx,
                    line.points.len() - 1,
                ));
            }
        }

        /// Track which lines have been processed
        /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:66
        /// C++: std::vector<bool> processed(lines.size(), false);
        let mut processed = vec![false; lines.len()];

        /// Process each line
        /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:68-189
        /// C++: for (size_t line_idx = 0; line_idx < lines.size(); line_idx++)
        for line_idx in 0..lines.len() {
            /// Skip if already processed
            /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:70-73
            /// C++: if (processed[line_idx])
            /// C++: {
            /// C++:     continue;
            /// C++: }
            if processed[line_idx] {
                continue;
            }

            // Mark as processed
            // C++ Reference: Arachne/utils/PolylineStitcher.hpp:74
            // C++: processed[line_idx] = true;
            processed[line_idx] = true;

            /// Get the line
            /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:75
            /// C++: const auto line = lines[line_idx];
            let line = &lines[line_idx];

            /// Check if should close (for polygons)
            /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:76
            /// C++: bool should_close = isOdd(line);
            let mut should_close = Self::is_odd_polygon(line);

            /// Start the chain with current line
            /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:78
            /// C++: Path chain = line;
            let mut chain = line.clone();

            /// Track if we found a closing segment
            /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:79
            /// C++: bool closest_is_closing_polygon = false;
            let mut closest_is_closing_polygon = false;

            /// Try extending chain in both directions
            /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:80-174
            /// C++: for (bool go_in_reverse_direction : { false, true })
            for go_in_reverse_direction in [false, true] {
                /// Reverse chain on second iteration
                /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:82-85
                /// C++: if (go_in_reverse_direction)
                /// C++: {
                /// C++:     chain.reverse();
                /// C++: }
                if go_in_reverse_direction {
                    chain.reverse();
                }

                /// Track chain length
                /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:86
                /// C++: int64_t chain_length = chain.polylineLength();
                let mut chain_length = (chain.perimeter() * 1_000_000.0) as i64;

                /// Keep extending chain
                /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:88-168
                /// C++: while (true)
                loop {
                    /// Get the endpoint to extend from
                    /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:90
                    /// C++: Point from = make_point(chain.back());
                    let from = *chain.points.last().unwrap();

                    /// Find nearest unprocessed endpoint
                    /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:92-94
                    /// C++: PathsPointIndex<Paths> closest;
                    /// C++: coord_t closest_distance = std::numeric_limits<coord_t>::max();
                    let mut closest: Option<PathsPointIndex> = None;
                    let mut closest_distance = i64::MAX;
                    let mut is_closing = false;

                    /// Search nearby endpoints in grid
                    /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:95-156
                    /// C++: grid.processNearby(from, max_stitch_distance, ...)
                    grid.process_nearby(from, max_stitch_distance, |nearby| {
                        /// Calculate distance to nearby point
                        /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:100
                        /// C++: coord_t dist = (nearby.p().template cast<int64_t>() - from.template cast<int64_t>()).norm();
                        let nearby_p = nearby.p();
                        let dx = nearby_p.x() as i64 - from.x() as i64;
                        let dy = nearby_p.y() as i64 - from.y() as i64;
                        let mut dist = ((dx * dx + dy * dy) as f64).sqrt() as i64;

                        /// Skip if too far
                        /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:101-104
                        /// C++: if (dist > max_stitch_distance)
                        /// C++: {
                        /// C++:     return true;
                        /// C++: }
                        if dist > max_stitch_distance {
                            return true; // keep looking
                        }

                        /// Check if this would close the polygon
                        /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:105-124
                        let front_p = chain.points[0];
                        let dx2 = nearby_p.x() as i64 - front_p.x() as i64;
                        let dy2 = nearby_p.y() as i64 - front_p.y() as i64;
                        let dist_to_front = dx2 * dx2 + dy2 * dy2;

                        let mut is_closing_segment = false;
                        if dist_to_front < snap_distance * snap_distance {
                            /// Would close the polygon
                            /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:107-120
                            /// C++: if (chain_length + dist < 3 * max_stitch_distance || chain.size() <= 2)
                            /// C++: {
                            /// C++:     return true;
                            /// C++: }
                            if chain_length + dist < 3 * max_stitch_distance
                                || chain.points.len() <= 2
                            {
                                return true; // too small to close
                            }

                            is_closing_segment = true;

                            // Adjust distance based on should_close preference
                            // C++ Reference: Arachne/utils/PolylineStitcher.hpp:114-121
                            if should_close {
                                dist = dist.saturating_sub(scaled(0.01));
                            } else {
                                dist = dist.saturating_add(scaled(0.01));
                            }
                        } else if processed[nearby.poly_idx] {
                            /// Already processed
                            /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:122-125
                            /// C++: else if (processed[nearby.poly_idx])
                            /// C++: {
                            /// C++:     return true;
                            /// C++: }
                            return true;
                        }

                        /// Check if we can reverse the nearby line
                        /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:126-130
                        /// C++: bool nearby_would_be_reversed = nearby.point_idx != 0;
                        /// C++: nearby_would_be_reversed = nearby_would_be_reversed != go_in_reverse_direction;
                        /// C++: if (!canReverse(nearby) && nearby_would_be_reversed)
                        /// C++: {
                        /// C++:     return true;
                        /// C++: }
                        let nearby_would_be_reversed =
                            (nearby.point_idx != 0) != go_in_reverse_direction;
                        if !Self::can_reverse_polygon(&nearby) && nearby_would_be_reversed {
                            return true;
                        }

                        /// Check if we can connect these paths
                        /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:131-134
                        /// C++: if (!canConnect(chain, (*nearby.polygons)[nearby.poly_idx]))
                        /// C++: {
                        /// C++:     return true;
                        /// C++: }
                        if let Some(nearby_polys) = nearby.polygons {
                            if !Self::can_connect_polygon(&chain, &nearby_polys[nearby.poly_idx]) {
                                return true;
                            }
                        }

                        /// Update closest if this is better
                        /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:135-143
                        /// C++: if (dist < closest_distance)
                        /// C++: {
                        /// C++:     closest_distance = dist;
                        /// C++:     closest = nearby;
                        /// C++:     closest_is_closing_polygon = is_closing_segment;
                        /// C++: }
                        if dist < closest_distance {
                            closest_distance = dist;
                            closest = Some(*nearby);
                            is_closing = is_closing_segment;
                        }

                        /// Stop if we found a snap-close match
                        /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:144-147
                        /// C++: if (dist < snap_distance)
                        /// C++: {
                        /// C++:     return false;
                        /// C++: }
                        if dist < snap_distance {
                            return false; // stop looking
                        }

                        true // keep processing
                    });

                    closest_is_closing_polygon = is_closing;

                    /// Break if no more connections or closed
                    /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:158-162
                    /// C++: if (!closest.initialized() || closest_is_closing_polygon)
                    /// C++: {
                    /// C++:     break;
                    /// C++: }
                    if closest.is_none() || closest_is_closing_polygon {
                        break;
                    }

                    let closest_idx = closest.unwrap();

                    /// Append the closest line to our chain
                    /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:164-182
                    if let Some(polys) = closest_idx.polygons {
                        let nearby_line = &polys[closest_idx.poly_idx];
                        let old_size = chain.points.len();

                        if closest_idx.point_idx == 0 {
                            /// Append forward
                            /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:167-173
                            let mut start_pos = 0;
                            if closest_distance < snap_distance {
                                start_pos = 1;
                            }
                            for i in start_pos..nearby_line.points.len() {
                                chain.points.push(nearby_line.points[i]);
                            }
                        } else {
                            /// Append reversed
                            /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:175-181
                            let mut start_idx = nearby_line.points.len();
                            if closest_distance < snap_distance {
                                start_idx -= 1;
                            }
                            for i in (0..start_idx).rev() {
                                chain.points.push(nearby_line.points[i]);
                            }
                        }

                        /// Update chain length
                        /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:183-186
                        /// C++: for(size_t i = old_size; i < chain.size(); ++i)
                        /// C++: {
                        /// C++:     chain_length += (make_point(chain[i]).template cast<int64_t>() - make_point(chain[i - 1]).template cast<int64_t>()).norm();
                        /// C++: }
                        for i in old_size..chain.points.len() {
                            let p1 = chain.points[i];
                            let p2 = chain.points[i - 1];
                            let dx = p1.x() as i64 - p2.x() as i64;
                            let dy = p1.y() as i64 - p2.y() as i64;
                            chain_length += ((dx * dx + dy * dy) as f64).sqrt() as i64;
                        }

                        // Update should_close flag
                        // C++ Reference: Arachne/utils/PolylineStitcher.hpp:187
                        // C++: should_close = should_close & !isOdd((*closest.polygons)[closest.poly_idx]);
                        should_close = should_close && !Self::is_odd_polygon(nearby_line);

                        // Mark as processed
                        // C++ Reference: Arachne/utils/PolylineStitcher.hpp:188
                        // C++: processed[closest.poly_idx] = true;
                        processed[closest_idx.poly_idx] = true;
                    }
                }

                /// Break if we closed the polygon
                /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:169-174
                /// C++: if (closest_is_closing_polygon)
                /// C++: {
                /// C++:     if (go_in_reverse_direction)
                /// C++:     {
                /// C++:         chain.reverse();
                /// C++:     }
                /// C++:     break;
                /// C++: }
                if closest_is_closing_polygon {
                    if go_in_reverse_direction {
                        chain.reverse();
                    }
                    break;
                }
            }

            /// Add to result polygons or result lines
            /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:175-189
            /// C++: if (closest_is_closing_polygon)
            /// C++: {
            /// C++:     result_polygons.emplace_back(chain);
            /// C++: }
            /// C++: else
            /// C++: {
            /// C++:     PathsPointIndex<Paths> ppi_here(&lines, line_idx, 0);
            /// C++:     if ( ! canReverse(ppi_here))
            /// C++:     {
            /// C++:         chain.reverse();
            /// C++:     }
            /// C++:     result_lines.emplace_back(chain);
            /// C++: }
            if closest_is_closing_polygon {
                result_polygons.push(chain);
            } else {
                let ppi_here = PathsPointIndex::with_indices(lines, line_idx, 0);
                if !Self::can_reverse_polygon(&ppi_here) {
                    chain.reverse();
                }
                result_lines.push(chain);
            }
        }
    }

    /// Whether a polyline is allowed to be reversed (always true for Polygons)
    ///
    /// C++ Reference: Arachne/utils/PolylineStitcher.cpp:9-15
    /// C++: template<> bool PolylineStitcher<Polygons, Polygon, Point>::canReverse(const PathsPointIndex<Polygons> &)
    /// C++: {
    /// C++:     return true;
    /// C++: }
    pub fn can_reverse_polygon(_polyline: &PathsPointIndex) -> bool {
        true
    }

    /// Whether two paths are allowed to be connected (always true for Polygons)
    ///
    /// C++ Reference: Arachne/utils/PolylineStitcher.cpp:17-21
    /// C++: template<> bool PolylineStitcher<Polygons, Polygon, Point>::canConnect(const Polygon &, const Polygon &)
    /// C++: {
    /// C++:     return true;
    /// C++: }
    pub fn can_connect_polygon(_a: &Polygon, _b: &Polygon) -> bool {
        true
    }

    /// Check if a polygon is odd (always false for Polygons)
    ///
    /// C++ Reference: Arachne/utils/PolylineStitcher.cpp:29-33
    /// C++: template<> bool PolylineStitcher<Polygons, Polygon, Point>::isOdd(const Polygon &)
    /// C++: {
    /// C++:     return false;
    /// C++: }
    pub fn is_odd_polygon(_line: &Polygon) -> bool {
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon, Polygons};

    fn create_simple_line(points: Vec<(i64, i64)>) -> Polygon {
        let mut poly = Polygon::new();
        for (x, y) in points {
            poly.points.push(Point::new(x, y));
        }
        poly
    }

    #[test]
    fn test_polyline_stitcher_empty() {
        /// Test stitching empty lines
        /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:53-56
        let lines = Polygons::new();
        let mut result_lines = Polygons::new();
        let mut result_polygons = Polygons::new();

        PolylineStitcher::stitch_polygons(
            &lines,
            &mut result_lines,
            &mut result_polygons,
            100_000, // 0.1mm scaled
            10_000,  // 0.01mm scaled
        );

        assert_eq!(result_lines.len(), 0);
        assert_eq!(result_polygons.len(), 0);
    }

    #[test]
    fn test_polyline_stitcher_single_line() {
        /// Test stitching single line (no connections)
        let mut lines = Polygons::new();
        lines.push(create_simple_line(vec![(0, 0), (100, 0), (100, 100)]));

        let mut result_lines = Polygons::new();
        let mut result_polygons = Polygons::new();

        PolylineStitcher::stitch_polygons(
            &lines,
            &mut result_lines,
            &mut result_polygons,
            100_000, // 0.1mm scaled
            10_000,  // 0.01mm scaled
        );

        assert_eq!(result_lines.len(), 1);
        assert_eq!(result_polygons.len(), 0);
    }

    #[test]
    fn test_polyline_stitcher_close_polygon() {
        /// Test stitching lines that form a closed polygon
        /// C++ Reference: Arachne/utils/PolylineStitcher.hpp:175-178
        let mut lines = Polygons::new();
        // Line 1: bottom
        lines.push(create_simple_line(vec![(0, 0), (100, 0)]));
        // Line 2: right
        lines.push(create_simple_line(vec![(100, 0), (100, 100)]));
        // Line 3: top
        lines.push(create_simple_line(vec![(100, 100), (0, 100)]));
        // Line 4: left (closes the square)
        lines.push(create_simple_line(vec![(0, 100), (0, 0)]));

        let mut result_lines = Polygons::new();
        let mut result_polygons = Polygons::new();

        PolylineStitcher::stitch_polygons(
            &lines,
            &mut result_lines,
            &mut result_polygons,
            100_000, // 0.1mm scaled
            10_000,  // 0.01mm scaled
        );

        // Should form a closed polygon
        assert_eq!(result_polygons.len(), 1);
        assert_eq!(result_lines.len(), 0);
    }

    #[test]
    fn test_polyline_stitcher_can_reverse() {
        /// Test can_reverse for polygons (always true)
        /// C++ Reference: Arachne/utils/PolylineStitcher.cpp:15
        let lines = Polygons::new();
        let ppi = PathsPointIndex::with_indices(&lines, 0, 0);
        assert!(PolylineStitcher::can_reverse_polygon(&ppi));
    }

    #[test]
    fn test_polyline_stitcher_can_connect() {
        /// Test can_connect for polygons (always true)
        /// C++ Reference: Arachne/utils/PolylineStitcher.cpp:21
        let poly1 = Polygon::new();
        let poly2 = Polygon::new();
        assert!(PolylineStitcher::can_connect_polygon(&poly1, &poly2));
    }

    #[test]
    fn test_polyline_stitcher_is_odd() {
        /// Test is_odd for polygons (always false)
        /// C++ Reference: Arachne/utils/PolylineStitcher.cpp:33
        let poly = Polygon::new();
        assert!(!PolylineStitcher::is_odd_polygon(&poly));
    }

    #[test]
    fn test_polyline_stitcher_two_lines() {
        /// Test stitching two lines that connect
        let mut lines = Polygons::new();
        lines.push(create_simple_line(vec![(0, 0), (50, 0)]));
        lines.push(create_simple_line(vec![(50, 0), (100, 0)]));

        let mut result_lines = Polygons::new();
        let mut result_polygons = Polygons::new();

        PolylineStitcher::stitch_polygons(
            &lines,
            &mut result_lines,
            &mut result_polygons,
            100_000, // 0.1mm scaled
            10_000,  // 0.01mm scaled
        );

        // Should stitch into one line
        assert_eq!(result_lines.len(), 1);
        assert_eq!(result_polygons.len(), 0);
        assert!(result_lines[0].points.len() >= 3);
    }
}
