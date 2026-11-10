//Copyright (c) 2022 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.
//
// 1:1 faithful port of:
//   Arachne/utils/PolylineStitcher.cpp
//   Arachne/utils/PolylineStitcher.hpp
//
// coord_t -> i64 (Coord), coordf_t -> f64 (CoordF), Point mirrors C++ Slic3r::Point.
//
// NOTE ON SPECIALIZATIONS:
// The C++ class is `template<typename Paths, typename Path, typename Junction>
// PolylineStitcher`, with two explicit instantiations in the .cpp:
//   * PolylineStitcher<Polygons, Polygon, Point>                   (fully ported below)
//   * PolylineStitcher<VariableWidthLines, ExtrusionLine, ExtrusionJunction>
// The `VariableWidthLines/ExtrusionLine` instantiation requires a
// `PathsPointIndex<VariableWidthLines>` (and a `SparsePointGrid` over it). The
// Rust `PathsPointIndex` (see polygons_point_index.rs) is currently hard-wired
// to `&Polygons`, so the `stitch` template body for that instantiation is
// BLOCKED on that not-yet-generic dependency. The three helper specializations
// (`canReverse`, `canConnect`, `isOdd`) only touch resolved `ExtrusionLine`
// fields, so they ARE ported below (taking `&ExtrusionLine` directly).

use super::polygons_point_index::{PathsPointIndex, PathsPointIndexLocator};
use super::sparse_point_grid::{LocatorTrait, SparsePointGrid};

use crate::arachne::utils::extrusion_line::{ExtrusionLine, VariableWidthLines};
use crate::geometry::{Point, Polygon, Polygons};
use crate::scaled;

/// Class for stitching polylines into longer polylines or into polygons
///
/// PolylineStitcher.hpp:19-21
/// template<typename Paths, typename Path, typename Junction>
/// class PolylineStitcher
pub struct PolylineStitcher;

impl PolylineStitcher {
    /// Stitch together the separate `lines` into `result_lines` and if they
    /// can be closed into `result_polygons`.
    ///
    /// Only introduce new segments shorter than `max_stitch_distance`, and
    /// larger than `snap_distance` but always try to take the shortest
    /// connection possible.
    ///
    /// Only stitch polylines into closed polygons if they are larger than 3 *
    /// `max_stitch_distance`, in order to prevent small segments to
    /// accidentally get closed into a polygon.
    ///
    /// \warning Tiny polylines (smaller than 3 * max_stitch_distance) will not
    /// be closed into polygons.
    ///
    /// PolylineStitcher.hpp:53 (PolylineStitcher<Polygons, Polygon, Point> instantiation)
    /// static void stitch(const Paths& lines, Paths& result_lines, Paths& result_polygons,
    ///                    coord_t max_stitch_distance = scaled<coord_t>(0.1),
    ///                    coord_t snap_distance = scaled<coord_t>(0.01))
    pub fn stitch_polygons(
        lines: &Polygons,
        result_lines: &mut Polygons,
        result_polygons: &mut Polygons,
        max_stitch_distance: i64,
        snap_distance: i64,
    ) {
        // PolylineStitcher.hpp:55-56
        if lines.is_empty() {
            return;
        }

        // PolylineStitcher.hpp:58
        // SparsePointGrid<PathsPointIndex<Paths>, PathsPointIndexLocator<Paths>> grid(max_stitch_distance, lines.size() * 2);
        let mut grid = SparsePointGrid::<PathsPointIndex, PathsPointIndexLocator>::new(
            max_stitch_distance,
            lines.len() * 2,
            1.0,
        );

        // populate grid
        // PolylineStitcher.hpp:61-66
        // for (size_t line_idx = 0; line_idx < lines.size(); line_idx++)
        // {
        //     const auto line = lines[line_idx];
        //     grid.insert(PathsPointIndex<Paths>(&lines, line_idx, 0));
        //     grid.insert(PathsPointIndex<Paths>(&lines, line_idx, line.size() - 1));
        // }
        for line_idx in 0..lines.len() {
            let line = &lines[line_idx];
            grid.insert(PathsPointIndex::with_indices(lines, line_idx, 0));
            grid.insert(PathsPointIndex::with_indices(
                lines,
                line_idx,
                line.points.len() - 1,
            ));
        }

        // PolylineStitcher.hpp:68
        // std::vector<bool> processed(lines.size(), false);
        let mut processed = vec![false; lines.len()];

        // PolylineStitcher.hpp:70
        // for (size_t line_idx = 0; line_idx < lines.size(); line_idx++)
        for line_idx in 0..lines.len() {
            // PolylineStitcher.hpp:72-75
            // if (processed[line_idx])
            // {
            //     continue;
            // }
            if processed[line_idx] {
                continue;
            }
            // PolylineStitcher.hpp:76
            processed[line_idx] = true;
            // PolylineStitcher.hpp:77
            // const auto line = lines[line_idx];
            let line = &lines[line_idx];
            // PolylineStitcher.hpp:78
            // bool should_close = isOdd(line);
            let mut should_close = Self::is_odd_polygon(line);

            // PolylineStitcher.hpp:80
            // Path chain = line;
            let mut chain = line.clone();
            // PolylineStitcher.hpp:81
            // bool closest_is_closing_polygon = false;
            let mut closest_is_closing_polygon = false;
            // PolylineStitcher.hpp:82-83
            // first go in the unreversed direction, to try to prevent the chain.reverse() operation.
            // NOTE: Implementation only works for this order; we currently only re-reverse the chain when it's closed.
            for go_in_reverse_direction in [false, true] {
                // PolylineStitcher.hpp:84-87
                // if (go_in_reverse_direction)
                // { // try extending chain in the other direction
                //     chain.reverse();
                // }
                if go_in_reverse_direction {
                    chain.reverse();
                }
                // PolylineStitcher.hpp:88
                // int64_t chain_length = chain.polylineLength();
                // (Polygon == MultiPoint: polylineLength() is the OPEN polyline length,
                //  i.e. the sum of consecutive segments, NOT including the closing edge.)
                let mut chain_length: i64 = polyline_length(&chain);

                // PolylineStitcher.hpp:90
                // while (true)
                loop {
                    // PolylineStitcher.hpp:92
                    // Point from = make_point(chain.back());
                    let from = *chain.points.last().unwrap();

                    // PolylineStitcher.hpp:94-95
                    // PathsPointIndex<Paths> closest;
                    // coord_t closest_distance = std::numeric_limits<coord_t>::max();
                    let mut closest: Option<PathsPointIndex> = None;
                    let mut closest_distance = i64::MAX;

                    // PolylineStitcher.hpp:96-153
                    // grid.processNearby(from, max_stitch_distance,
                    //     std::function<bool (const PathsPointIndex<Paths>&)> ([...](const PathsPointIndex<Paths>& nearby)->bool { ... }));
                    grid.process_nearby(from, max_stitch_distance, |nearby| {
                        // PolylineStitcher.hpp:101
                        // bool is_closing_segment = false;
                        let mut is_closing_segment = false;
                        // PolylineStitcher.hpp:102
                        // coord_t dist = (nearby.p().template cast<int64_t>() - from.template cast<int64_t>()).norm();
                        let nearby_p = nearby.p();
                        let dx = nearby_p.x() - from.x();
                        let dy = nearby_p.y() - from.y();
                        let mut dist = (((dx * dx + dy * dy) as f64).sqrt()) as i64;
                        // PolylineStitcher.hpp:103-106
                        // if (dist > max_stitch_distance)
                        // {
                        //     return true; // keep looking
                        // }
                        if dist > max_stitch_distance {
                            return true; // keep looking
                        }
                        // PolylineStitcher.hpp:107
                        // if ((nearby.p().template cast<int64_t>() - make_point(chain.front()).template cast<int64_t>()).squaredNorm() < snap_distance * snap_distance)
                        let front_p = chain.points[0];
                        let dx2 = nearby_p.x() - front_p.x();
                        let dy2 = nearby_p.y() - front_p.y();
                        let dist_to_front = dx2 * dx2 + dy2 * dy2;
                        if dist_to_front < snap_distance * snap_distance {
                            // PolylineStitcher.hpp:109-113
                            // if (chain_length + dist < 3 * max_stitch_distance // prevent closing of small poly, cause it might be able to continue making a larger polyline
                            //     || chain.size() <= 2) // don't make 2 vert polygons
                            // {
                            //     return true; // look for a better next line
                            // }
                            if chain_length + dist < 3 * max_stitch_distance
                                || chain.points.len() <= 2
                            {
                                return true; // look for a better next line
                            }
                            // PolylineStitcher.hpp:114
                            is_closing_segment = true;
                            // PolylineStitcher.hpp:115-125
                            if !should_close {
                                // PolylineStitcher.hpp:117
                                // dist += scaled<coord_t>(0.01); // prefer continuing polyline over closing a polygon; avoids closed zigzags from being printed separately
                                dist += scaled(0.01);
                                // continue to see if closing segment is also the closest
                                // there might be a segment smaller than [max_stitch_distance] which closes the polygon better
                            } else {
                                // PolylineStitcher.hpp:123
                                // dist -= scaled<coord_t>(0.01); //Prefer closing the polygon if it's 100% even lines. Used to create closed contours.
                                dist -= scaled(0.01);
                                //Continue to see if closing segment is also the closest.
                            }
                        } else if processed[nearby.poly_idx] {
                            // PolylineStitcher.hpp:127-130
                            // else if (processed[nearby.poly_idx])
                            // { // it was already moved to output
                            //     return true; // keep looking for a connection
                            // }
                            return true; // keep looking for a connection
                        }
                        // PolylineStitcher.hpp:131
                        // bool nearby_would_be_reversed = nearby.point_idx != 0;
                        let mut nearby_would_be_reversed = nearby.point_idx != 0;
                        // PolylineStitcher.hpp:132
                        // nearby_would_be_reversed = nearby_would_be_reversed != go_in_reverse_direction; // flip nearby_would_be_reversed when searching in the reverse direction
                        nearby_would_be_reversed = nearby_would_be_reversed != go_in_reverse_direction;
                        // PolylineStitcher.hpp:133-136
                        // if (!canReverse(nearby) && nearby_would_be_reversed)
                        // { // connecting the segment would reverse the polygon direction
                        //     return true; // keep looking for a connection
                        // }
                        if !Self::can_reverse_polygon(nearby) && nearby_would_be_reversed {
                            return true; // keep looking for a connection
                        }
                        // PolylineStitcher.hpp:137-140
                        // if (!canConnect(chain, (*nearby.polygons)[nearby.poly_idx]))
                        // {
                        //     return true; // keep looking for a connection
                        // }
                        if let Some(nearby_polys) = nearby.polygons {
                            if !Self::can_connect_polygon(&chain, &nearby_polys[nearby.poly_idx]) {
                                return true; // keep looking for a connection
                            }
                        }
                        // PolylineStitcher.hpp:141-146
                        // if (dist < closest_distance)
                        // {
                        //     closest_distance = dist;
                        //     closest = nearby;
                        //     closest_is_closing_polygon = is_closing_segment;
                        // }
                        if dist < closest_distance {
                            closest_distance = dist;
                            closest = Some(*nearby);
                            closest_is_closing_polygon = is_closing_segment;
                        }
                        // PolylineStitcher.hpp:147-150
                        // if (dist < snap_distance)
                        // { // we have found a good enough next line
                        //     return false; // stop looking for alternatives
                        // }
                        if dist < snap_distance {
                            return false; // stop looking for alternatives
                        }
                        // PolylineStitcher.hpp:151
                        true // keep processing elements
                    });

                    // PolylineStitcher.hpp:155-160
                    // if (!closest.initialized()          // we couldn't find any next line
                    //     || closest_is_closing_polygon   // we closed the polygon
                    // )
                    // {
                    //     break;
                    // }
                    if closest.is_none() || closest_is_closing_polygon {
                        break;
                    }

                    let closest = closest.unwrap();

                    // PolylineStitcher.hpp:162
                    // coord_t segment_dist = (make_point(chain.back()).template cast<int64_t>() - closest.p().template cast<int64_t>()).norm();
                    let back_p = *chain.points.last().unwrap();
                    let closest_p = closest.p();
                    let sdx = back_p.x() - closest_p.x();
                    let sdy = back_p.y() - closest_p.y();
                    let segment_dist = (((sdx * sdx + sdy * sdy) as f64).sqrt()) as i64;
                    // PolylineStitcher.hpp:163
                    // assert(segment_dist <= max_stitch_distance + scaled<coord_t>(0.01));
                    debug_assert!(segment_dist <= max_stitch_distance + scaled(0.01));
                    // PolylineStitcher.hpp:164
                    // const size_t old_size = chain.size();
                    let old_size = chain.points.len();
                    if let Some(polys) = closest.polygons {
                        let nearby_line = &polys[closest.poly_idx];
                        // PolylineStitcher.hpp:165-173
                        if closest.point_idx == 0 {
                            // auto start_pos = (*closest.polygons)[closest.poly_idx].begin();
                            // if (segment_dist < snap_distance) { ++start_pos; }
                            // chain.insert(chain.end(), start_pos, (*closest.polygons)[closest.poly_idx].end());
                            let mut start_pos = 0;
                            if segment_dist < snap_distance {
                                start_pos += 1;
                            }
                            for i in start_pos..nearby_line.points.len() {
                                chain.points.push(nearby_line.points[i]);
                            }
                        } else {
                            // PolylineStitcher.hpp:175-182
                            // auto start_pos = (*closest.polygons)[closest.poly_idx].rbegin();
                            // if (segment_dist < snap_distance) { ++start_pos; }
                            // chain.insert(chain.end(), start_pos, (*closest.polygons)[closest.poly_idx].rend());
                            let mut start_idx = nearby_line.points.len();
                            if segment_dist < snap_distance {
                                start_idx -= 1;
                            }
                            for i in (0..start_idx).rev() {
                                chain.points.push(nearby_line.points[i]);
                            }
                        }
                        // PolylineStitcher.hpp:183-186
                        // for(size_t i = old_size; i < chain.size(); ++i) //Update chain length.
                        // {
                        //     chain_length += (make_point(chain[i]).template cast<int64_t>() - make_point(chain[i - 1]).template cast<int64_t>()).norm();
                        // }
                        for i in old_size..chain.points.len() {
                            let p1 = chain.points[i];
                            let p0 = chain.points[i - 1];
                            let ldx = p1.x() - p0.x();
                            let ldy = p1.y() - p0.y();
                            chain_length += (((ldx * ldx + ldy * ldy) as f64).sqrt()) as i64;
                        }
                        // PolylineStitcher.hpp:187
                        // should_close = should_close & !isOdd((*closest.polygons)[closest.poly_idx]); //If we connect an even to an odd line, we should no longer try to close it.
                        should_close = should_close & !Self::is_odd_polygon(nearby_line);
                        // PolylineStitcher.hpp:188
                        // assert( ! processed[closest.poly_idx]);
                        debug_assert!(!processed[closest.poly_idx]);
                        // PolylineStitcher.hpp:189
                        // processed[closest.poly_idx] = true;
                        processed[closest.poly_idx] = true;
                    }
                }
                // PolylineStitcher.hpp:191-200
                // if (closest_is_closing_polygon)
                // {
                //     if (go_in_reverse_direction)
                //     { // re-reverse chain to retain original direction
                //         chain.reverse();
                //     }
                //     break; // don't consider reverse direction
                // }
                if closest_is_closing_polygon {
                    if go_in_reverse_direction {
                        // NOTE: not sure if this code could ever be reached, since if a polygon can be closed that should be already possible in the forward direction
                        chain.reverse();
                    }
                    break; // don't consider reverse direction
                }
            }
            // PolylineStitcher.hpp:202-215
            // if (closest_is_closing_polygon)
            // {
            //     result_polygons.emplace_back(chain);
            // }
            // else
            // {
            //     PathsPointIndex<Paths> ppi_here(&lines, line_idx, 0);
            //     if ( ! canReverse(ppi_here))
            //     { // ... the polyline isn't allowed to be reversed, so we re-reverse it.
            //         chain.reverse();
            //     }
            //     result_lines.emplace_back(chain);
            // }
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

    // ====================================================================
    // PolylineStitcher<Polygons, Polygon, Point> specializations
    // ====================================================================

    /// Whether a polyline is allowed to be reversed.
    ///
    /// PolylineStitcher.cpp:17-20
    /// template<> bool PolylineStitcher<Polygons, Polygon, Point>::canReverse(const PathsPointIndex<Polygons> &)
    /// {
    ///     return true;
    /// }
    pub fn can_reverse_polygon(_polyline: &PathsPointIndex) -> bool {
        true
    }

    /// Whether two paths are allowed to be connected.
    ///
    /// PolylineStitcher.cpp:27-30
    /// template<> bool PolylineStitcher<Polygons, Polygon, Point>::canConnect(const Polygon &, const Polygon &)
    /// {
    ///     return true;
    /// }
    pub fn can_connect_polygon(_a: &Polygon, _b: &Polygon) -> bool {
        true
    }

    /// Whether a path is odd.
    ///
    /// PolylineStitcher.cpp:37-40
    /// template<> bool PolylineStitcher<Polygons, Polygon, Point>::isOdd(const Polygon &)
    /// {
    ///     return false;
    /// }
    pub fn is_odd_polygon(_line: &Polygon) -> bool {
        false
    }

    // ====================================================================
    // PolylineStitcher<VariableWidthLines, ExtrusionLine, ExtrusionJunction>
    // specializations.
    //
    // `stitch_extrusion` below is the template body (PolylineStitcher.hpp:53-217)
    // instantiated for VariableWidthLines. Instead of making `PathsPointIndex`
    // generic over the container, the grid element (`ExtrusionPointIndex`)
    // carries a copy of the endpoint position — equivalent, since `lines` is
    // immutable for the duration of the stitch and `p()` in C++ only ever
    // resolves that same endpoint.
    // ====================================================================

    /// Stitch together the separate extrusion `lines` into `result_lines` and,
    /// when they can be closed, into `result_polygons`.
    ///
    /// PolylineStitcher.hpp:53-217, instantiated as
    /// `PolylineStitcher<VariableWidthLines, ExtrusionLine, ExtrusionJunction>::stitch`
    /// (the WallToolPaths.cpp:561 call).
    pub fn stitch_extrusion(
        lines: &VariableWidthLines,
        result_lines: &mut VariableWidthLines,
        result_polygons: &mut VariableWidthLines,
        max_stitch_distance: i64,
        snap_distance: i64,
    ) {
        // PolylineStitcher.hpp:55-56
        if lines.is_empty() {
            return;
        }

        // PolylineStitcher.hpp:58
        let mut grid = SparsePointGrid::<ExtrusionPointIndex, ExtrusionPointIndexLocator>::new(
            max_stitch_distance,
            lines.len() * 2,
            1.0,
        );

        // populate grid — PolylineStitcher.hpp:61-66
        for (line_idx, line) in lines.iter().enumerate() {
            if line.junctions.is_empty() {
                continue;
            }
            grid.insert(ExtrusionPointIndex {
                p: line.junctions[0].p,
                poly_idx: line_idx,
                point_idx: 0,
            });
            grid.insert(ExtrusionPointIndex {
                p: line.junctions[line.junctions.len() - 1].p,
                poly_idx: line_idx,
                point_idx: line.junctions.len() - 1,
            });
        }

        // PolylineStitcher.hpp:68
        let mut processed = vec![false; lines.len()];

        // PolylineStitcher.hpp:70
        for line_idx in 0..lines.len() {
            // PolylineStitcher.hpp:72-75
            if processed[line_idx] {
                continue;
            }
            // PolylineStitcher.hpp:76
            processed[line_idx] = true;
            let line = &lines[line_idx];
            // PolylineStitcher.hpp:78  bool should_close = isOdd(line);
            let mut should_close = Self::is_odd_extrusion(line);

            // PolylineStitcher.hpp:80
            let mut chain: ExtrusionLine = line.clone();
            // PolylineStitcher.hpp:81
            let mut closest_is_closing_polygon = false;
            // PolylineStitcher.hpp:82-83 — first the unreversed direction
            for go_in_reverse_direction in [false, true] {
                // PolylineStitcher.hpp:84-87
                if go_in_reverse_direction {
                    chain.reverse();
                }
                // PolylineStitcher.hpp:88
                let mut chain_length: i64 = chain.polyline_length();

                // PolylineStitcher.hpp:90
                loop {
                    // PolylineStitcher.hpp:92  Point from = make_point(chain.back());
                    let from = chain.junctions.last().unwrap().p;

                    // PolylineStitcher.hpp:94-95
                    let mut closest: Option<ExtrusionPointIndex> = None;
                    let mut closest_distance = i64::MAX;

                    // PolylineStitcher.hpp:96-153
                    grid.process_nearby(from, max_stitch_distance, |nearby| {
                        // PolylineStitcher.hpp:101
                        let mut is_closing_segment = false;
                        // PolylineStitcher.hpp:102
                        let dx = nearby.p.x() - from.x();
                        let dy = nearby.p.y() - from.y();
                        let mut dist = (((dx * dx + dy * dy) as f64).sqrt()) as i64;
                        // PolylineStitcher.hpp:103-106
                        if dist > max_stitch_distance {
                            return true; // keep looking
                        }
                        // PolylineStitcher.hpp:107
                        let front_p = chain.junctions[0].p;
                        let dx2 = nearby.p.x() - front_p.x();
                        let dy2 = nearby.p.y() - front_p.y();
                        if dx2 * dx2 + dy2 * dy2 < snap_distance * snap_distance {
                            // PolylineStitcher.hpp:109-113
                            if chain_length + dist < 3 * max_stitch_distance
                                || chain.junctions.len() <= 2
                            {
                                return true; // look for a better next line
                            }
                            // PolylineStitcher.hpp:114
                            is_closing_segment = true;
                            // PolylineStitcher.hpp:115-125
                            if !should_close {
                                dist += scaled(0.01); // prefer continuing polyline over closing a polygon
                            } else {
                                dist -= scaled(0.01); // prefer closing the polygon if it's 100% even lines
                            }
                        } else if processed[nearby.poly_idx] {
                            // PolylineStitcher.hpp:127-130 — already moved to output
                            return true; // keep looking for a connection
                        }
                        // PolylineStitcher.hpp:131-132
                        let mut nearby_would_be_reversed = nearby.point_idx != 0;
                        nearby_would_be_reversed = nearby_would_be_reversed != go_in_reverse_direction;
                        // PolylineStitcher.hpp:133-136
                        // canReverse(ppi) for this instantiation reads
                        // (*ppi.polygons)[ppi.poly_idx].is_odd (PolylineStitcher.cpp:9-15).
                        if !Self::can_reverse_extrusion(&lines[nearby.poly_idx])
                            && nearby_would_be_reversed
                        {
                            return true; // keep looking for a connection
                        }
                        // PolylineStitcher.hpp:137-140
                        if !Self::can_connect_extrusion(&chain, &lines[nearby.poly_idx]) {
                            return true; // keep looking for a connection
                        }
                        // PolylineStitcher.hpp:141-146
                        if dist < closest_distance {
                            closest_distance = dist;
                            closest = Some(*nearby);
                            closest_is_closing_polygon = is_closing_segment;
                        }
                        // PolylineStitcher.hpp:147-150
                        if dist < snap_distance {
                            return false; // stop looking for alternatives
                        }
                        true // keep processing elements
                    });

                    // PolylineStitcher.hpp:155-160
                    if closest.is_none() || closest_is_closing_polygon {
                        break;
                    }
                    let closest = closest.unwrap();

                    // PolylineStitcher.hpp:162-163
                    let back_p = chain.junctions.last().unwrap().p;
                    let sdx = back_p.x() - closest.p.x();
                    let sdy = back_p.y() - closest.p.y();
                    let segment_dist = (((sdx * sdx + sdy * sdy) as f64).sqrt()) as i64;
                    debug_assert!(segment_dist <= max_stitch_distance + scaled(0.01));
                    // PolylineStitcher.hpp:164
                    let old_size = chain.junctions.len();
                    let nearby_line = &lines[closest.poly_idx];
                    // PolylineStitcher.hpp:165-182 — append the nearby line's
                    // junctions, forward when we connect at its head, reversed
                    // when we connect at its tail; skip the coincident endpoint
                    // when the joint is within snap_distance.
                    if closest.point_idx == 0 {
                        let start_pos = if segment_dist < snap_distance { 1 } else { 0 };
                        for j in nearby_line.junctions[start_pos..].iter() {
                            chain.junctions.push(*j);
                        }
                    } else {
                        let mut start_idx = nearby_line.junctions.len();
                        if segment_dist < snap_distance {
                            start_idx -= 1;
                        }
                        for i in (0..start_idx).rev() {
                            chain.junctions.push(nearby_line.junctions[i]);
                        }
                    }
                    // PolylineStitcher.hpp:183-186 — update chain length
                    for i in old_size..chain.junctions.len() {
                        let p1 = chain.junctions[i].p;
                        let p0 = chain.junctions[i - 1].p;
                        let ldx = p1.x() - p0.x();
                        let ldy = p1.y() - p0.y();
                        chain_length += (((ldx * ldx + ldy * ldy) as f64).sqrt()) as i64;
                    }
                    // PolylineStitcher.hpp:187 — connecting an even to an odd
                    // line means we should no longer try to close it.
                    should_close = should_close & !Self::is_odd_extrusion(nearby_line);
                    // PolylineStitcher.hpp:188-189
                    debug_assert!(!processed[closest.poly_idx]);
                    processed[closest.poly_idx] = true;
                }
                // PolylineStitcher.hpp:191-200
                if closest_is_closing_polygon {
                    if go_in_reverse_direction {
                        // re-reverse chain to retain original direction
                        chain.reverse();
                    }
                    break; // don't consider reverse direction
                }
            }
            // PolylineStitcher.hpp:202-215
            if closest_is_closing_polygon {
                result_polygons.push(chain);
            } else {
                // canReverse(ppi_here) — PolylineStitcher.cpp:9-15 resolves to
                // lines[line_idx].is_odd.
                if !Self::can_reverse_extrusion(&lines[line_idx]) {
                    // Since closest_is_closing_polygon is false we went through
                    // the reverse iteration; a non-reversible polyline must be
                    // re-reversed to its original direction.
                    chain.reverse();
                }
                result_lines.push(chain);
            }
        }
    }

    /// Whether an extrusion-line polyline is allowed to be reversed.
    /// (Not true for wall polylines which are not odd.)
    ///
    /// PolylineStitcher.cpp:9-15
    /// template<> bool PolylineStitcher<VariableWidthLines, ExtrusionLine, ExtrusionJunction>::canReverse(const PathsPointIndex<VariableWidthLines> &ppi)
    /// {
    ///     if ((*ppi.polygons)[ppi.poly_idx].is_odd)
    ///         return true;
    ///     else
    ///         return false;
    /// }
    pub fn can_reverse_extrusion(line: &ExtrusionLine) -> bool {
        if line.is_odd {
            true
        } else {
            false
        }
    }

    /// Whether two extrusion lines are allowed to be connected.
    /// (Not true for an odd and an even wall.)
    ///
    /// PolylineStitcher.cpp:22-25
    /// template<> bool PolylineStitcher<VariableWidthLines, ExtrusionLine, ExtrusionJunction>::canConnect(const ExtrusionLine &a, const ExtrusionLine &b)
    /// {
    ///     return a.is_odd == b.is_odd;
    /// }
    pub fn can_connect_extrusion(a: &ExtrusionLine, b: &ExtrusionLine) -> bool {
        a.is_odd == b.is_odd
    }

    /// Whether an extrusion line is odd.
    ///
    /// PolylineStitcher.cpp:32-35
    /// template<> bool PolylineStitcher<VariableWidthLines, ExtrusionLine, ExtrusionJunction>::isOdd(const ExtrusionLine &line)
    /// {
    ///     return line.is_odd;
    /// }
    pub fn is_odd_extrusion(line: &ExtrusionLine) -> bool {
        line.is_odd
    }
}

/// Grid element for `stitch_extrusion`: one endpoint of extrusion line
/// `poly_idx` (junction index 0 or len-1), with its position copied out.
///
/// Stands in for C++ `PathsPointIndex<VariableWidthLines>` (PolylineStitcher.hpp:58):
/// C++ resolves `p()` through the container; since `lines` is immutable during
/// the stitch, carrying the endpoint copy is equivalent and avoids making
/// `PathsPointIndex` generic over the container type.
#[derive(Debug, Clone, Copy)]
pub struct ExtrusionPointIndex {
    pub p: Point,
    pub poly_idx: usize,
    pub point_idx: usize,
}

/// Locator for [`ExtrusionPointIndex`] — mirrors `PathsPointIndexLocator`.
#[derive(Debug, Clone, Copy, Default)]
pub struct ExtrusionPointIndexLocator;

impl LocatorTrait<ExtrusionPointIndex> for ExtrusionPointIndexLocator {
    fn locate(&self, elem: &ExtrusionPointIndex) -> Point {
        elem.p
    }
}

/// Open polyline length of a `Polygon`, i.e. the sum of the lengths of the
/// consecutive segments without the closing edge.
///
/// This mirrors C++ `MultiPoint::length()` (inherited by `Polygon`), which is
/// what `Polygon::polylineLength()` resolves to for the
/// `PolylineStitcher<Polygons, Polygon, Point>` instantiation. The result is in
/// scaled integer units (matching the `int64_t chain_length` used in `stitch`).
/// MultiPoint.hpp:43 (double length() const)
fn polyline_length(poly: &Polygon) -> i64 {
    if poly.points.len() < 2 {
        return 0;
    }
    let mut total = 0i64;
    for i in 1..poly.points.len() {
        let p1 = poly.points[i];
        let p0 = poly.points[i - 1];
        let dx = p1.x() - p0.x();
        let dy = p1.y() - p0.y();
        total += (((dx * dx + dy * dy) as f64).sqrt()) as i64;
    }
    total
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arachne::utils::extrusion_line::ExtrusionLine;
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
        // PolylineStitcher.hpp:55-56
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
        // Single line, no connections.
        let mut lines = Polygons::new();
        lines.push(create_simple_line(vec![(0, 0), (100, 0), (100, 100)]));

        let mut result_lines = Polygons::new();
        let mut result_polygons = Polygons::new();

        PolylineStitcher::stitch_polygons(
            &lines,
            &mut result_lines,
            &mut result_polygons,
            100_000,
            10_000,
        );

        assert_eq!(result_lines.len(), 1);
        assert_eq!(result_polygons.len(), 0);
    }

    #[test]
    fn test_polyline_stitcher_close_polygon() {
        // Lines that form a closed polygon. Coordinates are scaled large enough
        // that the perimeter exceeds 3 * max_stitch_distance so closing is allowed.
        let mut lines = Polygons::new();
        lines.push(create_simple_line(vec![(0, 0), (1_000_000, 0)]));
        lines.push(create_simple_line(vec![(1_000_000, 0), (1_000_000, 1_000_000)]));
        lines.push(create_simple_line(vec![(1_000_000, 1_000_000), (0, 1_000_000)]));
        lines.push(create_simple_line(vec![(0, 1_000_000), (0, 0)]));

        let mut result_lines = Polygons::new();
        let mut result_polygons = Polygons::new();

        PolylineStitcher::stitch_polygons(
            &lines,
            &mut result_lines,
            &mut result_polygons,
            100_000,
            10_000,
        );

        assert_eq!(result_polygons.len(), 1);
        assert_eq!(result_lines.len(), 0);
    }

    #[test]
    fn test_polyline_stitcher_can_reverse() {
        // PolylineStitcher.cpp:19
        let lines = Polygons::new();
        let ppi = PathsPointIndex::with_indices(&lines, 0, 0);
        assert!(PolylineStitcher::can_reverse_polygon(&ppi));
    }

    #[test]
    fn test_polyline_stitcher_can_connect() {
        // PolylineStitcher.cpp:29
        let poly1 = Polygon::new();
        let poly2 = Polygon::new();
        assert!(PolylineStitcher::can_connect_polygon(&poly1, &poly2));
    }

    #[test]
    fn test_polyline_stitcher_is_odd() {
        // PolylineStitcher.cpp:39
        let poly = Polygon::new();
        assert!(!PolylineStitcher::is_odd_polygon(&poly));
    }

    #[test]
    fn test_polyline_stitcher_two_lines() {
        // Two lines that connect end-to-end.
        let mut lines = Polygons::new();
        lines.push(create_simple_line(vec![(0, 0), (50, 0)]));
        lines.push(create_simple_line(vec![(50, 0), (100, 0)]));

        let mut result_lines = Polygons::new();
        let mut result_polygons = Polygons::new();

        PolylineStitcher::stitch_polygons(
            &lines,
            &mut result_lines,
            &mut result_polygons,
            100_000,
            10_000,
        );

        assert_eq!(result_lines.len(), 1);
        assert_eq!(result_polygons.len(), 0);
        assert!(result_lines[0].points.len() >= 3);
    }

    #[test]
    fn test_can_connect_extrusion() {
        // PolylineStitcher.cpp:24 — a.is_odd == b.is_odd
        let even_a = ExtrusionLine::new(0, false);
        let even_b = ExtrusionLine::new(0, false);
        let odd = ExtrusionLine::new(0, true);
        assert!(PolylineStitcher::can_connect_extrusion(&even_a, &even_b));
        assert!(!PolylineStitcher::can_connect_extrusion(&even_a, &odd));
        assert!(PolylineStitcher::can_connect_extrusion(&odd, &odd));
    }

    #[test]
    fn test_can_reverse_extrusion() {
        // PolylineStitcher.cpp:11 — return line.is_odd ? true : false
        let even = ExtrusionLine::new(0, false);
        let odd = ExtrusionLine::new(0, true);
        assert!(!PolylineStitcher::can_reverse_extrusion(&even));
        assert!(PolylineStitcher::can_reverse_extrusion(&odd));
    }

    #[test]
    fn test_is_odd_extrusion() {
        // PolylineStitcher.cpp:34 — line.is_odd
        let even = ExtrusionLine::new(0, false);
        let odd = ExtrusionLine::new(0, true);
        assert!(!PolylineStitcher::is_odd_extrusion(&even));
        assert!(PolylineStitcher::is_odd_extrusion(&odd));
    }
}
