//! Variable-width extrusion line representation for Arachne
//!
//! C++ Reference:
//! - Arachne/utils/ExtrusionLine.hpp
//! - Arachne/utils/ExtrusionLine.cpp
//!
//! **STATUS:** ✅ Complete implementation
//!
//! Represents a polyline that is to be extruded with variable line width.
//! Each polyline is a sequence of ExtrusionJunction points with per-vertex widths.

use crate::arachne::utils::extrusion_junction::ExtrusionJunction;
use crate::geometry::{cross2f, Coord, CoordF, Line, Point, Polygon, ThickPolyline};
use crate::scaled;

/// `scaled<double>(v)` — scale a millimetre value to the crate's coordinate space
/// as a floating-point value (the double-typed counterpart of [`crate::scaled`]).
#[inline]
fn scaled_f(mm: f64) -> f64 {
    mm * crate::SCALING_FACTOR
}

/// Represents a polyline (not just a line) that is to be extruded with variable line width.
/// ExtrusionLine.hpp:25-163
#[derive(Debug, Clone)]
pub struct ExtrusionLine {
    /// Which inset this path represents, counted from the outside inwards.
    /// The outer wall has index 0.
    /// ExtrusionLine.hpp:32
    pub inset_idx: usize,

    /// If a thin piece needs to be printed with an odd number of walls (e.g. 5 walls)
    /// then there will be one wall in the middle that is not a loop. This field indicates
    /// whether this path is such a line through the middle, that has no companion line
    /// going back on the other side and is not a closed loop.
    /// ExtrusionLine.hpp:40
    pub is_odd: bool,

    /// Whether this is a closed polygonal path
    /// ExtrusionLine.hpp:45
    pub is_closed: bool,

    /// The list of vertices along which this path runs.
    /// Each junction has a width, making this path a variable-width path.
    /// ExtrusionLine.hpp:61
    pub junctions: Vec<ExtrusionJunction>,
}

impl ExtrusionLine {
    /// Constructor with inset_idx and is_odd flag
    /// ExtrusionLine.cpp:13
    pub fn new(inset_idx: usize, is_odd: bool) -> Self {
        Self {
            inset_idx,
            is_odd,
            is_closed: false,
            junctions: Vec::new(),
        }
    }

    /// Constructor with inset_idx, is_odd, and is_closed flags
    /// ExtrusionLine.cpp:15
    pub fn with_closed(inset_idx: usize, is_odd: bool, is_closed: bool) -> Self {
        Self {
            inset_idx,
            is_odd,
            is_closed,
            junctions: Vec::new(),
        }
    }

    /// Default constructor
    /// ExtrusionLine.hpp:56
    pub fn default() -> Self {
        Self {
            inset_idx: usize::MAX, // C++ uses -1, we use MAX for unsigned
            is_odd: true,
            is_closed: false,
            junctions: Vec::new(),
        }
    }

    /// Gets the number of vertices in this polygon
    /// ExtrusionLine.hpp:50
    #[inline]
    pub fn size(&self) -> usize {
        self.junctions.len()
    }

    /// Whether there are no junctions
    /// ExtrusionLine.hpp:55
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.junctions.is_empty()
    }

    /// Get first junction
    /// ExtrusionLine.hpp:82
    #[inline]
    pub fn front(&self) -> &ExtrusionJunction {
        &self.junctions[0]
    }

    /// Get last junction
    /// ExtrusionLine.hpp:83
    #[inline]
    pub fn back(&self) -> &ExtrusionJunction {
        &self.junctions[self.junctions.len() - 1]
    }

    /// Add a junction to the end
    /// ExtrusionLine.hpp:90
    #[inline]
    pub fn push_back(&mut self, junction: ExtrusionJunction) {
        self.junctions.push(junction);
    }

    /// Remove junction at index
    /// ExtrusionLine.hpp:91
    #[inline]
    pub fn remove(&mut self, index: usize) {
        self.junctions.remove(index);
    }

    /// Insert junction at index
    /// ExtrusionLine.hpp:92
    #[inline]
    pub fn insert(&mut self, index: usize, junction: ExtrusionJunction) {
        self.junctions.insert(index, junction);
    }

    /// Clear all junctions
    /// ExtrusionLine.hpp:98
    #[inline]
    pub fn clear(&mut self) {
        self.junctions.clear();
    }

    /// Reverse the order of junctions
    /// ExtrusionLine.hpp:99
    #[inline]
    pub fn reverse(&mut self) {
        self.junctions.reverse();
    }

    /// Sum the total length of this path
    /// ExtrusionLine.cpp:17-32
    pub fn get_length(&self) -> Coord {
        if self.junctions.is_empty() {
            return 0;
        }

        let mut len: Coord = 0;
        let mut prev = self.junctions[0];

        for next in &self.junctions {
            len += (next.p - prev.p).length() as Coord;
            prev = *next;
        }

        if self.is_closed {
            len += (self.front().p - self.back().p).length() as Coord;
        }

        len
    }

    /// Alias for get_length
    /// ExtrusionLine.hpp:108
    #[inline]
    pub fn polyline_length(&self) -> Coord {
        self.get_length()
    }

    /// Put all junction locations into a polygon object
    /// ExtrusionLine.hpp:114-122
    pub fn to_polygon(&self) -> Polygon {
        let mut poly = Polygon::new();
        poly.points.reserve(self.junctions.len());
        for junction in &self.junctions {
            poly.points.push(junction.p);
        }
        poly
    }

    /// Get the minimal width of this path
    /// ExtrusionLine.cpp:34-41
    pub fn get_minimal_width(&self) -> Coord {
        self.junctions.iter().map(|j| j.w).min().unwrap_or(0)
    }

    /// Removes vertices of the ExtrusionLines to make sure that they are not too high resolution.
    ///
    /// This removes junctions which are connected to line segments that are shorter
    /// than the `smallest_line_segment`, unless that would introduce a deviation
    /// in the contour of more than `allowed_error_distance`.
    ///
    /// Criteria:
    /// 1. Never remove a junction if either of the connected segments is larger than smallest_line_segment
    /// 2. Never remove a junction if the distance between that junction and the final resulting polygon
    ///    would be higher than allowed_error_distance
    /// 3. The direction of segments longer than smallest_line_segment always remains unaltered
    ///    (but their end points may change if it is connected to a small segment)
    /// 4. Never remove a junction if it has a distinctively different width than the next junction,
    ///    as this can introduce unwanted irregularities on the wall widths.
    ///
    /// # Arguments
    /// * `smallest_line_segment_squared` - Maximal length of removed line segments (squared)
    /// * `allowed_error_distance_squared` - Maximum deviation from original path (squared)
    /// * `maximum_extrusion_area_deviation` - Maximum extrusion area deviation allowed
    ///
    /// ExtrusionLine.cpp:43-192
    pub fn simplify(
        &mut self,
        smallest_line_segment_squared: Coord,
        allowed_error_distance_squared: Coord,
        maximum_extrusion_area_deviation: Coord,
    ) {
        let min_path_size = if self.is_closed { 3 } else { 2 };
        if self.junctions.len() <= min_path_size {
            return;
        }

        let mut new_junctions = Vec::new();
        // Starting junction should always exist in the simplified path
        // ExtrusionLine.cpp:59
        new_junctions.push(self.junctions[0]);

        // Initially, previous_previous is always the same as previous
        // ExtrusionLine.cpp:63-64
        let mut previous_previous = self.junctions[0];
        let mut previous = self.junctions[0];

        // Shoelace formula for area accumulation
        // ExtrusionLine.cpp:82 — int64_t(previous.p.x()) * int64_t(initial.p.y()) - int64_t(previous.p.y()) * int64_t(initial.p.x())
        // FIDELITY-NOTE(F2): C++ widens int32 coord_t operands to int64 for the product;
        // Coord is i64 here, so the multiply is i64*i64. Values match for in-range coords.
        let initial = self.junctions[1];
        let mut accumulated_area_removed: Coord = (previous.p.x as Coord) * (initial.p.y as Coord)
            - (previous.p.y as Coord) * (initial.p.x as Coord);

        // Iterate through intermediate junctions
        // ExtrusionLine.cpp:84
        for point_idx in 1..(self.junctions.len() - 1) {
            let current = self.junctions[point_idx];

            // Spill over in case of overflow, unless the [next] vertex will then be equal to [previous]
            // ExtrusionLine.cpp:89-90
            let spill_over = point_idx + 1 == self.junctions.len() && new_junctions.len() > 1;
            let next = if spill_over {
                new_junctions[0]
            } else {
                self.junctions[point_idx + 1]
            };

            // Shoelace area calculation for removed segments
            // ExtrusionLine.cpp:92-94
            let removed_area_next: Coord = (current.p.x as Coord) * (next.p.y as Coord)
                - (current.p.y as Coord) * (next.p.x as Coord);
            let negative_area_closing: Coord = (next.p.x as Coord) * (previous.p.y as Coord)
                - (next.p.y as Coord) * (previous.p.x as Coord);
            accumulated_area_removed += removed_area_next;

            // const int64_t length2 = (current - previous).cast<int64_t>().squaredNorm();
            // ExtrusionLine.cpp:96
            let length2 = (current.p - previous.p).length_squared() as Coord;
            // ExtrusionLine.cpp:97
            if length2 < scaled(0.025) {
                // We're allowed to always delete segments of less than 5 micron. The width in this case doesn't matter that much.
                // ExtrusionLine.cpp:99-100
                continue;
            }

            // Close the shortcut area polygon
            // ExtrusionLine.cpp:103-104
            let area_removed_so_far = accumulated_area_removed + negative_area_closing;
            let base_length_2 = (next.p - previous.p).length_squared() as Coord;

            // Two line segments form a line back and forth with no area
            // ExtrusionLine.cpp:106-109
            if base_length_2 == 0 {
                continue; // Remove the junction (vertex)
            }

            // Calculate height of triangle formed by previous, current, next
            // Uses formula: h^2 = L^2 / b^2
            // ExtrusionLine.cpp:117
            let height_2 = (area_removed_so_far as f64 * area_removed_so_far as f64
                / base_length_2 as f64) as Coord;

            let mut weighted_average_width: Coord = 0;
            let extrusion_area_error = Self::calculate_extrusion_area_deviation_error(
                previous,
                current,
                next,
                &mut weighted_average_width,
            );

            // ExtrusionLine.cpp:120-123
            if (height_2 <= scaled(0.001) //Almost exactly colinear (barring rounding errors).
                && Line::distance_to_infinite(current.p, previous.p, next.p) <= scaled_f(0.001)) // Make sure that height_2 is not small because of cancellation of positive and negative areas
                // We shouldn't remove middle junctions of colinear segments if the area changed for the C-P segment is exceeding the maximum allowed
                && extrusion_area_error <= maximum_extrusion_area_deviation
            {
                // Remove the current junction (vertex).
                // ExtrusionLine.cpp:125-126
                continue;
            }

            // Check if segment is short and removal doesn't introduce too much error
            // ExtrusionLine.cpp:129-131
            if length2 < smallest_line_segment_squared && height_2 <= allowed_error_distance_squared
            {
                let next_length2 = (current.p - next.p).length_squared() as Coord;

                // Special case: next line is long
                // ExtrusionLine.cpp:132-168
                if next_length2 > 4 * smallest_line_segment_squared {
                    // Find intersection point to preserve direction
                    let intersection_point = Line::new(previous_previous.p, previous.p)
                        .intersection_infinite(&Line::new(current.p, next.p));

                    // Validate intersection point
                    // ExtrusionLine.cpp:141-148
                    if let Some(intersection_point) = intersection_point {
                        if Line::distance_to_infinite_squared(
                            intersection_point,
                            previous.p,
                            current.p,
                        ) <= allowed_error_distance_squared as f64
                            && (intersection_point - previous.p).length_squared() as Coord
                                <= smallest_line_segment_squared
                            && (intersection_point - next.p).length_squared() as Coord
                                <= smallest_line_segment_squared
                        {
                            // New point seems like a valid one.
                            // ExtrusionLine.cpp:151-152
                            let new_to_add = ExtrusionJunction::with_hole_compensation(
                                intersection_point,
                                current.w,
                                current.perimeter_index,
                                current.hole_compensation_flag,
                            );

                            // If there was a previous point added, remove it
                            // ExtrusionLine.cpp:154-157
                            if !new_junctions.is_empty() {
                                new_junctions.pop();
                                previous = previous_previous;
                            }

                            // The junction is replaced by the new one
                            // ExtrusionLine.cpp:160-163
                            accumulated_area_removed = removed_area_next;
                            previous_previous = previous;
                            previous = new_to_add;
                            new_junctions.push(new_to_add);
                            continue;
                        }
                    }
                    // Can't find a better spot, leave it in
                    // ExtrusionLine.cpp:150-151
                } else {
                    // ExtrusionLine.cpp:167
                    continue; // Remove the junction (vertex)
                }
            }

            // The junction isn't removed
            // ExtrusionLine.cpp:171-174
            accumulated_area_removed = removed_area_next;
            previous_previous = previous;
            previous = current;
            new_junctions.push(current);
        }

        // Ending junction should always exist in the simplified path
        // ExtrusionLine.cpp:177
        new_junctions.push(*self.back());

        // Enforce invariant for closed polygons: first and last points are the same
        // ExtrusionLine.cpp:182-186
        if (self.junctions.first().unwrap().p - self.junctions.last().unwrap().p).length_squared()
            == 0
        {
            new_junctions.last_mut().unwrap().p = self.junctions.first().unwrap().p;
        }

        self.junctions = new_junctions;
    }

    /// Computes the total area error (in μm²) of the AB and BC segments of an ABC straight
    /// ExtrusionLine when junction B is removed.
    ///
    /// # Arguments
    /// * `a` - Start point of the 3-point-straight line
    /// * `b` - Intermediate point of the 3-point-straight line
    /// * `c` - End point of the 3-point-straight line
    /// * `weighted_average_width` - Output: weighted average of the widths
    ///
    /// ExtrusionLine.cpp:194-235
    pub fn calculate_extrusion_area_deviation_error(
        a: ExtrusionJunction,
        b: ExtrusionJunction,
        c: ExtrusionJunction,
        weighted_average_width: &mut Coord,
    ) -> Coord {
        // Calculate segment lengths
        // ExtrusionLine.cpp:213-214
        let ab_length = (b.p - a.p).length() as Coord;
        let bc_length = (c.p - b.p).length() as Coord;
        let width_diff = std::cmp::max((b.w - a.w).abs(), (c.w - b.w).abs());

        // Adjust width only if there's a significant difference
        // ExtrusionLine.cpp:215-227
        if width_diff > 1 {
            // ExtrusionLine.cpp:220-221 — ab_weight/bc_weight are int64_t.
            let ab_weight = (a.w + b.w) / 2;
            let bc_weight = (b.w + c.w) / 2;
            // ExtrusionLine.cpp:223 — weighted_average_width is a coord_t& (int32_t); the
            // int64 division result is truncated to int32 on assignment.
            // FIDELITY-NOTE(F2): Coord is i64 here, so reproduce the C++ int32 truncation
            // locally with `as i32 as Coord` to match the assigned value bit-for-bit.
            *weighted_average_width =
                ((ab_length * ab_weight + bc_length * bc_weight) / (c.p - a.p).length() as Coord)
                    as i32 as Coord;
            // ExtrusionLine.cpp:225 — abs of (int64 weight - coord_t avg, promoted to int64).
            (ab_weight - *weighted_average_width).abs() * ab_length
                + (bc_weight - *weighted_average_width).abs() * bc_length
        } else {
            // If width difference is very small, select the width of the longer segment
            // ExtrusionLine.cpp:230-234
            *weighted_average_width = if ab_length > bc_length { a.w } else { b.w };
            if ab_length > bc_length {
                width_diff * bc_length
            } else {
                width_diff * ab_length
            }
        }
    }

    /// Check if this extrusion line should apply hole compensation based on the marked ratio
    /// of segments with hole_compensation_flag set.
    ///
    /// # Arguments
    /// * `threshold` - Minimum ratio of marked length to total length (default 0.8)
    ///
    /// ExtrusionLine.cpp:237-249
    pub fn should_apply_hole_compensation(&self, threshold: f64) -> bool {
        let mut total_length: Coord = 0;
        let mut marked_length: Coord = 0;

        // Iterate through segments
        // ExtrusionLine.cpp:241-246
        for idx in 1..self.junctions.len() {
            let length = (self.junctions[idx].p - self.junctions[idx - 1].p).length() as Coord;
            total_length += length;

            // Average the flags of the two endpoints
            let marked_rate = self.junctions[idx].hole_compensation_flag as i32
                + self.junctions[idx - 1].hole_compensation_flag as i32;
            marked_length += length * (marked_rate as Coord) / 2;
        }

        let rate = marked_length as f64 / total_length as f64;
        rate > threshold
    }

    /// Check if this is a contour (closed clockwise polygon) vs a hole (counterclockwise)
    /// Arachne produces contours with clockwise orientation and holes with counterclockwise.
    ///
    /// ExtrusionLine.cpp:251-263
    pub fn is_contour(&self) -> bool {
        if !self.is_closed {
            return false;
        }

        let mut poly = Polygon::new();
        poly.points.reserve(self.junctions.len());
        for junction in &self.junctions {
            poly.points.push(junction.p);
        }

        // Arachne produces contour with clockwise orientation
        poly.is_clockwise()
    }

    /// Calculate the signed area of this closed extrusion line
    ///
    /// ExtrusionLine.cpp:265-278
    pub fn area(&self) -> f64 {
        // ExtrusionLine.cpp:267
        debug_assert!(self.is_closed);
        // ExtrusionLine.cpp:268
        let mut a = 0.0;
        // ExtrusionLine.cpp:269
        if self.junctions.len() >= 3 {
            // ExtrusionLine.cpp:270 — Vec2d p1 = this->junctions.back().p.cast<double>();
            let mut p1 = self.junctions.last().unwrap().p.to_f64();
            // ExtrusionLine.cpp:271
            for junction in &self.junctions {
                // ExtrusionLine.cpp:272 — Vec2d p2 = junction.p.cast<double>();
                let p2 = junction.p.to_f64();
                // ExtrusionLine.cpp:273 — a += cross2(p1, p2);
                a += cross2f(p1, p2);
                // ExtrusionLine.cpp:274
                p1 = p2;
            }
        }
        // ExtrusionLine.cpp:277
        0.5 * a
    }
}

// ExtrusionLine.hpp:201-219 — static inline Slic3r::ThickPolyline to_thick_polyline(const Arachne::ExtrusionLine &line_junctions)
//
// NOTE: The crate's `ThickPolyline.widths` is `Vec<CoordF>` (the scaled width
// stored as a float), whereas C++ `ThickPolyline.width` is `std::vector<coord_t>`.
// The scaled junction width `w` (a `coord_t`/`i64`) is therefore widened to `f64`
// when pushed, mirroring how `variable_width.rs` consumes these widths.
pub fn to_thick_polyline(line_junctions: &ExtrusionLine) -> ThickPolyline {
    // ExtrusionLine.hpp:203
    debug_assert!(line_junctions.size() >= 2);
    // ExtrusionLine.hpp:204
    let mut out = ThickPolyline::new();
    // ExtrusionLine.hpp:205-206
    out.points.push(line_junctions.front().p);
    out.widths.push(line_junctions.front().w as CoordF);
    // ExtrusionLine.hpp:207-208
    out.points.push(line_junctions.junctions[1].p);
    out.widths.push(line_junctions.junctions[1].w as CoordF);

    // ExtrusionLine.hpp:210 — auto it_prev = line_junctions.begin() + 1;
    let mut it_prev = 1usize;
    // ExtrusionLine.hpp:211-216
    for it in 2..line_junctions.junctions.len() {
        out.points.push(line_junctions.junctions[it].p);
        out.widths.push(line_junctions.junctions[it_prev].w as CoordF);
        out.widths.push(line_junctions.junctions[it].w as CoordF);
        it_prev = it;
    }

    // ExtrusionLine.hpp:218
    out
}

// ExtrusionLine.hpp:221-239 — static inline Slic3r::ThickPolyline to_thick_polyline(const ClipperLib_Z::Path &path)
//
// `ClipperLib_Z::Path` maps to the crate's `ZPath` (`Vec<ZPoint>` where
// `ZPoint = (x, y, z)`); the `z` component carries the scaled width.
pub fn to_thick_polyline_z(path: &crate::clipper_z_utils::ZPath) -> ThickPolyline {
    // ExtrusionLine.hpp:223
    debug_assert!(path.len() >= 2);
    // ExtrusionLine.hpp:224
    let mut out = ThickPolyline::new();
    // ExtrusionLine.hpp:225-226
    out.points.push(Point::new(path[0].0, path[0].1));
    out.widths.push(path[0].2 as CoordF);
    // ExtrusionLine.hpp:227-228
    out.points.push(Point::new(path[1].0, path[1].1));
    out.widths.push(path[1].2 as CoordF);

    // ExtrusionLine.hpp:230 — auto it_prev = path.begin() + 1;
    let mut it_prev = 1usize;
    // ExtrusionLine.hpp:231-236
    for it in 2..path.len() {
        out.points.push(Point::new(path[it].0, path[it].1));
        out.widths.push(path[it_prev].2 as CoordF);
        out.widths.push(path[it].2 as CoordF);
        it_prev = it;
    }

    // ExtrusionLine.hpp:238
    out
}

// ExtrusionLine.hpp:241-250 — static inline Polygon to_polygon(const ExtrusionLine &line)
pub fn to_polygon(line: &ExtrusionLine) -> Polygon {
    // ExtrusionLine.hpp:243
    let mut out = Polygon::new();
    // ExtrusionLine.hpp:244-245
    debug_assert!(line.junctions.len() >= 3);
    debug_assert!(line.junctions.first().unwrap().p == line.junctions.last().unwrap().p);
    // ExtrusionLine.hpp:246
    out.points.reserve(line.junctions.len() - 1);
    // ExtrusionLine.hpp:247-248 — for (auto it = line.junctions.begin(); it != line.junctions.end() - 1; ++it)
    for it in 0..(line.junctions.len() - 1) {
        out.points.push(line.junctions[it].p);
    }
    // ExtrusionLine.hpp:249
    out
}

/// Collection of variable-width extrusion lines generated by Arachne
/// ExtrusionLine.hpp:279
pub type VariableWidthLines = Vec<ExtrusionLine>;

// ============================================================================
// namespace Slic3r { ... } — ExtrusionLine.cpp:282-311
//
// These free functions convert Arachne extrusion geometry into the classic
// `ExtrusionPath` representation via `thick_polyline_to_multi_path`. The fixed
// arguments mirror C++: `scaled<float>(0.05)` tolerance and `float(SCALED_EPSILON)`
// merge tolerance.
// ============================================================================

use crate::clipper_z_utils::ZPaths;
use crate::extrusion_entity::{ExtrusionPath, ExtrusionRole};
use crate::flow::Flow;
use crate::variable_width::{thick_polyline_to_multi_path, ExtrusionPaths};

// ExtrusionLine.cpp:284-291 — void extrusion_paths_append(std::list<ExtrusionPath> &dst, const ClipperLib_Z::Paths &extrusion_paths, const ExtrusionRole role, const Flow &flow, double overhang)
//
// C++ uses `std::list<ExtrusionPath>` here; the crate has no idiomatic linked
// list, so a `Vec<ExtrusionPath>` is used with the same append-to-end semantics.
pub fn extrusion_paths_append_list(
    dst: &mut Vec<ExtrusionPath>,
    extrusion_paths: &ZPaths,
    role: ExtrusionRole,
    flow: &Flow,
    overhang: f64,
) {
    // ExtrusionLine.cpp:286
    for extrusion_path in extrusion_paths {
        // ExtrusionLine.cpp:287
        let thick_polyline = to_thick_polyline_z(extrusion_path);
        // ExtrusionLine.cpp:288
        let path = thick_polyline_to_multi_path(
            &thick_polyline,
            role,
            flow,
            scaled_f(0.05) as f32,
            crate::libslic3r::SCALED_EPSILON as f32,
            overhang,
        )
        .paths;
        // ExtrusionLine.cpp:289 — dst.insert(dst.end(), std::make_move_iterator(path.begin()), std::make_move_iterator(path.end()));
        dst.extend(path);
    }
}

// ExtrusionLine.cpp:293-299 — void extrusion_paths_append(ExtrusionPaths &dst, const ClipperLib_Z::Paths &extrusion_paths, const ExtrusionRole role, const Flow &flow, double overhang)
pub fn extrusion_paths_append_zpaths(
    dst: &mut ExtrusionPaths,
    extrusion_paths: &ZPaths,
    role: ExtrusionRole,
    flow: &Flow,
    overhang: f64,
) {
    // ExtrusionLine.cpp:295
    for extrusion_path in extrusion_paths {
        // ExtrusionLine.cpp:296
        let thick_polyline = to_thick_polyline_z(extrusion_path);
        // ExtrusionLine.cpp:297 — Slic3r::append(dst, thick_polyline_to_multi_path(...).paths);
        let path = thick_polyline_to_multi_path(
            &thick_polyline,
            role,
            flow,
            scaled_f(0.05) as f32,
            crate::libslic3r::SCALED_EPSILON as f32,
            overhang,
        )
        .paths;
        dst.extend(path);
    }
}

/// PerimeterGenerator.cpp:604-626 — `detect_brigde_wall_arachne`. For each overhang
/// Z-path (produced by `clip_extrusion(..., Difference)`), decide bridge vs curved
/// overhang: if the straight end-to-end distance is shorter than the polyline
/// length the wall is CURVED (degree = OVERHANG_SAMPLING_NUMBER - 1); otherwise it
/// is a straight BRIDGE wall (degree = OVERHANG_SAMPLING_NUMBER). Appends each via
/// `extrusion_paths_append_zpaths` with the matching degree. Mirrors the classic
/// `detect_bridge_wall` (perimeter_generator.rs) for the arachne Z-path world.
/// (Wired into `arachne_line_to_extrusion_path` — R412 ports the fn; R413 wires it.)
#[allow(dead_code)]
pub fn detect_bridge_wall_arachne(
    dst: &mut ExtrusionPaths,
    path_overhang: &crate::clipper_z_utils::ZPaths,
    role: ExtrusionRole,
    flow: &Flow,
) {
    let n = crate::overhang_detector::OVERHANG_SAMPLING_NUMBER as f64;
    for zpath in path_overhang {
        // Arachne sometimes emits zero-length paths (two identical endpoints).
        if zpath.len() < 2 {
            continue;
        }
        // PerimeterGenerator.cpp:610-611 — Line(front, back).length() vs polyline length.
        let (fx, fy, _) = zpath[0];
        let (lx, ly, _) = *zpath.last().unwrap();
        let line_len = (((lx - fx) as f64).powi(2) + ((ly - fy) as f64).powi(2)).sqrt();
        let mut poly_len = 0.0_f64;
        for w in zpath.windows(2) {
            let (ax, ay, _) = w[0];
            let (bx, by, _) = w[1];
            poly_len += (((bx - ax) as f64).powi(2) + ((by - ay) as f64).powi(2)).sqrt();
        }
        // curved ⇒ overhang (n-1); straight ⇒ bridge (n). cpp:611-624.
        let degree = if line_len < poly_len { n - 1.0 } else { n };
        extrusion_paths_append_zpaths(dst, &vec![zpath.clone()], role, flow, degree);
    }
}

// ExtrusionLine.cpp:301-305 — void extrusion_paths_append(ExtrusionPaths &dst, const Arachne::ExtrusionLine &extrusion, const ExtrusionRole role, const Flow &flow, double overhang)
pub fn extrusion_paths_append_line(
    dst: &mut ExtrusionPaths,
    extrusion: &ExtrusionLine,
    role: ExtrusionRole,
    flow: &Flow,
    overhang: f64,
) {
    // ExtrusionLine.cpp:303
    let thick_polyline = to_thick_polyline(extrusion);
    // ExtrusionLine.cpp:304 — Slic3r::append(dst, thick_polyline_to_multi_path(...).paths);
    let path = thick_polyline_to_multi_path(
        &thick_polyline,
        role,
        flow,
        scaled_f(0.05) as f32,
        crate::libslic3r::SCALED_EPSILON as f32,
        overhang,
    )
    .paths;
    dst.extend(path);
}

// ExtrusionLine.cpp:307-310 — void extrusion_path_append(ExtrusionPaths &dst, const ThickPolyline &thick_polyline, const ExtrusionRole role, const Flow &flow, double overhang)
pub fn extrusion_path_append(
    dst: &mut ExtrusionPaths,
    thick_polyline: &ThickPolyline,
    role: ExtrusionRole,
    flow: &Flow,
    overhang: f64,
) {
    // ExtrusionLine.cpp:309 — Slic3r::append(dst, thick_polyline_to_multi_path(...).paths);
    let path = thick_polyline_to_multi_path(
        thick_polyline,
        role,
        flow,
        scaled_f(0.05) as f32,
        crate::libslic3r::SCALED_EPSILON as f32,
        overhang,
    )
    .paths;
    dst.extend(path);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_extrusion_line_new() {
        let line = ExtrusionLine::new(0, false);
        assert_eq!(line.inset_idx, 0);
        assert_eq!(line.is_odd, false);
        assert_eq!(line.is_closed, false);
        assert!(line.junctions.is_empty());
    }

    #[test]
    fn test_extrusion_line_with_closed() {
        let line = ExtrusionLine::with_closed(1, true, true);
        assert_eq!(line.inset_idx, 1);
        assert_eq!(line.is_odd, true);
        assert_eq!(line.is_closed, true);
    }

    #[test]
    fn test_extrusion_line_length() {
        let mut line = ExtrusionLine::new(0, false);
        line.push_back(ExtrusionJunction::new(Point::new(0, 0), 100, 0));
        line.push_back(ExtrusionJunction::new(Point::new(1000, 0), 100, 0));
        line.push_back(ExtrusionJunction::new(Point::new(1000, 1000), 100, 0));

        let length = line.get_length();
        // Should be 1000 + 1000 = 2000
        assert_eq!(length, 2000);
    }

    #[test]
    fn test_extrusion_line_minimal_width() {
        let mut line = ExtrusionLine::new(0, false);
        line.push_back(ExtrusionJunction::new(Point::new(0, 0), 100, 0));
        line.push_back(ExtrusionJunction::new(Point::new(1000, 0), 50, 0));
        line.push_back(ExtrusionJunction::new(Point::new(2000, 0), 75, 0));

        assert_eq!(line.get_minimal_width(), 50);
    }

    #[test]
    fn test_extrusion_line_to_polygon() {
        let mut line = ExtrusionLine::new(0, false);
        line.push_back(ExtrusionJunction::new(Point::new(0, 0), 100, 0));
        line.push_back(ExtrusionJunction::new(Point::new(1000, 0), 100, 0));
        line.push_back(ExtrusionJunction::new(Point::new(1000, 1000), 100, 0));

        let poly = line.to_polygon();
        assert_eq!(poly.points.len(), 3);
        assert_eq!(poly.points[0], Point::new(0, 0));
        assert_eq!(poly.points[1], Point::new(1000, 0));
        assert_eq!(poly.points[2], Point::new(1000, 1000));
    }

    #[test]
    fn test_simplify_removes_colinear_points() {
        let mut line = ExtrusionLine::new(0, false);
        // Create a straight line with a colinear point in the middle
        line.push_back(ExtrusionJunction::new(Point::new(0, 0), 100, 0));
        line.push_back(ExtrusionJunction::new(Point::new(500, 0), 100, 0));
        line.push_back(ExtrusionJunction::new(Point::new(1000, 0), 100, 0));

        line.simplify(100, 100, 1000);

        // Middle point should be removed (colinear)
        assert_eq!(line.junctions.len(), 2);
        assert_eq!(line.junctions[0].p, Point::new(0, 0));
        assert_eq!(line.junctions[1].p, Point::new(1000, 0));
    }

    #[test]
    fn test_calculate_extrusion_area_deviation() {
        let a = ExtrusionJunction::new(Point::new(0, 0), 100, 0);
        let b = ExtrusionJunction::new(Point::new(500, 0), 120, 0);
        let c = ExtrusionJunction::new(Point::new(1000, 0), 100, 0);

        let mut weighted_avg = 0;
        let error =
            ExtrusionLine::calculate_extrusion_area_deviation_error(a, b, c, &mut weighted_avg);

        // Should compute weighted average and error
        assert!(weighted_avg > 0);
        assert!(error >= 0);
    }

    #[test]
    fn test_should_apply_hole_compensation() {
        let mut line = ExtrusionLine::new(0, false);
        let mut j1 = ExtrusionJunction::new(Point::new(0, 0), 100, 0);
        j1.hole_compensation_flag = true;
        let mut j2 = ExtrusionJunction::new(Point::new(1000, 0), 100, 0);
        j2.hole_compensation_flag = true;
        let j3 = ExtrusionJunction::new(Point::new(2000, 0), 100, 0);

        line.push_back(j1);
        line.push_back(j2);
        line.push_back(j3);

        // 2/3 of segments are marked, should exceed 0.5 threshold
        assert!(line.should_apply_hole_compensation(0.5));
        // But not 0.8 threshold
        assert!(!line.should_apply_hole_compensation(0.8));
    }

    #[test]
    fn test_is_contour_open_line() {
        let mut line = ExtrusionLine::new(0, false);
        line.push_back(ExtrusionJunction::new(Point::new(0, 0), 100, 0));
        line.push_back(ExtrusionJunction::new(Point::new(1000, 0), 100, 0));
        line.push_back(ExtrusionJunction::new(Point::new(1000, 1000), 100, 0));

        assert!(!line.is_contour()); // Not closed
    }

    #[test]
    fn test_area_calculation() {
        let mut line = ExtrusionLine::with_closed(0, false, true);
        // Create a square: 1000x1000
        line.push_back(ExtrusionJunction::new(Point::new(0, 0), 100, 0));
        line.push_back(ExtrusionJunction::new(Point::new(1000, 0), 100, 0));
        line.push_back(ExtrusionJunction::new(Point::new(1000, 1000), 100, 0));
        line.push_back(ExtrusionJunction::new(Point::new(0, 1000), 100, 0));

        let area = line.area();
        // Area of 1000x1000 square = 1,000,000
        // Depending on winding order, could be positive or negative
        assert!((area.abs() - 1_000_000.0).abs() < 1.0);
    }

    #[test]
    fn test_reverse() {
        let mut line = ExtrusionLine::new(0, false);
        line.push_back(ExtrusionJunction::new(Point::new(0, 0), 100, 0));
        line.push_back(ExtrusionJunction::new(Point::new(1000, 0), 100, 0));
        line.push_back(ExtrusionJunction::new(Point::new(2000, 0), 100, 0));

        line.reverse();

        assert_eq!(line.junctions[0].p, Point::new(2000, 0));
        assert_eq!(line.junctions[1].p, Point::new(1000, 0));
        assert_eq!(line.junctions[2].p, Point::new(0, 0));
    }

    #[test]
    fn test_insert_and_remove() {
        let mut line = ExtrusionLine::new(0, false);
        line.push_back(ExtrusionJunction::new(Point::new(0, 0), 100, 0));
        line.push_back(ExtrusionJunction::new(Point::new(2000, 0), 100, 0));

        // Insert in the middle
        line.insert(1, ExtrusionJunction::new(Point::new(1000, 0), 100, 0));
        assert_eq!(line.junctions.len(), 3);
        assert_eq!(line.junctions[1].p, Point::new(1000, 0));

        // Remove the middle junction
        line.remove(1);
        assert_eq!(line.junctions.len(), 2);
        assert_eq!(line.junctions[1].p, Point::new(2000, 0));
    }
}
