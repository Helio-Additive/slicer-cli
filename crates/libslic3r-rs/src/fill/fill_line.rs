//! Line infill pattern.
//!
//! C++ Reference:
//! - Fill/FillLine.hpp
//! - Fill/FillLine.cpp
//!
//! Faithful 1:1 line-by-line port of `Slic3r::FillLine` (FillLine.cpp). Line
//! infill is a variant of rectilinear infill where alternating lines are offset
//! horizontally (`_line_oscillation`) to create a zig-zag wiggle, and adjacent
//! lines are greedily connected into continuous paths.

// FillLine.cpp:1-6
//   #include "../ClipperUtils.hpp"
//   #include "../ExPolygon.hpp"
//   #include "../ShortestPath.hpp"
//   #include "../Surface.hpp"
//   #include "FillLine.hpp"
use super::FillParams;
use crate::clipper_utils::{diff_pl, intersection_pl, offset_expolygon, OffsetJoinType};
use crate::geometry::{align_to_grid, BoundingBox, ExPolygon, Line, Point, Polyline};
use crate::shortest_path::chain_polylines;
use crate::{scale, unscale, Coord, CoordF};

// FillLine.cpp:8 — namespace Slic3r

/// Scaled epsilon for geometric tolerances (~0.001mm = 1000 units at 1e5 scale).
/// libslic3r.h: `constexpr coord_t SCALED_EPSILON = scale_(EPSILON)`; matched to
/// the sibling `fill_rectilinear` convention in this crate.
const SCALED_EPSILON: Coord = 1000;

/// Line infill pattern generator.
/// FillLine.hpp:12 — `class FillLine : public Fill`
///
/// The base `Slic3r::Fill::spacing` member that this filler reads and writes
/// (FillBase.hpp:115) is held here directly, mirroring the inherited C++ field.
/// The four `protected` members below are the FillLine-specific state
/// (FillLine.hpp:27-32).
#[derive(Debug, Clone, Default)]
pub struct FillLine {
    /// Base `Fill::spacing`, in unscaled coordinates (FillBase.hpp:115).
    pub spacing: CoordF,

    /// FillLine.hpp:27
    pub _min_spacing: Coord,
    /// FillLine.hpp:28
    pub _line_spacing: Coord,
    /// distance threshold for allowing the horizontal infill lines to be connected into a continuous path
    /// FillLine.hpp:29-30
    pub _diagonal_distance: Coord,
    /// only for line infill
    /// FillLine.hpp:31-32
    pub _line_oscillation: Coord,
}

impl FillLine {
    /// FillLine.hpp:15 — `Fill* clone() const override { return new FillLine(*this); };`
    pub fn clone_box(&self) -> FillLine {
        self.clone()
    }

    /// FillLine.hpp:17 — `bool is_self_crossing() override { return false; }`
    pub fn is_self_crossing(&self) -> bool {
        false
    }

    /// FillLine.hpp:34-37
    /// ```cpp
    /// Line _line(int i, coord_t x, coord_t y_min, coord_t y_max) const {
    ///     coord_t osc = (i & 1) ? this->_line_oscillation : 0;
    ///     return Line(Point(x - osc, y_min), Point(x + osc, y_max));
    /// }
    /// ```
    fn _line(&self, i: i32, x: Coord, y_min: Coord, y_max: Coord) -> Line {
        // FillLine.hpp:35
        let osc: Coord = if i & 1 != 0 { self._line_oscillation } else { 0 };
        // FillLine.hpp:36
        Line::new(Point::new(x - osc, y_min), Point::new(x + osc, y_max))
    }

    /// FillLine.hpp:39-45
    /// ```cpp
    /// bool _can_connect(coord_t dist_X, coord_t dist_Y)
    /// {
    ///     const auto TOLERANCE = coord_t(10 * SCALED_EPSILON);
    ///     return (dist_X >= (this->_line_spacing - this->_line_oscillation) - TOLERANCE)
    ///         && (dist_X <= (this->_line_spacing + this->_line_oscillation) + TOLERANCE)
    ///         && (dist_Y <= this->_diagonal_distance);
    /// }
    /// ```
    fn _can_connect(&self, dist_x: Coord, dist_y: Coord) -> bool {
        // FillLine.hpp:41
        let tolerance: Coord = 10 * SCALED_EPSILON;
        // FillLine.hpp:42-44
        (dist_x >= (self._line_spacing - self._line_oscillation) - tolerance)
            && (dist_x <= (self._line_spacing + self._line_oscillation) + tolerance)
            && (dist_y <= self._diagonal_distance)
    }

    /// FillLine.cpp:10-120 — `void FillLine::_fill_surface_single(...)`
    pub fn _fill_surface_single(
        &mut self,
        params: &FillParams,
        _thickness_layers: u32,
        direction: &(f32, Point),
        mut expolygon: ExPolygon,
        polylines_out: &mut Vec<Polyline>,
    ) {
        // FillLine.cpp:17
        // rotate polygons so that we can work with vertical lines here
        // FillLine.cpp:18
        expolygon.rotate(-(direction.0 as CoordF));

        // FillLine.cpp:20
        self._min_spacing = scale(self.spacing);
        // FillLine.cpp:21
        debug_assert!(params.density > 0.0001 && params.density <= 1.0);
        // FillLine.cpp:22
        self._line_spacing = ((self._min_spacing as CoordF) / params.density as CoordF) as Coord;
        // FillLine.cpp:23
        self._diagonal_distance = self._line_spacing * 2;
        // FillLine.cpp:24
        self._line_oscillation = self._line_spacing - self._min_spacing; // only for Line infill
        // FillLine.cpp:25
        let mut bounding_box: BoundingBox = expolygon.contour.bounding_box();

        // FillLine.cpp:27
        // define flow spacing according to requested density
        // FillLine.cpp:28
        if params.density > 0.9999 && !params.dont_adjust {
            // FillLine.cpp:29
            self._line_spacing =
                super::adjust_solid_spacing(bounding_box.size().x, self._line_spacing);
            // FillLine.cpp:30
            self.spacing = unscale(self._line_spacing);
        } else {
            // FillLine.cpp:32-33
            // extend bounding box so that our pattern will be aligned with other layers
            // Transform the reference point to the rotated coordinate system.
            // FillLine.cpp:34-37
            // align_to_grid(Point, Point spacing, Point base) — Point.hpp:595-596.
            // Reconstructed from the public `align_to_grid(coord, spacing)`:
            //   align_to_grid(c, s, b) = b + align_to_grid(c - b, s)
            let bb_min = bounding_box.min;
            let spacing_pt = Point::new(self._line_spacing, self._line_spacing);
            let base_pt = direction.1.rotate(-(direction.0 as CoordF));
            bounding_box.merge_point(Point::new(
                base_pt.x + align_to_grid(bb_min.x - base_pt.x, spacing_pt.x),
                base_pt.y + align_to_grid(bb_min.y - base_pt.y, spacing_pt.y),
            ));
        }

        // FillLine.cpp:40
        // generate the basic pattern
        // FillLine.cpp:41
        let x_max: Coord = bounding_box.max.x + SCALED_EPSILON;
        // FillLine.cpp:42
        let mut lines: Vec<Line> = Vec::new();
        // FillLine.cpp:43-44
        let mut x: Coord = bounding_box.min.x;
        while x <= x_max {
            lines.push(self._line(lines.len() as i32, x, bounding_box.min.y, bounding_box.max.y));
            x += self._line_spacing;
        }

        // FillLine.cpp:46-51
        // clip paths against a slightly larger expolygon, so that the first and last paths
        // are kept even if the expolygon has vertical sides
        // the minimum offset for preventing edge lines from being clipped is SCALED_EPSILON;
        // however we use a larger offset to support expolygons with slightly skewed sides and
        // not perfectly straight
        //FIXME Vojtech: Update the intersecton function to work directly with lines.
        // FillLine.cpp:52
        let mut polylines_src: Vec<Polyline> = Vec::new();
        // FillLine.cpp:53
        polylines_src.reserve(lines.len());
        // FillLine.cpp:54-60
        for it in lines.iter() {
            polylines_src.push(Polyline::default());
            let pts = &mut polylines_src.last_mut().unwrap().points;
            pts.reserve(2);
            pts.push(it.a);
            pts.push(it.b);
        }
        // FillLine.cpp:61
        let mut polylines: Vec<Polyline> =
            intersection_pl(&polylines_src, &offset_expolygon(&expolygon, scale(0.02) as CoordF, OffsetJoinType::Miter));

        // FillLine.cpp:63
        // FIXME Vojtech: This is only performed for horizontal lines, not for the vertical lines!
        // FillLine.cpp:64
        const INFILL_OVERLAP_OVER_SPACING: f32 = 0.3;
        // FillLine.cpp:65
        // How much to extend an infill path from expolygon outside?
        // FillLine.cpp:66
        let extra: Coord =
            (self._min_spacing as f32 * INFILL_OVERLAP_OVER_SPACING + 0.5).floor() as Coord;
        // FillLine.cpp:67
        for it_polyline in polylines.iter_mut() {
            // FillLine.cpp:68-69
            // first_point and last_point are *references* into the polyline that get
            // swapped (by reference) so that first_point is the lower-Y endpoint.
            let n = it_polyline.points.len();
            let mut first_idx: usize = 0;
            let mut last_idx: usize = n - 1;
            // FillLine.cpp:70-71
            if it_polyline.points[first_idx].y() > it_polyline.points[last_idx].y() {
                std::mem::swap(&mut first_idx, &mut last_idx);
            }
            // FillLine.cpp:72
            it_polyline.points[first_idx].y -= extra;
            // FillLine.cpp:73
            it_polyline.points[last_idx].y += extra;
        }

        // FillLine.cpp:76
        let n_polylines_out_old: usize = polylines_out.len();

        // FillLine.cpp:78
        // connect lines
        // FillLine.cpp:79
        if !params.dont_connect() && !polylines.is_empty() {
            // prevent calling leftmost_point() on empty collections
            // FillLine.cpp:80
            // offset the expolygon by max(min_spacing/2, extra)
            // FillLine.cpp:81
            let mut expolygon_off: ExPolygon = ExPolygon::default();
            // FillLine.cpp:82
            {
                // FillLine.cpp:83
                let mut expolygons_off: Vec<ExPolygon> = offset_expolygon(
                    &expolygon,
                    (self._min_spacing / 2) as CoordF,
                    OffsetJoinType::Miter,
                );
                // FillLine.cpp:84
                if !expolygons_off.is_empty() {
                    // FillLine.cpp:85
                    // When expanding a polygon, the number of islands could only shrink. Therefore the offset_ex shall generate exactly one expanded island for one input island.
                    // FillLine.cpp:86
                    debug_assert!(expolygons_off.len() == 1);
                    // FillLine.cpp:87
                    std::mem::swap(&mut expolygon_off, &mut expolygons_off[0]);
                }
            }
            // FillLine.cpp:90
            let mut first: bool = true;
            // FillLine.cpp:91
            for polyline in chain_polylines(std::mem::take(&mut polylines), None) {
                // FillLine.cpp:92
                if !first {
                    // FillLine.cpp:93
                    // Try to connect the lines.
                    // FillLine.cpp:94
                    // pts_end is a mutable reference to the points of the last output polyline.
                    // FillLine.cpp:95
                    let first_point: Point = polyline.points.first().copied().unwrap();
                    // FillLine.cpp:96
                    let last_point: Point = polylines_out.last().unwrap().points.last().copied().unwrap();
                    // FillLine.cpp:97-98
                    // Distance in X, Y.
                    let distance: Point = last_point - first_point;
                    // FillLine.cpp:99-100
                    // TODO: we should also check that both points are on a fill_boundary to avoid
                    // connecting paths on the boundaries of internal regions
                    // FillLine.cpp:101-102
                    if self._can_connect(distance.x().abs(), distance.y().abs())
                        && expolygon_contains_line(&expolygon_off, &Line::new(last_point, first_point))
                    {
                        // FillLine.cpp:103
                        // Append the polyline.
                        // FillLine.cpp:104
                        polylines_out
                            .last_mut()
                            .unwrap()
                            .points
                            .extend_from_slice(&polyline.points);
                        // FillLine.cpp:105
                        continue;
                    }
                }
                // FillLine.cpp:108
                // The lines cannot be connected.
                // FillLine.cpp:109
                polylines_out.push(polyline);
                // FillLine.cpp:110
                first = false;
            }
        }

        // FillLine.cpp:114
        // paths must be rotated back
        // FillLine.cpp:115
        for it in polylines_out.iter_mut().skip(n_polylines_out_old) {
            // FillLine.cpp:116-117
            // No need to translate, the absolute position is irrelevant.
            // it->translate(- direction.second(0), - direction.second(1));
            // FillLine.cpp:118
            it.rotate(direction.0 as CoordF);
        }
    }
}

/// `ExPolygon::contains(const Line&)` — ExPolygon.cpp:76-90.
///
/// ```cpp
/// bool ExPolygon::contains(const Line &line) const
/// { return this->contains(Polyline(line.a, line.b)); }
///
/// bool ExPolygon::contains(const Polyline &polyline) const
/// {
///     BoundingBox bbox1 = get_extents(*this);
///     BoundingBox bbox2 = get_extents(polyline);
///     bbox2.inflated(1);
///     if (!bbox1.overlap(bbox2))
///         return false;
///     return diff_pl(polyline, *this).empty();
/// }
/// ```
///
/// NOTE: the C++ `bbox2.inflated(1)` is a no-op in the original — `inflated`
/// returns a copy that is immediately discarded; the un-inflated `bbox2` is used
/// in the overlap test. We faithfully reproduce that bug-for-bug behavior.
fn expolygon_contains_line(expolygon: &ExPolygon, line: &Line) -> bool {
    // ExPolygon.cpp:78 — contains(Polyline(line.a, line.b))
    let polyline = Polyline::from_points(vec![line.a, line.b]);
    // ExPolygon.cpp:83
    let bbox1: BoundingBox = expolygon.contour.bounding_box();
    // ExPolygon.cpp:84 — get_extents(polyline) (Polyline.cpp:517-520)
    let bbox2: BoundingBox = polyline.bounding_box();
    // ExPolygon.cpp:85 — bbox2.inflated(1); (return value discarded, see NOTE; no-op)
    // ExPolygon.cpp:86-87 — `!bbox1.overlap(bbox2)`. `BoundingBox::overlap`
    // (BoundingBox.hpp) is the axis-aligned overlap test, which this crate
    // exposes as `BoundingBox::intersects`.
    if !bbox1.intersects(&bbox2) {
        return false;
    }
    // ExPolygon.cpp:89
    diff_pl(&[polyline], std::slice::from_ref(expolygon)).is_empty()
}
