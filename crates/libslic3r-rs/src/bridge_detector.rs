//! Bridge detection: optimizes the direction of bridges over a region.
//!
//! Line-by-line port of BridgeDetector.hpp and BridgeDetector.cpp at the pin, over the ClipperLib port
//! (`crate::clipper`) through the ClipperUtils operations of `crate::support::support_common::cu`.
//! Compared with the pin by tests/bridge_detector_native_parity.rs (tests/data/bridge_detector/gen.cpp).
//!
//! The bridge detector optimizes a direction of bridges over a region or a set of regions.
//! A bridge direction is considered optimal, if the length of the lines strang over the region
//! is maximal. This is optimal if the bridge is supported in a single direction only, but it may
//! not likely be optimal, if the bridge region is supported from all sides. Then an optimal
//! solution would find a direction with shortest bridges.
//! The bridge orientation is measured CCW from the X axis.
//!
//! Floating point: the C++ is compiled with `-ffp-contract=on`, so `a * b - c * d` is
//! `fma(a, b, -(c * d))` and `a * b + c * d` is `fma(a, b, c * d)`; `sin(x)` and `cos(x)` of one
//! argument in one function are one `__sincos_stret` call (`crate::clipper::sin_cos`). coord_t is i64.

// BridgeDetector.cpp:1-4
use crate::clipper::{libcxx_sort, sin_cos, ClipType};
use crate::clipper_utils as u;
use crate::geometry::{directions_parallel, ExPolygon, ExPolygons, Line, Lines, Point, PointF, Polygon, Polygons, Polyline};
use crate::libslic3r::EPSILON;
use crate::support::support_common::{cu, cu_expolygon, cu_polygon, BBox};
use crate::{Coord, CoordF};
use cu::ApplySafetyOffset;
use std::f64::consts::PI;

type Polylines = Vec<Polyline>;

// BridgeDetector.hpp:22-74
// The bridge detector optimizes a direction of bridges over a region or a set of regions.
pub struct BridgeDetector {
    // BridgeDetector.hpp:25 The non-grown holes.
    // (In C++ this is a reference into expolygons_owned or the caller's data.)
    expolygons: ExPolygons,
    // BridgeDetector.hpp:29 Lower slices, all regions.
    lower_slices: ExPolygons,
    // BridgeDetector.hpp:31 Scaled extrusion width of the infill.
    pub spacing: Coord,
    // BridgeDetector.hpp:33 Angle resolution for the brute force search of the best bridging angle.
    pub resolution: CoordF,
    // BridgeDetector.hpp:35 The final optimal angle.
    pub angle: CoordF,

    // BridgeDetector.hpp:71 Open lines representing the supporting edges.
    _edges: Polylines,
    // BridgeDetector.hpp:73 Closed polygons representing the supporting areas.
    _anchor_regions: ExPolygons,
}

// BridgeDetector.hpp:54-65
#[derive(Debug, Clone, Copy)]
struct BridgeDirection {
    // BridgeDetector.hpp:61
    angle: CoordF,
    // BridgeDetector.hpp:62
    coverage: CoordF,
    // BridgeDetector.hpp:63
    max_length: CoordF,
    // BridgeDetector.hpp:64
    archored_percent: CoordF,
}

impl BridgeDirection {
    // BridgeDetector.hpp:55
    fn new(a: CoordF) -> Self {
        Self { angle: a, coverage: 0., max_length: 0., archored_percent: 0. }
    }

    // BridgeDetector.hpp:57-60 operator<
    // the best direction is the one causing most lines to be bridged (thus most coverage)
    fn less(&self, other: &BridgeDirection) -> bool {
        // Initial sort by coverage only - comparator must obey strict weak ordering
        self.coverage > other.coverage //this->archored_percent > other.archored_percent;
    }
}

impl BridgeDetector {
    // BridgeDetector.cpp:8-20
    // BridgeDetector::BridgeDetector(ExPolygon, const ExPolygons&, coord_t)
    pub fn new(expolygon: ExPolygon, lower_slices: &ExPolygons, spacing: Coord) -> Self {
        let mut detector = Self {
            // BridgeDetector.cpp:13 The original infill polygon, not inflated.
            // BridgeDetector.cpp:18 this->expolygons_owned.push_back(std::move(_expolygon));
            expolygons: vec![expolygon],
            // BridgeDetector.cpp:15 All surfaces of the object supporting this region.
            lower_slices: lower_slices.clone(),
            // BridgeDetector.cpp:16
            spacing,
            resolution: 0.,
            angle: -1.,
            _edges: Vec::new(),
            _anchor_regions: Vec::new(),
        };
        // BridgeDetector.cpp:19
        detector.initialize();
        detector
    }

    // BridgeDetector.cpp:22-33
    // BridgeDetector::BridgeDetector(const ExPolygons&, const ExPolygons&, coord_t)
    pub fn new_multi(expolygons: &ExPolygons, lower_slices: &ExPolygons, spacing: Coord) -> Self {
        let mut detector = Self {
            // BridgeDetector.cpp:27 The original infill polygon, not inflated.
            expolygons: expolygons.clone(),
            // BridgeDetector.cpp:29 All surfaces of the object supporting this region.
            lower_slices: lower_slices.clone(),
            // BridgeDetector.cpp:30
            spacing,
            resolution: 0.,
            angle: -1.,
            _edges: Vec::new(),
            _anchor_regions: Vec::new(),
        };
        // BridgeDetector.cpp:32
        detector.initialize();
        detector
    }

    // BridgeDetector.cpp:35-73
    fn initialize(&mut self) {
        // 5 degrees stepping
        self.resolution = PI / 36.0;
        // output angle not known
        self.angle = -1.;

        // Outset our bridge by an arbitrary amout; we'll use this outer margin for detecting anchors.
        // offset(const ExPolygons &, float): ClipperUtils.cpp:561
        let grown: Polygons = cu::offset_expolygons(&self.expolygons, self.spacing as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);

        // Detect possible anchoring edges of this bridging region.
        // Detect what edges lie on lower slices by turning bridge contour and holes
        // into polylines and then clipping them with each lower slice's contour.
        // Currently _edges are only used to set a candidate direction of the bridge (see bridge_direction_candidates()).
        let mut contours: Polygons = Vec::with_capacity(self.lower_slices.len());
        for expoly in self.lower_slices.iter() {
            contours.push(expoly.contour.clone());
        }
        // intersection_pl(const Polylines &, const Polygons &)
        self._edges = cu::intersection_pl(&cu_polygon::to_polylines(&grown), &contours);

        // (SLIC3R_DEBUG) printf("  bridge has %zu support(s)\n", this->_edges.size());

        // detect anchors as intersection between our bridge expolygon and the lower slices
        // safety offset required to avoid Clipper from detecting empty intersection while Boost actually found some edges
        // union_safety_offset(const ExPolygons &): ClipperUtils.hpp `offset(expolygons, ClipperSafetyOffset)`
        self._anchor_regions = cu::intersection_ex(
            &grown,
            &cu::offset_expolygons(&self.lower_slices, cu::CLIPPER_SAFETY_OFFSET, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
            ApplySafetyOffset::No,
        );
    }

    // BridgeDetector.cpp:75-177
    // If bridge_direction_override != 0, then the angle is used instead of auto-detect.
    pub fn detect_angle(&mut self, bridge_direction_override: CoordF) -> bool {
        if self._edges.is_empty() || self._anchor_regions.is_empty() {
            // The bridging region is completely in the air, there are no anchors available at the layer below.
            return false;
        }

        let mut candidates: Vec<BridgeDirection> = Vec::new();
        if bridge_direction_override == 0. {
            let angles = self.bridge_direction_candidates();
            candidates.reserve(angles.len());
            for i in 0..angles.len() {
                candidates.push(BridgeDirection::new(angles[i]));
            }
        } else {
            candidates.push(BridgeDirection::new(bridge_direction_override));
        }

        /*  Outset the bridge expolygon by half the amount we used for detecting anchors;
            we'll use this one to clip our test lines and be sure that their endpoints
            are inside the anchors and not on their contours leading to false negatives. */
        let clip_area: Polygons = cu::offset_expolygons(&self.expolygons, 0.5f32 * self.spacing as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);

        /*  we'll now try several directions using a rudimentary visibility check:
            bridge in several directions and then sum the length of lines having both
            endpoints within anchors */

        let mut have_coverage = false;
        for i_angle in 0..candidates.len() {
            let angle = candidates[i_angle].angle;

            let mut lines: Lines = Vec::new();
            {
                // Get an oriented bounding box around _anchor_regions.
                let bbox = get_extents_rotated(&self._anchor_regions, -angle);
                // Cover the region with line segments.
                // lines.reserve((bbox.max(1) - bbox.min(1) + this->spacing) / this->spacing);
                let (s, c) = sin_cos(angle);
                //FIXME Vojtech: The lines shall be spaced half the line width from the edge, but then
                // some of the test cases fail. Need to adjust the test cases then?
                //            for (coord_t y = bbox.min(1) + this->spacing / 2; y <= bbox.max(1); y += this->spacing)
                let (min0, max0) = (bbox.min.x as f64, bbox.max.x as f64);
                let mut y = bbox.min.y;
                while y <= bbox.max.y {
                    let yf = y as f64;
                    // Point((coord_t)round(c * bbox.min(0) - s * y), (coord_t)round(c * y + s * bbox.min(0)))
                    lines.push(Line::new(
                        Point::new(c.mul_add(min0, -(s * yf)).round() as Coord, c.mul_add(yf, s * min0).round() as Coord),
                        Point::new(c.mul_add(max0, -(s * yf)).round() as Coord, c.mul_add(yf, s * max0).round() as Coord),
                    ));
                    y += self.spacing;
                }
            }

            let mut total_length: CoordF = 0.;
            let mut max_length: CoordF = 0.;
            {
                let clipped_lines = intersection_ln(&lines, &clip_area);
                let mut archored_line_num: usize = 0;
                for i in 0..clipped_lines.len() {
                    let line = &clipped_lines[i];
                    if expolygons_contain(&self._anchor_regions, &line.a) && expolygons_contain(&self._anchor_regions, &line.b) {
                        // This line could be anchored.
                        let len = line_length(line);
                        total_length += len;
                        // std::max(max_length, len)
                        max_length = if max_length < len { len } else { max_length };
                        archored_line_num += 1;
                    }
                }
                if !clipped_lines.is_empty() && archored_line_num > 0 {
                    candidates[i_angle].archored_percent = archored_line_num as f64 / clipped_lines.len() as f64;
                }
            }
            if total_length == 0. {
                continue;
            }

            have_coverage = true;
            // Sum length of bridged lines.
            candidates[i_angle].coverage = total_length;
            /*  The following produces more correct results in some cases and more broken in others.
                TODO: investigate, as it looks more reliable than line clipping. */
            // $directions_coverage{$angle} = sum(map $_->area, @{$self->coverage($angle)}) // 0;
            // max length of bridged lines
            candidates[i_angle].max_length = max_length;
        }

        // if no direction produced coverage, then there's no bridge direction
        if !have_coverage {
            return false;
        }

        // sort directions by coverage - most coverage first
        // std::sort(candidates.begin(), candidates.end()): the order libc++ leaves equal coverages in
        libcxx_sort(&mut candidates, |a, b| a.less(b));

        // if any other direction is within extrusion width of coverage, prefer it if shorter
        // TODO: There are two options here - within width of the angle with most coverage, or within width of the currently perferred?
        let mut i_best: usize = 0;
        //    for (size_t i = 1; i < candidates.size() && abs(candidates[i_best].archored_percent - candidates[i].archored_percent) < EPSILON; ++ i)
        let mut i = 1;
        while i < candidates.len() && candidates[i_best].coverage - candidates[i].coverage < self.spacing as f64 {
            if candidates[i].max_length < candidates[i_best].max_length {
                i_best = i;
            }
            i += 1;
        }

        // Diagnostics only (BRIDGEPROBE): the candidates in their sorted order.
        if crate::probe_enabled("BRIDGEPROBE") {
            use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
            static CALL: AtomicUsize = AtomicUsize::new(0);
            let call = CALL.fetch_add(1, Relaxed) + 1;
            if call <= 40 {
                eprintln!("[BRIDGEPROBE] call={call} ncand={} spacing={} i_best={i_best}", candidates.len(), self.spacing);
                for (bi, cd) in candidates.iter().enumerate().take(12) {
                    eprintln!(
                        "[BRIDGECAND] call={call} i={bi} angle={:.6} coverage={:.3} max_length={:.3} anchored={:.6}",
                        cd.angle, cd.coverage, cd.max_length, cd.archored_percent
                    );
                }
            }
        }

        self.angle = candidates[i_best].angle;
        if self.angle >= PI {
            self.angle -= PI;
        }

        // (SLIC3R_DEBUG) printf("  Optimal infill angle is %d degrees\n", (int)Slic3r::Geometry::rad2deg(this->angle));

        true
    }

    // BridgeDetector.cpp:179-214
    // Get possible briging direction candidates. (Private in the C++; public for the parity test.)
    pub fn bridge_direction_candidates(&self) -> Vec<CoordF> {
        // we test angles according to configured resolution
        let mut angles: Vec<CoordF> = Vec::new();
        // for (int i = 0; i <= PI/this->resolution; ++i)
        let mut i: i32 = 0;
        while (i as f64) <= PI / self.resolution {
            angles.push(i as f64 * self.resolution);
            i += 1;
        }

        // we also test angles of each bridge contour
        {
            let lines = to_lines_expolygons(&self.expolygons);
            for line in lines.iter() {
                angles.push(line_direction(line));
            }
        }

        /*  we also test angles of each open supporting edge
            (this finds the optimal angle for C-shaped supports) */
        for edge in self._edges.iter() {
            let (first, last) = (edge.points[0], edge.points[edge.points.len() - 1]);
            if first != last {
                angles.push(line_direction(&Line::new(first, last)));
            }
        }

        // remove duplicates
        let min_resolution = PI / 180.0; // 1 degree
        libcxx_sort(&mut angles, |a, b| a < b);
        // for (size_t i = 1; i < angles.size(); ++i) { if (parallel) { erase(i); --i; } }
        let mut i = 1usize;
        while i < angles.len() {
            if directions_parallel(angles[i], angles[i - 1], min_resolution) {
                angles.remove(i);
            } else {
                i += 1;
            }
        }
        /*  compare first value with last one and remove the greatest one (PI)
            in case they are parallel (PI, 0) */
        if directions_parallel(angles[0], angles[angles.len() - 1], min_resolution) {
            angles.pop();
        }

        angles
    }

    // BridgeDetector.cpp:314-408
    // Coverage is currently only used by the unit tests. It is extremely slow and unreliable!
    //
    // BridgeDetector.hpp:41-44: The "precise" mode counts anchor overlaps per trapezoid area instead of
    // requiring two long anchored edges. An annular counterbore shoulder only passes the precise test,
    // because the inner side of the ring hangs over the hole and never lands on an anchor.
    // The defaults of the C++ are `angle = -1, precise = false`.
    pub fn coverage(&self, angle: CoordF, precise: bool) -> Polygons {
        let mut angle = angle;
        if angle == -1. {
            angle = self.angle;
        }

        let mut covered: Polygons = Vec::new();

        if angle != -1. {
            // Get anchors, convert them to Polygons and rotate them.
            let mut anchors: Polygons = crate::geometry::to_polygons(&self._anchor_regions);
            polygons_rotate(&mut anchors, PI / 2.0 - angle);

            for expolygon in self.expolygons.iter() {
                let mut expolygon = expolygon.clone();
                // Clone our expolygon and rotate it so that we work with vertical lines.
                expolygon_rotate(&mut expolygon, PI / 2.0 - angle);
                // Outset the bridge expolygon by half the amount we used for detecting anchors;
                // we'll use this one to generate our trapezoids and be sure that their vertices
                // are inside the anchors and not on their contours leading to false negatives.
                for expoly in cu_expolygon::offset_ex(&expolygon, 0.5f32 * self.spacing as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT).iter() {
                    // Compute trapezoids according to a vertical orientation
                    let mut trapezoids: Polygons = Vec::new();
                    if precise {
                        get_trapezoids3_half(expoly, &mut trapezoids, self.spacing as f32);
                    } else {
                        get_trapezoids2_angle(expoly, &mut trapezoids, PI / 2.0);
                    }
                    for trapezoid in trapezoids.iter_mut() {
                        let mut n_supported: usize = 0;
                        if precise {
                            // Count the anchor patches the strip lands on. A ring-shaped overhang has
                            // its two anchors on opposite ends of the strip, which this detects while
                            // the edge-length test below does not.
                            let intersects: Polygons = cu::intersection(std::slice::from_ref(trapezoid), &anchors, ApplySafetyOffset::No);
                            n_supported = intersects.len();
                            if n_supported >= 2 {
                                // Clamp the strip to the span actually covered by the anchors, so the
                                // bridge does not reach past the material it is anchored on.
                                let center_bound = BBox::from_points(&intersects[0].points);
                                let mut min_y = bbox_center(&center_bound).y;
                                let mut max_y = bbox_center(&center_bound).y;
                                for poly_bound in intersects.iter() {
                                    let c = bbox_center(&BBox::from_points(&poly_bound.points));
                                    min_y = min_y.min(c.y);
                                    max_y = max_y.max(c.y);
                                }
                                let mut min_x = trapezoid.points[0].x;
                                let mut max_x = trapezoid.points[0].x;
                                for p in trapezoid.points.iter() {
                                    min_x = min_x.min(p.x);
                                    max_x = max_x.max(p.x);
                                }
                                // Restore the quarter spacing that get_trapezoids3_half() trimmed off.
                                min_x -= self.spacing / 4 + 1;
                                max_x += self.spacing / 4 + 1;
                                let mid_x: Coord = (min_x + max_x) / 2;
                                for p in trapezoid.points.iter_mut() {
                                    // std::clamp(p.y(), min_y, max_y)
                                    p.y = if p.y < min_y {
                                        min_y
                                    } else if max_y < p.y {
                                        max_y
                                    } else {
                                        p.y
                                    };
                                    if p.x > min_x && p.x < mid_x {
                                        p.x = min_x;
                                    }
                                    if p.x < max_x && p.x > mid_x {
                                        p.x = max_x;
                                    }
                                }
                            }
                        } else {
                            // not nice, we need a more robust non-numeric check
                            for supported_line in intersection_ln(&to_lines_polygon(trapezoid), &anchors).iter() {
                                if line_length(supported_line) >= self.spacing as f64 {
                                    n_supported += 1;
                                }
                            }
                        }
                        if n_supported >= 2 {
                            covered.push(std::mem::take(trapezoid));
                        }
                    }
                }
            }

            // Unite the trapezoids before rotation, as the rotation creates tiny gaps and intersections between the trapezoids
            // instead of exact overlaps.
            covered = cu::union_(&covered);
            // Intersect trapezoids with actual bridge area to remove extra margins and append it to result.
            polygons_rotate(&mut covered, -(PI / 2.0 - angle));
            // intersection(const ExPolygons &, const Polygons &): ClipperUtils.cpp
            covered = cu::clipper_paths(ClipType::ctIntersection, &cu::expolygons_provider(&self.expolygons), &cu::polygons_provider(&covered), ApplySafetyOffset::No);
            // (#if 0: the SVG dump)
        }
        covered
    }

    // BridgeDetector.cpp:410-452
    /*  This method returns the bridge edges (as polylines) that are not supported
        but would allow the entire bridge area to be bridged with detected angle
        if supported too */
    // C++: `void BridgeDetector::unsupported_edges(double angle, Polylines* unsupported) const`
    pub fn unsupported_edges_into(&self, angle: CoordF, unsupported: &mut Polylines) {
        let mut angle = angle;
        if angle == -1. {
            angle = self.angle;
        }
        if angle == -1. {
            return;
        }

        let grown_lower: Polygons = cu::offset_expolygons(&self.lower_slices, self.spacing as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT);

        for it_expoly in self.expolygons.iter() {
            // get unsupported bridge edges (both contour and holes)
            let unsupported_lines: Lines = to_lines_polylines(&cu::diff_pl(&cu_expolygon::to_polylines_closed(it_expoly), &grown_lower));
            /*  Split into individual segments and filter out edges parallel to the bridging angle
                TODO: angle tolerance should probably be based on segment length and flow width,
                so that we build supports whenever there's a chance that at least one or two bridge
                extrusions would be anchored within such length (i.e. a slightly non-parallel bridging
                direction might still benefit from anchors if long enough)
                double angle_tolerance = PI / 180.0 * 5.0; */
            for line in unsupported_lines.iter() {
                if !directions_parallel(line_direction(line), angle, 0.) {
                    unsupported.push(Polyline::from_points(vec![line.a, line.b]));
                }
            }
        }
        // (the commented-out SVG dump)
    }

    // BridgeDetector.cpp:454-460
    // C++: `Polylines BridgeDetector::unsupported_edges(double angle = -1) const`
    pub fn unsupported_edges(&self, angle: CoordF) -> Polylines {
        let mut pp: Polylines = Vec::new();
        self.unsupported_edges_into(angle, &mut pp);
        pp
    }

    // Accessors of the private members for callers / tests.
    pub fn anchor_regions(&self) -> &ExPolygons {
        &self._anchor_regions
    }

    pub fn edges(&self) -> &Polylines {
        &self._edges
    }
}

// BridgeDetector.cpp:233-264
// This algorithm may return more trapezoids than necessary
// (i.e. it may break a single trapezoid in several because
// other parts of the object have x coordinates in the middle)
pub fn get_trapezoids2(expoly: &ExPolygon, polygons: &mut Polygons) {
    let src_polygons: Polygons = crate::geometry::to_polygons(std::slice::from_ref(expoly));
    // get all points of this ExPolygon
    let mut pp: Vec<Point> = Vec::new();
    for polygon in src_polygons.iter() {
        pp.extend_from_slice(&polygon.points);
    }

    // build our bounding box
    let bb = BBox::from_points(&pp);

    // get all x coordinates
    let mut xx: Vec<Coord> = Vec::with_capacity(pp.len());
    for p in pp.iter() {
        xx.push(p.x);
    }
    xx.sort();

    // find trapezoids by looping from first to next-to-last coordinate
    let mut rectangle: Polygons = vec![Polygon::default()];
    // for (std::vector<coord_t>::const_iterator x = xx.begin(); x != xx.end()-1; ++x)
    // (an expolygon without points is undefined behaviour there; nothing is done here)
    for idx in 0..xx.len().saturating_sub(1) {
        let x = xx[idx];
        let next_x = xx[idx + 1];
        if x != next_x {
            // intersect with rectangle
            // append results to return value
            rectangle[0] = Polygon::from_points(vec![Point::new(x, bb.min.y), Point::new(next_x, bb.min.y), Point::new(next_x, bb.max.y), Point::new(x, bb.max.y)]);
            polygons.extend(cu::intersection(&rectangle, &src_polygons, ApplySafetyOffset::No));
        }
    }
}

// BridgeDetector.cpp:266-273
// C++: `static void get_trapezoids2(const ExPolygon &expoly, Polygons* polygons, double angle)`
pub fn get_trapezoids2_angle(expoly: &ExPolygon, polygons: &mut Polygons, angle: CoordF) {
    let mut clone = expoly.clone();
    // clone.rotate(PI/2 - angle, Point(0,0));
    expolygon_rotate_center(&mut clone, PI / 2. - angle, &Point::new(0, 0));
    get_trapezoids2(&clone, polygons);
    for polygon in polygons.iter_mut() {
        // polygon.rotate(-(PI/2 - angle), Point(0,0));
        multipoint_rotate_center(&mut polygon.points, -(PI / 2. - angle), &Point::new(0, 0));
    }
}

// BridgeDetector.cpp:275-312
// Slice the expolygon into vertical strips one spacing apart. Unlike get_trapezoids2() the strips
// are cut at a fixed pitch instead of at every vertex, so a curved contour does not explode into
// a huge number of slivers.
pub fn get_trapezoids3_half(expoly: &ExPolygon, polygons: &mut Polygons, spacing: f32) {
    // Points pp = to_points(expoly); ExPolygon.hpp:370-378: the contour, then the holes
    let mut pp: Vec<Point> = Vec::new();
    pp.extend_from_slice(&expoly.contour.points);
    for hole in expoly.holes.iter() {
        pp.extend_from_slice(&hole.points);
    }
    if pp.is_empty() {
        return;
    }

    let bb = BBox::from_points(&pp);

    let mut min_x = pp[0].x;
    let mut max_x = pp[0].x;
    for p in pp.iter() {
        min_x = min_x.min(p.x);
        max_x = max_x.max(p.x);
    }

    let mut xx: Vec<Coord> = Vec::new();
    // for (coord_t x = min_x; x < max_x - coord_t(spacing / 2); x += coord_t(spacing))
    let mut x = min_x;
    while x < max_x - (spacing / 2.) as Coord {
        xx.push(x);
        x += spacing as Coord;
    }
    xx.push(max_x);

    let src_polygons: Polygons = crate::geometry::to_polygons(std::slice::from_ref(expoly));
    for idx in 0..xx.len() - 1 {
        let x = xx[idx];
        let next_x = xx[idx + 1];
        if x == next_x {
            continue;
        }
        // Inset the strip by a quarter spacing on both sides so neighbouring strips do not touch.
        let quarter = spacing as Coord / 4;
        let poly = Polygon::from_points(vec![
            Point::new(x + quarter, bb.min.y),
            Point::new(next_x - quarter, bb.min.y),
            Point::new(next_x - quarter, bb.max.y),
            Point::new(x + quarter, bb.max.y),
        ]);
        polygons.extend(cu::intersection(std::slice::from_ref(&poly), &src_polygons, ApplySafetyOffset::No));
    }
}

// ============================================================================
// The functions of other files the C++ calls here, as that code is written.
// ============================================================================

// Line.cpp:60-66 `double Line::direction() const`, Line.hpp:176 `atan2_()`
pub fn line_direction(line: &Line) -> CoordF {
    let atan2 = ((line.b.y - line.a.y) as f64).atan2((line.b.x - line.a.x) as f64);
    if (atan2 - PI).abs() < EPSILON {
        0.
    } else if atan2 < 0. {
        atan2 + PI
    } else {
        atan2
    }
}

// Line.hpp:163 `double length() const { return (b - a).cast<double>().norm(); }`: the squares are summed
// by Eigen's reduction, not in one expression, so they are not fused.
pub fn line_length(line: &Line) -> CoordF {
    let dx = (line.b.x - line.a.x) as f64;
    let dy = (line.b.y - line.a.y) as f64;
    (dx * dx + dy * dy).sqrt()
}

// ExPolygon.hpp:443-449 `expolygons_contain(ExPolygons &expolys, const Point &pt, bool border_result = true)`
fn expolygons_contain(expolys: &[ExPolygon], pt: &Point) -> bool {
    for expoly in expolys {
        if cu_expolygon::contains(expoly, pt, true) {
            return true;
        }
    }
    false
}

// MultiPoint.cpp:441-465 `BoundingBox get_extents_rotated(const Points &points, double angle)`
fn get_extents_rotated_points(points: &[Point], angle: CoordF) -> BBox {
    let mut bbox = BBox::new();
    if !points.is_empty() {
        let (s, c) = sin_cos(angle);
        let cur_x = points[0].x as f64;
        let cur_y = points[0].y as f64;
        // (coord_t)round(c * cur_x - s * cur_y), (coord_t)round(c * cur_y + s * cur_x)
        let x = c.mul_add(cur_x, -(s * cur_y)).round() as Coord;
        let y = c.mul_add(cur_y, s * cur_x).round() as Coord;
        bbox.min = Point::new(x, y);
        bbox.max = Point::new(x, y);
        for it in &points[1..] {
            let cur_x = it.x as f64;
            let cur_y = it.y as f64;
            let x = c.mul_add(cur_x, -(s * cur_y)).round() as Coord;
            let y = c.mul_add(cur_y, s * cur_x).round() as Coord;
            bbox.min = Point::new(x.min(bbox.min.x), y.min(bbox.min.y));
            bbox.max = Point::new(x.max(bbox.max.x), y.max(bbox.max.y));
        }
        bbox.defined = true;
    }
    bbox
}

// ExPolygon.cpp:511-520 `BoundingBox get_extents_rotated(const ExPolygons &expolygons, double angle)`:
// the contours only.
fn get_extents_rotated(expolygons: &[ExPolygon], angle: CoordF) -> BBox {
    let mut bbox = BBox::new();
    if !expolygons.is_empty() {
        bbox = get_extents_rotated_points(&expolygons[0].contour.points, angle);
        for i in 1..expolygons.len() {
            bbox.merge(&get_extents_rotated_points(&expolygons[i].contour.points, angle));
        }
    }
    bbox
}

// BoundingBox.cpp:231-235 `center()`: `(this->min + this->max) / 2`, the integer division of each coordinate.
fn bbox_center(bbox: &BBox) -> Point {
    Point::new((bbox.min.x + bbox.max.x) / 2, (bbox.min.y + bbox.max.y) / 2)
}

// MultiPoint.cpp:27-35 `void MultiPoint::rotate(double cos_angle, double sin_angle)`
fn multipoint_rotate(points: &mut [Point], cos_angle: CoordF, sin_angle: CoordF) {
    for pt in points.iter_mut() {
        let cur_x = pt.x as f64;
        let cur_y = pt.y as f64;
        // coord_t(round(cos_angle * cur_x - sin_angle * cur_y)), coord_t(round(cos_angle * cur_y + sin_angle * cur_x))
        pt.x = cos_angle.mul_add(cur_x, -(sin_angle * cur_y)).round() as Coord;
        pt.y = cos_angle.mul_add(cur_y, sin_angle * cur_x).round() as Coord;
    }
}

// MultiPoint.cpp:37-46 `void MultiPoint::rotate(double angle, const Point &center)`
fn multipoint_rotate_center(points: &mut [Point], angle: CoordF, center: &Point) {
    let (s, c) = sin_cos(angle);
    for pt in points.iter_mut() {
        let v0 = (pt.x - center.x) as f64;
        let v1 = (pt.y - center.y) as f64;
        // (coord_t)round(double(center(0)) + c * v[0] - s * v[1])
        let x = (-s).mul_add(v1, c.mul_add(v0, center.x as f64)).round() as Coord;
        // (coord_t)round(double(center(1)) + c * v[1] + s * v[0])
        let y = s.mul_add(v0, c.mul_add(v1, center.y as f64)).round() as Coord;
        pt.x = x;
        pt.y = y;
    }
}

// Polygon.hpp:165-171 `polygons_rotate(Polygons &polys, double angle)`
fn polygons_rotate(polys: &mut Polygons, angle: CoordF) {
    let (sin_angle, cos_angle) = sin_cos(angle);
    for p in polys.iter_mut() {
        multipoint_rotate(&mut p.points, cos_angle, sin_angle);
    }
}

// ExPolygon.cpp:38-43 `void ExPolygon::rotate(double angle)`, over MultiPoint.hpp:31
// `void rotate(double angle) { this->rotate(cos(angle), sin(angle)); }`
fn expolygon_rotate(expolygon: &mut ExPolygon, angle: CoordF) {
    let (sin_angle, cos_angle) = sin_cos(angle);
    multipoint_rotate(&mut expolygon.contour.points, cos_angle, sin_angle);
    for hole in expolygon.holes.iter_mut() {
        multipoint_rotate(&mut hole.points, cos_angle, sin_angle);
    }
}

// ExPolygon.cpp:45-50 `void ExPolygon::rotate(double angle, const Point &center)`
fn expolygon_rotate_center(expolygon: &mut ExPolygon, angle: CoordF, center: &Point) {
    multipoint_rotate_center(&mut expolygon.contour.points, angle, center);
    for hole in expolygon.holes.iter_mut() {
        multipoint_rotate_center(&mut hole.points, angle, center);
    }
}

// ClipperUtils.cpp:940-958 `_clipper_ln(ctIntersection, subject, clip)` through ClipperUtils.hpp:536-539
// `intersection_ln(const Lines &subject, const Polygons &clip)`
fn intersection_ln(subject: &Lines, clip: &Polygons) -> Lines {
    // convert Lines to Polylines
    let mut polylines: Polylines = Vec::with_capacity(subject.len());
    for line in subject.iter() {
        polylines.push(Polyline::from_points(vec![line.a, line.b]));
    }

    // perform operation
    let paths = u::_clipper_pl_open(ClipType::ctIntersection, &cu::polylines_provider(&polylines), &cu::polygons_provider(clip));

    // convert Polylines to Lines
    let mut retval: Lines = Vec::new();
    for polyline in paths.iter() {
        if polyline.len() >= 2 {
            //FIXME It may happen, that Clipper produced a polyline with more than 2 collinear points by clipping a single line with polygons. It is a very rare issue, but it happens, see GH #6933.
            let (front, back) = (&polyline[0], &polyline[polyline.len() - 1]);
            retval.push(Line::new(Point::new(front.x as Coord, front.y as Coord), Point::new(back.x as Coord, back.y as Coord)));
        }
    }
    retval
}

// ExPolygon.hpp:135-148 `Lines to_lines(const ExPolygons &src)`
fn to_lines_expolygons(src: &[ExPolygon]) -> Lines {
    let mut lines: Lines = Vec::new();
    for it_expoly in src {
        for i in 0..=it_expoly.holes.len() {
            let points = if i == 0 { &it_expoly.contour.points } else { &it_expoly.holes[i - 1].points };
            if points.is_empty() {
                continue;
            }
            for k in 0..points.len() - 1 {
                lines.push(Line::new(points[k], points[k + 1]));
            }
            lines.push(Line::new(points[points.len() - 1], points[0]));
        }
    }
    lines
}

// Polygon.hpp:199-209 `Lines to_lines(const Polygon &poly)` (`Polygon::lines()`, Polygon.cpp:22-25)
fn to_lines_polygon(poly: &Polygon) -> Lines {
    let mut lines: Lines = Vec::with_capacity(poly.points.len());
    if poly.points.len() > 2 {
        for k in 0..poly.points.len() - 1 {
            lines.push(Line::new(poly.points[k], poly.points[k + 1]));
        }
        lines.push(Line::new(poly.points[poly.points.len() - 1], poly.points[0]));
    }
    lines
}

// Polyline.hpp:181-195 `Lines to_lines(const Polylines &polys)`
fn to_lines_polylines(polys: &[Polyline]) -> Lines {
    let mut lines: Lines = Vec::new();
    for poly in polys {
        if poly.points.is_empty() {
            continue;
        }
        for k in 0..poly.points.len() - 1 {
            lines.push(Line::new(poly.points[k], poly.points[k + 1]));
        }
    }
    lines
}

// ============================================================================
// Header inline free functions.
// ============================================================================

/// The iteration order of a libc++ `std::unordered_map<double, Vec2d>` that is only inserted into
/// (`emplace`), which is what `detect_bridging_direction` walks (BridgeDetector.hpp:90-100, 141-151).
///
/// libc++ keeps all nodes in one singly linked list and an array of buckets that point at the node
/// before a bucket's first node (`__hash_table`): a node whose bucket is empty goes to the front of
/// the list, any other one right behind its bucket's predecessor; the table starts with two buckets
/// and grows to the next prime at or above `2 * buckets + 1` when the load factor would pass 1,
/// re-linking the nodes in list order. `std::hash<double>` is the bit pattern, with both zeros
/// hashing to 0.
struct LibcxxUnorderedMapF64 {
    /// (hash, key, value, next)
    nodes: Vec<(u64, f64, PointF, Option<usize>)>,
    /// First node of the list (`__p1_.first().__next_`).
    head: Option<usize>,
    /// Per bucket: the node before its first node. `Some(None)` is the before-begin anchor, `None`
    /// an empty bucket.
    buckets: Vec<Option<Option<usize>>>,
}

impl LibcxxUnorderedMapF64 {
    fn new() -> Self {
        Self { nodes: Vec::new(), head: None, buckets: Vec::new() }
    }

    fn hash(key: f64) -> u64 {
        if key == 0.0 {
            0
        } else {
            key.to_bits()
        }
    }

    /// `std::__constrain_hash`
    fn constrain(h: u64, bc: usize) -> usize {
        let bc64 = bc as u64;
        if bc64 & (bc64 - 1) == 0 {
            (h & (bc64 - 1)) as usize
        } else if h < bc64 {
            h as usize
        } else {
            (h % bc64) as usize
        }
    }

    fn next_of(&self, pred: Option<usize>) -> Option<usize> {
        match pred {
            None => self.head,
            Some(i) => self.nodes[i].3,
        }
    }

    fn set_next(&mut self, pred: Option<usize>, next: Option<usize>) {
        match pred {
            None => self.head = next,
            Some(i) => self.nodes[i].3 = next,
        }
    }

    /// `std::__next_prime`: the smallest prime not below `n`.
    fn next_prime(mut n: usize) -> usize {
        fn is_prime(n: usize) -> bool {
            if n < 2 {
                return false;
            }
            let mut d = 2usize;
            while d * d <= n {
                if n % d == 0 {
                    return false;
                }
                d += 1;
            }
            true
        }
        while !is_prime(n) {
            n += 1;
        }
        n
    }

    /// `__hash_table::__rehash<true>` on the growing path, with `__do_rehash`.
    fn rehash(&mut self, mut n: usize) {
        if n == 1 {
            n = 2;
        } else if n & (n - 1) != 0 {
            n = Self::next_prime(n);
        }
        if n <= self.buckets.len() {
            return;
        }
        self.buckets = vec![None; n];
        let mut pp: Option<usize> = None; // before-begin
        let Some(first) = self.head else { return };
        let mut phash = Self::constrain(self.nodes[first].0, n);
        self.buckets[phash] = Some(pp);
        pp = Some(first);
        let mut cp = self.nodes[first].3;
        while let Some(c) = cp {
            let chash = Self::constrain(self.nodes[c].0, n);
            if chash == phash {
                pp = Some(c);
            } else if self.buckets[chash].is_none() {
                self.buckets[chash] = Some(pp);
                pp = Some(c);
                phash = chash;
            } else {
                // Unlink `c` and put it behind its bucket's predecessor.
                let after = self.nodes[c].3;
                self.set_next(pp, after);
                let bucket_pred = self.buckets[chash].unwrap();
                let bucket_first = self.next_of(bucket_pred);
                self.nodes[c].3 = bucket_first;
                self.set_next(bucket_pred, Some(c));
            }
            cp = self.next_of(pp);
        }
    }

    /// `emplace`: keeps the value already stored under an equal key.
    fn emplace(&mut self, key: f64, value: PointF) {
        let hash = Self::hash(key);
        let mut bc = self.buckets.len();
        let mut chash = 0usize;
        if bc != 0 {
            chash = Self::constrain(hash, bc);
            if let Some(pred) = self.buckets[chash] {
                let mut nd = self.next_of(pred);
                while let Some(i) = nd {
                    let (h, k, _, next) = self.nodes[i];
                    if !(h == hash || Self::constrain(h, bc) == chash) {
                        break;
                    }
                    if h == hash && k == key {
                        return;
                    }
                    nd = next;
                }
            }
        }
        let size = self.nodes.len();
        if (size + 1) as f32 > bc as f32 * 1.0f32 || bc == 0 {
            let is_power2 = bc > 2 && (bc & (bc - 1)) == 0;
            let want = (2 * bc + usize::from(!is_power2)).max(((size + 1) as f32 / 1.0f32).ceil() as usize);
            self.rehash(want);
            bc = self.buckets.len();
            chash = Self::constrain(hash, bc);
        }
        let new_idx = self.nodes.len();
        self.nodes.push((hash, key, value, None));
        match self.buckets[chash] {
            None => {
                // Empty bucket: to the front of the list.
                self.nodes[new_idx].3 = self.head;
                self.head = Some(new_idx);
                self.buckets[chash] = Some(None);
                if let Some(next) = self.nodes[new_idx].3 {
                    let next_bucket = Self::constrain(self.nodes[next].0, bc);
                    self.buckets[next_bucket] = Some(Some(new_idx));
                }
            }
            Some(pred) => {
                let after = self.next_of(pred);
                self.nodes[new_idx].3 = after;
                self.set_next(pred, Some(new_idx));
            }
        }
    }

    fn values_in_order(&self) -> Vec<PointF> {
        let mut out = Vec::with_capacity(self.nodes.len());
        let mut cur = self.head;
        while let Some(i) = cur {
            out.push(self.nodes[i].2);
            cur = self.nodes[i].3;
        }
        out
    }
}

// BridgeDetector.hpp:79-87, 129-137: the overhang anchored from all sides.
// consider this area anchored from all sides, pick bridging direction that will likely yield shortest bridges
fn bridging_direction_of_anchored_area(overhang_area: &Polygons) -> (PointF, CoordF) {
    // auto [pc1, pc2] = compute_principal_components(overhang_area);
    let (_pc1, pc2) = crate::principal_components2_d::compute_principal_components(overhang_area);
    // The components are Vec2f.
    let (x, y) = (pc2.x as f32, pc2.y as f32);
    if x == 0.0 && y == 0.0 {
        // overhang may be smaller than resolution. In this case, any direction is ok
        (PointF::new(1.0, 0.0), 0.0)
    } else {
        // pc2.normalized().cast<double>(): Eigen divides by the norm when the squared norm is positive
        let z = x * x + y * y;
        if z > 0.0 {
            let n = z.sqrt();
            (PointF::new((x / n) as f64, (y / n) as f64), 0.0)
        } else {
            (PointF::new(x as f64, y as f64), 0.0)
        }
    }
}

// BridgeDetector.hpp:89-120, 139-171: the overhang with floating edges.
// Overhang is not fully surrounded by anchors, in that case, find such direction that will minimize the number of bridge ends/180turns in the air
fn bridging_direction_of_floating_edges(floating_edges: &[Line]) -> (PointF, CoordF) {
    let mut directions = LibcxxUnorderedMapF64::new();
    for l in floating_edges.iter() {
        // Vec2d normal = l.normal().cast<double>().normalized(); Line.hpp:180 `Vector((b(1) - a(1)), -(b(0) - a(0)))`
        let nx = (l.b.y - l.a.y) as f64;
        let ny = (-(l.b.x - l.a.x)) as f64;
        let z = nx * nx + ny * ny;
        let normal = if z > 0.0 {
            let n = z.sqrt();
            PointF::new(nx / n, ny / n)
        } else {
            PointF::new(nx, ny)
        };
        let quantized_angle = (normal.y.atan2(normal.x) * 1000.0).ceil();
        directions.emplace(quantized_angle, normal);
    }
    let mut direction_costs: Vec<(PointF, CoordF)> = Vec::new();
    // it is acutally cost of a perpendicular bridge direction - we find the minimal cost and then return the perpendicular dir
    for d in directions.values_in_order() {
        direction_costs.push((d, 0.0));
    }

    for l in floating_edges.iter() {
        // Vec2d line = (l.b - l.a).cast<double>();
        let line = PointF::new((l.b.x - l.a.x) as f64, (l.b.y - l.a.y) as f64);
        for dir_cost in direction_costs.iter_mut() {
            // the dot product already contains the length of the line. dir_cost.first is normalized.
            dir_cost.1 += (line.x * dir_cost.0.x + line.y * dir_cost.0.y).abs();
        }
    }

    let mut result_dir = PointF::new(1.0, 1.0); // Vec2d::Ones()
    let mut min_cost = f64::MAX;
    for cost in direction_costs.iter() {
        if cost.1 < min_cost {
            // now flip the orientation back and return the direction of the bridge extrusions
            result_dir = PointF::new(cost.0.y, -cost.0.x);
            min_cost = cost.1;
        }
    }

    (result_dir, min_cost)
}

// BridgeDetector.hpp:76-121
// return ideal bridge direction and unsupported bridge endpoints distance.
// C++: `inline std::tuple<Vec2d, double> detect_bridging_direction(const Lines &floating_edges, const Polygons &overhang_area)`
pub fn detect_bridging_direction(floating_edges: &Lines, overhang_area: &Polygons) -> (PointF, CoordF) {
    if floating_edges.is_empty() {
        return bridging_direction_of_anchored_area(overhang_area);
    }
    bridging_direction_of_floating_edges(floating_edges)
}

// BridgeDetector.hpp:123-172
// return ideal bridge direction and unsupported bridge endpoints distance.
// C++: `inline std::tuple<Vec2d, double> detect_bridging_direction(const Polygons &to_cover, const Polygons &anchors_area)`
pub fn detect_bridging_direction_areas(to_cover: &Polygons, anchors_area: &Polygons) -> (PointF, CoordF) {
    let overhang_area: Polygons = cu::diff(to_cover, anchors_area, ApplySafetyOffset::No);
    // expand(anchors_area, float(SCALED_EPSILON))
    let floating_polylines: Polylines = cu::diff_pl(
        &cu_polygon::to_polylines(&overhang_area),
        &cu::expand(anchors_area, crate::libslic3r::SCALED_EPSILON as f32, cu::DEFAULT_JOIN_TYPE, cu::DEFAULT_MITER_LIMIT),
    );

    if floating_polylines.is_empty() {
        return bridging_direction_of_anchored_area(&overhang_area);
    }

    let floating_edges: Lines = to_lines_polylines(&floating_polylines);
    bridging_direction_of_floating_edges(&floating_edges)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Polygon;

    fn make_rect(x0: Coord, y0: Coord, x1: Coord, y1: Coord) -> ExPolygon {
        ExPolygon::new(Polygon::from_points(vec![Point::new(x0, y0), Point::new(x1, y0), Point::new(x1, y1), Point::new(x0, y1)]))
    }

    #[test]
    fn test_no_lower_slices_no_angle() {
        // Bridge with no lower slices: no edges, no anchors -> detect_angle false.
        let bridge = make_rect(0, 0, crate::scale(10.0), crate::scale(10.0));
        let lower: ExPolygons = Vec::new();
        let mut det = BridgeDetector::new(bridge, &lower, crate::scale(0.5));
        assert!(!det.detect_angle(0.0));
    }

    #[test]
    fn test_resolution_initialized() {
        let bridge = make_rect(0, 0, crate::scale(5.0), crate::scale(5.0));
        let lower: ExPolygons = Vec::new();
        let det = BridgeDetector::new(bridge, &lower, crate::scale(0.4));
        assert!((det.resolution - PI / 36.0).abs() < 1e-10);
        assert!(det.angle < 0.0);
    }

    #[test]
    fn test_anchored_bridge_detects_angle() {
        // A horizontal slot bridged between two supports on the left/right.
        let s = |v: f64| crate::scale(v);
        let bridge = make_rect(0, 0, s(20.0), s(4.0));
        // Two lower slices overlapping the bridge ends.
        let left = make_rect(s(-2.0), s(-2.0), s(2.0), s(6.0));
        let right = make_rect(s(18.0), s(-2.0), s(22.0), s(6.0));
        let lower = vec![left, right];
        let mut det = BridgeDetector::new(bridge, &lower, s(0.5));
        // Should find anchors and a direction.
        assert!(det.detect_angle(0.0));
        assert!(det.angle >= 0.0 && det.angle < PI);
    }

    #[test]
    fn test_detect_bridging_direction_empty() {
        let edges: Lines = Vec::new();
        let polygons: Polygons = Vec::new();
        let (dir, dist) = detect_bridging_direction(&edges, &polygons);
        // Default direction when no edges and no overhang -> (1, 0), 0.
        assert!((dir.x - 1.0).abs() < 1e-6);
        assert!(dir.y.abs() < 1e-6);
        assert!(dist.abs() < 1e-6);
    }
}
