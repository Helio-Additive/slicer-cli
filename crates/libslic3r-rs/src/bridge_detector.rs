//! Bridge detection: optimizes the direction of bridges over a region.
//!
//! 1:1 line-by-line port of:
//! - BridgeDetector.hpp
//! - BridgeDetector.cpp
//!
//! The bridge detector optimizes a direction of bridges over a region or a set of regions.
//! A bridge direction is considered optimal, if the length of the lines strang over the region
//! is maximal. This is optimal if the bridge is supported in a single direction only, but it may
//! not likely be optimal, if the bridge region is supported from all sides. Then an optimal
//! solution would find a direction with shortest bridges.
//! The bridge orientation is measured CCW from the X axis.

// BridgeDetector.cpp:1-4
use crate::clipper_utils::{self, diff_pl, intersection_pl};
use crate::geometry::{
    directions_parallel, expolygons_contain, to_lines, BoundingBox, ExPolygon, ExPolygons, Line,
    Lines, Point, PointF, Polygon, Polygons, Polyline,
};
use crate::libslic3r::EPSILON;
use crate::{Coord, CoordF};
use std::f64::consts::PI;

// ClipperSafetyOffset = 10.f scaled units (ClipperUtils.hpp:29). Unscaled = 10 / SCALING_FACTOR.
const CLIPPER_SAFETY_OFFSET: CoordF = 10.0 / crate::SCALING_FACTOR;
// SCALED_EPSILON used by detect_bridging_direction(Polygons, Polygons). libslic3r.h.
const SCALED_EPSILON: CoordF = crate::libslic3r::SCALED_EPSILON;

type Polylines = Vec<Polyline>;

// BridgeDetector.hpp:22-72
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

    // BridgeDetector.hpp:69 Open lines representing the supporting edges.
    _edges: Polylines,
    // BridgeDetector.hpp:71 Closed polygons representing the supporting areas.
    _anchor_regions: ExPolygons,
}

// BridgeDetector.hpp:52-63
struct BridgeDirection {
    // BridgeDetector.hpp:59
    angle: CoordF,
    // BridgeDetector.hpp:60
    coverage: CoordF,
    // BridgeDetector.hpp:61
    max_length: CoordF,
    // BridgeDetector.hpp:62
    archored_percent: CoordF,
}

impl BridgeDirection {
    // BridgeDetector.hpp:53
    fn new(a: CoordF) -> Self {
        Self {
            angle: a,
            coverage: 0.,
            max_length: 0.,
            archored_percent: 0.,
        }
    }

    // BridgeDetector.hpp:55-58 operator<
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
        // BridgeDetector.cpp:37-38 5 degrees stepping
        self.resolution = PI / 36.0;
        // BridgeDetector.cpp:39-40 output angle not known
        self.angle = -1.;

        // BridgeDetector.cpp:42-43
        // Outset our bridge by an arbitrary amout; we'll use this outer margin for detecting anchors.
        let grown: Polygons = offset(&self.expolygons, self.spacing as f32);

        // BridgeDetector.cpp:45-48
        // Detect possible anchoring edges of this bridging region.
        // Detect what edges lie on lower slices by turning bridge contour and holes
        // into polylines and then clipping them with each lower slice's contour.
        // Currently _edges are only used to set a candidate direction of the bridge (see bridge_direction_candidates()).
        // BridgeDetector.cpp:49-52
        let mut contours: Polygons = Vec::new();
        contours.reserve(self.lower_slices.len());
        for expoly in self.lower_slices.iter() {
            contours.push(expoly.contour.clone());
        }
        // BridgeDetector.cpp:53
        self._edges = intersection_pl_polygons(&to_polylines_polygons(&grown), &contours);

        // BridgeDetector.cpp:55-57 (SLIC3R_DEBUG)

        // BridgeDetector.cpp:59-61
        // detect anchors as intersection between our bridge expolygon and the lower slices
        // safety offset required to avoid Clipper from detecting empty intersection while Boost actually found some edges
        self._anchor_regions =
            intersection_ex(&grown, &union_safety_offset(&self.lower_slices));
    }

    // BridgeDetector.cpp:75-177
    pub fn detect_angle(&mut self, bridge_direction_override: CoordF) -> bool {
        // BridgeDetector.cpp:77-79
        if self._edges.is_empty() || self._anchor_regions.is_empty() {
            // The bridging region is completely in the air, there are no anchors available at the layer below.
            return false;
        }

        // BridgeDetector.cpp:81
        let mut candidates: Vec<BridgeDirection> = Vec::new();
        // BridgeDetector.cpp:82-88
        if bridge_direction_override == 0. {
            // BridgeDetector.cpp:83
            let angles = self.bridge_direction_candidates();
            // BridgeDetector.cpp:84
            candidates.reserve(angles.len());
            // BridgeDetector.cpp:85-86
            for i in 0..angles.len() {
                candidates.push(BridgeDirection::new(angles[i]));
            }
        } else {
            // BridgeDetector.cpp:88
            candidates.push(BridgeDirection::new(bridge_direction_override));
        }

        // BridgeDetector.cpp:90-93
        // Outset the bridge expolygon by half the amount we used for detecting anchors;
        // we'll use this one to clip our test lines and be sure that their endpoints
        // are inside the anchors and not on their contours leading to false negatives.
        let clip_area: Polygons = offset(&self.expolygons, 0.5 * self.spacing as f32);

        // BridgeDetector.cpp:95-98
        // we'll now try several directions using a rudimentary visibility check:
        // bridge in several directions and then sum the length of lines having both
        // endpoints within anchors

        // BridgeDetector.cpp:99
        let mut have_coverage = false;
        // BridgeDetector.cpp:100
        for i_angle in 0..candidates.len() {
            // BridgeDetector.cpp:102
            let angle = candidates[i_angle].angle;

            // BridgeDetector.cpp:104
            let mut lines: Lines = Vec::new();
            {
                // BridgeDetector.cpp:106-107
                // Get an oriented bounding box around _anchor_regions.
                let bbox = get_extents_rotated(&self._anchor_regions, -angle);
                // BridgeDetector.cpp:108-109
                // Cover the region with line segments.
                lines.reserve(((bbox.max.y - bbox.min.y + self.spacing) / self.spacing) as usize);
                // BridgeDetector.cpp:110
                let s = angle.sin();
                // BridgeDetector.cpp:111
                let c = angle.cos();
                // BridgeDetector.cpp:112-114 (FIXME comment)
                // for (coord_t y = bbox.min(1) + this->spacing / 2; ...
                // BridgeDetector.cpp:115-118
                let mut y = bbox.min.y;
                while y <= bbox.max.y {
                    // FIDELITY-NOTE(F2): C++ casts `round(...)` to `coord_t` (== int32_t,
                    // libslic3r.h:40); reproduce the int32 truncation via `as i32 as Coord`.
                    lines.push(Line::new(
                        Point::new(
                            (c * bbox.min.x as f64 - s * y as f64).round() as i32 as Coord,
                            (c * y as f64 + s * bbox.min.x as f64).round() as i32 as Coord,
                        ),
                        Point::new(
                            (c * bbox.max.x as f64 - s * y as f64).round() as i32 as Coord,
                            (c * y as f64 + s * bbox.max.x as f64).round() as i32 as Coord,
                        ),
                    ));
                    y += self.spacing;
                }
            }

            // BridgeDetector.cpp:121
            let mut total_length: CoordF = 0.;
            // BridgeDetector.cpp:122
            let mut max_length: CoordF = 0.;
            {
                // BridgeDetector.cpp:124
                let clipped_lines = intersection_ln(&lines, &clip_area);
                // BridgeDetector.cpp:125
                let mut archored_line_num: usize = 0;
                // BridgeDetector.cpp:126-135
                for i in 0..clipped_lines.len() {
                    // BridgeDetector.cpp:127
                    let line = &clipped_lines[i];
                    // BridgeDetector.cpp:128
                    if expolygons_contain(&self._anchor_regions, line.a)
                        && expolygons_contain(&self._anchor_regions, line.b)
                    {
                        // This line could be anchored.
                        // BridgeDetector.cpp:130
                        let len = line.length();
                        // BridgeDetector.cpp:131
                        total_length += len;
                        // BridgeDetector.cpp:132
                        max_length = max_length.max(len);
                        // BridgeDetector.cpp:133
                        archored_line_num += 1;
                    }
                }
                // BridgeDetector.cpp:136-138
                if !clipped_lines.is_empty() && archored_line_num > 0 {
                    candidates[i_angle].archored_percent =
                        archored_line_num as f64 / clipped_lines.len() as f64;
                }
            }
            // BridgeDetector.cpp:140-141
            if total_length == 0. {
                continue;
            }

            // BridgeDetector.cpp:143
            have_coverage = true;
            // BridgeDetector.cpp:144-145 Sum length of bridged lines.
            candidates[i_angle].coverage = total_length;
            // BridgeDetector.cpp:146-149 (commented-out alternative)
            // BridgeDetector.cpp:150 max length of bridged lines
            candidates[i_angle].max_length = max_length;
        }

        // BridgeDetector.cpp:153-155
        // if no direction produced coverage, then there's no bridge direction
        if !have_coverage {
            return false;
        }

        // BridgeDetector.cpp:157-158
        // sort directions by coverage - most coverage first
        candidates.sort_by(|a, b| {
            if a.less(b) {
                std::cmp::Ordering::Less
            } else if b.less(a) {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Equal
            }
        });

        // BridgeDetector.cpp:160-166
        // if any other direction is within extrusion width of coverage, prefer it if shorter
        // TODO: There are two options here - within width of the angle with most coverage, or within width of the currently perferred?
        let mut i_best: usize = 0;
        // for (size_t i = 1; i < candidates.size() && abs(candidates[i_best].archored_percent - candidates[i].archored_percent) < EPSILON; ++ i)
        let mut i = 1;
        while i < candidates.len()
            && candidates[i_best].coverage - candidates[i].coverage < self.spacing as f64
        {
            // BridgeDetector.cpp:165-166
            if candidates[i].max_length < candidates[i_best].max_length {
                i_best = i;
            }
            i += 1;
        }

        // BRIDGEPROBE (R595): R594 found the bridge fill DIRECTION differs per
        // layer (L47 rust 45 vs cpp 135). Dump every candidate so the divergence
        // point is visible: same candidate set and coverages => the difference is
        // tie-breaking (C++ std::sort is unstable, sort_by is stable); different
        // coverages => the cause is upstream in _anchor_regions / expolygons.
        if crate::probe_enabled("BRIDGEPROBE") {
            use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
            static CALL: AtomicUsize = AtomicUsize::new(0);
            let call = CALL.fetch_add(1, Relaxed) + 1;
            if call <= 40 {
                eprintln!(
                    "[BRIDGEPROBE] call={call} ncand={} spacing={} i_best={i_best}",
                    candidates.len(),
                    self.spacing
                );
                for (bi, cd) in candidates.iter().enumerate().take(12) {
                    eprintln!(
                        "[BRIDGECAND] call={call} i={bi} angle={:.6} coverage={:.3} max_length={:.3} anchored={:.6}",
                        cd.angle, cd.coverage, cd.max_length, cd.archored_percent
                    );
                }
            }
        }

        // BridgeDetector.cpp:168
        self.angle = candidates[i_best].angle;
        // BridgeDetector.cpp:169-170
        if self.angle >= PI {
            self.angle -= PI;
        }

        // BridgeDetector.cpp:172-174 (SLIC3R_DEBUG)

        // BridgeDetector.cpp:176
        true
    }

    // BridgeDetector.cpp:179-214
    fn bridge_direction_candidates(&self) -> Vec<CoordF> {
        // BridgeDetector.cpp:181-182
        // we test angles according to configured resolution
        let mut angles: Vec<CoordF> = Vec::new();
        // BridgeDetector.cpp:183-184
        // for (int i = 0; i <= PI/this->resolution; ++i)
        let mut i: i32 = 0;
        while (i as f64) <= PI / self.resolution {
            angles.push(i as f64 * self.resolution);
            i += 1;
        }

        // BridgeDetector.cpp:186-191
        // we also test angles of each bridge contour
        {
            let lines = to_lines(&self.expolygons);
            for line in lines.iter() {
                angles.push(line_direction(line));
            }
        }

        // BridgeDetector.cpp:193-197
        // we also test angles of each open supporting edge
        // (this finds the optimal angle for C-shaped supports)
        for edge in self._edges.iter() {
            if edge.first_point() != edge.last_point() {
                angles.push(line_direction(&Line::new(
                    edge.first_point(),
                    edge.last_point(),
                )));
            }
        }

        // BridgeDetector.cpp:199-207
        // remove duplicates
        let min_resolution = PI / 180.0; // 1 degree
        angles.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        // C++: for (size_t i = 1; i < angles.size(); ++i) { if (parallel) { erase(i); --i; } }
        // The `--i` followed by the loop `++i` leaves i unchanged after an erase.
        let mut i = 1usize;
        while i < angles.len() {
            if directions_parallel(angles[i], angles[i - 1], min_resolution) {
                angles.remove(i);
            } else {
                i += 1;
            }
        }

        // BridgeDetector.cpp:208-211
        // compare first value with last one and remove the greatest one (PI)
        // in case they are parallel (PI, 0)
        if directions_parallel(angles[0], *angles.last().unwrap(), min_resolution) {
            angles.pop();
        }

        // BridgeDetector.cpp:213
        angles
    }

    // BridgeDetector.cpp:276-333
    // Coverage is currently only used by the unit tests. It is extremely slow and unreliable!
    pub fn coverage(&self, angle: CoordF) -> Polygons {
        // BridgeDetector.cpp:278-279
        let mut angle = angle;
        if angle == -1. {
            angle = self.angle;
        }

        // BridgeDetector.cpp:281
        let mut covered: Polygons = Vec::new();

        // BridgeDetector.cpp:283
        if angle != -1. {
            // BridgeDetector.cpp:284-286
            // Get anchors, convert them to Polygons and rotate them.
            let mut anchors: Polygons = to_polygons(&self._anchor_regions);
            polygons_rotate(&mut anchors, PI / 2.0 - angle);

            // BridgeDetector.cpp:288
            for expolygon in self.expolygons.iter() {
                let mut expolygon = expolygon.clone();
                // BridgeDetector.cpp:289-290
                // Clone our expolygon and rotate it so that we work with vertical lines.
                expolygon.rotate(PI / 2.0 - angle);
                // BridgeDetector.cpp:291-294
                // Outset the bridge expolygon by half the amount we used for detecting anchors;
                // we'll use this one to generate our trapezoids and be sure that their vertices
                // are inside the anchors and not on their contours leading to false negatives.
                for expoly in offset_ex(&expolygon, 0.5 * self.spacing as f32).iter() {
                    // BridgeDetector.cpp:295-297
                    // Compute trapezoids according to a vertical orientation
                    let mut trapezoids: Polygons = Vec::new();
                    get_trapezoids2_angle(expoly, &mut trapezoids, PI / 2.0);
                    // BridgeDetector.cpp:298-306
                    for trapezoid in trapezoids.iter() {
                        // not nice, we need a more robust non-numeric check
                        // BridgeDetector.cpp:300
                        let mut n_supported: usize = 0;
                        // BridgeDetector.cpp:301-303
                        for supported_line in intersection_ln(&trapezoid.edges(), &anchors).iter() {
                            if supported_line.length() >= self.spacing as f64 {
                                n_supported += 1;
                            }
                        }
                        // BridgeDetector.cpp:304-305
                        if n_supported >= 2 {
                            covered.push(trapezoid.clone());
                        }
                    }
                }
            }

            // BridgeDetector.cpp:310-312
            // Unite the trapezoids before rotation, as the rotation creates tiny gaps and intersections between the trapezoids
            // instead of exact overlaps.
            covered = union_(&covered);
            // BridgeDetector.cpp:313-315
            // Intersect trapezoids with actual bridge area to remove extra margins and append it to result.
            polygons_rotate(&mut covered, -(PI / 2.0 - angle));
            covered = intersection_polygons_expolygons(&self.expolygons, &covered);
            // BridgeDetector.cpp:316-330 (#if 0 SVG dump)
        }
        // BridgeDetector.cpp:332
        covered
    }

    // BridgeDetector.cpp:335-377
    // This method returns the bridge edges (as polylines) that are not supported
    // but would allow the entire bridge area to be bridged with detected angle
    // if supported too
    pub fn unsupported_edges_into(&self, angle: CoordF, unsupported: &mut Polylines) {
        // BridgeDetector.cpp:341
        let mut angle = angle;
        if angle == -1. {
            angle = self.angle;
        }
        // BridgeDetector.cpp:342
        if angle == -1. {
            return;
        }

        // BridgeDetector.cpp:344
        let grown_lower: Polygons = offset(&self.lower_slices, self.spacing as f32);

        // BridgeDetector.cpp:346
        for it_expoly in self.expolygons.iter() {
            // BridgeDetector.cpp:347-348
            // get unsupported bridge edges (both contour and holes)
            let unsupported_lines =
                to_lines_polylines(&diff_pl_polygons(&it_expoly.to_polylines(), &grown_lower));
            // BridgeDetector.cpp:349-354 (TODO comment)
            // Split into individual segments and filter out edges parallel to the bridging angle
            // BridgeDetector.cpp:355-360
            for line in unsupported_lines.iter() {
                if !directions_parallel(line_direction(line), angle, 0.) {
                    unsupported.push(Polyline::from_points(vec![line.a, line.b]));
                }
            }
        }

        // BridgeDetector.cpp:363-376 (commented-out SVG dump)
    }

    // BridgeDetector.cpp:379-385
    pub fn unsupported_edges(&self, angle: CoordF) -> Polylines {
        // BridgeDetector.cpp:382
        let mut pp: Polylines = Vec::new();
        // BridgeDetector.cpp:383
        self.unsupported_edges_into(angle, &mut pp);
        // BridgeDetector.cpp:384
        pp
    }

    // Accessors mirroring the C++ private members for callers / tests.
    pub fn anchor_regions(&self) -> &ExPolygons {
        &self._anchor_regions
    }

    pub fn edges(&self) -> &Polylines {
        &self._edges
    }
}

// BridgeDetector.cpp:236-264
// This algorithm may return more trapezoids than necessary
// (i.e. it may break a single trapezoid in several because
// other parts of the object have x coordinates in the middle)
fn get_trapezoids2(expoly: &ExPolygon, polygons: &mut Polygons) {
    // BridgeDetector.cpp:238
    let src_polygons: Polygons = expoly.to_polygons();
    // BridgeDetector.cpp:239-240
    // get all points of this ExPolygon
    let pp: Vec<Point> = to_points_polygons(&src_polygons);

    // BridgeDetector.cpp:242-243
    // build our bounding box
    let bb = BoundingBox::from_points(&pp);

    // BridgeDetector.cpp:245-250
    // get all x coordinates
    let mut xx: Vec<Coord> = Vec::new();
    xx.reserve(pp.len());
    for p in pp.iter() {
        xx.push(p.x);
    }
    xx.sort();

    // BridgeDetector.cpp:252-263
    // find trapezoids by looping from first to next-to-last coordinate
    let mut rectangle: Polygons = Vec::new();
    rectangle.push(Polygon::new());
    // for (std::vector<coord_t>::const_iterator x = xx.begin(); x != xx.end()-1; ++x)
    if !xx.is_empty() {
        for idx in 0..xx.len() - 1 {
            let x = xx[idx];
            // BridgeDetector.cpp:256
            let next_x = xx[idx + 1];
            // BridgeDetector.cpp:257
            if x != next_x {
                // BridgeDetector.cpp:258-261
                // intersect with rectangle
                // append results to return value
                rectangle[0] = Polygon::from_points(vec![
                    Point::new(x, bb.min.y),
                    Point::new(next_x, bb.min.y),
                    Point::new(next_x, bb.max.y),
                    Point::new(x, bb.max.y),
                ]);
                polygons_append(
                    polygons,
                    &intersection_polygons_polygons(&rectangle, &src_polygons),
                );
            }
        }
    }
}

// BridgeDetector.cpp:266-273
fn get_trapezoids2_angle(expoly: &ExPolygon, polygons: &mut Polygons, angle: CoordF) {
    // BridgeDetector.cpp:268
    let mut clone = expoly.clone();
    // BridgeDetector.cpp:269
    clone.rotate(PI / 2.0 - angle);
    // BridgeDetector.cpp:270
    get_trapezoids2(&clone, polygons);
    // BridgeDetector.cpp:271-272
    for polygon in polygons.iter_mut() {
        polygon.rotate(-(PI / 2.0 - angle));
    }
}

// Faithful port of `Line::direction()` (Line.cpp:60-66).
fn line_direction(line: &Line) -> CoordF {
    // Line.hpp `atan2_() { return atan2(b(1) - a(1), b(0) - a(0)); }`
    let atan2 = ((line.b.y - line.a.y) as f64).atan2((line.b.x - line.a.x) as f64);
    // Line.cpp:63-65
    if (atan2 - PI).abs() < EPSILON {
        0.
    } else if atan2 < 0. {
        atan2 + PI
    } else {
        atan2
    }
}

// Faithful port of `get_extents_rotated(const ExPolygons&, double)` (ExPolygon.cpp:511-519)
// which considers only each ExPolygon's contour, and `get_extents_rotated(const Points&, double)`
// (MultiPoint.cpp:441-461) for the rotation/rounding.
fn get_extents_rotated(expolygons: &ExPolygons, angle: CoordF) -> BoundingBox {
    // MultiPoint.cpp:443
    let mut bbox = BoundingBox::new();
    // ExPolygon.cpp:514-517
    let s = angle.sin();
    let c = angle.cos();
    for expoly in expolygons.iter() {
        // ExPolygon.cpp:508 uses expolygon.contour only.
        for point in expoly.contour.points() {
            // MultiPoint.cpp:446-459
            let cur_x = point.x as f64;
            let cur_y = point.y as f64;
            // FIDELITY-NOTE(F2): C++ casts `round(...)` to `coord_t` (== int32_t,
            // libslic3r.h:40); reproduce the int32 truncation via `as i32 as Coord`.
            let x = (c * cur_x - s * cur_y).round() as i32 as Coord;
            let y = (c * cur_y + s * cur_x).round() as i32 as Coord;
            bbox.merge_point(Point::new(x, y));
        }
    }
    bbox
}

// ============================================================================
// Clipper helpers bridging the C++ Polygons-based overloads onto the ExPolygon-centric
// crate clipper backend, all numerically equivalent to ClipperUtils.{hpp,cpp}.
// FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib. These helpers
// (offset/offset_ex/union_/intersection*/diff*/expand) delegate to clipper_utils,
// which uses the `geo` crate (geo-clipper, fixed scale 1000) rather than ClipperLib
// at coord_t integer precision. Cross-cutting; not re-routed per-file.
// ============================================================================

// offset(const ExPolygons&, float) -> Polygons (ClipperUtils.hpp:350). Default jtMiter, ml 3.
fn offset(expolygons: &ExPolygons, delta: f32) -> Polygons {
    let ex = clipper_utils::offset_expolygons(
        expolygons,
        delta as CoordF / crate::SCALING_FACTOR,
        clipper_utils::OffsetJoinType::Miter,
    );
    expolygons_to_polygons(&ex)
}

// offset_ex(const ExPolygon&, float) -> ExPolygons (ClipperUtils.hpp:355). Default jtMiter, ml 3.
fn offset_ex(expolygon: &ExPolygon, delta: f32) -> ExPolygons {
    clipper_utils::offset_expolygons(
        std::slice::from_ref(expolygon),
        delta as CoordF / crate::SCALING_FACTOR,
        clipper_utils::OffsetJoinType::Miter,
    )
}

// union_safety_offset(const ExPolygons&) = offset(expolygons, ClipperSafetyOffset).
// The C++ overload returns Polygons; here we keep ExPolygons for the intersection_ex below.
fn union_safety_offset(expolygons: &ExPolygons) -> ExPolygons {
    clipper_utils::offset_expolygons(
        expolygons,
        CLIPPER_SAFETY_OFFSET,
        clipper_utils::OffsetJoinType::Miter,
    )
}

// intersection_ex(const Polygons&, const ExPolygons&) -> ExPolygons (ClipperUtils).
fn intersection_ex(subject: &Polygons, clip: &ExPolygons) -> ExPolygons {
    let subj_ex = polygons_to_expolygons(subject);
    clipper_utils::intersection(&subj_ex, clip)
}

// intersection(const Polygons&, const Polygons&) -> Polygons (ClipperUtils).
fn intersection_polygons_polygons(subject: &Polygons, clip: &Polygons) -> Polygons {
    let subj_ex = polygons_to_expolygons(subject);
    let clip_ex = polygons_to_expolygons(clip);
    expolygons_to_polygons(&clipper_utils::intersection(&subj_ex, &clip_ex))
}

// intersection(const ExPolygons&, const Polygons&) -> Polygons (ClipperUtils).
fn intersection_polygons_expolygons(subject: &ExPolygons, clip: &Polygons) -> Polygons {
    let clip_ex = polygons_to_expolygons(clip);
    expolygons_to_polygons(&clipper_utils::intersection(subject, &clip_ex))
}

// union_(const Polygons&) -> Polygons (ClipperUtils.cpp). NonZero fill.
fn union_(subject: &Polygons) -> Polygons {
    expolygons_to_polygons(&clipper_utils::union_polygons_ex(subject))
}

// intersection_pl(const Polylines&, const Polygons&) -> Polylines (ClipperUtils).
fn intersection_pl_polygons(subject: &Polylines, clip: &Polygons) -> Polylines {
    intersection_pl(subject, &polygons_to_expolygons(clip))
}

// diff_pl(const Polylines&, const Polygons&) -> Polylines (ClipperUtils).
fn diff_pl_polygons(subject: &Polylines, clip: &Polygons) -> Polylines {
    diff_pl(subject, &polygons_to_expolygons(clip))
}

// Faithful port of `intersection_ln(const Lines&, const Polygons&)` (ClipperUtils.hpp:536-539,
// ClipperUtils.cpp `_clipper_ln`). Converts each Line to a 2-point Polyline, clips against the
// polygons (holes honoured), then takes front/back of each surviving polyline as a Line.
fn intersection_ln(subject: &Lines, clip: &Polygons) -> Lines {
    // _clipper_ln: convert Lines to Polylines
    let mut polylines: Polylines = Vec::with_capacity(subject.len());
    for line in subject.iter() {
        polylines.push(Polyline::from_points(vec![line.a, line.b]));
    }
    // perform intersection
    let polylines = intersection_pl(&polylines, &polygons_to_expolygons(clip));
    // convert Polylines to Lines
    let mut retval: Lines = Vec::new();
    for polyline in polylines.iter() {
        if polyline.len() >= 2 {
            retval.push(Line::new(polyline.first_point(), polyline.last_point()));
        }
    }
    retval
}

// to_points(const Polygons&) -> Points (MultiPoint / Polygon helpers).
fn to_points_polygons(polygons: &Polygons) -> Vec<Point> {
    let mut pts: Vec<Point> = Vec::new();
    for polygon in polygons.iter() {
        for p in polygon.points() {
            pts.push(*p);
        }
    }
    pts
}

// to_lines(const Polylines&) -> Lines.
fn to_lines_polylines(polylines: &Polylines) -> Lines {
    let mut lines: Lines = Vec::new();
    for polyline in polylines.iter() {
        let pts = polyline.points();
        if pts.len() >= 2 {
            for i in 0..pts.len() - 1 {
                lines.push(Line::new(pts[i], pts[i + 1]));
            }
        }
    }
    lines
}

// to_polylines(const Polygons&) -> Polylines. Each polygon becomes a closed polyline
// (first point repeated at the end), matching to_polylines(ExPolygons) over contour+holes.
fn to_polylines_polygons(polygons: &Polygons) -> Polylines {
    let mut out: Polylines = Vec::new();
    for polygon in polygons.iter() {
        out.push(Polyline::from_polygon(polygon));
    }
    out
}

// polygons_append(Polygons&, Polygons) (clipper helper).
fn polygons_append(dst: &mut Polygons, src: &Polygons) {
    dst.extend(src.iter().cloned());
}

// polygons_rotate(Polygons&, double) (Polygon.hpp:165-171). Uses precomputed cos/sin.
fn polygons_rotate(polys: &mut Polygons, angle: CoordF) {
    for p in polys.iter_mut() {
        p.rotate(angle);
    }
}

// to_polygons(const ExPolygons&) -> Polygons.
fn expolygons_to_polygons(expolygons: &ExPolygons) -> Polygons {
    let mut out: Polygons = Vec::new();
    for expoly in expolygons.iter() {
        out.push(expoly.contour.clone());
        for hole in expoly.holes.iter() {
            out.push(hole.clone());
        }
    }
    out
}

// Wrap a Polygons set as ExPolygons. Used to feed the ExPolygon-centric clipper backend;
// the even-odd / nonzero fill rule reconciles holes, matching ClipperLib.
fn polygons_to_expolygons(polygons: &Polygons) -> ExPolygons {
    clipper_utils::union_polygons_ex(polygons)
}

// to_polygons(const ExPolygons&) used inside coverage(): wraps expolygons_to_polygons.
fn to_polygons(expolygons: &ExPolygons) -> Polygons {
    expolygons_to_polygons(expolygons)
}

// ============================================================================
// Header inline free functions.
// ============================================================================

// BridgeDetector.hpp:75-119
// return ideal bridge direction and unsupported bridge endpoints distance.
pub fn detect_bridging_direction(floating_edges: &Lines, overhang_area: &Polygons) -> (PointF, CoordF) {
    // BridgeDetector.hpp:77-85
    if floating_edges.is_empty() {
        // consider this area anchored from all sides, pick bridging direction that will likely yield shortest bridges
        let (_pc1, pc2) = crate::principal_components2_d::compute_principal_components(overhang_area);
        if pc2.x == 0.0 && pc2.y == 0.0 {
            // overhang may be smaller than resolution. In this case, any direction is ok
            return (PointF::new(1.0, 0.0), 0.0);
        } else {
            let n = (pc2.x * pc2.x + pc2.y * pc2.y).sqrt();
            return (PointF::new(pc2.x / n, pc2.y / n), 0.0);
        }
    }

    // BridgeDetector.hpp:87-93
    // Overhang is not fully surrounded by anchors, in that case, find such direction that will minimize the number of bridge ends/180turns in the air
    let mut directions: std::collections::HashMap<i64, PointF> = std::collections::HashMap::new();
    for l in floating_edges.iter() {
        // Vec2d normal = l.normal().cast<double>().normalized();
        let normal = normalized(line_normal(l));
        // double quantized_angle = std::ceil(std::atan2(normal.y(),normal.x()) * 1000.0);
        let quantized_angle = (normal.y.atan2(normal.x) * 1000.0).ceil() as i64;
        // directions.emplace(quantized_angle, normal); -- only inserts if key not present.
        directions.entry(quantized_angle).or_insert(normal);
    }
    // BridgeDetector.hpp:94-98
    let mut direction_costs: Vec<(PointF, CoordF)> = Vec::new();
    // it is acutally cost of a perpendicular bridge direction - we find the minimal cost and then return the perpendicular dir
    for (_k, d) in directions.iter() {
        direction_costs.push((*d, 0.0));
    }

    // BridgeDetector.hpp:100-106
    for l in floating_edges.iter() {
        // Vec2d line = (l.b - l.a).cast<double>();
        let line = PointF::new((l.b.x - l.a.x) as f64, (l.b.y - l.a.y) as f64);
        for dir_cost in direction_costs.iter_mut() {
            // the dot product already contains the length of the line. dir_cost.first is normalized.
            dir_cost.1 += (line.x * dir_cost.0.x + line.y * dir_cost.0.y).abs();
        }
    }

    // BridgeDetector.hpp:108-117
    let mut result_dir = PointF::new(1.0, 1.0); // Vec2d::Ones()
    let mut min_cost = f64::MAX;
    for cost in direction_costs.iter() {
        if cost.1 < min_cost {
            // now flip the orientation back and return the direction of the bridge extrusions
            result_dir = PointF::new(cost.0.y, -cost.0.x);
            min_cost = cost.1;
        }
    }

    // BridgeDetector.hpp:118
    (result_dir, min_cost)
}

// BridgeDetector.hpp:121-170
// return ideal bridge direction and unsupported bridge endpoints distance.
pub fn detect_bridging_direction_areas(
    to_cover: &Polygons,
    anchors_area: &Polygons,
) -> (PointF, CoordF) {
    // BridgeDetector.hpp:123
    let overhang_area: Polygons = diff_polygons_polygons(to_cover, anchors_area);
    // BridgeDetector.hpp:125
    let floating_polylines: Polylines = diff_pl_polygons(
        &to_polylines_polygons(&overhang_area),
        &expand(anchors_area, SCALED_EPSILON as f32),
    );

    // BridgeDetector.hpp:127-135
    if floating_polylines.is_empty() {
        // consider this area anchored from all sides, pick bridging direction that will likely yield shortest bridges
        let (_pc1, pc2) =
            crate::principal_components2_d::compute_principal_components(&overhang_area);
        if pc2.x == 0.0 && pc2.y == 0.0 {
            // overhang may be smaller than resolution. In this case, any direction is ok
            return (PointF::new(1.0, 0.0), 0.0);
        } else {
            let n = (pc2.x * pc2.x + pc2.y * pc2.y).sqrt();
            return (PointF::new(pc2.x / n, pc2.y / n), 0.0);
        }
    }

    // BridgeDetector.hpp:137-138
    // Overhang is not fully surrounded by anchors, in that case, find such direction that will minimize the number of bridge ends/180turns in the air
    let floating_edges: Lines = to_lines_polylines(&floating_polylines);
    // BridgeDetector.hpp:139-143
    let mut directions: std::collections::HashMap<i64, PointF> = std::collections::HashMap::new();
    for l in floating_edges.iter() {
        let normal = normalized(line_normal(l));
        let quantized_angle = (normal.y.atan2(normal.x) * 1000.0).ceil() as i64;
        directions.entry(quantized_angle).or_insert(normal);
    }
    // BridgeDetector.hpp:145-149
    let mut direction_costs: Vec<(PointF, CoordF)> = Vec::new();
    for (_k, d) in directions.iter() {
        direction_costs.push((*d, 0.0));
    }

    // BridgeDetector.hpp:151-157
    for l in floating_edges.iter() {
        let line = PointF::new((l.b.x - l.a.x) as f64, (l.b.y - l.a.y) as f64);
        for dir_cost in direction_costs.iter_mut() {
            dir_cost.1 += (line.x * dir_cost.0.x + line.y * dir_cost.0.y).abs();
        }
    }

    // BridgeDetector.hpp:159-167
    let mut result_dir = PointF::new(1.0, 1.0); // Vec2d::Ones()
    let mut min_cost = f64::MAX;
    for cost in direction_costs.iter() {
        if cost.1 < min_cost {
            result_dir = PointF::new(cost.0.y, -cost.0.x);
            min_cost = cost.1;
        }
    }

    // BridgeDetector.hpp:169
    (result_dir, min_cost)
}

// Faithful port of `Line::normal()` (Line.hpp:180):
//   Vector normal() const { return Vector((b(1) - a(1)), -(b(0) - a(0))); }
// i.e. normal = ( dy, -dx ). The crate's `Line::normal_f64()` uses the opposite
// convention (-dy, dx), which is the negated vector and would yield an atan2 off
// by +/- PI and a flipped result_dir; compute the C++ normal directly here.
fn line_normal(l: &Line) -> PointF {
    PointF::new(
        (l.b.y - l.a.y) as CoordF,
        -((l.b.x - l.a.x) as CoordF),
    )
}

// Vec2d::normalized() with the degenerate (zero) case mapped to zero (Eigen would give NaN;
// the C++ code only calls this on edge normals which are always non-zero for real edges).
fn normalized(v: PointF) -> PointF {
    let n = (v.x * v.x + v.y * v.y).sqrt();
    if n > 0.0 {
        PointF::new(v.x / n, v.y / n)
    } else {
        PointF::new(0.0, 0.0)
    }
}

// diff(const Polygons&, const Polygons&) -> Polygons (ClipperUtils).
fn diff_polygons_polygons(subject: &Polygons, clip: &Polygons) -> Polygons {
    let subj_ex = polygons_to_expolygons(subject);
    let clip_ex = polygons_to_expolygons(clip);
    expolygons_to_polygons(&clipper_utils::difference(&subj_ex, &clip_ex))
}

// expand(const Polygons&, float) = offset(polygons, delta) with delta > 0 (ClipperUtils.hpp:383).
fn expand(polygons: &Polygons, delta: f32) -> Polygons {
    let ex = clipper_utils::offset_polygons(
        polygons,
        delta as CoordF / crate::SCALING_FACTOR,
        clipper_utils::OffsetJoinType::Miter,
    );
    expolygons_to_polygons(&ex)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Polygon;

    fn make_rect(x0: Coord, y0: Coord, x1: Coord, y1: Coord) -> ExPolygon {
        ExPolygon::new(Polygon::from_points(vec![
            Point::new(x0, y0),
            Point::new(x1, y0),
            Point::new(x1, y1),
            Point::new(x0, y1),
        ]))
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
