//! Convex hull computation using Andrew's monotone chain algorithm
//!
//! C++ Reference:
//! - Geometry/ConvexHull.hpp (35 lines)
//! - Geometry/ConvexHull.cpp (287 lines)
//!
//! Implements 2D and 3D convex hull computation using Andrew's monotone chain
//! algorithm with O(n log n) time complexity.

use crate::geometry::{ExPolygon, Point, Polygon, Polyline, Vec2d, Vec3d};
use crate::CoordF;

/// Compute 2D convex hull of a set of points using Andrew's monotone chain algorithm
///
/// Geometry/ConvexHull.cpp:10-36
/// C++: Polygon convex_hull(Points pts)
pub fn convex_hull_points(mut pts: Vec<Point>) -> Polygon {
    // Sort points by x, then y
    // Geometry/ConvexHull.cpp:12
    // C++: std::sort(pts.begin(), pts.end(), [](const Point& a, const Point& b) {
    // C++:     return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    // C++: });
    pts.sort_by(|a, b| a.x().cmp(&b.x()).then_with(|| a.y().cmp(&b.y())));

    // Remove duplicates
    // Geometry/ConvexHull.cpp:13
    // C++: pts.erase(std::unique(pts.begin(), pts.end(), [](const Point& a, const Point& b) {
    // C++:     return a.x() == b.x() && a.y() == b.y();
    // C++: }), pts.end());
    pts.dedup_by(|a, b| a.x() == b.x() && a.y() == b.y());

    // Initialize hull
    // Geometry/ConvexHull.cpp:15-16
    // C++: Polygon hull;
    // C++: int n = (int)pts.size();
    let n = pts.len();

    if n < 3 {
        return Polygon::from_points(pts);
    }

    // Geometry/ConvexHull.cpp:17-35
    // C++: if (n >= 3) {
    // C++:     int k = 0;
    // C++:     hull.points.resize(2 * n);
    let mut hull: Vec<Point> = Vec::with_capacity(2 * n);
    let mut k = 0;

    // Build lower hull
    // Geometry/ConvexHull.cpp:19-24
    // C++: for (int i = 0; i < n; ++ i) {
    // C++:     while (k >= 2 && Geometry::orient(pts[i], hull[k-2], hull[k-1]) != Geometry::ORIENTATION_CCW)
    // C++:         -- k;
    // C++:     hull[k ++] = pts[i];
    // C++: }
    for i in 0..n {
        while k >= 2 && !is_ccw(&pts[i], &hull[k - 2], &hull[k - 1]) {
            hull.pop();
            k -= 1;
        }
        hull.push(pts[i]);
        k += 1;
    }

    // Build upper hull
    // Geometry/ConvexHull.cpp:25-30
    // C++: for (int i = n-2, t = k+1; i >= 0; i--) {
    // C++:     while (k >= t && Geometry::orient(pts[i], hull[k-2], hull[k-1]) != Geometry::ORIENTATION_CCW)
    // C++:         -- k;
    // C++:     hull[k ++] = pts[i];
    // C++: }
    let t = k + 1;
    for i in (0..n - 1).rev() {
        while k >= t && !is_ccw(&pts[i], &hull[k - 2], &hull[k - 1]) {
            hull.pop();
            k -= 1;
        }
        hull.push(pts[i]);
        k += 1;
    }

    // Remove duplicate last point
    // Geometry/ConvexHull.cpp:31-33
    // C++: hull.points.resize(k);
    // C++: assert(hull.points.front() == hull.points.back());
    // C++: hull.points.pop_back();
    hull.truncate(k);
    if !hull.is_empty() && hull.first() == hull.last() {
        hull.pop();
    }

    Polygon::from_points(hull)
}

/// Compute 3D convex hull projected onto XY plane
///
/// Geometry/ConvexHull.cpp:38-96
/// C++: Pointf3s convex_hull(Pointf3s points)
pub fn convex_hull_3d(mut points: Vec<Vec3d>) -> Vec<Vec3d> {
    // Sort points by x, then y
    // Geometry/ConvexHull.cpp:40
    // C++: std::sort(points.begin(), points.end(), [](const Vec3d &a, const Vec3d &b){
    // C++:     return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    // C++: });
    points.sort_by(|a, b| {
        a.x()
            .partial_cmp(&b.x())
            .unwrap_or(std::cmp::Ordering::Equal)
            .then_with(|| {
                a.y()
                    .partial_cmp(&b.y())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    });

    // Geometry/ConvexHull.cpp:42-43
    // C++: int n = points.size(), k = 0;
    // C++: Pointf3s hull;
    let n = points.len();
    let mut k = 0;
    let mut hull: Vec<Vec3d> = Vec::with_capacity(2 * n);

    if n < 3 {
        return points;
    }

    // Build lower hull
    // Geometry/ConvexHull.cpp:48-63
    // C++: for (int i = 0; i < n; ++i) {
    for i in 0..n {
        // C++: Point p = Point::new_scale(points[i](0), points[i](1));
        let p = Point::new_scale(points[i].x(), points[i].y());

        // C++: while (k >= 2) {
        while k >= 2 {
            // C++: Point k1 = Point::new_scale(hull[k - 1](0), hull[k - 1](1));
            // C++: Point k2 = Point::new_scale(hull[k - 2](0), hull[k - 2](1));
            let k1 = Point::new_scale(hull[k - 1].x(), hull[k - 1].y());
            let k2 = Point::new_scale(hull[k - 2].x(), hull[k - 2].y());

            // C++: if (Geometry::orient(p, k2, k1) != Geometry::ORIENTATION_CCW)
            // C++:     --k;
            // C++: else
            // C++:     break;
            if !is_ccw(&p, &k2, &k1) {
                hull.pop();
                k -= 1;
            } else {
                break;
            }
        }

        // C++: hull[k++] = points[i];
        hull.push(points[i]);
        k += 1;
    }

    // Build upper hull
    // Geometry/ConvexHull.cpp:65-84
    // C++: for (int i = n - 2, t = k + 1; i >= 0; --i) {
    let t = k + 1;
    for i in (0..n - 1).rev() {
        // C++: Point p = Point::new_scale(points[i](0), points[i](1));
        let p = Point::new_scale(points[i].x(), points[i].y());

        // C++: while (k >= t) {
        while k >= t {
            // C++: Point k1 = Point::new_scale(hull[k - 1](0), hull[k - 1](1));
            // C++: Point k2 = Point::new_scale(hull[k - 2](0), hull[k - 2](1));
            let k1 = Point::new_scale(hull[k - 1].x(), hull[k - 1].y());
            let k2 = Point::new_scale(hull[k - 2].x(), hull[k - 2].y());

            // C++: if (Geometry::orient(p, k2, k1) != Geometry::ORIENTATION_CCW)
            // C++:     --k;
            // C++: else
            // C++:     break;
            if !is_ccw(&p, &k2, &k1) {
                hull.pop();
                k -= 1;
            } else {
                break;
            }
        }

        // C++: hull[k++] = points[i];
        hull.push(points[i]);
        k += 1;
    }

    // Remove duplicate last point
    // Geometry/ConvexHull.cpp:87-92
    // C++: hull.resize(k);
    // C++: assert(hull.front() == hull.back());
    // C++: hull.pop_back();
    hull.truncate(k);
    if !hull.is_empty() && hull.first() == hull.last() {
        hull.pop();
    }

    hull
}

/// Compute convex hull of multiple polygons
///
/// Geometry/ConvexHull.cpp:98-104
/// C++: Polygon convex_hull(const Polygons &polygons)
pub fn convex_hull_polygons(polygons: &[Polygon]) -> Polygon {
    // Geometry/ConvexHull.cpp:99-102
    // C++: Points pp;
    // C++: for (Polygons::const_iterator p = polygons.begin(); p != polygons.end(); ++p) {
    // C++:     pp.insert(pp.end(), p->points.begin(), p->points.end());
    // C++: }
    let mut pp = Vec::new();
    for polygon in polygons {
        pp.extend_from_slice(polygon.points());
    }

    // Geometry/ConvexHull.cpp:103
    // C++: return convex_hull(std::move(pp));
    convex_hull_points(pp)
}

/// Compute convex hull of expolygons (contours only, not holes)
///
/// Geometry/ConvexHull.cpp:106-117
/// C++: Polygon convex_hull(const ExPolygons &expolygons)
pub fn convex_hull_expolygons(expolygons: &[ExPolygon]) -> Polygon {
    // Geometry/ConvexHull.cpp:108-113
    // C++: Points pp;
    // C++: size_t sz = 0;
    // C++: for (const auto &expoly : expolygons)
    // C++:     sz += expoly.contour.size();
    // C++: pp.reserve(sz);
    let sz: usize = expolygons.iter().map(|e| e.contour.points.len()).sum();
    let mut pp = Vec::with_capacity(sz);

    // Geometry/ConvexHull.cpp:114-115
    // C++: for (const auto &expoly : expolygons)
    // C++:     pp.insert(pp.end(), expoly.contour.points.begin(), expoly.contour.points.end());
    for expoly in expolygons {
        pp.extend_from_slice(&expoly.contour.points);
    }

    // Geometry/ConvexHull.cpp:116
    // C++: return convex_hull(pp);
    convex_hull_points(pp)
}

/// Compute convex hull of polylines
///
/// Geometry/ConvexHull.cpp:119-130
/// C++: Polygon convex_hulll(const Polylines &polylines)
pub fn convex_hull_polylines(polylines: &[Polyline]) -> Polygon {
    // Geometry/ConvexHull.cpp:121-126
    // C++: Points pp;
    // C++: size_t sz = 0;
    // C++: for (const auto &polyline : polylines)
    // C++:     sz += polyline.points.size();
    // C++: pp.reserve(sz);
    let sz: usize = polylines.iter().map(|p| p.points().len()).sum();
    let mut pp = Vec::with_capacity(sz);

    // Geometry/ConvexHull.cpp:127-128
    // C++: for (const auto &polyline : polylines)
    // C++:     pp.insert(pp.end(), polyline.points.begin(), polyline.points.end());
    for polyline in polylines {
        pp.extend_from_slice(polyline.points());
    }

    // Geometry/ConvexHull.cpp:129
    // C++: return convex_hull(pp);
    convex_hull_points(pp)
}

/// Check if two convex polygons intersect using separating axis theorem
///
/// Geometry/ConvexHull.cpp:232-263
/// C++: bool convex_polygons_intersect(const Polygon &A, const Polygon &B)
pub fn convex_polygons_intersect(a: &Polygon, b: &Polygon) -> bool {
    // Quick bounding box check first
    // Geometry/ConvexHull.cpp:233-236
    // C++: BoundingBox bba(A.points);
    // C++: bba.merge(B.points);
    // C++: if (bba.size().norm() < SCALED_EPSILON)
    // C++:     return false;
    let bba = a.bounding_box();
    let bbb = b.bounding_box();
    if !bba.intersects(&bbb) {
        return false;
    }

    // Use separating axis theorem - check edges from both polygons
    // Geometry/ConvexHull.cpp:238-261
    // For each edge of polygon A, check if it separates the polygons
    for polygon in [a, b] {
        let pts = polygon.points();
        for i in 0..pts.len() {
            let j = (i + 1) % pts.len();
            let edge = pts[j] - pts[i];
            let normal = Point::new(-edge.y(), edge.x()); // perpendicular

            // Project all points onto this axis
            let (min_a, max_a) = project_polygon(a, &normal);
            let (min_b, max_b) = project_polygon(b, &normal);

            // If projections don't overlap, polygons don't intersect
            if max_a < min_b || max_b < min_a {
                return false;
            }
        }
    }

    // All axes tested, no separation found
    // Geometry/ConvexHull.cpp:262
    // C++: return true;
    true
}

/// Decompose convex polygon into top and bottom chains
///
/// Geometry/ConvexHull.cpp:265-287
/// C++: std::pair<std::vector<Vec2d>, std::vector<Vec2d>>
/// C++: decompose_convex_polygon_top_bottom(const std::vector<Vec2d> &src)
pub fn decompose_convex_polygon_top_bottom(src: &[Vec2d]) -> (Vec<Vec2d>, Vec<Vec2d>) {
    if src.len() < 3 {
        return (Vec::new(), Vec::new());
    }

    // Find leftmost and rightmost points
    // Geometry/ConvexHull.cpp:266-274
    let mut left_idx = 0;
    let mut right_idx = 0;
    for (i, pt) in src.iter().enumerate() {
        if pt.x() < src[left_idx].x() {
            left_idx = i;
        }
        if pt.x() > src[right_idx].x() {
            right_idx = i;
        }
    }

    // Build top chain (left to right along top)
    // Geometry/ConvexHull.cpp:275-280
    let mut top = Vec::new();
    let mut i = left_idx;
    loop {
        top.push(src[i]);
        if i == right_idx {
            break;
        }
        i = (i + 1) % src.len();
    }

    // Build bottom chain (right to left along bottom)
    // Geometry/ConvexHull.cpp:281-286
    let mut bottom = Vec::new();
    let mut i = right_idx;
    loop {
        bottom.push(src[i]);
        if i == left_idx {
            break;
        }
        i = (i + src.len() - 1) % src.len();
    }

    (top, bottom)
}

/// Check if point is inside convex polygon using top/bottom decomposition
///
/// Geometry/ConvexHull.hpp:30
/// C++: bool inside_convex_polygon(
/// C++:     const std::pair<std::vector<Vec2d>, std::vector<Vec2d>> &top_bottom_decomposition,
/// C++:     const Vec2d &pt)
pub fn inside_convex_polygon(decomp: &(Vec<Vec2d>, Vec<Vec2d>), pt: &Vec2d) -> bool {
    let (top, bottom) = decomp;

    if top.is_empty() || bottom.is_empty() {
        return false;
    }

    // Check x bounds
    let x_min = top.first().unwrap().x().min(bottom.first().unwrap().x());
    let x_max = top.last().unwrap().x().max(bottom.last().unwrap().x());

    if pt.x() < x_min || pt.x() > x_max {
        return false;
    }

    // Binary search on top and bottom chains to find y bounds at pt.x()
    let y_top = interpolate_y_at_x(top, pt.x());
    let y_bottom = interpolate_y_at_x(bottom, pt.x());

    // Point is inside if it's between top and bottom chains
    pt.y() >= y_bottom && pt.y() <= y_top
}

// ---------------------------------------------------------------------------
// Helper Functions
// ---------------------------------------------------------------------------

/// Test if three points are in counter-clockwise order
///
/// Geometry.hpp (orient function)
/// C++: inline int orient(const Point &a, const Point &b, const Point &c)
fn is_ccw(p: &Point, a: &Point, b: &Point) -> bool {
    // Cross product test
    // C++: return cross2(b - a, c - b) > 0;
    let dx1 = b.x() - a.x();
    let dy1 = b.y() - a.y();
    let dx2 = p.x() - b.x();
    let dy2 = p.y() - b.y();

    let cross = dx1 as i64 * dy2 as i64 - dy1 as i64 * dx2 as i64;
    cross > 0
}

/// Project polygon onto axis defined by normal vector
fn project_polygon(polygon: &Polygon, normal: &Point) -> (i64, i64) {
    let pts = polygon.points();
    if pts.is_empty() {
        return (0, 0);
    }

    let mut min_proj = dot_product(&pts[0], normal);
    let mut max_proj = min_proj;

    for pt in &pts[1..] {
        let proj = dot_product(pt, normal);
        min_proj = min_proj.min(proj);
        max_proj = max_proj.max(proj);
    }

    (min_proj, max_proj)
}

/// Compute dot product of two points (treated as vectors)
fn dot_product(a: &Point, b: &Point) -> i64 {
    a.x() as i64 * b.x() as i64 + a.y() as i64 * b.y() as i64
}

/// Interpolate y-coordinate at given x along a chain of points
fn interpolate_y_at_x(chain: &[Vec2d], x: CoordF) -> CoordF {
    if chain.len() < 2 {
        return chain.first().map(|p| p.y()).unwrap_or(0.0);
    }

    // Binary search for segment containing x
    for i in 0..chain.len() - 1 {
        let p1 = &chain[i];
        let p2 = &chain[i + 1];

        if x >= p1.x() && x <= p2.x() {
            if (p2.x() - p1.x()).abs() < 1e-10 {
                return p1.y();
            }

            // Linear interpolation
            let t = (x - p1.x()) / (p2.x() - p1.x());
            return p1.y() + t * (p2.y() - p1.y());
        }
    }

    // Outside range, return nearest endpoint
    if x < chain.first().unwrap().x() {
        chain.first().unwrap().y()
    } else {
        chain.last().unwrap().y()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_convex_hull_square() {
        /// Test ConvexHull.cpp:10-36
        let points = vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
            Point::new(50, 50), // Interior point
        ];

        let hull = convex_hull_points(points);
        assert_eq!(hull.points().len(), 4); // Only boundary points
    }

    #[test]
    fn test_convex_hull_collinear() {
        /// Edge case: collinear points
        let points = vec![Point::new(0, 0), Point::new(50, 0), Point::new(100, 0)];

        let hull = convex_hull_points(points);
        assert!(hull.points().len() >= 2);
    }

    #[test]
    fn test_convex_polygons_intersect() {
        /// Test ConvexHull.cpp:232-263
        let a = Polygon::new(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ]);

        let b = Polygon::new(vec![
            Point::new(50, 50),
            Point::new(150, 50),
            Point::new(150, 150),
            Point::new(50, 150),
        ]);

        assert!(convex_polygons_intersect(&a, &b)); // Overlapping
    }

    #[test]
    fn test_convex_polygons_no_intersect() {
        let a = Polygon::new(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ]);

        let b = Polygon::new(vec![
            Point::new(200, 200),
            Point::new(300, 200),
            Point::new(300, 300),
            Point::new(200, 300),
        ]);

        assert!(!convex_polygons_intersect(&a, &b)); // Separated
    }
}
