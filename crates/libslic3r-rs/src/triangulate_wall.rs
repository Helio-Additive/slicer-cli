//! Wall triangulation between two polygon rings
//!
//! C++ Reference:
//! - TriangulateWall.hpp (lines 1-158)
//! - TriangulateWall.cpp (all code commented out, logic is in header)
//!
//! This module implements a greedy triangulation algorithm that connects
//! two polygon rings (lower and upper) at different Z heights to form a
//! vertical wall mesh. The algorithm iterates around both rings simultaneously,
//! choosing which triangle to emit based on a scoring heuristic.

use crate::geometry::{Point, Point3F, Polygon};
use crate::{unscale, Result};

/// A ring iterator that tracks position around a polygon contour
///
/// C++ Reference: TriangulateWall.hpp:10-35
///
/// The Ring manages iteration around a contiguous range of vertices,
/// wrapping around when reaching the end. It tracks the current index,
/// next index, and starting index for detecting when iteration is complete.
#[derive(Debug, Clone)]
struct Ring {
    /// Current index in the ring
    /// TriangulateWall.hpp:11
    idx: usize,

    /// Next index in the ring
    /// TriangulateWall.hpp:11
    nextidx: usize,

    /// Starting index for detecting completion
    /// TriangulateWall.hpp:11
    startidx: usize,

    /// Beginning of the ring range (inclusive)
    /// TriangulateWall.hpp:11
    begin: usize,

    /// End of the ring range (exclusive)
    /// TriangulateWall.hpp:11
    end: usize,
}

impl Ring {
    /// Create a new ring for a range of vertices
    ///
    /// TriangulateWall.hpp:14
    /// C++: explicit Ring(size_t from, size_t to) : begin(from), end(to) { init(begin); }
    fn new(from: usize, to: usize) -> Self {
        let mut ring = Self {
            idx: 0,
            nextidx: 1,
            startidx: 0,
            begin: from,
            end: to,
        };
        ring.init(from);
        ring
    }

    /// Get the number of vertices in this ring
    ///
    /// TriangulateWall.hpp:16
    /// C++: size_t size() const { return end - begin; }
    fn size(&self) -> usize {
        self.end - self.begin
    }

    /// Get the current position as (current_index, next_index)
    ///
    /// TriangulateWall.hpp:17
    /// C++: std::pair<size_t, size_t> pos() const { return {idx, nextidx}; }
    fn pos(&self) -> (usize, usize) {
        (self.idx, self.nextidx)
    }

    /// Check if this is the lower ring
    ///
    /// TriangulateWall.hpp:18
    /// C++: bool is_lower() const { return idx < size(); }
    fn is_lower(&self) -> bool {
        self.idx < self.size()
    }

    /// Increment to the next position in the ring
    ///
    /// TriangulateWall.hpp:20-26
    /// C++: void inc()
    /// C++: {
    /// C++:     if (nextidx != startidx) nextidx++;
    /// C++:     if (nextidx == end) nextidx = begin;
    /// C++:     idx ++;
    /// C++:     if (idx == end) idx = begin;
    /// C++: }
    fn inc(&mut self) {
        if self.nextidx != self.startidx {
            self.nextidx += 1;
        }
        if self.nextidx == self.end {
            self.nextidx = self.begin;
        }
        self.idx += 1;
        if self.idx == self.end {
            self.idx = self.begin;
        }
    }

    /// Initialize the ring to start at a given position
    ///
    /// TriangulateWall.hpp:28-33
    /// C++: void init(size_t pos)
    /// C++: {
    /// C++:     startidx = begin + (pos - begin) % size();
    /// C++:     idx = startidx;
    /// C++:     nextidx = begin + (idx + 1 - begin) % size();
    /// C++: }
    fn init(&mut self, pos: usize) {
        self.startidx = self.begin + (pos - self.begin) % self.size();
        self.idx = self.startidx;
        self.nextidx = self.begin + (self.idx + 1 - self.begin) % self.size();
    }

    /// Check if iteration is complete (wrapped around to start)
    ///
    /// TriangulateWall.hpp:35
    /// C++: bool is_finished() const { return nextidx == idx; }
    fn is_finished(&self) -> bool {
        self.nextidx == self.idx
    }
}

/// Compute squared distance between two 3D points (ignoring Z)
///
/// TriangulateWall.hpp:38-42
/// C++: template<class Sc>
/// C++: static Sc sq_dst(const Vec<3, Sc> &v1, const Vec<3, Sc>& v2)
/// C++: {
/// C++:     Vec<3, Sc> v = v1 - v2;
/// C++:     return v.x() * v.x() + v.y() * v.y() /*+ v.z() * v.z()*/;
/// C++: }
fn sq_dst(v1: &Point3F, v2: &Point3F) -> f64 {
    let dx = v1.x - v2.x;
    let dy = v1.y - v2.y;
    // Note: Z is intentionally ignored (see C++ comment)
    dx * dx + dy * dy
}

/// Compute triangulation score for current ring positions
///
/// TriangulateWall.hpp:44-51
/// C++: template<class Sc>
/// C++: static Sc trscore(const Ring &onring, const Ring &offring,
/// C++:                   const std::vector<Vec<3, Sc>> &pts)
/// C++: {
/// C++:     Sc a = sq_dst(pts[onring.pos().first], pts[offring.pos().first]);
/// C++:     Sc b = sq_dst(pts[onring.pos().second], pts[offring.pos().first]);
/// C++:     return (std::abs(a) + std::abs(b)) / 2.;
/// C++: }
fn trscore(onring: &Ring, offring: &Ring, pts: &[Point3F]) -> f64 {
    let (on_first, on_second) = onring.pos();
    let (off_first, _) = offring.pos();

    let a = sq_dst(&pts[on_first], &pts[off_first]);
    let b = sq_dst(&pts[on_second], &pts[off_first]);

    (a.abs() + b.abs()) / 2.0
}

/// Triangulator that greedily connects two polygon rings
///
/// TriangulateWall.hpp:53-109
///
/// This class implements a greedy algorithm that walks around two rings
/// (upper and lower) simultaneously, emitting triangles as it goes.
/// The algorithm chooses which ring to advance based on a distance-based
/// scoring heuristic.
struct Triangulator<'a> {
    /// Reference to the 3D points array
    /// TriangulateWall.hpp:54
    pts: &'a [Point3F],

    /// Pointer to the "on" ring (the one we're currently advancing)
    /// TriangulateWall.hpp:55
    onring_idx: usize, // 0 or 1

    /// The two rings we're working with
    /// TriangulateWall.hpp:55 (offring)
    rings: [Ring; 2],
}

impl<'a> Triangulator<'a> {
    /// Create a new triangulator
    ///
    /// TriangulateWall.hpp:103-107
    /// C++: explicit Triangulator(const std::vector<Vec<3, Sc>> *points,
    /// C++:                       Ring &lower, Ring &upper)
    /// C++:     : pts{points}, onring{&upper}, offring{&lower}
    /// C++: {}
    fn new(points: &'a [Point3F], lower: Ring, upper: Ring) -> Self {
        Self {
            pts: points,
            onring_idx: 1, // Start with upper ring as "on"
            rings: [lower, upper],
        }
    }

    /// Get reference to the "on" ring
    fn onring(&self) -> &Ring {
        &self.rings[self.onring_idx]
    }

    /// Get mutable reference to the "on" ring
    fn onring_mut(&mut self) -> &mut Ring {
        &mut self.rings[self.onring_idx]
    }

    /// Get reference to the "off" ring
    fn offring(&self) -> &Ring {
        &self.rings[1 - self.onring_idx]
    }

    /// Get mutable reference to the "off" ring
    fn offring_mut(&mut self) -> &mut Ring {
        &mut self.rings[1 - self.onring_idx]
    }

    /// Swap the on/off ring pointers
    fn swap_rings(&mut self) {
        self.onring_idx = 1 - self.onring_idx;
    }

    /// Calculate the triangulation score for current positions
    ///
    /// TriangulateWall.hpp:57-60
    /// C++: double calc_score() const
    /// C++: {
    /// C++:     return trscore(*onring, *offring, *pts);
    /// C++: }
    fn calc_score(&self) -> f64 {
        trscore(self.onring(), self.offring(), self.pts)
    }

    /// Synchronize the off-ring to minimize distance to on-ring
    ///
    /// TriangulateWall.hpp:62-77
    /// C++: void synchronize_rings()
    /// C++: {
    /// C++:     Ring lring = *offring;
    /// C++:     auto minsc = trscore(*onring, lring, *pts);
    /// C++:     size_t imin = lring.pos().first;
    /// C++:
    /// C++:     lring.inc();
    /// C++:
    /// C++:     while(!lring.is_finished()) {
    /// C++:         double score = trscore(*onring, lring, *pts);
    /// C++:         if (score < minsc) { minsc = score; imin = lring.pos().first; }
    /// C++:         lring.inc();
    /// C++:     }
    /// C++:
    /// C++:     offring->init(imin);
    /// C++: }
    fn synchronize_rings(&mut self) {
        let mut lring = self.offring().clone();
        let mut minsc = trscore(self.onring(), &lring, self.pts);
        let mut imin = lring.pos().0;

        lring.inc();

        while !lring.is_finished() {
            let score = trscore(self.onring(), &lring, self.pts);
            if score < minsc {
                minsc = score;
                imin = lring.pos().0;
            }
            lring.inc();
        }

        self.offring_mut().init(imin);
    }

    /// Emit a triangle into the indices array
    ///
    /// TriangulateWall.hpp:79-86
    /// C++: void emplace_indices(std::vector<Vec3i> &indices)
    /// C++: {
    /// C++:     Vec3i tr{int(onring->pos().first), int(onring->pos().second),
    /// C++:              int(offring->pos().first)};
    /// C++:     if (onring->is_lower()) std::swap(tr(0), tr(1));
    /// C++:     indices.emplace_back(tr);
    /// C++: }
    fn emplace_indices(&self, indices: &mut Vec<[usize; 3]>) {
        let (on_first, on_second) = self.onring().pos();
        let (off_first, _) = self.offring().pos();

        let mut tr = [on_first, on_second, off_first];

        // Swap first two indices if on-ring is the lower ring
        if self.onring().is_lower() {
            tr.swap(0, 1);
        }

        indices.push(tr);
    }

    /// Run the triangulation algorithm
    ///
    /// TriangulateWall.hpp:88-101
    /// C++: void run(std::vector<Vec3i> &indices)
    /// C++: {
    /// C++:     synchronize_rings();
    /// C++:
    /// C++:     double score = 0, prev_score = 0;
    /// C++:     while (!onring->is_finished() || !offring->is_finished()) {
    /// C++:         prev_score = score;
    /// C++:         if (onring->is_finished() || (score = calc_score()) > prev_score) {
    /// C++:             std::swap(onring, offring);
    /// C++:         } else {
    /// C++:             emplace_indices(indices);
    /// C++:             onring->inc();
    /// C++:         }
    /// C++:     }
    /// C++: }
    fn run(&mut self, indices: &mut Vec<[usize; 3]>) {
        self.synchronize_rings();

        let mut score = 0.0;
        let mut prev_score = 0.0;

        while !self.onring().is_finished() || !self.offring().is_finished() {
            prev_score = score;

            if self.onring().is_finished() {
                // On-ring is finished, must swap to continue
                self.swap_rings();
            } else {
                score = self.calc_score();
                if score > prev_score {
                    // Score got worse, swap rings
                    self.swap_rings();
                } else {
                    // Score is good, emit triangle and advance
                    self.emplace_indices(indices);
                    self.onring_mut().inc();
                }
            }
        }
    }
}

/// Triangulate a vertical wall between two polygons at different Z heights
///
/// TriangulateWall.hpp:111-140
/// C++: template<class Sc, class I>
/// C++: void triangulate_wall(std::vector<Vec<3, Sc>> &pts,
/// C++:                       std::vector<Vec<3, I>> & ind,
/// C++:                       const Polygon &          lower,
/// C++:                       const Polygon &          upper,
/// C++:                       double                   lower_z_mm,
/// C++:                       double                   upper_z_mm)
/// C++: {
/// C++:     using namespace trianglulate_wall_detail;
/// C++:
/// C++:     if (upper.points.size() < 3 || lower.points.size() < 3) return;
/// C++:
/// C++:     pts.reserve(lower.points.size() + upper.points.size());
/// C++:     for (auto &p : lower.points)
/// C++:         pts.emplace_back(unscaled(p.x()), unscaled(p.y()), lower_z_mm);
/// C++:     for (auto &p : upper.points)
/// C++:         pts.emplace_back(unscaled(p.x()), unscaled(p.y()), upper_z_mm);
/// C++:
/// C++:     ind.reserve(2 * (lower.size() + upper.size()));
/// C++:
/// C++:     Ring lring{0, lower.points.size()}, uring{lower.points.size(), pts.size()};
/// C++:     Triangulator t{&pts, lring, uring};
/// C++:     t.run(ind);
/// C++: }
pub fn triangulate_wall(
    lower: &Polygon,
    upper: &Polygon,
    lower_z_mm: f64,
    upper_z_mm: f64,
) -> Result<(Vec<Point3F>, Vec<[usize; 3]>)> {
    // Validate input
    if upper.points().len() < 3 || lower.points().len() < 3 {
        return Ok((Vec::new(), Vec::new()));
    }

    // Build the 3D points array
    let mut pts = Vec::with_capacity(lower.points().len() + upper.points().len());

    // Add lower polygon points at lower_z
    for p in lower.points() {
        pts.push(Point3F {
            x: unscale(p.x),
            y: unscale(p.y),
            z: lower_z_mm,
        });
    }

    // Add upper polygon points at upper_z
    for p in upper.points() {
        pts.push(Point3F {
            x: unscale(p.x),
            y: unscale(p.y),
            z: upper_z_mm,
        });
    }

    // Prepare indices array
    let mut indices = Vec::with_capacity(2 * (lower.points().len() + upper.points().len()));

    // Create rings for lower and upper polygons
    let lring = Ring::new(0, lower.points().len());
    let uring = Ring::new(lower.points().len(), pts.len());

    // Run triangulation
    let mut triangulator = Triangulator::new(&pts, lring, uring);
    triangulator.run(&mut indices);

    Ok((pts, indices))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scale;

    /// Helper to create a square polygon
    fn square_polygon(size: f64) -> Polygon {
        let s = crate::scale(size);
        Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(s, 0),
            Point::new(s, s),
            Point::new(0, s),
        ])
    }

    #[test]
    fn test_triangulate_wall_empty() {
        let lower = Polygon::from_points(vec![]);
        let upper = Polygon::from_points(vec![]);

        let result = triangulate_wall(&lower, &upper, 0.0, 1.0);
        assert!(result.is_ok());

        let (pts, indices) = result.unwrap();
        assert_eq!(pts.len(), 0);
        assert_eq!(indices.len(), 0);
    }

    #[test]
    fn test_triangulate_wall_too_few_points() {
        let lower = Polygon::from_points(vec![Point::new(0, 0), Point::new(100, 0)]);
        let upper = square_polygon(10.0);

        let result = triangulate_wall(&lower, &upper, 0.0, 1.0);
        assert!(result.is_ok());

        let (pts, indices) = result.unwrap();
        assert_eq!(pts.len(), 0);
        assert_eq!(indices.len(), 0);
    }

    #[test]
    fn test_triangulate_wall_squares() {
        let lower = square_polygon(10.0);
        let upper = square_polygon(8.0);

        let result = triangulate_wall(&lower, &upper, 0.0, 5.0);
        assert!(result.is_ok());

        let (pts, indices) = result.unwrap();

        // Should have 8 points total (4 lower + 4 upper)
        assert_eq!(pts.len(), 8);

        // Check Z coordinates
        for i in 0..4 {
            assert_eq!(pts[i].z, 0.0, "Lower points should be at z=0");
        }
        for i in 4..8 {
            assert_eq!(pts[i].z, 5.0, "Upper points should be at z=5");
        }

        // Should generate triangles (approximately 2 * (lower.len() + upper.len()))
        // Exact count depends on algorithm but should be non-empty
        assert!(!indices.is_empty(), "Should generate triangles");

        // All indices should be valid
        for tri in &indices {
            for &idx in tri {
                assert!(idx < pts.len(), "Triangle index out of bounds");
            }
        }
    }

    #[test]
    fn test_triangulate_wall_different_vertex_counts() {
        // Triangle lower, square upper
        let lower = Polygon::from_points(vec![
            Point::new(0, 0),
            Point::new(crate::scale(10.0), 0),
            Point::new(crate::scale(5.0), crate::scale(8.66)),
        ]);
        let upper = square_polygon(6.0);

        let result = triangulate_wall(&lower, &upper, 0.0, 3.0);
        assert!(result.is_ok());

        let (pts, indices) = result.unwrap();

        // 3 lower + 4 upper = 7 points
        assert_eq!(pts.len(), 7);

        // Should have generated triangles
        assert!(!indices.is_empty());

        // Validate all triangles
        for tri in &indices {
            for &idx in tri {
                assert!(idx < pts.len());
            }
        }
    }

    #[test]
    fn test_ring_basic() {
        let mut ring = Ring::new(0, 5);

        assert_eq!(ring.size(), 5);
        assert_eq!(ring.pos(), (0, 1));
        assert!(!ring.is_finished());

        // Increment through all positions
        for i in 1..5 {
            ring.inc();
            let (idx, nextidx) = ring.pos();
            assert_eq!(idx, i);
            assert_eq!(nextidx, (i + 1) % 5);
        }

        // One more increment should finish it
        ring.inc();
        assert!(ring.is_finished());
    }

    #[test]
    fn test_ring_init() {
        let mut ring = Ring::new(10, 15);

        // Initialize at position 12
        ring.init(12);
        assert_eq!(ring.pos(), (12, 13));

        ring.inc();
        assert_eq!(ring.pos(), (13, 14));

        ring.inc();
        assert_eq!(ring.pos(), (14, 10)); // wraps around
    }

    #[test]
    fn test_sq_dst() {
        let p1 = Point3F {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let p2 = Point3F {
            x: 3.0,
            y: 4.0,
            z: 100.0,
        }; // Z ignored

        let dist = sq_dst(&p1, &p2);
        assert_eq!(dist, 25.0); // 3^2 + 4^2 = 25
    }
}
