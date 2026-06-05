//! Wall triangulation between two polygon rings
//!
//! C++ Reference:
//! - TriangulateWall.hpp (lines 1-152)
//! - TriangulateWall.cpp (all code commented out, logic is in the header)
//!
//! This module implements a greedy triangulation algorithm that connects
//! two polygon rings (lower and upper) at different Z heights to form a
//! vertical wall mesh. The algorithm iterates around both rings simultaneously,
//! choosing which triangle to emit based on a scoring heuristic.
//!
//! The C++ `triangulate_wall<Sc, I>` is a template, but its sole instantiation
//! (`SLA/Pad.cpp:39`) fills an `indexed_triangle_set`, whose vertex element is
//! `stl_vertex = Eigen::Matrix<float, 3, 1>` (admesh/stl.h:42) and whose index
//! element is `Vec3i` (int). Hence `Sc = float` and `I = int`. To keep the
//! scoring decisions byte-exact, all distance arithmetic is carried out in
//! `f32` (matching `Sc`), with `calc_score()` widening the result to `f64`
//! (matching `double calc_score()`); vertices are stored as `Vec3f` so the
//! `unscaled(...)` (double) -> float narrowing happens at the same point.

use crate::geometry::Polygon;
use crate::slices_to_triangle_mesh::Vec3f;
use crate::{unscale, Result};

/// A ring iterator that tracks position around a polygon contour
///
/// C++ Reference: TriangulateWall.hpp:10-36 (`trianglulate_wall_detail::Ring`)
///
/// The Ring manages iteration around a contiguous range of vertices,
/// wrapping around when reaching the end. It tracks the current index,
/// next index, and starting index for detecting when iteration is complete.
// TriangulateWall.hpp:10
#[derive(Debug, Clone)]
struct Ring {
    /// Current index in the ring
    // TriangulateWall.hpp:11
    idx: usize,

    /// Next index in the ring
    // TriangulateWall.hpp:11
    nextidx: usize,

    /// Starting index for detecting completion
    // TriangulateWall.hpp:11
    startidx: usize,

    /// Beginning of the ring range (inclusive)
    // TriangulateWall.hpp:11
    begin: usize,

    /// End of the ring range (exclusive)
    // TriangulateWall.hpp:11
    end: usize,
}

impl Ring {
    /// Create a new ring for a range of vertices
    ///
    // TriangulateWall.hpp:14
    // C++: explicit Ring(size_t from, size_t to) : begin(from), end(to) { init(begin); }
    fn new(from: usize, to: usize) -> Self {
        let mut ring = Self {
            // C++ in-class member initializers (TriangulateWall.hpp:11):
            // idx = 0, nextidx = 1, startidx = 0, begin = 0, end = 0
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
    // TriangulateWall.hpp:16
    // C++: size_t size() const { return end - begin; }
    fn size(&self) -> usize {
        self.end - self.begin
    }

    /// Get the current position as (current_index, next_index)
    ///
    // TriangulateWall.hpp:17
    // C++: std::pair<size_t, size_t> pos() const { return {idx, nextidx}; }
    fn pos(&self) -> (usize, usize) {
        (self.idx, self.nextidx)
    }

    /// Check if this is the lower ring
    ///
    // TriangulateWall.hpp:18
    // C++: bool is_lower() const { return idx < size(); }
    fn is_lower(&self) -> bool {
        self.idx < self.size()
    }

    /// Increment to the next position in the ring
    ///
    // TriangulateWall.hpp:20-26
    // C++: void inc()
    // C++: {
    // C++:     if (nextidx != startidx) nextidx++;
    // C++:     if (nextidx == end) nextidx = begin;
    // C++:     idx ++;
    // C++:     if (idx == end) idx = begin;
    // C++: }
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
    // TriangulateWall.hpp:28-33
    // C++: void init(size_t pos)
    // C++: {
    // C++:     startidx = begin + (pos - begin) % size();
    // C++:     idx = startidx;
    // C++:     nextidx = begin + (idx + 1 - begin) % size();
    // C++: }
    fn init(&mut self, pos: usize) {
        self.startidx = self.begin + (pos - self.begin) % self.size();
        self.idx = self.startidx;
        self.nextidx = self.begin + (self.idx + 1 - self.begin) % self.size();
    }

    /// Check if iteration is complete (wrapped around to start)
    ///
    // TriangulateWall.hpp:35
    // C++: bool is_finished() const { return nextidx == idx; }
    fn is_finished(&self) -> bool {
        self.nextidx == self.idx
    }
}

/// Compute squared distance between two 3D points (ignoring Z)
///
// TriangulateWall.hpp:38-43
// C++: template<class Sc>
// C++: static Sc sq_dst(const Vec<3, Sc> &v1, const Vec<3, Sc>& v2)
// C++: {
// C++:     Vec<3, Sc> v = v1 - v2;
// C++:     return v.x() * v.x() + v.y() * v.y() /*+ v.z() * v.z()*/;
// C++: }
//
// Sole instantiation is `Sc = float`, so the subtraction and squaring are
// performed in f32 to keep the score decisions byte-exact.
fn sq_dst(v1: &Vec3f, v2: &Vec3f) -> f32 {
    let dx = v1.x - v2.x;
    let dy = v1.y - v2.y;
    // Note: Z is intentionally ignored (see C++ comment `/*+ v.z() * v.z()*/`).
    dx * dx + dy * dy
}

/// Compute triangulation score for current ring positions
///
// TriangulateWall.hpp:45-53
// C++: template<class Sc>
// C++: static Sc trscore(const Ring &onring, const Ring &offring,
// C++:                   const std::vector<Vec<3, Sc>> &pts)
// C++: {
// C++:     Sc a = sq_dst(pts[onring.pos().first], pts[offring.pos().first]);
// C++:     Sc b = sq_dst(pts[onring.pos().second], pts[offring.pos().first]);
// C++:     return (std::abs(a) + std::abs(b)) / 2.;
// C++: }
//
// Returns `Sc` (= f32). The `/ 2.` divisor is a `double` literal in C++, so the
// `(std::abs(a) + std::abs(b))` sum is promoted to `double` for the division and
// the result narrowed back to `float` on return; we reproduce that exactly.
fn trscore(onring: &Ring, offring: &Ring, pts: &[Vec3f]) -> f32 {
    let (on_first, on_second) = onring.pos();
    let (off_first, _) = offring.pos();

    let a = sq_dst(&pts[on_first], &pts[off_first]);
    let b = sq_dst(&pts[on_second], &pts[off_first]);

    ((a.abs() as f64 + b.abs() as f64) / 2.0) as f32
}

/// Triangulator that greedily connects two polygon rings
///
// TriangulateWall.hpp:55-112 (`trianglulate_wall_detail::Triangulator`)
///
/// This class implements a greedy algorithm that walks around two rings
/// (upper and lower) simultaneously, emitting triangles as it goes.
/// The algorithm chooses which ring to advance based on a distance-based
/// scoring heuristic.
///
/// The C++ uses two raw `Ring *` (`onring`, `offring`) that are swapped with
/// `std::swap`. We store both rings in a fixed array and track which slot is
/// "on" with `onring_idx`, swapping the index instead of the pointers.
struct Triangulator<'a> {
    /// Reference to the 3D points array
    // TriangulateWall.hpp:57: const std::vector<Vec<3, Sc>> *pts;
    pts: &'a [Vec3f],

    /// Index (0 or 1 into `rings`) of the "on" ring (currently advancing).
    // TriangulateWall.hpp:58: Ring *onring, *offring;
    onring_idx: usize,

    /// The two rings we're working with (slot 0 = lower, slot 1 = upper).
    // TriangulateWall.hpp:58 (offring/onring point into this pair)
    rings: [Ring; 2],
}

impl<'a> Triangulator<'a> {
    /// Create a new triangulator
    ///
    // TriangulateWall.hpp:107-111
    // C++: explicit Triangulator(const std::vector<Vec<3, Sc>> *points,
    // C++:                       Ring &lower, Ring &upper)
    // C++:     : pts{points}, onring{&upper}, offring{&lower}
    // C++: {}
    fn new(points: &'a [Vec3f], lower: Ring, upper: Ring) -> Self {
        Self {
            pts: points,
            // onring{&upper}, offring{&lower}: upper occupies slot 1 and starts "on".
            onring_idx: 1,
            rings: [lower, upper],
        }
    }

    /// Get reference to the "on" ring (C++ `*onring`)
    fn onring(&self) -> &Ring {
        &self.rings[self.onring_idx]
    }

    /// Get mutable reference to the "on" ring
    fn onring_mut(&mut self) -> &mut Ring {
        &mut self.rings[self.onring_idx]
    }

    /// Get reference to the "off" ring (C++ `*offring`)
    fn offring(&self) -> &Ring {
        &self.rings[1 - self.onring_idx]
    }

    /// Get mutable reference to the "off" ring
    fn offring_mut(&mut self) -> &mut Ring {
        &mut self.rings[1 - self.onring_idx]
    }

    /// Swap the on/off ring pointers (C++ `std::swap(onring, offring)`)
    fn swap_rings(&mut self) {
        self.onring_idx = 1 - self.onring_idx;
    }

    /// Calculate the triangulation score for current positions
    ///
    // TriangulateWall.hpp:60-63
    // C++: double calc_score() const
    // C++: {
    // C++:     return trscore(*onring, *offring, *pts);
    // C++: }
    //
    // Return type is `double`; `trscore` returns `Sc` (= f32), so the result is
    // widened to f64 here (matching the C++ implicit conversion on return).
    fn calc_score(&self) -> f64 {
        trscore(self.onring(), self.offring(), self.pts) as f64
    }

    /// Synchronize the off-ring to minimize distance to on-ring
    ///
    // TriangulateWall.hpp:65-80
    // C++: void synchronize_rings()
    // C++: {
    // C++:     Ring lring = *offring;
    // C++:     auto minsc = trscore(*onring, lring, *pts);
    // C++:     size_t imin = lring.pos().first;
    // C++:
    // C++:     lring.inc();
    // C++:
    // C++:     while(!lring.is_finished()) {
    // C++:         double score = trscore(*onring, lring, *pts);
    // C++:         if (score < minsc) { minsc = score; imin = lring.pos().first; }
    // C++:         lring.inc();
    // C++:     }
    // C++:
    // C++:     offring->init(imin);
    // C++: }
    //
    // `auto minsc` deduces to `Sc` (= f32); `score` is declared `double`. The
    // comparison `score < minsc` promotes `minsc` to f64; the assignment
    // `minsc = score` narrows `score` back to f32. We reproduce both casts.
    fn synchronize_rings(&mut self) {
        let mut lring = self.offring().clone();
        let mut minsc: f32 = trscore(self.onring(), &lring, self.pts);
        let mut imin = lring.pos().0;

        lring.inc();

        while !lring.is_finished() {
            let score: f64 = trscore(self.onring(), &lring, self.pts) as f64;
            if score < minsc as f64 {
                minsc = score as f32;
                imin = lring.pos().0;
            }
            lring.inc();
        }

        self.offring_mut().init(imin);
    }

    /// Emit a triangle into the indices array
    ///
    // TriangulateWall.hpp:82-88
    // C++: void emplace_indices(std::vector<Vec3i> &indices)
    // C++: {
    // C++:     Vec3i tr{int(onring->pos().first), int(onring->pos().second),
    // C++:              int(offring->pos().first)};
    // C++:     if (onring->is_lower()) std::swap(tr(0), tr(1));
    // C++:     indices.emplace_back(tr);
    // C++: }
    fn emplace_indices(&self, indices: &mut Vec<[usize; 3]>) {
        let (on_first, on_second) = self.onring().pos();
        let (off_first, _) = self.offring().pos();

        let mut tr = [on_first, on_second, off_first];

        // Swap first two indices if on-ring is the lower ring.
        if self.onring().is_lower() {
            tr.swap(0, 1);
        }

        indices.push(tr);
    }

    /// Run the triangulation algorithm
    ///
    // TriangulateWall.hpp:91-105
    // C++: void run(std::vector<Vec3i> &indices)
    // C++: {
    // C++:     synchronize_rings();
    // C++:
    // C++:     double score = 0, prev_score = 0;
    // C++:     while (!onring->is_finished() || !offring->is_finished()) {
    // C++:         prev_score = score;
    // C++:         if (onring->is_finished() || (score = calc_score()) > prev_score) {
    // C++:             std::swap(onring, offring);
    // C++:         } else {
    // C++:             emplace_indices(indices);
    // C++:             onring->inc();
    // C++:         }
    // C++:     }
    // C++: }
    fn run(&mut self, indices: &mut Vec<[usize; 3]>) {
        self.synchronize_rings();

        let mut score: f64 = 0.0;
        let mut prev_score: f64;

        while !self.onring().is_finished() || !self.offring().is_finished() {
            prev_score = score;

            // C++ short-circuit: `score = calc_score()` only evaluates (and
            // reassigns `score`) when `onring->is_finished()` is false.
            if self.onring().is_finished() {
                self.swap_rings();
            } else {
                score = self.calc_score();
                if score > prev_score {
                    self.swap_rings();
                } else {
                    self.emplace_indices(indices);
                    self.onring_mut().inc();
                }
            }
        }
    }
}

/// Triangulate a vertical wall between two polygons at different Z heights
///
// TriangulateWall.hpp:116-139
// C++: template<class Sc, class I>
// C++: void triangulate_wall(std::vector<Vec<3, Sc>> &pts,
// C++:                       std::vector<Vec<3, I>> & ind,
// C++:                       const Polygon &          lower,
// C++:                       const Polygon &          upper,
// C++:                       double                   lower_z_mm,
// C++:                       double                   upper_z_mm)
// C++: {
// C++:     using namespace trianglulate_wall_detail;
// C++:
// C++:     if (upper.points.size() < 3 || lower.points.size() < 3) return;
// C++:
// C++:     pts.reserve(lower.points.size() + upper.points.size());
// C++:     for (auto &p : lower.points)
// C++:         pts.emplace_back(unscaled(p.x()), unscaled(p.y()), lower_z_mm);
// C++:     for (auto &p : upper.points)
// C++:         pts.emplace_back(unscaled(p.x()), unscaled(p.y()), upper_z_mm);
// C++:
// C++:     ind.reserve(2 * (lower.size() + upper.size()));
// C++:
// C++:     Ring lring{0, lower.points.size()}, uring{lower.points.size(), pts.size()};
// C++:     Triangulator t{&pts, lring, uring};
// C++:     t.run(ind);
// C++: }
//
// The C++ template takes `pts`/`ind` as output references and appends to them.
// Its sole instantiation passes the (empty) `vertices`/`indices` of a fresh
// `indexed_triangle_set`, so returning freshly built vectors is equivalent. The
// vertex element matches `indexed_triangle_set::vertices` (= `Vec3f`, f32); the
// `unscaled(...)` (double) -> f32 narrowing happens on store, as in C++.
pub fn triangulate_wall(
    lower: &Polygon,
    upper: &Polygon,
    lower_z_mm: f64,
    upper_z_mm: f64,
) -> Result<(Vec<Vec3f>, Vec<[usize; 3]>)> {
    // C++: if (upper.points.size() < 3 || lower.points.size() < 3) return;
    if upper.points().len() < 3 || lower.points().len() < 3 {
        return Ok((Vec::new(), Vec::new()));
    }

    // C++: pts.reserve(lower.points.size() + upper.points.size());
    let mut pts: Vec<Vec3f> = Vec::with_capacity(lower.points().len() + upper.points().len());

    // C++: for (auto &p : lower.points)
    // C++:     pts.emplace_back(unscaled(p.x()), unscaled(p.y()), lower_z_mm);
    for p in lower.points() {
        pts.push(Vec3f {
            x: unscale(p.x) as f32,
            y: unscale(p.y) as f32,
            z: lower_z_mm as f32,
        });
    }

    // C++: for (auto &p : upper.points)
    // C++:     pts.emplace_back(unscaled(p.x()), unscaled(p.y()), upper_z_mm);
    for p in upper.points() {
        pts.push(Vec3f {
            x: unscale(p.x) as f32,
            y: unscale(p.y) as f32,
            z: upper_z_mm as f32,
        });
    }

    // C++: ind.reserve(2 * (lower.size() + upper.size()));
    let mut indices = Vec::with_capacity(2 * (lower.points().len() + upper.points().len()));

    // C++: Ring lring{0, lower.points.size()}, uring{lower.points.size(), pts.size()};
    let lring = Ring::new(0, lower.points().len());
    let uring = Ring::new(lower.points().len(), pts.len());

    // C++: Triangulator t{&pts, lring, uring};
    // C++: t.run(ind);
    let mut triangulator = Triangulator::new(&pts, lring, uring);
    triangulator.run(&mut indices);

    Ok((pts, indices))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;

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
        for p in pts.iter().take(4) {
            assert_eq!(p.z, 0.0, "Lower points should be at z=0");
        }
        for p in pts.iter().skip(4).take(4) {
            assert_eq!(p.z, 5.0, "Upper points should be at z=5");
        }

        // Should generate triangles.
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
        let p1 = Vec3f {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let p2 = Vec3f {
            x: 3.0,
            y: 4.0,
            z: 100.0,
        }; // Z ignored

        let dist = sq_dst(&p1, &p2);
        assert_eq!(dist, 25.0); // 3^2 + 4^2 = 25
    }
}
