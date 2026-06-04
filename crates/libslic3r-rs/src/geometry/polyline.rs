//! Polyline type for open paths.
//!
//! This module provides the Polyline type representing an open path (sequence of connected line segments),
//! mirroring BambuStudio's Polyline class.

use super::{BoundingBox, Line, Point, Polygon};
use crate::{Coord, CoordF};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Deref, DerefMut, Index, IndexMut};

/// An open polyline defined by a sequence of points.
///
/// Unlike a Polygon, a Polyline is not implicitly closed - it's a path from
/// the first point to the last point.
#[derive(Clone, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
/// Polyline - open path of 2D points
/// Polyline.hpp: class Polyline : public MultiPoint
/// C++: class MultiPoint { public: Points points; }
pub struct Polyline {
    /// Points defining the polyline (public to match C++ MultiPoint)
    /// MultiPoint.hpp:9
    /// C++: Points points;
    pub points: Vec<Point>,
}

impl Polyline {
    // Create a new empty polyline.
    #[inline]
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Create a polyline from a vector of points.
    #[inline]
    pub fn from_points(points: Vec<Point>) -> Self {
        Self { points }
    }

    /// Create a polyline from a polygon (closes the polygon by repeating the first point).
    #[inline]
    pub fn from_polygon(polygon: &Polygon) -> Self {
        let mut points = polygon.points().to_vec();
        if !points.is_empty() && points.first() != points.last() {
            points.push(points[0]);
        }
        Self { points }
    }

    /// Create a polyline with the given capacity.
    #[inline]
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            points: Vec::with_capacity(capacity),
        }
    }

    /// Get the points of this polyline.
    #[inline]
    pub fn points(&self) -> &[Point] {
        &self.points
    }

    /// Get a mutable reference to the points.
    #[inline]
    pub fn points_mut(&mut self) -> &mut Vec<Point> {
        &mut self.points
    }

    /// Consume the polyline and return its points.
    #[inline]
    pub fn into_points(self) -> Vec<Point> {
        self.points
    }

    /// Get the number of points in the polyline.
    #[inline]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Check if the polyline is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Add a point to the polyline.
    #[inline]
    pub fn push(&mut self, point: Point) {
        self.points.push(point);
    }

    /// Remove and return the last point.
    #[inline]
    pub fn pop(&mut self) -> Option<Point> {
        self.points.pop()
    }

    /// Clear all points from the polyline.
    #[inline]
    pub fn clear(&mut self) {
        self.points.clear();
    }

    /// Reserve capacity for additional points.
    #[inline]
    pub fn reserve(&mut self, additional: usize) {
        self.points.reserve(additional);
    }

    /// Get the first point, if any.
    #[inline]
    pub fn first(&self) -> Option<&Point> {
        self.points.first()
    }

    /// Get the last point, if any.
    #[inline]
    pub fn last(&self) -> Option<&Point> {
        self.points.last()
    }

    /// Get the first point, panicking if empty.
    #[inline]
    pub fn first_point(&self) -> Point {
        self.points[0]
    }

    /// Get the last point, panicking if empty.
    #[inline]
    pub fn last_point(&self) -> Point {
        self.points[self.points.len() - 1]
    }

    /// Get the line segment at the given index (from point[i] to point[i+1]).
    #[inline]
    pub fn edge(&self, index: usize) -> Line {
        Line::new(self.points[index], self.points[index + 1])
    }

    /// Get all edges of the polyline.
    pub fn edges(&self) -> Vec<Line> {
        if self.points.len() < 2 {
            return Vec::new();
        }

        let mut edges = Vec::with_capacity(self.points.len() - 1);
        for i in 0..(self.points.len() - 1) {
            edges.push(self.edge(i));
        }
        edges
    }

    /// Get the number of edges in the polyline.
    #[inline]
    pub fn edge_count(&self) -> usize {
        if self.points.len() < 2 {
            0
        } else {
            self.points.len() - 1
        }
    }

    /// Calculate the total length of the polyline.
    pub fn length(&self) -> CoordF {
        if self.points.len() < 2 {
            return 0.0;
        }

        let mut total = 0.0;
        for i in 0..(self.points.len() - 1) {
            total += self.points[i].distance(&self.points[i + 1]);
        }
        total
    }

    /// Check if this polyline is closed (first point equals last point).
    #[inline]
    pub fn is_closed(&self) -> bool {
        self.points.len() >= 2 && self.points.first() == self.points.last()
    }

    /// Reverse the order of points in the polyline.
    pub fn reverse(&mut self) {
        self.points.reverse();
    }

    /// Return a reversed copy of the polyline.
    pub fn reversed(&self) -> Self {
        let mut result = self.clone();
        result.reverse();
        result
    }

    /// Get the bounding box of the polyline.
    pub fn bounding_box(&self) -> BoundingBox {
        BoundingBox::from_points(&self.points)
    }

    /// Calculate the center point's projection onto a direction vector.
    /// Used for sorting polylines by their position along a sweep direction.
    pub fn center_projection(&self, cos_a: f64, sin_a: f64) -> f64 {
        if self.points.is_empty() {
            return 0.0;
        }

        // Calculate centroid
        let mut cx = 0i64;
        let mut cy = 0i64;
        for p in &self.points {
            cx += p.x;
            cy += p.y;
        }
        cx /= self.points.len() as i64;
        cy /= self.points.len() as i64;

        // Project onto direction vector
        cx as f64 * cos_a + cy as f64 * sin_a
    }

    /// Find the closest point on the polyline to the given point.
    pub fn closest_point(&self, p: &Point) -> Point {
        if self.points.is_empty() {
            return Point::zero();
        }

        if self.points.len() == 1 {
            return self.points[0];
        }

        let mut closest = self.points[0];
        let mut min_dist = i128::MAX;

        for edge in self.edges() {
            let proj = edge.project_point(p);
            let dist = p.distance_squared(&proj);
            if dist < min_dist {
                min_dist = dist;
                closest = proj;
            }
        }

        closest
    }

    /// Distance from a point to the polyline.
    pub fn distance_to_point(&self, p: &Point) -> CoordF {
        let closest = self.closest_point(p);
        p.distance(&closest)
    }

    /// Translate the polyline by a vector.
    pub fn translate(&mut self, v: Point) {
        for p in &mut self.points {
            *p = *p + v;
        }
    }

    /// Return a translated copy of the polyline.
    pub fn translated(&self, v: Point) -> Self {
        let mut result = self.clone();
        result.translate(v);
        result
    }

    /// Scale the polyline about the origin.
    pub fn scale(&mut self, factor: CoordF) {
        for p in &mut self.points {
            *p = *p * factor;
        }
    }

    /// Return a scaled copy of the polyline.
    pub fn scaled(&self, factor: CoordF) -> Self {
        let mut result = self.clone();
        result.scale(factor);
        result
    }

    /// Rotate the polyline about the origin.
    pub fn rotate(&mut self, angle: CoordF) {
        for p in &mut self.points {
            *p = p.rotate(angle);
        }
    }

    /// Return a rotated copy of the polyline.
    pub fn rotated(&self, angle: CoordF) -> Self {
        let mut result = self.clone();
        result.rotate(angle);
        result
    }

    /// Simplify the polyline by removing collinear and duplicate points.
    pub fn simplify(&mut self, tolerance: Coord) {
        if self.points.len() < 3 {
            return;
        }

        let mut new_points = Vec::with_capacity(self.points.len());
        new_points.push(self.points[0]);

        for i in 1..(self.points.len() - 1) {
            // Skip duplicate points
            if self.points[i].coincides_with(&self.points[i - 1], tolerance) {
                continue;
            }

            // Check if point is collinear with neighbors
            let prev = self.points[i - 1];
            let curr = self.points[i];
            let next = self.points[i + 1];

            let line = Line::new(prev, next);
            let dist = line.distance_to_point(&curr);

            if dist > tolerance as CoordF {
                new_points.push(curr);
            }
        }

        // Always keep the last point
        if let Some(&last) = self.points.last() {
            if !new_points
                .last()
                .map_or(false, |p| p.coincides_with(&last, tolerance))
            {
                new_points.push(last);
            }
        }

        self.points = new_points;
    }

    /// Return a simplified copy of the polyline.
    pub fn simplified(&self, tolerance: Coord) -> Self {
        let mut result = self.clone();
        result.simplify(tolerance);
        result
    }

    /// Split the polyline at a given index, returning two polylines.
    pub fn split_at(&self, index: usize) -> (Self, Self) {
        if index == 0 {
            return (Self::new(), self.clone());
        }
        if index >= self.points.len() {
            return (self.clone(), Self::new());
        }

        let first = Self::from_points(self.points[..=index].to_vec());
        let second = Self::from_points(self.points[index..].to_vec());

        (first, second)
    }

    /// Append another polyline to this one.
    pub fn append(&mut self, other: &Polyline) {
        self.points.extend_from_slice(&other.points);
    }

    /// Concatenate two polylines.
    pub fn concat(&self, other: &Polyline) -> Self {
        let mut result = self.clone();
        result.append(other);
        result
    }

    /// Clip the polyline to a given length from the start.
    pub fn clip_start(&mut self, distance: CoordF) {
        if distance <= 0.0 || self.points.len() < 2 {
            return;
        }

        let mut remaining = distance;
        let mut new_start = 0;

        for i in 0..(self.points.len() - 1) {
            let edge_len = self.points[i].distance(&self.points[i + 1]);
            if remaining < edge_len {
                // Interpolate the new start point
                let t = remaining / edge_len;
                let new_point = Point::new(
                    (self.points[i].x as CoordF
                        + t * (self.points[i + 1].x - self.points[i].x) as CoordF)
                        .round() as Coord,
                    (self.points[i].y as CoordF
                        + t * (self.points[i + 1].y - self.points[i].y) as CoordF)
                        .round() as Coord,
                );
                self.points = std::iter::once(new_point)
                    .chain(self.points[(i + 1)..].iter().copied())
                    .collect();
                return;
            }
            remaining -= edge_len;
            new_start = i + 1;
        }

        // Entire polyline is shorter than distance
        if new_start >= self.points.len() - 1 {
            self.points = vec![self.points[self.points.len() - 1]];
        } else {
            self.points = self.points[new_start..].to_vec();
        }
    }

    /// Clip the polyline to a given length from the end.
    pub fn clip_end(&mut self, distance: CoordF) {
        self.reverse();
        self.clip_start(distance);
        self.reverse();
    }

    /// Convert to a polygon (closing the path).
    pub fn to_polygon(&self) -> Polygon {
        Polygon::from_points(self.points.clone())
    }

    /// Check if the polyline is valid (has at least 2 points).
    pub fn is_valid(&self) -> bool {
        self.points.len() >= 2
    }

    /// Get the direction at a point on the polyline (as a unit vector).
    pub fn direction_at(&self, index: usize) -> Option<Point> {
        if self.points.len() < 2 || index >= self.points.len() {
            return None;
        }

        if index == 0 {
            // Direction at start
            let edge = self.edge(0);
            let dir = edge.direction();
            let len = dir.length();
            if len > 0.0 {
                Some(dir * (1.0 / len))
            } else {
                None
            }
        } else if index == self.points.len() - 1 {
            // Direction at end
            let edge = self.edge(index - 1);
            let dir = edge.direction();
            let len = dir.length();
            if len > 0.0 {
                Some(dir * (1.0 / len))
            } else {
                None
            }
        } else {
            // Average of incoming and outgoing edges
            let in_edge = self.edge(index - 1);
            let out_edge = self.edge(index);
            let in_dir = in_edge.direction();
            let out_dir = out_edge.direction();
            let in_len = in_dir.length();
            let out_len = out_dir.length();
            if in_len > 0.0 && out_len > 0.0 {
                let avg = Point::new(
                    ((in_dir.x as CoordF / in_len + out_dir.x as CoordF / out_len) / 2.0).round()
                        as Coord,
                    ((in_dir.y as CoordF / in_len + out_dir.y as CoordF / out_len) / 2.0).round()
                        as Coord,
                );
                let avg_len = avg.length();
                if avg_len > 0.0 {
                    Some(avg * (1.0 / avg_len))
                } else {
                    None
                }
            } else {
                None
            }
        }
    }

    /// Simplify polyline using Douglas-Peucker algorithm (static method)
    /// MultiPoint.hpp:99
    /// C++: static Points _douglas_peucker(const Points &points, const double tolerance);
    pub fn douglas_peucker(polyline: &Polyline, tolerance: CoordF) -> Polyline {
        use crate::geometry::simplify::douglas_peucker_polyline;
        douglas_peucker_polyline(polyline, tolerance)
    }
}

impl fmt::Debug for Polyline {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Polyline({} points)", self.points.len())
    }
}

impl fmt::Display for Polyline {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Polyline[")?;
        for (i, p) in self.points.iter().enumerate() {
            if i > 0 {
                write!(f, " -> ")?;
            }
            write!(f, "{}", p)?;
        }
        write!(f, "]")
    }
}

impl Deref for Polyline {
    type Target = [Point];

    fn deref(&self) -> &Self::Target {
        &self.points
    }
}

impl DerefMut for Polyline {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.points
    }
}

impl Index<usize> for Polyline {
    type Output = Point;

    fn index(&self, index: usize) -> &Self::Output {
        &self.points[index]
    }
}

impl IndexMut<usize> for Polyline {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.points[index]
    }
}

impl FromIterator<Point> for Polyline {
    fn from_iter<I: IntoIterator<Item = Point>>(iter: I) -> Self {
        Self {
            points: iter.into_iter().collect(),
        }
    }
}

impl IntoIterator for Polyline {
    type Item = Point;
    type IntoIter = std::vec::IntoIter<Point>;

    fn into_iter(self) -> Self::IntoIter {
        self.points.into_iter()
    }
}

impl<'a> IntoIterator for &'a Polyline {
    type Item = &'a Point;
    type IntoIter = std::slice::Iter<'a, Point>;

    fn into_iter(self) -> Self::IntoIter {
        self.points.iter()
    }
}

impl<'a> IntoIterator for &'a mut Polyline {
    type Item = &'a mut Point;
    type IntoIter = std::slice::IterMut<'a, Point>;

    fn into_iter(self) -> Self::IntoIter {
        self.points.iter_mut()
    }
}

impl From<Vec<Point>> for Polyline {
    fn from(points: Vec<Point>) -> Self {
        Self::from_points(points)
    }
}

impl From<Polyline> for Vec<Point> {
    fn from(polyline: Polyline) -> Self {
        polyline.into_points()
    }
}

impl From<Polygon> for Polyline {
    fn from(polygon: Polygon) -> Self {
        Self::from_points(polygon.into_points())
    }
}

/// Type alias for a collection of polylines.
pub type Polylines = Vec<Polyline>;

#[cfg(test)]
mod tests {
    use super::*;

    fn make_polyline() -> Polyline {
        Polyline::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 100),
        ])
    }

    #[test]
    fn test_polyline_new() {
        let pl = Polyline::new();
        assert!(pl.is_empty());
        assert_eq!(pl.len(), 0);
    }

    #[test]
    fn test_polyline_from_points() {
        let pl = make_polyline();
        assert_eq!(pl.len(), 4);
        assert!(!pl.is_empty());
    }

    #[test]
    fn test_polyline_edge() {
        let pl = make_polyline();
        let edge = pl.edge(0);
        assert_eq!(edge.a, Point::new(0, 0));
        assert_eq!(edge.b, Point::new(100, 0));
    }

    #[test]
    fn test_polyline_edges() {
        let pl = make_polyline();
        let edges = pl.edges();
        assert_eq!(edges.len(), 3); // 4 points = 3 edges (open path)
    }

    #[test]
    fn test_polyline_length() {
        let pl = make_polyline();
        let len = pl.length();
        assert!((len - 300.0).abs() < 1.0); // 100 + 100 + 100
    }

    #[test]
    fn test_polyline_is_closed() {
        let pl = make_polyline();
        assert!(!pl.is_closed());

        let closed = Polyline::from_points(vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 100),
            Point::new(0, 0),
        ]);
        assert!(closed.is_closed());
    }

    #[test]
    fn test_polyline_first_last() {
        let pl = make_polyline();
        assert_eq!(pl.first_point(), Point::new(0, 0));
        assert_eq!(pl.last_point(), Point::new(0, 100));
    }

    #[test]
    fn test_polyline_reverse() {
        let mut pl = make_polyline();
        pl.reverse();
        assert_eq!(pl.first_point(), Point::new(0, 100));
        assert_eq!(pl.last_point(), Point::new(0, 0));
    }

    #[test]
    fn test_polyline_translate() {
        let mut pl = make_polyline();
        pl.translate(Point::new(10, 20));
        assert_eq!(pl[0], Point::new(10, 20));
        assert_eq!(pl[1], Point::new(110, 20));
    }

    #[test]
    fn test_polyline_split() {
        let pl = make_polyline();
        let (first, second) = pl.split_at(2);
        assert_eq!(first.len(), 3);
        assert_eq!(second.len(), 2);
        assert_eq!(first.last_point(), second.first_point());
    }

    #[test]
    fn test_polyline_append() {
        let mut pl1 = Polyline::from_points(vec![Point::new(0, 0), Point::new(100, 0)]);
        let pl2 = Polyline::from_points(vec![Point::new(100, 0), Point::new(100, 100)]);
        pl1.append(&pl2);
        assert_eq!(pl1.len(), 4);
    }

    #[test]
    fn test_polyline_is_valid() {
        let pl = make_polyline();
        assert!(pl.is_valid());

        let single = Polyline::from_points(vec![Point::new(0, 0)]);
        assert!(!single.is_valid());
    }

    #[test]
    fn test_polyline_to_polygon() {
        let pl = make_polyline();
        let poly = pl.to_polygon();
        assert_eq!(poly.len(), 4);
    }

    #[test]
    fn test_polyline_closest_point() {
        let pl = make_polyline();
        let p = Point::new(50, -20);
        let closest = pl.closest_point(&p);
        assert_eq!(closest.x, 50);
        assert_eq!(closest.y, 0);
    }
}
