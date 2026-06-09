//! Faithful 1:1 port of `AABBTreeLines.hpp` (BambuStudio / libslic3r).
//!
//! This is a header-only set of templates in C++. The primary instantiation used
//! across libslic3r is `LineType = Line` whose `Scalar` is `coord_t` (`i64`), so the
//! `Floating` type resolves to `double` (`f64`) per the C++ `std::conditional`.
//!
//! The C++ relies on `AABBTreeIndirect::Tree<2, Scalar>` and its
//! `detail::squared_distance_to_indexed_primitives_recursive` /
//! `detail::indexed_primitives_within_distance_squared_recurisve` helpers. The Rust
//! `aabb_tree_indirect::Tree` is specialized to 3D (`Point3F`/`BoundingBox3F`) and does
//! not expose those two recursive helpers, so to keep this port faithful and 2D-correct
//! we mirror that tree design (implicit balanced array, `idx*2+1` children, NPOS/INNER
//! sentinels, identical QuickSelect partitioning) inline here as a 2D specialization.
//!
//! C++ Reference: AABBTreeLines.hpp

use crate::geometry::{Line, Point, PointF};

// ===========================================================================
// 2D AABBTreeIndirect specialization (Tree<2, coord_t>)
//
// Mirrors AABBTreeIndirect.hpp's implicit balanced tree, but in 2D and tailored
// to the integer (`coord_t`) line scalar used by AABBTreeLines. Bounding boxes are
// stored as `f64` since the AABBTreeLines distancers query with a `Floating` origin.
// ===========================================================================

pub mod aabb_tree_indirect_2d {
    // AABBTreeIndirect.hpp:47-52
    // Node is not used
    // AABBTreeIndirect.hpp:49
    pub const NPOS: usize = usize::MAX;
    // Inner node (not leaf)
    // AABBTreeIndirect.hpp:51
    pub const INNER: usize = usize::MAX - 1;

    /// 2D axis-aligned bounding box, stored as `f64` (`Tree<2, Scalar>::BoundingBox`).
    /// AABBTreeIndirect.hpp (Eigen AlignedBox<Scalar, 2>)
    #[derive(Debug, Clone, Copy)]
    pub struct BoundingBox {
        pub min: [f64; 2],
        pub max: [f64; 2],
    }

    impl BoundingBox {
        pub fn new_empty() -> Self {
            // Eigen AlignedBox default: min = +inf, max = -inf
            Self {
                min: [f64::INFINITY, f64::INFINITY],
                max: [f64::NEG_INFINITY, f64::NEG_INFINITY],
            }
        }

        /// Construct a degenerate box at a single point (Eigen `BoundingBox(p, p)`).
        pub fn from_point(p: [f64; 2]) -> Self {
            Self { min: p, max: p }
        }

        /// Eigen `AlignedBox::extend(point)`.
        pub fn extend_point(&mut self, p: [f64; 2]) {
            for i in 0..2 {
                if p[i] < self.min[i] {
                    self.min[i] = p[i];
                }
                if p[i] > self.max[i] {
                    self.max[i] = p[i];
                }
            }
        }

        /// Eigen `AlignedBox::extend(box)`.
        pub fn extend(&mut self, other: &BoundingBox) {
            for i in 0..2 {
                if other.min[i] < self.min[i] {
                    self.min[i] = other.min[i];
                }
                if other.max[i] > self.max[i] {
                    self.max[i] = other.max[i];
                }
            }
        }

        /// Eigen `AlignedBox::sizes()`.
        pub fn size(&self) -> [f64; 2] {
            [self.max[0] - self.min[0], self.max[1] - self.min[1]]
        }

        /// Eigen `AlignedBox::contains(point)`.
        pub fn contains(&self, p: [f64; 2]) -> bool {
            self.min[0] <= p[0] && p[0] <= self.max[0] && self.min[1] <= p[1] && p[1] <= self.max[1]
        }

        /// Eigen `AlignedBox::intersects(box)`.
        pub fn intersects(&self, other: &BoundingBox) -> bool {
            self.min[0] <= other.max[0]
                && self.max[0] >= other.min[0]
                && self.min[1] <= other.max[1]
                && self.max[1] >= other.min[1]
        }

        /// Eigen `AlignedBox::squaredExteriorDistance(point)`.
        pub fn squared_exterior_distance(&self, p: [f64; 2]) -> f64 {
            let mut d = 0.0;
            for i in 0..2 {
                let aux = if p[i] < self.min[i] {
                    self.min[i] - p[i]
                } else if p[i] > self.max[i] {
                    p[i] - self.max[i]
                } else {
                    0.0
                };
                d += aux * aux;
            }
            d
        }
    }

    /// Single node of the implicit balanced AABB tree.
    /// AABBTreeIndirect.hpp:56-72
    #[derive(Debug, Clone, Copy)]
    pub struct Node {
        // Index of the external source entity, NPOS for internal nodes
        // AABBTreeIndirect.hpp:58
        pub idx: usize,
        // Bounding box around this entity, possibly with epsilons applied
        // AABBTreeIndirect.hpp:61
        pub bbox: BoundingBox,
    }

    impl Node {
        // AABBTreeIndirect.hpp:63
        pub fn is_valid(&self) -> bool {
            self.idx != NPOS
        }
        // AABBTreeIndirect.hpp:64
        pub fn is_inner(&self) -> bool {
            self.idx == INNER
        }
        // AABBTreeIndirect.hpp:65
        pub fn is_leaf(&self) -> bool {
            !self.is_inner()
        }
    }

    /// Source node abstraction used while building (AABBTreeIndirect.hpp:76-90).
    pub trait SourceNode {
        fn idx(&self) -> usize;
        fn centroid(&self) -> [f64; 2];
        fn bbox(&self) -> BoundingBox;
    }

    /// Static balanced AABB tree (AABBTreeIndirect.hpp:39-214), 2D specialization.
    #[derive(Debug, Clone)]
    pub struct Tree {
        // AABBTreeIndirect.hpp:213
        nodes: Vec<Node>,
    }

    impl Tree {
        // AABBTreeIndirect.hpp:74
        pub fn new() -> Self {
            Self { nodes: Vec::new() }
        }

        // AABBTreeIndirect.hpp:74
        pub fn clear(&mut self) {
            self.nodes.clear();
        }

        // AABBTreeIndirect.hpp:86-90
        pub fn build<S: SourceNode>(&mut self, mut input: Vec<S>) {
            self.build_modify_input(&mut input);
            input.clear();
        }

        // AABBTreeIndirect.hpp:93-102
        pub fn build_modify_input<S: SourceNode>(&mut self, input: &mut [S]) {
            if input.is_empty() {
                // AABBTreeIndirect.hpp:95
                self.clear();
            } else {
                // Allocate enough memory for a full binary tree
                // AABBTreeIndirect.hpp:98
                let capacity = next_highest_power_of_2(input.len()) * 2 - 1;
                self.nodes = vec![
                    Node {
                        idx: NPOS,
                        bbox: BoundingBox::new_empty(),
                    };
                    capacity
                ];
                // AABBTreeIndirect.hpp:99
                let last = input.len() - 1;
                self.build_recursive(input, 0, 0, last);
            }
        }

        // AABBTreeIndirect.hpp:104
        pub fn nodes(&self) -> &[Node] {
            &self.nodes
        }

        // AABBTreeIndirect.hpp:105
        pub fn node(&self, idx: usize) -> &Node {
            &self.nodes[idx]
        }

        // AABBTreeIndirect.hpp:106
        pub fn empty(&self) -> bool {
            self.nodes.is_empty()
        }

        // AABBTreeIndirect.hpp:109
        pub fn left_child_idx(idx: usize) -> usize {
            idx * 2 + 1
        }

        // AABBTreeIndirect.hpp:110
        pub fn right_child_idx(idx: usize) -> usize {
            Self::left_child_idx(idx) + 1
        }

        // AABBTreeIndirect.hpp:124-150
        fn build_recursive<S: SourceNode>(
            &mut self,
            input: &mut [S],
            node_idx: usize,
            left: usize,
            right: usize,
        ) {
            // AABBTreeIndirect.hpp:125-126
            assert!(node_idx < self.nodes.len());
            assert!(left <= right);

            // AABBTreeIndirect.hpp:128-132
            if left == right {
                self.nodes[node_idx].idx = input[left].idx();
                self.nodes[node_idx].bbox = input[left].bbox();
                return;
            }

            // Calculate bounding box of the input
            // AABBTreeIndirect.hpp:135-137
            let mut bbox = input[left].bbox();
            for i in (left + 1)..=right {
                bbox.extend(&input[i].bbox());
            }

            // Find dimension with maximum extent
            // AABBTreeIndirect.hpp:138-139
            let size = bbox.size();
            let dimension = if size[0] >= size[1] { 0 } else { 1 };

            // Partition the input to left/right pieces to produce balanced tree
            // AABBTreeIndirect.hpp:142-143
            let center = (left + right) / 2;
            self.partition_input(input, dimension, left, right, center);

            // Insert an inner node into the tree
            // AABBTreeIndirect.hpp:145-147
            self.nodes[node_idx].idx = INNER;
            self.nodes[node_idx].bbox = bbox;

            // AABBTreeIndirect.hpp:148-149
            self.build_recursive(input, node_idx * 2 + 1, left, center);
            self.build_recursive(input, node_idx * 2 + 2, center + 1, right);
        }

        // AABBTreeIndirect.hpp:157-210
        fn partition_input<S: SourceNode>(
            &self,
            input: &mut [S],
            dimension: usize,
            mut left: usize,
            mut right: usize,
            k: usize,
        ) {
            // AABBTreeIndirect.hpp:159
            while left < right {
                let center = (left + right) / 2;

                // Median-of-three pivot selection with bubble sort
                // AABBTreeIndirect.hpp:161-179
                let pivot = {
                    let left_value = input[left].centroid()[dimension];
                    let center_value = input[center].centroid()[dimension];
                    let right_value = input[right].centroid()[dimension];

                    let (left_value, mut center_value, mut right_value) =
                        if left_value > center_value {
                            input.swap(left, center);
                            (center_value, left_value, right_value)
                        } else {
                            (left_value, center_value, right_value)
                        };

                    if left_value > right_value {
                        input.swap(left, right);
                        right_value = left_value;
                    }

                    if center_value > right_value {
                        input.swap(center, right);
                        center_value = right_value;
                    }

                    let _ = right_value;
                    center_value
                };

                // AABBTreeIndirect.hpp:181-183
                if right <= left + 2 {
                    break;
                }

                // AABBTreeIndirect.hpp:184-186
                let mut i = left;
                let mut j = right - 1;

                // AABBTreeIndirect.hpp:187
                input.swap(center, j);

                // AABBTreeIndirect.hpp:189
                loop {
                    // AABBTreeIndirect.hpp:191-192
                    loop {
                        i += 1;
                        if !(input[i].centroid()[dimension] < pivot) {
                            break;
                        }
                    }
                    // AABBTreeIndirect.hpp:194-195
                    loop {
                        if j == 0 || i >= j {
                            break;
                        }
                        if !(input[j - 1].centroid()[dimension] > pivot) {
                            break;
                        }
                        j -= 1;
                    }
                    // AABBTreeIndirect.hpp:196-197
                    if i >= j {
                        break;
                    }
                    // AABBTreeIndirect.hpp:198
                    input.swap(i, j - 1);
                }

                // AABBTreeIndirect.hpp:201
                input.swap(i, right - 1);

                // AABBTreeIndirect.hpp:203-209
                if k < i {
                    right = i - 1;
                } else if k == i {
                    break;
                } else {
                    left = i + 1;
                }
            }
        }
    }

    impl Default for Tree {
        fn default() -> Self {
            Self::new()
        }
    }

    /// Wrap a 2D Slic3r own `BoundingBox` to be passed to `Tree::build()` and similar
    /// to build an AABBTree over `coord_t` 2D bounding boxes.
    /// AABBTreeIndirect.hpp:223-236 `class BoundingBoxWrapper`
    pub struct BoundingBoxWrapper {
        // AABBTreeIndirect.hpp:234
        m_idx: usize,
        // AABBTreeIndirect.hpp:235 — `Eigen::AlignedBox<coord_t, 2>` inflated by SCALED_EPSILON.
        m_bbox: BoundingBox,
    }

    impl BoundingBoxWrapper {
        // AABBTreeIndirect.hpp:226-229
        // C++: BoundingBoxWrapper(const size_t idx, const Slic3r::BoundingBox &bbox) :
        // C++:     m_idx(idx),
        // C++:     // Inflate the bounding box a bit to account for numerical issues.
        // C++:     m_bbox(bbox.min - Point(SCALED_EPSILON, SCALED_EPSILON), bbox.max + Point(SCALED_EPSILON, SCALED_EPSILON)) {}
        pub fn new(idx: usize, bbox: &crate::geometry::BoundingBox) -> Self {
            // SCALED_EPSILON = 10.0; integer Point(delta, delta) truncates toward zero -> 10.
            let eps = crate::libslic3r::SCALED_EPSILON as i64;
            Self {
                m_idx: idx,
                m_bbox: BoundingBox {
                    min: [(bbox.min.x - eps) as f64, (bbox.min.y - eps) as f64],
                    max: [(bbox.max.x + eps) as f64, (bbox.max.y + eps) as f64],
                },
            }
        }
    }

    impl SourceNode for BoundingBoxWrapper {
        // AABBTreeIndirect.hpp:230
        fn idx(&self) -> usize {
            self.m_idx
        }
        // AABBTreeIndirect.hpp:232 — ((min.cast<int64_t>() + max.cast<int64_t>()) / 2).cast<int32_t>()
        fn centroid(&self) -> [f64; 2] {
            [
                ((self.m_bbox.min[0] as i64 + self.m_bbox.max[0] as i64) / 2) as f64,
                ((self.m_bbox.min[1] as i64 + self.m_bbox.max[1] as i64) / 2) as f64,
            ]
        }
        // AABBTreeIndirect.hpp:231
        fn bbox(&self) -> BoundingBox {
            self.m_bbox
        }
    }

    /// Recursive traversal helper.
    /// AABBTreeIndirect.hpp:943-971 `detail::traverse_recurse`
    /// Returns true in case traversal should continue,
    /// returns false if traversal should stop (for example if the first hit was found).
    fn traverse_recurse<P, F>(tree: &Tree, idx: usize, pred: &mut P, callback: &mut F) -> bool
    where
        P: FnMut(&Node) -> bool,
        F: FnMut(&Node) -> bool,
    {
        // AABBTreeIndirect.hpp:949
        debug_assert!(tree.node(idx).is_valid());

        // AABBTreeIndirect.hpp:951-953
        if !pred(tree.node(idx)) {
            // Continue traversal.
            return true;
        }

        // AABBTreeIndirect.hpp:955-970
        if tree.node(idx).is_leaf() {
            // Callback returns true to continue traversal, false to stop traversal.
            // AABBTreeIndirect.hpp:957
            callback(tree.node(idx))
        } else {
            // Left / right child node index.
            // Returns true if both children allow the traversal to continue.
            // AABBTreeIndirect.hpp:968-969
            traverse_recurse(tree, Tree::left_child_idx(idx), pred, callback)
                && traverse_recurse(tree, Tree::right_child_idx(idx), pred, callback)
        }
    }

    /// Tree traversal with a predicate.
    /// AABBTreeIndirect.hpp:980-987 `traverse`
    /// Callback shall return true to continue traversal, false if it wants to stop
    /// traversal, for example if it found the answer.
    pub fn traverse<P, F>(tree: &Tree, mut pred: P, mut callback: F)
    where
        P: FnMut(&Node) -> bool,
        F: FnMut(&Node) -> bool,
    {
        // AABBTreeIndirect.hpp:983
        if tree.empty() {
            return;
        }
        // AABBTreeIndirect.hpp:985
        traverse_recurse(tree, 0, &mut pred, &mut callback);
    }

    /// Calculate next highest power of 2 (Utils.hpp).
    fn next_highest_power_of_2(mut v: usize) -> usize {
        if v == 0 {
            return 1;
        }
        v -= 1;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        v |= v >> 32;
        v + 1
    }
}

pub use aabb_tree_indirect_2d as tree2d;

// ===========================================================================
// line_alg helpers (Line.hpp) specialized to the integer Line type.
// ===========================================================================

mod line_alg {
    use crate::geometry::{Line, Point, PointF};

    /// `line_alg::distance_to_squared(line, point, nearest_point)` (Line.hpp:42-69).
    /// `point` and `nearest_point` are `Floating` (f64) since the distancer casts the
    /// query origin to `Floating`; the returned squared distance is `double`.
    pub fn distance_to_squared(line: &Line, point: PointF, nearest_point: &mut PointF) -> f64 {
        // Line.hpp:45-46
        let v = PointF::new(
            (line.b.x - line.a.x) as f64,
            (line.b.y - line.a.y) as f64,
        );
        let va = PointF::new(point.x - line.a.x as f64, point.y - line.a.y as f64);
        // Line.hpp:47
        let l2 = v.x * v.x + v.y * v.y; // avoid a sqrt
        if l2 == 0.0 {
            // a == b case
            // Line.hpp:49-51
            *nearest_point = PointF::new(line.a.x as f64, line.a.y as f64);
            return va.x * va.x + va.y * va.y;
        }
        // It falls where t = [(this-a) . (b-a)] / |b-a|^2
        // Line.hpp:56
        let t = (va.x * v.x + va.y * v.y) / l2;
        if t <= 0.0 {
            // beyond the 'a' end of the segment
            // Line.hpp:57-60
            *nearest_point = PointF::new(line.a.x as f64, line.a.y as f64);
            va.x * va.x + va.y * va.y
        } else if t >= 1.0 {
            // beyond the 'b' end of the segment
            // Line.hpp:61-64
            *nearest_point = PointF::new(line.b.x as f64, line.b.y as f64);
            let d = PointF::new(point.x - line.b.x as f64, point.y - line.b.y as f64);
            d.x * d.x + d.y * d.y
        } else {
            // Line.hpp:67-68
            *nearest_point = PointF::new(line.a.x as f64 + t * v.x, line.a.y as f64 + t * v.y);
            let d = PointF::new(t * v.x - va.x, t * v.y - va.y);
            d.x * d.x + d.y * d.y
        }
    }

    /// `line_alg::intersection(l1, l2, intersection_pt)` (Line.hpp:123-148).
    /// `Floating` is `double` for the integer `Line`; the result is cast back to `coord_t`.
    pub fn intersection(l1: &Line, l2: &Line, intersection_pt: &mut Point) -> bool {
        // Line.hpp:127-128
        let v1 = PointF::new((l1.b.x - l1.a.x) as f64, (l1.b.y - l1.a.y) as f64);
        let v2 = PointF::new((l2.b.x - l2.a.x) as f64, (l2.b.y - l2.a.y) as f64);
        // Line.hpp:129 — cross2(v1, v2)
        let denom = v1.x * v2.y - v1.y * v2.x;
        // Line.hpp:130-136 (#if 0 branch is disabled in C++; the active branch returns false)
        if denom.abs() < crate::libslic3r::EPSILON {
            return false;
        }
        // Line.hpp:137-139
        let v12 = PointF::new((l1.a.x - l2.a.x) as f64, (l1.a.y - l2.a.y) as f64);
        let nume_a = v2.x * v12.y - v2.y * v12.x; // cross2(v2, v12)
        let nume_b = v1.x * v12.y - v1.y * v12.x; // cross2(v1, v12)
        let t1 = nume_a / denom;
        let t2 = nume_b / denom;
        // Line.hpp:142 — note the C++ compares against 1.0f (float literal); use f64 here.
        if (0.0..=1.0).contains(&t1) && (0.0..=1.0).contains(&t2) {
            // Line.hpp:144 — (l1.a + t1 * v1).cast<Scalar>()
            *intersection_pt = Point::new(
                (l1.a.x as f64 + t1 * v1.x) as i64,
                (l1.a.y as f64 + t1 * v1.y) as i64,
            );
            return true;
        }
        false // not intersecting
    }
}

// ===========================================================================
// namespace detail (AABBTreeLines.hpp:15-160)
// ===========================================================================

mod detail {
    use super::tree2d;
    use super::PointF;
    use crate::geometry::{Line, Point};

    /// AABBTreeLines.hpp:17-36
    ///
    /// `LineType = Line` (Scalar = coord_t), `TreeType = tree2d::Tree`,
    /// `VectorType = Vec<2, double>` (`PointF`), so `ScalarType = double`.
    pub struct IndexedLinesDistancer<'a> {
        // AABBTreeLines.hpp:24
        pub lines: &'a [Line],
        // AABBTreeLines.hpp:25
        pub tree: &'a tree2d::Tree,
        // AABBTreeLines.hpp:27
        pub origin: PointF,
    }

    impl<'a> IndexedLinesDistancer<'a> {
        // AABBTreeLines.hpp:29-35
        #[inline]
        pub fn closest_point_to_origin(
            &self,
            primitive_index: usize,
            squared_distance: &mut f64,
        ) -> PointF {
            // AABBTreeLines.hpp:31-32
            let mut nearest_point = PointF::zero();
            let line = &self.lines[primitive_index];
            // AABBTreeLines.hpp:33 — origin cast to LineType::Scalar (coord_t) for the C++
            //   distance call. For the integer Line, the f64 distance helper takes the
            //   origin directly (Floating); see line_alg::distance_to_squared.
            *squared_distance =
                super::line_alg::distance_to_squared(line, self.origin, &mut nearest_point);
            // AABBTreeLines.hpp:34 — nearest_point.cast<ScalarType>() (already double)
            nearest_point
        }
    }

    // returns number of intersections of ray starting in ray_origin and following the specified coordinate line with lines in tree
    // first number is hits in positive direction of ray, second number hits in negative direction. returns neagtive numbers when ray_origin is
    // on some line exactly.
    // AABBTreeLines.hpp:38-116
    //
    // For `LineType = Line` the Scalar is `coord_t` (integer), so `Floating` is `double`.
    // The ray_origin here is the integer query point (`outside()` passes `point` directly).
    pub fn coordinate_aligned_ray_hit_count<const COORDINATE: usize>(
        node_idx: usize,
        tree: &tree2d::Tree,
        lines: &[Line],
        ray_origin: Point,
    ) -> (i32, i32) {
        // AABBTreeLines.hpp:47
        const fn other(c: usize) -> usize {
            (c + 1) % 2
        }
        let other_coordinate = other(COORDINATE);
        // AABBTreeLines.hpp:50
        let node = tree.node(node_idx);
        // AABBTreeLines.hpp:51
        assert!(node.is_valid());
        if node.is_leaf() {
            // AABBTreeLines.hpp:53
            let line = &lines[node.idx];
            let a = [line.a.x, line.a.y];
            let b = [line.b.x, line.b.y];
            let ro = [ray_origin.x, ray_origin.y];
            // AABBTreeLines.hpp:54-59
            if ro[other_coordinate] < a[other_coordinate].min(b[other_coordinate])
                || ro[other_coordinate] >= a[other_coordinate].max(b[other_coordinate])
            {
                // the second inequality is nonsharp for a reason
                //  without it, we may count contour border twice when the lines meet exactly at the spot of intersection. this prevents is
                return (0, 0);
            }

            // AABBTreeLines.hpp:61-62
            let line_max = a[COORDINATE].max(b[COORDINATE]);
            let line_min = a[COORDINATE].min(b[COORDINATE]);
            if ro[COORDINATE] > line_max {
                // AABBTreeLines.hpp:63-64
                (1, 0)
            } else if ro[COORDINATE] < line_min {
                // AABBTreeLines.hpp:65-66
                (0, 1)
            } else {
                // find intersection of ray with line
                //  that is when ( line.a + t * (line.b - line.a) )[other_coordinate] == ray_origin[other_coordinate]
                //  t = ray_origin[oc] - line.a[oc] / (line.b[oc] - line.a[oc]);
                //  then we want to get value of intersection[ coordinate]
                //  val_c = line.a[c] + t * (line.b[c] - line.a[c]);
                //  Note that ray and line may overlap, when  (line.b[oc] - line.a[oc]) is zero
                //  In that case, we return negative number
                // AABBTreeLines.hpp:75-77 (Floating = double)
                let distance_oc = (b[other_coordinate] - a[other_coordinate]) as f64;
                let t = (ro[other_coordinate] - a[other_coordinate]) as f64 / distance_oc;
                let val_c = a[COORDINATE] as f64 + t * (b[COORDINATE] - a[COORDINATE]) as f64;
                if (ro[COORDINATE] as f64) > val_c {
                    // AABBTreeLines.hpp:78-79
                    (1, 0)
                } else if (ro[COORDINATE] as f64) < val_c {
                    // AABBTreeLines.hpp:80-81
                    (0, 1)
                } else {
                    // ray origin is on boundary
                    // AABBTreeLines.hpp:82-83
                    (-1, -1)
                }
            }
        } else {
            // AABBTreeLines.hpp:87-90
            let mut intersections_above = 0;
            let mut intersections_below = 0;
            let left_node_idx = node_idx * 2 + 1;
            let right_node_idx = left_node_idx + 1;
            let node_left = tree.node(left_node_idx);
            let node_right = tree.node(right_node_idx);
            // AABBTreeLines.hpp:93-94
            assert!(node_left.is_valid());
            assert!(node_right.is_valid());

            let ro = [ray_origin.x as f64, ray_origin.y as f64];

            // AABBTreeLines.hpp:96-104
            if node_left.bbox.min[other_coordinate] <= ro[other_coordinate]
                && node_left.bbox.max[other_coordinate] >= ro[other_coordinate]
            {
                let (above, below) = coordinate_aligned_ray_hit_count::<COORDINATE>(
                    left_node_idx,
                    tree,
                    lines,
                    ray_origin,
                );
                if above < 0 || below < 0 {
                    return (-1, -1);
                }
                intersections_above += above;
                intersections_below += below;
            }

            // AABBTreeLines.hpp:106-113
            if node_right.bbox.min[other_coordinate] <= ro[other_coordinate]
                && node_right.bbox.max[other_coordinate] >= ro[other_coordinate]
            {
                let (above, below) = coordinate_aligned_ray_hit_count::<COORDINATE>(
                    right_node_idx,
                    tree,
                    lines,
                    ray_origin,
                );
                if above < 0 || below < 0 {
                    return (-1, -1);
                }
                intersections_above += above;
                intersections_below += below;
            }
            // AABBTreeLines.hpp:114
            (intersections_above, intersections_below)
        }
    }

    // AABBTreeLines.hpp:118-158
    pub fn get_intersections_with_line(
        node_idx: usize,
        tree: &tree2d::Tree,
        lines: &[Line],
        line: &Line,
        line_bb: &tree2d::BoundingBox,
    ) -> Vec<(Point, usize)> {
        // AABBTreeLines.hpp:125
        let node = tree.node(node_idx);
        // AABBTreeLines.hpp:126
        assert!(node.is_valid());
        if node.is_leaf() {
            // AABBTreeLines.hpp:128-133
            let mut intersection_pt = Point::zero();
            if super::line_alg::intersection(line, &lines[node.idx], &mut intersection_pt) {
                vec![(intersection_pt, node.idx)]
            } else {
                Vec::new()
            }
        } else {
            // AABBTreeLines.hpp:135-138
            let left_node_idx = node_idx * 2 + 1;
            let right_node_idx = left_node_idx + 1;
            let node_left = tree.node(left_node_idx);
            let node_right = tree.node(right_node_idx);
            // AABBTreeLines.hpp:139-140
            assert!(node_left.is_valid());
            assert!(node_right.is_valid());

            // AABBTreeLines.hpp:142
            let mut result: Vec<(Point, usize)> = Vec::new();

            // AABBTreeLines.hpp:144-148
            if node_left.bbox.intersects(line_bb) {
                let intersections =
                    get_intersections_with_line(left_node_idx, tree, lines, line, line_bb);
                result.extend(intersections);
            }

            // AABBTreeLines.hpp:150-154
            if node_right.bbox.intersects(line_bb) {
                let intersections =
                    get_intersections_with_line(right_node_idx, tree, lines, line, line_bb);
                result.extend(intersections);
            }

            // AABBTreeLines.hpp:156
            result
        }
    }

    /// `AABBTreeIndirect::detail::squared_distance_to_indexed_primitives_recursive`
    /// (AABBTreeIndirect.hpp:551-632), specialized to `IndexedLinesDistancer`.
    // The `looked_left`/`looked_right` flags faithfully mirror the C++ lambdas that
    // always set them after recursing; the final-branch writes are intentionally dead.
    #[allow(unused_assignments)]
    pub fn squared_distance_to_indexed_primitives_recursive(
        distancer: &IndexedLinesDistancer,
        node_idx: usize,
        low_sqr_d: f64,
        mut up_sqr_d: f64,
        i: &mut usize,
        c: &mut PointF,
    ) -> f64 {
        // AABBTreeIndirect.hpp:562-563
        if low_sqr_d > up_sqr_d {
            return low_sqr_d;
        }

        // Save the best achieved hit (AABBTreeIndirect.hpp:566-572).
        // Inlined here so the borrow of `i`/`c`/`up_sqr_d` stays sound.
        macro_rules! set_min {
            ($sqr_d_candidate:expr, $i_candidate:expr, $c_candidate:expr) => {{
                let sqr_d_candidate: f64 = $sqr_d_candidate;
                if sqr_d_candidate < up_sqr_d {
                    *i = $i_candidate;
                    *c = $c_candidate;
                    up_sqr_d = sqr_d_candidate;
                }
            }};
        }

        // AABBTreeIndirect.hpp:574-575
        let node = distancer.tree.node(node_idx);
        assert!(node.is_valid());
        if node.is_leaf() {
            // AABBTreeIndirect.hpp:578-580
            let mut sqr_dist = 0.0;
            let c_candidate = distancer.closest_point_to_origin(node.idx, &mut sqr_dist);
            set_min!(sqr_dist, node.idx, c_candidate);
        } else {
            // AABBTreeIndirect.hpp:584-589
            let left_node_idx = node_idx * 2 + 1;
            let right_node_idx = left_node_idx + 1;
            let node_left = distancer.tree.node(left_node_idx);
            let node_right = distancer.tree.node(right_node_idx);
            assert!(node_left.is_valid());
            assert!(node_right.is_valid());

            // AABBTreeIndirect.hpp:591-608
            let mut looked_left = false;
            let mut looked_right = false;
            let origin = [distancer.origin.x, distancer.origin.y];

            macro_rules! look_left {
                () => {{
                    let mut i_left = 0usize;
                    let mut c_left = *c;
                    let sqr_d_left = squared_distance_to_indexed_primitives_recursive(
                        distancer,
                        left_node_idx,
                        low_sqr_d,
                        up_sqr_d,
                        &mut i_left,
                        &mut c_left,
                    );
                    set_min!(sqr_d_left, i_left, c_left);
                    looked_left = true;
                }};
            }
            macro_rules! look_right {
                () => {{
                    let mut i_right = 0usize;
                    let mut c_right = *c;
                    let sqr_d_right = squared_distance_to_indexed_primitives_recursive(
                        distancer,
                        right_node_idx,
                        low_sqr_d,
                        up_sqr_d,
                        &mut i_right,
                        &mut c_right,
                    );
                    set_min!(sqr_d_right, i_right, c_right);
                    looked_right = true;
                }};
            }

            // must look left or right if in box
            // AABBTreeIndirect.hpp:611-615 (origin cast to BBoxScalar)
            if node_left.bbox.contains(origin) {
                look_left!();
            }
            if node_right.bbox.contains(origin) {
                look_right!();
            }
            // if haven't looked left and could be less than current min, then look
            // AABBTreeIndirect.hpp:617-629
            let left_up_sqr_d = node_left.bbox.squared_exterior_distance(origin);
            let right_up_sqr_d = node_right.bbox.squared_exterior_distance(origin);
            if left_up_sqr_d < right_up_sqr_d {
                if !looked_left && left_up_sqr_d < up_sqr_d {
                    look_left!();
                }
                if !looked_right && right_up_sqr_d < up_sqr_d {
                    look_right!();
                }
            } else {
                if !looked_right && right_up_sqr_d < up_sqr_d {
                    look_right!();
                }
                if !looked_left && left_up_sqr_d < up_sqr_d {
                    look_left!();
                }
            }
        }
        // AABBTreeIndirect.hpp:631
        up_sqr_d
    }

    /// `AABBTreeIndirect::detail::indexed_primitives_within_distance_squared_recurisve`
    /// (AABBTreeIndirect.hpp:634-663), specialized to `IndexedLinesDistancer`.
    pub fn indexed_primitives_within_distance_squared_recurisve(
        distancer: &IndexedLinesDistancer,
        node_idx: usize,
        squared_distance_limit: f64,
        found_primitives_indices: &mut Vec<usize>,
    ) {
        // AABBTreeIndirect.hpp:640-641
        let node = distancer.tree.node(node_idx);
        assert!(node.is_valid());
        if node.is_leaf() {
            // AABBTreeIndirect.hpp:643-645
            let mut sqr_dist = 0.0;
            distancer.closest_point_to_origin(node.idx, &mut sqr_dist);
            if sqr_dist < squared_distance_limit {
                found_primitives_indices.push(node.idx);
            }
        } else {
            // AABBTreeIndirect.hpp:647-652
            let left_node_idx = node_idx * 2 + 1;
            let right_node_idx = left_node_idx + 1;
            let node_left = distancer.tree.node(left_node_idx);
            let node_right = distancer.tree.node(right_node_idx);
            assert!(node_left.is_valid());
            assert!(node_right.is_valid());

            let origin = [distancer.origin.x, distancer.origin.y];

            // AABBTreeIndirect.hpp:654-661
            if node_left.bbox.squared_exterior_distance(origin) < squared_distance_limit {
                indexed_primitives_within_distance_squared_recurisve(
                    distancer,
                    left_node_idx,
                    squared_distance_limit,
                    found_primitives_indices,
                );
            }
            if node_right.bbox.squared_exterior_distance(origin) < squared_distance_limit {
                indexed_primitives_within_distance_squared_recurisve(
                    distancer,
                    right_node_idx,
                    squared_distance_limit,
                    found_primitives_indices,
                );
            }
        }
    }
}

// ===========================================================================
// AABBTreeLines free functions (AABBTreeLines.hpp:162-293)
// ===========================================================================

// Build a balanced AABB Tree over a vector of lines, balancing the tree
// on centroids of the lines.
// Epsilon is applied to the bounding boxes of the AABB Tree to cope with numeric inaccuracies
// during tree traversal.
// AABBTreeLines.hpp:166-200
pub fn build_aabb_tree_over_indexed_lines(lines: &[Line]) -> tree2d::Tree {
    // AABBTreeLines.hpp:174-183
    struct InputType {
        // AABBTreeLines.hpp:180
        m_idx: usize,
        // AABBTreeLines.hpp:181
        m_bbox: tree2d::BoundingBox,
        // AABBTreeLines.hpp:182
        m_centroid: [f64; 2],
    }

    impl tree2d::SourceNode for InputType {
        // AABBTreeLines.hpp:176
        fn idx(&self) -> usize {
            self.m_idx
        }
        // AABBTreeLines.hpp:178
        fn centroid(&self) -> [f64; 2] {
            self.m_centroid
        }
        // AABBTreeLines.hpp:177
        fn bbox(&self) -> tree2d::BoundingBox {
            self.m_bbox
        }
    }

    // AABBTreeLines.hpp:185-186
    let mut input: Vec<InputType> = Vec::with_capacity(lines.len());
    // AABBTreeLines.hpp:187-195
    for (i, line) in lines.iter().enumerate() {
        // AABBTreeLines.hpp:189-194
        // n.m_centroid = (line.a + line.b) * 0.5;
        let m_centroid = [
            (line.a.x + line.b.x) as f64 * 0.5,
            (line.a.y + line.b.y) as f64 * 0.5,
        ];
        // n.m_bbox = BoundingBox(line.a, line.a); n.m_bbox.extend(line.b);
        let mut m_bbox =
            tree2d::BoundingBox::from_point([line.a.x as f64, line.a.y as f64]);
        m_bbox.extend_point([line.b.x as f64, line.b.y as f64]);
        input.push(InputType {
            m_idx: i,
            m_bbox,
            m_centroid,
        });
    }

    // AABBTreeLines.hpp:197-199
    let mut out = tree2d::Tree::new();
    out.build(input);
    out
}

// Finding a closest line, its closest point and squared distance to the closest point
// Returns squared distance to the closest point or -1 if the input is empty.
// or no closer point than max_sq_dist
// AABBTreeLines.hpp:205-219
//
// `VectorType = Vec<2, double>` (`PointF`), so `Scalar = double`.
pub fn squared_distance_to_indexed_lines(
    lines: &[Line],
    tree: &tree2d::Tree,
    point: PointF,
    hit_idx_out: &mut usize,
    hit_point_out: &mut PointF,
    max_sqr_dist: f64,
) -> f64 {
    // AABBTreeLines.hpp:215
    if tree.empty() {
        return -1.0;
    }
    // AABBTreeLines.hpp:216
    let distancer = detail::IndexedLinesDistancer {
        lines,
        tree,
        origin: point,
    };
    // AABBTreeLines.hpp:217-218
    detail::squared_distance_to_indexed_primitives_recursive(
        &distancer,
        0,
        0.0,
        max_sqr_dist,
        hit_idx_out,
        hit_point_out,
    )
}

// Returns all lines within the given radius limit
// AABBTreeLines.hpp:222-235
pub fn all_lines_in_radius(
    lines: &[Line],
    tree: &tree2d::Tree,
    point: PointF,
    max_distance_squared: f64,
) -> Vec<usize> {
    // AABBTreeLines.hpp:228
    let distancer = detail::IndexedLinesDistancer {
        lines,
        tree,
        origin: point,
    };

    // AABBTreeLines.hpp:230
    if tree.empty() {
        return Vec::new();
    }

    // AABBTreeLines.hpp:232-234
    let mut found_lines: Vec<usize> = Vec::new();
    detail::indexed_primitives_within_distance_squared_recurisve(
        &distancer,
        0,
        max_distance_squared,
        &mut found_lines,
    );
    found_lines
}

// return 1 if true, -1 if false, 0 for point on contour (or if cannot be determined)
// AABBTreeLines.hpp:238-262
pub fn point_outside_closed_contours(lines: &[Line], tree: &tree2d::Tree, point: Point) -> i32 {
    // AABBTreeLines.hpp:241
    if tree.empty() {
        return 1;
    }

    // AABBTreeLines.hpp:243
    let (hits_above, hits_below) =
        detail::coordinate_aligned_ray_hit_count::<0>(0, tree, lines, point);
    // AABBTreeLines.hpp:244-249
    if hits_above < 0 || hits_below < 0 {
        0
    } else if hits_above % 2 == 1 && hits_below % 2 == 1 {
        -1
    } else if hits_above % 2 == 0 && hits_below % 2 == 0 {
        1
    } else {
        // this should not happen with closed contours. lets check it in Y direction
        // AABBTreeLines.hpp:251
        let (hits_above, hits_below) =
            detail::coordinate_aligned_ray_hit_count::<1>(0, tree, lines, point);
        // AABBTreeLines.hpp:252-260
        if hits_above < 0 || hits_below < 0 {
            0
        } else if hits_above % 2 == 1 && hits_below % 2 == 1 {
            -1
        } else if hits_above % 2 == 0 && hits_below % 2 == 0 {
            1
        } else {
            // both results were unclear
            0
        }
    }
}

// AABBTreeLines.hpp:264-293
//
// `VectorType = Vec<2, Scalar>` = `Point` (coord_t). `Floating = double`.
pub fn get_intersections_with_line<const SORTED: bool>(
    lines: &[Line],
    tree: &tree2d::Tree,
    line: &Line,
) -> Vec<(Point, usize)> {
    // AABBTreeLines.hpp:269-271
    if tree.empty() {
        return Vec::new();
    }
    // AABBTreeLines.hpp:272-273
    let mut line_bb = tree2d::BoundingBox::from_point([line.a.x as f64, line.a.y as f64]);
    line_bb.extend_point([line.b.x as f64, line.b.y as f64]);

    // AABBTreeLines.hpp:275
    let mut intersections = detail::get_intersections_with_line(0, tree, lines, line, &line_bb);
    // AABBTreeLines.hpp:276-290
    if SORTED {
        // Floating = double
        // AABBTreeLines.hpp:280-283
        let mut points_with_sq_distance: Vec<(f64, (Point, usize))> = Vec::new();
        for p in &intersections {
            // (p.first - line.a).cast<Floating>().squaredNorm()
            let dx = (p.0.x - line.a.x) as f64;
            let dy = (p.0.y - line.a.y) as f64;
            points_with_sq_distance.push((dx * dx + dy * dy, *p));
        }
        // AABBTreeLines.hpp:284-286
        points_with_sq_distance.sort_by(|left, right| left.0.partial_cmp(&right.0).unwrap());
        // AABBTreeLines.hpp:287-289
        for (idx, item) in points_with_sq_distance.iter().enumerate() {
            intersections[idx] = item.1;
        }
    }

    // AABBTreeLines.hpp:292
    intersections
}

/// `LinesDistancer<LineType>` (AABBTreeLines.hpp:295-360), instantiated for
/// `LineType = Line` (Scalar = coord_t, Floating = double).
pub struct LinesDistancer {
    // AABBTreeLines.hpp:301
    lines: Vec<Line>,
    // AABBTreeLines.hpp:302
    tree: tree2d::Tree,
}

impl LinesDistancer {
    // AABBTreeLines.hpp:305-313 (the const-ref and rvalue ctors coincide once `lines`
    // is owned by the struct).
    pub fn new(lines: Vec<Line>) -> Self {
        // AABBTreeLines.hpp:307 / :312
        let tree = build_aabb_tree_over_indexed_lines(&lines);
        Self { lines, tree }
    }

    // AABBTreeLines.hpp:315
    pub fn default() -> Self {
        Self {
            lines: Vec::new(),
            tree: tree2d::Tree::new(),
        }
    }

    // 1 true, -1 false, 0 cannot determine
    // AABBTreeLines.hpp:318
    pub fn outside(&self, point: Point) -> i32 {
        point_outside_closed_contours(&self.lines, &self.tree, point)
    }

    // negative sign means inside
    // AABBTreeLines.hpp:321-339
    pub fn distance_from_lines_extra<const SIGNED_DISTANCE: bool>(
        &self,
        point: Point,
    ) -> (f64, usize, PointF) {
        // AABBTreeLines.hpp:324-326
        let mut nearest_line_index_out: usize = usize::MAX; // size_t(-1)
        let mut nearest_point_out = PointF::zero();
        // AABBTreeLines.hpp:326 — p = point.cast<Floating>()
        let p = PointF::new(point.x as f64, point.y as f64);
        // AABBTreeLines.hpp:327
        let mut distance = squared_distance_to_indexed_lines(
            &self.lines,
            &self.tree,
            p,
            &mut nearest_line_index_out,
            &mut nearest_point_out,
            f64::INFINITY,
        );

        // AABBTreeLines.hpp:329-331
        if distance < 0.0 {
            return (f64::INFINITY, nearest_line_index_out, nearest_point_out);
        }
        // AABBTreeLines.hpp:332
        distance = distance.sqrt();

        // AABBTreeLines.hpp:334-336
        if SIGNED_DISTANCE {
            distance *= self.outside(point) as f64;
        }

        // AABBTreeLines.hpp:338
        (distance, nearest_line_index_out, nearest_point_out)
    }

    // AABBTreeLines.hpp:341-345
    pub fn distance_from_lines<const SIGNED_DISTANCE: bool>(&self, point: Point) -> f64 {
        // AABBTreeLines.hpp:343-344
        let (dist, _idx, _np) = self.distance_from_lines_extra::<SIGNED_DISTANCE>(point);
        dist
    }

    // AABBTreeLines.hpp:347-350
    pub fn all_lines_in_radius(&self, point: Point, radius: f64) -> Vec<usize> {
        // AABBTreeLines.hpp:349 — point cast to Floating; radius * radius
        let p = PointF::new(point.x as f64, point.y as f64);
        all_lines_in_radius(&self.lines, &self.tree, p, radius * radius)
    }

    // AABBTreeLines.hpp:352-355
    pub fn intersections_with_line<const SORTED: bool>(&self, line: &Line) -> Vec<(Point, usize)> {
        get_intersections_with_line::<SORTED>(&self.lines, &self.tree, line)
    }

    // AABBTreeLines.hpp:357
    pub fn get_line(&self, line_idx: usize) -> &Line {
        &self.lines[line_idx]
    }

    // AABBTreeLines.hpp:359
    pub fn get_lines(&self) -> &[Line] {
        &self.lines
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_build_and_distance() {
        // A unit square contour as 4 lines.
        let lines = vec![
            Line::new(Point::new(0, 0), Point::new(100, 0)),
            Line::new(Point::new(100, 0), Point::new(100, 100)),
            Line::new(Point::new(100, 100), Point::new(0, 100)),
            Line::new(Point::new(0, 100), Point::new(0, 0)),
        ];
        let dist = LinesDistancer::new(lines);

        // Point at the center: distance to nearest edge is 50.
        let (d, _idx, _np) = dist.distance_from_lines_extra::<false>(Point::new(50, 50));
        assert!((d - 50.0).abs() < 1e-6, "got {d}");
    }

    #[test]
    fn test_outside_inside() {
        let lines = vec![
            Line::new(Point::new(0, 0), Point::new(100, 0)),
            Line::new(Point::new(100, 0), Point::new(100, 100)),
            Line::new(Point::new(100, 100), Point::new(0, 100)),
            Line::new(Point::new(0, 100), Point::new(0, 0)),
        ];
        let dist = LinesDistancer::new(lines);

        // -1 means inside.
        assert_eq!(dist.outside(Point::new(50, 50)), -1);
        // 1 means outside.
        assert_eq!(dist.outside(Point::new(200, 200)), 1);
    }

    #[test]
    fn test_intersections_with_line() {
        let lines = vec![
            Line::new(Point::new(0, 0), Point::new(0, 100)),
            Line::new(Point::new(100, 0), Point::new(100, 100)),
        ];
        let dist = LinesDistancer::new(lines);

        // Horizontal line at y=50 crosses both vertical lines.
        let probe = Line::new(Point::new(-10, 50), Point::new(110, 50));
        let hits = dist.intersections_with_line::<true>(&probe);
        assert_eq!(hits.len(), 2);
        // Sorted by distance from probe.a: first hit near x=0, then x=100.
        assert!(hits[0].0.x <= hits[1].0.x);
    }

    #[test]
    fn test_all_lines_in_radius() {
        let lines = vec![
            Line::new(Point::new(0, 0), Point::new(100, 0)),
            Line::new(Point::new(0, 1000), Point::new(100, 1000)),
        ];
        let dist = LinesDistancer::new(lines);
        // Near the first line only.
        let found = dist.all_lines_in_radius(Point::new(50, 5), 10.0);
        assert_eq!(found, vec![0]);
    }
}
