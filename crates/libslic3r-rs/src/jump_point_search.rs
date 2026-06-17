//! Jump Point Search (JPS) pathfinding algorithm.
//!
//! Faithful 1:1 line-by-line port of `libslic3r/JumpPointSearch.cpp` and
//! `JumpPointSearch.hpp` from BambuStudio.
//!
//! `coord_t` -> `i64` (`Coord`), `coordf_t` -> `f64` (`CoordF`).
//! `using Pixel = Point;` -> [`Pixel`] is a type alias for the crate's integer
//! [`Point`]. The C++ `std::unordered_set<Pixel, PointHash>` becomes a
//! `HashSet<Point>` (the derived `Hash` provides identical membership
//! semantics, which is all the algorithm relies on).
//!
//! The C++ JPS tracer is templated and consumed by the header-only
//! `Slic3r::astar` search; here it is the [`JpsTracer`] which implements the
//! ported [`crate::a_star::TracerTraits`].

// JumpPointSearch.cpp:1-9
use crate::a_star;
use crate::geometry::{BoundingBox, Line, Lines, Point, Points, Polygon, Polyline, Vec2d};
use crate::kd_tree_indirect::{find_closest_point, KDTreeIndirect};
use crate::{scaled, Coord, CoordF};
// JumpPointSearch.hpp:11-12
use std::collections::{HashMap, HashSet};

// JumpPointSearch.hpp:18 (using Pixel = Point)
/// `using Pixel = Point;`
pub type Pixel = Point;

// JumpPointSearch.cpp:31
// execute fn for each pixel on the line. If fn returns false, terminate the iteration
// JumpPointSearch.cpp:32
fn dda<PointFn>(x0: Coord, y0: Coord, x1: Coord, y1: Coord, mut f_n: PointFn)
where
    PointFn: FnMut(Coord, Coord) -> bool,
{
    // JumpPointSearch.cpp:34
    let mut dx: Coord = (x1 - x0).abs();
    // JumpPointSearch.cpp:35
    let mut dy: Coord = (y1 - y0).abs();
    // JumpPointSearch.cpp:36
    let mut x: Coord = x0;
    // JumpPointSearch.cpp:37
    let mut y: Coord = y0;
    // JumpPointSearch.cpp:38
    let n: Coord = 1 + dx + dy;
    // JumpPointSearch.cpp:39
    let x_inc: Coord = if x1 > x0 { 1 } else { -1 };
    // JumpPointSearch.cpp:40
    let y_inc: Coord = if y1 > y0 { 1 } else { -1 };
    // JumpPointSearch.cpp:41
    let mut error: Coord = dx - dy;
    // JumpPointSearch.cpp:42
    dx *= 2;
    // JumpPointSearch.cpp:43
    dy *= 2;

    // JumpPointSearch.cpp:45
    let mut n = n;
    while n > 0 {
        // JumpPointSearch.cpp:46
        if !f_n(x, y) {
            return;
        }

        // JumpPointSearch.cpp:48
        if error > 0 {
            // JumpPointSearch.cpp:49
            x += x_inc;
            // JumpPointSearch.cpp:50
            error -= dy;
        } else {
            // JumpPointSearch.cpp:52
            y += y_inc;
            // JumpPointSearch.cpp:53
            error += dx;
        }
        n -= 1;
    }
}

// JumpPointSearch.cpp:58-59
// will draw the line twice, second time with and offset of 1 in the direction of normal
// may call the fn on the same coordiantes multiple times!
// JumpPointSearch.cpp:60
fn double_dda_with_offset<PointFn>(x0: Coord, y0: Coord, x1: Coord, y1: Coord, mut f_n: PointFn)
where
    PointFn: FnMut(Coord, Coord) -> bool,
{
    // JumpPointSearch.cpp:62
    // Vec2d normal = Point{y1 - y0, x1 - x0}.cast<double>().normalized();
    // `.cast<double>()` is a raw integer->double cast (NOT an unscale), so the
    // f64 components are the integer coordinate values directly.
    //
    // FIDELITY-NOTE: degenerate line (x0==x1 && y0==y1) yields a zero vector;
    // Eigen `.normalized()` then produces NaN/NaN, `ceil(NaN)==NaN`, and the
    // subsequent `NaN.cast<coord_t>()` is undefined behavior in C++ (in practice
    // INT_MIN on x86). Rust's `NaN as Coord` saturates to 0, so the offset line
    // differs for zero-length obstacles. This is an unreachable/benign edge
    // (obstacle Lines come from polygon edges and pixelized segments, never
    // degenerate) and the float math above is otherwise bit-faithful.
    let mut normal: Vec2d = {
        let vx = (y1 - y0) as CoordF;
        let vy = (x1 - x0) as CoordF;
        let norm = (vx * vx + vy * vy).sqrt();
        Vec2d::new(vx / norm, vy / norm)
    };
    // JumpPointSearch.cpp:63
    normal.x = normal.x.ceil();
    // JumpPointSearch.cpp:64
    normal.y = normal.y.ceil();
    // JumpPointSearch.cpp:65
    // Point start_offset = Point(x0, y0) + (normal).cast<coord_t>();
    let start_offset: Point = Point::new(x0, y0) + Point::new(normal.x as Coord, normal.y as Coord);
    // JumpPointSearch.cpp:66
    // Point end_offset   = Point(x1, y1) + (normal).cast<coord_t>();
    let end_offset: Point = Point::new(x1, y1) + Point::new(normal.x as Coord, normal.y as Coord);

    // JumpPointSearch.cpp:68
    dda(x0, y0, x1, y1, &mut fn_adapter(&mut f_n));
    // JumpPointSearch.cpp:69
    dda(
        start_offset.x,
        start_offset.y,
        end_offset.x,
        end_offset.y,
        &mut fn_adapter(&mut f_n),
    );
}

// Helper to allow passing the same `&mut FnMut` callback to `dda` twice in
// `double_dda_with_offset` (the C++ passes the same `const PointFn&` to both
// `dda` calls; here we reborrow).
#[inline]
fn fn_adapter<'a, F>(f: &'a mut F) -> impl FnMut(Coord, Coord) -> bool + 'a
where
    F: FnMut(Coord, Coord) -> bool,
{
    move |x, y| f(x, y)
}

// JumpPointSearch.cpp:72-175
// template<typename CellPositionType, typename CellQueryFn> class JPSTracer
//
// Specialized here for CellPositionType = Pixel (== Point). CellQueryFn is the
// `cell_query` closure type, captured as a generic `Q: Fn(Pixel) -> bool`.
/// `JPSTracer<CellPositionType, CellQueryFn>`
pub struct JpsTracer<Q>
where
    Q: Fn(Pixel) -> bool,
{
    // JumpPointSearch.cpp:85
    target: Pixel,
    // JumpPointSearch.cpp:86
    // should return boolean whether the cell is passable or not
    is_passable: Q,
    // JumpPointSearch.cpp:174
    all_directions: Vec<Pixel>,
}

// JumpPointSearch.cpp:75-80
// Use incoming_dir [0,0] for starting points, so that all directions are checked from that point
/// `JPSTracer::Node`
// `Default` is required by `astar::search_route`'s `T::Node: Default` bound: the
// C++ `cached_nodes[succ_id]` (AStar.hpp:117) default-constructs `Node{}` when
// inserting stub entries. `Pixel` (= `Point`) derives `Default` (origin), so the
// default `Node` is `{position: (0,0), incoming_dir: (0,0)}`, matching C++ `Node{}`.
#[derive(Clone, Default)]
pub struct Node {
    // JumpPointSearch.cpp:78
    pub position: Pixel,
    // JumpPointSearch.cpp:79
    pub incoming_dir: Pixel,
}

impl<Q> JpsTracer<Q>
where
    Q: Fn(Pixel) -> bool,
{
    // JumpPointSearch.cpp:82
    // JPSTracer(CellPositionType target, CellQueryFn is_passable) : target(target), is_passable(is_passable) {}
    pub fn new(target: Pixel, is_passable: Q) -> Self {
        Self {
            target,
            is_passable,
            // JumpPointSearch.cpp:174
            // const std::vector<CellPositionType> all_directions{{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};
            all_directions: vec![
                Pixel::new(1, 0),
                Pixel::new(1, 1),
                Pixel::new(0, 1),
                Pixel::new(-1, 1),
                Pixel::new(-1, 0),
                Pixel::new(-1, -1),
                Pixel::new(0, -1),
                Pixel::new(1, -1),
            ],
        }
    }

    // JumpPointSearch.cpp:88
    fn find_jump_point(&self, start: Pixel, forward_dir: Pixel) -> Pixel {
        // JumpPointSearch.cpp:90
        let mut next: Pixel = start + forward_dir;
        // JumpPointSearch.cpp:91
        while next != self.target && (self.is_passable)(next) && !self.is_jump_point(next, forward_dir) {
            // JumpPointSearch.cpp:91
            next = next + forward_dir;
        }

        // JumpPointSearch.cpp:93
        if (self.is_passable)(next) {
            // JumpPointSearch.cpp:94
            next
        } else {
            // JumpPointSearch.cpp:96
            start
        }
    }

    // JumpPointSearch.cpp:100
    fn is_jump_point(&self, pos: Pixel, forward_dir: Pixel) -> bool {
        // JumpPointSearch.cpp:102
        if forward_dir.x.abs() + forward_dir.y.abs() == 2 {
            // JumpPointSearch.cpp:103
            // diagonal
            // JumpPointSearch.cpp:104
            let horizontal_check_dir: Pixel = Pixel::new(forward_dir.x, 0);
            // JumpPointSearch.cpp:105
            let vertical_check_dir: Pixel = Pixel::new(0, forward_dir.y);

            // JumpPointSearch.cpp:107
            if !(self.is_passable)(pos - horizontal_check_dir)
                && (self.is_passable)(pos + forward_dir - horizontal_check_dir * 2)
            {
                return true;
            }

            // JumpPointSearch.cpp:109
            if !(self.is_passable)(pos - vertical_check_dir)
                && (self.is_passable)(pos + forward_dir - vertical_check_dir * 2)
            {
                return true;
            }

            // JumpPointSearch.cpp:111
            if self.find_jump_point(pos, horizontal_check_dir) != pos {
                return true;
            }

            // JumpPointSearch.cpp:113
            if self.find_jump_point(pos, vertical_check_dir) != pos {
                return true;
            }

            // JumpPointSearch.cpp:115
            false
        } else {
            // JumpPointSearch.cpp:116
            // horizontal or vertical
            // JumpPointSearch.cpp:117
            let side_dir: Pixel = Pixel::new(forward_dir.y, forward_dir.x);

            // JumpPointSearch.cpp:119
            if !(self.is_passable)(pos + side_dir) && (self.is_passable)(pos + forward_dir + side_dir) {
                return true;
            }

            // JumpPointSearch.cpp:121
            if !(self.is_passable)(pos - side_dir) && (self.is_passable)(pos + forward_dir - side_dir) {
                return true;
            }

            // JumpPointSearch.cpp:123
            false
        }
    }
}

// JumpPointSearch.cpp:127-172
// public methods that hook into the astar search via TracerTraits.
impl<Q> a_star::TracerTraits for JpsTracer<Q>
where
    Q: Fn(Pixel) -> bool,
{
    type Node = Node;

    // JumpPointSearch.cpp:128
    // template<class Fn> void foreach_reachable(const Node &from, Fn &&fn) const
    fn foreach_reachable<F>(&self, from: &Self::Node, mut fn_cb: F)
    where
        F: FnMut(&Self::Node) -> bool,
    {
        // JumpPointSearch.cpp:130
        let pos: Pixel = from.position;
        // JumpPointSearch.cpp:131
        let forward_dir: Pixel = from.incoming_dir;
        // JumpPointSearch.cpp:132
        let mut dirs_to_check: Vec<Pixel> = Vec::new();

        // JumpPointSearch.cpp:134
        if forward_dir.x.abs() + forward_dir.y.abs() == 0 {
            // JumpPointSearch.cpp:134
            // special case for starting point
            // JumpPointSearch.cpp:135
            dirs_to_check = self.all_directions.clone();
        } else if forward_dir.x.abs() + forward_dir.y.abs() == 2 {
            // JumpPointSearch.cpp:137
            // diagonal
            // JumpPointSearch.cpp:138
            let horizontal_check_dir: Pixel = Pixel::new(forward_dir.x, 0);
            // JumpPointSearch.cpp:139
            let vertical_check_dir: Pixel = Pixel::new(0, forward_dir.y);

            // JumpPointSearch.cpp:141
            if !(self.is_passable)(pos - horizontal_check_dir)
                && (self.is_passable)(pos + forward_dir - horizontal_check_dir * 2)
            {
                // JumpPointSearch.cpp:142
                dirs_to_check.push(forward_dir - horizontal_check_dir * 2);
            }

            // JumpPointSearch.cpp:145
            if !(self.is_passable)(pos - vertical_check_dir)
                && (self.is_passable)(pos + forward_dir - vertical_check_dir * 2)
            {
                // JumpPointSearch.cpp:146
                dirs_to_check.push(forward_dir - vertical_check_dir * 2);
            }

            // JumpPointSearch.cpp:149
            dirs_to_check.push(horizontal_check_dir);
            // JumpPointSearch.cpp:150
            dirs_to_check.push(vertical_check_dir);
            // JumpPointSearch.cpp:151
            dirs_to_check.push(forward_dir);
        } else {
            // JumpPointSearch.cpp:153
            // horizontal or vertical
            // JumpPointSearch.cpp:154
            let side_dir: Pixel = Pixel::new(forward_dir.y, forward_dir.x);

            // JumpPointSearch.cpp:156
            if !(self.is_passable)(pos + side_dir) && (self.is_passable)(pos + forward_dir + side_dir) {
                // JumpPointSearch.cpp:156
                dirs_to_check.push(forward_dir + side_dir);
            }

            // JumpPointSearch.cpp:158
            if !(self.is_passable)(pos - side_dir) && (self.is_passable)(pos + forward_dir - side_dir) {
                // JumpPointSearch.cpp:158
                dirs_to_check.push(forward_dir - side_dir);
            }
            // JumpPointSearch.cpp:159
            dirs_to_check.push(forward_dir);
        }

        // JumpPointSearch.cpp:162
        for dir in &dirs_to_check {
            // JumpPointSearch.cpp:163
            let jp: Pixel = self.find_jump_point(pos, *dir);
            // JumpPointSearch.cpp:164
            if jp != pos {
                fn_cb(&Node {
                    position: jp,
                    incoming_dir: *dir,
                });
            }
        }
    }

    // JumpPointSearch.cpp:168
    // float distance(Node a, Node b) const { return (a.position - b.position).template cast<double>().norm(); }
    fn distance(&self, a: &Self::Node, b: &Self::Node) -> f32 {
        let d = a.position - b.position;
        let dx = d.x as CoordF;
        let dy = d.y as CoordF;
        (dx * dx + dy * dy).sqrt() as f32
    }

    // JumpPointSearch.cpp:170
    // float goal_heuristic(Node n) const { return n.position == target ? -1.f : (target - n.position).template cast<double>().norm(); }
    fn goal_heuristic(&self, n: &Self::Node) -> f32 {
        if n.position == self.target {
            -1.0
        } else {
            let d = self.target - n.position;
            let dx = d.x as CoordF;
            let dy = d.y as CoordF;
            (dx * dx + dy * dy).sqrt() as f32
        }
    }

    // JumpPointSearch.cpp:172
    // size_t unique_id(Node n) const { return (static_cast<size_t>(uint16_t(n.position.x())) << 16) + static_cast<size_t>(uint16_t(n.position.y())); }
    fn unique_id(&self, n: &Self::Node) -> usize {
        ((n.position.x as u16 as usize) << 16) + (n.position.y as u16 as usize)
    }
}

// JumpPointSearch.hpp:16-34
/// `class JPSPathFinder`
pub struct JPSPathFinder {
    // JumpPointSearch.hpp:19
    // std::unordered_set<Pixel, PointHash> inpassable;
    inpassable: HashSet<Pixel>,
    // JumpPointSearch.hpp:20
    #[allow(dead_code)]
    print_z: CoordF,
    // JumpPointSearch.hpp:21
    max_search_box: BoundingBox,
    // JumpPointSearch.hpp:22
    bed_shape: Lines,
    // JumpPointSearch.hpp:24
    // const coord_t resolution = scaled(1.5);
    resolution: Coord,
}

// JumpPointSearch.hpp:29 (JPSPathFinder() = default;)
impl Default for JPSPathFinder {
    fn default() -> Self {
        Self {
            // JumpPointSearch.hpp:19
            inpassable: HashSet::new(),
            // JumpPointSearch.hpp:20
            print_z: 0.0,
            // JumpPointSearch.hpp:21
            max_search_box: BoundingBox::new(),
            // JumpPointSearch.hpp:22
            bed_shape: Lines::new(),
            // JumpPointSearch.hpp:24
            resolution: scaled(1.5),
        }
    }
}

impl JPSPathFinder {
    // JumpPointSearch.hpp:29
    pub fn new() -> Self {
        Self::default()
    }

    // JumpPointSearch.hpp:25
    // Pixel pixelize(const Point &p) { return p / resolution; }
    fn pixelize(&self, p: Point) -> Pixel {
        p / self.resolution
    }

    // JumpPointSearch.hpp:26
    // Point unpixelize(const Pixel &p) { return p * resolution; }
    fn unpixelize(&self, p: Pixel) -> Point {
        p * self.resolution
    }

    // JumpPointSearch.hpp:30
    // void init_bed_shape(const Points &bed_shape) { this->bed_shape = (to_lines(Polygon{bed_shape})); };
    pub fn init_bed_shape(&mut self, bed_shape: &Points) {
        self.bed_shape = to_lines(&Polygon::from_points(bed_shape.clone()));
    }

    // JumpPointSearch.cpp:177
    pub fn clear(&mut self) {
        // JumpPointSearch.cpp:179
        self.inpassable.clear();
        // JumpPointSearch.cpp:180
        self.max_search_box.max = Pixel::new(Coord::MIN, Coord::MIN);
        // JumpPointSearch.cpp:181
        self.max_search_box.min = Pixel::new(Coord::MAX, Coord::MAX);
        // JumpPointSearch.cpp:182
        let bed_shape = self.bed_shape.clone();
        self.add_obstacles(&bed_shape);
    }

    // JumpPointSearch.cpp:185
    pub fn add_obstacles(&mut self, obstacles: &Lines) {
        // JumpPointSearch.cpp:187
        // auto store_obstacle = [&](coord_t x, coord_t y) { ... };
        //
        // The `store_obstacle` closure mutates `max_search_box` and `inpassable`.
        // To keep the borrow checker happy while mutating `self`, the closure is
        // materialized inline below, capturing local references to the two fields.
        for l in obstacles {
            // JumpPointSearch.cpp:197
            let start: Pixel = self.pixelize(l.a);
            // JumpPointSearch.cpp:198
            let end: Pixel = self.pixelize(l.b);
            // JumpPointSearch.cpp:199
            let max_search_box = &mut self.max_search_box;
            let inpassable = &mut self.inpassable;
            double_dda_with_offset(start.x, start.y, end.x, end.y, |x, y| {
                // JumpPointSearch.cpp:188
                max_search_box.max.x = max_search_box.max.x.max(x);
                // JumpPointSearch.cpp:189
                max_search_box.max.y = max_search_box.max.y.max(y);
                // JumpPointSearch.cpp:190
                max_search_box.min.x = max_search_box.min.x.min(x);
                // JumpPointSearch.cpp:191
                max_search_box.min.y = max_search_box.min.y.min(y);
                // JumpPointSearch.cpp:192
                inpassable.insert(Pixel::new(x, y));
                // JumpPointSearch.cpp:193
                true
            });
        }
    }

    // JumpPointSearch.cpp:203
    pub fn find_path(&self, p0: &Point, p1: &Point) -> Polyline {
        // JumpPointSearch.cpp:205
        let mut start: Pixel = self.pixelize(*p0);
        // JumpPointSearch.cpp:206
        let mut end: Pixel = self.pixelize(*p1);
        // JumpPointSearch.cpp:207
        // if (inpassable.empty() || (start - end).cast<float>().norm() < 3.0) { return Polyline{p0, p1}; }
        if self.inpassable.is_empty() || {
            let d = start - end;
            let dx = d.x as f32;
            let dy = d.y as f32;
            (dx * dx + dy * dy).sqrt() < 3.0
        } {
            // JumpPointSearch.cpp:207
            return Polyline::from_points(vec![*p0, *p1]);
        }

        // JumpPointSearch.cpp:209
        if self.inpassable.contains(&start) {
            // JumpPointSearch.cpp:210
            dda(start.x, start.y, end.x, end.y, |x, y| {
                // JumpPointSearch.cpp:211
                // new start not found yet, and xy passable
                if !self.inpassable.contains(&Pixel::new(x, y)) || start == end {
                    // JumpPointSearch.cpp:212
                    start = Pixel::new(x, y);
                    // JumpPointSearch.cpp:213
                    return false;
                }
                // JumpPointSearch.cpp:215
                true
            });
        }

        // JumpPointSearch.cpp:219
        if self.inpassable.contains(&end) {
            // JumpPointSearch.cpp:220
            dda(end.x, end.y, start.x, start.y, |x, y| {
                // JumpPointSearch.cpp:221
                // new start not found yet, and xy passable
                if !self.inpassable.contains(&Pixel::new(x, y)) || start == end {
                    // JumpPointSearch.cpp:222
                    end = Pixel::new(x, y);
                    // JumpPointSearch.cpp:223
                    return false;
                }
                // JumpPointSearch.cpp:225
                true
            });
        }

        // JumpPointSearch.cpp:229
        let mut search_box: BoundingBox = self.max_search_box;
        // JumpPointSearch.cpp:230
        search_box.max -= Pixel::new(1, 1);
        // JumpPointSearch.cpp:231
        search_box.min += Pixel::new(1, 1);

        // JumpPointSearch.cpp:233
        // BoundingBox bounding_square(Points{start, end});
        let mut bounding_square: BoundingBox = bounding_box_from_points(&[start, end]);
        // JumpPointSearch.cpp:234
        bounding_square.max += Pixel::new(5, 5);
        // JumpPointSearch.cpp:235
        bounding_square.min -= Pixel::new(5, 5);
        // JumpPointSearch.cpp:236
        let bounding_square_size: Coord =
            2 * bounding_square.size().x.max(bounding_square.size().y);
        // JumpPointSearch.cpp:237
        bounding_square.max.x += (bounding_square_size - bounding_square.size().x) / 2;
        // JumpPointSearch.cpp:238
        bounding_square.min.x -= (bounding_square_size - bounding_square.size().x) / 2;
        // JumpPointSearch.cpp:239
        bounding_square.max.y += (bounding_square_size - bounding_square.size().y) / 2;
        // JumpPointSearch.cpp:240
        bounding_square.min.y -= (bounding_square_size - bounding_square.size().y) / 2;

        // JumpPointSearch.cpp:242-243
        // Intersection - limit the search box to a square area around the start and end, to fasten the path searching
        // search_box.max = search_box.max.cwiseMin(bounding_square.max);
        search_box.max.x = search_box.max.x.min(bounding_square.max.x);
        search_box.max.y = search_box.max.y.min(bounding_square.max.y);
        // JumpPointSearch.cpp:244
        // search_box.min = search_box.min.cwiseMax(bounding_square.min);
        search_box.min.x = search_box.min.x.max(bounding_square.min.x);
        search_box.min.y = search_box.min.y.max(bounding_square.min.y);

        // JumpPointSearch.cpp:246
        // auto cell_query = [&](Pixel pixel) { return search_box.contains(pixel) && (pixel == start || pixel == end || inpassable.find(pixel) == inpassable.end()); };
        //
        // search_box.contains(point) is the BambuStudio inclusive bounds check
        // (BoundingBox.hpp:50-53), independent of the crate's `defined` flag.
        let search_box_min = search_box.min;
        let search_box_max = search_box.max;
        let cell_query = |pixel: Pixel| -> bool {
            let contains = pixel.x >= search_box_min.x
                && pixel.x <= search_box_max.x
                && pixel.y >= search_box_min.y
                && pixel.y <= search_box_max.y;
            contains && (pixel == start || pixel == end || !self.inpassable.contains(&pixel))
        };

        // JumpPointSearch.cpp:248
        let tracer = JpsTracer::new(end, cell_query);
        // JumpPointSearch.cpp:249
        // using QNode = astar::QNode<JPSTracer<...>>;

        // JumpPointSearch.cpp:251
        let mut astar_cache: HashMap<usize, a_star::QNode<Node>> = HashMap::new();
        // JumpPointSearch.cpp:252
        let mut out_path: Vec<Pixel> = Vec::new();
        // JumpPointSearch.cpp:253
        let mut out_nodes: Vec<Node> = Vec::new();

        // JumpPointSearch.cpp:255
        // if (!astar::search_route(tracer, {start, {0, 0}}, std::back_inserter(out_nodes), astar_cache)) {
        if !a_star::search_route(
            &tracer,
            &Node {
                position: start,
                incoming_dir: Pixel::new(0, 0),
            },
            &mut out_nodes,
            &mut astar_cache,
        ) {
            // JumpPointSearch.cpp:256-257
            // path not found - just reconstruct the best path from astar cache.
            // Note that astar_cache is NOT empty - at least the starting point should always be there
            // JumpPointSearch.cpp:258
            // auto coordiante_func = [&astar_cache](size_t idx, size_t dim) { return float(astar_cache[idx].node.position[dim]); };
            //
            // C++ uses `float`; the crate's KDTreeIndirect requires `T: From<f64>`
            // so f64 is used here. The pixel coordinates are exact integer casts
            // and the squared distances are exact in both f32 and f64 for the
            // grid sizes encountered, so the closest-point selection is identical.
            let coordinate_func = |idx: usize, dim: usize| -> f64 {
                let pos = astar_cache[&idx].node.position;
                if dim == 0 {
                    pos.x as f64
                } else {
                    pos.y as f64
                }
            };
            // JumpPointSearch.cpp:259
            let mut keys: Vec<usize> = Vec::new();
            // JumpPointSearch.cpp:260
            keys.reserve(astar_cache.len());
            // JumpPointSearch.cpp:261
            for (k, _v) in astar_cache.iter() {
                keys.push(*k);
            }
            // JumpPointSearch.cpp:262
            // KDTreeIndirect<2, float, decltype(coordiante_func)> kd_tree(coordiante_func, keys);
            let kd_tree: KDTreeIndirect<2, f64, _> =
                KDTreeIndirect::with_index_vec(coordinate_func, keys);
            // JumpPointSearch.cpp:263
            // size_t closest_qnode = find_closest_point(kd_tree, end.cast<float>());
            let end_f: [f64; 2] = [end.x as f64, end.y as f64];
            let mut closest_qnode: usize = find_closest_point(&kd_tree, &end_f, |_| true);

            // JumpPointSearch.cpp:265
            out_path.push(end);
            // JumpPointSearch.cpp:266
            while closest_qnode != a_star::UNASSIGNED {
                // JumpPointSearch.cpp:267
                out_path.push(astar_cache[&closest_qnode].node.position);
                // JumpPointSearch.cpp:268
                closest_qnode = astar_cache[&closest_qnode].parent;
            }
        } else {
            // JumpPointSearch.cpp:271
            for node in &out_nodes {
                out_path.push(node.position);
            }
            // JumpPointSearch.cpp:272
            out_path.push(start);
        }

        // JumpPointSearch.cpp:291
        let mut tmp_path: Vec<Pixel> = Vec::new();
        // JumpPointSearch.cpp:292
        tmp_path.reserve(out_path.len());
        // JumpPointSearch.cpp:293-294
        // Some path found, reverse and remove points that do not change direction
        out_path.reverse();
        // JumpPointSearch.cpp:295-302
        {
            // JumpPointSearch.cpp:296
            // first point
            tmp_path.push(out_path[0]);
            // JumpPointSearch.cpp:297
            for i in 1..(out_path.len() - 1) {
                // JumpPointSearch.cpp:298
                // if ((out_path[i] - out_path[i - 1]).cast<float>().normalized() != (out_path[i + 1] - out_path[i]).cast<float>().normalized())
                if normalized_f32(out_path[i] - out_path[i - 1])
                    != normalized_f32(out_path[i + 1] - out_path[i])
                {
                    // JumpPointSearch.cpp:298
                    tmp_path.push(out_path[i]);
                }
            }
            // JumpPointSearch.cpp:300
            // last_point
            tmp_path.push(out_path[out_path.len() - 1]);
            // JumpPointSearch.cpp:301
            out_path = tmp_path;
        }

        // JumpPointSearch.cpp:308
        let mut tmp_path: Vec<Pixel> = Vec::new();
        // JumpPointSearch.cpp:309-311
        // remove redundant jump points - there are points that change direction but are not needed - this inefficiency arises from the
        // usage of grid search The removal alg tries to find the longest Px Px+k path without obstacles. If Px Px+k+1 is blocked, it will
        // insert the Px+k point to result and continue search from Px+k
        // JumpPointSearch.cpp:312-333
        {
            // JumpPointSearch.cpp:313
            // first point
            tmp_path.push(out_path[0]);
            // JumpPointSearch.cpp:314
            let mut index_of_last_stored_point: usize = 0;
            // JumpPointSearch.cpp:315
            for i in 1..out_path.len() {
                // JumpPointSearch.cpp:316
                if i - index_of_last_stored_point < 2 {
                    continue;
                }
                // JumpPointSearch.cpp:317
                let mut passable: bool = true;
                // JumpPointSearch.cpp:318
                // auto store_obstacle = [&](coord_t x, coord_t y) { ... };
                {
                    let inpassable = &self.inpassable;
                    let from = *tmp_path.last().unwrap();
                    let to = out_path[i];
                    // JumpPointSearch.cpp:325
                    dda(from.x, from.y, to.x, to.y, |x, y| {
                        // JumpPointSearch.cpp:319
                        if Pixel::new(x, y) != start
                            && Pixel::new(x, y) != end
                            && inpassable.contains(&Pixel::new(x, y))
                        {
                            // JumpPointSearch.cpp:320
                            passable = false;
                            // JumpPointSearch.cpp:321
                            return false;
                        }
                        // JumpPointSearch.cpp:323
                        true
                    });
                }
                // JumpPointSearch.cpp:326
                if !passable {
                    // JumpPointSearch.cpp:327
                    tmp_path.push(out_path[i - 1]);
                    // JumpPointSearch.cpp:328
                    index_of_last_stored_point = i - 1;
                }
            }
            // JumpPointSearch.cpp:331
            // last_point
            tmp_path.push(out_path[out_path.len() - 1]);
            // JumpPointSearch.cpp:332
            out_path = tmp_path;
        }

        // JumpPointSearch.cpp:340-342
        // before returing the path, transform it from pixels back to points.
        // Also replace the first and last pixel by input points so that result path patches input params exactly.
        for p in out_path.iter_mut() {
            *p = self.unpixelize(*p);
        }
        // JumpPointSearch.cpp:343
        let n = out_path.len();
        out_path[0] = *p0;
        // JumpPointSearch.cpp:344
        out_path[n - 1] = *p1;

        // JumpPointSearch.cpp:346
        Polyline::from_points(out_path)
    }
}

// JumpPointSearch.cpp:298 (helper)
// (a - b).cast<float>().normalized() — Eigen normalization on a float vector.
// For a zero vector Eigen yields NaN/NaN; replicated here.
#[inline]
fn normalized_f32(v: Pixel) -> (f32, f32) {
    let x = v.x as f32;
    let y = v.y as f32;
    let norm = (x * x + y * y).sqrt();
    (x / norm, y / norm)
}

// JumpPointSearch.cpp:233 (BoundingBox bounding_square(Points{start, end});)
// Construct an inclusive BoundingBox from a set of points without relying on
// the crate's `defined` flag (the JPS search box is manipulated by raw
// min/max arithmetic afterwards). Mirrors BoundingBoxBase(const Points&).
#[inline]
fn bounding_box_from_points(points: &[Point]) -> BoundingBox {
    let mut min = Point::new(Coord::MAX, Coord::MAX);
    let mut max = Point::new(Coord::MIN, Coord::MIN);
    for p in points {
        min.x = min.x.min(p.x);
        min.y = min.y.min(p.y);
        max.x = max.x.max(p.x);
        max.y = max.y.max(p.y);
    }
    BoundingBox::from_points_minmax(min, max)
}

// Polygon.hpp:199-209
// inline Lines to_lines(const Polygon &poly)
fn to_lines(poly: &Polygon) -> Lines {
    // Polygon.hpp:201
    let mut lines: Lines = Lines::new();
    // Polygon.hpp:202
    lines.reserve(poly.points.len());
    // Polygon.hpp:203
    if poly.points.len() > 2 {
        // Polygon.hpp:204-205
        for it in 0..poly.points.len() - 1 {
            lines.push(Line::new(poly.points[it], poly.points[it + 1]));
        }
        // Polygon.hpp:206
        lines.push(Line::new(
            poly.points[poly.points.len() - 1],
            poly.points[0],
        ));
    }
    // Polygon.hpp:208
    lines
}
