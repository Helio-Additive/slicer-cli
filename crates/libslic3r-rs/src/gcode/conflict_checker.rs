//! Faithful 1:1 port of `src/libslic3r/GCode/ConflictChecker.cpp` (BambuStudio)
//! and its header `GCode/ConflictChecker.hpp`.
//!
//! ConflictChecker.cpp:1 `#include "ConflictChecker.hpp"`
//! ConflictChecker.cpp:3 `#include <tbb/parallel_for.h>`
//! ConflictChecker.cpp:4 `#include <tbb/concurrent_vector.h>`
//! ConflictChecker.cpp:6 `#include <map>`
//! ConflictChecker.cpp:7 `#include <functional>`
//! ConflictChecker.cpp:8 `#include <atomic>`
//!
//! C++ uses raw `const void *` pointers for object identity (`_id`, `_obj1`,
//! `_obj2`). The Rust port preserves the identity semantics with `*const ()`
//! raw pointers, which are compared by address exactly as in C++.
//!
//! NOTE on scaling: BambuStudio's `scale_(1)` (`SCALING_FACTOR = 0.000001`)
//! yields `1000000`. This crate's `scaled`/`scale` use `SCALING_FACTOR =
//! 100000`, so to remain consistent with the rest of the ported crate we use
//! the crate's `crate::scaled` here. The G-code parity target is the crate's
//! own scaling convention.

use std::collections::BTreeMap;

use crate::extrusion_entity::{
    ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionPath, ExtrusionRole,
};
use crate::geometry::{Line, Point};
use crate::layer::Layer;
use crate::libslic3r::{EPSILON, SCALED_EPSILON};
use crate::{scaled, unscale, CoordF};

// ConflictChecker.cpp:10 `namespace Slic3r {`

// ConflictChecker.cpp:12 `namespace RasterizationImpl {`
pub mod rasterization_impl {
    use super::*;

    // ConflictChecker.cpp:13 `using IndexPair = std::pair<int64_t, int64_t>;`
    pub type IndexPair = (i64, i64);
    // ConflictChecker.cpp:14 `using Grids     = std::vector<IndexPair>;`
    pub type Grids = Vec<IndexPair>;

    // ConflictChecker.cpp:16 `inline constexpr int64_t RasteXDistance = scale_(1);`
    pub fn raste_x_distance() -> i64 {
        scaled(1.0)
    }
    // ConflictChecker.cpp:17 `inline constexpr int64_t RasteYDistance = scale_(1);`
    pub fn raste_y_distance() -> i64 {
        scaled(1.0)
    }

    // ConflictChecker.cpp:19 `inline IndexPair point_map_grid_index(const Point &pt, int64_t xdist, int64_t ydist)`
    #[inline]
    pub fn point_map_grid_index(pt: &Point, xdist: i64, ydist: i64) -> IndexPair {
        // ConflictChecker.cpp:21 `auto x = pt.x() / xdist;`
        let x = pt.x() / xdist;
        // ConflictChecker.cpp:22 `auto y = pt.y() / ydist;`
        let y = pt.y() / ydist;
        // ConflictChecker.cpp:23 `return std::make_pair(x, y);`
        (x, y)
    }

    // ConflictChecker.cpp:26 `inline bool nearly_equal(const Point &p1, const Point &p2) { ... }`
    #[inline]
    #[allow(dead_code)]
    pub fn nearly_equal(p1: &Point, p2: &Point) -> bool {
        (p1.x() - p2.x()).abs() < SCALED_EPSILON as i64
            && (p1.y() - p2.y()).abs() < SCALED_EPSILON as i64
    }

    // ConflictChecker.cpp:28 `inline Grids line_rasterization(const Line &line, int64_t xdist = RasteXDistance, int64_t ydist = RasteYDistance)`
    pub fn line_rasterization(line: &Line, xdist: i64, ydist: i64) -> Grids {
        // ConflictChecker.cpp:30 `Grids     res;`
        let mut res: Grids = Grids::new();
        // ConflictChecker.cpp:31 `Point     rayStart     = line.a;`
        let ray_start = line.a;
        // ConflictChecker.cpp:32 `Point     rayEnd       = line.b;`
        let ray_end = line.b;
        // ConflictChecker.cpp:33 `IndexPair currentVoxel = point_map_grid_index(rayStart, xdist, ydist);`
        let mut current_voxel = point_map_grid_index(&ray_start, xdist, ydist);
        // ConflictChecker.cpp:34 `IndexPair firstVoxel   = currentVoxel;`
        let _first_voxel = current_voxel;
        // ConflictChecker.cpp:35 `IndexPair lastVoxel    = point_map_grid_index(rayEnd, xdist, ydist);`
        let last_voxel = point_map_grid_index(&ray_end, xdist, ydist);

        // ConflictChecker.cpp:37 `Point ray = rayEnd - rayStart;`
        let ray = ray_end - ray_start;

        // ConflictChecker.cpp:39 `double stepX = ray.x() >= 0 ? 1 : -1;`
        let step_x: CoordF = if ray.x() >= 0 { 1.0 } else { -1.0 };
        // ConflictChecker.cpp:40 `double stepY = ray.y() >= 0 ? 1 : -1;`
        let step_y: CoordF = if ray.y() >= 0 { 1.0 } else { -1.0 };

        // ConflictChecker.cpp:42 `double nextVoxelBoundaryX = (currentVoxel.first + stepX) * xdist;`
        let mut next_voxel_boundary_x = (current_voxel.0 as CoordF + step_x) * xdist as CoordF;
        // ConflictChecker.cpp:43 `double nextVoxelBoundaryY = (currentVoxel.second + stepY) * ydist;`
        let mut next_voxel_boundary_y = (current_voxel.1 as CoordF + step_y) * ydist as CoordF;

        // ConflictChecker.cpp:45 `if (stepX < 0) { nextVoxelBoundaryX += xdist; }`
        if step_x < 0.0 {
            next_voxel_boundary_x += xdist as CoordF;
        }
        // ConflictChecker.cpp:46 `if (stepY < 0) { nextVoxelBoundaryY += ydist; }`
        if step_y < 0.0 {
            next_voxel_boundary_y += ydist as CoordF;
        }

        // ConflictChecker.cpp:48 `double tMaxX = ray.x() != 0 ? (nextVoxelBoundaryX - rayStart.x()) / ray.x() : DBL_MAX;`
        let mut t_max_x = if ray.x() != 0 {
            (next_voxel_boundary_x - ray_start.x() as CoordF) / ray.x() as CoordF
        } else {
            CoordF::MAX
        };
        // ConflictChecker.cpp:49 `double tMaxY = ray.y() != 0 ? (nextVoxelBoundaryY - rayStart.y()) / ray.y() : DBL_MAX;`
        let mut t_max_y = if ray.y() != 0 {
            (next_voxel_boundary_y - ray_start.y() as CoordF) / ray.y() as CoordF
        } else {
            CoordF::MAX
        };

        // ConflictChecker.cpp:51 `double tDeltaX = ray.x() != 0 ? static_cast<double>(xdist) / ray.x() * stepX : DBL_MAX;`
        let t_delta_x = if ray.x() != 0 {
            xdist as CoordF / ray.x() as CoordF * step_x
        } else {
            CoordF::MAX
        };
        // ConflictChecker.cpp:52 `double tDeltaY = ray.y() != 0 ? static_cast<double>(ydist) / ray.y() * stepY : DBL_MAX;`
        let t_delta_y = if ray.y() != 0 {
            ydist as CoordF / ray.y() as CoordF * step_y
        } else {
            CoordF::MAX
        };

        // ConflictChecker.cpp:54 `res.push_back(currentVoxel);`
        res.push(current_voxel);

        // ConflictChecker.cpp:56 `double tx = tMaxX;`
        let tx = &mut t_max_x;
        // ConflictChecker.cpp:57 `double ty = tMaxY;`
        let ty = &mut t_max_y;

        // ConflictChecker.cpp:59 `while (lastVoxel != currentVoxel) {`
        while last_voxel != current_voxel {
            // ConflictChecker.cpp:60 `if (lastVoxel.first == currentVoxel.first) {`
            if last_voxel.0 == current_voxel.0 {
                // ConflictChecker.cpp:61 `for (int64_t i = currentVoxel.second; i != lastVoxel.second; i += (int64_t) stepY) {`
                let mut i = current_voxel.1;
                while i != last_voxel.1 {
                    // ConflictChecker.cpp:62 `currentVoxel.second += (int64_t) stepY;`
                    current_voxel.1 += step_y as i64;
                    // ConflictChecker.cpp:63 `res.push_back(currentVoxel);`
                    res.push(current_voxel);
                    i += step_y as i64;
                }
                // ConflictChecker.cpp:65 `break;`
                break;
            }
            // ConflictChecker.cpp:67 `if (lastVoxel.second == currentVoxel.second) {`
            if last_voxel.1 == current_voxel.1 {
                // ConflictChecker.cpp:68 `for (int64_t i = currentVoxel.first; i != lastVoxel.first; i += (int64_t) stepX) {`
                let mut i = current_voxel.0;
                while i != last_voxel.0 {
                    // ConflictChecker.cpp:69 `currentVoxel.first += (int64_t) stepX;`
                    current_voxel.0 += step_x as i64;
                    // ConflictChecker.cpp:70 `res.push_back(currentVoxel);`
                    res.push(current_voxel);
                    i += step_x as i64;
                }
                // ConflictChecker.cpp:72 `break;`
                break;
            }

            // ConflictChecker.cpp:75 `if (tx < ty) {`
            if *tx < *ty {
                // ConflictChecker.cpp:76 `currentVoxel.first += (int64_t) stepX;`
                current_voxel.0 += step_x as i64;
                // ConflictChecker.cpp:77 `tx += tDeltaX;`
                *tx += t_delta_x;
            } else {
                // ConflictChecker.cpp:79 `currentVoxel.second += (int64_t) stepY;`
                current_voxel.1 += step_y as i64;
                // ConflictChecker.cpp:80 `ty += tDeltaY;`
                *ty += t_delta_y;
            }
            // ConflictChecker.cpp:82 `res.push_back(currentVoxel);`
            res.push(current_voxel);
            // ConflictChecker.cpp:83 `if (res.size() >= 100000) { // bug`
            if res.len() >= 100000 {
                // ConflictChecker.cpp:84 `assert(0);`
                debug_assert!(false);
            }
        }

        // ConflictChecker.cpp:88 `return res;`
        res
    }
} // ConflictChecker.cpp:90 `} // namespace RasterizationImpl`

// ===========================================================================
// ConflictChecker.hpp types
// ===========================================================================

// ConflictChecker.hpp:15 `struct LineWithID`
#[derive(Clone, Copy)]
pub struct LineWithID {
    // ConflictChecker.hpp:17 `Line          _line;`
    pub line: Line,
    // ConflictChecker.hpp:18 `const void *  _id;`
    pub id: *const (),
    // ConflictChecker.hpp:19 `ExtrusionRole _role;`
    pub role: ExtrusionRole,
}

impl LineWithID {
    // ConflictChecker.hpp:21 `LineWithID(const Line &line, const void* id, ExtrusionRole role) : _line(line), _id(id), _role(role) {}`
    pub fn new(line: Line, id: *const (), role: ExtrusionRole) -> Self {
        Self { line, id, role }
    }
}

// ConflictChecker.hpp:24 `using LineWithIDs = std::vector<LineWithID>;`
pub type LineWithIDs = Vec<LineWithID>;

// ConflictChecker.hpp:26 `struct ExtrusionLayer`
#[derive(Clone)]
pub struct ExtrusionLayer {
    // ConflictChecker.hpp:28 `ExtrusionPaths paths;`
    pub paths: Vec<ExtrusionPath>,
    // ConflictChecker.hpp:29 `const Layer *  layer;`
    pub layer: *const Layer,
    // ConflictChecker.hpp:30 `float          bottom_z;`
    pub bottom_z: f32,
    // ConflictChecker.hpp:31 `float          height;`
    pub height: f32,
}

impl Default for ExtrusionLayer {
    fn default() -> Self {
        Self {
            paths: Vec::new(),
            layer: std::ptr::null(),
            bottom_z: 0.0,
            height: 0.0,
        }
    }
}

// ConflictChecker.hpp:34 `enum class ExtrusionLayersType { INFILL, PERIMETERS, SUPPORT, WIPE_TOWER };`
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ExtrusionLayersType {
    Infill,
    Perimeters,
    Support,
    WipeTower,
}

// ConflictChecker.hpp:36 `class ExtrusionLayers : public std::vector<ExtrusionLayer>`
#[derive(Clone)]
pub struct ExtrusionLayers {
    // base: std::vector<ExtrusionLayer>
    pub layers: Vec<ExtrusionLayer>,
    // ConflictChecker.hpp:39 `ExtrusionLayersType type;`
    pub type_: ExtrusionLayersType,
}

impl ExtrusionLayers {
    fn new() -> Self {
        Self {
            layers: Vec::new(),
            // C++ leaves `type` default-initialized; callers always set it via
            // ObjectExtrusions or the WIPE_TOWER constructor before use.
            type_: ExtrusionLayersType::Infill,
        }
    }
}

// ConflictChecker.hpp:42 `struct ObjectExtrusions`
pub struct ObjectExtrusions {
    // ConflictChecker.hpp:44 `ExtrusionLayers perimeters;`
    pub perimeters: ExtrusionLayers,
    // ConflictChecker.hpp:45 `ExtrusionLayers support;`
    pub support: ExtrusionLayers,
}

impl Default for ObjectExtrusions {
    // ConflictChecker.hpp:47 `ObjectExtrusions()`
    fn default() -> Self {
        let mut perimeters = ExtrusionLayers::new();
        let mut support = ExtrusionLayers::new();
        // ConflictChecker.hpp:49 `perimeters.type = ExtrusionLayersType::PERIMETERS;`
        perimeters.type_ = ExtrusionLayersType::Perimeters;
        // ConflictChecker.hpp:50 `support.type    = ExtrusionLayersType::SUPPORT;`
        support.type_ = ExtrusionLayersType::Support;
        Self {
            perimeters,
            support,
        }
    }
}

// ConflictChecker.hpp:54 `class LinesBucket`
pub struct LinesBucket {
    // ConflictChecker.hpp:57 `float    _curBottomZ = 0.0;`
    pub cur_bottom_z: f32,
    // ConflictChecker.hpp:58 `unsigned _curPileIdx = 0;`
    pub cur_pile_idx: u32,

    // ConflictChecker.hpp:60 `ExtrusionLayers _piles;`
    pub piles: ExtrusionLayers,
    // ConflictChecker.hpp:61 `const void*     _id;`
    pub id: *const (),
    // ConflictChecker.hpp:62 `Point           _offset;`
    pub offset: Point,
}

impl LinesBucket {
    // ConflictChecker.hpp:65 `LinesBucket(ExtrusionLayers &&paths, const void* id, Point offset) : _piles(paths), _id(id), _offset(offset) {}`
    pub fn new(paths: ExtrusionLayers, id: *const (), offset: Point) -> Self {
        Self {
            cur_bottom_z: 0.0,
            cur_pile_idx: 0,
            piles: paths,
            id,
            offset,
        }
    }

    // ConflictChecker.hpp:68 `std::pair<int, int> curRange() const`
    pub fn cur_range(&self) -> (i32, i32) {
        // ConflictChecker.hpp:70 `auto begin = std::lower_bound(... [](l, r) { return l.bottom_z < r.bottom_z; });`
        // ConflictChecker.hpp:71 `auto end = std::upper_bound(... [](l, r) { return l.bottom_z < r.bottom_z; });`
        // Both bounds are taken against the pile at `_curPileIdx`, over the
        // bottom_z-sorted `_piles`. lower_bound = first index whose bottom_z is
        // not less than the key; upper_bound = first index whose bottom_z is
        // greater than the key.
        let key = self.piles.layers[self.cur_pile_idx as usize].bottom_z;
        let mut begin = 0usize;
        while begin < self.piles.layers.len() && self.piles.layers[begin].bottom_z < key {
            begin += 1;
        }
        let mut end = 0usize;
        while end < self.piles.layers.len() && !(key < self.piles.layers[end].bottom_z) {
            end += 1;
        }
        // ConflictChecker.hpp:72 `return std::make_pair<int, int>(std::distance(_piles.begin(), begin), std::distance(_piles.begin(), end));`
        (begin as i32, end as i32)
    }

    // ConflictChecker.hpp:74 `bool valid() const { return _curPileIdx < _piles.size(); }`
    pub fn valid(&self) -> bool {
        (self.cur_pile_idx as usize) < self.piles.layers.len()
    }

    // ConflictChecker.hpp:75 `void raise()`
    pub fn raise(&mut self) {
        // ConflictChecker.hpp:77 `if (!valid()) { return; }`
        if !self.valid() {
            return;
        }
        // ConflictChecker.hpp:78 `auto [b, e] = curRange();`
        let (b, e) = self.cur_range();
        // ConflictChecker.hpp:79 `_curPileIdx += (e - b);`
        self.cur_pile_idx = (self.cur_pile_idx as i32 + (e - b)) as u32;
        // ConflictChecker.hpp:80 `_curBottomZ = _curPileIdx == _piles.size() ? _piles.back().bottom_z : _piles[_curPileIdx].bottom_z;`
        self.cur_bottom_z = if self.cur_pile_idx as usize == self.piles.layers.len() {
            self.piles.layers.last().unwrap().bottom_z
        } else {
            self.piles.layers[self.cur_pile_idx as usize].bottom_z
        };
    }

    // ConflictChecker.hpp:82 `float curBottomZ() const { return _curBottomZ; }`
    pub fn cur_bottom_z(&self) -> f32 {
        self.cur_bottom_z
    }

    // ConflictChecker.hpp:83 `LineWithIDs curLines() const`
    pub fn cur_lines(&self) -> LineWithIDs {
        // ConflictChecker.hpp:85 `auto [b, e] = curRange();`
        let (b, e) = self.cur_range();
        // ConflictChecker.hpp:86 `LineWithIDs lines;`
        let mut lines: LineWithIDs = LineWithIDs::new();
        // ConflictChecker.hpp:87 `for (int i = b; i < e; ++i) {`
        for i in b..e {
            // ConflictChecker.hpp:88 `for (const ExtrusionPath &path : _piles[i].paths) {`
            for path in &self.piles.layers[i as usize].paths {
                // ConflictChecker.hpp:89 `if (path.is_force_no_extrusion() == false) {`
                if !path.is_force_no_extrusion() {
                    // ConflictChecker.hpp:90 `Polyline check_polyline = path.polyline;`
                    let mut check_polyline = path.polyline.clone();
                    // ConflictChecker.hpp:91 `check_polyline.translate(_offset);`
                    check_polyline.translate(self.offset);
                    // ConflictChecker.hpp:92 `Lines tmpLines = check_polyline.lines();`
                    let tmp_lines = check_polyline.lines();
                    // ConflictChecker.hpp:93 `for (const Line &line : tmpLines) { lines.emplace_back(line, _id, path.role()); }`
                    for line in tmp_lines {
                        lines.push(LineWithID::new(line, self.id, path.role));
                    }
                }
            }
        }
        // ConflictChecker.hpp:97 `return lines;`
        lines
    }
}

// ConflictChecker.hpp:100-102 `friend bool operator>/</== (left, right) { return left._curBottomZ <op> right._curBottomZ; }`
// (implemented inline at the comparison sites in the priority queue below)

// ConflictChecker.hpp:110 `class LinesBucketQueue`
pub struct LinesBucketQueue {
    // ConflictChecker.hpp:113 `std::vector<LinesBucket> line_buckets;`
    pub line_buckets: Vec<LinesBucket>,
    // ConflictChecker.hpp:114 `std::priority_queue<LinesBucket *, std::vector<LinesBucket *>, LinesBucketPtrComp> line_bucket_ptr_queue;`
    // The C++ priority queue stores raw pointers into `line_buckets`. We mirror
    // this with indices into `line_buckets` (avoiding the raw-pointer
    // invalidation footgun that the C++ `emplace_back_bucket` works around).
    // `LinesBucketPtrComp` compares `*left > *right`, i.e. by `_curBottomZ`, so
    // the std::priority_queue (a max-heap) actually pops the *smallest*
    // `_curBottomZ` first. We mirror that ordering here.
    pub line_bucket_ptr_queue: Vec<usize>,
}

impl LinesBucketQueue {
    pub fn new() -> Self {
        Self {
            line_buckets: Vec::new(),
            line_bucket_ptr_queue: Vec::new(),
        }
    }

    // Internal: comparator mirroring `LinesBucketPtrComp::operator()`
    // (ConflictChecker.hpp:107 `return *left > *right;`). std::priority_queue
    // uses Compare as "less"; the element for which `comp` returns false against
    // all others bubbles to top(). With `*left > *right` as the comparator, the
    // top() is the bucket with the smallest `_curBottomZ`.
    #[inline]
    fn top_index(&self) -> Option<usize> {
        // top() of the C++ priority_queue == element that is not "greater than"
        // (per LinesBucketPtrComp, which is operator>) any other == smallest
        // _curBottomZ.
        self.line_bucket_ptr_queue
            .iter()
            .copied()
            .min_by(|&a, &b| {
                self.line_buckets[a]
                    .cur_bottom_z
                    .partial_cmp(&self.line_buckets[b].cur_bottom_z)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    }

    // ConflictChecker.cpp:92 `void LinesBucketQueue::emplace_back_bucket(ExtrusionLayers &&els, const void *objPtr, Point offset)`
    pub fn emplace_back_bucket(&mut self, els: ExtrusionLayers, obj_ptr: *const (), offset: Point) {
        // ConflictChecker.cpp:94-107 The C++ code carefully rebuilds the pointer
        // queue when `line_buckets` reallocates (since it stores raw pointers).
        // Our index-based queue is immune to reallocation, so we just push the
        // new bucket and enqueue its index.
        let idx = self.line_buckets.len();
        // ConflictChecker.cpp:95 `line_buckets.emplace_back(std::move(els), objPtr, offset);`
        self.line_buckets.push(LinesBucket::new(els, obj_ptr, offset));
        // ConflictChecker.cpp:101 `line_bucket_ptr_queue.push(&line_buckets.back());`
        self.line_bucket_ptr_queue.push(idx);
    }

    // ConflictChecker.cpp:111 `float LinesBucketQueue::getCurrBottomZ()`
    // remove lowest and get the current bottom z
    pub fn get_curr_bottom_z(&mut self) -> f32 {
        // ConflictChecker.cpp:113 `auto lowest = line_bucket_ptr_queue.top();`
        let lowest = self.top_index().unwrap();
        // ConflictChecker.cpp:114 `line_bucket_ptr_queue.pop();`
        self.pop_index(lowest);
        // ConflictChecker.cpp:115 `float layerBottomZ = lowest->curBottomZ();`
        let layer_bottom_z = self.line_buckets[lowest].cur_bottom_z();
        // ConflictChecker.cpp:116 `std::vector<LinesBucket *> lowests;`
        let mut lowests: Vec<usize> = Vec::new();
        // ConflictChecker.cpp:117 `lowests.push_back(lowest);`
        lowests.push(lowest);

        // ConflictChecker.cpp:119 `while (line_bucket_ptr_queue.empty() == false && std::abs(line_bucket_ptr_queue.top()->curBottomZ() - lowest->curBottomZ()) < EPSILON) {`
        while !self.line_bucket_ptr_queue.is_empty() && {
            let top = self.top_index().unwrap();
            (self.line_buckets[top].cur_bottom_z() - self.line_buckets[lowest].cur_bottom_z()).abs()
                < EPSILON as f32
        } {
            // ConflictChecker.cpp:120 `lowests.push_back(line_bucket_ptr_queue.top());`
            let top = self.top_index().unwrap();
            lowests.push(top);
            // ConflictChecker.cpp:121 `line_bucket_ptr_queue.pop();`
            self.pop_index(top);
        }

        // ConflictChecker.cpp:124 `for (LinesBucket *bp : lowests) {`
        for bp in lowests {
            // ConflictChecker.cpp:125 `float prevZ = bp->curBottomZ();`
            let prev_z = self.line_buckets[bp].cur_bottom_z();
            // ConflictChecker.cpp:126 `bp->raise();`
            self.line_buckets[bp].raise();
            // ConflictChecker.cpp:127 `if (bp->curBottomZ() == prevZ) continue;`
            if self.line_buckets[bp].cur_bottom_z() == prev_z {
                continue;
            }
            // ConflictChecker.cpp:128 `if (bp->valid()) { line_bucket_ptr_queue.push(bp); }`
            if self.line_buckets[bp].valid() {
                self.line_bucket_ptr_queue.push(bp);
            }
        }
        // ConflictChecker.cpp:130 `return layerBottomZ;`
        layer_bottom_z
    }

    // Helper mirroring priority_queue::pop() for a specific index.
    #[inline]
    fn pop_index(&mut self, idx: usize) {
        if let Some(pos) = self.line_bucket_ptr_queue.iter().position(|&i| i == idx) {
            self.line_bucket_ptr_queue.remove(pos);
        }
    }

    // ConflictChecker.hpp:118 `bool valid() const { return line_bucket_ptr_queue.empty() == false; }`
    pub fn valid(&self) -> bool {
        !self.line_bucket_ptr_queue.is_empty()
    }

    // ConflictChecker.cpp:133 `LineWithIDs LinesBucketQueue::getCurLines() const`
    pub fn get_cur_lines(&self) -> LineWithIDs {
        // ConflictChecker.cpp:135 `LineWithIDs lines;`
        let mut lines: LineWithIDs = LineWithIDs::new();
        // ConflictChecker.cpp:136 `for (const LinesBucket &bucket : line_buckets) {`
        for bucket in &self.line_buckets {
            // ConflictChecker.cpp:137 `if (bucket.valid()) {`
            if bucket.valid() {
                // ConflictChecker.cpp:138 `LineWithIDs tmpLines = bucket.curLines();`
                let tmp_lines = bucket.cur_lines();
                // ConflictChecker.cpp:139 `lines.insert(lines.end(), tmpLines.begin(), tmpLines.end());`
                lines.extend(tmp_lines);
            }
        }
        // ConflictChecker.cpp:142 `return lines;`
        lines
    }
}

// ConflictChecker.cpp:145 `void getExtrusionPathsFromEntity(const ExtrusionEntityCollection *entity, ExtrusionPaths &paths)`
pub fn get_extrusion_paths_from_entity(
    entity: &ExtrusionEntityCollection,
    paths: &mut Vec<ExtrusionPath>,
) {
    // ConflictChecker.cpp:147 `std::function<void(...)> getExtrusionPathImpl = [&](...) {`
    fn get_extrusion_path_impl(entity: &ExtrusionEntityCollection, paths: &mut Vec<ExtrusionPath>) {
        // ConflictChecker.cpp:148 `for (auto entityPtr : entity->entities) {`
        for entity_ptr in &entity.entities {
            match entity_ptr {
                // ConflictChecker.cpp:149 `if (const ExtrusionEntityCollection *collection = dynamic_cast<...>(entityPtr)) {`
                ExtrusionEntityType::Collection(collection) => {
                    // ConflictChecker.cpp:150 `getExtrusionPathImpl(collection, paths);`
                    get_extrusion_path_impl(collection, paths);
                }
                // ConflictChecker.cpp:151 `} else if (const ExtrusionPath *path = dynamic_cast<...>(entityPtr)) {`
                ExtrusionEntityType::Path(path) => {
                    // ConflictChecker.cpp:152 `paths.push_back(*path);`
                    paths.push(path.clone());
                }
                // ConflictChecker.cpp:155 `} else if (const ExtrusionLoop *loop = dynamic_cast<...>(entityPtr)) {`
                ExtrusionEntityType::Loop(loop_) => {
                    // ConflictChecker.cpp:156 `for (const ExtrusionPath &path : loop->paths) { paths.push_back(path); }`
                    for path in &loop_.paths {
                        paths.push(path.clone());
                    }
                }
            }
            // NOTE: ConflictChecker.cpp:153-154 handle `ExtrusionMultiPath`, which
            // has no representation in this crate's `ExtrusionEntityType` enum
            // (multipaths are not stored in collections here). If/when the enum
            // grows a MultiPath variant, mirror the loop over `multipath->paths`.
        }
    }
    // ConflictChecker.cpp:160 `getExtrusionPathImpl(entity, paths);`
    get_extrusion_path_impl(entity, paths);
}

// ConflictChecker.cpp:289 `ConflictComputeResult ConflictChecker::line_intersect(...)` and the
// supporting types are declared in ConflictChecker.hpp.

// ConflictChecker.hpp:131 `struct ConflictComputeResult`
#[derive(Clone, Copy)]
pub struct ConflictComputeResult {
    // ConflictChecker.hpp:133 `const void* _obj1;`
    pub obj1: *const (),
    // ConflictChecker.hpp:134 `const void* _obj2;`
    pub obj2: *const (),
}

impl ConflictComputeResult {
    // ConflictChecker.hpp:136 `ConflictComputeResult(const void* o1, const void* o2) : _obj1(o1), _obj2(o2) {}`
    pub fn new(o1: *const (), o2: *const ()) -> Self {
        Self {
            obj1: o1,
            obj2: o2,
        }
    }
}

// ConflictChecker.hpp:140 `using ConflictComputeOpt = std::optional<ConflictComputeResult>;`
pub type ConflictComputeOpt = Option<ConflictComputeResult>;

// ConflictChecker.hpp:142 `using ConflictObjName = std::optional<std::pair<std::string, std::string>>;`
pub type ConflictObjName = Option<(String, String)>;

// ConflictChecker.hpp:144 `struct ConflictChecker`
pub struct ConflictChecker;

impl ConflictChecker {
    // ConflictChecker.cpp:203 `ConflictComputeOpt ConflictChecker::find_inter_of_lines(const LineWithIDs &lines)`
    pub fn find_inter_of_lines(lines: &LineWithIDs) -> ConflictComputeOpt {
        // ConflictChecker.cpp:205 `using namespace RasterizationImpl;`
        use rasterization_impl::*;
        // ConflictChecker.cpp:206 `std::map<IndexPair, std::vector<int>> indexToLine;`
        let mut index_to_line: BTreeMap<IndexPair, Vec<i32>> = BTreeMap::new();

        // ConflictChecker.cpp:208 `for (int i = 0; i < lines.size(); ++i) {`
        for i in 0..lines.len() as i32 {
            // ConflictChecker.cpp:209 `const LineWithID &l1      = lines[i];`
            let l1 = &lines[i as usize];
            // ConflictChecker.cpp:210 `auto              indexes = line_rasterization(l1._line);`
            let indexes = line_rasterization(&l1.line, raste_x_distance(), raste_y_distance());
            // ConflictChecker.cpp:211 `for (auto index : indexes) {`
            for index in indexes {
                // ConflictChecker.cpp:212 `const auto &possibleIntersectIdxs = indexToLine[index];`
                // ConflictChecker.cpp:213 `for (auto possibleIntersectIdx : possibleIntersectIdxs) {`
                let possible = index_to_line.get(&index).cloned().unwrap_or_default();
                for possible_intersect_idx in &possible {
                    // ConflictChecker.cpp:214 `const LineWithID &l2 = lines[possibleIntersectIdx];`
                    let l2 = &lines[*possible_intersect_idx as usize];
                    // ConflictChecker.cpp:215 `if (auto interRes = line_intersect(l1, l2); interRes.has_value()) { return interRes; }`
                    let inter_res = Self::line_intersect(l1, l2);
                    if inter_res.is_some() {
                        return inter_res;
                    }
                }
                // ConflictChecker.cpp:217 `indexToLine[index].push_back(i);`
                index_to_line.entry(index).or_default().push(i);
            }
        }
        // ConflictChecker.cpp:220 `return {};`
        None
    }

    // ConflictChecker.cpp:289 `ConflictComputeOpt ConflictChecker::line_intersect(const LineWithID &l1, const LineWithID &l2)`
    pub fn line_intersect(l1: &LineWithID, l2: &LineWithID) -> ConflictComputeOpt {
        // ConflictChecker.cpp:291 `constexpr double SUPPORT_THRESHOLD = 100;  // this large almost disables conflict check of supports`
        const SUPPORT_THRESHOLD: f64 = 100.0;
        // ConflictChecker.cpp:292 `constexpr double OTHER_THRESHOLD   = 0.01;`
        const OTHER_THRESHOLD: f64 = 0.01;
        // ConflictChecker.cpp:293 `if (l1._id == l2._id) { return {}; } // return true if lines are from same object`
        if l1.id == l2.id {
            return None;
        }
        // ConflictChecker.cpp:294 `double overlap_length = 0.;`
        // ConflictChecker.cpp:295 `bool   overlap  = l1._line.overlap(l2._line, overlap_length);`
        // The crate's `Line::overlap` returns Option<CoordF> (length) instead of
        // a bool + out-param; None means no overlap.
        let overlap_opt = l1.line.overlap(&l2.line);
        let overlap = overlap_opt.is_some();
        let overlap_length = overlap_opt.unwrap_or(0.0);
        // ConflictChecker.cpp:296 `if (overlap && overlap_length > scaled(OTHER_THRESHOLD)) return std::make_optional<ConflictComputeResult>(l1._id, l2._id);`
        if overlap && overlap_length > scaled(OTHER_THRESHOLD) as CoordF {
            return Some(ConflictComputeResult::new(l1.id, l2.id));
        }
        // ConflictChecker.cpp:297 `Point inter;`
        // ConflictChecker.cpp:298 `bool  intersect = l1._line.intersection(l2._line, &inter);`
        let inter_opt = l1.line.intersection(&l2.line);
        let intersect = inter_opt.is_some();

        // ConflictChecker.cpp:300 `if (intersect) {`
        if intersect {
            let inter = inter_opt.unwrap();
            // ConflictChecker.cpp:301 `double dist1 = std::min(unscale(Point(l1._line.a - inter)).norm(), unscale(Point(l1._line.b - inter)).norm());`
            let dist1 = (l1.line.a - inter)
                .to_f64()
                .length()
                .min((l1.line.b - inter).to_f64().length());
            // ConflictChecker.cpp:302 `double dist2 = std::min(unscale(Point(l2._line.a - inter)).norm(), unscale(Point(l2._line.b - inter)).norm());`
            let dist2 = (l2.line.a - inter)
                .to_f64()
                .length()
                .min((l2.line.b - inter).to_f64().length());
            // ConflictChecker.cpp:303 `double dist  = std::min(dist1, dist2);`
            let dist = dist1.min(dist2);
            // ConflictChecker.cpp:304 `ExtrusionRole r1        = l1._role;`
            let r1 = l1.role;
            // ConflictChecker.cpp:305 `ExtrusionRole r2        = l2._role;`
            let r2 = l2.role;
            // ConflictChecker.cpp:306 `bool both_support = r1 == erSupportMaterial || r1 == erSupportMaterialInterface || r1 == erSupportTransition;`
            let mut both_support = r1 == ExtrusionRole::SupportMaterial
                || r1 == ExtrusionRole::SupportMaterialInterface
                || r1 == ExtrusionRole::SupportTransition;
            // ConflictChecker.cpp:307 `both_support = both_support && ( r2 == erSupportMaterial || r2 == erSupportMaterialInterface || r2 == erSupportTransition);`
            both_support = both_support
                && (r2 == ExtrusionRole::SupportMaterial
                    || r2 == ExtrusionRole::SupportMaterialInterface
                    || r2 == ExtrusionRole::SupportTransition);
            // ConflictChecker.cpp:308 `if (dist > (both_support ? SUPPORT_THRESHOLD:OTHER_THRESHOLD)) {`
            if dist > (if both_support { SUPPORT_THRESHOLD } else { OTHER_THRESHOLD }) {
                // ConflictChecker.cpp:309 the two lines intersects if dist>0.01mm for regular lines, and if dist>1mm for both supports
                // ConflictChecker.cpp:310 `return std::make_optional<ConflictComputeResult>(l1._id, l2._id);`
                return Some(ConflictComputeResult::new(l1.id, l2.id));
            }
        }
        // ConflictChecker.cpp:313 `return {};`
        None
    }
}

// NOTE on `unscale`: ConflictChecker.cpp:301-302 use the free function
// `unscale(Point)` (Vec2crd -> Vec2d, dividing by SCALING_FACTOR). The crate's
// `Point::to_f64()` performs exactly this conversion, so we use it above.
#[allow(dead_code)]
fn _unscale_marker(p: Point) -> f64 {
    // referenced solely to keep the `unscale` import documented/available
    unscale(p.x())
}

#[cfg(test)]
mod tests {
    use super::rasterization_impl::*;
    use super::*;

    #[test]
    fn test_point_map_grid_index() {
        let p = Point::new(2_500_000, -1_500_000);
        let idx = point_map_grid_index(&p, 1_000_000, 1_000_000);
        assert_eq!(idx, (2, -1));
    }

    #[test]
    fn test_line_rasterization_single_cell() {
        // A line that stays within a single grid cell yields one voxel.
        let line = Line::new(Point::new(10, 10), Point::new(20, 20));
        let grids = line_rasterization(&line, 1_000_000, 1_000_000);
        assert_eq!(grids, vec![(0, 0)]);
    }

    #[test]
    fn test_line_rasterization_horizontal() {
        // Horizontal line crossing three cells along X.
        let line = Line::new(Point::new(500_000, 500_000), Point::new(2_500_000, 500_000));
        let grids = line_rasterization(&line, 1_000_000, 1_000_000);
        assert_eq!(grids, vec![(0, 0), (1, 0), (2, 0)]);
    }

    #[test]
    fn test_line_intersect_same_id_none() {
        let id = 0x1usize as *const ();
        let l1 = LineWithID::new(
            Line::new(Point::new(0, 0), Point::new(1_000_000, 0)),
            id,
            ExtrusionRole::Perimeter,
        );
        let l2 = LineWithID::new(
            Line::new(Point::new(500_000, -500_000), Point::new(500_000, 500_000)),
            id,
            ExtrusionRole::Perimeter,
        );
        assert!(ConflictChecker::line_intersect(&l1, &l2).is_none());
    }

    #[test]
    fn test_line_intersect_crossing() {
        let id1 = 0x1usize as *const ();
        let id2 = 0x2usize as *const ();
        // Two crossing perimeters from different objects, intersecting in the
        // middle (dist > OTHER_THRESHOLD) -> conflict.
        let l1 = LineWithID::new(
            Line::new(Point::new(0, 0), Point::new(2_000_000, 0)),
            id1,
            ExtrusionRole::Perimeter,
        );
        let l2 = LineWithID::new(
            Line::new(Point::new(1_000_000, -1_000_000), Point::new(1_000_000, 1_000_000)),
            id2,
            ExtrusionRole::Perimeter,
        );
        assert!(ConflictChecker::line_intersect(&l1, &l2).is_some());
    }
}
