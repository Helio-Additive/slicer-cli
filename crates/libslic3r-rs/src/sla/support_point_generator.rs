//! Faithful port of SLA/SupportPointGenerator.{hpp,cpp}.
//!
//! C++ References:
//! - SLA/SupportPointGenerator.hpp
//! - SLA/SupportPointGenerator.cpp
//!
//! Porting notes (divergences are local representation changes only, each one
//! marked at its use site):
//! - `MyLayer *layer` / `Structure *island` raw pointers are represented by
//!   indices plus copies of the only members ever read through the pointers
//!   (`layer_id`, `print_z`). `Structure::Link::island` is the island's index
//!   within the *adjacent* layer's `islands` vector (links always connect
//!   adjacent layers in the C++ code).
//! - `const ExPolygon *polygon` keeps pointer semantics via a `&'slc ExPolygon`
//!   borrow of the caller-owned slices.
//! - `std::mt19937`, `std::uniform_real_distribution`, `std::uniform_int_distribution`
//!   and `std::shuffle` are ported 1:1 against libstdc++ behavior (see the
//!   "std <random> support" section at the bottom). `std::random_device` is
//!   nondeterministic by design; a time/thread-hash substitute is used
//!   (same approach as fuzzy_skin.rs) to stay wasm-safe.
//! - `std::unordered_map` / `std::unordered_multimap` iteration order is
//!   implementation-defined in C++; the port iterates the poisson cells in
//!   insertion order (deterministic), which is one valid C++ ordering.
//! - ClipperUtils calls (`diff_ex`, `diff`, `expand`, `intersection_ex`,
//!   `intersection`) all use NonZero fill in C++; the crate's geo-clipper
//!   backend exposes ExPolygons operations with NonZero fill, so raw
//!   `Polygons` operands (contours CCW + holes CW) are reassembled with
//!   `union_polygons_ex` (a set-wise no-op under NonZero fill).

use std::collections::HashMap;
use std::slice;

use crate::clipper_utils::{
    difference, intersection, intersection_pl, offset_polygons, union_polygons_ex, OffsetJoinType,
};
use crate::geometry::{
    convex_hull_expolygons, get_extents_polygons, to_polylines_expoly, BoundingBox, ExPolygon,
    ExPolygons, Polygon, Polygons,
};
use crate::libslic3r::{EPSILON, SCALING_FACTOR};
use crate::min_area_bounding_box::{MinAreaBoundigBox, PolygonLevel};
use crate::sla::ccr_par;
use crate::sla::indexed_mesh::{IndexedMesh, Vec3d};
use crate::sla::support_point::SupportPoint;
use crate::tesselate::{triangulate_expolygon_2f, NORMALS_UP};
use crate::triangle_mesh::{Vec2f, Vec2i, Vec3f, Vec3i};

/// SupportPointGenerator.hpp:32 — `std::function<void(void)> throw_on_cancel`
pub type ThrowOnCancel = Box<dyn Fn() + Send + Sync>;
/// SupportPointGenerator.hpp:32 — `std::function<void(int)> statusfn`
pub type StatusFn = Box<dyn Fn(i32) + Send + Sync>;

/// `cross2()` for `Vec2f` (Point.hpp helper used at SupportPointGenerator.cpp:351).
#[inline]
fn cross2(v1: &Vec2f, v2: &Vec2f) -> f32 {
    v1.x * v2.y - v1.y * v2.x
}

/// Shared-mutable raw pointer wrapper so that `ccr_par::for_each` index loops can
/// mutate distinct vector elements from parallel workers, exactly like the C++
/// TBB loops do. Soundness: every loop below visits each index exactly once, so
/// no two workers ever form references to the same element.
struct SyncMutPtr<T>(*mut T);
unsafe impl<T> Send for SyncMutPtr<T> {}
unsafe impl<T> Sync for SyncMutPtr<T> {}

impl<T> SyncMutPtr<T> {
    /// Accessor taking `&self` so closures capture the whole `Sync` wrapper
    /// (Rust 2021 field-precise capture would otherwise grab the raw pointer).
    #[inline]
    fn get(&self) -> *mut T {
        self.0
    }
}

// SupportPointGenerator.hpp:19
pub struct SupportPointGenerator<'a> {
    // SupportPointGenerator.hpp:198
    m_output: Vec<SupportPoint>,
    // SupportPointGenerator.hpp:200
    m_config: Config,
    // SupportPointGenerator.hpp:220
    m_emesh: &'a IndexedMesh,
    // SupportPointGenerator.hpp:221
    m_throw_on_cancel: ThrowOnCancel,
    // SupportPointGenerator.hpp:222
    m_statusfn: StatusFn,
    // SupportPointGenerator.hpp:224
    m_rng: Mt19937,
}

// SupportPointGenerator.hpp:21
#[derive(Debug, Clone)]
pub struct Config {
    // SupportPointGenerator.hpp:22
    pub density_relative: f32, // {1.f}
    // SupportPointGenerator.hpp:23
    pub minimal_distance: f32, // {1.f}
    // SupportPointGenerator.hpp:24
    pub head_diameter: f32, // {0.4f}
}

impl Default for Config {
    fn default() -> Self {
        Self {
            density_relative: 1.0,  // SupportPointGenerator.hpp:22
            minimal_distance: 1.0,  // SupportPointGenerator.hpp:23
            head_diameter: 0.4,     // SupportPointGenerator.hpp:24
        }
    }
}

impl Config {
    // SupportPointGenerator.hpp:26 — Originally calibrated to 7.7f, reduced
    // density by Tamas to 70% which is 11.1 (7.7 / 0.7) to adjust for new
    // algorithm changes in tm_suppt_gen_improve
    // SupportPointGenerator.hpp:27
    /// a force one point can support       (arbitrary force unit)
    #[inline]
    pub fn support_force(&self) -> f32 {
        11.1 / self.density_relative
    }

    // SupportPointGenerator.hpp:28
    /// pressure that the display exerts    (the force unit per mm2)
    #[inline]
    pub fn tear_pressure(&self) -> f32 {
        1.0
    }
}

// SupportPointGenerator.hpp:63
#[derive(Debug, Clone, Copy)]
pub struct Link {
    // SupportPointGenerator.hpp:65 — `Structure *island;`
    // Index of the island within the adjacent layer's `islands` vector (see
    // module porting notes).
    pub island: usize,
    // SupportPointGenerator.hpp:66
    pub overlap_area: f32,
}

impl Link {
    // SupportPointGenerator.hpp:64
    pub fn new(island: usize, overlap_area: f32) -> Self {
        Self { island, overlap_area }
    }
}

// SupportPointGenerator.hpp:41
pub struct Structure<'slc> {
    // SupportPointGenerator.hpp:48 — `MyLayer *layer;` (id + print_z copy; the
    // only members read through the pointer are `print_z` and identity).
    pub layer_id: usize,
    pub layer_print_z: f64,
    // SupportPointGenerator.hpp:49 — `const ExPolygon* polygon = nullptr;`
    pub polygon: &'slc ExPolygon,
    // SupportPointGenerator.hpp:50
    pub bbox: BoundingBox,
    // SupportPointGenerator.hpp:51
    pub centroid: Vec2f,
    // SupportPointGenerator.hpp:52
    pub area: f32,
    // SupportPointGenerator.hpp:53
    pub zlevel: f32,
    // SupportPointGenerator.hpp:54-56 — How well is this ExPolygon held to the
    // print base? Positive number, the higher the better.
    pub supports_force_this_layer: f32,
    // SupportPointGenerator.hpp:57
    pub supports_force_inherited: f32,
    // SupportPointGenerator.hpp:69-77 — boost::container::small_vector<Link, 4>
    // in release builds, std::vector<Link> in debug; both are a plain `Vec` here.
    pub islands_above: Vec<Link>,
    pub islands_below: Vec<Link>,
    // SupportPointGenerator.hpp:78-79 — Overhangs, that are dangling considerably.
    pub dangling_areas: ExPolygons,
    // SupportPointGenerator.hpp:80-81 — Complete overhands.
    pub overhangs: ExPolygons,
    // SupportPointGenerator.hpp:82-83 — Overhangs, where the surface must slope.
    pub overhangs_slopes: ExPolygons,
    // SupportPointGenerator.hpp:84
    pub overhangs_area: f32,
}

impl<'slc> Structure<'slc> {
    // SupportPointGenerator.hpp:42-47
    // Structure(MyLayer &layer, const ExPolygon& poly, const BoundingBox &bbox,
    //           const Vec2f &centroid, float area, float h)
    pub fn new(
        layer_id: usize,
        layer_print_z: f64,
        poly: &'slc ExPolygon,
        bbox: BoundingBox,
        centroid: Vec2f,
        area: f32,
        h: f32,
    ) -> Self {
        Self {
            layer_id,
            layer_print_z,
            polygon: poly,
            bbox,
            centroid,
            area,
            zlevel: h,
            supports_force_this_layer: 0.0,  // SupportPointGenerator.hpp:56
            supports_force_inherited: 0.0,   // SupportPointGenerator.hpp:57
            islands_above: Vec::new(),
            islands_below: Vec::new(),
            dangling_areas: Vec::new(),
            overhangs: Vec::new(),
            overhangs_slopes: Vec::new(),
            overhangs_area: 0.0,             // SupportPointGenerator.hpp:84
        }
    }

    // SupportPointGenerator.hpp:58
    #[inline]
    pub fn supports_force_total(&self) -> f32 {
        self.supports_force_this_layer + self.supports_force_inherited
    }

    // SupportPointGenerator.hpp:86-89
    // bool overlaps(const Structure &rhs) const
    pub fn overlaps(&self, rhs: &Structure) -> bool {
        // SupportPointGenerator.hpp:87 — FIXME ExPolygon::overlaps() shall be
        // commutative, it is not!
        // SupportPointGenerator.hpp:88
        self.bbox.intersects(&rhs.bbox)
            && (expolygon_overlaps(self.polygon, rhs.polygon)
                || expolygon_overlaps(rhs.polygon, self.polygon))
    }

    // SupportPointGenerator.hpp:90-98
    // float overlap_area(const Structure &rhs) const
    pub fn overlap_area(&self, rhs: &Structure) -> f32 {
        // SupportPointGenerator.hpp:91
        let mut out: f64 = 0.;
        // SupportPointGenerator.hpp:92
        if self.bbox.intersects(&rhs.bbox) {
            // SupportPointGenerator.hpp:93 — Polygons polys = intersection(...)
            // (the geo-clipper wrapper returns ExPolygons; summing `ExPolygon::area()`
            // equals the C++ sum over the flattened Polygons, holes negative)
            let polys = intersection(slice::from_ref(self.polygon), slice::from_ref(rhs.polygon));
            // SupportPointGenerator.hpp:94-95
            for poly in &polys {
                out += poly.area();
            }
        }
        // SupportPointGenerator.hpp:97
        out as f32
    }

    // SupportPointGenerator.hpp:99-104
    // float area_below() const
    // (the C++ method dereferences `Link::island`; the port takes the layer
    // below's islands explicitly)
    pub fn area_below(&self, below_islands: &[Structure]) -> f32 {
        // SupportPointGenerator.hpp:100
        let mut area = 0.0f32;
        // SupportPointGenerator.hpp:101-102
        for below in &self.islands_below {
            area += below_islands[below.island].area;
        }
        // SupportPointGenerator.hpp:103
        area
    }

    // SupportPointGenerator.hpp:105-116
    // Polygons polygons_below() const
    pub fn polygons_below(&self, below_islands: &[Structure]) -> Polygons {
        // SupportPointGenerator.hpp:106-108
        let mut cnt: usize = 0;
        for below in &self.islands_below {
            cnt += 1 + below_islands[below.island].polygon.holes.len();
        }
        // SupportPointGenerator.hpp:109-110
        let mut out: Polygons = Vec::with_capacity(cnt);
        // SupportPointGenerator.hpp:111-114
        for below in &self.islands_below {
            out.push(below_islands[below.island].polygon.contour.clone());
            out.extend(below_islands[below.island].polygon.holes.iter().cloned());
        }
        // SupportPointGenerator.hpp:115
        out
    }

    // SupportPointGenerator.hpp:117-123
    // ExPolygons expolygons_below() const
    pub fn expolygons_below(&self, below_islands: &[Structure]) -> ExPolygons {
        // SupportPointGenerator.hpp:118-119
        let mut out: ExPolygons = Vec::with_capacity(self.islands_below.len());
        // SupportPointGenerator.hpp:120-121
        for below in &self.islands_below {
            out.push(below_islands[below.island].polygon.clone());
        }
        // SupportPointGenerator.hpp:122
        out
    }

    // SupportPointGenerator.hpp:124-125 — Positive deficit of the supports.
    // If negative, this area is well supported. If positive, more supports need
    // to be added.
    #[inline]
    pub fn support_force_deficit(&self, tear_pressure: f32) -> f32 {
        self.area * tear_pressure - self.supports_force_total()
    }
}

/// `ExPolygon::overlaps(const ExPolygon &other)` (ExPolygon.cpp), needed by
/// `Structure::overlaps` (SupportPointGenerator.hpp:88).
fn expolygon_overlaps(this: &ExPolygon, other: &ExPolygon) -> bool {
    // ExPolygon.cpp: Polylines pl_out = intersection_pl(to_polylines(other), *this);
    let pl_out = intersection_pl(&to_polylines_expoly(other), slice::from_ref(this));
    // ExPolygon.cpp: return ! pl_out.empty() || other.contains(this->contour.points.front());
    !pl_out.is_empty() || other.contains_point(&this.contour.points[0])
}

// SupportPointGenerator.hpp:128
pub struct MyLayer<'slc> {
    // SupportPointGenerator.hpp:130
    pub layer_id: usize,
    // SupportPointGenerator.hpp:131 — coordf_t print_z
    pub print_z: f64,
    // SupportPointGenerator.hpp:132
    pub islands: Vec<Structure<'slc>>,
}

impl<'slc> MyLayer<'slc> {
    // SupportPointGenerator.hpp:129
    // MyLayer(const size_t layer_id, coordf_t print_z)
    pub fn new(layer_id: usize, print_z: f64) -> Self {
        Self {
            layer_id,
            print_z,
            islands: Vec::new(),
        }
    }
}

// SupportPointGenerator.hpp:135
#[derive(Debug, Clone, Copy)]
pub struct RichSupportPoint {
    // SupportPointGenerator.hpp:136
    pub position: Vec3f,
    // SupportPointGenerator.hpp:137 — `Structure *island;`
    // The pointer is stored at insertion time and never dereferenced afterwards;
    // the port keeps the owning layer id for parity (see module porting notes).
    pub island_layer_id: usize,
}

// SupportPointGenerator.hpp:140
pub struct PointGrid3D {
    // SupportPointGenerator.hpp:141-145 — struct GridHash { ... } — the C++
    // custom hash (`hash<int>(x) ^ hash<int>(y*593) ^ hash<int>(z*7919)`) only
    // affects bucket placement; the port uses the std HashMap with tuple keys.
    // SupportPointGenerator.hpp:146 — typedef std::unordered_multimap<Vec3i,
    // RichSupportPoint, GridHash> Grid; (multimap == Vec per key)
    // SupportPointGenerator.hpp:148
    pub cell_size: Vec3f,
    // SupportPointGenerator.hpp:149
    pub grid: HashMap<(i32, i32, i32), Vec<RichSupportPoint>>,
}

impl Default for PointGrid3D {
    fn default() -> Self {
        Self {
            cell_size: Vec3f::zeros(),
            grid: HashMap::new(),
        }
    }
}

impl PointGrid3D {
    // SupportPointGenerator.hpp:151-155
    // Vec3i cell_id(const Vec3f &pos)
    pub fn cell_id(&self, pos: &Vec3f) -> Vec3i {
        Vec3i::new(
            (pos.x / self.cell_size.x).floor() as i32,  // SupportPointGenerator.hpp:152
            (pos.y / self.cell_size.y).floor() as i32,  // SupportPointGenerator.hpp:153
            (pos.z / self.cell_size.z).floor() as i32,  // SupportPointGenerator.hpp:154
        )
    }

    // SupportPointGenerator.hpp:157-162
    // void insert(const Vec2f &pos, Structure *island)
    pub fn insert(&mut self, pos: &Vec2f, island: &Structure) {
        // SupportPointGenerator.hpp:158-160
        let pt = RichSupportPoint {
            // SupportPointGenerator.hpp:159 — Vec3f(pos.x(), pos.y(), float(island->layer->print_z))
            position: Vec3f::new(pos.x, pos.y, island.layer_print_z as f32),
            island_layer_id: island.layer_id,
        };
        // SupportPointGenerator.hpp:161
        let cell = self.cell_id(&pt.position);
        self.grid
            .entry((cell.x, cell.y, cell.z))
            .or_default()
            .push(pt);
    }

    // SupportPointGenerator.hpp:164-180
    // bool collides_with(const Vec2f &pos, float print_z, float radius)
    pub fn collides_with(&self, pos: &Vec2f, print_z: f32, radius: f32) -> bool {
        // SupportPointGenerator.hpp:165
        let pos3d = Vec3f::new(pos.x, pos.y, print_z);
        // SupportPointGenerator.hpp:166
        let cell = self.cell_id(&pos3d);
        // SupportPointGenerator.hpp:167-169
        if let Some(points) = self.grid.get(&(cell.x, cell.y, cell.z)) {
            if self.collides_with_range(&pos3d, radius, points) {
                return true;
            }
        }
        // SupportPointGenerator.hpp:170-178 — NOTE: k only spans {-1, 0}
        // (`for (int k = -1; k < 1; ++ k)`), preserved verbatim.
        for i in -1..2 {
            for j in -1..2 {
                for k in -1..1 {
                    if i == 0 && j == 0 && k == 0 {
                        continue;
                    }
                    if let Some(points) = self.grid.get(&(cell.x + i, cell.y + j, cell.z + k)) {
                        if self.collides_with_range(&pos3d, radius, points) {
                            return true;
                        }
                    }
                }
            }
        }
        // SupportPointGenerator.hpp:179
        false
    }

    // SupportPointGenerator.hpp:183-190
    // bool collides_with(const Vec3f &pos, float radius, Grid::const_iterator
    //                    it_begin, Grid::const_iterator it_end)
    // (private overload; renamed since Rust has no overloading)
    fn collides_with_range(&self, pos: &Vec3f, radius: f32, points: &[RichSupportPoint]) -> bool {
        // SupportPointGenerator.hpp:184-188
        for it in points {
            let dist2 = (it.position - pos).norm_squared();
            if dist2 < radius * radius {
                return true;
            }
        }
        // SupportPointGenerator.hpp:189
        false
    }
}

// SupportPointGenerator.hpp:205
// enum IslandCoverageFlags : uint8_t { icfNone = 0x0, icfIsNew = 0x1, icfWithBoundary = 0x2 }
pub type IslandCoverageFlags = u8;
#[allow(non_upper_case_globals)]
pub const icfNone: IslandCoverageFlags = 0x0;
#[allow(non_upper_case_globals)]
pub const icfIsNew: IslandCoverageFlags = 0x1;
#[allow(non_upper_case_globals)]
pub const icfWithBoundary: IslandCoverageFlags = 0x2;

// SupportPointGenerator.cpp:24-53 — commented out in the C++ source, preserved:
// /*float SupportPointGenerator::approximate_geodesic_distance(const Vec3d& p1, const Vec3d& p2, Vec3d& n1, Vec3d& n2)
// {
//     n1.normalize();
//     n2.normalize();
//
//     Vec3d v = (p2-p1);
//     v.normalize();
//
//     float c1 = n1.dot(v);
//     float c2 = n2.dot(v);
//     float result = pow(p1(0)-p2(0), 2) + pow(p1(1)-p2(1), 2) + pow(p1(2)-p2(2), 2);
//     // Check for division by zero:
//     if(fabs(c1 - c2) > 0.0001)
//         result *= (asin(c1) - asin(c2)) / (c1 - c2);
//     return result;
// }
//
//
// float SupportPointGenerator::get_required_density(float angle) const
// {
//     // calculation would be density_0 * cos(angle). To provide one more degree of freedom, we will scale the angle
//     // to get the user-set density for 45 deg. So it ends up as density_0 * cos(K * angle).
//     float K = 4.f * float(acos(m_config.density_at_45/m_config.density_at_horizontal) / M_PI);
//     return std::max(0.f, float(m_config.density_at_horizontal * cos(K*angle)));
// }
//
// float SupportPointGenerator::distance_limit(float angle) const
// {
//     return 1./(2.4*get_required_density(angle));
// }*/

impl<'a> SupportPointGenerator<'a> {
    // SupportPointGenerator.cpp:55-67
    // SupportPointGenerator::SupportPointGenerator(
    //         const sla::IndexedMesh &emesh,
    //         const std::vector<ExPolygons> &slices,
    //         const std::vector<float> &     heights,
    //         const Config &                 config,
    //         std::function<void(void)> throw_on_cancel,
    //         std::function<void(int)>  statusfn)
    pub fn new(
        emesh: &'a IndexedMesh,
        slices: &[ExPolygons],
        heights: &[f32],
        config: &Config,
        throw_on_cancel: ThrowOnCancel,
        statusfn: StatusFn,
    ) -> Self {
        // SupportPointGenerator.cpp:62 — delegating constructor
        let mut this = Self::with_config(emesh, config, throw_on_cancel, statusfn);
        // SupportPointGenerator.cpp:64-65 — std::random_device rd; m_rng.seed(rd());
        // (std::random_device is nondeterministic; see module porting notes)
        this.m_rng.seed(random_device_substitute());
        // SupportPointGenerator.cpp:66
        this.execute(slices, heights);
        this
    }

    // SupportPointGenerator.cpp:69-79
    // SupportPointGenerator::SupportPointGenerator(
    //         const IndexedMesh &emesh,
    //         const SupportPointGenerator::Config &config,
    //         std::function<void ()> throw_on_cancel,
    //         std::function<void (int)> statusfn)
    pub fn with_config(
        emesh: &'a IndexedMesh,
        config: &Config,
        throw_on_cancel: ThrowOnCancel,
        statusfn: StatusFn,
    ) -> Self {
        Self {
            m_output: Vec::new(),
            m_config: config.clone(),          // SupportPointGenerator.cpp:74
            m_emesh: emesh,                    // SupportPointGenerator.cpp:75
            m_throw_on_cancel: throw_on_cancel, // SupportPointGenerator.cpp:76
            m_statusfn: statusfn,              // SupportPointGenerator.cpp:77
            m_rng: Mt19937::new(),             // std::mt19937 default ctor (seed 5489)
        }
    }

    // SupportPointGenerator.hpp:36
    pub fn output(&self) -> &Vec<SupportPoint> {
        &self.m_output
    }

    // SupportPointGenerator.hpp:37
    pub fn output_mut(&mut self) -> &mut Vec<SupportPoint> {
        &mut self.m_output
    }

    // SupportPointGenerator.hpp:196
    // void seed(std::mt19937::result_type s) { m_rng.seed(s); }
    pub fn seed(&mut self, s: u32) {
        self.m_rng.seed(s);
    }

    // SupportPointGenerator.cpp:81-86
    // void SupportPointGenerator::execute(const std::vector<ExPolygons> &slices,
    //                                     const std::vector<float> &     heights)
    pub fn execute(&mut self, slices: &[ExPolygons], heights: &[f32]) {
        // SupportPointGenerator.cpp:84
        self.process(slices, heights);
        // SupportPointGenerator.cpp:85 — project_onto_mesh(m_output);
        // (the vector is taken out temporarily so the `const` method can mutate
        // it through its own parameter, exactly as in C++)
        let mut output = std::mem::take(&mut self.m_output);
        self.project_onto_mesh(&mut output);
        self.m_output = output;
    }

    // SupportPointGenerator.cpp:88-115
    // void SupportPointGenerator::project_onto_mesh(std::vector<sla::SupportPoint>& points) const
    pub fn project_onto_mesh(&self, points: &mut Vec<SupportPoint>) {
        // SupportPointGenerator.cpp:90 — The function makes sure that all the
        // points are really exactly placed on the mesh.

        // SupportPointGenerator.cpp:92-93 — Use a reasonable granularity to
        // account for the worker thread synchronization cost.
        const GRANSIZE: usize = 64;

        let n = points.len();
        let points_ptr = SyncMutPtr(points.as_mut_ptr());
        // SupportPointGenerator.cpp:95
        ccr_par::for_each(
            0,
            n,
            |idx| {
                // SupportPointGenerator.cpp:97-99 — Don't call the following
                // function too often as it flushes CPU write caches due to
                // synchronization primitves.
                if idx % 16 == 0 {
                    (self.m_throw_on_cancel)();
                }

                // SupportPointGenerator.cpp:101
                let p: &mut Vec3f = unsafe { &mut (*points_ptr.get().add(idx)).pos };
                // SupportPointGenerator.cpp:102-104 — Project the point upward
                // and downward and choose the closer intersection with the mesh.
                let pd: Vec3d = p.cast::<f64>();
                let hit_up = self.m_emesh.query_ray_hit(&pd, &Vec3d::new(0., 0., 1.));
                let hit_down = self.m_emesh.query_ray_hit(&pd, &Vec3d::new(0., 0., -1.));

                // SupportPointGenerator.cpp:106-107
                let up = hit_up.is_hit();
                let down = hit_down.is_hit();

                // SupportPointGenerator.cpp:109-110
                if !up && !down {
                    return;
                }

                // SupportPointGenerator.cpp:112
                let hit = if !down || (hit_up.distance() < hit_down.distance()) {
                    &hit_up
                } else {
                    &hit_down
                };
                // SupportPointGenerator.cpp:113
                *p = *p + (*hit.direction() * hit.distance()).map(|v| v as f32);
            },
            GRANSIZE,
        );
    }

    // SupportPointGenerator.cpp:227-295
    // void SupportPointGenerator::process(const std::vector<ExPolygons>& slices,
    //                                     const std::vector<float>& heights)
    fn process(&mut self, slices: &[ExPolygons], heights: &[f32]) {
        // SupportPointGenerator.cpp:229-231 — SLA_SUPPORTPOINTGEN_DEBUG only.

        // SupportPointGenerator.cpp:233
        let mut layers: Vec<MyLayer<'_>> =
            make_layers(slices, heights, self.m_throw_on_cancel.as_ref());

        // SupportPointGenerator.cpp:235-236
        let mut point_grid = PointGrid3D::default();
        point_grid.cell_size = Vec3f::new(10., 10., 10.);

        // SupportPointGenerator.cpp:238-239
        let increment: f64 = 100.0 / layers.len() as f64;
        let mut status: f64 = 0.;

        // SupportPointGenerator.cpp:241
        for layer_id in 0..layers.len() {
            // SupportPointGenerator.cpp:242-243
            let (left, right) = layers.split_at_mut(layer_id);
            let layer_top: &mut MyLayer = &mut right[0];
            let layer_bottom: Option<&MyLayer> = if layer_id > 0 {
                Some(&left[layer_id - 1])
            } else {
                None
            };
            // SupportPointGenerator.cpp:244-249
            let mut support_force_bottom: Vec<f32> = Vec::new();
            if let Some(layer_bottom) = layer_bottom {
                support_force_bottom = vec![0.0f32; layer_bottom.islands.len()];
                for i in 0..layer_bottom.islands.len() {
                    support_force_bottom[i] = layer_bottom.islands[i].supports_force_total();
                }
            }
            // SupportPointGenerator.cpp:250-262
            for top in &layer_top.islands {
                for bottom_link in &top.islands_below {
                    let bottom = &layer_bottom.unwrap().islands[bottom_link.island];
                    // SupportPointGenerator.cpp:253-255 (commented out in C++):
                    // float centroids_dist = (bottom.centroid - top.centroid).norm();
                    // // Penalization resulting from centroid offset:
                    // bottom.supports_force *= std::min(1.f, 1.f - std::min(1.f, (1600.f * layer_height) * centroids_dist * centroids_dist / bottom.area));
                    // SupportPointGenerator.cpp:256 — &bottom - layer_bottom->islands.data()
                    let support_force = &mut support_force_bottom[bottom_link.island];
                    // SupportPointGenerator.cpp:257-259 (C++ FIXME comment):
                    // FIXME this condition does not reflect a bifurcation into a one
                    // large island and one tiny island well, it incorrectly resets
                    // the support force to zero. One should rather work with the
                    // overlap area vs overhang area.
                    // support_force *= std::min(1.f, 1.f - std::min(1.f, 0.1f * centroids_dist * centroids_dist / bottom.area));
                    // SupportPointGenerator.cpp:260-261 — Penalization resulting
                    // from increasing polygon area:
                    *support_force *= f32::min(1., 20. * bottom.area / top.area);
                }
            }
            // SupportPointGenerator.cpp:263-273 — Let's assign proper support
            // force to each of them:
            if layer_id > 0 {
                let layer_bottom = layer_bottom.unwrap();
                for (below_idx, below) in layer_bottom.islands.iter().enumerate() {
                    // SupportPointGenerator.cpp:266 — &below - layer_bottom->islands.data()
                    let below_support_force = support_force_bottom[below_idx];
                    // SupportPointGenerator.cpp:267-269
                    let mut above_overlap_area = 0.0f32;
                    for above_link in &below.islands_above {
                        above_overlap_area += above_link.overlap_area;
                    }
                    // SupportPointGenerator.cpp:270-271
                    for above_link in &below.islands_above {
                        layer_top.islands[above_link.island].supports_force_inherited +=
                            below_support_force * above_link.overlap_area / above_overlap_area;
                    }
                }
            }
            // SupportPointGenerator.cpp:274-280 — Now iterate over all polygons
            // and append new points if needed.
            for s in &mut layer_top.islands {
                // SupportPointGenerator.cpp:276-277 — Penalization resulting from
                // large diff from the last layer:
                s.supports_force_inherited /= f32::max(1., 0.17 * s.overhangs_area / s.area);

                // SupportPointGenerator.cpp:279
                self.add_support_points(s, &mut point_grid);
            }

            // SupportPointGenerator.cpp:282
            (self.m_throw_on_cancel)();

            // SupportPointGenerator.cpp:284-285
            status += increment;
            (self.m_statusfn)(status.round() as i32);

            // SupportPointGenerator.cpp:287-293 — SLA_SUPPORTPOINTGEN_DEBUG only:
            // /*std::string layer_num_str = ...;
            // output_expolygons(expolys_top, "top" + layer_num_str + ".svg");
            // output_expolygons(diff, "diff" + layer_num_str + ".svg");
            // if (!islands.empty())
            //     output_expolygons(islands, "islands" + layer_num_str + ".svg");*/
        }
    }

    // SupportPointGenerator.cpp:297-332
    // void SupportPointGenerator::add_support_points(SupportPointGenerator::Structure &s,
    //                                                SupportPointGenerator::PointGrid3D &grid3d)
    #[allow(unused_assignments)]
    fn add_support_points(&mut self, s: &mut Structure, grid3d: &mut PointGrid3D) {
        // SupportPointGenerator.cpp:299-300 — Select each type of surface
        // (overrhang, dangling, slope), derive the support force deficit for it
        // and call uniformly conver with the right params

        // SupportPointGenerator.cpp:302-303
        let tp: f32 = self.m_config.tear_pressure();
        let mut current: f32 = s.supports_force_total();

        // SupportPointGenerator.cpp:305-310
        if s.islands_below.is_empty() {
            // completely new island - needs support no doubt
            // deficit is full, there is nothing below that would hold this island
            let islands: ExPolygons = vec![s.polygon.clone()]; // { *s.polygon }
            self.uniformly_cover(&islands, s, s.area * tp, grid3d, icfIsNew | icfWithBoundary);
            return;
        }

        // SupportPointGenerator.cpp:312-314
        if !s.overhangs.is_empty() {
            let islands = s.overhangs.clone(); // (borrow-checker copy; not mutated by uniformly_cover)
            self.uniformly_cover(&islands, s, s.overhangs_area * tp, grid3d, icfNone);
        }

        // SupportPointGenerator.cpp:316
        // auto areafn = [](double sum, auto &p) { return sum + p.area() * SCALING_FACTOR * SCALING_FACTOR; };
        let areafn =
            |sum: f64, p: &ExPolygon| sum + p.area() * SCALING_FACTOR * SCALING_FACTOR;

        // SupportPointGenerator.cpp:318
        current = s.supports_force_total();
        // SupportPointGenerator.cpp:319-325
        if !s.dangling_areas.is_empty() {
            // Let's see if there's anything that overlaps enough to need supports:
            // What we now have in polygons needs support, regardless of what the
            // forces are, so we can add them.

            // SupportPointGenerator.cpp:323
            let a: f64 = s.dangling_areas.iter().fold(0., &areafn);
            // SupportPointGenerator.cpp:324
            let islands = s.dangling_areas.clone();
            let deficit = (a * tp as f64 - a * current as f64 * s.area as f64) as f32;
            self.uniformly_cover(&islands, s, deficit, grid3d, icfWithBoundary);
        }

        // SupportPointGenerator.cpp:327
        current = s.supports_force_total();
        // SupportPointGenerator.cpp:328-331
        if !s.overhangs_slopes.is_empty() {
            // SupportPointGenerator.cpp:329
            let a: f64 = s.overhangs_slopes.iter().fold(0., &areafn);
            // SupportPointGenerator.cpp:330
            let islands = s.overhangs_slopes.clone();
            let deficit = (a * tp as f64 - a * current as f64 / s.area as f64) as f32;
            self.uniformly_cover(&islands, s, deficit, grid3d, icfWithBoundary);
        }
    }

    // SupportPointGenerator.cpp:545-624
    // void SupportPointGenerator::uniformly_cover(const ExPolygons& islands,
    //         Structure& structure, float deficit, PointGrid3D &grid3d,
    //         IslandCoverageFlags flags)
    fn uniformly_cover(
        &mut self,
        islands: &[ExPolygon],
        structure: &mut Structure,
        deficit: f32,
        grid3d: &mut PointGrid3D,
        flags: IslandCoverageFlags,
    ) {
        // SupportPointGenerator.cpp:547 (commented out in C++):
        // int num_of_points = std::max(1, (int)((island.area()*pow(SCALING_FACTOR, 2) * m_config.tear_pressure)/m_config.support_force));

        // SupportPointGenerator.cpp:549
        let mut support_force_deficit: f32 = deficit;
        // SupportPointGenerator.cpp:550 — auto bb = get_extents(islands); (commented out)

        // SupportPointGenerator.cpp:552-561
        if flags & icfIsNew != 0 {
            // SupportPointGenerator.cpp:553
            let chull = convex_hull_expolygons(islands);
            // SupportPointGenerator.cpp:554
            let rotbox = MinAreaBoundigBox::from_polygon(&chull, PolygonLevel::PcConvex);
            // SupportPointGenerator.cpp:555 — Vec2d bbdim = {unscaled(rotbox.width()), unscaled(rotbox.height())};
            let mut bbdim_x: f64 = rotbox.width() * SCALING_FACTOR;
            let mut bbdim_y: f64 = rotbox.height() * SCALING_FACTOR;

            // SupportPointGenerator.cpp:557
            if bbdim_x > bbdim_y {
                std::mem::swap(&mut bbdim_x, &mut bbdim_y);
            }
            // SupportPointGenerator.cpp:558
            let aspectr: f64 = bbdim_y / bbdim_x;

            // SupportPointGenerator.cpp:560
            support_force_deficit = (support_force_deficit as f64 * (1. + aspectr / 2.)) as f32;
        }

        // SupportPointGenerator.cpp:563-564
        if support_force_deficit < 0. {
            return;
        }

        // SupportPointGenerator.cpp:566-567 — Number of newly added points.
        let poisson_samples_target: usize =
            ((support_force_deficit / self.m_config.support_force()) as f64).ceil() as usize;

        // SupportPointGenerator.cpp:569
        let density_horizontal: f32 =
            self.m_config.tear_pressure() / self.m_config.support_force();
        // SupportPointGenerator.cpp:570-571 — FIXME why?
        let mut poisson_radius: f32 =
            f32::max(self.m_config.minimal_distance, 1. / (5. * density_horizontal));
        // SupportPointGenerator.cpp:572 (commented out):
        // const float poisson_radius     = 1.f / (15.f * density_horizontal);
        // SupportPointGenerator.cpp:573
        let samples_per_mm2: f32 =
            30. / (std::f32::consts::PI * poisson_radius * poisson_radius);
        // SupportPointGenerator.cpp:574-576 — Minimum distance between samples, in 3D space.
        // float min_spacing = poisson_radius / 3.f; (commented out)
        let mut min_spacing: f32 = poisson_radius;

        // SupportPointGenerator.cpp:578 — FIXME share the random generator. The
        // random generator may be not so cheap to initialize, also we don't want
        // the random generator to be restarted for each polygon.

        // SupportPointGenerator.cpp:580-584
        let raw_samples: Vec<Vec2f> = if flags & icfWithBoundary != 0 {
            sample_expolygons_with_boundary(
                islands,
                samples_per_mm2,
                5. / poisson_radius,
                &mut self.m_rng,
            )
        } else {
            sample_expolygons(islands, samples_per_mm2, &mut self.m_rng)
        };

        // SupportPointGenerator.cpp:586-599
        let mut poisson_samples: Vec<Vec2f> = Vec::new();
        for _iter in 0..4 {
            // SupportPointGenerator.cpp:588-591
            poisson_samples = poisson_disk_from_samples(&raw_samples, poisson_radius, |pos| {
                grid3d.collides_with(pos, structure.layer_print_z as f32, min_spacing)
            });
            // SupportPointGenerator.cpp:592-593
            if poisson_samples.len() >= poisson_samples_target
                || (self.m_config.minimal_distance as f64) > (poisson_radius as f64) - EPSILON
            {
                break;
            }
            // SupportPointGenerator.cpp:594-596
            let mut coeff: f32 = 0.5;
            if poisson_samples.len() * 2 > poisson_samples_target {
                coeff = poisson_samples.len() as f32 / poisson_samples_target as f32;
            }
            // SupportPointGenerator.cpp:597-598
            poisson_radius = f32::max(self.m_config.minimal_distance, poisson_radius * coeff);
            min_spacing = f32::max(self.m_config.minimal_distance, min_spacing * coeff);
        }

        // SupportPointGenerator.cpp:601-612 — SLA_SUPPORTPOINTGEN_DEBUG only
        // (SVG dump of raw and poisson samples).

        // SupportPointGenerator.cpp:614 — assert(! poisson_samples.empty()); (commented out)
        // SupportPointGenerator.cpp:615-618
        if poisson_samples_target < poisson_samples.len() {
            shuffle(&mut poisson_samples, &mut self.m_rng);
            poisson_samples.truncate(poisson_samples_target); // erase(begin()+target, end())
        }
        // SupportPointGenerator.cpp:619-623
        for pt in &poisson_samples {
            // SupportPointGenerator.cpp:620
            self.m_output.push(SupportPoint::from_coords(
                pt.x,
                pt.y,
                structure.zlevel,
                self.m_config.head_diameter / 2.,
                flags & icfIsNew != 0,
            ));
            // SupportPointGenerator.cpp:621
            structure.supports_force_this_layer += self.m_config.support_force();
            // SupportPointGenerator.cpp:622
            grid3d.insert(pt, structure);
        }
    }
}

// SupportPointGenerator.cpp:117-225
// static std::vector<SupportPointGenerator::MyLayer> make_layers(
//     const std::vector<ExPolygons>& slices, const std::vector<float>& heights,
//     std::function<void(void)> throw_on_cancel)
fn make_layers<'slc>(
    slices: &'slc [ExPolygons],
    heights: &[f32],
    throw_on_cancel: &(dyn Fn() + Send + Sync),
) -> Vec<MyLayer<'slc>> {
    // SupportPointGenerator.cpp:121
    assert!(slices.len() == heights.len());

    // SupportPointGenerator.cpp:123-127 — Allocate empty layers.
    let mut layers: Vec<MyLayer<'slc>> = Vec::with_capacity(slices.len());
    for i in 0..slices.len() {
        layers.push(MyLayer::new(i, heights[i] as f64));
    }

    // SupportPointGenerator.cpp:129-131 — FIXME: calculate actual pixel area
    // from printer config:
    // const float pixel_area = pow(wxGetApp().preset_bundle->project_config.option<ConfigOptionFloat>("display_width") / wxGetApp().preset_bundle->project_config.option<ConfigOptionInt>("display_pixels_x"), 2.f);
    let pixel_area: f32 = 0.047f32.powi(2); // pow(0.047f, 2.f)

    // SupportPointGenerator.cpp:133-155
    let layers_len = layers.len();
    {
        let layers_ptr = SyncMutPtr(layers.as_mut_ptr());
        ccr_par::for_each(
            0,
            layers_len,
            |layer_id| {
                // SupportPointGenerator.cpp:136-139 — Don't call the following
                // function too often as it flushes CPU write caches due to
                // synchronization primitves.
                if layer_id % 8 == 0 {
                    throw_on_cancel();
                }

                // SupportPointGenerator.cpp:141-142
                // (raw pointer access: each layer_id is visited exactly once)
                let layer: &mut MyLayer<'slc> = unsafe { &mut *layers_ptr.get().add(layer_id) };
                let islands: &'slc ExPolygons = &slices[layer_id];
                // SupportPointGenerator.cpp:143-146 — FIXME WTF?
                let height: f32 = if layer_id > 2 {
                    heights[layer_id - 3]
                } else {
                    heights[0] - (heights[1] - heights[0])
                };
                // SupportPointGenerator.cpp:147
                layer.islands.reserve(islands.len());
                // SupportPointGenerator.cpp:148-154
                for island in islands {
                    let area: f32 = (island.area() * SCALING_FACTOR * SCALING_FACTOR) as f32;
                    if area >= pixel_area {
                        // FIXME this is not a correct centroid of a polygon with holes.
                        let centroid_pt = island.contour.centroid();
                        // unscaled<float>(...) = v.cast<float>() * float(SCALING_FACTOR)
                        let centroid = Vec2f::new(
                            centroid_pt.x as f32 * SCALING_FACTOR as f32,
                            centroid_pt.y as f32 * SCALING_FACTOR as f32,
                        );
                        layer.islands.push(Structure::new(
                            layer.layer_id,
                            layer.print_z,
                            island,
                            get_extents_polygons(slice::from_ref(&island.contour)),
                            centroid,
                            area,
                            height,
                        ));
                    }
                }
            },
            32, // SupportPointGenerator.cpp:155 — gransize
        );
    }

    // SupportPointGenerator.cpp:157-222 — Calculate overlap of successive layers.
    // Link overlapping islands.
    // NOTE: the C++ runs this with `ccr_par::for_each(1, layers.size(), ..., 8)`,
    // racing on *disjoint fields* of shared Structures (iteration N pushes into
    // layers[N-1].islands[*].islands_above while iteration N-1 owns the rest of
    // that Structure). Safe Rust cannot express that aliasing; since every field
    // is written by exactly one iteration, the loop is order-independent and is
    // executed sequentially here with identical results.
    for layer_id in 1..layers.len() {
        // SupportPointGenerator.cpp:161-163 — Don't call the following function
        // too often as it flushes CPU write caches due to synchronization primitves.
        if layer_id % 2 == 0 {
            throw_on_cancel();
        }
        // SupportPointGenerator.cpp:164-165
        let (left, right) = layers.split_at_mut(layer_id);
        let layer_below: &mut MyLayer = &mut left[layer_id - 1];
        let layer_above: &mut MyLayer = &mut right[0];
        // SupportPointGenerator.cpp:166-167 — FIXME WTF?
        let layer_height: f32 = if layer_id != 0 {
            heights[layer_id] - heights[layer_id - 1]
        } else {
            heights[0]
        };
        // SupportPointGenerator.cpp:168 — smaller number - less supports
        let safe_angle: f32 = 35. * (std::f32::consts::PI / 180.);
        // SupportPointGenerator.cpp:169 — scaled<float>(v) = v / float(SCALING_FACTOR)
        let between_layers_offset: f32 =
            (layer_height * safe_angle.tan()) / SCALING_FACTOR as f32;
        // SupportPointGenerator.cpp:170 — smaller number - less supports
        let slope_angle: f32 = 75. * (std::f32::consts::PI / 180.);
        // SupportPointGenerator.cpp:171
        let slope_offset: f32 = (layer_height * slope_angle.tan()) / SCALING_FACTOR as f32;
        // SupportPointGenerator.cpp:172 — FIXME This has a quadratic time
        // complexity, it will be excessively slow for many tiny islands.
        for (top_idx, top) in layer_above.islands.iter_mut().enumerate() {
            // SupportPointGenerator.cpp:174-180
            for (bottom_idx, bottom) in layer_below.islands.iter_mut().enumerate() {
                let overlap_area = top.overlap_area(bottom);
                if overlap_area > 0. {
                    top.islands_below.push(Link::new(bottom_idx, overlap_area));
                    bottom.islands_above.push(Link::new(top_idx, overlap_area));
                }
            }
            // SupportPointGenerator.cpp:181-220
            if !top.islands_below.is_empty() {
                // SupportPointGenerator.cpp:182
                let bottom_polygons: Polygons = top.polygons_below(&layer_below.islands);
                // NonZero reassembly of the raw Polygons clip (set-wise no-op,
                // see module porting notes).
                let bottom_polygons_ex: ExPolygons = union_polygons_ex(&bottom_polygons);
                // SupportPointGenerator.cpp:183 — top.overhangs = diff_ex(*top.polygon, bottom_polygons);
                top.overhangs = difference(slice::from_ref(top.polygon), &bottom_polygons_ex);
                // SupportPointGenerator.cpp:184
                if !top.overhangs.is_empty() {
                    // SupportPointGenerator.cpp:186-190 — Produce 2 bands around
                    // the island, a safe band for dangling overhangs and an
                    // unsafe band for sloped overhangs.
                    // These masks include the original island
                    // (expand(Polygons, delta, jtSquare); delta converted from
                    // scaled units to mm for the geo-clipper backend)
                    let mut dangl_mask: ExPolygons = offset_polygons(
                        &bottom_polygons,
                        between_layers_offset as f64 * SCALING_FACTOR,
                        OffsetJoinType::Square,
                    );
                    let mut overh_mask: ExPolygons = offset_polygons(
                        &bottom_polygons,
                        slope_offset as f64 * SCALING_FACTOR,
                        OffsetJoinType::Square,
                    );

                    // SupportPointGenerator.cpp:192-193 — Absolutely hopeless
                    // overhangs are those outside the unsafe band
                    top.overhangs = difference(slice::from_ref(top.polygon), &overh_mask);

                    // SupportPointGenerator.cpp:195-199 — Now cut out the
                    // supported core from the safe band and cut the safe band
                    // from the unsafe band to get distinct zones.
                    overh_mask = difference(&overh_mask, &dangl_mask);
                    dangl_mask = difference(&dangl_mask, &bottom_polygons_ex);

                    // SupportPointGenerator.cpp:201-202
                    top.dangling_areas = intersection(slice::from_ref(top.polygon), &dangl_mask);
                    top.overhangs_slopes = intersection(slice::from_ref(top.polygon), &overh_mask);

                    // SupportPointGenerator.cpp:204-210
                    top.overhangs_area = 0.;
                    let mut expolys_with_areas: Vec<(usize, f32)> = Vec::new();
                    for (idx, ex) in top.overhangs.iter().enumerate() {
                        let area = ex.area() as f32;
                        expolys_with_areas.push((idx, area));
                        top.overhangs_area += area;
                    }
                    // SupportPointGenerator.cpp:211-213
                    // { return p1.second > p2.second; }
                    expolys_with_areas.sort_by(|p1, p2| {
                        p2.1.partial_cmp(&p1.1).unwrap_or(std::cmp::Ordering::Equal)
                    });
                    // SupportPointGenerator.cpp:214-217
                    let mut overhangs_sorted: ExPolygons =
                        Vec::with_capacity(expolys_with_areas.len());
                    for &(idx, _) in &expolys_with_areas {
                        overhangs_sorted.push(std::mem::take(&mut top.overhangs[idx]));
                    }
                    top.overhangs = overhangs_sorted;
                    // SupportPointGenerator.cpp:218
                    top.overhangs_area *= (SCALING_FACTOR * SCALING_FACTOR) as f32;
                }
            }
        }
    }

    // SupportPointGenerator.cpp:224
    layers
}

// SupportPointGenerator.cpp:334-385
// std::vector<Vec2f> sample_expolygon(const ExPolygon &expoly, float samples_per_mm2, std::mt19937 &rng)
pub fn sample_expolygon(expoly: &ExPolygon, samples_per_mm2: f32, rng: &mut Mt19937) -> Vec<Vec2f> {
    // SupportPointGenerator.cpp:336-337 — Triangulate the polygon with holes
    // into triplets of 3D points.
    let triangles: Vec<Vec2f> = triangulate_expolygon_2f(expoly, NORMALS_UP)
        .unwrap_or_default()
        .iter()
        .map(|p| Vec2f::new(p.x, p.y))
        .collect();

    // SupportPointGenerator.cpp:339
    let mut out: Vec<Vec2f> = Vec::new();
    if !triangles.is_empty() {
        // SupportPointGenerator.cpp:342-353 — Calculate area of each triangle.
        let mut areas: Vec<f32> = Vec::with_capacity(triangles.len() / 3);
        let mut aback: f64 = 0.;
        let mut i: usize = 0;
        while i < triangles.len() {
            let a = triangles[i];
            i += 1;
            let v1 = triangles[i] - a;
            i += 1;
            let v2 = triangles[i] - a;
            i += 1;

            // SupportPointGenerator.cpp:350-352 — Prefix sum of the areas.
            areas.push((aback + (0.5f32 * cross2(&v1, &v2).abs()) as f64) as f32);
            aback = *areas.last().unwrap() as f64;
        }

        // SupportPointGenerator.cpp:355
        let num_samples: usize =
            ((*areas.last().unwrap() * samples_per_mm2) as f64).ceil() as usize;
        // SupportPointGenerator.cpp:356-357
        // std::uniform_real_distribution<> random_triangle(0., double(areas.back()));
        // std::uniform_real_distribution<> random_float(0., 1.);
        let areas_back: f64 = *areas.last().unwrap() as f64;
        for _ in 0..num_samples {
            // SupportPointGenerator.cpp:359
            let r: f64 = uniform_real(0., areas_back, rng);
            // SupportPointGenerator.cpp:360
            let upper = areas.partition_point(|&a| a <= r as f32); // std::upper_bound
            let mut idx_triangle: usize = upper.min(areas.len() - 1) * 3;
            // SupportPointGenerator.cpp:361-364 — Select a random point on the triangle.
            let a = triangles[idx_triangle];
            idx_triangle += 1;
            let b = triangles[idx_triangle];
            idx_triangle += 1;
            let c = triangles[idx_triangle];
            // SupportPointGenerator.cpp:365-370 (#if 1 branch)
            // https://www.cs.princeton.edu/~funk/tog02.pdf
            // page 814, formula 1.
            let u: f64 = (uniform_real(0., 1., rng).sqrt() as f32) as f64;
            let v: f64 = (uniform_real(0., 1., rng) as f32) as f64;
            out.push(
                a * ((1. - u) as f32) + b * ((u * (1. - v)) as f32) + c * ((v * u) as f32),
            );
            // SupportPointGenerator.cpp:371-381 (#else branch, not compiled):
            // // Greg Turk, Graphics Gems
            // // https://devsplorer.wordpress.com/2019/08/07/find-a-random-point-on-a-plane-using-barycentric-coordinates-in-unity/
            // double u = float(random_float(rng));
            // double v = float(random_float(rng));
            // if (u + v >= 1.f) { u = 1.f - u; v = 1.f - v; }
            // out.emplace_back(a + u * (b - a) + v * (c - a));
        }
    }
    // SupportPointGenerator.cpp:384
    out
}

// SupportPointGenerator.cpp:388-395
// std::vector<Vec2f> sample_expolygon(const ExPolygons &expolys, float samples_per_mm2, std::mt19937 &rng)
// (C++ overload of sample_expolygon; renamed for Rust)
pub fn sample_expolygons(
    expolys: &[ExPolygon],
    samples_per_mm2: f32,
    rng: &mut Mt19937,
) -> Vec<Vec2f> {
    // SupportPointGenerator.cpp:390-392
    let mut out: Vec<Vec2f> = Vec::new();
    for expoly in expolys {
        out.extend(sample_expolygon(expoly, samples_per_mm2, rng)); // append(out, ...)
    }
    // SupportPointGenerator.cpp:394
    out
}

// SupportPointGenerator.cpp:397-412
// void sample_expolygon_boundary(const ExPolygon &expoly, float samples_per_mm,
//                                std::vector<Vec2f> &out, std::mt19937 &/*rng*/)
pub fn sample_expolygon_boundary(
    expoly: &ExPolygon,
    samples_per_mm: f32,
    out: &mut Vec<Vec2f>,
    _rng: &mut Mt19937,
) {
    // SupportPointGenerator.cpp:402 — double point_stepping_scaled = scale_(1.f) / samples_per_mm;
    let point_stepping_scaled: f64 = (1.0f64 / SCALING_FACTOR) / samples_per_mm as f64;
    // SupportPointGenerator.cpp:403-405
    for i_contour in 0..=expoly.holes.len() {
        let contour: &Polygon = if i_contour == 0 {
            &expoly.contour
        } else {
            &expoly.holes[i_contour - 1]
        };

        // SupportPointGenerator.cpp:407
        let pts = contour.equally_spaced_points(point_stepping_scaled);
        // SupportPointGenerator.cpp:408-410 — unscale<float>(v) = float(v) * float(SCALING_FACTOR)
        for pt in &pts {
            out.push(Vec2f::new(
                pt.x as f32 * SCALING_FACTOR as f32,
                pt.y as f32 * SCALING_FACTOR as f32,
            ));
        }
    }
}

// SupportPointGenerator.cpp:414-419
// std::vector<Vec2f> sample_expolygon_with_boundary(const ExPolygon &expoly,
//     float samples_per_mm2, float samples_per_mm_boundary, std::mt19937 &rng)
pub fn sample_expolygon_with_boundary(
    expoly: &ExPolygon,
    samples_per_mm2: f32,
    samples_per_mm_boundary: f32,
    rng: &mut Mt19937,
) -> Vec<Vec2f> {
    // SupportPointGenerator.cpp:416
    let mut out = sample_expolygon(expoly, samples_per_mm2, rng);
    // SupportPointGenerator.cpp:417
    sample_expolygon_boundary(expoly, samples_per_mm_boundary, &mut out, rng);
    // SupportPointGenerator.cpp:418
    out
}

// SupportPointGenerator.cpp:421-427
// std::vector<Vec2f> sample_expolygon_with_boundary(const ExPolygons &expolys, ...)
// (C++ overload; renamed for Rust)
pub fn sample_expolygons_with_boundary(
    expolys: &[ExPolygon],
    samples_per_mm2: f32,
    samples_per_mm_boundary: f32,
    rng: &mut Mt19937,
) -> Vec<Vec2f> {
    // SupportPointGenerator.cpp:423-425
    let mut out: Vec<Vec2f> = Vec::new();
    for expoly in expolys {
        out.extend(sample_expolygon_with_boundary(
            expoly,
            samples_per_mm2,
            samples_per_mm_boundary,
            rng,
        )); // append(out, ...)
    }
    // SupportPointGenerator.cpp:426
    out
}

// SupportPointGenerator.cpp:429-542
// template<typename REFUSE_FUNCTION>
// static inline std::vector<Vec2f> poisson_disk_from_samples(const std::vector<Vec2f> &raw_samples,
//                                                            float radius, REFUSE_FUNCTION refuse_function)
fn poisson_disk_from_samples<F>(raw_samples: &[Vec2f], radius: f32, refuse_function: F) -> Vec<Vec2f>
where
    F: Fn(&Vec2f) -> bool,
{
    // SupportPointGenerator.cpp:432-436
    let mut corner_min = Vec2f::new(f32::MAX, f32::MAX);
    for pt in raw_samples {
        corner_min.x = f32::min(corner_min.x, pt.x);
        corner_min.y = f32::min(corner_min.y, pt.y);
    }

    // SupportPointGenerator.cpp:438-444 — Assign the raw samples to grid cells,
    // sort the grid cells lexicographically.
    struct RawSample {
        coord: Vec2f,
        cell_id: Vec2i,
    }

    // SupportPointGenerator.cpp:446-448
    let mut raw_samples_sorted: Vec<RawSample> = Vec::with_capacity(raw_samples.len());
    for pt in raw_samples {
        let c = (pt - corner_min) / radius;
        raw_samples_sorted.push(RawSample {
            coord: *pt,
            cell_id: Vec2i::new(c.x as i32, c.y as i32), // .cast<int>() truncates
        });
    }

    // SupportPointGenerator.cpp:450-451
    // { return lhs.cell_id.x() < rhs.cell_id.x() || (lhs.cell_id.x() == rhs.cell_id.x() && lhs.cell_id.y() < rhs.cell_id.y()); }
    // (C++ std::sort is unstable — tie order among equal cell ids is
    // implementation-defined there; Rust's stable sort is one valid outcome)
    raw_samples_sorted
        .sort_by(|lhs, rhs| (lhs.cell_id.x, lhs.cell_id.y).cmp(&(rhs.cell_id.x, rhs.cell_id.y)));

    // SupportPointGenerator.cpp:453-464
    #[derive(Clone, Copy)]
    struct PoissonDiskGridEntry {
        // Resulting output sample points for this cell:
        // SupportPointGenerator.cpp:455-457 — enum { max_positions = 4 };
        poisson_samples: [Vec2f; MAX_POSITIONS],
        num_poisson_samples: i32,
        // Index into raw_samples:
        first_sample_idx: i32,
        sample_cnt: i32,
    }
    const MAX_POSITIONS: usize = 4;

    // SupportPointGenerator.cpp:466-470 — struct CellIDHash { ... } — bucket
    // hash only; not needed for the HashMap port.

    // SupportPointGenerator.cpp:472-496 — Map from cell IDs to hash_data. Each
    // hash_data points to the range in raw_samples corresponding to that cell.
    // (We could just store the samples in hash_data. This implementation is an
    // artifact of the reference paper, which is optimizing for GPU acceleration
    // that we haven't implemented currently.)
    // NOTE: unordered_map iteration order is implementation-defined in C++; the
    // port iterates cells in insertion order via `cell_order` (deterministic).
    let mut cells: HashMap<(i32, i32), PoissonDiskGridEntry> = HashMap::new();
    let mut cell_order: Vec<(i32, i32)> = Vec::new();
    {
        let mut last_cell_id = Vec2i::new(-1, -1);
        for (i, sample) in raw_samples_sorted.iter().enumerate() {
            if sample.cell_id == last_cell_id {
                // SupportPointGenerator.cpp:482-485 — This sample is in the same
                // cell as the previous, so just increase the count. Cells are
                // always contiguous, since we've sorted raw_samples_sorted by cell ID.
                cells
                    .get_mut(&(last_cell_id.x, last_cell_id.y))
                    .unwrap()
                    .sample_cnt += 1;
            } else {
                // SupportPointGenerator.cpp:486-494 — This is a new cell.
                let data = PoissonDiskGridEntry {
                    poisson_samples: [Vec2f::zeros(); MAX_POSITIONS],
                    num_poisson_samples: 0,
                    first_sample_idx: i as i32,
                    sample_cnt: 1,
                };
                cells.insert((sample.cell_id.x, sample.cell_id.y), data);
                cell_order.push((sample.cell_id.x, sample.cell_id.y));
                last_cell_id = sample.cell_id;
            }
        }
    }

    // SupportPointGenerator.cpp:498-499
    const MAX_TRIALS: i32 = 5;
    let radius_squared = radius * radius;
    // SupportPointGenerator.cpp:500-534
    for trial in 0..MAX_TRIALS {
        // Create sample points for each entry in cells.
        for cell_id in &cell_order {
            let (first_sample_idx, sample_cnt) = {
                let cell_data = &cells[cell_id];
                (cell_data.first_sample_idx, cell_data.sample_cnt)
            };
            // SupportPointGenerator.cpp:505-509 — This cell's raw sample points
            // start at first_sample_idx. On trial 0, try the first one. On trial
            // 1, try first_sample_idx + 1.
            let next_sample_idx = first_sample_idx + trial;
            if trial >= sample_cnt {
                // There are no more points to try for this cell.
                continue;
            }
            // SupportPointGenerator.cpp:510
            let candidate_coord = raw_samples_sorted[next_sample_idx as usize].coord;
            // SupportPointGenerator.cpp:511-526 — See if this point conflicts
            // with any other points in this cell, or with any points in
            // neighboring cells. Note that it's possible to have more than one
            // point in the same cell.
            let mut conflict = refuse_function(&candidate_coord);
            let mut i: i32 = -1;
            while i < 2 && !conflict {
                for j in -1..2 {
                    if let Some(neighbor) = cells.get(&(cell_id.0 + i, cell_id.1 + j)) {
                        for i_sample in 0..neighbor.num_poisson_samples {
                            if (neighbor.poisson_samples[i_sample as usize] - candidate_coord)
                                .norm_squared()
                                < radius_squared
                            {
                                conflict = true;
                                break;
                            }
                        }
                    }
                }
                i += 1;
            }
            // SupportPointGenerator.cpp:527-532
            if !conflict {
                // Store the new sample.
                let cell_data = cells.get_mut(cell_id).unwrap();
                debug_assert!(cell_data.num_poisson_samples < MAX_POSITIONS as i32);
                if cell_data.num_poisson_samples < MAX_POSITIONS as i32 {
                    cell_data.poisson_samples[cell_data.num_poisson_samples as usize] =
                        candidate_coord;
                    cell_data.num_poisson_samples += 1;
                }
            }
        }
    }

    // SupportPointGenerator.cpp:536-541 — Copy the results to the output.
    let mut out: Vec<Vec2f> = Vec::new();
    for cell_id in &cell_order {
        let cell_data = &cells[cell_id];
        for i in 0..cell_data.num_poisson_samples {
            out.push(cell_data.poisson_samples[i as usize]);
        }
    }
    out
}

// SupportPointGenerator.cpp:627-637
// void remove_bottom_points(std::vector<SupportPoint> &pts, float lvl)
pub fn remove_bottom_points(pts: &mut Vec<SupportPoint>, lvl: f32) {
    // SupportPointGenerator.cpp:629-636 — get iterator to the reorganized vector
    // end (std::remove_if with `sp.pos.z() <= lvl`), erase all elements after
    // the new end. `Vec::retain` keeps the relative order, like remove_if.
    pts.retain(|sp| !(sp.pos.z <= lvl));
}

// SupportPointGenerator.cpp:639-665 — #ifdef SLA_SUPPORTPOINTGEN_DEBUG (not
// defined; the debug-only `output_structures` / `output_expolygons` SVG dump
// helpers are not compiled in C++ either):
// void SupportPointGenerator::output_structures(const std::vector<Structure>& structures)
// void SupportPointGenerator::output_expolygons(const ExPolygons& expolys, const std::string &filename)

// ===========================================================================
// std <random> support — faithful ports of the libstdc++ facilities used by
// SupportPointGenerator.cpp: std::mt19937, std::uniform_real_distribution<>,
// std::uniform_int_distribution (via std::shuffle), std::shuffle and a
// std::random_device stand-in.
// ===========================================================================

/// `std::mt19937` — Mersenne Twister with the standard 32-bit parameters
/// (same engine as `circle.rs`; duplicated there as a private type).
pub struct Mt19937 {
    mt: [u32; Self::N],
    index: usize,
}

impl Mt19937 {
    const N: usize = 624;
    const M: usize = 397;
    const MATRIX_A: u32 = 0x9908_b0df;
    const UPPER_MASK: u32 = 0x8000_0000;
    const LOWER_MASK: u32 = 0x7fff_ffff;
    /// `std::mt19937::default_seed`
    const DEFAULT_SEED: u32 = 5489;

    /// Default constructor (seed 5489).
    pub fn new() -> Self {
        Self::with_seed(Self::DEFAULT_SEED)
    }

    /// Seeding per the standard `mersenne_twister_engine` initialization.
    pub fn with_seed(seed: u32) -> Self {
        let mut mt = [0u32; Self::N];
        mt[0] = seed;
        for i in 1..Self::N {
            mt[i] = (1_812_433_253u32.wrapping_mul(mt[i - 1] ^ (mt[i - 1] >> 30)))
                .wrapping_add(i as u32);
        }
        Self { mt, index: Self::N }
    }

    /// `std::mt19937::seed(result_type)` — reinitializes the state.
    pub fn seed(&mut self, s: u32) {
        *self = Self::with_seed(s);
    }

    fn generate(&mut self) {
        for i in 0..Self::N {
            let y = (self.mt[i] & Self::UPPER_MASK) | (self.mt[(i + 1) % Self::N] & Self::LOWER_MASK);
            let mut next = self.mt[(i + Self::M) % Self::N] ^ (y >> 1);
            if y & 1 != 0 {
                next ^= Self::MATRIX_A;
            }
            self.mt[i] = next;
        }
        self.index = 0;
    }

    /// `operator()` — next 32-bit value with the standard tempering.
    pub fn next_u32(&mut self) -> u32 {
        if self.index >= Self::N {
            self.generate();
        }
        let mut y = self.mt[self.index];
        self.index += 1;
        y ^= y >> 11;
        y ^= (y << 7) & 0x9d2c_5680;
        y ^= (y << 15) & 0xefc6_0000;
        y ^= y >> 18;
        y
    }
}

impl Default for Mt19937 {
    fn default() -> Self {
        Self::new()
    }
}

/// `std::random_device` stand-in (SupportPointGenerator.cpp:64).
///
/// NATIVE-DEP NOTE: `std::random_device` draws OS entropy and is
/// nondeterministic by design; the exact bit stream cannot (and need not) be
/// reproduced. Mirroring fuzzy_skin.rs, the seed is derived from a hash of the
/// thread id combined with the current time, avoiding any system entropy
/// dependency.
fn random_device_substitute() -> u32 {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};
    use std::thread;
    use std::time::{SystemTime, UNIX_EPOCH};

    let mut hasher = DefaultHasher::new();
    thread::current().id().hash(&mut hasher);
    let thread_hash = hasher.finish();
    let time_seed = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0x5DEECE66D);
    let seed = thread_hash ^ time_seed;
    (seed ^ (seed >> 32)) as u32
}

/// libstdc++ `std::generate_canonical<double, 53, std::mt19937>` — for a 32-bit
/// engine, `__k = ceil(53 / 32) = 2` draws are consumed:
/// `__ret = (draw1 + draw2 * 2^32) / 2^64`, clamped to `nextafter(1, 0)`.
fn generate_canonical_f64_53(urng: &mut Mt19937) -> f64 {
    // __r = (urng.max() - urng.min()) + 1 == 2^32
    let r: f64 = 4_294_967_296.0; // 2^32
    let mut sum: f64 = 0.;
    let mut tmp: f64 = 1.;
    for _k in 0..2 {
        sum += urng.next_u32() as f64 * tmp;
        tmp *= r;
    }
    let mut ret = sum / tmp;
    if ret >= 1. {
        // std::nextafter(1.0, 0.0) == 1 - 2^-53
        ret = 1.0 - f64::EPSILON / 2.0;
    }
    ret
}

/// libstdc++ `std::uniform_real_distribution<double>(a, b)::operator()(urng)`:
/// `(__aurng() * (b - a)) + a` where `__aurng` is `generate_canonical<double, 53>`.
fn uniform_real(a: f64, b: f64, urng: &mut Mt19937) -> f64 {
    generate_canonical_f64_53(urng) * (b - a) + a
}

/// libstdc++ `std::uniform_int_distribution<uint64_t>{0, urange}(mt19937)` —
/// the rejection-sampling implementation from `bits/uniform_int_dist.h`.
fn uniform_int_u64(g: &mut Mt19937, urange: u64) -> u64 {
    let urngrange: u64 = u32::MAX as u64; // engine max - min
    if urngrange > urange {
        // downscaling
        let uerange = urange + 1; // __urange can be zero
        let scaling = urngrange / uerange;
        let past = uerange * scaling;
        let mut ret: u64;
        loop {
            ret = g.next_u32() as u64;
            if ret < past {
                break;
            }
        }
        ret / scaling
    } else if urngrange < urange {
        // upscaling
        // Note that every value in [0, n] can be written uniquely as
        // g * (s + 1) + b, where g in [0, n/(s+1)], and b in [0, s].
        let mut ret: u64;
        loop {
            let uerngrange = urngrange.wrapping_add(1); // 2^32
            let tmp = uerngrange.wrapping_mul(uniform_int_u64(g, urange / uerngrange));
            ret = tmp.wrapping_add(g.next_u32() as u64);
            if !(ret > urange || ret < tmp) {
                break;
            }
        }
        ret
    } else {
        g.next_u32() as u64
    }
}

/// libstdc++ `std::__gen_two_uniform_ints(b0, b1, g)` (bits/stl_algo.h):
/// one distribution invocation producing two swap positions.
fn gen_two_uniform_ints(b0: u64, b1: u64, g: &mut Mt19937) -> (u64, u64) {
    let x = uniform_int_u64(g, b0 * b1 - 1);
    (x / b1, x % b1)
}

/// libstdc++ `std::shuffle(first, last, g)` (bits/stl_algo.h) with the
/// two-uniform-ints fast path.
fn shuffle(v: &mut [Vec2f], g: &mut Mt19937) {
    if v.is_empty() {
        return;
    }
    let urngrange: u64 = u32::MAX as u64; // __g.max() - __g.min()
    let urange: u64 = v.len() as u64;

    if urngrange / urange >= urange
    // I.e. (__urngrange >= __urange * __urange) but without wrapping issues.
    {
        let mut i: usize = 1;

        // Since we know the range isn't empty, an even number of elements
        // means an uneven number of elements /to swap/, in which case we
        // do the first one up front:
        if urange % 2 == 0 {
            let d = uniform_int_u64(g, 1); // __distr_type __d{0, 1}
            v.swap(i, d as usize);
            i += 1;
        }

        // Now we know that __last - __i is even, so we do the rest in pairs,
        // using a single distribution invocation to produce swap positions
        // for two successive elements at a time:
        while i != v.len() {
            let swap_range: u64 = i as u64 + 1;
            let (pos0, pos1) = gen_two_uniform_ints(swap_range, swap_range + 1, g);
            v.swap(i, pos0 as usize);
            i += 1;
            v.swap(i, pos1 as usize);
            i += 1;
        }

        return;
    }

    for i in 1..v.len() {
        let d = uniform_int_u64(g, i as u64); // __d(__g, __p_type(0, __i - __first))
        v.swap(i, d as usize);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// std::mt19937 with default seed: the 10000th draw must be 4123659995
    /// (well-known reference value, also asserted by the C++ standard).
    #[test]
    fn test_mt19937_reference_value() {
        let mut rng = Mt19937::new();
        let mut last = 0u32;
        for _ in 0..10000 {
            last = rng.next_u32();
        }
        assert_eq!(last, 4123659995);
    }

    #[test]
    fn test_uniform_real_range() {
        let mut rng = Mt19937::with_seed(42);
        for _ in 0..1000 {
            let v = uniform_real(0., 1., &mut rng);
            assert!((0. ..1.).contains(&v));
        }
    }

    #[test]
    fn test_shuffle_is_permutation() {
        let mut rng = Mt19937::with_seed(7);
        let mut v: Vec<Vec2f> = (0..37).map(|i| Vec2f::new(i as f32, 0.)).collect();
        shuffle(&mut v, &mut rng);
        let mut xs: Vec<i32> = v.iter().map(|p| p.x as i32).collect();
        xs.sort_unstable();
        assert_eq!(xs, (0..37).collect::<Vec<i32>>());
    }

    #[test]
    fn test_remove_bottom_points() {
        // SupportPointGenerator.cpp:627-637
        let mut pts = vec![
            SupportPoint::from_coords(0., 0., 1.0, 0.2, false),
            SupportPoint::from_coords(0., 0., 0.0, 0.2, false),
            SupportPoint::from_coords(0., 0., 2.0, 0.2, false),
        ];
        remove_bottom_points(&mut pts, 0.5);
        assert_eq!(pts.len(), 2);
        assert!(pts.iter().all(|sp| sp.pos.z > 0.5));
    }

    #[test]
    fn test_point_grid3d_collision() {
        // SupportPointGenerator.hpp:140-191
        let mut grid = PointGrid3D::default();
        grid.cell_size = Vec3f::new(10., 10., 10.);
        let expoly = ExPolygon::default();
        let s = Structure::new(0, 5.0, &expoly, BoundingBox::default(), Vec2f::zeros(), 1., 5.);
        grid.insert(&Vec2f::new(1., 1.), &s);
        assert!(grid.collides_with(&Vec2f::new(1.2, 1.2), 5.0, 1.0));
        assert!(!grid.collides_with(&Vec2f::new(9., 9.), 5.0, 1.0));
    }
}
