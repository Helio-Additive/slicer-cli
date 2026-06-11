//! Faithful port of SLA/SupportTreeBuildsteps.{hpp,cpp}.
//!
//! C++ Reference:
//! - SLA/SupportTreeBuildsteps.hpp (379 lines)
//! - SLA/SupportTreeBuildsteps.cpp (1278 lines)
//!
//! C++ includes (SupportTreeBuildsteps.cpp:1-5): SupportTreeBuildsteps.hpp,
//! SpatIndex.hpp, Optimize/NLoptOptimizer.hpp, boost/log/trivial.hpp.
//! Header includes (SupportTreeBuildsteps.hpp:4-9): cstdint, optional,
//! SupportTreeBuilder.hpp, Clustering.hpp, SpatIndex.hpp.
//!
//! Fidelity notes:
//! - Two `Vec3d` representations exist in the crate: the geometry `Vec3d`
//!   (struct with `x/y/z` fields, used by SupportTreeBuilder/SpatIndex/
//!   Clustering) and the nalgebra `Vector3<f64>` used by the IndexedMesh API.
//!   Both mirror Eigen `Vec3d`; `to_na`/`from_na` below convert losslessly at
//!   the IndexedMesh boundary.
//! - Eigen's `.normalized()` divides by the norm unguarded; the crate
//!   `Vec3::normalized()` has an epsilon guard, so `eigen_normalized` divides
//!   explicitly (same values for every non-degenerate vector).
//! - BLOCKED BACKEND DIVERGENCE: the C++ optimizer calls (`AlgNLoptSubplex` /
//!   `AlgNLoptGenetic`, SupportTreeBuildsteps.cpp:589,714,925) run the native
//!   NLopt C library, which is a non-wasm-safe native dependency that the
//!   crate intentionally does not link (see optimize/n_lopt_optimizer.rs).
//!   The optimizer setup (algorithm, criteria, seed, bounds, initvals,
//!   objective closures) is translated faithfully and invoked through the
//!   faithful `NLoptAlg*Optimizer` API; when the backend reports
//!   `NLoptBackendError`, `opt_result_or_backend_unavailable` substitutes the
//!   documented degenerate result {optimum = initvals (exactly what the C++
//!   seeds into `r.optimum` before `nlopt_optimize`, NLoptOptimizer.hpp:142),
//!   score = NaN} so every `oresult.score > x` / `>= x` test is false, i.e.
//!   "the search found no improvement". No substitute solver is fabricated.
//! - The C++ `ccr::for_each` parallel loops are reproduced with the crate
//!   `ccr` facade where safe Rust can express the aliasing (the ray-sampling
//!   loops over disjoint `hits[i]`, via the index-tagged-buffer precedent from
//!   indexed_mesh.rs); the loops whose bodies mutate the shared builder under
//!   C++ mutexes (filter dispatch cpp:751, routing_to_model cpp:1040) are
//!   executed sequentially — a valid interleaving of the C++ parallel loop —
//!   because `&mut self` cannot be shared across threads (crate precedent:
//!   support_point_generator.rs).
//! - `BOOST_LOG_TRIVIAL(...)` maps to the `log` crate macros (crate precedent:
//!   sla/pad.rs).
//! - C++ default arguments (`norm = 1.`, `safety_d`, `head_id = ID_UNSET`,
//!   `base_en = true`) cannot be expressed in Rust; callers pass them
//!   explicitly / the defaulted overloads carry a `_default_sd` suffix.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use crate::geometry::{Vec2d, Vec3d};
use crate::libslic3r::EPSILON;
use crate::mt_utils::linspace_array;
use crate::optimize::optimizer::{bounds, initvals};
use crate::optimize::{
    alg_nlopt_genetic, alg_nlopt_subplex, Bound, Input, NLoptAlgCombOptimizer, NLoptAlgOptimizer,
    NLoptBackendError, OptResult, StopCriteria,
};
use crate::sla::ccr;
use crate::sla::clustering::{cluster_by_predicate, cluster_centroid, cluster_points, ClusterEl};
use crate::sla::indexed_mesh::{hit_result, normals, IndexedMesh, PointSet, Vec3d as NaVec3d};
use crate::sla::job_controller::CancelFn;
use crate::sla::spat_index::{PointIndex, PointIndexEl};
use crate::sla::support_point::SupportPoints;
use crate::sla::support_tree::{PillarConnectionMode, SupportTreeConfig, SupportableMesh};
use crate::sla::support_tree_builder::{
    distance_between, Anchor, Bridge, DiffBridge, Head, Junction, Pillar, SupportTreeBuilder,
    DOWN, ID_UNSET,
};

// libslic3r.h — `PI` (M_PI).
const PI: f64 = std::f64::consts::PI;

// SLA/SupportTree.hpp `using ThrowOnCancel = std::function<void(void)>;`
// (the type of the `m_thr` member, SupportTreeBuildsteps.hpp:217; aliased to
// the JobController CancelFn it is copied from at cpp:43)
type ThrowOnCancel = CancelFn;

// ---------------------------------------------------------------------------
// Representation bridges (see module notes)
// ---------------------------------------------------------------------------

/// geometry `Vec3d` -> nalgebra `Vec3d` (IndexedMesh API boundary).
#[inline]
fn to_na(v: &Vec3d) -> NaVec3d {
    NaVec3d::new(v.x, v.y, v.z)
}

/// nalgebra `Vec3d` -> geometry `Vec3d`.
#[inline]
fn from_na(v: &NaVec3d) -> Vec3d {
    Vec3d::new(v[0], v[1], v[2])
}

/// Eigen `.normalized()` — unguarded division by the norm (the crate
/// `Vec3::normalized()` has an epsilon guard; see module notes).
#[inline]
fn eigen_normalized(v: Vec3d) -> Vec3d {
    v / v.norm()
}

// SupportTreeBuilder.hpp:53-60 — the `distance` templates, monomorphized for
// Vec2d (the Vec3d instantiations live in sla/support_tree_builder.rs).
#[inline]
fn distance_vec2(p: &Vec2d) -> f64 {
    // SupportTreeBuilder.hpp:54 — std::sqrt(p.transpose() * p)
    (p.x * p.x + p.y * p.y).sqrt()
}

#[inline]
fn distance_between_vec2(pp1: &Vec2d, pp2: &Vec2d) -> f64 {
    // SupportTreeBuilder.hpp:58-59
    let p = *pp2 - *pp1;
    distance_vec2(&p)
}

/// Point.hpp — `to_2d(const Vec<3,T>&)` (head of the vector), as called at
/// SupportTreeBuildsteps.cpp:400,808.
#[inline]
fn to_2d(v: &Vec3d) -> Vec2d {
    Vec2d::new(v.x, v.y)
}

/// Backend-unavailable fallback for the BLOCKED native NLopt solver — see the
/// module notes ("BLOCKED BACKEND DIVERGENCE"). `optimum` carries `initvals`
/// exactly as the C++ seeds `r.optimum` before `nlopt_optimize`
/// (NLoptOptimizer.hpp:142); `score = NaN` makes every score comparison false.
/// `result_code = -1` is `NLOPT_FAILURE`.
fn opt_result_or_backend_unavailable<const N: usize>(
    res: Result<OptResult<N>, NLoptBackendError>,
    iv: &Input<N>,
) -> OptResult<N> {
    match res {
        Ok(r) => r,
        Err(_) => OptResult {
            result_code: -1, // NLOPT_FAILURE
            optimum: *iv,
            score: f64::NAN,
        },
    }
}

// ---------------------------------------------------------------------------
// SupportTreeBuildsteps.hpp
// ---------------------------------------------------------------------------

// The minimum distance for two support points to remain valid.
// SupportTreeBuildsteps.hpp:15 — const double /*constexpr*/ D_SP = 0.1;
pub const D_SP: f64 = 0.1;

// SupportTreeBuildsteps.hpp:17-19
// enum { // For indexing Eigen vectors as v(X), v(Y), v(Z) instead of numbers
//     X, Y, Z
// };
pub const X: usize = 0;
pub const Y: usize = 1;
pub const Z: usize = 2;

// SupportTreeBuildsteps.hpp:21
// inline Vec2d to_vec2(const Vec3d &v3) { return {v3(X), v3(Y)}; }
#[inline]
pub fn to_vec2(v3: &Vec3d) -> Vec2d {
    Vec2d::new(v3.x, v3.y)
}

// SupportTreeBuildsteps.hpp:23-30
// inline std::pair<double, double> dir_to_spheric(const Vec3d &n, double norm = 1.)
// (C++ default argument `norm = 1.`; callers pass it explicitly.)
#[inline]
pub fn dir_to_spheric(n: &Vec3d, norm: f64) -> (f64, f64) {
    // SupportTreeBuildsteps.hpp:25
    let z = n.z;
    // SupportTreeBuildsteps.hpp:26
    let r = norm;
    // SupportTreeBuildsteps.hpp:27
    let polar = (z / r).acos();
    // SupportTreeBuildsteps.hpp:28
    let azimuth = n.y.atan2(n.x);
    // SupportTreeBuildsteps.hpp:29
    (polar, azimuth)
}

// SupportTreeBuildsteps.hpp:32-36
// inline Vec3d spheric_to_dir(double polar, double azimuth)
#[inline]
pub fn spheric_to_dir(polar: f64, azimuth: f64) -> Vec3d {
    // SupportTreeBuildsteps.hpp:34-35
    Vec3d::new(
        azimuth.cos() * polar.sin(),
        azimuth.sin() * polar.sin(),
        polar.cos(),
    )
}

// SupportTreeBuildsteps.hpp:38-42
// inline Vec3d spheric_to_dir(const std::tuple<double, double> &v)
// SupportTreeBuildsteps.hpp:44-47
// inline Vec3d spheric_to_dir(const std::pair<double, double> &v)
// (C++ tuple and pair overloads — both are `(f64, f64)` in Rust.)
#[inline]
pub fn spheric_to_dir_tuple(v: (f64, f64)) -> Vec3d {
    // SupportTreeBuildsteps.hpp:40-41 / hpp:46
    let (plr, azm) = v;
    spheric_to_dir(plr, azm)
}

// SupportTreeBuildsteps.hpp:49-52
// inline Vec3d spheric_to_dir(const std::array<double, 2> &v)
#[inline]
pub fn spheric_to_dir_arr(v: &[f64; 2]) -> Vec3d {
    // SupportTreeBuildsteps.hpp:51
    spheric_to_dir(v[0], v[1])
}

/// Give points on a 3D ring with given center, radius and orientation
/// method based on:
/// https://math.stackexchange.com/questions/73237/parametric-equation-of-a-circle-in-3d-space
// SupportTreeBuildsteps.hpp:57-110 — template<size_t N> class PointRing
pub struct PointRing<const N: usize> {
    // SupportTreeBuildsteps.hpp:59 — std::array<double, N> m_phis;
    m_phis: [f64; N],

    // Two vectors that will be perpendicular to each other and to the
    // axis. Values for a(X) and a(Y) are now arbitrary, a(Z) is just a
    // placeholder.
    // a and b vectors are perpendicular to the ring direction and to each other.
    // Together they define the plane where we have to iterate with the
    // given angles in the 'm_phis' vector
    // SupportTreeBuildsteps.hpp:67 — Vec3d a = {0, 1, 0}, b;
    a: Vec3d,
    b: Vec3d,
    // SupportTreeBuildsteps.hpp:68 — double m_radius = 0.;
    #[allow(dead_code)]
    m_radius: f64,
}

impl<const N: usize> PointRing<N> {
    // SupportTreeBuildsteps.hpp:70-73
    // static inline bool constexpr is_one(double val)
    // {
    //     return std::abs(std::abs(val) - 1) < 1e-20;
    // }
    #[inline]
    fn is_one(val: f64) -> bool {
        (val.abs() - 1.0).abs() < 1e-20
    }

    // SupportTreeBuildsteps.hpp:77-94 — PointRing(const Vec3d &n)
    pub fn new(n: &Vec3d) -> Self {
        // SupportTreeBuildsteps.hpp:67 — Vec3d a = {0, 1, 0}, b;
        let mut a = Vec3d::new(0.0, 1.0, 0.0);
        let b;

        // SupportTreeBuildsteps.hpp:79
        let m_phis: [f64; N] = linspace_array::<N, f64>(0.0, 2.0 * PI);

        // We have to address the case when the direction vector v (same as
        // dir) is coincident with one of the world axes. In this case two of
        // its components will be completely zero and one is 1.0. Our method
        // becomes dangerous here due to division with zero. Instead, vector
        // 'a' can be an element-wise rotated version of 'v'
        // SupportTreeBuildsteps.hpp:86-93
        if Self::is_one(n.x) || Self::is_one(n.y) || Self::is_one(n.z) {
            // SupportTreeBuildsteps.hpp:87-88
            a = Vec3d::new(n.z, n.x, n.y);
            b = Vec3d::new(n.y, n.z, n.x);
        } else {
            // SupportTreeBuildsteps.hpp:91 — a(Z) = -(n(Y)*a(Y)) / n(Z); a.normalize();
            a.z = -(n.y * a.y) / n.z;
            a = eigen_normalized(a);
            // SupportTreeBuildsteps.hpp:92 — b = a.cross(n);
            b = a.cross(n);
        }

        Self {
            m_phis,
            a,
            b,
            // SupportTreeBuildsteps.hpp:68
            m_radius: 0.0,
        }
    }

    // SupportTreeBuildsteps.hpp:96-109
    // Vec3d get(size_t idx, const Vec3d src, double r) const
    pub fn get(&self, idx: usize, src: &Vec3d, r: f64) -> Vec3d {
        // SupportTreeBuildsteps.hpp:98-100
        let phi = self.m_phis[idx];
        let sinphi = phi.sin();
        let cosphi = phi.cos();

        // SupportTreeBuildsteps.hpp:102-103
        let rpscos = r * cosphi;
        let rpssin = r * sinphi;

        // Point on the sphere
        // SupportTreeBuildsteps.hpp:106-108
        Vec3d::new(
            src.x + rpscos * self.a.x + rpssin * self.b.x,
            src.y + rpscos * self.a.y + rpssin * self.b.y,
            src.z + rpscos * self.a.z + rpssin * self.b.z,
        )
    }
}

// SupportTreeBuildsteps.hpp:112-113 — commented-out query_hit declarations
// (kept commented out in the C++ source).

// SupportTreeBuildsteps.hpp:115-117
// inline Vec3d dirv(const Vec3d& startp, const Vec3d& endp) {
//     return (endp - startp).normalized();
// }
#[inline]
pub fn dirv(startp: &Vec3d, endp: &Vec3d) -> Vec3d {
    eigen_normalized(*endp - *startp)
}

// SupportTreeBuildsteps.hpp:121 — using Mutex = ccr::BlockingMutex;
type Mutex = ccr::BlockingMutex<()>;

// SupportTreeBuildsteps.hpp:119-162 — class PillarIndex
pub struct PillarIndex {
    // SupportTreeBuildsteps.hpp:120 — PointIndex m_index;
    m_index: PointIndex,
    // SupportTreeBuildsteps.hpp:122 — mutable Mutex m_mutex;
    m_mutex: Mutex,
}

impl Default for PillarIndex {
    fn default() -> Self {
        Self {
            m_index: PointIndex::new(),
            m_mutex: Mutex::default(),
        }
    }
}

impl PillarIndex {
    // SupportTreeBuildsteps.hpp:126-130
    // template<class...Args> inline void guarded_insert(Args&&...args)
    // (Variadic forwarder monomorphized for the call form actually used:
    //  `guarded_insert(endpt, unsigned(pillar_id))`, cpp:578-579,992.)
    #[inline]
    pub fn guarded_insert(&mut self, v: Vec3d, idx: u32) {
        // SupportTreeBuildsteps.hpp:128 — std::lock_guard<Mutex> lck(m_mutex);
        let _lck = self.m_mutex.lock().unwrap();
        // SupportTreeBuildsteps.hpp:129 — m_index.insert(...);
        self.m_index.insert((v, idx));
    }

    // SupportTreeBuildsteps.hpp:132-137
    // template<class...Args>
    // inline std::vector<PointIndexEl> guarded_query(Args&&...args) const
    // (Monomorphized for the predicate form of PointIndex::query; this member
    //  template is not instantiated anywhere in the C++ build.)
    #[inline]
    pub fn guarded_query(&self, fn_: &dyn Fn(&PointIndexEl) -> bool) -> Vec<PointIndexEl> {
        // SupportTreeBuildsteps.hpp:135 — std::lock_guard<Mutex> lck(m_mutex);
        let _lck = self.m_mutex.lock().unwrap();
        // SupportTreeBuildsteps.hpp:136
        self.m_index.query(fn_)
    }

    // SupportTreeBuildsteps.hpp:139-142
    // template<class...Args> inline void insert(Args&&...args)
    // (Monomorphized for `insert(pp.endpoint(), unsigned(pp.id))`, cpp:1244.)
    #[inline]
    pub fn insert(&mut self, v: Vec3d, idx: u32) {
        // SupportTreeBuildsteps.hpp:141
        self.m_index.insert((v, idx));
    }

    // SupportTreeBuildsteps.hpp:144-148
    // template<class...Args>
    // inline std::vector<PointIndexEl> query(Args&&...args) const
    // (The C++ template forwards to either PointIndex::query overload; both
    //  instantiated call forms get a Rust method: the predicate form
    //  (cpp:1102-1104) keeps the name, the (point, k) nearest form (cpp:871)
    //  is `query_nearest`, matching the PointIndex wrapper hpp:44-47.)
    #[inline]
    pub fn query(&self, fn_: &dyn Fn(&PointIndexEl) -> bool) -> Vec<PointIndexEl> {
        // SupportTreeBuildsteps.hpp:147
        self.m_index.query(fn_)
    }

    #[inline]
    pub fn query_nearest(&self, v: &Vec3d, k: u32) -> Vec<PointIndexEl> {
        // SupportTreeBuildsteps.hpp:147
        self.m_index.query_nearest(v, k)
    }

    // SupportTreeBuildsteps.hpp:150
    // template<class Fn> inline void foreach(Fn fn) { m_index.foreach(fn); }
    #[inline]
    pub fn foreach(&self, fn_: &mut dyn FnMut(&PointIndexEl)) {
        self.m_index.foreach(fn_);
    }

    // SupportTreeBuildsteps.hpp:151-155
    // template<class Fn> inline void guarded_foreach(Fn fn)
    #[inline]
    pub fn guarded_foreach(&self, fn_: &mut dyn FnMut(&PointIndexEl)) {
        // SupportTreeBuildsteps.hpp:153 — std::lock_guard<Mutex> lck(m_mutex);
        let _lck = self.m_mutex.lock().unwrap();
        // SupportTreeBuildsteps.hpp:154
        self.m_index.foreach(fn_);
    }

    // SupportTreeBuildsteps.hpp:157-161
    // PointIndex guarded_clone()
    pub fn guarded_clone(&self) -> PointIndex {
        // SupportTreeBuildsteps.hpp:159 — std::lock_guard<Mutex> lck(m_mutex);
        let _lck = self.m_mutex.lock().unwrap();
        // SupportTreeBuildsteps.hpp:160 — return m_index;
        self.m_index.clone()
    }
}

// Helper function for pillar interconnection where pairs of already connected
// pillars should be checked for not to be processed again. This can be done
// in constant time with a set of hash values uniquely representing a pair of
// integers. The order of numbers within the pair should not matter, it has
// the same unique hash. The hash value has to have twice as many bits as the
// arguments need. If the same integral type is used for args and return val,
// make sure the arguments use only the half of the type's bit depth.
// SupportTreeBuildsteps.hpp:171-186
// template<class I, class DoubleI = IntegerOnly<I>>
// IntegerOnly<DoubleI> pairhash(I a, I b)
// (Monomorphized for the only instantiation in the C++ build: I = unsigned
//  (the PointIndexEl ids at cpp:1116-1119), for which the default
//  DoubleI = IntegerOnly<I> is the SAME 32-bit type, so shift = Ibits/2 = 16
//  and the arithmetic wraps like C++ unsigned.)
pub fn pairhash(a: u32, b: u32) -> u32 {
    // SupportTreeBuildsteps.hpp:175-177
    const IBITS: u32 = 32; // int(sizeof(I) * CHAR_BIT)
    const DOUBLEIBITS: u32 = 32; // int(sizeof(DoubleI) * CHAR_BIT)
    const SHIFT: u32 = if DOUBLEIBITS / 2 < IBITS { IBITS / 2 } else { IBITS };

    // SupportTreeBuildsteps.hpp:179
    let g = a.min(b);
    let l = a.max(b);

    // Assume the hash will fit into the output variable
    // SupportTreeBuildsteps.hpp:182-183
    debug_assert!((if g != 0 { (g as f64).log2().ceil() } else { 0.0 }) <= SHIFT as f64);
    debug_assert!((if l != 0 { (l as f64).log2().ceil() } else { 0.0 }) <= SHIFT as f64);

    // SupportTreeBuildsteps.hpp:185 — (DoubleI(g) << shift) + l
    g.wrapping_shl(SHIFT).wrapping_add(l)
}

// SupportTreeBuildsteps.hpp:193 — using PtIndices = std::vector<unsigned>;
type PtIndices = Vec<u32>;

// SupportTreeBuildsteps.hpp:188-373 — class SupportTreeBuildsteps
pub struct SupportTreeBuildsteps<'a> {
    // SupportTreeBuildsteps.hpp:189 — const SupportTreeConfig& m_cfg;
    m_cfg: &'a SupportTreeConfig,
    // SupportTreeBuildsteps.hpp:190 — const IndexedMesh& m_mesh;
    m_mesh: &'a IndexedMesh,
    // SupportTreeBuildsteps.hpp:191 — const std::vector<SupportPoint>& m_support_pts;
    m_support_pts: &'a SupportPoints,

    // SupportTreeBuildsteps.hpp:195 — PtIndices m_iheads; // support points with pinhead
    m_iheads: PtIndices,
    // SupportTreeBuildsteps.hpp:196
    m_iheads_onmodel: PtIndices,
    // SupportTreeBuildsteps.hpp:197 — PtIndices m_iheadless; // headless support points
    #[allow(dead_code)]
    m_iheadless: PtIndices,

    // SupportTreeBuildsteps.hpp:199 — std::map<unsigned, IndexedMesh::hit_result>
    m_head_to_ground_scans: BTreeMap<u32, hit_result<'a>>,

    // normals for support points from model faces.
    // SupportTreeBuildsteps.hpp:202 — PointSet m_support_nmls;
    #[allow(dead_code)]
    m_support_nmls: PointSet,

    // Clusters of points which can reach the ground directly and can be
    // bridged to one central pillar
    // SupportTreeBuildsteps.hpp:206 — std::vector<PtIndices> m_pillar_clusters;
    m_pillar_clusters: Vec<PtIndices>,

    // This algorithm uses the SupportTreeBuilder class to fill gradually
    // the support elements (heads, pillars, bridges, ...)
    // SupportTreeBuildsteps.hpp:210 — SupportTreeBuilder& m_builder;
    m_builder: &'a mut SupportTreeBuilder,

    // support points in Eigen/IGL format
    // SupportTreeBuildsteps.hpp:213 — PointSet m_points;
    m_points: PointSet,

    // throw if canceled: It will be called many times so a shorthand will
    // come in handy.
    // SupportTreeBuildsteps.hpp:217 — ThrowOnCancel m_thr;
    m_thr: ThrowOnCancel,

    // A spatial index to easily find strong pillars to connect to.
    // SupportTreeBuildsteps.hpp:220 — PillarIndex m_pillar_index;
    m_pillar_index: PillarIndex,

    // When bridging heads to pillars... TODO: find a cleaner solution
    // SupportTreeBuildsteps.hpp:223 — ccr::BlockingMutex m_bridge_mutex;
    m_bridge_mutex: ccr::BlockingMutex<()>,
}

// ---------------------------------------------------------------------------
// SupportTreeBuildsteps.cpp
// ---------------------------------------------------------------------------

// SupportTreeBuildsteps.cpp:10-15 — using declarations for Slic3r::opt
// (imported at the top of this module).

// SupportTreeBuildsteps.cpp:17-22
// StopCriteria get_criteria(const SupportTreeConfig &cfg)
pub fn get_criteria(_cfg: &SupportTreeConfig) -> StopCriteria {
    // SupportTreeBuildsteps.cpp:19-21
    // (optimizer_rel_score_diff / optimizer_max_iterations are static constexpr
    //  members of SupportTreeConfig, accessed through the instance in C++.)
    let mut c = StopCriteria::new();
    c.rel_score_diff(SupportTreeConfig::OPTIMIZER_REL_SCORE_DIFF);
    c.max_iterations(SupportTreeConfig::OPTIMIZER_MAX_ITERATIONS as f64);
    c
}

// SupportTreeBuildsteps.cpp:24-33
// template<class C, class Hit = IndexedMesh::hit_result>
// static Hit min_hit(const C &hits)
fn min_hit<'b>(hits: &[hit_result<'b>]) -> hit_result<'b> {
    // SupportTreeBuildsteps.cpp:27-30 — std::min_element with
    // `h1.distance() < h2.distance()` (returns the FIRST smallest element).
    let mut mit = 0usize;
    for i in 1..hits.len() {
        if hits[i].distance() < hits[mit].distance() {
            mit = i;
        }
    }
    // SupportTreeBuildsteps.cpp:32
    hits[mit]
}

impl<'a> SupportTreeBuildsteps<'a> {
    // SupportTreeBuildsteps.cpp:35-55
    // SupportTreeBuildsteps::SupportTreeBuildsteps(SupportTreeBuilder & builder,
    //                                              const SupportableMesh &sm)
    pub fn new(builder: &'a mut SupportTreeBuilder, sm: &'a SupportableMesh) -> Self {
        // SupportTreeBuildsteps.cpp:43 — m_thr(builder.ctl().cancelfn)
        // (std::function copy -> Arc clone)
        let m_thr: ThrowOnCancel = builder.ctl().cancelfn.clone();

        // SupportTreeBuildsteps.cpp:42 — m_points(sm.pts.size(), 3)
        // (Eigen leaves the matrix uninitialized; every row is written below.)
        let mut m_points = PointSet::zeros(sm.pts.len(), 3);

        // Prepare the support points in Eigen/IGL format as well, we will use
        // it mostly in this form.
        // SupportTreeBuildsteps.cpp:48-54
        let mut i = 0usize;
        for sp in sm.pts.iter() {
            // SupportTreeBuildsteps.cpp:50-52
            m_points[(i, X)] = sp.pos[0] as f64;
            m_points[(i, Y)] = sp.pos[1] as f64;
            m_points[(i, Z)] = sp.pos[2] as f64;
            // SupportTreeBuildsteps.cpp:53
            i += 1;
        }

        Self {
            // SupportTreeBuildsteps.cpp:37 — m_cfg(sm.cfg)
            m_cfg: &sm.cfg,
            // SupportTreeBuildsteps.cpp:38 — m_mesh(sm.emesh)
            m_mesh: &sm.emesh,
            // SupportTreeBuildsteps.cpp:39 — m_support_pts(sm.pts)
            m_support_pts: &sm.pts,
            m_iheads: PtIndices::new(),
            m_iheads_onmodel: PtIndices::new(),
            m_iheadless: PtIndices::new(),
            m_head_to_ground_scans: BTreeMap::new(),
            // SupportTreeBuildsteps.cpp:40 — m_support_nmls(sm.pts.size(), 3)
            // (sized but never written in this translation unit)
            m_support_nmls: PointSet::zeros(sm.pts.len(), 3),
            m_pillar_clusters: Vec::new(),
            // SupportTreeBuildsteps.cpp:41 — m_builder(builder)
            m_builder: builder,
            m_points,
            m_thr,
            m_pillar_index: PillarIndex::default(),
            m_bridge_mutex: ccr::BlockingMutex::new(()),
        }
    }

    // SupportTreeBuildsteps.cpp:57-176
    // bool SupportTreeBuildsteps::execute(SupportTreeBuilder & builder,
    //                                     const SupportableMesh &sm)
    pub fn execute(builder: &mut SupportTreeBuilder, sm: &SupportableMesh) -> bool {
        // SupportTreeBuildsteps.cpp:60
        if sm.pts.is_empty() {
            return false;
        }

        // SupportTreeBuildsteps.cpp:62
        builder.ground_level = sm.emesh.ground_level() - sm.cfg.object_elevation_mm;

        // SupportTreeBuildsteps.cpp:64
        let mut alg = SupportTreeBuildsteps::new(builder, sm);

        // Let's define the individual steps of the processing. We can experiment
        // later with the ordering and the dependencies between them.
        // SupportTreeBuildsteps.cpp:68-81
        #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
        #[allow(clippy::upper_case_acronyms, non_camel_case_types)]
        enum Steps {
            BEGIN,
            FILTER,
            PINHEADS,
            CLASSIFY,
            ROUTING_GROUND,
            ROUTING_NONGROUND,
            CASCADE_PILLARS,
            MERGE_RESULT,
            DONE,
            ABORT,
            // NUM_STEPS
            //...
        }
        const NUM_STEPS: usize = 10;

        // Collect the algorithm steps into a nice sequence
        // SupportTreeBuildsteps.cpp:83-111
        // (C++ builds an array of std::bind'ed member-function thunks; Rust
        //  cannot store closures that each mutably borrow `alg` in one array,
        //  so `program[pc]()` is dispatched through the equivalent `match`
        //  in the driver loop below — same steps, same order.)

        // SupportTreeBuildsteps.cpp:113
        let mut pc = Steps::BEGIN;

        // SupportTreeBuildsteps.cpp:115-120
        // if(sm.cfg.ground_facing_only) program[ROUTING_NONGROUND] = <log lambda>;
        // (folded into the ROUTING_NONGROUND arm of the dispatch match below)

        // Let's define a simple automaton that will run our program.
        // SupportTreeBuildsteps.cpp:123-167
        fn progress(builder: &SupportTreeBuilder, pc: &mut Steps) {
            // SupportTreeBuildsteps.cpp:124-135
            static STEPSTR: [&str; NUM_STEPS] = [
                "Starting",
                "Filtering",
                "Generate pinheads",
                "Classification",
                "Routing to ground",
                "Routing supports to model surface",
                "Interconnecting pillars",
                "Merging support mesh",
                "Done",
                "Abort",
            ];

            // SupportTreeBuildsteps.cpp:137-148
            static STEPSTATE: [u32; NUM_STEPS] = [0, 10, 30, 50, 60, 70, 80, 99, 100, 0];

            // SupportTreeBuildsteps.cpp:150
            if (builder.ctl().stopcondition)() {
                *pc = Steps::ABORT;
            }

            // SupportTreeBuildsteps.cpp:152-164
            match *pc {
                Steps::BEGIN => *pc = Steps::FILTER,
                Steps::FILTER => *pc = Steps::PINHEADS,
                Steps::PINHEADS => *pc = Steps::CLASSIFY,
                Steps::CLASSIFY => *pc = Steps::ROUTING_GROUND,
                Steps::ROUTING_GROUND => *pc = Steps::ROUTING_NONGROUND,
                Steps::ROUTING_NONGROUND => *pc = Steps::CASCADE_PILLARS,
                Steps::CASCADE_PILLARS => *pc = Steps::MERGE_RESULT,
                Steps::MERGE_RESULT => *pc = Steps::DONE,
                Steps::DONE | Steps::ABORT => {}
            }

            // SupportTreeBuildsteps.cpp:166
            (builder.ctl().statuscb)(STEPSTATE[*pc as usize], STEPSTR[*pc as usize]);
        }

        // Just here we run the computation...
        // SupportTreeBuildsteps.cpp:170-173
        while pc < Steps::DONE {
            progress(alg.m_builder, &mut pc);
            // program[pc]() — SupportTreeBuildsteps.cpp:84-111
            match pc {
                Steps::BEGIN => {
                    // Begin...
                    // Potentially clear up the shared data (not needed for now)
                }
                Steps::FILTER => alg.filter(),
                Steps::PINHEADS => alg.add_pinheads(),
                Steps::CLASSIFY => alg.classify(),
                Steps::ROUTING_GROUND => alg.routing_to_ground(),
                Steps::ROUTING_NONGROUND => {
                    if sm.cfg.ground_facing_only {
                        // SupportTreeBuildsteps.cpp:116-119
                        log::info!("Skipping model-facing supports as requested.");
                    } else {
                        alg.routing_to_model();
                    }
                }
                Steps::CASCADE_PILLARS => alg.interconnect_pillars(),
                Steps::MERGE_RESULT => alg.merge_result(),
                Steps::DONE => {
                    // Done
                }
                Steps::ABORT => {
                    // Abort
                }
            }
        }

        // SupportTreeBuildsteps.cpp:175
        pc == Steps::ABORT
    }

    // SupportTreeBuildsteps.hpp:225-229
    // inline IndexedMesh::hit_result ray_mesh_intersect(const Vec3d& s,
    //                                                   const Vec3d& dir)
    #[inline]
    #[allow(dead_code)]
    fn ray_mesh_intersect(&self, s: &Vec3d, dir: &Vec3d) -> hit_result<'a> {
        let m: &'a IndexedMesh = self.m_mesh;
        // SupportTreeBuildsteps.hpp:228
        m.query_ray_hit(&to_na(s), &to_na(dir))
    }

    // This function will test if a future pinhead would not collide with the
    // model geometry. It does not take a 'Head' object because those are
    // created after this test. Parameters: s: The touching point on the model
    // surface. dir: This is the direction of the head from the pin to the back
    // r_pin, r_back: the radiuses of the pin and the back sphere width: This
    // is the full width from the pin center to the back center m: The object
    // mesh.
    // The return value is the hit result from the ray casting. If the starting
    // point was inside the model, an "invalid" hit_result will be returned
    // with a zero distance value instead of a NAN. This way the result can
    // be used safely for comparison with other distances.
    // SupportTreeBuildsteps.hpp:242-248 (declaration) /
    // SupportTreeBuildsteps.cpp:178-257 (definition)
    fn pinhead_mesh_intersect(
        &self,
        s: &Vec3d,
        dir: &Vec3d,
        r_pin: f64,
        r_back: f64,
        width: f64,
        sd: f64,
    ) -> hit_result<'a> {
        // SupportTreeBuildsteps.cpp:186
        const SAMPLES: usize = 8;

        // Move away slightly from the touching point to avoid raycasting on the
        // inner surface of the mesh.

        // SupportTreeBuildsteps.cpp:191
        let m: &'a IndexedMesh = self.m_mesh;
        // SupportTreeBuildsteps.cpp:192 — using HitResult = IndexedMesh::hit_result;

        // SupportTreeBuildsteps.cpp:197-206
        struct Rings {
            rpin: f64,
            rback: f64,
            spin: Vec3d,
            sback: Vec3d,
            ring: PointRing<8>,
        }
        impl Rings {
            // SupportTreeBuildsteps.cpp:204
            fn backring(&self, idx: usize) -> Vec3d {
                self.ring.get(idx, &self.sback, self.rback)
            }
            // SupportTreeBuildsteps.cpp:205
            fn pinring(&self, idx: usize) -> Vec3d {
                self.ring.get(idx, &self.spin, self.rpin)
            }
        }
        // SupportTreeBuildsteps.cpp:206 — rings {r_pin + sd, r_back + sd, s, s + width * dir, dir};
        let rings = Rings {
            rpin: r_pin + sd,
            rback: r_back + sd,
            spin: *s,
            sback: *s + *dir * width,
            ring: PointRing::<8>::new(dir),
        };
        // `spin` participates in pinring via `self.spin`; silence the unused
        // field lint without changing the structure.
        let _ = &rings.spin;

        // Hit results
        // SupportTreeBuildsteps.cpp:195 — std::array<HitResult, SAMPLES> hits;
        // (index-tagged buffer so the disjoint per-sample writes can run through
        //  the same ccr facade — crate precedent: indexed_mesh.rs normals())
        let mut hits: Vec<(usize, hit_result<'a>)> =
            (0..SAMPLES).map(|i| (i, hit_result::default())).collect();

        // We will shoot multiple rays from the head pinpoint in the direction
        // of the pinhead robe (side) surface. The result will be the smallest
        // hit distance.

        // SupportTreeBuildsteps.cpp:212-254
        // ccr::for_each(size_t(0), hits.size(), [&m, &rings, sd, &hits](size_t i){...});
        ccr::for_each_mut(&mut hits, |&mut (i, ref mut hit)| {
            // Point on the circle on the pin sphere
            // SupportTreeBuildsteps.cpp:216
            let ps = rings.pinring(i);
            // This is the point on the circle on the back sphere
            // SupportTreeBuildsteps.cpp:218
            let p = rings.backring(i);

            // Point ps is not on mesh but can be inside or
            // outside as well. This would cause many problems
            // with ray-casting. To detect the position we will
            // use the ray-casting result (which has an is_inside
            // predicate).

            // SupportTreeBuildsteps.cpp:228
            let n = eigen_normalized(p - ps);
            // SupportTreeBuildsteps.cpp:229
            let q = m.query_ray_hit(&to_na(&(ps + n * sd)), &to_na(&n));

            // SupportTreeBuildsteps.cpp:231-253
            if q.is_inside() {
                // the hit is inside the model
                if q.distance() > rings.rpin {
                    // If we are inside the model and the hit
                    // distance is bigger than our pin circle
                    // diameter, it probably indicates that the
                    // support point was already inside the
                    // model, or there is really no space
                    // around the point. We will assign a zero
                    // hit distance to these cases which will
                    // enforce the function return value to be
                    // an invalid ray with zero hit distance.
                    // (see min_element at the end)
                    // SupportTreeBuildsteps.cpp:243
                    *hit = hit_result::new(0.0);
                } else {
                    // re-cast the ray from the outside of the
                    // object. The starting point has an offset
                    // of 2*safety_distance because the
                    // original ray has also had an offset
                    // SupportTreeBuildsteps.cpp:249-250
                    let q2 = m.query_ray_hit(&to_na(&(ps + n * (q.distance() + 2.0 * sd))), &to_na(&n));
                    *hit = q2;
                }
            } else {
                // SupportTreeBuildsteps.cpp:252-253
                *hit = q;
            }
        });

        // SupportTreeBuildsteps.cpp:256
        let plain: Vec<hit_result<'a>> = hits.into_iter().map(|(_, h)| h).collect();
        min_hit(&plain)
    }

    // SupportTreeBuildsteps.hpp:250-260
    // IndexedMesh::hit_result pinhead_mesh_intersect(
    //     const Vec3d& s, const Vec3d& dir, double r_pin, double r_back, double width)
    // (the overload supplying the default safety distance)
    fn pinhead_mesh_intersect_default_sd(
        &self,
        s: &Vec3d,
        dir: &Vec3d,
        r_pin: f64,
        r_back: f64,
        width: f64,
    ) -> hit_result<'a> {
        // SupportTreeBuildsteps.hpp:257-259
        self.pinhead_mesh_intersect(
            s,
            dir,
            r_pin,
            r_back,
            width,
            r_back * SupportTreeConfig::SAFETY_DISTANCE_MM / self.m_cfg.head_back_radius_mm,
        )
    }

    // Checking bridge (pillar and stick as well) intersection with the model.
    // If the function is used for headless sticks, the ins_check parameter
    // have to be true as the beginning of the stick might be inside the model
    // geometry.
    // The return value is the hit result from the ray casting. If the starting
    // point was inside the model, an "invalid" hit_result will be returned
    // with a zero distance value instead of a NAN. This way the result can
    // be used safely for comparison with other distances.
    // SupportTreeBuildsteps.hpp:270-274 (declaration) /
    // SupportTreeBuildsteps.cpp:259-290 (definition)
    fn bridge_mesh_intersect(&self, src: &Vec3d, dir: &Vec3d, r: f64, sd: f64) -> hit_result<'a> {
        // SupportTreeBuildsteps.cpp:262
        const SAMPLES: usize = 8;
        // SupportTreeBuildsteps.cpp:263
        let ring = PointRing::<8>::new(dir);

        // SupportTreeBuildsteps.cpp:265 — using Hit = IndexedMesh::hit_result;
        let m: &'a IndexedMesh = self.m_mesh;
        let src = *src;
        let dir = *dir;

        // Hit results
        // SupportTreeBuildsteps.cpp:268 — std::array<Hit, SAMPLES> hits;
        // (index-tagged buffer; see pinhead_mesh_intersect)
        let mut hits: Vec<(usize, hit_result<'a>)> =
            (0..SAMPLES).map(|i| (i, hit_result::default())).collect();

        // SupportTreeBuildsteps.cpp:270-287
        // ccr::for_each(size_t(0), hits.size(),
        //               [this, r, src, /*ins_check,*/ &ring, dir, sd, &hits](size_t i){...});
        ccr::for_each_mut(&mut hits, |&mut (i, ref mut hit)| {
            // Point on the circle on the pin sphere
            // SupportTreeBuildsteps.cpp:276
            let p = ring.get(i, &src, r + sd);

            // SupportTreeBuildsteps.cpp:278
            let hr = m.query_ray_hit(&to_na(&(p + dir * r)), &to_na(&dir));

            // SupportTreeBuildsteps.cpp:280-286
            if /*ins_check && */ hr.is_inside() {
                if hr.distance() > 2.0 * r + sd {
                    // SupportTreeBuildsteps.cpp:281
                    *hit = hit_result::new(0.0);
                } else {
                    // re-cast the ray from the outside of the object
                    // SupportTreeBuildsteps.cpp:284
                    *hit = m.query_ray_hit(
                        &to_na(&(p + dir * (hr.distance() + EPSILON))),
                        &to_na(&dir),
                    );
                }
            } else {
                *hit = hr;
            }
        });

        // SupportTreeBuildsteps.cpp:289
        let plain: Vec<hit_result<'a>> = hits.into_iter().map(|(_, h)| h).collect();
        min_hit(&plain)
    }

    // SupportTreeBuildsteps.hpp:276-284
    // IndexedMesh::hit_result bridge_mesh_intersect(
    //     const Vec3d& s, const Vec3d& dir, double r)
    // (the overload supplying the default safety distance)
    fn bridge_mesh_intersect_default_sd(&self, s: &Vec3d, dir: &Vec3d, r: f64) -> hit_result<'a> {
        // SupportTreeBuildsteps.hpp:281-283
        self.bridge_mesh_intersect(
            s,
            dir,
            r,
            r * SupportTreeConfig::SAFETY_DISTANCE_MM / self.m_cfg.head_back_radius_mm,
        )
    }

    // SupportTreeBuildsteps.hpp:286-289
    // template<class...Args>
    // inline double bridge_mesh_distance(Args&&...args) {
    //     return bridge_mesh_intersect(std::forward<Args>(args)...).distance();
    // }
    // (Variadic forwarder monomorphized for the only instantiated call form:
    //  the 3-argument default-safety-distance overload.)
    #[inline]
    fn bridge_mesh_distance(&self, s: &Vec3d, dir: &Vec3d, r: f64) -> f64 {
        self.bridge_mesh_intersect_default_sd(s, dir, r).distance()
    }

    // Helper function for interconnecting two pillars with zig-zag bridges.
    // SupportTreeBuildsteps.hpp:292 (declaration) /
    // SupportTreeBuildsteps.cpp:292-383 (definition)
    #[allow(unused_assignments)]
    fn interconnect(&mut self, pillar: &Pillar, nextpillar: &Pillar) -> bool {
        // We need to get the starting point of the zig-zag pattern. We have to
        // be aware that the two head junctions are at different heights. We
        // may start from the lowest junction and call it a day but this
        // strategy would leave unconnected a lot of pillar duos where the
        // shorter pillar is too short to start a new bridge but the taller
        // pillar could still be bridged with the shorter one.
        // SupportTreeBuildsteps.cpp:301
        let mut was_connected = false;

        // SupportTreeBuildsteps.cpp:303-306
        let mut supper = pillar.startpoint();
        let mut slower = nextpillar.startpoint();
        let mut eupper = *pillar.endpoint();
        let mut elower = *nextpillar.endpoint();

        // SupportTreeBuildsteps.cpp:308-310
        let zmin = self.m_builder.ground_level + self.m_cfg.base_height_mm;
        // std::max(eupper(Z), zmin)
        eupper.z = if eupper.z < zmin { zmin } else { eupper.z };
        elower.z = if elower.z < zmin { zmin } else { elower.z };

        // The usable length of both pillars should be positive
        // SupportTreeBuildsteps.cpp:313-314
        if slower.z - elower.z < 0.0 {
            return false;
        }
        if supper.z - eupper.z < 0.0 {
            return false;
        }

        // SupportTreeBuildsteps.cpp:316-319
        let pillar_dist = distance_between_vec2(
            &Vec2d::new(slower.x, slower.y),
            &Vec2d::new(supper.x, supper.y),
        );
        let bridge_distance = pillar_dist / (-self.m_cfg.bridge_slope).cos();
        let zstep = pillar_dist * (-self.m_cfg.bridge_slope).tan();

        // SupportTreeBuildsteps.cpp:321-322
        if pillar_dist < 2.0 * self.m_cfg.head_back_radius_mm
            || pillar_dist > self.m_cfg.max_pillar_link_distance_mm
        {
            return false;
        }

        // SupportTreeBuildsteps.cpp:324-325
        if supper.z < slower.z {
            std::mem::swap(&mut supper, &mut slower);
        }
        if eupper.z < elower.z {
            std::mem::swap(&mut eupper, &mut elower);
        }

        // SupportTreeBuildsteps.cpp:327
        let mut startz;
        let mut endz;

        // SupportTreeBuildsteps.cpp:329-330
        startz = if slower.z - zstep < supper.z {
            slower.z - zstep
        } else {
            slower.z
        };
        endz = if eupper.z + zstep > elower.z {
            eupper.z + zstep
        } else {
            eupper.z
        };

        // SupportTreeBuildsteps.cpp:332-343
        if slower.z - eupper.z < zstep.abs() {
            // no space for even one cross

            // Get max available space
            // SupportTreeBuildsteps.cpp:336 — std::min(supper(Z), slower(Z) - zstep)
            // (std::min(a,b) == (b<a)?b:a, written out to keep exact tie/NaN
            //  semantics)
            startz = if slower.z - zstep < supper.z {
                slower.z - zstep
            } else {
                supper.z
            };
            // SupportTreeBuildsteps.cpp:337 — std::max(eupper(Z) + zstep, elower(Z))
            endz = if eupper.z + zstep < elower.z {
                elower.z
            } else {
                eupper.z + zstep
            };

            // Align to center
            // SupportTreeBuildsteps.cpp:340-342
            let available_dist = startz - endz;
            let rounds = (available_dist / zstep.abs()).floor();
            startz -= 0.5 * (available_dist - rounds * zstep.abs());
        }

        // SupportTreeBuildsteps.cpp:345-349
        let pcm = self.m_cfg.pillar_connection_mode;
        let docrosses = pcm == PillarConnectionMode::Cross
            || (pcm == PillarConnectionMode::Dynamic
                && pillar_dist > 2.0 * self.m_cfg.base_radius_mm);

        // 'sj' means starting junction, 'ej' is the end junction of a bridge.
        // They will be swapped in every iteration thus the zig-zag pattern.
        // According to a config parameter, a second bridge may be added which
        // results in a cross connection between the pillars.
        // SupportTreeBuildsteps.cpp:355
        let mut sj = supper;
        let mut ej = slower;
        sj.z = startz;
        ej.z = sj.z + zstep;

        // TODO: This is a workaround to not have a faulty last bridge
        // SupportTreeBuildsteps.cpp:358-380
        while ej.z >= eupper.z
        /*endz*/
        {
            // SupportTreeBuildsteps.cpp:359-363
            if self.bridge_mesh_distance(&sj, &dirv(&sj, &ej), pillar.r) >= bridge_distance {
                self.m_builder.add_crossbridge(Bridge::new(sj, ej, pillar.r));
                was_connected = true;
            }

            // double bridging: (crosses)
            // SupportTreeBuildsteps.cpp:366-376
            if docrosses {
                // SupportTreeBuildsteps.cpp:367-368
                let sjback = Vec3d::new(ej.x, ej.y, sj.z);
                let ejback = Vec3d::new(sj.x, sj.y, ej.z);
                // SupportTreeBuildsteps.cpp:369-371
                if sjback.z <= slower.z
                    && ejback.z >= eupper.z
                    && self.bridge_mesh_distance(&sjback, &dirv(&sjback, &ejback), pillar.r)
                        >= bridge_distance
                {
                    // need to check collision for the cross stick
                    self.m_builder
                        .add_crossbridge(Bridge::new(sjback, ejback, pillar.r));
                    was_connected = true;
                }
            }

            // SupportTreeBuildsteps.cpp:378-379
            std::mem::swap(&mut sj, &mut ej);
            ej.z = sj.z + zstep;
        }

        // SupportTreeBuildsteps.cpp:382
        was_connected
    }

    // For connecting a head to a nearby pillar.
    // SupportTreeBuildsteps.hpp:295 (declaration) /
    // SupportTreeBuildsteps.cpp:385-466 (definition)
    // (`touchjp.z = zdown` at cpp:425 is dead in the C++ source too — kept)
    #[allow(unused_assignments)]
    fn connect_to_nearpillar(&mut self, head: &Head, nearpillar_id: i64) -> bool {
        // SupportTreeBuildsteps.cpp:388-390
        // auto nearpillar = [this, nearpillar_id]() -> const Pillar& { ... };
        // (Rust: a fresh copy is fetched at every C++ `nearpillar()` call site
        //  so that mutations between calls are observed identically.)

        // SupportTreeBuildsteps.cpp:392-393
        let np = self.m_builder.pillar(nearpillar_id).clone();
        if self.m_builder.bridgecount(&np) > self.m_cfg.max_bridges_on_pillar {
            return false;
        }

        // SupportTreeBuildsteps.cpp:395-397
        let headjp = head.junction_point();
        let nearjp_u = self.m_builder.pillar(nearpillar_id).startpoint();
        let nearjp_l = *self.m_builder.pillar(nearpillar_id).endpoint();

        // SupportTreeBuildsteps.cpp:399-401
        let r = head.r_back_mm;
        let d2d = distance_between_vec2(&to_2d(&headjp), &to_2d(&nearjp_u));
        let d3d = distance_between(&headjp, &nearjp_u);

        // SupportTreeBuildsteps.cpp:403-404
        let hdiff = nearjp_u.z - headjp.z;
        let slope = hdiff.atan2(d2d);

        // SupportTreeBuildsteps.cpp:406-410
        let mut bridgestart = headjp;
        let mut bridgeend = nearjp_u;
        let max_len = r * self.m_cfg.max_bridge_length_mm / self.m_cfg.head_back_radius_mm;
        let max_slope = self.m_cfg.bridge_slope;
        let mut zdiff = 0.0;

        // check the default situation if feasible for a bridge
        // SupportTreeBuildsteps.cpp:413-438
        if d3d > max_len || slope > -max_slope {
            // not feasible to connect the two head junctions. We have to search
            // for a suitable touch point.

            // SupportTreeBuildsteps.cpp:417-420
            let mut zdown = headjp.z + d2d * (-max_slope).tan();
            let mut touchjp = bridgeend;
            touchjp.z = zdown;
            let d_cap = distance_between(&headjp, &touchjp); // cpp:419 `double D`
            zdiff = zdown - nearjp_u.z;

            // SupportTreeBuildsteps.cpp:422-432
            if zdiff > 0.0 {
                // SupportTreeBuildsteps.cpp:423-425
                zdown -= zdiff;
                bridgestart.z -= zdiff;
                touchjp.z = zdown;

                // SupportTreeBuildsteps.cpp:427
                let t = self.bridge_mesh_distance(&headjp, &DOWN, r);

                // We can't insert a pillar under the source head to connect
                // with the nearby pillar's starting junction
                // SupportTreeBuildsteps.cpp:431
                if t < zdiff {
                    return false;
                }
            }

            // SupportTreeBuildsteps.cpp:434-437
            if zdown <= nearjp_u.z && zdown >= nearjp_l.z && d_cap < max_len {
                bridgeend.z = zdown;
            } else {
                return false;
            }
        }

        // There will be a minimum distance from the ground where the
        // bridge is allowed to connect. This is an empiric value.
        // SupportTreeBuildsteps.cpp:442-443
        let minz = self.m_builder.ground_level + 4.0 * head.r_back_mm;
        if bridgeend.z < minz {
            return false;
        }

        // SupportTreeBuildsteps.cpp:445
        let t = self.bridge_mesh_distance(&bridgestart, &dirv(&bridgestart, &bridgeend), r);

        // Cannot insert the bridge. (further search might not worth the hassle)
        // SupportTreeBuildsteps.cpp:448
        if t < distance_between(&bridgestart, &bridgeend) {
            return false;
        }

        // SupportTreeBuildsteps.cpp:450
        let _lk = self.m_bridge_mutex.lock().unwrap();

        // SupportTreeBuildsteps.cpp:452-463
        let np2 = self.m_builder.pillar(nearpillar_id).clone();
        if self.m_builder.bridgecount(&np2) < self.m_cfg.max_bridges_on_pillar {
            // A partial pillar is needed under the starting head.
            // SupportTreeBuildsteps.cpp:454-460
            if zdiff > 0.0 {
                // SupportTreeBuildsteps.cpp:455-457
                self.m_builder
                    .add_pillar_from_head(head.id, headjp.z - bridgestart.z);
                self.m_builder.add_junction(Junction::new(bridgestart, r));
                self.m_builder.add_bridge(bridgestart, bridgeend, r);
            } else {
                // SupportTreeBuildsteps.cpp:459
                self.m_builder.add_bridge_from_head(head.id, bridgeend);
            }

            // SupportTreeBuildsteps.cpp:462
            self.m_builder.increment_bridges(&np2);
        } else {
            return false;
        }

        // SupportTreeBuildsteps.cpp:465
        true
    }

    // This is a proxy function for pillar creation which will mind the gap
    // between the pad and the model bottom in zero elevation mode.
    // jp is the starting junction point which needs to be routed down.
    // sourcedir is the allowed direction of an optional bridge between the
    // jp junction and the final pillar.
    // SupportTreeBuildsteps.hpp:313-316 (declaration, head_id = ID_UNSET default) /
    // SupportTreeBuildsteps.cpp:468-582 (definition)
    fn create_ground_pillar(
        &mut self,
        hjp: &Vec3d,
        sourcedir: &Vec3d,
        radius: f64,
        head_id: i64,
    ) -> bool {
        let mut radius = radius;

        // SupportTreeBuildsteps.cpp:473-475
        let jp = *hjp;
        let mut endp = jp;
        let mut dir = *sourcedir;
        let pillar_id: i64;
        let mut can_add_base = false;
        let mut non_head = false;

        // SupportTreeBuildsteps.cpp:477-479
        let mut gndlvl = 0.0; // The Z level where pedestals should be
        let mut jp_gnd = 0.0; // The lowest Z where a junction center can be
        let mut gap_dist = 0.0; // The gap distance between the model and the pad

        // SupportTreeBuildsteps.cpp:481
        // auto to_floor = [&gndlvl](const Vec3d &p) { return Vec3d{p.x(), p.y(), gndlvl}; };
        // (gndlvl mutates between calls; passed explicitly)
        let to_floor = |p: &Vec3d, gndlvl: f64| Vec3d::new(p.x, p.y, gndlvl);

        // SupportTreeBuildsteps.cpp:483-492
        // auto eval_limits = [this, &radius, &can_add_base, &gndlvl, &gap_dist, &jp_gnd]
        //     (bool base_en = true) {...};
        // (captured state passed explicitly)
        fn eval_limits(
            cfg: &SupportTreeConfig,
            builder_ground_level: f64,
            mesh_ground_level_offset: f64,
            radius: f64,
            base_en: bool,
            can_add_base: &mut bool,
            gndlvl: &mut f64,
            jp_gnd: &mut f64,
            gap_dist: &mut f64,
        ) {
            // SupportTreeBuildsteps.cpp:486
            *can_add_base = base_en && radius >= cfg.head_back_radius_mm;
            // SupportTreeBuildsteps.cpp:487
            let base_r = if *can_add_base { cfg.base_radius_mm } else { 0.0 };
            // SupportTreeBuildsteps.cpp:488
            *gndlvl = builder_ground_level;
            // SupportTreeBuildsteps.cpp:489
            if !*can_add_base {
                *gndlvl -= mesh_ground_level_offset;
            }
            // SupportTreeBuildsteps.cpp:490
            *jp_gnd = *gndlvl + if *can_add_base { 0.0 } else { cfg.head_back_radius_mm };
            // SupportTreeBuildsteps.cpp:491
            *gap_dist = cfg.pillar_base_safety_distance_mm + base_r + EPSILON;
        }

        // SupportTreeBuildsteps.cpp:494
        eval_limits(
            self.m_cfg,
            self.m_builder.ground_level,
            self.m_mesh.ground_level_offset(),
            radius,
            true,
            &mut can_add_base,
            &mut gndlvl,
            &mut jp_gnd,
            &mut gap_dist,
        );

        // We are dealing with a mini pillar that's potentially too long
        // SupportTreeBuildsteps.cpp:497-512
        if radius < self.m_cfg.head_back_radius_mm && jp.z - gndlvl > 20.0 * radius {
            // SupportTreeBuildsteps.cpp:499-500
            let diffbr =
                self.search_widening_path(&jp, &dir, radius, self.m_cfg.head_back_radius_mm);

            // SupportTreeBuildsteps.cpp:502-511
            match diffbr {
                Some(diffbr) if diffbr.endp.z > jp_gnd => {
                    // SupportTreeBuildsteps.cpp:503-504
                    let br_id = self.m_builder.add_diffbridge(diffbr.clone()).bridge.id;
                    if head_id >= 0 {
                        self.m_builder.head(head_id as u32).bridge_id = br_id;
                    }
                    // SupportTreeBuildsteps.cpp:505-506
                    endp = diffbr.endp;
                    radius = diffbr.end_r;
                    // SupportTreeBuildsteps.cpp:507
                    self.m_builder.add_junction(Junction::new(endp, radius));
                    // SupportTreeBuildsteps.cpp:508
                    non_head = true;
                    // SupportTreeBuildsteps.cpp:509
                    dir = diffbr.get_dir();
                    // SupportTreeBuildsteps.cpp:510
                    eval_limits(
                        self.m_cfg,
                        self.m_builder.ground_level,
                        self.m_mesh.ground_level_offset(),
                        radius,
                        true,
                        &mut can_add_base,
                        &mut gndlvl,
                        &mut jp_gnd,
                        &mut gap_dist,
                    );
                }
                _ => return false, // SupportTreeBuildsteps.cpp:511
            }
        }

        // SupportTreeBuildsteps.cpp:514-566
        if self.m_cfg.object_elevation_mm < EPSILON {
            // get a suitable direction for the corrector bridge. It is the
            // original sourcedir's azimuth but the polar angle is saturated to the
            // configured bridge slope.
            // SupportTreeBuildsteps.cpp:519-521
            let (_polar0, azimuth) = dir_to_spheric(&dir, 1.0);
            let polar = PI - self.m_cfg.bridge_slope;
            let d = eigen_normalized(spheric_to_dir(polar, azimuth));
            // SupportTreeBuildsteps.cpp:522-524
            let mut t = self.bridge_mesh_distance(&endp, &d, radius);
            // std::min(m_cfg.max_bridge_length_mm, t)
            let mut tmax = if t < self.m_cfg.max_bridge_length_mm {
                t
            } else {
                self.m_cfg.max_bridge_length_mm
            };
            t = 0.0;

            // SupportTreeBuildsteps.cpp:526-528
            let mut zd = endp.z - jp_gnd;
            let mut tmax2 =
                zd / (1.0 - self.m_cfg.bridge_slope * self.m_cfg.bridge_slope).sqrt();
            tmax = if tmax2 < tmax { tmax2 } else { tmax };

            // SupportTreeBuildsteps.cpp:530-536
            let mut nexp = endp;
            let mut dlast;
            loop {
                // (dlast = std::sqrt(m_mesh.squared_distance(to_floor(nexp))))
                dlast = self
                    .m_mesh
                    .squared_distance_simple(&to_na(&to_floor(&nexp, gndlvl)))
                    .sqrt();
                if !((dlast < gap_dist
                    || !self.bridge_mesh_distance(&nexp, &DOWN, radius).is_infinite())
                    && t < tmax)
                {
                    break;
                }
                // SupportTreeBuildsteps.cpp:534-535
                t += radius;
                nexp = endp + d * t;
            }

            // SupportTreeBuildsteps.cpp:538-553
            if dlast < gap_dist && can_add_base {
                // SupportTreeBuildsteps.cpp:539-542
                nexp = endp;
                t = 0.0;
                can_add_base = false;
                let base_en = can_add_base;
                eval_limits(
                    self.m_cfg,
                    self.m_builder.ground_level,
                    self.m_mesh.ground_level_offset(),
                    radius,
                    base_en,
                    &mut can_add_base,
                    &mut gndlvl,
                    &mut jp_gnd,
                    &mut gap_dist,
                );

                // SupportTreeBuildsteps.cpp:544-546
                zd = endp.z - jp_gnd;
                tmax2 = zd / (1.0 - self.m_cfg.bridge_slope * self.m_cfg.bridge_slope).sqrt();
                tmax = if tmax2 < tmax { tmax2 } else { tmax };

                // SupportTreeBuildsteps.cpp:548-552
                loop {
                    dlast = self
                        .m_mesh
                        .squared_distance_simple(&to_na(&to_floor(&nexp, gndlvl)))
                        .sqrt();
                    if !((dlast < gap_dist
                        || !self.bridge_mesh_distance(&nexp, &DOWN, radius).is_infinite())
                        && t < tmax)
                    {
                        break;
                    }
                    t += radius;
                    nexp = endp + d * t;
                }
            }

            // Could not find a path to avoid the pad gap
            // SupportTreeBuildsteps.cpp:556
            if dlast < gap_dist {
                return false;
            }

            // SupportTreeBuildsteps.cpp:558-565
            if t > 0.0 {
                // Need to make additional bridge
                // SupportTreeBuildsteps.cpp:559-560
                let br_id = self.m_builder.add_bridge(endp, nexp, radius).id;
                if head_id >= 0 {
                    self.m_builder.head(head_id as u32).bridge_id = br_id;
                }

                // SupportTreeBuildsteps.cpp:562-564
                self.m_builder.add_junction(Junction::new(nexp, radius));
                endp = nexp;
                non_head = true;
            }
        }

        // SupportTreeBuildsteps.cpp:568-569
        let gp = to_floor(&endp, gndlvl);
        let h = endp.z - gp.z;

        // SupportTreeBuildsteps.cpp:571-572
        pillar_id = if head_id >= 0 && !non_head {
            self.m_builder.add_pillar_from_head(head_id, h)
        } else {
            self.m_builder.add_pillar(Pillar::new(gp, h, radius))
        };

        // SupportTreeBuildsteps.cpp:574-575
        if can_add_base {
            self.add_pillar_base(pillar_id);
        }

        // SupportTreeBuildsteps.cpp:577-579
        if pillar_id >= 0 {
            // Save the pillar endpoint in the spatial index
            let endpt = self.m_builder.pillar(pillar_id).endpt;
            self.m_pillar_index.guarded_insert(endpt, pillar_id as u32);
        }

        // SupportTreeBuildsteps.cpp:581
        true
    }

    // SupportTreeBuildsteps.hpp:318-321
    // void add_pillar_base(long pid)
    fn add_pillar_base(&mut self, pid: i64) {
        // SupportTreeBuildsteps.hpp:320
        self.m_builder
            .add_pillar_base(pid, self.m_cfg.base_height_mm, self.m_cfg.base_radius_mm);
    }

    // SupportTreeBuildsteps.hpp:323-326 (declaration) /
    // SupportTreeBuildsteps.cpp:584-627 (definition)
    // std::optional<DiffBridge> SupportTreeBuildsteps::search_widening_path(
    //     const Vec3d &jp, const Vec3d &dir, double radius, double new_radius)
    fn search_widening_path(
        &self,
        jp: &Vec3d,
        dir: &Vec3d,
        radius: f64,
        new_radius: f64,
    ) -> Option<DiffBridge> {
        // SupportTreeBuildsteps.cpp:587-589
        let w = radius + 2.0 * self.m_cfg.head_back_radius_mm;
        let stopval = w + jp.z - self.m_builder.ground_level;
        let mut criteria = get_criteria(self.m_cfg);
        criteria.stop_score(stopval);
        // Optimizer<AlgNLoptSubplex> solver(get_criteria(m_cfg).stop_score(stopval));
        let mut solver = NLoptAlgOptimizer::new(alg_nlopt_subplex(), criteria);

        // SupportTreeBuildsteps.cpp:591
        let (polar, azimuth) = dir_to_spheric(dir, 1.0);

        // SupportTreeBuildsteps.cpp:593
        let fallback_ratio = radius / self.m_cfg.head_back_radius_mm;

        // SupportTreeBuildsteps.cpp:595-615
        let jp_v = *jp;
        let iv = initvals([polar, azimuth, w]); // start with what we have
        let bs = bounds([
            // Must not exceed the slope limit
            Bound::new(PI - self.m_cfg.bridge_slope, PI),
            // azimuth can be a full search
            Bound::new(-PI, PI),
            Bound::new(
                radius + self.m_cfg.head_back_radius_mm,
                fallback_ratio * self.m_cfg.max_bridge_length_mm,
            ),
        ]);
        let ores = solver.to_max().optimize(
            |input: &Input<3>| -> f64 {
                // SupportTreeBuildsteps.cpp:597
                let [plr, azm, t] = *input;

                // SupportTreeBuildsteps.cpp:599-602
                let d = eigen_normalized(spheric_to_dir(plr, azm));
                let mut ret = self
                    .pinhead_mesh_intersect_default_sd(&jp_v, &d, radius, new_radius, t)
                    .distance();
                let down = self.bridge_mesh_distance(&(jp_v + d * t), &d, new_radius);

                // SupportTreeBuildsteps.cpp:604-605
                if ret > t && down.is_infinite() {
                    ret += jp_v.z - self.m_builder.ground_level;
                }

                // SupportTreeBuildsteps.cpp:607
                ret
            },
            &iv,
            &bs,
        );
        // BLOCKED BACKEND DIVERGENCE — see module notes.
        let oresult = opt_result_or_backend_unavailable(ores, &iv);

        // SupportTreeBuildsteps.cpp:617-624
        if oresult.score >= stopval {
            // SupportTreeBuildsteps.cpp:618-621
            let polar = oresult.optimum[0];
            let azimuth = oresult.optimum[1];
            let t = oresult.optimum[2];
            let endp = jp_v + spheric_to_dir(polar, azimuth) * t;

            // SupportTreeBuildsteps.cpp:623
            return Some(DiffBridge::new(
                jp_v,
                endp,
                radius,
                self.m_cfg.head_back_radius_mm,
            ));
        }

        // SupportTreeBuildsteps.cpp:626
        None
    }

    // Filtering step: here we will discard inappropriate support points
    // and decide the future of the appropriate ones. We will check if a
    // pinhead is applicable and adjust its angle at each support point. We
    // will also merge the support points that are just too close and can
    // be considered as one.
    // SupportTreeBuildsteps.hpp:338 (declaration) /
    // SupportTreeBuildsteps.cpp:629-763 (definition)
    pub fn filter(&mut self) {
        // Get the points that are too close to each other and keep only the
        // first one
        // SupportTreeBuildsteps.cpp:633 — auto aliases = cluster(m_points, D_SP, 2);
        // (PointSet rows -> &[Vec3d] representation bridge for the cluster API)
        let pts_rows: Vec<Vec3d> = (0..self.m_points.nrows())
            .map(|r| {
                let row = self.m_points.row(r);
                Vec3d::new(row[X], row[Y], row[Z])
            })
            .collect();
        let aliases = cluster_points(&pts_rows, D_SP, 2);

        // SupportTreeBuildsteps.cpp:635-642
        let mut filtered_indices: PtIndices = Vec::with_capacity(aliases.len());
        self.m_iheads.reserve(aliases.len());
        self.m_iheadless.reserve(aliases.len());
        for a in &aliases {
            // Here we keep only the front point of the cluster.
            filtered_indices.push(a[0]);
        }

        // calculate the normals to the triangles for filtered points
        // SupportTreeBuildsteps.cpp:645-646
        let nmls = normals(
            &self.m_points,
            self.m_mesh,
            self.m_cfg.head_front_radius_mm,
            self.m_thr.as_ref(),
            &filtered_indices,
        );

        // Not all of the support points have to be a valid position for
        // support creation. The angle may be inappropriate or there may
        // not be enough space for the pinhead. Filtering is applied for
        // these reasons.

        // SupportTreeBuildsteps.cpp:653-664
        let mut heads: Vec<Head> = Vec::with_capacity(self.m_support_pts.len());
        for sp in self.m_support_pts.iter() {
            // SupportTreeBuildsteps.cpp:655
            (self.m_thr)();
            // SupportTreeBuildsteps.cpp:656-663
            heads.push(Head::new(
                f64::NAN,
                sp.head_front_radius as f64,
                0.0,
                self.m_cfg.head_penetration_mm,
                Vec3d::new(0.0, 0.0, 0.0),                                       // dir
                Vec3d::new(sp.pos[0] as f64, sp.pos[1] as f64, sp.pos[2] as f64), // displacement
            ));
        }

        // SupportTreeBuildsteps.cpp:666-749 — filterfn (ported as the
        // `filterfn` method below; the C++ recursive std::function lambda
        // becomes a recursive method).

        // SupportTreeBuildsteps.cpp:751-754
        // ccr::for_each(size_t(0), filtered_indices.size(), ...);
        // (sequential execution of the parallel loop — disjoint writes into
        //  `heads`; see module notes)
        for i in 0..filtered_indices.len() {
            self.filterfn(
                filtered_indices[i],
                i,
                self.m_cfg.head_back_radius_mm,
                &nmls,
                &mut heads,
            );
        }

        // SupportTreeBuildsteps.cpp:756-760
        for i in 0..heads.len() {
            if heads[i].is_valid() {
                self.m_builder.add_head(i as u32, heads[i].clone());
                self.m_iheads.push(i as u32);
            }
        }

        // SupportTreeBuildsteps.cpp:762
        (self.m_thr)();
    }

    // SupportTreeBuildsteps.cpp:666-749
    // std::function<void(unsigned, size_t, double)> filterfn;
    // filterfn = [this, &nmls, &heads, &filterfn](unsigned fidx, size_t i, double back_r) {...}
    fn filterfn(&self, fidx: u32, i: usize, back_r: f64, nmls: &PointSet, heads: &mut Vec<Head>) {
        // SupportTreeBuildsteps.cpp:668
        (self.m_thr)();

        // SupportTreeBuildsteps.cpp:670
        let nrow = nmls.row(i);
        let n = Vec3d::new(nrow[X], nrow[Y], nrow[Z]);

        // for all normals we generate the spherical coordinates and
        // saturate the polar angle to 45 degrees from the bottom then
        // convert back to standard coordinates to get the new normal.
        // Then we just create a quaternion from the two normals
        // (Quaternion::FromTwoVectors) and apply the rotation to the
        // arrow head.

        // SupportTreeBuildsteps.cpp:679
        let (mut polar, mut azimuth) = dir_to_spheric(&n, 1.0);

        // skip if the tilt is not sane
        // SupportTreeBuildsteps.cpp:682
        if polar < PI - SupportTreeConfig::NORMAL_CUTOFF_ANGLE {
            return;
        }

        // We saturate the polar angle to 3pi/4
        // SupportTreeBuildsteps.cpp:685 — polar = std::max(polar, PI - m_cfg.bridge_slope);
        polar = if polar < PI - self.m_cfg.bridge_slope {
            PI - self.m_cfg.bridge_slope
        } else {
            polar
        };

        // save the head (pinpoint) position
        // SupportTreeBuildsteps.cpp:688
        let hprow = self.m_points.row(fidx as usize);
        let hp = Vec3d::new(hprow[X], hprow[Y], hprow[Z]);

        // SupportTreeBuildsteps.cpp:690
        let mut lmin = self.m_cfg.head_width_mm;
        let mut lmax = lmin;

        // SupportTreeBuildsteps.cpp:692-694
        if back_r < self.m_cfg.head_back_radius_mm {
            lmin = 0.0;
            lmax = self.m_cfg.head_penetration_mm;
        }

        // The distance needed for a pinhead to not collide with model.
        // SupportTreeBuildsteps.cpp:697-698
        let w = lmin + 2.0 * back_r + 2.0 * self.m_cfg.head_front_radius_mm
            - self.m_cfg.head_penetration_mm;

        // SupportTreeBuildsteps.cpp:700
        let pin_r = self.m_support_pts[fidx as usize].head_front_radius as f64;

        // Reassemble the now corrected normal
        // SupportTreeBuildsteps.cpp:703
        let mut nn = eigen_normalized(spheric_to_dir(polar, azimuth));

        // check available distance
        // SupportTreeBuildsteps.cpp:706-707
        let mut t = self.pinhead_mesh_intersect_default_sd(&hp, &nn, pin_r, back_r, w);

        // SupportTreeBuildsteps.cpp:709-741
        if t.distance() < w {
            // Let's try to optimize this angle, there might be a
            // viable normal that doesn't collide with the model
            // geometry and its very close to the default.

            // SupportTreeBuildsteps.cpp:714-715
            let mut solver = NLoptAlgCombOptimizer::new(alg_nlopt_genetic(), get_criteria(self.m_cfg));
            solver.seed(0); // we want deterministic behavior

            // SupportTreeBuildsteps.cpp:717-732
            let iv = initvals([polar, azimuth, (lmin + lmax) / 2.0]); // start with what we have
            let bs = bounds([
                // Must not exceed the slope limit
                Bound::new(PI - self.m_cfg.bridge_slope, PI),
                // azimuth can be a full search
                Bound::new(-PI, PI),
                Bound::new(lmin, lmax),
            ]);
            let ores = solver.to_max().optimize(
                |input: &Input<3>| -> f64 {
                    // SupportTreeBuildsteps.cpp:720
                    let [plr, azm, l] = *input;

                    // SupportTreeBuildsteps.cpp:722
                    let dir = eigen_normalized(spheric_to_dir(plr, azm));

                    // SupportTreeBuildsteps.cpp:724-725
                    self.pinhead_mesh_intersect_default_sd(&hp, &dir, pin_r, back_r, l)
                        .distance()
                },
                &iv,
                &bs,
            );
            // BLOCKED BACKEND DIVERGENCE — see module notes.
            let oresult = opt_result_or_backend_unavailable(ores, &iv);

            // SupportTreeBuildsteps.cpp:734-740
            if oresult.score > w {
                polar = oresult.optimum[0];
                azimuth = oresult.optimum[1];
                nn = eigen_normalized(spheric_to_dir(polar, azimuth));
                lmin = oresult.optimum[2];
                t = hit_result::new(oresult.score);
            }
        }

        // SupportTreeBuildsteps.cpp:743-748
        if t.distance() > w && hp.z + w * nn.z >= self.m_builder.ground_level {
            // SupportTreeBuildsteps.cpp:744-745
            let h = &mut heads[fidx as usize];
            h.id = fidx as i64;
            h.dir = nn;
            h.width_mm = lmin;
            h.r_back_mm = back_r;
        } else if back_r > self.m_cfg.head_fallback_radius_mm {
            // SupportTreeBuildsteps.cpp:747
            self.filterfn(fidx, i, self.m_cfg.head_fallback_radius_mm, nmls, heads);
        }
    }

    // Pinhead creation: based on the filtering results, the Head objects
    // will be constructed (together with their triangle meshes).
    // SupportTreeBuildsteps.hpp:342 (declaration) /
    // SupportTreeBuildsteps.cpp:765-767 (definition)
    pub fn add_pinheads(&mut self) {}

    // Further classification of the support points with pinheads. If the
    // ground is directly reachable through a vertical line parallel to the
    // Z axis we consider a support point as pillar candidate. If touches
    // the model geometry, it will be marked as non-ground facing and
    // further steps will process it. Also, the pillars will be grouped
    // into clusters that can be interconnected with bridges. Elements of
    // these groups may or may not be interconnected. Here we only run the
    // clustering algorithm.
    // SupportTreeBuildsteps.hpp:352 (declaration) /
    // SupportTreeBuildsteps.cpp:769-816 (definition)
    pub fn classify(&mut self) {
        // We should first get the heads that reach the ground directly
        // SupportTreeBuildsteps.cpp:772-774
        let mut ground_head_indices: PtIndices = Vec::with_capacity(self.m_iheads.len());
        self.m_iheads_onmodel.reserve(self.m_iheads.len());

        // First we decide which heads reach the ground and can be full
        // pillars and which shall be connected to the model surface (or
        // search a suitable path around the surface that leads to the
        // ground -- TODO)
        // SupportTreeBuildsteps.cpp:780-795
        for k in 0..self.m_iheads.len() {
            let i = self.m_iheads[k];
            // SupportTreeBuildsteps.cpp:781
            (self.m_thr)();

            // SupportTreeBuildsteps.cpp:783-785
            let (r, headjp) = {
                let head = self.m_builder.head(i);
                (head.r_back_mm, head.junction_point())
            };

            // collision check
            // SupportTreeBuildsteps.cpp:788
            let hit = self.bridge_mesh_intersect_default_sd(&headjp, &DOWN, r);

            // SupportTreeBuildsteps.cpp:790-792
            if hit.distance().is_infinite() {
                ground_head_indices.push(i);
            } else if self.m_cfg.ground_facing_only {
                self.m_builder.head(i).invalidate();
            } else {
                self.m_iheads_onmodel.push(i);
            }

            // SupportTreeBuildsteps.cpp:794
            self.m_head_to_ground_scans.insert(i, hit);
        }

        // We want to search for clusters of points that are far enough
        // from each other in the XY plane to not cross their pillar bases
        // These clusters of support points will join in one pillar,
        // possibly in their centroid support point.

        // SupportTreeBuildsteps.cpp:802-804
        // auto pointfn = [this](unsigned i) { return m_builder.head(i).junction_point(); };
        // (junction points pre-fetched: `head()` requires &mut access to the
        //  builder which a shared Fn capture cannot hold; values identical)
        let mut jps: HashMap<u32, Vec3d> = HashMap::new();
        for k in 0..ground_head_indices.len() {
            let i = ground_head_indices[k];
            let jp = self.m_builder.head(i).junction_point();
            jps.insert(i, jp);
        }
        let pointfn = |i: u32| -> Vec3d { jps[&i] };

        // SupportTreeBuildsteps.cpp:806-812
        let base_radius_mm = self.m_cfg.base_radius_mm;
        let max_bridge_length_mm = self.m_cfg.max_bridge_length_mm;
        let predicate = |e1: &PointIndexEl, e2: &PointIndexEl| -> bool {
            // SupportTreeBuildsteps.cpp:808-809
            let d2d = distance_between_vec2(&to_2d(&e1.0), &to_2d(&e2.0));
            let d3d = distance_between(&e1.0, &e2.0);
            // SupportTreeBuildsteps.cpp:810-811
            d2d < 2.0 * base_radius_mm && d3d < max_bridge_length_mm
        };

        // SupportTreeBuildsteps.cpp:814-815
        self.m_pillar_clusters = cluster_by_predicate(
            &ground_head_indices,
            &pointfn,
            &predicate,
            self.m_cfg.max_bridges_on_pillar,
        );
    }

    // Step: Routing the ground connected pinheads, and interconnecting
    // them with additional (angled) bridges. Not all of these pinheads
    // will be a full pillar (ground connected). Some will connect to a
    // nearby pillar using a bridge. The max number of such side-heads for
    // a central pillar is limited to avoid bad weight distribution.
    // SupportTreeBuildsteps.hpp:359 (declaration) /
    // SupportTreeBuildsteps.cpp:818-890 (definition)
    pub fn routing_to_ground(&mut self) {
        // SupportTreeBuildsteps.cpp:820-821
        let mut cl_centroids: ClusterEl = Vec::with_capacity(self.m_pillar_clusters.len());

        // SupportTreeBuildsteps.cpp:823-860
        for k in 0..self.m_pillar_clusters.len() {
            // C++ iterates `for (auto &cl : m_pillar_clusters)` — cloned here
            // to release the field borrow while &mut self methods run.
            let cl = self.m_pillar_clusters[k].clone();
            // SupportTreeBuildsteps.cpp:824
            (self.m_thr)();

            // place all the centroid head positions into the index. We
            // will query for alternative pillar positions. If a sidehead
            // cannot connect to the cluster centroid, we have to search
            // for another head with a full pillar. Also when there are two
            // elements in the cluster, the centroid is arbitrary and the
            // sidehead is allowed to connect to a nearby pillar to
            // increase structural stability.

            // SupportTreeBuildsteps.cpp:834
            if cl.is_empty() {
                continue;
            }

            // get the current cluster centroid
            // SupportTreeBuildsteps.cpp:837-845
            let thr = self.m_thr.clone(); // auto & thr = m_thr; (captured by value below)
            let points = &self.m_points; // const auto &points = m_points;

            let lcid = cluster_centroid(
                &cl,
                // SupportTreeBuildsteps.cpp:841
                |idx: u32| -> Vec3d {
                    let row = points.row(idx as usize);
                    Vec3d::new(row[X], row[Y], row[Z])
                },
                // SupportTreeBuildsteps.cpp:842-845
                |p1: Vec3d, p2: Vec3d| -> f64 {
                    thr();
                    distance_between_vec2(&Vec2d::new(p1.x, p1.y), &Vec2d::new(p2.x, p2.y))
                },
            );

            // SupportTreeBuildsteps.cpp:847-848
            debug_assert!(lcid >= 0);
            let hid = cl[lcid as usize]; // Head ID

            // SupportTreeBuildsteps.cpp:850
            cl_centroids.push(hid);

            // SupportTreeBuildsteps.cpp:852
            let (h_jp, h_dir, h_r_back, h_id) = {
                let h = self.m_builder.head(hid);
                (h.junction_point(), h.dir, h.r_back_mm, h.id)
            };

            // SupportTreeBuildsteps.cpp:854-859
            if !self.create_ground_pillar(&h_jp, &h_dir, h_r_back, h_id) {
                log::warn!("Pillar cannot be created for support point id: {}", hid);
                self.m_iheads_onmodel.push(h_id as u32);
                continue;
            }
        }

        // now we will go through the clusters ones again and connect the
        // sidepoints with the cluster centroid (which is a ground pillar)
        // or a nearby pillar if the centroid is unreachable.
        // SupportTreeBuildsteps.cpp:865-889
        let mut ci = 0usize;
        for k in 0..self.m_pillar_clusters.len() {
            // SupportTreeBuildsteps.cpp:866 — for (auto cl : m_pillar_clusters)
            // (iterated BY VALUE in C++)
            let cl = self.m_pillar_clusters[k].clone();
            // SupportTreeBuildsteps.cpp:867
            (self.m_thr)();

            // SupportTreeBuildsteps.cpp:869
            let cidx = cl_centroids[ci];
            ci += 1;

            // SupportTreeBuildsteps.cpp:871
            let jp = self.m_builder.head(cidx).junction_point();
            let q = self.m_pillar_index.query_nearest(&jp, 1);
            // SupportTreeBuildsteps.cpp:872-888
            if !q.is_empty() {
                // SupportTreeBuildsteps.cpp:873
                let centerpillar_id = q[0].1 as i64;
                // SupportTreeBuildsteps.cpp:874-887
                for c in cl {
                    // SupportTreeBuildsteps.cpp:875-876
                    (self.m_thr)();
                    if c == cidx {
                        continue;
                    }

                    // SupportTreeBuildsteps.cpp:878
                    let sidehead = self.m_builder.head(c).clone();

                    // SupportTreeBuildsteps.cpp:880-885
                    if !self.connect_to_nearpillar(&sidehead, centerpillar_id)
                        && !self.search_pillar_and_connect(&sidehead)
                    {
                        let pstart = sidehead.junction_point();
                        // Vec3d pend = Vec3d{pstart(X), pstart(Y), gndlvl};
                        // Could not find a pillar, create one
                        self.create_ground_pillar(
                            &pstart,
                            &sidehead.dir,
                            sidehead.r_back_mm,
                            sidehead.id,
                        );
                    }
                }
            }
        }
    }

    // Find route for a head to the ground. Inserts additional bridge from the
    // head to the pillar if cannot create pillar directly.
    // The optional dir parameter is the direction of the bridge which is the
    // direction of the pinhead if omitted.
    // SupportTreeBuildsteps.hpp:301 (declaration) /
    // SupportTreeBuildsteps.cpp:892-914 (definition)
    // bool SupportTreeBuildsteps::connect_to_ground(Head &head, const Vec3d &dir)
    fn connect_to_ground_dir(&mut self, head: &Head, dir: &Vec3d) -> bool {
        // SupportTreeBuildsteps.cpp:894-896
        let hjp = head.junction_point();
        let r = head.r_back_mm;
        let mut t = self.bridge_mesh_distance(&hjp, dir, head.r_back_mm);
        // SupportTreeBuildsteps.cpp:897-898
        let mut d = 0.0;
        let mut tdown = 0.0;
        // t = std::min(t, m_cfg.max_bridge_length_mm * r / m_cfg.head_back_radius_mm);
        let lim = self.m_cfg.max_bridge_length_mm * r / self.m_cfg.head_back_radius_mm;
        t = if lim < t { lim } else { t };

        // SupportTreeBuildsteps.cpp:900-901
        // while (d < t && !std::isinf(tdown = bridge_mesh_distance(hjp + d * dir, DOWN, r)))
        //     d += r;
        loop {
            if !(d < t) {
                break;
            }
            tdown = self.bridge_mesh_distance(&(hjp + *dir * d), &DOWN, r);
            if tdown.is_infinite() {
                break;
            }
            d += r;
        }

        // SupportTreeBuildsteps.cpp:903
        if !tdown.is_infinite() {
            return false;
        }

        // SupportTreeBuildsteps.cpp:905-906
        let endp = hjp + *dir * d;
        let ret;

        // SupportTreeBuildsteps.cpp:908-911
        ret = self.create_ground_pillar(&endp, dir, head.r_back_mm, ID_UNSET);
        if ret {
            self.m_builder.add_bridge_from_head(head.id, endp);
            self.m_builder
                .add_junction(Junction::new(endp, head.r_back_mm));
        }

        // SupportTreeBuildsteps.cpp:913
        ret
    }

    // SupportTreeBuildsteps.hpp:302 (declaration) /
    // SupportTreeBuildsteps.cpp:916-942 (definition)
    // bool SupportTreeBuildsteps::connect_to_ground(Head &head)
    fn connect_to_ground(&mut self, head: &Head) -> bool {
        // SupportTreeBuildsteps.cpp:918
        if self.connect_to_ground_dir(head, &head.dir) {
            return true;
        }

        // Optimize bridge direction:
        // Straight path failed so we will try to search for a suitable
        // direction out of the cavity.
        // SupportTreeBuildsteps.cpp:923
        let (polar, azimuth) = dir_to_spheric(&head.dir, 1.0);

        // SupportTreeBuildsteps.cpp:925-926
        let mut criteria = get_criteria(self.m_cfg);
        criteria.stop_score(1e6);
        let mut solver = NLoptAlgCombOptimizer::new(alg_nlopt_genetic(), criteria);
        solver.seed(0); // we want deterministic behavior

        // SupportTreeBuildsteps.cpp:928-938
        let r_back = head.r_back_mm;
        let hjp = head.junction_point();
        let iv = initvals([polar, azimuth]); // let's start with what we have
        let bs = bounds([Bound::new(PI - self.m_cfg.bridge_slope, PI), Bound::new(-PI, PI)]);
        let ores = {
            let this = &*self;
            this.run_bridge_direction_optimization(&mut solver, hjp, r_back, &iv, &bs)
        };
        // BLOCKED BACKEND DIVERGENCE — see module notes. (With the backend
        // unavailable, `optimum == initvals`, so `bridgedir` reduces to the
        // re-normalized original head direction.)
        let oresult = opt_result_or_backend_unavailable(ores, &iv);

        // SupportTreeBuildsteps.cpp:940-941
        let bridgedir = eigen_normalized(spheric_to_dir_arr(&oresult.optimum));
        self.connect_to_ground_dir(head, &bridgedir)
    }

    // SupportTreeBuildsteps.cpp:930-938 — the objective lambda of
    // connect_to_ground(Head&):
    //     [this, hjp, r_back](const opt::Input<2> &input) {
    //         auto &[plr, azm] = input;
    //         Vec3d n = spheric_to_dir(plr, azm).normalized();
    //         return bridge_mesh_distance(hjp, n, r_back);
    //     }
    // (split into a helper so the immutable `self` capture is explicit)
    fn run_bridge_direction_optimization(
        &self,
        solver: &mut NLoptAlgCombOptimizer,
        hjp: Vec3d,
        r_back: f64,
        iv: &Input<2>,
        bs: &[Bound; 2],
    ) -> Result<OptResult<2>, NLoptBackendError> {
        solver.to_max().optimize(
            |input: &Input<2>| -> f64 {
                let [plr, azm] = *input;
                let n = eigen_normalized(spheric_to_dir(plr, azm));
                self.bridge_mesh_distance(&hjp, &n, r_back)
            },
            iv,
            bs,
        )
    }

    // SupportTreeBuildsteps.hpp:304 (declaration) /
    // SupportTreeBuildsteps.cpp:944-995 (definition)
    fn connect_to_model_body(&mut self, head: &Head) -> bool {
        // SupportTreeBuildsteps.cpp:946
        if head.id <= ID_UNSET {
            return false;
        }

        // SupportTreeBuildsteps.cpp:948-949
        let hit = match self.m_head_to_ground_scans.get(&(head.id as u32)) {
            Some(h) => *h,
            None => return false,
        };

        // SupportTreeBuildsteps.cpp:953-956
        if !hit.is_hit() {
            // TODO scan for potential anchor points on model surface
            return false;
        }

        // SupportTreeBuildsteps.cpp:958-961
        let hjp = head.junction_point();
        let mut zangle = hit.direction()[Z].asin();
        // zangle = std::max(zangle, PI/4);
        zangle = if zangle < PI / 4.0 { PI / 4.0 } else { zangle };
        let mut h = zangle.sin() * head.fullwidth();

        // The width of the tail head that we would like to have...
        // SupportTreeBuildsteps.cpp:964 — h = std::min(hit.distance() - head.r_back_mm, h);
        // (std::min(a,b) == (b<a)?b:a)
        let cap = hit.distance() - head.r_back_mm;
        h = if h < cap { h } else { cap };

        // If this is a mini pillar dont bother with the tail width, can be 0.
        // SupportTreeBuildsteps.cpp:967-968
        if head.r_back_mm < self.m_cfg.head_back_radius_mm {
            // h = std::max(h, 0.);
            h = if h < 0.0 { 0.0 } else { h };
        } else if h <= 0.0 {
            return false;
        }

        // SupportTreeBuildsteps.cpp:970-971
        let endp = Vec3d::new(hjp.x, hjp.y, hjp.z - hit.distance() + h);
        let center_hit = self.m_mesh.query_ray_hit(&to_na(&hjp), &to_na(&DOWN));

        // SupportTreeBuildsteps.cpp:973-975
        let hitdiff = center_hit.distance() - hit.distance();
        let hitp = if hitdiff.abs() < 2.0 * head.r_back_mm {
            from_na(&center_hit.position())
        } else {
            from_na(&hit.position())
        };

        // SupportTreeBuildsteps.cpp:977-978
        let pillar_id = self.m_builder.add_pillar_from_head(head.id, hjp.z - endp.z);
        let (pill_endpt, pill_id) = {
            let pill = self.m_builder.pillar(pillar_id);
            (*pill.endpoint(), pill.id)
        };

        // SupportTreeBuildsteps.cpp:980-982
        let taildir = endp - hitp;
        let dist = (hitp - endp).norm() + self.m_cfg.head_penetration_mm;
        let mut w = dist - 2.0 * head.r_pin_mm - head.r_back_mm;

        // SupportTreeBuildsteps.cpp:984-987
        if w < 0.0 {
            log::error!("Pinhead width is negative!");
            w = 0.0;
        }

        // SupportTreeBuildsteps.cpp:989-990
        self.m_builder.add_anchor(Anchor::new(
            head.r_back_mm,
            head.r_pin_mm,
            w,
            self.m_cfg.head_penetration_mm,
            taildir,
            hitp,
        ));

        // SupportTreeBuildsteps.cpp:992
        self.m_pillar_index.guarded_insert(pill_endpt, pill_id as u32);

        // SupportTreeBuildsteps.cpp:994
        true
    }

    // SupportTreeBuildsteps.hpp:306 (declaration) /
    // SupportTreeBuildsteps.cpp:997-1032 (definition)
    fn search_pillar_and_connect(&mut self, source: &Head) -> bool {
        // Hope that a local copy takes less time than the whole search loop.
        // We also need to remove elements progressively from the copied index.
        // SupportTreeBuildsteps.cpp:1001
        let mut spindex = self.m_pillar_index.guarded_clone();

        // SupportTreeBuildsteps.cpp:1003
        let mut nearest_id: i64 = ID_UNSET;

        // SupportTreeBuildsteps.cpp:1005
        let querypt = source.junction_point();

        // SupportTreeBuildsteps.cpp:1007-1029
        while nearest_id < 0 && !spindex.empty() {
            (self.m_thr)();
            // loop until a suitable head is not found
            // if there is a pillar closer than the cluster center
            // (this may happen as the clustering is not perfect)
            // than we will bridge to this closer pillar

            // SupportTreeBuildsteps.cpp:1013-1015
            let qp = Vec3d::new(querypt.x, querypt.y, self.m_builder.ground_level);
            let qres = spindex.nearest(&qp, 1);
            if qres.is_empty() {
                break;
            }

            // SupportTreeBuildsteps.cpp:1017-1018
            let ne = qres[0];
            nearest_id = ne.1 as i64;

            // SupportTreeBuildsteps.cpp:1020-1028
            if nearest_id >= 0 {
                if (nearest_id as usize) < self.m_builder.pillarcount() {
                    // SupportTreeBuildsteps.cpp:1022-1023
                    if !self.connect_to_nearpillar(source, nearest_id)
                        || self.m_builder.pillar(nearest_id).r < source.r_back_mm
                    {
                        nearest_id = ID_UNSET; // continue searching
                        spindex.remove(&ne); // without the current pillar
                    }
                }
            }
        }

        // SupportTreeBuildsteps.cpp:1031
        nearest_id >= 0
    }

    // Step: routing the pinheads that would connect to the model surface
    // along the Z axis downwards. For now these will actually be connected with
    // the model surface with a flipped pinhead. In the future here we could use
    // some smart algorithms to search for a safe path to the ground or to a
    // nearby pillar that can hold the supported weight.
    // SupportTreeBuildsteps.hpp:366 (declaration) /
    // SupportTreeBuildsteps.cpp:1034-1062 (definition)
    pub fn routing_to_model(&mut self) {
        // We need to check if there is an easy way out to the bed surface.
        // If it can be routed there with a bridge shorter than
        // min_bridge_distance.

        // SupportTreeBuildsteps.cpp:1040-1041
        // ccr::for_each(m_iheads_onmodel.begin(), m_iheads_onmodel.end(), ...)
        // (sequential execution of the parallel loop — the body mutates the
        //  shared builder under C++ mutexes; see module notes)
        let idxs = self.m_iheads_onmodel.clone();
        for idx in idxs {
            // SupportTreeBuildsteps.cpp:1042
            (self.m_thr)();

            // SupportTreeBuildsteps.cpp:1044
            let head = self.m_builder.head(idx).clone();

            // Search nearby pillar
            // SupportTreeBuildsteps.cpp:1047
            if self.search_pillar_and_connect(&head) {
                continue;
            }

            // Cannot connect to nearby pillar. We will try to search for
            // a route to the ground.
            // SupportTreeBuildsteps.cpp:1051
            if self.connect_to_ground(&head) {
                continue;
            }

            // No route to the ground, so connect to the model body as a last resort
            // SupportTreeBuildsteps.cpp:1054
            if self.connect_to_model_body(&head) {
                continue;
            }

            // We have failed to route this head.
            // SupportTreeBuildsteps.cpp:1057-1058
            log::warn!("Failed to route model facing support point. ID: {}", idx);

            // SupportTreeBuildsteps.cpp:1060
            self.m_builder.head(idx).invalidate();
        }
    }

    // SupportTreeBuildsteps.cpp:1087-1148 — cascadefn
    // auto cascadefn = [this, d, &pairs, min_height_ratio, H1](const PointIndexEl& el)
    // (lambda becomes a method; the captures become parameters)
    fn cascadefn(
        &mut self,
        el: &PointIndexEl,
        d: f64,
        pairs: &mut BTreeSet<u64>,
        min_height_ratio: f64,
        h1: f64,
    ) {
        // SupportTreeBuildsteps.cpp:1090
        let qp = el.0; // endpoint of the pillar

        // SupportTreeBuildsteps.cpp:1092 — const Pillar& pillar = m_builder.pillar(el.second);
        // (actual pillar; immutable fields copied — `links` is re-read live
        //  below because increment_links mutates it through the builder)
        let pillar_id = el.1 as i64;
        let pillar = self.m_builder.pillar(pillar_id).clone();

        // Get the max number of neighbors a pillar should connect to
        // SupportTreeBuildsteps.cpp:1095
        let neighbors = SupportTreeConfig::PILLAR_CASCADE_NEIGHBORS;

        // connections are already enough for the pillar
        // SupportTreeBuildsteps.cpp:1098
        if self.m_builder.pillar(pillar_id).links >= neighbors {
            return;
        }

        // SupportTreeBuildsteps.cpp:1100
        let max_d = d * pillar.r / self.m_cfg.head_back_radius_mm;
        // Query all remaining points within reach
        // SupportTreeBuildsteps.cpp:1102-1104
        let mut qres = self
            .m_pillar_index
            .query(&|e: &PointIndexEl| distance_between(&e.0, &qp) < max_d);

        // sort the result by distance (have to check if this is needed)
        // SupportTreeBuildsteps.cpp:1107-1110
        qres.sort_by(|e1, e2| {
            distance_between(&e1.0, &qp)
                .partial_cmp(&distance_between(&e2.0, &qp))
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        // SupportTreeBuildsteps.cpp:1112-1147
        for re in &qres {
            // process the queried neighbors

            // SupportTreeBuildsteps.cpp:1114
            if re.1 == el.1 {
                continue; // Skip self
            }

            // SupportTreeBuildsteps.cpp:1116
            let a = el.1;
            let b = re.1;

            // Get unique hash for the given pair (order doesn't matter)
            // SupportTreeBuildsteps.cpp:1119
            let hashval = pairhash(a, b);

            // Search for the pair amongst the remembered pairs
            // SupportTreeBuildsteps.cpp:1122
            if pairs.contains(&(hashval as u64)) {
                continue;
            }

            // SupportTreeBuildsteps.cpp:1124
            let neighborpillar = self.m_builder.pillar(re.1 as i64).clone();

            // this neighbor is occupied, skip
            // SupportTreeBuildsteps.cpp:1127-1128
            if neighborpillar.links >= neighbors {
                continue;
            }
            if neighborpillar.r < pillar.r {
                continue;
            }

            // SupportTreeBuildsteps.cpp:1130-1143
            if self.interconnect(&pillar, &neighborpillar) {
                // SupportTreeBuildsteps.cpp:1131
                pairs.insert(hashval as u64);

                // If the interconnection length between the two pillars is
                // less than 50% of the longer pillar's height, don't count
                // SupportTreeBuildsteps.cpp:1135-1137
                if pillar.height < h1
                    || neighborpillar.height / pillar.height > min_height_ratio
                {
                    self.m_builder.increment_links(&pillar);
                }

                // SupportTreeBuildsteps.cpp:1139-1141
                if neighborpillar.height < h1
                    || pillar.height / neighborpillar.height > min_height_ratio
                {
                    self.m_builder.increment_links(&neighborpillar);
                }
            }

            // connections are enough for one pillar
            // SupportTreeBuildsteps.cpp:1146 — if(pillar.links >= neighbors) break;
            // (C++ `pillar` is a live reference; re-read through the builder)
            if self.m_builder.pillar(pillar_id).links >= neighbors {
                break;
            }
        }
    }

    // SupportTreeBuildsteps.hpp:368 (declaration) /
    // SupportTreeBuildsteps.cpp:1064-1275 (definition)
    pub fn interconnect_pillars(&mut self) {
        // Now comes the algorithm that connects pillars with each other.
        // Ideally every pillar should be connected with at least one of its
        // neighbors if that neighbor is within max_pillar_link_distance

        // Pillars with height exceeding H1 will require at least one neighbor
        // to connect with. Height exceeding H2 require two neighbors.
        // SupportTreeBuildsteps.cpp:1072-1074
        let h1 = SupportTreeConfig::MAX_SOLO_PILLAR_HEIGHT_MM;
        let h2 = SupportTreeConfig::MAX_DUAL_PILLAR_HEIGHT_MM;
        let d = self.m_cfg.max_pillar_link_distance_mm;

        // A connection between two pillars only counts if the height ratio is
        // bigger than 50%
        // SupportTreeBuildsteps.cpp:1078
        let min_height_ratio = 0.5;

        // SupportTreeBuildsteps.cpp:1080
        let mut pairs: BTreeSet<u64> = BTreeSet::new();

        // A function to connect one pillar with its neighbors. THe number of
        // neighbors is given in the configuration. This function if called
        // for every pillar in the pillar index. A pair of pillar will not
        // be connected multiple times this is ensured by the 'pairs' set which
        // remembers the processed pillar pairs
        // SupportTreeBuildsteps.cpp:1087-1148 — cascadefn (method above)

        // Run the cascade for the pillars in the index
        // SupportTreeBuildsteps.cpp:1151 — m_pillar_index.foreach(cascadefn);
        // (foreach over a snapshot of the elements: cascadefn needs &mut self
        //  while foreach borrows the index; cascadefn never inserts into the
        //  index, so iterating the snapshot is identical)
        let mut elements: Vec<PointIndexEl> = Vec::new();
        self.m_pillar_index.foreach(&mut |el| elements.push(el.clone()));
        for el in &elements {
            self.cascadefn(el, d, &mut pairs, min_height_ratio, h1);
        }

        // We would be done here if we could allow some pillars to not be
        // connected with any neighbors. But this might leave the support tree
        // unprintable.
        //
        // The current solution is to insert additional pillars next to these
        // lonely pillars. One or even two additional pillar might get inserted
        // depending on the length of the lonely pillar.

        // SupportTreeBuildsteps.cpp:1161
        let pillarcount = self.m_builder.pillarcount();

        // Again, go through all pillars, this time in the whole support tree
        // not just the index.
        // SupportTreeBuildsteps.cpp:1165-1274
        for pid in 0..pillarcount {
            // SupportTreeBuildsteps.cpp:1166
            // auto pillar = [this, pid]() { return m_builder.pillar(pid); };
            // (the C++ lambda returns the Pillar BY VALUE — a fresh copy per
            //  call; only immutable fields (r/height/endpt/bridges) and the
            //  pre-mutation `links` are read, so one copy per iteration is
            //  value-identical)
            let pillar = self.m_builder.pillar(pid as i64).clone();

            // Decide how many additional pillars will be needed:

            // SupportTreeBuildsteps.cpp:1170-1179
            let mut needpillars: u32 = 0;
            if pillar.bridges > self.m_cfg.max_bridges_on_pillar {
                needpillars = 3;
            } else if pillar.links < 2 && pillar.height > h2 {
                // Not enough neighbors to support this pillar
                needpillars = 2;
            } else if pillar.links < 1 && pillar.height > h1 {
                // No neighbors could be found and the pillar is too long.
                needpillars = 1;
            }

            // SupportTreeBuildsteps.cpp:1181-1182
            // needpillars = std::max(pillar().links, needpillars) - pillar().links;
            needpillars = pillar.links.max(needpillars) - pillar.links;
            if needpillars == 0 {
                continue;
            }

            // Search for new pillar locations:

            // SupportTreeBuildsteps.cpp:1186-1189
            let mut found = false;
            let mut alpha = 0.0; // goes to 2Pi
            let r = 2.0 * self.m_cfg.base_radius_mm;
            let pillarsp = pillar.startpoint();

            // temp value for starting point detection
            // SupportTreeBuildsteps.cpp:1192
            let sp = Vec3d::new(pillarsp.x, pillarsp.y, pillarsp.z - r);

            // A vector of bool for placement feasbility
            // SupportTreeBuildsteps.cpp:1195-1196
            let mut canplace: Vec<bool> = vec![false; needpillars as usize];
            let mut spts: Vec<Vec3d> = vec![Vec3d::new(0.0, 0.0, 0.0); needpillars as usize];

            // SupportTreeBuildsteps.cpp:1198-1200
            let gnd = self.m_builder.ground_level;
            let min_dist = self.m_cfg.pillar_base_safety_distance_mm
                + self.m_cfg.base_radius_mm
                + EPSILON;

            // SupportTreeBuildsteps.cpp:1202-1229
            while !found && alpha < 2.0 * PI {
                // SupportTreeBuildsteps.cpp:1203-1222
                let mut n: u32 = 0;
                while n < needpillars && (n == 0 || canplace[(n - 1) as usize]) {
                    // SupportTreeBuildsteps.cpp:1207-1211
                    let a = alpha + n as f64 * PI / 3.0;
                    let mut s = sp;
                    s.x += a.cos() * r;
                    s.y += a.sin() * r;
                    spts[n as usize] = s;

                    // Check the path vertically down
                    // SupportTreeBuildsteps.cpp:1214-1216
                    let check_from = s + Vec3d::new(0.0, 0.0, pillar.r);
                    let hr = self.bridge_mesh_intersect_default_sd(&check_from, &DOWN, pillar.r);
                    let gndsp = Vec3d::new(s.x, s.y, gnd);

                    // If the path is clear, check for pillar base collisions
                    // SupportTreeBuildsteps.cpp:1219-1221
                    canplace[n as usize] = hr.distance().is_infinite()
                        && self.m_mesh.squared_distance_simple(&to_na(&gndsp)).sqrt() > min_dist;

                    n += 1;
                }

                // SupportTreeBuildsteps.cpp:1224-1225
                found = canplace.iter().all(|&v| v);

                // 20 angles will be tried...
                // SupportTreeBuildsteps.cpp:1228
                alpha += 0.1 * PI;
            }

            // SupportTreeBuildsteps.cpp:1231-1232
            let mut newpills: Vec<i64> = Vec::with_capacity(needpillars as usize);

            // SupportTreeBuildsteps.cpp:1234-1259
            if found {
                for n in 0..needpillars as usize {
                    // SupportTreeBuildsteps.cpp:1236-1237
                    let s = spts[n];
                    let p = Pillar::new(Vec3d::new(s.x, s.y, gnd), s.z - gnd, pillar.r);

                    // SupportTreeBuildsteps.cpp:1239-1258
                    if self.interconnect(&pillar, &p) {
                        // SupportTreeBuildsteps.cpp:1240
                        let ppid = self.m_builder.add_pillar(p);
                        let pp = self.m_builder.pillar(ppid).clone();

                        // SupportTreeBuildsteps.cpp:1242
                        self.add_pillar_base(pp.id);

                        // SupportTreeBuildsteps.cpp:1244
                        self.m_pillar_index.insert(*pp.endpoint(), pp.id as u32);

                        // SupportTreeBuildsteps.cpp:1246-1248
                        self.m_builder.add_junction(Junction::new(s, pillar.r));
                        let t =
                            self.bridge_mesh_distance(&pillarsp, &dirv(&pillarsp, &s), pillar.r);
                        // SupportTreeBuildsteps.cpp:1249-1250
                        if distance_between(&pillarsp, &s) < t {
                            self.m_builder.add_bridge(pillarsp, s, pillar.r);
                        }

                        // SupportTreeBuildsteps.cpp:1252-1253
                        if pillar.endpoint().z > self.m_builder.ground_level + pillar.r {
                            self.m_builder
                                .add_junction(Junction::new(*pillar.endpoint(), pillar.r));
                        }

                        // SupportTreeBuildsteps.cpp:1255-1257
                        newpills.push(pp.id);
                        self.m_builder.increment_links(&pillar);
                        self.m_builder.increment_links(&pp);
                    }
                }
            }

            // SupportTreeBuildsteps.cpp:1261-1273
            if !newpills.is_empty() {
                // SupportTreeBuildsteps.cpp:1262-1270
                for w in newpills.windows(2) {
                    let itpll = self.m_builder.pillar(w[0]).clone();
                    let nxpll = self.m_builder.pillar(w[1]).clone();
                    if self.interconnect(&itpll, &nxpll) {
                        self.m_builder.increment_links(&itpll);
                        self.m_builder.increment_links(&nxpll);
                    }
                }

                // SupportTreeBuildsteps.cpp:1272 — m_pillar_index.foreach(cascadefn);
                // (snapshot iteration, see above)
                let mut elements: Vec<PointIndexEl> = Vec::new();
                self.m_pillar_index
                    .foreach(&mut |el| elements.push(el.clone()));
                for el in &elements {
                    self.cascadefn(el, d, &mut pairs, min_height_ratio, h1);
                }
            }
        }
    }

    // SupportTreeBuildsteps.hpp:370
    // inline void merge_result() { m_builder.merged_mesh(); }
    #[inline]
    pub fn merge_result(&mut self) {
        // (C++ default argument steps = 45, SupportTreeBuilder.hpp:421)
        self.m_builder.merged_mesh(45);
    }
}

// SupportTreeBuildsteps.cpp:1277 — }} // namespace Slic3r::sla
