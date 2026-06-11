//! Faithful port of SLA/SupportTreeBuilder.{hpp,cpp}.
//!
//! C++ Reference:
//! - SLA/SupportTreeBuilder.hpp
//! - SLA/SupportTreeBuilder.cpp
//!
//! SupportTreeBuilder.cpp:1 — #define NOMINMAX (Windows macro guard, N/A)
//! C++ includes (SupportTreeBuilder.cpp:3-6): SupportTreeBuilder.hpp,
//! SupportTreeBuildsteps.hpp, SupportTreeMesher.hpp, //Contour3D.hpp (off).
//! Header includes (SupportTreeBuilder.hpp:4-9): Concurrency, SupportTree,
//! //Contour3D (off), TriangleMesh, Pad, MTUtils.
//!
//! SupportTreeBuilder.hpp:14-51
//! /**
//!  * Terminology:
//!  *
//!  * Support point:
//!  * The point on the model surface that needs support.
//!  *
//!  * Pillar:
//!  * A thick column that spans from a support point to the ground and has
//!  * a thick cone shaped base where it touches the ground.
//!  *
//!  * Ground facing support point:
//!  * A support point that can be directly connected with the ground with a pillar
//!  * that does not collide or cut through the model.
//!  *
//!  * Non ground facing support point:
//!  * A support point that cannot be directly connected with the ground (only with
//!  * the model surface).
//!  *
//!  * Head:
//!  * The pinhead that connects to the model surface with the sharp end end
//!  * to a pillar or bridge stick with the dull end.
//!  *
//!  * Headless support point:
//!  * A support point on the model surface for which there is not enough place for
//!  * the head. It is either in a hole or there is some barrier that would collide
//!  * with the head geometry. The headless support point can be ground facing and
//!  * non ground facing as well.
//!  *
//!  * Bridge:
//!  * A stick that connects two pillars or a head with a pillar.
//!  *
//!  * Junction:
//!  * A small ball in the intersection of two or more sticks (pillar, bridge, ...)
//!  *
//!  * CompactBridge:
//!  * A bridge that connects a headless support point with the model surface or a
//!  * nearby pillar.
//!  */

use crate::geometry::Vec3d;
use crate::normal_utils::indexed_triangle_set;
use crate::sla::ccr;
use crate::sla::job_controller::JobController;
use crate::sla::pad::PadConfig;
use crate::sla::support_tree_mesher::{
    get_mesh_bridge, get_mesh_diff_bridge, get_mesh_head, get_mesh_junction, get_mesh_pedestal,
    get_mesh_pillar,
};
use crate::triangle_mesh::{bounding_box, its_merge, its_merge_vertices};

// ---------------------------------------------------------------------------
// SupportTreeBuilder.hpp
// ---------------------------------------------------------------------------

// SupportTreeBuilder.hpp:53-55
// template<class Vec> double distance(const Vec& p) {
//     return std::sqrt(p.transpose() * p);
// }
// (Monomorphized for Vec3d, the only instantiation used by the SLA code.)
pub fn distance(p: &Vec3d) -> f64 {
    // SupportTreeBuilder.hpp:54
    p.dot(p).sqrt()
}

// SupportTreeBuilder.hpp:57-60
// template<class Vec> double distance(const Vec& pp1, const Vec& pp2) {
//     auto p = pp2 - pp1;
//     return distance(p);
// }
// (C++ overload of `distance`; Rust cannot overload, hence the suffixed name.)
pub fn distance_between(pp1: &Vec3d, pp2: &Vec3d) -> f64 {
    // SupportTreeBuilder.hpp:58
    let p = *pp2 - *pp1;
    // SupportTreeBuilder.hpp:59
    distance(&p)
}

// SupportTreeBuilder.hpp:62 — const Vec3d DOWN = {0.0, 0.0, -1.0};
pub const DOWN: Vec3d = Vec3d {
    x: 0.0,
    y: 0.0,
    z: -1.0,
};

// SupportTreeBuilder.hpp:64-69
// struct SupportTreeNode
// {
//     static const constexpr long ID_UNSET = -1;
//     long id = ID_UNSET; // For identification withing a tree.
// };
//
// C++ uses this as a base class for Head, Junction, Pillar, Pedestal, Bridge,
// DiffBridge. Rust has no struct inheritance, so each subtype carries the
// flattened `id` field and implements this trait, which is what the generic
// `_add_bridge` (SupportTreeBuilder.hpp:241-249) needs to set `id`.
pub trait SupportTreeNode {
    // SupportTreeBuilder.hpp:66
    const ID_UNSET: i64 = -1;

    // SupportTreeBuilder.hpp:68 — long id = ID_UNSET;
    fn id(&self) -> i64;
    fn set_id(&mut self, id: i64);
}

// SupportTreeBuilder.hpp:66 — module-level convenience mirror of
// SupportTreeNode::ID_UNSET.
pub const ID_UNSET: i64 = -1;

/// A pinhead originating from a support point
// SupportTreeBuilder.hpp:71-72 — struct Head: public SupportTreeNode
#[derive(Debug, Clone)]
pub struct Head {
    // SupportTreeBuilder.hpp:68 (SupportTreeNode base) — long id = ID_UNSET;
    pub id: i64,
    // SupportTreeBuilder.hpp:73 — Vec3d dir = DOWN;
    pub dir: Vec3d,
    // SupportTreeBuilder.hpp:74 — Vec3d pos = {0, 0, 0};
    pub pos: Vec3d,

    // SupportTreeBuilder.hpp:76 — double r_back_mm = 1;
    pub r_back_mm: f64,
    // SupportTreeBuilder.hpp:77 — double r_pin_mm = 0.5;
    pub r_pin_mm: f64,
    // SupportTreeBuilder.hpp:78 — double width_mm = 2;
    pub width_mm: f64,
    // SupportTreeBuilder.hpp:79 — double penetration_mm = 0.5;
    pub penetration_mm: f64,

    // If there is a pillar connecting to this head, then the id will be set.
    // SupportTreeBuilder.hpp:82-83 — long pillar_id = ID_UNSET;
    pub pillar_id: i64,

    // SupportTreeBuilder.hpp:85 — long bridge_id = ID_UNSET;
    pub bridge_id: i64,
}

impl SupportTreeNode for Head {
    fn id(&self) -> i64 {
        self.id
    }
    fn set_id(&mut self, id: i64) {
        self.id = id;
    }
}

impl Head {
    // SupportTreeBuilder.hpp:90-96 (declaration) / SupportTreeBuilder.cpp:11-24
    // Head::Head(double       r_big_mm,
    //            double       r_small_mm,
    //            double       length_mm,
    //            double       penetration,
    //            const Vec3d &direction,   // = DOWN (normal to the dull end)
    //            const Vec3d &offset)      // = {0, 0, 0} (displacement)
    //     : dir(direction)
    //     , pos(offset)
    //     , r_back_mm(r_big_mm)
    //     , r_pin_mm(r_small_mm)
    //     , width_mm(length_mm)
    //     , penetration_mm(penetration)
    // (C++ default arguments `direction = DOWN`, `offset = {0,0,0}` cannot be
    //  expressed in Rust; callers pass them explicitly.)
    pub fn new(
        r_big_mm: f64,
        r_small_mm: f64,
        length_mm: f64,
        penetration: f64,
        direction: Vec3d,
        offset: Vec3d,
    ) -> Self {
        Self {
            // SupportTreeBuilder.cpp:17
            dir: direction,
            // SupportTreeBuilder.cpp:18
            pos: offset,
            // SupportTreeBuilder.cpp:19
            r_back_mm: r_big_mm,
            // SupportTreeBuilder.cpp:20
            r_pin_mm: r_small_mm,
            // SupportTreeBuilder.cpp:21
            width_mm: length_mm,
            // SupportTreeBuilder.cpp:22
            penetration_mm: penetration,
            // id, pillar_id, bridge_id keep their default member initializers
            // (SupportTreeBuilder.hpp:68,83,85).
            id: ID_UNSET,
            pillar_id: ID_UNSET,
            bridge_id: ID_UNSET,
        }
    }

    // SupportTreeBuilder.hpp:87 — inline void invalidate() { id = ID_UNSET; }
    #[inline]
    pub fn invalidate(&mut self) {
        self.id = ID_UNSET;
    }

    // SupportTreeBuilder.hpp:88 — inline bool is_valid() const { return id >= 0; }
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.id >= 0
    }

    // SupportTreeBuilder.hpp:98-101
    // inline double real_width() const
    // {
    //     return 2 * r_pin_mm + width_mm + 2 * r_back_mm ;
    // }
    #[inline]
    pub fn real_width(&self) -> f64 {
        2.0 * self.r_pin_mm + self.width_mm + 2.0 * self.r_back_mm
    }

    // SupportTreeBuilder.hpp:103-106
    // inline double fullwidth() const
    // {
    //     return real_width() - penetration_mm;
    // }
    #[inline]
    pub fn fullwidth(&self) -> f64 {
        self.real_width() - self.penetration_mm
    }

    // SupportTreeBuilder.hpp:108-111
    // inline Vec3d junction_point() const
    // {
    //     return pos + (fullwidth() - r_back_mm) * dir;
    // }
    // (scalar * vector reordered to vector * scalar; identical result)
    #[inline]
    pub fn junction_point(&self) -> Vec3d {
        self.pos + self.dir * (self.fullwidth() - self.r_back_mm)
    }

    // SupportTreeBuilder.hpp:113-117
    // inline double request_pillar_radius(double radius) const
    // {
    //     const double rmax = r_back_mm;
    //     return radius > 0 && radius < rmax ? radius : rmax;
    // }
    #[inline]
    pub fn request_pillar_radius(&self, radius: f64) -> f64 {
        let rmax = self.r_back_mm;
        if radius > 0.0 && radius < rmax {
            radius
        } else {
            rmax
        }
    }
}

/// A junction connecting bridges and pillars
// SupportTreeBuilder.hpp:120-126 — struct Junction: public SupportTreeNode
#[derive(Debug, Clone)]
pub struct Junction {
    // SupportTreeBuilder.hpp:68 (SupportTreeNode base)
    pub id: i64,
    // SupportTreeBuilder.hpp:122 — double r = 1;
    pub r: f64,
    // SupportTreeBuilder.hpp:123 — Vec3d pos;
    pub pos: Vec3d,
}

impl SupportTreeNode for Junction {
    fn id(&self) -> i64 {
        self.id
    }
    fn set_id(&mut self, id: i64) {
        self.id = id;
    }
}

impl Junction {
    // SupportTreeBuilder.hpp:125
    // Junction(const Vec3d &tr, double r_mm) : r(r_mm), pos(tr) {}
    pub fn new(tr: Vec3d, r_mm: f64) -> Self {
        Self {
            id: ID_UNSET,
            r: r_mm,
            pos: tr,
        }
    }
}

// SupportTreeBuilder.hpp:128 — struct Pillar: public SupportTreeNode
#[derive(Debug, Clone)]
pub struct Pillar {
    // SupportTreeBuilder.hpp:68 (SupportTreeNode base)
    pub id: i64,
    // SupportTreeBuilder.hpp:129 — double height, r;
    pub height: f64,
    pub r: f64,
    // SupportTreeBuilder.hpp:130 — Vec3d endpt;
    pub endpt: Vec3d,

    // If the pillar connects to a head, this is the id of that head
    // SupportTreeBuilder.hpp:132-133 — bool starts_from_head = true;
    // Could start from a junction as well
    pub starts_from_head: bool,
    // SupportTreeBuilder.hpp:134 — long start_junction_id = ID_UNSET;
    pub start_junction_id: i64,

    // How many bridges are connected to this pillar
    // SupportTreeBuilder.hpp:136-137 — unsigned bridges = 0;
    pub bridges: u32,

    // How many pillars are cascaded with this one
    // SupportTreeBuilder.hpp:139-140 — unsigned links = 0;
    pub links: u32,
}

impl SupportTreeNode for Pillar {
    fn id(&self) -> i64 {
        self.id
    }
    fn set_id(&mut self, id: i64) {
        self.id = id;
    }
}

impl Pillar {
    // SupportTreeBuilder.hpp:142-143
    // Pillar(const Vec3d &endp, double h, double radius = 1.):
    //     height{h}, r(radius), endpt(endp), starts_from_head(false) {}
    // (C++ default argument `radius = 1.` cannot be expressed in Rust.)
    pub fn new(endp: Vec3d, h: f64, radius: f64) -> Self {
        Self {
            id: ID_UNSET,
            height: h,
            r: radius,
            endpt: endp,
            starts_from_head: false,
            start_junction_id: ID_UNSET,
            bridges: 0,
            links: 0,
        }
    }

    // SupportTreeBuilder.hpp:145-148
    // Vec3d startpoint() const
    // {
    //     return {endpt.x(), endpt.y(), endpt.z() + height};
    // }
    pub fn startpoint(&self) -> Vec3d {
        Vec3d {
            x: self.endpt.x(),
            y: self.endpt.y(),
            z: self.endpt.z() + self.height,
        }
    }

    // SupportTreeBuilder.hpp:150
    // const Vec3d& endpoint() const { return endpt; }
    pub fn endpoint(&self) -> &Vec3d {
        &self.endpt
    }
}

/// A base for pillars or bridges that end on the ground
// SupportTreeBuilder.hpp:153-161 — struct Pedestal: public SupportTreeNode
#[derive(Debug, Clone)]
pub struct Pedestal {
    // SupportTreeBuilder.hpp:68 (SupportTreeNode base)
    pub id: i64,
    // SupportTreeBuilder.hpp:155 — Vec3d pos;
    pub pos: Vec3d,
    // SupportTreeBuilder.hpp:156 — double height, r_bottom, r_top;
    pub height: f64,
    pub r_bottom: f64,
    pub r_top: f64,
}

impl SupportTreeNode for Pedestal {
    fn id(&self) -> i64 {
        self.id
    }
    fn set_id(&mut self, id: i64) {
        self.id = id;
    }
}

impl Pedestal {
    // SupportTreeBuilder.hpp:158-160
    // Pedestal(const Vec3d &p, double h, double rbottom, double rtop)
    //     : pos{p}, height{h}, r_bottom{rbottom}, r_top{rtop}
    // {}
    pub fn new(p: Vec3d, h: f64, rbottom: f64, rtop: f64) -> Self {
        Self {
            id: ID_UNSET,
            pos: p,
            height: h,
            r_bottom: rbottom,
            r_top: rtop,
        }
    }
}

/// This is the thing that anchors a pillar or bridge to the model body.
/// It is actually a reverse pinhead.
// SupportTreeBuilder.hpp:163-165
// struct Anchor: public Head { using Head::Head; };
//
// (Newtype over Head: keeps the distinct type — needed for the dedicated
//  `get_mesh(const Anchor&)` overload in SupportTreeMesher.hpp — while
//  Deref/DerefMut provide the inherited member access.)
#[derive(Debug, Clone)]
pub struct Anchor(pub Head);

impl std::ops::Deref for Anchor {
    type Target = Head;
    fn deref(&self) -> &Head {
        &self.0
    }
}

impl std::ops::DerefMut for Anchor {
    fn deref_mut(&mut self) -> &mut Head {
        &mut self.0
    }
}

impl SupportTreeNode for Anchor {
    fn id(&self) -> i64 {
        self.0.id
    }
    fn set_id(&mut self, id: i64) {
        self.0.id = id;
    }
}

impl Anchor {
    // SupportTreeBuilder.hpp:165 — using Head::Head; (inherited constructor)
    pub fn new(
        r_big_mm: f64,
        r_small_mm: f64,
        length_mm: f64,
        penetration: f64,
        direction: Vec3d,
        offset: Vec3d,
    ) -> Self {
        Anchor(Head::new(
            r_big_mm, r_small_mm, length_mm, penetration, direction, offset,
        ))
    }
}

/// A Bridge between two pillars (with junction endpoints)
// SupportTreeBuilder.hpp:167-179 — struct Bridge: public SupportTreeNode
#[derive(Debug, Clone)]
pub struct Bridge {
    // SupportTreeBuilder.hpp:68 (SupportTreeNode base)
    pub id: i64,
    // SupportTreeBuilder.hpp:169 — double r = 0.8;
    pub r: f64,
    // SupportTreeBuilder.hpp:170 — Vec3d startp = Vec3d::Zero(), endp = Vec3d::Zero();
    pub startp: Vec3d,
    pub endp: Vec3d,
}

impl SupportTreeNode for Bridge {
    fn id(&self) -> i64 {
        self.id
    }
    fn set_id(&mut self, id: i64) {
        self.id = id;
    }
}

impl Bridge {
    // SupportTreeBuilder.hpp:172-175
    // Bridge(const Vec3d &j1,
    //        const Vec3d &j2,
    //        double       r_mm  = 0.8): r{r_mm}, startp{j1}, endp{j2}
    // {}
    // (C++ default argument `r_mm = 0.8` cannot be expressed in Rust.)
    pub fn new(j1: Vec3d, j2: Vec3d, r_mm: f64) -> Self {
        Self {
            id: ID_UNSET,
            r: r_mm,
            startp: j1,
            endp: j2,
        }
    }

    // SupportTreeBuilder.hpp:177
    // double get_length() const { return (endp - startp).norm(); }
    pub fn get_length(&self) -> f64 {
        (self.endp - self.startp).norm()
    }

    // SupportTreeBuilder.hpp:178
    // Vec3d  get_dir() const { return (endp - startp).normalized(); }
    // (Explicit division by the norm: Eigen's normalized() is unguarded,
    //  while this crate's Vec3::normalized() has an epsilon guard that would
    //  silently diverge for degenerate zero-length bridges.)
    pub fn get_dir(&self) -> Vec3d {
        let d = self.endp - self.startp;
        d / d.norm()
    }
}

// SupportTreeBuilder.hpp:181-187 — struct DiffBridge: public Bridge
#[derive(Debug, Clone)]
pub struct DiffBridge {
    // Base class subobject (Rust models the `: public Bridge` inheritance by
    // embedding; Deref/DerefMut provide the inherited member access).
    pub bridge: Bridge,
    // SupportTreeBuilder.hpp:182 — double end_r;
    pub end_r: f64,
}

impl std::ops::Deref for DiffBridge {
    type Target = Bridge;
    fn deref(&self) -> &Bridge {
        &self.bridge
    }
}

impl std::ops::DerefMut for DiffBridge {
    fn deref_mut(&mut self) -> &mut Bridge {
        &mut self.bridge
    }
}

impl SupportTreeNode for DiffBridge {
    fn id(&self) -> i64 {
        self.bridge.id
    }
    fn set_id(&mut self, id: i64) {
        self.bridge.id = id;
    }
}

impl DiffBridge {
    // SupportTreeBuilder.hpp:184-186
    // DiffBridge(const Vec3d &p_s, const Vec3d &p_e, double r_s, double r_e)
    //     : Bridge{p_s, p_e, r_s}, end_r{r_e}
    // {}
    pub fn new(p_s: Vec3d, p_e: Vec3d, r_s: f64, r_e: f64) -> Self {
        Self {
            bridge: Bridge::new(p_s, p_e, r_s),
            end_r: r_e,
        }
    }
}

/// A wrapper struct around the pad
// SupportTreeBuilder.hpp:189-204
#[derive(Debug, Clone, Default)]
pub struct Pad {
    // SupportTreeBuilder.hpp:191 — indexed_triangle_set tmesh;
    pub tmesh: indexed_triangle_set,
    // SupportTreeBuilder.hpp:192 — PadConfig cfg;
    pub cfg: PadConfig,
    // SupportTreeBuilder.hpp:193 — double zlevel = 0;
    pub zlevel: f64,
}

impl Pad {
    // SupportTreeBuilder.hpp:195 — Pad() = default;  → #[derive(Default)]

    // SupportTreeBuilder.hpp:197-201 / SupportTreeBuilder.cpp:26-48
    // Pad::Pad(const indexed_triangle_set &support_mesh,
    //          const ExPolygons &          model_contours,
    //          double                      ground_level,
    //          const PadConfig &           pcfg,
    //          ThrowOnCancel               thr)
    //     : cfg(pcfg)
    //     , zlevel(ground_level + pcfg.full_height() - pcfg.required_elevation())
    // {
    //     thr();
    //
    //     ExPolygons sup_contours;
    //
    //     float zstart = float(zlevel);
    //     float zend   = zstart + float(pcfg.full_height() + EPSILON);
    //
    //     pad_blueprint(support_mesh, sup_contours, grid(zstart, zend, 0.1f), thr);
    //     create_pad(sup_contours, model_contours, tmesh, pcfg);
    //
    //     Vec3f offs{.0f, .0f, float(zlevel)};
    //     for (auto &p : tmesh.vertices) p += offs;
    //
    //     its_merge_vertices(tmesh);
    // }
    //
    // BLOCKED: requires `pad_blueprint`, `create_pad`,
    // `PadConfig::full_height()`, `PadConfig::required_elevation()` and
    // `ThrowOnCancel` from SLA/Pad.cpp plus `grid()` from MTUtils.hpp —
    // SLA/Pad.cpp is still an auto-generated placeholder stub in this crate
    // (`sla/pad.rs`). No fake is provided; port SLA/Pad.cpp first, then add
    // this constructor.

    // SupportTreeBuilder.hpp:203
    // bool empty() const { return tmesh.indices.size() == 0; }
    pub fn empty(&self) -> bool {
        self.tmesh.indices.len() == 0
    }
}

// SupportTreeBuilder.hpp:234 — using Mutex = ccr::SpinningMutex;
// (The Rust `ccr::SpinningMutex<T>` carries a payload type; the C++ mutex is
//  data-less, so the unit type is used.)
type Mutex = ccr::SpinningMutex<()>;

/// This class will hold the support tree meshes with some additional
/// bookkeeping as well. Various parts of the support geometry are stored
/// separately and are merged when the caller queries the merged mesh. The
/// merged result is cached for fast subsequent delivery of the merged mesh
/// which can be quite complex. The support tree creation algorithm can use an
/// instance of this class as a somewhat higher level tool for crafting the 3D
/// support mesh. Parts can be added with the appropriate methods such as
/// add_head or add_pillar which forwards the constructor arguments and fills
/// the IDs of these substructures. The IDs are basically indices into the
/// arrays of the appropriate type (heads, pillars, etc...). One can later query
/// e.g. a pillar for a specific head...
///
/// The support pad is considered an auxiliary geometry and is not part of the
/// merged mesh. It can be retrieved using a dedicated method (pad())
// SupportTreeBuilder.hpp:206-220 — class SupportTreeBuilder: public SupportTree
//
// C++ inheritance note: the base class `SupportTree` (SupportTree.hpp:140-170)
// holds the private member `JobController m_ctl` (SupportTree.hpp:143). The
// Rust `SupportTree` is a trait and cannot hold data, so the member lives here
// (see the corresponding note in sla/support_tree.rs).
//
// Mutability note: the C++ `mutable` members (m_meshcache, m_mutex,
// m_meshcache_valid, m_model_height — SupportTreeBuilder.hpp:236-239) are
// mutated from const methods. The Rust port expresses those methods with
// `&mut self` instead of interior mutability; see the BLOCKED notes on
// merged_mesh()/retrieve_mesh() below.
#[derive(Default)]
pub struct SupportTreeBuilder {
    // SupportTree.hpp:143 (base class) — JobController m_ctl;
    // Public so that the future `SupportTree::create`
    // (SupportTree.cpp:80-96) can do `builder->m_ctl = ctl;`.
    pub m_ctl: JobController,

    // For heads it is beneficial to use the same IDs as for the support points.
    // SupportTreeBuilder.hpp:221-222 — std::vector<Head> m_heads;
    m_heads: Vec<Head>,
    // SupportTreeBuilder.hpp:223 — std::vector<size_t> m_head_indices;
    m_head_indices: Vec<usize>,
    // SupportTreeBuilder.hpp:224 — std::vector<Pillar> m_pillars;
    m_pillars: Vec<Pillar>,
    // SupportTreeBuilder.hpp:225 — std::vector<Junction> m_junctions;
    m_junctions: Vec<Junction>,
    // SupportTreeBuilder.hpp:226 — std::vector<Bridge> m_bridges;
    m_bridges: Vec<Bridge>,
    // SupportTreeBuilder.hpp:227 — std::vector<Bridge> m_crossbridges;
    m_crossbridges: Vec<Bridge>,
    // SupportTreeBuilder.hpp:228 — std::vector<DiffBridge> m_diffbridges;
    m_diffbridges: Vec<DiffBridge>,
    // SupportTreeBuilder.hpp:229 — std::vector<Pedestal> m_pedestals;
    m_pedestals: Vec<Pedestal>,
    // SupportTreeBuilder.hpp:230 — std::vector<Anchor> m_anchors;
    m_anchors: Vec<Anchor>,

    // SupportTreeBuilder.hpp:232 — Pad m_pad;
    m_pad: Pad,

    // SupportTreeBuilder.hpp:236 — mutable indexed_triangle_set m_meshcache;
    m_meshcache: indexed_triangle_set,
    // SupportTreeBuilder.hpp:237 — mutable Mutex m_mutex;
    m_mutex: Mutex,
    // SupportTreeBuilder.hpp:238 — mutable bool m_meshcache_valid = false;
    m_meshcache_valid: bool,
    // SupportTreeBuilder.hpp:239 — mutable double m_model_height = 0;
    // the full height of the model
    m_model_height: f64,

    // SupportTreeBuilder.hpp:252 — double ground_level = 0;
    pub ground_level: f64,
}

// SupportTreeBuilder.cpp:57-68 — move constructor and
// SupportTreeBuilder.cpp:83-96 — move assignment operator are subsumed by
// Rust's built-in move semantics. (Divergence note: the C++ move special
// members default-initialize the unlisted members m_junctions, m_diffbridges,
// m_pedestals, m_anchors and m_ctl in the destination, while a Rust move
// transfers all fields. No caller depends on that difference.)

impl Clone for SupportTreeBuilder {
    // SupportTreeBuilder.cpp:70-81 — copy constructor.
    // SupportTreeBuilder::SupportTreeBuilder(const SupportTreeBuilder &o)
    // The C++ mem-initializer list copies ONLY the members below; m_junctions,
    // m_diffbridges, m_pedestals, m_anchors, m_mutex and the base-class
    // JobController are default-initialized — reproduced faithfully here.
    fn clone(&self) -> Self {
        Self {
            // SupportTreeBuilder.cpp:71 — m_heads(o.m_heads)
            m_heads: self.m_heads.clone(),
            // SupportTreeBuilder.cpp:72 — m_head_indices{o.m_head_indices}
            m_head_indices: self.m_head_indices.clone(),
            // SupportTreeBuilder.cpp:73 — m_pillars{o.m_pillars}
            m_pillars: self.m_pillars.clone(),
            // SupportTreeBuilder.cpp:74 — m_bridges{o.m_bridges}
            m_bridges: self.m_bridges.clone(),
            // SupportTreeBuilder.cpp:75 — m_crossbridges{o.m_crossbridges}
            m_crossbridges: self.m_crossbridges.clone(),
            // SupportTreeBuilder.cpp:76 — m_pad{o.m_pad}
            m_pad: self.m_pad.clone(),
            // SupportTreeBuilder.cpp:77 — m_meshcache{o.m_meshcache}
            m_meshcache: self.m_meshcache.clone(),
            // SupportTreeBuilder.cpp:78 — m_meshcache_valid{o.m_meshcache_valid}
            m_meshcache_valid: self.m_meshcache_valid,
            // SupportTreeBuilder.cpp:79 — m_model_height{o.m_model_height}
            m_model_height: self.m_model_height,
            // SupportTreeBuilder.cpp:80 — ground_level{o.ground_level}
            ground_level: self.ground_level,
            // Members absent from the C++ mem-initializer list are
            // default-initialized:
            m_ctl: JobController::default(),
            m_junctions: Vec::new(),
            m_diffbridges: Vec::new(),
            m_pedestals: Vec::new(),
            m_anchors: Vec::new(),
            m_mutex: Mutex::default(),
        }
    }

    // SupportTreeBuilder.cpp:98-111 — copy assignment operator.
    // SupportTreeBuilder &SupportTreeBuilder::operator=(const SupportTreeBuilder &o)
    // Maps to Clone::clone_from: assigns ONLY the listed members; the
    // destination keeps its own m_junctions, m_diffbridges, m_pedestals,
    // m_anchors, m_mutex and m_ctl, exactly like the C++ operator=.
    fn clone_from(&mut self, o: &Self) {
        // SupportTreeBuilder.cpp:100
        self.m_heads = o.m_heads.clone();
        // SupportTreeBuilder.cpp:101
        self.m_head_indices = o.m_head_indices.clone();
        // SupportTreeBuilder.cpp:102
        self.m_pillars = o.m_pillars.clone();
        // SupportTreeBuilder.cpp:103
        self.m_bridges = o.m_bridges.clone();
        // SupportTreeBuilder.cpp:104
        self.m_crossbridges = o.m_crossbridges.clone();
        // SupportTreeBuilder.cpp:105
        self.m_pad = o.m_pad.clone();
        // SupportTreeBuilder.cpp:106
        self.m_meshcache = o.m_meshcache.clone();
        // SupportTreeBuilder.cpp:107
        self.m_meshcache_valid = o.m_meshcache_valid;
        // SupportTreeBuilder.cpp:108
        self.m_model_height = o.m_model_height;
        // SupportTreeBuilder.cpp:109
        self.ground_level = o.ground_level;
        // SupportTreeBuilder.cpp:110 — return *this;
    }
}

impl SupportTreeBuilder {
    // SupportTreeBuilder.hpp:254 — SupportTreeBuilder() = default;
    // → #[derive(Default)] (all C++ default member initializers are zero /
    //   false / empty, matching Rust's Default).

    // SupportTreeBuilder.hpp:241-249
    // template<class BridgeT, class...Args>
    // const BridgeT& _add_bridge(std::vector<BridgeT> &br, Args&&... args)
    // {
    //     std::lock_guard<Mutex> lk(m_mutex);
    //     br.emplace_back(std::forward<Args>(args)...);
    //     br.back().id = long(br.size() - 1);
    //     m_meshcache_valid = false;
    //     return br.back();
    // }
    //
    // (Associated fn taking the member vector + cache flag as disjoint field
    //  borrows, because Rust cannot pass `&mut self.m_bridges` alongside
    //  `&mut self`. The forwarded constructor arguments become the already
    //  constructed `item`, which is what emplace_back produces.)
    fn _add_bridge<'a, BridgeT: SupportTreeNode>(
        mutex: &Mutex,
        br: &'a mut Vec<BridgeT>,
        meshcache_valid: &mut bool,
        item: BridgeT,
    ) -> &'a BridgeT {
        // SupportTreeBuilder.hpp:244 — std::lock_guard<Mutex> lk(m_mutex);
        let _lk = mutex.lock();
        // SupportTreeBuilder.hpp:245 — br.emplace_back(...);
        br.push(item);
        // SupportTreeBuilder.hpp:246 — br.back().id = long(br.size() - 1);
        let id = (br.len() - 1) as i64;
        br.last_mut().unwrap().set_id(id);
        // SupportTreeBuilder.hpp:247
        *meshcache_valid = false;
        // SupportTreeBuilder.hpp:248
        br.last().unwrap()
    }

    // SupportTreeBuilder.hpp:260-271
    // template<class...Args> Head& add_head(unsigned id, Args&&... args)
    // (The forwarded Head constructor arguments become the already constructed
    //  `head` value.)
    pub fn add_head(&mut self, id: u32, head: Head) -> &mut Head {
        // SupportTreeBuilder.hpp:262 — std::lock_guard<Mutex> lk(m_mutex);
        let _lk = self.m_mutex.lock();
        // SupportTreeBuilder.hpp:263 — m_heads.emplace_back(...);
        self.m_heads.push(head);
        // SupportTreeBuilder.hpp:264 — m_heads.back().id = id;
        self.m_heads.last_mut().unwrap().id = id as i64;

        // SupportTreeBuilder.hpp:266
        // if (id >= m_head_indices.size()) m_head_indices.resize(id + 1);
        if id as usize >= self.m_head_indices.len() {
            self.m_head_indices.resize(id as usize + 1, 0);
        }
        // SupportTreeBuilder.hpp:267 — m_head_indices[id] = m_heads.size() - 1;
        let last = self.m_heads.len() - 1;
        self.m_head_indices[id as usize] = last;

        // SupportTreeBuilder.hpp:269
        self.m_meshcache_valid = false;
        // SupportTreeBuilder.hpp:270 — return m_heads.back();
        self.m_heads.last_mut().unwrap()
    }

    // SupportTreeBuilder.hpp:273-293
    // template<class...Args> long add_pillar(long headid, double length)
    // (Overload disambiguated as `add_pillar_from_head`; the C++ template
    //  parameter pack is unused in this signature.)
    pub fn add_pillar_from_head(&mut self, headid: i64, length: f64) -> i64 {
        // SupportTreeBuilder.hpp:275 — std::lock_guard<Mutex> lk(m_mutex);
        let _lk = self.m_mutex.lock();
        // SupportTreeBuilder.hpp:276-277
        // if (m_pillars.capacity() < m_heads.size())
        //     m_pillars.reserve(m_heads.size() * 10);
        // (C++ reserve(n) ensures capacity >= n; Rust reserve() takes the
        //  additional element count.)
        if self.m_pillars.capacity() < self.m_heads.len() {
            let want = self.m_heads.len() * 10;
            self.m_pillars
                .reserve(want.saturating_sub(self.m_pillars.len()));
        }

        // SupportTreeBuilder.hpp:279
        // assert(headid >= 0 && size_t(headid) < m_head_indices.size());
        debug_assert!(headid >= 0 && (headid as usize) < self.m_head_indices.len());
        // SupportTreeBuilder.hpp:280 — Head &head = m_heads[m_head_indices[size_t(headid)]];
        let head_idx = self.m_head_indices[headid as usize];

        // SupportTreeBuilder.hpp:282
        // Vec3d hjp = head.junction_point() - Vec3d{0, 0, length};
        let hjp = self.m_heads[head_idx].junction_point()
            - Vec3d {
                x: 0.0,
                y: 0.0,
                z: length,
            };
        // SupportTreeBuilder.hpp:283
        // m_pillars.emplace_back(hjp, length, head.r_back_mm);
        let r_back_mm = self.m_heads[head_idx].r_back_mm;
        self.m_pillars.push(Pillar::new(hjp, length, r_back_mm));

        // SupportTreeBuilder.hpp:285-286
        // Pillar& pillar = m_pillars.back();
        // pillar.id = long(m_pillars.size() - 1);
        let pillar_id = (self.m_pillars.len() - 1) as i64;
        let head_id = self.m_heads[head_idx].id;
        {
            let pillar = self.m_pillars.last_mut().unwrap();
            pillar.id = pillar_id;
            // SupportTreeBuilder.hpp:288 — pillar.start_junction_id = head.id;
            pillar.start_junction_id = head_id;
            // SupportTreeBuilder.hpp:289 — pillar.starts_from_head = true;
            pillar.starts_from_head = true;
        }
        // SupportTreeBuilder.hpp:287 — head.pillar_id = pillar.id;
        self.m_heads[head_idx].pillar_id = pillar_id;

        // SupportTreeBuilder.hpp:291
        self.m_meshcache_valid = false;
        // SupportTreeBuilder.hpp:292 — return pillar.id;
        pillar_id
    }

    // SupportTreeBuilder.hpp:295 (declaration) / SupportTreeBuilder.cpp:113-123
    // void SupportTreeBuilder::add_pillar_base(long pid, double baseheight, double radius)
    // (C++ default arguments `baseheight = 3`, `radius = 2` cannot be
    //  expressed in Rust; callers pass them explicitly.)
    pub fn add_pillar_base(&mut self, pid: i64, baseheight: f64, radius: f64) {
        // SupportTreeBuilder.cpp:115 — std::lock_guard<Mutex> lk(m_mutex);
        let _lk = self.m_mutex.lock();
        // SupportTreeBuilder.cpp:116
        // assert(pid >= 0 && size_t(pid) < m_pillars.size());
        debug_assert!(pid >= 0 && (pid as usize) < self.m_pillars.len());
        // SupportTreeBuilder.cpp:117 — Pillar& pll = m_pillars[size_t(pid)];
        let pll = &self.m_pillars[pid as usize];
        // SupportTreeBuilder.cpp:118-119
        // m_pedestals.emplace_back(pll.endpt, std::min(baseheight, pll.height),
        //                          std::max(radius, pll.r), pll.r);
        // (std::min(a,b) == (b<a)?b:a and std::max(a,b) == (a<b)?b:a, written
        //  out explicitly to keep the exact NaN/tie semantics.)
        let h = if pll.height < baseheight {
            pll.height
        } else {
            baseheight
        };
        let r = if radius < pll.r { pll.r } else { radius };
        let pedestal = Pedestal::new(pll.endpt, h, r, pll.r);
        self.m_pedestals.push(pedestal);

        // SupportTreeBuilder.cpp:121
        // m_pedestals.back().id = m_pedestals.size() - 1;
        let id = (self.m_pedestals.len() - 1) as i64;
        self.m_pedestals.last_mut().unwrap().id = id;
        // SupportTreeBuilder.cpp:122
        self.m_meshcache_valid = false;
    }

    // SupportTreeBuilder.hpp:297-304
    // template<class...Args> const Anchor& add_anchor(Args&&...args)
    // (The forwarded Anchor/Head constructor arguments become the already
    //  constructed `anchor` value.)
    pub fn add_anchor(&mut self, anchor: Anchor) -> &Anchor {
        // SupportTreeBuilder.hpp:299 — std::lock_guard<Mutex> lk(m_mutex);
        let _lk = self.m_mutex.lock();
        // SupportTreeBuilder.hpp:300 — m_anchors.emplace_back(...);
        self.m_anchors.push(anchor);
        // SupportTreeBuilder.hpp:301
        // m_anchors.back().id = long(m_junctions.size() - 1);
        // (sic: the C++ derives the anchor id from m_junctions — preserved
        //  faithfully, including the size_t wrap-around to -1 when
        //  m_junctions is empty.)
        let id = self.m_junctions.len().wrapping_sub(1) as i64;
        self.m_anchors.last_mut().unwrap().set_id(id);
        // SupportTreeBuilder.hpp:302
        self.m_meshcache_valid = false;
        // SupportTreeBuilder.hpp:303 — return m_anchors.back();
        self.m_anchors.last().unwrap()
    }

    // SupportTreeBuilder.hpp:306-313
    // void increment_bridges(const Pillar& pillar)
    pub fn increment_bridges(&mut self, pillar: &Pillar) {
        // SupportTreeBuilder.hpp:308 — std::lock_guard<Mutex> lk(m_mutex);
        let _lk = self.m_mutex.lock();
        // SupportTreeBuilder.hpp:309
        // assert(pillar.id >= 0 && size_t(pillar.id) < m_pillars.size());
        debug_assert!(pillar.id >= 0 && (pillar.id as usize) < self.m_pillars.len());

        // SupportTreeBuilder.hpp:311-312
        if pillar.id >= 0 && (pillar.id as usize) < self.m_pillars.len() {
            self.m_pillars[pillar.id as usize].bridges += 1;
        }
    }

    // SupportTreeBuilder.hpp:315-322
    // void increment_links(const Pillar& pillar)
    pub fn increment_links(&mut self, pillar: &Pillar) {
        // SupportTreeBuilder.hpp:317 — std::lock_guard<Mutex> lk(m_mutex);
        let _lk = self.m_mutex.lock();
        // SupportTreeBuilder.hpp:318
        // assert(pillar.id >= 0 && size_t(pillar.id) < m_pillars.size());
        debug_assert!(pillar.id >= 0 && (pillar.id as usize) < self.m_pillars.len());

        // SupportTreeBuilder.hpp:320-321
        if pillar.id >= 0 && (pillar.id as usize) < self.m_pillars.len() {
            self.m_pillars[pillar.id as usize].links += 1;
        }
    }

    // SupportTreeBuilder.hpp:324-328
    // unsigned bridgecount(const Pillar &pillar) const
    pub fn bridgecount(&self, pillar: &Pillar) -> u32 {
        // SupportTreeBuilder.hpp:325 — std::lock_guard<Mutex> lk(m_mutex);
        let _lk = self.m_mutex.lock();
        // SupportTreeBuilder.hpp:326
        debug_assert!(pillar.id >= 0 && (pillar.id as usize) < self.m_pillars.len());
        // SupportTreeBuilder.hpp:327 — return pillar.bridges;
        // (sic: returns the field of the passed-in pillar, not of
        //  m_pillars[pillar.id] — preserved faithfully.)
        pillar.bridges
    }

    // SupportTreeBuilder.hpp:330-342
    // template<class...Args> long add_pillar(Args&&...args)
    // (The forwarded Pillar constructor arguments become the already
    //  constructed `pillar` value.)
    pub fn add_pillar(&mut self, pillar: Pillar) -> i64 {
        // SupportTreeBuilder.hpp:332 — std::lock_guard<Mutex> lk(m_mutex);
        let _lk = self.m_mutex.lock();
        // SupportTreeBuilder.hpp:333-334
        // if (m_pillars.capacity() < m_heads.size())
        //     m_pillars.reserve(m_heads.size() * 10);
        if self.m_pillars.capacity() < self.m_heads.len() {
            let want = self.m_heads.len() * 10;
            self.m_pillars
                .reserve(want.saturating_sub(self.m_pillars.len()));
        }

        // SupportTreeBuilder.hpp:336 — m_pillars.emplace_back(...);
        self.m_pillars.push(pillar);
        // SupportTreeBuilder.hpp:337-338
        // Pillar& pillar = m_pillars.back();
        // pillar.id = long(m_pillars.size() - 1);
        let id = (self.m_pillars.len() - 1) as i64;
        let p = self.m_pillars.last_mut().unwrap();
        p.id = id;
        // SupportTreeBuilder.hpp:339 — pillar.starts_from_head = false;
        p.starts_from_head = false;
        // SupportTreeBuilder.hpp:340
        self.m_meshcache_valid = false;
        // SupportTreeBuilder.hpp:341 — return pillar.id;
        id
    }

    // SupportTreeBuilder.hpp:344-351
    // template<class...Args> const Junction& add_junction(Args&&... args)
    // (The forwarded Junction constructor arguments become the already
    //  constructed `junction` value.)
    pub fn add_junction(&mut self, junction: Junction) -> &Junction {
        // SupportTreeBuilder.hpp:346 — std::lock_guard<Mutex> lk(m_mutex);
        let _lk = self.m_mutex.lock();
        // SupportTreeBuilder.hpp:347 — m_junctions.emplace_back(...);
        self.m_junctions.push(junction);
        // SupportTreeBuilder.hpp:348
        // m_junctions.back().id = long(m_junctions.size() - 1);
        let id = (self.m_junctions.len() - 1) as i64;
        self.m_junctions.last_mut().unwrap().id = id;
        // SupportTreeBuilder.hpp:349
        self.m_meshcache_valid = false;
        // SupportTreeBuilder.hpp:350 — return m_junctions.back();
        self.m_junctions.last().unwrap()
    }

    // SupportTreeBuilder.hpp:353-356
    // const Bridge& add_bridge(const Vec3d &s, const Vec3d &e, double r)
    // {
    //     return _add_bridge(m_bridges, s, e, r);
    // }
    pub fn add_bridge(&mut self, s: Vec3d, e: Vec3d, r: f64) -> &Bridge {
        // SupportTreeBuilder.hpp:355
        Self::_add_bridge(
            &self.m_mutex,
            &mut self.m_bridges,
            &mut self.m_meshcache_valid,
            Bridge::new(s, e, r),
        )
    }

    // SupportTreeBuilder.hpp:358-370
    // const Bridge& add_bridge(long headid, const Vec3d &endp)
    // (Overload disambiguated as `add_bridge_from_head`.)
    pub fn add_bridge_from_head(&mut self, headid: i64, endp: Vec3d) -> &Bridge {
        // SupportTreeBuilder.hpp:360 — std::lock_guard<Mutex> lk(m_mutex);
        let _lk = self.m_mutex.lock();
        // SupportTreeBuilder.hpp:361
        // assert(headid >= 0 && size_t(headid) < m_head_indices.size());
        debug_assert!(headid >= 0 && (headid as usize) < self.m_head_indices.len());

        // SupportTreeBuilder.hpp:363 — Head &h = m_heads[m_head_indices[size_t(headid)]];
        let head_idx = self.m_head_indices[headid as usize];
        // SupportTreeBuilder.hpp:364
        // m_bridges.emplace_back(h.junction_point(), endp, h.r_back_mm);
        let (jp, r_back_mm) = {
            let h = &self.m_heads[head_idx];
            (h.junction_point(), h.r_back_mm)
        };
        self.m_bridges.push(Bridge::new(jp, endp, r_back_mm));
        // SupportTreeBuilder.hpp:365
        // m_bridges.back().id = long(m_bridges.size() - 1);
        let id = (self.m_bridges.len() - 1) as i64;
        self.m_bridges.last_mut().unwrap().id = id;

        // SupportTreeBuilder.hpp:367 — h.bridge_id = m_bridges.back().id;
        self.m_heads[head_idx].bridge_id = id;
        // SupportTreeBuilder.hpp:368
        self.m_meshcache_valid = false;
        // SupportTreeBuilder.hpp:369 — return m_bridges.back();
        self.m_bridges.last().unwrap()
    }

    // SupportTreeBuilder.hpp:372-375
    // template<class...Args> const Bridge& add_crossbridge(Args&&... args)
    // {
    //     return _add_bridge(m_crossbridges, std::forward<Args>(args)...);
    // }
    pub fn add_crossbridge(&mut self, bridge: Bridge) -> &Bridge {
        // SupportTreeBuilder.hpp:374
        Self::_add_bridge(
            &self.m_mutex,
            &mut self.m_crossbridges,
            &mut self.m_meshcache_valid,
            bridge,
        )
    }

    // SupportTreeBuilder.hpp:377-380
    // template<class...Args> const DiffBridge& add_diffbridge(Args&&... args)
    // {
    //     return _add_bridge(m_diffbridges, std::forward<Args>(args)...);
    // }
    pub fn add_diffbridge(&mut self, bridge: DiffBridge) -> &DiffBridge {
        // SupportTreeBuilder.hpp:379
        Self::_add_bridge(
            &self.m_mutex,
            &mut self.m_diffbridges,
            &mut self.m_meshcache_valid,
            bridge,
        )
    }

    // SupportTreeBuilder.hpp:382-389
    // Head &head(unsigned id)
    pub fn head(&mut self, id: u32) -> &mut Head {
        // SupportTreeBuilder.hpp:384 — std::lock_guard<Mutex> lk(m_mutex);
        let _lk = self.m_mutex.lock();
        // SupportTreeBuilder.hpp:385 — assert(id < m_head_indices.size());
        debug_assert!((id as usize) < self.m_head_indices.len());

        // SupportTreeBuilder.hpp:387
        self.m_meshcache_valid = false;
        // SupportTreeBuilder.hpp:388 — return m_heads[m_head_indices[id]];
        let idx = self.m_head_indices[id as usize];
        &mut self.m_heads[idx]
    }

    // SupportTreeBuilder.hpp:391-394
    // inline size_t pillarcount() const {
    //     std::lock_guard<Mutex> lk(m_mutex);
    //     return m_pillars.size();
    // }
    #[inline]
    pub fn pillarcount(&self) -> usize {
        let _lk = self.m_mutex.lock();
        self.m_pillars.len()
    }

    // SupportTreeBuilder.hpp:396
    // inline const std::vector<Pillar> &pillars() const { return m_pillars; }
    #[inline]
    pub fn pillars(&self) -> &Vec<Pillar> {
        &self.m_pillars
    }

    // SupportTreeBuilder.hpp:397
    // inline const std::vector<Head> &heads() const { return m_heads; }
    #[inline]
    pub fn heads(&self) -> &Vec<Head> {
        &self.m_heads
    }

    // SupportTreeBuilder.hpp:398
    // inline const std::vector<Bridge> &bridges() const { return m_bridges; }
    #[inline]
    pub fn bridges(&self) -> &Vec<Bridge> {
        &self.m_bridges
    }

    // SupportTreeBuilder.hpp:399
    // inline const std::vector<Bridge> &crossbridges() const { return m_crossbridges; }
    #[inline]
    pub fn crossbridges(&self) -> &Vec<Bridge> {
        &self.m_crossbridges
    }

    // SupportTreeBuilder.hpp:401-408
    // template<class T> inline IntegerOnly<T, const Pillar&> pillar(T id) const
    // (IntegerOnly<T> from MTUtils.hpp monomorphized to i64.)
    #[inline]
    pub fn pillar(&self, id: i64) -> &Pillar {
        // SupportTreeBuilder.hpp:403 — std::lock_guard<Mutex> lk(m_mutex);
        let _lk = self.m_mutex.lock();
        // SupportTreeBuilder.hpp:404-405
        // assert(id >= 0 && size_t(id) < m_pillars.size() &&
        //        size_t(id) < std::numeric_limits<size_t>::max());
        debug_assert!(
            id >= 0 && (id as usize) < self.m_pillars.len() && (id as u64) < u64::MAX
        );
        // SupportTreeBuilder.hpp:407 — return m_pillars[size_t(id)];
        &self.m_pillars[id as usize]
    }

    // SupportTreeBuilder.hpp:410-417
    // template<class T> inline IntegerOnly<T, Pillar&> pillar(T id)
    // (Mutable overload disambiguated as `pillar_mut`.)
    #[inline]
    pub fn pillar_mut(&mut self, id: i64) -> &mut Pillar {
        // SupportTreeBuilder.hpp:412 — std::lock_guard<Mutex> lk(m_mutex);
        let _lk = self.m_mutex.lock();
        // SupportTreeBuilder.hpp:413-414
        debug_assert!(
            id >= 0 && (id as usize) < self.m_pillars.len() && (id as u64) < u64::MAX
        );
        // SupportTreeBuilder.hpp:416 — return m_pillars[size_t(id)];
        &mut self.m_pillars[id as usize]
    }

    // SupportTreeBuilder.hpp:419
    // const Pad& pad() const { return m_pad; }
    pub fn pad(&self) -> &Pad {
        &self.m_pad
    }

    // WITHOUT THE PAD!!!
    // SupportTreeBuilder.hpp:421-422 (declaration, default steps = 45) /
    // SupportTreeBuilder.cpp:125-188
    // const indexed_triangle_set &SupportTreeBuilder::merged_mesh(size_t steps) const
    // (C++ `const` with `mutable` cache members; expressed with `&mut self`.
    //  The C++ default argument `steps = 45` cannot be expressed in Rust;
    //  callers pass it explicitly. The `get_mesh(anch, ...)` call resolves to
    //  `get_mesh(const Head&, ...)` since Anchor derives from Head with no
    //  dedicated overload, hence `get_mesh_head(&anch.0, ...)` here.)
    pub fn merged_mesh(&mut self, steps: usize) -> &indexed_triangle_set {
        // SupportTreeBuilder.cpp:127
        if self.m_meshcache_valid {
            return &self.m_meshcache;
        }

        // SupportTreeBuilder.cpp:129
        let mut merged = indexed_triangle_set::default();

        // SupportTreeBuilder.cpp:131-134
        for head in &self.m_heads {
            if (self.m_ctl.stopcondition)() {
                break;
            }
            if head.is_valid() {
                its_merge(&mut merged, &get_mesh_head(head, steps));
            }
        }

        // SupportTreeBuilder.cpp:136-139
        for pill in &self.m_pillars {
            if (self.m_ctl.stopcondition)() {
                break;
            }
            its_merge(&mut merged, &get_mesh_pillar(pill, steps));
        }

        // SupportTreeBuilder.cpp:141-144
        for pedest in &self.m_pedestals {
            if (self.m_ctl.stopcondition)() {
                break;
            }
            its_merge(&mut merged, &get_mesh_pedestal(pedest, steps));
        }

        // SupportTreeBuilder.cpp:146-149
        for j in &self.m_junctions {
            if (self.m_ctl.stopcondition)() {
                break;
            }
            its_merge(&mut merged, &get_mesh_junction(j, steps));
        }

        // SupportTreeBuilder.cpp:151-154
        for bs in &self.m_bridges {
            if (self.m_ctl.stopcondition)() {
                break;
            }
            its_merge(&mut merged, &get_mesh_bridge(bs, steps));
        }

        // SupportTreeBuilder.cpp:156-159
        for bs in &self.m_crossbridges {
            if (self.m_ctl.stopcondition)() {
                break;
            }
            its_merge(&mut merged, &get_mesh_bridge(bs, steps));
        }

        // SupportTreeBuilder.cpp:161-164
        for bs in &self.m_diffbridges {
            if (self.m_ctl.stopcondition)() {
                break;
            }
            its_merge(&mut merged, &get_mesh_diff_bridge(bs, steps));
        }

        // SupportTreeBuilder.cpp:166-169
        for anch in &self.m_anchors {
            if (self.m_ctl.stopcondition)() {
                break;
            }
            its_merge(&mut merged, &get_mesh_head(&anch.0, steps));
        }

        // SupportTreeBuilder.cpp:171-175
        if (self.m_ctl.stopcondition)() {
            // In case of failure we have to return an empty mesh
            self.m_meshcache = indexed_triangle_set::default();
            return &self.m_meshcache;
        }

        // SupportTreeBuilder.cpp:177
        self.m_meshcache = merged;

        // The mesh will be passed by const-pointer to TriangleMeshSlicer,
        // which will need this.
        // SupportTreeBuilder.cpp:181 — its_merge_vertices(m_meshcache);
        // (TriangleMesh.hpp default argument `shrink_to_fit = false`)
        its_merge_vertices(&mut self.m_meshcache, false);

        // SupportTreeBuilder.cpp:183-184
        // BoundingBoxf3 bb = bounding_box(m_meshcache);
        // m_model_height   = bb.max(Z) - bb.min(Z);
        // (C++ BoundingBoxf3 default-constructs min/max to Zero, so an empty
        //  mesh yields height 0; the crate's empty BoundingBox3F holds MAX/MIN
        //  sentinels, hence the is_defined() guard — same value.)
        let bb = bounding_box(&self.m_meshcache);
        self.m_model_height = if bb.is_defined() {
            bb.max.z - bb.min.z
        } else {
            0.0
        };

        // SupportTreeBuilder.cpp:186-187
        self.m_meshcache_valid = true;
        &self.m_meshcache
    }

    // WITH THE PAD
    // SupportTreeBuilder.hpp:424-425 (declaration) /
    // SupportTreeBuilder.cpp:190-198
    // double SupportTreeBuilder::full_height() const
    // {
    //     if (merged_mesh().indices.empty() && !pad().empty())
    //         return pad().cfg.full_height();
    //
    //     double h = mesh_height();
    //     if (!pad().empty()) h += pad().cfg.required_elevation();
    //     return h;
    // }
    //
    // BLOCKED: depends on the blocked merged_mesh()/mesh_height() above and
    // on `PadConfig::full_height()` / `PadConfig::required_elevation()` from
    // the still-stubbed SLA/Pad.cpp port (`sla/pad.rs`).

    // WITHOUT THE PAD!!!
    // SupportTreeBuilder.hpp:427-432
    // inline double mesh_height() const
    // {
    //     if (!m_meshcache_valid) merged_mesh();
    //     return m_model_height;
    // }
    //
    // BLOCKED: depends on the blocked merged_mesh() above.

    // Intended to be called after the generation is fully complete
    // SupportTreeBuilder.hpp:434-435 (declaration) /
    // SupportTreeBuilder.cpp:200-213
    // const indexed_triangle_set &SupportTreeBuilder::merge_and_cleanup()
    // {
    //     // in case the mesh is not generated, it should be...
    //     auto &ret = merged_mesh();
    //
    //     // Doing clear() does not garantee to release the memory.
    //     m_heads = {};
    //     m_head_indices = {};
    //     m_pillars = {};
    //     m_junctions = {};
    //     m_bridges = {};
    //
    //     return ret;
    // }
    //
    // BLOCKED: depends on the blocked merged_mesh() above.

    // Implement SupportTree interface:

    // SupportTreeBuilder.hpp:439-440 (declaration) /
    // SupportTreeBuilder.cpp:50-55
    // const indexed_triangle_set &SupportTreeBuilder::add_pad(
    //     const ExPolygons &modelbase, const PadConfig &cfg)
    // {
    //     m_pad = Pad{merged_mesh(), modelbase, ground_level, cfg, ctl().cancelfn};
    //     return m_pad.tmesh;
    // }
    //
    // BLOCKED: depends on the blocked merged_mesh() above and on the blocked
    // Pad constructor (which needs SLA/Pad.cpp).

    // SupportTreeBuilder.hpp:442
    // void remove_pad() override { m_pad = Pad(); }
    pub fn remove_pad(&mut self) {
        self.m_pad = Pad::default();
    }

    // SupportTreeBuilder.hpp:444-445 (declaration) /
    // SupportTreeBuilder.cpp:215-223
    // const indexed_triangle_set &SupportTreeBuilder::retrieve_mesh(MeshType meshtype) const
    // {
    //     switch(meshtype) {
    //     case MeshType::Support: return merged_mesh();
    //     case MeshType::Pad:     return pad().tmesh;
    //     }
    //
    //     return m_meshcache;
    // }
    //
    // BLOCKED: depends on the blocked merged_mesh() above.

    // SupportTree.hpp:169 (base class)
    // const JobController &ctl() const { return m_ctl; }
    // (Exposed as an inherent method; the `impl SupportTree for
    //  SupportTreeBuilder` trait impl is BLOCKED because its required
    //  `retrieve_mesh` / `add_pad` methods are blocked, see above.)
    pub fn ctl(&self) -> &JobController {
        &self.m_ctl
    }
}

// SupportTreeBuilder.cpp:225 — }} // namespace Slic3r::sla
