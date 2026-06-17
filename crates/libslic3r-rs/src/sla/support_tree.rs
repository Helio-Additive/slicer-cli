//! Faithful port of SLA/SupportTree.{hpp,cpp}.
//!
//! C++ Reference:
//! - SLA/SupportTree.hpp
//! - SLA/SupportTree.cpp
//!
//! SupportTree.cpp:1-4
//! /**
//!  * In this file we will implement the automatic SLA support tree generation.
//!  *
//!  */
//!
//! C++ includes (SupportTree.cpp:6-20): SpatIndex, SupportTreeBuilder,
//! SupportTreeBuildsteps, MTUtils, ClipperUtils, Model, TriangleMeshSlicer,
//! libnest2d genetic/subplex optimizers, boost::log, I18N. Of these only
//! TriangleMeshSlicer (`slice_mesh_ex`), TriangleMesh (`its_merge`,
//! `bounding_box`) and the SLA support headers are actually used by the code
//! in this translation unit.
//!
//! SupportTree.cpp:22-24 — the `L(s)` localization macro
//! (`Slic3r::I18N::translate`) is defined but unused in this file.

use crate::geometry::ExPolygons;
use crate::normal_utils::indexed_triangle_set;
use crate::sla::indexed_mesh::IndexedMesh;
use crate::sla::job_controller::JobController;
use crate::sla::pad::PadConfig;
use crate::sla::support_point::SupportPoints;
use crate::triangle_mesh::{bounding_box, its_merge};
use crate::triangle_mesh_slicer::{slice_mesh_ex_its, MeshSlicingParamsEx};

// ---------------------------------------------------------------------------
// SupportTree.hpp
// ---------------------------------------------------------------------------

// SupportTree.hpp:27-32
// enum class PillarConnectionMode { zigzag, cross, dynamic };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PillarConnectionMode {
    Zigzag,  // SupportTree.hpp:29
    Cross,   // SupportTree.hpp:30
    Dynamic, // SupportTree.hpp:31
}

// SupportTree.hpp:34
#[derive(Debug, Clone)]
pub struct SupportTreeConfig {
    // SupportTree.hpp:36
    pub enabled: bool,

    // Radius in mm of the pointing side of the head.
    // SupportTree.hpp:39
    pub head_front_radius_mm: f64,

    // How much the pinhead has to penetrate the model surface
    // SupportTree.hpp:42
    pub head_penetration_mm: f64,

    // Radius of the back side of the 3d arrow.
    // SupportTree.hpp:45
    pub head_back_radius_mm: f64,

    // SupportTree.hpp:47
    pub head_fallback_radius_mm: f64,

    // Width in mm from the back sphere center to the front sphere center.
    // SupportTree.hpp:50
    pub head_width_mm: f64,

    // How to connect pillars
    // SupportTree.hpp:53
    pub pillar_connection_mode: PillarConnectionMode,

    // Only generate pillars that can be routed to ground
    // SupportTree.hpp:56
    pub ground_facing_only: bool,

    // TODO: unimplemented at the moment. This coefficient will have an impact
    // when bridges and pillars are merged. The resulting pillar should be a bit
    // thicker than the ones merging into it. How much thicker? I don't know
    // but it will be derived from this value.
    // SupportTree.hpp:62
    pub pillar_widening_factor: f64,

    // Radius in mm of the pillar base.
    // SupportTree.hpp:65
    pub base_radius_mm: f64,

    // The height of the pillar base cone in mm.
    // SupportTree.hpp:68
    pub base_height_mm: f64,

    // The default angle for connecting support sticks and junctions.
    // SupportTree.hpp:71
    pub bridge_slope: f64,

    // The max length of a bridge in mm
    // SupportTree.hpp:74
    pub max_bridge_length_mm: f64,

    // The max distance of a pillar to pillar link.
    // SupportTree.hpp:77
    pub max_pillar_link_distance_mm: f64,

    // The elevation in Z direction upwards. This is the space between the pad
    // and the model object's bounding box bottom.
    // SupportTree.hpp:81
    pub object_elevation_mm: f64,

    // The shortest distance between a pillar base perimeter from the model
    // body. This is only useful when elevation is set to zero.
    // SupportTree.hpp:85
    pub pillar_base_safety_distance_mm: f64,

    // SupportTree.hpp:87
    pub max_bridges_on_pillar: u32,
}

// SupportTree.hpp:36-87 — default member initializers.
impl Default for SupportTreeConfig {
    fn default() -> Self {
        Self {
            // SupportTree.hpp:36 — bool enabled = true;
            enabled: true,
            // SupportTree.hpp:39 — double head_front_radius_mm = 0.2;
            head_front_radius_mm: 0.2,
            // SupportTree.hpp:42 — double head_penetration_mm = 0.5;
            head_penetration_mm: 0.5,
            // SupportTree.hpp:45 — double head_back_radius_mm = 0.5;
            head_back_radius_mm: 0.5,
            // SupportTree.hpp:47 — double head_fallback_radius_mm = 0.25;
            head_fallback_radius_mm: 0.25,
            // SupportTree.hpp:50 — double head_width_mm = 1.0;
            head_width_mm: 1.0,
            // SupportTree.hpp:53 — PillarConnectionMode::dynamic
            pillar_connection_mode: PillarConnectionMode::Dynamic,
            // SupportTree.hpp:56 — bool ground_facing_only = false;
            ground_facing_only: false,
            // SupportTree.hpp:62 — double pillar_widening_factor = 0.5;
            pillar_widening_factor: 0.5,
            // SupportTree.hpp:65 — double base_radius_mm = 2.0;
            base_radius_mm: 2.0,
            // SupportTree.hpp:68 — double base_height_mm = 1.0;
            base_height_mm: 1.0,
            // SupportTree.hpp:71 — double bridge_slope = M_PI/4;
            bridge_slope: std::f64::consts::PI / 4.0,
            // SupportTree.hpp:74 — double max_bridge_length_mm = 10.0;
            max_bridge_length_mm: 10.0,
            // SupportTree.hpp:77 — double max_pillar_link_distance_mm = 10.0;
            max_pillar_link_distance_mm: 10.0,
            // SupportTree.hpp:81 — double object_elevation_mm = 10;
            object_elevation_mm: 10.0,
            // SupportTree.hpp:85 — double pillar_base_safety_distance_mm = 0.5;
            pillar_base_safety_distance_mm: 0.5,
            // SupportTree.hpp:87 — unsigned max_bridges_on_pillar = 3;
            max_bridges_on_pillar: 3,
        }
    }
}

impl SupportTreeConfig {
    // SupportTree.hpp:89-92
    // double head_fullwidth() const {
    //     return 2 * head_front_radius_mm + head_width_mm +
    //            2 * head_back_radius_mm - head_penetration_mm;
    // }
    pub fn head_fullwidth(&self) -> f64 {
        2.0 * self.head_front_radius_mm
            + self.head_width_mm
            + 2.0 * self.head_back_radius_mm
            - self.head_penetration_mm
    }

    // /////////////////////////////////////////////////////////////////////////
    // Compile time configuration values (candidates for runtime)
    // SupportTree.hpp:94-96
    // /////////////////////////////////////////////////////////////////////////

    // The max Z angle for a normal at which it will get completely ignored.
    // SupportTree.hpp:99 — 150.0 * M_PI / 180.0
    pub const NORMAL_CUTOFF_ANGLE: f64 = 150.0 * std::f64::consts::PI / 180.0;

    // The shortest distance of any support structure from the model surface
    // SupportTree.hpp:102
    pub const SAFETY_DISTANCE_MM: f64 = 0.5;

    // SupportTree.hpp:104
    pub const MAX_SOLO_PILLAR_HEIGHT_MM: f64 = 15.0;
    // SupportTree.hpp:105
    pub const MAX_DUAL_PILLAR_HEIGHT_MM: f64 = 35.0;
    // SupportTree.hpp:106
    pub const OPTIMIZER_REL_SCORE_DIFF: f64 = 1e-6;
    // SupportTree.hpp:107
    pub const OPTIMIZER_MAX_ITERATIONS: u32 = 1000;
    // SupportTree.hpp:108
    pub const PILLAR_CASCADE_NEIGHBORS: u32 = 3;
}

// SupportTree.hpp:112-116
// TODO: Part of future refactor
//class SupportConfig {
//    std::optional<SupportTreeConfig> tree_cfg {std::in_place_t{}}; // fill up
//    std::optional<PadConfig>         pad_cfg;
//};

// SupportTree.hpp:118
// enum class MeshType { Support, Pad };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MeshType {
    Support,
    Pad,
}

// SupportTree.hpp:120-138
pub struct SupportableMesh {
    // SupportTree.hpp:122
    pub emesh: IndexedMesh,
    // SupportTree.hpp:123
    pub pts: SupportPoints,
    // SupportTree.hpp:124
    pub cfg: SupportTreeConfig,
    // SupportTree.hpp:125 — //    PadConfig     pad_cfg;
}

impl SupportableMesh {
    // SupportTree.hpp:127-131
    // explicit SupportableMesh(const indexed_triangle_set &trmsh,
    //                          const SupportPoints &sp,
    //                          const SupportTreeConfig &c)
    //     : emesh{trmsh}, pts{sp}, cfg{c}
    //
    // The `emesh{trmsh}` member-initializer invokes
    // `IndexedMesh(const indexed_triangle_set&, calculate_epsilon = false)`
    // (IndexedMesh.hpp:51). That constructor is now ported as
    // `IndexedMesh::new(tmesh, calculate_epsilon)` (sla/indexed_mesh.rs:417),
    // so the default `calculate_epsilon = false` is supplied explicitly here.
    pub fn new(trmsh: &indexed_triangle_set, sp: &SupportPoints, c: &SupportTreeConfig) -> Self {
        Self {
            // SupportTree.hpp:130 — emesh{trmsh}
            emesh: IndexedMesh::new(trmsh, false),
            // SupportTree.hpp:130 — pts{sp}
            pts: sp.clone(),
            // SupportTree.hpp:130 — cfg{c}
            cfg: c.clone(),
        }
    }

    // SupportTree.hpp:133-137
    // explicit SupportableMesh(const IndexedMesh   &em,
    //                          const SupportPoints &sp,
    //                          const SupportTreeConfig &c)
    //     : emesh{em}, pts{sp}, cfg{c}
    pub fn from_emesh(em: &IndexedMesh, sp: &SupportPoints, c: &SupportTreeConfig) -> Self {
        Self {
            emesh: em.clone(),
            pts: sp.clone(),
            cfg: c.clone(),
        }
    }
}

/// The class containing mesh data for the generated supports.
// SupportTree.hpp:140-170
//
// C++ `class SupportTree` is abstract (pure virtual `retrieve_mesh`,
// `add_pad`, `remove_pad`); it maps to a Rust trait. The private member
// `JobController m_ctl` (SupportTree.hpp:143) cannot live on a trait, so each
// implementor (SupportTreeBuilder) stores it and exposes it through the
// `ctl()` accessor (SupportTree.hpp:169), which therefore becomes a required
// trait method instead of a concrete getter.
pub trait SupportTree {
    // SupportTree.hpp:147-148
    // static UPtr create(const SupportableMesh &input, const JobController &ctl = {});
    // -> defined as `SupportTree::create` in SupportTree.cpp:80-96; see the
    //    BLOCKED note at the end of this file.

    // SupportTree.hpp:150 — virtual ~SupportTree() = default; (implicit in Rust)

    // SupportTree.hpp:152
    // virtual const indexed_triangle_set &retrieve_mesh(MeshType meshtype) const = 0;
    fn retrieve_mesh(&self, meshtype: MeshType) -> &indexed_triangle_set;

    /// Adding the "pad" under the supports.
    /// modelbase will be used according to the embed_object flag in PoolConfig.
    /// If set, the plate will be interpreted as the model's intrinsic pad.
    /// Otherwise, the modelbase will be unified with the base plate calculated
    /// from the supports.
    // SupportTree.hpp:154-160
    // virtual const indexed_triangle_set &add_pad(const ExPolygons &modelbase,
    //                                             const PadConfig & pcfg) = 0;
    fn add_pad(&mut self, modelbase: &ExPolygons, pcfg: &PadConfig) -> &indexed_triangle_set;

    // SupportTree.hpp:162
    // virtual void remove_pad() = 0;
    fn remove_pad(&mut self);

    // SupportTree.hpp:164-165
    // std::vector<ExPolygons> slice(const std::vector<float> &, float closing_radius) const;
    //
    // Defined in SupportTree.cpp:34-78.
    fn slice(&self, grid: &[f32], cr: f32) -> Vec<ExPolygons> {
        // SupportTree.cpp:37
        let sup_mesh: &indexed_triangle_set = self.retrieve_mesh(MeshType::Support);
        // SupportTree.cpp:38
        let pad_mesh: &indexed_triangle_set = self.retrieve_mesh(MeshType::Pad);

        // SupportTree.cpp:40 — using Slices = std::vector<ExPolygons>;
        // SupportTree.cpp:41 — auto slices = reserve_vector<Slices>(2);
        let mut slices: Vec<Vec<ExPolygons>> = Vec::with_capacity(2);

        // SupportTree.cpp:43 — if (!sup_mesh.empty())
        // (admesh stl.h:247 — empty() == indices.empty() || vertices.empty())
        if !(sup_mesh.indices.is_empty() || sup_mesh.vertices.is_empty()) {
            // SupportTree.cpp:44
            slices.push(Vec::new());
            // SupportTree.cpp:45
            // slices.back() = slice_mesh_ex(sup_mesh, grid, cr, ctl().cancelfn);
            // TriangleMeshSlicer.hpp:86-95 — the closing_radius overload builds
            // MeshSlicingParamsEx{} with params.closing_radius = cr.
            let mut params = MeshSlicingParamsEx::default();
            params.closing_radius = cr;
            *slices.last_mut().unwrap() =
                slice_mesh_ex_its(sup_mesh, grid, &params, self.ctl().cancelfn.as_ref());
        }

        // SupportTree.cpp:48 — if (!pad_mesh.empty())
        if !(pad_mesh.indices.is_empty() || pad_mesh.vertices.is_empty()) {
            // SupportTree.cpp:49
            slices.push(Vec::new());

            // SupportTree.cpp:51
            let bb = bounding_box(pad_mesh);
            // SupportTree.cpp:52
            // auto maxzit = std::upper_bound(grid.begin(), grid.end(), bb.max.z());
            // (the float grid element is promoted to double for the comparison)
            let maxzit = grid.partition_point(|&z| !(bb.max.z() < z as f64));

            // SupportTree.cpp:54 — auto cap = grid.end() - maxzit;
            let cap = grid.len() as isize - maxzit as isize;
            // SupportTree.cpp:55
            // auto padgrid = reserve_vector<float>(size_t(cap > 0 ? cap : 0));
            let mut padgrid: Vec<f32> = Vec::with_capacity(if cap > 0 { cap as usize } else { 0 });
            // SupportTree.cpp:56
            // std::copy(grid.begin(), maxzit, std::back_inserter(padgrid));
            padgrid.extend_from_slice(&grid[..maxzit]);

            // SupportTree.cpp:58
            // slices.back() = slice_mesh_ex(pad_mesh, padgrid, cr, ctl().cancelfn);
            let mut params = MeshSlicingParamsEx::default();
            params.closing_radius = cr;
            *slices.last_mut().unwrap() =
                slice_mesh_ex_its(pad_mesh, &padgrid, &params, self.ctl().cancelfn.as_ref());
        }

        // SupportTree.cpp:61
        let mut len = grid.len();
        // SupportTree.cpp:62
        for slv in &slices {
            len = std::cmp::min(len, slv.len());
        }

        // Either the support or the pad or both has to be non empty
        // SupportTree.cpp:65
        if slices.is_empty() {
            return Vec::new();
        }

        // SupportTree.cpp:67 — Slices &mrg = slices.front();
        let (mrg, rest) = slices.split_first_mut().unwrap();

        // SupportTree.cpp:69
        for slv in rest {
            // SupportTree.cpp:70
            for i in 0..len {
                // SupportTree.cpp:72
                // std::copy(slv[i].begin(), slv[i].end(), std::back_inserter(mrg[i]));
                // SupportTree.cpp:73 — slv[i] = {}; // clear and delete
                // (append moves the elements out and leaves slv[i] empty —
                //  the same net effect as copy-then-clear.)
                mrg[i].append(&mut slv[i]);
            }
        }

        // SupportTree.cpp:77 — return mrg;
        slices.swap_remove(0)
    }

    // SupportTree.hpp:167
    // void retrieve_full_mesh(indexed_triangle_set &outmesh) const;
    //
    // Defined in SupportTree.cpp:29-32.
    fn retrieve_full_mesh(&self, outmesh: &mut indexed_triangle_set) {
        // SupportTree.cpp:30
        its_merge(outmesh, self.retrieve_mesh(MeshType::Support));
        // SupportTree.cpp:31
        its_merge(outmesh, self.retrieve_mesh(MeshType::Pad));
    }

    // SupportTree.hpp:169
    // const JobController &ctl() const { return m_ctl; }
    fn ctl(&self) -> &JobController;
}

// SupportTree.hpp:145 — using UPtr = std::unique_ptr<SupportTree>;
// (C++ nests this inside the class; Rust traits cannot hold type aliases for
// trait objects of themselves, so it lives at module level.)
pub type UPtr = Box<dyn SupportTree>;

// SupportTree.cpp:80-96
// SupportTree::UPtr SupportTree::create(const SupportableMesh &sm,
//                                       const JobController &  ctl)
// {
//     auto builder = make_unique<SupportTreeBuilder>();
//     builder->m_ctl = ctl;
//
//     if (sm.cfg.enabled) {
//         // Execute takes care about the ground_level
//         SupportTreeBuildsteps::execute(*builder, sm);
//         builder->merge_and_cleanup();   // clean metadata, leave only the meshes.
//     } else {
//         // If a pad gets added later, it will be in the right Z level
//         builder->ground_level = sm.emesh.ground_level();
//     }
//
//     return std::move(builder);
// }
//
// BLOCKED: `SupportTreeBuilder` (with `m_ctl`, `ground_level`),
// `SupportTreeBuildsteps::execute` and `IndexedMesh::ground_level()` are all
// now ported. The remaining blocker is that `SupportTreeBuilder` does NOT yet
// `impl SupportTree` — its required trait methods `merge_and_cleanup()`,
// `add_pad()` and `retrieve_mesh()` are still commented-out BLOCKED bodies in
// SLA/support_tree_builder.rs (they need the Pad constructor / merged_mesh
// plumbing wired through the trait). Until `impl SupportTree for
// SupportTreeBuilder` exists, there is no faithful way to produce the
// `UPtr = Box<dyn SupportTree>` return value, so no fake is provided. Once the
// builder implements the trait, this becomes:
//     pub fn create(sm: &SupportableMesh, ctl: JobController) -> UPtr {
//         let mut builder = Box::new(SupportTreeBuilder::default());
//         builder.m_ctl = ctl;
//         if sm.cfg.enabled {
//             SupportTreeBuildsteps::execute(&mut builder, sm);
//             builder.merge_and_cleanup();
//         } else {
//             builder.ground_level = sm.emesh.ground_level();
//         }
//         builder
//     }
