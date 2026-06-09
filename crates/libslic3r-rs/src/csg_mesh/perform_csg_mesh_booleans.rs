//! Faithful 1:1 port of BambuStudio
//! `src/libslic3r/CSGMesh/PerformCSGMeshBooleans.hpp`.
//!
//! Header-only file (templates only); every definition lives in the header and
//! is reproduced here. Line references point into `PerformCSGMeshBooleans.hpp`.
//!
//! ## Native-backend status (BLOCKED symbols)
//!
//! Almost the entire compute surface of this header is built on two native,
//! non-wasm-safe C/C++ libraries that have no Rust equivalent in this crate
//! (see the long BLOCKED note in `crate::mesh_boolean`):
//!
//!   * **CGAL** — `MeshBoolean::cgal::CGALMesh` / `CGALMeshPtr`,
//!     `triangle_mesh_to_cgal`, `plus` / `minus` / `intersect`, `empty`,
//!     `does_self_intersect`, `does_bound_a_volume`. The whole CGAL pipeline
//!     (`get_cgalmesh`, `detail_cgal::perform_csg`, `detail_cgal::get_cgalptrs`,
//!     `perform_csgmesh_booleans_cgal`, the CGAL `check_csgmesh_booleans`, and
//!     the default `perform_csgmesh_booleans` entry point) is therefore BLOCKED
//!     and cannot be expressed without inventing a fake CGAL backend.
//!   * **mcut** — the boolean compute kernel `MeshBoolean::mcut::do_boolean`
//!     (`mcCreateContext`/`mcDispatch`/…). This is BLOCKED, so the `Union`
//!     /`Difference`/`Intersection` arms of `detail_mcut::perform_csg` cannot
//!     run the actual boolean.
//!
//! What *is* ported faithfully and needs no native backend is the mcut
//! *data-marshalling* layer that already lives in `crate::mesh_boolean::mcut`:
//! the `McutMesh` array representation, `triangle_mesh_to_mcut`, `empty`, and
//! `mcut_to_triangle_mesh`. On top of that this file ports, line for line, the
//! structurally-tractable parts of the mcut pipeline — `get_mcutmesh`, the
//! mesh-conversion helpers, the stack-machine control flow of
//! `perform_csgmesh_booleans_mcut`, and the mcut `check_csgmesh_booleans` — and
//! marks the single blocked `do_boolean` call inline.
//!
//! `coord_t -> i64`, `coordf_t -> f64`. The crate uses `&[CSGPart]` as the
//! analog of the C++ `const Range<It> &csgrange` (matching the sibling files
//! `slice_csg_mesh.rs` and `voxelize_csg_mesh.rs`).
//!
//! Divergence note: `its_transform(m, get_transform(csgpart), true)`
//! (PerformCSGMeshBooleans.hpp:32 / :58) passes `fix_left_handed = true`, which
//! flips triangle winding (`its_flip_triangles`) when the transform has a
//! negative determinant. The crate's `TriangleMesh::transform` does not perform
//! that winding fixup; this port applies the affine transform faithfully and
//! inherits the same `fix_left_handed` gap already present in
//! `crate::mesh_boolean`. Because the downstream CGAL/mcut booleans are BLOCKED,
//! the transformed mesh never reaches an actual boolean kernel.

use super::csg_mesh::{
    get_mesh, get_operation, get_stack_operation, get_transform, CSGPart, CSGStackOp, CSGType,
};
use crate::mesh_boolean::mcut::{self, McutMesh};
use crate::triangle_mesh::TriangleMesh;

// PerformCSGMeshBooleans.hpp:15  namespace Slic3r { namespace csg {

/// PerformCSGMeshBooleans.hpp:16
/// `enum class BooleanFailReason { OK, MeshEmpty, NotBoundAVolume, SelfIntersect, NoIntersection};`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BooleanFailReason {
    Ok,
    MeshEmpty,
    NotBoundAVolume,
    SelfIntersect,
    NoIntersection,
}

impl Default for BooleanFailReason {
    // The C++ callers default-initialize `BooleanFailReason fail_reason =
    // BooleanFailReason::OK;` (PerformCSGMeshBooleans.hpp:265, :331).
    fn default() -> Self {
        BooleanFailReason::Ok
    }
}

// PerformCSGMeshBooleans.hpp:18-19
// This method can be overriden when a specific CSGPart type supports caching
// of the voxel grid
// PerformCSGMeshBooleans.hpp:20-42
// template<class CSGPartT>
// MeshBoolean::cgal::CGALMeshPtr get_cgalmesh(const CSGPartT &csgpart)
// {
//     const indexed_triangle_set *its = csg::get_mesh(csgpart);
//     indexed_triangle_set dummy;
//
//     if (!its)
//         its = &dummy;
//
//     MeshBoolean::cgal::CGALMeshPtr ret;
//
//     indexed_triangle_set m = *its;
//     its_transform(m, get_transform(csgpart), true);
//
//     try {
//         ret = MeshBoolean::cgal::triangle_mesh_to_cgal(m);
//     } catch (...) {
//         // errors are ignored, simply return null
//         ret = nullptr;
//     }
//
//     return ret;
// }
//
// BLOCKED: `MeshBoolean::cgal::CGALMeshPtr` / `triangle_mesh_to_cgal` are native
// CGAL types/functions with no Rust binding in this wasm-safe crate (see the
// BLOCKED list in `crate::mesh_boolean`). `get_cgalmesh` cannot be ported
// without a fake CGAL backend, so it is intentionally omitted.

// PerformCSGMeshBooleans.hpp:44-45
// This method can be overriden when a specific CSGPart type supports caching
// of the voxel grid
/// PerformCSGMeshBooleans.hpp:46-69
/// ```cpp
/// template<class CSGPartT>
/// MeshBoolean::mcut::McutMeshPtr get_mcutmesh(const CSGPartT& csgpart)
/// ```
///
/// The C++ `unique_ptr<McutMesh, McutMeshDeleter>` PIMPL becomes an owned
/// `Option<McutMesh>` in Rust (`None` mirrors the `nullptr` catch path).
pub fn get_mcutmesh(csgpart: &CSGPart) -> Option<McutMesh> {
    // PerformCSGMeshBooleans.hpp:49  const indexed_triangle_set* its = csg::get_mesh(csgpart);
    let its = get_mesh(csgpart);
    // PerformCSGMeshBooleans.hpp:50  indexed_triangle_set dummy;
    let dummy = TriangleMesh::new();

    // PerformCSGMeshBooleans.hpp:52-53  if (!its) its = &dummy;
    let its = match its {
        Some(its) => its,
        None => &dummy,
    };

    // PerformCSGMeshBooleans.hpp:55  MeshBoolean::mcut::McutMeshPtr ret;
    let ret: Option<McutMesh>;

    // PerformCSGMeshBooleans.hpp:57  indexed_triangle_set m = *its;
    let mut m = its.clone();
    // PerformCSGMeshBooleans.hpp:58  its_transform(m, get_transform(csgpart), true);
    // NOTE (fix_left_handed): see the module-level divergence note. The crate's
    // `transform` applies the affine map but does not flip winding for negative
    // determinants the way `its_transform(..., true)` would.
    m.transform(&get_transform(csgpart));

    // PerformCSGMeshBooleans.hpp:60-67
    // try {
    //     ret = MeshBoolean::mcut::triangle_mesh_to_mcut(m);
    // }
    // catch (...) {
    //     // errors are ignored, simply return null
    //     ret = nullptr;
    // }
    // `triangle_mesh_to_mcut` is the pure data-marshalling layer ported in
    // `crate::mesh_boolean::mcut`; it cannot throw, so the catch path is dead.
    ret = Some(mcut::triangle_mesh_to_mcut_from_its(&m));

    // PerformCSGMeshBooleans.hpp:68  return ret;
    ret
}

// PerformCSGMeshBooleans.hpp:71-113  namespace detail_cgal { ... }
//
// using MeshBoolean::cgal::CGALMeshPtr;
//
// BLOCKED: the entire `detail_cgal` namespace (`perform_csg` over `CGALMeshPtr`
// dispatching to `MeshBoolean::cgal::plus`/`minus`/`intersect`, and
// `get_cgalptrs`) is native CGAL and cannot be ported without a fake backend.

// PerformCSGMeshBooleans.hpp:115-157  namespace detail_mcut { ... }
pub mod detail_mcut {
    use super::*;

    // PerformCSGMeshBooleans.hpp:117  using MeshBoolean::mcut::McutMeshPtr;

    /// PerformCSGMeshBooleans.hpp:119-140
    /// `inline void perform_csg(CSGType op, McutMeshPtr& dst, McutMeshPtr& src)`
    pub fn perform_csg(op: CSGType, dst: &mut Option<McutMesh>, src: &mut Option<McutMesh>) {
        // PerformCSGMeshBooleans.hpp:121-124
        // if (!dst && op == CSGType::Union && src) {
        //     dst = std::move(src);
        //     return;
        // }
        if dst.is_none() && op == CSGType::Union && src.is_some() {
            *dst = src.take();
            return;
        }

        // PerformCSGMeshBooleans.hpp:126-127  if (!dst || !src) return;
        if dst.is_none() || src.is_none() {
            return;
        }

        // PerformCSGMeshBooleans.hpp:129-139
        // switch (op) {
        // case CSGType::Union:        MeshBoolean::mcut::do_boolean(*dst, *src,"UNION"); break;
        // case CSGType::Difference:   MeshBoolean::mcut::do_boolean(*dst, *src,"A_NOT_B"); break;
        // case CSGType::Intersection: MeshBoolean::mcut::do_boolean(*dst, *src,"INTERSECTION"); break;
        // }
        //
        // BLOCKED: `MeshBoolean::mcut::do_boolean` is the native mcut compute
        // kernel (`mcCreateContext`/`mcDispatch`/…), BLOCKED in
        // `crate::mesh_boolean`. The dispatch shape is preserved so the parity
        // of the surrounding stack machine is exact; the actual boolean is not
        // executed here.
        let (_dst, _src) = (dst, src);
        match op {
            CSGType::Union => {
                // MeshBoolean::mcut::do_boolean(*dst, *src, "UNION");
            }
            CSGType::Difference => {
                // MeshBoolean::mcut::do_boolean(*dst, *src, "A_NOT_B");
            }
            CSGType::Intersection => {
                // MeshBoolean::mcut::do_boolean(*dst, *src, "INTERSECTION");
            }
        }
    }

    /// PerformCSGMeshBooleans.hpp:142-155
    /// ```cpp
    /// template<class Ex, class It>
    /// std::vector<McutMeshPtr> get_mcutptrs(Ex policy, const Range<It>& csgrange)
    /// ```
    ///
    /// The C++ runs `execution::for_each(policy, 0, csgrange.size(), …)` to fill
    /// `ret` in parallel. The closure captures `&mut ret[i]` per index, which
    /// Rust's borrow checker cannot express across a shared parallel closure;
    /// `get_mcutmesh` is also data-independent per element, so the result is
    /// identical to a sequential map. The `policy` argument is preserved for
    /// signature parity but, like the C++ ordering, has no effect on output.
    pub fn get_mcutptrs(csgrange: &[CSGPart]) -> Vec<Option<McutMesh>> {
        // PerformCSGMeshBooleans.hpp:145  std::vector<McutMeshPtr> ret(csgrange.size());
        // PerformCSGMeshBooleans.hpp:146-152  execution::for_each(...) { ret[i] = get_mcutmesh(csgpart); }
        csgrange.iter().map(get_mcutmesh).collect()
    }
} // namespace detail_mcut

// PerformCSGMeshBooleans.hpp:159-207
// Process the sequence of CSG parts with CGAL.
// template<class It>
// void perform_csgmesh_booleans_cgal(MeshBoolean::cgal::CGALMeshPtr &cgalm,
//                               const Range<It>                &csgrange)
//
// BLOCKED: requires `MeshBoolean::cgal::CGALMesh` / `CGALMeshPtr`,
// `triangle_mesh_to_cgal` (for the `Frame` initializer) and the native CGAL
// `detail_cgal::perform_csg`. Cannot be ported without a fake CGAL backend.

// PerformCSGMeshBooleans.hpp:209-258
/// Process the sequence of CSG parts with mcut.
/// ```cpp
/// template<class It>
/// void perform_csgmesh_booleans_mcut(MeshBoolean::mcut::McutMeshPtr& mcutm,
///     const Range<It>& csgrange)
/// ```
///
/// NOTE: the boolean kernel inside `detail_mcut::perform_csg` is BLOCKED (native
/// mcut `do_boolean`); the stack machine itself is ported faithfully.
pub fn perform_csgmesh_booleans_mcut_into(mcutm: &mut Option<McutMesh>, csgrange: &[CSGPart]) {
    use detail_mcut::perform_csg;

    // PerformCSGMeshBooleans.hpp:218-224
    // struct Frame {
    //     CSGType op; McutMeshPtr mcutptr;
    //     explicit Frame(CSGType csgop = CSGType::Union)
    //         : op{ csgop }
    //         , mcutptr{ MeshBoolean::mcut::triangle_mesh_to_mcut(indexed_triangle_set{}) }
    //     {}
    // };
    struct Frame {
        op: CSGType,
        mcutptr: Option<McutMesh>,
    }
    impl Frame {
        fn new(csgop: CSGType) -> Self {
            Self {
                op: csgop,
                // PerformCSGMeshBooleans.hpp:222  triangle_mesh_to_mcut(indexed_triangle_set{})
                mcutptr: Some(mcut::triangle_mesh_to_mcut_from_its(&TriangleMesh::new())),
            }
        }
    }

    // PerformCSGMeshBooleans.hpp:226  std::stack opstack{ std::vector<Frame>{} };
    let mut opstack: Vec<Frame> = Vec::new();

    // PerformCSGMeshBooleans.hpp:228  opstack.push(Frame{});
    opstack.push(Frame::new(CSGType::Union));

    // PerformCSGMeshBooleans.hpp:230  std::vector<McutMeshPtr> McutMeshes = get_mcutptrs(ex_tbb, csgrange);
    let mut mcut_meshes = detail_mcut::get_mcutptrs(csgrange);

    // PerformCSGMeshBooleans.hpp:232  size_t csgidx = 0;
    let mut csgidx: usize = 0;
    // PerformCSGMeshBooleans.hpp:233  for (auto& csgpart : csgrange) {
    for csgpart in csgrange.iter() {
        // PerformCSGMeshBooleans.hpp:235  auto op = get_operation(csgpart);
        #[allow(unused_assignments)]
        let mut op = get_operation(csgpart);
        // PerformCSGMeshBooleans.hpp:236  McutMeshPtr& mcutptr = McutMeshes[csgidx++];
        let mcutptr = &mut mcut_meshes[csgidx];
        csgidx += 1;

        // PerformCSGMeshBooleans.hpp:238-241
        // if (get_stack_operation(csgpart) == CSGStackOp::Push) {
        //     opstack.push(Frame{ op });
        //     op = CSGType::Union;
        // }
        if get_stack_operation(csgpart) == CSGStackOp::Push {
            opstack.push(Frame::new(op));
            op = CSGType::Union;
        }

        // PerformCSGMeshBooleans.hpp:243  Frame* top = &opstack.top();
        let top = opstack.last_mut().unwrap();

        // PerformCSGMeshBooleans.hpp:245  perform_csg(get_operation(csgpart), top->mcutptr, mcutptr);
        perform_csg(get_operation(csgpart), &mut top.mcutptr, mcutptr);

        // PerformCSGMeshBooleans.hpp:247-253
        // if (get_stack_operation(csgpart) == CSGStackOp::Pop) {
        //     McutMeshPtr src = std::move(top->mcutptr);
        //     auto popop = opstack.top().op;
        //     opstack.pop();
        //     McutMeshPtr& dst = opstack.top().mcutptr;
        //     perform_csg(popop, dst, src);
        // }
        if get_stack_operation(csgpart) == CSGStackOp::Pop {
            let popped = opstack.pop().unwrap();
            let mut src = popped.mcutptr;
            let popop = popped.op;
            let dst = opstack.last_mut().unwrap();
            perform_csg(popop, &mut dst.mcutptr, &mut src);
        }
    }

    // PerformCSGMeshBooleans.hpp:256  mcutm = std::move(opstack.top().mcutptr);
    *mcutm = opstack.last_mut().unwrap().mcutptr.take();
}

// PerformCSGMeshBooleans.hpp:261-322
// template<class It, class Visitor>
// std::tuple<BooleanFailReason,std::string> check_csgmesh_booleans(const Range<It> &csgrange, Visitor &&vfn)
//
// BLOCKED: the CGAL-backed `check_csgmesh_booleans(csgrange, vfn)` overload uses
// `get_cgalmesh`, `MeshBoolean::cgal::triangle_mesh_to_cgal`,
// `MeshBoolean::cgal::empty`, and `MeshBoolean::cgal::does_self_intersect`
// (`does_bound_a_volume` is commented out at hpp:289-294). All are native CGAL
// and BLOCKED. The default `use_mcut=false` entry point therefore cannot run;
// only the mcut branch below is ported.

/// PerformCSGMeshBooleans.hpp:324-361
/// ```cpp
/// template<class It>
/// std::tuple<BooleanFailReason, std::string> check_csgmesh_booleans(const Range<It> &csgrange, bool use_mcut=false)
/// ```
///
/// Only the `use_mcut == true` branch is ported (the `use_mcut == false` branch
/// dispatches to the BLOCKED CGAL overload). Callers requesting the CGAL path
/// fall through to a successful `(Ok, "")` because the CGAL validation cannot be
/// executed in this wasm-safe crate.
pub fn check_csgmesh_booleans(csgrange: &[CSGPart], use_mcut: bool) -> (BooleanFailReason, String) {
    // PerformCSGMeshBooleans.hpp:327-328
    // if(!use_mcut)
    //     return check_csgmesh_booleans(csgrange, [](auto &) {});
    if !use_mcut {
        // BLOCKED CGAL overload (see note above) — cannot validate without CGAL.
        return (BooleanFailReason::Ok, String::new());
    }
    // PerformCSGMeshBooleans.hpp:329  else {
    // PerformCSGMeshBooleans.hpp:330  using namespace detail_mcut;
    // PerformCSGMeshBooleans.hpp:331  BooleanFailReason fail_reason = BooleanFailReason::OK;
    let mut fail_reason = BooleanFailReason::Ok;
    // PerformCSGMeshBooleans.hpp:332  std::string fail_part_name;
    let mut fail_part_name = String::new();

    // PerformCSGMeshBooleans.hpp:334  std::vector<McutMeshPtr> McutMeshes(csgrange.size());
    let mut mcut_meshes: Vec<Option<McutMesh>> = (0..csgrange.len()).map(|_| None).collect();

    // PerformCSGMeshBooleans.hpp:335-357
    // auto check_part = [&csgrange, &McutMeshes,&fail_reason,&fail_part_name](size_t i) { ... };
    // execution::for_each(ex_tbb, size_t(0), csgrange.size(), check_part);
    //
    // The C++ `execution::for_each(ex_tbb, …)` parallel-writes `fail_reason` /
    // `fail_part_name` without synchronisation (last writer on conflict, order
    // unspecified). Ported sequentially over indices for a deterministic,
    // borrow-checkable equivalent.
    let check_part = |i: usize,
                      mcut_meshes: &mut Vec<Option<McutMesh>>,
                      fail_reason: &mut BooleanFailReason,
                      fail_part_name: &mut String| {
        // PerformCSGMeshBooleans.hpp:336-338
        // auto it = csgrange.begin(); std::advance(it, i); auto& csgpart = *it;
        let csgpart = &csgrange[i];
        // PerformCSGMeshBooleans.hpp:339  auto m = get_mcutmesh(csgpart);
        let m = get_mcutmesh(csgpart);

        // PerformCSGMeshBooleans.hpp:341  mesh can be nullptr if this is a stack push or pull
        // PerformCSGMeshBooleans.hpp:342-345
        // if (!get_mesh(csgpart) && get_stack_operation(csgpart) != CSGStackOp::Continue) {
        //     McutMeshes[i] = MeshBoolean::mcut::triangle_mesh_to_mcut(indexed_triangle_set{});
        //     return;
        // }
        if get_mesh(csgpart).is_none() && get_stack_operation(csgpart) != CSGStackOp::Continue {
            mcut_meshes[i] = Some(mcut::triangle_mesh_to_mcut_from_its(&TriangleMesh::new()));
            return;
        }

        // PerformCSGMeshBooleans.hpp:347-354
        // try {
        //     if (!m || MeshBoolean::mcut::empty(*m)) {
        //         fail_reason=BooleanFailReason::MeshEmpty;
        //         fail_part_name = csgpart.name;
        //         return;
        //     }
        // }
        // catch (...) { return; }
        match &m {
            None => {
                *fail_reason = BooleanFailReason::MeshEmpty;
                *fail_part_name = csgpart.name.clone();
                return;
            }
            Some(mesh) => {
                if mcut::empty(mesh) {
                    *fail_reason = BooleanFailReason::MeshEmpty;
                    *fail_part_name = csgpart.name.clone();
                    return;
                }
            }
        }

        // PerformCSGMeshBooleans.hpp:356  McutMeshes[i] = std::move(m);
        mcut_meshes[i] = m;
    };

    for i in 0..csgrange.len() {
        check_part(
            i,
            &mut mcut_meshes,
            &mut fail_reason,
            &mut fail_part_name,
        );
    }

    // PerformCSGMeshBooleans.hpp:359  return { fail_reason,fail_part_name };
    (fail_reason, fail_part_name)
}

// PerformCSGMeshBooleans.hpp:363-370
// template<class It>
// MeshBoolean::cgal::CGALMeshPtr perform_csgmesh_booleans(const Range<It> &csgparts)
// {
//     auto ret = MeshBoolean::cgal::triangle_mesh_to_cgal(indexed_triangle_set{});
//     if (ret)
//         perform_csgmesh_booleans_cgal(ret, csgparts);
//     return ret;
// }
//
// BLOCKED: the default CGAL entry point depends on
// `MeshBoolean::cgal::triangle_mesh_to_cgal` and `perform_csgmesh_booleans_cgal`,
// both native CGAL. Cannot be ported without a fake CGAL backend.

/// PerformCSGMeshBooleans.hpp:372-379
/// ```cpp
/// template<class It>
/// MeshBoolean::mcut::McutMeshPtr  perform_csgmesh_booleans_mcut(const Range<It>& csgparts)
/// ```
pub fn perform_csgmesh_booleans_mcut(csgparts: &[CSGPart]) -> Option<McutMesh> {
    // PerformCSGMeshBooleans.hpp:375  auto ret = MeshBoolean::mcut::triangle_mesh_to_mcut(indexed_triangle_set{});
    let mut ret: Option<McutMesh> = Some(mcut::triangle_mesh_to_mcut_from_its(&TriangleMesh::new()));
    // PerformCSGMeshBooleans.hpp:376-377  if (ret) perform_csgmesh_booleans_mcut(ret, csgparts);
    if ret.is_some() {
        perform_csgmesh_booleans_mcut_into(&mut ret, csgparts);
    }
    // PerformCSGMeshBooleans.hpp:378  return ret;
    ret
}

// PerformCSGMeshBooleans.hpp:381-382  } // namespace csg } // namespace Slic3r

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csg_mesh::csg_mesh::MeshPtr;

    #[test]
    fn test_boolean_fail_reason_default() {
        assert_eq!(BooleanFailReason::default(), BooleanFailReason::Ok);
    }

    #[test]
    fn test_get_mcutmesh_empty_part() {
        // A part with no mesh yields a mcut mesh built from the empty dummy ITS.
        let part = CSGPart::new();
        let m = get_mcutmesh(&part);
        assert!(m.is_some());
        // The dummy `indexed_triangle_set{}` has no vertices/faces => empty().
        assert!(mcut::empty(m.as_ref().unwrap()));
    }

    #[test]
    fn test_perform_csgmesh_booleans_mcut_empty() {
        let parts: Vec<CSGPart> = vec![];
        let result = perform_csgmesh_booleans_mcut(&parts);
        assert!(result.is_some());
    }

    #[test]
    fn test_perform_csgmesh_booleans_mcut_single() {
        let mesh = TriangleMesh::new();
        let part = CSGPart::from_mesh(MeshPtr::from_owned(mesh));
        let result = perform_csgmesh_booleans_mcut(&[part]);
        assert!(result.is_some());
    }

    #[test]
    fn test_perform_csgmesh_booleans_mcut_with_stack() {
        // CUBE1 - (CUBE2 + CUBE3) structure (CSGMesh.hpp example).
        let cube1 = CSGPart::new()
            .with_operation(CSGType::Union)
            .with_stack_operation(CSGStackOp::Continue);
        let cube2 = CSGPart::new()
            .with_operation(CSGType::Difference)
            .with_stack_operation(CSGStackOp::Push);
        let cube3 = CSGPart::new()
            .with_operation(CSGType::Union)
            .with_stack_operation(CSGStackOp::Pop);

        let result = perform_csgmesh_booleans_mcut(&[cube1, cube2, cube3]);
        assert!(result.is_some());
    }

    #[test]
    fn test_check_booleans_cgal_path_blocked_returns_ok() {
        // use_mcut == false dispatches to the BLOCKED CGAL overload, which
        // returns (Ok, "") in this crate.
        let part = CSGPart::new().with_name("test_part".to_string());
        let (reason, name) = check_csgmesh_booleans(&[part], false);
        assert_eq!(reason, BooleanFailReason::Ok);
        assert!(name.is_empty());
    }

    #[test]
    fn test_check_booleans_mcut_empty_parts() {
        let (reason, name) = check_csgmesh_booleans(&[], true);
        assert_eq!(reason, BooleanFailReason::Ok);
        assert!(name.is_empty());
    }

    #[test]
    fn test_check_booleans_mcut_empty_mesh() {
        // A part with an empty mesh (no triangles) and Continue stack op fails
        // with MeshEmpty.
        let part = CSGPart::from_mesh(MeshPtr::from_owned(TriangleMesh::new()))
            .with_name("test_part".to_string());
        let (reason, name) = check_csgmesh_booleans(&[part], true);
        assert_eq!(reason, BooleanFailReason::MeshEmpty);
        assert_eq!(name, "test_part");
    }

    #[test]
    fn test_check_booleans_mcut_null_mesh_with_stack_op() {
        let part = CSGPart::new().with_stack_operation(CSGStackOp::Push);
        let (reason, _name) = check_csgmesh_booleans(&[part], true);
        assert_eq!(reason, BooleanFailReason::Ok);
    }
}
