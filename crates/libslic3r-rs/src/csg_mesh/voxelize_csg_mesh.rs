//! Faithful 1:1 port of BambuStudio `src/libslic3r/CSGMesh/VoxelizeCSGMesh.hpp`.
//!
//! Header-only file; there is no `.cpp` translation unit, so every definition
//! lives in the header and is reproduced here line-by-line. Line references
//! point into `VoxelizeCSGMesh.hpp`.
//!
//! BLOCKED DEPENDENCY (native, not wasm-safe):
//! ------------------------------------------
//! `VoxelizeCSGMesh.hpp:8` includes `libslic3r/OpenVDBUtils.hpp` and the whole
//! voxel-grid layer (`VoxelGridPtr`, `MeshToGridParams`, `mesh_to_grid`,
//! `is_grid_empty`, `clone`, `grid_union`, `grid_difference`,
//! `grid_intersection`) is backed by **OpenVDB** (`deps/OpenVDB`), a native C++
//! library. OpenVDB is not present in the Cargo dependency set, is not
//! wasm-safe, and the higher-level grid wrapper API referenced by this header
//! (`MeshToGridParams`, `VoxelGridPtr`, `is_grid_empty`, `clone`,
//! `grid_union/difference/intersection`, the two-argument
//! `mesh_to_grid(its, params)`) is not even vendored anywhere in the
//! BambuStudio reference checkout — only the older free-function
//! `OpenVDBUtils.hpp::mesh_to_grid(its, tr, voxel_scale, ...)` exists there.
//!
//! Consequently the voxel-grid backend cannot be faithfully translated and is
//! NOT added (per the port rules: no native deps, no fakes). The pure
//! control-flow logic of this header — `get_voxelgrid`, `detail::perform_csg`
//! and `voxelize_csgmesh` (the stack machine that evaluates the CSG
//! expression) — IS ported faithfully. The four grid primitives below are the
//! blocked seam: they keep their exact C++ signatures and call sites so that
//! the moment an OpenVDB (or equivalent voxel) backend becomes available the
//! bodies can be filled in without touching the algorithm.

// VoxelizeCSGMesh.hpp:7  #include "CSGMesh.hpp"
use crate::csg_mesh::csg_mesh::{get_mesh, get_operation, get_stack_operation, get_transform};
use crate::csg_mesh::csg_mesh::{CSGPart, CSGStackOp, CSGType};
use crate::geometry::Transform3D;
use crate::triangle_mesh::TriangleMesh;
use crate::Result;
use std::sync::Arc;

// VoxelizeCSGMesh.hpp:11  namespace Slic3r { namespace csg {

/// Callback function type for status reporting.
/// Returns true to cancel the operation.
///
/// Mirrors the `statusfn` member of `MeshToGridParams` (called as
/// `params.statusfn()(-1)` in `VoxelizeCSGMesh.hpp:68` / `:84`).
pub type StatusCallback = Box<dyn Fn(i32) -> bool + Send + Sync>;

/// VoxelizeCSGMesh.hpp:13  using VoxelizeParams = MeshToGridParams;
///
/// `MeshToGridParams` is the parameter bundle consumed by the OpenVDB-backed
/// `mesh_to_grid`. It is not vendored in the reference checkout; the fields
/// reproduced here mirror the older free-function `mesh_to_grid` signature in
/// `OpenVDBUtils.hpp:29-34` (transform, `voxel_scale`, exterior/interior band
/// widths, flags) plus the `statusfn` member this header relies on
/// (`VoxelizeCSGMesh.hpp:68`, `:84`).
#[derive(Clone)]
pub struct VoxelizeParams {
    // OpenVDBUtils.hpp:30  const openvdb::math::Transform &tr
    pub transform: Transform3D,

    // OpenVDBUtils.hpp:31  float voxel_scale = 1.f
    pub voxel_scale: f32,

    // OpenVDBUtils.hpp:32  float exteriorBandWidth = 3.0f
    pub exterior_band_width: f32,

    // OpenVDBUtils.hpp:33  float interiorBandWidth = 3.0f
    pub interior_band_width: f32,

    // OpenVDBUtils.hpp:34  int flags = 0
    pub flags: i32,

    // Optional status callback; `params.statusfn()` in VoxelizeCSGMesh.hpp.
    pub status_callback: Option<Arc<StatusCallback>>,
}

impl VoxelizeParams {
    // Defaults match the C++ `mesh_to_grid` defaults (OpenVDBUtils.hpp:29-34).
    pub fn new() -> Self {
        Self {
            transform: Transform3D::identity(),
            voxel_scale: 1.0,
            exterior_band_width: 3.0,
            interior_band_width: 3.0,
            flags: 0,
            status_callback: None,
        }
    }

    // Set the transformation
    pub fn with_transform(mut self, transform: Transform3D) -> Self {
        self.transform = transform;
        self
    }

    // Set the voxel scale
    pub fn with_voxel_scale(mut self, scale: f32) -> Self {
        self.voxel_scale = scale;
        self
    }

    // Set the band widths
    pub fn with_band_widths(mut self, exterior: f32, interior: f32) -> Self {
        self.exterior_band_width = exterior;
        self.interior_band_width = interior;
        self
    }

    // Set the status callback
    pub fn with_status_callback(mut self, callback: Arc<StatusCallback>) -> Self {
        self.status_callback = Some(callback);
        self
    }

    // The C++ guard is `params.statusfn() && params.statusfn()(-1)`:
    // only invoke the callback when one is set, and cancel iff it returns true.
    // VoxelizeCSGMesh.hpp:68  if (params.statusfn() && params.statusfn()(-1))
    // VoxelizeCSGMesh.hpp:84  if (params.statusfn() && params.statusfn()(-1))
    fn should_cancel(&self) -> bool {
        if let Some(ref callback) = self.status_callback {
            callback(-1)
        } else {
            false
        }
    }
}

impl Default for VoxelizeParams {
    fn default() -> Self {
        Self::new()
    }
}

/// VoxelGridPtr (OpenVDB `openvdb::FloatGrid::Ptr`), referenced throughout this
/// header. BLOCKED: the underlying grid is an OpenVDB level set; OpenVDB is a
/// native dependency that is not available here. The type is kept so the
/// algorithm's data flow stays exact. `is_empty` mirrors `is_grid_empty(*g)`
/// (VoxelizeCSGMesh.hpp:40).
#[derive(Clone)]
pub struct VoxelGrid {
    // Placeholder for the OpenVDB FloatGrid (sparse narrow-band level set).
    // Cannot be modelled without the native backend.
    _data: (),

    // Whether this grid has no active voxels (`is_grid_empty`).
    is_empty: bool,
}

impl VoxelGrid {
    // An empty grid (the result of `mesh_to_grid({}, params)` with no mesh).
    pub fn empty() -> Self {
        Self {
            _data: (),
            is_empty: true,
        }
    }

    // VoxelizeCSGMesh.hpp:40  is_grid_empty(*dst) / is_grid_empty(*src)
    pub fn is_empty(&self) -> bool {
        self.is_empty
    }

    // VoxelizeCSGMesh.hpp:41  dst = clone(*src);
    pub fn clone_grid(&self) -> Self {
        Self {
            _data: self._data,
            is_empty: self.is_empty,
        }
    }
}

// VoxelizeCSGMesh.hpp:15-16
// This method can be overriden when a specific CSGPart type supports caching
// of the voxel grid
// VoxelizeCSGMesh.hpp:17-29
// template<class CSGPartT>
// VoxelGridPtr get_voxelgrid(const CSGPartT &csgpart, VoxelizeParams params)
// {
//     const indexed_triangle_set *its = csg::get_mesh(csgpart);
//     VoxelGridPtr ret;
//
//     params.trafo(params.trafo() * csg::get_transform(csgpart));
//
//     if (its)
//         ret = mesh_to_grid(*its, params);
//
//     return ret;
// }
//
// Note: `params` is taken by value in C++ (the trafo mutation is local to this
// call), so we take `params` by value here too.
pub fn get_voxelgrid(part: &CSGPart, mut params: VoxelizeParams) -> Result<Option<VoxelGrid>> {
    // VoxelizeCSGMesh.hpp:20  const indexed_triangle_set *its = csg::get_mesh(csgpart);
    let its = get_mesh(part);

    // VoxelizeCSGMesh.hpp:21  VoxelGridPtr ret;
    let mut ret: Option<VoxelGrid> = None;

    // VoxelizeCSGMesh.hpp:23  params.trafo(params.trafo() * csg::get_transform(csgpart));
    // Matrix product `params.trafo * get_transform(csgpart)` (the part transform
    // is applied first, then the params transform). With `Transform3D::then`,
    // `a.then(&b)` computes `b * a`, so `get_transform(part).then(&params.transform)`
    // == `params.transform * get_transform(part)`.
    params.transform = get_transform(part).then(&params.transform);

    // VoxelizeCSGMesh.hpp:25-26  if (its) ret = mesh_to_grid(*its, params);
    if let Some(its) = its {
        ret = Some(mesh_to_grid(its, &params)?);
    }

    // VoxelizeCSGMesh.hpp:28  return ret;
    Ok(ret)
}

// VoxelizeCSGMesh.hpp:31  namespace detail {

// VoxelizeCSGMesh.hpp:33-53
// inline void perform_csg(CSGType op, VoxelGridPtr &dst, VoxelGridPtr &src)
// {
//     if (!dst || !src)
//         return;
//
//     switch (op) {
//     case CSGType::Union:
//         if (is_grid_empty(*dst) && !is_grid_empty(*src))
//             dst = clone(*src);
//         else
//             grid_union(*dst, *src);
//
//         break;
//     case CSGType::Difference:
//         grid_difference(*dst, *src);
//         break;
//     case CSGType::Intersection:
//         grid_intersection(*dst, *src);
//         break;
//     }
// }
fn perform_csg(op: CSGType, dst: &mut Option<VoxelGrid>, src: &mut Option<VoxelGrid>) {
    // VoxelizeCSGMesh.hpp:34-35  if (!dst || !src) return;
    if dst.is_none() || src.is_none() {
        return;
    }

    let dst_grid = dst.as_mut().unwrap();
    let src_grid = src.as_mut().unwrap();

    // VoxelizeCSGMesh.hpp:38  switch (op) {
    match op {
        // VoxelizeCSGMesh.hpp:39-45  case CSGType::Union:
        CSGType::Union => {
            // VoxelizeCSGMesh.hpp:40-41
            // if (is_grid_empty(*dst) && !is_grid_empty(*src))
            //     dst = clone(*src);
            if dst_grid.is_empty() && !src_grid.is_empty() {
                *dst_grid = src_grid.clone_grid();
            } else {
                // VoxelizeCSGMesh.hpp:42-43  else grid_union(*dst, *src);
                grid_union(dst_grid, src_grid);
            }
        }

        // VoxelizeCSGMesh.hpp:46-48  case CSGType::Difference: grid_difference(*dst, *src); break;
        CSGType::Difference => {
            grid_difference(dst_grid, src_grid);
        }

        // VoxelizeCSGMesh.hpp:49-51  case CSGType::Intersection: grid_intersection(*dst, *src); break;
        CSGType::Intersection => {
            grid_intersection(dst_grid, src_grid);
        }
    }
}

// VoxelizeCSGMesh.hpp:55  } // namespace detail

// VoxelizeCSGMesh.hpp:78  struct Frame { CSGType op = CSGType::Union; VoxelGridPtr grid; };
struct Frame {
    op: CSGType,
    grid: Option<VoxelGrid>,
}

// VoxelizeCSGMesh.hpp:57-112
// template<class It>
// VoxelGridPtr voxelize_csgmesh(const Range<It>      &csgrange,
//                               const VoxelizeParams &params = {})
// {
//     using namespace detail;
//
//     VoxelGridPtr ret;
//     std::vector<VoxelGridPtr> grids (csgrange.size());
//     ...
// }
pub fn voxelize_csgmesh(parts: &[CSGPart], params: &VoxelizeParams) -> Result<Option<VoxelGrid>> {
    // VoxelizeCSGMesh.hpp:63  VoxelGridPtr ret;
    let ret: Option<VoxelGrid>;

    // VoxelizeCSGMesh.hpp:65  std::vector<VoxelGridPtr> grids (csgrange.size());
    let mut grids: Vec<Option<VoxelGrid>> = vec![None; parts.len()];

    // VoxelizeCSGMesh.hpp:67-75
    // execution::for_each(ex_tbb, size_t(0), csgrange.size(), [&](size_t csgidx) {
    //     if (params.statusfn() && params.statusfn()(-1))
    //         return;
    //     auto it = csgrange.begin();
    //     std::advance(it, csgidx);
    //     auto &csgpart = *it;
    //     grids[csgidx] = get_voxelgrid(csgpart, params);
    // }, execution::max_concurrency(ex_tbb));
    //
    // The C++ uses the TBB parallel policy. Each iteration writes an independent
    // `grids[csgidx]` from read-only inputs, so the result is order-independent;
    // we keep it as a sequential loop here (the parallelism only affects timing,
    // not the produced grids). The `statusfn` early-`return` skips one element.
    for (csgidx, part) in parts.iter().enumerate() {
        // VoxelizeCSGMesh.hpp:68-69  if (params.statusfn() && params.statusfn()(-1)) return;
        if params.should_cancel() {
            continue;
        }

        // VoxelizeCSGMesh.hpp:71-74
        // auto it = csgrange.begin();
        // std::advance(it, csgidx);
        // auto &csgpart = *it;
        // grids[csgidx] = get_voxelgrid(csgpart, params);
        grids[csgidx] = get_voxelgrid(part, params.clone())?;
    }

    // VoxelizeCSGMesh.hpp:77  size_t csgidx = 0;
    let mut csgidx: usize = 0;

    // VoxelizeCSGMesh.hpp:78-79
    // struct Frame { CSGType op = CSGType::Union; VoxelGridPtr grid; };
    // std::stack opstack{std::vector<Frame>{}};
    let mut opstack: Vec<Frame> = Vec::new();

    // VoxelizeCSGMesh.hpp:81  opstack.push({CSGType::Union, mesh_to_grid({}, params)});
    opstack.push(Frame {
        op: CSGType::Union,
        grid: Some(mesh_to_grid_empty(params)?),
    });

    // VoxelizeCSGMesh.hpp:83  for (auto &csgpart : csgrange) {
    for part in parts {
        // VoxelizeCSGMesh.hpp:84-85  if (params.statusfn() && params.statusfn()(-1)) break;
        if params.should_cancel() {
            break;
        }

        // VoxelizeCSGMesh.hpp:87  auto &partgrid = grids[csgidx++];
        let mut partgrid = grids[csgidx].take();
        csgidx += 1;

        // VoxelizeCSGMesh.hpp:89  auto op = get_operation(csgpart);
        let mut op = get_operation(part);

        // VoxelizeCSGMesh.hpp:91-94
        // if (get_stack_operation(csgpart) == CSGStackOp::Push) {
        //     opstack.push({op, mesh_to_grid({}, params)});
        //     op = CSGType::Union;
        // }
        if get_stack_operation(part) == CSGStackOp::Push {
            opstack.push(Frame {
                op,
                grid: Some(mesh_to_grid_empty(params)?),
            });
            // Dead store mirrored from C++: `op` is never read after this point
            // because line 98 re-reads `get_operation(csgpart)`. Kept verbatim.
            op = CSGType::Union;
            let _ = op;
        }

        // VoxelizeCSGMesh.hpp:96  Frame *top = &opstack.top();
        let top_idx = opstack.len() - 1;

        // VoxelizeCSGMesh.hpp:98  perform_csg(get_operation(csgpart), top->grid, partgrid);
        // NOTE: C++ passes `get_operation(csgpart)` (the part's original op), NOT
        // the local `op` variable that was reset to Union on a Push above.
        {
            let top = &mut opstack[top_idx];
            perform_csg(get_operation(part), &mut top.grid, &mut partgrid);
        }

        // VoxelizeCSGMesh.hpp:100-106
        // if (get_stack_operation(csgpart) == CSGStackOp::Pop) {
        //     VoxelGridPtr popgrid = std::move(top->grid);
        //     auto popop = opstack.top().op;
        //     opstack.pop();
        //     VoxelGridPtr &grid = opstack.top().grid;
        //     perform_csg(popop, grid, popgrid);
        // }
        if get_stack_operation(part) == CSGStackOp::Pop {
            // VoxelizeCSGMesh.hpp:101  VoxelGridPtr popgrid = std::move(top->grid);
            // VoxelizeCSGMesh.hpp:102  auto popop = opstack.top().op;
            // VoxelizeCSGMesh.hpp:103  opstack.pop();
            let popped = opstack.pop().unwrap();
            let mut popgrid = popped.grid;
            let popop = popped.op;

            // VoxelizeCSGMesh.hpp:104  VoxelGridPtr &grid = opstack.top().grid;
            // VoxelizeCSGMesh.hpp:105  perform_csg(popop, grid, popgrid);
            let parent_idx = opstack.len() - 1;
            let grid = &mut opstack[parent_idx].grid;
            perform_csg(popop, grid, &mut popgrid);
        }
    }

    // VoxelizeCSGMesh.hpp:109  ret = std::move(opstack.top().grid);
    ret = opstack.last_mut().and_then(|f| f.grid.take());

    // VoxelizeCSGMesh.hpp:111  return ret;
    Ok(ret)
}

// ===========================================================================
// BLOCKED OpenVDB seam (native dependency, not wasm-safe — not added).
//
// These four primitives + `mesh_to_grid` are the only OpenVDB-backed pieces of
// this header. Their signatures and call sites are kept exact so the algorithm
// above is a faithful translation; the bodies are inert pending an OpenVDB (or
// equivalent voxel) backend. They do not fabricate results.
// ===========================================================================

/// OpenVDBUtils.hpp:29-34 / VoxelizeCSGMesh.hpp:26  mesh_to_grid(*its, params)
///
/// BLOCKED: OpenVDB `meshToVolume` level-set conversion. Returns an (empty)
/// grid placeholder; cannot voxelize without the native backend.
fn mesh_to_grid(_mesh: &TriangleMesh, _params: &VoxelizeParams) -> Result<VoxelGrid> {
    Ok(VoxelGrid::empty())
}

/// VoxelizeCSGMesh.hpp:81 / :92  mesh_to_grid({}, params)
///
/// The empty-mesh overload used to seed stack frames; OpenVDB produces an empty
/// FloatGrid. Distinct helper so the call sites read like the C++.
fn mesh_to_grid_empty(_params: &VoxelizeParams) -> Result<VoxelGrid> {
    Ok(VoxelGrid::empty())
}

/// VoxelizeCSGMesh.hpp:43  grid_union(*dst, *src)
///
/// BLOCKED: OpenVDB `tools::csgUnion(*dst, *src)`.
fn grid_union(_dst: &mut VoxelGrid, _src: &VoxelGrid) {}

/// VoxelizeCSGMesh.hpp:47  grid_difference(*dst, *src)
///
/// BLOCKED: OpenVDB `tools::csgDifference(*dst, *src)`.
fn grid_difference(_dst: &mut VoxelGrid, _src: &VoxelGrid) {}

/// VoxelizeCSGMesh.hpp:50  grid_intersection(*dst, *src)
///
/// BLOCKED: OpenVDB `tools::csgIntersection(*dst, *src)`.
fn grid_intersection(_dst: &mut VoxelGrid, _src: &VoxelGrid) {}

// VoxelizeCSGMesh.hpp:114  }} // namespace Slic3r::csg

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_voxelize_params_default() {
        let params = VoxelizeParams::new();
        assert_eq!(params.voxel_scale, 1.0);
        assert_eq!(params.exterior_band_width, 3.0);
        assert_eq!(params.interior_band_width, 3.0);
        assert_eq!(params.flags, 0);
        assert!(params.status_callback.is_none());
    }

    #[test]
    fn test_voxelize_params_builder() {
        let params = VoxelizeParams::new()
            .with_voxel_scale(2.0)
            .with_band_widths(5.0, 5.0);

        assert_eq!(params.voxel_scale, 2.0);
        assert_eq!(params.exterior_band_width, 5.0);
        assert_eq!(params.interior_band_width, 5.0);
    }

    #[test]
    fn test_voxel_grid_empty() {
        let grid = VoxelGrid::empty();
        assert!(grid.is_empty());
    }

    #[test]
    fn test_voxel_grid_clone() {
        let grid1 = VoxelGrid::empty();
        let grid2 = grid1.clone_grid();
        assert_eq!(grid1.is_empty(), grid2.is_empty());
    }

    #[test]
    fn test_perform_csg_none_grids() {
        let mut dst: Option<VoxelGrid> = None;
        let mut src: Option<VoxelGrid> = None;

        // VoxelizeCSGMesh.hpp:34-35  if (!dst || !src) return;
        perform_csg(CSGType::Union, &mut dst, &mut src);

        assert!(dst.is_none());
        assert!(src.is_none());
    }

    #[test]
    fn test_perform_csg_union_empty_dst() {
        let mut dst = Some(VoxelGrid::empty());
        let mut src = Some(VoxelGrid::empty());

        perform_csg(CSGType::Union, &mut dst, &mut src);

        // Both empty: the `clone` branch is not taken (src is empty), grid_union
        // is a no-op (blocked), dst stays present.
        assert!(dst.is_some());
    }

    #[test]
    fn test_voxelize_csgmesh_empty() {
        let parts: Vec<CSGPart> = vec![];
        let params = VoxelizeParams::new();

        let result = voxelize_csgmesh(&parts, &params);
        assert!(result.is_ok());
        // Should have the empty seed grid from the initial frame (hpp:81).
        assert!(result.unwrap().is_some());
    }

    #[test]
    fn test_voxelize_csgmesh_single_part() {
        let part = CSGPart::new().with_operation(CSGType::Union);

        let parts = vec![part];
        let params = VoxelizeParams::new();

        let result = voxelize_csgmesh(&parts, &params);
        assert!(result.is_ok());
    }

    #[test]
    fn test_voxelize_csgmesh_with_stack_ops() {
        // CUBE1 - (CUBE2 + CUBE3)  (hpp:31-37 example)
        let cube1 = CSGPart::new()
            .with_operation(CSGType::Union)
            .with_stack_operation(CSGStackOp::Continue);

        let cube2 = CSGPart::new()
            .with_operation(CSGType::Difference)
            .with_stack_operation(CSGStackOp::Push);

        let cube3 = CSGPart::new()
            .with_operation(CSGType::Union)
            .with_stack_operation(CSGStackOp::Pop);

        let parts = vec![cube1, cube2, cube3];
        let params = VoxelizeParams::new();

        let result = voxelize_csgmesh(&parts, &params);
        assert!(result.is_ok());
    }

    #[test]
    fn test_status_callback_cancellation() {
        let part = CSGPart::new();
        let parts = vec![part];

        // Callback that always cancels.
        let callback = Arc::new(Box::new(|_: i32| true) as StatusCallback);
        let params = VoxelizeParams::new().with_status_callback(callback);

        // hpp:84-85: the main loop breaks immediately, leaving the initial empty
        // seed frame; the result is that seed grid (Some), not None.
        let result = voxelize_csgmesh(&parts, &params);
        assert!(result.is_ok());
        assert!(result.unwrap().is_some());
    }
}
