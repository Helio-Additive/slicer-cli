//! CSG mesh voxelization using OpenVDB.
//!
//! This module provides functions for converting CSG mesh collections into voxel grids
//! using OpenVDB's level set functionality. The voxelization process supports:
//! - Parallel mesh-to-grid conversion
//! - CSG operations on voxel grids (union, difference, intersection)
//! - Stack-based expression evaluation (parentheses in CSG expressions)
//!
//! C++ Reference:
//! - CSGMesh/VoxelizeCSGMesh.hpp
//!
//! **Note:** This implementation provides stubs for OpenVDB operations. Full functionality
//! requires OpenVDB bindings (e.g., via FFI to C++ OpenVDB or a pure Rust voxel library).
//!
//! ## Architecture
//!
//! The voxelization process has two phases:
//!
//! 1. **Parallel Phase**: Convert each CSG part's mesh to a voxel grid independently
//! 2. **Sequential Phase**: Evaluate the CSG expression using a stack-based algorithm
//!
//! ## Example CSG Expression
//!
//! For the expression `CUBE1 - (CUBE2 + CUBE3)`:
//!
//! ```text
//! Stack operations:
//! 1. Start with empty grid (Union operation)
//! 2. Union CUBE1 → result in top stack frame
//! 3. Push new frame (Difference operation starts)
//! 4. Union CUBE2 into new frame
//! 5. Union CUBE3 into new frame
//! 6. Pop frame → apply Difference to previous frame
//! ```

use crate::csg_mesh::csg_mesh::{get_mesh, get_operation, get_stack_operation, get_transform};
use crate::csg_mesh::csg_mesh::{CSGPart, CSGStackOp, CSGType};
use crate::geometry::Transform3D;
use crate::triangle_mesh::TriangleMesh;
use crate::{Error, Result};
use std::sync::Arc;

/// Callback function type for status reporting.
/// Returns true to cancel the operation.
/// VoxelizeCSGMesh.hpp (statusfn in MeshToGridParams)
pub type StatusCallback = Box<dyn Fn(i32) -> bool + Send + Sync>;

/// Parameters for mesh-to-grid conversion.
///
/// This is the Rust equivalent of C++'s `MeshToGridParams` type alias.
/// In C++, this is defined in OpenVDBUtils and aliased in VoxelizeCSGMesh.hpp:13.
///
/// VoxelizeCSGMesh.hpp:13
/// C++: using VoxelizeParams = MeshToGridParams;
#[derive(Clone)]
pub struct VoxelizeParams {
    // Transformation to apply to the mesh
    // OpenVDBUtils.hpp (openvdb::math::Transform parameter)
    pub transform: Transform3D,

    // Voxel scale factor (1.0 = 1 voxel per unit cube)
    // OpenVDBUtils.hpp:32
    pub voxel_scale: f32,

    // Width of the exterior narrow band (in voxels)
    // OpenVDBUtils.hpp:33
    pub exterior_band_width: f32,

    // Width of the interior narrow band (in voxels)
    // OpenVDBUtils.hpp:34
    pub interior_band_width: f32,

    // OpenVDB mesh-to-volume conversion flags
    // OpenVDBUtils.hpp:35
    pub flags: i32,

    // Optional status callback for progress reporting and cancellation
    pub status_callback: Option<Arc<StatusCallback>>,
}

impl VoxelizeParams {
    // Create default voxelization parameters
    // OpenVDBUtils.hpp:29-35
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

    // Check if the operation should be cancelled
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

/// A voxel grid handle.
///
/// This is the Rust equivalent of C++'s `VoxelGridPtr` (openvdb::FloatGrid::Ptr).
/// Currently a placeholder for future OpenVDB integration.
///
/// VoxelizeCSGMesh.hpp:14 (VoxelGridPtr is openvdb::FloatGrid::Ptr)
#[derive(Clone)]
pub struct VoxelGrid {
    // Internal grid data (placeholder for OpenVDB FloatGrid)
    // In a full implementation, this would contain:
    // - Voxel data structure (sparse grid)
    // - Transform/scale information
    // - Metadata (voxel_scale, etc.)
    _data: (),

    // Whether this grid is empty (contains no active voxels)
    is_empty: bool,
}

impl VoxelGrid {
    // Create an empty voxel grid
    pub fn empty() -> Self {
        Self {
            _data: (),
            is_empty: true,
        }
    }

    // Check if the grid is empty
    pub fn is_empty(&self) -> bool {
        self.is_empty
    }

    // Clone this grid
    // VoxelizeCSGMesh.hpp:41 (clone(*src))
    pub fn clone_grid(&self) -> Self {
        Self {
            _data: self._data,
            is_empty: self.is_empty,
        }
    }
}

/// Get the voxel grid for a CSG part.
///
/// This method can be overridden when a specific CSGPart type supports caching
/// of the voxel grid. It converts the mesh to a voxel grid with the given parameters.
///
/// VoxelizeCSGMesh.hpp:16-28
pub fn get_voxelgrid(part: &CSGPart, mut params: VoxelizeParams) -> Result<Option<VoxelGrid>> {
    // Get the mesh pointer from the CSG part
    // VoxelizeCSGMesh.hpp:18
    // C++: const indexed_triangle_set *its = csg::get_mesh(csgpart);
    let mesh_opt = get_mesh(part);

    // Initialize return value
    // VoxelizeCSGMesh.hpp:19
    // C++: VoxelGridPtr ret;
    let mut ret: Option<VoxelGrid> = None;

    // Get the part transformation
    // VoxelizeCSGMesh.hpp:21
    // C++: params.trafo(params.trafo() * csg::get_transform(csgpart));
    // TODO: Compose transformations when Transform3D::compose() is implemented
    let _part_transform = get_transform(part);
    // For now, use params.transform as-is
    // In full implementation: params.transform = params.transform * part_transform

    // Convert mesh to grid if mesh is present
    // VoxelizeCSGMesh.hpp:23-24
    // C++: if (its)
    // C++: ret = mesh_to_grid(*its, params);
    if let Some(mesh) = mesh_opt {
        ret = Some(mesh_to_grid(mesh, &params)?);
    }

    // Return the voxel grid
    // VoxelizeCSGMesh.hpp:26
    // C++: return ret;
    Ok(ret)
}

/// Perform a CSG operation on two voxel grids.
///
/// This is an internal helper function that applies the specified CSG operation
/// (union, difference, or intersection) to modify `dst` using `src`.
///
/// VoxelizeCSGMesh.hpp:32-55
fn perform_csg(op: CSGType, dst: &mut Option<VoxelGrid>, src: &mut Option<VoxelGrid>) {
    // Early return if either grid is None
    // VoxelizeCSGMesh.hpp:34-35
    // C++: if (!dst || !src)
    // C++: return;
    if dst.is_none() || src.is_none() {
        return;
    }

    let dst_grid = dst.as_mut().unwrap();
    let src_grid = src.as_mut().unwrap();

    // Switch on the CSG operation type
    // VoxelizeCSGMesh.hpp:37
    // C++: switch (op) {
    match op {
        // Union operation - combine the two grids
        // VoxelizeCSGMesh.hpp:38-44
        // C++: case CSGType::Union:
        CSGType::Union => {
            /// If destination is empty and source is not, clone source to destination
            /// VoxelizeCSGMesh.hpp:39-40
            /// C++: if (is_grid_empty(*dst) && !is_grid_empty(*src))
            /// C++: dst = clone(*src);
            if dst_grid.is_empty() && !src_grid.is_empty() {
                *dst_grid = src_grid.clone_grid();
            } else {
                /// Otherwise perform CSG union
                /// VoxelizeCSGMesh.hpp:41-42
                /// C++: else
                /// C++: grid_union(*dst, *src);
                grid_union(dst_grid, src_grid);
            }
        }

        // Difference operation - subtract source from destination
        // VoxelizeCSGMesh.hpp:45-47
        // C++: case CSGType::Difference:
        // C++: grid_difference(*dst, *src);
        // C++: break;
        CSGType::Difference => {
            grid_difference(dst_grid, src_grid);
        }

        // Intersection operation - only keep overlapping regions
        // VoxelizeCSGMesh.hpp:48-50
        // C++: case CSGType::Intersection:
        // C++: grid_intersection(*dst, *src);
        // C++: break;
        CSGType::Intersection => {
            grid_intersection(dst_grid, src_grid);
        }
    }
}

/// Stack frame for CSG expression evaluation.
///
/// Used to implement parentheses in CSG expressions through push/pop operations.
///
/// VoxelizeCSGMesh.hpp:76
/// C++: struct Frame { CSGType op = CSGType::Union; VoxelGridPtr grid; };
struct Frame {
    // The CSG operation to apply when this frame is popped
    // VoxelizeCSGMesh.hpp:76
    op: CSGType,

    // The accumulated voxel grid for this frame
    // VoxelizeCSGMesh.hpp:76
    grid: Option<VoxelGrid>,
}

impl Frame {
    // Create a new frame with default Union operation
    // VoxelizeCSGMesh.hpp:76
    fn new() -> Self {
        Self {
            op: CSGType::Union,
            grid: Some(VoxelGrid::empty()),
        }
    }

    // Create a frame with a specific operation
    fn with_operation(op: CSGType) -> Self {
        Self {
            op,
            grid: Some(VoxelGrid::empty()),
        }
    }
}

/// Voxelize a collection of CSG parts into a single voxel grid.
///
/// This function processes a collection of CSG parts in two phases:
///
/// 1. **Parallel Phase**: Convert each mesh to a voxel grid independently
/// 2. **Sequential Phase**: Evaluate the CSG expression using a stack-based algorithm
///
/// The stack-based evaluation supports parentheses through Push/Pop operations.
///
/// VoxelizeCSGMesh.hpp:59-108
pub fn voxelize_csgmesh(parts: &[CSGPart], params: &VoxelizeParams) -> Result<Option<VoxelGrid>> {
    // Initialize return value
    // VoxelizeCSGMesh.hpp:64
    // C++: VoxelGridPtr ret;
    let ret: Option<VoxelGrid>;

    // Pre-allocate vector for voxel grids
    // VoxelizeCSGMesh.hpp:66
    // C++: std::vector<VoxelGridPtr> grids (csgrange.size());
    let mut grids: Vec<Option<VoxelGrid>> = vec![None; parts.len()];

    // Parallel mesh-to-grid conversion
    // VoxelizeCSGMesh.hpp:68-75
    // C++: execution::for_each(ex_tbb, size_t(0), csgrange.size(), [&](size_t csgidx) {
    // C++: if (params.statusfn() && params.statusfn()(-1))
    // C++: return;
    // C++: auto it = csgrange.begin();
    // C++: std::advance(it, csgidx);
    // C++: auto &csgpart = *it;
    // C++: grids[csgidx] = get_voxelgrid(csgpart, params);
    // C++: }, execution::max_concurrency(ex_tbb));
    // TODO: Use rayon for parallel iteration when OpenVDB bindings are available
    // For now, use sequential processing
    for (csgidx, part) in parts.iter().enumerate() {
        // Check for cancellation
        if params.should_cancel() {
            return Ok(None);
        }

        // Convert mesh to voxel grid
        grids[csgidx] = get_voxelgrid(part, params.clone())?;
    }

    // Sequential CSG expression evaluation with stack
    // VoxelizeCSGMesh.hpp:77
    // C++: size_t csgidx = 0;
    let mut csgidx: usize = 0;

    // Initialize operation stack with initial frame
    // VoxelizeCSGMesh.hpp:78-79
    // C++: struct Frame { CSGType op = CSGType::Union; VoxelGridPtr grid; };
    // C++: std::stack opstack{std::vector<Frame>{}};
    let mut opstack: Vec<Frame> = Vec::new();

    // Push initial frame with empty grid
    // VoxelizeCSGMesh.hpp:81
    // C++: opstack.push({CSGType::Union, mesh_to_grid({}, params)});
    opstack.push(Frame::new());

    // Iterate through CSG parts and evaluate expression
    // VoxelizeCSGMesh.hpp:83
    // C++: for (auto &csgpart : csgrange) {
    for part in parts {
        // Check for cancellation
        // VoxelizeCSGMesh.hpp:84-85
        // C++: if (params.statusfn() && params.statusfn()(-1))
        // C++: break;
        if params.should_cancel() {
            break;
        }

        // Get the pre-computed voxel grid for this part
        // VoxelizeCSGMesh.hpp:87
        // C++: auto &partgrid = grids[csgidx++];
        let mut partgrid = grids[csgidx].take();
        csgidx += 1;

        // Get the CSG operation for this part
        // VoxelizeCSGMesh.hpp:89
        // C++: auto op = get_operation(csgpart);
        let mut op = get_operation(part);

        // Handle Push operation - start a new sub-expression
        // VoxelizeCSGMesh.hpp:91-94
        // C++: if (get_stack_operation(csgpart) == CSGStackOp::Push) {
        // C++: opstack.push({op, mesh_to_grid({}, params)});
        // C++: op = CSGType::Union;
        // C++: }
        if get_stack_operation(part) == CSGStackOp::Push {
            opstack.push(Frame::with_operation(op));
            op = CSGType::Union;
        }

        // Get the top frame from the stack
        // VoxelizeCSGMesh.hpp:96
        // C++: Frame *top = &opstack.top();
        let top_idx = opstack.len() - 1;

        // Perform CSG operation on the top frame's grid
        // VoxelizeCSGMesh.hpp:98
        // C++: perform_csg(get_operation(csgpart), top->grid, partgrid);
        let top_frame = &mut opstack[top_idx];
        perform_csg(op, &mut top_frame.grid, &mut partgrid);

        // Handle Pop operation - complete the sub-expression
        // VoxelizeCSGMesh.hpp:100-106
        // C++: if (get_stack_operation(csgpart) == CSGStackOp::Pop) {
        // C++: VoxelGridPtr popgrid = std::move(top->grid);
        // C++: auto popop = opstack.top().op;
        // C++: opstack.pop();
        // C++: VoxelGridPtr &grid = opstack.top().grid;
        // C++: perform_csg(popop, grid, popgrid);
        // C++: }
        if get_stack_operation(part) == CSGStackOp::Pop {
            let popped = opstack.pop().unwrap();
            let mut popgrid = popped.grid;
            let popop = popped.op;

            if let Some(parent) = opstack.last_mut() {
                perform_csg(popop, &mut parent.grid, &mut popgrid);
            }
        }
    }

    // Extract final result from the top of the stack
    // VoxelizeCSGMesh.hpp:109
    // C++: ret = std::move(opstack.top().grid);
    ret = opstack.pop().map(|f| f.grid).flatten();

    // Return the final voxel grid
    // VoxelizeCSGMesh.hpp:111
    // C++: return ret;
    Ok(ret)
}

// ============================================================================
// OpenVDB Grid Operation Stubs
// ============================================================================
//
// The following functions are stubs for OpenVDB operations. In a full
// implementation, these would either:
// 1. Use FFI to call C++ OpenVDB functions
// 2. Use a Rust voxel library with equivalent functionality
// 3. Bind to the openvdb-rs crate (if it becomes available)

/// Convert a triangle mesh to a voxel grid.
///
/// **STUB:** This is a placeholder for OpenVDB's mesh_to_volume functionality.
/// A full implementation would use OpenVDB's level set conversion.
///
/// OpenVDBUtils.hpp:29-35
/// OpenVDBUtils.cpp:48-88
fn mesh_to_grid(_mesh: &TriangleMesh, _params: &VoxelizeParams) -> Result<VoxelGrid> {
    // TODO: Implement mesh-to-grid conversion
    // This requires OpenVDB bindings or equivalent voxel library
    //
    // C++ implementation:
    // 1. Split mesh into parts (its_split)
    // 2. For each part:
    //    - Create TriangleMeshDataAdapter
    //    - Call openvdb::tools::meshToVolume
    //    - Union the resulting grids
    // 3. Rebuild level set (levelSetRebuild)
    // 4. Store voxel_scale in metadata

    Ok(VoxelGrid::empty())
}

/// Perform union of two voxel grids (modifies dst).
///
/// **STUB:** This is a placeholder for OpenVDB's csgUnion operation.
///
/// VoxelizeCSGMesh.hpp:42 (calls OpenVDB tools::csgUnion)
fn grid_union(_dst: &mut VoxelGrid, _src: &VoxelGrid) {
    // TODO: Implement grid union
    // C++ uses: openvdb::tools::csgUnion(*dst, *src)
}

/// Perform difference of two voxel grids (dst = dst - src).
///
/// **STUB:** This is a placeholder for OpenVDB's csgDifference operation.
///
/// VoxelizeCSGMesh.hpp:46 (calls OpenVDB tools::csgDifference)
fn grid_difference(_dst: &mut VoxelGrid, _src: &VoxelGrid) {
    // TODO: Implement grid difference
    // C++ uses: openvdb::tools::csgDifference(*dst, *src)
}

/// Perform intersection of two voxel grids (modifies dst).
///
/// **STUB:** This is a placeholder for OpenVDB's csgIntersection operation.
///
/// VoxelizeCSGMesh.hpp:49 (calls OpenVDB tools::csgIntersection)
fn grid_intersection(_dst: &mut VoxelGrid, _src: &VoxelGrid) {
    // TODO: Implement grid intersection
    // C++ uses: openvdb::tools::csgIntersection(*dst, *src)
}

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
    fn test_frame_default() {
        let frame = Frame::new();
        assert_eq!(frame.op, CSGType::Union);
        assert!(frame.grid.is_some());
    }

    #[test]
    fn test_frame_with_operation() {
        let frame = Frame::with_operation(CSGType::Difference);
        assert_eq!(frame.op, CSGType::Difference);
        assert!(frame.grid.is_some());
    }

    #[test]
    fn test_perform_csg_none_grids() {
        let mut dst: Option<VoxelGrid> = None;
        let mut src: Option<VoxelGrid> = None;

        // Should not panic with None grids
        perform_csg(CSGType::Union, &mut dst, &mut src);

        assert!(dst.is_none());
        assert!(src.is_none());
    }

    #[test]
    fn test_perform_csg_union_empty_dst() {
        let mut dst = Some(VoxelGrid::empty());
        let mut src = Some(VoxelGrid::empty());

        perform_csg(CSGType::Union, &mut dst, &mut src);

        // Both empty, so dst should remain empty after cloning
        assert!(dst.is_some());
    }

    #[test]
    fn test_voxelize_csgmesh_empty() {
        let parts: Vec<CSGPart> = vec![];
        let params = VoxelizeParams::new();

        let result = voxelize_csgmesh(&parts, &params);
        assert!(result.is_ok());
        assert!(result.unwrap().is_some()); // Should have empty grid from initial frame
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
        // Test the example: CUBE1 - (CUBE2 + CUBE3)
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

        // Create params with callback that always cancels
        let callback = Arc::new(Box::new(|_: i32| true) as StatusCallback);
        let params = VoxelizeParams::new().with_status_callback(callback);

        assert!(params.should_cancel());

        let result = voxelize_csgmesh(&parts, &params);
        assert!(result.is_ok());
        // Result should be None due to cancellation
        assert!(result.unwrap().is_none());
    }
}
