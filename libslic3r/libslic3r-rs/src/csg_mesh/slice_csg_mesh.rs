//! CSG mesh slicing.
//!
//! C++ Reference:
//! - CSGMesh/SliceCSGMesh.hpp
//!
//! Provides functions for slicing CSG mesh collections into 2D polygon layers.
//! The slicing respects CSG operations (union, difference, intersection) and
//! stack-based sub-expression evaluation.

use super::csg_mesh::{get_mesh, get_operation, get_stack_operation, CSGPart, CSGStackOp, CSGType};
use crate::geometry::ExPolygon;

/// Type alias for a vector of ExPolygons (one layer of sliced polygons).
pub type ExPolygons = Vec<ExPolygon>;

/// Merge slices according to CSG operation.
///
/// SliceCSGMesh.hpp:16-32
pub fn merge_slices(
    op: CSGType,
    i: usize,
    target: &mut Vec<ExPolygons>,
    source: &mut Vec<ExPolygons>,
) {
    match op {
        CSGType::Union => {
            // Move source polygons into target
            // SliceCSGMesh.hpp:22-23
            let mut src_polys = std::mem::take(&mut source[i]);
            target[i].append(&mut src_polys);
        }
        CSGType::Difference => {
            // target[i] = diff_ex(target[i], source[i])
            // SliceCSGMesh.hpp:25
            // NOTE: Requires clipper diff implementation
            // For now, keep target as-is (stub for actual polygon boolean)
            let _source_polys = &source[i];
            // TODO: target[i] = diff_ex(target[i], source[i]);
        }
        CSGType::Intersection => {
            // target[i] = intersection_ex(target[i], source[i])
            // SliceCSGMesh.hpp:27-28
            // NOTE: Requires clipper intersection implementation
            let _source_polys = &source[i];
            // TODO: target[i] = intersection_ex(target[i], source[i]);
        }
    }
}

/// Collect indices of non-empty slices (or all for intersection).
///
/// SliceCSGMesh.hpp:34-44
pub fn collect_nonempty_indices(
    op: CSGType,
    slicegrid: &[f32],
    slices: &[ExPolygons],
    indices: &mut Vec<usize>,
) {
    indices.clear();
    for i in 0..slicegrid.len() {
        // For intersection, we process all indices
        // For other operations, only non-empty slices
        // SliceCSGMesh.hpp:41-43
        if op == CSGType::Intersection || !slices[i].is_empty() {
            indices.push(i);
        }
    }
}

/// Stack frame for CSG slice expression evaluation.
///
/// SliceCSGMesh.hpp:57
struct Frame {
    op: CSGType,
    slices: Vec<ExPolygons>,
}

/// Slice a CSG mesh collection into 2D layers.
///
/// This is the main entry point for CSG-aware mesh slicing. It processes
/// the collection of CSG parts using a stack-based algorithm to handle
/// sub-expressions (Push/Pop operations).
///
/// SliceCSGMesh.hpp:48-127
pub fn slice_csgmesh_ex(parts: &[CSGPart], slicegrid: &[f32]) -> Vec<ExPolygons> {
    let grid_size = slicegrid.len();

    let mut opstack: Vec<Frame> = Vec::new();
    let mut nonempty_indices: Vec<usize> = Vec::with_capacity(grid_size);

    // Initialize with Union frame
    // SliceCSGMesh.hpp:65
    opstack.push(Frame {
        op: CSGType::Union,
        slices: vec![Vec::new(); grid_size],
    });

    // Process each CSG part
    // SliceCSGMesh.hpp:67
    for part in parts {
        let mesh_opt = get_mesh(part);
        let mut op = get_operation(part);

        // Handle Push: start a new sub-expression
        // SliceCSGMesh.hpp:72-75
        if get_stack_operation(part) == CSGStackOp::Push {
            opstack.push(Frame {
                op,
                slices: vec![Vec::new(); grid_size],
            });
            op = CSGType::Union;
        }

        // Slice the mesh if present
        // SliceCSGMesh.hpp:79-93
        if let Some(_mesh) = mesh_opt {
            // TODO: Call actual mesh slicer
            // let slices = slice_mesh_ex(mesh, slicegrid, params, throw_on_cancel);
            let mut slices: Vec<ExPolygons> = vec![Vec::new(); grid_size];

            collect_nonempty_indices(op, slicegrid, &slices, &mut nonempty_indices);

            let top = opstack.last_mut().unwrap();
            for &i in &nonempty_indices {
                merge_slices(op, i, &mut top.slices, &mut slices);
            }
        }

        // Handle Pop: complete the sub-expression
        // SliceCSGMesh.hpp:96-109
        if get_stack_operation(part) == CSGStackOp::Pop {
            let popped = opstack.pop().unwrap();
            let mut popslices = popped.slices;
            let popop = popped.op;

            collect_nonempty_indices(popop, slicegrid, &popslices, &mut nonempty_indices);

            let prev = opstack.last_mut().unwrap();
            for &i in &nonempty_indices {
                merge_slices(popop, i, &mut prev.slices, &mut popslices);
            }
        }
    }

    // Extract final result
    // SliceCSGMesh.hpp:112
    let mut result = opstack.pop().map(|f| f.slices).unwrap_or_default();

    // Clean up tiny polygons and union each layer
    // SliceCSGMesh.hpp:115-124
    for slice in &mut result {
        slice.retain(|p: &ExPolygon| {
            // Remove very small polygons
            // SliceCSGMesh.hpp:116-118
            p.area().abs() >= 1e-6 // SCALED_EPSILON^2 equivalent
        });
        // TODO: slice = union_ex(slice) when clipper union is available
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_collect_nonempty_indices_union() {
        let slicegrid = vec![0.0f32, 1.0, 2.0];
        let slices: Vec<ExPolygons> = vec![vec![], vec![ExPolygon::default()], vec![]];
        let mut indices = Vec::new();

        collect_nonempty_indices(CSGType::Union, &slicegrid, &slices, &mut indices);
        assert_eq!(indices, vec![1]);
    }

    #[test]
    fn test_collect_nonempty_indices_intersection() {
        let slicegrid = vec![0.0f32, 1.0, 2.0];
        let slices: Vec<ExPolygons> = vec![vec![], vec![], vec![]];
        let mut indices = Vec::new();

        collect_nonempty_indices(CSGType::Intersection, &slicegrid, &slices, &mut indices);
        assert_eq!(indices, vec![0, 1, 2]); // All indices for intersection
    }

    #[test]
    fn test_slice_csgmesh_ex_empty() {
        let parts: Vec<CSGPart> = vec![];
        let slicegrid = vec![0.0f32, 1.0, 2.0];
        let result = slice_csgmesh_ex(&parts, &slicegrid);
        assert_eq!(result.len(), 3);
    }

    #[test]
    fn test_slice_csgmesh_ex_with_parts() {
        let part = CSGPart::new();
        let slicegrid = vec![0.0f32, 0.5, 1.0];
        let result = slice_csgmesh_ex(&[part], &slicegrid);
        assert_eq!(result.len(), 3);
    }

    #[test]
    fn test_slice_csgmesh_ex_with_stack() {
        let cube1 = CSGPart::new()
            .with_operation(CSGType::Union)
            .with_stack_operation(CSGStackOp::Continue);
        let cube2 = CSGPart::new()
            .with_operation(CSGType::Difference)
            .with_stack_operation(CSGStackOp::Push);
        let cube3 = CSGPart::new()
            .with_operation(CSGType::Union)
            .with_stack_operation(CSGStackOp::Pop);

        let slicegrid = vec![0.0f32, 0.5, 1.0];
        let result = slice_csgmesh_ex(&[cube1, cube2, cube3], &slicegrid);
        assert_eq!(result.len(), 3);
    }
}
