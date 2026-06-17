//! CSG mesh slicing.
//!
//! Faithful 1:1 port of BambuStudio `src/libslic3r/CSGMesh/SliceCSGMesh.hpp`.
//! Header-only file; every definition lives in the header and is reproduced
//! here. Line references point into `SliceCSGMesh.hpp`.
//!
//! Provides functions for slicing CSG mesh collections into 2D polygon layers.
//! The slicing respects CSG operations (union, difference, intersection) and
//! stack-based sub-expression evaluation.

use super::csg_mesh::{
    get_mesh, get_operation, get_stack_operation, get_transform, CSGPart, CSGStackOp, CSGType,
};
use crate::geometry::{ExPolygon, Transform3D};
use crate::libslic3r::SCALED_EPSILON;
use crate::triangle_mesh_slicer::{slice_mesh_ex, MeshSlicingParamsEx};

/// Type alias for a vector of ExPolygons (one layer of sliced polygons).
pub type ExPolygons = Vec<ExPolygon>;

// SliceCSGMesh.hpp:14  namespace detail {

/// Merge slices according to CSG operation.
///
/// SliceCSGMesh.hpp:16-32
/// `inline void merge_slices(csg::CSGType op, size_t i,
///                           std::vector<ExPolygons> &target,
///                           std::vector<ExPolygons> &source)`
pub fn merge_slices(
    op: CSGType,
    i: usize,
    target: &mut [ExPolygons],
    source: &mut [ExPolygons],
) {
    // SliceCSGMesh.hpp:20  switch(op) {
    match op {
        // SliceCSGMesh.hpp:21-24
        // case CSGType::Union:
        //     for (ExPolygon &expoly : source[i])
        //         target[i].emplace_back(std::move(expoly));
        CSGType::Union => {
            let mut src_polys = std::mem::take(&mut source[i]);
            target[i].append(&mut src_polys);
        }
        // SliceCSGMesh.hpp:25-26
        // case CSGType::Difference:
        //     target[i] = diff_ex(target[i], source[i]);
        CSGType::Difference => {
            target[i] = crate::clipper_utils::difference(&target[i], &source[i]);
        }
        // SliceCSGMesh.hpp:28-29
        // case CSGType::Intersection:
        //     target[i] = intersection_ex(target[i], source[i]);
        CSGType::Intersection => {
            target[i] = crate::clipper_utils::intersection(&target[i], &source[i]);
        }
    }
}

/// Collect indices of non-empty slices (or all for intersection).
///
/// SliceCSGMesh.hpp:34-44
/// `inline void collect_nonempty_indices(csg::CSGType op,
///                                        const std::vector<float> &slicegrid,
///                                        const std::vector<ExPolygons> &slices,
///                                        std::vector<size_t> &indices)`
pub fn collect_nonempty_indices(
    op: CSGType,
    slicegrid: &[f32],
    slices: &[ExPolygons],
    indices: &mut Vec<usize>,
) {
    // SliceCSGMesh.hpp:39  indices.clear();
    indices.clear();
    // SliceCSGMesh.hpp:40  for (size_t i = 0; i < slicegrid.size(); ++i) {
    for i in 0..slicegrid.len() {
        // SliceCSGMesh.hpp:41-42
        // if (op == CSGType::Intersection || !slices[i].empty())
        //     indices.emplace_back(i);
        if op == CSGType::Intersection || !slices[i].is_empty() {
            indices.push(i);
        }
    }
}

// SliceCSGMesh.hpp:46  } // namespace detail

/// Stack frame for CSG slice expression evaluation.
///
/// SliceCSGMesh.hpp:57  `struct Frame { CSGType op; std::vector<ExPolygons> slices; };`
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
/// `template<class ItCSG> std::vector<ExPolygons> slice_csgmesh_ex(
///      const Range<ItCSG> &csgrange,
///      const std::vector<float> &slicegrid,
///      const MeshSlicingParamsEx &params,
///      const std::function<void()> &throw_on_cancel = [] {})`
pub fn slice_csgmesh_ex(
    csgrange: &[CSGPart],
    slicegrid: &[f32],
    params: &MeshSlicingParamsEx,
    throw_on_cancel: &dyn Fn(),
) -> Vec<ExPolygons> {
    // SliceCSGMesh.hpp:59  std::stack opstack{std::vector<Frame>{}};
    let mut opstack: Vec<Frame> = Vec::new();

    // SliceCSGMesh.hpp:61  MeshSlicingParamsEx params_cpy = params;
    // (C++ mutates `params_cpy.trafo` per-part; this crate's `MeshSlicingParamsEx`
    //  carries no `trafo`, so the copy is effectively immutable here.)
    let params_cpy = params.clone();
    // SliceCSGMesh.hpp:62  auto trafo = params.trafo;
    //
    // FIDELITY-NOTE: `MeshSlicingParamsEx` in this crate does not carry a
    // `Transform3d trafo` (the slicer applies identity only). C++ stores
    // `params.trafo` here and composes per-part transforms below. We reproduce
    // the composition arithmetic faithfully so the control flow matches, but
    // `slice_mesh_ex` ignores the composed trafo (identity-trafo slicer). The
    // base/initial trafo is therefore identity.
    let trafo = Transform3D::identity();
    // SliceCSGMesh.hpp:63  auto nonempty_indices = reserve_vector<size_t>(slicegrid.size());
    let mut nonempty_indices: Vec<usize> = Vec::with_capacity(slicegrid.len());

    // SliceCSGMesh.hpp:65
    // opstack.push({CSGType::Union, std::vector<ExPolygons>(slicegrid.size())});
    opstack.push(Frame {
        op: CSGType::Union,
        slices: vec![ExPolygons::new(); slicegrid.len()],
    });

    // SliceCSGMesh.hpp:67  for (const auto &csgpart : csgrange) {
    for csgpart in csgrange {
        // SliceCSGMesh.hpp:68  const indexed_triangle_set *its = csg::get_mesh(csgpart);
        let its = get_mesh(csgpart);

        // SliceCSGMesh.hpp:70  auto op = get_operation(csgpart);
        let mut op = get_operation(csgpart);

        // SliceCSGMesh.hpp:72  if (get_stack_operation(csgpart) == CSGStackOp::Push) {
        if get_stack_operation(csgpart) == CSGStackOp::Push {
            // SliceCSGMesh.hpp:73
            // opstack.push({op, std::vector<ExPolygons>(slicegrid.size())});
            opstack.push(Frame {
                op,
                slices: vec![ExPolygons::new(); slicegrid.len()],
            });
            // SliceCSGMesh.hpp:74  op = CSGType::Union;
            op = CSGType::Union;
        }

        // SliceCSGMesh.hpp:77  Frame *top = &opstack.top();
        // (Borrowed as the mutable last frame at the point of use below.)

        // SliceCSGMesh.hpp:79  if (its) {
        if let Some(mesh) = its {
            // SliceCSGMesh.hpp:80
            // params_cpy.trafo = trafo * csg::get_transform(csgpart).cast<double>();
            //
            // FIDELITY-NOTE: composed faithfully here, but `slice_mesh_ex`
            // (identity-trafo slicer) does not consume it.
            let _composed_trafo = trafo.then(&get_transform(csgpart));

            // SliceCSGMesh.hpp:81-83
            // std::vector<ExPolygons> slices = slice_mesh_ex(*its, slicegrid,
            //                                                 params_cpy,
            //                                                 throw_on_cancel);
            let mut slices: Vec<ExPolygons> =
                slice_mesh_ex(mesh, slicegrid, &params_cpy, throw_on_cancel);

            // SliceCSGMesh.hpp:85  assert(slices.size() == slicegrid.size());
            debug_assert_eq!(slices.len(), slicegrid.len());

            // SliceCSGMesh.hpp:87
            // collect_nonempty_indices(op, slicegrid, slices, nonempty_indices);
            collect_nonempty_indices(op, slicegrid, &slices, &mut nonempty_indices);

            // SliceCSGMesh.hpp:89-93  (execution::for_each(ex_tbb, ...) -> sequential)
            // merge_slices(op, i, top->slices, slices);
            let top = opstack.last_mut().unwrap();
            for &i in &nonempty_indices {
                merge_slices(op, i, &mut top.slices, &mut slices);
            }
        }

        // SliceCSGMesh.hpp:96  if (get_stack_operation(csgpart) == CSGStackOp::Pop) {
        if get_stack_operation(csgpart) == CSGStackOp::Pop {
            // SliceCSGMesh.hpp:97  std::vector<ExPolygons> popslices = std::move(top->slices);
            // SliceCSGMesh.hpp:98  auto popop = opstack.top().op;
            // SliceCSGMesh.hpp:99  opstack.pop();
            let popped = opstack.pop().unwrap();
            let mut popslices = popped.slices;
            let popop = popped.op;

            // SliceCSGMesh.hpp:100
            // std::vector<ExPolygons> &prev_slices = opstack.top().slices;
            // SliceCSGMesh.hpp:102
            // collect_nonempty_indices(popop, slicegrid, popslices, nonempty_indices);
            collect_nonempty_indices(popop, slicegrid, &popslices, &mut nonempty_indices);

            // SliceCSGMesh.hpp:104-108  (execution::for_each(ex_tbb, ...) -> sequential)
            // merge_slices(popop, i, prev_slices, popslices);
            let prev = opstack.last_mut().unwrap();
            for &i in &nonempty_indices {
                merge_slices(popop, i, &mut prev.slices, &mut popslices);
            }
        }
    }

    // SliceCSGMesh.hpp:112  std::vector<ExPolygons> ret = std::move(opstack.top().slices);
    let mut ret = opstack.pop().map(|f| f.slices).unwrap_or_default();

    // SliceCSGMesh.hpp:114  // TODO: verify if this part can be omitted or not.
    // SliceCSGMesh.hpp:115-124  (execution::for_each(ex_tbb, ...) -> sequential)
    for slice in &mut ret {
        // SliceCSGMesh.hpp:116-118
        // auto it = std::remove_if(slice.begin(), slice.end(), [](const ExPolygon &p){
        //     return p.area() < double(SCALED_EPSILON) * double(SCALED_EPSILON);
        // });
        // SliceCSGMesh.hpp:122  slice.erase(it, slice.end());
        //
        // FIDELITY-NOTE(F1): `ExPolygon::area()` here is the geo-clipper-backed
        // signed area (scaled coord_t units), matching the C++ `area()` units
        // so the SCALED_EPSILON^2 threshold compares like-for-like.
        slice.retain(|p: &ExPolygon| {
            !(p.area() < (SCALED_EPSILON as f64) * (SCALED_EPSILON as f64))
        });
        // SliceCSGMesh.hpp:123  slice = union_ex(slice);
        *slice = crate::clipper_utils::union_ex(slice);
    }

    // SliceCSGMesh.hpp:126  return ret;
    ret
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
        let params = MeshSlicingParamsEx::default();
        let result = slice_csgmesh_ex(&parts, &slicegrid, &params, &|| {});
        assert_eq!(result.len(), 3);
    }

    #[test]
    fn test_slice_csgmesh_ex_with_parts() {
        let part = CSGPart::new();
        let slicegrid = vec![0.0f32, 0.5, 1.0];
        let params = MeshSlicingParamsEx::default();
        let result = slice_csgmesh_ex(&[part], &slicegrid, &params, &|| {});
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
        let params = MeshSlicingParamsEx::default();
        let result = slice_csgmesh_ex(&[cube1, cube2, cube3], &slicegrid, &params, &|| {});
        assert_eq!(result.len(), 3);
    }
}
