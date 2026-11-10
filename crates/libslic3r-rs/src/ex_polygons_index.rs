//! Index into ExPolygons.
//!
//! C++ Reference:
//! - ExPolygonsIndex.hpp
//! - ExPolygonsIndex.cpp
//!
//! Faithful 1:1 line-by-line port of BambuStudio's ExPolygonsIndex.{hpp,cpp}.

use crate::geometry::ExPolygons;

/// Index into ExPolygons
/// Identify expolygon, its contour (or hole) and point
/// ExPolygonsIndex.hpp:11
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ExPolygonsIndex {
    // index of ExPolygons
    // ExPolygonsIndex.hpp:14
    pub expolygons_index: u32,

    // index of Polygon
    // 0 .. contour
    // N .. hole[N-1]
    // ExPolygonsIndex.hpp:19
    pub polygon_index: u32,

    // index of point in polygon
    // ExPolygonsIndex.hpp:22
    pub point_index: u32,
}

impl ExPolygonsIndex {
    // ExPolygonsIndex.hpp:24
    pub fn is_contour(&self) -> bool {
        self.polygon_index == 0
    }
    // ExPolygonsIndex.hpp:25
    pub fn is_hole(&self) -> bool {
        self.polygon_index != 0
    }
    // ExPolygonsIndex.hpp:26
    pub fn hole_index(&self) -> u32 {
        self.polygon_index - 1
    }
}

/// Keep conversion from ExPolygonsIndex to Index and vice versa
/// ExPolygonsIndex .. contour(or hole) point from ExPolygons
/// Index           .. continous number
///
/// index is used to address lines and points as result from function
/// Slic3r::to_lines, Slic3r::to_points
/// ExPolygonsIndex.hpp:37
pub struct ExPolygonsIndices {
    // ExPolygonsIndex.hpp:39
    m_offsets: Vec<Vec<u32>>,
    // for check range of index
    // ExPolygonsIndex.hpp:41
    m_count: u32, // count of points
}

impl ExPolygonsIndices {
    // IMPROVE: use one dimensional vector for polygons offset with searching by std::lower_bound
    // ExPolygonsIndex.cpp:5
    pub fn new(shapes: &ExPolygons) -> Self {
        // prepare offsets
        // ExPolygonsIndex.cpp:8
        let mut m_offsets: Vec<Vec<u32>> = Vec::with_capacity(shapes.len());
        let mut offset: u32 = 0;
        // ExPolygonsIndex.cpp:10
        for shape in shapes {
            debug_assert!(!shape.contour.points.is_empty()); // ExPolygonsIndex.cpp:11
            let mut shape_offsets: Vec<u32> = Vec::with_capacity(shape.holes.len() + 1); // ExPolygonsIndex.cpp:12-13
            shape_offsets.push(offset); // ExPolygonsIndex.cpp:14
            offset += shape.contour.points.len() as u32; // ExPolygonsIndex.cpp:15
            // ExPolygonsIndex.cpp:16
            for hole in &shape.holes {
                shape_offsets.push(offset); // ExPolygonsIndex.cpp:17
                offset += hole.points.len() as u32; // ExPolygonsIndex.cpp:18
            }
            m_offsets.push(shape_offsets); // ExPolygonsIndex.cpp:20
        }
        // ExPolygonsIndex.cpp:22
        Self {
            m_offsets,
            m_count: offset,
        }
    }

    /// Convert to one index number
    /// id: Compose of adress into expolygons
    /// returns: Index
    /// ExPolygonsIndex.cpp:25
    pub fn cvt(&self, id: &ExPolygonsIndex) -> u32 {
        debug_assert!((id.expolygons_index as usize) < self.m_offsets.len()); // ExPolygonsIndex.cpp:27
        let shape_offset = &self.m_offsets[id.expolygons_index as usize]; // ExPolygonsIndex.cpp:28
        debug_assert!((id.polygon_index as usize) < shape_offset.len()); // ExPolygonsIndex.cpp:29
        let res = shape_offset[id.polygon_index as usize] + id.point_index; // ExPolygonsIndex.cpp:30
        debug_assert!(res < self.m_count); // ExPolygonsIndex.cpp:31
        res // ExPolygonsIndex.cpp:32
    }

    /// Separate to multi index
    /// index: adress into expolygons
    /// ExPolygonsIndex.cpp:35
    pub fn cvt_index(&self, index: u32) -> ExPolygonsIndex {
        debug_assert!(index < self.m_count); // ExPolygonsIndex.cpp:37
        let mut result = ExPolygonsIndex {
            expolygons_index: 0,
            polygon_index: 0,
            point_index: 0,
        }; // ExPolygonsIndex.cpp:38
        // find expolygon index
        // ExPolygonsIndex.cpp:40
        // auto fn = [](const std::vector<uint32_t> &offsets, uint32_t index) { return offsets[0] < index; };
        // std::lower_bound on [m_offsets.begin() + 1, m_offsets.end()) with comparator fn:
        //   returns first iterator `it` in the range such that !(offsets[0] < index), i.e. offsets[0] >= index.
        // ExPolygonsIndex.cpp:41
        let it = lower_bound_by(&self.m_offsets, 1, self.m_offsets.len(), |offsets| {
            offsets[0] < index
        });
        result.expolygons_index = it as u32; // ExPolygonsIndex.cpp:42
        // ExPolygonsIndex.cpp:43
        if it == self.m_offsets.len() || self.m_offsets[it][0] != index {
            result.expolygons_index -= 1;
        }

        // find polygon index
        // ExPolygonsIndex.cpp:46
        let shape_offset = &self.m_offsets[result.expolygons_index as usize];
        // ExPolygonsIndex.cpp:47
        // std::lower_bound on [shape_offset.begin() + 1, shape_offset.end()) for `index`:
        //   returns first iterator `it2` such that !(*it2 < index), i.e. *it2 >= index.
        let it2 = lower_bound_value(shape_offset, 1, shape_offset.len(), index);
        result.polygon_index = it2 as u32; // ExPolygonsIndex.cpp:48
        // ExPolygonsIndex.cpp:49
        if it2 == shape_offset.len() || shape_offset[it2] != index {
            result.polygon_index -= 1;
        }

        // calculate point index
        // ExPolygonsIndex.cpp:52
        let polygon_offset = shape_offset[result.polygon_index as usize];
        debug_assert!(index >= polygon_offset); // ExPolygonsIndex.cpp:53
        result.point_index = index - polygon_offset; // ExPolygonsIndex.cpp:54
        result // ExPolygonsIndex.cpp:55
    }

    /// Check whether id is last point in polygon
    /// id: Identify point in expolygon
    /// returns: True when id is last point in polygon otherwise false
    /// ExPolygonsIndex.cpp:58
    pub fn is_last_point(&self, id: &ExPolygonsIndex) -> bool {
        debug_assert!((id.expolygons_index as usize) < self.m_offsets.len()); // ExPolygonsIndex.cpp:59
        let shape_offset = &self.m_offsets[id.expolygons_index as usize]; // ExPolygonsIndex.cpp:60
        debug_assert!((id.polygon_index as usize) < shape_offset.len()); // ExPolygonsIndex.cpp:61
        let index = shape_offset[id.polygon_index as usize] + id.point_index; // ExPolygonsIndex.cpp:62
        debug_assert!(index < self.m_count); // ExPolygonsIndex.cpp:63
        // next index
        // ExPolygonsIndex.cpp:65
        let next_point_index = index + 1;
        let next_poly_index = id.polygon_index + 1; // ExPolygonsIndex.cpp:66
        let next_expoly_index = id.expolygons_index + 1; // ExPolygonsIndex.cpp:67
        // is last expoly?
        // ExPolygonsIndex.cpp:69
        if next_expoly_index as usize == self.m_offsets.len() {
            // is last expoly last poly?
            // ExPolygonsIndex.cpp:71
            if next_poly_index as usize == shape_offset.len() {
                return next_point_index == self.m_count; // ExPolygonsIndex.cpp:72
            }
        } else {
            // (not last expoly) is expoly last poly?
            // ExPolygonsIndex.cpp:75
            if next_poly_index as usize == shape_offset.len() {
                return next_point_index == self.m_offsets[next_expoly_index as usize][0]; // ExPolygonsIndex.cpp:76
            }
        }
        // Not last polygon in expolygon
        // ExPolygonsIndex.cpp:79
        next_point_index == shape_offset[next_poly_index as usize]
    }

    /// Count of points in expolygons
    /// returns: Count of points in expolygons
    /// ExPolygonsIndex.cpp:82
    pub fn get_count(&self) -> u32 {
        self.m_count
    }
}

/// Equivalent of `std::lower_bound(first, last, value, comp)` operating on a slice
/// of `Vec<u32>` rows. Searches the half-open range `[lo, hi)` of `data`.
/// `comp(row)` plays the role of `comp(*it, value)` (returns true while the element
/// is ordered *before* the search value). Returns the index of the first element for
/// which `comp` is false, or `hi` if none.
fn lower_bound_by<F>(data: &[Vec<u32>], lo: usize, hi: usize, comp: F) -> usize
where
    F: Fn(&[u32]) -> bool,
{
    let mut first = lo;
    let mut count = hi - lo;
    while count > 0 {
        let step = count / 2;
        let it = first + step;
        if comp(&data[it]) {
            first = it + 1;
            count -= step + 1;
        } else {
            count = step;
        }
    }
    first
}

/// Equivalent of `std::lower_bound(first, last, value)` (default `operator<`) over the
/// half-open range `[lo, hi)` of a `&[u32]`. Returns the index of the first element
/// that is not less than `value`, or `hi` if none.
fn lower_bound_value(data: &[u32], lo: usize, hi: usize, value: u32) -> usize {
    let mut first = lo;
    let mut count = hi - lo;
    while count > 0 {
        let step = count / 2;
        let it = first + step;
        if data[it] < value {
            first = it + 1;
            count -= step + 1;
        } else {
            count = step;
        }
    }
    first
}
