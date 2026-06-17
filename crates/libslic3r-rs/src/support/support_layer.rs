//! Support layer types.
//!
//! C++ Reference:
//! - Support/SupportLayer.hpp
//!
//! Faithful 1:1 line-by-line port of `Support/SupportLayer.hpp`. Defines the
//! support-layer type enum and the `SupportGeneratorLayer` struct used
//! internally by the SupportMaterial class. These carry much more detailed
//! information than the final support layers stored in a PrintObject (mainly
//! the bridging flow and the interface gaps between the object and support).
//! This is from the old "MyLayer".
//!
//! coord_t -> i64, coordf_t -> f64 per crate conventions.

// SupportLayer.hpp:3-9 (includes)
// oneapi/tbb/scalable_allocator.h, oneapi/tbb/spin_mutex.h, PrintConfig.hpp,
// Slicing.hpp, Fill/FillBase.hpp, ClipperUtils.hpp, Polygon.hpp
use crate::clipper_utils::union_polygons_ex;
use crate::geometry::{to_polygons, Polygons};

// SupportLayer.hpp:11 namespace Slic3r {

// SupportLayer.hpp:13-15
// class PrintObject;
// class PrintConfig;
// class PrintObjectConfig;

// SupportLayer.hpp:17-18
// Support layer type to be used by MyLayer. This type carries a much more detailed information
// about the support layer type than the final support layers stored in a PrintObject.
// SupportLayer.hpp:19 enum SupporLayerType {
//
// NOTE: the C++ enumerator name `SupporLayerType` (with the single 'r') is
// preserved verbatim for byte-exact parity tracking against the upstream
// source.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SupporLayerType {
    // SupportLayer.hpp:20
    SltUnknown = 0,
    // SupportLayer.hpp:21-22
    // Ratft base layer, to be printed with the support material.
    SltRaftBase,
    // SupportLayer.hpp:23-24
    // Raft interface layer, to be printed with the support interface material.
    SltRaftInterface,
    // SupportLayer.hpp:25-26
    // Bottom contact layer placed over a top surface of an object. To be printed with a support interface material.
    SltBottomContact,
    // SupportLayer.hpp:27-29
    // Dense interface layer, to be printed with the support interface material.
    // This layer is separated from an object by an sltBottomContact layer.
    SltBottomInterface,
    // SupportLayer.hpp:30-31
    // Sparse base support layer, to be printed with a support material.
    SltBase,
    // SupportLayer.hpp:32-34
    // Dense interface layer, to be printed with the support interface material.
    // This layer is separated from an object with sltTopContact layer.
    SltTopInterface,
    // SupportLayer.hpp:35-36
    // Top contact layer directly supporting an overhang. To be printed with a support interface material.
    SltTopContact,
    // SupportLayer.hpp:37-38
    // Some undecided type yet. It will turn into sltBase first, then it may turn into sltBottomInterface or sltTopInterface.
    SltIntermediate,
}

impl Default for SupporLayerType {
    fn default() -> Self {
        // SupportLayer.hpp:97 SupporLayerType layer_type{ SupporLayerType::sltUnknown };
        Self::SltUnknown
    }
}

// SupportLayer.hpp:41-44
// A support layer type used internally by the SupportMaterial class. This class carries a much more detailed
// information about the support layer than the layers stored in the PrintObject, mainly
// the SupportGeneratorLayer is aware of the bridging flow and the interface gaps between the object and the support.
// This is from the old "MyLayer".
// SupportLayer.hpp:45 class SupportGeneratorLayer
#[derive(Debug, Clone)]
pub struct SupportGeneratorLayer {
    // SupportLayer.hpp:97 SupporLayerType layer_type{ SupporLayerType::sltUnknown };
    pub layer_type: SupporLayerType,
    // SupportLayer.hpp:98-99
    // Z used for printing, in unscaled coordinates.
    pub print_z: f64,
    // SupportLayer.hpp:100-102
    // Bottom Z of this layer. For soluble layers, bottom_z + height = print_z,
    // otherwise bottom_z + gap + height = print_z.
    pub bottom_z: f64,
    // SupportLayer.hpp:103-104
    // Layer height in unscaled coordinates.
    pub height: f64,
    // SupportLayer.hpp:105-107
    // Index of a PrintObject layer_id supported by this layer. This will be set for top contact layers.
    // If this is not a contact layer, it will be set to size_t(-1).
    pub idx_object_layer_above: usize,
    // SupportLayer.hpp:108-110
    // Index of a PrintObject layer_id, which supports this layer. This will be set for bottom contact layers.
    // If this is not a contact layer, it will be set to size_t(-1).
    pub idx_object_layer_below: usize,
    // SupportLayer.hpp:111-112
    // Use a bridging flow when printing this support layer.
    pub bridging: bool,
    // SupportLayer.hpp:113-114
    //order of the transition layers
    pub up: bool,

    // SupportLayer.hpp:116-117
    // Polygons to be filled by the support pattern.
    pub polygons: Polygons,
    // SupportLayer.hpp:118-119
    // Currently for the contact layers only.
    pub contact_polygons: Option<Polygons>,
    // SupportLayer.hpp:120
    pub overhang_polygons: Option<Polygons>,
    // SupportLayer.hpp:121-122
    // Enforcers need to be propagated independently in case the "support on build plate only" option is enabled.
    pub enforcer_polygons: Option<Polygons>,
}

impl Default for SupportGeneratorLayer {
    fn default() -> Self {
        Self {
            // SupportLayer.hpp:97
            layer_type: SupporLayerType::SltUnknown,
            // SupportLayer.hpp:99
            print_z: 0.0,
            // SupportLayer.hpp:102
            bottom_z: 0.0,
            // SupportLayer.hpp:104
            height: 0.0,
            // SupportLayer.hpp:107 size_t(-1)
            idx_object_layer_above: usize::MAX,
            // SupportLayer.hpp:110 size_t(-1)
            idx_object_layer_below: usize::MAX,
            // SupportLayer.hpp:112
            bridging: false,
            // SupportLayer.hpp:114
            up: false,
            // SupportLayer.hpp:117
            polygons: Polygons::new(),
            // SupportLayer.hpp:119
            contact_polygons: None,
            // SupportLayer.hpp:120
            overhang_polygons: None,
            // SupportLayer.hpp:122
            enforcer_polygons: None,
        }
    }
}

impl SupportGeneratorLayer {
    pub fn new() -> Self {
        Self::default()
    }

    // SupportLayer.hpp:48-50
    pub fn reset(&mut self) {
        *self = SupportGeneratorLayer::default();
    }

    // SupportLayer.hpp:52-54
    // bool operator==(const SupportGeneratorLayer& layer2) const {
    //     return print_z == layer2.print_z && height == layer2.height && bridging == layer2.bridging;
    // }
    //
    // Implemented as `PartialEq` below.

    // SupportLayer.hpp:56-73
    // Order the layers by lexicographically by an increasing print_z and a decreasing layer height.
    // bool operator<(const SupportGeneratorLayer& layer2) const { ... }
    //
    // Implemented as `Ord`/`PartialOrd` below.

    // SupportLayer.hpp:75-88
    // void merge(SupportGeneratorLayer&& rhs)
    pub fn merge(&mut self, mut rhs: SupportGeneratorLayer) {
        // SupportLayer.hpp:76-77
        // The union_() does not support move semantic yet, but maybe one day it will.
        // this->polygons = union_(this->polygons, std::move(rhs.polygons));
        self.polygons = union_polygons(&self.polygons, &rhs.polygons);
        // SupportLayer.hpp:78-83
        // auto merge = [](std::unique_ptr<Polygons>& dst, std::unique_ptr<Polygons>& src) {
        //     if (!dst || dst->empty())
        //         dst = std::move(src);
        //     else if (src && !src->empty())
        //         *dst = union_(*dst, std::move(*src));
        //     };
        let merge = |dst: &mut Option<Polygons>, src: &mut Option<Polygons>| {
            if dst.is_none() || dst.as_ref().unwrap().is_empty() {
                *dst = src.take();
            } else if src.as_ref().map(|s| !s.is_empty()).unwrap_or(false) {
                let new_dst = union_polygons(dst.as_ref().unwrap(), src.as_ref().unwrap());
                *dst.as_mut().unwrap() = new_dst;
            }
        };
        // SupportLayer.hpp:84
        merge(&mut self.contact_polygons, &mut rhs.contact_polygons);
        // SupportLayer.hpp:85
        merge(&mut self.overhang_polygons, &mut rhs.overhang_polygons);
        // SupportLayer.hpp:86
        merge(&mut self.enforcer_polygons, &mut rhs.enforcer_polygons);
        // SupportLayer.hpp:87
        rhs.reset();
    }

    // SupportLayer.hpp:90-92
    // For the bridging flow, bottom_print_z will be above bottom_z to account for the vertical separation.
    // For the non-bridging flow, bottom_print_z will be equal to bottom_z.
    // coordf_t bottom_print_z() const { return print_z - height; }
    pub fn bottom_print_z(&self) -> f64 {
        self.print_z - self.height
    }

    // SupportLayer.hpp:94-95
    // To sort the extremes of top / bottom interface layers.
    // coordf_t extreme_z() const { return (this->layer_type == SupporLayerType::sltTopContact) ? this->bottom_z : this->print_z; }
    pub fn extreme_z(&self) -> f64 {
        if self.layer_type == SupporLayerType::SltTopContact {
            self.bottom_z
        } else {
            self.print_z
        }
    }
}

// SupportLayer.hpp:52-54 bool operator==
impl PartialEq for SupportGeneratorLayer {
    fn eq(&self, layer2: &Self) -> bool {
        self.print_z == layer2.print_z
            && self.height == layer2.height
            && self.bridging == layer2.bridging
    }
}

impl Eq for SupportGeneratorLayer {}

impl SupportGeneratorLayer {
    // SupportLayer.hpp:56-73
    // Order the layers by lexicographically by an increasing print_z and a decreasing layer height.
    // bool operator<(const SupportGeneratorLayer& layer2) const { ... }
    //
    // Faithful translation of the C++ `operator<` returning the same bool. This is
    // a strict weak ordering; the `Ord`/`PartialOrd` impls below derive a total
    // order from it in the standard way (Less if a<b, Greater if b<a, else Equal)
    // so `std::sort`/`std::set` parity holds without altering the comparison.
    fn lt(&self, layer2: &Self) -> bool {
        // SupportLayer.hpp:58-60
        // if (print_z < layer2.print_z) { return true; }
        if self.print_z < layer2.print_z {
            true
        }
        // SupportLayer.hpp:61
        // else if (print_z == layer2.print_z) {
        else if self.print_z == layer2.print_z {
            // SupportLayer.hpp:62-63
            // if (height > layer2.height) return true;
            if self.height > layer2.height {
                true
            }
            // SupportLayer.hpp:64
            // else if (height == layer2.height) {
            else if self.height == layer2.height {
                // SupportLayer.hpp:65-66
                // Bridging layers first.
                // return bridging && !layer2.bridging;
                self.bridging && !layer2.bridging
            }
            // SupportLayer.hpp:68-69
            // else return false;
            else {
                false
            }
        }
        // SupportLayer.hpp:71-72
        // else return false;
        else {
            false
        }
    }
}

// SupportLayer.hpp:56-73 bool operator<
impl PartialOrd for SupportGeneratorLayer {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for SupportGeneratorLayer {
    // SupportLayer.hpp:57-73
    // Derive a total ordering from the C++ strict-weak `operator<` (`lt`).
    fn cmp(&self, layer2: &Self) -> std::cmp::Ordering {
        use std::cmp::Ordering;
        if self.lt(layer2) {
            Ordering::Less
        } else if layer2.lt(self) {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    }
}

// SupportLayer.hpp:75-83
// The C++ merge() uses ClipperUtils::union_(Polygons, Polygons) which concatenates
// the two polygon sets and then runs a Clipper union, returning the (flattened)
// result as Polygons (contours + holes as separate Polygon entries).
//
// ClipperUtils.cpp:727-734
//   Slic3r::Polygons union_(const Slic3r::Polygons &subject, const Slic3r::Polygons &subject2)
//   {
//       Polygons polys = subject;
//       for (const Polygon& poly : subject2)
//           polys.push_back(poly);
//       return union_(polys);
//   }
//
// The crate exposes `union_polygons_ex(&[Polygon]) -> ExPolygons`; re-flattening
// via `to_polygons` reproduces the Polygons-returning `union_` semantics.
//
// FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib. The underlying
// `union_polygons_ex` routes through the `geo` crate (geo-clipper, fixed scale
// 1000) rather than ClipperLib at coord_t integer precision, so the union result
// may differ at the sub-scale level from upstream.
fn union_polygons(subject: &Polygons, subject2: &Polygons) -> Polygons {
    let mut polys = subject.clone();
    for poly in subject2 {
        polys.push(poly.clone());
    }
    to_polygons(&union_polygons_ex(&polys))
}

// SupportLayer.hpp:125-127
// Layers are allocated and owned by a deque. Once a layer is allocated, it is maintained
// up to the end of a generate() method. The layer storage may be replaced by an allocator class in the future,
// which would allocate layers by multiple chunks.
// SupportLayer.hpp:128 class SupportGeneratorLayerStorage {
//
// In C++ the storage is a `Slic3r::deque` guarded by a `tbb::spin_mutex` so that
// references handed out by `allocate()` stay valid as more layers are appended.
// In the wasm-safe Rust port we back the storage with a boxed deque so element
// addresses stay stable; access is single-threaded here so `allocate` and
// `allocate_unguarded` are equivalent (the TBB spin_mutex is not portable and
// is not needed without the parallel allocation path).
#[derive(Debug, Default)]
pub struct SupportGeneratorLayerStorage {
    // SupportLayer.hpp:147-149
    // template<typename BaseType> using Allocator = tbb::scalable_allocator<BaseType>;
    // Slic3r::deque<SupportGeneratorLayer, Allocator<SupportGeneratorLayer>> m_storage;
    m_storage: std::collections::VecDeque<Box<SupportGeneratorLayer>>,
    // SupportLayer.hpp:150 tbb::spin_mutex m_mutex; (not portable; single-threaded port)
}

impl SupportGeneratorLayerStorage {
    pub fn new() -> Self {
        Self::default()
    }

    // SupportLayer.hpp:130-134
    // SupportGeneratorLayer& allocate_unguarded(SupporLayerType layer_type) {
    //     m_storage.emplace_back();
    //     m_storage.back().layer_type = layer_type;
    //     return m_storage.back();
    // }
    pub fn allocate_unguarded(&mut self, layer_type: SupporLayerType) -> &mut SupportGeneratorLayer {
        self.m_storage.push_back(Box::new(SupportGeneratorLayer::default()));
        let layer = self.m_storage.back_mut().unwrap();
        layer.layer_type = layer_type;
        layer
    }

    // SupportLayer.hpp:136-144
    // SupportGeneratorLayer& allocate(SupporLayerType layer_type)
    // {
    //     m_mutex.lock();
    //     m_storage.emplace_back();
    //     SupportGeneratorLayer *layer_new = &m_storage.back();
    //     m_mutex.unlock();
    //     layer_new->layer_type = layer_type;
    //     return *layer_new;
    // }
    pub fn allocate(&mut self, layer_type: SupporLayerType) -> &mut SupportGeneratorLayer {
        // m_mutex.lock(); (not portable; single-threaded port)
        self.m_storage.push_back(Box::new(SupportGeneratorLayer::default()));
        let layer_new = self.m_storage.back_mut().unwrap();
        // m_mutex.unlock();
        layer_new.layer_type = layer_type;
        layer_new
    }

    // Number of layers currently allocated in the deque. Used by the index-based
    // modeling of `SupportGeneratorLayer*` pointers (see SupportGeneratorLayersPtr):
    // the layer-allocation helpers hand back `len() - 1` as the "pointer".
    pub fn len(&self) -> usize {
        self.m_storage.len()
    }

    pub fn is_empty(&self) -> bool {
        self.m_storage.is_empty()
    }
}

// Index access into the storage by the integer "pointer" used in
// `SupportGeneratorLayersPtr`. In C++ these are raw `SupportGeneratorLayer*`;
// the Rust port stores `usize` indices into the deque and dereferences them here.
impl std::ops::Index<usize> for SupportGeneratorLayerStorage {
    type Output = SupportGeneratorLayer;
    fn index(&self, idx: usize) -> &SupportGeneratorLayer {
        &self.m_storage[idx]
    }
}

impl std::ops::IndexMut<usize> for SupportGeneratorLayerStorage {
    fn index_mut(&mut self, idx: usize) -> &mut SupportGeneratorLayer {
        &mut self.m_storage[idx]
    }
}

// SupportLayer.hpp:152
// using SupportGeneratorLayersPtr = std::vector<SupportGeneratorLayer*>;
//
// In Rust the raw-pointer-vector of `SupportGeneratorLayer*` is represented as a
// `Vec` of indices into a `SupportGeneratorLayerStorage` at the call sites; the
// type alias is provided for parity with downstream signatures that pass these
// around. (Direct `&mut` pointers cannot be aliased in a Vec.)
pub type SupportGeneratorLayersPtr = Vec<usize>;

// SupportLayer.hpp:153 } // namespace Slic3r
