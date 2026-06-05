//! Surface types for layer regions.
//!
//! This module provides the Surface type representing classified regions
//! within a layer, mirroring BambuStudio's Surface class.
//!
//! # Surface Type Detection
//!
//! Surface types are detected by comparing the current layer's geometry with
//! adjacent layers:
//!
//! - **Top**: Areas of the current layer not covered by the layer above
//! - **Bottom**: Areas of the current layer not supported by the layer below
//! - **BottomBridge**: Bottom areas that span over air (no support below)
//! - **Internal**: Areas covered both above and below (get sparse infill)
//! - **InternalSolid**: Internal areas that need solid infill (near top/bottom)
//!
//! # BambuStudio Reference
//!
//! This module corresponds to:
//! - `src/libslic3r/Surface.hpp/cpp`
//! - `PrintObject::detect_surfaces_type()` in `PrintObject.cpp`

use crate::clipper_utils::{
    difference, grow, intersection, opening, shrink, union_ex, OffsetJoinType,
};
use crate::geometry::{ExPolygon, ExPolygons};
use crate::CoordF;
use serde::{Deserialize, Serialize};
use std::fmt;

/// Classification of a surface within a layer.
///
/// Surfaces are classified to determine how they should be filled:
/// - Top/bottom surfaces get solid infill
/// - Internal surfaces get sparse infill
/// - Bridge surfaces need special handling
///
/// The variant ORDER must match the C++ `enum SurfaceType` exactly, because the
/// discriminant is round-tripped through `usize` in `LayerRegion` and used to
/// index `std::array<SurfacesPtr, size_t(stCount)>`.
/// Surface.hpp:9-30
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SurfaceType {
    // Top horizontal surface, visible from the top.
    /// Surface.hpp:11 stTop
    Top,
    // Bottom horizontal surface, visible from the bottom, printed with a normal extrusion flow.
    /// Surface.hpp:13 stBottom
    Bottom,
    // Bottom horizontal surface, visible from the bottom, unsupported, printed with a bridging extrusion flow.
    /// Surface.hpp:15 stBottomBridge
    BottomBridge,
    // Normal sparse infill.
    /// Surface.hpp:17 stInternal
    #[default]
    Internal,
    /// Surface.hpp:18 stFloatingVerticalShell
    FloatingVerticalShell,
    // Full infill, supporting the top surfaces and/or defining the verticall wall thickness.
    /// Surface.hpp:20 stInternalSolid
    InternalSolid,
    // 1st layer of dense infill over sparse infill, printed with a bridging extrusion flow.
    /// Surface.hpp:22 stInternalBridge
    InternalBridge,
    // stInternal turns into void surfaces if the sparse infill is used for supports only,
    // or if sparse infill layers get combined into a single layer.
    /// Surface.hpp:25 stInternalVoid
    InternalVoid,
    // Inner/outer perimeters.
    /// Surface.hpp:27 stPerimeter
    Perimeter,
    // Number of SurfaceType enums.
    // Surface.hpp:29 stCount
}

impl SurfaceType {
    /// Number of SurfaceType enums (C++ stCount).
    /// Surface.hpp:29
    pub const COUNT: usize = 9;

    /// Convert from u8 index (for array indexing from C++).
    /// Inverse of `self as u8`; ordering matches Surface.hpp:9-30.
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => SurfaceType::Top,
            1 => SurfaceType::Bottom,
            2 => SurfaceType::BottomBridge,
            3 => SurfaceType::Internal,
            4 => SurfaceType::FloatingVerticalShell,
            5 => SurfaceType::InternalSolid,
            6 => SurfaceType::InternalBridge,
            7 => SurfaceType::InternalVoid,
            8 => SurfaceType::Perimeter,
            _ => SurfaceType::Internal, // Default fallback
        }
    }
}

impl SurfaceType {
    // The following methods do not test for stPerimeter.
    // Surface.hpp:104

    // bool is_top() const { return this->surface_type == stTop; }
    /// Surface.hpp:105
    #[inline]
    pub fn is_top(&self) -> bool {
        matches!(self, SurfaceType::Top)
    }

    // bool is_bottom() const { return this->surface_type == stBottom || this->surface_type == stBottomBridge; }
    /// Surface.hpp:106
    #[inline]
    pub fn is_bottom(&self) -> bool {
        matches!(self, SurfaceType::Bottom | SurfaceType::BottomBridge)
    }

    // bool is_bridge() const { return this->surface_type == stBottomBridge || this->surface_type == stInternalBridge; }
    /// Surface.hpp:107
    #[inline]
    pub fn is_bridge(&self) -> bool {
        matches!(
            self,
            SurfaceType::BottomBridge | SurfaceType::InternalBridge
        )
    }

    // bool is_external() const { return this->is_top() || this->is_bottom(); }
    /// Surface.hpp:108
    #[inline]
    pub fn is_external(&self) -> bool {
        self.is_top() || self.is_bottom()
    }

    // bool is_internal() const { return ! this->is_external(); }
    /// Surface.hpp:109
    #[inline]
    pub fn is_internal(&self) -> bool {
        !self.is_external()
    }

    // bool is_floating_vertical_shell() const { return this->surface_type == stFloatingVerticalShell; }
    /// Surface.hpp:110
    #[inline]
    pub fn is_floating_vertical_shell(&self) -> bool {
        matches!(self, SurfaceType::FloatingVerticalShell)
    }

    // bool is_solid() const { return this->is_external() || this->is_floating_vertical_shell() || this->surface_type == stInternalSolid || this->surface_type == stInternalBridge; }
    /// Surface.hpp:111
    #[inline]
    pub fn is_solid(&self) -> bool {
        self.is_external()
            || self.is_floating_vertical_shell()
            || matches!(
                self,
                SurfaceType::InternalSolid | SurfaceType::InternalBridge
            )
    }

    // bool is_solid_infill() const { return this->surface_type == stInternalSolid; }
    /// Surface.hpp:112
    #[inline]
    pub fn is_solid_infill(&self) -> bool {
        matches!(self, SurfaceType::InternalSolid)
    }

    /// Get a human-readable name for this surface type.
    /// (Derived from `surface_type_to_color_name` labels in Surface.cpp.)
    pub fn name(&self) -> &'static str {
        match self {
            SurfaceType::Top => "top",
            SurfaceType::Bottom => "bottom",
            SurfaceType::BottomBridge => "bottom bridge",
            SurfaceType::Internal => "internal",
            SurfaceType::FloatingVerticalShell => "floating vertical shell",
            SurfaceType::InternalSolid => "internal solid",
            SurfaceType::InternalBridge => "internal bridge",
            SurfaceType::InternalVoid => "internal void",
            SurfaceType::Perimeter => "perimeter",
        }
    }
}

impl fmt::Display for SurfaceType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.name())
    }
}

/// A surface is a classified region within a layer.
///
/// Each surface has a type (determining how it should be filled)
/// and geometry (the ExPolygon defining its shape).
#[derive(Clone, Default, Serialize, Deserialize)]
pub struct Surface {
    /// The geometry of this surface.
    pub expolygon: ExPolygon,

    /// The type/classification of this surface.
    pub surface_type: SurfaceType,

    /// Thickness of this surface (layer height), in mm.
    pub thickness: CoordF,

    /// Thickness of the layer below, in mm (for bridge calculations).
    pub thickness_layers: usize,

    /// Bridge angle in radians (for bridge surfaces).
    /// None if not a bridge or angle not yet determined.
    pub bridge_angle: Option<CoordF>,

    /// Extra perimeters needed for this surface.
    pub extra_perimeters: usize,
}

impl Surface {
    // Create a new surface with the given type and geometry.
    pub fn new(surface_type: SurfaceType, expolygon: ExPolygon) -> Self {
        Self {
            expolygon,
            surface_type,
            thickness: 0.0,
            thickness_layers: 1,
            bridge_angle: None,
            extra_perimeters: 0,
        }
    }

    /// Create a new top surface.
    pub fn top(expolygon: ExPolygon) -> Self {
        Self::new(SurfaceType::Top, expolygon)
    }

    /// Create a new bottom surface.
    pub fn bottom(expolygon: ExPolygon) -> Self {
        Self::new(SurfaceType::Bottom, expolygon)
    }

    /// Create a new internal surface.
    pub fn internal(expolygon: ExPolygon) -> Self {
        Self::new(SurfaceType::Internal, expolygon)
    }

    /// Create a new internal solid surface.
    pub fn internal_solid(expolygon: ExPolygon) -> Self {
        Self::new(SurfaceType::InternalSolid, expolygon)
    }

    /// Create a new bridge surface.
    pub fn bridge(expolygon: ExPolygon, angle: Option<CoordF>) -> Self {
        Self {
            expolygon,
            surface_type: SurfaceType::BottomBridge,
            thickness: 0.0,
            thickness_layers: 1,
            bridge_angle: angle,
            extra_perimeters: 0,
        }
    }

    /// Check if this surface is empty (no geometry).
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.expolygon.is_empty()
    }

    /// Get the area of this surface.
    #[inline]
    pub fn area(&self) -> CoordF {
        self.expolygon.area()
    }

    /// Check if this is a top surface.
    #[inline]
    pub fn is_top(&self) -> bool {
        self.surface_type.is_top()
    }

    /// Check if this is a bottom surface.
    #[inline]
    pub fn is_bottom(&self) -> bool {
        self.surface_type.is_bottom()
    }

    /// Check if this is a bridge surface.
    #[inline]
    pub fn is_bridge(&self) -> bool {
        self.surface_type.is_bridge()
    }

    /// Check if this is a solid surface.
    #[inline]
    pub fn is_solid(&self) -> bool {
        self.surface_type.is_solid()
    }

    /// Check if this is an internal surface.
    #[inline]
    pub fn is_internal(&self) -> bool {
        self.surface_type.is_internal()
    }

    /// Check if this is an external surface.
    #[inline]
    pub fn is_external(&self) -> bool {
        self.surface_type.is_external()
    }

    // bool is_floating_vertical_shell() const ...
    /// Surface.hpp:110
    #[inline]
    pub fn is_floating_vertical_shell(&self) -> bool {
        self.surface_type.is_floating_vertical_shell()
    }

    // bool is_solid_infill() const { return this->surface_type == stInternalSolid; }
    /// Surface.hpp:112
    #[inline]
    pub fn is_solid_infill(&self) -> bool {
        self.surface_type.is_solid_infill()
    }

    /// Set the surface type.
    pub fn set_type(&mut self, surface_type: SurfaceType) {
        self.surface_type = surface_type;
    }

    /// Set the bridge angle.
    pub fn set_bridge_angle(&mut self, angle: CoordF) {
        self.bridge_angle = Some(angle);
    }

    /// Set the thickness.
    pub fn set_thickness(&mut self, thickness: CoordF) {
        self.thickness = thickness;
    }

    /// Set the number of thickness layers.
    pub fn set_thickness_layers(&mut self, layers: usize) {
        self.thickness_layers = layers;
    }
}

impl fmt::Debug for Surface {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Surface({:?}, area={:.2}mm²)",
            self.surface_type,
            self.area()
        )
    }
}

impl fmt::Display for Surface {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} surface (area={:.2}mm²)",
            self.surface_type,
            self.area()
        )
    }
}

impl From<ExPolygon> for Surface {
    fn from(expolygon: ExPolygon) -> Self {
        Self::new(SurfaceType::default(), expolygon)
    }
}

/// Type alias for a collection of surfaces.
pub type Surfaces = Vec<Surface>;

// typedef std::vector<Surface*> SurfacesPtr;
/// Surface.hpp:116
pub type SurfacesPtr<'a> = Vec<&'a Surface>;

// Helper function to append ExPolygons as surfaces with given type
// inline void surfaces_append(Surfaces &dst, ExPolygons &&src, SurfaceType surfaceType)
/// Surface.hpp:261
pub fn surfaces_append(surfaces: &mut Surfaces, expolygons: ExPolygons, surface_type: SurfaceType) {
    surfaces.reserve(surfaces.len() + expolygons.len());
    for expolygon in expolygons {
        surfaces.push(Surface::new(surface_type, expolygon));
    }
}

// inline void surfaces_append(Surfaces &dst, ExPolygons &&src, const Surface &surfaceTempl)
/// Surface.hpp:269
pub fn surfaces_append_templ(surfaces: &mut Surfaces, expolygons: ExPolygons, surface_templ: &Surface) {
    surfaces.reserve(surfaces.len() + number_polygons_ex(&expolygons));
    for expolygon in expolygons {
        // C++ Surface(surfaceTempl, std::move(*it)) copies all template fields but the expolygon.
        let mut s = surface_templ.clone();
        s.expolygon = expolygon;
        surfaces.push(s);
    }
}

// inline void surfaces_append(Surfaces &dst, Surfaces &&src)
/// Surface.hpp:277
pub fn surfaces_append_surfaces(dst: &mut Surfaces, mut src: Surfaces) {
    if dst.is_empty() {
        *dst = std::mem::take(&mut src);
    } else {
        dst.append(&mut src);
        src.clear();
    }
}

// ---------------------------------------------------------------------------
// Surface.hpp inline free functions
// ---------------------------------------------------------------------------

// inline Polygons to_polygons(const Surfaces &src)
// (Already faithfully ported as `crate::clipper_utils::to_polygons(&[Surface])`;
//  re-exported here for call sites matching the Surface.hpp signature.)
/// Surface.hpp:128
pub fn to_polygons(src: &[Surface]) -> crate::geometry::Polygons {
    let mut num: usize = 0;
    for it in src {
        num += it.expolygon.holes.len() + 1;
    }
    let mut polygons: crate::geometry::Polygons = Vec::with_capacity(num);
    for it in src {
        polygons.push(it.expolygon.contour.clone());
        for ith in &it.expolygon.holes {
            polygons.push(ith.clone());
        }
    }
    polygons
}

// inline ExPolygons to_expolygons(const Surfaces &src)
/// Surface.hpp:158
pub fn to_expolygons(src: &[Surface]) -> ExPolygons {
    let mut expolygons: ExPolygons = Vec::with_capacity(src.len());
    for it in src {
        expolygons.push(it.expolygon.clone());
    }
    expolygons
}

// Count a number of polygons stored inside the vector of expolygons.
// Useful for allocating space for polygons when converting expolygons to polygons.
// inline size_t number_polygons(const Surfaces &surfaces)
/// Surface.hpp:188
pub fn number_polygons(surfaces: &[Surface]) -> usize {
    let mut n_polygons: usize = 0;
    for it in surfaces {
        n_polygons += it.expolygon.holes.len() + 1;
    }
    n_polygons
}

// Same helper, counting over a vector of ExPolygons (used by surfaces_append).
fn number_polygons_ex(expolygons: &[ExPolygon]) -> usize {
    let mut n_polygons: usize = 0;
    for it in expolygons {
        n_polygons += it.holes.len() + 1;
    }
    n_polygons
}

// Append a vector of Surfaces at the end of another vector of polygons.
// inline void polygons_append(Polygons &dst, const Surfaces &src)
/// Surface.hpp:204
pub fn polygons_append(dst: &mut crate::geometry::Polygons, src: &[Surface]) {
    dst.reserve(dst.len() + number_polygons(src));
    for it in src {
        dst.push(it.expolygon.contour.clone());
        dst.extend(it.expolygon.holes.iter().cloned());
    }
}

// inline bool surfaces_could_merge(const Surface &s1, const Surface &s2)
/// Surface.hpp:291
pub fn surfaces_could_merge(s1: &Surface, s2: &Surface) -> bool {
    s1.surface_type == s2.surface_type
        && s1.thickness == s2.thickness
        && s1.thickness_layers == s2.thickness_layers
        && s1.bridge_angle == s2.bridge_angle
}

// ---------------------------------------------------------------------------
// Surface.cpp free functions
// ---------------------------------------------------------------------------

// BoundingBox get_extents(const Surface &surface)
/// Surface.cpp:7
pub fn get_extents(surface: &Surface) -> crate::geometry::BoundingBox {
    // return get_extents(surface.expolygon.contour);                    Surface.cpp:9
    surface.expolygon.contour.bounding_box()
}

// BoundingBox get_extents(const Surfaces &surfaces)
/// Surface.cpp:12
pub fn get_extents_surfaces(surfaces: &[Surface]) -> crate::geometry::BoundingBox {
    // BoundingBox bbox;                                 Surface.cpp:14
    let mut bbox = crate::geometry::BoundingBox::new();
    // if (! surfaces.empty()) {                         Surface.cpp:15
    if !surfaces.is_empty() {
        // bbox = get_extents(surfaces.front());         Surface.cpp:16
        bbox = get_extents(&surfaces[0]);
        // for (size_t i = 1; i < surfaces.size(); ++ i)
        //     bbox.merge(get_extents(surfaces[i]));     Surface.cpp:17-18
        for i in 1..surfaces.len() {
            bbox.merge(&get_extents(&surfaces[i]));
        }
    }
    // return bbox;                                      Surface.cpp:20
    bbox
}

// BoundingBox get_extents(const SurfacesPtr &surfaces)
/// Surface.cpp:23
pub fn get_extents_surfaces_ptr(surfaces: &[&Surface]) -> crate::geometry::BoundingBox {
    // BoundingBox bbox;                                 Surface.cpp:25
    let mut bbox = crate::geometry::BoundingBox::new();
    // if (! surfaces.empty()) {                         Surface.cpp:26
    if !surfaces.is_empty() {
        // bbox = get_extents(*surfaces.front());        Surface.cpp:27
        bbox = get_extents(surfaces[0]);
        // for (size_t i = 1; i < surfaces.size(); ++ i)
        //     bbox.merge(get_extents(*surfaces[i]));     Surface.cpp:28-29
        for i in 1..surfaces.len() {
            bbox.merge(&get_extents(surfaces[i]));
        }
    }
    // return bbox;                                      Surface.cpp:31
    bbox
}

// const char* surface_type_to_color_name(const SurfaceType surface_type)
/// Surface.cpp:34
pub fn surface_type_to_color_name(surface_type: SurfaceType) -> &'static str {
    // switch (surface_type) {                                            Surface.cpp:36
    match surface_type {
        // case stTop:             return "rgb(255,0,0)"; // "red";       Surface.cpp:37
        SurfaceType::Top => "rgb(255,0,0)", // "red";
        // case stBottom:          return "rgb(0,255,0)"; // "green";     Surface.cpp:38
        SurfaceType::Bottom => "rgb(0,255,0)", // "green";
        // case stBottomBridge:    return "rgb(0,0,255)"; // "blue";      Surface.cpp:39
        SurfaceType::BottomBridge => "rgb(0,0,255)", // "blue";
        // case stInternal:        return "rgb(255,255,128)"; // yellow   Surface.cpp:40
        SurfaceType::Internal => "rgb(255,255,128)", // yellow
        // case stFloatingVerticalShell:
        // case stInternalSolid:   return "rgb(255,0,255)"; // magenta    Surface.cpp:41-42
        SurfaceType::FloatingVerticalShell | SurfaceType::InternalSolid => "rgb(255,0,255)", // magenta
        // case stInternalBridge:  return "rgb(0,255,255)";               Surface.cpp:43
        SurfaceType::InternalBridge => "rgb(0,255,255)",
        // case stInternalVoid:    return "rgb(128,128,128)";             Surface.cpp:44
        SurfaceType::InternalVoid => "rgb(128,128,128)",
        // case stPerimeter:       return "rgb(128,0,0)"; // maroon       Surface.cpp:45
        SurfaceType::Perimeter => "rgb(128,0,0)", // maroon
        // default:                return "rgb(64,64,64)";  (SurfaceType(-1))  Surface.cpp:46
        #[allow(unreachable_patterns)]
        _ => "rgb(64,64,64)",
    }
}

// Point export_surface_type_legend_to_svg_box_size()
/// Surface.cpp:50
pub fn export_surface_type_legend_to_svg_box_size() -> crate::geometry::Point {
    // return Point(scale_(1.+10.*8.), scale_(3.));                       Surface.cpp:52
    crate::geometry::Point::new(crate::scale(1. + 10. * 8.), crate::scale(3.))
}

/// Extension trait for Surfaces to add helper methods
pub trait SurfacesExt {
    fn is_empty(&self) -> bool;
    fn clear(&mut self);
}

impl SurfacesExt for Surfaces {
    fn is_empty(&self) -> bool {
        self.is_empty()
    }

    fn clear(&mut self) {
        self.clear();
    }
}

/// Collection of surfaces with utility methods.
#[derive(Clone, Default, Serialize, Deserialize, Debug)]
pub struct SurfaceCollection {
    /// The surfaces in this collection.
    pub surfaces: Vec<Surface>,
}

impl SurfaceCollection {
    // Create a new empty surface collection.
    pub fn new() -> Self {
        Self {
            surfaces: Vec::new(),
        }
    }

    /// Create a surface collection from a vector of surfaces.
    pub fn from_surfaces(surfaces: Vec<Surface>) -> Self {
        Self { surfaces }
    }

    /// Check if the collection is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.surfaces.is_empty()
    }

    /// Get the number of surfaces.
    #[inline]
    pub fn len(&self) -> usize {
        self.surfaces.len()
    }

    /// Add a surface to the collection.
    pub fn push(&mut self, surface: Surface) {
        self.surfaces.push(surface);
    }

    /// Clear all surfaces.
    pub fn clear(&mut self) {
        self.surfaces.clear();
    }

    /// Remove surfaces of specific types.
    ///
    /// Reference: SurfaceCollection.cpp
    pub fn remove_types(&mut self, types: &[SurfaceType]) {
        self.surfaces
            .retain(|surface| !types.contains(&surface.surface_type));
    }

    /// Get all surfaces of a specific type.
    pub fn filter_by_type(&self, surface_type: SurfaceType) -> Vec<&Surface> {
        self.surfaces
            .iter()
            .filter(|s| s.surface_type == surface_type)
            .collect()
    }

    /// Get all surfaces whose type is in `types`.
    /// C++ SurfaceCollection::filter_by_types(const SurfaceType *types, int ntypes)
    pub fn filter_by_types(&self, types: &[SurfaceType]) -> Vec<&Surface> {
        self.surfaces
            .iter()
            .filter(|s| types.contains(&s.surface_type))
            .collect()
    }

    /// Keep only surfaces whose type is in `types` (drop the rest).
    /// C++ SurfaceCollection::keep_types(const SurfaceType *types, int ntypes)
    pub fn keep_types(&mut self, types: &[SurfaceType]) {
        self.surfaces
            .retain(|surface| types.contains(&surface.surface_type));
    }

    /// Get all top surfaces.
    pub fn top_surfaces(&self) -> Vec<&Surface> {
        self.surfaces.iter().filter(|s| s.is_top()).collect()
    }

    /// Get all bottom surfaces.
    pub fn bottom_surfaces(&self) -> Vec<&Surface> {
        self.surfaces.iter().filter(|s| s.is_bottom()).collect()
    }

    /// Get all solid surfaces.
    pub fn solid_surfaces(&self) -> Vec<&Surface> {
        self.surfaces.iter().filter(|s| s.is_solid()).collect()
    }

    /// Get all bridge surfaces.
    pub fn bridge_surfaces(&self) -> Vec<&Surface> {
        self.surfaces.iter().filter(|s| s.is_bridge()).collect()
    }

    /// Get the total area of all surfaces.
    pub fn total_area(&self) -> CoordF {
        self.surfaces.iter().map(|s| s.area()).sum()
    }

    /// Check if any surface has the given type.
    pub fn has_type(&self, surface_type: SurfaceType) -> bool {
        self.surfaces.iter().any(|s| s.surface_type == surface_type)
    }

    /// Convert all surfaces to ExPolygons
    pub fn to_expolygons(&self) -> ExPolygons {
        self.surfaces.iter().map(|s| s.expolygon.clone()).collect()
    }

    /// Set surfaces from ExPolygons with a given type
    pub fn set(&mut self, expolygons: &ExPolygons, surface_type: SurfaceType) {
        self.surfaces.clear();
        for expolygon in expolygons {
            self.surfaces
                .push(Surface::new(surface_type, expolygon.clone()));
        }
    }

    /// Set all surfaces to a given type
    /// Surface.cpp
    pub fn set_type(&mut self, surface_type: SurfaceType) {
        for surface in &mut self.surfaces {
            surface.surface_type = surface_type;
        }
    }

    /// Append ExPolygons as surfaces with given type
    /// Surface.cpp helper
    pub fn append(&mut self, expolygons: ExPolygons, surface_type: SurfaceType) {
        for expolygon in expolygons {
            self.surfaces.push(Surface::new(surface_type, expolygon));
        }
    }
}

impl fmt::Display for SurfaceCollection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SurfaceCollection({} surfaces)", self.surfaces.len())
    }
}

// Implement IntoIterator to allow for-in loops on references
impl<'a> IntoIterator for &'a SurfaceCollection {
    type Item = &'a Surface;
    type IntoIter = std::slice::Iter<'a, Surface>;

    fn into_iter(self) -> Self::IntoIter {
        self.surfaces.iter()
    }
}

// Implement IntoIterator for owned collection
impl IntoIterator for SurfaceCollection {
    type Item = Surface;
    type IntoIter = std::vec::IntoIter<Surface>;

    fn into_iter(self) -> Self::IntoIter {
        self.surfaces.into_iter()
    }
}

/// Detect surface types for a layer by comparing with adjacent layers.
///
/// This implements the core surface classification algorithm from BambuStudio's
/// `PrintObject::detect_surfaces_type()`.
///
/// # Arguments
///
/// * `current_slices` - The ExPolygons of the current layer
/// * `lower_slices` - The ExPolygons of the layer below (if any)
/// * `upper_slices` - The ExPolygons of the layer above (if any)
/// * `offset` - Small offset for robust intersection (typically flow width / 10)
///
/// # Returns
///
/// A vector of Surfaces with appropriate types assigned.
///
/// # Algorithm
///
/// 1. Find TOP surfaces: areas of current layer not covered by upper layer
/// 2. Find BOTTOM surfaces: areas of current layer not supported by lower layer
/// 3. Find BOTTOM_BRIDGE surfaces: bottom areas completely over air
/// 4. Remaining areas are INTERNAL
pub fn detect_surface_types(
    current_slices: &ExPolygons,
    lower_slices: Option<&ExPolygons>,
    upper_slices: Option<&ExPolygons>,
    offset: CoordF,
) -> Vec<Surface> {
    if current_slices.is_empty() {
        return Vec::new();
    }

    let mut surfaces = Vec::new();

    // Minimum area threshold for surfaces (in scaled coordinates²).
    // We use 0.01mm² as the minimum to filter out microscopic fragments
    // while keeping small but real features.
    // 0.01mm² = 0.01 × 1e12 scaled units² = 1e10
    let min_area_scaled = 0.01 * crate::SCALING_FACTOR * crate::SCALING_FACTOR;

    // Find TOP surfaces: difference between current and upper
    let top_expolygons = if let Some(upper) = upper_slices {
        if !upper.is_empty() {
            // Top = areas not covered by the layer above
            let diff = difference(current_slices, upper);
            // Apply small opening to remove noise
            if offset > 0.0 {
                opening(&diff, offset, OffsetJoinType::Miter)
            } else {
                diff
            }
        } else {
            // No upper layer geometry - entire layer is top
            current_slices.clone()
        }
    } else {
        // No upper layer - entire layer is top
        current_slices.clone()
    };

    // Find BOTTOM surfaces: difference between current and lower
    let (bottom_expolygons, bottom_bridge_expolygons) = if let Some(lower) = lower_slices {
        if !lower.is_empty() {
            // Bottom = areas not supported by the layer below
            let diff = difference(current_slices, lower);
            let bottom = if offset > 0.0 {
                opening(&diff, offset, OffsetJoinType::Miter)
            } else {
                diff
            };
            // TODO: Properly distinguish between partial support (Bottom) and true bridges (BottomBridge)
            // For now, all unsupported areas are classified as bridges to ensure bridge detection works
            (Vec::new(), bottom)
        } else {
            // Lower layer exists but is empty - this is a bridge over void
            (Vec::new(), current_slices.clone())
        }
    } else {
        // First layer - all is bottom (on build plate)
        (current_slices.clone(), Vec::new())
    };

    // Collect top and bottom polygons for computing internal areas
    let mut top_bottom_polygons: Vec<crate::geometry::Polygon> = Vec::new();

    // Add top surfaces
    for expoly in &top_expolygons {
        if expoly.area().abs() > min_area_scaled {
            // Filter tiny areas (< 0.01mm²)
            top_bottom_polygons.push(expoly.contour.clone());
            for hole in &expoly.holes {
                top_bottom_polygons.push(hole.clone());
            }
            surfaces.push(Surface::top(expoly.clone()));
        }
    }

    // Add bottom surfaces (on build plate)
    for expoly in &bottom_expolygons {
        if expoly.area().abs() > min_area_scaled {
            top_bottom_polygons.push(expoly.contour.clone());
            for hole in &expoly.holes {
                top_bottom_polygons.push(hole.clone());
            }
            surfaces.push(Surface::bottom(expoly.clone()));
        }
    }

    // Add bridge surfaces
    for expoly in &bottom_bridge_expolygons {
        if expoly.area().abs() > min_area_scaled {
            top_bottom_polygons.push(expoly.contour.clone());
            for hole in &expoly.holes {
                top_bottom_polygons.push(hole.clone());
            }
            surfaces.push(Surface::bridge(expoly.clone(), None));
        }
    }

    // Handle overlapping top and bottom (thin membranes)
    // If areas are both top and bottom, prefer bottom (allows bridge detection)
    if !top_expolygons.is_empty()
        && (!bottom_expolygons.is_empty() || !bottom_bridge_expolygons.is_empty())
    {
        let all_bottom: ExPolygons = bottom_expolygons
            .iter()
            .chain(bottom_bridge_expolygons.iter())
            .cloned()
            .collect();

        // Remove overlapping areas from top
        let top_only = difference(&top_expolygons, &all_bottom);

        // Rebuild surfaces with non-overlapping top
        surfaces.retain(|s| !s.is_top());
        for expoly in top_only {
            if expoly.area().abs() > min_area_scaled {
                surfaces.push(Surface::top(expoly));
            }
        }
    }

    // Find INTERNAL surfaces: areas that are neither top nor bottom
    // Convert surfaces to ExPolygons for difference operation
    let classified: ExPolygons = surfaces.iter().map(|s| s.expolygon.clone()).collect();

    let internal_expolygons = if !classified.is_empty() {
        difference(current_slices, &classified)
    } else {
        // No surfaces classified yet - if we have both upper and lower layers,
        // and both have geometry covering us, then the whole slice is internal
        if upper_slices.is_some() && lower_slices.is_some() {
            // Check if we're fully covered by both
            let upper = upper_slices.unwrap();
            let lower = lower_slices.unwrap();
            if !upper.is_empty() && !lower.is_empty() {
                // Both layers have geometry - classify remaining as internal
                current_slices.to_vec()
            } else {
                Vec::new()
            }
        } else {
            Vec::new()
        }
    };

    // Add internal surfaces
    for expoly in internal_expolygons {
        if expoly.area().abs() > min_area_scaled {
            surfaces.push(Surface::internal(expoly));
        }
    }

    surfaces
}

/// Detect surface types for multiple layers in parallel.
///
/// This processes all layers and assigns surface types based on
/// comparison with adjacent layers.
///
/// # Arguments
///
/// * `layer_slices` - Slice geometry for each layer
/// * `offset` - Small offset for robust intersection
///
/// # Returns
///
/// A vector of Surface vectors, one per layer.
pub fn detect_all_surface_types(layer_slices: &[ExPolygons], offset: CoordF) -> Vec<Vec<Surface>> {
    let num_layers = layer_slices.len();
    let mut result = Vec::with_capacity(num_layers);

    for i in 0..num_layers {
        let lower = if i > 0 {
            Some(&layer_slices[i - 1])
        } else {
            None
        };
        let upper = if i + 1 < num_layers {
            Some(&layer_slices[i + 1])
        } else {
            None
        };

        let surfaces = detect_surface_types(&layer_slices[i], lower, upper, offset);
        result.push(surfaces);
    }

    result
}

/// Configuration for surface type detection.
#[derive(Debug, Clone)]
pub struct SurfaceDetectionConfig {
    /// Number of solid top layers.
    pub top_solid_layers: usize,
    /// Number of solid bottom layers.
    pub bottom_solid_layers: usize,
    /// Offset for robust intersection (typically flow width / 10).
    pub offset: CoordF,
    /// Minimum area for a surface to be kept (mm²).
    pub min_area: CoordF,
    /// Shell growth distance (mm) for solid propagation.
    ///
    /// When propagating solid infill from top/bottom surfaces to adjacent layers,
    /// the shell region is grown by this amount before intersecting with internal
    /// surfaces. BambuStudio's `discover_vertical_shells` uses a similar growth
    /// step so that thin top-surface strips along sloped walls expand to cover
    /// the full infill region rather than leaving narrow slivers as sparse infill.
    ///
    /// A value of 0.0 disables growth (old behaviour). A good default is around
    /// 2× the nozzle diameter (e.g. 0.8 mm for a 0.4 mm nozzle).
    pub shell_growth: CoordF,
    /// Approximate fill boundary inset (mm).
    ///
    /// In C++, `discover_horizontal_shells` operates on `fill_surfaces` which
    /// have been clipped to perimeter-generated fill boundaries. This means
    /// the internal regions used for progressive narrowing exclude the
    /// perimeter shell area. Without this clipping, the internal regions
    /// are the full layer slices minus top/bottom — too large for the
    /// "shadow" narrowing to be effective.
    ///
    /// As an approximation, we shrink the internal regions by this amount
    /// before intersection. A good value is:
    ///   `external_perimeter_width/2 + external_perimeter_spacing
    ///    + perimeter_spacing * (perimeters - 1)`
    ///
    /// A value of 0.0 disables the approximation (no shrinking).
    pub fill_boundary_inset: CoordF,
    /// Solid infill line width (mm) for too-narrow filtering.
    ///
    /// When propagating solid shells, strips narrower than `3 * solid_infill_width`
    /// are detected and grown to ensure they don't collapse during infill generation.
    /// This matches BambuStudio's margin calculation in PrintObject.cpp:3499.
    ///
    /// A value of 0.0 disables too-narrow filtering.
    pub solid_infill_width: CoordF,
}

impl Default for SurfaceDetectionConfig {
    fn default() -> Self {
        Self {
            top_solid_layers: 3,
            bottom_solid_layers: 3,
            offset: 0.045,            // ~0.45mm / 10
            min_area: 0.5,            // 0.5 mm²
            shell_growth: 0.0,        // disabled by default; set >0 to grow shells
            fill_boundary_inset: 0.0, // disabled by default
            solid_infill_width: 0.0,  // disabled by default; set to nozzle diameter to enable
        }
    }
}

/// Clip classified surfaces to perimeter-generated fill boundaries.
///
/// This is a port of BambuStudio's `LayerRegion::slices_to_fill_surfaces_clipped()`
/// (LayerRegion.cpp:50-68). After surface classification (`detect_surface_types`),
/// the classified surfaces cover the full layer slice area — including the perimeter
/// shell. This function clips them to just the infill area (inside all perimeters),
/// producing `fill_surfaces` that match the C++ data flow.
///
/// Without this clipping:
/// - Surface propagation (`discover_horizontal_shells`) considers the perimeter area
///   as "Internal", making progressive narrowing ineffective
/// - Infill generation intersects on-the-fly, but the surface regions used for
///   propagation are too large
///
/// # Arguments
///
/// * `surfaces` - Per-layer classified surfaces (modified in place)
/// * `fill_areas` - Per-layer fill boundaries from perimeter generation.
///   Each entry is the union of `perimeter_result.infill_area` across all islands
///   on that layer. If a layer has no fill area (e.g. empty layer), pass an empty Vec.
///
/// # Algorithm
///
/// For each layer:
///   1. Group surfaces by type (Top, Bottom, BottomBridge, Internal, InternalSolid, …)
///   2. For each group, intersect with the fill area for that layer
///   3. Replace the layer's surfaces with the clipped results
pub fn clip_surfaces_to_fill_boundaries(surfaces: &mut [Vec<Surface>], fill_areas: &[ExPolygons]) {
    assert_eq!(
        surfaces.len(),
        fill_areas.len(),
        "surfaces and fill_areas must have the same number of layers"
    );

    for (layer_idx, layer_surfaces) in surfaces.iter_mut().enumerate() {
        let fill_area = &fill_areas[layer_idx];
        if fill_area.is_empty() {
            // No fill area → no fill surfaces (layer is entirely perimeter or empty)
            layer_surfaces.clear();
            continue;
        }

        // Group surfaces by type, then intersect each group with fill_area.
        // This matches C++: for each surface_type, collect all ExPolygons of
        // that type, then `intersection_ex(group, fill_expolygons)`.
        let mut clipped: Vec<Surface> = Vec::new();

        // Collect unique surface types present on this layer
        let mut seen_types: Vec<SurfaceType> = Vec::new();
        for s in layer_surfaces.iter() {
            if !seen_types.contains(&s.surface_type) {
                seen_types.push(s.surface_type);
            }
        }

        for stype in &seen_types {
            // Bridge surfaces should NOT be clipped to fill boundaries yet.
            // In C++, bridges are clipped here but then immediately expanded
            // back by `process_external_surfaces()` (Step 4). Without that
            // expansion step, clipping removes most bridge area, causing
            // bridge infill to crash (0.76× → 0.04×). Preserve them as-is
            // until Step 4 is implemented.
            if *stype == SurfaceType::BottomBridge || *stype == SurfaceType::InternalBridge {
                for s in layer_surfaces.iter() {
                    if s.surface_type == *stype {
                        clipped.push(s.clone());
                    }
                }
                continue;
            }

            // Collect all ExPolygons of this type
            let group: ExPolygons = layer_surfaces
                .iter()
                .filter(|s| s.surface_type == *stype)
                .map(|s| s.expolygon.clone())
                .collect();

            if group.is_empty() {
                continue;
            }

            // Intersect with fill boundaries
            let trimmed = intersection(&group, fill_area);

            // Find a template surface for metadata (thickness, bridge_angle, etc.)
            let template = layer_surfaces
                .iter()
                .find(|s| s.surface_type == *stype)
                .unwrap();

            for ep in trimmed {
                // Skip tiny slivers from boolean ops (< 0.01mm²)
                // Use proper scaled area: 0.01mm² = 0.01 × 1e12 scaled units² = 1e10
                let min_area_scaled = 0.01 * crate::SCALING_FACTOR * crate::SCALING_FACTOR;
                if ep.area().abs() < min_area_scaled {
                    continue;
                }
                let mut s = template.clone();
                s.expolygon = ep;
                clipped.push(s);
            }
        }

        *layer_surfaces = clipped;
    }
}

/// Prepare fill surfaces — post-classification cleanup before propagation.
///
/// Port of BambuStudio's `LayerRegion::prepare_fill_surfaces()` (LayerRegion.cpp:645-693).
/// This is Step 2 in the `prepare_infill` pipeline, called after
/// `clip_surfaces_to_fill_boundaries()` (Step 1) and before `discover_vertical_shells()`
/// (Step 3).
///
/// Three operations:
/// 1. If `top_solid_layers == 0`, demote Top surfaces to Internal (user wants no solid top)
/// 2. If `bottom_solid_layers == 0`, demote Bottom surfaces to Internal (user wants no solid bottom)
/// 3. If `minimum_sparse_infill_area > 0`, promote small Internal regions to InternalSolid
///
/// # Arguments
///
/// * `surfaces` - Per-layer fill surfaces (modified in place)
/// * `config` - Surface detection config (provides top/bottom solid layer counts)
/// * `minimum_sparse_infill_area` - Area threshold in mm². Internal surfaces with area
///   smaller than this are promoted to InternalSolid. Pass 0.0 to disable.
pub fn prepare_fill_surfaces(
    surfaces: &mut [Vec<Surface>],
    config: &SurfaceDetectionConfig,
    minimum_sparse_infill_area: CoordF,
) {
    // Convert mm² threshold to scaled-coordinate² units.
    // Our polygon areas are computed in scaled coordinates (1 mm = 1_000_000 units),
    // so area is in units² where 1 mm² = SCALING_FACTOR².
    // C++ does: `surface.area() < scale_(scale_(min_area))` for the same reason.
    let scaled_area_threshold =
        minimum_sparse_infill_area * crate::SCALING_FACTOR * crate::SCALING_FACTOR;

    for layer_surfaces in surfaces.iter_mut() {
        // (1) If no solid top layers requested, demote Top → Internal
        if config.top_solid_layers == 0 {
            for s in layer_surfaces.iter_mut() {
                if s.surface_type == SurfaceType::Top {
                    s.surface_type = SurfaceType::Internal;
                }
            }
        }

        // (2) If no solid bottom layers requested, demote Bottom → Internal
        if config.bottom_solid_layers == 0 {
            for s in layer_surfaces.iter_mut() {
                if s.surface_type == SurfaceType::Bottom {
                    s.surface_type = SurfaceType::Internal;
                }
            }
        }

        // (3) Promote tiny sparse infill regions to solid infill.
        // Surface::area() returns area in scaled-coordinate² (nanometer²),
        // so we compare against the pre-scaled threshold.
        if scaled_area_threshold > 0.0 {
            for s in layer_surfaces.iter_mut() {
                if s.surface_type == SurfaceType::Internal && s.area().abs() < scaled_area_threshold
                {
                    s.surface_type = SurfaceType::InternalSolid;
                }
            }
        }
    }
}

/// Discover vertical shells — ensure minimum solid shell width on each layer.
///
/// Port of BambuStudio's `PrintObject::discover_vertical_shells()`
/// (PrintObject.cpp:1732-1822). This is Step 3 in the `prepare_infill` pipeline,
/// called after `prepare_fill_surfaces()` (Step 2) and before
/// `process_external_surfaces()` (Step 4).
///
/// For each layer, this function:
/// 1. Collects all solid surfaces (Top, Bottom, BottomBridge, InternalSolid)
/// 2. Unions them and grows the union by `shell_growth` distance
/// 3. Intersects the grown region with Internal surfaces on the SAME layer
/// 4. Converts matched Internal regions to InternalSolid
///
/// This ensures that near sloped walls where top/bottom surfaces form thin strips,
/// the solid infill extends wide enough to create a proper shell. Without this,
/// sloped walls may have narrow slivers of solid infill surrounded by sparse infill.
///
/// # Arguments
///
/// * `surfaces` - Per-layer fill surfaces (modified in place)
/// * `config` - Surface detection config (provides `shell_growth` distance)
///
/// A `shell_growth` of 0.0 makes this function a no-op.
pub fn discover_vertical_shells(surfaces: &mut [Vec<Surface>], config: &SurfaceDetectionConfig) {
    let num_layers = surfaces.len();
    if num_layers == 0 {
        return;
    }
    let n_top = config.top_solid_layers;
    let n_bottom = config.bottom_solid_layers;
    if n_top == 0 && n_bottom == 0 {
        return;
    }

    let min_area = config.min_area;
    let top_bottom_expansion = config.solid_infill_width * 0.05; // C++ top_bottom_expansion_coeff

    // Cache top, bottom, holes per layer
    struct LayerCache {
        top_surfaces: ExPolygons,
        bottom_surfaces: ExPolygons,
        holes: ExPolygons, // internal fill regions
    }
    let mut cache: Vec<LayerCache> = Vec::with_capacity(num_layers);
    for layer_surfaces in surfaces.iter() {
        let top: ExPolygons = layer_surfaces
            .iter()
            .filter(|s| s.surface_type == SurfaceType::Top)
            .map(|s| s.expolygon.clone())
            .collect();
        let bottom: ExPolygons = layer_surfaces
            .iter()
            .filter(|s| {
                s.surface_type == SurfaceType::Bottom || s.surface_type == SurfaceType::BottomBridge
            })
            .map(|s| s.expolygon.clone())
            .collect();
        let holes: ExPolygons = layer_surfaces
            .iter()
            .filter(|s| {
                s.surface_type == SurfaceType::Internal
                    || s.surface_type == SurfaceType::InternalVoid
                    || s.surface_type == SurfaceType::InternalSolid
            })
            .map(|s| s.expolygon.clone())
            .collect();

        // Grow top/bottom for robust booleans
        let top_grown = if top_bottom_expansion > 0.0 && !top.is_empty() {
            grow(&top, top_bottom_expansion, OffsetJoinType::Miter)
        } else {
            top
        };
        let bottom_grown = if top_bottom_expansion > 0.0 && !bottom.is_empty() {
            grow(&bottom, top_bottom_expansion, OffsetJoinType::Miter)
        } else {
            bottom
        };

        cache.push(LayerCache {
            top_surfaces: top_grown,
            bottom_surfaces: bottom_grown,
            holes,
        });
    }

    // : For each layer, propagate shells and update surfaces
    for idx in 0..num_layers {
        let mut shell: ExPolygons = Vec::new();
        let mut holes: ExPolygons = cache[idx].holes.clone();

        // Gather top surfaces from layers ABOVE (idx+1 .. idx+n_top)
        if n_top > 0 {
            let end = (idx + n_top + 1).min(num_layers);
            for i in (idx + 1)..end {
                // Combine shells
                if shell.is_empty() {
                    shell = cache[i].top_surfaces.clone();
                } else if !cache[i].top_surfaces.is_empty() {
                    shell.extend(cache[i].top_surfaces.clone());
                    shell = union_ex(&shell);
                }
                // Intersect holes
                if !holes.is_empty() && !cache[i].holes.is_empty() {
                    holes = intersection(&holes, &cache[i].holes);
                } else {
                    holes.clear();
                }
            }
        }

        // Gather bottom surfaces from layers BELOW (idx-1 .. idx-n_bottom)
        if n_bottom > 0 {
            let start = if idx >= n_bottom { idx - n_bottom } else { 0 };
            for i in start..idx {
                if shell.is_empty() {
                    shell = cache[i].bottom_surfaces.clone();
                } else if !cache[i].bottom_surfaces.is_empty() {
                    shell.extend(cache[i].bottom_surfaces.clone());
                    shell = union_ex(&shell);
                }
                if !holes.is_empty() && !cache[i].holes.is_empty() {
                    holes = intersection(&holes, &cache[i].holes);
                } else {
                    holes.clear();
                }
            }
        }

        if shell.is_empty() {
            continue;
        }

        // Intersect shell with current layer's internal surfaces
        let internal_polys: ExPolygons = surfaces[idx]
            .iter()
            .filter(|s| {
                s.surface_type == SurfaceType::Internal
                    || s.surface_type == SurfaceType::InternalVoid
                    || s.surface_type == SurfaceType::InternalSolid
            })
            .map(|s| s.expolygon.clone())
            .collect();

        if internal_polys.is_empty() {
            continue;
        }

        // shell = intersection(shell, internal) + diff(internal, holes)
        let shell_in_internal = intersection(&shell, &internal_polys);
        let internal_not_holes = if !holes.is_empty() {
            difference(&internal_polys, &holes)
        } else {
            Vec::new()
        };

        let mut combined_shell = shell_in_internal;
        combined_shell.extend(internal_not_holes);
        if combined_shell.is_empty() {
            continue;
        }

        // Also add existing InternalSolid
        let existing_solid: ExPolygons = surfaces[idx]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::InternalSolid)
            .map(|s| s.expolygon.clone())
            .collect();
        combined_shell.extend(existing_solid);
        let combined_shell = union_ex(&combined_shell);

        // Regularize — remove narrow slivers, fill narrow gaps
        let min_spacing = config.solid_infill_width * 1.05;
        let regularized = if min_spacing > 0.0 {
            let narrow_wall_radius = 0.5 * 0.65 * min_spacing;
            let narrow_infill_radius = 0.5 * 1.2 * min_spacing;
            let tiny_overlap = 0.2 * min_spacing;

            // Open (shrink then grow) to remove narrow regions
            let opened = shrink(&combined_shell, narrow_wall_radius, OffsetJoinType::Square);
            let closed = grow(
                &opened,
                narrow_wall_radius + narrow_infill_radius,
                OffsetJoinType::Square,
            );
            let final_shell = shrink(
                &closed,
                narrow_infill_radius - tiny_overlap,
                OffsetJoinType::Square,
            );

            // Filter tiny fragments
            final_shell
                .into_iter()
                .filter(|ep| ep.area().abs() > min_area)
                .collect::<ExPolygons>()
        } else {
            combined_shell
        };

        if regularized.is_empty() {
            continue;
        }

        // Replace surfaces
        let new_internal_solid = intersection(&internal_polys, &regularized);
        let new_internal = difference(
            &surfaces[idx]
                .iter()
                .filter(|s| s.surface_type == SurfaceType::Internal)
                .map(|s| s.expolygon.clone())
                .collect::<ExPolygons>(),
            &regularized,
        );
        let new_internal_void = difference(
            &surfaces[idx]
                .iter()
                .filter(|s| s.surface_type == SurfaceType::InternalVoid)
                .map(|s| s.expolygon.clone())
                .collect::<ExPolygons>(),
            &regularized,
        );

        // Rebuild: keep Top, Bottom, BottomBridge; replace Internal/InternalVoid/InternalSolid
        let mut new_surfaces: Vec<Surface> = Vec::new();
        for s in surfaces[idx].iter() {
            match s.surface_type {
                SurfaceType::Top | SurfaceType::Bottom | SurfaceType::BottomBridge => {
                    new_surfaces.push(s.clone());
                }
                _ => {} // Will be replaced below
            }
        }

        let layer_height = surfaces[idx].first().map(|s| s.thickness).unwrap_or(0.2);

        for ep in new_internal {
            if ep.area().abs() > min_area {
                new_surfaces.push(Surface {
                    expolygon: ep,
                    surface_type: SurfaceType::Internal,
                    thickness: layer_height,
                    thickness_layers: 1,
                    bridge_angle: None,
                    extra_perimeters: 0,
                });
            }
        }
        for ep in new_internal_void {
            if ep.area().abs() > min_area {
                new_surfaces.push(Surface {
                    expolygon: ep,
                    surface_type: SurfaceType::InternalVoid,
                    thickness: layer_height,
                    thickness_layers: 1,
                    bridge_angle: None,
                    extra_perimeters: 0,
                });
            }
        }
        for ep in new_internal_solid {
            if ep.area().abs() > min_area {
                new_surfaces.push(Surface {
                    expolygon: ep,
                    surface_type: SurfaceType::InternalSolid,
                    thickness: layer_height,
                    thickness_layers: 1,
                    bridge_angle: None,
                    extra_perimeters: 0,
                });
            }
        }

        surfaces[idx] = new_surfaces;
    }
}

/// Process external surfaces — expand top/bottom/bridge surfaces into surrounding fill area.
///
/// Port of BambuStudio's `LayerRegion::process_external_surfaces()`
/// (LayerRegion.cpp:517-643) and the orchestrator in `PrintObject.cpp:1654-1730`.
/// This is Step 4 in the `prepare_infill` pipeline, called after
/// `discover_vertical_shells()` (Step 3) and before `discover_horizontal_shells()` (Step 5).
///
/// After `detect_surface_types()`, top/bottom surfaces are thin geometric strips along
/// the boundary between the current layer and adjacent layers. This function grows
/// them outward into surrounding Internal/InternalSolid fill area so that:
/// - Top surfaces are wide enough for proper top-surface infill (monotonic, slower speed)
/// - Bottom surfaces properly cover the supported area
/// - Bridge surfaces have proper anchoring
///
/// # Simplified approach (vs full C++ wave expansion)
///
/// The C++ implementation uses iterative wave expansion (0.1mm steps). This simplified
/// version grows in a single step by `expansion_distance`, which captures ~80% of the
/// C++ behavior. The wave expansion can be added later for better accuracy on complex
/// geometries.
///
/// # Arguments
///
/// * `surfaces` - Per-layer fill surfaces (modified in place)
/// * `expansion_distance` - How far to grow external surfaces (mm).
///   BambuStudio uses `shell_width * sqrt(2)` where shell_width ≈ nozzle_diameter.
///   Typical value: 0.566mm for a 0.4mm nozzle.
/// * `min_area_mm2` - Minimum area (mm²) for surfaces to be kept.
///   Tiny slivers from boolean ops are filtered out.
pub fn process_external_surfaces(
    surfaces: &mut [Vec<Surface>],
    expansion_distance: CoordF,
    min_area_mm2: CoordF,
) {
    if expansion_distance <= 0.0 {
        return;
    }

    // Area is in scaled^2 units: 1 mm^2 = SCALING_FACTOR^2 (= 1e10, since SCALING_FACTOR=1e5).
    // The previous `* 1e12` assumed SCALING_FACTOR=1e6, making this threshold 100x too large
    // (50 mm^2 instead of 0.5 mm^2) — which deleted nearly every Top/Bottom surface here.
    let min_area_scaled = min_area_mm2 * crate::SCALING_FACTOR * crate::SCALING_FACTOR;

    for layer_surfaces in surfaces.iter_mut() {
        // Collect the "available fill area" = union of (Internal ∪ InternalSolid).
        // External surfaces can expand into this area.
        let fill_area: ExPolygons = layer_surfaces
            .iter()
            .filter(|s| {
                s.surface_type == SurfaceType::Internal
                    || s.surface_type == SurfaceType::InternalSolid
            })
            .map(|s| s.expolygon.clone())
            .collect();

        if fill_area.is_empty() {
            continue;
        }

        // Process each external surface type: Top, Bottom, BottomBridge.
        // For each type, grow the surfaces and intersect with fill area.
        let external_types = [
            SurfaceType::Top,
            SurfaceType::Bottom,
            SurfaceType::BottomBridge,
        ];

        let mut grown_by_type: Vec<(SurfaceType, ExPolygons)> = Vec::new();
        let mut all_grown: ExPolygons = Vec::new();

        for &stype in &external_types {
            let type_polys: ExPolygons = layer_surfaces
                .iter()
                .filter(|s| s.surface_type == stype)
                .map(|s| s.expolygon.clone())
                .collect();

            if type_polys.is_empty() {
                continue;
            }

            // Grow external surfaces outward by expansion_distance.
            let grown = grow(&type_polys, expansion_distance, OffsetJoinType::Miter);

            // Intersect with the available fill area.
            // The intersection limits expansion to only the internal regions,
            // preventing growth outside the object boundary.
            let expanded = intersection(&grown, &fill_area);

            if expanded.is_empty() {
                // Growth didn't reach any fill area — keep original surfaces as-is.
                // This can happen when external surfaces are surrounded by other
                // external surfaces (e.g. a thin wall that is all Top/Bottom).
                grown_by_type.push((stype, type_polys));
                all_grown.extend(
                    layer_surfaces
                        .iter()
                        .filter(|s| s.surface_type == stype)
                        .map(|s| s.expolygon.clone()),
                );
                continue;
            }

            // Union the expanded area with the original surfaces.
            // This ensures the original region is always included even if
            // growth didn't fully cover it (e.g. due to Miter join artifacts).
            let merged = union_ex(&[type_polys, expanded].concat());

            // Re-intersect with original + fill area to stay within bounds.
            // The original surfaces may be outside fill_area (they're already
            // classified), so we include them in the clip boundary.
            let original_area: ExPolygons = layer_surfaces
                .iter()
                .filter(|s| s.surface_type == stype)
                .map(|s| s.expolygon.clone())
                .collect();
            let clip_boundary = union_ex(&[fill_area.clone(), original_area].concat());
            let final_expanded = intersection(&merged, &clip_boundary);

            let final_filtered: ExPolygons = final_expanded
                .into_iter()
                .filter(|ep| ep.area().abs() >= min_area_scaled)
                .collect();

            if !final_filtered.is_empty() {
                all_grown.extend(final_filtered.iter().cloned());
                grown_by_type.push((stype, final_filtered));
            }
        }

        if all_grown.is_empty() {
            continue;
        }

        // Rebuild the surface list:
        // 1. Subtract all grown external area from Internal surfaces
        // 2. Subtract all grown external area from InternalSolid surfaces
        // 3. Add the grown external surfaces
        // 4. Preserve other surface types (InternalBridge, InternalVoid)
        let mut new_surfaces: Vec<Surface> = Vec::new();

        // (a) Remaining Internal after subtracting grown externals
        let internal_polys: ExPolygons = layer_surfaces
            .iter()
            .filter(|s| s.surface_type == SurfaceType::Internal)
            .map(|s| s.expolygon.clone())
            .collect();
        if !internal_polys.is_empty() {
            let remaining = difference(&internal_polys, &all_grown);
            for ep in remaining {
                if ep.area().abs() >= min_area_scaled {
                    new_surfaces.push(Surface::internal(ep));
                }
            }
        }

        // (b) Remaining InternalSolid after subtracting grown externals
        let internal_solid_polys: ExPolygons = layer_surfaces
            .iter()
            .filter(|s| s.surface_type == SurfaceType::InternalSolid)
            .map(|s| s.expolygon.clone())
            .collect();
        if !internal_solid_polys.is_empty() {
            let remaining = difference(&internal_solid_polys, &all_grown);
            for ep in remaining {
                if ep.area().abs() >= min_area_scaled {
                    new_surfaces.push(Surface::internal_solid(ep));
                }
            }
        }

        // (c) Add grown external surfaces (replacing originals)
        for (stype, polys) in &grown_by_type {
            for ep in polys {
                if ep.area().abs() >= min_area_scaled {
                    let mut s = Surface::new(*stype, ep.clone());
                    // Preserve bridge angle from original surfaces if applicable
                    if *stype == SurfaceType::BottomBridge {
                        if let Some(orig) = layer_surfaces
                            .iter()
                            .find(|s| s.surface_type == SurfaceType::BottomBridge)
                        {
                            s.bridge_angle = orig.bridge_angle;
                        }
                    }
                    new_surfaces.push(s);
                }
            }
        }

        // (d) Preserve other surface types unchanged
        for s in layer_surfaces.iter() {
            match s.surface_type {
                SurfaceType::Internal
                | SurfaceType::InternalSolid
                | SurfaceType::Top
                | SurfaceType::Bottom
                | SurfaceType::BottomBridge => {
                    // Already handled above
                }
                _ => {
                    new_surfaces.push(s.clone());
                }
            }
        }

        *layer_surfaces = new_surfaces;
    }
}

/// Propagate solid infill through layers (legacy wrapper).
///
/// Calls `discover_horizontal_shells` with the same config.
pub fn propagate_solid_infill(surfaces: &mut [Vec<Surface>], config: &SurfaceDetectionConfig) {
    discover_horizontal_shells(surfaces, config);
}

/// Discover horizontal shells — propagate solid infill N layers deep from
/// top/bottom surfaces using **progressive intersection narrowing**.
///
/// This is a faithful port of BambuStudio's `PrintObject::discover_horizontal_shells()`
/// (PrintObject.cpp:3385). The key algorithmic property is that the solid region
/// **shrinks** as it propagates deeper: at each depth, we intersect the current
/// `solid` with the neighbor layer's internal surfaces, and the result becomes
/// the new `solid` for the next depth. This "shadow" effect prevents solid infill
/// from bleeding into areas that aren't geometrically related to the source
/// top/bottom surface — for example, it prevents bottom shells in a hollow sloping
/// vase from propagating up through the walls where there are many narrow bottom
/// surfaces.
///
/// # Algorithm (for top surfaces; bottom is symmetric)
///
/// ```text
/// For each layer i with top surfaces:
///     solid = collect top surface polygons at layer i
///     for n = i-1 downto max(0, i - num_solid_layers + 1):
///         internal = neighbor layer n's (Internal ∪ InternalSolid) polygons
///         new_solid = intersection(solid, internal)
///         if new_solid is empty:
///             if sparse_density == 0: break // hollow object, stop
///             else: continue // has infill, keep searching
///         // Filter too-narrow regions and regrow them
///         too_narrow = diff(new_solid, opening(new_solid, margin))
///         if too_narrow is not empty:
///             new_solid ∪= intersection(expand(too_narrow, margin), internal)
///         // Update neighbor layer: convert Internal → InternalSolid where covered
///         neighbor[InternalSolid] = union(new_solid, existing InternalSolid)
///         neighbor[Internal] = diff(existing Internal, new_solid)
///         // Progressive narrowing: solid shrinks for next iteration
///         solid = new_solid
/// ```
///
/// # Arguments
///
/// * `surfaces` - Mutable surface vectors for each layer (modified in place)
/// * `config` - Surface detection configuration (layer counts, min_area, etc.)
pub fn discover_horizontal_shells(surfaces: &mut [Vec<Surface>], config: &SurfaceDetectionConfig) {
    let num_layers = surfaces.len();
    if num_layers == 0 {
        return;
    }

    // DEBUG: Layer-limited logging (set via env var SURFACE_DEBUG_LAYER)
    let debug_layer: Option<usize> = std::env::var("SURFACE_DEBUG_LAYER")
        .ok()
        .and_then(|s| s.parse().ok());
    let should_log = |layer: usize| debug_layer.map_or(false, |dl| layer == dl);

    // Minimum area threshold in scaled units² for a solid region to be kept.
    // config.min_area is in mm²; scale² = 1e12.
    let min_area_scaled = config.min_area * 1e12;

    // Too-narrow filtering margin.
    // C++ uses `3.0 * layerm->flow(frSolidInfill).scaled_width()` ≈ 1.35mm.
    // However, this filtering requires properly clipped fill_surfaces (Step 1)
    // to work correctly — without clipping, raw surfaces include the perimeter
    // area, making the too-narrow filter expand regions far beyond their intended
    // extent. Disabled (0.0) until slices_to_fill_surfaces_clipped is implemented.
    let _too_narrow_margin = 1.35; // will be used after Step 1

    if debug_layer.is_some() {
        eprintln!("[SURFACE DEBUG] discover_horizontal_shells: num_layers={}, config={{ top_solid_layers={}, bottom_solid_layers={}, fill_boundary_inset={:.3}mm, solid_infill_width={:.3}mm }}",
            num_layers, config.top_solid_layers, config.bottom_solid_layers, config.fill_boundary_inset, config.solid_infill_width);
    }

    // Process each surface type: Top (propagate downward), Bottom and BottomBridge
    // (propagate upward). This matches the C++ loop over idx_surface_type 0..3.
    for surface_type_idx in 0..3u8 {
        let (stype, propagation_dir) = match surface_type_idx {
            0 => (SurfaceType::Top, -1i32),         // top → propagate downward
            1 => (SurfaceType::Bottom, 1i32),       // bottom → propagate upward
            _ => (SurfaceType::BottomBridge, 1i32), // bridge → propagate upward
        };

        let num_solid_layers = if stype == SurfaceType::Top {
            config.top_solid_layers
        } else {
            config.bottom_solid_layers
        };

        if num_solid_layers == 0 {
            continue;
        }

        if debug_layer.is_some() {
            eprintln!("\n[PROPAGATE] ════════════════════════════════════════════════════");
            eprintln!(
                "[PROPAGATE] Starting {:?} propagation, direction={}, num_solid_layers={}",
                stype,
                if propagation_dir < 0 { "DOWN" } else { "UP" },
                num_solid_layers
            );
        }

        for i in 0..num_layers {
            // DEBUG: Show surface composition for debug layer
            if should_log(i) {
                eprintln!(
                    "[SURFACE DEBUG] Layer {} surface types BEFORE propagation:",
                    i
                );
                let mut type_counts: std::collections::HashMap<SurfaceType, usize> =
                    std::collections::HashMap::new();
                for s in &surfaces[i] {
                    *type_counts.entry(s.surface_type).or_insert(0) += 1;
                }
                for (stype_key, count) in type_counts.iter() {
                    eprintln!("  {:?}: {}", stype_key, count);
                }
                let total_area_mm2: f64 = surfaces[i]
                    .iter()
                    .map(|s| s.expolygon.area().abs() / 1e12)
                    .sum();
                eprintln!("  Total area: {:.4} mm²", total_area_mm2);
            }

            // Collect polygons of the current surface type at this layer.
            // C++ collects from both `slices` and `fill_surfaces`; since Rust
            // doesn't have separate fill_surfaces yet, we collect from our
            // unified surface list (which is equivalent to slices at this point).
            let solid_regions: ExPolygons = surfaces[i]
                .iter()
                .filter(|s| s.surface_type == stype)
                .map(|s| s.expolygon.clone())
                .collect();

            if solid_regions.is_empty() {
                if should_log(i) {
                    eprintln!(
                        "[PROPAGATE] Layer {}: No {:?} regions found, skipping",
                        i, stype
                    );
                }
                continue;
            }

            if should_log(i) {
                let total_area_mm2: f64 = solid_regions.iter().map(|e| e.area().abs() / 1e12).sum();
                eprintln!("\n[PROPAGATE] ──────────────────────────────────────────────────────");
                eprintln!(
                    "[PROPAGATE] Layer {} is SOURCE for {:?} propagation",
                    i, stype
                );
                eprintln!(
                    "  - Found {} {:?} regions, total area: {:.4} mm²",
                    solid_regions.len(),
                    stype,
                    total_area_mm2
                );
                eprintln!(
                    "  - Will propagate {} for {} layers (max layer: {})",
                    if propagation_dir < 0 { "DOWN" } else { "UP" },
                    num_solid_layers,
                    if propagation_dir < 0 {
                        i as i32 - num_solid_layers as i32 + 1
                    } else {
                        i as i32 + num_solid_layers as i32 - 1
                    }
                );
            }

            // `solid` is the propagation front — it progressively narrows.
            let mut solid = solid_regions.clone();

            // Scatter to neighbor layers.
            let mut n = i as i32 + propagation_dir;
            let mut steps = 1usize; // how many layers we've propagated

            while steps < num_solid_layers {
                if n < 0 || n >= num_layers as i32 {
                    if should_log(i) {
                        eprintln!(
                            "[PROPAGATE]   Step {}: Reached boundary (n={}), stopping propagation",
                            steps, n
                        );
                    }
                    break;
                }
                let neighbor_idx = n as usize;

                if should_log(i) || should_log(neighbor_idx) {
                    let solid_area_mm2: f64 = solid.iter().map(|e| e.area().abs() / 1e12).sum();
                    eprintln!(
                        "\n[PROPAGATE]   Step {}/{}: Layer {} → Layer {}",
                        steps, num_solid_layers, i, neighbor_idx
                    );
                    eprintln!(
                        "[PROPAGATE]     Current solid front: {} regions, {:.4} mm²",
                        solid.len(),
                        solid_area_mm2
                    );
                }

                // Collect the neighbor's Internal + InternalSolid regions
                // (these are the regions into which solid can propagate).
                let raw_internal: ExPolygons = surfaces[neighbor_idx]
                    .iter()
                    .filter(|s| {
                        s.surface_type == SurfaceType::Internal
                            || s.surface_type == SurfaceType::InternalSolid
                    })
                    .map(|s| s.expolygon.clone())
                    .collect();

                if should_log(i) || should_log(neighbor_idx) {
                    let raw_area_mm2: f64 =
                        raw_internal.iter().map(|e| e.area().abs() / 1e12).sum();
                    eprintln!(
                        "[PROPAGATE]     Neighbor layer {} has {} Internal regions, {:.4} mm²",
                        neighbor_idx,
                        raw_internal.len(),
                        raw_area_mm2
                    );
                }

                // Approximate fill boundaries by shrinking the internal regions.
                // In C++, these would already be clipped to the perimeter-generated
                // fill area. We approximate this by insetting by the perimeter
                // shell width, which removes the perimeter area from the internal
                // regions and makes progressive narrowing effective.
                let internal_regions = if config.fill_boundary_inset > 0.0 {
                    let shrunk = shrink(
                        &raw_internal,
                        config.fill_boundary_inset,
                        OffsetJoinType::Miter,
                    );
                    if shrunk.is_empty() {
                        if should_log(i) || should_log(neighbor_idx) {
                            eprintln!(
                                "[PROPAGATE]     fill_boundary_inset={:.4}mm shrink resulted in EMPTY, using raw",
                                config.fill_boundary_inset
                            );
                        }
                        raw_internal
                    } else {
                        if should_log(i) || should_log(neighbor_idx) {
                            let shrunk_area_mm2: f64 =
                                shrunk.iter().map(|e| e.area().abs() / 1e12).sum();
                            eprintln!(
                                "[PROPAGATE]     After fill_boundary_inset={:.4}mm: {} regions, {:.4} mm²",
                                config.fill_boundary_inset, shrunk.len(), shrunk_area_mm2
                            );
                        }
                        shrunk
                    }
                } else {
                    raw_internal
                };

                // Intersect the current solid front with the neighbor's internal area.
                // This is the key progressive narrowing step.
                let new_internal_solid = intersection(&solid, &internal_regions);

                if should_log(i) || should_log(neighbor_idx) {
                    let solid_area_mm2: f64 = solid.iter().map(|e| e.area().abs() / 1e12).sum();
                    let internal_area_mm2: f64 =
                        internal_regions.iter().map(|e| e.area().abs() / 1e12).sum();
                    let new_solid_area_mm2: f64 = new_internal_solid
                        .iter()
                        .map(|e| e.area().abs() / 1e12)
                        .sum();
                    eprintln!(
                        "[PROPAGATE]     INTERSECTION: solid {:.4}mm² × internal {:.4}mm² → new_solid {:.4}mm² ({} regions)",
                        solid_area_mm2, internal_area_mm2, new_solid_area_mm2, new_internal_solid.len()
                    );
                }

                if new_internal_solid.is_empty() {
                    // No overlap — the solid front has been fully shadowed.
                    // C++ behavior: if sparse_density == 0 (hollow), break entirely.
                    // Otherwise, continue searching further layers (the solid front
                    // may find internal area again after skipping a gap).
                    // For now, we always continue (matches non-hollow behavior).
                    if should_log(i) || should_log(neighbor_idx) {
                        eprintln!(
                            "[PROPAGATE]     ❌ Intersection is EMPTY - no overlap between solid front and Internal regions"
                        );
                        eprintln!(
                            "[PROPAGATE]     Continuing to next layer (may find Internal area after gap)..."
                        );
                    }
                    n += propagation_dir;
                    steps += 1;
                    continue;
                }

                // Filter the raw intersection by minimum area BEFORE too-narrow
                // expansion. This prevents tiny slivers (e.g. 0.04 mm²) from
                // being expanded into large solid regions by the regrowth step.
                // In C++ this is less of an issue because surfaces are already
                // clipped to fill boundaries, so tiny overlaps rarely occur.
                let before_filter_count = new_internal_solid.len();
                let new_internal_solid: ExPolygons = new_internal_solid
                    .into_iter()
                    .filter(|ep| ep.area().abs() >= min_area_scaled)
                    .collect();

                if should_log(i) || should_log(neighbor_idx) {
                    if before_filter_count != new_internal_solid.len() {
                        eprintln!(
                            "[PROPAGATE]     Filtered by min_area ({:.4}mm²): {} → {} regions",
                            config.min_area,
                            before_filter_count,
                            new_internal_solid.len()
                        );
                    }
                }

                if new_internal_solid.is_empty() {
                    if should_log(i) || should_log(neighbor_idx) {
                        eprintln!(
                            "[PROPAGATE]     ❌ All regions filtered out by min_area, continuing..."
                        );
                    }
                    n += propagation_dir;
                    steps += 1;
                    continue;
                }

                // ── Too-narrow filtering ─────────────────────────────────
                // C++ ensures the new internal solid is wide enough that it
                // won't collapse when fill spacing is applied. It removes
                // strips narrower than `3 * solid_infill_width`, then regrows
                // them and adds the expanded area back.
                //
                // Reference: PrintObject.cpp:3497-3527
                let final_solid = if config.solid_infill_width > 0.0 {
                    let margin = 3.0 * config.solid_infill_width;
                    let opened = opening(&new_internal_solid, margin, OffsetJoinType::Miter);
                    let too_narrow = difference(&new_internal_solid, &opened);
                    if !too_narrow.is_empty() {
                        // Grow the collapsing parts and add to new_internal_solid
                        // clipped to internal regions (excluding bridges)
                        let regrown = grow(&too_narrow, margin, OffsetJoinType::Miter);
                        let internal_no_bridge: ExPolygons = surfaces[neighbor_idx]
                            .iter()
                            .filter(|s| {
                                s.surface_type == SurfaceType::Internal
                                    || s.surface_type == SurfaceType::InternalSolid
                            })
                            .map(|s| s.expolygon.clone())
                            .collect();
                        let regrown_clipped = intersection(&regrown, &internal_no_bridge);
                        union_ex(&[new_internal_solid.clone(), regrown_clipped].concat())
                    } else {
                        new_internal_solid.clone()
                    }
                } else {
                    new_internal_solid.clone()
                };

                // Post-expansion area filter (catches any degenerate slivers
                // introduced by the boolean operations).
                let before_final_filter = final_solid.len();
                let final_solid: ExPolygons = final_solid
                    .into_iter()
                    .filter(|ep| ep.area().abs() >= min_area_scaled)
                    .collect();

                if should_log(i) || should_log(neighbor_idx) {
                    let final_area_mm2: f64 =
                        final_solid.iter().map(|e| e.area().abs() / 1e12).sum();
                    eprintln!(
                        "[PROPAGATE]     ✅ Final solid after too-narrow handling: {} regions, {:.4} mm²",
                        final_solid.len(), final_area_mm2
                    );
                    if before_final_filter != final_solid.len() {
                        eprintln!(
                            "[PROPAGATE]        (filtered {} tiny regions post-expansion)",
                            before_final_filter - final_solid.len()
                        );
                    }
                }

                if final_solid.is_empty() {
                    if should_log(i) || should_log(neighbor_idx) {
                        eprintln!(
                            "[PROPAGATE]     ❌ All regions filtered out post-expansion, continuing..."
                        );
                    }
                    n += propagation_dir;
                    steps += 1;
                    continue;
                }

                // ── Update neighbor layer surfaces ────────────────────────
                // Only modify Internal and InternalSolid surfaces. Leave
                // Top/Bottom/Bridge surfaces completely untouched.
                //
                // In C++, non-internal surfaces are clipped against the new
                // internal polygons to avoid overlap, but that relies on
                // properly clipped fill_surfaces. Without fill_surface
                // clipping (Step 1), clipping other surfaces here causes
                // regressions because the internal polygons are too large.
                let existing_internal_solid: ExPolygons = surfaces[neighbor_idx]
                    .iter()
                    .filter(|s| s.surface_type == SurfaceType::InternalSolid)
                    .map(|s| s.expolygon.clone())
                    .collect();

                let merged_solid = if existing_internal_solid.is_empty() {
                    final_solid.clone()
                } else {
                    union_ex(&[final_solid.clone(), existing_internal_solid].concat())
                };

                // Subtract solid from existing Internal surfaces
                let existing_internal: ExPolygons = surfaces[neighbor_idx]
                    .iter()
                    .filter(|s| s.surface_type == SurfaceType::Internal)
                    .map(|s| s.expolygon.clone())
                    .collect();
                let remaining_internal = difference(&existing_internal, &merged_solid);

                // Rebuild the neighbor layer's surface list:
                // 1. Non-internal surfaces pass through unchanged
                // 2. InternalSolid = merged (new + existing)
                // 3. Internal = existing minus new solid
                let mut new_surfaces: Vec<Surface> = Vec::new();

                // Preserve all non-internal surfaces as-is
                for surface in &surfaces[neighbor_idx] {
                    match surface.surface_type {
                        SurfaceType::Internal | SurfaceType::InternalSolid => {
                            // Replaced below
                        }
                        _ => {
                            new_surfaces.push(surface.clone());
                        }
                    }
                }

                // Add merged InternalSolid
                for ep in &merged_solid {
                    if ep.area().abs() >= min_area_scaled {
                        new_surfaces.push(Surface::internal_solid(ep.clone()));
                    }
                }

                // Add remaining Internal
                for ep in &remaining_internal {
                    if ep.area().abs() >= min_area_scaled {
                        new_surfaces.push(Surface::internal(ep.clone()));
                    }
                }

                if should_log(i) || should_log(neighbor_idx) {
                    let internal_solid_count = new_surfaces
                        .iter()
                        .filter(|s| s.surface_type == SurfaceType::InternalSolid)
                        .count();
                    let internal_count = new_surfaces
                        .iter()
                        .filter(|s| s.surface_type == SurfaceType::Internal)
                        .count();
                    let internal_solid_area: f64 = new_surfaces
                        .iter()
                        .filter(|s| s.surface_type == SurfaceType::InternalSolid)
                        .map(|s| s.expolygon.area().abs() / 1e12)
                        .sum();
                    eprintln!(
                        "[PROPAGATE]     💾 Updated layer {}: InternalSolid={} ({:.4}mm²), Internal={}",
                        neighbor_idx, internal_solid_count, internal_solid_area, internal_count
                    );
                }

                surfaces[neighbor_idx] = new_surfaces;

                // ── Progressive narrowing ─────────────────────────────────
                // The solid front for the next iteration is the intersection
                // result (not the original top/bottom region). This ensures
                // shells are always a subset of shells found on the previous
                // layer — the "shadow" effect from C++.
                solid = final_solid;

                n += propagation_dir;
                steps += 1;
            }
        }
    }
}

/// Split Internal surfaces at `target_surfaces` that overlap with `shell_regions`.
///
/// For each Internal surface:
///   1. Compute the intersection with `shell_regions` → becomes InternalSolid.
///   2. Compute the difference (original − shell_regions) → stays Internal.
///   3. Filter out tiny slivers (area < `min_area_scaled`).
///
/// This mirrors BambuStudio's `discover_vertical_shells()` which splits surfaces
/// rather than converting entire surfaces wholesale.
/// Helper: split Internal surfaces in a target layer by a shell region.
///
/// This is still used by some tests and can serve as a simpler fallback.
/// The main propagation now happens inside `discover_horizontal_shells`.
fn split_internal_surfaces(
    target_surfaces: &mut Vec<Surface>,
    shell_regions: &ExPolygons,
    min_area_scaled: f64,
) {
    let mut new_surfaces: Vec<Surface> = Vec::new();

    for surface in target_surfaces.drain(..) {
        if surface.surface_type != SurfaceType::Internal {
            // Non-internal surfaces pass through unchanged
            new_surfaces.push(surface);
            continue;
        }

        // Compute the overlapping portion → InternalSolid
        let solid_parts = intersection(&[surface.expolygon.clone()], shell_regions);

        // Check if there is any meaningful overlap
        let has_solid = solid_parts
            .iter()
            .any(|ep| ep.area().abs() >= min_area_scaled);

        if !has_solid {
            // No significant overlap — keep the surface as Internal unchanged
            new_surfaces.push(surface);
            continue;
        }

        // Compute the remainder → stays Internal
        let remaining = difference(&[surface.expolygon.clone()], shell_regions);

        // Add the solid portions
        for ep in solid_parts {
            if ep.area().abs() >= min_area_scaled {
                let mut solid_surface = Surface::internal_solid(ep);
                solid_surface.thickness = surface.thickness;
                solid_surface.thickness_layers = surface.thickness_layers;
                new_surfaces.push(solid_surface);
            }
        }

        // Add the remaining internal portions
        for ep in remaining {
            if ep.area().abs() >= min_area_scaled {
                let mut int_surface = Surface::internal(ep);
                int_surface.thickness = surface.thickness;
                int_surface.thickness_layers = surface.thickness_layers;
                new_surfaces.push(int_surface);
            }
        }
    }

    *target_surfaces = new_surfaces;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};

    fn make_square_expolygon() -> ExPolygon {
        let poly = Polygon::rectangle(Point::new(0, 0), Point::new(1000000, 1000000));
        ExPolygon::new(poly)
    }

    #[test]
    fn test_surface_type_classification() {
        assert!(SurfaceType::Top.is_top());
        assert!(!SurfaceType::Top.is_bottom());
        assert!(SurfaceType::Top.is_solid());
        assert!(SurfaceType::Top.is_external());

        assert!(SurfaceType::Bottom.is_bottom());
        assert!(SurfaceType::Bottom.is_solid());
        assert!(SurfaceType::Bottom.is_external());

        assert!(SurfaceType::BottomBridge.is_bottom());
        assert!(SurfaceType::BottomBridge.is_bridge());
        assert!(SurfaceType::BottomBridge.is_solid());

        assert!(SurfaceType::Internal.is_internal());
        assert!(!SurfaceType::Internal.is_solid());

        assert!(SurfaceType::InternalSolid.is_internal());
        assert!(SurfaceType::InternalSolid.is_solid());

        assert!(SurfaceType::InternalBridge.is_bridge());
        assert!(SurfaceType::InternalBridge.is_solid());
    }

    #[test]
    fn test_surface_new() {
        let expoly = make_square_expolygon();
        let surface = Surface::new(SurfaceType::Top, expoly);

        assert!(surface.is_top());
        assert!(surface.is_solid());
        assert!(!surface.is_empty());
        assert!(surface.area() > 0.0);
    }

    #[test]
    fn test_surface_constructors() {
        let expoly = make_square_expolygon();

        let top = Surface::top(expoly.clone());
        assert!(top.is_top());

        let bottom = Surface::bottom(expoly.clone());
        assert!(bottom.is_bottom());

        let internal = Surface::internal(expoly.clone());
        assert!(internal.is_internal());
        assert!(!internal.is_solid());

        let bridge = Surface::bridge(expoly.clone(), Some(0.5));
        assert!(bridge.is_bridge());
        assert_eq!(bridge.bridge_angle, Some(0.5));
    }

    #[test]
    fn test_surface_setters() {
        let expoly = make_square_expolygon();
        let mut surface = Surface::new(SurfaceType::Internal, expoly);

        surface.set_type(SurfaceType::Top);
        assert!(surface.is_top());

        surface.set_bridge_angle(1.5);
        assert_eq!(surface.bridge_angle, Some(1.5));

        surface.set_thickness(0.2);
        assert!((surface.thickness - 0.2).abs() < 1e-6);

        surface.set_thickness_layers(3);
        assert_eq!(surface.thickness_layers, 3);
    }

    #[test]
    fn test_surface_collection() {
        let expoly = make_square_expolygon();

        let mut collection = SurfaceCollection::new();
        assert!(collection.is_empty());

        collection.push(Surface::top(expoly.clone()));
        collection.push(Surface::bottom(expoly.clone()));
        collection.push(Surface::internal(expoly.clone()));

        assert_eq!(collection.len(), 3);
        assert!(!collection.is_empty());

        assert_eq!(collection.top_surfaces().len(), 1);
        assert_eq!(collection.bottom_surfaces().len(), 1);
        assert_eq!(collection.solid_surfaces().len(), 2); // top + bottom

        assert!(collection.has_type(SurfaceType::Top));
        assert!(collection.has_type(SurfaceType::Bottom));
        assert!(collection.has_type(SurfaceType::Internal));
        assert!(!collection.has_type(SurfaceType::InternalBridge));
    }

    #[test]
    fn test_surface_collection_filter() {
        let expoly = make_square_expolygon();

        let mut collection = SurfaceCollection::new();
        collection.push(Surface::top(expoly.clone()));
        collection.push(Surface::top(expoly.clone()));
        collection.push(Surface::bottom(expoly.clone()));

        let tops = collection.filter_by_type(SurfaceType::Top);
        assert_eq!(tops.len(), 2);

        let bottoms = collection.filter_by_type(SurfaceType::Bottom);
        assert_eq!(bottoms.len(), 1);
    }

    #[test]
    fn test_surface_type_name() {
        assert_eq!(SurfaceType::Top.name(), "top");
        assert_eq!(SurfaceType::Bottom.name(), "bottom");
        assert_eq!(SurfaceType::BottomBridge.name(), "bottom bridge");
        assert_eq!(SurfaceType::InternalSolid.name(), "internal solid");
        assert_eq!(SurfaceType::Internal.name(), "internal");
        assert_eq!(SurfaceType::InternalBridge.name(), "internal bridge");
        assert_eq!(SurfaceType::InternalVoid.name(), "internal void");
    }

    #[test]
    fn test_detect_surface_types_first_layer() {
        let expoly = make_square_expolygon();
        let current_slices = vec![expoly];

        let surfaces = detect_surface_types(&current_slices, None, None, 0.01);

        // First layer with no layer below should be all bottom
        assert_eq!(surfaces.len(), 1);
        assert!(surfaces[0].is_bottom());
    }

    #[test]
    fn test_detect_surface_types_top_layer() {
        let expoly = make_square_expolygon();
        let current_slices = vec![expoly.clone()];
        let lower_slices = vec![expoly];

        let surfaces = detect_surface_types(&current_slices, Some(&lower_slices), None, 0.01);

        // Top layer with support below but nothing above should be top
        assert_eq!(surfaces.len(), 1);
        assert!(surfaces[0].is_top());
    }

    #[test]
    fn test_detect_surface_types_internal() {
        let expoly = make_square_expolygon();
        let current_slices = vec![expoly.clone()];
        let lower_slices = vec![expoly.clone()];
        let upper_slices = vec![expoly];

        let surfaces = detect_surface_types(
            &current_slices,
            Some(&lower_slices),
            Some(&upper_slices),
            0.01,
        );

        // Middle layer with same geometry above and below should be internal
        assert_eq!(surfaces.len(), 1);
        assert_eq!(surfaces[0].surface_type, SurfaceType::Internal);
    }

    /// Helper: create a rectangle ExPolygon from (x0,y0) to (x1,y1) in mm,
    /// automatically scaling to internal coordinates.
    fn make_rect_mm(x0: f64, y0: f64, x1: f64, y1: f64) -> ExPolygon {
        use crate::scale;
        let poly = Polygon::rectangle(
            Point::new(scale(x0), scale(y0)),
            Point::new(scale(x1), scale(y1)),
        );
        ExPolygon::new(poly)
    }

    #[test]
    fn test_propagate_solid_splits_surfaces() {
        // Manually construct surfaces to test splitting in isolation.
        //
        //   Layer 0: Internal (big 10×10 square — acts as filler, no top/bottom).
        //   Layer 1: Internal (big 10×10 square — target for propagation).
        //   Layer 2: Top (5×5 square centered at (2.5,2.5)→(7.5,7.5)).
        //
        // With top_solid_layers=2, bottom_solid_layers=1 (no bottom propagation),
        // propagation should split Layer 1's Internal surface: the overlap region
        // becomes InternalSolid, and the remainder stays Internal.
        //
        // Note: discover_horizontal_shells includes too-narrow filtering (matching
        // C++ PrintObject.cpp:3385) which expands narrow solid regions by ~1.35mm
        // so the fill algorithm can produce lines. We use a 5×5mm square here
        // (large enough to survive the opening filter without expansion) to test
        // the core splitting logic in isolation.

        let big_square = make_rect_mm(0.0, 0.0, 10.0, 10.0); // 100 mm²
        let medium_square = make_rect_mm(2.5, 2.5, 7.5, 7.5); // 25 mm²

        // Manually construct per-layer surfaces (no detect_surface_types to
        // avoid unrelated classification effects).
        let layer0_surfaces = vec![Surface::internal(big_square.clone())];
        let layer1_surfaces = vec![Surface::internal(big_square.clone())];
        let layer2_surfaces = vec![Surface::top(medium_square.clone())];

        let mut all_surfaces = vec![layer0_surfaces, layer1_surfaces, layer2_surfaces];

        // top_solid_layers=2 so layer 2's top propagates to layer 1.
        // bottom_solid_layers=1 so no bottom propagation beyond the source layer.
        let config = SurfaceDetectionConfig {
            top_solid_layers: 2,
            bottom_solid_layers: 1,
            offset: 0.0,
            min_area: 0.0,
            shell_growth: 0.0,
            fill_boundary_inset: 0.0,
            solid_infill_width: 0.0,
        };
        propagate_solid_infill(&mut all_surfaces, &config);

        // After propagation, layer 1 should have BOTH InternalSolid AND Internal surfaces.
        let internal_solid: Vec<_> = all_surfaces[1]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::InternalSolid)
            .collect();
        let internal_remaining: Vec<_> = all_surfaces[1]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::Internal)
            .collect();

        assert!(
            !internal_solid.is_empty(),
            "Layer 1 should have InternalSolid surfaces after propagation (the overlap with medium square)"
        );
        assert!(
            !internal_remaining.is_empty(),
            "Layer 1 should STILL have Internal surfaces after propagation (the non-overlapping remainder)"
        );

        // The solid area should be approximately 25mm² (the medium square region).
        // Too-narrow filtering may expand it slightly but it should stay well below 100mm².
        let solid_area_mm2: f64 = internal_solid
            .iter()
            .map(|s| s.expolygon.area().abs() / 1e12)
            .sum();
        assert!(
            solid_area_mm2 < 50.0,
            "Solid area ({:.1} mm²) should be much less than the full 100mm² square — \
             only the ~25mm² overlap region should be solid",
            solid_area_mm2
        );
        assert!(
            solid_area_mm2 > 10.0,
            "Solid area ({:.1} mm²) should be at least 10mm² (the medium square overlap is ~25mm²)",
            solid_area_mm2
        );

        // The remaining internal area should be the majority of the square.
        let internal_area_mm2: f64 = internal_remaining
            .iter()
            .map(|s| s.expolygon.area().abs() / 1e12)
            .sum();
        assert!(
            internal_area_mm2 > 40.0,
            "Remaining internal area ({:.1} mm²) should be the bulk of the 100mm² square",
            internal_area_mm2
        );
    }

    #[test]
    fn test_propagate_solid_small_region_without_regrowth() {
        // Test that a small 2×2mm top region propagates as-is (without
        // too-narrow regrowth, which is currently disabled pending Step 1).
        // The solid area at the target layer should be approximately 4mm²
        // — the raw intersection of the 2×2 top square with the 10×10
        // internal square below.
        //
        // When too-narrow filtering is re-enabled (after fill_surface
        // clipping is implemented), this test should be updated to expect
        // a larger solid area (~22mm²) due to regrowth.

        let big_square = make_rect_mm(0.0, 0.0, 10.0, 10.0);
        let small_square = make_rect_mm(4.0, 4.0, 6.0, 6.0); // 2×2 = 4 mm²

        let layer0_surfaces = vec![Surface::internal(big_square.clone())];
        let layer1_surfaces = vec![Surface::internal(big_square.clone())];
        let layer2_surfaces = vec![Surface::top(small_square.clone())];

        let mut all_surfaces = vec![layer0_surfaces, layer1_surfaces, layer2_surfaces];

        let config = SurfaceDetectionConfig {
            top_solid_layers: 2,
            bottom_solid_layers: 1,
            offset: 0.0,
            min_area: 0.0,
            shell_growth: 0.0,
            fill_boundary_inset: 0.0,
            solid_infill_width: 0.0,
        };
        propagate_solid_infill(&mut all_surfaces, &config);

        let internal_solid: Vec<_> = all_surfaces[1]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::InternalSolid)
            .collect();
        let internal_remaining: Vec<_> = all_surfaces[1]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::Internal)
            .collect();

        assert!(
            !internal_solid.is_empty(),
            "Layer 1 should have InternalSolid from propagation"
        );
        assert!(
            !internal_remaining.is_empty(),
            "Layer 1 should still have Internal surfaces (not everything should be solid)"
        );

        // Without too-narrow regrowth, the solid area should be approximately
        // 4mm² (the 2×2 intersection).
        let solid_area_mm2: f64 = internal_solid
            .iter()
            .map(|s| s.expolygon.area().abs() / 1e12)
            .sum();
        assert!(
            solid_area_mm2 > 1.0,
            "Solid area ({:.1} mm²) should be at least 1mm² (the 2×2 overlap is ~4mm²)",
            solid_area_mm2
        );
        assert!(
            solid_area_mm2 < 10.0,
            "Solid area ({:.1} mm²) should be close to 4mm² without regrowth",
            solid_area_mm2
        );
    }

    #[test]
    fn test_propagate_solid_no_overlap_stays_internal() {
        // If the top surface does NOT overlap the internal surface at all,
        // the internal surface should remain completely unchanged.
        // Use bottom_solid_layers=1 to prevent bottom propagation from
        // interfering.

        let left_square = make_rect_mm(0.0, 0.0, 4.0, 4.0); // 16 mm²
        let right_square = make_rect_mm(6.0, 6.0, 10.0, 10.0); // 16 mm²

        // Layer 0: filler (no top/bottom)
        let layer0_surfaces = vec![Surface::internal(left_square.clone())];

        // Layer 1: internal (left square)
        let layer1_surfaces = vec![Surface::internal(left_square.clone())];

        // Layer 2: top (right square — no overlap with left)
        let layer2_surfaces = vec![Surface::top(right_square.clone())];

        let mut all_surfaces = vec![layer0_surfaces, layer1_surfaces, layer2_surfaces];

        let config = SurfaceDetectionConfig {
            top_solid_layers: 2,
            bottom_solid_layers: 1,
            shell_growth: 0.0,
            fill_boundary_inset: 0.0,
            offset: 0.01,
            min_area: 0.5,
            solid_infill_width: 0.0,
        };
        propagate_solid_infill(&mut all_surfaces, &config);

        // Layer 1 should still be entirely Internal (no overlap with right_square)
        assert!(
            all_surfaces[1]
                .iter()
                .all(|s| s.surface_type == SurfaceType::Internal),
            "Non-overlapping surfaces should remain Internal"
        );
        assert_eq!(
            all_surfaces[1].len(),
            1,
            "Should still have exactly 1 surface (no splitting needed)"
        );
    }

    #[test]
    fn test_propagate_solid_full_overlap_converts_entirely() {
        // If the top surface fully covers the internal surface,
        // the entire surface should become InternalSolid (no Internal remainder).
        // Use bottom_solid_layers=1 to isolate top propagation.

        let big_square = make_rect_mm(0.0, 0.0, 10.0, 10.0);

        let layer0_surfaces = vec![Surface::internal(big_square.clone())];
        let layer1_surfaces = vec![Surface::internal(big_square.clone())];
        let layer2_surfaces = vec![Surface::top(big_square.clone())];

        let mut all_surfaces = vec![layer0_surfaces, layer1_surfaces, layer2_surfaces];

        let config = SurfaceDetectionConfig {
            top_solid_layers: 2,
            bottom_solid_layers: 1,
            shell_growth: 0.0,
            fill_boundary_inset: 0.0,
            offset: 0.01,
            min_area: 0.5,
            solid_infill_width: 0.0,
        };
        propagate_solid_infill(&mut all_surfaces, &config);

        // Layer 1 should be entirely InternalSolid.
        let solid: Vec<_> = all_surfaces[1]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::InternalSolid)
            .collect();
        let internal: Vec<_> = all_surfaces[1]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::Internal)
            .collect();

        assert!(
            !solid.is_empty(),
            "Full overlap should produce InternalSolid"
        );
        // The remainder (difference of identical shapes) should be empty or
        // produce only tiny slivers that get filtered by min_area.
        let remaining_area: f64 = internal
            .iter()
            .map(|s| s.expolygon.area().abs() / 1e12)
            .sum();
        assert!(
            remaining_area < 1.0,
            "Full overlap should leave negligible Internal area, got {:.2} mm²",
            remaining_area
        );
    }

    #[test]
    fn test_propagate_solid_tiny_overlap_filtered() {
        // A tiny overlap region smaller than min_area should NOT trigger
        // splitting — the surface should remain Internal.
        // Use bottom_solid_layers=1 to isolate top propagation.

        let big_square = make_rect_mm(0.0, 0.0, 10.0, 10.0);
        // Tiny square that barely overlaps the corner: overlap ≈ 0.2×0.2 = 0.04mm²
        let tiny_square = make_rect_mm(9.8, 9.8, 10.1, 10.1);

        let layer0_surfaces = vec![Surface::internal(big_square.clone())];
        let layer1_surfaces = vec![Surface::internal(big_square.clone())];
        let layer2_surfaces = vec![Surface::top(tiny_square.clone())];

        let mut all_surfaces = vec![layer0_surfaces, layer1_surfaces, layer2_surfaces];

        let config = SurfaceDetectionConfig {
            top_solid_layers: 2,
            bottom_solid_layers: 1,
            shell_growth: 0.0,
            fill_boundary_inset: 0.0,
            offset: 0.01,
            min_area: 0.5,
            solid_infill_width: 0.0,
        };
        propagate_solid_infill(&mut all_surfaces, &config);

        // The overlap is ~0.2mm × 0.2mm = 0.04mm² which is below min_area (0.5mm²).
        // So the Internal surface should remain as-is, with no InternalSolid created.
        let solid: Vec<_> = all_surfaces[1]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::InternalSolid)
            .collect();

        assert!(
            solid.is_empty(),
            "Tiny overlap ({:.3} mm²) below min_area should not create InternalSolid surfaces",
            solid
                .iter()
                .map(|s| s.expolygon.area().abs() / 1e12)
                .sum::<f64>()
        );
    }

    #[test]
    fn test_propagate_solid_bottom_upward_splits() {
        // Verify that bottom-surface upward propagation also splits correctly.
        // Use top_solid_layers=1 to isolate bottom propagation.
        //
        //   Layer 0: Bottom (small 3×3 square = 9mm²).
        //   Layer 1: Internal (big 10×10 square = 100mm²).
        //   Layer 2: filler.

        let big_square = make_rect_mm(0.0, 0.0, 10.0, 10.0);
        let small_square = make_rect_mm(2.0, 2.0, 5.0, 5.0); // 9 mm²

        let layer0_surfaces = vec![Surface::bottom(small_square.clone())];
        let layer1_surfaces = vec![Surface::internal(big_square.clone())];
        let layer2_surfaces = vec![Surface::internal(big_square.clone())];

        let mut all_surfaces = vec![layer0_surfaces, layer1_surfaces, layer2_surfaces];

        let config = SurfaceDetectionConfig {
            top_solid_layers: 1,
            bottom_solid_layers: 2,
            offset: 0.0,
            min_area: 0.0,
            shell_growth: 0.0,
            fill_boundary_inset: 0.0,
            solid_infill_width: 0.0,
        };
        propagate_solid_infill(&mut all_surfaces, &config);

        // Layer 1 should be split by bottom propagation from layer 0.
        let solid: Vec<_> = all_surfaces[1]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::InternalSolid)
            .collect();
        let internal: Vec<_> = all_surfaces[1]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::Internal)
            .collect();

        assert!(
            !solid.is_empty(),
            "Layer 1 should have InternalSolid from bottom propagation"
        );
        // The solid part should be ~9mm², not 100mm².
        let solid_area_mm2: f64 = solid.iter().map(|s| s.expolygon.area().abs() / 1e12).sum();
        assert!(
            solid_area_mm2 < 20.0,
            "Solid area ({:.1} mm²) should be ~9mm², not the full 100mm²",
            solid_area_mm2
        );

        // The remainder should still be Internal.
        assert!(
            !internal.is_empty(),
            "Layer 1 should still have Internal surfaces (the non-overlapping remainder)"
        );
        let internal_area_mm2: f64 = internal
            .iter()
            .map(|s| s.expolygon.area().abs() / 1e12)
            .sum();
        assert!(
            internal_area_mm2 > 70.0,
            "Remaining internal area ({:.1} mm²) should be the bulk (~91mm²)",
            internal_area_mm2
        );
    }

    #[test]
    fn test_propagate_preserves_non_internal_surfaces() {
        // Non-Internal surfaces (Top, Bottom, Bridge, etc.) should pass through
        // unchanged even if they overlap with shell regions.

        let big_square = make_rect_mm(0.0, 0.0, 10.0, 10.0);

        let layer0_surfaces = vec![Surface::internal(big_square.clone())];
        let layer1_surfaces = vec![
            Surface::top(big_square.clone()),
            Surface::bridge(big_square.clone(), Some(0.0)),
        ];
        let layer2_surfaces = vec![Surface::top(big_square.clone())];

        let mut all_surfaces = vec![layer0_surfaces, layer1_surfaces, layer2_surfaces];

        let config = SurfaceDetectionConfig {
            top_solid_layers: 2,
            bottom_solid_layers: 1,
            shell_growth: 0.0,
            fill_boundary_inset: 0.0,
            offset: 0.01,
            min_area: 0.5,
            solid_infill_width: 0.0,
        };
        propagate_solid_infill(&mut all_surfaces, &config);

        // Layer 1's Top and Bridge surfaces should be unchanged
        assert!(
            all_surfaces[1].iter().any(|s| s.is_top()),
            "Top surface should be preserved"
        );
        assert!(
            all_surfaces[1].iter().any(|s| s.is_bridge()),
            "Bridge surface should be preserved"
        );
        // No InternalSolid should be created (there were no Internal surfaces to split)
        assert!(
            !all_surfaces[1]
                .iter()
                .any(|s| s.surface_type == SurfaceType::InternalSolid),
            "No InternalSolid should appear when there are no Internal surfaces to split"
        );
    }

    // ── prepare_fill_surfaces tests ──────────────────────────────────

    /// Helper: make a surface of the given type with a rectangle of known area.
    /// `size_mm` is the side length in mm; area = size_mm² (in scaled coords).
    fn make_surface_with_area(stype: SurfaceType, size_mm: f64) -> Surface {
        let s = crate::scale(size_mm);
        let ep = ExPolygon::new(Polygon::rectangle(Point::new(0, 0), Point::new(s, s)));
        let mut surf = Surface::new(stype, ep);
        surf.thickness = 0.2;
        surf
    }

    #[test]
    fn test_prepare_fill_surfaces_demote_top_when_zero_layers() {
        // When top_solid_layers == 0, Top surfaces should become Internal
        let mut surfaces = vec![vec![
            make_surface_with_area(SurfaceType::Top, 10.0),
            make_surface_with_area(SurfaceType::Internal, 10.0),
            make_surface_with_area(SurfaceType::Bottom, 10.0),
        ]];

        let config = SurfaceDetectionConfig {
            top_solid_layers: 0,
            bottom_solid_layers: 3,
            ..SurfaceDetectionConfig::default()
        };

        prepare_fill_surfaces(&mut surfaces, &config, 0.0);

        // Top should have been demoted to Internal
        assert_eq!(surfaces[0][0].surface_type, SurfaceType::Internal);
        // Original Internal stays Internal
        assert_eq!(surfaces[0][1].surface_type, SurfaceType::Internal);
        // Bottom is untouched
        assert_eq!(surfaces[0][2].surface_type, SurfaceType::Bottom);
    }

    #[test]
    fn test_prepare_fill_surfaces_demote_bottom_when_zero_layers() {
        // When bottom_solid_layers == 0, Bottom surfaces should become Internal
        let mut surfaces = vec![vec![
            make_surface_with_area(SurfaceType::Top, 10.0),
            make_surface_with_area(SurfaceType::Bottom, 10.0),
            make_surface_with_area(SurfaceType::BottomBridge, 10.0),
        ]];

        let config = SurfaceDetectionConfig {
            top_solid_layers: 3,
            bottom_solid_layers: 0,
            ..SurfaceDetectionConfig::default()
        };

        prepare_fill_surfaces(&mut surfaces, &config, 0.0);

        // Top is untouched
        assert_eq!(surfaces[0][0].surface_type, SurfaceType::Top);
        // Bottom should have been demoted to Internal
        assert_eq!(surfaces[0][1].surface_type, SurfaceType::Internal);
        // BottomBridge is NOT demoted (only plain Bottom is)
        assert_eq!(surfaces[0][2].surface_type, SurfaceType::BottomBridge);
    }

    #[test]
    fn test_prepare_fill_surfaces_demote_both_when_zero() {
        // When both top and bottom solid layers are 0
        let mut surfaces = vec![vec![
            make_surface_with_area(SurfaceType::Top, 5.0),
            make_surface_with_area(SurfaceType::Bottom, 5.0),
            make_surface_with_area(SurfaceType::InternalSolid, 5.0),
        ]];

        let config = SurfaceDetectionConfig {
            top_solid_layers: 0,
            bottom_solid_layers: 0,
            ..SurfaceDetectionConfig::default()
        };

        prepare_fill_surfaces(&mut surfaces, &config, 0.0);

        assert_eq!(surfaces[0][0].surface_type, SurfaceType::Internal);
        assert_eq!(surfaces[0][1].surface_type, SurfaceType::Internal);
        // InternalSolid is not affected by the demotion logic
        assert_eq!(surfaces[0][2].surface_type, SurfaceType::InternalSolid);
    }

    #[test]
    fn test_prepare_fill_surfaces_promote_small_internal_to_solid() {
        // A tiny Internal region (1mm × 1mm = 1 mm²) should be promoted
        // when the threshold is 2 mm²
        let mut surfaces = vec![vec![
            make_surface_with_area(SurfaceType::Internal, 1.0), // 1 mm² — below threshold
            make_surface_with_area(SurfaceType::Internal, 5.0), // 25 mm² — above threshold
            make_surface_with_area(SurfaceType::Top, 1.0),      // Top, not affected
        ]];

        let config = SurfaceDetectionConfig::default();
        let threshold_mm2 = 2.0; // promote Internal regions < 2 mm²

        prepare_fill_surfaces(&mut surfaces, &config, threshold_mm2);

        // Small Internal → InternalSolid
        assert_eq!(
            surfaces[0][0].surface_type,
            SurfaceType::InternalSolid,
            "Small Internal surface should be promoted to InternalSolid"
        );
        // Large Internal stays Internal
        assert_eq!(
            surfaces[0][1].surface_type,
            SurfaceType::Internal,
            "Large Internal surface should remain Internal"
        );
        // Top is unaffected
        assert_eq!(surfaces[0][2].surface_type, SurfaceType::Top);
    }

    #[test]
    fn test_prepare_fill_surfaces_zero_threshold_no_promotion() {
        // With threshold == 0, no area-based promotion should happen
        let mut surfaces = vec![vec![
            make_surface_with_area(SurfaceType::Internal, 0.5), // tiny
        ]];

        let config = SurfaceDetectionConfig::default();
        prepare_fill_surfaces(&mut surfaces, &config, 0.0);

        assert_eq!(
            surfaces[0][0].surface_type,
            SurfaceType::Internal,
            "No promotion should happen when threshold is 0"
        );
    }

    #[test]
    fn test_prepare_fill_surfaces_multiple_layers() {
        // Operates independently on each layer
        let mut surfaces = vec![
            vec![make_surface_with_area(SurfaceType::Internal, 1.0)],
            vec![make_surface_with_area(SurfaceType::Internal, 5.0)],
            vec![make_surface_with_area(SurfaceType::Internal, 1.0)],
        ];

        let config = SurfaceDetectionConfig::default();
        prepare_fill_surfaces(&mut surfaces, &config, 2.0);

        assert_eq!(surfaces[0][0].surface_type, SurfaceType::InternalSolid);
        assert_eq!(surfaces[1][0].surface_type, SurfaceType::Internal);
        assert_eq!(surfaces[2][0].surface_type, SurfaceType::InternalSolid);
    }

    #[test]
    fn test_prepare_fill_surfaces_demotion_then_promotion() {
        // Top demoted to Internal, then promoted to InternalSolid if tiny
        let mut surfaces = vec![vec![
            make_surface_with_area(SurfaceType::Top, 1.0), // 1 mm²
        ]];

        let config = SurfaceDetectionConfig {
            top_solid_layers: 0,
            ..SurfaceDetectionConfig::default()
        };

        prepare_fill_surfaces(&mut surfaces, &config, 2.0);

        // Top → Internal (demotion) → InternalSolid (area promotion)
        assert_eq!(
            surfaces[0][0].surface_type,
            SurfaceType::InternalSolid,
            "Demoted Top that is small should be promoted to InternalSolid"
        );
    }

    #[test]
    fn test_prepare_fill_surfaces_only_internal_promoted() {
        // InternalSolid, Top, Bottom, BottomBridge are NOT touched by area promotion
        let mut surfaces = vec![vec![
            make_surface_with_area(SurfaceType::InternalSolid, 0.5),
            make_surface_with_area(SurfaceType::Top, 0.5),
            make_surface_with_area(SurfaceType::Bottom, 0.5),
            make_surface_with_area(SurfaceType::BottomBridge, 0.5),
            make_surface_with_area(SurfaceType::InternalBridge, 0.5),
        ]];

        let config = SurfaceDetectionConfig::default();
        prepare_fill_surfaces(&mut surfaces, &config, 100.0); // huge threshold

        // None of these should change
        assert_eq!(surfaces[0][0].surface_type, SurfaceType::InternalSolid);
        assert_eq!(surfaces[0][1].surface_type, SurfaceType::Top);
        assert_eq!(surfaces[0][2].surface_type, SurfaceType::Bottom);
        assert_eq!(surfaces[0][3].surface_type, SurfaceType::BottomBridge);
        assert_eq!(surfaces[0][4].surface_type, SurfaceType::InternalBridge);
    }

    // ── discover_vertical_shells tests ───────────────────────────────

    #[test]
    fn test_vertical_shells_noop_when_growth_zero() {
        // With shell_growth = 0.0, the function should be a no-op
        let mut surfaces = vec![vec![
            make_surface_with_area(SurfaceType::Top, 5.0),
            make_surface_with_area(SurfaceType::Internal, 10.0),
        ]];

        let config = SurfaceDetectionConfig {
            shell_growth: 0.0,
            ..SurfaceDetectionConfig::default()
        };

        let before_count = surfaces[0].len();
        discover_vertical_shells(&mut surfaces, &config);
        assert_eq!(
            surfaces[0].len(),
            before_count,
            "No-op when shell_growth is 0"
        );
    }

    #[test]
    fn test_vertical_shells_grows_solid_into_adjacent_internal() {
        // A small Top surface next to a larger Internal surface.
        // With shell_growth, the solid region should expand into the Internal region.
        //
        // Layout (in mm, scaled to coords):
        //   [0, 10] × [0, 10] = Top surface
        //   [10, 30] × [0, 10] = Internal surface (adjacent on the right)
        //
        // With shell_growth = 2.0mm, the Top grows rightward into Internal.

        let top_ep = ExPolygon::new(Polygon::rectangle(
            Point::new(0, 0),
            Point::new(crate::scale(10.0), crate::scale(10.0)),
        ));
        let internal_ep = ExPolygon::new(Polygon::rectangle(
            Point::new(crate::scale(10.0), 0),
            Point::new(crate::scale(30.0), crate::scale(10.0)),
        ));

        let mut surfaces = vec![vec![
            Surface::new(SurfaceType::Top, top_ep),
            Surface::new(SurfaceType::Internal, internal_ep),
        ]];

        let config = SurfaceDetectionConfig {
            shell_growth: 2.0, // 2mm growth
            min_area: 0.1,
            ..SurfaceDetectionConfig::default()
        };

        discover_vertical_shells(&mut surfaces, &config);

        // After growth, there should be:
        // - The original Top surface (unchanged)
        // - Some InternalSolid (the part of Internal that overlaps with grown solid)
        // - Some remaining Internal (the part that wasn't covered)
        let has_internal_solid = surfaces[0]
            .iter()
            .any(|s| s.surface_type == SurfaceType::InternalSolid);
        assert!(
            has_internal_solid,
            "Shell growth should create InternalSolid from adjacent Internal"
        );

        // The original Top surface should still be present
        let has_top = surfaces[0]
            .iter()
            .any(|s| s.surface_type == SurfaceType::Top);
        assert!(has_top, "Original Top surface should be preserved");

        // There should still be some remaining Internal (growth was only 2mm
        // into a 20mm wide Internal region)
        let has_internal = surfaces[0]
            .iter()
            .any(|s| s.surface_type == SurfaceType::Internal);
        assert!(
            has_internal,
            "Some Internal should remain after partial shell growth"
        );
    }

    #[test]
    fn test_vertical_shells_no_internal_no_change() {
        // If a layer has only solid surfaces (no Internal), nothing changes
        let mut surfaces = vec![vec![
            make_surface_with_area(SurfaceType::Top, 10.0),
            make_surface_with_area(SurfaceType::Bottom, 10.0),
        ]];

        let config = SurfaceDetectionConfig {
            shell_growth: 2.0,
            min_area: 0.1,
            ..SurfaceDetectionConfig::default()
        };

        discover_vertical_shells(&mut surfaces, &config);

        // No Internal existed, so no InternalSolid should be created
        assert!(
            !surfaces[0]
                .iter()
                .any(|s| s.surface_type == SurfaceType::InternalSolid),
            "No InternalSolid without Internal surfaces to convert"
        );
        assert_eq!(surfaces[0].len(), 2);
    }

    #[test]
    fn test_vertical_shells_no_solid_no_change() {
        // If a layer has only Internal surfaces (no solid to grow), nothing changes
        let mut surfaces = vec![vec![make_surface_with_area(SurfaceType::Internal, 10.0)]];

        let config = SurfaceDetectionConfig {
            shell_growth: 2.0,
            min_area: 0.1,
            ..SurfaceDetectionConfig::default()
        };

        discover_vertical_shells(&mut surfaces, &config);

        assert_eq!(surfaces[0].len(), 1);
        assert_eq!(surfaces[0][0].surface_type, SurfaceType::Internal);
    }

    #[test]
    fn test_vertical_shells_multiple_layers_independent() {
        // Each layer is processed independently
        let top_ep = ExPolygon::new(Polygon::rectangle(
            Point::new(0, 0),
            Point::new(crate::scale(5.0), crate::scale(5.0)),
        ));
        let internal_ep = ExPolygon::new(Polygon::rectangle(
            Point::new(crate::scale(5.0), 0),
            Point::new(crate::scale(20.0), crate::scale(5.0)),
        ));

        let mut surfaces = vec![
            // Layer 0: has solid + internal → should grow
            vec![
                Surface::new(SurfaceType::Top, top_ep.clone()),
                Surface::new(SurfaceType::Internal, internal_ep.clone()),
            ],
            // Layer 1: only internal → no growth
            vec![Surface::new(SurfaceType::Internal, internal_ep.clone())],
        ];

        let config = SurfaceDetectionConfig {
            shell_growth: 2.0,
            min_area: 0.1,
            ..SurfaceDetectionConfig::default()
        };

        discover_vertical_shells(&mut surfaces, &config);

        // Layer 0 should have InternalSolid
        assert!(
            surfaces[0]
                .iter()
                .any(|s| s.surface_type == SurfaceType::InternalSolid),
            "Layer 0 should have InternalSolid from shell growth"
        );

        // Layer 1 should NOT have InternalSolid (no solid surfaces to grow)
        assert!(
            !surfaces[1]
                .iter()
                .any(|s| s.surface_type == SurfaceType::InternalSolid),
            "Layer 1 should not be affected (no solid surfaces)"
        );
    }

    // ── process_external_surfaces tests ──────────────────────────────

    #[test]
    fn test_process_external_surfaces_noop_when_zero_expansion() {
        // With expansion_distance = 0.0, the function should be a no-op
        let mut surfaces = vec![vec![
            make_surface_with_area(SurfaceType::Top, 5.0),
            make_surface_with_area(SurfaceType::Internal, 10.0),
        ]];

        let before_count = surfaces[0].len();
        process_external_surfaces(&mut surfaces, 0.0, 0.5);
        assert_eq!(
            surfaces[0].len(),
            before_count,
            "No-op when expansion_distance is 0"
        );
    }

    #[test]
    fn test_process_external_surfaces_top_expands_into_internal() {
        // A small Top surface next to a large Internal surface.
        // After expansion, the Top should grow into the Internal region.
        //
        // Layout:
        //   [0, 10mm] × [0, 10mm] = Top
        //   [10mm, 30mm] × [0, 10mm] = Internal
        //
        // With 2mm expansion, Top should grow ~2mm into Internal.

        let top_ep = ExPolygon::new(Polygon::rectangle(
            Point::new(0, 0),
            Point::new(crate::scale(10.0), crate::scale(10.0)),
        ));
        let internal_ep = ExPolygon::new(Polygon::rectangle(
            Point::new(crate::scale(10.0), 0),
            Point::new(crate::scale(30.0), crate::scale(10.0)),
        ));

        let mut surfaces = vec![vec![
            Surface::new(SurfaceType::Top, top_ep),
            Surface::new(SurfaceType::Internal, internal_ep),
        ]];

        process_external_surfaces(&mut surfaces, 2.0, 0.1);

        // Top surface should still be present (and larger)
        let top_area: f64 = surfaces[0]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::Top)
            .map(|s| s.area().abs())
            .sum();
        // Original Top was 10×10 = 100 mm² (in scaled: 100e12).
        // After expansion, it should be larger (grew into Internal).
        let original_top_area = 100.0 * 1e12;
        assert!(
            top_area > original_top_area,
            "Top surface should have grown: area={top_area}, original={original_top_area}"
        );

        // Internal surface should have shrunk
        let internal_area: f64 = surfaces[0]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::Internal)
            .map(|s| s.area().abs())
            .sum();
        let original_internal_area = 200.0 * 1e12; // 20×10 = 200 mm²
        assert!(
            internal_area < original_internal_area,
            "Internal should have shrunk: area={internal_area}, original={original_internal_area}"
        );
    }

    #[test]
    fn test_process_external_surfaces_bottom_expands_too() {
        // Bottom surfaces should also expand, not just Top.
        let bottom_ep = ExPolygon::new(Polygon::rectangle(
            Point::new(0, 0),
            Point::new(crate::scale(5.0), crate::scale(5.0)),
        ));
        let internal_ep = ExPolygon::new(Polygon::rectangle(
            Point::new(crate::scale(5.0), 0),
            Point::new(crate::scale(25.0), crate::scale(5.0)),
        ));

        let mut surfaces = vec![vec![
            Surface::new(SurfaceType::Bottom, bottom_ep),
            Surface::new(SurfaceType::Internal, internal_ep),
        ]];

        process_external_surfaces(&mut surfaces, 2.0, 0.1);

        let bottom_area: f64 = surfaces[0]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::Bottom)
            .map(|s| s.area().abs())
            .sum();
        let original_bottom_area = 25.0 * 1e12; // 5×5 = 25 mm²
        assert!(
            bottom_area > original_bottom_area,
            "Bottom surface should have grown: area={bottom_area}, original={original_bottom_area}"
        );
    }

    #[test]
    fn test_process_external_surfaces_no_fill_area_no_change() {
        // If there's no Internal/InternalSolid, nothing can be expanded into
        let mut surfaces = vec![vec![
            make_surface_with_area(SurfaceType::Top, 10.0),
            make_surface_with_area(SurfaceType::Bottom, 10.0),
        ]];

        process_external_surfaces(&mut surfaces, 2.0, 0.1);

        // Should still have Top and Bottom, no new types
        assert!(surfaces[0]
            .iter()
            .any(|s| s.surface_type == SurfaceType::Top));
        assert!(surfaces[0]
            .iter()
            .any(|s| s.surface_type == SurfaceType::Bottom));
        assert!(
            !surfaces[0]
                .iter()
                .any(|s| s.surface_type == SurfaceType::Internal),
            "No Internal should appear when none existed"
        );
    }

    #[test]
    fn test_process_external_surfaces_preserves_other_types() {
        // InternalBridge and InternalVoid should be preserved unchanged
        let top_ep = ExPolygon::new(Polygon::rectangle(
            Point::new(0, 0),
            Point::new(crate::scale(5.0), crate::scale(5.0)),
        ));
        let internal_ep = ExPolygon::new(Polygon::rectangle(
            Point::new(crate::scale(5.0), 0),
            Point::new(crate::scale(15.0), crate::scale(5.0)),
        ));
        // InternalBridge placed far away so it's not overlapped
        let bridge_ep = ExPolygon::new(Polygon::rectangle(
            Point::new(crate::scale(50.0), crate::scale(50.0)),
            Point::new(crate::scale(60.0), crate::scale(60.0)),
        ));

        let mut surfaces = vec![vec![
            Surface::new(SurfaceType::Top, top_ep),
            Surface::new(SurfaceType::Internal, internal_ep),
            Surface::new(SurfaceType::InternalBridge, bridge_ep),
        ]];

        process_external_surfaces(&mut surfaces, 2.0, 0.1);

        assert!(
            surfaces[0]
                .iter()
                .any(|s| s.surface_type == SurfaceType::InternalBridge),
            "InternalBridge should be preserved"
        );
    }

    #[test]
    fn test_process_external_surfaces_expands_into_internal_solid() {
        // External surfaces should also expand into InternalSolid, not just Internal
        let top_ep = ExPolygon::new(Polygon::rectangle(
            Point::new(0, 0),
            Point::new(crate::scale(5.0), crate::scale(5.0)),
        ));
        let solid_ep = ExPolygon::new(Polygon::rectangle(
            Point::new(crate::scale(5.0), 0),
            Point::new(crate::scale(25.0), crate::scale(5.0)),
        ));

        let mut surfaces = vec![vec![
            Surface::new(SurfaceType::Top, top_ep),
            Surface::new(SurfaceType::InternalSolid, solid_ep),
        ]];

        process_external_surfaces(&mut surfaces, 2.0, 0.1);

        // Top should have grown
        let top_area: f64 = surfaces[0]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::Top)
            .map(|s| s.area().abs())
            .sum();
        let original_top_area = 25.0 * 1e12; // 5×5
        assert!(
            top_area > original_top_area,
            "Top should expand into InternalSolid: area={top_area}, original={original_top_area}"
        );

        // InternalSolid should have shrunk
        let solid_area: f64 = surfaces[0]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::InternalSolid)
            .map(|s| s.area().abs())
            .sum();
        let original_solid_area = 100.0 * 1e12; // 20×5
        assert!(
            solid_area < original_solid_area,
            "InternalSolid should shrink: area={solid_area}, original={original_solid_area}"
        );
    }

    #[test]
    fn test_process_external_surfaces_multiple_layers() {
        // Each layer is independent
        let top_ep = ExPolygon::new(Polygon::rectangle(
            Point::new(0, 0),
            Point::new(crate::scale(5.0), crate::scale(5.0)),
        ));
        let internal_ep = ExPolygon::new(Polygon::rectangle(
            Point::new(crate::scale(5.0), 0),
            Point::new(crate::scale(20.0), crate::scale(5.0)),
        ));

        let mut surfaces = vec![
            // Layer 0: Top + Internal → should expand
            vec![
                Surface::new(SurfaceType::Top, top_ep.clone()),
                Surface::new(SurfaceType::Internal, internal_ep.clone()),
            ],
            // Layer 1: only Internal → no external surfaces, nothing to expand
            vec![Surface::new(SurfaceType::Internal, internal_ep.clone())],
        ];

        process_external_surfaces(&mut surfaces, 2.0, 0.1);

        // Layer 0: Top should have grown
        let top_area_l0: f64 = surfaces[0]
            .iter()
            .filter(|s| s.surface_type == SurfaceType::Top)
            .map(|s| s.area().abs())
            .sum();
        assert!(top_area_l0 > 25.0 * 1e12, "Layer 0 Top should expand");

        // Layer 1: only Internal, no external surfaces to grow
        assert!(
            !surfaces[1]
                .iter()
                .any(|s| s.surface_type == SurfaceType::Top),
            "Layer 1 should have no Top surface"
        );
        assert!(
            surfaces[1]
                .iter()
                .any(|s| s.surface_type == SurfaceType::Internal),
            "Layer 1 Internal should remain"
        );
    }
}
