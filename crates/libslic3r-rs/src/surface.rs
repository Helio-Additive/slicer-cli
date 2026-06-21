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
#[derive(Clone, Serialize, Deserialize)]
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

impl Default for Surface {
    // C++ default ctor: Surface(SurfaceType _surface_type = stInternal)
    //   : surface_type(stInternal), thickness(-1), thickness_layers(1),
    //     bridge_angle(-1), extra_perimeters(0)
    // Surface.hpp:44-47
    fn default() -> Self {
        Self {
            expolygon: ExPolygon::default(),
            surface_type: SurfaceType::Internal,
            thickness: -1.0,
            thickness_layers: 1,
            bridge_angle: None,
            extra_perimeters: 0,
        }
    }
}

impl Surface {
    // Create a new surface with the given type and geometry.
    // Surface.hpp:54-57 Surface(SurfaceType, const ExPolygon&)
    //   : thickness(-1), thickness_layers(1), bridge_angle(-1), extra_perimeters(0)
    pub fn new(surface_type: SurfaceType, expolygon: ExPolygon) -> Self {
        Self {
            expolygon,
            surface_type,
            thickness: -1.0,
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
    /// (Rust convenience; mirrors Surface(stBottomBridge, expoly) with bridge_angle set.)
    pub fn bridge(expolygon: ExPolygon, angle: Option<CoordF>) -> Self {
        Self {
            expolygon,
            surface_type: SurfaceType::BottomBridge,
            thickness: -1.0,
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

// inline Polygons to_polygons(const SurfacesPtr &src)
/// Surface.hpp:143
pub fn to_polygons_ptr(src: &[&Surface]) -> crate::geometry::Polygons {
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

// inline ExPolygons to_expolygons(const SurfacesPtr &src)
/// Surface.hpp:177
pub fn to_expolygons_ptr(src: &[&Surface]) -> ExPolygons {
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

// inline size_t number_polygons(const SurfacesPtr &surfaces)
/// Surface.hpp:195
pub fn number_polygons_ptr(surfaces: &[&Surface]) -> usize {
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

// Append a vector of Surfaces at the end of another vector of polygons.
// inline void polygons_append(Polygons &dst, const SurfacesPtr &src)
/// Surface.hpp:224
pub fn polygons_append_ptr(dst: &mut crate::geometry::Polygons, src: &[&Surface]) {
    dst.reserve(dst.len() + number_polygons_ptr(src));
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

// void export_surface_type_legend_to_svg(SVG &svg, const Point &pos)
/// Surface.cpp:55
pub fn export_surface_type_legend_to_svg(svg: &mut crate::svg::SVG, pos: &crate::geometry::Point) {
    // 1st row                                                            Surface.cpp:57
    // coord_t pos_x0 = pos(0) + scale_(1.);                             Surface.cpp:58
    let pos_x0: crate::Coord = pos.x() + crate::scale(1.);
    // coord_t pos_x = pos_x0;                                            Surface.cpp:59
    let mut pos_x: crate::Coord = pos_x0;
    // coord_t pos_y = pos(1) + scale_(1.5);                             Surface.cpp:60
    let pos_y: crate::Coord = pos.y() + crate::scale(1.5);
    // coord_t step_x = scale_(10.);                                      Surface.cpp:61
    let step_x: crate::Coord = crate::scale(10.);
    // svg.draw_legend(Point(pos_x, pos_y), "perimeter"      , surface_type_to_color_name(stPerimeter));  Surface.cpp:62
    svg.draw_legend(
        &crate::geometry::Point::new(pos_x, pos_y),
        "perimeter",
        surface_type_to_color_name(SurfaceType::Perimeter),
    );
    // pos_x += step_x;                                                   Surface.cpp:63
    pos_x += step_x;
    // svg.draw_legend(Point(pos_x, pos_y), "top"            , surface_type_to_color_name(stTop));         Surface.cpp:64
    svg.draw_legend(
        &crate::geometry::Point::new(pos_x, pos_y),
        "top",
        surface_type_to_color_name(SurfaceType::Top),
    );
    // pos_x += step_x;                                                   Surface.cpp:65
    pos_x += step_x;
    // svg.draw_legend(Point(pos_x, pos_y), "bottom"         , surface_type_to_color_name(stBottom));      Surface.cpp:66
    svg.draw_legend(
        &crate::geometry::Point::new(pos_x, pos_y),
        "bottom",
        surface_type_to_color_name(SurfaceType::Bottom),
    );
    // pos_x += step_x;                                                   Surface.cpp:67
    pos_x += step_x;
    // svg.draw_legend(Point(pos_x, pos_y), "bottom bridge"  , surface_type_to_color_name(stBottomBridge)); Surface.cpp:68
    svg.draw_legend(
        &crate::geometry::Point::new(pos_x, pos_y),
        "bottom bridge",
        surface_type_to_color_name(SurfaceType::BottomBridge),
    );
    // pos_x += step_x;                                                   Surface.cpp:69
    pos_x += step_x;
    // svg.draw_legend(Point(pos_x, pos_y), "invalid"        , surface_type_to_color_name(SurfaceType(-1))); Surface.cpp:70
    svg.draw_legend(
        &crate::geometry::Point::new(pos_x, pos_y),
        "invalid",
        // C++ casts -1 to SurfaceType (out of range) → the switch `default:`
        // branch returns "rgb(64,64,64)". `from_u8` maps unknown values to the
        // Internal fallback, which would give the wrong colour here, so call the
        // color helper through the default branch by passing an invalid value.
        surface_type_invalid_color_name(),
    );
    // 2nd row                                                            Surface.cpp:71
    // pos_x = pos_x0;                                                    Surface.cpp:72
    pos_x = pos_x0;
    // pos_y = pos(1)+scale_(2.8);                                        Surface.cpp:73
    let pos_y: crate::Coord = pos.y() + crate::scale(2.8);
    // svg.draw_legend(Point(pos_x, pos_y), "internal"       , surface_type_to_color_name(stInternal));    Surface.cpp:74
    svg.draw_legend(
        &crate::geometry::Point::new(pos_x, pos_y),
        "internal",
        surface_type_to_color_name(SurfaceType::Internal),
    );
    // pos_x += step_x;                                                   Surface.cpp:75
    pos_x += step_x;
    // svg.draw_legend(Point(pos_x, pos_y), "internal solid" , surface_type_to_color_name(stInternalSolid)); Surface.cpp:76
    svg.draw_legend(
        &crate::geometry::Point::new(pos_x, pos_y),
        "internal solid",
        surface_type_to_color_name(SurfaceType::InternalSolid),
    );
    // pos_x += step_x;                                                   Surface.cpp:77
    pos_x += step_x;
    // svg.draw_legend(Point(pos_x, pos_y), "internal bridge", surface_type_to_color_name(stInternalBridge)); Surface.cpp:78
    svg.draw_legend(
        &crate::geometry::Point::new(pos_x, pos_y),
        "internal bridge",
        surface_type_to_color_name(SurfaceType::InternalBridge),
    );
    // pos_x += step_x;                                                   Surface.cpp:79
    pos_x += step_x;
    // svg.draw_legend(Point(pos_x, pos_y), "internal void"  , surface_type_to_color_name(stInternalVoid)); Surface.cpp:80
    svg.draw_legend(
        &crate::geometry::Point::new(pos_x, pos_y),
        "internal void",
        surface_type_to_color_name(SurfaceType::InternalVoid),
    );
}

// Helper: reproduce the `surface_type_to_color_name(SurfaceType(-1))` default
// branch (Surface.cpp:46), which the Rust enum cannot represent as a value.
#[inline]
fn surface_type_invalid_color_name() -> &'static str {
    // default:                return "rgb(64,64,64)";                    Surface.cpp:46
    "rgb(64,64,64)"
}

// bool export_to_svg(const char *path, const Surfaces &surfaces, const float transparency)
/// Surface.cpp:83
pub fn export_to_svg(path: &str, surfaces: &[Surface], transparency: f32) -> bool {
    // BoundingBox bbox;                                                  Surface.cpp:85
    let mut bbox = crate::geometry::BoundingBox::new();
    // for (Surfaces::const_iterator surface = surfaces.begin(); surface != surfaces.end(); ++surface)
    //     bbox.merge(get_extents(surface->expolygon));                   Surface.cpp:86-87
    for surface in surfaces {
        bbox.merge(&crate::geometry::get_extents_expoly(&surface.expolygon));
    }

    // SVG svg(path, bbox);                                               Surface.cpp:89
    let mut svg = crate::svg::SVG::new_bbox_default(path, &bbox);
    // for (Surfaces::const_iterator surface = surfaces.begin(); surface != surfaces.end(); ++surface)
    //     svg.draw(surface->expolygon, surface_type_to_color_name(surface->surface_type), transparency);  Surface.cpp:90-91
    for surface in surfaces {
        svg.draw_expolygon(
            &surface.expolygon,
            surface_type_to_color_name(surface.surface_type),
            transparency,
        );
    }
    // svg.Close();                                                       Surface.cpp:92
    svg.close();
    // return true;                                                       Surface.cpp:93
    true
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

// `SurfaceCollection` (`class SurfaceCollection`, SurfaceCollection.hpp:10) — the
// struct definition + all its impls now live in `crate::surface_collection`
// (mirroring the C++ file SurfaceCollection.{hpp,cpp}). Only the `Surfaces` /
// `SurfacesPtr` typedefs (Surface.hpp:115-116) belong here.


#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{Point, Polygon};
    use crate::surface_collection::SurfaceCollection;
    // The surface-classification fns these tests exercise were relocated to
    // crate::print_object (they mirror PrintObject/LayerRegion, not Surface.cpp).
    use crate::print_object::{
        clip_surfaces_to_fill_boundaries, detect_all_surface_types, detect_surface_types,
        discover_horizontal_shells, discover_vertical_shells, prepare_fill_surfaces,
        process_external_surfaces, propagate_solid_infill, split_internal_surfaces,
        SurfaceDetectionConfig,
    };

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
