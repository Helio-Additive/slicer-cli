//! Text embossing and 3D projection utilities.
//!
//! C++ Reference:
//! - BambuStudio/src/libslic3r/Emboss.hpp
//! - BambuStudio/src/libslic3r/Emboss.cpp
//! - BambuStudio/src/imgui/imstb_truetype.h (the stb_truetype functions Emboss.cpp calls, in
//!   the inner module [`imstb_truetype`])
//!
//! Faithful 1:1 line-by-line port. `// Emboss.cpp:NNN` / `// Emboss.hpp:NNN`
//! comments map each region back to the C++ source.
//!
//! The polygon operations go through the crate's ClipperLib port (`crate::clipper`) and the
//! `ClipperUtils.cpp` templates of `crate::clipper_utils`, call for call as Emboss.cpp makes them
//! (`SimplifyPolygons`, `CleanPolygons`, `union_ex`, `diff_ex`, `offset_ex`).
//!
//! C++ overloads: Rust has no overloading, so a second (third) overload of a name is written in
//! a small module named after what tells it apart (`for_expolygons::remove_bad`,
//! `cached::get_glyph`, `from_path::create_font_file`, ...), keeping the C++ name.
//!
//! `std::wstring` is a `[char]` here (UTF-32, as `wchar_t` is on macOS and Linux) and
//! `boost::nowide::widen` is `str::chars`.
//! FIDELITY-NOTE: on Windows `wchar_t` is UTF-16, so the C++ iterates a character outside the
//! Basic Multilingual Plane as two surrogates; here it is one `char` on every platform.
//!
//! ## Not ported
//!
//! - The `#ifdef _WIN32` font enumeration through the registry and GDI (`get_font_path`,
//!   `get_font_list` of Windows, `get_font_list_by_register`, `EnumFamCallBack`,
//!   `get_font_list_by_enumeration`, `get_font_list_by_folder`, `load_hfont`, `can_load`,
//!   `create_font_file(void *hfont)`): they need the registry and GDI entry points of the Win32
//!   API, which the crate's `winapi` dependency is not built with. The non-Windows arms of
//!   `get_font_list` and `get_font_path` are ported (they return nothing).
//! - `polygons2model` is ported, but the triangles of the front and back faces come from
//!   `crate::triangulation`, whose CGAL constrained Delaunay core is not ported yet (it returns
//!   no triangles); the vertices and the side quads are complete.

// Emboss.cpp:7     #include <ClipperUtils.hpp> // union_ex + for boldness(polygon extend(offset))
// Emboss.cpp:8     #include "IntersectionPoints.hpp"
// Emboss.cpp:20-23 #include "ExPolygonsIndex.hpp" / AABBTreeLines / Line / BoundingBox
use crate::emboss_shape::{EmbossShape, ExPolygonsWithId, ExPolygonsWithIds, Glyphs};
use crate::ex_polygons_index::ExPolygonsIndices;
use crate::geometry::geometry::{eigen, is_approx, Transform3d};
use crate::geometry::{
    collect_duplicates, get_extents as get_extents_expolygons, remove_same_neighbor, to_points, BoundingBox,
    ExPolygon, ExPolygons, Line, Lines, Point, PointF, Points, Polygon, Polygons, Vec3d,
};
use crate::intersection_points::{get_intersections_expolygons, IntersectionsLines};
#[cfg(windows)]
use crate::text_configuration::EmbossStyle;
use crate::text_configuration::{EmbossStyles, FontProp, HorizontalAlign};
use crate::triangle_mesh::{indexed_triangle_set, Vec3f};
use crate::aabb_tree_lines;
use imstb_truetype::{StbttFontinfo, StbttPoint};
use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

// `Vec2d` (Eigen Vec2d) maps to the crate's floating-point 2D point.
type Vec2d = PointF;

// Emboss.cpp:25-27
// Experimentaly suggested ration of font ascent by multiple fonts
// to get approx center of normal text line
// const double ASCENT_CENTER = 1/2.5; // 0.5 is above small letter
const ASCENT_CENTER: f64 = 1.0 / 2.5; // 0.5 is above small letter

// Emboss.cpp:29-31
// every glyph's shape point is divided by SHAPE_SCALE - increase precission of fixed point value
// stored in fonts (to be able represents curve by sequence of lines)
// static constexpr double SHAPE_SCALE = 0.001; // SCALING_FACTOR promile is fine enough
const SHAPE_SCALE: f64 = 0.001; // SCALING_FACTOR promile is fine enough

// Emboss.cpp:32  static unsigned MAX_HEAL_ITERATION_OF_TEXT = 10;
const MAX_HEAL_ITERATION_OF_TEXT: u32 = 10;

// Emboss.hpp:23  static const float UNION_DELTA = 50.0f; // [approx in nano meters depends on volume scale]
pub const UNION_DELTA: f32 = 50.0;
// Emboss.hpp:24  static const unsigned UNION_MAX_ITERATIN = 10; // [count]
pub const UNION_MAX_ITERATIN: u32 = 10;

// Emboss.hpp:96  const unsigned ENTER_UNICODE = static_cast<unsigned>('\n');
pub const ENTER_UNICODE: u32 = '\n' as u32;

// ===========================================================================
// Types from EmbossShape.hpp `namespace Emboss` and Emboss.hpp
// ===========================================================================

/// description of one letter
///
/// EmbossShape.hpp:43  struct Glyph
#[derive(Debug, Clone, Default)]
pub struct Glyph {
    // EmbossShape.hpp:45-47
    // NOTE: shape is scaled by SHAPE_SCALE
    // to be able store points without floating points
    pub shape: ExPolygons,

    // EmbossShape.hpp:49-50  values are in font points
    pub advance_width: i32,
    pub left_side_bearing: i32,
}

/// keep information from file about font (store file data itself) + cache data
///
/// EmbossShape.hpp:59  struct FontFile
#[derive(Debug, Clone)]
pub struct FontFile {
    // EmbossShape.hpp:61-65
    // loaded data from font file
    pub data: Option<Vec<u8>>,

    // EmbossShape.hpp:75-76  info for each font in data
    pub infos: Vec<FontFileInfo>,
}

impl FontFile {
    /// EmbossShape.hpp:78-85  `FontFile(std::unique_ptr<std::vector<unsigned char>> data,
    /// std::vector<Info> &&infos)`
    pub fn new(data: Vec<u8>, infos: Vec<FontFileInfo>) -> Self {
        // assert(this->data != nullptr); assert(!this->data->empty());
        FontFile { data: Some(data), infos }
    }
}

/// EmbossShape.hpp:67  struct Info
#[derive(Debug, Clone, Copy)]
pub struct FontFileInfo {
    // EmbossShape.hpp:69-70  vertical position is "scale*(ascent - descent + lineGap)"
    pub ascent: i32,
    pub descent: i32,
    pub linegap: i32,

    // EmbossShape.hpp:72-73  for convert font units to pixel
    pub unit_per_em: i32,
}

/// Add caching for shape of glyphs
///
/// EmbossShape.hpp:97  struct FontFileWithCache
///
/// FIDELITY-NOTE: the C++ holds the cache through a `std::shared_ptr<Glyphs>`, so copies of a
/// `FontFileWithCache` share it; here the cache is owned and a clone copies it.
#[derive(Debug, Clone, Default)]
pub struct FontFileWithCache {
    // EmbossShape.hpp:99-100  Pointer on data of the font file
    pub font_file: Option<Arc<FontFile>>,
    // EmbossShape.hpp:102-105  Cache for glyph shape
    pub cache: Option<Glyphs>,
}

impl FontFileWithCache {
    // EmbossShape.hpp:109  bool has_value() const { return font_file != nullptr && cache != nullptr; }
    pub fn has_value(&self) -> bool {
        self.font_file.is_some() && self.cache.is_some()
    }
}

/// Extend expolygons with information whether it was successfull healed
///
/// EmbossShape.hpp:36-40  struct HealedExPolygons
/// EmbossShape.hpp:213 cereal serialize archives `expolygons` then `is_healed`; the
/// derived serde impls mirror that field order.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealedExPolygons {
    pub expolygons: ExPolygons,
    pub is_healed: bool,
}

/// Define polygon for draw letters
///
/// Emboss.hpp:368  struct TextLine
#[derive(Debug, Clone)]
pub struct TextLine {
    // Emboss.hpp:371  slice of object
    pub polygon: Polygon,
    // Emboss.hpp:374  point laying on polygon closest to zero
    pub start: PolygonPoint,
    // Emboss.hpp:377  offset of text line in volume mm
    pub y: f32,
}

/// Emboss.hpp:379  using TextLines = std::vector<TextLine>;
pub type TextLines = Vec<TextLine>;

// `PolygonPoint` and `PolygonPoints` originate from EmbossShape/PolygonPoint.hpp
// (used by `sample_slice` / `calculate_angle`). Mirror the C++ structure: a point
// on a polygon, identified by an index of the segment it lies on plus the
// coordinate of the point itself.
//
// PolygonPoint.hpp  struct PolygonPoint { size_t index; Point point; };
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PolygonPoint {
    pub index: usize,
    pub point: Point,
}

/// PolygonPoint.hpp  using PolygonPoints = std::vector<PolygonPoint>;
pub type PolygonPoints = Vec<PolygonPoint>;

/// Emboss.hpp:101  using LineRanges = std::vector<std::pair<size_t, size_t>>;
pub type LineRanges = Vec<(usize, usize)>;

/// Emboss.hpp:76  typedef std::function<std::vector<FontFileWithCache>()> BackFontCacheFn;
pub type BackFontCacheFn<'f> = Option<&'f dyn Fn() -> Vec<FontFileWithCache>>;

/// `std::wstring` (see the module note).
pub type WString = Vec<char>;

/// `fontinfo_opt` (Emboss.cpp:36): `std::optional<stbtt_fontinfo>`.
type FontinfoOpt<'a> = Option<StbttFontinfo<'a>>;

/// The two members `text2vshapes` (Emboss.cpp:1515-1518) writes into `EmbossShape` that the
/// crate's `EmbossShape` (emboss_shape.rs) does not have yet: `line_height` and
/// `first_line_offset_y` (EmbossShape.hpp:196, 198).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct TextLineLayout {
    pub line_height: f32,
    pub first_line_offset_y: f32,
}

// ===========================================================================
// ClipperLib / ClipperUtils calls of Emboss.cpp
// ===========================================================================

// ClipperUtils.hpp  `ClipperUtils::PolygonsProvider`: every polygon is one path.
fn polygons_provider(polygons: &[Polygon]) -> crate::clipper::Paths {
    polygons.iter().map(|p| path_of_points(p.points())).collect()
}

// ClipperUtils.hpp  `ClipperUtils::ExPolygonsProvider`: the contour, then the holes, of every
// expolygon.
fn expolygons_provider(expolygons: &[ExPolygon]) -> crate::clipper::Paths {
    let mut paths = crate::clipper::Paths::new();
    for e in expolygons {
        paths.push(path_of_points(e.contour.points()));
        for h in &e.holes {
            paths.push(path_of_points(h.points()));
        }
    }
    paths
}

// The engine builds ClipperLib with `CLIPPERLIB_INT32`: `cInt` is 32 bits wide.
fn path_of_points(points: &[Point]) -> crate::clipper::Path {
    points.iter().map(|p| crate::clipper::IntPoint::new(p.x as i32, p.y as i32)).collect()
}

// ClipperUtils.cpp  `Polygons to_polygons(const ClipperLib::Paths &paths)`
fn to_polygons_paths(paths: &crate::clipper::Paths) -> Polygons {
    paths
        .iter()
        .map(|p| Polygon::from_points(p.iter().map(|q| Point::new(q.x as i64, q.y as i64)).collect()))
        .collect()
}

// ClipperUtils.cpp:570-571  `ExPolygons offset_ex(const ExPolygons &expolygons, const float delta,
// ClipperLib::JoinType joinType = DefaultJoinType, double miterLimit = DefaultMiterLimit)`
// with the defaults `jtMiter` and 3 (ClipperUtils.hpp:30-31).
fn offset_ex(expolygons: &[ExPolygon], delta: f32) -> ExPolygons {
    let rings: Vec<(crate::clipper::Path, crate::clipper::Paths)> = expolygons
        .iter()
        .map(|e| {
            (
                path_of_points(e.contour.points()),
                e.holes.iter().map(|h| path_of_points(h.points())).collect(),
            )
        })
        .collect();
    crate::clipper_utils::offset_ex_expolygons_paths(&rings, delta as f64, crate::clipper::JoinType::jtMiter, 3.0)
}

// ClipperUtils.cpp:813-814  `ExPolygons union_ex(const Polygons &subject, ClipperLib::PolyFillType
// fill_type)` = `_clipper_ex(ctUnion, PolygonsProvider(subject), EmptyPathsProvider(),
// ApplySafetyOffset::No, fill_type)`
fn union_ex_polygons(subject: &[Polygon], fill_type: crate::clipper::PolyFillType) -> ExPolygons {
    crate::clipper_utils::_clipper_ex(
        crate::clipper::ClipType::ctUnion,
        &polygons_provider(subject),
        &crate::clipper::Paths::new(),
        crate::clipper_utils::ApplySafetyOffset::No,
        fill_type,
    )
}

// ClipperUtils.cpp:815-816  `ExPolygons union_ex(const ExPolygons &subject)`
fn union_ex(subject: &[ExPolygon]) -> ExPolygons {
    crate::clipper_utils::_clipper_ex(
        crate::clipper::ClipType::ctUnion,
        &expolygons_provider(subject),
        &crate::clipper::Paths::new(),
        crate::clipper_utils::ApplySafetyOffset::No,
        crate::clipper::PolyFillType::pftNonZero,
    )
}

// ClipperUtils.cpp:817-818  `ExPolygons union_ex(const ExPolygons &subject, const Polygons &subject2)`
fn union_ex_expolygons_polygons(subject: &[ExPolygon], subject2: &[Polygon]) -> ExPolygons {
    crate::clipper_utils::_clipper_ex(
        crate::clipper::ClipType::ctUnion,
        &expolygons_provider(subject),
        &polygons_provider(subject2),
        crate::clipper_utils::ApplySafetyOffset::No,
        crate::clipper::PolyFillType::pftNonZero,
    )
}

// ClipperUtils.cpp:753-754  `ExPolygons diff_ex(const ExPolygons &subject, const Polygons &clip,
// ApplySafetyOffset do_safety_offset)`
fn diff_ex(
    subject: &[ExPolygon],
    clip: &[Polygon],
    do_safety_offset: crate::clipper_utils::ApplySafetyOffset,
) -> ExPolygons {
    crate::clipper_utils::_clipper_ex(
        crate::clipper::ClipType::ctDifference,
        &expolygons_provider(subject),
        &polygons_provider(clip),
        do_safety_offset,
        crate::clipper::PolyFillType::pftNonZero,
    )
}

// ===========================================================================
// Anonymous namespace helpers (Emboss.cpp:38-303)
// ===========================================================================

// Emboss.cpp:50-75  (#ifdef VISUALIZE_HEAL, for debug purpose only)
// NOTE: check scale when store svg !!
#[allow(dead_code)]
fn visualize_heal(svg_filepath: &str, expolygons: &ExPolygons) {
    // Emboss.cpp:59-60
    let pts = to_points(expolygons);
    let bb = BoundingBox::from_points(&pts);
    // double svg_scale = SHAPE_SCALE / unscale<double>(1.);
    //  bb.scale(svg_scale);
    // Emboss.cpp:63-64
    let mut svg = crate::svg::SVG::new_bbox_default(svg_filepath, &bb);
    svg.draw_expolygons(expolygons, "grey", 1.0);

    // Emboss.cpp:66-68
    let duplicits = collect_duplicates(pts);
    let bb_size = bb.size();
    let black_size: i32 = (bb_size.x.max(bb_size.y) / 20) as i32;
    svg.draw_points(&duplicits, "black", black_size as i64);

    // Emboss.cpp:70-72
    let intersections_f = get_intersections_expolygons(expolygons);
    let intersections = get_unique_intersections(&intersections_f);
    svg.draw_points(&intersections, "red", (black_size as f64 * 1.2) as i64);
}

// Emboss.cpp:101  const Points pts_2x2({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)});
fn pts_2x2() -> Points {
    vec![Point::new(0, 0), Point::new(1, 0), Point::new(1, 1), Point::new(0, 1)]
}

// Emboss.cpp:102  const Points pts_3x3({Point(-1, -1), Point(1, -1), Point(1, 1), Point(-1, 1)});
fn pts_3x3() -> Points {
    vec![Point::new(-1, -1), Point::new(1, -1), Point::new(1, 1), Point::new(-1, 1)]
}

// Emboss.cpp:104  struct SpikeDesc
pub struct SpikeDesc {
    // Emboss.cpp:106-107  cosinus of max spike angle (speed up to skip acos)
    cos_angle: f64,
    // Emboss.cpp:109-110  Half of Wanted bevel size
    half_bevel: f64,
}

impl SpikeDesc {
    // Emboss.cpp:117-127
    // SpikeDesc(double bevel_size, double pixel_spike_length = 6)
    pub fn new(bevel_size: f64, pixel_spike_length: f64) -> Self {
        SpikeDesc {
            // Emboss.cpp:118-122
            // create min angle given by spike_length
            // Use it as minimal height of 1 pixel base spike
            cos_angle: (2.0 * f64::atan2(pixel_spike_length, 0.5)).cos().abs(),
            // Emboss.cpp:124-126
            // When remove spike this angle is set.
            // Value must be grater than min_angle
            half_bevel: bevel_size / 2.0,
        }
    }
}

// Emboss.cpp:143-227
// spike ... very sharp corner - when not removed cause iteration of heal process
// index ... index of duplicit point in polygon
// return TRUE when remove point. It could create polygon with 2 points.
pub fn remove_when_spike(polygon: &mut Polygon, index: usize, spike_desc: &SpikeDesc) -> bool {
    // Emboss.cpp:147-149
    let mut add: Option<Point> = None;
    let mut do_erase = false;
    {
        // Emboss.cpp:151-153
        let pts = polygon.points();
        let pts_size = pts.len();
        if pts_size < 3 {
            return false;
        }

        // Emboss.cpp:155-157
        let a = if index == 0 { pts[pts_size - 1] } else { pts[index - 1] };
        let b = pts[index];
        let c = if index == (pts_size - 1) { pts[0] } else { pts[index + 1] };

        // Emboss.cpp:159-161  calc sides
        let (ba, bc) = (a - b, c - b);
        let ba: Vec2d = Vec2d::new(ba.x as f64, ba.y as f64);
        let bc: Vec2d = Vec2d::new(bc.x as f64, bc.y as f64);

        // Emboss.cpp:163
        let dot_product = ba.dot(&bc);

        // Emboss.cpp:165-169  sqrt together after multiplication save one sqrt
        let ba_size_sq = ba.dot(&ba);
        let bc_size_sq = bc.dot(&bc);
        let norm = (ba_size_sq * bc_size_sq).sqrt();
        let mut cos_angle = dot_product / norm;

        // Emboss.cpp:171-173  small angle are around 1 --> cos(0) = 1
        if cos_angle < spike_desc.cos_angle {
            return false; // not a spike
        }

        // Emboss.cpp:175-180
        // has to be in range <-1, 1>
        // Due to preccission of floating point number could be sligtly out of range
        if cos_angle > 1.0 {
            cos_angle = 1.0;
        }
        // if (cos_angle < -1.) cos_angle = -1.;

        // Emboss.cpp:182-185  Current Spike angle
        let angle = cos_angle.acos();
        let wanted_size = spike_desc.half_bevel / (angle / 2.0).cos();
        let wanted_size_sq = wanted_size * wanted_size;

        // Emboss.cpp:187-188
        let is_ba_short = ba_size_sq < wanted_size_sq;
        let is_bc_short = bc_size_sq < wanted_size_sq;

        // Emboss.cpp:190-193
        // (wanted_size * ba_norm).cast<coord_t>() truncates toward zero (Eigen cast<int>)
        // `b` is a reference to `pts[index]` in the C++: the lambdas read it when they run, so
        // they take it as an argument here.
        let a_side = |b: Point| -> Point {
            let s = ba_size_sq.sqrt();
            let ba_norm = (ba.x / s, ba.y / s);
            b + Point::new((wanted_size * ba_norm.0) as i64, (wanted_size * ba_norm.1) as i64)
        };
        // Emboss.cpp:194-197
        let c_side = |b: Point| -> Point {
            let s = bc_size_sq.sqrt();
            let bc_norm = (bc.x / s, bc.y / s);
            b + Point::new((wanted_size * bc_norm.0) as i64, (wanted_size * bc_norm.1) as i64)
        };

        // Emboss.cpp:199-218
        if is_ba_short && is_bc_short {
            // Emboss.cpp:199-201  remove short spike
            do_erase = true;
        } else if is_ba_short {
            // Emboss.cpp:202-204  move point B on C-side
            polygon.points_mut()[index] = c_side(b);
        } else if is_bc_short {
            // Emboss.cpp:205-207  move point B on A-side
            polygon.points_mut()[index] = a_side(b);
        } else {
            // Emboss.cpp:208-217
            // move point B on C-side and add point on A-side(left - before)
            let moved = c_side(b);
            polygon.points_mut()[index] = moved;
            // `a_side()` runs after the assignment: its `b` is the moved point
            add = Some(a_side(moved));
            if add == Some(polygon.points()[index]) {
                // should be very rare, when SpikeDesc has small base
                // will be fixed by remove B point
                add = None;
                do_erase = true;
            }
        }
    }
    // Emboss.cpp:220-223
    if do_erase {
        polygon.points_mut().remove(index);
        return true;
    }
    // Emboss.cpp:224-226
    if let Some(p) = add {
        polygon.points_mut().insert(index, p);
    }
    false
}

// Emboss.cpp:229-260
pub fn remove_spikes_in_duplicates(expolygons: &mut ExPolygons, duplicates: &Points) {
    // Emboss.cpp:230-231
    if duplicates.is_empty() {
        return;
    }
    // Emboss.cpp:232-244
    let check = |polygon: &mut Polygon, d: &Point| -> bool {
        // Emboss.cpp:233-235
        let spike_bevel = 1.0 / SHAPE_SCALE;
        let spike_length = 5.0;
        let sd = SpikeDesc::new(spike_bevel, spike_length);
        // Emboss.cpp:236-243
        let mut exist_remove = false;
        let mut i = 0;
        while i < polygon.points().len() {
            if polygon.points()[i] != *d {
                i += 1;
                continue;
            }
            exist_remove |= remove_when_spike(polygon, i, &sd);
            i += 1;
        }
        exist_remove && polygon.points().len() < 3
    };

    // Emboss.cpp:246-256
    let mut exist_remove = false;
    for expolygon in expolygons.iter_mut() {
        // Emboss.cpp:248
        let bb = BoundingBox::from_points(expolygon.contour.points());
        // Emboss.cpp:249-255
        for d in duplicates {
            if !bb.contains_point(d) {
                continue;
            }
            exist_remove |= check(&mut expolygon.contour, d);
            for hole in expolygon.holes.iter_mut() {
                exist_remove |= check(hole, d);
            }
        }
    }

    // Emboss.cpp:258-259
    if exist_remove {
        for_expolygons::remove_bad(expolygons);
    }
}

// Emboss.cpp:262-267
fn is_valid(font: &FontFile, index: u32) -> bool {
    // Emboss.cpp:263  if (font.data == nullptr) return false;
    let data = match &font.data {
        None => return false,
        Some(d) => d,
    };
    // Emboss.cpp:264  if (font.data->empty()) return false;
    if data.is_empty() {
        return false;
    }
    // Emboss.cpp:265  if (index >= font.infos.size()) return false;
    if index as usize >= font.infos.len() {
        return false;
    }
    // Emboss.cpp:266
    true
}

// Emboss.cpp:269-285
fn load_font_info(data: &[u8], index: u32) -> FontinfoOpt<'_> {
    // Emboss.cpp:272
    let font_offset = imstb_truetype::stbtt_get_font_offset_for_index(data, index as i32);
    if font_offset < 0 {
        // assert(false);
        // "Font index(" << index << ") doesn't exist.";
        return None;
    }
    // Emboss.cpp:278-283
    let font_info = imstb_truetype::stbtt_init_font(data, font_offset);
    if font_info.is_none() {
        // Can't initialize font.
        // assert(false);
        return None;
    }
    // Emboss.cpp:284
    font_info
}

// Emboss.cpp:287-292
fn remove_bad(polygons: &mut Polygons) {
    polygons.retain(|p| p.points().len() >= 3);
}

mod for_expolygons {
    use super::*;

    // Emboss.cpp:294-302
    pub(super) fn remove_bad(expolygons: &mut ExPolygons) {
        // Emboss.cpp:295-298
        expolygons.retain(|p| p.contour.points().len() >= 3);
        // Emboss.cpp:300-301
        for expolygon in expolygons.iter_mut() {
            super::remove_bad(&mut expolygon.holes);
        }
    }
}

// ===========================================================================
// Emboss:: heal functions (Emboss.cpp:305-446)
// ===========================================================================

// Emboss.cpp:305-406
pub fn divide_segments_for_close_point(expolygons: &mut ExPolygons, distance: f64) -> bool {
    // Emboss.cpp:307-308
    if expolygons.is_empty() {
        return false;
    }
    if distance < 0.0 {
        return false;
    }

    // Emboss.cpp:310-311  ExPolygons can't contain same neigbours
    remove_same_neighbor(expolygons);

    // Emboss.cpp:313-316
    // IMPROVE: use int(insted of double) lines and tree
    let ids = ExPolygonsIndices::new(expolygons);
    let lines: Lines = to_linesf(expolygons, ids.get_count());
    let tree = build_aabb_tree_over_indexed_linesf(&lines);

    // Emboss.cpp:317  using Div = std::pair<Point, size_t>;
    type Div = (Point, usize);
    // Emboss.cpp:318
    let mut divs: Vec<Div> = Vec::new();
    // Emboss.cpp:319
    let mut point_index: usize = 0;

    // Emboss.cpp:320-346
    let check_points = |pts: &Points, divs: &mut Vec<Div>, point_index: &mut usize| {
        // Emboss.cpp:321
        for (pi, p) in pts.iter().enumerate() {
            let p = *p;
            // Emboss.cpp:322
            let p_d = Vec2d::new(p.x as f64, p.y as f64);
            // Emboss.cpp:323
            let close_lines = aabb_tree_lines::all_lines_in_radius(&lines, &tree, p_d, distance);
            // Emboss.cpp:324
            for index in close_lines {
                // Emboss.cpp:325-329  skip point neighbour lines indices
                if index == *point_index {
                    continue;
                }
                if pi != 0 {
                    if index == point_index.wrapping_sub(1) {
                        continue;
                    }
                } else if index == (pts.len() - 1) {
                    continue;
                }

                // Emboss.cpp:331-340  do not doubled side point of segment
                let id = ids.cvt_index(index as u32);
                let expoly = &expolygons[id.expolygons_index as usize];
                let poly = if id.is_contour() { &expoly.contour } else { &expoly.holes[id.hole_index() as usize] };
                let poly_pts = poly.points();
                let line_a = poly_pts[id.point_index as usize];
                let line_b = if !ids.is_last_point(&id) { poly_pts[id.point_index as usize + 1] } else { poly_pts[0] };
                debug_assert_eq!(line_a, lines[index].a);
                debug_assert_eq!(line_b, lines[index].b);
                if p == line_a || p == line_b {
                    continue;
                }

                // Emboss.cpp:342
                divs.push((p, index));
            }
            // Emboss.cpp:344
            *point_index += 1;
        }
    };

    // Emboss.cpp:347-351
    // NOTE: iterate over an immutable snapshot of the contours while accumulating divs;
    // the polygons are only mutated in the second pass below.
    let snapshot: Vec<(Points, Vec<Points>)> = expolygons
        .iter()
        .map(|e| (e.contour.points().to_vec(), e.holes.iter().map(|h| h.points().to_vec()).collect()))
        .collect();
    for (contour_pts, holes_pts) in &snapshot {
        check_points(contour_pts, &mut divs, &mut point_index);
        for hole_pts in holes_pts {
            check_points(hole_pts, &mut divs, &mut point_index);
        }
    }

    // Emboss.cpp:353-354  check if exist division
    if divs.is_empty() {
        return false;
    }

    // Emboss.cpp:356-359
    // sort from biggest index to zero
    // to be able add points and not interupt indices
    // (libc++ `std::sort`, which is not stable)
    crate::clipper::libcxx_sort(&mut divs, |d1, d2| d1.1 > d2.1);

    // Emboss.cpp:361-404
    let mut it: usize = 0;
    while it != divs.len() {
        // Emboss.cpp:364-367  colect division of a line segmen
        let index = divs[it].1;
        let mut it2 = it + 1;
        while it2 != divs.len() && divs[it2].1 == index {
            it2 += 1;
        }

        // Emboss.cpp:369-373
        let id = ids.cvt_index(index as u32);
        let expoly = &mut expolygons[id.expolygons_index as usize];
        let poly = if id.is_contour() { &mut expoly.contour } else { &mut expoly.holes[id.hole_index() as usize] };
        let count = it2 - it;

        // Emboss.cpp:375-402  add points into polygon to divide in place of near point
        if count == 1 {
            poly.points_mut().insert(id.point_index as usize + 1, divs[it].0);
            it += 1;
        } else {
            // collect points to add into polygon
            let mut points: Points = Vec::with_capacity(count);
            for d in &divs[it..it2] {
                points.push(d.0);
            }
            it = it2;

            // Emboss.cpp:386-394  need sort by line direction
            let line = &lines[index];
            let dir = line.b - line.a;
            // select mayorit direction
            let axis = if dir.x.abs() > dir.y.abs() { 0 } else { 1 };
            let reverse = if axis == 0 { dir.x < 0 } else { dir.y < 0 };
            crate::clipper::libcxx_sort(&mut points, |p1, p2| {
                let (a, b) = if axis == 0 { (p1.x, p2.x) } else { (p1.y, p2.y) };
                if reverse {
                    a > b
                } else {
                    a < b
                }
            });

            // Emboss.cpp:396-397  use only unique points
            points.dedup();

            // Emboss.cpp:399-401  divide line by adding points into polygon
            let pos = id.point_index as usize + 1;
            for (offset, pt) in points.into_iter().enumerate() {
                poly.points_mut().insert(pos + offset, pt);
            }
        }
        // Emboss.cpp:403  assert(it == it2);
        debug_assert_eq!(it, it2);
    }
    // Emboss.cpp:405
    true
}

// Emboss.cpp:408-440
pub fn heal_polygons(shape: &Polygons, is_non_zero: bool, max_iteration: u32) -> HealedExPolygons {
    // Emboss.cpp:410-412
    let clean_distance = 1.415; // little grater than sqrt(2)
    let fill_type = if is_non_zero {
        crate::clipper::PolyFillType::pftNonZero
    } else {
        crate::clipper::PolyFillType::pftEvenOdd
    };

    // Emboss.cpp:414-421
    // When edit this code check that font 'ALIENATE.TTF' and glyph 'i' still work
    // fix of self intersections
    // http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/SimplifyPolygon.htm
    let mut paths = crate::clipper::simplify_polygons(&polygons_provider(shape), fill_type);
    // ClipperLib::CleanPolygons(paths, clean_distance);  (in place: CleanPolygons(polys, polys, distance))
    let in_paths = paths.clone();
    crate::clipper::clean_polygons(&in_paths, &mut paths, clean_distance);
    let mut polygons = to_polygons_paths(&paths);
    polygons.retain(|p| p.points().len() >= 3);

    // Emboss.cpp:423-424
    if polygons.is_empty() {
        return HealedExPolygons { expolygons: ExPolygons::new(), is_healed: false };
    }

    // Emboss.cpp:426-436
    // Do not remove all duplicates but do it better way
    // Overlap all duplicit points by rectangle 3x3
    let duplicits = collect_duplicates(to_points_polygons(&polygons));
    if !duplicits.is_empty() {
        polygons.reserve(duplicits.len());
        for p in &duplicits {
            let mut rect_3x3 = Polygon::from_points(pts_3x3());
            rect_3x3.translate(*p);
            polygons.push(rect_3x3);
        }
    }
    // Emboss.cpp:437-439
    let mut res = union_ex_polygons(&polygons, fill_type);
    let is_healed = heal_expolygons(&mut res, max_iteration);
    HealedExPolygons { expolygons: res, is_healed }
}

// Emboss.cpp:443-446
pub fn heal_expolygons(shape: &mut ExPolygons, max_iteration: u32) -> bool {
    heal_dupl_inter(shape, max_iteration)
}

// ===========================================================================
// Anonymous namespace heal helpers (Emboss.cpp:448-684)
// ===========================================================================

// Emboss.cpp:450-468
fn get_unique_intersections(intersections: &IntersectionsLines) -> Points {
    let mut result: Points = Points::new();
    if intersections.is_empty() {
        return result;
    }

    // Emboss.cpp:456-462  convert intersections into Points
    // Point(std::floor(x), std::floor(y)): the double constructor rounds (lrint) the floored value.
    result.reserve(intersections.len());
    for i in intersections {
        result.push(Point::new(
            i.intersection.x.floor().round_ties_even() as i64,
            i.intersection.y.floor().round_ties_even() as i64,
        ));
    }
    // Emboss.cpp:463-467  intersections should be unique poits
    result.sort_by(|a, b| (a.x, a.y).cmp(&(b.x, b.y)));
    result.dedup();
    result
}

// Emboss.cpp:470-480
fn get_holes_with_points(holes: &Polygons, points: &Points) -> Polygons {
    let mut result: Polygons = Polygons::new();
    for hole in holes {
        for p in points {
            for h in hole.points() {
                if *p == *h {
                    result.push(hole.clone());
                    break;
                }
            }
        }
    }
    result
}

// Emboss.cpp:482-505
// Fill holes which create duplicits or intersections
fn fill_trouble_holes(holes: &Polygons, duplicates: &Points, intersections: &Points, shape: &mut ExPolygons) -> bool {
    // Emboss.cpp:493-496
    if holes.is_empty() {
        return false;
    }
    if duplicates.is_empty() && intersections.is_empty() {
        return false;
    }

    // Emboss.cpp:498-501
    let mut fill = get_holes_with_points(holes, duplicates);
    fill.extend(get_holes_with_points(holes, intersections));
    if fill.is_empty() {
        return false;
    }

    // Emboss.cpp:503-504
    *shape = union_ex_expolygons_polygons(shape, &fill);
    true
}

// Emboss.cpp:507-513
// extend functionality from Points.cpp --> collect_duplicates with address of duplicated points
struct Duplicate {
    point: Point,
    indices: Vec<u32>,
}
type Duplicates = Vec<Duplicate>;

// Emboss.cpp:514-543
fn collect_duplicit_indices(expoly: &ExPolygons) -> Duplicates {
    // Emboss.cpp:516
    let pts = to_points(expoly);

    // Emboss.cpp:518-522  initialize original index locations
    let mut idx: Vec<u32> = (0..pts.len() as u32).collect();
    idx.sort_by(|&i1, &i2| {
        let a = pts[i1 as usize];
        let b = pts[i2 as usize];
        (a.x, a.y).cmp(&(b.x, b.y))
    });

    // Emboss.cpp:524-541
    let mut result: Duplicates = Vec::new();
    if idx.is_empty() {
        return result;
    }
    let mut prev = pts[idx[0] as usize];
    for i in 1..idx.len() {
        let index = idx[i];
        let act = pts[index as usize];
        if prev == act {
            // duplicit point
            if !result.is_empty() && result.last().unwrap().point == act {
                // more than 2 points with same coordinate
                result.last_mut().unwrap().indices.push(index);
            } else {
                let prev_index = idx[i - 1];
                result.push(Duplicate { point: act, indices: vec![prev_index, index] });
            }
            continue;
        }
        prev = act;
    }
    result
}

// Emboss.cpp:545-556
fn get_points(duplicate_indices: &Duplicates) -> Points {
    let mut result: Points = Points::new();
    if duplicate_indices.is_empty() {
        return result;
    }
    result.reserve(duplicate_indices.len());
    for d in duplicate_indices {
        result.push(d.point);
    }
    result
}

// Emboss.cpp:558-639
pub fn heal_dupl_inter(shape: &mut ExPolygons, mut max_iteration: u32) -> bool {
    // Emboss.cpp:560-561
    if shape.is_empty() {
        return true;
    }
    remove_same_neighbor(shape);

    // Emboss.cpp:563-564  create loop permanent memory
    let mut holes: Polygons = Polygons::new();
    // Emboss.cpp:565  while (--max_iteration)
    loop {
        max_iteration = max_iteration.wrapping_sub(1);
        if max_iteration == 0 {
            break;
        }

        // Emboss.cpp:566-568
        let duplicate_indices = collect_duplicit_indices(shape);
        let intersections = get_intersections_expolygons(shape);

        // Emboss.cpp:570-572  Check whether shape is already healed
        if intersections.is_empty() && duplicate_indices.is_empty() {
            return true;
        }

        // Emboss.cpp:574-575
        let duplicate_points = get_points(&duplicate_indices);
        let intersection_points = get_unique_intersections(&intersections);

        // Emboss.cpp:577-580
        if fill_trouble_holes(&holes, &duplicate_points, &intersection_points, shape) {
            holes.clear();
            continue;
        }

        // Emboss.cpp:582-583
        holes.clear();
        holes.reserve(intersections.len() + duplicate_points.len());

        // Emboss.cpp:585
        remove_spikes_in_duplicates(shape, &duplicate_points);

        // Emboss.cpp:587-592  Fix self intersection in result by subtracting hole 2x2
        for p in &intersection_points {
            let mut hole = Polygon::from_points(pts_2x2());
            hole.translate(*p);
            holes.push(hole);
        }

        // Emboss.cpp:594-599  Fix duplicit points by hole 3x3 around duplicit point
        for p in &duplicate_points {
            let mut hole = Polygon::from_points(pts_3x3());
            hole.translate(*p);
            holes.push(hole);
        }

        // Emboss.cpp:601-602
        // ApplySafetyOffset::Yes is incompatible with function fill_trouble_holes
        *shape = diff_ex(shape, &holes, crate::clipper_utils::ApplySafetyOffset::No);
    }

    // Emboss.cpp:605-611  Create partialy healed output
    let duplicates = collect_duplicit_indices(shape);
    let intersections = get_intersections_expolygons(shape);
    if duplicates.is_empty() && intersections.is_empty() {
        // healed in the last loop
        return true;
    }

    // Emboss.cpp:617  assert(false); // Can not heal this shape
    // investigate how to heal better way

    // Emboss.cpp:620-629
    let ei = ExPolygonsIndices::new(shape);
    let mut is_healed: Vec<bool> = vec![true; shape.len()];
    for duplicate in &duplicates {
        for &i in &duplicate.indices {
            is_healed[ei.cvt_index(i).expolygons_index as usize] = false;
        }
    }
    for intersection in &intersections {
        is_healed[ei.cvt_index(intersection.line_index1).expolygons_index as usize] = false;
        is_healed[ei.cvt_index(intersection.line_index2).expolygons_index as usize] = false;
    }

    // Emboss.cpp:631-637
    for shape_index in 0..shape.len() {
        if !is_healed[shape_index] {
            // exchange non healed expoly with bb rect
            let expoly = shape[shape_index].clone();
            shape[shape_index] = create_bounding_rect(&[expoly]);
        }
    }
    false
}

// Emboss.cpp:641-664
pub fn create_bounding_rect(shape: &[ExPolygon]) -> ExPolygon {
    // Emboss.cpp:642-647
    let mut bb = get_extents_expolygons(shape);
    let size = bb.size();
    if size.x < 10 {
        bb.max.x += 10;
    }
    if size.y < 10 {
        bb.max.y += 10;
    }

    // Emboss.cpp:649-653  CCW
    let rect = Polygon::from_points(vec![bb.min, Point::new(bb.max.x, bb.min.y), bb.max, Point::new(bb.min.x, bb.max.y)]);

    // Emboss.cpp:655  Point offset = bb.size() * 0.1;
    // `bb.size()` is an rvalue Point: the member `Point::operator*(const double&)` (Point.hpp:200)
    // applies, which builds the Point from doubles (lrint).
    let bb_size = bb.size();
    let offset = Point::new(
        ((bb_size.x as f64) * 0.1).round_ties_even() as i64,
        ((bb_size.y as f64) * 0.1).round_ties_even() as i64,
    );
    // Emboss.cpp:656-661  CW
    let hole = Polygon::from_points(vec![
        bb.min + offset,
        Point::new(bb.min.x + offset.x, bb.max.y - offset.y),
        bb.max - offset,
        Point::new(bb.max.x - offset.x, bb.min.y + offset.y),
    ]);

    // Emboss.cpp:663  return ExPolygon(rect, hole);
    let mut ex = ExPolygon::new(rect);
    ex.holes.push(hole);
    ex
}

// Emboss.cpp:666-684  (#ifdef REMOVE_SMALL_ISLANDS)
#[allow(dead_code)]
pub fn remove_small_islands(expolygons: &mut ExPolygons, minimal_area: f64) {
    // Emboss.cpp:668-669
    if expolygons.is_empty() {
        return;
    }

    // Emboss.cpp:671-674  remove small expolygons contours
    expolygons.retain(|p| !(p.contour.area() < minimal_area));

    // Emboss.cpp:676-682  remove small holes in expolygons
    for expoly in expolygons.iter_mut() {
        let holes = &mut expoly.holes;
        holes.retain(|p| !(-p.area() < minimal_area));
    }
}
fn get_glyph(font_info: &StbttFontinfo, unicode_letter: i32, flatness: f32) -> Option<Glyph> {
    let glyph_index = imstb_truetype::stbtt_find_glyph_index(font_info, unicode_letter);
    if glyph_index == 0 {
        return None;
    }

    let mut glyph = Glyph::default();
    let (advance_width, left_side_bearing) = imstb_truetype::stbtt_get_glyph_h_metrics(font_info, glyph_index);
    glyph.advance_width = advance_width;
    glyph.left_side_bearing = left_side_bearing;

    let vertices = imstb_truetype::stbtt_get_glyph_shape(font_info, glyph_index);
    let num_verts = vertices.len();
    if num_verts == 0 {
        return Some(glyph); // no shape
    }

    let Some((points, contour_lengths)) = imstb_truetype::stbtt_flatten_curves(&vertices, flatness) else {
        return Some(glyph); // no valid flattening
    };

    let num_contour = contour_lengths.len();
    let mut glyph_polygons: Polygons = Vec::with_capacity(num_contour);
    let mut pi: usize = 0; // point index
    for ci in 0..num_contour {
        let mut length = contour_lengths[ci];
        if length < 4 {
            pi = pi.wrapping_add(length as usize);
            continue;
        }
        length -= 1;
        let mut pts: Points = Vec::with_capacity(length as usize);
        for _ in 0..length {
            pts.push(to_point(&points[pi]));
            pi += 1;
        }

        debug_assert!(pts[0] == to_point(&points[pi]));
        pi += 1;

        pts.reverse();
        glyph_polygons.push(Polygon::from_points(pts));
    }
    if !glyph_polygons.is_empty() {
        let max_iteration = 10;
        let is_non_zero = true;
        glyph.shape = heal_polygons(&glyph_polygons, is_non_zero, max_iteration).expolygons;
    }
    Some(glyph)
}

mod cached {
    use super::*;

    pub(super) fn get_glyph<'c, 'a>(
        unicode: i32,
        font_file: &'a FontFile,
        font_prop: &FontProp,
        cache: &'c mut Glyphs,
        font_info_opt: &mut FontinfoOpt<'a>,
    ) -> Option<&'c Glyph> {
        const RESOLUTION: f32 = 0.0125; // [in mm]
        if cache.contains_key(&unicode) {
            return cache.get(&unicode);
        }

        let font_index = font_prop.collection_number.unwrap_or(0);
        if !is_valid(font_file, font_index) {
            return None;
        }

        if font_info_opt.is_none() {
            *font_info_opt = load_font_info(font_file.data.as_deref().unwrap_or(&[]), font_index);
            if font_info_opt.is_none() {
                return None;
            }
        }

        let mut flatness = font_file.infos[font_index as usize].ascent as f32 * RESOLUTION / font_prop.size_in_mm;

        if flatness < RESOLUTION {
            flatness = RESOLUTION;
        }

        let glyph_opt = super::get_glyph(font_info_opt.as_ref().unwrap(), unicode, flatness);

        let mut glyph = glyph_opt?;

        if let Some(char_gap) = font_prop.char_gap {
            glyph.advance_width += char_gap;
        }

        glyph.advance_width = (glyph.advance_width as f64 / SHAPE_SCALE) as i32;
        glyph.left_side_bearing = (glyph.left_side_bearing as f64 / SHAPE_SCALE) as i32;
        if !glyph.shape.is_empty() {
            if let Some(boldness) = font_prop.boldness {
                let delta = (boldness as f64 / SHAPE_SCALE / font_prop.size_in_mm as f64) as f32;
                glyph.shape = union_ex(&offset_ex(&glyph.shape, delta));
            }
            if let Some(skew) = font_prop.skew {
                let ratio = skew as f64;
                let skew = |polygon: &mut Polygon| {
                    for p in polygon.points_mut().iter_mut() {
                        p.x += (p.y as f64 * ratio).round() as i64;
                    }
                };
                for expolygon in glyph.shape.iter_mut() {
                    skew(&mut expolygon.contour);
                    for hole in expolygon.holes.iter_mut() {
                        skew(hole);
                    }
                }
            }
        }
        let previous = cache.insert(unicode, glyph);
        debug_assert!(previous.is_none());
        cache.get(&unicode)
    }
}

fn to_point(point: &StbttPoint) -> Point {
    Point::new(
        ((point.x as f64) / SHAPE_SCALE).round() as i32 as i64,
        ((point.y as f64) / SHAPE_SCALE).round() as i32 as i64,
    )
}

#[cfg(windows)]
#[allow(dead_code)]
fn create_style(name: &[char], path: &[char]) -> EmbossStyle {
    EmbossStyle {
        name: name.iter().collect(),
        path: path.iter().collect(),
        r#type: crate::text_configuration::EmbossStyleType::FilePath,
        prop: FontProp::new(10.0),
    }
}

#[cfg(not(windows))]
pub fn get_font_list() -> EmbossStyles {
    EmbossStyles::new()
}

#[cfg(not(windows))]
pub fn get_font_path(_font_face_name: &[char]) -> Option<WString> {
    None
}

pub fn create_font_file(data: Vec<u8>) -> Option<Box<FontFile>> {
    let collection_size = imstb_truetype::stbtt_get_number_of_fonts(&data);
    if collection_size < 1 {
        return None;
    }

    let c_size = collection_size as u32;
    let mut infos: Vec<FontFileInfo> = Vec::with_capacity(c_size as usize);
    for i in 0..c_size {
        let font_info = load_font_info(&data, i)?;

        let (ascent, descent, linegap) = imstb_truetype::stbtt_get_font_v_metrics(&font_info);

        let pixels: f32 = 1000.0; // value is irelevant
        let em_pixels = imstb_truetype::stbtt_scale_for_mapping_em_to_pixels(&font_info, pixels);
        let units_per_em = (pixels / em_pixels).round() as i32;

        infos.push(FontFileInfo { ascent, descent, linegap, unit_per_em: units_per_em });
    }
    Some(Box::new(FontFile::new(data, infos)))
}

pub mod from_path {
    use super::*;

    pub fn create_font_file(file_path: &str) -> Option<Box<FontFile>> {
        let buffer = match std::fs::read(file_path) {
            Ok(b) => b,
            Err(_) => {
                crate::boost_log_trivial!(Error, "Couldn't open ", file_path, " for reading.");
                return None;
            }
        };
        if buffer.is_empty() {
            crate::boost_log_trivial!(Error, "Size of font file is zero. Can't read.");
            return None;
        }
        super::create_font_file(buffer)
    }
}

pub fn letter2glyph(font: &FontFile, font_index: u32, letter: i32, flatness: f32) -> Option<Glyph> {
    if !is_valid(font, font_index) {
        return None;
    }
    let font_info_opt = load_font_info(font.data.as_deref().unwrap_or(&[]), font_index)?;
    get_glyph(&font_info_opt, letter, flatness)
}

pub fn get_font_info<'a>(font: &'a FontFile, prop: &FontProp) -> &'a FontFileInfo {
    let font_index = prop.collection_number.unwrap_or(0);
    debug_assert!(is_valid(font, font_index));
    &font.infos[font_index as usize]
}

pub fn get_line_height(font: &FontFile, prop: &FontProp) -> i32 {
    let info = get_font_info(font, prop);
    let mut line_height = info.ascent - info.descent + info.linegap;
    line_height += prop.line_gap.unwrap_or(0);
    ((line_height as f64) / SHAPE_SCALE) as i32
}

fn letter2shapes<'a>(
    letter: char,
    cursor: &mut Point,
    font_file: Option<&'a FontFile>,
    cache: Option<&mut Glyphs>,
    font_prop: &FontProp,
    font_info_cache: &mut FontinfoOpt<'a>,
) -> ExPolygons {
    let (Some(font), Some(cache)) = (font_file, cache) else {
        return ExPolygons::new();
    };

    if letter == '\n' {
        cursor.x = 0;
        cursor.y -= get_line_height(font, font_prop) as i64;
        return ExPolygons::new();
    }
    if letter == '\t' {
        let count_spaces = 4;
        let Some(space) = cached::get_glyph(' ' as i32, font, font_prop, cache, font_info_cache) else {
            return ExPolygons::new();
        };
        cursor.x += (count_spaces * space.advance_width) as i64;
        return ExPolygons::new();
    }
    if letter == '\r' {
        return ExPolygons::new();
    }

    let unicode = letter as i32;
    let Some(glyph_ptr) = cached::get_glyph(unicode, font, font_prop, cache, font_info_cache) else {
        return ExPolygons::new();
    };
    let mut expolygons = glyph_ptr.shape.clone(); // copy
    for expolygon in expolygons.iter_mut() {
        expolygon.translate(*cursor);
    }

    cursor.x += glyph_ptr.advance_width as i64;
    expolygons
}

mod with_back_fonts {
    use super::*;

    #[allow(clippy::too_many_arguments)]
    pub(super) fn letter2shapes<'a>(
        result: &mut ExPolygons,
        cur_scale: f32,
        real_use_font: &mut FontFileWithCache,
        real_scale: &mut f32,
        letter: char,
        cursor: &mut Point,
        font_file: Option<&'a FontFile>,
        cache: Option<&mut Glyphs>,
        font_prop: &FontProp,
        font_info_cache: &mut FontinfoOpt<'a>,
        bfc_fn: BackFontCacheFn,
    ) {
        let (Some(font), Some(cache)) = (font_file, cache) else {
            return;
        };

        if letter == '\n' {
            cursor.x = 0;
            cursor.y -= get_line_height(font, font_prop) as i64;
            return;
        }
        if letter == '\t' {
            let count_spaces = 4;
            let Some(space) = cached::get_glyph(' ' as i32, font, font_prop, cache, font_info_cache) else {
                return;
            };
            cursor.x += (count_spaces * space.advance_width) as i64;
            return;
        }
        if letter == '\r' {
            return;
        }

        let unicode = letter as i32;
        let mut glyph: Option<Glyph> = cached::get_glyph(unicode, font, font_prop, cache, font_info_cache).cloned();
        if glyph.as_ref().map_or(true, |g| g.shape.is_empty()) {
            let mut can_gen_from_back_font = false;
            if let Some(bfc_fn) = bfc_fn {
                let fn_result = bfc_fn();
                for mut temp_font in fn_result {
                    if !temp_font.has_value() {
                        continue;
                    }
                    let cur_font: Arc<FontFile> = temp_font.font_file.clone().unwrap();
                    let cur_glyph = {
                        let cur_cache = temp_font.cache.as_mut().unwrap();
                        cur_cache.clear();
                        let mut temp_font_prop = font_prop.clone();
                        let font_info = get_font_info(&cur_font, &temp_font_prop);
                        temp_font_prop.boldness = Some(if font_prop.boldness.map_or(false, |b| b > 0.0) {
                            font_info.ascent as f32 / 4.0
                        } else {
                            0.0
                        }); // from GLGizmoText::set_default_boldness(std::optional<float> &boldness)

                        let new_scale = get_text_shape_scale(font_prop, &cur_font);
                        let mut cur_font_info_cache: FontinfoOpt = None;
                        cached::get_glyph(unicode, &cur_font, &temp_font_prop, cur_cache, &mut cur_font_info_cache)
                            .cloned()
                            .map(|g| (g, new_scale))
                    };
                    if let Some((g, new_scale)) = cur_glyph {
                        glyph = Some(g);
                        *real_use_font = temp_font;
                        *real_scale = new_scale as f32;
                        can_gen_from_back_font = true;
                        break;
                    }
                }
            }
            if !can_gen_from_back_font {
                return;
            }
        }
        let glyph_ptr = glyph.unwrap();

        let mut expolygons = glyph_ptr.shape.clone(); // copy
        if *real_scale > 0.0 {
            let ratio = *real_scale / cur_scale;
            for expolygon in expolygons.iter_mut() {
                expolygon_scale(expolygon, ratio as f64);
                expolygon.translate(*cursor);
            }
            let new_advance_width = glyph_ptr.advance_width as f32 * ratio;
            cursor.x = (cursor.x as f32 + new_advance_width) as i64;
        } else {
            for expolygon in expolygons.iter_mut() {
                expolygon.translate(*cursor);
            }
            cursor.x += glyph_ptr.advance_width as i64;
        }
        *result = expolygons;
    }
}

const CANCEL_CHECK: u32 = 10;

mod with_ids {
    use super::*;

    pub(super) fn union_with_delta(shapes: &ExPolygonsWithIds, delta: f32, max_heal_iteration: u32) -> HealedExPolygons {
        let mut expolygons: ExPolygons = ExPolygons::new();
        for shape in shapes {
            if shape.expoly.is_empty() {
                continue;
            }
            expolygons.extend(offset_ex(&shape.expoly, delta));
        }
        let mut result = union_ex(&expolygons);
        result = offset_ex(&result, -delta);
        let is_healed = heal_expolygons(&mut result, max_heal_iteration);
        HealedExPolygons { expolygons: result, is_healed }
    }
}

pub mod for_emboss_shape {
    use super::*;

    pub fn union_with_delta(shape: &mut EmbossShape, delta: f32, max_heal_iteration: u32) -> ExPolygons {
        if !shape.final_shape.expolygons.is_empty() {
            return shape.final_shape.expolygons.clone();
        }

        shape.final_shape = with_ids::union_with_delta(&shape.shapes_with_ids, delta, max_heal_iteration);
        for e in &shape.shapes_with_ids {
            if !e.is_healed {
                shape.final_shape.is_healed = false;
            }
        }
        shape.final_shape.expolygons.clone()
    }
}

pub fn union_with_delta(expoly: ExPolygons, delta: f32, max_heal_iteration: u32) -> HealedExPolygons {
    let mut expolygons: ExPolygons = ExPolygons::new();
    expolygons.extend(offset_ex(&expoly, delta));
    let mut result = union_ex(&expolygons);
    result = offset_ex(&result, -delta);
    let is_healed = heal_expolygons(&mut result, max_heal_iteration);
    HealedExPolygons { expolygons: result, is_healed }
}

pub fn translate(expolygons_with_ids: &mut ExPolygonsWithIds, p: Point) {
    for expolygons_with_id in expolygons_with_ids.iter_mut() {
        for ex in expolygons_with_id.expoly.iter_mut() {
            ex.translate(p);
        }
    }
}

pub fn get_extents(expolygons_with_ids: &ExPolygonsWithIds) -> BoundingBox {
    let mut bb = BoundingBox::new();
    for expolygons_with_id in expolygons_with_ids {
        bb.merge(&get_extents_expolygons(&expolygons_with_id.expoly));
    }
    bb
}

pub fn center(e: &mut ExPolygonsWithIds) {
    let bb = get_extents(e);
    let c = bb.center();
    translate(e, Point::new(-c.x, -c.y));
}

pub fn text2shapes(
    emboss_shape: &mut EmbossShape,
    font_with_cache: &mut FontFileWithCache,
    text: &str,
    font_prop: &FontProp,
    standard_scale: f64,
    was_canceled: &dyn Fn() -> bool,
    bfc_fn: BackFontCacheFn,
) -> (HealedExPolygons, TextLineLayout) {
    let text_w: WString = text.chars().collect();
    let layout = text2vshapes(emboss_shape, font_with_cache, &text_w, font_prop, standard_scale, was_canceled, bfc_fn); // ExPolygonsWithIds vshapes =
    let vshapes = &emboss_shape.shapes_with_ids;
    let delta = (1.0 / SHAPE_SCALE) as f32;
    (with_ids::union_with_delta(vshapes, delta, MAX_HEAL_ITERATION_OF_TEXT), layout)
}

pub fn text2vshapes(
    emboss_shape: &mut EmbossShape,
    font_with_cache: &mut FontFileWithCache,
    text: &[char],
    font_prop: &FontProp,
    standard_scale: f64,
    was_canceled: &dyn Fn() -> bool,
    bfc_fn: BackFontCacheFn,
) -> TextLineLayout {
    debug_assert!(font_with_cache.has_value());
    let FontFileWithCache { font_file, cache } = font_with_cache;
    let Some(font) = font_file.as_deref() else {
        return TextLineLayout::default();
    };
    let font_index = font_prop.collection_number.unwrap_or(0);
    if !is_valid(font, font_index) {
        return TextLineLayout::default();
    }

    let mut counter: u32 = 0;
    let mut cursor = Point::new(0, 0);
    let mut text_cursors: Vec<f32> = Vec::new();
    let mut text_absolute_cursors: Vec<f32> = Vec::new();
    let mut last_x: f32 = 0.0;
    let mut cur_x: f32;
    let mut font_info_cache: FontinfoOpt = None;
    let mut result: ExPolygonsWithIds = Vec::with_capacity(text.len());
    let mut text_scales: Vec<f32> = Vec::with_capacity(text.len());
    let mut text_map_font: Vec<FontFileWithCache> = Vec::with_capacity(text.len());

    for &letter0 in text {
        let mut letter = letter0;
        counter += 1;
        if counter == CANCEL_CHECK {
            counter = 0;
            if was_canceled() {
                return TextLineLayout::default();
            }
        }
        if letter == ' ' {
            letter = 'i';
        }
        let id = letter as u32;
        let mut real_scale: f32 = -1.0;
        if bfc_fn.is_some() {
            let mut exps = ExPolygons::new();
            let mut real_use_font = FontFileWithCache::default();
            with_back_fonts::letter2shapes(
                &mut exps,
                standard_scale as f32,
                &mut real_use_font,
                &mut real_scale,
                letter,
                &mut cursor,
                Some(font),
                cache.as_mut(),
                font_prop,
                &mut font_info_cache,
                bfc_fn,
            );
            result.push(ExPolygonsWithId::new(id, exps));
            text_scales.push(real_scale);
            text_map_font.push(real_use_font);
        } else {
            result.push(ExPolygonsWithId::new(
                id,
                letter2shapes(letter, &mut cursor, Some(font), cache.as_mut(), font_prop, &mut font_info_cache),
            ));
            text_scales.push(real_scale);
        }
        cur_x = (cursor.x as f64 * standard_scale) as f32;
        text_cursors.push(if letter == '\n' { 0.0 } else { cur_x - last_x });
        text_absolute_cursors.push(cur_x);
        last_x = cur_x;
    }
    let mut offset_xy: Vec<Point> = Vec::new();
    if bfc_fn.is_some() {
        align_with_back_fonts::align_shape(
            &mut result,
            &mut offset_xy,
            &mut text_scales,
            standard_scale as f32,
            text_map_font,
            text,
            font_prop,
            font,
        );
    } else {
        align_shape(&mut result, &mut offset_xy, text, font_prop, font);
    }
    let mut text_align_offsets: Vec<crate::emboss_shape::Vec2f> = vec![crate::emboss_shape::Vec2f { x: 0.0, y: 0.0 }; text_scales.len()];
    for i in 0..text_scales.len() {
        let x = (offset_xy[i].x as f32 as f64 * standard_scale) as f32;
        let y = (offset_xy[i].y as f32 as f64 * standard_scale) as f32;
        text_align_offsets[i].x = x;
        text_align_offsets[i].y = y;
    }
    emboss_shape.text_align_offsets = text_align_offsets;
    let mut no_use_text_scales: Vec<f32> = Vec::new();
    for _letter in text {
        no_use_text_scales.push(-1.0);
    }
    emboss_shape.text_scales = no_use_text_scales;
    emboss_shape.text_cursors = text_cursors;
    emboss_shape.text_absolute_cursors = text_absolute_cursors;
    for i in 0..result.len() {
        if text[i] == ' ' {
            result[i].expoly.clear();
        }
    }
    emboss_shape.shapes_with_ids = result;
    emboss_shape.align_type = (font_prop.align.0 as i32, font_prop.align.1 as i32);
    let count_lines = get_count_lines(text);
    let line_height = (get_line_height(font, font_prop) as f64 * standard_scale) as f32;
    let first_line_offset_y = if count_lines > 1 {
        (get_align_y_offset_in_mm(font_prop.align.1, count_lines, font, font_prop)
            - get_align_y_offset_in_mm(font_prop.align.1, 1, font, font_prop)) as f32
    } else {
        0.0
    };
    TextLineLayout { line_height, first_line_offset_y }
}

pub fn get_count_lines(ws: &[char]) -> u32 {
    if ws.is_empty() {
        return 0;
    }

    let mut count = 1u32;
    for &wc in ws {
        if wc == '\n' {
            count += 1;
        }
    }
    count
}

pub mod from_utf8 {
    pub fn get_count_lines(text: &str) -> u32 {
        let ws: super::WString = text.chars().collect();
        super::get_count_lines(&ws)
    }
}

pub mod from_shapes {
    use super::*;

    pub fn get_count_lines(shapes: &ExPolygonsWithIds) -> u32 {
        if shapes.is_empty() {
            return 0; // no glyphs
        }
        let mut result = 1u32; // one line is minimum
        for shape_id in shapes {
            if shape_id.id == ENTER_UNICODE {
                result += 1;
            }
        }
        result
    }
}

pub fn get_line_ranges(shapes: &ExPolygonsWithIds) -> LineRanges {
    let mut result: LineRanges = LineRanges::new();
    let mut first: usize = 0;
    for i in 0..shapes.len() {
        if shapes[i].id != ENTER_UNICODE {
            continue;
        }
        result.push((first, i));
        first = i + 1;
    }
    result.push((first, shapes.len()));
    result
}

pub fn apply_transformation(angle: Option<f32>, distance: Option<f32>, transformation: &mut Transform3d) {
    if let Some(angle) = angle {
        let angle_z = angle as f64;
        let rot = eigen::AngleAxisd::new(angle_z, Vector3::new(0.0, 0.0, 1.0)).to_rotation_matrix();
        eigen::rotate(transformation, &rot);
    }
    if let Some(distance) = distance {
        let translate = Vector3::new(0.0, 0.0, 1.0) * (distance as f64);
        eigen::translate(transformation, &translate);
    }
}

pub fn is_italic(font: &FontFile, font_index: u32) -> bool {
    if font_index as usize >= font.infos.len() {
        return false;
    }
    let Some(info) = load_font_info(font.data.as_deref().unwrap_or(&[]), font_index) else {
        return false;
    };

    let name_id = 2;
    let value = imstb_truetype::stbtt_get_font_name_string(
        &info,
        imstb_truetype::STBTT_PLATFORM_ID_MICROSOFT,
        imstb_truetype::STBTT_MS_EID_UNICODE_BMP,
        imstb_truetype::STBTT_MS_LANG_ENGLISH,
        name_id,
    );
    let value = value.unwrap_or(&[]);
    let length = value.len();

    let mut value_str: Vec<u8> = Vec::with_capacity(length / 2);
    let mut i = 1;
    while i < length {
        value_str.push(value[i]);
        i += 2;
    }

    for c in value_str.iter_mut() {
        *c = c.to_ascii_lowercase();
    }

    let italics: [&[u8]; 2] = [b"italic", b"oblique"];
    for it in italics {
        if value_str.windows(it.len()).any(|w| w == it) {
            return true;
        }
    }
    false
}

pub fn remove_duplicates(input: &[char]) -> WString {
    let mut seen: std::collections::BTreeSet<char> = std::collections::BTreeSet::new();
    let mut result: WString = WString::new();
    for &c in input {
        if !seen.contains(&c) {
            seen.insert(c);
            result.push(c);
        }
    }
    result
}

pub mod with_fonts {
    use super::*;

    pub fn create_range_text(text: &mut String, fonts: Vec<Arc<FontFile>>, font_index: u32, exist_unknown: &mut bool) -> String {
        if fonts.is_empty() {
            return String::new();
        }
        *exist_unknown = false;
        let mut temp_exist_unknown = false;
        let mut not_dup_text: WString = remove_duplicates(&text.chars().collect::<WString>());
        not_dup_text.retain(|&wc| !(wc == '\n' || wc == '\r' || wc == '\t'));
        not_dup_text.sort();
        let mut results: Vec<String> = Vec::with_capacity(fonts.len());
        for i in 0..fonts.len() {
            let temp_text = super::create_range_text(text, &fonts[i], font_index, Some(&mut temp_exist_unknown));
            results.push(temp_text.clone());
            let valid_text: WString = temp_text.chars().collect();
            if valid_text.len() == not_dup_text.len() {
                if i > 0 {
                    *exist_unknown = true;
                }
                return results.last().unwrap().clone();
            }
        }
        *exist_unknown = true;
        for r in &results {
            if r.chars().count() == not_dup_text.len() {
                return r.clone();
            }
        }
        results[0].clone()
    }
}

pub fn create_range_text(text: &str, font: &FontFile, font_index: u32, mut exist_unknown: Option<&mut bool>) -> String {
    if !is_valid(font, font_index) {
        return String::new();
    }

    let mut ws: WString = text.chars().collect();

    ws.sort();

    let Some(font_info) = load_font_info(font.data.as_deref().unwrap_or(&[]), 0) else {
        return String::new();
    };

    if let Some(e) = exist_unknown.as_deref_mut() {
        *e = false;
    }
    let mut prev_unicode: i32 = -1;
    ws.retain(|&wc| {
        let unicode = wc as i32;

        if unicode == '\n' as i32 || unicode == '\r' as i32 || unicode == '\t' as i32 {
            return false;
        }

        if prev_unicode == unicode {
            return false;
        }
        prev_unicode = unicode;

        let is_unknown = imstb_truetype::stbtt_find_glyph_index(&font_info, unicode) == 0;
        if is_unknown {
            if let Some(e) = exist_unknown.as_deref_mut() {
                *e = true;
            }
        }
        !is_unknown
    });

    ws.iter().collect()
}

pub fn get_text_shape_scale(fp: &FontProp, ff: &FontFile) -> f64 {
    let info = get_font_info(ff, fp);
    let scale = (fp.size_in_mm as f64) / (info.unit_per_em as f64);
    scale * SHAPE_SCALE
}

fn add_indice(its: &mut indexed_triangle_set, f0: i32, f1: i32, f2: i32, check_pts_size: bool) {
    if f0 < 0 || f1 < 0 || f2 < 0 {
        return;
    }
    if check_pts_size {
        let pts_size = its.vertices.len();
        if (f0 as usize) < pts_size && (f1 as usize) < pts_size && (f2 as usize) < pts_size {
            its.indices.push(crate::normal_utils::Vec3crd::new(f0, f1, f2));
        }
    } else {
        its.indices.push(crate::normal_utils::Vec3crd::new(f0, f1, f2));
    }
}

fn add_quad(i1: u32, i2: u32, result: &mut indexed_triangle_set, count_point: u32) {
    let i1_ = i1.wrapping_add(count_point);
    let i2_ = i2.wrapping_add(count_point);
    add_indice(result, i2 as i32, i2_ as i32, i1 as i32, true);
    add_indice(result, i1_ as i32, i1 as i32, i2_ as i32, true);
}

fn polygons2model_unique(shape2d: &ExPolygons, projection: &dyn IProjection, points: &Points) -> indexed_triangle_set {
    let shape_triangles = crate::triangulation::triangulate_expolygons_with_points(shape2d, points)
        .expect("Triangulation::triangulate(shape2d, points)");
    let count_point = points.len() as u32;

    let mut result = indexed_triangle_set::default();
    result.vertices.reserve(2 * count_point as usize);
    let mut back_points: Vec<Vec3f> = Vec::with_capacity(count_point as usize);

    for p in points {
        let p2 = projection.create_front_back(p);
        result.vertices.push(Vec3f::new(p2.0.x as f32, p2.0.y as f32, p2.0.z as f32));
        back_points.push(Vec3f::new(p2.1.x as f32, p2.1.y as f32, p2.1.z as f32));
    }

    result.vertices.extend(back_points);
    result.indices.reserve(shape_triangles.len() * 2 + points.len() * 2);
    for t in &shape_triangles {
        add_indice(&mut result, t[0], t[2], t[1], true);
    }
    for t in &shape_triangles {
        add_indice(
            &mut result,
            t[0].wrapping_add(count_point as i32),
            t[1].wrapping_add(count_point as i32),
            t[2].wrapping_add(count_point as i32),
            true,
        );
    }

    let mut polygon_offset: usize = 0;
    let mut add_quads = |polygon: &Polygon, result: &mut indexed_triangle_set| {
        let polygon_points = polygon.points().len() as u32;
        let mut prev = (polygon_offset as u32).wrapping_add(polygon_points).wrapping_sub(1);
        for p in 0..polygon_points {
            let index = polygon_offset as u32 + p;
            add_quad(prev, index, result, count_point);
            prev = index;
        }
        polygon_offset += polygon_points as usize;
    };

    for expolygon in shape2d {
        add_quads(&expolygon.contour, &mut result);
        for hole in &expolygon.holes {
            add_quads(hole, &mut result);
        }
    }

    result
}

fn polygons2model_duplicit(
    shape2d: &ExPolygons,
    projection: &dyn IProjection,
    points: &Points,
    duplicits: &Points,
) -> indexed_triangle_set {
    let changes = crate::triangulation::create_changes(points, duplicits);
    let shape_triangles = crate::triangulation::triangulate_expolygons_with_changes(shape2d, points, &changes)
        .expect("Triangulation::triangulate(shape2d, points, changes)");
    let count_point = *changes.iter().max().unwrap() + 1;

    let mut result = indexed_triangle_set::default();
    result.vertices.reserve(2 * count_point as usize);
    let mut back_points: Vec<Vec3f> = Vec::with_capacity(count_point as usize);

    let mut max_index = u32::MAX;
    for i in 0..changes.len() as u32 {
        let index = changes[i as usize];
        if max_index != u32::MAX && index <= max_index {
            continue; // duplicit point
        }
        debug_assert!(index == max_index.wrapping_add(1));
        debug_assert!(result.vertices.len() == index as usize);
        debug_assert!(back_points.len() == index as usize);
        max_index = index;
        let p = &points[i as usize];
        let p2 = projection.create_front_back(p);
        result.vertices.push(Vec3f::new(p2.0.x as f32, p2.0.y as f32, p2.0.z as f32));
        back_points.push(Vec3f::new(p2.1.x as f32, p2.1.y as f32, p2.1.z as f32));
    }
    debug_assert!(max_index.wrapping_add(1) == count_point);

    result.vertices.extend(back_points);

    result.indices.reserve(shape_triangles.len() * 2 + points.len() * 2);
    for t in &shape_triangles {
        add_indice(&mut result, t[0], t[2], t[1], true);
    }
    for t in &shape_triangles {
        add_indice(
            &mut result,
            t[0].wrapping_add(count_point as i32),
            t[1].wrapping_add(count_point as i32),
            t[2].wrapping_add(count_point as i32),
            true,
        );
    }

    let mut polygon_offset: usize = 0;
    let mut add_quads = |polygon: &Polygon, result: &mut indexed_triangle_set| {
        let polygon_points = polygon.points().len() as u32;
        let mut prev = changes[polygon_offset + polygon_points as usize - 1];
        for p in 0..polygon_points {
            let index = changes[polygon_offset + p as usize];
            if prev == index {
                continue;
            }
            add_quad(prev, index, result, count_point);
            prev = index;
        }
        polygon_offset += polygon_points as usize;
    };

    for expolygon in shape2d {
        add_quads(&expolygon.contour, &mut result);
        for hole in &expolygon.holes {
            add_quads(hole, &mut result);
        }
    }
    result
}

pub fn polygons2model(shape2d: &ExPolygons, projection: &dyn IProjection) -> indexed_triangle_set {
    let points = to_points(shape2d);
    let duplicits = collect_duplicates(points.clone());
    if duplicits.is_empty() {
        polygons2model_unique(shape2d, projection, &points)
    } else {
        polygons2model_duplicit(shape2d, projection, &points, &duplicits)
    }
}


// ===========================================================================
// Projections (Emboss.hpp:199-363, Emboss.cpp:1847-1975)
// ===========================================================================

// Emboss.hpp:199-211  class IProject3d
pub trait IProject3d {
    // Emboss.hpp:210  virtual Vec3d project(const Vec3d &point) const = 0;
    fn project(&self, point: &Vec3d) -> Vec3d;
}

// Emboss.hpp:217-239  class IProjection : public IProject3d
pub trait IProjection: IProject3d {
    // Emboss.hpp:230  virtual std::pair<Vec3d, Vec3d> create_front_back(const Point &p) const = 0;
    fn create_front_back(&self, p: &Point) -> (Vec3d, Vec3d);
    // Emboss.hpp:238  virtual std::optional<Vec2d> unproject(const Vec3d &p, double * depth = nullptr) const = 0;
    fn unproject(&self, p: &Vec3d, depth: Option<&mut f64>) -> Option<Vec2d>;
}

// Emboss.hpp:275-284  class ProjectZ : public IProjection
#[derive(Debug, Clone, Copy)]
pub struct ProjectZ {
    // Emboss.hpp:283  double m_depth;
    pub m_depth: f64,
}

impl ProjectZ {
    // Emboss.hpp:278  explicit ProjectZ(double depth) : m_depth(depth) {}
    pub fn new(depth: f64) -> Self {
        ProjectZ { m_depth: depth }
    }
}

impl IProject3d for ProjectZ {
    // Emboss.cpp:1853-1858
    // Vec3d Emboss::ProjectZ::project(const Vec3d &point) const
    fn project(&self, point: &Vec3d) -> Vec3d {
        let mut res = *point; // copy
        res.z = self.m_depth;
        res
    }
}

impl IProjection for ProjectZ {
    // Emboss.cpp:1847-1851
    // std::pair<Vec3d, Vec3d> Emboss::ProjectZ::create_front_back(const Point &p) const
    fn create_front_back(&self, p: &Point) -> (Vec3d, Vec3d) {
        let front = Vec3d::new(p.x as f64, p.y as f64, 0.0);
        (front, self.project(&front))
    }

    // Emboss.cpp:1860-1862
    // std::optional<Vec2d> Emboss::ProjectZ::unproject(const Vec3d &p, double *depth) const
    fn unproject(&self, p: &Vec3d, _depth: Option<&mut f64>) -> Option<Vec2d> {
        Some(Vec2d::new(p.x, p.y))
    }
}

// Emboss.hpp:286-309  class ProjectScale : public IProjection
pub struct ProjectScale {
    // Emboss.hpp:288  std::unique_ptr<IProjection> core;
    core: Box<dyn IProjection>,
    // Emboss.hpp:289  double m_scale;
    m_scale: f64,
}

impl ProjectScale {
    // Emboss.hpp:291-293
    pub fn new(core: Box<dyn IProjection>, scale: f64) -> Self {
        ProjectScale { core, m_scale: scale }
    }
}

impl IProject3d for ProjectScale {
    // Emboss.hpp:301-303  Vec3d project(const Vec3d &point) const override { return core->project(point); }
    fn project(&self, point: &Vec3d) -> Vec3d {
        self.core.project(point)
    }
}

impl IProjection for ProjectScale {
    // Emboss.hpp:296-300
    fn create_front_back(&self, p: &Point) -> (Vec3d, Vec3d) {
        let res = self.core.create_front_back(p);
        let m = |v: Vec3d| Vec3d::new(v.x * self.m_scale, v.y * self.m_scale, v.z * self.m_scale);
        (m(res.0), m(res.1))
    }

    // Emboss.hpp:304-308
    fn unproject(&self, p: &Vec3d, mut depth: Option<&mut f64>) -> Option<Vec2d> {
        // auto res = core->unproject(p / m_scale, depth);
        let scaled = Vec3d::new(p.x / self.m_scale, p.y / self.m_scale, p.z / self.m_scale);
        let res = self.core.unproject(&scaled, depth.as_deref_mut());
        // if (depth != nullptr) *depth *= m_scale;
        if let Some(d) = depth {
            *d *= self.m_scale;
        }
        res
    }
}

// Emboss.hpp:311-339  class ProjectTransform : public IProjection
pub struct ProjectTransform {
    // Emboss.hpp:313  std::unique_ptr<IProjection> m_core;
    m_core: Box<dyn IProjection>,
    // Emboss.hpp:314  Transform3d m_tr;
    m_tr: Transform3d,
    // Emboss.hpp:315  Transform3d m_tr_inv;
    m_tr_inv: Transform3d,
    // Emboss.hpp:316  double z_scale;
    z_scale: f64,
}

impl ProjectTransform {
    // Emboss.hpp:318-322
    pub fn new(core: Box<dyn IProjection>, tr: Transform3d) -> Self {
        // Emboss.hpp:320  m_tr_inv = m_tr.inverse();
        let m_tr_inv = eigen::transform_inverse(&tr);
        // Emboss.hpp:321  z_scale = (m_tr.linear() * Vec3d::UnitZ()).norm();
        let z_scale = eigen::norm3(&eigen::mat3_mul_vec(&eigen::linear(&tr), &Vector3::new(0.0, 0.0, 1.0)));
        ProjectTransform {
            m_core: core,
            m_tr: tr,
            m_tr_inv,
            z_scale,
        }
    }
}

impl IProject3d for ProjectTransform {
    // Emboss.hpp:330-332  Vec3d project(const Vec3d &point) const override { return m_core->project(point); }
    fn project(&self, point: &Vec3d) -> Vec3d {
        self.m_core.project(point)
    }
}

impl IProjection for ProjectTransform {
    // Emboss.hpp:325-329
    fn create_front_back(&self, p: &Point) -> (Vec3d, Vec3d) {
        let (front, back) = self.m_core.create_front_back(p);
        (
            transform_point(&self.m_tr, &front),
            transform_point(&self.m_tr, &back),
        )
    }

    // Emboss.hpp:333-338
    fn unproject(&self, p: &Vec3d, mut depth: Option<&mut f64>) -> Option<Vec2d> {
        // auto res = m_core->unproject(m_tr_inv * p, depth);
        let inv_p = transform_point(&self.m_tr_inv, p);
        let res = self.m_core.unproject(&inv_p, depth.as_deref_mut());
        // if (depth != nullptr) *depth *= z_scale;
        if let Some(d) = depth {
            *d *= self.z_scale;
        }
        res
    }
}

// Emboss.hpp:341-348  class OrthoProject3d : public Emboss::IProject3d
#[derive(Debug, Clone, Copy)]
pub struct OrthoProject3d {
    // Emboss.hpp:344  size and direction of emboss for ortho projection  Vec3d m_direction;
    m_direction: Vec3d,
}

impl OrthoProject3d {
    // Emboss.hpp:346  OrthoProject3d(Vec3d direction) : m_direction(direction) {}
    pub fn new(direction: Vec3d) -> Self {
        OrthoProject3d { m_direction: direction }
    }
}

impl IProject3d for OrthoProject3d {
    // Emboss.hpp:347  Vec3d project(const Vec3d &point) const override { return point + m_direction; }
    fn project(&self, point: &Vec3d) -> Vec3d {
        *point + self.m_direction
    }
}

// Emboss.hpp:350-363  class OrthoProject: public Emboss::IProjection
#[derive(Debug, Clone, Copy)]
pub struct OrthoProject {
    // Emboss.hpp:351  Transform3d m_matrix;
    m_matrix: Transform3d,
    // Emboss.hpp:353  size and direction of emboss for ortho projection  Vec3d m_direction;
    m_direction: Vec3d,
    // Emboss.hpp:354  Transform3d m_matrix_inv;
    m_matrix_inv: Transform3d,
}

impl OrthoProject {
    // Emboss.hpp:356-358  OrthoProject(Transform3d matrix, Vec3d direction)
    pub fn new(matrix: Transform3d, direction: Vec3d) -> Self {
        OrthoProject {
            m_matrix: matrix,
            m_direction: direction,
            // Emboss.hpp:357  m_matrix_inv(matrix.inverse())
            m_matrix_inv: eigen::transform_inverse(&matrix),
        }
    }
}

impl IProject3d for OrthoProject {
    // Emboss.cpp:1965-1968  Vec3d Emboss::OrthoProject::project(const Vec3d &point) const
    fn project(&self, point: &Vec3d) -> Vec3d {
        *point + self.m_direction
    }
}

impl IProjection for OrthoProject {
    // Emboss.cpp:1959-1963
    // std::pair<Vec3d, Vec3d> Emboss::OrthoProject::create_front_back(const Point &p) const
    fn create_front_back(&self, p: &Point) -> (Vec3d, Vec3d) {
        let front = Vec3d::new(p.x as f64, p.y as f64, 0.0);
        let front_tr = transform_point(&self.m_matrix, &front);
        (front_tr, self.project(&front_tr))
    }

    // Emboss.cpp:1970-1975
    // std::optional<Vec2d> Emboss::OrthoProject::unproject(const Vec3d &p, double *depth) const
    fn unproject(&self, p: &Vec3d, depth: Option<&mut f64>) -> Option<Vec2d> {
        let pp = transform_point(&self.m_matrix_inv, p);
        if let Some(d) = depth {
            *d = pp.z;
        }
        Some(Vec2d::new(pp.x, pp.y))
    }
}

// ===========================================================================
// suggest_up / calc_up / create_transformation_onto_surface (Emboss.cpp:1865-1954)
// ===========================================================================

// Emboss.cpp:1889-1906
// Vec3d Emboss::suggest_up(const Vec3d normal, double up_limit)
// The vector algebra of this section is Eigen's, in Eigen's evaluation order (`geometry::eigen`).
pub fn suggest_up(normal: Vec3d, up_limit: f64) -> Vec3d {
    gv(&suggest_up_e(&ev(&normal), up_limit))
}

fn suggest_up_e(normal: &eigen::Vec3d, up_limit: f64) -> eigen::Vec3d {
    // Emboss.cpp:1891-1892  Normal must be 1
    debug_assert!(is_approx(eigen::squared_norm3(normal), 1.0));

    // Emboss.cpp:1894-1897  wanted up direction of result
    let wanted_up_side = if normal.z.abs() > up_limit {
        eigen::Vec3d::new(0.0, 1.0, 0.0) // Vec3d::UnitY()
    } else {
        eigen::Vec3d::new(0.0, 0.0, 1.0) // Vec3d::UnitZ()
    };

    // Emboss.cpp:1899-1901
    // create perpendicular unit vector to surface triangle normal vector
    // lay on surface of triangle and define up vector for text
    let mut wanted_up_dir = eigen::cross3(&eigen::cross3(normal, &wanted_up_side), normal);
    // Emboss.cpp:1902-1903  normal3d is NOT perpendicular to normal_up_dir
    eigen::normalize3(&mut wanted_up_dir);

    // Emboss.cpp:1905
    wanted_up_dir
}

// Emboss.cpp:1908-1930
// std::optional<float> Emboss::calc_up(const Transform3d &tr, double up_limit)
pub fn calc_up(tr: &Transform3d, up_limit: f64) -> Option<f32> {
    // Emboss.cpp:1910
    let tr_linear = eigen::linear(tr);
    // Emboss.cpp:1911-1912  z base of transformation ( tr * UnitZ )
    let mut normal = eigen::Vec3d::new(tr_linear[(0, 2)], tr_linear[(1, 2)], tr_linear[(2, 2)]);
    // Emboss.cpp:1913-1914  scaled matrix has base with different size
    eigen::normalize3(&mut normal);
    // Emboss.cpp:1915-1916
    let suggested = suggest_up_e(&normal, up_limit);
    debug_assert!(is_approx(eigen::squared_norm3(&suggested), 1.0));

    // Emboss.cpp:1918-1919  up = tr_linear.col(1); // tr * UnitY()
    let mut up = eigen::Vec3d::new(tr_linear[(0, 1)], tr_linear[(1, 1)], tr_linear[(2, 1)]);
    eigen::normalize3(&mut up);
    // Emboss.cpp:1920-1923
    // Matrix3d m; m.row(0) = up; m.row(1) = suggested; m.row(2) = normal;
    let m = Matrix3::<f64>::new(
        up.x, up.y, up.z,
        suggested.x, suggested.y, suggested.z,
        normal.x, normal.y, normal.z,
    );
    // Emboss.cpp:1924
    let det = eigen::determinant3(&m);
    // Emboss.cpp:1925  double dot = suggested.dot(up);
    let dot = eigen::dot3(&suggested, &up);
    // Emboss.cpp:1926  double res = -atan2(det, dot);
    let res = -f64::atan2(det, dot);
    // Emboss.cpp:1927-1928
    if is_approx(res, 0.0) {
        return None;
    }
    // Emboss.cpp:1929  return res; // implicit narrowing double -> float (std::optional<float>)
    Some(res as f32)
}

// Emboss.cpp:1932-1978
// Transform3d Emboss::create_transformation_onto_surface(const Vec3d &position, const Vec3d &normal, double up_limit)
pub fn create_transformation_onto_surface(position: Vec3d, normal: Vec3d, up_limit: f64) -> Transform3d {
    let position = ev(&position);
    let normal = ev(&normal);
    // Emboss.cpp:1936-1937  is normalized ?
    debug_assert!(is_approx(eigen::squared_norm3(&normal), 1.0));

    // Emboss.cpp:1939-1941  up and emboss direction for generated model
    let up_dir = eigen::Vec3d::new(0.0, 1.0, 0.0); // Vec3d::UnitY()
    let emboss_dir = eigen::Vec3d::new(0.0, 0.0, 1.0); // Vec3d::UnitZ()

    // Emboss.cpp:1943-1944  after cast from float it needs to be normalized again
    let wanted_up_dir = suggest_up_e(&normal, up_limit);

    // Emboss.cpp:1946-1957  perpendicular to emboss vector of text and normal
    let mut axis_view;
    let angle_view;
    if normal == eigen::Vec3d::new(-0.0, -0.0, -1.0) {
        // text_emboss_dir has opposit direction to wanted_emboss_dir
        axis_view = eigen::Vec3d::new(0.0, 1.0, 0.0); // Vec3d::UnitY()
        angle_view = std::f64::consts::PI;
    } else {
        axis_view = eigen::cross3(&emboss_dir, &normal);
        angle_view = eigen::dot3(&emboss_dir, &normal).acos(); // in rad
        eigen::normalize3(&mut axis_view);
    }

    // Emboss.cpp:1959  Eigen::AngleAxis view_rot(angle_view, axis_view);
    let view_rot = eigen::AngleAxisd::new(angle_view, axis_view);
    // Emboss.cpp:1960-1961  wanterd_up_rotated = view_rot.matrix().inverse() * wanted_up_dir;
    let view_rot_matrix = view_rot.to_rotation_matrix();
    let mut wanterd_up_rotated = eigen::mat3_mul_vec(&eigen::mat3_inverse(&view_rot_matrix), &wanted_up_dir);
    eigen::normalize3(&mut wanterd_up_rotated);
    // Emboss.cpp:1962  double angle_up = std::acos(up_dir.dot(wanterd_up_rotated));
    let mut angle_up = eigen::dot3(&up_dir, &wanterd_up_rotated).acos();

    // Emboss.cpp:1964-1969
    let text_view = eigen::cross3(&up_dir, &wanterd_up_rotated);
    let diff_view = eigen::Vec3d::new(emboss_dir.x - text_view.x, emboss_dir.y - text_view.y, emboss_dir.z - text_view.z);
    if diff_view.x.abs() > 1.0 || diff_view.y.abs() > 1.0 || diff_view.z.abs() > 1.0 {
        // oposit direction
        angle_up *= -1.0;
    }

    // Emboss.cpp:1971  Eigen::AngleAxis up_rot(angle_up, emboss_dir);
    let up_rot = eigen::AngleAxisd::new(angle_up, emboss_dir);

    // Emboss.cpp:1973-1977
    let mut transform = eigen::identity();
    eigen::translate(&mut transform, &position);
    eigen::rotate(&mut transform, &view_rot_matrix);
    eigen::rotate(&mut transform, &up_rot.to_rotation_matrix());
    transform
}

// ===========================================================================
// sample_slice + point_in_distance + calculate_angle (Emboss.cpp:1977-2154)
// ===========================================================================

// Emboss.cpp:1981-1982  using Coord2 = double; using P2 = Eigen::Matrix<Coord2, 2, 1>;
type Coord2 = f64;

// Emboss.cpp:1984-2050
fn point_in_distance(
    distance_sq: Coord2,
    polygon_point: &mut PolygonPoint,
    i: usize,
    polygon: &Polygon,
    is_first: bool,
    is_reverse: bool,
) -> bool {
    // Emboss.cpp:1986-1987
    let s = polygon.points().len();
    let ii = (i + polygon_point.index) % s;

    // Emboss.cpp:1989-1991  second point of line
    let p = polygon.points()[ii];
    let p_d = p - polygon_point.point;

    // Emboss.cpp:1993-1994
    let p_d2 = Vec2d::new(p_d.x as f64, p_d.y as f64);
    let p_distance_sq = p_d2.dot(&p_d2);
    // Emboss.cpp:1995-1996
    if p_distance_sq < distance_sq {
        return false;
    }

    // Emboss.cpp:1998-2005  found line
    if is_first {
        // on same line; center also lay on line
        // new point is distance moved from point by direction
        let factor = (distance_sq / p_distance_sq).sqrt();
        // `p_d` is a non-const `Point`: `p_d * double` is the member operator (Point.hpp:200),
        // which builds `Point(double, double)`, a round to nearest (lrint).
        polygon_point.point = polygon_point.point
            + Point::new(
                (p_d.x as f64 * factor).round_ties_even() as i64,
                (p_d.y as f64 * factor).round_ties_even() as i64,
            );
        return true;
    }

    // Emboss.cpp:2007-2012  line cross circle; start point of line
    let ii2 = if is_reverse {
        (ii + 1) % s
    } else {
        (ii + s - 1) % s
    };
    polygon_point.index = if is_reverse { ii } else { ii2 };
    let p2 = polygon.points()[ii2];

    // Emboss.cpp:2014-2019
    let line_dir = p2 - p;
    let line_dir2 = Vec2d::new(line_dir.x as f64, line_dir.y as f64);

    let a = line_dir2.dot(&line_dir2);
    let b = 2.0 * p_d2.dot(&line_dir2);
    let c = p_d2.dot(&p_d2) - distance_sq;

    // Emboss.cpp:2021-2027
    let mut discriminant = b.mul_add(b, -(4.0 * a * c));
    if discriminant < 0.0 {
        debug_assert!(false);
        // no intersection
        polygon_point.point = p;
        return true;
    }

    // Emboss.cpp:2029-2032
    // ray didn't totally miss sphere, so there is a solution to the equation.
    discriminant = discriminant.sqrt();

    // Emboss.cpp:2034-2038
    // either solution may be on or off the ray so need to test both
    // t1 is always the smaller value, because BOTH discriminant and a are nonnegative.
    let t1 = (-b - discriminant) / (2.0 * a);
    let t2 = (-b + discriminant) / (2.0 * a);

    // Emboss.cpp:2040
    let t = t1.min(t2);
    // Emboss.cpp:2041-2046
    if t < 0.0 || t > 1.0 {
        // Bad intersection
        debug_assert!(false);
        polygon_point.point = p;
        return true;
    }

    // Emboss.cpp:2048
    polygon_point.point = p
        + Point::new(
            (t * line_dir2.x) as i64,
            (t * line_dir2.y) as i64,
        );
    // Emboss.cpp:2049
    true
}

mod by_distance {
    use super::*;

    // Emboss.cpp:2076-2086 (anonymous namespace)
    pub(super) fn point_in_distance(distance: i32, p: &mut PolygonPoint, polygon: &Polygon) {
        let distance_sq = (distance as Coord2) * (distance as Coord2);
        let mut is_first = true;
        for i in 1..polygon.points().len() {
            if super::point_in_distance(distance_sq, p, i, polygon, is_first, false) {
                return;
            }
            is_first = false;
        }
        // There is not point on polygon with this distance
    }
}

// Emboss.cpp:2088-2099 (anonymous namespace)
fn point_in_reverse_distance(distance: i32, p: &mut PolygonPoint, polygon: &Polygon) {
    let distance_sq = (distance as Coord2) * (distance as Coord2);
    let mut is_first = true;
    let is_reverse = true;
    let mut i = polygon.points().len();
    while i > 0 {
        if point_in_distance(distance_sq, p, i, polygon, is_first, is_reverse) {
            return;
        }
        is_first = false;
        i -= 1;
    }
    // There is not point on polygon with this distance
}

// Emboss.cpp:2079-2090
// double Emboss::calculate_angle(int32_t distance, PolygonPoint polygon_point, const Polygon &polygon)
pub fn calculate_angle(distance: i32, polygon_point: PolygonPoint, polygon: &Polygon) -> f64 {
    // Emboss.cpp:2081  PolygonPoint polygon_point2 = polygon_point; // copy
    let mut polygon_point = polygon_point;
    let mut polygon_point2 = polygon_point;
    // Emboss.cpp:2082-2083
    by_distance::point_in_distance(distance, &mut polygon_point, polygon);
    point_in_reverse_distance(distance, &mut polygon_point2, polygon);

    // Emboss.cpp:2085-2086
    let surface_dir = polygon_point2.point - polygon_point.point;
    let norm = Point::new(-surface_dir.y, surface_dir.x);
    // Emboss.cpp:2087-2089
    let norm_d = Vec2d::new(norm.x as f64, norm.y as f64);
    //norm_d.normalize();
    f64::atan2(norm_d.y, norm_d.x)
}

// Emboss.cpp:2092-2099
// std::vector<double> Emboss::calculate_angles(int32_t distance, const PolygonPoints& polygon_points, const Polygon &polygon)
pub fn calculate_angles(distance: i32, polygon_points: &PolygonPoints, polygon: &Polygon) -> Vec<f64> {
    let mut result: Vec<f64> = Vec::with_capacity(polygon_points.len());
    for pp in polygon_points {
        result.push(calculate_angle(distance, *pp, polygon));
    }
    result
}

// Emboss.cpp:2101-2154
// PolygonPoints Emboss::sample_slice(const TextLine &slice, const BoundingBoxes &bbs, double scale)
pub fn sample_slice(slice: &TextLine, bbs: &[BoundingBox], scale: f64) -> PolygonPoints {
    // Emboss.cpp:2104-2111  find BB in center of line
    let mut first_right_index = 0usize;
    for bb in bbs {
        // NOTE: Slic3r BoundingBox has `defined` flag; here an empty bb is "undefined".
        if !bb_defined(bb) {
            // white char do not have bb
            continue;
        } else if bb.min.x < 0 {
            first_right_index += 1;
        } else {
            break;
        }
    }

    // Emboss.cpp:2113-2114
    let mut samples: PolygonPoints = vec![slice.start; bbs.len()];
    let mut shapes_x_cursor: i32 = 0;

    // Emboss.cpp:2116
    let mut cursor = slice.start; // copy

    // Emboss.cpp:2118-2132  create_sample lambda (inlined call sites below)
    let create_sample =
        |bb: &BoundingBox, is_reverse: bool, cursor: &mut PolygonPoint, shapes_x_cursor: &mut i32| -> PolygonPoint {
            // Emboss.cpp:2120-2121
            if !bb_defined(bb) {
                return *cursor;
            }
            // Emboss.cpp:2122-2125
            let letter_center = bb.center();
            let shape_distance = *shapes_x_cursor - letter_center.x as i32;
            *shapes_x_cursor = letter_center.x as i32;
            let distance_mm = shape_distance as f64 * scale;
            // Emboss.cpp:2126
            // scale_(val) is `(val) / SCALING_FACTOR` (libslic3r.h:81), SCALING_FACTOR the double 0.00001
            let distance_polygon = (distance_mm / 0.00001).round() as i32;
            // Emboss.cpp:2127-2130
            if is_reverse {
                by_distance::point_in_distance(distance_polygon, cursor, &slice.polygon);
            } else {
                point_in_reverse_distance(distance_polygon, cursor, &slice.polygon);
            }
            // Emboss.cpp:2131
            *cursor
        };

    // Emboss.cpp:2134-2137  calc transformation for letters on the Right side from center
    let is_reverse = true;
    for index in first_right_index..bbs.len() {
        samples[index] = create_sample(&bbs[index], is_reverse, &mut cursor, &mut shapes_x_cursor);
    }

    // Emboss.cpp:2139-2147  calc transformation for letters on the Left side from center
    if first_right_index < bbs.len() {
        shapes_x_cursor = bbs[first_right_index].center().x as i32;
        cursor = samples[first_right_index];
    } else {
        // only left side exists
        shapes_x_cursor = 0;
        cursor = slice.start; // copy
    }
    // Emboss.cpp:2148-2152
    let is_reverse = false;
    let mut index_plus_one = first_right_index;
    while index_plus_one > 0 {
        let index = index_plus_one - 1;
        samples[index] = create_sample(&bbs[index], is_reverse, &mut cursor, &mut shapes_x_cursor);
        index_plus_one -= 1;
    }
    // Emboss.cpp:2153
    samples
}

// ===========================================================================
// Local glue helpers bridging C++ free functions to crate primitives.
// ===========================================================================

// ===========================================================================
// Vertical / Horizontal align (Emboss.cpp:2156-2302)
// ===========================================================================

// Emboss.cpp:2157-2173 (anonymous namespace)
// float get_align_y_offset(FontProp::VerticalAlign align, unsigned count_lines, const FontFile &ff, const FontProp &fp)
pub fn get_align_y_offset(
    align: crate::text_configuration::VerticalAlign,
    count_lines: u32,
    ff: &FontFile,
    fp: &crate::text_configuration::FontProp,
) -> f32 {
    use crate::text_configuration::VerticalAlign;
    // Emboss.cpp:2159
    // assert(count_lines != 0);  (text2vshapes of an empty text comes here with 0 lines: the
    // release build goes on with the unsigned `count_lines - 1`)
    // Emboss.cpp:2160-2162
    let line_height = get_line_height(ff, fp);
    let ascent = ((get_font_info(ff, fp).ascent as f64) / SHAPE_SCALE) as i32;
    let line_center = ((ascent as f64) * ASCENT_CENTER).round() as f32;

    // Emboss.cpp:2164-2172
    // direction of Y in 2d is from top to bottom
    // zero is on base line of first line
    match align {
        // Emboss.cpp:2167  bottom: return line_height * (count_lines - 1);
        // int * unsigned is unsigned arithmetic
        VerticalAlign::Bottom => (line_height as u32).wrapping_mul(count_lines.wrapping_sub(1)) as f32,
        // Emboss.cpp:2168  top: return -ascent;
        VerticalAlign::Top => -ascent as f32,
        // Emboss.cpp:2169-2171  center (default): return -line_center + line_height * (count_lines - 1) / 2.;
        // Mirror C++: `line_height * (count_lines - 1)` is integer multiplication, then `/ 2.`
        // promotes to floating point division.
        // (the sum is a double, converted to float by the return)
        VerticalAlign::Center => {
            ((-line_center) as f64 + (line_height as u32).wrapping_mul(count_lines.wrapping_sub(1)) as f64 / 2.) as f32
        }
    }
}

// Emboss.cpp:2175-2184 (anonymous namespace)
// int32_t get_align_x_offset(FontProp::HorizontalAlign align, const BoundingBox &shape_bb, const BoundingBox &line_bb)
#[allow(dead_code)]
pub fn get_align_x_offset(
    align: crate::text_configuration::HorizontalAlign,
    shape_bb: &BoundingBox,
    line_bb: &BoundingBox,
) -> i32 {
    use crate::text_configuration::HorizontalAlign;
    match align {
        // Emboss.cpp:2178  right: return -shape_bb.max.x() + (shape_bb.size().x() - line_bb.size().x());
        HorizontalAlign::Right => {
            (-bb_max(shape_bb).x + (shape_bb.size().x - line_bb.size().x)) as i32
        }
        // Emboss.cpp:2179  center: return -shape_bb.center().x() + (shape_bb.size().x() - line_bb.size().x()) / 2;
        HorizontalAlign::Center => {
            (-shape_bb.center().x + (shape_bb.size().x - line_bb.size().x) / 2) as i32
        }
        // Emboss.cpp:2180-2181  left: no change
        HorizontalAlign::Left => 0,
    }
}

// `BoundingBox::min` / `max` as the C++ members read: a default constructed (undefined)
// `BoundingBoxBase` holds `min = max = Zero` (BoundingBox.hpp), the crate's holds sentinels.
fn bb_max(bb: &BoundingBox) -> Point {
    if bb.is_defined() {
        bb.max
    } else {
        Point::new(0, 0)
    }
}

// ExPolygon.cpp:17-22 `ExPolygon::scale(double factor)` over MultiPoint.cpp:6-10 and
// Point.hpp:199 `Point::operator*=(const double &rhs)`: `coord_t(x * rhs)`, a truncation.
fn expolygon_scale(expolygon: &mut ExPolygon, factor: f64) {
    let scale = |polygon: &mut Polygon| {
        for pt in polygon.points_mut().iter_mut() {
            pt.x = (pt.x as f64 * factor) as i64;
            pt.y = (pt.y as f64 * factor) as i64;
        }
    };
    scale(&mut expolygon.contour);
    for hole in expolygon.holes.iter_mut() {
        scale(hole);
    }
}

// Emboss.cpp:2210-2256 (anonymous namespace)
fn align_shape(shapes: &mut ExPolygonsWithIds, offset_xy: &mut Vec<Point>, text: &[char], prop: &FontProp, font: &FontFile) {
    // Emboss.cpp:2212-2213  Shapes have to match letters in text
    debug_assert!(shapes.len() == text.len());

    // Emboss.cpp:2215-2216
    let count_lines = get_count_lines(text);
    let y_offset: i32 = get_align_y_offset(prop.align.1, count_lines, font, prop) as i32;

    // Emboss.cpp:2218-2226  Speed up for left aligned text
    if prop.align.0 == HorizontalAlign::Left {
        // already horizontaly aligned
        offset_xy.clear();
        offset_xy.resize(shapes.len(), Point::new(0, y_offset as i64));
        for shape in shapes.iter_mut() {
            for s in shape.expoly.iter_mut() {
                s.translate(Point::new(0, y_offset as i64));
            }
        }
        return;
    }

    // Emboss.cpp:2228-2230
    let mut shape_bb = BoundingBox::new();
    for shape in shapes.iter() {
        shape_bb.merge(&get_extents_expolygons(&shape.expoly));
    }

    // Emboss.cpp:2232-2237
    let get_line_bb = |shapes: &ExPolygonsWithIds, mut j: usize| {
        let mut line_bb = BoundingBox::new();
        while j < text.len() && text[j] != '\n' {
            line_bb.merge(&get_extents_expolygons(&shapes[j].expoly));
            j += 1;
        }
        line_bb
    };

    // Emboss.cpp:2239-2255  Align x line by line
    let mut offset = Point::new(
        get_align_x_offset(prop.align.0, &shape_bb, &get_line_bb(shapes, 0)) as i64,
        y_offset as i64,
    );
    for i in 0..shapes.len() {
        let letter = text[i];
        if letter == '\n' {
            // keep offsets aligned with shapes
            offset_xy.push(offset);
            offset.x = get_align_x_offset(prop.align.0, &shape_bb, &get_line_bb(shapes, i + 1)) as i64;
            continue;
        }
        let shape = &mut shapes[i].expoly;
        offset_xy.push(offset);
        for s in shape.iter_mut() {
            s.translate(offset);
        }
    }
}

mod align_with_back_fonts {
    use super::*;

    // Emboss.cpp:2258-2325 (anonymous namespace)
    #[allow(clippy::too_many_arguments)]
    pub(super) fn align_shape(
        shapes: &mut ExPolygonsWithIds,
        offset_xy: &mut Vec<Point>,
        text_scales: &mut Vec<f32>,
        standard_scale: f32,
        real_fonts: Vec<FontFileWithCache>,
        text: &[char],
        prop: &FontProp,
        font: &FontFile,
    ) {
        // Emboss.cpp:2264-2265  Shapes have to match letters in text
        debug_assert!(shapes.len() == text.len());

        // Emboss.cpp:2267-2269
        let count_lines = get_count_lines(text);
        let main_y_offset: i32 = get_align_y_offset(prop.align.1, count_lines, font, prop) as i32;
        let main_single_line_y_offset: f32 = get_align_y_offset(prop.align.1, 1, font, prop);
        // Emboss.cpp:2270-2280
        let glyph_y_offset = |index: usize| -> i32 {
            if !real_fonts[index].has_value() || text_scales[index] <= 0.0 {
                return main_y_offset;
            }

            let fallback_font: &FontFile = real_fonts[index].font_file.as_deref().unwrap();
            let fallback_scale: f32 = text_scales[index] / standard_scale;
            let fallback_single_line_y_offset: f32 =
                get_align_y_offset(prop.align.1, 1, fallback_font, prop) * fallback_scale;
            // std::lround(float): to nearest, halfway cases away from zero
            (main_y_offset as f32 + fallback_single_line_y_offset - main_single_line_y_offset).round() as i64 as i32
        };

        // Emboss.cpp:2282-2294  Speed up for left aligned text
        if prop.align.0 == HorizontalAlign::Left {
            // already horizontaly aligned
            for index in 0..shapes.len() {
                let temp_y_offset = glyph_y_offset(index);
                let shape = &mut shapes[index];
                offset_xy.push(Point::new(0, temp_y_offset as i64));
                for s in shape.expoly.iter_mut() {
                    s.translate(Point::new(0, temp_y_offset as i64));
                }
            }
            return;
        }

        // Emboss.cpp:2296-2298
        let mut shape_bb = BoundingBox::new();
        for shape in shapes.iter() {
            shape_bb.merge(&get_extents_expolygons(&shape.expoly));
        }

        // Emboss.cpp:2300-2304
        let get_line_bb = |shapes: &ExPolygonsWithIds, mut j: usize| {
            let mut line_bb = BoundingBox::new();
            while j < text.len() && text[j] != '\n' {
                line_bb.merge(&get_extents_expolygons(&shapes[j].expoly));
                j += 1;
            }
            line_bb
        };

        // Emboss.cpp:2306-2324  Align x line by line
        let mut main_offset = Point::new(
            get_align_x_offset(prop.align.0, &shape_bb, &get_line_bb(shapes, 0)) as i64,
            main_y_offset as i64,
        );
        for i in 0..shapes.len() {
            let letter = text[i];
            if letter == '\n' {
                //Enter the next line of text
                offset_xy.push(main_offset); // keep offsets aligned with shapes
                main_offset.x = get_align_x_offset(prop.align.0, &shape_bb, &get_line_bb(shapes, i + 1)) as i64;
                continue;
            }
            let mut temp_offset = main_offset;
            if real_fonts[i].has_value() {
                temp_offset.y = glyph_y_offset(i) as i64;
            }
            let shape = &mut shapes[i].expoly;
            offset_xy.push(temp_offset);
            for s in shape.iter_mut() {
                s.translate(temp_offset);
            }
        }
    }
}

// Emboss.cpp:2298-2302
// double Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign align, unsigned count_lines, const FontFile &ff, const FontProp &fp)
pub fn get_align_y_offset_in_mm(
    align: crate::text_configuration::VerticalAlign,
    count_lines: u32,
    ff: &FontFile,
    fp: &crate::text_configuration::FontProp,
) -> f64 {
    // Emboss.cpp:2299-2301
    let offset_in_font_point = get_align_y_offset(align, count_lines, ff, fp);
    let scale = get_text_shape_scale(fp, ff);
    scale * (offset_in_font_point as f64)
}

// ===========================================================================
// remove_spikes (Emboss.cpp:2334-2489, `#ifdef REMOVE_SPIKES`)
// ===========================================================================
// `REMOVE_SPIKES` is not defined at the pin (Emboss.cpp:42 is commented out), so the engine does
// not compile these three; they are ported as the source reads and nothing calls them.

// Emboss.cpp:2336-2471
#[allow(dead_code)]
fn remove_spikes(polygon: &mut Polygon, spike_desc: &SpikeDesc) {
    // Emboss.cpp:2338-2352
    #[derive(Clone, Copy, PartialEq)]
    enum Type {
        Add,   // Move with point B on A-side and add new point on C-side
        Move, // Only move with point B
        Erase, // left only points A and C without move
    }
    #[derive(Clone, Copy)]
    struct SpikeHeal {
        r#type: Type,
        index: usize,
        b: Point,
        add: Point,
    }
    let mut heals: Vec<SpikeHeal> = Vec::new();

    // Emboss.cpp:2354-2356
    let count = polygon.points().len();
    if count < 3 {
        return;
    }

    // Emboss.cpp:2358-2433
    let mut ptr_a = count - 2;
    let mut ptr_b = count - 1;
    for ic in 0..count {
        let a = polygon.points()[ptr_a];
        let b = polygon.points()[ptr_b];
        let c = polygon.points()[ic];
        let b_index = ptr_b;
        // ScopeGuard sg: prepare for next loop
        ptr_a = ptr_b;
        ptr_b = ic;

        // calc sides
        let ba = a - b;
        let bc = c - b;

        let ba_f = (ba.x as f64, ba.y as f64);
        let bc_f = (bc.x as f64, bc.y as f64);
        let dot_product = ba_f.0.mul_add(bc_f.0, ba_f.1 * bc_f.1);

        // sqrt together after multiplication save one sqrt
        let ba_size_sq = ba_f.0.mul_add(ba_f.0, ba_f.1 * ba_f.1);
        let bc_size_sq = bc_f.0.mul_add(bc_f.0, bc_f.1 * bc_f.1);
        let norm = (ba_size_sq * bc_size_sq).sqrt();
        let mut cos_angle = dot_product / norm;

        // small angle are around 1 --> cos(0) = 1
        if cos_angle < spike_desc.cos_angle {
            continue;
        }

        let mut heal = SpikeHeal { r#type: Type::Erase, index: b_index, b: Point::new(0, 0), add: Point::new(0, 0) };

        // has to be in range <-1, 1>
        // Due to preccission of floating point number could be sligtly out of range
        if cos_angle > 1. {
            cos_angle = 1.;
        }
        if cos_angle < -1. {
            cos_angle = -1.;
        }

        // Current Spike angle
        let angle = cos_angle.acos();
        let wanted_size = spike_desc.half_bevel / (angle / 2.).cos();
        let wanted_size_sq = wanted_size * wanted_size;

        let is_ba_short = ba_size_sq < wanted_size_sq;
        let is_bc_short = bc_size_sq < wanted_size_sq;
        let a_side = || {
            let s = ba_size_sq.sqrt();
            let ba_norm = (ba_f.0 / s, ba_f.1 / s);
            Point::new(b.x + (wanted_size * ba_norm.0) as i64, b.y + (wanted_size * ba_norm.1) as i64)
        };
        let c_side = || {
            let s = bc_size_sq.sqrt();
            let bc_norm = (bc_f.0 / s, bc_f.1 / s);
            Point::new(b.x + (wanted_size * bc_norm.0) as i64, b.y + (wanted_size * bc_norm.1) as i64)
        };
        if is_ba_short && is_bc_short {
            // remove short spike
            heal.r#type = Type::Erase;
        } else if is_ba_short {
            // move point B on C-side
            heal.r#type = Type::Move;
            heal.b = c_side();
        } else if is_bc_short {
            // move point B on A-side
            heal.r#type = Type::Move;
            heal.b = a_side();
        } else {
            // move point B on A-side and add point on C-side
            heal.r#type = Type::Add;
            heal.b = a_side();
            heal.add = c_side();
        }
        heals.push(heal);
    }

    // Emboss.cpp:2435-2436
    if heals.is_empty() {
        return;
    }

    // Emboss.cpp:2438-2441  sort index from high to low
    if heals[0].index == (count - 1) {
        heals.rotate_left(1);
    }
    heals.reverse();

    // Emboss.cpp:2443-2454
    let mut extend: i32 = 0;
    let mut curr_extend: i32 = 0;
    for h in &heals {
        match h.r#type {
            Type::Add => {
                curr_extend += 1;
                if extend < curr_extend {
                    extend = curr_extend;
                }
            }
            Type::Erase => {
                curr_extend -= 1;
            }
            _ => {}
        }
    }

    // Emboss.cpp:2456-2458
    let pts = polygon.points_mut();
    if extend > 0 {
        pts.reserve(extend as usize);
    }

    // Emboss.cpp:2460-2470
    for h in &heals {
        match h.r#type {
            Type::Add => {
                pts[h.index] = h.b;
                pts.insert(h.index + 1, h.add);
            }
            Type::Erase => {
                pts.remove(h.index);
            }
            Type::Move => {
                pts[h.index] = h.b;
            }
        }
    }
}

mod spikes_of_polygons {
    use super::*;

    // Emboss.cpp:2473-2478
    #[allow(dead_code)]
    pub(super) fn remove_spikes(polygons: &mut Polygons, spike_desc: &SpikeDesc) {
        for polygon in polygons.iter_mut() {
            super::remove_spikes(polygon, spike_desc);
        }
        remove_bad(polygons);
    }
}

mod spikes_of_expolygons {
    use super::*;

    // Emboss.cpp:2480-2487
    #[allow(dead_code)]
    pub(super) fn remove_spikes(expolygons: &mut ExPolygons, spike_desc: &SpikeDesc) {
        for expolygon in expolygons.iter_mut() {
            super::remove_spikes(&mut expolygon.contour, spike_desc);
            spikes_of_polygons::remove_spikes(&mut expolygon.holes, spike_desc);
        }
        for_expolygons::remove_bad(expolygons);
    }
}

// ===========================================================================
// Helpers of this file
// ===========================================================================


// `BoundingBox::defined` flag — Slic3r's BoundingBox starts "undefined"; a merged
// box becomes defined. Mirror C++ `bb.defined` exactly via the crate's accessor
// rather than approximating with a min<=max comparison.
fn bb_defined(bb: &BoundingBox) -> bool {
    bb.is_defined()
}

// Point.hpp  to_points(const Polygons&) — flatten all polygons' points.
fn to_points_polygons(polygons: &Polygons) -> Points {
    let mut result: Points = Points::new();
    for p in polygons {
        result.extend_from_slice(p.points());
    }
    result
}

// transform a Vec3d point (homogeneous affine) by a Transform3d (4x4 matrix).
// `Transform3d * Vec3d` in Eigen's evaluation order.
fn transform_point(tr: &Transform3d, p: &Vec3d) -> Vec3d {
    gv(&eigen::transform_mul_vec(tr, &ev(p)))
}

// The crate's `Vec3d` of this file to the nalgebra vector of `geometry::eigen`, and back.
fn ev(v: &Vec3d) -> eigen::Vec3d {
    eigen::Vec3d::new(v.x, v.y, v.z)
}

fn gv(v: &eigen::Vec3d) -> Vec3d {
    Vec3d::new(v.x, v.y, v.z)
}

// AABBTreeLines.hpp:166-200 `build_aabb_tree_over_indexed_lines` for `Linef`: the tree of the
// crate's `aabb_tree_lines` is the one over `Line`, whose centroids are integers
// (Point.hpp:255); over `Linef` the centroid `(line.a + line.b) * 0.5` keeps its half. The lines
// are held as integer `Line`s here (a `Linef` of Emboss.cpp is the cast of integer points), the
// distances of `all_lines_in_radius` are the same doubles for both.
fn build_aabb_tree_over_indexed_linesf(lines: &[Line]) -> aabb_tree_lines::tree2d::Tree {
    struct InputType {
        m_idx: usize,
        m_bbox: aabb_tree_lines::tree2d::BoundingBox,
        m_centroid: [f64; 2],
    }
    impl aabb_tree_lines::tree2d::SourceNode for InputType {
        fn idx(&self) -> usize {
            self.m_idx
        }
        fn centroid(&self) -> [f64; 2] {
            self.m_centroid
        }
        fn bbox(&self) -> aabb_tree_lines::tree2d::BoundingBox {
            self.m_bbox
        }
    }
    let mut input: Vec<InputType> = Vec::with_capacity(lines.len());
    for (i, line) in lines.iter().enumerate() {
        let (a, b) = ([line.a.x as f64, line.a.y as f64], [line.b.x as f64, line.b.y as f64]);
        let m_centroid = [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
        let mut m_bbox = aabb_tree_lines::tree2d::BoundingBox::from_point(a);
        m_bbox.extend_point(b);
        input.push(InputType { m_idx: i, m_bbox, m_centroid });
    }
    let mut out = aabb_tree_lines::tree2d::Tree::new();
    out.build(input);
    out
}

// ExPolygon.hpp  std::vector<Linef> to_linesf(const ExPolygons&, count)
// The crate's AABB line tree operates over integer `Line`s; build the indexed line
// list (contour then holes, each closing back to its first point) matching the
// ExPolygonsIndices ordering used for `cvt_index`.
fn to_linesf(expolygons: &ExPolygons, count: u32) -> Lines {
    let mut lines: Lines = Vec::with_capacity(count as usize);
    let push_poly = |lines: &mut Lines, poly: &Polygon| {
        let pts = poly.points();
        let n = pts.len();
        // ExPolygon.hpp:161  if (pts.size() < 2) return;
        if n < 2 {
            return;
        }
        for i in 0..n {
            let a = pts[i];
            let b = pts[(i + 1) % n];
            lines.push(Line::new(a, b));
        }
    };
    for ex in expolygons {
        push_poly(&mut lines, &ex.contour);
        for hole in &ex.holes {
            push_poly(&mut lines, hole);
        }
    }
    lines
}

/// Port of the functions of `src/imgui/imstb_truetype.h` (stb_truetype, as vendored by the pin;
/// Emboss.cpp:12-13 compiles its implementation with `STB_TRUETYPE_IMPLEMENTATION`) that
/// `Emboss.cpp` calls, with the static helpers they reach: the table lookup and `stbtt_InitFont`
/// (TrueType and CFF fonts, `.ttc` collections), `stbtt_FindGlyphIndex` (cmap formats 0, 4, 6, 12
/// and 13), `stbtt_GetGlyphHMetrics`, `stbtt_GetGlyphShape` (TrueType outlines, simple and
/// compound, and the Type 2 charstring interpreter of CFF fonts), `stbtt_FlattenCurves`,
/// `stbtt_GetFontVMetrics`, `stbtt_ScaleForMappingEmToPixels`, `stbtt_GetNumberOfFonts`,
/// `stbtt_GetFontOffsetForIndex` and `stbtt_GetFontNameString`. The rasterizer, the packer, the
/// kerning tables and the SDF code are not called by Emboss.cpp and are not here.
///
/// The C reads the font through raw pointers. Here the font is a byte slice and every read goes
/// through [`tt_byte`], which gives 0 past the end of the data.
/// FIDELITY-NOTE: the C reads past the end of a truncated or corrupt font (undefined behaviour,
/// and `stbtt_InitFont` gives the CFF buffer a size of 512 MB whatever the file holds); for a
/// font whose tables lie inside the file, which is every font the C reads without faulting, the
/// two read the same bytes.
///
/// The float expressions are written as clang contracts them for the engine's build
/// (`-ffp-contract=on`): a product added inside one expression is a fused multiply-add.
#[allow(non_snake_case)]
pub mod imstb_truetype {
    pub const STBTT_VMOVE: u8 = 1;
    pub const STBTT_VLINE: u8 = 2;
    pub const STBTT_VCURVE: u8 = 3;
    pub const STBTT_VCUBIC: u8 = 4;

    pub const STBTT_PLATFORM_ID_UNICODE: i32 = 0;
    pub const STBTT_PLATFORM_ID_MAC: i32 = 1;
    pub const STBTT_PLATFORM_ID_ISO: i32 = 2;
    pub const STBTT_PLATFORM_ID_MICROSOFT: i32 = 3;
    pub const STBTT_MS_EID_SYMBOL: i32 = 0;
    pub const STBTT_MS_EID_UNICODE_BMP: i32 = 1;
    pub const STBTT_MS_EID_SHIFTJIS: i32 = 2;
    pub const STBTT_MS_EID_UNICODE_FULL: i32 = 10;
    pub const STBTT_MS_LANG_ENGLISH: i32 = 0x0409;

    /// imstb_truetype.h:822-829  `stbtt_vertex` (`stbtt_vertex_type` is `short`).
    #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
    pub struct StbttVertex {
        pub x: i16,
        pub y: i16,
        pub cx: i16,
        pub cy: i16,
        pub cx1: i16,
        pub cy1: i16,
        pub type_: u8,
        pub padding: u8,
    }

    /// imstb_truetype.h:620-623  `stbtt__point`
    #[derive(Debug, Clone, Copy, Default, PartialEq)]
    pub struct StbttPoint {
        pub x: f32,
        pub y: f32,
    }

    /// imstb_truetype.h:514-519  `stbtt__buf`: `data` is the slice from the buffer's first byte to
    /// the end of the font data (empty for the C's `NULL`); `size` may be larger than the slice.
    #[derive(Debug, Clone, Copy)]
    pub struct StbttBuf<'a> {
        pub data: &'a [u8],
        pub cursor: i32,
        pub size: i32,
    }

    /// imstb_truetype.h:714-732  `struct stbtt_fontinfo`. `data` is the whole font file;
    /// `userdata` is not used by the calls Emboss.cpp makes.
    #[derive(Debug, Clone, Copy)]
    pub struct StbttFontinfo<'a> {
        pub data: &'a [u8],
        pub fontstart: i32,
        pub num_glyphs: i32,
        pub loca: i32,
        pub head: i32,
        pub glyf: i32,
        pub hhea: i32,
        pub hmtx: i32,
        pub kern: i32,
        pub gpos: i32,
        pub index_map: i32,
        pub index_to_loc_format: i32,
        pub cff: StbttBuf<'a>,
        pub charstrings: StbttBuf<'a>,
        pub gsubrs: StbttBuf<'a>,
        pub subrs: StbttBuf<'a>,
        pub fontdicts: StbttBuf<'a>,
        pub fdselect: StbttBuf<'a>,
    }

    /// One byte of the font at `off`, 0 past the end (see the module note).
    #[inline]
    fn tt_byte(data: &[u8], off: i64) -> u8 {
        if off < 0 {
            return 0;
        }
        data.get(off as usize).copied().unwrap_or(0)
    }

    /// imstb_truetype.h:1116-1121
    pub fn stbtt__buf_get8(b: &mut StbttBuf) -> u8 {
        if b.cursor >= b.size {
            return 0;
        }
        let v = tt_byte(b.data, b.cursor as i64);
        b.cursor += 1;
        v
    }

    /// imstb_truetype.h:1123-1128
    pub fn stbtt__buf_peek8(b: &StbttBuf) -> u8 {
        if b.cursor >= b.size {
            return 0;
        }
        tt_byte(b.data, b.cursor as i64)
    }

    /// imstb_truetype.h:1130-1134
    pub fn stbtt__buf_seek(b: &mut StbttBuf, o: i32) {
        b.cursor = if o > b.size || o < 0 { b.size } else { o };
    }

    /// imstb_truetype.h:1136-1139
    pub fn stbtt__buf_skip(b: &mut StbttBuf, o: i32) {
        let c = b.cursor.wrapping_add(o);
        stbtt__buf_seek(b, c);
    }

    /// imstb_truetype.h:1141-1149
    pub fn stbtt__buf_get(b: &mut StbttBuf, n: i32) -> u32 {
        let mut v: u32 = 0;
        for _ in 0..n {
            v = (v << 8) | stbtt__buf_get8(b) as u32;
        }
        v
    }

    /// imstb_truetype.h:1151-1159  `stbtt__new_buf(p, size)`: `p` is the slice from the
    /// buffer's first byte, `size` its claimed size.
    pub fn stbtt__new_buf(p: &[u8], size: usize) -> StbttBuf<'_> {
        StbttBuf { data: p, cursor: 0, size: size as i32 }
    }

    #[inline]
    fn stbtt__buf_get16(b: &mut StbttBuf) -> u32 {
        stbtt__buf_get(b, 2)
    }
    #[inline]
    fn stbtt__buf_get32(b: &mut StbttBuf) -> u32 {
        stbtt__buf_get(b, 4)
    }

    /// imstb_truetype.h:1164-1171
    pub fn stbtt__buf_range<'a>(b: &StbttBuf<'a>, o: i32, s: i32) -> StbttBuf<'a> {
        let mut r = stbtt__new_buf(&[], 0);
        if o < 0 || s < 0 || o > b.size || s > b.size - o {
            return r;
        }
        let start = (o as usize).min(b.data.len());
        r.data = &b.data[start..];
        r.size = s;
        r
    }

    /// imstb_truetype.h:1173-1185
    pub fn stbtt__cff_get_index<'a>(b: &mut StbttBuf<'a>) -> StbttBuf<'a> {
        let start = b.cursor;
        let count = stbtt__buf_get16(b) as i32;
        if count != 0 {
            let offsize = stbtt__buf_get8(b) as i32;
            stbtt__buf_skip(b, offsize.wrapping_mul(count));
            let last = stbtt__buf_get(b, offsize);
            stbtt__buf_skip(b, last.wrapping_sub(1) as i32);
        }
        let len = b.cursor - start;
        stbtt__buf_range(b, start, len)
    }

    /// imstb_truetype.h:1187-1197
    pub fn stbtt__cff_int(b: &mut StbttBuf) -> u32 {
        let b0 = stbtt__buf_get8(b) as i32;
        if (32..=246).contains(&b0) {
            (b0 - 139) as u32
        } else if (247..=250).contains(&b0) {
            ((b0 - 247) * 256 + stbtt__buf_get8(b) as i32 + 108) as u32
        } else if (251..=254).contains(&b0) {
            (-(b0 - 251) * 256 - stbtt__buf_get8(b) as i32 - 108) as u32
        } else if b0 == 28 {
            stbtt__buf_get16(b)
        } else if b0 == 29 {
            stbtt__buf_get32(b)
        } else {
            0
        }
    }

    /// imstb_truetype.h:1199-1212
    pub fn stbtt__cff_skip_operand(b: &mut StbttBuf) {
        let b0 = stbtt__buf_peek8(b) as i32;
        if b0 == 30 {
            stbtt__buf_skip(b, 1);
            while b.cursor < b.size {
                let v = stbtt__buf_get8(b) as i32;
                if (v & 0xF) == 0xF || (v >> 4) == 0xF {
                    break;
                }
            }
        } else {
            stbtt__cff_int(b);
        }
    }

    /// imstb_truetype.h:1214-1227
    pub fn stbtt__dict_get<'a>(b: &mut StbttBuf<'a>, key: i32) -> StbttBuf<'a> {
        stbtt__buf_seek(b, 0);
        while b.cursor < b.size {
            let start = b.cursor;
            while stbtt__buf_peek8(b) >= 28 {
                stbtt__cff_skip_operand(b);
            }
            let end = b.cursor;
            let mut op = stbtt__buf_get8(b) as i32;
            if op == 12 {
                op = stbtt__buf_get8(b) as i32 | 0x100;
            }
            if op == key {
                return stbtt__buf_range(b, start, end - start);
            }
        }
        stbtt__buf_range(b, 0, 0)
    }

    /// imstb_truetype.h:1229-1235
    pub fn stbtt__dict_get_ints(b: &mut StbttBuf, key: i32, outcount: i32, out: &mut [u32]) {
        let mut operands = stbtt__dict_get(b, key);
        let mut i = 0;
        while i < outcount && operands.cursor < operands.size {
            out[i as usize] = stbtt__cff_int(&mut operands);
            i += 1;
        }
    }

    /// imstb_truetype.h:1237-1241
    pub fn stbtt__cff_index_count(b: &mut StbttBuf) -> i32 {
        stbtt__buf_seek(b, 0);
        stbtt__buf_get16(b) as i32
    }

    /// imstb_truetype.h:1243-1255
    pub fn stbtt__cff_index_get(mut b: StbttBuf, i: i32) -> StbttBuf {
        stbtt__buf_seek(&mut b, 0);
        let count = stbtt__buf_get16(&mut b) as i32;
        let offsize = stbtt__buf_get8(&mut b) as i32;
        stbtt__buf_skip(&mut b, i.wrapping_mul(offsize));
        let start = stbtt__buf_get(&mut b, offsize) as i32;
        let end = stbtt__buf_get(&mut b, offsize) as i32;
        let o = 2i32
            .wrapping_add((count + 1).wrapping_mul(offsize))
            .wrapping_add(start);
        stbtt__buf_range(&b, o, end.wrapping_sub(start))
    }

    /// imstb_truetype.h:1265  `ttBYTE(p)`
    #[inline]
    pub fn tt_byte_at(data: &[u8], off: i64) -> u8 {
        tt_byte(data, off)
    }
    /// imstb_truetype.h:1266  `ttCHAR(p)`
    #[inline]
    pub fn tt_char(data: &[u8], off: i64) -> i8 {
        tt_byte(data, off) as i8
    }
    /// imstb_truetype.h:1269  `ttUSHORT(p)`: `p[0]*256 + p[1]`
    #[inline]
    pub fn tt_ushort(data: &[u8], off: i64) -> u16 {
        ((tt_byte(data, off) as i32) * 256 + tt_byte(data, off + 1) as i32) as u16
    }
    /// imstb_truetype.h:1270  `ttSHORT(p)`
    #[inline]
    pub fn tt_short(data: &[u8], off: i64) -> i16 {
        ((tt_byte(data, off) as i32) * 256 + tt_byte(data, off + 1) as i32) as i16
    }
    /// imstb_truetype.h:1271  `ttULONG(p)`
    #[inline]
    pub fn tt_ulong(data: &[u8], off: i64) -> u32 {
        u32::from_be_bytes([
            tt_byte(data, off),
            tt_byte(data, off + 1),
            tt_byte(data, off + 2),
            tt_byte(data, off + 3),
        ])
    }
    /// imstb_truetype.h:1272  `ttLONG(p)`
    #[inline]
    pub fn tt_long(data: &[u8], off: i64) -> i32 {
        tt_ulong(data, off) as i32
    }

    /// imstb_truetype.h:1274  `stbtt_tag4(p,c0,c1,c2,c3)`
    #[inline]
    fn stbtt_tag4(data: &[u8], off: i64, c: [u8; 4]) -> bool {
        tt_byte(data, off) == c[0]
            && tt_byte(data, off + 1) == c[1]
            && tt_byte(data, off + 2) == c[2]
            && tt_byte(data, off + 3) == c[3]
    }
    /// imstb_truetype.h:1275  `stbtt_tag(p,str)`
    #[inline]
    fn stbtt_tag(data: &[u8], off: i64, s: &[u8; 4]) -> bool {
        stbtt_tag4(data, off, *s)
    }

    /// imstb_truetype.h:1277-1286
    pub fn stbtt__isfont(font: &[u8]) -> i32 {
        if stbtt_tag4(font, 0, [b'1', 0, 0, 0]) {
            return 1; // TrueType 1
        }
        if stbtt_tag(font, 0, b"typ1") {
            return 1; // TrueType with type 1 font -- we don't support this!
        }
        if stbtt_tag(font, 0, b"OTTO") {
            return 1; // OpenType with CFF
        }
        if stbtt_tag4(font, 0, [0, 1, 0, 0]) {
            return 1; // OpenType 1.0
        }
        if stbtt_tag(font, 0, b"true") {
            return 1; // Apple specification for TrueType fonts
        }
        0
    }

    /// imstb_truetype.h:1288-1300
    pub fn stbtt__find_table(data: &[u8], fontstart: u32, tag: &[u8; 4]) -> u32 {
        let num_tables = tt_ushort(data, fontstart as i64 + 4) as i32;
        let tabledir = fontstart.wrapping_add(12);
        for i in 0..num_tables {
            let loc = tabledir.wrapping_add(16 * i as u32);
            if stbtt_tag(data, loc as i64, tag) {
                return tt_ulong(data, loc as i64 + 8);
            }
        }
        0
    }

    /// imstb_truetype.h:1302-1319
    pub fn stbtt_get_font_offset_for_index_internal(font_collection: &[u8], index: i32) -> i32 {
        if stbtt__isfont(font_collection) != 0 {
            return if index == 0 { 0 } else { -1 };
        }

        if stbtt_tag(font_collection, 0, b"ttcf") {
            if tt_ulong(font_collection, 4) == 0x00010000 || tt_ulong(font_collection, 4) == 0x00020000 {
                let n = tt_long(font_collection, 8);
                if index >= n {
                    return -2;
                }
                return tt_ulong(font_collection, 12 + index as i64 * 4) as i32;
            }
        }
        -3
    }

    /// imstb_truetype.h:1321-1335
    pub fn stbtt_get_number_of_fonts_internal(font_collection: &[u8]) -> i32 {
        if stbtt__isfont(font_collection) != 0 {
            return 1;
        }

        if stbtt_tag(font_collection, 0, b"ttcf") {
            if tt_ulong(font_collection, 4) == 0x00010000 || tt_ulong(font_collection, 4) == 0x00020000 {
                return tt_long(font_collection, 8);
            }
        }
        0
    }

    /// imstb_truetype.h:1337-1348
    pub fn stbtt__get_subrs<'a>(mut cff: StbttBuf<'a>, mut fontdict: StbttBuf<'a>) -> StbttBuf<'a> {
        let mut subrsoff: [u32; 1] = [0];
        let mut private_loc: [u32; 2] = [0, 0];
        stbtt__dict_get_ints(&mut fontdict, 18, 2, &mut private_loc);
        if private_loc[1] == 0 || private_loc[0] == 0 {
            return stbtt__new_buf(&[], 0);
        }
        let mut pdict = stbtt__buf_range(&cff, private_loc[1] as i32, private_loc[0] as i32);
        stbtt__dict_get_ints(&mut pdict, 19, 1, &mut subrsoff);
        if subrsoff[0] == 0 {
            return stbtt__new_buf(&[], 0);
        }
        stbtt__buf_seek(&mut cff, private_loc[1].wrapping_add(subrsoff[0]) as i32);
        stbtt__cff_get_index(&mut cff)
    }

    /// imstb_truetype.h:1350-1468  `stbtt_InitFont_internal`; returns `None` where the C returns 0.
    pub fn stbtt_init_font_internal(data: &[u8], fontstart: i32) -> Option<StbttFontinfo<'_>> {
        let empty = stbtt__new_buf(&[], 0);
        let mut info = StbttFontinfo {
            data,
            fontstart,
            num_glyphs: 0,
            loca: 0,
            head: 0,
            glyf: 0,
            hhea: 0,
            hmtx: 0,
            kern: 0,
            gpos: 0,
            index_map: 0,
            index_to_loc_format: 0,
            cff: empty,
            charstrings: empty,
            gsubrs: empty,
            subrs: empty,
            fontdicts: empty,
            fdselect: empty,
        };
        let fs = fontstart as u32;

        let cmap = stbtt__find_table(data, fs, b"cmap"); // required
        info.loca = stbtt__find_table(data, fs, b"loca") as i32; // required
        info.head = stbtt__find_table(data, fs, b"head") as i32; // required
        info.glyf = stbtt__find_table(data, fs, b"glyf") as i32; // required
        info.hhea = stbtt__find_table(data, fs, b"hhea") as i32; // required
        info.hmtx = stbtt__find_table(data, fs, b"hmtx") as i32; // required
        info.kern = stbtt__find_table(data, fs, b"kern") as i32; // not required
        info.gpos = stbtt__find_table(data, fs, b"GPOS") as i32; // not required

        if cmap == 0 || info.head == 0 || info.hhea == 0 || info.hmtx == 0 {
            crate::boost_log_trivial!(Info, "Cannot find cmap/head/hhea/hmtx table");
            return None;
        }
        if info.glyf != 0 {
            if info.loca == 0 {
                crate::boost_log_trivial!(Info, "Cannot find loca table");
                return None;
            }
        } else {
            crate::boost_log_trivial!(Info, "initialization for CFF / Type2 fonts (OTF)");
            let mut cstype: [u32; 1] = [2];
            let mut charstrings: [u32; 1] = [0];
            let mut fdarrayoff: [u32; 1] = [0];
            let mut fdselectoff: [u32; 1] = [0];

            let cff = stbtt__find_table(data, fs, b"CFF ");
            if cff == 0 {
                crate::boost_log_trivial!(Info, "Cannot find cff table");
                return None;
            }

            info.fontdicts = stbtt__new_buf(&[], 0);
            info.fdselect = stbtt__new_buf(&[], 0);

            let cff_start = (cff as usize).min(data.len());
            info.cff = stbtt__new_buf(&data[cff_start..], 512 * 1024 * 1024);
            let mut b = info.cff;

            stbtt__buf_skip(&mut b, 2);
            let hdrsize = stbtt__buf_get8(&mut b) as i32;
            stbtt__buf_seek(&mut b, hdrsize); // hdrsize

            stbtt__cff_get_index(&mut b); // name INDEX
            let topdictidx = stbtt__cff_get_index(&mut b);
            let mut topdict = stbtt__cff_index_get(topdictidx, 0);
            stbtt__cff_get_index(&mut b); // string INDEX
            info.gsubrs = stbtt__cff_get_index(&mut b);

            stbtt__dict_get_ints(&mut topdict, 17, 1, &mut charstrings);
            stbtt__dict_get_ints(&mut topdict, 0x100 | 6, 1, &mut cstype);
            stbtt__dict_get_ints(&mut topdict, 0x100 | 36, 1, &mut fdarrayoff);
            stbtt__dict_get_ints(&mut topdict, 0x100 | 37, 1, &mut fdselectoff);
            info.subrs = stbtt__get_subrs(b, topdict);

            if cstype[0] != 2 {
                return None;
            }
            if charstrings[0] == 0 {
                return None;
            }

            if fdarrayoff[0] != 0 {
                if fdselectoff[0] == 0 {
                    return None;
                }
                stbtt__buf_seek(&mut b, fdarrayoff[0] as i32);
                info.fontdicts = stbtt__cff_get_index(&mut b);
                info.fdselect = stbtt__buf_range(&b, fdselectoff[0] as i32, b.size - fdselectoff[0] as i32);
            }

            stbtt__buf_seek(&mut b, charstrings[0] as i32);
            info.charstrings = stbtt__cff_get_index(&mut b);
        }

        let t = stbtt__find_table(data, fs, b"maxp");
        if t != 0 {
            info.num_glyphs = tt_ushort(data, t as i64 + 4) as i32;
        } else {
            info.num_glyphs = 0xffff;
        }

        let num_tables = tt_ushort(data, cmap as i64 + 2) as i32;
        info.index_map = 0;
        for i in 0..num_tables {
            let encoding_record = cmap.wrapping_add(4 + 8 * i as u32) as i64;
            match tt_ushort(data, encoding_record) as i32 {
                STBTT_PLATFORM_ID_MICROSOFT => match tt_ushort(data, encoding_record + 2) as i32 {
                    STBTT_MS_EID_UNICODE_BMP | STBTT_MS_EID_UNICODE_FULL => {
                        info.index_map = cmap.wrapping_add(tt_ulong(data, encoding_record + 4)) as i32;
                    }
                    _ => {}
                },
                STBTT_PLATFORM_ID_UNICODE => {
                    info.index_map = cmap.wrapping_add(tt_ulong(data, encoding_record + 4)) as i32;
                }
                _ => {}
            }
        }
        if info.index_map == 0 {
            return None;
        }

        info.index_to_loc_format = tt_ushort(data, info.head as i64 + 50) as i32;
        Some(info)
    }

    /// imstb_truetype.h:1470-1563
    pub fn stbtt_find_glyph_index(info: &StbttFontinfo, unicode_codepoint: i32) -> i32 {
        let data = info.data;
        let index_map = info.index_map as u32 as i64;

        let format = tt_ushort(data, index_map);
        if format == 0 {
            let bytes = tt_ushort(data, index_map + 2) as i32;
            if unicode_codepoint < bytes - 6 {
                return tt_byte_at(data, index_map + 6 + unicode_codepoint as i64) as i32;
            }
            0
        } else if format == 6 {
            let first = tt_ushort(data, index_map + 6) as u32;
            let count = tt_ushort(data, index_map + 8) as u32;
            if (unicode_codepoint as u32) >= first && (unicode_codepoint as u32) < first.wrapping_add(count) {
                return tt_ushort(
                    data,
                    index_map + 10 + ((unicode_codepoint as u32).wrapping_sub(first) as i64) * 2,
                ) as i32;
            }
            0
        } else if format == 2 {
            0
        } else if format == 4 {
            let segcount = tt_ushort(data, index_map + 6) >> 1;
            let mut search_range = tt_ushort(data, index_map + 8) >> 1;
            let mut entry_selector = tt_ushort(data, index_map + 10);
            let range_shift = tt_ushort(data, index_map + 12) >> 1;

            let end_count = (index_map as u32).wrapping_add(14);
            let mut search = end_count;

            if unicode_codepoint > 0xffff {
                return 0;
            }

            if unicode_codepoint >= tt_ushort(data, search as i64 + range_shift as i64 * 2) as i32 {
                search = search.wrapping_add(range_shift as u32 * 2);
            }

            search = search.wrapping_sub(2);
            while entry_selector != 0 {
                search_range >>= 1;
                let end = tt_ushort(data, search as i64 + search_range as i64 * 2);
                if unicode_codepoint > end as i32 {
                    search = search.wrapping_add(search_range as u32 * 2);
                }
                entry_selector -= 1;
            }
            search = search.wrapping_add(2);

            {
                let item = (search.wrapping_sub(end_count) >> 1) as u16;

                let start = tt_ushort(data, index_map + 14 + segcount as i64 * 2 + 2 + 2 * item as i64);
                if unicode_codepoint < start as i32 {
                    return 0;
                }

                let offset = tt_ushort(data, index_map + 14 + segcount as i64 * 6 + 2 + 2 * item as i64);
                if offset == 0 {
                    return (unicode_codepoint
                        + tt_short(data, index_map + 14 + segcount as i64 * 4 + 2 + 2 * item as i64) as i32)
                        as u16 as i32;
                }

                tt_ushort(
                    data,
                    offset as i64
                        + (unicode_codepoint - start as i32) as i64 * 2
                        + index_map
                        + 14
                        + segcount as i64 * 6
                        + 2
                        + 2 * item as i64,
                ) as i32
            }
        } else if format == 12 || format == 13 {
            let ngroups = tt_ulong(data, index_map + 12);
            let mut low: i32 = 0;
            let mut high: i32 = ngroups as i32;
            while low < high {
                let mid = low + ((high - low) >> 1); // rounds down, so low <= mid < high
                let start_char = tt_ulong(data, index_map + 16 + mid as i64 * 12);
                let end_char = tt_ulong(data, index_map + 16 + mid as i64 * 12 + 4);
                if (unicode_codepoint as u32) < start_char {
                    high = mid;
                } else if (unicode_codepoint as u32) > end_char {
                    low = mid + 1;
                } else {
                    let start_glyph = tt_ulong(data, index_map + 16 + mid as i64 * 12 + 8);
                    if format == 12 {
                        return start_glyph
                            .wrapping_add(unicode_codepoint as u32)
                            .wrapping_sub(start_char) as i32;
                    } else {
                        return start_glyph as i32;
                    }
                }
            }
            0 // not found
        } else {
            0
        }
    }

    /// imstb_truetype.h:1570-1577
    fn stbtt_setvertex(v: &mut StbttVertex, type_: u8, x: i32, y: i32, cx: i32, cy: i32) {
        v.type_ = type_;
        v.x = x as i16;
        v.y = y as i16;
        v.cx = cx as i16;
        v.cy = cy as i16;
    }

    /// imstb_truetype.h:1579-1597
    fn stbtt__get_glyf_offset(info: &StbttFontinfo, glyph_index: i32) -> i32 {

        if glyph_index >= info.num_glyphs {
            return -1; // glyph index out of range
        }
        if info.index_to_loc_format >= 2 {
            return -1; // unknown index->glyph map format
        }

        let g1: i32;
        let g2: i32;
        let loca = info.loca as i64;
        if info.index_to_loc_format == 0 {
            g1 = info.glyf.wrapping_add(tt_ushort(info.data, loca + glyph_index as i64 * 2) as i32 * 2);
            g2 = info.glyf.wrapping_add(tt_ushort(info.data, loca + glyph_index as i64 * 2 + 2) as i32 * 2);
        } else {
            g1 = info.glyf.wrapping_add(tt_ulong(info.data, loca + glyph_index as i64 * 4) as i32);
            g2 = info.glyf.wrapping_add(tt_ulong(info.data, loca + glyph_index as i64 * 4 + 4) as i32);
        }

        if g1 == g2 {
            -1
        } else {
            g1
        } // if length is 0, return -1
    }

    /// imstb_truetype.h:1634-1648
    #[allow(clippy::too_many_arguments)]
    fn stbtt__close_shape(
        vertices: &mut [StbttVertex],
        mut num_vertices: i32,
        was_off: i32,
        start_off: i32,
        sx: i32,
        sy: i32,
        scx: i32,
        scy: i32,
        cx: i32,
        cy: i32,
    ) -> i32 {
        if start_off != 0 {
            if was_off != 0 {
                stbtt_setvertex(&mut vertices[num_vertices as usize], STBTT_VCURVE, (cx + scx) >> 1, (cy + scy) >> 1, cx, cy);
                num_vertices += 1;
            }
            stbtt_setvertex(&mut vertices[num_vertices as usize], STBTT_VCURVE, sx, sy, scx, scy);
            num_vertices += 1;
        } else if was_off != 0 {
            stbtt_setvertex(&mut vertices[num_vertices as usize], STBTT_VCURVE, sx, sy, cx, cy);
            num_vertices += 1;
        } else {
            stbtt_setvertex(&mut vertices[num_vertices as usize], STBTT_VLINE, sx, sy, 0, 0);
            num_vertices += 1;
        }
        num_vertices
    }

    /// imstb_truetype.h:1650-1874  `stbtt__GetGlyphShapeTT`: the vertices, empty where the C
    /// returns 0 vertices.
    ///
    /// FIDELITY-NOTE: the C allocates the array with `malloc` and leaves `cx1`/`cy1` (and, for
    /// the uninterpreted slots, `cx`/`cy`) unset; they are 0 here. A TrueType outline has no cubic
    /// segment, so nothing reads them. When a contour's last point starts a contour the C reads
    /// one element past the array (`vertices[off+i+1]`); here that read gives a zeroed vertex.
    fn stbtt__get_glyph_shape_tt(info: &StbttFontinfo, glyph_index: i32) -> Vec<StbttVertex> {
        let data = info.data;
        let mut vertices: Vec<StbttVertex> = Vec::new();
        let mut num_vertices: i32 = 0;
        let g = stbtt__get_glyf_offset(info, glyph_index);

        if g < 0 {
            return Vec::new();
        }
        let g = g as i64;

        let number_of_contours = tt_short(data, g);

        if number_of_contours > 0 {
            let mut flags: u8 = 0;
            let mut flagcount: u8;
            let mut j: i32 = 0;
            let mut was_off: i32 = 0;
            let mut start_off: i32 = 0;
            let end_pts_of_contours = g + 10;
            let ins = tt_ushort(data, g + 10 + number_of_contours as i64 * 2) as i64;
            let mut points = g + 10 + number_of_contours as i64 * 2 + 2 + ins;

            let n = 1 + tt_ushort(data, end_pts_of_contours + number_of_contours as i64 * 2 - 2) as i32;

            let m = n + 2 * number_of_contours as i32; // a loose bound on how many vertices we might need
            vertices = vec![StbttVertex::default(); m as usize];

            let mut next_move: i32 = 0;
            flagcount = 0;

            let off = (m - n) as usize; // starting offset for uninterpreted data, regardless of how m ends up being calculated

            for i in 0..n as usize {
                if flagcount == 0 {
                    flags = tt_byte_at(data, points);
                    points += 1;
                    if flags & 8 != 0 {
                        flagcount = tt_byte_at(data, points);
                        points += 1;
                    }
                } else {
                    flagcount -= 1;
                }
                vertices[off + i].type_ = flags;
            }

            let mut x: i32 = 0;
            for i in 0..n as usize {
                flags = vertices[off + i].type_;
                if flags & 2 != 0 {
                    let dx = tt_byte_at(data, points) as i16 as i32;
                    points += 1;
                    x += if flags & 16 != 0 { dx } else { -dx }; // ???
                } else if flags & 16 == 0 {
                    x += ((tt_byte_at(data, points) as i32) * 256 + tt_byte_at(data, points + 1) as i32) as i16 as i32;
                    points += 2;
                }
                vertices[off + i].x = x as i16;
            }

            let mut y: i32 = 0;
            for i in 0..n as usize {
                flags = vertices[off + i].type_;
                if flags & 4 != 0 {
                    let dy = tt_byte_at(data, points) as i16 as i32;
                    points += 1;
                    y += if flags & 32 != 0 { dy } else { -dy }; // ???
                } else if flags & 32 == 0 {
                    y += ((tt_byte_at(data, points) as i32) * 256 + tt_byte_at(data, points + 1) as i32) as i16 as i32;
                    points += 2;
                }
                vertices[off + i].y = y as i16;
            }

            num_vertices = 0;
            let (mut sx, mut sy, mut cx, mut cy, mut scx, mut scy) = (0i32, 0i32, 0i32, 0i32, 0i32, 0i32);
            let at = |vertices: &Vec<StbttVertex>, k: usize| -> StbttVertex {
                vertices.get(k).copied().unwrap_or_default()
            };
            let mut i: i32 = 0;
            while i < n {
                let iu = i as usize;
                flags = vertices[off + iu].type_;
                x = vertices[off + iu].x as i32;
                y = vertices[off + iu].y as i32;

                if next_move == i {
                    if i != 0 {
                        num_vertices = stbtt__close_shape(&mut vertices, num_vertices, was_off, start_off, sx, sy, scx, scy, cx, cy);
                    }

                    start_off = (flags & 1 == 0) as i32;
                    if start_off != 0 {
                        scx = x;
                        scy = y;
                        let nv = at(&vertices, off + iu + 1);
                        if nv.type_ & 1 == 0 {
                            sx = (x + nv.x as i32) >> 1;
                            sy = (y + nv.y as i32) >> 1;
                        } else {
                            sx = nv.x as i32;
                            sy = nv.y as i32;
                            i += 1; // we're using point i+1 as the starting point, so skip it
                        }
                    } else {
                        sx = x;
                        sy = y;
                    }
                    stbtt_setvertex(&mut vertices[num_vertices as usize], STBTT_VMOVE, sx, sy, 0, 0);
                    num_vertices += 1;
                    was_off = 0;
                    next_move = 1 + tt_ushort(data, end_pts_of_contours + j as i64 * 2) as i32;
                    j += 1;
                } else if flags & 1 == 0 {
                    if was_off != 0 {
                        stbtt_setvertex(&mut vertices[num_vertices as usize], STBTT_VCURVE, (cx + x) >> 1, (cy + y) >> 1, cx, cy);
                        num_vertices += 1;
                    }
                    cx = x;
                    cy = y;
                    was_off = 1;
                } else {
                    if was_off != 0 {
                        stbtt_setvertex(&mut vertices[num_vertices as usize], STBTT_VCURVE, x, y, cx, cy);
                    } else {
                        stbtt_setvertex(&mut vertices[num_vertices as usize], STBTT_VLINE, x, y, 0, 0);
                    }
                    num_vertices += 1;
                    was_off = 0;
                }
                i += 1;
            }
            num_vertices = stbtt__close_shape(&mut vertices, num_vertices, was_off, start_off, sx, sy, scx, scy, cx, cy);
        } else if number_of_contours == -1 {
            let mut more = 1;
            let mut comp = g + 10;
            num_vertices = 0;
            vertices = Vec::new();
            while more != 0 {
                let mut mtx: [f32; 6] = [1.0, 0.0, 0.0, 1.0, 0.0, 0.0];

                let flags = tt_short(data, comp) as u16;
                comp += 2;
                let gidx = tt_short(data, comp) as u16;
                comp += 2;

                if flags & 2 != 0 {
                    if flags & 1 != 0 {
                        mtx[4] = tt_short(data, comp) as f32;
                        comp += 2;
                        mtx[5] = tt_short(data, comp) as f32;
                        comp += 2;
                    } else {
                        mtx[4] = tt_char(data, comp) as f32;
                        comp += 1;
                        mtx[5] = tt_char(data, comp) as f32;
                        comp += 1;
                    }
                } else {
                }
                if flags & (1 << 3) != 0 {
                    mtx[0] = tt_short(data, comp) as f32 / 16384.0f32;
                    mtx[3] = mtx[0];
                    comp += 2;
                    mtx[1] = 0.0;
                    mtx[2] = 0.0;
                } else if flags & (1 << 6) != 0 {
                    mtx[0] = tt_short(data, comp) as f32 / 16384.0f32;
                    comp += 2;
                    mtx[1] = 0.0;
                    mtx[2] = 0.0;
                    mtx[3] = tt_short(data, comp) as f32 / 16384.0f32;
                    comp += 2;
                } else if flags & (1 << 7) != 0 {
                    mtx[0] = tt_short(data, comp) as f32 / 16384.0f32;
                    comp += 2;
                    mtx[1] = tt_short(data, comp) as f32 / 16384.0f32;
                    comp += 2;
                    mtx[2] = tt_short(data, comp) as f32 / 16384.0f32;
                    comp += 2;
                    mtx[3] = tt_short(data, comp) as f32 / 16384.0f32;
                    comp += 2;
                }

                let m = (mtx[0].mul_add(mtx[0], mtx[1] * mtx[1]) as f64).sqrt() as f32;
                let n = (mtx[2].mul_add(mtx[2], mtx[3] * mtx[3]) as f64).sqrt() as f32;

                let mut comp_verts = stbtt_get_glyph_shape(info, gidx as i32);
                let comp_num_verts = comp_verts.len() as i32;
                if comp_num_verts > 0 {
                    for v in comp_verts.iter_mut() {
                        let mut x = v.x as f32;
                        let mut y = v.y as f32;
                        v.x = (m * (mtx[0].mul_add(x, mtx[2] * y) + mtx[4])) as i16;
                        v.y = (n * (mtx[1].mul_add(x, mtx[3] * y) + mtx[5])) as i16;
                        x = v.cx as f32;
                        y = v.cy as f32;
                        v.cx = (m * (mtx[0].mul_add(x, mtx[2] * y) + mtx[4])) as i16;
                        v.cy = (n * (mtx[1].mul_add(x, mtx[3] * y) + mtx[5])) as i16;
                    }
                    vertices.truncate(num_vertices as usize);
                    vertices.extend_from_slice(&comp_verts);
                    num_vertices += comp_num_verts;
                }
                more = flags & (1 << 5);
            }
        } else if number_of_contours < 0 {
        } else {
        }

        vertices.truncate(num_vertices.max(0) as usize);
        vertices
    }

    /// imstb_truetype.h:1876-1888  `stbtt__csctx`; `pvertices` is `None` in the counting pass
    /// (`bounds` set).
    struct StbttCsctx {
        bounds: i32,
        started: i32,
        first_x: f32,
        first_y: f32,
        x: f32,
        y: f32,
        min_x: i32,
        max_x: i32,
        min_y: i32,
        max_y: i32,
        pvertices: Option<Vec<StbttVertex>>,
        num_vertices: i32,
    }

    impl StbttCsctx {
        /// imstb_truetype.h:1888  `STBTT__CSCTX_INIT(bounds)`
        fn init(bounds: i32) -> Self {
            StbttCsctx {
                bounds,
                started: 0,
                first_x: 0.0,
                first_y: 0.0,
                x: 0.0,
                y: 0.0,
                min_x: 0,
                max_x: 0,
                min_y: 0,
                max_y: 0,
                pvertices: None,
                num_vertices: 0,
            }
        }
    }

    /// imstb_truetype.h:1890-1897
    fn stbtt__track_vertex(c: &mut StbttCsctx, x: i32, y: i32) {
        if x > c.max_x || c.started == 0 {
            c.max_x = x;
        }
        if y > c.max_y || c.started == 0 {
            c.max_y = y;
        }
        if x < c.min_x || c.started == 0 {
            c.min_x = x;
        }
        if y < c.min_y || c.started == 0 {
            c.min_y = y;
        }
        c.started = 1;
    }

    /// imstb_truetype.h:1899-1913
    #[allow(clippy::too_many_arguments)]
    fn stbtt__csctx_v(c: &mut StbttCsctx, type_: u8, x: i32, y: i32, cx: i32, cy: i32, cx1: i32, cy1: i32) {
        if c.bounds != 0 {
            stbtt__track_vertex(c, x, y);
            if type_ == STBTT_VCUBIC {
                stbtt__track_vertex(c, cx, cy);
                stbtt__track_vertex(c, cx1, cy1);
            }
        } else if let Some(pv) = c.pvertices.as_mut() {
            let k = c.num_vertices as usize;
            stbtt_setvertex(&mut pv[k], type_, x, y, cx, cy);
            pv[k].cx1 = cx1 as i16;
            pv[k].cy1 = cy1 as i16;
        }
        c.num_vertices += 1;
    }

    /// imstb_truetype.h:1915-1919
    fn stbtt__csctx_close_shape(ctx: &mut StbttCsctx) {
        if ctx.first_x != ctx.x || ctx.first_y != ctx.y {
            let (fx, fy) = (ctx.first_x as i32, ctx.first_y as i32);
            stbtt__csctx_v(ctx, STBTT_VLINE, fx, fy, 0, 0, 0, 0);
        }
    }

    /// imstb_truetype.h:1921-1927
    fn stbtt__csctx_rmove_to(ctx: &mut StbttCsctx, dx: f32, dy: f32) {
        stbtt__csctx_close_shape(ctx);
        ctx.x += dx;
        ctx.first_x = ctx.x;
        ctx.y += dy;
        ctx.first_y = ctx.y;
        let (x, y) = (ctx.x as i32, ctx.y as i32);
        stbtt__csctx_v(ctx, STBTT_VMOVE, x, y, 0, 0, 0, 0);
    }

    /// imstb_truetype.h:1929-1934
    fn stbtt__csctx_rline_to(ctx: &mut StbttCsctx, dx: f32, dy: f32) {
        ctx.x += dx;
        ctx.y += dy;
        let (x, y) = (ctx.x as i32, ctx.y as i32);
        stbtt__csctx_v(ctx, STBTT_VLINE, x, y, 0, 0, 0, 0);
    }

    /// imstb_truetype.h:1936-1945
    #[allow(clippy::too_many_arguments)]
    fn stbtt__csctx_rccurve_to(ctx: &mut StbttCsctx, dx1: f32, dy1: f32, dx2: f32, dy2: f32, dx3: f32, dy3: f32) {
        let cx1 = ctx.x + dx1;
        let cy1 = ctx.y + dy1;
        let cx2 = cx1 + dx2;
        let cy2 = cy1 + dy2;
        ctx.x = cx2 + dx3;
        ctx.y = cy2 + dy3;
        let (x, y) = (ctx.x as i32, ctx.y as i32);
        stbtt__csctx_v(ctx, STBTT_VCUBIC, x, y, cx1 as i32, cy1 as i32, cx2 as i32, cy2 as i32);
    }

    /// imstb_truetype.h:1947-1959
    fn stbtt__get_subr(mut idx: StbttBuf, mut n: i32) -> StbttBuf {
        let count = stbtt__cff_index_count(&mut idx);
        let mut bias = 107;
        if count >= 33900 {
            bias = 32768;
        } else if count >= 1240 {
            bias = 1131;
        }
        n += bias;
        if n < 0 || n >= count {
            return stbtt__new_buf(&[], 0);
        }
        stbtt__cff_index_get(idx, n)
    }

    /// imstb_truetype.h:1961-1987
    fn stbtt__cid_get_glyph_subrs<'a>(info: &StbttFontinfo<'a>, glyph_index: i32) -> StbttBuf<'a> {
        let mut fdselect = info.fdselect;
        let mut fdselector: i32 = -1;

        stbtt__buf_seek(&mut fdselect, 0);
        let fmt = stbtt__buf_get8(&mut fdselect) as i32;
        if fmt == 0 {
            stbtt__buf_skip(&mut fdselect, glyph_index);
            fdselector = stbtt__buf_get8(&mut fdselect) as i32;
        } else if fmt == 3 {
            let nranges = stbtt__buf_get16(&mut fdselect) as i32;
            let mut start = stbtt__buf_get16(&mut fdselect) as i32;
            for _ in 0..nranges {
                let v = stbtt__buf_get8(&mut fdselect) as i32;
                let end = stbtt__buf_get16(&mut fdselect) as i32;
                if glyph_index >= start && glyph_index < end {
                    fdselector = v;
                    break;
                }
                start = end;
            }
        }
        stbtt__get_subrs(info.cff, stbtt__cff_index_get(info.fontdicts, fdselector))
    }

    /// imstb_truetype.h:1989-2246  `stbtt__run_charstring`; returns 1 on success, 0 where the C's
    /// `STBTT__CSERR` returns 0.
    fn stbtt__run_charstring(info: &StbttFontinfo, glyph_index: i32, c: &mut StbttCsctx) -> i32 {
        let mut in_header = 1;
        let mut maskbits: i32 = 0;
        let mut subr_stack_height: usize = 0;
        let mut sp: usize = 0;
        let mut has_subrs = 0;
        let mut s: [f32; 48] = [0.0; 48];
        let empty = stbtt__new_buf(&[], 0);
        let mut subr_stack: [StbttBuf; 10] = [empty; 10];
        let mut subrs = info.subrs;
        let mut f: f32;

        let mut b = stbtt__cff_index_get(info.charstrings, glyph_index);
        while b.cursor < b.size {
            let mut i: usize = 0;
            let mut clear_stack = 1;
            let b0 = stbtt__buf_get8(&mut b) as i32;
            match b0 {
                0x13 | 0x14 => {
                    if in_header != 0 {
                        maskbits += (sp / 2) as i32; // implicit "vstem"
                    }
                    in_header = 0;
                    stbtt__buf_skip(&mut b, (maskbits + 7) / 8);
                }

                0x01 | 0x03 | 0x12 | 0x17 => {
                    maskbits += (sp / 2) as i32;
                }

                0x15 => {
                    in_header = 0;
                    if sp < 2 {
                        return 0; // STBTT__CSERR("rmoveto stack")
                    }
                    stbtt__csctx_rmove_to(c, s[sp - 2], s[sp - 1]);
                }
                0x04 => {
                    in_header = 0;
                    if sp < 1 {
                        return 0; // STBTT__CSERR("vmoveto stack")
                    }
                    stbtt__csctx_rmove_to(c, 0.0, s[sp - 1]);
                }
                0x16 => {
                    in_header = 0;
                    if sp < 1 {
                        return 0; // STBTT__CSERR("hmoveto stack")
                    }
                    stbtt__csctx_rmove_to(c, s[sp - 1], 0.0);
                }

                0x05 => {
                    if sp < 2 {
                        return 0; // STBTT__CSERR("rlineto stack")
                    }
                    while i + 1 < sp {
                        stbtt__csctx_rline_to(c, s[i], s[i + 1]);
                        i += 2;
                    }
                }

                0x07 | 0x06 => {
                    if sp < 1 {
                        return 0; // STBTT__CSERR("vlineto stack") / ("hlineto stack")
                    }
                    let mut vertical = b0 == 0x07;
                    loop {
                        if !vertical {
                            if i >= sp {
                                break;
                            }
                            stbtt__csctx_rline_to(c, s[i], 0.0);
                            i += 1;
                        }
                        vertical = false;
                        if i >= sp {
                            break;
                        }
                        stbtt__csctx_rline_to(c, 0.0, s[i]);
                        i += 1;
                    }
                }

                0x1F | 0x1E => {
                    if sp < 4 {
                        return 0; // STBTT__CSERR("hvcurveto stack") / ("vhcurveto stack")
                    }
                    let mut hv = b0 == 0x1F;
                    loop {
                        if !hv {
                            if i + 3 >= sp {
                                break;
                            }
                            stbtt__csctx_rccurve_to(
                                c,
                                0.0,
                                s[i],
                                s[i + 1],
                                s[i + 2],
                                s[i + 3],
                                if sp - i == 5 { s[i + 4] } else { 0.0 },
                            );
                            i += 4;
                        }
                        hv = false;
                        if i + 3 >= sp {
                            break;
                        }
                        stbtt__csctx_rccurve_to(
                            c,
                            s[i],
                            0.0,
                            s[i + 1],
                            s[i + 2],
                            if sp - i == 5 { s[i + 4] } else { 0.0 },
                            s[i + 3],
                        );
                        i += 4;
                    }
                }

                0x08 => {
                    if sp < 6 {
                        return 0; // STBTT__CSERR("rcurveline stack")
                    }
                    while i + 5 < sp {
                        stbtt__csctx_rccurve_to(c, s[i], s[i + 1], s[i + 2], s[i + 3], s[i + 4], s[i + 5]);
                        i += 6;
                    }
                }

                0x18 => {
                    if sp < 8 {
                        return 0; // STBTT__CSERR("rcurveline stack")
                    }
                    while i + 5 < sp - 2 {
                        stbtt__csctx_rccurve_to(c, s[i], s[i + 1], s[i + 2], s[i + 3], s[i + 4], s[i + 5]);
                        i += 6;
                    }
                    if i + 1 >= sp {
                        return 0; // STBTT__CSERR("rcurveline stack")
                    }
                    stbtt__csctx_rline_to(c, s[i], s[i + 1]);
                }

                0x19 => {
                    if sp < 8 {
                        return 0; // STBTT__CSERR("rlinecurve stack")
                    }
                    while i + 1 < sp - 6 {
                        stbtt__csctx_rline_to(c, s[i], s[i + 1]);
                        i += 2;
                    }
                    if i + 5 >= sp {
                        return 0; // STBTT__CSERR("rlinecurve stack")
                    }
                    stbtt__csctx_rccurve_to(c, s[i], s[i + 1], s[i + 2], s[i + 3], s[i + 4], s[i + 5]);
                }

                0x1A | 0x1B => {
                    if sp < 4 {
                        return 0; // STBTT__CSERR("(vv|hh)curveto stack")
                    }
                    f = 0.0;
                    if sp & 1 != 0 {
                        f = s[i];
                        i += 1;
                    }
                    while i + 3 < sp {
                        if b0 == 0x1B {
                            stbtt__csctx_rccurve_to(c, s[i], f, s[i + 1], s[i + 2], s[i + 3], 0.0);
                        } else {
                            stbtt__csctx_rccurve_to(c, f, s[i], s[i + 1], s[i + 2], 0.0, s[i + 3]);
                        }
                        f = 0.0;
                        i += 4;
                    }
                }

                0x0A | 0x1D => {
                    if b0 == 0x0A {
                        if has_subrs == 0 {
                            if info.fdselect.size != 0 {
                                subrs = stbtt__cid_get_glyph_subrs(info, glyph_index);
                            }
                            has_subrs = 1;
                        }
                    }
                    if sp < 1 {
                        return 0; // STBTT__CSERR("call(g|)subr stack")
                    }
                    sp -= 1;
                    let v = s[sp] as i32;
                    if subr_stack_height >= 10 {
                        return 0; // STBTT__CSERR("recursion limit")
                    }
                    subr_stack[subr_stack_height] = b;
                    subr_stack_height += 1;
                    b = stbtt__get_subr(if b0 == 0x0A { subrs } else { info.gsubrs }, v);
                    if b.size == 0 {
                        return 0; // STBTT__CSERR("subr not found")
                    }
                    b.cursor = 0;
                    clear_stack = 0;
                }

                0x0B => {
                    if subr_stack_height == 0 {
                        return 0; // STBTT__CSERR("return outside subr")
                    }
                    subr_stack_height -= 1;
                    b = subr_stack[subr_stack_height];
                    clear_stack = 0;
                }

                0x0E => {
                    stbtt__csctx_close_shape(c);
                    return 1;
                }

                0x0C => {
                    let b1 = stbtt__buf_get8(&mut b) as i32;
                    match b1 {
                        0x22 => {
                            if sp < 7 {
                                return 0; // STBTT__CSERR("hflex stack")
                            }
                            let dx1 = s[0];
                            let dx2 = s[1];
                            let dy2 = s[2];
                            let dx3 = s[3];
                            let dx4 = s[4];
                            let dx5 = s[5];
                            let dx6 = s[6];
                            stbtt__csctx_rccurve_to(c, dx1, 0.0, dx2, dy2, dx3, 0.0);
                            stbtt__csctx_rccurve_to(c, dx4, 0.0, dx5, -dy2, dx6, 0.0);
                        }

                        0x23 => {
                            if sp < 13 {
                                return 0; // STBTT__CSERR("flex stack")
                            }
                            let dx1 = s[0];
                            let dy1 = s[1];
                            let dx2 = s[2];
                            let dy2 = s[3];
                            let dx3 = s[4];
                            let dy3 = s[5];
                            let dx4 = s[6];
                            let dy4 = s[7];
                            let dx5 = s[8];
                            let dy5 = s[9];
                            let dx6 = s[10];
                            let dy6 = s[11];
                            stbtt__csctx_rccurve_to(c, dx1, dy1, dx2, dy2, dx3, dy3);
                            stbtt__csctx_rccurve_to(c, dx4, dy4, dx5, dy5, dx6, dy6);
                        }

                        0x24 => {
                            if sp < 9 {
                                return 0; // STBTT__CSERR("hflex1 stack")
                            }
                            let dx1 = s[0];
                            let dy1 = s[1];
                            let dx2 = s[2];
                            let dy2 = s[3];
                            let dx3 = s[4];
                            let dx4 = s[5];
                            let dx5 = s[6];
                            let dy5 = s[7];
                            let dx6 = s[8];
                            stbtt__csctx_rccurve_to(c, dx1, dy1, dx2, dy2, dx3, 0.0);
                            stbtt__csctx_rccurve_to(c, dx4, 0.0, dx5, dy5, dx6, -(dy1 + dy2 + dy5));
                        }

                        0x25 => {
                            if sp < 11 {
                                return 0; // STBTT__CSERR("flex1 stack")
                            }
                            let dx1 = s[0];
                            let dy1 = s[1];
                            let dx2 = s[2];
                            let dy2 = s[3];
                            let dx3 = s[4];
                            let dy3 = s[5];
                            let dx4 = s[6];
                            let dy4 = s[7];
                            let dx5 = s[8];
                            let dy5 = s[9];
                            let mut dx6 = s[10];
                            let mut dy6 = s[10];
                            let dx = dx1 + dx2 + dx3 + dx4 + dx5;
                            let dy = dy1 + dy2 + dy3 + dy4 + dy5;
                            if dx.abs() > dy.abs() {
                                dy6 = -dy;
                            } else {
                                dx6 = -dx;
                            }
                            stbtt__csctx_rccurve_to(c, dx1, dy1, dx2, dy2, dx3, dy3);
                            stbtt__csctx_rccurve_to(c, dx4, dy4, dx5, dy5, dx6, dy6);
                        }

                        _ => {
                            return 0; // STBTT__CSERR("unimplemented")
                        }
                    }
                }

                _ => {
                    if b0 != 255 && b0 != 28 && !(32..=254).contains(&b0) {
                        return 0; // STBTT__CSERR("reserved operator")
                    }

                    if b0 == 255 {
                        f = (stbtt__buf_get32(&mut b) as i32) as f32 / 0x10000 as f32;
                    } else {
                        stbtt__buf_skip(&mut b, -1);
                        f = (stbtt__cff_int(&mut b) as i16) as f32;
                    }
                    if sp >= 48 {
                        return 0; // STBTT__CSERR("push stack overflow")
                    }
                    s[sp] = f;
                    sp += 1;
                    clear_stack = 0;
                }
            }
            if clear_stack != 0 {
                sp = 0;
            }
        }
        0 // STBTT__CSERR("no endchar")
    }

    /// imstb_truetype.h:2248-2263
    fn stbtt__get_glyph_shape_t2(info: &StbttFontinfo, glyph_index: i32) -> Vec<StbttVertex> {
        let mut count_ctx = StbttCsctx::init(1);
        let mut output_ctx = StbttCsctx::init(0);
        if stbtt__run_charstring(info, glyph_index, &mut count_ctx) != 0 {
            output_ctx.pvertices = Some(vec![StbttVertex::default(); count_ctx.num_vertices.max(0) as usize]);
            if stbtt__run_charstring(info, glyph_index, &mut output_ctx) != 0 {
                let mut v = output_ctx.pvertices.take().unwrap_or_default();
                v.truncate(output_ctx.num_vertices.max(0) as usize);
                return v;
            }
        }
        Vec::new()
    }

    /// imstb_truetype.h:2276-2282  `stbtt_GetGlyphShape`: the vertices (their count is the C's
    /// return value).
    pub fn stbtt_get_glyph_shape(info: &StbttFontinfo, glyph_index: i32) -> Vec<StbttVertex> {
        if info.cff.size == 0 {
            stbtt__get_glyph_shape_tt(info, glyph_index)
        } else {
            stbtt__get_glyph_shape_t2(info, glyph_index)
        }
    }

    /// imstb_truetype.h:2284-2294  `stbtt_GetGlyphHMetrics`: `(advanceWidth, leftSideBearing)`.
    pub fn stbtt_get_glyph_h_metrics(info: &StbttFontinfo, glyph_index: i32) -> (i32, i32) {
        let num_of_long_hor_metrics = tt_ushort(info.data, info.hhea as i64 + 34) as i32;
        let hmtx = info.hmtx as i64;
        if glyph_index < num_of_long_hor_metrics {
            (
                tt_short(info.data, hmtx + 4 * glyph_index as i64) as i32,
                tt_short(info.data, hmtx + 4 * glyph_index as i64 + 2) as i32,
            )
        } else {
            (
                tt_short(info.data, hmtx + 4 * (num_of_long_hor_metrics as i64 - 1)) as i32,
                tt_short(
                    info.data,
                    hmtx + 4 * num_of_long_hor_metrics as i64 + 2 * (glyph_index - num_of_long_hor_metrics) as i64,
                ) as i32,
            )
        }
    }

    /// imstb_truetype.h:2592-2597  `stbtt_GetFontVMetrics`: `(ascent, descent, lineGap)`.
    pub fn stbtt_get_font_v_metrics(info: &StbttFontinfo) -> (i32, i32, i32) {
        let hhea = info.hhea as i64;
        (
            tt_short(info.data, hhea + 4) as i32,
            tt_short(info.data, hhea + 6) as i32,
            tt_short(info.data, hhea + 8) as i32,
        )
    }

    /// imstb_truetype.h:2624-2628
    pub fn stbtt_scale_for_mapping_em_to_pixels(info: &StbttFontinfo, pixels: f32) -> f32 {
        let units_per_em = tt_ushort(info.data, info.head as i64 + 18) as i32;
        pixels / units_per_em as f32
    }

    /// imstb_truetype.h:3398-3403  (`points` is `None` in the counting pass)
    fn stbtt__add_point(points: &mut Option<Vec<StbttPoint>>, n: i32, x: f32, y: f32) {
        let Some(points) = points.as_mut() else {
            return; // during first pass, it's unallocated
        };
        points[n as usize].x = x;
        points[n as usize].y = y;
    }

    /// imstb_truetype.h:3405-3424  tessellate until threshold p is happy...
    #[allow(clippy::too_many_arguments)]
    fn stbtt__tesselate_curve(
        points: &mut Option<Vec<StbttPoint>>,
        num_points: &mut i32,
        x0: f32,
        y0: f32,
        x1: f32,
        y1: f32,
        x2: f32,
        y2: f32,
        objspace_flatness_squared: f32,
        n: i32,
    ) -> i32 {
        let mx = (2.0f32.mul_add(x1, x0) + x2) / 4.0;
        let my = (2.0f32.mul_add(y1, y0) + y2) / 4.0;
        let dx = (x0 + x2) / 2.0 - mx;
        let dy = (y0 + y2) / 2.0 - my;
        if n > 16 {
            return 1;
        }
        if dx.mul_add(dx, dy * dy) > objspace_flatness_squared {
            stbtt__tesselate_curve(points, num_points, x0, y0, (x0 + x1) / 2.0f32, (y0 + y1) / 2.0f32, mx, my, objspace_flatness_squared, n + 1);
            stbtt__tesselate_curve(points, num_points, mx, my, (x1 + x2) / 2.0f32, (y1 + y2) / 2.0f32, x2, y2, objspace_flatness_squared, n + 1);
        } else {
            stbtt__add_point(points, *num_points, x2, y2);
            *num_points += 1;
        }
        1
    }

    /// imstb_truetype.h:3426-3466
    #[allow(clippy::too_many_arguments)]
    fn stbtt__tesselate_cubic(
        points: &mut Option<Vec<StbttPoint>>,
        num_points: &mut i32,
        x0: f32,
        y0: f32,
        x1: f32,
        y1: f32,
        x2: f32,
        y2: f32,
        x3: f32,
        y3: f32,
        objspace_flatness_squared: f32,
        n: i32,
    ) {
        let dx0 = x1 - x0;
        let dy0 = y1 - y0;
        let dx1 = x2 - x1;
        let dy1 = y2 - y1;
        let dx2 = x3 - x2;
        let dy2 = y3 - y2;
        let dx = x3 - x0;
        let dy = y3 - y0;
        let longlen = ((dx0.mul_add(dx0, dy0 * dy0) as f64).sqrt()
            + (dx1.mul_add(dx1, dy1 * dy1) as f64).sqrt()
            + (dx2.mul_add(dx2, dy2 * dy2) as f64).sqrt()) as f32;
        let shortlen = (dx.mul_add(dx, dy * dy) as f64).sqrt() as f32;
        let flatness_squared = longlen.mul_add(longlen, -(shortlen * shortlen));

        if n > 16 {
            return;
        }

        if flatness_squared > objspace_flatness_squared {
            let x01 = (x0 + x1) / 2.0;
            let y01 = (y0 + y1) / 2.0;
            let x12 = (x1 + x2) / 2.0;
            let y12 = (y1 + y2) / 2.0;
            let x23 = (x2 + x3) / 2.0;
            let y23 = (y2 + y3) / 2.0;

            let xa = (x01 + x12) / 2.0;
            let ya = (y01 + y12) / 2.0;
            let xb = (x12 + x23) / 2.0;
            let yb = (y12 + y23) / 2.0;

            let mx = (xa + xb) / 2.0;
            let my = (ya + yb) / 2.0;

            stbtt__tesselate_cubic(points, num_points, x0, y0, x01, y01, xa, ya, mx, my, objspace_flatness_squared, n + 1);
            stbtt__tesselate_cubic(points, num_points, mx, my, xb, yb, x23, y23, x3, y3, objspace_flatness_squared, n + 1);
        } else {
            stbtt__add_point(points, *num_points, x3, y3);
            *num_points += 1;
        }
    }

    /// imstb_truetype.h:3468-3544  `stbtt_FlattenCurves`: `(points, contour_lengths)`, `None`
    /// where the C returns NULL (no contour). `num_contours` is `contour_lengths.len()`.
    pub fn stbtt_flatten_curves(vertices: &[StbttVertex], objspace_flatness: f32) -> Option<(Vec<StbttPoint>, Vec<i32>)> {
        let mut points: Option<Vec<StbttPoint>> = None;
        let mut num_points: i32 = 0;
        let num_verts = vertices.len();

        let objspace_flatness_squared = objspace_flatness * objspace_flatness;
        let mut n: i32 = 0;
        let mut start: i32 = 0;

        for v in vertices.iter().take(num_verts) {
            if v.type_ == STBTT_VMOVE {
                n += 1;
            }
        }

        if n == 0 {
            return None;
        }

        let mut contour_lengths: Vec<i32> = vec![0; n as usize];

        for pass in 0..2 {
            let mut x: f32 = 0.0;
            let mut y: f32 = 0.0;
            if pass == 1 {
                points = Some(vec![StbttPoint::default(); num_points.max(0) as usize]);
            }
            num_points = 0;
            n = -1;
            for v in vertices.iter().take(num_verts) {
                match v.type_ {
                    STBTT_VMOVE => {
                        if n >= 0 {
                            contour_lengths[n as usize] = num_points - start;
                        }
                        n += 1;
                        start = num_points;

                        x = v.x as f32;
                        y = v.y as f32;
                        stbtt__add_point(&mut points, num_points, x, y);
                        num_points += 1;
                    }
                    STBTT_VLINE => {
                        x = v.x as f32;
                        y = v.y as f32;
                        stbtt__add_point(&mut points, num_points, x, y);
                        num_points += 1;
                    }
                    STBTT_VCURVE => {
                        stbtt__tesselate_curve(
                            &mut points,
                            &mut num_points,
                            x,
                            y,
                            v.cx as f32,
                            v.cy as f32,
                            v.x as f32,
                            v.y as f32,
                            objspace_flatness_squared,
                            0,
                        );
                        x = v.x as f32;
                        y = v.y as f32;
                    }
                    STBTT_VCUBIC => {
                        stbtt__tesselate_cubic(
                            &mut points,
                            &mut num_points,
                            x,
                            y,
                            v.cx as f32,
                            v.cy as f32,
                            v.cx1 as f32,
                            v.cy1 as f32,
                            v.x as f32,
                            v.y as f32,
                            objspace_flatness_squared,
                            0,
                        );
                        x = v.x as f32;
                        y = v.y as f32;
                    }
                    _ => {}
                }
            }
            contour_lengths[n as usize] = num_points - start;
        }

        points.map(|p| (p, contour_lengths))
    }

    /// imstb_truetype.h:4665-4684  `stbtt_GetFontNameString`: the bytes of the string (which may
    /// be big-endian double byte); its length is the C's `*length`.
    pub fn stbtt_get_font_name_string<'a>(
        font: &StbttFontinfo<'a>,
        platform_id: i32,
        encoding_id: i32,
        language_id: i32,
        name_id: i32,
    ) -> Option<&'a [u8]> {
        let fc = font.data;
        let offset = font.fontstart as u32;
        let nm = stbtt__find_table(fc, offset, b"name");
        if nm == 0 {
            return None;
        }

        let count = tt_ushort(fc, nm as i64 + 2) as i32;
        let string_offset = nm as i64 + tt_ushort(fc, nm as i64 + 4) as i64;
        for i in 0..count {
            let loc = nm as i64 + 6 + 12 * i as i64;
            if platform_id == tt_ushort(fc, loc) as i32
                && encoding_id == tt_ushort(fc, loc + 2) as i32
                && language_id == tt_ushort(fc, loc + 4) as i32
                && name_id == tt_ushort(fc, loc + 6) as i32
            {
                let length = tt_ushort(fc, loc + 8) as usize;
                let start = (string_offset + tt_ushort(fc, loc + 10) as i64) as usize;
                let start = start.min(fc.len());
                let end = (start + length).min(fc.len());
                return Some(&fc[start..end]);
            }
        }
        None
    }

    /// imstb_truetype.h:4785-4788
    pub fn stbtt_get_font_offset_for_index(data: &[u8], index: i32) -> i32 {
        stbtt_get_font_offset_for_index_internal(data, index)
    }

    /// imstb_truetype.h:4790-4793
    pub fn stbtt_get_number_of_fonts(data: &[u8]) -> i32 {
        stbtt_get_number_of_fonts_internal(data)
    }

    /// imstb_truetype.h:4795-4798  `stbtt_InitFont(info, data, offset)`; `None` where it returns 0.
    pub fn stbtt_init_font(data: &[u8], offset: i32) -> Option<StbttFontinfo<'_>> {
        stbtt_init_font_internal(data, offset)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_count_lines() {
        // Emboss.cpp:1514  get_count_lines
        assert_eq!(from_utf8::get_count_lines(""), 0);
        assert_eq!(from_utf8::get_count_lines("line1\nline2"), 2);
        assert_eq!(from_utf8::get_count_lines("a\nb\nc\nd"), 4);
    }

    #[test]
    fn test_project_z_create_front_back() {
        // Emboss.cpp:1847  ProjectZ::create_front_back
        let proj = ProjectZ::new(5.0);
        let p = Point::new(100, 200);
        let (front, back) = proj.create_front_back(&p);
        assert_eq!(front.x, 100.0);
        assert_eq!(front.y, 200.0);
        assert_eq!(front.z, 0.0);
        assert_eq!(back.x, 100.0);
        assert_eq!(back.y, 200.0);
        assert_eq!(back.z, 5.0);
    }

    #[test]
    fn test_ortho_project_3d() {
        // Emboss.hpp:347  OrthoProject3d::project
        let direction = Vec3d::new(0.0, 0.0, 10.0);
        let proj = OrthoProject3d::new(direction);
        let point = Vec3d::new(5.0, 5.0, 0.0);
        let projected = proj.project(&point);
        assert_eq!(projected.x, 5.0);
        assert_eq!(projected.y, 5.0);
        assert_eq!(projected.z, 10.0);
    }

    #[test]
    fn test_constants() {
        assert_eq!(UNION_DELTA, 50.0);
        assert_eq!(UNION_MAX_ITERATIN, 10);
        assert_eq!(ENTER_UNICODE, '\n' as u32);
        assert_eq!(SHAPE_SCALE, 0.001);
    }
}
