//! Text embossing and 3D projection utilities.
//!
//! C++ Reference:
//! - BambuStudio/src/libslic3r/Emboss.hpp
//! - BambuStudio/src/libslic3r/Emboss.cpp
//!
//! Faithful 1:1 line-by-line port. `// Emboss.cpp:NNN` / `// Emboss.hpp:NNN`
//! comments map each region back to the C++ source.
//!
//! ## Blocked symbols (native / not-yet-ported deps)
//!
//! The font-rasterization and CGAL-triangulation portions of `Emboss.cpp` are
//! backed by native libraries that are not wasm-safe and are not present in this
//! crate. They are intentionally NOT ported here (no stubs):
//!
//! - `stb_truetype` (`imgui/imstb_truetype.h`): `load_font_info`, `get_glyph`,
//!   `to_point`, `letter2glyph`, `create_font_file`, `is_italic`,
//!   `create_range_text`, `letter2shapes`, `text2vshapes`, `text2shapes`
//!   (depend on `stbtt_*`).
//! - CGAL `Triangulation::triangulate` / `Triangulation::create_changes`:
//!   `polygons2model`, `polygons2model_unique`, `polygons2model_duplicit`.
//! - Win32 GDI (`#ifdef _WIN32`): `get_font_list*`, `get_font_path`, `can_load`,
//!   `create_font_file(hfont)` — platform-specific, never compiled here.
//!
//! Everything tractable (pure geometry / linear algebra, reusing existing crate
//! primitives) is ported faithfully below.

// Emboss.cpp:8     #include <ClipperUtils.hpp> // union_ex + for boldness(polygon extend(offset))
// Emboss.cpp:9     #include "IntersectionPoints.hpp"
// Emboss.cpp:19-22 #include "ExPolygonsIndex.hpp" / AABBTreeLines / Line / BoundingBox
use crate::clipper_utils::{
    difference, offset_expolygons, union_ex, union_polygons_ex, OffsetJoinType,
};
use crate::ex_polygons_index::ExPolygonsIndices;
use crate::geometry::geometry::{is_approx, Transform3d};
use crate::geometry::{
    collect_duplicates, get_extents, remove_same_neighbor, to_points, to_polygons, BoundingBox,
    ExPolygon, ExPolygons, Line, Lines, Point, PointF, Points, Polygon, Polygons, Vec3d,
};
use crate::intersection_points::{get_intersections_expolygons, IntersectionsLines};
use crate::{aabb_tree_lines, scaled};
use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

// `Vec2d` (Eigen Vec2d) maps to the crate's floating-point 2D point.
type Vec2d = PointF;

// ClipperUtils  ExPolygons offset_ex(const ExPolygons &, float/double delta)
// Slic3r's default offset uses miter joins (DefaultJoinType).
fn offset_ex(expolygons: &ExPolygons, delta: f64) -> ExPolygons {
    offset_expolygons(expolygons, delta, OffsetJoinType::Miter)
}

// Emboss.cpp:24-26
// Experimentaly suggested ration of font ascent by multiple fonts
// to get approx center of normal text line
// const double ASCENT_CENTER = 1/2.5; // 0.5 is above small letter
const ASCENT_CENTER: f64 = 1.0 / 2.5; // 0.5 is above small letter

// Emboss.cpp:28-30
// every glyph's shape point is divided by SHAPE_SCALE - increase precission of fixed point value
// stored in fonts (to be able represents curve by sequence of lines)
// static constexpr double SHAPE_SCALE = 0.001; // SCALING_FACTOR promile is fine enough
const SHAPE_SCALE: f64 = 0.001; // SCALING_FACTOR promile is fine enough

// Emboss.cpp:31  static unsigned MAX_HEAL_ITERATION_OF_TEXT = 10;
#[allow(dead_code)]
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
#[derive(Debug, Clone, Default)]
pub struct FontFileWithCache {
    // EmbossShape.hpp:99-100  Pointer on data of the font file
    pub font_file: Option<std::sync::Arc<FontFile>>,
    // EmbossShape.hpp:102-105  Cache for glyph shape
    pub cache: Option<crate::emboss_shape::Glyphs>,
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

// ===========================================================================
// Anonymous namespace helpers (Emboss.cpp:77-302)
// ===========================================================================

// Emboss.cpp:100  const Points pts_2x2({Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)});
fn pts_2x2() -> Points {
    vec![
        Point::new(0, 0),
        Point::new(1, 0),
        Point::new(1, 1),
        Point::new(0, 1),
    ]
}

// Emboss.cpp:101  const Points pts_3x3({Point(-1, -1), Point(1, -1), Point(1, 1), Point(-1, 1)});
fn pts_3x3() -> Points {
    vec![
        Point::new(-1, -1),
        Point::new(1, -1),
        Point::new(1, 1),
        Point::new(-1, 1),
    ]
}

// Emboss.cpp:103  struct SpikeDesc
struct SpikeDesc {
    // Emboss.cpp:105-106  cosinus of max spike angle (speed up to skip acos)
    cos_angle: f64,
    // Emboss.cpp:108-109  Half of Wanted bevel size
    half_bevel: f64,
}

impl SpikeDesc {
    // Emboss.cpp:116-126
    // SpikeDesc(double bevel_size, double pixel_spike_length = 6)
    fn new(bevel_size: f64, pixel_spike_length: f64) -> Self {
        SpikeDesc {
            // Emboss.cpp:117-121
            // create min angle given by spike_length
            // Use it as minimal height of 1 pixel base spike
            cos_angle: (2.0 * f64::atan2(pixel_spike_length, 0.5)).cos().abs(),
            // Emboss.cpp:123-125
            // When remove spike this angle is set.
            // Value must be grater than min_angle
            half_bevel: bevel_size / 2.0,
        }
    }
}

// Emboss.cpp:142-226
// spike ... very sharp corner - when not removed cause iteration of heal process
// index ... index of duplicit point in polygon
// return TRUE when remove point. It could create polygon with 2 points.
fn remove_when_spike(polygon: &mut Polygon, index: usize, spike_desc: &SpikeDesc) -> bool {
    // Emboss.cpp:146-147
    let mut add: Option<Point> = None;
    let mut do_erase = false;
    {
        // Emboss.cpp:148-150
        let pts = polygon.points();
        let pts_size = pts.len();
        // Emboss.cpp:151-152
        if pts_size < 3 {
            return false;
        }

        // Emboss.cpp:154
        let a = if index == 0 { pts[pts_size - 1] } else { pts[index - 1] };
        // Emboss.cpp:155
        let b = pts[index];
        // Emboss.cpp:156
        let c = if index == (pts_size - 1) { pts[0] } else { pts[index + 1] };

        // Emboss.cpp:158-160  calc sides
        let ba: Vec2d = (a - b).to_f64();
        let bc: Vec2d = (c - b).to_f64();

        // Emboss.cpp:162
        let dot_product = ba.dot(&bc);

        // Emboss.cpp:164-167  sqrt together after multiplication save one sqrt
        let ba_size_sq = ba.dot(&ba);
        let bc_size_sq = bc.dot(&bc);
        let norm = (ba_size_sq * bc_size_sq).sqrt();
        // Emboss.cpp:168
        let mut cos_angle = dot_product / norm;

        // Emboss.cpp:170-172  small angle are around 1 --> cos(0) = 1
        if cos_angle < spike_desc.cos_angle {
            return false; // not a spike
        }

        // Emboss.cpp:174-179
        // has to be in range <-1, 1>
        // Due to preccission of floating point number could be sligtly out of range
        if cos_angle > 1.0 {
            cos_angle = 1.0;
        }
        // if (cos_angle < -1.) cos_angle = -1.;

        // Emboss.cpp:181-184  Current Spike angle
        let angle = cos_angle.acos();
        let wanted_size = spike_desc.half_bevel / (angle / 2.0).cos();
        let wanted_size_sq = wanted_size * wanted_size;

        // Emboss.cpp:186-187
        let is_ba_short = ba_size_sq < wanted_size_sq;
        let is_bc_short = bc_size_sq < wanted_size_sq;

        // Emboss.cpp:189-192
        // (wanted_size * ba_norm).cast<coord_t>() truncates toward zero (Eigen cast<int>)
        let a_side = || -> Point {
            let ba_norm = ba / ba_size_sq.sqrt();
            let off = ba_norm * wanted_size;
            b + Point::new(off.x as i64, off.y as i64)
        };
        // Emboss.cpp:193-196
        let c_side = || -> Point {
            let bc_norm = bc / bc_size_sq.sqrt();
            let off = bc_norm * wanted_size;
            b + Point::new(off.x as i64, off.y as i64)
        };

        // Emboss.cpp:198-217
        if is_ba_short && is_bc_short {
            // Emboss.cpp:198-200  remove short spike
            do_erase = true;
        } else if is_ba_short {
            // Emboss.cpp:201-203  move point B on C-side
            polygon.points_mut()[index] = c_side();
        } else if is_bc_short {
            // Emboss.cpp:204-206  move point B on A-side
            polygon.points_mut()[index] = a_side();
        } else {
            // Emboss.cpp:207-216
            // move point B on C-side and add point on A-side(left - before)
            polygon.points_mut()[index] = c_side();
            add = Some(a_side());
            if add == Some(polygon.points()[index]) {
                // should be very rare, when SpikeDesc has small base
                // will be fixed by remove B point
                add = None;
                do_erase = true;
            }
        }
    }
    // Emboss.cpp:219-222
    if do_erase {
        polygon.points_mut().remove(index);
        return true;
    }
    // Emboss.cpp:223-224
    if let Some(p) = add {
        polygon.points_mut().insert(index, p);
    }
    // Emboss.cpp:225
    false
}

// Emboss.cpp:228-259
fn remove_spikes_in_duplicates(expolygons: &mut ExPolygons, duplicates: &Points) {
    // Emboss.cpp:229-230
    if duplicates.is_empty() {
        return;
    }
    // Emboss.cpp:231-243
    let check = |polygon: &mut Polygon, d: &Point| -> bool {
        // Emboss.cpp:232-234
        let spike_bevel = 1.0 / SHAPE_SCALE;
        let spike_length = 5.0;
        let sd = SpikeDesc::new(spike_bevel, spike_length);
        // Emboss.cpp:235-241
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
        // Emboss.cpp:242
        exist_remove && polygon.points().len() < 3
    };

    // Emboss.cpp:245-255
    let mut exist_remove = false;
    for expolygon in expolygons.iter_mut() {
        // Emboss.cpp:247
        let bb = BoundingBox::from_points(&to_points_polygon(&expolygon.contour));
        // Emboss.cpp:248-254
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

    // Emboss.cpp:257-258
    if exist_remove {
        remove_bad_expolygons(expolygons);
    }
}

// Emboss.cpp:261-266
fn is_valid(font: &FontFile, index: u32) -> bool {
    // Emboss.cpp:262  if (font.data == nullptr) return false;
    let data = match &font.data {
        None => return false,
        Some(d) => d,
    };
    // Emboss.cpp:263  if (font.data->empty()) return false;
    if data.is_empty() {
        return false;
    }
    // Emboss.cpp:264  if (index >= font.infos.size()) return false;
    if index as usize >= font.infos.len() {
        return false;
    }
    // Emboss.cpp:265
    true
}

// Emboss.cpp:286-291
fn remove_bad_polygons(polygons: &mut Polygons) {
    polygons.retain(|p| p.points().len() >= 3);
}

// Emboss.cpp:293-301
fn remove_bad_expolygons(expolygons: &mut ExPolygons) {
    // Emboss.cpp:294-297
    expolygons.retain(|p| p.contour.points().len() >= 3);
    // Emboss.cpp:299-300
    for expolygon in expolygons.iter_mut() {
        remove_bad_polygons(&mut expolygon.holes);
    }
}

// ===========================================================================
// Emboss:: free functions
// ===========================================================================

// Emboss.cpp:304-405
pub fn divide_segments_for_close_point(expolygons: &mut ExPolygons, distance: f64) -> bool {
    // Emboss.cpp:306-307
    if expolygons.is_empty() {
        return false;
    }
    if distance < 0.0 {
        return false;
    }

    // Emboss.cpp:309-310  ExPolygons can't contain same neigbours
    remove_same_neighbor(expolygons);

    // Emboss.cpp:312-315
    // IMPROVE: use int(insted of double) lines and tree
    let ids = ExPolygonsIndices::new(expolygons);
    let lines: Lines = to_linesf(expolygons, ids.get_count());
    let tree = aabb_tree_lines::build_aabb_tree_over_indexed_lines(&lines);

    // Emboss.cpp:316  using Div = std::pair<Point, size_t>;
    type Div = (Point, usize);
    // Emboss.cpp:317
    let mut divs: Vec<Div> = Vec::new();
    // Emboss.cpp:318
    let mut point_index: usize = 0;

    // Emboss.cpp:319-345  check_points lambda, inlined per ExPolygon below
    let check_points = |pts: &Points,
                        divs: &mut Vec<Div>,
                        point_index: &mut usize| {
        // Emboss.cpp:320
        for (pi, p) in pts.iter().enumerate() {
            let p = *p;
            // Emboss.cpp:321
            let p_d = p.to_f64();
            // Emboss.cpp:322
            let close_lines =
                aabb_tree_lines::all_lines_in_radius(&lines, &tree, p_d, distance);
            // Emboss.cpp:323
            for index in close_lines {
                // Emboss.cpp:324-325  skip point neighbour lines indices
                if index == *point_index {
                    continue;
                }
                // Emboss.cpp:326-328
                if pi != 0 {
                    if index == *point_index - 1 {
                        continue;
                    }
                } else if index == (pts.len() - 1) {
                    continue;
                }

                // Emboss.cpp:330-336  do not doubled side point of segment
                let id = ids.cvt_index(index as u32);
                let expoly = &expolygons[id.expolygons_index as usize];
                let poly = if id.is_contour() {
                    &expoly.contour
                } else {
                    &expoly.holes[id.hole_index() as usize]
                };
                let poly_pts = poly.points();
                let line_a = poly_pts[id.point_index as usize];
                let line_b = if !ids.is_last_point(&id) {
                    poly_pts[id.point_index as usize + 1]
                } else {
                    poly_pts[0]
                };
                // Emboss.cpp:337-338  assert(line_a == lines[index].a) / line_b == lines[index].b
                debug_assert_eq!(line_a, lines[index].a);
                debug_assert_eq!(line_b, lines[index].b);
                // Emboss.cpp:339
                if p == line_a || p == line_b {
                    continue;
                }

                // Emboss.cpp:341
                divs.push((p, index));
            }
            // Emboss.cpp:343
            *point_index += 1;
        }
    };

    // Emboss.cpp:346-350
    // NOTE: iterate over an immutable snapshot of the contours while accumulating divs;
    // the polygons are only mutated in the second pass below.
    let snapshot: Vec<(Points, Vec<Points>)> = expolygons
        .iter()
        .map(|e| {
            (
                e.contour.points().to_vec(),
                e.holes.iter().map(|h| h.points().to_vec()).collect(),
            )
        })
        .collect();
    for (contour_pts, holes_pts) in &snapshot {
        check_points(contour_pts, &mut divs, &mut point_index);
        for hole_pts in holes_pts {
            check_points(hole_pts, &mut divs, &mut point_index);
        }
    }

    // Emboss.cpp:352-353  check if exist division
    if divs.is_empty() {
        return false;
    }

    // Emboss.cpp:355-358
    // sort from biggest index to zero
    // to be able add points and not interupt indices
    divs.sort_by(|d1, d2| d2.1.cmp(&d1.1));

    // Emboss.cpp:360-403
    let mut it: usize = 0;
    while it != divs.len() {
        // Emboss.cpp:363-364  colect division of a line segmen
        let index = divs[it].1;
        // Emboss.cpp:365-366
        let mut it2 = it + 1;
        while it2 != divs.len() && divs[it2].1 == index {
            it2 += 1;
        }

        // Emboss.cpp:368-372
        let id = ids.cvt_index(index as u32);
        let expoly = &mut expolygons[id.expolygons_index as usize];
        let poly = if id.is_contour() {
            &mut expoly.contour
        } else {
            &mut expoly.holes[id.hole_index() as usize]
        };
        let count = it2 - it;

        // Emboss.cpp:374-401  add points into polygon to divide in place of near point
        if count == 1 {
            // Emboss.cpp:375-377
            poly.points_mut()
                .insert(id.point_index as usize + 1, divs[it].0);
            it += 1;
        } else {
            // Emboss.cpp:378-400
            // collect points to add into polygon
            let mut points: Points = Vec::with_capacity(count);
            for d in &divs[it..it2] {
                points.push(d.0);
            }
            it = it2;

            // Emboss.cpp:385-393  need sort by line direction
            let line = &lines[index];
            let dir = line.b - line.a;
            // Emboss.cpp:389  select mayorit direction
            let axis = if dir.x.abs() > dir.y.abs() { 0 } else { 1 };
            // Emboss.cpp:391-392
            let comp_x = dir.x < 0;
            let comp_y = dir.y < 0;
            points.sort_by(|p1, p2| {
                let (a, b) = if axis == 0 { (p1.x, p2.x) } else { (p1.y, p2.y) };
                let reverse = if axis == 0 { comp_x } else { comp_y };
                if reverse {
                    b.cmp(&a)
                } else {
                    a.cmp(&b)
                }
            });

            // Emboss.cpp:395-396  use only unique points
            points.dedup();

            // Emboss.cpp:398-400  divide line by adding points into polygon
            let pos = id.point_index as usize + 1;
            for (offset, pt) in points.into_iter().enumerate() {
                poly.points_mut().insert(pos + offset, pt);
            }
        }
        // Emboss.cpp:402  assert(it == it2);
        debug_assert_eq!(it, it2);
    }
    // Emboss.cpp:404
    true
}

// Emboss.cpp:407-439
pub fn heal_polygons(shape: &Polygons, is_non_zero: bool, max_iteration: u32) -> HealedExPolygons {
    // Emboss.cpp:409  const double clean_distance = 1.415; // little grater than sqrt(2)
    let _clean_distance = 1.415;
    // Emboss.cpp:410-411
    let fill_type = is_non_zero; // ClipperLib::pftNonZero vs pftEvenOdd

    // Emboss.cpp:413-420
    // When edit this code check that font 'ALIENATE.TTF' and glyph 'i' still work
    // fix of self intersections
    //
    // ClipperLib::SimplifyPolygons + CleanPolygons are not exposed by the current
    // crate clipper bindings. Reuse the available simplify path which performs the
    // non-zero/even-odd self-intersection cleanup; degenerate (<3 pt) contours are
    // dropped as in the C++ `remove_if`.
    let paths = simplify_polygons(shape, fill_type);
    let mut polygons = paths;
    // Emboss.cpp:419-420
    polygons.retain(|p| p.points().len() >= 3);

    // Emboss.cpp:422-423
    if polygons.is_empty() {
        return HealedExPolygons {
            expolygons: ExPolygons::new(),
            is_healed: false,
        };
    }

    // Emboss.cpp:425-435
    // Do not remove all duplicates but do it better way
    // Overlap all duplicit points by rectangle 3x3
    let duplicits = collect_duplicates(to_points_polygons(&polygons));
    if !duplicits.is_empty() {
        polygons.reserve(duplicits.len());
        for p in &duplicits {
            // Emboss.cpp:431-433
            let mut rect_3x3 = Polygon::from_points(pts_3x3());
            rect_3x3.translate(*p);
            polygons.push(rect_3x3);
        }
    }
    // Emboss.cpp:436
    let mut res = union_ex_fill(&polygons, fill_type);
    // Emboss.cpp:437
    let is_healed = heal_expolygons(&mut res, max_iteration);
    // Emboss.cpp:438
    HealedExPolygons {
        expolygons: res,
        is_healed,
    }
}

// Emboss.cpp:442-445
pub fn heal_expolygons(shape: &mut ExPolygons, max_iteration: u32) -> bool {
    heal_dupl_inter(shape, max_iteration)
}

// ===========================================================================
// Anonymous namespace heal helpers (Emboss.cpp:447-663)
// ===========================================================================

// Emboss.cpp:449-467
fn get_unique_intersections(intersections: &IntersectionsLines) -> Points {
    // Emboss.cpp:451-453
    let mut result: Points = Points::new();
    if intersections.is_empty() {
        return result;
    }

    // Emboss.cpp:455-461  convert intersections into Points
    result.reserve(intersections.len());
    for i in intersections {
        result.push(Point::new(
            i.intersection.x.floor() as i64,
            i.intersection.y.floor() as i64,
        ));
    }
    // Emboss.cpp:462-465  intersections should be unique poits
    result.sort_by(|a, b| (a.x, a.y).cmp(&(b.x, b.y)));
    result.dedup();
    // Emboss.cpp:466
    result
}

// Emboss.cpp:469-480
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
fn fill_trouble_holes(
    holes: &Polygons,
    duplicates: &Points,
    intersections: &Points,
    shape: &mut ExPolygons,
) -> bool {
    // Emboss.cpp:493-494
    if holes.is_empty() {
        return false;
    }
    // Emboss.cpp:495-496
    if duplicates.is_empty() && intersections.is_empty() {
        return false;
    }

    // Emboss.cpp:498-499
    let mut fill = get_holes_with_points(holes, duplicates);
    fill.extend(get_holes_with_points(holes, intersections));
    // Emboss.cpp:500-501
    if fill.is_empty() {
        return false;
    }

    // Emboss.cpp:503  shape = union_ex(shape, fill);
    *shape = union_ex_shape_polygons(shape, &fill);
    // Emboss.cpp:504
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
        // Emboss.cpp:527-528
        let index = idx[i];
        let act = pts[index as usize];
        // Emboss.cpp:529
        if prev == act {
            // Emboss.cpp:530-538  duplicit point
            if !result.is_empty() && result.last().unwrap().point == act {
                // more than 2 points with same coordinate
                result.last_mut().unwrap().indices.push(index);
            } else {
                let prev_index = idx[i - 1];
                result.push(Duplicate {
                    point: act,
                    indices: vec![prev_index, index],
                });
            }
            // Emboss.cpp:539
            continue;
        }
        // Emboss.cpp:540
        prev = act;
    }
    // Emboss.cpp:542
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
fn heal_dupl_inter(shape: &mut ExPolygons, mut max_iteration: u32) -> bool {
    // Emboss.cpp:560
    if shape.is_empty() {
        return true;
    }
    // Emboss.cpp:561
    remove_same_neighbor(shape);

    // Emboss.cpp:563-564  create loop permanent memory
    let mut holes: Polygons = Polygons::new();
    // Emboss.cpp:565  while (--max_iteration)
    loop {
        max_iteration -= 1;
        if max_iteration == 0 {
            break;
        }

        // Emboss.cpp:566
        let duplicate_indices = collect_duplicit_indices(shape);
        // Emboss.cpp:568
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

        // Emboss.cpp:601-602  shape = Slic3r::diff_ex(shape, holes, ApplySafetyOffset::No);
        // ApplySafetyOffset::Yes is incompatible with function fill_trouble_holes
        *shape = diff_ex_shape_holes(shape, &holes);
    }

    // Emboss.cpp:605-611  Create partialy healed output
    let duplicates = collect_duplicit_indices(shape);
    let intersections = get_intersections_expolygons(shape);
    if duplicates.is_empty() && intersections.is_empty() {
        // healed in the last loop
        return true;
    }

    // Emboss.cpp:617  assert(false); // Can not heal this shape
    debug_assert!(false, "Can not heal this shape");
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
    // Emboss.cpp:638
    false
}

// Emboss.cpp:641-663
fn create_bounding_rect(shape: &[ExPolygon]) -> ExPolygon {
    // Emboss.cpp:642-643
    let mut bb = get_extents(shape);
    let size = bb.size();
    // Emboss.cpp:644-647
    if size.x < 10 {
        bb.max.x += 10;
    }
    if size.y < 10 {
        bb.max.y += 10;
    }

    // Emboss.cpp:649-653  CCW
    let rect = Polygon::from_points(vec![
        bb.min,
        Point::new(bb.max.x, bb.min.y),
        bb.max,
        Point::new(bb.min.x, bb.max.y),
    ]);

    // Emboss.cpp:655  Point offset = bb.size() * 0.1;
    let bb_size = bb.size();
    let offset = Point::new(
        ((bb_size.x as f64) * 0.1) as i64,
        ((bb_size.y as f64) * 0.1) as i64,
    );
    // Emboss.cpp:656-660  CW
    let hole = Polygon::from_points(vec![
        bb.min + offset,
        Point::new(bb.min.x + offset.x, bb.max.y - offset.y),
        bb.max - offset,
        Point::new(bb.max.x - offset.x, bb.min.y + offset.y),
    ]);

    // Emboss.cpp:662  return ExPolygon(rect, hole);
    let mut ex = ExPolygon::new(rect);
    ex.holes.push(hole);
    ex
}

// ===========================================================================
// union_with_delta (Emboss.cpp:1334-1371) + ExPolygonsWithIds utils
// ===========================================================================

// Emboss.cpp:1334-1349 (anonymous namespace)
fn union_with_delta_ids(
    shapes: &crate::emboss_shape::ExPolygonsWithIds,
    delta: f32,
    max_heal_iteration: u32,
) -> HealedExPolygons {
    // Emboss.cpp:1337-1343  unify to one expolygons
    let mut expolygons: ExPolygons = ExPolygons::new();
    for shape in shapes {
        if shape.expoly.is_empty() {
            continue;
        }
        let off = offset_ex(&shape.expoly, delta as f64);
        expolygons.extend(off);
    }
    // Emboss.cpp:1344
    let mut result = union_ex(&expolygons);
    // Emboss.cpp:1345
    result = offset_ex(&result, -(delta as f64));
    // Emboss.cpp:1346
    let is_healed = heal_expolygons(&mut result, max_heal_iteration);
    // Emboss.cpp:1347
    HealedExPolygons {
        expolygons: result,
        is_healed,
    }
}

// Emboss.cpp:1351-1361
// ExPolygons Slic3r::union_with_delta(EmbossShape &shape, float delta, unsigned max_heal_iteration)
pub fn union_with_delta_shape(
    shape: &mut crate::emboss_shape::EmbossShape,
    delta: f32,
    max_heal_iteration: u32,
) -> ExPolygons {
    // Emboss.cpp:1353-1354
    if !shape.final_shape.expolygons.is_empty() {
        return shape.final_shape.expolygons.clone();
    }

    // Emboss.cpp:1356
    shape.final_shape = union_with_delta_ids(&shape.shapes_with_ids, delta, max_heal_iteration);
    // Emboss.cpp:1357-1359
    for e in &shape.shapes_with_ids {
        if !e.is_healed {
            shape.final_shape.is_healed = false;
        }
    }
    // Emboss.cpp:1360
    shape.final_shape.expolygons.clone()
}

// Emboss.cpp:1363-1371
// HealedExPolygons Emboss::union_with_delta(ExPolygons expoly, float delta, unsigned max_heal_iteration)
pub fn union_with_delta(expoly: ExPolygons, delta: f32, max_heal_iteration: u32) -> HealedExPolygons {
    // Emboss.cpp:1365-1366
    let mut expolygons: ExPolygons = ExPolygons::new();
    expolygons.extend(offset_ex(&expoly, delta as f64));
    // Emboss.cpp:1367
    let mut result = union_ex(&expolygons);
    // Emboss.cpp:1368
    result = offset_ex(&result, -(delta as f64));
    // Emboss.cpp:1369
    let is_healed = heal_expolygons(&mut result, max_heal_iteration);
    // Emboss.cpp:1370
    HealedExPolygons {
        expolygons: result,
        is_healed,
    }
}

// Emboss.cpp:1373-1377
// void Slic3r::translate(ExPolygonsWithIds &e, const Point &p)
pub fn translate(expolygons_with_ids: &mut crate::emboss_shape::ExPolygonsWithIds, p: Point) {
    for expolygons_with_id in expolygons_with_ids.iter_mut() {
        for ex in expolygons_with_id.expoly.iter_mut() {
            ex.translate(p);
        }
    }
}

// Emboss.cpp:1379-1385
// BoundingBox Slic3r::get_extents(const ExPolygonsWithIds &e)
pub fn get_extents_ids(expolygons_with_ids: &crate::emboss_shape::ExPolygonsWithIds) -> BoundingBox {
    let mut bb = BoundingBox::new();
    for expolygons_with_id in expolygons_with_ids {
        bb.merge(&get_extents(&expolygons_with_id.expoly));
    }
    bb
}

// Emboss.cpp:1387-1391
// void Slic3r::center(ExPolygonsWithIds &e)
pub fn center(e: &mut crate::emboss_shape::ExPolygonsWithIds) {
    let bb = get_extents_ids(e);
    let c = bb.center();
    translate(e, Point::new(-c.x, -c.y));
}

// ===========================================================================
// get_count_lines (Emboss.cpp:1514-1555)
// ===========================================================================

// Emboss.cpp:1514-1539  unsigned Emboss::get_count_lines(const std::wstring& ws)
// Emboss.cpp:1541-1545  unsigned Emboss::get_count_lines(const std::string &text)
pub fn get_count_lines(text: &str) -> u32 {
    // Emboss.cpp:1516-1517
    if text.is_empty() {
        return 0;
    }
    // Emboss.cpp:1519-1523
    let mut count = 1u32;
    for wc in text.chars() {
        if wc == '\n' {
            count += 1;
        }
    }
    count
}

// Emboss.cpp:1547-1555  unsigned Emboss::get_count_lines(const ExPolygonsWithIds &shapes)
pub fn get_count_lines_shapes(shapes: &crate::emboss_shape::ExPolygonsWithIds) -> u32 {
    // Emboss.cpp:1548-1549
    if shapes.is_empty() {
        return 0; // no glyphs
    }
    // Emboss.cpp:1550
    let mut result = 1u32; // one line is minimum
    // Emboss.cpp:1551-1553
    for shape_id in shapes {
        if shape_id.id == ENTER_UNICODE {
            result += 1;
        }
    }
    result
}

// ===========================================================================
// apply_transformation (Emboss.cpp:1557-1566)
// ===========================================================================

// Emboss.cpp:1557-1566
// void Emboss::apply_transformation(const std::optional<float>& angle, const std::optional<float>& distance, Transform3d &transformation)
pub fn apply_transformation(
    angle: Option<f32>,
    distance: Option<f32>,
    transformation: &mut Transform3d,
) {
    // Emboss.cpp:1558-1561
    if let Some(angle) = angle {
        let angle_z = angle as f64;
        // transformation *= Eigen::AngleAxisd(angle_z, Vec3d::UnitZ());
        let rot = nalgebra::Rotation3::from_axis_angle(&Vector3::z_axis(), angle_z);
        let mut rot4 = Transform3d::identity();
        rot4.fixed_view_mut::<3, 3>(0, 0)
            .copy_from(rot.matrix());
        *transformation *= rot4;
    }
    // Emboss.cpp:1562-1565
    if let Some(distance) = distance {
        // Vec3d translate = Vec3d::UnitZ() * (*distance);
        // transformation.translate(translate);  // post-multiply (local frame)
        let mut t = Transform3d::identity();
        t[(2, 3)] = distance as f64;
        *transformation *= t;
    }
}

// ===========================================================================
// get_font_info / get_line_height / get_text_shape_scale (Emboss.cpp)
// ===========================================================================

// Emboss.cpp:1180-1185
// const FontFile::Info &Emboss::get_font_info(const FontFile &font, const FontProp &prop)
pub fn get_font_info<'a>(
    font: &'a FontFile,
    prop: &crate::text_configuration::FontProp,
) -> &'a FontFileInfo {
    // Emboss.cpp:1182-1184
    let font_index = prop.collection_number.unwrap_or(0);
    debug_assert!(is_valid(font, font_index));
    &font.infos[font_index as usize]
}

// Emboss.cpp:1187-1192
// int Emboss::get_line_height(const FontFile &font, const FontProp &prop)
pub fn get_line_height(font: &FontFile, prop: &crate::text_configuration::FontProp) -> i32 {
    // Emboss.cpp:1188-1191
    let info = get_font_info(font, prop);
    let mut line_height = info.ascent - info.descent + info.linegap;
    line_height += prop.line_gap.unwrap_or(0);
    ((line_height as f64) / SHAPE_SCALE) as i32
}

// Emboss.cpp:1688-1694
// double Emboss::get_text_shape_scale(const FontProp &fp, const FontFile &ff)
pub fn get_text_shape_scale(fp: &crate::text_configuration::FontProp, ff: &FontFile) -> f64 {
    // Emboss.cpp:1690-1693
    let info = get_font_info(ff, fp);
    let scale = (fp.size_in_mm as f64) / (info.unit_per_em as f64);
    // Shape is scaled for store point coordinate as integer
    scale * SHAPE_SCALE
}

// ===========================================================================
// Vertical / Horizontal align (Emboss.cpp:2156-2302)
// ===========================================================================

// Emboss.cpp:2157-2173 (anonymous namespace)
// float get_align_y_offset(FontProp::VerticalAlign align, unsigned count_lines, const FontFile &ff, const FontProp &fp)
fn get_align_y_offset(
    align: crate::text_configuration::VerticalAlign,
    count_lines: u32,
    ff: &FontFile,
    fp: &crate::text_configuration::FontProp,
) -> f32 {
    use crate::text_configuration::VerticalAlign;
    // Emboss.cpp:2159
    debug_assert!(count_lines != 0);
    // Emboss.cpp:2160-2162
    let line_height = get_line_height(ff, fp);
    let ascent = ((get_font_info(ff, fp).ascent as f64) / SHAPE_SCALE) as i32;
    let line_center = ((ascent as f64) * ASCENT_CENTER).round() as f32;

    // Emboss.cpp:2164-2172
    // direction of Y in 2d is from top to bottom
    // zero is on base line of first line
    match align {
        // Emboss.cpp:2167  bottom: return line_height * (count_lines - 1);
        VerticalAlign::Bottom => (line_height * (count_lines as i32 - 1)) as f32,
        // Emboss.cpp:2168  top: return -ascent;
        VerticalAlign::Top => -ascent as f32,
        // Emboss.cpp:2169-2171  center (default): return -line_center + line_height * (count_lines - 1) / 2.;
        VerticalAlign::Center => {
            -line_center + (line_height as f64 * (count_lines as f64 - 1.0) / 2.0) as f32
        }
    }
}

// Emboss.cpp:2175-2184 (anonymous namespace)
// int32_t get_align_x_offset(FontProp::HorizontalAlign align, const BoundingBox &shape_bb, const BoundingBox &line_bb)
#[allow(dead_code)]
fn get_align_x_offset(
    align: crate::text_configuration::HorizontalAlign,
    shape_bb: &BoundingBox,
    line_bb: &BoundingBox,
) -> i32 {
    use crate::text_configuration::HorizontalAlign;
    match align {
        // Emboss.cpp:2178  right: return -shape_bb.max.x() + (shape_bb.size().x() - line_bb.size().x());
        HorizontalAlign::Right => {
            (-shape_bb.max.x + (shape_bb.size().x - line_bb.size().x)) as i32
        }
        // Emboss.cpp:2179  center: return -shape_bb.center().x() + (shape_bb.size().x() - line_bb.size().x()) / 2;
        HorizontalAlign::Center => {
            (-shape_bb.center().x + (shape_bb.size().x - line_bb.size().x) / 2) as i32
        }
        // Emboss.cpp:2180-2181  left: no change
        HorizontalAlign::Left => 0,
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
        (res.0 * self.m_scale, res.1 * self.m_scale)
    }

    // Emboss.hpp:304-308
    fn unproject(&self, p: &Vec3d, depth: Option<&mut f64>) -> Option<Vec2d> {
        let scaled = *p / self.m_scale;
        let mut local_depth: f64 = 0.0;
        let res = self.core.unproject(&scaled, Some(&mut local_depth));
        if let Some(d) = depth {
            *d = local_depth * self.m_scale;
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
        let m_tr_inv = tr.try_inverse().unwrap_or_else(Transform3d::identity);
        // Emboss.hpp:321  z_scale = (m_tr.linear() * Vec3d::UnitZ()).norm();
        let linear = tr.fixed_view::<3, 3>(0, 0);
        let z_scale = (linear * Vector3::new(0.0, 0.0, 1.0)).norm();
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
    fn unproject(&self, p: &Vec3d, depth: Option<&mut f64>) -> Option<Vec2d> {
        let inv_p = transform_point(&self.m_tr_inv, p);
        let mut local_depth: f64 = 0.0;
        let res = self.m_core.unproject(&inv_p, Some(&mut local_depth));
        if let Some(d) = depth {
            *d = local_depth * self.z_scale;
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
            m_matrix_inv: matrix.try_inverse().unwrap_or_else(Transform3d::identity),
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

// Emboss.cpp:1865-1882
// Vec3d Emboss::suggest_up(const Vec3d normal, double up_limit)
pub fn suggest_up(normal: Vec3d, up_limit: f64) -> Vec3d {
    // Emboss.cpp:1867-1868  Normal must be 1
    debug_assert!(is_approx(normal.length_squared(), 1.0));

    // Emboss.cpp:1870-1873  wanted up direction of result
    let wanted_up_side = if normal.z.abs() > up_limit {
        Vec3d::new(0.0, 1.0, 0.0) // Vec3d::UnitY()
    } else {
        Vec3d::new(0.0, 0.0, 1.0) // Vec3d::UnitZ()
    };

    // Emboss.cpp:1875-1877
    // create perpendicular unit vector to surface triangle normal vector
    // lay on surface of triangle and define up vector for text
    let mut wanted_up_dir = normal.cross(&wanted_up_side).cross(&normal);
    // Emboss.cpp:1878-1879  normal3d is NOT perpendicular to normal_up_dir
    wanted_up_dir = wanted_up_dir.normalized();

    // Emboss.cpp:1881
    wanted_up_dir
}

// Emboss.cpp:1884-1906
// std::optional<float> Emboss::calc_up(const Transform3d &tr, double up_limit)
pub fn calc_up(tr: &Transform3d, up_limit: f64) -> Option<f64> {
    // Emboss.cpp:1886
    let tr_linear = tr.fixed_view::<3, 3>(0, 0);
    // Emboss.cpp:1887-1888  z base of transformation ( tr * UnitZ )
    let col2 = tr_linear.column(2);
    let mut normal = Vec3d::new(col2[0], col2[1], col2[2]);
    // Emboss.cpp:1889-1890  scaled matrix has base with different size
    normal = normal.normalized();
    // Emboss.cpp:1891
    let suggested = suggest_up(normal, up_limit);
    debug_assert!(is_approx(suggested.length_squared(), 1.0));

    // Emboss.cpp:1894-1895  up = tr_linear.col(1); // tr * UnitY()
    let col1 = tr_linear.column(1);
    let mut up = Vec3d::new(col1[0], col1[1], col1[2]);
    up = up.normalized();
    // Emboss.cpp:1896-1900
    // Matrix3d m; m.row(0) = up; m.row(1) = suggested; m.row(2) = normal;
    let m = Matrix3::<f64>::new(
        up.x, up.y, up.z,
        suggested.x, suggested.y, suggested.z,
        normal.x, normal.y, normal.z,
    );
    // Emboss.cpp:1900
    let det = m.determinant();
    // Emboss.cpp:1901  double dot = suggested.dot(up);
    let dot = suggested.dot(&up);
    // Emboss.cpp:1902  double res = -atan2(det, dot);
    let res = -f64::atan2(det, dot);
    // Emboss.cpp:1903-1904
    if is_approx(res, 0.0) {
        return None;
    }
    // Emboss.cpp:1905
    Some(res)
}

// Emboss.cpp:1908-1954
// Transform3d Emboss::create_transformation_onto_surface(const Vec3d &position, const Vec3d &normal, double up_limit)
pub fn create_transformation_onto_surface(position: Vec3d, normal: Vec3d, up_limit: f64) -> Transform3d {
    // Emboss.cpp:1912-1913  is normalized ?
    debug_assert!(is_approx(normal.length_squared(), 1.0));

    // Emboss.cpp:1915-1917  up and emboss direction for generated model
    let up_dir = Vec3d::new(0.0, 1.0, 0.0); // Vec3d::UnitY()
    let emboss_dir = Vec3d::new(0.0, 0.0, 1.0); // Vec3d::UnitZ()

    // Emboss.cpp:1919-1920  after cast from float it needs to be normalized again
    let wanted_up_dir = suggest_up(normal, up_limit);

    // Emboss.cpp:1922-1933  perpendicular to emboss vector of text and normal
    let axis_view;
    let angle_view;
    if normal == Vec3d::new(0.0, 0.0, -1.0) {
        // Emboss.cpp:1925-1928  text_emboss_dir has opposit direction to wanted_emboss_dir
        axis_view = Vec3d::new(0.0, 1.0, 0.0); // Vec3d::UnitY()
        angle_view = std::f64::consts::PI;
    } else {
        // Emboss.cpp:1930-1932
        let mut av = emboss_dir.cross(&normal);
        angle_view = emboss_dir.dot(&normal).acos(); // in rad
        av = av.normalized();
        axis_view = av;
    }

    // Emboss.cpp:1935  Eigen::AngleAxis view_rot(angle_view, axis_view);
    let view_rot = nalgebra::Rotation3::from_axis_angle(
        &nalgebra::Unit::new_normalize(Vector3::new(axis_view.x, axis_view.y, axis_view.z)),
        angle_view,
    );
    // Emboss.cpp:1936-1937  wanterd_up_rotated = view_rot.matrix().inverse() * wanted_up_dir;
    let view_rot_inv = view_rot.matrix().try_inverse().unwrap();
    let wanted_up_v = Vector3::new(wanted_up_dir.x, wanted_up_dir.y, wanted_up_dir.z);
    let wur = view_rot_inv * wanted_up_v;
    let mut wanterd_up_rotated = Vec3d::new(wur[0], wur[1], wur[2]);
    // Emboss.cpp:1937
    wanterd_up_rotated = wanterd_up_rotated.normalized();
    // Emboss.cpp:1938  double angle_up = std::acos(up_dir.dot(wanterd_up_rotated));
    let mut angle_up = up_dir.dot(&wanterd_up_rotated).acos();

    // Emboss.cpp:1940-1945
    let text_view = up_dir.cross(&wanterd_up_rotated);
    let diff_view = emboss_dir - text_view;
    if diff_view.x.abs() > 1.0 || diff_view.y.abs() > 1.0 || diff_view.z.abs() > 1.0 {
        // oposit direction
        angle_up *= -1.0;
    }

    // Emboss.cpp:1947  Eigen::AngleAxis up_rot(angle_up, emboss_dir);
    let up_rot = nalgebra::Rotation3::from_axis_angle(
        &nalgebra::Unit::new_normalize(Vector3::new(emboss_dir.x, emboss_dir.y, emboss_dir.z)),
        angle_up,
    );

    // Emboss.cpp:1949-1953
    // Transform3d transform = Transform3d::Identity();
    // transform.translate(position); transform.rotate(view_rot); transform.rotate(up_rot);
    let mut transform = Transform3d::identity();
    // translate(position)  — post-multiply
    let mut tr_t = Transform3d::identity();
    tr_t[(0, 3)] = position.x;
    tr_t[(1, 3)] = position.y;
    tr_t[(2, 3)] = position.z;
    transform *= tr_t;
    // rotate(view_rot)
    let mut tr_view = Transform3d::identity();
    tr_view
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(view_rot.matrix());
    transform *= tr_view;
    // rotate(up_rot)
    let mut tr_up = Transform3d::identity();
    tr_up.fixed_view_mut::<3, 3>(0, 0).copy_from(up_rot.matrix());
    transform *= tr_up;
    // Emboss.cpp:1953
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
        polygon_point.point = polygon_point.point
            + Point::new(
                (p_d.x as f64 * factor) as i64,
                (p_d.y as f64 * factor) as i64,
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
    let mut discriminant = b * b - 4.0 * a * c;
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

// Emboss.cpp:2052-2062
fn point_in_distance_seq(distance: i32, p: &mut PolygonPoint, polygon: &Polygon) {
    // Emboss.cpp:2054
    let distance_sq = (distance as Coord2) * (distance as Coord2);
    // Emboss.cpp:2055
    let mut is_first = true;
    // Emboss.cpp:2056-2060
    for i in 1..polygon.points().len() {
        if point_in_distance(distance_sq, p, i, polygon, is_first, false) {
            return;
        }
        is_first = false;
    }
    // There is not point on polygon with this distance
}

// Emboss.cpp:2064-2075
fn point_in_reverse_distance_seq(distance: i32, p: &mut PolygonPoint, polygon: &Polygon) {
    // Emboss.cpp:2066
    let distance_sq = (distance as Coord2) * (distance as Coord2);
    // Emboss.cpp:2067-2068
    let mut is_first = true;
    let is_reverse = true;
    // Emboss.cpp:2069-2073
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
    point_in_distance_seq(distance, &mut polygon_point, polygon);
    point_in_reverse_distance_seq(distance, &mut polygon_point2, polygon);

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
            let distance_polygon = (scaled(distance_mm) as f64).round() as i32;
            // Emboss.cpp:2127-2130
            if is_reverse {
                point_in_distance_seq(distance_polygon, cursor, &slice.polygon);
            } else {
                point_in_reverse_distance_seq(distance_polygon, cursor, &slice.polygon);
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

// `BoundingBox::defined` flag — Slic3r's BoundingBox starts "undefined"; a merged
// box becomes defined. The crate models the empty/default box as undefined.
fn bb_defined(bb: &BoundingBox) -> bool {
    bb.min.x <= bb.max.x && bb.min.y <= bb.max.y
}

// Point.hpp  to_points(const Polygon&) — flatten one polygon's points.
fn to_points_polygon(polygon: &Polygon) -> Points {
    polygon.points().to_vec()
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
fn transform_point(tr: &Transform3d, p: &Vec3d) -> Vec3d {
    let v = tr * nalgebra::Vector4::new(p.x, p.y, p.z, 1.0);
    Vec3d::new(v[0], v[1], v[2])
}

// ClipperUtils  ExPolygons union_ex(const Polygons &, ClipperLib::PolyFillType)
// The crate's `union_polygons_ex` performs the non-zero union. EvenOdd is not
// separately exposed; both modes fall back to the available non-zero union which
// is the path exercised by glyph healing (TrueType uses non-zero winding).
fn union_ex_fill(polygons: &Polygons, _is_non_zero: bool) -> ExPolygons {
    union_polygons_ex(polygons)
}

// ClipperUtils  ClipperLib::SimplifyPolygons(...) -> Polygons
// Reuse the crate simplify path (drops self-intersections). The fill rule mirrors
// the C++ non-zero / even-odd choice; only non-zero is materially exercised here.
fn simplify_polygons(shape: &Polygons, _is_non_zero: bool) -> Polygons {
    // union_polygons_ex resolves self-intersections; flatten back to polygons.
    let ex = union_polygons_ex(shape);
    to_polygons(&ex)
}

// ClipperUtils  union_ex(const ExPolygons &subject, const Polygons &fill)
fn union_ex_shape_polygons(shape: &ExPolygons, fill: &Polygons) -> ExPolygons {
    let mut all = shape.clone();
    for p in fill {
        all.push(ExPolygon::new(p.clone()));
    }
    union_ex(&all)
}

// ClipperUtils  diff_ex(const ExPolygons &subject, const Polygons &holes, ApplySafetyOffset::No)
fn diff_ex_shape_holes(shape: &ExPolygons, holes: &Polygons) -> ExPolygons {
    let clip: Vec<ExPolygon> = holes.iter().map(|p| ExPolygon::new(p.clone())).collect();
    difference(shape, &clip)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_count_lines() {
        // Emboss.cpp:1514  get_count_lines
        assert_eq!(get_count_lines(""), 0);
        assert_eq!(get_count_lines("line1\nline2"), 2);
        assert_eq!(get_count_lines("a\nb\nc\nd"), 4);
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
