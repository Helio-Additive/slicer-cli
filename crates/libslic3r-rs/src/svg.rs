//! Faithful 1:1 port of `SVG.cpp` / `SVG.hpp` from BambuStudio.
//!
//! C++ Reference:
//! - SVG.hpp
//! - SVG.cpp
//!
//! Coordinate conventions follow the rest of `libslic3r-rs`:
//! - `coord_t`  -> `i64`
//! - `coordf_t` -> `f64`
//!
//! Notes on the `ClipperLib::Path` overloads: BambuStudio renders raw
//! `ClipperLib::Path` values, where a `ClipperLib::Path` is a
//! `std::vector<IntPoint>` and `IntPoint::x()/y()` return 64-bit integers.
//! There is no separate `ClipperLib::Path` primitive in this crate, so the
//! faithful representation of a `ClipperLib::Path` is `Vec<Point>` (each
//! `Point` holds the same `i64` coordinates as an `IntPoint`).

// SVG.cpp:1   #include "SVG.hpp"
// SVG.cpp:2-6 (fstream/iostream/pugixml(commented)/nowide/json includes)

use std::fmt::Write as _;
use std::io::Write as _;

use crate::geometry::{
    get_extents as get_extents_expolygons, BoundingBox, ExPolygon, Line, Lines, Point, Polygon,
    Polygons, Polyline, Polylines, ThickLine, ThickLines, ThickPolylines, Vec2d,
};
use crate::surface::{Surface, Surfaces, SurfacesPtr};
use crate::{scale, scaled, unscale, Coord, CoordF};

// SVG.hpp:177   static float to_svg_coord(float x) throw() { return unscale<float>(x) * 10.f; }
#[inline]
fn to_svg_coord(x: f32) -> f32 {
    // C++ `unscale<float>(coord_t)` casts the integer coordinate to float and
    // multiplies by the (float) scaling factor; we route through the crate
    // `unscale` to stay consistent with the rest of the crate's coordinate
    // convention, then narrow to `f32` to honour the `float` template arg.
    (unscale(x as Coord) as f32) * 10.0f32
}

// SVG.hpp:178   static float to_svg_x(float x) throw() { return to_svg_coord(x); }
#[inline]
fn to_svg_x(x: f32) -> f32 {
    to_svg_coord(x)
}

/// SVG.hpp:13   class SVG
pub struct SVG {
    // SVG.hpp:16   bool arrows;
    pub arrows: bool,
    // SVG.hpp:17   std::string fill, stroke;
    pub fill: String,
    pub stroke: String,
    // SVG.hpp:18   Point origin;
    pub origin: Point,
    // SVG.hpp:19   float height;
    pub height: f32,
    // SVG.hpp:20   bool flipY;
    pub flip_y: bool,

    // SVG.hpp:88   std::string filename;
    filename: String,
    // SVG.hpp:89   FILE* f;
    f: Option<std::fs::File>,
}

/// SVG.hpp:104   struct ExPolygonAttributes
#[derive(Debug, Clone)]
pub struct ExPolygonAttributes {
    // SVG.hpp:155   std::string legend;
    pub legend: String,
    // SVG.hpp:156   std::string color_fill;
    pub color_fill: String,
    // SVG.hpp:157   std::string color_contour;
    pub color_contour: String,
    // SVG.hpp:158   std::string color_holes;
    pub color_holes: String,
    // SVG.hpp:159   std::string color_points;
    pub color_points: String,
    // SVG.hpp:160   coord_t outline_width { 0 };
    pub outline_width: Coord,
    // SVG.hpp:161   float fill_opacity;
    pub fill_opacity: f32,
    // SVG.hpp:162   coord_t radius_points { 0 };
    pub radius_points: Coord,
}

impl ExPolygonAttributes {
    // SVG.hpp:106   ExPolygonAttributes() : ExPolygonAttributes("gray", "black", "blue") {}
    pub fn new() -> Self {
        Self::with_3colors("gray", "black", "blue")
    }

    // SVG.hpp:107-108   ExPolygonAttributes(const std::string &color) : ExPolygonAttributes(color, color, color) {}
    pub fn with_color(color: &str) -> Self {
        Self::with_3colors(color, color, color)
    }

    // SVG.hpp:110-125   ExPolygonAttributes(color_fill, color_contour, color_holes, outline_width = scale_(0.05), fill_opacity = 0.5f, color_points = "black", radius_points = 0)
    pub fn with_3colors_full(
        color_fill: &str,
        color_contour: &str,
        color_holes: &str,
        outline_width: Coord,
        fill_opacity: f32,
        color_points: &str,
        radius_points: Coord,
    ) -> Self {
        Self {
            legend: String::new(),
            color_fill: color_fill.to_string(),
            color_contour: color_contour.to_string(),
            color_holes: color_holes.to_string(),
            outline_width,
            fill_opacity,
            color_points: color_points.to_string(),
            radius_points,
        }
    }

    // SVG.hpp:110-125 with default args (outline_width = scale_(0.05), fill_opacity = 0.5f, color_points = "black", radius_points = 0)
    pub fn with_3colors(color_fill: &str, color_contour: &str, color_holes: &str) -> Self {
        Self::with_3colors_full(
            color_fill,
            color_contour,
            color_holes,
            scale(0.05),
            0.5f32,
            "black",
            0,
        )
    }

    // SVG.hpp:127-144   ExPolygonAttributes(legend, color_fill, color_contour, color_holes, outline_width = scale_(0.05), fill_opacity = 0.5f, color_points = "black", radius_points = 0)
    pub fn with_legend_full(
        legend: &str,
        color_fill: &str,
        color_contour: &str,
        color_holes: &str,
        outline_width: Coord,
        fill_opacity: f32,
        color_points: &str,
        radius_points: Coord,
    ) -> Self {
        Self {
            legend: legend.to_string(),
            color_fill: color_fill.to_string(),
            color_contour: color_contour.to_string(),
            color_holes: color_holes.to_string(),
            outline_width,
            fill_opacity,
            color_points: color_points.to_string(),
            radius_points,
        }
    }

    // SVG.hpp:146-153   ExPolygonAttributes(legend, color_fill, fill_opacity)
    pub fn with_legend_opacity(legend: &str, color_fill: &str, fill_opacity: f32) -> Self {
        Self {
            legend: legend.to_string(),
            color_fill: color_fill.to_string(),
            color_contour: String::new(),
            color_holes: String::new(),
            // SVG.hpp:160   coord_t outline_width { 0 };
            outline_width: 0,
            fill_opacity,
            color_points: String::new(),
            // SVG.hpp:162   coord_t radius_points { 0 };
            radius_points: 0,
        }
    }
}

impl Default for ExPolygonAttributes {
    fn default() -> Self {
        Self::new()
    }
}

impl SVG {
    // SVG.hpp:23-25   SVG(const char* afilename) : arrows(false), fill("grey"), stroke("black"), filename(afilename), flipY(false) { open(filename); }
    pub fn new_filename(afilename: &str) -> Self {
        let mut svg = Self {
            arrows: false,
            fill: "grey".to_string(),
            stroke: "black".to_string(),
            origin: Point::zero(),
            height: 0.0,
            flip_y: false,
            filename: afilename.to_string(),
            f: None,
        };
        let filename = svg.filename.clone();
        svg.open(&filename);
        svg
    }

    // SVG.hpp:26-28   SVG(const char* afilename, const BoundingBox &bbox, const coord_t bbox_offset = scale_(1.), bool flipY = true) :
    //                  arrows(false), fill("grey"), stroke("black"), filename(afilename), origin(bbox.min - Point(bbox_offset, bbox_offset)), flipY(flipY) { open(filename, bbox, bbox_offset, flipY); }
    pub fn new_bbox(afilename: &str, bbox: &BoundingBox, bbox_offset: Coord, flip_y: bool) -> Self {
        let mut svg = Self {
            arrows: false,
            fill: "grey".to_string(),
            stroke: "black".to_string(),
            origin: bbox.min - Point::new(bbox_offset, bbox_offset),
            height: 0.0,
            flip_y,
            filename: afilename.to_string(),
            f: None,
        };
        let filename = svg.filename.clone();
        svg.open_bbox(&filename, bbox, bbox_offset, flip_y);
        svg
    }

    // SVG.hpp:26   bbox_offset = scale_(1.), flipY = true defaults
    pub fn new_bbox_default(afilename: &str, bbox: &BoundingBox) -> Self {
        Self::new_bbox(afilename, bbox, scale(1.0), true)
    }

    // SVG.hpp:43   bool is_opened() { return f != NULL; }
    pub fn is_opened(&self) -> bool {
        self.f.is_some()
    }

    // SVG.cpp:10   bool SVG::open(const char* afilename)
    pub fn open(&mut self, afilename: &str) -> bool {
        // SVG.cpp:12   this->filename = afilename;
        self.filename = afilename.to_string();
        // SVG.cpp:13   this->f = boost::nowide::fopen(afilename, "w");
        self.f = std::fs::File::create(afilename).ok();
        // SVG.cpp:14-15   if (this->f == NULL) return false;
        if self.f.is_none() {
            return false;
        }
        let f = self.f.as_mut().unwrap();
        // SVG.cpp:16-23   fprintf(this->f, "<?xml ...>");
        let _ = write!(
            f,
            "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n\
             <!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.0//EN\" \"http://www.w3.org/TR/2001/REC-SVG-20010904/DTD/svg10.dtd\">\n\
             <svg height=\"2000\" width=\"2000\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:svg=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\">\n\
                <marker id=\"endArrow\" markerHeight=\"8\" markerUnits=\"strokeWidth\" markerWidth=\"10\" orient=\"auto\" refX=\"1\" refY=\"5\" viewBox=\"0 0 10 10\">\n\
                   <polyline fill=\"darkblue\" points=\"0,0 10,5 0,10 1,5\" />\n\
                </marker>\n"
        );
        // SVG.cpp:24   fprintf(this->f, "<rect fill='white' stroke='none' x='0' y='0' width='%f' height='%f'/>\n", 2000.f, 2000.f);
        let _ = write!(
            f,
            "<rect fill='white' stroke='none' x='0' y='0' width='{:.6}' height='{:.6}'/>\n",
            2000.0f32, 2000.0f32
        );
        // SVG.cpp:25   return true;
        true
    }

    // SVG.cpp:28   bool SVG::open(const char* afilename, const BoundingBox &bbox, const coord_t bbox_offset, bool aflipY)
    pub fn open_bbox(
        &mut self,
        afilename: &str,
        bbox: &BoundingBox,
        bbox_offset: Coord,
        aflip_y: bool,
    ) -> bool {
        // SVG.cpp:30   this->filename = afilename;
        self.filename = afilename.to_string();
        // SVG.cpp:31   this->origin   = bbox.min - Point(bbox_offset, bbox_offset);
        self.origin = bbox.min - Point::new(bbox_offset, bbox_offset);
        // SVG.cpp:32   this->flipY    = aflipY;
        self.flip_y = aflip_y;
        // SVG.cpp:33   this->f        = boost::nowide::fopen(afilename, "w");
        self.f = std::fs::File::create(afilename).ok();
        // SVG.cpp:34-35   if (f == NULL) return false;
        if self.f.is_none() {
            return false;
        }
        // SVG.cpp:36   float w = to_svg_coord(bbox.max(0) - bbox.min(0) + 2 * bbox_offset);
        let w = to_svg_coord((bbox.max.x() - bbox.min.x() + 2 * bbox_offset) as f32);
        // SVG.cpp:37   float h = to_svg_coord(bbox.max(1) - bbox.min(1) + 2 * bbox_offset);
        let h = to_svg_coord((bbox.max.y() - bbox.min.y() + 2 * bbox_offset) as f32);
        // SVG.cpp:38   this->height   = h;
        self.height = h;
        let f = self.f.as_mut().unwrap();
        // SVG.cpp:39-46   fprintf(this->f, "<?xml ...>", h, w);
        let _ = write!(
            f,
            "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n\
             <!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.0//EN\" \"http://www.w3.org/TR/2001/REC-SVG-20010904/DTD/svg10.dtd\">\n\
             <svg height=\"{:.6}\" width=\"{:.6}\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:svg=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\">\n\
                <marker id=\"endArrow\" markerHeight=\"8\" markerUnits=\"strokeWidth\" markerWidth=\"10\" orient=\"auto\" refX=\"1\" refY=\"5\" viewBox=\"0 0 10 10\">\n\
                   <polyline fill=\"darkblue\" points=\"0,0 10,5 0,10 1,5\" />\n\
                </marker>\n",
            h, w
        );
        // SVG.cpp:47   fprintf(this->f, "<rect fill='white' stroke='none' x='0' y='0' width='%f' height='%f'/>\n", w, h);
        let _ = write!(
            f,
            "<rect fill='white' stroke='none' x='0' y='0' width='{:.6}' height='{:.6}'/>\n",
            w, h
        );
        // SVG.cpp:48   return true;
        true
    }

    // SVG.hpp:179   float to_svg_y(float x) const throw() { return flipY ? this->height - to_svg_coord(x) : to_svg_coord(x); }
    #[inline]
    fn to_svg_y(&self, x: f32) -> f32 {
        if self.flip_y {
            self.height - to_svg_coord(x)
        } else {
            to_svg_coord(x)
        }
    }

    // SVG.cpp:51   void SVG::draw(const Line &line, std::string stroke, coordf_t stroke_width)
    pub fn draw_line(&mut self, line: &Line, stroke: &str, stroke_width: CoordF) {
        let arrows = self.arrows;
        let origin = self.origin;
        // SVG.cpp:55 (argument computation done before borrow of f)
        let x1 = to_svg_x((line.a.x() - origin.x()) as f32);
        let y1 = self.to_svg_y((line.a.y() - origin.y()) as f32);
        let x2 = to_svg_x((line.b.x() - origin.x()) as f32);
        let y2 = self.to_svg_y((line.b.y() - origin.y()) as f32);
        let sw = if stroke_width == 0.0 {
            1.0f32
        } else {
            to_svg_coord(stroke_width as f32)
        };
        let f = self.f.as_mut().unwrap();
        // SVG.cpp:53-55   fprintf(this->f, "   <line x1=\"%f\" ...");
        let _ = write!(
            f,
            "   <line x1=\"{:.6}\" y1=\"{:.6}\" x2=\"{:.6}\" y2=\"{:.6}\" style=\"stroke: {}; stroke-width: {:.6}\"",
            x1, y1, x2, y2, stroke, sw
        );
        // SVG.cpp:56-57   if (this->arrows) fprintf(this->f, " marker-end=\"url(#endArrow)\"");
        if arrows {
            let _ = write!(f, " marker-end=\"url(#endArrow)\"");
        }
        // SVG.cpp:58   fprintf(this->f, "/>\n");
        let _ = write!(f, "/>\n");
    }

    // SVG.cpp:61   void SVG::draw(const ThickLine &line, const std::string &fill, const std::string &stroke, coordf_t stroke_width)
    pub fn draw_thick_line(
        &mut self,
        line: &ThickLine,
        fill: &str,
        stroke: &str,
        stroke_width: CoordF,
    ) {
        let origin = self.origin;
        // SVG.cpp:63   Vec2d dir(line.b(0)-line.a(0), line.b(1)-line.a(1));
        let dir = Vec2d::new(
            (line.b.x() - line.a.x()) as CoordF,
            (line.b.y() - line.a.y()) as CoordF,
        );
        // SVG.cpp:64   Vec2d perp(-dir(1), dir(0));
        let perp = Vec2d::new(-dir.y(), dir.x());
        // SVG.cpp:65   coordf_t len = sqrt(perp(0)*perp(0) + perp(1)*perp(1));
        let len: CoordF = (perp.x() * perp.x() + perp.y() * perp.y()).sqrt();
        // SVG.cpp:66   coordf_t da  = coordf_t(0.5)*line.a_width/len;
        let da: CoordF = 0.5 * line.a_width / len;
        // SVG.cpp:67   coordf_t db  = coordf_t(0.5)*line.b_width/len;
        let db: CoordF = 0.5 * line.b_width / len;
        let sw = if stroke_width == 0.0 {
            1.0f32
        } else {
            to_svg_coord(stroke_width as f32)
        };
        // SVG.cpp:70-77 (compute coordinates before borrowing the output file)
        let p0x = to_svg_x((line.a.x() as CoordF - da * perp.x() - origin.x() as CoordF) as f32);
        let p0y = self.to_svg_y((line.a.y() as CoordF - da * perp.y() - origin.y() as CoordF) as f32);
        let p1x = to_svg_x((line.b.x() as CoordF - db * perp.x() - origin.x() as CoordF) as f32);
        let p1y = self.to_svg_y((line.b.y() as CoordF - db * perp.y() - origin.y() as CoordF) as f32);
        let p2x = to_svg_x((line.b.x() as CoordF + db * perp.x() - origin.x() as CoordF) as f32);
        let p2y = self.to_svg_y((line.b.y() as CoordF + db * perp.y() - origin.y() as CoordF) as f32);
        let p3x = to_svg_x((line.a.x() as CoordF + da * perp.x() - origin.x() as CoordF) as f32);
        let p3y = self.to_svg_y((line.a.y() as CoordF + da * perp.y() - origin.y() as CoordF) as f32);
        let f = self.f.as_mut().unwrap();
        // SVG.cpp:68-79   fprintf(this->f, "   <polygon points=\"...");
        let _ = write!(
            f,
            "   <polygon points=\"{:.6},{:.6} {:.6},{:.6} {:.6},{:.6} {:.6},{:.6}\" style=\"fill:{}; stroke: {}; stroke-width: {:.6}\"/>\n",
            p0x, p0y, p1x, p1y, p2x, p2y, p3x, p3y,
            fill,
            stroke,
            sw
        );
    }

    // SVG.cpp:82   void SVG::draw(const Lines &lines, std::string stroke, coordf_t stroke_width)
    pub fn draw_lines(&mut self, lines: &Lines, stroke: &str, stroke_width: CoordF) {
        // SVG.cpp:84-85   for (const Line &l : lines) this->draw(l, stroke, stroke_width);
        for l in lines {
            self.draw_line(l, stroke, stroke_width);
        }
    }

    // SVG.cpp:88   void SVG::draw(const ExPolygon &expolygon, std::string fill, const float fill_opacity)
    pub fn draw_expolygon(&mut self, expolygon: &ExPolygon, fill: &str, fill_opacity: f32) {
        // SVG.cpp:90   this->fill = fill;
        self.fill = fill.to_string();
        // SVG.cpp:92   std::string d;
        let mut d = String::new();
        // SVG.cpp:93-94   for (const Polygon &p : to_polygons(expolygon)) d += this->get_path_d(p, true) + " ";
        for p in expolygon.to_polygons() {
            d += &self.get_path_d(&p.points, true);
            d += " ";
        }
        // SVG.cpp:95   this->path(d, true, 0, fill_opacity);
        self.path(&d, true, 0.0, fill_opacity);
    }

    // SVG.cpp:98   void SVG::draw_outline(const ExPolygon &expolygon, std::string stroke_outer, std::string stroke_holes, coordf_t stroke_width)
    pub fn draw_outline_expolygon(
        &mut self,
        expolygon: &ExPolygon,
        stroke_outer: &str,
        stroke_holes: &str,
        stroke_width: CoordF,
    ) {
        // SVG.cpp:100   draw_outline(expolygon.contour, stroke_outer, stroke_width);
        self.draw_outline_polygon(&expolygon.contour, stroke_outer, stroke_width);
        // SVG.cpp:101-103   for (Polygons::const_iterator it = expolygon.holes.begin(); ...) draw_outline(*it, stroke_holes, stroke_width);
        for it in expolygon.holes.iter() {
            self.draw_outline_polygon(it, stroke_holes, stroke_width);
        }
    }

    // SVG.cpp:106   void SVG::draw(const ExPolygons &expolygons, std::string fill, const float fill_opacity)
    pub fn draw_expolygons(&mut self, expolygons: &[ExPolygon], fill: &str, fill_opacity: f32) {
        // SVG.cpp:108-109   for (...) this->draw(*it, fill, fill_opacity);
        for it in expolygons {
            self.draw_expolygon(it, fill, fill_opacity);
        }
    }

    // SVG.cpp:112   void SVG::draw_outline(const ExPolygons &expolygons, std::string stroke_outer, std::string stroke_holes, coordf_t stroke_width)
    pub fn draw_outline_expolygons(
        &mut self,
        expolygons: &[ExPolygon],
        stroke_outer: &str,
        stroke_holes: &str,
        stroke_width: CoordF,
    ) {
        // SVG.cpp:114-115   for (...) draw_outline(*it, stroke_outer, stroke_holes, stroke_width);
        for it in expolygons {
            self.draw_outline_expolygon(it, stroke_outer, stroke_holes, stroke_width);
        }
    }

    // SVG.cpp:118   void SVG::draw(const Surface &surface, std::string fill, const float fill_opacity)
    pub fn draw_surface(&mut self, surface: &Surface, fill: &str, fill_opacity: f32) {
        // SVG.cpp:120   draw(surface.expolygon, fill, fill_opacity);
        self.draw_expolygon(&surface.expolygon, fill, fill_opacity);
    }

    // SVG.cpp:123   void SVG::draw_outline(const Surface &surface, std::string stroke_outer, std::string stroke_holes, coordf_t stroke_width)
    pub fn draw_outline_surface(
        &mut self,
        surface: &Surface,
        stroke_outer: &str,
        stroke_holes: &str,
        stroke_width: CoordF,
    ) {
        // SVG.cpp:125   draw_outline(surface.expolygon, stroke_outer, stroke_holes, stroke_width);
        self.draw_outline_expolygon(&surface.expolygon, stroke_outer, stroke_holes, stroke_width);
    }

    // SVG.cpp:128   void SVG::draw(const Surfaces &surfaces, std::string fill, const float fill_opacity)
    pub fn draw_surfaces(&mut self, surfaces: &Surfaces, fill: &str, fill_opacity: f32) {
        // SVG.cpp:130-131   for (...) this->draw(*it, fill, fill_opacity);
        for it in surfaces {
            self.draw_surface(it, fill, fill_opacity);
        }
    }

    // SVG.cpp:134   void SVG::draw_outline(const Surfaces &surfaces, std::string stroke_outer, std::string stroke_holes, coordf_t stroke_width)
    pub fn draw_outline_surfaces(
        &mut self,
        surfaces: &Surfaces,
        stroke_outer: &str,
        stroke_holes: &str,
        stroke_width: CoordF,
    ) {
        // SVG.cpp:136-137   for (...) draw_outline(*it, stroke_outer, stroke_holes, stroke_width);
        for it in surfaces {
            self.draw_outline_surface(it, stroke_outer, stroke_holes, stroke_width);
        }
    }

    // SVG.cpp:140   void SVG::draw(const SurfacesPtr &surfaces, std::string fill, const float fill_opacity)
    pub fn draw_surfaces_ptr(&mut self, surfaces: &SurfacesPtr<'_>, fill: &str, fill_opacity: f32) {
        // SVG.cpp:142-143   for (...) this->draw(*(*it), fill, fill_opacity);
        for it in surfaces {
            self.draw_surface(it, fill, fill_opacity);
        }
    }

    // SVG.cpp:146   void SVG::draw_outline(const SurfacesPtr &surfaces, std::string stroke_outer, std::string stroke_holes, coordf_t stroke_width)
    pub fn draw_outline_surfaces_ptr(
        &mut self,
        surfaces: &SurfacesPtr<'_>,
        stroke_outer: &str,
        stroke_holes: &str,
        stroke_width: CoordF,
    ) {
        // SVG.cpp:148-149   for (...) draw_outline(*(*it), stroke_outer, stroke_holes, stroke_width);
        for it in surfaces {
            self.draw_outline_surface(it, stroke_outer, stroke_holes, stroke_width);
        }
    }

    // SVG.cpp:152   void SVG::draw(const Polygon &polygon, std::string fill)
    pub fn draw_polygon(&mut self, polygon: &Polygon, fill: &str) {
        // SVG.cpp:154   this->fill = fill;
        self.fill = fill.to_string();
        // SVG.cpp:155   this->path(this->get_path_d(polygon, true), !fill.empty(), 0, 1.f);
        let d = self.get_path_d(&polygon.points, true);
        self.path(&d, !fill.is_empty(), 0.0, 1.0);
    }

    // SVG.cpp:158   void SVG::draw(const Polygons &polygons, std::string fill)
    pub fn draw_polygons(&mut self, polygons: &Polygons, fill: &str) {
        // SVG.cpp:160-166   for (...) { if (it->is_counter_clockwise()) this->draw(*it, fill); else this->draw(*it, "white"); }
        for it in polygons {
            // BBS
            if it.is_counter_clockwise() {
                self.draw_polygon(it, fill);
            } else {
                self.draw_polygon(it, "white");
            }
        }
    }

    // SVG.cpp:169   void SVG::draw(const Polyline &polyline, std::string stroke, coordf_t stroke_width)
    pub fn draw_polyline(&mut self, polyline: &Polyline, stroke: &str, stroke_width: CoordF) {
        // SVG.cpp:171   this->stroke = stroke;
        self.stroke = stroke.to_string();
        // SVG.cpp:172   this->path(this->get_path_d(polyline, false), false, stroke_width, 1.f);
        let d = self.get_path_d(&polyline.points, false);
        self.path(&d, false, stroke_width, 1.0);
    }

    // SVG.cpp:175   void SVG::draw(const Polylines &polylines, std::string stroke, coordf_t stroke_width)
    pub fn draw_polylines(&mut self, polylines: &Polylines, stroke: &str, stroke_width: CoordF) {
        // SVG.cpp:177-178   for (...) this->draw(*it, stroke, stroke_width);
        for it in polylines {
            self.draw_polyline(it, stroke, stroke_width);
        }
    }

    // SVG.cpp:181   void SVG::draw(const ThickLines &thicklines, const std::string &fill, const std::string &stroke, coordf_t stroke_width)
    pub fn draw_thick_lines(
        &mut self,
        thicklines: &ThickLines,
        fill: &str,
        stroke: &str,
        stroke_width: CoordF,
    ) {
        // SVG.cpp:183-184   for (...) this->draw(*it, fill, stroke, stroke_width);
        for it in thicklines {
            self.draw_thick_line(it, fill, stroke, stroke_width);
        }
    }

    // SVG.cpp:187   void SVG::draw(const ThickPolylines &polylines, const std::string &stroke, coordf_t stroke_width)
    pub fn draw_thick_polylines_stroke(
        &mut self,
        polylines: &ThickPolylines,
        stroke: &str,
        stroke_width: CoordF,
    ) {
        // SVG.cpp:189-190   for (...) this->draw((Polyline)*it, stroke, stroke_width);
        for it in polylines {
            self.draw_polyline(&it.to_polyline(), stroke, stroke_width);
        }
    }

    // SVG.cpp:193   void SVG::draw(const ThickPolylines &thickpolylines, const std::string &fill, const std::string &stroke, coordf_t stroke_width)
    pub fn draw_thick_polylines(
        &mut self,
        thickpolylines: &ThickPolylines,
        fill: &str,
        stroke: &str,
        stroke_width: CoordF,
    ) {
        // SVG.cpp:195-196   for (...) draw(it->thicklines(), fill, stroke, stroke_width);
        for it in thickpolylines {
            self.draw_thick_lines(&it.thicklines(), fill, stroke, stroke_width);
        }
    }

    // SVG.cpp:199   void SVG::draw(const Point &point, std::string fill, coord_t iradius)
    pub fn draw_point(&mut self, point: &Point, fill: &str, iradius: Coord) {
        let origin = self.origin;
        // SVG.cpp:201   float radius = (iradius == 0) ? 3.f : to_svg_coord(iradius);
        let radius = if iradius == 0 {
            3.0f32
        } else {
            to_svg_coord(iradius as f32)
        };
        // SVG.cpp:202-205   std::ostringstream svg; svg << "   <circle cx=\"" << ... ;
        let cx = to_svg_x((point.x() - origin.x()) as f32);
        let cy = self.to_svg_y((point.y() - origin.y()) as f32);
        let mut svg = String::new();
        // Mirror the C++ std::ostream default float formatting (6 significant digits).
        let _ = write!(
            svg,
            "   <circle cx=\"{}\" cy=\"{}\" r=\"{}\" style=\"stroke: none; fill: {}\" />",
            ostream_f32(cx),
            ostream_f32(cy),
            ostream_f32(radius),
            fill
        );
        // SVG.cpp:207   fprintf(this->f, "%s\n", svg.str().c_str());
        let f = self.f.as_mut().unwrap();
        let _ = write!(f, "{}\n", svg);
    }

    // SVG.cpp:210   void SVG::draw(const Points &points, std::string fill, coord_t radius)
    pub fn draw_points(&mut self, points: &[Point], fill: &str, radius: Coord) {
        // SVG.cpp:212-213   for (...) this->draw(*it, fill, radius);
        for it in points {
            self.draw_point(it, fill, radius);
        }
    }

    // SVG.cpp:216   void SVG::draw(const ClipperLib::Path &polygon, double scale, std::string stroke, coordf_t stroke_width)
    pub fn draw_clipper_path(
        &mut self,
        polygon: &[Point],
        scale: f64,
        stroke: &str,
        stroke_width: CoordF,
    ) {
        // SVG.cpp:218   this->stroke = stroke;
        self.stroke = stroke.to_string();
        // SVG.cpp:219   this->path(this->get_path_d(polygon, scale, true), false, stroke_width, 1.f);
        let d = self.get_path_d_clipper(polygon, scale, true);
        self.path(&d, false, stroke_width, 1.0);
    }

    // SVG.cpp:222   void SVG::draw(const ClipperLib::Paths &polygons, double scale, std::string stroke, coordf_t stroke_width)
    pub fn draw_clipper_paths(
        &mut self,
        polygons: &[Vec<Point>],
        scale: f64,
        stroke: &str,
        stroke_width: CoordF,
    ) {
        // SVG.cpp:224-225   for (...) draw(*it, scale, stroke, stroke_width);
        for it in polygons {
            self.draw_clipper_path(it, scale, stroke, stroke_width);
        }
    }

    // SVG.cpp:228   void SVG::draw_outline(const Polygon &polygon, std::string stroke, coordf_t stroke_width)
    pub fn draw_outline_polygon(&mut self, polygon: &Polygon, stroke: &str, stroke_width: CoordF) {
        // SVG.cpp:230   this->stroke = stroke;
        self.stroke = stroke.to_string();
        // SVG.cpp:231   this->path(this->get_path_d(polygon, true), false, stroke_width, 1.f);
        let d = self.get_path_d(&polygon.points, true);
        self.path(&d, false, stroke_width, 1.0);
    }

    // SVG.cpp:234   void SVG::draw_outline(const Polygons &polygons, std::string stroke, coordf_t stroke_width)
    pub fn draw_outline_polygons(
        &mut self,
        polygons: &Polygons,
        stroke: &str,
        stroke_width: CoordF,
    ) {
        // SVG.cpp:236-237   for (...) draw_outline(*it, stroke, stroke_width);
        for it in polygons {
            self.draw_outline_polygon(it, stroke, stroke_width);
        }
    }

    // SVG.cpp:240   void SVG::path(const std::string &d, bool fill, coordf_t stroke_width, const float fill_opacity)
    fn path(&mut self, d: &str, fill: bool, stroke_width: CoordF, fill_opacity: f32) {
        // SVG.cpp:242   float lineWidth = 0.f;
        let mut line_width = 0.0f32;
        // SVG.cpp:243-244   if (! fill) lineWidth = (stroke_width == 0) ? 2.f : to_svg_coord(stroke_width);
        if !fill {
            line_width = if stroke_width == 0.0 {
                2.0f32
            } else {
                to_svg_coord(stroke_width as f32)
            };
        }
        // SVG.cpp:250   fill ? this->fill.c_str() : "none",
        let fill_str: &str = if fill { self.fill.as_str() } else { "none" };
        // SVG.cpp:253   (this->arrows && !fill) ? " marker-end=\"url(#endArrow)\"" : "",
        let marker: &str = if self.arrows && !fill {
            " marker-end=\"url(#endArrow)\""
        } else {
            ""
        };
        let f = self.f.as_mut().unwrap();
        // SVG.cpp:246-255   fprintf(this->f, "   <path d=\"%s\" style=\"fill: %s; stroke: %s; stroke-width: %f; fill-type: evenodd\" %s fill-opacity=\"%f\" />\n", ...);
        let _ = write!(
            f,
            "   <path d=\"{}\" style=\"fill: {}; stroke: {}; stroke-width: {:.6}; fill-type: evenodd\" {} fill-opacity=\"{:.6}\" />\n",
            d,
            fill_str,
            self.stroke,
            line_width,
            marker,
            fill_opacity
        );
    }

    // SVG.cpp:258   std::string SVG::get_path_d(const MultiPoint &mp, bool closed) const
    fn get_path_d(&self, mp: &[Point], closed: bool) -> String {
        // SVG.cpp:260   std::ostringstream d;
        let mut d = String::new();
        // SVG.cpp:261   d << "M ";
        d.push_str("M ");
        // SVG.cpp:262-265   for (...) { d << to_svg_x((*p)(0) - origin(0)) << " "; d << to_svg_y((*p)(1) - origin(1)) << " "; }
        for p in mp {
            let _ = write!(d, "{} ", ostream_f32(to_svg_x((p.x() - self.origin.x()) as f32)));
            let _ = write!(
                d,
                "{} ",
                ostream_f32(self.to_svg_y((p.y() - self.origin.y()) as f32))
            );
        }
        // SVG.cpp:266   if (closed) d << "z";
        if closed {
            d.push('z');
        }
        // SVG.cpp:267   return d.str();
        d
    }

    // SVG.cpp:270   std::string SVG::get_path_d(const ClipperLib::Path &path, double scale, bool closed) const
    fn get_path_d_clipper(&self, path: &[Point], scale: f64, closed: bool) -> String {
        // SVG.cpp:272   std::ostringstream d;
        let mut d = String::new();
        // SVG.cpp:273   d << "M ";
        d.push_str("M ");
        // SVG.cpp:274-277   for (...) { d << to_svg_x(scale * p->x() - origin(0)) << " "; d << to_svg_y(scale * p->y() - origin(1)) << " "; }
        for p in path {
            let _ = write!(
                d,
                "{} ",
                ostream_f32(to_svg_x((scale * p.x() as f64 - self.origin.x() as f64) as f32))
            );
            let _ = write!(
                d,
                "{} ",
                ostream_f32(self.to_svg_y((scale * p.y() as f64 - self.origin.y() as f64) as f32))
            );
        }
        // SVG.cpp:278   if (closed) d << "z";
        if closed {
            d.push('z');
        }
        // SVG.cpp:279   return d.str();
        d
    }

    // SVG.cpp:282-283   // font_size: font-size={font_size*10}px
    //               void SVG::draw_text(const Point &pt, const char *text, const char *color, int font_size)
    pub fn draw_text(&mut self, pt: &Point, text: &str, color: &str, font_size: i32) {
        let origin = self.origin;
        let x = to_svg_x((pt.x() - origin.x()) as f32);
        let y = self.to_svg_y((pt.y() - origin.y()) as f32);
        let f = self.f.as_mut().unwrap();
        // SVG.cpp:285-289   fprintf(this->f, "<text x=\"%f\" y=\"%f\" font-family=\"sans-serif\" font-size=\"%dpx\" fill=\"%s\">%s</text>", ..., font_size*10, color, text);
        let _ = write!(
            f,
            "<text x=\"{:.6}\" y=\"{:.6}\" font-family=\"sans-serif\" font-size=\"{}px\" fill=\"{}\">{}</text>",
            x,
            y,
            font_size * 10,
            color,
            text
        );
    }

    // SVG.cpp:292   void SVG::draw_legend(const Point &pt, const char *text, const char *color)
    pub fn draw_legend(&mut self, pt: &Point, text: &str, color: &str) {
        let origin = self.origin;
        let cx = to_svg_x((pt.x() - origin.x()) as f32);
        let cy = self.to_svg_y((pt.y() - origin.y()) as f32);
        let tx = to_svg_x((pt.x() - origin.x()) as f32) + 20.0f32;
        let ty = self.to_svg_y((pt.y() - origin.y()) as f32);
        let f = self.f.as_mut().unwrap();
        // SVG.cpp:294-298   fprintf(this->f, "<circle cx=\"%f\" cy=\"%f\" r=\"10\" fill=\"%s\"/>", ..., color);
        let _ = write!(
            f,
            "<circle cx=\"{:.6}\" cy=\"{:.6}\" r=\"10\" fill=\"{}\"/>",
            cx, cy, color
        );
        // SVG.cpp:299-303   fprintf(this->f, "<text x=\"%f\" y=\"%f\" font-family=\"sans-serif\" font-size=\"10px\" fill=\"%s\">%s</text>", ..., "black", text);
        let _ = write!(
            f,
            "<text x=\"{:.6}\" y=\"{:.6}\" font-family=\"sans-serif\" font-size=\"10px\" fill=\"{}\">{}</text>",
            tx, ty, "black", text
        );
    }

    //BBS
    // SVG.cpp:307   void SVG::draw_grid(const BoundingBox& bbox, const std::string& stroke, coordf_t stroke_width, coordf_t step)
    pub fn draw_grid(&mut self, bbox: &BoundingBox, stroke: &str, stroke_width: CoordF, step: CoordF) {
        // SVG.cpp:309-310   // draw grid
        //                   Point bbox_size = bbox.size();
        let bbox_size = bbox.size();
        // SVG.cpp:311-312   if (bbox_size(0) < step || bbox_size(1) < step) return;
        if (bbox_size.x() as CoordF) < step || (bbox_size.y() as CoordF) < step {
            return;
        }

        // SVG.cpp:314   Point start_pt(bbox.min(0), bbox.min(1));
        let mut start_pt = Point::new(bbox.min.x(), bbox.min.y());
        // SVG.cpp:315   Point end_pt(bbox.max(1), bbox.min(1));
        let mut end_pt = Point::new(bbox.max.y(), bbox.min.y());
        // SVG.cpp:316   for (coordf_t y = bbox.min(1); y <= bbox.max(1); y += step) {
        let mut y = bbox.min.y() as CoordF;
        while y <= bbox.max.y() as CoordF {
            // SVG.cpp:317   start_pt(1) = y;
            start_pt.y = y as Coord;
            // SVG.cpp:318   end_pt(1) = y;
            end_pt.y = y as Coord;
            // SVG.cpp:319   draw(Line(start_pt, end_pt), stroke, stroke_width);
            self.draw_line(&Line::new(start_pt, end_pt), stroke, stroke_width);
            y += step;
        }

        // SVG.cpp:322   start_pt(1) = bbox.min(1);
        start_pt.y = bbox.min.y();
        // SVG.cpp:323   end_pt(1) = bbox.max(1);
        end_pt.y = bbox.max.y();
        // SVG.cpp:324   for (coordf_t x = bbox.min(0); x <= bbox.max(0); x += step) {
        let mut x = bbox.min.x() as CoordF;
        while x <= bbox.max.x() as CoordF {
            // SVG.cpp:325   start_pt(0) = x;
            start_pt.x = x as Coord;
            // SVG.cpp:326   end_pt(0) = x;
            end_pt.x = x as Coord;
            // SVG.cpp:327   draw(Line(start_pt, end_pt), stroke, stroke_width);
            self.draw_line(&Line::new(start_pt, end_pt), stroke, stroke_width);
            x += step;
        }
    }

    // SVG.cpp:331   void SVG::add_comment(const std::string comment)
    pub fn add_comment(&mut self, comment: &str) {
        // SVG.cpp:333   fprintf(this->f, "<!-- %s -->\n", comment.c_str());
        let f = self.f.as_mut().unwrap();
        let _ = write!(f, "<!-- {} -->\n", comment);
    }

    // SVG.cpp:430   std::vector<ExPolygon> SVG::load(const std::string &svgFilePath)
    pub fn load(_svg_file_path: &str) -> Vec<ExPolygon> {
        // SVG.cpp:432   std::vector<ExPolygon> polygons;
        let polygons: Vec<ExPolygon> = Vec::new();
        // SVG.cpp:433-457   The pugixml-based parsing body is commented out in the C++ source,
        //                   so this function returns an empty vector.
        // SVG.cpp:458   return polygons;
        polygons
    }

    // SVG.cpp:462   void SVG::Close()
    pub fn close(&mut self) {
        // SVG.cpp:464   fprintf(this->f, "</svg>\n");
        if let Some(f) = self.f.as_mut() {
            let _ = write!(f, "</svg>\n");
        }
        // SVG.cpp:465-466   fclose(this->f); this->f = NULL;
        self.f = None;
        // SVG.cpp:467   //    printf("SVG written to %s\n", this->filename.c_str());
    }

    // SVG.cpp:470   void SVG::export_expolygons(const char *path, const BoundingBox &bbox, const Slic3r::ExPolygons &expolygons, std::string stroke_outer, std::string stroke_holes, coordf_t stroke_width)
    pub fn export_expolygons_bbox(
        path: &str,
        bbox: &BoundingBox,
        expolygons: &[ExPolygon],
        stroke_outer: &str,
        stroke_holes: &str,
        stroke_width: CoordF,
    ) {
        // SVG.cpp:472   SVG svg(path, bbox);
        let mut svg = SVG::new_bbox_default(path, bbox);
        // SVG.cpp:473   svg.draw(expolygons);
        svg.draw_expolygons(expolygons, "grey", 1.0);
        // SVG.cpp:474   svg.draw_outline(expolygons, stroke_outer, stroke_holes, stroke_width);
        svg.draw_outline_expolygons(expolygons, stroke_outer, stroke_holes, stroke_width);
        // SVG.cpp:475   svg.Close();
        svg.close();
    }

    // SVG.hpp:99-100   export_expolygons(path, expolygons, ...) { export_expolygons(path, get_extents(expolygons), expolygons, ...); }
    pub fn export_expolygons(
        path: &str,
        expolygons: &[ExPolygon],
        stroke_outer: &str,
        stroke_holes: &str,
        stroke_width: CoordF,
    ) {
        SVG::export_expolygons_bbox(
            path,
            &get_extents_expolygons(expolygons),
            expolygons,
            stroke_outer,
            stroke_holes,
            stroke_width,
        );
    }

    // SVG.cpp:478-484 (comment block)
    // Paint the expolygons in the order they are presented, thus the latter overwrites the former expolygon.
    // 1) Paint all areas with the provided ExPolygonAttributes::color_fill and ExPolygonAttributes::fill_opacity.
    // 2) Optionally paint outlines of the areas if ExPolygonAttributes::outline_width > 0.
    //    Paint with ExPolygonAttributes::color_contour and ExPolygonAttributes::color_holes.
    //    If color_contour is empty, color_fill is used. If color_hole is empty, color_contour is used.
    // 3) Optionally paint points of all expolygon contours with ExPolygonAttributes::radius_points if radius_points > 0.
    // 4) Paint ExPolygonAttributes::legend into legend using the ExPolygonAttributes::color_fill if legend is not empty.
    // SVG.cpp:485   void SVG::export_expolygons(const char *path, const std::vector<std::pair<Slic3r::ExPolygons, ExPolygonAttributes>> &expolygons_with_attributes)
    pub fn export_expolygons_with_attributes(
        path: &str,
        expolygons_with_attributes: &[(Vec<ExPolygon>, ExPolygonAttributes)],
    ) {
        // SVG.cpp:487-488   if (expolygons_with_attributes.empty()) return;
        if expolygons_with_attributes.is_empty() {
            return;
        }

        // SVG.cpp:490   size_t num_legend = std::count_if(..., [](const auto &v){ return ! v.second.legend.empty(); });
        let num_legend = expolygons_with_attributes
            .iter()
            .filter(|v| !v.1.legend.is_empty())
            .count();
        // SVG.cpp:491-492   // Format in num_columns.
        //                   size_t num_columns = 3;
        let num_columns: usize = 3;
        // SVG.cpp:493-494   // Width of the column.
        //                   coord_t step_x = scale_(20.);
        let step_x: Coord = scale(20.0);
        // SVG.cpp:495   Point legend_size(scale_(1.) + num_columns * step_x, scale_(0.4 + 1.3 * (num_legend + num_columns - 1) / num_columns));
        // NOTE: C++ `scale_(val)` is the macro `((val) / SCALING_FACTOR)` (libslic3r.h:81),
        // which performs a raw double divide (== `val * 100000.0`) followed by an implicit
        // truncating conversion to `coord_t`; it does NOT round like the crate `scale()`.
        // Also: the y expression `1.3 * (num_legend + num_columns - 1) / num_columns` is
        // evaluated in floating point (operator `*`/`/` left-assoc, `1.3` promotes the whole
        // chain to double), so this is a double division, NOT integer division.
        // crate::SCALING_FACTOR == 100000.0 == 1.0 / 0.00001, so `v * crate::SCALING_FACTOR`
        // reproduces the C++ `(v) / SCALING_FACTOR` macro exactly.
        let scale_trunc = |v: CoordF| -> Coord { (v * crate::SCALING_FACTOR) as Coord };
        let legend_size = Point::new(
            scale_trunc(1.0) + num_columns as Coord * step_x,
            scale_trunc(
                0.4 + 1.3 * (num_legend + num_columns - 1) as CoordF / num_columns as CoordF,
            ),
        );

        // SVG.cpp:497   BoundingBox bbox = get_extents(expolygons_with_attributes.front().first);
        let mut bbox = get_extents_expolygons(&expolygons_with_attributes[0].0);
        // SVG.cpp:498-499   for (size_t i = 0; i < ...; ++i) bbox.merge(get_extents(expolygons_with_attributes[i].first));
        for i in 0..expolygons_with_attributes.len() {
            bbox.merge(&get_extents_expolygons(&expolygons_with_attributes[i].0));
        }
        // SVG.cpp:500-501   // Legend y.
        //                   coord_t pos_y  = bbox.max.y() + scale_(1.5);
        let mut pos_y: Coord = bbox.max.y() + scale(1.5);
        // SVG.cpp:502   bbox.merge(Point(std::max(bbox.min.x() + legend_size.x(), bbox.max.x()), bbox.max.y() + legend_size.y()));
        bbox.merge_point(Point::new(
            std::cmp::max(bbox.min.x() + legend_size.x(), bbox.max.x()),
            bbox.max.y() + legend_size.y(),
        ));

        // SVG.cpp:504   SVG svg(path, bbox);
        let mut svg = SVG::new_bbox_default(path, &bbox);
        // SVG.cpp:505-506   for (const auto &exp_with_attr : ...) svg.draw(exp_with_attr.first, exp_with_attr.second.color_fill, exp_with_attr.second.fill_opacity);
        for exp_with_attr in expolygons_with_attributes {
            svg.draw_expolygons(
                &exp_with_attr.0,
                &exp_with_attr.1.color_fill,
                exp_with_attr.1.fill_opacity,
            );
        }
        // SVG.cpp:507-517   for (...) { if (outline_width > 0) { ... svg.draw_outline(...); } }
        for exp_with_attr in expolygons_with_attributes {
            if exp_with_attr.1.outline_width > 0 {
                // SVG.cpp:509-511   std::string color_contour = ...; if (color_contour.empty()) color_contour = color_fill;
                let mut color_contour = exp_with_attr.1.color_contour.clone();
                if color_contour.is_empty() {
                    color_contour = exp_with_attr.1.color_fill.clone();
                }
                // SVG.cpp:512-514   std::string color_holes = ...; if (color_holes.empty()) color_holes = color_contour;
                let mut color_holes = exp_with_attr.1.color_holes.clone();
                if color_holes.is_empty() {
                    color_holes = color_contour.clone();
                }
                // SVG.cpp:515   svg.draw_outline(exp_with_attr.first, color_contour, color_holes, exp_with_attr.second.outline_width);
                svg.draw_outline_expolygons(
                    &exp_with_attr.0,
                    &color_contour,
                    &color_holes,
                    exp_with_attr.1.outline_width as CoordF,
                );
            }
        }
        // SVG.cpp:518-521   for (...) if (radius_points > 0) for (...) svg.draw(to_points(expoly), color_points, radius_points);
        for exp_with_attr in expolygons_with_attributes {
            if exp_with_attr.1.radius_points > 0 {
                for expoly in &exp_with_attr.0 {
                    svg.draw_points(
                        &to_points_expoly(expoly),
                        &exp_with_attr.1.color_points,
                        exp_with_attr.1.radius_points,
                    );
                }
            }
        }

        // SVG.cpp:523-524   // Export legend.
        //                   // 1st row
        // SVG.cpp:525   coord_t pos_x0 = bbox.min.x() + scale_(1.);
        let pos_x0: Coord = bbox.min.x() + scale(1.0);
        // SVG.cpp:526   coord_t pos_x  = pos_x0;
        let mut pos_x: Coord = pos_x0;
        // SVG.cpp:527   size_t  i_legend = 0;
        let mut i_legend: usize = 0;
        // SVG.cpp:528-538   for (...) { if (!legend.empty()) { svg.draw_legend(...); if ((++ i_legend) % num_columns == 0) {...} else {...} } }
        for exp_with_attr in expolygons_with_attributes {
            if !exp_with_attr.1.legend.is_empty() {
                svg.draw_legend(
                    &Point::new(pos_x, pos_y),
                    &exp_with_attr.1.legend,
                    &exp_with_attr.1.color_fill,
                );
                i_legend += 1;
                if i_legend % num_columns == 0 {
                    pos_x = pos_x0;
                    pos_y += scale(1.3);
                } else {
                    pos_x += step_x;
                }
            }
        }
        // SVG.cpp:539   svg.Close();
        svg.close();
    }
}

// SVG.hpp:35   ~SVG() { if (f != NULL) Close(); }
impl Drop for SVG {
    fn drop(&mut self) {
        if self.f.is_some() {
            self.close();
        }
    }
}

// Mirror C++ `std::ostream operator<<(float)` default formatting: 6 significant
// digits, trailing zeros stripped (e.g. 3 not 3.000000). Used by `draw(Point)`
// and `get_path_d`, which build their output with ostringstream rather than
// fprintf("%f").
fn ostream_f32(v: f32) -> String {
    // std::ostream defaults to std::defaultfloat with precision 6 (significant digits).
    let mut s = format!("{:.*}", 6usize.saturating_sub(int_digits(v)), v);
    if s.contains('.') {
        while s.ends_with('0') {
            s.pop();
        }
        if s.ends_with('.') {
            s.pop();
        }
    }
    s
}

#[inline]
fn int_digits(v: f32) -> usize {
    let a = v.abs();
    if a < 1.0 {
        1
    } else {
        (a.log10().floor() as usize) + 1
    }
}

// SVG.cpp:336-410   Points ParseSVGPath(const std::string &pathData)
// Function to parse the SVG path data
pub fn parse_svg_path(path_data: &str) -> Vec<Point> {
    // SVG.cpp:339   Points points;
    let mut points: Vec<Point> = Vec::new();
    // SVG.cpp:340   Vec2d currentPoint = {0, 0};
    let mut current_point = Vec2d::new(0.0, 0.0);
    // SVG.cpp:341   char command = 0;
    let mut command: u8 = 0;
    // SVG.cpp:342   std::istringstream stream(pathData);
    let mut stream = TokenStream::new(path_data);

    // SVG.cpp:344   while (stream) {
    while stream.good() {
        // SVG.cpp:345-346   // Read the command or continue with the previous command
        //                   if (!std::isdigit(stream.peek()) && stream.peek() != '-' && stream.peek() != '.') { stream >> command; }
        let peek = stream.peek();
        if !peek.map(|c| c.is_ascii_digit()).unwrap_or(false)
            && peek != Some('-')
            && peek != Some('.')
        {
            if let Some(c) = stream.read_char_token() {
                command = c as u8;
            }
        }

        // SVG.cpp:348   if (command == 'M' || command == 'm') { // Move to
        if command == b'M' || command == b'm' {
            // SVG.cpp:349-352   double x, y; stream >> x; stream.ignore(1, ','); stream >> y;
            let x = stream.read_f64();
            stream.ignore_comma();
            let y = stream.read_f64();
            if let (Some(x), Some(y)) = (x, y) {
                // SVG.cpp:354-360   if (command == 'm') { relative } else { absolute }
                if command == b'm' {
                    // Relative
                    current_point.x += x;
                    current_point.y += y;
                } else {
                    // Absolute
                    current_point.x = x;
                    current_point.y = y;
                }
                // SVG.cpp:361   points.push_back(scaled<coord_t>(currentPoint));
                points.push(scaled_point(current_point));
            }
        } else if command == b'L' || command == b'l' {
            // SVG.cpp:362   } else if (command == 'L' || command == 'l') { // Line to
            // SVG.cpp:363-366   double x, y; stream >> x; stream.ignore(1, ','); stream >> y;
            let x = stream.read_f64();
            stream.ignore_comma();
            let y = stream.read_f64();
            if let (Some(x), Some(y)) = (x, y) {
                // SVG.cpp:368-374   if (command == 'l') { relative } else { absolute }
                if command == b'l' {
                    current_point.x += x;
                    current_point.y += y;
                } else {
                    current_point.x = x;
                    current_point.y = y;
                }
                // SVG.cpp:375   points.push_back(scaled<coord_t>(currentPoint));
                points.push(scaled_point(current_point));
            }
        } else if command == b'Z' || command == b'z' {
            // SVG.cpp:376   } else if (command == 'Z' || command == 'z') { // Close path
            // SVG.cpp:377-379   if (!points.empty()) points.push_back(points.front());
            if !points.is_empty() {
                points.push(points[0]); // Close the polygon by returning to the start
            }
        } else if command == b'H' || command == b'h' {
            // SVG.cpp:380   } else if (command == 'H' || command == 'h') { // Horizontal line
            // SVG.cpp:381-382   double x; stream >> x;
            let x = stream.read_f64();
            if let Some(x) = x {
                // SVG.cpp:384-388   if (command == 'h') { relative } else { absolute }
                if command == b'h' {
                    current_point.x += x;
                } else {
                    current_point.x = x;
                }
                // SVG.cpp:389   points.push_back(scaled<coord_t>(currentPoint));
                points.push(scaled_point(current_point));
            }
        } else if command == b'V' || command == b'v' {
            // SVG.cpp:390   } else if (command == 'V' || command == 'v') { // Vertical line
            // SVG.cpp:391-392   double y; stream >> y;
            let y = stream.read_f64();
            if let Some(y) = y {
                // SVG.cpp:394-398   if (command == 'v') { relative } else { absolute }
                if command == b'v' {
                    current_point.y += y;
                } else {
                    current_point.y = y;
                }
                // SVG.cpp:399   points.push_back(scaled<coord_t>(currentPoint));
                points.push(scaled_point(current_point));
            }
        } else if command == b'z' {
            // SVG.cpp:400   } else if (command == 'z') {
            // SVG.cpp:401-403   if (!points.empty()) points.push_back(points.front());
            if !points.is_empty() {
                points.push(points[0]); // Close path
            }
        } else {
            // SVG.cpp:404-406   } else { stream.ignore(1); }
            stream.ignore(1); // Skip invalid commands or extra spaces
        }
    }

    // SVG.cpp:409   return points;
    points
}

// SVG.cpp:413   ExPolygon ConvertToExPolygon(const std::vector<std::string> &svgPaths)
// Convert SVG path to ExPolygon
pub fn convert_to_expolygon(svg_paths: &[String]) -> ExPolygon {
    // SVG.cpp:415   ExPolygon exPolygon;
    let mut ex_polygon = ExPolygon::default();

    // SVG.cpp:417-424   for (const auto &pathData : svgPaths) { ... }
    for path_data in svg_paths {
        // SVG.cpp:418   auto points = ParseSVGPath(pathData);
        let points = parse_svg_path(path_data);
        // SVG.cpp:419-423   if (exPolygon.contour.empty()) { contour } else { holes }
        if ex_polygon.contour.points.is_empty() {
            ex_polygon.contour.points = points; // First path is outer
        } else {
            ex_polygon.holes.push(Polygon { points }); // Subsequent paths are holes
        }
    }

    // SVG.cpp:426   return exPolygon;
    ex_polygon
}

// SVG.cpp:544   void to_json(nlohmann::json &j, const Point &p) { j = nlohmann::json{p.x(), p.y()}; }
// JSON serialization for Point using compact format [x, y]
pub fn point_to_json(p: &Point) -> serde_json::Value {
    serde_json::json!([p.x(), p.y()])
}

// SVG.cpp:546   void from_json(const nlohmann::json &j, Point &p)
pub fn point_from_json(j: &serde_json::Value) -> crate::Result<Point> {
    // SVG.cpp:548-553   if (j.is_array() && j.size() == 2) { ... } else { throw ... }
    if j.is_array() && j.as_array().map(|a| a.len()).unwrap_or(0) == 2 {
        let arr = j.as_array().unwrap();
        let x = arr[0].as_i64().ok_or_else(|| {
            crate::Error::Config("Invalid Point JSON format. Expected [x, y].".to_string())
        })?;
        let y = arr[1].as_i64().ok_or_else(|| {
            crate::Error::Config("Invalid Point JSON format. Expected [x, y].".to_string())
        })?;
        Ok(Point::new(x as Coord, y as Coord))
    } else {
        Err(crate::Error::Config(
            "Invalid Point JSON format. Expected [x, y].".to_string(),
        ))
    }
}

// SVG.cpp:557   void to_json(nlohmann::json &j, const Polygon &polygon)
// Serialization for Polygon
pub fn polygon_to_json(polygon: &Polygon) -> serde_json::Value {
    // SVG.cpp:559   j = nlohmann::json::array();
    let mut j = Vec::new();
    // SVG.cpp:560-562   for (const auto &point : polygon.points) j.push_back(point);
    for point in &polygon.points {
        j.push(point_to_json(point)); // Push each point (serialized as [x, y])
    }
    serde_json::Value::Array(j)
}

// SVG.cpp:565   void from_json(const nlohmann::json &j, Polygon &polygon)
pub fn polygon_from_json(j: &serde_json::Value) -> crate::Result<Polygon> {
    // SVG.cpp:567-572   if (j.is_array()) { ... } else { throw ... }
    if let Some(arr) = j.as_array() {
        // SVG.cpp:568   polygon.clear();
        let mut polygon = Polygon::default();
        // SVG.cpp:569   for (const auto &item : j) polygon.append(item.get<Point>());
        for item in arr {
            polygon.points.push(point_from_json(item)?);
        }
        Ok(polygon)
    } else {
        Err(crate::Error::Config(
            "Invalid Polygon JSON format. Expected array of points.".to_string(),
        ))
    }
}

// SVG.cpp:577   void to_json(nlohmann::json &j, const ExPolygon &exPolygon)
// Serialization for ExPolygon
pub fn expolygon_to_json(ex_polygon: &ExPolygon) -> serde_json::Value {
    // SVG.cpp:578   j = nlohmann::json{{"contour", exPolygon.contour}, {"holes", exPolygon.holes}};
    let holes: Vec<serde_json::Value> = ex_polygon.holes.iter().map(polygon_to_json).collect();
    serde_json::json!({
        "contour": polygon_to_json(&ex_polygon.contour),
        "holes": serde_json::Value::Array(holes),
    })
}

// SVG.cpp:581   void from_json(const nlohmann::json &j, ExPolygon &exPolygon)
pub fn expolygon_from_json(j: &serde_json::Value) -> crate::Result<ExPolygon> {
    // SVG.cpp:583-590   if (j.contains("contour")) { ... } else { throw ... }
    if let Some(contour) = j.get("contour") {
        let mut ex_polygon = ExPolygon::default();
        // SVG.cpp:584   j.at("contour").get_to(exPolygon.contour);
        ex_polygon.contour = polygon_from_json(contour)?;
        // SVG.cpp:585-587   if (j.contains("holes")) j.at("holes").get_to(exPolygon.holes);
        if let Some(holes) = j.get("holes") {
            if let Some(arr) = holes.as_array() {
                ex_polygon.holes.clear();
                for item in arr {
                    ex_polygon.holes.push(polygon_from_json(item)?);
                }
            }
        }
        Ok(ex_polygon)
    } else {
        Err(crate::Error::Config(
            "Invalid ExPolygon JSON format. Missing 'contour' or 'holes'.".to_string(),
        ))
    }
}

// SVG.cpp:594   void to_json(nlohmann::json &j, const std::vector<ExPolygon> &exPolygons)
// Serialization for ExPolygons
pub fn expolygons_to_json(ex_polygons: &[ExPolygon]) -> serde_json::Value {
    // SVG.cpp:596   j = nlohmann::json::array();
    let mut j = Vec::new();
    // SVG.cpp:597-599   for (const auto &exPolygon : exPolygons) j.push_back(exPolygon);
    for ex_polygon in ex_polygons {
        j.push(expolygon_to_json(ex_polygon)); // Serialize each ExPolygon
    }
    serde_json::Value::Array(j)
}

// SVG.cpp:602   void from_json(const nlohmann::json& j, std::vector<ExPolygon>& exPolygons)
pub fn expolygons_from_json(j: &serde_json::Value) -> crate::Result<Vec<ExPolygon>> {
    // SVG.cpp:604-612   if (j.is_array()) { ... } else { throw ... }
    if let Some(arr) = j.as_array() {
        // SVG.cpp:605   exPolygons.clear();
        let mut ex_polygons = Vec::new();
        // SVG.cpp:606-608   for (const auto& item : j) exPolygons.push_back(item.get<ExPolygon>());
        for item in arr {
            ex_polygons.push(expolygon_from_json(item)?);
        }
        Ok(ex_polygons)
    } else {
        Err(crate::Error::Config(
            "Invalid ExPolygons JSON format. Expected array of ExPolygons.".to_string(),
        ))
    }
}

// SVG.cpp:616   void dumpExPolygonToJson(const ExPolygon &exPolygon, const std::string &filePath)
// Function to dump ExPolygons to JSON
pub fn dump_expolygon_to_json(ex_polygon: &ExPolygon, file_path: &str) {
    // SVG.cpp:618   nlohmann::json j = exPolygon;
    let j = expolygon_to_json(ex_polygon);

    // SVG.cpp:621-626   std::ofstream file(filePath); if (!file) { cerr ...; return; } file << j.dump(4);
    match std::fs::File::create(file_path) {
        Ok(mut file) => {
            // SVG.cpp:626   file << j.dump(4); // Pretty print with 4 spaces of indentation
            let s = serde_json::to_string_pretty(&j).unwrap_or_default();
            let _ = file.write_all(s.as_bytes());
            // SVG.cpp:627   file.close();
            // SVG.cpp:629   std::cout << "ExPolygons dumped to " << filePath << "\n";
            println!("ExPolygons dumped to {}", file_path);
        }
        Err(_) => {
            // SVG.cpp:623   std::cerr << "Error: Cannot open file for writing: " << filePath << "\n";
            eprintln!("Error: Cannot open file for writing: {}", file_path);
        }
    }
}

// SVG.cpp:633   void dumpExPolygonsToJson(const std::vector<ExPolygon> &exPolygons, const std::string &filePath)
// Function to dump ExPolygons to JSON
pub fn dump_expolygons_to_json(ex_polygons: &[ExPolygon], file_path: &str) {
    // SVG.cpp:635   nlohmann::json j = exPolygons;
    let j = expolygons_to_json(ex_polygons);

    // SVG.cpp:638-643   std::ofstream file(filePath); if (!file) { cerr ...; return; } file << j.dump(4);
    match std::fs::File::create(file_path) {
        Ok(mut file) => {
            // SVG.cpp:643   file << j.dump(4); // Pretty print with 4 spaces of indentation
            let s = serde_json::to_string_pretty(&j).unwrap_or_default();
            let _ = file.write_all(s.as_bytes());
            // SVG.cpp:644   file.close();
            // SVG.cpp:646   std::cout << "ExPolygons dumped to " << filePath << "\n";
            println!("ExPolygons dumped to {}", file_path);
        }
        Err(_) => {
            // SVG.cpp:640   std::cerr << "Error: Cannot open file for writing: " << filePath << "\n";
            eprintln!("Error: Cannot open file for writing: {}", file_path);
        }
    }
}

// SVG.cpp:650   std::vector<ExPolygon> loadExPolygonsFromJson(const std::string &filePath)
// Function to load ExPolygons from JSON
pub fn load_expolygons_from_json(file_path: &str) -> Vec<ExPolygon> {
    // SVG.cpp:652   std::vector<ExPolygon> exPolygons;
    let mut ex_polygons: Vec<ExPolygon> = Vec::new();

    // SVG.cpp:654-658   std::ifstream file(filePath); if (!file) { cerr ...; return exPolygons; }
    let content = match std::fs::read_to_string(file_path) {
        Ok(c) => c,
        Err(_) => {
            // SVG.cpp:656   std::cerr << "Error: Cannot open file for reading: " << filePath << "\n";
            eprintln!("Error: Cannot open file for reading: {}", file_path);
            return ex_polygons;
        }
    };

    // SVG.cpp:660-662   std::stringstream buffer; buffer << file.rdbuf(); std::string content = buffer.str();
    // SVG.cpp:664-671   nlohmann::json j; try { j = nlohmann::json::parse(content); } catch (...) { cerr ...; return exPolygons; }
    let j: serde_json::Value = match serde_json::from_str(&content) {
        Ok(v) => v,
        Err(e) => {
            // SVG.cpp:668-669   std::cerr << "JSON parsing error: " << e.what() << std::endl;
            eprintln!("JSON parsing error: {}", e);
            return ex_polygons; // Return empty vector on failure
        }
    };

    // SVG.cpp:681-690   if (j.is_array()) { ... } else if (j.is_object()) { ... } else { throw ... }
    if j.is_array() {
        // SVG.cpp:682-684   for (const auto& item : j) exPolygons.push_back(item.get<ExPolygon>());
        for item in j.as_array().unwrap() {
            if let Ok(ep) = expolygon_from_json(item) {
                ex_polygons.push(ep);
            }
        }
    } else if j.is_object() {
        // SVG.cpp:686   exPolygons.push_back(j.get<ExPolygon>());
        if let Ok(ep) = expolygon_from_json(&j) {
            ex_polygons.push(ep);
        }
    } else {
        // SVG.cpp:689   throw std::runtime_error("Invalid ExPolygons JSON format. Expected array of ExPolygons.");
        // (Rust port surfaces this as an empty result rather than an unwinding panic.)
    }

    // SVG.cpp:692   return exPolygons;
    ex_polygons
}

// SVG.cpp:696   void dumpExPolygonsToTxt(const std::vector<ExPolygon> &exPolygons, const std::string &filePath)
// Save ExPolygons to a file
pub fn dump_expolygons_to_txt(ex_polygons: &[ExPolygon], file_path: &str) {
    // SVG.cpp:698-702   std::ofstream file(filePath); if (!file) { cerr ...; return; }
    let mut file = match std::fs::File::create(file_path) {
        Ok(f) => f,
        Err(_) => {
            // SVG.cpp:700   std::cerr << "Error: Cannot open file for writing: " << filePath << std::endl;
            eprintln!("Error: Cannot open file for writing: {}", file_path);
            return;
        }
    };

    // SVG.cpp:704-719   for (size_t i = 0; i < exPolygons.size(); ++i) { ... }
    for (i, ex_polygon) in ex_polygons.iter().enumerate() {
        // SVG.cpp:706   file << "# ExPolygon " << i + 1 << "\n";
        let _ = write!(file, "# ExPolygon {}\n", i + 1);

        // SVG.cpp:708-711   // Save the outer contour
        //                   file << "contour:"; for (...) file << " " << point.x() << " " << point.y(); file << "\n";
        let _ = write!(file, "contour:");
        for point in &ex_polygon.contour.points {
            let _ = write!(file, " {} {}", point.x(), point.y());
        }
        let _ = write!(file, "\n");

        // SVG.cpp:713-718   // Save the holes
        //                   for (const auto &hole : exPolygon.holes) { file << "hole:"; for (...) ...; file << "\n"; }
        for hole in &ex_polygon.holes {
            let _ = write!(file, "hole:");
            for point in &hole.points {
                let _ = write!(file, " {} {}", point.x(), point.y());
            }
            let _ = write!(file, "\n");
        }
    }

    // SVG.cpp:721   file.close();
    // SVG.cpp:722   std::cout << "ExPolygons saved to " << filePath << std::endl;
    println!("ExPolygons saved to {}", file_path);
}

// SVG.cpp:726   std::vector<ExPolygon> loadExPolygonsFromTxt(const std::string &filePath)
// Load ExPolygons from a file
pub fn load_expolygons_from_txt(file_path: &str) -> Vec<ExPolygon> {
    // SVG.cpp:728   std::vector<ExPolygon> exPolygons;
    let mut ex_polygons: Vec<ExPolygon> = Vec::new();

    // SVG.cpp:730-734   std::ifstream file(filePath); if (!file) { cerr ...; return exPolygons; }
    let content = match std::fs::read_to_string(file_path) {
        Ok(c) => c,
        Err(_) => {
            // SVG.cpp:732   std::cerr << "Error: Cannot open file for reading: " << filePath << std::endl;
            eprintln!("Error: Cannot open file for reading: {}", file_path);
            return ex_polygons;
        }
    };

    // SVG.cpp:736-737   std::string line; ExPolygon currentPolygon;
    let mut current_polygon = ExPolygon::default();
    // SVG.cpp:738   while (std::getline(file, line)) {
    for line in content.lines() {
        // SVG.cpp:739-746   if (line.empty() || line[0] == '#') { ... continue; }
        if line.is_empty() || line.as_bytes()[0] == b'#' {
            // Start of a new polygon
            if !current_polygon.contour.points.is_empty() || !current_polygon.holes.is_empty() {
                ex_polygons.push(current_polygon);
                current_polygon = ExPolygon::default();
            }
            continue;
        }

        // SVG.cpp:748-750   std::istringstream stream(line); std::string keyword; stream >> keyword;
        let mut stream = WordStream::new(line);
        let keyword = stream.next_word();

        // SVG.cpp:752-755   if (keyword == "contour:") { ... }
        if keyword.as_deref() == Some("contour:") {
            // SVG.cpp:753   currentPolygon.contour.clear();
            current_polygon.contour.clear();
            // SVG.cpp:754-755   coord_t x, y; while (stream >> x >> y) currentPolygon.contour.append({x, y});
            loop {
                let x = stream.next_i64();
                let y = stream.next_i64();
                match (x, y) {
                    (Some(x), Some(y)) => current_polygon
                        .contour
                        .points
                        .push(Point::new(x as Coord, y as Coord)),
                    _ => break,
                }
            }
        } else if keyword.as_deref() == Some("hole:") {
            // SVG.cpp:756   } else if (keyword == "hole:") {
            // SVG.cpp:757-760   Polygon hole; coord_t x, y; while (stream >> x >> y) hole.append({x, y}); currentPolygon.holes.push_back(hole);
            let mut hole = Polygon::default();
            loop {
                let x = stream.next_i64();
                let y = stream.next_i64();
                match (x, y) {
                    (Some(x), Some(y)) => hole.points.push(Point::new(x as Coord, y as Coord)),
                    _ => break,
                }
            }
            current_polygon.holes.push(hole);
        }
    }

    // SVG.cpp:764-765   // Add the last polygon if any
    //                   if (!currentPolygon.contour.empty() || !currentPolygon.holes.empty()) exPolygons.push_back(currentPolygon);
    if !current_polygon.contour.points.is_empty() || !current_polygon.holes.is_empty() {
        ex_polygons.push(current_polygon);
    }

    // SVG.cpp:767   file.close();
    // SVG.cpp:768   std::cout << "Loaded " << exPolygons.size() << " ExPolygons from " << filePath << std::endl;
    println!(
        "Loaded {} ExPolygons from {}",
        ex_polygons.len(),
        file_path
    );
    // SVG.cpp:769   return exPolygons;
    ex_polygons
}

// SVG.cpp:361   scaled<coord_t>(Vec2d) — scale a floating-point Vec2d to an integer Point.
#[inline]
fn scaled_point(p: Vec2d) -> Point {
    Point::new(scaled(p.x()), scaled(p.y()))
}

// Helper: ExPolygon::to_points() (used by export_expolygons_with_attributes via to_points(expoly)).
// ExPolygon.hpp: to_points returns the contour and hole vertices.
fn to_points_expoly(expoly: &ExPolygon) -> Vec<Point> {
    let mut pts = Vec::new();
    pts.extend_from_slice(&expoly.contour.points);
    for hole in &expoly.holes {
        pts.extend_from_slice(&hole.points);
    }
    pts
}

// std::istringstream emulation for ParseSVGPath: reads whitespace/comma-delimited
// floating tokens and single command characters, with `peek`/`ignore` semantics.
struct TokenStream {
    chars: Vec<char>,
    pos: usize,
    failed: bool,
}

impl TokenStream {
    fn new(s: &str) -> Self {
        Self {
            chars: s.chars().collect(),
            pos: 0,
            failed: false,
        }
    }

    // operator bool(): true while not failed and not at end-of-stream.
    fn good(&self) -> bool {
        !self.failed && self.pos < self.chars.len()
    }

    // istream::peek(): returns the next char without consuming, skipping leading
    // whitespace the way `operator>>` would NOT — peek is raw. The C++ code calls
    // peek() directly, which returns the raw next char (whitespace included).
    fn peek(&self) -> Option<char> {
        self.chars.get(self.pos).copied()
    }

    // Read a single non-whitespace char as a "command" (mirrors `stream >> char`,
    // which skips leading whitespace then reads one char).
    fn read_char_token(&mut self) -> Option<char> {
        self.skip_ws();
        if self.pos < self.chars.len() {
            let c = self.chars[self.pos];
            self.pos += 1;
            Some(c)
        } else {
            self.failed = true;
            None
        }
    }

    // `stream >> double`: skip whitespace, parse a floating-point token.
    fn read_f64(&mut self) -> Option<f64> {
        self.skip_ws();
        let start = self.pos;
        let mut end = self.pos;
        // optional sign
        if end < self.chars.len() && (self.chars[end] == '+' || self.chars[end] == '-') {
            end += 1;
        }
        let mut saw_digit = false;
        while end < self.chars.len() && self.chars[end].is_ascii_digit() {
            end += 1;
            saw_digit = true;
        }
        if end < self.chars.len() && self.chars[end] == '.' {
            end += 1;
            while end < self.chars.len() && self.chars[end].is_ascii_digit() {
                end += 1;
                saw_digit = true;
            }
        }
        // exponent
        if saw_digit && end < self.chars.len() && (self.chars[end] == 'e' || self.chars[end] == 'E')
        {
            let mut e = end + 1;
            if e < self.chars.len() && (self.chars[e] == '+' || self.chars[e] == '-') {
                e += 1;
            }
            let mut exp_digit = false;
            while e < self.chars.len() && self.chars[e].is_ascii_digit() {
                e += 1;
                exp_digit = true;
            }
            if exp_digit {
                end = e;
            }
        }
        if !saw_digit {
            self.failed = true;
            return None;
        }
        let tok: String = self.chars[start..end].iter().collect();
        self.pos = end;
        match tok.parse::<f64>() {
            Ok(v) => Some(v),
            Err(_) => {
                self.failed = true;
                None
            }
        }
    }

    // istream::ignore(1, ','): skip up to 1 char, stopping after a comma is found.
    fn ignore_comma(&mut self) {
        if self.pos < self.chars.len() {
            // ignore(1, ',') extracts at most 1 char, stopping if it is ','.
            self.pos += 1;
        }
    }

    // istream::ignore(1): skip 1 char.
    fn ignore(&mut self, n: usize) {
        for _ in 0..n {
            if self.pos < self.chars.len() {
                self.pos += 1;
            } else {
                self.failed = true;
                break;
            }
        }
    }

    fn skip_ws(&mut self) {
        while self.pos < self.chars.len() && self.chars[self.pos].is_whitespace() {
            self.pos += 1;
        }
    }
}

// std::istringstream emulation for loadExPolygonsFromTxt: whitespace-delimited
// word / integer extraction (`stream >> std::string`, `stream >> coord_t`).
struct WordStream<'a> {
    rest: &'a str,
}

impl<'a> WordStream<'a> {
    fn new(s: &'a str) -> Self {
        Self { rest: s }
    }

    fn next_word(&mut self) -> Option<String> {
        let trimmed = self.rest.trim_start();
        if trimmed.is_empty() {
            self.rest = trimmed;
            return None;
        }
        let end = trimmed
            .find(|c: char| c.is_whitespace())
            .unwrap_or(trimmed.len());
        let word = &trimmed[..end];
        self.rest = &trimmed[end..];
        Some(word.to_string())
    }

    fn next_i64(&mut self) -> Option<i64> {
        self.next_word().and_then(|w| w.parse::<i64>().ok())
    }
}

