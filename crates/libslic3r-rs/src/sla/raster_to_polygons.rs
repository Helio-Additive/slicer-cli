//! Faithful 1:1 line-by-line port of BambuStudio
//! `src/libslic3r/SLA/RasterToPolygons.cpp` (+ `SLA/RasterToPolygons.hpp`).
//!
//! C++ Reference:
//! - SLA/RasterToPolygons.hpp
//! - SLA/RasterToPolygons.cpp
//!
//! Vectorizes a rendered `RasterGrayscaleAA` back into `ExPolygons` via the
//! marching-squares contour extraction (`crate::marching_squares`, the port of
//! `MarchingSquares.hpp`), then undoes the raster transformations.

use crate::clipper_utils::union_polygons_ex;
use crate::geometry::{ExPolygon, ExPolygons, Point, Polygon, Polygons};
use crate::marching_squares::{self as marchsq, RasterTraits};
use crate::scaled;
use crate::sla::agg_raster::RasterGrayscaleAA;
use crate::triangle_mesh::Vec2i;
use crate::Coord;

// RasterToPolygons.cpp:8-25  namespace marchsq { ... }
//
// RasterToPolygons.cpp:10-11
// // Specialize this struct to register a raster type for the Marching squares alg
// template<> struct _RasterTraits<Slic3r::sla::RasterGrayscaleAA> {
//     using Rst = Slic3r::sla::RasterGrayscaleAA;
//
// In Rust the `_RasterTraits` specialization point is the
// `marchsq::RasterTraits` trait; the specialization for `RasterGrayscaleAA`
// lives in this translation unit, exactly like the C++.
impl RasterTraits for RasterGrayscaleAA {
    // RasterToPolygons.cpp:14-15
    // // The type of pixel cell in the raster
    // using ValueType = uint8_t;
    type ValueType = u8;

    // RasterToPolygons.cpp:17-18
    // // Value at a given position
    // static uint8_t get(const Rst &rst, size_t row, size_t col) { return rst.read_pixel(col, row); }
    fn get(&self, row: usize, col: usize) -> u8 {
        self.read_pixel(col, row)
    }

    // RasterToPolygons.cpp:20-21
    // // Number of rows and cols of the raster
    // static size_t rows(const Rst &rst) { return rst.resolution().height_px; }
    fn rows(&self) -> usize {
        self.resolution().height_px
    }

    // RasterToPolygons.cpp:22
    // static size_t cols(const Rst &rst) { return rst.resolution().width_px; }
    fn cols(&self) -> usize {
        self.resolution().width_px
    }
}

// RasterToPolygons.cpp:29-34
// template<class Fn> void foreach_vertex(ExPolygon &poly, Fn &&fn)
fn foreach_vertex<F: FnMut(&mut Point)>(poly: &mut ExPolygon, mut fn_: F) {
    // RasterToPolygons.cpp:31  for (auto &p : poly.contour.points) fn(p);
    for p in poly.contour.points.iter_mut() {
        fn_(p);
    }
    // RasterToPolygons.cpp:32-33
    // for (auto &h : poly.holes)
    //     for (auto &p : h.points) fn(p);
    for h in poly.holes.iter_mut() {
        for p in h.points.iter_mut() {
            fn_(p);
        }
    }
}

// RasterToPolygons.hpp:11
// ExPolygons raster_to_polygons(const RasterGrayscaleAA &rst, Vec2i windowsize = {2, 2});
// (C++ default argument `windowsize = {2, 2}`; pass `Vec2i::new(2, 2)` for the
// default.)
//
// RasterToPolygons.cpp:36-89
// ExPolygons raster_to_polygons(const RasterGrayscaleAA &rst, Vec2i windowsize)
pub fn raster_to_polygons(rst: &RasterGrayscaleAA, windowsize: Vec2i) -> ExPolygons {
    // RasterToPolygons.cpp:38
    // size_t rows = rst.resolution().height_px, cols = rst.resolution().width_px;
    let rows: usize = rst.resolution().height_px;
    let cols: usize = rst.resolution().width_px;

    // RasterToPolygons.cpp:40  if (rows < 2 || cols < 2) return {};
    if rows < 2 || cols < 2 {
        return ExPolygons::new();
    }

    // RasterToPolygons.cpp:42  Polygons polys;
    let mut polys: Polygons = Polygons::new();
    // RasterToPolygons.cpp:43  long w_rows = std::max(2l, long(windowsize.y()));
    let w_rows: i64 = std::cmp::max(2i64, windowsize.y as i64);
    // RasterToPolygons.cpp:44  long w_cols = std::max(2l, long(windowsize.x()));
    let w_cols: i64 = std::cmp::max(2i64, windowsize.x as i64);

    // RasterToPolygons.cpp:46-47
    // std::vector<marchsq::Ring> rings =
    //     marchsq::execute(rst, 128, {w_rows, w_cols});
    let rings: Vec<marchsq::Ring> = marchsq::execute(rst, 128u8, marchsq::Coord::rc(w_rows, w_cols));

    // RasterToPolygons.cpp:49  polys.reserve(rings.size());
    polys.reserve(rings.len());

    // RasterToPolygons.cpp:51  auto pxd = rst.pixel_dimensions();
    let mut pxd = rst.pixel_dimensions();
    // RasterToPolygons.cpp:52
    // pxd.w_mm = (rst.resolution().width_px * pxd.w_mm) / (rst.resolution().width_px - 1);
    pxd.w_mm =
        (rst.resolution().width_px as f64 * pxd.w_mm) / (rst.resolution().width_px - 1) as f64;
    // RasterToPolygons.cpp:53
    // pxd.h_mm = (rst.resolution().height_px * pxd.h_mm) / (rst.resolution().height_px - 1);
    pxd.h_mm =
        (rst.resolution().height_px as f64 * pxd.h_mm) / (rst.resolution().height_px - 1) as f64;

    // RasterToPolygons.cpp:55  for (const marchsq::Ring &ring : rings) {
    for ring in rings.iter() {
        // RasterToPolygons.cpp:56  Polygon poly; Points &pts = poly.points;
        let mut poly = Polygon::new();
        // RasterToPolygons.cpp:57  pts.reserve(ring.size());
        poly.points.reserve(ring.len());

        // RasterToPolygons.cpp:59-60
        // for (const marchsq::Coord &crd : ring)
        //     pts.emplace_back(scaled(crd.c * pxd.w_mm), scaled(crd.r * pxd.h_mm));
        for crd in ring.iter() {
            poly.points.push(Point::new(
                scaled(crd.c as f64 * pxd.w_mm),
                scaled(crd.r as f64 * pxd.h_mm),
            ));
        }

        // RasterToPolygons.cpp:62  polys.emplace_back(poly);
        polys.push(poly);
    }

    // RasterToPolygons.cpp:65  // reverse the raster transformations
    // RasterToPolygons.cpp:66  ExPolygons unioned = union_ex(polys);
    // FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib.
    // `union_polygons_ex` routes through the `geo` crate (geo-clipper, fixed
    // scale 1000) rather than ClipperLib at coord_t integer precision, and adds
    // a `make_canonical()` winding pass not present in the C++ `union_ex`.
    let mut unioned: ExPolygons = union_polygons_ex(&polys);
    // RasterToPolygons.cpp:67
    // coord_t width = scaled(cols * pxd.h_mm), height = scaled(rows * pxd.w_mm);
    // (NOTE, faithful to the C++: `width` uses `pxd.h_mm` and `height` uses
    // `pxd.w_mm`.)
    let width: Coord = scaled(cols as f64 * pxd.h_mm);
    let height: Coord = scaled(rows as f64 * pxd.w_mm);

    // RasterToPolygons.cpp:69  auto tr = rst.trafo();
    let tr = rst.trafo();
    // RasterToPolygons.cpp:70  for (ExPolygon &expoly : unioned) {
    for expoly in unioned.iter_mut() {
        // RasterToPolygons.cpp:71-72
        // if (tr.mirror_y)
        //     foreach_vertex(expoly, [height](Point &p) {p.y() = height - p.y(); });
        if tr.mirror_y {
            foreach_vertex(expoly, |p| p.y = height - p.y);
        }

        // RasterToPolygons.cpp:74-75
        // if (tr.mirror_x)
        //     foreach_vertex(expoly, [width](Point &p) {p.x() = width - p.x(); });
        if tr.mirror_x {
            foreach_vertex(expoly, |p| p.x = width - p.x);
        }

        // RasterToPolygons.cpp:77  expoly.translate(-tr.center_x, -tr.center_y);
        expoly.translate(Point::new(-tr.center_x, -tr.center_y));

        // RasterToPolygons.cpp:79-80
        // if (tr.flipXY)
        //     foreach_vertex(expoly, [](Point &p) { std::swap(p.x(), p.y()); });
        if tr.flip_xy {
            foreach_vertex(expoly, |p| std::mem::swap(&mut p.x, &mut p.y));
        }

        // RasterToPolygons.cpp:82  if ((tr.mirror_x + tr.mirror_y + tr.flipXY) % 2) {
        if (tr.mirror_x as i32 + tr.mirror_y as i32 + tr.flip_xy as i32) % 2 != 0 {
            // RasterToPolygons.cpp:83  expoly.contour.reverse();
            expoly.contour.reverse();
            // RasterToPolygons.cpp:84  for (auto &h : expoly.holes) h.reverse();
            for h in expoly.holes.iter_mut() {
                h.reverse();
            }
        }
    }

    // RasterToPolygons.cpp:88  return unioned;
    unioned
}
