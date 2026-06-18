//! Faithful 1:1 port of libslic3r/SLA/Pad.{hpp,cpp}
//!
//! C++ Reference:
//! - SLA/Pad.hpp (95 lines)
//! - SLA/Pad.cpp (538 lines)
//!
//! Fidelity notes:
//! - `coord_t` -> `Coord` (i64), `coordf_t` -> `f64`.
//! - The C++ free-function `scaled<coord_t>()` (libslic3r.h) performs a
//!   TRUNCATING cast (`Tout(v / Tin(SCALING_FACTOR))`), unlike `crate::scaled`
//!   which rounds; a local `scaled_trunc` mirrors the C++ semantics (same as
//!   `sla/concave_hull.rs`).
//! - C++ overload sets (`breakstick_holes`, `pad_blueprint`,
//!   `create_pad_geometry`) get distinct Rust names (`*_ex`, `*_with_height`,
//!   `*_from_blueprints`) since Rust has no overloading.
//! - The `_AroundPadSkeleton<_Intersector>` / `BelowPadSkeleton` classes only
//!   carry state during construction; they are ported as constructor functions
//!   returning the `PadSkeleton` base, with the template parameter expressed
//!   as the private `IntersectorLike` trait.
//! - `divide_blueprint` (Pad.cpp:172-193) relies on the legacy
//!   `ClipperLib::PolyTree` from `union_pt`. Per the upstream comment
//!   (ClipperUtils.cpp:961) `union_pt` performs NO union — with pftEvenOdd it
//!   merely arranges the already disjoint contours into a containment tree —
//!   so the tree levels are reconstructed here by strict containment between
//!   the (canonical, disjoint) input ExPolygons.
//! - Functions that triangulate (`walls`, `add_cavity`,
//!   `create_*_pad_geometry`, `create_pad`) return `crate::Result` because the
//!   crate's wasm-safe tesselation backend (`tesselate.rs`, earcut instead of
//!   the native GLU libtess) is fallible; the C++ counterparts cannot fail.
//!
//! FIDELITY-NOTE(F1): geo-clipper approximation vs C++ ClipperLib. Every Clipper
//!   primitive used here (`offset_polygon`, `offset_expolygons_miter_limit`,
//!   `difference`, `union_ex`, `union_polygons_ex`, plus the `offset_waffle_*`
//!   helpers in concave_hull.rs) routes through `clipper_utils.rs`, which is
//!   built on the `geo` crate (geo-clipper, fixed scale 1000) rather than
//!   ClipperLib at coord_t integer precision. This is a cross-cutting backend
//!   gap; it is NOT re-routed per-file.
//! FIDELITY-NOTE(F2): crate-wide `Coord = i64` vs C++ `coord_t = int32_t`
//!   (libslic3r.h:40). Coord is not narrowed per-file; the `(delta as f32)`
//!   narrowings below mirror the C++ `float(delta)` / `scaled<float>` casts.

use crate::bounding_box::BoundingBox;
use crate::clipper_utils::{
    difference, offset_expolygons_miter_limit, offset_polygon, union_ex, union_polygons_ex,
    OffsetJoinType,
};
use crate::geometry::{
    to_expolygons_simple, to_polygons, ExPolygon, ExPolygons, Point, Point3F, Points, Polygon,
    Polygons, Vec2d,
};
use crate::libslic3r::{EPSILON, SCALING_FACTOR};
use crate::mt_utils::grid_f32;
use crate::normal_utils::{indexed_triangle_set, StlTriangleVertexIndices, StlVertex};
use crate::sla::concave_hull::{offset_waffle_style_ex, ConcaveHull};
use crate::sla::spat_index::{BoxIndex, BoxIndexEl, QueryType};
use crate::tesselate::{triangulate_expolygon_3d, Vec3d as TessVec3d, NORMALS_DOWN, NORMALS_UP};
use crate::triangle_mesh::{bounding_box, its_merge, its_merge_pointf3s};
use crate::triangle_mesh_slicer::{slice_mesh_ex_its, MeshSlicingParamsEx};
use crate::triangulate_wall::triangulate_wall;
use crate::{Coord, Result};

// Pad.cpp:24-26
// //! macro used to mark string used at localization, return same string
// #define L(s) Slic3r::I18N::translate(s)
#[inline]
fn l(s: &str) -> String {
    crate::i18n::translate(s)
}

/// Pad.hpp:20 — `using ThrowOnCancel = std::function<void(void)>;`
pub type ThrowOnCancel<'a> = &'a dyn Fn();

/// libslic3r.h `scaled<coord_t>()`: `Tout(v / Tin(SCALING_FACTOR))`.
/// NOTE: this is a truncating cast (toward zero), unlike `crate::scaled()`
/// which rounds; Pad.cpp:78-82/141/366-367/396 rely on the C++ semantics.
#[inline]
fn scaled_trunc(v: f64) -> Coord {
    (v / SCALING_FACTOR) as Coord
}

/// `its_merge(indexed_triangle_set &, const Pointf3s &)` adapter: the crate's
/// tesselator returns its own `Vec3d` (tesselate.rs); convert to `Point3F`
/// (= C++ `Vec3d` / `Pointf3s` element) for `its_merge_pointf3s`.
#[inline]
fn to_pointf3s(v: Vec<TessVec3d>) -> Vec<Point3F> {
    v.into_iter().map(|p| Point3F::new(p.x, p.y, p.z)).collect()
}

// Pad.cpp:28  namespace Slic3r { namespace sla {
// Pad.cpp:30  namespace {

// Pad.cpp:32-43
// indexed_triangle_set walls(
//     const Polygon &lower, const Polygon &upper,
//     double lower_z_mm, double upper_z_mm)
fn walls(
    lower: &Polygon,
    upper: &Polygon,
    lower_z_mm: f64,
    upper_z_mm: f64,
) -> Result<indexed_triangle_set> {
    // Pad.cpp:38  indexed_triangle_set w;
    let mut w = indexed_triangle_set::default();
    // Pad.cpp:39-40  triangulate_wall(w.vertices, w.indices, lower, upper,
    //                                  lower_z_mm, upper_z_mm);
    let (vertices, indices) = triangulate_wall(lower, upper, lower_z_mm, upper_z_mm)?;
    w.vertices = vertices
        .iter()
        .map(|v| StlVertex::new(v.x, v.y, v.z))
        .collect();
    // The template instantiation uses `I = int` (Vec3i indices).
    w.indices = indices
        .iter()
        .map(|t| StlTriangleVertexIndices::new(t[0] as i32, t[1] as i32, t[2] as i32))
        .collect();

    // Pad.cpp:42  return w;
    Ok(w)
}

// Pad.cpp:45-51
// Same as walls() but with identical higher and lower polygons.
// inline indexed_triangle_set straight_walls(const Polygon &plate,
//                                            double lo_z, double hi_z)
#[inline]
fn straight_walls(plate: &Polygon, lo_z: f64, hi_z: f64) -> Result<indexed_triangle_set> {
    // Pad.cpp:50  return walls(plate, plate, lo_z, hi_z);
    walls(plate, plate, lo_z, hi_z)
}

// Pad.cpp:53-57
// Function to cut tiny connector cavities for a given polygon. The input poly
// will be offsetted by "padding" and small rectangle shaped cavities will be
// inserted along the perimeter in every "stride" distance. The stick rectangles
// will have a with about "stick_width". The input dimensions are in world
// measure, not the scaled clipper units.
// Pad.cpp:58-125
// void breakstick_holes(Points& pts, double padding, double stride,
//                       double stick_width, double penetration)
fn breakstick_holes(
    pts: &mut Points,
    padding: f64,
    stride: f64,
    stick_width: f64,
    penetration: f64,
) {
    // Pad.cpp:64-65
    if stride <= EPSILON || stick_width <= EPSILON || padding <= EPSILON {
        return;
    }

    // Pad.cpp:67-68
    // SVG svg("bridgestick_plate.svg");
    // svg.draw(poly);

    // Pad.cpp:70-72
    // The connector stick will be a small rectangle with dimensions
    // stick_width x (penetration + padding) to have some penetration
    // into the input polygon.

    // Pad.cpp:74-75
    let mut out: Points = Vec::with_capacity(2 * pts.len()); // output polygon points

    // Pad.cpp:77-79 — stick bottom and right edge dimensions
    // `double sbottom = scaled(stick_width);` — scaled() yields a truncated
    // coord_t which is then stored into a double.
    let sbottom: f64 = scaled_trunc(stick_width) as f64;
    let sright: f64 = scaled_trunc(penetration + padding) as f64;

    // Pad.cpp:81-83 — scaled stride distance
    let sstride: f64 = scaled_trunc(stride) as f64;
    let mut t: f64 = 0.;

    // Pad.cpp:85-87 — process pairs of vertices as an edge, start with the
    // last and first point
    // for (size_t i = pts.size() - 1, j = 0; j < pts.size(); i = j, ++j)
    // (with an empty `pts` the C++ size_t wraps but the body never runs).
    let mut i = pts.len().wrapping_sub(1);
    for j in 0..pts.len() {
        // Pad.cpp:88-89 — Get vertices and the direction vectors
        let a: Point = pts[i];
        let b: Point = pts[j];
        // Pad.cpp:90  Vec2d dir = b.cast<double>() - a.cast<double>();
        let mut dir = Vec2d::new(b.x as f64 - a.x as f64, b.y as f64 - a.y as f64);
        // Pad.cpp:91  double nrm = dir.norm();
        let nrm: f64 = dir.length();
        // Pad.cpp:92  dir /= nrm;
        dir.x /= nrm;
        dir.y /= nrm;
        // Pad.cpp:93  Vec2d dirp(-dir(Y), dir(X));
        let dirp = Vec2d::new(-dir.y, dir.x);

        // Pad.cpp:95-96 — Insert start point
        out.push(a);

        // Pad.cpp:98-99 — dodge the start point, do not make sticks on the joins
        while t < sbottom {
            t += sbottom;
        }
        // Pad.cpp:100
        let tend: f64 = nrm - sbottom;

        // Pad.cpp:102 — insert the stick on the polygon perimeter
        while t < tend {
            // Pad.cpp:104-106 — calculate the stick rectangle vertices and
            // insert them into the output.
            // `(v).cast<coord_t>()` truncates toward zero, as does Rust `as`.
            let p1 = Point::new(a.x + (t * dir.x) as Coord, a.y + (t * dir.y) as Coord);
            // Pad.cpp:107
            let p2 = Point::new(
                p1.x + (sright * dirp.x) as Coord,
                p1.y + (sright * dirp.y) as Coord,
            );
            // Pad.cpp:108
            let p3 = Point::new(
                p2.x + (sbottom * dir.x) as Coord,
                p2.y + (sbottom * dir.y) as Coord,
            );
            // Pad.cpp:109  Point p4 = p3 + (sright * -dirp).cast<coord_t>();
            let p4 = Point::new(
                p3.x + (sright * -dirp.x) as Coord,
                p3.y + (sright * -dirp.y) as Coord,
            );
            // Pad.cpp:110  out.insert(out.end(), {p1, p2, p3, p4});
            out.extend_from_slice(&[p1, p2, p3, p4]);

            // Pad.cpp:112-113 — continue along the perimeter
            t += sstride;
        }

        // Pad.cpp:116
        t -= nrm;

        // Pad.cpp:118-119 — Insert edge endpoint
        out.push(b);

        // (loop increment: i = j, ++j)
        i = j;
    }

    // Pad.cpp:122-124 — move the new points
    out.shrink_to_fit();
    std::mem::swap(pts, &mut out);
}

// Pad.cpp:127-137
// template<class...Args>
// ExPolygons breakstick_holes(const ExPolygons &input, Args...args)
// (the sole instantiation passes the four doubles of the function above;
//  distinct Rust name since Rust has no overloading)
fn breakstick_holes_ex(
    input: &ExPolygons,
    padding: f64,
    stride: f64,
    stick_width: f64,
    penetration: f64,
) -> ExPolygons {
    // Pad.cpp:130  ExPolygons ret = input;
    let mut ret = input.clone();
    // Pad.cpp:131-134
    for p in &mut ret {
        breakstick_holes(&mut p.contour.points, padding, stride, stick_width, penetration);
        for h in &mut p.holes {
            breakstick_holes(&mut h.points, padding, stride, stick_width, penetration);
        }
    }

    // Pad.cpp:136
    ret
}

// Pad.cpp:139-142
// static inline coord_t get_waffle_offset(const PadConfig &c)
#[inline]
fn get_waffle_offset(c: &PadConfig) -> Coord {
    // Pad.cpp:141  return scaled(c.brim_size_mm + c.wing_distance());
    scaled_trunc(c.brim_size_mm + c.wing_distance())
}

// Pad.cpp:144-147
// static inline double get_merge_distance(const PadConfig &c)
#[inline]
fn get_merge_distance(c: &PadConfig) -> f64 {
    // Pad.cpp:146  return 2. * (1.8 * c.wall_thickness_mm) + c.max_merge_dist_mm;
    2. * (1.8 * c.wall_thickness_mm) + c.max_merge_dist_mm
}

// Pad.cpp:149-164
// Part of the pad configuration that is used for 3D geometry generation
// struct PadConfig3D
struct PadConfig3D {
    // Pad.cpp:151  double thickness, height, wing_height, slope;
    thickness: f64,
    height: f64,
    wing_height: f64,
    slope: f64,
}

impl PadConfig3D {
    // Pad.cpp:153-158  explicit PadConfig3D(const PadConfig &cfg2d)
    fn new(cfg2d: &PadConfig) -> Self {
        Self {
            // Pad.cpp:154  thickness{cfg2d.wall_thickness_mm}
            thickness: cfg2d.wall_thickness_mm,
            // Pad.cpp:155  height{cfg2d.full_height()}
            height: cfg2d.full_height(),
            // Pad.cpp:156  wing_height{cfg2d.wall_height_mm}
            wing_height: cfg2d.wall_height_mm,
            // Pad.cpp:157  slope{cfg2d.wall_slope}
            slope: cfg2d.wall_slope,
        }
    }

    // Pad.cpp:160-163
    // inline double bottom_offset() const
    #[inline]
    fn bottom_offset(&self) -> f64 {
        // Pad.cpp:162  return (thickness + wing_height) / std::tan(slope);
        (self.thickness + self.wing_height) / self.slope.tan()
    }
}

// Pad.cpp:166-170
// Outer part of the skeleton is used to generate the waffled edges of the pad.
// Inner parts will not be waffled or offsetted. Inner parts are only used if
// pad is generated around the object and correspond to holes and inner polygons
// in the model blueprint.
// struct PadSkeleton { ExPolygons inner, outer; };
#[derive(Default)]
struct PadSkeleton {
    inner: ExPolygons,
    outer: ExPolygons,
}

// Pad.cpp:172-193
// PadSkeleton divide_blueprint(const ExPolygons &bp)
fn divide_blueprint(bp: &ExPolygons) -> PadSkeleton {
    // Pad.cpp:174  ClipperLib::PolyTree ptree = union_pt(bp);
    //
    // `union_pt` (ClipperUtils.cpp:966) converts the ExPolygons into a
    // ClipperLib::PolyTree with pftEvenOdd; per its upstream comment
    // (ClipperUtils.cpp:961) NO union is performed and non-intersecting
    // contours are not modified. The input `bp` is the canonical, disjoint
    // output of `diff_ex` (Pad.cpp:271), so the PolyTree merely encodes the
    // containment hierarchy of the ExPolygons. The legacy PolyTree type is not
    // available in this crate; the hierarchy is reconstructed by strict
    // containment between the ExPolygons:
    //   - an ExPolygon contained in no other is a top-level node: its contour
    //     plus first-level hole children form one `outer` entry
    //     (Pad.cpp:180-189);
    //   - an ExPolygon contained in k >= 1 others sits below a hole level;
    //     `traverse_pt(child->Childs, &ret.inner)` (Pad.cpp:186, recursive
    //     ClipperUtils.hpp:625-650) flattens every such contour+holes pair
    //     into `ret.inner`.
    let mut ret = PadSkeleton::default();
    // Pad.cpp:177-178  ret.inner.reserve(...); ret.outer.reserve(...);
    ret.inner.reserve(bp.len());
    ret.outer.reserve(bp.len());

    // `union_pt(bp)` (ClipperUtils.cpp:966) flattens every ExPolygon's contour
    // and holes into one Clipper subject and, under pftEvenOdd, builds a
    // containment PolyTree without performing any union (the inputs are the
    // disjoint output of diff_ex). The tree levels alternate solid/hole by
    // even-odd parity of the nesting depth:
    //   depth 0  -> top-level solid node            (Pad.cpp:181 node)
    //   depth 1  -> hole of the enclosing depth-0   (Pad.cpp:184 child)
    //   depth 2  -> solid node inside that hole, fed to traverse_pt -> inner
    //   depth 3  -> hole of the depth-2 solid, etc.
    // Rebuild the same hierarchy from the flattened rings by ray-cast point
    // containment, then assemble depth-0 nodes (with their depth-1 holes) into
    // `outer` (Pad.cpp:180-189) and depth-2 nodes (with their depth-3 holes)
    // and deeper even levels into `inner` (Pad.cpp:186 traverse_pt recursion).

    // Flatten the rings: (ring, owning ExPolygon contour-or-hole). Hole rings
    // are reversed by diff_ex (clockwise) but orientation is irrelevant to the
    // even-odd containment test used below.
    let mut rings: Vec<&Polygon> = Vec::new();
    for poly in bp {
        rings.push(&poly.contour);
        for h in &poly.holes {
            rings.push(h);
        }
    }
    let n = rings.len();

    // depth[i] = number of OTHER rings strictly containing ring i's first
    // vertex == the node's nesting level in the even-odd PolyTree.
    let mut depth = vec![0usize; n];
    for i in 0..n {
        if rings[i].points.is_empty() {
            continue;
        }
        let p = &rings[i].points[0];
        for (j, ring) in rings.iter().enumerate() {
            if i != j && ring.contains(p) {
                depth[i] += 1;
            }
        }
    }

    // immediate_children(parent): rings whose depth == depth[parent] + 1 and
    // that are contained by ring `parent` (their first vertex lies inside).
    let immediate_children = |parent: usize| -> Vec<usize> {
        let mut out = Vec::new();
        if rings[parent].points.is_empty() {
            return out;
        }
        for (k, ring) in rings.iter().enumerate() {
            if k != parent
                && depth[k] == depth[parent] + 1
                && !ring.points.is_empty()
                && rings[parent].contains(&ring.points[0])
            {
                out.push(k);
            }
        }
        out
    };

    // Assemble a solid node at `idx` into one ExPolygon: contour = node ring,
    // holes = immediate (depth+1) children. Recurse into the holes' children
    // (depth+2 solids) which become the next-level entries.
    fn assemble<F: Fn(usize) -> Vec<usize>>(
        idx: usize,
        rings: &[&Polygon],
        children_of: &F,
    ) -> (ExPolygon, Vec<usize>) {
        // Pad.cpp:181  poly.contour.points = std::move(node->Contour);
        let mut poly = ExPolygon::new(rings[idx].clone());
        let mut grandchildren = Vec::new();
        for hole in children_of(idx) {
            // Pad.cpp:184  poly.holes.emplace_back(std::move(child->Contour));
            poly.holes.push(rings[hole].clone());
            // Pad.cpp:186  traverse_pt(child->Childs, &ret.inner);
            grandchildren.extend(children_of(hole));
        }
        (poly, grandchildren)
    }

    // Pad.cpp:180-189 — top-level (depth 0) nodes -> outer; their depth-2
    // grandchildren begin the inner recursion.
    let mut inner_queue: Vec<usize> = Vec::new();
    for (idx, _) in rings.iter().enumerate() {
        if depth[idx] == 0 {
            let (poly, grandchildren) = assemble(idx, &rings, &immediate_children);
            ret.outer.push(poly);
            inner_queue.extend(grandchildren);
        }
    }

    // Pad.cpp:625-650 traverse_pt(ExPolygons) — every even-depth solid below a
    // hole becomes a flat `inner` entry (contour + immediate holes), recursing
    // through the odd hole levels.
    while let Some(idx) = inner_queue.pop() {
        let (poly, grandchildren) = assemble(idx, &rings, &immediate_children);
        ret.inner.push(poly);
        inner_queue.extend(grandchildren);
    }

    // Pad.cpp:192
    ret
}

/// Common interface of `Intersector` (Pad.cpp:197) and `DummyIntersector`
/// (Pad.cpp:229), standing in for the C++ `_Intersector` template parameter
/// of `_AroundPadSkeleton`.
trait IntersectorLike: Default {
    fn add(&mut self, ep: &ExPolygon);
    fn intersects(&self, poly: &ExPolygon) -> bool;
}

// Pad.cpp:195-226
// A helper class for storing polygons and maintaining a spatial index of their
// bounding boxes.
// class Intersector
#[derive(Default)]
struct Intersector {
    // Pad.cpp:198  BoxIndex m_index;
    m_index: BoxIndex,
    // Pad.cpp:199  ExPolygons m_polys;
    m_polys: ExPolygons,
}

impl IntersectorLike for Intersector {
    // Pad.cpp:203-208 — Add a new polygon to the index
    fn add(&mut self, ep: &ExPolygon) {
        // Pad.cpp:206  m_polys.emplace_back(ep);
        self.m_polys.push(ep.clone());
        // Pad.cpp:207  m_index.insert(get_extents(ep), unsigned(m_index.size()));
        // get_extents(const ExPolygon&) == extents of the contour points.
        self.m_index.insert_box(
            BoundingBox::new_from_points_slice(&ep.contour.points),
            self.m_index.size() as u32,
        );
    }

    // Pad.cpp:210-225 — Check an arbitrary polygon for intersection with the
    // indexed polygons
    fn intersects(&self, poly: &ExPolygon) -> bool {
        // Pad.cpp:213-214 — Create a suitable query bounding box.
        let bb = BoundingBox::new_from_points_slice(&poly.contour.points);

        // Pad.cpp:216  std::vector<BoxIndexEl> qres = m_index.query(bb, BoxIndex::qtIntersects);
        let qres: Vec<BoxIndexEl> = self.m_index.query(&bb, QueryType::Intersects);

        // Pad.cpp:218-222 — Now check intersections on the actual polygons
        // (not just the boxes)
        let mut is_overlap = false;
        let mut qit = qres.iter();
        while !is_overlap {
            match qit.next() {
                Some(el) => is_overlap = is_overlap || poly.overlaps(&self.m_polys[el.1 as usize]),
                None => break,
            }
        }

        // Pad.cpp:224
        is_overlap
    }
}

// Pad.cpp:228-233
// This dummy intersector to implement the "force pad everywhere" feature
// struct DummyIntersector
#[derive(Default)]
struct DummyIntersector;

impl IntersectorLike for DummyIntersector {
    // Pad.cpp:231  inline void add(const ExPolygon &) {}
    #[inline]
    fn add(&mut self, _ep: &ExPolygon) {}
    // Pad.cpp:232  inline bool intersects(const ExPolygon &) { return true; }
    #[inline]
    fn intersects(&self, _poly: &ExPolygon) -> bool {
        true
    }
}

// Pad.cpp:235-316
// template<class _Intersector>
// class _AroundPadSkeleton : public PadSkeleton
// (constructor-only class; ported as a generic constructor function returning
//  the PadSkeleton base, `m_intersector` being a local)
#[allow(non_snake_case)]
fn _AroundPadSkeleton<I: IntersectorLike>(
    support_blueprint: &ExPolygons,
    model_blueprint: &ExPolygons,
    cfg: &PadConfig,
    thr: ThrowOnCancel,
) -> PadSkeleton {
    // Pad.cpp:238-240 — A spatial index used to be able to efficiently find
    // intersections of support polygons with the model polygons.
    let mut m_intersector = I::default();

    // Pad.cpp:248-253
    // We need to merge the support and the model contours in a special
    // way in which the model contours have to be substracted from the
    // support contours. The pad has to have a hole in which the model can
    // fit perfectly (thus the substraction -- diff_ex). Also, the pad has
    // to be eliminated from areas where there is no need for a pad, due
    // to missing supports.

    // Pad.cpp:255
    add_supports_to_index(&mut m_intersector, support_blueprint);

    // Pad.cpp:257-260
    // auto model_bp_offs = offset_ex(model_blueprint,
    //                                scaled<float>(cfg.embed_object.object_gap_mm),
    //                                ClipperLib::jtMiter, 1);
    // `scaled<float>` keeps the C++ float narrowing of the scaled delta; the
    // crate clipper wrapper takes unscaled mm, hence the SCALING_FACTOR
    // round-trip.
    let gap_scaled_f: f32 = (cfg.embed_object.object_gap_mm / SCALING_FACTOR) as f32;
    let model_bp_offs = offset_expolygons_miter_limit(
        model_blueprint,
        gap_scaled_f as f64 * SCALING_FACTOR,
        1.0,
    );

    // Pad.cpp:262-263
    let fullcvh = wafflized_concave_hull(support_blueprint, &model_bp_offs, cfg, thr);

    // Pad.cpp:265-269
    let model_bp_sticks = breakstick_holes_ex(
        &model_bp_offs,
        cfg.embed_object.object_gap_mm,
        cfg.embed_object.stick_stride_mm,
        cfg.embed_object.stick_width_mm,
        cfg.embed_object.stick_penetration_mm,
    );

    // Pad.cpp:271  ExPolygons fullpad = diff_ex(fullcvh, model_bp_sticks);
    let fullpad = difference(&fullcvh, &model_bp_sticks);

    // Pad.cpp:273  PadSkeleton divided = divide_blueprint(fullpad);
    let mut divided = divide_blueprint(&fullpad);

    // Pad.cpp:275-276
    remove_redundant_parts(&m_intersector, &mut divided.outer);
    remove_redundant_parts(&m_intersector, &mut divided.inner);

    // Pad.cpp:278-279
    PadSkeleton {
        outer: divided.outer,
        inner: divided.inner,
    }
}

// Pad.cpp:284-288 — Add the support blueprint to the search index to be
// queried later
// void add_supports_to_index(const ExPolygons &supp_bp)
fn add_supports_to_index<I: IntersectorLike>(m_intersector: &mut I, supp_bp: &ExPolygons) {
    // Pad.cpp:287
    for ep in supp_bp {
        m_intersector.add(ep);
    }
}

// Pad.cpp:290-304 — Create the wafflized pad around all object in the scene.
// This pad doesnt have any holes yet.
// ExPolygons wafflized_concave_hull(const ExPolygons &supp_bp,
//                                   const ExPolygons &model_bp,
//                                   const PadConfig &cfg, ThrowOnCancel thr)
fn wafflized_concave_hull(
    supp_bp: &ExPolygons,
    model_bp: &ExPolygons,
    cfg: &PadConfig,
    thr: ThrowOnCancel,
) -> ExPolygons {
    // Pad.cpp:297  auto allin = reserve_vector<ExPolygon>(supp_bp.size() + model_bp.size());
    let mut allin: ExPolygons = Vec::with_capacity(supp_bp.len() + model_bp.len());

    // Pad.cpp:299  for (auto &ep : supp_bp) allin.emplace_back(ep.contour);
    for ep in supp_bp {
        allin.push(ExPolygon::new(ep.contour.clone()));
    }
    // Pad.cpp:300  for (auto &ep : model_bp) allin.emplace_back(ep.contour);
    for ep in model_bp {
        allin.push(ExPolygon::new(ep.contour.clone()));
    }

    // Pad.cpp:302  ConcaveHull cchull{allin, get_merge_distance(cfg), thr};
    let cchull = ConcaveHull::from_ex_polygons(&allin, get_merge_distance(cfg), thr);
    // Pad.cpp:303  return offset_waffle_style_ex(cchull, get_waffle_offset(cfg));
    offset_waffle_style_ex(&cchull, get_waffle_offset(cfg))
}

// Pad.cpp:306-315 — To remove parts of the pad skeleton which do not host any
// supports
// void remove_redundant_parts(ExPolygons &parts)
fn remove_redundant_parts<I: IntersectorLike>(m_intersector: &I, parts: &mut ExPolygons) {
    // Pad.cpp:309-314
    // auto endit = std::remove_if(parts.begin(), parts.end(),
    //                             [this](const ExPolygon &p) {
    //                                 return !m_intersector.intersects(p);
    //                             });
    // parts.erase(endit, parts.end());
    parts.retain(|p| m_intersector.intersects(p));
}

// Pad.cpp:318  using AroundPadSkeleton = _AroundPadSkeleton<Intersector>;
#[allow(non_snake_case)]
fn AroundPadSkeleton(
    support_blueprint: &ExPolygons,
    model_blueprint: &ExPolygons,
    cfg: &PadConfig,
    thr: ThrowOnCancel,
) -> PadSkeleton {
    _AroundPadSkeleton::<Intersector>(support_blueprint, model_blueprint, cfg, thr)
}

// Pad.cpp:319  using BrimPadSkeleton = _AroundPadSkeleton<DummyIntersector>;
#[allow(non_snake_case)]
fn BrimPadSkeleton(
    support_blueprint: &ExPolygons,
    model_blueprint: &ExPolygons,
    cfg: &PadConfig,
    thr: ThrowOnCancel,
) -> PadSkeleton {
    _AroundPadSkeleton::<DummyIntersector>(support_blueprint, model_blueprint, cfg, thr)
}

// Pad.cpp:321-338
// class BelowPadSkeleton : public PadSkeleton
// (constructor-only class, see _AroundPadSkeleton note)
#[allow(non_snake_case)]
fn BelowPadSkeleton(
    support_blueprint: &ExPolygons,
    model_blueprint: &ExPolygons,
    cfg: &PadConfig,
    thr: ThrowOnCancel,
) -> PadSkeleton {
    let mut this = PadSkeleton::default();
    // Pad.cpp:329  outer.reserve(support_blueprint.size() + model_blueprint.size());
    this.outer
        .reserve(support_blueprint.len() + model_blueprint.len());

    // Pad.cpp:331  for (auto &ep : support_blueprint) outer.emplace_back(ep.contour);
    for ep in support_blueprint {
        this.outer.push(ExPolygon::new(ep.contour.clone()));
    }
    // Pad.cpp:332  for (auto &ep : model_blueprint) outer.emplace_back(ep.contour);
    for ep in model_blueprint {
        this.outer.push(ExPolygon::new(ep.contour.clone()));
    }

    // Pad.cpp:334  ConcaveHull ochull{outer, get_merge_distance(cfg), thr};
    let ochull = ConcaveHull::from_ex_polygons(&this.outer, get_merge_distance(cfg), thr);

    // Pad.cpp:336  outer = offset_waffle_style_ex(ochull, get_waffle_offset(cfg));
    this.outer = offset_waffle_style_ex(&ochull, get_waffle_offset(cfg));
    this
}

// Pad.cpp:340-356
// Offset the contour only, leave the holes untouched
// template<class...Args>
// ExPolygon offset_contour_only(const ExPolygon &poly, coord_t delta, Args...args)
// (every call site in this file passes no extra args, i.e. the ClipperUtils
//  DefaultJoinType = jtMiter with DefaultMiterLimit = 3.0)
fn offset_contour_only(poly: &ExPolygon, delta: Coord) -> ExPolygon {
    // Pad.cpp:344  Polygons tmp = offset(poly.contour, float(delta), args...);
    // C++ narrows the scaled delta to float; the crate clipper wrapper takes
    // unscaled mm, hence the SCALING_FACTOR conversion of the narrowed value.
    let tmp: Polygons = to_polygons(&offset_polygon(
        &poly.contour,
        (delta as f32) as f64 * SCALING_FACTOR,
        OffsetJoinType::Miter,
    ));

    // Pad.cpp:346  if (tmp.empty()) return {};
    if tmp.is_empty() {
        return ExPolygon::empty();
    }

    // Pad.cpp:348-349
    let mut holes: Polygons = poly.holes.clone();
    for h in &mut holes {
        h.reverse();
    }

    // Pad.cpp:351  ExPolygons tmp2 = diff_ex(tmp, holes);
    let tmp2 = difference(&to_expolygons_simple(&tmp), &to_expolygons_simple(&holes));

    // Pad.cpp:353  if (tmp2.empty()) return {};
    if tmp2.is_empty() {
        return ExPolygon::empty();
    }

    // Pad.cpp:355  return std::move(tmp2.front());
    tmp2.into_iter().next().unwrap()
}

// Pad.cpp:358-385
// bool add_cavity(indexed_triangle_set &pad, ExPolygon &top_poly,
//                 const PadConfig3D &cfg, ThrowOnCancel thr)
fn add_cavity(
    pad: &mut indexed_triangle_set,
    top_poly: &mut ExPolygon,
    cfg: &PadConfig3D,
    thr: ThrowOnCancel,
) -> Result<bool> {
    // Pad.cpp:363
    // auto logerr = []{BOOST_LOG_TRIVIAL(error)<<"Could not create pad cavity";};
    let logerr = || log::error!("Could not create pad cavity");

    // Pad.cpp:365-369
    let wing_distance: f64 = cfg.wing_height / cfg.slope.tan();
    let delta_inner: Coord = -scaled_trunc(cfg.thickness + wing_distance);
    let delta_middle: Coord = -scaled_trunc(cfg.thickness);
    let inner_base: ExPolygon = offset_contour_only(top_poly, delta_inner);
    let middle_base: ExPolygon = offset_contour_only(top_poly, delta_middle);

    // Pad.cpp:371
    if inner_base.is_empty() || middle_base.is_empty() {
        logerr();
        return Ok(false);
    }

    // Pad.cpp:373  ExPolygons pdiff = diff_ex(top_poly, middle_base.contour);
    let pdiff = difference(
        std::slice::from_ref(top_poly),
        &[ExPolygon::new(middle_base.contour.clone())],
    );

    // Pad.cpp:375
    if pdiff.len() != 1 {
        logerr();
        return Ok(false);
    }

    // Pad.cpp:377  top_poly = pdiff.front();
    *top_poly = pdiff.into_iter().next().unwrap();

    // Pad.cpp:379
    let z_min: f64 = -cfg.wing_height;
    let z_max: f64 = 0.;
    // Pad.cpp:380
    its_merge(
        pad,
        &walls(&inner_base.contour, &middle_base.contour, z_min, z_max)?,
    );
    // Pad.cpp:381
    thr();
    // Pad.cpp:382
    // its_merge(pad, triangulate_expolygon_3d(inner_base, z_min, NORMALS_UP));
    its_merge_pointf3s(
        pad,
        &to_pointf3s(triangulate_expolygon_3d(&inner_base, z_min, NORMALS_UP)?),
    );

    // Pad.cpp:384
    Ok(true)
}

// Pad.cpp:387-415
// indexed_triangle_set create_outer_pad_geometry(const ExPolygons &skeleton,
//                                                const PadConfig3D &cfg,
//                                                ThrowOnCancel thr)
fn create_outer_pad_geometry(
    skeleton: &ExPolygons,
    cfg: &PadConfig3D,
    thr: ThrowOnCancel,
) -> Result<indexed_triangle_set> {
    // Pad.cpp:391  indexed_triangle_set ret;
    let mut ret = indexed_triangle_set::default();

    // Pad.cpp:393
    for pad_part in skeleton {
        // Pad.cpp:394  ExPolygon top_poly{pad_part};
        let mut top_poly: ExPolygon = pad_part.clone();
        // Pad.cpp:395-396
        let bottom_poly: ExPolygon =
            offset_contour_only(pad_part, -scaled_trunc(cfg.bottom_offset()));

        // Pad.cpp:398
        if bottom_poly.is_empty() {
            continue;
        }
        // Pad.cpp:399
        thr();

        // Pad.cpp:401
        let z_min: f64 = -cfg.height;
        let mut z_max: f64 = 0.;
        // Pad.cpp:402
        its_merge(
            &mut ret,
            &walls(&top_poly.contour, &bottom_poly.contour, z_max, z_min)?,
        );

        // Pad.cpp:404-405
        if cfg.wing_height > 0. && add_cavity(&mut ret, &mut top_poly, cfg, thr)? {
            z_max = -cfg.wing_height;
        }

        // Pad.cpp:407-408
        for h in &bottom_poly.holes {
            its_merge(&mut ret, &straight_walls(h, z_max, z_min)?);
        }

        // Pad.cpp:410
        // its_merge(ret, triangulate_expolygon_3d(bottom_poly, z_min, NORMALS_DOWN));
        its_merge_pointf3s(
            &mut ret,
            &to_pointf3s(triangulate_expolygon_3d(&bottom_poly, z_min, NORMALS_DOWN)?),
        );
        // Pad.cpp:411
        // its_merge(ret, triangulate_expolygon_3d(top_poly, NORMALS_UP));
        // NOTE: in C++ `NORMALS_UP` (= false) binds to the `coordf_t z = 0`
        // parameter (value 0.0) and `flip` takes its default `false`
        // (= NORMALS_UP) — i.e. top surface at z = 0 with normals up.
        its_merge_pointf3s(
            &mut ret,
            &to_pointf3s(triangulate_expolygon_3d(&top_poly, 0., NORMALS_UP)?),
        );
    }

    // Pad.cpp:414
    Ok(ret)
}

// Pad.cpp:417-436
// indexed_triangle_set create_inner_pad_geometry(const ExPolygons &skeleton,
//                                                const PadConfig3D &cfg,
//                                                ThrowOnCancel thr)
fn create_inner_pad_geometry(
    skeleton: &ExPolygons,
    cfg: &PadConfig3D,
    thr: ThrowOnCancel,
) -> Result<indexed_triangle_set> {
    // Pad.cpp:421  indexed_triangle_set ret;
    let mut ret = indexed_triangle_set::default();

    // Pad.cpp:423
    let z_max: f64 = 0.;
    let z_min: f64 = -cfg.height;
    // Pad.cpp:424
    for pad_part in skeleton {
        // Pad.cpp:425
        thr();
        // Pad.cpp:426
        its_merge(&mut ret, &straight_walls(&pad_part.contour, z_max, z_min)?);

        // Pad.cpp:428-429
        for h in &pad_part.holes {
            its_merge(&mut ret, &straight_walls(h, z_max, z_min)?);
        }

        // Pad.cpp:431
        its_merge_pointf3s(
            &mut ret,
            &to_pointf3s(triangulate_expolygon_3d(pad_part, z_min, NORMALS_DOWN)?),
        );
        // Pad.cpp:432
        its_merge_pointf3s(
            &mut ret,
            &to_pointf3s(triangulate_expolygon_3d(pad_part, z_max, NORMALS_UP)?),
        );
    }

    // Pad.cpp:435
    Ok(ret)
}

// Pad.cpp:438-454
// indexed_triangle_set create_pad_geometry(const PadSkeleton &skelet,
//                                          const PadConfig &cfg,
//                                          ThrowOnCancel thr)
fn create_pad_geometry(
    skelet: &PadSkeleton,
    cfg: &PadConfig,
    thr: ThrowOnCancel,
) -> Result<indexed_triangle_set> {
    // Pad.cpp:442-447
    // #ifndef NDEBUG
    //     SVG svg("pad_skeleton.svg");
    //     svg.draw(skelet.outer, "green");
    //     svg.draw(skelet.inner, "blue");
    //     svg.Close();
    // #endif
    // (debug-only SVG dump with no effect on the result; omitted — release /
    //  NDEBUG parity.)

    // Pad.cpp:449  PadConfig3D cfg3d(cfg);
    let cfg3d = PadConfig3D::new(cfg);
    // Pad.cpp:450
    let mut pg = create_outer_pad_geometry(&skelet.outer, &cfg3d, thr)?;
    // Pad.cpp:451
    its_merge(&mut pg, &create_inner_pad_geometry(&skelet.inner, &cfg3d, thr)?);

    // Pad.cpp:453
    Ok(pg)
}

// Pad.cpp:456-472
// indexed_triangle_set create_pad_geometry(const ExPolygons &supp_bp,
//                                          const ExPolygons &model_bp,
//                                          const PadConfig &cfg,
//                                          ThrowOnCancel thr)
// (overload of the function above; distinct Rust name)
fn create_pad_geometry_from_blueprints(
    supp_bp: &ExPolygons,
    model_bp: &ExPolygons,
    cfg: &PadConfig,
    thr: ThrowOnCancel,
) -> Result<indexed_triangle_set> {
    // Pad.cpp:461  PadSkeleton skelet;
    let skelet: PadSkeleton;

    // Pad.cpp:463-469
    if cfg.embed_object.enabled {
        if cfg.embed_object.everywhere {
            skelet = BrimPadSkeleton(supp_bp, model_bp, cfg, thr);
        } else {
            skelet = AroundPadSkeleton(supp_bp, model_bp, cfg, thr);
        }
    } else {
        skelet = BelowPadSkeleton(supp_bp, model_bp, cfg, thr);
    }

    // Pad.cpp:471
    create_pad_geometry(&skelet, cfg, thr)
}

// Pad.cpp:474  } // namespace (anonymous)

// Pad.cpp:476-502
// void pad_blueprint(const indexed_triangle_set &mesh, ExPolygons &output,
//                    const std::vector<float> &heights, ThrowOnCancel thrfn)
pub fn pad_blueprint(
    mesh: &indexed_triangle_set,
    output: &mut ExPolygons,
    heights: &[f32],
    thrfn: ThrowOnCancel,
) {
    // Pad.cpp:481  if (mesh.empty()) return;
    // (its::empty() == indices.empty() || vertices.empty(), admesh/stl.h:247)
    if mesh.indices.is_empty() || mesh.vertices.is_empty() {
        return;
    }

    // Pad.cpp:483  std::vector<ExPolygons> out = slice_mesh_ex(mesh, heights, thrfn);
    // (the 3-arg overload forwards MeshSlicingParamsEx{} defaults,
    //  TriangleMeshSlicer.hpp)
    let out: Vec<ExPolygons> =
        slice_mesh_ex_its(mesh, heights, &MeshSlicingParamsEx::default(), thrfn);

    // Pad.cpp:485-486
    let mut count: usize = 0;
    for o in &out {
        count += o.len();
    }

    // Pad.cpp:488-489 — Unification is expensive, a simplify also speeds up
    // the pad generation
    let mut tmp: ExPolygons = Vec::with_capacity(count);
    // Pad.cpp:490-494
    for o in out {
        for e in o {
            // Pad.cpp:492  auto&& exss = e.simplify(scaled<double>(0.1));
            // ExPolygon::simplify(tolerance) == union_ex(simplify_p(tolerance))
            // (ExPolygon.cpp:253-256); the crate `simplify_p` takes the
            // UNSCALED tolerance (0.1 mm) and re-scales internally.
            let exss: ExPolygons = union_polygons_ex(&e.simplify_p(0.1));
            // Pad.cpp:493
            for ep in exss {
                tmp.push(ep);
            }
        }
    }

    // Pad.cpp:496  ExPolygons utmp = union_ex(tmp);
    let utmp: ExPolygons = union_ex(&tmp);

    // Pad.cpp:498-501
    for o in utmp {
        // Pad.cpp:499  auto&& smp = o.simplify(scaled<double>(0.1));
        let smp: ExPolygons = union_polygons_ex(&o.simplify_p(0.1));
        // Pad.cpp:500  output.insert(output.end(), smp.begin(), smp.end());
        output.extend(smp);
    }
}

// Pad.cpp:504-514
// void pad_blueprint(const indexed_triangle_set &mesh, ExPolygons &output,
//                    float h, float layerh, ThrowOnCancel thrfn)
// (overload of the function above; distinct Rust name. Pad.hpp:32-33 defaults:
//  h = 0.1f, layerh = 0.05f)
pub fn pad_blueprint_with_height(
    mesh: &indexed_triangle_set,
    output: &mut ExPolygons,
    h: f32,
    layerh: f32,
    thrfn: ThrowOnCancel,
) {
    // Pad.cpp:510  float gnd = float(bounding_box(mesh).min(Z));
    let gnd: f32 = bounding_box(mesh).min.z as f32;

    // Pad.cpp:512  std::vector<float> slicegrid = grid(gnd, gnd + h, layerh);
    let slicegrid: Vec<f32> = grid_f32(gnd, gnd + h, layerh);
    // Pad.cpp:513
    pad_blueprint(mesh, output, &slicegrid, thrfn);
}

// Pad.cpp:516-524
// void create_pad(const ExPolygons &sup_blueprint, const ExPolygons &model_blueprint,
//                 indexed_triangle_set &out, const PadConfig &cfg, ThrowOnCancel thr)
pub fn create_pad(
    sup_blueprint: &ExPolygons,
    model_blueprint: &ExPolygons,
    out: &mut indexed_triangle_set,
    cfg: &PadConfig,
    thr: ThrowOnCancel,
) -> Result<()> {
    // Pad.cpp:522  auto t = create_pad_geometry(sup_blueprint, model_blueprint, cfg, thr);
    let t = create_pad_geometry_from_blueprints(sup_blueprint, model_blueprint, cfg, thr)?;
    // Pad.cpp:523  its_merge(out, t);
    its_merge(out, &t);
    Ok(())
}

// ===========================================================================
// Pad.hpp
// ===========================================================================

/// Pad.hpp:43-51 — `struct EmbedObject` (nested in PadConfig)
#[derive(Debug, Clone, PartialEq)]
pub struct EmbedObject {
    /// Pad.hpp:44  double object_gap_mm = 1.;
    pub object_gap_mm: f64,
    /// Pad.hpp:45  double stick_stride_mm = 10.;
    pub stick_stride_mm: f64,
    /// Pad.hpp:46  double stick_width_mm = 0.5;
    pub stick_width_mm: f64,
    /// Pad.hpp:47  double stick_penetration_mm = 0.1;
    pub stick_penetration_mm: f64,
    /// Pad.hpp:48  bool enabled = false;
    pub enabled: bool,
    /// Pad.hpp:49  bool everywhere = false;
    pub everywhere: bool,
}

impl Default for EmbedObject {
    // Pad.hpp:44-49 default member initializers
    fn default() -> Self {
        Self {
            object_gap_mm: 1.,
            stick_stride_mm: 10.,
            stick_width_mm: 0.5,
            stick_penetration_mm: 0.1,
            enabled: false,
            everywhere: false,
        }
    }
}

impl EmbedObject {
    /// Pad.hpp:50  `operator bool() const { return enabled; }`
    #[inline]
    pub fn as_bool(&self) -> bool {
        self.enabled
    }
}

/// Pad.hpp:36-83 — `struct PadConfig`
#[derive(Debug, Clone, PartialEq)]
pub struct PadConfig {
    /// Pad.hpp:37  double wall_thickness_mm = 1.;
    pub wall_thickness_mm: f64,
    /// Pad.hpp:38  double wall_height_mm = 1.;
    pub wall_height_mm: f64,
    /// Pad.hpp:39  double max_merge_dist_mm = 50;
    pub max_merge_dist_mm: f64,
    /// Pad.hpp:40  double wall_slope = std::atan(1.0); // Universal constant for Pi/4
    pub wall_slope: f64,
    /// Pad.hpp:41  double brim_size_mm = 1.6;
    pub brim_size_mm: f64,
    /// Pad.hpp:51  } embed_object;
    pub embed_object: EmbedObject,
}

impl Default for PadConfig {
    // Pad.hpp:53  inline PadConfig() = default;  (with the default member
    // initializers of Pad.hpp:37-41)
    fn default() -> Self {
        Self {
            wall_thickness_mm: 1.,
            wall_height_mm: 1.,
            max_merge_dist_mm: 50.,
            wall_slope: 1.0f64.atan(),
            brim_size_mm: 1.6,
            embed_object: EmbedObject::default(),
        }
    }
}

impl PadConfig {
    /// Pad.hpp:53  `inline PadConfig() = default;`
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Pad.hpp:54-62
    /// `inline PadConfig(double thickness, double height, double mergedist, double slope)`
    #[inline]
    pub fn with_params(thickness: f64, height: f64, mergedist: f64, slope: f64) -> Self {
        Self {
            // Pad.hpp:58  wall_thickness_mm(thickness)
            wall_thickness_mm: thickness,
            // Pad.hpp:59  wall_height_mm(height)
            wall_height_mm: height,
            // Pad.hpp:60  max_merge_dist_mm(mergedist)
            max_merge_dist_mm: mergedist,
            // Pad.hpp:61  wall_slope(slope)
            wall_slope: slope,
            // (brim_size_mm / embed_object keep their defaults)
            ..Self::default()
        }
    }

    /// Pad.hpp:64-67  `inline double bottom_offset() const`
    #[inline]
    pub fn bottom_offset(&self) -> f64 {
        // Pad.hpp:66  return (wall_thickness_mm + wall_height_mm) / std::tan(wall_slope);
        (self.wall_thickness_mm + self.wall_height_mm) / self.wall_slope.tan()
    }

    /// Pad.hpp:69-72  `inline double wing_distance() const`
    #[inline]
    pub fn wing_distance(&self) -> f64 {
        // Pad.hpp:71  return wall_height_mm / std::tan(wall_slope);
        self.wall_height_mm / self.wall_slope.tan()
    }

    /// Pad.hpp:74-77  `inline double full_height() const`
    #[inline]
    pub fn full_height(&self) -> f64 {
        // Pad.hpp:76  return wall_height_mm + wall_thickness_mm;
        self.wall_height_mm + self.wall_thickness_mm
    }

    /// Pad.hpp:79-80 — Returns the elevation needed for compensating the pad.
    /// `inline double required_elevation() const { return wall_thickness_mm; }`
    #[inline]
    pub fn required_elevation(&self) -> f64 {
        self.wall_thickness_mm
    }

    // Pad.cpp:526-536
    // std::string PadConfig::validate() const
    pub fn validate(&self) -> String {
        // Pad.cpp:528  static const double constexpr MIN_BRIM_SIZE_MM = .1;
        const MIN_BRIM_SIZE_MM: f64 = 0.1;

        // Pad.cpp:530-533
        // NOTE: `get_waffle_offset(*this) <= MIN_BRIM_SIZE_MM` compares the
        // SCALED coord_t (converted to double) against 0.1, exactly as the
        // C++ does.
        if self.brim_size_mm < MIN_BRIM_SIZE_MM
            || self.bottom_offset() > self.brim_size_mm + self.wing_distance()
            || (get_waffle_offset(self) as f64) <= MIN_BRIM_SIZE_MM
        {
            return l("Pad brim size is too small for the current configuration.");
        }

        // Pad.cpp:535
        String::new()
    }
}

// Pad.cpp:538  }} // namespace Slic3r::sla
