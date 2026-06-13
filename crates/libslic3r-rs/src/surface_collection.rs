//! Faithful 1:1 port of `src/libslic3r/SurfaceCollection.cpp`
//! (BambuStudio) and the inline methods of `SurfaceCollection.hpp`.
//!
//! The `SurfaceCollection` *struct* itself lives in [`crate::surface`] (mirroring
//! the C++ split where `Surfaces`/`SurfacesPtr` are declared in `Surface.hpp`),
//! so this module only contributes the methods declared & defined in
//! `SurfaceCollection.{hpp,cpp}`. Rust permits multiple `impl` blocks for the
//! same type across modules of the same crate, so these methods are merged into
//! the one canonical `SurfaceCollection`.
//!
//! C++ Reference:
//! - SurfaceCollection.hpp
//! - SurfaceCollection.cpp
//!
//! Translation conventions:
//! - `coord_t`  -> `i64`
//! - `coordf_t` -> `f64`
//! - `SurfacesPtr` (`std::vector<Surface*>`) -> `Vec<&Surface>` borrowing `self`.
//! - `std::initializer_list<SurfaceType>` -> `&[SurfaceType]`.

// #include "SurfaceCollection.hpp"                                     SurfaceCollection.cpp:1
// #include "BoundingBox.hpp"                                           SurfaceCollection.cpp:2
// #include "SVG.hpp"                                                   SurfaceCollection.cpp:3
//
// #include <map>                                                       SurfaceCollection.cpp:5
//
// namespace Slic3r {                                                   SurfaceCollection.cpp:7

use crate::geometry::{polygons_append_expoly, ExPolygon, ExPolygons, Polygons};
use crate::surface::{surfaces_append, surfaces_could_merge, Surface, SurfaceCollection, SurfaceType};

impl SurfaceCollection {
    // void SurfaceCollection::simplify(double tolerance)                SurfaceCollection.cpp:9
    pub fn simplify(&mut self, tolerance: f64) {
        // Surfaces ss;                                                  SurfaceCollection.cpp:11
        let mut ss: Vec<Surface> = Vec::new();
        // for (Surfaces::const_iterator it_s = this->surfaces.begin(); it_s != this->surfaces.end(); ++it_s) {  SurfaceCollection.cpp:12
        for it_s in &self.surfaces {
            // ExPolygons expp;                                          SurfaceCollection.cpp:13
            let mut expp: ExPolygons = Vec::new();
            // it_s->expolygon.simplify(tolerance, &expp);               SurfaceCollection.cpp:14
            //
            // C++ `ExPolygon::simplify(double tolerance, ExPolygons*)` is
            // `append(*expolygons, this->simplify(tolerance))`, and
            // `ExPolygon::simplify(double tolerance)` is
            // `union_ex(this->simplify_p(tolerance))` where the `Polygons`
            // overload of `union_ex` maps to `union_polygons_ex`.
            expp.extend(crate::clipper_utils::union_polygons_ex(
                &it_s.expolygon.simplify_p(tolerance),
            ));
            // for (ExPolygons::const_iterator it_e = expp.begin(); it_e != expp.end(); ++it_e) {  SurfaceCollection.cpp:15
            for it_e in &expp {
                // Surface s = *it_s;                                    SurfaceCollection.cpp:16
                let mut s = it_s.clone();
                // s.expolygon = *it_e;                                  SurfaceCollection.cpp:17
                s.expolygon = it_e.clone();
                // ss.push_back(s);                                      SurfaceCollection.cpp:18
                ss.push(s);
            }
        }
        // this->surfaces = ss;                                          SurfaceCollection.cpp:21
        self.surfaces = ss;
    }

    /* group surfaces by common properties */ //                        SurfaceCollection.cpp:24
    // void SurfaceCollection::group(std::vector<SurfacesPtr> *retval)   SurfaceCollection.cpp:25
    //
    // In C++ this fills `*retval` with raw `Surface*` borrowed from `this->surfaces`.
    // The faithful Rust analog borrows `&self` and returns the grouping.
    pub fn group(&self) -> Vec<Vec<&Surface>> {
        let mut retval: Vec<Vec<&Surface>> = Vec::new();
        // for (Surfaces::iterator it = this->surfaces.begin(); it != this->surfaces.end(); ++it) {  SurfaceCollection.cpp:27
        for it in &self.surfaces {
            // find a group with the same properties                    SurfaceCollection.cpp:28
            // SurfacesPtr* group = NULL;                                SurfaceCollection.cpp:29
            let mut group: Option<usize> = None;
            // for (std::vector<SurfacesPtr>::iterator git = retval->begin(); git != retval->end(); ++git)  SurfaceCollection.cpp:30
            for (gi, git) in retval.iter().enumerate() {
                // if (! git->empty() && surfaces_could_merge(*git->front(), *it)) {  SurfaceCollection.cpp:31
                if !git.is_empty() && surfaces_could_merge(git[0], it) {
                    // group = &*git;                                    SurfaceCollection.cpp:32
                    group = Some(gi);
                    // break;                                            SurfaceCollection.cpp:33
                    break;
                }
            }
            // if no group with these properties exists, add one         SurfaceCollection.cpp:35
            // if (group == NULL) {                                      SurfaceCollection.cpp:36
            let gi = match group {
                Some(gi) => gi,
                None => {
                    // retval->resize(retval->size() + 1);               SurfaceCollection.cpp:37
                    // group = &retval->back();                          SurfaceCollection.cpp:38
                    retval.push(Vec::new());
                    retval.len() - 1
                }
            };
            // append surface to group                                  SurfaceCollection.cpp:40
            // group->push_back(&*it);                                   SurfaceCollection.cpp:41
            retval[gi].push(it);
        }
        retval
    }

    // SurfacesPtr SurfaceCollection::filter_by_type(const SurfaceType type)  SurfaceCollection.cpp:45
    pub fn filter_by_type(&self, ty: SurfaceType) -> Vec<&Surface> {
        // SurfacesPtr ss;                                              SurfaceCollection.cpp:47
        let mut ss: Vec<&Surface> = Vec::new();
        // for (Surface &surface : this->surfaces)                      SurfaceCollection.cpp:48
        for surface in &self.surfaces {
            // if (surface.surface_type == type)                        SurfaceCollection.cpp:49
            if surface.surface_type == ty {
                // ss.push_back(&surface);                              SurfaceCollection.cpp:50
                ss.push(surface);
            }
        }
        // return ss;                                                   SurfaceCollection.cpp:51
        ss
    }

    // SurfacesPtr SurfaceCollection::filter_by_types(std::initializer_list<SurfaceType> types)  SurfaceCollection.cpp:54
    pub fn filter_by_types(&self, types: &[SurfaceType]) -> Vec<&Surface> {
        // SurfacesPtr ss;                                              SurfaceCollection.cpp:56
        let mut ss: Vec<&Surface> = Vec::new();
        // for (Surface& surface : this->surfaces)                      SurfaceCollection.cpp:57
        for surface in &self.surfaces {
            // if (std::find(types.begin(), types.end(), surface.surface_type) != types.end())  SurfaceCollection.cpp:58
            if types.iter().any(|t| *t == surface.surface_type) {
                // ss.push_back(&surface);                              SurfaceCollection.cpp:59
                ss.push(surface);
            }
        }
        // return ss;                                                   SurfaceCollection.cpp:60
        ss
    }

    // void SurfaceCollection::filter_by_type(SurfaceType type, Polygons* polygons)  SurfaceCollection.cpp:63
    //
    // Renamed `filter_by_type_into` (Rust has no overloading on the
    // `filter_by_type(SurfaceType) -> SurfacesPtr` above).
    pub fn filter_by_type_into(&self, ty: SurfaceType, polygons: &mut Polygons) {
        // for (const Surface &surface : this->surfaces)                SurfaceCollection.cpp:65
        for surface in &self.surfaces {
            // if (surface.surface_type == type)                        SurfaceCollection.cpp:66
            if surface.surface_type == ty {
                // polygons_append(*polygons, to_polygons(surface.expolygon));  SurfaceCollection.cpp:67
                polygons_append_expoly(polygons, &surface.expolygon);
            }
        }
    }

    // void SurfaceCollection::keep_type(SurfaceType type, ExPolygons &exps)  SurfaceCollection.cpp:70
    //
    // Renamed `keep_type_collect_exps` to disambiguate from the
    // `keep_type(SurfaceType)` overload (Rust has no fn overloading).
    pub fn keep_type_collect_exps(&mut self, ty: SurfaceType, exps: &mut ExPolygons) {
        // size_t j = 0;                                                SurfaceCollection.cpp:72
        let mut j: usize = 0;
        // for (size_t i = 0; i < surfaces.size(); ++ i) {              SurfaceCollection.cpp:73
        for i in 0..self.surfaces.len() {
            // if (surfaces[i].surface_type == type) {                  SurfaceCollection.cpp:74
            if self.surfaces[i].surface_type == ty {
                // if (j < i)                                           SurfaceCollection.cpp:75
                if j < i {
                    // std::swap(surfaces[i], surfaces[j]);              SurfaceCollection.cpp:76
                    self.surfaces.swap(i, j);
                }
                // ++ j;                                                SurfaceCollection.cpp:77
                j += 1;
            } else {
                // exps.push_back(surfaces[i].expolygon);               SurfaceCollection.cpp:79
                //
                // NOTE: C++ reads `surfaces[i]` *after* the potential swap above.
                // The swap only triggers in the `== type` branch, so in this
                // `else` branch `surfaces[i]` is untouched and equals the
                // pre-swap value, matching C++ exactly.
                exps.push(self.surfaces[i].expolygon.clone());
            }
        }
        // if (j < surfaces.size())                                     SurfaceCollection.cpp:82
        if j < self.surfaces.len() {
            // surfaces.erase(surfaces.begin() + j, surfaces.end());    SurfaceCollection.cpp:83
            self.surfaces.truncate(j);
        }
    }

    // void SurfaceCollection::keep_type(const SurfaceType type)        SurfaceCollection.cpp:86
    pub fn keep_type(&mut self, ty: SurfaceType) {
        // size_t j = 0;                                                SurfaceCollection.cpp:88
        let mut j: usize = 0;
        // for (size_t i = 0; i < surfaces.size(); ++ i) {              SurfaceCollection.cpp:89
        for i in 0..self.surfaces.len() {
            // if (surfaces[i].surface_type == type) {                  SurfaceCollection.cpp:90
            if self.surfaces[i].surface_type == ty {
                // if (j < i)                                           SurfaceCollection.cpp:91
                if j < i {
                    // std::swap(surfaces[i], surfaces[j]);              SurfaceCollection.cpp:92
                    self.surfaces.swap(i, j);
                }
                // ++ j;                                                SurfaceCollection.cpp:93
                j += 1;
            }
        }
        // if (j < surfaces.size())                                     SurfaceCollection.cpp:96
        if j < self.surfaces.len() {
            // surfaces.erase(surfaces.begin() + j, surfaces.end());    SurfaceCollection.cpp:97
            self.surfaces.truncate(j);
        }
    }

    // void SurfaceCollection::keep_types(std::initializer_list<SurfaceType> types)  SurfaceCollection.cpp:100
    pub fn keep_types(&mut self, types: &[SurfaceType]) {
        // size_t j = 0;                                                SurfaceCollection.cpp:101
        let mut j: usize = 0;
        // for (size_t i = 0; i < surfaces.size(); ++ i)                SurfaceCollection.cpp:103
        for i in 0..self.surfaces.len() {
            // if (std::find(types.begin(), types.end(), surfaces[i].surface_type) != types.end()) {  SurfaceCollection.cpp:104
            if types.iter().any(|t| *t == self.surfaces[i].surface_type) {
                // if (j < i)                                           SurfaceCollection.cpp:105
                if j < i {
                    // std::swap(surfaces[i], surfaces[j]);              SurfaceCollection.cpp:106
                    self.surfaces.swap(i, j);
                }
                // ++ j;                                                SurfaceCollection.cpp:107
                j += 1;
            }
        }
        // if (j < surfaces.size())                                     SurfaceCollection.cpp:109
        if j < self.surfaces.len() {
            // surfaces.erase(surfaces.begin() + j, surfaces.end());    SurfaceCollection.cpp:110
            self.surfaces.truncate(j);
        }
    }

    // void SurfaceCollection::remove_type(const SurfaceType type)      SurfaceCollection.cpp:113
    pub fn remove_type(&mut self, ty: SurfaceType) {
        // size_t j = 0;                                                SurfaceCollection.cpp:115
        let mut j: usize = 0;
        // for (size_t i = 0; i < surfaces.size(); ++ i) {              SurfaceCollection.cpp:116
        for i in 0..self.surfaces.len() {
            // if (surfaces[i].surface_type != type) {                  SurfaceCollection.cpp:117
            if self.surfaces[i].surface_type != ty {
                // if (j < i)                                           SurfaceCollection.cpp:118
                if j < i {
                    // std::swap(surfaces[i], surfaces[j]);              SurfaceCollection.cpp:119
                    self.surfaces.swap(i, j);
                }
                // ++ j;                                                SurfaceCollection.cpp:120
                j += 1;
            }
        }
        // if (j < surfaces.size())                                     SurfaceCollection.cpp:123
        if j < self.surfaces.len() {
            // surfaces.erase(surfaces.begin() + j, surfaces.end());    SurfaceCollection.cpp:124
            self.surfaces.truncate(j);
        }
    }

    // void SurfaceCollection::remove_types(std::initializer_list<SurfaceType> types)  SurfaceCollection.cpp:127
    pub fn remove_types(&mut self, types: &[SurfaceType]) {
        // size_t j = 0;                                                SurfaceCollection.cpp:129
        let mut j: usize = 0;
        // for (size_t i = 0; i < surfaces.size(); ++ i)                SurfaceCollection.cpp:130
        for i in 0..self.surfaces.len() {
            // if (std::find(types.begin(), types.end(), surfaces[i].surface_type) == types.end()) {  SurfaceCollection.cpp:131
            if !types.iter().any(|t| *t == self.surfaces[i].surface_type) {
                // if (j < i)                                           SurfaceCollection.cpp:132
                if j < i {
                    // std::swap(surfaces[i], surfaces[j]);              SurfaceCollection.cpp:133
                    self.surfaces.swap(i, j);
                }
                // ++ j;                                                SurfaceCollection.cpp:134
                j += 1;
            }
        }
        // if (j < surfaces.size())                                     SurfaceCollection.cpp:136
        if j < self.surfaces.len() {
            // surfaces.erase(surfaces.begin() + j, surfaces.end());    SurfaceCollection.cpp:137
            self.surfaces.truncate(j);
        }
    }

    // void SurfaceCollection::export_to_svg(const char *path, bool show_labels)  SurfaceCollection.cpp:140
    pub fn export_to_svg(&self, path: &str, show_labels: bool) {
        // BoundingBox bbox;                                            SurfaceCollection.cpp:142
        let mut bbox = crate::geometry::BoundingBox::new();
        // for (Surfaces::const_iterator surface = this->surfaces.begin(); surface != this->surfaces.end(); ++surface)  SurfaceCollection.cpp:143
        //     bbox.merge(get_extents(surface->expolygon));             SurfaceCollection.cpp:144
        for surface in &self.surfaces {
            bbox.merge(&crate::geometry::get_extents_expoly(&surface.expolygon));
        }
        // Point legend_size = export_surface_type_legend_to_svg_box_size();  SurfaceCollection.cpp:145
        let legend_size = crate::surface::export_surface_type_legend_to_svg_box_size();
        // Point legend_pos(bbox.min(0), bbox.max(1));                   SurfaceCollection.cpp:146
        let legend_pos = crate::geometry::Point::new(bbox.min.x(), bbox.max.y());
        // bbox.merge(Point(std::max(bbox.min(0) + legend_size(0), bbox.max(0)), bbox.max(1) + legend_size(1)));  SurfaceCollection.cpp:147
        bbox.merge_point(crate::geometry::Point::new(
            std::cmp::max(bbox.min.x() + legend_size.x(), bbox.max.x()),
            bbox.max.y() + legend_size.y(),
        ));

        // SVG svg(path, bbox);                                         SurfaceCollection.cpp:149
        let mut svg = crate::svg::SVG::new_bbox_default(path, &bbox);
        // const float transparency = 0.5f;                            SurfaceCollection.cpp:150
        let transparency: f32 = 0.5f32;
        // for (Surfaces::const_iterator surface = this->surfaces.begin(); surface != this->surfaces.end(); ++surface) {  SurfaceCollection.cpp:151
        for (idx, surface) in self.surfaces.iter().enumerate() {
            // svg.draw(surface->expolygon, surface_type_to_color_name(surface->surface_type), transparency);  SurfaceCollection.cpp:152
            svg.draw_expolygon(
                &surface.expolygon,
                crate::surface::surface_type_to_color_name(surface.surface_type),
                transparency,
            );
            // if (show_labels) {                                       SurfaceCollection.cpp:153
            if show_labels {
                // int idx = int(surface - this->surfaces.begin());     SurfaceCollection.cpp:154
                // char label[64];                                      SurfaceCollection.cpp:155
                // sprintf(label, "%d", idx);                           SurfaceCollection.cpp:156
                let label = format!("{}", idx);
                // svg.draw_text(surface->expolygon.contour.points.front(), label, "black");  SurfaceCollection.cpp:157
                svg.draw_text(&surface.expolygon.contour.points[0], &label, "black", 20);
            }
        }
        // export_surface_type_legend_to_svg(svg, legend_pos);          SurfaceCollection.cpp:160
        crate::surface::export_surface_type_legend_to_svg(&mut svg, &legend_pos);
        // svg.Close();                                                 SurfaceCollection.cpp:161
        svg.close();
    }

    // ---- SurfaceCollection.hpp inline methods ----

    // template <class T> bool any_internal_contains(const T &item) const  SurfaceCollection.hpp:21
    pub fn any_internal_contains<T: ExPolygonContains>(&self, item: &T) -> bool {
        // for (const Surface &surface : this->surfaces) if (surface.is_internal() && surface.expolygon.contains(item)) return true;  SurfaceCollection.hpp:22
        for surface in &self.surfaces {
            if surface.is_internal() && item.contained_in(&surface.expolygon) {
                return true;
            }
        }
        // return false;                                                SurfaceCollection.hpp:23
        false
    }

    // template <class T> bool any_bottom_contains(const T &item) const   SurfaceCollection.hpp:25
    pub fn any_bottom_contains<T: ExPolygonContains>(&self, item: &T) -> bool {
        // for (const Surface &surface : this->surfaces) if (surface.is_bottom() && surface.expolygon.contains(item)) return true;  SurfaceCollection.hpp:26
        for surface in &self.surfaces {
            if surface.is_bottom() && item.contained_in(&surface.expolygon) {
                return true;
            }
        }
        // return false;                                                SurfaceCollection.hpp:27
        false
    }

    // void set_type(SurfaceType type)                                  SurfaceCollection.hpp:37
    pub fn set_type(&mut self, ty: SurfaceType) {
        // for (Surface &surface : this->surfaces)                      SurfaceCollection.hpp:38
        //     surface.surface_type = type;                             SurfaceCollection.hpp:39
        for surface in &mut self.surfaces {
            surface.surface_type = ty;
        }
    }

    //BBS                                                               SurfaceCollection.hpp:41
    // void change_to_new_type(SurfaceType old_type, SurfaceType new_type)  SurfaceCollection.hpp:42
    pub fn change_to_new_type(&mut self, old_type: SurfaceType, new_type: SurfaceType) {
        // for (Surface& surface : this->surfaces)                      SurfaceCollection.hpp:43
        //     if (surface.surface_type == old_type)                    SurfaceCollection.hpp:44
        //         surface.surface_type = new_type;                     SurfaceCollection.hpp:45
        for surface in &mut self.surfaces {
            if surface.surface_type == old_type {
                surface.surface_type = new_type;
            }
        }
    }

    // bool has(SurfaceType type) const                                 SurfaceCollection.hpp:51
    pub fn has(&self, ty: SurfaceType) -> bool {
        // for (const Surface &surface : this->surfaces)                SurfaceCollection.hpp:52
        //     if (surface.surface_type == type) return true;           SurfaceCollection.hpp:53
        for surface in &self.surfaces {
            if surface.surface_type == ty {
                return true;
            }
        }
        // return false;                                                SurfaceCollection.hpp:54
        false
    }

    // void set(const SurfaceCollection &coll)                          SurfaceCollection.hpp:57
    pub fn set_from_collection(&mut self, coll: &SurfaceCollection) {
        self.surfaces = coll.surfaces.clone();
    }

    // void set(SurfaceCollection &&coll)                               SurfaceCollection.hpp:58
    pub fn set_from_collection_owned(&mut self, coll: SurfaceCollection) {
        self.surfaces = coll.surfaces;
    }

    // void set(const ExPolygons &src, SurfaceType surfaceType)         SurfaceCollection.hpp:59
    //
    // Renamed `set_expolygons` (clashes with `surface.rs`'s convenience
    // `set(&ExPolygons, SurfaceType)`; both are faithful, see note below).
    pub fn set_expolygons(&mut self, src: ExPolygons, surface_type: SurfaceType) {
        // clear(); this->append(src, surfaceType);                     SurfaceCollection.hpp:59
        self.clear();
        self.append_expolygons(src, surface_type);
    }

    // void set(const ExPolygons &src, const Surface &surfaceTempl)     SurfaceCollection.hpp:60
    pub fn set_expolygons_templ(&mut self, src: ExPolygons, surface_templ: &Surface) {
        // clear(); this->append(src, surfaceTempl);                    SurfaceCollection.hpp:60
        self.clear();
        self.append_expolygons_templ(src, surface_templ);
    }

    // void set(const Surfaces &src)                                    SurfaceCollection.hpp:61
    pub fn set_surfaces(&mut self, src: Vec<Surface>) {
        // clear(); this->append(src);                                  SurfaceCollection.hpp:61
        self.clear();
        self.append_surfaces(src);
    }

    // void append(const SurfaceCollection &coll)                       SurfaceCollection.hpp:66
    pub fn append_collection(&mut self, coll: &SurfaceCollection) {
        // this->append(coll.surfaces);                                 SurfaceCollection.hpp:66
        self.append_surfaces(coll.surfaces.clone());
    }

    // void append(const ExPolygons &src, SurfaceType surfaceType)      SurfaceCollection.hpp:68
    pub fn append_expolygons(&mut self, src: ExPolygons, surface_type: SurfaceType) {
        // surfaces_append(this->surfaces, src, surfaceType);           SurfaceCollection.hpp:68
        surfaces_append(&mut self.surfaces, src, surface_type);
    }

    // void append(const ExPolygons &src, const Surface &surfaceTempl)  SurfaceCollection.hpp:69
    pub fn append_expolygons_templ(&mut self, src: ExPolygons, surface_templ: &Surface) {
        // surfaces_append(this->surfaces, src, surfaceTempl);          SurfaceCollection.hpp:69
        crate::surface::surfaces_append_templ(&mut self.surfaces, src, surface_templ);
    }

    // void append(const Surfaces &src)                                 SurfaceCollection.hpp:70
    pub fn append_surfaces(&mut self, src: Vec<Surface>) {
        // surfaces_append(this->surfaces, src);                        SurfaceCollection.hpp:70
        crate::surface::surfaces_append_surfaces(&mut self.surfaces, src);
    }
}

/// Helper trait modeling the C++ `ExPolygon::contains(const T&)` overload set
/// used by the `any_internal_contains` / `any_bottom_contains` templates
/// (`SurfaceCollection.hpp:21-28`). Only the ported `contains(const Point&)`
/// overload is currently available; the `Line`/`Polyline`/`Polylines`
/// overloads are added here as `ExPolygon::contains` becomes ported.
pub trait ExPolygonContains {
    fn contained_in(&self, ex: &ExPolygon) -> bool;
}

// bool ExPolygon::contains(const Point &point, bool border_result = true) const  ExPolygon.hpp:54
impl ExPolygonContains for crate::geometry::Point {
    fn contained_in(&self, ex: &ExPolygon) -> bool {
        ex.contains_point(self)
    }
}

// } // namespace Slic3r                                                SurfaceCollection.cpp:164
