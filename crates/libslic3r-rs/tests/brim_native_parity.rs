//! `slicer::brim` (the port of libslic3r/Brim.cpp) against the C++ at the pin.
//!
//! `tests/data/brim/vectors.txt` and `vectors_print.txt.gz` are the output of `gen.cpp` beside them, which
//! compiles the pin's Brim.cpp in; the header of gen.cpp describes the lines.
//!
//! `comp_second_moment_matches_the_pin`: the three overloads of `compSecondMoment`, the second moment of
//! area the automatic brim width is computed from, over 42 polygons (counter-clockwise, clockwise, of two
//! points), 30 expolygons with up to three holes and 20 groups of them. The same calls run here and have
//! to give the same doubles, bit for bit, with the faithful arm of the gate `BRIM_SECOND_MOMENT_PIN` (the
//! default).
//!
//! `brim_of_a_print_matches_the_pin`: the rest of Brim.cpp. `connect_brim_lines`, `tryExPolygonOffset` and
//! `makeBrimInfill` on polygons; then, per scenario, a Print of boxes assembled as `Print::apply` does,
//! whose first layers, volume groups and first support layers are read back from the vectors, and on it
//! the brim areas, the brims of the inner islands and of the holes, and the two overloads of `make_brim`.
//! Lines that start with "in" are inputs; every other line has to be reproduced.

mod support_shared;

use slicer::brim::{
    comp_second_moment_of_expolygon, comp_second_moment_of_expolygons, comp_second_moment_of_polygon, ExPolyProp,
};
use slicer::geometry::{ExPolygon, Point, PointF, Polygon};
use std::collections::BTreeMap;

fn hd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}

fn poly(tok: &[&str], k: &mut usize) -> Polygon {
    let n: usize = tok[*k].parse().unwrap();
    *k += 1;
    let mut p = Polygon::default();
    for _ in 0..n {
        p.points.push(Point::new(tok[*k].parse().unwrap(), tok[*k + 1].parse().unwrap()));
        *k += 2;
    }
    p
}

fn expoly(tok: &[&str], k: &mut usize) -> ExPolygon {
    let holes: usize = tok[*k].parse().unwrap();
    *k += 1;
    let mut e = ExPolygon::default();
    e.contour = poly(tok, k);
    for _ in 0..holes {
        e.holes.push(poly(tok, k));
    }
    e
}

#[test]
fn comp_second_moment_matches_the_pin() {
    let path = format!("{}/tests/data/brim/vectors.txt", env!("CARGO_MANIFEST_DIR"));
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut by_op: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    for line in text.lines() {
        let (lhs, want) = line.split_once(" -> ").expect("arrow");
        let a: Vec<&str> = lhs.split(' ').collect();
        let mut k = 1usize;
        let got = match a[0] {
            "poly" => {
                let p = poly(&a, &mut k);
                let mut sm = PointF::new(1., 2.);
                let ok = comp_second_moment_of_polygon::comp_second_moment(&p, &mut sm);
                format!("{} {} {}", ok as i32, hd(sm.x()), hd(sm.y()))
            }
            "expoly" => {
                let e = expoly(&a, &mut k);
                let mut prop = ExPolyProp::default();
                let ok = comp_second_moment_of_expolygon::comp_second_moment(&e, &mut prop);
                format!(
                    "{} {} {} {} {} {}",
                    ok as i32,
                    hd(prop.aera),
                    hd(prop.centroid.x()),
                    hd(prop.centroid.y()),
                    hd(prop.second_moment_of_area_respect_to_centroid.x()),
                    hd(prop.second_moment_of_area_respect_to_centroid.y())
                )
            }
            "expolys" => {
                let n: usize = a[k].parse().unwrap();
                k += 1;
                let group: Vec<ExPolygon> = (0..n).map(|_| expoly(&a, &mut k)).collect();
                let (mut ix, mut iy) = (-1.0, -2.0);
                let ok = comp_second_moment_of_expolygons::comp_second_moment(&group, &mut ix, &mut iy);
                format!("{} {} {}", ok as i32, hd(ix), hd(iy))
            }
            other => panic!("unknown line {other}"),
        };
        assert_eq!(k, a.len(), "{}", a[0]);
        let e = by_op.entry(a[0].to_string()).or_insert((0, 0));
        e.0 += 1;
        if got != want {
            e.1 += 1;
            let head: String = lhs.chars().take(70).collect();
            eprintln!("{head}: rust {got} c++ {want}");
        }
    }
    println!("lines by overload (compared, differing): {by_op:?}");
    assert!(by_op.values().all(|v| v.1 == 0), "lines differ from the C++: {by_op:?}");
}

// ---------------------------------------------------------------------------------------------------------
// The rest of Brim.cpp, on a Print.
// ---------------------------------------------------------------------------------------------------------

mod print_level {
    use super::support_shared::*;
    use slicer::brim::{areas, auto_width, loops};
    use slicer::brim_ears_point::BrimPoint;
    use slicer::config::{ConfigOptionInt, ConfigOptionInts, ConfigOptionStrings, ConfigSubstitutionContext, ForwardCompatibilitySubstitutionRule};
    use slicer::extrusion_entity::{ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionLoopRole, ExtrusionPath, ExtrusionRole};
    use slicer::flow::Flow;
    use slicer::geometry::geometry::Vec3d;
    use slicer::geometry::{ExPolygons, Point, Polygons, Polyline, Polylines};
    use slicer::layer::{Layer, LayerRegion};
    use slicer::model::*;
    use slicer::object_id::ObjectID;
    use slicer::print::faithful::{groupedVolumeSlices, Print, SupportLayer};
    use slicer::print_config::config_def::DynamicPrintConfig;
    use slicer::surface::{Surface, SurfaceType};
    use std::collections::BTreeMap;

    fn pts_s(pts: &[Point]) -> String {
        let mut o = format!("{}", pts.len());
        for q in pts {
            o += &format!(" {} {}", q.x, q.y);
        }
        o
    }
    fn pls_s(ps: &[Polyline]) -> String {
        let mut o = format!("{}", ps.len());
        for p in ps {
            o += &format!(" {}", pts_s(&p.points));
        }
        o
    }
    /// gen.cpp `PATH(p)`
    fn path_s(p: &ExtrusionPath) -> String {
        format!("{} {} {} {} {}", p.role as i32, hd(p.mm3_per_mm), hf(p.width as f32), hf(p.height as f32), pts_s(&p.polyline.points))
    }
    /// gen.cpp `ENTS(v)`
    fn ents_s(v: &[ExtrusionEntityType]) -> String {
        let mut o = format!("{}", v.len());
        for e in v {
            match e {
                ExtrusionEntityType::Loop(l) => {
                    o += &format!(" L {}", l.paths.len());
                    for p in &l.paths {
                        o += &format!(" {}", path_s(p));
                    }
                }
                ExtrusionEntityType::Path(p) => o += &format!(" P {}", path_s(p)),
                _ => o += " X",
            }
        }
        o
    }
    fn tok_pts(t: &mut Tok) -> Vec<Point> {
        let n = t.i();
        (0..n).map(|_| t.pt()).collect()
    }
    fn tok_pls(t: &mut Tok) -> Polylines {
        let n = t.i();
        (0..n).map(|_| Polyline::from_points(tok_pts(t))).collect()
    }
    fn tok_path(t: &mut Tok) -> ExtrusionPath {
        let role = match t.i() {
            2 => ExtrusionRole::ExternalPerimeter,
            12 => ExtrusionRole::Skirt,
            13 => ExtrusionRole::Brim,
            14 => ExtrusionRole::SupportMaterial,
            v => panic!("role {v}"),
        };
        let mm3_per_mm = t.d();
        let (w, h) = (t.f(), t.f());
        let mut p = ExtrusionPath::with_params(role, mm3_per_mm, w as f64, h as f64, false);
        p.polyline.points = tok_pts(t);
        p
    }
    fn tok_ents(t: &mut Tok) -> Vec<ExtrusionEntityType> {
        let n = t.i();
        (0..n)
            .map(|_| match t.s() {
                "L" => {
                    let np = t.i();
                    let paths: Vec<ExtrusionPath> = (0..np).map(|_| tok_path(t)).collect();
                    ExtrusionEntityType::Loop(ExtrusionLoop::new(paths, ExtrusionLoopRole::DEFAULT))
                }
                "P" => ExtrusionEntityType::Path(tok_path(t)),
                other => panic!("entity {other}"),
            })
            .collect()
    }
    fn object_index(print: &Print, id: ObjectID) -> i64 {
        let mut idx = -1;
        for (i, obj) in print.objects().iter().enumerate() {
            if obj.id() == id {
                idx = i as i64;
            }
        }
        idx
    }
    /// gen.cpp `MAP(print, m)`
    fn map_s(print: &Print, m: &BTreeMap<ObjectID, ExPolygons>) -> String {
        let mut o = format!("{}", m.len());
        for (id, v) in m {
            o += &format!(" {} {}", object_index(print, *id), expolys_s(v));
        }
        o
    }
    /// gen.cpp `EMAP(print, m)`
    fn emap_s(print: &Print, m: &BTreeMap<ObjectID, ExtrusionEntityCollection>) -> String {
        let mut o = format!("{}", m.len());
        for (id, v) in m {
            o += &format!(" {} {}", object_index(print, *id), ents_s(&v.entities));
        }
        o
    }
    fn objs_s(v: &[usize]) -> String {
        let mut o = format!("{}", v.len());
        for i in v {
            o += &format!(" {i}");
        }
        o
    }

    /// gen.cpp `set_common`
    fn common_config() -> DynamicPrintConfig {
        let mut config = DynamicPrintConfig::full_print_config();
        config.set_num_filaments(3).unwrap();
        config.set_key_value("filament_colour", Box::new(ConfigOptionStrings::from_values(vec!["#FF0000".into(), "#00FF00".into(), "#0000FF".into()])));
        config.set_key_value("filament_map", Box::new(ConfigOptionInts::from_values(vec![1, 1, 1])));
        config
    }

    struct Statics {
        speed: GlobalSpeedMap,
        extruders: BTreeMap<usize, ExtruderParams>,
    }

    fn flow_line(flow: &Flow) -> String {
        format!("{} {} {}", hf(flow.width() as f32), hf(flow.height() as f32), hf(flow.spacing() as f32))
    }

    /// gen.cpp `run_helpers`
    fn run_helpers(r: &mut Run) {
        while let Some(line) = r.input("in cbl") {
            let mut t = Tok::new(&line);
            let pls = tok_pls(&mut t);
            let area: Polygons = t.polys();
            let max_len = t.f();
            r.out(format!("cbl {}", pls_s(&loops::connect_brim_lines(pls, &area, max_len))));
        }
        let mut model = Model::new();
        {
            let o = model.add_object_new();
            o.name = "box".into();
            o.add_volume_mesh(ModelMesh::from_its(box_its(20., 20., 10., 0., 0., 0.)), ModelVolumeType::ModelPart, true);
            o.add_instance().set_offset(&Vec3d::new(100., 100., 0.));
        }
        let mut print = assemble(&model, &common_config());
        let flow = print.brim_flow().expect("brim flow");
        r.out(format!(
            "helpers flow {} {} {} {}",
            flow_line(&flow),
            hf(flow.nozzle_diameter() as f32),
            hd(flow.mm3_per_mm().expect("mm3_per_mm")),
            hd(print.skirt_first_layer_height())
        ));
        while let Some(line) = r.input("in bi") {
            let mut t = Tok::new(&line);
            let area = t.expolys();
            let (ox, oy) = (t.d(), t.d());
            print.set_plate_origin([ox, oy, 0.].into());
            r.out(format!("bi offset {}", polys_s(&loops::try_ex_polygon_offset(&area, &print, &flow))));
            let islands: Polygons = Vec::new();
            let c = loops::make_brim_infill(&area, &print, &islands, &flow).expect("make_brim_infill");
            r.out(format!("bi infill {}", ents_s(&c.entities)));
        }
    }

    struct ObjSpec {
        extruder: i64,
    }

    /// gen.cpp `run_scenario`
    fn run_scenario(r: &mut Run, k: usize, statics: &Statics) -> bool {
        let s = format!("b{k}");
        let mut model = Model::new();
        let mut specs: Vec<ObjSpec> = Vec::new();
        while let Some(line) = r.input(&format!("in {s} object")) {
            let mut t = Tok::new(&line);
            let (sx, sy, sz) = (t.d(), t.d(), t.d());
            let _kind = t.i();
            let extruder = t.i();
            let _support = t.b();
            let ears = t.i();
            let o = model.add_object_new();
            o.name = "box".into();
            o.add_volume_mesh(ModelMesh::from_its(box_its(sx, sy, sz, -(sx as f32) / 2.0, -(sy as f32) / 2.0, 0.0)), ModelVolumeType::ModelPart, true);
            let n = t.i();
            for _ in 0..n {
                let (x, y) = (t.d(), t.d());
                o.add_instance().set_offset(&Vec3d::new(x, y, 0.0));
            }
            let n = t.i();
            for _ in 0..n {
                let (key, value) = (t.s(), t.s());
                let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
                o.config.set_deserialize(key, value, &mut ctxt, false).unwrap();
            }
            if extruder != 1 {
                o.config.set_key_value("extruder", Box::new(ConfigOptionInt::with_value(extruder as i32)));
            }
            for e in 0..ears {
                let e = e as f64;
                o.brim_points.push(BrimPoint::from_coords((-sx / 2. + e * sx) as f32, (sy / 2. - e * 0.5 * sy) as f32, 0., 4. + e as f32, -1));
            }
            specs.push(ObjSpec { extruder });
        }
        if specs.is_empty() {
            return false;
        }
        let mut config = common_config();
        while let Some(line) = r.input(&format!("in {s} cfg")) {
            let (key, value) = line.split_once(' ').expect("key value");
            deser(&mut config, key, value);
        }
        let mut print = assemble(&model, &config);
        let line = r.input(&format!("in {s} print")).expect("print");
        let mut t = Tok::new(&line);
        let (ox, oy) = (t.d(), t.d());
        let extra = t.i();
        print.set_plate_origin([ox, oy, 0.].into());
        // The materials of the first two filaments (Model::extruderParamsMap).
        let mut extruder_params = statics.extruders.clone();
        for key in [1usize, 2] {
            extruder_params.get_mut(&key).expect("extruder").material_name = t.s().to_string();
        }
        let statics = Statics { speed: statics.speed.clone(), extruders: extruder_params };

        let mut obj_print_vec: Vec<(ObjectID, u32)> = Vec::new();
        let mut print_extruders: Vec<u32> = Vec::new();
        for i in 0..print.m_objects.len() {
            let line = r.input(&format!("in {s} layer")).expect("layer");
            let mut t = Tok::new(&line);
            assert_eq!(t.i() as usize, i);
            let (z, h, slice_z) = (t.d(), t.d(), t.d());
            let mut layer = Layer::new(0, i, h, z, slice_z);
            layer.lslices = t.expolys();
            let n_regions = t.i() as usize;
            for _ in 0..n_regions {
                // The region of the object (it has one volume, so one region); both layer regions print with it.
                let mut lr = LayerRegion::new(0, 0);
                for e in t.expolys() {
                    lr.slices.surfaces.push(Surface::new(SurfaceType::Internal, e));
                }
                lr.perimeters.entities = tok_ents(&mut t);
                layer.add_region(lr);
            }
            print.m_objects[i].m_layers.push(layer);

            let line = r.input(&format!("in {s} groups")).expect("groups");
            let mut t = Tok::new(&line);
            assert_eq!(t.i() as usize, i);
            let vid = print.m_objects[i].model_object(&print).volumes[0].id();
            let n = t.i();
            for _ in 0..n {
                let group_id = t.i() as i32;
                let slices = t.expolys();
                print.m_objects[i].firstLayerObjSliceByGroups.push(groupedVolumeSlices { groupId: group_id, volume_ids: vec![vid], slices });
            }

            if let Some(line) = r.input(&format!("in {s} support")) {
                let mut t = Tok::new(&line);
                assert_eq!(t.i() as usize, i);
                let (z, h, slice_z) = (t.d(), t.d(), t.d());
                let mut sl = SupportLayer::new(0, 0, i, h, z, slice_z);
                sl.support_islands = t.expolys();
                sl.support_fills.entities = tok_ents(&mut t);
                print.m_objects[i].m_support_layers.push(sl);
            }
            let extruder = specs[i].extruder as u32;
            obj_print_vec.push((print.m_objects[i].id(), extruder));
            if !print_extruders.contains(&(extruder - 1)) {
                print_extruders.push(extruder - 1);
            }
        }
        print_extruders.sort();
        let mut skirt = ExtrusionEntityCollection::new();
        if extra == 1 {
            let line = r.input(&format!("in {s} tower")).expect("tower");
            let mut t = Tok::new(&line);
            let z = t.f();
            let wall = Polyline::from_points(tok_pts(&mut t));
            print.m_fake_wipe_tower.outer_wall.insert(ordered_float::OrderedFloat(z), vec![wall]);
            print.m_fake_wipe_tower.pos = [t.f(), t.f()];
        }
        if extra == 2 {
            let line = r.input(&format!("in {s} skirt")).expect("skirt");
            skirt.entities = tok_ents(&mut Tok::new(&line));
        }
        {
            let mut line = format!("{s} setup {} {} {}", hb(print.has_wipe_tower()), hb(print.has_support_material()), print.extruders(false).len());
            for obj in print.objects() {
                line += &format!(" {} {}", hb(obj.has_brim()), obj.instances().len());
                for inst in obj.instances() {
                    let shift = inst.shift_without_plate_offset(&print);
                    line += &format!(" {} {}", shift.x, shift.y);
                }
            }
            r.out(line);
        }

        {
            let mut line = format!("{s} adhesion");
            for obj in print.objects() {
                line += &format!(
                    " {} {}",
                    hd(auto_width::get_temperature_from_extruder(&print, obj)),
                    hd(auto_width::getadhesion_coeff(&print, obj, &statics.extruders))
                );
            }
            r.out(line);
        }

        let flow = print.brim_flow().expect("brim flow");
        let fs = flow.scaled_spacing() as f32;
        r.out(format!("{s} flow {} {} {}", flow_line(&flow), hd(flow.mm3_per_mm().expect("mm3_per_mm")), hd(print.skirt_first_layer_height())));

        // The areas.
        let bottom = areas::get_print_bottom_layers_expolygons(&print);
        {
            let mut line = format!("{s} bottom {}", bottom.len());
            for e in &bottom {
                line += &format!(" {}", expolys_s(e));
            }
            r.out(line);
        }
        let top = areas::get_top_level_objects_with_brim(&print, &bottom);
        r.out(format!("{s} top {}", objs_s(&top)));
        let scaled_resolution = print.config().opt_float("resolution") / slicer::libslic3r::SCALING_FACTOR;
        if !top.is_empty() {
            r.out(format!("{s} islands {}", polys_s(&areas::top_level_outer_brim_islands(&print, &top, scaled_resolution))));
            {
                let mut wmax = 0.;
                let mut wmap: BTreeMap<ObjectID, f64> = BTreeMap::new();
                let a = areas::top_level_outer_brim_area(&print, &top, &bottom, fs, &mut wmax, &mut wmap);
                let mut line = format!("{s} outer_area {} {} {}", expolys_s(&a), hd(wmax), wmap.len());
                for v in wmap.values() {
                    line += &format!(" {}", hd(*v));
                }
                r.out(line);
            }
            {
                let mut wmax = 0.;
                let mut wmap: BTreeMap<ObjectID, f64> = BTreeMap::new();
                let (mut am, mut sm) = (BTreeMap::new(), BTreeMap::new());
                let a = areas::top_level_outer_brim_area_map(&print, &top, fs, &mut wmax, &mut wmap, &mut am, &mut sm, &obj_print_vec);
                r.out(format!("{s} outer_area_map {} {} | {} | {}", expolys_s(&a), hd(wmax), map_s(&print, &am), map_s(&print, &sm)));
            }
            r.out(format!("{s} inner_area {}", expolys_s(&areas::inner_brim_area(&print, &top, &bottom, fs))));
            {
                let (mut am, mut sm) = (BTreeMap::new(), BTreeMap::new());
                let a = areas::inner_brim_area_map(&print, &top, fs, &mut am, &mut sm, &obj_print_vec);
                r.out(format!("{s} inner_area_map {} | {} | {}", expolys_s(&a), map_s(&print, &am), map_s(&print, &sm)));
            }
            // The inner islands.
            {
                let mut brim = ExtrusionEntityCollection::new();
                let mut islands: ExPolygons = Vec::new();
                loops::make_inner_island_brim(&print, &top, &mut brim, &mut islands).expect("make_inner_island_brim");
                r.out(format!("{s} inner_island {} | {}", ents_s(&brim.entities), expolys_s(&islands)));
            }
            {
                let line = r.input(&format!("in {s} nobrim")).expect("nobrim");
                let nobrim = Tok::new(&line).expolys();
                let (mut am, mut sm) = (BTreeMap::new(), BTreeMap::new());
                let mut islands: ExPolygons = Vec::new();
                loops::make_inner_island_brim_map(&print, &top, &mut am, &mut sm, &mut islands, &nobrim, &obj_print_vec).expect("make_inner_island_brim");
                r.out(format!("{s} inner_island_map {} | {} | {}", expolys_s(&islands), map_s(&print, &am), map_s(&print, &sm)));
            }
            {
                let mut brim = ExtrusionEntityCollection::new();
                loops::make_inner_brim(&print, &top, &bottom, &mut brim).expect("make_inner_brim");
                r.out(format!("{s} inner_brim {}", ents_s(&brim.entities)));
            }
            {
                let (mut am, mut sm) = (BTreeMap::new(), BTreeMap::new());
                loops::make_inner_brim_map(&print, &top, &mut am, &mut sm, &obj_print_vec).expect("make_inner_brim");
                r.out(format!("{s} inner_brim_map {} | {}", map_s(&print, &am), map_s(&print, &sm)));
            }
        }
        // The brims of all objects by volume groups.
        {
            let (mut am, mut sm) = (BTreeMap::new(), BTreeMap::new());
            let a = loops::outer_inner_brim_area(&print, fs, &mut am, &mut sm, &obj_print_vec, &print_extruders, &statics.extruders, &statics.speed)
                .expect("outer_inner_brim_area");
            r.out(format!("{s} oiba {} | {} | {}", expolys_s(&a), map_s(&print, &am), map_s(&print, &sm)));
        }
        {
            let mut islands_area: Polygons = Vec::new();
            let (mut bm, mut sbm) = (BTreeMap::new(), BTreeMap::new());
            loops::make_brim_map(&mut print, None, &mut islands_area, &mut bm, &mut sbm, &obj_print_vec, &print_extruders, &statics.extruders, &statics.speed)
                .expect("make_brim");
            r.out(format!("{s} make_brim_map {} | {} | {}", polys_s(&islands_area), emap_s(&print, &bm), emap_s(&print, &sbm)));
            let mut line = format!("{s} bbox");
            for obj in print.objects() {
                let bb = &obj.firstLayerObjectBrimBoundingBox;
                line += &format!(" {}", hb(bb.is_defined()));
                if bb.is_defined() {
                    line += &format!(" {} {} {} {}", bb.min.x, bb.min.y, bb.max.x, bb.max.y);
                }
            }
            r.out(line);
        }
        // The brim around all objects.
        if !top.is_empty() {
            let mut islands_area: Polygons = Vec::new();
            let brim = loops::make_brim(&print, None, &mut islands_area, &skirt).expect("make_brim");
            r.out(format!("{s} make_brim {} | {}", polys_s(&islands_area), ents_s(&brim.entities)));
        }
        true
    }

    #[test]
    fn brim_of_a_print_matches_the_pin() {
        use std::io::Read;
        let path = format!("{}/tests/data/brim/vectors_print.txt.gz", env!("CARGO_MANIFEST_DIR"));
        let mut text = String::new();
        flate2::read::GzDecoder::new(std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"))).read_to_string(&mut text).expect("vectors_print.txt.gz");
        let mut r = Run { expected: text.lines().map(|l| l.to_string()).collect(), pos: 0, lines: Vec::new(), surplus: 0 };

        // The statics of Model the brim reads.
        let line = r.input("in statics").expect("statics");
        let mut t = Tok::new(&line);
        let mut speed = GlobalSpeedMap::default();
        speed.perimeter_speed = t.d();
        speed.external_perimeter_speed = t.d();
        speed.infill_speed = t.d();
        speed.solid_infill_speed = t.d();
        speed.top_solid_infill_speed = t.d();
        speed.support_speed = t.d();
        speed.small_perimeter_speed = t.d();
        speed.max_speed = t.d();
        let mut extruders: BTreeMap<usize, ExtruderParams> = BTreeMap::new();
        let n = t.i();
        for _ in 0..n {
            let key = t.i() as usize;
            let name = t.s();
            let bed_temp = t.i() as i32;
            let heat_end_temp = t.d();
            extruders.insert(key, ExtruderParams { material_name: if name == "-" { String::new() } else { name.to_string() }, bed_temp, heat_end_temp });
        }
        let statics = Statics { speed, extruders };

        run_helpers(&mut r);
        let mut k = 0;
        while run_scenario(&mut r, k, &statics) {
            k += 1;
        }
        let bad = r.finish();
        // Per stage (the token after the scenario, or the name of the helper): lines equal to the C++, lines.
        let mut by_stage: BTreeMap<String, (usize, usize)> = BTreeMap::new();
        for (i, e) in r.expected.iter().enumerate() {
            if e.starts_with("in ") {
                continue;
            }
            let mut tok = e.split(' ');
            let first = tok.next().unwrap_or("");
            let stage = if first.len() > 1 && first.starts_with('b') && first[1..].chars().all(|c| c.is_ascii_digit()) {
                tok.next().unwrap_or("").to_string()
            } else if first == "bi" || first == "helpers" {
                format!("{first} {}", tok.next().unwrap_or(""))
            } else {
                first.to_string()
            };
            let entry = by_stage.entry(stage).or_insert((0, 0));
            entry.1 += 1;
            if r.lines.get(i) == Some(e) {
                entry.0 += 1;
            }
        }
        let mut report = String::new();
        for (stage, (ok, n)) in &by_stage {
            report += &format!("{stage} {ok}/{n}; ");
        }
        println!("{k} scenarios; lines equal to the C++ by stage: {report}");
        assert_eq!(bad, 0, "lines differ from the C++: {report}");
    }
}
