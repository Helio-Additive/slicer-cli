//! Native parity of the Support/SupportMaterial.cpp port (`slicer::support::support_material`) against
//! vectors made by the C++ at the pin (tests/data/support_material/gen.cpp).
//!
//! The first part ("grid") is the support grid: rasterize_polygons, contours_simplified, SupportGridParams
//! and SupportGridPattern (the constructor, extract_support, dilate_trimming_region, seed_fill_block,
//! island_samples) on polygons and rasters the generator made. The lines of the vectors that start with
//! "in" are the inputs, which this test reads back; every other line it has to reproduce.

mod support_shared;

use slicer::config::{ConfigOptionInts, ConfigOptionStrings};
use slicer::extrusion_entity::{ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionLoopRole, ExtrusionMultiPath, ExtrusionPath, ExtrusionRole};
use slicer::flow::{Flow, FlowRole};
use slicer::geometry::{Point, Polygons, Polyline, Polylines};
use slicer::layer::{Layer, LayerRegion};
use slicer::model::*;
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::fill::fill_base::Fill;
use slicer::fill::FillParams;
use slicer::print_config::InfillPattern;
use slicer::support::support_common::{cu, BBox, InfillFailedException, SupportFill, SupportFillFactory, NULL_LAYER};
use slicer::support::support_layer::{SupportGeneratorLayer, SupportGeneratorLayerStorage, SupportGeneratorLayersPtr};
use slicer::support::support_material::grid::{contours_simplified, rasterize_polygons, SupportGridParams, SupportGridPattern, SMS_GRID};
use slicer::support::support_material::material::*;
use slicer::surface::{Surface, SurfaceType};
use support_shared::*;

/// gen.cpp `HEX`
fn hex(d: &[u8]) -> String {
    if d.is_empty() {
        return "-".into();
    }
    const DIGITS: &[u8; 16] = b"0123456789abcdef";
    let mut o = String::with_capacity(d.len() * 2);
    for c in d {
        o.push(DIGITS[(c >> 4) as usize] as char);
        o.push(DIGITS[(c & 15) as usize] as char);
    }
    o
}
fn unhex(s: &str) -> Vec<u8> {
    if s == "-" {
        return Vec::new();
    }
    let b = s.as_bytes();
    let v = |c: u8| if c <= b'9' { c - b'0' } else { c - b'a' + 10 };
    (0..b.len() / 2).map(|i| (v(b[2 * i]) << 4) | v(b[2 * i + 1])).collect()
}
fn pts_s(pts: &[Point]) -> String {
    let mut o = format!("{}", pts.len());
    for q in pts {
        o += &format!(" {}", pt(q));
    }
    o
}
/// gen.cpp `PARAMS`
fn params_s(p: &SupportGridParams) -> String {
    format!(
        "{} {} {} {} {} {} {}",
        p.style,
        hd(p.grid_resolution),
        hd(p.support_angle),
        hd(p.extrusion_width),
        hd(p.support_closing_radius),
        p.expansion_to_slice,
        p.expansion_to_propagate
    )
}
fn read_params(t: &mut Tok) -> SupportGridParams {
    SupportGridParams {
        style: t.i() as i32,
        grid_resolution: t.d(),
        support_angle: t.d(),
        extrusion_width: t.d(),
        support_closing_radius: t.d(),
        expansion_to_slice: t.i() as i32,
        expansion_to_propagate: t.i() as i32,
    }
}

/// gen.cpp `run_grid`
fn run_grid(r: &mut Run) {
    while let Some(line) = r.input("in rast") {
        let mut t = Tok::new(&line);
        let grid_size = (t.i() as i32, t.i() as i32);
        let pixel_size = t.d();
        let left_bottom = t.pt();
        let polygons: Polygons = t.polys();
        r.out(format!("rast {}", hex(&rasterize_polygons(grid_size, pixel_size, &left_bottom, &polygons))));
    }
    while let Some(line) = r.input("in dil") {
        let mut t = Tok::new(&line);
        let grid_size = (t.i() as i32, t.i() as i32);
        let trimming = unhex(t.s());
        r.out(format!("dil {}", hex(&SupportGridPattern::dilate_trimming_region(&trimming, grid_size))));
    }
    while let Some(line) = r.input("in seed") {
        let mut t = Tok::new(&line);
        let grid_size = (t.i() as i32, t.i() as i32);
        let grid_blocks = (t.i() as i32, t.i() as i32);
        let oversampling = t.i() as i32;
        let mut grid = unhex(t.s());
        let trimming = unhex(t.s());
        SupportGridPattern::seed_fill_block(&mut grid, grid_size, &trimming, grid_blocks, oversampling);
        r.out(format!("seed {}", hex(&grid)));
    }
    while let Some(line) = r.input("in cs") {
        let mut t = Tok::new(&line);
        let grid_size = (t.i() as i32, t.i() as i32);
        let pixel_size = t.d();
        let left_bottom = t.pt();
        let offset = t.i() as i32;
        let fill_holes = t.b();
        let grid = unhex(t.s());
        r.out(format!("cs {}", polys_s(&contours_simplified(grid_size, pixel_size, left_bottom, &grid, offset, fill_holes))));
    }
    while let Some(line) = r.input("in isl") {
        let mut t = Tok::new(&line);
        let expolygons = t.expolys();
        r.out(format!("isl {}", pts_s(&SupportGridPattern::island_samples(&expolygons))));
    }
    while let Some(line) = r.input("in par") {
        let mut t = Tok::new(&line);
        let style = t.i();
        let spacing = t.d();
        let angle = t.d();
        let (w, h, n) = (t.d(), t.d(), t.d());
        let mut config = DynamicPrintConfig::new();
        let style_name = ["default", "grid", "snug", "tree_slim", "tree_strong", "tree_hybrid", "tree_organic"][style as usize];
        deser(&mut config, "support_style", style_name);
        deser(&mut config, "support_base_pattern_spacing", &format!("{spacing}"));
        deser(&mut config, "support_angle", &format!("{angle}"));
        let flow = Flow::new(w, h, n).expect("flow");
        r.out(format!("par {}", params_s(&SupportGridParams::new(&config, &flow))));
    }
    while let Some(line) = r.input("in pat") {
        let mut t = Tok::new(&line);
        let params = read_params(&mut t);
        let support_polygons: Polygons = t.polys();
        let trimming_polygons: Polygons = t.polys();
        let pattern = SupportGridPattern::new(&support_polygons, &trimming_polygons, &params);
        if pattern.style() == SMS_GRID {
            let (grid_size, pixel_size, min, max, grid) = pattern.grid_state();
            r.out(format!("pat {} {} {} {} {} {}", grid_size.0, grid_size.1, hd(pixel_size), pt(&min), pt(&max), hex(grid)));
        } else {
            r.out(format!("pat {}", pattern.style()));
        }
        while let Some(line) = r.input("in ext") {
            let mut t = Tok::new(&line);
            let offset = t.i() as i32;
            let fill_holes = t.b();
            r.out(format!("ext {}", polys_s(&pattern.extract_support(offset, fill_holes))));
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// The stages of PrintObjectSupportMaterial on the object of a Print (gen.cpp `run_object`).
// ---------------------------------------------------------------------------------------------------------

fn floats_s(v: &[f32]) -> String {
    let mut o = format!("{}", v.len());
    for f in v {
        o += &format!(" {}", hf(*f));
    }
    o
}
fn polysv_s(v: &[Polygons]) -> String {
    let mut o = format!("{}", v.len());
    for p in v {
        o += &format!(" {}", polys_s(p));
    }
    o
}
fn role_from(v: i64) -> ExtrusionRole {
    match v {
        1 => ExtrusionRole::Perimeter,
        2 => ExtrusionRole::ExternalPerimeter,
        3 => ExtrusionRole::OverhangPerimeter,
        4 => ExtrusionRole::InternalInfill,
        5 => ExtrusionRole::SolidInfill,
        10 => ExtrusionRole::BridgeInfill,
        _ => panic!("role {v}"),
    }
}
fn tok_path(t: &mut Tok) -> ExtrusionPath {
    assert_eq!(t.s(), "P");
    let role = role_from(t.i());
    let mm3_per_mm = t.d();
    let (w, h) = (t.f(), t.f());
    let mut q = ExtrusionPath::with_params(role, mm3_per_mm, w as f64, h as f64, false);
    let n = t.i();
    q.polyline = Polyline::from_points((0..n).map(|_| t.pt()).collect());
    q
}
fn tok_paths(t: &mut Tok) -> Vec<ExtrusionPath> {
    let n = t.i();
    (0..n).map(|_| tok_path(t)).collect()
}
fn tok_ent(t: &mut Tok) -> ExtrusionEntityType {
    // A path starts with "P", which tok_path reads itself.
    let mut peek = t.t.clone();
    match peek.next().expect("entity") {
        "P" => ExtrusionEntityType::Path(tok_path(t)),
        "L" => {
            t.s();
            ExtrusionEntityType::Loop(ExtrusionLoop::new(tok_paths(t), ExtrusionLoopRole::DEFAULT))
        }
        "M" => {
            t.s();
            let mut m = ExtrusionMultiPath::new();
            m.paths = tok_paths(t);
            ExtrusionEntityType::MultiPath(m)
        }
        "C" => {
            t.s();
            let mut c = ExtrusionEntityCollection::new();
            let n = t.i();
            for _ in 0..n {
                c.entities.push(tok_ent(t));
            }
            ExtrusionEntityType::Collection(Box::new(c))
        }
        other => panic!("entity {other}"),
    }
}
fn tok_ents(t: &mut Tok) -> Vec<ExtrusionEntityType> {
    let n = t.i();
    (0..n).map(|_| tok_ent(t)).collect()
}
fn tok_surfs(t: &mut Tok) -> Vec<Surface> {
    let n = t.i();
    (0..n)
        .map(|_| {
            let ty = match t.i() {
                0 => SurfaceType::Top,
                1 => SurfaceType::Bottom,
                2 => SurfaceType::BottomBridge,
                3 => SurfaceType::Internal,
                v => panic!("surface type {v}"),
            };
            let has_angle = t.b();
            let angle = t.d();
            let mut s = Surface::new(ty, t.expoly());
            s.bridge_angle = if has_angle { Some(angle) } else { None };
            s
        })
        .collect()
}
fn tok_pls(t: &mut Tok) -> Polylines {
    let n = t.i();
    (0..n)
        .map(|_| {
            let m = t.i();
            Polyline::from_points((0..m).map(|_| t.pt()).collect())
        })
        .collect()
}
fn opt_s(p: &Option<Polygons>) -> String {
    match p {
        Some(p) => format!("1 {}", polys_s(p)),
        None => "0".into(),
    }
}
/// gen.cpp `LAYER(l)`
fn layer_s(l: &SupportGeneratorLayer) -> String {
    format!(
        "{} {} {} {} {} {} {} {} {} {} {} {}",
        l.layer_type as i32,
        hd(l.print_z),
        hd(l.bottom_z),
        hd(l.height),
        l.idx_object_layer_above as i64,
        l.idx_object_layer_below as i64,
        hb(l.bridging),
        hb(l.up),
        polys_s(&l.polygons),
        opt_s(&l.contact_polygons),
        opt_s(&l.overhang_polygons),
        opt_s(&l.enforcer_polygons)
    )
}
fn list_s(v: &SupportGeneratorLayersPtr) -> String {
    let mut o = format!("{}", v.len());
    for &l in v {
        o += &format!(" {}", if l == NULL_LAYER { -1 } else { l as i64 });
    }
    o
}
fn dump_storage(r: &mut Run, tag: &str, storage: &SupportGeneratorLayerStorage) {
    r.out(format!("{tag} storage {}", storage.len()));
    for i in 0..storage.len() {
        r.out(format!("{tag} L {}", layer_s(&storage[i])));
    }
}

/// gen.cpp `run_object`. False when the vectors have no such scenario.
fn run_object(r: &mut Run, scenario: usize) -> bool {
    let s = format!("s{scenario}");
    let Some(line) = r.input(&format!("in {s} model")) else {
        return false;
    };
    let mut t = Tok::new(&line);
    let (sx, sy, sz, ix, iy) = (t.i() as f64, t.i() as f64, t.i() as f64, t.i() as f64, t.i() as f64);
    let volumes = t.i();
    let mut model = Model::new();
    {
        let o = model.add_object_new();
        o.name = "box".into();
        o.add_volume_mesh(ModelMesh::from_its(box_its(sx, sy, sz, -(sx as f32) / 2.0, -(sy as f32) / 2.0, 0.0)), ModelVolumeType::ModelPart, true);
        if volumes & 1 != 0 {
            o.add_volume_mesh(ModelMesh::from_its(box_its(4., 14., 0.95, -10., -7., 2.35)), ModelVolumeType::SupportEnforcer, true);
        }
        if volumes & 2 != 0 {
            o.add_volume_mesh(ModelMesh::from_its(box_its(4., 7., 0.95, 6., -7., 2.35)), ModelVolumeType::SupportBlocker, true);
        }
        o.add_instance().set_offset(&slicer::geometry::geometry::Vec3d::new(ix, iy, 0.0));
    }
    let mut config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(3).unwrap();
    config.set_key_value(
        "filament_colour",
        Box::new(ConfigOptionStrings::from_values(vec!["#FF0000".into(), "#00FF00".into(), "#0000FF".into()])),
    );
    config.set_key_value("filament_map", Box::new(ConfigOptionInts::from_values(vec![1, 1, 1])));
    while let Some(line) = r.input(&format!("in {s} cfg")) {
        let (key, value) = line.split_once(' ').expect("key value");
        deser(&mut config, key, value);
    }
    let mut print = assemble(&model, &config);
    let mut obj = print.m_objects.remove(0);
    let sp = obj.slicing_parameters().clone();
    r.out(format!(
        "{s} slicing {} {} {} {} {} {} {} {} {} {} {} {}",
        hd(sp.layer_height),
        hd(sp.first_print_layer_height),
        hd(sp.first_object_layer_height),
        sp.raft_layers(),
        hd(sp.raft_interface_top_z),
        hd(sp.raft_contact_top_z),
        hd(sp.contact_raft_layer_height),
        hd(sp.object_print_z_min),
        hb(sp.soluble_interface),
        hd(sp.gap_support_object),
        hd(sp.gap_object_support),
        hd(sp.max_suport_layer_height)
    ));

    // The object layers.
    while let Some(line) = r.input(&format!("in {s} layer")) {
        let mut t = Tok::new(&line);
        let id = t.i() as usize;
        let (print_z, height, slice_z) = (t.d(), t.d(), t.d());
        let mut layer = Layer::new(id, obj.m_model_object, height, print_z, slice_z);
        layer.lslices = t.expolys();
        layer.lslices_bboxes = layer.lslices.iter().map(|e| BBox::from_points(&e.contour.points).to_bounding_box()).collect();
        let line = r.input(&format!("in {s} region")).expect("region");
        let mut t = Tok::new(&line);
        let mut lr = LayerRegion::new(id, 0);
        lr.slices.surfaces = tok_surfs(&mut t);
        lr.raw_slices.surfaces = t.expolys().into_iter().map(|e| Surface::new(SurfaceType::Internal, e)).collect();
        lr.fill_surfaces.surfaces = tok_surfs(&mut t);
        lr.perimeters.entities = tok_ents(&mut t);
        lr.fills.entities = tok_ents(&mut t);
        lr.unsupported_bridge_edges = tok_pls(&mut t);
        layer.add_region(lr);
        let idx = obj.m_layers.len();
        if idx > 0 {
            layer.lower_layer_id = Some(idx - 1);
            obj.m_layers[idx - 1].upper_layer_id = Some(idx);
        }
        obj.m_layers.push(layer);
    }
    let n_layers = obj.m_layers.len();
    let n_pillar = 12usize;

    let mut psm = PrintObjectSupportMaterial::new(&obj, &print, &sp).expect("PrintObjectSupportMaterial");
    r.out(format!(
        "{s} psm {} {} {} {} {} {} {}",
        hb(psm.has_raft()),
        hb(psm.has_support()),
        hb(psm.build_plate_only()),
        hb(psm.synchronize_layers()),
        hb(psm.has_contact_loops()),
        hd(psm.m_support_params.gap_xy),
        hd(psm.m_support_params.support_layer_height_min)
    ));
    let gap_xy = psm.m_support_params.gap_xy;

    // SupportMaterialInternal
    for i in 0..n_layers {
        let layer = &obj.m_layers[i];
        let lr = &layer.regions()[0];
        let mut areas: Polygons = Vec::new();
        collect_bridging_perimeter_areas(&lr.perimeters, (gap_xy / slicer::libslic3r::SCALING_FACTOR) as f32, &mut areas);
        r.out(format!(
            "{s} internal {i} {} {} {} {}",
            hb(has_bridging_extrusions(layer)),
            hb(has_bridging_perimeters(&lr.perimeters)),
            hb(has_bridging_fills(&lr.fills)),
            polys_s(&areas)
        ));
        if i >= n_pillar && i <= n_pillar + 1 {
            let lower = &obj.m_layers[i - 1];
            let lower_polygons: Polygons = slicer::geometry::to_polygons(&lower.lslices);
            let mut contact: Polygons = cu::diff(&slicer::surface::to_polygons(&lr.slices.surfaces), &lower_polygons, cu::ApplySafetyOffset::No);
            let region = obj.printing_region(0);
            let fw = print_region_flow(&region.borrow(), &obj, &print, FlowRole::ExternalPerimeter, layer.height, layer.id() == 0).expect("flow").scaled_width() as f32;
            remove_bridges_from_contacts(&obj, &print, print.config(), lower, &lower_polygons, layer, lr, fw, &mut contact).expect("remove_bridges_from_contacts");
            r.out(format!("{s} rbfc {i} {} {}", hf(fw), polys_s(&contact)));
        }
    }
    // sync_gap_with_object_layer
    {
        let gaps = [0., 0.05, 0.1, 0.2, 0.25, 0.3, 0.45, 0.61];
        let idx = |l: Option<usize>| l.map(|v| v as i64).unwrap_or(-1);
        for i in [1, 2, 5, n_layers - 2] {
            for gap in gaps {
                r.out(format!(
                    "{s} sync {i} {} {} {}",
                    hd(gap),
                    idx(sync_gap_with_object_layer(&obj.m_layers, &obj.m_layers[i], gap, true)),
                    idx(sync_gap_with_object_layer(&obj.m_layers, &obj.m_layers[i], gap, false))
                ));
            }
        }
    }

    let mut buildplate_covered: Vec<Polygons> = psm.buildplate_covered(&obj);
    r.out(format!("{s} bp {}", polysv_s(&buildplate_covered)));

    // detect_overhangs layer by layer.
    {
        let annotations = SupportAnnotations::new(&obj, &print, &buildplate_covered, None).expect("SupportAnnotations");
        r.out(format!("{s} annotations {} {}", polysv_s(&annotations.enforcers_layers), polysv_s(&annotations.blockers_layers)));
        let num_layers = if psm.has_support() { obj.layer_count() } else { 1 };
        let layer_id_start = if psm.has_raft() { 0 } else { 1 };
        for layer_id in layer_id_start..num_layers {
            let layer = &obj.m_layers[layer_id];
            let lower_layer_polygons: Polygons = if layer_id == 0 { Vec::new() } else { slicer::geometry::to_polygons(&obj.m_layers[layer_id - 1].lslices) };
            let (mut sharp_tails, mut sharp_tails_height, mut cantilevers) = (Vec::new(), Vec::new(), Vec::new());
            let overhangs = detect_overhangs(
                &obj,
                &print,
                layer,
                layer_id,
                &lower_layer_polygons,
                print.config(),
                obj.config(),
                &annotations,
                gap_xy,
                &mut sharp_tails,
                &mut sharp_tails_height,
                &mut cantilevers,
            )
            .expect("detect_overhangs");
            r.out(format!(
                "{s} ov {layer_id} {} {} {} {}",
                expolys_s(&overhangs),
                expolys_s(&sharp_tails),
                floats_s(&sharp_tails_height),
                expolys_s(&cantilevers)
            ));
        }
    }

    let mut storage = SupportGeneratorLayerStorage::new();
    let top_contacts = psm.top_contact_layers(&obj, &buildplate_covered, &mut storage).expect("top_contact_layers");
    dump_storage(r, &format!("{s} top"), &storage);
    r.out(format!("{s} top list {}", list_s(&top_contacts)));
    for i in 0..n_layers {
        r.out(format!(
            "{s} tails {i} {} {} {}",
            expolys_s(&psm.layer_sharp_tails[i]),
            floats_s(&psm.layer_sharp_tails_height[i]),
            expolys_s(&psm.layer_cantilevers[i])
        ));
    }
    // The later stages, when the vectors have them.
    if !r.expected.iter().any(|l| l.starts_with(&format!("{s} bottom "))) {
        return true;
    }

    let mut layer_support_areas: Vec<Polygons> = Vec::new();
    let bottom_contacts = psm.bottom_contact_layers_and_layer_support_areas(&obj, &top_contacts, &mut buildplate_covered, &mut storage, &mut layer_support_areas);
    dump_storage(r, &format!("{s} bottom"), &storage);
    r.out(format!("{s} bottom list {}", list_s(&bottom_contacts)));
    r.out(format!("{s} bottom areas {}", polysv_s(&layer_support_areas)));
    r.out(format!("{s} bottom bp {}", polysv_s(&buildplate_covered)));

    let intermediate_layers = psm.raft_and_intermediate_support_layers(&obj, &bottom_contacts, &top_contacts, &mut storage);
    dump_storage(r, &format!("{s} inter"), &storage);
    r.out(format!("{s} inter list {}", list_s(&intermediate_layers)));

    psm.trim_support_layers_by_object(&obj, &top_contacts, sp.gap_support_object, sp.gap_object_support, gap_xy, &mut storage);
    dump_storage(r, &format!("{s} trim"), &storage);

    psm.generate_base_layers(&obj, &bottom_contacts, &top_contacts, &intermediate_layers, &layer_support_areas, &mut storage);
    dump_storage(r, &format!("{s} base"), &storage);

    psm.trim_top_contacts_by_bottom_contacts(&obj, &bottom_contacts, &top_contacts, &mut storage);
    dump_storage(r, &format!("{s} ttb"), &storage);
    if !r.expected.iter().any(|l| l.starts_with(&format!("{s} gen "))) {
        return true;
    }

    // generate(): the support layers it leaves on the object, without their extrusions.
    psm.generate(&mut obj, &NoFillFactory).expect("generate");
    r.out(format!("{s} gen {}", obj.support_layers().len()));
    for sl in obj.support_layers() {
        r.out(format!(
            "{s} gen L {} {} {} {} {} {} {} {} {} {} {} {}",
            sl.layer.id(),
            sl.interface_id(),
            hd(sl.layer.print_z),
            hd(sl.layer.height),
            hd(sl.layer.slice_z),
            sl.planned_filament_id,
            hd(sl.planned_nozzle_diameter),
            sl.planned_interface_filament_id,
            hd(sl.planned_interface_nozzle_diameter),
            hd(sl.base_slab_height),
            hd(sl.interface_slab_height),
            expolys_s(&sl.support_islands)
        ));
    }
    for i in 0..n_layers {
        r.out(format!(
            "{s} gen tails {i} {} {} {}",
            expolys_s(&psm.layer_sharp_tails[i]),
            floats_s(&psm.layer_sharp_tails_height[i]),
            expolys_s(&psm.layer_cantilevers[i])
        ));
    }
    true
}

/// A filler that fills nothing: the extrusions of the supports are not compared (they are the fillers').
struct NoFill {
    base: Fill,
}
impl SupportFill for NoFill {
    fn fill(&self) -> &Fill {
        &self.base
    }
    fn fill_mut(&mut self) -> &mut Fill {
        &mut self.base
    }
    fn fill_surface(&mut self, _surface: &Surface, _params: &FillParams) -> Result<Polylines, InfillFailedException> {
        Ok(Vec::new())
    }
}
struct NoFillFactory;
impl SupportFillFactory for NoFillFactory {
    fn new_from_type(&self, _pattern: InfillPattern) -> Box<dyn SupportFill> {
        Box::new(NoFill { base: Fill::new() })
    }
}

#[test]
fn support_material_native_parity() {
    let mut r = Run::load("support_material");
    run_grid(&mut r);
    let mut scenarios = 0;
    while run_object(&mut r, scenarios) {
        scenarios += 1;
    }
    eprintln!("support_material: {scenarios} object scenarios");
    let n = r.expected.iter().filter(|l| !l.starts_with("in ")).count();
    for tag in ["rast ", "dil ", "seed ", "cs ", "isl ", "par ", "pat ", "ext "] {
        let (total, ok) = r.tally(tag);
        eprintln!("support_material: {tag}: {ok} of {total}");
    }
    for stage in ["slicing", "psm", "internal", "rbfc", "sync", "bp", "annotations", "ov", "top", "tails", "bottom", "inter", "trim", "base", "ttb", "gen"] {
        let (mut total, mut ok) = (0, 0);
        for k in 0..scenarios {
            let (t, o) = r.tally(&format!("s{k} {stage} "));
            total += t;
            ok += o;
        }
        eprintln!("support_material: stage {stage}: {ok} of {total}");
    }
    let bad = r.finish();
    eprintln!("support_material: {} result lines, {} differ", n, bad);
    assert_eq!(bad, 0);
}
