//! `slicer::gcode::tool_ordering` (the port of libslic3r/GCode/ToolOrdering.cpp) against the C++ at
//! the pin.
//!
//! `tests/data/tool_ordering/vectors.txt.gz` is the output of `gen.cpp` beside it (gzip -9n), which
//! compiles the pin's ToolOrdering.cpp into itself and drives its free functions, GroupReorder and
//! the member functions of ToolOrdering and LayerTools on synthetic layers, configurations and
//! nozzle group results. The header of gen.cpp describes the lines. Every operation is repeated
//! here on the Rust port, and what it returns has to be the same: floats and doubles bit for bit,
//! integers and texts equal.

use slicer::config::ConfigBase;
use slicer::custom_g_code;
use slicer::extrusion_entity::{
    ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionLoopRole, ExtrusionMultiPath, ExtrusionPath,
    ExtrusionRole,
};
use slicer::gcode::tool_ordering::*;
use slicer::multi_nozzle_utils::{LayeredNozzleGroupResult, NozzleInfo};
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/tool_ordering/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn hex(s: &[u8]) -> String {
    if s.is_empty() {
        return "-".to_string();
    }
    s.iter().map(|c| format!("{:02x}", c)).collect()
}
fn unhex(t: &str) -> String {
    if t == "-" {
        return String::new();
    }
    let b: Vec<u8> = (0..t.len() / 2).map(|i| u8::from_str_radix(&t[2 * i..2 * i + 2], 16).expect("hex")).collect();
    String::from_utf8(b).expect("the vectors are UTF-8")
}
fn ff(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}
fn dd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}

/// A cursor over the tokens of one line.
struct Toks<'a> {
    t: Vec<&'a str>,
    i: usize,
}

impl<'a> Toks<'a> {
    fn new(s: &'a str) -> Self {
        Toks { t: s.split(' ').filter(|x| !x.is_empty()).collect(), i: 0 }
    }
    fn s(&mut self) -> &'a str {
        let v = self.t[self.i];
        self.i += 1;
        v
    }
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn u(&mut self) -> usize {
        self.i() as usize
    }
    fn b(&mut self) -> bool {
        self.i() != 0
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    fn text(&mut self) -> String {
        unhex(self.s())
    }
    fn list_u32(&mut self) -> Vec<u32> {
        let n = self.u();
        (0..n).map(|_| self.i() as u32).collect()
    }
    fn list_i32(&mut self) -> Vec<i32> {
        let n = self.u();
        (0..n).map(|_| self.i() as i32).collect()
    }
    fn list_usize(&mut self) -> Vec<usize> {
        let n = self.u();
        (0..n).map(|_| self.u()).collect()
    }
    fn arrow(&mut self) {
        assert_eq!(self.s(), "=>");
    }
    fn cfg(&mut self) -> ToolOrderingPrintConfig {
        assert_eq!(self.s(), "C");
        let n = self.u();
        let mut cfg = ToolOrderingPrintConfig::full_print_config();
        for _ in 0..n {
            let k = self.text();
            let v = self.text();
            // As the generator: an enum vector is given the definition's key map before names are
            // deserialized into it (its default value has none, Config.hpp:2135).
            if let Some(def) = slicer::print_config::config_def::print_config_def().base.get(&k) {
                if def.r#type == slicer::config::ConfigOptionType::CoEnums {
                    if let Ok(Some(o)) = cfg.option_mut(&k, true) {
                        if let Some(e) = o.as_any_mut().downcast_mut::<slicer::config::ConfigOptionEnumsGeneric>() {
                            e.keys_map = def.enum_keys_map;
                        }
                    }
                }
            }
            cfg.set_deserialize_strict(&k, &v, false).unwrap_or_else(|e| panic!("{k}={v}: {e:?}"));
        }
        cfg
    }
    fn layers(&mut self) -> Vec<LayerTools> {
        assert_eq!(self.s(), "L");
        let n = self.u();
        (0..n)
            .map(|_| {
                let mut lt = LayerTools::new(self.d());
                lt.has_object = self.b();
                lt.has_support = self.b();
                lt.extruders = self.list_u32();
                lt
            })
            .collect()
    }
    fn nozzles(&mut self) -> Vec<NozzleInfo> {
        let n = self.u();
        (0..n)
            .map(|_| {
                let mut ni = NozzleInfo::default();
                ni.diameter = self.text();
                ni.volume_type = slicer::extruder::NozzleVolumeType::from_i32(self.i() as i32);
                ni.extruder_id = self.i() as i32;
                ni.group_id = self.i() as i32;
                ni
            })
            .collect()
    }
}

/// What the C++ writes, token by token.
#[derive(Default)]
struct Out(Vec<String>);

impl Out {
    fn s(&mut self, v: impl Into<String>) {
        self.0.push(v.into());
    }
    fn i(&mut self, v: i64) {
        self.0.push(v.to_string());
    }
    fn d(&mut self, v: f64) {
        self.0.push(dd(v));
    }
    fn f(&mut self, v: f32) {
        self.0.push(ff(v));
    }
    fn text(&mut self, v: &str) {
        self.0.push(hex(v.as_bytes()));
    }
    fn list<T: Copy + Into<i64>>(&mut self, v: &[T]) {
        self.i(v.len() as i64);
        for &x in v {
            self.i(x.into());
        }
    }
    fn layers(&mut self, lts: &[LayerTools]) {
        self.s("L");
        self.i(lts.len() as i64);
        for lt in lts {
            self.d(lt.print_z);
            self.i(lt.has_object as i64);
            self.i(lt.has_support as i64);
            self.list(&lt.extruders);
        }
    }
    fn layers_full(&mut self, lts: &[LayerTools]) {
        self.i(lts.len() as i64);
        for lt in lts {
            self.d(lt.print_z);
            self.list(&lt.extruders);
            self.i(lt.has_wipe_tower as i64);
            self.i(lt.wipe_tower_partitions as i64);
            self.d(lt.wipe_tower_layer_height);
            self.i(lt.has_skirt as i64);
        }
    }
    fn nozzles(&mut self, list: &[NozzleInfo]) {
        self.i(list.len() as i64);
        for n in list {
            self.text(&n.diameter);
            self.i(n.volume_type as i64);
            self.i(n.extruder_id as i64);
            self.i(n.group_id as i64);
        }
    }
    fn line(&self) -> String {
        self.0.join(" ")
    }
}

/// The C++ `ExtrusionRole` values, in the order of the enum (ExtrusionEntity.hpp:45-70).
const ROLES: [ExtrusionRole; 22] = [
    ExtrusionRole::None,
    ExtrusionRole::Perimeter,
    ExtrusionRole::ExternalPerimeter,
    ExtrusionRole::OverhangPerimeter,
    ExtrusionRole::InternalInfill,
    ExtrusionRole::SolidInfill,
    ExtrusionRole::FloatingVerticalShell,
    ExtrusionRole::TopSolidInfill,
    ExtrusionRole::BottomSurface,
    ExtrusionRole::Ironing,
    ExtrusionRole::BridgeInfill,
    ExtrusionRole::GapFill,
    ExtrusionRole::Skirt,
    ExtrusionRole::Brim,
    ExtrusionRole::SupportMaterial,
    ExtrusionRole::SupportMaterialInterface,
    ExtrusionRole::SupportTransition,
    ExtrusionRole::SupportIroning,
    ExtrusionRole::WipeTower,
    ExtrusionRole::Custom,
    ExtrusionRole::Flush,
    ExtrusionRole::Mixed,
];

fn role_index(r: ExtrusionRole) -> i64 {
    ROLES.iter().position(|&x| x == r).expect("role") as i64
}

fn path(t: &mut Toks) -> ExtrusionPath {
    let mut p = ExtrusionPath::new(ROLES[t.u()]);
    p.height = t.f() as f64;
    p
}

fn coll(t: &mut Toks) -> ExtrusionEntityCollection {
    let n = t.u();
    let mut c = ExtrusionEntityCollection::new();
    for _ in 0..n {
        match t.s() {
            "P" => c.append(ExtrusionEntityType::Path(path(t))),
            "M" => {
                let k = t.u();
                let mut mp = ExtrusionMultiPath::new();
                for _ in 0..k {
                    mp.paths.push(path(t));
                }
                c.append(ExtrusionEntityType::MultiPath(mp));
            }
            "O" => {
                let k = t.u();
                let ps: Vec<ExtrusionPath> = (0..k).map(|_| path(t)).collect();
                c.append(ExtrusionEntityType::Loop(ExtrusionLoop::new(ps, ExtrusionLoopRole::default())));
            }
            "C" => c.append(ExtrusionEntityType::Collection(Box::new(coll(t)))),
            k => panic!("entity kind {k}"),
        }
    }
    c
}

/// The `Print` the member functions read, as the generator builds it.
struct TestPrint {
    config: ToolOrderingPrintConfig,
    custom_gcodes: custom_g_code::Info,
    group: Option<LayeredNozzleGroupResult>,
}

impl ToolOrderingPrint for TestPrint {
    fn config(&self) -> &ToolOrderingPrintConfig {
        &self.config
    }
    fn full_print_config(&self) -> &ToolOrderingPrintConfig {
        &self.config
    }
    fn uses_multiple_nozzle_diameters(&self) -> bool {
        false
    }
    fn object_extruders(&self) -> Vec<u32> {
        // a Print without objects
        Vec::new()
    }
    fn curr_plate_custom_gcodes(&self) -> &custom_g_code::Info {
        &self.custom_gcodes
    }
    fn get_layered_nozzle_group_result(&self) -> Option<&LayeredNozzleGroupResult> {
        self.group.as_ref()
    }
    fn get_geometric_unprintable_filaments(&self) -> Vec<BTreeSet<i32>> {
        Vec::new()
    }
    fn get_physical_unprintable_filaments(&self, _used_filaments: &[u32]) -> Vec<BTreeSet<i32>> {
        Vec::new()
    }
    fn get_filament_unprintable_flow(&self, _used_filaments: &[u32]) -> BTreeMap<i32, BTreeSet<slicer::extruder::NozzleVolumeType>> {
        BTreeMap::new()
    }
}

fn ordering(lts: Vec<LayerTools>) -> ToolOrdering {
    let mut to = ToolOrdering::cpp_default();
    *to.layer_tools_mut() = lts;
    to
}

/// The `Print` of the "build" lines: the objects the generator built by hand.
struct BuildPrint {
    config: ToolOrderingPrintConfig,
    has_skirt: bool,
    objects: Vec<ToolOrderingObject>,
    custom_gcodes: custom_g_code::Info,
}

impl ToolOrderingPrint for BuildPrint {
    fn config(&self) -> &ToolOrderingPrintConfig {
        &self.config
    }
    fn full_print_config(&self) -> &ToolOrderingPrintConfig {
        &self.config
    }
    fn uses_multiple_nozzle_diameters(&self) -> bool {
        false
    }
    fn object_extruders(&self) -> Vec<u32> {
        // model objects without volumes
        Vec::new()
    }
    fn curr_plate_custom_gcodes(&self) -> &custom_g_code::Info {
        &self.custom_gcodes
    }
    fn get_layered_nozzle_group_result(&self) -> Option<&LayeredNozzleGroupResult> {
        None
    }
    fn get_geometric_unprintable_filaments(&self) -> Vec<BTreeSet<i32>> {
        Vec::new()
    }
    fn get_physical_unprintable_filaments(&self, _used_filaments: &[u32]) -> Vec<BTreeSet<i32>> {
        Vec::new()
    }
    fn get_filament_unprintable_flow(&self, _used_filaments: &[u32]) -> BTreeMap<i32, BTreeSet<slicer::extruder::NozzleVolumeType>> {
        BTreeMap::new()
    }
}

impl ToolOrderingPrintObjects for BuildPrint {
    fn as_tool_ordering_print(&self) -> &dyn ToolOrderingPrint {
        self
    }
    fn objects(&self) -> Vec<&ToolOrderingObject> {
        self.objects.iter().collect()
    }
    fn has_skirt(&self) -> bool {
        self.has_skirt
    }
    fn extruders(&self) -> Vec<u32> {
        Vec::new()
    }
    fn nozzle_diameter_for_filament(&self, _filament: u32) -> f32 {
        // not read: multi-diameter mode is off, extrusion_width_for_nozzle() keeps the width
        0.4
    }
}

fn expolys(t: &mut Toks) -> Vec<slicer::geometry::ExPolygon> {
    let n = t.u();
    (0..n)
        .map(|_| {
            let k = t.u();
            let pts: Vec<slicer::geometry::Point> = (0..k).map(|_| slicer::geometry::Point::new(t.i(), t.i())).collect();
            slicer::geometry::ExPolygon::new(slicer::geometry::Polygon::from_points(pts))
        })
        .collect()
}

fn dump_build(o: &mut Out, to: &ToolOrdering) {
    o.i(to.layer_tools().len() as i64);
    for lt in to.layer_tools() {
        o.d(lt.print_z);
        o.list(&lt.extruders);
        o.i(lt.has_object as i64);
        o.i(lt.has_support as i64);
        o.i(lt.has_skirt as i64);
        o.i(lt.extruder_override as i64);
        o.i(lt.wiping_extrusions().is_anything_overridable() as i64);
    }
    o.i(to.first_extruder().unwrap_or(u32::MAX) as i64);
    o.i(to.last_extruder().unwrap_or(u32::MAX) as i64);
    o.list(to.all_extruders());
    o.i(to.bed_contact_filaments.len() as i64);
    for &(z, f) in &to.bed_contact_filaments {
        o.d(z);
        o.i(f as i64);
    }
}

fn dump_outputs(o: &mut Out, out: &ToolOrderingObjectOutputs) {
    o.list(&out.object_first_layer_wall_extruders);
    o.i(out.object_brim_filament as i64);
    o.d(out.object_brim_print_z);
}

/// Runs one line; `Some((got, want))` when they differ.
fn run_line(line: &str) -> Option<(String, String)> {
    let (inp, want) = line.split_once(" => ").expect("=>");
    let want_line = want.to_string();
    let mut t = Toks::new(inp);
    let op = t.s();
    let mut o = Out::default();
    match op {
        "calc_max_layer_height" => {
            let cfg = t.cfg();
            let moh = t.d();
            o.d(calc_max_layer_height(&cfg, moh));
        }
        "snap_to_simple_fraction" => {
            let r = t.d();
            let md = t.i() as i32;
            o.d(snap_to_simple_fraction(r, md));
        }
        "get_filament_by_type" => {
            let cfg = t.cfg();
            let used = t.list_u32();
            let ty = t.text();
            let r: Vec<i32> = get_filament_by_type(&used, &cfg, &ty).into_iter().collect();
            o.list(&r);
        }
        "check_filament_printable_after_group" => {
            let cfg = t.cfg();
            let used = t.list_u32();
            let maps = t.list_i32();
            let n = t.u();
            let mut fv: HashMap<i32, Vec<String>> = HashMap::new();
            for _ in 0..n {
                let k = t.i() as i32;
                let m = t.u();
                fv.insert(k, (0..m).map(|_| t.text()).collect());
            }
            let mut ut = Vec::new();
            match check_filament_printable_after_group(&used, &maps, &mut fv, &mut ut, &cfg) {
                Ok(r) => o.i(r as i64),
                Err(e) => {
                    o.s("!R");
                    o.text(&e.message);
                }
            }
        }
        "calc_filament_change_info_by_toolorder" => {
            let cfg = t.cfg();
            let map = t.list_i32();
            let nozzles = t.nozzles();
            let used = t.list_u32();
            let ne = t.u();
            let nf = t.u();
            let fm: Vec<Vec<Vec<f32>>> = (0..ne).map(|_| (0..nf).map(|_| (0..nf).map(|_| t.f()).collect()).collect()).collect();
            let ns = t.u();
            let seqs: Vec<Vec<u32>> = (0..ns).map(|_| t.list_u32()).collect();
            let gr = LayeredNozzleGroupResult::create(&map, &nozzles, &used).expect("group");
            let st = calc_filament_change_info_by_toolorder(&cfg, &gr, &fm, &seqs);
            o.i(st.filament_flush_weight as i64);
            o.i(st.flush_filament_change_count as i64);
            o.i(st.filament_change_count as i64);
        }
        "handle_dontcare_extruder_vec" => {
            let lts = t.layers();
            let order = t.list_u32();
            let mut to = ordering(lts);
            HandleDontcareExtruder::handle_dontcare_extruder(&mut to, &order[..]);
            o.layers(to.layer_tools());
        }
        "handle_dontcare_extruder_u32" => {
            let lts = t.layers();
            let has_cfg = t.b();
            let cfg = t.cfg();
            let last = t.i() as u32;
            let mut to = ordering(lts);
            if has_cfg {
                to.print_config_ptr = Some(ToolOrderingConfigPtr(std::sync::Arc::new(cfg)));
            }
            HandleDontcareExtruder::handle_dontcare_extruder(&mut to, last);
            o.layers(to.layer_tools());
        }
        "initialize_layers" => {
            let n = t.u();
            let zs: Vec<f64> = (0..n).map(|_| t.d()).collect();
            let mut to = ToolOrdering::cpp_default();
            to.initialize_layers(zs);
            o.i(to.layer_tools().len() as i64);
            for lt in to.layer_tools() {
                o.d(lt.print_z);
            }
        }
        "fill_wipe_tower_partitions" => {
            let cfg = t.cfg();
            let lts = t.layers();
            let bottom = t.d();
            let mlh = t.d();
            let mut to = ordering(lts);
            ToolOrderingCpp::fill_wipe_tower_partitions(&mut to, &cfg, bottom, mlh);
            o.layers_full(to.layer_tools());
        }
        "mark_skirt_layers" => {
            let lts = t.layers();
            let mlh = t.d();
            if want_line == "skip" {
                return None;
            }
            let mut to = ordering(lts);
            ToolOrderingCpp::mark_skirt_layers(&mut to, &ToolOrderingPrintConfig::full_print_config(), mlh, None);
            o.layers_full(to.layer_tools());
        }
        "collect_extruder_statistics" => {
            let lts = t.layers();
            let prime = t.b();
            let mut to = ordering(lts);
            to.collect_extruder_statistics(prime);
            o.i(to.first_extruder().unwrap_or(u32::MAX) as i64);
            o.i(to.last_extruder().unwrap_or(u32::MAX) as i64);
            o.list(to.all_extruders());
        }
        "seed_first_tower_layer" => {
            let mut lts = t.layers();
            let n = t.u();
            for lt in lts.iter_mut().take(n) {
                lt.has_wipe_tower = t.b();
            }
            let f = t.i() as u32;
            let mut to = ordering(lts);
            to.seed_first_tower_layer(f);
            o.layers(to.layer_tools());
        }
        "cal_max_additional_fan" => {
            let cfg = t.cfg();
            let lts = t.layers();
            let mut to = ordering(lts);
            o.f(to.cal_max_additional_fan(&cfg));
        }
        "cal_non_support_filaments" => {
            let cfg = t.cfg();
            let lts = t.layers();
            let ne = t.u();
            let mut to = ordering(lts);
            to.collect_extruder_statistics(false);
            let mut first = u32::MAX;
            let mut init_ns = vec![-1i32; ne];
            let mut init = vec![-1i32; ne];
            let hns = to.has_non_support_filament(&cfg);
            let r = to.cal_non_support_filaments(&cfg, &mut first, &mut init_ns, &mut init);
            o.i(hns as i64);
            o.i(r as i64);
            o.i(first as i64);
            o.list(&init_ns);
            o.list(&init);
        }
        "calc_most_used_extruder" => {
            let cfg = t.cfg();
            let map = t.list_i32();
            let nozzles = t.nozzles();
            let used = t.list_u32();
            let lts = t.layers();
            let print = TestPrint {
                config: ToolOrderingPrintConfig::full_print_config(),
                custom_gcodes: custom_g_code::Info::new(),
                group: LayeredNozzleGroupResult::create(&map, &nozzles, &used),
            };
            let mut to = ordering(lts);
            to.calc_most_used_extruder(&cfg, &print);
            o.i(to.get_most_used_extruder() as i32 as i64);
        }
        "prepare_flush_matrices" => {
            let cfg = t.cfg();
            let m = group_reorder::prepare_flush_matrices(&cfg);
            o.i(m.len() as i64);
            for mm in &m {
                o.i(mm.len() as i64);
                for row in mm {
                    o.i(row.len() as i64);
                    for &v in row {
                        o.f(v);
                    }
                }
            }
        }
        "nozzle_lists" => {
            let cfg = t.cfg();
            let ne = t.u();
            let groups = group_reorder::build_nozzle_groups(&cfg, ne);
            o.i(groups.len() as i64);
            for g in &groups {
                o.text(&g.diameter);
                o.i(g.volume_type as i64);
                o.i(g.extruder_id as i64);
                o.i(g.nozzle_count as i64);
            }
            let list = group_reorder::build_default_nozzle_list(&cfg, ne);
            o.nozzles(&list);
        }
        "create_custom_seq_function" => {
            let cfg = t.cfg();
            let include_first = t.b();
            let first = t.list_u32();
            let f = group_reorder::create_custom_seq_function(&cfg, include_first, &first);
            for l in 0..16 {
                let mut out = vec![-5i32];
                let r = f(l, &mut out);
                o.i(r as i64);
                o.list(&out);
            }
        }
        "validate_manual_diameter_or_throw" => {
            let cfg = t.cfg();
            let used = t.list_u32();
            let map = t.list_i32();
            let mode = t.b();
            match validate_manual_diameter_or_throw(&cfg, &used, &map, mode) {
                Ok(()) => o.s("ok"),
                Err(e) => {
                    o.s("!R");
                    o.text(&e.message);
                }
            }
        }
        "slots_and_bed" => {
            let c = coll(&mut t);
            let mut rc = slicer::region_config::PrintRegionConfig::default();
            rc.wall_filament = t.u();
            rc.sparse_infill_filament = t.u();
            rc.solid_infill_filament = t.u();
            let region = slicer::print_region::PrintRegion::new(rc.clone());
            let mut lt = LayerTools::new(1.0);
            lt.extruder_override = t.i() as u32;
            let nres = t.u();
            for _ in 0..nres {
                let k = t.i() as u32;
                let v = t.i() as u32;
                lt.mixed_filament_resolution.insert(k, v);
            }
            let print_z = t.d();
            o.i(role_index(c.role()));
            o.i(LayerTools::filament_slot_by_role(&c, &rc) as i64);
            o.i(lt.filament_slot(&c, &region) as i64);
            o.i(lt.extruder(&c, &region) as i64);
            o.i(lt.wall_filament(rc.wall_filament as u32) as i64);
            o.i(lt.sparse_infill_filament(rc.sparse_infill_filament as u32) as i64);
            o.i(lt.solid_infill_filament(rc.solid_infill_filament as u32) as i64);
            o.i(c.entities.iter().any(|e| extrusion_touches_bed(e, print_z)) as i64);
        }
        "tools_for_layer" => {
            let lts = t.layers();
            let to = ordering(lts);
            let mut w = Toks::new(want);
            for _ in 0..to.layer_tools().len() {
                let z = w.d();
                w.s();
                o.d(z);
                o.i(to.tools_for_layer_index(z) as i64);
            }
        }
        "resolve_mixed_filaments" => {
            let cfg = t.cfg();
            let lts = t.layers();
            let mut to = ordering(lts);
            let n = t.u();
            for _ in 0..n {
                let m = t.i() as u32;
                let obj = t.u();
                let ls = t.list_usize();
                to.mixed_object_layers.entry(m).or_default().insert(obj, ls);
            }
            let n = t.u();
            for _ in 0..n {
                let obj = t.u();
                let ls = t.list_usize();
                to.object_all_layer_indices.insert(obj, ls);
            }
            let n = t.u();
            for _ in 0..n {
                let m = t.i() as u32;
                let obj = t.u();
                let vol = t.u();
                let ls = t.list_usize();
                to.gradient_volume_layers.entry(m).or_default().insert(VolumeKey { obj, volume_id: vol }, ls);
            }
            let n = t.u();
            for _ in 0..n {
                let m = t.i() as u32;
                let li = t.u();
                let h = t.d();
                to.mixed_layer_heights.entry(m).or_default().insert(li, h);
            }
            to.resolve_mixed_filaments(&cfg);
            // write_mixed
            o.i(to.layer_tools().len() as i64);
            for lt in to.layer_tools() {
                o.list(&lt.extruders);
                o.i(lt.mixed_filament_resolution.len() as i64);
                for (&k, &v) in &lt.mixed_filament_resolution {
                    o.i(k as i64);
                    o.i(v as i64);
                }
                o.i(lt.mixed_sub_layer_groups.len() as i64);
                for g in &lt.mixed_sub_layer_groups {
                    o.i(g.mixed_slot_0based as i64);
                    o.list(&g.components_0based);
                    o.i(g.sub_heights.len() as i64);
                    for &h in &g.sub_heights {
                        o.d(h);
                    }
                    o.d(g.layer_height);
                    o.i(g.is_gradient as i64);
                    o.i(g.gradient_first_sorted_idx as i64);
                    o.i(g.per_object_gradient.len() as i64);
                    for (&obj, og) in &g.per_object_gradient {
                        o.i(obj as i64);
                        o.i(og.total_layers as i64);
                        o.i(og.current_idx as i64);
                        o.d(og.gradient_start);
                        o.d(og.gradient_end);
                        o.i(og.curve.points.len() as i64);
                    }
                    o.i(g.per_volume_gradient.len() as i64);
                    for (vk, og) in &g.per_volume_gradient {
                        o.i(vk.obj as i64);
                        o.i(vk.volume_id as i64);
                        o.i(og.total_layers as i64);
                        o.i(og.current_idx as i64);
                    }
                }
            }
            o.list(to.used_mixed_filaments());
            o.list(to.directly_used_filaments());
            // the order the generator gives enforce_mixed_component_order(), read from its output
            let mut w = Toks::new(want);
            w.i = o.0.len();
            let nl = w.u();
            o.i(nl as i64);
            for li in 0..nl {
                let e = w.list_u32();
                o.list(&e);
                to.layer_tools_mut()[li].extruders = e;
            }
            to.enforce_mixed_component_order();
            o.i(to.layer_tools().len() as i64);
            for lt in to.layer_tools() {
                o.list(&lt.extruders);
            }
            // first_layer_extruders over the bed contacts the generator drew
            w.i = o.0.len();
            let nbc = w.u();
            o.i(nbc as i64);
            for _ in 0..nbc {
                let z = w.d();
                let f = w.i() as u32;
                o.d(z);
                o.i(f as i64);
                to.bed_contact_filaments.push((z, f));
            }
            o.list(&to.first_layer_extruders());
        }
        "assign_custom_gcodes" => {
            let cfg = t.cfg();
            let lts = t.layers();
            let mode = match t.i() {
                0 => custom_g_code::Mode::Undef,
                1 => custom_g_code::Mode::SingleExtruder,
                2 => custom_g_code::Mode::MultiAsSingle,
                _ => custom_g_code::Mode::MultiExtruder,
            };
            let n = t.u();
            let mut info = custom_g_code::Info::new_with_mode(mode);
            for _ in 0..n {
                let print_z = t.d();
                let ty = match t.i() {
                    0 => custom_g_code::Type::ColorChange,
                    1 => custom_g_code::Type::PausePrint,
                    2 => custom_g_code::Type::ToolChange,
                    3 => custom_g_code::Type::Template,
                    4 => custom_g_code::Type::Custom,
                    _ => custom_g_code::Type::Unknown,
                };
                let extruder = t.i() as i32;
                let color = t.text();
                let extra = t.text();
                let mut item = custom_g_code::Item::new(print_z, ty, extruder, color);
                item.extra = extra;
                info.gcodes.push(item);
            }
            let print = TestPrint { config: cfg, custom_gcodes: info, group: None };
            let mut to = ordering(lts);
            ToolOrderingCpp::assign_custom_gcodes(&mut to, &print);
            o.i(to.layer_tools().len() as i64);
            for lt in to.layer_tools() {
                match &lt.custom_gcode_item {
                    Some(item) => o.text(&item.extra),
                    None => o.text("none"),
                }
            }
        }
        "build" => {
            let cfg = t.cfg();
            let has_skirt = t.b();
            let nobj = t.u();
            let mut objects = Vec::new();
            for key in 0..nobj {
                let support_filament = t.i() as u32;
                let support_interface_filament = t.i() as u32;
                let support_interface_not_for_body = t.b();
                let flush_into_objects = t.b();
                let flush_into_infill = t.b();
                let flush_into_support = t.b();
                let line_width = t.d();
                let layer_height = t.d();
                let has_brim = t.b();
                let nreg = t.u();
                let nl = t.u();
                let mut layers = Vec::new();
                for _ in 0..nl {
                    let id = t.u();
                    let print_z = t.d();
                    let height = t.d();
                    let has_extrusions = t.b();
                    let nr = t.u();
                    let mut regions = Vec::new();
                    for _ in 0..nr {
                        let mut rc = slicer::region_config::PrintRegionConfig::default();
                        rc.wall_filament = t.u();
                        rc.sparse_infill_filament = t.u();
                        rc.solid_infill_filament = t.u();
                        let perimeters = coll(&mut t);
                        let fills = coll(&mut t);
                        let raw_slices = expolys(&mut t);
                        let slices_empty = t.b();
                        regions.push(ToolOrderingLayerRegion {
                            region: slicer::print_region::PrintRegion::new(rc),
                            gradient_volume_id: None,
                            perimeters,
                            fills,
                            raw_slices,
                            slices_empty,
                            adaptive_merge_patches: Vec::new(),
                        });
                    }
                    layers.push(ToolOrderingLayer { id, print_z, height, has_extrusions, regions });
                }
                let ns = t.u();
                let mut support_layers = Vec::new();
                for _ in 0..ns {
                    let print_z = t.d();
                    let support_fills = coll(&mut t);
                    let planned_filament_id = t.i() as i32;
                    let planned_interface_filament_id = t.i() as i32;
                    support_layers.push(ToolOrderingSupportLayer { print_z, support_fills, planned_filament_id, planned_interface_filament_id });
                }
                objects.push(ToolOrderingObject {
                    key,
                    layers,
                    support_layers,
                    raft_layers: 0,
                    max_layer_height_inputs: ObjectMaxLayerHeightInputs {
                        single_diameter_layering_nozzle: 0.0,
                        slicing_max_layer_height: 0.0,
                        layer_height,
                    },
                    printing_region_bridge_flows: vec![1.0; nreg],
                    support_filament,
                    support_interface_filament,
                    support_interface_not_for_body,
                    flush_into_objects,
                    flush_into_infill,
                    flush_into_support,
                    line_width,
                    has_brim,
                });
            }
            let first = t.i() as u32;
            let prime = t.b();
            let print = BuildPrint { config: cfg.clone(), has_skirt, objects, custom_gcodes: custom_g_code::Info::new() };
            let (to, outs) = ToolOrdering::from_print(&print, ToolOrderingConfigPtr(std::sync::Arc::new(cfg)), first, prime);
            dump_build(&mut o, &to);
            for out in &outs {
                dump_outputs(&mut o, out);
            }
            for object in &print.objects {
                let (to, out) = ToolOrdering::from_object(object, &print, first, prime);
                dump_build(&mut o, &to);
                dump_outputs(&mut o, &out);
            }
        }
        _ => panic!("unknown op {op}"),
    }
    let got = o.line();
    if got != want_line {
        return Some((got, want_line));
    }
    None
}

#[test]
fn tool_ordering_matches_the_pin() {
    let text = read_vectors();
    let mut counts: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    let mut first_failures: Vec<String> = Vec::new();
    for line in text.lines() {
        let op = line.split(' ').next().unwrap().to_string();
        let r = std::panic::catch_unwind(|| run_line(line));
        let entry = counts.entry(op.clone()).or_insert((0, 0));
        entry.0 += 1;
        match r {
            Ok(None) => {}
            Ok(Some((got, want))) => {
                entry.1 += 1;
                if first_failures.len() < 12 {
                    first_failures.push(format!("{}\n  got:  {}\n  want: {}", &line[..line.len().min(400)], got, want));
                }
            }
            Err(_) => {
                entry.1 += 1;
                if first_failures.len() < 12 {
                    first_failures.push(format!("PANIC {}", &line[..line.len().min(400)]));
                }
            }
        }
    }
    let mut report = String::new();
    let mut failed = 0;
    for (op, (n, bad)) in &counts {
        report.push_str(&format!("{op}: {n} cases, {bad} differ\n"));
        failed += bad;
    }
    std::fs::write(format!("{}/target/tool_ordering_native_parity.txt", env!("CARGO_MANIFEST_DIR").trim_end_matches("/crates/libslic3r-rs")), format!("{report}\n{}", first_failures.join("\n\n"))).ok();
    assert!(failed == 0, "{report}\n{}", first_failures.join("\n\n"));
}
