//! `slicer::format::bbs_3mf` (the port of libslic3r/Format/bbs_3mf.cpp) against the C++ at the pin.
//!
//! `tests/data/bbs_3mf/vectors.txt` is the output of `gen.cpp` beside it, which links the
//! pin's bbs_3mf.cpp; the header of gen.cpp describes the two parts and the line format:
//!  - load: the 3MFs of the repository read with `load_bbs_3mf` and the loaded model, config,
//!    presets and plates dumped;
//!  - store: two models built in code written with `store_bbs_3mf` (SaveStrategy::Zip64 and
//!    SplitModel | Zip64), the normalised text of every XML / JSON / config / txt entry of the
//!    archive, and the archive read back.
//!
//! The same dumps are made here with the port and have to be equal line for line.
//!
//! `BBS_3MF_EXTRA=<a.3mf> BBS_3MF_EXTRA_VECTORS=<the output of gen <repo> <scratch> <a.3mf>>`
//! compares the load of one more file (a project that is not part of the repository); without
//! them that test does nothing.

use std::collections::BTreeMap;
use std::io::Read;
use std::sync::Arc;

use ordered_float::OrderedFloat;
use slicer::config::{
    ConfigBase, ConfigOptionBool, ConfigOptionEnum, ConfigOptionFloat, ConfigOptionInt, ConfigOptionInts, ConfigOptionPercent,
    ConfigSubstitutionContext, ForwardCompatibilitySubstitutionRule,
};
use slicer::format::bbs_3mf::*;
use slicer::geometry::geometry::{Transform3d, Vec3d};
use slicer::model::{FacetsAnnotation, Model, ModelDesignInfo, ModelInfo, ModelMesh, ModelVolumeType};
use slicer::normal_utils::indexed_triangle_set;
use slicer::preset::Preset;
use slicer::print_config::config_def::DynamicPrintConfig;
use slicer::print_config::enums::{BedType, FilamentMapMode, PrintSequence};

/// One scratch directory for the process: the backup directories of the models
/// (Model::get_backup_path) are made under the global temporary directory, which both tests set.
fn scratch_dir() -> String {
    static DIR: std::sync::OnceLock<tempfile::TempDir> = std::sync::OnceLock::new();
    let dir = DIR.get_or_init(|| tempfile::tempdir().unwrap()).path().to_string_lossy().into_owned();
    slicer::utils::set_temporary_dir(&dir);
    dir
}

fn repo_root() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn hd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}
fn hf(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}
fn hb(v: bool) -> &'static str {
    if v {
        "1"
    } else {
        "0"
    }
}
fn ht(s: &str) -> String {
    if s.is_empty() {
        return "-".to_string();
    }
    s.bytes().map(|c| format!("{:02x}", c)).collect()
}
fn fnv(s: &[u8]) -> u64 {
    let mut h: u64 = 1469598103934665603;
    for &c in s {
        h ^= c as u64;
        h = h.wrapping_mul(1099511628211);
    }
    h
}
fn hx(v: u64) -> String {
    format!("{:016x}", v)
}
fn hm(t: &Transform3d) -> String {
    let mut o = Vec::new();
    for r in 0..3 {
        for c in 0..4 {
            o.push(hd(t[(r, c)]));
        }
    }
    o.join(" ")
}
fn v3(v: &Vec3d) -> String {
    format!("{} {} {}", hd(v.x), hd(v.y), hd(v.z))
}
fn its_digest(its: &indexed_triangle_set) -> String {
    let mut h: u64 = 1469598103934665603;
    let mut mix = |v: u32| {
        for k in 0..4 {
            h ^= ((v >> (8 * k)) & 0xff) as u64;
            h = h.wrapping_mul(1099511628211);
        }
    };
    for v in &its.vertices {
        for k in 0..3 {
            mix(v[k].to_bits());
        }
    }
    for f in &its.indices {
        for k in 0..3 {
            mix(f[k] as u32);
        }
    }
    format!("{} {} {}", its.vertices.len(), its.indices.len(), hx(h))
}
fn cfg(c: &DynamicPrintConfig) -> String {
    let keys = ConfigBase::keys(&**c);
    let mut s = String::new();
    for k in &keys {
        s.push_str(&format!("{}={}\n", k, c.opt_serialize(k).unwrap_or_default()));
    }
    format!("{} {}", keys.len(), hx(fnv(s.as_bytes())))
}
fn facets(f: &FacetsAnnotation) -> String {
    let d = &f.data;
    let mut s = String::new();
    for p in &d.0 {
        s.push_str(&format!("{},{};", p.0, p.1));
    }
    s.push('|');
    for &b in &d.1 {
        s.push(if b { '1' } else { '0' });
    }
    format!("{} {} {}", d.0.len(), d.1.len(), hx(fnv(s.as_bytes())))
}
fn basename_of(p: &str) -> String {
    let q = p.replace('\\', "/");
    match q.rfind('/') {
        Some(pos) => q[pos + 1..].to_string(),
        None => q,
    }
}
fn norm_path(p: &str, backup: &str) -> String {
    if !backup.is_empty() && p.starts_with(backup) {
        return format!("<backup>{}", &p[backup.len()..]);
    }
    p.to_string()
}

/// gen.cpp `LOADED`: `id_base` >= 0 writes a loaded identify ID relative to that model ID.
fn loaded(id: usize, id_base: i64) -> String {
    if id_base < 0 || id == 0 {
        // 0: no identify ID was loaded
        return id.to_string();
    }
    format!("@{}", id as i64 - id_base)
}

fn dump_model(
    out: &mut Vec<String>,
    model: &mut Model,
    plates: &PlateDataPtrs,
    presets: &[Preset],
    config: &DynamicPrintConfig,
    normalize_guid: bool,
    id_base: i64,
) {
    let backup = model.get_backup_path();
    out.push(format!("CFG {}", cfg(config)));
    for p in presets {
        out.push(format!("PRESET {} {} {} {}", p.r#type as i32, ht(&p.name), cfg(&p.config), ht(&p.version.to_string())));
    }
    match &model.model_info {
        Some(mi) => {
            out.push(format!(
                "MI {} {} {} {} {} {}",
                ht(&mi.model_name),
                ht(&mi.origin),
                ht(&mi.description),
                ht(&mi.license),
                ht(&mi.copyright),
                ht(&mi.cover_file)
            ));
            for (k, v) in &mi.metadata_items {
                out.push(format!("MD {} {}", ht(k), ht(v)));
            }
        }
        None => out.push("MI none".to_string()),
    }
    match &model.design_info {
        Some(di) => out.push(format!("DI {} {} {}", ht(&di.designer), ht(&di.designer_user_id), ht(&di.designer_id))),
        None => out.push("DI none".to_string()),
    }
    match &model.profile_info {
        Some(pi) => out.push(format!(
            "PI {} {} {} {} {} {}",
            ht(&pi.profile_tile),
            ht(&pi.profile_cover),
            ht(&pi.profile_description),
            ht(&pi.profile_user_id),
            ht(&pi.profile_user_name),
            ht(&model.design_id)
        )),
        None => out.push("PI none".to_string()),
    }
    out.push(format!("OBJECTS {}", model.objects.len()));
    for (oi, o) in model.objects.iter().enumerate() {
        out.push(format!("O {} {} {} {} {} {}", oi, ht(&o.name), ht(&o.module_name), o.instances.len(), o.volumes.len(), cfg(o.config.get())));
        for k in o.config.keys() {
            out.push(format!("OC {} {}", k, ht(&o.config.get().opt_serialize(&k).unwrap_or_default())));
        }
        {
            let lhp = o.layer_height_profile.get();
            let mut line = format!("LHP {}", lhp.len());
            for v in lhp {
                line.push_str(&format!(" {}", hd(v)));
            }
            out.push(line);
        }
        for (r, c) in &o.layer_config_ranges {
            out.push(format!("LCR {} {} {}", hd(r.0 .0), hd(r.1 .0), cfg(c.get())));
        }
        let mut line = format!("BP {}", o.brim_points.len());
        for p in &o.brim_points {
            line.push_str(&format!(" {} {} {} {} {}", hf(p.pos[0]), hf(p.pos[1]), hf(p.pos[2]), hf(p.head_front_radius), p.volume_idx));
        }
        out.push(line);
        out.push(format!("CUT {} {} {}", o.cut_id.id().id, o.cut_id.check_sum(), o.cut_id.connectors_cnt()));
        for (ii, i) in o.instances.iter().enumerate() {
            out.push(format!(
                "I {} {} {} {} {} {}",
                ii,
                hb(i.printable),
                loaded(i.loaded_id, id_base),
                hm(&i.matrix()),
                hb(i.is_assemble_initialized()),
                v3(&i.get_offset_to_assembly())
            ));
        }
        for (vi, v) in o.volumes.iter().enumerate() {
            let mut guid = v.part_guid();
            if normalize_guid && !guid.is_empty() {
                guid = "<guid>".to_string();
            }
            out.push(format!(
                "V {} {} {} {} {} {} {} {} {} {} {}",
                vi,
                ht(&v.name),
                v.type_() as i32,
                ht(&guid),
                ht(&basename_of(&v.source.input_file)),
                v.source.object_idx,
                v.source.volume_idx,
                v3(&v.source.mesh_offset),
                hb(v.source.is_converted_from_inches),
                hb(v.source.is_converted_from_meters),
                hm(&v.source.transform.matrix())
            ));
            out.push(format!("VM {} {}", hm(&v.matrix()), hb(v.is_assemble_initialized())));
            let e = v.mesh().stats().repaired_errors;
            out.push(format!(
                "VMESH {} {} {} {} {} {} {}",
                its_digest(&v.mesh().its),
                e.edges_fixed,
                e.degenerate_facets,
                e.facets_removed,
                e.facets_reversed,
                e.backwards_edges,
                v3(&v.mesh().get_init_shift())
            ));
            out.push(format!("VC {}", cfg(v.config.get())));
            for k in v.config.keys() {
                out.push(format!("VCK {} {}", k, ht(&v.config.get().opt_serialize(&k).unwrap_or_default())));
            }
            out.push(format!(
                "VF {} {} {} {}",
                facets(&v.supported_facets),
                facets(&v.seam_facets),
                facets(&v.mmu_segmentation_facets),
                facets(&v.fuzzy_skin_facets)
            ));
            out.push(format!("VT {} {} {}", ht(&v.get_text_info().m_text), ht(&v.get_text_info().m_font_name), hb(v.emboss_shape.is_some())));
            out.push(format!(
                "VCUT {} {} {} {} {} {}",
                hb(v.cut_info.is_connector),
                v.cut_info.connector_type as i32,
                hf(v.cut_info.radius),
                hf(v.cut_info.height),
                hf(v.cut_info.radius_tolerance),
                hf(v.cut_info.height_tolerance)
            ));
        }
    }
    out.push(format!("PLATES {}", plates.len()));
    for (pi, p) in plates.iter().enumerate() {
        out.push(format!("P {} {} {} {}", pi, p.plate_index, ht(&p.plate_name), hb(p.locked)));
        for (k, v) in &p.obj_inst_map {
            out.push(format!("POI {} {} {}", k, v.0, loaded(v.1 as usize, id_base)));
        }
        out.push(format!(
            "PF {} {} {} {} {} {}",
            ht(&norm_path(&p.gcode_file, &backup)),
            ht(&norm_path(&p.thumbnail_file, &backup)),
            ht(&norm_path(&p.no_light_thumbnail_file, &backup)),
            ht(&norm_path(&p.top_file, &backup)),
            ht(&norm_path(&p.pick_file, &backup)),
            ht(&norm_path(&p.pattern_bbox_file, &backup))
        ));
        out.push(format!(
            "PS {} {} {} {} {} {} {} {} {}",
            ht(&p.gcode_prediction),
            ht(&p.gcode_weight),
            hb(p.toolpath_outside),
            hb(p.is_support_used),
            hb(p.is_label_object_enabled),
            hb(p.support_material_on_wipe_tower),
            ht(&p.printer_model_id),
            ht(&p.nozzle_diameters),
            ht(&p.nozzle_volume_types)
        ));
        for (fi, f) in p.slice_filaments_info.iter().enumerate() {
            let extra = p.slice_filaments_extra.get(fi).cloned().unwrap_or_default();
            let mut line = format!(
                "PFIL {} {} {} {} {} {} {}",
                f.id,
                ht(&f.type_),
                ht(&f.color),
                ht(&f.filament_id),
                hf(f.used_m),
                hf(f.used_g),
                f.group_id.len()
            );
            for g in &f.group_id {
                line.push_str(&format!(" {}", g));
            }
            line.push_str(&format!(
                " {} {} {} {} {} {} {}",
                hd(f.nozzle_diameter),
                ht(&f.nozzle_volume_type),
                hb(f.used_for_object),
                hb(f.used_for_support),
                hb(extra.directly_used),
                hd(extra.total_load_time),
                hd(extra.total_unload_time)
            ));
            out.push(line);
        }
        for w in &p.warnings {
            out.push(format!("PW {} {}", ht(&w.msg), w.level));
        }
        let mut line = format!("PMAP {}", p.filament_maps.len());
        for v in &p.filament_maps {
            line.push_str(&format!(" {}", v));
        }
        line.push_str(&format!(" | {}", p.filament_change_sequence.len()));
        for v in &p.filament_change_sequence {
            line.push_str(&format!(" {}", v));
        }
        line.push_str(&format!(" | {}", p.nozzle_change_sequence.len()));
        for v in &p.nozzle_change_sequence {
            line.push_str(&format!(" {}", v));
        }
        line.push_str(&format!(" | {}", p.optimal_assignment.len()));
        for v in &p.optimal_assignment {
            line.push_str(&format!(" {}", v));
        }
        out.push(line);
        for n in &p.nozzles_info {
            out.push(format!("PNOZ {} {} {} {}", n.group_id, n.extruder_id, ht(&n.diameter), n.volume_type as i32));
        }
        out.push(format!("PCFG {}", cfg(&p.config)));
        for k in ConfigBase::keys(&*p.config) {
            out.push(format!("PCK {} {}", k, ht(&p.config.opt_serialize(&k).unwrap_or_default())));
        }
    }
    for (k, info) in &model.plates_custom_gcodes {
        out.push(format!("CG {} {} {}", k, info.mode as i32, info.gcodes.len()));
        for it in &info.gcodes {
            out.push(format!("CGI {} {} {} {} {}", hd(it.print_z), it.gcode_type as i32, it.extruder, ht(&it.color), ht(&it.extra)));
        }
    }
    out.push(format!(
        "ASM {} {} {}",
        hx(fnv(model.get_assembly_tree_json_str().as_bytes())),
        hx(fnv(model.get_assembly_steps_json_str().as_bytes())),
        hx(fnv(model.get_assembly_model_json_str().as_bytes()))
    ));
}

fn load_and_dump(out: &mut Vec<String>, label: &str, path: &str, normalize_guid: bool, id_base: i64) {
    out.push(format!("FILE {}", label));
    let mut model = Model::new();
    let mut config = DynamicPrintConfig::new();
    let mut subs = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Enable);
    let mut plates = PlateDataPtrs::new();
    let mut presets: Vec<Preset> = Vec::new();
    let mut is_bbl = false;
    let mut file_version = slicer::semver::Semver::new();
    let res = match load_bbs_3mf(
        path,
        &mut config,
        &mut subs,
        &mut model,
        &mut plates,
        &mut presets,
        Some(&mut is_bbl),
        &mut file_version,
        None,
        LoadStrategy::LoadModel | LoadStrategy::LoadConfig,
        None,
        0,
        None,
        None,
        None,
        None,
    ) {
        Ok(r) => r,
        Err(e) => {
            out.push(format!("EXCEPTION {}", ht(&e.0)));
            false
        }
    };
    out.push(format!("R {} {} {}", hb(res), hb(is_bbl), ht(&file_version.to_string())));
    dump_model(out, &mut model, &plates, &presets, &config, normalize_guid, id_base);
}

fn is_text_entry(name: &str) -> bool {
    [".xml", ".model", ".config", ".rels", ".json", ".txt"].iter().any(|e| name.ends_with(e))
}

/// gen.cpp `relative_ids`: the number after `prefix` becomes "@" (number - base).
fn relative_ids(s: &str, prefix: &str, base: usize) -> String {
    let mut out = String::new();
    let mut rest = s;
    while let Some(pos) = rest.find(prefix) {
        let start = pos + prefix.len();
        out.push_str(&rest[..start]);
        let tail = &rest[start..];
        let end = tail.find(|c: char| !c.is_ascii_digit()).unwrap_or(tail.len());
        if end > 0 {
            let v: i64 = tail[..end].parse().unwrap();
            out.push_str(&format!("@{}", v - base as i64));
        }
        rest = &tail[end..];
    }
    out.push_str(rest);
    out
}

/// gen.cpp `replace_between`
fn replace_between(s: &str, open: &str, close: &str, with: &str) -> String {
    let mut out = String::new();
    let mut rest = s;
    while let Some(pos) = rest.find(open) {
        let start = pos + open.len();
        match rest[start..].find(close) {
            Some(end) => {
                out.push_str(&rest[..start]);
                out.push_str(with);
                out.push_str(close);
                rest = &rest[start + end + close.len()..];
            }
            None => break,
        }
    }
    out.push_str(rest);
    out
}

/// gen.cpp `sort_object_blocks`: the "  <object id=..." blocks of model_settings.config sorted by
/// id (the C++ exporter writes them in the order of the objects' addresses).
fn sort_object_blocks(s: &str) -> String {
    let open = "  <object id=\"";
    let close = "  </object>\n";
    let first = match s.find(open) {
        Some(f) => f,
        None => return s.to_string(),
    };
    let mut blocks: Vec<(i64, String)> = Vec::new();
    let mut pos = first;
    while s[pos..].starts_with(open) {
        let end = match s[pos..].find(close) {
            Some(e) => pos + e + close.len(),
            None => return s.to_string(),
        };
        let digits: String = s[pos + open.len()..].chars().take_while(|c| c.is_ascii_digit()).collect();
        blocks.push((digits.parse().unwrap_or(0), s[pos..end].to_string()));
        pos = end;
    }
    blocks.sort_by_key(|b| b.0);
    let mut out = s[..first].to_string();
    for b in &blocks {
        out.push_str(&b.1);
    }
    out.push_str(&s[pos..]);
    out
}

fn normalize_entry(s: &str, base: usize) -> String {
    let s = sort_object_blocks(s);
    let s = replace_between(&s, "<metadata name=\"CreationDate\">", "</metadata>", "DATE");
    let s = replace_between(&s, "<metadata name=\"ModificationDate\">", "</metadata>", "DATE");
    let s = replace_between(&s, " uuid=\"", "\"", "UUID");
    let s = relative_ids(&s, "key=\"identify_id\" value=\"", base);
    relative_ids(&s, "identify_id=\"", base)
}

fn dump_archive(out: &mut Vec<String>, path: &str, base: usize) {
    let file = match std::fs::File::open(path) {
        Ok(f) => f,
        Err(_) => {
            out.push("ARCHIVE none".to_string());
            return;
        }
    };
    let mut archive = zip::ZipArchive::new(file).unwrap();
    let mut entries: Vec<(String, Vec<u8>)> = Vec::new();
    for i in 0..archive.len() {
        let mut f = archive.by_index(i).unwrap();
        let name = String::from_utf8_lossy(f.name_raw()).into_owned();
        let mut data = Vec::new();
        f.read_to_end(&mut data).unwrap();
        entries.push((name, data));
    }
    entries.sort_by(|a, b| a.0.cmp(&b.0));
    out.push(format!("ARCHIVE {}", entries.len()));
    for (name, data) in entries {
        if is_text_entry(&name) {
            let t = normalize_entry(&String::from_utf8_lossy(&data), base);
            out.push(format!("ENTRY {} {}", name, t.len()));
            for line in t.split('\n') {
                out.push(line.to_string());
            }
            out.push("@@END@@".to_string());
        } else {
            out.push(format!("BINARY {} {}", name, data.len()));
        }
    }
}

fn cube(x: f64, y: f64, z: f64) -> ModelMesh {
    ModelMesh::from_its(slicer::triangle_mesh::its_make_cube(x, y, z))
}

/// gen.cpp `build_model`
fn build_model(model: &mut Model, plates: &mut PlateDataPtrs) {
    {
        let o1 = model.add_object_new();
        o1.name = "cube_a".to_string();
        {
            let v1 = o1.add_volume_mesh(cube(10., 10., 10.), ModelVolumeType::ModelPart, true);
            v1.name = "cube_a_part".to_string();
            v1.config.set_key_value("extruder", Box::new(ConfigOptionInt::with_value(2)));
        }
        o1.config.set_key_value("sparse_infill_density", Box::new(ConfigOptionPercent::with_value(25.0)));
        o1.config.set_key_value("extruder", Box::new(ConfigOptionInt::with_value(1)));
        o1.add_instance().set_offset(&Vec3d::new(50., 60., 5.));
        {
            let i2 = o1.add_instance();
            i2.set_offset(&Vec3d::new(120., 60., 5.));
            i2.set_rotation(&Vec3d::new(0., 0., 0.5));
        }
        o1.layer_height_profile.set(&[0., 0.2, 10., 0.2]);
        {
            let mut c = DynamicPrintConfig::new();
            c.set_key_value("layer_height", Box::new(ConfigOptionFloat::with_value(0.1)));
            o1.layer_config_ranges.entry((OrderedFloat(2.0), OrderedFloat(4.0))).or_default().assign_config_data(&c);
        }
        o1.brim_points.push(slicer::brim_ears_point::BrimPoint::from_coords(1.0, 2.0, 0.0, 0.5, 0));
        o1.brim_points.push(slicer::brim_ears_point::BrimPoint::from_coords(-3.25, 4.5, 0.0, 0.75, 0));
        let v1 = &mut o1.volumes[0];
        v1.mmu_segmentation_facets.set_triangle_from_string(0, "8");
        v1.mmu_segmentation_facets.set_triangle_from_string(3, "C");
        v1.supported_facets.set_triangle_from_string(1, "4");
        v1.seam_facets.set_triangle_from_string(2, "8");
    }
    {
        let o2 = model.add_object_new();
        o2.name = "two & parts".to_string();
        o2.module_name = "mod".to_string();
        {
            let v2 = o2.add_volume_mesh(cube(5., 6., 7.), ModelVolumeType::ModelPart, true);
            v2.name = "part \"main\"".to_string();
            v2.source.input_file = "/some/dir/part.stl".to_string();
            v2.source.object_idx = 0;
            v2.source.volume_idx = 1;
            v2.source.mesh_offset = Vec3d::new(1., 2., 3.);
        }
        {
            let v3 = o2.add_volume_mesh(cube(2., 2., 2.), ModelVolumeType::ParameterModifier, true);
            v3.name = "modifier".to_string();
            v3.config.set_key_value("wall_loops", Box::new(ConfigOptionInt::with_value(3)));
            v3.set_offset(&Vec3d::new(1.5, -0.5, 2.));
        }
        o2.config.set_key_value("extruder", Box::new(ConfigOptionInt::with_value(2)));
        let i3 = o2.add_instance();
        i3.set_offset(&Vec3d::new(100., 100., 0.));
        i3.printable = true;
    }

    {
        use slicer::custom_g_code::{Info, Item, Mode, Type};
        let info = model.plates_custom_gcodes.entry(0).or_insert_with(Info::new);
        info.mode = Mode::SingleExtruder;
        info.gcodes.push(Item::new_with_extra(2.4, Type::ColorChange, 1, "#FF0000".to_string(), String::new()));
        info.gcodes.push(Item::new_with_extra(5.0, Type::Custom, 1, String::new(), "G4 S1".to_string()));
    }

    let mut mi = ModelInfo::default();
    mi.model_name = "Test & <model>".to_string();
    mi.description = "a description".to_string();
    mi.license = "BY-SA".to_string();
    model.model_info = Some(Arc::new(mi));
    model.design_info =
        Some(Arc::new(ModelDesignInfo { designer: "alice".to_string(), designer_user_id: "42".to_string(), designer_id: String::new() }));

    let mut p = PlateData::new();
    p.plate_index = 0;
    p.plate_name = "Plate One".to_string();
    p.locked = false;
    p.objects_and_instances = vec![(0, 0), (0, 1), (1, 0)];
    p.is_sliced_valid = true;
    p.gcode_prediction = "1234".to_string();
    p.gcode_weight = "5.67".to_string();
    p.first_layer_time = "88".to_string();
    let mut fi = slicer::project_task::FilamentInfo::default();
    fi.id = 0;
    fi.type_ = "PLA".to_string();
    fi.color = "#FFFFFF".to_string();
    fi.used_m = 1.234;
    fi.used_g = 3.21;
    fi.filament_id = "GFL99".to_string();
    fi.group_id = vec![0];
    fi.nozzle_diameter = 0.4;
    fi.nozzle_volume_type = "Standard".to_string();
    fi.used_for_object = true;
    p.slice_filaments_info.push(fi);
    let mut sw = slicer::gcode::g_code_processor::SliceWarning::default();
    sw.msg = "test_warning".to_string();
    sw.level = 1;
    sw.error_code = "1000C001".to_string();
    p.warnings.push(sw);
    p.filament_change_sequence = vec![0, 1, 0];
    p.nozzle_change_sequence = vec![0];
    p.config.set_key_value("curr_bed_type", Box::new(ConfigOptionEnum::<BedType>::with_value(BedType::BtPEI)));
    p.config.set_key_value("print_sequence", Box::new(ConfigOptionEnum::<PrintSequence>::with_value(PrintSequence::ByObject)));
    p.config.set_key_value("first_layer_print_sequence", Box::new(ConfigOptionInts::from_values(vec![1, 2])));
    p.config.set_key_value("spiral_mode", Box::new(ConfigOptionBool::with_value(false)));
    p.config.set_key_value("filament_map_mode", Box::new(ConfigOptionEnum::<FilamentMapMode>::with_value(FilamentMapMode::FmmManual)));
    p.config.set_key_value("filament_map", Box::new(ConfigOptionInts::from_values(vec![1, 2])));
    plates.push(p);
}

fn store_scenario(out: &mut Vec<String>, label: &str, dir: &str, strategy: SaveStrategy) {
    out.push(format!("STORE {}", label));
    let mut model = Model::new();
    let mut plates = PlateDataPtrs::new();
    build_model(&mut model, &mut plates);
    let keys = ConfigBase::keys(&*DynamicPrintConfig::full_print_config());
    let config = *DynamicPrintConfig::new_from_defaults_keys(&keys).unwrap();
    let path = format!("{}/{}.3mf", dir, label);
    let mut sp = StoreParams::new();
    sp.path = path.clone();
    sp.model = Some(model);
    sp.plate_data_list = plates;
    sp.config = Some(config);
    sp.strategy = strategy;
    let res = store_bbs_3mf(&mut sp).unwrap_or(false);
    out.push(format!("SR {}", hb(res)));
    let base = sp.model.as_ref().unwrap().id().id;
    dump_archive(out, &path, base);
    load_and_dump(out, &format!("{} reload", label), &path, true, base as i64);
}

/// The vectors' lines (text entries keep their own lines).
fn read_vectors(path: &std::path::Path) -> Vec<String> {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("{}: {}", path.display(), e));
    let text = if path.extension().map(|e| e == "gz").unwrap_or(false) {
        let mut d = flate2::read::GzDecoder::new(&bytes[..]);
        let mut s = String::new();
        d.read_to_string(&mut s).unwrap();
        s
    } else {
        String::from_utf8(bytes).unwrap()
    };
    let mut lines: Vec<String> = text.split('\n').map(|s| s.to_string()).collect();
    if lines.last().map(|l| l.is_empty()).unwrap_or(false) {
        lines.pop();
    }
    lines
}

/// Compares two dumps; returns the number of differing lines (reports the first ones) and the
/// count of compared lines per line kind.
fn compare(expected: &[String], actual: &[String]) -> (usize, BTreeMap<String, usize>) {
    let mut kinds: BTreeMap<String, usize> = BTreeMap::new();
    let mut diffs = 0usize;
    let n = expected.len().max(actual.len());
    let mut in_entry = false;
    for i in 0..n {
        let e = expected.get(i).map(|s| s.as_str()).unwrap_or("<missing>");
        let a = actual.get(i).map(|s| s.as_str()).unwrap_or("<missing>");
        let kind = if in_entry { "entry-text".to_string() } else { e.split(' ').next().unwrap_or("").to_string() };
        if e.starts_with("ENTRY ") {
            in_entry = true;
        } else if e == "@@END@@" {
            in_entry = false;
        }
        *kinds.entry(kind).or_default() += 1;
        if e != a {
            if diffs < 25 {
                eprintln!("line {}:\n  C++:  {}\n  Rust: {}", i + 1, &e[..e.len().min(400)], &a[..a.len().min(400)]);
            }
            diffs += 1;
        }
    }
    (diffs, kinds)
}

#[test]
fn bbs_3mf_load_and_store_match_the_pin() {
    let dir = scratch_dir();

    let root = repo_root();
    let mut out: Vec<String> = Vec::new();
    let files = [
        "crates/libslic3r-rs/tests/data/painted_cube.3mf",
        "tests/fixtures/calib_base.3mf",
        "tests/fixtures/calib_base_orca.3mf",
        "tests/fixtures/calib_base_snapmaker.3mf",
        "tests/fixtures/legacy_token.3mf",
        "tests/fixtures/legacy_token_orca.3mf",
        "tests/fixtures/legacy_token_snapmaker.3mf",
    ];
    for f in files {
        load_and_dump(&mut out, f, &root.join(f).to_string_lossy(), false, -1);
    }
    store_scenario(&mut out, "store_zip64", &dir, SaveStrategy::Zip64);
    store_scenario(&mut out, "store_split", &dir, SaveStrategy::SplitModel | SaveStrategy::Zip64);

    let data = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/bbs_3mf");
    let vectors = if data.join("vectors.txt.gz").exists() { data.join("vectors.txt.gz") } else { data.join("vectors.txt") };
    let expected = read_vectors(&vectors);
    let (diffs, kinds) = compare(&expected, &out);
    eprintln!("compared lines per kind: {:?}", kinds);
    assert_eq!(diffs, 0, "{} of {} lines differ from the pin", diffs, expected.len());
}

#[test]
fn bbs_3mf_extra_file_matches_the_pin() {
    let (path, vectors) = match (std::env::var("BBS_3MF_EXTRA"), std::env::var("BBS_3MF_EXTRA_VECTORS")) {
        (Ok(p), Ok(v)) => (p, v),
        _ => return,
    };
    scratch_dir();
    let mut out: Vec<String> = Vec::new();
    load_and_dump(&mut out, &basename_of(&path), &path, false, -1);
    let expected = read_vectors(std::path::Path::new(&vectors));
    let (diffs, kinds) = compare(&expected, &out);
    eprintln!("compared lines per kind: {:?}", kinds);
    assert_eq!(diffs, 0, "{} of {} lines differ from the pin", diffs, expected.len());
}
