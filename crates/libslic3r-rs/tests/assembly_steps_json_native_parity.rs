//! `format::assembly_steps_json` against Format/AssemblyStepsJson.cpp at the pin.
//!
//! `tests/data/assembly_steps_json/vectors_steps.txt` and `vectors_model.txt` are the output of
//! `gen.cpp` beside them, which compiles AssemblyStepsJson.cpp itself with the engine's
//! nlohmann::json (the header comment there says how, and describes the lines).
//!
//! Against the built C++, exactly (a text by its bytes, or by its FNV-1a hash and length where the
//! file keeps only those; doubles by their bits): `AssemblyStepJson::load_from_string`, `load`,
//! `to_json_string` and the getters, and through them every `to_json` / `from_json` of the unit;
//! the `assembly_steps_json_*` functions; `qualified_assembly_result_json`;
//! `AssemblyStepsTreeData::from_json_string` / `to_json_string`; `assembly_model_to_json_string`;
//! `assembly_model_from_json_string`. The last four and `..._last_step_covers_model` take a
//! `Model`: the C++ ran on a stand-in with the members the unit uses, as the Rust does.
//!
//! Read off the C++ only (the tests at the end): the inline members of the header, `save`,
//! `get_debug_file_path`, and the GUID `ensure_part_guid` makes for a part that has none.

use slicer::format::assembly_steps_json::*;
use slicer::geometry::geometry::{Transform3d, Transformation, Vec3d};
use slicer::model::ModelVolumeType;
use slicer::object_id::ObjectID;

const STEPS: &str = include_str!("data/assembly_steps_json/vectors_steps.txt");
const MODEL: &str = include_str!("data/assembly_steps_json/vectors_model.txt");

// ---- the text forms of gen.cpp ----

fn esc(s: &str) -> String {
    if s.is_empty() {
        return "\\e".to_string();
    }
    let mut out = String::new();
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            ' ' => out.push_str("\\s"),
            c if (c as u32) < 0x20 || c as u32 == 0x7f => out.push_str(&format!("\\x{:02X}", c as u32)),
            c => out.push(c),
        }
    }
    out
}

fn unesc(s: &str) -> String {
    if s == "\\e" {
        return String::new();
    }
    let mut out = String::new();
    let mut chars = s.chars();
    while let Some(c) = chars.next() {
        if c != '\\' {
            out.push(c);
            continue;
        }
        match chars.next().expect("escape") {
            '\\' => out.push('\\'),
            'n' => out.push('\n'),
            'r' => out.push('\r'),
            's' => out.push(' '),
            'x' => {
                let hex: String = [chars.next().expect("hex"), chars.next().expect("hex")].iter().collect();
                out.push(u8::from_str_radix(&hex, 16).expect("hex") as char);
            }
            other => panic!("escape \\{other}"),
        }
    }
    out
}

fn fnv(s: &str) -> String {
    let mut h: u64 = 0xcbf29ce484222325;
    for b in s.bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    format!("#{:016x} {}", h, s.len())
}

/// A text the function under test returned, as gen.cpp writes it.
fn out(s: &str) -> String {
    if s.len() > 160 {
        fnv(s)
    } else {
        esc(s)
    }
}

fn dbits(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}

fn fbits(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}

fn mat(t: &Transform3d) -> String {
    let mut s = String::new();
    for r in 0..4 {
        for c in 0..4 {
            s.push(' ');
            s.push_str(&dbits(t[(r, c)]));
        }
    }
    s
}

fn ext(extruder: Option<i32>) -> String {
    match extruder {
        None => "-".to_string(),
        Some(v) => v.to_string(),
    }
}

fn dump_model(model: &Model) -> String {
    let mut s = format!("M {}", model.objects.len());
    for mo in &model.objects {
        let mo = match mo {
            None => {
                s.push_str(" N");
                continue;
            }
            Some(mo) => mo,
        };
        s.push_str(&format!(" O {} {} {} {}", mo.id().id, esc(&mo.name), ext(mo.config_extruder), mo.volumes.len()));
        for mv in &mo.volumes {
            let mv = match mv {
                None => {
                    s.push_str(" n");
                    continue;
                }
                Some(mv) => mv,
            };
            s.push_str(&format!(
                " v {} {} {} {} {} {}{}{} {}",
                mv.id().id,
                esc(&mv.name),
                mv.m_type as i32,
                ext(mv.config_extruder),
                esc(&mv.m_part_guid.borrow()),
                esc(&mv.m_assembly_src_guid),
                mat(&mv.m_transformation.matrix()),
                mat(&mv.m_assemble_transformation.matrix()),
                mv.m_assemble_initialized as i32
            ));
        }
        s.push_str(&format!(" {}", mo.instances.len()));
        for mi in &mo.instances {
            let mi = match mi {
                None => {
                    s.push_str(" n");
                    continue;
                }
                Some(mi) => mi,
            };
            s.push_str(&format!(
                " i{}{} {} {} {} {} {}",
                mat(&mi.m_transformation.matrix()),
                mat(&mi.m_assemble_transformation.borrow().matrix()),
                dbits(mi.m_offset_to_assembly.x),
                dbits(mi.m_offset_to_assembly.y),
                dbits(mi.m_offset_to_assembly.z),
                mi.m_assemble_initialized as i32,
                *mi.m_assemble_scalling_factor_dirty.borrow() as i32
            ));
        }
    }
    s
}

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn u(&mut self) -> usize {
        self.s().parse().expect("unsigned")
    }
    fn b(&mut self) -> bool {
        self.u() != 0
    }
    fn text(&mut self) -> String {
        unesc(self.s())
    }
    fn ext(&mut self) -> Option<i32> {
        match self.s() {
            "-" => None,
            v => Some(v.parse().expect("extruder")),
        }
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double bits"))
    }
    fn mat(&mut self) -> Transform3d {
        let mut t = Transform3d::identity();
        for r in 0..4 {
            for c in 0..4 {
                t[(r, c)] = self.d();
            }
        }
        t
    }
}

fn parse_model(line: &str) -> Model {
    let mut t = Tok(line.split_whitespace());
    assert_eq!(t.s(), "M");
    let mut model = Model::default();
    for _ in 0..t.u() {
        match t.s() {
            "N" => {
                model.objects.push(None);
                continue;
            }
            "O" => {}
            other => panic!("object tag {other}"),
        }
        let mut mo = ModelObject { m_id: ObjectID::new(t.u()), name: t.text(), config_extruder: t.ext(), ..ModelObject::default() };
        for _ in 0..t.u() {
            match t.s() {
                "n" => {
                    mo.volumes.push(None);
                    continue;
                }
                "v" => {}
                other => panic!("volume tag {other}"),
            }
            let mut mv = ModelVolume::default();
            mv.m_id = ObjectID::new(t.u());
            mv.name = t.text();
            mv.m_type = match t.s() {
                "0" => ModelVolumeType::ModelPart,
                "1" => ModelVolumeType::NegativeVolume,
                "2" => ModelVolumeType::ParameterModifier,
                other => panic!("volume type {other}"),
            };
            mv.config_extruder = t.ext();
            *mv.m_part_guid.borrow_mut() = t.text();
            mv.m_assembly_src_guid = t.text();
            mv.m_transformation = Transformation::from_transform(t.mat());
            mv.m_assemble_transformation = Transformation::from_transform(t.mat());
            mv.m_assemble_initialized = t.b();
            mo.volumes.push(Some(mv));
        }
        for _ in 0..t.u() {
            match t.s() {
                "n" => {
                    mo.instances.push(None);
                    continue;
                }
                "i" => {}
                other => panic!("instance tag {other}"),
            }
            let mut mi = ModelInstance::default();
            mi.m_transformation = Transformation::from_transform(t.mat());
            *mi.m_assemble_transformation.borrow_mut() = Transformation::from_transform(t.mat());
            mi.m_offset_to_assembly = Vec3d::new(t.d(), t.d(), t.d());
            mi.m_assemble_initialized = t.b();
            *mi.m_assemble_scalling_factor_dirty.borrow_mut() = t.b();
            mo.instances.push(Some(mi));
        }
        model.objects.push(Some(mo));
    }
    assert!(t.0.next().is_none(), "model line read to its end");
    model
}

fn dump_tree(tree: &AssemblyStepsTreeData) -> String {
    let mut s = format!("T {}", tree.nodes.len());
    for n in &tree.nodes {
        s.push_str(&format!(
            " {} {} {} {} {} {} {} {} {}",
            n.r#type as i32,
            n.id,
            n.step,
            esc(&n.name),
            n.object_idx,
            n.object_id,
            n.visible as i32,
            n.is_final_assembly,
            n.inherited_from_step_id
        ));
        s.push_str(&format!(" {}", n.inherited_object_ids.len()));
        for id in &n.inherited_object_ids {
            s.push_str(&format!(" {id}"));
        }
        s.push_str(&format!(" {}", n.children.len()));
        for c in &n.children {
            s.push_str(&format!(" {c}"));
        }
        match &n.assembly_tree_checked {
            None => s.push_str(" -"),
            Some(checked) => {
                let sorted: std::collections::BTreeMap<&String, &bool> = checked.iter().collect();
                s.push_str(&format!(" {}", sorted.len()));
                for (key, value) in sorted {
                    s.push_str(&format!(" {} {}", esc(key), *value as i32));
                }
            }
        }
        s.push_str(&format!(" {} {} {} {}", n.kf_data.node_idx, n.kf_data.object_idx, n.kf_data.is_folder as i32, n.kf_data.entries.len()));
        for e in &n.kf_data.entries {
            s.push_str(&format!(
                " {} {} {} {} {}",
                e.need_save as i32, e.data.is_sub_assembly as i32, e.data.is_interpolation as i32, e.data.play_node_idx, e.data.play_frame_idx
            ));
            s.push_str(&format!(" {}", e.data.volume_names.len()));
            for (key, name) in &e.data.volume_names {
                s.push_str(&format!(" {} {} {}", key.0, key.1, esc(name)));
            }
            s.push_str(&format!(" {}", e.data.volume_guids.len()));
            for (key, guid) in &e.data.volume_guids {
                s.push_str(&format!(" {} {} {}", key.0, key.1, esc(guid)));
            }
            let mut kj = serde_json::Value::Null;
            e.data.to_json(&mut kj);
            s.push_str(&format!(" {}", out(&dump_compact(&kj))));
        }
    }
    s.push_str(&format!(" R {}", tree.roots.len()));
    for r in &tree.roots {
        s.push_str(&format!(" {r}"));
    }
    s.push_str(&format!(
        " {} {} {} {}",
        tree.camera_ref_viewport_w,
        tree.camera_ref_viewport_h,
        tree.has_loaded_recorded_baseline as i32,
        tree.loaded_recorded_volumes.len()
    ));
    for g in &tree.loaded_recorded_volumes {
        s.push_str(&format!(" {}", esc(g)));
    }
    s
}

/// `json.dump()` of what `KeyFrame::to_json` made.
fn dump_compact(json: &serde_json::Value) -> String {
    dump(json, -1)
}

// ---- the cases ----

struct Lines<'a> {
    lines: std::str::Lines<'a>,
    n: usize,
}

impl<'a> Lines<'a> {
    fn new(text: &'a str) -> Self {
        Lines { lines: text.lines(), n: 0 }
    }
    fn next(&mut self) -> Option<&'a str> {
        self.n += 1;
        self.lines.next()
    }
    fn line(&mut self) -> &'a str {
        self.next().expect("line")
    }
}

fn rt_out(ok: bool, doc: &AssemblyStepJson) -> [String; 3] {
    [
        format!(
            "{} {} {} {} {}",
            ok as i32,
            fbits(doc.get_assembly_part_number_label_font_size()),
            doc.get_camera_ref_viewport_w(),
            doc.get_camera_ref_viewport_h(),
            doc.get_items().len()
        ),
        esc(&doc.get_pdf_export_params().title),
        out(&doc.to_json_string()),
    ]
}

/// Runs every case of one vectors file; returns how many cases of each kind it ran.
fn run(vectors: &str, tmp_name: &str) -> std::collections::BTreeMap<String, usize> {
    let mut counts: std::collections::BTreeMap<String, usize> = std::collections::BTreeMap::new();
    let mut lines = Lines::new(vectors);
    let mut doc = String::new();
    let mut model = Model::default();
    let tmp = std::env::temp_dir().join(format!("{}_{}.json", tmp_name, std::process::id()));
    while let Some(head) = lines.next() {
        let at = lines.n;
        let mut head_tokens = head.split_whitespace();
        let op = head_tokens.next().expect("op");
        *counts.entry(op.to_string()).or_insert(0) += 1;
        match op {
            "DOC" => doc = unesc(lines.line()),
            "RT" => {
                let mut json_doc = AssemblyStepJson::default();
                let ok = json_doc.load_from_string(&doc);
                for got in rt_out(ok, &json_doc) {
                    assert_eq!(got, lines.line(), "RT, line {at}, document {doc:?}");
                }
            }
            "LOAD" => {
                std::fs::write(&tmp, doc.as_bytes()).expect("write the scratch file");
                let mut json_doc = AssemblyStepJson::default();
                let ok = json_doc.load(tmp.to_str().expect("utf-8 temp path"));
                for got in rt_out(ok, &json_doc) {
                    assert_eq!(got, lines.line(), "LOAD, line {at}, document {doc:?}");
                }
            }
            "FIN" => {
                let got = assembly_steps_json_last_step_is_final_assembly(&doc) as i32;
                assert_eq!(got.to_string(), lines.line(), "FIN, line {at}, document {doc:?}");
            }
            "MARK" => {
                let got = assembly_steps_json_mark_last_step_final_assembly(&doc);
                assert_eq!(out(&got), lines.line(), "MARK, line {at}, document {doc:?}\n{got}");
            }
            "QUAL" => {
                for qualified in [true, false] {
                    let got = assembly_steps_json_with_qualified_assembly(&doc, qualified);
                    assert_eq!(out(&got), lines.line(), "QUAL {qualified}, line {at}, document {doc:?}\n{got}");
                }
            }
            "META" => {
                let meta = assembly_steps_json_file_meta(&doc);
                let got = format!(
                    "{} {} {} {}",
                    dbits(meta.version),
                    meta.has_qualified as i32,
                    meta.qualified as i32,
                    assembly_steps_version_needs_recompute(meta.version) as i32
                );
                assert_eq!(got, lines.line(), "META, line {at}, document {doc:?}");
            }
            "VER" => {
                let version = f64::from_bits(u64::from_str_radix(head_tokens.next().expect("version"), 16).expect("bits"));
                let got = assembly_steps_json_with_version(&doc, version);
                assert_eq!(out(&got), lines.line(), "VER {version}, line {at}, document {doc:?}\n{got}");
            }
            "QRES" => {
                for qualified in [false, true] {
                    for need_rewrite_3mf in [false, true] {
                        assert_eq!(esc(&qualified_assembly_result_json(qualified, need_rewrite_3mf)), lines.line(), "QRES, line {at}");
                    }
                }
            }
            "MODEL" => {
                let line = lines.line();
                model = parse_model(line);
                assert_eq!(dump_model(&model), line, "the model is read back as written, line {at}");
            }
            "COV" => {
                let amodel = unesc(lines.line());
                let got = assembly_steps_json_last_step_covers_model(&doc, &model, &amodel) as i32;
                assert_eq!(got.to_string(), lines.line(), "COV, line {at}, document {doc:?}, assembly model {amodel:?}");
            }
            "TREE" => {
                let mut tree = AssemblyStepsTreeData::default();
                let mut error = "unset".to_string();
                let mut font = -1.0f32;
                let ok = AssemblyStepsTreeData::from_json_string(&doc, &mut tree, &model, Some(&mut error), Some(&mut font));
                assert_eq!(format!("{} {}", ok as i32, fbits(font)), lines.line(), "TREE, line {at}, document {doc:?}");
                assert_eq!(esc(&error), lines.line(), "TREE error, line {at}");
                assert_eq!(dump_tree(&tree), lines.line(), "TREE tree, line {at}, document {doc:?}");
                assert_eq!(out(&tree.to_json_string()), lines.line(), "TREE to_json_string, line {at}, document {doc:?}");
            }
            "AM2J" => {
                let got = assembly_model_to_json_string(&model);
                assert_eq!(out(&got), lines.line(), "AM2J, line {at}\n{got}");
            }
            "J2AM" => {
                let mut out_model = Model::default();
                out_model.add_object().name = "stale".to_string();
                let mut error = "unset".to_string();
                let result = match assembly_model_from_json_string(&doc, &mut out_model, &model, Some(&mut error)) {
                    Ok(true) => "ok",
                    Ok(false) => "false",
                    Err(e) => {
                        error = e.what().to_string();
                        "throw"
                    }
                };
                assert_eq!(result, lines.line(), "J2AM, line {at}, document {doc:?}");
                let want_error = unesc(lines.line());
                if want_error.starts_with("[json.exception.parse_error.") {
                    // The text of a parse error is not nlohmann's (the FIDELITY-NOTE of the unit).
                    assert_eq!(error, PARSE_ERROR_WHAT, "J2AM error, line {at}");
                } else {
                    assert_eq!(error, want_error, "J2AM error, line {at}, document {doc:?}");
                }
                // The objects the function added have new ids, which depend on what else made
                // objects before: they are checked for being valid and ascending, then set to the
                // ids of the reference run.
                let want_line = lines.line();
                let want = parse_model(want_line);
                assert_eq!(out_model.objects.len(), want.objects.len(), "J2AM objects, line {at}");
                let mut last_id = 0;
                for (got_object, want_object) in out_model.objects.iter_mut().zip(&want.objects) {
                    let got_object = got_object.as_mut().expect("add_object never adds a null");
                    assert!(got_object.m_id.id > last_id, "J2AM: a new id for every object, line {at}");
                    last_id = got_object.m_id.id;
                    got_object.m_id = want_object.as_ref().expect("an object").m_id;
                }
                assert_eq!(dump_model(&out_model), want_line, "J2AM model, line {at}, document {doc:?}");
                let got = assembly_model_to_json_string(&out_model);
                assert_eq!(out(&got), lines.line(), "J2AM to json, line {at}, document {doc:?}\n{got}");
            }
            other => panic!("line {at}: unknown case {other}"),
        }
    }
    let _ = std::fs::remove_file(&tmp);
    counts
}

#[test]
fn steps_document_functions_match_the_cpp() {
    let counts = run(STEPS, "asj_steps");
    assert_eq!(counts["DOC"], 104);
    for op in ["RT", "FIN", "MARK", "QUAL", "META", "VER"] {
        assert_eq!(counts[op], 104, "{op}");
    }
    assert_eq!(counts["LOAD"], 48);
    assert_eq!(counts["QRES"], 1);
}

#[test]
fn model_functions_match_the_cpp() {
    let counts = run(MODEL, "asj_model");
    assert_eq!(counts["MODEL"], 6);
    assert_eq!(counts["COV"], 6 * 9);
    assert_eq!(counts["TREE"], 6 * 6);
    assert_eq!(counts["AM2J"], 12);
    assert_eq!(counts["J2AM"], 3 * 16);
}

// ---- read off the C++ ----

/// AssemblyStepsJson.hpp:137-163, 220-227, 280-289, 312-321, 361-364.
#[test]
fn header_inline_members() {
    let mut kf = KeyFrame::default();
    assert!(kf.is_last() && !kf.is_start() && !kf.is_transition());
    kf.id = 1;
    assert!(!kf.is_last() && kf.is_start() && !kf.is_transition());
    kf.id = 2;
    assert!(!kf.is_last() && !kf.is_start() && kf.is_transition());

    // clone_from copies everything but the three members marked "no need to save".
    let mut src = KeyFrame::default();
    src.id = 7;
    src.name = "n".to_string();
    src.is_sub_assembly = true;
    src.is_camera_define = true;
    src.camera_user_defined = true;
    src.view_matrix[(0, 3)] = 2.0;
    src.projection_matrix[(1, 3)] = 3.0;
    src.camera_target = Vec3d::new(1.0, 2.0, 3.0);
    src.camera_zoom = 4.0;
    src.camera_margin_factor = 5.0;
    src.object_transformations.insert(1, Transformation::default());
    src.volume_transformations.insert((1, 2), Transformation::default());
    src.volume_names.insert((1, 2), "v".to_string());
    src.volume_guids.insert((1, 2), "g".to_string());
    src.assembly_note.show_part_labels = false;
    src.labels_show_type = LabelsShowType::ONLY_MODEL_VOLUME;
    src.is_interpolation = true;
    src.play_node_idx = 5;
    src.play_frame_idx = 6;
    let mut dst = KeyFrame::default();
    dst.clone_from(&src);
    let mut want = src.clone();
    want.is_interpolation = false;
    want.play_node_idx = -1;
    want.play_frame_idx = -1;
    assert_eq!(dst, want);

    let mut entry = KeyFrameEntry { need_save: false, data: KeyFrame::default() };
    entry.clone_from(&KeyFrameEntry { need_save: false, data: src.clone() });
    assert!(entry.need_save);
    assert_eq!(entry.data, want);
    assert!(entry.is_transition() && !entry.is_last() && !entry.is_start());

    let mut tree = AssemblyStepsTreeData::default();
    assert!(tree.empty());
    tree.nodes.push(AssemblyStepsTreeNode::default());
    tree.roots.push(0);
    tree.camera_ref_viewport_w = 3;
    tree.camera_ref_viewport_h = 4;
    tree.has_loaded_recorded_baseline = true;
    tree.loaded_recorded_volumes.insert("g".to_string());
    assert!(!tree.empty());
    tree.clear();
    assert_eq!(tree, AssemblyStepsTreeData::default());

    let mut doc = AssemblyStepJson::default();
    doc.set_pdf_export_params(&PdfExportParams { title: "t".to_string() });
    doc.set_assembly_part_number_label_font_size(12.5);
    doc.set_camera_ref_viewport(640, 480);
    assert_eq!(doc.get_pdf_export_params().title, "t");
    assert_eq!(doc.get_assembly_part_number_label_font_size(), 12.5);
    assert_eq!((doc.get_camera_ref_viewport_w(), doc.get_camera_ref_viewport_h()), (640, 480));

    assert_eq!(AssembleSingleInfo::default().get_type(), "single");
    assert_eq!(AssembleSub::default().get_type(), "sub");
    assert_eq!((AssemblyStepKind::Normal, AssemblyStepKind::FinalAssembly, AssemblyStepKind::OverallPreview), (0, 1, 2));
    assert_eq!(K_ASSEMBLY_STEPS_QUALIFIED_KEY, "qualified_assembly");
    assert_eq!(K_QUALIFIED_ASSEMBLY_RESULT_KEY, "qualified_assembly");
    assert_eq!(K_NEED_REWRITE_3MF_RESULT_KEY, "need_rewrite_3mf");

    // version < 1.2 - 1e-3
    assert!(assembly_steps_version_needs_recompute(1.1));
    assert!(assembly_steps_version_needs_recompute(1.1989));
    assert!(!assembly_steps_version_needs_recompute(1.1995));
    assert!(!assembly_steps_version_needs_recompute(1.2));
}

/// AssemblyStepsJson.cpp:808-827: the directory is made, the file holds `to_json_string()`; a path
/// without a directory part is the `filesystem_error` of `create_directories("")`.
#[test]
fn save_writes_what_to_json_string_returns() {
    let dir = std::env::temp_dir().join(format!("asj_save_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let path = dir.join("a").join("b").join("steps.json");
    let mut doc = AssemblyStepJson::default();
    assert!(doc.load_from_string(r#"{"camera_ref_viewport_w":3,"items":[{"type":"sub","id":1,"name":"s"}]}"#));
    assert_eq!(doc.save(path.to_str().unwrap()).unwrap(), true);
    assert_eq!(std::fs::read_to_string(&path).unwrap(), doc.to_json_string());

    // The file cannot be opened: it is a directory.
    assert_eq!(doc.save(dir.join("a").join("b").to_str().unwrap()).unwrap(), false);

    let mut loaded = AssemblyStepJson::default();
    assert!(loaded.load(path.to_str().unwrap()));
    assert_eq!(loaded.to_json_string(), doc.to_json_string());
    assert!(!loaded.load(dir.join("missing.json").to_str().unwrap()));
    let _ = std::fs::remove_dir_all(&dir);

    assert!(doc.save("asj_no_directory_part.json").is_err());
}

/// AssemblyStepsJson.cpp:698-700
#[test]
fn debug_file_path_is_under_the_data_dir() {
    let want: std::path::PathBuf = [slicer::log_sink::data_dir().as_str(), "cache", "assembly_step.json"].iter().collect();
    assert_eq!(AssemblyStepJson::get_debug_file_path(), want.to_string_lossy());
}

/// AssemblyStepsJson.cpp:1384-1394 with Model.cpp:2215-2221: a model part without a GUID gets one
/// (a random version 4 UUID) when the tree is loaded, and keeps it.
#[test]
fn loading_a_tree_gives_a_part_without_guid_one() {
    let mut model = Model::default();
    let mut part = ModelVolume::default();
    part.name = "p".to_string();
    let mut modifier = ModelVolume::default();
    modifier.m_type = ModelVolumeType::ParameterModifier;
    let object = ModelObject { m_id: ObjectID::new(4), name: "o".to_string(), volumes: vec![Some(part), None, Some(modifier)], ..ModelObject::default() };
    model.objects.push(Some(object));

    let mut tree = AssemblyStepsTreeData::default();
    assert!(AssemblyStepsTreeData::from_json_string(r#"{"items":[{"type":"single","name":"o"}]}"#, &mut tree, &model, None, None));
    assert_eq!(tree.loaded_recorded_volumes.len(), 1);
    let guid = tree.loaded_recorded_volumes.iter().next().unwrap().clone();
    let volumes = &model.objects[0].as_ref().unwrap().volumes;
    assert_eq!(volumes[0].as_ref().unwrap().part_guid(), guid);
    assert_eq!(volumes[2].as_ref().unwrap().part_guid(), "");
    let bytes = guid.as_bytes();
    assert_eq!(bytes.len(), 36);
    for (i, b) in bytes.iter().enumerate() {
        if [8, 13, 18, 23].contains(&i) {
            assert_eq!(*b, b'-');
        } else {
            assert!(b.is_ascii_digit() || (b'a'..=b'f').contains(b), "{guid}");
        }
    }
    assert_eq!(bytes[14], b'4');
    assert!(matches!(bytes[19], b'8' | b'9' | b'a' | b'b'));
    assert_eq!(tree.nodes.len(), 1);
    assert_eq!((tree.nodes[0].object_idx, tree.nodes[0].object_id), (0, 4));
}
