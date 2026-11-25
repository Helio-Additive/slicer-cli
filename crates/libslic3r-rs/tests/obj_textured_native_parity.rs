//! `obj_to_textured_mesh` (OBJ.cpp:287-420), `load_obj` with its MtlData (OBJ.cpp:25-264) and the textured-OBJ
//! branch of `Model::read_from_file` (Model.cpp:344-370) against the pin. Vectors from
//! `tests/data/obj_textured/gen.cpp`, run from this crate's directory: `textured.obj` (UVs, a quad, four `usemtl`
//! ranges, one texture shared by two materials, a missing texture, a colour-only material, `mtllib ./...`) and
//! `plain.obj` (UVs, colour-only material: not textured), `ngon.obj` (a pentagon and a hexagon triangulated as
//! fans, a corner without a texture coordinate) and `bad_uv.obj` (a texture coordinate index past the table:
//! refused). Every line must be equal, floats bit for bit.
use slicer::format::bbs_3mf::LoadStrategy;
use slicer::format::obj::{load_obj_to_model_with_mtl, obj_to_textured_mesh, ObjInfo};
use slicer::format::objparser::MtlData;
use slicer::model::Model;
use slicer::texture_painting::TexturedMesh;

/// C's `%a` for a double (normal numbers and zero, as the generator prints them).
fn fmt_a(x: f64) -> String {
    if x == 0.0 {
        return if x.is_sign_negative() { "-0x0p+0".into() } else { "0x0p+0".into() };
    }
    let bits = x.to_bits();
    let sign = if bits >> 63 != 0 { "-" } else { "" };
    let exp = ((bits >> 52) & 0x7ff) as i64 - 1023;
    let mut mant = format!("{:013x}", bits & ((1u64 << 52) - 1));
    while mant.ends_with('0') {
        mant.pop();
    }
    let e = if exp >= 0 { format!("+{exp}") } else { format!("{exp}") };
    if mant.is_empty() {
        format!("{sign}0x1p{e}")
    } else {
        format!("{sign}0x1.{mant}p{e}")
    }
}

fn a32(x: f32) -> String {
    fmt_a(x as f64)
}

fn print_tex(out: &mut Vec<String>, tag: &str, ok: bool, t: &TexturedMesh) {
    out.push(format!(
        "{} {} {} {} {} {} {} {} {} {} {}",
        tag,
        ok as i32,
        t.vertices.len(),
        t.indices.len(),
        t.uv_coords.len(),
        t.uv_indices.len(),
        t.material_colors.len(),
        t.material_texture_map.len(),
        t.material_ids.len(),
        t.textures.len(),
        t.precomputed_face_colors.len()
    ));
    for v in &t.vertices {
        out.push(format!("v {} {} {}", a32(v[0]), a32(v[1]), a32(v[2])));
    }
    for f in &t.indices {
        out.push(format!("f {} {} {}", f[0], f[1], f[2]));
    }
    for uv in &t.uv_coords {
        out.push(format!("uv {} {}", a32(uv[0]), a32(uv[1])));
    }
    for f in &t.uv_indices {
        out.push(format!("uf {} {} {}", f[0], f[1], f[2]));
    }
    for c in &t.material_colors {
        out.push(format!("mc {} {} {} {}", a32(c[0]), a32(c[1]), a32(c[2]), a32(c[3])));
    }
    let mut mt = String::from("mt");
    for m in &t.material_texture_map {
        mt.push_str(&format!(" {m}"));
    }
    out.push(mt);
    let mut mi = String::from("mi");
    for m in &t.material_ids {
        mi.push_str(&format!(" {m}"));
    }
    out.push(mi);
    for ti in &t.textures {
        let sum: u64 = ti.data.iter().map(|&b| b as u64).sum();
        out.push(format!("tex {} {} {} {} {}", ti.width, ti.height, ti.channels, ti.data.len(), sum));
    }
}

fn rust_case(path: &str) -> Vec<String> {
    let mut out = vec![format!("case {path}")];
    let mut obj_info = ObjInfo::new();
    let mut mtl_data = MtlData::new();
    let mut message = String::new();
    let loaded = load_obj_to_model_with_mtl(std::path::Path::new(path), &mut obj_info, &mut message, None, false, Some(&mut mtl_data));
    out.push(format!(
        "load {} {} {} {} {}",
        loaded.is_ok() as i32,
        obj_info.has_uv_png as i32,
        obj_info.uvs.len(),
        obj_info.usemtls.len(),
        mtl_data.mtl_orders.len()
    ));
    let model = match loaded {
        Ok(m) => m,
        Err(_) => return out,
    };
    for u in &obj_info.uvs {
        out.push(format!("ouv {} {} {} {} {} {}", a32(u[0][0]), a32(u[0][1]), a32(u[1][0]), a32(u[1][1]), a32(u[2][0]), a32(u[2][1])));
    }
    for u in &obj_info.usemtls {
        out.push(format!("um {} {} {}", u.face_start, u.face_end, u.name));
    }
    let obj_dir = std::path::Path::new(path).parent().map(|p| p.to_string_lossy().into_owned()).unwrap_or_default();
    let mut direct = TexturedMesh::default();
    let dok = obj_to_textured_mesh(&obj_info, &model.objects.last().unwrap().volumes[0].mesh().its, &mtl_data, &obj_dir, &mut direct);
    print_tex(&mut out, "direct", dok, &direct);
    let mut config = slicer::print_config::config_def::DynamicPrintConfig::new();
    let mut subs =
        slicer::config::ConfigSubstitutionContext::new(slicer::config::ForwardCompatibilitySubstitutionRule::EnableSilent);
    match Model::read_from_file(
        path,
        Some(&mut config),
        Some(&mut subs),
        LoadStrategy(1),
        None,
        None,
        None,
        None,
        None,
        None,
        None,
        0,
        None,
    ) {
        Ok(m2) => match &m2.texture_mesh {
            Some(t) => print_tex(&mut out, "read", true, t),
            None => out.push("read none".into()),
        },
        Err(e) => out.push(format!("read err {e}")),
    }
    out
}

#[test]
fn obj_to_textured_mesh_matches_the_pin() {
    std::env::set_current_dir(env!("CARGO_MANIFEST_DIR")).unwrap();
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/obj_textured/vectors.txt")).unwrap();
    let want: Vec<&str> = text.lines().filter(|l| !l.starts_with('#')).collect();
    let paths: Vec<&str> = want.iter().filter_map(|l| l.strip_prefix("case ")).collect();
    assert_eq!(paths.len(), 4);
    let mut got = Vec::new();
    for p in &paths {
        got.extend(rust_case(p));
    }
    let mut bad = 0;
    for (i, (g, w)) in got.iter().zip(want.iter()).enumerate() {
        if g != w {
            if bad < 20 {
                eprintln!("line {i}: got {g:?} want {w:?}");
            }
            bad += 1;
        }
    }
    assert_eq!(got.len(), want.len(), "line count");
    assert_eq!(bad, 0, "{bad} of {} lines differ", want.len());
    eprintln!("obj_to_textured_mesh: {} lines of {} files equal", want.len(), paths.len());
}
