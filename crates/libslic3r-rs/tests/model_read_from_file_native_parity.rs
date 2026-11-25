//! `Model::read_from_file` (Model.cpp:306-491) against the pin: OBJ (plain and with vertex colours), AMF, STL
//! (ASCII, binary, empty, malformed, missing), an unknown extension and 3MF files, with LoadStrategy 0
//! (Default), 1 (AddDefaultInstances) and 12 (LoadModel | LoadConfig). Vectors from
//! `tests/data/model_read_from_file/gen.cpp`, run from this crate's directory with the same relative paths
//! (they become `ModelObject::input_file`): the exception message, or the objects' names, input files,
//! volume and instance counts, per instance its offset, rotation, scaling factor, mirror and printable flag, and
//! per volume the facet and vertex counts, the vertex sums, an index checksum, its name, source file, type,
//! source indices, source mesh offset and offset. `multi.amf` has two objects, two named volumes over one
//! vertex list, a flipped winding, a material and three instances (one a clone of its object); `multi_zip.amf`
//! is it in a zip archive (load_amf_archive / extract_model_from_archive), `notamf_zip.amf` an archive without an
//! .amf entry and `bad_zip.amf` one whose model the parser stops on.
//! All 60 cases are equal and asserted (R952: the 9 STL cases through `load_stl`'s Model::add_object).
use slicer::format::bbs_3mf::LoadStrategy;
use slicer::model::Model;

fn hexf(s: &str) -> f64 {
    let (neg, s) = match s.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, s),
    };
    if s == "0x0p+0" {
        return if neg { -0.0 } else { 0.0 };
    }
    let s = s.strip_prefix("0x").expect("hex float");
    let (mant, exp) = s.split_once('p').expect("exponent");
    let exp: i32 = exp.parse().expect("exp");
    let (int_part, frac_part) = mant.split_once('.').unwrap_or((mant, ""));
    let digits = format!("{int_part}{frac_part}");
    let m = u64::from_str_radix(&digits, 16).expect("mantissa");
    let v = (m as f64) * 2f64.powi(exp - 4 * frac_part.len() as i32);
    if neg {
        -v
    } else {
        v
    }
}

fn hex(s: &str) -> String {
    if s.is_empty() {
        return "-".to_string();
    }
    s.bytes().map(|b| format!("{:02x}", b)).collect()
}

/// The lines the generator prints for one case, with the vertex sums as numbers.
fn rust_case(path: &str, options: u32) -> Vec<String> {
    let mut out = Vec::new();
    let mut is_xxx = false;
    let mut subs = slicer::config::ConfigSubstitutionContext::new(slicer::config::ForwardCompatibilitySubstitutionRule::EnableSilent);
    let mut config = slicer::print_config::config_def::DynamicPrintConfig::new();
    match Model::read_from_file(
        path,
        Some(&mut config),
        Some(&mut subs),
        LoadStrategy(options),
        None,
        None,
        Some(&mut is_xxx),
        None,
        None,
        None,
        None,
        0,
        None,
    ) {
        Err(e) => out.push(format!("err {}", hex(&e.to_string()))),
        Ok(model) => {
            out.push(format!(
                "ok {} {} {} {}",
                model.objects.len(),
                is_xxx as i32,
                subs.substitutions.len(),
                model.texture_mesh.is_some() as i32
            ));
            for o in &model.objects {
                out.push(format!("obj {} {} {} {}", hex(&o.name), hex(&o.input_file), o.volumes.len(), o.instances.len()));
                for i in &o.instances {
                    let (off, rot, sc, mir) = (i.get_offset(), i.get_rotation(), i.get_scaling_factor(), i.get_mirror());
                    out.push(format!(
                        "ins {:e} {:e} {:e} {:e} {:e} {:e} {:e} {:e} {:e} {:e} {:e} {:e} {}",
                        off.x, off.y, off.z, rot.x, rot.y, rot.z, sc.x, sc.y, sc.z, mir.x, mir.y, mir.z, i.printable as i32
                    ));
                }
                for v in &o.volumes {
                    let its = &v.mesh().its;
                    let (mut sx, mut sy, mut sz) = (0f64, 0f64, 0f64);
                    for p in &its.vertices {
                        sx += p.x as f64;
                        sy += p.y as f64;
                        sz += p.z as f64;
                    }
                    let mut si: i64 = 0;
                    for f in &its.indices {
                        si += f[0] as i64 * 3 + f[1] as i64 * 5 + f[2] as i64 * 7;
                    }
                    out.push(format!("vol {} {} {:e} {:e} {:e} {}", its.indices.len(), its.vertices.len(), sx, sy, sz, si));
                    let (mo, vo) = (v.source.mesh_offset, v.get_offset());
                    out.push(format!(
                        "src {} {} {} {} {} {:e} {:e} {:e} {:e} {:e} {:e}",
                        hex(&v.name),
                        hex(&v.source.input_file),
                        v.type_() as i32,
                        v.source.object_idx,
                        v.source.volume_idx,
                        mo.x,
                        mo.y,
                        mo.z,
                        vo.x,
                        vo.y,
                        vo.z
                    ));
                }
            }
        }
    }
    out
}

/// A generator line with its `%a` sums turned into the Rust form of `rust_case`.
fn normalize(line: &str) -> String {
    if let Some(rest) = line.strip_prefix("vol ") {
        let f: Vec<&str> = rest.split(' ').collect();
        return format!("vol {} {} {:e} {:e} {:e} {}", f[0], f[1], hexf(f[2]), hexf(f[3]), hexf(f[4]), f[5]);
    }
    if let Some(rest) = line.strip_prefix("ins ") {
        let f: Vec<&str> = rest.split(' ').collect();
        let v: Vec<String> = f[..12].iter().map(|x| format!("{:e}", hexf(x))).collect();
        return format!("ins {} {}", v.join(" "), f[12]);
    }
    if let Some(rest) = line.strip_prefix("src ") {
        let f: Vec<&str> = rest.split(' ').collect();
        let v: Vec<String> = f[5..11].iter().map(|x| format!("{:e}", hexf(x))).collect();
        return format!("src {} {} {} {} {} {}", f[0], f[1], f[2], f[3], f[4], v.join(" "));
    }
    line.to_string()
}

#[test]
fn read_from_file_matches_the_pin() {
    let tmp = std::env::temp_dir().join("model_read_from_file_native_parity");
    std::fs::create_dir_all(&tmp).unwrap();
    slicer::utils::set_temporary_dir(tmp.to_str().unwrap());
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/model_read_from_file/vectors.txt")).unwrap();
    std::env::set_current_dir(env!("CARGO_MANIFEST_DIR")).unwrap();
    let mut cases: Vec<(String, u32, Vec<String>)> = Vec::new();
    for line in text.lines().filter(|l| !l.starts_with('#')) {
        if let Some(rest) = line.strip_prefix("case ") {
            let (path, opt) = rest.rsplit_once(' ').unwrap();
            cases.push((path.to_string(), opt.parse().unwrap(), Vec::new()));
        } else {
            cases.last_mut().unwrap().2.push(normalize(line));
        }
    }
    assert_eq!(cases.len(), 60);
    // NOT EQUAL YET (reported, not asserted): the STL loader builds its object with the port's
    // `ModelObject::new(name, mesh)` (stl.rs `load_stl`) where the pin calls `Model::add_object(name, path, mesh)`
    // (STL.cpp:38; model.rs `add_object_with_mesh`: one volume centred by `center_geometry_after_creation`, no
    // instance until `add_default_instances`). Those cases are counted; every other case (OBJ, AMF, errors,
    // messages, 3MF through load_bbs_3mf) must be equal.
    let mut bad = Vec::new();
    let mut loader_object_build = Vec::new();
    for (path, options, want) in &cases {
        let got = rust_case(path, *options);
        if &got != want {
            let geometry = [".stl", ".obj", ".amf"].iter().any(|e| path.ends_with(e));
            if geometry && got.first().map(|l| l.split(' ').nth(1)) == want.first().map(|l| l.split(' ').nth(1)) {
                loader_object_build.push(format!("{path} options {options}"));
            } else {
                bad.push(format!("{path} options {options}: got {got:?} want {want:?}"));
            }
        }
    }
    eprintln!(
        "read_from_file: {} of {} cases equal; {} differ in the loaders' object build (not equal yet): {:?}",
        cases.len() - bad.len() - loader_object_build.len(),
        cases.len(),
        loader_object_build.len(),
        loader_object_build
    );
    assert!(bad.is_empty(), "{} of {} cases differ:\n{}", bad.len(), cases.len(), bad.join("\n"));
    // R951: 12 (STL and AMF); R952: 9, the STL cases only (the AMF loader builds its volumes as AMF.cpp does).
    // R952 (worker A): 0, stl.rs `load_stl` builds the object with Model::add_object(name, path, mesh) as
    // STL.cpp:38 does (STL_LOAD_ADD_OBJECT_PIN) and the STL repair is admesh's (STL_ADMESH_REPAIR_PIN).
    assert!(loader_object_build.is_empty(), "cases differ in the loaders' object build: {loader_object_build:?}");
}
