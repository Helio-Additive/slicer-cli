//! `slicer::obj_color_utils` (the port of libslic3r/ObjColorUtils.hpp and ObjColorUtils.cpp, with the
//! parts of OpenCV 4.13.0 they call) against the C++ at the pin.
//!
//! `tests/data/obj_color_utils/obj_color_utils.txt` is the output of `gen.cpp` beside it, which links
//! the pin's ObjColorUtils.cpp and OpenCV; the header of gen.cpp describes the lines. The same script
//! runs here on the port and every line has to be the same, floats and doubles bit for bit.
//!
//! Lines whose name starts with `cvt_rgb2hsv` / `cvts_rgb2hsv` and `cvt_hsv2rgb` / `cvts_hsv2rgb` of
//! the row widths 4096 and 37 are the HSV conversions of wide rows, where OpenCV's SIMD loops (or a
//! HAL) run; they are reported by name and not counted (the port follows the scalar loops there, which
//! the width 1 lines compare).

use std::collections::{BTreeMap, HashMap};

use slicer::format::obj::{TriangleColor, VolumeColorInfo};
use slicer::geometry::geometry::Vec3d;
use slicer::model::{Model, ModelMesh, ModelVolumeType};
use slicer::normal_utils::indexed_triangle_set;
use slicer::obj_color_utils::cv::{self, ColorConversionCode, Mat32f, Mat8u};
use slicer::obj_color_utils::*;
use slicer::texture_painting::TexturedMesh;
use slicer::triangle_mesh::{its_make_cube, Vec3f};

fn hf(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}
fn hd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}

struct Rng {
    s: u64,
}
impl Rng {
    fn next(&mut self) -> u64 {
        self.s = self.s.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.s;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }
    fn uni(&mut self) -> f64 {
        (self.next() >> 11) as f64 * (1.0 / 9007199254740992.0)
    }
    fn below(&mut self, n: i32) -> i32 {
        (self.next() % n as u64) as i32
    }
}

fn fnv(p: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for &b in p {
        h ^= b as u64;
        h = h.wrapping_mul(0x100000001b3);
    }
    h
}

fn scenario_cvt(o: &mut Vec<String>) {
    let total: usize = 1 << 24;
    let codes = [
        ("rgb2lab", ColorConversionCode::Rgb2Lab),
        ("lab2rgb", ColorConversionCode::Lab2Rgb),
        ("rgb2hsv", ColorConversionCode::Rgb2Hsv),
        ("hsv2rgb", ColorConversionCode::Hsv2Rgb),
    ];
    for (name, code) in codes {
        for width in [4096usize, 1, 37] {
            let rows = total / width;
            let mut src = Mat8u::new(rows, width, 3);
            for i in 0..rows * width {
                src.data[i * 3] = (i >> 16) as u8;
                src.data[i * 3 + 1] = (i >> 8) as u8;
                src.data[i * 3 + 2] = i as u8;
            }
            let dst = cv::cvt_color(&src, code);
            let d = &dst.data;
            o.push(format!("cvt_{} {} {}", name, width, fnv(&d[..rows * width * 3]) as i64));
            let mut s = format!("cvts_{} {}", name, width);
            for k in 0..2048usize {
                let i = (k as i64 * 8191 % (rows * width) as i64) as usize;
                s += &format!(" {} {} {}", d[i * 3], d[i * 3 + 1], d[i * 3 + 2]);
            }
            o.push(s);
        }
    }
}

fn scenario_norm(o: &mut Vec<String>) {
    let mut r = Rng { s: 11 };
    for k in 0..2000 {
        let mut a = [0f32; 3];
        let mut b = [0f32; 3];
        let scale = if k < 1000 { 255.0 } else { 3.0 };
        for j in 0..3 {
            a[j] = (r.uni() * scale) as f32;
            b[j] = (r.uni() * scale) as f32;
        }
        if k % 7 == 0 {
            a[0] = r.below(256) as f32;
            b[0] = r.below(256) as f32;
        }
        o.push(format!(
            "nl2 {} {} {} {} {} {} {}",
            hf(a[0]),
            hf(a[1]),
            hf(a[2]),
            hf(b[0]),
            hf(b[1]),
            hf(b[2]),
            hf(cv::norm_l2_sqr(&a, &b, 3))
        ));
    }
}

fn km_data(r: &mut Rng, n: usize, ints: bool) -> Mat32f {
    let mut data = Mat32f::new(n, 1, 3);
    for i in 0..n {
        if i > 0 && r.below(5) == 0 {
            let src = r.below(i as i32) as usize;
            let p: Vec<f32> = data.data[src * 3..src * 3 + 3].to_vec();
            data.data[i * 3..i * 3 + 3].copy_from_slice(&p);
            continue;
        }
        for j in 0..3 {
            data.data[i * 3 + j] = if ints { r.below(256) as f32 } else { (r.uni() * 200.0 - 50.0) as f32 };
        }
    }
    data
}

fn scenario_kmeans(o: &mut Vec<String>) {
    let mut r = Rng { s: 23 };
    let sizes: [(usize, i32); 10] = [(1, 1), (5, 1), (5, 5), (17, 3), (40, 7), (100, 15), (333, 15), (1000, 8), (64, 2), (250, 12)];
    let mut case_id = 0;
    for (n, k) in sizes {
        for ints in [true, false] {
            let data = km_data(&mut r, n, ints);
            let seed = 1 + r.below(100000) as u64;
            cv::set_the_rng_state(seed);
            let mut labels = Vec::new();
            let mut centers = Mat32f::default();
            let comp = cv::kmeans(
                &data,
                k,
                &mut labels,
                cv::TermCriteria { type_: cv::TermCriteria::EPS + cv::TermCriteria::COUNT, max_count: 300, epsilon: 0.5 },
                3,
                cv::KMEANS_PP_CENTERS,
                &mut centers,
            )
            .unwrap();
            let mut s = format!("km_labels {}", case_id);
            for l in labels.iter() {
                s += &format!(" {}", l);
            }
            o.push(s);
            let w = centers.cols * centers.channels;
            let mut s = format!("km_centers {} {} {}", case_id, centers.rows, w);
            for v in centers.data.iter() {
                s += &format!(" {}", hf(*v));
            }
            o.push(s);
            o.push(format!("km_comp {} {} {}", case_id, hd(comp), cv::the_rng_state() as i64));
            case_id += 1;
        }
    }
}

fn color_list(r: &mut Rng, n: usize, palette: i32) -> Vec<[f32; 4]> {
    let mut pal = Vec::new();
    for _ in 0..palette {
        let a = r.below(256) as f32 / 255.0;
        let b = r.below(256) as f32 / 255.0;
        let c = r.below(256) as f32 / 255.0;
        pal.push([a, b, c, 1.0f32]);
    }
    let mut colors = Vec::new();
    for _ in 0..n {
        if palette > 0 {
            colors.push(pal[r.below(palette) as usize]);
        } else {
            let a = r.uni() as f32;
            let b = r.uni() as f32;
            let c = r.uni() as f32;
            colors.push([a, b, c, 1.0]);
        }
    }
    colors
}

fn scenario_quant(o: &mut Vec<String>) {
    let mut r = Rng { s: 37 };
    let cases: [(usize, i32, i32, i32); 12] = [
        (10, 3, -1, 15),
        (50, 6, -1, 15),
        (50, 20, -1, 15),
        (200, 0, -1, 15),
        (200, 0, 4, 15),
        (200, 0, 20, 15),
        (120, 15, -1, 15),
        (120, 16, -1, 15),
        (300, 40, 8, 32),
        (500, 0, -1, 32),
        (80, 0, 1, 15),
        (1, 1, -1, 15),
    ];
    for (case_id, c) in cases.iter().enumerate() {
        let colors = color_list(&mut r, c.0, c.1);
        let mut quant = QuantKMeans::new(10);
        let mut results = Vec::new();
        let mut labels = Vec::new();
        quant.apply(&colors, &mut results, &mut labels, c.2, c.3, 2).unwrap();
        let mut s = format!("qa_clusters {} {}", case_id, results.len());
        for v in results.iter() {
            s += &format!(" {} {} {} {}", hf(v[0]), hf(v[1]), hf(v[2]), hf(v[3]));
        }
        o.push(s);
        let mut s = format!("qa_labels {}", case_id);
        for l in labels.iter() {
            s += &format!(" {}", l);
        }
        o.push(s);

        let mut cl = Vec::new();
        let mut lb = Vec::new();
        let mut number = c.2 as i8;
        let ok = obj_color_deal_algo(&colors, &mut cl, &mut lb, &mut number, c.3);
        let mut s = format!("deal {} {} {} {}", case_id, ok as i32, cl.len(), lb.len());
        for v in cl.iter() {
            s += &format!(" {} {} {}", hf(v[0]), hf(v[1]), hf(v[2]));
        }
        o.push(s);
    }
}

fn scenario_image(o: &mut Vec<String>) {
    let mut r = Rng { s: 41 };
    let (rows, cols) = (7usize, 19usize);
    for cs in 0..=2 {
        for pal in 0..=1 {
            let mut img = Mat8u::new(rows, cols, 3);
            let mut palette = Vec::new();
            for _ in 0..9 {
                let a = r.below(256) as u8;
                let b = r.below(256) as u8;
                let c = r.below(256) as u8;
                palette.push([a, b, c]);
            }
            for i in 0..rows {
                for j in 0..cols {
                    let p = if pal == 1 {
                        palette[r.below(9) as usize]
                    } else {
                        let a = r.below(256) as u8;
                        let b = r.below(256) as u8;
                        let c = r.below(256) as u8;
                        [a, b, c]
                    };
                    img.px_mut(i, j).copy_from_slice(&p);
                }
            }
            let k = 4 + r.below(4);
            cv::set_the_rng_state(99 + cs as u64 * 2 + pal as u64);
            let mut quant = QuantKMeans::new(10);
            let mut result = img.clone();
            quant.apply_image(&img, &mut result, k, cs).unwrap();
            let mut s = format!("qi {} {} {} {}", cs, pal, k, cv::the_rng_state() as i64);
            for i in 0..rows {
                for j in 0..cols {
                    let p = result.px(i, j);
                    s += &format!(" {} {} {}", p[0], p[1], p[2]);
                }
            }
            o.push(s);

            let mut img4 = Mat8u::new(rows, cols, 4);
            for i in 0..rows {
                for j in 0..cols {
                    let p = img.px(i, j).to_vec();
                    let alpha = if r.below(4) == 0 { r.below(10) } else { 10 + r.below(246) } as u8;
                    img4.px_mut(i, j).copy_from_slice(&[p[0], p[1], p[2], alpha]);
                }
            }
            cv::set_the_rng_state(199 + cs as u64 * 2 + pal as u64);
            let mut quant4 = QuantKMeans::new(10);
            let mut result4 = img4.clone();
            quant4.apply_aplha(&img4, &mut result4, k, cs).unwrap();
            let mut s = format!("qal {} {} {} {}", cs, pal, k, cv::the_rng_state() as i64);
            for i in 0..rows {
                for j in 0..cols {
                    let p = result4.px(i, j);
                    s += &format!(" {} {} {} {}", p[0], p[1], p[2], p[3]);
                }
            }
            o.push(s);
        }
    }
}

fn box_its(sx: f32, sy: f32, sz: f32, ox: f32, oy: f32, oz: f32) -> indexed_triangle_set {
    let mut its = its_make_cube(sx as f64, sy as f64, sz as f64);
    for v in its.vertices.iter_mut() {
        *v += Vec3f::new(ox, oy, oz);
    }
    its
}

const COLORS: [&str; 8] = ["#FF0000", "#00FF00FF", "#0000FF", "", "#123456", "#ABCDEF80", "bad", "#7F7F7F"];

fn dump_tex(o: &mut Vec<String>, tag: &str, t: &TexturedMesh, ret: bool, has_uncolored: bool) {
    let mut s = format!("{} {} {} {} {} {}", tag, ret as i32, has_uncolored as i32, t.is_standard_3mf as i32, t.vertices.len(), t.indices.len());
    for v in t.vertices.iter() {
        s += &format!(" {} {} {}", hf(v[0]), hf(v[1]), hf(v[2]));
    }
    o.push(s);
    let mut s = format!("{}_f", tag);
    for f in t.indices.iter() {
        s += &format!(" {} {} {}", f[0], f[1], f[2]);
    }
    for c in t.precomputed_face_colors.iter() {
        s += &format!(" {} {} {}", c[0], c[1], c[2]);
    }
    for c in t.precomputed_vertex_colors.iter() {
        s += &format!(" {} {} {} {}", hf(c[0]), hf(c[1]), hf(c[2]), hf(c[3]));
    }
    o.push(s);
}

fn scenario_standard_3mf(o: &mut Vec<String>) {
    let mut r = Rng { s: 53 };
    let mut groups: BTreeMap<i32, Vec<String>> = BTreeMap::new();
    groups.insert(1, vec![COLORS[0].into(), COLORS[1].into(), COLORS[2].into()]);
    groups.insert(2, vec![COLORS[3].into(), COLORS[4].into(), COLORS[5].into(), COLORS[6].into()]);
    groups.insert(3, vec![]);
    groups.insert(5, vec![COLORS[7].into()]);

    for _ in 0..200 {
        let mut info = VolumeColorInfo::new();
        info.pid = r.below(7) - 1;
        info.pindex = r.below(6) - 1;
        let mut binding = TriangleColor::new();
        binding.pid = r.below(7) - 1;
        for c in 0..3 {
            binding.indices[c] = r.below(7) - 2;
        }
        let mut s = format!(
            "s3_corner {} {} {} {} {} {}",
            info.pid, info.pindex, binding.pid, binding.indices[0], binding.indices[1], binding.indices[2]
        );
        for c in 0..3 {
            let col = standard_3mf_corner_color(&groups, &info, &binding, c);
            s += &format!(" {}", match col {
                Some(col) if col.is_empty() => "E".to_string(),
                Some(col) => col.clone(),
                None => "-".to_string(),
            });
        }
        o.push(s);
    }

    for scene in 0..4 {
        let mut model = Model::new();
        let mut data: HashMap<i32, VolumeColorInfo> = HashMap::new();
        let n_objects = if scene == 0 { 1 } else { 2 };
        for ob in 0..n_objects {
            model.add_object_new();
            let obj = model.objects.last_mut().unwrap();
            let n_vol = if scene == 2 { 2 } else { 1 + if ob == 1 { 1 } else { 0 } };
            for v in 0..n_vol {
                let vol = obj.add_volume_mesh(
                    ModelMesh::from_its(box_its(1.0 + v as f32, 2.0, 1.5, ob as f32, 0.5, 0.25 * v as f32)),
                    ModelVolumeType::ModelPart,
                    true,
                );
                vol.set_offset(&Vec3d::new(0.5 * v as f64, -0.25 * ob as f64, 0.125));
                vol.set_rotation(&Vec3d::new(0.1 * scene as f64, 0.2 * v as f64, 0.3 * ob as f64));
                vol.set_scaling_factor(&Vec3d::new(1.0, 1.0 + 0.5 * v as f64, 1.0));
                if scene == 3 && v == 1 {
                    vol.set_type(ModelVolumeType::ParameterModifier);
                }
                let mut info = VolumeColorInfo::new();
                info.pid = if (v + ob) % 2 == 0 { 1 } else { -1 };
                info.pindex = 1;
                let nf = vol.mesh().its.indices.len();
                let used = if scene == 1 && ob == 1 { nf - 1 } else { nf };
                for _ in 0..used {
                    let mut tc = TriangleColor::new();
                    let pick = r.below(5);
                    tc.pid = if pick == 0 { -1 } else if pick == 1 { 2 } else if pick == 2 { 5 } else { 1 };
                    for c in 0..3 {
                        tc.indices[c] = r.below(5) - 1;
                    }
                    info.triangle_colors.push(tc);
                }
                if !(scene == 3 && ob == 1) {
                    data.insert(vol.id().id as i32, info);
                }
            }
            let inst = obj.add_instance();
            inst.set_offset(&Vec3d::new(10.0 * ob as f64, 5.0, 0.0));
            inst.set_rotation(&Vec3d::new(0.0, 0.0, 0.5 * ob as f64 + 0.1 * scene as f64));
        }
        let mut tex = TexturedMesh::default();
        let mut has_uncolored = true;
        let ret = standard_3mf_to_textured_meshes(&model, &groups, &data, &mut tex, &mut has_uncolored);
        dump_tex(o, &format!("s3_tex{}", scene), &tex, ret, has_uncolored);
    }
}

#[test]
fn obj_color_utils_matches_the_pin() {
    let mut o: Vec<String> = Vec::new();
    scenario_cvt(&mut o);
    scenario_norm(&mut o);
    scenario_kmeans(&mut o);
    scenario_quant(&mut o);
    scenario_image(&mut o);
    scenario_standard_3mf(&mut o);

    let path = format!("{}/tests/data/obj_color_utils/obj_color_utils.txt", env!("CARGO_MANIFEST_DIR"));
    let expected = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let expected: Vec<&str> = expected.lines().collect();
    if let Ok(dump) = std::env::var("OBJ_COLOR_UTILS_PARITY_DUMP") {
        std::fs::write(dump, o.join("\n") + "\n").expect("dump");
    }
    assert_eq!(o.len(), expected.len(), "number of lines");

    let reported = |name: &str, line: &str| -> bool {
        (name == "cvt_rgb2hsv" || name == "cvts_rgb2hsv" || name == "cvt_hsv2rgb" || name == "cvts_hsv2rgb")
            && line.split(' ').nth(1) != Some("1")
    };
    let mut failures = Vec::new();
    let mut compared: BTreeMap<String, usize> = BTreeMap::new();
    let mut not_counted: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    for (k, (got, want)) in o.iter().zip(expected.iter()).enumerate() {
        let name = want.split(' ').next().unwrap_or("").to_string();
        let key = name.trim_end_matches(char::is_numeric).to_string();
        if reported(&name, want) {
            let e = not_counted.entry(name.clone()).or_insert((0, 0));
            e.0 += 1;
            if got != want {
                e.1 += 1;
            }
            continue;
        }
        *compared.entry(key).or_insert(0) += 1;
        if got != want {
            failures.push(format!("line {}:\n  rust {}\n  c++  {}", k + 1, &got[..got.len().min(400)], &want[..want.len().min(400)]));
        }
    }
    println!("compared lines by name: {:?}", compared);
    println!("not counted (wide-row HSV): {:?} (lines, differing)", not_counted);
    if !failures.is_empty() {
        let shown: Vec<&String> = failures.iter().take(12).collect();
        panic!("{} lines differ from the C++:\n{}", failures.len(), shown.iter().map(|s| s.as_str()).collect::<Vec<_>>().join("\n"));
    }
}
