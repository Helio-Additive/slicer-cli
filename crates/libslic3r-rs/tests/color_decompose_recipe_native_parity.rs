//! `color_decompose_recipe.rs` (and `blend_color_multi` of `filament_mixer.rs`) against Bambu
//! Studio's `ColorDecomposeRecipe.cpp` at the pin.
//!
//! `tests/data/color_decompose_recipe/vectors.txt` is written by `gen.cpp` of that directory,
//! which compiles the C++ unit unchanged over the table of standard recipes the pin ships (how to
//! build it is in its header). Each line is one call with its arguments and its result; the test
//! makes the same call and requires the same result, character for character. The table is read
//! from `references/BambuStudio/resources`, as the generator read it.

use slicer::color_decompose_recipe::*;
use slicer::filament_mixer::blend_color_multi;

const VECTORS: &str = include_str!("data/color_decompose_recipe/vectors.txt");

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn i(&mut self) -> i32 {
        self.s().parse().expect("integer")
    }
    fn u8(&mut self) -> u8 {
        self.s().parse().expect("byte")
    }
    fn string(&mut self) -> String {
        decode(self.s())
    }
    fn rgb(&mut self) -> ColorDecomposeRgb {
        ColorDecomposeRgb { r: self.u8(), g: self.u8(), b: self.u8() }
    }
    fn blend(&mut self) -> (Vec<String>, Vec<i32>) {
        let n = self.i();
        let mut hexes = Vec::new();
        let mut ratios = Vec::new();
        for _ in 0..n {
            hexes.push(self.string());
            ratios.push(self.i());
        }
        (hexes, ratios)
    }
}

/// A string token: `~` and the text, with `%XX` for the bytes gen.cpp encodes.
fn decode(token: &str) -> String {
    let bytes = token.strip_prefix('~').expect("string token").as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'%' {
            out.push(u8::from_str_radix(std::str::from_utf8(&bytes[i + 1..i + 3]).expect("hex"), 16).expect("hex"));
            i += 3;
        } else {
            out.push(bytes[i]);
            i += 1;
        }
    }
    String::from_utf8(out).expect("utf-8 text")
}

/// `tok` of gen.cpp.
fn tok(s: &str) -> String {
    let mut out = String::from("~");
    for &c in s.as_bytes() {
        if c <= b' ' || c == b'%' || c >= 0x7f {
            out += &format!("%{:02X}", c);
        } else {
            out.push(c as char);
        }
    }
    out
}

fn mode_of(value: i32) -> ColorDecomposeRecipeMode {
    match value {
        0 => ColorDecomposeRecipeMode::MaterialList,
        1 => ColorDecomposeRecipeMode::CMYW,
        2 => ColorDecomposeRecipeMode::RYBW,
        _ => ColorDecomposeRecipeMode::CMYKW,
    }
}

/// `print_result` of gen.cpp.
fn result_text(r: &ColorDecomposeRecipeResult) -> String {
    let mut out = format!("{} {} {} {}", r.valid as i32, r.mode as i32, tok(&r.matched_color_hex), r.components.len());
    for c in r.components.iter() {
        out += &format!(" {} {} {} {}", tok(&c.color_hex), tok(&c.base_color), c.ratio, c.filament_index);
    }
    out
}

#[test]
fn color_decompose_recipe_matches_cpp() {
    let resources = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../references/BambuStudio/resources");
    assert!(
        resources.join("filament_mixing/standard_color_recipes.json").is_file(),
        "the table of standard recipes is read from the BambuStudio submodule (references/BambuStudio)"
    );
    slicer::utils::set_resources_dir(resources.to_str().expect("utf-8 path"));

    let mut counts = std::collections::BTreeMap::<&str, usize>::new();
    for line in VECTORS.lines() {
        let (call, expected) = line.split_once(" -> ").expect("call and result");
        let mut t = Tok(call.split_whitespace());
        let tag = t.s();
        let got = match tag {
            "HEX" => {
                let hex = t.string();
                // a value that shows a result left alone
                let mut rgb = ColorDecomposeRgb { r: 1, g: 2, b: 3 };
                let ok = color_decompose_hex_to_rgb(&hex, &mut rgb);
                format!("{} {} {} {}", ok as i32, rgb.r, rgb.g, rgb.b)
            }
            "RGB" => tok(&color_decompose_rgb_to_hex(&t.rgb())),
            "FAM" => {
                let a = t.string();
                let b = t.string();
                (color_decompose_material_family_matches(&a, &b) as i32).to_string()
            }
            "STD" => {
                let target = t.rgb();
                let mode = mode_of(t.i());
                let material = t.string();
                result_text(&lookup_standard_recipe(&target, mode, &material))
            }
            "MEAS" => {
                let (hexes, ratios) = t.blend();
                tok(&lookup_measured_blend_color(&hexes, &ratios))
            }
            "BLEND" => {
                let (hexes, ratios) = t.blend();
                tok(&blend_color_multi(&hexes, &ratios))
            }
            "REC" => {
                let target = t.rgb();
                let preferred = t.string();
                let n = t.i();
                let mut filaments = Vec::new();
                for _ in 0..n {
                    let color_hex = t.string();
                    let name = t.string();
                    let r#type = t.string();
                    let is_mixed = t.i() != 0;
                    let filament_index = t.s().parse::<u32>().expect("index");
                    filaments.push(ColorDecomposePhysicalFilament { color_hex, name, r#type, is_mixed, filament_index });
                }
                result_text(&recommend_from_physical_filaments(&target, &filaments, &preferred))
            }
            other => panic!("tag {other}"),
        };
        assert!(t.0.next().is_none(), "tokens left in: {call}");
        assert_eq!(got, expected, "{call}");
        *counts.entry(tag).or_insert(0) += 1;
    }
    assert_eq!(counts["HEX"], 143);
    assert_eq!(counts["RGB"], 60);
    assert_eq!(counts["FAM"], 100);
    assert_eq!(counts["STD"], 420);
    assert_eq!(counts["MEAS"], 1500);
    assert_eq!(counts["BLEND"], 1500);
    assert_eq!(counts["REC"], 260);
    println!("color_decompose_recipe: {counts:?} equal to the C++");
}
