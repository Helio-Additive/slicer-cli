//! Port of `ColorDecomposeRecipe.hpp` / `ColorDecomposeRecipe.cpp`: the recipe (components and
//! ratios) that mixes a target color, from the physical filaments at hand or from the table of
//! measured standard recipes, and the measured color of a blend.
//!
//! C++ Reference:
//! - src/libslic3r/ColorDecomposeRecipe.hpp
//! - src/libslic3r/ColorDecomposeRecipe.cpp
//!
//! Floating point follows the reference build (clang, `-ffp-contract=on`): `a * b + c` inside one
//! source expression is one fused operation, written `mul_add` here.
//!
//! FIDELITY-NOTE: `std::pow(x, 2.0)` (ColorDecomposeRecipe.cpp:103, 551-552) is written `x * x`.
//! clang turns that call into the multiplication, and the product is not fused with the sum around
//! it: contraction applies to products written in the source.
//!
//! FIDELITY-NOTE: the table is read with `serde_json` where C++ uses nlohmann::json. Both accept
//! the same documents and give the same values for them, with two differences that the shipped
//! table does not meet: a number written `-0` is an integer for nlohmann and a float for
//! serde_json (so it would be skipped as a ratio), and a field of the wrong type (a number where a
//! string is expected) throws `type_error` in C++ and panics here.
//!
//! Nothing in the slicing path calls this unit. Its callers in C++ are `blend_color_multi`
//! (FilamentMixer.cpp:120, ported with this unit in `filament_mixer.rs`) and the desktop dialogs
//! (MixedFilamentDialog.cpp, TextureImportDialog.cpp).

use std::collections::HashMap;
use std::sync::OnceLock;

use crate::clipper::libcxx_sort;
use crate::filament_mixer::blend_color_multi;
use crate::utils::resources_dir;

/// ColorDecomposeRecipe.hpp:9-14
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ColorDecomposeRecipeMode {
    #[default]
    MaterialList,
    CMYW,
    RYBW,
    CMYKW,
}

/// ColorDecomposeRecipe.hpp:16-20
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ColorDecomposeRgb {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

/// ColorDecomposeRecipe.hpp:22-28
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ColorDecomposePhysicalFilament {
    pub color_hex: String,
    pub name: String,
    pub r#type: String,
    pub is_mixed: bool,
    /// 1-based physical filament index
    pub filament_index: u32,
}

/// ColorDecomposeRecipe.hpp:30-35
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ColorDecomposeRecipeComponent {
    pub color_hex: String,
    pub base_color: String,
    pub ratio: i32,
    /// 1-based for physical filaments, 0 for standard base colors
    pub filament_index: u32,
}

/// ColorDecomposeRecipe.hpp:37-42
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct ColorDecomposeRecipeResult {
    pub valid: bool,
    pub mode: ColorDecomposeRecipeMode,
    pub matched_color_hex: String,
    pub components: Vec<ColorDecomposeRecipeComponent>,
}

/// ColorDecomposeRecipe.cpp:18-22
#[derive(Debug, Clone, Copy, PartialEq, Default)]
struct LabColor {
    l: f64,
    a: f64,
    b: f64,
}

/// ColorDecomposeRecipe.cpp:24-33
#[derive(Debug, Clone)]
struct StandardRecipeEntry {
    mode: ColorDecomposeRecipeMode,
    material: String,
    source: String,
    component_keys: Vec<String>,
    component_hexes: Vec<String>,
    ratios: Vec<i32>,
    measured_hex: String,
    measured_lab: LabColor,
}

impl Default for StandardRecipeEntry {
    /// ColorDecomposeRecipe.cpp:25: the mode of a new entry is CMYW.
    fn default() -> Self {
        StandardRecipeEntry {
            mode: ColorDecomposeRecipeMode::CMYW,
            material: String::new(),
            source: String::new(),
            component_keys: Vec::new(),
            component_hexes: Vec::new(),
            ratios: Vec::new(),
            measured_hex: String::new(),
            measured_lab: LabColor::default(),
        }
    }
}

/// ColorDecomposeRecipe.cpp:35-39
fn srgb_to_linear(mut v: f64) -> f64 {
    v /= 255.0;
    if v <= 0.04045 {
        v / 12.92
    } else {
        ((v + 0.055) / 1.055).powf(2.4)
    }
}

/// ColorDecomposeRecipe.cpp:41-46
fn xyz_to_lab_component(v: f64) -> f64 {
    const EPS: f64 = 216.0 / 24389.0;
    const KAPPA: f64 = 24389.0 / 27.0;
    if v > EPS {
        v.cbrt()
    } else {
        KAPPA.mul_add(v, 16.0) / 116.0
    }
}

/// ColorDecomposeRecipe.cpp:48-63
fn rgb_to_lab(rgb: &ColorDecomposeRgb) -> LabColor {
    let r = srgb_to_linear(rgb.r as f64);
    let g = srgb_to_linear(rgb.g as f64);
    let b = srgb_to_linear(rgb.b as f64);

    let x = 0.1804375_f64.mul_add(b, 0.4124564_f64.mul_add(r, 0.3575761 * g)) / 0.95047;
    let y = 0.0721750_f64.mul_add(b, 0.2126729_f64.mul_add(r, 0.7151522 * g));
    let z = 0.9503041_f64.mul_add(b, 0.0193339_f64.mul_add(r, 0.1191920 * g)) / 1.08883;

    let fx = xyz_to_lab_component(x);
    let fy = xyz_to_lab_component(y);
    let fz = xyz_to_lab_component(z);

    LabColor { l: 116.0_f64.mul_add(fy, -16.0), a: 500.0 * (fx - fy), b: 200.0 * (fy - fz) }
}

/// ColorDecomposeRecipe.cpp:65-99
fn lab_to_srgb_hex(lab: &LabColor) -> String {
    const XN: f64 = 0.95047;
    const YN: f64 = 1.0;
    const ZN: f64 = 1.08883;

    let f_inv = |t: f64| -> f64 {
        const EPS: f64 = 216.0 / 24389.0;
        const KAPPA: f64 = 24389.0 / 27.0;
        let t3 = t * t * t;
        if t3 > EPS {
            t3
        } else {
            t.mul_add(116.0, -16.0) / KAPPA
        }
    };

    let fy = (lab.l + 16.0) / 116.0;
    let fx = lab.a / 500.0 + fy;
    let fz = fy - lab.b / 200.0;

    let x = XN * f_inv(fx);
    let y = YN * f_inv(fy);
    let z = ZN * f_inv(fz);

    let r = (-0.4986_f64).mul_add(z, 3.2406_f64.mul_add(x, -(1.5372 * y)));
    let g = 0.0415_f64.mul_add(z, (-0.9689_f64).mul_add(x, 1.8758 * y));
    let b = 1.0570_f64.mul_add(z, 0.0557_f64.mul_add(x, -(0.2040 * y)));

    let gamma = |c: f64| -> f64 {
        // std::max(0.0, std::min(1.0, c)), with the comparisons std::min and std::max make.
        let c = if c < 1.0 { c } else { 1.0 };
        let c = if 0.0 < c { c } else { 0.0 };
        if c <= 0.0031308 {
            12.92 * c
        } else {
            1.055_f64.mul_add(c.powf(1.0 / 2.4), -0.055)
        }
    };
    let u8_ = |c: f64| -> i32 { 0.max(255.min((gamma(c) * 255.0).round() as i64 as i32)) };

    format!("#{:02X}{:02X}{:02X}", u8_(r), u8_(g), u8_(b))
}

/// ColorDecomposeRecipe.cpp:101-104
fn delta_e76(a: &LabColor, b: &LabColor) -> f64 {
    let dl = a.l - b.l;
    let da = a.a - b.a;
    let db = a.b - b.b;
    (dl * dl + da * da + db * db).sqrt()
}

/// ColorDecomposeRecipe.cpp:106-113 — series-level match, used to pick standard recipe entries.
/// Keeps distinct official series ("PLA Basic" vs "PLA Palette") from sharing measured data.
fn material_matches(a: &str, b: &str) -> bool {
    if a.is_empty() || b.is_empty() {
        return false;
    }
    a == b || a == b.to_string() + " Basic" || b == a.to_string() + " Basic"
}

/// ColorDecomposeRecipe.cpp:115-119
fn material_family(type_: &str) -> &str {
    match type_.find(' ') {
        None => type_,
        Some(sp) => &type_[..sp],
    }
}

/// ColorDecomposeRecipe.cpp:121-136
fn ratio_grid(n: usize) -> Vec<Vec<i32>> {
    let mut out: Vec<Vec<i32>> = Vec::new();
    if n == 2 {
        let mut a = 20;
        while a <= 80 {
            out.push(vec![a, 100 - a]);
            a += 5;
        }
    } else if n == 3 {
        let mut a = 20;
        while a <= 60 {
            let mut b = 20;
            while b <= 80 - a {
                let c = 100 - a - b;
                if c >= 20 {
                    out.push(vec![a, b, c]);
                }
                b += 5;
            }
            a += 5;
        }
    }
    out
}

/// ColorDecomposeRecipe.cpp:138-145
fn parse_mode(s: &str) -> ColorDecomposeRecipeMode {
    if s == "RYBW" || s == "RGBY" {
        return ColorDecomposeRecipeMode::RYBW;
    }
    if s == "CMYKW" {
        return ColorDecomposeRecipeMode::CMYKW;
    }
    ColorDecomposeRecipeMode::CMYW
}

/// `basic_json::value(key, default)` for a string: the value of `key` if the object has it, else
/// the default. A value that is not a string throws `type_error.302` in C++.
fn json_value_string(item: &serde_json::Value, key: &str, default_value: &str) -> String {
    match item.get(key) {
        None => default_value.to_string(),
        Some(serde_json::Value::String(s)) => s.clone(),
        Some(other) => panic!("[json.exception.type_error.302] type must be string, but is {}", json_type_name(other)),
    }
}

/// `basic_json::get<double>()`: a number, or a boolean as 0 or 1; anything else throws
/// `type_error.302` in C++.
fn json_get_double(value: &serde_json::Value) -> f64 {
    match value {
        serde_json::Value::Number(n) => n.as_f64().unwrap_or(f64::NAN),
        serde_json::Value::Bool(b) => {
            if *b {
                1.0
            } else {
                0.0
            }
        }
        other => panic!("[json.exception.type_error.302] type must be number, but is {}", json_type_name(other)),
    }
}

/// `basic_json::type_name()`
fn json_type_name(value: &serde_json::Value) -> &'static str {
    match value {
        serde_json::Value::Null => "null",
        serde_json::Value::Bool(_) => "boolean",
        serde_json::Value::Number(_) => "number",
        serde_json::Value::String(_) => "string",
        serde_json::Value::Array(_) => "array",
        serde_json::Value::Object(_) => "object",
    }
}

/// ColorDecomposeRecipe.cpp:147-200
fn load_standard_entries() -> Vec<StandardRecipeEntry> {
    let mut entries: Vec<StandardRecipeEntry> = Vec::new();
    let path = resources_dir().to_string_lossy().into_owned() + "/filament_mixing/standard_color_recipes.json";
    let Ok(bytes) = std::fs::read(&path) else {
        return entries;
    };

    // nlohmann::json::parse(ifs, nullptr, false): a document that does not parse is discarded. The
    // parser skips a UTF-8 byte order mark.
    let text = bytes.strip_prefix(b"\xEF\xBB\xBF").unwrap_or(&bytes);
    let Ok(root) = serde_json::from_slice::<serde_json::Value>(text) else {
        return entries;
    };
    let Some(items) = root.get("entries").and_then(|entries| entries.as_array()) else {
        return entries;
    };

    for item in items {
        if !item.is_object() {
            continue;
        }
        let mut entry = StandardRecipeEntry::default();
        entry.mode = parse_mode(&json_value_string(item, "mode", "CMYW"));
        entry.material = json_value_string(item, "material", "");
        entry.source = json_value_string(item, "source", "");
        entry.measured_hex = json_value_string(item, "measured_rgb", "");

        if let Some(components) = item.get("components").and_then(|components| components.as_array()) {
            for comp in components {
                if comp.is_object() {
                    entry.component_keys.push(json_value_string(comp, "key", ""));
                    entry.component_hexes.push(json_value_string(comp, "rgb", ""));
                }
            }
        }
        if let Some(ratios) = item.get("ratios").and_then(|ratios| ratios.as_array()) {
            for ratio in ratios {
                // is_number_integer() || is_number_unsigned(), then get<int>()
                if let Some(value) = ratio.as_i64() {
                    entry.ratios.push(value as i32);
                } else if let Some(value) = ratio.as_u64() {
                    entry.ratios.push(value as i32);
                }
            }
        }
        match item.get("measured_lab").and_then(|lab| lab.as_array()).filter(|lab| lab.len() >= 3) {
            Some(lab) => {
                entry.measured_lab =
                    LabColor { l: json_get_double(&lab[0]), a: json_get_double(&lab[1]), b: json_get_double(&lab[2]) };
            }
            None => {
                let mut measured_rgb = ColorDecomposeRgb::default();
                if !color_decompose_hex_to_rgb(&entry.measured_hex, &mut measured_rgb) {
                    continue;
                }
                entry.measured_lab = rgb_to_lab(&measured_rgb);
            }
        }

        if entry.component_hexes.len() >= 2
            && entry.component_hexes.len() == entry.ratios.len()
            && !entry.measured_hex.is_empty()
        {
            entries.push(entry);
        }
    }
    entries
}

/// ColorDecomposeRecipe.cpp:202-206 — the table, read once.
fn standard_entries() -> &'static Vec<StandardRecipeEntry> {
    static ENTRIES: OnceLock<Vec<StandardRecipeEntry>> = OnceLock::new();
    ENTRIES.get_or_init(load_standard_entries)
}

/// ColorDecomposeRecipe.cpp:208-242 — canonical form of a component list: (normalized hex, ratio)
/// pairs sorted so that matching is independent of the caller's component order. The key is the
/// concatenated hexes, which also encodes the component count because every normalized hex is
/// exactly 7 characters. Returns false on any invalid hex.
fn canonicalize_blend_components(hexes: &[String], ratios: &[i32], key_out: &mut String, ratios_out: &mut Vec<i32>) -> bool {
    let n = hexes.len();
    if n == 0 || n != ratios.len() {
        return false;
    }

    let mut pairs: Vec<(String, i32)> = Vec::with_capacity(n);
    for i in 0..n {
        let mut rgb = ColorDecomposeRgb::default();
        if !color_decompose_hex_to_rgb(&hexes[i], &mut rgb) {
            return false;
        }
        pairs.push((format!("#{:02X}{:02X}{:02X}", rgb.r, rgb.g, rgb.b), ratios[i]));
    }
    // std::sort with operator< of the pair: equal elements are identical, so their order is free.
    pairs.sort();

    key_out.clear();
    key_out.reserve(n * 7);
    ratios_out.clear();
    ratios_out.reserve(n);
    for p in pairs.iter() {
        *key_out += &p.0;
        ratios_out.push(p.1);
    }
    true
}

/// ColorDecomposeRecipe.cpp:244-248
#[derive(Debug, Clone, Default)]
struct MeasuredBlendAnchor {
    /// reordered to match the canonical hex order
    ratios: Vec<i32>,
    lab: LabColor,
    hex: String,
}

/// ColorDecomposeRecipe.cpp:250-272 — measured/interpolated entries grouped by their canonical
/// component set. Built once: re-normalizing and re-sorting every entry's components on each lookup
/// used to dominate the decomposition search, which issues one lookup per point of the ratio grid
/// for every candidate filament combination.
///
/// The map is only searched by key, never iterated, so the order of `std::unordered_map` does not
/// show; the anchors of a key are in table order.
fn measured_blend_index() -> &'static HashMap<String, Vec<MeasuredBlendAnchor>> {
    static INDEX: OnceLock<HashMap<String, Vec<MeasuredBlendAnchor>>> = OnceLock::new();
    INDEX.get_or_init(|| {
        let mut out: HashMap<String, Vec<MeasuredBlendAnchor>> = HashMap::new();
        for entry in standard_entries() {
            if entry.source != "measured" && entry.source != "interpolated" {
                continue;
            }
            let mut key = String::new();
            let mut anchor = MeasuredBlendAnchor::default();
            if !canonicalize_blend_components(&entry.component_hexes, &entry.ratios, &mut key, &mut anchor.ratios) {
                continue;
            }
            anchor.lab = entry.measured_lab;
            anchor.hex = entry.measured_hex.clone();
            out.entry(key).or_default().push(anchor);
        }
        out
    })
}

/// ColorDecomposeRecipe.cpp:274-303
fn evaluate_candidate(
    target: &ColorDecomposeRgb,
    hexes: &[String],
    ratios: &[i32],
    indices: &[u32],
    mode: ColorDecomposeRecipeMode,
    best_score: &mut f64,
    best: &mut ColorDecomposeRecipeResult,
) {
    let mixed = blend_color_multi(hexes, ratios);
    let mut mixed_rgb = ColorDecomposeRgb::default();
    if !color_decompose_hex_to_rgb(&mixed, &mut mixed_rgb) {
        return;
    }

    let score = delta_e76(&rgb_to_lab(target), &rgb_to_lab(&mixed_rgb));
    if score >= *best_score {
        return;
    }

    *best_score = score;
    best.valid = true;
    best.mode = mode;
    best.matched_color_hex = mixed;
    best.components.clear();
    for i in 0..hexes.len() {
        let mut comp = ColorDecomposeRecipeComponent::default();
        comp.color_hex = hexes[i].clone();
        comp.ratio = ratios[i];
        comp.filament_index = if i < indices.len() { indices[i] } else { 0 };
        best.components.push(comp);
    }
}

/// ColorDecomposeRecipe.cpp:307-312 — whether two physical filaments are mixable, i.e. belong to the
/// same material family. Official series names ("PLA Basic", "PLA Palette", "PETG Basic") normalize
/// to their family ("PLA", "PETG"); plain filament_type values contain no space and pass through
/// unchanged.
///
/// Note this is deliberately looser than the series matching used to select standard recipe
/// entries, where PLA Basic and PLA Palette must stay apart.
pub fn color_decompose_material_family_matches(a: &str, b: &str) -> bool {
    if a.is_empty() || b.is_empty() {
        return false;
    }
    material_family(a) == material_family(b)
}

/// ColorDecomposeRecipe.cpp:314-319
pub fn color_decompose_rgb_to_hex(rgb: &ColorDecomposeRgb) -> String {
    format!("#{:02X}{:02X}{:02X}", rgb.r, rgb.g, rgb.b)
}

/// `std::sscanf(hex, "#%02x%02x%02x", &r, &g, &b)`: the three numbers if all three convert.
///
/// A `%02x` conversion skips white space, then takes at most two characters of what `strtoul` with
/// base 16 reads: an optional sign, an optional `0x`, hexadecimal digits. This follows the scanner
/// of the reference platform's C library (FreeBSD `vfscanf.c`, `CT_INT`), where a `0x` with no room
/// left for a digit reads as 0 and gives the `x` back.
pub(crate) fn sscanf_hex_rgb(hex: &str) -> Option<[u32; 3]> {
    // c_str(): the text ends at the first NUL.
    let bytes = hex.as_bytes();
    let bytes = match bytes.iter().position(|&c| c == 0) {
        Some(end) => &bytes[..end],
        None => bytes,
    };
    if bytes.first() != Some(&b'#') {
        return None;
    }
    let mut pos = 1usize;
    let mut out = [0u32; 3];
    for value in out.iter_mut() {
        while pos < bytes.len() && matches!(bytes[pos], b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r') {
            pos += 1;
        }
        // flags of the scanner
        let mut signok = true;
        let mut ndigits = true;
        let mut nzdigits = true;
        let mut pfxok = true;
        let mut havesign = false;
        let mut buf: Vec<u8> = Vec::with_capacity(2);
        let mut width = 2;
        while width > 0 && pos < bytes.len() {
            let c = bytes[pos];
            let ok = match c {
                b'0' => {
                    if nzdigits {
                        signok = false;
                        nzdigits = false;
                        ndigits = false;
                    } else {
                        signok = false;
                        pfxok = false;
                        ndigits = false;
                    }
                    true
                }
                b'1'..=b'9' | b'A'..=b'F' | b'a'..=b'f' => {
                    signok = false;
                    pfxok = false;
                    ndigits = false;
                    true
                }
                b'+' | b'-' => {
                    if signok {
                        signok = false;
                        havesign = true;
                        true
                    } else {
                        false
                    }
                }
                // x ok iff flag still set & 2nd char (or 3rd char if we have a sign).
                b'x' | b'X' => {
                    if pfxok && buf.len() == 1 + havesign as usize {
                        pfxok = false;
                        true
                    } else {
                        false
                    }
                }
                _ => false,
            };
            if !ok {
                break;
            }
            buf.push(c);
            pos += 1;
            width -= 1;
        }
        // If we had only a sign, it is no good.
        if ndigits {
            return None;
        }
        if matches!(buf.last(), Some(b'x') | Some(b'X')) {
            buf.pop();
            pos -= 1;
        }
        // strtoul(buf, NULL, 16), stored in an unsigned int
        let (negative, digits) = match buf.first() {
            Some(b'-') => (true, &buf[1..]),
            Some(b'+') => (false, &buf[1..]),
            _ => (false, &buf[..]),
        };
        let mut number: u32 = 0;
        for &c in digits {
            let digit = match c {
                b'0'..=b'9' => c - b'0',
                b'a'..=b'f' => c - b'a' + 10,
                _ => c - b'A' + 10,
            };
            number = number * 16 + digit as u32;
        }
        *value = if negative { number.wrapping_neg() } else { number };
    }
    Some(out)
}

/// ColorDecomposeRecipe.cpp:321-330
pub fn color_decompose_hex_to_rgb(hex: &str, out: &mut ColorDecomposeRgb) -> bool {
    if hex.len() < 7 || hex.as_bytes()[0] != b'#' {
        return false;
    }
    let Some([r, g, b]) = sscanf_hex_rgb(hex) else {
        return false;
    };
    *out = ColorDecomposeRgb { r: r as u8, g: g as u8, b: b as u8 };
    true
}

/// ColorDecomposeRecipe.cpp:332-425
pub fn recommend_from_physical_filaments(
    target: &ColorDecomposeRgb,
    physical_filaments: &[ColorDecomposePhysicalFilament],
    preferred_material_type: &str,
) -> ColorDecomposeRecipeResult {
    let mut candidates: Vec<ColorDecomposePhysicalFilament> = Vec::new();
    for filament in physical_filaments {
        if filament.is_mixed {
            continue;
        }
        let mut ignored = ColorDecomposeRgb::default();
        if !color_decompose_hex_to_rgb(&filament.color_hex, &mut ignored) {
            continue;
        }
        if preferred_material_type.is_empty()
            || color_decompose_material_family_matches(&filament.r#type, preferred_material_type)
        {
            candidates.push(filament.clone());
        }
    }

    // Early exit: if a material-matched candidate has the exact target color,
    // return it as 100%. Downstream rejects single-component results (no mixed
    // slot created), which is correct -- the color already exists.
    let target_hex = color_decompose_rgb_to_hex(target);
    for cand in candidates.iter() {
        let mut cand_rgb = ColorDecomposeRgb::default();
        if !color_decompose_hex_to_rgb(&cand.color_hex, &mut cand_rgb) {
            continue;
        }
        if color_decompose_rgb_to_hex(&cand_rgb) == target_hex {
            let mut exact = ColorDecomposeRecipeResult::default();
            exact.valid = true;
            exact.mode = ColorDecomposeRecipeMode::MaterialList;
            exact.matched_color_hex = cand.color_hex.clone();
            let mut comp = ColorDecomposeRecipeComponent::default();
            comp.color_hex = cand.color_hex.clone();
            comp.ratio = 100;
            comp.filament_index = cand.filament_index;
            exact.components.push(comp);
            return exact;
        }
    }

    if candidates.len() < 2 {
        candidates = physical_filaments.to_vec();
    }
    candidates.retain(|filament| {
        if filament.is_mixed {
            return false;
        }
        let mut ignored = ColorDecomposeRgb::default();
        color_decompose_hex_to_rgb(&filament.color_hex, &mut ignored)
    });

    // Identical colors are one pigment: keep the first slot so recipes never repeat a color.
    let mut seen_hexes: Vec<String> = Vec::new();
    candidates.retain(|filament| {
        let mut rgb = ColorDecomposeRgb::default();
        color_decompose_hex_to_rgb(&filament.color_hex, &mut rgb);
        let hex = color_decompose_rgb_to_hex(&rgb);
        if seen_hexes.contains(&hex) {
            return false;
        }
        seen_hexes.push(hex);
        true
    });

    const K_MAX_CANDIDATES: usize = 8;
    if candidates.len() > K_MAX_CANDIDATES {
        let target_lab = rgb_to_lab(target);
        // std::sort is not stable: the order of filaments at the same distance is that of libc++.
        // The filaments are sorted through their positions, which the algorithm moves as it would
        // move the filaments.
        let mut order: Vec<usize> = (0..candidates.len()).collect();
        libcxx_sort(&mut order, |&a, &b| {
            let mut rgb_a = ColorDecomposeRgb::default();
            let mut rgb_b = ColorDecomposeRgb::default();
            color_decompose_hex_to_rgb(&candidates[a].color_hex, &mut rgb_a);
            color_decompose_hex_to_rgb(&candidates[b].color_hex, &mut rgb_b);
            delta_e76(&target_lab, &rgb_to_lab(&rgb_a)) < delta_e76(&target_lab, &rgb_to_lab(&rgb_b))
        });
        candidates = order.iter().map(|&i| candidates[i].clone()).collect();
        candidates.truncate(K_MAX_CANDIDATES);
    }

    let mut best = ColorDecomposeRecipeResult::default();
    let mut best_score = f64::MAX;

    for i in 0..candidates.len() {
        for j in i + 1..candidates.len() {
            let hexes: Vec<String> = vec![candidates[i].color_hex.clone(), candidates[j].color_hex.clone()];
            let indices: Vec<u32> = vec![candidates[i].filament_index, candidates[j].filament_index];
            for ratios in ratio_grid(2).iter() {
                evaluate_candidate(
                    target,
                    &hexes,
                    ratios,
                    &indices,
                    ColorDecomposeRecipeMode::MaterialList,
                    &mut best_score,
                    &mut best,
                );
            }

            for k in j + 1..candidates.len() {
                let hexes3: Vec<String> = vec![
                    candidates[i].color_hex.clone(),
                    candidates[j].color_hex.clone(),
                    candidates[k].color_hex.clone(),
                ];
                let indices3: Vec<u32> =
                    vec![candidates[i].filament_index, candidates[j].filament_index, candidates[k].filament_index];
                for ratios in ratio_grid(3).iter() {
                    evaluate_candidate(
                        target,
                        &hexes3,
                        ratios,
                        &indices3,
                        ColorDecomposeRecipeMode::MaterialList,
                        &mut best_score,
                        &mut best,
                    );
                }
            }
        }
    }

    best
}

/// ColorDecomposeRecipe.cpp:427-469
pub fn lookup_standard_recipe(
    target: &ColorDecomposeRgb,
    mode: ColorDecomposeRecipeMode,
    preferred_material_type: &str,
) -> ColorDecomposeRecipeResult {
    let target_lab = rgb_to_lab(target);
    let mut best = ColorDecomposeRecipeResult::default();
    let mut best_score = f64::MAX;

    // ColorDecomposeRecipe.cpp:436-463: the lambda `consider`, which captures the locals above.
    let mut consider = |require_material_match: bool, best: &mut ColorDecomposeRecipeResult| {
        for entry in standard_entries() {
            if entry.mode != mode {
                continue;
            }
            if require_material_match && !material_matches(&entry.material, preferred_material_type) {
                continue;
            }
            if !require_material_match
                && !preferred_material_type.is_empty()
                && material_matches(&entry.material, preferred_material_type)
            {
                continue;
            }

            let score = delta_e76(&target_lab, &entry.measured_lab);
            if score >= best_score {
                continue;
            }

            best_score = score;
            best.valid = true;
            best.mode = mode;
            best.matched_color_hex = entry.measured_hex.clone();
            best.components.clear();
            for i in 0..entry.component_hexes.len() {
                let mut comp = ColorDecomposeRecipeComponent::default();
                comp.color_hex = entry.component_hexes[i].clone();
                comp.base_color = if i < entry.component_keys.len() { entry.component_keys[i].clone() } else { String::new() };
                comp.ratio = entry.ratios[i];
                comp.filament_index = 0;
                best.components.push(comp);
            }
        }
    };

    consider(true, &mut best);
    if !best.valid {
        consider(false, &mut best);
    }
    best
}

/// sift_down.h `__sift_down` of libc++, on the heap `s[..len]`.
fn libcxx_sift_down<T: Copy, F: FnMut(&T, &T) -> bool>(s: &mut [T], c: &mut F, len: usize, mut start: usize) {
    // left-child of start is at 2 * start + 1, right-child at 2 * start + 2
    let mut child = start;
    if len < 2 || (len - 2) / 2 < child {
        return;
    }
    child = 2 * child + 1;
    if child + 1 < len && c(&s[child], &s[child + 1]) {
        // right-child exists and is greater than left-child
        child += 1;
    }
    // check if we are in heap-order
    if c(&s[child], &s[start]) {
        // we are, start is larger than its largest child
        return;
    }
    let top = s[start];
    loop {
        // we are not in heap-order, swap the parent with its largest child
        s[start] = s[child];
        start = child;
        if (len - 2) / 2 < child {
            break;
        }
        // recompute the child based off of the updated parent
        child = 2 * child + 1;
        if child + 1 < len && c(&s[child], &s[child + 1]) {
            child += 1;
        }
        // check if we are in heap-order
        if c(&s[child], &top) {
            break;
        }
    }
    s[start] = top;
}

/// sift_down.h `__floyd_sift_down` of libc++, on the heap `s[..len]`; returns the hole.
fn libcxx_floyd_sift_down<T: Copy, F: FnMut(&T, &T) -> bool>(s: &mut [T], c: &mut F, len: usize) -> usize {
    let mut hole = 0usize;
    let mut child = 0usize;
    loop {
        child = 2 * child + 1;
        if child + 1 < len && c(&s[child], &s[child + 1]) {
            // right-child exists and is greater than left-child
            child += 1;
        }
        // swap hole with its largest child
        s[hole] = s[child];
        hole = child;
        // if hole is now a leaf, we're done
        if child > (len - 2) / 2 {
            return hole;
        }
    }
}

/// push_heap.h `__sift_up` of libc++, on `s[..last]` with `len == last`.
fn libcxx_sift_up<T: Copy, F: FnMut(&T, &T) -> bool>(s: &mut [T], mut last: usize, c: &mut F, mut len: usize) {
    if len > 1 {
        len = (len - 2) / 2;
        let mut ptr = len;
        last -= 1;
        if c(&s[ptr], &s[last]) {
            let t = s[last];
            loop {
                s[last] = s[ptr];
                last = ptr;
                if len == 0 {
                    break;
                }
                len = (len - 1) / 2;
                ptr = len;
                if !c(&s[ptr], &t) {
                    break;
                }
            }
            s[last] = t;
        }
    }
}

/// pop_heap.h `__pop_heap` of libc++, on the heap `s[..len]`.
fn libcxx_pop_heap<T: Copy, F: FnMut(&T, &T) -> bool>(s: &mut [T], c: &mut F, len: usize) {
    if len > 1 {
        let top = s[0]; // create a hole at first
        let mut hole = libcxx_floyd_sift_down(s, c, len);
        let last = len - 1;
        if hole == last {
            s[hole] = top;
        } else {
            s[hole] = s[last];
            hole += 1;
            s[last] = top;
            libcxx_sift_up(s, hole, c, hole);
        }
    }
}

/// `std::partial_sort(first, first + middle, last, comp)` of libc++ (partial_sort.h
/// `__partial_sort_impl`, with make_heap.h and sort_heap.h): which of several equal elements end
/// up in front is the algorithm's choice, so the algorithm is the one of the reference build.
fn libcxx_partial_sort<T: Copy, F: FnMut(&T, &T) -> bool>(s: &mut [T], middle: usize, mut comp: F) {
    if middle == 0 {
        return;
    }
    let c = &mut comp;
    // __make_heap(first, middle)
    let n = middle;
    if n > 1 {
        // start from the first parent, there is no need to consider children
        let mut start = (n - 2) / 2;
        loop {
            libcxx_sift_down(s, c, n, start);
            if start == 0 {
                break;
            }
            start -= 1;
        }
    }

    let len = middle;
    for i in middle..s.len() {
        if c(&s[i], &s[0]) {
            s.swap(i, 0);
            libcxx_sift_down(s, c, len, 0);
        }
    }
    // __sort_heap(first, middle)
    let mut n = middle;
    while n > 1 {
        libcxx_pop_heap(s, c, n);
        n -= 1;
    }
}

/// ColorDecomposeRecipe.cpp:471-569 — look up the measured blend color for (component_hexes,
/// ratios) in the standard color recipe table. Exact grid hits return measured_rgb; off-grid ratios
/// interpolate in Lab (2-color may extrapolate a short way past the measured hull, e.g. 14/86 from
/// the 15/85 and 20/80 anchors). Returns empty when the component set is not in the table.
pub fn lookup_measured_blend_color(component_hexes: &[String], ratios: &[i32]) -> String {
    if component_hexes.len() < 2 || component_hexes.len() != ratios.len() {
        return String::new();
    }

    // Stage 1: canonicalize input by sorting (hex, ratio) pairs so matching
    // is independent of the caller's component order.
    let n = component_hexes.len();
    let mut in_key = String::new();
    let mut in_ratios: Vec<i32> = Vec::new();
    if !canonicalize_blend_components(component_hexes, ratios, &mut in_key, &mut in_ratios) {
        return String::new();
    }

    // Normalize ratios to sum=100 (callers may pass arbitrary weights,
    // e.g. MixedFilamentDialog uses ratio*10000).
    {
        let mut sum: i32 = 0;
        for &r in in_ratios.iter() {
            sum = sum.wrapping_add(r);
        }
        if sum > 0 && sum != 100 {
            let mut new_sum: i32 = 0;
            for i in 0..in_ratios.len() {
                in_ratios[i] = (in_ratios[i] as f64 * 100.0 / sum as f64).round() as i64 as i32;
                new_sum = new_sum.wrapping_add(in_ratios[i]);
            }
            if new_sum != 100 {
                // std::max_element: the first of the largest
                let mut it = 0usize;
                for i in 1..in_ratios.len() {
                    if in_ratios[it] < in_ratios[i] {
                        it = i;
                    }
                }
                in_ratios[it] = in_ratios[it].wrapping_add(100 - new_sum);
            }
        }
    }

    // Stage 2: anchors sharing the component set; try exact ratio match.
    let index = measured_blend_index();
    let Some(anchors) = index.get(&in_key) else {
        return String::new();
    };

    for a in anchors.iter() {
        if a.ratios == in_ratios {
            return a.hex.clone();
        }
    }

    if anchors.len() < 2 {
        return String::new();
    }

    // Stage 3: interpolation in Lab space.
    if n == 2 {
        // 1D linear interpolation along ratio[0]. t may fall outside [0, 1]:
        // 14/86 uses the 15–20 segment with t = -0.2, i.e. the 20→15 Lab
        // delta applied once more beyond 15, instead of dropping to the
        // polynomial model (which caused a preview jump at the hull edge).
        //
        // std::sort is not stable, and two tables can measure the same pair at the same ratio, so
        // the order of equal ratios is that of libc++.
        let mut by_ratio: Vec<usize> = (0..anchors.len()).collect();
        libcxx_sort(&mut by_ratio, |&a, &b| anchors[a].ratios[0] < anchors[b].ratios[0]);
        let x = in_ratios[0] as f64;
        let mut lo: usize = 0;
        while lo + 2 < by_ratio.len() && anchors[by_ratio[lo + 1]].ratios[0] as f64 <= x {
            lo += 1;
        }
        let a0 = &anchors[by_ratio[lo]];
        let a1 = &anchors[by_ratio[lo + 1]];
        let span = (a1.ratios[0] - a0.ratios[0]) as f64;
        let t = if span > 0.0 { (x - a0.ratios[0] as f64) / span } else { 0.0 };
        return lab_to_srgb_hex(&LabColor {
            l: t.mul_add(a1.lab.l - a0.lab.l, a0.lab.l),
            a: t.mul_add(a1.lab.a - a0.lab.a, a0.lab.a),
            b: t.mul_add(a1.lab.b - a0.lab.b, a0.lab.b),
        });
    }

    // 3+ color: IDW (p=2) with 3 nearest anchors in the (ratio[0], ratio[1]) plane.
    // Queries slightly outside the measured hull (e.g. a 10% component) still
    // use nearby measured anchors instead of falling back to the polynomial.
    let ra = in_ratios[0] as f64;
    let rb = in_ratios[1] as f64;
    let mut dists: Vec<(f64, usize)> = Vec::with_capacity(anchors.len());
    for (anchor_id, a) in anchors.iter().enumerate() {
        let da = ra - a.ratios[0] as f64;
        let db = rb - a.ratios[1] as f64;
        let d = (da * da + db * db).sqrt();
        if d == 0.0 {
            return a.hex.clone();
        }
        dists.push((d, anchor_id));
    }
    let k = 3usize.min(dists.len());
    libcxx_partial_sort(&mut dists, k, |a, b| a.0 < b.0);
    let mut num_l = 0.0_f64;
    let mut num_a = 0.0_f64;
    let mut num_b = 0.0_f64;
    let mut den = 0.0_f64;
    for j in 0..k {
        let w = 1.0 / (dists[j].0 * dists[j].0);
        let anchor = &anchors[dists[j].1];
        num_l = w.mul_add(anchor.lab.l, num_l);
        num_a = w.mul_add(anchor.lab.a, num_a);
        num_b = w.mul_add(anchor.lab.b, num_b);
        den += w;
    }
    lab_to_srgb_hex(&LabColor { l: num_l / den, a: num_a / den, b: num_b / den })
}

#[cfg(test)]
mod tests {
    use super::*;

    // ColorDecomposeRecipe.cpp:121-136: 13 two-color ratios, and the three-color ratios in steps of
    // 5 with every component at 20 or more.
    #[test]
    fn ratio_grid_sizes() {
        let two = ratio_grid(2);
        assert_eq!(two.len(), 13);
        assert_eq!(two[0], vec![20, 80]);
        assert_eq!(two[12], vec![80, 20]);
        let three = ratio_grid(3);
        assert_eq!(three.len(), 45);
        assert!(three.iter().all(|r| r.iter().sum::<i32>() == 100 && r.iter().all(|&v| v >= 20)));
        assert!(ratio_grid(4).is_empty());
    }

    // ColorDecomposeRecipe.cpp:108-119, 138-145
    #[test]
    fn materials_and_modes() {
        assert!(material_matches("PLA", "PLA Basic"));
        assert!(material_matches("PLA Basic", "PLA"));
        assert!(!material_matches("PLA Basic", "PLA Palette"));
        assert!(!material_matches("", ""));
        assert_eq!(material_family("PLA Basic"), "PLA");
        assert_eq!(material_family("PETG"), "PETG");
        assert_eq!(parse_mode("RGBY"), ColorDecomposeRecipeMode::RYBW);
        assert_eq!(parse_mode("CMYKW"), ColorDecomposeRecipeMode::CMYKW);
        assert_eq!(parse_mode("anything"), ColorDecomposeRecipeMode::CMYW);
    }
}
