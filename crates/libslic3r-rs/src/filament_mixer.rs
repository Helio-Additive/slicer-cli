//! Faithful 1:1 port of FilamentMixer.cpp / FilamentMixer.hpp
//!
//! Public API mirrors the C++ free functions in `namespace Slic3r`.

// FilamentMixer.cpp:9
use crate::filament_mixer_model;

// FilamentMixer.cpp:12 — anonymous namespace (file-local helpers)

// FilamentMixer.cpp:14
#[inline]
fn clamp01(x: f32) -> f32 {
    x.max(0.0f32).min(1.0f32)
}

// FilamentMixer.cpp:19
#[inline]
fn srgb_to_linear(x: f32) -> f32 {
    if x >= 0.04045f32 {
        ((x + 0.055f32) / 1.055f32).powf(2.4f32)
    } else {
        x / 12.92f32
    }
}

// FilamentMixer.cpp:24
#[inline]
fn linear_to_srgb(x: f32) -> f32 {
    if x >= 0.0031308f32 {
        1.055f32 * x.powf(1.0f32 / 2.4f32) - 0.055f32
    } else {
        12.92f32 * x
    }
}

// FilamentMixer.cpp:29
#[inline]
fn to_u8(x: f32) -> u8 {
    let clamped = clamp01(x);
    (clamped * 255.0f32 + 0.5f32) as u8
}

// FilamentMixer.cpp:35
#[inline]
fn to_f01(x: u8) -> f32 {
    (x as f32) / 255.0f32
}

// FilamentMixer.cpp:42
pub fn filament_mixer_lerp(
    r1: u8,
    g1: u8,
    b1: u8,
    r2: u8,
    g2: u8,
    b2: u8,
    t: f32,
    out_r: &mut u8,
    out_g: &mut u8,
    out_b: &mut u8,
) {
    filament_mixer_model::lerp(r1, g1, b1, r2, g2, b2, t, out_r, out_g, out_b);
}

// FilamentMixer.cpp:50
pub fn filament_mixer_lerp_float(
    r1: f32,
    g1: f32,
    b1: f32,
    r2: f32,
    g2: f32,
    b2: f32,
    t: f32,
    out_r: &mut f32,
    out_g: &mut f32,
    out_b: &mut f32,
) {
    let mut ur: u8 = 0;
    let mut ug: u8 = 0;
    let mut ub: u8 = 0;
    filament_mixer_lerp(
        to_u8(r1),
        to_u8(g1),
        to_u8(b1),
        to_u8(r2),
        to_u8(g2),
        to_u8(b2),
        t,
        &mut ur,
        &mut ug,
        &mut ub,
    );
    *out_r = to_f01(ur);
    *out_g = to_f01(ug);
    *out_b = to_f01(ub);
}

// FilamentMixer.cpp:64
pub fn filament_mixer_lerp_linear_float(
    r1: f32,
    g1: f32,
    b1: f32,
    r2: f32,
    g2: f32,
    b2: f32,
    t: f32,
    out_r: &mut f32,
    out_g: &mut f32,
    out_b: &mut f32,
) {
    let sr1 = linear_to_srgb(clamp01(r1));
    let sg1 = linear_to_srgb(clamp01(g1));
    let sb1 = linear_to_srgb(clamp01(b1));
    let sr2 = linear_to_srgb(clamp01(r2));
    let sg2 = linear_to_srgb(clamp01(g2));
    let sb2 = linear_to_srgb(clamp01(b2));

    let mut out_sr: f32 = 0.0f32;
    let mut out_sg: f32 = 0.0f32;
    let mut out_sb: f32 = 0.0f32;
    filament_mixer_lerp_float(
        sr1, sg1, sb1, sr2, sg2, sb2, t, &mut out_sr, &mut out_sg, &mut out_sb,
    );

    *out_r = srgb_to_linear(clamp01(out_sr));
    *out_g = srgb_to_linear(clamp01(out_sg));
    *out_b = srgb_to_linear(clamp01(out_sb));
}

// FilamentMixer.cpp:84
fn parse_hex(hex: &str, r: &mut u8, g: &mut u8, b: &mut u8) -> bool {
    if hex.len() < 7 || hex.as_bytes()[0] != b'#' {
        return false;
    }
    // Mirror std::sscanf("#%02x%02x%02x"): parse exactly two hex digits each
    // for r, g, b directly after the '#'.
    let bytes = hex.as_bytes();
    let hexval = |c: u8| -> Option<u32> {
        match c {
            b'0'..=b'9' => Some((c - b'0') as u32),
            b'a'..=b'f' => Some((c - b'a' + 10) as u32),
            b'A'..=b'F' => Some((c - b'A' + 10) as u32),
            _ => None,
        }
    };
    let mut vals = [0u32; 3];
    let mut pos = 1usize; // skip '#'
    for v in vals.iter_mut() {
        // %02x reads up to two hex digits; sscanf requires at least one.
        let d0 = match bytes.get(pos).copied().and_then(hexval) {
            Some(d) => d,
            None => return false,
        };
        pos += 1;
        match bytes.get(pos).copied().and_then(hexval) {
            Some(d1) => {
                *v = d0 * 16 + d1;
                pos += 1;
            }
            None => {
                *v = d0;
            }
        }
    }
    let rv = vals[0];
    let gv = vals[1];
    let bv = vals[2];
    *r = rv as u8;
    *g = gv as u8;
    *b = bv as u8;
    true
}

// FilamentMixer.cpp:93
pub fn blend_color(hex_a: &str, hex_b: &str, ratio_b: f32) -> String {
    let mut r1: u8 = 128;
    let mut g1: u8 = 128;
    let mut b1: u8 = 128;
    let mut r2: u8 = 128;
    let mut g2: u8 = 128;
    let mut b2: u8 = 128;
    parse_hex(hex_a, &mut r1, &mut g1, &mut b1);
    parse_hex(hex_b, &mut r2, &mut g2, &mut b2);

    let mut mr: u8 = 0;
    let mut mg: u8 = 0;
    let mut mb: u8 = 0;
    filament_mixer_lerp(r1, g1, b1, r2, g2, b2, ratio_b, &mut mr, &mut mg, &mut mb);

    format!("#{:02X}{:02X}{:02X}", mr, mg, mb)
}

// Mirror std::getline(ss, token, ',') tokenization: split on ',', producing
// a trailing empty token only when the string ends with ',' (and no token at
// all for an empty input).
fn getline_split(s: &str) -> Vec<&str> {
    if s.is_empty() {
        return Vec::new();
    }
    s.split(',').collect()
}

// FilamentMixer.cpp:108
pub fn parse_mixed_components(str_: &str) -> Vec<u32> {
    let mut components: Vec<u32> = Vec::new();
    if str_.is_empty() {
        return components;
    }
    for token in getline_split(str_) {
        // try { std::stoi(token) } catch (...) {}
        // std::stoi parses a leading integer (with optional sign/whitespace);
        // it throws if no conversion can be performed.
        if let Some(val) = stoi(token) {
            if val >= 0 {
                components.push(val as u32);
            }
        }
    }
    components
}

// FilamentMixer.cpp:125
pub fn parse_mixed_ratios(str_: &str, n_components: usize) -> Vec<f64> {
    let mut ratios: Vec<f64> = Vec::new();
    if !str_.is_empty() {
        for token in getline_split(str_) {
            if let Some(val) = stod(token) {
                if val > 0.0 {
                    ratios.push(val);
                }
            }
        }
    }

    if ratios.len() != n_components || n_components == 0 {
        let fill = if n_components > 0 {
            1.0 / n_components as f64
        } else {
            0.0
        };
        ratios = vec![fill; n_components];
        return ratios;
    }

    let sum: f64 = ratios.iter().fold(0.0, |acc, &x| acc + x);
    if sum > 0.0 && (sum - 1.0).abs() > 1e-6 {
        for r in ratios.iter_mut() {
            *r /= sum;
        }
    }
    ratios
}

// FilamentMixer.cpp:153
pub fn has_any_mixed_filament(is_mixed: &[u8]) -> bool {
    for &v in is_mixed {
        if v != 0 {
            return true;
        }
    }
    false
}

// FilamentMixer.cpp:160
pub fn check_mixed_filament_integrity(
    is_mixed: &[u8],
    comp_strs: &[String],
    num_physical: usize,
) -> Vec<usize> {
    let mut broken: Vec<usize> = Vec::new();
    for i in 0..is_mixed.len() {
        if is_mixed[i] == 0 {
            continue;
        }
        if i >= comp_strs.len() || comp_strs[i].is_empty() {
            broken.push(i);
            continue;
        }
        let comps = parse_mixed_components(&comp_strs[i]);
        if comps.len() < 2 {
            broken.push(i);
            continue;
        }
        for &c in &comps {
            if c < 1 || c as usize > num_physical {
                broken.push(i);
                break;
            }
        }
    }
    broken
}

// FilamentMixer.cpp:187
pub fn expand_mixed_filaments(
    extruders_0based: &[u32],
    is_mixed: &[u8],
    comp_strs: &[String],
) -> Vec<u32> {
    let mut result: Vec<u32> = Vec::new();
    for &ext in extruders_0based {
        if (ext as usize) < is_mixed.len()
            && is_mixed[ext as usize] != 0
            && (ext as usize) < comp_strs.len()
        {
            let comps = parse_mixed_components(&comp_strs[ext as usize]);
            for &c in &comps {
                if c >= 1 {
                    result.push(c - 1);
                }
            }
        } else {
            result.push(ext);
        }
    }
    result.sort();
    result.dedup();
    result
}

// FilamentMixer.cpp:207
pub fn remap_mixed_components_on_delete(
    is_mixed: &[u8],
    comp_strs: &mut [String],
    del_1based: u32,
) {
    for i in 0..is_mixed.len() {
        if is_mixed[i] == 0 {
            continue;
        }
        if i >= comp_strs.len() || comp_strs[i].is_empty() {
            continue;
        }

        let comps = parse_mixed_components(&comp_strs[i]);
        let mut ss = String::new();
        for j in 0..comps.len() {
            if j > 0 {
                ss.push(',');
            }
            if comps[j] == del_1based {
                ss.push_str(&0.to_string());
            } else if comps[j] > del_1based {
                ss.push_str(&(comps[j] - 1).to_string());
            } else {
                ss.push_str(&comps[j].to_string());
            }
        }
        comp_strs[i] = ss;
    }
}

// FilamentMixer.cpp:231
pub fn check_mixed_filament_type_consistency(
    is_mixed: &[u8],
    comp_strs: &[String],
    filament_types: &[String],
) -> Vec<usize> {
    let mut result: Vec<usize> = Vec::new();
    for i in 0..is_mixed.len() {
        if is_mixed[i] == 0 {
            continue;
        }
        if i >= comp_strs.len() || comp_strs[i].is_empty() {
            continue;
        }
        let comps = parse_mixed_components(&comp_strs[i]);
        if comps.len() < 2 {
            continue;
        }

        let mut ref_type = String::new();
        let mut mismatch = false;
        for &c in &comps {
            if c == 0 {
                continue; // sentinel for deleted component
            }
            let idx = (c as usize) - 1; // 1-based -> 0-based
            if idx >= filament_types.len() {
                continue;
            }
            if ref_type.is_empty() {
                ref_type = filament_types[idx].clone();
            } else if filament_types[idx] != ref_type {
                mismatch = true;
                break;
            }
        }
        if mismatch {
            result.push(i);
        }
    }
    result
}

// std::stoi — parse a leading integer from `s`, skipping leading whitespace,
// honoring an optional sign, and stopping at the first non-digit. Returns None
// when no conversion can be performed (mirrors the C++ exception path that the
// callers swallow).
fn stoi(s: &str) -> Option<i32> {
    let bytes = s.as_bytes();
    let mut i = 0usize;
    while i < bytes.len() && (bytes[i] as char).is_whitespace() {
        i += 1;
    }
    let start = i;
    if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
        i += 1;
    }
    let digits_start = i;
    while i < bytes.len() && bytes[i].is_ascii_digit() {
        i += 1;
    }
    if i == digits_start {
        return None; // no digits -> std::invalid_argument
    }
    s[start..i].parse::<i32>().ok()
}

// std::stod — parse a leading floating point number from `s`. Returns None when
// no conversion can be performed (mirrors the swallowed C++ exception).
fn stod(s: &str) -> Option<f64> {
    let trimmed = s.trim_start();
    if trimmed.is_empty() {
        return None;
    }
    // Find the longest prefix that parses as a double the way strtod does:
    // optional sign, digits, decimal point, exponent. Walk a candidate window
    // and shrink until it parses.
    let bytes = trimmed.as_bytes();
    let mut end = bytes.len();
    while end > 0 {
        if let Ok(v) = trimmed[..end].trim_end().parse::<f64>() {
            return Some(v);
        }
        end -= 1;
    }
    None
}
