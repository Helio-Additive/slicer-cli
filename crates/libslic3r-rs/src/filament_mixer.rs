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
    // std::sscanf(hex.c_str(), "#%02x%02x%02x", &rv, &gv, &bv) != 3: the scanner is the one of
    // ColorDecomposeRecipe.cpp:326, which reads a color the same way.
    let Some([rv, gv, bv]) = crate::color_decompose_recipe::sscanf_hex_rgb(hex) else {
        return false;
    };
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

// FilamentMixer.cpp:116-162
pub fn blend_color_multi(hex_colors: &[String], weights: &[i32]) -> String {
    if hex_colors.len() >= 2 && hex_colors.len() == weights.len() {
        let measured = crate::color_decompose_recipe::lookup_measured_blend_color(hex_colors, weights);
        if !measured.is_empty() {
            return measured;
        }
    }

    if hex_colors.is_empty() {
        return "#000000".to_string();
    }
    if hex_colors.len() == 1 {
        let mut cr: u8 = 128;
        let mut cg: u8 = 128;
        let mut cb: u8 = 128;
        parse_hex(&hex_colors[0], &mut cr, &mut cg, &mut cb);
        return format!("#{:02X}{:02X}{:02X}", cr, cg, cb);
    }

    debug_assert!(hex_colors.len() == weights.len());

    let mut r: u8 = 128;
    let mut g: u8 = 128;
    let mut b: u8 = 128;
    let mut accumulated: i32 = 0;

    let mut i = 0usize;
    while i < hex_colors.len() && i < weights.len() {
        if weights[i] <= 0 {
            i += 1;
            continue;
        }
        let mut cr: u8 = 128;
        let mut cg: u8 = 128;
        let mut cb: u8 = 128;
        parse_hex(&hex_colors[i], &mut cr, &mut cg, &mut cb);
        if accumulated == 0 {
            r = cr;
            g = cg;
            b = cb;
            accumulated = weights[i];
        } else {
            let new_total = accumulated.wrapping_add(weights[i]);
            let t = weights[i] as f32 / new_total as f32;
            let (r1, g1, b1) = (r, g, b);
            filament_mixer_lerp(r1, g1, b1, cr, cg, cb, t, &mut r, &mut g, &mut b);
            accumulated = new_total;
        }
        i += 1;
    }

    if accumulated == 0 {
        return "#000000".to_string();
    }

    format!("#{:02X}{:02X}{:02X}", r, g, b)
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

// =============================================================================
// FilamentMixer.hpp:16-55 / FilamentMixer.cpp:182-395, 562-581: the gradient curve of a mixed
// slot and the expansion of mixed slots in unprintable sets (read by GCode/ToolOrdering.cpp).
// =============================================================================

/// FilamentMixer.hpp:19-24 `struct GradientAnchor`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GradientAnchor {
    pub x: f64,
    pub y: f64,
    pub m_in: f64,
    pub m_out: f64,
}

impl Default for GradientAnchor {
    fn default() -> Self {
        GradientAnchor { x: 0.0, y: 0.0, m_in: f64::NAN, m_out: f64::NAN }
    }
}

/// FilamentMixer.hpp:28-31 `struct GradientCurve`
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GradientCurve {
    pub points: Vec<GradientAnchor>,
}

impl GradientCurve {
    /// FilamentMixer.hpp:30
    pub fn empty(&self) -> bool {
        self.points.is_empty()
    }
}

/// FilamentMixer.hpp:37
pub const K_GRADIENT_MIN_RATIO: f64 = 0.1;
/// FilamentMixer.hpp:38
pub const K_GRADIENT_MAX_RATIO: f64 = 0.9;

/// FilamentMixer.cpp:184-200 `parse_tangent_token`
fn parse_tangent_token(tok: &str) -> f64 {
    if tok.is_empty() {
        return f64::NAN;
    }
    let lower: String = tok.bytes().map(|c| c.to_ascii_lowercase() as char).collect();
    if lower == "nan" {
        return f64::NAN;
    }
    match stod(tok) {
        Some(v) => {
            if !v.is_finite() {
                return f64::NAN;
            }
            v
        }
        None => f64::NAN,
    }
}

/// FilamentMixer.cpp:205-218 `split_commas`
fn split_commas(seg: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut start = 0usize;
    loop {
        match seg[start..].find(',') {
            None => {
                out.push(seg[start..].to_string());
                return out;
            }
            Some(off) => {
                let comma = start + off;
                out.push(seg[start..comma].to_string());
                start = comma + 1;
            }
        }
    }
}

/// FilamentMixer.cpp:225-260 `compute_pchip_default_tangents`
pub fn compute_pchip_default_tangents(pts: &[GradientAnchor]) -> Vec<f64> {
    let n = pts.len();
    let mut m = vec![0.0f64; n];
    if n < 2 {
        return m;
    }

    let mut d = vec![0.0f64; n - 1];
    for i in 0..n - 1 {
        let h = (pts[i + 1].x - pts[i].x).max(1e-12);
        d[i] = (pts[i + 1].y - pts[i].y) / h;
    }

    m[0] = d[0];
    m[n - 1] = d[n - 2];
    for i in 1..n - 1 {
        m[i] = 0.5 * (d[i - 1] + d[i]);
    }

    for i in 0..n - 1 {
        if d[i] == 0.0 {
            m[i] = 0.0;
            m[i + 1] = 0.0;
            continue;
        }
        let a = m[i] / d[i];
        let b = m[i + 1] / d[i];
        // `a * a + b * b`: clang contracts it into fma(a, a, b * b) (-ffp-contract=on).
        let s = a.mul_add(a, b * b);
        if s > 9.0 {
            let tau = 3.0 / s.sqrt();
            m[i] = tau * a * d[i];
            m[i + 1] = tau * b * d[i];
        }
    }
    m
}

/// FilamentMixer.cpp:262-315 `parse_gradient_curve`
pub fn parse_gradient_curve(s: &str) -> GradientCurve {
    let mut curve = GradientCurve::default();
    if s.is_empty() {
        return curve;
    }

    // `while (std::getline(ss, segment, '|'))`: a trailing empty piece is not produced.
    let mut segments: Vec<&str> = s.split('|').collect();
    if segments.last().map_or(false, |l| l.is_empty()) {
        segments.pop();
    }
    for segment in segments {
        if segment.is_empty() {
            continue;
        }
        let fields = split_commas(segment);
        if fields.len() != 2 && fields.len() != 4 {
            log::warn!(
                "ignoring malformed segment \"{}\" (expected 2 or 4 comma-separated fields, got {})",
                segment,
                fields.len()
            );
            continue;
        }
        // try { std::stod(fields[0]); std::stod(fields[1]); ... } catch (...) { continue; }
        let (Some(x), Some(y)) = (stod(&fields[0]), stod(&fields[1])) else {
            log::warn!("ignoring unparseable segment \"{}\"", segment);
            continue;
        };
        let x = 0.0f64.max(1.0f64.min(x));
        let y = K_GRADIENT_MIN_RATIO.max(K_GRADIENT_MAX_RATIO.min(y));
        let mut a = GradientAnchor::default();
        a.x = x;
        a.y = y;
        if fields.len() == 4 {
            a.m_in = parse_tangent_token(&fields[2]);
            a.m_out = parse_tangent_token(&fields[3]);
        }
        curve.points.push(a);
    }

    if curve.points.len() < 2 {
        if !curve.points.is_empty() {
            log::warn!("only {} valid point(s), need at least 2; discarding", curve.points.len());
        }
        curve.points.clear();
        return curve;
    }

    // std::sort by x (libc++ introsort; anchors of equal x keep libc++'s order).
    crate::clipper::libcxx_sort(&mut curve.points, |a, b| a.x < b.x);
    curve
}

/// FilamentMixer.cpp:350-395 `sample_gradient_curve`
pub fn sample_gradient_curve(c: &GradientCurve, t: f64) -> f64 {
    let pts = &c.points;
    if pts.len() < 2 {
        return 0.5;
    }
    if t <= pts[0].x {
        return pts[0].y;
    }
    if t >= pts[pts.len() - 1].x {
        return pts[pts.len() - 1].y;
    }

    let m_def = compute_pchip_default_tangents(pts);
    let n = pts.len();

    for i in 1..n {
        let x0 = pts[i - 1].x;
        let x1 = pts[i].x;
        if t > x1 {
            continue;
        }

        let y0 = pts[i - 1].y;
        let y1 = pts[i].y;
        let h = (x1 - x0).max(1e-12);
        let m_left = if pts[i - 1].m_out.is_finite() { pts[i - 1].m_out } else { m_def[i - 1] };
        let m_right = if pts[i].m_in.is_finite() { pts[i].m_in } else { m_def[i] };

        let u = (t - x0) / h;
        let u2 = u * u;
        let u3 = u2 * u;
        // The fmas clang contracts these sums of products into (-ffp-contract=on): the first product
        // of `a * b + c * d` / `a * b - c * d` is fused, a product added to a sum is fused.
        let h00 = 2.0f64.mul_add(u3, -(3.0 * u2)) + 1.0;
        let h10 = (-2.0f64).mul_add(u2, u3) + u;
        let h01 = (-2.0f64).mul_add(u3, 3.0 * u2);
        let h11 = u3 - u2;
        let s0 = h00.mul_add(y0, h10 * h * m_left);
        let s1 = h01.mul_add(y1, s0);
        let mut y = (h11 * h).mul_add(m_right, s1);
        if y < K_GRADIENT_MIN_RATIO {
            y = K_GRADIENT_MIN_RATIO;
        }
        if y > K_GRADIENT_MAX_RATIO {
            y = K_GRADIENT_MAX_RATIO;
        }
        return y;
    }
    pts[pts.len() - 1].y
}

/// FilamentMixer.cpp:562-581 `expand_mixed_slots_in_unprintables`
pub fn expand_mixed_slots_in_unprintables(
    unprintables: &mut Vec<std::collections::BTreeSet<i32>>,
    is_mixed: &[u8],
    comp_strs: &[String],
) {
    for unprintable_set in unprintables.iter_mut() {
        let mut expanded = std::collections::BTreeSet::new();
        for &fid in unprintable_set.iter() {
            if fid >= 0 && (fid as usize) < is_mixed.len() && is_mixed[fid as usize] != 0 && (fid as usize) < comp_strs.len() {
                let comps = parse_mixed_components(&comp_strs[fid as usize]);
                for c in comps {
                    if c >= 1 {
                        expanded.insert((c - 1) as i32);
                    }
                }
            } else {
                expanded.insert(fid);
            }
        }
        *unprintable_set = expanded;
    }
}

// FilamentMixer.cpp:535-842: the strict mixed-filament checks that `validate(const FullPrintConfig&, bool)`
// (PrintConfig.cpp:10641-10916) and `DynamicPrintConfig::validate` (PrintConfig.cpp:8969-9007) call, moved here
// from print_config.rs (`config_def::filament_mixer_validate` re-exports them).
use std::collections::{BTreeMap, BTreeSet};


/// `std::getline(ss, token, ',')` over a string: a trailing separator gives no empty last token.
fn getline_tokens(s: &str) -> Vec<&str> {
    let mut tokens: Vec<&str> = s.split(',').collect();
    if tokens.last().map_or(false, |t| t.is_empty()) {
        tokens.pop();
    }
    tokens
}

/// C `isspace` in the "C" locale.
fn c_isspace(c: u8) -> bool {
    matches!(c, b' ' | b'\t' | b'\n' | b'\x0b' | b'\x0c' | b'\r')
}

/// `std::stol(token)`: `strtol` base 10 over the leading integer; None where it throws
/// (`invalid_argument`: no digits; `out_of_range`: beyond `long`).
fn std_stol(token: &str) -> Option<i64> {
    let b = token.as_bytes();
    let mut i = 0;
    while i < b.len() && c_isspace(b[i]) {
        i += 1;
    }
    let start = i;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    let digits_start = i;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
    }
    if i == digits_start {
        return None;
    }
    token[start..i].parse::<i64>().ok()
}

/// The longest prefix `strtod` / `strtof` reads (after leading white space): decimal, `inf`/`infinity`,
/// `nan`; None where no conversion is performed. Hexadecimal floats are not read (see the note at
/// `std_stod`).
fn strtod_prefix(token: &str) -> Option<&str> {
    let b = token.as_bytes();
    let mut i = 0;
    while i < b.len() && c_isspace(b[i]) {
        i += 1;
    }
    let start = i;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    let rest = token[i..].to_ascii_lowercase();
    if rest.starts_with("infinity") {
        return Some(&token[start..i + 8]);
    }
    if rest.starts_with("inf") || rest.starts_with("nan") {
        return Some(&token[start..i + 3]);
    }
    let mut digits = 0;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
        digits += 1;
    }
    if i < b.len() && b[i] == b'.' {
        i += 1;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
            digits += 1;
        }
    }
    if digits == 0 {
        return None;
    }
    if i < b.len() && (b[i] == b'e' || b[i] == b'E') {
        let mut j = i + 1;
        if j < b.len() && (b[j] == b'+' || b[j] == b'-') {
            j += 1;
        }
        let exp_start = j;
        while j < b.len() && b[j].is_ascii_digit() {
            j += 1;
        }
        if j > exp_start {
            i = j;
        }
    }
    Some(&token[start..i])
}

/// `std::stod(token)`: None where it throws (`invalid_argument`: no conversion; `out_of_range`: `ERANGE`,
/// an overflow to infinity or an underflow to zero of a non-zero number). FIDELITY-NOTE: hexadecimal
/// floats (`0x1p3`) read as `0` here; the pin's `strtod` reads them whole.
fn std_stod(token: &str) -> Option<f64> {
    let text = strtod_prefix(token)?;
    let lower = text.to_ascii_lowercase();
    let v: f64 = text.parse().ok()?;
    let literal_inf_nan = lower.contains("inf") || lower.contains("nan");
    if !literal_inf_nan {
        if v.is_infinite() {
            return None;
        }
        let mantissa_nonzero = text.bytes().take_while(|c| *c != b'e' && *c != b'E').any(|c| (b'1'..=b'9').contains(&c));
        if v == 0.0 && mantissa_nonzero {
            return None;
        }
    }
    Some(v)
}

/// FilamentMixer.cpp:698-704 `static void append_error(...)`.
fn append_error(errors: &mut BTreeMap<String, String>, key: &str, msg: &str) {
    match errors.get_mut(key) {
        None => {
            errors.insert(key.to_string(), msg.to_string());
        }
        Some(v) => {
            *v += "; ";
            *v += msg;
        }
    }
}

/// FilamentMixer.cpp:706-718 `static bool has_mixed_sub_params_specified(...)`.
fn has_mixed_sub_params_specified(comp_strs: &[String], ratio_strs: &[String], gradient_flags: &[u8]) -> bool {
    comp_strs.iter().any(|s| !s.is_empty()) || ratio_strs.iter().any(|s| !s.is_empty()) || gradient_flags.iter().any(|g| *g != 0)
}

/// FilamentMixer.cpp:720-726 `static bool mixed_string_array_was_specified(...)`.
fn mixed_string_array_was_specified(vals: &[String]) -> bool {
    vals.iter().any(|s| !s.is_empty())
}

/// FilamentMixer.cpp:728-734 `static bool mixed_bool_array_was_specified(...)`.
fn mixed_bool_array_was_specified(vals: &[u8]) -> bool {
    vals.iter().any(|v| *v != 0)
}

/// FilamentMixer.cpp:736-746 `static void check_mixed_array_size_required(...)`.
fn check_mixed_array_size_required(errors: &mut BTreeMap<String, String>, opt_key: &str, actual_size: usize, expected_size: usize) {
    if actual_size != expected_size {
        append_error(errors, opt_key, &format!("array size {} does not match filament slot count {}", actual_size, expected_size));
    }
}

/// FilamentMixer.cpp:535-560 `std::vector<size_t> check_mixed_filament_diameter_consistency(...)`.
pub fn check_mixed_filament_diameter_consistency(is_mixed: &[u8], comp_strs: &[String], filament_diameters: &[String]) -> Vec<usize> {
    let mut result = Vec::new();
    for i in 0..is_mixed.len() {
        if is_mixed[i] == 0 || i >= comp_strs.len() || comp_strs[i].is_empty() {
            continue;
        }
        let components = parse_mixed_components(&comp_strs[i]);
        if components.len() < 2 {
            continue;
        }
        let mut diameters: BTreeSet<&String> = BTreeSet::new();
        for &component in &components {
            if component < 1 {
                continue;
            }
            let index = (component - 1) as usize;
            if index < filament_diameters.len() {
                diameters.insert(&filament_diameters[index]);
            }
        }
        if diameters.len() > 1 {
            result.push(i);
        }
    }
    result
}

/// FilamentMixer.cpp:597-639 `bool try_parse_mixed_components_strict(...)`.
pub fn try_parse_mixed_components_strict(str_: &str, components: &mut Vec<u32>, err: &mut String) -> bool {
    components.clear();
    if str_.is_empty() {
        *err = "empty component list".to_string();
        return false;
    }
    for token in getline_tokens(str_) {
        if token.is_empty() {
            *err = "empty component index".to_string();
            return false;
        }
        match std_stol(token) {
            Some(val) => {
                if val < 1 {
                    *err = format!("component index must be >= 1 (got {})", token);
                    return false;
                }
                components.push(val as u32);
            }
            None => {
                *err = format!("invalid component index \"{}\"", token);
                return false;
            }
        }
    }
    if components.len() < 2 {
        *err = format!("at least 2 components required (got {})", components.len());
        return false;
    }
    let mut seen: BTreeSet<u32> = BTreeSet::new();
    for &c in components.iter() {
        if !seen.insert(c) {
            *err = format!("duplicate component index {}", c);
            return false;
        }
    }
    true
}

/// FilamentMixer.cpp:641-675 `bool try_parse_mixed_ratios_strict(...)`.
pub fn try_parse_mixed_ratios_strict(str_: &str, n_components: usize, err: &mut String) -> bool {
    if str_.is_empty() {
        return true;
    }
    let mut ratios: Vec<f64> = Vec::new();
    for token in getline_tokens(str_) {
        if token.is_empty() {
            *err = "empty ratio value".to_string();
            return false;
        }
        match std_stod(token) {
            Some(val) => {
                if !(val > 0.0) {
                    *err = format!("ratio must be positive (got {})", token);
                    return false;
                }
                ratios.push(val);
            }
            None => {
                *err = format!("invalid ratio \"{}\"", token);
                return false;
            }
        }
    }
    if ratios.len() != n_components {
        *err = format!("expected {} ratio(s), got {}", n_components, ratios.len());
        return false;
    }
    true
}

/// FilamentMixer.cpp:677-696 `bool validate_gradient_range_strict(const std::string &str, std::string &err)`:
/// `std::sscanf(str.c_str(), "%f,%f", &v0, &v1) != 2`.
pub fn validate_gradient_range_strict(str_: &str, err: &mut String) -> bool {
    if str_.is_empty() {
        return true;
    }
    let scanned = (|| -> Option<(f32, f32)> {
        let a = strtod_prefix(str_)?;
        let a_end = str_.find(a).map(|p| p + a.len())?;
        let v0: f32 = a.parse().ok()?;
        let rest = &str_[a_end..];
        let rest = rest.strip_prefix(',')?;
        let b = strtod_prefix(rest)?;
        let v1: f32 = b.parse().ok()?;
        Some((v0, v1))
    })();
    let (v0, v1) = match scanned {
        Some(v) => v,
        None => {
            *err = "expected two comma-separated floats, e.g. \"0.10,0.90\"".to_string();
            return false;
        }
    };
    if !(v0 > 0.0 && v0 < 1.0 && v1 > 0.0 && v1 < 1.0) {
        *err = "start and end ratios must be in (0, 1)".to_string();
        return false;
    }
    true
}

/// FilamentMixer.cpp:748-840 `std::map<std::string, std::string> validate_mixed_filament_params(...)`.
pub fn validate_mixed_filament_params(
    is_mixed: &[u8],
    comp_strs: &[String],
    ratio_strs: &[String],
    gradient_flags: &[u8],
    gradient_range_strs: &[String],
    gradient_curve_strs: &[String],
) -> BTreeMap<String, String> {
    let mut errors: BTreeMap<String, String> = BTreeMap::new();

    if has_mixed_sub_params_specified(comp_strs, ratio_strs, gradient_flags) && !has_any_mixed_filament(is_mixed) {
        append_error(&mut errors, "filament_is_mixed", "must be set when mixed filament parameters are specified");
        return errors;
    }

    if !has_any_mixed_filament(is_mixed) {
        return errors;
    }

    let slot_count = is_mixed.len();

    // Rule 1: mixed filament model → components & ratios arrays must cover every slot.
    check_mixed_array_size_required(&mut errors, "filament_mixed_components", comp_strs.len(), slot_count);
    check_mixed_array_size_required(&mut errors, "filament_mixed_sublayer_ratios", ratio_strs.len(), slot_count);

    // Rule 2: gradient passed (any slot true) → gradient & range arrays must cover every slot.
    let gradient_specified = mixed_bool_array_was_specified(gradient_flags);
    if gradient_specified {
        check_mixed_array_size_required(&mut errors, "filament_mixed_gradient", gradient_flags.len(), slot_count);
        check_mixed_array_size_required(&mut errors, "filament_mixed_gradient_range", gradient_range_strs.len(), slot_count);
    }

    // Rule 3: curve passed (any non-empty entry) → curve array must cover every slot.
    let curve_specified = mixed_string_array_was_specified(gradient_curve_strs);
    if curve_specified {
        check_mixed_array_size_required(&mut errors, "filament_mixed_gradient_curve", gradient_curve_strs.len(), slot_count);
    }

    let num_physical = is_mixed.iter().filter(|v| **v == 0).count();

    for i in 0..is_mixed.len() {
        if is_mixed[i] == 0 {
            continue;
        }

        let slot = format!("slot {}", i + 1);
        let comp_str = if i < comp_strs.len() { comp_strs[i].as_str() } else { "" };

        let mut components: Vec<u32> = Vec::new();
        let mut comp_err = String::new();
        if !try_parse_mixed_components_strict(comp_str, &mut components, &mut comp_err) {
            append_error(&mut errors, "filament_mixed_components", &format!("{}: {}", slot, comp_err));
            continue;
        }

        for &c in &components {
            if c as usize > num_physical {
                append_error(
                    &mut errors,
                    "filament_mixed_components",
                    &format!("{}: component {} out of range (max physical filament index is {})", slot, c, num_physical),
                );
                break;
            }
            if c as usize == i + 1 {
                append_error(&mut errors, "filament_mixed_components", &format!("{}: cannot reference itself as a component", slot));
                break;
            }
            let idx0 = c.wrapping_sub(1) as usize;
            if idx0 < is_mixed.len() && is_mixed[idx0] != 0 {
                append_error(&mut errors, "filament_mixed_components", &format!("{}: component {} references a mixed filament slot", slot, c));
                break;
            }
        }

        let mut ratio_err = String::new();
        let ratio_str = if i < ratio_strs.len() { ratio_strs[i].as_str() } else { "" };
        if !try_parse_mixed_ratios_strict(ratio_str, components.len(), &mut ratio_err) {
            append_error(&mut errors, "filament_mixed_sublayer_ratios", &format!("{}: {}", slot, ratio_err));
        }

        let gradient_on = i < gradient_flags.len() && gradient_flags[i] != 0;
        if gradient_on {
            if components.len() != 2 {
                append_error(&mut errors, "filament_mixed_gradient", &format!("{}: gradient requires exactly 2 components", slot));
            }

            if gradient_specified {
                let mut range_err = String::new();
                let range_str = if i < gradient_range_strs.len() { gradient_range_strs[i].as_str() } else { "" };
                if !validate_gradient_range_strict(range_str, &mut range_err) {
                    append_error(&mut errors, "filament_mixed_gradient_range", &format!("{}: {}", slot, range_err));
                }
            }

            if curve_specified {
                let curve_str = if i < gradient_curve_strs.len() { gradient_curve_strs[i].as_str() } else { "" };
                if !curve_str.is_empty() && parse_gradient_curve(curve_str).empty() {
                    append_error(
                        &mut errors,
                        "filament_mixed_gradient_curve",
                        &format!("{}: invalid curve (need at least 2 valid control points)", slot),
                    );
                }
            }
        }
    }

    errors
}
