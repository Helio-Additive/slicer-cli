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
