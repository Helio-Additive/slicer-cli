//! Faithful 1:1 port of `src/libslic3r/GCode.cpp` (BambuStudio).
//!
//! GCode.cpp is the central G-code generator orchestrator. The bulk of the
//! file is the `GCode` class together with the helper classes `OozePrevention`,
//! `Wipe` and `WipeTowerIntegration`, all of which require the full `Print` /
//! `PrintObject` / `Layer` graph, a live `GCodeWriter`, `GCodeProcessor`,
//! `PlaceholderParser`, `WipeTower::ToolChangeResult` instances, the TBB
//! parallel pipeline, `DoExport`, `EdgeGrid`, `Skirt` and many other
//! dependencies to be threaded through. Those are NOT yet wired up in this
//! crate, so the `GCode` class methods are listed as blocked (see PORT_LEDGER
//! and the porter report).
//!
//! This module ports the genuinely self-contained, free-standing helpers from
//! GCode.cpp line-by-line: the module-level constants, the bed-type mapping,
//! the pure string utilities (`check_add_eol`, `custom_gcode_changes_tool`,
//! `transform_gcode`), and `get_wipe_avoid_pos_x`.

use crate::libslic3r::scale;

// GCode.cpp:84
pub const G_MIN_PURGE_VOLUME: f32 = 100.0;
// GCode.cpp:85
pub const G_PURGE_VOLUME_ONE_TIME: f32 = 135.0;
// GCode.cpp:86
pub const G_MAX_FLUSH_COUNT: i32 = 4;
// GCode.cpp:87
pub const G_MAX_LABEL_OBJECT: usize = 64;
// GCode.cpp:88
pub const SMOOTH_SPEED_STEP: f64 = 10.0;

// GCode.cpp:89 — static const double not_split_length = scale_(1.0);
#[inline]
pub fn not_split_length() -> f64 {
    scale(1.0) as f64
}
// GCode.cpp:90 — static const double max_step_length = scale_(1.0); // cut path if the path too long
#[inline]
pub fn max_step_length() -> f64 {
    scale(1.0) as f64 // cut path if the path too long
}
// GCode.cpp:91 — static const double min_step_length = scale_(0.4); // cut step
#[inline]
pub fn min_step_length() -> f64 {
    scale(0.4) as f64 // cut step
}

// GCode.cpp:218
// Only add a newline in case the current G-code does not end with a newline.
pub fn check_add_eol(gcode: &mut String) {
    // GCode.cpp:221
    if !gcode.is_empty() && !gcode.ends_with('\n') {
        gcode.push('\n');
    }
}

// GCode.cpp:227
// Return true if tch_prefix is found in custom_gcode
pub fn custom_gcode_changes_tool(custom_gcode: &str, tch_prefix: &str, next_extruder: u32) -> bool {
    // GCode.cpp:229
    let mut ok = false;
    // GCode.cpp:230
    let mut from_pos: usize = 0;
    // GCode.cpp:232
    let bytes = custom_gcode.as_bytes();
    while let Some(rel) = custom_gcode[from_pos..].find(tch_prefix) {
        let mut pos = from_pos + rel;
        // GCode.cpp:233
        if pos + 1 == custom_gcode.len() {
            break;
        }
        // GCode.cpp:235
        from_pos = pos + 1;
        // GCode.cpp:236-240 — only whitespace is allowed before the command.
        //   while (--pos < custom_gcode.size() && custom_gcode[pos] != '\n')
        // C++ uses an unsigned size_t for `pos`, so when `pos` is 0 the
        // pre-decrement wraps to SIZE_MAX which fails the `< size()` test and
        // terminates the loop. Replicate that wrap-around exactly.
        let mut next_iter = true;
        loop {
            pos = pos.wrapping_sub(1);
            if !(pos < custom_gcode.len() && bytes[pos] != b'\n') {
                break;
            }
            // GCode.cpp:238 — if (!std::isspace(custom_gcode[pos])) goto NEXT;
            if !is_space(bytes[pos]) {
                next_iter = false;
                break;
            }
        }
        if next_iter {
            // GCode.cpp:241-247
            // we should also check that the extruder changes to what was expected
            let tail = &custom_gcode[from_pos..];
            if let Some(num) = parse_leading_uint(tail) {
                ok = num == next_extruder;
            }
        }
        // GCode.cpp:248 NEXT:;
    }
    // GCode.cpp:250
    ok
}

// std::isspace for the default "C" locale: space, \t, \n, \v, \f, \r.
#[inline]
fn is_space(c: u8) -> bool {
    matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

// Mirror of C++ `std::istringstream >> unsigned`: skip leading whitespace, then
// read a run of decimal digits. Returns None if no number could be parsed.
fn parse_leading_uint(s: &str) -> Option<u32> {
    let bytes = s.as_bytes();
    let mut i = 0;
    while i < bytes.len() && is_space(bytes[i]) {
        i += 1;
    }
    let start = i;
    while i < bytes.len() && bytes[i].is_ascii_digit() {
        i += 1;
    }
    if start == i {
        return None;
    }
    s[start..i].parse::<u32>().ok()
}

// GCode.cpp:298
// Postprocesses gcode: rotates and moves G1 extrusions and returns result.
//
// `WipeTower::never_skip_tag()` (WipeTower.hpp:28) is not yet ported; its value
// is the literal below.
pub fn transform_gcode(gcode: &str, mut pos: Vec2f, translation: &Vec2f, angle: f32) -> String {
    // GCode.cpp:300
    let extruder_offset = Vec2f::new(0.0, 0.0);
    // GCode.cpp:302
    let mut gcode_out = String::new();
    // GCode.cpp:305
    let mut old_pos = Vec2f::new(-1000.1, -1000.1);

    // GCode.cpp:307-308 — read the gcode line by line.
    for raw_line in split_getlines(gcode) {
        let mut line = raw_line;

        // GCode.cpp:310
        if line.starts_with("G1 ") {
            // GCode.cpp:311
            let mut never_skip = false;
            // GCode.cpp:312-317
            if let Some(it) = line.find(never_skip_tag()) {
                // remove the tag and remember we saw it
                never_skip = true;
                line.replace_range(it..it + never_skip_tag().len(), "");
            }
            // GCode.cpp:318-327 — parse X/Y values out of the line, building
            // line_out from every character that is not an X/Y coordinate.
            //   line_str >> std::noskipws; (don't skip whitespace)
            let mut line_out = String::new();
            let chars: Vec<char> = line.chars().collect();
            let mut idx = 0;
            while idx < chars.len() {
                let ch = chars[idx];
                idx += 1;
                if ch == 'X' || ch == 'Y' {
                    // line_str >> (ch == 'X' ? pos.x() : pos.y());
                    let (val, consumed) = stream_read_float(&chars[idx..]);
                    idx += consumed;
                    if let Some(v) = val {
                        if ch == 'X' {
                            pos[0] = v;
                        } else {
                            pos[1] = v;
                        }
                    }
                } else {
                    line_out.push(ch);
                }
            }

            // GCode.cpp:329 — transformed_pos = Eigen::Rotation2Df(angle) * pos + translation;
            let transformed_pos = rotate2d(angle, pos) + translation;

            // GCode.cpp:331
            if transformed_pos != old_pos || never_skip {
                // GCode.cpp:332
                line = line_out;
                // GCode.cpp:333-336
                let mut oss = String::from("G1 ");
                // oss << std::fixed << std::setprecision(3)
                if transformed_pos.x != old_pos.x || never_skip {
                    oss.push_str(&format!(" X{:.3}", transformed_pos.x - extruder_offset.x));
                }
                if transformed_pos.y != old_pos.y || never_skip {
                    oss.push_str(&format!(" Y{:.3}", transformed_pos.y - extruder_offset.y));
                }
                // GCode.cpp:337
                oss.push(' ');
                // GCode.cpp:338 — line.replace(line.find("G1 "), 3, oss.str());
                if let Some(g1) = line.find("G1 ") {
                    line.replace_range(g1..g1 + 3, &oss);
                }
                // GCode.cpp:339
                old_pos = transformed_pos;
            }
        }

        // GCode.cpp:343
        gcode_out.push_str(&line);
        gcode_out.push('\n');
    }
    // GCode.cpp:345
    gcode_out
}

// GCode.cpp:348
pub fn get_wipe_avoid_pos_x(wt_min: &Vec2f, wt_max: &Vec2f, offset: f32) -> f32 {
    // GCode.cpp:350
    let left: f32 = 100.0;
    let right: f32 = 250.0;
    // GCode.cpp:351
    let default_value: f32 = 110.0;
    // GCode.cpp:352
    let a: f32;
    let b: f32;
    // GCode.cpp:353
    a = wt_max.x + offset;
    // GCode.cpp:354
    b = wt_min.x - offset;
    // GCode.cpp:355
    if a > left && a < right {
        return a;
    }
    // GCode.cpp:356
    if b > left && b < right {
        return b;
    }
    // GCode.cpp:357
    default_value
}

// GCode.cpp:2036
pub fn to_bambu_bed_type(ty: BedType) -> BambuBedType {
    // GCode.cpp:2038
    let mut bambu_bed_type = BambuBedType::Unknown;
    // GCode.cpp:2039-2048
    if ty == BedType::Pc {
        bambu_bed_type = BambuBedType::CoolPlate;
    } else if ty == BedType::Ep {
        bambu_bed_type = BambuBedType::EngineeringPlate;
    } else if ty == BedType::Pei {
        bambu_bed_type = BambuBedType::HighTemperaturePlate;
    } else if ty == BedType::Pte {
        bambu_bed_type = BambuBedType::TexturedPeiPlate;
    } else if ty == BedType::SuperTack {
        bambu_bed_type = BambuBedType::SuperTackPlate;
    }
    // GCode.cpp:2050
    bambu_bed_type
}

// ---------------------------------------------------------------------------
// Local geometry / enum mirrors
// ---------------------------------------------------------------------------

/// `Vec2f` = `Eigen::Vector2f`. A 2-component f32 vector with named fields so
/// the ported code reads like the C++ `.x()` / `.y()` accessors.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec2f {
    pub x: f32,
    pub y: f32,
}

impl Vec2f {
    #[inline]
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
}

impl std::ops::Index<usize> for Vec2f {
    type Output = f32;
    #[inline]
    fn index(&self, i: usize) -> &f32 {
        match i {
            0 => &self.x,
            1 => &self.y,
            _ => panic!("Vec2f index out of range"),
        }
    }
}

impl std::ops::IndexMut<usize> for Vec2f {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut f32 {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            _ => panic!("Vec2f index out of range"),
        }
    }
}

impl std::ops::Add<&Vec2f> for Vec2f {
    type Output = Vec2f;
    #[inline]
    fn add(self, rhs: &Vec2f) -> Vec2f {
        Vec2f::new(self.x + rhs.x, self.y + rhs.y)
    }
}

/// Eigen `Rotation2Df(angle) * v`: rotates by `[cos -sin; sin cos] * v`.
/// (Mirrors the helper used in `triangle_mesh.rs`.)
#[inline]
fn rotate2d(angle: f32, v: Vec2f) -> Vec2f {
    let c = angle.cos();
    let s = angle.sin();
    Vec2f::new(c * v.x - s * v.y, s * v.x + c * v.y)
}

/// Mirror of `WipeTower::never_skip_tag()` (WipeTower.hpp:28).
#[inline]
fn never_skip_tag() -> &'static str {
    "_GCODE_WIPE_TOWER_NEVER_SKIP_TAG"
}

/// `BedType` (PrintConfig.hpp). Variant names mirror the C++ `btPC` etc.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BedType {
    Pc,
    Ep,
    Pei,
    Pte,
    SuperTack,
}

/// `BambuBedType` (GCode/GCodeProcessor or BBS plate types).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BambuBedType {
    Unknown,
    CoolPlate,
    EngineeringPlate,
    HighTemperaturePlate,
    TexturedPeiPlate,
    SuperTackPlate,
}

// ---------------------------------------------------------------------------
// C++ stream-parsing helpers (faithful to std::getline / std::istream >> float)
// ---------------------------------------------------------------------------

/// Mirror of repeated `std::getline(gcode_str, line)` over an istringstream.
///
/// `while (gcode_str) { std::getline(...); ... }` reads one line per iteration.
/// std::getline strips the delimiter `\n`. After the final `\n` the stream is
/// still "good" so getline runs once more, yielding an empty trailing line
/// (which sets eof and ends the loop). For a string without a trailing newline,
/// getline yields the last partial line then the loop ends. We replicate the
/// exact set of lines C++ would observe.
fn split_getlines(s: &str) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    if s.is_empty() {
        // C++: the first getline on an empty (but good) stream extracts nothing
        // and sets failbit/eof; the body still runs once with an empty `line`.
        out.push(String::new());
        return out;
    }
    let mut cur = String::new();
    for ch in s.chars() {
        if ch == '\n' {
            out.push(std::mem::take(&mut cur));
        } else {
            cur.push(ch);
        }
    }
    if s.ends_with('\n') {
        // After consuming the trailing '\n', the stream is still good, so
        // getline runs once more producing an empty line before hitting eof.
        out.push(String::new());
    } else {
        // Final partial line.
        out.push(cur);
    }
    out
}

/// Mirror of `std::istream >> float` with `std::noskipws` already cleared for
/// the numeric extraction (operator>> always skips leading whitespace by
/// default for arithmetic types). Returns the parsed value (None on failure)
/// and the number of characters consumed.
fn stream_read_float(chars: &[char]) -> (Option<f32>, usize) {
    let mut i = 0;
    // operator>> for arithmetic types skips leading whitespace.
    while i < chars.len() && chars[i].is_whitespace() {
        i += 1;
    }
    let start = i;
    // optional sign
    if i < chars.len() && (chars[i] == '+' || chars[i] == '-') {
        i += 1;
    }
    // integer part
    while i < chars.len() && chars[i].is_ascii_digit() {
        i += 1;
    }
    // fractional part
    if i < chars.len() && chars[i] == '.' {
        i += 1;
        while i < chars.len() && chars[i].is_ascii_digit() {
            i += 1;
        }
    }
    // exponent
    if i < chars.len() && (chars[i] == 'e' || chars[i] == 'E') {
        let mut j = i + 1;
        if j < chars.len() && (chars[j] == '+' || chars[j] == '-') {
            j += 1;
        }
        let exp_digits_start = j;
        while j < chars.len() && chars[j].is_ascii_digit() {
            j += 1;
        }
        if j > exp_digits_start {
            i = j;
        }
    }
    let text: String = chars[start..i].iter().collect();
    let val = text.parse::<f32>().ok();
    (val, i)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_check_add_eol() {
        let mut g = String::new();
        check_add_eol(&mut g);
        assert_eq!(g, ""); // empty stays empty
        let mut g = String::from("G1");
        check_add_eol(&mut g);
        assert_eq!(g, "G1\n");
        let mut g = String::from("G1\n");
        check_add_eol(&mut g);
        assert_eq!(g, "G1\n"); // already terminated
    }

    #[test]
    fn test_custom_gcode_changes_tool() {
        // "T1" at line start, followed by extruder index 1 -> ok when next==1
        assert!(custom_gcode_changes_tool("T1\n", "T", 1));
        assert!(!custom_gcode_changes_tool("T1\n", "T", 2));
        // preceded by non-whitespace -> not a tool change
        assert!(!custom_gcode_changes_tool("XT1\n", "T", 1));
    }

    #[test]
    fn test_to_bambu_bed_type() {
        assert_eq!(to_bambu_bed_type(BedType::Pc), BambuBedType::CoolPlate);
        assert_eq!(
            to_bambu_bed_type(BedType::Pte),
            BambuBedType::TexturedPeiPlate
        );
        assert_eq!(
            to_bambu_bed_type(BedType::SuperTack),
            BambuBedType::SuperTackPlate
        );
    }

    #[test]
    fn test_get_wipe_avoid_pos_x() {
        let wt_min = Vec2f::new(120.0, 0.0);
        let wt_max = Vec2f::new(150.0, 0.0);
        // a = 150+offset within (100,250) -> returns a
        assert_eq!(get_wipe_avoid_pos_x(&wt_min, &wt_max, 10.0), 160.0);
        // both out of range -> default
        let wt_min = Vec2f::new(50.0, 0.0);
        let wt_max = Vec2f::new(300.0, 0.0);
        assert_eq!(get_wipe_avoid_pos_x(&wt_min, &wt_max, 0.0), 110.0);
    }
}
