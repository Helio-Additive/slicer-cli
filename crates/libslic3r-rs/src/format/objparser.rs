//! OBJ/MTL file parser.
//!
//! C++ Reference:
//! - Format/objparser.hpp
//! - Format/objparser.cpp
//!
//! Faithful 1:1 port. The C++ works on NUL-terminated `const char *` lines with
//! `strtod`/`strtol`; the Rust port mirrors that with byte slices plus an index
//! cursor, where reading past the end of the slice yields the NUL terminator
//! (`0`). `strtod`/`strtol` are emulated with exact C semantics (longest valid
//! prefix, `endptr` left at the first unconsumed byte, no conversion =>
//! `endptr == nptr` and value 0).

use std::collections::HashMap;
use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::Path;
use std::sync::Arc;

use log::error;

use crate::locales_utils::{is_decimal_separator_point, CNumericLocalesSetter};

// ---------------------------------------------------------------------------
// Constants  (objparser.hpp:93-95)
// ---------------------------------------------------------------------------

/// objparser.hpp:93 — `#define OBJ_VERTEX_COLOR_ALPHA 6`
pub const OBJ_VERTEX_COLOR_ALPHA: usize = 6;

/// objparser.hpp:94 — `#define OBJ_VERTEX_LENGTH 7` (x, y, z, color_x, color_y, color_z, color_w)
pub const OBJ_VERTEX_LENGTH: usize = 7;

/// objparser.hpp:95 — `#define ONE_FACE_SIZE 4` (ONE_FACE format: f 8/4/6 7/3/6 6/2/6 -1/-1/-1)
pub const ONE_FACE_SIZE: usize = 4;

// ---------------------------------------------------------------------------
// Data structures  (objparser.hpp)
// ---------------------------------------------------------------------------

/// objparser.hpp:12-17 — struct ObjVertex
// objparser.hpp:19-24 — operator== compares all three members; the derive matches.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ObjVertex {
    pub coord_idx: i32,
    pub texture_coord_idx: i32,
    pub normal_idx: i32,
}

impl ObjVertex {
    /// Sentinel vertex used to delimit faces in the vertex list (objparser.cpp:272-275).
    pub fn delimiter() -> Self {
        Self {
            coord_idx: -1,
            texture_coord_idx: -1,
            normal_idx: -1,
        }
    }
}

/// objparser.hpp:26-33 — struct ObjUseMtl
///
/// In C++ `vertexIdxFirst` and `face_start` carry no initializer (zero under
/// value-initialization `T()`); `vertexIdxEnd{-1}` and `face_end{-1}` default
/// to -1.
#[derive(Debug, Clone)]
pub struct ObjUseMtl {
    pub vertex_idx_first: i32,
    pub vertex_idx_end: i32,
    pub face_start: i32,
    pub face_end: i32,
    pub name: String,
}

impl ObjUseMtl {
    pub fn new() -> Self {
        Self {
            vertex_idx_first: 0,
            vertex_idx_end: -1,
            face_start: 0,
            face_end: -1,
            name: String::new(),
        }
    }
}

impl Default for ObjUseMtl {
    fn default() -> Self {
        Self::new()
    }
}

/// objparser.hpp:51-55 — operator==(const ObjUseMtl&, const ObjUseMtl&)
/// compares only `vertexIdxFirst` and `name`.
impl PartialEq for ObjUseMtl {
    fn eq(&self, other: &Self) -> bool {
        // hpp:53-54
        self.vertex_idx_first == other.vertex_idx_first && self.name == other.name
    }
}

/// objparser.hpp:35-49 — struct ObjNewMtl
///
/// In C++ only `Tr{1.0f}` has an initializer; the remaining floats are
/// uninitialized under default construction (zero under `make_shared`'s
/// value-initialization). Rust defaults everything to 0.0 except `tr`.
#[derive(Debug, Clone)]
pub struct ObjNewMtl {
    pub name: String,
    pub ns: f32,
    pub ni: f32,
    pub d: f32,
    pub illum: f32,
    /// Transmission (hpp:42 — `Tr{1.0f}`)
    pub tr: f32,
    pub tf: [f32; 3],
    pub ka: [f32; 3],
    pub kd: [f32; 3],
    pub ks: [f32; 3],
    pub ke: [f32; 3],
    /// default png (hpp:48)
    pub map_kd: String,
}

impl ObjNewMtl {
    pub fn new() -> Self {
        Self {
            name: String::new(),
            ns: 0.0,
            ni: 0.0,
            d: 0.0,
            illum: 0.0,
            tr: 1.0,
            tf: [0.0; 3],
            ka: [0.0; 3],
            kd: [0.0; 3],
            ks: [0.0; 3],
            ke: [0.0; 3],
            map_kd: String::new(),
        }
    }
}

impl Default for ObjNewMtl {
    fn default() -> Self {
        Self::new()
    }
}

/// objparser.hpp:57-61 — struct ObjObject
// objparser.hpp:63-68 — operator== compares vertexIdxFirst and name; the derive matches.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObjObject {
    pub vertex_idx_first: i32,
    pub name: String,
}

/// objparser.hpp:70-74 — struct ObjGroup
// objparser.hpp:76-80 — operator== compares vertexIdxFirst and name; the derive matches.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObjGroup {
    pub vertex_idx_first: i32,
    pub name: String,
}

/// objparser.hpp:82-86 — struct ObjSmoothingGroup (both members are C++ `int`)
// objparser.hpp:88-92 — operator== compares both members; the derive matches.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ObjSmoothingGroup {
    pub vertex_idx_first: i32,
    pub smoothing_group_id: i32,
}

/// objparser.hpp:96-122 — struct ObjData
#[derive(Debug, Clone)]
pub struct ObjData {
    /// Version of the data structure for load / store in the private binary format. (hpp:98)
    pub version: i32,
    /// x, y, z, color_x, color_y, color_z, color_w (hpp:100-101)
    pub coordinates: Vec<f32>,
    /// hpp:102 — `has_vertex_color{false}`
    pub has_vertex_color: bool,
    /// u, v, w (hpp:103-104)
    pub texture_coordinates: Vec<f32>,
    /// x, y, z (hpp:105-106)
    pub normals: Vec<f32>,
    /// u, v, w (hpp:107-108)
    pub parameters: Vec<f32>,
    pub mtllibs: Vec<String>,
    pub usemtls: Vec<ObjUseMtl>,
    pub objects: Vec<ObjObject>,
    pub groups: Vec<ObjGroup>,
    pub smoothing_groups: Vec<ObjSmoothingGroup>,
    /// List of faces, delimited by an ObjVertex with all members set to -1. (hpp:116-117)
    pub vertices: Vec<ObjVertex>,

    // hpp:119-121 — MakerLab metadata
    pub ml_region: String,
    pub ml_name: String,
    pub ml_id: String,
}

impl ObjData {
    pub fn new() -> Self {
        Self {
            version: 0,
            coordinates: Vec::new(),
            has_vertex_color: false,
            texture_coordinates: Vec::new(),
            normals: Vec::new(),
            parameters: Vec::new(),
            mtllibs: Vec::new(),
            usemtls: Vec::new(),
            objects: Vec::new(),
            groups: Vec::new(),
            smoothing_groups: Vec::new(),
            vertices: Vec::new(),
            ml_region: String::new(),
            ml_name: String::new(),
            ml_id: String::new(),
        }
    }
}

impl Default for ObjData {
    fn default() -> Self {
        Self::new()
    }
}

/// objparser.hpp:124-131 — struct MtlData
#[derive(Debug, Clone)]
pub struct MtlData {
    /// Version of the data structure for load / store in the private binary format. (hpp:127)
    pub version: i32,
    /// hpp:128 — `first_time_using_makerlab{false}`
    pub first_time_using_makerlab: bool,
    /// hpp:129 — `std::unordered_map<std::string, std::shared_ptr<ObjNewMtl>>`
    pub new_mtl_unmap: HashMap<String, Arc<ObjNewMtl>>,
    /// hpp:130
    pub mtl_orders: Vec<String>,
}

impl MtlData {
    pub fn new() -> Self {
        Self {
            version: 0,
            first_time_using_makerlab: false,
            new_mtl_unmap: HashMap::new(),
            mtl_orders: Vec::new(),
        }
    }
}

impl Default for MtlData {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// C string / strtod / strtol emulation helpers
// ---------------------------------------------------------------------------

/// Read the byte at index `i`; past the end of the slice this yields the NUL
/// terminator of the C string (`0`).
#[inline]
fn at(line: &[u8], i: usize) -> u8 {
    if i < line.len() {
        line[i]
    } else {
        0
    }
}

/// objparser.cpp:12 — `#define EATWS() while (*line == ' ' || *line == '\t') ++line`
#[inline]
fn eatws(line: &[u8], p: &mut usize) {
    while at(line, *p) == b' ' || at(line, *p) == b'\t' {
        *p += 1;
    }
}

/// The remainder of the C string starting at `p` (i.e. `std::string(line)`):
/// truncated at the first NUL byte, as C string handling would.
fn cstr_tail(line: &[u8], p: usize) -> &[u8] {
    let t = &line[p.min(line.len())..];
    &t[..t.iter().position(|&b| b == 0).unwrap_or(t.len())]
}

/// `std::string` from raw bytes (C++ strings are byte strings; invalid UTF-8 is
/// replaced, which is the closest safe equivalent for Rust's `String`).
fn lossy(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes).into_owned()
}

/// Case-insensitive ASCII prefix test (for strtod's `inf`/`nan` spellings).
fn starts_with_ci(haystack: &[u8], needle: &[u8]) -> bool {
    haystack.len() >= needle.len()
        && haystack[..needle.len()]
            .iter()
            .zip(needle)
            .all(|(a, b)| a.eq_ignore_ascii_case(b))
}

/// The consumed bytes are pure ASCII digits, so this never fails.
fn ascii(bytes: &[u8]) -> &str {
    core::str::from_utf8(bytes).unwrap_or("0")
}

/// C `strtod(line + p, &endptr)` emulation over the NUL-terminated C string
/// `line`. Returns `(value, endptr)` where `endptr` is the index just past the
/// consumed prefix; when no conversion is performed the C contract is value
/// `0.0` and `endptr == p` (the original pointer).
/// `pub(crate)`: also backs `atof` in `format::amf` (AMF.cpp parses all
/// numeric character data with C `atof`, which is `strtod(nptr, NULL)`).
pub(crate) fn strtod(line: &[u8], p: usize) -> (f64, usize) {
    let n = line.len();
    // strtod skips leading C-locale whitespace.
    let mut i = p;
    while i < n && matches!(line[i], b' ' | b'\t' | b'\n' | 0x0B | 0x0C | b'\r') {
        i += 1;
    }
    // Optional sign.
    let mut j = i;
    let mut negative = false;
    if j < n && (line[j] == b'+' || line[j] == b'-') {
        negative = line[j] == b'-';
        j += 1;
    }
    let rest = &line[j.min(n)..];
    // inf / infinity (case-insensitive).
    if starts_with_ci(rest, b"infinity") {
        let v = if negative {
            f64::NEG_INFINITY
        } else {
            f64::INFINITY
        };
        return (v, j + 8);
    }
    if starts_with_ci(rest, b"inf") {
        let v = if negative {
            f64::NEG_INFINITY
        } else {
            f64::INFINITY
        };
        return (v, j + 3);
    }
    // nan with an optional (n-char-sequence).
    if starts_with_ci(rest, b"nan") {
        let mut k = j + 3;
        if at(line, k) == b'(' {
            let mut m = k + 1;
            while m < n && (line[m].is_ascii_alphanumeric() || line[m] == b'_') {
                m += 1;
            }
            if m < n && line[m] == b')' {
                k = m + 1;
            }
        }
        return (f64::NAN, k);
    }
    // Hexadecimal form: 0x/0X with at least one hex digit.
    if rest.len() >= 2 && rest[0] == b'0' && (rest[1] == b'x' || rest[1] == b'X') {
        let mut k = j + 2;
        let int_start = k;
        while k < n && line[k].is_ascii_hexdigit() {
            k += 1;
        }
        let int_end = k;
        let mut frac_start = k;
        let mut frac_end = k;
        if k < n && line[k] == b'.' {
            k += 1;
            frac_start = k;
            while k < n && line[k].is_ascii_hexdigit() {
                k += 1;
            }
            frac_end = k;
        }
        if int_end > int_start || frac_end > frac_start {
            let mut value: f64 = 0.0;
            for &b in &line[int_start..int_end] {
                value = value * 16.0 + (b as char).to_digit(16).unwrap_or(0) as f64;
            }
            let mut scale = 1.0f64 / 16.0;
            for &b in &line[frac_start..frac_end] {
                value += (b as char).to_digit(16).unwrap_or(0) as f64 * scale;
                scale /= 16.0;
            }
            // Optional binary exponent: p/P [sign] decimal-digits.
            if k < n && (line[k] == b'p' || line[k] == b'P') {
                let mut m = k + 1;
                let mut exp_neg = false;
                if m < n && (line[m] == b'+' || line[m] == b'-') {
                    exp_neg = line[m] == b'-';
                    m += 1;
                }
                let exp_digits = m;
                let mut exp: i32 = 0;
                while m < n && line[m].is_ascii_digit() {
                    exp = exp
                        .saturating_mul(10)
                        .saturating_add((line[m] - b'0') as i32);
                    m += 1;
                }
                if m > exp_digits {
                    k = m;
                    let e = if exp_neg { -exp } else { exp };
                    value *= 2.0f64.powi(e);
                }
            }
            return (if negative { -value } else { value }, k);
        }
        // "0x" with no hex digit: the subject sequence is just the "0".
        return (0.0, j + 1);
    }
    // Decimal form: digits [. digits] [e/E [sign] digits].
    let int_start = j;
    let mut k = j;
    while k < n && line[k].is_ascii_digit() {
        k += 1;
    }
    let int_end = k;
    let mut frac_start = k;
    let mut frac_end = k;
    if k < n && line[k] == b'.' {
        k += 1;
        frac_start = k;
        while k < n && line[k].is_ascii_digit() {
            k += 1;
        }
        frac_end = k;
    }
    if int_end == int_start && frac_end == frac_start {
        // No conversion performed: value 0, endptr = the original nptr.
        return (0.0, p);
    }
    // The exponent is only consumed when well formed (>= 1 digit).
    let mut exp_text: &[u8] = b"0";
    let mut exp_neg = false;
    if k < n && (line[k] == b'e' || line[k] == b'E') {
        let mut m = k + 1;
        if m < n && (line[m] == b'+' || line[m] == b'-') {
            exp_neg = line[m] == b'-';
            m += 1;
        }
        let exp_digits = m;
        while m < n && line[m].is_ascii_digit() {
            m += 1;
        }
        if m > exp_digits {
            exp_text = &line[exp_digits..m];
            k = m;
        }
    }
    // Rebuild a normalized literal that Rust's correctly-rounded parser accepts;
    // the decimal value is textually identical, so the resulting f64 matches
    // a correctly-rounded C strtod bit for bit.
    let mut text = String::with_capacity((frac_end - int_start) + exp_text.len() + 8);
    if int_end == int_start {
        text.push('0');
    } else {
        text.push_str(ascii(&line[int_start..int_end]));
    }
    text.push('.');
    if frac_end == frac_start {
        text.push('0');
    } else {
        text.push_str(ascii(&line[frac_start..frac_end]));
    }
    text.push('e');
    if exp_neg {
        text.push('-');
    }
    text.push_str(ascii(exp_text));
    let magnitude: f64 = text.parse().unwrap_or(0.0);
    (if negative { -magnitude } else { magnitude }, k)
}

/// C `strtol(line + p, &endptr, 10)` emulation (LP64 `long` = i64). Returns
/// `(value, endptr)`; no conversion yields `(0, p)`; out-of-range values clamp
/// to LONG_MAX / LONG_MIN as strtol does.
fn strtol(line: &[u8], p: usize) -> (i64, usize) {
    let n = line.len();
    // strtol skips leading C-locale whitespace.
    let mut i = p;
    while i < n && matches!(line[i], b' ' | b'\t' | b'\n' | 0x0B | 0x0C | b'\r') {
        i += 1;
    }
    let mut negative = false;
    if i < n && (line[i] == b'+' || line[i] == b'-') {
        negative = line[i] == b'-';
        i += 1;
    }
    let digits_start = i;
    let mut value: u128 = 0;
    while i < n && line[i].is_ascii_digit() {
        value = value * 10 + (line[i] - b'0') as u128;
        if value > 1u128 << 63 {
            // Sticky cap: keep the overflow detectable without u128 overflow.
            value = (1u128 << 63) + 1;
        }
        i += 1;
    }
    if i == digits_start {
        // No conversion performed: value 0, endptr = the original nptr.
        return (0, p);
    }
    let v = if negative {
        if value >= 1u128 << 63 {
            i64::MIN
        } else {
            -(value as i64)
        }
    } else if value > (1u128 << 63) - 1 {
        i64::MAX
    } else {
        value as i64
    };
    (v, i)
}

// ---------------------------------------------------------------------------
// obj_parseline  (objparser.cpp:13-375)
// ---------------------------------------------------------------------------

/// objparser.cpp:13 — `static bool obj_parseline(const char *line, ObjData &data)`
fn obj_parseline(line: &[u8], data: &mut ObjData) -> bool {
    let mut p = 0usize;
    // objparser.cpp:15-16 — if (*line == 0) return true;
    if at(line, p) == 0 {
        return true;
    }
    // objparser.cpp:17 — assert(Slic3r::is_decimal_separator_point());
    debug_assert!(is_decimal_separator_point());
    // Ignore whitespaces at the beginning of the line.
    // FIXME is this a good idea?
    // objparser.cpp:20 — EATWS();
    eatws(line, &mut p);

    // objparser.cpp:22 — char c1 = *line ++;
    let c1 = at(line, p);
    p += 1;
    match c1 {
        // objparser.cpp:24-26
        b'#' => {
            // Comment, ignore the rest of the line.
        }
        // objparser.cpp:27-199
        b'v' => {
            // Parse vertex geometry (position, normal, texture coordinates)
            // objparser.cpp:30 — char c2 = *line ++;
            let c2 = at(line, p);
            p += 1;
            match c2 {
                // objparser.cpp:32-68
                b't' => {
                    // vt - vertex texture parameter
                    // u v [w], w == 0 (or w == 1)
                    // objparser.cpp:36 — char c2 = *line ++;
                    let c2 = at(line, p);
                    p += 1;
                    if c2 != b' ' && c2 != b'\t' {
                        return false;
                    }
                    eatws(line, &mut p);
                    // objparser.cpp:41 — double u = strtod(line, &endptr);
                    let (u, endptr) = strtod(line, p);
                    // objparser.cpp:42 — endptr == 0 cannot happen with strtol/strtod.
                    if at(line, endptr) != b' ' && at(line, endptr) != b'\t' {
                        return false;
                    }
                    p = endptr;
                    eatws(line, &mut p);
                    // objparser.cpp:46-53
                    let mut v = 0.0f64;
                    if at(line, p) != 0 {
                        let (vv, endptr) = strtod(line, p);
                        let e = at(line, endptr);
                        if e != b' ' && e != b'\t' && e != 0 {
                            return false;
                        }
                        v = vv;
                        p = endptr;
                        eatws(line, &mut p);
                    }
                    /* objparser.cpp:54-61 — (commented-out w parse)
                    double w = 0;
                    if (*line != 0) {
                        w = strtod(line, &endptr);
                        ...
                    } */
                    // objparser.cpp:62-63
                    if at(line, p) != 0 {
                        return false;
                    }
                    // objparser.cpp:64-65
                    data.texture_coordinates.push(u as f32);
                    data.texture_coordinates.push(v as f32);
                    // objparser.cpp:66 — //data.textureCoordinates.push_back((float)w);
                }
                // objparser.cpp:69-99
                b'n' => {
                    // vn - vertex normal
                    // x y z
                    // objparser.cpp:73 — char c2 = *line ++;
                    let c2 = at(line, p);
                    p += 1;
                    if c2 != b' ' && c2 != b'\t' {
                        return false;
                    }
                    eatws(line, &mut p);
                    // objparser.cpp:78-81
                    let (x, endptr) = strtod(line, p);
                    if at(line, endptr) != b' ' && at(line, endptr) != b'\t' {
                        return false;
                    }
                    p = endptr;
                    eatws(line, &mut p);
                    // objparser.cpp:83-86
                    let (y, endptr) = strtod(line, p);
                    if at(line, endptr) != b' ' && at(line, endptr) != b'\t' {
                        return false;
                    }
                    p = endptr;
                    eatws(line, &mut p);
                    // objparser.cpp:88-91
                    let (z, endptr) = strtod(line, p);
                    let e = at(line, endptr);
                    if e != b' ' && e != b'\t' && e != 0 {
                        return false;
                    }
                    p = endptr;
                    eatws(line, &mut p);
                    // objparser.cpp:93-94
                    if at(line, p) != 0 {
                        return false;
                    }
                    // objparser.cpp:95-97
                    data.normals.push(x as f32);
                    data.normals.push(y as f32);
                    data.normals.push(z as f32);
                }
                // objparser.cpp:100-132
                b'p' => {
                    // vp - vertex parameter
                    // objparser.cpp:103 — char c2 = *line ++;
                    let c2 = at(line, p);
                    p += 1;
                    if c2 != b' ' && c2 != b'\t' {
                        return false;
                    }
                    eatws(line, &mut p);
                    // objparser.cpp:108-111
                    let (u, endptr) = strtod(line, p);
                    let e = at(line, endptr);
                    if e != b' ' && e != b'\t' && e != 0 {
                        return false;
                    }
                    p = endptr;
                    eatws(line, &mut p);
                    // objparser.cpp:113-116
                    let (v, endptr) = strtod(line, p);
                    let e = at(line, endptr);
                    if e != b' ' && e != b'\t' && e != 0 {
                        return false;
                    }
                    p = endptr;
                    eatws(line, &mut p);
                    // objparser.cpp:118-125
                    let mut w = 0.0f64;
                    if at(line, p) != 0 {
                        let (ww, endptr) = strtod(line, p);
                        let e = at(line, endptr);
                        if e != b' ' && e != b'\t' && e != 0 {
                            return false;
                        }
                        w = ww;
                        p = endptr;
                        eatws(line, &mut p);
                    }
                    // objparser.cpp:126-127
                    if at(line, p) != 0 {
                        return false;
                    }
                    // objparser.cpp:128-130
                    data.parameters.push(u as f32);
                    data.parameters.push(v as f32);
                    data.parameters.push(w as f32);
                }
                // objparser.cpp:133-196
                _ => {
                    // v - vertex geometry
                    if c2 != b' ' && c2 != b'\t' {
                        return false;
                    }
                    eatws(line, &mut p);
                    // objparser.cpp:140-143
                    let (x, endptr) = strtod(line, p);
                    if at(line, endptr) != b' ' && at(line, endptr) != b'\t' {
                        return false;
                    }
                    p = endptr;
                    eatws(line, &mut p);
                    // objparser.cpp:145-148
                    let (y, endptr) = strtod(line, p);
                    if at(line, endptr) != b' ' && at(line, endptr) != b'\t' {
                        return false;
                    }
                    p = endptr;
                    eatws(line, &mut p);
                    // objparser.cpp:150-153
                    let (z, endptr) = strtod(line, p);
                    let e = at(line, endptr);
                    if e != b' ' && e != b'\t' && e != 0 {
                        return false;
                    }
                    p = endptr;
                    eatws(line, &mut p);
                    // objparser.cpp:155 — undefine color
                    let mut color_x = 0.0f64;
                    let mut color_y = 0.0f64;
                    let mut color_z = 0.0f64;
                    let mut color_w = 0.0f64;
                    // objparser.cpp:156-182
                    if at(line, p) != 0 {
                        if !data.has_vertex_color {
                            data.has_vertex_color = true;
                        }
                        // objparser.cpp:160-163
                        let (cx, endptr) = strtod(line, p);
                        let e = at(line, endptr);
                        if e != b' ' && e != b'\t' && e != 0 {
                            return false;
                        }
                        color_x = cx;
                        p = endptr;
                        eatws(line, &mut p);
                        // objparser.cpp:165-168
                        let (cy, endptr) = strtod(line, p);
                        let e = at(line, endptr);
                        if e != b' ' && e != b'\t' && e != 0 {
                            return false;
                        }
                        color_y = cy;
                        p = endptr;
                        eatws(line, &mut p);
                        // objparser.cpp:170-173
                        let (cz, endptr) = strtod(line, p);
                        let e = at(line, endptr);
                        if e != b' ' && e != b'\t' && e != 0 {
                            return false;
                        }
                        color_z = cz;
                        p = endptr;
                        eatws(line, &mut p);
                        // objparser.cpp:175 — default define alpha = 1.0
                        color_w = 1.0;
                        // objparser.cpp:176-181
                        if at(line, p) != 0 {
                            let (cw, endptr) = strtod(line, p);
                            let e = at(line, endptr);
                            if e != b' ' && e != b'\t' && e != 0 {
                                return false;
                            }
                            color_w = cw;
                            p = endptr;
                            eatws(line, &mut p);
                        }
                    }
                    // objparser.cpp:183-187 — the following check is commented out because
                    // there may be obj files containing extra data, as those generated by
                    // Meshlab, see https://dev.prusa3d.com/browse/SPE-1019 for an example,
                    // and this would lead to a crash because no vertex would be stored
                    // if (*line != 0) return false;
                    // objparser.cpp:188-194
                    data.coordinates.push(x as f32);
                    data.coordinates.push(y as f32);
                    data.coordinates.push(z as f32);
                    data.coordinates.push(color_x as f32);
                    data.coordinates.push(color_y as f32);
                    data.coordinates.push(color_z as f32);
                    data.coordinates.push(color_w as f32);
                }
            }
        }
        // objparser.cpp:200-277
        b'f' => {
            // face
            eatws(line, &mut p);
            // objparser.cpp:204-205
            if at(line, p) == 0 {
                return false;
            }
            // current vertex to be parsed (objparser.cpp:208)
            let mut vertex;
            // objparser.cpp:210
            while at(line, p) != 0 {
                // Parse a single vertex reference.
                // objparser.cpp:212-214
                vertex = ObjVertex {
                    coord_idx: 0,
                    normal_idx: 0,
                    texture_coord_idx: 0,
                };
                // objparser.cpp:215
                let (ci, endptr) = strtol(line, p);
                vertex.coord_idx = ci as i32;
                // Coordinate has to be defined
                // objparser.cpp:217-218
                let e = at(line, endptr);
                if e != b' ' && e != b'\t' && e != b'/' && e != 0 {
                    return false;
                }
                p = endptr;
                // objparser.cpp:220-238
                if at(line, p) == b'/' {
                    p += 1;
                    // Texture coordinate index may be missing after a 1st slash, but then
                    // the normal index has to be present.
                    if at(line, p) != b'/' {
                        // Parse the texture coordinate index.
                        // objparser.cpp:225-228
                        let (ti, endptr) = strtol(line, p);
                        vertex.texture_coord_idx = ti as i32;
                        let e = at(line, endptr);
                        if e != b' ' && e != b'\t' && e != b'/' && e != 0 {
                            return false;
                        }
                        p = endptr;
                    }
                    if at(line, p) == b'/' {
                        // Parse normal index.
                        // objparser.cpp:232-236
                        p += 1;
                        let (ni, endptr) = strtol(line, p);
                        vertex.normal_idx = ni as i32;
                        let e = at(line, endptr);
                        if e != b' ' && e != b'\t' && e != 0 {
                            return false;
                        }
                        p = endptr;
                    }
                }
                // objparser.cpp:239-242
                if vertex.coord_idx < 0 {
                    vertex.coord_idx += (data.coordinates.len() as i32) / OBJ_VERTEX_LENGTH as i32;
                } else {
                    vertex.coord_idx -= 1;
                }
                // objparser.cpp:243-246
                if vertex.normal_idx < 0 {
                    vertex.normal_idx += (data.normals.len() as i32) / 3;
                } else {
                    vertex.normal_idx -= 1;
                }
                // objparser.cpp:247-250 — note: textureCoordinates are stored 2 floats
                // per entry but C++ divides by 3; the quirk is preserved.
                if vertex.texture_coord_idx < 0 {
                    vertex.texture_coord_idx += (data.texture_coordinates.len() as i32) / 3;
                } else {
                    vertex.texture_coord_idx -= 1;
                }
                // objparser.cpp:251-252
                data.vertices.push(vertex);
                eatws(line, &mut p);
            }
            // objparser.cpp:254-256
            if !data.usemtls.is_empty() {
                let n = data.vertices.len() as i32;
                data.usemtls.last_mut().unwrap().vertex_idx_end = n;
            }
            // objparser.cpp:257-271
            if !data.usemtls.is_empty() {
                let mut face_index_count = 0i32;
                // objparser.cpp:259-264
                let mut i = data.vertices.len() as i64 - 1;
                while i >= 0 {
                    if data.vertices[i as usize].coord_idx == -1 {
                        break;
                    }
                    face_index_count += 1;
                    i -= 1;
                }
                if face_index_count == 3 {
                    // tri (objparser.cpp:265-266)
                    data.usemtls.last_mut().unwrap().face_end += 1;
                } else if face_index_count == 4 {
                    // quad (objparser.cpp:267-270)
                    data.usemtls.last_mut().unwrap().face_end += 1;
                    data.usemtls.last_mut().unwrap().face_end += 1;
                }
            }
            // objparser.cpp:272-275 — face delimiter
            data.vertices.push(ObjVertex {
                coord_idx: -1,
                normal_idx: -1,
                texture_coord_idx: -1,
            });
        }
        // objparser.cpp:278-291
        b'm' => {
            // objparser.cpp:280-285
            for &ch in b"tllib" {
                let c = at(line, p);
                p += 1;
                if c != ch {
                    return false;
                }
            }
            // mtllib [external .mtl file name]
            // printf("mtllib %s\r\n", line);
            eatws(line, &mut p);
            // objparser.cpp:289
            data.mtllibs.push(lossy(cstr_tail(line, p)));
        }
        // objparser.cpp:292-321
        b'u' => {
            // objparser.cpp:294-299
            for &ch in b"semtl" {
                let c = at(line, p);
                p += 1;
                if c != ch {
                    return false;
                }
            }
            // usemtl [material name]
            // printf("usemtl %s\r\n", line);
            eatws(line, &mut p);
            // objparser.cpp:303-305
            if !data.usemtls.is_empty() {
                let n = data.vertices.len() as i32;
                data.usemtls.last_mut().unwrap().vertex_idx_end = n;
            }
            // objparser.cpp:306-309
            let mut usemtl = ObjUseMtl::new();
            usemtl.vertex_idx_first = data.vertices.len() as i32;
            usemtl.name = lossy(cstr_tail(line, p));
            data.usemtls.push(usemtl);
            // objparser.cpp:310-318
            if data.usemtls.len() == 1 {
                data.usemtls.last_mut().unwrap().face_start = 0;
            } else {
                // >= 2
                let count = data.usemtls.len();
                let last_last_face_end = data.usemtls[count - 2].face_end;
                data.usemtls[count - 1].face_start = last_last_face_end + 1;
            }
            // objparser.cpp:319
            let last = data.usemtls.last_mut().unwrap();
            last.face_end = last.face_start - 1;
        }
        // objparser.cpp:322-337
        b'o' => {
            // o [object name]
            eatws(line, &mut p);
            // objparser.cpp:326-327
            while at(line, p) != b' ' && at(line, p) != b'\t' && at(line, p) != 0 {
                p += 1;
            }
            // copy name to line.
            eatws(line, &mut p);
            // objparser.cpp:330-331
            if at(line, p) != 0 {
                return false;
            }
            // objparser.cpp:332-335 — note: `line` points at the terminating NUL here,
            // so the stored name is always empty (faithful to the C++).
            let object = ObjObject {
                vertex_idx_first: data.vertices.len() as i32,
                name: lossy(cstr_tail(line, p)),
            };
            data.objects.push(object);
        }
        // objparser.cpp:338-347
        b'g' => {
            // g [group name]
            // printf("group %s\r\n", line);
            // objparser.cpp:342-344 — note: no EATWS, the name keeps the leading
            // whitespace after the 'g' (faithful to the C++).
            let group = ObjGroup {
                vertex_idx_first: data.vertices.len() as i32,
                name: lossy(cstr_tail(line, p)),
            };
            data.groups.push(group);
        }
        // objparser.cpp:348-368
        b's' => {
            // s 1 / off
            // objparser.cpp:351 — char c2 = *line ++;
            let c2 = at(line, p);
            p += 1;
            if c2 != b' ' && c2 != b'\t' {
                return false;
            }
            eatws(line, &mut p);
            // objparser.cpp:356-358 — note: "s off" yields no strtol conversion, so
            // *endptr == 'o' and the line is rejected (faithful to the C++).
            let (g, endptr) = strtol(line, p);
            let e = at(line, endptr);
            if e != b' ' && e != b'\t' && e != 0 {
                return false;
            }
            p = endptr;
            eatws(line, &mut p);
            // objparser.cpp:361-362
            if at(line, p) != 0 {
                return false;
            }
            // objparser.cpp:363-366
            let group = ObjSmoothingGroup {
                vertex_idx_first: data.vertices.len() as i32,
                smoothing_group_id: g as i32,
            };
            data.smoothing_groups.push(group);
        }
        // objparser.cpp:369-371
        _ => {
            error!("ObjParser: Unknown command: {}", c1 as char);
        }
    }

    true
}

// ---------------------------------------------------------------------------
// mtl_parseline  (objparser.cpp:376-577)
// ---------------------------------------------------------------------------

// objparser.cpp:376 — `static std::string cur_mtl_name = "";`
// The C++ file-scope static is threaded through as a parameter instead of a
// global; `mtlparse` resets it to "" before parsing, so behaviour is identical.

/// Mirrors the repeated x/y/z parse blocks inside `mtl_parseline` (e.g.
/// objparser.cpp:440-453): EATWS, then three strtod calls with the C++ endptr
/// delimiter checks (x: {' ','\t'}, y: {' ','\t'}, z: {' ','\t',0}).
fn mtl_parse_xyz(line: &[u8], mut p: usize) -> Option<(f64, f64, f64)> {
    eatws(line, &mut p);
    let (x, endptr) = strtod(line, p);
    if at(line, endptr) != b' ' && at(line, endptr) != b'\t' {
        return None;
    }
    p = endptr;
    eatws(line, &mut p);
    let (y, endptr) = strtod(line, p);
    if at(line, endptr) != b' ' && at(line, endptr) != b'\t' {
        return None;
    }
    p = endptr;
    eatws(line, &mut p);
    let (z, endptr) = strtod(line, p);
    let e = at(line, endptr);
    if e != b' ' && e != b'\t' && e != 0 {
        return None;
    }
    p = endptr;
    eatws(line, &mut p);
    let _ = p;
    Some((x, y, z))
}

/// objparser.cpp:377 — `static bool mtl_parseline(const char *line, MtlData &data)`
fn mtl_parseline(line: &[u8], data: &mut MtlData, cur_mtl_name: &mut String) -> bool {
    let mut p = 0usize;
    // objparser.cpp:379
    if at(line, p) == 0 {
        return true;
    }
    // objparser.cpp:380 — assert(Slic3r::is_decimal_separator_point());
    debug_assert!(is_decimal_separator_point());
    // Ignore whitespaces at the beginning of the line.
    // FIXME is this a good idea?
    // objparser.cpp:383
    eatws(line, &mut p);

    // objparser.cpp:385 — char c1 = *line++;
    let c1 = at(line, p);
    p += 1;
    match c1 {
        // objparser.cpp:387-399 — Comment, ignore the rest of the line.
        b'#' => {
            // "First" "Time" "Using" "MakerLab" checked char by char in the C++.
            if cstr_tail(line, p).starts_with(b"FirstTimeUsingMakerLab") {
                data.first_time_using_makerlab = true;
            }
        }
        // objparser.cpp:400-409
        b'n' => {
            // objparser.cpp:401-402
            for &ch in b"ewmtl" {
                let c = at(line, p);
                p += 1;
                if c != ch {
                    return false;
                }
            }
            eatws(line, &mut p);
            // objparser.cpp:404 — `ObjNewMtl new_mtl;` (unused in the C++)
            // objparser.cpp:405-407
            *cur_mtl_name = lossy(cstr_tail(line, p));
            data.new_mtl_unmap
                .insert(cur_mtl_name.clone(), Arc::new(ObjNewMtl::new()));
            data.mtl_orders.push(cur_mtl_name.clone());
        }
        // objparser.cpp:410-417
        b'm' => {
            // objparser.cpp:411
            for &ch in b"ap_Kd" {
                let c = at(line, p);
                p += 1;
                if c != ch {
                    return false;
                }
            }
            eatws(line, &mut p);
            // objparser.cpp:413-415
            if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name.as_str()) {
                Arc::make_mut(mtl).map_kd = lossy(cstr_tail(line, p));
            }
        }
        // objparser.cpp:418-436
        b'N' => {
            // objparser.cpp:419 — char cur_char = *(line++);
            let cur_char = at(line, p);
            p += 1;
            if cur_char == b's' {
                eatws(line, &mut p);
                // objparser.cpp:423 — no endptr validation: a failed parse stores 0.
                let (ns, _endptr) = strtod(line, p);
                if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name.as_str()) {
                    Arc::make_mut(mtl).ns = ns as f32;
                }
            } else if cur_char == b'i' {
                eatws(line, &mut p);
                // objparser.cpp:430
                let (ni, _endptr) = strtod(line, p);
                if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name.as_str()) {
                    Arc::make_mut(mtl).ni = ni as f32;
                }
            }
        }
        // objparser.cpp:437-521
        b'K' => {
            // objparser.cpp:438 — char cur_char = *(line++);
            let cur_char = at(line, p);
            p += 1;
            if cur_char == b'a' {
                // objparser.cpp:440-458
                let (x, y, z) = match mtl_parse_xyz(line, p) {
                    Some(v) => v,
                    None => return false,
                };
                if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name.as_str()) {
                    let mtl = Arc::make_mut(mtl);
                    mtl.ka[0] = x as f32;
                    mtl.ka[1] = y as f32;
                    mtl.ka[2] = z as f32;
                }
            } else if cur_char == b'd' {
                // objparser.cpp:459-478
                let (x, y, z) = match mtl_parse_xyz(line, p) {
                    Some(v) => v,
                    None => return false,
                };
                if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name.as_str()) {
                    let mtl = Arc::make_mut(mtl);
                    mtl.kd[0] = x as f32;
                    mtl.kd[1] = y as f32;
                    mtl.kd[2] = z as f32;
                }
            } else if cur_char == b's' {
                // objparser.cpp:479-498
                let (x, y, z) = match mtl_parse_xyz(line, p) {
                    Some(v) => v,
                    None => return false,
                };
                if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name.as_str()) {
                    let mtl = Arc::make_mut(mtl);
                    mtl.ks[0] = x as f32;
                    mtl.ks[1] = y as f32;
                    mtl.ks[2] = z as f32;
                }
            } else if cur_char == b'e' {
                // objparser.cpp:499-519
                let (x, y, z) = match mtl_parse_xyz(line, p) {
                    Some(v) => v,
                    None => return false,
                };
                if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name.as_str()) {
                    let mtl = Arc::make_mut(mtl);
                    mtl.ke[0] = x as f32;
                    mtl.ke[1] = y as f32;
                    mtl.ke[2] = z as f32;
                }
            }
        }
        // objparser.cpp:522-532
        b'i' => {
            // objparser.cpp:523-524
            for &ch in b"llum" {
                let c = at(line, p);
                p += 1;
                if c != ch {
                    return false;
                }
            }
            eatws(line, &mut p);
            // objparser.cpp:527 — no endptr validation: a failed parse stores 0.
            let (illum, _endptr) = strtod(line, p);
            if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name.as_str()) {
                Arc::make_mut(mtl).illum = illum as f32;
            }
        }
        // objparser.cpp:533-541
        b'd' => {
            eatws(line, &mut p);
            // objparser.cpp:536 — no endptr validation: a failed parse stores 0.
            let (d, _endptr) = strtod(line, p);
            if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name.as_str()) {
                Arc::make_mut(mtl).d = d as f32;
            }
        }
        // objparser.cpp:542-574
        b'T' => {
            // objparser.cpp:543 — char cur_char = *(line++);
            let cur_char = at(line, p);
            p += 1;
            if cur_char == b'r' {
                eatws(line, &mut p);
                // objparser.cpp:547 — no endptr validation: a failed parse stores 1.0
                // through the range check below.
                let (tr, _endptr) = strtod(line, p);
                if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name.as_str()) {
                    // objparser.cpp:549
                    Arc::make_mut(mtl).tr = if tr > 0.0 && tr <= 1.0 { tr as f32 } else { 1.0 };
                }
            } else if cur_char == b'f' {
                // objparser.cpp:552-572
                let (x, y, z) = match mtl_parse_xyz(line, p) {
                    Some(v) => v,
                    None => return false,
                };
                if let Some(mtl) = data.new_mtl_unmap.get_mut(cur_mtl_name.as_str()) {
                    let mtl = Arc::make_mut(mtl);
                    mtl.tf[0] = x as f32;
                    mtl.tf[1] = y as f32;
                    mtl.tf[2] = z as f32;
                }
            }
        }
        // The C++ switch has no default case: unknown commands are ignored.
        _ => {}
    }
    true
}

// ---------------------------------------------------------------------------
// Chunked file readers  (objparser.cpp:579-729)
// ---------------------------------------------------------------------------

/// `::fread(ptr, 1, n, stream)` emulation: keeps reading until `buf` is full
/// or EOF/error, returning the number of bytes read.
fn fread(file: &mut dyn Read, buf: &mut [u8]) -> usize {
    let mut total = 0usize;
    while total < buf.len() {
        match file.read(&mut buf[total..]) {
            Ok(0) => break,
            Ok(n) => total += n,
            Err(_) => break,
        }
    }
    total
}

/// objparser.cpp:579 — `bool objparse(const char *path, ObjData &data)`
pub fn objparse(path: &Path, data: &mut ObjData) -> bool {
    // objparser.cpp:581 — Slic3r::CNumericLocalesSetter locales_setter;
    let _locales_setter = CNumericLocalesSetter::new();

    // objparser.cpp:583-585
    let mut file = match File::open(path) {
        Ok(f) => f,
        Err(_) => return false,
    };

    // objparser.cpp:588-591
    let mut buf = vec![0u8; 65536 * 2];
    let mut len: usize;
    let mut len_prev = 0usize;
    let mut line_count = 0usize;

    // objparser.cpp:593
    loop {
        len = fread(&mut file, &mut buf[len_prev..len_prev + 65536]);
        if len == 0 {
            break;
        }
        // objparser.cpp:594
        len += len_prev;
        let mut last_line = 0usize;
        // objparser.cpp:596-613
        for i in 0..len {
            if buf[i] == b'\r' || buf[i] == b'\n' {
                buf[i] = 0;
                // objparser.cpp:599-601 — char *c = buf + lastLine; skip ' '/'\t'.
                let mut c = last_line;
                while c < i && (buf[c] == b' ' || buf[c] == b'\t') {
                    c += 1;
                }
                let line = &buf[c..i];
                // FIXME check the return value and exit on error?
                // Will it break parsing of some obj files?
                obj_parseline(line, data);

                /* for ml (objparser.cpp:606-609) */
                if line_count == 0 {
                    data.ml_region = parsemlinfo_bytes(line, b"region:");
                }
                if line_count == 1 {
                    data.ml_name = parsemlinfo_bytes(line, b"ml_name:");
                }
                if line_count == 2 {
                    data.ml_id = parsemlinfo_bytes(line, b"ml_file_id:");
                }

                line_count += 1;
                last_line = i + 1;
            }
        }
        // objparser.cpp:614-620
        len_prev = len - last_line;
        if len_prev > 65536 {
            error!("ObjParser: Excessive line length");
            return false;
        }
        buf.copy_within(last_line..len, 0);
    }
    // objparser.cpp:623-625 — catch (std::bad_alloc&): Rust aborts on OOM instead.
    // objparser.cpp:626-627
    true
}

/// objparser.cpp:630 — `std::string parsemlinfo(const char* input, const char* condition)`
pub fn parsemlinfo(input: &str, condition: &str) -> String {
    parsemlinfo_bytes(input.as_bytes(), condition.as_bytes())
}

/// Byte-level body of `parsemlinfo` (objparser.cpp:630-649); the inputs are C
/// strings, so they are truncated at the first NUL.
fn parsemlinfo_bytes(input: &[u8], condition: &[u8]) -> String {
    let input = &input[..input.iter().position(|&b| b == 0).unwrap_or(input.len())];
    // objparser.cpp:631 — const char* regionPtr = std::strstr(input, condition);
    let region_ptr = find_subslice(input, condition);

    // objparser.cpp:633
    let mut region_content = String::new();

    if let Some(pos) = region_ptr {
        // objparser.cpp:636
        let mut region_ptr = pos + condition.len();

        // objparser.cpp:638-640
        while region_ptr < input.len()
            && (input[region_ptr] == b' ' || input[region_ptr] == b'\t')
        {
            region_ptr += 1;
        }

        // objparser.cpp:642-643
        let rest = &input[region_ptr..];
        let length = rest.iter().position(|&b| b == b'\n').unwrap_or(rest.len());

        // objparser.cpp:645
        region_content = lossy(&rest[..length]);
    }

    // objparser.cpp:648
    region_content
}

/// `std::strstr` over byte slices.
fn find_subslice(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    if needle.is_empty() {
        return Some(0);
    }
    if needle.len() > haystack.len() {
        return None;
    }
    haystack.windows(needle.len()).position(|w| w == needle)
}

/// objparser.cpp:652 — `bool mtlparse(const char *path, MtlData &data)`
pub fn mtlparse(path: &Path, data: &mut MtlData) -> bool {
    // objparser.cpp:654 — Slic3r::CNumericLocalesSetter locales_setter;
    let _locales_setter = CNumericLocalesSetter::new();

    // objparser.cpp:656-657
    let mut file = match File::open(path) {
        Ok(f) => f,
        Err(_) => return false,
    };
    // objparser.cpp:658 — cur_mtl_name = "";
    let mut cur_mtl_name = String::new();
    // objparser.cpp:660-662
    let mut buf = vec![0u8; 65536 * 2];
    let mut len: usize;
    let mut len_prev = 0usize;
    // objparser.cpp:663
    loop {
        len = fread(&mut file, &mut buf[len_prev..len_prev + 65536]);
        if len == 0 {
            break;
        }
        len += len_prev;
        let mut last_line = 0usize;
        // objparser.cpp:666-675
        for i in 0..len {
            if buf[i] == b'\r' || buf[i] == b'\n' {
                buf[i] = 0;
                let mut c = last_line;
                while c < i && (buf[c] == b' ' || buf[c] == b'\t') {
                    c += 1;
                }
                // FIXME check the return value and exit on error?
                // Will it break parsing of some obj files?
                mtl_parseline(&buf[c..i], data, &mut cur_mtl_name);
                last_line = i + 1;
            }
        }
        // objparser.cpp:676-682
        len_prev = len - last_line;
        if len_prev > 65536 {
            error!("MtlParser: Excessive line length");
            return false;
        }
        buf.copy_within(last_line..len, 0);
    }
    // objparser.cpp:684-686 — catch (std::bad_alloc&): Rust aborts on OOM instead.
    // objparser.cpp:687-688
    true
}

/// objparser.cpp:691 — `bool objparse(std::istream &stream, ObjData &data)`
///
/// Rust cannot overload `objparse`, hence the `_stream` suffix.
pub fn objparse_stream<R: Read>(reader: R, data: &mut ObjData) -> bool {
    // objparser.cpp:693 — Slic3r::CNumericLocalesSetter locales_setter;
    let _locales_setter = CNumericLocalesSetter::new();

    let mut stream = reader;
    // objparser.cpp:696-698
    let mut buf = vec![0u8; 65536 * 2];
    let mut len: usize;
    let mut len_prev = 0usize;
    // objparser.cpp:699
    loop {
        // The C++ has no excessive-line check here; with a >128K unterminated
        // line it would overflow its stack buffer (UB). We clamp the read
        // window instead, which simply stops consuming input at that point.
        let window_end = (len_prev + 65536).min(buf.len());
        len = fread(&mut stream, &mut buf[len_prev..window_end]);
        if len == 0 {
            break;
        }
        len += len_prev;
        let mut last_line = 0usize;
        // objparser.cpp:702-718
        for i in 0..len {
            if buf[i] == b'\r' || buf[i] == b'\n' {
                buf[i] = 0;
                let mut c = last_line;
                while c < i && (buf[c] == b' ' || buf[c] == b'\t') {
                    c += 1;
                }
                let line = &buf[c..i];
                obj_parseline(line, data);

                /* for ml (objparser.cpp:711-715) — note: the C++ tests `lastLine < 3`
                (a byte offset within the buffer), not the line count. */
                if last_line < 3 {
                    data.ml_region = parsemlinfo_bytes(line, b"region");
                    data.ml_name = parsemlinfo_bytes(line, b"ml_name");
                    data.ml_id = parsemlinfo_bytes(line, b"ml_file_id");
                }

                last_line = i + 1;
            }
        }
        // objparser.cpp:719-720
        len_prev = len - last_line;
        buf.copy_within(last_line..len, 0);
    }
    // objparser.cpp:723-726 — catch (std::bad_alloc&) returns false; Rust aborts on OOM.
    true
}

// ---------------------------------------------------------------------------
// Binary save  (objparser.cpp:731-766)
// ---------------------------------------------------------------------------
//
// The binary format mirrors the C++ byte for byte on the same target:
// `size_t` counts/lengths are written as native-endian `usize` (8 bytes on
// 64-bit, 4 on wasm32), elements as their raw native-endian bytes. The C++
// ignores all `fwrite` return values, so write errors are ignored here too.

/// objparser.cpp:731-740 — `template<typename T> bool savevector(FILE*, const std::vector<T>&)`
/// instantiated for `float`.
fn savevector_f32(f: &mut dyn Write, v: &[f32]) -> bool {
    let cnt = v.len();
    let _ = f.write_all(&cnt.to_ne_bytes());
    // FIXME sizeof(T) works for data types leaving no gaps in the allocated
    // vector because of alignment of the T type.
    if !v.is_empty() {
        for x in v {
            let _ = f.write_all(&x.to_ne_bytes());
        }
    }
    true
}

/// objparser.cpp:731-740 instantiated for `ObjSmoothingGroup` (two `int`s).
fn savevector_smoothing(f: &mut dyn Write, v: &[ObjSmoothingGroup]) -> bool {
    let cnt = v.len();
    let _ = f.write_all(&cnt.to_ne_bytes());
    if !v.is_empty() {
        for g in v {
            let _ = f.write_all(&g.vertex_idx_first.to_ne_bytes());
            let _ = f.write_all(&g.smoothing_group_id.to_ne_bytes());
        }
    }
    true
}

/// objparser.cpp:731-740 instantiated for `ObjVertex` (three `int`s, in
/// declaration order: coordIdx, textureCoordIdx, normalIdx).
fn savevector_vertex(f: &mut dyn Write, v: &[ObjVertex]) -> bool {
    let cnt = v.len();
    let _ = f.write_all(&cnt.to_ne_bytes());
    if !v.is_empty() {
        for x in v {
            let _ = f.write_all(&x.coord_idx.to_ne_bytes());
            let _ = f.write_all(&x.texture_coord_idx.to_ne_bytes());
            let _ = f.write_all(&x.normal_idx.to_ne_bytes());
        }
    }
    true
}

/// objparser.cpp:742-752 — `bool savevector(FILE*, const std::vector<std::string>&)`
fn savevector_string(f: &mut dyn Write, v: &[String]) -> bool {
    let cnt = v.len();
    let _ = f.write_all(&cnt.to_ne_bytes());
    for s in v {
        let len = s.len();
        let _ = f.write_all(&len.to_ne_bytes());
        let _ = f.write_all(s.as_bytes());
    }
    true
}

/// objparser.cpp:754-766 — `template<typename T> bool savevectornameidx(...)`:
/// writes only `vertexIdxFirst` and `name` for each element.
fn savevectornameidx<T>(f: &mut dyn Write, v: &[T], get: fn(&T) -> (i32, &str)) -> bool {
    let cnt = v.len();
    let _ = f.write_all(&cnt.to_ne_bytes());
    for item in v {
        let (vertex_idx_first, name) = get(item);
        let _ = f.write_all(&vertex_idx_first.to_ne_bytes());
        let len = name.len();
        let _ = f.write_all(&len.to_ne_bytes());
        let _ = f.write_all(name.as_bytes());
    }
    true
}

// ---------------------------------------------------------------------------
// Binary load  (objparser.cpp:768-822)
// ---------------------------------------------------------------------------
//
// Where the C++ pre-allocates `cnt` elements (`v.assign(cnt, T())`) before
// `fread` — which would throw out of `objbinload` for a corrupt count — the
// Rust port reads element by element and returns false on a short read.

fn read_usize(r: &mut dyn Read) -> Option<usize> {
    let mut buf = [0u8; std::mem::size_of::<usize>()];
    r.read_exact(&mut buf).ok()?;
    Some(usize::from_ne_bytes(buf))
}

fn read_i32(r: &mut dyn Read) -> Option<i32> {
    let mut buf = [0u8; 4];
    r.read_exact(&mut buf).ok()?;
    Some(i32::from_ne_bytes(buf))
}

fn read_f32(r: &mut dyn Read) -> Option<f32> {
    let mut buf = [0u8; 4];
    r.read_exact(&mut buf).ok()?;
    Some(f32::from_ne_bytes(buf))
}

/// Read `len` raw bytes (bounded by what the stream actually holds).
fn read_bytes(r: &mut dyn Read, len: usize) -> Option<Vec<u8>> {
    let mut buf = Vec::new();
    r.take(len as u64).read_to_end(&mut buf).ok()?;
    if buf.len() != len {
        return None;
    }
    Some(buf)
}

/// objparser.cpp:768-782 — `template<typename T> bool loadvector(FILE*, std::vector<T>&)`
/// instantiated for `float`.
fn loadvector_f32(r: &mut dyn Read, v: &mut Vec<f32>) -> bool {
    v.clear();
    let cnt = match read_usize(r) {
        Some(c) => c,
        None => return false,
    };
    // FIXME sizeof(T) works for data types leaving no gaps in the allocated
    // vector because of alignment of the T type.
    for _ in 0..cnt {
        match read_f32(r) {
            Some(x) => v.push(x),
            None => return false,
        }
    }
    true
}

/// objparser.cpp:768-782 instantiated for `ObjSmoothingGroup`.
fn loadvector_smoothing(r: &mut dyn Read, v: &mut Vec<ObjSmoothingGroup>) -> bool {
    v.clear();
    let cnt = match read_usize(r) {
        Some(c) => c,
        None => return false,
    };
    for _ in 0..cnt {
        let vertex_idx_first = match read_i32(r) {
            Some(x) => x,
            None => return false,
        };
        let smoothing_group_id = match read_i32(r) {
            Some(x) => x,
            None => return false,
        };
        v.push(ObjSmoothingGroup {
            vertex_idx_first,
            smoothing_group_id,
        });
    }
    true
}

/// objparser.cpp:768-782 instantiated for `ObjVertex`.
fn loadvector_vertex(r: &mut dyn Read, v: &mut Vec<ObjVertex>) -> bool {
    v.clear();
    let cnt = match read_usize(r) {
        Some(c) => c,
        None => return false,
    };
    for _ in 0..cnt {
        let coord_idx = match read_i32(r) {
            Some(x) => x,
            None => return false,
        };
        let texture_coord_idx = match read_i32(r) {
            Some(x) => x,
            None => return false,
        };
        let normal_idx = match read_i32(r) {
            Some(x) => x,
            None => return false,
        };
        v.push(ObjVertex {
            coord_idx,
            texture_coord_idx,
            normal_idx,
        });
    }
    true
}

/// objparser.cpp:784-801 — `bool loadvector(FILE*, std::vector<std::string>&)`
fn loadvector_string(r: &mut dyn Read, v: &mut Vec<String>) -> bool {
    v.clear();
    let cnt = match read_usize(r) {
        Some(c) => c,
        None => return false,
    };
    for _ in 0..cnt {
        let len = match read_usize(r) {
            Some(l) => l,
            None => return false,
        };
        // objparser.cpp:795-797 — std::string s(" ", len) then fread into it.
        let bytes = match read_bytes(r, len) {
            Some(b) => b,
            None => return false,
        };
        v.push(lossy(&bytes));
    }
    true
}

/// objparser.cpp:803-822 — `template<typename T> bool loadvectornameidx(...)`:
/// reads `vertexIdxFirst` and `name`, leaving the other members default.
fn loadvectornameidx<T>(
    r: &mut dyn Read,
    v: &mut Vec<T>,
    make: impl Fn(i32, String) -> T,
) -> bool {
    v.clear();
    let cnt = match read_usize(r) {
        Some(c) => c,
        None => return false,
    };
    for _ in 0..cnt {
        let vertex_idx_first = match read_i32(r) {
            Some(x) => x,
            None => return false,
        };
        let len = match read_usize(r) {
            Some(l) => l,
            None => return false,
        };
        let bytes = match read_bytes(r, len) {
            Some(b) => b,
            None => return false,
        };
        v.push(make(vertex_idx_first, lossy(&bytes)));
    }
    true
}

// Named extractors for `savevectornameidx` (the C++ template reads
// `v[i].vertexIdxFirst` and `v[i].name` directly, objparser.cpp:760-763).
fn usemtl_nameidx(m: &ObjUseMtl) -> (i32, &str) {
    (m.vertex_idx_first, m.name.as_str())
}
fn object_nameidx(o: &ObjObject) -> (i32, &str) {
    (o.vertex_idx_first, o.name.as_str())
}
fn group_nameidx(g: &ObjGroup) -> (i32, &str) {
    (g.vertex_idx_first, g.name.as_str())
}

/// objparser.cpp:824 — `bool objbinsave(const char *path, const ObjData &data)`
pub fn objbinsave(path: &Path, data: &ObjData) -> bool {
    // objparser.cpp:826-828
    let file = match File::create(path) {
        Ok(f) => f,
        Err(_) => return false,
    };
    let mut w = BufWriter::new(file);

    // objparser.cpp:830-831 — size_t version = 1;
    let version: usize = 1;
    let _ = w.write_all(&version.to_ne_bytes());

    // objparser.cpp:833-843
    let result = savevector_f32(&mut w, &data.coordinates)
        && savevector_f32(&mut w, &data.texture_coordinates)
        && savevector_f32(&mut w, &data.normals)
        && savevector_f32(&mut w, &data.parameters)
        && savevector_string(&mut w, &data.mtllibs)
        && savevectornameidx(&mut w, &data.usemtls, usemtl_nameidx)
        && savevectornameidx(&mut w, &data.objects, object_nameidx)
        && savevectornameidx(&mut w, &data.groups, group_nameidx)
        && savevector_smoothing(&mut w, &data.smoothing_groups)
        && savevector_vertex(&mut w, &data.vertices);

    // objparser.cpp:845-846 — ::fclose(pFile); (write errors are ignored, as in C++)
    let _ = w.flush();
    result
}

/// objparser.cpp:849 — `bool objbinload(const char *path, ObjData &data)`
///
/// Note: faithful to the C++, the version is read as a 4-byte `int`
/// (`sizeof(data.version)`), although `objbinsave` writes it as a `size_t`.
/// On 64-bit targets the C++ therefore leaves half of the version field in
/// the stream and every subsequent count is misread, making the load fail;
/// the Rust port returns false in that case (instead of the C++'s unguarded
/// huge `vector::assign`).
pub fn objbinload(path: &Path, data: &mut ObjData) -> bool {
    // objparser.cpp:851-855 — the C++ even calls fclose(NULL) when fopen fails.
    let mut file = match File::open(path) {
        Ok(f) => f,
        Err(_) => return false,
    };

    // objparser.cpp:857-861
    data.version = 0;
    let mut vbuf = [0u8; 4];
    if file.read_exact(&mut vbuf).is_err() {
        return false;
    }
    data.version = i32::from_ne_bytes(vbuf);
    // objparser.cpp:862-865
    if data.version != 1 {
        return false;
    }
    // objparser.cpp:866-876
    let result = loadvector_f32(&mut file, &mut data.coordinates)
        && loadvector_f32(&mut file, &mut data.texture_coordinates)
        && loadvector_f32(&mut file, &mut data.normals)
        && loadvector_f32(&mut file, &mut data.parameters)
        && loadvector_string(&mut file, &mut data.mtllibs)
        && loadvectornameidx(&mut file, &mut data.usemtls, |idx, name| {
            let mut m = ObjUseMtl::new();
            m.vertex_idx_first = idx;
            m.name = name;
            m
        })
        && loadvectornameidx(&mut file, &mut data.objects, |idx, name| ObjObject {
            vertex_idx_first: idx,
            name,
        })
        && loadvectornameidx(&mut file, &mut data.groups, |idx, name| ObjGroup {
            vertex_idx_first: idx,
            name,
        })
        && loadvector_smoothing(&mut file, &mut data.smoothing_groups)
        && loadvector_vertex(&mut file, &mut data.vertices);

    // objparser.cpp:878-879
    result
}

// ---------------------------------------------------------------------------
// Equality  (objparser.cpp:882-918)
// ---------------------------------------------------------------------------

/// objparser.cpp:882-891 — `template<typename T> bool vectorequal(...)`
/// (the std::string overload at cpp:893-901 behaves identically through
/// `PartialEq`).
fn vectorequal<T: PartialEq>(v1: &[T], v2: &[T]) -> bool {
    if v1.len() != v2.len() {
        return false;
    }
    for i in 0..v1.len() {
        if v1[i] != v2[i] {
            return false;
        }
    }
    true
}

/// objparser.cpp:903 — `extern bool objequal(const ObjData &data1, const ObjData &data2)`
pub fn objequal(data1: &ObjData, data2: &ObjData) -> bool {
    // FIXME ignore version number
    // version;

    // objparser.cpp:908-917 — note: smoothingGroups is not compared (faithful
    // to the C++).
    vectorequal(&data1.coordinates, &data2.coordinates)
        && vectorequal(&data1.texture_coordinates, &data2.texture_coordinates)
        && vectorequal(&data1.normals, &data2.normals)
        && vectorequal(&data1.parameters, &data2.parameters)
        && vectorequal(&data1.mtllibs, &data2.mtllibs)
        && vectorequal(&data1.usemtls, &data2.usemtls)
        && vectorequal(&data1.objects, &data2.objects)
        && vectorequal(&data1.groups, &data2.groups)
        && vectorequal(&data1.vertices, &data2.vertices)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parsemlinfo() {
        assert_eq!(parsemlinfo("# region: foo", "region:"), "foo");
        assert_eq!(parsemlinfo("# nothing here", "region:"), "");
    }

    #[test]
    fn test_obj_vertex_delimiter() {
        let d = ObjVertex::delimiter();
        assert_eq!(d.coord_idx, -1);
        assert_eq!(d.texture_coord_idx, -1);
        assert_eq!(d.normal_idx, -1);
    }

    #[test]
    fn test_obj_parseline_vertex() {
        let mut data = ObjData::new();
        assert!(obj_parseline(b"v 1.0 2.0 3.0", &mut data));
        assert_eq!(data.coordinates.len(), OBJ_VERTEX_LENGTH);
        assert!((data.coordinates[0] - 1.0).abs() < 1e-6);
        assert!((data.coordinates[1] - 2.0).abs() < 1e-6);
        assert!((data.coordinates[2] - 3.0).abs() < 1e-6);
        // No colour data: x, y, z, 0, 0, 0, 0.
        assert_eq!(&data.coordinates[3..], &[0.0, 0.0, 0.0, 0.0]);
        assert!(!data.has_vertex_color);
    }

    #[test]
    fn test_obj_parseline_vertex_color() {
        let mut data = ObjData::new();
        assert!(obj_parseline(b"v 1 2 3 0.5 0.25 0.125", &mut data));
        assert!(data.has_vertex_color);
        // Alpha defaults to 1.0 (objparser.cpp:175).
        assert_eq!(data.coordinates[OBJ_VERTEX_COLOR_ALPHA], 1.0);
    }

    #[test]
    fn test_obj_parseline_face() {
        let mut data = ObjData::new();
        // Add 3 vertices first
        obj_parseline(b"v 0 0 0", &mut data);
        obj_parseline(b"v 1 0 0", &mut data);
        obj_parseline(b"v 0 1 0", &mut data);
        obj_parseline(b"f 1 2 3", &mut data);
        // 3 vertex refs + 1 delimiter
        assert_eq!(data.vertices.len(), 4);
        assert_eq!(data.vertices[3], ObjVertex::delimiter());
        assert_eq!(data.vertices[0].coord_idx, 0);
    }

    #[test]
    fn test_obj_parseline_smoothing_off_rejected() {
        // "s off" yields no strtol conversion -> the C++ returns false.
        let mut data = ObjData::new();
        assert!(!obj_parseline(b"s off", &mut data));
        assert!(data.smoothing_groups.is_empty());
        assert!(obj_parseline(b"s 1", &mut data));
        assert_eq!(data.smoothing_groups[0].smoothing_group_id, 1);
    }

    #[test]
    fn test_obj_parseline_object_name_empty() {
        // objparser.cpp:332-335 — the C++ stores the remainder at the NUL: "".
        let mut data = ObjData::new();
        assert!(obj_parseline(b"o myobject", &mut data));
        assert_eq!(data.objects.len(), 1);
        assert_eq!(data.objects[0].name, "");
    }

    #[test]
    fn test_obj_parseline_group_keeps_leading_ws() {
        // objparser.cpp:342-344 — no EATWS before the group name.
        let mut data = ObjData::new();
        assert!(obj_parseline(b"g mygroup", &mut data));
        assert_eq!(data.groups[0].name, " mygroup");
    }

    #[test]
    fn test_usemtl_face_counting() {
        let mut data = ObjData::new();
        obj_parseline(b"v 0 0 0", &mut data);
        obj_parseline(b"v 1 0 0", &mut data);
        obj_parseline(b"v 0 1 0", &mut data);
        obj_parseline(b"v 1 1 0", &mut data);
        obj_parseline(b"usemtl mat1", &mut data);
        assert_eq!(data.usemtls[0].face_start, 0);
        assert_eq!(data.usemtls[0].face_end, -1);
        obj_parseline(b"f 1 2 3", &mut data); // tri
        assert_eq!(data.usemtls[0].face_end, 0);
        obj_parseline(b"f 1 2 3 4", &mut data); // quad counts as two
        assert_eq!(data.usemtls[0].face_end, 2);
        obj_parseline(b"usemtl mat2", &mut data);
        assert_eq!(data.usemtls[1].face_start, 3);
        assert_eq!(data.usemtls[1].face_end, 2);
    }

    #[test]
    fn test_mtl_parseline_newmtl() {
        let mut data = MtlData::new();
        let mut name = String::new();
        mtl_parseline(b"newmtl TestMaterial", &mut data, &mut name);
        assert_eq!(name, "TestMaterial");
        assert!(data.new_mtl_unmap.contains_key("TestMaterial"));
        assert_eq!(data.mtl_orders, vec!["TestMaterial".to_string()]);
    }

    #[test]
    fn test_mtl_parseline_kd() {
        let mut data = MtlData::new();
        let mut name = String::new();
        mtl_parseline(b"newmtl m", &mut data, &mut name);
        assert!(mtl_parseline(b"Kd 0.1 0.2 0.3", &mut data, &mut name));
        let mtl = &data.new_mtl_unmap["m"];
        assert_eq!(mtl.kd, [0.1, 0.2, 0.3]);
        // Missing third component -> false (endptr check).
        assert!(!mtl_parseline(b"Ka 0.1 0.2", &mut data, &mut name));
    }

    #[test]
    fn test_strtod_strtol_endptr_contracts() {
        // No conversion: endptr == original pointer.
        assert_eq!(strtod(b"abc", 0), (0.0, 0));
        assert_eq!(strtol(b"abc", 0), (0, 0));
        // Longest valid prefix.
        let (v, e) = strtod(b"1.5x", 0);
        assert_eq!((v, e), (1.5, 3));
        let (v, e) = strtol(b"-12/", 0);
        assert_eq!((v, e), (-12, 3));
        // Incomplete exponent is not consumed.
        let (v, e) = strtod(b"1e", 0);
        assert_eq!((v, e), (1.0, 1));
    }

    #[test]
    fn test_objequal_identical() {
        let d1 = ObjData::new();
        let d2 = ObjData::new();
        assert!(objequal(&d1, &d2));
    }

    #[test]
    fn test_objbin_save_then_load_fails_like_cpp() {
        // The C++ writes the version as size_t but reads it back as int, so a
        // round-trip fails on 64-bit targets; the port preserves that.
        let dir = std::env::temp_dir();
        let path = dir.join("objparser_rs_binsave_test.bin");
        let mut data = ObjData::new();
        obj_parseline(b"v 0 0 0", &mut data);
        assert!(objbinsave(&path, &data));
        let mut loaded = ObjData::new();
        if std::mem::size_of::<usize>() == 8 {
            assert!(!objbinload(&path, &mut loaded));
        }
        let _ = std::fs::remove_file(&path);
    }
}
