//! Port of `libslic3r/Config.hpp` and `libslic3r/Config.cpp` (Bambu Studio at the pin).
//!
//! The C++ class family is here under its own names: the virtual base `ConfigOption` is a
//! trait, `ConfigOptionVectorBase` a second trait for the vector valued options, and every
//! concrete class (`ConfigOptionFloat`, `ConfigOptionFloats`, `ConfigOptionFloatsNullable`, ...)
//! is a struct implementing them. `ConfigOptionSingle<T>` and `ConfigOptionVector<T>` hold the
//! method bodies the concrete classes inherit. `ConfigOptionResolver` and `ConfigBase` are
//! traits with the non-virtual methods as provided methods; `DynamicConfig` is a struct and
//! `StaticConfig` carries the two methods a static config store takes from its base.
//!
//! What the live engine uses of this file today: `FloatOrPercent`, `PrinterTechnology` and the
//! string helpers. The engine's own settings structs (`print_config.rs`, `region_config.rs`)
//! do not go through `DynamicConfig` yet.
//!
//! An exception thrown by the C++ is an `Err(ConfigurationError)` with the same text; a null
//! pointer the C++ dereferences without a check is an `expect` with a message naming it.
//!
//! Not here: the cereal hooks (`serialize(Archive&)`, `save`/`load` of ConfigOptionPoints,
//! `ConfigOptionDef::load_option_from_archive` / `save_option_to_archive`, the
//! `CEREAL_REGISTER_TYPE` lists at the end of Config.cpp): the crate has no cereal archive.

use crate::geometry::{Vec2d, Vec3d};

/// Point.hpp `typedef std::vector<Vec2d> Vec2ds;`
type Vec2ds = Vec<Vec2d>;

// ===========================================================================
// ConfigHelpers (Config.hpp:89-117)
// ===========================================================================
pub mod config_helpers {
    // Config.hpp:90-99  inline bool looks_like_enum_value(std::string value)
    pub fn looks_like_enum_value(value: &str) -> bool {
        // Config.hpp:92  boost::trim(value);
        // boost::trim defaults to std::isspace (C locale): ' ' \t \n \v \f \r
        let value = value.trim_matches(is_space);
        // Config.hpp:93-94  if (value.empty() || value.size() > 64 || ! isalpha(value.front())) return false;
        let bytes = value.as_bytes();
        if value.is_empty() || value.len() > 64 || !is_alpha(bytes[0]) {
            return false;
        }
        // Config.hpp:95-97  for (const char c : value) if (! (isalnum(c) || c == '_' || c == '-')) return false;
        for &c in bytes {
            if !(is_alnum(c) || c == b'_' || c == b'-') {
                return false;
            }
        }
        // Config.hpp:98  return true;
        true
    }

    // Config.hpp:101-104  inline bool enum_looks_like_true_value(std::string value)
    pub fn enum_looks_like_true_value(value: &str) -> bool {
        // Config.hpp:102  boost::trim(value);
        let value = value.trim_matches(is_space);
        // Config.hpp:103  return boost::iequals(value, "enabled") || boost::iequals(value, "on");
        value.eq_ignore_ascii_case("enabled") || value.eq_ignore_ascii_case("on")
    }

    // Config.hpp:106-110  enum class DeserializationSubstitution
    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    pub enum DeserializationSubstitution {
        Disabled,
        DefaultsToFalse,
        DefaultsToTrue,
    }

    // Config.hpp:112-116  enum class DeserializationResult
    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    pub enum DeserializationResult {
        Loaded,
        Substituted,
        Failed,
    }

    // C-locale `std::isspace` used by `boost::trim`: ' ' \t \n \v \f \r.
    fn is_space(c: char) -> bool {
        c == ' ' || c == '\t' || c == '\n' || c == '\u{0B}' || c == '\u{0C}' || c == '\r'
    }

    // C `isalpha` for ASCII (matches the C locale used by libslic3r).
    fn is_alpha(c: u8) -> bool {
        c.is_ascii_alphabetic()
    }

    // C `isalnum` for ASCII.
    fn is_alnum(c: u8) -> bool {
        c.is_ascii_alphanumeric()
    }
}

// ===========================================================================
// PrinterTechnology (Config.hpp:210-220)
// ===========================================================================

// Config.hpp:210  enum PrinterTechnology : unsigned char
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
#[repr(u8)]
pub enum PrinterTechnology {
    // Fused Filament Fabrication
    PtFFF,
    // Stereolitography
    PtSLA,
    // Unknown, useful for command line processing
    PtUnknown,
    // Any technology, useful for parameters compatible with both ptFFF and ptSLA
    PtAny,
}

// ===========================================================================
// ForwardCompatibilitySubstitutionRule (Config.hpp:222-234)
// ===========================================================================

// Config.hpp:222  enum ForwardCompatibilitySubstitutionRule
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ForwardCompatibilitySubstitutionRule {
    // Disable susbtitution, throw exception if an option value is not recognized.
    Disable,
    // Enable substitution of an unknown option value with default. Log the substitution.
    Enable,
    // Enable substitution of an unknown option value with default. Don't log the substitution.
    EnableSilent,
    // Enable substitution of an unknown option value with default. Log substitutions in user profiles, don't log substitutions in system profiles.
    EnableSystemSilent,
    // Enable silent substitution of an unknown option value with default when loading user profiles. Throw on an unknown option value in a system profile.
    EnableSilentDisableSystem,
}

// ===========================================================================
// FloatOrPercent (Config.hpp:30-47)
// ===========================================================================

// Config.hpp:30-43  struct FloatOrPercent
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct FloatOrPercent {
    // Config.hpp:32  double value = 0;
    pub value: f64,
    // Config.hpp:33  bool percent = false;
    pub percent: bool,
}

impl FloatOrPercent {
    // Config.hpp:35  FloatOrPercent() {}
    pub fn new() -> Self {
        FloatOrPercent {
            value: 0.0,
            percent: false,
        }
    }

    // Config.hpp:36  FloatOrPercent(double value_, bool percent_)
    pub fn with(value_: f64, percent_: bool) -> Self {
        FloatOrPercent {
            value: value_,
            percent: percent_,
        }
    }

    // Config.hpp:38  double get_abs_value(double ratio_over) const
    pub fn get_abs_value(&self, ratio_over: f64) -> f64 {
        if self.percent {
            ratio_over * self.value / 100.0
        } else {
            self.value
        }
    }
}

impl Default for FloatOrPercent {
    fn default() -> Self {
        FloatOrPercent::new()
    }
}

// Config.hpp:45  inline bool operator==(const FloatOrPercent& l, const FloatOrPercent& r)
impl PartialEq for FloatOrPercent {
    fn eq(&self, other: &Self) -> bool {
        self.value == other.value && self.percent == other.percent
    }
}

impl PartialOrd for FloatOrPercent {
    // Config.hpp:47  inline bool operator< (const FloatOrPercent& l, const FloatOrPercent& r)
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        // l.value < r.value || (l.value == r.value && int(l.percent) < int(r.percent))
        match self.value.partial_cmp(&other.value) {
            Some(std::cmp::Ordering::Equal) => {
                (self.percent as i32).partial_cmp(&(other.percent as i32))
            }
            ord => ord,
        }
    }
}

// ===========================================================================
// C-style string (un)escaping (Config.cpp:48-229, 217-232)
// ===========================================================================

// Escape \n, \r, quotes and backslash
// Config.cpp:60  std::string escape_string_cstyle(const std::string &str)
pub fn escape_string_cstyle(str: &str) -> String {
    // Allocate a buffer twice the input string length,
    // so the output will fit even if all input characters get escaped.
    // Config.cpp:64-65  std::vector<char> out(str.size() * 2, 0); char *outptr = out.data();
    let mut out: Vec<u8> = Vec::with_capacity(str.len() * 2);
    // Config.cpp:66-82
    for &c in str.as_bytes() {
        if c == b'\r' {
            // Config.cpp:68-70
            out.push(b'\\');
            out.push(b'r');
        } else if c == b'\n' {
            // Config.cpp:71-73
            out.push(b'\\');
            out.push(b'n');
        } else if c == b'"' {
            // Config.cpp:74-76
            out.push(b'\\');
            out.push(b'"');
        } else if c == b'\\' {
            // Config.cpp:77-79
            out.push(b'\\');
            out.push(b'\\');
        } else {
            // Config.cpp:80-81
            out.push(c);
        }
    }
    // Config.cpp:83  return std::string(out.data(), outptr - out.data());
    String::from_utf8_lossy(&out).into_owned()
}

// Config.cpp:86  std::string escape_strings_cstyle(const std::vector<std::string> &strs)
pub fn escape_strings_cstyle(strs: &[String]) -> String {
    // 1) Estimate the output buffer size to avoid buffer reallocation.
    // Config.cpp:89-92
    // (Rust Vec grows automatically; the estimate is purely a reserve hint.)
    let mut outbuflen: usize = 0;
    for str in strs.iter() {
        // Reserve space for every character escaped + quotes + semicolon.
        outbuflen += str.len() * 2 + 3;
    }
    // 2) Fill in the buffer.
    // Config.cpp:94-95  std::vector<char> out(outbuflen, 0); char *outptr = out.data();
    let mut out: Vec<u8> = Vec::with_capacity(outbuflen);
    // Config.cpp:96-132
    for (j, str) in strs.iter().enumerate() {
        // Config.cpp:97-99  if (j > 0) separate the strings.
        if j > 0 {
            out.push(b';');
        }
        let bytes = str.as_bytes();
        // Is the string simple or complex? Complex string contains spaces, tabs, new lines and other
        // escapable characters. Empty string shall be quoted as well, if it is the only string in strs.
        // Config.cpp:103  bool should_quote = strs.size() == 1 && str.empty();
        let mut should_quote = strs.len() == 1 && str.is_empty();
        // Config.cpp:104-110
        for &c in bytes {
            if c == b' ' || c == b'\t' || c == b'\\' || c == b'"' || c == b'\r' || c == b'\n' {
                should_quote = true;
                break;
            }
        }
        if should_quote {
            // Config.cpp:112-127
            out.push(b'"');
            for &c in bytes {
                if c == b'\\' || c == b'"' {
                    out.push(b'\\');
                    out.push(c);
                } else if c == b'\r' {
                    out.push(b'\\');
                    out.push(b'r');
                } else if c == b'\n' {
                    out.push(b'\\');
                    out.push(b'n');
                } else {
                    out.push(c);
                }
            }
            out.push(b'"');
        } else {
            // Config.cpp:128-131  memcpy(outptr, str.data(), str.size());
            out.extend_from_slice(bytes);
        }
    }
    // Config.cpp:133  return std::string(out.data(), outptr - out.data());
    String::from_utf8_lossy(&out).into_owned()
}

// Unescape \n, \r and backslash
// Config.cpp:137  bool unescape_string_cstyle(const std::string &str, std::string &str_out)
pub fn unescape_string_cstyle(str: &str, str_out: &mut String) -> bool {
    // Config.cpp:139-140  std::vector<char> out(str.size(), 0); char *outptr = out.data();
    let mut out: Vec<u8> = Vec::with_capacity(str.len());
    let bytes = str.as_bytes();
    let len = bytes.len();
    // Config.cpp:141  for (size_t i = 0; i < str.size(); ++ i)
    let mut i: usize = 0;
    while i < len {
        // Config.cpp:142  char c = str[i];
        let mut c = bytes[i];
        // Config.cpp:143  if (c == '\\')
        if c == b'\\' {
            // Config.cpp:144-145  if (++ i == str.size()) return false;
            i += 1;
            if i == len {
                return false;
            }
            // Config.cpp:146  c = str[i];
            c = bytes[i];
            if c == b'r' {
                // Config.cpp:147-148  if (c == 'r') (*outptr ++) = '\r';
                out.push(b'\r');
            } else if c == b'n' {
                // Config.cpp:149-150  else if (c == 'n') (*outptr ++) = '\n';
                out.push(b'\n');
            } else {
                // Config.cpp:151-152  else (*outptr ++) = c;
                out.push(c);
            }
        } else {
            // Config.cpp:153-154  else (*outptr ++) = c;
            out.push(c);
        }
        i += 1;
    }
    // Config.cpp:156  str_out.assign(out.data(), outptr - out.data());
    *str_out = String::from_utf8_lossy(&out).into_owned();
    // Config.cpp:157  return true;
    true
}

// Config.cpp:160  bool unescape_strings_cstyle(const std::string &str, std::vector<std::string> &out)
pub fn unescape_strings_cstyle(str: &str, out: &mut Vec<String>) -> bool {
    let bytes = str.as_bytes();
    let len = bytes.len();
    // Config.cpp:162-163  if (str.empty()) return true;
    if len == 0 {
        return true;
    }

    // Config.cpp:165  size_t i = 0;
    let mut i: usize = 0;
    // Config.cpp:166  for (;;)
    loop {
        // Skip white spaces.
        // Config.cpp:168-173
        let mut c = bytes[i];
        while c == b' ' || c == b'\t' {
            i += 1;
            if i == len {
                return true;
            }
            c = bytes[i];
        }
        // Start of a word.
        // Config.cpp:175-176  std::vector<char> buf; buf.reserve(16);
        let mut buf: Vec<u8> = Vec::with_capacity(16);
        // Is it enclosed in quotes?
        // Config.cpp:178  c = str[i];
        c = bytes[i];
        // Config.cpp:179  if (c == '"')
        if c == b'"' {
            // Complex case, string is enclosed in quotes.
            // Config.cpp:181-197  for (++ i; i < str.size(); ++ i)
            i += 1;
            while i < len {
                c = bytes[i];
                // Config.cpp:183-186  if (c == '"') break;
                if c == b'"' {
                    // End of string.
                    break;
                }
                // Config.cpp:187-195
                if c == b'\\' {
                    // Config.cpp:188-189  if (++ i == str.size()) return false;
                    i += 1;
                    if i == len {
                        return false;
                    }
                    c = bytes[i];
                    if c == b'r' {
                        c = b'\r';
                    } else if c == b'n' {
                        c = b'\n';
                    }
                }
                // Config.cpp:196  buf.push_back(c);
                buf.push(c);
                i += 1;
            }
            // Config.cpp:198-199  if (i == str.size()) return false;
            if i == len {
                return false;
            }
            // Config.cpp:200  ++ i;
            i += 1;
        } else {
            // Config.cpp:201-208  for (; i < str.size(); ++ i)
            while i < len {
                c = bytes[i];
                // Config.cpp:204-205  if (c == ';') break;
                if c == b';' {
                    break;
                }
                buf.push(c);
                i += 1;
            }
        }
        // Store the string into the output vector.
        // Config.cpp:210  out.push_back(std::string(buf.data(), buf.size()));
        out.push(String::from_utf8_lossy(&buf).into_owned());
        // Config.cpp:211-212  if (i == str.size()) return true;
        if i == len {
            return true;
        }
        // Skip white spaces.
        // Config.cpp:214-220
        c = bytes[i];
        while c == b' ' || c == b'\t' {
            i += 1;
            if i == len {
                // End of string. This is correct.
                return true;
            }
            c = bytes[i];
        }
        // Config.cpp:221-222  if (c != ';') return false;
        if c != b';' {
            return false;
        }
        // Config.cpp:223-227  if (++ i == str.size())
        i += 1;
        if i == len {
            // Emit one additional empty string.
            out.push(String::new());
            return true;
        }
    }
}

// Config.cpp:231  std::string escape_ampersand(const std::string& str)
pub fn escape_ampersand(str: &str) -> String {
    // Allocate a buffer 2 times the input string length,
    // so the output will fit even if all input characters get escaped.
    // Config.cpp:235-236  std::vector<char> out(str.size() * 6, 0); char* outptr = out.data();
    let mut out: Vec<u8> = Vec::with_capacity(str.len() * 6);
    // Config.cpp:237-244
    for &c in str.as_bytes() {
        if c == b'&' {
            // Config.cpp:239-241
            out.push(b'&');
            out.push(b'&');
        } else {
            // Config.cpp:242-243
            out.push(c);
        }
    }
    // Config.cpp:245  return std::string(out.data(), outptr - out.data());
    String::from_utf8_lossy(&out).into_owned()
}

// ===========================================================================
// G-code line-scanning helpers (Config.cpp:1318-1334)
// ===========================================================================

// BBS
// Config.cpp:1319  static bool is_whitespace(char c)
pub fn is_whitespace(c: u8) -> bool {
    c == b' ' || c == b'\t'
}

// Config.cpp:1320  static bool is_end_of_line(char c)
pub fn is_end_of_line(c: u8) -> bool {
    c == b'\r' || c == b'\n' || c == 0
}

// Config.cpp:1321  static bool is_end_of_gcode_line(char c)
pub fn is_end_of_gcode_line(c: u8) -> bool {
    c == b';' || is_end_of_line(c)
}

// Config.cpp:1322  static bool is_end_of_word(char c)
pub fn is_end_of_word(c: u8) -> bool {
    is_whitespace(c) || is_end_of_gcode_line(c)
}

// Config.cpp:1324-1328  static const char* skip_word(const char* c)
// Returns the index of the first end-of-word byte at or after `start`.
// (NUL termination is modelled by treating end-of-slice as a 0 byte.)
pub fn skip_word(bytes: &[u8], start: usize) -> usize {
    let mut c = start;
    // for (; !is_end_of_word(*c); ++c) ;
    while c < bytes.len() && !is_end_of_word(bytes[c]) {
        c += 1;
    }
    c
}

// Config.cpp:1330-1334  static const char* skip_whitespaces(const char* c)
// Returns the index of the first non-whitespace byte at or after `start`.
pub fn skip_whitespaces(bytes: &[u8], start: usize) -> usize {
    let mut c = start;
    // for (; is_whitespace(*c); ++c) ;
    while c < bytes.len() && is_whitespace(bytes[c]) {
        c += 1;
    }
    c
}

// ---------------------------------------------------------------------------
// The text conversions Config.hpp gets from the C++ standard library.
//
// FIDELITY-NOTE: `ss << double`, `iss >> double`, `iss >> int`, `sscanf("%lf")`,
// `std::getline` and `boost::trim` are library calls in Config.hpp. What follows
// reproduces them as they behave in the C locale with libc++ and Apple's libc (its BSD
// printf / vfscanf and gdtoa strtod; the build the parity vectors of
// tests/config_native_parity.rs come from). Where libstdc++, glibc or the MSVC library
// differ on malformed input (they do: "1.5abc", "nan(...)" payloads, hexadecimal floats,
// "-nan" printed for a negative NaN), this file follows libc++ and Apple's libc.
// ---------------------------------------------------------------------------

/// `std::isspace` in the classic locale: what `boost::trim` and the stream sentry skip.
fn is_space_byte(c: u8) -> bool {
    c == b' ' || c == b'\t' || c == b'\n' || c == 0x0B || c == 0x0C || c == b'\r'
}

/// `boost::trim(std::string&)` (boost/algorithm/string/trim.hpp), classic locale.
fn boost_trim(s: &[u8]) -> &[u8] {
    let mut b = 0usize;
    let mut e = s.len();
    while b < e && is_space_byte(s[b]) {
        b += 1;
    }
    while e > b && is_space_byte(s[e - 1]) {
        e -= 1;
    }
    &s[b..e]
}

/// The items a `while (std::getline(is, item, delim))` loop sees: the pieces between the
/// delimiters, without a last empty piece (the stream is at EOF by then and `getline` fails).
fn getline_split(s: &[u8], delim: u8) -> Vec<&[u8]> {
    let mut out: Vec<&[u8]> = s.split(|&c| c == delim).collect();
    if out.last().map_or(false, |l| l.is_empty()) {
        out.pop();
    }
    out
}

fn bytes_to_string(b: &[u8]) -> String {
    String::from_utf8_lossy(b).into_owned()
}

/// `std::ostringstream << double` with the default flags: `%g`, precision 6.
pub(crate) fn ostream_double(v: f64) -> String {
    if v.is_nan() {
        // The BSD printf of Apple's libc prints "nan" whatever the sign bit (glibc: "-nan").
        return "nan".to_string();
    }
    crate::locales_utils::general_format(v, 6)
}

/// `std::ostringstream << int`.
fn ostream_int(v: i32) -> String {
    v.to_string()
}

/// Result of `strtod`: the value, the number of bytes consumed (0: no conversion) and
/// whether `errno` was set to `ERANGE`.
pub(crate) struct Strtod {
    pub(crate) value: f64,
    pub(crate) consumed: usize,
    pub(crate) erange: bool,
}

fn hex_digit_value(c: u8) -> Option<u32> {
    match c {
        b'0'..=b'9' => Some((c - b'0') as u32),
        b'a'..=b'f' => Some((c - b'a') as u32 + 10),
        b'A'..=b'F' => Some((c - b'A') as u32 + 10),
        _ => None,
    }
}

/// `x * 2^e` for an `x` whose scaled value is exactly representable (or overflows): the steps
/// are exact multiplications by powers of two.
fn ldexp(mut x: f64, mut e: i64) -> f64 {
    if x == 0.0 || !x.is_finite() {
        return x;
    }
    while e > 1000 {
        x *= f64::from_bits(0x7E70000000000000); // 2^1000
        e -= 1000;
        if x.is_infinite() {
            return x;
        }
    }
    while e < -1000 {
        x *= f64::from_bits(0x0170000000000000); // 2^-1000
        e += 1000;
        if x == 0.0 {
            return x;
        }
    }
    x * 2f64.powi(e as i32)
}

/// The payload of "nan(n-char-sequence)" in Apple's `strtod`, see strtod_c.
fn nan_payload(seq: &[u8]) -> u64 {
    let (digits, base) = if seq.starts_with(b"0x") {
        (&seq[2..], 16u64)
    } else if seq.starts_with(b"0") {
        (&seq[1..], 8u64)
    } else {
        (seq, 10u64)
    };
    let mut v: u64 = 0;
    for &c in digits {
        match hex_digit_value(c) {
            Some(d) if (d as u64) < base => v = v.wrapping_mul(base).wrapping_add(d as u64),
            _ => return 0,
        }
    }
    v
}

/// C `strtod` of the BSD libc (gdtoa) in the C locale, over the bytes of `s` (the end of the
/// slice is the terminating NUL). Leading white space, a sign, then one of: `inf`/`infinity`,
/// `nan` with an optional `(n-char-sequence)`, a hexadecimal float, a decimal float.
pub(crate) fn strtod_c(s: &[u8]) -> Strtod {
    let n = s.len();
    let mut i = 0usize;
    while i < n && is_space_byte(s[i]) {
        i += 1;
    }
    let mut negative = false;
    if i < n && (s[i] == b'+' || s[i] == b'-') {
        negative = s[i] == b'-';
        i += 1;
    }
    let apply_sign = |v: f64| if negative { -v } else { v };
    let lower = |k: usize| if k < n { s[k].to_ascii_lowercase() } else { 0 };
    // inf / infinity
    if lower(i) == b'i' && lower(i + 1) == b'n' && lower(i + 2) == b'f' {
        let mut end = i + 3;
        if lower(i + 3) == b'i'
            && lower(i + 4) == b'n'
            && lower(i + 5) == b'i'
            && lower(i + 6) == b't'
            && lower(i + 7) == b'y'
        {
            end = i + 8;
        }
        return Strtod { value: apply_sign(f64::INFINITY), consumed: end, erange: false };
    }
    // nan, nan(...): everything up to the first ')' belongs to the number. The payload is the
    // text between the parentheses read as an unsigned integer ("0x" hexadecimal, a leading "0"
    // octal, else decimal; wrapping at 64 bits; any other character makes it 0), kept in the
    // 51 bits below the quiet bit. That is what Apple's libc does (probed on macOS 26).
    if lower(i) == b'n' && lower(i + 1) == b'a' && lower(i + 2) == b'n' {
        let mut end = i + 3;
        let mut payload: u64 = 0;
        if end < n && s[end] == b'(' {
            let mut k = end + 1;
            while k < n && s[k] != b')' {
                k += 1;
            }
            if k < n {
                payload = nan_payload(&s[end + 1..k]);
                end = k + 1;
            }
        }
        let value = f64::from_bits(0x7ff8_0000_0000_0000 | (payload & 0x0007_ffff_ffff_ffff));
        return Strtod { value: apply_sign(value), consumed: end, erange: false };
    }
    // hexadecimal float
    if i + 1 < n && s[i] == b'0' && (s[i + 1] == b'x' || s[i + 1] == b'X') {
        let mut k = i + 2;
        let mut mant: u128 = 0;
        let mut mant_digits = 0u32; // significant hex digits kept in `mant`
        let mut sticky = false;
        let mut any_digit = false;
        let mut exp2: i64 = 0;
        let mut seen_point = false;
        loop {
            if k < n && s[k] == b'.' && !seen_point {
                seen_point = true;
                k += 1;
                continue;
            }
            let d = if k < n { hex_digit_value(s[k]) } else { None };
            match d {
                Some(d) => {
                    any_digit = true;
                    if mant_digits < 28 {
                        mant = (mant << 4) | d as u128;
                        if mant != 0 {
                            mant_digits += 1;
                        }
                        if seen_point {
                            exp2 -= 4;
                        }
                    } else {
                        if d != 0 {
                            sticky = true;
                        }
                        if !seen_point {
                            exp2 += 4;
                        }
                    }
                    k += 1;
                }
                None => break,
            }
        }
        if any_digit {
            // A '.' that was consumed without any digit after it still belongs to the number.
            let mut end = k;
            if k < n && (s[k] == b'p' || s[k] == b'P') {
                let mut q = k + 1;
                let mut eneg = false;
                if q < n && (s[q] == b'+' || s[q] == b'-') {
                    eneg = s[q] == b'-';
                    q += 1;
                }
                if q < n && s[q].is_ascii_digit() {
                    let mut e: i64 = 0;
                    while q < n && s[q].is_ascii_digit() {
                        if e < 1_000_000 {
                            e = e * 10 + (s[q] - b'0') as i64;
                        }
                        q += 1;
                    }
                    exp2 += if eneg { -e } else { e };
                    end = q;
                }
            }
            if sticky {
                mant |= 1;
            }
            // Round the 128-bit integer to 53 bits (ties to even, the sticky bit breaks ties),
            // unless the result is subnormal, where the rounding position moves.
            let mut inexact = false;
            let value = if mant == 0 {
                0.0
            } else {
                let bits = 128 - mant.leading_zeros() as i64; // value = mant * 2^exp2
                let top_exp = bits - 1 + exp2; // exponent of the leading bit
                if top_exp < -1075 {
                    // below half of the smallest subnormal
                    inexact = true;
                    0.0
                } else if top_exp > 1023 {
                    f64::INFINITY
                } else {
                    // Bits that fit: 53, fewer in the subnormal range (lowest bit is 2^-1074).
                    let keep = if top_exp < -1022 { top_exp + 1075 } else { 53 };
                    if bits <= keep {
                        ldexp(mant as f64, exp2)
                    } else {
                        let shift = (bits - keep) as u32;
                        let mut q = mant >> shift;
                        let rem = mant & ((1u128 << shift) - 1);
                        let half = 1u128 << (shift - 1);
                        inexact = rem != 0;
                        if rem > half || (rem == half && (q & 1) == 1) {
                            q += 1;
                        }
                        ldexp(q as f64, exp2 + shift as i64)
                    }
                }
            };
            // ERANGE: an overflow, or an underflow that lost bits (an exact subnormal such as
            // 0x1p-1074 is not one).
            let erange = value.is_infinite() || (inexact && value.abs() < f64::MIN_POSITIVE);
            return Strtod { value: apply_sign(value), consumed: end, erange };
        }
        // "0x" without a hexadecimal digit: the "0" alone is the number.
        return Strtod { value: apply_sign(0.0), consumed: i + 1, erange: false };
    }
    // decimal float
    let start = i;
    let mut k = i;
    let mut digits = 0usize;
    let mut nonzero = false;
    while k < n && s[k].is_ascii_digit() {
        nonzero |= s[k] != b'0';
        k += 1;
        digits += 1;
    }
    if k < n && s[k] == b'.' {
        let mut q = k + 1;
        let mut frac = 0usize;
        while q < n && s[q].is_ascii_digit() {
            nonzero |= s[q] != b'0';
            q += 1;
            frac += 1;
        }
        if digits + frac > 0 {
            k = q;
            digits += frac;
        }
    }
    if digits == 0 {
        return Strtod { value: 0.0, consumed: 0, erange: false };
    }
    if k < n && (s[k] == b'e' || s[k] == b'E') {
        let mut q = k + 1;
        if q < n && (s[q] == b'+' || s[q] == b'-') {
            q += 1;
        }
        if q < n && s[q].is_ascii_digit() {
            while q < n && s[q].is_ascii_digit() {
                q += 1;
            }
            k = q;
        }
    }
    // The slice is ASCII digits, '.', 'e', sign: Rust's parser is correctly rounded like gdtoa.
    let text = std::str::from_utf8(&s[start..k]).unwrap_or("0");
    let value: f64 = text.parse().unwrap_or(0.0);
    let erange = value.is_infinite() || (nonzero && value.abs() < f64::MIN_POSITIVE);
    Strtod { value: apply_sign(value), consumed: k, erange }
}

/// `std::stof(str)` (libc++: `strtof` over the C string, `invalid_argument` when nothing is read,
/// `out_of_range` when errno is ERANGE): leading C-locale white space, a sign, then "inf"/"infinity",
/// "nan", a hexadecimal float (`0x` digits, '.', binary exponent `p`) or a decimal float. None for the
/// two exceptions. An underflow keeps what the decimal reader gives (the pin's vectors agree with it for
/// the smallest normal and a subnormal input).
pub(crate) fn float_prefix_value(s: &str) -> Option<f32> {
    let b = s.as_bytes();
    let mut i = 0usize;
    while i < b.len() && matches!(b[i], b' ' | b'\t' | b'\n' | b'\x0b' | b'\x0c' | b'\r') {
        i += 1;
    }
    let mut neg = false;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        neg = b[i] == b'-';
        i += 1;
    }
    let rest = &b[i..];
    let lower_starts = |w: &[u8]| rest.len() >= w.len() && rest[..w.len()].eq_ignore_ascii_case(w);
    let signed = |v: f32| if neg { -v } else { v };
    if lower_starts(b"inf") {
        return Some(signed(f32::INFINITY));
    }
    if lower_starts(b"nan") {
        return Some(signed(f32::NAN));
    }
    // Hexadecimal float: "0x" then at least one hex digit (before or after the point).
    if rest.len() >= 2 && rest[0] == b'0' && (rest[1] == b'x' || rest[1] == b'X') {
        let mut j = 2usize;
        let mut mant: u64 = 0;
        let mut exp: i64 = 0;
        let mut any = false;
        let mut sticky = false;
        let mut seen_dot = false;
        while j < rest.len() {
            let c = rest[j];
            if c == b'.' && !seen_dot {
                seen_dot = true;
                j += 1;
                continue;
            }
            let d = match (c as char).to_digit(16) {
                Some(d) => d as u64,
                None => break,
            };
            any = true;
            if mant >> 56 == 0 {
                mant = (mant << 4) | d;
                if seen_dot {
                    exp -= 4;
                }
            } else {
                sticky |= d != 0;
                if !seen_dot {
                    exp += 4;
                }
            }
            j += 1;
        }
        if any {
            if j < rest.len() && (rest[j] == b'p' || rest[j] == b'P') {
                let mut k = j + 1;
                let mut eneg = false;
                if k < rest.len() && (rest[k] == b'+' || rest[k] == b'-') {
                    eneg = rest[k] == b'-';
                    k += 1;
                }
                let ds = k;
                let mut e: i64 = 0;
                while k < rest.len() && rest[k].is_ascii_digit() {
                    e = (e * 10 + (rest[k] - b'0') as i64).min(1 << 20);
                    k += 1;
                }
                if k > ds {
                    exp += if eneg { -e } else { e };
                }
            }
            if sticky {
                mant |= 1;
            }
            // One rounding to float (u64 -> f32), then an exact scaling by a power of two in double.
            let v = ((mant as f32) as f64 * 2f64.powi(exp.clamp(-2000, 2000) as i32)) as f32;
            if v.is_infinite() {
                return None; // ERANGE: std::out_of_range
            }
            return Some(signed(v));
        }
        // "0x" without digits: strtof reads the "0".
        return Some(signed(0.0));
    }
    // Decimal: digits, an optional point and digits (at least one digit), an exponent only when digits
    // follow it.
    let mut j = 0usize;
    let mut digits = 0usize;
    while j < rest.len() && rest[j].is_ascii_digit() {
        j += 1;
        digits += 1;
    }
    if j < rest.len() && rest[j] == b'.' {
        j += 1;
        while j < rest.len() && rest[j].is_ascii_digit() {
            j += 1;
            digits += 1;
        }
    }
    if digits == 0 {
        return None; // std::invalid_argument
    }
    if j < rest.len() && (rest[j] == b'e' || rest[j] == b'E') {
        let mut k = j + 1;
        if k < rest.len() && (rest[k] == b'+' || rest[k] == b'-') {
            k += 1;
        }
        if k < rest.len() && rest[k].is_ascii_digit() {
            while k < rest.len() && rest[k].is_ascii_digit() {
                k += 1;
            }
            j = k;
        }
    }
    let text = std::str::from_utf8(&rest[..j]).ok()?;
    let v = text.parse::<f32>().ok()?;
    if v.is_infinite() {
        return None; // ERANGE: std::out_of_range
    }
    Some(signed(v))
}

// The native strtof accepts exact hexadecimal subnormals and reports ERANGE
// for decimal subnormals. Check the lowest set bit of the hex significand
// before rounding, including digits below the precision of a double.
fn exact_hex_float_subnormal(prefix: &str) -> bool {
    let unsigned = prefix.trim_ascii_start().trim_start_matches(['+', '-']);
    let Some(hex) = unsigned.strip_prefix("0x").or_else(|| unsigned.strip_prefix("0X")) else {
        return false;
    };
    let (mantissa, exponent) = hex.split_once(['p', 'P']).unwrap_or((hex, "0"));
    let decimals = mantissa.split_once('.').map_or(0, |(_, fraction)| fraction.len());
    let digits = mantissa.replace('.', "");
    let significant = digits.trim_end_matches('0');
    let Some(last) = significant.as_bytes().last().and_then(|d| hex_digit_value(*d)) else {
        return false;
    };
    let shift = exponent.parse::<i128>().unwrap_or(0) - 4 * decimals as i128
        + 4 * (digits.len() - significant.len()) as i128 + i128::from(last.trailing_zeros());
    shift >= -149
}

/// main.cpp 9745–9753 uses std::stod and requires the end of the input.
pub fn complete_finite_double(text: &str) -> Option<f64> {
    let p = strtod_c(text.as_bytes());
    (p.consumed != 0 && p.consumed == text.len() && !p.erange && p.value.is_finite())
        .then_some(p.value)
}

/// main.cpp 7250 checks std::stof's exceptions, then the option deserializes
/// the original string as doubles. Trailing text is accepted by this check.
pub(crate) fn valid_float_prefix(text: &str) -> bool {
    let p = strtod_c(text.as_bytes());
    if p.consumed == 0 || p.erange { return false; }
    if !p.value.is_finite() { return true; } // Explicit infinity or NaN.
    let Some(rounded) = float_prefix_value(text) else { return false };
    if rounded == 0.0 { return p.value == 0.0; }
    if !rounded.is_subnormal() { return true; }
    p.value == f64::from(rounded) && exact_hex_float_subnormal(&text[..p.consumed])
}

/// What `std::istringstream(str) >> double` did: `value` is `None` when the sentry failed
/// (empty input or white space only: the target is left untouched), `fail` is `iss.fail()`.
struct IstreamResult<T> {
    value: Option<T>,
    fail: bool,
}

/// `std::istringstream iss(str); iss >> value;` for `double`, libc++
/// (`num_get::__do_get_floating_point`, `__stage2_float_loop`, `__num_get_float`).
fn istream_double(s: &[u8]) -> IstreamResult<f64> {
    let n = s.len();
    let mut b = 0usize;
    while b < n && is_space_byte(s[b]) {
        b += 1;
    }
    if b == n {
        // sentry: eofbit | failbit, no extraction
        return IstreamResult { value: None, fail: true };
    }
    // Stage 2: collect the characters that may belong to a floating point number.
    const SRC: &[u8; 32] = b"0123456789abcdefABCDEFxX+-pPiInN";
    let mut a: Vec<u8> = Vec::new();
    let mut in_units = true;
    let mut exp = b'E';
    let mut is_leading_parsed = false;
    while b < n {
        let ct = s[b];
        // __stage2_float_loop
        if ct == b'.' {
            if !in_units {
                break;
            }
            in_units = false;
            a.push(b'.');
        } else {
            let f = match SRC.iter().position(|&c| c == ct) {
                Some(f) => f,
                None => break,
            };
            let x = SRC[f];
            if x == b'-' || x == b'+' {
                if a.is_empty() || a[a.len() - 1].to_ascii_uppercase() == exp.to_ascii_uppercase() {
                    a.push(x);
                } else {
                    break;
                }
            } else {
                if x == b'x' || x == b'X' {
                    exp = b'P';
                } else if x.to_ascii_uppercase() == exp {
                    exp = exp.to_ascii_lowercase();
                    if in_units {
                        in_units = false;
                    }
                }
                a.push(x);
            }
        }
        // the leading character excluding the sign must be a decimal digit
        if !is_leading_parsed {
            if !a.is_empty() && a[0] != b'-' && a[0] != b'+' {
                if a[0].is_ascii_digit() || a[0] == b'.' {
                    is_leading_parsed = true;
                } else {
                    break;
                }
            } else if a.len() >= 2 && (a[0] == b'-' || a[0] == b'+') {
                if a[1].is_ascii_digit() || a[1] == b'.' {
                    is_leading_parsed = true;
                } else {
                    break;
                }
            }
        }
        b += 1;
    }
    // Stage 3: __num_get_float
    if a.is_empty() {
        return IstreamResult { value: Some(0.0), fail: true };
    }
    let r = strtod_c(&a);
    if r.consumed != a.len() {
        return IstreamResult { value: Some(0.0), fail: true };
    }
    IstreamResult { value: Some(r.value), fail: r.erange }
}

/// `std::istringstream iss(str); iss >> value;` for `int`, libc++
/// (`__input_arithmetic_with_numeric_limits<int>`, `__stage2_int_loop` in base 10,
/// `__num_get_signed_integral<long>`).
fn istream_int(s: &[u8]) -> IstreamResult<i32> {
    let n = s.len();
    let mut b = 0usize;
    while b < n && is_space_byte(s[b]) {
        b += 1;
    }
    if b == n {
        return IstreamResult { value: None, fail: true };
    }
    let mut a: Vec<u8> = Vec::new();
    while b < n {
        let ct = s[b];
        if a.is_empty() && (ct == b'+' || ct == b'-') {
            a.push(ct);
        } else if ct.is_ascii_digit() {
            a.push(ct);
        } else {
            break;
        }
        b += 1;
    }
    if a.is_empty() {
        return IstreamResult { value: Some(0), fail: true };
    }
    // strtoll(a, &p2, 10): a sign alone converts nothing, p2 != a_end.
    let (neg, digits) = match a[0] {
        b'-' => (true, &a[1..]),
        b'+' => (false, &a[1..]),
        _ => (false, &a[..]),
    };
    if digits.is_empty() {
        return IstreamResult { value: Some(0), fail: true };
    }
    let mut ll: i128 = 0;
    for &d in digits {
        ll = ll * 10 + (d - b'0') as i128;
        if ll > (i64::MAX as i128) + 1 {
            ll = (i64::MAX as i128) + 2; // saturate, out of the range of long
        }
    }
    if neg {
        ll = -ll;
    }
    // ERANGE of strtoll: the long saturates and failbit is set; then the int range check.
    let mut fail = false;
    let temp: i64 = if ll > i64::MAX as i128 {
        fail = true;
        i64::MAX
    } else if ll < i64::MIN as i128 {
        fail = true;
        i64::MIN
    } else {
        ll as i64
    };
    if temp < i32::MIN as i64 {
        IstreamResult { value: Some(i32::MIN), fail: true }
    } else if temp > i32::MAX as i64 {
        IstreamResult { value: Some(i32::MAX), fail: true }
    } else {
        IstreamResult { value: Some(temp as i32), fail }
    }
}

/// The text a `%lf` of the BSD `vfscanf` (`parsefloat()`) hands to `strtod`: what strtod would
/// take, except in "nan(...)", where only letters, digits and '_' may stand between the
/// parentheses; anything else (or no ')') ends the field after "nan".
fn scanf_float_field(s: &[u8]) -> &[u8] {
    let mut k = 0usize;
    if k < s.len() && (s[k] == b'+' || s[k] == b'-') {
        k += 1;
    }
    if s.len() >= k + 4 && s[k..k + 3].eq_ignore_ascii_case(b"nan") && s[k + 3] == b'(' {
        let mut q = k + 4;
        while q < s.len() && (s[q].is_ascii_alphanumeric() || s[q] == b'_') {
            q += 1;
        }
        if q < s.len() && s[q] == b')' {
            return &s[..q + 1];
        }
        return &s[..k + 3];
    }
    s
}

/// `sscanf(str.data(), " %lf SEP %lf [SEP %lf] %c", ...)` as ConfigOptionPoint::deserialize and
/// ConfigOptionPoint3::deserialize call it: white space, a double, white space, the separator,
/// ..., white space and one more character. `out` receives every double that was converted
/// (a failed call leaves the rest untouched, as `sscanf` does); the return value is the number
/// of assignments, the trailing `%c` included.
fn sscanf_doubles(str: &[u8], sep: u8, out: &mut [f64]) -> i32 {
    // str.data() is a C string: it ends at the first NUL.
    let end = str.iter().position(|&c| c == 0).unwrap_or(str.len());
    let s = &str[..end];
    let n = s.len();
    let mut i = 0usize;
    let mut assigned = 0i32;
    let count = out.len();
    for (k, slot) in out.iter_mut().enumerate() {
        // " %lf": the directive skips white space itself as well.
        while i < n && is_space_byte(s[i]) {
            i += 1;
        }
        if i == n {
            // input failure before the first conversion is EOF (-1)
            return if assigned == 0 { -1 } else { assigned };
        }
        let r = strtod_c(scanf_float_field(&s[i..]));
        if r.consumed == 0 {
            return assigned;
        }
        *slot = r.value;
        assigned += 1;
        i += r.consumed;
        if k + 1 < count {
            // " SEP "
            while i < n && is_space_byte(s[i]) {
                i += 1;
            }
            if i == n || s[i] != sep {
                return assigned;
            }
            i += 1;
        }
    }
    // " %c"
    while i < n && is_space_byte(s[i]) {
        i += 1;
    }
    if i < n {
        assigned += 1;
    }
    assigned
}

// ---------------------------------------------------------------------------
// std::hash / boost::hash_combine as Config.hpp uses them.
//
// FIDELITY-NOTE: the values of `ConfigOption::hash()` are those of libc++'s `std::hash`
// (identity for integers, the bit pattern for doubles with both zeroes hashing to 0,
// CityHash64 for strings) and of `boost::hash_combine` from Boost 1.81 on (`hash_mix`).
// Another standard library or an older Boost gives other numbers for the same option; the
// engine only compares hashes of one process with each other.
// ---------------------------------------------------------------------------

/// `std::hash<double>` (libc++ `__functional/hash.h`): -0.0 and 0.0 hash to 0, anything else
/// to its bit pattern.
fn std_hash_double(v: f64) -> u64 {
    if v == 0.0 {
        0
    } else {
        v.to_bits()
    }
}

/// `std::hash<int>`: the value converted to `size_t`.
fn std_hash_int(v: i32) -> u64 {
    v as i64 as u64
}

/// `boost::hash_combine(seed, v)` for a `size_t` v (boost/container_hash/hash.hpp,
/// `hash_detail::hash_mix` for 64 bits).
fn boost_hash_combine(seed: &mut u64, v: u64) {
    let mut x = seed.wrapping_add(0x9e3779b9).wrapping_add(v);
    const M: u64 = 0xe9846af9b1a615d;
    x ^= x >> 32;
    x = x.wrapping_mul(M);
    x ^= x >> 32;
    x = x.wrapping_mul(M);
    x ^= x >> 28;
    *seed = x;
}

/// `std::hash<std::string>` of libc++ on a 64 bit target: `__murmur2_or_cityhash<size_t, 64>`
/// (CityHash64) over the bytes.
fn std_hash_string(s: &[u8]) -> u64 {
    const K0: u64 = 0xc3a5c85c97cb3127;
    const K1: u64 = 0xb492b66fbe98f273;
    const K2: u64 = 0x9ae16a3b2f90404f;
    const K3: u64 = 0xc949d7c7509e6557;
    fn load64(s: &[u8], p: usize) -> u64 {
        u64::from_le_bytes([s[p], s[p + 1], s[p + 2], s[p + 3], s[p + 4], s[p + 5], s[p + 6], s[p + 7]])
    }
    fn load32(s: &[u8], p: usize) -> u32 {
        u32::from_le_bytes([s[p], s[p + 1], s[p + 2], s[p + 3]])
    }
    fn rotate(val: u64, shift: u32) -> u64 {
        if shift == 0 {
            val
        } else {
            (val >> shift) | (val << (64 - shift))
        }
    }
    fn shift_mix(val: u64) -> u64 {
        val ^ (val >> 47)
    }
    fn hash_len_16(u: u64, v: u64) -> u64 {
        const MUL: u64 = 0x9ddfea08eb382d69;
        let mut a = (u ^ v).wrapping_mul(MUL);
        a ^= a >> 47;
        let mut b = (v ^ a).wrapping_mul(MUL);
        b ^= b >> 47;
        b.wrapping_mul(MUL)
    }
    fn weak_hash_len_32_with_seeds(s: &[u8], p: usize, mut a: u64, mut b: u64) -> (u64, u64) {
        let w = load64(s, p);
        let x = load64(s, p + 8);
        let y = load64(s, p + 16);
        let z = load64(s, p + 24);
        a = a.wrapping_add(w);
        b = rotate(b.wrapping_add(a).wrapping_add(z), 21);
        let c = a;
        a = a.wrapping_add(x);
        a = a.wrapping_add(y);
        b = b.wrapping_add(rotate(a, 44));
        (a.wrapping_add(z), b.wrapping_add(c))
    }
    let len = s.len();
    let len64 = len as u64;
    if len <= 16 {
        // __hash_len_0_to_16
        if len > 8 {
            let a = load64(s, 0);
            let b = load64(s, len - 8);
            let sh = (len as u32) & 63;
            let rot = (b.wrapping_add(len64) >> sh) | (b.wrapping_add(len64) << (64 - sh));
            return hash_len_16(a, rot) ^ b;
        }
        if len >= 4 {
            let a = load32(s, 0);
            let b = load32(s, len - 4);
            // `__len + (__a << 3)`: the shift is done in 32 bits (no
            // _LIBCPP_ABI_FIX_CITYHASH_IMPLEMENTATION in the stable ABI).
            return hash_len_16(len64.wrapping_add(a.wrapping_shl(3) as u64), b as u64);
        }
        if len > 0 {
            let a = s[0] as u32;
            let b = s[len >> 1] as u32;
            let c = s[len - 1] as u32;
            let y = a.wrapping_add(b << 8);
            let z = (len as u32).wrapping_add(c << 2);
            return shift_mix((y as u64).wrapping_mul(K2) ^ (z as u64).wrapping_mul(K3)).wrapping_mul(K2);
        }
        return K2;
    }
    if len <= 32 {
        // __hash_len_17_to_32
        let a = load64(s, 0).wrapping_mul(K1);
        let b = load64(s, 8);
        let c = load64(s, len - 8).wrapping_mul(K2);
        let d = load64(s, len - 16).wrapping_mul(K0);
        return hash_len_16(
            rotate(a.wrapping_sub(b), 43).wrapping_add(rotate(c, 30)).wrapping_add(d),
            a.wrapping_add(rotate(b ^ K3, 20)).wrapping_sub(c).wrapping_add(len64),
        );
    }
    if len <= 64 {
        // __hash_len_33_to_64
        let mut z = load64(s, 24);
        let mut a = load64(s, 0).wrapping_add(len64.wrapping_add(load64(s, len - 16)).wrapping_mul(K0));
        let mut b = rotate(a.wrapping_add(z), 52);
        let mut c = rotate(a, 37);
        a = a.wrapping_add(load64(s, 8));
        c = c.wrapping_add(rotate(a, 7));
        a = a.wrapping_add(load64(s, 16));
        let vf = a.wrapping_add(z);
        let vs = b.wrapping_add(rotate(a, 31)).wrapping_add(c);
        a = load64(s, 16).wrapping_add(load64(s, len - 32));
        z = z.wrapping_add(load64(s, len - 8));
        b = rotate(a.wrapping_add(z), 52);
        c = rotate(a, 37);
        a = a.wrapping_add(load64(s, len - 24));
        c = c.wrapping_add(rotate(a, 7));
        a = a.wrapping_add(load64(s, len - 16));
        let wf = a.wrapping_add(z);
        let ws = b.wrapping_add(rotate(a, 31)).wrapping_add(c);
        let r = shift_mix(
            vf.wrapping_add(ws).wrapping_mul(K2).wrapping_add(wf.wrapping_add(vs).wrapping_mul(K0)),
        );
        return shift_mix(r.wrapping_mul(K0).wrapping_add(vs)).wrapping_mul(K2);
    }
    // For strings over 64 bytes the end is hashed first, then 64 byte chunks.
    let mut x = load64(s, len - 40);
    let mut y = load64(s, len - 16).wrapping_add(load64(s, len - 56));
    let mut z = hash_len_16(load64(s, len - 48).wrapping_add(len64), load64(s, len - 24));
    let mut v = weak_hash_len_32_with_seeds(s, len - 64, len64, z);
    let mut w = weak_hash_len_32_with_seeds(s, len - 32, y.wrapping_add(K1), x);
    x = x.wrapping_mul(K1).wrapping_add(load64(s, 0));
    let mut remaining = (len - 1) & !63usize;
    let mut p = 0usize;
    loop {
        x = rotate(x.wrapping_add(y).wrapping_add(v.0).wrapping_add(load64(s, p + 8)), 37).wrapping_mul(K1);
        y = rotate(y.wrapping_add(v.1).wrapping_add(load64(s, p + 48)), 42).wrapping_mul(K1);
        x ^= w.1;
        y = y.wrapping_add(v.0.wrapping_add(load64(s, p + 40)));
        z = rotate(z.wrapping_add(w.0), 33).wrapping_mul(K1);
        v = weak_hash_len_32_with_seeds(s, p, v.1.wrapping_mul(K1), x.wrapping_add(w.0));
        w = weak_hash_len_32_with_seeds(s, p + 32, z.wrapping_add(w.1), y.wrapping_add(load64(s, p + 16)));
        std::mem::swap(&mut z, &mut x);
        p += 64;
        remaining -= 64;
        if remaining == 0 {
            break;
        }
    }
    hash_len_16(
        hash_len_16(v.0, w.0).wrapping_add(shift_mix(y).wrapping_mul(K1)).wrapping_add(z),
        hash_len_16(v.1, w.1).wrapping_add(x),
    )
}

/// Config.hpp:50-56 `std::hash<Slic3r::FloatOrPercent>`.
fn std_hash_float_or_percent(v: &FloatOrPercent) -> u64 {
    let seed = std_hash_double(v.value);
    if v.percent {
        seed ^ 0x9e3779b9
    } else {
        seed
    }
}

/// Config.hpp:58-64 `std::hash<Slic3r::Vec2d>`.
fn std_hash_vec2d(v: &Vec2d) -> u64 {
    let mut seed = std_hash_double(v.x);
    boost_hash_combine(&mut seed, std_hash_double(v.y));
    seed
}

/// Config.hpp:66-73 `std::hash<Slic3r::Vec3d>`.
fn std_hash_vec3d(v: &Vec3d) -> u64 {
    let mut seed = std_hash_double(v.x);
    boost_hash_combine(&mut seed, std_hash_double(v.y));
    boost_hash_combine(&mut seed, std_hash_double(v.z));
    seed
}

/// Point.hpp:624-634 `std::hash<Slic3r::Vec2ds>` ("requseted by ConfigOptionPointsGroups").
fn std_hash_vec2ds(vec: &Vec2ds) -> u64 {
    let mut seed: u64 = 0;
    for element in vec {
        seed ^= std_hash_double(element.x)
            .wrapping_add(0x9e3779b9)
            .wrapping_add(seed << 6)
            .wrapping_add(seed >> 2);
        seed ^= std_hash_double(element.y)
            .wrapping_add(0x9e3779b9)
            .wrapping_add(seed << 6)
            .wrapping_add(seed >> 2);
    }
    seed
}

/// Point.hpp:636-647 `std::hash<std::vector<int>>`: each int goes through `std::hash<double>`.
fn std_hash_vector_int(vec: &[i32]) -> u64 {
    let mut seed: u64 = 0;
    for &element in vec {
        seed ^= std_hash_double(element as f64)
            .wrapping_add(0x9e3779b9)
            .wrapping_add(seed << 6)
            .wrapping_add(seed >> 2);
    }
    seed
}
// ---------------------------------------------------------------------------
// Config.hpp:76-263: keys, exceptions, option types, substitutions.
// ---------------------------------------------------------------------------

/// Config.hpp:79 `typedef std::string t_config_option_key;`
#[allow(non_camel_case_types)]
pub type t_config_option_key = String;
/// Config.hpp:80 `typedef std::vector<std::string> t_config_option_keys;`
#[allow(non_camel_case_types)]
pub type t_config_option_keys = Vec<String>;
/// Config.hpp:1999 `typedef std::vector<std::string> t_config_enum_names;`
#[allow(non_camel_case_types)]
pub type t_config_enum_names = Vec<String>;
/// Config.hpp:2001 `typedef std::map<std::string,int> t_config_enum_values;`
#[allow(non_camel_case_types)]
pub type t_config_enum_values = std::collections::BTreeMap<String, i32>;

/// Which class of Config.hpp:119-160 an error value stands for. The C++ classes all derive
/// from `ConfigurationError` (a `Slic3r::RuntimeError`); `catch` clauses that name one of the
/// derived classes become a test of this field.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum ConfigurationErrorKind {
    /// Config.hpp:120 `class ConfigurationError : public Slic3r::RuntimeError`
    ConfigurationError,
    /// Config.hpp:126 `class UnknownOptionException : public ConfigurationError`
    UnknownOptionException,
    /// Config.hpp:135 `class NoDefinitionException : public ConfigurationError`
    NoDefinitionException,
    /// Config.hpp:145 `class BadOptionTypeException : public ConfigurationError`
    BadOptionTypeException,
    /// Config.hpp:154 `class BadOptionValueException : public ConfigurationError`
    BadOptionValueException,
    /// `Slic3r::RuntimeError` thrown as such (Config.cpp:1505, 1530, 1555, 1575).
    RuntimeError,
    /// `boost::property_tree::ini_parser_error` out of `read_ini` (Config.cpp:1250, 1261).
    IniParserError,
}

/// Config.hpp:119-123: base for all exceptions thrown by the configuration layer. A thrown
/// exception is an `Err` of this type with the same `what()` text.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct ConfigurationError {
    pub kind: ConfigurationErrorKind,
    pub message: String,
}

impl ConfigurationError {
    /// Config.hpp:122 `using RuntimeError::RuntimeError;`
    pub fn new(message: impl Into<String>) -> Self {
        ConfigurationError { kind: ConfigurationErrorKind::ConfigurationError, message: message.into() }
    }

    /// An error of one of the derived classes.
    pub fn with_kind(kind: ConfigurationErrorKind, message: impl Into<String>) -> Self {
        ConfigurationError { kind, message: message.into() }
    }

    /// `std::exception::what()`.
    pub fn what(&self) -> &str {
        &self.message
    }

    /// `catch (const ConfigurationError &)`: every class of Config.hpp, not the plain
    /// `RuntimeError` and not the INI parser's error.
    pub fn is_configuration_error(&self) -> bool {
        !matches!(self.kind, ConfigurationErrorKind::RuntimeError | ConfigurationErrorKind::IniParserError)
    }
}

impl std::fmt::Display for ConfigurationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl std::error::Error for ConfigurationError {}

type CfgResult<T> = Result<T, ConfigurationError>;

/// Config.hpp:125-132: an unknown config option has been encountered.
pub struct UnknownOptionException;

impl UnknownOptionException {
    /// Config.hpp:128-129 `UnknownOptionException()`
    pub fn new() -> ConfigurationError {
        ConfigurationError::with_kind(ConfigurationErrorKind::UnknownOptionException, "Unknown option exception")
    }

    /// Config.hpp:130-131 `UnknownOptionException(const std::string &opt_key)`
    pub fn with_key(opt_key: &str) -> ConfigurationError {
        ConfigurationError::with_kind(
            ConfigurationErrorKind::UnknownOptionException,
            format!("Unknown option exception: {}", opt_key),
        )
    }
}

/// Config.hpp:134-142: the ConfigBase derived class does not provide a config definition.
pub struct NoDefinitionException;

impl NoDefinitionException {
    /// Config.hpp:138-139 `NoDefinitionException()`
    pub fn new() -> ConfigurationError {
        ConfigurationError::with_kind(ConfigurationErrorKind::NoDefinitionException, "No definition exception")
    }

    /// Config.hpp:140-141 `NoDefinitionException(const std::string &opt_key)`
    pub fn with_key(opt_key: &str) -> ConfigurationError {
        ConfigurationError::with_kind(
            ConfigurationErrorKind::NoDefinitionException,
            format!("No definition exception: {}", opt_key),
        )
    }
}

/// Config.hpp:144-151: an unsupported accessor was called on a config option.
pub struct BadOptionTypeException;

impl BadOptionTypeException {
    /// Config.hpp:148 `BadOptionTypeException()`
    pub fn new() -> ConfigurationError {
        ConfigurationError::with_kind(ConfigurationErrorKind::BadOptionTypeException, "Bad option type exception")
    }

    /// Config.hpp:149-150 `BadOptionTypeException(message)`
    pub fn with_message(message: impl Into<String>) -> ConfigurationError {
        ConfigurationError::with_kind(ConfigurationErrorKind::BadOptionTypeException, message)
    }
}

/// Config.hpp:153-160: an option has been deserialized from an invalid value.
pub struct BadOptionValueException;

impl BadOptionValueException {
    /// Config.hpp:157 `BadOptionValueException()`
    pub fn new() -> ConfigurationError {
        ConfigurationError::with_kind(ConfigurationErrorKind::BadOptionValueException, "Bad option value exception")
    }

    /// Config.hpp:158-159 `BadOptionValueException(message)`
    pub fn with_message(message: impl Into<String>) -> ConfigurationError {
        ConfigurationError::with_kind(ConfigurationErrorKind::BadOptionValueException, message)
    }
}

/// Config.hpp:162-202: type of a configuration value.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
#[repr(i32)]
pub enum ConfigOptionType {
    CoVectorType = 0x4000,
    CoNone = 0,
    /// single float
    CoFloat = 1,
    /// vector of floats
    CoFloats = 1 + 0x4000,
    /// single int
    CoInt = 2,
    /// vector of ints
    CoInts = 2 + 0x4000,
    /// single string
    CoString = 3,
    /// vector of strings
    CoStrings = 3 + 0x4000,
    /// percent value. Currently only used for infill.
    CoPercent = 4,
    /// percents value. Currently used for retract before wipe only.
    CoPercents = 4 + 0x4000,
    /// a fraction or an absolute value
    CoFloatOrPercent = 5,
    /// vector of the above
    CoFloatsOrPercents = 5 + 0x4000,
    /// single 2d point (Point2f). Currently not used.
    CoPoint = 6,
    /// vector of 2d points (Point2f).
    CoPoints = 6 + 0x4000,
    CoPoint3 = 7,
    /// single boolean value
    CoBool = 8,
    /// vector of boolean values
    CoBools = 8 + 0x4000,
    /// a generic enum
    CoEnum = 9,
    /// BBS: vector of enums
    CoEnums = 9 + 0x4000,
    CoPointsGroups = 10 + 0x4000,
    CoIntsGroups = 11 + 0x4000,
}

/// Config.hpp:204-208
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum ConfigOptionMode {
    ComSimple = 0,
    ComAdvanced,
    ComDevelop,
}

/// Config.hpp:245-249: when parsing a configuration value, if the old_value is not understood
/// by this version, it is being substituted with some default value. This structure serves to
/// inform the user about the substitutions having been done during file import.
pub struct ConfigSubstitution {
    /// `const ConfigOptionDef *opt_def`: the definitions live in a static ConfigDef.
    pub opt_def: Option<&'static ConfigOptionDef>,
    pub old_value: String,
    /// `ConfigOptionUniquePtr new_value` (Config.hpp:239-240, Config.cpp:248-250: the deleter
    /// only exists for the forward declaration and has no Rust counterpart).
    pub new_value: Option<Box<dyn ConfigOption>>,
}

impl ConfigSubstitution {
    /// Config.hpp:2433-2436 `operator<(const ConfigSubstitution&, const ConfigSubstitution&)`
    pub fn operator_lt(&self, rhs: &ConfigSubstitution) -> bool {
        let l = &self.opt_def.expect("ConfigSubstitution::opt_def is null").opt_key;
        let r = &rhs.opt_def.expect("ConfigSubstitution::opt_def is null").opt_key;
        l < r || (l == r && self.old_value < rhs.old_value)
    }

    /// Config.hpp:2437-2439 `operator==(const ConfigSubstitution&, const ConfigSubstitution&)`:
    /// the same definition object and the same old value.
    pub fn operator_eq(&self, rhs: &ConfigSubstitution) -> bool {
        let same_def = match (self.opt_def, rhs.opt_def) {
            (Some(l), Some(r)) => std::ptr::eq(l, r),
            (None, None) => true,
            _ => false,
        };
        same_def && self.old_value == rhs.old_value
    }
}

/// Config.hpp:251 `using ConfigSubstitutions = std::vector<ConfigSubstitution>;`
pub type ConfigSubstitutions = Vec<ConfigSubstitution>;

/// Config.hpp:253-263: filled in by ConfigBase::set_deserialize_raw(), which based on "rule"
/// either bails out or performs substitutions when encountering an unknown configuration value.
pub struct ConfigSubstitutionContext {
    pub rule: ForwardCompatibilitySubstitutionRule,
    pub substitutions: ConfigSubstitutions,
    pub unrecogized_keys: Vec<String>,
}

impl ConfigSubstitutionContext {
    /// Config.hpp:257 `ConfigSubstitutionContext(ForwardCompatibilitySubstitutionRule rl)`
    pub fn new(rl: ForwardCompatibilitySubstitutionRule) -> Self {
        ConfigSubstitutionContext { rule: rl, substitutions: Vec::new(), unrecogized_keys: Vec::new() }
    }

    /// Config.hpp:258 `bool empty() const throw()`
    pub fn empty(&self) -> bool {
        self.substitutions.is_empty()
    }
}

// ---------------------------------------------------------------------------
// Config.hpp:265-386: ConfigOption, ConfigOptionVectorBase.
// ---------------------------------------------------------------------------

/// Config.hpp:265-302: a generic value of a configuration option. The virtual methods of the
/// C++ class are the methods of this trait; the concrete classes below carry the C++ names.
///
/// `Send + Sync`: a ConfigDef holds default options and lives in a `static`, as the C++
/// definitions are static objects; every option class is plain data.
pub trait ConfigOption: std::any::Any + Send + Sync {
    /// Config.hpp:270 `virtual ConfigOptionType type() const = 0;`
    fn r#type(&self) -> ConfigOptionType;
    /// Config.hpp:271 `virtual std::string serialize() const = 0;` (some classes throw).
    fn serialize(&self) -> CfgResult<String>;
    /// Config.hpp:272 `virtual bool deserialize(const std::string &str, bool append = false) = 0;`
    fn deserialize(&mut self, str: &str, append: bool) -> CfgResult<bool>;
    /// Config.hpp:273 `virtual ConfigOption* clone() const = 0;`
    fn clone(&self) -> Box<dyn ConfigOption>;
    /// Config.hpp:275: set a value from a ConfigOption. The two options should be compatible.
    fn set(&mut self, option: &dyn ConfigOption) -> CfgResult<()>;
    /// Config.hpp:276 `virtual int getInt() const`
    fn get_int(&self) -> CfgResult<i32> {
        Err(BadOptionTypeException::with_message("Calling ConfigOption::getInt on a non-int ConfigOption"))
    }
    /// Config.hpp:277 `virtual double getFloat() const`
    fn get_float(&self) -> CfgResult<f64> {
        Err(BadOptionTypeException::with_message("Calling ConfigOption::getFloat on a non-float ConfigOption"))
    }
    /// Config.hpp:278 `virtual bool getBool() const`
    fn get_bool(&self) -> CfgResult<bool> {
        Err(BadOptionTypeException::with_message("Calling ConfigOption::getBool on a non-boolean ConfigOption"))
    }
    /// Config.hpp:279 `virtual void setInt(int)`
    fn set_int(&mut self, _val: i32) -> CfgResult<()> {
        Err(BadOptionTypeException::with_message("Calling ConfigOption::setInt on a non-int ConfigOption"))
    }
    /// Config.hpp:280 `virtual bool operator==(const ConfigOption &rhs) const = 0;`
    fn operator_eq(&self, rhs: &dyn ConfigOption) -> CfgResult<bool>;
    /// Config.hpp:281 `bool operator!=(const ConfigOption &rhs) const`
    fn operator_ne(&self, rhs: &dyn ConfigOption) -> CfgResult<bool> {
        Ok(!self.operator_eq(rhs)?)
    }
    /// Config.hpp:282 `virtual size_t hash() const throw() = 0;`
    fn hash(&self) -> u64;
    /// Config.hpp:283 `bool is_scalar() const`
    fn is_scalar(&self) -> bool {
        (self.r#type() as i32 & ConfigOptionType::CoVectorType as i32) == 0
    }
    /// Config.hpp:284 `bool is_vector() const`
    fn is_vector(&self) -> bool {
        !self.is_scalar()
    }
    /// Config.hpp:286: if this option is nullable, then it may have its value or values set to nil.
    fn nullable(&self) -> bool {
        false
    }
    /// Config.hpp:288: a scalar is nil, or all values of a vector are nil.
    fn is_nil(&self) -> bool {
        false
    }
    /// Config.hpp:291-294: is this option overridden by another option?
    /// An option overrides another option if it is not nil and not equal.
    fn overriden_by(&self, rhs: &dyn ConfigOption) -> CfgResult<bool> {
        // assert(!self.nullable() && !rhs.nullable()): an `assert` of the C++, compiled out in the reference build (-DNDEBUG).
        self.operator_ne(rhs)
    }
    /// Config.hpp:296-301: apply an override option, possibly a nullable one.
    ///
    /// FIDELITY-NOTE: the C++ body is `*this = *rhs`, the copy assignment of the base class
    /// `ConfigOption`, which has no data members: the value of a scalar option is not changed
    /// and `true` is returned whenever the two differ. That is what this does.
    fn apply_override(&mut self, rhs: &dyn ConfigOption, _default_index: &mut Vec<i32>) -> CfgResult<bool> {
        if self.operator_eq(rhs)? {
            return Ok(false);
        }
        Ok(true)
    }

    // --- what `static_cast` / `dynamic_cast` do in C++ ---

    /// `dynamic_cast` to a concrete class: `opt.as_any().downcast_ref::<ConfigOptionFloat>()`.
    fn as_any(&self) -> &dyn std::any::Any;
    /// Mutable `dynamic_cast` to a concrete class.
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;
    /// `dynamic_cast<const ConfigOptionVectorBase*>(this)`.
    fn as_vector_base(&self) -> Option<&dyn ConfigOptionVectorBase> {
        None
    }
    /// `dynamic_cast<ConfigOptionVectorBase*>(this)`.
    fn as_vector_base_mut(&mut self) -> Option<&mut dyn ConfigOptionVectorBase> {
        None
    }
    /// `static_cast<const ConfigOptionSingle<T>*>(this)->value` as `&T` behind `Any`.
    fn single_value_any(&self) -> Option<&dyn std::any::Any> {
        None
    }
    /// `static_cast<const ConfigOptionVector<T>*>(this)->values` as `&Vec<T>` behind `Any`.
    fn vector_values_any(&self) -> Option<&dyn std::any::Any> {
        None
    }
    /// `static_cast<ConfigOptionVector<T>*>(this)->values` as `&mut Vec<T>` behind `Any`.
    fn vector_values_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        None
    }
}

/// `clonable_ptr<const ConfigOption>` and the copy constructors that clone options.
impl Clone for Box<dyn ConfigOption> {
    fn clone(&self) -> Self {
        ConfigOption::clone(&**self)
    }
}

/// Config.hpp:304-305 `typedef ConfigOption* ConfigOptionPtr; typedef const ConfigOption* ConfigOptionConstPtr;`
pub type ConfigOptionPtr = Box<dyn ConfigOption>;

/// `TYPE::static_type()` of the concrete option classes, for `option<TYPE>()`.
pub trait ConfigOptionStaticType: ConfigOption + Sized {
    fn static_type() -> ConfigOptionType;
}

/// Config.hpp:342-386: value of a vector valued option (bools, ints, floats, strings, points).
pub trait ConfigOptionVectorBase: ConfigOption {
    /// Config.hpp:346: currently used only to initialize the PlaceholderParser.
    fn vserialize(&self) -> CfgResult<Vec<String>>;
    /// Config.hpp:351 `virtual void set(const std::vector<const ConfigOption*> &rhs) = 0;`
    /// Set from a vector of ConfigOptions. If the rhs ConfigOption is scalar, then its value is
    /// used, otherwise for each of rhs, the first value of a vector is used.
    fn set_from_options(&mut self, rhs: &[&dyn ConfigOption]) -> CfgResult<()>;
    /// Config.hpp:354: set a single vector item from either a scalar option or the first value
    /// of a vector option.
    fn set_at(&mut self, rhs: &dyn ConfigOption, i: usize, j: usize) -> CfgResult<()>;
    /// Config.hpp:356 `virtual void set_at_to_nil(size_t i) = 0;`
    fn set_at_to_nil(&mut self, i: usize);
    /// Config.hpp:357 `virtual void append(const ConfigOption *rhs) = 0;`
    fn append(&mut self, rhs: &dyn ConfigOption) -> CfgResult<()>;
    /// Config.hpp:358 `virtual void set(const ConfigOption* rhs, size_t start, size_t len) = 0;`
    fn set_range(&mut self, rhs: &dyn ConfigOption, start: usize, len: usize) -> CfgResult<()>;
    /// Config.hpp:359
    fn set_with_restore(&mut self, rhs: &dyn ConfigOption, restore_index: &mut Vec<i32>, stride: i32) -> CfgResult<()>;
    /// Config.hpp:360. `rhs` is `const` in C++ and is resized through a `const_cast` when
    /// `skip_error` is set, hence the mutable borrow.
    fn set_with_restore_2(
        &mut self,
        key: &str,
        rhs: &mut dyn ConfigOption,
        restore_index: &mut Vec<i32>,
        start: i32,
        len: i32,
        skip_error: bool,
    ) -> CfgResult<()>;
    /// Config.hpp:361
    fn set_only_diff(&mut self, key: &str, rhs: &dyn ConfigOption, diff_index: &mut Vec<i32>, stride: i32) -> CfgResult<()>;
    /// Config.hpp:362
    fn set_to_index(&mut self, rhs: &dyn ConfigOption, dest_index: &mut Vec<i32>, stride: i32) -> CfgResult<()>;
    /// Config.hpp:363
    fn set_with_nil(&mut self, rhs: &dyn ConfigOption, inherits: &dyn ConfigOption, stride: i32) -> CfgResult<()>;
    /// Config.hpp:364
    fn set_with_default(&mut self, inherits: &dyn ConfigOption) -> CfgResult<()>;
    /// Config.hpp:366: resize the vector of values, copy the newly added values from
    /// opt_default if provided.
    fn resize(&mut self, n: usize, opt_default: Option<&dyn ConfigOption>) -> CfgResult<()>;
    /// Config.hpp:368: clear the values vector.
    fn clear(&mut self);
    /// Config.hpp:371: get size of this vector.
    fn size(&self) -> usize;
    /// Config.hpp:373: is this vector empty?
    fn empty(&self) -> bool;
    /// Config.hpp:375 `virtual bool is_nil(size_t idx) const = 0;`
    fn is_nil_at(&self, idx: usize) -> bool;
    /// Config.hpp:385: used to verify type compatibility when assigning to / from a scalar
    /// ConfigOption. `coPointsGroups` and `coIntsGroups` give 10 and 11, which no scalar
    /// option has, so the value stays an integer.
    fn scalar_type(&self) -> i32 {
        self.r#type() as i32 - ConfigOptionType::CoVectorType as i32
    }
}

/// The element types `T` of `ConfigOptionSingle<T>` / `ConfigOptionVector<T>`.
pub trait ConfigValue: Clone + PartialEq + 'static {
    /// `static_cast<const ConfigOptionSingle<T>*>(opt)->value` after the type of `opt` was
    /// compared with `scalar_type()`.
    fn from_single(opt: &dyn ConfigOption) -> Self;
    /// `std::hash<T>{}(value)`.
    fn std_hash(&self) -> u64;
    /// `T()`: what `std::vector<T>::resize(n)` fills in.
    fn value_initialized() -> Self;
}

fn single_ref<T: 'static>(opt: &dyn ConfigOption) -> &T {
    opt.single_value_any()
        .and_then(|a| a.downcast_ref::<T>())
        .expect("static_cast<ConfigOptionSingle<T>*>: the option holds another value type")
}

impl ConfigValue for f64 {
    fn from_single(opt: &dyn ConfigOption) -> Self {
        *single_ref::<f64>(opt)
    }
    fn std_hash(&self) -> u64 {
        std_hash_double(*self)
    }
    fn value_initialized() -> Self {
        0.0
    }
}

impl ConfigValue for i32 {
    fn from_single(opt: &dyn ConfigOption) -> Self {
        // ConfigOptionInt, ConfigOptionEnumGeneric: the int itself. ConfigOptionEnum<T>: the
        // enum read as an int.
        match opt.single_value_any().and_then(|a| a.downcast_ref::<i32>()) {
            Some(v) => *v,
            None => opt.get_int().expect("static_cast<ConfigOptionSingle<int>*>: not an int option"),
        }
    }
    fn std_hash(&self) -> u64 {
        std_hash_int(*self)
    }
    fn value_initialized() -> Self {
        0
    }
}

impl ConfigValue for String {
    fn from_single(opt: &dyn ConfigOption) -> Self {
        Clone::clone(single_ref::<String>(opt))
    }
    fn std_hash(&self) -> u64 {
        std_hash_string(self.as_bytes())
    }
    fn value_initialized() -> Self {
        String::new()
    }
}

impl ConfigValue for FloatOrPercent {
    fn from_single(opt: &dyn ConfigOption) -> Self {
        // FIDELITY-NOTE: ConfigOptionFloatOrPercent is a ConfigOptionSingle<double> followed by
        // `bool percent`; the C++ cast to ConfigOptionSingle<FloatOrPercent> reads both through
        // the layout of FloatOrPercent {double, bool}. The same two fields are read here.
        let o = opt
            .as_any()
            .downcast_ref::<ConfigOptionFloatOrPercent>()
            .expect("static_cast<ConfigOptionSingle<FloatOrPercent>*>: not a ConfigOptionFloatOrPercent");
        FloatOrPercent { value: o.value, percent: o.percent }
    }
    fn std_hash(&self) -> u64 {
        std_hash_float_or_percent(self)
    }
    fn value_initialized() -> Self {
        FloatOrPercent::new()
    }
}

impl ConfigValue for Vec2d {
    fn from_single(opt: &dyn ConfigOption) -> Self {
        *single_ref::<Vec2d>(opt)
    }
    fn std_hash(&self) -> u64 {
        std_hash_vec2d(self)
    }
    fn value_initialized() -> Self {
        Vec2d::new(0.0, 0.0)
    }
}

impl ConfigValue for Vec3d {
    fn from_single(opt: &dyn ConfigOption) -> Self {
        *single_ref::<Vec3d>(opt)
    }
    fn std_hash(&self) -> u64 {
        std_hash_vec3d(self)
    }
    fn value_initialized() -> Self {
        Vec3d::new(0.0, 0.0, 0.0)
    }
}

impl ConfigValue for Vec2ds {
    fn from_single(_opt: &dyn ConfigOption) -> Self {
        unreachable!("no scalar option has the type coPointsGroups - coVectorType")
    }
    fn std_hash(&self) -> u64 {
        std_hash_vec2ds(self)
    }
    fn value_initialized() -> Self {
        Vec::new()
    }
}

impl ConfigValue for Vec<i32> {
    fn from_single(_opt: &dyn ConfigOption) -> Self {
        unreachable!("no scalar option has the type coIntsGroups - coVectorType")
    }
    fn std_hash(&self) -> u64 {
        std_hash_vector_int(self)
    }
    fn value_initialized() -> Self {
        Vec::new()
    }
}

impl ConfigValue for u8 {
    fn from_single(opt: &dyn ConfigOption) -> Self {
        // ConfigOptionVector<unsigned char> reads the `bool` of a ConfigOptionBool as a byte.
        *single_ref::<bool>(opt) as u8
    }
    fn std_hash(&self) -> u64 {
        *self as u64
    }
    fn value_initialized() -> Self {
        0
    }
}

impl ConfigValue for bool {
    fn from_single(opt: &dyn ConfigOption) -> Self {
        *single_ref::<bool>(opt)
    }
    fn std_hash(&self) -> u64 {
        *self as u64
    }
    fn value_initialized() -> Self {
        false
    }
}

/// Config.hpp:307-340 `template <class T> class ConfigOptionSingle : public ConfigOption`:
/// the method bodies, shared by the scalar classes (each holds its own `value`).
pub struct ConfigOptionSingle<T>(std::marker::PhantomData<T>);

impl<T: ConfigValue> ConfigOptionSingle<T> {
    /// Config.hpp:315-321 `void set(const ConfigOption *rhs) override`
    pub fn set(value: &mut T, this_type: ConfigOptionType, rhs: &dyn ConfigOption) -> CfgResult<()> {
        if rhs.r#type() != this_type {
            return Err(ConfigurationError::new("ConfigOptionSingle: Assigning an incompatible type"));
        }
        *value = T::from_single(rhs);
        Ok(())
    }

    /// Config.hpp:323-329 `bool operator==(const ConfigOption &rhs) const override`
    pub fn operator_eq(value: &T, this_type: ConfigOptionType, rhs: &dyn ConfigOption) -> CfgResult<bool> {
        if rhs.r#type() != this_type {
            return Err(ConfigurationError::new("ConfigOptionSingle: Comparing incompatible types"));
        }
        Ok(*value == T::from_single(rhs))
    }

    /// Config.hpp:335 `size_t hash() const throw() override`
    pub fn hash(value: &T) -> u64 {
        value.std_hash()
    }
}

fn vector_ref<T: 'static>(opt: &dyn ConfigOption) -> &Vec<T> {
    opt.vector_values_any()
        .and_then(|a| a.downcast_ref::<Vec<T>>())
        .expect("static_cast<ConfigOptionVector<T>*>: the option holds another value type")
}

fn vector_mut<T: 'static>(opt: &mut dyn ConfigOption) -> &mut Vec<T> {
    opt.vector_values_any_mut()
        .and_then(|a| a.downcast_mut::<Vec<T>>())
        .expect("static_cast<ConfigOptionVector<T>*>: the option holds another value type")
}

fn is_nil_of(opt: &dyn ConfigOption, idx: usize) -> bool {
    opt.as_vector_base().expect("not a ConfigOptionVectorBase").is_nil_at(idx)
}

/// Config.hpp:388-820 `template <class T> class ConfigOptionVector : public ConfigOptionVectorBase`:
/// the method bodies, shared by the vector classes (each holds its own `values`). `this_type`
/// is `this->type()`; `nil` is what the class's `set_at_to_nil()` stores (`None`: the base
/// class's empty body).
pub struct ConfigOptionVector<T>(std::marker::PhantomData<T>);

impl<T: ConfigValue> ConfigOptionVector<T> {
    fn scalar_type(this_type: ConfigOptionType) -> i32 {
        this_type as i32 - ConfigOptionType::CoVectorType as i32
    }

    /// Config.hpp:725 `void clear() override { this->values.clear(); }`
    pub fn clear(values: &mut Vec<T>) {
        values.clear();
    }

    /// Config.hpp:726 `size_t size() const override { return this->values.size(); }`
    pub fn size(values: &[T]) -> usize {
        values.len()
    }

    /// Config.hpp:727 `bool empty() const override { return this->values.empty(); }`
    pub fn empty(values: &[T]) -> bool {
        values.is_empty()
    }

    /// Config.hpp:400-406 `void set(const ConfigOption *rhs) override`
    pub fn set(values: &mut Vec<T>, this_type: ConfigOptionType, rhs: &dyn ConfigOption) -> CfgResult<()> {
        if rhs.r#type() != this_type {
            return Err(ConfigurationError::new("ConfigOptionVector: Assigning an incompatible type"));
        }
        *values = Clone::clone(vector_ref::<T>(rhs));
        Ok(())
    }

    /// Config.hpp:412-427 `void set(const std::vector<const ConfigOption*> &rhs) override`
    pub fn set_from_options(values: &mut Vec<T>, this_type: ConfigOptionType, rhs: &[&dyn ConfigOption]) -> CfgResult<()> {
        values.clear();
        values.reserve(rhs.len());
        for opt in rhs {
            if opt.r#type() == this_type {
                let other = vector_ref::<T>(*opt);
                if other.is_empty() {
                    return Err(ConfigurationError::new("ConfigOptionVector::set(): Assigning from an empty vector"));
                }
                values.push(Clone::clone(&other[0]));
            } else if opt.r#type() as i32 == Self::scalar_type(this_type) {
                values.push(T::from_single(*opt));
            } else {
                return Err(ConfigurationError::new("ConfigOptionVector::set():: Assigning an incompatible type"));
            }
        }
        Ok(())
    }

    /// Config.hpp:431-450 `void set_at(const ConfigOption *rhs, size_t i, size_t j) override`
    pub fn set_at(values: &mut Vec<T>, this_type: ConfigOptionType, rhs: &dyn ConfigOption, i: usize, j: usize) -> CfgResult<()> {
        // It is expected that the vector value has at least one value, which is the default, if not overwritten.
        assert!(!values.is_empty());
        if values.len() <= i {
            // Resize this vector, fill in the new vector fields with the copy of the first field.
            let v = Clone::clone(&values[0]);
            values.resize(i + 1, v);
        }
        if rhs.r#type() == this_type {
            // Assign the first value of the rhs vector.
            let other = vector_ref::<T>(rhs);
            if other.is_empty() {
                return Err(ConfigurationError::new("ConfigOptionVector::set_at(): Assigning from an empty vector"));
            }
            values[i] = Clone::clone(Self::get_at(other, j));
        } else if rhs.r#type() as i32 == Self::scalar_type(this_type) {
            values[i] = T::from_single(rhs);
        } else {
            return Err(ConfigurationError::new("ConfigOptionVector::set_at(): Assigning an incompatible type"));
        }
        Ok(())
    }

    /// Config.hpp:455-467 `void append(const ConfigOption *rhs) override`
    pub fn append(values: &mut Vec<T>, this_type: ConfigOptionType, rhs: &dyn ConfigOption) -> CfgResult<()> {
        if rhs.r#type() == this_type {
            let other = vector_ref::<T>(rhs);
            if other.is_empty() {
                return Err(ConfigurationError::new("ConfigOptionVector::append(): append an empty vector"));
            }
            values.extend(other.iter().cloned());
        } else if rhs.r#type() as i32 == Self::scalar_type(this_type) {
            values.push(T::from_single(rhs));
        } else {
            return Err(ConfigurationError::new("ConfigOptionVector::append(): append an incompatible type"));
        }
        Ok(())
    }

    /// Config.hpp:471-487 `void set(const ConfigOption* rhs, size_t start, size_t len) override`:
    /// set from a range of another vector option.
    pub fn set_range(values: &mut Vec<T>, this_type: ConfigOptionType, rhs: &dyn ConfigOption, start: usize, len: usize) -> CfgResult<()> {
        // It is expected that the vector value has at least one value, which is the default, if not overwritten.
        assert!(!values.is_empty());
        let v = Clone::clone(&values[0]);
        values.resize(len, v);
        if rhs.r#type() == this_type {
            let other = vector_ref::<T>(rhs);
            if other.len() < start.wrapping_add(len) {
                return Err(ConfigurationError::new("ConfigOptionVector::set_with(): Assigning from an vector with invalid size"));
            }
            for i in 0..len {
                values[i] = Clone::clone(Self::get_at(other, start + i));
            }
        } else {
            return Err(ConfigurationError::new("ConfigOptionVector::set_with(): Assigning an incompatible type"));
        }
        Ok(())
    }

    /// Config.hpp:492-515: set a item related with extruder variants when loading config from
    /// 3mf, restore the non change values to system config.
    /// rhs: item from systemconfig(inherits); restore_index: which index in this vector need
    /// to be restored.
    pub fn set_with_restore(
        values: &mut Vec<T>,
        this_type: ConfigOptionType,
        rhs: &dyn ConfigOption,
        restore_index: &mut Vec<i32>,
        stride: i32,
    ) -> CfgResult<()> {
        if rhs.r#type() == this_type {
            // backup original ones
            let backup_values: Vec<T> = Clone::clone(values);
            let other = vector_ref::<T>(rhs);
            *values = Clone::clone(other);

            if other.len() != restore_index.len().wrapping_mul(stride as usize) {
                return Err(ConfigurationError::new(
                    "ConfigOptionVector::set_with_restore(): Assigning from an vector with invalid restore_index size",
                ));
            }

            let stride_u = stride as usize;
            for i in 0..restore_index.len() {
                if restore_index[i] != -1 {
                    for j in 0..stride_u {
                        let src = (restore_index[i].wrapping_mul(stride) as usize).wrapping_add(j);
                        if src < backup_values.len() {
                            values[i * stride_u + j] = Clone::clone(&backup_values[src]);
                        }
                    }
                }
            }
            Ok(())
        } else {
            Err(ConfigurationError::new("ConfigOptionVector::set_with_restore(): Assigning an incompatible type"))
        }
    }

    /// Config.hpp:522-568: set a item related with extruder variants when loading config from
    /// filament json, replace the original filament items.
    /// rhs: item from seperate filament config; restore_index: which index in this vector need
    /// to be restored; start: which index in this vector need to be replaced; len: how many
    /// items in this vector need to be replaced.
    pub fn set_with_restore_2(
        values: &mut Vec<T>,
        this_type: ConfigOptionType,
        key: &str,
        rhs: &mut dyn ConfigOption,
        restore_index: &mut Vec<i32>,
        start: i32,
        mut len: i32,
        skip_error: bool,
    ) -> CfgResult<()> {
        if rhs.r#type() == this_type {
            // backup original ones
            let backup_values: Vec<T> = Clone::clone(values);
            let start_u = start as usize;

            if values.len() < start_u {
                let v = Clone::clone(&values[0]);
                values.resize(start_u, v);
            } else {
                if values.len() < (start.wrapping_add(len)) as usize {
                    len = (values.len().wrapping_sub(start_u)) as i32;
                }
                // erase the original ones
                if len > 0 {
                    values.drain(start_u..start_u + len as usize);
                }
            }

            // Assign the new value from the rhs vector.
            let other = vector_mut::<T>(rhs);

            if other.len() != restore_index.len() {
                if skip_error {
                    let default_v = Clone::clone(&other[0]);
                    other.resize(restore_index.len(), default_v);
                } else {
                    return Err(ConfigurationError::new(format!(
                        "ConfigOptionVector::set_with_restore_2(): Assigning from an vector with invalid restore_index size: key={}",
                        key
                    )));
                }
            }

            for i in 0..restore_index.len() {
                if restore_index[i] != -1 && (restore_index[i] as usize) < backup_values.len() {
                    values.insert(start_u + i, Clone::clone(&backup_values[restore_index[i] as usize]));
                } else {
                    values.insert(start_u + i, Clone::clone(&other[i]));
                }
            }
            Ok(())
        } else {
            Err(ConfigurationError::new(format!(
                "ConfigOptionVector::set_with_restore_2(): Assigning an incompatible type: key={}",
                key
            )))
        }
    }

    /// Config.hpp:573-597: set a item related with extruder variants when loading user config,
    /// only set the different value of some extruder.
    /// rhs: item from user config; diff_index: which index in this vector need to be set.
    pub fn set_only_diff(
        values: &mut Vec<T>,
        this_type: ConfigOptionType,
        key: &str,
        rhs: &dyn ConfigOption,
        diff_index: &mut Vec<i32>,
        stride: i32,
    ) -> CfgResult<()> {
        if rhs.r#type() == this_type {
            let other = vector_ref::<T>(rhs);

            if values.len() != diff_index.len().wrapping_mul(stride as usize) {
                return Err(ConfigurationError::new(format!(
                    "ConfigOptionVector::set_only_diff(): Assigning from an vector with invalid diff_index size: key={}",
                    key
                )));
            }

            let stride_u = stride as usize;
            for i in 0..diff_index.len() {
                if diff_index[i] != -1 {
                    for j in 0..stride_u {
                        let base = diff_index[i].wrapping_mul(stride) as usize;
                        if base.wrapping_add(j) < other.len() && !is_nil_of(rhs, base) {
                            values[i * stride_u + j] = Clone::clone(&other[base + j]);
                        }
                    }
                }
            }
            Ok(())
        } else {
            Err(ConfigurationError::new(format!(
                "ConfigOptionVector::set_only_diff(): Assigning an incompatible type: key={}",
                key
            )))
        }
    }

    /// Config.hpp:602-620: set a item related with extruder variants when apply static config
    /// with dynamic config.
    /// rhs: item from dynamic config; dest_index: which index in this vector need to be used.
    pub fn set_to_index(
        values: &mut Vec<T>,
        this_type: ConfigOptionType,
        rhs: &dyn ConfigOption,
        dest_index: &mut Vec<i32>,
        stride: i32,
    ) -> CfgResult<()> {
        if rhs.r#type() == this_type {
            let other = vector_ref::<T>(rhs);
            let v = Clone::clone(&other[0]);
            values.resize(dest_index.len(), v);

            let stride_u = stride as usize;
            for i in 0..dest_index.len() {
                for j in 0..stride_u {
                    let d = dest_index[i] as usize;
                    if d < other.len() && !is_nil_of(rhs, d) {
                        values[i * stride_u + j] = Clone::clone(&other[(dest_index[i].wrapping_mul(stride) as usize) + j]);
                    }
                }
            }
            Ok(())
        } else {
            Err(ConfigurationError::new("ConfigOptionVector::set_with_index(): Assigning an incompatible type"))
        }
    }

    /// Config.hpp:626-657: set a item related with extruder variants when saving user config,
    /// set the non-diff value of some extruder to nill.
    /// rhs: item from userconfig; inherits: item from inherit config.
    pub fn set_with_nil(
        values: &mut Vec<T>,
        this_type: ConfigOptionType,
        nil: Option<T>,
        rhs: &dyn ConfigOption,
        inherits: &dyn ConfigOption,
        stride: i32,
    ) -> CfgResult<()> {
        if rhs.r#type() == this_type && inherits.r#type() == this_type {
            let rhs_opt = vector_ref::<T>(rhs);
            let inherits_opt = vector_ref::<T>(inherits);

            if inherits_opt.len() != rhs_opt.len() {
                return Err(ConfigurationError::new("ConfigOptionVector::set_with_nil(): rhs size different with inherits size"));
            }

            if values.len() != inherits_opt.len() {
                let front = Clone::clone(values.first().expect("ConfigOptionVector::set_with_nil(): this->values is empty"));
                values.resize(inherits_opt.len(), front);
            }

            let stride_u = stride as usize;
            let mut i = 0usize;
            while i < inherits_opt.len() {
                let mut set_nil = true;
                for j in 0..stride_u {
                    if inherits_opt[i + j] != rhs_opt[i + j] {
                        set_nil = false;
                        break;
                    }
                }

                for j in 0..stride_u {
                    if set_nil {
                        // this->set_at_to_nil(i + j)
                        if let Some(nil) = &nil {
                            values[i + j] = Clone::clone(nil);
                        }
                    } else {
                        values[i + j] = Clone::clone(&rhs_opt[i + j]);
                    }
                }
                i += stride_u;
            }
            Ok(())
        } else {
            Err(ConfigurationError::new("ConfigOptionVector::set_with_nil(): Assigning an incompatible type"))
        }
    }

    /// Config.hpp:662-679: set a item related with extruder variants when load user config,
    /// set the missed value of some extruder to default ones from inherits.
    pub fn set_with_default(values: &mut Vec<T>, this_type: ConfigOptionType, inherits: &dyn ConfigOption) -> CfgResult<()> {
        if inherits.r#type() == this_type {
            let inherits_opt = vector_ref::<T>(inherits);

            if inherits_opt.len() <= values.len() {
                return Ok(());
            }
            let delta = inherits_opt.len() - values.len();
            // this->values.resize(inherits->size(), this->values.front()): every added element
            // is overwritten below, so the fill value does not show.
            let fill = match values.first() {
                Some(v) => Clone::clone(v),
                None => T::value_initialized(),
            };
            values.resize(inherits_opt.len(), fill);

            for i in 0..delta {
                let index = inherits_opt.len() - delta + i;
                values[index] = Clone::clone(&inherits_opt[index]);
            }
            Ok(())
        } else {
            Err(ConfigurationError::new("ConfigOptionVector::set_with_default(): Assigning an incompatible type"))
        }
    }

    /// Config.hpp:681-685 `const T& get_at(size_t i) const`
    pub fn get_at(values: &[T], i: usize) -> &T {
        assert!(!values.is_empty());
        if i < values.len() {
            &values[i]
        } else {
            &values[0]
        }
    }

    /// Config.hpp:687 `T& get_at(size_t i)`
    pub fn get_at_mut(values: &mut [T], i: usize) -> &mut T {
        assert!(!values.is_empty());
        if i < values.len() {
            &mut values[i]
        } else {
            &mut values[0]
        }
    }

    /// Config.hpp:692-722: resize this vector by duplicating the first value. If the current
    /// vector is empty, the default value is used instead. BBS: support scaler opt_default.
    pub fn resize(values: &mut Vec<T>, this_type: ConfigOptionType, n: usize, opt_default: Option<&dyn ConfigOption>) -> CfgResult<()> {
        if n == 0 {
            values.clear();
        } else if n < values.len() {
            values.truncate(n);
        } else if n > values.len() {
            if values.is_empty() {
                match opt_default {
                    None => {
                        return Err(ConfigurationError::new("ConfigOptionVector::resize(): No default value provided."));
                    }
                    Some(opt_default) if opt_default.is_vector() => {
                        if opt_default.r#type() != this_type {
                            return Err(ConfigurationError::new("ConfigOptionVector::resize(): Extending with an incompatible type."));
                        }
                        let front = Clone::clone(
                            vector_ref::<T>(opt_default).first().expect("ConfigOptionVector::resize(): the default vector is empty"),
                        );
                        values.resize(n, front);
                    }
                    Some(opt_default) => {
                        if opt_default.r#type() as i32 != Self::scalar_type(this_type) {
                            return Err(ConfigurationError::new("ConfigOptionVector::resize(): Extending with an incompatible type."));
                        }
                        values.resize(n, T::from_single(opt_default));
                    }
                }
            } else {
                // Resize by duplicating the first value.
                let front = Clone::clone(&values[0]);
                values.resize(n, front);
            }
        }
        Ok(())
    }

    /// Config.hpp:729-735 `bool operator==(const ConfigOption &rhs) const override`
    pub fn operator_eq(values: &[T], this_type: ConfigOptionType, rhs: &dyn ConfigOption) -> CfgResult<bool> {
        if rhs.r#type() != this_type {
            return Err(ConfigurationError::new("ConfigOptionVector: Comparing incompatible types"));
        }
        Ok(values == vector_ref::<T>(rhs).as_slice())
    }

    /// Config.hpp:740-746 `size_t hash() const throw() override`
    pub fn hash(values: &[T]) -> u64 {
        let mut seed: u64 = 0;
        for v in values {
            boost_hash_combine(&mut seed, v.std_hash());
        }
        seed
    }

    /// Config.hpp:751-769: is this option overridden by another option? An option overrides
    /// another option if it is not nil and not equal. Assume lhs is not nullable even it is
    /// nullable.
    pub fn overriden_by(values: &[T], this_type: ConfigOptionType, rhs: &dyn ConfigOption) -> CfgResult<bool> {
        if rhs.r#type() != this_type {
            return Err(ConfigurationError::new("ConfigOptionVector.overriden_by() applied to different types."));
        }
        let rhs_vec = vector_ref::<T>(rhs);
        if !rhs.nullable() {
            // Overridding a non-nullable object with another non-nullable object.
            return Ok(values != rhs_vec.as_slice());
        }
        let mut i = 0usize;
        let cnt = values.len().min(rhs_vec.len());
        while i < cnt {
            if !is_nil_of(rhs, i) && values[i] != rhs_vec[i] {
                return Ok(true);
            }
            i += 1;
        }
        while i < rhs_vec.len() {
            if !is_nil_of(rhs, i) {
                return Ok(true);
            }
            i += 1;
        }
        Ok(false)
    }

    /// Config.hpp:773-815: apply an override option, possibly a nullable one. Assume lhs is
    /// not nullable even it is nullable. default_index are 0 based.
    pub fn apply_override(values: &mut Vec<T>, this_type: ConfigOptionType, rhs: &dyn ConfigOption, default_index: &mut Vec<i32>) -> CfgResult<bool> {
        if rhs.r#type() != this_type {
            return Err(ConfigurationError::new("ConfigOptionVector.apply_override() applied to different types."));
        }
        let rhs_vec = vector_ref::<T>(rhs);
        if !rhs.nullable() {
            // Overridding a non-nullable object with another non-nullable object.
            if values.as_slice() != rhs_vec.as_slice() {
                *values = Clone::clone(rhs_vec);
                return Ok(true);
            }
            return Ok(false);
        }

        let cnt = values.len().min(rhs_vec.len());
        if cnt < 1 {
            return Ok(false);
        }

        let default_value: Vec<T> = Clone::clone(values);

        if values.is_empty() {
            values.resize(rhs_vec.len(), T::value_initialized());
        } else {
            let front = Clone::clone(&values[0]);
            values.resize(rhs_vec.len(), front);
        }

        let mut modified = false;

        for i in 0..rhs_vec.len() {
            if !is_nil_of(rhs, i) {
                values[i] = Clone::clone(&rhs_vec[i]);
                modified = true;
            } else if i < default_index.len() && (default_index[i] as usize) < default_value.len() {
                values[i] = Clone::clone(&default_value[default_index[i] as usize]);
            } else {
                values[i] = Clone::clone(&default_value[0]);
            }
        }
        Ok(modified)
    }
}

/// `as_any` / `as_any_mut` of a concrete option class.
macro_rules! cfg_option_any {
    () => {
        fn as_any(&self) -> &dyn std::any::Any {
            self
        }
        fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
            self
        }
    };
}

/// The ConfigOptionSingle<T> part of a scalar class: `value` behind `Any`, and `hash()`.
macro_rules! cfg_single_value {
    ($t:ty) => {
        fn single_value_any(&self) -> Option<&dyn std::any::Any> {
            Some(&self.value)
        }
    };
}

/// Config.hpp:315-335: `set`, `operator==` and `hash` of ConfigOptionSingle<T>, for the
/// scalar classes that do not override them.
macro_rules! cfg_single_set_eq_hash {
    ($t:ty) => {
        fn set(&mut self, rhs: &dyn ConfigOption) -> CfgResult<()> {
            let this_type = self.r#type();
            ConfigOptionSingle::<$t>::set(&mut self.value, this_type, rhs)
        }
        fn operator_eq(&self, rhs: &dyn ConfigOption) -> CfgResult<bool> {
            ConfigOptionSingle::<$t>::operator_eq(&self.value, self.r#type(), rhs)
        }
        fn hash(&self) -> u64 {
            ConfigOptionSingle::<$t>::hash(&self.value)
        }
    };
}

/// The ConfigOptionVector<T> part of a vector class inside `impl ConfigOption`: the casts,
/// `hash`, `overriden_by`, `apply_override`.
macro_rules! cfg_vector_common {
    ($t:ty) => {
        fn as_vector_base(&self) -> Option<&dyn ConfigOptionVectorBase> {
            Some(self)
        }
        fn as_vector_base_mut(&mut self) -> Option<&mut dyn ConfigOptionVectorBase> {
            Some(self)
        }
        fn vector_values_any(&self) -> Option<&dyn std::any::Any> {
            Some(&self.values)
        }
        fn vector_values_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
            Some(&mut self.values)
        }
        fn hash(&self) -> u64 {
            ConfigOptionVector::<$t>::hash(&self.values)
        }
        fn overriden_by(&self, rhs: &dyn ConfigOption) -> CfgResult<bool> {
            ConfigOptionVector::<$t>::overriden_by(&self.values, self.r#type(), rhs)
        }
        fn apply_override(&mut self, rhs: &dyn ConfigOption, default_index: &mut Vec<i32>) -> CfgResult<bool> {
            let this_type = self.r#type();
            ConfigOptionVector::<$t>::apply_override(&mut self.values, this_type, rhs, default_index)
        }
    };
}

/// Config.hpp:400-406 `ConfigOptionVector<T>::set(const ConfigOption*)`, for the classes that
/// do not override it.
macro_rules! cfg_vector_set {
    ($t:ty) => {
        fn set(&mut self, rhs: &dyn ConfigOption) -> CfgResult<()> {
            let this_type = self.r#type();
            ConfigOptionVector::<$t>::set(&mut self.values, this_type, rhs)
        }
    };
}

/// Config.hpp:729-735 `ConfigOptionVector<T>::operator==(const ConfigOption&)`, for the
/// classes that do not override it.
macro_rules! cfg_vector_eq {
    ($t:ty) => {
        fn operator_eq(&self, rhs: &dyn ConfigOption) -> CfgResult<bool> {
            ConfigOptionVector::<$t>::operator_eq(&self.values, self.r#type(), rhs)
        }
    };
}

/// The methods of ConfigOptionVectorBase a vector class inherits from ConfigOptionVector<T>
/// (everything but `vserialize`, `is_nil(idx)` and `set_at_to_nil`). `$nil` is the value the
/// class's `set_at_to_nil` stores, as an `Option`.
macro_rules! cfg_vector_base_methods {
    ($t:ty, $nil:expr) => {
        fn set_from_options(&mut self, rhs: &[&dyn ConfigOption]) -> CfgResult<()> {
            let this_type = self.r#type();
            ConfigOptionVector::<$t>::set_from_options(&mut self.values, this_type, rhs)
        }
        fn set_at(&mut self, rhs: &dyn ConfigOption, i: usize, j: usize) -> CfgResult<()> {
            let this_type = self.r#type();
            ConfigOptionVector::<$t>::set_at(&mut self.values, this_type, rhs, i, j)
        }
        fn append(&mut self, rhs: &dyn ConfigOption) -> CfgResult<()> {
            let this_type = self.r#type();
            ConfigOptionVector::<$t>::append(&mut self.values, this_type, rhs)
        }
        fn set_range(&mut self, rhs: &dyn ConfigOption, start: usize, len: usize) -> CfgResult<()> {
            let this_type = self.r#type();
            ConfigOptionVector::<$t>::set_range(&mut self.values, this_type, rhs, start, len)
        }
        fn set_with_restore(&mut self, rhs: &dyn ConfigOption, restore_index: &mut Vec<i32>, stride: i32) -> CfgResult<()> {
            let this_type = self.r#type();
            ConfigOptionVector::<$t>::set_with_restore(&mut self.values, this_type, rhs, restore_index, stride)
        }
        fn set_with_restore_2(
            &mut self,
            key: &str,
            rhs: &mut dyn ConfigOption,
            restore_index: &mut Vec<i32>,
            start: i32,
            len: i32,
            skip_error: bool,
        ) -> CfgResult<()> {
            let this_type = self.r#type();
            ConfigOptionVector::<$t>::set_with_restore_2(&mut self.values, this_type, key, rhs, restore_index, start, len, skip_error)
        }
        fn set_only_diff(&mut self, key: &str, rhs: &dyn ConfigOption, diff_index: &mut Vec<i32>, stride: i32) -> CfgResult<()> {
            let this_type = self.r#type();
            ConfigOptionVector::<$t>::set_only_diff(&mut self.values, this_type, key, rhs, diff_index, stride)
        }
        fn set_to_index(&mut self, rhs: &dyn ConfigOption, dest_index: &mut Vec<i32>, stride: i32) -> CfgResult<()> {
            let this_type = self.r#type();
            ConfigOptionVector::<$t>::set_to_index(&mut self.values, this_type, rhs, dest_index, stride)
        }
        fn set_with_nil(&mut self, rhs: &dyn ConfigOption, inherits: &dyn ConfigOption, stride: i32) -> CfgResult<()> {
            let this_type = self.r#type();
            let nil: Option<$t> = $nil;
            ConfigOptionVector::<$t>::set_with_nil(&mut self.values, this_type, nil, rhs, inherits, stride)
        }
        fn set_with_default(&mut self, inherits: &dyn ConfigOption) -> CfgResult<()> {
            let this_type = self.r#type();
            ConfigOptionVector::<$t>::set_with_default(&mut self.values, this_type, inherits)
        }
        fn resize(&mut self, n: usize, opt_default: Option<&dyn ConfigOption>) -> CfgResult<()> {
            let this_type = self.r#type();
            ConfigOptionVector::<$t>::resize(&mut self.values, this_type, n, opt_default)
        }
        fn clear(&mut self) {
            ConfigOptionVector::<$t>::clear(&mut self.values)
        }
        fn size(&self) -> usize {
            ConfigOptionVector::<$t>::size(&self.values)
        }
        fn empty(&self) -> bool {
            ConfigOptionVector::<$t>::empty(&self.values)
        }
    };
}
// ---------------------------------------------------------------------------
// Config.hpp:822-2230: the option classes.
// ---------------------------------------------------------------------------

/// `std::vector<T>::operator<` (`std::lexicographical_compare` with `<`).
fn lexicographical_less<T>(v1: &[T], v2: &[T], less: impl Fn(&T, &T) -> bool) -> bool {
    let mut i = 0usize;
    while i < v1.len() && i < v2.len() {
        if less(&v1[i], &v2[i]) {
            return true;
        }
        if less(&v2[i], &v1[i]) {
            return false;
        }
        i += 1;
    }
    i == v1.len() && i != v2.len()
}

/// Config.hpp:47 `operator<(const FloatOrPercent&, const FloatOrPercent&)`
fn float_or_percent_less(l: &FloatOrPercent, r: &FloatOrPercent) -> bool {
    l.value < r.value || (l.value == r.value && (l.percent as i32) < (r.percent as i32))
}

/// Config.hpp:822-859 `class ConfigOptionFloat : public ConfigOptionSingle<double>`
#[derive(Debug)]
pub struct ConfigOptionFloat {
    pub value: f64,
}

impl ConfigOptionFloat {
    /// Config.hpp:829 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:825 `ConfigOptionFloat() : ConfigOptionSingle<double>(0) {}`
    pub fn new() -> Self {
        ConfigOptionFloat { value: 0.0 }
    }
    /// Config.hpp:826 `explicit ConfigOptionFloat(double _value)`
    pub fn with_value(value: f64) -> Self {
        ConfigOptionFloat { value }
    }
    /// Config.hpp:828 `static ConfigOptionType static_type()`
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoFloat
    }
    /// Config.hpp:833 `bool operator< (const ConfigOptionFloat &rhs) const throw()`
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        self.value < rhs.value
    }
}

impl ConfigOptionStaticType for ConfigOptionFloat {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoFloat
    }
}

impl ConfigOption for ConfigOptionFloat {
    cfg_option_any!();
    cfg_single_value!(f64);
    cfg_single_set_eq_hash!(f64);
    /// Config.hpp:829
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:830 `double getFloat() const override`
    fn get_float(&self) -> CfgResult<f64> {
        Ok(self.value)
    }
    /// Config.hpp:831
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionFloat { value: self.value })
    }
    /// Config.hpp:835-840: `ss << this->value`
    fn serialize(&self) -> CfgResult<String> {
        Ok(ostream_double(self.value))
    }
    /// Config.hpp:842-848: `iss >> this->value; return !iss.fail();`
    fn deserialize(&mut self, str: &str, _append: bool) -> CfgResult<bool> {
        let r = istream_double(str.as_bytes());
        if let Some(v) = r.value {
            self.value = v;
        }
        Ok(!r.fail)
    }
}

/// Config.hpp:861-987 `template<bool NULLABLE> class ConfigOptionFloatsTempl : public ConfigOptionVector<double>`
#[derive(Debug)]
pub struct ConfigOptionFloatsTempl<const NULLABLE: bool> {
    pub values: Vec<f64>,
}

/// Config.hpp:989
pub type ConfigOptionFloats = ConfigOptionFloatsTempl<false>;
/// Config.hpp:990
pub type ConfigOptionFloatsNullable = ConfigOptionFloatsTempl<true>;

impl<const NULLABLE: bool> ConfigOptionFloatsTempl<NULLABLE> {
    /// Config.hpp:872 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:865
    pub fn new() -> Self {
        Self { values: Vec::new() }
    }
    /// Config.hpp:866 `explicit ConfigOptionFloatsTempl(size_t n, double value)`
    pub fn with_size(n: usize, value: f64) -> Self {
        Self { values: vec![value; n] }
    }
    /// Config.hpp:867-869 (initializer list, vector)
    pub fn from_values(vec: Vec<f64>) -> Self {
        Self { values: vec }
    }
    /// Config.hpp:871
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoFloats
    }
    /// Config.hpp:875 `bool operator< (const ConfigOptionFloatsTempl &rhs) const throw()`
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        Self::vectors_lower(&self.values, &rhs.values)
    }
    /// Config.hpp:885: special "nil" value to be stored into the vector if this->supports_nil().
    pub fn nil_value() -> f64 {
        f64::NAN
    }
    /// Config.hpp:681 `const T& get_at(size_t i) const`
    pub fn get_at(&self, i: usize) -> &f64 {
        ConfigOptionVector::<f64>::get_at(&self.values, i)
    }
    /// Config.hpp:687 `T& get_at(size_t i)`
    pub fn get_at_mut(&mut self, i: usize) -> &mut f64 {
        ConfigOptionVector::<f64>::get_at_mut(&mut self.values, i)
    }

    /// Config.hpp:918-939 `bool deserialize(const std::string &str, bool append = false) override`,
    /// also what ConfigOptionPercentsTempl inherits.
    fn deserialize_values(values: &mut Vec<f64>, str: &str, append: bool) -> CfgResult<bool> {
        if !append {
            values.clear();
        }
        // FIDELITY-NOTE: `double value; iss >> value;` leaves `value` uninitialized when the item
        // is empty (the stream sentry fails on an empty string). The C++ then pushes whatever the
        // stack slot holds; here that slot starts at 0 and keeps the value of the previous item.
        let mut value: f64 = 0.0;
        for item_str in getline_split(str.as_bytes(), b',') {
            let item_str = boost_trim(item_str);
            if item_str == b"nil" {
                if NULLABLE {
                    values.push(Self::nil_value());
                } else {
                    return Err(ConfigurationError::new("Deserializing nil into a non-nullable object"));
                }
            } else {
                if let Some(v) = istream_double(item_str).value {
                    value = v;
                }
                values.push(value);
            }
        }
        Ok(true)
    }

    /// Config.hpp:948-958 `void serialize_single_value(std::ostringstream &ss, const double v) const`
    pub(crate) fn serialize_single_value(ss: &mut String, v: f64) -> CfgResult<()> {
        if v.is_finite() {
            ss.push_str(&ostream_double(v));
        } else if v.is_nan() {
            if NULLABLE {
                ss.push_str("nil");
            } else {
                return Err(ConfigurationError::new("Serializing NaN"));
            }
        } else {
            return Err(ConfigurationError::new("Serializing invalid number"));
        }
        Ok(())
    }

    /// Config.hpp:959-970 `static bool vectors_equal(const std::vector<double> &v1, const std::vector<double> &v2)`
    pub(crate) fn vectors_equal(v1: &[f64], v2: &[f64]) -> bool {
        if NULLABLE {
            if v1.len() != v2.len() {
                return false;
            }
            for (it1, it2) in v1.iter().zip(v2.iter()) {
                if !((it1.is_nan() && it2.is_nan()) || *it1 == *it2) {
                    return false;
                }
            }
            true
        } else {
            // Not supporting nullable values, the default vector compare is cheaper.
            v1 == v2
        }
    }

    /// Config.hpp:971-982 `static bool vectors_lower(const std::vector<double> &v1, const std::vector<double> &v2)`
    pub(crate) fn vectors_lower(v1: &[f64], v2: &[f64]) -> bool {
        if NULLABLE {
            // The loop body returns on the first pair, as in the C++.
            #[allow(clippy::never_loop)]
            for (it1, it2) in v1.iter().zip(v2.iter()) {
                let null1 = it1.is_nan() as i32;
                let null2 = it2.is_nan() as i32;
                return (null1 < null2) || (null1 == null2 && *it1 < *it2);
            }
            v1.len() < v2.len()
        } else {
            // Not supporting nullable values, the default vector compare is cheaper.
            lexicographical_less(v1, v2, |a, b| a < b)
        }
    }
}

impl<const NULLABLE: bool> ConfigOptionStaticType for ConfigOptionFloatsTempl<NULLABLE> {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoFloats
    }
}

impl<const NULLABLE: bool> ConfigOption for ConfigOptionFloatsTempl<NULLABLE> {
    cfg_option_any!();
    cfg_vector_common!(f64);
    cfg_vector_set!(f64);
    /// Config.hpp:872
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:873
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(Self { values: self.values.clone() })
    }
    /// Config.hpp:876-881 `bool operator==(const ConfigOption &rhs) const override`
    fn operator_eq(&self, rhs: &dyn ConfigOption) -> CfgResult<bool> {
        if rhs.r#type() != self.r#type() {
            return Err(ConfigurationError::new("ConfigOptionFloatsTempl: Comparing incompatible types"));
        }
        Ok(Self::vectors_equal(&self.values, vector_ref::<f64>(rhs)))
    }
    /// Config.hpp:883: could a special "nil" value be stored inside the vector, indicating undefined value?
    fn nullable(&self) -> bool {
        NULLABLE
    }
    /// Config.hpp:887: a scalar is nil, or all values of a vector are nil.
    fn is_nil(&self) -> bool {
        for v in &self.values {
            if !v.is_nan() {
                return false;
            }
        }
        true
    }
    /// Config.hpp:895-904
    fn serialize(&self) -> CfgResult<String> {
        let mut ss = String::new();
        for (i, v) in self.values.iter().enumerate() {
            if i != 0 {
                ss.push(',');
            }
            Self::serialize_single_value(&mut ss, *v)?;
        }
        Ok(ss)
    }
    /// Config.hpp:918-939
    fn deserialize(&mut self, str: &str, append: bool) -> CfgResult<bool> {
        Self::deserialize_values(&mut self.values, str, append)
    }
}

impl<const NULLABLE: bool> ConfigOptionVectorBase for ConfigOptionFloatsTempl<NULLABLE> {
    cfg_vector_base_methods!(f64, Some(Self::nil_value()));
    /// Config.hpp:906-916
    fn vserialize(&self) -> CfgResult<Vec<String>> {
        let mut vv = Vec::with_capacity(self.values.len());
        for v in &self.values {
            let mut ss = String::new();
            Self::serialize_single_value(&mut ss, *v)?;
            vv.push(ss);
        }
        Ok(vv)
    }
    /// Config.hpp:888 `bool is_nil(size_t idx) const override`
    fn is_nil_at(&self, idx: usize) -> bool {
        self.values[idx].is_nan()
    }
    /// Config.hpp:889-893
    fn set_at_to_nil(&mut self, i: usize) {
        // assert(self.nullable() && i < self.values.len()): an `assert` of the C++, compiled out in the reference build (-DNDEBUG).
        self.values[i] = Self::nil_value();
    }
}

/// Config.hpp:992-1030 `class ConfigOptionInt : public ConfigOptionSingle<int>`
#[derive(Debug)]
pub struct ConfigOptionInt {
    pub value: i32,
}

impl ConfigOptionInt {
    /// Config.hpp:1000 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:995
    pub fn new() -> Self {
        ConfigOptionInt { value: 0 }
    }
    /// Config.hpp:996 `explicit ConfigOptionInt(int value)`
    pub fn with_value(value: i32) -> Self {
        ConfigOptionInt { value }
    }
    /// Config.hpp:997 `explicit ConfigOptionInt(double _value) : ConfigOptionSingle<int>(int(floor(_value + 0.5)))`
    pub fn with_double(value: f64) -> Self {
        ConfigOptionInt { value: (value + 0.5).floor() as i32 }
    }
    /// Config.hpp:999
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoInt
    }
}

impl ConfigOptionStaticType for ConfigOptionInt {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoInt
    }
}

impl ConfigOption for ConfigOptionInt {
    cfg_option_any!();
    cfg_single_value!(i32);
    cfg_single_set_eq_hash!(i32);
    /// Config.hpp:1000
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:1001 `int getInt() const override`
    fn get_int(&self) -> CfgResult<i32> {
        Ok(self.value)
    }
    /// Config.hpp:1002 `void setInt(int val) override`
    fn set_int(&mut self, val: i32) -> CfgResult<()> {
        self.value = val;
        Ok(())
    }
    /// Config.hpp:1003
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionInt { value: self.value })
    }
    /// Config.hpp:1006-1011
    fn serialize(&self) -> CfgResult<String> {
        Ok(ostream_int(self.value))
    }
    /// Config.hpp:1013-1019
    fn deserialize(&mut self, str: &str, _append: bool) -> CfgResult<bool> {
        let r = istream_int(str.as_bytes());
        if let Some(v) = r.value {
            self.value = v;
        }
        Ok(!r.fail)
    }
}

/// Config.hpp:1032-1120 `template<bool NULLABLE> class ConfigOptionIntsTempl : public ConfigOptionVector<int>`
#[derive(Debug)]
pub struct ConfigOptionIntsTempl<const NULLABLE: bool> {
    pub values: Vec<i32>,
}

/// Config.hpp:1122
pub type ConfigOptionInts = ConfigOptionIntsTempl<false>;
/// Config.hpp:1123
pub type ConfigOptionIntsNullable = ConfigOptionIntsTempl<true>;

impl<const NULLABLE: bool> ConfigOptionIntsTempl<NULLABLE> {
    /// Config.hpp:1043 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:1036
    pub fn new() -> Self {
        Self { values: Vec::new() }
    }
    /// Config.hpp:1037 `explicit ConfigOptionIntsTempl(size_t n, int value)`
    pub fn with_size(n: usize, value: i32) -> Self {
        Self { values: vec![value; n] }
    }
    /// Config.hpp:1038-1040
    pub fn from_values(vec: Vec<i32>) -> Self {
        Self { values: vec }
    }
    /// Config.hpp:1042
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoInts
    }
    /// Config.hpp:1047 `bool operator< (const ConfigOptionIntsTempl &rhs) const throw()`
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        self.values < rhs.values
    }
    /// Config.hpp:1051: special "nil" value to be stored into the vector if this->supports_nil().
    pub fn nil_value() -> i32 {
        i32::MAX
    }
    /// Config.hpp:681
    pub fn get_at(&self, i: usize) -> &i32 {
        ConfigOptionVector::<i32>::get_at(&self.values, i)
    }
    /// Config.hpp:687
    pub fn get_at_mut(&mut self, i: usize) -> &mut i32 {
        ConfigOptionVector::<i32>::get_at_mut(&mut self.values, i)
    }
    /// Config.hpp:1108-1116 `void serialize_single_value(std::ostringstream &ss, const int v) const`
    fn serialize_single_value(ss: &mut String, v: i32) -> CfgResult<()> {
        if v == Self::nil_value() {
            if NULLABLE {
                ss.push_str("nil");
            } else {
                return Err(ConfigurationError::new("Serializing NaN"));
            }
        } else {
            ss.push_str(&ostream_int(v));
        }
        Ok(())
    }
}

impl<const NULLABLE: bool> ConfigOptionStaticType for ConfigOptionIntsTempl<NULLABLE> {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoInts
    }
}

impl<const NULLABLE: bool> ConfigOption for ConfigOptionIntsTempl<NULLABLE> {
    cfg_option_any!();
    cfg_vector_common!(i32);
    cfg_vector_set!(i32);
    cfg_vector_eq!(i32);
    /// Config.hpp:1043
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:1044
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(Self { values: self.values.clone() })
    }
    /// Config.hpp:1049
    fn nullable(&self) -> bool {
        NULLABLE
    }
    /// Config.hpp:1053
    fn is_nil(&self) -> bool {
        for v in &self.values {
            if *v != Self::nil_value() {
                return false;
            }
        }
        true
    }
    /// Config.hpp:1061-1070
    fn serialize(&self) -> CfgResult<String> {
        let mut ss = String::new();
        for (i, v) in self.values.iter().enumerate() {
            if i != 0 {
                ss.push(',');
            }
            Self::serialize_single_value(&mut ss, *v)?;
        }
        Ok(ss)
    }
    /// Config.hpp:1084-1105
    fn deserialize(&mut self, str: &str, append: bool) -> CfgResult<bool> {
        if !append {
            self.values.clear();
        }
        // FIDELITY-NOTE: `int value; iss >> value;` is uninitialized for an empty item, see
        // ConfigOptionFloatsTempl::deserialize.
        let mut value: i32 = 0;
        for item_str in getline_split(str.as_bytes(), b',') {
            let item_str = boost_trim(item_str);
            if item_str == b"nil" {
                if NULLABLE {
                    self.values.push(Self::nil_value());
                } else {
                    return Err(ConfigurationError::new("Deserializing nil into a non-nullable object"));
                }
            } else {
                if let Some(v) = istream_int(item_str).value {
                    value = v;
                }
                self.values.push(value);
            }
        }
        Ok(true)
    }
}

impl<const NULLABLE: bool> ConfigOptionVectorBase for ConfigOptionIntsTempl<NULLABLE> {
    cfg_vector_base_methods!(i32, Some(Self::nil_value()));
    /// Config.hpp:1072-1082
    fn vserialize(&self) -> CfgResult<Vec<String>> {
        let mut vv = Vec::with_capacity(self.values.len());
        for v in &self.values {
            let mut ss = String::new();
            Self::serialize_single_value(&mut ss, *v)?;
            vv.push(ss);
        }
        Ok(vv)
    }
    /// Config.hpp:1054
    fn is_nil_at(&self, idx: usize) -> bool {
        self.values[idx] == Self::nil_value()
    }
    /// Config.hpp:1055-1059
    fn set_at_to_nil(&mut self, i: usize) {
        // assert(self.nullable() && i < self.values.len()): an `assert` of the C++, compiled out in the reference build (-DNDEBUG).
        self.values[i] = Self::nil_value();
    }
}

/// Config.hpp:1125-1153 `class ConfigOptionString : public ConfigOptionSingle<std::string>`
#[derive(Debug)]
pub struct ConfigOptionString {
    pub value: String,
}

impl ConfigOptionString {
    /// Config.hpp:1132 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:1128
    pub fn new() -> Self {
        ConfigOptionString { value: String::new() }
    }
    /// Config.hpp:1129 `explicit ConfigOptionString(const std::string &value)`
    pub fn with_value(value: impl Into<String>) -> Self {
        ConfigOptionString { value: value.into() }
    }
    /// Config.hpp:1131
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoString
    }
    /// Config.hpp:1136 `bool operator< (const ConfigOptionString &rhs) const throw()`
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        self.value < rhs.value
    }
    /// Config.hpp:1137 `bool empty() const`
    pub fn empty(&self) -> bool {
        self.value.is_empty()
    }
}

impl ConfigOptionStaticType for ConfigOptionString {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoString
    }
}

impl ConfigOption for ConfigOptionString {
    cfg_option_any!();
    cfg_single_value!(String);
    cfg_single_set_eq_hash!(String);
    /// Config.hpp:1132
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:1133
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionString { value: self.value.clone() })
    }
    /// Config.hpp:1139-1142
    fn serialize(&self) -> CfgResult<String> {
        Ok(escape_string_cstyle(&self.value))
    }
    /// Config.hpp:1144-1148
    fn deserialize(&mut self, str: &str, _append: bool) -> CfgResult<bool> {
        Ok(unescape_string_cstyle(str, &mut self.value))
    }
}

/// Config.hpp:1155-1201 `class ConfigOptionStrings : public ConfigOptionVector<std::string>`:
/// semicolon-separated strings.
#[derive(Debug)]
pub struct ConfigOptionStrings {
    pub values: Vec<String>,
}

impl ConfigOptionStrings {
    /// Config.hpp:1166 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:1159
    pub fn new() -> Self {
        ConfigOptionStrings { values: Vec::new() }
    }
    /// Config.hpp:1160 `explicit ConfigOptionStrings(size_t n, const std::string &value)`
    pub fn with_size(n: usize, value: &str) -> Self {
        ConfigOptionStrings { values: vec![value.to_string(); n] }
    }
    /// Config.hpp:1161-1163
    pub fn from_values(values: Vec<String>) -> Self {
        ConfigOptionStrings { values }
    }
    /// Config.hpp:1165
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoStrings
    }
    /// Config.hpp:1170 `bool operator< (const ConfigOptionStrings &rhs) const throw()`
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        self.values < rhs.values
    }
    /// Config.hpp:681
    pub fn get_at(&self, i: usize) -> &String {
        ConfigOptionVector::<String>::get_at(&self.values, i)
    }
    /// Config.hpp:687
    pub fn get_at_mut(&mut self, i: usize) -> &mut String {
        ConfigOptionVector::<String>::get_at_mut(&mut self.values, i)
    }
}

impl ConfigOptionStaticType for ConfigOptionStrings {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoStrings
    }
}

impl ConfigOption for ConfigOptionStrings {
    cfg_option_any!();
    cfg_vector_common!(String);
    cfg_vector_set!(String);
    cfg_vector_eq!(String);
    /// Config.hpp:1166
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:1167
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionStrings { values: self.values.clone() })
    }
    /// Config.hpp:1173-1176
    fn serialize(&self) -> CfgResult<String> {
        Ok(escape_strings_cstyle(&self.values))
    }
    /// Config.hpp:1191-1196
    fn deserialize(&mut self, str: &str, append: bool) -> CfgResult<bool> {
        if !append {
            self.values.clear();
        }
        Ok(unescape_strings_cstyle(str, &mut self.values))
    }
}

impl ConfigOptionVectorBase for ConfigOptionStrings {
    cfg_vector_base_methods!(String, None);
    /// Config.hpp:1178-1189
    fn vserialize(&self) -> CfgResult<Vec<String>> {
        Ok(self.values.clone())
    }
    /// Config.hpp:1171
    fn is_nil_at(&self, _idx: usize) -> bool {
        false
    }
    /// Config.hpp:453 `virtual void set_at_to_nil(size_t i) override {}`
    fn set_at_to_nil(&mut self, _i: usize) {}
}

/// Config.hpp:1203-1239 `class ConfigOptionPercent : public ConfigOptionFloat`
#[derive(Debug)]
pub struct ConfigOptionPercent {
    pub value: f64,
}

impl ConfigOptionPercent {
    /// Config.hpp:1210 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:1206
    pub fn new() -> Self {
        ConfigOptionPercent { value: 0.0 }
    }
    /// Config.hpp:1207 `explicit ConfigOptionPercent(double _value)`
    pub fn with_value(value: f64) -> Self {
        ConfigOptionPercent { value }
    }
    /// Config.hpp:1209
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoPercent
    }
    /// Config.hpp:1214 `bool operator< (const ConfigOptionPercent &rhs) const throw()`
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        self.value < rhs.value
    }
    /// Config.hpp:1216 `double get_abs_value(double ratio_over) const`
    pub fn get_abs_value(&self, ratio_over: f64) -> f64 {
        ratio_over * self.value / 100.0
    }
}

impl ConfigOptionStaticType for ConfigOptionPercent {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoPercent
    }
}

impl ConfigOption for ConfigOptionPercent {
    cfg_option_any!();
    cfg_single_value!(f64);
    cfg_single_set_eq_hash!(f64);
    /// Config.hpp:1210
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:830 (ConfigOptionFloat::getFloat)
    fn get_float(&self) -> CfgResult<f64> {
        Ok(self.value)
    }
    /// Config.hpp:1211
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionPercent { value: self.value })
    }
    /// Config.hpp:1218-1225
    fn serialize(&self) -> CfgResult<String> {
        let mut s = ostream_double(self.value);
        s.push('%');
        Ok(s)
    }
    /// Config.hpp:1227-1234: don't try to parse the trailing % since it's optional
    fn deserialize(&mut self, str: &str, _append: bool) -> CfgResult<bool> {
        let r = istream_double(str.as_bytes());
        if let Some(v) = r.value {
            self.value = v;
        }
        Ok(!r.fail)
    }
}

/// Config.hpp:1241-1292 `template<bool NULLABLE> class ConfigOptionPercentsTempl : public ConfigOptionFloatsTempl<NULLABLE>`
#[derive(Debug)]
pub struct ConfigOptionPercentsTempl<const NULLABLE: bool> {
    pub values: Vec<f64>,
}

/// Config.hpp:1294
pub type ConfigOptionPercents = ConfigOptionPercentsTempl<false>;
/// Config.hpp:1295
pub type ConfigOptionPercentsNullable = ConfigOptionPercentsTempl<true>;

impl<const NULLABLE: bool> ConfigOptionPercentsTempl<NULLABLE> {
    /// Config.hpp:1252 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:1245
    pub fn new() -> Self {
        Self { values: Vec::new() }
    }
    /// Config.hpp:1246 `explicit ConfigOptionPercentsTempl(size_t n, double value)`
    pub fn with_size(n: usize, value: f64) -> Self {
        Self { values: vec![value; n] }
    }
    /// Config.hpp:1247-1249
    pub fn from_values(vec: Vec<f64>) -> Self {
        Self { values: vec }
    }
    /// Config.hpp:1251
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoPercents
    }
    /// Config.hpp:1256 `bool operator< (const ConfigOptionPercentsTempl &rhs) const throw()`
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        ConfigOptionFloatsTempl::<NULLABLE>::vectors_lower(&self.values, &rhs.values)
    }
    /// Config.hpp:885 (inherited)
    pub fn nil_value() -> f64 {
        f64::NAN
    }
    /// Config.hpp:681
    pub fn get_at(&self, i: usize) -> &f64 {
        ConfigOptionVector::<f64>::get_at(&self.values, i)
    }
    /// Config.hpp:687
    pub fn get_at_mut(&mut self, i: usize) -> &mut f64 {
        ConfigOptionVector::<f64>::get_at_mut(&mut self.values, i)
    }
}

impl<const NULLABLE: bool> ConfigOptionStaticType for ConfigOptionPercentsTempl<NULLABLE> {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoPercents
    }
}

impl<const NULLABLE: bool> ConfigOption for ConfigOptionPercentsTempl<NULLABLE> {
    cfg_option_any!();
    cfg_vector_common!(f64);
    cfg_vector_set!(f64);
    /// Config.hpp:1252
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:1253
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(Self { values: self.values.clone() })
    }
    /// Config.hpp:876-881 (ConfigOptionFloatsTempl::operator==, inherited)
    fn operator_eq(&self, rhs: &dyn ConfigOption) -> CfgResult<bool> {
        if rhs.r#type() != self.r#type() {
            return Err(ConfigurationError::new("ConfigOptionFloatsTempl: Comparing incompatible types"));
        }
        Ok(ConfigOptionFloatsTempl::<NULLABLE>::vectors_equal(&self.values, vector_ref::<f64>(rhs)))
    }
    /// Config.hpp:883 (inherited)
    fn nullable(&self) -> bool {
        NULLABLE
    }
    /// Config.hpp:887 (inherited)
    fn is_nil(&self) -> bool {
        for v in &self.values {
            if !v.is_nan() {
                return false;
            }
        }
        true
    }
    /// Config.hpp:1258-1270
    fn serialize(&self) -> CfgResult<String> {
        let mut ss = String::new();
        for (i, v) in self.values.iter().enumerate() {
            if i != 0 {
                ss.push(',');
            }
            ConfigOptionFloatsTempl::<NULLABLE>::serialize_single_value(&mut ss, *v)?;
            if !v.is_nan() {
                ss.push('%');
            }
        }
        Ok(ss)
    }
    /// Config.hpp:1286-1287: the float's deserialize function shall ignore the trailing optional %.
    fn deserialize(&mut self, str: &str, append: bool) -> CfgResult<bool> {
        ConfigOptionFloatsTempl::<NULLABLE>::deserialize_values(&mut self.values, str, append)
    }
}

impl<const NULLABLE: bool> ConfigOptionVectorBase for ConfigOptionPercentsTempl<NULLABLE> {
    cfg_vector_base_methods!(f64, Some(Self::nil_value()));
    /// Config.hpp:1272-1284
    fn vserialize(&self) -> CfgResult<Vec<String>> {
        let mut vv = Vec::with_capacity(self.values.len());
        for v in &self.values {
            let mut ss = String::new();
            ConfigOptionFloatsTempl::<NULLABLE>::serialize_single_value(&mut ss, *v)?;
            if !v.is_nan() {
                ss.push('%');
            }
            vv.push(ss);
        }
        Ok(vv)
    }
    /// Config.hpp:888 (inherited)
    fn is_nil_at(&self, idx: usize) -> bool {
        self.values[idx].is_nan()
    }
    /// Config.hpp:889-893 (inherited)
    fn set_at_to_nil(&mut self, i: usize) {
        // assert(self.nullable() && i < self.values.len()): an `assert` of the C++, compiled out in the reference build (-DNDEBUG).
        self.values[i] = Self::nil_value();
    }
}

/// Config.hpp:1297-1353 `class ConfigOptionFloatOrPercent : public ConfigOptionPercent`
#[derive(Debug)]
pub struct ConfigOptionFloatOrPercent {
    pub value: f64,
    pub percent: bool,
}

impl ConfigOptionFloatOrPercent {
    /// Config.hpp:1305 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:1301
    pub fn new() -> Self {
        ConfigOptionFloatOrPercent { value: 0.0, percent: false }
    }
    /// Config.hpp:1302 `explicit ConfigOptionFloatOrPercent(double _value, bool _percent)`
    pub fn with_value(value: f64, percent: bool) -> Self {
        ConfigOptionFloatOrPercent { value, percent }
    }
    /// Config.hpp:1304
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoFloatOrPercent
    }
    /// Config.hpp:1319-1320 `bool operator< (const ConfigOptionFloatOrPercent &rhs) const throw()`
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        self.value < rhs.value || (self.value == rhs.value && (self.percent as i32) < (rhs.percent as i32))
    }
    /// Config.hpp:1322-1323 `double get_abs_value(double ratio_over) const`
    pub fn get_abs_value(&self, ratio_over: f64) -> f64 {
        if self.percent {
            ratio_over * self.value / 100.0
        } else {
            self.value
        }
    }
}

impl ConfigOptionStaticType for ConfigOptionFloatOrPercent {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoFloatOrPercent
    }
}

impl ConfigOption for ConfigOptionFloatOrPercent {
    cfg_option_any!();
    cfg_single_value!(f64);
    /// Config.hpp:1305
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:830 (ConfigOptionFloat::getFloat)
    fn get_float(&self) -> CfgResult<f64> {
        Ok(self.value)
    }
    /// Config.hpp:1306
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionFloatOrPercent { value: self.value, percent: self.percent })
    }
    /// Config.hpp:1308-1316 `bool operator==(const ConfigOption &rhs) const override`
    fn operator_eq(&self, rhs: &dyn ConfigOption) -> CfgResult<bool> {
        if rhs.r#type() != self.r#type() {
            return Err(ConfigurationError::new("ConfigOptionFloatOrPercent: Comparing incompatible types"));
        }
        let rhs = rhs
            .as_any()
            .downcast_ref::<ConfigOptionFloatOrPercent>()
            .expect("static_cast<const ConfigOptionFloatOrPercent*>");
        Ok(self.value == rhs.value && self.percent == rhs.percent)
    }
    /// Config.hpp:1317-1318
    fn hash(&self) -> u64 {
        let seed = std_hash_double(self.value);
        if self.percent {
            seed ^ 0x9e3779b9
        } else {
            seed
        }
    }
    /// Config.hpp:1325-1330
    fn set(&mut self, rhs: &dyn ConfigOption) -> CfgResult<()> {
        if rhs.r#type() != self.r#type() {
            return Err(ConfigurationError::new("ConfigOptionFloatOrPercent: Assigning an incompatible type"));
        }
        let rhs = rhs
            .as_any()
            .downcast_ref::<ConfigOptionFloatOrPercent>()
            .expect("static_cast<const ConfigOptionFloatOrPercent*>");
        self.value = rhs.value;
        self.percent = rhs.percent;
        Ok(())
    }
    /// Config.hpp:1332-1339
    fn serialize(&self) -> CfgResult<String> {
        let mut s = ostream_double(self.value);
        if self.percent {
            s.push('%');
        }
        Ok(s)
    }
    /// Config.hpp:1341-1348
    fn deserialize(&mut self, str: &str, _append: bool) -> CfgResult<bool> {
        self.percent = str.as_bytes().contains(&b'%');
        let r = istream_double(str.as_bytes());
        if let Some(v) = r.value {
            self.value = v;
        }
        Ok(!r.fail)
    }
}

/// Config.hpp:1355-1485 `template<bool NULLABLE> class ConfigOptionFloatsOrPercentsTempl : public ConfigOptionVector<FloatOrPercent>`
#[derive(Debug)]
pub struct ConfigOptionFloatsOrPercentsTempl<const NULLABLE: bool> {
    pub values: Vec<FloatOrPercent>,
}

/// Config.hpp:1487
pub type ConfigOptionFloatsOrPercents = ConfigOptionFloatsOrPercentsTempl<false>;
/// Config.hpp:1488
pub type ConfigOptionFloatsOrPercentsNullable = ConfigOptionFloatsOrPercentsTempl<true>;

impl<const NULLABLE: bool> ConfigOptionFloatsOrPercentsTempl<NULLABLE> {
    /// Config.hpp:1366 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:1359
    pub fn new() -> Self {
        Self { values: Vec::new() }
    }
    /// Config.hpp:1360 `explicit ConfigOptionFloatsOrPercentsTempl(size_t n, FloatOrPercent value)`
    pub fn with_size(n: usize, value: FloatOrPercent) -> Self {
        Self { values: vec![value; n] }
    }
    /// Config.hpp:1361-1363
    pub fn from_values(vec: Vec<FloatOrPercent>) -> Self {
        Self { values: vec }
    }
    /// Config.hpp:1365
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoFloatsOrPercents
    }
    /// Config.hpp:1375 `bool operator< (const ConfigOptionFloatsOrPercentsTempl &rhs) const throw()`
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        Self::vectors_lower(&self.values, &rhs.values)
    }
    /// Config.hpp:1380
    pub fn nil_value() -> FloatOrPercent {
        FloatOrPercent { value: f64::NAN, percent: false }
    }
    /// Config.hpp:681
    pub fn get_at(&self, i: usize) -> &FloatOrPercent {
        ConfigOptionVector::<FloatOrPercent>::get_at(&self.values, i)
    }
    /// Config.hpp:687
    pub fn get_at_mut(&mut self, i: usize) -> &mut FloatOrPercent {
        ConfigOptionVector::<FloatOrPercent>::get_at_mut(&mut self.values, i)
    }
    /// Config.hpp:1444-1456 `void serialize_single_value(std::ostringstream &ss, const FloatOrPercent &v) const`
    fn serialize_single_value(ss: &mut String, v: &FloatOrPercent) -> CfgResult<()> {
        if v.value.is_finite() {
            ss.push_str(&ostream_double(v.value));
            if v.percent {
                ss.push('%');
            }
        } else if v.value.is_nan() {
            if NULLABLE {
                ss.push_str("nil");
            } else {
                return Err(ConfigurationError::new("Serializing NaN"));
            }
        } else {
            return Err(ConfigurationError::new("Serializing invalid number"));
        }
        Ok(())
    }
    /// Config.hpp:1457-1468 `static bool vectors_equal(...)`
    fn vectors_equal(v1: &[FloatOrPercent], v2: &[FloatOrPercent]) -> bool {
        if NULLABLE {
            if v1.len() != v2.len() {
                return false;
            }
            for (it1, it2) in v1.iter().zip(v2.iter()) {
                if !((it1.value.is_nan() && it2.value.is_nan()) || *it1 == *it2) {
                    return false;
                }
            }
            true
        } else {
            // Not supporting nullable values, the default vector compare is cheaper.
            v1 == v2
        }
    }
    /// Config.hpp:1469-1480 `static bool vectors_lower(...)`
    fn vectors_lower(v1: &[FloatOrPercent], v2: &[FloatOrPercent]) -> bool {
        if NULLABLE {
            // The loop body returns on the first pair, as in the C++.
            #[allow(clippy::never_loop)]
            for (it1, it2) in v1.iter().zip(v2.iter()) {
                let null1 = it1.value.is_nan() as i32;
                let null2 = it2.value.is_nan() as i32;
                return (null1 < null2) || (null1 == null2 && float_or_percent_less(it1, it2));
            }
            v1.len() < v2.len()
        } else {
            // Not supporting nullable values, the default vector compare is cheaper.
            lexicographical_less(v1, v2, float_or_percent_less)
        }
    }
}

impl<const NULLABLE: bool> ConfigOptionStaticType for ConfigOptionFloatsOrPercentsTempl<NULLABLE> {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoFloatsOrPercents
    }
}

impl<const NULLABLE: bool> ConfigOption for ConfigOptionFloatsOrPercentsTempl<NULLABLE> {
    cfg_option_any!();
    cfg_vector_common!(FloatOrPercent);
    cfg_vector_set!(FloatOrPercent);
    /// Config.hpp:1366
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:1367
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(Self { values: self.values.clone() })
    }
    /// Config.hpp:1369-1374 `bool operator==(const ConfigOption &rhs) const override`
    fn operator_eq(&self, rhs: &dyn ConfigOption) -> CfgResult<bool> {
        if rhs.r#type() != self.r#type() {
            return Err(ConfigurationError::new("ConfigOptionFloatsOrPercentsTempl: Comparing incompatible types"));
        }
        Ok(Self::vectors_equal(&self.values, vector_ref::<FloatOrPercent>(rhs)))
    }
    /// Config.hpp:1378
    fn nullable(&self) -> bool {
        NULLABLE
    }
    /// Config.hpp:1382
    fn is_nil(&self) -> bool {
        for v in &self.values {
            if !v.value.is_nan() {
                return false;
            }
        }
        true
    }
    /// Config.hpp:1390-1399
    fn serialize(&self) -> CfgResult<String> {
        let mut ss = String::new();
        for (i, v) in self.values.iter().enumerate() {
            if i != 0 {
                ss.push(',');
            }
            Self::serialize_single_value(&mut ss, v)?;
        }
        Ok(ss)
    }
    /// Config.hpp:1413-1435
    fn deserialize(&mut self, str: &str, append: bool) -> CfgResult<bool> {
        if !append {
            self.values.clear();
        }
        // FIDELITY-NOTE: `double value; iss >> value;` is uninitialized for an empty item, see
        // ConfigOptionFloatsTempl::deserialize.
        let mut value: f64 = 0.0;
        for item_str in getline_split(str.as_bytes(), b',') {
            let item_str = boost_trim(item_str);
            if item_str == b"nil" {
                if NULLABLE {
                    self.values.push(Self::nil_value());
                } else {
                    return Err(ConfigurationError::new("Deserializing nil into a non-nullable object"));
                }
            } else {
                let percent = item_str.contains(&b'%');
                if let Some(v) = istream_double(item_str).value {
                    value = v;
                }
                self.values.push(FloatOrPercent { value, percent });
            }
        }
        Ok(true)
    }
}

impl<const NULLABLE: bool> ConfigOptionVectorBase for ConfigOptionFloatsOrPercentsTempl<NULLABLE> {
    cfg_vector_base_methods!(FloatOrPercent, Some(Self::nil_value()));
    /// Config.hpp:1401-1411
    fn vserialize(&self) -> CfgResult<Vec<String>> {
        let mut vv = Vec::with_capacity(self.values.len());
        for v in &self.values {
            let mut ss = String::new();
            Self::serialize_single_value(&mut ss, v)?;
            vv.push(ss);
        }
        Ok(vv)
    }
    /// Config.hpp:1383
    fn is_nil_at(&self, idx: usize) -> bool {
        self.values[idx].value.is_nan()
    }
    /// Config.hpp:1384-1388
    fn set_at_to_nil(&mut self, i: usize) {
        // assert(self.nullable() && i < self.values.len()): an `assert` of the C++, compiled out in the reference build (-DNDEBUG).
        self.values[i] = Self::nil_value();
    }
}

/// `ss << (*it)(0); ss << "x"; ss << (*it)(1);`
fn serialize_point_x(ss: &mut String, p: &Vec2d) {
    ss.push_str(&ostream_double(p.x));
    ss.push('x');
    ss.push_str(&ostream_double(p.y));
}

/// Config.hpp:1576-1585 / 1699-1708: one `XxY` item of ConfigOptionPoints / ConfigOptionPointsGroups.
fn deserialize_point_x(point_str: &[u8]) -> Vec2d {
    let mut point = Vec2d::new(0.0, 0.0);
    let coords = getline_split(point_str, b'x');
    if let Some(coord_str) = coords.first() {
        if let Some(v) = istream_double(coord_str).value {
            point.x = v;
        }
        if let Some(coord_str) = coords.get(1) {
            if let Some(v) = istream_double(coord_str).value {
                point.y = v;
            }
        }
    }
    point
}

/// Config.hpp:1490-1523 `class ConfigOptionPoint : public ConfigOptionSingle<Vec2d>`
#[derive(Debug)]
pub struct ConfigOptionPoint {
    pub value: Vec2d,
}

impl ConfigOptionPoint {
    /// Config.hpp:1497 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:1493
    pub fn new() -> Self {
        ConfigOptionPoint { value: Vec2d::new(0.0, 0.0) }
    }
    /// Config.hpp:1494 `explicit ConfigOptionPoint(const Vec2d &value)`
    pub fn with_value(value: Vec2d) -> Self {
        ConfigOptionPoint { value }
    }
    /// Config.hpp:1496
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoPoint
    }
    /// Config.hpp:1501 `bool operator< (const ConfigOptionPoint &rhs) const throw()`: the
    /// `operator<(const Vec2d&, const Vec2d&)` of Point.hpp:80.
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        self.value.x < rhs.value.x || (self.value.x == rhs.value.x && self.value.y < rhs.value.y)
    }
}

impl ConfigOptionStaticType for ConfigOptionPoint {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoPoint
    }
}

impl ConfigOption for ConfigOptionPoint {
    cfg_option_any!();
    cfg_single_value!(Vec2d);
    cfg_single_set_eq_hash!(Vec2d);
    /// Config.hpp:1497
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:1498
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionPoint { value: self.value })
    }
    /// Config.hpp:1503-1510
    fn serialize(&self) -> CfgResult<String> {
        let mut ss = ostream_double(self.value.x);
        ss.push(',');
        ss.push_str(&ostream_double(self.value.y));
        Ok(ss)
    }
    /// Config.hpp:1512-1518: `sscanf(" %lf , %lf %c") == 2 || sscanf(" %lf x %lf %c") == 2`
    fn deserialize(&mut self, str: &str, _append: bool) -> CfgResult<bool> {
        let mut v = [self.value.x, self.value.y];
        let ok = sscanf_doubles(str.as_bytes(), b',', &mut v) == 2 || sscanf_doubles(str.as_bytes(), b'x', &mut v) == 2;
        self.value.x = v[0];
        self.value.y = v[1];
        Ok(ok)
    }
}

/// Config.hpp:1525-1603 `class ConfigOptionPoints : public ConfigOptionVector<Vec2d>`
#[derive(Debug)]
pub struct ConfigOptionPoints {
    pub values: Vec<Vec2d>,
}

impl ConfigOptionPoints {
    /// Config.hpp:1534 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:1528
    pub fn new() -> Self {
        ConfigOptionPoints { values: Vec::new() }
    }
    /// Config.hpp:1529 `explicit ConfigOptionPoints(size_t n, const Vec2d &value)`
    pub fn with_size(n: usize, value: Vec2d) -> Self {
        ConfigOptionPoints { values: vec![value; n] }
    }
    /// Config.hpp:1530-1531
    pub fn from_values(values: Vec<Vec2d>) -> Self {
        ConfigOptionPoints { values }
    }
    /// Config.hpp:1533
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoPoints
    }
    /// Config.hpp:1538-1539 `bool operator< (const ConfigOptionPoints &rhs) const throw()`:
    /// lexicographic with `Vec2d < Vec2d` (Point.hpp: x first, then y).
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        lexicographical_less(&self.values, &rhs.values, |l, r| l.x < r.x || (l.x == r.x && l.y < r.y))
    }
    /// Config.hpp:681
    pub fn get_at(&self, i: usize) -> &Vec2d {
        ConfigOptionVector::<Vec2d>::get_at(&self.values, i)
    }
    /// Config.hpp:687
    pub fn get_at_mut(&mut self, i: usize) -> &mut Vec2d {
        ConfigOptionVector::<Vec2d>::get_at_mut(&mut self.values, i)
    }
}

impl ConfigOptionStaticType for ConfigOptionPoints {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoPoints
    }
}

impl ConfigOption for ConfigOptionPoints {
    cfg_option_any!();
    cfg_vector_common!(Vec2d);
    cfg_vector_set!(Vec2d);
    cfg_vector_eq!(Vec2d);
    /// Config.hpp:1534
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:1535
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionPoints { values: self.values.clone() })
    }
    /// Config.hpp:1542-1552
    fn serialize(&self) -> CfgResult<String> {
        let mut ss = String::new();
        for (i, it) in self.values.iter().enumerate() {
            if i != 0 {
                ss.push(',');
            }
            serialize_point_x(&mut ss, it);
        }
        Ok(ss)
    }
    /// Config.hpp:1569-1588
    fn deserialize(&mut self, str: &str, append: bool) -> CfgResult<bool> {
        if !append {
            self.values.clear();
        }
        for point_str in getline_split(str.as_bytes(), b',') {
            self.values.push(deserialize_point_x(point_str));
        }
        Ok(true)
    }
}

impl ConfigOptionVectorBase for ConfigOptionPoints {
    cfg_vector_base_methods!(Vec2d, None);
    /// Config.hpp:1554-1567
    fn vserialize(&self) -> CfgResult<Vec<String>> {
        let mut vv = Vec::new();
        for it in &self.values {
            let mut ss = String::new();
            serialize_point_x(&mut ss, it);
            vv.push(ss);
        }
        Ok(vv)
    }
    /// Config.hpp:1540
    fn is_nil_at(&self, _idx: usize) -> bool {
        false
    }
    /// Config.hpp:453
    fn set_at_to_nil(&mut self, _i: usize) {}
}

/// Config.hpp:1605-1641 `class ConfigOptionPoint3 : public ConfigOptionSingle<Vec3d>`
#[derive(Debug)]
pub struct ConfigOptionPoint3 {
    pub value: Vec3d,
}

impl ConfigOptionPoint3 {
    /// Config.hpp:1612 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:1608
    pub fn new() -> Self {
        ConfigOptionPoint3 { value: Vec3d::new(0.0, 0.0, 0.0) }
    }
    /// Config.hpp:1609 `explicit ConfigOptionPoint3(const Vec3d &value)`
    pub fn with_value(value: Vec3d) -> Self {
        ConfigOptionPoint3 { value }
    }
    /// Config.hpp:1611
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoPoint3
    }
    /// Config.hpp:1616-1617 `bool operator< (const ConfigOptionPoint3 &rhs) const throw()`
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        self.value.x < rhs.value.x
            || (self.value.x == rhs.value.x
                && (self.value.y < rhs.value.y || (self.value.y == rhs.value.y && self.value.z < rhs.value.z)))
    }
}

impl ConfigOptionStaticType for ConfigOptionPoint3 {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoPoint3
    }
}

impl ConfigOption for ConfigOptionPoint3 {
    cfg_option_any!();
    cfg_single_value!(Vec3d);
    cfg_single_set_eq_hash!(Vec3d);
    /// Config.hpp:1612
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:1613
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionPoint3 { value: self.value })
    }
    /// Config.hpp:1619-1628
    fn serialize(&self) -> CfgResult<String> {
        let mut ss = ostream_double(self.value.x);
        ss.push(',');
        ss.push_str(&ostream_double(self.value.y));
        ss.push(',');
        ss.push_str(&ostream_double(self.value.z));
        Ok(ss)
    }
    /// Config.hpp:1630-1636
    fn deserialize(&mut self, str: &str, _append: bool) -> CfgResult<bool> {
        let mut v = [self.value.x, self.value.y, self.value.z];
        let ok = sscanf_doubles(str.as_bytes(), b',', &mut v) == 3 || sscanf_doubles(str.as_bytes(), b'x', &mut v) == 3;
        self.value.x = v[0];
        self.value.y = v[1];
        self.value.z = v[2];
        Ok(ok)
    }
}

/// Config.hpp:1643-1740 `class ConfigOptionPointsGroups : public ConfigOptionVector<Vec2ds>`
#[derive(Debug)]
pub struct ConfigOptionPointsGroups {
    pub values: Vec<Vec2ds>,
}

impl ConfigOptionPointsGroups {
    /// Config.hpp:1651 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:1646
    pub fn new() -> Self {
        ConfigOptionPointsGroups { values: Vec::new() }
    }
    /// Config.hpp:1647-1648
    pub fn from_values(values: Vec<Vec2ds>) -> Self {
        ConfigOptionPointsGroups { values }
    }
    /// Config.hpp:1650
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoPointsGroups
    }
    /// Config.hpp:681
    pub fn get_at(&self, i: usize) -> &Vec2ds {
        ConfigOptionVector::<Vec2ds>::get_at(&self.values, i)
    }
    /// Config.hpp:1714-1726 `std::vector<std::string> vserialize_single(int idx) const`
    pub fn vserialize_single(&self, idx: i32) -> Vec<String> {
        let mut ret = Vec::new();
        // assert((idx as usize) < self.values.len()): an `assert` of the C++, compiled out in the reference build (-DNDEBUG).
        for iter in &self.values[idx as usize] {
            let mut ss = String::new();
            serialize_point_x(&mut ss, iter);
            ret.push(ss);
        }
        ret
    }
    /// Config.hpp:1728-1736 `void serialize_single_value(std::ostringstream& ss, const Vec2ds& v) const`
    fn serialize_single_value(ss: &mut String, v: &Vec2ds) {
        for (i, iter) in v.iter().enumerate() {
            if i != 0 {
                ss.push(',');
            }
            serialize_point_x(ss, iter);
        }
    }
}

impl ConfigOptionStaticType for ConfigOptionPointsGroups {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoPointsGroups
    }
}

impl ConfigOption for ConfigOptionPointsGroups {
    cfg_option_any!();
    cfg_vector_common!(Vec2ds);
    cfg_vector_set!(Vec2ds);
    /// Config.hpp:1651
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:1652
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionPointsGroups { values: self.values.clone() })
    }
    /// Config.hpp:1655-1661 `bool operator == (const ConfigOption& rhs) const override`
    fn operator_eq(&self, rhs: &dyn ConfigOption) -> CfgResult<bool> {
        if rhs.r#type() != self.r#type() {
            return Err(ConfigurationError::new("ConfigOptionPointsGroupsTempl: Comparing incompatible types"));
        }
        Ok(&self.values == vector_ref::<Vec2ds>(rhs))
    }
    /// Config.hpp:1662
    fn nullable(&self) -> bool {
        false
    }
    /// Config.hpp:1665-1675
    fn serialize(&self) -> CfgResult<String> {
        let mut ss = String::new();
        for (i, iter) in self.values.iter().enumerate() {
            if i != 0 {
                ss.push('#');
            }
            Self::serialize_single_value(&mut ss, iter);
        }
        Ok(ss)
    }
    /// Config.hpp:1688-1713
    fn deserialize(&mut self, str: &str, append: bool) -> CfgResult<bool> {
        if !append {
            self.values.clear();
        }
        for group_str in getline_split(str.as_bytes(), b'#') {
            let mut group: Vec2ds = Vec::new();
            for point_str in getline_split(group_str, b',') {
                group.push(deserialize_point_x(point_str));
            }
            self.values.push(group);
        }
        Ok(true)
    }
}

impl ConfigOptionVectorBase for ConfigOptionPointsGroups {
    cfg_vector_base_methods!(Vec2ds, None);
    /// Config.hpp:1677-1686
    fn vserialize(&self) -> CfgResult<Vec<String>> {
        let mut ret = Vec::new();
        for points in &self.values {
            let mut ss = String::new();
            Self::serialize_single_value(&mut ss, points);
            ret.push(ss);
        }
        Ok(ret)
    }
    /// Config.hpp:1663
    fn is_nil_at(&self, _idx: usize) -> bool {
        false
    }
    /// Config.hpp:453
    fn set_at_to_nil(&mut self, _i: usize) {}
}

/// Config.hpp:1742-1844 `class ConfigOptionIntsGroups : public ConfigOptionVector<std::vector<int>>`
#[derive(Debug)]
pub struct ConfigOptionIntsGroups {
    pub values: Vec<Vec<i32>>,
}

impl ConfigOptionIntsGroups {
    /// Config.hpp:1750 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:1745
    pub fn new() -> Self {
        ConfigOptionIntsGroups { values: Vec::new() }
    }
    /// Config.hpp:1746-1747
    pub fn from_values(values: Vec<Vec<i32>>) -> Self {
        ConfigOptionIntsGroups { values }
    }
    /// Config.hpp:1749
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoIntsGroups
    }
    /// Config.hpp:1765-1774 `bool operator<(const ConfigOptionIntsGroups &rhs) const throw()`
    /// (reads `rhs.values[i]` for every index of this vector, as the C++ does).
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        let is_lower = true;
        for i in 0..self.values.len() {
            if self.values[i] == rhs.values[i] {
                continue;
            }
            return self.values[i] < rhs.values[i];
        }
        is_lower
    }
    /// Config.hpp:681
    pub fn get_at(&self, i: usize) -> &Vec<i32> {
        ConfigOptionVector::<Vec<i32>>::get_at(&self.values, i)
    }
    /// Config.hpp:1819-1829 `std::vector<std::string> vserialize_single(int idx) const`
    pub fn vserialize_single(&self, idx: i32) -> Vec<String> {
        let mut ret = Vec::new();
        // assert((idx as usize) < self.values.len()): an `assert` of the C++, compiled out in the reference build (-DNDEBUG).
        for iter in &self.values[idx as usize] {
            ret.push(ostream_int(*iter));
        }
        ret
    }
    /// Config.hpp:1832-1839 `void serialize_single_value(std::ostringstream &ss, const std::vector<int> &v) const`
    fn serialize_single_value(ss: &mut String, v: &[i32]) {
        for (i, iter) in v.iter().enumerate() {
            if i != 0 {
                ss.push(',');
            }
            ss.push_str(&ostream_int(*iter));
        }
    }
}

impl ConfigOptionStaticType for ConfigOptionIntsGroups {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoIntsGroups
    }
}

impl ConfigOption for ConfigOptionIntsGroups {
    cfg_option_any!();
    cfg_vector_common!(Vec<i32>);
    cfg_vector_set!(Vec<i32>);
    /// Config.hpp:1750
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:1751
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionIntsGroups { values: self.values.clone() })
    }
    /// Config.hpp:1758-1764 `bool operator==(const ConfigOption &rhs) const override`
    fn operator_eq(&self, rhs: &dyn ConfigOption) -> CfgResult<bool> {
        if rhs.r#type() != self.r#type() {
            return Err(ConfigurationError::new("ConfigConfigOptionIntsGroups: Comparing incompatible types"));
        }
        Ok(&self.values == vector_ref::<Vec<i32>>(rhs))
    }
    /// Config.hpp:1775
    fn nullable(&self) -> bool {
        false
    }
    /// Config.hpp:1778-1788
    fn serialize(&self) -> CfgResult<String> {
        let mut ss = String::new();
        for (i, iter) in self.values.iter().enumerate() {
            if i != 0 {
                ss.push('#');
            }
            Self::serialize_single_value(&mut ss, iter);
        }
        Ok(ss)
    }
    /// Config.hpp:1801-1818
    fn deserialize(&mut self, str: &str, append: bool) -> CfgResult<bool> {
        if !append {
            self.values.clear();
        }
        // FIDELITY-NOTE: `int value; std::istringstream(value_str) >> value;` is uninitialized
        // for an item that is empty or white space only, see ConfigOptionFloatsTempl::deserialize.
        let mut value: i32 = 0;
        for group_str in getline_split(str.as_bytes(), b'#') {
            let mut group_values: Vec<i32> = Vec::new();
            for value_str in getline_split(group_str, b',') {
                if let Some(v) = istream_int(value_str).value {
                    value = v;
                }
                group_values.push(value);
            }
            self.values.push(group_values);
        }
        Ok(true)
    }
}

impl ConfigOptionVectorBase for ConfigOptionIntsGroups {
    cfg_vector_base_methods!(Vec<i32>, None);
    /// Config.hpp:1790-1799
    fn vserialize(&self) -> CfgResult<Vec<String>> {
        let mut ret = Vec::new();
        for value in &self.values {
            let mut ss = String::new();
            Self::serialize_single_value(&mut ss, value);
            ret.push(ss);
        }
        Ok(ret)
    }
    /// Config.hpp:1776
    fn is_nil_at(&self, _idx: usize) -> bool {
        false
    }
    /// Config.hpp:453
    fn set_at_to_nil(&mut self, _i: usize) {}
}

/// Config.hpp:1847-1883 `class ConfigOptionBool : public ConfigOptionSingle<bool>`
#[derive(Debug)]
pub struct ConfigOptionBool {
    pub value: bool,
}

impl ConfigOptionBool {
    /// Config.hpp:1854 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:1850
    pub fn new() -> Self {
        ConfigOptionBool { value: false }
    }
    /// Config.hpp:1851 `explicit ConfigOptionBool(bool _value)`
    pub fn with_value(value: bool) -> Self {
        ConfigOptionBool { value }
    }
    /// Config.hpp:1853
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoBool
    }
    /// Config.hpp:1859 `bool operator< (const ConfigOptionBool &rhs) const throw()`
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        (self.value as i32) < (rhs.value as i32)
    }
}

impl ConfigOptionStaticType for ConfigOptionBool {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoBool
    }
}

impl ConfigOption for ConfigOptionBool {
    cfg_option_any!();
    cfg_single_value!(bool);
    cfg_single_set_eq_hash!(bool);
    /// Config.hpp:1854
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:1855 `bool getBool() const override`
    fn get_bool(&self) -> CfgResult<bool> {
        Ok(self.value)
    }
    /// Config.hpp:1856
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionBool { value: self.value })
    }
    /// Config.hpp:1861-1864
    fn serialize(&self) -> CfgResult<String> {
        Ok(if self.value { "1" } else { "0" }.to_string())
    }
    /// Config.hpp:1866-1878
    fn deserialize(&mut self, str: &str, _append: bool) -> CfgResult<bool> {
        if str == "1" {
            self.value = true;
            return Ok(true);
        }
        if str == "0" {
            self.value = false;
            return Ok(true);
        }
        Ok(false)
    }
}

/// Config.hpp:1885-1993 `template<bool NULLABLE> class ConfigOptionBoolsTempl : public ConfigOptionVector<unsigned char>`
#[derive(Debug)]
pub struct ConfigOptionBoolsTempl<const NULLABLE: bool> {
    pub values: Vec<u8>,
}

/// Config.hpp:1995
pub type ConfigOptionBools = ConfigOptionBoolsTempl<false>;
/// Config.hpp:1996
pub type ConfigOptionBoolsNullable = ConfigOptionBoolsTempl<true>;

impl<const NULLABLE: bool> ConfigOptionBoolsTempl<NULLABLE> {
    /// Config.hpp:1897 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:1889
    pub fn new() -> Self {
        Self { values: Vec::new() }
    }
    /// Config.hpp:1890 `explicit ConfigOptionBoolsTempl(size_t n, bool value)`
    pub fn with_size(n: usize, value: bool) -> Self {
        Self { values: vec![value as u8; n] }
    }
    /// Config.hpp:1891 `explicit ConfigOptionBoolsTempl(std::initializer_list<bool> il)`
    pub fn from_bools(il: &[bool]) -> Self {
        Self { values: il.iter().map(|&b| b as u8).collect() }
    }
    /// Config.hpp:1892-1894 (`unsigned char` values)
    pub fn from_values(vec: Vec<u8>) -> Self {
        Self { values: vec }
    }
    /// Config.hpp:1896
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoBools
    }
    /// Config.hpp:1901 `bool operator< (const ConfigOptionBoolsTempl &rhs) const throw()`
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        self.values < rhs.values
    }
    /// Config.hpp:1905: special "nil" value to be stored into the vector if this->supports_nil().
    pub fn nil_value() -> u8 {
        u8::MAX
    }
    /// Config.hpp:1915-1918 `bool& get_at(size_t i)`: the byte, to be written as 0 or 1.
    pub fn get_at_mut(&mut self, i: usize) -> &mut u8 {
        ConfigOptionVector::<u8>::get_at_mut(&mut self.values, i)
    }
    /// Config.hpp:1921 `bool get_at(size_t i) const`
    pub fn get_at(&self, i: usize) -> bool {
        (if i < self.values.len() { self.values[i] } else { self.values[0] }) != 0
    }
    /// Config.hpp:1945-1972
    pub fn deserialize_with_substitutions(
        &mut self,
        str: &str,
        append: bool,
        substitution: config_helpers::DeserializationSubstitution,
    ) -> CfgResult<config_helpers::DeserializationResult> {
        use config_helpers::{DeserializationResult, DeserializationSubstitution};
        if !append {
            self.values.clear();
        }
        let mut substituted = false;
        for item_str in getline_split(str.as_bytes(), b',') {
            let item_str = boost_trim(item_str);
            let new_value: u8;
            if item_str == b"nil" {
                if NULLABLE {
                    new_value = Self::nil_value();
                } else {
                    return Err(ConfigurationError::new("Deserializing nil into a non-nullable object"));
                }
            } else if item_str == b"1" {
                new_value = 1;
            } else if item_str == b"0" {
                new_value = 0;
            } else if substitution != DeserializationSubstitution::Disabled
                && config_helpers::looks_like_enum_value(&bytes_to_string(item_str))
            {
                new_value = (config_helpers::enum_looks_like_true_value(&bytes_to_string(item_str))
                    || substitution == DeserializationSubstitution::DefaultsToTrue) as u8;
                substituted = true;
            } else {
                return Ok(DeserializationResult::Failed);
            }
            self.values.push(new_value);
        }
        Ok(if substituted { DeserializationResult::Substituted } else { DeserializationResult::Loaded })
    }
    /// Config.hpp:1980-1988 `void serialize_single_value(std::ostringstream &ss, const unsigned char v) const`
    fn serialize_single_value(ss: &mut String, v: u8) -> CfgResult<()> {
        if v == Self::nil_value() {
            if NULLABLE {
                ss.push_str("nil");
            } else {
                return Err(ConfigurationError::new("Serializing NaN"));
            }
        } else {
            ss.push_str(if v != 0 { "1" } else { "0" });
        }
        Ok(())
    }
}

impl<const NULLABLE: bool> ConfigOptionStaticType for ConfigOptionBoolsTempl<NULLABLE> {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoBools
    }
}

impl<const NULLABLE: bool> ConfigOption for ConfigOptionBoolsTempl<NULLABLE> {
    cfg_option_any!();
    cfg_vector_common!(u8);
    cfg_vector_set!(u8);
    cfg_vector_eq!(u8);
    /// Config.hpp:1897
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:1898
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(Self { values: self.values.clone() })
    }
    /// Config.hpp:1903
    fn nullable(&self) -> bool {
        NULLABLE
    }
    /// Config.hpp:1907
    fn is_nil(&self) -> bool {
        for v in &self.values {
            if *v != Self::nil_value() {
                return false;
            }
        }
        true
    }
    /// Config.hpp:1923-1932
    fn serialize(&self) -> CfgResult<String> {
        let mut ss = String::new();
        for (i, v) in self.values.iter().enumerate() {
            if i != 0 {
                ss.push(',');
            }
            Self::serialize_single_value(&mut ss, *v)?;
        }
        Ok(ss)
    }
    /// Config.hpp:1974-1977
    fn deserialize(&mut self, str: &str, append: bool) -> CfgResult<bool> {
        Ok(self.deserialize_with_substitutions(str, append, config_helpers::DeserializationSubstitution::Disabled)?
            == config_helpers::DeserializationResult::Loaded)
    }
}

impl<const NULLABLE: bool> ConfigOptionVectorBase for ConfigOptionBoolsTempl<NULLABLE> {
    cfg_vector_base_methods!(u8, Some(Self::nil_value()));
    /// Config.hpp:1934-1943
    fn vserialize(&self) -> CfgResult<Vec<String>> {
        let mut vv = Vec::new();
        for v in &self.values {
            let mut ss = String::new();
            Self::serialize_single_value(&mut ss, *v)?;
            vv.push(ss);
        }
        Ok(vv)
    }
    /// Config.hpp:1908
    fn is_nil_at(&self, idx: usize) -> bool {
        self.values[idx] == Self::nil_value()
    }
    /// Config.hpp:1909-1913
    fn set_at_to_nil(&mut self, i: usize) {
        // assert(self.nullable() && i < self.values.len()): an `assert` of the C++, compiled out in the reference build (-DNDEBUG).
        self.values[i] = Self::nil_value();
    }
}

/// The enum types `T` of `ConfigOptionEnum<T>`: the two static maps the C++ specializes per
/// enum (Config.hpp:2057-2059) and the conversion from and to `int`.
pub trait ConfigEnum: Copy + PartialEq + Send + Sync + 'static {
    /// Config.hpp:2057 `static const t_config_enum_names& get_enum_names();`
    fn get_enum_names() -> &'static t_config_enum_names;
    /// Config.hpp:2059 `static const t_config_enum_values& get_enum_values();`
    fn get_enum_values() -> &'static t_config_enum_values;
    /// `(int)value`
    fn to_int(self) -> i32;
    /// `T(val)`
    fn from_int(val: i32) -> Self;
}

/// Config.hpp:2003-2070 `template <class T> class ConfigOptionEnum : public ConfigOptionSingle<T>`
pub struct ConfigOptionEnum<T: ConfigEnum> {
    pub value: T,
}

impl<T: ConfigEnum> ConfigOptionEnum<T> {
    /// Config.hpp:2012 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:2008: by default, use the first value (0) of the T enum type
    pub fn new() -> Self {
        ConfigOptionEnum { value: T::from_int(0) }
    }
    /// Config.hpp:2009 `explicit ConfigOptionEnum(T _value)`
    pub fn with_value(value: T) -> Self {
        ConfigOptionEnum { value }
    }
    /// Config.hpp:2011
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoEnum
    }
    /// Config.hpp:2016 `bool operator< (const ConfigOptionEnum<T> &rhs) const throw()`
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        self.value.to_int() < rhs.value.to_int()
    }
    /// Config.hpp:2048-2054 `static bool has(T value)`
    pub fn has(value: T) -> bool {
        for (_k, v) in T::get_enum_values() {
            if *v == value.to_int() {
                return true;
            }
        }
        false
    }
    /// Config.hpp:2057: map from an enum integer value to an enum name.
    pub fn get_enum_names() -> &'static t_config_enum_names {
        T::get_enum_names()
    }
    /// Config.hpp:2059: map from an enum name to an enum integer value.
    pub fn get_enum_values() -> &'static t_config_enum_values {
        T::get_enum_values()
    }
    /// Config.hpp:2061-2069 `static bool from_string(const std::string &str, T &value)`
    pub fn from_string(str: &str, value: &mut T) -> bool {
        let enum_keys_map = T::get_enum_values();
        match enum_keys_map.get(str) {
            None => false,
            Some(v) => {
                *value = T::from_int(*v);
                true
            }
        }
    }
}

impl<T: ConfigEnum> ConfigOptionStaticType for ConfigOptionEnum<T> {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoEnum
    }
}

impl<T: ConfigEnum> ConfigOption for ConfigOptionEnum<T> {
    cfg_option_any!();
    cfg_single_value!(T);
    /// Config.hpp:2012
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:2013
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionEnum::<T> { value: self.value })
    }
    /// Config.hpp:2017 `int getInt() const override`
    fn get_int(&self) -> CfgResult<i32> {
        Ok(self.value.to_int())
    }
    /// Config.hpp:2018 `void setInt(int val) override`
    fn set_int(&mut self, val: i32) -> CfgResult<()> {
        self.value = T::from_int(val);
        Ok(())
    }
    /// Config.hpp:2020-2026: rhs could be of the following type: ConfigOptionEnumGeneric or ConfigOptionEnum<T>
    fn operator_eq(&self, rhs: &dyn ConfigOption) -> CfgResult<bool> {
        if rhs.r#type() != self.r#type() {
            return Err(ConfigurationError::new("ConfigOptionEnum<T>: Comparing incompatible types"));
        }
        Ok(self.value == T::from_int(rhs.get_int()?))
    }
    /// Config.hpp:2028-2033
    fn set(&mut self, rhs: &dyn ConfigOption) -> CfgResult<()> {
        if rhs.r#type() != self.r#type() {
            return Err(ConfigurationError::new("ConfigOptionEnum<T>: Assigning an incompatible type"));
        }
        self.value = T::from_int(rhs.get_int()?);
        Ok(())
    }
    /// Config.hpp:335 `std::hash<T>` of an enum: the hash of its integer value.
    fn hash(&self) -> u64 {
        std_hash_int(self.value.to_int())
    }
    /// Config.hpp:2035-2040
    fn serialize(&self) -> CfgResult<String> {
        let names = T::get_enum_names();
        Ok(names[self.value.to_int() as usize].clone())
    }
    /// Config.hpp:2042-2046
    fn deserialize(&mut self, str: &str, _append: bool) -> CfgResult<bool> {
        Ok(Self::from_string(str, &mut self.value))
    }
}

/// Config.hpp:2072-2126 `class ConfigOptionEnumGeneric : public ConfigOptionInt`: generic enum
/// configuration value. We use this one in DynamicConfig objects when creating a config value
/// object for ConfigOptionType == coEnum.
#[derive(Debug)]
pub struct ConfigOptionEnumGeneric {
    pub value: i32,
    pub keys_map: Option<&'static t_config_enum_values>,
}

impl ConfigOptionEnumGeneric {
    /// Config.hpp:2084 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:2078 `ConfigOptionEnumGeneric(const t_config_enum_values* keys_map = nullptr)`
    pub fn new(keys_map: Option<&'static t_config_enum_values>) -> Self {
        ConfigOptionEnumGeneric { value: 0, keys_map }
    }
    /// Config.hpp:2079 `explicit ConfigOptionEnumGeneric(const t_config_enum_values* keys_map, int value)`
    pub fn with_value(keys_map: Option<&'static t_config_enum_values>, value: i32) -> Self {
        ConfigOptionEnumGeneric { value, keys_map }
    }
    /// Config.hpp:2083
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoEnum
    }
    /// Config.hpp:2088 `bool operator< (const ConfigOptionEnumGeneric &rhs) const throw()`
    pub fn operator_lt(&self, rhs: &Self) -> bool {
        self.value < rhs.value
    }
}

impl ConfigOptionStaticType for ConfigOptionEnumGeneric {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoEnum
    }
}

impl ConfigOption for ConfigOptionEnumGeneric {
    cfg_option_any!();
    cfg_single_value!(i32);
    /// Config.hpp:2084
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:2085
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(ConfigOptionEnumGeneric { value: self.value, keys_map: self.keys_map })
    }
    /// Config.hpp:1001 (ConfigOptionInt::getInt)
    fn get_int(&self) -> CfgResult<i32> {
        Ok(self.value)
    }
    /// Config.hpp:1002 (ConfigOptionInt::setInt)
    fn set_int(&mut self, val: i32) -> CfgResult<()> {
        self.value = val;
        Ok(())
    }
    /// Config.hpp:2090-2096
    fn operator_eq(&self, rhs: &dyn ConfigOption) -> CfgResult<bool> {
        if rhs.r#type() != self.r#type() {
            return Err(ConfigurationError::new("ConfigOptionEnumGeneric: Comparing incompatible types"));
        }
        // rhs could be of the following type: ConfigOptionEnumGeneric or ConfigOptionEnum<T>
        Ok(self.value == rhs.get_int()?)
    }
    /// Config.hpp:2098-2103
    fn set(&mut self, rhs: &dyn ConfigOption) -> CfgResult<()> {
        if rhs.r#type() != self.r#type() {
            return Err(ConfigurationError::new("ConfigOptionEnumGeneric: Assigning an incompatible type"));
        }
        // rhs could be of the following type: ConfigOptionEnumGeneric or ConfigOptionEnum<T>
        self.value = rhs.get_int()?;
        Ok(())
    }
    /// Config.hpp:335 (ConfigOptionSingle<int>::hash)
    fn hash(&self) -> u64 {
        std_hash_int(self.value)
    }
    /// Config.hpp:2105-2111
    fn serialize(&self) -> CfgResult<String> {
        for (k, v) in self.keys_map.expect("ConfigOptionEnumGeneric::keys_map is null") {
            if *v == self.value {
                return Ok(k.clone());
            }
        }
        Ok(String::new())
    }
    /// Config.hpp:2113-2121
    fn deserialize(&mut self, str: &str, _append: bool) -> CfgResult<bool> {
        match self.keys_map.expect("ConfigOptionEnumGeneric::keys_map is null").get(str) {
            None => Ok(false),
            Some(v) => {
                self.value = *v;
                Ok(true)
            }
        }
    }
}

/// The `values` of a `const ConfigOptionInts&`: a ConfigOptionInts or one of the enums classes,
/// which derive from it in the C++.
fn ints_of(rhs: &dyn ConfigOption) -> &Vec<i32> {
    if let Some(o) = rhs.as_any().downcast_ref::<ConfigOptionInts>() {
        &o.values
    } else if let Some(o) = rhs.as_any().downcast_ref::<ConfigOptionEnumsGeneric>() {
        &o.values
    } else {
        &rhs.as_any()
            .downcast_ref::<ConfigOptionEnumsGenericNullable>()
            .expect("const ConfigOptionInts&: not a ConfigOptionInts or a class derived from it")
            .values
    }
}

/// Config.hpp:2128-2227 `template <bool NULLABLE> class ConfigOptionEnumsGenericTempl : public ConfigOptionInts`
/// (BBS). The base class is the non-nullable ConfigOptionInts whatever NULLABLE is.
#[derive(Debug)]
pub struct ConfigOptionEnumsGenericTempl<const NULLABLE: bool> {
    pub values: Vec<i32>,
    pub keys_map: Option<&'static t_config_enum_values>,
}

/// Config.hpp:2229
pub type ConfigOptionEnumsGeneric = ConfigOptionEnumsGenericTempl<false>;
/// Config.hpp:2230
pub type ConfigOptionEnumsGenericNullable = ConfigOptionEnumsGenericTempl<true>;

impl<const NULLABLE: bool> ConfigOptionEnumsGenericTempl<NULLABLE> {
    /// Config.hpp:2142 `ConfigOptionType type() const override { return static_type(); }` (the `ConfigOption`
    /// trait has it as `r#type`; this is the C++ name with the keyword suffix).
    pub fn type_(&self) -> ConfigOptionType {
        <Self as ConfigOptionStaticType>::static_type()
    }
    /// Config.hpp:2133 `ConfigOptionEnumsGenericTempl(const t_config_enum_values *keys_map = nullptr)`
    pub fn new(keys_map: Option<&'static t_config_enum_values>) -> Self {
        Self { values: Vec::new(), keys_map }
    }
    /// Config.hpp:2134 `explicit ConfigOptionEnumsGenericTempl(const t_config_enum_values *keys_map, size_t size, int value)`
    pub fn with_size(keys_map: Option<&'static t_config_enum_values>, size: usize, value: i32) -> Self {
        Self { values: vec![value; size], keys_map }
    }
    /// Config.hpp:2136-2137 (`keys_map` stays null)
    pub fn from_values(vec: Vec<i32>) -> Self {
        Self { values: vec, keys_map: None }
    }
    /// Config.hpp:2141
    pub fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoEnums
    }
    /// Config.hpp:2145 `bool operator< (const ConfigOptionInts& rhs) const throw()`. `rhs` is a
    /// ConfigOptionInts or a class derived from it (ConfigOptionEnumsGeneric,
    /// ConfigOptionEnumsGenericNullable), as the C++ reference may be.
    pub fn operator_lt(&self, rhs: &dyn ConfigOption) -> bool {
        self.values < *ints_of(rhs)
    }
    /// Config.hpp:2147-2152 `bool operator==(const ConfigOptionInts& rhs) const`: `rhs` as in
    /// operator_lt; the virtual `type()` of a derived class is coEnums.
    pub fn operator_eq_ints(&self, rhs: &dyn ConfigOption) -> CfgResult<bool> {
        if rhs.r#type() != self.r#type() {
            return Err(ConfigurationError::new("ConfigOptionEnumsGeneric: Comparing incompatible types"));
        }
        Ok(self.values == *ints_of(rhs))
    }
    /// Config.hpp:1051 (ConfigOptionInts::nil_value)
    pub fn nil_value() -> i32 {
        i32::MAX
    }
    /// Config.hpp:681
    pub fn get_at(&self, i: usize) -> &i32 {
        ConfigOptionVector::<i32>::get_at(&self.values, i)
    }
    /// Config.hpp:687
    pub fn get_at_mut(&mut self, i: usize) -> &mut i32 {
        ConfigOptionVector::<i32>::get_at_mut(&mut self.values, i)
    }
    /// Config.hpp:2210-2223 `void serialize_single_value(std::ostringstream& ss, const int v) const`:
    /// every key that maps to the value is written.
    fn serialize_single_value(&self, ss: &mut String, v: i32) -> CfgResult<()> {
        if v == Self::nil_value() {
            if NULLABLE {
                ss.push_str("nil");
            } else {
                return Err(ConfigurationError::new("Serializing NaN"));
            }
        } else {
            for (k, kv) in self.keys_map.expect("ConfigOptionEnumsGenericTempl::keys_map is null") {
                if *kv == v {
                    ss.push_str(k);
                }
            }
        }
        Ok(())
    }
}

impl<const NULLABLE: bool> ConfigOptionStaticType for ConfigOptionEnumsGenericTempl<NULLABLE> {
    fn static_type() -> ConfigOptionType {
        ConfigOptionType::CoEnums
    }
}

impl<const NULLABLE: bool> ConfigOption for ConfigOptionEnumsGenericTempl<NULLABLE> {
    cfg_option_any!();
    cfg_vector_common!(i32);
    cfg_vector_eq!(i32);
    /// Config.hpp:2142
    fn r#type(&self) -> ConfigOptionType {
        Self::static_type()
    }
    /// Config.hpp:2143
    fn clone(&self) -> Box<dyn ConfigOption> {
        Box::new(Self { values: self.values.clone(), keys_map: self.keys_map })
    }
    /// Config.hpp:2153
    fn nullable(&self) -> bool {
        NULLABLE
    }
    /// Config.hpp:1053 (ConfigOptionInts::is_nil)
    fn is_nil(&self) -> bool {
        for v in &self.values {
            if *v != Self::nil_value() {
                return false;
            }
        }
        true
    }
    /// Config.hpp:2155-2160. The C++ `dynamic_cast`s rhs to this very instantiation and
    /// dereferences the result: an option of the other nullability is a null pointer there.
    fn set(&mut self, rhs: &dyn ConfigOption) -> CfgResult<()> {
        if rhs.r#type() != self.r#type() {
            return Err(ConfigurationError::new("ConfigOptionEnumGeneric: Assigning an incompatible type"));
        }
        // rhs could be of the following type: ConfigOptionEnumsGeneric
        let rhs = rhs
            .as_any()
            .downcast_ref::<ConfigOptionEnumsGenericTempl<NULLABLE>>()
            .expect("ConfigOptionEnumsGenericTempl::set(): dynamic_cast<const ConfigOptionEnumsGenericTempl*>(rhs) is null");
        self.values = rhs.values.clone();
        Ok(())
    }
    /// Config.hpp:2162-2171
    fn serialize(&self) -> CfgResult<String> {
        let mut ss = String::new();
        for (i, v) in self.values.iter().enumerate() {
            if i != 0 {
                ss.push(',');
            }
            self.serialize_single_value(&mut ss, *v)?;
        }
        Ok(ss)
    }
    /// Config.hpp:2185-2207
    fn deserialize(&mut self, str: &str, append: bool) -> CfgResult<bool> {
        if !append {
            self.values.clear();
        }
        for item_str in getline_split(str.as_bytes(), b',') {
            let item_str = boost_trim(item_str);
            if item_str == b"nil" {
                if NULLABLE {
                    self.values.push(Self::nil_value());
                } else {
                    return Err(ConfigurationError::new("Deserializing nil into a non-nullable object"));
                }
            } else {
                let keys_map = self.keys_map.expect("ConfigOptionEnumsGenericTempl::keys_map is null");
                match keys_map.get(bytes_to_string(item_str).as_str()) {
                    None => return Ok(false),
                    Some(v) => self.values.push(*v),
                }
            }
        }
        Ok(true)
    }
}

impl<const NULLABLE: bool> ConfigOptionVectorBase for ConfigOptionEnumsGenericTempl<NULLABLE> {
    cfg_vector_base_methods!(i32, Some(Self::nil_value()));
    /// Config.hpp:2173-2183
    fn vserialize(&self) -> CfgResult<Vec<String>> {
        let mut vv = Vec::with_capacity(self.values.len());
        for v in &self.values {
            let mut ss = String::new();
            self.serialize_single_value(&mut ss, *v)?;
            vv.push(ss);
        }
        Ok(vv)
    }
    /// Config.hpp:1054 (ConfigOptionInts::is_nil(size_t))
    fn is_nil_at(&self, idx: usize) -> bool {
        self.values[idx] == Self::nil_value()
    }
    /// Config.hpp:1055-1059 (ConfigOptionInts::set_at_to_nil)
    fn set_at_to_nil(&mut self, i: usize) {
        self.values[i] = Self::nil_value();
    }
}
// ---------------------------------------------------------------------------
// Config.hpp:2232-2476, Config.cpp:252-452: ConfigOptionDef, ConfigDef.
// ---------------------------------------------------------------------------

/// Config.hpp:2236-2253 `enum class GUIType`
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum GUIType {
    Undefined,
    /// Open enums, integer value could be one of the enumerated values or something else.
    IEnumOpen,
    /// Open enums, float value could be one of the enumerated values or something else.
    FEnumOpen,
    /// Color picker, string value.
    Color,
    SelectOpen,
    /// Currently unused.
    Slider,
    /// Static text
    Legend,
    /// Vector value, but edited as a single string.
    OneString,
    MultiVariant,
}

/// Config.hpp:2232-2431: definition of a configuration value for the purpose of GUI
/// presentation, editing, value mapping and config file handling.
#[derive(Clone)]
pub struct ConfigOptionDef {
    /// Identifier of this option. It is stored here so that it is accessible through the
    /// by_serialization_key_ordinal map.
    pub opt_key: t_config_option_key,
    /// What type? bool, int, string etc.
    pub r#type: ConfigOptionType,
    /// If a type is nullable, then it accepts a "nil" value (scalar) or "nil" values (vector).
    pub nullable: bool,
    /// Default value of this option (`Slic3r::clonable_ptr<const ConfigOption>`).
    pub default_value: Option<Box<dyn ConfigOption>>,
    /// Usually empty. Special values - "i_enum_open", "f_enum_open" to provide combo box for int
    /// or float selection, "select_open" - to open a selection dialog.
    pub gui_type: GUIType,
    /// Usually empty. Otherwise "serialized" or "show_value". The flags may be combined.
    pub gui_flags: String,
    /// Label of the GUI input field.
    pub label: String,
    pub full_label: String,
    /// With which printer technology is this configuration valid?
    pub printer_technology: PrinterTechnology,
    /// Category of a configuration field, from the GUI perspective.
    pub category: String,
    /// A tooltip text shown in the GUI.
    pub tooltip: String,
    /// Text right from the input field, usually a unit of measurement.
    pub sidetext: String,
    /// Format of this parameter on a command line.
    pub cli: String,
    /// BBS: add cli command line params
    pub cli_params: String,
    /// Set for type == coFloatOrPercent. It provides a link to a configuration value, of which
    /// this option provides a ratio.
    pub ratio_over: t_config_option_key,
    /// True for multiline strings.
    pub multiline: bool,
    /// For text input: If true, the GUI text box spans the complete page width.
    pub full_width: bool,
    /// For text input: If true, the GUI formats text as code (fixed-width)
    pub is_code: bool,
    /// Not editable. Currently only used for the display of the number of threads.
    pub readonly: bool,
    /// Height of a multiline GUI text box.
    pub height: i32,
    /// Optional width of an input field.
    pub width: i32,
    /// <min, max> limit of a numeric input.
    pub min: f64,
    pub max: f64,
    /// To check if it's not a typo and a % is missing
    pub max_literal: f64,
    pub mode: ConfigOptionMode,
    /// Legacy names for this configuration option. Used when parsing legacy configuration file.
    pub aliases: Vec<t_config_option_key>,
    /// Sometimes a single value may well define multiple values in a "beginner" mode.
    pub shortcut: Vec<t_config_option_key>,
    /// Definition of values / labels for a combo box.
    pub enum_values: Vec<String>,
    pub enum_labels: Vec<String>,
    /// For enums (when type == coEnum). Maps enum_values to enums.
    /// Initialized by ConfigOptionEnum<xxx>::get_enum_values()
    pub enum_keys_map: Option<&'static t_config_enum_values>,
    /// 0 is an invalid key.
    pub serialization_key_ordinal: usize,
}

impl Default for ConfigOptionDef {
    /// The member initializers of Config.hpp:2256-2423.
    fn default() -> Self {
        ConfigOptionDef {
            opt_key: String::new(),
            r#type: ConfigOptionType::CoNone,
            nullable: false,
            default_value: None,
            gui_type: GUIType::Undefined,
            gui_flags: String::new(),
            label: String::new(),
            full_label: String::new(),
            printer_technology: PrinterTechnology::PtUnknown,
            category: String::new(),
            tooltip: String::new(),
            sidetext: String::new(),
            cli: String::new(),
            cli_params: String::new(),
            ratio_over: String::new(),
            multiline: false,
            full_width: false,
            is_code: false,
            readonly: false,
            height: -1,
            width: -1,
            min: Self::MIN_DEFAULT,
            max: Self::MAX_DEFAULT,
            max_literal: 1.0,
            mode: ConfigOptionMode::ComSimple,
            aliases: Vec::new(),
            shortcut: Vec::new(),
            enum_values: Vec::new(),
            enum_labels: Vec::new(),
            enum_keys_map: None,
            serialization_key_ordinal: 0,
        }
    }
}

impl ConfigOptionDef {
    /// Config.hpp:2394 `static constexpr double min_default = -DBL_MAX;`
    pub const MIN_DEFAULT: f64 = -f64::MAX;
    /// Config.hpp:2395 `static constexpr double max_default = DBL_MAX;`
    pub const MAX_DEFAULT: f64 = f64::MAX;
    /// Config.hpp:2430: assign this key to cli to disable CLI for this option.
    pub const NOCLI: &'static str = "~~~noCLI";

    /// Config.hpp:2263 `void set_default_value(const ConfigOption* ptr)`
    pub fn set_default_value(&mut self, ptr: Box<dyn ConfigOption>) {
        self.default_value = Some(ptr);
    }

    /// Config.hpp:2264 `template<typename T> const T* get_default_value() const`
    /// (a `static_cast` in C++; `None` here when the default is of another class).
    pub fn get_default_value<T: ConfigOption>(&self) -> Option<&T> {
        self.default_value.as_ref().and_then(|d| d.as_any().downcast_ref::<T>())
    }

    /// Config.hpp:2415-2420 `bool has_enum_value(const std::string &value) const`
    pub fn has_enum_value(&self, value: &str) -> bool {
        for v in &self.enum_values {
            if v == value {
                return true;
            }
        }
        false
    }

    /// Config.cpp:252-270: returns the alternative CLI arguments for the given option.
    /// If there are no cli arguments defined, use the key and replace underscores with dashes.
    pub fn cli_args(&self, key: &str) -> Vec<String> {
        let mut args: Vec<String> = Vec::new();
        if self.cli != Self::NOCLI {
            let cli = &self.cli;
            if cli.is_empty() {
                // Convert an option key to CLI argument by replacing underscores with dashes.
                let opt = key.replace('_', "-");
                args.push(opt);
            } else {
                // boost::split(args, cli, boost::is_any_of("|"))
                args = cli.split('|').map(|s| s.to_string()).collect();
            }
        }
        args
    }

    /// Config.cpp:272-307: create an empty option to be used as a base for deserialization of
    /// DynamicConfig.
    pub fn create_empty_option(&self) -> CfgResult<Box<dyn ConfigOption>> {
        use ConfigOptionType::*;
        if self.nullable {
            match self.r#type {
                CoFloats => Ok(Box::new(ConfigOptionFloatsNullable::new())),
                CoInts => Ok(Box::new(ConfigOptionIntsNullable::new())),
                CoPercents => Ok(Box::new(ConfigOptionPercentsNullable::new())),
                CoFloatsOrPercents => Ok(Box::new(ConfigOptionFloatsOrPercentsNullable::new())),
                CoBools => Ok(Box::new(ConfigOptionBoolsNullable::new())),
                _ => Err(ConfigurationError::new(format!("Unknown option type for nullable option {}", self.label))),
            }
        } else {
            match self.r#type {
                CoFloat => Ok(Box::new(ConfigOptionFloat::new())),
                CoFloats => Ok(Box::new(ConfigOptionFloats::new())),
                CoInt => Ok(Box::new(ConfigOptionInt::new())),
                CoInts => Ok(Box::new(ConfigOptionInts::new())),
                CoString => Ok(Box::new(ConfigOptionString::new())),
                CoStrings => Ok(Box::new(ConfigOptionStrings::new())),
                CoPercent => Ok(Box::new(ConfigOptionPercent::new())),
                CoPercents => Ok(Box::new(ConfigOptionPercents::new())),
                CoFloatOrPercent => Ok(Box::new(ConfigOptionFloatOrPercent::new())),
                CoFloatsOrPercents => Ok(Box::new(ConfigOptionFloatsOrPercents::new())),
                CoPoint => Ok(Box::new(ConfigOptionPoint::new())),
                CoPoints => Ok(Box::new(ConfigOptionPoints::new())),
                CoPoint3 => Ok(Box::new(ConfigOptionPoint3::new())),
                CoBool => Ok(Box::new(ConfigOptionBool::new())),
                CoBools => Ok(Box::new(ConfigOptionBools::new())),
                CoEnum => Ok(Box::new(ConfigOptionEnumGeneric::new(self.enum_keys_map))),
                // BBS
                CoEnums => Ok(Box::new(ConfigOptionEnumsGeneric::new(self.enum_keys_map))),
                _ => Err(ConfigurationError::new(format!("Unknown option type for option {}", self.label))),
            }
        }
    }

    /// Config.cpp:309-330: create a default option to be inserted into a DynamicConfig.
    pub fn create_default_option(&self) -> CfgResult<Box<dyn ConfigOption>> {
        if let Some(default_value) = &self.default_value {
            let r#type = default_value.r#type();
            if r#type == ConfigOptionType::CoEnum {
                return Ok(Box::new(ConfigOptionEnumGeneric::with_value(self.enum_keys_map, default_value.get_int()?)));
            }

            if r#type == ConfigOptionType::CoEnums {
                let mut cloned = ConfigOption::clone(&**default_value);
                if default_value.nullable() {
                    let opt = cloned
                        .as_any_mut()
                        .downcast_mut::<ConfigOptionEnumsGenericNullable>()
                        .expect("dynamic_cast<ConfigOptionEnumsGenericNullable*>(default_value->clone()) is null");
                    opt.keys_map = self.enum_keys_map;
                    return Ok(cloned);
                }
                let opt = cloned
                    .as_any_mut()
                    .downcast_mut::<ConfigOptionEnumsGeneric>()
                    .expect("dynamic_cast<ConfigOptionEnumsGeneric*>(default_value->clone()) is null");
                opt.keys_map = self.enum_keys_map;
                return Ok(cloned);
            }

            return Ok(ConfigOption::clone(&**default_value));
        }
        self.create_empty_option()
    }
}

/// Config.hpp:2444 `typedef std::map<t_config_option_key, ConfigOptionDef> t_optiondef_map;`
#[allow(non_camel_case_types)]
pub type t_optiondef_map = std::collections::BTreeMap<t_config_option_key, ConfigOptionDef>;

/// Config.cpp:335 `static size_t serialization_key_ordinal_last = 0;`
static SERIALIZATION_KEY_ORDINAL_LAST: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

/// Config.hpp:2446-2476: definition of configuration values for the purpose of GUI presentation,
/// editing, value mapping and config file handling. The configuration definition is static: it
/// does not carry the actual configuration values, but it carries the defaults.
#[derive(Clone, Default)]
pub struct ConfigDef {
    pub options: t_optiondef_map,
    /// `std::map<size_t, const ConfigOptionDef*>`: the pointer is the key of `options` here.
    pub by_serialization_key_ordinal: std::collections::BTreeMap<usize, t_config_option_key>,
}

impl ConfigDef {
    /// An empty definition.
    pub fn new() -> Self {
        ConfigDef::default()
    }

    /// Config.hpp:2455 `bool has(const t_config_option_key &opt_key) const`
    pub fn has(&self, opt_key: &str) -> bool {
        self.options.contains_key(opt_key)
    }

    /// Config.hpp:2456-2459 `const ConfigOptionDef* get(const t_config_option_key &opt_key) const`
    pub fn get(&self, opt_key: &str) -> Option<&ConfigOptionDef> {
        self.options.get(opt_key)
    }

    /// Config.hpp:2460-2466 `std::vector<std::string> keys() const`
    pub fn keys(&self) -> Vec<String> {
        let mut out = Vec::with_capacity(self.options.len());
        for (k, _) in &self.options {
            out.push(k.clone());
        }
        out
    }

    /// Config.cpp:332-342 (protected in C++; the derived definition classes call it).
    /// Assignment of the serialization IDs is not thread safe in C++; here the counter is atomic.
    pub fn add(&mut self, opt_key: &str, r#type: ConfigOptionType) -> &mut ConfigOptionDef {
        let ordinal = SERIALIZATION_KEY_ORDINAL_LAST.fetch_add(1, std::sync::atomic::Ordering::SeqCst) + 1;
        let opt = self.options.entry(opt_key.to_string()).or_default();
        opt.opt_key = opt_key.to_string();
        opt.r#type = r#type;
        opt.serialization_key_ordinal = ordinal;
        self.by_serialization_key_ordinal.insert(ordinal, opt_key.to_string());
        opt
    }

    /// Config.cpp:344-349 `ConfigOptionDef* ConfigDef::add_nullable(const t_config_option_key &opt_key, ConfigOptionType type)`
    pub fn add_nullable(&mut self, opt_key: &str, r#type: ConfigOptionType) -> &mut ConfigOptionDef {
        let def = self.add(opt_key, r#type);
        def.nullable = true;
        def
    }

    /// Config.cpp:351-452: iterate through all of the CLI options and write them to a stream.
    pub fn print_cli_help(
        &self,
        out: &mut String,
        show_defaults: bool,
        filter: &dyn Fn(&ConfigOptionDef) -> bool,
    ) -> CfgResult<()> {
        // prepare a function for wrapping text
        let wrap = |text: &str, line_length: usize| -> String {
            let mut words = text.as_bytes().split(|c| is_space_byte(*c)).filter(|w| !w.is_empty());
            let mut wrapped: Vec<u8> = Vec::new();
            if let Some(word) = words.next() {
                wrapped.extend_from_slice(word);
                let mut space_left = line_length.wrapping_sub(word.len());
                for word in words {
                    if space_left < word.len() + 1 {
                        wrapped.push(b'\n');
                        wrapped.extend_from_slice(word);
                        space_left = line_length.wrapping_sub(word.len());
                    } else {
                        wrapped.push(b' ');
                        wrapped.extend_from_slice(word);
                        space_left -= word.len() + 1;
                    }
                }
            }
            bytes_to_string(&wrapped)
        };

        // get the unique categories
        let mut categories: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();
        for (_k, def) in &self.options {
            if filter(def) {
                categories.insert(def.category.clone());
            }
        }

        for category in &categories {
            if !category.is_empty() {
                out.push_str(category);
                out.push_str(":\n");
            } else if categories.len() > 1 {
                out.push_str("Misc options:\n");
            }

            for (opt_first, def) in &self.options {
                if def.category != *category || def.cli == ConfigOptionDef::NOCLI || !filter(def) {
                    continue;
                }

                // get all possible variations: --foo, --foobar, -f...
                let mut cli_args = def.cli_args(opt_first);
                if cli_args.is_empty() {
                    continue;
                }

                for arg in cli_args.iter_mut() {
                    arg.insert_str(0, if arg.len() == 1 { "-" } else { "--" });
                    // BBS: refine the print help format
                    if !def.cli_params.is_empty() {
                        arg.push(' ');
                        arg.push_str(&def.cli_params);
                    }
                }

                // left: command line options
                let cli = cli_args.join(", ");
                out.push(' ');
                out.push_str(&cli);
                // std::left << std::setw(20)
                for _ in cli.len()..20 {
                    out.push(' ');
                }

                // right: option description
                let mut descr = def.tooltip.clone();
                let show_defaults_this = show_defaults || def.opt_key == "config_compatibility";
                if let Some(default_value) = &def.default_value {
                    if show_defaults_this
                        && def.r#type != ConfigOptionType::CoBool
                        && (def.r#type != ConfigOptionType::CoString || !default_value.serialize()?.is_empty())
                    {
                        descr.push_str(" (");
                        if !def.sidetext.is_empty() {
                            descr.push_str(&def.sidetext);
                            descr.push_str(", ");
                        } else if !def.enum_values.is_empty() {
                            descr.push_str(&def.enum_values.join(", "));
                            descr.push_str("; ");
                        }
                        descr.push_str("default: ");
                        descr.push_str(&default_value.serialize()?);
                        descr.push(')');
                    }
                }

                // wrap lines of description
                descr = wrap(&descr, 80);
                let lines: Vec<&str> = descr.split('\n').collect();

                // if command line options are too long, print description in new line
                for (i, line) in lines.iter().enumerate() {
                    if i == 0 && cli.len() > 19 {
                        out.push('\n');
                    }
                    if i > 0 || cli.len() > 19 {
                        out.push_str(&" ".repeat(21));
                    }
                    out.push_str(line);
                    out.push('\n');
                }
            }
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Config.hpp:2478-2668, Config.cpp:454-1659: ConfigOptionResolver, ConfigBase.
// ---------------------------------------------------------------------------

/// Config.hpp:2478-2517: a pure interface to resolving ConfigOptions. This pure interface is
/// useful as a base of ConfigBase, also it may be overriden to combine various config sources.
pub trait ConfigOptionResolver {
    /// Config.hpp:2488: find a ConfigOption instance for a given name.
    fn optptr(&self, opt_key: &str) -> Option<&dyn ConfigOption>;

    /// Config.hpp:2490 `bool has(const t_config_option_key &opt_key) const`
    fn has(&self, opt_key: &str) -> bool {
        self.optptr(opt_key).is_some()
    }

    /// Config.hpp:2492 `const ConfigOption* option(const t_config_option_key &opt_key) const`
    fn option(&self, opt_key: &str) -> Option<&dyn ConfigOption> {
        self.optptr(opt_key)
    }

    /// Config.hpp:2501-2507 `const ConfigOption* option_throw(const t_config_option_key& opt_key) const`
    fn option_throw(&self, opt_key: &str) -> CfgResult<&dyn ConfigOption> {
        match self.optptr(opt_key) {
            None => Err(UnknownOptionException::with_key(opt_key)),
            Some(opt) => Ok(opt),
        }
    }
}

/// The template members of ConfigOptionResolver and ConfigBase (`option<TYPE>()`,
/// `option_throw<TYPE>()`), available on every resolver / config store.
///
/// FIDELITY-NOTE: the C++ compares `opt->type()` with `TYPE::static_type()` and then
/// `static_cast`s. Classes that share a type code (the nullable and non-nullable variant of a
/// vector class, ConfigOptionEnumGeneric and ConfigOptionEnum<T>) pass that test for each other
/// and the C++ reads one through the layout of the other. Here the class has to be the very one
/// asked for: `option_typed` gives `None` and `option_throw_typed` "Conversion to a wrong type"
/// otherwise. Read the values through `ConfigOption::vector_values_any()` / `get_int()` where
/// either class may be stored.
pub trait ConfigOptionResolverTyped: ConfigOptionResolver {
    /// Config.hpp:2494-2499 `template<typename TYPE> const TYPE* option(const t_config_option_key& opt_key) const`
    fn option_typed<T: ConfigOptionStaticType>(&self, opt_key: &str) -> Option<&T> {
        match self.optptr(opt_key) {
            Some(opt) if opt.r#type() == T::static_type() => opt.as_any().downcast_ref::<T>(),
            _ => None,
        }
    }

    /// Config.hpp:2509-2516 `template<typename TYPE> const TYPE* option_throw(const t_config_option_key& opt_key) const`
    fn option_throw_typed<T: ConfigOptionStaticType>(&self, opt_key: &str) -> CfgResult<&T> {
        let opt = self.option_throw(opt_key)?;
        if opt.r#type() != T::static_type() {
            return Err(BadOptionTypeException::with_message("Conversion to a wrong type"));
        }
        opt.as_any()
            .downcast_ref::<T>()
            .ok_or_else(|| BadOptionTypeException::with_message("Conversion to a wrong type"))
    }
}

impl<R: ConfigOptionResolver + ?Sized> ConfigOptionResolverTyped for R {}

/// Config.cpp:40-48: the optional auxiliary fan (P10) has no entry in any profile: its settings
/// fall back to the standard fan (P2) of the same profile level, so presets and 3MFs that
/// predate the split keep driving both fans at the same speed.
const S_AUX_FAN_STANDARD_TO_OPTIONAL_KEYS: [(&str, &str); 4] = [
    ("additional_cooling_fan_speed", "optional_cooling_fan_speed"),
    ("close_additional_fan_first_x_layers", "close_optional_fan_first_x_layers"),
    ("first_x_layer_fan_speed", "first_x_layer_optional_fan_speed"),
    ("additional_fan_full_speed_layer", "optional_fan_full_speed_layer"),
];

/// Preset.hpp:38-57 `BBL_JSON_KEY_*`, the ones Config.cpp reads and writes.
const BBL_JSON_KEY_VERSION: &str = "version";
const BBL_JSON_KEY_IS_CUSTOM: &str = "is_custom_defined";
const BBL_JSON_KEY_URL: &str = "url";
const BBL_JSON_KEY_NAME: &str = "name";
const BBL_JSON_KEY_DESCRIPTION: &str = "description";
const BBL_JSON_KEY_TYPE: &str = "type";
const BBL_JSON_KEY_FROM: &str = "from";
const BBL_JSON_KEY_SETTING_ID: &str = "setting_id";
const BBL_JSON_KEY_FILAMENT_ID: &str = "filament_id";
const BBL_JSON_KEY_INHERITS: &str = "inherits";
const BBL_JSON_KEY_INCLUDES: &str = "include";
const BBL_JSON_KEY_INSTANTIATION: &str = "instantiation";

/// libslic3r.h:5 `#define SLIC3R_APP_FULL_NAME "Bambu Studio"`
const SLIC3R_APP_FULL_NAME: &str = "Bambu Studio";

/// PrintConfig.hpp:405-409 `ReduceInfillRetractionMode::rirDisabled`
const RIR_DISABLED: i32 = 0;
/// PrintConfig.hpp:174-176 `SupportMaterialStyle::smsTreeHybrid`
const SMS_TREE_HYBRID: i32 = 5;

/// Config.cpp:536-563: best-effort, never-throwing diagnostic dump of an option's raw values,
/// used to record the "crime scene" when serialization fails (e.g. a NaN in a non-nullable
/// float vector). It bypasses serialize() on purpose so it works even for the very values that
/// make serialize() throw.
fn describe_option_values(opt: Option<&dyn ConfigOption>) -> String {
    let opt = match opt {
        None => return "<null option>".to_string(),
        Some(opt) => opt,
    };
    let mut ss = String::new();
    ss.push_str("type=");
    ss.push_str(&ostream_int(opt.r#type() as i32));
    ss.push_str(" nullable=");
    ss.push_str(if opt.nullable() { "1" } else { "0" });
    // The failing options are float / float-or-percent vectors; dump their doubles directly and
    // flag which entries are non-finite so the offending index is obvious in the log.
    if let Some(fv) = opt.vector_values_any().and_then(|a| a.downcast_ref::<Vec<f64>>()) {
        ss.push_str(" size=");
        ss.push_str(&fv.len().to_string());
        ss.push_str(" values=[");
        for (i, &v) in fv.iter().enumerate() {
            if i != 0 {
                ss.push(',');
            }
            if v.is_nan() {
                ss.push('#');
                ss.push_str(&i.to_string());
                ss.push_str(":NaN");
            } else if !v.is_finite() {
                ss.push('#');
                ss.push_str(&i.to_string());
                ss.push_str(":Inf");
            } else {
                ss.push_str(&ostream_double(v));
            }
        }
        ss.push(']');
    } else {
        // For any other type, serialize() is what threw; fall back to a size hint if it is a vector.
        if let Some(vb) = opt.as_vector_base() {
            ss.push_str(" size=");
            ss.push_str(&vb.size().to_string());
        }
    }
    ss
}

/// C `atoi`: white space, a sign, decimal digits.
fn c_atoi(s: &[u8]) -> i32 {
    let mut i = 0usize;
    while i < s.len() && is_space_byte(s[i]) {
        i += 1;
    }
    let mut neg = false;
    if i < s.len() && (s[i] == b'+' || s[i] == b'-') {
        neg = s[i] == b'-';
        i += 1;
    }
    let mut v: i64 = 0;
    while i < s.len() && s[i].is_ascii_digit() {
        v = v.wrapping_mul(10).wrapping_add((s[i] - b'0') as i64);
        i += 1;
    }
    (if neg { v.wrapping_neg() } else { v }) as i32
}

/// `boost::iequals` for ASCII keys.
fn iequals(a: &str, b: &str) -> bool {
    a.eq_ignore_ascii_case(b)
}

/// What `ifs >> j` of nlohmann::json 3.10.4 (src/nlohmann at the pin) does with the bytes of a
/// file: `parser(input_adapter(ifs)).parse(false, j)`, i.e. detail/input/lexer.hpp and
/// detail/input/parser.hpp with no callback, exceptions on, comments not skipped and the strict
/// flag off (what follows the first value is not read). The DOM it builds is held as a
/// serde_json::Value (an object is sorted by key as the std::map of nlohmann::json is when it is
/// iterated; a later duplicate key replaces the earlier value, as `operator[]` does).
pub(crate) mod nlohmann_json {
    /// lexer_base::token_type (lexer.hpp:34-52)
    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    enum TokenType {
        Uninitialized,
        LiteralTrue,
        LiteralFalse,
        LiteralNull,
        ValueString,
        ValueUnsigned,
        ValueInteger,
        ValueFloat,
        BeginArray,
        BeginObject,
        EndArray,
        EndObject,
        NameSeparator,
        ValueSeparator,
        ParseError,
        EndOfInput,
        LiteralOrValue,
    }

    /// lexer_base::token_type_name (lexer.hpp:54-94)
    fn token_type_name(t: TokenType) -> &'static str {
        use TokenType::*;
        match t {
            Uninitialized => "<uninitialized>",
            LiteralTrue => "true literal",
            LiteralFalse => "false literal",
            LiteralNull => "null literal",
            ValueString => "string literal",
            ValueUnsigned | ValueInteger | ValueFloat => "number literal",
            BeginArray => "'['",
            BeginObject => "'{'",
            EndArray => "']'",
            EndObject => "'}'",
            NameSeparator => "':'",
            ValueSeparator => "','",
            ParseError => "<parse error>",
            EndOfInput => "end of input",
            LiteralOrValue => "'[', '{', or a literal",
        }
    }

    /// position_t (position_t.hpp)
    #[derive(Clone, Copy, Default)]
    struct Position {
        chars_read_total: usize,
        chars_read_current_line: usize,
        lines_read: usize,
    }

    /// `std::char_traits<char>::eof()`
    const EOF: i32 = -1;

    /// The exceptions `parse` throws: parse_error (Config.cpp catches it by its class) and the
    /// out_of_range of a number that overflows (caught as a std::exception).
    pub(crate) enum Error {
        Parse(String),
        Other(String),
    }

    /// lexer<BasicJsonType, input_stream_adapter> (lexer.hpp:102-1622)
    struct Lexer<'a> {
        ia: &'a [u8],
        ia_pos: usize,
        current: i32,
        next_unget: bool,
        position: Position,
        token_string: Vec<u8>,
        token_buffer: Vec<u8>,
        error_message: &'static str,
        value_integer: i64,
        value_unsigned: u64,
        value_float: f64,
    }

    impl<'a> Lexer<'a> {
        fn new(ia: &'a [u8]) -> Self {
            Lexer {
                ia,
                ia_pos: 0,
                current: EOF,
                next_unget: false,
                position: Position::default(),
                token_string: Vec::new(),
                token_buffer: Vec::new(),
                error_message: "",
                value_integer: 0,
                value_unsigned: 0,
                value_float: 0.0,
            }
        }

        /// input_stream_adapter::get_character: `sb->sbumpc()`, a byte as 0..255.
        fn get_character(&mut self) -> i32 {
            if self.ia_pos < self.ia.len() {
                self.ia_pos += 1;
                self.ia[self.ia_pos - 1] as i32
            } else {
                EOF
            }
        }

        /// lexer.hpp:160-190
        fn get_codepoint(&mut self) -> i32 {
            let mut codepoint: i32 = 0;
            for factor in [12u32, 8, 4, 0] {
                self.get();
                let c = self.current;
                if (b'0' as i32..=b'9' as i32).contains(&c) {
                    codepoint += ((c as u32 - 0x30) << factor) as i32;
                } else if (b'A' as i32..=b'F' as i32).contains(&c) {
                    codepoint += ((c as u32 - 0x37) << factor) as i32;
                } else if (b'a' as i32..=b'f' as i32).contains(&c) {
                    codepoint += ((c as u32 - 0x57) << factor) as i32;
                } else {
                    return -1;
                }
            }
            codepoint
        }

        /// lexer.hpp:207-228
        fn next_byte_in_range(&mut self, ranges: &[i32]) -> bool {
            self.add(self.current);
            let mut k = 0;
            while k < ranges.len() {
                self.get();
                if ranges[k] <= self.current && self.current <= ranges[k + 1] {
                    self.add(self.current);
                } else {
                    self.error_message = "invalid string: ill-formed UTF-8 byte";
                    return false;
                }
                k += 2;
            }
            true
        }

        /// lexer.hpp:245-829 `token_type scan_string()`
        fn scan_string(&mut self) -> TokenType {
            const CONTROL: [&str; 32] = [
                "invalid string: control character U+0000 (NUL) must be escaped to \\u0000",
                "invalid string: control character U+0001 (SOH) must be escaped to \\u0001",
                "invalid string: control character U+0002 (STX) must be escaped to \\u0002",
                "invalid string: control character U+0003 (ETX) must be escaped to \\u0003",
                "invalid string: control character U+0004 (EOT) must be escaped to \\u0004",
                "invalid string: control character U+0005 (ENQ) must be escaped to \\u0005",
                "invalid string: control character U+0006 (ACK) must be escaped to \\u0006",
                "invalid string: control character U+0007 (BEL) must be escaped to \\u0007",
                "invalid string: control character U+0008 (BS) must be escaped to \\u0008 or \\b",
                "invalid string: control character U+0009 (HT) must be escaped to \\u0009 or \\t",
                "invalid string: control character U+000A (LF) must be escaped to \\u000A or \\n",
                "invalid string: control character U+000B (VT) must be escaped to \\u000B",
                "invalid string: control character U+000C (FF) must be escaped to \\u000C or \\f",
                "invalid string: control character U+000D (CR) must be escaped to \\u000D or \\r",
                "invalid string: control character U+000E (SO) must be escaped to \\u000E",
                "invalid string: control character U+000F (SI) must be escaped to \\u000F",
                "invalid string: control character U+0010 (DLE) must be escaped to \\u0010",
                "invalid string: control character U+0011 (DC1) must be escaped to \\u0011",
                "invalid string: control character U+0012 (DC2) must be escaped to \\u0012",
                "invalid string: control character U+0013 (DC3) must be escaped to \\u0013",
                "invalid string: control character U+0014 (DC4) must be escaped to \\u0014",
                "invalid string: control character U+0015 (NAK) must be escaped to \\u0015",
                "invalid string: control character U+0016 (SYN) must be escaped to \\u0016",
                "invalid string: control character U+0017 (ETB) must be escaped to \\u0017",
                "invalid string: control character U+0018 (CAN) must be escaped to \\u0018",
                "invalid string: control character U+0019 (EM) must be escaped to \\u0019",
                "invalid string: control character U+001A (SUB) must be escaped to \\u001A",
                "invalid string: control character U+001B (ESC) must be escaped to \\u001B",
                "invalid string: control character U+001C (FS) must be escaped to \\u001C",
                "invalid string: control character U+001D (GS) must be escaped to \\u001D",
                "invalid string: control character U+001E (RS) must be escaped to \\u001E",
                "invalid string: control character U+001F (US) must be escaped to \\u001F",
            ];
            self.reset();
            loop {
                let c = self.get();
                match c {
                    EOF => {
                        self.error_message = "invalid string: missing closing quote";
                        return TokenType::ParseError;
                    }
                    0x22 => return TokenType::ValueString,
                    0x5C => {
                        match self.get() {
                            0x22 => self.add(0x22),
                            0x5C => self.add(0x5C),
                            0x2F => self.add(0x2F),
                            0x62 => self.add(0x08),
                            0x66 => self.add(0x0C),
                            0x6E => self.add(0x0A),
                            0x72 => self.add(0x0D),
                            0x74 => self.add(0x09),
                            0x75 => {
                                let codepoint1 = self.get_codepoint();
                                let mut codepoint = codepoint1;
                                if codepoint1 == -1 {
                                    self.error_message = "invalid string: '\\u' must be followed by 4 hex digits";
                                    return TokenType::ParseError;
                                }
                                if (0xD800..=0xDBFF).contains(&codepoint1) {
                                    if self.get() == 0x5C && self.get() == 0x75 {
                                        let codepoint2 = self.get_codepoint();
                                        if codepoint2 == -1 {
                                            self.error_message = "invalid string: '\\u' must be followed by 4 hex digits";
                                            return TokenType::ParseError;
                                        }
                                        if (0xDC00..=0xDFFF).contains(&codepoint2) {
                                            codepoint = ((codepoint1 as u32) << 10)
                                                .wrapping_add(codepoint2 as u32)
                                                .wrapping_sub(0x35FDC00) as i32;
                                        } else {
                                            self.error_message =
                                                "invalid string: surrogate U+D800..U+DBFF must be followed by U+DC00..U+DFFF";
                                            return TokenType::ParseError;
                                        }
                                    } else {
                                        self.error_message =
                                            "invalid string: surrogate U+D800..U+DBFF must be followed by U+DC00..U+DFFF";
                                        return TokenType::ParseError;
                                    }
                                } else if (0xDC00..=0xDFFF).contains(&codepoint1) {
                                    self.error_message = "invalid string: surrogate U+DC00..U+DFFF must follow U+D800..U+DBFF";
                                    return TokenType::ParseError;
                                }
                                let cp = codepoint as u32;
                                if cp < 0x80 {
                                    self.add(cp as i32);
                                } else if cp <= 0x7FF {
                                    self.add((0xC0 | (cp >> 6)) as i32);
                                    self.add((0x80 | (cp & 0x3F)) as i32);
                                } else if cp <= 0xFFFF {
                                    self.add((0xE0 | (cp >> 12)) as i32);
                                    self.add((0x80 | ((cp >> 6) & 0x3F)) as i32);
                                    self.add((0x80 | (cp & 0x3F)) as i32);
                                } else {
                                    self.add((0xF0 | (cp >> 18)) as i32);
                                    self.add((0x80 | ((cp >> 12) & 0x3F)) as i32);
                                    self.add((0x80 | ((cp >> 6) & 0x3F)) as i32);
                                    self.add((0x80 | (cp & 0x3F)) as i32);
                                }
                            }
                            _ => {
                                self.error_message = "invalid string: forbidden character after backslash";
                                return TokenType::ParseError;
                            }
                        }
                    }
                    0x00..=0x1F => {
                        self.error_message = CONTROL[c as usize];
                        return TokenType::ParseError;
                    }
                    0x20..=0x7F => self.add(c),
                    // U+0080..U+07FF: bytes C2..DF 80..BF
                    0xC2..=0xDF => {
                        if !self.next_byte_in_range(&[0x80, 0xBF]) {
                            return TokenType::ParseError;
                        }
                    }
                    // U+0800..U+0FFF: bytes E0 A0..BF 80..BF
                    0xE0 => {
                        if !self.next_byte_in_range(&[0xA0, 0xBF, 0x80, 0xBF]) {
                            return TokenType::ParseError;
                        }
                    }
                    // U+1000..U+CFFF: bytes E1..EC 80..BF 80..BF
                    // U+E000..U+FFFF: bytes EE..EF 80..BF 80..BF
                    0xE1..=0xEC | 0xEE | 0xEF => {
                        if !self.next_byte_in_range(&[0x80, 0xBF, 0x80, 0xBF]) {
                            return TokenType::ParseError;
                        }
                    }
                    // U+D000..U+D7FF: bytes ED 80..9F 80..BF
                    0xED => {
                        if !self.next_byte_in_range(&[0x80, 0x9F, 0x80, 0xBF]) {
                            return TokenType::ParseError;
                        }
                    }
                    // U+10000..U+3FFFF F0 90..BF 80..BF 80..BF
                    0xF0 => {
                        if !self.next_byte_in_range(&[0x90, 0xBF, 0x80, 0xBF, 0x80, 0xBF]) {
                            return TokenType::ParseError;
                        }
                    }
                    // U+40000..U+FFFFF F1..F3 80..BF 80..BF 80..BF
                    0xF1..=0xF3 => {
                        if !self.next_byte_in_range(&[0x80, 0xBF, 0x80, 0xBF, 0x80, 0xBF]) {
                            return TokenType::ParseError;
                        }
                    }
                    // U+100000..U+10FFFF F4 80..8F 80..BF 80..BF
                    0xF4 => {
                        if !self.next_byte_in_range(&[0x80, 0x8F, 0x80, 0xBF, 0x80, 0xBF]) {
                            return TokenType::ParseError;
                        }
                    }
                    // remaining bytes (80..C1 and F5..FF) are ill-formed
                    _ => {
                        self.error_message = "invalid string: ill-formed UTF-8 byte";
                        return TokenType::ParseError;
                    }
                }
            }
        }

        /// lexer.hpp:968-1311 `token_type scan_number()`: the state machine of RFC 8259 Sect. 6,
        /// one arm per label of the C++.
        fn scan_number(&mut self) -> TokenType {
            #[derive(Clone, Copy)]
            enum S {
                Minus,
                Zero,
                Any1,
                Decimal1,
                Decimal2,
                Exponent,
                Sign,
                Any2,
                Done,
            }
            self.reset();
            let mut number_type = TokenType::ValueUnsigned;
            let is_digit = |c: i32| (b'0' as i32..=b'9' as i32).contains(&c);
            let is_digit19 = |c: i32| (b'1' as i32..=b'9' as i32).contains(&c);
            let mut state = match self.current {
                c if c == b'-' as i32 => {
                    self.add(c);
                    S::Minus
                }
                c if c == b'0' as i32 => {
                    self.add(c);
                    S::Zero
                }
                c => {
                    // '1'..'9' (scan() only calls this for '-' and digits)
                    self.add(c);
                    S::Any1
                }
            };
            loop {
                state = match state {
                    S::Minus => {
                        number_type = TokenType::ValueInteger;
                        let c = self.get();
                        if c == b'0' as i32 {
                            self.add(c);
                            S::Zero
                        } else if is_digit19(c) {
                            self.add(c);
                            S::Any1
                        } else {
                            self.error_message = "invalid number; expected digit after '-'";
                            return TokenType::ParseError;
                        }
                    }
                    S::Zero => {
                        let c = self.get();
                        if c == b'.' as i32 {
                            // add(decimal_point_char): '.' in the C locale CNumericLocalesSetter sets
                            self.add(b'.' as i32);
                            S::Decimal1
                        } else if c == b'e' as i32 || c == b'E' as i32 {
                            self.add(c);
                            S::Exponent
                        } else {
                            S::Done
                        }
                    }
                    S::Any1 => {
                        let c = self.get();
                        if is_digit(c) {
                            self.add(c);
                            S::Any1
                        } else if c == b'.' as i32 {
                            self.add(b'.' as i32);
                            S::Decimal1
                        } else if c == b'e' as i32 || c == b'E' as i32 {
                            self.add(c);
                            S::Exponent
                        } else {
                            S::Done
                        }
                    }
                    S::Decimal1 => {
                        number_type = TokenType::ValueFloat;
                        let c = self.get();
                        if is_digit(c) {
                            self.add(c);
                            S::Decimal2
                        } else {
                            self.error_message = "invalid number; expected digit after '.'";
                            return TokenType::ParseError;
                        }
                    }
                    S::Decimal2 => {
                        let c = self.get();
                        if is_digit(c) {
                            self.add(c);
                            S::Decimal2
                        } else if c == b'e' as i32 || c == b'E' as i32 {
                            self.add(c);
                            S::Exponent
                        } else {
                            S::Done
                        }
                    }
                    S::Exponent => {
                        number_type = TokenType::ValueFloat;
                        let c = self.get();
                        if c == b'+' as i32 || c == b'-' as i32 {
                            self.add(c);
                            S::Sign
                        } else if is_digit(c) {
                            self.add(c);
                            S::Any2
                        } else {
                            self.error_message = "invalid number; expected '+', '-', or digit after exponent";
                            return TokenType::ParseError;
                        }
                    }
                    S::Sign => {
                        let c = self.get();
                        if is_digit(c) {
                            self.add(c);
                            S::Any2
                        } else {
                            self.error_message = "invalid number; expected digit after exponent sign";
                            return TokenType::ParseError;
                        }
                    }
                    S::Any2 => {
                        let c = self.get();
                        if is_digit(c) {
                            self.add(c);
                            S::Any2
                        } else {
                            S::Done
                        }
                    }
                    S::Done => break,
                };
            }
            // scan_number_done:
            self.unget();
            // The buffer holds ASCII digits, '-', '.', 'e', 'E', '+' only.
            let text = std::str::from_utf8(&self.token_buffer).unwrap_or("");
            if number_type == TokenType::ValueUnsigned {
                // strtoull(..., 10) with errno == 0
                if let Ok(x) = text.parse::<u64>() {
                    self.value_unsigned = x;
                    return TokenType::ValueUnsigned;
                }
            } else if number_type == TokenType::ValueInteger {
                // strtoll(..., 10) with errno == 0
                if let Ok(x) = text.parse::<i64>() {
                    self.value_integer = x;
                    return TokenType::ValueInteger;
                }
            }
            // strtod: correctly rounded, as the gdtoa strtod of the reference libc.
            self.value_float = text.parse::<f64>().unwrap_or(0.0);
            TokenType::ValueFloat
        }

        /// lexer.hpp:1318-1331 `token_type scan_literal(...)`
        fn scan_literal(&mut self, literal_text: &[u8], return_type: TokenType) -> TokenType {
            for &lc in &literal_text[1..] {
                // to_char_type(get()): EOF becomes the char 0xFF
                if self.get() as u8 != lc {
                    self.error_message = "invalid literal";
                    return TokenType::ParseError;
                }
            }
            return_type
        }

        /// lexer.hpp:1338-1343
        fn reset(&mut self) {
            self.token_buffer.clear();
            self.token_string.clear();
            self.token_string.push(self.current as u8);
        }

        /// lexer.hpp:1355-1381 `char_int_type get()`
        fn get(&mut self) -> i32 {
            self.position.chars_read_total += 1;
            self.position.chars_read_current_line += 1;
            if self.next_unget {
                // just reset the next_unget variable and work with current
                self.next_unget = false;
            } else {
                self.current = self.get_character();
            }
            if self.current != EOF {
                self.token_string.push(self.current as u8);
            }
            if self.current == b'\n' as i32 {
                self.position.lines_read += 1;
                self.position.chars_read_current_line = 0;
            }
            self.current
        }

        /// lexer.hpp:1392-1415 `void unget()`
        fn unget(&mut self) {
            self.next_unget = true;
            self.position.chars_read_total -= 1;
            // in case we "unget" a newline, we have to also decrement the lines_read
            if self.position.chars_read_current_line == 0 {
                if self.position.lines_read > 0 {
                    self.position.lines_read -= 1;
                }
            } else {
                self.position.chars_read_current_line -= 1;
            }
            if self.current != EOF {
                self.token_string.pop();
            }
        }

        /// lexer.hpp:1418-1421
        fn add(&mut self, c: i32) {
            self.token_buffer.push(c as u8);
        }

        /// lexer.hpp:1458-1479: escape control characters as <U+XXXX>.
        fn get_token_string(&self) -> String {
            let mut result: Vec<u8> = Vec::new();
            for &c in &self.token_string {
                if c <= 0x1F {
                    result.extend_from_slice(format!("<U+{:04X}>", c).as_bytes());
                } else {
                    result.push(c);
                }
            }
            String::from_utf8_lossy(&result).into_owned()
        }

        /// lexer.hpp:1497-1506 `bool skip_bom()`
        fn skip_bom(&mut self) -> bool {
            if self.get() == 0xEF {
                // check if we completely parse the BOM
                return self.get() == 0xBB && self.get() == 0xBF;
            }
            // the first character is not the beginning of the BOM; unget it to process is later
            self.unget();
            true
        }

        /// lexer.hpp:1508-1515
        fn skip_whitespace(&mut self) {
            loop {
                self.get();
                let c = self.current;
                if !(c == b' ' as i32 || c == b'\t' as i32 || c == b'\n' as i32 || c == b'\r' as i32) {
                    break;
                }
            }
        }

        /// lexer.hpp:1517-1594 `token_type scan()` (comments are not skipped)
        fn scan(&mut self) -> TokenType {
            // initially, skip the BOM
            if self.position.chars_read_total == 0 && !self.skip_bom() {
                self.error_message = "invalid BOM; must be 0xEF 0xBB 0xBF if given";
                return TokenType::ParseError;
            }
            // read next character and ignore whitespace
            self.skip_whitespace();
            match self.current {
                c if c == b'[' as i32 => TokenType::BeginArray,
                c if c == b']' as i32 => TokenType::EndArray,
                c if c == b'{' as i32 => TokenType::BeginObject,
                c if c == b'}' as i32 => TokenType::EndObject,
                c if c == b':' as i32 => TokenType::NameSeparator,
                c if c == b',' as i32 => TokenType::ValueSeparator,
                c if c == b't' as i32 => self.scan_literal(b"true", TokenType::LiteralTrue),
                c if c == b'f' as i32 => self.scan_literal(b"false", TokenType::LiteralFalse),
                c if c == b'n' as i32 => self.scan_literal(b"null", TokenType::LiteralNull),
                c if c == b'"' as i32 => self.scan_string(),
                c if c == b'-' as i32 || (b'0' as i32..=b'9' as i32).contains(&c) => self.scan_number(),
                // end of input (the null byte is needed when parsing from string literals)
                0 | EOF => TokenType::EndOfInput,
                _ => {
                    self.error_message = "invalid literal";
                    TokenType::ParseError
                }
            }
        }
    }

    /// parser<BasicJsonType, InputAdapterType> with a json_sax_dom_parser (parser.hpp).
    struct Parser<'a> {
        last_token: TokenType,
        m_lexer: Lexer<'a>,
    }

    /// A value under construction (json_sax_dom_parser::ref_stack).
    enum Building {
        Array(Vec<serde_json::Value>),
        Object(serde_json::Map<String, serde_json::Value>, String),
    }

    impl<'a> Parser<'a> {
        /// parser.hpp:63-72: read first token
        fn new(ia: &'a [u8]) -> Self {
            let mut p = Parser { last_token: TokenType::Uninitialized, m_lexer: Lexer::new(ia) };
            p.get_token();
            p
        }

        /// parser.hpp:480-484
        fn get_token(&mut self) -> TokenType {
            self.last_token = self.m_lexer.scan();
            self.last_token
        }

        /// parser.hpp:486-512 `std::string exception_message(...)`
        fn exception_message(&self, expected: TokenType, context: &str) -> String {
            let mut error_msg = String::from("syntax error ");
            if !context.is_empty() {
                error_msg.push_str(&format!("while parsing {} ", context));
            }
            error_msg.push_str("- ");
            if self.last_token == TokenType::ParseError {
                error_msg.push_str(&format!(
                    "{}; last read: '{}'",
                    self.m_lexer.error_message,
                    self.m_lexer.get_token_string()
                ));
            } else {
                error_msg.push_str(&format!("unexpected {}", token_type_name(self.last_token)));
            }
            if expected != TokenType::Uninitialized {
                error_msg.push_str(&format!("; expected {}", token_type_name(expected)));
            }
            error_msg
        }

        /// parse_error::create(101, position, what) (exceptions.hpp:196-202, 228-232)
        fn parse_error(&self, expected: TokenType, context: &str) -> Error {
            let pos = self.m_lexer.position;
            Error::Parse(format!(
                "[json.exception.parse_error.101] parse error at line {}, column {}: {}",
                pos.lines_read + 1,
                pos.chars_read_current_line,
                self.exception_message(expected, context)
            ))
        }

        /// json_sax_dom_parser::handle_value: the value goes into the array or under the key
        /// on top of the stack, or becomes the result.
        fn handle_value(stack: &mut Vec<Building>, result: &mut Option<serde_json::Value>, v: serde_json::Value) {
            match stack.last_mut() {
                None => *result = Some(v),
                Some(Building::Array(a)) => a.push(v),
                Some(Building::Object(m, key)) => {
                    m.insert(std::mem::take(key), v);
                }
            }
        }

        /// parser.hpp:153-478 `bool sax_parse_internal(SAX* sax)`, the states vector being the
        /// stack of the values under construction (true = array, false = object).
        fn sax_parse_internal(&mut self) -> Result<serde_json::Value, Error> {
            let mut stack: Vec<Building> = Vec::new();
            let mut result: Option<serde_json::Value> = None;
            // value to avoid a goto (see comment where set to true)
            let mut skip_to_state_evaluation = false;

            loop {
                if !skip_to_state_evaluation {
                    // invariant: get_token() was called before each iteration
                    match self.last_token {
                        TokenType::BeginObject => {
                            // closing } -> we are done
                            if self.get_token() == TokenType::EndObject {
                                Self::handle_value(&mut stack, &mut result, serde_json::Value::Object(Default::default()));
                            } else {
                                // parse key
                                if self.last_token != TokenType::ValueString {
                                    return Err(self.parse_error(TokenType::ValueString, "object key"));
                                }
                                let key = String::from_utf8_lossy(&self.m_lexer.token_buffer).into_owned();
                                // parse separator (:)
                                if self.get_token() != TokenType::NameSeparator {
                                    return Err(self.parse_error(TokenType::NameSeparator, "object separator"));
                                }
                                // remember we are now inside an object
                                stack.push(Building::Object(Default::default(), key));
                                // parse values
                                self.get_token();
                                continue;
                            }
                        }
                        TokenType::BeginArray => {
                            // closing ] -> we are done
                            if self.get_token() == TokenType::EndArray {
                                Self::handle_value(&mut stack, &mut result, serde_json::Value::Array(Vec::new()));
                            } else {
                                // remember we are now inside an array
                                stack.push(Building::Array(Vec::new()));
                                // parse values (no need to call get_token)
                                continue;
                            }
                        }
                        TokenType::ValueFloat => {
                            let res = self.m_lexer.value_float;
                            if !res.is_finite() {
                                return Err(Error::Other(format!(
                                    "[json.exception.out_of_range.406] number overflow parsing '{}'",
                                    self.m_lexer.get_token_string()
                                )));
                            }
                            let n = serde_json::Number::from_f64(res).expect("a finite double");
                            Self::handle_value(&mut stack, &mut result, serde_json::Value::Number(n));
                        }
                        TokenType::LiteralFalse => Self::handle_value(&mut stack, &mut result, serde_json::Value::Bool(false)),
                        TokenType::LiteralNull => Self::handle_value(&mut stack, &mut result, serde_json::Value::Null),
                        TokenType::LiteralTrue => Self::handle_value(&mut stack, &mut result, serde_json::Value::Bool(true)),
                        TokenType::ValueInteger => {
                            let v = serde_json::Value::Number(self.m_lexer.value_integer.into());
                            Self::handle_value(&mut stack, &mut result, v);
                        }
                        TokenType::ValueString => {
                            let v = serde_json::Value::String(String::from_utf8_lossy(&self.m_lexer.token_buffer).into_owned());
                            Self::handle_value(&mut stack, &mut result, v);
                        }
                        TokenType::ValueUnsigned => {
                            let v = serde_json::Value::Number(self.m_lexer.value_unsigned.into());
                            Self::handle_value(&mut stack, &mut result, v);
                        }
                        TokenType::ParseError => {
                            // using "uninitialized" to avoid "expected" message
                            return Err(self.parse_error(TokenType::Uninitialized, "value"));
                        }
                        _ => {
                            // the last token was unexpected
                            return Err(self.parse_error(TokenType::LiteralOrValue, "value"));
                        }
                    }
                } else {
                    skip_to_state_evaluation = false;
                }

                // we reached this line after we successfully parsed a value
                let is_array = match stack.last() {
                    // empty stack: we reached the end of the hierarchy: done
                    None => return Ok(result.expect("a parsed value")),
                    Some(Building::Array(_)) => true,
                    Some(Building::Object(..)) => false,
                };
                if is_array {
                    // comma -> next value
                    if self.get_token() == TokenType::ValueSeparator {
                        // parse a new value
                        self.get_token();
                        continue;
                    }
                    // closing ]
                    if self.last_token == TokenType::EndArray {
                        let done = match stack.pop() {
                            Some(Building::Array(a)) => serde_json::Value::Array(a),
                            _ => unreachable!(),
                        };
                        Self::handle_value(&mut stack, &mut result, done);
                        // We are done with this array. Before we can parse a new value, we need
                        // to evaluate the new state first.
                        skip_to_state_evaluation = true;
                        continue;
                    }
                    return Err(self.parse_error(TokenType::EndArray, "array"));
                }

                // states.back() is false -> object
                // comma -> next value
                if self.get_token() == TokenType::ValueSeparator {
                    // parse key
                    if self.get_token() != TokenType::ValueString {
                        return Err(self.parse_error(TokenType::ValueString, "object key"));
                    }
                    let key = String::from_utf8_lossy(&self.m_lexer.token_buffer).into_owned();
                    if let Some(Building::Object(_, k)) = stack.last_mut() {
                        *k = key;
                    }
                    // parse separator (:)
                    if self.get_token() != TokenType::NameSeparator {
                        return Err(self.parse_error(TokenType::NameSeparator, "object separator"));
                    }
                    // parse values
                    self.get_token();
                    continue;
                }
                // closing }
                if self.last_token == TokenType::EndObject {
                    let done = match stack.pop() {
                        Some(Building::Object(m, _)) => serde_json::Value::Object(m),
                        _ => unreachable!(),
                    };
                    Self::handle_value(&mut stack, &mut result, done);
                    // We are done with this object. Before we can parse a new value, we need to
                    // evaluate the new state first.
                    skip_to_state_evaluation = true;
                    continue;
                }
                return Err(self.parse_error(TokenType::EndObject, "object"));
            }
        }
    }

    /// `ifs >> j` (json.hpp:7131-7135): `parser(detail::input_adapter(i)).parse(false, j)`.
    pub(crate) fn parse(data: &[u8]) -> Result<serde_json::Value, Error> {
        let mut p = Parser::new(data);
        p.sax_parse_internal()
    }

    /// `json::parse(std::string)` (json.hpp `basic_json::parse(InputType&&, cb = nullptr,
    /// allow_exceptions = true, ignore_comments = false)`: `parser(...).parse(true, result)`): the strict
    /// flag on, so the input must be read to its end after the first value (parser.hpp:85-99: `if
    /// (strict && (get_token() != token_type::end_of_input))` a parse_error 101 "value").
    pub(crate) fn parse_strict(data: &[u8]) -> Result<serde_json::Value, Error> {
        let mut p = Parser::new(data);
        let result = p.sax_parse_internal()?;
        // parser.hpp:92-99
        if p.get_token() != TokenType::EndOfInput {
            return Err(p.parse_error(TokenType::EndOfInput, "value"));
        }
        Ok(result)
    }
}

/// `nlohmann::json::type_name()`.
fn json_type_name(v: &serde_json::Value) -> &'static str {
    match v {
        serde_json::Value::Null => "null",
        serde_json::Value::Bool(_) => "boolean",
        serde_json::Value::Number(_) => "number",
        serde_json::Value::String(_) => "string",
        serde_json::Value::Array(_) => "array",
        serde_json::Value::Object(_) => "object",
    }
}

/// `ifs >> j` of nlohmann::json over the bytes of a file, for the readers of Preset.cpp
/// (get_version_from_json, get_values_from_json): `Err((true, what))` for a parse_error,
/// `Err((false, what))` for the other exceptions.
pub(crate) fn nlohmann_json_parse(data: &[u8]) -> Result<serde_json::Value, (bool, String)> {
    match nlohmann_json::parse(data) {
        Ok(v) => Ok(v),
        Err(nlohmann_json::Error::Parse(what)) => Err((true, what)),
        Err(nlohmann_json::Error::Other(what)) => Err((false, what)),
    }
}

/// `nlohmann::json::parse(std::string)` of nlohmann::json 3.10.4 (strict: nothing may follow the
/// value), with the lexer's correctly rounded `strtod` for floats: `Err(what)` where it throws.
/// Desensitize.cpp's json_log and the assembly JSON readers call it.
pub(crate) fn nlohmann_json_parse_strict(data: &[u8]) -> Result<serde_json::Value, String> {
    match nlohmann_json::parse_strict(data) {
        Ok(v) => Ok(v),
        Err(nlohmann_json::Error::Parse(what)) | Err(nlohmann_json::Error::Other(what)) => Err(what),
    }
}

/// `nlohmann::json::type_name()`, for Preset.cpp.
pub(crate) fn nlohmann_type_name(v: &serde_json::Value) -> &'static str {
    json_type_name(v)
}

/// C `atoi` of a text, for Preset.cpp.
pub(crate) fn c_atoi_str(s: &str) -> i32 {
    c_atoi(s.as_bytes())
}

/// The implicit conversion of a `nlohmann::json` to `std::string`: a type_error 302 for
/// anything but a string. The error is a `std::exception`, not a ConfigurationError; the one
/// caller turns it into the `reason` text.
fn json_to_string(v: &serde_json::Value) -> Result<String, String> {
    match v {
        serde_json::Value::String(s) => Ok(s.clone()),
        other => Err(format!(
            "[json.exception.type_error.302] type must be string, but is {}",
            json_type_name(other)
        )),
    }
}

/// Config.cpp:900-942 `parse_str_arr`: BBS: we only support 2 depth array.
fn parse_str_arr(
    value: &[serde_json::Value],
    single_sep: char,
    array_sep: char,
    escape_string_style: bool,
    value_str: &mut String,
) -> bool {
    // must have consistent type name
    let mut consistent_type = String::new();
    for iter in value {
        if consistent_type.is_empty() {
            consistent_type = json_type_name(iter).to_string();
        } else if consistent_type != json_type_name(iter) {
            return false;
        }
    }

    let mut first = true;
    for iter in value {
        if let serde_json::Value::Array(inner) = iter {
            if !first {
                value_str.push(array_sep);
            } else {
                first = false;
            }
            let success = parse_str_arr(inner, single_sep, array_sep, escape_string_style, value_str);
            if !success {
                return false;
            }
        } else if let serde_json::Value::String(s) = iter {
            if !first {
                value_str.push(single_sep);
            } else {
                first = false;
            }
            if !escape_string_style {
                value_str.push_str(s);
            } else {
                value_str.push('"');
                value_str.push_str(&escape_string_cstyle(s));
                value_str.push('"');
            }
        } else {
            // should not happen
            return false;
        }
    }
    true
}

/// `boost::property_tree::read_ini` (boost/property_tree/ini_parser.hpp) as far as
/// ConfigBase::load(const ptree&) looks at the tree: the children of the root in file order,
/// each with its data. A section is a child with empty data (its own keys are not visited).
fn property_tree_read_ini(data: &[u8]) -> CfgResult<Vec<(String, String)>> {
    struct Node {
        key: String,
        data: String,
        children: Vec<(String, String)>,
    }
    let err = |msg: &str, line_no: usize| {
        ConfigurationError::with_kind(
            ConfigurationErrorKind::IniParserError,
            format!("<unspecified file>({}): {}", line_no, msg),
        )
    };
    let mut local: Vec<Node> = Vec::new();
    let mut section: Option<usize> = None;
    let mut line_no = 0usize;
    // For all lines
    for raw in data.split(|&c| c == b'\n') {
        line_no += 1;
        let line = boost_trim(raw);
        // If line is non-empty
        if line.is_empty() {
            continue;
        }
        // Comment, section or key?
        if line[0] == b';' || line[0] == b'#' {
            // Ignore comments
        } else if line[0] == b'[' {
            // If the previous section was empty, drop it again.
            if let Some(s) = section {
                if local[s].children.is_empty() {
                    local.pop();
                }
            }
            let end = match line.iter().position(|&c| c == b']') {
                None => return Err(err("unmatched '['", line_no)),
                Some(end) => end,
            };
            let key = bytes_to_string(boost_trim(&line[1..end]));
            if local.iter().any(|n| n.key == key) {
                return Err(err("duplicate section name", line_no));
            }
            local.push(Node { key, data: String::new(), children: Vec::new() });
            section = Some(local.len() - 1);
        } else {
            let eqpos = match line.iter().position(|&c| c == b'=') {
                None => return Err(err("'=' character not found in line", line_no)),
                Some(eqpos) => eqpos,
            };
            if eqpos == 0 {
                return Err(err("key expected", line_no));
            }
            let key = bytes_to_string(boost_trim(&line[..eqpos]));
            let data = bytes_to_string(boost_trim(&line[eqpos + 1..]));
            match section {
                Some(s) => {
                    if local[s].children.iter().any(|(k, _)| *k == key) {
                        return Err(err("duplicate key name", line_no));
                    }
                    local[s].children.push((key, data));
                }
                None => {
                    if local.iter().any(|n| n.key == key) {
                        return Err(err("duplicate key name", line_no));
                    }
                    local.push(Node { key, data, children: Vec::new() });
                }
            }
        }
    }
    // If the last section was empty, drop it again.
    if let Some(s) = section {
        if local[s].children.is_empty() {
            local.pop();
        }
    }
    Ok(local.into_iter().map(|n| (n.key, n.data)).collect())
}

/// What a `nlohmann::json` string looks like in `dump()`: `"`, `\` and the control characters
/// escaped, everything else as it is (`ensure_ascii` is off).
fn json_dump_string(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\u{08}' => out.push_str("\\b"),
            '\u{0C}' => out.push_str("\\f"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
}

/// Config.hpp:2619-2631 `struct SetDeserializeItem`
#[derive(Clone, Debug)]
pub struct SetDeserializeItem {
    pub opt_key: String,
    pub opt_value: String,
    pub append: bool,
}

impl SetDeserializeItem {
    /// Config.hpp:2620-2621 (key, string value)
    pub fn new(opt_key: &str, opt_value: &str, append: bool) -> Self {
        SetDeserializeItem { opt_key: opt_key.to_string(), opt_value: opt_value.to_string(), append }
    }
    /// Config.hpp:2622-2623 (key, bool)
    pub fn from_bool(opt_key: &str, value: bool, append: bool) -> Self {
        SetDeserializeItem { opt_key: opt_key.to_string(), opt_value: if value { "1" } else { "0" }.to_string(), append }
    }
    /// Config.hpp:2624-2625 (key, int): `std::to_string(value)`
    pub fn from_int(opt_key: &str, value: i32, append: bool) -> Self {
        SetDeserializeItem { opt_key: opt_key.to_string(), opt_value: value.to_string(), append }
    }
    /// Config.hpp:2626-2627 (key, float): `float_to_string_decimal_point(value)`
    pub fn from_float(opt_key: &str, value: f32, append: bool) -> Self {
        SetDeserializeItem {
            opt_key: opt_key.to_string(),
            opt_value: crate::locales_utils::float_to_string_decimal_point(value as f64, -1),
            append,
        }
    }
    /// Config.hpp:2628-2629 (key, double): `float_to_string_decimal_point(value)`
    pub fn from_double(opt_key: &str, value: f64, append: bool) -> Self {
        SetDeserializeItem {
            opt_key: opt_key.to_string(),
            opt_value: crate::locales_utils::float_to_string_decimal_point(value, -1),
            append,
        }
    }
}

/// `static_cast<TYPE*>(opt)` after `option_throw<TYPE>(opt_key, create)`.
fn option_throw_typed_mut<'a, C: ConfigBase + ?Sized, T: ConfigOptionStaticType>(
    config: &'a mut C,
    opt_key: &str,
    create: bool,
) -> CfgResult<&'a mut T> {
    let opt = config.option_throw_mut(opt_key, create)?;
    if opt.r#type() != T::static_type() {
        return Err(BadOptionTypeException::with_message("Conversion to a wrong type"));
    }
    opt.as_any_mut()
        .downcast_mut::<T>()
        .ok_or_else(|| BadOptionTypeException::with_message("Conversion to a wrong type"))
}

/// Config.hpp:2521-2668: an abstract configuration store.
///
/// The virtual methods are `def`, `optptr_mut`, `keys` and `handle_legacy` (and `optptr` of
/// the supertrait); everything else is a non-virtual member in C++ and is not meant to be
/// overridden.
pub trait ConfigBase: ConfigOptionResolver {
    // Virtual overridables:

    /// Config.hpp:2535: static configuration definition. Any value stored into this ConfigBase
    /// shall have its definition here. The definitions are static objects.
    fn def(&self) -> Option<&'static ConfigDef>;
    /// Config.hpp:2538 `virtual ConfigOption* optptr(const t_config_option_key &opt_key, bool create = false) = 0;`
    /// Find ando/or create a ConfigOption instance for a given name.
    fn optptr_mut(&mut self, opt_key: &str, create: bool) -> CfgResult<Option<&mut dyn ConfigOption>>;
    /// Config.hpp:2540: collect names of all configuration values maintained by this configuration store.
    fn keys(&self) -> t_config_option_keys;
    /// Config.hpp:2547: verify whether the opt_key has not been obsoleted or renamed. Both
    /// opt_key and value may be modified by handle_legacy(). If the opt_key is no more valid in
    /// this version of Slic3r, opt_key is cleared by handle_legacy(). handle_legacy() is called
    /// internally by set_deserialize().
    fn handle_legacy(&self, _opt_key: &mut t_config_option_key, _value: &mut String) {}

    // Non-virtual methods:

    /// Config.hpp:2554-2555 `ConfigOption* option(const t_config_option_key &opt_key, bool create = false)`
    fn option_mut(&mut self, opt_key: &str, create: bool) -> CfgResult<Option<&mut dyn ConfigOption>> {
        self.optptr_mut(opt_key, create)
    }

    /// Config.hpp:2564-2570 `ConfigOption* option_throw(const t_config_option_key &opt_key, bool create = false)`
    fn option_throw_mut(&mut self, opt_key: &str, create: bool) -> CfgResult<&mut dyn ConfigOption> {
        match self.optptr_mut(opt_key, create)? {
            None => Err(UnknownOptionException::with_key(opt_key)),
            Some(opt) => Ok(opt),
        }
    }

    /// Config.hpp:2584: apply all keys of other ConfigBase defined by this->def() to this
    /// ConfigBase. An UnknownOptionException is thrown in case some option keys of other are
    /// not defined by this->def(), or this ConfigBase is of a StaticConfig type and it does not
    /// support some of the keys, and ignore_nonexistent is not set.
    fn apply(&mut self, other: &dyn ConfigBase, ignore_nonexistent: bool) -> CfgResult<()> {
        self.apply_only(other, &other.keys(), ignore_nonexistent)
    }

    /// Config.cpp:454-493: apply explicitely enumerated keys of other ConfigBase defined by
    /// this->def() to this ConfigBase.
    fn apply_only(&mut self, other: &dyn ConfigBase, keys: &[t_config_option_key], ignore_nonexistent: bool) -> CfgResult<()> {
        // loop through options and apply them
        for opt_key in keys {
            // Create a new option with default value for the key.
            // If the key is not in the parameter definition, or this ConfigBase is a static type and it does not support the parameter,
            // an exception is thrown if not ignore_nonexistent.
            let my_opt_is_null = self.optptr_mut(opt_key, true)?.is_none();
            if my_opt_is_null {
                // opt_key does not exist in this ConfigBase and it cannot be created, because it is not defined by this->def().
                // This is only possible if other is of DynamicConfig type.
                if let Some(n) = opt_key.find('#') {
                    let opt_key2 = &opt_key[..n];
                    let mut my_opt2 = self.optptr(opt_key2).map_or(false, |o| o.as_vector_base().is_some());
                    let other_opt = other.option(opt_key2);
                    if !my_opt2 && other_opt.is_some() {
                        let created = self
                            .optptr_mut(opt_key2, true)?
                            .and_then(|o| o.as_vector_base_mut())
                            .expect("ConfigBase::apply_only(): dynamic_cast<ConfigOptionVectorBase*>(this->option(opt_key2, true)) is null");
                        if created.empty() {
                            created.resize(1, other_opt)?;
                        }
                        my_opt2 = true;
                    }
                    if my_opt2 {
                        let index = c_atoi(opt_key[n + 1..].as_bytes());
                        if let Some(other_opt) = other_opt {
                            let my_opt2 = self
                                .optptr_mut(opt_key2, false)?
                                .and_then(|o| o.as_vector_base_mut())
                                .expect("ConfigBase::apply_only(): my_opt2");
                            my_opt2.set_at(other_opt, index as usize, index as usize)?;
                        }
                        continue;
                    }
                }
                if ignore_nonexistent {
                    continue;
                }
                return Err(UnknownOptionException::with_key(opt_key));
            }
            let other_opt = other.option(opt_key);
            match other_opt {
                None => {
                    // The key was not found in the source config, therefore it will not be initialized!
                }
                Some(other_opt) => {
                    let my_opt = self.optptr_mut(opt_key, true)?.expect("ConfigBase::apply_only(): my_opt");
                    my_opt.set(other_opt)?;
                }
            }
        }
        Ok(())
    }

    /// Config.cpp:495-508: are the two configs equal? Ignoring options not present in both
    /// configs. BBS: add skipped keys logic.
    fn equals(&self, other: &dyn ConfigBase, skipped_keys: Option<&std::collections::BTreeSet<String>>) -> CfgResult<bool> {
        for opt_key in self.keys() {
            if let Some(skipped_keys) = skipped_keys {
                if skipped_keys.contains(&opt_key) {
                    continue;
                }
            }
            let this_opt = self.option(&opt_key);
            let other_opt = other.option(&opt_key);
            if let (Some(this_opt), Some(other_opt)) = (this_opt, other_opt) {
                if this_opt.operator_ne(other_opt)? {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }

    /// Config.cpp:510-521: returns options differing in the two configs, ignoring options not
    /// present in both configs.
    fn diff(&self, other: &dyn ConfigBase) -> CfgResult<t_config_option_keys> {
        let mut diff = Vec::new();
        for opt_key in self.keys() {
            let this_opt = self.option(&opt_key);
            let other_opt = other.option(&opt_key);
            if let (Some(this_opt), Some(other_opt)) = (this_opt, other_opt) {
                if this_opt.operator_ne(other_opt)? {
                    diff.push(opt_key);
                }
            }
        }
        Ok(diff)
    }

    /// Config.cpp:523-534: returns options being equal in the two configs, ignoring options
    /// not present in both configs.
    fn equal(&self, other: &dyn ConfigBase) -> CfgResult<t_config_option_keys> {
        let mut equal = Vec::new();
        for opt_key in self.keys() {
            let this_opt = self.option(&opt_key);
            let other_opt = other.option(&opt_key);
            if let (Some(this_opt), Some(other_opt)) = (this_opt, other_opt) {
                if this_opt.operator_eq(other_opt)? {
                    equal.push(opt_key);
                }
            }
        }
        Ok(equal)
    }

    /// Config.cpp:565-581 `std::string ConfigBase::opt_serialize(const t_config_option_key &opt_key) const`
    fn opt_serialize(&self, opt_key: &str) -> CfgResult<String> {
        let opt = self.option(opt_key).expect("ConfigBase::opt_serialize(): opt != nullptr");
        match opt.serialize() {
            Ok(s) => Ok(s),
            Err(err) if err.is_configuration_error() => {
                // Record the crime scene: which option, its type, and every raw value (NaN/Inf indices
                // marked). serialize()'s own message never carries the key, so we log it here where the key
                // is known, then rethrow an enriched error so the caller can name the setting to the user.
                let details = describe_option_values(Some(opt));
                log::error!("failed for option \"{}\": {} | {}", opt_key, err.what(), details);
                Err(ConfigurationError::new(format!(
                    "Failed to serialize option \"{}\": {} ({})",
                    opt_key,
                    err.what(),
                    details
                )))
            }
            Err(err) => Err(err),
        }
    }

    /// Config.hpp:2603-2604 `void set(const std::string &opt_key, bool value, bool create = false)`
    fn set_bool(&mut self, opt_key: &str, value: bool, create: bool) -> CfgResult<()> {
        option_throw_typed_mut::<Self, ConfigOptionBool>(self, opt_key, create)?.value = value;
        Ok(())
    }

    /// Config.cpp:583-593 `void ConfigBase::set(const std::string &opt_key, int value, bool create)`
    fn set_int(&mut self, opt_key: &str, value: i32, create: bool) -> CfgResult<()> {
        let opt = self.option_throw_mut(opt_key, create)?;
        match opt.r#type() {
            ConfigOptionType::CoInt => {
                opt.as_any_mut().downcast_mut::<ConfigOptionInt>().expect("static_cast<ConfigOptionInt*>").value = value;
            }
            ConfigOptionType::CoFloat => {
                opt.as_any_mut().downcast_mut::<ConfigOptionFloat>().expect("static_cast<ConfigOptionFloat*>").value =
                    value as f64;
            }
            ConfigOptionType::CoFloatOrPercent => {
                let o = opt
                    .as_any_mut()
                    .downcast_mut::<ConfigOptionFloatOrPercent>()
                    .expect("static_cast<ConfigOptionFloatOrPercent*>");
                o.value = value as f64;
                o.percent = false;
            }
            ConfigOptionType::CoString => {
                opt.as_any_mut().downcast_mut::<ConfigOptionString>().expect("static_cast<ConfigOptionString*>").value =
                    value.to_string();
            }
            _ => return Err(BadOptionTypeException::with_message("Configbase::set() - conversion from int not possible")),
        }
        Ok(())
    }

    /// Config.cpp:595-604 `void ConfigBase::set(const std::string &opt_key, double value, bool create)`
    fn set_double(&mut self, opt_key: &str, value: f64, create: bool) -> CfgResult<()> {
        let opt = self.option_throw_mut(opt_key, create)?;
        match opt.r#type() {
            ConfigOptionType::CoFloat => {
                opt.as_any_mut().downcast_mut::<ConfigOptionFloat>().expect("static_cast<ConfigOptionFloat*>").value = value;
            }
            ConfigOptionType::CoFloatOrPercent => {
                let o = opt
                    .as_any_mut()
                    .downcast_mut::<ConfigOptionFloatOrPercent>()
                    .expect("static_cast<ConfigOptionFloatOrPercent*>");
                o.value = value;
                o.percent = false;
            }
            ConfigOptionType::CoString => {
                opt.as_any_mut().downcast_mut::<ConfigOptionString>().expect("static_cast<ConfigOptionString*>").value =
                    crate::locales_utils::float_to_string_decimal_point(value, -1);
            }
            _ => return Err(BadOptionTypeException::with_message("Configbase::set() - conversion from float not possible")),
        }
        Ok(())
    }

    /// Config.hpp:2607-2610 `void set(const std::string &opt_key, const std::string &value, bool create = false)`
    /// (and the `const char*` overload).
    fn set_string(&mut self, opt_key: &str, value: &str, create: bool) -> CfgResult<()> {
        option_throw_typed_mut::<Self, ConfigOptionString>(self, opt_key, create)?.value = value.to_string();
        Ok(())
    }

    /// Config.cpp:606-622: set a configuration value from a string, it will call an overridable
    /// handle_legacy() to resolve renamed and removed configuration keys.
    fn set_deserialize_nothrow(
        &mut self,
        opt_key_src: &str,
        value_src: &str,
        substitutions_ctxt: &mut ConfigSubstitutionContext,
        append: bool,
    ) -> CfgResult<bool> {
        let mut opt_key = opt_key_src.to_string();
        let mut value = value_src.to_string();
        // Both opt_key and value may be modified by handle_legacy().
        // If the opt_key is no more valid in this version of Slic3r, opt_key is cleared by handle_legacy().
        self.handle_legacy(&mut opt_key, &mut value);
        if opt_key.is_empty() {
            // Ignore the option.
            // BBS: record these options, keep only one repeated opt_key
            if !substitutions_ctxt.unrecogized_keys.iter().any(|k| k == opt_key_src) {
                substitutions_ctxt.unrecogized_keys.push(opt_key_src.to_string());
            }
            return Ok(true);
        }
        self.set_deserialize_raw(&opt_key, &value, substitutions_ctxt, append)
    }

    /// Config.cpp:624-628: may throw BadOptionTypeException() if the operation fails.
    fn set_deserialize(
        &mut self,
        opt_key_src: &str,
        value_src: &str,
        substitutions_ctxt: &mut ConfigSubstitutionContext,
        append: bool,
    ) -> CfgResult<()> {
        if !self.set_deserialize_nothrow(opt_key_src, value_src, substitutions_ctxt, append)? {
            return Err(BadOptionValueException::with_message(format!(
                "Invalid value provided for parameter {}: {}",
                opt_key_src, value_src
            )));
        }
        Ok(())
    }

    /// Config.hpp:2617-2618 `void set_deserialize_strict(const t_config_option_key &opt_key, const std::string &str, bool append = false)`
    fn set_deserialize_strict(&mut self, opt_key: &str, str: &str, append: bool) -> CfgResult<()> {
        let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
        self.set_deserialize(opt_key, str, &mut ctxt, append)
    }

    /// Config.cpp:630-634 `void ConfigBase::set_deserialize(std::initializer_list<SetDeserializeItem> items, ConfigSubstitutionContext&)`
    fn set_deserialize_items(&mut self, items: &[SetDeserializeItem], substitutions_ctxt: &mut ConfigSubstitutionContext) -> CfgResult<()> {
        for item in items {
            self.set_deserialize(&item.opt_key, &item.opt_value, substitutions_ctxt, item.append)?;
        }
        Ok(())
    }

    /// Config.hpp:2634-2635 `void set_deserialize_strict(std::initializer_list<SetDeserializeItem> items)`
    fn set_deserialize_strict_items(&mut self, items: &[SetDeserializeItem]) -> CfgResult<()> {
        let mut ctxt = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
        self.set_deserialize_items(items, &mut ctxt)
    }

    /// Config.cpp:636-719: set a configuration value from a string (private in C++).
    fn set_deserialize_raw(
        &mut self,
        opt_key_src: &str,
        value: &str,
        substitutions_ctxt: &mut ConfigSubstitutionContext,
        append: bool,
    ) -> CfgResult<bool> {
        let mut opt_key = opt_key_src.to_string();
        // Try to deserialize the option by its name.
        let def = match self.def() {
            None => return Err(NoDefinitionException::with_key(&opt_key)),
            Some(def) => def,
        };
        let mut optdef = def.get(&opt_key);
        if optdef.is_none() {
            // If we didn't find an option, look for any other option having this as an alias.
            for (opt_first, opt_second) in &def.options {
                for opt_key2 in &opt_second.aliases {
                    if *opt_key2 == opt_key {
                        opt_key = opt_first.clone();
                        optdef = Some(opt_second);
                        break;
                    }
                }
                if optdef.is_some() {
                    break;
                }
            }
            if optdef.is_none() {
                return Err(UnknownOptionException::with_key(&opt_key));
            }
        }
        let optdef = optdef.expect("optdef");

        if !optdef.shortcut.is_empty() {
            // Aliasing for example "solid_layers" to "top_shell_layers" and "bottom_shell_layers".
            for shortcut in &optdef.shortcut {
                // Recursive call.
                if !self.set_deserialize_raw(shortcut, value, substitutions_ctxt, append)? {
                    return Ok(false);
                }
            }
            return Ok(true);
        }

        let rule = substitutions_ctxt.rule;
        let opt = self.optptr_mut(&opt_key, true)?.expect("ConfigBase::set_deserialize_raw(): opt != nullptr");
        let success;
        let mut substituted = false;
        if optdef.r#type == ConfigOptionType::CoBools && rule != ForwardCompatibilitySubstitutionRule::Disable {
            // FIXME Special handling of vectors of bools, quick and not so dirty solution before PrusaSlicer 2.3.2 release.
            let nullable = opt.nullable();
            let mut default_value = config_helpers::DeserializationSubstitution::DefaultsToFalse;
            if let Some(dv) = &optdef.default_value {
                // Default value for vectors of booleans used in a "per extruder" context, thus the default contains just a single value.
                let values = vector_ref::<u8>(&**dv);
                if values.len() == 1 && values[0] == 1 {
                    default_value = config_helpers::DeserializationSubstitution::DefaultsToTrue;
                }
            }
            let result = if nullable {
                opt.as_any_mut()
                    .downcast_mut::<ConfigOptionBoolsNullable>()
                    .expect("static_cast<ConfigOptionBoolsNullable*>")
                    .deserialize_with_substitutions(value, append, default_value)?
            } else {
                opt.as_any_mut()
                    .downcast_mut::<ConfigOptionBools>()
                    .expect("static_cast<ConfigOptionBools*>")
                    .deserialize_with_substitutions(value, append, default_value)?
            };
            success = result != config_helpers::DeserializationResult::Failed;
            substituted = result == config_helpers::DeserializationResult::Substituted;
        } else {
            let mut ok = opt.deserialize(value, append)?;
            if !ok
                && rule != ForwardCompatibilitySubstitutionRule::Disable
                // Only allow substitutions of an enum value by another enum value or a boolean value with an enum value.
                // That means, we expect enum values being added in the future and possibly booleans being converted to enums.
                && (optdef.r#type == ConfigOptionType::CoEnum
                    || optdef.r#type == ConfigOptionType::CoEnums
                    || optdef.r#type == ConfigOptionType::CoBool)
            {
                // Deserialize failed, try to substitute with a default value.
                if optdef.r#type == ConfigOptionType::CoBool {
                    opt.as_any_mut().downcast_mut::<ConfigOptionBool>().expect("static_cast<ConfigOptionBool*>").value =
                        config_helpers::enum_looks_like_true_value(value);
                } else {
                    // Just use the default of the option.
                    let dv = optdef.default_value.as_ref().expect("ConfigBase::set_deserialize_raw(): optdef->default_value is null");
                    opt.set(&**dv)?;
                }
                ok = true;
                substituted = true;
            }
            success = ok;
        }

        if substituted
            && (rule == ForwardCompatibilitySubstitutionRule::Enable || rule == ForwardCompatibilitySubstitutionRule::EnableSystemSilent)
        {
            // Log the substitution.
            let config_substitution =
                ConfigSubstitution { opt_def: Some(optdef), old_value: value.to_string(), new_value: Some(ConfigOption::clone(&*opt)) };
            substitutions_ctxt.substitutions.push(config_substitution);
        }
        Ok(success)
    }

    /// Config.cpp:721-745 `double ConfigBase::get_abs_value_at(const t_config_option_key &opt_key, size_t index) const`
    fn get_abs_value_at(&self, opt_key: &str, index: usize) -> CfgResult<f64> {
        let raw_opt = self.option(opt_key).expect("ConfigBase::get_abs_value_at(): raw_opt != nullptr");
        if raw_opt.r#type() == ConfigOptionType::CoFloats {
            return Ok(*ConfigOptionVector::<f64>::get_at(vector_ref::<f64>(raw_opt), index));
        }
        if raw_opt.r#type() == ConfigOptionType::CoFloatsOrPercents {
            let def = match self.def() {
                None => return Err(NoDefinitionException::with_key(opt_key)),
                Some(def) => def,
            };
            let opt_def = def.get(opt_key).expect("ConfigBase::get_abs_value_at(): opt_def != nullptr");

            if opt_def.ratio_over.is_empty() {
                return Ok(0.0);
            } else {
                let ratio_opt = self.option(&opt_def.ratio_over).expect("ConfigBase::get_abs_value_at(): ratio_opt is null");
                let ratio_values = vector_ref::<f64>(ratio_opt);
                return Ok(ConfigOptionVector::<FloatOrPercent>::get_at(vector_ref::<FloatOrPercent>(raw_opt), index)
                    .get_abs_value(*ConfigOptionVector::<f64>::get_at(ratio_values, index)));
            }
        }

        Err(ConfigurationError::new("ConfigBase::get_abs_value_at(): Not a valid option type for get_abs_value_at()"))
    }

    /// Config.cpp:747-788: return an absolute value of a possibly relative config variable.
    /// For example, return absolute infill extrusion width, either from an absolute value, or
    /// relative to the layer height.
    fn get_abs_value(&self, opt_key: &str) -> CfgResult<f64> {
        // Get stored option value.
        let raw_opt = self.option(opt_key).expect("ConfigBase::get_abs_value(): raw_opt != nullptr");
        if raw_opt.r#type() == ConfigOptionType::CoFloat {
            return Ok(*single_ref::<f64>(raw_opt));
        }
        if raw_opt.r#type() == ConfigOptionType::CoFloatOrPercent {
            // Get option definition.
            let def = match self.def() {
                None => return Err(NoDefinitionException::with_key(opt_key)),
                Some(def) => def,
            };
            let opt_def = def.get(opt_key).expect("ConfigBase::get_abs_value(): opt_def != nullptr");
            // Compute absolute value over the absolute value of the base option.
            // FIXME there are some ratio_over chains, which end with empty ratio_with.
            // For example, XXX_extrusion_width parameters are not handled by get_abs_value correctly.
            if opt_def.ratio_over.is_empty() {
                return Ok(0.0);
            }
            let opt = raw_opt
                .as_any()
                .downcast_ref::<ConfigOptionFloatOrPercent>()
                .expect("static_cast<const ConfigOptionFloatOrPercent*>");
            // The "ratio_over" reference may point to a per-extruder vector option (e.g. line width
            // expressed as a percentage of nozzle_diameter); resolve it over the first value in that case.
            let ratio_opt = match self.option(&opt_def.ratio_over) {
                None => return Ok(if opt.percent { 0.0 } else { opt.value }),
                Some(ratio_opt) => ratio_opt,
            };

            let ratio_over_value;
            if ratio_opt.r#type() == ConfigOptionType::CoFloats {
                let vec = vector_ref::<f64>(ratio_opt);
                if vec.is_empty() || vec[0].is_nan() {
                    return Ok(if opt.percent { 0.0 } else { opt.value });
                }
                ratio_over_value = vec[0];
            } else {
                ratio_over_value = self.get_abs_value(&opt_def.ratio_over)?;
            }
            return Ok(opt.get_abs_value(ratio_over_value));
        }
        Err(ConfigurationError::new(format!(
            "ConfigBase::get_abs_value(): Not a valid option type for get_abs_value(), parameter : {}",
            opt_key
        )))
    }

    /// Config.cpp:790-801: return an absolute value of a possibly relative config variable.
    /// For example, return absolute infill extrusion width, either from an absolute value, or
    /// relative to a provided value.
    fn get_abs_value_over(&self, opt_key: &str, ratio_over: f64) -> CfgResult<f64> {
        // Get stored option value.
        let raw_opt = self.option(opt_key).expect("ConfigBase::get_abs_value(): raw_opt != nullptr");
        if raw_opt.r#type() != ConfigOptionType::CoFloatOrPercent {
            return Err(ConfigurationError::new("ConfigBase::get_abs_value(): opt_key is not of coFloatOrPercent"));
        }
        // Compute absolute value.
        Ok(raw_opt
            .as_any()
            .downcast_ref::<ConfigOptionFloatOrPercent>()
            .expect("static_cast<const ConfigOptionFloatOrPercent*>")
            .get_abs_value(ratio_over))
    }

    /// Config.cpp:803-819 `void ConfigBase::setenv_() const`
    fn setenv_(&self) -> CfgResult<()> {
        let opt_keys = self.keys();
        for it in &opt_keys {
            // prepend the SLIC3R_ prefix
            let mut envname = String::from("SLIC3R_");
            envname.push_str(it);

            // capitalize environment variable name
            let envname: Vec<u8> =
                envname.bytes().map(|c| if c <= b'z' && c >= b'a' { c - (b'a' - b'A') } else { c }).collect();

            let value = self.opt_serialize(it)?;
            // boost::nowide::setenv(envname.c_str(), value.c_str(), 1): both texts end at their
            // first NUL, and setenv() refuses (EINVAL, not checked) an empty name or one with '='.
            let c_str = |b: &[u8]| -> Vec<u8> { b[..b.iter().position(|&c| c == 0).unwrap_or(b.len())].to_vec() };
            let name = c_str(&envname);
            let value = c_str(value.as_bytes());
            if name.is_empty() || name.contains(&b'=') {
                continue;
            }
            std::env::set_var(bytes_to_string(&name), bytes_to_string(&value));
        }
        Ok(())
    }

    /// Config.cpp:821-856 (BBS). `CNumericLocalesSetter` has no counterpart: the conversions
    /// of this file do not depend on a locale.
    fn load_string_map(
        &mut self,
        key_values: &std::collections::BTreeMap<String, String>,
        compatibility_rule: ForwardCompatibilitySubstitutionRule,
    ) -> CfgResult<ConfigSubstitutions> {
        let mut substitutions_ctxt = ConfigSubstitutionContext::new(compatibility_rule);
        for (first, second) in key_values {
            match self.set_deserialize(first, second, &mut substitutions_ctxt, false) {
                Ok(()) => {}
                Err(e) if e.kind == ConfigurationErrorKind::UnknownOptionException => {
                    // ignore
                }
                Err(e) => return Err(e),
            }
        }
        if self.has("close_fan_the_first_x_layers") && !self.has("close_additional_fan_first_x_layers") {
            if let Some(src) = self.option("close_fan_the_first_x_layers").map(|o| o.clone()) {
                if let Some(dst) = self.optptr_mut("close_additional_fan_first_x_layers", true)? {
                    dst.set(&*src)?;
                }
            }
        }
        for keys in S_AUX_FAN_STANDARD_TO_OPTIONAL_KEYS.iter() {
            if !self.has(keys.0) || self.has(keys.1) {
                continue;
            }
            let src = match self.option(keys.0).map(|o| o.clone()) {
                None => continue,
                Some(src) => src,
            };
            if let Some(dst) = self.optptr_mut(keys.1, true)? {
                dst.set(&*src)?;
            }
        }
        Ok(substitutions_ctxt.substitutions)
    }

    /// Config.cpp:858-873 (BBS: add json support)
    fn load(&mut self, file: &str, compatibility_rule: ForwardCompatibilitySubstitutionRule) -> CfgResult<ConfigSubstitutions> {
        let mut key_values: std::collections::BTreeMap<String, String> = std::collections::BTreeMap::new();
        if crate::utils::is_gcode_file(file) {
            self.load_from_gcode_file(file, compatibility_rule)
        } else if crate::utils::is_json_file(file) {
            let mut reason = String::new();
            Ok(self.load_from_json(file, compatibility_rule, &mut key_values, &mut reason))
        } else {
            log::error!("unsupported format for config file{}", file);
            Ok(Vec::new())
        }
    }

    /// Config.cpp:875-883 (BBS: add json support)
    fn load_from_json(
        &mut self,
        file: &str,
        compatibility_rule: ForwardCompatibilitySubstitutionRule,
        key_values: &mut std::collections::BTreeMap<String, String>,
        reason: &mut String,
    ) -> ConfigSubstitutions {
        let mut substitutions_ctxt = ConfigSubstitutionContext::new(compatibility_rule);

        let _ret = self.load_from_json_with_context(file, &mut substitutions_ctxt, true, key_values, reason);
        substitutions_ctxt.substitutions
    }

    /// Config.cpp:885-1243 `int ConfigBase::load_from_json(const std::string &file, ConfigSubstitutionContext&, bool load_inherits_to_config, std::map<std::string, std::string>& key_values, std::string& reason)`
    ///
    /// The file is read as nlohmann::json reads it (`nlohmann_json::parse`, the same errors).
    /// FIDELITY-NOTE: `include` is written back with serde_json's compact form of the value
    /// (`it.value().dump()` in the C++): the same text for strings, arrays of strings and integers;
    /// a floating point number would be printed by ryu rather than by nlohmann's dtoa.
    fn load_from_json_with_context(
        &mut self,
        file: &str,
        substitution_context: &mut ConfigSubstitutionContext,
        load_inherits_to_config: bool,
        key_values: &mut std::collections::BTreeMap<String, String>,
        reason: &mut String,
    ) -> i32 {
        if !std::path::Path::new(file).exists() {
            log::error!("{} file not exist.", file);
            return -1;
        }
        // boost::nowide::ifstream ifs(file); ifs >> j; (a stream that failed to open reads as empty)
        let text = std::fs::read(file).unwrap_or_default();
        let j: serde_json::Value = match nlohmann_json::parse(&text) {
            Ok(j) => j,
            Err(nlohmann_json::Error::Parse(what)) => {
                // catch(nlohmann::detail::parse_error &err)
                log::error!("parse {} got a nlohmann::detail::parse_error, reason = {}", file, what);
                *reason = format!("JsonParseError: {}", what);
                return -1;
            }
            Err(nlohmann_json::Error::Other(what)) => {
                // catch(std::exception &err): out_of_range 406 of a number that overflows
                log::error!("parse {} got a generic exception, reason = {}", file, what);
                *reason = format!("std::exception: {}", what);
                return -1;
            }
        };

        let config_def = match self.def() {
            None => {
                log::error!("no config defs!");
                return -1;
            }
            Some(config_def) => config_def,
        };

        // The body of the try block. A ConfigurationError out of it and the std::exception of
        // nlohmann::json both end in `catch (std::exception &err)`.
        enum Thrown {
            Config(ConfigurationError),
            Std(String),
        }
        impl From<ConfigurationError> for Thrown {
            fn from(e: ConfigurationError) -> Self {
                Thrown::Config(e)
            }
        }
        let mut body = || -> Result<i32, Thrown> {
            let mut different_settings_append: Vec<String> = Vec::new();
            let mut new_support_style = String::new();
            let mut is_infill_first = String::new();
            let mut get_wall_sequence = String::new();
            let mut is_project_settings = false;

            // parse the json elements: an object iterates its members in key order, null has
            // nothing to iterate, anything else has no key().
            let mut entries: Vec<(&String, &serde_json::Value)> = match &j {
                serde_json::Value::Object(map) => map.iter().collect(),
                serde_json::Value::Null => Vec::new(),
                serde_json::Value::Array(a) if a.is_empty() => Vec::new(),
                _ => {
                    return Err(Thrown::Std(
                        "[json.exception.invalid_iterator.207] cannot use key() for non-object iterators".to_string(),
                    ))
                }
            };
            entries.sort_by(|a, b| a.0.as_bytes().cmp(b.0.as_bytes()));

            for (it_key, it_value) in entries {
                let it_key: &str = it_key;
                let emplace = |key_values: &mut std::collections::BTreeMap<String, String>, k: &str| -> Result<(), Thrown> {
                    let v = json_to_string(it_value).map_err(Thrown::Std)?;
                    key_values.entry(k.to_string()).or_insert(v);
                    Ok(())
                };
                if iequals(it_key, BBL_JSON_KEY_VERSION) {
                    emplace(key_values, BBL_JSON_KEY_VERSION)?;
                } else if iequals(it_key, BBL_JSON_KEY_IS_CUSTOM) {
                    // skip it
                } else if iequals(it_key, BBL_JSON_KEY_NAME) {
                    emplace(key_values, BBL_JSON_KEY_NAME)?;
                    if it_value.as_str() == Some("project_settings") {
                        is_project_settings = true;
                    }
                } else if iequals(it_key, BBL_JSON_KEY_URL) {
                    emplace(key_values, BBL_JSON_KEY_URL)?;
                } else if iequals(it_key, BBL_JSON_KEY_TYPE) {
                    emplace(key_values, BBL_JSON_KEY_TYPE)?;
                } else if iequals(it_key, BBL_JSON_KEY_SETTING_ID) {
                    emplace(key_values, BBL_JSON_KEY_SETTING_ID)?;
                } else if iequals(it_key, BBL_JSON_KEY_FILAMENT_ID) {
                    emplace(key_values, BBL_JSON_KEY_FILAMENT_ID)?;
                } else if iequals(it_key, BBL_JSON_KEY_FROM) {
                    emplace(key_values, BBL_JSON_KEY_FROM)?;
                } else if iequals(it_key, BBL_JSON_KEY_DESCRIPTION) {
                    emplace(key_values, BBL_JSON_KEY_DESCRIPTION)?;
                } else if iequals(it_key, BBL_JSON_KEY_INSTANTIATION) {
                    emplace(key_values, BBL_JSON_KEY_INSTANTIATION)?;
                } else if !load_inherits_to_config && iequals(it_key, BBL_JSON_KEY_INHERITS) {
                    emplace(key_values, BBL_JSON_KEY_INHERITS)?;
                } else if !load_inherits_to_config && iequals(it_key, BBL_JSON_KEY_INCLUDES) {
                    let dumped = serde_json::to_string(it_value).unwrap_or_default();
                    key_values.entry(BBL_JSON_KEY_INCLUDES.to_string()).or_insert(dumped);
                } else {
                    let mut opt_key: t_config_option_key = it_key.to_string();
                    let mut value_str = String::new();

                    if let serde_json::Value::String(it_str) = it_value {
                        if opt_key == "reduce_infill_retraction" {
                            // Legacy bool: false(0) -> Disabled, true(1) bypass
                            // New profiles that don't set this will get the default "Auto"
                            if it_str == "0" || it_str == "false" {
                                // should disable
                                different_settings_append.push("reduce_infill_retraction_mode".to_string());
                            }
                        }
                        self.set_deserialize(&opt_key, it_str, substitution_context, false)?;
                        // some logic for special values
                        if opt_key == "support_type" {
                            if it_str == "hybrid(auto)" {
                                different_settings_append.push(opt_key.clone());
                                different_settings_append.push("support_style".to_string());
                                new_support_style = "tree_hybrid".to_string();
                            }
                        } else if opt_key == "wall_infill_order" {
                            // BBS: check wall_infill order to decide if it be different and append to diff_setting_append
                            if it_str == "outer wall/inner wall/infill"
                                || it_str == "infill/outer wall/inner wall"
                                || it_str == "inner-outer-inner wall/infill"
                            {
                                get_wall_sequence = "wall_seq_diff_to_system".to_string();
                            }

                            if it_str == "infill/outer wall/inner wall" || it_str == "infill/inner wall/outer wall" {
                                different_settings_append.push("is_infill_first".to_string());
                                is_infill_first = "true".to_string();
                            }
                        }
                    } else if let serde_json::Value::Array(it_arr) = it_value {
                        let opt_key_src = opt_key.clone();
                        self.handle_legacy(&mut opt_key, &mut value_str);
                        if opt_key.is_empty() {
                            // BBS: record these options
                            substitution_context.unrecogized_keys.push(opt_key_src);
                            continue;
                        }
                        let mut optdef = config_def.get(&opt_key);
                        if optdef.is_none() {
                            // If we didn't find an option, look for any other option having this as an alias.
                            for (opt_first, opt_second) in &config_def.options {
                                for opt_key2 in &opt_second.aliases {
                                    if *opt_key2 == opt_key {
                                        opt_key = opt_first.clone();
                                        optdef = Some(opt_second);
                                        break;
                                    }
                                }
                                if optdef.is_some() {
                                    break;
                                }
                            }
                        }

                        let mut single_sep = ',';
                        let array_sep = '#'; // currenty not used
                        let mut escape_string_type = false;
                        if let Some(optdef) = optdef {
                            match optdef.r#type {
                                ConfigOptionType::CoStrings => {
                                    escape_string_type = true;
                                    single_sep = ';';
                                }
                                ConfigOptionType::CoPointsGroups => {
                                    single_sep = '#';
                                }
                                _ => {}
                            }
                        }

                        // BBS: we only support 2 depth array
                        let valid = parse_str_arr(it_arr, single_sep, array_sep, escape_string_type, &mut value_str);
                        if !valid {
                            log::error!("parse {} error, invalid json array for {}", file, it_key);
                            break;
                        }
                        if valid {
                            self.set_deserialize(&opt_key, &value_str, substitution_context, false)?;
                        }
                    } else {
                        // should not happen
                        log::error!("parse {} error, invalid json type for {}", file, it_key);
                    }
                }
            }

            // Migrate: when opening old 3MF that has close_fan_the_first_x_layers but not
            // close_additional_fan_first_x_layers, copy the value to the new key as well.
            if self.has("close_fan_the_first_x_layers") && !self.has("close_additional_fan_first_x_layers") {
                if let Some(src) = self.option("close_fan_the_first_x_layers").map(|o| o.clone()) {
                    if let Some(dst) = self.optptr_mut("close_additional_fan_first_x_layers", true)? {
                        dst.set(&*src)?;
                    }
                }
                if self.has("different_settings_to_system") {
                    if let Some(diff_opt) = different_settings_to_system_mut(self, false)? {
                        for diff_str in diff_opt.iter_mut() {
                            if diff_str.contains("close_fan_the_first_x_layers")
                                && !diff_str.contains("close_additional_fan_first_x_layers")
                            {
                                diff_str.push_str(";close_additional_fan_first_x_layers");
                            }
                        }
                    }
                }
            }

            // The optional auxiliary fan inherits the standard one whenever this config level
            // does not set it explicitly, so both fans keep running at the same speed for
            // every preset and 3MF written before the optional fan became configurable.
            for keys in S_AUX_FAN_STANDARD_TO_OPTIONAL_KEYS.iter() {
                if !self.has(keys.0) || self.has(keys.1) {
                    continue;
                }
                let src = match self.option(keys.0).map(|o| o.clone()) {
                    None => continue,
                    Some(src) => src,
                };
                if let Some(dst) = self.optptr_mut(keys.1, true)? {
                    dst.set(&*src)?;
                }
            }
            if self.has("different_settings_to_system") {
                if let Some(diff_opt) = different_settings_to_system_mut(self, false)? {
                    for diff_str in diff_opt.iter_mut() {
                        for keys in S_AUX_FAN_STANDARD_TO_OPTIONAL_KEYS.iter() {
                            if diff_str.contains(keys.0) && !diff_str.contains(keys.1) {
                                diff_str.push(';');
                                diff_str.push_str(keys.1);
                            }
                        }
                    }
                }
            }

            if !different_settings_append.is_empty() {
                if different_settings_append.iter().any(|k| k == "reduce_infill_retraction_mode") {
                    // should disable
                    // FIDELITY-NOTE: the C++ takes option<ConfigOptionEnum<ReduceInfillRetractionMode>>(key, true)
                    // and writes `value` without a null check; where the option cannot be created
                    // (or is no enum) it dereferences null. Nothing is written here in that case.
                    if let Some(opt) = self.optptr_mut("reduce_infill_retraction_mode", true)? {
                        if opt.r#type() == ConfigOptionType::CoEnum {
                            opt.set_int(RIR_DISABLED)?;
                        }
                    }
                }

                if !new_support_style.is_empty() {
                    // FIDELITY-NOTE: as above, for option<ConfigOptionEnum<SupportMaterialStyle>>("support_style", true).
                    if let Some(opt) = self.optptr_mut("support_style", true)? {
                        if opt.r#type() == ConfigOptionType::CoEnum {
                            opt.set_int(SMS_TREE_HYBRID)?;
                        }
                    }
                }

                if !is_infill_first.is_empty() {
                    // FIDELITY-NOTE: as above, for option<ConfigOptionBool>("is_infill_first", true).
                    if let Some(opt) = self.optptr_mut("is_infill_first", true)? {
                        if let Some(opt) = opt.as_any_mut().downcast_mut::<ConfigOptionBool>() {
                            opt.value = true;
                        }
                    }
                }

                if is_project_settings {
                    let filament_ids_size = self
                        .option_typed::<ConfigOptionStrings>("filament_settings_id")
                        .map_or(0, |filament_ids| filament_ids.values.len());
                    let different_settings = different_settings_to_system_mut(self, true)?
                        .expect("ConfigBase::load_from_json(): option<ConfigOptionStrings>(\"different_settings_to_system\", true) is null");
                    let mut size = different_settings.len();
                    if size == 0 {
                        size = filament_ids_size + 2;
                        different_settings.resize(size, String::new());
                    }

                    let mut is_first = vec![false; size];
                    let mut original_diffs: Vec<Vec<String>> = vec![Vec::new(); size];
                    for index in 0..size {
                        if different_settings[index].is_empty() {
                            is_first[index] = true;
                        } else {
                            // remove unneeded key
                            if get_wall_sequence.is_empty() {
                                let wall_sqe_string = "wall_sequence";
                                if let Some(pos) = different_settings[index].find(wall_sqe_string) {
                                    let mut erase_len = wall_sqe_string.len();
                                    if pos + erase_len < different_settings[index].len()
                                        && different_settings[index].as_bytes()[pos + erase_len] == b';'
                                    {
                                        erase_len += 1;
                                    }
                                    different_settings[index].replace_range(pos..pos + erase_len, "");
                                }
                            }

                            if different_settings[index].is_empty() {
                                is_first[index] = true;
                                continue;
                            }

                            unescape_strings_cstyle(&different_settings[index], &mut original_diffs[index]);
                        }
                    }

                    for diff_key in &different_settings_append {
                        // get the index in the group
                        let index = 0usize;
                        let mut need_insert = true;

                        // check whether exist firstly
                        if !original_diffs[index].is_empty() {
                            for j in 0..original_diffs[index].len() {
                                if original_diffs[index][j] == *diff_key {
                                    need_insert = false;
                                    break;
                                }
                            }
                        }
                        if !need_insert {
                            continue;
                        }

                        // insert this key
                        if !is_first[index] {
                            different_settings[index].push(';');
                        } else {
                            is_first[index] = false;
                        }
                        different_settings[index].push_str(diff_key);
                    }
                }
            }
            Ok(0)
        };
        match body() {
            Ok(ret) => ret,
            Err(Thrown::Config(err)) => {
                log::error!("parse {} got a generic exception, reason = {}", file, err.what());
                *reason = format!("std::exception: {}", err.what());
                -1
            }
            Err(Thrown::Std(what)) => {
                log::error!("parse {} got a generic exception, reason = {}", file, what);
                *reason = format!("std::exception: {}", what);
                -1
            }
        }
    }

    /// Config.cpp:1245-1255 `ConfigSubstitutions ConfigBase::load_from_ini(const std::string &file, ForwardCompatibilitySubstitutionRule)`
    fn load_from_ini(&mut self, file: &str, compatibility_rule: ForwardCompatibilitySubstitutionRule) -> CfgResult<ConfigSubstitutions> {
        // A stream that failed to open is not good(): read_ini reads nothing.
        let data = std::fs::read(file).unwrap_or_default();
        let result = property_tree_read_ini(&data).and_then(|tree| self.load_tree(&tree, compatibility_rule));
        match result {
            Err(e) if e.is_configuration_error() => Err(ConfigurationError::new(format!(
                "Failed loading configuration file \"{}\": {}",
                file,
                e.what()
            ))),
            other => other,
        }
    }

    /// Config.cpp:1257-1263 `ConfigSubstitutions ConfigBase::load_from_ini_string(const std::string &data, ForwardCompatibilitySubstitutionRule)`
    fn load_from_ini_string(&mut self, data: &str, compatibility_rule: ForwardCompatibilitySubstitutionRule) -> CfgResult<ConfigSubstitutions> {
        let tree = property_tree_read_ini(data.as_bytes())?;
        self.load_tree(&tree, compatibility_rule)
    }

    /// Config.cpp:1265-1302: loading a "will be one day a legacy format" of configuration stored
    /// into 3MF or AMF. Accepts the same data as load_from_ini_string(), only with each
    /// configuration line possibly prefixed with a semicolon (G-code comment).
    fn load_from_ini_string_commented(
        &mut self,
        data: String,
        compatibility_rule: ForwardCompatibilitySubstitutionRule,
    ) -> CfgResult<ConfigSubstitutions> {
        let mut data: Vec<u8> = data.into_bytes();
        // As in the C++, a text whose first character is none of ' ', '\t', ';', '\n' and that does
        // not start with "generated by " never leaves the first branch (`i` stays 0): the loop does
        // not end (Config.cpp:1272-1296). The callers pass 3MF / AMF config texts, which start
        // with "; generated by".
        // Convert the "data" string into INI format by removing the semi-colons at the start of a line.
        // Also the "; generated by PrusaSlicer ..." comment line will be removed.
        let mut j = 0usize;
        let mut i = 0usize;
        while i < data.len() {
            if i == 0 || data[i] == b'\n' {
                // Start of a line.
                if data[i] == b'\n' {
                    // Consume LF, don't keep empty lines.
                    if j > 0 && data[j - 1] != b'\n' {
                        data[j] = data[i];
                        j += 1;
                    }
                    i += 1;
                }
                // Skip all leading spaces;
                while i < data.len() && (data[i] == b' ' || data[i] == b'\t') {
                    i += 1;
                }
                // Skip the semicolon (comment indicator).
                if i < data.len() && data[i] == b';' {
                    i += 1;
                }
                // Skip all leading spaces after semicolon.
                while i < data.len() && (data[i] == b' ' || data[i] == b'\t') {
                    i += 1;
                }
                if data[i.min(data.len())..].starts_with(b"generated by ") {
                    // Skip the "; generated by ..." line.
                    while i < data.len() && data[i] != b'\n' {
                        i += 1;
                    }
                }
            } else if data[i] == b'\r' && i + 1 < data.len() && data[i + 1] == b'\n' {
                // Skip CR.
                i += 1;
            } else {
                // Consume the rest of the data.
                data[j] = data[i];
                j += 1;
                i += 1;
            }
        }
        data.truncate(j);

        self.load_from_ini_string(&bytes_to_string(&data), compatibility_rule)
    }

    /// Config.cpp:1304-1316 `ConfigSubstitutions ConfigBase::load(const boost::property_tree::ptree &tree, ForwardCompatibilitySubstitutionRule)`:
    /// `tree` is the list of the root's children with their data, see property_tree_read_ini.
    fn load_tree(&mut self, tree: &[(String, String)], compatibility_rule: ForwardCompatibilitySubstitutionRule) -> CfgResult<ConfigSubstitutions> {
        let mut substitutions_ctxt = ConfigSubstitutionContext::new(compatibility_rule);
        for v in tree {
            match self.set_deserialize(&v.0, &v.1, &mut substitutions_ctxt, false) {
                Ok(()) => {}
                Err(e) if e.kind == ConfigurationErrorKind::UnknownOptionException => {
                    // ignore
                }
                Err(e) => return Err(e),
            }
        }
        Ok(substitutions_ctxt.substitutions)
    }

    /// Config.cpp:1336-1404: load the config keys from the given string (`static` in C++, with
    /// the config as its first argument). `str` is the C string: `None` is the null pointer and
    /// the text ends at the first NUL.
    fn load_from_gcode_string_legacy(&mut self, str: Option<&[u8]>, substitutions: &mut ConfigSubstitutionContext) -> CfgResult<usize> {
        let str = match str {
            None => return Ok(0),
            Some(str) => str,
        };
        let str = &str[..str.iter().position(|&c| c == 0).unwrap_or(str.len())];

        // BBS. Remove line numbers.
        // std::regex match_pattern("\nN[0-9]* *"); std::string replace_pattern = "\n";
        let mut result: Vec<u8> = Vec::with_capacity(str.len() + 1);
        {
            let mut i = 0usize;
            while i < str.len() {
                if str[i] == b'\n' && i + 1 < str.len() && str[i + 1] == b'N' {
                    let mut k = i + 2;
                    while k < str.len() && str[k].is_ascii_digit() {
                        k += 1;
                    }
                    while k < str.len() && str[k] == b' ' {
                        k += 1;
                    }
                    result.push(b'\n');
                    i = k;
                } else {
                    result.push(str[i]);
                    i += 1;
                }
            }
        }
        let text_len = result.len();
        // The terminating NUL the pointer walk below may look at.
        result.push(0);
        let at = |p: isize| -> u8 { result[p as usize] };

        // Walk line by line in reverse until a non-configuration key appears.
        let mut data_start = skip_whitespaces(&result[..text_len], 0) as isize;
        if at(data_start).to_ascii_uppercase() == b'N' {
            data_start = skip_word(&result[..text_len], data_start as usize) as isize;
        }

        let mut end: isize = text_len as isize;
        let mut num_key_value_pairs = 0usize;
        loop {
            // Extract next line.
            end -= 1;
            while end > data_start && (at(end) == b'\r' || at(end) == b'\n') {
                end -= 1;
            }
            if end == data_start {
                break;
            }
            let mut start = end;
            end += 1;
            while start > data_start && at(start) != b'\r' && at(start) != b'\n' {
                start -= 1;
            }
            if start == data_start {
                break;
            }
            // Extracted a line from start to end. Extract the key = value pair.
            start += 1;
            if end - start < 10 || at(start) != b';' || at(start + 1) != b' ' {
                break;
            }
            let key = start + 2;
            if !(at(key).is_ascii_lowercase() || at(key).is_ascii_uppercase()) {
                // A key must start with a letter.
                break;
            }
            let mut sep = key;
            while sep != end && at(sep) != b'=' {
                sep += 1;
            }
            if sep == end || at(sep - 1) != b' ' || at(sep + 1) != b' ' {
                break;
            }
            let value = sep + 2;
            if value > end {
                break;
            }
            let key_end = sep - 1;
            if key_end - key < 3 {
                break;
            }
            // The key may contain letters, digits and underscores.
            let mut key_valid = true;
            let mut c = key;
            while c != key_end {
                let ch = at(c);
                if !(ch.is_ascii_lowercase() || ch.is_ascii_uppercase() || ch.is_ascii_digit() || ch == b'_') {
                    key_valid = false;
                    break;
                }
                c += 1;
            }
            if !key_valid {
                break;
            }
            let key_str = bytes_to_string(&result[key as usize..key_end as usize]);
            let value_str = bytes_to_string(&result[value as usize..end as usize]);
            match self.set_deserialize(&key_str, &value_str, substitutions, false) {
                Ok(()) => num_key_value_pairs += 1,
                Err(e) if e.kind == ConfigurationErrorKind::UnknownOptionException => {
                    // ignore
                }
                Err(e) => return Err(e),
            }
            end = start;
        }

        Ok(num_key_value_pairs)
    }

    /// Config.cpp:1471-1580: load the config keys from the tail of a G-code file.
    fn load_from_gcode_file(&mut self, file: &str, compatibility_rule: ForwardCompatibilitySubstitutionRule) -> CfgResult<ConfigSubstitutions> {
        let runtime_error = |m: String| ConfigurationError::with_kind(ConfigurationErrorKind::RuntimeError, m);
        // boost::nowide::ifstream ifs(file): a file that cannot be opened reads as empty.
        let content = std::fs::read(file).unwrap_or_default();
        log::info!("before parse_file {}", file);
        // `std::getline(ifs, line)` over the whole file.
        let lines = getline_split(&content, b'\n');
        let mut line_idx = 0usize;
        // Look for Slic3r or BambuStudio header.
        // Look for the header across the whole file as the G-code may have been extended at the start by a post-processing script or the user.
        // BBS
        let has_delimiters = true;
        let mut header_end_pos = 0usize;
        {
            // BBS
            let bambuslicer_gcode_header = format!("; {}", crate::utils::SLIC3R_APP_NAME);

            let mut header_found = false;
            while line_idx < lines.len() {
                let header = lines[line_idx];
                line_idx += 1;
                header_end_pos += header.len() + 1;
                // BBS: header.c_str() ends at the first NUL
                let header = &header[..header.iter().position(|&c| c == 0).unwrap_or(header.len())];
                let mut line_c = skip_whitespaces(header, 0);
                if line_c < header.len() && header[line_c].to_ascii_uppercase() == b'N' {
                    line_c = skip_word(header, line_c);
                }
                line_c = skip_whitespaces(header, line_c);
                // BBS
                if header[line_c..].starts_with(bambuslicer_gcode_header.as_bytes()) {
                    header_found = true;
                    break;
                }
            }
            if !header_found {
                let error_message = format!("Not a gcode file generated by {}.", SLIC3R_APP_FULL_NAME);
                log::error!("{}", error_message);
                return Err(runtime_error(error_message));
            }
        }

        let mut substitutions_ctxt = ConfigSubstitutionContext::new(compatibility_rule);
        let mut key_value_pairs = 0usize;

        if has_delimiters {
            // BBS
            // PrusaSlicer starting with 2.4.0-alpha0 delimits the config section stored into G-code with
            // ; CONFIG_BLOCK_START
            // ...
            // ; CONFIG_BLOCK_END
            let mut begin_found = false;
            let mut end_found = false;
            while line_idx < lines.len() {
                let line = lines[line_idx];
                line_idx += 1;
                if line.starts_with(b"; CONFIG_BLOCK_START") {
                    begin_found = true;
                    break;
                }
            }
            if !begin_found {
                return Err(runtime_error("Config tag \"; CONFIG_BLOCK_START\" not found".to_string()));
            }
            while line_idx < lines.len() {
                let line = lines[line_idx];
                line_idx += 1;
                if line.starts_with(b"; CONFIG_BLOCK_END") {
                    end_found = true;
                    break;
                }
                // line should be a valid key = value pair.
                if let Some(pos) = line.iter().position(|&c| c == b'=') {
                    if pos > 1 && line[0] == b';' {
                        let key = bytes_to_string(boost_trim(&line[1..pos]));
                        let value = bytes_to_string(boost_trim(&line[pos + 1..]));
                        match self.set_deserialize(&key, &value, &mut substitutions_ctxt, false) {
                            Ok(()) => key_value_pairs += 1,
                            Err(e) if e.kind == ConfigurationErrorKind::UnknownOptionException => {
                                // ignore
                            }
                            Err(e) => return Err(e),
                        }
                    }
                }
            }
            if !end_found {
                return Err(runtime_error("Config tag \"; CONFIG_BLOCK_END\" not found".to_string()));
            }
        } else {
            // Slicer older than 2.4.0-alpha0 do not emit any delimiter.
            // Try a heuristics reading the G-code from back.
            let file_length = content.len();
            let header_end_pos = header_end_pos.min(file_length);
            let data_length = 65535usize.min(file_length - header_end_pos);
            let data = &content[file_length - data_length..];
            key_value_pairs = self.load_from_gcode_string_legacy(Some(data), &mut substitutions_ctxt)?;
        }

        if key_value_pairs < 80 {
            let m = format!(
                "Suspiciously low number of configuration values extracted from {}: {}",
                file, key_value_pairs
            );
            log::error!("{}", m);
            return Err(runtime_error(m));
        }

        log::info!("finished to parse_file {}", file);
        Ok(substitutions_ctxt.substitutions)
    }

    /// Config.cpp:1582-1636 (BBS: add json support). The text is what
    /// `c << std::setw(4) << j << std::endl` writes for an object of strings and arrays of
    /// strings: members in key order, four spaces of indentation.
    fn save_to_json(&self, file: &str, name: &str, from: &str, version: &str) -> CfgResult<()> {
        enum J {
            Str(String),
            Arr(Vec<String>),
        }
        let mut j: std::collections::BTreeMap<String, J> = std::collections::BTreeMap::new();
        // record the headers
        j.insert(BBL_JSON_KEY_VERSION.to_string(), J::Str(version.to_string()));
        j.insert(BBL_JSON_KEY_NAME.to_string(), J::Str(name.to_string()));
        j.insert(BBL_JSON_KEY_FROM.to_string(), J::Str(from.to_string()));

        // record all the key-values
        for opt_key in self.keys() {
            if from == "project" {
                if opt_key == "print_host"
                    || opt_key == "print_host_webui"
                    || opt_key == "printhost_apikey"
                    || opt_key == "printhost_cafile"
                    || opt_key == "printhost_user"
                    || opt_key == "printhost_password"
                    || opt_key == "printhost_port"
                {
                    continue;
                }
            }
            let opt = self.option(&opt_key).expect("ConfigBase::save_to_json(): opt is null");
            if opt.is_scalar() {
                if opt.r#type() == ConfigOptionType::CoString {
                    // keep \n, \r, \t
                    let s = opt
                        .as_any()
                        .downcast_ref::<ConfigOptionString>()
                        .expect("dynamic_cast<const ConfigOptionString *>(opt) is null");
                    j.insert(opt_key, J::Str(s.value.clone()));
                } else {
                    j.insert(opt_key, J::Str(opt.serialize()?));
                }
            } else {
                let vec = opt.as_vector_base().expect("static_cast<const ConfigOptionVectorBase*>(opt)");
                let string_values = vec.vserialize()?;
                j.insert(opt_key, J::Arr(string_values));
            }
        }

        let mut c = String::new();
        c.push_str("{\n");
        let count = j.len();
        for (n, (k, v)) in j.iter().enumerate() {
            c.push_str("    ");
            json_dump_string(&mut c, k);
            c.push_str(": ");
            match v {
                J::Str(s) => json_dump_string(&mut c, s),
                J::Arr(a) => {
                    if a.is_empty() {
                        c.push_str("[]");
                    } else {
                        c.push_str("[\n");
                        for (m, s) in a.iter().enumerate() {
                            c.push_str("        ");
                            json_dump_string(&mut c, s);
                            if m + 1 != a.len() {
                                c.push(',');
                            }
                            c.push('\n');
                        }
                        c.push_str("    ]");
                    }
                }
            }
            if n + 1 != count {
                c.push(',');
            }
            c.push('\n');
        }
        c.push_str("}\n");
        // boost::nowide::ofstream: a file that cannot be opened is not reported.
        let _ = std::fs::write(file, c.as_bytes());

        log::info!("saved config to {}\n", file);
        Ok(())
    }

    /// Config.cpp:1638-1648 `void ConfigBase::save(const std::string &file) const`
    fn save(&self, file: &str) -> CfgResult<()> {
        let mut c = String::new();
        c.push_str("# ");
        c.push_str(&crate::utils::header_slic3r_generated());
        c.push('\n');
        // The stream is written key by key: what was written before a failing opt_serialize()
        // stays in the file.
        let mut result = Ok(());
        for opt_key in self.keys() {
            match self.opt_serialize(&opt_key) {
                Ok(v) => {
                    c.push_str(&opt_key);
                    c.push_str(" = ");
                    c.push_str(&v);
                    c.push('\n');
                }
                Err(e) => {
                    c.push_str(&opt_key);
                    c.push_str(" = ");
                    result = Err(e);
                    break;
                }
            }
        }
        let _ = std::fs::write(file, c.as_bytes());
        result?;

        log::info!("saved config to {}\n", file);
        Ok(())
    }

    /// Config.cpp:1650-1659: set all the nullable values to nils.
    fn null_nullables(&mut self) -> CfgResult<()> {
        for opt_key in self.keys() {
            let opt = self.optptr_mut(&opt_key, false)?.expect("ConfigBase::null_nullables(): opt != nullptr");
            if opt.nullable() {
                // deserialize("nil", ForwardCompatibilitySubstitutionRule::Disable): the enum
                // converts to `append = false`.
                opt.deserialize("nil", false)?;
            }
        }
        Ok(())
    }
}

/// `this->option<ConfigOptionStrings>("different_settings_to_system", create)->values`
/// (Config.cpp:1099, 1125, 1154).
fn different_settings_to_system_mut<C: ConfigBase + ?Sized>(config: &mut C, create: bool) -> CfgResult<Option<&mut Vec<String>>> {
    Ok(match config.optptr_mut("different_settings_to_system", create)? {
        Some(opt) if opt.r#type() == ConfigOptionType::CoStrings => {
            opt.as_any_mut().downcast_mut::<ConfigOptionStrings>().map(|o| &mut o.values)
        }
        _ => None,
    })
}

/// The template members of ConfigBase that return a mutable option of a given class
/// (Config.hpp:2557-2562, 2572-2579), see ConfigOptionResolverTyped for the cast.
pub trait ConfigBaseTyped: ConfigBase {
    /// Config.hpp:2557-2562 `template<typename TYPE> TYPE* option(const t_config_option_key &opt_key, bool create = false)`
    fn option_typed_mut<T: ConfigOptionStaticType>(&mut self, opt_key: &str, create: bool) -> CfgResult<Option<&mut T>> {
        Ok(match self.optptr_mut(opt_key, create)? {
            Some(opt) if opt.r#type() == T::static_type() => opt.as_any_mut().downcast_mut::<T>(),
            _ => None,
        })
    }

    /// Config.hpp:2572-2579 `template<typename TYPE> TYPE* option_throw(const t_config_option_key &opt_key, bool create = false)`
    fn option_throw_typed_mut<T: ConfigOptionStaticType>(&mut self, opt_key: &str, create: bool) -> CfgResult<&mut T> {
        option_throw_typed_mut::<Self, T>(self, opt_key, create)
    }
}

impl<C: ConfigBase + ?Sized> ConfigBaseTyped for C {}

// ---------------------------------------------------------------------------
// Config.hpp:2670-2850, Config.cpp:1661-1976: DynamicConfig.
// ---------------------------------------------------------------------------

/// Config.hpp:2670-2850: configuration store with dynamic number of configuration values.
/// In Slic3r, the dynamic config is mostly used at the user interface layer.
///
/// FIDELITY-NOTE: the C++ classes derived from DynamicConfig (DynamicPrintConfig, ...) override
/// two virtual methods, `def()` and `handle_legacy()`. Rust has no derived classes: what the
/// two overrides return is held by the object (`with_def`). `DynamicConfig::new()` is the base
/// class, whose `def()` is null.
pub struct DynamicConfig {
    options: std::collections::BTreeMap<t_config_option_key, Box<dyn ConfigOption>>,
    m_def: Option<&'static ConfigDef>,
    m_handle_legacy: Option<fn(&mut t_config_option_key, &mut String)>,
}

impl Default for DynamicConfig {
    fn default() -> Self {
        DynamicConfig::new()
    }
}

/// Config.hpp:2676 `DynamicConfig(const DynamicConfig &rhs) { *this = rhs; }`
impl Clone for DynamicConfig {
    fn clone(&self) -> Self {
        let mut c = DynamicConfig { options: Default::default(), m_def: self.m_def, m_handle_legacy: self.m_handle_legacy };
        c.assign(self);
        c
    }
}

impl DynamicConfig {
    /// Config.hpp:2675 `DynamicConfig() = default;`
    pub fn new() -> Self {
        DynamicConfig { options: Default::default(), m_def: None, m_handle_legacy: None }
    }

    /// A DynamicConfig of a derived class: `def` is what its `def()` returns, `handle_legacy`
    /// its `handle_legacy()` override (`None`: the empty base implementation).
    pub fn with_def(def: &'static ConfigDef, handle_legacy: Option<fn(&mut t_config_option_key, &mut String)>) -> Self {
        DynamicConfig { options: Default::default(), m_def: Some(def), m_handle_legacy: handle_legacy }
    }

    /// Config.cpp:1661-1665 `DynamicConfig::DynamicConfig(const ConfigBase& rhs, const t_config_option_keys& keys)`
    /// (and Config.hpp:2679 with `rhs.keys()`).
    pub fn from_config(rhs: &dyn ConfigBase, keys: &[t_config_option_key]) -> Self {
        let mut c = DynamicConfig::new();
        for opt_key in keys {
            let opt = rhs.option(opt_key).expect("DynamicConfig::DynamicConfig(): rhs.option(opt_key) is null");
            c.options.insert(opt_key.clone(), opt.clone());
        }
        c
    }

    /// Config.hpp:2684-2691 `DynamicConfig& operator=(const DynamicConfig &rhs)`: copy a content
    /// of one DynamicConfig to another DynamicConfig.
    pub fn assign(&mut self, rhs: &DynamicConfig) {
        self.clear();
        for (k, v) in &rhs.options {
            self.options.insert(k.clone(), ConfigOption::clone(&**v));
        }
    }

    /// Config.hpp:2695-2702 `DynamicConfig& operator=(DynamicConfig &&rhs) noexcept`
    pub fn assign_move(&mut self, rhs: &mut DynamicConfig) {
        self.clear();
        self.options = std::mem::take(&mut rhs.options);
    }

    /// Config.hpp:2706-2722 `DynamicConfig& operator+=(const DynamicConfig &rhs)`: add a content
    /// of one DynamicConfig to another DynamicConfig.
    ///
    /// FIDELITY-NOTE: for a key both hold with the same type the C++ runs
    /// `*it->second = *kvp.second`, the copy assignment of the base class ConfigOption, which
    /// has no data members: the value this config already holds stays. That is what this does.
    pub fn operator_add_assign(&mut self, rhs: &DynamicConfig) {
        for (k, v) in &rhs.options {
            match self.options.get_mut(k) {
                None => {
                    self.options.insert(k.clone(), ConfigOption::clone(&**v));
                }
                Some(it) => {
                    if it.r#type() == v.r#type() {
                        // *it->second = *kvp.second;
                    } else {
                        *it = ConfigOption::clone(&**v);
                    }
                }
            }
        }
    }

    /// Config.hpp:2726-2740 `DynamicConfig& operator+=(DynamicConfig &&rhs)`: move a content of
    /// one DynamicConfig to another DynamicConfig.
    pub fn operator_add_assign_move(&mut self, rhs: &mut DynamicConfig) {
        for (k, v) in std::mem::take(&mut rhs.options) {
            self.options.insert(k, v);
        }
    }

    /// Config.cpp:1667-1678 `bool DynamicConfig::operator==(const DynamicConfig &rhs) const`
    pub fn operator_eq(&self, rhs: &DynamicConfig) -> CfgResult<bool> {
        let mut it1 = self.options.iter();
        let mut it2 = rhs.options.iter();
        loop {
            match (it1.next(), it2.next()) {
                (Some((k1, v1)), Some((k2, v2))) => {
                    if k1 != k2 || v1.operator_ne(&**v2)? {
                        // key or value differ
                        return Ok(false);
                    }
                }
                (None, None) => return Ok(true),
                _ => return Ok(false),
            }
        }
    }

    /// Config.hpp:2743 `bool operator!=(const DynamicConfig &rhs) const`
    pub fn operator_ne(&self, rhs: &DynamicConfig) -> CfgResult<bool> {
        Ok(!self.operator_eq(rhs)?)
    }

    /// Config.hpp:2745-2748 `void swap(DynamicConfig &other)`
    pub fn swap(&mut self, other: &mut DynamicConfig) {
        std::mem::swap(&mut self.options, &mut other.options);
    }

    /// Config.hpp:2750-2753 `void clear()`
    pub fn clear(&mut self) {
        self.options.clear();
    }

    /// Config.hpp:2755-2762 `bool erase(const t_config_option_key &opt_key)`
    pub fn erase(&mut self, opt_key: &str) -> bool {
        self.options.remove(opt_key).is_some()
    }

    /// Config.cpp:1680-1691: remove options with all nil values, those are optional and it does
    /// not help to hold them.
    pub fn remove_nil_options(&mut self) -> usize {
        let mut cnt_removed = 0usize;
        self.options.retain(|_, v| {
            if v.is_nil() {
                cnt_removed += 1;
                false
            } else {
                true
            }
        });
        cnt_removed
    }

    /// Config.hpp:2770-2771 `template<class T> T* opt(const t_config_option_key &opt_key, bool create = false)`.
    /// A `dynamic_cast` in C++, which also finds a class derived from `T`; here `T` is the class
    /// of the stored option itself.
    pub fn opt_mut<T: ConfigOption>(&mut self, opt_key: &str, create: bool) -> CfgResult<Option<&mut T>> {
        Ok(self.optptr_mut(opt_key, create)?.and_then(|o| o.as_any_mut().downcast_mut::<T>()))
    }

    /// Config.hpp:2772-2773 `template<class T> const T* opt(const t_config_option_key &opt_key) const`
    pub fn opt<T: ConfigOption>(&self, opt_key: &str) -> Option<&T> {
        self.optptr(opt_key).and_then(|o| o.as_any().downcast_ref::<T>())
    }

    /// Config.hpp:2780 `bool empty() const`
    pub fn empty(&self) -> bool {
        self.options.is_empty()
    }

    /// Config.hpp:2785-2795: set a value for an opt_key. Returns true if the value did not exist
    /// yet. This DynamicConfig will take ownership of opt. Be careful, as this method does not
    /// test the existence of opt_key in this->def().
    pub fn set_key_value(&mut self, opt_key: &str, opt: Box<dyn ConfigOption>) -> bool {
        self.options.insert(opt_key.to_string(), opt).is_none()
    }

    /// Config.cpp:1908-1915: are the two configs equal? Ignoring options not present in both
    /// configs. BBS: add skipped keys logic.
    pub fn equals(&self, other: &DynamicConfig, skipped_keys: Option<&std::collections::BTreeSet<String>>) -> CfgResult<bool> {
        Ok(!dynamic_config_iterate(self, other, |_key, l, r| l.operator_ne(r), skipped_keys)?)
    }

    /// Config.cpp:1917-1929: returns options differing in the two configs, ignoring options not
    /// present in both configs.
    pub fn diff(&self, other: &DynamicConfig) -> CfgResult<t_config_option_keys> {
        let mut diff = Vec::new();
        dynamic_config_iterate(
            self,
            other,
            |key, l, r| {
                if l.operator_ne(r)? {
                    diff.push(key.to_string());
                }
                // Continue iterating.
                Ok(false)
            },
            None,
        )?;
        Ok(diff)
    }

    /// Config.cpp:1931-1943: returns options being equal in the two configs, ignoring options
    /// not present in both configs.
    pub fn equal(&self, other: &DynamicConfig) -> CfgResult<t_config_option_keys> {
        let mut equal = Vec::new();
        dynamic_config_iterate(
            self,
            other,
            |key, l, r| {
                if l.operator_eq(r)? {
                    equal.push(key.to_string());
                }
                // Continue iterating.
                Ok(false)
            },
            None,
        )?;
        Ok(equal)
    }

    fn opt_expect(&self, opt_key: &str) -> &dyn ConfigOption {
        self.optptr(opt_key).expect("DynamicConfig: the option does not exist (null pointer in C++)")
    }

    fn opt_expect_mut(&mut self, opt_key: &str, create: bool) -> &mut dyn ConfigOption {
        self.optptr_mut(opt_key, create)
            .expect("DynamicConfig: no definition")
            .expect("DynamicConfig: the option does not exist (null pointer in C++)")
    }

    /// Config.hpp:2805 `std::string& opt_string(const t_config_option_key &opt_key, bool create = false)`
    pub fn opt_string_mut(&mut self, opt_key: &str, create: bool) -> &mut String {
        let opt = self.opt_expect_mut(opt_key, create);
        assert!(opt.r#type() == ConfigOptionType::CoString, "option<ConfigOptionString>() is null");
        &mut opt.as_any_mut().downcast_mut::<ConfigOptionString>().expect("ConfigOptionString").value
    }

    /// Config.hpp:2806 `const std::string& opt_string(const t_config_option_key &opt_key) const`
    pub fn opt_string(&self, opt_key: &str) -> &String {
        let opt = self.opt_expect(opt_key);
        assert!(opt.r#type() == ConfigOptionType::CoString, "option<ConfigOptionString>() is null");
        single_ref::<String>(opt)
    }

    /// Config.hpp:2807 `std::string& opt_string(const t_config_option_key &opt_key, unsigned int idx)`
    pub fn opt_string_at_mut(&mut self, opt_key: &str, idx: u32) -> &mut String {
        let opt = self.opt_expect_mut(opt_key, false);
        assert!(opt.r#type() == ConfigOptionType::CoStrings, "option<ConfigOptionStrings>() is null");
        ConfigOptionVector::<String>::get_at_mut(vector_mut::<String>(opt), idx as usize)
    }

    /// Config.hpp:2808 `const std::string& opt_string(const t_config_option_key &opt_key, unsigned int idx) const`
    pub fn opt_string_at(&self, opt_key: &str, idx: u32) -> &String {
        let opt = self.opt_expect(opt_key);
        assert!(opt.r#type() == ConfigOptionType::CoStrings, "option<ConfigOptionStrings>() is null");
        ConfigOptionVector::<String>::get_at(vector_ref::<String>(opt), idx as usize)
    }

    /// Config.hpp:2810 `double& opt_float(const t_config_option_key &opt_key)`
    pub fn opt_float_mut(&mut self, opt_key: &str) -> &mut f64 {
        let opt = self.opt_expect_mut(opt_key, false);
        assert!(opt.r#type() == ConfigOptionType::CoFloat, "option<ConfigOptionFloat>() is null");
        &mut opt.as_any_mut().downcast_mut::<ConfigOptionFloat>().expect("ConfigOptionFloat").value
    }

    /// Config.hpp:2811 `const double& opt_float(const t_config_option_key &opt_key) const`:
    /// `dynamic_cast<const ConfigOptionFloat*>`, which a ConfigOptionPercent and a
    /// ConfigOptionFloatOrPercent pass as well.
    pub fn opt_float(&self, opt_key: &str) -> &f64 {
        let opt = self.opt_expect(opt_key);
        assert!(
            matches!(
                opt.r#type(),
                ConfigOptionType::CoFloat | ConfigOptionType::CoPercent | ConfigOptionType::CoFloatOrPercent
            ),
            "dynamic_cast<const ConfigOptionFloat*>() is null"
        );
        single_ref::<f64>(opt)
    }

    /// Config.cpp:1945-1954 `double& DynamicConfig::opt_float(const t_config_option_key &opt_key, unsigned int idx)`:
    /// a ConfigOptionFloats or a ConfigOptionFloatsNullable (the percent vectors derive from them).
    pub fn opt_float_at_mut(&mut self, opt_key: &str, idx: u32) -> &mut f64 {
        let opt = self.opt_expect_mut(opt_key, false);
        assert!(
            matches!(opt.r#type(), ConfigOptionType::CoFloats | ConfigOptionType::CoPercents),
            "opt_floats_nullable != nullptr"
        );
        ConfigOptionVector::<f64>::get_at_mut(vector_mut::<f64>(opt), idx as usize)
    }

    /// Config.cpp:1955-1965 `const double& DynamicConfig::opt_float(const t_config_option_key &opt_key, unsigned int idx) const`
    pub fn opt_float_at(&self, opt_key: &str, idx: u32) -> &f64 {
        let opt = self.opt_expect(opt_key);
        assert!(matches!(opt.r#type(), ConfigOptionType::CoFloats | ConfigOptionType::CoPercents), "assert(false)");
        ConfigOptionVector::<f64>::get_at(vector_ref::<f64>(opt), idx as usize)
    }

    /// Config.hpp:2814 `double & opt_float_nullable(const t_config_option_key &opt_key, unsigned int idx)`
    pub fn opt_float_nullable_mut(&mut self, opt_key: &str, idx: u32) -> &mut f64 {
        let opt = self.opt_expect_mut(opt_key, false);
        assert!(opt.r#type() == ConfigOptionType::CoFloats, "option<ConfigOptionFloatsNullable>() is null");
        ConfigOptionVector::<f64>::get_at_mut(vector_mut::<f64>(opt), idx as usize)
    }

    /// Config.hpp:2815 `const double & opt_float_nullable(const t_config_option_key &opt_key, unsigned int idx) const`
    pub fn opt_float_nullable(&self, opt_key: &str, idx: u32) -> &f64 {
        let opt = self.opt_expect(opt_key);
        assert!(
            opt.nullable() && matches!(opt.r#type(), ConfigOptionType::CoFloats | ConfigOptionType::CoPercents),
            "dynamic_cast<const ConfigOptionFloatsNullable *>() is null"
        );
        ConfigOptionVector::<f64>::get_at(vector_ref::<f64>(opt), idx as usize)
    }

    /// Config.hpp:2817 `int& opt_int(const t_config_option_key &opt_key)`
    pub fn opt_int_mut(&mut self, opt_key: &str) -> &mut i32 {
        let opt = self.opt_expect_mut(opt_key, false);
        assert!(opt.r#type() == ConfigOptionType::CoInt, "option<ConfigOptionInt>() is null");
        &mut opt.as_any_mut().downcast_mut::<ConfigOptionInt>().expect("ConfigOptionInt").value
    }

    /// Config.hpp:2818 `int opt_int(const t_config_option_key &opt_key) const`:
    /// `dynamic_cast<const ConfigOptionInt*>`, which a ConfigOptionEnumGeneric passes as well.
    pub fn opt_int(&self, opt_key: &str) -> i32 {
        let opt = self.opt_expect(opt_key);
        *opt.single_value_any()
            .and_then(|a| a.downcast_ref::<i32>())
            .expect("dynamic_cast<const ConfigOptionInt*>() is null")
    }

    /// Config.hpp:2819 `int& opt_int(const t_config_option_key &opt_key, unsigned int idx)`
    pub fn opt_int_at_mut(&mut self, opt_key: &str, idx: u32) -> &mut i32 {
        let opt = self.opt_expect_mut(opt_key, false);
        assert!(opt.r#type() == ConfigOptionType::CoInts, "option<ConfigOptionInts>() is null");
        ConfigOptionVector::<i32>::get_at_mut(vector_mut::<i32>(opt), idx as usize)
    }

    /// Config.hpp:2820 `int opt_int(const t_config_option_key &opt_key, unsigned int idx) const`:
    /// `dynamic_cast<const ConfigOptionInts*>`: a ConfigOptionInts or one of the enum vectors,
    /// not a ConfigOptionIntsNullable.
    pub fn opt_int_at(&self, opt_key: &str, idx: u32) -> i32 {
        let opt = self.opt_expect(opt_key);
        assert!(
            opt.as_any().is::<ConfigOptionInts>()
                || opt.as_any().is::<ConfigOptionEnumsGeneric>()
                || opt.as_any().is::<ConfigOptionEnumsGenericNullable>(),
            "dynamic_cast<const ConfigOptionInts*>() is null"
        );
        *ConfigOptionVector::<i32>::get_at(vector_ref::<i32>(opt), idx as usize)
    }

    /// Config.hpp:2821 `int& opt_int_nullable(const t_config_option_key &opt_key, unsigned int idx)`
    pub fn opt_int_nullable_mut(&mut self, opt_key: &str, idx: u32) -> &mut i32 {
        let opt = self.opt_expect_mut(opt_key, false);
        assert!(opt.r#type() == ConfigOptionType::CoInts, "option<ConfigOptionIntsNullable>() is null");
        ConfigOptionVector::<i32>::get_at_mut(vector_mut::<i32>(opt), idx as usize)
    }

    /// Config.hpp:2822 `const int & opt_int_nullable(const t_config_option_key &opt_key, unsigned int idx) const`
    pub fn opt_int_nullable(&self, opt_key: &str, idx: u32) -> &i32 {
        let opt = self.opt_expect(opt_key);
        assert!(opt.as_any().is::<ConfigOptionIntsNullable>(), "dynamic_cast<const ConfigOptionIntsNullable*>() is null");
        ConfigOptionVector::<i32>::get_at(vector_ref::<i32>(opt), idx as usize)
    }

    /// Config.hpp:2826-2827 `template<typename ENUM> ENUM opt_enum(const t_config_option_key &opt_key) const`:
    /// it is called on option with type ConfigOptionEnumGeneric* and also ConfigOptionEnum*.
    /// Thus the virtual method getInt() is used to retrieve the enum value.
    pub fn opt_enum<E: ConfigEnum>(&self, opt_key: &str) -> CfgResult<E> {
        Ok(E::from_int(self.opt_expect(opt_key).get_int()?))
    }

    /// Config.hpp:2829 `int opt_enum(const t_config_option_key &opt_key, unsigned int idx) const` (BBS)
    pub fn opt_enum_at(&self, opt_key: &str, idx: u32) -> i32 {
        let opt = self
            .opt_expect(opt_key)
            .as_any()
            .downcast_ref::<ConfigOptionEnumsGeneric>()
            .expect("dynamic_cast<const ConfigOptionEnumsGeneric*>() is null");
        *opt.get_at(idx as usize)
    }

    /// Config.hpp:2830 `int opt_enum_nullable(const t_config_option_key &opt_key, unsigned int idx) const`
    pub fn opt_enum_nullable(&self, opt_key: &str, idx: u32) -> i32 {
        let opt = self
            .opt_expect(opt_key)
            .as_any()
            .downcast_ref::<ConfigOptionEnumsGenericNullable>()
            .expect("dynamic_cast<const ConfigOptionEnumsGenericNullable*>() is null");
        *opt.get_at(idx as usize)
    }

    /// Config.hpp:2833 `bool opt_bool(const t_config_option_key &opt_key) const`
    pub fn opt_bool(&self, opt_key: &str) -> bool {
        let opt = self.opt_expect(opt_key);
        assert!(opt.r#type() == ConfigOptionType::CoBool, "option<ConfigOptionBool>() is null");
        *single_ref::<bool>(opt)
    }

    /// Config.cpp:1967-1976 `bool DynamicConfig::opt_bool(const t_config_option_key &opt_key, unsigned int idx) const`
    pub fn opt_bool_at(&self, opt_key: &str, idx: u32) -> bool {
        let opt = self.opt_expect(opt_key);
        assert!(opt.r#type() == ConfigOptionType::CoBools, "opt_s != nullptr");
        let values = vector_ref::<u8>(opt);
        (if (idx as usize) < values.len() { values[idx as usize] } else { values[0] }) != 0
    }

    /// Config.hpp:2835 `bool opt_bool_nullable(const t_config_option_key &opt_key, unsigned int idx) const`
    pub fn opt_bool_nullable(&self, opt_key: &str, idx: u32) -> bool {
        let opt = self
            .opt_expect(opt_key)
            .as_any()
            .downcast_ref::<ConfigOptionBoolsNullable>()
            .expect("dynamic_cast<const ConfigOptionBoolsNullable*>() is null");
        opt.get_at(idx as usize)
    }

    /// Config.cpp:1722-1844: command line processing. `argv[0]` is the program name.
    pub fn read_cli(
        &mut self,
        argv: &[String],
        extra: &mut t_config_option_keys,
        mut keys: Option<&mut t_config_option_keys>,
    ) -> CfgResult<bool> {
        let def = self.def().expect("DynamicConfig::read_cli(): this->def() is null");
        // cache the CLI option => opt_key mapping
        let mut opts: std::collections::BTreeMap<String, String> = std::collections::BTreeMap::new();
        for (oit_first, oit_second) in &def.options {
            for t in oit_second.cli_args(oit_first) {
                opts.insert(t, oit_first.clone());
            }
        }

        let mut parse_options = true;
        let argc = argv.len();
        let mut i = 1usize;
        while i < argc {
            let mut token = argv[i].clone();
            i += 1;
            // Store non-option arguments in the provided vector.
            if !parse_options || !token.starts_with('-') {
                extra.push(token);
                continue;
            }
            if cfg!(target_os = "macos") && token.starts_with("-psn_") {
                // OSX launcher may add a "process serial number", for example "-psn_0_989382" to the command line.
                // While it is supposed to be dropped since OSX 10.9, we will rather ignore it.
                continue;
            }
            // Stop parsing tokens as options when -- is supplied.
            if token == "--" {
                parse_options = false;
                continue;
            }
            // Remove leading dashes (one or two).
            let dashes = if token.starts_with("--") { 2 } else { 1 };
            token.replace_range(..dashes, "");
            // Read value when supplied in the --key=value form.
            let mut value = String::new();
            {
                if let Some(equals_pos) = token.find('=') {
                    value = token[equals_pos + 1..].to_string();
                    token.truncate(equals_pos);
                }
            }
            // Look for the cli -> option mapping.
            let no = false;
            let opt_key = match opts.get(&token) {
                None => {
                    // BBS: don't use 'no-' for boolean options
                    eprintln!("Invalid option --{}", token);
                    return Ok(false);
                }
                Some(opt_key) => opt_key.clone(),
            };
            let optdef = def.options.get(&opt_key).expect("this->def()->options.at(opt_key)");

            // If the option type expects a value and it was not already provided,
            // look for it in the next token.
            if value.is_empty() && optdef.r#type != ConfigOptionType::CoBool && optdef.r#type != ConfigOptionType::CoBools {
                if i == argc {
                    eprintln!("Need values for option --{}", token);
                    return Ok(false);
                }
                value = argv[i].clone();
                i += 1;
            }

            // Store the option value.
            let existing = self.has(&opt_key);
            if let Some(keys) = keys.as_mut() {
                if !existing {
                    // Save the order of detected keys.
                    keys.push(opt_key.clone());
                }
            }
            let opt_base = self.optptr_mut(&opt_key, true)?.expect("DynamicConfig::read_cli(): opt_base is null");
            if opt_base.is_vector() {
                if !existing {
                    // remove the default values
                    opt_base.as_vector_base_mut().expect("ConfigOptionVectorBase").clear();
                }
                // Vector values will be chained. Repeated use of a parameter will append the parameter or parameters
                // to the end of the value.
                if opt_base.r#type() == ConfigOptionType::CoBools && value.is_empty() {
                    vector_mut::<u8>(opt_base).push(!no as u8);
                } else {
                    // Deserialize any other vector value (ConfigOptionInts, Floats, Percents, Points) the same way
                    // they get deserialized from an .ini file. For ConfigOptionStrings, that means that the C-style unescape
                    // will be applied for values enclosed in quotes, while values non-enclosed in quotes are left to be
                    // unescaped by the calling shell.
                    opt_base.deserialize(&value, true)?;
                }
            } else if opt_base.r#type() == ConfigOptionType::CoBool {
                if value.is_empty() {
                    opt_base.as_any_mut().downcast_mut::<ConfigOptionBool>().expect("static_cast<ConfigOptionBool*>").value = !no;
                } else {
                    opt_base.deserialize(&value, false)?;
                }
            } else if opt_base.r#type() == ConfigOptionType::CoString {
                // Do not unescape single string values, the unescaping is left to the calling shell.
                opt_base.as_any_mut().downcast_mut::<ConfigOptionString>().expect("static_cast<ConfigOptionString*>").value = value;
            } else {
                // Just bail out if the configuration value is not understood.
                let mut context = ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::Disable);
                // Any scalar value of a type different from Bool and String.
                if !self.set_deserialize_nothrow(&opt_key, &value, &mut context, false)? {
                    eprintln!("Invalid value for option --{}", token);
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }

    /// Config.hpp:2841-2842 `cbegin()` / `cend()`: the options in key order.
    pub fn cbegin(&self) -> std::collections::btree_map::Iter<'_, t_config_option_key, Box<dyn ConfigOption>> {
        self.options.iter()
    }

    /// Config.hpp:2843 `size_t size() const`
    pub fn size(&self) -> usize {
        self.options.len()
    }
}

impl ConfigOptionResolver for DynamicConfig {
    /// Config.cpp:1716-1720 `const ConfigOption* DynamicConfig::optptr(const t_config_option_key &opt_key) const`
    fn optptr(&self, opt_key: &str) -> Option<&dyn ConfigOption> {
        self.options.get(opt_key).map(|o| &**o)
    }
}

impl ConfigBase for DynamicConfig {
    /// Config.hpp:2769: allow DynamicConfig to be instantiated on ints own without a definition.
    /// If the definition is not defined, the method requiring the definition will throw
    /// NoDefinitionException.
    fn def(&self) -> Option<&'static ConfigDef> {
        self.m_def
    }

    /// Config.cpp:1693-1714 `ConfigOption* DynamicConfig::optptr(const t_config_option_key &opt_key, bool create)`
    fn optptr_mut(&mut self, opt_key: &str, create: bool) -> CfgResult<Option<&mut dyn ConfigOption>> {
        if self.options.contains_key(opt_key) {
            // Option was found.
            return Ok(self.options.get_mut(opt_key).map(|o| &mut **o));
        }
        if !create {
            // Option was not found and a new option shall not be created.
            return Ok(None);
        }
        // Try to create a new ConfigOption.
        let def = match self.def() {
            None => return Err(NoDefinitionException::with_key(opt_key)),
            Some(def) => def,
        };
        let optdef = match def.get(opt_key) {
            // Let the parent decide what to do if the opt_key is not defined by this->def().
            None => return Ok(None),
            Some(optdef) => optdef,
        };
        let opt = optdef.create_default_option()?;
        Ok(Some(&mut **self.options.entry(opt_key.to_string()).or_insert(opt)))
    }

    /// Config.cpp:1846-1853 `t_config_option_keys DynamicConfig::keys() const`
    fn keys(&self) -> t_config_option_keys {
        let mut keys = Vec::with_capacity(self.options.len());
        for (k, _) in &self.options {
            keys.push(k.clone());
        }
        keys
    }

    /// The `handle_legacy()` override of the derived class, see `with_def`.
    fn handle_legacy(&self, opt_key: &mut t_config_option_key, value: &mut String) {
        if let Some(f) = self.m_handle_legacy {
            f(opt_key, value);
        }
    }
}

/// Config.cpp:1879-1906: iterate over the pairs of options with equal keys, call the fn.
/// Returns true on early exit by fn(). BBS: add skipped key logic.
fn dynamic_config_iterate(
    lhs: &DynamicConfig,
    rhs: &DynamicConfig,
    mut r#fn: impl FnMut(&str, &dyn ConfigOption, &dyn ConfigOption) -> CfgResult<bool>,
    skipped_keys: Option<&std::collections::BTreeSet<String>>,
) -> CfgResult<bool> {
    let mut i = lhs.cbegin().peekable();
    let mut j = rhs.cbegin().peekable();
    while let (Some((ik, iv)), Some((jk, jv))) = (i.peek(), j.peek()) {
        if ik < jk {
            i.next();
        } else if ik > jk {
            j.next();
        } else {
            if skipped_keys.map_or(false, |s| s.contains(ik.as_str())) {
                // do nothing
            } else if r#fn(ik, &***iv, &***jv)? {
                // Early exit by fn.
                return Ok(true);
            }
            i.next();
            j.next();
        }
    }
    // Finished to the end.
    Ok(false)
}

// ---------------------------------------------------------------------------
// Config.hpp:2852-2866, Config.cpp:1855-1877: StaticConfig.
// ---------------------------------------------------------------------------

/// Config.hpp:2852-2866: configuration store with a static definition of configuration values.
/// In Slic3r, the static configuration stores are during the slicing / g-code generation for
/// efficiency reasons, because the configuration values could be accessed directly.
///
/// The class adds two methods to ConfigBase. A static config store implements `ConfigBase`
/// with `keys()` forwarding to `StaticConfig::keys(self)` and calls
/// `StaticConfig::set_defaults(self)` from its constructor.
pub struct StaticConfig;

impl StaticConfig {
    /// Config.cpp:1855-1867: set all statically defined config options to their defaults
    /// defined by this->def().
    pub fn set_defaults<C: ConfigBase + ?Sized>(this: &mut C) -> CfgResult<()> {
        // use defaults from definition
        if let Some(defs) = this.def() {
            for key in this.keys() {
                let def = defs.get(&key);
                let opt = this.optptr_mut(&key, false)?;
                if let (Some(def), Some(opt)) = (def, opt) {
                    if let Some(default_value) = &def.default_value {
                        opt.set(&**default_value)?;
                    }
                }
            }
        }
        Ok(())
    }

    /// Config.cpp:1869-1877: gets list of config option names for each config option of
    /// this->def, which has a static counter-part defined by the derived object and which could
    /// be resolved by this->optptr(key) call.
    pub fn keys<C: ConfigBase + ?Sized>(this: &C) -> t_config_option_keys {
        let mut keys = Vec::new();
        let def = this.def().expect("StaticConfig::keys(): this->def() != nullptr");
        for (opt_def_first, _) in &def.options {
            if this.option(opt_def_first).is_some() {
                keys.push(opt_def_first.clone());
            }
        }
        keys
    }
}

// ---------------------------------------------------------------------------
// Config.cpp:1406-1469: ReverseLineReader.
// ---------------------------------------------------------------------------

/// Config.cpp:1406-1469: reading a config from G-code back to front for performance reasons: we
/// don't want to scan hundreds of MB file for a short config block, which we expect to find at
/// the end of the G-code. (The class is defined in Config.cpp and not used by it at the pin.)
pub struct ReverseLineReader<'a, R: std::io::Read + std::io::Seek> {
    m_ifs: &'a mut R,
    m_block: Vec<u8>,
    m_block_size: usize,
    m_block_len: usize,
    m_file_start: u64,
    m_file_pos: u64,
}

impl<'a, R: std::io::Read + std::io::Seek> ReverseLineReader<'a, R> {
    /// Config.cpp:1414-1419: stop at file_start.
    pub fn new(ifs: &'a mut R, file_start: u64) -> Self {
        let m_block_size = 65536usize;
        let m_file_pos = ifs.seek(std::io::SeekFrom::End(0)).unwrap_or(0);
        ReverseLineReader { m_ifs: ifs, m_block: vec![0u8; m_block_size], m_block_size, m_block_len: 0, m_file_start: file_start, m_file_pos }
    }

    /// Config.cpp:1421-1460 `bool getline(std::string &out)`
    pub fn getline(&mut self, out: &mut String) -> bool {
        out.clear();
        // The bytes of the line, collected back to front.
        let mut line: Vec<u8> = Vec::new();
        loop {
            if self.m_block_len == 0 {
                // Read the next block.
                self.m_block_len = (self.m_block_size as u64).min(self.m_file_pos.saturating_sub(self.m_file_start)) as usize;
                if self.m_block_len == 0 {
                    *out = bytes_to_string(&line);
                    return false;
                }
                self.m_file_pos -= self.m_block_len as u64;
                let read_ok = self.m_ifs.seek(std::io::SeekFrom::Start(self.m_file_pos)).is_ok()
                    && self.m_ifs.read_exact(&mut self.m_block[..self.m_block_len]).is_ok();
                if !read_ok {
                    *out = bytes_to_string(&line);
                    return false;
                }
            }

            // Non-empty buffer. Find another LF.
            let mut i: isize = self.m_block_len as isize - 1;
            while i >= 0 {
                if self.m_block[i as usize] == b'\n' {
                    break;
                }
                i -= 1;
            }
            // i is position of LF or -1 if not found.
            if i == -1 {
                // LF not found. Just make a backup of the buffer and continue.
                line.splice(0..0, self.m_block[..self.m_block_len].iter().cloned());
                self.m_block_len = 0;
            } else {
                // Copy new line to the output. It may be empty.
                line.splice(0..0, self.m_block[i as usize + 1..self.m_block_len].iter().cloned());
                // Block length without the newline.
                self.m_block_len = i as usize;
                // Remove CRLF from the end of the block.
                if self.m_block_len > 0 && self.m_block[self.m_block_len - 1] == b'\r' {
                    self.m_block_len -= 1;
                }
                *out = bytes_to_string(&line);
                return true;
            }
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_escape_string_cstyle() {
        assert_eq!(escape_string_cstyle("a\nb\rc\\d"), "a\\nb\\rc\\\\d");
        assert_eq!(escape_string_cstyle("plain"), "plain");
    }

    #[test]
    fn test_unescape_string_cstyle() {
        let mut out = String::new();
        assert!(unescape_string_cstyle("a\\nb\\rc\\\\d", &mut out));
        assert_eq!(out, "a\nb\rc\\d");
    }

    #[test]
    fn test_escape_strings_roundtrip() {
        let strs = vec![
            "simple".to_string(),
            "has space".to_string(),
            "has\"quote".to_string(),
        ];
        let escaped = escape_strings_cstyle(&strs);
        let mut back: Vec<String> = Vec::new();
        assert!(unescape_strings_cstyle(&escaped, &mut back));
        assert_eq!(back, strs);
    }

    #[test]
    fn test_unescape_trailing_semicolon_emits_empty() {
        let mut out: Vec<String> = Vec::new();
        assert!(unescape_strings_cstyle("a;", &mut out));
        assert_eq!(out, vec!["a".to_string(), String::new()]);
    }

    #[test]
    fn test_escape_ampersand() {
        assert_eq!(escape_ampersand("a&b&&c"), "a&&b&&&&c");
    }

    #[test]
    fn test_config_helpers() {
        assert!(config_helpers::looks_like_enum_value("Enabled_value-1"));
        assert!(!config_helpers::looks_like_enum_value("1abc"));
        assert!(!config_helpers::looks_like_enum_value(""));
        assert!(config_helpers::enum_looks_like_true_value("  ON "));
        assert!(config_helpers::enum_looks_like_true_value("enabled"));
        assert!(!config_helpers::enum_looks_like_true_value("off"));
    }

    #[test]
    fn test_float_or_percent() {
        let p = FloatOrPercent::with(50.0, true);
        assert_eq!(p.get_abs_value(200.0), 100.0);
        let a = FloatOrPercent::with(7.0, false);
        assert_eq!(a.get_abs_value(200.0), 7.0);
        assert!(FloatOrPercent::with(1.0, false) < FloatOrPercent::with(1.0, true));
    }

    #[test]
    fn test_gcode_helpers() {
        assert!(is_whitespace(b' '));
        assert!(is_end_of_line(b'\n'));
        assert!(is_end_of_gcode_line(b';'));
        let b = b"N123 G1";
        assert_eq!(skip_word(b, 0), 4);
        assert_eq!(skip_whitespaces(b, 4), 5);
    }
}
