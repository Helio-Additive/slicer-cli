//! Faithful 1:1 port of the tractable, self-contained portions of
//! `libslic3r/Config.cpp` (+ `Config.hpp`).
//!
//! SCOPE NOTE (read before extending this file):
//!
//! The bulk of `Config.cpp`/`Config.hpp` implements the C++ polymorphic
//! configuration-reflection system: the `ConfigOption` virtual hierarchy
//! (`ConfigOptionFloat`, `ConfigOptionFloats`, `ConfigOptionEnumGeneric`, …),
//! `ConfigOptionDef`/`ConfigDef`, and the `ConfigBase`/`DynamicConfig`/
//! `StaticConfig` runtime-typed dictionaries, together with cereal
//! (de)serialization, nlohmann-json profile loading, and boost
//! property_tree INI parsing.
//!
//! This crate deliberately does NOT mirror that runtime-typed dictionary.
//! Instead it uses plain typed Rust structs (`PrintConfig`,
//! `PrintRegionConfig`, `GCodeConfig`, `AppConfig`, …) with serde. Porting
//! the `ConfigOption`/`ConfigBase` hierarchy faithfully would require
//! reproducing the entire 2870-line header type system (virtual dispatch,
//! templated vector options, cereal `CEREAL_REGISTER_TYPE` polymorphism) plus
//! the `PrintConfig.hpp` enum keys-maps threaded through `handle_legacy`. None
//! of those types exist in this crate, so the dictionary-bound symbols are
//! intentionally left unported (see the porter report for the blocked list).
//!
//! What IS ported here, line-by-line, are the standalone helper functions that
//! have NO dependency on the `ConfigOption` hierarchy and that are relevant to
//! byte-exact config/G-code (de)serialization:
//!   * the C-style string (un)escaping functions
//!   * `escape_ampersand`
//!   * the `ConfigHelpers` inline predicates
//!   * the static G-code line-scanning helpers
//!
//! `coord_t -> i64`, `coordf_t -> f64` per the porting convention (none of the
//! ported functions use those types, but the convention is noted for future
//! work). wasm-safe: no system/dylib dependencies.

// ===========================================================================
// ConfigHelpers (Config.hpp:89-117)
// ===========================================================================
pub mod config_helpers {
    // Config.hpp:90-99  inline bool looks_like_enum_value(std::string value)
    pub fn looks_like_enum_value(value: &str) -> bool {
        // Config.hpp:92  boost::trim(value);
        let value = value.trim_matches(|c: char| c == ' ' || c == '\t' || c == '\r' || c == '\n');
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
        let value = value.trim_matches(|c: char| c == ' ' || c == '\t' || c == '\r' || c == '\n');
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
#[derive(Clone, Copy, Debug)]
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
// C-style string (un)escaping (Config.cpp:48-215, 217-232)
// ===========================================================================

// Escape \n, \r and backslash
// Config.cpp:49  std::string escape_string_cstyle(const std::string &str)
pub fn escape_string_cstyle(str: &str) -> String {
    // Allocate a buffer twice the input string length,
    // so the output will fit even if all input characters get escaped.
    // Config.cpp:53-54  std::vector<char> out(str.size() * 2, 0); char *outptr = out.data();
    let mut out: Vec<u8> = Vec::with_capacity(str.len() * 2);
    // Config.cpp:55-68
    for &c in str.as_bytes() {
        if c == b'\r' {
            // Config.cpp:57-59
            out.push(b'\\');
            out.push(b'r');
        } else if c == b'\n' {
            // Config.cpp:60-62
            out.push(b'\\');
            out.push(b'n');
        } else if c == b'\\' {
            // Config.cpp:63-65
            out.push(b'\\');
            out.push(b'\\');
        } else {
            // Config.cpp:66-67
            out.push(c);
        }
    }
    // Config.cpp:69  return std::string(out.data(), outptr - out.data());
    String::from_utf8_lossy(&out).into_owned()
}

// Config.cpp:72  std::string escape_strings_cstyle(const std::vector<std::string> &strs)
pub fn escape_strings_cstyle(strs: &[String]) -> String {
    // 1) Estimate the output buffer size to avoid buffer reallocation.
    // Config.cpp:75-78
    // (Rust Vec grows automatically; the estimate is purely a reserve hint.)
    let mut outbuflen: usize = 0;
    for str in strs.iter() {
        // Reserve space for every character escaped + quotes + semicolon.
        outbuflen += str.len() * 2 + 3;
    }
    // 2) Fill in the buffer.
    // Config.cpp:80-81  std::vector<char> out(outbuflen, 0); char *outptr = out.data();
    let mut out: Vec<u8> = Vec::with_capacity(outbuflen);
    // Config.cpp:82-118
    for (j, str) in strs.iter().enumerate() {
        // Config.cpp:83-85  if (j > 0) separate the strings.
        if j > 0 {
            out.push(b';');
        }
        let bytes = str.as_bytes();
        // Is the string simple or complex? Complex string contains spaces, tabs, new lines and other
        // escapable characters. Empty string shall be quoted as well, if it is the only string in strs.
        // Config.cpp:89  bool should_quote = strs.size() == 1 && str.empty();
        let mut should_quote = strs.len() == 1 && str.is_empty();
        // Config.cpp:90-96
        for &c in bytes {
            if c == b' ' || c == b'\t' || c == b'\\' || c == b'"' || c == b'\r' || c == b'\n' {
                should_quote = true;
                break;
            }
        }
        if should_quote {
            // Config.cpp:98-113
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
            // Config.cpp:114-117  memcpy(outptr, str.data(), str.size());
            out.extend_from_slice(bytes);
        }
    }
    // Config.cpp:119  return std::string(out.data(), outptr - out.data());
    String::from_utf8_lossy(&out).into_owned()
}

// Unescape \n, \r and backslash
// Config.cpp:123  bool unescape_string_cstyle(const std::string &str, std::string &str_out)
pub fn unescape_string_cstyle(str: &str, str_out: &mut String) -> bool {
    // Config.cpp:125-126  std::vector<char> out(str.size(), 0); char *outptr = out.data();
    let mut out: Vec<u8> = Vec::with_capacity(str.len());
    let bytes = str.as_bytes();
    let len = bytes.len();
    // Config.cpp:127  for (size_t i = 0; i < str.size(); ++ i)
    let mut i: usize = 0;
    while i < len {
        // Config.cpp:128  char c = str[i];
        let mut c = bytes[i];
        // Config.cpp:129  if (c == '\\')
        if c == b'\\' {
            // Config.cpp:130-131  if (++ i == str.size()) return false;
            i += 1;
            if i == len {
                return false;
            }
            // Config.cpp:132  c = str[i];
            c = bytes[i];
            if c == b'r' {
                // Config.cpp:133-134  if (c == 'r') (*outptr ++) = '\r';
                out.push(b'\r');
            } else if c == b'n' {
                // Config.cpp:135-136  else if (c == 'n') (*outptr ++) = '\n';
                out.push(b'\n');
            } else {
                // Config.cpp:137-138  else (*outptr ++) = c;
                out.push(c);
            }
        } else {
            // Config.cpp:139-140  else (*outptr ++) = c;
            out.push(c);
        }
        i += 1;
    }
    // Config.cpp:142  str_out.assign(out.data(), outptr - out.data());
    *str_out = String::from_utf8_lossy(&out).into_owned();
    // Config.cpp:143  return true;
    true
}

// Config.cpp:146  bool unescape_strings_cstyle(const std::string &str, std::vector<std::string> &out)
pub fn unescape_strings_cstyle(str: &str, out: &mut Vec<String>) -> bool {
    let bytes = str.as_bytes();
    let len = bytes.len();
    // Config.cpp:148-149  if (str.empty()) return true;
    if len == 0 {
        return true;
    }

    // Config.cpp:151  size_t i = 0;
    let mut i: usize = 0;
    // Config.cpp:152  for (;;)
    loop {
        // Skip white spaces.
        // Config.cpp:154-159
        let mut c = bytes[i];
        while c == b' ' || c == b'\t' {
            i += 1;
            if i == len {
                return true;
            }
            c = bytes[i];
        }
        // Start of a word.
        // Config.cpp:161-162  std::vector<char> buf; buf.reserve(16);
        let mut buf: Vec<u8> = Vec::with_capacity(16);
        // Is it enclosed in quotes?
        // Config.cpp:164  c = str[i];
        c = bytes[i];
        // Config.cpp:165  if (c == '"')
        if c == b'"' {
            // Complex case, string is enclosed in quotes.
            // Config.cpp:167-183  for (++ i; i < str.size(); ++ i)
            i += 1;
            while i < len {
                c = bytes[i];
                // Config.cpp:169-172  if (c == '"') break;
                if c == b'"' {
                    // End of string.
                    break;
                }
                // Config.cpp:173-181
                if c == b'\\' {
                    // Config.cpp:174-175  if (++ i == str.size()) return false;
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
                // Config.cpp:182  buf.push_back(c);
                buf.push(c);
                i += 1;
            }
            // Config.cpp:184-185  if (i == str.size()) return false;
            if i == len {
                return false;
            }
            // Config.cpp:186  ++ i;
            i += 1;
        } else {
            // Config.cpp:187-194  for (; i < str.size(); ++ i)
            while i < len {
                c = bytes[i];
                // Config.cpp:190-191  if (c == ';') break;
                if c == b';' {
                    break;
                }
                buf.push(c);
                i += 1;
            }
        }
        // Store the string into the output vector.
        // Config.cpp:196  out.push_back(std::string(buf.data(), buf.size()));
        out.push(String::from_utf8_lossy(&buf).into_owned());
        // Config.cpp:197-198  if (i == str.size()) return true;
        if i == len {
            return true;
        }
        // Skip white spaces.
        // Config.cpp:200-206
        c = bytes[i];
        while c == b' ' || c == b'\t' {
            i += 1;
            if i == len {
                // End of string. This is correct.
                return true;
            }
            c = bytes[i];
        }
        // Config.cpp:207-208  if (c != ';') return false;
        if c != b';' {
            return false;
        }
        // Config.cpp:209-213  if (++ i == str.size())
        i += 1;
        if i == len {
            // Emit one additional empty string.
            out.push(String::new());
            return true;
        }
    }
}

// Config.cpp:217  std::string escape_ampersand(const std::string& str)
pub fn escape_ampersand(str: &str) -> String {
    // Allocate a buffer 2 times the input string length,
    // so the output will fit even if all input characters get escaped.
    // Config.cpp:221-222  std::vector<char> out(str.size() * 6, 0); char* outptr = out.data();
    let mut out: Vec<u8> = Vec::with_capacity(str.len() * 6);
    // Config.cpp:223-230
    for &c in str.as_bytes() {
        if c == b'&' {
            // Config.cpp:225-227
            out.push(b'&');
            out.push(b'&');
        } else {
            // Config.cpp:228-229
            out.push(c);
        }
    }
    // Config.cpp:231  return std::string(out.data(), outptr - out.data());
    String::from_utf8_lossy(&out).into_owned()
}

// ===========================================================================
// G-code line-scanning helpers (Config.cpp:1212-1228)
// ===========================================================================

// BBS
// Config.cpp:1213  static bool is_whitespace(char c)
pub fn is_whitespace(c: u8) -> bool {
    c == b' ' || c == b'\t'
}

// Config.cpp:1214  static bool is_end_of_line(char c)
pub fn is_end_of_line(c: u8) -> bool {
    c == b'\r' || c == b'\n' || c == 0
}

// Config.cpp:1215  static bool is_end_of_gcode_line(char c)
pub fn is_end_of_gcode_line(c: u8) -> bool {
    c == b';' || is_end_of_line(c)
}

// Config.cpp:1216  static bool is_end_of_word(char c)
pub fn is_end_of_word(c: u8) -> bool {
    is_whitespace(c) || is_end_of_gcode_line(c)
}

// Config.cpp:1218-1222  static const char* skip_word(const char* c)
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

// Config.cpp:1224-1228  static const char* skip_whitespaces(const char* c)
// Returns the index of the first non-whitespace byte at or after `start`.
pub fn skip_whitespaces(bytes: &[u8], start: usize) -> usize {
    let mut c = start;
    // for (; is_whitespace(*c); ++c) ;
    while c < bytes.len() && is_whitespace(bytes[c]) {
        c += 1;
    }
    c
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
