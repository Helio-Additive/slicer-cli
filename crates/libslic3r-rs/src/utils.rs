//! Faithful 1:1 port of BambuStudio `src/libslic3r/Utils.{hpp,cpp}`.
//!
//! Translation rules: snake_case names, same order, same control flow, same
//! constants, same rounding/integer-vs-float, same edge cases. `// Utils.*:NNN`
//! refs mark the originating C++ line. coord_t -> i64, coordf_t -> f64.
//!
//! Native/platform-only symbols (TBB, Win32, openssl MD5, raw syscalls,
//! boost::log sinks) are noted where they are blocked and intentionally not
//! reimplemented here so the crate stays wasm-safe.

use std::sync::Mutex;

// chrono traits for get_bbl_finish_time_dhm (Utils.hpp:827) std::localtime / std::tm fields.
use chrono::{Datelike, Timelike};

// Reuse the data_dir global that already lives in log_sink (Utils.cpp:264-273),
// so there is a single source of truth for `g_data_dir`.
pub use crate::log_sink::{data_dir, set_data_dir};
// Reuse the LogEncOptions/LogEncType definitions (Utils.hpp:366-378).
pub use crate::log_sink::{LogEncOptions, LogEncType};

// ============================================================================
// CLI error codes (Utils.hpp:21-75)
// ============================================================================

// Utils.hpp:21
pub const CLI_SUCCESS: i32 = 0;
// Utils.hpp:22
pub const CLI_ENVIRONMENT_ERROR: i32 = -1;
// Utils.hpp:23
pub const CLI_INVALID_PARAMS: i32 = -2;
// Utils.hpp:24
pub const CLI_FILE_NOTFOUND: i32 = -3;
// Utils.hpp:25
pub const CLI_FILELIST_INVALID_ORDER: i32 = -4;
// Utils.hpp:26
pub const CLI_CONFIG_FILE_ERROR: i32 = -5;
// Utils.hpp:27
pub const CLI_DATA_FILE_ERROR: i32 = -6;
// Utils.hpp:28
pub const CLI_INVALID_PRINTER_TECH: i32 = -7;
// Utils.hpp:29
pub const CLI_UNSUPPORTED_OPERATION: i32 = -8;

// Utils.hpp:31
pub const CLI_COPY_OBJECTS_ERROR: i32 = -9;
// Utils.hpp:32
pub const CLI_SCALE_TO_FIT_ERROR: i32 = -10;
// Utils.hpp:33
pub const CLI_EXPORT_STL_ERROR: i32 = -11;
// Utils.hpp:34
pub const CLI_EXPORT_OBJ_ERROR: i32 = -12;
// Utils.hpp:35
pub const CLI_EXPORT_3MF_ERROR: i32 = -13;
// Utils.hpp:36
pub const CLI_OUT_OF_MEMORY: i32 = -14;
// Utils.hpp:37
pub const CLI_3MF_NOT_SUPPORT_MACHINE_CHANGE: i32 = -15;
// Utils.hpp:38
pub const CLI_3MF_NEW_MACHINE_NOT_SUPPORTED: i32 = -16;
// Utils.hpp:39
pub const CLI_PROCESS_NOT_COMPATIBLE: i32 = -17;
// Utils.hpp:40
pub const CLI_INVALID_VALUES_IN_3MF: i32 = -18;
// Utils.hpp:41
pub const CLI_POSTPROCESS_NOT_SUPPORTED: i32 = -19;
// Utils.hpp:42
pub const CLI_PRINTABLE_SIZE_REDUCED: i32 = -20;
// Utils.hpp:43
pub const CLI_OBJECT_ARRANGE_FAILED: i32 = -21;
// Utils.hpp:44
pub const CLI_OBJECT_ORIENT_FAILED: i32 = -22;
// Utils.hpp:45
pub const CLI_MODIFIED_PARAMS_TO_PRINTER: i32 = -23;
// Utils.hpp:46
pub const CLI_FILE_VERSION_NOT_SUPPORTED: i32 = -24;
// Utils.hpp:47
pub const CLI_3MF_FEATURE_NOT_SUPPORTED: i32 = -25;

// Utils.hpp:50
pub const CLI_NO_SUITABLE_OBJECTS: i32 = -50;
// Utils.hpp:51
pub const CLI_VALIDATE_ERROR: i32 = -51;
// Utils.hpp:52
pub const CLI_OBJECTS_PARTLY_INSIDE: i32 = -52;
// Utils.hpp:53
pub const CLI_EXPORT_CACHE_DIRECTORY_CREATE_FAILED: i32 = -53;
// Utils.hpp:54
pub const CLI_EXPORT_CACHE_WRITE_FAILED: i32 = -54;
// Utils.hpp:55
pub const CLI_IMPORT_CACHE_NOT_FOUND: i32 = -55;
// Utils.hpp:56
pub const CLI_IMPORT_CACHE_DATA_CAN_NOT_USE: i32 = -56;
// Utils.hpp:57
pub const CLI_IMPORT_CACHE_LOAD_FAILED: i32 = -57;
// Utils.hpp:58
pub const CLI_SLICING_TIME_EXCEEDS_LIMIT: i32 = -58;
// Utils.hpp:59
pub const CLI_TRIANGLE_COUNT_EXCEEDS_LIMIT: i32 = -59;
// Utils.hpp:60
pub const CLI_NO_SUITABLE_OBJECTS_AFTER_SKIP: i32 = -60;
// Utils.hpp:61
pub const CLI_FILAMENT_NOT_MATCH_BED_TYPE: i32 = -61;
// Utils.hpp:62
pub const CLI_FILAMENTS_DIFFERENT_TEMP: i32 = -62;
// Utils.hpp:63
pub const CLI_OBJECT_COLLISION_IN_SEQ_PRINT: i32 = -63;
// Utils.hpp:64
pub const CLI_OBJECT_COLLISION_IN_LAYER_PRINT: i32 = -64;
// Utils.hpp:65
pub const CLI_SPIRAL_MODE_INVALID_PARAMS: i32 = -65;
// Utils.hpp:66
pub const CLI_FILAMENT_CAN_NOT_MAP: i32 = -66;
// Utils.hpp:67
pub const CLI_ONLY_ONE_TPU_SUPPORTED: i32 = -67;
// Utils.hpp:68
pub const CLI_FILAMENTS_NOT_SUPPORTED_BY_EXTRUDER: i32 = -68;

// Utils.hpp:70
pub const CLI_SLICING_ERROR: i32 = -100;
// Utils.hpp:71
pub const CLI_GCODE_PATH_CONFLICTS: i32 = -101;
// Utils.hpp:72
pub const CLI_GCODE_PATH_IN_UNPRINTABLE_AREA: i32 = -102;
// Utils.hpp:73
pub const CLI_FILAMENT_UNPRINTABLE_ON_FIRST_LAYER: i32 = -103;
// Utils.hpp:74
pub const CLI_GCODE_PATH_OUTSIDE: i32 = -104;
// Utils.hpp:75
pub const CLI_GCODE_IN_WRAPPING_DETECT_AREA: i32 = -105;

// ============================================================================
// CopyFileResult enum (Utils.hpp:401-408)
// ============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CopyFileResult {
    // Utils.hpp:402
    Success = 0,
    // Utils.hpp:403
    FailCopyFile,
    // Utils.hpp:404
    FailFilesDifferent,
    // Utils.hpp:405
    FailRenaming,
    // Utils.hpp:406
    FailCheckOriginNotOpened,
    // Utils.hpp:407
    FailCheckTargetNotOpened,
}

// ============================================================================
// Inline helpers from Utils.hpp
// ============================================================================

// Utils.hpp:116
// BBS: convert 0.1.3.4 version format to 00.01.03.04 format, like AA.BB.CC.DD
pub fn convert_to_full_version(short_version: &str) -> String {
    // Utils.hpp:118
    let mut result = String::from("");
    // Utils.hpp:119-120  boost::split(items, short_version, boost::is_any_of("."));
    let items: Vec<&str> = short_version.split('.').collect();
    // Utils.hpp:121
    if items.len() == 4 {
        // Utils.hpp:122
        for i in 0..4 {
            // Utils.hpp:123-125  ss << std::setw(2) << std::setfill('0') << items[i];
            result += &format!("{:0>2}", items[i]);
            // Utils.hpp:126-127
            if i != 4 - 1 {
                result += ".";
            }
        }
        // Utils.hpp:129
        return result;
    }
    // Utils.hpp:131
    result
}

// ----------------------------------------------------------------------------
// PathSanitizer (Utils.hpp:131-146, utils.cpp:921-1052)
//
// Makes file paths safe to write into logs by masking the user name and user id. Usually only the
// file name is returned; a path that only looks like it holds the user name is returned whole,
// masked. Never splits a UTF-8 character.
// ----------------------------------------------------------------------------

/// utils.cpp:923  constexpr const char *path_separators = "\\/";
const PATH_SEPARATORS: [u8; 2] = [b'\\', b'/'];

/// utils.cpp:925  bool is_separator(char c) { return c == '\\' || c == '/'; }
fn is_separator(c: u8) -> bool {
    c == b'\\' || c == b'/'
}

/// Where the user name and user id sit in a path under the user profile, e.g.
/// C:\Users\<name>\AppData\Roaming\BambuStudio\user\<id>\...
/// utils.cpp:927-937
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct UserPathLayout {
    /// user profile directory
    pub profile: String,
    /// first byte of the user name in the profile path (`std::string::npos` is `None`)
    pub name_pos: Option<usize>,
    pub name_size: usize,
    /// first byte of the user id folder
    pub id_pos: Option<usize>,
}

impl UserPathLayout {
    /// utils.cpp:936  bool valid() const { return name_pos != std::string::npos && name_size > 0; }
    pub fn valid(&self) -> bool {
        self.name_pos.is_some() && self.name_size > 0
    }

    /// The body of the lambda of utils.cpp:942-978 once the environment value is read: the
    /// layout of `env` (USERPROFILE on Windows, HOME elsewhere) with the length of the folder
    /// suffix of the release build (BBL_RELEASE_TO_PUBLIC, as slicer_cli builds it).
    pub fn from_profile(env: Option<&str>, suffix_len: usize) -> Self {
        let mut l = UserPathLayout::default();
        // utils.cpp:969-970  if (env == nullptr) return l;
        let env = match env {
            Some(e) => e,
            None => return l,
        };
        // utils.cpp:971  l.profile = env;
        l.profile = env.to_string();
        // utils.cpp:972-974  const size_t sep = l.profile.find_last_of(path_separators);
        let sep = match l.profile.bytes().rposition(|c| PATH_SEPARATORS.contains(&c)) {
            Some(sep) => sep,
            None => return l,
        };
        // utils.cpp:975-977
        l.name_pos = Some(sep + 1);
        l.name_size = l.profile.len() - (sep + 1);
        l.id_pos = Some(l.profile.len() + suffix_len + 1);
        l
    }
}

/// utils.cpp:939-981  const UserPathLayout &user_path_layout()
pub fn user_path_layout() -> &'static UserPathLayout {
    static LAYOUT: std::sync::OnceLock<UserPathLayout> = std::sync::OnceLock::new();
    LAYOUT.get_or_init(|| {
        // utils.cpp:944-967, the BBL_RELEASE_TO_PUBLIC branches.
        #[cfg(windows)]
        let (env, suffix_len) = (std::env::var("USERPROFILE").ok(), "\\AppData\\Roaming\\BambuStudio\\user".len());
        #[cfg(target_os = "macos")]
        let (env, suffix_len) = (std::env::var("HOME").ok(), "/Library/Application Support/BambuStudio/user".len());
        #[cfg(all(not(windows), not(target_os = "macos")))]
        let (env, suffix_len) = (std::env::var("HOME").ok(), "/.config/BambuStudio/user".len());
        UserPathLayout::from_profile(env.as_deref(), suffix_len)
    })
}

/// utils.cpp:982  bool is_utf8_continuation(char c) { return (static_cast<unsigned char>(c) & 0xC0) == 0x80; }
fn is_utf8_continuation(c: u8) -> bool {
    (c & 0xC0) == 0x80
}

/// Replace bytes [pos, pos + count) with '*', widened so that no UTF-8 character is cut in half.
/// utils.cpp:984-995
fn mask_bytes(s: &mut Vec<u8>, mut pos: usize, count: usize) {
    if pos >= s.len() {
        return;
    }
    let mut end = s.len().min(pos.saturating_add(count));
    while pos > 0 && is_utf8_continuation(s[pos]) {
        pos -= 1;
    }
    while end < s.len() && is_utf8_continuation(s[end]) {
        end += 1;
    }
    // s.replace(pos, end - pos, end - pos, '*');
    for c in &mut s[pos..end] {
        *c = b'*';
    }
}

/// utils.cpp:997-1001  std::string file_name_part(const std::string &path)
fn file_name_part(path: &[u8]) -> Vec<u8> {
    match path.iter().rposition(|c| PATH_SEPARATORS.contains(c)) {
        None => path.to_vec(),
        Some(sep) => path[sep + 1..].to_vec(),
    }
}

/// utils.cpp:1003-1007  bool is_upper_of(char upper, char lower)
/// `std::isupper` / `std::tolower` in the "C" locale: A-Z only.
fn is_upper_of(upper: u8, lower: u8) -> bool {
    upper.is_ascii_uppercase() && upper.to_ascii_lowercase() == lower
}

/// The argument types of the four `PathSanitizer::sanitize` overloads (Utils.hpp:141-144).
pub trait SanitizeArg {
    fn sanitize(self) -> String;
}

impl SanitizeArg for &str {
    // Utils.hpp:141  static std::string sanitize(const std::string &path) { return sanitize_impl(path); }
    fn sanitize(self) -> String {
        PathSanitizer::sanitize_impl(self.as_bytes().to_vec())
    }
}

impl SanitizeArg for String {
    // Utils.hpp:142  static std::string sanitize(std::string &&path) { return sanitize_impl(std::move(path)); }
    fn sanitize(self) -> String {
        PathSanitizer::sanitize_impl(self.into_bytes())
    }
}

impl SanitizeArg for Option<&str> {
    // Utils.hpp:143  static std::string sanitize(const char *path) { return path ? sanitize_impl(path) : std::string(); }
    fn sanitize(self) -> String {
        match self {
            Some(path) => PathSanitizer::sanitize_impl(path.as_bytes().to_vec()),
            None => String::new(),
        }
    }
}

impl SanitizeArg for &std::path::Path {
    // Utils.hpp:144  static std::string sanitize(const boost::filesystem::path &path) { return sanitize_impl(path.string()); }
    fn sanitize(self) -> String {
        PathSanitizer::sanitize_impl(self.to_string_lossy().into_owned().into_bytes())
    }
}

/// Utils.hpp:137-146
pub struct PathSanitizer;

impl PathSanitizer {
    /// Utils.hpp:141-144 — the four overloads, by argument type.
    pub fn sanitize<T: SanitizeArg>(path: T) -> String {
        path.sanitize()
    }

    /// utils.cpp:1011  std::string PathSanitizer::sanitize_impl(std::string raw)
    fn sanitize_impl(raw: Vec<u8>) -> String {
        // The bytes of a &str stay valid UTF-8: the masks never split a character and the file
        // name part starts after an ASCII separator.
        let out = Self::sanitize_with_layout(raw, user_path_layout());
        String::from_utf8(out).unwrap_or_else(|e| String::from_utf8_lossy(e.as_bytes()).into_owned())
    }

    /// The body of utils.cpp:1011-1050 on bytes, for a given layout (the C++ reads the static
    /// `user_path_layout()`); lets a test give the layout a generator used.
    pub fn sanitize_with_layout(mut raw: Vec<u8>, layout: &UserPathLayout) -> Vec<u8> {
        let out = |b: Vec<u8>| b;
        // utils.cpp:1013-1014  Bytes masked when the user name shows up in a path that is not under the user profile.
        const GUESSED_NAME_SIZE: usize = 12;

        // utils.cpp:1016-1018
        if !layout.valid() || raw.is_empty() || raw.len() < layout.profile.len() {
            return out(file_name_part(&raw));
        }
        let name_pos = layout.name_pos.unwrap();

        // utils.cpp:1020-1027
        let full = layout.profile.as_bytes();
        if raw[0] != full[0] || raw[full.len() - 1] != full[full.len() - 1] {
            if is_upper_of(raw[name_pos], full[name_pos]) {
                mask_bytes(&mut raw, name_pos, GUESSED_NAME_SIZE);
                return out(raw);
            }
            return out(file_name_part(&raw));
        }

        // utils.cpp:1029-1036
        let name_end = name_pos + layout.name_size;
        if name_end == raw.len() || is_separator(raw[name_end]) {
            mask_bytes(&mut raw, name_pos, layout.name_size);
        } else if is_upper_of(raw[name_pos], full[name_pos]) {
            mask_bytes(&mut raw, name_pos, GUESSED_NAME_SIZE);
        } else {
            return out(file_name_part(&raw));
        }

        // utils.cpp:1038-1044
        let id_pos = layout.id_pos.unwrap();
        if id_pos < raw.len() && is_separator(raw[id_pos - 1]) && raw[id_pos].is_ascii_digit() {
            let id_end = raw[id_pos..]
                .iter()
                .position(|c| PATH_SEPARATORS.contains(c))
                .map(|p| p + id_pos)
                .unwrap_or(raw.len());
            mask_bytes(&mut raw, id_pos, id_end - id_pos);
        }

        // utils.cpp:1046
        out(file_name_part(&raw))
    }
}

// Utils.hpp:295  Return dividend divided by divisor rounded to the nearest integer
#[inline]
pub fn round_divide(dividend: i64, divisor: i64) -> i64 {
    // Utils.hpp:297
    (dividend + divisor / 2) / divisor
}

// Utils.hpp:300  Return dividend divided by divisor rounded to the nearest integer
#[inline]
pub fn round_up_divide(dividend: i64, divisor: i64) -> i64 {
    // Utils.hpp:302
    (dividend + divisor - 1) / divisor
}

// Utils.hpp:306
pub fn get_max_element<T: Copy + PartialOrd + Default>(vec: &[T]) -> T {
    // Utils.hpp:309-310
    if vec.is_empty() {
        return T::default();
    }
    // Utils.hpp:312  *std::max_element(vec.begin(), vec.end())
    let mut best = vec[0];
    for &v in vec.iter().skip(1) {
        if best < v {
            best = v;
        }
    }
    best
}

// Utils.hpp:316  template <typename From, typename To> std::vector<To> convert_vector(const std::vector<From>&)
// Converts every element, throwing on out-of-range. Rust has no exceptions, so the
// faithful equivalent returns a Result: the C++ overflow/underflow/invalid_argument
// throws become Err(String) with the same messages and ordering of checks.
//
// `To: Bounded` carries the destination min/max plus the signedness flag, mirroring
// the `if constexpr (std::is_signed_v<To>)` split in the C++ source.
pub trait ConvertBound: Copy {
    const SIGNED: bool;
    fn min_f64() -> f64;
    fn max_f64() -> f64;
    fn from_f64(v: f64) -> Self;
}

// Source-side conversion to f64 for the range check. `i64 as f64` is lossy and so
// is NOT covered by std's `Into<f64>`; we provide an explicit widening accessor
// for every integer source type instead.
pub trait ConvertSource: Copy {
    fn as_f64(self) -> f64;
}

macro_rules! impl_convert_bound {
    ($t:ty, $signed:expr) => {
        impl ConvertBound for $t {
            const SIGNED: bool = $signed;
            fn min_f64() -> f64 {
                <$t>::MIN as f64
            }
            fn max_f64() -> f64 {
                <$t>::MAX as f64
            }
            fn from_f64(v: f64) -> Self {
                v as $t
            }
        }
        impl ConvertSource for $t {
            fn as_f64(self) -> f64 {
                self as f64
            }
        }
    };
}
impl_convert_bound!(i8, true);
impl_convert_bound!(i16, true);
impl_convert_bound!(i32, true);
impl_convert_bound!(i64, true);
impl_convert_bound!(u8, false);
impl_convert_bound!(u16, false);
impl_convert_bound!(u32, false);
impl_convert_bound!(u64, false);

pub fn convert_vector<From, To>(src: &[From]) -> Result<Vec<To>, String>
where
    From: ConvertSource,
    To: ConvertBound,
{
    // Utils.hpp:318-319  dst.reserve(src.size());
    let mut dst: Vec<To> = Vec::with_capacity(src.len());
    // Utils.hpp:320  for (const auto& elem : src)
    // FIDELITY-NOTE: C++ casts numeric_limits<To>::{min,max} back to `From` before
    // comparing; we compare via f64 (range check by value). Equivalent for the
    // integer From/To pairs callers use; differs only for extreme out-of-range
    // To-widths where the cast-to-From itself would overflow.
    for &elem in src {
        let ef: f64 = elem.as_f64();
        // Utils.hpp:321  if constexpr (std::is_signed_v<To>)
        if To::SIGNED {
            // Utils.hpp:322-324  elem > static_cast<From>(numeric_limits<To>::max())
            if ef > To::max_f64() {
                return Err("Source value exceeds destination maximum".to_string());
            }
            // Utils.hpp:325-327  elem < static_cast<From>(numeric_limits<To>::min())
            if ef < To::min_f64() {
                return Err("Source value below destination minimum".to_string());
            }
        } else {
            // Utils.hpp:329-332  if (elem < 0) throw invalid_argument(...)
            if ef < 0.0 {
                return Err("Negative value in source for unsigned destination".to_string());
            }
        }
        // Utils.hpp:334  dst.push_back(static_cast<To>(elem));
        dst.push(To::from_f64(ef));
    }
    // Utils.hpp:336  return dst;
    Ok(dst)
}

// ----------------------------------------------------------------------------
// next_highest_power_of_2 (Utils.hpp:463-515)
//
// The C++ source has overloads for u16/u32/u64 plus SFINAE size_t shims. Rust
// has no integer overloading; expose the u64 algorithm under a generic `usize`
// entry point used by callers (e.g. kd_tree_indirect) and explicit-width helpers
// matching each C++ overload bit-for-bit.
// ----------------------------------------------------------------------------

// Utils.hpp:463  inline uint16_t next_highest_power_of_2(uint16_t v)
#[inline]
pub fn next_highest_power_of_2_u16(mut v: u16) -> u16 {
    // Utils.hpp:466-467
    if v != 0 {
        v -= 1;
    }
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v.wrapping_add(1)
}

// Utils.hpp:473  inline uint32_t next_highest_power_of_2(uint32_t v)
#[inline]
pub fn next_highest_power_of_2_u32(mut v: u32) -> u32 {
    // Utils.hpp:475-476
    if v != 0 {
        v -= 1;
    }
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v.wrapping_add(1)
}

// Utils.hpp:484  inline uint64_t next_highest_power_of_2(uint64_t v)
#[inline]
pub fn next_highest_power_of_2_u64(mut v: u64) -> u64 {
    // Utils.hpp:486-487
    if v != 0 {
        v -= 1;
    }
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v |= v >> 32;
    v.wrapping_add(1)
}

// Utils.hpp:500-515  size_t next_highest_power_of_2(size_t v) -> dispatches to the
// 64-bit form (size_t is 64-bit on our targets, incl. wasm64; on wasm32 size_t
// is 32-bit so this mirrors the uint32 overload via the same bit operations).
#[inline]
pub fn next_highest_power_of_2(v: usize) -> usize {
    next_highest_power_of_2_u64(v as u64) as usize
}

// Utils.hpp:354-357  template<class VectorType> void reserve_more(VectorType &vector, size_t n)
// `std::vector::reserve(size() + n)`; `Vec::reserve` takes the additional count.
#[inline]
pub fn reserve_more<T>(vector: &mut Vec<T>, n: usize) {
    vector.reserve(n);
}

// Utils.hpp:359-362  template<class VectorType> void reserve_more_power_of_2(VectorType &vector, size_t n)
// `vector.reserve(next_highest_power_of_2(vector.size() + n))`
#[inline]
pub fn reserve_more_power_of_2<T>(vector: &mut Vec<T>, n: usize) {
    let target = next_highest_power_of_2(vector.len() + n);
    vector.reserve(target.saturating_sub(vector.len()));
}

// ----------------------------------------------------------------------------
// modulo index helpers (Utils.hpp:527-577)
// ----------------------------------------------------------------------------

// Utils.hpp:528
#[inline]
pub fn prev_idx_modulo(mut idx: usize, count: usize) -> usize {
    // Utils.hpp:530-531
    if idx == 0 {
        idx = count;
    }
    // Utils.hpp:532  return -- idx;
    idx - 1
}

// Utils.hpp:536
#[inline]
pub fn next_idx_modulo(mut idx: usize, count: usize) -> usize {
    // Utils.hpp:538-539  if (++ idx == count) idx = 0;
    idx += 1;
    if idx == count {
        idx = 0;
    }
    idx
}

// Utils.hpp:556  prev_value_modulo
#[inline]
pub fn prev_value_modulo<T>(idx: usize, container: &[T]) -> &T {
    &container[prev_idx_modulo(idx, container.len())]
}

// Utils.hpp:568  next_value_modulo
#[inline]
pub fn next_value_modulo<T>(idx: usize, container: &[T]) -> &T {
    &container[next_idx_modulo(idx, container.len())]
}

// ----------------------------------------------------------------------------
// Time formatting helpers (Utils.hpp inline)
// ----------------------------------------------------------------------------

// Utils.hpp:638  Shorten the dhms time by removing the seconds, rounding the dhm
// to full minutes and removing spaces.
pub fn short_time(time: &str) -> String {
    // Utils.hpp:641-645  Parse the dhms time format.
    let mut days: i32 = 0;
    let mut hours: i32 = 0;
    let mut minutes: i32 = 0;
    let mut seconds: i32 = 0;
    let mut f_seconds: f32 = 0.0;
    // Utils.hpp:646-655  branch on which leading unit is present.
    if time.contains('d') {
        // "%dd %dh %dm %ds"
        let v = sscanf_ints(time, 4);
        days = v[0];
        hours = v[1];
        minutes = v[2];
        seconds = v[3];
    } else if time.contains('h') {
        // "%dh %dm %ds"
        let v = sscanf_ints(time, 3);
        hours = v[0];
        minutes = v[1];
        seconds = v[2];
    } else if time.contains('m') {
        // "%dm %ds"
        let v = sscanf_ints(time, 2);
        minutes = v[0];
        seconds = v[1];
    } else if time.contains('s') {
        // "%fs"
        f_seconds = sscanf_first_float(time);
        seconds = f_seconds as i32;
    }
    // Utils.hpp:657-665  Round to full minutes.
    if days + hours > 0 && seconds >= 30 {
        minutes += 1;
        if minutes == 60 {
            minutes = 0;
            hours += 1;
            if hours == 24 {
                hours = 0;
                days += 1;
            }
        }
    }
    // Utils.hpp:667-681  Format the dhm time.
    if days > 0 {
        format!("{}d{}h{}m", days, hours, minutes)
    } else if hours > 0 {
        format!("{}h{}m", hours, minutes)
    } else if minutes > 0 {
        format!("{}m{}s", minutes, seconds)
    } else if seconds >= 1 {
        format!("{}s", seconds)
    } else if f_seconds > 0.0 && f_seconds < 1.0 {
        "<1s".to_string()
    } else if seconds == 0 {
        "0s".to_string()
    } else {
        String::new()
    }
}

// Utils.hpp:685  Returns the given time is seconds in format DDd HHh MMm SSs
pub fn get_time_dhms(mut time_in_secs: f32) -> String {
    // Utils.hpp:687-692
    let days = (time_in_secs / 86400.0f32) as i32;
    time_in_secs -= days as f32 * 86400.0f32;
    let hours = (time_in_secs / 3600.0f32) as i32;
    time_in_secs -= hours as f32 * 3600.0f32;
    let minutes = (time_in_secs / 60.0f32) as i32;
    time_in_secs -= minutes as f32 * 60.0f32;

    // Utils.hpp:694-704
    if days > 0 {
        format!("{}d {}h {}m {}s", days, hours, minutes, time_in_secs as i32)
    } else if hours > 0 {
        format!("{}h {}m {}s", hours, minutes, time_in_secs as i32)
    } else if minutes > 0 {
        format!("{}m {}s", minutes, time_in_secs as i32)
    } else if time_in_secs > 1.0 {
        format!("{}s", time_in_secs as i32)
    } else {
        // ::sprintf(buffer, "%fs", time_in_secs);  default %f -> 6 decimals
        format!("{:.6}s", time_in_secs)
    }
}

// Utils.hpp:709
pub fn get_bbl_time_dhms(mut time_in_secs: f32) -> String {
    // Utils.hpp:711-716
    let days = (time_in_secs / 86400.0f32) as i32;
    time_in_secs -= days as f32 * 86400.0f32;
    let hours = (time_in_secs / 3600.0f32) as i32;
    time_in_secs -= hours as f32 * 3600.0f32;
    let minutes = (time_in_secs / 60.0f32) as i32;
    time_in_secs -= minutes as f32 * 60.0f32;

    // Utils.hpp:718-726
    if days > 0 {
        format!("{}d{}h{}m{}s", days, hours, minutes, time_in_secs as i32)
    } else if hours > 0 {
        format!("{}h{}m{}s", hours, minutes, time_in_secs as i32)
    } else if minutes > 0 {
        format!("{}m{}s", minutes, time_in_secs as i32)
    } else {
        format!("{}s", time_in_secs as i32)
    }
}

// Utils.hpp:568-595  inline long get_local_utc_offset_seconds(std::time_t instant)
// Utils.hpp:597      inline long get_local_utc_offset_seconds() { return get_local_utc_offset_seconds(std::time(nullptr)); }
// (`None` is the second overload, the current time.)
//
// Current UTC offset of the system time zone, in seconds east of UTC, daylight saving included.
// The C++ breaks the instant down as UTC (gmtime_r) and as local time (localtime_r), pins
// tm_isdst = 0 on both and diffs their mktime() readings: both wall clocks are then read with the
// same standard offset, which cancels, and the difference is the local wall clock minus the UTC
// wall clock of the instant, i.e. its full offset. chrono gives that difference of wall clocks
// directly; it returns 0 where the C runtime calls fail (an instant chrono cannot place).
pub fn get_local_utc_offset_seconds(instant: Option<i64>) -> i64 {
    // Utils.hpp:597  std::time(nullptr)
    let instant = instant.unwrap_or_else(|| chrono::Utc::now().timestamp());
    // Utils.hpp:570-578  gmtime / localtime of the instant
    let utc_tm = match chrono::DateTime::from_timestamp(instant, 0) {
        Some(t) => t.naive_utc(),
        None => return 0,
    };
    let local_tm = match chrono::TimeZone::timestamp_opt(&chrono::Local, instant, 0).single() {
        Some(t) => t.naive_local(),
        None => return 0,
    };
    // Utils.hpp:584-594  (long) std::difftime(local_as_local, utc_as_local)
    (local_tm - utc_tm).num_seconds()
}

// Utils.hpp:599
pub fn get_timezone_utc_hm(mut second: i64) -> String {
    // Utils.hpp:733-737
    let mut pos = true;
    if second < 0 {
        pos = false;
        second = -second;
    }

    // Utils.hpp:739-742
    let hours = (second as f32 / 3600.0f32) as i32;
    second -= (hours as f32 * 3600.0f32) as i64;
    let minutes = (second as f32 / 60.0f32) as i32;
    second -= (minutes as f32 * 60.0f32) as i64;
    let _ = second;

    // Utils.hpp:744-745  ::sprintf(buffer, "UTC%s%02d:%02d", pos ? "+" : "-", hours, minutes);
    format!("UTC{}{:02}:{:02}", if pos { "+" } else { "-" }, hours, minutes)
}

// Utils.hpp:749
pub fn get_time_dhm(mut time_in_secs: f32) -> String {
    // Utils.hpp:751-755
    let days = (time_in_secs / 86400.0f32) as i32;
    time_in_secs -= days as f32 * 86400.0f32;
    let hours = (time_in_secs / 3600.0f32) as i32;
    time_in_secs -= hours as f32 * 3600.0f32;
    let minutes = (time_in_secs / 60.0f32) as i32;

    // Utils.hpp:757-765
    if days > 0 {
        format!("{}d {}h {}m", days, hours, minutes)
    } else if hours > 0 {
        format!("{}h {}m", hours, minutes)
    } else if minutes > 0 {
        format!("{}m", minutes)
    } else {
        format!("{}m", 0)
    }
}

// Utils.hpp:770
pub fn get_time_hms(mut time_in_secs: f32) -> String {
    // Utils.hpp:772-776
    let hours = (time_in_secs / 3600.0f32) as i32;
    time_in_secs -= hours as f32 * 3600.0f32;
    let minutes = (time_in_secs / 60.0f32) as i32;
    time_in_secs -= minutes as f32 * 60.0f32;
    let secs = time_in_secs as i32;

    // Utils.hpp:778-779  ::sprintf(buffer, "%02d:%02d:%02d", hours, minutes, secs);
    format!("{:02}:{:02}:{:02}", hours, minutes, secs)
}

// Utils.hpp:783
pub fn get_bbl_monitor_time_dhm(mut time_in_secs: f32) -> String {
    // Utils.hpp:785-789
    let days = (time_in_secs / 86400.0f32) as i32;
    time_in_secs -= days as f32 * 86400.0f32;
    let hours = (time_in_secs / 3600.0f32) as i32;
    time_in_secs -= hours as f32 * 3600.0f32;
    let minutes = (time_in_secs / 60.0f32).ceil() as i32;

    // Utils.hpp:791-800
    if days > 0 {
        format!("{}d{}h{}min", days, hours, minutes)
    } else if hours > 0 {
        format!("{}h{}min", hours, minutes)
    } else if minutes >= 0 {
        format!("{}min", minutes)
    } else {
        String::new()
    }
}

// Utils.hpp:672-692  Centralized function to format time from a std::tm structure.
// This is the single source of truth for time formatting throughout the application.
//
// (hour, min) replace the relevant std::tm fields (tm_hour, tm_min).
pub fn format_time_hm(tm_hour: i32, tm_min: i32, use_12h_format: bool) -> String {
    // Utils.hpp:811-822
    if use_12h_format {
        let hour = tm_hour;
        let suffix = if hour >= 12 { "PM" } else { "AM" };
        let mut display_hour = hour % 12;
        if display_hour == 0 {
            display_hour = 12; // Midnight = 12AM, Noon = 12PM
        }
        format!("{:02}:{:02}{}", display_hour, tm_min, suffix)
    } else {
        // 24-hour format
        format!("{:02}:{:02}", tm_hour, tm_min)
    }
}

// Utils.hpp:695-699
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct BBLFinishTime {
    pub time: String,
    pub day_offset: i32,
}

// Utils.hpp:701-738  inline BBLFinishTime get_bbl_finish_time(float time_in_secs, bool use_12h_format = false)
// std::time / std::localtime are read through chrono's local time zone.
pub fn get_bbl_finish_time(time_in_secs: f32, use_12h_format: bool) -> BBLFinishTime {
    // Utils.hpp:703
    if time_in_secs < 1.0 {
        return BBLFinishTime::default();
    }

    // Utils.hpp:705-710  Get current time first
    let current_time = chrono::Local::now().timestamp();
    let current_tm = match chrono::TimeZone::timestamp_opt(&chrono::Local, current_time, 0).single() {
        Some(tm) => tm,
        None => return BBLFinishTime::default(),
    };
    // tm_yday is 0-based, chrono's ordinal 1-based.
    let current_day: i32 = current_tm.ordinal() as i32 - 1;
    let current_year: i32 = current_tm.year();

    // Utils.hpp:712-718  Calculate finish time and get its local time
    let finish_time = current_time + time_in_secs as i64;
    let finish_tm = match chrono::TimeZone::timestamp_opt(&chrono::Local, finish_time, 0).single() {
        Some(tm) => tm,
        None => return BBLFinishTime::default(),
    };
    let finish_day: i32 = finish_tm.ordinal() as i32 - 1;
    let finish_year: i32 = finish_tm.year();

    // Utils.hpp:720-735
    let mut diff_day;
    if current_year != finish_year {
        if (current_year % 4 == 0 && current_year % 100 != 0) || current_year % 400 == 0 {
            diff_day = 366 - current_day;
        } else {
            diff_day = 365 - current_day;
        }
        // The loop tests `current_year`, not `year`, as the C++ does.
        let mut year = current_year + 1;
        while year < finish_year {
            if (current_year % 4 == 0 && current_year % 100 != 0) || current_year % 400 == 0 {
                diff_day += 366;
            } else {
                diff_day += 365;
            }
            year += 1;
        }
        diff_day += finish_day;
    } else {
        diff_day = finish_day - current_day;
    }

    // Utils.hpp:737  return {format_time_hm(&finish_tm, use_12h_format), diff_day};
    BBLFinishTime {
        time: format_time_hm(finish_tm.hour() as i32, finish_tm.minute() as i32, use_12h_format),
        day_offset: diff_day,
    }
}

// Utils.hpp:740-746  inline std::string get_bbl_finish_time_dhm(float time_in_secs, bool use_12h_format = false)
pub fn get_bbl_finish_time_dhm(time_in_secs: f32, use_12h_format: bool) -> String {
    // Utils.hpp:742
    if time_in_secs < 1.0 {
        return "Finished".to_string();
    }

    // Utils.hpp:744-745
    let finish_time = get_bbl_finish_time(time_in_secs, use_12h_format);
    finish_time.time
        + &(if finish_time.day_offset == 0 { String::new() } else { format!("+{}", finish_time.day_offset) })
}

// Utils.hpp:879
pub fn get_bbl_remain_time_dhms(mut time_in_secs: f32) -> String {
    // Utils.hpp:881-886
    let days = (time_in_secs / 86400.0f32) as i32;
    time_in_secs -= days as f32 * 86400.0f32;
    let hours = (time_in_secs / 3600.0f32) as i32;
    time_in_secs -= hours as f32 * 3600.0f32;
    let minutes = (time_in_secs / 60.0f32).ceil() as i32;
    time_in_secs -= minutes as f32 * 60.0f32;

    // Utils.hpp:888-897
    if days > 0 {
        format!("{}d{}h{}m{}s", days, hours, minutes, time_in_secs as i32)
    } else if hours > 0 {
        format!("{}h{}m{}s", hours, minutes, time_in_secs as i32)
    } else if minutes > 0 {
        format!("{}m{}s", minutes, time_in_secs as i32)
    } else {
        format!("{}s", time_in_secs as i32)
    }
}

// Utils.hpp:903
pub fn filter_characters(s: &str, filter_chars: &str) -> String {
    // Utils.hpp:905-911  erase-remove_if of every char present in filterChars.
    s.chars().filter(|ch| !filter_chars.contains(*ch)).collect()
}

// ============================================================================
// Utils.cpp definitions
// ============================================================================

// Utils.cpp:93  static logSeverity, modelled as the boost severity index 0..5.
// 0=fatal 1=error 2=warning 3=info 4=debug 5=trace ; default error (1).
static LOG_SEVERITY: Mutex<u32> = Mutex::new(1);

// Utils.cpp:95-111  static boost::log::trivial::severity_level level_to_boost(unsigned)
// Returns the severity index that the boost severity level maps to. We carry the
// numeric severity directly (boost's trivial levels are an ordered enum 0..5).
fn level_to_boost(level: u32) -> u32 {
    // Utils.cpp:97-110
    match level {
        // Report fatal errors only.
        0 => 0, // fatal
        // Report fatal errors and errors.
        1 => 1, // error
        // Report fatal errors, errors and warnings.
        2 => 2, // warning
        // Report all errors, warnings and infos.
        3 => 3, // info
        // Report all errors, warnings, infos and debugging.
        4 => 4, // debug
        // Report everyting including fine level tracing information.
        _ => 5, // trace
    }
}

// Utils.cpp:113  void set_logging_level(unsigned int level)
pub fn set_logging_level(level: u32) {
    // Utils.cpp:115  logSeverity = level_to_boost(level);
    *LOG_SEVERITY.lock().unwrap() = level_to_boost(level);
    // Utils.cpp:117-120  boost::log::core::get()->set_filter(...): boost-log only,
    // no faithful equivalent without the boost::log backend (native dep).
}

// Utils.cpp:123  unsigned int level_string_to_boost(std::string level)
pub fn level_string_to_boost(level: &str) -> u32 {
    // Utils.cpp:125-133  std::map<std::string,int> default 0 for unknown keys.
    match level {
        "fatal" => 0,
        "error" => 1,
        "warning" => 2,
        "info" => 3,
        "debug" => 4,
        "trace" => 5,
        // std::map operator[] inserts a value-initialized int (0) for missing keys.
        _ => 0,
    }
}

// Utils.cpp:136  std::string get_string_logging_level(unsigned level)
pub fn get_string_logging_level(level: u32) -> String {
    // Utils.cpp:138-146
    match level {
        0 => "fatal".to_string(),
        1 => "error".to_string(),
        2 => "warning".to_string(),
        3 => "info".to_string(),
        4 => "debug".to_string(),
        5 => "trace".to_string(),
        _ => "error".to_string(),
    }
}

// Utils.cpp:149  unsigned get_logging_level()
pub fn get_logging_level() -> u32 {
    // Utils.cpp:151-159  map boost severity back to 0..5 index.
    match *LOG_SEVERITY.lock().unwrap() {
        0 => 0, // fatal
        1 => 1, // error
        2 => 2, // warning
        3 => 3, // info
        4 => 4, // debug
        5 => 5, // trace
        _ => 1,
    }
}

// Utils.cpp:172  void trace(unsigned int level, const char *message)
// BLOCKED: emits through boost::log trivial logger; no faithful 1:1 without the
// boost::log backend. Provided as a no-op to keep the symbol; see notes.
pub fn trace(_level: u32, _message: &str) {
    // Utils.cpp:174-177  boost::log severity dispatch — native logging backend only.
}

// Utils.cpp:180  void disable_multi_threading()
// BLOCKED: TBB global_control / task_scheduler_init — native threading backend.
pub fn disable_multi_threading() {
    // Utils.cpp:182-188  Disable parallelization so the Shiny profiler works.
}

// Utils.cpp:191  static std::string g_var_dir;
static G_VAR_DIR: Mutex<String> = Mutex::new(String::new());

// Utils.cpp:193  void set_var_dir(const std::string &dir)
pub fn set_var_dir(dir: &str) {
    // Utils.cpp:195  g_var_dir = dir;
    *G_VAR_DIR.lock().unwrap() = dir.to_string();
}

// Utils.cpp:198  const std::string& var_dir()
pub fn var_dir() -> String {
    // Utils.cpp:200  return g_var_dir;
    G_VAR_DIR.lock().unwrap().clone()
}

// Utils.cpp:203  std::string var(const std::string &file_name)
pub fn var(file_name: &str) -> String {
    // Utils.cpp:205-208  if (boost::filesystem::exists(file_name)) return file_name;
    if std::path::Path::new(file_name).exists() {
        return file_name.to_string();
    }
    // Utils.cpp:210-211  auto file = (path(g_var_dir) / file_name).make_preferred();
    let file = std::path::Path::new(&*G_VAR_DIR.lock().unwrap()).join(file_name);
    file.to_string_lossy().into_owned()
}

// Utils.cpp:214  static std::string g_resources_dir;
static G_RESOURCES_DIR: Mutex<String> = Mutex::new(String::new());

// Utils.cpp:216  void set_resources_dir(const std::string &dir)
pub fn set_resources_dir(dir: &str) {
    // Utils.cpp:218  g_resources_dir = dir;
    *G_RESOURCES_DIR.lock().unwrap() = dir.to_string();
}

// Utils.cpp:221  const std::string& resources_dir()
// Returns a PathBuf for caller ergonomics (flush_vol_predictor pushes onto it);
// the stored value is g_resources_dir, faithfully empty until set.
pub fn resources_dir() -> std::path::PathBuf {
    // Utils.cpp:223  return g_resources_dir;
    std::path::PathBuf::from(G_RESOURCES_DIR.lock().unwrap().clone())
}

// Utils.cpp:227  static std::string g_temporary_dir;
static G_TEMPORARY_DIR: Mutex<String> = Mutex::new(String::new());

// Utils.cpp:228  void set_temporary_dir(const std::string &dir)
pub fn set_temporary_dir(dir: &str) {
    // Utils.cpp:230  g_temporary_dir = dir;
    *G_TEMPORARY_DIR.lock().unwrap() = dir.to_string();
}

// Utils.cpp:233  const std::string& temporary_dir()
pub fn temporary_dir() -> String {
    // Utils.cpp:235  return g_temporary_dir;
    G_TEMPORARY_DIR.lock().unwrap().clone()
}

// Utils.cpp:238  static std::string g_local_dir;
static G_LOCAL_DIR: Mutex<String> = Mutex::new(String::new());

// Utils.cpp:240  void set_local_dir(const std::string &dir)
pub fn set_local_dir(dir: &str) {
    // Utils.cpp:242  g_local_dir = dir;
    *G_LOCAL_DIR.lock().unwrap() = dir.to_string();
}

// Utils.cpp:245  const std::string& localization_dir()
pub fn localization_dir() -> String {
    // Utils.cpp:247  return g_local_dir;
    G_LOCAL_DIR.lock().unwrap().clone()
}

// Utils.cpp:250  static std::string g_sys_shapes_dir;
static G_SYS_SHAPES_DIR: Mutex<String> = Mutex::new(String::new());

// Utils.cpp:252  void set_sys_shapes_dir(const std::string &dir)
pub fn set_sys_shapes_dir(dir: &str) {
    // Utils.cpp:254  g_sys_shapes_dir = dir;
    *G_SYS_SHAPES_DIR.lock().unwrap() = dir.to_string();
}

// Utils.cpp:257  const std::string& sys_shapes_dir()
pub fn sys_shapes_dir() -> String {
    // Utils.cpp:259  return g_sys_shapes_dir;
    G_SYS_SHAPES_DIR.lock().unwrap().clone()
}

// Utils.cpp:276  std::string custom_shapes_dir()
pub fn custom_shapes_dir() -> String {
    // Utils.cpp:278  return (path(g_data_dir) / "shapes").string();
    std::path::Path::new(&data_dir())
        .join("shapes")
        .to_string_lossy()
        .into_owned()
}

// Utils.cpp:281  static std::atomic<bool> debug_out_path_called(false);
static DEBUG_OUT_PATH_CALLED: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

// Utils.cpp:283  std::string debug_out_path(const char *name, ...)
// The C++ takes printf-style varargs; the Rust port takes the already-formatted
// name (callers format their own string), since Rust has no C varargs. The
// directory bookkeeping (create g_data_dir/SVG, optionally a subdir) is preserved.
pub fn debug_out_path(name: &str) -> String {
    // Utils.cpp:286  auto svg_folder = boost::filesystem::path(g_data_dir) / "SVG/";
    let svg_folder = std::path::Path::new(&data_dir()).join("SVG/");
    // Utils.cpp:287-293  if (! debug_out_path_called.exchange(true)) { ... }
    if !DEBUG_OUT_PATH_CALLED.swap(true, std::sync::atomic::Ordering::SeqCst) {
        // Utils.cpp:288-290
        if !svg_folder.exists() {
            let _ = std::fs::create_dir(&svg_folder);
        }
        // Utils.cpp:291-292  print the absolute output directory.
        let path = std::fs::canonicalize(&svg_folder)
            .unwrap_or_else(|_| svg_folder.clone())
            .to_string_lossy()
            .into_owned();
        println!("Debugging output files will be written to {}", path);
    }
    // Utils.cpp:294-300  format the name into buf (already-formatted here).
    let buf = name.to_string();
    // Utils.cpp:301-304  if a '/' is present, create the leading sub directory.
    let svg_folder_str = svg_folder.to_string_lossy().into_owned();
    if let Some(pos) = buf.find('/') {
        let sub_dir = &buf[0..pos];
        let _ = std::fs::create_dir(format!("{}{}", svg_folder_str, sub_dir));
    }
    // Utils.cpp:305  return svg_folder.string() + std::string(buffer);
    format!("{}{}", svg_folder_str, buf)
}

// Utils.cpp:310  bool is_log_trivival_valid()
// BLOCKED: tracks the boost::log sink pointer g_log_sink. No boost::log backend
// is ported (native dep), so faithfully reports false (no sink installed).
pub fn is_log_trivival_valid() -> bool {
    // Utils.cpp:312  return g_log_sink != nullptr;
    false
}

// Utils.cpp:320  void set_log_path_and_level(const std::string& file, unsigned int level, const LogEncOptions&)
// BLOCKED: installs a boost::log file sink (g_log_sink / LogSinkBackend) — native
// logging backend, not wasm-safe. Faithfully applies only the level (Utils.cpp:342).
pub fn set_log_path_and_level(_file: &str, level: u32, _enc_options: &LogEncOptions) {
    // Utils.cpp:330-340  create the LogSink backend + sink: boost::log only.
    // Utils.cpp:342  set_logging_level(level);
    set_logging_level(level);
}

// Utils.cpp:346  void update_log_sink(const std::string& file, const LogEncOptions& enc_options)
// BLOCKED: forwards to g_log_sink_backend->update_enc_option — boost::log backend.
pub fn update_log_sink(_file: &str, _enc_options: &LogEncOptions) {
    // Utils.cpp:348-350  if (g_log_sink_backend) g_log_sink_backend->update_enc_option(...);
}

// Utils.cpp:353  void flush_logs()
// BLOCKED: flushes the boost::log sink g_log_sink — boost::log backend.
pub fn flush_logs() {
    // Utils.cpp:355-356  if (g_log_sink) g_log_sink->flush();
}

// Utils.cpp:610  std::error_code rename_file(const std::string &from, const std::string &to)
// On non-Windows: remove(to); rename(from, to). Returns true on success.
pub fn rename_file(from: &str, to: &str) -> std::io::Result<()> {
    // Utils.cpp:615  boost::nowide::remove(to.c_str());
    let _ = std::fs::remove_file(to);
    // Utils.cpp:616  boost::nowide::rename(from.c_str(), to.c_str());
    std::fs::rename(from, to)
}

// Utils.cpp:814  CopyFileResult copy_file_inner(from, to, error_message)
pub fn copy_file_inner(from: &str, to: &str, error_message: &mut String) -> CopyFileResult {
    // Utils.cpp:816-848  Non-Windows path (boost::filesystem::copy_file with
    // overwrite_if_exists). Permission fiddling is best-effort and ignored.
    match std::fs::copy(from, to) {
        Ok(_) => CopyFileResult::Success,
        Err(e) => {
            // Utils.cpp:839  error_message = ec.message();
            *error_message = e.to_string();
            CopyFileResult::FailCopyFile
        }
    }
}

// Utils.cpp:851  CopyFileResult copy_file(from, to, error_message, with_check = false)
pub fn copy_file(
    from: &str,
    to: &str,
    error_message: &mut String,
    with_check: bool,
) -> CopyFileResult {
    // Utils.cpp:935  std::string to_temp = to + ".tmp";
    let to_temp = format!("{}.tmp", to);
    // Utils.cpp:936  copy_file_inner(from, to_temp, error_message);
    let mut ret_val = copy_file_inner(from, &to_temp, error_message);
    // Utils.cpp:937
    if ret_val == CopyFileResult::Success {
        // Utils.cpp:939-940  if (with_check) ret_val = check_copy(from, to_temp);
        if with_check {
            ret_val = check_copy(from, &to_temp);
        }
        // Utils.cpp:942-943  if (ret_val == 0 && rename_file(to_temp, to)) ret_val = FAIL_RENAMING;
        if ret_val == CopyFileResult::Success && rename_file(&to_temp, to).is_err() {
            ret_val = CopyFileResult::FailRenaming;
        }
    }
    // Utils.cpp:945
    ret_val
}

// Utils.cpp:949  bool copy_framework(const std::string &from, const std::string &to)
pub fn copy_framework(from: &str, to: &str) -> bool {
    // Utils.cpp:951  boost::filesystem::path src(from), dst(to);
    let src = std::path::Path::new(from);
    let dst = std::path::Path::new(to);
    // Utils.cpp:952-973  try { ... } catch (filesystem_error) { log; }
    // Utils.cpp:953-956  if (!is_directory(src)) { cerr; return false; }
    if !src.is_dir() {
        eprintln!("Error: Source is not a directory: {}", src.display());
        return false;
    }
    // Utils.cpp:957  boost::filesystem::create_directories(dst);
    if std::fs::create_dir_all(dst).is_err() {
        // Utils.cpp:971-973  filesystem_error -> log error, fall through to return false.
        return false;
    }
    // Utils.cpp:958  for (directory_iterator it(src); ...; ++it)
    let entries = match std::fs::read_dir(src) {
        Ok(e) => e,
        Err(_) => return false,
    };
    for it in entries {
        let entry = match it {
            Ok(e) => e.path(),
            Err(_) => return false,
        };
        // Utils.cpp:959-960  const auto &entry = it->path(); dest_path = dst / entry.filename();
        let dest_path = dst.join(entry.file_name().unwrap_or_default());
        // Utils.cpp:962-963  if (is_symlink(entry)) copy_symlink(entry, dest_path);
        let meta = match std::fs::symlink_metadata(&entry) {
            Ok(m) => m,
            Err(_) => return false,
        };
        if meta.file_type().is_symlink() {
            #[cfg(unix)]
            {
                let target = match std::fs::read_link(&entry) {
                    Ok(t) => t,
                    Err(_) => return false,
                };
                if std::os::unix::fs::symlink(target, &dest_path).is_err() {
                    return false;
                }
            }
            #[cfg(not(unix))]
            {
                // boost::filesystem::copy_symlink on non-unix; best-effort copy.
                if std::fs::copy(&entry, &dest_path).is_err() {
                    return false;
                }
            }
        } else if entry.is_dir() {
            // Utils.cpp:964-965  else if (is_directory(entry)) copy_framework(...);
            if !copy_framework(&entry.to_string_lossy(), &dest_path.to_string_lossy()) {
                return false;
            }
        } else {
            // Utils.cpp:966-967  else copy(entry, dest_path, overwrite_existing);
            if std::fs::copy(&entry, &dest_path).is_err() {
                return false;
            }
        }
    }
    // Utils.cpp:970  return true;
    true
}

// Utils.cpp:977  CopyFileResult check_copy(const std::string& origin, const std::string& copy)
pub fn check_copy(origin: &str, copy: &str) -> CopyFileResult {
    // Utils.cpp:979-985  open both in binary; report which failed to open.
    let buf_origin = match std::fs::read(origin) {
        Ok(b) => b,
        Err(_) => return CopyFileResult::FailCheckOriginNotOpened,
    };
    let buf_copy = match std::fs::read(copy) {
        Ok(b) => b,
        Err(_) => return CopyFileResult::FailCheckTargetNotOpened,
    };
    // Utils.cpp:987-1011  compare sizes then contents.
    if buf_origin.len() != buf_copy.len() {
        return CopyFileResult::FailFilesDifferent;
    }
    if buf_origin != buf_copy {
        return CopyFileResult::FailFilesDifferent;
    }
    // Utils.cpp:1011  All data read and compared equal.
    CopyFileResult::Success
}

// Utils.cpp:1015  bool is_plain_file(const boost::filesystem::directory_entry &dir_entry)
// Ignore system and hidden files, which may be created by the DropBox synchronisation process.
pub fn is_plain_file(dir_entry: &std::fs::DirEntry) -> bool {
    // Utils.cpp:1017-1018  if (! is_regular_file(status())) return false;
    match dir_entry.metadata() {
        Ok(m) => {
            if !m.is_file() {
                return false;
            }
        }
        Err(_) => return false,
    }
    // Utils.cpp:1019-1024  on non-MSVC: return true (no hidden/system attribute check).
    true
}

// Utils.cpp:1027  bool is_ini_file(const boost::filesystem::directory_entry &dir_entry)
pub fn is_ini_file(dir_entry: &std::fs::DirEntry) -> bool {
    // Utils.cpp:1029  is_plain_file(...) && strcasecmp(extension, ".ini") == 0
    is_plain_file(dir_entry) && strcasecmp(&dir_entry_extension(dir_entry), ".ini") == 0
}

// Utils.cpp:1032  bool is_idx_file(const boost::filesystem::directory_entry &dir_entry)
pub fn is_idx_file(dir_entry: &std::fs::DirEntry) -> bool {
    // Utils.cpp:1034  is_plain_file(...) && strcasecmp(extension, ".idx") == 0
    is_plain_file(dir_entry) && strcasecmp(&dir_entry_extension(dir_entry), ".idx") == 0
}

// Utils.cpp:1038  bool is_gcode_file(const std::string &path)
pub fn is_gcode_file(path: &str) -> bool {
    // Utils.cpp:1040  boost::iends_with(path, ".gcode");
    iends_with(path, ".gcode")
}

// Utils.cpp:1044  bool is_json_file(const std::string& path)
pub fn is_json_file(path: &str) -> bool {
    // Utils.cpp:1046  boost::iends_with(path, ".json");
    iends_with(path, ".json")
}

// Utils.cpp:1049  bool is_img_file(const std::string &path)
pub fn is_img_file(path: &str) -> bool {
    // Utils.cpp:1051  iends_with(path, ".png") || iends_with(path, ".svg");
    iends_with(path, ".png") || iends_with(path, ".svg")
}

// Utils.cpp:1054  bool is_gallery_file(const boost::filesystem::directory_entry& dir_entry, char const* type)
pub fn is_gallery_file_entry(dir_entry: &std::fs::DirEntry, ty: &str) -> bool {
    // Utils.cpp:1056  is_plain_file(...) && strcasecmp(extension, type) == 0
    is_plain_file(dir_entry) && strcasecmp(&dir_entry_extension(dir_entry), ty) == 0
}

// Utils.cpp:1059  bool is_gallery_file(const std::string &path, char const* type)
pub fn is_gallery_file(path: &str, ty: &str) -> bool {
    // Utils.cpp:1061  boost::iends_with(path, type);
    iends_with(path, ty)
}

// Utils.cpp:1064  bool is_shapes_dir(const std::string& dir)
pub fn is_shapes_dir(dir: &str) -> bool {
    // Utils.cpp:1066  dir == sys_shapes_dir() || dir == custom_shapes_dir();
    dir == sys_shapes_dir() || dir == custom_shapes_dir()
}

// Utils.cpp:1081  std::string encode_path(const char *src)
// On OSX/Linux this is a no-op identity copy (only Windows converts code page).
pub fn encode_path(src: &str) -> String {
    // Utils.cpp:1094  return src;
    src.to_string()
}

// Utils.cpp:1100  std::string decode_path(const char *src)
// On OSX/Linux this is a no-op identity copy.
pub fn decode_path(src: &str) -> String {
    // Utils.cpp:1113  return src;
    src.to_string()
}

// Utils.cpp:1117  std::string normalize_utf8_nfc(const char *src)
// BLOCKED for true NFC normalization: boost::locale::normalize(norm_nfc) needs
// an ICU/locale backend (native, not wasm-safe). Faithfully returns the input
// unchanged, which matches NFC for already-normalized ASCII/UTF-8 inputs.
pub fn normalize_utf8_nfc(src: &str) -> String {
    // Utils.cpp:1119-1120
    src.to_string()
}

// utils.cpp:1102  std::vector<std::string> split_string(const std::string &str, char delimiter)
pub fn split_string(s: &str, delimiter: char) -> Vec<String> {
    // utils.cpp:1104  std::vector<std::string> result;
    let mut result: Vec<String> = Vec::new();
    // utils.cpp:1105  std::stringstream ss(str);
    let mut ss = s.chars().peekable();
    // utils.cpp:1106  std::string substr;
    let mut substr = String::new();

    // utils.cpp:1108-1110  while (std::getline(ss, substr, delimiter)) result.push_back(substr);
    // std::getline clears `substr`, extracts up to the delimiter (which it consumes and does not
    // store) or the end of the stream, and fails only when it extracted nothing at the end of the
    // stream: "a,b," gives "a" and "b", no trailing empty string, and "" gives nothing.
    loop {
        substr.clear();
        if ss.peek().is_none() {
            break;
        }
        for ch in ss.by_ref() {
            if ch == delimiter {
                break;
            }
            substr.push(ch);
        }
        result.push(substr.clone());
    }
    // utils.cpp:1111  return result;
    result
}

// Utils.cpp:1135  namespace PerlUtils
pub mod perl_utils {
    use std::path::Path;

    // Utils.cpp:1137  Get a file name including the extension.
    pub fn path_to_filename(src: &str) -> String {
        Path::new(src)
            .file_name()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default()
    }

    // Utils.cpp:1139  Get a file name without the extension.
    pub fn path_to_stem(src: &str) -> String {
        Path::new(src)
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default()
    }

    // Utils.cpp:1141  Get just the extension.
    pub fn path_to_extension(src: &str) -> String {
        // boost::filesystem extension() includes the leading dot.
        match Path::new(src).extension() {
            Some(ext) => format!(".{}", ext.to_string_lossy()),
            None => String::new(),
        }
    }

    // Utils.cpp:1143  Get a directory without the trailing slash.
    pub fn path_to_parent_path(src: &str) -> String {
        Path::new(src)
            .parent()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default()
    }
}

// Utils.cpp:1147  std::string string_printf(const char *format, ...)
// The C++ takes printf-style varargs and runs vsnprintf into a growing buffer.
// Rust has no C varargs, so callers pre-format their string; this faithfully
// returns that string (the vsnprintf round-trip is identity for the result).
pub fn string_printf(formatted: &str) -> String {
    // Utils.cpp:1149-1166  vsnprintf into INITIAL_LEN buffer, grow if truncated.
    formatted.to_string()
}

// libslic3r_version.h:4  #define SLIC3R_APP_NAME "BambuStudio"  (from version.inc)
pub const SLIC3R_APP_NAME: &str = "BambuStudio";
// libslic3r.h:6  #define GCODEVIEWER_APP_NAME "BambuStudio G-code Viewer"
pub const GCODEVIEWER_APP_NAME: &str = "BambuStudio G-code Viewer";

// Utils.cpp:1169  std::string header_slic3r_generated()
pub fn header_slic3r_generated() -> String {
    // Utils.cpp:1171  return SLIC3R_APP_NAME " " SLIC3R_VERSION;
    format!("{} {}", SLIC3R_APP_NAME, crate::semver::SLIC3R_VERSION)
}

// Utils.cpp:1174  std::string header_gcodeviewer_generated()
pub fn header_gcodeviewer_generated() -> String {
    // Utils.cpp:1176  return GCODEVIEWER_APP_NAME " " SLIC3R_VERSION;
    format!("{} {}", GCODEVIEWER_APP_NAME, crate::semver::SLIC3R_VERSION)
}

// Utils.cpp:1179  unsigned get_current_pid()
pub fn get_current_pid() -> u32 {
    // Utils.cpp:1184  return ::getpid();
    std::process::id()
}

#[cfg(target_os = "macos")]
extern "C" {
    // <libproc.h>  int proc_pidpath(int pid, void *buffer, uint32_t buffersize);
    fn proc_pidpath(pid: i32, buffer: *mut u8, buffersize: u32) -> i32;
}

// utils.cpp:1167-1205  std::string get_process_name(int pid)
// BBS: backup & restore. Returns the executable's name from its last separator on: the C++ walks
// `p` to the last separator with `strchr(p + 1, sep)` and returns from there, so the separator is
// part of the result ("/usr/bin/foo" gives "/foo"; a path without one is returned whole).
pub fn get_process_name(pid: i32) -> String {
    // The tail from the last separator after the first byte, as the strchr loop leaves it.
    fn tail_from_last(path: &[u8], sep: u8) -> Vec<u8> {
        let mut p = 0usize;
        while let Some(q) = path.get(p + 1..).and_then(|rest| rest.iter().position(|&c| c == sep)) {
            p = p + 1 + q;
        }
        path[p.min(path.len())..].to_vec()
    }
    #[cfg(windows)]
    {
        // utils.cpp:1170-1182  GetModuleFileNameA (pid 0) / GetModuleFileNameExA, MAX_PATH bytes.
        use winapi::um::processthreadsapi::OpenProcess;
        use winapi::um::psapi::GetModuleFileNameExA;
        use winapi::um::winnt::{HANDLE, PROCESS_QUERY_LIMITED_INFORMATION};
        extern "system" {
            // <handleapi.h>, kernel32 (the crate's winapi features leave handleapi out).
            fn CloseHandle(h: HANDLE) -> i32;
        }
        // <handleapi.h>  #define INVALID_HANDLE_VALUE ((HANDLE)(LONG_PTR)-1)
        let invalid_handle_value: HANDLE = -1isize as HANDLE;
        let mut name = [0u8; 260];
        unsafe {
            if pid == 0 {
                // GetModuleFileNameExA on the current process is GetModuleFileNameA(NULL).
                let h = winapi::um::processthreadsapi::GetCurrentProcess();
                GetModuleFileNameExA(h, std::ptr::null_mut(), name.as_mut_ptr() as *mut i8, 260);
            } else {
                let h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, pid as u32);
                // The C++ compares with INVALID_HANDLE_VALUE (OpenProcess fails with NULL).
                if h == invalid_handle_value {
                    return String::new();
                }
                GetModuleFileNameExA(h, std::ptr::null_mut(), name.as_mut_ptr() as *mut i8, 260);
                CloseHandle(h);
            }
        }
        let len = name.iter().position(|&c| c == 0).unwrap_or(name.len());
        // utils.cpp:1183-1186  return decode_path(p);
        return decode_path(&String::from_utf8_lossy(&tail_from_last(&name[..len], b'\\')));
    }
    #[cfg(target_os = "macos")]
    {
        // utils.cpp:1187-1194  PROC_PIDPATHINFO_MAXSIZE = 4 * MAXPATHLEN
        let mut pathbuf = [0u8; 4 * 1024];
        let pid = if pid == 0 { std::process::id() as i32 } else { pid };
        let ret = unsafe { proc_pidpath(pid, pathbuf.as_mut_ptr(), pathbuf.len() as u32) };
        if ret <= 0 {
            return String::new();
        }
        let len = pathbuf.iter().position(|&c| c == 0).unwrap_or(pathbuf.len());
        return String::from_utf8_lossy(&tail_from_last(&pathbuf[..len], b'/')).into_owned();
    }
    #[cfg(all(not(windows), not(target_os = "macos")))]
    {
        // utils.cpp:1195-1204  readlink("/proc/self/exe" or "/proc/<pid>/exe") into 512 bytes.
        let proc_path = if pid != 0 { format!("/proc/{}/exe", pid) } else { "/proc/self/exe".to_string() };
        let target = match std::fs::read_link(&proc_path) {
            Ok(t) => t,
            Err(e) => {
                // perror(NULL)
                eprintln!("{}", e);
                return String::new();
            }
        };
        let bytes = target.to_string_lossy().into_owned().into_bytes();
        let bytes = &bytes[..bytes.len().min(511)];
        return String::from_utf8_lossy(&tail_from_last(bytes, b'/')).into_owned();
    }
}

// Utils.cpp:1189  std::string get_process_name(int pid)
// BLOCKED: reads the executable path via proc_pidpath()/readlink("/proc/.../exe")
// /GetModuleFileNameExA — all native, not wasm-safe. Symbol intentionally omitted
// here; see notes.

// Utils.cpp:1229  std::string xml_escape(std::string text, bool is_marked = false)
// FIXME this has potentially O(n^2) time complexity!
pub fn xml_escape(text: &str, is_marked: bool) -> String {
    // Utils.cpp:1231-1251  scan for "\"'&<>" and replace one char at a time.
    let mut text: Vec<char> = text.chars().collect();
    let mut pos = 0usize;
    loop {
        // find_first_of("\"\'&<>", pos)
        let mut found: Option<usize> = None;
        let mut i = pos;
        while i < text.len() {
            match text[i] {
                '"' | '\'' | '&' | '<' | '>' => {
                    found = Some(i);
                    break;
                }
                _ => i += 1,
            }
        }
        let p = match found {
            Some(p) => p,
            None => break,
        };

        // Utils.cpp:1238-1247
        let replacement: &str = match text[p] {
            '"' => "&quot;",
            '\'' => "&apos;",
            '&' => "&amp;",
            '<' => {
                if is_marked {
                    "<"
                } else {
                    "&lt;"
                }
            }
            '>' => {
                if is_marked {
                    ">"
                } else {
                    "&gt;"
                }
            }
            _ => "",
        };

        // Utils.cpp:1249-1250  text.replace(pos, 1, replacement); pos += replacement.size();
        let rep_chars: Vec<char> = replacement.chars().collect();
        let rep_len = rep_chars.len();
        text.splice(p..p + 1, rep_chars);
        pos = p + rep_len;
    }

    // Utils.cpp:1253
    text.into_iter().collect()
}

// Utils.cpp:1229 convenience overload matching default `is_marked = false`.
pub fn xml_escape_default(text: &str) -> String {
    xml_escape(text, false)
}

// Utils.cpp:1259  std::string xml_escape_double_quotes_attribute_value(std::string text)
pub fn xml_escape_double_quotes_attribute_value(text: &str) -> String {
    // Utils.cpp:1261-1279
    let mut text: Vec<char> = text.chars().collect();
    let mut pos = 0usize;
    loop {
        // find_first_of("\"&<\r\n\t", pos)
        let mut found: Option<usize> = None;
        let mut i = pos;
        while i < text.len() {
            match text[i] {
                '"' | '&' | '<' | '\r' | '\n' | '\t' => {
                    found = Some(i);
                    break;
                }
                _ => i += 1,
            }
        }
        let p = match found {
            Some(p) => p,
            None => break,
        };

        // Utils.cpp:1266-1274
        let replacement: &str = match text[p] {
            '"' => "&quot;",
            '&' => "&amp;",
            '<' => "&lt;",
            '\r' => "&#xD;",
            '\n' => "&#xA;",
            '\t' => "&#x9;",
            _ => "",
        };

        // Utils.cpp:1277-1278
        let rep_chars: Vec<char> = replacement.chars().collect();
        let rep_len = rep_chars.len();
        text.splice(p..p + 1, rep_chars);
        pos = p + rep_len;
    }

    // Utils.cpp:1281
    text.into_iter().collect()
}

// Utils.cpp:1284  std::string xml_unescape(std::string s)
pub fn xml_unescape(s: &str) -> String {
    // Utils.cpp:1286-1318  operate on bytes/positions like std::string.
    let s: Vec<u8> = s.as_bytes().to_vec();
    let mut ret: Vec<u8> = Vec::new();
    let mut i: usize = 0;
    let mut pos: usize = 0;
    let substr = |from: usize, len: usize| -> &[u8] {
        let end = (from + len).min(s.len());
        if from >= s.len() {
            &[]
        } else {
            &s[from..end]
        }
    };
    while i < s.len() {
        if s[i] == b'&' {
            if substr(i, 4) == b"&lt;" {
                ret.extend_from_slice(&s[pos..i]);
                ret.push(b'<');
                i += 4;
                pos = i;
            } else if substr(i, 4) == b"&gt;" {
                ret.extend_from_slice(&s[pos..i]);
                ret.push(b'>');
                i += 4;
                pos = i;
            } else if substr(i, 5) == b"&amp;" {
                ret.extend_from_slice(&s[pos..i]);
                ret.push(b'&');
                i += 5;
                pos = i;
            } else {
                i += 1;
            }
        } else {
            i += 1;
        }
    }
    // Utils.cpp:1316  ret += s.substr(pos);
    ret.extend_from_slice(&s[pos..]);
    // Lossless reconstruction (inputs are UTF-8 with ASCII delimiters).
    String::from_utf8_lossy(&ret).into_owned()
}

// Utils.cpp:1320  std::string format_memsize_MB(size_t n)
pub fn format_memsize_mb(mut n: usize) -> String {
    // Utils.cpp:1322-1324
    let mut out: String;
    let mut n2: usize = 0;
    let mut scale: usize = 1;
    // Utils.cpp:1326-1327  Round to MB.
    n += 500000;
    n /= 1000000;
    // Utils.cpp:1328-1332
    while n >= 1000 {
        n2 += scale * (n % 1000);
        n /= 1000;
        scale *= 1000;
    }
    // Utils.cpp:1333-1335  sprintf(buf, "%d", (int)n);
    out = format!("{}", n as i32);
    // Utils.cpp:1336-1342
    while scale != 1 {
        scale /= 1000;
        n = n2 / scale;
        n2 %= scale;
        // sprintf(buf, ",%03d", (int)n);
        out += &format!(",{:03}", n as i32);
    }
    // Utils.cpp:1343  return out + "MB";
    out + "MB"
}

// Utils.cpp:1346  std::string format_diameter_to_str(double diameter, int precision)
//   header default: int precision = 1  (Utils.hpp:104)
pub fn format_diameter_to_str(diameter: f64, precision: usize) -> String {
    // Utils.cpp:1348  double candidates[] = {0.2, 0.4, 0.6, 0.8};
    let candidates: [f64; 4] = [0.2, 0.4, 0.6, 0.8];
    // Utils.cpp:1349
    //   double best = *std::min_element(begin, end,
    //       [diameter](double a, double b) { return std::abs(a - diameter) < std::abs(b - diameter); });
    // std::min_element returns the first of equal elements (strict `<` comparator),
    // so iterate and only replace when strictly closer.
    let mut best = candidates[0];
    for &c in candidates.iter().skip(1) {
        if (c - diameter).abs() < (best - diameter).abs() {
            best = c;
        }
    }
    // Utils.cpp:1350-1352  oss << std::fixed << std::setprecision(precision) << best;
    format!("{:.*}", precision, best)
}

// Utils.cpp:1346 — convenience overload matching the Utils.hpp default `precision = 1`.
pub fn format_diameter_to_str_default(diameter: f64) -> String {
    format_diameter_to_str(diameter, 1)
}

// Utils.cpp:1358  std::string log_memory_info(bool ignore_loglevel = false)
// BLOCKED: queries platform process/RSS counters (mach task_info, /proc/self/statm,
// getrusage, GetProcessMemoryInfo). All native, not wasm-safe.
// Faithfully returns an empty string when the log level is above info (the C++
// guard at Utils.cpp:1361 also returns empty otherwise).
pub fn log_memory_info(ignore_loglevel: bool) -> String {
    // Utils.cpp:1360-1422
    let _ = ignore_loglevel;
    String::new()
}

// Utils.cpp:1428  size_t total_physical_memory()
// BLOCKED: sysctl(HW_MEMSIZE)/sysconf(_SC_PHYS_PAGES)/GlobalMemoryStatusEx — native
// only, not wasm-safe. Returns 0 like the C++ "Unknown OS" fallback (Utils.cpp:1493).
#[cfg(unix)]
extern "C" {
    // <sys/resource.h>  int getrusage(int who, struct rusage *r_usage);
    fn getrusage(who: i32, r_usage: *mut i64) -> i32;
}

// utils.cpp:1407-1426  double get_peak_rss_mb()
pub fn get_peak_rss_mb() -> f64 {
    #[cfg(windows)]
    {
        // utils.cpp:1409-1412  GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))
        use winapi::um::psapi::{GetProcessMemoryInfo, PROCESS_MEMORY_COUNTERS};
        let mut pmc: PROCESS_MEMORY_COUNTERS = unsafe { std::mem::zeroed() };
        let ok = unsafe {
            GetProcessMemoryInfo(
                winapi::um::processthreadsapi::GetCurrentProcess(),
                &mut pmc,
                std::mem::size_of::<PROCESS_MEMORY_COUNTERS>() as u32,
            )
        };
        if ok != 0 {
            return pmc.PeakWorkingSetSize as f64 / (1024.0 * 1024.0);
        }
    }
    #[cfg(unix)]
    {
        // utils.cpp:1413-1421  getrusage(RUSAGE_SELF, &memory_info): struct rusage is two
        // struct timeval (16 bytes each on 64-bit targets) and fourteen longs, ru_maxrss the
        // first of them.
        let mut memory_info = [0i64; 18];
        if unsafe { getrusage(0 /* RUSAGE_SELF */, memory_info.as_mut_ptr()) } == 0 {
            #[allow(unused_mut)]
            let mut peak_rss = memory_info[4] as usize;
            // ru_maxrss is in kilobytes on Linux, bytes on macOS.
            #[cfg(target_os = "linux")]
            {
                peak_rss *= 1024;
            }
            return peak_rss as f64 / (1024.0 * 1024.0);
        }
    }
    // utils.cpp:1425
    0.0
}

pub fn total_physical_memory() -> usize {
    // Utils.cpp:1493  return 0L; // Unknown OS.
    0
}

// Utils.cpp:1497  bool makedir(const std::string path)
pub fn makedir(path: &str) -> bool {
    // Utils.cpp:1498-1508  create dir if missing; return true if exists/created.
    // Non-Windows/non-linux branch in C++ is empty and falls through to `return true`.
    let p = std::path::Path::new(path);
    if p.is_dir() {
        // dir already exists (Utils.cpp:1508)
        return true;
    }
    std::fs::create_dir(p).is_ok()
}

/// MD5 (RFC 1321) for `bbl_calc_md5`, which the C++ takes from OpenSSL (MD5_Init / MD5_Update /
/// MD5_Final).
struct Md5Ctx {
    state: [u32; 4],
    len: u64,
    buf: Vec<u8>,
}

impl Md5Ctx {
    fn new() -> Self {
        Md5Ctx { state: [0x6745_2301, 0xefcd_ab89, 0x98ba_dcfe, 0x1032_5476], len: 0, buf: Vec::with_capacity(64) }
    }

    fn block(state: &mut [u32; 4], chunk: &[u8]) {
        const S: [u32; 64] = [
            7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 5, 9, 14, 20, 5, 9, 14, 20, 5, 9, 14, 20, 5,
            9, 14, 20, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 6, 10, 15, 21, 6, 10, 15, 21, 6,
            10, 15, 21, 6, 10, 15, 21,
        ];
        let mut m = [0u32; 16];
        for (i, w) in m.iter_mut().enumerate() {
            *w = u32::from_le_bytes([chunk[4 * i], chunk[4 * i + 1], chunk[4 * i + 2], chunk[4 * i + 3]]);
        }
        let (mut a, mut b, mut c, mut d) = (state[0], state[1], state[2], state[3]);
        for i in 0..64 {
            let (f, g) = match i / 16 {
                0 => ((b & c) | (!b & d), i),
                1 => ((d & b) | (!d & c), (5 * i + 1) % 16),
                2 => (b ^ c ^ d, (3 * i + 5) % 16),
                _ => (c ^ (b | !d), (7 * i) % 16),
            };
            // K[i] = floor(abs(sin(i + 1)) * 2^32)
            let k = ((i as f64 + 1.0).sin().abs() * 4_294_967_296.0) as u32;
            let f = f.wrapping_add(a).wrapping_add(k).wrapping_add(m[g]);
            a = d;
            d = c;
            c = b;
            b = b.wrapping_add(f.rotate_left(S[i]));
        }
        state[0] = state[0].wrapping_add(a);
        state[1] = state[1].wrapping_add(b);
        state[2] = state[2].wrapping_add(c);
        state[3] = state[3].wrapping_add(d);
    }

    fn update(&mut self, data: &[u8]) {
        self.len += data.len() as u64;
        self.buf.extend_from_slice(data);
        let full = self.buf.len() / 64 * 64;
        for chunk in self.buf[..full].chunks(64) {
            Self::block(&mut self.state, chunk);
        }
        self.buf.drain(..full);
    }

    fn finalize(mut self) -> [u8; 16] {
        let bits = self.len.wrapping_mul(8);
        let mut pad = vec![0x80u8];
        while (self.buf.len() + pad.len()) % 64 != 56 {
            pad.push(0);
        }
        pad.extend_from_slice(&bits.to_le_bytes());
        let len = self.len;
        self.update(&pad);
        self.len = len;
        let mut out = [0u8; 16];
        for (i, w) in self.state.iter().enumerate() {
            out[4 * i..4 * i + 4].copy_from_slice(&w.to_le_bytes());
        }
        out
    }
}

// utils.cpp:1511-1530  bool bbl_calc_md5(std::string &filename, std::string &md5_out)
// `boost::filesystem::file_size` throws when the file cannot be read; that is the `Err` here.
pub fn bbl_calc_md5(filename: &str, md5_out: &mut String) -> std::io::Result<bool> {
    use std::io::Read;
    // utils.cpp:1513-1515  unsigned char digest[16]; MD5_CTX ctx; MD5_Init(&ctx);
    let mut ctx = Md5Ctx::new();
    // utils.cpp:1516-1519
    let mut ifs = std::fs::File::open(filename)?;
    let mut buf = vec![0u8; 64 * 1024];
    let _size = std::fs::metadata(filename)?.len();
    // utils.cpp:1520-1524  while (ifs) { ifs.read(buf.data(), buf.size()); MD5_Update(..., ifs.gcount()); }
    loop {
        let mut read_bytes = 0usize;
        while read_bytes < buf.len() {
            match ifs.read(&mut buf[read_bytes..]) {
                Ok(0) => break,
                Ok(n) => read_bytes += n,
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(e),
            }
        }
        ctx.update(&buf[..read_bytes]);
        if read_bytes < buf.len() {
            break;
        }
    }
    // utils.cpp:1525-1528  MD5_Final; sprintf("%02X") per byte
    let digest = ctx.finalize();
    *md5_out = digest.iter().map(|b| format!("{:02X}", b)).collect();
    // utils.cpp:1529
    Ok(true)
}

// Utils.cpp:1511  bool bbl_calc_md5(std::string &filename, std::string &md5_out)
// BLOCKED: depends on openssl MD5_* (native crypto backend). Not ported to keep
// the crate dependency-free / wasm-safe; see notes.

// Utils.cpp:1532  void save_string_file(const path& p, const std::string& str)
pub fn save_string_file(p: &std::path::Path, s: &str) -> std::io::Result<()> {
    // Utils.cpp:1534-1537  open binary, write str bytes.
    std::fs::write(p, s.as_bytes())
}

// Utils.cpp:1540  void load_string_file(const path& p, std::string& str)
pub fn load_string_file(p: &std::path::Path, s: &mut String) -> std::io::Result<()> {
    // Utils.cpp:1542-1547  open binary, read file_size() bytes into str.
    let bytes = std::fs::read(p)?;
    *s = String::from_utf8_lossy(&bytes).into_owned();
    Ok(())
}

// ----------------------------------------------------------------------------
// Local helpers (not in the C++ public API; replicate boost behaviour used above).
// ----------------------------------------------------------------------------

// strcasecmp(a, b) == 0 : case-insensitive equality of two C strings (ASCII).
// Used by is_ini_file/is_idx_file/is_gallery_file to compare file extensions.
fn strcasecmp(a: &str, b: &str) -> i32 {
    let abytes = a.as_bytes();
    let bbytes = b.as_bytes();
    let n = abytes.len().min(bbytes.len());
    for i in 0..n {
        let ca = abytes[i].to_ascii_lowercase();
        let cb = bbytes[i].to_ascii_lowercase();
        if ca != cb {
            return ca as i32 - cb as i32;
        }
    }
    abytes.len() as i32 - bbytes.len() as i32
}

// boost::filesystem::path(dir_entry).extension().string(): the extension
// including the leading dot, or "" if none.
fn dir_entry_extension(dir_entry: &std::fs::DirEntry) -> String {
    let path = dir_entry.path();
    match path.extension() {
        Some(ext) => format!(".{}", ext.to_string_lossy()),
        None => String::new(),
    }
}

// boost::iends_with: case-insensitive suffix test.
fn iends_with(haystack: &str, suffix: &str) -> bool {
    let h = haystack.as_bytes();
    let s = suffix.as_bytes();
    if s.len() > h.len() {
        return false;
    }
    let tail = &h[h.len() - s.len()..];
    tail.iter()
        .zip(s.iter())
        .all(|(a, b)| a.to_ascii_lowercase() == b.to_ascii_lowercase())
}

// ::sscanf(time, "%dd %dh %dm %ds", ...) style: pull the first `count` signed
// integers out of the string, in order, defaulting unmatched fields to 0
// (matching C's behavior where unmatched %d leaves its variable unchanged at 0).
fn sscanf_ints(s: &str, count: usize) -> Vec<i32> {
    let mut out = vec![0i32; count];
    let mut idx = 0usize;
    let bytes = s.as_bytes();
    let mut i = 0usize;
    while i < bytes.len() && idx < count {
        let c = bytes[i];
        if c == b'-' && i + 1 < bytes.len() && bytes[i + 1].is_ascii_digit() {
            // signed number
            let start = i;
            i += 1;
            while i < bytes.len() && bytes[i].is_ascii_digit() {
                i += 1;
            }
            out[idx] = s[start..i].parse::<i32>().unwrap_or(0);
            idx += 1;
        } else if c.is_ascii_digit() {
            let start = i;
            while i < bytes.len() && bytes[i].is_ascii_digit() {
                i += 1;
            }
            out[idx] = s[start..i].parse::<i32>().unwrap_or(0);
            idx += 1;
        } else {
            i += 1;
        }
    }
    out
}

// ::sscanf(time, "%fs", &f_seconds): pull the first float out of the string.
fn sscanf_first_float(s: &str) -> f32 {
    let bytes = s.as_bytes();
    let mut i = 0usize;
    while i < bytes.len() {
        let c = bytes[i];
        if c.is_ascii_digit()
            || ((c == b'-' || c == b'+' || c == b'.')
                && i + 1 < bytes.len()
                && (bytes[i + 1].is_ascii_digit() || bytes[i + 1] == b'.'))
        {
            let start = i;
            if c == b'-' || c == b'+' {
                i += 1;
            }
            while i < bytes.len() && (bytes[i].is_ascii_digit() || bytes[i] == b'.') {
                i += 1;
            }
            return s[start..i].parse::<f32>().unwrap_or(0.0);
        }
        i += 1;
    }
    0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cli_error_codes() {
        // Utils.hpp:21,22,47,70
        assert_eq!(CLI_SUCCESS, 0);
        assert_eq!(CLI_ENVIRONMENT_ERROR, -1);
        assert_eq!(CLI_3MF_FEATURE_NOT_SUPPORTED, -25);
        assert_eq!(CLI_SLICING_ERROR, -100);
    }

    #[test]
    fn test_convert_to_full_version() {
        // Utils.hpp:116
        assert_eq!(convert_to_full_version("0.1.3.4"), "00.01.03.04");
        assert_eq!(convert_to_full_version("10.2.30.4"), "10.02.30.04");
        // Wrong number of items -> empty.
        assert_eq!(convert_to_full_version("1.2.3"), "");
    }

    #[test]
    fn test_round_divide() {
        // Utils.hpp:295,300
        assert_eq!(round_divide(10, 4), 3); // (10+2)/4 = 3
        assert_eq!(round_up_divide(10, 4), 3); // (10+3)/4 = 3
        assert_eq!(round_up_divide(9, 4), 3); // (9+3)/4 = 3
    }

    #[test]
    fn test_get_max_element() {
        // Utils.hpp:306
        assert_eq!(get_max_element::<i32>(&[]), 0);
        assert_eq!(get_max_element(&[3, 1, 4, 1, 5, 9, 2]), 9);
    }

    #[test]
    fn test_next_highest_power_of_2() {
        // Utils.hpp:463-515
        assert_eq!(next_highest_power_of_2(0), 1);
        assert_eq!(next_highest_power_of_2(1), 1);
        assert_eq!(next_highest_power_of_2(2), 2);
        assert_eq!(next_highest_power_of_2(3), 4);
        assert_eq!(next_highest_power_of_2(7), 8);
        assert_eq!(next_highest_power_of_2(9), 16);
        assert_eq!(next_highest_power_of_2_u16(0), 1);
        assert_eq!(next_highest_power_of_2_u32(33), 64);
    }

    #[test]
    fn test_modulo_helpers() {
        // Utils.hpp:528,536
        assert_eq!(prev_idx_modulo(0, 5), 4);
        assert_eq!(prev_idx_modulo(3, 5), 2);
        assert_eq!(next_idx_modulo(4, 5), 0);
        assert_eq!(next_idx_modulo(2, 5), 3);
        let v = [10, 20, 30];
        assert_eq!(*prev_value_modulo(0, &v), 30);
        assert_eq!(*next_value_modulo(2, &v), 10);
    }

    #[test]
    fn test_short_time() {
        // Utils.hpp:638
        assert_eq!(short_time("2d 3h 30m 0s"), "2d3h30m");
        assert_eq!(short_time("5h 12m 0s"), "5h12m");
        assert_eq!(short_time("7m 0s"), "7m0s");
        assert_eq!(short_time("0s"), "0s");
    }

    #[test]
    fn test_get_time_dhms() {
        // Utils.hpp:685
        assert_eq!(get_time_dhms(90061.0), "1d 1h 1m 1s");
        assert_eq!(get_time_dhms(3661.0), "1h 1m 1s");
        assert_eq!(get_time_dhms(61.0), "1m 1s");
        assert_eq!(get_time_dhms(5.0), "5s");
    }

    #[test]
    fn test_get_bbl_time_dhms() {
        // Utils.hpp:709
        assert_eq!(get_bbl_time_dhms(90061.0), "1d1h1m1s");
        assert_eq!(get_bbl_time_dhms(61.0), "1m1s");
    }

    #[test]
    fn test_get_timezone_utc_hm() {
        // Utils.hpp:731
        assert_eq!(get_timezone_utc_hm(28800), "UTC+08:00");
        assert_eq!(get_timezone_utc_hm(-18000), "UTC-05:00");
    }

    #[test]
    fn test_get_time_hms() {
        // Utils.hpp:770
        assert_eq!(get_time_hms(3661.0), "01:01:01");
    }

    #[test]
    fn test_format_time_hm() {
        // Utils.hpp:807
        assert_eq!(format_time_hm(13, 5, false), "13:05");
        assert_eq!(format_time_hm(13, 5, true), "01:05PM");
        assert_eq!(format_time_hm(0, 30, true), "12:30AM");
    }

    #[test]
    fn test_filter_characters() {
        // Utils.hpp:903
        assert_eq!(filter_characters("a/b\\c:d", "/\\:"), "abcd");
    }

    #[test]
    fn test_get_bbl_finish_time_dhm() {
        // Utils.hpp:827  time_in_secs < 1 -> "Finished".
        assert_eq!(get_bbl_finish_time_dhm(0.0, false), "Finished");
        // Non-trivial values produce HH:MM (possibly with +N day suffix).
        let s = get_bbl_finish_time_dhm(3600.0, false);
        assert!(s.len() >= 5, "expected HH:MM-ish, got {s}");
    }

    #[test]
    fn test_convert_vector() {
        // Utils.hpp:316
        let v: Vec<i32> = convert_vector::<i64, i32>(&[1i64, 2, 3]).unwrap();
        assert_eq!(v, vec![1, 2, 3]);
        // signed overflow -> Err
        assert!(convert_vector::<i64, i8>(&[1000i64]).is_err());
        // negative into unsigned -> Err
        assert!(convert_vector::<i32, u8>(&[-1i32]).is_err());
    }

    #[test]
    fn test_logging_levels() {
        // Utils.cpp:123,136,149
        assert_eq!(level_string_to_boost("info"), 3);
        assert_eq!(level_string_to_boost("unknown"), 0);
        assert_eq!(get_string_logging_level(5), "trace");
        assert_eq!(get_string_logging_level(99), "error");
        set_logging_level(3);
        assert_eq!(get_logging_level(), 3);
        set_logging_level(1);
    }

    #[test]
    fn test_dirs() {
        // Utils.cpp:191-259
        set_var_dir("/tmp/var");
        assert_eq!(var_dir(), "/tmp/var");
        set_temporary_dir("/tmp/tmp");
        assert_eq!(temporary_dir(), "/tmp/tmp");
        set_local_dir("/tmp/loc");
        assert_eq!(localization_dir(), "/tmp/loc");
        set_sys_shapes_dir("/tmp/shapes");
        assert_eq!(sys_shapes_dir(), "/tmp/shapes");
    }

    #[test]
    fn test_file_predicates() {
        // Utils.cpp:1038-1061
        assert!(is_gcode_file("model.GCODE"));
        assert!(!is_gcode_file("model.stl"));
        assert!(is_json_file("a.JSON"));
        assert!(is_img_file("a.png"));
        assert!(is_img_file("a.SVG"));
        assert!(is_gallery_file("a.stl", ".stl"));
    }

    #[test]
    fn test_split_string() {
        // utils.cpp:1102
        assert_eq!(split_string("a,b,c", ','), vec!["a", "b", "c"]);
        // A trailing delimiter gives no trailing empty string: the getline after "b," extracts
        // nothing at the end of the stream and fails. The pin's split_string gives ["a", "b"]
        // (a generator calling it).
        assert_eq!(split_string("a,b,", ','), vec!["a", "b"]);
        assert!(split_string("", ',').is_empty());
        // Empty strings between delimiters and a leading delimiter are kept.
        assert_eq!(split_string(",a,,b", ','), vec!["", "a", "", "b"]);
        assert_eq!(split_string(",", ','), vec![""]);
    }

    #[test]
    fn test_perl_utils() {
        // Utils.cpp:1135-1144
        assert_eq!(perl_utils::path_to_filename("/a/b/c.txt"), "c.txt");
        assert_eq!(perl_utils::path_to_stem("/a/b/c.txt"), "c");
        assert_eq!(perl_utils::path_to_extension("/a/b/c.txt"), ".txt");
        assert_eq!(perl_utils::path_to_parent_path("/a/b/c.txt"), "/a/b");
    }

    #[test]
    fn test_xml_escape() {
        // Utils.cpp:1229
        assert_eq!(xml_escape("a<b>&\"'", false), "a&lt;b&gt;&amp;&quot;&apos;");
        assert_eq!(xml_escape("a<b>", true), "a<b>");
    }

    #[test]
    fn test_xml_escape_attr() {
        // Utils.cpp:1259
        assert_eq!(
            xml_escape_double_quotes_attribute_value("a\"b&c\r\n\t"),
            "a&quot;b&amp;c&#xD;&#xA;&#x9;"
        );
    }

    #[test]
    fn test_xml_unescape() {
        // Utils.cpp:1284
        assert_eq!(xml_unescape("a&lt;b&gt;&amp;c"), "a<b>&c");
        // Unknown entity is left as-is.
        assert_eq!(xml_unescape("&quot;"), "&quot;");
    }

    #[test]
    fn test_format_memsize_mb() {
        // Utils.cpp:1320 — rounds to MB, comma-groups, appends "MB".
        assert_eq!(format_memsize_mb(1_000_000), "1MB");
        assert_eq!(format_memsize_mb(1_500_000), "2MB");
        assert_eq!(format_memsize_mb(1_000_000_000), "1,000MB");
    }

    #[test]
    fn test_format_diameter_to_str() {
        // Utils.cpp:1346
        assert_eq!(format_diameter_to_str(0.42, 1), "0.4");
        assert_eq!(format_diameter_to_str_default(0.61), "0.6");
    }

    #[test]
    fn test_total_physical_memory_unknown() {
        // Utils.cpp:1428 — wasm-safe stub returns 0 (Unknown OS fallback).
        assert_eq!(total_physical_memory(), 0);
    }
}
