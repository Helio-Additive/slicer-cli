//! Faithful 1:1 Rust port of `libslic3r/Semver.{hpp,cpp}`.
//!
//! C++ Reference:
//! - src/libslic3r/Semver.hpp  (the `Slic3r::Semver` wrapper class)
//! - src/libslic3r/Semver.cpp  (the `SEMVER` global)
//! - src/semver/semver.{h,c}   (bundled MIT C `semver` library by Tomas Aparicio,
//!                              which `Semver.hpp` delegates all behaviour to)
//!
//! Because `Semver.hpp` is a thin wrapper that forwards every operation to the
//! bundled C `semver` library, byte-exact parity requires faithfully porting
//! that C library as well. It lives in the private `csemver` submodule below,
//! line-referenced against `semver.c`. The public `Semver` type mirrors the
//! `Slic3r::Semver` class exactly.

use crate::{Error, Result};

// SLIC3R_VERSION, configured by version.inc:16 -> set(SLIC3R_VERSION "02.06.00.51")
// libslic3r_version.h.in:6 : #define SLIC3R_VERSION "@SLIC3R_VERSION@"
pub const SLIC3R_VERSION: &str = "02.06.00.51";

/// Faithful port of the bundled C `semver` library (`src/semver/semver.{h,c}`).
///
/// Heap-allocated `char *` fields (`metadata`, `prerelease`) are modelled as
/// `Option<String>`: `None` mirrors a null pointer, `Some` a non-null pointer.
mod csemver {
    // semver.c:13-20
    const SLICE_SIZE: usize = 50;
    const DELIMITER: char = '.';
    const PR_DELIMITER: char = '-';
    const MT_DELIMITER: char = '+';
    const NUMBERS: &str = "0123456789";
    const ALPHA: &str = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
    // DELIMITERS  = DELIMITER PR_DELIMITER MT_DELIMITER
    const DELIMITERS: &str = ".-+";
    // VALID_CHARS = NUMBERS ALPHA DELIMITERS
    // (built at use sites to mirror the C concatenation)

    // semver.c:22-23
    const MAX_SIZE: usize = std::mem::size_of::<u8>() * 255;
    // (unsigned int) -1 >> 1
    const MAX_SAFE_INT: i32 = (u32::MAX >> 1) as i32;

    /// semver_t struct
    /// semver.h:23-29
    #[derive(Debug, Clone, Default)]
    pub struct SemverT {
        pub major: i32,
        pub minor: i32,
        pub patch: i32,
        pub metadata: Option<String>,
        pub prerelease: Option<String>,
    }

    // VALID_CHARS, assembled from NUMBERS + ALPHA + DELIMITERS (semver.c:20).
    fn valid_chars() -> String {
        let mut s = String::with_capacity(NUMBERS.len() + ALPHA.len() + DELIMITERS.len());
        s.push_str(NUMBERS);
        s.push_str(ALPHA);
        s.push_str(DELIMITERS);
        s
    }

    /// semver.c:64-70
    fn contains(c: char, matrix: &str) -> i32 {
        // for (x = 0; x < len; x++) if ((char) matrix[x] == c) return 1; return 0;
        for m in matrix.bytes() {
            if m as char == c {
                return 1;
            }
        }
        0
    }

    /// semver.c:72-83
    fn has_valid_chars(s: &str, matrix: &str) -> i32 {
        // for (i = 0; i < len; i++) if (contains(str[i], matrix, mlen) == 0) return 0; return 1;
        for ch in s.bytes() {
            if contains(ch as char, matrix) == 0 {
                return 0;
            }
        }
        1
    }

    /// semver.c:85-90
    fn binary_comparison(x: i32, y: i32) -> i32 {
        if x == y {
            return 0;
        }
        if x > y {
            return 1;
        }
        -1
    }

    /// semver.c:92-102
    fn parse_int(s: &str) -> i32 {
        // valid = has_valid_chars(s, NUMBERS); if (valid == 0) return -1;
        let valid = has_valid_chars(s, NUMBERS);
        if valid == 0 {
            return -1;
        }
        // num = strtol(s, NULL, 10);
        let num = strtol_full(s);
        if num > MAX_SAFE_INT as i64 {
            return -1;
        }
        num as i32
    }

    /// Mirror of C `strtol(s, NULL, base=10)` over an entire (digit-only) string.
    /// Used where the C code passes `NULL` for `endptr`.
    fn strtol_full(s: &str) -> i64 {
        // The caller has already verified `s` is all-numeric via has_valid_chars,
        // but strtol stops at the first non-digit; replicate that to be safe.
        let bytes = s.as_bytes();
        let mut i = 0;
        // skip leading whitespace (strtol behaviour); semver strings have none.
        while i < bytes.len() && (bytes[i] as char).is_ascii_whitespace() {
            i += 1;
        }
        let mut sign: i64 = 1;
        if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
            if bytes[i] == b'-' {
                sign = -1;
            }
            i += 1;
        }
        let mut acc: i64 = 0;
        while i < bytes.len() && bytes[i].is_ascii_digit() {
            acc = acc * 10 + (bytes[i] - b'0') as i64;
            i += 1;
        }
        sign * acc
    }

    /// Mirror of C `strtol(slice, &endptr, 10)`: returns the parsed `int` value and
    /// the number of consumed bytes (so the caller can compute `endptr`).
    fn strtol_endptr(s: &str) -> (i32, usize) {
        let bytes = s.as_bytes();
        let mut i = 0;
        while i < bytes.len() && (bytes[i] as char).is_ascii_whitespace() {
            i += 1;
        }
        let mut sign: i64 = 1;
        if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
            if bytes[i] == b'-' {
                sign = -1;
            }
            i += 1;
        }
        let mut acc: i64 = 0;
        let digit_start = i;
        while i < bytes.len() && bytes[i].is_ascii_digit() {
            acc = acc * 10 + (bytes[i] - b'0') as i64;
            i += 1;
        }
        // If no conversion was performed, strtol leaves endptr at the original `s`.
        let consumed = if i == digit_start { 0 } else { i };
        ((sign * acc) as i32, consumed)
    }

    /// semver.c:141-163 : semver_parse
    ///
    /// Returns `0` on success, `-1` on error.
    pub fn semver_parse(str: &str, ver: &mut SemverT) -> i32 {
        // valid = semver_is_valid(str); if (!valid) return -1;
        let valid = semver_is_valid(str);
        if valid == 0 {
            return -1;
        }

        // buf = calloc(...); strcpy(buf, str);
        let mut buf = str.to_string();

        // ver->metadata   = parse_slice(buf, MT_DELIMITER[0]);
        ver.metadata = parse_slice(&mut buf, MT_DELIMITER);
        // ver->prerelease = parse_slice(buf, PR_DELIMITER[0]);
        ver.prerelease = parse_slice(&mut buf, PR_DELIMITER);

        // res = semver_parse_version(buf, ver);
        semver_parse_version(&buf, ver)
    }

    /// semver.c:108-130 : parse_slice
    ///
    /// Returns a heap string with the content from `sep` (exclusive) to end, and
    /// truncates `buf` at `sep`. Returns `None` when `sep` is not found.
    fn parse_slice(buf: &mut String, sep: char) -> Option<String> {
        // pr = strchr(buf, sep); if (pr == NULL) return NULL;
        let pos = buf.find(sep)?;
        // part = everything after the separator (pr + 1 .. end)
        let part = buf[pos + sep.len_utf8()..].to_string();
        // *pr = '\0'  -> truncate buf at the separator
        buf.truncate(pos);
        Some(part)
    }

    /// semver.c:174-214 : semver_parse_version
    ///
    /// Returns `0` on success, `-1` on parse error / invalid.
    pub fn semver_parse_version(str: &str, ver: &mut SemverT) -> i32 {
        // slice = (char *) str; index = 0;
        let mut slice: Option<&str> = Some(str);
        let mut index: i32 = 0;

        // non mandatory
        ver.patch = 0;

        // while (slice != NULL && index++ < 4)
        while slice.is_some() && {
            let cont = index < 4;
            index += 1;
            cont
        } {
            let cur = slice.unwrap();
            // next = strchr(slice, DELIMITER[0]);
            let next = cur.find(DELIMITER);
            // if (next == NULL) len = strlen(slice); else len = next - slice;
            let len = match next {
                None => cur.len(),
                Some(n) => n,
            };
            // if (len > SLICE_SIZE) return -1;
            if len > SLICE_SIZE {
                return -1;
            }

            // value = strtol(slice, &endptr, 10);
            let (value, consumed) = strtol_endptr(cur);
            // if (endptr != next && *endptr != '\0') return -1;
            //   endptr == slice + consumed; next == slice + (next index) or end-of-string.
            //   In C, `next` is either the '.' pointer or, if absent, NULL (handled by
            //   the *endptr=='\0' branch below). We reproduce: error unless endptr
            //   lands exactly on the delimiter, or at the string terminator.
            let endptr_at_delim = match next {
                Some(n) => consumed == n,
                None => false,
            };
            let endptr_at_nul = consumed == cur.len();
            if !endptr_at_delim && !endptr_at_nul {
                return -1;
            }

            // switch (index) { ... }
            match index {
                1 => ver.major = value,
                2 => ver.minor = value,
                3 => ver.patch = value,
                // BBS: add convert for AA.BB.CC.DD
                4 => ver.patch = (ver.patch * 100) + value,
                _ => {}
            }

            // if (next == NULL) slice = NULL; else slice = next + 1;
            slice = match next {
                None => None,
                Some(n) => Some(&cur[n + DELIMITER.len_utf8()..]),
            };
        }

        // return (index == 2 || index == 3 || index == 4) ? 0 : -1;
        if index == 2 || index == 3 || index == 4 {
            0
        } else {
            -1
        }
    }

    /// semver.c:216-265 : compare_prerelease
    fn compare_prerelease(x: Option<&str>, y: Option<&str>) -> i32 {
        // if (x == NULL && y == NULL) return 0;
        if x.is_none() && y.is_none() {
            return 0;
        }
        // if (y == NULL && x) return -1;
        if y.is_none() && x.is_some() {
            return -1;
        }
        // if (x == NULL && y) return 1;
        if x.is_none() && y.is_some() {
            return 1;
        }

        let x = x.unwrap();
        let y = y.unwrap();
        let xb = x.as_bytes();
        let yb = y.as_bytes();
        let xlen = xb.len();
        let ylen = yb.len();

        // lastx = x; lasty = y; (indices into x / y)
        let mut lastx: usize = 0;
        let mut lasty: usize = 0;

        loop {
            // if ((xptr = strchr(lastx, '.')) == NULL) xptr = x + xlen;
            let xptr = match x[lastx..].find(DELIMITER) {
                Some(p) => lastx + p,
                None => xlen,
            };
            // if ((yptr = strchr(lasty, '.')) == NULL) yptr = y + ylen;
            let yptr = match y[lasty..].find(DELIMITER) {
                Some(p) => lasty + p,
                None => ylen,
            };

            // xnum = strtol(lastx, &endptr, 10); xisnum = endptr == xptr ? 1 : 0;
            let (xnum, xconsumed) = strtol_endptr(&x[lastx..]);
            let xisnum = if lastx + xconsumed == xptr { 1 } else { 0 };
            // ynum = strtol(lasty, &endptr, 10); yisnum = endptr == yptr ? 1 : 0;
            let (ynum, yconsumed) = strtol_endptr(&y[lasty..]);
            let yisnum = if lasty + yconsumed == yptr { 1 } else { 0 };

            // if (xisnum && !yisnum) return -1;
            if xisnum != 0 && yisnum == 0 {
                return -1;
            }
            // if (!xisnum && yisnum) return 1;
            if xisnum == 0 && yisnum != 0 {
                return 1;
            }

            if xisnum != 0 && yisnum != 0 {
                // Numerical comparison
                if xnum != ynum {
                    return if xnum < ynum { -1 } else { 1 };
                }
            } else {
                // String comparison
                let xn = xptr - lastx;
                let yn = yptr - lasty;
                let min = if xn < yn { xn } else { yn };
                // res = strncmp(lastx, lasty, min);
                let res = strncmp(&xb[lastx..], &yb[lasty..], min);
                if res != 0 {
                    return if res < 0 { -1 } else { 1 };
                }
                // if (xn != yn) return xn < yn ? -1 : 1;
                if xn != yn {
                    return if xn < yn { -1 } else { 1 };
                }
            }

            // lastx = xptr + 1; lasty = yptr + 1;
            lastx = xptr + 1;
            lasty = yptr + 1;
            // if (lastx == x + xlen + 1 && lasty == y + ylen + 1) break;
            if lastx == xlen + 1 && lasty == ylen + 1 {
                break;
            }
            // if (lastx == x + xlen + 1) return -1;
            if lastx == xlen + 1 {
                return -1;
            }
            // if (lasty == y + ylen + 1) return 1;
            if lasty == ylen + 1 {
                return 1;
            }
        }

        0
    }

    /// Mirror of C `strncmp(a, b, n)` (compares up to `n` bytes; the slices may be
    /// shorter than `n`, in which case the NUL terminator stops the comparison —
    /// here represented by reaching the end of the slice, treated as a 0 byte).
    fn strncmp(a: &[u8], b: &[u8], n: usize) -> i32 {
        for i in 0..n {
            let ca = a.get(i).copied().unwrap_or(0);
            let cb = b.get(i).copied().unwrap_or(0);
            if ca != cb {
                return ca as i32 - cb as i32;
            }
            if ca == 0 {
                break;
            }
        }
        0
    }

    /// semver.c:267-270 : semver_compare_prerelease
    pub fn semver_compare_prerelease(x: &SemverT, y: &SemverT) -> i32 {
        compare_prerelease(x.prerelease.as_deref(), y.prerelease.as_deref())
    }

    /// semver.c:283-294 : semver_compare_version
    pub fn semver_compare_version(x: &SemverT, y: &SemverT) -> i32 {
        let mut res;
        if {
            res = binary_comparison(x.major, y.major);
            res
        } == 0
        {
            if {
                res = binary_comparison(x.minor, y.minor);
                res
            } == 0
            {
                return binary_comparison(x.patch, y.patch);
            }
        }
        res
    }

    /// semver.c:305-314 : semver_compare
    pub fn semver_compare(x: &SemverT, y: &SemverT) -> i32 {
        let res = semver_compare_version(x, y);
        if res == 0 {
            return semver_compare_prerelease(x, y);
        }
        res
    }

    /// semver.c:320-323 : semver_gt
    pub fn semver_gt(x: &SemverT, y: &SemverT) -> i32 {
        (semver_compare(x, y) == 1) as i32
    }

    /// semver.c:329-332 : semver_lt
    pub fn semver_lt(x: &SemverT, y: &SemverT) -> i32 {
        (semver_compare(x, y) == -1) as i32
    }

    /// semver.c:338-341 : semver_eq
    pub fn semver_eq(x: &SemverT, y: &SemverT) -> i32 {
        (semver_compare(x, y) == 0) as i32
    }

    /// semver.c:347-350 : semver_neq
    pub fn semver_neq(x: &SemverT, y: &SemverT) -> i32 {
        (semver_compare(x, y) != 0) as i32
    }

    /// semver.c:356-359 : semver_gte
    pub fn semver_gte(x: &SemverT, y: &SemverT) -> i32 {
        (semver_compare(x, y) >= 0) as i32
    }

    /// semver.c:365-368 : semver_lte
    pub fn semver_lte(x: &SemverT, y: &SemverT) -> i32 {
        (semver_compare(x, y) <= 0) as i32
    }

    /// semver.c:382-391 : semver_satisfies_caret
    pub fn semver_satisfies_caret(x: &SemverT, y: &SemverT) -> i32 {
        if x.major == y.major {
            if x.major == 0 {
                return (x.minor >= y.minor) as i32;
            }
            return 1;
        }
        0
    }

    /// semver.c:405-409 : semver_satisfies_patch
    pub fn semver_satisfies_patch(x: &SemverT, y: &SemverT) -> i32 {
        (x.major == y.major && x.minor == y.minor) as i32
    }

    // semver.c:34-40 : enum operators (ASCII codes of the comparison symbols)
    const SYMBOL_GT: i32 = 0x3e;
    const SYMBOL_LT: i32 = 0x3c;
    const SYMBOL_EQ: i32 = 0x3d;
    const SYMBOL_TF: i32 = 0x7e;
    const SYMBOL_CF: i32 = 0x5e;

    /// semver.c:431-467 : semver_satisfies
    pub fn semver_satisfies(x: &SemverT, y: &SemverT, op: &str) -> i32 {
        let ob = op.as_bytes();
        // first = op[0]; second = op[1];
        let first = *ob.first().unwrap_or(&0) as i32;
        let second = *ob.get(1).unwrap_or(&0) as i32;

        // Caret operator
        if first == SYMBOL_CF {
            return semver_satisfies_caret(x, y);
        }
        // Tilde operator
        if first == SYMBOL_TF {
            return semver_satisfies_patch(x, y);
        }
        // Strict equality
        if first == SYMBOL_EQ {
            return semver_eq(x, y);
        }
        // Greater than or equal comparison
        if first == SYMBOL_GT {
            if second == SYMBOL_EQ {
                return semver_gte(x, y);
            }
            return semver_gt(x, y);
        }
        // Lower than or equal comparison
        if first == SYMBOL_LT {
            if second == SYMBOL_EQ {
                return semver_lte(x, y);
            }
            return semver_lt(x, y);
        }
        0
    }

    /// semver.c:475-485 : semver_free
    ///
    /// Frees `metadata` and `prerelease` and sets them to null.
    pub fn semver_free(x: &mut SemverT) {
        if x.metadata.is_some() {
            x.metadata = None;
        }
        if x.prerelease.is_some() {
            x.prerelease = None;
        }
    }

    /// semver.c:491-497 : concat_num
    fn concat_num(str: &mut String, x: i32, sep: Option<char>) {
        // char buf[SLICE_SIZE] = {0};
        match sep {
            None => str.push_str(&format!("{}", x)),
            Some(s) => str.push_str(&format!("{}{}", s, x)),
        }
    }

    /// semver.c:499-504 : concat_char
    fn concat_char(str: &mut String, x: &str, sep: char) {
        str.push_str(&format!("{}{}", sep, x));
    }

    /// semver.c:510-517 : semver_render
    pub fn semver_render(x: &SemverT, dest: &mut String) {
        if x.major != 0 {
            concat_num(dest, x.major, None);
        }
        if x.minor != 0 {
            concat_num(dest, x.minor, Some(DELIMITER));
        }
        if x.patch != 0 {
            concat_num(dest, x.patch, Some(DELIMITER));
        }
        if let Some(pre) = &x.prerelease {
            concat_char(dest, pre, PR_DELIMITER);
        }
        if let Some(meta) = &x.metadata {
            concat_char(dest, meta, MT_DELIMITER);
        }
    }

    /// semver.c:523-526 : semver_bump
    pub fn semver_bump(x: &mut SemverT) {
        x.major += 1;
    }

    /// semver.c:528-531 : semver_bump_minor
    pub fn semver_bump_minor(x: &mut SemverT) {
        x.minor += 1;
    }

    /// semver.c:533-536 : semver_bump_patch
    pub fn semver_bump_patch(x: &mut SemverT) {
        x.patch += 1;
    }

    /// semver.c:542-545 : has_valid_length
    fn has_valid_length(s: &str) -> i32 {
        (s.len() <= MAX_SIZE) as i32
    }

    /// semver.c:556-560 : semver_is_valid
    pub fn semver_is_valid(s: &str) -> i32 {
        (has_valid_length(s) != 0 && has_valid_chars(s, &valid_chars()) != 0) as i32
    }

    /// semver.c:591-604 : char_to_int
    fn char_to_int(str: &str) -> i32 {
        let mut buf = 0;
        let vc = valid_chars();
        for ch in str.bytes() {
            if contains(ch as char, &vc) != 0 {
                buf += ch as i32;
            }
        }
        buf
    }

    /// semver.c:611-628 : semver_numeric
    pub fn semver_numeric(x: &SemverT) -> i32 {
        // char buf[SLICE_SIZE * 3]; memset(&buf, 0, ...);
        let mut buf = String::with_capacity(SLICE_SIZE * 3);

        if x.major != 0 {
            concat_num(&mut buf, x.major, None);
        }
        if x.minor != 0 {
            concat_num(&mut buf, x.minor, None);
        }
        if x.patch != 0 {
            concat_num(&mut buf, x.patch, None);
        }

        let num = parse_int(&buf);
        if num == -1 {
            return -1;
        }

        let mut num = num;
        if let Some(pre) = &x.prerelease {
            num += char_to_int(pre);
        }
        if let Some(meta) = &x.metadata {
            num += char_to_int(meta);
        }

        num
    }

    /// semver.c:637-647 : semver_copy
    ///
    /// (In Rust this is just `Clone`; provided for naming parity.)
    pub fn semver_copy(ver: &SemverT) -> SemverT {
        ver.clone()
    }
}

use csemver::SemverT;

/// Port of `Slic3r::Semver`.
/// Semver.hpp:18
#[derive(Debug)]
pub struct Semver {
    // Semver.hpp:165 : semver_t ver;
    ver: SemverT,
}

/// Semver.hpp:21 : struct Major { const int i; Major(int i) : i(i) {} };
#[derive(Debug, Clone, Copy)]
pub struct Major {
    pub i: i32,
}
impl Major {
    pub fn new(i: i32) -> Self {
        Major { i }
    }
}

/// Semver.hpp:22 : struct Minor { const int i; Minor(int i) : i(i) {} };
#[derive(Debug, Clone, Copy)]
pub struct Minor {
    pub i: i32,
}
impl Minor {
    pub fn new(i: i32) -> Self {
        Minor { i }
    }
}

/// Semver.hpp:23 : struct Patch { const int i; Patch(int i) : i(i) {} };
#[derive(Debug, Clone, Copy)]
pub struct Patch {
    pub i: i32,
}
impl Patch {
    pub fn new(i: i32) -> Self {
        Patch { i }
    }
}

impl Semver {
    /// Semver.hpp:25 : Semver() : ver(semver_zero()) {}
    pub fn new() -> Self {
        Semver { ver: Self::semver_zero() }
    }

    /// Semver.hpp:27-36
    /// Semver(int major, int minor, int patch,
    ///        boost::optional<const std::string&> metadata,
    ///        boost::optional<const std::string&> prerelease)
    pub fn with_parts(
        major: i32,
        minor: i32,
        patch: i32,
        metadata: Option<&str>,
        prerelease: Option<&str>,
    ) -> Self {
        // : ver(semver_zero())
        let mut s = Semver { ver: Self::semver_zero() };
        s.ver.major = major;
        s.ver.minor = minor;
        s.ver.patch = patch;
        s.set_metadata(metadata);
        s.set_prerelease(prerelease);
        s
    }

    /// Semver.hpp:48-56 : Semver(const std::string &str)
    ///
    /// The C++ ctor throws `Slic3r::RuntimeError` on parse failure; the fallible
    /// Rust equivalent returns `Result`. See `from_str_or_panic` for the throwing
    /// form used by the `SEMVER` global.
    pub fn from_str(str: &str) -> Result<Self> {
        // auto parsed = parse(str);
        let parsed = Self::parse(str);
        match parsed {
            // if (! parsed) { throw Slic3r::RuntimeError(...); }
            None => Err(Error::ParseError(format!(
                "Could not parse version string: {}",
                str
            ))),
            // ver = parsed->ver; parsed->ver = semver_zero();
            Some(parsed) => Ok(Semver { ver: parsed.ver }),
        }
    }

    /// Throwing form of `Semver(const std::string&)` used by the `SEMVER` global
    /// initialiser (which cannot recover from a malformed compile-time constant).
    /// Semver.hpp:48-56
    pub fn from_str_or_panic(str: &str) -> Self {
        match Self::from_str(str) {
            Ok(s) => s,
            Err(_) => panic!("Could not parse version string: {}", str),
        }
    }

    /// Semver.hpp:58-66 : static boost::optional<Semver> parse(const std::string &str)
    pub fn parse(str: &str) -> Option<Semver> {
        // semver_t ver = semver_zero();
        let mut ver = Self::semver_zero();
        // if (::semver_parse(str.c_str(), &ver) == 0) return Semver(ver); else return boost::none;
        if csemver::semver_parse(str, &mut ver) == 0 {
            Some(Semver { ver })
        } else {
            None
        }
    }

    /// Semver.hpp:68 : static const Semver zero()
    pub fn zero() -> Semver {
        Semver { ver: Self::semver_zero() }
    }

    /// Semver.hpp:70-74 : static const Semver inf()
    pub fn inf() -> Semver {
        // static semver_t ver = { INT_MAX, INT_MAX, INT_MAX, nullptr, nullptr };
        let ver = SemverT {
            major: i32::MAX,
            minor: i32::MAX,
            patch: i32::MAX,
            metadata: None,
            prerelease: None,
        };
        Semver { ver }
    }

    /// Semver.hpp:76-80 : static const Semver invalid()
    pub fn invalid() -> Semver {
        // static semver_t ver = { -1, 0, 0, nullptr, nullptr };
        let ver = SemverT {
            major: -1,
            minor: 0,
            patch: 0,
            metadata: None,
            prerelease: None,
        };
        Semver { ver }
    }

    // const accessors

    /// Semver.hpp:103 : int maj() const { return ver.major; }
    pub fn maj(&self) -> i32 {
        self.ver.major
    }

    /// Semver.hpp:104 : int min() const { return ver.minor; }
    pub fn min(&self) -> i32 {
        self.ver.minor
    }

    /// Semver.hpp:105 : int patch() const { return ver.patch; }
    pub fn patch(&self) -> i32 {
        self.ver.patch
    }

    /// Semver.hpp:106 : const char* prerelease() const { return ver.prerelease; }
    pub fn prerelease(&self) -> Option<&str> {
        self.ver.prerelease.as_deref()
    }

    /// Semver.hpp:107 : const char* metadata() const { return ver.metadata; }
    pub fn metadata(&self) -> Option<&str> {
        self.ver.metadata.as_deref()
    }

    // Setters

    /// Semver.hpp:110 : void set_maj(int maj) { ver.major = maj; }
    pub fn set_maj(&mut self, maj: i32) {
        self.ver.major = maj;
    }

    /// Semver.hpp:111 : void set_min(int min) { ver.minor = min; }
    pub fn set_min(&mut self, min: i32) {
        self.ver.minor = min;
    }

    /// Semver.hpp:112 : void set_patch(int patch) { ver.patch = patch; }
    pub fn set_patch(&mut self, patch: i32) {
        self.ver.patch = patch;
    }

    /// Semver.hpp:113-114 : void set_metadata(...) { ver.metadata = meta ? strdup(*meta) : nullptr; }
    pub fn set_metadata(&mut self, meta: Option<&str>) {
        self.ver.metadata = meta.map(Self::strdup);
    }

    /// Semver.hpp:115-116 : void set_prerelease(...) { ver.prerelease = pre ? strdup(*pre) : nullptr; }
    pub fn set_prerelease(&mut self, pre: Option<&str>) {
        self.ver.prerelease = pre.map(Self::strdup);
    }

    // Comparison

    /// Semver.hpp:129 : bool in_range(const Semver &low, const Semver &high) const
    pub fn in_range(&self, low: &Semver, high: &Semver) -> bool {
        // return low <= *this && *this <= high;
        low <= self && self <= high
    }

    /// Semver.hpp:127 : bool operator&(const Semver &b) const
    /// Satisfies patch if Major and minor are equal.
    /// We're using '&' instead of the '~' operator here as '~' is unary-only.
    pub fn satisfies_patch(&self, b: &Semver) -> bool {
        csemver::semver_satisfies_patch(&self.ver, &b.ver) != 0
    }

    /// Semver.hpp:128 : bool operator^(const Semver &b) const
    pub fn satisfies_caret(&self, b: &Semver) -> bool {
        csemver::semver_satisfies_caret(&self.ver, &b.ver) != 0
    }

    /// Semver.hpp:130 : bool valid() const
    pub fn valid(&self) -> bool {
        // return *this != zero() && *this != inf() && *this != invalid();
        *self != Self::zero() && *self != Self::inf() && *self != Self::invalid()
    }

    // Conversion

    /// Semver.hpp:133-143 : std::string to_string() const
    pub fn to_string(&self) -> String {
        // BBS: version format
        let mut res: String;
        // int patch_1 = ver.patch/100;
        let patch_1 = self.ver.patch / 100;
        // int patch_2 = ver.patch%100;
        let patch_2 = self.ver.patch % 100;
        // res = (boost::format("%1%.%2%.%3%.%4%") % major % minor % patch_1 % patch_2).str();
        res = format!(
            "{}.{}.{}.{}",
            self.ver.major, self.ver.minor, patch_1, patch_2
        );

        // if (ver.prerelease != nullptr) { res += '-'; res += ver.prerelease; }
        if let Some(pre) = &self.ver.prerelease {
            res.push('-');
            res.push_str(pre);
        }
        // if (ver.metadata != nullptr) { res += '+'; res += ver.metadata; }
        if let Some(meta) = &self.ver.metadata {
            res.push('+');
            res.push_str(meta);
        }
        res
    }

    // Arithmetics — see the std::ops impls below for the operator forms.

    /// Semver.hpp:169 : static semver_t semver_zero() { return { 0, 0, 0, nullptr, nullptr }; }
    fn semver_zero() -> SemverT {
        SemverT {
            major: 0,
            minor: 0,
            patch: 0,
            metadata: None,
            prerelease: None,
        }
    }

    /// Semver.hpp:170 : static char * strdup(const std::string &str) { return ::semver_strdup(str.data()); }
    fn strdup(str: &str) -> String {
        // ::semver_strdup just duplicates the bytes; an owned String is the analogue.
        str.to_string()
    }

    /// Semver.hpp:167 : Semver(semver_t ver) : ver(ver) {}
    fn from_semver_t(ver: SemverT) -> Self {
        Semver { ver }
    }
}

impl Default for Semver {
    fn default() -> Self {
        Semver::new()
    }
}

/// Semver.hpp:83 : Semver(const Semver &other) : ver(::semver_copy(&other.ver)) {}
impl Clone for Semver {
    fn clone(&self) -> Self {
        Semver::from_semver_t(csemver::semver_copy(&self.ver))
    }
}

// Comparison operators — Semver.hpp:119-124

/// Semver.hpp:121 : bool operator==(const Semver &b) const { return ::semver_compare(...) == 0; }
impl PartialEq for Semver {
    fn eq(&self, b: &Semver) -> bool {
        csemver::semver_compare(&self.ver, &b.ver) == 0
    }
}
impl Eq for Semver {}

impl PartialOrd for Semver {
    fn partial_cmp(&self, b: &Semver) -> Option<std::cmp::Ordering> {
        Some(self.cmp(b))
    }

    /// Semver.hpp:119 : bool operator<(const Semver &b) const { return ::semver_compare(...) == -1; }
    fn lt(&self, b: &Semver) -> bool {
        csemver::semver_compare(&self.ver, &b.ver) == -1
    }
    /// Semver.hpp:120 : bool operator<=(const Semver &b) const { return ::semver_compare(...) <= 0; }
    fn le(&self, b: &Semver) -> bool {
        csemver::semver_compare(&self.ver, &b.ver) <= 0
    }
    /// Semver.hpp:123 : bool operator>=(const Semver &b) const { return ::semver_compare(...) >= 0; }
    fn ge(&self, b: &Semver) -> bool {
        csemver::semver_compare(&self.ver, &b.ver) >= 0
    }
    /// Semver.hpp:124 : bool operator>(const Semver &b) const { return ::semver_compare(...) == 1; }
    fn gt(&self, b: &Semver) -> bool {
        csemver::semver_compare(&self.ver, &b.ver) == 1
    }
}

impl Ord for Semver {
    fn cmp(&self, b: &Semver) -> std::cmp::Ordering {
        // Maps ::semver_compare's {-1,0,1} onto Ordering.
        match csemver::semver_compare(&self.ver, &b.ver) {
            -1 => std::cmp::Ordering::Less,
            0 => std::cmp::Ordering::Equal,
            _ => std::cmp::Ordering::Greater,
        }
    }
}

// Arithmetics — Semver.hpp:146-157

/// Semver.hpp:146 : Semver& operator+=(const Major &b) { ver.major += b.i; return *this; }
impl std::ops::AddAssign<Major> for Semver {
    fn add_assign(&mut self, b: Major) {
        self.ver.major += b.i;
    }
}
/// Semver.hpp:147 : Semver& operator+=(const Minor &b) { ver.minor += b.i; return *this; }
impl std::ops::AddAssign<Minor> for Semver {
    fn add_assign(&mut self, b: Minor) {
        self.ver.minor += b.i;
    }
}
/// Semver.hpp:148 : Semver& operator+=(const Patch &b) { ver.patch += b.i; return *this; }
impl std::ops::AddAssign<Patch> for Semver {
    fn add_assign(&mut self, b: Patch) {
        self.ver.patch += b.i;
    }
}
/// Semver.hpp:149 : Semver& operator-=(const Major &b) { ver.major -= b.i; return *this; }
impl std::ops::SubAssign<Major> for Semver {
    fn sub_assign(&mut self, b: Major) {
        self.ver.major -= b.i;
    }
}
/// Semver.hpp:150 : Semver& operator-=(const Minor &b) { ver.minor -= b.i; return *this; }
impl std::ops::SubAssign<Minor> for Semver {
    fn sub_assign(&mut self, b: Minor) {
        self.ver.minor -= b.i;
    }
}
/// Semver.hpp:151 : Semver& operator-=(const Patch &b) { ver.patch -= b.i; return *this; }
impl std::ops::SubAssign<Patch> for Semver {
    fn sub_assign(&mut self, b: Patch) {
        self.ver.patch -= b.i;
    }
}

/// Semver.hpp:152 : Semver operator+(const Major &b) const { Semver res(*this); return res += b; }
impl std::ops::Add<Major> for Semver {
    type Output = Semver;
    fn add(self, b: Major) -> Semver {
        let mut res = self.clone();
        res += b;
        res
    }
}
/// Semver.hpp:153 : Semver operator+(const Minor &b) const
impl std::ops::Add<Minor> for Semver {
    type Output = Semver;
    fn add(self, b: Minor) -> Semver {
        let mut res = self.clone();
        res += b;
        res
    }
}
/// Semver.hpp:154 : Semver operator+(const Patch &b) const
impl std::ops::Add<Patch> for Semver {
    type Output = Semver;
    fn add(self, b: Patch) -> Semver {
        let mut res = self.clone();
        res += b;
        res
    }
}
/// Semver.hpp:155 : Semver operator-(const Major &b) const
impl std::ops::Sub<Major> for Semver {
    type Output = Semver;
    fn sub(self, b: Major) -> Semver {
        let mut res = self.clone();
        res -= b;
        res
    }
}
/// Semver.hpp:156 : Semver operator-(const Minor &b) const
impl std::ops::Sub<Minor> for Semver {
    type Output = Semver;
    fn sub(self, b: Minor) -> Semver {
        let mut res = self.clone();
        res -= b;
        res
    }
}
/// Semver.hpp:157 : Semver operator-(const Patch &b) const
impl std::ops::Sub<Patch> for Semver {
    type Output = Semver;
    fn sub(self, b: Patch) -> Semver {
        let mut res = self.clone();
        res -= b;
        res
    }
}

/// Semver.hpp:160-163 : friend std::ostream& operator<<(std::ostream& os, const Semver &self)
impl std::fmt::Display for Semver {
    fn fmt(&self, os: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // os << self.to_string();
        write!(os, "{}", self.to_string())
    }
}

// Semver.cpp:5 : Semver SEMVER { SLIC3R_VERSION };
//
// A lazily-initialised global mirroring the C++ translation-unit-level `SEMVER`.
// `Semver` is not trivially const-constructible, so we use a thread-safe lazy
// initialiser. The throwing form is used to match the C++ ctor semantics.
pub fn semver() -> &'static Semver {
    use std::sync::OnceLock;
    static SEMVER: OnceLock<Semver> = OnceLock::new();
    SEMVER.get_or_init(|| Semver::from_str_or_panic(SLIC3R_VERSION))
}
