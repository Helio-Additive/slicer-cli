//! 1:1 port of `LocalesUtils.{hpp,cpp}` from BambuStudio's libslic3r.
//!
//! C++ source:
//!   - `src/libslic3r/LocalesUtils.hpp`
//!   - `src/libslic3r/LocalesUtils.cpp`
//!
//! Faithfulness notes (see RULES):
//!   * `coord_t` -> `i64`, `coordf_t` -> `f64` (not used in this file).
//!   * The C++ relies on `<charconv>`/`fast_float` and the C `setlocale`/
//!     `uselocale` family. Those are native libc facilities; this port keeps
//!     the file wasm-safe by reimplementing the *observable behaviour* with
//!     `core`/`std` only — Rust's float formatting/parsing is always
//!     locale-independent ("C" locale), which is exactly what the C++ code
//!     goes out of its way to guarantee.

// LocalesUtils.cpp:1   #include "LocalesUtils.hpp"
// LocalesUtils.cpp:3-5 #ifdef _WIN32 / #include <charconv> / #endif
// LocalesUtils.cpp:6   #include <stdexcept>
// LocalesUtils.cpp:8   #include <fast_float/fast_float.h>

// namespace Slic3r {  (LocalesUtils.cpp:11) — represented by this module.

// ---------------------------------------------------------------------------
// CNumericLocalesSetter  (LocalesUtils.hpp:18-31)
// ---------------------------------------------------------------------------

/// RAII wrapper that sets LC_NUMERIC to "C" on construction
/// and restores the old value on destruction.
///
/// LocalesUtils.hpp:16-31
///
/// In C++ this manipulates the per-thread C locale via
/// `_configthreadlocale`/`setlocale` (Windows) or
/// `uselocale`/`newlocale`/`duplocale`/`freelocale` (macOS / Linux / BSD), so
/// that `sprintf`/`strtod` use a decimal point regardless of the user's
/// locale. Rust's standard library never consults the C `LC_NUMERIC` locale
/// for float parsing/formatting (it is always "C"-equivalent), and touching
/// the process locale would require the native libc functions which are not
/// available under wasm. The faithful, wasm-safe equivalent is therefore a
/// type that holds no state and whose construction/destruction are no-ops —
/// the invariant the C++ class establishes ("decimal point separator is in
/// effect") already holds unconditionally in Rust.
#[derive(Debug)]
pub struct CNumericLocalesSetter {
    // LocalesUtils.hpp:23-29 private members are platform locale handles
    // (`std::string m_orig_numeric_locale` on Windows, `locale_t
    // m_original_locale` / `locale_t m_new_locale` elsewhere). They have no
    // Rust counterpart; the struct is intentionally empty.
    _private: (),
}

impl CNumericLocalesSetter {
    /// LocalesUtils.cpp:14-30  CNumericLocalesSetter::CNumericLocalesSetter()
    ///
    /// C++ saves the current `LC_NUMERIC` locale and installs the "C" locale.
    /// No-op here (see the type-level documentation above).
    pub fn new() -> Self {
        // #ifdef _WIN32
        //     _configthreadlocale(_ENABLE_PER_THREAD_LOCALE);
        //     m_orig_numeric_locale = std::setlocale(LC_NUMERIC, nullptr);
        //     std::setlocale(LC_NUMERIC, "C");
        // #elif __APPLE__
        //     m_original_locale = uselocale((locale_t)0);
        //     m_new_locale = newlocale(LC_NUMERIC_MASK, "C", m_original_locale);
        //     uselocale(m_new_locale);
        // #else // linux / BSD
        //     m_original_locale = uselocale((locale_t)0);
        //     m_new_locale = duplocale(m_original_locale);
        //     m_new_locale = newlocale(LC_NUMERIC_MASK, "C", m_new_locale);
        //     uselocale(m_new_locale);
        // #endif
        Self { _private: () }
    }
}

impl Default for CNumericLocalesSetter {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for CNumericLocalesSetter {
    /// LocalesUtils.cpp:34-42  CNumericLocalesSetter::~CNumericLocalesSetter()
    ///
    /// C++ restores the previously saved locale. No-op here.
    fn drop(&mut self) {
        // #ifdef _WIN32
        //     std::setlocale(LC_NUMERIC, m_orig_numeric_locale.data());
        // #else
        //     uselocale(m_original_locale);
        //     freelocale(m_new_locale);
        // #endif
    }
}

// ---------------------------------------------------------------------------
// is_decimal_separator_point  (LocalesUtils.cpp:46-51)
// ---------------------------------------------------------------------------

/// A function to check that current C locale uses decimal point as a separator.
/// Intended mostly for asserts.
///
/// LocalesUtils.cpp:46-51
///
/// ```cpp
/// bool is_decimal_separator_point()
/// {
///     char str[5] = "";
///     sprintf(str, "%.1f", 0.5f);
///     return str[1] == '.';
/// }
/// ```
///
/// Rust always formats `0.5` as `"0.5"` (locale-independent), so the second
/// character is always `'.'`; the result is unconditionally `true`. We still
/// perform the format-and-inspect to mirror the C++ logic byte for byte.
pub fn is_decimal_separator_point() -> bool {
    // char str[5] = "";
    // sprintf(str, "%.1f", 0.5f);
    let str = format!("{:.1}", 0.5_f32);
    // return str[1] == '.';
    str.as_bytes().get(1).copied() == Some(b'.')
}

// ---------------------------------------------------------------------------
// string_to_double_decimal_point  (LocalesUtils.cpp:54-61)
// ---------------------------------------------------------------------------

/// A substitute for `strtod` that always treats `'.'` as the decimal point.
///
/// LocalesUtils.cpp:54-61
///
/// ```cpp
/// double string_to_double_decimal_point(const std::string_view str, size_t* pos)
/// {
///     double out;
///     size_t p = fast_float::from_chars(str.data(), str.data() + str.size(), out).ptr - str.data();
///     if (pos)
///         *pos = p;
///     return out;
/// }
/// ```
///
/// `fast_float::from_chars` parses the longest valid floating-point prefix
/// (matching `std::from_chars` with `chars_format::general`) and returns a
/// pointer just past the consumed characters; on failure that pointer equals
/// the start, i.e. `p == 0`, and `out` is left untouched. We faithfully return
/// the parsed value together with the number of consumed *bytes* `p`. To keep
/// the C++ contract — where `out` is whatever the caller's stack held when no
/// parse occurred (in practice ignored because callers test `pos`) — we return
/// `0.0` for the value when nothing is consumed.
pub fn string_to_double_decimal_point(str: &str) -> (f64, usize) {
    let mut out: f64;
    // size_t p = fast_float::from_chars(...).ptr - str.data();
    let p = match from_chars(str.as_bytes()) {
        Some((value, consumed)) => {
            out = value;
            consumed
        }
        None => {
            // `out` is uninitialised in C++ on failure; callers rely on
            // `p == 0` to detect this. Use 0.0 as a defined value.
            out = 0.0;
            0
        }
    };
    // if (pos) *pos = p;  — `pos` is returned alongside the value in Rust.
    // return out;
    let _ = &mut out;
    (out, p)
}

// ---------------------------------------------------------------------------
// float_to_string_decimal_point  (LocalesUtils.cpp:63-85)
// ---------------------------------------------------------------------------

/// A substitute for `std::to_string` that always uses `'.'` as the decimal
/// separator. `precision < 0` selects the "general" format with 6 significant
/// digits (matching the C++ default of `-1`).
///
/// LocalesUtils.cpp:63-85
///
/// ```cpp
/// std::string float_to_string_decimal_point(double value, int precision)
/// {
/// #ifdef _WIN32
///     ... std::to_chars(out, out+SIZE, value, std::chars_format::fixed, precision);   // precision >= 0
///     ... std::to_chars(out, out+SIZE, value, std::chars_format::general, 6);          // precision < 0
/// #else
///     std::stringstream buf;
///     if (precision >= 0) buf << std::fixed << std::setprecision(precision);
///     buf << value;                                                                    // default: 6 sig. digits
///     return buf.str();
/// #endif
/// }
/// ```
///
/// Both branches agree on the observable output:
///   * `precision >= 0` -> fixed-point with exactly `precision` fractional
///     digits (`std::fixed` / `chars_format::fixed`).
///   * `precision < 0`  -> "general" format with 6 significant digits, which is
///     the default `std::stringstream`/`%g` precision (`chars_format::general`
///     with precision 6).
pub fn float_to_string_decimal_point(value: f64, precision: i32) -> String {
    if precision >= 0 {
        // std::fixed << std::setprecision(precision) << value
        format!("{:.*}", precision as usize, value)
    } else {
        // Default stream / chars_format::general, precision 6.
        general_format(value, 6)
    }
}

// ---------------------------------------------------------------------------
// Helpers reproducing <charconv> / fast_float behaviour with std only.
// ---------------------------------------------------------------------------

/// Reproduces `fast_float::from_chars` / `std::from_chars` with
/// `chars_format::general`: parse the longest valid floating-point prefix of
/// `bytes`, returning `(value, bytes_consumed)`, or `None` when no valid prefix
/// exists (mirroring `ptr == first`).
///
/// Grammar (matching `std::from_chars`, which `fast_float` follows):
///   * optional leading `'-'` (a leading `'+'` is *not* accepted);
///   * `inf` / `infinity` / `nan` (case-insensitive); otherwise
///   * decimal digits with an optional `'.'` and fractional digits — at least
///     one digit must appear on one side of the point;
///   * an optional exponent `e`/`E` with an optional sign and at least one
///     digit (the exponent is only consumed if well formed).
/// No leading whitespace is skipped.
fn from_chars(bytes: &[u8]) -> Option<(f64, usize)> {
    let len = bytes.len();
    let mut i = 0usize;

    // Optional minus sign (std::from_chars rejects a leading '+').
    let negative = i < len && bytes[i] == b'-';
    if negative {
        i += 1;
    }

    // inf / infinity / nan, case-insensitive.
    if let Some(rest) = bytes.get(i..) {
        if starts_with_ci(rest, b"infinity") {
            let v = f64::INFINITY;
            return Some((if negative { -v } else { v }, i + 8));
        }
        if starts_with_ci(rest, b"inf") {
            let v = f64::INFINITY;
            return Some((if negative { -v } else { v }, i + 3));
        }
        if starts_with_ci(rest, b"nan") {
            // Sign on NaN is irrelevant for our purposes.
            return Some((f64::NAN, i + 3));
        }
    }

    let mantissa_start = i;
    let mut saw_digit = false;

    // Integer part.
    while i < len && bytes[i].is_ascii_digit() {
        i += 1;
        saw_digit = true;
    }

    // Fractional part.
    if i < len && bytes[i] == b'.' {
        i += 1;
        while i < len && bytes[i].is_ascii_digit() {
            i += 1;
            saw_digit = true;
        }
    }

    // Need at least one mantissa digit to have a valid number.
    if !saw_digit {
        return None;
    }

    // Optional exponent: only consumed if fully well formed.
    if i < len && (bytes[i] == b'e' || bytes[i] == b'E') {
        let mut j = i + 1;
        if j < len && (bytes[j] == b'+' || bytes[j] == b'-') {
            j += 1;
        }
        let exp_digits_start = j;
        while j < len && bytes[j].is_ascii_digit() {
            j += 1;
        }
        if j > exp_digits_start {
            // At least one exponent digit -> accept the exponent.
            i = j;
        }
        // Otherwise leave `i` before the 'e' (exponent not part of the number).
    }

    let consumed = i;
    // SAFETY: only ASCII bytes [-, 0-9, ., e, E, +] were consumed, so the
    // slice is valid UTF-8 / ASCII and parses via Rust's locale-independent
    // float parser.
    let text = core::str::from_utf8(&bytes[mantissa_start..consumed]).ok()?;
    let magnitude: f64 = text.parse().ok()?;
    let value = if negative { -magnitude } else { magnitude };
    Some((value, consumed))
}

/// Case-insensitive ASCII prefix test for the `inf`/`nan` spellings.
fn starts_with_ci(haystack: &[u8], needle: &[u8]) -> bool {
    haystack.len() >= needle.len()
        && haystack[..needle.len()]
            .iter()
            .zip(needle)
            .all(|(a, b)| a.eq_ignore_ascii_case(b))
}

/// Reproduces `std::chars_format::general` / the default `std::stringstream`
/// `operator<<(double)` — i.e. printf `%.*g` — with `precision` significant
/// digits. This chooses between fixed and scientific notation the way `%g`
/// does and strips trailing zeros (and a trailing decimal point).
fn general_format(value: f64, precision: usize) -> String {
    // %g treats precision 0 as 1.
    let prec = if precision == 0 { 1 } else { precision };

    if value == 0.0 {
        // Preserve the sign of negative zero like the C++ streams do.
        return if value.is_sign_negative() {
            "-0".to_string()
        } else {
            "0".to_string()
        };
    }
    if value.is_nan() {
        return "nan".to_string();
    }
    if value.is_infinite() {
        return if value < 0.0 {
            "-inf".to_string()
        } else {
            "inf".to_string()
        };
    }

    // Decimal exponent X of `value` (i.e. value = m * 10^X, 1 <= |m| < 10).
    let exp = value.abs().log10().floor() as i32;

    // Per C's %g: use scientific notation if exponent < -4 or >= precision,
    // otherwise fixed notation. The number of significant digits is `prec`.
    if exp < -4 || exp >= prec as i32 {
        // Scientific: prec-1 fractional digits in the mantissa.
        let s = format!("{:.*e}", prec - 1, value);
        strip_scientific(&s)
    } else {
        // Fixed: (prec - 1 - exp) fractional digits gives `prec` sig. digits.
        let frac = (prec as i32 - 1 - exp).max(0) as usize;
        let s = format!("{:.*}", frac, value);
        strip_fixed(&s)
    }
}

/// Strip trailing zeros (and a dangling decimal point) from a fixed-point
/// string, mirroring `%g`'s removal of insignificant trailing zeros.
fn strip_fixed(s: &str) -> String {
    if !s.contains('.') {
        return s.to_string();
    }
    let trimmed = s.trim_end_matches('0');
    let trimmed = trimmed.trim_end_matches('.');
    trimmed.to_string()
}

/// Normalise Rust's `{:e}` output to a `%g`-style scientific string: strip
/// insignificant trailing zeros in the mantissa and format the exponent as
/// `e[+-]NN` with at least two digits, matching C's `printf`.
fn strip_scientific(s: &str) -> String {
    let (mantissa, exp) = match s.split_once(['e', 'E']) {
        Some((m, e)) => (m, e),
        None => return s.to_string(),
    };

    // Trim trailing zeros / point from the mantissa.
    let mantissa = if mantissa.contains('.') {
        let t = mantissa.trim_end_matches('0');
        t.trim_end_matches('.')
    } else {
        mantissa
    };

    // Rust emits the exponent without a sign for non-negatives and without
    // zero padding; C's %g uses a sign and at least two digits.
    let (sign, digits) = if let Some(rest) = exp.strip_prefix('-') {
        ('-', rest)
    } else if let Some(rest) = exp.strip_prefix('+') {
        ('+', rest)
    } else {
        ('+', exp)
    };
    let digits = if digits.len() < 2 {
        format!("{:0>2}", digits)
    } else {
        digits.to_string()
    };

    format!("{}e{}{}", mantissa, sign, digits)
}

// } // namespace Slic3r  (LocalesUtils.cpp:88)

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raii_setter_is_noop_and_constructible() {
        let _setter = CNumericLocalesSetter::new();
        let _default = CNumericLocalesSetter::default();
        assert!(is_decimal_separator_point());
    }

    #[test]
    fn is_decimal_separator_point_true() {
        assert!(is_decimal_separator_point());
    }

    #[test]
    fn string_to_double_basic() {
        let (v, p) = string_to_double_decimal_point("123.456");
        assert!((v - 123.456).abs() < 1e-9);
        assert_eq!(p, 7);
    }

    #[test]
    fn string_to_double_negative() {
        let (v, p) = string_to_double_decimal_point("-45.67");
        assert!((v - (-45.67)).abs() < 1e-9);
        assert_eq!(p, 6);
    }

    #[test]
    fn string_to_double_leading_plus_rejected_like_from_chars() {
        // std::from_chars / fast_float reject a leading '+'.
        let (_v, p) = string_to_double_decimal_point("+5");
        assert_eq!(p, 0);
    }

    #[test]
    fn string_to_double_scientific() {
        let (v, p) = string_to_double_decimal_point("1.23e5");
        assert!((v - 123000.0).abs() < 1e-6);
        assert_eq!(p, 6);
    }

    #[test]
    fn string_to_double_trailing_garbage() {
        let (v, p) = string_to_double_decimal_point("3.14abc");
        assert!((v - 3.14).abs() < 1e-9);
        assert_eq!(p, 4);
    }

    #[test]
    fn string_to_double_only_consumes_first_float() {
        // "1.2.3" parses "1.2" and stops at the second dot.
        let (v, p) = string_to_double_decimal_point("1.2.3");
        assert!((v - 1.2).abs() < 1e-9);
        assert_eq!(p, 3);
    }

    #[test]
    fn string_to_double_dangling_exponent() {
        // "5e" -> exponent has no digits, so only "5" is consumed.
        let (v, p) = string_to_double_decimal_point("5e");
        assert!((v - 5.0).abs() < 1e-9);
        assert_eq!(p, 1);
    }

    #[test]
    fn string_to_double_integer() {
        let (v, p) = string_to_double_decimal_point("42");
        assert!((v - 42.0).abs() < 1e-9);
        assert_eq!(p, 2);
    }

    #[test]
    fn string_to_double_invalid_returns_zero_pos() {
        let (v, p) = string_to_double_decimal_point("abc");
        assert_eq!(p, 0);
        assert_eq!(v, 0.0);
    }

    #[test]
    fn float_to_string_fixed_precision() {
        assert_eq!(float_to_string_decimal_point(3.14159, 2), "3.14");
        assert_eq!(float_to_string_decimal_point(3.7, 0), "4");
        assert_eq!(float_to_string_decimal_point(-42.5, 1), "-42.5");
        assert_eq!(float_to_string_decimal_point(1234567.89, 2), "1234567.89");
    }

    #[test]
    fn float_to_string_general_matches_printf_g() {
        // %.6g semantics for the default precision == -1.
        assert_eq!(float_to_string_decimal_point(123.456, -1), "123.456");
        assert_eq!(float_to_string_decimal_point(0.5, -1), "0.5");
        assert_eq!(float_to_string_decimal_point(0.0, -1), "0");
        // 1234567.89 has 8 sig. digits; %.6g -> scientific 1.23457e+06.
        assert_eq!(float_to_string_decimal_point(1234567.89, -1), "1.23457e+06");
        // Small magnitude -> scientific once exponent < -4.
        assert_eq!(float_to_string_decimal_point(0.0001234, -1), "0.0001234");
        assert_eq!(float_to_string_decimal_point(0.00001234, -1), "1.234e-05");
        // Trailing zeros stripped.
        assert_eq!(float_to_string_decimal_point(1.0, -1), "1");
        assert_eq!(float_to_string_decimal_point(100.0, -1), "100");
    }

    #[test]
    fn raii_drop_restores_invariant() {
        {
            let _setter = CNumericLocalesSetter::new();
            assert!(is_decimal_separator_point());
        }
        assert!(is_decimal_separator_point());
    }
}
