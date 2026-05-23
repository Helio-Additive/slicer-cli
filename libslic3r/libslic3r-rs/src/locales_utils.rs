//! Locale utilities for numeric parsing and formatting
//!
//! C++ Reference: LocalesUtils.hpp, LocalesUtils.cpp
//!
//! This module provides utilities for handling numeric locale issues,
//! ensuring that decimal points are used consistently regardless of system locale.


/// RAII wrapper that sets LC_NUMERIC to "C" on construction
/// and restores the old value on destruction.
///
/// In Rust, we don't need platform-specific locale manipulation since
/// Rust's standard library always uses "C" locale for numeric parsing/formatting.
/// This struct is provided for API compatibility but is essentially a no-op.
///
/// LocalesUtils.hpp:17-30
#[derive(Debug)]
pub struct CNumericLocalesSetter {
    // Rust doesn't need locale manipulation - kept for API compatibility
    _marker: std::marker::PhantomData<()>,
}

impl CNumericLocalesSetter {
    /// Create a new numeric locale setter
    ///
    /// In C++, this sets LC_NUMERIC to "C". In Rust, this is a no-op
    /// since Rust always uses "C" locale for numeric operations.
    ///
    /// LocalesUtils.cpp:14-31
    pub fn new() -> Self {
        Self {
            _marker: std::marker::PhantomData,
        }
    }
}

impl Default for CNumericLocalesSetter {
    fn default() -> Self {
        Self::new()
    }
}

/// Drop implementation restores original locale
///
/// In C++, this restores the original LC_NUMERIC. In Rust, this is a no-op.
///
/// LocalesUtils.cpp:35-42
impl Drop for CNumericLocalesSetter {
    fn drop(&mut self) {
        // No-op in Rust - kept for API compatibility
    }
}

/// Check that current locale uses decimal point as separator
///
/// In Rust, this always returns true since Rust's standard library
/// always uses "." as the decimal separator.
///
/// LocalesUtils.cpp:46-51
pub fn is_decimal_separator_point() -> bool {
    // Rust always uses "." as decimal separator
    true
}

/// Convert string to double using decimal point separator
///
/// Parses a floating-point number from a string, ensuring that
/// "." is used as the decimal separator regardless of system locale.
///
/// # Arguments
/// * `s` - String to parse
///
/// # Returns
/// Tuple of (parsed value, number of characters consumed)
///
/// LocalesUtils.cpp:54-62
pub fn string_to_double_decimal_point(s: &str) -> Result<(f64, usize), std::num::ParseFloatError> {
    // Find the end of the numeric portion
    let end = s
        .char_indices()
        .take_while(|(_, c)| {
            c.is_ascii_digit() || *c == '.' || *c == '-' || *c == '+' || *c == 'e' || *c == 'E'
        })
        .last()
        .map(|(i, c)| i + c.len_utf8())
        .unwrap_or(0);

    if end == 0 {
        return Err("".parse::<f64>().unwrap_err());
    }

    let numeric_part = &s[..end];
    let value = numeric_part.parse::<f64>()?;
    Ok((value, end))
}

/// Convert float to string using decimal point separator
///
/// Formats a floating-point number as a string, ensuring that
/// "." is used as the decimal separator regardless of system locale.
///
/// # Arguments
/// * `value` - The floating-point value to format
/// * `precision` - Number of decimal places (-1 for automatic)
///
/// LocalesUtils.cpp:64-85
pub fn float_to_string_decimal_point(value: f64, precision: i32) -> String {
    if precision >= 0 {
        // Fixed precision
        format!("{:.prec$}", value, prec = precision as usize)
    } else {
        // Automatic precision (general format)
        // Rust's default Display uses appropriate precision
        value.to_string()
    }
}

/// Convert float to string with specified precision (convenience wrapper)
pub fn float_to_string_decimal_point_with_precision(value: f64, precision: usize) -> String {
    float_to_string_decimal_point(value, precision as i32)
}

/// Parse a double from string view, with optional position output
///
/// This is a convenience wrapper around string_to_double_decimal_point
/// that matches the C++ API more closely.
pub fn parse_double(s: &str) -> Option<f64> {
    string_to_double_decimal_point(s).ok().map(|(v, _)| v)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_numeric_locale_setter() {
        let _setter = CNumericLocalesSetter::new();
        // Should not panic or cause issues
        assert!(is_decimal_separator_point());
    }

    #[test]
    fn test_is_decimal_separator_point() {
        assert!(is_decimal_separator_point());
    }

    #[test]
    fn test_string_to_double_basic() {
        let (value, pos) = string_to_double_decimal_point("123.456").unwrap();
        assert!((value - 123.456).abs() < 0.0001);
        assert_eq!(pos, 7);
    }

    #[test]
    fn test_string_to_double_negative() {
        let (value, pos) = string_to_double_decimal_point("-45.67").unwrap();
        assert!((value - (-45.67)).abs() < 0.0001);
        assert_eq!(pos, 6);
    }

    #[test]
    fn test_string_to_double_scientific() {
        let (value, _) = string_to_double_decimal_point("1.23e5").unwrap();
        assert!((value - 123000.0).abs() < 0.1);
    }

    #[test]
    fn test_string_to_double_with_trailing() {
        let (value, pos) = string_to_double_decimal_point("3.14abc").unwrap();
        assert!((value - 3.14).abs() < 0.0001);
        assert_eq!(pos, 4);
    }

    #[test]
    fn test_string_to_double_integer() {
        let (value, pos) = string_to_double_decimal_point("42").unwrap();
        assert!((value - 42.0).abs() < 0.0001);
        assert_eq!(pos, 2);
    }

    #[test]
    fn test_string_to_double_invalid() {
        let result = string_to_double_decimal_point("abc");
        assert!(result.is_err());
    }

    #[test]
    fn test_float_to_string_default() {
        let s = float_to_string_decimal_point(123.456, -1);
        assert!(s.contains("123.456") || s.starts_with("123.456"));
    }

    #[test]
    fn test_float_to_string_fixed_precision() {
        let s = float_to_string_decimal_point(3.14159, 2);
        assert_eq!(s, "3.14");
    }

    #[test]
    fn test_float_to_string_zero_precision() {
        let s = float_to_string_decimal_point(3.7, 0);
        assert_eq!(s, "4");
    }

    #[test]
    fn test_float_to_string_negative() {
        let s = float_to_string_decimal_point(-42.5, 1);
        assert_eq!(s, "-42.5");
    }

    #[test]
    fn test_float_to_string_large() {
        let s = float_to_string_decimal_point(1234567.89, 2);
        assert_eq!(s, "1234567.89");
    }

    #[test]
    fn test_parse_double() {
        assert_eq!(parse_double("3.14"), Some(3.14));
        assert_eq!(parse_double("invalid"), None);
    }

    #[test]
    fn test_raii_behavior() {
        {
            let _setter = CNumericLocalesSetter::new();
            assert!(is_decimal_separator_point());
        }
        // After drop, should still work
        assert!(is_decimal_separator_point());
    }

    #[test]
    fn test_multiple_setters() {
        let _setter1 = CNumericLocalesSetter::new();
        let _setter2 = CNumericLocalesSetter::new();
        assert!(is_decimal_separator_point());
    }

    #[test]
    fn test_float_to_string_with_precision_wrapper() {
        let s = float_to_string_decimal_point_with_precision(3.14159, 3);
        assert_eq!(s, "3.142");
    }
}
