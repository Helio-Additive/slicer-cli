//! Internationalization (I18N) support for translatable strings
//!
//! Provides a translation callback system that allows GUI applications
//! to hook in their translation functions while the library remains
//! GUI-agnostic. Strings pass through unchanged unless a translation
//! callback is registered.
//!
//! C++ Reference: I18N.hpp

use std::sync::Mutex;

/// Type alias for translation function callbacks
/// I18N.hpp:15
pub type TranslateFn = fn(&str) -> String;

/// Global translation function callback (protected by mutex for thread safety)
/// I18N.hpp:16
static TRANSLATE_FN: Mutex<Option<TranslateFn>> = Mutex::new(None);

/// Set the translation callback function
/// I18N.hpp:17
pub fn set_translate_callback(callback: TranslateFn) {
    if let Ok(mut fn_guard) = TRANSLATE_FN.lock() {
        *fn_guard = Some(callback);
    }
}

/// Clear the translation callback (reset to passthrough mode)
/// I18N.hpp:17 (utility)
pub fn clear_translate_callback() {
    if let Ok(mut fn_guard) = TRANSLATE_FN.lock() {
        *fn_guard = None;
    }
}

/// Translate a string using the registered callback, or return as-is
/// I18N.hpp:18 (translate(const std::string&)) and I18N.hpp:19 (translate(const char*));
/// the two C++ overloads collapse into one Rust fn since `&str` covers both cases.
pub fn translate(s: &str) -> String {
    if let Ok(fn_guard) = TRANSLATE_FN.lock() {
        if let Some(callback) = *fn_guard {
            callback(s)
        } else {
            s.to_string()
        }
    } else {
        // If lock fails, return untranslated
        s.to_string()
    }
}

/// Macro for marking strings as translatable (no-op in library mode)
/// I18N.hpp:31
#[macro_export]
macro_rules! L {
    ($s:expr) => {
        $s
    };
}

/// Macro for marking strings with context (no-op in library mode)
/// I18N.hpp:32
#[macro_export]
macro_rules! L_CONTEXT {
    ($s:expr, $context:expr) => {
        $s
    };
}

/// Macro for translating strings at runtime
/// I18N.hpp:33
#[macro_export]
macro_rules! _u8L {
    ($s:expr) => {
        $crate::i18n::translate($s)
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_translate_passthrough() {
        // Without callback, strings pass through unchanged
        clear_translate_callback();
        assert_eq!(translate("Hello"), "Hello");
        assert_eq!(translate("World"), "World");
    }

    #[test]
    fn test_translate_with_callback() {
        // Set a simple uppercase "translation"
        fn uppercase_translator(s: &str) -> String {
            s.to_uppercase()
        }

        set_translate_callback(uppercase_translator);
        assert_eq!(translate("hello"), "HELLO");
        assert_eq!(translate("world"), "WORLD");

        // Clean up
        clear_translate_callback();
    }

    #[test]
    fn test_translate_callback_replacement() {
        // First callback: uppercase
        fn uppercase_translator(s: &str) -> String {
            s.to_uppercase()
        }

        // Second callback: lowercase
        fn lowercase_translator(s: &str) -> String {
            s.to_lowercase()
        }

        set_translate_callback(uppercase_translator);
        assert_eq!(translate("Test"), "TEST");

        // Replace with different callback
        set_translate_callback(lowercase_translator);
        assert_eq!(translate("Test"), "test");

        clear_translate_callback();
    }

    #[test]
    fn test_macros() {
        // Test that macros compile and work
        let s1 = L!("Hello");
        assert_eq!(s1, "Hello");

        let s2 = L_CONTEXT!("File", "menu");
        assert_eq!(s2, "File");

        clear_translate_callback();
        let s3 = _u8L!("Test");
        assert_eq!(s3, "Test");
    }

    #[test]
    fn test_translate_empty_string() {
        clear_translate_callback();
        assert_eq!(translate(""), "");

        fn mock_translator(s: &str) -> String {
            format!("[{}]", s)
        }

        set_translate_callback(mock_translator);
        assert_eq!(translate(""), "[]");

        clear_translate_callback();
    }
}
