//! 1:1 port of `libslic3r/format.hpp`.
//!
//! NOTE on file placement: the C++ source lives at `src/libslic3r/format.hpp`
//! and the PORT_LEDGER maps it to `src/format.rs`. That path collides with the
//! existing `src/format/` module directory (the port of BambuStudio's
//! `src/libslic3r/Format/` model-IO subdir), so the Rust module system cannot
//! host both `src/format.rs` and `src/format/mod.rs`. To avoid duplicating /
//! breaking the model-IO module, this header is ported here as a submodule and
//! re-exported, while the public macro `slic3r_format!` is `#[macro_export]`ed
//! at crate root to mirror `Slic3r::format(...)`.
//
// format.hpp:1   #ifndef slic3r_format_hpp_
// format.hpp:2   #define slic3r_format_hpp_
// format.hpp:3
// format.hpp:4   // Functional wrapper around boost::format.
// format.hpp:5   // One day we may replace this wrapper with C++20 format
// format.hpp:6   // https://en.cppreference.com/w/cpp/utility/format/format
// format.hpp:7   // though C++20 format uses a different template pattern for position independent parameters.
// format.hpp:8   //
// format.hpp:9   // Boost::format works around the missing variadic templates by an ugly % chaining operator. The usage of boost::format looks like this:
// format.hpp:10  // (boost::format("template") % arg1 %arg2).str()
// format.hpp:11  // This wrapper allows for a nicer syntax:
// format.hpp:12  // Slic3r::format("template", arg1, arg2)
// format.hpp:13  // One can also override Slic3r::internal::format::cook() function to convert a Slic3r::format() argument to something that
// format.hpp:14  // boost::format may convert to string, see slic3r/GUI/I18N.hpp for a "cook" function to convert wxString to UTF8.
// format.hpp:16  #include <boost/format.hpp>
//
// format.hpp:18  namespace Slic3r {
//
// Rust differs from C++ here by necessity. The C++ wrapper exists only to work
// around C++'s missing variadic-template ergonomics for `boost::format`'s `%`
// chaining operator (see comment at format.hpp:9). Rust already has a
// first-class variadic, position-aware formatting facility (`std::format!`),
// which plays the exact role `boost::format` plays in the C++ code, so the
// public surface is a macro that forwards to `format!`. The internal
// `cook` / `format_recursive` helpers are ported below to preserve the source
// structure and the `cook` customization point.

// format.hpp:20  // https://gist.github.com/gchudnov/6a90d51af004d97337ec
// format.hpp:21  namespace internal {
// format.hpp:22      namespace format {
pub mod internal {
    pub mod format {
        // format.hpp:23  // Default "cook" function - just forward.
        // format.hpp:24  template<typename T>
        // format.hpp:25  inline T&& cook(T&& arg) {
        // format.hpp:26      return std::forward<T>(arg);
        // format.hpp:27  }
        //
        // The default `cook` is a perfect-forwarding identity. In Rust the
        // identity passthrough is simply returning the value unchanged; the
        // customization point (cf. format.hpp:13-14, slic3r/GUI/I18N.hpp) is
        // preserved by letting callers shadow `cook` with their own.
        #[inline]
        pub fn cook<T>(arg: T) -> T {
            arg
        }

        // format.hpp:29  // End of the recursive chain.
        // format.hpp:30  inline std::string format_recursive(boost::format& message) {
        // format.hpp:31      return message.str();
        // format.hpp:32  }
        //
        // boost::format accumulates `%`-fed arguments into a stateful
        // `message` object and `.str()` renders it. Rust's `format!` is not
        // stateful in the same way; the recursion that fed arguments one at a
        // time (format.hpp:34-39) collapses into a single `format!` expansion
        // in the `slic3r_format!` macro below. This terminal overload, which
        // just renders the accumulated message, corresponds to a `format!`
        // call with no further arguments.
        #[inline]
        pub fn format_recursive(message: &str) -> String {
            // format.hpp:31  return message.str();
            message.to_string()
        }

        // format.hpp:34  template<typename TValue, typename... TArgs>
        // format.hpp:35  std::string format_recursive(boost::format& message, TValue&& arg, TArgs&&... args) {
        // format.hpp:36      // Format, possibly convert the argument by the "cook" function.
        // format.hpp:37      message % cook(std::forward<TValue>(arg));
        // format.hpp:38      return format_recursive(message, std::forward<TArgs>(args)...);
        // format.hpp:39  }
        //
        // The recursive variadic argument-feeding overload. There is no
        // by-value variadic equivalent in stable Rust that preserves the
        // boost `%`-chaining semantics, and `format!` already feeds all
        // arguments positionally in one shot. The `cook` customization point
        // from format.hpp:37 is exposed by the `slic3r_format!` macro, which
        // applies `cook` to every argument before substitution.
    }
}

// format.hpp:43  template<typename... TArgs>
// format.hpp:44  inline std::string format(const char* fmt, TArgs&&... args) {
// format.hpp:45      boost::format message(fmt);
// format.hpp:46      return internal::format::format_recursive(message, std::forward<TArgs>(args)...);
// format.hpp:47  }
//
// format.hpp:49  template<typename... TArgs>
// format.hpp:50  inline std::string format(const std::string& fmt, TArgs&&... args) {
// format.hpp:51      boost::format message(fmt);
// format.hpp:52      return internal::format::format_recursive(message, std::forward<TArgs>(args)...);
// format.hpp:53  }
//
// Both `format` overloads (for `const char*` and `const std::string&`) collapse
// into a single `slic3r_format!` macro in Rust, because `format!` accepts any
// string-literal template uniformly. Each argument is routed through
// `crate::format::slic3r_format::internal::format::cook` to preserve the
// customization point (format.hpp:37). NOTE: unlike the C++ wrapper, the format
// template is a Rust `format!` template (`{}` / `{0}`) rather than a
// `boost::format` / printf template (`%s` / `%1%`); callers porting C++ call
// sites must translate the directives accordingly.
#[macro_export]
macro_rules! slic3r_format {
    ($fmt:expr $(, $arg:expr)* $(,)?) => {
        ::std::format!(
            $fmt
            $(, $crate::format::slic3r_format::internal::format::cook($arg))*
        )
    };
}

// format.hpp:55  } // namespace Slic3r
// format.hpp:57  #endif // slic3r_format_hpp_

#[cfg(test)]
mod tests {
    use super::internal::format::{cook, format_recursive};

    #[test]
    fn cook_is_identity() {
        assert_eq!(cook(42), 42);
        assert_eq!(cook("abc"), "abc");
    }

    #[test]
    fn format_recursive_terminal_renders_message() {
        assert_eq!(format_recursive("done"), "done".to_string());
    }

    #[test]
    fn slic3r_format_forwards_to_format() {
        // Mirrors Slic3r::format("Plate %d: %s", 1, "x") shape, using Rust
        // `format!` directives.
        assert_eq!(slic3r_format!("Plate {}: {}", 1, "x"), "Plate 1: x");
        assert_eq!(slic3r_format!("no args"), "no args");
        assert_eq!(slic3r_format!("trailing {},", 7), "trailing 7,");
    }
}
