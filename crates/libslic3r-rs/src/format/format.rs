//! Port of `libslic3r/format.hpp` of Bambu Studio at the pin: `Slic3r::format(fmt, args...)`, the
//! functional wrapper around `boost::format`.
//!
//! The file is `format/format.rs` because `src/format.rs` cannot stand beside the directory
//! `src/format/` (the port of `libslic3r/Format/`), whose module file is `format/mod.rs`.
//!
//! The header is four functions over `boost::format`, so what it does is what `boost::format`
//! does: the second half of this file is the port of the part of Boost.Format the wrapper reaches
//! (`boost/format/parsing.hpp`, `feed_args.hpp`, `internals.hpp`, `format_implementation.hpp`,
//! `exceptions.hpp`, Boost 1.90), and of what `operator<<` of a `std::ostream` writes for the
//! argument types given here, as libc++ (the reference build) writes it.
//!
//! A format string is a `boost::format` one: `%1%`, `%2$5.3f`, `%|1$+5|`, the printf directives
//! (`%d`, `%s`, `%.3f`, `%-8x`...), `%%`, the tabulations `%20t` and `%20T.`. The directive gives
//! the stream state, the argument its type: `%d` fed a text prints the text.
//!
//! Exceptions. `boost::format` is made with every error bit set, so an ill-formed format string
//! and a wrong number of arguments throw; here they are the `Err` of [`format`].
//!
//! FIDELITY-NOTE: bytes. A `std::string` is bytes and a `%.3s` cuts after three of them, in the
//! middle of a UTF-8 sequence if it comes to that. [`BoostFormat::str`] returns the bytes;
//! [`format`] returns a `String`, with U+FFFD where such a cut left an incomplete sequence.
//!
//! FIDELITY-NOTE: argument types. The arguments are the types [`FormatArg`] is implemented for:
//! the integers of 16, 32 and 64 bits, `f32`, `f64`, `bool`, `char` and texts. Not there: `i8` /
//! `u8` (a `signed char` / `unsigned char` prints as a character in C++), `long double`, pointers
//! (`%p` of a `void*`), the manipulators of `boost::io::group`, and a locale other than "C". A
//! `char` that is not ASCII is written as its UTF-8 bytes, where a C++ `char` is one byte.

use std::cell::Cell;

// ---------------------------------------------------------------------------------------------
// format.hpp
// ---------------------------------------------------------------------------------------------

// Functional wrapper around boost::format.
// One day we may replace this wrapper with C++20 format
// https://en.cppreference.com/w/cpp/utility/format/format
// though C++20 format uses a different template pattern for position independent parameters.
//
// Boost::format works around the missing variadic templates by an ugly % chaining operator. The usage of boost::format looks like this:
// (boost::format("template") % arg1 %arg2).str()
// This wrapper allows for a nicer syntax:
// Slic3r::format("template", arg1, arg2)
// One can also override Slic3r::internal::format::cook() function to convert a Slic3r::format() argument to something that
// boost::format may convert to string, see slic3r/GUI/I18N.hpp for a "cook" function to convert wxString to UTF8.

// https://gist.github.com/gchudnov/6a90d51af004d97337ec
pub mod internal {
    pub mod format {
        use super::super::{BoostFormat, FormatArg, FormatError};

        /// format.hpp:23-27
        ///
        /// Default "cook" function - just forward.
        #[inline]
        pub fn cook<T>(arg: T) -> T {
            arg
        }

        /// format.hpp:29-39, the two overloads: the end of the recursive chain (`args` empty) and
        /// the step that feeds one argument and goes on with the rest.
        pub fn format_recursive(message: &mut BoostFormat, args: &[&dyn FormatArg]) -> Result<String, FormatError> {
            match args.split_first() {
                // End of the recursive chain.
                None => message.str().map(|bytes| String::from_utf8_lossy(&bytes).into_owned()),
                Some((arg, args)) => {
                    // Format, possibly convert the argument by the "cook" function.
                    message.feed(cook(*arg))?;
                    format_recursive(message, args)
                }
            }
        }
    }
}

/// format.hpp:43-53, the overloads for `const char*` and `const std::string&`.
pub fn format(fmt: &str, args: &[&dyn FormatArg]) -> Result<String, FormatError> {
    let mut message = BoostFormat::new(fmt.as_bytes())?;
    internal::format::format_recursive(&mut message, args)
}

/// `Slic3r::format(fmt, args...)` with the arguments written as in C++:
/// `slic3r_format!("Plate %d: %s", 1, "x")` is `format("Plate %d: %s", &[&1, &"x"])`.
#[macro_export]
macro_rules! slic3r_format {
    ($fmt:expr $(, $arg:expr)* $(,)?) => {
        $crate::format::format::format($fmt, &[$(&$arg as &dyn $crate::format::format::FormatArg),*])
    };
}

// ---------------------------------------------------------------------------------------------
// boost/format/exceptions.hpp
// ---------------------------------------------------------------------------------------------

/// The exceptions of `boost::io` a `boost::format` with `all_error_bits` throws.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FormatError {
    /// exceptions.hpp:41-52 `bad_format_string(pos, size)`
    BadFormatString { pos: usize, size: usize },
    /// exceptions.hpp:54-66 `too_few_args(cur, expected)`
    TooFewArgs { cur: usize, expected: usize },
    /// exceptions.hpp:68-80 `too_many_args(cur, expected)`
    TooManyArgs { cur: usize, expected: usize },
    /// Not of Boost: the `std::length_error` that `std::string::append` of libc++ throws out of
    /// `put` (feed_args.hpp:267) for a directive that asks for internal padding, a space before
    /// the number and a truncation to nothing at once (`%|_ 1.0s|`), where the count it is given
    /// has wrapped around.
    LengthError,
}

impl FormatError {
    /// `what()`
    pub fn what(&self) -> &'static str {
        match self {
            FormatError::BadFormatString { .. } => "boost::bad_format_string: format-string is ill-formed",
            FormatError::TooFewArgs { .. } => "boost::too_few_args: format-string referred to more arguments than were passed",
            FormatError::TooManyArgs { .. } => "boost::too_many_args: format-string referred to fewer arguments than were passed",
            FormatError::LengthError => "basic_string",
        }
    }
}

impl std::fmt::Display for FormatError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.what())
    }
}

impl std::error::Error for FormatError {}

// ---------------------------------------------------------------------------------------------
// std::ios_base and operator<<, as libc++ has them
// ---------------------------------------------------------------------------------------------

/// `std::ios_base::fmtflags`, the bits Boost.Format sets and reads.
#[allow(non_upper_case_globals)]
pub mod ios_base {
    pub const boolalpha: u32 = 0x0001;
    pub const dec: u32 = 0x0002;
    pub const fixed: u32 = 0x0004;
    pub const hex: u32 = 0x0008;
    pub const internal: u32 = 0x0010;
    pub const left: u32 = 0x0020;
    pub const oct: u32 = 0x0040;
    pub const right: u32 = 0x0080;
    pub const scientific: u32 = 0x0100;
    pub const showbase: u32 = 0x0200;
    pub const showpoint: u32 = 0x0400;
    pub const showpos: u32 = 0x0800;
    pub const skipws: u32 = 0x1000;
    pub const uppercase: u32 = 0x4000;
    pub const adjustfield: u32 = left | right | internal;
    pub const basefield: u32 = dec | oct | hex;
    pub const floatfield: u32 = scientific | fixed;
}

/// The format state of the `std::ostream` an argument is written to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OStream {
    pub flags: u32,
    pub width: i64,
    pub precision: i64,
    pub fill: u8,
}

impl OStream {
    /// A stream as constructed: `dec | skipws`, width 0, precision 6, fill `' '`.
    fn new() -> Self {
        OStream { flags: ios_base::dec | ios_base::skipws, width: 0, precision: 6, fill: b' ' }
    }

    /// libc++ `__pad_and_output(s, ob, op, oe, iob, fl)`: `[ob, op)`, the fill up to the width,
    /// `[op, oe)`.
    fn pad_and_output(&self, text: &[u8], op: usize) -> Vec<u8> {
        let mut out = Vec::with_capacity(text.len());
        out.extend_from_slice(&text[..op]);
        if self.width > text.len() as i64 {
            out.resize(op + (self.width as usize - text.len()), self.fill);
        }
        out.extend_from_slice(&text[op..]);
        out
    }

    /// libc++ `__num_put_base::__identify_padding`, then the padding: a number.
    fn pad_number(&self, text: &[u8]) -> Vec<u8> {
        let adjust = self.flags & ios_base::adjustfield;
        let op = if adjust == ios_base::left {
            text.len()
        } else if adjust == ios_base::internal {
            if !text.is_empty() && (text[0] == b'-' || text[0] == b'+') {
                1
            } else if text.len() >= 2 && text[0] == b'0' && (text[1] == b'x' || text[1] == b'X') {
                2
            } else {
                0
            }
        } else {
            0
        };
        self.pad_and_output(text, op)
    }

    /// libc++ `__put_character_sequence`: a text or a character, padded on the right for `left`
    /// and on the left otherwise.
    fn pad_text(&self, text: &[u8]) -> Vec<u8> {
        let op = if self.flags & ios_base::adjustfield == ios_base::left { text.len() } else { 0 };
        self.pad_and_output(text, op)
    }

    /// `num_put::do_put` of an integer (libc++ `__format_int` and the `printf` it calls).
    /// `value` is the integer, `bits` the width of its C++ type, `signed` whether that is signed:
    /// in octal and hexadecimal a negative number is written as the unsigned number of its type.
    fn put_integer(&self, value: i128, bits: u32, signed: bool) -> Vec<u8> {
        let basefield = self.flags & ios_base::basefield;
        let showbase = self.flags & ios_base::showbase != 0;
        let unsigned: u64 = if bits == 64 { value as u64 } else { (value as u64) & ((1u64 << bits) - 1) };
        let text = if basefield == ios_base::oct {
            let digits = format!("{:o}", unsigned);
            if showbase && unsigned != 0 {
                format!("0{digits}")
            } else {
                digits
            }
        } else if basefield == ios_base::hex {
            let upper = self.flags & ios_base::uppercase != 0;
            let digits = if upper { format!("{:X}", unsigned) } else { format!("{:x}", unsigned) };
            if showbase && unsigned != 0 {
                format!("{}{digits}", if upper { "0X" } else { "0x" })
            } else {
                digits
            }
        } else if signed {
            if self.flags & ios_base::showpos != 0 && value >= 0 {
                format!("+{value}")
            } else {
                value.to_string()
            }
        } else {
            unsigned.to_string()
        };
        self.pad_number(text.as_bytes())
    }

    /// `num_put::do_put(double)` (libc++ `__format_float` and the `printf` it calls).
    fn put_double(&self, value: f64) -> Vec<u8> {
        let floatfield = self.flags & ios_base::floatfield;
        let upper = self.flags & ios_base::uppercase != 0;
        let plus = self.flags & ios_base::showpos != 0;
        let alt = self.flags & ios_base::showpoint != 0;
        let precision = if self.precision < 0 { 6 } else { self.precision as usize };
        let text = if floatfield == (ios_base::fixed | ios_base::scientific) {
            // The precision is not given to "%a".
            printf_a(value, plus, alt, upper)
        } else if floatfield == ios_base::fixed {
            printf_f(value, precision, plus, alt, upper)
        } else if floatfield == ios_base::scientific {
            printf_e(value, precision, plus, alt, upper)
        } else {
            printf_g(value, precision, plus, alt, upper)
        };
        self.pad_number(text.as_bytes())
    }
}

/// The sign `printf` writes, and the text of what is not finite.
fn printf_sign_and_special(value: f64, plus: bool, upper: bool) -> (&'static str, Option<&'static str>) {
    // The libc of the reference build writes a NaN without a sign, `+` flag or not.
    let sign = if value.is_nan() {
        ""
    } else if value.is_sign_negative() {
        "-"
    } else if plus {
        "+"
    } else {
        ""
    };
    let special = if value.is_nan() {
        Some(if upper { "NAN" } else { "nan" })
    } else if value.is_infinite() {
        Some(if upper { "INF" } else { "inf" })
    } else {
        None
    };
    (sign, special)
}

/// `{:.*e}` of Rust as the `d.ddde±dd` of C.
fn c_exponent(mantissa_and_exponent: &str, alt: bool, upper: bool) -> String {
    let (mantissa, exponent) = mantissa_and_exponent.split_once('e').expect("an exponent");
    let exponent: i32 = exponent.parse().expect("an exponent");
    let mut out = mantissa.to_string();
    if alt && !mantissa.contains('.') {
        out.push('.');
    }
    out.push(if upper { 'E' } else { 'e' });
    out.push(if exponent < 0 { '-' } else { '+' });
    out.push_str(&format!("{:02}", exponent.abs()));
    out
}

/// `printf("%.*e")`
fn printf_e(value: f64, precision: usize, plus: bool, alt: bool, upper: bool) -> String {
    let (sign, special) = printf_sign_and_special(value, plus, upper);
    match special {
        Some(special) => format!("{sign}{special}"),
        None => format!("{sign}{}", c_exponent(&format!("{:.*e}", precision, value.abs()), alt, upper)),
    }
}

/// `printf("%.*f")`
fn printf_f(value: f64, precision: usize, plus: bool, alt: bool, upper: bool) -> String {
    let (sign, special) = printf_sign_and_special(value, plus, upper);
    match special {
        Some(special) => format!("{sign}{special}"),
        None => {
            let mut digits = format!("{:.*}", precision, value.abs());
            if alt && precision == 0 {
                digits.push('.');
            }
            format!("{sign}{digits}")
        }
    }
}

/// `printf("%.*g")`
fn printf_g(value: f64, precision: usize, plus: bool, alt: bool, upper: bool) -> String {
    let (sign, special) = printf_sign_and_special(value, plus, upper);
    if let Some(special) = special {
        return format!("{sign}{special}");
    }
    let value = value.abs();
    // "P = 6 if the precision is omitted, 1 if it is zero"; X is the exponent of the %e form.
    let p = if precision == 0 { 1 } else { precision };
    let scientific = format!("{:.*e}", p - 1, value);
    let x: i32 = scientific.split_once('e').expect("an exponent").1.parse().expect("an exponent");
    let mut digits = if (p as i32) > x && x >= -4 {
        let mut fixed = format!("{:.*}", (p as i32 - 1 - x) as usize, value);
        if !alt && fixed.contains('.') {
            let trimmed = fixed.trim_end_matches('0').trim_end_matches('.').len();
            fixed.truncate(trimmed);
        } else if alt && !fixed.contains('.') {
            fixed.push('.');
        }
        fixed
    } else {
        let (mantissa, exponent) = scientific.split_once('e').expect("an exponent");
        let mut mantissa = mantissa.to_string();
        if !alt && mantissa.contains('.') {
            let trimmed = mantissa.trim_end_matches('0').trim_end_matches('.').len();
            mantissa.truncate(trimmed);
        }
        c_exponent(&format!("{mantissa}e{exponent}"), alt, upper)
    };
    if upper {
        digits = digits.to_uppercase();
    }
    format!("{sign}{digits}")
}

/// `printf("%a")`: the hexadecimal form, with as many digits as the value needs.
fn printf_a(value: f64, plus: bool, alt: bool, upper: bool) -> String {
    let (sign, special) = printf_sign_and_special(value, plus, upper);
    if let Some(special) = special {
        return format!("{sign}{special}");
    }
    let bits = value.to_bits();
    let biased = ((bits >> 52) & 0x7ff) as i32;
    let fraction = bits & 0x000f_ffff_ffff_ffff;
    // A subnormal number is written normalized, as the libc of the reference build does:
    // 0x1p-1074.
    let (lead, exponent, fraction) = if biased == 0 {
        if fraction == 0 {
            (0, 0, 0)
        } else {
            let msb = 63 - fraction.leading_zeros() as i32;
            (1, msb - 1074, (fraction & ((1u64 << msb) - 1)) << (52 - msb))
        }
    } else {
        (1, biased - 1023, fraction)
    };
    let mut hex = format!("{:013x}", fraction);
    let trimmed = hex.trim_end_matches('0').len();
    hex.truncate(trimmed);
    let mut out = format!("0x{lead}");
    if !hex.is_empty() || alt {
        out.push('.');
        out.push_str(&hex);
    }
    out.push('p');
    out.push(if exponent < 0 { '-' } else { '+' });
    out.push_str(&exponent.abs().to_string());
    if upper {
        out = out.to_uppercase();
    }
    format!("{sign}{out}")
}

/// An argument of `boost::format`: what `os << x` writes for it under the state of `os`
/// (feed_args.hpp:97-100 `put_last`).
pub trait FormatArg {
    fn put_last(&self, os: &OStream) -> Vec<u8>;
}

impl<T: FormatArg + ?Sized> FormatArg for &T {
    fn put_last(&self, os: &OStream) -> Vec<u8> {
        (**self).put_last(os)
    }
}

macro_rules! format_arg_integer {
    ($($t:ty, $bits:expr, $signed:expr;)*) => {
        $(impl FormatArg for $t {
            fn put_last(&self, os: &OStream) -> Vec<u8> {
                os.put_integer(*self as i128, $bits, $signed)
            }
        })*
    };
}

format_arg_integer! {
    i16, 16, true;
    i32, 32, true;
    i64, 64, true;
    isize, 64, true;
    u16, 16, false;
    u32, 32, false;
    u64, 64, false;
    usize, 64, false;
}

impl FormatArg for f64 {
    fn put_last(&self, os: &OStream) -> Vec<u8> {
        os.put_double(*self)
    }
}

impl FormatArg for f32 {
    /// `os << float` is `num_put::do_put(double)`.
    fn put_last(&self, os: &OStream) -> Vec<u8> {
        os.put_double(*self as f64)
    }
}

impl FormatArg for bool {
    /// libc++ `num_put::do_put(bool)`: the name, not padded, with `boolalpha`; otherwise the
    /// `unsigned long` 0 or 1.
    fn put_last(&self, os: &OStream) -> Vec<u8> {
        if os.flags & ios_base::boolalpha == 0 {
            return os.put_integer(*self as i128, 64, false);
        }
        if *self {
            b"true".to_vec()
        } else {
            b"false".to_vec()
        }
    }
}

impl FormatArg for char {
    fn put_last(&self, os: &OStream) -> Vec<u8> {
        let mut buf = [0u8; 4];
        os.pad_text(self.encode_utf8(&mut buf).as_bytes())
    }
}

impl FormatArg for str {
    fn put_last(&self, os: &OStream) -> Vec<u8> {
        os.pad_text(self.as_bytes())
    }
}

impl FormatArg for String {
    fn put_last(&self, os: &OStream) -> Vec<u8> {
        os.pad_text(self.as_bytes())
    }
}

// ---------------------------------------------------------------------------------------------
// boost/format/internals.hpp
// ---------------------------------------------------------------------------------------------

/// internals.hpp:34-58 `stream_format_state`: set of params that define the format state of a
/// stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct StreamFormatState {
    width_: i64,
    precision_: i64,
    fill_: u8,
    flags_: u32,
}

impl StreamFormatState {
    /// internals.hpp:39
    fn new(fill: u8) -> Self {
        let mut state = StreamFormatState { width_: 0, precision_: 0, fill_: 0, flags_: 0 };
        state.reset(fill);
        state
    }

    /// internals.hpp:151-160
    fn reset(&mut self, fill: u8) {
        // set our params to standard's default state.   cf 27.4.4.1 of the C++ norm
        self.width_ = 0;
        self.precision_ = 6;
        self.fill_ = fill; // default is widen(' '), but we cant compute it without the locale
        self.flags_ = ios_base::dec | ios_base::skipws;
        // the adjust_field part is left equal to 0, which means right.
    }

    /// internals.hpp:105-127
    fn apply_on(&self, os: &mut OStream) {
        // set the state of this stream according to our params
        if self.width_ != -1 {
            os.width = self.width_;
        }
        if self.precision_ != -1 {
            os.precision = self.precision_;
        }
        if self.fill_ != 0 {
            os.fill = self.fill_;
        }
        os.flags = self.flags_;
    }
}

// internals.hpp:67 `enum pad_values`
const ZEROPAD: u32 = 1;
const SPACEPAD: u32 = 2;
const CENTERED: u32 = 4;
const TABULATION: u32 = 8;

// internals.hpp:71-74 `enum arg_values`
/// non-positional directive. will set argN later
const ARG_N_NO_POSIT: i32 = -1;
/// tabulation directive. (no argument read)
const ARG_N_TABULATION: i32 = -2;
/// ignored directive. (no argument read)
const ARG_N_IGNORED: i32 = -3;

/// internals.hpp:64-98 `format_item`: stores all parameters that can be specified in format
/// strings.
#[derive(Debug, Clone, PartialEq, Eq)]
struct FormatItem {
    /// argument number (starts at 0,  eg : %1 => argN=0)
    /// negative values for items that don't process an argument
    arg_n_: i32,
    /// result of the formatting of this item
    res_: Vec<u8>,
    /// piece of string between this item and the next
    appendix_: Vec<u8>,
    /// set by parsing, is only affected by modify_item
    fmtstate_: StreamFormatState,
    /// is set for directives like %.5s that ask truncation
    truncate_: i64,
    /// several possible padding schemes can mix. see pad_values
    pad_scheme_: u32,
}

impl FormatItem {
    /// internals.hpp:79-80
    fn new(fill: u8) -> Self {
        FormatItem {
            arg_n_: ARG_N_NO_POSIT,
            res_: Vec::new(),
            appendix_: Vec::new(),
            fmtstate_: StreamFormatState::new(fill),
            truncate_: i64::MAX,
            pad_scheme_: 0,
        }
    }

    /// internals.hpp:173-197
    fn compute_states(&mut self) {
        // reflect pad_scheme_   on  fmt_state_
        //   because some pad_schemes has complex consequences on several state params.
        if self.pad_scheme_ & ZEROPAD != 0 {
            // ignore zeropad in left alignment :
            if self.fmtstate_.flags_ & ios_base::left != 0 {
                // only left bit might be set. (not right, nor internal)
                self.pad_scheme_ &= !ZEROPAD;
            } else {
                self.pad_scheme_ &= !SPACEPAD; // printf ignores spacepad when zeropadding
                self.fmtstate_.fill_ = b'0';
                self.fmtstate_.flags_ = (self.fmtstate_.flags_ & !ios_base::adjustfield) | ios_base::internal;
                // removes all adjustfield bits, and adds internal.
            }
        }
        if self.pad_scheme_ & SPACEPAD != 0 && self.fmtstate_.flags_ & ios_base::showpos != 0 {
            self.pad_scheme_ &= !SPACEPAD;
        }
    }
}

// ---------------------------------------------------------------------------------------------
// boost/format/parsing.hpp
// ---------------------------------------------------------------------------------------------

/// parsing.hpp:70-83 `str2int`: reads the digits at `start`, returns the number (wrapping, where
/// the C++ overflows) and the place after them.
fn str2int(buf: &[u8], start: usize) -> (i64, usize) {
    let mut it = start;
    let mut res: i64 = 0;
    while it < buf.len() && buf[it].is_ascii_digit() {
        res = res.wrapping_mul(10).wrapping_add((buf[it] - b'0') as i64);
        it += 1;
    }
    (res, it)
}

/// parsing.hpp:106-417 `parse_printf_directive`. `start` is the place after the `%`; on return it
/// is the place after the directive. Every `maybe_throw_exception` of the C++ throws, the
/// exceptions being all enabled: it is the `Err` here, and what the C++ does after it where they
/// are disabled is not ported.
fn parse_printf_directive(buf: &[u8], start: &mut usize, fpar: &mut FormatItem, offset: usize) -> Result<(), FormatError> {
    let last = buf.len();
    fpar.arg_n_ = ARG_N_NO_POSIT; // if no positional-directive
    let mut precision_set = false;
    let mut in_brackets = false;
    let start0 = *start;
    let fstring_size = last - start0 + offset;
    let mut mssiz: u8 = 0;
    let bad = |start: usize| FormatError::BadFormatString { pos: start - start0 + offset, size: fstring_size };

    // The labels of the C++: where a `goto` enters the chain below.
    #[derive(PartialEq, PartialOrd)]
    enum Label {
        Flags,
        Width,
        Precision,
    }
    let mut label = Label::Flags;

    if *start >= last {
        // empty directive : this is a trailing %
        return Err(bad(*start));
    }

    if buf[*start] == b'|' {
        in_brackets = true;
        *start += 1;
        if *start >= last {
            return Err(bad(*start));
        }
    }

    // the flag '0' would be picked as a digit for argument order, but here it's a flag :
    if buf[*start] == b'0' {
        // goto parse_flags;
    } else if buf[*start].is_ascii_digit() {
        // handle argument order (%2$d)  or possibly width specification: %2d
        let (n, it) = str2int(buf, *start);
        *start = it;
        if *start >= last {
            return Err(bad(*start));
        }

        // %N% case : this is already the end of the directive
        if buf[*start] == b'%' {
            fpar.arg_n_ = (n as i32).wrapping_sub(1);
            *start += 1;
            if in_brackets {
                return Err(bad(*start));
            }
            return Ok(());
        }

        if buf[*start] == b'$' {
            fpar.arg_n_ = (n as i32).wrapping_sub(1);
            *start += 1;
        } else {
            // non-positional directive
            fpar.fmtstate_.width_ = n as i32 as i64;
            fpar.arg_n_ = ARG_N_NO_POSIT;
            label = Label::Precision;
        }
    }

    // parse_flags:
    if label <= Label::Flags {
        // handle flags
        let mut to_width = false;
        while *start != last {
            // as long as char is one of + - = _ # 0 or ' '
            match buf[*start] {
                b'\'' => {} // no effect yet. (painful to implement)
                b'-' => fpar.fmtstate_.flags_ |= ios_base::left,
                b'=' => fpar.pad_scheme_ |= CENTERED,
                b'_' => fpar.fmtstate_.flags_ |= ios_base::internal,
                b' ' => fpar.pad_scheme_ |= SPACEPAD,
                b'+' => fpar.fmtstate_.flags_ |= ios_base::showpos,
                b'0' => {
                    fpar.pad_scheme_ |= ZEROPAD;
                    // need to know alignment before really setting flags,
                    // so just add 'zeropad' flag for now, it will be processed later.
                }
                b'#' => fpar.fmtstate_.flags_ |= ios_base::showpoint | ios_base::showbase,
                _ => {
                    to_width = true;
                    break;
                }
            }
            *start += 1;
        } // loop on flag.

        if !to_width && *start >= last {
            return Err(bad(*start));
        }
    }

    // first skip 'asterisk fields' : * or num (length)
    // parse_width:
    if label <= Label::Width {
        if buf[*start] == b'*' {
            *start += 1;
        } else if *start != last && buf[*start].is_ascii_digit() {
            let (width, it) = str2int(buf, *start);
            fpar.fmtstate_.width_ = width;
            *start = it;
        }
    }

    // parse_precision:
    if *start >= last {
        return Err(bad(*start));
    }
    // handle precision spec
    if buf[*start] == b'.' {
        *start += 1;
        if *start != last && buf[*start] == b'*' {
            *start += 1;
        } else if *start != last && buf[*start].is_ascii_digit() {
            let (precision, it) = str2int(buf, *start);
            fpar.fmtstate_.precision_ = precision;
            *start = it;
            precision_set = true;
        } else {
            fpar.fmtstate_.precision_ = 0;
        }
    }

    // argument type modifiers
    while *start != last {
        match buf[*start] {
            b'h' | b'l' | b'j' | b'z' | b'L' => {
                // boost::format ignores argument type modifiers as it relies on
                // the type of the argument fed into it by operator %
            }

            // Note that the ptrdiff_t argument type 't' from C++11 is not honored
            // because it was already in use as the tabulation specifier in boost::format
            // case 't':

            // Microsoft extensions:
            // https://msdn.microsoft.com/en-us/library/tcxf1dw6.aspx
            b'w' => {}
            b'I' => mssiz = b'I',
            b'3' => {
                if mssiz != b'I' {
                    return Err(bad(*start));
                }
                mssiz = b'3';
            }
            b'2' => {
                if mssiz != b'3' {
                    return Err(bad(*start));
                }
                mssiz = 0x00;
            }
            b'6' => {
                if mssiz != b'I' {
                    return Err(bad(*start));
                }
                mssiz = b'6';
            }
            b'4' => {
                if mssiz != b'6' {
                    return Err(bad(*start));
                }
                mssiz = 0x00;
            }
            _ => {
                if mssiz != 0 && mssiz == b'I' {
                    mssiz = 0;
                }
                break;
            }
        }
        *start += 1;
    } // loop on argument type modifiers to pick up 'hh', 'll', and the more complex microsoft ones

    // parse_conversion_specification:
    if *start >= last || mssiz != 0 {
        return Err(bad(*start));
    }

    if in_brackets && buf[*start] == b'|' {
        *start += 1;
        return Ok(());
    }

    // The default flags are "dec" and "skipws"
    // so if changing the base, need to unset basefield first

    match buf[*start] {
        // Boolean
        b'b' => fpar.fmtstate_.flags_ |= ios_base::boolalpha,

        // Decimal
        b'u' | b'd' | b'i' => {
            // Defaults are sufficient
        }

        // Hex
        b'X' | b'x' | b'p' => {
            if buf[*start] == b'X' {
                fpar.fmtstate_.flags_ |= ios_base::uppercase;
            }
            // pointer => set hex.
            fpar.fmtstate_.flags_ &= !ios_base::basefield;
            fpar.fmtstate_.flags_ |= ios_base::hex;
        }

        // Octal
        b'o' => {
            fpar.fmtstate_.flags_ &= !ios_base::basefield;
            fpar.fmtstate_.flags_ |= ios_base::oct;
        }

        // Floating
        b'A' | b'a' => {
            if buf[*start] == b'A' {
                fpar.fmtstate_.flags_ |= ios_base::uppercase;
            }
            fpar.fmtstate_.flags_ &= !ios_base::basefield;
            fpar.fmtstate_.flags_ |= ios_base::fixed;
            fpar.fmtstate_.flags_ |= ios_base::scientific;
        }
        b'E' | b'e' => {
            if buf[*start] == b'E' {
                fpar.fmtstate_.flags_ |= ios_base::uppercase;
            }
            fpar.fmtstate_.flags_ |= ios_base::scientific;
        }
        b'F' | b'f' => {
            if buf[*start] == b'F' {
                fpar.fmtstate_.flags_ |= ios_base::uppercase;
            }
            fpar.fmtstate_.flags_ |= ios_base::fixed;
        }
        b'G' | b'g' => {
            if buf[*start] == b'G' {
                fpar.fmtstate_.flags_ |= ios_base::uppercase;
            }
            // default flags are correct here
        }

        // Tabulation (a boost::format extension)
        b'T' => {
            *start += 1;
            if *start >= last {
                return Err(bad(*start));
            } else {
                fpar.fmtstate_.fill_ = buf[*start];
            }
            fpar.pad_scheme_ |= TABULATION;
            fpar.arg_n_ = ARG_N_TABULATION;
        }
        b't' => {
            fpar.fmtstate_.fill_ = b' ';
            fpar.pad_scheme_ |= TABULATION;
            fpar.arg_n_ = ARG_N_TABULATION;
        }

        // Character
        b'C' | b'c' => fpar.truncate_ = 1,

        // String
        b'S' | b's' => {
            if precision_set {
                // handle truncation manually, with own parameter.
                fpar.truncate_ = fpar.fmtstate_.precision_;
            }
            fpar.fmtstate_.precision_ = 6; // default stream precision.
        }

        // %n is insecure and ignored by boost::format
        b'n' => fpar.arg_n_ = ARG_N_IGNORED,

        _ => return Err(bad(*start)),
    }
    *start += 1;

    if in_brackets {
        if *start != last && buf[*start] == b'|' {
            *start += 1;
            return Ok(());
        } else {
            return Err(bad(*start));
        }
    }
    Ok(())
}

/// `buf.find(arg_mark, from)`
fn find_mark(buf: &[u8], from: usize) -> Option<usize> {
    if from >= buf.len() {
        return None;
    }
    buf[from..].iter().position(|c| *c == b'%').map(|i| i + from)
}

/// parsing.hpp:419-451 `upper_bound_from_fstring`: the quick parsing of the format string that
/// counts the argument marks. The count sizes an allocation in the C++ and is not needed here;
/// what is kept is the exception for a string that ends in a single `%`.
fn upper_bound_from_fstring(buf: &[u8]) -> Result<i32, FormatError> {
    let mut i1 = 0;
    let mut num_items = 0;
    while let Some(found) = find_mark(buf, i1) {
        i1 = found;
        if i1 + 1 >= buf.len() {
            return Err(FormatError::BadFormatString { pos: i1, size: buf.len() }); // must not end in ".. %"
        }
        if buf[i1 + 1] == buf[i1] {
            // escaped "%%"
            i1 += 2;
            continue;
        }

        i1 += 1;
        // in case of %N% directives, dont count it double (wastes allocations..) :
        while i1 < buf.len() && buf[i1].is_ascii_digit() {
            i1 += 1;
        }
        if i1 < buf.len() && buf[i1] == b'%' {
            i1 += 1;
        }
        num_items += 1;
    }
    Ok(num_items)
}

// ---------------------------------------------------------------------------------------------
// boost/format/feed_args.hpp
// ---------------------------------------------------------------------------------------------

/// feed_args.hpp:29-70 `mk_str`: applies centered/left/right padding to the string `beg`.
fn mk_str(beg: &[u8], w: i64, fill_char: u8, f: u32, prefix_space: u8, center: bool) -> Vec<u8> {
    let size = beg.len();
    let mut res: Vec<u8> = Vec::new();
    if w <= 0 || (w as usize) <= size {
        // no need to pad.
        if prefix_space != 0 {
            res.push(prefix_space);
        }
        res.extend_from_slice(beg);
    } else {
        let n: i64 = w - size as i64 - (prefix_space != 0) as i64;
        let (mut n_after, mut n_before): (i64, i64) = (0, 0);
        if center {
            n_after = n / 2;
            n_before = n - n_after;
        } else if f & ios_base::left != 0 {
            n_after = n;
        } else {
            n_before = n;
        }
        // now make the res string :
        if n_before != 0 {
            res.resize(n_before as usize, fill_char);
        }
        if prefix_space != 0 {
            res.push(prefix_space);
        }
        res.extend_from_slice(beg);
        if n_after != 0 {
            res.resize(res.len() + n_after as usize, fill_char);
        }
    }
    res
}

/// feed_args.hpp:153-278 `put`: does the actual conversion of x, with given params, into a
/// string.
fn put(x: &dyn FormatArg, specs: &FormatItem) -> Result<Vec<u8>, FormatError> {
    let mut oss = OStream::new();
    specs.fmtstate_.apply_on(&mut oss);

    let fl = oss.flags;
    let internal = fl & ios_base::internal != 0;
    let w = oss.width;
    let two_stepped_padding = internal && w != 0;

    if !two_stepped_padding {
        if w > 0 {
            // handle padding via mk_str, not natively in stream
            oss.width = 0;
        }
        let buf = x.put_last(&oss);
        let mut prefix_space: u8 = 0;
        if specs.pad_scheme_ & SPACEPAD != 0 && (buf.is_empty() || (buf[0] != b'+' && buf[0] != b'-')) {
            prefix_space = b' ';
        }
        let res_size = std::cmp::min((specs.truncate_ as usize).wrapping_sub((prefix_space != 0) as usize), buf.len());
        Ok(mk_str(&buf[..res_size], w, oss.fill, fl, prefix_space, specs.pad_scheme_ & CENTERED != 0))
    } else {
        // 2-stepped padding
        // internal can be implied by zeropad, or user-set.
        // left, right, and centered alignment overrule internal,
        // but spacepad or truncate might be mixed with internal (using manipulator)
        let buf = x.put_last(&oss); // may pad
        let res_size = buf.len();
        let mut prefix_space = false;
        if specs.pad_scheme_ & SPACEPAD != 0 && (buf.is_empty() || (buf[0] != b'+' && buf[0] != b'-')) {
            prefix_space = true;
        }
        if res_size == w as usize && w <= specs.truncate_ && !prefix_space {
            // okay, only one thing was printed and padded, so res is fine
            Ok(buf)
        } else {
            //   length w exceeded
            // either it was multi-output with first output padding up all width..
            // either it was one big arg and we are fine.
            // Note that res_size<w is possible  (in case of bad user-defined formatting)
            let res = buf;

            // make a new stream, to start re-formatting from scratch :
            let mut oss2 = OStream::new();
            specs.fmtstate_.apply_on(&mut oss2);

            oss2.width = 0;
            let mut tmp: Vec<u8> = Vec::new();
            if prefix_space {
                tmp.push(b' ');
            }
            tmp.extend_from_slice(&x.put_last(&oss2));
            if tmp.is_empty() && specs.pad_scheme_ & SPACEPAD != 0 {
                prefix_space = true;
                tmp.push(b' ');
            }
            // we now have the minimal-length output
            let tmp_size = std::cmp::min(specs.truncate_ as usize, tmp.len());

            if (w as usize) <= tmp_size {
                // minimal length is already >= w, so no padding (cool!)
                Ok(tmp[..tmp_size].to_vec())
            } else {
                // hum..  we need to pad (multi_output, or spacepad present)
                //find where we should pad
                let sz = std::cmp::min(res_size + prefix_space as usize, tmp_size);
                let mut i = prefix_space as usize;
                while i < sz && tmp[i] == res[i - prefix_space as usize] {
                    i += 1;
                }
                if i >= tmp_size {
                    i = prefix_space as usize;
                }
                let mut out = tmp[..i].to_vec();
                let d = w - tmp_size as i64;
                out.resize(i + d as usize, oss2.fill);
                // res.append(tmp_beg+i, tmp_size-i): the count wraps around for i > tmp_size.
                if i > tmp_size {
                    return Err(FormatError::LengthError);
                }
                out.extend_from_slice(&tmp[i..tmp_size]);
                Ok(out)
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// boost/format/format_class.hpp, format_implementation.hpp
// ---------------------------------------------------------------------------------------------

/// `boost::format`, as far as `Slic3r::format` uses it: made from a format string, fed with
/// `operator%`, read with `str()`.
#[derive(Debug, Clone)]
pub struct BoostFormat {
    /// format_items list itself
    items_: Vec<FormatItem>,
    /// keep track of wich argument is current
    cur_arg_: i32,
    /// number of expected arguments
    num_args_: i32,
    /// true only after call to str() or <<
    dumped_: Cell<bool>,
    /// piece of string to insert before first item
    prefix_: Vec<u8>,
}

impl BoostFormat {
    /// format_implementation.hpp:27-34, 59-65 `basic_format(s)`
    pub fn new(s: &[u8]) -> Result<BoostFormat, FormatError> {
        let mut format = BoostFormat { items_: Vec::new(), cur_arg_: 0, num_args_: 0, dumped_: Cell::new(false), prefix_: Vec::new() };
        format.parse(s)?;
        Ok(format)
    }

    /// parsing.hpp:467-559 `basic_format::parse`
    // `i0 = i1` before a directive is read back only where the directive is printed verbatim,
    // which does not happen with the exceptions enabled; the assignment stays as in the C++.
    #[allow(unused_assignments)]
    fn parse(&mut self, buf: &[u8]) -> Result<(), FormatError> {
        // parse the format-string
        let mut ordered_args = true;
        let mut max_arg_n: i32 = -1;

        // A: find upper_bound on num_items and allocates arrays
        upper_bound_from_fstring(buf)?;
        self.items_.clear();
        self.prefix_.clear();

        // B: Now the real parsing of the format string :
        let mut num_items: usize = 0;
        let (mut i0, mut i1): (usize, usize) = (0, 0);
        let mut cur_item: usize = 0;
        while let Some(found) = find_mark(buf, i1) {
            i1 = found;
            if i1 + 1 < buf.len() && buf[i1 + 1] == buf[i1] {
                // escaped mark, '%%'
                let piece = if cur_item == 0 { &mut self.prefix_ } else { &mut self.items_[cur_item - 1].appendix_ };
                piece.extend_from_slice(&buf[i0..i1 + 1]);
                i1 += 2;
                i0 = i1;
                continue;
            }

            if i1 != i0 {
                let piece = if cur_item == 0 { &mut self.prefix_ } else { &mut self.items_[cur_item - 1].appendix_ };
                piece.extend_from_slice(&buf[i0..i1]);
                i0 = i1;
            }
            i1 += 1;
            let mut it = i1;
            // items_[cur_item]: the C++ allocated the items before; an item an ignored directive
            // left behind is parsed into again as it is.
            if cur_item == self.items_.len() {
                self.items_.push(FormatItem::new(b' '));
            }
            parse_printf_directive(buf, &mut it, &mut self.items_[cur_item], i1)?;
            i1 = it;
            i0 = i1;
            self.items_[cur_item].compute_states(); // process complex options, like zeropad, into params

            let arg_n = self.items_[cur_item].arg_n_;
            if arg_n == ARG_N_IGNORED {
                continue;
            }
            if arg_n == ARG_N_NO_POSIT {
                ordered_args = false;
            } else if arg_n == ARG_N_TABULATION {
                // special_things
            } else if arg_n > max_arg_n {
                max_arg_n = arg_n;
            }
            num_items += 1;
            cur_item += 1;
        } // loop on %'s

        // store the final piece of string
        {
            let piece = if cur_item == 0 { &mut self.prefix_ } else { &mut self.items_[cur_item - 1].appendix_ };
            piece.extend_from_slice(&buf[i0..]);
        }

        if !ordered_args {
            if max_arg_n >= 0 {
                // dont mix positional with non-positionnal directives
                return Err(FormatError::BadFormatString { pos: max_arg_n as usize, size: 0 });
            }
            // set things like it would have been with positional directives :
            let mut non_ordered_items: i32 = 0;
            for i in 0..num_items {
                if self.items_[i].arg_n_ == ARG_N_NO_POSIT {
                    self.items_[i].arg_n_ = non_ordered_items;
                    non_ordered_items += 1;
                }
            }
            max_arg_n = non_ordered_items - 1;
        }

        // C: set some member data :
        self.items_.truncate(num_items);

        self.num_args_ = max_arg_n + 1;
        Ok(())
    }

    /// format_implementation.hpp:129-149 `clear`: empty the string buffers and make the format
    /// object ready for formatting a new set of arguments. (There are no bound arguments here.)
    pub fn clear(&mut self) -> &mut BoostFormat {
        for item in &mut self.items_ {
            item.res_.clear();
        }
        self.cur_arg_ = 0;
        self.dumped_.set(false);
        self
    }

    /// `operator%`: feed_args.hpp:297-314 `feed` / `feed_impl`, with `distribute`
    /// (feed_args.hpp:281-295) in it.
    pub fn feed(&mut self, x: &dyn FormatArg) -> Result<&mut BoostFormat, FormatError> {
        if self.dumped_.get() {
            self.clear();
        }
        // distribute: call put(x, ..) on every occurrence of the current argument :
        if self.cur_arg_ >= self.num_args_ {
            return Err(FormatError::TooManyArgs { cur: self.cur_arg_ as usize, expected: self.num_args_ as usize });
        }
        for i in 0..self.items_.len() {
            if self.items_[i].arg_n_ == self.cur_arg_ {
                self.items_[i].res_ = put(x, &self.items_[i])?;
            }
        }
        self.cur_arg_ += 1;
        Ok(self)
    }

    /// format_implementation.hpp:215-243 `str`
    pub fn str(&self) -> Result<Vec<u8>, FormatError> {
        if self.items_.is_empty() {
            return Ok(self.prefix_.clone());
        }
        if self.cur_arg_ < self.num_args_ {
            // not enough variables supplied
            return Err(FormatError::TooFewArgs { cur: self.cur_arg_ as usize, expected: self.num_args_ as usize });
        }

        let mut res: Vec<u8> = Vec::new();
        res.extend_from_slice(&self.prefix_);
        for item in &self.items_ {
            res.extend_from_slice(&item.res_);
            if item.arg_n_ == ARG_N_TABULATION && (item.fmtstate_.width_ as usize) > res.len() {
                res.resize(item.fmtstate_.width_ as usize, item.fmtstate_.fill_);
            }
            res.extend_from_slice(&item.appendix_);
        }
        self.dumped_.set(true);
        Ok(res)
    }
}

#[cfg(test)]
mod tests {
    use super::internal::format::cook;
    use super::*;

    #[test]
    fn cook_is_identity() {
        assert_eq!(cook(42), 42);
        assert_eq!(cook("abc"), "abc");
    }

    /// The format strings libslic3r passes (Config.cpp 627, Print.cpp 1821, GCode.cpp 1749).
    #[test]
    fn the_forms_libslic3r_uses() {
        assert_eq!(
            format("Invalid value provided for parameter %1%: %2%", &[&"layer_height", &"x"]).unwrap(),
            "Invalid value provided for parameter layer_height: x"
        );
        assert_eq!(slic3r_format!("Plate %d: %s does not support filament %s", 1, "Cool Plate", 3).unwrap(), "Plate 1: Cool Plate does not support filament 3");
        assert_eq!(slic3r_format!("no args").unwrap(), "no args");
        assert_eq!(slic3r_format!("100%% of %1%", "it").unwrap(), "100% of it");
    }

    #[test]
    fn errors_are_the_exceptions_of_boost() {
        assert_eq!(format("%1% %2%", &[&1]).unwrap_err().what(), "boost::too_few_args: format-string referred to more arguments than were passed");
        assert_eq!(format("%1%", &[&1, &2]).unwrap_err().what(), "boost::too_many_args: format-string referred to fewer arguments than were passed");
        assert_eq!(format("50%", &[]).unwrap_err().what(), "boost::bad_format_string: format-string is ill-formed");
        assert_eq!(format("%1% %d", &[&1, &2]).unwrap_err(), FormatError::BadFormatString { pos: 0, size: 0 });
    }
}
