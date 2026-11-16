//! `format::format` (the port of libslic3r/format.hpp) against the C++ at the pin.
//!
//! `tests/data/format/vectors.txt` is the output of `gen.cpp` beside it, which includes format.hpp
//! itself and calls `Slic3r::format()` with Boost.Format and libc++ (the header comment there says
//! how, and describes the lines). Every call is repeated here with the same format string and the
//! same arguments, of the Rust types that stand for the C++ ones, and the text returned, or the
//! `what()` of the exception thrown, has to be the same, byte for byte.

use slicer::format::format::{format, BoostFormat, FormatArg, FormatError};

const VECTORS: &[u8] = include_bytes!("data/format/vectors.txt");

fn unesc(s: &str) -> String {
    if s == "\\e" {
        return String::new();
    }
    let mut out = String::new();
    let mut chars = s.chars();
    while let Some(c) = chars.next() {
        if c != '\\' {
            out.push(c);
            continue;
        }
        match chars.next().expect("escape") {
            '\\' => out.push('\\'),
            'n' => out.push('\n'),
            'r' => out.push('\r'),
            's' => out.push(' '),
            'x' => {
                let hex: String = [chars.next().expect("hex"), chars.next().expect("hex")].iter().collect();
                out.push(u8::from_str_radix(&hex, 16).expect("hex") as char);
            }
            other => panic!("escape \\{other}"),
        }
    }
    out
}

/// An argument of gen.cpp as the Rust type that stands for its C++ type.
fn argument(token: &str) -> Box<dyn FormatArg> {
    let (kind, value) = token.split_once(':').expect("an argument");
    match kind {
        "h" => Box::new(value.parse::<i16>().expect("short")),
        "i" => Box::new(value.parse::<i32>().expect("int")),
        "l" => Box::new(value.parse::<i64>().expect("long long")),
        "H" => Box::new(value.parse::<u16>().expect("unsigned short")),
        "u" => Box::new(value.parse::<u32>().expect("unsigned")),
        "U" => Box::new(value.parse::<u64>().expect("unsigned long long")),
        "f" => Box::new(f32::from_bits(u32::from_str_radix(value, 16).expect("float bits"))),
        "d" => Box::new(f64::from_bits(u64::from_str_radix(value, 16).expect("double bits"))),
        "b" => Box::new(value == "1"),
        "c" => Box::new(value.parse::<u8>().expect("char") as char),
        "s" => Box::new(unesc(value)),
        other => panic!("argument kind {other}"),
    }
}

#[test]
fn format_matches_the_cpp() {
    // The file is bytes: a `%.2s` of the C++ cuts a text in the middle of a UTF-8 sequence.
    let mut lines = VECTORS.split(|b| *b == b'\n').filter(|line| !line.is_empty());
    let (mut ok, mut thrown, mut not_utf8) = (0, 0, 0);
    while let Some(case) = lines.next() {
        let case = std::str::from_utf8(case).expect("a case line is UTF-8");
        let mut tokens = case.split(' ');
        assert_eq!(tokens.next(), Some("F"));
        let fmt = unesc(tokens.next().expect("format string"));
        let n: usize = tokens.next().expect("count").parse().expect("count");
        let args: Vec<Box<dyn FormatArg>> = tokens.map(argument).collect();
        assert_eq!(args.len(), n);
        let refs: Vec<&dyn FormatArg> = args.iter().map(|a| a.as_ref()).collect();

        // boost::format itself: (boost::format(fmt) % a % b).str(), the bytes.
        let fed = || -> Result<Vec<u8>, FormatError> {
            let mut message = BoostFormat::new(fmt.as_bytes())?;
            for arg in &refs {
                message.feed(*arg)?;
            }
            message.str()
        };
        let bytes = fed();
        let got: Vec<u8> = match &bytes {
            Ok(text) => {
                ok += 1;
                [b"ok ".as_slice(), esc_bytes(text).as_slice()].concat()
            }
            Err(e) => {
                thrown += 1;
                format!("throw {}", esc(e.what())).into_bytes()
            }
        };
        let want = lines.next().expect("result");
        assert_eq!(String::from_utf8_lossy(&got), String::from_utf8_lossy(want), "{case}");
        assert_eq!(got, want, "{case}");

        // Slic3r::format: the same, as a String.
        match (format(&fmt, &refs), bytes) {
            (Ok(text), Ok(bytes)) => {
                not_utf8 += std::str::from_utf8(&bytes).is_err() as usize;
                assert_eq!(text, String::from_utf8_lossy(&bytes), "{case}");
            }
            (Err(e), Err(want)) => assert_eq!(e, want, "{case}"),
            (got, want) => panic!("{case}: format() {got:?}, boost::format {want:?}"),
        }
    }
    assert_eq!((ok, thrown), (3609, 552));
    assert!(not_utf8 > 0);
}

/// `esc` of gen.cpp, on bytes.
fn esc_bytes(s: &[u8]) -> Vec<u8> {
    if s.is_empty() {
        return b"\\e".to_vec();
    }
    let mut out = Vec::new();
    for c in s {
        match *c {
            b'\\' => out.extend_from_slice(b"\\\\"),
            b'\n' => out.extend_from_slice(b"\\n"),
            b'\r' => out.extend_from_slice(b"\\r"),
            b' ' => out.extend_from_slice(b"\\s"),
            c if c < 0x20 || c == 0x7f => out.extend_from_slice(format!("\\x{:02X}", c).as_bytes()),
            c => out.push(c),
        }
    }
    out
}

fn esc(s: &str) -> String {
    if s.is_empty() {
        return "\\e".to_string();
    }
    let mut out = String::new();
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            ' ' => out.push_str("\\s"),
            c if (c as u32) < 0x20 || c as u32 == 0x7f => out.push_str(&format!("\\x{:02X}", c as u32)),
            c => out.push(c),
        }
    }
    out
}
