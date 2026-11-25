//! string_printf's buffer handling (utils.cpp:1126-1146) against the pin's `string_printf("%s", s)`
//! (`tests/data/utils_string_printf/gen.cpp`): a 199-byte result keeps 198 bytes and a NUL.

use slicer::utils::{string_printf, string_printf_bytes};

fn unhex(s: &str) -> Vec<u8> {
    if s == "-" {
        return Vec::new();
    }
    (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
}

#[test]
fn string_printf_matches_the_pin() {
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/utils_string_printf/vectors.txt")).unwrap();
    let mut cases = 0;
    for line in text.lines() {
        let (input, output) = line.split_once(' ').unwrap();
        let (input, output) = (unhex(input), unhex(output));
        assert_eq!(string_printf_bytes(&input), output, "length {}", input.len());
        if let (Ok(s), Ok(o)) = (std::str::from_utf8(&input), std::str::from_utf8(&output)) {
            assert_eq!(string_printf(s), o, "length {}", input.len());
        }
        cases += 1;
    }
    assert_eq!(cases, 269);
}
