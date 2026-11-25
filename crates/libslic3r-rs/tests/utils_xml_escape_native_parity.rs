//! xml_escape (both is_marked values), xml_escape_double_quotes_attribute_value and xml_unescape
//! (utils.cpp:1207-1300) against the pin's (`tests/data/utils_xml_escape/gen.cpp`).

use slicer::utils::{xml_escape, xml_escape_double_quotes_attribute_value, xml_unescape};

fn unhex(s: &str) -> String {
    if s == "-" {
        return String::new();
    }
    let bytes: Vec<u8> = (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect();
    String::from_utf8(bytes).unwrap()
}

#[test]
fn xml_escapes_match_the_pin() {
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/utils_xml_escape/vectors.txt")).unwrap();
    let mut cases = 0;
    for line in text.lines() {
        let f: Vec<String> = line.split(' ').map(unhex).collect();
        let s = &f[0];
        assert_eq!(xml_escape(s, false), f[1], "xml_escape({s:?}, false)");
        assert_eq!(xml_escape(s, true), f[2], "xml_escape({s:?}, true)");
        assert_eq!(xml_escape_double_quotes_attribute_value(s), f[3], "xml_escape_double_quotes_attribute_value({s:?})");
        assert_eq!(xml_unescape(s), f[4], "xml_unescape({s:?})");
        cases += 1;
    }
    assert_eq!(cases, 400);
}
