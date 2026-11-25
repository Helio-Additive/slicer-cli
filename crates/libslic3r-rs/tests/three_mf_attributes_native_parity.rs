//! The XML attribute helpers of 3mf.cpp (get_attribute_value_charptr / _string / _float / _int /
//! _bool, 3mf.cpp:165-204) and their bbs_3mf.cpp twins (490-536) against the pin's (`tests/data/three_mf_attributes/gen.cpp`): fast_float's
//! from_chars for the float, qi::int_ for the int, atoi for the bool.

use slicer::format::bbs_3mf::{
    bbs_get_attribute_value_bool, bbs_get_attribute_value_charptr, bbs_get_attribute_value_float, bbs_get_attribute_value_int,
    bbs_get_attribute_value_string,
};
use slicer::format::three_mf::{
    get_attribute_value_bool, get_attribute_value_charptr, get_attribute_value_float, get_attribute_value_int,
    get_attribute_value_string,
};

fn unhex(s: &str) -> String {
    if s == "-" {
        return String::new();
    }
    let bytes: Vec<u8> = (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect();
    String::from_utf8(bytes).unwrap()
}

#[test]
fn attribute_helpers_match_the_pin() {
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/three_mf_attributes/vectors.txt")).unwrap();
    let mut cases = 0;
    for line in text.lines() {
        let f: Vec<&str> = line.split(' ').collect();
        let attrs: Vec<(String, String)> = if f[0] == "missing" {
            vec![("other".into(), "x".into())]
        } else {
            vec![("other".into(), "x".into()), ("key".into(), unhex(f[0]))]
        };
        let v = if f[0] == "missing" { String::new() } else { unhex(f[0]) };
        assert_eq!(get_attribute_value_charptr(&attrs, "key").is_some(), f[1] == "1", "found {v:?}");
        assert_eq!(get_attribute_value_string(&attrs, "key"), unhex(f[2]), "string {v:?}");
        assert_eq!(format!("{:08x}", get_attribute_value_float(&attrs, "key").to_bits()), f[3], "float {v:?}");
        assert_eq!(get_attribute_value_int(&attrs, "key").to_string(), f[4], "int {v:?}");
        assert_eq!((get_attribute_value_bool(&attrs, "key") as i32).to_string(), f[5], "bool {v:?}");
        // bbs_3mf.cpp:490-536
        assert_eq!(bbs_get_attribute_value_charptr(&attrs, "key").is_some(), f[6] == "1", "bbs found {v:?}");
        assert_eq!(bbs_get_attribute_value_string(&attrs, "key"), unhex(f[7]), "bbs string {v:?}");
        assert_eq!(format!("{:08x}", bbs_get_attribute_value_float(&attrs, "key").to_bits()), f[8], "bbs float {v:?}");
        assert_eq!(bbs_get_attribute_value_int(&attrs, "key").to_string(), f[9], "bbs int {v:?}");
        assert_eq!((bbs_get_attribute_value_bool(&attrs, "key") as i32).to_string(), f[10], "bbs bool {v:?}");
        cases += 1;
    }
    assert_eq!(cases, 47);
}
