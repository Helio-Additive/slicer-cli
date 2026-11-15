//! `desensitize` against Desensitize.cpp at the pin.
//!
//! `tests/data/desensitize/vectors.txt` is the output of `gen.cpp` beside it, which compiles the C++
//! file itself with nlohmann::json and the masks on, as in the build of `slicer_cli` (the header
//! comment there says how, and describes the lines). Compared byte for byte: the five masks on
//! strings with multi-byte characters, `mask_by_key` for every registered field and for names that
//! only resemble one, `json_log` on generated documents (nested objects and arrays, json texts
//! inside strings, every kind of number, compact and indented), and the float and integer printing
//! of nlohmann's dump on doubles given by their bits.

use slicer::desensitize::*;

const VECTORS: &str = include_str!("data/desensitize/vectors.txt");

fn unhex(hex: &str) -> Vec<u8> {
    if hex == "-" {
        return Vec::new();
    }
    (0..hex.len() / 2).map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).expect("hex")).collect()
}

fn text(hex: &str) -> String {
    String::from_utf8(unhex(hex)).expect("utf-8")
}

/// The fields of every line tagged `tag`, without the tag and the arrow.
fn lines(tag: &str) -> Vec<Vec<&'static str>> {
    VECTORS
        .lines()
        .filter_map(|line| {
            let mut fields = line.split_whitespace();
            (fields.next() == Some(tag)).then(|| fields.filter(|f| *f != "->").collect())
        })
        .collect()
}

#[test]
fn masks() {
    let cases = lines("MASK");
    assert_eq!(cases.len(), 300);
    let mut cut = 0;
    for f in &cases {
        let input = unhex(f[0]);
        let want: Vec<Vec<u8>> = f[1..6].iter().map(|h| unhex(h)).collect();
        let got = [
            dev_id_bytes(&input),
            dev_ip_bytes(&input),
            dev_name_bytes(&input),
            user_id_bytes(&input),
            mask_all_bytes(&input),
        ];
        for (k, name) in ["dev_id", "dev_ip", "dev_name", "user_id", "mask_all"].iter().enumerate() {
            assert_eq!(got[k], want[k], "{name}({:?})", String::from_utf8_lossy(&input));
        }
        // The `str` forms: the same bytes, a cut UTF-8 sequence replaced.
        let s = std::str::from_utf8(&input).expect("utf-8");
        let strs = [dev_id(s), dev_ip(s), dev_name(s), user_id(s), mask_all(s)];
        for k in 0..5 {
            assert_eq!(strs[k], String::from_utf8_lossy(&want[k]));
            cut += usize::from(std::str::from_utf8(&want[k]).is_err());
        }
    }
    // Some masks do cut a character in two.
    assert!(cut > 20, "{cut}");
}

#[test]
fn by_key() {
    let cases = lines("KEY");
    assert_eq!(cases.len(), 138);
    let mut registered = 0;
    for f in &cases {
        let (key, value) = (text(f[0]), unhex(f[1]));
        let want = (f[2] == "1").then(|| unhex(f[3]));
        assert_eq!(mask_by_key_bytes(&key, &value), want, "mask_by_key({key:?})");
        let s = std::str::from_utf8(&value).expect("utf-8");
        assert_eq!(mask_by_key(&key, s), want.as_ref().map(|w| String::from_utf8_lossy(w).into_owned()));
        registered += usize::from(want.is_some());
    }
    assert_eq!(registered, 13 * 3);
}

#[test]
fn json_documents() {
    let cases = lines("JSON");
    assert_eq!(cases.len(), 190);
    let (mut failed, mut indented) = (0, 0);
    for (case, f) in cases.iter().enumerate() {
        let indent: i32 = f[0].parse().unwrap();
        let doc = text(f[1]);
        let want = text(f[2]);
        let json = nlohmann_parse(&doc).unwrap_or_else(|| panic!("case {case}: {doc} does not parse"));
        assert_eq!(json_log(&json, indent), want, "case {case}: indent {indent}, document {doc}");
        failed += usize::from(want == "<json not logged: masking failed>");
        indented += usize::from(indent >= 0);
    }
    assert!(failed > 10 && failed < 80 && indented > 30, "{failed} placeholders, {indented} indented");
}

#[test]
fn float_and_integer_printing() {
    let cases = lines("FLT");
    assert!(cases.len() > 1300, "{}", cases.len());
    for f in &cases {
        let value = f64::from_bits(u64::from_str_radix(f[0], 16).unwrap());
        let json = serde_json::json!({ "x": value });
        assert_eq!(json_log(&json, -1), text(f[1]), "{value:e} ({})", f[0]);
    }
    let cases = lines("INT");
    assert_eq!(cases.len(), 7);
    for f in &cases {
        let number: serde_json::Value = serde_json::from_str(&text(f[0])).unwrap();
        let json = serde_json::json!({ "x": number });
        assert_eq!(json_log(&json, -1), text(f[1]));
    }
}

#[test]
fn json_log_examples() {
    // Read off Desensitize.cpp:112-236.
    let json = serde_json::json!({
        "dev_id": "01S00A123456789",
        "print": { "dev_ip": "192.168.1.20", "ipcam": "192.168.1.20", "uid": 123456, "ip": 3232235796u32 },
        "name": "plain",
        "list": [ { "token": "abc" } ]
    });
    assert_eq!(
        json_log(&json, -1),
        r#"{"dev_id":"01S*********789","list":[{"token":"******"}],"name":"plain","print":{"dev_ip":"***.***.1.20","ip":"******","ipcam":"192.168.1.20","uid":"****56"}}"#
    );
    // An array inside an array is reached through the traversal stack, and `iter.key()` throws for it.
    assert_eq!(json_log(&serde_json::json!({ "a": [[1]] }), -1), "<json not logged: masking failed>");
    assert_eq!(json_log(&serde_json::json!({ "a": [[]] }), -1), r#"{"a":[[]]}"#);
    assert_eq!(json_log(&serde_json::Value::Null, 1), "null");
    assert_eq!(json_log(&serde_json::json!({ "a": { "b": [] } }), 2), "{\n  \"a\": {\n    \"b\": []\n  }\n}");
}
