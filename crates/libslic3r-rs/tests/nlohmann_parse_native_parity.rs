//! R954: `crate::desensitize::nlohmann_parse` against the pin's `nlohmann::json::parse(std::string)`
//! (the nlohmann 3.10.4 the pin bundles, src/nlohmann; parser.hpp:85-99 strict, lexer.hpp
//! scan_number's `strtod`), the reader of Desensitize.cpp:219 and of the assembly JSON readers
//! (format/assembly_steps_json.rs, assembly_tree_json.rs). Vectors: tests/data/nlohmann_parse
//! (gen.cpp: 300 documents nlohmann itself wrote from LCG doubles, 44 edge cases). Every number must
//! come out with the pin's bits (NLOHMANN_PARSE_EXACT; with it at 0 serde_json reads 17-digit doubles
//! one ulp off).

use serde_json::Value;

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn unhex(s: &str) -> Vec<u8> {
    (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
}

/// The generator's walk: objects in std::map order (byte order of the keys).
fn walk(v: &Value, out: &mut String) {
    match v {
        Value::Null => out.push_str("Z\n"),
        Value::Bool(b) => out.push_str(&format!("B {}\n", *b as i32)),
        Value::Number(n) => {
            if let Some(u) = n.as_u64() {
                out.push_str(&format!("N {u}\n"));
            } else if let Some(i) = n.as_i64() {
                out.push_str(&format!("N {i}\n"));
            } else {
                out.push_str(&format!("F {:016x}\n", n.as_f64().unwrap().to_bits()));
            }
        }
        Value::String(s) => out.push_str(&format!("S {}\n", hex(s.as_bytes()))),
        Value::Array(a) => {
            out.push_str(&format!("A {}\n", a.len()));
            for x in a {
                walk(x, out);
            }
        }
        Value::Object(m) => {
            out.push_str(&format!("O {}\n", m.len()));
            let mut keys: Vec<&String> = m.keys().collect();
            keys.sort_by(|a, b| a.as_bytes().cmp(b.as_bytes()));
            for k in keys {
                out.push_str(&format!("K {}\n", hex(k.as_bytes())));
                walk(&m[k], out);
            }
        }
    }
}

#[test]
fn nlohmann_parse_reads_as_the_pin() {
    let text = include_str!("data/nlohmann_parse/vectors.txt");
    let mut cases = 0;
    let mut bad = Vec::new();
    let mut lines = text.lines().peekable();
    while let Some(line) = lines.next() {
        let doc = line.strip_prefix("DOC ").expect("DOC line");
        let bytes = unhex(doc);
        let mut want = String::new();
        for l in lines.by_ref() {
            if l == "END" {
                break;
            }
            want.push_str(l);
            want.push('\n');
        }
        let doc_text = String::from_utf8(bytes).expect("utf-8 document");
        let mut got = String::new();
        match slicer::desensitize::nlohmann_parse(&doc_text) {
            None => got.push_str("ERR\n"),
            Some(v) => walk(&v, &mut got),
        }
        if got != want {
            bad.push(format!("{doc_text:?}\n want {want:?}\n got  {got:?}"));
        }
        cases += 1;
    }
    assert_eq!(cases, 344);
    assert!(bad.is_empty(), "{} of {cases} documents differ:\n{}", bad.len(), bad[..bad.len().min(8)].join("\n"));
}
