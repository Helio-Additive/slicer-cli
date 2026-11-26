//! Callback order, tag spelling and attribute bytes from the pinned nsvg__parseXML.
use nanosvg::{parse_xml, XmlEvent};
use std::fmt::Write;

fn field(row: &mut String, bytes: &[u8]) {
    row.push(' ');
    if bytes.is_empty() {
        row.push('-');
    }
    for b in bytes {
        write!(row, "{b:02x}").unwrap();
    }
}

fn callbacks(input: &[u8]) -> Vec<String> {
    let mut rows = Vec::new();
    parse_xml(input, |event| {
        let mut row = String::new();
        match event {
            XmlEvent::Start { name, attributes } => {
                row.push('S');
                field(&mut row, name);
                for (key, value) in attributes {
                    field(&mut row, key);
                    field(&mut row, value);
                }
            }
            XmlEvent::End(name) => {
                row.push('E');
                field(&mut row, name);
            }
            XmlEvent::Content(text) => {
                row.push('C');
                field(&mut row, text);
            }
        }
        rows.push(row);
    });
    rows
}

#[test]
fn matches_pinned_xml_callbacks() {
    let mut lines = include_str!("data/xml.txt").lines().peekable();
    while let Some(input) = lines.next() {
        let hex = input.strip_prefix("I ").unwrap();
        let bytes: Vec<u8> = (0..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
            .collect();
        let mut expected = Vec::new();
        while lines.peek().is_some_and(|line| !line.starts_with("I ")) {
            expected.push(lines.next().unwrap());
        }
        assert_eq!(callbacks(&bytes), expected, "{input}");
    }
}
