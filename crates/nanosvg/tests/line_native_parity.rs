//! Complete SVG line paths from the pinned nsvgParse.
use nanosvg::{
    line::{parse_line, Viewport},
    parse_xml, XmlEvent,
};
use std::fmt::Write;

#[test]
fn line_coordinates_match_native_documents() {
    let mut rows = include_str!("data/lines.txt").lines();
    let mut cases = 0;
    while let Some(row) = rows.next() {
        let xml = row.strip_prefix("svg ").unwrap();
        let mut output = Vec::new();
        parse_xml(xml.as_bytes(), |event| {
            let XmlEvent::Start { name, attributes } = event else {
                return;
            };
            if name != b"line" {
                return;
            }
            // The caller supplies the resolved SVG transform. This oracle uses
            // one explicit matrix and the identity for all other elements.
            let transform = if attributes.iter().any(|(key, _)| *key == b"transform") {
                [0., 1., -1., 0., 3., -8.]
            } else {
                [1., 0., 0., 1., 0., 0.]
            };
            let Some(path) = parse_line(
                &attributes,
                Viewport {
                    origin: [0., 0.],
                    size: [128., 80.],
                    dpi: 96.,
                },
                13.,
                transform,
            ) else {
                return;
            };
            let mut line = format!("path {} {}", u8::from(path.closed), path.points.len());
            for coordinate in path
                .bounds
                .into_iter()
                .chain(path.points.into_iter().flatten())
            {
                write!(line, " {:08x}", coordinate.to_bits()).unwrap();
            }
            output.push(line);
        });
        let expected: Vec<_> = rows.by_ref().take_while(|r| *r != "end").collect();
        assert_eq!(output, expected, "{xml}");
        cases += 1;
    }
    assert_eq!(cases, 32);
}
