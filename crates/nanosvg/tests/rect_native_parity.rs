//! Sharp and rounded rectangle paths in complete SVG documents from the pinned nsvgParse.
use nanosvg::{line::Viewport, parse_xml, rect::parse_rect, XmlEvent};
use std::fmt::Write;

#[test]
fn rectangle_coordinates_match_native_documents() {
    #[cfg(any(target_arch = "aarch64", target_feature = "fma"))]
    let mut rows = include_str!("data/rectangles-fused.txt").lines();
    #[cfg(not(any(target_arch = "aarch64", target_feature = "fma")))]
    let mut rows = include_str!("data/rectangles-unfused.txt").lines();
    let mut cases = 0;
    while let Some(row) = rows.next() {
        let xml = row.strip_prefix("svg ").unwrap();
        let mut output = Vec::new();
        parse_xml(xml.as_bytes(), |event| {
            let XmlEvent::Start { name, attributes } = event else {
                return;
            };
            if name != b"rect" {
                return;
            }
            // The caller supplies the resolved SVG transform. This oracle uses
            // one explicit matrix and the identity for all other elements.
            let transform = if attributes.iter().any(|(key, _)| *key == b"transform") {
                [0., 1., -1., 0., 3., -8.]
            } else {
                [1., 0., 0., 1., 0., 0.]
            };
            let Some(path) = parse_rect(
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
    assert_eq!(cases, 43);
}
