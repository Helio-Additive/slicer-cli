//! NanoSVG polygon and polyline point data. The caller supplies the element transform.
use super::number::{atof, parse_number};
use super::path::{add_path, line_to, Path, Point};
use super::whitespace;

fn next_path_item<'a>(input: &mut &'a [u8]) -> &'a [u8] {
    let offset = input
        .iter()
        .position(|&b| !whitespace(b) && b != b',')
        .unwrap_or(input.len());
    let rest = &input[offset..];
    let Some(&first) = rest.first() else {
        *input = rest;
        return rest;
    };
    let n = if first.is_ascii_digit() || matches!(first, b'+' | b'-' | b'.') {
        parse_number(rest)
    } else {
        1
    };
    *input = &rest[n..];
    &rest[..n.min(63)]
}

/// Parse the points of an SVG polygon or polyline using the caller's resolved
/// element transform. nsvg__parsePoly (nanosvg.h 2556–2590) restarts pairs
/// at each points attribute and appends them to the same path.
pub fn parse_poly(
    attributes: &[(&[u8], &[u8])],
    closed: bool,
    transform: [f32; 6],
) -> Option<Path> {
    let mut points: Vec<Point> = Vec::new();
    for &(name, value) in attributes {
        if name != b"points" {
            continue;
        }
        let mut coordinates = &value[..value.iter().position(|&b| b == 0).unwrap_or(value.len())];
        while !coordinates.is_empty() {
            let x = atof(next_path_item(&mut coordinates)) as f32;
            if coordinates.is_empty() {
                break;
            }
            let y = atof(next_path_item(&mut coordinates)) as f32;
            let point = [x, y];
            if points.is_empty() {
                points.push(point);
            } else {
                line_to(&mut points, point);
            }
        }
    }
    add_path(points, closed, transform)
}
