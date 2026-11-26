//! NanoSVG polygon and polyline point data. The caller supplies the element transform.
use super::path::{add_path, line_to, Path, Point};
use super::whitespace;

// nanosvg.h 1157–1198. The native buffer holds 63 item bytes and a terminator.
// Its input pointer advances across the entire item even when the buffer fills.
fn parse_number(input: &[u8]) -> usize {
    let mut i = usize::from(matches!(input.first(), Some(b'+' | b'-')));
    while input.get(i).is_some_and(u8::is_ascii_digit) {
        i += 1;
    }
    if input.get(i) == Some(&b'.') {
        i += 1;
        while input.get(i).is_some_and(u8::is_ascii_digit) {
            i += 1;
        }
    }
    if matches!(input.get(i), Some(b'e' | b'E')) && !matches!(input.get(i + 1), Some(b'm' | b'x')) {
        i += 1;
        if matches!(input.get(i), Some(b'+' | b'-')) {
            i += 1;
        }
        while input.get(i).is_some_and(u8::is_ascii_digit) {
            i += 1;
        }
    }
    i
}

// nsvg__getNextPathItem, nanosvg.h 1213–1230.
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

// nanosvg.h 1090–1154. strtoll saturates each digit sequence separately.
fn digits(input: &mut &[u8]) -> (i64, usize) {
    let mut result = 0i64;
    let mut count = 0;
    while let Some((&b, tail)) = input.split_first() {
        if !b.is_ascii_digit() {
            break;
        }
        result = result
            .saturating_mul(10)
            .saturating_add(i64::from(b - b'0'));
        count += 1;
        *input = tail;
    }
    (result, count)
}

fn sign(input: &mut &[u8]) -> f64 {
    match input.first() {
        Some(b'-') => {
            *input = &input[1..];
            -1.0
        }
        Some(b'+') => {
            *input = &input[1..];
            1.0
        }
        _ => 1.0,
    }
}

fn atof(mut input: &[u8]) -> f64 {
    let multiplier = sign(&mut input);
    let (int_part, n_int) = digits(&mut input);
    let mut value = int_part as f64;
    let mut n_frac = 0;
    if input.first() == Some(&b'.') {
        input = &input[1..];
        let (fraction, count) = digits(&mut input);
        n_frac = count;
        if count > 0 {
            value += fraction as f64 / 10.0f64.powf(count as f64);
        }
    }
    if n_int == 0 && n_frac == 0 {
        return 0.0;
    }
    if matches!(input.first(), Some(b'E' | b'e')) {
        input = &input[1..];
        let exponent_sign = sign(&mut input);
        let (exponent, count) = digits(&mut input);
        if count > 0 {
            value *= 10.0f64.powf(exponent_sign * exponent as f64);
        }
    }
    value * multiplier
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
