//! Rectangle geometry before NanoSVG's image-wide viewBox transform.
use crate::coordinate::Coordinate;
use crate::line::Viewport;
use crate::path::{add_path, cubic_handle, line_to, Path, KAPPA90};

/// nsvg__parseRect (nanosvg.h 2411–2464). An omitted corner radius takes
/// the other radius before each is clamped to half of its own dimension.
pub fn parse_rect(
    attributes: &[(&[u8], &[u8])],
    viewport: Viewport,
    mut font_size: f32,
    transform: [f32; 6],
) -> Option<Path> {
    let (mut x, mut y, mut width, mut height) = (0., 0., 0., 0.);
    let (mut rx, mut ry) = (-1., -1.);
    for &(name, value) in attributes {
        let length = |axis: usize| {
            Coordinate::parse(value).to_pixels(0., viewport.size[axis], viewport.dpi, font_size)
        };
        match name {
            b"x" => x = viewport.coordinate(value, 0, font_size),
            b"y" => y = viewport.coordinate(value, 1, font_size),
            b"width" => width = length(0),
            b"height" => height = length(1),
            b"rx" => rx = length(0).abs(),
            b"ry" => ry = length(1).abs(),
            b"font-size" => font_size = viewport.diagonal_coordinate(value, font_size),
            b"style" => font_size = viewport.style_font_size(value, font_size),
            _ => {}
        }
    }
    if width == 0. || height == 0. {
        return None;
    }
    if rx < 0. && ry > 0. {
        rx = ry;
    }
    if ry < 0. && rx > 0. {
        ry = rx;
    }
    if rx < 0. {
        rx = 0.;
    }
    if ry < 0. {
        ry = 0.;
    }
    if rx > width / 2. {
        rx = width / 2.;
    }
    if ry > height / 2. {
        ry = height / 2.;
    }

    let right = x + width;
    let bottom = y + height;
    let mut points = Vec::new();
    if rx < 0.00001 || ry < 0.0001 {
        points.push([x, y]);
        line_to(&mut points, [right, y]);
        line_to(&mut points, [right, bottom]);
        line_to(&mut points, [x, bottom]);
    } else {
        let handle = |corner, radius| cubic_handle(corner, radius, 1.0 - KAPPA90);
        points.push([x + rx, y]);
        line_to(&mut points, [right - rx, y]);
        points.extend([
            [handle(right, -rx), y],
            [right, handle(y, ry)],
            [right, y + ry],
        ]);
        line_to(&mut points, [right, bottom - ry]);
        points.extend([
            [right, handle(bottom, -ry)],
            [handle(right, -rx), bottom],
            [right - rx, bottom],
        ]);
        line_to(&mut points, [x + rx, bottom]);
        points.extend([
            [handle(x, rx), bottom],
            [x, handle(bottom, -ry)],
            [x, bottom - ry],
        ]);
        line_to(&mut points, [x, y + ry]);
        points.extend([[x, handle(y, ry)], [handle(x, rx), y], [x + rx, y]]);
    }
    add_path(points, true, transform)
}
