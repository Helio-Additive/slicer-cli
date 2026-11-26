//! NanoSVG circle and ellipse geometry before the image-wide viewBox transform.
use crate::coordinate::Coordinate;
use crate::line::Viewport;
use crate::path::{add_path, Path};

/// nsvg__parseCircle (nanosvg.h 2466–2494). Percent radii use the viewport
/// diagonal divided by the square root of two.
pub fn parse_circle(
    attributes: &[(&[u8], &[u8])],
    viewport: Viewport,
    mut font_size: f32,
    transform: [f32; 6],
) -> Option<Path> {
    let (mut cx, mut cy, mut r) = (0., 0., 0.);
    for &(name, value) in attributes {
        match name {
            b"cx" => cx = viewport.coordinate(value, 0, font_size),
            b"cy" => cy = viewport.coordinate(value, 1, font_size),
            b"r" => r = viewport.diagonal_coordinate(value, font_size).abs(),
            b"font-size" => font_size = viewport.diagonal_coordinate(value, font_size),
            b"style" => font_size = viewport.style_font_size(value, font_size),
            _ => {}
        }
    }
    ellipse_path(cx, cy, r, r, transform)
}

/// nsvg__parseEllipse (nanosvg.h 2496–2527). Percent radii use the width
/// and height separately, each from an origin of zero.
pub fn parse_ellipse(
    attributes: &[(&[u8], &[u8])],
    viewport: Viewport,
    mut font_size: f32,
    transform: [f32; 6],
) -> Option<Path> {
    let (mut cx, mut cy, mut rx, mut ry) = (0., 0., 0., 0.);
    for &(name, value) in attributes {
        let length = |axis: usize| Coordinate::parse(value).to_pixels(0., viewport.size[axis], viewport.dpi, font_size).abs();
        match name {
            b"cx" => cx = viewport.coordinate(value, 0, font_size),
            b"cy" => cy = viewport.coordinate(value, 1, font_size),
            b"rx" => rx = length(0),
            b"ry" => ry = length(1),
            b"font-size" => font_size = viewport.diagonal_coordinate(value, font_size),
            b"style" => font_size = viewport.style_font_size(value, font_size),
            _ => {}
        }
    }
    ellipse_path(cx, cy, rx, ry, transform)
}

// nanosvg.h 196, 2480–2492, 2511–2525. Each quadrant is one cubic.
fn ellipse_path(cx: f32, cy: f32, rx: f32, ry: f32, transform: [f32; 6]) -> Option<Path> {
    if !(rx > 0. && ry > 0.) {
        return None;
    }
    const KAPPA90: f32 = 0.5522847493;
    #[cfg(any(target_arch = "aarch64", target_feature = "fma"))]
    let handle = |center: f32, radius: f32| radius.mul_add(KAPPA90, center);
    #[cfg(not(any(target_arch = "aarch64", target_feature = "fma")))]
    let handle = |center: f32, radius: f32| center + radius * KAPPA90;
    let right = cx + rx;
    let left = cx - rx;
    let bottom = cy + ry;
    let top = cy - ry;
    let points = vec![
        [right, cy],
        [right, handle(cy, ry)], [handle(cx, rx), bottom], [cx, bottom],
        [handle(cx, -rx), bottom], [left, handle(cy, ry)], [left, cy],
        [left, handle(cy, -ry)], [handle(cx, -rx), top], [cx, top],
        [handle(cx, rx), top], [right, handle(cy, -ry)], [right, cy],
    ];
    add_path(points, true, transform)
}
