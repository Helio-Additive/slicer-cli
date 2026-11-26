//! NanoSVG line geometry before the image-wide viewBox transform.
use crate::coordinate::Coordinate;
use crate::path::{add_path, line_to, Path, Point};
use crate::whitespace;

/// The user-space viewBox and document resolution available to nsvg__parseLine.
#[derive(Clone, Copy, Debug)]
pub struct Viewport {
    pub origin: Point,
    pub size: Point,
    pub dpi: f32,
}

impl Viewport {
    fn coordinate(self, text: &[u8], axis: usize, font_size: f32) -> f32 {
        Coordinate::parse(text).to_pixels(self.origin[axis], self.size[axis], self.dpi, font_size)
    }

    // nanosvg.h 783-787, 1840-1841. Relative font sizes are resolved as
    // each attribute is encountered, including within a style declaration.
    fn font_size(self, text: &[u8], inherited: f32) -> f32 {
        let [w, h] = self.size;
        #[cfg(any(target_arch = "aarch64", target_feature = "fma"))]
        let squared = w.mul_add(w, h * h);
        #[cfg(not(any(target_arch = "aarch64", target_feature = "fma")))]
        let squared = w * w + h * h;
        Coordinate::parse(text).to_pixels(0., squared.sqrt() / 2.0f32.sqrt(), self.dpi, inherited)
    }

    // Font geometry from nsvg__parseStyle / nsvg__parseNameValue
    // (nanosvg.h 1860-1913). The caller supplies the element transform.
    fn style_font_size(self, text: &[u8], mut font: f32) -> f32 {
        for decl in text.split(|b| *b == b';') {
            let Some(colon) = decl.iter().position(|b| *b == b':') else {
                continue;
            };
            let key = &decl[..colon];
            let Some(start) = key.iter().position(|b| !whitespace(*b)) else {
                continue;
            };
            let end = key.iter().rposition(|b| !whitespace(*b)).unwrap() + 1;
            let key = &key[start..end.min(start + 511)];
            let value = &decl[colon..];
            let i = value
                .iter()
                .position(|b| *b != b':' && !whitespace(*b))
                .unwrap_or(value.len());
            let value = &value[i..value.len().min(i + 511)];
            match key {
                b"font-size" => font = self.font_size(value, font),
                b"style" => font = self.style_font_size(value, font),
                _ => {}
            }
        }
        font
    }
}

/// nsvg__parseLine (nanosvg.h 2529–2554). Read the endpoints in XML
/// attribute order, then convert the segment into an open cubic path.
pub fn parse_line(
    attributes: &[(&[u8], &[u8])],
    viewport: Viewport,
    mut font_size: f32,
    transform: [f32; 6],
) -> Option<Path> {
    let mut first = [0.; 2];
    let mut last = [0.; 2];
    for &(name, value) in attributes {
        match name {
            b"x1" => first[0] = viewport.coordinate(value, 0, font_size),
            b"y1" => first[1] = viewport.coordinate(value, 1, font_size),
            b"x2" => last[0] = viewport.coordinate(value, 0, font_size),
            b"y2" => last[1] = viewport.coordinate(value, 1, font_size),
            b"font-size" => font_size = viewport.font_size(value, font_size),
            b"style" => font_size = viewport.style_font_size(value, font_size),
            _ => {}
        }
    }
    let mut points = vec![first];
    line_to(&mut points, last);
    add_path(points, false, transform)
}
