//! NanoSVG coordinate units before viewBox and element transforms.
use crate::number::{atof, parse_number};

/// NSVGunits, nanosvg.h 371–382.
#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Unit {
    User,
    Px,
    Pt,
    Pc,
    Mm,
    Cm,
    In,
    Percent,
    Em,
    Ex,
}

impl Unit {
    // nsvg__parseUnits, nanosvg.h 1494–1514. Only the prefix is inspected.
    fn parse(suffix: &[u8]) -> Self {
        match suffix {
            [b'p', b'x', ..] => Self::Px,
            [b'p', b't', ..] => Self::Pt,
            [b'p', b'c', ..] => Self::Pc,
            [b'm', b'm', ..] => Self::Mm,
            [b'c', b'm', ..] => Self::Cm,
            [b'i', b'n', ..] => Self::In,
            [b'%', ..] => Self::Percent,
            [b'e', b'm', ..] => Self::Em,
            [b'e', b'x', ..] => Self::Ex,
            _ => Self::User,
        }
    }
}

/// NSVGcoordinate, nanosvg.h 384–387.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Coordinate {
    pub value: f32,
    pub units: Unit,
}

impl Coordinate {
    /// nsvg__parseCoordinateRaw, nanosvg.h 1526–1533. The unit suffix starts
    /// after the whole number, including digits beyond the 63-byte buffer.
    pub fn parse(text: &[u8]) -> Self {
        let n = parse_number(text);
        Self {
            value: atof(&text[..n.min(63)]) as f32,
            units: Unit::parse(&text[n..]),
        }
    }

    /// nsvg__convertToPixels, nanosvg.h 789–805. Percentages use the
    /// supplied axis origin and length. em and ex use the current font size.
    pub fn to_pixels(self, origin: f32, length: f32, dpi: f32, font_size: f32) -> f32 {
        let v = self.value;
        match self.units {
            Unit::User | Unit::Px => v,
            Unit::Pt => v / 72. * dpi,
            Unit::Pc => v / 6. * dpi,
            Unit::Mm => v / 25.4 * dpi,
            Unit::Cm => v / 2.54 * dpi,
            Unit::In => v * dpi,
            Unit::Em => v * font_size,
            Unit::Ex => v * font_size * 0.52,
            Unit::Percent => {
                let fraction = v / 100.;
                // The pinned native build contracts the multiply and add on arm64.
                #[cfg(any(target_arch = "aarch64", target_feature = "fma"))]
                {
                    fraction.mul_add(length, origin)
                }
                #[cfg(not(any(target_arch = "aarch64", target_feature = "fma")))]
                {
                    origin + fraction * length
                }
            }
        }
    }
}
