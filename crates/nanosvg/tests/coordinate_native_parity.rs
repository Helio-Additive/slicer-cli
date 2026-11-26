//! Raw NanoSVG coordinates and conversion to pixels before shape transforms.
use nanosvg::coordinate::Coordinate;

#[test]
fn raw_units_and_pixels_match_native() {
    for row in include_str!("data/coordinates.txt").lines() {
        let (value, fields) = row.split_once('|').unwrap();
        let mut fields = fields.split_ascii_whitespace();
        let mut number =
            || f32::from_bits(u32::from_str_radix(fields.next().unwrap(), 16).unwrap());
        let (dpi, font, origin, length) = (number(), number(), number(), number());
        let units = fields.next().unwrap().parse::<u8>().unwrap();
        let raw = u32::from_str_radix(fields.next().unwrap(), 16).unwrap();
        let arm = u32::from_str_radix(fields.next().unwrap(), 16).unwrap();
        let x86 = u32::from_str_radix(fields.next().unwrap(), 16).unwrap();
        let actual = Coordinate::parse(value.as_bytes());
        assert_eq!(actual.units as u8, units, "{row}");
        assert_eq!(actual.value.to_bits(), raw, "{row}");
        let expected = if cfg!(any(target_arch = "aarch64", target_feature = "fma")) {
            arm
        } else {
            x86
        };
        assert_eq!(
            actual.to_pixels(origin, length, dpi, font).to_bits(),
            expected,
            "{row}"
        );
    }
}
