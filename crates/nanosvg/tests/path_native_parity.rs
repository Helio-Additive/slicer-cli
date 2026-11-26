//! Path coordinates and extrema from nsvg__addPath at the BambuStudio pin.
use nanosvg::path::add_path;

#[test]
fn cubic_paths_match_native_float_bits() {
    #[cfg(any(target_arch = "aarch64", target_feature = "fma"))]
    let vectors = include_str!("data/paths-fused.txt");
    #[cfg(not(any(target_arch = "aarch64", target_feature = "fma")))]
    let vectors = include_str!("data/paths-unfused.txt");

    let mut rows = vectors.lines();
    let mut cases = 0;
    while let Some(input) = rows.next() {
        let mut words = input.split_ascii_whitespace();
        assert_eq!(words.next(), Some("input"));
        let closed = words.next().unwrap() == "1";
        let count: usize = words.next().unwrap().parse().unwrap();
        let mut number = || f32::from_bits(u32::from_str_radix(words.next().unwrap(), 16).unwrap());
        let transform = std::array::from_fn(|_| number());
        let points = (0..count).map(|_| [number(), number()]).collect();
        let expected = rows.next().unwrap();
        let result = add_path(points, closed, transform);
        if expected == "none" {
            assert!(result.is_none(), "{input}");
        } else {
            let path = result.expect(input);
            let mut fields = expected.split_ascii_whitespace();
            assert_eq!(fields.next(), Some("path"));
            assert_eq!(fields.next().unwrap() == "1", path.closed);
            assert_eq!(
                fields.next().unwrap().parse::<usize>().unwrap(),
                path.points.len()
            );
            let want: Vec<_> = fields
                .map(|word| u32::from_str_radix(word, 16).unwrap())
                .collect();
            let got: Vec<_> = path
                .bounds
                .into_iter()
                .chain(path.points.into_iter().flatten())
                .map(f32::to_bits)
                .collect();
            assert_eq!(want, got, "{input}");
        }
        cases += 1;
    }
    assert_eq!(cases, 344);
}
