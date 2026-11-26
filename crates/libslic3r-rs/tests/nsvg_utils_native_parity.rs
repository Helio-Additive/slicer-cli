//! BambuStudio 56a57d0 NSVGUtils.cpp 325–374, native NanoSVG paths and output.
use nanosvg::path::Path;
use slicer::geometry::Point;
use slicer::nsvg_utils::{linearize_path, NSVGLineParams};

fn expected_points(row: &str, kind: &str) -> Vec<Point> {
    let mut fields = row.split_ascii_whitespace();
    assert_eq!(fields.next(), Some(kind));
    fields
        .map(|field| {
            let (x, y) = field.split_once(',').unwrap();
            Point::new(x.parse().unwrap(), y.parse().unwrap())
        })
        .collect()
}

#[test]
fn linearized_native_paths_match_polygons_and_polylines() {
    let mut paths = Vec::new();
    let mut cases = 0;
    let mut outputs = 0;
    let mut rows = include_str!("data/nsvg_utils/linear.txt").lines();
    while let Some(row) = rows.next() {
        let mut words = row.split_ascii_whitespace();
        match words.next().unwrap() {
            "case" => {
                paths.clear();
                cases += 1;
            }
            "path" => {
                let closed = words.next().unwrap() == "1";
                let count: usize = words.next().unwrap().parse().unwrap();
                let mut number =
                    || f32::from_bits(u32::from_str_radix(words.next().unwrap(), 16).unwrap());
                let points = (0..count).map(|_| [number(), number()]).collect();
                paths.push(Path {
                    points,
                    closed,
                    bounds: [0.0; 4],
                });
            }
            "lines" => {
                let variant: usize = words.next().unwrap().parse().unwrap();
                let mut params = NSVGLineParams::new(if variant == 0 { 0.15 } else { 2.0 });
                params.max_level = if variant == 2 { 2 } else { 9 };
                params.scale = if variant == 0 { 1.0 } else { 128.0 };
                params.is_y_negative = variant == 1;
                let result = linearize_path(&paths, &params);
                assert_eq!(
                    result.polygons.len(),
                    words.next().unwrap().parse::<usize>().unwrap()
                );
                assert_eq!(
                    result.polylines.len(),
                    words.next().unwrap().parse::<usize>().unwrap()
                );
                for polygon in result.polygons {
                    assert_eq!(
                        polygon.points,
                        expected_points(rows.next().unwrap(), "polygon"),
                        "case {cases} variant {variant}"
                    );
                }
                for polyline in result.polylines {
                    assert_eq!(
                        polyline.points,
                        expected_points(rows.next().unwrap(), "polyline"),
                        "case {cases} variant {variant}"
                    );
                }
                outputs += 1;
            }
            "end" => (),
            other => panic!("unexpected vector row {other}"),
        }
    }
    assert_eq!((cases, outputs), (21, 51));
}
