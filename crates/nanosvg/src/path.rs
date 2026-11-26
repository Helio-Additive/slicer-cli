//! Cubic paths in the pinned NanoSVG parser (nanosvg.h 130–137, 540–616, 1029–1078).
pub type Point = [f32; 2];

#[derive(Clone, Debug, PartialEq)]
pub struct Path {
    pub points: Vec<Point>,
    pub closed: bool,
    pub bounds: [f32; 4],
}

// nanosvg.h 540–544, nsvg__xformPoint.
fn transform(point: Point, t: [f32; 6]) -> Point {
    // Under the pinned build flags clang contracts the first product on
    // arm64. The baseline x86_64 build rounds both products before addition.
    #[cfg(any(target_arch = "aarch64", target_feature = "fma"))]
    let xy = [
        point[0].mul_add(t[0], point[1] * t[2]),
        point[0].mul_add(t[1], point[1] * t[3]),
    ];
    #[cfg(not(any(target_arch = "aarch64", target_feature = "fma")))]
    let xy = [
        point[0] * t[0] + point[1] * t[2],
        point[0] * t[1] + point[1] * t[3],
    ];
    [xy[0] + t[4], xy[1] + t[5]]
}

// nanosvg.h 230–232. Equal operands (including signed zero) select b.
fn minf(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

fn maxf(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}

// nanosvg.h 555–616, nsvg__curveBounds. The extrema are found in double
// precision; the resulting coordinates are stored as floats.
fn curve_bounds(curve: &[Point]) -> [f32; 4] {
    const EPSILON: f64 = 1e-12;
    let mut bounds = [
        minf(curve[0][0], curve[3][0]),
        minf(curve[0][1], curve[3][1]),
        maxf(curve[0][0], curve[3][0]),
        maxf(curve[0][1], curve[3][1]),
    ];
    if curve[1..3]
        .iter()
        .all(|p| p[0] >= bounds[0] && p[0] <= bounds[2] && p[1] >= bounds[1] && p[1] <= bounds[3])
    {
        return bounds;
    }
    for dim in 0..2 {
        let p0 = f64::from(curve[0][dim]);
        let p1 = f64::from(curve[1][dim]);
        let p2 = f64::from(curve[2][dim]);
        let p3 = f64::from(curve[3][dim]);
        let a = -3.0 * p0 + 9.0 * p1 - 9.0 * p2 + 3.0 * p3;
        let b = 6.0 * p0 - 12.0 * p1 + 6.0 * p2;
        let c = 3.0 * p1 - 3.0 * p0;
        let mut add_root = |t: f64| {
            if t > EPSILON && t < 1.0 - EPSILON {
                let it = 1.0 - t;
                let v = (it * it * it * p0
                    + 3.0 * it * it * t * p1
                    + 3.0 * it * t * t * p2
                    + t * t * t * p3) as f32;
                bounds[dim] = minf(bounds[dim], v);
                bounds[dim + 2] = maxf(bounds[dim + 2], v);
            }
        };
        if a.abs() < EPSILON {
            if b.abs() > EPSILON {
                add_root(-c / b);
            }
        } else {
            let discriminant = b * b - 4.0 * c * a;
            if discriminant > EPSILON {
                let sqrt = discriminant.sqrt();
                add_root((-b + sqrt) / (2.0 * a));
                add_root((-b - sqrt) / (2.0 * a));
            }
        }
    }
    bounds
}

// nsvg__lineTo, nanosvg.h 721–733. Each line uses two controls at thirds.
pub(crate) fn line_to(points: &mut Vec<Point>, endpoint: Point) {
    let Some(&last) = points.last() else { return };
    let dx = endpoint[0] - last[0];
    let dy = endpoint[1] - last[1];
    points.extend([
        [last[0] + dx / 3.0, last[1] + dy / 3.0],
        [endpoint[0] - dx / 3.0, endpoint[1] - dy / 3.0],
        endpoint,
    ]);
}

/// nsvg__addPath. Consume the parser's start point followed by three
/// control/end points per cubic segment and apply the current SVG transform.
pub fn add_path(mut points: Vec<Point>, closed: bool, xform: [f32; 6]) -> Option<Path> {
    if points.len() < 4 || (points.len() - 1) % 3 != 0 {
        return None;
    }
    if closed {
        let first = points[0];
        line_to(&mut points, first);
    }
    for p in &mut points {
        *p = transform(*p, xform);
    }
    let mut bounds = curve_bounds(&points[..4]);
    for curve in points[3..].windows(4).step_by(3) {
        let b = curve_bounds(curve);
        bounds = [
            minf(bounds[0], b[0]),
            minf(bounds[1], b[1]),
            maxf(bounds[2], b[2]),
            maxf(bounds[3], b[3]),
        ];
    }
    Some(Path {
        points,
        closed,
        bounds,
    })
}
