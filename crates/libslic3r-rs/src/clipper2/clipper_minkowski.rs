//! `include/clipper2/clipper.minkowski.h` of Clipper2 as the engine pins it (version 1.5.2,
//! `src/clipper2/Clipper2Lib`): Minkowski Sum and Difference. Compiled twice, see `mod.rs`.
//!
//! `tests/clipper2_d_native_parity.rs` and its `_z` twin hold it equal to the library.

use super::clipper_core::*;
use super::clipper_engine::*;

/// clipper.minkowski.h:23-78 `detail::Minkowski`
/// C++: inline Paths64 Minkowski(const Path64& pattern, const Path64& path, bool isSum, bool isClosed)
/// `p + pt2` and `p - pt2` are `Point::operator+` and `operator-`: new points, z = 0.
fn minkowski(pattern: &Path64, path: &Path64, is_sum: bool, is_closed: bool) -> Paths64 {
    let delta: usize = if is_closed { 0 } else { 1 };
    let pat_len = pattern.len();
    let path_len = path.len();
    if pat_len == 0 || path_len == 0 {
        return Paths64::new();
    }
    let mut tmp = Paths64::with_capacity(path_len);

    if is_sum {
        for p in path {
            let mut path2 = Path64::with_capacity(pattern.len());
            for pt2 in pattern {
                path2.push(*p + *pt2);
            }
            tmp.push(path2);
        }
    } else {
        for p in path {
            let mut path2 = Path64::with_capacity(pattern.len());
            for pt2 in pattern {
                path2.push(*p - *pt2);
            }
            tmp.push(path2);
        }
    }

    let mut result = Paths64::with_capacity((path_len - delta) * pat_len);
    let mut g = if is_closed { path_len - 1 } else { 0 };
    let mut h = pat_len - 1;
    for i in delta..path_len {
        for j in 0..pat_len {
            let mut quad = Path64::with_capacity(4);
            {
                quad.push(tmp[g][h]);
                quad.push(tmp[i][h]);
                quad.push(tmp[i][j]);
                quad.push(tmp[g][j]);
            };
            if !is_positive(&quad) {
                quad.reverse();
            }
            result.push(quad);
            h = j;
        }
        g = i;
    }
    result
}

/// clipper.minkowski.h:80-87 `detail::Union`
/// C++: inline Paths64 Union(const Paths64& subjects, FillRule fillrule)
fn union(subjects: &Paths64, fillrule: FillRule) -> Paths64 {
    let mut result = Paths64::new();
    let mut clipper = Clipper64::new();
    clipper.add_subject(subjects);
    clipper.execute(ClipType::Union, fillrule, &mut result);
    result
}

/// clipper.minkowski.h:91-94
/// C++: inline Paths64 MinkowskiSum(const Path64& pattern, const Path64& path, bool isClosed)
pub fn minkowski_sum(pattern: &Path64, path: &Path64, is_closed: bool) -> Paths64 {
    union(&minkowski(pattern, path, true, is_closed), FillRule::NonZero)
}

/// clipper.minkowski.h:96-104
/// C++: inline PathsD MinkowskiSum(const PathD& pattern, const PathD& path, bool isClosed, int decimalPlaces = 2)
/// `pow(10, decimalPlaces)` is `pow(double, double)`. `ScalePath` throws for a scale of
/// zero (10 to a power below -323), and `ScalePaths` back for `1 / scale` of zero.
pub fn minkowski_sum_d(pattern: &PathD, path: &PathD, is_closed: bool, decimal_places: i32) -> Result<PathsD, Clipper2Exception> {
    let mut error_code = 0;
    let scale = 10.0f64.powf(decimal_places as f64);
    let pat64 = scale_path64_from_d_uniform(pattern, scale, &mut error_code)?;
    let path64 = scale_path64_from_d_uniform(path, scale, &mut error_code)?;
    let tmp = union(&minkowski(&pat64, &path64, true, is_closed), FillRule::NonZero);
    scale_pathsd_from_64_uniform(&tmp, 1.0 / scale, &mut error_code)
}

/// clipper.minkowski.h:106-109
/// C++: inline Paths64 MinkowskiDiff(const Path64& pattern, const Path64& path, bool isClosed)
pub fn minkowski_diff(pattern: &Path64, path: &Path64, is_closed: bool) -> Paths64 {
    union(&minkowski(pattern, path, false, is_closed), FillRule::NonZero)
}

/// clipper.minkowski.h:111-119
/// C++: inline PathsD MinkowskiDiff(const PathD& pattern, const PathD& path, bool isClosed, int decimalPlaces = 2)
pub fn minkowski_diff_d(pattern: &PathD, path: &PathD, is_closed: bool, decimal_places: i32) -> Result<PathsD, Clipper2Exception> {
    let mut error_code = 0;
    let scale = 10.0f64.powf(decimal_places as f64);
    let pat64 = scale_path64_from_d_uniform(pattern, scale, &mut error_code)?;
    let path64 = scale_path64_from_d_uniform(path, scale, &mut error_code)?;
    let tmp = union(&minkowski(&pat64, &path64, false, is_closed), FillRule::NonZero);
    scale_pathsd_from_64_uniform(&tmp, 1.0 / scale, &mut error_code)
}
