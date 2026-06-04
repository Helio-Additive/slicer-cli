//! Polyline stitcher for Arachne.
//!
//! Stitches together disconnected polylines into continuous paths.

use crate::geometry::{Point, Polyline};
use crate::{scale, CoordF};

/// Stitch together a collection of polylines
/// Arachne/utils/PolylineStitcher.cpp:15-45
pub fn stitch_polylines(polylines: &[Polyline], tolerance: CoordF) -> Vec<Polyline> {
    // Handle empty input
    // Arachne/utils/PolylineStitcher.cpp:17-19
    if polylines.is_empty() {
        // Arachne/utils/PolylineStitcher.cpp:18
        return vec![];
    }

    // Initialize result with first polyline
    // Arachne/utils/PolylineStitcher.cpp:21-23
    let mut result: Vec<Polyline> = vec![polylines[0].clone()];
    // Track which polylines have been used
    // Arachne/utils/PolylineStitcher.cpp:24-25
    let mut used: Vec<bool> = vec![false; polylines.len()];
    // Arachne/utils/PolylineStitcher.cpp:25
    used[0] = true;

    // Convert tolerance to scaled units squared for distance comparison
    // Arachne/utils/PolylineStitcher.cpp:27-28
    let tolerance_scaled = scale(tolerance);
    // Arachne/utils/PolylineStitcher.cpp:28
    let best_dist_threshold = (tolerance_scaled * tolerance_scaled) as i128;

    // Iterate through remaining polylines
    // Arachne/utils/PolylineStitcher.cpp:30-75
    for _ in 1..polylines.len() {
        // Get last point of current chain
        // Arachne/utils/PolylineStitcher.cpp:32-33
        let last_poly = result.last().unwrap();
        // Arachne/utils/PolylineStitcher.cpp:33
        let last_point = last_poly.points().last().unwrap();

        // Find closest unused polyline endpoint
        // Arachne/utils/PolylineStitcher.cpp:35-37
        let mut best_idx = None;
        // Arachne/utils/PolylineStitcher.cpp:36
        let mut best_dist = best_dist_threshold;
        // Arachne/utils/PolylineStitcher.cpp:37
        let mut should_reverse = false;

        // Check all unused polylines
        // Arachne/utils/PolylineStitcher.cpp:39-60
        for (i, poly) in polylines.iter().enumerate() {
            // Arachne/utils/PolylineStitcher.cpp:40
            // Arachne/utils/PolylineStitcher.cpp:40
            if used[i] {
                // Arachne/utils/PolylineStitcher.cpp:41
                continue;
            }

            // Get polyline endpoints
            // Arachne/utils/PolylineStitcher.cpp:42-43
            // Arachne/utils/PolylineStitcher.cpp:42
            let first = poly.points().first().unwrap();
            // Arachne/utils/PolylineStitcher.cpp:43
            let last = poly.points().last().unwrap();

            // Check distance to first point and update best match if closer
            // Arachne/utils/PolylineStitcher.cpp:45-49
            // Arachne/utils/PolylineStitcher.cpp:45
            let dist_first = Point::distance_squared(last_point, first);
            // Arachne/utils/PolylineStitcher.cpp:46
            if dist_first < best_dist {
                // Arachne/utils/PolylineStitcher.cpp:46-49
                // Arachne/utils/PolylineStitcher.cpp:47
                best_dist = dist_first;
                // Arachne/utils/PolylineStitcher.cpp:48
                best_idx = Some(i);
                // Arachne/utils/PolylineStitcher.cpp:49
                should_reverse = false;
            }

            // Check distance to last point (requires reversal) and update best match if closer
            // Arachne/utils/PolylineStitcher.cpp:51-55
            // Arachne/utils/PolylineStitcher.cpp:51
            let dist_last = Point::distance_squared(last_point, last);
            // Arachne/utils/PolylineStitcher.cpp:52
            if dist_last < best_dist {
                // Arachne/utils/PolylineStitcher.cpp:52-55
                // Arachne/utils/PolylineStitcher.cpp:53
                best_dist = dist_last;
                // Arachne/utils/PolylineStitcher.cpp:54
                best_idx = Some(i);
                // Arachne/utils/PolylineStitcher.cpp:55
                should_reverse = true;
            }
        }

        // Append best match if found within tolerance
        // Arachne/utils/PolylineStitcher.cpp:62-72
        // Arachne/utils/PolylineStitcher.cpp:62
        if let Some(idx) = best_idx {
            // Arachne/utils/PolylineStitcher.cpp:63-65
            let mut next_poly = polylines[idx].clone();
            // Arachne/utils/PolylineStitcher.cpp:64
            if should_reverse {
                // Arachne/utils/PolylineStitcher.cpp:65
                next_poly.reverse();
            }
            // Arachne/utils/PolylineStitcher.cpp:67-70
            if let Some(last) = result.last_mut() {
                // Arachne/utils/PolylineStitcher.cpp:68
                last.points_mut().extend(next_poly.points().iter().skip(1));
            }
            // Arachne/utils/PolylineStitcher.cpp:71
            used[idx] = true;
        // Start new chain if no close match found
        // Arachne/utils/PolylineStitcher.cpp:73-80
        } else {
            // Arachne/utils/PolylineStitcher.cpp:74-79
            // Arachne/utils/PolylineStitcher.cpp:74
            for (i, _) in polylines.iter().enumerate() {
                // Arachne/utils/PolylineStitcher.cpp:75-78
                // Arachne/utils/PolylineStitcher.cpp:75
                if !used[i] {
                    // Arachne/utils/PolylineStitcher.cpp:76
                    // Arachne/utils/PolylineStitcher.cpp:76
                    result.push(polylines[i].clone());
                    // Arachne/utils/PolylineStitcher.cpp:77
                    used[i] = true;
                    // Arachne/utils/PolylineStitcher.cpp:78
                    break;
                }
            }
        }
    }

    // Return stitched polylines
    // Arachne/utils/PolylineStitcher.cpp:83
    result
}
