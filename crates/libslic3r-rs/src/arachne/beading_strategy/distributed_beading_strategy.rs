//! Distributed beading strategy implementation.
//!
//! C++ Reference:
//! - Arachne/BeadingStrategy/DistributedBeadingStrategy.hpp
//! - Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp
//!
//! This beading strategy chooses a wall count that would make the line width
//! deviate the least from the optimal line width, and then distributes the lines
//! evenly among the thickness available.

use super::beading_strategy::{Beading, BeadingStrategy};
use crate::Coord;

// Distributed beading strategy that evenly distributes line width discrepancies
// across multiple beads using a gaussian-like distribution.
// Arachne/BeadingStrategy/DistributedBeadingStrategy.hpp:16-36
#[derive(Debug, Clone)]
pub struct DistributedBeadingStrategy {
    // Base strategy fields (from BeadingStrategy parent class)
    // Arachne/BeadingStrategy/BeadingStrategy.hpp
    optimal_width: Coord,
    default_transition_length: Coord,
    transitioning_angle: f64,
    wall_split_middle_threshold: f64,
    wall_add_middle_threshold: f64,

    // (1 / distribution_radius)^2 - used for gaussian-like weight calculation
    // Arachne/BeadingStrategy/DistributedBeadingStrategy.hpp:19
    one_over_distribution_radius_squared: f32,

    // Strategy name for debugging
    name: String,
}

impl DistributedBeadingStrategy {
    // Create a new distributed beading strategy
    // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:9-22
    ///
    // # Arguments
    // * `optimal_width` - The optimal bead width
    // * `default_transition_length` - Default length for transitions between different bead counts
    // * `transitioning_angle` - Angle for transitioning between bead counts
    // * `wall_split_middle_threshold` - Threshold for splitting walls in the middle
    // * `wall_add_middle_threshold` - Threshold for adding walls in the middle
    // * `distribution_radius` - Radius (in number of beads) over which to distribute discrepancies
    pub fn new(
        optimal_width: Coord,
        default_transition_length: Coord,
        transitioning_angle: f64,
        wall_split_middle_threshold: f64,
        wall_add_middle_threshold: f64,
        distribution_radius: i32,
    ) -> Self {
        // Calculate one_over_distribution_radius_squared from distribution_radius
        // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:16-20
        // C++: if(distribution_radius >= 2)
        // C++:     one_over_distribution_radius_squared = 1.0f / (distribution_radius - 1) * 1.0f / (distribution_radius - 1);
        // C++: else
        // C++:     one_over_distribution_radius_squared = 1.0f / 1 * 1.0f / 1;
        let one_over_distribution_radius_squared = if distribution_radius >= 2 {
            let divisor = (distribution_radius - 1) as f32;
            1.0 / divisor * 1.0 / divisor
        } else {
            1.0
        };

        // Create instance with initialized fields
        // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:21
        // C++: name = "DistributedBeadingStrategy";
        Self {
            optimal_width,
            default_transition_length,
            transitioning_angle,
            wall_split_middle_threshold,
            wall_add_middle_threshold,
            one_over_distribution_radius_squared,
            name: "DistributedBeadingStrategy".to_string(),
        }
    }

    // Compute beading for a given thickness and bead count
    // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:24-85
    pub fn compute(&self, thickness: Coord, bead_count: Coord) -> Beading {
        // Initialize beading result
        // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:26-27
        // C++: Beading ret;
        // C++: ret.total_thickness = thickness;
        let mut ret = Beading {
            total_thickness: thickness,
            bead_widths: Vec::new(),
            toolpath_locations: Vec::new(),
            left_over: 0,
        };

        // Handle case where bead_count > 2
        // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:29-60
        // C++: if (bead_count > 2) {
        if bead_count > 2 {
            // Calculate amount to be distributed among beads
            // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:30-31
            // C++: const coord_t to_be_divided = thickness - bead_count * optimal_width;
            // C++: const float middle = static_cast<float>(bead_count - 1) / 2;
            // FIDELITY-NOTE(F2): C++ `bead_count * optimal_width` is int32*int32 and
            // wraps at 32 bits; crate Coord is i64 so this is wider. Realistic values
            // stay within int32 range. Narrowing Coord is the crate-wide F2 rework.
            let to_be_divided = thickness - bead_count * self.optimal_width;
            let middle = (bead_count - 1) as f32 / 2.0;

            // Lambda to calculate gaussian-like weight for each bead
            // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:33-36
            // C++: const auto getWeight = [middle, this](coord_t bead_idx) {
            // C++:     const float dev_from_middle = bead_idx - middle;
            // C++:     return std::max(0.0f, 1.0f - one_over_distribution_radius_squared * dev_from_middle * dev_from_middle);
            // C++: };
            let get_weight = |bead_idx: Coord| -> f32 {
                let dev_from_middle = bead_idx as f32 - middle;
                0.0_f32.max(
                    1.0 - self.one_over_distribution_radius_squared
                        * dev_from_middle
                        * dev_from_middle,
                )
            };

            // Calculate weights for all beads
            // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:38-41
            // C++: std::vector<float> weights;
            // C++: weights.resize(bead_count);
            // C++: for (coord_t bead_idx = 0; bead_idx < bead_count; bead_idx++)
            // C++:     weights[bead_idx] = getWeight(bead_idx);
            let mut weights = Vec::with_capacity(bead_count as usize);
            for bead_idx in 0..bead_count {
                weights.push(get_weight(bead_idx));
            }

            // Calculate total weight for normalization
            // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:43
            // C++: const float total_weight = std::accumulate(weights.cbegin(), weights.cend(), 0.f);
            let total_weight: f32 = weights.iter().sum();

            // Distribute the extra thickness among beads according to their weights
            // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:44-58
            // C++: coord_t accumulated_width = 0;
            // C++: for (coord_t bead_idx = 0; bead_idx < bead_count; bead_idx++) {
            let mut accumulated_width = 0;
            for bead_idx in 0..bead_count {
                // Calculate weight fraction for this bead
                // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:46-48
                // C++: const float weight_fraction = weights[bead_idx] / total_weight;
                // C++: const coord_t splitup_left_over_weight = to_be_divided * weight_fraction;
                // C++: const coord_t width = (bead_idx == bead_count - 1) ? thickness - accumulated_width : optimal_width + splitup_left_over_weight;
                let weight_fraction = weights[bead_idx as usize] / total_weight;
                // C++: const coord_t splitup_left_over_weight = to_be_divided * weight_fraction;
                // `to_be_divided` is coord_t and `weight_fraction` is float (f32).
                // Per C++ usual arithmetic conversions, the integer operand is promoted to
                // `float`, the product is computed in f32, then truncated to coord_t.
                // Mirror that exact order: f32 multiply, then truncate to Coord.
                // FIDELITY-NOTE(F2): C++ coord_t is int32; crate Coord is i64. The promotion
                // of the integer operand to f32 (and the truncation back) is identical for
                // values within int32 range, which `to_be_divided` always is in practice.
                let splitup_left_over_weight = (to_be_divided as f32 * weight_fraction) as Coord;
                let width = if bead_idx == bead_count - 1 {
                    thickness - accumulated_width
                } else {
                    self.optimal_width + splitup_left_over_weight
                };

                // Calculate toolpath location and add bead width
                // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:50-57
                // C++: // Be aware that toolpath_locations is computed by dividing the width by 2, so toolpath_locations
                // C++: // could be off by 1 because of rounding errors.
                // C++: if (bead_idx == 0)
                // C++:     ret.toolpath_locations.emplace_back(width / 2);
                // C++: else
                // C++:     ret.toolpath_locations.emplace_back(ret.toolpath_locations.back() + (ret.bead_widths.back() + width) / 2);
                // C++: ret.bead_widths.emplace_back(width);
                // C++: accumulated_width += width;
                if bead_idx == 0 {
                    ret.toolpath_locations.push(width / 2);
                } else {
                    let prev_location = *ret.toolpath_locations.last().unwrap();
                    let prev_width = *ret.bead_widths.last().unwrap();
                    ret.toolpath_locations
                        .push(prev_location + (prev_width + width) / 2);
                }
                ret.bead_widths.push(width);
                accumulated_width += width;
            }

            // No leftover for this case
            // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:59-60
            // C++: ret.left_over = 0;
            // C++: assert((accumulated_width + ret.left_over) == thickness);
            ret.left_over = 0;
            debug_assert_eq!(accumulated_width + ret.left_over, thickness);

        // Handle case where bead_count == 2
        // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:61-67
        // C++: } else if (bead_count == 2) {
        } else if bead_count == 2 {
            // Split thickness equally between two beads
            // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:62-67
            // C++: const coord_t outer_width = thickness / 2;
            // C++: ret.bead_widths.emplace_back(outer_width);
            // C++: ret.bead_widths.emplace_back(outer_width);
            // C++: ret.toolpath_locations.emplace_back(outer_width / 2);
            // C++: ret.toolpath_locations.emplace_back(thickness - outer_width / 2);
            let outer_width = thickness / 2;
            ret.bead_widths.push(outer_width);
            ret.bead_widths.push(outer_width);
            ret.toolpath_locations.push(outer_width / 2);
            ret.toolpath_locations.push(thickness - outer_width / 2);

            // C++: ret.left_over = 0;
            ret.left_over = 0;

        // Handle case where bead_count == 1
        // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:68-72
        // C++: } else if (bead_count == 1) {
        } else if bead_count == 1 {
            // Single bead takes full thickness
            // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:69-71
            // C++: const coord_t outer_width = thickness;
            // C++: ret.bead_widths.emplace_back(outer_width);
            // C++: ret.toolpath_locations.emplace_back(outer_width / 2);
            let outer_width = thickness;
            ret.bead_widths.push(outer_width);
            ret.toolpath_locations.push(outer_width / 2);

            // C++: ret.left_over = 0;
            ret.left_over = 0;

        // Handle case where bead_count == 0
        // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:73-75
        // C++: } else {
        } else {
            // All thickness is leftover
            // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:74
            // C++: ret.left_over = thickness;
            ret.left_over = thickness;
        }

        // Assert that total bead width plus leftover equals thickness
        // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:77-82
        // C++: assert(([&ret = std::as_const(ret), thickness]() -> bool {
        // C++:     coord_t total_bead_width = 0;
        // C++:     for (const coord_t &bead_width : ret.bead_widths)
        // C++:         total_bead_width += bead_width;
        // C++:     return (total_bead_width + ret.left_over) == thickness;
        // C++: }()));
        debug_assert_eq!(
            ret.bead_widths.iter().sum::<Coord>() + ret.left_over,
            thickness
        );

        // Return computed beading
        // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:84
        // C++: return ret;
        ret
    }

    // Get the optimal bead count for a given thickness
    // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:87-93
    pub fn get_optimal_bead_count(&self, thickness: Coord) -> Coord {
        // Calculate naive count that fits for sure
        // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:89
        // C++: const coord_t naive_count = thickness / optimal_width; // How many lines we can fit in for sure.
        let naive_count = thickness / self.optimal_width;

        // Calculate remaining space
        // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:90
        // C++: const coord_t remainder = thickness - naive_count * optimal_width; // Space left after fitting that many lines.
        let remainder = thickness - naive_count * self.optimal_width;

        // Determine threshold based on whether naive_count is odd or even
        // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:91
        // C++: const coord_t minimum_line_width = optimal_width * (naive_count % 2 == 1 ? wall_split_middle_threshold : wall_add_middle_threshold);
        // C++ promotes `optimal_width` (coord_t) to double, multiplies in double, then
        // truncates the result back to coord_t. Mirror that exact order: f64 multiply,
        // then truncate to Coord.
        let threshold = if naive_count % 2 == 1 {
            self.wall_split_middle_threshold
        } else {
            self.wall_add_middle_threshold
        };
        let minimum_line_width = (self.optimal_width as f64 * threshold) as Coord;

        // Return count plus 1 if remainder is large enough
        // Arachne/BeadingStrategy/DistributedBeadingStrategy.cpp:92
        // C++: return naive_count + (remainder >= minimum_line_width); // If there's enough space, fit an extra one.
        naive_count
            + if remainder >= minimum_line_width {
                1
            } else {
                0
            }
    }

    // Get the strategy name
    pub fn name(&self) -> &str {
        &self.name
    }

    // Get the optimal width
    pub fn optimal_width(&self) -> Coord {
        self.optimal_width
    }
}

impl BeadingStrategy for DistributedBeadingStrategy {
    fn compute(&self, thickness: Coord, bead_count: Coord) -> Beading {
        self.compute(thickness, bead_count)
    }

    fn get_optimal_bead_count(&self, thickness: Coord) -> Coord {
        self.get_optimal_bead_count(thickness)
    }

    fn name(&self) -> &str {
        self.name()
    }

    fn optimal_width(&self) -> Coord {
        self.optimal_width
    }

    fn default_transition_length(&self) -> Coord {
        self.default_transition_length
    }

    fn transitioning_angle(&self) -> f64 {
        self.transitioning_angle
    }

    fn wall_split_middle_threshold(&self) -> f64 {
        self.wall_split_middle_threshold
    }

    fn wall_add_middle_threshold(&self) -> f64 {
        self.wall_add_middle_threshold
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scaled_f64;

    #[test]
    fn test_distributed_strategy_creation() {
        // Test basic strategy creation
        let strategy = DistributedBeadingStrategy::new(
            scaled_f64(0.4),
            scaled_f64(0.3),
            std::f64::consts::PI / 4.0,
            0.5,
            0.5,
            2,
        );

        assert_eq!(strategy.name(), "DistributedBeadingStrategy");
        assert_eq!(strategy.optimal_width(), scaled_f64(0.4));
    }

    #[test]
    fn test_optimal_bead_count() {
        // Test optimal bead count calculation
        let strategy = DistributedBeadingStrategy::new(
            scaled_f64(0.4),
            scaled_f64(0.3),
            std::f64::consts::PI / 4.0,
            0.5,
            0.5,
            2,
        );

        let thickness = scaled_f64(1.2); // 3x optimal width
        let count = strategy.get_optimal_bead_count(thickness);
        assert_eq!(count, 3);
    }

    #[test]
    fn test_compute_single_bead() {
        // Test computation with single bead
        let strategy = DistributedBeadingStrategy::new(
            scaled_f64(0.4),
            scaled_f64(0.3),
            std::f64::consts::PI / 4.0,
            0.5,
            0.5,
            2,
        );

        let thickness = scaled_f64(0.4);
        let beading = strategy.compute(thickness, 1);

        assert_eq!(beading.bead_widths.len(), 1);
        assert_eq!(beading.bead_widths[0], thickness);
        assert_eq!(beading.left_over, 0);
    }

    #[test]
    fn test_compute_two_beads() {
        // Test computation with two beads
        let strategy = DistributedBeadingStrategy::new(
            scaled_f64(0.4),
            scaled_f64(0.3),
            std::f64::consts::PI / 4.0,
            0.5,
            0.5,
            2,
        );

        let thickness = scaled_f64(0.8);
        let beading = strategy.compute(thickness, 2);

        assert_eq!(beading.bead_widths.len(), 2);
        assert_eq!(beading.bead_widths[0], thickness / 2);
        assert_eq!(beading.bead_widths[1], thickness / 2);
        assert_eq!(beading.left_over, 0);
    }

    #[test]
    fn test_compute_multiple_beads() {
        // Test computation with multiple beads (uses weight distribution)
        let strategy = DistributedBeadingStrategy::new(
            scaled_f64(0.4),
            scaled_f64(0.3),
            std::f64::consts::PI / 4.0,
            0.5,
            0.5,
            2,
        );

        let thickness = scaled_f64(1.2);
        let beading = strategy.compute(thickness, 3);

        assert_eq!(beading.bead_widths.len(), 3);
        assert_eq!(beading.left_over, 0);
        // Total width should equal thickness
        let total: Coord = beading.bead_widths.iter().sum();
        assert_eq!(total, thickness);
    }
}
