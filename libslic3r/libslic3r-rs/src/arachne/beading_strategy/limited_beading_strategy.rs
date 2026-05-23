//! Limited beading strategy implementation.
//!
//! C++ Reference:
//! - Arachne/BeadingStrategy/LimitedBeadingStrategy.hpp
//! - Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp
//!
//! This is a meta-strategy that can be applied on top of any other beading strategy,
//! which limits the thickness of the walls to the thickness that the lines can
//! reasonably print. The width of the wall is limited to the maximum number of
//! contours times the maximum width of each of these contours.

use super::beading_strategy::{
    Beading, BeadingStrategy, BeadingStrategyPtr, WALL_CONTOUR_MARKED_WIDTH,
};
use crate::{scale, Coord};

/// Limited beading strategy that caps the maximum number of beads.
/// Arachne/BeadingStrategy/LimitedBeadingStrategy.hpp:26-47
///
/// This is a meta-strategy that can be applied on top of any other beading strategy,
/// which limits the thickness of the walls to the thickness that the lines can reasonably print.
///
/// The width of the wall is limited to the maximum number of contours times the maximum
/// width of each of these contours.
///
/// If the width of the wall gets limited, this strategy outputs one additional bead with 0 width.
/// This bead is used to denote the limits of the walled area. Other structures can then use this
/// border to align their structures to, such as to create correctly overlapping infill or skin,
/// or to align the infill pattern to any extra infill walls.
#[derive(Clone)]
pub struct LimitedBeadingStrategy {
    /// Maximum number of beads allowed
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.hpp:45
    /// C++: const coord_t max_bead_count;
    max_bead_count: Coord,

    /// Parent beading strategy
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.hpp:46
    /// C++: const BeadingStrategyPtr parent;
    parent: BeadingStrategyPtr,

    /// Strategy name for debugging
    name: String,
}

impl LimitedBeadingStrategy {
    /// Create a new limited beading strategy
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:28-36
    ///
    /// # Arguments
    /// * `max_bead_count` - Maximum number of beads to generate
    /// * `parent` - Parent beading strategy to wrap
    ///
    /// C++: LimitedBeadingStrategy::LimitedBeadingStrategy(const coord_t max_bead_count, BeadingStrategyPtr parent)
    /// C++:     : BeadingStrategy(*parent)
    /// C++:     , max_bead_count(max_bead_count)
    /// C++:     , parent(std::move(parent))
    /// C++: {
    /// C++:     if (max_bead_count % 2 == 1)
    /// C++:     {
    /// C++:         BOOST_LOG_TRIVIAL(warning) << "LimitedBeadingStrategy with odd bead count is odd indeed!";
    /// C++:     }
    /// C++: }
    pub fn new(max_bead_count: Coord, parent: BeadingStrategyPtr) -> Self {
        if max_bead_count % 2 == 1 {
            log::warn!("LimitedBeadingStrategy with odd bead count is odd indeed!");
        }

        Self {
            max_bead_count,
            parent,
            name: "LimitedBeadingStrategy".to_string(),
        }
    }
}

impl BeadingStrategy for LimitedBeadingStrategy {
    /// Compute beading for a given thickness and bead count
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:38-95
    ///
    /// C++: LimitedBeadingStrategy::Beading LimitedBeadingStrategy::compute(coord_t thickness, coord_t bead_count) const
    fn compute(&self, thickness: Coord, bead_count: Coord) -> Beading {
        // If within limits, delegate to parent and possibly add marker bead
        // Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:40-54
        // C++: if (bead_count <= max_bead_count)
        // C++: {
        // C++:     Beading ret = parent->compute(thickness, bead_count);
        // C++:     bead_count = ret.toolpath_locations.size();
        // C++:
        // C++:     if (bead_count % 2 == 0 && bead_count == max_bead_count)
        // C++:     {
        // C++:         const coord_t innermost_toolpath_location = ret.toolpath_locations[max_bead_count / 2 - 1];
        // C++:         const coord_t innermost_toolpath_width = ret.bead_widths[max_bead_count / 2 - 1];
        // C++:         ret.toolpath_locations.insert(ret.toolpath_locations.begin() + max_bead_count / 2, innermost_toolpath_location + innermost_toolpath_width / 2);
        // C++:         ret.bead_widths.insert(ret.bead_widths.begin() + max_bead_count / 2, WallContourMarkedWidth);
        // C++:     }
        // C++:     return ret;
        // C++: }
        if bead_count <= self.max_bead_count {
            let mut ret = self.parent.compute(thickness, bead_count);
            let bead_count = ret.toolpath_locations.len() as Coord;

            if bead_count % 2 == 0 && bead_count == self.max_bead_count {
                let idx = (self.max_bead_count / 2 - 1) as usize;
                let innermost_toolpath_location = ret.toolpath_locations[idx];
                let innermost_toolpath_width = ret.bead_widths[idx];
                let insert_idx = (self.max_bead_count / 2) as usize;
                ret.toolpath_locations.insert(
                    insert_idx,
                    innermost_toolpath_location + innermost_toolpath_width / 2,
                );
                ret.bead_widths
                    .insert(insert_idx, WALL_CONTOUR_MARKED_WIDTH);
            }
            return ret;
        }

        // Assert that bead_count is at most max_bead_count + 1
        // Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:55-60
        // C++: assert(bead_count == max_bead_count + 1);
        // C++: if(bead_count != max_bead_count + 1)
        // C++: {
        // C++:     BOOST_LOG_TRIVIAL(warning) << "Too many beads! " << bead_count << " != " << max_bead_count + 1;
        // C++: }
        debug_assert_eq!(bead_count, self.max_bead_count + 1);
        if bead_count != self.max_bead_count + 1 {
            log::warn!(
                "Too many beads! {} != {}",
                bead_count,
                self.max_bead_count + 1
            );
        }

        // Compute at optimal thickness for max_bead_count
        // Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:62-66
        // C++: coord_t optimal_thickness = parent->getOptimalThickness(max_bead_count);
        // C++: Beading ret = parent->compute(optimal_thickness, max_bead_count);
        // C++: bead_count = ret.toolpath_locations.size();
        // C++: ret.left_over += thickness - ret.total_thickness;
        // C++: ret.total_thickness = thickness;
        let optimal_thickness = self.parent.get_optimal_thickness(self.max_bead_count);
        let mut ret = self.parent.compute(optimal_thickness, self.max_bead_count);
        let bead_count = ret.toolpath_locations.len() as Coord;
        ret.left_over += thickness - ret.total_thickness;
        ret.total_thickness = thickness;

        // Enforce symmetry
        // Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:68-73
        // C++: // Enforce symmetry
        // C++: if (bead_count % 2 == 1) {
        // C++:     ret.toolpath_locations[bead_count / 2] = thickness / 2;
        // C++:     ret.bead_widths[bead_count / 2] = thickness - optimal_thickness;
        // C++: }
        // C++: for (coord_t bead_idx = 0; bead_idx < (bead_count + 1) / 2; bead_idx++)
        // C++:     ret.toolpath_locations[bead_count - 1 - bead_idx] = thickness - ret.toolpath_locations[bead_idx];
        if bead_count % 2 == 1 {
            let mid_idx = (bead_count / 2) as usize;
            ret.toolpath_locations[mid_idx] = thickness / 2;
            ret.bead_widths[mid_idx] = thickness - optimal_thickness;
        }
        for bead_idx in 0..((bead_count + 1) / 2) {
            let idx = bead_idx as usize;
            let opposite_idx = (bead_count - 1 - bead_idx) as usize;
            ret.toolpath_locations[opposite_idx] = thickness - ret.toolpath_locations[idx];
        }

        // Create fake inner walls with 0 width to denote boundaries
        // Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:75-88
        // C++: //Create a "fake" inner wall with 0 width to indicate the edge of the walled area.
        // C++: //This wall can then be used by other structures to e.g. fill the infill area adjacent to the variable-width walls.
        // C++: coord_t innermost_toolpath_location = ret.toolpath_locations[max_bead_count / 2 - 1];
        // C++: coord_t innermost_toolpath_width = ret.bead_widths[max_bead_count / 2 - 1];
        // C++: ret.toolpath_locations.insert(ret.toolpath_locations.begin() + max_bead_count / 2, innermost_toolpath_location + innermost_toolpath_width / 2);
        // C++: ret.bead_widths.insert(ret.bead_widths.begin() + max_bead_count / 2, WallContourMarkedWidth);
        // C++:
        // C++: //Symmetry on both sides. Symmetry is guaranteed since this code is stopped early if the bead_count <= max_bead_count, and never reaches this point then.
        // C++: const size_t opposite_bead = bead_count - (max_bead_count / 2 - 1);
        // C++: innermost_toolpath_location = ret.toolpath_locations[opposite_bead];
        // C++: innermost_toolpath_width = ret.bead_widths[opposite_bead];
        // C++: ret.toolpath_locations.insert(ret.toolpath_locations.begin() + opposite_bead, innermost_toolpath_location - innermost_toolpath_width / 2);
        // C++: ret.bead_widths.insert(ret.bead_widths.begin() + opposite_bead, WallContourMarkedWidth);
        let idx = (self.max_bead_count / 2 - 1) as usize;
        let innermost_toolpath_location = ret.toolpath_locations[idx];
        let innermost_toolpath_width = ret.bead_widths[idx];
        let insert_idx = (self.max_bead_count / 2) as usize;
        ret.toolpath_locations.insert(
            insert_idx,
            innermost_toolpath_location + innermost_toolpath_width / 2,
        );
        ret.bead_widths
            .insert(insert_idx, WALL_CONTOUR_MARKED_WIDTH);

        // Symmetry on both sides
        let opposite_bead = (bead_count - (self.max_bead_count / 2 - 1)) as usize;
        let innermost_toolpath_location = ret.toolpath_locations[opposite_bead];
        let innermost_toolpath_width = ret.bead_widths[opposite_bead];
        ret.toolpath_locations.insert(
            opposite_bead,
            innermost_toolpath_location - innermost_toolpath_width / 2,
        );
        ret.bead_widths
            .insert(opposite_bead, WALL_CONTOUR_MARKED_WIDTH);

        ret
    }

    /// Get the optimal bead count for a given thickness
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:114-125
    ///
    /// C++: coord_t LimitedBeadingStrategy::getOptimalBeadCount(coord_t thickness) const
    /// C++: {
    /// C++:     coord_t parent_bead_count = parent->getOptimalBeadCount(thickness);
    /// C++:     if (parent_bead_count <= max_bead_count) {
    /// C++:         return parent->getOptimalBeadCount(thickness);
    /// C++:     } else if (parent_bead_count == max_bead_count + 1) {
    /// C++:         if (thickness < parent->getOptimalThickness(max_bead_count + 1) - scaled<coord_t>(0.01))
    /// C++:             return max_bead_count;
    /// C++:         else
    /// C++:             return max_bead_count + 1;
    /// C++:     }
    /// C++:     else return max_bead_count + 1;
    /// C++: }
    fn get_optimal_bead_count(&self, thickness: Coord) -> Coord {
        let parent_bead_count = self.parent.get_optimal_bead_count(thickness);
        if parent_bead_count <= self.max_bead_count {
            self.parent.get_optimal_bead_count(thickness)
        } else if parent_bead_count == self.max_bead_count + 1 {
            if thickness < self.parent.get_optimal_thickness(self.max_bead_count + 1) - scale(0.01)
            {
                self.max_bead_count
            } else {
                self.max_bead_count + 1
            }
        } else {
            self.max_bead_count + 1
        }
    }

    /// Get the strategy name
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:13-16
    ///
    /// C++: std::string LimitedBeadingStrategy::toString() const
    /// C++: {
    /// C++:     return std::string("LimitedBeadingStrategy+") + parent->toString();
    /// C++: }
    fn name(&self) -> &str {
        &self.name
    }

    /// Get the optimal thickness for a given bead count
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:97-102
    ///
    /// C++: coord_t LimitedBeadingStrategy::getOptimalThickness(coord_t bead_count) const
    /// C++: {
    /// C++:     if (bead_count <= max_bead_count)
    /// C++:         return parent->getOptimalThickness(bead_count);
    /// C++:     assert(false);
    /// C++:     return scaled<coord_t>(1000.); // 1 meter (Cura was returning 10 meter)
    /// C++: }
    fn get_optimal_thickness(&self, bead_count: Coord) -> Coord {
        if bead_count <= self.max_bead_count {
            self.parent.get_optimal_thickness(bead_count)
        } else {
            debug_assert!(false, "bead_count > max_bead_count");
            scale(1000.0) // 1 meter
        }
    }

    /// Get the transition thickness for a given lower bead count
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:104-112
    ///
    /// C++: coord_t LimitedBeadingStrategy::getTransitionThickness(coord_t lower_bead_count) const
    /// C++: {
    /// C++:     if (lower_bead_count < max_bead_count)
    /// C++:         return parent->getTransitionThickness(lower_bead_count);
    /// C++:
    /// C++:     if (lower_bead_count == max_bead_count)
    /// C++:         return parent->getOptimalThickness(lower_bead_count + 1) - scaled<coord_t>(0.01);
    /// C++:
    /// C++:     assert(false);
    /// C++:     return scaled<coord_t>(900.); // 0.9 meter;
    /// C++: }
    fn get_transition_thickness(&self, lower_bead_count: Coord) -> Coord {
        if lower_bead_count < self.max_bead_count {
            self.parent.get_transition_thickness(lower_bead_count)
        } else if lower_bead_count == self.max_bead_count {
            self.parent.get_optimal_thickness(lower_bead_count + 1) - scale(0.01)
        } else {
            debug_assert!(false, "lower_bead_count > max_bead_count");
            scale(900.0) // 0.9 meter
        }
    }

    /// Get the transitioning length
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:18-21
    ///
    /// C++: coord_t LimitedBeadingStrategy::getTransitioningLength(coord_t lower_bead_count) const
    /// C++: {
    /// C++:     return parent->getTransitioningLength(lower_bead_count);
    /// C++: }
    fn get_transitioning_length(&self, lower_bead_count: Coord) -> Coord {
        self.parent.get_transitioning_length(lower_bead_count)
    }

    /// Get the transition anchor position
    /// Arachne/BeadingStrategy/LimitedBeadingStrategy.cpp:23-26
    ///
    /// C++: float LimitedBeadingStrategy::getTransitionAnchorPos(coord_t lower_bead_count) const
    /// C++: {
    /// C++:     return parent->getTransitionAnchorPos(lower_bead_count);
    /// C++: }
    fn get_transition_anchor_pos(&self, lower_bead_count: Coord) -> f32 {
        self.parent.get_transition_anchor_pos(lower_bead_count)
    }

    /// Get the optimal width (delegates to parent)
    fn optimal_width(&self) -> Coord {
        self.parent.optimal_width()
    }

    /// Get the default transition length (delegates to parent)
    fn default_transition_length(&self) -> Coord {
        self.parent.default_transition_length()
    }

    /// Get the transitioning angle (delegates to parent)
    fn transitioning_angle(&self) -> f64 {
        self.parent.get_transitioning_angle()
    }

    /// Get the wall split middle threshold (delegates to parent)
    fn wall_split_middle_threshold(&self) -> f64 {
        self.parent.get_split_middle_threshold()
    }

    /// Get the wall add middle threshold (delegates to parent)
    fn wall_add_middle_threshold(&self) -> f64 {
        self.parent.wall_add_middle_threshold()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arachne::beading_strategy::{make_strategy, DistributedBeadingStrategy};

    /// Helper to create a parent strategy for testing
    fn create_test_parent() -> BeadingStrategyPtr {
        make_strategy(DistributedBeadingStrategy::new(
            400_000,                    // optimal_width (0.4mm)
            100_000,                    // default_transition_length
            std::f64::consts::PI / 3.0, // transitioning_angle
            0.4,                        // wall_split_middle_threshold
            0.7,                        // wall_add_middle_threshold
            2,                          // distribution_radius
        ))
    }

    #[test]
    fn test_limited_strategy_creation() {
        let parent = create_test_parent();
        let strategy = LimitedBeadingStrategy::new(
            4, // max_bead_count
            parent,
        );

        assert_eq!(strategy.name(), "LimitedBeadingStrategy");
        assert_eq!(strategy.max_bead_count, 4);
    }

    #[test]
    fn test_compute_within_limit() {
        let parent = create_test_parent();
        let strategy = LimitedBeadingStrategy::new(4, parent);

        // Request 2 beads, which is within the limit of 4
        let beading = strategy.compute(800_000, 2);
        assert_eq!(beading.total_thickness, 800_000);
        assert_eq!(beading.bead_widths.len(), 2);
    }

    #[test]
    fn test_compute_at_even_limit() {
        let parent = create_test_parent();
        let strategy = LimitedBeadingStrategy::new(4, parent);

        // Request exactly max_bead_count (even number)
        let beading = strategy.compute(1_600_000, 4);
        assert_eq!(beading.total_thickness, 1_600_000);
        // Should have marker bead inserted
        assert_eq!(beading.bead_widths.len(), 5); // 4 + 1 marker
    }

    #[test]
    fn test_optimal_bead_count_within_limit() {
        let parent = create_test_parent();
        let strategy = LimitedBeadingStrategy::new(4, parent);

        let count = strategy.get_optimal_bead_count(800_000);
        assert!(count <= 4);
    }

    #[test]
    fn test_optimal_bead_count_exceeds_limit() {
        let parent = create_test_parent();
        let strategy = LimitedBeadingStrategy::new(2, parent);

        // Request thickness that would normally give more beads
        let count = strategy.get_optimal_bead_count(2_000_000);
        // Should be capped at max + 1
        assert!(count <= 3);
    }

    #[test]
    fn test_transition_thickness_at_limit() {
        let parent = create_test_parent();
        let strategy = LimitedBeadingStrategy::new(4, parent);

        let thickness = strategy.get_transition_thickness(4);
        // Should return special value near optimal thickness
        assert!(thickness > 0);
    }
}
