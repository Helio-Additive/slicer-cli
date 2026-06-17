//! OuterWallContourStrategy - wrapper strategy for outer wall contour generation
//!
//! C++ Reference:
//! - Arachne/BeadingStrategy/OuterWallContourStrategy.hpp
//! - Arachne/BeadingStrategy/OuterWallContourStrategy.cpp
//!
//! Faithful 1:1 line-by-line port.

use crate::arachne::beading_strategy::beading_strategy::{
    Beading, BeadingStrategy, BeadingStrategyPtr, FIRST_WALL_CONTOUR_MARKED_WIDTH,
};
use crate::geometry::Coord;
use std::sync::Arc;

/// Wrapper strategy that adds outer wall contour beads around a parent strategy
/// OuterWallContourStrategy.hpp:8-26
#[derive(Clone)]
pub struct OuterWallContourStrategy {
    /// The parent strategy being wrapped
    /// OuterWallContourStrategy.hpp:24
    parent: BeadingStrategyPtr,

    /// Base strategy fields (name, default_transition_length)
    name: String,
    default_transition_length: Coord,
}

impl OuterWallContourStrategy {
    /// Create a new OuterWallContourStrategy wrapping a parent strategy
    /// OuterWallContourStrategy.cpp:9-13
    // C++: OuterWallContourStrategy::OuterWallContourStrategy(BeadingStrategyPtr parent)
    // C++:     : BeadingStrategy(*parent)
    // C++:     , parent(std::move(parent))
    // C++: {
    // C++: }
    //
    // The C++ `: BeadingStrategy(*parent)` copy-constructs the base sub-object from
    // the parent, so every base member (optimal_width, default_transition_length,
    // transitioning_angle, wall_split/add_middle_threshold, name) becomes the
    // parent's value. We cache `name` and `default_transition_length`, and delegate
    // the remaining base getters to `parent` directly (equivalent to the copy).
    pub fn new(parent: BeadingStrategyPtr) -> Self {
        let default_transition_length = parent.default_transition_length();
        let name = parent.name().to_string();

        Self {
            parent,
            name,
            default_transition_length,
        }
    }
}

impl BeadingStrategy for OuterWallContourStrategy {
    /// Compute beading for given thickness and bead count
    /// OuterWallContourStrategy.cpp:62-82
    // C++: BeadingStrategy::Beading OuterWallContourStrategy::compute(coord_t thickness, coord_t bead_count) const
    fn compute(&self, thickness: Coord, bead_count: Coord) -> Beading {
        // OuterWallContourStrategy.cpp:64
        // C++:     if (bead_count <= 1)
        if bead_count <= 1 {
            // OuterWallContourStrategy.cpp:65
            // C++:         return parent->compute(thickness, bead_count);
            return self.parent.compute(thickness, bead_count);
        }

        // OuterWallContourStrategy.cpp:67
        // C++:     assert(bead_count >= 3);
        // C++ `assert` is compiled out under NDEBUG (release); mirror with debug_assert!.
        debug_assert!(bead_count >= 3);
        // OuterWallContourStrategy.cpp:68
        // C++:     Beading ret = parent->compute(thickness, bead_count - 2);
        let mut ret = self.parent.compute(thickness, bead_count - 2);
        // OuterWallContourStrategy.cpp:69
        // C++:     if(ret.toolpath_locations.size() == 1){
        if ret.toolpath_locations.len() == 1 {
            // OuterWallContourStrategy.cpp:70
            // C++:         return ret;
            return ret;
        }
        // OuterWallContourStrategy.cpp:72
        // C++:     if(ret.toolpath_locations.size() > 0 ){
        if !ret.toolpath_locations.is_empty() {
            // OuterWallContourStrategy.cpp:73
            // C++:         assert(ret.bead_widths.size()>0);
            // C++ `assert` is compiled out under NDEBUG (release); mirror with debug_assert!.
            debug_assert!(!ret.bead_widths.is_empty());
            // OuterWallContourStrategy.cpp:74
            // C++:         double location = ret.toolpath_locations.front() + ret.bead_widths.front() / 2;
            // Note: `location`/`location_reverse` are computed from the original
            // front()/back() values before any insertion mutates the vectors.
            let location = ret.toolpath_locations[0] + ret.bead_widths[0] / 2;
            // OuterWallContourStrategy.cpp:75
            // C++:         double location_reverse = ret.toolpath_locations.back() - ret.bead_widths.back() / 2;
            let location_reverse =
                *ret.toolpath_locations.last().unwrap() - *ret.bead_widths.last().unwrap() / 2;
            // OuterWallContourStrategy.cpp:76
            // C++:         ret.toolpath_locations.insert(ret.toolpath_locations.begin()+1, location);
            // begin()+1 -> index 1.
            ret.toolpath_locations.insert(1, location);
            // OuterWallContourStrategy.cpp:77
            // C++:         ret.bead_widths.insert(ret.bead_widths.begin()+1, FirstWallContourMarkedWidth);
            // begin()+1 -> index 1.
            ret.bead_widths.insert(1, FIRST_WALL_CONTOUR_MARKED_WIDTH);
            // OuterWallContourStrategy.cpp:78
            // C++:         ret.toolpath_locations.insert((ret.toolpath_locations.rbegin()+1).base(), location_reverse);
            // (rbegin()+1).base() == end()-1, i.e. insert just before the last element.
            let insert_pos = ret.toolpath_locations.len() - 1;
            ret.toolpath_locations.insert(insert_pos, location_reverse);
            // OuterWallContourStrategy.cpp:79
            // C++:         ret.bead_widths.insert((ret.bead_widths.rbegin()).base(), FirstWallContourMarkedWidth);
            // rbegin().base() == end(), i.e. append at the very end (note: NO +1, unlike line 78).
            ret.bead_widths.push(FIRST_WALL_CONTOUR_MARKED_WIDTH);
        }
        // OuterWallContourStrategy.cpp:81
        // C++:     return ret;
        ret
    }

    /// Get optimal thickness for given bead count
    /// OuterWallContourStrategy.cpp:55-60
    // C++: coord_t OuterWallContourStrategy::getOptimalThickness(coord_t bead_count) const
    fn get_optimal_thickness(&self, bead_count: Coord) -> Coord {
        // OuterWallContourStrategy.cpp:57
        // C++:     if (bead_count <= 1)
        if bead_count <= 1 {
            // OuterWallContourStrategy.cpp:58
            // C++:         return parent->getOptimalThickness(bead_count);
            return self.parent.get_optimal_thickness(bead_count);
        }
        // OuterWallContourStrategy.cpp:59
        // C++:     return parent->getOptimalThickness(bead_count - 2) + 2;
        self.parent.get_optimal_thickness(bead_count - 2) + 2
    }

    /// Get transition thickness for given lower bead count
    /// OuterWallContourStrategy.cpp:36-43
    // C++: coord_t OuterWallContourStrategy::getTransitionThickness(coord_t lower_bead_count) const
    fn get_transition_thickness(&self, lower_bead_count: Coord) -> Coord {
        // OuterWallContourStrategy.cpp:38
        // C++:     if(lower_bead_count <= 1)
        if lower_bead_count <= 1 {
            // OuterWallContourStrategy.cpp:39
            // C++:         return parent->getTransitionThickness(lower_bead_count);
            self.parent.get_transition_thickness(lower_bead_count)
        } else if lower_bead_count == 2 || lower_bead_count == 3 {
            // OuterWallContourStrategy.cpp:40
            // C++:     else if(lower_bead_count == 2 || lower_bead_count ==3)
            // OuterWallContourStrategy.cpp:41
            // C++:         return parent->getTransitionThickness(1);
            self.parent.get_transition_thickness(1)
        } else {
            // OuterWallContourStrategy.cpp:42
            // C++:     return parent->getTransitionThickness(lower_bead_count-2);
            self.parent.get_transition_thickness(lower_bead_count - 2)
        }
    }

    /// Get optimal bead count for given thickness
    /// OuterWallContourStrategy.cpp:46-52
    // C++: coord_t OuterWallContourStrategy::getOptimalBeadCount(coord_t thickness) const
    fn get_optimal_bead_count(&self, thickness: Coord) -> Coord {
        // OuterWallContourStrategy.cpp:48
        // C++:     coord_t parent_bead_count = parent->getOptimalBeadCount(thickness);
        let parent_bead_count = self.parent.get_optimal_bead_count(thickness);
        // OuterWallContourStrategy.cpp:49
        // C++:     if(parent_bead_count <= 1)
        if parent_bead_count <= 1 {
            // OuterWallContourStrategy.cpp:50
            // C++:         return parent_bead_count;
            return parent_bead_count;
        }
        // OuterWallContourStrategy.cpp:51
        // C++:     return parent_bead_count + 2;
        parent_bead_count + 2
    }

    /// Get transitioning length for given lower bead count
    /// OuterWallContourStrategy.cpp:20-23
    // C++: coord_t OuterWallContourStrategy::getTransitioningLength(coord_t lower_bead_count) const
    fn get_transitioning_length(&self, lower_bead_count: Coord) -> Coord {
        // OuterWallContourStrategy.cpp:22
        // C++:     return parent->getTransitioningLength(lower_bead_count);
        self.parent.get_transitioning_length(lower_bead_count)
    }

    /// Get transition anchor position
    /// OuterWallContourStrategy.cpp:25-28
    // C++: float OuterWallContourStrategy::getTransitionAnchorPos(coord_t lower_bead_count) const
    fn get_transition_anchor_pos(&self, lower_bead_count: Coord) -> f32 {
        // OuterWallContourStrategy.cpp:27
        // C++:     return parent->getTransitionAnchorPos(lower_bead_count);
        self.parent.get_transition_anchor_pos(lower_bead_count)
    }

    /// Get nonlinear thicknesses for given lower bead count
    /// OuterWallContourStrategy.cpp:30-33
    // C++: std::vector<coord_t> OuterWallContourStrategy::getNonlinearThicknesses(coord_t lower_bead_count) const
    fn get_nonlinear_thicknesses(&self, lower_bead_count: Coord) -> Vec<Coord> {
        // OuterWallContourStrategy.cpp:32
        // C++:     return parent->getNonlinearThicknesses(lower_bead_count);
        self.parent.get_nonlinear_thicknesses(lower_bead_count)
    }

    /// Get the strategy name
    /// OuterWallContourStrategy.cpp:15-18
    // C++: std::string OuterWallContourStrategy::toString() const
    // C++: {
    // C++:     return std::string("OuterWallContourStrategy+") + parent->toString();
    // C++: }
    //
    // Note: unlike the sibling meta-strategies (Limited/OuterWallInset/...), this
    // strategy never assigns its own base-class `name`; its constructor copies the
    // parent (`BeadingStrategy(*parent)`), so the inherited `name` member is the
    // parent's name. We mirror that here by initialising `self.name` from the
    // parent in `new()`. The C++ `toString()` additionally prepends
    // `"OuterWallContourStrategy+"`, which cannot be expressed through a `&str`
    // return; like the other meta-strategies in this crate it is omitted.
    fn name(&self) -> &str {
        &self.name
    }

    /// Get the default transition length
    fn default_transition_length(&self) -> Coord {
        self.default_transition_length
    }

    /// Get the optimal width (delegate to parent)
    fn optimal_width(&self) -> Coord {
        self.parent.optimal_width()
    }

    /// Get the transitioning angle (delegate to parent)
    fn transitioning_angle(&self) -> f64 {
        self.parent.transitioning_angle()
    }

    /// Get the wall split middle threshold (delegate to parent)
    fn wall_split_middle_threshold(&self) -> f64 {
        self.parent.wall_split_middle_threshold()
    }

    /// Get the wall add middle threshold (delegate to parent)
    fn wall_add_middle_threshold(&self) -> f64 {
        self.parent.wall_add_middle_threshold()
    }
}

/// Helper function to create Arc-wrapped OuterWallContourStrategy
pub fn outer_wall_contour_strategy(parent: BeadingStrategyPtr) -> BeadingStrategyPtr {
    Arc::new(OuterWallContourStrategy::new(parent))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arachne::beading_strategy::beading_strategy::BeadingStrategy;
    use crate::arachne::beading_strategy::distributed_beading_strategy::DistributedBeadingStrategy;

    fn create_test_parent() -> BeadingStrategyPtr {
        Arc::new(DistributedBeadingStrategy::new(
            400,  // preferred_bead_width
            1.0,  // transitioning_angle
            0.4,  // wall_split_middle_threshold
            0.5,  // wall_add_middle_threshold
            1000, // default_transition_length
        ))
    }

    #[test]
    fn test_outer_wall_contour_strategy_creation() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent);
        assert!(strategy.name().contains("Distributed"));
    }

    #[test]
    fn test_bead_count_adjustment() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        // Single bead - should pass through to parent
        let count1 = strategy.get_optimal_bead_count(400);
        let parent_count1 = parent.get_optimal_bead_count(400);
        assert_eq!(count1, parent_count1);

        // Multiple beads - should add 2
        let count2 = strategy.get_optimal_bead_count(1200);
        let parent_count2 = parent.get_optimal_bead_count(1200);
        assert_eq!(count2, parent_count2 + 2);
    }

    #[test]
    fn test_optimal_thickness_adjustment() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        // Single bead - pass through
        let thickness1 = strategy.get_optimal_thickness(1);
        let parent_thickness1 = parent.get_optimal_thickness(1);
        assert_eq!(thickness1, parent_thickness1);

        // Multiple beads - adjust by parent(count-2) + 2
        let thickness3 = strategy.get_optimal_thickness(3);
        let parent_thickness1_plus2 = parent.get_optimal_thickness(1) + 2;
        assert_eq!(thickness3, parent_thickness1_plus2);
    }

    #[test]
    fn test_compute_single_bead() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        let beading = strategy.compute(400, 1);
        let parent_beading = parent.compute(400, 1);

        // Should be identical to parent for single bead
        assert_eq!(
            beading.toolpath_locations.len(),
            parent_beading.toolpath_locations.len()
        );
        assert_eq!(beading.bead_widths.len(), parent_beading.bead_widths.len());
    }

    #[test]
    fn test_compute_adds_contour_beads() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        // Request 3 beads (parent will compute 1 bead)
        let beading = strategy.compute(1200, 3);
        let parent_beading = parent.compute(1200, 1);

        // Parent with 1 bead returns 1 toolpath - strategy should keep it as-is
        if parent_beading.toolpath_locations.len() == 1 {
            assert_eq!(beading.toolpath_locations.len(), 1);
        }

        // Request more beads to test contour insertion
        let beading5 = strategy.compute(2000, 5);
        let parent_beading3 = parent.compute(2000, 3);

        // Should add 2 contour beads (one on each side)
        if parent_beading3.toolpath_locations.len() > 1 {
            assert_eq!(
                beading5.toolpath_locations.len(),
                parent_beading3.toolpath_locations.len() + 2
            );
            assert_eq!(
                beading5.bead_widths.len(),
                parent_beading3.bead_widths.len() + 2
            );

            // Check that contour beads have marked width
            assert!(beading5
                .bead_widths
                .contains(&FIRST_WALL_CONTOUR_MARKED_WIDTH));
        }
    }

    #[test]
    fn test_transition_thickness() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        // Lower bead count <= 1: pass through
        let t1 = strategy.get_transition_thickness(1);
        assert_eq!(t1, parent.get_transition_thickness(1));

        // Lower bead count 2 or 3: use parent's transition for 1
        let t2 = strategy.get_transition_thickness(2);
        assert_eq!(t2, parent.get_transition_thickness(1));

        let t3 = strategy.get_transition_thickness(3);
        assert_eq!(t3, parent.get_transition_thickness(1));

        // Higher counts: subtract 2
        let t5 = strategy.get_transition_thickness(5);
        assert_eq!(t5, parent.get_transition_thickness(3));
    }

    #[test]
    fn test_transitioning_length_passthrough() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        let length = strategy.get_transitioning_length(2);
        assert_eq!(length, parent.get_transitioning_length(2));
    }

    #[test]
    fn test_transition_anchor_pos_passthrough() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        let pos = strategy.get_transition_anchor_pos(2);
        assert_eq!(pos, parent.get_transition_anchor_pos(2));
    }

    #[test]
    fn test_nonlinear_thicknesses_passthrough() {
        let parent = create_test_parent();
        let strategy = OuterWallContourStrategy::new(parent.clone());

        let thicknesses = strategy.get_nonlinear_thicknesses(2);
        assert_eq!(thicknesses, parent.get_nonlinear_thicknesses(2));
    }
}
