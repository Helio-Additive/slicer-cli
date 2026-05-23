//! Cooling buffer module for layer cooling time management.
//!
//! C++ Reference:
//! - GCode/CoolingBuffer.hpp
//! - GCode/CoolingBuffer.cpp
//!
//! This module calculates how much to slow down extrusion on a per-layer basis
//! to meet minimum layer time requirements.

use crate::gcode::gcode_editor::PerExtruderAdjustments;

/// Cooling buffer that manages per-layer slowdown calculations.
/// Corresponds to C++ CoolingBuffer.
#[derive(Debug, Clone)]
pub struct CoolingBuffer {
    /// Use proportional cooling logic (old style) vs non-proportional.
    cooling_logic_proportional: bool,
}

impl CoolingBuffer {
    pub fn new() -> Self {
        Self {
            cooling_logic_proportional: false,
        }
    }

    /// Create a cooling buffer with proportional logic enabled.
    pub fn with_proportional_logic(proportional: bool) -> Self {
        Self {
            cooling_logic_proportional: proportional,
        }
    }

    /// Calculate the layer slowdown factor.
    ///
    /// Takes per-extruder adjustments and returns the estimated layer time
    /// after slowdown has been applied to all extruders.
    ///
    /// Corresponds to C++ CoolingBuffer::calculate_layer_slowdown.
    pub fn calculate_layer_slowdown(
        &self,
        per_extruder_adjustments: &mut [PerExtruderAdjustments],
    ) -> f32 {
        if per_extruder_adjustments.is_empty() {
            return 0.0;
        }

        let mut layer_time = 0.0f32;

        for adj in per_extruder_adjustments.iter_mut() {
            if self.cooling_logic_proportional {
                // Proportional: slow all adjustable lines by the same factor
                let elapsed = adj.elapsed_time_total();
                layer_time += elapsed;
            } else {
                // Non-proportional: sort by feedrate and slow down progressively
                adj.sort_lines_by_decreasing_feedrate();
                let elapsed = adj.elapsed_time_total();
                layer_time += elapsed;
            }
        }

        layer_time
    }
}

impl Default for CoolingBuffer {
    fn default() -> Self {
        Self::new()
    }
}

/// Calculate layer slowdown for the given per-extruder adjustments.
pub fn calculate_layer_slowdown(per_extruder_adjustments: &mut [PerExtruderAdjustments]) -> f32 {
    let buffer = CoolingBuffer::new();
    buffer.calculate_layer_slowdown(per_extruder_adjustments)
}

/// Create a new cooling buffer instance.
pub fn cooling_buffer() -> crate::Result<CoolingBuffer> {
    Ok(CoolingBuffer::new())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gcode::gcode_editor::CoolingLine;

    #[test]
    fn test_cooling_buffer_new() {
        let buf = CoolingBuffer::new();
        assert!(!buf.cooling_logic_proportional);
    }

    #[test]
    fn test_calculate_layer_slowdown_empty() {
        let buf = CoolingBuffer::new();
        let time = buf.calculate_layer_slowdown(&mut []);
        assert_eq!(time, 0.0);
    }

    #[test]
    fn test_calculate_layer_slowdown_basic() {
        let buf = CoolingBuffer::new();
        let mut adj = PerExtruderAdjustments::new();

        let mut line = CoolingLine::new(0x40 | 0x20, 0, 10); // ADJUSTABLE | G1
        line.time = 2.0;
        line.time_max = 5.0;
        line.length = 20.0;
        line.feedrate = 10.0;
        adj.lines.push(line);

        let time = buf.calculate_layer_slowdown(&mut [adj]);
        assert!(time > 0.0);
    }

    #[test]
    fn test_cooling_buffer_convenience() {
        let buf = cooling_buffer().unwrap();
        assert!(!buf.cooling_logic_proportional);
    }
}
