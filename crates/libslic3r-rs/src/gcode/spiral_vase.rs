//! Faithful 1:1 port of BambuStudio `src/libslic3r/GCode/SpiralVase.cpp`
//! (+ `SpiralVase.hpp`).
//!
//! This post-processor turns ordinary layer G-code into a continuous spiral
//! ("vase mode"): it ramps Z smoothly across each layer and — when smooth
//! spiral is enabled — interpolates X/Y with the previous layer so there is no
//! visible seam at layer changes.

// SpiralVase.cpp:1: #include "SpiralVase.hpp"
// SpiralVase.cpp:2: #include "GCode.hpp"
// SpiralVase.cpp:3: #include <sstream>
// SpiralVase.cpp:4: #include <cmath>
// SpiralVase.cpp:5: #include <limits>

use std::cell::RefCell;
use std::rc::Rc;

use crate::g_code_reader::{Axis, GCodeReader};
use crate::print_config::{GCodeConfig, PrintConfig};

// SpiralVase.cpp:7: namespace Slic3r {

// ---------------------------------------------------------------------------
// SpiralVase.hpp:9-50 — class SpiralVase
// ---------------------------------------------------------------------------

/// SpiralVase.hpp:12-19
/// class SpiralPoint {
/// public:
///     SpiralPoint(float paramx, float paramy) : x(paramx), y(paramy) {}
/// public:
///     float x, y;
/// };
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpiralPoint {
    /// SpiralVase.hpp:18: float x, y;
    pub x: f32,
    pub y: f32,
}

impl SpiralPoint {
    /// SpiralVase.hpp:15: SpiralPoint(float paramx, float paramy) : x(paramx), y(paramy) {}
    pub fn new(paramx: f32, paramy: f32) -> Self {
        Self { x: paramx, y: paramy }
    }
}

// SpiralVase.cpp:9: namespace SpiralVaseHelpers {
/// == Smooth Spiral Helpers ==
pub mod spiral_vase_helpers {
    use super::SpiralPoint;

    /// SpiralVase.cpp:11-14
    /// Distance between a and b
    /// float distance(SpiralVase::SpiralPoint a, SpiralVase::SpiralPoint b) {
    ///     return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    /// }
    pub fn distance(a: SpiralPoint, b: SpiralPoint) -> f32 {
        ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt()
    }

    /// SpiralVase.cpp:16-19
    /// SpiralVase::SpiralPoint subtract(SpiralVase::SpiralPoint a, SpiralVase::SpiralPoint b)
    /// {
    ///     return SpiralVase::SpiralPoint(a.x - b.x, a.y - b.y);
    /// }
    pub fn subtract(a: SpiralPoint, b: SpiralPoint) -> SpiralPoint {
        SpiralPoint::new(a.x - b.x, a.y - b.y)
    }

    /// SpiralVase.cpp:21-23
    /// SpiralVase::SpiralPoint add(SpiralVase::SpiralPoint a, SpiralVase::SpiralPoint b) {
    ///     return SpiralVase::SpiralPoint(a.x + b.x, a.y + b.y);
    /// }
    pub fn add(a: SpiralPoint, b: SpiralPoint) -> SpiralPoint {
        SpiralPoint::new(a.x + b.x, a.y + b.y)
    }

    /// SpiralVase.cpp:25-27
    /// SpiralVase::SpiralPoint scale(SpiralVase::SpiralPoint a, float factor) {
    ///     return SpiralVase::SpiralPoint(a.x * factor, a.y * factor);
    /// }
    pub fn scale(a: SpiralPoint, factor: f32) -> SpiralPoint {
        SpiralPoint::new(a.x * factor, a.y * factor)
    }

    /// SpiralVase.cpp:29-32
    /// dot product
    /// float dot(SpiralVase::SpiralPoint a, SpiralVase::SpiralPoint b) {
    ///     return a.x * b.x + a.y * b.y;
    /// }
    pub fn dot(a: SpiralPoint, b: SpiralPoint) -> f32 {
        a.x * b.x + a.y * b.y
    }

    /// SpiralVase.cpp:34-45
    /// Find the point on line ab closes to point c
    /// SpiralVase::SpiralPoint nearest_point_on_line(SpiralVase::SpiralPoint c, SpiralVase::SpiralPoint a, SpiralVase::SpiralPoint b, float& dist)
    pub fn nearest_point_on_line(
        c: SpiralPoint,
        a: SpiralPoint,
        b: SpiralPoint,
        dist: &mut f32,
    ) -> SpiralPoint {
        // SpiralVase.cpp:37: SpiralVase::SpiralPoint ab = subtract(b, a);
        let ab = subtract(b, a);
        // SpiralVase.cpp:38: SpiralVase::SpiralPoint ac = subtract(c, a);
        let ac = subtract(c, a);
        // SpiralVase.cpp:39: float t = dot(ac, ab) / dot(ab, ab);
        let mut t = dot(ac, ab) / dot(ab, ab);
        // SpiralVase.cpp:40: t = t > 1 ? 1 : t;
        t = if t > 1.0 { 1.0 } else { t };
        // SpiralVase.cpp:41: t = t < 0 ? 0 : t;
        t = if t < 0.0 { 0.0 } else { t };
        // SpiralVase.cpp:42: SpiralVase::SpiralPoint closest = SpiralVase::SpiralPoint(add(a, scale(ab, t)));
        let closest = add(a, scale(ab, t));
        // SpiralVase.cpp:43: dist = distance(c, closest);
        *dist = distance(c, closest);
        // SpiralVase.cpp:44: return closest;
        closest
    }

    /// SpiralVase.cpp:47-71
    /// Given a set of lines defined by points such as line[n] is the line from points[n] to points[n+1],
    /// find the closest point to p that falls on any of the lines
    /// SpiralVase::SpiralPoint nearest_point_on_lines(SpiralVase::SpiralPoint p,
    ///                                                std::shared_ptr<std::vector<SpiralVase::SpiralPoint>> points,
    ///                                                bool& found, float& dist)
    pub fn nearest_point_on_lines(
        p: SpiralPoint,
        points: &[SpiralPoint],
        found: &mut bool,
        dist: &mut f32,
    ) -> SpiralPoint {
        // SpiralVase.cpp:54-57
        // if (points->size() < 2) {
        //     found = false;
        //     return SpiralVase::SpiralPoint(0, 0);
        // }
        if points.len() < 2 {
            *found = false;
            return SpiralPoint::new(0.0, 0.0);
        }
        // SpiralVase.cpp:58: float min = std::numeric_limits<float>::max();
        let mut min = f32::MAX;
        // SpiralVase.cpp:59: SpiralVase::SpiralPoint closest(0, 0);
        let mut closest = SpiralPoint::new(0.0, 0.0);
        // SpiralVase.cpp:60: for (unsigned long i = 0; i < points->size() - 1; i++) {
        for i in 0..points.len() - 1 {
            // SpiralVase.cpp:61: float currentDist = 0;
            let mut current_dist = 0.0_f32;
            // SpiralVase.cpp:62: SpiralVase::SpiralPoint current = nearest_point_on_line(p, points->at(i), points->at(i + 1), currentDist);
            let current = nearest_point_on_line(p, points[i], points[i + 1], &mut current_dist);
            // SpiralVase.cpp:63: if (currentDist < min) {
            if current_dist < min {
                // SpiralVase.cpp:64: min = currentDist;
                min = current_dist;
                // SpiralVase.cpp:65: closest = current;
                closest = current;
                // SpiralVase.cpp:66: found = true;
                *found = true;
            }
        }
        // SpiralVase.cpp:69: dist = min;
        *dist = min;
        // SpiralVase.cpp:70: return closest;
        closest
    }
} // SpiralVase.cpp:72: } // namespace SpiralVase

/// SpiralVase.hpp:9-50 — class SpiralVase
#[derive(Debug)]
pub struct SpiralVase {
    /// SpiralVase.hpp:40: const PrintConfig &m_config;
    m_config: PrintConfig,
    /// SpiralVase.hpp:41: GCodeReader m_reader;
    m_reader: GCodeReader,
    /// SpiralVase.hpp:42: float m_max_xy_smoothing = 0.f;
    m_max_xy_smoothing: f32,

    /// SpiralVase.hpp:44: bool m_enabled = false;
    m_enabled: bool,
    /// SpiralVase.hpp:45-46
    /// First spiral vase layer. Layer height has to be ramped up from zero to the target layer height.
    /// bool m_transition_layer = false;
    m_transition_layer: bool,
    /// SpiralVase.hpp:47-48
    /// Whether to interpolate XY coordinates with the previous layer. Results in no seam at layer changes
    /// bool m_smooth_spiral = false;
    m_smooth_spiral: bool,
    /// SpiralVase.hpp:49: std::shared_ptr<std::vector<SpiralPoint>> m_previous_layer;
    m_previous_layer: Option<Rc<RefCell<Vec<SpiralPoint>>>>,
}

impl SpiralVase {
    /// SpiralVase.hpp:20-28
    /// SpiralVase(const PrintConfig &config) : m_config(config)
    /// {
    ///     //BBS
    ///     //m_reader.z() = (float)m_config.z_offset;
    ///     m_reader.z() = 0.0f;
    ///     m_reader.apply_config(m_config);
    ///     m_previous_layer = NULL;
    ///     m_smooth_spiral = config.spiral_mode_smooth;
    /// };
    pub fn new(config: &PrintConfig) -> Self {
        let mut m_reader = GCodeReader::new();
        // m_reader.z() = 0.0f;
        *m_reader.z_mut() = 0.0;
        // m_reader.apply_config(m_config);
        m_reader.apply_config(gcode_config_from_print_config(config));
        Self {
            m_config: config.clone(),
            m_reader,
            m_max_xy_smoothing: 0.0,
            m_enabled: false,
            m_transition_layer: false,
            // m_previous_layer = NULL;
            m_previous_layer: None,
            // m_smooth_spiral = config.spiral_mode_smooth;
            m_smooth_spiral: config.spiral_mode_smooth,
        }
    }

    /// SpiralVase.hpp:30-33
    /// void enable(bool en) {
    ///     m_transition_layer = en && ! m_enabled;
    ///     m_enabled          = en;
    /// }
    pub fn enable(&mut self, en: bool) {
        self.m_transition_layer = en && !self.m_enabled;
        self.m_enabled = en;
    }

    /// SpiralVase.hpp:36-38
    /// void set_max_xy_smoothing(float max) {
    ///     m_max_xy_smoothing = max;
    /// }
    pub fn set_max_xy_smoothing(&mut self, max: f32) {
        self.m_max_xy_smoothing = max;
    }

    /// SpiralVase.cpp:74-216
    /// std::string SpiralVase::process_layer(const std::string &gcode, bool last_layer)
    pub fn process_layer(&mut self, gcode: &str, last_layer: bool) -> String {
        // SpiralVase.cpp:76-82
        /*  This post-processor relies on several assumptions:
            - all layers are processed through it, including those that are not supposed
              to be transformed, in order to update the reader with the XY positions
            - each call to this method includes a full layer, with a single Z move
              at the beginning
            - each layer is composed by suitable geometry (i.e. a single complete loop)
            - loops were not clipped before calling this method  */

        // SpiralVase.cpp:84-89
        // If we're not going to modify G-code, just feed it to the reader
        // in order to update positions.
        if !self.m_enabled {
            self.m_reader.parse_buffer_noop(gcode);
            return gcode.to_string();
        }

        // SpiralVase.cpp:91-94
        // Get total XY length for this layer by summing all extrusion moves.
        let mut total_layer_length = 0.0_f32;
        let mut layer_height = 0.0_f32;
        let mut z = 0.0_f32;

        // SpiralVase.cpp:96-114
        {
            //FIXME Performance warning: This copies the GCodeConfig of the reader.
            // GCodeReader r = m_reader;  // clone
            let mut r = self.m_reader.clone();
            // bool set_z = false;
            let mut set_z = false;
            r.parse_buffer(gcode, |reader, line| {
                // if (line.cmd_is("G1")) {
                if line.cmd_is("G1") {
                    // if (line.extruding(reader)) {
                    if line.extruding(reader) {
                        // total_layer_length += line.dist_XY(reader);
                        total_layer_length += line.dist_xy(reader);
                    // } else if (line.has(Z)) {
                    } else if line.has(Axis::Z) {
                        // layer_height += line.dist_Z(reader);
                        layer_height += line.dist_z(reader);
                        // if (!set_z) {
                        if !set_z {
                            // z = line.new_Z(reader);
                            z = line.new_z(reader);
                            // set_z = true;
                            set_z = true;
                        }
                    }
                }
            });
        }

        // SpiralVase.cpp:116-117
        // Remove layer height from initial Z.
        z -= layer_height;

        // SpiralVase.cpp:119
        // std::shared_ptr<std::vector<SpiralVase::SpiralPoint>> current_layer = std::make_shared<std::vector<SpiralVase::SpiralPoint>>();
        let current_layer: Rc<RefCell<Vec<SpiralPoint>>> = Rc::new(RefCell::new(Vec::new()));
        // SpiralVase.cpp:120
        // std::shared_ptr<std::vector<SpiralVase::SpiralPoint>> previous_layer = m_previous_layer;
        let previous_layer = self.m_previous_layer.clone();

        // SpiralVase.cpp:122: bool smooth_spiral = m_smooth_spiral;
        let smooth_spiral = self.m_smooth_spiral;
        // SpiralVase.cpp:123: std::string new_gcode;
        let mut new_gcode = String::new();
        // SpiralVase.cpp:124: std::string transition_gcode;
        let mut transition_gcode = String::new();
        // SpiralVase.cpp:125: float max_xy_dist_for_smoothing = m_max_xy_smoothing;
        let max_xy_dist_for_smoothing = self.m_max_xy_smoothing;
        // SpiralVase.cpp:126-130
        //FIXME Tapering of the transition layer only works reliably with relative extruder distances.
        // For absolute extruder distances it will be switched off.
        // Tapering the absolute extruder distances requires to process every extrusion value after the first transition
        // layer.
        // bool transition_in = m_transition_layer && m_config.use_relative_e_distances.value;
        let transition_in = self.m_transition_layer && self.m_config.use_relative_e;
        // SpiralVase.cpp:131: bool transition_out = last_layer && m_config.use_relative_e_distances.value;
        let transition_out = last_layer && self.m_config.use_relative_e;
        // SpiralVase.cpp:132: float len = 0.f;
        let mut len = 0.0_f32;
        // SpiralVase.cpp:133-134
        //set initial point
        // SpiralVase::SpiralPoint last_point = previous_layer != NULL && previous_layer->size() > 0 ? previous_layer->at(previous_layer->size()-1): SpiralVase::SpiralPoint(0,0);
        let mut last_point = match &previous_layer {
            Some(pl) if !pl.borrow().is_empty() => {
                let pl_ref = pl.borrow();
                pl_ref[pl_ref.len() - 1]
            }
            _ => SpiralPoint::new(0.0, 0.0),
        };

        // SpiralVase.cpp:136-211
        // m_reader.parse_buffer(gcode, [...](GCodeReader &reader, GCodeReader::GCodeLine line) { ... });
        // NOTE: the C++ lambda takes `GCodeLine line` BY VALUE, so any `line.set(...)`
        // mutations are local to the callback and never feed back into the reader's
        // coordinate tracking (which still uses the original parsed values). We mirror
        // this exactly by cloning into a local mutable `line` at the top of the closure.
        self.m_reader.parse_buffer(gcode, |reader, line| {
            let mut line = line.clone();
            // SpiralVase.cpp:139: if (line.cmd_is("G1")) {
            if line.cmd_is("G1") {
                // SpiralVase.cpp:140: if (line.has_z()) {
                if line.has_z() {
                    // SpiralVase.cpp:141-145
                    // If this is the initial Z move of the layer, replace it with a
                    // (redundant) move to the last Z of previous layer.
                    // line.set(reader, Z, z);
                    line.set(reader, Axis::Z, z, 3);
                    // new_gcode += line.raw() + '\n';
                    new_gcode.push_str(line.raw());
                    new_gcode.push('\n');
                    // return;
                    return;
                } else {
                    // SpiralVase.cpp:147: float dist_XY = line.dist_XY(reader);
                    let dist_xy = line.dist_xy(reader);
                    // SpiralVase.cpp:148: if (dist_XY > 0) {
                    if dist_xy > 0.0 {
                        // SpiralVase.cpp:149: if (line.extruding(reader)) { // Exclude wipe and retract
                        if line.extruding(reader) {
                            // SpiralVase.cpp:150: len += dist_XY;
                            len += dist_xy;
                            // SpiralVase.cpp:151: float factor = len / total_layer_length;
                            let factor = len / total_layer_length;
                            // SpiralVase.cpp:152-154
                            // if (transition_in)
                            //     // Transition layer, interpolate the amount of extrusion from zero to the final value.
                            //     line.set(reader, E, line.e() * factor, 5 /*decimal_digits*/);
                            if transition_in {
                                line.set(reader, Axis::E, line.e() * factor, 5);
                            }
                            // SpiralVase.cpp:155-162
                            // else if (transition_out) {
                            else if transition_out {
                                // We want the last layer to ramp down extrusion, but without changing z height!
                                // So clone the line before we mess with its Z and duplicate it into a new layer that ramps down E
                                // We add this new layer at the very end
                                // GCodeReader::GCodeLine transitionLine(line);
                                let mut transition_line = line.clone();
                                // transitionLine.set(reader, E, line.e() * (1 - factor), 5 /*decimal_digits*/);
                                transition_line.set(reader, Axis::E, line.e() * (1.0 - factor), 5);
                                // transition_gcode += transitionLine.raw() + '\n';
                                transition_gcode.push_str(transition_line.raw());
                                transition_gcode.push('\n');
                            }
                            // SpiralVase.cpp:163-164
                            // This line is the core of Spiral Vase mode, ramp up the Z smoothly
                            // line.set(reader, Z, z + factor * layer_height);
                            line.set(reader, Axis::Z, z + factor * layer_height, 3);
                            // SpiralVase.cpp:165: if (smooth_spiral) {
                            if smooth_spiral {
                                // SpiralVase.cpp:166-167
                                // Now we also need to try to interpolate X and Y
                                // SpiralVase::SpiralPoint p(line.x(), line.y()); // Get current x/y coordinates
                                let p = SpiralPoint::new(line.x(), line.y());
                                // SpiralVase.cpp:168: current_layer->push_back(p); // Store that point for later use on the next layer
                                current_layer.borrow_mut().push(p);
                                // SpiralVase.cpp:169: if (previous_layer != NULL) {
                                if let Some(previous_layer) = &previous_layer {
                                    // SpiralVase.cpp:170: bool found = false;
                                    let mut found = false;
                                    // SpiralVase.cpp:171: float dist = 0;
                                    let mut dist = 0.0_f32;
                                    // SpiralVase.cpp:172: SpiralVase::SpiralPoint nearestp = SpiralVaseHelpers::nearest_point_on_lines(p, previous_layer, found, dist);
                                    let nearestp = spiral_vase_helpers::nearest_point_on_lines(
                                        p,
                                        &previous_layer.borrow(),
                                        &mut found,
                                        &mut dist,
                                    );
                                    // SpiralVase.cpp:173: if (found && dist < max_xy_dist_for_smoothing) {
                                    if found && dist < max_xy_dist_for_smoothing {
                                        // SpiralVase.cpp:174-175
                                        // Interpolate between the point on this layer and the point on the previous layer
                                        // SpiralVase::SpiralPoint target = SpiralVaseHelpers::add(SpiralVaseHelpers::scale(nearestp, 1 - factor), SpiralVaseHelpers::scale(p, factor));
                                        let target = spiral_vase_helpers::add(
                                            spiral_vase_helpers::scale(nearestp, 1.0 - factor),
                                            spiral_vase_helpers::scale(p, factor),
                                        );

                                        // SpiralVase.cpp:177-179
                                        // BBS: remove too short movement
                                        // We need to figure out the distance of this new line!
                                        // float modified_dist_XY = SpiralVaseHelpers::distance(last_point, target);
                                        let modified_dist_xy =
                                            spiral_vase_helpers::distance(last_point, target);
                                        // SpiralVase.cpp:180-181
                                        // if (modified_dist_XY < 0.001)
                                        //     line.clear();
                                        if modified_dist_xy < 0.001 {
                                            line.clear();
                                        // SpiralVase.cpp:182-188
                                        // else {
                                        } else {
                                            // line.set(reader, X, target.x);
                                            line.set(reader, Axis::X, target.x, 3);
                                            // line.set(reader, Y, target.y);
                                            line.set(reader, Axis::Y, target.y, 3);
                                            // Scale the extrusion amount according to change in length
                                            // line.set(reader, E, line.e() * modified_dist_XY / dist_XY, 5 /*decimal_digits*/);
                                            line.set(
                                                reader,
                                                Axis::E,
                                                line.e() * modified_dist_xy / dist_xy,
                                                5,
                                            );
                                            // last_point = target;
                                            last_point = target;
                                        }
                                    // SpiralVase.cpp:189-191
                                    // } else {
                                    //     last_point = p;
                                    // }
                                    } else {
                                        last_point = p;
                                    }
                                }
                            }
                            // SpiralVase.cpp:194: new_gcode += line.raw() + '\n';
                            new_gcode.push_str(line.raw());
                            new_gcode.push('\n');
                        }
                        // SpiralVase.cpp:196: return;
                        return;
                        // SpiralVase.cpp:197-203
                        /*  Skip travel moves: the move to first perimeter point will
                            cause a visible seam when loops are not aligned in XY; by skipping
                            it we blend the first loop move in the XY plane (although the smoothness
                            of such blend depend on how long the first segment is; maybe we should
                            enforce some minimum length?).
                            When smooth_spiral is enabled, we're gonna end up exactly where the next layer should
                            start anyway, so we don't need the travel move */
                    }
                }
            }
            // SpiralVase.cpp:207: new_gcode += line.raw() + '\n';
            new_gcode.push_str(line.raw());
            new_gcode.push('\n');
            // SpiralVase.cpp:208-210
            // if(transition_out) {
            //     transition_gcode += line.raw() + '\n';
            // }
            if transition_out {
                transition_gcode.push_str(line.raw());
                transition_gcode.push('\n');
            }
        });

        // SpiralVase.cpp:213: m_previous_layer = current_layer;
        self.m_previous_layer = Some(current_layer);

        // SpiralVase.cpp:215: return new_gcode + transition_gcode;
        new_gcode.push_str(&transition_gcode);
        new_gcode
    }
}

// SpiralVase.cpp:218: }  // namespace Slic3r

/// Build the `GCodeConfig` subset consumed by `GCodeReader` from a `PrintConfig`.
///
/// In BambuStudio `PrintConfig` derives from `GCodeConfig`, so
/// `m_reader.apply_config(m_config)` simply slices the relevant fields out of the
/// full config. This crate keeps the two structs separate, so we copy the fields
/// the reader actually needs (notably `use_relative_e_distances`).
fn gcode_config_from_print_config(config: &PrintConfig) -> GCodeConfig {
    GCodeConfig {
        gcode_flavor: config.gcode_flavor,
        travel_speed: config.travel_speed,
        toolchange_gcode: String::new(),
        use_relative_e_distances: config.use_relative_e,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config(smooth: bool, relative_e: bool) -> PrintConfig {
        let mut c = PrintConfig::default();
        c.spiral_mode_smooth = smooth;
        c.use_relative_e = relative_e;
        c
    }

    // ---- SpiralVaseHelpers (SpiralVase.cpp:9-72) ----

    #[test]
    fn test_distance() {
        let a = SpiralPoint::new(0.0, 0.0);
        let b = SpiralPoint::new(3.0, 4.0);
        assert!((spiral_vase_helpers::distance(a, b) - 5.0).abs() < 1e-5);
    }

    #[test]
    fn test_subtract_add_scale_dot() {
        let a = SpiralPoint::new(5.0, 7.0);
        let b = SpiralPoint::new(2.0, 3.0);
        assert_eq!(spiral_vase_helpers::subtract(a, b), SpiralPoint::new(3.0, 4.0));
        assert_eq!(spiral_vase_helpers::add(a, b), SpiralPoint::new(7.0, 10.0));
        assert_eq!(spiral_vase_helpers::scale(a, 2.0), SpiralPoint::new(10.0, 14.0));
        assert!((spiral_vase_helpers::dot(a, b) - (5.0 * 2.0 + 7.0 * 3.0)).abs() < 1e-5);
    }

    #[test]
    fn test_nearest_point_on_line_clamped() {
        // Segment from (0,0) to (10,0); c is past the far end -> clamps to b.
        let mut dist = 0.0;
        let closest = spiral_vase_helpers::nearest_point_on_line(
            SpiralPoint::new(20.0, 5.0),
            SpiralPoint::new(0.0, 0.0),
            SpiralPoint::new(10.0, 0.0),
            &mut dist,
        );
        assert_eq!(closest, SpiralPoint::new(10.0, 0.0));
        assert!((dist - (100.0_f32 + 25.0).sqrt()).abs() < 1e-4);

        // Point projecting onto the middle.
        let mut dist2 = 0.0;
        let closest2 = spiral_vase_helpers::nearest_point_on_line(
            SpiralPoint::new(5.0, 3.0),
            SpiralPoint::new(0.0, 0.0),
            SpiralPoint::new(10.0, 0.0),
            &mut dist2,
        );
        assert_eq!(closest2, SpiralPoint::new(5.0, 0.0));
        assert!((dist2 - 3.0).abs() < 1e-5);
    }

    #[test]
    fn test_nearest_point_on_lines_not_enough_points() {
        let mut found = true;
        let mut dist = 0.0;
        let pts = vec![SpiralPoint::new(1.0, 1.0)];
        let r = spiral_vase_helpers::nearest_point_on_lines(
            SpiralPoint::new(0.0, 0.0),
            &pts,
            &mut found,
            &mut dist,
        );
        assert!(!found);
        assert_eq!(r, SpiralPoint::new(0.0, 0.0));
    }

    #[test]
    fn test_nearest_point_on_lines_found() {
        let mut found = false;
        let mut dist = 0.0;
        let pts = vec![
            SpiralPoint::new(0.0, 0.0),
            SpiralPoint::new(10.0, 0.0),
            SpiralPoint::new(10.0, 10.0),
        ];
        let r = spiral_vase_helpers::nearest_point_on_lines(
            SpiralPoint::new(5.0, 2.0),
            &pts,
            &mut found,
            &mut dist,
        );
        assert!(found);
        assert_eq!(r, SpiralPoint::new(5.0, 0.0));
        assert!((dist - 2.0).abs() < 1e-5);
    }

    // ---- process_layer (SpiralVase.cpp:74-216) ----

    #[test]
    fn test_process_layer_disabled_passthrough() {
        let mut sv = SpiralVase::new(&make_config(false, true));
        let gcode = "G1 X10 Y0 E1\nG1 X10 Y10 E1\n";
        // m_enabled is false -> output is identical to input.
        let out = sv.process_layer(gcode, false);
        assert_eq!(out, gcode);
    }

    #[test]
    fn test_process_layer_ramps_z() {
        let mut sv = SpiralVase::new(&make_config(false, true));
        // enable() sets transition_layer because it was previously disabled.
        sv.enable(true);
        // Initial Z move, then two extruding moves around a (degenerate) loop.
        // total_layer_length = 10 + 10 = 20; layer_height = 0.2; z = 0.2 - 0.2 = 0.
        let gcode = "G1 Z0.2 F600\nG1 X10 Y0 E1\nG1 X10 Y10 E1\n";
        let out = sv.process_layer(gcode, false);
        // The initial Z move is rewritten to the previous-layer Z (0).
        assert!(out.contains("Z0.000"));
        // Final extruding move should reach the full layer height (factor == 1 -> Z 0.200).
        assert!(out.contains("Z0.200"), "out was:\n{}", out);
    }

    #[test]
    fn test_process_layer_transition_in_scales_e() {
        let mut sv = SpiralVase::new(&make_config(false, true));
        sv.enable(true); // first enabled layer => transition_layer == true
        // Single extruding move spanning the whole layer: factor goes 1.0 at the end.
        let gcode = "G1 Z0.2\nG1 X10 Y0 E2\n";
        let out = sv.process_layer(gcode, false);
        // With factor == 1.0 the E value is unchanged (E2.00000) but is reformatted to 5 dp.
        assert!(out.contains("E2.00000"), "out was:\n{}", out);
    }
}
