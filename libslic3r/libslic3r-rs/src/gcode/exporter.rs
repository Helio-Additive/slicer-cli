//! G-code export helper functions.
//!
//! This module provides helper functions for exporting ExtrusionEntity objects
//! to G-code, mirroring the helper methods in BambuStudio's GCode class.
//!
//! C++ reference: GCode.cpp (helper methods throughout)

use crate::extrusion_entity::{
    ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionPath, ExtrusionRole,
};
use crate::gcode::arc_fitting::{ArcFitter, ArcFittingConfig, PathSegment};
use crate::gcode::cooling::CoolingBuffer;
use crate::gcode::writer::GCodeWriter;
use crate::geometry::{Point, PointF, Polyline};
use crate::print_config::PrintConfig;
use crate::Result;
use crate::{scale, unscale};

/// Configuration for travel moves.
///
/// C++ reference: GCode class member variables
/// GCode.hpp:100-200
#[derive(Debug, Clone)]
pub struct TravelConfig {
    /// Avoid crossing perimeters if enabled
    pub avoid_crossing_perimeters: bool,
    /// Enable retraction on travel moves
    pub retract_on_travel: bool,
    /// Minimum travel distance to trigger retraction (mm)
    pub retract_length_travel: f64,
    /// Enable Z-hop during travel
    pub z_hop: bool,
    /// Z-hop height (mm)
    pub z_hop_height: f64,
}

impl Default for TravelConfig {
    fn default() -> Self {
        Self {
            avoid_crossing_perimeters: false,
            retract_on_travel: true,
            retract_length_travel: 2.0,
            z_hop: false,
            z_hop_height: 0.0,
        }
    }
}

/// Extrude a single extrusion loop.
///
/// C++ reference: GCode::extrude_loop()
/// GCode.cpp:5071-5270 (~200 lines)
///
/// This function:
/// 1. Makes loop counter-clockwise (CCW orientation)
/// 2. Finds optimal seam/starting point
/// 3. Splits loop at seam point
/// 4. Clips end to create small gap at seam
/// 5. Extrudes each path segment
/// 6. Handles variable width and arc fitting
///
/// # Arguments
/// * `loop_entity` - The extrusion loop to extrude
/// * `writer` - GCodeWriter to emit commands
/// * `config` - Print configuration for flow calculations
/// * `is_first_layer` - Whether this is the first layer (affects flow ratio)
pub fn extrude_loop(
    loop_entity: &ExtrusionLoop,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) {
    // C++ reference: GCode.cpp:5071-5076
    // C++: std::string GCode::extrude_loop(ExtrusionLoop loop, std::string description, double speed)
    // C++: {
    // C++: // get a copy; don't modify the orientation of the original loop object otherwise
    // C++: // next copies (if any) would not detect the correct orientation
    // Make a copy to avoid modifying the original
    let mut loop_copy = loop_entity.clone();

    // C++ reference: GCode.cpp:5077-5078
    // C++: // extrude all loops ccw
    // C++: bool was_clockwise = loop.make_counter_clockwise();
    // Ensure counter-clockwise orientation
    loop_copy.make_counter_clockwise();

    // C++ reference: GCode.cpp:5079-5080
    // C++: bool is_hole = loop.loop_role() & elrPerimeterHole;
    // C++: Point last_pos = this->last_pos();
    // Get current position for seam placement
    let last_pos_f = writer.position();
    let last_pos = Point::new(
        (last_pos_f.x * 1_000_000.0) as i64,
        (last_pos_f.y * 1_000_000.0) as i64,
    );

    // C++ reference: GCode.cpp:5081-5088
    // C++: if (!m_config.spiral_mode && description == "perimeter") {
    // C++: assert(m_layer != nullptr);
    // C++: bool is_outer_wall_first = m_config.wall_sequence == WallSequence::OuterInner;
    // C++: m_seam_placer.place_seam(m_layer, loop, is_outer_wall_first, this->last_pos(), satisfy_scarf_seam_angle_threshold);
    // C++: } else
    // C++: loop.split_at(last_pos, false);
    // SeamPlacer integration (GCode.cpp:5081-5088)
    // Use find_best_seam_index for angle-based scoring with rear bias + alignment
    let polygon = loop_copy.as_polygon();
    let seam_idx = super::seam_placer::find_best_seam_index(
        &polygon,
        Some(last_pos),
        &super::seam_placer::SeamPlacerConfig::default(),
    );
    let seam_point = polygon.points()[seam_idx];
    split_loop_at_closest_point(&mut loop_copy, seam_point);

    // C++ reference: GCode.cpp:5107-5117
    // C++: const double seam_gap = scale_(EXTRUDER_CONFIG(nozzle_diameter)) * (m_config.seam_gap.value / 100);
    // C++: const double clip_length = m_enable_loop_clipping && !enable_seam_slope ? seam_gap : 0;
    // C++: // get paths
    // C++: ExtrusionPaths paths;
    // C++: ...
    // C++: loop.clip_end(clip_length, &paths);
    // C++: if (paths.empty()) return "";
    // TODO: Implement seam gap clipping (GCode.cpp:5107-5117)
    // For now, use the paths directly without clipping
    let paths = &loop_copy.paths;

    // C++ reference: GCode.cpp:5119-5122
    // C++: double small_peri_speed=-1;
    // C++: // apply the small perimeter speed
    // C++: if (loop.length() <= SMALL_PERIMETER_LENGTH(NOZZLE_CONFIG(small_perimeter_threshold)))
    // C++: small_peri_speed = NOZZLE_CONFIG(small_perimeter_speed).get_abs_value(NOZZLE_CONFIG(outer_wall_speed));
    // TODO: Implement small perimeter speed adjustment (GCode.cpp:5119-5122)

    // C++ reference: GCode.cpp:5089-5195
    // TODO: Implement seam slope (scarf seam) feature (GCode.cpp:5089-5195)
    // This is an advanced feature for better seam quality
    // Skip for initial implementation

    // C++ reference: GCode.cpp:5208-5218
    // C++: if (!enable_seam_slope || slope_has_overhang) {
    // C++: ...
    // C++: for (ExtrusionPaths::iterator path = paths.begin(); path != paths.end(); ++path) {
    // C++: gcode += this->_extrude(*path, description, speed_for_path(*path), set_holes_and_compensation_speed);
    // C++: }
    // C++: set_last_scarf_seam_flag(false);
    // C++: }
    // Extrude each path in the loop
    for path in paths {
        extrude_path(path, writer, config, is_first_layer);
    }

    // C++ reference: GCode.cpp:5220-5227
    // C++: //BBS: don't reset acceleration when printing first layer. During first layer, acceleration is always same value.
    // C++: if (!this->on_first_layer()) {
    // C++: // reset acceleration
    // C++: m_writer.set_acceleration((unsigned int) (NOZZLE_CONFIG(default_acceleration) + 0.5));
    // C++: if (!this->is_BBL_Printer())
    // C++: gcode += m_writer.set_jerk_xy(m_config.default_jerk.value);
    // C++: }
    // TODO: Implement acceleration reset (GCode.cpp:5220-5227)

    // C++ reference: GCode.cpp:5230-5241
    // C++: // BBS
    // C++: if (m_wipe.enable && FILAMENT_CONFIG(wipe)) {
    // C++: m_wipe.path = Polyline();
    // C++: for (ExtrusionPath &path : paths) {
    // C++: ...
    // C++: m_wipe.path.append(path.polyline);
    // C++: }
    // C++: }
    // TODO: Implement wipe path saving (GCode.cpp:5230-5241)
}

/// Helper function to split loop at point closest to current position.
///
/// C++ reference: ExtrusionLoop::split_at()
/// ExtrusionEntity.cpp:350-400
///
/// This reorders the paths in the loop so that the split point becomes
/// the new starting point.
fn split_loop_at_closest_point(loop_entity: &mut ExtrusionLoop, point: Point) {
    // C++ reference: ExtrusionEntity.cpp:350-360
    // C++: void ExtrusionLoop::split_at(const Point &point, bool prefer_non_overhang)
    // C++: {
    // C++: if (this->paths.empty()) return;
    // Check if loop is empty
    if loop_entity.paths.is_empty() {
        return;
    }

    // C++ reference: ExtrusionEntity.cpp:361-380
    // C++: // Find the path containing the point and the idx of the point in the path
    // C++: size_t path_idx = 0;
    // C++: Point p = this->paths.front().polyline.first_point();
    // C++: for (size_t i = 0; i < this->paths.size(); ++ i) {
    // C++: const Polyline &polyline = this->paths[i].polyline;
    // C++: for (size_t j = 0; j < polyline.points.size(); ++ j) {
    // C++: if (polyline.points[j].distance_to(point) < p.distance_to(point)) {
    // C++: p = polyline.points[j];
    // C++: path_idx = i;
    // C++: }
    // C++: }
    // C++: }
    // Find closest point in loop
    let mut best_path_idx = 0;
    let mut best_point_idx = 0;
    let mut best_distance = loop_entity.paths[0].polyline.points()[0].distance_to(&point);

    for (path_idx, path) in loop_entity.paths.iter().enumerate() {
        for (point_idx, p) in path.polyline.points().iter().enumerate() {
            let dist = p.distance_to(&point);
            if dist < best_distance {
                best_distance = dist;
                best_path_idx = path_idx;
                best_point_idx = point_idx;
            }
        }
    }

    // C++ reference: ExtrusionEntity.cpp:381-395
    // C++: // Split the path at the closest point
    // C++: if (path_idx == 0 && point_idx == 0) {
    // C++: // Already at the start
    // C++: return;
    // C++: }
    // C++: // Rotate the paths
    // C++: std::rotate(this->paths.begin(), this->paths.begin() + path_idx, this->paths.end());
    // If already at start, nothing to do
    if best_path_idx == 0 && best_point_idx == 0 {
        return;
    }

    // TODO: Implement proper path splitting at the exact point (GCode.cpp:381-395)
    // For now, rotate paths to start at the closest path
    // This is a simplification - proper implementation should split the path at the point
    loop_entity.paths.rotate_left(best_path_idx);
}

/// Extrude an extrusion entity collection.
///
/// C++ reference: GCode::extrude_multi_path() / extrude_collection()
/// GCode.cpp:2800-3100 (~300 lines)
///
/// This function:
/// 1. Iterates through all entities in the collection
/// 2. Handles role changes between entities
/// 3. Emits feature type annotations
/// 4. Calls appropriate extrude function for each entity
///
/// # Arguments
/// * `collection` - The collection to extrude
/// * `writer` - GCodeWriter to emit commands
/// * `config` - Print configuration for flow calculations
/// * `is_first_layer` - Whether this is the first layer (affects flow ratio)
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors
pub fn extrude_collection(
    collection: &ExtrusionEntityCollection,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) -> Result<()> {
    // Extrude an ExtrusionEntityCollection by iterating entities
    // GCode.cpp:2800-3100
    // C++: std::string GCode::extrude_entity(const ExtrusionEntity &entity, const std::string &description, double speed)

    // Early return for empty collection
    // GCode.cpp:2802-2803
    // C++: if (entity.is_collection())
    // C++: return this->_extrude(*static_cast<const ExtrusionEntityCollection*>(&entity), description, speed);
    if collection.entities.is_empty() {
        return Ok(());
    }

    // Track current role for feature comments
    // GCode.cpp:2850-2855
    // C++: ExtrusionRole current_role = erNone;
    let mut current_role: Option<ExtrusionRole> = None;

    // Iterate through entities in collection
    // GCode.cpp:2860-2900
    // C++: for (const ExtrusionEntity *entity : collection.entities) {
    // C++: // Emit feature comment on role change
    // C++: if (entity->role() != current_role) {
    // C++: gcode += "; FEATURE: " + entity->role_to_string() + "\n";
    // C++: current_role = entity->role();
    // C++: }
    // C++: gcode += this->extrude_entity(*entity, description, speed);
    // C++: }
    for entity in &collection.entities {
        // Get entity role for feature tracking
        // GCode.cpp:2862-2863
        let entity_role = get_entity_role(entity);

        // Emit feature comment when role changes
        // GCode.cpp:2864-2868
        // C++: if (entity->role() != current_role) {
        // C++: gcode += "; FEATURE: ";
        // C++: gcode += ExtrusionEntity::role_to_string(entity->role());
        // C++: gcode += "\n";
        // C++: }
        if Some(entity_role) != current_role {
            writer.write_comment(&format!("FEATURE: {}", entity_role.to_string()));
            // Emit LINE_WIDTH annotation for the feature
            // First layer uses initial_layer_line_width if set
            let line_width = if is_first_layer && config.initial_layer_line_width > 0.0 {
                config.initial_layer_line_width
            } else {
                match entity_role {
                    ExtrusionRole::ExternalPerimeter => config.outer_wall_line_width,
                    ExtrusionRole::Perimeter => config.inner_wall_line_width,
                    ExtrusionRole::InternalInfill => config.sparse_infill_line_width,
                    ExtrusionRole::SolidInfill => config.solid_infill_line_width,
                    ExtrusionRole::TopSolidInfill => config.top_surface_line_width,
                    _ => config.outer_wall_line_width,
                }
            };
            if line_width > 0.0 {
                // Format LINE_WIDTH to match BambuStudio: trim trailing zeros
                let lw_str = format!("{:.5}", line_width);
                let lw_trimmed = lw_str.trim_end_matches('0').trim_end_matches('.');
                writer.write_comment(&format!("LINE_WIDTH: {}", lw_trimmed));
            }
            // Set speed for this feature (before M204, matching reference order)
            // C++ GCode.cpp:6175-6200: first layer uses initial_layer_speed from config
            let feature_speed = if is_first_layer {
                // BambuStudio uses initial_layer_infill_speed for infill on first layer,
                // and initial_layer_speed for walls/other features.
                match entity_role {
                    ExtrusionRole::InternalInfill
                    | ExtrusionRole::SolidInfill
                    | ExtrusionRole::TopSolidInfill
                    | ExtrusionRole::BottomSurface => {
                        if config.initial_layer_infill_speed > 0.0 {
                            config.initial_layer_infill_speed
                        } else {
                            config.initial_layer_speed
                        }
                    }
                    _ => config.initial_layer_speed,
                }
            } else {
                match entity_role {
                    ExtrusionRole::ExternalPerimeter => config.external_perimeter_speed,
                    ExtrusionRole::Perimeter => config.perimeter_speed,
                    ExtrusionRole::InternalInfill => config.infill_speed,
                    ExtrusionRole::SolidInfill => config.solid_infill_speed,
                    ExtrusionRole::TopSolidInfill => config.top_solid_infill_speed,
                    ExtrusionRole::BridgeInfill => config.bridge_speed,
                    ExtrusionRole::GapFill => config.gap_fill_speed,
                    _ => config.perimeter_speed,
                }
            };
            // Emit cooling markers for CoolingBuffer post-processor
            // C++ GCode.cpp:6253-6272
            let cooling_comment = if entity_role == ExtrusionRole::BridgeInfill {
                // Bridge moves are not adjustable
                ""
            } else if entity_role == ExtrusionRole::ExternalPerimeter {
                ";_EXTRUDE_SET_SPEED;_EXTERNAL_PERIMETER"
            } else {
                ";_EXTRUDE_SET_SPEED"
            };
            writer.set_speed(feature_speed * 60.0, cooling_comment);
            // Emit per-feature acceleration (M204) matching BambuStudio
            // First layer uses initial_layer_acceleration from settings
            let accel = if is_first_layer {
                500u32 // initial_layer_acceleration default for X1C
            } else {
                match entity_role {
                    ExtrusionRole::ExternalPerimeter => 5000,
                    ExtrusionRole::Perimeter => 5000,
                    ExtrusionRole::InternalInfill => 10000,
                    ExtrusionRole::SolidInfill => 10000,
                    ExtrusionRole::TopSolidInfill => 5000,
                    ExtrusionRole::BridgeInfill => 2500,
                    ExtrusionRole::GapFill => 5000,
                    _ => 10000,
                }
            };
            writer.write_raw(&format!("M204 S{}", accel));
            current_role = Some(entity_role);
        }

        // Travel to start of this entity if nozzle is not already there.
        // C++ GCode::extrude_entity() calls travel_to() before extruding each entity.
        // Without this, consecutive entities in the same collection are connected
        // by a spurious extrusion line across open space.
        if let Some(first_pt) = get_entity_first_point(entity) {
            let pos = writer.position();
            let tx = crate::unscale(first_pt.x());
            let ty = crate::unscale(first_pt.y());
            let dist_sq = (pos.x - tx) * (pos.x - tx) + (pos.y - ty) * (pos.y - ty);
            if dist_sq > 0.001 * 0.001 {
                // Intra-collection travel: direct G1 travel, no retract/unretract.
                // C++ GCode.cpp emits M204 S6000 + G1 F30000 XY without retracting
                // for adjacent loops within the same perimeter/infill collection.
                writer.set_travel_acceleration(6000.0);
                writer.travel_to(tx, ty, None);
            }
        }

        // Recursively extrude the entity
        // GCode.cpp:2870-2880
        // C++: gcode += this->extrude_entity(*entity, description, speed);
        extrude_entity(entity, writer, config, is_first_layer)?;
    }

    Ok(())
}

/// Get the first point of an extrusion entity for travel-to targeting.
pub fn get_entity_first_point(entity: &ExtrusionEntityType) -> Option<Point> {
    match entity {
        ExtrusionEntityType::Path(path) => path.polyline.points().first().copied(),
        ExtrusionEntityType::Loop(loop_entity) => loop_entity
            .paths
            .first()
            .and_then(|p| p.polyline.points().first().copied()),
        ExtrusionEntityType::Collection(coll) => coll
            .entities
            .first()
            .and_then(|e| get_entity_first_point(e)),
    }
}

/// Helper to extract role from ExtrusionEntityType
/// GCode.cpp:2100-2120 (role accessor methods)
fn get_entity_role(entity: &ExtrusionEntityType) -> ExtrusionRole {
    match entity {
        ExtrusionEntityType::Path(path) => path.role,
        ExtrusionEntityType::Loop(loop_entity) => {
            // Loops may have multiple paths with different roles
            // Use first path's role, or Mixed if empty
            // GCode.cpp:2105
            // C++: ExtrusionRole ExtrusionLoop::role() const {
            // C++: return paths.empty() ? erNone : paths.front().role();
            // C++: }
            loop_entity
                .paths
                .first()
                .map(|p| p.role)
                .unwrap_or(ExtrusionRole::None)
        }
        ExtrusionEntityType::Collection(coll) => {
            // Recurse into first entity of collection to get the actual role
            coll.entities
                .first()
                .map(|e| get_entity_role(e))
                .unwrap_or(ExtrusionRole::Mixed)
        }
    }
}

/// Alias for backward compatibility (C++ has both multi_path and collections)
pub use extrude_collection as extrude_multi_path;

/// Extrude a single extrusion path.
///
/// C++ reference: GCode::_extrude()
/// GCode.cpp:4200-4800 (~600 lines)
///
/// This function:
/// 1. Generates G1 commands for each point in the path
/// 2. Calculates extrusion amount (E value) for each segment
/// 3. Handles variable width segments
/// 4. Applies arc fitting if enabled
///
/// # Arguments
/// * `path` - The extrusion path to extrude
/// * `writer` - GCodeWriter to emit commands
/// * `config` - Print configuration for flow calculations
/// * `is_first_layer` - Whether this is the first layer (affects flow ratio)
pub fn extrude_path(
    path: &ExtrusionPath,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) {
    extrude_path_ex(path, writer, config, is_first_layer, None);
}

/// Extrude a path with access to actual PrintConfig for arc fitting.
pub fn extrude_path_ex(
    path: &ExtrusionPath,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
    print_config: Option<&PrintConfig>,
) {
    // Enable arc fitting only for fuzzy skin perimeters with many points
    let is_perimeter = matches!(
        path.role,
        crate::extrusion_entity::ExtrusionRole::ExternalPerimeter
            | crate::extrusion_entity::ExtrusionRole::Perimeter
    );
    if is_perimeter && config.fuzzy_skin && path.polyline.points().len() > 20 {
        extrude_path_with_arc_fitting(path, writer, config, is_first_layer, print_config);
    } else {
        extrude_path_with_arc_fitting(path, writer, config, is_first_layer, None);
    }
}

/// Extrude a path with optional arc fitting.
///
/// When `print_config` is provided and arc fitting is enabled in it,
/// sequences of line segments that form arcs will be converted to
/// G2/G3 arc commands.
pub fn extrude_path_with_arc_fitting(
    path: &ExtrusionPath,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
    print_config: Option<&PrintConfig>,
) {
    // C++ reference: GCode.cpp:4200-4210
    // C++: std::string GCode::_extrude(const ExtrusionPath &path, std::string description, double speed, bool is_hole_or_comp_speed)
    // C++: {
    // C++: std::string gcode;
    // C++:
    // C++: // check if path is empty
    // C++: if (path.polyline.points.empty())
    // C++: return "";
    // Check if path is empty
    if path.polyline.points().is_empty() {
        return;
    }

    // C++ reference: GCode.cpp:4211-4220
    // C++: // get path properties
    // C++: const Polyline &polyline = path.polyline;
    // C++: const double mm3_per_mm = path.mm3_per_mm;
    // C++: const double width = path.width;
    // C++: const double height = path.height;
    // Get path properties
    let points = path.polyline.points();
    let mm3_per_mm = path.mm3_per_mm;
    let _width = path.width;
    let _height = path.height;

    // Calculate extrusion length per distance unit
    // GCode.cpp:6071-6081
    // C++: auto _mm3_per_mm = path.mm3_per_mm * double(this->config().print_flow_ratio.value);
    // C++: if( path.role() == erTopSolidInfill )
    // C++: _mm3_per_mm *= NOZZLE_CONFIG(top_solid_infill_flow_ratio);
    // C++: else if (this->on_first_layer())
    // C++: _mm3_per_mm *= m_config.initial_layer_flow_ratio.value;
    // C++: double e_per_mm = m_writer.filament()->e_per_mm3() * _mm3_per_mm;
    //
    // Apply extrusion multiplier (print_flow_ratio)
    // GCode.cpp:6073
    // C++: auto _mm3_per_mm = path.mm3_per_mm * double(this->config().print_flow_ratio.value);
    let mut adjusted_mm3_per_mm = mm3_per_mm * config.print_flow_ratio;

    // Apply role-specific flow multipliers
    // GCode.cpp:6074-6077
    // C++: if( path.role() == erTopSolidInfill )
    // C++: _mm3_per_mm *= NOZZLE_CONFIG(top_solid_infill_flow_ratio);
    // C++: else if (this->on_first_layer())
    // C++: _mm3_per_mm *= m_config.initial_layer_flow_ratio.value;
    if path.role == crate::extrusion_entity::ExtrusionRole::TopSolidInfill {
        adjusted_mm3_per_mm *= config.top_solid_infill_flow_ratio;
    } else if is_first_layer {
        adjusted_mm3_per_mm *= config.initial_layer_flow_ratio;
    }

    // Use extruder's e_per_mm3 to convert mm³/mm to filament mm/mm
    let e_per_mm = writer.extruder_e_per_mm(adjusted_mm3_per_mm);

    // Speed is set per-feature in extrude_collection (before M204)
    // No need to set again here.

    // Determine if arc fitting should be applied
    let arc_fitting_enabled = print_config
        .map(|pc| pc.arc_fitting_enabled)
        .unwrap_or(false);

    if arc_fitting_enabled {
        // Convert all scaled points to mm-coordinate PointF values
        let mm_points: Vec<PointF> = points
            .iter()
            .map(|p| PointF::new(unscale(p.x()), unscale(p.y())))
            .collect();

        // Build arc fitting config from PrintConfig
        let pc = print_config.unwrap();
        let arc_config = ArcFittingConfig::new()
            .tolerance(pc.arc_fitting_tolerance)
            .min_radius(pc.arc_fitting_min_radius)
            .max_radius(pc.arc_fitting_max_radius)
            .enabled(true);
        let fitter = ArcFitter::new(arc_config);
        let segments = fitter.process_points(&mm_points);

        // Emit G-code for each path segment
        for segment in &segments {
            match segment {
                PathSegment::Line(line_points) => {
                    // Emit G1 for each consecutive pair of points
                    for i in 1..line_points.len() {
                        let from = &line_points[i - 1];
                        let to = &line_points[i];
                        let dx = to.x - from.x;
                        let dy = to.y - from.y;
                        let segment_length = (dx * dx + dy * dy).sqrt();
                        let de = segment_length * e_per_mm;
                        writer.extrude_to(to.x, to.y, de, None);
                    }
                }
                PathSegment::Arc(arc) => {
                    // Compute E delta from arc length
                    let arc_length = arc.arc_length();
                    let de = arc_length * e_per_mm;

                    // Convert arc direction to the writer's ArcDirection type
                    let direction = arc.direction;

                    // Emit G2/G3 with end point, center offset (I, J), and E delta
                    writer.extrude_arc(arc.end.x, arc.end.y, arc.i, arc.j, de, direction, None);
                }
            }
        }
    } else {
        // Original behavior: emit one G1 per point pair
        // GCode.cpp:4250-4300
        // CRITICAL: Pass dE (delta), not absolute E!
        for i in 1..points.len() {
            let from = points[i - 1];
            let to = points[i];

            let to_x_mm = unscale(to.x());
            let to_y_mm = unscale(to.y());

            let segment_length = unscale(from.distance_to_f64(to) as i64);
            let de = segment_length * e_per_mm;

            if i == 1 {}

            writer.extrude_to(to_x_mm, to_y_mm, de, None);
        }
    }

    // Emit cooling end marker after extrusion
    // C++ GCode.cpp:6361-6363
    writer.write_raw(";_EXTRUDE_END");
}

/// Extrude any extrusion entity (dispatcher).
///
/// C++ reference: GCode::extrude_entity()
/// GCode.cpp:3100-3400 (~300 lines)
///
/// This function dispatches to the appropriate extrusion function
/// based on the entity type (Loop, Path, Collection).
///
/// # Arguments
/// * `entity` - The extrusion entity type to extrude
/// * `writer` - GCodeWriter to emit commands
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors or unsupported entity type
pub fn extrude_entity(
    entity: &ExtrusionEntityType,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) -> Result<()> {
    // Dispatch based on entity type
    // GCode.cpp:3120-3180
    // C++: std::string GCode::extrude_entity(const ExtrusionEntity &entity, const std::string &description, double speed) {
    // C++: if (entity.is_loop()) {
    // C++: return this->extrude_loop(*static_cast<const ExtrusionLoop*>(&entity), description, speed);
    // C++: } else if (entity.is_path()) {
    // C++: return this->_extrude(*static_cast<const ExtrusionPath*>(&entity), description, speed);
    // C++: } else if (entity.is_collection()) {
    // C++: return this->_extrude(*static_cast<const ExtrusionEntityCollection*>(&entity), description, speed);
    // C++: } else {
    // C++: throw std::runtime_error("Unknown extrusion entity type");
    /// C++: }
    /// C++: }
    match entity {
        // Dispatch to loop handler
        // GCode.cpp:3122-3125
        // C++: if (entity.is_loop()) {
        // C++: return this->extrude_loop(*static_cast<const ExtrusionLoop*>(&entity), description, speed);
        // C++: }
        ExtrusionEntityType::Loop(loop_entity) => {
            extrude_loop(loop_entity, writer, config, is_first_layer);
        }

        // Dispatch to path handler
        // GCode.cpp:3126-3129
        // C++: else if (entity.is_path()) {
        // C++: return this->_extrude(*static_cast<const ExtrusionPath*>(&entity), description, speed);
        // C++: }
        ExtrusionEntityType::Path(path) => {
            extrude_path(path, writer, config, is_first_layer);
        }

        // Dispatch to collection handler (recursive)
        // GCode.cpp:3130-3133
        // C++: else if (entity.is_collection()) {
        // C++: return this->_extrude(*static_cast<const ExtrusionEntityCollection*>(&entity), description, speed);
        // C++: }
        ExtrusionEntityType::Collection(collection) => {
            extrude_collection(collection, writer, config, is_first_layer)?;
        }
    }

    // TODO : Add feature type annotations
    // GCode.cpp:3150-3200
    // C++: // Emit feature comment if role changed
    // C++: if (entity->role() != m_last_extrusion_role) {
    // C++: gcode += "; FEATURE: " + entity->role_to_string() + "\n";
    // C++: m_last_extrusion_role = entity->role();
    // C++: }

    // TODO : Add metadata comments (description, speed, etc.)
    // GCode.cpp:3210-3250
    // C++: if (!description.empty()) {
    // C++: gcode += "; " + description + "\n";
    /// C++: }
    Ok(())
}

/// Extrude perimeters for a single region on one island.
///
/// C++ reference: GCode::extrude_perimeters()
/// GCode.cpp:5362-5383
///
/// Iterates the region's perimeters and extrudes each entity.
/// When `skip_inner_walls` is true (spiral vase mode), only external
/// perimeters are emitted.
pub fn extrude_perimeters(
    region: &crate::layer::LayerRegion,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
    skip_inner_walls: bool,
) {
    if region.perimeters.entities.is_empty() {
        return;
    }

    // Retract and travel to first perimeter point
    writer.retract();
    writer.set_travel_acceleration(6000.0);
    if let Some(first_pt) = get_entity_first_point(&region.perimeters.entities[0]) {
        let target_x = crate::unscale(first_pt.x());
        let target_y = crate::unscale(first_pt.y());
        writer.travel_to(target_x, target_y, None);
    }
    writer.unretract();

    if skip_inner_walls {
        // Spiral vase mode: only emit outer perimeter entities
        use crate::extrusion_entity::ExtrusionRole;
        for entity in &region.perimeters.entities {
            let role = get_entity_role(entity);
            if role == ExtrusionRole::ExternalPerimeter {
                let _ = extrude_entity(entity, writer, config, is_first_layer);
            }
        }
    } else {
        // GCode.cpp:5372-5380: extrude perimeters via extrude_collection
        // which handles FEATURE comments, LINE_WIDTH, feedrate (F), and
        // extrusion acceleration (M204) per entity role change.
        let _ = extrude_collection(&region.perimeters, writer, config, is_first_layer);
    }
}

/// Extrude infill for a single region.
///
/// C++ reference: GCode::extrude_infill()
/// GCode.cpp:5386-5412
///
/// Chains infill entities by a greedy nearest-neighbor algorithm, then
/// extrudes each. Separates normal infill from ironing.
pub fn extrude_infill(
    region: &crate::layer::LayerRegion,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) {
    if region.fills.entities.is_empty() {
        return;
    }

    // Retract and travel to first infill point
    writer.retract();
    writer.set_travel_acceleration(6000.0);
    if let Some(first_pt) = get_entity_first_point(&region.fills.entities[0]) {
        let target_x = crate::unscale(first_pt.x());
        let target_y = crate::unscale(first_pt.y());
        writer.travel_to(target_x, target_y, None);
    }
    writer.unretract();

    // GCode.cpp:5391-5408: chain and reorder, then extrude
    for fill in &region.fills.entities {
        match fill {
            ExtrusionEntityType::Collection(eec) => {
                let _ = extrude_collection(eec, writer, config, is_first_layer);
            }
            _ => {
                let _ = extrude_entity(fill, writer, config, is_first_layer);
            }
        }
    }
}

/// Extrude support material fills.
///
/// C++ reference: GCode::extrude_support()
/// GCode.cpp:5414-5470
///
/// Handles support material, support interface, support transition,
/// and support ironing. Each role gets its own label and speed.
pub fn extrude_support(
    support_fills: &ExtrusionEntityCollection,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) {
    if support_fills.entities.is_empty() {
        return;
    }

    // Retract and travel to first support point
    writer.retract();
    writer.set_travel_acceleration(6000.0);
    if let Some(first_pt) = get_entity_first_point(&support_fills.entities[0]) {
        let target_x = crate::unscale(first_pt.x());
        let target_y = crate::unscale(first_pt.y());
        writer.travel_to(target_x, target_y, None);
    }
    writer.unretract();

    // GCode.cpp:5423-5470: chain and extrude with role labels
    for ee in &support_fills.entities {
        let role = get_entity_role(ee);
        let label = match role {
            crate::extrusion_entity::ExtrusionRole::SupportMaterial => "support material",
            crate::extrusion_entity::ExtrusionRole::SupportMaterialInterface => {
                "support material interface"
            }
            _ => "support material",
        };
        writer.write_comment(&format!("FEATURE: {}", label));
        let _ = extrude_entity(ee, writer, config, is_first_layer);
    }
}

/// Generate a travel move to the specified point.
///
/// C++ reference: GCode::travel_to()
/// GCode.cpp:6416-6572 (~157 lines)
///
/// This function:
/// 1. Decides whether retraction is needed
/// 2. Performs retraction if needed
/// 3. Optionally avoids crossing perimeters
/// 4. Optionally performs Z-hop
/// 5. Generates G0/G1 travel move
/// 6. Optionally performs un-retraction
///
/// # Arguments
/// * `point` - Destination point
/// * `writer` - GCodeWriter to emit commands
/// * `config` - Travel configuration
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors
pub fn travel_to(point: Point, writer: &mut GCodeWriter, config: &TravelConfig) -> Result<()> {
    // Define the travel move as a line between current position and target point
    // GCode.cpp:6418-6420
    // C++: Polyline travel { this->last_pos(), point };
    let current_pos = writer.position();
    let current_pos_scaled = Point::new(
        (current_pos.x * 1_000_000.0) as i64,
        (current_pos.y * 1_000_000.0) as i64,
    );

    // Create travel polyline
    // GCode.cpp:6418-6420
    let travel = Polyline::from_points(vec![current_pos_scaled, point]);

    // Calculate travel distance and check if retraction is needed
    // GCode.cpp:6422-6424
    // C++: bool needs_retraction = this->needs_retraction(travel, role, lift_type);
    let travel_distance = unscale(travel.length() as i64);
    let needs_retraction =
        config.retract_on_travel && travel_distance >= config.retract_length_travel;

    // If reduce_crossing_wall is enabled, try to plan multi-hop path
    // GCode.cpp:6431-6440
    // C++: if (m_config.reduce_crossing_wall && !m_avoid_crossing_perimeters.disabled_once())
    // C++: {
    // C++: travel = m_avoid_crossing_perimeters.travel_to(*this, point, &could_be_wipe_disabled);
    // C++: needs_retraction = this->needs_retraction(travel, role, lift_type);
    // C++: }
    // TODO: Implement avoid_crossing_perimeters integration (GCode.cpp:6431-6440)
    // For now, skip path optimization - use direct travel

    // Perform retraction if needed
    // GCode.cpp:6447-6465
    // C++: if (needs_retraction) {
    // C++: if (m_config.reduce_crossing_wall && could_be_wipe_disabled && !m_last_scarf_seam_flag)
    /// C++: m_wipe.reset_path();
    /// C++: Point last_post_before_retract = this->last_pos();
    /// C++: gcode += this->retract(false, false, lift_type);
    /// C++: ...
    /// C++: } else {
    /// C++: m_wipe.reset_path();
    /// C++: }
    if needs_retraction {
        retract(writer, false)?;
    }

    // Emit travel move(s)
    // GCode.cpp:6475-6562
    // C++: if (travel.size() >= 2) {
    // C++: ...
    // C++: for (size_t i = 1; i < travel.size(); ++i)
    // C++: gcode += m_writer.travel_to_xy(this->point_to_gcode(travel.points[i]), comment, use_short_travel_accel);
    // C++: ...
    // C++: }
    if travel.points().len() >= 2 {
        // Emit travel moves for each segment
        // GCode.cpp:6520-6562
        for i in 1..travel.points().len() {
            let target = travel.points()[i];
            let target_x = unscale(target.x);
            let target_y = unscale(target.y);
            writer.travel_to(target_x, target_y, None);
        }
    }

    Ok(())
}

/// Perform retraction.
///
/// C++ reference: GCode::retract()
/// GCode.cpp:6693-6725 (~33 lines)
///
/// This function:
/// 1. Checks if already retracted (avoid double retraction)
/// 2. Emits retraction command (G1 E-<length> or G10 for firmware retract)
/// 3. Optionally performs wipe move
/// 4. Updates retraction state
///
/// # Arguments
/// * `writer` - GCodeWriter to emit commands
/// * `wipe` - Whether to perform wipe move after retraction
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors
pub fn retract(writer: &mut GCodeWriter, wipe: bool) -> Result<()> {
    // Check if already retracted - no-op if yes
    // GCode.cpp:6695-6697
    // C++: if (m_writer.filament() == nullptr)
    // C++: return gcode;
    if writer.is_retracted() {
        return Ok(());
    }

    // Perform wipe if enabled and wipe path available
    // GCode.cpp:6699-6702
    // C++: if (FILAMENT_CONFIG(wipe) && m_wipe.has_path() && scale_(FILAMENT_CONFIG(wipe_distance)) > SCALED_EPSILON) {
    // C++: gcode += toolchange ? m_writer.retract_for_toolchange(true) : m_writer.retract(true);
    // C++: gcode += m_wipe.wipe(*this, toolchange, is_last_retraction);
    // C++: }
    // TODO: Implement wipe integration (GCode.cpp:6699-6702)
    // For now, skip wipe - just do direct retraction
    let _ = wipe;

    // Call writer's retract method (handles firmware retract or manual retract)
    // GCode.cpp:6707-6708
    // C++: gcode += toolchange ? m_writer.retract_for_toolchange() : m_writer.retract();
    writer.retract();

    // Reset E position after retraction
    // GCode.cpp:6710
    // C++: gcode += m_writer.reset_e();
    // Note: GCodeWriter::retract() already handles E tracking

    // Perform Z-lift if retraction length > 0 or firmware retraction enabled
    // GCode.cpp:6711-6720
    // C++: if (m_writer.filament()->retraction_length() > 0 || m_config.use_firmware_retraction) {
    // C++: if (apply_instantly)
    // C++: gcode += m_writer.eager_lift(lift_type,toolchange);
    // C++: else
    // C++: gcode += m_writer.lazy_lift(lift_type, m_spiral_vase != nullptr, toolchange);
    // C++: }
    // Note: Z-lift is handled by GCodeWriter::retract() which calls do_z_hop()
    Ok(())
}

/// Perform un-retraction (prime).
///
/// C++ reference: GCodeWriter::unretract()
/// GCodeWriter.cpp:808-839 (~32 lines)
///
/// This function:
/// 1. Checks if currently retracted (no-op if not)
/// 2. Emits un-retraction command (G1 E<length> or G11 for firmware)
/// 3. Optionally adds extra restart length
/// 4. Updates retraction state
///
/// # Arguments
/// * `writer` - GCodeWriter to emit commands
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors
pub fn unretract(writer: &mut GCodeWriter) -> Result<()> {
    // Check if currently retracted - no-op if not
    // GCodeWriter.cpp:810-820
    // C++: if (double dE = filament()->unretract(); dE != 0) {
    // C++: if (config.use_firmware_retraction) {
    // C++: gcode += FLAVOR_IS(gcfMachinekit) ? "G23 ;unretract \n" : "G11 ;unretract \n";
    // C++: gcode += reset_e();
    // C++: }
    // C++: else {
    // C++: ...
    // C++: }
    // C++: }
    if !writer.is_retracted() {
        return Ok(());
    }

    // Call writer's unretract method (handles firmware unretract or manual unretract)
    // GCodeWriter.cpp:815-828
    // C++: GCodeG1Formatter w;
    // C++: w.emit_e(filament()->E()+extra_retract);
    // C++: w.emit_f(filament()->deretract_speed() * 60.);
    // C++: w.emit_comment(GCodeWriter::full_gcode_comment, " ; unretract");
    // C++: gcode += w.string();
    writer.unretract();

    Ok(())
}

/// Perform a wipe move.
///
/// C++ reference: Wipe::wipe()
/// GCode.cpp:355-438 (~83 lines)
///
/// This function:
/// 1. Calculates wipe speed (reduced from travel speed)
/// 2. Takes the stored wipe path and clips it to wipe distance
/// 3. Retracts while traveling along wipe path
/// 4. Emits wipe start/end tags for processor
///
/// # Arguments
/// * `writer` - GCodeWriter to emit commands
/// * `wipe_path` - Path to wipe along (from last extrusion)
/// * `wipe_distance` - Target wipe distance (mm)
/// * `retraction_length` - Total retraction length to distribute over wipe
/// * `wipe_speed` - Wipe speed in mm/s
/// * `toolchange` - Whether this is a toolchange wipe
/// * `is_last` - Whether this is the last wipe (affects cooling markers)
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors
pub fn wipe(
    writer: &mut GCodeWriter,
    wipe_path: &Polyline,
    wipe_distance: f64,
    retraction_length: f64,
    wipe_speed: f64,
    _toolchange: bool,
    is_last: bool,
) -> Result<()> {
    // Get retraction length to apply during wipe
    // GCode.cpp:366-373
    // C++: double length = toolchange
    // C++: ? gcodegen.writer().filament()->retract_length_toolchange()
    // C++: : gcodegen.writer().filament()->retraction_length();
    // C++: // Shorten the retraction length by the amount already retracted before wipe.
    // C++: length *= (1. - gcodegen.writer().filament()->retract_before_wipe());
    let length = retraction_length;

    // Only wipe if retraction length is positive
    // GCode.cpp:375
    // C++: if (length >= 0) {
    if length < 1e-6 {
        return Ok(());
    }

    // Calculate wipe distance
    // GCode.cpp:379-380
    // C++: // BBS
    // C++: double wipe_dist = scale_(gcodegen.config().wipe_distance.get_at(gcodegen.writer().filament()->id()));
    let wipe_dist_scaled = scale(wipe_distance);

    // Take the stored wipe path and replace first point with current position
    // GCode.cpp:382-388
    // C++: /* Take the stored wipe path and replace first point with the current actual position
    // C++: (they might be different, for example, in case of loop clipping). */
    // C++: Polyline wipe_path;
    // C++: wipe_path.append(gcodegen.last_pos());
    // C++: wipe_path.append(
    // C++: this->path.points.begin() + 1,
    // C++: this->path.points.end()
    // C++: );
    let mut wipe_polyline = wipe_path.clone();
    if wipe_polyline.points.is_empty() {
        return Ok(());
    }

    // Replace first point with current position
    let current_pos_f = writer.position();
    let current_pos = Point::new(
        (current_pos_f.x * 1_000_000.0) as i64,
        (current_pos_f.y * 1_000_000.0) as i64,
    );
    wipe_polyline.points[0] = current_pos;

    // Clip wipe path to wipe distance
    // GCode.cpp:390
    // C++: wipe_path.clip_end(wipe_path.length() - wipe_dist);
    let total_length = wipe_polyline.length() as i64;
    if total_length > wipe_dist_scaled {
        wipe_polyline.clip_end((total_length - wipe_dist_scaled) as f64);
    }

    // Subdivide the retraction in segments along wipe path
    // GCode.cpp:392-407
    // C++: // subdivide the retraction in segments
    // C++: if (!wipe_path.empty()) {
    if !wipe_polyline.points.is_empty() {
        // Handle short path case
        // GCode.cpp:393-399
        // C++: // BBS. Handle short path case.
        // C++: if (wipe_path.length() < wipe_dist) {
        // C++: wipe_dist = wipe_path.length();
        // C++: //BBS: avoid to divide 0
        // C++: wipe_dist = wipe_dist < EPSILON ? EPSILON : wipe_dist;
        // C++: }
        let actual_wipe_dist = if total_length < wipe_dist_scaled {
            (total_length as f64).max(1e-6) // Avoid division by zero
        } else {
            wipe_dist_scaled as f64
        };

        // Add wipe start tag for processor
        // GCode.cpp:401
        // C++: // add tag for processor
        // C++: gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Start) + "\n";
        writer.write_comment("TYPE:Wipe_Start");

        // Set wipe speed
        // GCode.cpp:402-403
        // C++: //BBS: don't need to enable cooling markers when this is the last wipe. Because no more cooling layer will clean this "_WIPE"
        // C++: gcode += gcodegen.writer().set_speed(wipe_speed * 60, "", (gcodegen.enable_cooling_markers() && !is_last) ? ";_WIPE" : "");
        let comment = if !is_last { ";_WIPE" } else { "" };
        writer.set_speed(wipe_speed * 60.0, comment);

        // Iterate through wipe path segments and retract while traveling
        // GCode.cpp:404-416
        // C++: for (const Line& line : wipe_path.lines()) {
        // C++: double segment_length = line.length();
        // C++: /* Reduce retraction length a bit to avoid effective retraction speed to be greater than the configured one
        // C++: due to rounding (TODO: test and/or better math for this) */
        // C++: double dE = length * (segment_length / wipe_dist) * 0.95;
        // C++: //BBS: fix this FIXME
        // C++: //FIXME one shall not generate the unnecessary G1 Fxxx commands, here wipe_speed is a constant inside this cycle.
        // C++: // Is it here for the cooling markers? Or should it be outside of the cycle?
        // C++: //gcode += gcodegen.writer().set_speed(wipe_speed * 60, "", gcodegen.enable_cooling_markers() ? ";_WIPE" : "");
        /// C++: gcode += gcodegen.writer().extrude_to_xy(
        /// C++: gcodegen.point_to_gcode(line.b),
        /// C++: -dE,
        /// C++: "wipe and retract"
        /// C++: );
        /// C++: }
        for i in 1..wipe_polyline.points.len() {
            let from = wipe_polyline.points[i - 1];
            let to = wipe_polyline.points[i];

            // Calculate segment length
            let segment_length = from.distance_to_f64(to);

            // Calculate retraction for this segment (95% to avoid rounding issues)
            let de = length * (segment_length / actual_wipe_dist) * 0.95;

            // Emit wipe move with negative extrusion (retraction)
            let to_x = unscale(to.x());
            let to_y = unscale(to.y());
            writer.extrude_to_xy(to_x, to_y, -de, Some("wipe and retract"));
        }

        // Add wipe end tag for processor
        // GCode.cpp:418-419
        // C++: // add tag for processor
        // C++: gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_End) + "\n";
        writer.write_comment("TYPE:Wipe_End");

        // Update last position
        // GCode.cpp:420
        // C++: gcodegen.set_last_pos(wipe_path.points.back());
        if let Some(last_point) = wipe_polyline.points.last() {
            let last_x = unscale(last_point.x());
            let last_y = unscale(last_point.y());
            writer.set_position_xy(last_x, last_y);
        }
    }

    // Prevent wiping again on same path (path is reset by caller)
    // GCode.cpp:424
    // C++: // prevent wiping again on same path
    // C++: this->reset_path();
    // Note: Wipe path management is handled by the caller
    Ok(())
}

/// Change to a different extruder (tool change).
///
/// C++ reference: GCode::set_extruder()
/// GCode.cpp:6726-6950 (~224 lines)
///
/// This function orchestrates a complete tool change:
/// 1. Checks if tool change is needed (no-op if same extruder)
/// 2. Performs retraction on old extruder (with optional wipe)
/// 3. Resets wipe path
/// 4. Processes filament end G-code
/// 5. Handles ooze prevention
/// 6. Emits tool change command (T<n>)
/// 7. Processes filament start G-code
/// 8. Handles temperature changes
/// 9. Performs un-retraction on new extruder
/// 10. Updates active extruder state
///
/// # Arguments
/// * `new_extruder_id` - Target extruder ID (0-based)
/// * `writer` - GCodeWriter to emit commands
/// * `print_z` - Current Z height for temperature decisions
/// * `config` - Print configuration for tool change settings
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors or invalid extruder ID
pub fn set_extruder(
    new_extruder_id: usize,
    writer: &mut GCodeWriter,
    _print_z: f64,
    _config: &PrintConfig,
) -> Result<()> {
    // Check if tool change is needed
    // GCode.cpp:6728-6729
    // C++: int new_extruder_id = get_extruder_id(new_filament_id);
    // C++: if (!m_writer.need_toolchange(new_filament_id))
    // C++: return "";
    if !writer.need_toolchange(new_extruder_id) {
        return Ok(());
    }

    // Single extruder setup - just set extruder and return
    // GCode.cpp:6731-6761
    // C++: // if we are running a single-extruder setup, just set the extruder and return nothing
    // C++: if (!m_writer.multiple_extruders) {
    // C++: m_placeholder_parser.set("current_extruder", new_filament_id);
    // C++: ...
    // C++: gcode += m_writer.toolchange(new_filament_id);
    // C++: return gcode;
    // C++: }
    if !writer.has_multiple_extruders() {
        // Single extruder - just emit T command and update state
        writer.write_command_with_comment(&format!("T{}", new_extruder_id), Some("tool change"));
        writer.set_extruder(new_extruder_id);
        return Ok(());
    }

    // Multi-extruder setup - full tool change sequence
    // GCode.cpp:6763-6765
    // C++: // BBS. Should be placed before retract.
    // C++: m_toolchange_count++;
    // Tool change counter would be tracked here

    // Prepend retraction on the current extruder
    // GCode.cpp:6767
    // C++: // prepend retraction on the current extruder
    // C++: std::string gcode = this->retract(true, false);
    retract(writer, true)?; // true = with wipe

    // Reset wipe path to avoid reusing it
    // GCode.cpp:6770
    // C++: // Always reset the extrusion path, even if the tool change retract is set to zero.
    // C++: m_wipe.reset_path();
    // Note: Wipe path management is handled by caller

    // Insert skip object labels for sequential printing
    // GCode.cpp:6772-6776
    // C++: // BBS: insert skip object label before change filament while by object
    // C++: if (by_object)
    // C++: m_writer.add_object_change_labels(gcode);
    // C++: else
    // C++: m_writer.add_object_end_labels(gcode);
    // TODO: Implement object change labels for sequential printing

    /// Process filament end G-code if current filament exists
    /// GCode.cpp:6778-6794
    /// C++: bool add_change_filament_624 = false;
    /// C++: if (m_writer.filament() != nullptr) {
    /// C++: // Process the custom filament_end_gcode. set_extruder() is only called if there is no wipe tower
    /// C++: // so it should not be injected twice.
    /// C++: unsigned int old_filament_id = m_writer.filament()->id();
    /// C++: const std::string &filament_end_gcode = m_config.filament_end_gcode.get_at(old_filament_id);
    /// C++: if (! filament_end_gcode.empty()) {
    /// C++: ...
    /// C++: }
    /// C++: }
    // TODO: Process filament_end_gcode custom G-code

    /// Handle ooze prevention (park extruder at standby position)
    /// GCode.cpp:6797-6798
    /// C++: // If ooze prevention is enabled, park current extruder in the nearest
    /// C++: // standby point and set it to the standby temperature.
    /// C++: if (m_ooze_prevention.enable && m_writer.filament() != nullptr)
    /// C++: gcode += m_ooze_prevention.pre_toolchange(*this);
    // TODO: Implement ooze prevention pre-toolchange parking

    /// Calculate flush/wipe volumes and temperatures
    /// GCode.cpp:6800-6880
    /// C++: // BBS
    /// C++: float new_retract_length = m_config.retraction_length.get_at(new_filament_id);
    /// C++: float new_retract_length_toolchange = m_config.retract_length_toolchange.get_at(new_filament_id);
    /// C++: ...
    /// C++: wipe_volume = flush_matrix[old_filament_id * number_of_extruders + new_filament_id];
    /// C++: wipe_volume *= m_config.flush_multiplier.get_at(new_extruder_id);
    // TODO: Calculate flush volumes from flush matrix

    /// Process change_filament_gcode with placeholder substitution
    /// GCode.cpp:6882-6920
    /// C++: dyn_config.set_key_value("outer_wall_volumetric_speed", new ConfigOptionFloat(outer_wall_volumetric_speed));
    /// C++: dyn_config.set_key_value("previous_extruder", new ConfigOptionInt(old_filament_id));
    /// C++: dyn_config.set_key_value("next_extruder", new ConfigOptionInt((int)new_filament_id));
    /// C++: ...
    /// C++: gcode += this->placeholder_parser_process("change_filament_gcode", change_filament_gcode, new_filament_id, &dyn_config);
    // TODO: Process change_filament_gcode with dynamic config placeholders

    /// Emit tool change command
    /// GCode.cpp:6926
    /// C++: gcode += m_writer.toolchange(new_filament_id);
    writer.write_command_with_comment(&format!("T{}", new_extruder_id), Some("tool change"));

    // Handle ooze prevention post-toolchange
    // GCode.cpp:6929-6931
    // C++: // append custom toolchange gcode
    // C++: if (m_ooze_prevention.enable && m_writer.filament() != nullptr)
    // C++: gcode += m_ooze_prevention.post_toolchange(*this);
    // TODO: Implement ooze prevention post-toolchange

    // Process filament_start_gcode for new extruder
    // GCode.cpp:6933-6947
    // C++: // Append the filament start G-code.
    // C++: const std::string &filament_start_gcode = m_config.filament_start_gcode.get_at(new_filament_id);
    // C++: if (! filament_start_gcode.empty()) {
    // C++: // Process the filament_start_gcode for the filament.
    /// C++: DynamicConfig config;
    /// C++: ...
    /// C++: gcode += this->placeholder_parser_process("filament_start_gcode", filament_start_gcode, new_filament_id, &config);
    /// C++: check_add_eol(gcode);
    /// C++: }
    // TODO: Process filament_start_gcode custom G-code

    /// Update active extruder state
    /// GCode.cpp:6949
    /// C++: return gcode;
    writer.set_extruder(new_extruder_id);

    Ok(())
}

/// Apply cooling adjustments to a layer.
///
/// C++ reference: CoolingBuffer::process()
/// GCode/CoolingBuffer.cpp:200-400 (~200 lines)
///
/// This function:
/// 1. Calculates actual layer print time
/// 2. Determines if slowdown is needed
/// 3. Adjusts speeds to meet minimum layer time
/// 4. Calculates appropriate fan speed
///
/// # Arguments
/// * `writer` - GCodeWriter to apply speed adjustments
/// * `cooling_buffer` - CoolingBuffer with configuration
/// * `layer_time` - Estimated layer time (seconds)
/// * `layer_index` - Current layer number (0-based)
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on processing errors
pub fn apply_layer_cooling(
    writer: &mut GCodeWriter,
    cooling_buffer: &CoolingBuffer,
    layer_time: f64,
    layer_index: usize,
) -> Result<()> {
    // Check if cooling is needed based on minimum layer time
    // CoolingBuffer.cpp:250-260
    // C++: float CoolingBuffer::calculate_layer_slowdown(
    // C++: std::vector<PerExtruderAdjustments> &per_extruder_adjustments) {
    // C++: float layer_time_stretched = 0.f;
    // C++: ...
    // C++: }
    let config = cooling_buffer.config();

    // Skip cooling for first layer if configured
    // CoolingBuffer.cpp:255-257
    // C++: if (m_layer_id < config.disable_fan_first_layers.value) {
    // C++: return layer_time;
    // C++: }
    if layer_index < config.disable_fan_first_layers as usize {
        return Ok(());
    }

    // Calculate fan speed based on layer time
    // CoolingBuffer.cpp:280-300
    // C++: unsigned int fan_speed = 0;
    // C++: if (layer_time < config.fan_below_layer_time.value) {
    // C++: fan_speed = config.fan_speed.value;
    // C++: }
    let fan_speed = cooling_buffer.calculate_fan_speed(layer_index as u32, layer_time);
    let fan_speed_pwm = (fan_speed * 255.0) as u32;

    // Emit fan speed command
    // CoolingBuffer.cpp:305-310
    // C++: if (fan_speed != current_fan_speed) {
    // C++: gcode += m_gcodegen->writer().set_fan(fan_speed);
    // C++: current_fan_speed = fan_speed;
    // C++: }
    writer.set_fan_speed(fan_speed_pwm);

    // TODO : Implement speed slowdown
    // CoolingBuffer.cpp:320-380
    // C++: if (layer_time < config.min_layer_time.value) {
    // C++: // Calculate slowdown factor
    // C++: float slowdown_factor = config.min_layer_time.value / layer_time;
    // C++: // Apply slowdown to adjustable moves
    // C++: ...
    // C++: }
    // For , we only implement fan control
    // Speed slowdown will be added
    Ok(())
}

/// Calculate fan speed for bridge features.
///
/// C++ reference: CoolingBuffer::bridge_fan_speed()
/// GCode/CoolingBuffer.cpp:450-470
///
/// # Arguments
/// * `cooling_buffer` - CoolingBuffer with configuration
///
/// # Returns
/// * Fan speed (0.0 - 1.0)
pub fn bridge_fan_speed(cooling_buffer: &CoolingBuffer) -> f64 {
    // Return bridge fan speed from config
    // CoolingBuffer.cpp:452-454
    // C++: float CoolingBuffer::bridge_fan_speed() const {
    // C++: return m_config.bridge_fan_speed.value;
    // C++: }
    cooling_buffer.bridge_fan_speed().unwrap_or(0.0)
}

/// Calculate fan speed for overhang features.
///
/// C++ reference: CoolingBuffer::overhang_fan_speed()
/// GCode/CoolingBuffer.cpp:475-495
///
/// # Arguments
/// * `cooling_buffer` - CoolingBuffer with configuration
///
/// # Returns
/// * Fan speed (0.0 - 1.0)
pub fn overhang_fan_speed(cooling_buffer: &CoolingBuffer) -> f64 {
    // Return overhang fan speed from config
    // CoolingBuffer.cpp:477-479
    // C++: float CoolingBuffer::overhang_fan_speed() const {
    // C++: return m_config.overhang_fan_speed.value;
    // C++: }
    cooling_buffer.overhang_fan_speed().unwrap_or(0.0)
}

/// Set fan speed with optional override for special features.
///
/// C++ reference: GCode helper methods
/// GCode.cpp:various locations
///
/// # Arguments
/// * `writer` - GCodeWriter to emit fan command
/// * `base_fan_speed` - Base fan speed (0.0 - 1.0)
/// * `role` - Extrusion role (may trigger override)
/// * `cooling_buffer` - CoolingBuffer for override config
///
/// # Returns
/// * `Ok(())` on success
pub fn set_fan_speed_for_role(
    writer: &mut GCodeWriter,
    base_fan_speed: f64,
    role: ExtrusionRole,
    cooling_buffer: &CoolingBuffer,
) -> Result<()> {
    // Check for bridge override
    // GCode.cpp:various - bridge detection
    // C++: if (entity.role() == erBridgeInfill && config.bridge_fan_override) {
    // C++: fan_speed = config.bridge_fan_speed;
    // C++: }
    let fan_speed = match role {
        ExtrusionRole::BridgeInfill => {
            if cooling_buffer.config().bridge_fan_override {
                cooling_buffer.bridge_fan_speed().unwrap_or(base_fan_speed)
            } else {
                base_fan_speed
            }
        }
        ExtrusionRole::OverhangPerimeter => {
            if cooling_buffer.config().overhang_fan_override {
                cooling_buffer
                    .overhang_fan_speed()
                    .unwrap_or(base_fan_speed)
            } else {
                base_fan_speed
            }
        }
        _ => base_fan_speed,
    };

    // Convert to PWM (0-255) and emit
    // GCodeWriter.cpp:860-900
    // C++: std::string GCodeWriter::set_fan(unsigned int speed) {
    // C++: gcode << "M106 S" << 255.0 * speed / 100.0;
    // C++: }
    let fan_speed_pwm = (fan_speed * 255.0) as u32;
    writer.set_fan_speed(fan_speed_pwm);

    Ok(())
}

// ---------------------------------------------------------------------------
// Layer orchestration dispatch functions
// Ports of GCode::change_layer(), extrude_perimeters(), extrude_infill(),
// extrude_support() from BambuStudio GCode.cpp
// ---------------------------------------------------------------------------

/// State tracked across layers during G-code export.
/// Mirrors key fields of the C++ GCode class that persist across process_layer() calls.
///
/// C++ reference: GCode.hpp member variables
#[derive(Debug, Clone)]
pub struct GCodeLayerState {
    /// Current layer index (incremented by change_layer).
    /// C++ GCode.hpp: m_layer_index
    pub layer_index: usize,
    /// Total layer count for progress reporting.
    /// C++ GCode.hpp: m_layer_count
    pub layer_count: usize,
    /// Last layer Z height (for computing layer height delta).
    /// C++ GCode.hpp: m_last_layer_z
    pub last_layer_z: f64,
    /// Maximum Z reached so far.
    /// C++ GCode.hpp: m_max_layer_z
    pub max_layer_z: f64,
    /// Last layer height (for HEIGHT tag).
    /// C++ GCode.hpp: m_last_height
    pub last_height: f64,
    /// Whether second-layer setup has been done (temp transitions, etc).
    /// C++ GCode.hpp: m_second_layer_things_done
    pub second_layer_things_done: bool,
    /// Whether spiral vase mode is active.
    /// C++ GCode.hpp: m_spiral_vase
    pub spiral_vase: bool,
    /// Whether to enable loop clipping (seam gap).
    /// C++ GCode.hpp: m_enable_loop_clipping
    pub enable_loop_clipping: bool,
    /// Set of object label IDs printed on the current layer.
    /// Used for M624/M625 emission.
    pub layer_object_label_ids: Vec<usize>,
    /// Whether change_layer lift is pending (deferred Z move).
    /// C++ GCode.hpp: m_need_change_layer_lift_z
    pub need_change_layer_lift_z: bool,
    /// Nominal Z position.
    /// C++ GCode.hpp: m_nominal_z
    pub nominal_z: f64,
}

impl Default for GCodeLayerState {
    fn default() -> Self {
        Self {
            layer_index: 0,
            layer_count: 0,
            last_layer_z: 0.0,
            max_layer_z: 0.0,
            last_height: 0.0,
            second_layer_things_done: false,
            spiral_vase: false,
            enable_loop_clipping: true,
            layer_object_label_ids: Vec::new(),
            need_change_layer_lift_z: false,
            nominal_z: 0.0,
        }
    }
}

/// Emit layer-change G-code: progress update, optional retract, Z move.
///
/// C++ reference: GCode::change_layer()
/// GCode.cpp:4904-4940
///
/// This function:
/// 1. Increments progress counter and emits M73
/// 2. Optionally retracts if retract_when_changing_layer is set
/// 3. In spiral vase mode, does immediate Z travel; otherwise defers Z move
/// 4. Updates nominal_z
///
/// # Arguments
/// * `print_z` - Target Z height for this layer
/// * `writer` - GCodeWriter to emit commands
/// * `state` - Mutable layer state (layer_index is incremented)
/// * `config` - Print configuration
///
/// # Returns
/// G-code string for the layer change
pub fn change_layer(
    print_z: f64,
    writer: &mut GCodeWriter,
    state: &mut GCodeLayerState,
    _config: &crate::print_config::PrintConfig,
) -> String {
    let mut gcode = String::new();

    // C++ GCode.cpp:4908-4910
    // Increment progress and emit M73 progress command
    if state.layer_count > 0 {
        state.layer_index += 1;
        let pct = (state.layer_index as f64 / state.layer_count as f64 * 100.0).min(100.0) as u32;
        // Estimate remaining time proportionally (simple linear estimate)
        gcode += &format!("M73 P{} R0\n", pct);
    }

    // C++ GCode.cpp:4913
    // BBS: no z_offset applied
    let z = print_z;

    // C++ GCode.cpp:4914-4918
    // Retract when changing layer if configured
    // C++ uses FILAMENT_CONFIG(retract_when_changing_layer) — per-filament setting.
    // For now, always retract on layer change if Z increases (matching default behavior).
    if writer.z() < z {
        writer.retract();
    }

    // C++ GCode.cpp:4922-4931
    if state.spiral_vase {
        // In spiral vase mode, travel to Z immediately
        // C++ GCode.cpp:4924-4926
        gcode += &format!("; move to next layer ({})\n", state.layer_index);
        writer.travel_to_z(z, None);
    } else {
        // C++ GCode.cpp:4930
        // Defer Z move — it will happen on next travel_to()
        state.need_change_layer_lift_z = true;
    }

    // C++ GCode.cpp:4933
    state.nominal_z = print_z;

    gcode
}

/// Emit the CHANGE_LAYER tag, Z_HEIGHT, HEIGHT metadata, and custom gcode.
///
/// C++ reference: GCode::process_layer() lines 3922-3983
/// This is the per-layer preamble that runs before change_layer().
///
/// # Arguments
/// * `print_z` - Z height of this layer
/// * `state` - Mutable layer state
/// * `config` - Print configuration
///
/// # Returns
/// G-code string for layer preamble
pub fn emit_layer_preamble(
    print_z: f64,
    state: &mut GCodeLayerState,
    config: &crate::print_config::PrintConfig,
) -> String {
    let mut gcode = String::new();

    // C++ GCode.cpp:3923
    // Add tag for processor
    gcode += "; CHANGE_LAYER\n";

    // C++ GCode.cpp:3925-3927
    // Export layer z
    gcode += &format!("; Z_HEIGHT: {}\n", print_z);

    // C++ GCode.cpp:3929-3931
    // Export layer height
    let first_layer = state.layer_index == 0 && state.last_layer_z.abs() < 1e-6;
    let height = if first_layer {
        print_z as f32
    } else {
        (print_z as f32) - (state.last_layer_z as f32)
    };
    gcode += &format!(";HEIGHT:{}\n", height);

    // C++ GCode.cpp:3933-3934
    // Update caches
    state.last_layer_z = print_z;
    state.max_layer_z = state.max_layer_z.max(print_z);
    state.last_height = height as f64;

    // C++ GCode.cpp:3938-3946
    // Before layer change custom G-code
    if !config.before_layer_change_gcode.is_empty() {
        let processed = config
            .before_layer_change_gcode
            .replace("{layer_num}", &(state.layer_index + 1).to_string())
            .replace("{layer_z}", &format!("{}", print_z))
            .replace("{max_layer_z}", &format!("{}", state.max_layer_z));
        gcode += &processed;
        gcode += "\n";
    }

    gcode
}

/// Emit post-change_layer custom G-code and fan speed marker.
///
/// C++ reference: GCode::process_layer() lines 3972-3983
///
/// # Arguments
/// * `print_z` - Z height of this layer
/// * `state` - Layer state
/// * `config` - Print configuration
///
/// # Returns
/// G-code string
pub fn emit_layer_postamble(
    print_z: f64,
    state: &GCodeLayerState,
    config: &crate::print_config::PrintConfig,
) -> String {
    let mut gcode = String::new();

    // C++ GCode.cpp:3972-3981
    // After layer change custom G-code
    if !config.layer_change_gcode.is_empty() {
        let processed = config
            .layer_change_gcode
            .replace("{layer_num}", &state.layer_index.to_string())
            .replace("{layer_z}", &format!("{}", print_z))
            .replace("{max_layer_z}", &format!("{}", state.max_layer_z));
        gcode += &processed;
        gcode += "\n";
    }

    // C++ GCode.cpp:3983
    // Set layer time fan speed marker for cooling post-processor
    gcode += ";_SET_FAN_SPEED_CHANGING_LAYER\n";

    gcode
}

/// Encode a list of object label IDs to a base64-like string for M624.
///
/// C++ reference: GCode::_encode_label_ids_to_base64()
/// GCode.cpp (helper function)
///
/// Each label ID is encoded as a character in a compact format.
/// For simplicity, we encode as comma-separated decimal in a string.
fn encode_label_ids_to_base64(ids: &[usize]) -> String {
    // C++ uses a custom base64-like encoding.
    // For compatibility we use a simplified approach that encodes IDs as
    // a compact string. The firmware (M624) just needs a parseable token.
    if ids.is_empty() {
        return String::new();
    }

    // Simple encoding: base64 of the bit-set of object IDs
    // For up to 64 objects, pack into a u64 bitmask then base64-encode
    let mut bitmask: u64 = 0;
    for &id in ids {
        if id < 64 {
            bitmask |= 1u64 << id;
        }
    }

    // Encode as base64 characters (6 bits each)
    const BASE64_CHARS: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut result = String::new();
    let mut val = bitmask;
    if val == 0 {
        result.push(BASE64_CHARS[0] as char);
    } else {
        while val > 0 {
            result.push(BASE64_CHARS[(val & 0x3F) as usize] as char);
            val >>= 6;
        }
    }
    result
}

/// Emit M624 start label for an object instance.
///
/// C++ reference: GCode::process_layer() lines 4591-4602
///
/// # Arguments
/// * `label_object_id` - Unique label ID for the object instance
/// * `enable_label_object` - Whether M624/M625 labeling is enabled
///
/// # Returns
/// G-code string with start label
pub fn emit_object_start_label(label_object_id: usize, enable_label_object: bool) -> String {
    let mut gcode = String::new();

    // C++ GCode.cpp:4589
    gcode += &format!("; OBJECT_ID: {}\n", label_object_id);

    if enable_label_object {
        // C++ GCode.cpp:4592-4595
        gcode += &format!(
            "; start printing object, unique label id: {}\n",
            label_object_id
        );
        // C++ GCode.cpp:4594
        gcode += &format!("M624 {}\n", encode_label_ids_to_base64(&[label_object_id]));
    }

    gcode
}

/// Emit M625 end label for an object instance.
///
/// C++ reference: GCode::process_layer() lines 4753-4758
///
/// # Arguments
/// * `label_object_id` - Unique label ID for the object instance
/// * `enable_label_object` - Whether M624/M625 labeling is enabled
///
/// # Returns
/// G-code string with end label
pub fn emit_object_end_label(label_object_id: usize, enable_label_object: bool) -> String {
    let mut gcode = String::new();

    if enable_label_object {
        // C++ GCode.cpp:4754-4756
        gcode += &format!(
            "; stop printing object, unique label id: {}\n",
            label_object_id
        );
        gcode += "M625\n";
    }
    let _ = label_object_id; // suppress warning when labels disabled

    gcode
}

/// Emit M624/M625 wrapping around the entire layer's timelapse position.
///
/// C++ reference: GCode::process_layer() lines 4395-4407
///
/// # Arguments
/// * `object_label_ids` - Set of object IDs present on this layer
/// * `layer_index` - Current layer index
///
/// # Returns
/// Pair of (start_gcode, end_gcode) strings
pub fn emit_layer_object_labels(
    object_label_ids: &[usize],
    layer_index: usize,
) -> (String, String) {
    if object_label_ids.is_empty() {
        return (String::new(), String::new());
    }

    // C++ GCode.cpp:4395-4401
    let ids_str = object_label_ids
        .iter()
        .map(|id| id.to_string())
        .collect::<Vec<_>>()
        .join(",");

    let start = format!(
        "; object ids of layer {} start: {}\nM624 {}\n",
        layer_index + 1,
        ids_str,
        encode_label_ids_to_base64(object_label_ids),
    );

    // C++ GCode.cpp:4403-4404
    let end = format!(
        "; object ids of this layer{} end: {}\nM625\n",
        layer_index + 1,
        ids_str,
    );

    (start, end)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::extrusion_entity::{ExtrusionLoopRole, ExtrusionRole};
    use crate::geometry::{Point, Polyline};

    /// Helper to create a simple square loop for testing.
    /// Creates a 10mm x 10mm square starting at (5,5).
    fn create_square_loop() -> ExtrusionLoop {
        // Create a square: (5,5) -> (15,5) -> (15,15) -> (5,15) -> back to (5,5)
        let points = vec![
            Point::new_scale(5.0, 5.0),
            Point::new_scale(15.0, 5.0),
            Point::new_scale(15.0, 15.0),
            Point::new_scale(5.0, 15.0),
        ];

        let mut path = ExtrusionPath::new(ExtrusionRole::ExternalPerimeter);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.1; // 0.1 mm³/mm extrusion
        path.width = 0.4;
        path.height = 0.2;

        ExtrusionLoop::new(vec![path], ExtrusionLoopRole::Default)
    }

    /// Helper to create a simple path for testing
    fn create_simple_path() -> ExtrusionPath {
        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];

        let mut path = ExtrusionPath::new(ExtrusionRole::Perimeter);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.05;
        path.width = 0.4;
        path.height = 0.2;

        path
    }

    #[test]
    fn test_extrude_simple_square_loop() {
        let mut writer = GCodeWriter::new();
        let loop_entity = create_square_loop();

        // Get initial E value
        let initial_e = writer.e();

        // Extrude the loop
        extrude_loop(&loop_entity, &mut writer).expect("extrude_loop should succeed");

        // Get final E value
        let final_e = writer.e();

        // E should have increased (we extruded material)
        assert!(
            final_e > initial_e,
            "E value should increase after extrusion: {} -> {}",
            initial_e,
            final_e
        );

        // Get the generated G-code
        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should contain G1 commands (extrusion moves)
        assert!(content.contains("G1"), "G-code should contain G1 commands");

        // Should contain E values (extrusion)
        assert!(
            content.contains(" E"),
            "G-code should contain extrusion (E) values"
        );

        // Should have multiple lines (one per segment)
        let lines: Vec<&str> = content.lines().collect();
        assert!(
            lines.len() >= 4,
            "Should have at least 4 move commands for square (got {})",
            lines.len()
        );
    }

    #[test]
    fn test_extrude_loop_seam_placement() {
        let mut writer = GCodeWriter::new();

        // Set writer to a known position closer to (15,15) corner
        writer.set_x(15.0);
        writer.set_y(15.0);

        let loop_entity = create_square_loop();

        // Extrude the loop
        extrude_loop(&loop_entity, &mut writer).expect("extrude_loop should succeed");

        // The loop should have been rotated to start near the writer's position
        // We can't easily verify the exact starting point without inspecting internals,
        // but we can verify that extrusion happened successfully
        let final_e = writer.e();
        assert!(final_e > 0.0, "Should have extruded material");
    }

    #[test]
    fn test_extrude_loop_ccw_orientation() {
        let mut writer = GCodeWriter::new();

        // Create a clockwise loop (will be reversed to CCW)
        let points = vec![
            Point::new_scale(0.0, 0.0),
            Point::new_scale(0.0, 10.0), // Clockwise: up first
            Point::new_scale(10.0, 10.0),
            Point::new_scale(10.0, 0.0),
        ];

        let mut path = ExtrusionPath::new(ExtrusionRole::ExternalPerimeter);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.1;
        path.width = 0.4;
        path.height = 0.2;

        let loop_entity = ExtrusionLoop::new(vec![path], ExtrusionLoopRole::Default);

        let initial_e = writer.e();

        // Should succeed and make loop CCW
        extrude_loop(&loop_entity, &mut writer).expect("extrude_loop should succeed");

        let final_e = writer.e();
        assert!(final_e > initial_e, "Should have extruded material");
    }

    #[test]
    fn test_extrude_path_empty() {
        let mut writer = GCodeWriter::new();

        // Create an empty path
        let path = ExtrusionPath::new(ExtrusionRole::Perimeter);

        let initial_e = writer.e();

        // Should not panic, just do nothing
        let config = crate::print_config::PrintObjectConfig::default();
        extrude_path(&path, &mut writer, &config, false);

        let final_e = writer.e();

        // E should not change for empty path
        assert_eq!(final_e, initial_e, "E should not change for empty path");

        // G-code should be empty or unchanged
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.is_empty() || !content.contains("G1"),
            "Should not generate moves for empty path"
        );
    }

    #[test]
    fn test_extrude_path_single_segment() {
        let mut writer = GCodeWriter::new();

        // Create a path with two points (one segment)
        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];

        let mut path = ExtrusionPath::new(ExtrusionRole::Perimeter);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.1;
        path.width = 0.4;
        path.height = 0.2;

        let initial_e = writer.e();

        let config = crate::print_config::PrintObjectConfig::default();
        extrude_path(&path, &mut writer, &config, false);

        let final_e = writer.e();

        // Should have extruded for the 10mm segment
        assert!(final_e > initial_e, "E should increase for single segment");

        // Calculate expected E delta: length * mm3_per_mm = 10.0 * 0.1 = 1.0
        let expected_delta = 10.0 * 0.1;
        let actual_delta = final_e - initial_e;

        // Allow small floating-point tolerance
        assert!(
            (actual_delta - expected_delta).abs() < 0.01,
            "E delta should be approximately {}, got {}",
            expected_delta,
            actual_delta
        );

        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should have exactly one G1 command
        let g1_count = content.matches("G1").count();
        assert_eq!(
            g1_count, 1,
            "Should have exactly 1 G1 command for single segment, got {}",
            g1_count
        );
    }

    #[test]
    fn test_extrude_path_multi_segment() {
        let mut writer = GCodeWriter::new();

        // Create a path with 4 points (3 segments) forming an L shape
        let points = vec![
            Point::new_scale(0.0, 0.0),
            Point::new_scale(10.0, 0.0), // 10mm horizontal
            Point::new_scale(10.0, 5.0), // 5mm vertical
            Point::new_scale(20.0, 5.0), // 10mm horizontal
        ];

        let mut path = ExtrusionPath::new(ExtrusionRole::Infill);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.08;
        path.width = 0.4;
        path.height = 0.2;

        let initial_e = writer.e();

        let config = crate::print_config::PrintObjectConfig::default();
        extrude_path(&path, &mut writer, &config, false);

        let final_e = writer.e();

        // Total length: 10 + 5 + 10 = 25mm
        // Expected E delta: 25 * 0.08 = 2.0
        let expected_delta = 25.0 * 0.08;
        let actual_delta = final_e - initial_e;

        assert!(
            (actual_delta - expected_delta).abs() < 0.01,
            "E delta should be approximately {}, got {}",
            expected_delta,
            actual_delta
        );

        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should have 3 G1 commands (one per segment)
        let g1_count = content.matches("G1").count();
        assert_eq!(
            g1_count, 3,
            "Should have exactly 3 G1 commands for 3 segments, got {}",
            g1_count
        );
    }

    #[test]
    fn test_coordinate_conversion() {
        let mut writer = GCodeWriter::new();

        // Create a path with known scaled coordinates
        let points = vec![Point::new_scale(5.5, 7.3), Point::new_scale(15.8, 12.1)];

        let mut path = ExtrusionPath::new(ExtrusionRole::Perimeter);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.1;
        path.width = 0.4;
        path.height = 0.2;

        let config = crate::print_config::PrintObjectConfig::default();
        extrude_path(&path, &mut writer, &config, false);

        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Verify that coordinates were properly unscaled to mm
        // Should contain X15.800 and Y12.100 (or similar precision)
        assert!(
            content.contains("X15.8") || content.contains("X15.80"),
            "Should contain unscaled X coordinate ~15.8"
        );
        assert!(
            content.contains("Y12.1") || content.contains("Y12.10"),
            "Should contain unscaled Y coordinate ~12.1"
        );
    }

    #[test]
    fn test_extrude_path_cumulative_e() {
        let mut writer = GCodeWriter::new();

        // Extrude multiple paths and verify E accumulates
        let path1 = create_simple_path();
        let path2 = create_simple_path();

        let e0 = writer.e();
        let config = crate::print_config::PrintObjectConfig::default();
        extrude_path(&path1, &mut writer, &config, false);
        let e1 = writer.e();
        extrude_path(&path2, &mut writer, &config, false);
        let e2 = writer.e();

        // Each path should increase E
        assert!(e1 > e0, "First path should increase E");
        assert!(e2 > e1, "Second path should increase E further");

        // E should be cumulative (absolute mode)
        let delta1 = e1 - e0;
        let delta2 = e2 - e1;

        // Both deltas should be approximately equal (same path)
        assert!(
            (delta1 - delta2).abs() < 0.01,
            "Both paths should extrude similar amounts: {} vs {}",
            delta1,
            delta2
        );
    }

    // === Tests for extrude_collection() and extrude_entity() ===

    #[test]
    fn test_extrude_collection_empty() {
        let mut writer = GCodeWriter::new();

        // Empty collection should not panic
        let collection = ExtrusionEntityCollection {
            entities: Vec::new(),
            no_sort: false,
            orig_indices: Vec::new(),
        };

        let result = extrude_collection(&collection, &mut writer);
        assert!(result.is_ok(), "Empty collection should succeed");

        // Should not generate any G-code
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.is_empty() || !content.contains("G1"),
            "Empty collection should not generate moves"
        );
    }

    #[test]
    fn test_extrude_collection_single_path() {
        let mut writer = GCodeWriter::new();

        // Create collection with single path
        let path = create_simple_path();
        let collection = ExtrusionEntityCollection {
            entities: vec![ExtrusionEntityType::Path(path)],
            no_sort: false,
            orig_indices: Vec::new(),
        };

        let e_before = writer.e();
        extrude_collection(&collection, &mut writer).expect("Should extrude single path");
        let e_after = writer.e();

        // Should have extruded
        assert!(e_after > e_before, "E should increase");

        // Should have feature comment
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(content.contains("FEATURE:"), "Should have feature comment");
    }

    #[test]
    fn test_extrude_collection_multiple_paths() {
        let mut writer = GCodeWriter::new();

        // Create collection with 3 paths
        let path1 = create_simple_path();
        let path2 = create_simple_path();
        let path3 = create_simple_path();

        let collection = ExtrusionEntityCollection {
            entities: vec![
                ExtrusionEntityType::Path(path1),
                ExtrusionEntityType::Path(path2),
                ExtrusionEntityType::Path(path3),
            ],
            no_sort: false,
            orig_indices: Vec::new(),
        };

        let e_before = writer.e();
        extrude_collection(&collection, &mut writer).expect("Should extrude all paths");
        let e_after = writer.e();

        // E should increase significantly
        assert!(e_after > e_before, "E should increase for multiple paths");

        // Should have multiple G1 commands (one per path segment)
        let gcode = writer.get_gcode();
        let content = gcode.content();
        let g1_count = content.matches("G1").count();
        assert!(g1_count >= 3, "Should have at least 3 G1 commands");
    }

    #[test]
    fn test_extrude_collection_with_loop() {
        let mut writer = GCodeWriter::new();

        // Create collection with a loop
        let loop_entity = create_square_loop();
        let collection = ExtrusionEntityCollection {
            entities: vec![ExtrusionEntityType::Loop(loop_entity)],
            no_sort: false,
            orig_indices: Vec::new(),
        };

        let e_before = writer.e();
        extrude_collection(&collection, &mut writer).expect("Should extrude loop");
        let e_after = writer.e();

        // Should have extruded
        assert!(e_after > e_before, "E should increase");

        // Should have feature comment
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(content.contains("FEATURE:"), "Should have feature comment");
    }

    #[test]
    fn test_extrude_collection_nested() {
        let mut writer = GCodeWriter::new();

        // Create nested collection
        let path = create_simple_path();
        let inner_collection = ExtrusionEntityCollection {
            entities: vec![ExtrusionEntityType::Path(path)],
            no_sort: false,
            orig_indices: Vec::new(),
        };

        let outer_collection = ExtrusionEntityCollection {
            entities: vec![ExtrusionEntityType::Collection(Box::new(inner_collection))],
            no_sort: false,
            orig_indices: Vec::new(),
        };

        let e_before = writer.e();
        extrude_collection(&outer_collection, &mut writer)
            .expect("Should handle nested collections");
        let e_after = writer.e();

        // Should have extruded inner path
        assert!(
            e_after > e_before,
            "E should increase for nested collection"
        );
    }

    #[test]
    fn test_extrude_entity_path() {
        let mut writer = GCodeWriter::new();
        let path = create_simple_path();
        let entity = ExtrusionEntityType::Path(path);

        let e_before = writer.e();
        extrude_entity(&entity, &mut writer).expect("Should extrude path entity");
        let e_after = writer.e();

        assert!(e_after > e_before, "Path entity should extrude");
    }

    #[test]
    fn test_extrude_entity_loop() {
        let mut writer = GCodeWriter::new();
        let loop_entity = create_square_loop();
        let entity = ExtrusionEntityType::Loop(loop_entity);

        let e_before = writer.e();
        extrude_entity(&entity, &mut writer).expect("Should extrude loop entity");
        let e_after = writer.e();

        assert!(e_after > e_before, "Loop entity should extrude");
    }

    #[test]
    fn test_extrude_entity_collection() {
        let mut writer = GCodeWriter::new();

        let path = create_simple_path();
        let collection = ExtrusionEntityCollection {
            entities: vec![ExtrusionEntityType::Path(path)],
            no_sort: false,
            orig_indices: Vec::new(),
        };
        let entity = ExtrusionEntityType::Collection(Box::new(collection));

        let e_before = writer.e();
        extrude_entity(&entity, &mut writer).expect("Should extrude collection entity");
        let e_after = writer.e();

        assert!(e_after > e_before, "Collection entity should extrude");
    }

    #[test]
    fn test_feature_comment_on_role_change() {
        let mut writer = GCodeWriter::new();

        // Create two paths with different roles
        let mut path1 = create_simple_path();
        path1.role = ExtrusionRole::Perimeter;

        let mut path2 = create_simple_path();
        path2.role = ExtrusionRole::InternalInfill;

        let collection = ExtrusionEntityCollection {
            entities: vec![
                ExtrusionEntityType::Path(path1),
                ExtrusionEntityType::Path(path2),
            ],
            no_sort: false,
            orig_indices: Vec::new(),
        };

        extrude_collection(&collection, &mut writer).expect("Should extrude");

        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should have feature comments for both roles
        assert!(
            content.contains("Perimeter"),
            "Should have Perimeter feature comment"
        );
        assert!(
            content.contains("InternalInfill"),
            "Should have InternalInfill feature comment"
        );

        // Should have at least 2 feature comments
        let feature_count = content.matches("FEATURE:").count();
        assert!(
            feature_count >= 2,
            "Should have at least 2 feature comments, got {}",
            feature_count
        );
    }

    // Tests for travel_to, retract, and unretract

    #[test]
    fn test_travel_to_short_distance_no_retract() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        // Short travel (1mm) - should not trigger retraction
        let target = Point::new_scale(1.0, 0.0);
        let config = TravelConfig {
            avoid_crossing_perimeters: false,
            retract_on_travel: true,
            retract_length_travel: 2.0, // Threshold is 2mm
            z_hop: false,
            z_hop_height: 0.0,
        };

        travel_to(target, &mut writer, &config).expect("travel_to should succeed");

        // Should not be retracted
        assert!(
            !writer.is_retracted(),
            "Should not retract for short travel"
        );

        // Position should have moved
        let pos = writer.position();
        assert!((pos.x - 1.0).abs() < 0.001, "X position should be 1.0");
    }

    #[test]
    fn test_travel_to_long_distance_with_retract() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        // Long travel (10mm) - should trigger retraction
        let target = Point::new_scale(10.0, 0.0);
        let config = TravelConfig {
            avoid_crossing_perimeters: false,
            retract_on_travel: true,
            retract_length_travel: 2.0, // Threshold is 2mm
            z_hop: false,
            z_hop_height: 0.0,
        };

        travel_to(target, &mut writer, &config).expect("travel_to should succeed");

        // Should have retracted during travel
        // Note: Writer may have unretracted at destination, check G-code content
        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should contain travel move
        assert!(
            content.contains("G0") || content.contains("G1"),
            "Should have travel move"
        );

        // Position should have moved
        let pos = writer.position();
        assert!((pos.x - 10.0).abs() < 0.001, "X position should be 10.0");
    }

    #[test]
    fn test_travel_to_retract_disabled() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        // Long travel but retraction disabled
        let target = Point::new_scale(10.0, 0.0);
        let config = TravelConfig {
            avoid_crossing_perimeters: false,
            retract_on_travel: false, // Retraction disabled
            retract_length_travel: 2.0,
            z_hop: false,
            z_hop_height: 0.0,
        };

        travel_to(target, &mut writer, &config).expect("travel_to should succeed");

        // Should not be retracted
        assert!(!writer.is_retracted(), "Should not retract when disabled");
    }

    #[test]
    fn test_retract_basic() {
        let mut writer = GCodeWriter::new();
        writer.set_x(5.0);
        writer.set_y(5.0);

        // Initially not retracted
        assert!(!writer.is_retracted(), "Should start not retracted");

        retract(&mut writer, false).expect("retract should succeed");

        // Should now be retracted
        assert!(writer.is_retracted(), "Should be retracted after retract()");

        // G-code should contain retraction
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("G1") && content.contains("E"),
            "Should have retraction G-code"
        );
    }

    #[test]
    fn test_retract_when_already_retracted() {
        let mut writer = GCodeWriter::new();

        // Retract once
        retract(&mut writer, false).expect("First retract should succeed");
        let gcode_after_first = writer.get_gcode().content().to_string();

        // Retract again - should be no-op
        retract(&mut writer, false).expect("Second retract should succeed");
        let gcode_after_second = writer.get_gcode().content().to_string();

        // G-code should not have changed (no double retraction)
        assert_eq!(
            gcode_after_first, gcode_after_second,
            "Should not emit duplicate retraction"
        );
    }

    #[test]
    fn test_unretract_basic() {
        let mut writer = GCodeWriter::new();

        // First retract
        retract(&mut writer, false).expect("retract should succeed");
        assert!(writer.is_retracted(), "Should be retracted");

        // Then unretract
        unretract(&mut writer).expect("unretract should succeed");

        // Should no longer be retracted
        assert!(
            !writer.is_retracted(),
            "Should not be retracted after unretract()"
        );

        // G-code should contain unretraction
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("G1") && content.contains("E"),
            "Should have unretraction G-code"
        );
    }

    #[test]
    fn test_unretract_when_not_retracted() {
        let mut writer = GCodeWriter::new();

        // Initially not retracted
        assert!(!writer.is_retracted(), "Should start not retracted");

        let gcode_before = writer.get_gcode().content().to_string();

        // Unretract when not retracted - should be no-op
        unretract(&mut writer).expect("unretract should succeed");

        let gcode_after = writer.get_gcode().content().to_string();

        // G-code should not have changed
        assert_eq!(
            gcode_before, gcode_after,
            "Should not emit unretraction when not retracted"
        );
    }

    #[test]
    fn test_retract_unretract_cycle() {
        let mut writer = GCodeWriter::new();

        // Start not retracted
        assert!(!writer.is_retracted());

        // Retract
        retract(&mut writer, false).expect("retract should succeed");
        assert!(writer.is_retracted());

        // Unretract
        unretract(&mut writer).expect("unretract should succeed");
        assert!(!writer.is_retracted());

        // Retract again
        retract(&mut writer, false).expect("second retract should succeed");
        assert!(writer.is_retracted());

        // Unretract again
        unretract(&mut writer).expect("second unretract should succeed");
        assert!(!writer.is_retracted());
    }

    // ===== Wipe Tests =====

    #[test]
    fn test_wipe_basic() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        // Create a simple wipe path (10mm straight line)
        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];
        let wipe_path = Polyline::from_points(points);

        // Perform wipe with 5mm distance, 0.4mm retraction, 50mm/s speed
        wipe(
            &mut writer,
            &wipe_path,
            5.0,   // wipe_distance
            0.4,   // retraction_length
            50.0,  // wipe_speed
            false, // toolchange
            false, // is_last
        )
        .expect("wipe should succeed");

        // Verify position updated
        let final_pos = writer.position();
        assert!(final_pos.x > 0.0, "X position should advance during wipe");

        // Verify G-code contains wipe markers
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("TYPE:Wipe_Start"),
            "Should contain wipe start marker"
        );
        assert!(
            content.contains("TYPE:Wipe_End"),
            "Should contain wipe end marker"
        );
        assert!(
            content.contains("wipe and retract"),
            "Should contain wipe comment"
        );
    }

    #[test]
    fn test_wipe_zero_length() {
        let mut writer = GCodeWriter::new();

        // Create a wipe path
        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];
        let wipe_path = Polyline::from_points(points);

        // Wipe with zero retraction length (should be no-op)
        wipe(
            &mut writer,
            &wipe_path,
            5.0,  // wipe_distance
            0.0,  // retraction_length = 0
            50.0, // wipe_speed
            false,
            false,
        )
        .expect("wipe with zero length should succeed");

        // Should be no-op, no G-code emitted
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            !content.contains("Wipe_Start"),
            "Should not contain wipe markers for zero-length wipe"
        );
    }

    #[test]
    fn test_wipe_empty_path() {
        let mut writer = GCodeWriter::new();

        // Empty wipe path
        let wipe_path = Polyline::from_points(vec![]);

        // Should handle gracefully
        wipe(&mut writer, &wipe_path, 5.0, 0.4, 50.0, false, false)
            .expect("wipe with empty path should succeed");

        // No wipe should occur
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            !content.contains("Wipe_Start"),
            "Should not wipe with empty path"
        );
    }

    #[test]
    fn test_wipe_toolchange() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];
        let wipe_path = Polyline::from_points(points);

        // Toolchange wipe
        wipe(
            &mut writer,
            &wipe_path,
            5.0,
            0.4,
            50.0,
            true, // toolchange = true
            false,
        )
        .expect("toolchange wipe should succeed");

        // Should still perform wipe
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("TYPE:Wipe_Start"),
            "Toolchange wipe should emit markers"
        );
    }

    #[test]
    fn test_wipe_is_last() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];
        let wipe_path = Polyline::from_points(points);

        // Last wipe (no cooling markers)
        wipe(
            &mut writer,
            &wipe_path,
            5.0,
            0.4,
            50.0,
            false,
            true, // is_last = true
        )
        .expect("last wipe should succeed");

        // Should still perform wipe
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("TYPE:Wipe_Start"),
            "Last wipe should still emit markers"
        );
    }

    // ===== Set Extruder Tests =====

    #[test]
    fn test_set_extruder_no_change() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // Current extruder is 0 (default)
        assert_eq!(writer.extruder(), 0);

        // Try to set to same extruder (should be no-op)
        set_extruder(0, &mut writer, 0.2, &config).expect("set_extruder should succeed");

        // Should not emit tool change
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            !content.contains("T0"),
            "Should not emit T command when extruder unchanged"
        );
    }

    #[test]
    fn test_set_extruder_single_extruder() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // Change to extruder 1 (single extruder mode)
        set_extruder(1, &mut writer, 0.2, &config).expect("set_extruder should succeed");

        // Should emit T command
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("T1") || content.contains("tool change"),
            "Should emit tool change command"
        );

        // Extruder should be updated
        assert_eq!(writer.extruder(), 1);
    }

    #[test]
    fn test_set_extruder_with_retraction() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // Set initial position
        writer.set_x(10.0);
        writer.set_y(10.0);

        // Change extruder (should retract first)
        set_extruder(1, &mut writer, 0.2, &config).expect("set_extruder should succeed");

        // Should have retracted
        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should contain both retraction and tool change
        assert!(
            content.contains(" E") || content.contains("retract"),
            "Should retract before tool change"
        );
        assert!(
            content.contains("T1") || content.contains("tool change"),
            "Should emit tool change command"
        );
    }

    #[test]
    fn test_set_extruder_multiple_changes() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // Change extruder multiple times
        set_extruder(1, &mut writer, 0.2, &config).expect("first change should succeed");
        assert_eq!(writer.extruder(), 1);

        set_extruder(0, &mut writer, 0.2, &config).expect("second change should succeed");
        assert_eq!(writer.extruder(), 0);

        set_extruder(2, &mut writer, 0.2, &config).expect("third change should succeed");
        assert_eq!(writer.extruder(), 2);

        // Should have emitted multiple tool changes
        let gcode = writer.get_gcode();
        let content = gcode.content();
        let t_count = content.matches("T").count();
        assert!(
            t_count >= 3,
            "Should emit at least 3 tool change commands (got {})",
            t_count
        );
    }

    #[test]
    fn test_set_extruder_first_layer() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // First layer (Z = 0.2) should use first layer temperatures
        set_extruder(1, &mut writer, 0.2, &config).expect("set_extruder should succeed");

        // Should complete successfully
        assert_eq!(writer.extruder(), 1);
    }

    #[test]
    fn test_set_extruder_mid_print() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // Mid-print (Z = 10.0) should use normal temperatures
        set_extruder(1, &mut writer, 10.0, &config).expect("set_extruder should succeed");

        // Should complete successfully
        assert_eq!(writer.extruder(), 1);
    }

    // ===== Cooling Integration Tests =====

    #[test]
    fn test_apply_layer_cooling_first_layer() {
        let mut writer = GCodeWriter::new();
        let cooling_buffer = CoolingBuffer::new(CoolingConfig::default());

        // First layer should not apply cooling
        apply_layer_cooling(&mut writer, &cooling_buffer, 10.0, 0).expect("cooling should succeed");

        // No fan command should be emitted
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            !content.contains("M106"),
            "First layer should not have fan commands"
        );
    }

    #[test]
    fn test_apply_layer_cooling_fast_layer() {
        let mut writer = GCodeWriter::new();
        let mut config = CoolingConfig::default();
        config.min_layer_time = 10.0;
        config.fan_below_layer_time = 15.0;
        config.fan_speed = 1.0;
        config.disable_fan_first_layers = 1;
        let cooling_buffer = CoolingBuffer::new(config);

        // Fast layer (5 seconds < 15 second threshold) should enable fan
        apply_layer_cooling(&mut writer, &cooling_buffer, 5.0, 1).expect("cooling should succeed");

        // Fan should be enabled
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(content.contains("M106"), "Fast layer should enable fan");
    }

    #[test]
    fn test_apply_layer_cooling_slow_layer() {
        let mut writer = GCodeWriter::new();
        let mut config = CoolingConfig::default();
        config.min_layer_time = 10.0;
        config.fan_below_layer_time = 15.0;
        config.disable_fan_first_layers = 1;
        let cooling_buffer = CoolingBuffer::new(config);

        // Slow layer (20 seconds > 15 second threshold) should not enable fan
        apply_layer_cooling(&mut writer, &cooling_buffer, 20.0, 1).expect("cooling should succeed");

        // No fan command (or M107 fan off)
        let gcode = writer.get_gcode();
        let content = gcode.content();
        // Either no M106 or has M107 (fan off)
        let has_fan_on = content.contains("M106 S");
        let has_fan_off = content.contains("M107");
        assert!(
            !has_fan_on || has_fan_off,
            "Slow layer should have fan off or no fan command"
        );
    }

    #[test]
    fn test_bridge_fan_speed() {
        let mut config = CoolingConfig::default();
        config.bridge_fan_override = true;
        config.bridge_fan_speed = 1.0;
        let cooling_buffer = CoolingBuffer::new(config);

        let speed = bridge_fan_speed(&cooling_buffer);
        assert_eq!(speed, 1.0, "Bridge fan should be full speed");
    }

    #[test]
    fn test_overhang_fan_speed() {
        let mut config = CoolingConfig::default();
        config.overhang_fan_override = true;
        config.overhang_fan_speed = 0.5;
        let cooling_buffer = CoolingBuffer::new(config);

        let speed = overhang_fan_speed(&cooling_buffer);
        assert_eq!(speed, 0.5, "Overhang fan should be 50%");
    }

    #[test]
    fn test_set_fan_speed_for_role_normal() {
        let mut writer = GCodeWriter::new();
        let cooling_buffer = CoolingBuffer::new(CoolingConfig::default());

        // Normal extrusion should use base fan speed
        set_fan_speed_for_role(&mut writer, 0.8, ExtrusionRole::Perimeter, &cooling_buffer)
            .expect("set fan should succeed");

        let gcode = writer.get_gcode();
        let content = gcode.content();
        // 0.8 * 255 = 204
        assert!(
            content.contains("M106 S204"),
            "Should set fan to 80% ({})",
            content
        );
    }

    #[test]
    fn test_set_fan_speed_for_role_bridge() {
        let mut writer = GCodeWriter::new();
        let mut config = CoolingConfig::default();
        config.bridge_fan_override = true;
        config.bridge_fan_speed = 1.0;
        let cooling_buffer = CoolingBuffer::new(config);

        // Bridge should use override speed
        set_fan_speed_for_role(
            &mut writer,
            0.5,
            ExtrusionRole::BridgeInfill,
            &cooling_buffer,
        )
        .expect("set fan should succeed");

        let gcode = writer.get_gcode();
        let content = gcode.content();
        // Bridge override: 1.0 * 255 = 255
        assert!(
            content.contains("M106 S255"),
            "Bridge should use 100% fan ({})",
            content
        );
    }

    #[test]
    fn test_set_fan_speed_for_role_overhang() {
        let mut writer = GCodeWriter::new();
        let mut config = CoolingConfig::default();
        config.overhang_fan_override = true;
        config.overhang_fan_speed = 0.7;
        let cooling_buffer = CoolingBuffer::new(config);

        // Overhang should use override speed
        set_fan_speed_for_role(
            &mut writer,
            0.5,
            ExtrusionRole::OverhangPerimeter,
            &cooling_buffer,
        )
        .expect("set fan should succeed");

        let gcode = writer.get_gcode();
        let content = gcode.content();
        // Overhang override: 0.7 * 255 = 178.5 ≈ 178
        assert!(
            content.contains("M106 S178") || content.contains("M106 S179"),
            "Overhang should use 70% fan ({})",
            content
        );
    }
}
