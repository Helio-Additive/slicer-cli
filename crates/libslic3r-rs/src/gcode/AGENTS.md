# G-code Generation Module

## Purpose

The `gcode` module handles the final stage of slicing: converting toolpaths into G-code commands that can be executed by a 3D printer. This includes path ordering, extrusion calculations, arc fitting, cooling control, and special modes like spiral vase.

## What This Module Does

1. **Path Management (`path.rs`)**: Represents extrusion paths with proper cross-section calculations, role classification, and length/volume computations.

2. **G-code Writing (`writer.rs`)**: Stateful writer that tracks position, extrusion, temperature, and generates valid G-code commands.

3. **Pressure Equalization (`pressure_equalizer.rs`)**: Smooths volumetric extrusion rate changes by adjusting feedrates, preventing pressure spikes in the extruder that cause artifacts like blobs and zits.

4. **G-code Container (`generator.rs`)**: Holds the complete G-code output with statistics and metadata.

5. **Arc Fitting (`arc_fitting.rs`)**: Converts sequences of linear segments into G2/G3 arc commands where appropriate, reducing file size and improving motion quality.

6. **Cooling Control (`cooling.rs`)**: Implements layer time slowdown, fan speed control, and per-extruder move adjustments.

7. **Spiral Vase Mode (`spiral_vase.rs`)**: Continuous Z-lifting for single-wall vase prints with smooth XY interpolation.

8. **Ironing (`ironing.rs`)**: Extra pass over top surfaces with low flow to smooth and improve surface quality.

9. **Seam Placement (`seam_placer.rs`)**: Intelligent seam placement for perimeter loops, considering visibility, overhangs, corners, and layer alignment.

10. **G-code Comparison (`compare.rs`)**: Tools for comparing generated G-code against reference files for parity testing.

## libslic3r Reference

This module corresponds to several C++ files in BambuStudio:

| Rust File | C++ File(s) | Description |
|-----------|-------------|-------------|
| `path.rs` | `ExtrusionEntity.hpp/cpp` | Extrusion path representation |
| `path.rs` (cross-section) | `Flow.hpp/cpp` | Cross-section area calculations |
| `writer.rs` | `GCodeWriter.hpp/cpp` | G-code command emission |
| `generator.rs` | `GCode.hpp/cpp` | High-level G-code generation |
| `arc_fitting.rs` | `GCode/ArcFitter.cpp` | Arc fitting algorithm |
| `cooling.rs` | `GCode/CoolingBuffer.cpp` | Cooling and slowdown logic |
| `spiral_vase.rs` | `GCode/SpiralVase.cpp` | Spiral vase mode |
| `pressure_equalizer.rs` | `GCode/PressureEqualizer.cpp` | Pressure equalization |
| `ironing.rs` | `Fill/Fill.cpp` (`Layer::make_ironing()`) | Surface ironing |
| `seam_placer.rs` | `GCode/SeamPlacer.cpp` | Seam placement algorithm |
| `wipe_tower.rs` | `GCode/WipeTower.cpp` | Multi-material wipe tower |
| `tool_ordering.rs` | `GCode/ToolOrdering.cpp` | Multi-extruder tool ordering |
| `multi_material.rs` | `GCode.cpp` (multi-material parts) | Coordinates tool ordering and wipe tower |

## Critical: Extrusion Calculations

The extrusion calculations in `path.rs` use the **exact formula from libslic3r/Flow.cpp**:

### Normal Extrusions (Rounded Rectangle)
```
area = height × (width - height × (1 - π/4))
     ≈ height × (width - 0.2146 × height)
```

### Bridge Extrusions (Circular)
```
area = π × (width/2)²
```

**WARNING**: Using `width × height` (simple rectangle) gives 10-15% error!

The `ExtrusionPath::cross_section_area()` method implements this correctly. The full `Flow` struct in `../flow/mod.rs` provides the complete flow calculation API.

## ExtrusionRole Mapping

| Rust | C++ | Description |
|------|-----|-------------|
| `ExtrusionRole::ExternalPerimeter` | `erExternalPerimeter` | Outer visible surface |
| `ExtrusionRole::Perimeter` | `erPerimeter` | Inner perimeters |
| `ExtrusionRole::InternalInfill` | `erInternalInfill` | Sparse infill |
| `ExtrusionRole::SolidInfill` | `erSolidInfill` | Top/bottom solid |
| `ExtrusionRole::TopSolidInfill` | `erTopSolidInfill` | Topmost surface |
| `ExtrusionRole::BridgeInfill` | `erBridgeInfill` | Bridging extrusions |
| `ExtrusionRole::GapFill` | `erGapFill` | Thin gap filling |
| `ExtrusionRole::Skirt` | `erSkirt` | Skirt/brim |
| `ExtrusionRole::SupportMaterial` | `erSupportMaterial` | Support structure |
| `ExtrusionRole::SupportMaterialInterface` | `erSupportMaterialInterface` | Support interface |

## Pressure Equalization Algorithm

The pressure equalizer works by analyzing volumetric extrusion rates and limiting how fast they can change:

### Volumetric Rate Calculation
```
volumetric_rate = filament_area × feedrate × (extrusion_length / travel_length)
```

Where `filament_area = π × (diameter/2)²`

### Rate Limiting
- **Backward pass**: Limits how fast the rate can increase (negative slope)
- **Forward pass**: Limits how fast the rate can decrease (positive slope)
- Default slope limit: ~1.8 mm³/s² (corresponds to 20→60 mm/s over 2 seconds)

### Special Cases
- **Bridge infill**: Unlimited slope (needs consistent flow for bridging)
- **Gap fill**: Unlimited slope (thin features need precise flow)

### Segment Splitting
Long segments are split when the rate change is significant, allowing smoother acceleration/deceleration profiles.

## Ironing Algorithm

Ironing adds an extra pass over top surfaces with very low flow to smooth them:

### Configuration
- **IroningType**: NoIroning, TopSurfaces, TopmostOnly, AllSolid
- **Flow**: Typically 10-15% of normal (default: 10%)
- **Line spacing**: Very tight, typically 0.1mm
- **Speed**: Moderate, typically 20mm/s
- **Inset**: Distance from edge (default: half nozzle diameter)

### Process
1. Identify surfaces to iron based on IroningType
2. Shrink surfaces by inset to avoid edge artifacts
3. Generate rectilinear fill pattern with tight spacing
4. Extrude with very low flow rate
5. Use nozzle heat to melt and smooth the surface

### Ironing Types
- **NoIroning**: Disabled
- **TopSurfaces**: Iron all top surfaces
- **TopmostOnly**: Only iron the topmost layer (no layer above)
- **AllSolid**: Iron all solid surfaces (top, bottom, solid infill)

## Seam Placement Algorithm

The seam placer determines optimal starting points for perimeter extrusion loops:

### Seam Position Modes
- **Aligned**: Aligns seams vertically across layers for cleaner seam lines (default)
- **Nearest**: Places seam nearest to previous/current position for shortest travel
- **Random**: Randomizes seam position (deterministically) to scatter seam artifacts
- **Rear**: Places seam at rear of print (highest Y) to hide on back of model
- **Hidden**: Actively seeks concave corners to hide seams

### Scoring Factors
1. **Point Type Priority**: Enforced > Neutral > Blocked (user-painted preferences)
2. **Overhang Avoidance**: Prefers points with solid support beneath
3. **Embedded Distance**: Prefers hidden points inside the print (multi-material joins)
4. **Corner Angle**: Concave corners (negative angle) have lower penalty than convex
5. **Visibility**: Optional mesh-based visibility scoring

### Angle Penalty Function
```
penalty = gauss(angle, 0, 1, 3) + sigmoid(-angle)
```
- Gaussian provides base penalty (peaks at straight sections)
- Sigmoid biases toward concave corners (negative angles)

### Seam Alignment
For aligned mode, seams are grouped across layers based on proximity:
1. Find seam strings (vertically adjacent seams within tolerance)
2. Calculate average position for each string
3. Adjust seam indices toward average while respecting quality constraints
4. Minimum string length required for alignment (default: 6 layers)

### Key Structures
- **SeamCandidate**: Per-vertex candidate with visibility, overhang, angle attributes
- **Perimeter**: Metadata for a perimeter loop (start/end indices, seam index)
- **LayerSeams**: All candidates and perimeters for one layer
- **SeamComparator**: Comparison logic for candidate scoring

## Wipe Tower Algorithm

The wipe tower is a sacrificial structure printed alongside multi-material prints for filament purging during tool changes:

### Purpose
1. **Purging**: Remove old filament from nozzle before printing new color
2. **Priming**: Prime the new filament to ensure consistent extrusion
3. **Stabilization**: Stabilize extrusion after material switch

### Key Components
- **WipeTower**: Main generator that plans and generates tower G-code
- **WipeTowerConfig**: Configuration (position, dimensions, speeds)
- **WipeTowerWriter**: Specialized G-code writer for tower operations
- **ToolChangeResult**: Result of each tool change operation
- **WipeTowerLayerInfo**: Planning data for each layer

### Tower Planning
```
1. plan_toolchange() - Record each tool change with purge volumes
2. plan_tower() - Calculate tower depth, spacing, and layer info
3. generate() - Generate G-code for all layers
```

### Tool Change Sequence
```
1. Retract old filament
2. Ramming (fast extrusion to push out old material)
3. Tool change command (T#)
4. Load new filament
5. Wiping (back-and-forth to clean and prime)
6. Continue to next position
```

### Depth Calculation
Tower depth is calculated based on:
- Total purge volume needed per layer
- Perimeter width and layer height
- Minimum depth for structural stability (height-dependent)

### Multi-Extruder Support
- Tracks filament adhesiveness categories
- Separate blocks for different material combinations
- Nozzle change handling for multi-nozzle setups

### libslic3r Reference
| Rust | C++ |
|------|-----|
| `wipe_tower.rs` | `GCode/WipeTower.cpp` |
| `WipeTowerWriter` | `WipeTowerWriter` (nested class) |

## Tool Ordering Algorithm

Tool ordering determines the optimal sequence of extruder switches for multi-material prints:

### Purpose
1. **Minimize Flush Volume**: Order extruders to reduce material wasted during color changes
2. **Coordinate Wipe Tower**: Calculate partitions needed per layer for multi-material
3. **Handle Custom Events**: Assign color changes, pauses, and custom G-code to layers

### Key Components
- **ToolOrdering**: Main engine that manages per-layer tool sequences
- **LayerTools**: Per-layer information about extruders, partitions, and custom G-code
- **FlushMatrix**: Matrix of purge volumes between filament pairs (mm³)
- **WipingExtrusions**: Tracks which extrusions are used for wiping during tool changes

### Algorithm Steps
```
1. initialize_layers() - Create LayerTools for each Z height
2. collect_extruders() - Gather required extruders per layer from objects/supports
3. handle_dontcare_extruders() - Assign "don't care" regions to minimize switches
4. reorder_extruders_for_minimum_flush() - Optimize order within each layer
5. fill_wipe_tower_partitions() - Calculate wipe tower requirements
6. assign_custom_gcodes() - Place color changes and pauses at correct layers
```

### Flush Optimization
Uses greedy nearest-neighbor algorithm to minimize total flush volume:
```
for each layer:
    start with last extruder from previous layer (if present)
    while extruders remain:
        pick next extruder with minimum flush from current
```

For small extruder counts (≤8), exhaustive search can find optimal ordering.

### "Don't Care" Extruder Handling
Regions marked with extruder 0 ("don't care") can be printed with any extruder:
- First layer: Can use automatic ordering based on object geometry
- Other layers: Use last extruder to minimize unnecessary switches
- Soluble filaments: Preferred at layer start (not wiped on first layer)

### Wipe Tower Partitions
Each layer needs partitions for tool changes:
```
partitions = extruders.len() - 1 (if first matches last from prev layer)
           = extruders.len()     (otherwise)
```
Partitions propagate downward (lower layers must support upper).

### Custom G-code Assignment
Color changes and pauses are assigned to the nearest layer:
- **Color Change**: Only applied if extruder prints above that layer
- **Tool Change**: Handled by normal tool ordering
- **Pause/Custom**: Always applied

### libslic3r Reference
| Rust | C++ |
|------|-----|
| `tool_ordering.rs` | `GCode/ToolOrdering.cpp` |
| `LayerTools` | `LayerTools` |
| `FlushMatrix` | Flush matrix from `PrintConfig` |
| `WipingExtrusions` | `WipingExtrusions` |

## File Structure

```
slicer/src/gcode/
├── AGENTS.md              # This file
├── mod.rs                 # Module exports and GCodeCommand enum
├── path.rs                # ExtrusionPath, ExtrusionRole, PathGenerator
├── writer.rs              # GCodeWriter stateful G-code emitter
├── generator.rs           # GCode container with stats
├── arc_fitting.rs         # Arc fitting (G2/G3) algorithm
├── cooling.rs             # CoolingBuffer layer time control
├── spiral_vase.rs         # Spiral vase mode implementation
├── pressure_equalizer.rs  # Pressure equalization for smooth extrusion
├── ironing.rs             # Surface ironing for top layer smoothing
├── seam_placer.rs         # Intelligent seam placement for perimeters
├── tool_ordering.rs       # Multi-extruder tool change optimization
├── wipe_tower.rs          # Multi-material wipe tower generation
├── multi_material.rs      # Multi-material coordination (integrates tool_ordering + wipe_tower)
└── compare.rs             # G-code comparison utilities
```

## Key Design Differences from C++

1. **Separation of Concerns**: Path representation (`ExtrusionPath`) is separate from G-code writing (`GCodeWriter`) and statistics (`GCode`).

2. **Immutable by Default**: Paths are created with builder pattern (`.with_width()`, `.with_height()`) rather than mutable setters.

3. **Error Handling**: Flow calculations return `Result` types rather than throwing exceptions.

4. **Type Safety**: Strong typing for roles, directions, and extrusion modes prevents invalid combinations.

## Usage Flow

```
LayerPaths (from pipeline)
    │
    ▼
ExtrusionPath[] ──── cross_section_area() uses Flow formula
    │
    ▼
PathGenerator ──── orders paths, calculates E values
    │
    ▼
GCodeWriter ──── emits G0/G1/G2/G3 commands
    │
    ▼
GCode ──── final output with stats
```

## Testing Strategy

1. **Unit Tests**: Each file has `#[cfg(test)]` module with function-level tests
2. **Parity Tests**: `compare.rs` enables comparison against BambuStudio reference G-code
3. **Integration Tests**: `tests/benchy_integration.rs` runs full pipeline on 3DBenchy

## Dependencies

- `crate::flow` - Flow calculations (cross-section area, E values)
- `crate::geometry` - Point, Polygon, Polyline types
- `crate::infill` - InfillPath, InfillResult
- `crate::perimeter` - PerimeterResult, ArachneResult

## Multi-Material Coordination

The multi_material module provides the `MultiMaterialCoordinator` that integrates tool ordering and wipe tower for seamless multi-extruder printing:

### Purpose
1. **Coordinate Tool Ordering**: Wire ToolOrdering to determine optimal extruder sequences
2. **Manage Wipe Tower**: Initialize and plan wipe tower for tool changes
3. **Provide Unified API**: Single entry point for multi-material planning

### Key Components
- **MultiMaterialConfig**: Configuration for all multi-material settings
- **MultiMaterialCoordinator**: Main coordinator that brings together tool ordering and wipe tower
- **MultiMaterialPlan**: Result containing per-layer tool change data
- **MultiMaterialLayer**: Per-layer data with extruders, tool changes, and G-code events
- **ToolChange**: Individual tool change event with flush volume
- **WipeTowerBounds**: Bounding box of wipe tower for collision avoidance

### Usage Flow
```rust
// Create coordinator with configuration
let config = MultiMaterialConfig::new(4); // 4 filaments
let mut coordinator = MultiMaterialCoordinator::new(config);

// Initialize layers
coordinator.initialize_layers(&[(0.2, 0.2), (0.4, 0.2), (0.6, 0.2)]);

// Add extruder usage for each layer/role
coordinator.add_extruder_to_layer(0.2, 0, ExtrusionRoleType::Perimeter);
coordinator.add_extruder_to_layer(0.2, 1, ExtrusionRoleType::InternalInfill);

// Plan multi-material operations
let plan = coordinator.plan(max_print_z);

// Generate wipe tower G-code
let wipe_tower_results = coordinator.generate_wipe_tower();
```

### Integration with Pipeline
The coordinator is designed to be used by the main print pipeline:
1. During layer processing, call `add_extruder_to_layer()` for each extrusion
2. After all layers processed, call `plan()` to optimize tool sequences
3. During G-code generation, query `get_extruder_for_role()` for each feature
4. Insert wipe tower G-code at appropriate layer points

### Wipe Tower Avoidance
The coordinator provides `get_wipe_tower_avoidance()` for travel planning:
```rust
if let Some(avoidance) = coordinator.get_wipe_tower_avoidance() {
    travel_planner.add_obstacle(avoidance);
}
```

### libslic3r Reference
| Rust | C++ |
|------|-----|
| `multi_material.rs` | `GCode.cpp` (multi-material integration) |
| `MultiMaterialCoordinator` | `GCode` class (wipe tower + tool ordering integration) |

## Related Modules

- `../flow/` - Flow calculations (MUST be used for extrusion math)
- `../pipeline/` - Orchestrates path generation and G-code output
- `../perimeter/` - Provides perimeter paths to convert
- `../infill/` - Provides infill paths to convert