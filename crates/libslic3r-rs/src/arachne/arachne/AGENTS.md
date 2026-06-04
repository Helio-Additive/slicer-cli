# Arachne Variable-Width Perimeter Module

## Purpose

The `arachne` submodule implements variable-width perimeter generation based on the Arachne algorithm originally developed by Ultimaker for Cura. Unlike classic fixed-width perimeters, Arachne adapts extrusion width locally to fill thin sections completely without gaps or overlaps.

## What This Module Does

1. **Variable-Width Walls**: Generate perimeters that vary in width along their length
2. **Beading Strategy**: Calculate optimal bead (extrusion) widths for any wall thickness
3. **Thin Wall Handling**: Properly fill areas too narrow for standard perimeters
4. **Junction Management**: Handle points where extrusion width changes
5. **Toolpath Conversion**: Convert variable-width data to printable toolpaths

## libslic3r Reference

| Rust File | C++ File(s) | Description |
|-----------|-------------|-------------|
| `mod.rs` | `Arachne/WallToolPaths.cpp` | Main wall generation |
| `beading.rs` | `Arachne/BeadingStrategy/*.cpp` | Bead width calculation |
| `junction.rs` | `Arachne/ExtrusionJunction.hpp` | Junction point type |
| `line.rs` | `Arachne/ExtrusionLine.hpp` | Variable-width line type |

### C++ Source Files

The Arachne algorithm in libslic3r comes from CuraEngine:

```
BambuStudio/src/libslic3r/Arachne/
├── BeadingStrategy/
│   ├── BeadingStrategy.hpp/cpp       # Base strategy
│   ├── BeadingStrategyFactory.cpp    # Strategy selection
│   ├── DistributedBeadingStrategy.cpp
│   ├── LimitedBeadingStrategy.cpp
│   ├── OuterWallInsetBeadingStrategy.cpp
│   ├── RedistributeBeadingStrategy.cpp
│   └── WideningBeadingStrategy.cpp
├── ExtrusionJunction.hpp             # Junction point
├── ExtrusionLine.hpp                 # Variable-width line
├── SkeletalTrapezoidation.cpp        # Medial axis computation
├── SkeletalTrapezoidationGraph.cpp   # Graph data structure
├── SkeletalTrapezoidationJoint.hpp   # Graph joints
├── WallToolPaths.cpp                 # Main entry point
└── utils/
    ├── ExtrusionJunction.cpp
    ├── ExtrusionLine.cpp
    └── HalfEdgeGraph.hpp
```

### Key C++ Classes

- **`WallToolPaths`**: Orchestrates variable-width wall generation
- **`SkeletalTrapezoidation`**: Computes the medial axis (skeleton) of polygon
- **`BeadingStrategy`**: Abstract base for bead width strategies
- **`ExtrusionLine`**: Polyline with variable width at each point
- **`ExtrusionJunction`**: Single point with position and width

## The Arachne Algorithm

### Overview

Arachne uses the medial axis (skeleton) of a polygon to determine local wall thickness, then distributes beads (extrusion lines) optimally:

```
Input polygon         Medial axis          Variable-width walls
┌────────────┐       ┌─────·─────┐         ┌════════════┐
│            │       │    /│\    │         │▓▓▓▓▓▓▓▓▓▓▓▓│
│            │  →    │   / │ \   │    →    │▓▓▓▓▓▓▓▓▓▓▓▓│
│     ╲╱     │       │  ╲  │  ╱  │         │▓▓▓╲▓▓╱▓▓▓▓│
│            │       │   ╲ │ ╱   │         │▓▓▓▓▓▓▓▓▓▓▓▓│
└────────────┘       └────·──────┘         └════════════┘
                          ↑
                    Skeleton encodes
                    local thickness
```

### Steps

1. **Skeleton Computation**: Compute the medial axis of the polygon using Voronoi diagram
2. **Distance Annotation**: Annotate skeleton edges with distance to polygon boundary
3. **Bead Distribution**: Determine how many beads fit at each skeleton point
4. **Toolpath Generation**: Generate extrusion lines with varying width

## Key Types

### ArachneConfig

```rust
pub struct ArachneConfig {
    pub wall_count: u32,              // Desired number of walls
    pub min_bead_width: f64,          // Minimum extrusion width (mm)
    pub max_bead_width: f64,          // Maximum extrusion width (mm)
    pub preferred_bead_width: f64,    // Optimal extrusion width (mm)
    pub wall_transition_length: f64,  // Length over which width transitions
    pub wall_transition_angle: f64,   // Max angle for transitions
    pub wall_distribution_count: u32, // How to distribute bead count changes
}
```

### ExtrusionJunction

A point with an associated extrusion width:

```rust
pub struct ExtrusionJunction {
    /// Position in scaled coordinates
    pub position: Point,
    
    /// Extrusion width at this point (mm)
    pub width: f64,
    
    /// Which perimeter this belongs to (0 = outermost)
    pub perimeter_index: u32,
}
```

### ExtrusionLine

A polyline where width can vary along its length:

```rust
pub struct ExtrusionLine {
    /// Sequence of junctions defining the line
    pub junctions: Vec<ExtrusionJunction>,
    
    /// Whether this forms a closed loop
    pub is_closed: bool,
    
    /// Inset index (0 = outermost wall)
    pub inset_index: u32,
    
    /// Whether this is part of an odd bead count region
    pub is_odd: bool,
    
    /// The region this belongs to
    pub region_id: usize,
}
```

### ArachneResult

```rust
pub struct ArachneResult {
    /// Generated variable-width walls
    pub walls: Vec<ExtrusionLine>,
    
    /// Area remaining for infill
    pub infill_area: Vec<ExPolygon>,
    
    /// Statistics
    pub stats: ArachneStats,
}

pub struct ArachneStats {
    pub total_wall_count: usize,
    pub variable_width_sections: usize,
    pub min_width_used: f64,
    pub max_width_used: f64,
}
```

### BeadingStrategy

```rust
pub trait BeadingStrategy {
    /// Calculate optimal bead configuration for given thickness
    fn compute(&self, thickness: f64) -> BeadingResult;
    
    /// Get the optimal thickness for N beads
    fn optimal_thickness(&self, bead_count: u32) -> f64;
    
    /// Get transition threshold between N and N+1 beads
    fn transition_threshold(&self, lower_bead_count: u32) -> f64;
}

pub struct BeadingResult {
    /// Width of each bead
    pub bead_widths: Vec<f64>,
    
    /// Center position of each bead (relative to wall center)
    pub toolpath_locations: Vec<f64>,
    
    /// Total thickness filled
    pub total_thickness: f64,
}
```

## Beading Strategies

### Distributed Beading

Distributes width evenly across all beads:

```
Wall thickness: 1.5mm, preferred width: 0.45mm
→ 3 beads at 0.5mm each (1.5 / 3 = 0.5)
```

### Limited Beading

Caps bead count to prevent too-thin extrusions:

```
Wall thickness: 0.3mm, preferred width: 0.45mm
→ 1 bead at 0.3mm (not 0 beads)
```

### Widening Beading

Allows beads to widen to fill space:

```
Wall thickness: 0.6mm, preferred width: 0.45mm
→ 1 bead at 0.6mm (widened, not 2 thin beads)
```

## File Structure

```
slicer/src/perimeter/arachne/
├── AGENTS.md       # This file
├── mod.rs          # Module entry point, ArachneGenerator
├── beading.rs      # BeadingStrategy implementations
├── junction.rs     # ExtrusionJunction type
└── line.rs         # ExtrusionLine type
```

## Algorithm Details

### Medial Axis Computation

The skeleton is computed using Voronoi diagrams:

```rust
fn compute_skeleton(polygon: &Polygon) -> SkeletonGraph {
    // 1. Compute Voronoi diagram of polygon edges
    let voronoi = compute_voronoi(&polygon.points());
    
    // 2. Extract internal edges (inside polygon)
    let skeleton_edges = voronoi.edges()
        .filter(|e| is_inside_polygon(e, polygon))
        .collect();
    
    // 3. Annotate with distance to boundary
    for edge in &mut skeleton_edges {
        edge.distance = distance_to_boundary(edge.midpoint(), polygon);
    }
    
    SkeletonGraph::from_edges(skeleton_edges)
}
```

### Bead Distribution

At each skeleton point, determine how many beads fit:

```rust
fn distribute_beads(skeleton: &SkeletonGraph, config: &ArachneConfig) -> Vec<ExtrusionLine> {
    let mut lines = Vec::new();
    
    for edge in skeleton.edges() {
        // Local thickness = 2 × distance to boundary
        let thickness = edge.distance * 2.0;
        
        // Calculate bead configuration
        let beading = config.strategy.compute(thickness);
        
        // Generate extrusion lines for each bead
        for (i, (width, offset)) in beading.bead_widths.iter()
            .zip(beading.toolpath_locations.iter())
            .enumerate() 
        {
            lines.push(create_extrusion_line(
                edge,
                *width,
                *offset,
                i as u32,
            ));
        }
    }
    
    lines
}
```

### Width Transitions

When bead count changes, widths transition smoothly:

```
        ← 3 beads →  transition  ← 2 beads →
    ════════════════╲        ╱════════════════
    ════════════════ ╲      ╱ ════════════════
    ════════════════  ╲    ╱
                       ╲  ╱
                        ╲╱
                         (one bead ends)
```

## Classic vs Arachne Comparison

| Aspect | Classic | Arachne |
|--------|---------|---------|
| Width | Fixed | Variable |
| Thin walls | Gaps possible | Filled properly |
| Computation | Fast | Slower |
| G-code | Standard | Needs width per segment |
| Quality | Good | Better |
| Memory | Less | More |

## G-code Considerations

Variable-width extrusion requires:

1. **Per-segment width**: G-code generator must handle changing width
2. **Flow calculation**: Use correct `mm3_per_mm` for each width
3. **Smooth transitions**: Interpolate width between junctions

```gcode
; Classic (fixed width)
G1 X10 Y10 E0.5 ; width=0.45mm throughout

; Arachne (variable width)
G1 X10 Y10 E0.5 ; width=0.45mm
G1 X15 Y10 E0.6 ; width=0.50mm (widening)
G1 X20 Y10 E0.4 ; width=0.40mm (narrowing)
```

## Dependencies

- `crate::geometry::{Point, Polygon, ExPolygon}` - Geometry types
- `crate::clipper` - Boolean operations
- `crate::flow::Flow` - Width-dependent flow calculations
- Voronoi/skeleton computation (internal or external library)

## Related Modules

- `perimeter/mod.rs` - Parent module, selects Classic vs Arachne
- `flow/` - Flow calculations for variable widths
- `gcode/path.rs` - ExtrusionPath must handle variable width
- `geometry/` - Polygon types

## Testing Strategy

1. **Unit tests**: Beading calculations for known thicknesses
2. **Junction tests**: Width interpolation along lines
3. **Geometry tests**: Simple shapes (rectangle, circle)
4. **Thin wall tests**: Very narrow features filled correctly
5. **Parity tests**: Compare with BambuStudio Arachne output

## Performance Considerations

1. **Voronoi computation**: O(n log n) in point count
2. **Skeleton simplification**: Reduce nodes for faster processing
3. **Caching**: Cache beading results for common thicknesses
4. **Parallelization**: Each region can be processed independently

## Known Limitations

1. **Complex geometries**: Very intricate shapes may have skeleton artifacts
2. **Sharp corners**: May need special handling
3. **Self-intersecting polygons**: Not supported
4. **Performance**: Slower than classic perimeters

## Future Enhancements

| Feature | Status | Description |
|---------|--------|-------------|
| Basic Arachne | ✅ Done | Variable-width generation |
| Beading strategies | ✅ Done | Width distribution |
| ExtrusionLine | ✅ Done | Variable-width type |
| Width transitions | 🔄 Partial | Smooth width changes |
| Full skeleton | 📋 Planned | Complete Voronoi-based skeleton |
| Inward ordering | 📋 Planned | Print order optimization |
| Seam placement | 📋 Planned | Seam on variable-width walls |

## References

- [Arachne Paper](https://github.com/Ultimaker/CuraEngine/wiki/Arachne) - Original algorithm description
- [CuraEngine Implementation](https://github.com/Ultimaker/CuraEngine/tree/main/src/WallToolPaths) - Reference implementation
- libslic3r Arachne port in `BambuStudio/src/libslic3r/Arachne/`
