# Retract When Crossing Perimeters Module

## Overview

This module determines whether to perform retraction when a travel move crosses perimeter boundaries. It helps reduce stringing by retracting when the nozzle travels outside internal regions or crosses perimeters.

## BambuStudio Reference

This module corresponds to:
- `src/libslic3r/GCode/RetractWhenCrossingPerimeters.cpp`
- `src/libslic3r/GCode/RetractWhenCrossingPerimeters.hpp`

## Algorithm

The module provides a two-stage check for each travel move:

1. **Internal Region Check**: Is the travel path entirely within internal regions?
   - Internal regions are areas where sparse infill will be printed
   - Stringing in these areas will be hidden by infill, so retraction is unnecessary

2. **Perimeter Crossing Check**: Does the travel path cross any perimeter lines?
   - Even if inside internal regions, crossing perimeters can cause visible defects
   - This check uses line segment intersection testing

### Decision Logic

```
if travel is inside internal regions AND doesn't cross perimeters:
    → NoRetract (skip retraction, stringing will be hidden)
else:
    → Retract (perform retraction to avoid visible stringing)
```

## Key Types

### `RetractDecision`

```rust
pub enum RetractDecision {
    Retract,    // Retraction is recommended
    NoRetract,  // Retraction can be skipped
}
```

### `RetractWhenCrossingPerimeters`

The main checker struct with layer caching for efficient repeated queries.

```rust
pub struct RetractWhenCrossingPerimeters {
    // Caches internal islands (ExPolygons) for containment checks
    internal_islands: Vec<ExPolygon>,
    // Caches perimeter lines for intersection checks
    perimeter_lines: Vec<Line>,
    // Bounding boxes for fast rejection
    ...
}
```

### `RetractCrossingConfig`

Configuration options:

```rust
pub struct RetractCrossingConfig {
    pub enabled: bool,                    // Feature toggle
    pub check_perimeter_crossings: bool,  // Enable crossing check
    pub min_travel_distance: CoordF,      // Minimum travel to consider
}
```

## Usage

```rust
use slicer::gcode::{RetractWhenCrossingPerimeters, RetractDecision};

let mut checker = RetractWhenCrossingPerimeters::new();

// For each travel move on a layer
let travel = Polyline::from_points(vec![start, end]);
let decision = checker.check_travel(&layer, &travel);

if decision.should_retract() {
    // Perform retraction
    gcode_writer.retract();
}
```

## Performance Considerations

### Caching Strategy

The module caches layer data to avoid recomputing for every travel move:

1. **Internal Islands Cache**: Built from `fill_surfaces` with `SurfaceType::Internal`
2. **Perimeter Lines Cache**: Built from internal island boundaries and perimeter polygons
3. **Cache Invalidation**: Automatically cleared when layer ID changes

### Bounding Box Acceleration

- Each internal island has a pre-computed bounding box
- Perimeter lines have a combined bounding box
- Travel paths are rejected early if bounding boxes don't intersect

### Line Intersection

Uses cross-product method for exact line segment intersection:

```rust
fn lines_intersect(line1: &Line, line2: &Line) -> bool {
    // Cross product of direction vectors
    // Check if intersection parameter is in [0, 1] for both segments
    // Uses i128 arithmetic to avoid overflow
}
```

## Integration Points

This module integrates with:

1. **GCodeWriter** (`gcode/writer.rs`): Called before travel moves to decide retraction
2. **Layer** (`slice/layer.rs`): Reads fill_surfaces and perimeters
3. **AvoidCrossingPerimeters** (`travel/mod.rs`): Complementary feature for path planning

## Difference from AvoidCrossingPerimeters

| Feature | RetractWhenCrossing | AvoidCrossing |
|---------|---------------------|---------------|
| Purpose | Decide when to retract | Plan travel path |
| Output | Retract/NoRetract decision | Modified travel path |
| When used | After path is planned | During path planning |
| Complexity | O(n) line checks | O(n²) pathfinding |

Both features work together:
1. `AvoidCrossingPerimeters` plans travel paths to minimize crossings
2. `RetractWhenCrossingPerimeters` decides retraction for remaining crossings

## Test Coverage

17 unit tests covering:
- Empty and single-point travels
- Travel inside/outside internal regions
- Boundary crossing detection
- Line intersection edge cases
- Layer caching behavior
- Multiple internal regions
- Configuration options