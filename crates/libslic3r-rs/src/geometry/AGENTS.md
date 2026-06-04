# Geometry Module

## Purpose

The `geometry` module provides the fundamental geometric primitives used throughout the slicing pipeline. These types form the foundation for all spatial operations: slicing, offsetting, boolean operations, path generation, and G-code output.

## What This Module Does

1. **Point Types** (`point.rs`): 2D and 3D point representations in both scaled integer and floating-point forms
2. **Line Segments** (`line.rs`): Line segments with intersection, distance, and projection operations
3. **Polygons** (`polygon.rs`): Closed contours representing boundaries (outer shells, holes)
4. **Polylines** (`polyline.rs`): Open paths representing toolpaths and travel moves
5. **ExPolygons** (`expolygon.rs`): Polygons with holes - the primary representation for slice regions
6. **Bounding Boxes** (`bounding_box.rs`): Axis-aligned bounding boxes for spatial queries
7. **Transforms** (`transform.rs`): 2D and 3D transformation matrices
8. **Elephant Foot Compensation** (`elephant_foot.rs`): First layer shrinkage compensation

## libslic3r Reference

| Rust File | C++ File(s) | Description |
|-----------|-------------|-------------|
| `point.rs` | `Point.hpp/cpp` | Point types and operations |
| `line.rs` | `Line.hpp/cpp` | Line segment operations |
| `polygon.rs` | `Polygon.hpp/cpp` | Closed polygon type |
| `polyline.rs` | `Polyline.hpp/cpp` | Open polyline type |
| `expolygon.rs` | `ExPolygon.hpp/cpp` | Polygon with holes |
| `bounding_box.rs` | `BoundingBox.hpp/cpp` | AABB types |
| `elephant_foot.rs` | `ElephantFootCompensation.hpp/cpp` | First layer compensation |

## Coordinate System

**Critical**: The slicer uses scaled integer coordinates internally.

```
Coord (i64)  - Scaled integer, 1 unit = 1 nanometer
CoordF (f64) - Floating-point millimeters

SCALING_FACTOR = 1,000,000
1 mm = 1,000,000 internal units
```

### Why Scaled Integers?

Floating-point arithmetic can accumulate errors, especially in boolean operations where exact geometry matters. By using integers:
- Operations are exact (no rounding errors)
- Comparison is trivial (no epsilon needed)
- Clipper library works with integers

### Conversion Functions

```rust
scale(mm: f64) -> Coord      // mm to internal units
unscale(coord: Coord) -> f64 // internal units to mm
```

## Key Types

### Point / PointF

```rust
// Scaled integer point (internal use)
pub struct Point { pub x: Coord, pub y: Coord }

// Floating-point point in mm (external use)
pub struct PointF { pub x: f64, pub y: f64 }

// 3D variants
pub struct Point3 { pub x: Coord, pub y: Coord, pub z: Coord }
pub struct Point3F { pub x: f64, pub y: f64, pub z: f64 }
```

### Polygon

Closed contour represented as a sequence of points. The last point implicitly connects to the first.

```rust
pub struct Polygon {
    points: Vec<Point>,
}

// Key methods:
polygon.area()           // Signed area (positive = CCW, negative = CW)
polygon.is_clockwise()   // Check winding direction
polygon.contains(point)  // Point-in-polygon test
polygon.centroid()       // Center of mass
polygon.perimeter()      // Total edge length
```

### ExPolygon

A polygon with holes - the fundamental unit for layer regions.

```rust
pub struct ExPolygon {
    pub contour: Polygon,      // Outer boundary (CCW)
    pub holes: Vec<Polygon>,   // Inner boundaries (CW)
}
```

**Convention**: 
- Outer contour is counter-clockwise (positive area)
- Holes are clockwise (negative area)

### Line

A directed line segment from point A to point B.

```rust
pub struct Line {
    pub a: Point,  // Start point
    pub b: Point,  // End point
}
```

### Polyline

An open path (sequence of connected line segments).

```rust
pub struct Polyline {
    points: Vec<Point>,
}
```

### BoundingBox

Axis-aligned bounding box for spatial queries.

```rust
pub struct BoundingBox {
    pub min: Point,
    pub max: Point,
}
```

## File Structure

```
slicer/src/geometry/
├── AGENTS.md           # This file
├── mod.rs              # Module exports
├── point.rs            # Point types (2D/3D, int/float)
├── line.rs             # Line segment type
├── polygon.rs          # Closed polygon type
├── polyline.rs         # Open polyline type
├── expolygon.rs        # Polygon with holes
├── bounding_box.rs     # AABB types
├── transform.rs        # Transformation matrices
└── elephant_foot.rs    # First layer compensation
```

## Common Operations

### Area Calculation

```rust
// Shoelace formula for polygon area
// Returns signed area: positive = CCW, negative = CW
fn signed_area(points: &[Point]) -> f64 {
    let mut sum = 0i128;
    for i in 0..points.len() {
        let j = (i + 1) % points.len();
        sum += points[i].x as i128 * points[j].y as i128;
        sum -= points[j].x as i128 * points[i].y as i128;
    }
    unscale(sum as Coord) / 2.0
}
```

### Point-in-Polygon

Uses ray casting algorithm - count intersections with polygon edges.

### Distance Calculations

```rust
point.distance_to(other)        // Euclidean distance
line.distance_to_point(point)   // Perpendicular distance
```

## Elephant Foot Compensation

First layer of prints often "squishes" wider than intended due to bed adhesion. This module compensates by shrinking first-layer polygons.

```rust
pub struct ElephantFootConfig {
    pub compensation: f64,      // Amount to shrink (mm)
    pub min_width: f64,         // Minimum feature width to preserve
}
```

**libslic3r Reference**: `ElephantFootCompensation.hpp/cpp`

## Relationship to Clipper

The `clipper/` module wraps the Clipper2 library for boolean operations (union, intersection, difference, offset). Clipper operates on the same `Polygon` and `ExPolygon` types defined here.

## Dependencies

- `crate::{Coord, CoordF, scale, unscale}` - Coordinate types and scaling
- No external dependencies for core types

## Related Modules

- `clipper/` - Boolean operations on polygons
- `slice/` - Uses ExPolygon for layer regions
- `perimeter/` - Generates polygon offsets for shells
- `infill/` - Fills ExPolygon regions
- `gcode/path.rs` - Converts to toolpaths

## Testing Strategy

1. **Unit tests** for each type's methods
2. **Property tests** for area/perimeter calculations
3. **Round-trip tests** for coordinate scaling
4. **Comparison tests** against libslic3r output for complex operations

---

## AABB Tree (`aabb_tree.rs`)

### Purpose

The AABB (Axis-Aligned Bounding Box) Tree provides spatial acceleration for mesh queries. It enables efficient:
- Ray casting (first hit, all hits)
- Closest point queries
- Distance queries
- Range queries (find primitives within a distance)

### libslic3r Reference

| Rust | C++ File | Description |
|------|----------|-------------|
| `aabb_tree.rs` | `AABBTreeIndirect.hpp` | Main AABB tree implementation |
| - | `AABBTreeLines.hpp` | 2D line variant (not yet ported) |

### Algorithm

The tree is a **balanced binary tree** built over bounding boxes of primitives (triangles). Key properties:

1. **Implicit Indexing**: Children of node `i` are at positions `2*i+1` (left) and `2*i+2` (right)
2. **Balanced**: Built using QuickSelect to partition by median centroid
3. **Split Axis**: At each level, split along the longest axis of the combined bounding box
4. **Cache Friendly**: Single contiguous array storage

### Key Types

```rust
/// 3D vector for AABB tree calculations
pub struct Vec3 {
    pub x: CoordF,
    pub y: CoordF,
    pub z: CoordF,
}

/// 3D axis-aligned bounding box
pub struct AABB3 {
    pub min: Vec3,
    pub max: Vec3,
}

/// Tree node (leaf or internal)
pub struct AABBNode {
    pub idx: usize,    // Primitive index (or INNER for internal nodes)
    pub bbox: AABB3,   // Bounding box
}

/// The AABB tree itself
pub struct AABBTree {
    nodes: Vec<AABBNode>,  // Flat array with implicit child indexing
}

/// Triangle mesh with AABB tree for queries
pub struct IndexedTriangleSet {
    pub vertices: Vec<Vec3>,
    pub triangles: Vec<[usize; 3]>,
    pub tree: AABBTree,
}
```

### Query Results

```rust
/// Ray intersection result
pub struct RayHit {
    pub primitive_idx: usize,  // Triangle index
    pub t: CoordF,             // Distance along ray
    pub u: CoordF,             // Barycentric U
    pub v: CoordF,             // Barycentric V
}

/// Closest point result
pub struct AABBClosestPointResult {
    pub primitive_idx: usize,
    pub point: Vec3,
    pub squared_distance: CoordF,
}
```

### Key Algorithms

#### Tree Building
```rust
// QuickSelect partitioning for balanced tree
fn build_recursive(input: &mut [BuildInput], node_idx: usize, left: usize, right: usize) {
    if left == right {
        // Leaf node
        return;
    }
    
    // Find longest axis of combined bounding box
    let dimension = bbox.longest_axis();
    
    // Partition around median using QuickSelect
    let center = (left + right) / 2;
    partition_input(input, dimension, left, right, center);
    
    // Recurse on children
    build_recursive(input, left_child_idx(node_idx), left, center);
    build_recursive(input, right_child_idx(node_idx), center + 1, right);
}
```

#### Ray-Box Intersection (Slab Method)
```rust
// "An Efficient and Robust Ray–Box Intersection Algorithm"
// by Amy Williams et al.
fn ray_box_intersect(origin: &Vec3, inv_dir: &Vec3, bbox: &AABB3, t0: f64, t1: f64) -> bool
```

#### Ray-Triangle Intersection (Möller–Trumbore)
```rust
// Fast ray-triangle intersection without precomputing plane equation
fn ray_triangle_intersect(origin, dir, v0, v1, v2, eps) -> Option<(t, u, v)>
```

#### Closest Point on Triangle (Ericson)
```rust
// From "Real-Time Collision Detection" Chapter 5
// Handles all cases: closest to vertex, edge, or face
fn closest_point_on_triangle(p: &Vec3, a: &Vec3, b: &Vec3, c: &Vec3) -> Vec3
```

### Usage Example

```rust
use slicer::geometry::{IndexedTriangleSet, Vec3};

// Create mesh from vertices and triangles
let vertices = vec![
    Vec3::new(0.0, 0.0, 0.0),
    Vec3::new(1.0, 0.0, 0.0),
    Vec3::new(0.0, 1.0, 0.0),
];
let triangles = vec![[0, 1, 2]];

let mesh = IndexedTriangleSet::new(vertices, triangles);

// Ray casting
let origin = Vec3::new(0.25, 0.25, 1.0);
let direction = Vec3::new(0.0, 0.0, -1.0);
if let Some(hit) = mesh.ray_cast_first(&origin, &direction) {
    println!("Hit triangle {} at t={}", hit.primitive_idx, hit.t);
}

// Closest point query
let query_point = Vec3::new(0.5, 0.5, 0.5);
if let Some(result) = mesh.closest_point(&query_point) {
    println!("Closest point: {:?}, distance: {}", result.point, result.distance());
}

// Range query
let nearby = mesh.triangles_within_distance(&query_point, 1.0);
println!("Found {} triangles within distance 1.0", nearby.len());
```

### Performance Characteristics

- **Build Time**: O(n log n) using QuickSelect
- **Ray Cast First Hit**: O(log n) average, O(n) worst case
- **Closest Point**: O(log n) average with early termination
- **Memory**: O(n) nodes, stored contiguously for cache efficiency

### Use Cases in Slicer

1. **Support Generation**: Find triangles under overhangs
2. **Tree Support Collision**: Check branch paths against model
3. **Adaptive Infill**: Query triangles near slice plane
4. **Seam Placement**: Find surface normals at seam candidates