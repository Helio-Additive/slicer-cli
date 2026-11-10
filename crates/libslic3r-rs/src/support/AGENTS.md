# Support Module

## Purpose

The `support` module generates support structures for overhanging regions of 3D printed objects. Supports provide temporary scaffolding during printing that is removed afterward, enabling the printing of geometries that would otherwise be impossible with FDM technology.

## What This Module Does

1. **Overhang Detection**: Identify regions that need support based on overhang angle
2. **Support Generation**: Create support structures (normal or tree-based)
3. **Interface Layers**: Generate dense interface layers between support and model
4. **Support Patterns**: Various infill patterns for support material
5. **Contact Distance**: Manage gap between support and model for easy removal

## libslic3r Reference

| Rust File | C++ File(s) | Description |
|-----------|-------------|-------------|
| `mod.rs` | `Support/SupportMaterial.hpp/cpp` | Main support generation |
| `tree_model_volumes.rs` | `Support/TreeModelVolumes.hpp/cpp` | Collision detection & avoidance volumes |
| `tree_support_settings.rs` | `Support/TreeSupportCommon.hpp` | Settings & element state types |
| Tree support | `Support/TreeSupport.hpp/cpp` | Tree-style supports |
| | `Support/TreeSupport3D.cpp` | 3D tree collision avoidance |
| | `Support/SupportCommon.hpp/cpp` | Shared support utilities |
| | `Support/SupportParameters.hpp` | Support configuration |
| | `Support/SupportLayer.hpp` | Support layer data |

### Key C++ Classes

- **`SupportMaterial`**: Main support generation algorithm
- **`TreeSupport`**: Tree-style organic supports
- **`TreeSupport3D`**: 3D tree with collision avoidance
- **`TreeModelVolumes`**: Collision detection for tree growth
- **`TreeSupportSettings`**: Derived settings for tree generation
- **`SupportElement`**: State machine for branch elements
- **`SupportParameters`**: Configuration parameters
- **`SupportLayer`**: Per-layer support data

### Rust Implementation

- **`TreeModelVolumes`**: Pre-computed collision and avoidance areas per layer/radius
- **`TreeModelVolumesConfig`**: Configuration for volume computation
- **`RadiusLayerPolygonCache`**: Thread-safe cache for computed areas
- **`TreeSupportSettings`**: Computed settings from mesh group settings
- **`SupportElementState`**: State tracking for each support element
- **`SupportElement`**: Complete element with influence area and parents

## Support Types

### Normal Support

Traditional support with vertical pillars and interface layers:

```
Model overhang
    ↓
████████████████
     ████████        ← Interface layer (dense)
     ╎╎╎╎╎╎╎╎        ← Support body (sparse)
     ╎╎╎╎╎╎╎╎
     ╎╎╎╎╎╎╎╎
─────────────────    ← Build plate
```

**Characteristics**:
- Simple vertical pillars
- Easy to generate
- May leave marks on surface
- Can be difficult to remove in tight spaces

### Tree Support

Organic tree-like structures that branch to reach overhangs:

```
Model overhang
    ↓
████████████████
     ╲    ╱          ← Branches reach overhang
      ╲  ╱
       ╲╱            ← Trunk
       │
───────┴─────────    ← Build plate
```

**Characteristics**:
- Minimal contact with model
- Less material usage
- Better surface quality
- More complex to generate
- Better for organic shapes

## Key Types

### SupportConfig

```rust
pub struct SupportConfig {
    pub enabled: bool,                    // Generate supports
    pub support_type: SupportType,        // Normal or Tree
    pub pattern: SupportPattern,          // Infill pattern
    pub density: f64,                     // Support density (0-1)
    pub overhang_threshold: f64,          // Angle in degrees (usually 45°)
    pub contact_distance: f64,            // Gap to model (mm)
    pub interface_layers: u32,            // Dense interface layer count
    pub interface_density: f64,           // Interface layer density
    pub support_on_build_plate_only: bool, // Only from bed
    pub xy_distance: f64,                 // Horizontal gap to model
    pub z_distance: f64,                  // Vertical gap to model
}
```

### SupportType

```rust
pub enum SupportType {
    Normal,           // Traditional pillar support
    Tree,             // Tree/organic support
    Hybrid,           // Combination of both
}
```

### SupportPattern

```rust
pub enum SupportPattern {
    Rectilinear,      // Lines
    Grid,             // Cross-hatch
    Honeycomb,        // Hexagonal
    Gyroid,           // TPMS pattern
    Lightning,        // Minimal tree-like
}
```

### SupportLayer

```rust
pub struct SupportLayer {
    pub z: f64,                           // Layer height
    pub polygons: Vec<ExPolygon>,         // Support regions
    pub interface_polygons: Vec<ExPolygon>, // Interface regions
    pub contact_polygons: Vec<ExPolygon>, // Contact with model
}
```

### TreeBranch

```rust
pub struct TreeBranch {
    pub tip: Point,           // Contact point with model
    pub path: Vec<Point>,     // Path down to trunk/bed
    pub radius: f64,          // Branch thickness
    pub children: Vec<TreeBranch>, // Sub-branches
}
```

### TreeModelVolumes

```rust
pub struct TreeModelVolumes {
    config: TreeModelVolumesConfig,
    layer_outlines: Vec<ExPolygons>,  // Model geometry per layer
    collision_cache: RadiusLayerPolygonCache,  // Cached collision areas
    avoidance_cache_fast: RadiusLayerPolygonCache,  // Fast avoidance
    avoidance_cache_slow: RadiusLayerPolygonCache,  // Precise avoidance
    placeable_cache: RadiusLayerPolygonCache,  // Where tips can go
    wall_restriction_cache: RadiusLayerPolygonCache,  // Wall distances
}
```

### SupportElementState

```rust
pub struct SupportElementState {
    pub bits: SupportElementStateBits,  // Flags (to_buildplate, etc.)
    pub layer_idx: usize,               // Current layer
    pub target_position: Point,         // Where to support
    pub next_position: Point,           // Suggested next move
    pub distance_to_top: u32,           // Layers from tip
    pub effective_radius_height: u32,   // For radius calculation
    pub result_on_layer: Option<Point>, // Final position
    pub elephant_foot_increases: f64,   // Foot compensation
}
```

### TreeSupport3D

```rust
pub struct TreeSupport3D {
    config: TreeSupport3DConfig,
    volumes: TreeModelVolumes,
    move_bounds: LayerSupportElements,  // Support elements per layer
    num_layers: usize,
}

impl TreeSupport3D {
    pub fn generate(&mut self, overhangs: &[Vec<Polygon>]) -> TreeSupport3DResult;
}
```

### OrganicSmoother

```rust
pub struct OrganicSmoother {
    config: OrganicSmoothConfig,
    spheres: Vec<CollisionSphere>,       // Branch positions as spheres
    layer_caches: Vec<LayerCollisionCache>,  // Collision data per layer
    sphere_index: HashMap<(usize, usize), usize>,  // Layer/element to sphere index
}

impl OrganicSmoother {
    pub fn smooth(&mut self) -> usize;   // Returns iterations performed
    pub fn get_layer_positions(&self) -> HashMap<usize, Vec<Point>>;
}
```

### OrganicSmoothConfig

```rust
pub struct OrganicSmoothConfig {
    pub max_nudge_collision: CoordF,  // Max collision avoidance distance (0.5mm)
    pub max_nudge_smoothing: CoordF,  // Max smoothing nudge distance (0.2mm)
    pub collision_extra_gap: CoordF,  // Extra gap from model (0.1mm)
    pub smoothing_factor: f64,        // Laplacian smoothing factor (0.5)
    pub max_iterations: usize,        // Maximum iterations (100)
    pub convergence_threshold: usize, // Stop if fewer spheres moved
    pub layer_height: CoordF,         // Layer height for Z calculations
}
```

### SphereMapping

Maps collision spheres back to their source support elements:

```rust
pub struct SphereMapping {
    pub layer_idx: usize,      // Layer index in move_bounds
    pub element_idx: usize,    // Element index within the layer
    pub sphere_idx: usize,     // Index of sphere in smoother
}
```

### SphereBuildResult

Result of building spheres from move_bounds:

```rust
pub struct SphereBuildResult {
    pub smoother: OrganicSmoother,      // Smoother with spheres added
    pub mappings: Vec<SphereMapping>,   // Mapping from spheres to elements
    pub linear_data_layers: Vec<usize>, // Cumulative element counts per layer
}
```

### CollisionSphere

```rust
pub struct CollisionSphere {
    pub position: Point3F,            // 3D position
    pub radius: CoordF,               // Branch radius at this point
    pub element_below_id: Option<usize>,  // Link to element below
    pub parent_ids: Vec<usize>,       // Links to parent elements above
    pub locked: bool,                 // Tips and roots don't move
    pub layer_idx: usize,             // Current layer
    pub min_z: CoordF,                // Minimum Z for collision search
    pub max_z: CoordF,                // Maximum Z for collision search
}
```

### TreeSupport3DResult

```rust
pub struct TreeSupport3DResult {
    pub layers: Vec<SupportLayer>,  // Support polygons per layer
    pub branch_count: usize,        // Total branches generated
    pub tip_count: usize,           // Total contact points
}
```

## Overhang Detection

Overhangs are detected by analyzing layer-to-layer differences:

```rust
fn detect_overhangs(
    current_layer: &[ExPolygon],
    previous_layer: &[ExPolygon],
    threshold_angle: f64,
) -> Vec<ExPolygon> {
    // Calculate how far an overhang can extend
    let max_extension = layer_height / tan(threshold_angle);
    
    // Expand previous layer by max extension
    let supported = grow(previous_layer, max_extension);
    
    // Overhang = current layer - supported area
    difference(current_layer, &supported)
}
```

### Overhang Angle

The threshold angle determines what needs support:

```
      │← 45° typical threshold
      │
      ▼
    ████
   ██████      ← Needs support (>45°)
  ████████
 ██████████    ← Self-supporting (≤45°)
████████████
```

## Support Generation Algorithm

### Normal Support

```rust
fn generate_normal_support(
    overhangs: &[Vec<ExPolygon>],  // Per-layer overhangs
    config: &SupportConfig,
) -> Vec<SupportLayer> {
    let mut support_layers = Vec::new();
    
    // Start from top, project downward
    for (layer_idx, layer_overhangs) in overhangs.iter().enumerate().rev() {
        // Combine with support from layer above
        let combined = if layer_idx < overhangs.len() - 1 {
            union(layer_overhangs, &support_layers[layer_idx + 1].polygons)
        } else {
            layer_overhangs.clone()
        };
        
        // Apply XY distance from model
        let support_area = shrink(&combined, config.xy_distance);
        
        // Determine interface vs body
        let (interface, body) = split_interface(
            &support_area,
            layer_idx,
            config.interface_layers,
        );
        
        support_layers.push(SupportLayer {
            z: layer_z(layer_idx),
            polygons: body,
            interface_polygons: interface,
            contact_polygons: Vec::new(),
        });
    }
    
    support_layers.reverse();
    support_layers
}
```

### Tree Support Algorithm

```rust
fn generate_tree_support(
    overhangs: &[Vec<ExPolygon>],
    model_volumes: &TreeModelVolumes,
    config: &SupportConfig,
) -> Vec<TreeBranch> {
    // 1. Sample contact points on overhangs
    let contact_points = sample_overhang_points(overhangs);
    
    // 2. For each contact point, grow tree downward
    let mut trees = Vec::new();
    for point in contact_points {
        let tree = grow_tree_branch(
            point,
            model_volumes,
            config,
        );
        trees.push(tree);
    }
    
    // 3. Merge nearby branches
    merge_tree_branches(&mut trees);
    
    // 4. Convert to layer polygons
    rasterize_trees(&trees)
}
```

## Organic Smoothing Algorithm

Organic smoothing refines branch positions after initial generation using
Laplacian smoothing combined with collision avoidance:

```rust
fn smooth_branches(spheres: &mut [CollisionSphere], layer_caches: &[LayerCollisionCache]) {
    for iteration in 0..MAX_ITERATIONS {
        // 1. Backup positions for Laplacian averaging
        for sphere in spheres {
            sphere.backup_position();
        }
        
        let mut num_moved = 0;
        
        for sphere in spheres.iter_mut() {
            if sphere.locked { continue; }
            
            // 2. Check collision with model
            if let Some(collision) = find_collision(sphere, layer_caches) {
                // Nudge away from collision
                let nudge = (sphere.position - collision).normalized() * nudge_dist;
                sphere.position += nudge;
                num_moved += 1;
            }
            
            // 3. Laplacian smoothing
            let avg = weighted_average_of_neighbors(sphere);
            sphere.position = lerp(sphere.position, avg, SMOOTHING_FACTOR);
        }
        
        // 4. Check convergence
        if num_moved == 0 { break; }
    }
}
```

### Laplacian Smoothing

Each sphere's position is smoothed by averaging with its neighbors:

```
Before:         After:
    ○               ○
    │               │
    ●  ←── sphere   ● ←── moved toward average
   ╱ ╲             ╱ ╲
  ○   ○           ○   ○
```

The smoothing formula:
```
new_pos = (1 - factor) * old_pos + factor * weighted_avg(neighbors)
```

Where neighbors are:
- Parent elements (above) - weighted by sphere radius
- Element below - weighted to balance with parents

fn grow_tree_branch(
    tip: Point3,
    volumes: &TreeModelVolumes,
    config: &SupportConfig,
) -> TreeBranch {
    let mut path = vec![tip.to_2d()];
    let mut current = tip;
    
    // Grow downward, avoiding model
    while current.z > 0.0 {
        // Find direction that avoids model and moves toward bed
        let direction = find_safe_direction(current, volumes);
        
        // Move one layer down
        current = current + direction * layer_height;
        path.push(current.to_2d());
        
        // Increase radius as we go down
        current_radius += config.branch_radius_increase;
    }
    
    TreeBranch {
        tip: tip.to_2d(),
        path,
        radius: config.tip_radius,
        children: Vec::new(),
    }
}
```

## Interface Layers

Dense interface layers improve surface quality where support contacts model:

```
████████████████  ← Model
════════════════  ← Interface (90-100% density)
════════════════  ← Interface
╎╎╎╎╎╎╎╎╎╎╎╎╎╎╎╎  ← Support body (20-40% density)
╎╎╎╎╎╎╎╎╎╎╎╎╎╎╎╎
```

## Contact Distance

Gap between support and model for easier removal:

```
████████████████  ← Model surface
                  ← Z gap (contact_distance)
════════════════  ← Support top
```

## Pipeline Integration

The support module is integrated into the main slicing pipeline via `SupportGenerator`:

```rust
// In pipeline/mod.rs
let support_layers = if self.config.support.enabled {
    let gen = SupportGenerator::new(self.config.support.clone());
    gen.generate(&layer_slices)  // Dispatches to normal or tree support
} else {
    Vec::new()
};
```

### Support Type Dispatch

`SupportGenerator::generate()` automatically dispatches based on `SupportType`:

```rust
match self.config.support_type {
    SupportType::Normal => self.generate_normal_support(layer_slices),
    SupportType::Tree | SupportType::Organic => self.generate_tree_support(layer_slices),
}
```

### Tree Support Flow

1. **Overhang Detection**: Same algorithm as normal support
2. **TreeModelVolumes Setup**: Create volume calculator with layer outlines
3. **TreeSupport3D Generation**: Run branch generation algorithm
4. **Post-processing**: Mark interface layers, merge with overhangs

```rust
fn generate_tree_support(&self, layer_slices: &[(CoordF, CoordF, ExPolygons)]) -> Vec<SupportLayer> {
    // 1. Detect overhangs
    let overhangs = self.detect_overhangs(layer_slices);
    
    // 2. Setup TreeModelVolumes
    let volumes = TreeModelVolumes::with_layer_outlines(config, layer_outlines);
    
    // 3. Create and run TreeSupport3D
    let mut tree_support = TreeSupport3D::new(tree_config, volumes);
    let result = tree_support.generate(&overhang_polygons);
    
    // 4. Post-process and return
    result.layers
}
```

## File Structure

```
slicer/src/support/
├── AGENTS.md               # This file
├── mod.rs                  # Main support implementations & SupportGenerator
├── branch_mesh.rs          # Branch mesh drawing (3D tube extrusion & slicing)
├── organic_smooth.rs       # Organic smoothing (Laplacian + collision avoidance)
├── tree_model_volumes.rs   # Collision/avoidance volume computation
├── tree_support_3d.rs      # Tree support 3D branch generation
└── tree_support_settings.rs # Tree support settings & element state
```

## Branch Mesh Drawing

The `branch_mesh.rs` module implements 3D tube mesh generation for tree support branches,
providing more accurate geometry than simple circle-based rasterization.

### Algorithm Overview

1. **Build Branch Paths**: Traverse support elements to extract connected paths
2. **Extrude Branches**: Create tube meshes with hemispheres at endpoints
3. **Slice Mesh**: Slice the cumulative mesh at each layer height

### Key Types

```rust
pub struct BranchMeshBuilder {
    config: BranchMeshConfig,
    mesh: TriangleMesh,
}

pub struct BranchPath {
    elements: Vec<BranchPathElement>,  // Points along the branch
}

pub struct BranchPathElement {
    pub position: Point3D,  // 3D position in mm
    pub radius: f64,        // Branch radius at this point
    pub layer_idx: usize,
}

pub struct BranchMeshResult {
    pub mesh: TriangleMesh,
    pub z_span: (f64, f64),
    pub branch_count: usize,
}
```

### Mesh Generation Functions

```rust
// Build paths from support elements
pub fn build_branch_paths(
    move_bounds: &[Vec<SupportElement>],
    settings: &TreeSupportSettings,
) -> Vec<BranchPath>;

// Generate combined branch mesh
pub fn generate_branch_mesh(
    move_bounds: &[Vec<SupportElement>],
    settings: &TreeSupportSettings,
    config: BranchMeshConfig,
) -> BranchMeshResult;

// Slice mesh to get per-layer polygons
pub fn slice_branch_mesh(
    mesh: &TriangleMesh,
    layer_zs: &[CoordF],
    layer_heights: &[CoordF],
) -> Vec<Vec<ExPolygon>>;
```

### Usage with TreeSupport3D

```rust
// Simple circle-based generation (faster)
let result = tree_support.generate(&overhangs);

// Mesh-based generation (more accurate geometry)
let result = tree_support.generate_with_mesh(&overhangs);

// Mesh-based generation with organic smoothing (best quality)
let result = tree_support.generate_with_organic_smoothing(&overhangs, &model_outlines);
```

### Organic Smoothing Integration

The organic smoothing can be integrated with branch mesh generation for
smooth, collision-free branch paths. This follows the BambuStudio workflow:

1. **Build spheres from move_bounds**: Extract branch positions as collision spheres
2. **Run organic smoothing**: Apply Laplacian smoothing with collision avoidance
3. **Apply smoothed positions**: Write positions back to support elements
4. **Generate branch mesh**: Create tube mesh from smoothed positions

```rust
use slicer::support::{
    build_spheres_from_move_bounds,
    apply_smoothed_positions_to_move_bounds,
    smooth_move_bounds,
    OrganicSmoothConfig,
};

// High-level API (recommended):
let result = smooth_move_bounds(
    &mut move_bounds,
    &settings,
    &layer_outlines,
    OrganicSmoothConfig::default(),
);

// Low-level API (for custom control):
let build_result = build_spheres_from_move_bounds(&move_bounds, &settings, config);
build_result.smoother.build_layer_caches(&layer_outlines);
let iterations = build_result.smoother.smooth();
apply_smoothed_positions_to_move_bounds(
    &build_result.smoother,
    &build_result.mappings,
    &mut move_bounds,
);
```

### TreeSupport3D Methods for Organic Smoothing

```rust
impl TreeSupport3D {
    // Apply organic smoothing with default config
    pub fn apply_organic_smoothing(&mut self, model_outlines: &[ExPolygons]) -> OrganicSmoothResult;
    
    // Apply organic smoothing with custom config
    pub fn apply_organic_smoothing_with_config(
        &mut self,
        model_outlines: &[ExPolygons],
        config: OrganicSmoothConfig,
    ) -> OrganicSmoothResult;
    
    // Generate with mesh and automatic organic smoothing
    pub fn generate_with_organic_smoothing(
        &mut self,
        overhangs: &[Vec<Polygon>],
        model_outlines: &[ExPolygons],
    ) -> TreeSupport3DResult;
}
```

### BambuStudio Reference

- `Support/TreeSupport3D.cpp`: `draw_branches()`, `extrude_branch()`
- `Support/TreeSupport3D.cpp`: `discretize_circle()`, `triangulate_fan()`, `triangulate_strip()`
- `Support/TreeSupport3D.cpp`: `slice_branches()`

## Dependencies

- `crate::geometry::{ExPolygon, Polygon, Point, Point3}` - Geometry types
- `crate::clipper::{union, difference, grow, shrink}` - Boolean operations
- `crate::flow::Flow` - Extrusion calculations
- `crate::slice::Layer` - Layer data

## Related Modules

- `slice/` - Provides layer data for overhang detection
- `infill/` - Support patterns use infill algorithms
- `gcode/` - Converts support to toolpaths
- `flow/` - Support extrusion parameters

## Testing Strategy

1. **Unit tests**: Overhang detection, angle calculations (in `mod.rs`)
2. **Shape tests**: Known overhang geometries
3. **Tree tests**: Branch generation and merging (in `tree_support_3d.rs`)
4. **Smoothing tests**: Collision detection, Laplacian smoothing (in `organic_smooth.rs`)
5. **Integration tests**: Full pipeline with tree support (`tests/tree_support_integration.rs`)

### Test Coverage

- `support::tests` - 22 tests for SupportGenerator, SupportConfig, bridge detection
- `support::tree_model_volumes::tests` - 17 tests for collision/avoidance volumes
- `support::tree_support_3d::tests` - 18 tests for branch generation
- `support::tree_support_settings::tests` - 14 tests for settings and element state
- `support::organic_smooth::tests` - 17 tests for organic smoothing
- `tree_support_integration` - 16 integration tests for pipeline integration

## Future Work

1. ~~**Organic Smoothing**~~: ✅ Implemented - Laplacian smoothing + collision avoidance
2. **Branch Mesh Drawing**: Create 3D mesh for branches, then slice
3. **AABBTree Acceleration**: Speed up collision queries for large models
4. **Parallel Processing**: Parallelize branch growth across threads
5. **Advanced Merging**: Better branch merging heuristics
4. **Interface tests**: Proper interface layer placement
5. **Parity tests**: Compare with BambuStudio support

## Performance Considerations

1. **Spatial indexing**: Use AABB tree for collision detection
2. **Parallel processing**: Independent branches can grow in parallel
3. **Level of detail**: Simplify support polygons where possible
4. **Caching**: Cache model collision volumes

## Support Material vs Model Material

Support typically uses:
- Lower density (15-30%)
- Faster print speed
- Different material (soluble support)
- Less cooling

## Future Enhancements

| Feature | Status | libslic3r Reference |
|---------|--------|---------------------|
| Normal support | ✅ Done | `SupportMaterial.cpp` |
| Overhang detection | ✅ Done | `SupportMaterial.cpp` |
| Interface layers | ✅ Done | `SupportMaterial.cpp` |
| Basic tree support | 🔄 Partial | `TreeSupport.cpp` |
| TreeModelVolumes | ✅ Done | `TreeModelVolumes.cpp` |
| TreeSupportSettings | ✅ Done | `TreeSupportCommon.hpp` |
| SupportElement state | ✅ Done | `TreeSupport3D.hpp` |
| TreeSupport3D generator | ✅ Done | `TreeSupport3D.cpp` |
| 3D branch generation | ✅ Done | `TreeSupport3D.cpp` |
| Branch merging | ✅ Done | `TreeSupport3D.cpp` |
| Organic tree shapes | 📋 Planned | `TreeSupport.cpp` |
| Support enforcers | 📋 Planned | UI + `SupportMaterial.cpp` |
| Support blockers | 📋 Planned | UI + `SupportMaterial.cpp` |
| Paint-on support | 📋 Planned | `TriangleSelector.cpp` |
| Raft generation | ✅ Done | `SupportMaterial.cpp` |

## Support Removal Tips

Generated supports should be:
- Easy to break away from model
- Leave minimal surface marks
- Not trap inside enclosed spaces
- Accessible for removal tools