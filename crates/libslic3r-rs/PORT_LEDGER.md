# libslic3r C++ → Rust faithful port ledger

**Progress: 95/278 units ported** (34%)  ·  partial 48  ·  deferred 3  ·  source of truth: `PORT_LEDGER.json`

Each unit = one C++ `.cpp` (line-by-line port to the mirrored snake_case Rust file) or a header-only `.hpp`. Driven by the `libslic3r-systematic-port` workflow (one phase per file, build-gated, resumable).

## By area

| Area | Done | Total |
|------|------|-------|
| root | 85 | 152 |
| Algorithm | 0 | 1 |
| Arachne | 0 | 26 |
| CSGMesh | 4 | 7 |
| Execution | 0 | 3 |
| Fill | 1 | 18 |
| Format | 0 | 10 |
| GCode | 0 | 17 |
| Geometry | 5 | 10 |
| Interlocking | 0 | 2 |
| Optimize | 0 | 3 |
| SLA | 0 | 20 |
| Shape | 0 | 1 |
| Support | 0 | 8 |

## Next pending

- [ ] `ElephantFootCompensation.cpp` (647 loc) → `crates/libslic3r-rs/src/elephant_foot_compensation.rs`
- [ ] `Polyline.cpp` (687 loc) → `crates/libslic3r-rs/src/geometry/polyline.rs`
- [ ] `ExtrusionEntity.cpp` (691 loc) → `crates/libslic3r-rs/src/extrusion_entity.rs`
- [ ] `Polygon.cpp` (747 loc) → `crates/libslic3r-rs/src/geometry/polygon.rs`
- [ ] `TriangleSelector.cpp` (2288 loc) → `crates/libslic3r-rs/src/triangle_selector.rs`
- [ ] `Emboss.cpp` (2459 loc) → `crates/libslic3r-rs/src/emboss.rs`
- [ ] `TriangleMeshSlicer.cpp` (2635 loc) → `crates/libslic3r-rs/src/triangle_mesh_slicer.rs`
- [ ] `MultiMaterialSegmentation.cpp` (2652 loc) → `crates/libslic3r-rs/src/multi_material_segmentation.rs`
- [ ] `Preset.cpp` (4039 loc) → `crates/libslic3r-rs/src/preset.rs`
- [ ] `CutSurface.cpp` (4082 loc) → `crates/libslic3r-rs/src/cut_surface.rs`
- [ ] `PrintObject.cpp` (4128 loc) → `crates/libslic3r-rs/src/print_object.rs`
- [ ] `Model.cpp` (4554 loc) → `crates/libslic3r-rs/src/model.rs`
- [ ] `Print.cpp` (4834 loc) → `crates/libslic3r-rs/src/print.rs`
- [ ] `PresetBundle.cpp` (6031 loc) → `crates/libslic3r-rs/src/preset_bundle.rs`
- [ ] `GCode.cpp` (7703 loc) → `crates/libslic3r-rs/src/g_code.rs`
- [ ] `PrintConfig.cpp` (9849 loc) → `crates/libslic3r-rs/src/print_config.rs`
- [ ] `Geometry/Curves.hpp` (218 loc) → `crates/libslic3r-rs/src/geometry/curves.rs`
- [ ] `Geometry/Bicubic.hpp` (291 loc) → `crates/libslic3r-rs/src/sla/bicubic.rs`
- [ ] `Geometry/VoronoiUtilsCgal.cpp` (326 loc) → `crates/libslic3r-rs/src/geometry/voronoi_utils_cgal.rs`
- [ ] `Geometry/VoronoiVisualUtils.hpp` (453 loc) → `crates/libslic3r-rs/src/geometry/voronoi_visual_utils.rs`
- [ ] `Algorithm/LineSegmentation/LineSegmentation.cpp` (583 loc) → `crates/libslic3r-rs/src/line_segmentation.rs`
- [ ] `Geometry/VoronoiOffset.cpp` (1638 loc) → `crates/libslic3r-rs/src/geometry/voronoi_offset.rs`
- [ ] `Arachne/utils/ExtrusionJunction.cpp` (19 loc) → `crates/libslic3r-rs/src/arachne/utils/extrusion_junction.rs`
- [ ] `Arachne/utils/HalfEdgeGraph.hpp` (29 loc) → `crates/libslic3r-rs/src/arachne/utils/half_edge_graph.rs`
- [ ] `Arachne/utils/HalfEdgeNode.hpp` (38 loc) → `crates/libslic3r-rs/src/arachne/utils/half_edge_node.rs`
