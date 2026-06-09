# libslic3r C++ → Rust faithful port ledger

**Progress: 147/278 units ported** (52%)  ·  partial 100  ·  deferred 4  ·  pending 27  ·  source of truth: `PORT_LEDGER.json`

Each unit = one C++ `.cpp` (line-by-line port to the mirrored snake_case Rust file) or header-only `.hpp`. Driven by the `libslic3r-systematic-port` workflow (one phase per file, build-gated, resumable).

## By area

| Area | Done | Total |
|------|------|-------|
| root | 88 | 152 |
| Algorithm | 0 | 1 |
| Arachne | 22 | 26 |
| CSGMesh | 5 | 7 |
| Execution | 3 | 3 |
| Fill | 10 | 18 |
| Format | 2 | 10 |
| GCode | 7 | 17 |
| Geometry | 7 | 10 |
| Interlocking | 0 | 2 |
| Optimize | 2 | 3 |
| SLA | 0 | 20 |
| Shape | 0 | 1 |
| Support | 1 | 8 |

## Remaining pending

- [ ] `Arachne/utils/SparsePointGrid.hpp` (90 loc) → `crates/libslic3r-rs/src/arachne/utils/sparse_point_grid.rs`
- [ ] `Format/objparser.cpp` (920 loc) → `crates/libslic3r-rs/src/format/objparser.rs`
- [ ] `Format/AMF.cpp` (1397 loc) → `crates/libslic3r-rs/src/format/amf.rs`
- [ ] `Format/3mf.cpp` (3278 loc) → `crates/libslic3r-rs/src/format/3mf.rs`
- [ ] `Format/bbs_3mf.cpp` (9455 loc) → `crates/libslic3r-rs/src/format/bbs_3mf.rs`
- [ ] `Interlocking/VoxelUtils.cpp` (219 loc) → `crates/libslic3r-rs/src/interlocking/voxel_utils.rs`
- [ ] `Interlocking/InterlockingGenerator.cpp` (497 loc) → `crates/libslic3r-rs/src/interlocking/interlocking_generator.rs`
- [ ] `SLA/JobController.hpp` (32 loc) → `crates/libslic3r-rs/src/sla/job_controller.rs`
- [ ] `SLA/ReprojectPointsOnMesh.hpp` (46 loc) → `crates/libslic3r-rs/src/sla/reproject_points_on_mesh.rs`
- [ ] `SLA/SupportPoint.hpp` (67 loc) → `crates/libslic3r-rs/src/sla/support_point.rs`
- [ ] `SLA/Concurrency.hpp` (70 loc) → `crates/libslic3r-rs/src/sla/concurrency.rs`
- [ ] `SLA/RasterBase.cpp` (86 loc) → `crates/libslic3r-rs/src/sla/raster_base.rs`
- [ ] `SLA/RasterToPolygons.cpp` (91 loc) → `crates/libslic3r-rs/src/sla/raster_to_polygons.rs`
- [ ] `SLA/SupportTree.cpp` (98 loc) → `crates/libslic3r-rs/src/sla/support_tree.rs`
- [ ] `SLA/BoostAdapter.hpp` (134 loc) → `crates/libslic3r-rs/src/sla/boost_adapter.rs`
- [ ] `SLA/ConcaveHull.cpp` (145 loc) → `crates/libslic3r-rs/src/sla/concave_hull.rs`
- [ ] `SLA/Clustering.cpp` (152 loc) → `crates/libslic3r-rs/src/sla/clustering.rs`
- [ ] `SLA/SpatIndex.cpp` (161 loc) → `crates/libslic3r-rs/src/sla/spat_index.rs`
- [ ] `SLA/AGGRaster.hpp` (222 loc) → `crates/libslic3r-rs/src/sla/agg_raster.rs`
- [ ] `SLA/SupportTreeBuilder.cpp` (225 loc) → `crates/libslic3r-rs/src/sla/support_tree_builder.rs`
- [ ] `SLA/SupportTreeMesher.cpp` (270 loc) → `crates/libslic3r-rs/src/sla/support_tree_mesher.rs`
- [ ] `SLA/IndexedMesh.cpp` (456 loc) → `crates/libslic3r-rs/src/sla/indexed_mesh.rs`
- [ ] `SLA/Rotfinder.cpp` (476 loc) → `crates/libslic3r-rs/src/sla/rotfinder.rs`
- [ ] `SLA/Pad.cpp` (538 loc) → `crates/libslic3r-rs/src/sla/pad.rs`
- [ ] `SLA/Hollowing.cpp` (563 loc) → `crates/libslic3r-rs/src/sla/hollowing.rs`
- [ ] `SLA/SupportPointGenerator.cpp` (668 loc) → `crates/libslic3r-rs/src/sla/support_point_generator.rs`
- [ ] `SLA/SupportTreeBuildsteps.cpp` (1277 loc) → `crates/libslic3r-rs/src/sla/support_tree_buildsteps.rs`
