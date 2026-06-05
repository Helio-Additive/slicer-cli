# libslic3r C++ → Rust faithful port ledger

**Progress: 74/278 units ported** (26%)  ·  partial 17  ·  deferred 0  ·  source of truth: `PORT_LEDGER.json`

Each unit = one C++ `.cpp` (line-by-line port to the mirrored snake_case Rust file) or a header-only `.hpp`. Driven by the `libslic3r-systematic-port` workflow (one phase per file, build-gated, resumable).

## By area

| Area | Done | Total |
|------|------|-------|
| root | 64 | 152 |
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

- [ ] `SurfaceMesh.hpp` (167 loc) → `crates/libslic3r-rs/src/surface_mesh.rs`
- [ ] `clonable_ptr.hpp` (168 loc) → `crates/libslic3r-rs/src/clonable_ptr.rs`
- [ ] `TextConfiguration.hpp` (181 loc) → `crates/libslic3r-rs/src/text_configuration.rs`
- [ ] `ModelArrange.cpp` (185 loc) → `crates/libslic3r-rs/src/model_arrange.rs`
- [ ] `ShortEdgeCollapse.cpp` (187 loc) → `crates/libslic3r-rs/src/short_edge_collapse.rs`
- [ ] `ProjectTask.cpp` (196 loc) → `crates/libslic3r-rs/src/project_task.rs`
- [ ] `CurveAnalyzer.cpp` (203 loc) → `crates/libslic3r-rs/src/curve_analyzer.rs`
- [ ] `Clipper2Utils.cpp` (211 loc) → `crates/libslic3r-rs/src/clipper2_utils.rs`
- [ ] `SlicingAdaptive.cpp` (216 loc) → `crates/libslic3r-rs/src/slicing_adaptive.rs`
- [ ] `Flow.cpp` (266 loc) → `crates/libslic3r-rs/src/flow.rs`
- [ ] `MeshSplitImpl.hpp` (346 loc) → `crates/libslic3r-rs/src/mesh_split_impl.rs`
- [ ] `JumpPointSearch.cpp` (349 loc) → `crates/libslic3r-rs/src/jump_point_search.rs`
- [ ] `PNGReadWrite.cpp` (363 loc) → `crates/libslic3r-rs/src/png_read_write.rs`
- [ ] `AABBTreeLines.hpp` (364 loc) → `crates/libslic3r-rs/src/aabb_tree_lines.rs`
- [ ] `KDTreeIndirect.hpp` (374 loc) → `crates/libslic3r-rs/src/kd_tree_indirect.rs`
- [ ] `BridgeDetector.cpp` (387 loc) → `crates/libslic3r-rs/src/bridge_detector.rs`
- [ ] `MutablePolygon.cpp` (387 loc) → `crates/libslic3r-rs/src/mutable_polygon.rs`
- [ ] `MeasureUtils.hpp` (390 loc) → `crates/libslic3r-rs/src/measure_utils.rs`
- [ ] `LogSink.cpp` (448 loc) → `crates/libslic3r-rs/src/log_sink.rs`
- [ ] `MarchingSquares.hpp` (448 loc) → `crates/libslic3r-rs/src/marching_squares.rs`
- [ ] `MutablePriorityQueue.hpp` (453 loc) → `crates/libslic3r-rs/src/mutable_priority_queue.rs`
- [ ] `Color.cpp` (486 loc) → `crates/libslic3r-rs/src/color.rs`
- [ ] `OverhangDetector.cpp` (508 loc) → `crates/libslic3r-rs/src/overhang_detector.rs`
- [ ] `NSVGUtils.cpp` (543 loc) → `crates/libslic3r-rs/src/nsvg_utils.rs`
- [ ] `GCodeSender.cpp` (580 loc) → `crates/libslic3r-rs/src/g_code_sender.rs`
