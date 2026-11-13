# What the Rust engine does not port yet

A short list, kept by hand, of C++ the Rust engine still runs as C++ (through the four shim crates or a
child process), does differently by construction, or does not have. The per-function view of the whole
library is [`../FILE_MAP.md`](../FILE_MAP.md), which is generated; this file names the items worth a round
and records the ones that were closed. A round that closes or adds an item edits this file.

An item leaves section 1 or 2 only when the fixtures' scores do not go down with the Rust arm, or the drop
has a documented reason; until then the Rust arm sits behind an opt-in gate.

## 1. C++ behind the shim crates

Counted in `crates/libslic3r-rs/src` at R862; a number in brackets is the count of call sites when it is not one.
"Stands for" is read off the function's name and the comment at its call site, and is not otherwise checked.

### `clipper-z-sys` (ClipperLib, ClipperLib_Z, boost::polygon Voronoi)

The engine's own `clipper.cpp` is vendored and compiled; `libslic3r/clipper.cpp` has no Rust file.

| Shim function | Called from | Stands for |
|-|-|-|
| `cz_union_ex`, `cz_union_ex_safety`, `cz_union_flat`, `cz_union_polytree` | `clipper_utils.rs` | `union_ex`, `union_`, `union_pt` of ClipperUtils.cpp |
| `cz_difference_closed` (3), `cz_difference_closed_safety` | `clipper_utils.rs` | `diff`, `diff_ex` |
| `cz_intersection_closed` (2), `cz_intersection_closed_safety` | `clipper_utils.rs` | `intersection`, `intersection_ex` |
| `cz_intersection_pl`, `cz_diff_pl` | `clipper_utils.rs` | `intersection_pl`, `diff_pl` (open paths) |
| `cz_offset_paths`, `cz_offset_ex_paths`, `cz_offset_expolygon`, `cz_offset2_ex` | `clipper_utils.rs` | `offset`, `offset_ex`, `offset2_ex` |
| `cz_offset_polyline` | `extrusion_entity.rs` | `offset(Polyline)` in `polygons_covered_by_width` |
| `cz_simplify_polygons` | `clipper_utils.rs` | `simplify_polygons` |
| `cz_variable_offset_inner_ex` | `clipper_utils.rs` | `variable_offset_inner_ex` |
| `cz_clip_extrusion`, `cz_detect_floating` | `clipper_z.rs` | `clip_extrusion` (PerimeterGenerator.cpp), the floating-line clip of FillFloatingConcentric.cpp |
| `cz_propagate_wave` | `region_expansion.rs` | `wave_seeds` / `propagate_waves` of RegionExpansion.cpp |
| `cz_smooth_compensation_banded` | `elephant_foot_compensation.rs` | the banded solve of ElephantFootCompensation.cpp |
| `medial_axis_native` | `geometry/expolygon.rs` | `ExPolygon::medial_axis` (MedialAxis.cpp over boost Voronoi), gate `MEDIALAXIS_NATIVE` |

Geometry helpers of `clipper_utils.rs` without `_clib` in the name compute in pure Rust on a 1 um grid; none
is left on the fixtures' path (`GEOCALLS=1` lists them, README "Probes").

### `clipper2-z-sys` (Clipper2)

| Shim function | Called from | Stands for |
|-|-|-|
| `cz2_offset_z`, `cz2_intersect_open_z` | `clipper2_z.rs` | Clipper2 with Z in RegionExpansion.cpp |
| `cz2_pl_open` | `clipper2_utils.rs` | `intersection_pl_2`, `diff_pl_2` of Clipper2Utils.cpp |

### `eigen-transform-sys` (Eigen, AABB tree)

| Shim function | Called from | Stands for |
|-|-|-|
| `transform_verts_unified` (3), `transform_verts_for_slicing`, `transform_verts_affine_f32_pair` | `triangle_mesh_slicer.rs`, `multi_material_segmentation.rs`, `app_slice.rs` | `its_transform`, the vertex transform of `slice_mesh` (Eigen float evaluation order) |
| `douglas_peucker_keep` | `geometry/simplify.rs` (only with `DP_RUST=0`), `tests/douglas_peucker_native_parity.rs` | `MultiPoint::_douglas_peucker`; replaced by the Rust port in R867, kept as the test's reference |
| `project_line_on_line` | `multi_material_segmentation.rs` | the projection in `MultiMaterialSegmentation.cpp` |
| `short_edge_collapse` | `short_edge_collapse.rs` | `its_short_edge_collpase` |
| `sample_uniform` | `triangle_set_sampling.rs` | `sample_its_uniform_parallel` |
| `raycast_visibility_native` | `gcode/seam_placer.rs` | the visibility rays of SeamPlacer.cpp (AABBTreeIndirect) |

### `boost-voronoi-sys`

Not called by the engine. One example (`examples/voronoi_ab.rs`) uses it. It can go when the medial axis no
longer needs a second opinion.

## 2. Child processes

| Where | What | Disposition |
|-|-|-|
| `gcode/post_processor.rs` | the user's post-processing scripts | as C++ does (PostProcessor.cpp runs a shell); stays |
| `mac_utils.rs`, `platform.rs` | `sw_vers`, `sysctl` | stand-ins for the Objective-C and sysctl calls of the C++; to replace with the system calls |
| `slicer-cli-rust/src/parity/bambu.rs` | `slicer_cli`, for `slicer-parity` | the oracle; not part of the engine |
| `slicer-cli-rust/src/parity/profiles.rs` | `jsonnet`, for the job configs | test tooling; not part of the engine |

## 3. Structure the port does differently on the fixtures' path

Found while localizing; each one is a place where the score can agree today and still break on another model.

| Item | C++ | Port | Since |
|-|-|-|-|
| The description of an extrusion | `extrude_entity(entity, "perimeter" / "infill" ...)` decides seam placement and speed | not carried; the role stands in for it (`LOOP_SPLIT_NONPERIMETER`) | R858 |
| Speed of a path | one `calc_speed_by_role` in `_extrude` (GCode.cpp:7702-7790) | three places: the collection's feature speed, `extrude_loop`'s per-path speed, the single-path branch of `extrude_entity` | R860 |
| Which filler takes a surface | `Fill::new_from_type` per pattern; 48 calls of `FillRectilinear::fill_surface_by_lines` on Benchy | 123 calls of the port's `fill_surface_by_lines`: patterns C++ gives to other classes go through the rectilinear filler | R859 |
| `extrude_multi_path` | its own function | a no-sort collection of the paths, then the wipe path | R861 |
| libc++ container order | `std::unordered_map` iteration | emulated twice, for `double` keys (`region_expansion.rs`) and for `Point` keys (`perimeter_generator.rs`); one generic type would do | R860 |
| Non-determinism of the oracle | multi-material segmentation differs run to run on 80 of Majora's 656 layers | the port is deterministic; Majora's ceiling against one kept oracle is 99.87 to 99.92% | R855 |

## 4. Library not ported

From the file map at R862: 33 C++ units have no Rust file (668 functions), and 1,929 functions of 25 tokens or
more are missing from a paired Rust file and from every other. `crates/libslic3r-rs/PORT_LEDGER.json` marks
105 units `partial` and 4 `deferred`. The ones a slicing job can reach:

| Unit | Reached by | State |
|-|-|-|
| `Fill/FillRadialZigZag`, `Fill/FillConformal`, `Fill/FillConformalChart` | infill patterns of 02.09 | no Rust file |
| `GCode/BedExcludeChecker`, `ColorDecomposeRecipe` | pre-slice checks, mixed colours | no Rust file |
| `Support/*` (5 units) | any job with supports | `partial`; the three fixtures have `enable_support = 0` |
| `Fill/FillAdaptive`, `Fill/FillLightning`, `Fill/Lightning/Generator` | those patterns | `partial`; the fixtures use grid and gyroid |
| `Brim`, `GCode/AvoidCrossingPerimeters`, `GCode/ConflictChecker` | brims, `reduce_crossing_wall` (0 in the fixtures), the conflict check | `partial` |
| `discover_sub_top_surfaces`' callers, `combine_infill`, the 74 newer setting keys | 02.09 | README, "What the port has to catch up with" |
| `Format/AssimpImport`, `Format/AssemblyStepsJson` | model and project loading | no Rust file. (`Format/3mf` is listed by the map as unpaired because its Rust file is named `format/three_mf.rs`.) |
| SLA (23 units) | SLA printers | out of scope for `slicer_cli`, which is FFF |

## 5. Closed

| Round | Item |
|-|-|
| R856 | every helper on the 1 um grid on the fixtures' path computes at coord_t |
| R867 | `douglas_peucker_keep` of `eigen-transform-sys` is no longer on the engine's path: `multi_point::douglas_peucker_faithful` gives the same bytes on the fixtures and the same result as the C++ on 400,000 test polylines |
| R861 | `ExtrusionMultiPath` is an entity (`ExtrusionEntityType::MultiPath`); ConflictChecker, LayerRegion, PrintExtents and ShortestPath dispatch on it as C++ does |
