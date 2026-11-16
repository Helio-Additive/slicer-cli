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

Since R891 the engine runs every routine below that uses only the 2D library on the Rust port (`clipper.rs`);
since R892 it has no shim arm for them, and the shim keeps them as the reference of the tests. Since R893 the two
ClipperLib_Z routines, `cz_clip_extrusion` and `cz_detect_floating`, run on the Rust Z build too (`clipper_lib_z.rs`);
since R895 the engine has no shim arm for them either. Since R896 the elephant-foot solver
(`cz_smooth_compensation_banded`) runs in Rust too (`EFC_SMOOTH_RUST`). Since R898 the medial axis (`medial_axis_native`, boost Voronoi)
runs in Rust as well (`MEDIAL_AXIS_RUST`): since R903 (gates `EFC_SMOOTH_RUST` and `MEDIAL_AXIS_RUST` retired) the engine does not
depend on this crate: it is a dev-dependency, the reference of the `*_native_parity` tests. The table is what each routine stands for.

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

Every geometry helper of `clipper_utils.rs` runs on the Rust ClipperLib (`clipper.rs`) since R910-R911; the
helpers that went through geo-clipper follow ClipperUtils.cpp there, and geo-clipper is out of the crate.

### `clipper2-z-sys` (Clipper2)

Out of the engine's dependencies since R906: a dev-dependency, the reference of the `clipper2_*_native_parity`
tests (it builds the pin's Clipper2 sources for them). What its functions stood for, now in Rust:

| Shim function | Was called from | Stands for |
|-|-|-|
| `cz2_offset_z`, `cz2_intersect_open_z` | `clipper2_z.rs` | Clipper2 with Z in RegionExpansion.cpp |
| `cz2_pl_open` | `clipper2_utils.rs` | `intersection_pl_2`, `diff_pl_2` of Clipper2Utils.cpp |

#### Clipper2 1.5.2 (the pin) and 1.5.4

Until R905 `clipper2_utils.rs` reached Clipper2 through `clipper2c-sys`, which is 1.5.4; the engine pins 1.5.2 and
the port (`clipper2/`) follows the pin only. What 1.5.4 does otherwise in the code the engine reaches, found in R905
by making the port reproduce the 1.5.4 results exactly on 16,000 generated cases per function (the test and its
probe are gone with the FFI arm; this list is what they established). A bump of the pin past 1.5.2 has to port these:

| Function (1.5.4 lines) | What changed from 1.5.2 |
|-|-|
| `PointInOpPolygon`, clipper.engine.cpp:534-536, 544-546 | the side of the point is `CrossProductSign` (exact, 128 bit; new in clipper.core.h:753-793) where 1.5.2 has `CrossProduct` on doubles |
| `IsValidAelOrder`, clipper.engine.cpp:1125-1153 | `CrossProductSign` where 1.5.2 has `CrossProduct` |
| `Path2ContainsPath1`, clipper.engine.cpp:576-599 with clipper.h:716-742 | replaces `Path1InsidePath2` (1.5.2: clipper.engine.cpp:584-603): two consecutive points on the same side decide, then the cleaned paths are tried, then the midpoint of the bounds |
| `FixSelfIntersects`, clipper.engine.cpp:1646-1681 | adjacent (micro) intersections are joined, not split off; a triangle ends the loop only before it and after a split |
| `AdjustCurrXAndCopyToSEL`, clipper.engine.cpp:2112-2126 | a joined edge gets its own `TopX`, where 1.5.2 copies the `curr_x` of the edge it is joined with |
| `MoveSplits`, clipper.engine.cpp:2269-2277 (#987) | an `OutRec` is not moved into its own splits |
| `CheckSplitOwner`, clipper.engine.cpp:2941-2964 (#942) | a split without points that has splits of its own is searched first |
| `CheckSplitOwner`, same lines (#957) | a containing split that `outrec` owns is taken as the owner after it is given `outrec`'s owner, where 1.5.2 skips it (`IsValidOwner` in the condition) |
| `GetLowestClosedPathInfo`, clipper.offset.cpp:36-58, `Group::Group` :148-155 | replaces `GetLowestClosedPathIdx` (1.5.2: clipper.offset.cpp:40-57): a path without area is not the lowest path of a group, and the group's orientation comes from the same pass |
| `ClipperOffset::ExecuteInternal`, clipper.offset.cpp:610 | the finishing union takes the offsetter's `preserve_collinear_`, where 1.5.2 gives it `false` |

On 16,000 generated cases each, the 1.5.4 arm of `union_ex_2`, `offset_ex_2` and `offset2_ex_2` differed from the
pin's result in 804, 3,763 and 5,428; for the two offsets part of that was the old arm's own sequence of calls
(offset to paths and a second union for the tree; contours simplified as open paths), not the library. No fixture
reaches those functions.

### `eigen-transform-sys` (Eigen, AABB tree)

Since R883 the engine does not depend on this crate: it is a dev-dependency of `libslic3r-rs`, and the
functions below are the C++ reference of the `*_native_parity` tests. The table is kept for what each stood for.

| Shim function | Called from | Stands for |
|-|-|-|
| `transform_verts_unified`, `transform_verts_for_slicing`, `transform_verts_affine_f32_pair` | `tests/vertex_transform_native_parity.rs` only (since R875) | `its_transform`, the vertex transform of `slice_mesh` (Eigen float evaluation order) |
| `douglas_peucker_keep` | `tests/douglas_peucker_native_parity.rs` only (since R870) | `MultiPoint::_douglas_peucker`; replaced by the Rust port in R867, kept as the test's reference |
| `project_line_on_line` | `tests/project_line_on_line_native_parity.rs` only (since R875) | `project_line_on_line` of MultiMaterialSegmentation.cpp; replaced by the Rust port in R873 |
| `short_edge_collapse` | `short_edge_collapse.rs` | `its_short_edge_collpase` |
| `sample_uniform` | `triangle_set_sampling.rs` | `sample_its_uniform_parallel` |
| `raycast_visibility_native` | `gcode/seam_placer.rs` | the visibility rays of SeamPlacer.cpp (AABBTreeIndirect) |

### `boost-voronoi-sys`

Not a dependency of the engine since R884 (a dev-dependency: `examples/voronoi_ab.rs` uses it). The boost
Voronoi diagram behind the medial axis is compiled inside `clipper-z-sys`.

Since R897 the builder itself is ported: `boost_polygon_voronoi.rs` equals boost's diagram in every vertex bit,
link and flag on 110,546 inputs (`tests/boost_polygon_voronoi_native_parity.rs`, for which this crate is the
reference). The medial axis runs on it since R898, Arachne's skeletal trapezoidation and the multi-material segmentation
since R901 (`ARACHNE_VORONOI_PORT`, `MMSEG_VORONOI_PORT`); the `boostvoronoi` crate is left for the gate-off arms
and the older helpers.

Clipper2 is ported as far as the engine reaches it (R904-R905: `clipper2/clipper_core.rs`, `clipper_engine.rs`,
`clipper_offset.rs`, `clipper_h.rs`, both builds, equal to the pin's Clipper2 1.5.2) and since R906 the engine
calls nothing else for it. Not ported, not reached: clipper.rectclip.cpp, `ClipperD`, clipper.minkowski.h.

### C++ that comes in through published crates

None since R911. `cargo tree -e normal -p slicer-cli-rust` lists no C++ library: the last one, `clipper-sys`
(Clipper 6, through `geo-clipper`), served sixteen helpers of `clipper_utils.rs` (`union`, `xor`, the polyline
offsets, `offset_polygon`, `offset_expolygon(s)`, `offset_polygons` and their round and miter variants,
`intersection_base`, and the old arms of `union_ex`, `intersection`, `difference`). R910 put them on the Rust
ClipperLib following ClipperUtils.cpp (415-419, 567-571, 737-838; compared with the pin in
`tests/clipper_utils_geo_free_native_parity.rs`) behind `GEO_CLIPPER_CLIB`, and R911 removed that gate, the geo
arms and both crates. The four `*-sys` crates of this repository remain dev-dependencies only, as the C++
reference of their tests.

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
| R903 | `clipper-z-sys` is out of the engine's dependencies (a dev-dependency for the tests): ClipperLib, ClipperLib_Z, the medial axis over boost Voronoi and the elephant-foot solver all run in Rust |
| R906 | `clipper2-z-sys`, `clipper2c-sys` and the `clipper2` crate are out of the engine's dependencies: Clipper2 runs in Rust (`clipper2/`, R904-R905) |
| R898 | the medial axis (`annotate_inside_outside`, `MedialAxis`, `ExPolygon::medial_axis`) runs in Rust on the ported Voronoi builder (`MEDIAL_AXIS_RUST`): no call of `clipper-z-sys` is left on the default path |
| R897 | boost::polygon's Voronoi builder is ported (`boost_polygon_voronoi.rs`), equal to boost on 110,546 diagrams; not yet called by the engine |
| R896 | the elephant-foot solver runs in Rust (`EFC_SMOOTH_RUST`) |
| R893 | `ClipperLib_Z` (the XYZ build) is ported as a second compilation of `clipper.rs`, and `clip_extrusion` / `detect_floating` run on it (gate `CLIPPER_Z_RUST`, retired in R895) |
| R889-R891 | ClipperLib is ported (`clipper.rs`: the sweep, PolyTree, ClipperOffset, equal to the library on 320,000 cases) and the engine's 2D polygon operations run on it: 19 of the 23 `clipper-z-sys` functions are out of the engine (gate retired in R892) |
| R884 | `boost-voronoi-sys` removed from the engine's dependencies (a dev-dependency for one example) |
| R883 | `eigen-transform-sys` removed from the engine's dependencies (a dev-dependency for the tests that compare with the C++) |
| R882 | `raycast_visibility_native`, the last `eigen-transform-sys` function on the engine's path, is replaced (`RAYCAST_RUST`, `AABB_EIGEN_FLOAT`): `raycast_visibility` equals the C++ on the Benchy's 30,000 samples in `tests/raycast_visibility_native_parity.rs` |
| R880 | `short_edge_collapse` of `eigen-transform-sys` is off the engine's path (`SEC_RUST`): `its_short_edge_collpase` equals the C++ on the Benchy and 106 synthetic meshes in `tests/short_edge_collapse_native_parity.rs` |
| R876 | `sample_uniform` of `eigen-transform-sys` is off the engine's path (`SAMPLE_RUST`): `sample_its_uniform_parallel` is bit-equal to the C++ in `tests/triangle_set_sampling_native_parity.rs` |
| R874 | the three vertex transforms of `eigen-transform-sys` are off the engine's path (`XF_RUST`): `make_trafo_for_slicing` and the f32 products are in `triangle_mesh_slicer.rs`, bit-equal to the C++ in `tests/vertex_transform_native_parity.rs` |
| R873 | `project_line_on_line` of `eigen-transform-sys` is off the engine's path: the Rust port gives the same bytes on the fixtures and the same result as the C++ on 900,000 test cases |
| R867 | `douglas_peucker_keep` of `eigen-transform-sys` is no longer on the engine's path: `multi_point::douglas_peucker_faithful` gives the same bytes on the fixtures and the same result as the C++ on 400,000 test polylines |
| R861 | `ExtrusionMultiPath` is an entity (`ExtrusionEntityType::MultiPath`); ConflictChecker, LayerRegion, PrintExtents and ShortestPath dispatch on it as C++ does |
