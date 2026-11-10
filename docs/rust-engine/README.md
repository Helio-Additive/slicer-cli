# The Rust engine

`crates/libslic3r-rs` (crate `slicer`) is a line-by-line Rust port of Bambu
Studio's C++ `libslic3r`. The aim is G-code byte-identical to `slicer_cli` for
the same job. It is not there: the scores are below. `slicer_cli-rust` is the
program built from it; it is not packaged.

## What is here

| Path | What it is |
|-|-|
| `crates/libslic3r-rs/` | The port. `src/` mirrors libslic3r's file layout. `HANDOFF.md` has the test status, `PORT_LEDGER.md` which units are ported |
| `crates/clipper-z-sys/`, `clipper2-z-sys/`, `eigen-transform-sys/`, `boost-voronoi-sys/` | C++ behind C shims, where pure Rust could not match the C++ engine bit for bit: Bambu's ClipperLib_Z, Clipper2 with `USINGZ`, Eigen's f32 transforms, boost::polygon voronoi |
| `crates/slicer-cli-rust/` | `slicer_cli-rust` (`src/engine_cli.rs`) and `slicer-parity` (`src/parity/`) |
| `scripts/parity/` | The scoring scripts |
| `tests/rust-engine/` | `test_slicer_cli_rust.sh`, `test_semantic_parity.sh`, the job configs, the fixtures |
| `docs/rust-engine/` | This file; the round log (`PARITY_STATUS.md` to R471, `parity/rounds-R472-R808.md` after); `PORTING_PATTERNS.md`; the working notes of the branch the port was developed on |

The port was developed on the branch `alex/frame-unify` (1,233 commits, rounds
R1 to R808), which forked from `main` before the three-engine front end. The
older notes here still use that branch's paths:

| In the older notes | Now |
|-|-|
| `libslic3r/bambustudio/references/BambuStudio` | `references/BambuStudio` |
| `libslic3r/bambustudio/build/slicer_cli` | `build-bambu/slicer_cli` |
| `slicer-cli compare`, `slicer-cli slice --engine` | `slicer-parity compare`, `slicer-parity slice --engine` |
| `scripts/*.py` | `scripts/parity/*.py` |
| `tests/configs/`, `tests/.tmp/` | `tests/rust-engine/configs/`, `tests/rust-engine/.tmp/` |
| `native.gcode` in `COMPARE_KEEP_DIR` | `bambu.gcode` |
| `devbox run -- cargo ...` | `cargo ...` (see "Build") |
| "the bun suite" | not brought over; it tested the branch's wrapper program |

## Build

```sh
git submodule update --init references/BambuStudio
cargo build -p slicer-cli-rust     # target/debug/slicer_cli-rust, target/debug/slicer-parity
```

The dev profile compiles the engine at `opt-level = 3` with debug assertions
off: the port mirrors C++ `assert()` as `debug_assert!`, and the C++ it is
scored against is a release build. Tests keep the assertions on.

**The Eigen version changes the G-code.** `eigen-transform-sys` compiles its
mesh-simplification kernels against the Eigen that `pkg-config eigen3` (or
`EIGEN3_INCLUDE_DIR`) names, because they have to round as the C++ engine's
build does. The recorded scores were taken with Eigen 3.4.0. Measured on
Benchy, same sources:

| Compiler | Eigen | Boost | Output |
|-|-|-|-|
| clang 21.1.8, rustc 1.94.1 | 3.4.0 | 1.87 | the R808 bytes |
| Apple clang 17, rustc 1.95.0 | 3.4.0 | 1.87 | the R808 bytes |
| Apple clang 17, rustc 1.95.0 | 3.4.0 | 1.90 | the R808 bytes |
| Apple clang 17, rustc 1.95.0 | 5.0.1 | 1.87 or 1.90 | 14,023 lines differ; 114,124 lines in order against the 02.06.00.51 engine, not 121,714 |

The right Eigen for scoring against a given `slicer_cli` is the one that
`slicer_cli` was built with. The scores from R809 on were taken with both
programs built against Homebrew's Eigen 5.0.1; Ubuntu 22.04's `libeigen3-dev`
is 3.4.0, which is what the R808 scores against the 02.06.00.51 engine used.

## The programs

`slicer_cli-rust [--config FILE] [-o FILE] [-v] MODEL` slices an STL (with a
settings bundle) or a Bambu Studio 3MF. It refuses every other flag by name
([cli-reference.md](../cli-reference.md), "slicer_cli-rust").

`slicer-parity` runs one job through both engines:

```sh
slicer-parity compare --config tests/rust-engine/configs/benchy-016.jsonnet
slicer-parity slice --engine bambu|rust --config <job>
```

A job config is JSON or Jsonnet (`jsonnet` on `PATH`): the model, and for an STL
the machine, filament and process profiles, which are resolved through their
`inherits` chains into one flat settings bundle. Both engines get that same
bundle: `slicer_cli --config <bundle> --output <file> <model>` and the Rust
engine in-process. `slicer_cli` is looked for in `$BAMBUSTUDIO_SLICER`, beside
the program, in `build-bambu/` and in `build/`. `COMPARE_KEEP_DIR=<dir>` keeps
`bambu.gcode` and `rust.gcode`.

## Fixtures

| Fixture | Job config | Model |
|-|-|-|
| Benchy classic | `benchy-016.jsonnet` | `tests/rust-engine/fixtures/3DBenchy.stl`, not in the repository: `curl -L -o tests/rust-engine/fixtures/3DBenchy.stl https://helioadditive-public.s3.ap-east-1.amazonaws.com/3DBenchy.stl` |
| Benchy arachne | `benchy-016-arachne.jsonnet` | the same |
| Majora | `nu3mf.jsonnet` | a painted eight-colour MakerWorld 3MF with a prime tower, not redistributable; put it at `tests/rust-engine/fixtures/28_MAJORASMASK_FULLCOLOUR_Makerworld_1plate_multicol.3mf` |
| Cube | `stl-cube-config.jsonnet` | `tests/rust-engine/fixtures/Cube_25.6.stl` |
| Painted cube | `painted-cube.jsonnet` | `crates/libslic3r-rs/tests/data/painted_cube.3mf` |

The STL job configs take their profiles from `references/BambuStudio`, so the
settings move with the engine pin.

## Parity workflow

1. `COMPARE_KEEP_DIR=<dir> slicer-parity compare --config <job>`.
2. `python3 scripts/parity/seq_parity.py <dir>/rust.gcode <dir>/bambu.gcode`.
   It counts lines that are the same to 1e-3 mm and reachable in order, per
   (layer, feature) group. It is a lower bound, and the headline score.
   `line_parity.py` (content, order ignored), `order_loss_kinds.py`,
   `island_order.py` and `semantic_compare.py` (is it the same printed object)
   look at the same pair from other sides.
3. Find the first stage where the two engines differ before changing anything:
   dump the same intermediate from both.
4. Land the change behind an env gate (`faithful_gate`, `opt_in_gate` in
   `crates/libslic3r-rs/src/lib.rs`), state every fixture's score before and
   after in the commit, and log the round.

`slicer_cli` is not reproducible run to run to the line: two runs of the same
C++ binary moved the Majora score by about 900 lines and Benchy arachne by 19.

## Scores

In-order matched lines (`seq_parity.py`) against this repository's `slicer_cli`
(Bambu Studio 02.09.01.70), both programs built on one machine with the same
Eigen (5.0.1) and Boost (1.90). `slicer_cli` is not reproducible run to run, so
one oracle G-code per fixture is kept and every round is scored against it. The
round log is [`parity/rounds-R809-on.md`](parity/rounds-R809-on.md).

| Fixture | As brought over (R808) | Now (R818) |
|-|-|-|
| Benchy classic | 30,486 (15.72%) | 149,451 (80.36%) |
| Benchy arachne | 32,769 (13.68%) | 158,538 (67.00%) |
| Majora | 1,512,438 (55.88%) | 1,796,767 (66.27%) |
| Majora, prime tower | 83.5% | 83.6% |

Against the engine the port was written for (02.06.00.51, with the branch's own
driver) R808 scored Majora 66.06%, Benchy classic 121,714 lines and Benchy
arachne 138,771 lines. Those numbers are not comparable with the table: the
oracle is a different engine version with a different front end, and the Benchy
jobs now take the 02.09.01.70 profiles.

The oracle these numbers are taken against is a macOS arm64 build. Two of its
results depend on that: the order of a `std::unordered_map` (libc++) decides
the sign of a bridge direction with tied costs, and clang fuses multiply-adds
in `float` expressions on arm64. The port reproduces both behind the gates
`BRIDGE_DIR_HASH_ORDER` and `PCA_FP_CONTRACT`; against a Linux or x86-64 build
of `slicer_cli` those two would have to follow that build instead.

Where each fixture stands, from the `PP` probe (below):

- **Benchy classic.** Slices identical on 299 of 300 layers, perimeters on 298
  of 300, gap fills on 248 of 254. The fill surfaces the perimeter step leaves
  are identical on 197 of 204 layers, the top surfaces on all 48 through
  `prepare_infill`, the sub-top surfaces on all 3, the bottom bridges on all 4.
  The internal bridges of `bridge_over_infill` are identical on all 11 layers,
  and the fills on 185 of 289 layers. Walls are 86% in order; top surface 92%,
  solid infill 79%, bridges 54%, sparse infill 47%.
- **Benchy arachne.** Slices as classic. 2,695 of 2,721 `WallToolPaths` calls
  get the oracle's outline and return its toolpaths to within two units. 88.5%
  of the lines match in content, 67.0% in order: walls 65 to 71%, solid infill
  66%, bridges 72%.
- **Majora.** The merged slices are identical on 582 of 656 layers, the region
  slices on 574. 77.1% of the lines match in content, 66.3% in order.

## Finding the first stage that differs

`PP=1` makes the Rust engine print one checksum line per layer and stage
(`crates/libslic3r-rs/src/stage_dump.rs`): after `slice()`, after
`make_perimeters()`, after each step of `prepare_infill()` and after `infill()`.
`PPL=<layer>` adds every perimeter entity and fill-surface point of one layer.

The C++ side is [`probes/pp_probe.hpp`](probes/pp_probe.hpp). To use it, copy
`references/BambuStudio/src/libslic3r/PrintObject.cpp` to
`libslic3r/bambustudio/libslic3r/PrintObject.cpp` (the override layer), include
the header there, and call `Slic3r::pp::slices(m_layers)` after `this->slice()`
in `make_perimeters`, `pp::perimeters` before `set_done(posPerimeters)`,
`pp::typed("<stage>", m_layers)` after each step of `prepare_infill` (stage
names `a_detect`, `b_prepfill`, `c_vshell`, `d_external`, `e_hshell`, `f_clip`,
`g_bridge`, `h_combine`, `i_subtop`) and `pp::infill` before
`set_done(posInfill)`. Configure and build that in a build tree of its own, not
the one the oracle comes from, give it to `slicer-parity slice --engine bambu
--bambu-binary`, and delete the copied file afterwards: it must never be
committed, and a configure run of any other build tree would pick it up.

`FBIS=1` prints the same kind of line for the stages inside the classic
perimeter generator that lead to the fill boundary (`last`, `notfilled`,
`infillraw`, `topfills`, `fillclip`, `topinfill`, `final`, `nooverlap`, and
`t_*` inside the top-one-wall branch), and `FBISL=<layer>` every point of one
layer; the C++ twin is the same `fprintf` at the same places of a probed
`PerimeterGenerator.cpp`.

Sort the `PP` lines of both engines and diff them. A stage whose lines all
agree is done; the first stage with differing layers is where to look.

## What the port has to catch up with

`main` moved the Bambu Studio engine twice while the port was developed
(02.06.00.51 → 02.08.01.55 → 02.08.02.61 → 02.09.01.70), and its front end now
does work the branch's driver did not.

Engine, `references/BambuStudio/src/libslic3r` between the port's pin `b506005`
and `56a57d0`:

- To 02.08.02.61: 111 files, +19,710/−1,904. Most in `GCode.cpp`,
  `FilamentGroup.cpp`, `Print.cpp`, `PrintConfig.cpp`, `ToolOrdering.cpp`,
  `TreeSupport3D.cpp`, `MultiNozzleUtils.cpp`, `GCodeProcessor.cpp`,
  `AvoidCrossingPerimeters.cpp`, `PerimeterGenerator.cpp`, `PrintApply.cpp`,
  `TriangleMeshSlicer.cpp`, `GCodeWriter.cpp`.
- To 02.09.01.70 (`engine-bump-expected-diff.md`): 217 more files,
  +36,554/−5,339. 53 new settings and `nozzle_type` gone; the sub-top-surface
  pattern defaults to monotonic; top-surface travel optimisation; the
  multi-nozzle rework in `PrintObject`, `ToolOrdering`, `WipeTower` and
  `TreeSupport`; conformal infill; mixed nozzle diameters.
- Ported from that range so far: the machine G-code placeholders of 02.09
  (R810), farthest-point timelapse (R812). Known and not ported:
  `discover_sub_top_surfaces` and the `stSubTop` surface type, `combine_infill`,
  the settings block's 74 new keys, the time estimate behind the `M73` lines.
- The override layer: `libslic3r/bambustudio/libslic3r/Fill/FillFloatingConcentric.cpp`
  fixes five out-of-bounds reads in the engine's floating-concentric fill. The
  port should follow the fixed code; R801 found the unfixed code reading freed
  memory.

Front end (`main.cpp`, `cli_*.cpp`), each of which changes what the engine is
given. Ported so far: placement of an STL on the plate (R809), the alias of
`initial_no_support_filament_id` (R810), `is_BBL_Printer` (R810). Open:

- Plate type and plate origin (#35).
- `Model::setExtruderParams` and `setPrintSpeedTable` filled before
  `process()`; `Brim.cpp` reads them (#35).
- Per-filament setting arrays extended to the project's filament count (#25).
- The nozzle map: derived only when the input gave one; automatic grouping
  otherwise (#22, #45).
- `filament_map_mode` derived from the printer's nozzle counts (#11).
- A height range without `layer_height` takes the object's (#55).
- Nozzle inventory and per-slot keys for a settings-file input (#56).

`--simplify`, `--gcode-timing`, `--layout-plan`, calibration and the rest of
the command line are refused by `slicer_cli-rust` by name until ported.

## Tests

```sh
cargo test -p slicer-cli-rust
bash tests/rust-engine/test_slicer_cli_rust.sh target/debug/slicer_cli-rust
cargo test --workspace --no-run      # compiles; crates/libslic3r-rs/HANDOFF.md lists what fails
bash tests/rust-engine/test_semantic_parity.sh   # needs slicer_cli, numpy, jsonnet, Benchy
```

## Before this is published

The working notes brought from the branch (`PARITY_STATUS.md`,
`semantic-parity.md`, `main-cpp-correspondence.md`, `parity/R*.md`,
`crates/libslic3r-rs/PARITY.md`) and a few comments and probe paths under
`crates/libslic3r-rs/src` still name temporary files and private notes of the
machine they were written on. They need the same scrub `HANDOFF.md` had.
