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

Ubuntu 22.04's `libeigen3-dev` is 3.4.0; Homebrew's `eigen` is 5.x, so on
macOS point `EIGEN3_INCLUDE_DIR` at a 3.4.0 tree to reproduce a recorded score.
The right Eigen for scoring against a given `slicer_cli` is the one that
`slicer_cli` was built with.

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

In-order matched lines (`seq_parity.py`), the Rust engine at R808. The Rust
G-code of this branch is byte-identical to the R808 build's on all three
fixtures (same settings bundle, Eigen 3.4.0).

| Fixture | Against the engine the port follows (02.06.00.51, the branch's driver) | Against this repository's `slicer_cli` (02.09.01.70) |
|-|-|-|
| Majora | 1,788,608 (66.09%); recorded at R808: 1,787,698 (66.06%) | 1,511,749 (55.86%) |
| Majora, prime tower | 83.0%; recorded: 83.0% | 83.5% |
| Benchy classic | 121,714 (77.56%); recorded: 121,714 | 50,966 (32.48%) |
| Benchy arachne | 138,790 (65.44%); recorded: 138,771 | 47,623 (22.45%) |

Why the right-hand column is lower. None of it is a change in the Rust engine.

- **The C++ engine moved.** The 02.06.00.51 C++ G-code itself matches the
  02.09.01.70 C++ G-code only 74.1% in order on Majora (outer wall 76.0%, inner
  wall 70.4%, floating vertical shell 26.5%, sparse infill 80.2%, prime tower
  97.4%).
- **`slicer_cli` now places an STL on the plate as the desktop does.** Benchy
  comes out centred on the H2D plate, 174.176 mm and 160 mm from where the
  branch's driver and the Rust engine put it, so every coordinate differs. With
  the Rust G-code translated by that offset, Benchy classic scores about 82,200
  (52.4%) and Benchy arachne about 86,900 (41.0%); the translation is not on
  the 1e-3 grid, so these are approximate. A 3MF carries its placement, which is
  why Majora is not affected.
- **`slicer_cli` reshapes the settings before the engine sees them**: on the
  two-nozzle H2D the per-filament arrays are extended to one entry per nozzle,
  and 74 settings exist that 02.06 did not have.

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
- The override layer: `libslic3r/bambustudio/libslic3r/Fill/FillFloatingConcentric.cpp`
  fixes five out-of-bounds reads in the engine's floating-concentric fill. The
  port should follow the fixed code; R801 found the unfixed code reading freed
  memory.

Front end (`main.cpp`, `cli_*.cpp`), each of which changes what the engine is
given:

- Placement of an STL on the plate, plate type and plate origin (#35).
- `Model::setExtruderParams` and `setPrintSpeedTable` filled before
  `process()`; `Brim.cpp` reads them (#35).
- Per-filament setting arrays extended to the project's filament count (#25).
- The nozzle map: derived only when the input gave one; automatic grouping
  otherwise (#22, #45).
- `filament_map_mode` derived from the printer's nozzle counts (#11).
- A height range without `layer_height` takes the object's (#55).
- `initial_no_support_filament_id` aliased in custom G-code (#6).
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
