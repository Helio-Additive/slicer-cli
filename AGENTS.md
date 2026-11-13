# AGENTS.md

For coding agents working in this repository. Short and exact; the linked files
are the authority.

## What this is

A command-line slicer: one front end, three slicing engines. It ships three
programs, `slicer_cli` (Bambu Studio engine, source pin `56a57d0`),
`slicer_cli-orcaslicer` (OrcaSlicer engine, pin `8500fcd`) and
`slicer_cli-snapmaker` (Snapmaker Orca engine, a Snapmaker fork of OrcaSlicer,
pin `bac6fd5`), for Linux x86_64,
Linux arm64, macOS arm64 and Windows x86_64. Flags that share a name with the desktop apps'
own command lines behave the same way.

A fourth program is in progress and is not packaged: `slicer_cli-rust`, a Rust
port of the Bambu Studio engine (`crates/`, built by cargo). Its G-code does not
match `slicer_cli`'s yet. Its own section is "The Rust engine" below;
[docs/rust-engine/README.md](docs/rust-engine/README.md) is the authority for it.

Read first, and change here rather than copy: [README.md](README.md),
[CONTRIBUTING.md](CONTRIBUTING.md), [docs/cli-reference.md](docs/cli-reference.md),
[docs/building.md](docs/building.md), `.github/workflows/slicer-cli-ci.yml`
(the full, tested recipe), `CMakeLists.txt`.

## Layout

| Path | What it is |
|-|-|
| `main.cpp` | The front end: option parsing, run modes, the engine-specific blocks (`ENGINE_BAMBU` / `ENGINE_ORCA` / `ENGINE_SNAPMAKER`) |
| `cli_snapmaker.cpp` | The Snapmaker Orca build's desktop rules (mixed filaments, paint remap, pre-slice checks, the prime tower) and its profile lookup |
| `cli_command_line.cpp`, `cli_flag_support.cpp` | Flag table, `--help`, the refusal sentences for flags a program does not take |
| `cli_load_settings.cpp`, `desktop_presets.cpp` | Preset/profile resolution, `inherits` chains, desktop-equivalent selection |
| `cli_model_load.cpp`, `cli_assemble.cpp`, `cli_run_steps.cpp`, `cli_repetitions.cpp` | Model and 3MF loading, plate assembly, the run itself, repeated plates |
| `cli_pipe.cpp` | The official Linux progress pipe |
| `layout_plan.cpp`, `calib_args.cpp`, `startup_logging.cpp` | `--layout` planning, calibration arguments, startup log |
| `libslic3r/bambustudio/`, `libslic3r/orcaslicer/`, `libslic3r/snapmaker/` | The override layer: files that replace engine source of the same relative path |
| `references/BambuStudio`, `references/OrcaSlicer`, `references/SnapmakerOrca` | The pinned engine submodules. Read-only, upstream as-is |
| `tests/` | The test scripts, run against built or packaged programs |
| `docs/` | `cli-reference.md` (every flag, outputs, `result.json`, event lines, return codes), `building.md` |
| `ci/` | vcpkg overlay port and patches the build applies |
| `Cargo.toml`, `crates/` | The Rust engine: `libslic3r-rs` (the port, crate `slicer`), four `*-sys` FFI shims, `slicer-cli-rust` (the programs `slicer_cli-rust` and `slicer-parity`) |
| `scripts/parity/`, `tests/rust-engine/`, `docs/rust-engine/` | The Rust engine's scoring scripts, tests and job configs, and records |
| `.github/workflows/slicer-cli-ci.yml` | Build, test, package, release |

The build picks its engine with the CMake cache variable `ENGINE`
(`-DENGINE=bambu` default, `-DENGINE=orca`, `-DENGINE=snapmaker`;
`CMakeLists.txt`), which defines `ENGINE_BAMBU`, `ENGINE_ORCA`, or
`ENGINE_ORCA` and `ENGINE_SNAPMAKER` together for the front-end blocks: the
Snapmaker build is an OrcaSlicer build, and `ENGINE_SNAPMAKER` marks what only
it does. The Bambu Studio and OrcaSlicer engines build a program called
`slicer_cli`, and packaging renames the OrcaSlicer one; the Snapmaker engine
builds `slicer_cli-snapmaker`.

## Build and test

One engine per build tree; build three times for all three. Build per operating system and
engine by [docs/building.md](docs/building.md): its "Dependencies" and "Configure
and build" sections carry the pinned versions, the per-platform recipe and the
prefix each engine needs. The tested recipe is the `Build & Test` job of
`.github/workflows/slicer-cli-ci.yml`, in steps `Install dependencies (macOS)`,
`Install dependencies (Linux)`, `Install dependencies (Windows)`, `Configure
(macOS / Linux)`, `Configure (Windows)` and `Build`.

The job and step names quoted below are those in that workflow, with the
`${{ matrix.* }}` suffix left off where a name is parameterised. Every build and
test job is skipped on a draft pull request (`github.event.pull_request.draft ==
false`), so mark a PR ready for review before reading its checks. A change that
touches only markdown other than `README.md`, or files under `docs/`, starts no
CI run: both triggers carry an ordered `paths` filter that excludes `**/*.md`
and `docs/**` and re-includes `README.md`, which the packages, CPack and the
install rules copy. `LICENSE` and `NOTICE` are never excluded, for the same
reason.

Tests. The build-tree tests take the program the build produced; CI runs them on
Linux and macOS, never on Windows, in steps `Run structured-diagnostics event
tests`, `Run feature-exclusion regression tests`, `Test diagnostic transport`
and `Run nozzle-map provenance regression tests (bambu)`:

```sh
bash tests/test_diagnostic_events.sh build/slicer_cli        # engine read from the binary
bash tests/test_excluded_features.sh build/slicer_cli
python3 tests/test_diagnostic_transport.py                   # no arguments; compiles its own probe
bash tests/test_nozzle_map_provenance.sh build/slicer_cli    # Bambu build only
```

The package tests take the packaged programs, from the `Package` job (macOS and
Windows) and the `Package - Linux-x86_64` and `Package - Linux-arm64` jobs. Linux keeps them in `bin/` (step
`Assemble and validate Linux package`); macOS (step `Assemble + validate package
(macOS arm64)`) and Windows (step `Assemble + validate package (Windows x86_64)`)
put them at the package root, with `.exe` on Windows:

```sh
# every OS, the first two programs, in that order
bash tests/test-complete-slice-path.sh <slicer_cli> <slicer_cli-orcaslicer>
# macOS and Windows; the second and third programs are optional
bash tests/test-packaged-slice.sh <slicer_cli> [<slicer_cli-orcaslicer> [<slicer_cli-snapmaker>]]
# macOS only; both programs required
bash tests/test_list_settings.sh <slicer_cli> <slicer_cli-orcaslicer>
# Linux package only; takes the package root and checks the bin/ layout
bash tests/test-portable-bundle.sh <package root>
```

`tests/test-complete-slice-path.sh` covers the Bambu Studio and OrcaSlicer engines: the flags, plate type
rules, model formats, `result.json` and exported projects. Flags and outputs are
documented in [docs/cli-reference.md](docs/cli-reference.md); a first build
compiles about 1,500 engine files.

## Rules

- **Never edit anything under `references/`.** The pinned engine sources stay
  byte-identical upstream so an engine bump stays easy. A `libslic3r` fix goes
  in the override layer — a file at the same relative path under
  `libslic3r/bambustudio/`, `libslic3r/orcaslicer/` or `libslic3r/snapmaker/`, which CMake compiles in
  place of the engine's file ([CONTRIBUTING.md](CONTRIBUTING.md), "Engine
  source"; docs/building.md, "The override layer") — or in the front end.
  An override freezes its file at the commit it was taken from; a bump has to
  re-apply the change to the new file.
- **Port, don't invent.** Behaviour follows the desktop app's code at the
  pinned references. A commit that ports behaviour cites the upstream file and
  line numbers (`BambuStudio.cpp 1234`, `OrcaSlicer.cpp 1234`).
- **All engines, all three operating systems, every change.** No single-OS or
  single-engine code path. Where the engines differ, follow each engine's own
  code and say which one a line applies to; engine-only code is guarded by
  `#ifdef ENGINE_ORCA` / `#ifndef ENGINE_ORCA`, and Snapmaker-only code by
  `#ifdef ENGINE_SNAPMAKER` (in `cli_snapmaker.cpp` where it is more than a
  branch). A change for the Snapmaker build must leave the other two programs'
  output as it was.
- **New behaviour gets a new flag; an existing flag keeps its meaning.** What
  the program prints — messages, event kinds and tags, `result.json` keys,
  return codes — is read by other programs: add, never change.
- **Refusals are clear sentences that say what to do**, and name the program
  that fits when the other engine does (`cli_flag_support.cpp`): a flag a
  program does not take is refused by name, before anything loads, and points
  at `slicer_cli`, `slicer_cli-orcaslicer` or `slicer_cli-snapmaker` as appropriate.
- **A test proves its fix:** it fails before the change and passes after.
- **No machine-specific paths, hostnames or internal names** in code, docs or
  commits. This repository is public.
- C++17, the engine's conventions. New command-line code goes in its own
  `cli_*.cpp`, not in `main.cpp`.

## The Rust engine

`crates/libslic3r-rs` is a line-by-line port of Bambu Studio's `libslic3r`; the
aim is G-code byte-identical to `slicer_cli` for the same job. Nothing in the
CMake build depends on it, and no flag, default, sentence or schema of the three
C++ programs changes for it. Read
[docs/rust-engine/README.md](docs/rust-engine/README.md) before working on it:
build, fixtures, the parity workflow, the scores, and what the port still has to
catch up with.

```sh
cargo build -p slicer-cli-rust        # target/debug/slicer_cli-rust and slicer-parity
cargo test -p slicer-cli-rust
bash tests/rust-engine/test_slicer_cli_rust.sh target/debug/slicer_cli-rust
cargo test --workspace --no-run       # the engine's tests compile; some fail (HANDOFF.md)
# one job through slicer_cli and the Rust engine, then the score
COMPARE_KEEP_DIR=<dir> target/debug/slicer-parity compare --config tests/rust-engine/configs/<job>.jsonnet
python3 scripts/parity/seq_parity.py <dir>/rust.gcode <dir>/bambu.gcode
```

Rules for it, on top of the ones above:

- **The port follows the C++ at the pin**: same functions, order, names in
  snake_case, control flow, constants, rounding and tolerances. Where the
  override layer carries a `libslic3r` file, that file is the one to follow:
  it is what `slicer_cli` compiles. Cite the
  upstream file and line in the code and the commit. No stubs: a unit that
  cannot be finished is marked `partial` in `crates/libslic3r-rs/PORT_LEDGER.json`
  with what blocks it. Read `docs/rust-engine/PORTING_PATTERNS.md` before
  touching geometry.
- **Behaviour changes land behind an env gate**, so the earlier arm can be
  reproduced byte for byte (`crates/libslic3r-rs/src/lib.rs`):
  `faithful_gate("NAME")` is on unless `NAME=0`; `opt_in_gate("NAME")` is off
  unless `NAME=1`, for work that is faithful but lowers a score until its
  counterpart lands; `probe_enabled("NAME")` is for diagnostics only and must
  never change output.
- **Localize before fixing**: dump the same intermediate from both engines and
  find the first stage that differs.
- **Two sources of work**: the G-code difference (the parity workflow) and the source map,
  `docs/rust-engine/FILE_MAP.md`, which pairs every libslic3r file with its Rust file and lists per function
  what is missing, what exists only in Rust and how closely the body follows. A round takes from both: the
  biggest gaps of the map on the path the fixtures run through first, then supports and the rest. Regenerate
  the map in every round that touches the port (`python3 scripts/parity/file_map.py`; `--check` tells whether
  the committed copy is current) and commit it with the round.
- **The checklist is where work is chosen from**: `docs/parity/PORT_CHECKLIST.md` lists every C++ file of
  `libslic3r` with its classes and functions, the Rust file each maps to and a status (not started, partial,
  ported, verified), with the overall percentage at the top. The aim is a Rust file for every C++ file, with
  the same function, type and method names and the same logic, and no FFI shim left. Regenerate it in every
  round (`python3 scripts/parity/port_checklist.py`; `--check`) and commit it with the round. A function is
  *verified* only with evidence, recorded in `scripts/parity/port_checklist_verified.json`: a twin probe that
  agreed with the C++ over a fixture, or a test that compares with C++ output. The fixtures' byte parity and
  the engine tests are how a round's work is checked.
- **A Rust file is named after the C++ file it ports**, word for word, in snake_case. The map flags the
  ones that are not.
- **A round states its scores**: commits are `R<round>: <what>` with the
  in-order score of every fixture before and after. A round that finds nothing
  is still logged in `docs/rust-engine/parity/`, so it is not chased again.
- **Do not lower a fixture's score.** The scores depend on the Eigen and Boost
  headers the shims are built with; use the ones docs/rust-engine/README.md
  names when comparing with recorded numbers.
- `slicer_cli-rust` refuses, by name, every flag it does not take. A flag it
  gains must behave as the same flag of `slicer_cli` does.

## Releases

Tags are `slicer-cli/v<version>`; the `.github/workflows/slicer-cli-ci.yml` job
`Publish slicer-cli release assets` publishes them. Contributors do not tag or
publish.

Everything else — the CLA, what is and is not accepted, style — is in
[CONTRIBUTING.md](CONTRIBUTING.md).
