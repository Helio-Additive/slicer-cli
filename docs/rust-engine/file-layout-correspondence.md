# libslic3r file-layout correspondence (C++ → Rust)

For maintainers who know the BambuStudio C++ tree and need to find the
equivalent Rust code. This documents the **naming and directory rules**, the
**measured coverage**, and every **deliberate divergence**.

Companion doc: [`main-cpp-correspondence.md`](./main-cpp-correspondence.md)
maps the CLI driver `main.cpp` symbol-by-symbol. This file maps the *library*.

Measured R526 (2026-08-04) against
`libslic3r/bambustudio/references/BambuStudio/src/libslic3r` (the pinned
submodule) and `crates/libslic3r-rs/src`.

## The rule

```
BambuStudio/src/libslic3r/<Dir>/<CamelCaseName>.{cpp,hpp}
        ->  crates/libslic3r-rs/src/<dir>/<snake_case_name>.rs
```

`CamelCase` → `snake_case`, directories lowercased. A C++ `.cpp`/`.hpp` **pair**
collapses into one `.rs` (Rust has no separate declaration file), so expect
roughly one Rust file per C++ translation unit, not per C++ file.

## Coverage

| | count |
|---|---|
| C++ translation units (distinct `.cpp`/`.hpp` stems) | 276 |
| Mirrored by a same-named Rust file | **~273 (99%)** |
| Deliberately absent | 3 |
| Rust `.rs` files | 326 |

The Rust file count is higher because several C++ headers that declare multiple
types are split, and because Rust adds `mod.rs` per directory.

## Directories — 1:1, all 13

| C++ | Rust |
|---|---|
| `Algorithm/` | `algorithm/` |
| `Arachne/` | `arachne/` |
| `CSGMesh/` | `csg_mesh/` |
| `Execution/` | `execution/` |
| `Fill/` | `fill/` |
| `Format/` | `format/` |
| `GCode/` | `gcode/` |
| `Geometry/` | `geometry/` |
| `Interlocking/` | `interlocking/` |
| `Optimize/` | `optimize/` |
| `SLA/` | `sla/` |
| `Shape/` | `shape/` |
| `Support/` | `support/` |

Rust adds two directories with no C++ counterpart, both intentional:

- `bin/` — the `slicer-cli` binary entry point (C++ keeps this in `main.cpp`).
- `debug/` — port-only instrumentation (e.g. `debug/compare.rs`, the stage
  dumps behind the `*_DEBUG`/`*DBG` env probes). No C++ equivalent by design.

## Deliberate divergences

Three C++ translation units have **no** Rust counterpart, each for a reason:

| C++ | Why absent |
|---|---|
| `GCodeSender.{cpp,hpp}` | Serial-port communication with a physical printer. Out of scope for an offline CLI slicer. |
| `clipper.{cpp,hpp}` | The vendored ClipperLib itself. Rather than port it, the Rust build **links the same C++ library** through `clipper_z_sys` (see `clipper_z.rs` / `clipper_z_utils.rs` / `clipper2_z*.rs`). Porting it would risk exactly the geometric divergence the binding avoids. |
| `Format/format.hpp` | Header aggregator only; its role is filled by `format/mod.rs`. |

Two more look absent under a naive CamelCase→snake_case transform but are
present under a different name — **check here before concluding a file is
unported**:

| C++ | Rust | Reason |
|---|---|---|
| `Format/3mf.{cpp,hpp}` | `format/three_mf.rs` (plus `format/bbs_3mf.rs`) | Rust identifiers cannot begin with a digit, so `3mf` becomes `three_mf`. A language constraint, not a design choice. |
| `Support/TreeSupport3D.{cpp,hpp}` | `support/tree_support_3d.rs` | Trailing digit grouping: `TreeSupport3D` → `tree_support_3d`, not `tree_support3_d`. |

## Finding things

1. **Know the C++ file?** Lowercase the directory, snake_case the stem, add
   `.rs`. `GCode/SeamPlacer.cpp` → `gcode/seam_placer.rs`.
2. **Digit in the name?** See the table above.
3. **Still missing?** Check the three deliberate absences, then
   `main-cpp-correspondence.md` for CLI-driver symbols.

Ported functions carry `C++: <File>.cpp:<line>` comments at their definition
(and often per-statement inside), so once you are in the right `.rs` file you
can navigate by C++ line number. Example, from `gcode/seam_placer.rs`:

```rust
/// SeamPlacer.cpp:962-1046
///
/// C++ runs under `tbb::parallel_for` over layer ranges, seeding each
/// range's `prev_layer_distancer` from `r.begin() - 1`; ...
```

## Regenerating this audit

`$D/layout2.py` in the session scratch dir walks both trees and reports
mirrored / not-mirrored counts. Re-run it after any C++ submodule re-sync; the
figures above are only valid for the currently pinned submodule revision.
