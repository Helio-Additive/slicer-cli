# slicer-cli — supported-feature matrix

This document lists every upstream source file intentionally excluded from the
standalone `slicer-cli` build, with:
- the reason for exclusion
- the user-visible behaviour when the excluded path would have been triggered
- a regression test that pins that behaviour

The intent is to ensure excluded features **fail loudly** rather than
silently mis-processing. A CI test per row runs on every build.

See `libslic3r/bambustudio/CMakeLists.txt` for the CMake filter/exclusion
statements.

---

## Excluded source files

### `pchheader.cpp`
**Reason:** PCH source artefact (`pchheader.hpp` is used via `target_precompile_headers`; its `.cpp` companion is not needed in the standalone build and may conflict with CMake's own PCH mechanism).
**User-visible behaviour:** n/a — compile-time artefact only.
**Regression:** CI build succeeds without it (implicit).

---

### `ExPolygonCollection.cpp`
**Reason:** Not referenced by any source compiled into the headless build. The type exists in headers but the `.cpp` compiled separately triggered link-time redefinition errors in earlier versions.
**User-visible behaviour:** n/a.
**Regression:** Build succeeds; `nm slicer_cli | grep ExPolygonCollection` returns nothing (or minimal refs via inlines).

---

### `GCodeSender.cpp`
**Reason:** Sends gcode to a network printer (octoprint / Bambu network push). Not in scope for a CLI tool that writes to a file.
**User-visible behaviour:** No `--send-to-printer` flag exists. If a future flag is added that exercises this code path, the linker will surface the missing symbol immediately.
**Regression test:** `tests/test_excluded_features.sh` — asserts `--send-to-printer` produces an "Unknown argument" error.

---

### `PressureEqualizer.cpp`
**Reason:** BBS-internal pressure-equalizer feature not surfaced by the CLI's profile system.
**User-visible behaviour:** n/a at the CLI surface today (the feature is gated by a profile key; the key simply doesn't trigger the missing code path in normal FDM profiles).
**Regression:** Build succeeds; feature is silently absent.

---

### `OpenVDBUtils.cpp`
**Reason:** Depends on the OpenVDB library (SLA hollowing path). OpenVDB is not listed in `install_deps.sh` and is not a system package on most hosts.
**User-visible behaviour:** `.sl1` files are rejected by `main.cpp:1187` with "Unsupported file format. Use .stl or .3mf" before the code path is reached.
**Regression test:** `test_excluded_features.sh` — passes a fake `.sl1` file and asserts exit-code ≠ 0 plus the "Unsupported file format" message.

---

### `SLA/Hollowing.cpp`
**Reason:** SLA hollowing — FDM-only CLI has no use for it.
**User-visible behaviour:** No SLA hollowing flag. `.sl1` rejected at input-format check.
**Regression:** Same as `OpenVDBUtils.cpp` above — the input-format rejection fires first.

---

### `TryCatchSignalSEH.cpp`
**Reason:** Windows Structured Exception Handling variant. Excluded on non-Windows via the `*.mm` / SEH platform filter.
**User-visible behaviour:** n/a — compile-time platform exclusion.
**Regression:** CI passes on all three platforms.

---

### `LogSink.cpp`
**Reason:** Logging sink that emits to a UI widget (wxWidgets). Not present in headless builds.
**User-visible behaviour:** n/a.
**Regression:** Build succeeds.

---

### `NSVGUtils.cpp` (reference)
**Reason:** The reference `NSVGUtils.cpp` is excluded from the main source glob
and then re-added through the override-aware resolver. If an override exists at
`libslic3r/bambustudio/libslic3r/NSVGUtils.cpp`, that file is compiled;
otherwise the reference file is used.
**User-visible behaviour:** n/a.
**Regression:** Build succeeds; the override file is used.

---

### `PostProcessor.cpp`
**Reason:** Runs user-supplied shell scripts on the gcode output. Security-sensitive feature excluded from v1 scope; no sandboxing is in place.
**User-visible behaviour:** No `--post-process` flag. Adding one without re-enabling the source will produce a link error (intentional early warning).
**Regression test:** `test_excluded_features.sh` — asserts `--post-process` produces an "Unknown argument" error.

---

### `CutSurface.cpp`
**Reason:** Uses CGAL 5.x APIs (`add_property_map` returns `std::pair`, `AABB_traits` template form) that are incompatible with system CGAL 6.x. Needed only for mesh cutting operations, not basic STL→gcode FDM slicing.
**User-visible behaviour:** No mesh-cut flag. If the excluded code path were reached, the linker would surface undefined symbols immediately during development.
**Regression test:** `test_excluded_features.sh` — asserts binary launches and produces `--version` output (implies the CGAL 6.x build succeeded without `CutSurface.cpp`).

---

## Supported input formats

| Format | Supported | Notes |
|-|-|-|
| `.stl` / `.STL` | ✅ | Single-object binary or ASCII STL |
| `.3mf` / `.3MF` | ✅ | BambuStudio project 3MF (with `project_settings.config` + multi-plate support) |
| `.sl1` (SLA) | ❌ | Rejected with "Unsupported file format. Use .stl or .3mf" |
| Any other extension | ❌ | Rejected with "Unsupported file format. Use .stl or .3mf" |

---

## Regression test script

`tests/test_excluded_features.sh` — runs on every CI build as a post-build
check. Requires the built `slicer_cli` binary in the PATH or as `$1`.

```sh
#!/usr/bin/env bash
# See tests/test_excluded_features.sh
```

The script is the canonical source; this doc lists intent. When adding a new
excluded feature, add a row here AND a case in the test script.
