# Public `slicer-cli` repo — extraction plan

This document names every file or directory destined for the public AGPLv3
`slicer-cli` repo, with one-line rationale per item. **In-tree prep only** —
nothing is moved out of the current monorepo until the user signs off and
explicitly authorises the extraction.

The companion artifacts (LICENSE, NOTICE, README, CONTRIBUTING, SECURITY,
CPack metadata fix, supported-feature matrix, CI workflows) live alongside
this doc and are listed in OVERNIGHT_HANDOFF.md.

## Source-tree layout (current monorepo state)

| Path | Role | Public repo? |
|-|-|-|
| `cli/` | CLI binary source: `main.cpp`, `CMakeLists.txt` (626 lines), `standalone_stubs.cpp`, `Dockerfile`, `install_deps.sh`, `Makefile`, `example_benchy_h2d.sh` | **Yes** — entire directory |
| `cli/build/` | Out-of-source build directory | **No** — gitignored |
| `cli/output/` | Test-output gcode artefacts | **No** — gitignored |
| `libslic3r/bambustudio/libslic3r/` | Override layer — files here shadow the BambuStudio reference 1-for-1. **Currently empty** in this monorepo (no files override) but the directory + its include-search position are load-bearing in `cli/CMakeLists.txt` | **Yes** — directory is preserved (empty is fine; the override mechanism is the contract) |
| `libslic3r/bambustudio/libigl/` | Patched copy of `libigl` (4 files patched for Eigen 3.4 SparseMatrix migration). Full tree required because of relative `../../` includes anchoring inside the patched files. | **Yes** — entire patched tree, plus the README explaining why |
| `libslic3r/rs/` | WIP Rust port of the BambuStudio slicing core. Production slicing today is the C++ engine in `cli/`. | **Optional** — see "Rust FFI question" below |
| `references/BambuStudio` | Git submodule pinned at `b506005bc4ee62124e24bf00e0f58656db3646a6` (BambuStudio v02.06.00.51). All bundled sub-libraries (admesh, clipper, clipper2, miniz, glu-libtess, semver, libnest2d, mcut, boost-nowide, libigl-reference) live under `references/BambuStudio/src/` — no separate copies needed. | **Yes** — submodule kept as-is |
| `references/OrcaSlicer` | Git submodule, also AGPLv3 (Orca-derived). Not used by the CLI build today. | **No** — Orca support is post-v1; not part of this repo |
| `data/` | STL + reference gcode test fixtures | **Partial** — see "Test fixtures" below |
| `schemas/` | Slice config JSON schemas | **Yes** — referenced from CLI |
| `wasm/` | Arrange WASM module (built from `libnest2d`) | **No** — UI-side concern; stays closed |
| `ui/`, `slicer-mcp/`, `slicer-catalog/`, `server/` | Closed Helio runtime — Tauri shell, MCP server, web server, catalog crate | **No** — all closed |
| Top-level `admesh/`, `boost/`, `clipper/`, … | **Build artefacts** from a CMake out-of-source build at the repo root. Each contains `CMakeFiles/`, `Makefile`, the compiled `.a`. They are NOT source. | **No** — gitignored in the public repo |

## Bundled sub-libraries (all sourced from the BambuStudio submodule)

The CLI build pulls these directly from `references/BambuStudio/src/` via
`add_subdirectory`. No copy lives in the override layer except the patched
libigl. The public repo inherits all of them through the submodule — no
files need to be carved into the repo body.

| Sub-library | Location | Patched? |
|-|-|-|
| admesh | `references/BambuStudio/src/admesh/` | No |
| clipper | `references/BambuStudio/src/clipper/` | No |
| clipper2 | `references/BambuStudio/src/clipper2/` | No |
| miniz | `references/BambuStudio/src/miniz/` | No |
| glu-libtess | `references/BambuStudio/src/glu-libtess/` | No |
| semver | `references/BambuStudio/src/semver/` | No |
| libnest2d | `references/BambuStudio/src/libnest2d/` | No |
| mcut | `references/BambuStudio/src/mcut/` | No |
| boost (Boost.Nowide) | `references/BambuStudio/src/boost/` | No |
| libigl | `libslic3r/bambustudio/libigl/` (patched copy) AND `references/BambuStudio/src/libigl/` (fallback) | **4 files** patched in the override layer for Eigen 3.4 SparseMatrix removal. See `libslic3r/bambustudio/libigl/README.md`. |
| nanosvg | `references/BambuStudio/src/nanosvg/` | No |

## External dependencies (system packages, not vendored)

These come from `find_package` against system installs (Homebrew on macOS,
apt on Debian/Ubuntu, vcpkg on Windows). The public repo's `install_deps.sh`
documents the expected packages.

- TBB (Threading Building Blocks)
- Boost ≥ 1.73 (filesystem, thread, log, log_setup, regex, atomic, locale)
- Eigen3
- libpng, libz, libexpat, OpenSSL
- CGAL ≥ 5.x (CGAL 6.x supported via the `CutSurface.cpp` exclusion documented below)
- OpenCV (core only)
- OpenCASCADE (OCCT)
- Qhull
- Cereal (header-only)
- libnoise — required by `FuzzySkin.cpp` since BBS v02.05.03; source: `https://github.com/bambulab/libnoise` tag `v1.0.0`
- NLopt (optional; reduces arrangement quality if absent)
- Freetype (Linux/macOS), Fontconfig (Linux), ICU (Linux/macOS via Homebrew keg)

## Files excluded from the build

The CLI build is libslic3r-headless. These upstream sources are intentionally
excluded; each gets a fail-fast capability check + regression test in the
supported-feature matrix (see `docs/slicer-cli-supported-features.md`).

| Excluded file | Reason | User-visible behaviour when triggered |
|-|-|-|
| `pchheader.cpp` | PCH source — build artefact, not real source | n/a |
| `ExPolygonCollection.cpp` | Not needed in headless mode | n/a |
| `GCodeSender.cpp` | Sends gcode to a network printer; not part of CLI scope | CLI rejects `--send-to-printer` (or whatever flag, when added) with a clear error |
| `PressureEqualizer.cpp` | BBS-internal feature not used by the CLI surface | CLI rejects `--pressure-equalizer` flag with a clear error |
| `OpenVDBUtils.cpp` | Depends on OpenVDB; SLA hollowing feature, not FDM | CLI rejects SLA `.sl1` inputs with a clear error |
| `SLA/Hollowing.cpp` | SLA hollowing feature; not FDM | CLI rejects hollowing flags with a clear error |
| `TryCatchSignalSEH.cpp` | Windows SEH variant; non-Windows builds use the POSIX path | n/a (compile-time platform exclusion) |
| `LogSink.cpp` | Logging sink override (UI-coupled) | n/a |
| `NSVGUtils.cpp` | Replaced by an override version in the patched layer | n/a |
| `Print.cpp.ANNOTATED` | Annotated reference copy; not actual source | n/a |
| `PostProcessor.cpp` | User post-processor scripts — security-sensitive feature out of v1 scope | CLI rejects `--post-process` flag with a clear error |
| `CutSurface.cpp` | Uses CGAL 5.x APIs (`add_property_map` returns pair, AABB_traits) incompatible with system CGAL 6.x; needed only for mesh cutting, not basic STL→gcode slicing | CLI rejects mesh-cut operations with a clear error |
| `*.mm` (Objective-C++) | macOS-only sources are excluded on non-Apple platforms | n/a (compile-time platform exclusion) |

## Carve-out summary

What moves to the public `slicer-cli` repo:
- `cli/` (entire directory)
- `libslic3r/bambustudio/` (entire override layer — both `libslic3r/` and patched `libigl/`)
- `references/BambuStudio` (git submodule)
- `schemas/`
- The new top-level files: `LICENSE` (AGPLv3), `NOTICE`, `README.md`, `CONTRIBUTING.md`, `SECURITY.md`
- `.github/workflows/` (CI for cross-platform build/test/package/sign)
- `docs/slicer-cli-*.md` (this plan, supported-feature matrix, build notes)

What stays in the closed `helio-platform` monorepo:
- `ui/`, `slicer-mcp/`, `slicer-catalog/`, `server/`, `wasm/`
- `data/` (mostly — see Test fixtures below)
- All Helio agent code (`ui/src/agent/`, `helio-evals` workspace member, etc.)
- All telemetry / dogfood-only artifacts

What's gitignored in both:
- Build directories: `cli/build/`, `target/`, `.cache/`, `CMakeFiles/`, `CMakeCache.txt`, `cmake_install.cmake`, `Makefile`, `compile_commands.json`
- Top-level vendored sub-lib copies (`admesh/`, `boost/`, `clipper/`, `clipper2/`, `glu-libtess/`, `libigl/`, `libnest2d/`, `mcut/`, `miniz/`, `semver/`, `nanosvg_impl.cpp`, `standalone_compat.hpp`, `libslic3r_version.h`) — these are CMake build artefacts when at the repo root
- Test-output gcode (`cli/output/`)

## Test fixtures

Public repo carries a minimal corpus needed for CI smoke tests:
- `data/stl/27_Buzz_Multipart_3MF_Bambu.stl` (referenced in the plan)
- One reference gcode for byte-identical-output regression testing
- Profiles for one printer × one filament × one process (e.g. X1C + Bambu PLA Basic + 0.20mm Standard)

Larger corpora (filament library, multi-vendor profiles, regression suites)
stay in `helio-platform` and are loaded into the CLI on demand for internal CI.

## Rust FFI question

`libslic3r/rs/` is a WIP Rust port of the C++ slicing core. It's not on the
production slice path today (the C++ CLI in `cli/` is). Two options:

- **(A) Carve it out into the public repo** alongside the C++ engine. Pros:
  reuses the AGPL boundary; future Rust consumers don't need a separate repo.
  Cons: WIP code with 168 known cargo errors (per the project memory file)
  goes public; reviewers may misread it as supported.
- **(B) Keep it in the closed monorepo** until it stabilizes, then re-evaluate.
  Pros: avoids confusion; v1 ships the C++ CLI as the AGPL boundary, which
  is the boundary that actually matters. Cons: future consumers may want
  Rust bindings before we're ready to publish.

**Recommendation: (B) for v1.** The Rust port doesn't ship in any product
today; carving it now doesn't unblock anything and ships incomplete code as
public. Revisit when `cargo test -p slicer-rs` is green.

## Submodule pinning policy

The public repo pins `references/BambuStudio` at the commit currently
checked out in this monorepo (`b506005bc4ee62124e24bf00e0f58656db3646a6`,
v02.06.00.51). Updating the submodule is a deliberate maintenance action —
not a routine CI bump — because every BBS version change ripples through:

1. The override layer at `libslic3r/bambustudio/libslic3r/` may need new
   shadowed files if BBS introduces incompatible upstream changes.
2. The patched `libigl` files may need re-patching against newer libigl.
3. The bundled sub-libraries (admesh, clipper, etc.) come along; their
   own minor version bumps can break the build.

For v1 the pin stays. Bumps are tested on a feature branch before being
merged into the public `main`.

## What "extraction-ready" means

Once this plan is approved and the companion artifacts (LICENSE, NOTICE,
README, CONTRIBUTING, SECURITY, CPack fix, supported-feature matrix, CI
workflows) are landed in this monorepo, the actual extraction is a script:

```sh
# Pseudocode — not run yet
git subtree split --prefix=cli                       -b slicer-cli/cli
git subtree split --prefix=libslic3r/bambustudio     -b slicer-cli/override
# Compose into a fresh repo with submodule wired:
git init slicer-cli && cd slicer-cli
git pull /path/to/monorepo slicer-cli/cli
git pull /path/to/monorepo slicer-cli/override
git submodule add https://github.com/bambulab/BambuStudio.git references/BambuStudio
git -C references/BambuStudio checkout b506005bc4ee62124e24bf00e0f58656db3646a6
# Add the new top-level files (LICENSE, NOTICE, README, CONTRIBUTING, SECURITY, .github/workflows)
# CI smoke test must pass on a clean macOS arm64 host before publish.
```

The script lands in a follow-up session once the user explicitly OKs the
public-repo creation. **Do not run `gh repo create` from this session.**

## Open items (handed to the user, not auto-resolved)

| Item | Owner | Why blocking |
|-|-|-|
| CLA bot setup (`cla-assistant` or similar) | User | Needed before merging external PRs to preserve dual-license optionality |
| Apple Developer ID + Windows Authenticode certificates | User | Needed for code signing in Phase 1 CI; lead time |
| Final repo name (`slicer-cli` vs alternative) | User | Affects package metadata, CI workflow names, README copy |
| Whether to include `libslic3r/rs/` (Rust FFI) in the v1 public repo | User | Recommendation above is "no for v1"; user can override |
| Test-fixture corpus list (which STLs, gcodes, profiles ship with the public repo for CI) | User | Affects repo size + what regression tests can run upstream |
