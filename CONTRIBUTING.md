# Contributing to slicer-cli

Thank you for contributing. Before opening a PR, please read the sections below.

## CLA

All contributions require signing the Helio Additive CLA. This preserves
dual-licence optionality and is required before any external PR is merged.
The CLA bot will comment on your first PR with instructions.

> **Why a CLA for an AGPL project?** The AGPL covers distribution and network
> use. The CLA additionally grants Helio a non-exclusive licence to use
> contributions in closed commercial builds (the Helio Tauri client, cloud
> server) that invoke `slicer-cli` via subprocess. This separation is the legal
> basis for the open/closed boundary described in `README.md`.

## What to contribute

Good first contributions:
- **Bug fixes** in the supported-feature matrix (see `docs/slicer-cli-supported-features.md`)
- **Dependency updates** — when a system dep (Boost, Eigen, CGAL) releases a
  new major version, the compat shims in `libslic3r/bambustudio/CMakeLists.txt`
  may need updating
- **Platform CI fixes** — if a release target (Linux arm64, Windows, etc.)
  breaks, a PR that restores it is very welcome
- **Profile additions / corrections** for printers and filaments under
  `libslic3r/bambustudio/references/BambuStudio/resources/profiles/` — but note these go upstream to
  BambuStudio first; file there, then bump our submodule pin here

**Not accepted (for now)**:
- New upstream `libslic3r` features — this CLI tracks the BambuStudio submodule;
  features belong upstream
- New CLI flags that exercise excluded features (SLA, mesh-cut, post-processor) —
  bring the dependency situation up to scratch first, then open an issue
- Changes that cause the binary to fail the clean-host smoke test or the
  package-metadata CI assertion

## CI requirements

PRs must be green on all three platforms before merge:
- Linux x86_64
- macOS arm64
- Windows x86_64

The CI matrix is in `.github/workflows/slicer-cli-*.yml`.

## Style

The reference source (`libslic3r/bambustudio/references/BambuStudio/src/libslic3r/`) is **not
editable** — it tracks the BambuStudio upstream byte-for-byte. If you need to
change the behaviour of a reference file, add an override file at the same
relative path under `libslic3r/bambustudio/libslic3r/`. See the override
mechanism description in `libslic3r/bambustudio/CMakeLists.txt`.

C++17, same conventions as BambuStudio.
