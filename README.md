# slicer-cli

Standalone dual-engine command-line slicer derived from BambuStudio and
OrcaSlicer's `libslic3r` forks. AGPLv3.

`slicer-cli` is the AGPL boundary that closed apps subprocess. Run it directly,
embed it in CI, or wrap it from any tool that wants reproducible BambuStudio-
compatible STL → gcode slicing without dragging in a UI.

## License

AGPL-3.0-or-later. See `LICENSE`.

This project includes separate BambuStudio and OrcaSlicer engine binaries. Both
are derived from PrusaSlicer and Slic3r under AGPLv3. The full attribution chain
and engine-specific dependency provenance are in `NOTICE`.

If you run `slicer-cli` (or a modified version of it) on a server and let
network users interact with it, AGPL § 13 requires you to offer them the
source. The recommended pattern is a "Source" link in the UI of whatever
service wraps the binary. The Helio closed apps wrap `slicer-cli` via
subprocess invocation only — no AGPL linking — so their UIs are not bound by
§ 13; this README is for direct downstream consumers who are.

## What it does

- Reads STL or 3MF input.
- Resolves a printer / filament / process profile triple (BBL profile schema —
  same as BambuStudio).
- Generates G-code byte-identical to what BambuStudio Desktop would produce
  for the same inputs (within the supported-feature matrix below).

## What it does NOT do

This is `libslic3r`-headless. The following upstream features are intentionally
excluded; full list with rationale and user-visible behaviour in
`docs/slicer-cli-supported-features.md`:

- SLA hollowing (`OpenVDBUtils.cpp`, `SLA/Hollowing.cpp`)
- Mesh cutting (`CutSurface.cpp` — incompatible with system CGAL 6.x)
- User post-processor scripts (`PostProcessor.cpp` — security-sensitive,
  out of v1 scope)
- Network printer push (`GCodeSender.cpp`)
- Pressure equalizer (`PressureEqualizer.cpp`)

Any of these surfacing as a CLI flag returns a clear capability error rather
than silently mis-processing. A regression test per excluded feature pins the
behaviour.

## Build

Dependencies are system packages — see `install_deps.sh`.

```sh
git clone --recurse-submodules https://github.com/<org>/slicer-cli.git
cd slicer-cli
./install_deps.sh        # installs Homebrew / apt packages
mkdir -p cli/build && cd cli/build
cmake ..
cmake --build . -j
./slicer_cli --help
```

Submodule pin: `references/BambuStudio` is pinned at a known-good commit
(see `.gitmodules`). Bumping the pin is a deliberate maintenance action —
it can ripple through the override layer and the patched libigl tree. Test on
a feature branch first.

## Releases

Release builds are produced by GitHub Actions (`.github/workflows/`) for:

- Linux x86_64
- macOS arm64
- Windows x86_64

Linux packages target Ubuntu 22.04/glibc 2.35 and bundle non-glibc runtime
libraries. macOS packages bundle non-system dylibs and reject Homebrew or build
paths. CI artifacts are not currently notarized or Authenticode-signed.

The engine dependency contract, upstream-aligned pins, and documented platform
exceptions are recorded in `docs/engine-dependency-contract.md`.

Package metadata identifies the artefact as `slicer_cli` (not `BambuStudio`).
A CI assertion fails the build if the metadata regresses.

## Using a release package

Choose the engine by choosing its binary: `slicer_cli` uses BambuStudio;
`slicer_cli-orcaslicer` uses OrcaSlicer. Run the examples from the extracted
`slicer-cli` directory, using profiles from the matching engine's tree.
On Windows, use the corresponding `.exe` filename.

BambuStudio can read the settings embedded in a Bambu 3MF:

```sh
./slicer_cli model.3mf -o bambu.gcode
```

For OrcaSlicer, this example selects the packaged Snapmaker U1 profiles.
First supply `resolved-orca-config.json` containing their complete inherited
settings. The caller must resolve the profiles' `inherits` chains: the CLI
loads JSON overrides directly and does not resolve those chains itself.
Passing only the leaf files below would leave parent settings at defaults.

```sh
./slicer_cli-orcaslicer model.stl \
  --config resolved-orca-config.json \
  --machine 'resources/profiles-orca/Snapmaker/machine/Snapmaker U1 (0.4 nozzle).json' \
  --filament 'resources/profiles-orca/Snapmaker/filament/Snapmaker PLA @U1.json' \
  --process 'resources/profiles-orca/Snapmaker/process/0.20 Standard @Snapmaker U1 (0.4 nozzle).json' \
  -o orca.gcode
```

Use profiles matching your actual printer, nozzle, and material before printing.

## Versioning

`slicer-cli` uses semver. The major version may bump when the supported-
feature matrix narrows. The version reported in gcode headers is the pinned
BambuStudio submodule version + a `slicer-cli/<version>` suffix so you can
tell which lineage produced the output.

## Contributing

See `CONTRIBUTING.md`. External PRs require signing the CLA (preserves
dual-licence optionality) and a green CI build on all three platforms.

## Security

See `SECURITY.md` for the disclosure policy.

## Related projects

`slicer-cli` is one of two repos in the Helio source release:

- **`slicer-cli` (this repo, public AGPLv3)** — the engine.
- **`helio-platform` (private, commercial)** — the Tauri client, the Rust
  MCP server, the React UI, the rust-server cloud deploy, the Helio agent
  intelligence. Subprocess-invokes `slicer-cli`. No linking, no AGPL
  propagation.

This separation is intentional: AGPLv3 stays pinned to the engine where it
actually matters; closed-product code stays closed without cross-contaminating
licences.
