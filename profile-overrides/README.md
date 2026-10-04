# Profile overrides

Files here replace the same path in the packaged profile trees. The package
steps in `.github/workflows/slicer-cli-ci.yml` copy them over the pinned
references (`orcaslicer/` → `resources/profiles-orca/`). Only upstream fixes
made after our pin go here, each one the upstream change and nothing else.

| File | Upstream change | Why |
|-|-|-|
| `orcaslicer/Snapmaker/machine/Snapmaker U1 (0.4 nozzle).json` | `default_print_profile` line from OrcaSlicer 6312caaf13 (#15039, 2026-08-04); the pin is 31f6803 (2026-07-31) | At the pin the printer inherits `"0.20mm Standard @Snapmaker"` (`fdm_toolchanger.json:129`), which no shipped process has, so a slice with only `--printer-preset` fell back to another process. |
