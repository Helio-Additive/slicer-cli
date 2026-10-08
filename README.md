# slicer-cli

Command-line slicers built from the Bambu Studio and OrcaSlicer slicing
engines. No user interface: give it a model or a project, get G-code.

A release package holds two programs:

| Program | Engine |
|-|-|
| `slicer_cli` | Bambu Studio 02.08.01.55 (source pin `5873b5f`) |
| `slicer_cli-orcaslicer` | OrcaSlicer 2.4.0-alpha (source pin `31f6803`) |

Each program slices with its engine's own code and its own printer, process
and filament profiles, so a file sliced here gives the G-code the desktop app
gives for the same settings. Flags that share a name with Bambu Studio's or
OrcaSlicer's own command line behave the same way.

## Install

Download the package for your system from the
[releases page](https://github.com/Helio-Additive/slicer-cli/releases) and
extract it:

| System | Package | Programs |
|-|-|-|
| Linux x86_64 (glibc 2.35 or later) | `slicer-cli-v<version>-linux-x86_64.tar.gz` | `bin/slicer_cli`, `bin/slicer_cli-orcaslicer` |
| macOS arm64 | `slicer-cli-v<version>-macos-arm64.tar.gz` | `slicer_cli`, `slicer_cli-orcaslicer` |
| Windows x86_64 | `slicer-cli-v<version>-windows-x86_64.zip` | `slicer_cli.exe`, `slicer_cli-orcaslicer.exe` |

Keep the programs inside the extracted folder: they read their profiles and
engine data from the `resources` folder beside them. The macOS and Windows
builds are not notarized or code-signed.

To build from source instead, see [docs/building.md](docs/building.md).

## Quick start

The examples use the Linux paths; on macOS and Windows run the programs from
the top of the extracted folder.

Slice one plate of a Bambu Studio or OrcaSlicer project. The project's own
printer, process and filaments are used:

```sh
./bin/slicer_cli model.3mf --plate 1 -o plate1.gcode
```

Slice a model file for a printer, by preset name (the names the desktop app
shows). The process and filament default to the printer's own:

```sh
./bin/slicer_cli part.stl --printer-preset "Bambu Lab A1 mini 0.4 nozzle" -o part.gcode
./bin/slicer_cli-orcaslicer part.stl --printer-preset "Snapmaker U1 (0.4 nozzle)" \
    --process-preset "0.20 Standard @Snapmaker U1 (0.4 nozzle)" -o part.gcode
```

Move a saved project to another printer the same way, by the name the desktop
app shows:

```sh
./bin/slicer_cli-orcaslicer project.3mf --printer-preset "Snapmaker U1 (0.4 nozzle)" \
    --slice 1 --outputdir out
```

Use the official command-line form, which writes `result.json` with the
outcome of each plate and exports a sliced project:

```sh
./bin/slicer_cli model.3mf --slice 0 --outputdir out --export-3mf sliced.3mf
```

Change any print setting with a flag of the same name:

```sh
./bin/slicer_cli part.stl --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
    --sparse-infill-density 15% --wall-loops 3 --curr-bed-type "Textured PEI Plate" -o part.gcode
```

List the presets of an engine, or the ones that suit a printer:

```sh
./bin/slicer_cli --list-presets
./bin/slicer_cli-orcaslicer --list-presets --printer "Prusa MK4 0.4 nozzle"
```

`--help` lists every flag of the program you run. Check the printer, nozzle and
material settings before you print.

## Documentation

- [Command-line reference](docs/cli-reference.md): every flag, the outputs,
  `result.json`, the event lines and the return codes.
- [Building from source](docs/building.md): both engines on Linux, macOS and
  Windows, the pinned versions and the engine override layer.
- [Contributing](CONTRIBUTING.md)
- [Security policy](SECURITY.md)

## License

AGPL-3.0-or-later. See `LICENSE`.

Both engines derive from PrusaSlicer and Slic3r under the AGPL. The
attribution chain and the third-party licenses are in `NOTICE` and in the
package's `THIRD_PARTY_LICENSES` folder.

If you run `slicer-cli` (or a modified version of it) on a server and let
network users interact with it, AGPL section 13 requires you to offer them the
source, for example with a "Source" link in the service.
