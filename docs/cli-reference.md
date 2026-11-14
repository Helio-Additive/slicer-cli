# Command-line reference

Flags that share a name with Bambu Studio's or OrcaSlicer's own command line
behave the same way.

A package holds three programs. `slicer_cli` slices with the Bambu Studio
engine, `slicer_cli-orcaslicer` with the OrcaSlicer engine and
`slicer_cli-snapmaker` with the Snapmaker Orca engine. The "Works in" column of
each table names the programs that take the flag:

- all binaries
- slicer_cli only (Bambu Studio engine)
- slicer_cli-orcaslicer and slicer_cli-snapmaker (the OrcaSlicer engines)
- slicer_cli and slicer_cli-orcaslicer

A flag that a program does not take is refused by name, with the program that
has it, before anything is loaded.

`--help` prints the same list for the program you run.

## Two ways to call it

The default call slices one plate and writes one G-code file:

```sh
slicer_cli model.3mf --plate 1 -o plate1.gcode
```

The official call, as the desktop apps' own command lines take it, slices the
plates you name and writes `plate_N.gcode` files and `result.json` into a
folder:

```sh
slicer_cli model.3mf --slice 0 --outputdir out            # every plate
slicer_cli model.3mf --slice 2 --outputdir out --export-3mf sliced.3mf
```

With `--slice 0` on a project of several plates, every plate is checked
(bed, settings) before any plate is sliced, as the official command lines do.
A plate that fails ends the run, and no plate's G-code is written.

Every flag below works with both calls unless its row says otherwise.

## Input files

Give one or more model files. A project 3MF (one saved by Bambu Studio,
OrcaSlicer or slicer-cli) comes first; other files are added to its plate.

| Kind | Extensions | Works in |
|-|-|-|
| Project or geometry 3MF | `.3mf` | all binaries |
| STL, OBJ, AMF | `.stl`, `.obj`, `.amf` | all binaries |
| STEP | `.step`, `.stp` | all binaries |
| glTF, FBX | `.glb`, `.gltf`, `.fbx` | slicer_cli only (Bambu Studio engine) |
| SVG | `.svg` | slicer_cli-orcaslicer and slicer_cli-snapmaker (the OrcaSlicer engines) |

A project 3MF brings its printer, process, filaments and plates. Any other
file needs a printer: name one with `--printer-preset`, or give settings files
with `--load-settings`, `--machine` or `--config`. A 3MF that holds only
geometry (no `Metadata/project_settings.config`) is treated like an STL. A
member of the archive with a rooted path (`C:/slicer-created/x`,
`//server/share/x`, `\server\share\x`, `/x`) is skipped: nothing is written for
it, inside or outside the run's own folder, a `ThreeMfMemberSkipped` event names
it, and the project loads as it does in the desktop app, whose loader skips
such a member too.

`--printer-preset` on a project 3MF moves the project to that printer, the way
the desktop app's printer list does: the printer's process is the one the
desktop selects for it (or the project's own, while the printer is one it
suits), and the plate is moved onto the new bed.

A settings file given to any of these flags is read the way the desktop reads
a preset: if it states `inherits`, the file's parents are resolved out of this
engine's own profiles tree and applied under the file's own values, so a file
the package ships — which holds only its differences from its parents — means
the same here as it does in the desktop app. A parent this engine does not
ship refuses the run before slicing, naming the preset to pass instead.

The desktop's own files load too: a preset saved from the settings panel
carries no `type` (the kind comes from the keys it states, or from the flag
that read it), and an "Export current configs" file carries an empty `from`.

Model files are placed the way the desktop apps place them when you add a
file: the first object at the centre of the bed, the next ones in the nearest
free spot, each lowered onto the bed. With `--slice`, several objects from
model files are then arranged, as the official command lines arrange model
files. With `--arrange 0` or `--arrange 1` a model file keeps its own
coordinates instead of this placement (and `1` then arranges it). A model that looks like it was saved in
meters or inches is reported (`model_warning` event `ModelUnitsLookWrong`,
and in `result.json`'s `error_reason` when the plate then fails);
`--convert-unit` converts it. An object larger than the bed is scaled down to
fit when it is far too large, as the desktop does, and reported.

### Assemble lists

`--load-assemble-list list.json --slice 0 --outputdir out` builds the plates
from a list instead of model files, as the official command lines do (no
model files with it; slicer_cli-orcaslicer also takes no transforms with it).
Every plate of the list is sliced, whatever plate `--slice` names. The list:

```json
{"plates": [
  {"plate_name": "left", "need_arrange": true,
   "plate_params": {"sparse_infill_density": "25%"},
   "objects": [
     {"path": "part.stl", "count": 2, "filaments": [1, 2],
      "assemble_index": [0], "pos_x": [60], "pos_y": [60], "pos_z": [0],
      "print_params": {"wall_loops": "3"},
      "height_ranges": [{"min_z": 0, "max_z": 5, "range_params": {"sparse_infill_density": "50%"}}]}],
   "assembled_params": [{"assemble_index": 1, "print_params": {"wall_loops": "4"}}]}]}
```

- `plate_params` are the plate's own settings; `print_params` and
  `height_ranges` an object's.
- A height range's `range_params` are the settings that band slices with. The
  band's layer height is `layer_height` in `range_params`; a band that states
  none keeps the object's own layer height (its `layer_height`, else the
  project's), and the run says so with a `config_normalized` event,
  `RangeLayerHeightFilled`. The same holds for a 3MF's own height ranges
  (`Metadata/layer_config_ranges.xml`): one that states no layer height keeps
  the object's, as the desktop's height-range editor always writes one.
- `count` copies of each part. Each per-copy list (`filaments`,
  `assemble_index`, `pos_x`, `pos_y`, `pos_z`) holds one value per copy or one
  value for all.
- Parts with the same `assemble_index` above 0 are merged into one object,
  `assemble_<n>`, whose settings come from `assembled_params`.
- `need_arrange` arranges the plate; otherwise the objects stay at their
  positions. A copy is moved by its own position from where the first copy is.
- STL parts, and OBJ parts (an OBJ's colours become filaments; on slicer_cli
  that needs `--load-filaments`). slicer_cli also reads `"subtype"` (`normal_part`,
  `modifier_part`, `negative_part`, `support_enforcer`, `support_blocker`)
  for a part that is merged into another.

A missing list or part is -3, a list that cannot be read or has wrong counts
-5, an unreadable part -6, a part of another kind -2, and a plate that does
not fit -21.

### Progress pipe

On Linux, `--pipe NAME` writes the official progress lines into the named
pipe NAME (make it with `mkfifo` and read it before the run starts; a pipe
nobody reads within 1 second is left unused). One JSON object per line:

```json
{"message":"Slicing begins","plate_count":2,"plate_index":1,"plate_percent":4,"total_percent":4}
```

A slicing warning comes as `"warning"` in place of `"message"`. As in the
official command lines, the pipe carries the latest step each time its writer
wakes, so a fast slice can skip steps; the run ends with `"All done, Success"`
at 100. slicer-cli writes no "Generate thumbnails"
line, as it makes no pictures.

## Plate type

The plate type (`curr_bed_type`) follows the desktop app:

1. a project 3MF keeps its own plate type, and a plate with its own type uses
   it;
2. a model file takes the type the desktop app picks for the printer:
   - Bambu Studio: the printer model's default plate (for example Textured
     PEI Plate on the A1 mini, X1 Carbon, P1S and H2D), else High Temp Plate;
   - OrcaSlicer: the printer preset's `default_bed_type`, else the type for
     its printer model (Cool Plate for the Bambu Lab X1, P1 and A1 models),
     else High Temp Plate;
3. `--curr-bed-type` wins over both, for every plate.

| Program | `--curr-bed-type` values |
|-|-|
| slicer_cli | Cool Plate, Engineering Plate, High Temp Plate, Textured PEI Plate, Supertack Plate |
| slicer_cli-orcaslicer | Cool Plate, Engineering Plate, High Temp Plate, Textured PEI Plate, Textured Cool Plate, Supertack Plate |

Any other value is refused with this list (return code -2).

## Flags

A switch (a flag whose value column is empty or says `=0` or `=1`) takes no
separate value, as on the official command lines (`DynamicConfig::read_cli`,
Config.cpp 1719-1726 at both pins): `--normative-check` or
`--normative-check=1` turns it on, `--normative-check=0` turns it off, and
`--normative-check 0` turns it on and reads `0` as a model file. Every other
flag takes its value as the next word or after `=`.

### Files and presets

| Flag | Value | What it does | Works in |
|-|-|-|-|
| `-o`, `--output` | file | G-code file of the default call (default `output.gcode`) | all binaries |
| `--plate` | N | Default call: the plate of a 3MF to slice (1-based) | all binaries |
| `--printer-preset` | name | A printer system preset by name, every parent applied; on a project 3MF, the printer the project moves to | all binaries |
| `--process-preset` | name | A process system preset by name (default: the printer's) | all binaries |
| `--filament-preset` | name | A filament system preset by name; repeat for more filaments | all binaries |
| `--filament-map` | "1[,2...]" | Printers with a left and a right extruder (H2D, H2D Pro, H2C, X2D): the one each filament prints with, one value per filament (1 = left, 2 = right); sets `filament_map_mode` to `Manual` (see "Every print setting") | slicer_cli and slicer_cli-orcaslicer |
| `--machine` | file | A printer settings file (JSON) | all binaries |
| `--process` | file | A process settings file (JSON) | all binaries |
| `--filament` | file | A filament settings file (JSON) | all binaries |
| `--config` | file | A settings bundle (JSON) | all binaries |
| `--load-settings` | "machine.json;process.json" | Printer and process settings files, merged over the project's | all binaries |
| `--load-filaments` | "f1.json;f2.json;..." | Filament settings files, one per filament | all binaries |
| `--load-defaultfila` | `=0` or `=1` | Use the first filament file for filaments without one | all binaries |
| `--uptodate` | | Update the project's settings to the current system presets | all binaries |
| `--uptodate-settings` | "machine.json;process.json" | The system printer and process files for `--uptodate` | all binaries |
| `--uptodate-filaments` | "f1.json;..." | The system filament files for `--uptodate` | all binaries |
| `--estimate-mode` | | After a printer change, fill filament presets and extruder state for an estimate | slicer_cli only (Bambu Studio engine) |
| `--load-filament-ids` | "1,2,3" | The filament of each loaded object | all binaries |
| `--clone-objects` | "1,3" | Copies of each loaded object | all binaries |
| `--load-custom-gcodes` | file | Plate custom G-code (tool changes, pauses) from JSON | all binaries |
| `--skip-modified-gcodes` | `=0` or `=1` | Keep the system presets' G-code where the project changed it | all binaries |
| `--allow-newer-file` | `=0` or `=1` | Slice a 3MF saved by a newer app version | all binaries |
| `--allow-substitution` | | With `--slice`: slice a 3MF whose values this engine has to substitute | all binaries |
| `--project-settings-as-saved` | | Slice a Bambu Studio project with the values saved in the file | all binaries |
| `--datadir` | folder | Accepted; no effect (slicer-cli keeps no settings between runs) | all binaries |

**A Bambu Studio project loads the way the desktop app loads it.** A project
file holds a full copy of its printer, process and filament settings from the
day it was saved, and the list of the settings the maker changed
(`different_settings_to_system`). The app keeps the changed ones and takes
every other value from the preset it has today (`load_config_model`,
`Plater.cpp` 12155 at v02.09.01.70, `load_external_preset` in `Preset.cpp`);
the presets saved inside the project and the file's own nozzle rack are kept
(`Plater.cpp` 11953-11958, 12160). All three programs do the same with the
presets the package ships, and say so (event `config_normalized` /
`ProjectRebasedOnSystemPresets`). So a project saved by an older version
slices with today's start G-code, as it does in the app. A project whose
printer or process preset the package does not ship keeps its saved values
(`ProjectPresetRebaseSkipped`).

`--project-settings-as-saved` turns this off: the run slices the values in
the file, which is what the official command lines do. `slicer_cli-orcaslicer`
and `slicer_cli-snapmaker` load only a Bambu Studio project this way; a
project their own app saved is sliced as saved.

The app can also download newer presets than the ones its installer ships,
and slices with those. slicer-cli has the presets of its pinned engine
version, so a setting Bambu changed after that version differs.

Settings priority, highest first: setting flags on the command line, then
`--load-settings` and `--load-filaments`, then the 3MF.

`SNAPMAKER_ORCA_ALLOW_NEWER_FILE` (in the `slicer_cli-snapmaker` binary only):
any value but empty or `0` turns on `--allow-newer-file`, as Snapmaker Orca's
own CLI does (`Snapmaker_Orca.cpp:1274-1279`). It prints
`Forcing CLI newer-file compatibility from SNAPMAKER_ORCA_ALLOW_NEWER_FILE` to
stderr.

A refusal that a flag gets past says so in its own sentence: a file newer than
the engine names `--allow-newer-file`, and a value this engine does not know
names the setting's own flag (`--<setting> <value>`) and `--allow-substitution`,
which lets the engine use its own default instead.

`--allow-mix-temp` in the `slicer_cli-snapmaker` binary: slice a plate whose
filaments mix high- and low-temperature materials. Snapmaker Orca's engine has
no `allow_mix_temp` setting, so this build takes the desktop's rule (a used
filament flagged `filament_is_high_temperature` next to one that is not) as a
refusal, code `-62`, and this flag lifts it. It is read as the other builds
read theirs: bare or `=1` lifts the refusal, `=0` keeps it.

A project moved to a printer with another bed (a machine file in
`--load-settings`) has its plate moved onto the new bed, as the official
command lines do: on a larger bed the plate keeps its place around the bed
centre; on a smaller one, or one with another exclusion area, the objects and
the prime tower are centred (event `arranged` / `MovedToNewBed`). The
OrcaSlicer engine decides this from the two beds alone, as its official command
line does. A plate chosen with `--slice N` that is printed by object, and
whose printer has other extruder clearances, is arranged again (`arranged` /
`ClearanceArrange`); `--slice 0` is never re-arranged. Works in the BambuStudio
build only after a printer change; in the OrcaSlicer build with or without one.

Which process settings such a change slices with differs between the two
binaries. The BambuStudio build keeps the project's own process and only points
its extruder variants at the new printer, as its official command line does
(the printer's default process comes from the `process_full` folder its package
ships). The OrcaSlicer build takes the process the desktop app takes when the
printer is picked: the preset of the current recipe first (the part of the
project's process name before its `@`), else the printer's
`default_print_profile`, else the best other preset the printer is compatible
with, searched in the new printer's own vendor bundle (the desktop tags each
preset with the bundle it came from and resolves a printer model through that
bundle's `machine_list`; with no bundle naming the preset, the project's is
searched), with the preset's settings resolved over their `inherits` chain, and
the slice is named for the preset taken. A printer the project's process already
says it suits keeps it. (The OrcaSlicer package ships no `process_full` folder,
and its own command line refuses every such change with `cannot find the
settings file .../BBL/process_full/<name>.json`, -3.)

### Every print setting

Every setting of the program's engine is a flag of the same name, with dashes:

```sh
slicer_cli part.stl --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
    --sparse-infill-density 15% --wall-loops 4 --layer-height 0.16 -o part.gcode
```

The values win over the presets' and the file's, for every plate. A value the
setting does not take is refused before slicing (return code -2), naming the
setting. `post_process` is refused: neither official command line runs
post-processing scripts. `--list-settings` marks every setting a binary
refuses as a flag (`refused_as_flag`).

Nozzles and filament grouping. Each binary takes the settings its official
command line reads from the flags, and what its desktop app's own screens
choose:

| Setting | slicer_cli | slicer_cli-orcaslicer | slicer_cli-snapmaker |
|-|-|-|-|
| `--filament-map-mode` | `Auto For Flush`, `Auto For Match`, `Auto For Quality`, `Manual`, `Nozzle Manual` | `Auto For Flush`, `Auto For Match`, `Manual` | refused: the engine keeps no map on a plate |
| `--filament-map` | the extruder of each filament (1 = left, 2 = right); on a printer with one extruder and a nozzle rack, Bambu Studio's grouping (1 = the nozzle the AMS feeds, 2 = the fixed nozzles) | the extruder of each filament, Bambu Lab printers with two extruders | refused |
| `--nozzle-volume-type` | the flow type of each extruder (`Standard`, `High Flow`, ...) | the same | the flow type of each extruder's nozzle (`standard`, `high_flow`) |
| `--filament-volume-map`, `--filament-nozzle-map` | the flow type and the rack nozzle of each filament | refused: the official command line does not read them | no such setting |
| `--extruder-nozzle-count`, `--extruder-nozzle-volume-type`, `--extruder-nozzle-diameter`, `--extruder-nozzle-slot`, `--ams-nozzle-idx` | the nozzle rack: how many nozzles each extruder holds, each one's flow type and diameter, and the nozzle the AMS feeds | refused: the official command line does not read them | no such setting |
| `--filament-volume-type` | no such setting | no such setting | with standard and high-flow nozzles mixed, the nozzle type of each filament |

The mode and the map from the command line win over the plate's and the
project's, as in the official command lines. `--filament-map` with no
`--filament-map-mode` means `Manual`, so the engine keeps that grouping; with
an automatic mode the engine groups the filaments itself, whatever the map
says. `-1` in the map takes the first value given. The run says what it
applied with a `config_normalized` event (`FilamentMapFromCommandLine`, or
`FilamentMapModeFromCommandLine` for a mode alone). `--export-settings` and
an exported 3MF state the mode with the map.

`--filament-map` on a printer with a left and a right extruder (the H2D,
H2D Pro, H2C and X2D in `slicer_cli`; the H2D, H2D Pro and X2D in
`slicer_cli-orcaslicer`): one value per filament of the project, in filament
order (`--filament-map 2,1`). Refused before slicing (return code -2): a
count other than the project's filament count, an extruder the printer does
not have, a one-nozzle printer, and in `slicer_cli-orcaslicer` any printer
that is not a Bambu Lab one with two extruders, where every filament has its
own extruder. On the H2C the right extruder changes between up to six
nozzles; with `Manual` the engine picks among them. A model with no saved
prime tower gets the tower inside the area both nozzles reach. In a manual
mode `slicer_cli` refuses a filament the plate uses that cannot print with
the extruder its map names (the filament's `filament_printable`; return code
-68, as the official command line), whether the map comes from the flag or
from the plate. An object placed where only one nozzle reaches is checked
before slicing, in `slicer_cli` and `slicer_cli-orcaslicer`: a manual map
that names the other extruder for its filament, or in an automatic mode a
filament no single extruder can print, ends the run with return code -66 and
a `plate_error` / `FilamentCannotMapToExtruder` event.

A project and its plates remember whether the maker's printer had a filament
switcher (`has_filament_switcher`) and the smart filament assignment that
needs it (`enable_filament_dynamic_map`). As in the official command line,
`slicer_cli` slices with both off unless a flag states them
(`--enable-filament-dynamic-map=1`, `--has-filament-switcher=1`).

`--filament-map` on a printer with one extruder and a nozzle rack (the H3C,
`slicer_cli`): 1 puts a filament on the nozzle the AMS feeds, 2 on the fixed
nozzles, where the engine picks the nozzle that fits. The settings must
state an AMS: a project saved with one does, and `--ams-nozzle-idx N` names
the rack nozzle it feeds (0 is the first; -1 is no AMS). Nothing is assumed:
with no flag the run slices with what the file says, as Bambu Studio does
with no printer connected (the official command line instead puts an AMS on
every such plate, `BambuStudio.cpp` 7448-7455). State the AMS the person has.
Without an AMS every nozzle is a fixed one, and a value of 1 is refused.

`--filament-map-mode "Nozzle Manual"` (`slicer_cli`, printers with a nozzle
rack) names the rack nozzle of each filament. As in the official command
line it needs all of `--filament-map`, `--filament-volume-map`,
`--filament-nozzle-map`, `--extruder-nozzle-count` and
`--extruder-nozzle-volume-type`, and is refused on a printer with no rack:

```sh
slicer_cli two-colours.3mf --printer-preset "Bambu Lab H2C 0.4 nozzle" --slice 1 --outputdir out \
    --filament-map-mode "Nozzle Manual" --filament-map 2,2 --filament-volume-map 0,0 \
    --filament-nozzle-map 1,3 --extruder-nozzle-count 1,3 \
    --extruder-nozzle-volume-type "Standard,Standard,Standard,Standard"
```

`--extruder-nozzle-count` with `--extruder-nozzle-volume-type` describes the
rack in any mode: the count per extruder, then one flow type per nozzle, in
order. `--extruder-nozzle-diameter` gives each nozzle's diameter (the
printer's own when left out) and `--ams-nozzle-idx` the nozzle the AMS feeds
(none when left out). `"Auto For Quality"` is Bambu Studio's Quality Mode:
it is taken on a printer whose extruders differ in type (the X2D) and
refused on any other. `--estimate-mode` slices with `Auto For Flush` and a
full rack of Standard nozzles; another mode with it is refused.

`slicer_cli-snapmaker`: `--nozzle-volume-type` sets each nozzle to
`standard` or `high_flow`, as the desktop's nozzle selector does. When the
nozzles are all one type every filament follows it. When they mix, the
desktop asks which nozzle type each filament uses: `--filament-volume-type`
is that answer, one value per filament; without it the file's values stand
and a `MixedNozzleFlowTypes` warning says so. As in the desktop's dialog, a
filament the plate uses that cannot print with a high-flow nozzle is refused
(return code -2) and one that is not recommended gets a
`HighFlowNotRecommended` warning; the lists are the engine's own
(`filament_allow_list.json`).

`result.json` reports what the slice used: `sliced_plates[].filament_maps`
(the extruder of each filament, the plate's `Metadata/slice_info.config`
`filament_maps`), `filament_nozzle_map` on a printer with a nozzle rack (the
rack nozzle of each filament, 0-based, as the G-code states it) and
`filament_map_mode`. Read the slice's choice there, not from the project's
`filament_map`: in an automatic mode `project_settings.config` keeps the map
it was saved with while the slice used another.

Mixed nozzle diameters (`slicer_cli`, Bambu Studio 2.9.1): `filament_diameter_map`
names the nozzle diameter each filament prints with, one entry per filament
(`--filament-diameter-map "0.4;0.2"`). Without the flag every loaded
filament takes the printer's diameter, as the official command line fills
it. The project's `multi_nozzle_diameter_mode` turns the checks on (or
`--multi-nozzle-diameter-mode=1`, which wins over the project's): the map
must then select two or more diameters, one entry per filament of the run
(the project's or the named presets', plus any slot `--load-filaments`
adds); each file `--load-filaments` loads is also checked, in the slot it
fills, against the diameters its preset supports (`compatible_printers`),
while the project's own filaments are checked for the entry count only.
The entry count is checked when the run merges settings (`--load-settings`,
`--load-filaments`, `--filament-colour`, `--uptodate`); a project or a
named-preset run sliced with only a map skips it (named presets enter the
check through `--filament-colour` or a loaded file), and the plate check
below still refuses a plate that ends up with one diameter.
Every plate must use two or more diameters; a
plate that uses one is refused (-108, `config_refused` /
`PlateSingleNozzleDiameter`). With the mode off, the project's map is set to
the printer's diameter and nothing is checked (a map given on the command
line still applies, as every setting flag does). `--downward-check` fails a printer that mounts fewer nozzles than
the diameters the project uses (`warning` / `PrinterNozzleCountTooSmall`).

These older shortcuts set one setting each and keep working:

| Flag | Value | Sets | Works in |
|-|-|-|-|
| `--infill` | percent | `sparse_infill_density` | all binaries |
| `--perimeters` | N | `wall_loops` | all binaries |
| `--nozzle` | mm | `nozzle_diameter` | all binaries |
| `--temp` | °C | `nozzle_temperature` | all binaries |
| `--bed-temp` | °C | `bed_temperature` | all binaries |

### Slicing and plates

| Flag | Value | What it does | Works in |
|-|-|-|-|
| `--slice` | 0 or N | Slice every plate (0) or plate N; writes `plate_N.gcode` and `result.json` | all binaries |
| `--outputdir` | folder | Where `--slice` writes (default: the current folder) | all binaries |
| `--skip-objects` | "3,5,10" | Leave out the objects with these ids: the id `--engine-info` lists for each placed object (the plate instance's `identify_id` in the 3MF; the loader gives the object that id). An id the file has on no plate is ignored, as the official command line ignores it, and a `warning` / `SkipObjectsUnknownIds` event names the file's ids; an id on another plate than the one sliced is left alone. A plain model file gives its objects no ids | all binaries |
| `--mtcpp` | count | Refuse a plate with more triangles (-59) | all binaries |
| `--mstpp` | time | Refuse a plate that takes longer to slice (-58). Bambu Studio counts seconds, OrcaSlicer milliseconds | all binaries |
| `--no-check` | | Skip some validity checks: a layer height over the printer's limit becomes a warning, and the G-code conflict check, the support-needed warning and the stop on empty-layer or overlap warnings are skipped. The bed refusals stay, as in the official CLI: an object over the bed's edge (-52) and a plate with nothing fully inside (-50) | all binaries |
| `--allow-mix-temp` | `=0` or `=1` | Allow filaments with very different temperatures on one plate. `slicer_cli` accepts it and does nothing with it: Bambu Studio 2.9.1 dropped the plate-level temperature-mix check, so there is nothing to lift; the `slicer_cli` run says so once with a `warning` / `AllowMixTempIgnored` event. `slicer_cli-snapmaker` applies it to its own pre-slice temperature check | all binaries |
| `--normative-check` | `=0` or `=1` | Refuse a project with post-processing scripts (-19). On by default with `--slice` | all binaries |
| `--enable-timelapse` | | Arrange as for a smooth timelapse (keeps room for the prime tower) | all binaries |
| `--load-slicedata` | folder | Slice each plate from the slicing data saved in `<folder>/<plate>`; a plate without usable data is sliced normally | all binaries |
| `--export-slicedata` | folder | Save each plate's slicing data in `<folder>/<plate>` | all binaries |
| `--downward-check` | | List the other printers the plates also fit on, in `result.json` | slicer_cli and slicer_cli-orcaslicer |
| `--downward-settings` | "m1.json;m2.json" | The printers (system machine files) for `--downward-check` | slicer_cli and slicer_cli-orcaslicer |
| `--progress` | | Progress events (on with `--slice`) | all binaries |
| `--load-assemble-list` | file.json | Build the plates from a JSON list of STL and OBJ parts, then slice every plate (needs `--slice`) | all binaries |
| `--pipe` | name | Linux only: write the official progress lines into the named pipe (a FIFO the caller made and reads) | all binaries |

`--load-slicedata` is refused with `--repetitions`; slicer_cli-orcaslicer also
refuses it together with `--export-slicedata`, as OrcaSlicer's command line
does. OrcaSlicer's loader cannot read the data its own exporter writes (see
[Known upstream behaviour](#known-upstream-behaviour)), so slicer_cli-orcaslicer
slices every plate normally and reports `SliceDataNotLoaded`. The official
command lines honour `--export-slicedata` and `--load-slicedata` only when
they come before `--slice` on the command line; slicer-cli takes them in any
order. `--downward-check` needs a project 3MF. Without `--downward-settings` it
uses the printer's list in `resources/profiles/BBL/cli_config.json`, whose
machine files (the `machine_full` folder) the open-source apps do not ship;
each missing file is reported and left out.

### Transforms

Transforms run in the order given, before the objects are placed.

| Flag | Value | What it does | Works in |
|-|-|-|-|
| `--scale` | factor | Scale every object | all binaries |
| `--rotate` | degrees | Rotate every object around Z | all binaries |
| `--rotate-x` | degrees | Rotate every object around X | all binaries |
| `--rotate-y` | degrees | Rotate every object around Y | all binaries |
| `--orient` | 0 or 1 | Turn each object to its best printing face | all binaries |
| `--assemble` | | Merge the model files into one object | all binaries |
| `--convert-unit` | | Convert a model saved in meters or inches to mm | all binaries |
| `--ensure-on-bed` | | Lift an object that is partly below the bed | all binaries |
| `--repetitions` | count | Print the plate's objects this many times in all (count - 1 copies), as many as fit; needs `--slice N` | all binaries |
| `--arrange` | 0, 1 or other | 0: keep the positions (a model file keeps its own coordinates); 1: arrange the plate (needs `--slice`); other or not given: with `--slice`, model files with more than one object are arranged | all binaries |
| `--allow-rotations` | | Let the arrange turn objects (off by default) | all binaries |
| `--allow-multicolor-oneplate` | | Let the arrange put several colours on one plate (on by default) | all binaries |
| `--avoid-extrusion-cali-region` | | Keep the arrange off the extrusion calibration area | all binaries |
| `--simplify` | `extra-high`, `high`, `medium`, `low` or `extra-low` | Reduce every part's triangles like the desktop Simplify tool, at that detail level (`medium` is the tool's default) | all binaries |
| `--simplify-ratio` | percent | Simplify: remove this share of each part's triangles, the tool's decimate ratio; above 0 and below 100 | all binaries |
| `--simplify-triangles` | count | Simplify: reduce each part to about this many triangles; a part already at or under it is left as it is | all binaries |
| `--simplify-objects` | "3,5" | Simplify only these objects, by the ids `--engine-info` lists (what `--skip-objects` takes); an id the file has on no plate is refused naming the file's ids (-2); default: every object. A plain model file has no ids | all binaries |
| `--simplify-keep-paint` | | Simplify: carry painted colours, supports, seams and fuzzy skin onto the reduced mesh, as the desktop's "preserve painting" / "keep painting" setting does. Off, the paint is cleared, as the desktop clears it | `slicer_cli`, `slicer_cli-orcaslicer` |

`--simplify`, `--simplify-ratio` and `--simplify-triangles` are the desktop
Simplify dialog's two controls, headless: the detail level bounds the error
each collapse may add (extra-high 0.001, high 0.01, medium 0.1, low 0.5,
extra-low 1.0), the ratio and the count bound how many triangles stay. Give
one of the three; the other simplify flags need one of them. The reduction
is the engine's own quadric edge collapse, run on each model part (modifiers
and negative parts are not touched) after the transforms above and before
the objects are placed, so the placement, the bed check, `--mtcpp`, the
exports and the slice all see the reduced meshes. Each part reports a
`mesh_simplified` event with its triangle count before and after; a part the
collapse leaves at its count is left untouched, paint included, and says so.
`--simplify-keep-paint` is refused by `slicer_cli-snapmaker`: its engine has
no way to carry paint onto a changed mesh, and its desktop clears the paint.
The refusal names the binaries that keep it: `slicer_cli-orcaslicer` for a
file an Orca printer can take, `slicer_cli` for a Bambu Studio project.

A single-part object that still has a million triangles or more after this
step gets the desktop's suggestion as a `warning` / `LargeMesh` event; the
run goes on.

### Exports and reports

| Flag | Value | What it does | Works in |
|-|-|-|-|
| `--export-3mf` | file | With `--slice`: the sliced project, written into `--outputdir`; a name that would overwrite one of the run's own input files (`../project.3mf` onto the input project itself, or a file a model's loader reads beside it, such as the `.mtl` an OBJ's `mtllib` names), or a file the run writes (`result.json`, `plate_N.gcode`, the `--export-settings` file, an `--export-stl`/`--export-stls` object STL, anything under `--export-slicedata`), is refused (-2), as is a name that differs from one of the run's own outputs only by letter case. An input is overwritten only when the name is that file, so `../PART.STL` beside an input `part.stl` writes where the file system tells the two apart (Linux). The same check runs, with or without `--export-3mf`, between any two files a run writes (`result.json`, the plates' G-code, `-o`, `--export-settings`, `--export-stl`/`--export-stls`, `--export-slicedata`) and between each of them and the run's inputs, before anything is sliced | all binaries |
| `--min-save` | `=0` or `=1` | Write the 3MF without the model geometry | all binaries |
| `--metadata-name` | "n1;n2" | Metadata names for the 3MF (pair with `--metadata-value`) | all binaries |
| `--metadata-value` | "v1;v2" | Metadata values for the 3MF | all binaries |
| `--makerlab-name` | name | MakerLab name in the 3MF | all binaries |
| `--makerlab-version` | version | MakerLab version in the 3MF | all binaries |
| `--skip-useless-pick` | `=0` or `=1` | Leave pick pictures out of the 3MF | slicer_cli only (Bambu Studio engine) |
| `--export-settings` | file | Write the project's settings as JSON (not a plate's own) and stop; with `--slice` the slice goes on | all binaries |
| `--export-stl` | | Write the objects as one STL and stop | all binaries |
| `--export-stls` | folder | Write each object as its own STL and stop | all binaries |
| `--info` | | Print each object's size, volume and facets, and stop | all binaries |
| `--engine-info` | file | What a file is (printer, plates, the app that made it), which of the package's programs fit it and which to use (`recommended`), as JSON. For a project 3MF, its placed objects (`objects`: the id `--skip-objects` and `--simplify-objects` take, the file's object id, instance, plate and name) | all binaries |
| `--gcode-timing` | `ORIGINAL OPTIMIZED` | Import two existing G-code files with this binary's native processor and print their normal-mode estimates as one JSON document. It never slices or rewrites either input, and it cannot be combined with any slicing, output, or probe flag. Orca and Snapmaker also require the printer vendor context described below | all binaries |
| `--list-presets` | | This engine's system presets as JSON | all binaries |
| `--list-settings` | | This engine's settings table as JSON: every key with its type and edit mode (`simple`/`advanced`/`develop`, plus `expert` on the OrcaSlicer build), and where the engine has them, its label, tooltip, category, default, bounds (`min`/`max`), unit, the preset that holds it (`scope`: `process`, `filament` or `printer`; left out for a setting in none of them) and, for enums, the values it takes. A field the engine has no value for is left out | all binaries |
| `--list-flags` | | This program's flags as JSON: every flag with what it does, what value it takes, whether this binary takes it and which binaries of the package do; the official flags with their keys, types and defaults. Print settings are not in it (`--list-settings`) | all binaries |
| `--printer` | name | With `--list-presets`: only this printer's presets, with its defaults and plate type | all binaries |

For Orca and Snapmaker, pass the original printer preset's BBL-vendor Boolean explicitly:

```sh
slicer_cli-orcaslicer --gcode-timing original.gcode optimized.gcode --printer-vendor-bbl false
slicer_cli-snapmaker --gcode-timing original.gcode optimized.gcode --printer-vendor-bbl false
```

Use `true` only when the original selected printer preset belongs to the `BBL`
vendor; otherwise use `false`. This is the same context the Snapmaker and Orca
desktops pass to their native G-code importers. It selects printer semantics
and the comment dialect, including pause and color-change timing boundaries; it is not inferred from the engine
name. Omitting it returns `PRINTER_CONTEXT_REQUIRED` with a nonzero exit code.
Bambu uses the two-path form without this option. Orca can further refine this
initial context from its embedded `printer_model`, as its desktop importer does,
and it does so for each file on its own: a pair whose optimized file lost its
embedded configuration is timed in two different printer contexts, and a
non-BBL context counts 260 s for each `G29` where a BBL context counts none.

The document names the engine the way `result.json` and `--engine-info` do
(`bambustudio`, `orcaslicer`, `snapmaker`) and carries the pinned engine
commit as `engine_revision`. A refusal is the same one-line JSON document with
an `error_code` instead of the two times. `CONFLICTING_ARGUMENTS` (any other
flag, or the pair not given first) and `PRINTER_CONTEXT_REQUIRED` exit 64.
`INPUT_UNREADABLE` (a missing, empty or non-regular input), `UNAVAILABLE`
(the environment) and `TIMING_UNAVAILABLE` (the importer gave no estimate)
exit 1. These codes are this mode's own; the slicing return codes below do
not apply to it.

### Output schemas

The JSON documents a caller reads have JSON Schemas (draft 2020-12) in
[`schemas/`](../schemas). Every release package carries the same files in its
own `schemas/` folder, so a caller checks against the build it runs:

| Document | Schema |
|-|-|
| `--engine-info` | `engine-info.schema.json` |
| `--gcode-timing ORIGINAL OPTIMIZED` | `gcode-timing.schema.json` |
| `result.json` in `--outputdir` | `result.schema.json` |
| `--list-presets` | `list-presets.schema.json` |
| `--list-settings` | `list-settings.schema.json` |
| `--list-flags` | `list-flags.schema.json` |
| An event line's JSON (`[[SLICER_EVENT]] {...}`) | `event.schema.json` |

A change to one of these documents changes its schema in the same commit. The
tests check every document they make against its schema, so the two cannot
drift apart. A field the schema does not list is a test failure, so a new field
is always a schema change that callers can see. The one exception is the event
lines: `event.schema.json` states the progress event in full and lists every
event kind, but the other events may carry fields it does not list, so a caller
reads only the fields it knows.

`--list-flags` is the same contract for the flags: a caller that drives the
binary (a worker, an agent) reads which flags this build takes, what each one
does and takes, and which binaries of the package take it, from the binary it
runs. A flag a release adds is a new entry, so a caller can check its own
flag list against the release it pins instead of reading this document.

```sh
# The flags this build refuses, and why.
slicer_cli-orcaslicer --list-flags | python3 -c '
import json, sys
for f in json.load(sys.stdin)["flags"]:
    if not f["supported"]:
        print(f["flag"], "|", f["refusal"])'
```

`--list-settings` is how an agent driving this binary finds a setting without
reading the engine's source: the keys, their meaning and their valid values are
all in the engine's own settings table, so it is printed rather than guessed.

```sh
# The setting behind "infill", and what it takes: key, label, type, min/max.
slicer_cli --list-settings | python3 -c '
import json, sys
for s in json.load(sys.stdin)["settings"]:
    if "infill" in s["key"]:
        print(s["key"], s["type"], s.get("min"), s.get("max"), s.get("default"), "|", s.get("label"))'
# sparse_infill_density percent 0 100 20% | Sparse infill density

# The values an enum takes, with their labels.
slicer_cli --list-settings | python3 -c '
import json, sys
for s in json.load(sys.stdin)["settings"]:
    if s["key"] == "sparse_infill_pattern":
        print([v["value"] for v in s["enum"]])'
```

slicer-cli makes no pictures: the exported 3MF has no plate thumbnails (the
`export_note` event `NoPlatePictures` says so) and the G-code has no
thumbnail block.

### Flags that are refused

| Flag | Works in | Why |
|-|-|-|
| `--export-png`, `--camera-view` | slicer_cli only (Bambu Studio engine) | slicer-cli makes no pictures |
| `--pipe` on macOS and Windows | all binaries | the official pipe exists on Linux only; progress goes to stdout as events on every system (`--progress`) |

Names that both official command lines only mention in comments (such as
`--cut`, `--export-obj`, `--center`, `--repair`) are refused as not being flags
of either.

### Other flags

| Flag | Value | What it does | Works in |
|-|-|-|-|
| `-h`, `--help` | | The flags of this program | all binaries |
| `-v`, `--verbose` | | The engine's log on stdout | all binaries |
| `--debug` | 0-5 | The engine's log level | all binaries |
| `--single-instance` | | Accepted and ignored | all binaries |
| `--calib-mode` | mode | A calibration print: `temp_tower`, `retraction_tower`, `pressure_advance_line`, `pressure_advance_tower` | all binaries |
| `--calib-mode` | `pressure_advance_pattern` | The pressure advance pattern calibration print (slicer_cli-orcaslicer refuses it: the OrcaSlicer engine's pattern calibration differs) | slicer_cli only (Bambu Studio engine) |
| `--calib-start`, `--calib-end`, `--calib-step` | number | The calibration range | all binaries |
| `--calib-extruder-id` | N | The extruder to calibrate (default 0) | all binaries |
| `--calib-no-numbers` | | No number labels (`pressure_advance_line`) | all binaries |
| `--input` | file | The input file (layout modes, or the model file) | all binaries |
| `--layout` | file | Arrange objects from an older JSON form, no slicing; refused with `--slice` | all binaries |
| `--layout-plan` | | Arrange objects from a versioned JSON request (stdin or `--input`), no slicing; refused with `--slice` | all binaries |
| `--no-normalize-legacy-gcode` | | Keep unbound legacy placeholders in custom G-code as they are | all binaries |

### `--layout-plan` request, schemaVersion 3

The request and answer types are in `layout_plan.hpp`. `layout capabilities --json`
reports the schema versions a binary takes. Version 3 adds, all binaries:

- `"beds": "all"`: every bed the arrange fills is a placement. A model's `bed_idx`
  is 0, 1, 2… (bed k becomes plate k+1), and its `x_mm`/`y_mm` are on that bed.
  Only a model that fits on no bed, or that is past 36 beds, has `bed_idx` -1;
  that answer exits 4 with `UNFITTABLE`, as before. The default, `"first"`, is
  the single-bed answer of versions 1 and 2. Every version 3 answer gives
  `beds_used`, the number of beds its placements use.
- `"primeTower": {"x_mm", "y_mm", "width_mm", "depth_mm", "brim_mm"}`: the tower
  rectangle in bed millimetres, where it prints, with `brim_mm` optional. It is
  a fixed item on every bed, as Bambu Studio's and OrcaSlicer's Arrange keep a
  tower on every plate. A tower that, with its brim, is not inside the
  printer's `printable_area` is refused.
- Kept clear on every bed: the machine profile's `bed_exclude_area` boxes, the
  wrapping-detection area when `enable_wrapping_detection` is on, and, on a
  printer that scans its first layer (`scan_first_layer`), the front
  calibration strip from (18, 0) to (240, 15) mm (OrcaSlicer: Bambu Lab
  printers only), as the desktop's Arrange keeps them. A locked model that
  stands on the tower or one of these areas is `UNFITTABLE`.

Give the printer's own rectangular `printable_area`. A bed with a notch cut out
for the tower makes the arrange use its irregular-bed path, which places objects
worse; the tower belongs in `primeTower`. A version 1 or 2 request that holds
`beds` or `primeTower` is refused, and its answers are unchanged.

## Outputs

### G-code

The default call writes the file named by `-o`. `--slice` writes
`plate_N.gcode` for each plate into `--outputdir`. With `--export-3mf` the
sliced project goes there too, holding the G-code of each sliced plate and a
`plate_N.json` with each object's position. It holds the model as it was
sliced: transforms, the arrange, `--repetitions` copies and skipped objects
(written as not printable). With `--slice N` it holds only plate N's objects,
as the official command line loads only that plate. After a printer change,
only plate N's prime tower position moves to the new bed; the other plates'
entries stay as given.

`--export-settings`, the exported project's settings and `result.json` all
state the project's settings, including the towers an arrange placed. Each
G-code header states its plate's own settings. On the Bambu build these can
also turn the prime tower off when every filament is the same, and fill the
per-filament lists to the filament count. So those keys may differ between the
header and the project.

### result.json

`--slice` always writes `result.json` into `--outputdir`, also when the run
fails. Its fields follow the official command lines' result file:

A run of `slicer_cli cube.stl --slice 1 --printer-preset "Bambu Lab A1 mini 0.4 nozzle" --outputdir out`
(the feature times are shortened here):

```json
{
    "engine": "bambustudio",
    "error_string": "Success.",
    "export_time": 0,
    "filament_presets": ["Bambu PLA Basic @BBL A1M"],
    "layer_height": 0.20000000298023224,
    "plate_index": 1,
    "prepare_time": 1,
    "printer_preset": "Bambu Lab A1 mini 0.4 nozzle",
    "process_preset": "0.20mm Standard @BBL A1M",
    "return_code": 0,
    "sliced_plates": [
        {
            "feature_type_times": {
                "Outer wall": 133.3519744873047,
                "Sparse infill": 253.5850372314453,
                "Travel": 185.76556396484375
            },
            "filament_change_times": 0,
            "filaments": [
                {
                    "filament_id": "GFA00",
                    "id": 1,
                    "main_used_g": 3.7003644055772265,
                    "total_used_g": 3.7003644055772265
                }
            ],
            "gcode_file": "out/plate_1.gcode",
            "id": 1,
            "main_predication": 746.7215576171875,
            "objects": [
                {
                    "bbox": {"depth": 20.0, "height": 20.0, "width": 20.0, "x": 80.0, "y": 80.0, "z": 0.0},
                    "id": 18,
                    "name": "cube.stl",
                    "triangle_count": 12
                }
            ],
            "sliced_time": 58,
            "sliced_time_with_cache": 0,
            "total_predication": 1133.287353515625,
            "triangle_count": 12,
            "unknown_settings": [],
            "warning_message": "",
            "warnings": []
        }
    ],
    "sparse_infill_density": 15.0,
    "unknown_settings": [],
    "wall_loops": 2
}
```

| Field | Meaning |
|-|-|
| `return_code` | 0, or the return code of the failure (see below) |
| `error_string` | The official sentence for the code, then what failed |
| `error_reason` | Only when a plate failed and more is known: the engine's own reason (which the official command lines print to stderr only), then what the load noted that can cause it (a model that looks saved in meters or inches, a `--machine`/`--process`/`--filament` file that could not be read) |
| `plate_index` | The `--slice` value |
| `prepare_time`, `export_time` | Milliseconds |
| `engine` | `bambustudio` or `orcaslicer` |
| `printer_preset`, `process_preset`, `filament_presets` | Optional catalog-validated names of the printer, process, and filament presets a model-file run with `--printer-preset` used. A process or filament not named is the printer's default, and every name is the preset's current one (an old name from its `renamed_from` is reported as the current name). They are captured before raw `--machine`/`--process`/`--filament` settings files can supply arbitrary ids, once the model reaches per-plate slicing; they are absent for 3MF projects, settings-file-only runs, project printer switches, and failures before that capture point. |
| `layer_height`, `wall_loops`, `sparse_infill_density` | The first sliced plate's values |
| `unknown_settings` | Settings in the file this engine does not have |
| `sliced_plates[]` | One entry per sliced plate |
| `sliced_plates[].id` | The plate number |
| `sliced_plates[].sliced_time` | Milliseconds |
| `sliced_plates[].triangle_count` | Triangles on the plate |
| `sliced_plates[].total_predication`, `main_predication` | Estimated print time, seconds |
| `sliced_plates[].filament_change_times` | Filament changes |
| `sliced_plates[].feature_type_times` | Seconds per feature (walls, infill, ...) |
| `sliced_plates[].objects[]` | `id`, `name`, `triangle_count`, `bbox` (`x`, `y`, `z`, `width`, `depth`, `height`). The box is in the project's scene, as the official result file states it: an object on plate 2 or later sits at that plate's place in the plate grid, not at the bed's origin |
| `sliced_plates[].filaments[]` | `id`, `filament_id`, `total_used_g`, `main_used_g`; on a printer with more than one nozzle also `extruder`, the nozzle the filament printed with (1 = left, 2 = right) |
| `sliced_plates[].filament_maps` | Printers with more than one nozzle (slicer_cli and slicer_cli-orcaslicer): the nozzle each of the project's filaments printed with, in filament order (1 = left, 2 = right), the plate's `slice_info.config` `filament_maps`. Set by `--filament-map`, else by the engine's own grouping |
| `sliced_plates[].filament_nozzle_map` | Printers with a nozzle rack (slicer_cli): the rack nozzle each of the project's filaments printed with, in filament order, 0-based, as the G-code's `filament_nozzle_map` states it |
| `sliced_plates[].filament_map_mode` | With `filament_maps` or `filament_nozzle_map`: the mode that chose it (`Manual` with `--filament-map`, `Auto For Flush` by default) |
| `sliced_plates[].warning_message` | The last slicing warning |
| `sliced_plates[].warnings` | Every slicing warning |
| `sliced_plates[].gcode_file` | The plate's G-code file |
| `downward_compatible_machine` | With `--downward-check`: the printers every plate fits |
| `upward_compatible_machine` | slicer_cli, after a settings merge: the newer printers the project suits |
| `upward_compatibility_taint` | slicer_cli with `--downward-check`: `PrintSequenceByObject` when a plate prints by object; `MultiNozzleDiameters` when the project's filaments select more than one nozzle diameter (the upward printers were then judged on their nozzle count) |

The preset-name fields and the last three compatibility fields are optional.

### Event lines

While it runs, slicer-cli writes one line per event on stdout:

```
[[SLICER_EVENT]] {"event":"<kind>","tag":"<what happened>","message":"<for people>", ...}
```

From the same run:

```
[[SLICER_EVENT]] {"curr_bed_type":"Textured PEI Plate","curr_bed_type_reason":"the printer model's default plate","defaults_replaced":[],"event":"presets_resolved","filaments":["Bambu PLA Basic @BBL A1M"],"message":"Settings built from BambuStudio system presets with every parent applied","printer":"Bambu Lab A1 mini 0.4 nozzle","process":"0.20mm Standard @BBL A1M","tag":"NamedPresetsResolved"}
[[SLICER_EVENT]] {"event":"model_loaded","message":"Loaded 1 object(s) from cube.stl","objects":1,"path":"cube.stl","tag":"ModelFileLoaded"}
[[SLICER_EVENT]] {"event":"progress","message":"All done, Success","percent":100,"plate_count":1,"plate_id":1,"plate_index":1,"plate_percent":100,"tag":"SliceProgress"}
```

The `event` kinds:

| Kind | When |
|-|-|
| `progress` | Progress of the run (`--progress`, on with `--slice`) |
| `presets_resolved`, `preset_resolution_failed`, `preset_error`, `preset_warning` | Named presets and system preset files: `ProjectPrinterSwitch` when `--printer-preset` moves a project 3MF, `ProcessSwitched` / `ProcessKept` when the new printer takes another process or keeps the project's |
| `model_loaded`, `model_warning`, `load_error`, `mesh_repaired` | Reading the model files |
| `mesh_simplified` | `--simplify`: `MeshSimplified` per reduced part (`triangles_before`, `triangles_after`), `SimplifySkipped` for a part left as it is: already at or under the asked count, or nothing to collapse at the asked detail level. A `warning` / `LargeMesh` is the desktop's suggestion for a single-part object of a million triangles or more |
| `config_normalized`, `config_substituted`, `config_unknown_keys`, `config_value_rejected`, `config_load_failed`, `config_refused`, `override_rejected` | Reading and merging settings |
| `engine_mismatch` | A file made for the other engine |
| `arranged` | Placement, arrange and `--repetitions` |
| `plate_error`, `input_error`, `output_error` | A plate or a file that cannot be used |
| `validation_error`, `validation_warning` | The engine's checks before slicing |
| `slicing_error`, `slice_warning`, `warning`, `timelapse_warning`, `filament_map_limited` | Slicing |
| `toolpath_conflict`, `toolpath_outside_bed`, `clearance_violation`, `filament_unprintable`, `gcode_check` | The checks on the finished G-code |
| `export`, `exported_3mf`, `export_note` | Files written by actions and exports |
| `engine_log` | Engine log lines (`--verbose`) |

The `tag` names the case within a kind; tools should read `event` and `tag`,
not the message.

### Return codes

The default call exits 0 when the G-code is written and 1 when it is not.
`--slice` exits with the return code also written to `result.json`
(on Linux and macOS the shell shows it as 256 plus the code, for example 254
for -2):

| Code | Meaning |
|-|-|
| 0 | Success |
| -1 | Failed setting up the environment |
| -2 | Invalid parameters (an unknown flag, a value a flag does not take) |
| -3 | Input file not found |
| -5 | Settings file cannot be parsed |
| -6 | Model file cannot be parsed |
| -8 | Unsupported instruction |
| -11 | STL export failed |
| -13 | 3MF export failed |
| -15, -16, -17 | The printer or process does not suit the project (settings merge) |
| -18 | Invalid values in the 3MF |
| -19 | The project names post-processing scripts |
| -21 | Arrange failed |
| -24 | The 3MF is from a newer app version |
| -25 | Unsupported feature in the 3MF (the official code; no slicer-cli check gives it since Bambu Studio 2.9.1 slices a mixed filament) |
| -50 | A plate is empty, or an object is larger than the bed |
| -51 | Invalid slicing settings |
| -52 | An object is partly outside the bed |
| -53, -54, -55 | Export or import of slicing data failed |
| -58 | Slicing took longer than `--mstpp` |
| -59 | More triangles than `--mtcpp` |
| -60 | Every object was skipped |
| -61 | Filaments do not suit the plate type |
| -62 | Filament temperatures too far apart |
| -63, -64 | Objects collide (by object, by layer) |
| -100 | Slicing failed |
| -101 | G-code path conflicts |
| -102 | G-code in an area a multi-extruder printer cannot reach |
| -103 | A filament cannot print on the first layer of this plate type |
| -104 | G-code outside the printable area |
| -105 | G-code in the wrapping detection area |
| -108 | Mixed nozzle diameters: a plate uses one nozzle diameter, or the map selects fewer than two (`slicer_cli`) |

`result.json` gives the sentence for each code.

## Known upstream behaviour

These OrcaSlicer behaviours come from the OrcaSlicer engine or the printer
profiles it ships at its pin, not from slicer-cli; slicer_cli-orcaslicer keeps
them as they are.

- **`--load-slicedata` returns -57 and the plate is sliced normally.**
  OrcaSlicer's `Print.cpp` writes each extrusion path's polyline as a
  `Polyline3` (`[[x, y, z]]`, line 3861; `ExtrusionEntity.hpp` 153) and reads it
  back as a 2D `Polyline` (lines 4116 and 4137), so the read stops with a JSON
  type error 305 and the load returns -57 (lines 4902-4904). The official
  OrcaSlicer alpha gives the same -57. slicer_cli-orcaslicer then slices the
  plate normally and reports `SliceDataNotLoaded`. Bambu Studio's paths hold a
  plain `Polyline` (`ExtrusionEntity.hpp` 215), so slicer_cli round-trips.
- **`--arrange 1` cannot place even a small part on some printers.** The
  official OrcaSlicer v2.4.0-alpha CLI, given the printer's own shipped
  profiles, fails the same way (its arrange leaves the part off the plate and the run ends with
  -50, "nothing to be sliced"; slicer_cli-orcaslicer stops at the arrange with
  -21). Without `--arrange 1` these printers slice. Three causes:
  - A skirt with a first-layer line width given as a percentage (Construct 1,
    iQ TiQ8; any printer with `skirt_loops` above 0 and, for example,
    `initial_layer_line_width = 120%`): `get_real_skirt_dist` reads the
    percentage as mm (`PrintConfig.cpp` 11314-11338), and `update_arrange_params`
    shrinks the bed by that much on each side (`Arrange.cpp` 87-92). Avoid it
    with `--initial-layer-line-width` in mm (for example 0.48) or `--skirt-loops 0`.
  - Anycubic Kobra 3 (all nozzles): its `bed_exclude_area` lists the whole bed
    square and then an inner square; the engine makes one exclusion box from
    every 4 points (`PartPlate.cpp` 418-435), so the first box covers the
    whole bed. Avoid it with `--bed-exclude-area 0x0`.
  - Folgertech i3 0.6: its `printable_area` is not a rectangle
    (0x0, 20x0, 200x200, 0x200). No flag avoids it; without `--arrange 1` the
    part is refused as over the bed edge (-52).
- **The prime volume of a Bambu project is kept as the file has it.** The
  official Bambu Studio command line sets `filament_prime_volume` to 45 for
  every filament of a Bambu project it slices without `--load-filaments`
  (`BambuStudio.cpp` 7422-7431 at v02.09.01.70); the desktop app slices with
  the file's value, and so does slicer-cli. For the official command line's
  G-code, pass `--filament-prime-volume` with 45 for each filament.
- **A deep prime tower can be refused -104 at the desktop's own tower spot.**
  On a plate with a fixed layout (`--load-assemble-list` with
  `need_arrange: false`) and two or more filaments, slicer_cli-orcaslicer puts
  the prime tower where the desktop puts it for a new plate
  (`set_default_wipe_tower_pos_for_plate`, `PartPlate.cpp` 4115-4190). That
  spot comes from an estimate of the tower's size. Some filament pairs make a
  deeper tower, for example Bambu PLA Basic with Bambu PETG HF on the X1 Carbon:
  the tower then runs past the back of the bed by a few mm. The engine's
  G-code check after slicing allows only 2 mm (`GCodeProcessor.cpp` 1861-1883),
  and the official OrcaSlicer CLI runs the same check, so the plate is refused
  -104 and the error names the prime tower. Move the tower with
  `--wipe-tower-x` and `--wipe-tower-y`, or turn it off with
  `--enable-prime-tower=0`.

In both engines on Windows, an STL or OBJ object is named from the text after
the last `\` of its path only (`DIR_SEPARATOR` in `Format/STL.cpp` lines 10-12
and 33-34, and in `Format/OBJ.cpp`, of both engines). A path written with `/`,
such as `parts/cube.stl`, names the object `parts/cube.stl`. `--export-stls`
then writes `<folder>/obj_1_parts/cube.stl` into a folder that does not exist,
and reports "Writing ... failed". Give the path with `\`, or run from the
model's folder.
