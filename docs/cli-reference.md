# Command-line reference

Flags that share a name with Bambu Studio's or OrcaSlicer's own command line
behave the same way.

A package holds two programs. `slicer_cli` slices with the Bambu Studio
engine, `slicer_cli-orcaslicer` with the OrcaSlicer engine. The "Works in"
column of each table names the program that takes the flag:

- both binaries
- slicer_cli only (Bambu Studio engine)
- slicer_cli-orcaslicer only

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
| Project or geometry 3MF | `.3mf` | both binaries |
| STL, OBJ, AMF | `.stl`, `.obj`, `.amf` | both binaries |
| STEP | `.step`, `.stp` | both binaries |
| glTF, FBX | `.glb`, `.gltf`, `.fbx` | slicer_cli only (Bambu Studio engine) |
| SVG | `.svg` | slicer_cli-orcaslicer only |

A project 3MF brings its printer, process, filaments and plates. Any other
file needs a printer: name one with `--printer-preset`, or give settings files
with `--load-settings`, `--machine` or `--config`. A 3MF that holds only
geometry (no `Metadata/project_settings.config`) is treated like an STL.

Model files are placed the way the desktop apps place them when you add a
file: the first object at the centre of the bed, the next ones in the nearest
free spot, each lowered onto the bed. With `--slice`, several objects from
model files are then arranged, as the official command lines arrange model
files. With `--arrange 0` or `--arrange 1` a model file keeps its own
coordinates instead of this placement (and `1` then arranges it). A model that looks like it was saved in
meters or inches is reported (`model_warning` event `ModelUnitsLookWrong`);
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
| `-o`, `--output` | file | G-code file of the default call (default `output.gcode`) | both binaries |
| `--plate` | N | Default call: the plate of a 3MF to slice (1-based) | both binaries |
| `--printer-preset` | name | A printer system preset by name, every parent applied | both binaries |
| `--process-preset` | name | A process system preset by name (default: the printer's) | both binaries |
| `--filament-preset` | name | A filament system preset by name; repeat for more filaments | both binaries |
| `--machine` | file | A printer settings file (JSON) | both binaries |
| `--process` | file | A process settings file (JSON) | both binaries |
| `--filament` | file | A filament settings file (JSON) | both binaries |
| `--config` | file | A settings bundle (JSON) | both binaries |
| `--load-settings` | "machine.json;process.json" | Printer and process settings files, merged over the project's | both binaries |
| `--load-filaments` | "f1.json;f2.json;..." | Filament settings files, one per filament | both binaries |
| `--load-defaultfila` | `=0` or `=1` | Use the first filament file for filaments without one | both binaries |
| `--uptodate` | | Update the project's settings to the current system presets | both binaries |
| `--uptodate-settings` | "machine.json;process.json" | The system printer and process files for `--uptodate` | both binaries |
| `--uptodate-filaments` | "f1.json;..." | The system filament files for `--uptodate` | both binaries |
| `--estimate-mode` | | After a printer change, fill filament presets and extruder state for an estimate | slicer_cli only (Bambu Studio engine) |
| `--load-filament-ids` | "1,2,3" | The filament of each loaded object | both binaries |
| `--clone-objects` | "1,3" | Copies of each loaded object | both binaries |
| `--load-custom-gcodes` | file | Plate custom G-code (tool changes, pauses) from JSON | both binaries |
| `--skip-modified-gcodes` | `=0` or `=1` | Keep the system presets' G-code where the project changed it | both binaries |
| `--allow-newer-file` | `=0` or `=1` | Slice a 3MF saved by a newer app version | both binaries |
| `--allow-substitution` | | With `--slice`: slice a 3MF whose values this engine has to substitute | both binaries |
| `--datadir` | folder | Accepted; no effect (slicer-cli keeps no settings between runs) | both binaries |

Settings priority, highest first: setting flags on the command line, then
`--load-settings` and `--load-filaments`, then the 3MF.

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

### Every print setting

Every setting of the program's engine is a flag of the same name, with dashes:

```sh
slicer_cli part.stl --printer-preset "Bambu Lab X1 Carbon 0.4 nozzle" \
    --sparse-infill-density 15% --wall-loops 4 --layer-height 0.16 -o part.gcode
```

The values win over the presets' and the file's, for every plate. A value the
setting does not take is refused before slicing (return code -2), naming the
setting. `post_process` is refused: neither official command line runs
post-processing scripts. The nozzle and filament mapping settings
(`nozzle_volume_type`, `filament_map_mode`, `filament_map`,
`filament_volume_map`, `filament_nozzle_map`, `extruder_nozzle_count`,
`extruder_nozzle_volume_type`) are refused as flags; the engine sets them
from the printer and the filaments.

These older shortcuts set one setting each and keep working:

| Flag | Value | Sets | Works in |
|-|-|-|-|
| `--infill` | percent | `sparse_infill_density` | both binaries |
| `--perimeters` | N | `wall_loops` | both binaries |
| `--nozzle` | mm | `nozzle_diameter` | both binaries |
| `--temp` | °C | `nozzle_temperature` | both binaries |
| `--bed-temp` | °C | `bed_temperature` | both binaries |

### Slicing and plates

| Flag | Value | What it does | Works in |
|-|-|-|-|
| `--slice` | 0 or N | Slice every plate (0) or plate N; writes `plate_N.gcode` and `result.json` | both binaries |
| `--outputdir` | folder | Where `--slice` writes (default: the current folder) | both binaries |
| `--skip-objects` | "3,5,10" | Leave out the objects with these ids (the 3MF's object ids) | both binaries |
| `--mtcpp` | count | Refuse a plate with more triangles (-59) | both binaries |
| `--mstpp` | time | Refuse a plate that takes longer to slice (-58). Bambu Studio counts seconds, OrcaSlicer milliseconds | both binaries |
| `--no-check` | | Skip some validity checks: a layer height over the printer's limit becomes a warning, and the G-code conflict check, the support-needed warning and the stop on empty-layer or overlap warnings are skipped. The bed refusals stay, as in the official CLI: an object over the bed's edge (-52) and a plate with nothing fully inside (-50) | both binaries |
| `--allow-mix-temp` | `=0` or `=1` | Allow filaments with very different temperatures on one plate | both binaries |
| `--normative-check` | `=0` or `=1` | Refuse a project with post-processing scripts (-19), or with a mixed filament (-25, Bambu Studio). On by default with `--slice` | both binaries |
| `--enable-timelapse` | | Arrange as for a smooth timelapse (keeps room for the prime tower) | both binaries |
| `--load-slicedata` | folder | Slice each plate from the slicing data saved in `<folder>/<plate>`; a plate without usable data is sliced normally | both binaries |
| `--export-slicedata` | folder | Save each plate's slicing data in `<folder>/<plate>` | both binaries |
| `--downward-check` | | List the other printers the plates also fit on, in `result.json` | both binaries |
| `--downward-settings` | "m1.json;m2.json" | The printers (system machine files) for `--downward-check` | both binaries |
| `--progress` | | Progress events (on with `--slice`) | both binaries |
| `--load-assemble-list` | file.json | Build the plates from a JSON list of STL and OBJ parts, then slice every plate (needs `--slice`) | both binaries |
| `--pipe` | name | Linux only: write the official progress lines into the named pipe (a FIFO the caller made and reads) | both binaries |

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
| `--scale` | factor | Scale every object | both binaries |
| `--rotate` | degrees | Rotate every object around Z | both binaries |
| `--rotate-x` | degrees | Rotate every object around X | both binaries |
| `--rotate-y` | degrees | Rotate every object around Y | both binaries |
| `--orient` | 0 or 1 | Turn each object to its best printing face | both binaries |
| `--assemble` | | Merge the model files into one object | both binaries |
| `--convert-unit` | | Convert a model saved in meters or inches to mm | both binaries |
| `--ensure-on-bed` | | Lift an object that is partly below the bed | both binaries |
| `--repetitions` | count | Print the plate's objects this many times in all (count - 1 copies), as many as fit; needs `--slice N` | both binaries |
| `--arrange` | 0, 1 or other | 0: keep the positions (a model file keeps its own coordinates); 1: arrange the plate (needs `--slice`); other or not given: with `--slice`, model files with more than one object are arranged | both binaries |
| `--allow-rotations` | | Let the arrange turn objects (off by default) | both binaries |
| `--allow-multicolor-oneplate` | | Let the arrange put several colours on one plate (on by default) | both binaries |
| `--avoid-extrusion-cali-region` | | Keep the arrange off the extrusion calibration area | both binaries |

### Exports and reports

| Flag | Value | What it does | Works in |
|-|-|-|-|
| `--export-3mf` | file | With `--slice`: the sliced project, written into `--outputdir` | both binaries |
| `--min-save` | `=0` or `=1` | Write the 3MF without the model geometry | both binaries |
| `--metadata-name` | "n1;n2" | Metadata names for the 3MF (pair with `--metadata-value`) | both binaries |
| `--metadata-value` | "v1;v2" | Metadata values for the 3MF | both binaries |
| `--makerlab-name` | name | MakerLab name in the 3MF | both binaries |
| `--makerlab-version` | version | MakerLab version in the 3MF | both binaries |
| `--skip-useless-pick` | `=0` or `=1` | Leave pick pictures out of the 3MF | slicer_cli only (Bambu Studio engine) |
| `--export-settings` | file | Write the project's settings as JSON (not a plate's own) and stop; with `--slice` the slice goes on | both binaries |
| `--export-stl` | | Write the objects as one STL and stop | both binaries |
| `--export-stls` | folder | Write each object as its own STL and stop | both binaries |
| `--info` | | Print each object's size, volume and facets, and stop | both binaries |
| `--engine-info` | file | What a file is (printer, plates, the app that made it) and which program fits it, as JSON | both binaries |
| `--list-presets` | | This engine's system presets as JSON | both binaries |
| `--printer` | name | With `--list-presets`: only this printer's presets, with its defaults and plate type | both binaries |

slicer-cli makes no pictures: the exported 3MF has no plate thumbnails (the
`export_note` event `NoPlatePictures` says so) and the G-code has no
thumbnail block.

### Flags that are refused

| Flag | Works in | Why |
|-|-|-|
| `--export-png`, `--camera-view` | slicer_cli only (Bambu Studio engine) | slicer-cli makes no pictures |
| `--pipe` on macOS and Windows | both binaries | the official pipe exists on Linux only; progress goes to stdout as events on every system (`--progress`) |

Names that both official command lines only mention in comments (such as
`--cut`, `--export-obj`, `--center`, `--repair`) are refused as not being flags
of either.

### Other flags

| Flag | Value | What it does | Works in |
|-|-|-|-|
| `-h`, `--help` | | The flags of this program | both binaries |
| `-v`, `--verbose` | | The engine's log on stdout | both binaries |
| `--debug` | 0-5 | The engine's log level | both binaries |
| `--single-instance` | | Accepted and ignored | both binaries |
| `--calib-mode` | mode | A calibration print: `temp_tower`, `retraction_tower`, `pressure_advance_line`, `pressure_advance_tower` | both binaries |
| `--calib-mode` | `pressure_advance_pattern` | The pressure advance pattern calibration print (slicer_cli-orcaslicer refuses it: the OrcaSlicer engine's pattern calibration differs) | slicer_cli only (Bambu Studio engine) |
| `--calib-start`, `--calib-end`, `--calib-step` | number | The calibration range | both binaries |
| `--calib-extruder-id` | N | The extruder to calibrate (default 0) | both binaries |
| `--calib-no-numbers` | | No number labels (`pressure_advance_line`) | both binaries |
| `--input` | file | The input file (layout modes, or the model file) | both binaries |
| `--layout` | file | Arrange objects from an older JSON form, no slicing | both binaries |
| `--layout-plan` | | Arrange objects from a versioned JSON request (stdin or `--input`), no slicing | both binaries |
| `--no-normalize-legacy-gcode` | | Keep unbound legacy placeholders in custom G-code as they are | both binaries |

## Outputs

### G-code

The default call writes the file named by `-o`. `--slice` writes
`plate_N.gcode` for each plate into `--outputdir`. With `--export-3mf` the
sliced project goes there too, holding the G-code of each sliced plate and a
`plate_N.json` with each object's position.

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
    "layer_height": 0.20000000298023224,
    "plate_index": 1,
    "prepare_time": 1,
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
| `plate_index` | The `--slice` value |
| `prepare_time`, `export_time` | Milliseconds |
| `engine` | `bambustudio` or `orcaslicer` |
| `layer_height`, `wall_loops`, `sparse_infill_density` | The first sliced plate's values |
| `unknown_settings` | Settings in the file this engine does not have |
| `sliced_plates[]` | One entry per sliced plate |
| `sliced_plates[].id` | The plate number |
| `sliced_plates[].sliced_time` | Milliseconds |
| `sliced_plates[].triangle_count` | Triangles on the plate |
| `sliced_plates[].total_predication`, `main_predication` | Estimated print time, seconds |
| `sliced_plates[].filament_change_times` | Filament changes |
| `sliced_plates[].feature_type_times` | Seconds per feature (walls, infill, ...) |
| `sliced_plates[].objects[]` | `id`, `name`, `triangle_count`, `bbox` (`x`, `y`, `z`, `width`, `depth`, `height`) |
| `sliced_plates[].filaments[]` | `id`, `filament_id`, `total_used_g`, `main_used_g` |
| `sliced_plates[].warning_message` | The last slicing warning |
| `sliced_plates[].warnings` | Every slicing warning |
| `sliced_plates[].gcode_file` | The plate's G-code file |
| `downward_compatible_machine` | With `--downward-check`: the printers every plate fits |
| `upward_compatible_machine` | slicer_cli, after a settings merge: the newer printers the project suits |
| `upward_compatibility_taint` | slicer_cli with `--downward-check`: `PrintSequenceByObject` when a plate prints by object |

The last three are written only when they have entries.

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
| `presets_resolved`, `preset_resolution_failed`, `preset_error`, `preset_warning` | Named presets and system preset files |
| `model_loaded`, `model_warning`, `load_error`, `mesh_repaired` | Reading the model files |
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
| -25 | Unsupported feature in the 3MF (mixed filament, slicer_cli) |
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

In both engines on Windows, an STL or OBJ object is named from the text after
the last `\` of its path only (`DIR_SEPARATOR` in `Format/STL.cpp` lines 10-12
and 33-34, and in `Format/OBJ.cpp`, of both engines). A path written with `/`,
such as `parts/cube.stl`, names the object `parts/cube.stl`. `--export-stls`
then writes `<folder>/obj_1_parts/cube.stl` into a folder that does not exist,
and reports "Writing ... failed". Give the path with `\`, or run from the
model's folder.
