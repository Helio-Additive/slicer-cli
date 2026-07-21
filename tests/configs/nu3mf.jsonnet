// Prepared MakerWorld 3MF (28_MAJORASMASK, multicolour). It carries its own
// Metadata/project_settings.config + model_settings.config, so no profile
// triple is needed — the embedded config drives the slice.
//
//   slicer-cli slice --config tests/configs/nu3mf.jsonnet --engine bambu
//
// The Rust engine now also accepts 3MF input (reads the embedded
// project_settings.config): `--engine rust`. NOTE: the Rust 3MF path is
// Tier-1 — it merges all objects into a single mesh and slices with one
// material, so this multicolour model will slice but WON'T match the bambu
// multicolour output (per-object filaments / painted MMU segmentation are a
// later milestone). For a faithful single-colour Rust 3MF slice, use a
// single-material 3MF.
{
  input: {
    type: '3mf',
    model: 'tests/28_MAJORASMASK_FULLCOLOUR_Makerworld_1plate_multicol.3mf',
  },
  output: {
    gcode: 'tests/.tmp/nu3mf/majorasmask.gcode',
    resolved_config: 'tests/.tmp/nu3mf/project_settings.config',
  },
}
