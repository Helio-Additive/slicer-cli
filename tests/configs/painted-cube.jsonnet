// Multicolour smoke fixture: 20mm cube, +X face painted extruder 1, -X face
// extruder 2 (see crates/libslic3r-rs/tests/data/painted_cube.3mf, committed).
// Carries an embedded 2-filament project_settings.config — no profile triple.
// Rust engine emits Tier-1 toolchanges (bare T commands, no wipe tower yet).
{
  input: {
    type: '3mf',
    model: 'crates/libslic3r-rs/tests/data/painted_cube.3mf',
  },
  output: {
    gcode: 'tests/.tmp/painted-cube/cube.gcode',
  },
}
