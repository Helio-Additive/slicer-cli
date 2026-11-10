// Flow-vs-layer-height regression fixture (R451).
//
// WHY THIS EXISTS: every other single-material fixture in this repo slices at a
// 0.2mm layer height, and `fill/mod.rs` used to hardcode 0.2 as the infill flow
// height (Fill.cpp:255 actually says `(surface.thickness == -1) ? layer.height
// : surface.thickness`). A hardcoded 0.2 is INVISIBLE at 0.2mm — the bug only
// shows up when the layer height differs, and it under/over-extrudes EVERY
// infill feature by the layer-height ratio while leaving walls perfect.
//
// This config is deliberately built from the real BBL profile JSONs so it loads
// in BOTH engines — `--engine bambu` produces the C++ reference and
// `--engine rust` the port, from the identical config. The other Benchy fixture
// (stl-file-config.jsonnet) uses hand-written jsonnet whose numeric
// `layer_height: 0.24` the C++ loader REJECTS ("invalid json type for
// layer_height"), so it can never be used for a C++ cross-check.
//
// Usage:
//   slicer-parity slice --engine bambu --config tests/rust-engine/configs/benchy-016.jsonnet
//   cp tests/rust-engine/.tmp/benchy-016/out.gcode cpp.gcode
//   slicer-parity slice --engine rust  --config tests/rust-engine/configs/benchy-016.jsonnet
//   python3 scripts/parity/semantic_compare.py tests/rust-engine/.tmp/benchy-016/out.gcode cpp.gcode
// Expect per-feature E-per-mm ratios ~1.000 and object material ~1.001.
local bbl = 'references/BambuStudio/resources/profiles/BBL';
{
  input: {
    type: 'stl',
    model: { location: 'tests/rust-engine/fixtures/3DBenchy.stl' },
    config: {
      profile_roots: [bbl],
      machine: import '../../../references/BambuStudio/resources/profiles/BBL/machine/Bambu Lab H2D 0.4 nozzle.json',
      filament: import '../../../references/BambuStudio/resources/profiles/BBL/filament/Bambu PLA Basic @BBL H2D.json',
      // 0.16mm — chosen precisely because it is NOT the 0.2 that used to be hardcoded.
      process: import '../../../references/BambuStudio/resources/profiles/BBL/process/0.16mm Standard @BBL H2D.json',
    },
  },
  output: { gcode: { location: 'tests/rust-engine/.tmp/benchy-016/out.gcode' } },
}
