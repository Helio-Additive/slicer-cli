// Second-model semantic-parity fixture (R353): a clean solid cube on the bed.
// Complements the Benchy job (stl-inline-config.jsonnet) so the semantic-
// equivalence regression test proves the Rust engine generalizes beyond Benchy.
// The cube scored SEMANTICALLY EQUIVALENT with silhouette IoU 100% (R351-R352).
local bbl = 'libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL';

{
  input: {
    type: 'stl',
    model: { location: 'fixtures/smoke/stl/Cube_25.6.stl' },
    config: {
      profile_roots: [bbl],
      machine: import '../../libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL/machine/Bambu Lab H2D 0.4 nozzle.json',
      filament: import '../../libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL/filament/Bambu PLA Basic @BBL H2D.json',
      process: import '../../libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL/process/0.20mm Standard @BBL H2D.json',
    },
  },
  output: {
    gcode: { location: 'tests/.tmp/stl-cube-config/cube.gcode' },
  },
}
