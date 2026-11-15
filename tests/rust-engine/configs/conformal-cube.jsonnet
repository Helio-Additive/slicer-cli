// The cube with `conformal_infill` on (R907): the job the ports of FillConformal,
// FillConformalChart and FillRadialZigZag were checked with against slicer_cli.
// Sparse infill is verbatim on all 120 layers. Variants tried the same way:
// conformal_stagger 'alternate', conformal_pole 'axis' and 'bezier',
// conformal_ray_count '12' with conformal_hub_radius '3'.
local bbl = 'references/BambuStudio/resources/profiles/BBL';

{
  input: {
    type: 'stl',
    model: { location: 'tests/rust-engine/fixtures/Cube_25.6.stl' },
    config: {
      profile_roots: [bbl],
      machine: import '../../../references/BambuStudio/resources/profiles/BBL/machine/Bambu Lab H2D 0.4 nozzle.json',
      filament: import '../../../references/BambuStudio/resources/profiles/BBL/filament/Bambu PLA Basic @BBL H2D.json',
      process: (import '../../../references/BambuStudio/resources/profiles/BBL/process/0.20mm Standard @BBL H2D.json') + {
        sparse_infill_pattern: 'zig-zag',
        conformal_infill: '1',
      },
    },
  },
  output: {
    gcode: { location: 'tests/rust-engine/.tmp/conformal-cube/cube.gcode' },
  },
}
