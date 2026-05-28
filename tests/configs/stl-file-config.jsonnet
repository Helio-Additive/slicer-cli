{
  input: {
    type: 'stl',
    model: '_downloads/3DBenchy.stl',
    config: {
      machine: { path: 'tests/configs/stl-file-machine.jsonnet' },
      filament: { file: 'tests/configs/stl-file-filament.jsonnet' },
      process: { location: 'tests/configs/stl-file-process.jsonnet' },
    },
  },
  output: {
    gcode: 'tests/.tmp/stl-file-config/out.gcode',
  },
}
