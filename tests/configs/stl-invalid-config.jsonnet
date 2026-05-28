{
  input: {
    type: 'stl',
    model: '_downloads/3DBenchy.stl',
    config: {
      machine: { name: 'machine' },
      filament: { name: 'filament' },
    },
  },
  output: {
    gcode: 'tests/.tmp/stl-invalid-config/out.gcode',
  },
}
