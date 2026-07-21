local bbl = 'libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL';

{
  bambu_binary: 'libslic3r/bambustudio/build/slicer_cli',

  input: {
    type: 'stl',
    model: {
      location: 'examples/3DBenchy.stl',
    },
    config: {
      profile_roots: [bbl],
      machine: import '../libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL/machine/Bambu Lab H2D 0.4 nozzle.json',
      filament: import '../libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL/filament/Bambu PLA Basic @BBL H2D.json',
      process: import '../libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL/process/0.20mm Standard @BBL H2D.json',
    },
  },

  output: {
    gcode: {
      location: 'examples/out/3DBenchy_H2D_PLA.gcode',
    },
    resolved_config: {
      location: 'examples/out/resolved-config.json',
    },
    callback: null,
  },
}
