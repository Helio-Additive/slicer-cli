local bbl = 'libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL';

{
  input: {
    type: 'stl',
    model: '_downloads/3DBenchy.stl',
    config: {
      profile_roots: [bbl],
      machine: import '../../libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL/machine/Bambu Lab H2D 0.4 nozzle.json',
      filament: import '../../libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL/filament/Bambu PLA Basic @BBL H2D.json',
      process: import '../../libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL/process/0.20mm Standard @BBL H2D.json',
    },
  },
  output: {
    gcode: 'tests/.tmp/3mf-job/source.gcode',
    resolved_config: 'tests/.tmp/3mf-job/project_settings.config',
  },
}
