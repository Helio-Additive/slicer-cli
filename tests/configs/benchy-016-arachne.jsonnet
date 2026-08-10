// R705 — Benchy, identical to benchy-016.jsonnet except wall_generator=arachne.
//
// The discriminator for a 25-round assumption. Benchy (classic walls, single
// material) matches C++'s outer wall at 93.6% content / 81.8% in-order; Majora
// (arachne + 8-colour multi-material) manages 20.6% / 9.0%. Two variables differ
// at once. This changes ONE of them, so the wall deficit can be attributed.
local bbl = 'libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL';
{
  input: {
    type: 'stl',
    model: { location: '_downloads/3DBenchy.stl' },
    config: {
      profile_roots: [bbl],
      machine: import '../../libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL/machine/Bambu Lab H2D 0.4 nozzle.json',
      filament: import '../../libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL/filament/Bambu PLA Basic @BBL H2D.json',
      process: (import '../../libslic3r/bambustudio/references/BambuStudio/resources/profiles/BBL/process/0.16mm Standard @BBL H2D.json')
               + { wall_generator: 'arachne' },
    },
  },
  output: { gcode: { location: 'tests/.tmp/benchy-016-arachne/out.gcode' } },
}
