{
  job_id: 'benchy-x1c-001',
  input: {
    kind: 'path',
    path: std.extVar('input_path'),
  },
  output: {
    kind: 'path',
    path: std.extVar('output_path'),
  },
  machine: 'Bambu Lab X1 Carbon 0.4 nozzle',
  filament: ['Bambu PLA Basic @BBL X1C'],
  process: '0.20mm Standard @BBL X1C',
}
