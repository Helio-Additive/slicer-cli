# Examples

Run the single Jsonnet job config and slice the included Benchy STL:

```sh
devbox run setup
devbox run native:build
devbox run example
```

`examples/config.jsonnet` imports and lightly constrains the same BambuStudio
profiles used by `example_benchy_h2d.sh`:

- `Bambu Lab H2D 0.4 nozzle.json`
- `Bambu PLA Basic @BBL H2D.json`
- `0.20mm Standard @BBL H2D.json`

The wrapper writes the flattened config to `examples/out/resolved-config.json`
and asks the native slicer to write `examples/out/3DBenchy_H2D_PLA.gcode`.

The job config accepts local paths or `s3://...` locations for model/config
inputs and G-code/resolved-config outputs. S3 transfer and optional callback
requests are handled by the Rust CLI itself.
