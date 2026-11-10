//! `slicer-parity`: run one job through `slicer_cli` and through the Rust
//! engine. A development tool; it is not packaged.

use std::path::PathBuf;
use std::process::ExitCode;

use slicer_cli_rust::parity::commands::{compare, slice, CompareArgs, Engine, SliceArgs};

const USAGE: &str = "\
slicer-parity: one job through slicer_cli (the C++ Bambu Studio engine) and the Rust engine.

Usage:
  slicer-parity compare --config JOB [--bambu-binary FILE] [-v]
  slicer-parity slice   --config JOB --engine bambu|rust [--bambu-binary FILE] [-v] [--dry-run]

  --config JOB          Job config: a JSON or Jsonnet file, or base64-encoded JSON
                        (Jsonnet needs the `jsonnet` program on PATH)
  --engine NAME         slice: which engine writes the job's output file
  --bambu-binary FILE   slicer_cli to run (default: $BAMBUSTUDIO_SLICER, a slicer_cli beside
                        this program, build-bambu/slicer_cli, then build/slicer_cli)
  -v, --verbose         Verbose engine log
  --dry-run             slice: print what would run

compare slices with both engines and prints how the two G-codes differ; it returns 0 when
they are byte-identical and 1 when they differ. COMPARE_KEEP_DIR=DIR keeps bambu.gcode and
rust.gcode in DIR for the scripts in scripts/parity/.
";

fn run() -> Result<u8, String> {
    let mut args = std::env::args().skip(1);
    let command = args.next().unwrap_or_default();
    if matches!(command.as_str(), "" | "-h" | "--help" | "help") {
        print!("{USAGE}");
        return Ok(0);
    }

    let mut config = None;
    let mut bambu_binary = std::env::var_os("BAMBUSTUDIO_SLICER").map(PathBuf::from);
    let mut engine = None;
    let mut verbose = false;
    let mut dry_run = false;
    while let Some(arg) = args.next() {
        let mut value = |name: &str| {
            args.next()
                .ok_or_else(|| format!("{name} needs a value after it."))
        };
        match arg.as_str() {
            "-c" | "--config" => config = Some(value("--config")?),
            "--bambu-binary" => bambu_binary = Some(PathBuf::from(value("--bambu-binary")?)),
            "--engine" => {
                engine = Some(match value("--engine")?.as_str() {
                    "bambu" => Engine::Bambu,
                    "rust" => Engine::Rust,
                    other => return Err(format!("--engine is bambu or rust, not {other}.")),
                })
            }
            "-v" | "--verbose" => verbose = true,
            "--dry-run" => dry_run = true,
            "-h" | "--help" => {
                print!("{USAGE}");
                return Ok(0);
            }
            other => return Err(format!("slicer-parity does not take {other}. See --help.")),
        }
    }
    let config = config.ok_or_else(|| "--config JOB is required.".to_owned())?;
    if verbose {
        let _ = env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
            .try_init();
    }

    match command.as_str() {
        "compare" => compare(CompareArgs {
            config,
            bambu_binary,
            verbose,
        }),
        "slice" => slice(SliceArgs {
            config,
            bambu_binary,
            verbose,
            dry_run,
            engine: engine.ok_or_else(|| "slice needs --engine bambu or --engine rust.".to_owned())?,
        }),
        other => Err(format!(
            "slicer-parity has the commands compare and slice, not {other}. See --help."
        )),
    }
}

fn main() -> ExitCode {
    match run() {
        Ok(code) => ExitCode::from(code),
        Err(message) => {
            eprintln!("{message}");
            ExitCode::from(2)
        }
    }
}
