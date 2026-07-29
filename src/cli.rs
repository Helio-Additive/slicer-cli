//! CLI argument surface for `slicer-cli`.
//!
//! Corresponds to `libslic3r/bambustudio/main.cpp::print_usage` and the
//! `argc`/`argv` handling at the top of `main()`. See
//! `docs/main-cpp-correspondence.md` for the full `main.cpp` → Rust map.

use clap::{Parser, Subcommand, ValueEnum};
use std::path::PathBuf;

#[derive(Parser)]
#[command(name = "slicer-cli")]
#[command(about = "Rust wrapper for the BambuStudio libslic3r slicer binary")]
pub struct Cli {
    #[command(subcommand)]
    pub command: Commands,
}

#[derive(Subcommand)]
pub enum Commands {
    /// Slice from a JSON/Jsonnet job config path or base64-encoded JSON config.
    Slice(SliceArgs),
    /// Slice the same job with both engines and diff the resulting G-code.
    Compare(CompareArgs),
    /// Resolve preset files into one slicer config without slicing.
    Presets(PresetsArgs),
    /// Inspect bundled or external BambuStudio profile catalogs.
    Profiles(ProfilesArgs),
}

/// Which slicing engine to drive.
#[derive(Clone, Copy, Debug, Eq, PartialEq, ValueEnum)]
pub enum Engine {
    /// The C++ BambuStudio `slicer_cli` binary, run as a subprocess.
    /// (`native` is accepted as a deprecated alias.)
    #[value(alias = "native")]
    Bambu,
    /// The in-process Rust libslic3r port (`slicer` crate).
    Rust,
}

#[derive(Parser)]
pub struct SliceArgs {
    #[arg(short, long)]
    pub config: String,
    /// Path to the C++ BambuStudio slicer binary. (`--native-binary` is a
    /// deprecated alias.)
    #[arg(long, alias = "native-binary", env = "BAMBUSTUDIO_SLICER")]
    pub bambu_binary: Option<PathBuf>,
    #[arg(short, long)]
    pub verbose: bool,
    #[arg(long)]
    pub dry_run: bool,
    /// Slicing engine to use.
    #[arg(long, value_enum, default_value = "bambu")]
    pub engine: Engine,
}

#[derive(Parser)]
pub struct CompareArgs {
    #[arg(short, long)]
    pub config: String,
    /// Path to the C++ BambuStudio slicer binary. (`--native-binary` is a
    /// deprecated alias.)
    #[arg(long, alias = "native-binary", env = "BAMBUSTUDIO_SLICER")]
    pub bambu_binary: Option<PathBuf>,
    #[arg(short, long)]
    pub verbose: bool,
}

#[derive(Parser)]
pub struct PresetsArgs {
    #[arg(long)]
    pub machine: String,
    #[arg(long)]
    pub filament: String,
    #[arg(long)]
    pub process: String,
    #[arg(short, long)]
    pub output: PathBuf,
    #[arg(long)]
    pub profile_root: Vec<PathBuf>,
}

#[derive(Parser)]
pub struct ProfilesArgs {
    #[command(subcommand)]
    pub command: ProfilesCommand,
}

#[derive(Subcommand)]
pub enum ProfilesCommand {
    /// List profile files of one kind as JSON on stdout.
    List(ProfileListArgs),
    /// List process profiles compatible with a machine profile name, ID, path, or printer model.
    CompatibleProcesses(CompatibleProcessesArgs),
}

#[derive(Parser)]
pub struct ProfileListArgs {
    #[arg(long, value_enum)]
    pub kind: ProfileKind,
    #[arg(long)]
    pub profile_root: Vec<PathBuf>,
}

#[derive(Parser)]
pub struct CompatibleProcessesArgs {
    /// Machine profile name, setting ID, profile path, or printer_model value.
    #[arg(long)]
    pub printer: String,
    #[arg(long)]
    pub profile_root: Vec<PathBuf>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, ValueEnum)]
pub enum ProfileKind {
    Machine,
    Filament,
    Process,
}

impl ProfileKind {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Machine => "machine",
            Self::Filament => "filament",
            Self::Process => "process",
        }
    }
}
