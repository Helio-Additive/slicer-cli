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
    /// Resolve preset files into one slicer config without slicing.
    Presets(PresetsArgs),
    /// Inspect bundled or external BambuStudio profile catalogs.
    Profiles(ProfilesArgs),
}

#[derive(Parser)]
pub struct SliceArgs {
    #[arg(short, long)]
    pub config: String,
    #[arg(long, env = "BAMBUSTUDIO_SLICER")]
    pub native_binary: Option<PathBuf>,
    #[arg(short, long)]
    pub verbose: bool,
    #[arg(long)]
    pub dry_run: bool,
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
