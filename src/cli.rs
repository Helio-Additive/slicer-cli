use clap::{Parser, Subcommand};
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
