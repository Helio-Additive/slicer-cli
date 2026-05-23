use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use slicer_cli::ffi::{self, EventSink};
use slicer_cli::job::JobSpec;
use slicer_cli::preset_resolver;
use slicer_cli::profiles::ProfilesIndex;
use std::pin::Pin;

#[derive(Parser)]
#[command(
    name = "slicer-cli",
    about = "BambuStudio-based slicer CLI",
    version
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Slice a model using a JobSpec JSON file.
    Slice {
        /// Path to the JobSpec JSON file.
        job: std::path::PathBuf,
    },
    /// Look up bundled BambuStudio presets.
    Presets {
        #[command(subcommand)]
        action: PresetsAction,
    },
}

#[derive(Subcommand)]
enum PresetsAction {
    /// List all preset names for a category.
    List {
        /// Preset category.
        #[arg(value_enum)]
        kind: PresetKind,
    },
    /// Print a preset's JSON by name.
    Get {
        /// Preset category.
        #[arg(value_enum)]
        kind: PresetKind,
        /// Exact preset name (e.g. "Bambu Lab X1 Carbon 0.4 nozzle").
        name: String,
    },
}

#[derive(clap::ValueEnum, Clone)]
enum PresetKind {
    Machine,
    Filament,
    Process,
}

impl PresetKind {
    fn as_str(&self) -> &'static str {
        match self {
            PresetKind::Machine => "machine",
            PresetKind::Filament => "filament",
            PresetKind::Process => "process",
        }
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        Command::Slice { job } => handle_slice(&job).await,
        Command::Presets { action } => handle_presets(action),
    }
}

fn handle_presets(action: PresetsAction) -> Result<()> {
    match action {
        PresetsAction::List { kind } => {
            let json = ffi::ffi::slicer_list_presets(kind.as_str());
            println!("{json}");
        }
        PresetsAction::Get { kind, name } => {
            let json = ffi::ffi::slicer_get_preset(kind.as_str(), &name);
            if json == "null" {
                anyhow::bail!("{} preset not found: {:?}", kind.as_str(), name);
            }
            println!("{json}");
        }
    }
    Ok(())
}

fn find_profiles_dir() -> Option<std::path::PathBuf> {
    if let Ok(p) = std::env::var("SLICER_PROFILES_DIR") {
        let path = std::path::PathBuf::from(p);
        if path.is_dir() {
            return Some(path);
        }
    }
    let mut dir = std::env::current_exe().ok()?;
    for _ in 0..8 {
        dir.pop();
        let candidate = dir.join("references/BambuStudio/resources/profiles/BBL");
        if candidate.is_dir() {
            return Some(candidate);
        }
    }
    None
}

async fn handle_slice(job_path: &std::path::Path) -> Result<()> {
    let raw = std::fs::read_to_string(job_path)
        .with_context(|| format!("read {}", job_path.display()))?;
    let job: JobSpec = serde_json::from_str(&raw).context("invalid JobSpec")?;

    let profiles_dir = find_profiles_dir()
        .context("Cannot find BambuStudio profiles directory. Set SLICER_PROFILES_DIR or run from the repo root.")?;
    let index = ProfilesIndex::load(profiles_dir)
        .context("Failed to load profile index")?;

    let machine_json = preset_resolver::resolve_profile(&index, &job.machine)
        .map_err(|e| anyhow::anyhow!("machine profile '{}': {}", job.machine, e))?;

    let filament_jsons: Vec<serde_json::Value> = job
        .filament
        .iter()
        .map(|name| {
            preset_resolver::resolve_profile(&index, name)
                .map(serde_json::Value::Object)
                .map_err(|e| anyhow::anyhow!("filament profile '{}': {}", name, e))
        })
        .collect::<Result<_>>()?;

    let process_json = preset_resolver::resolve_profile(&index, &job.process)
        .map_err(|e| anyhow::anyhow!("process profile '{}': {}", job.process, e))?;

    let resolved_job = serde_json::json!({
        "job_id": job.job_id,
        "input": serde_json::to_value(&job.input)?,
        "output": serde_json::to_value(&job.output)?,
        "machine": machine_json,
        "filament": filament_jsons,
        "process": process_json,
    });

    let job_json = serde_json::to_string(&resolved_job)?;
    let mut sink = EventSink::new();
    let rc = ffi::ffi::slicer_run(&job_json, Pin::new(&mut sink));
    if rc != 0 {
        anyhow::bail!("slicer_run failed (rc={rc})");
    }

    for ev in &sink.events {
        println!("{ev}");
    }

    Ok(())
}
