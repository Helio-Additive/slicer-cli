use clap::Parser;
use std::process::ExitCode;

mod cli;
mod commands;
// main.cpp:55-137 — the structured stdout event protocol (R528).
// Emitters are ported and unit-tested; the emission SITES are not all wired
// yet (see docs/main-cpp-correspondence.md), hence dead_code here.
#[allow(dead_code)]
mod events;
mod config;
mod job_config;
mod json_utils;
mod locations;
mod bambu;
mod profiles;

use cli::{Cli, Commands};

fn main() -> ExitCode {
    // Surface the slicer library's `log::info!/warn!` output (progress markers,
    // painted-MMU diagnostics). Off by default; enable with RUST_LOG=info.
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    let r = run_cli();
    // R729 — UNIONPROF census (default OFF); prints once at exit.
    slicer::clipper_utils::unionprof_report();
    match r {
        Ok(code) => ExitCode::from(code),
        Err(err) => {
            eprintln!("error: {err}");
            ExitCode::from(1)
        }
    }
}

fn run_cli() -> Result<u8, String> {
    match Cli::parse().command {
        Commands::Slice(args) => commands::slice(args),
        Commands::Compare(args) => commands::compare(args),
        Commands::Presets(args) => commands::presets(args),
        Commands::Profiles(args) => commands::profiles(args),
    }
}
