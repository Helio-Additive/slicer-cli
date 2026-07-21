use clap::Parser;
use std::process::ExitCode;

mod cli;
mod commands;
mod config;
mod job_config;
mod json_utils;
mod locations;
mod bambu;
mod profiles;

use cli::{Cli, Commands};

fn main() -> ExitCode {
    match run_cli() {
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
