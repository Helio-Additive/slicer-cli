use std::{
    ffi::{OsStr, OsString},
    fs,
    path::{Path, PathBuf},
    process::{Command, Stdio},
};

pub fn run_native_slice(
    native_binary: &Path,
    input: &Path,
    config: Option<&Path>,
    output: &Path,
    verbose: bool,
    dry_run: bool,
) -> Result<u8, String> {
    let native_binary = native_binary.canonicalize().map_err(|e| {
        format!(
            "native slicer binary not found: {e}\n\
             Build it with: cmake -S libslic3r/bambustudio -B libslic3r/bambustudio/build && \
             cmake --build libslic3r/bambustudio/build"
        )
    })?;

    if let Some(parent) = output.parent() {
        fs::create_dir_all(parent).map_err(|e| format!("create {}: {e}", parent.display()))?;
    }

    let mut cmd = Command::new(&native_binary);
    if let Some(config) = config {
        cmd.arg("--config").arg(config);
    }
    cmd.arg("--output").arg(output);
    if verbose {
        cmd.arg("--verbose");
    }
    cmd.arg(input);

    if dry_run {
        println!("{}", render_command(&cmd));
        return Ok(0);
    }

    let status = cmd
        .stdin(Stdio::null())
        .status()
        .map_err(|e| format!("run native slicer: {e}"))?;

    Ok(status.code().unwrap_or(1) as u8)
}

pub fn default_native_binary() -> PathBuf {
    let exe = if cfg!(windows) {
        "slicer_cli.exe"
    } else {
        "slicer_cli"
    };

    if let Ok(current_exe) = std::env::current_exe() {
        if let Some(dir) = current_exe.parent() {
            let sibling = dir.join(exe);
            if sibling.is_file() {
                return sibling;
            }
        }
    }

    PathBuf::from("libslic3r")
        .join("bambustudio")
        .join("build")
        .join(exe)
}

fn render_command(command: &Command) -> String {
    let mut parts = Vec::new();
    parts.push(shell_quote(command.get_program()));
    parts.extend(command.get_args().map(shell_quote));
    parts.join(" ")
}

fn shell_quote(arg: &OsStr) -> String {
    let s = arg.to_string_lossy();
    if s.chars()
        .all(|c| c.is_ascii_alphanumeric() || matches!(c, '/' | '.' | '_' | '-' | ':'))
    {
        s.into_owned()
    } else {
        let mut quoted = OsString::from("'");
        quoted.push(s.replace('\'', "'\\''"));
        quoted.push("'");
        quoted.to_string_lossy().into_owned()
    }
}
