use clap::{Parser, Subcommand};
use reqwest::Method;
use serde_json::{json, Map, Value};
use std::{
    collections::{BTreeSet, HashMap, HashSet},
    ffi::{OsStr, OsString},
    fs,
    path::{Path, PathBuf},
    process::{Command, ExitCode, Stdio},
};
use tempfile::{NamedTempFile, TempDir};
use walkdir::WalkDir;

const STRIP_KEYS: &[&str] = &[
    "name",
    "inherits",
    "include",
    "type",
    "from",
    "instantiation",
    "setting_id",
    "description",
    "label",
    "version",
    "compatible_filaments",
    "compatible_printers",
    "compatible_prints",
    "default_filament_profile",
    "default_print_profile",
    "printer_model",
    "printer_variant",
    "upward_compatible_machine",
];

#[derive(Parser)]
#[command(name = "slicer-cli")]
#[command(about = "Rust wrapper for the BambuStudio libslic3r slicer binary")]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Run a complete slicing job from a JSON or Jsonnet config.
    Run(RunArgs),
    /// Resolve profiles into one config and invoke the native slicer.
    Slice(SliceArgs),
    /// Resolve profiles into one config without slicing.
    Resolve(ResolveArgs),
}

#[derive(Parser)]
struct RunArgs {
    #[arg(short, long)]
    config: PathBuf,
    #[arg(long, env = "BAMBUSTUDIO_SLICER")]
    native_binary: Option<PathBuf>,
    #[arg(short, long)]
    verbose: bool,
    #[arg(long)]
    dry_run: bool,
}

#[derive(Parser)]
struct SliceArgs {
    #[arg(long)]
    input: PathBuf,
    #[arg(long)]
    machine: PathBuf,
    #[arg(long)]
    filament: PathBuf,
    #[arg(long)]
    process: PathBuf,
    #[arg(short, long)]
    output: PathBuf,
    #[arg(long, env = "BAMBUSTUDIO_SLICER")]
    native_binary: Option<PathBuf>,
    #[arg(long)]
    emit_config: Option<PathBuf>,
    #[arg(long)]
    profile_root: Vec<PathBuf>,
    #[arg(short, long)]
    verbose: bool,
    #[arg(long)]
    dry_run: bool,
}

#[derive(Parser)]
struct ResolveArgs {
    #[arg(long)]
    machine: PathBuf,
    #[arg(long)]
    filament: PathBuf,
    #[arg(long)]
    process: PathBuf,
    #[arg(short, long)]
    output: PathBuf,
    #[arg(long)]
    profile_root: Vec<PathBuf>,
}

struct PreparedOutput {
    requested: String,
    local_path: PathBuf,
    upload_uri: Option<String>,
}

enum ProfileSource {
    Path(PathBuf),
    Inline(Value),
}

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
        Commands::Run(args) => run_job(args),
        Commands::Slice(args) => slice(args),
        Commands::Resolve(args) => {
            let config = resolve_config(
                &args.machine,
                &args.filament,
                &args.process,
                &args.profile_root,
            )?;
            write_json(&args.output, &Value::Object(config))?;
            Ok(0)
        }
    }
}

fn run_job(args: RunArgs) -> Result<u8, String> {
    let job = load_profile(&args.config)?;
    let job_obj = expect_object(&job, "job config")?;
    let input_obj = object_field(job_obj, "input")?;
    let output_obj = object_field(job_obj, "output")?;

    let mut temp_dirs = Vec::new();
    let input_location = first_location(input_obj, &["model", "file", "location"], "input")?;
    let input_kind = input_kind(input_obj, &input_location)?;
    let input_path = materialize_input(&input_location, "input", &mut temp_dirs)?;
    let output_location = first_location(output_obj, &["gcode", "file", "location"], "output")?;
    let output = prepare_output(&output_location, "output.gcode", &mut temp_dirs)?;

    let native_binary = args
        .native_binary
        .or_else(|| optional_path(job_obj, "native_binary"))
        .unwrap_or_else(default_native_binary);

    let mut temp_config = None;
    let resolved_config_location =
        optional_location(output_obj, "resolved_config", "output.resolved_config")?;

    if input_kind == "stl" {
        let config_value = input_obj
            .get("config")
            .or_else(|| job_obj.get("config"))
            .ok_or_else(|| "STL input requires input.config".to_owned())?;
        let resolved = Value::Object(resolve_config_value(config_value, &mut temp_dirs)?);
        let mut file =
            NamedTempFile::new().map_err(|e| format!("create temp slicer config: {e}"))?;
        serde_json::to_writer_pretty(&mut file, &resolved)
            .map_err(|e| format!("write temp slicer config: {e}"))?;
        file.as_file_mut()
            .sync_all()
            .map_err(|e| format!("flush temp slicer config: {e}"))?;

        if let Some(location) = &resolved_config_location {
            write_json_to_location(location, &resolved)?;
        }
        temp_config = Some(file);
    } else if input_kind != "3mf" {
        return Err(format!("unsupported input type: {input_kind}"));
    }

    let code = run_native_slice(
        &native_binary,
        &input_path,
        temp_config.as_ref().map(NamedTempFile::path),
        &output.local_path,
        args.verbose,
        args.dry_run,
    )?;

    if code == 0 && output.upload_uri.is_some() && !args.dry_run {
        upload_output(&output)?;
    }

    if let Some(callback) = output_obj
        .get("callback")
        .or_else(|| job_obj.get("callback"))
    {
        let payload = json!({
            "status": if code == 0 { "succeeded" } else { "failed" },
            "exit_code": code,
            "input": {
                "type": input_kind,
                "location": input_location,
            },
            "output": {
                "location": output.requested,
            },
            "resolved_config": resolved_config_location,
        });
        send_callback(callback, &payload, args.dry_run)?;
    }

    Ok(code)
}

fn slice(args: SliceArgs) -> Result<u8, String> {
    let config = Value::Object(resolve_config(
        &args.machine,
        &args.filament,
        &args.process,
        &args.profile_root,
    )?);

    let mut temp_config = NamedTempFile::new().map_err(|e| format!("create temp config: {e}"))?;
    serde_json::to_writer_pretty(&mut temp_config, &config)
        .map_err(|e| format!("write temp config: {e}"))?;
    temp_config
        .as_file_mut()
        .sync_all()
        .map_err(|e| format!("flush temp config: {e}"))?;

    if let Some(path) = &args.emit_config {
        write_json(path, &config)?;
    }

    run_native_slice(
        &args.native_binary.unwrap_or_else(default_native_binary),
        &args.input,
        Some(temp_config.path()),
        &args.output,
        args.verbose,
        args.dry_run,
    )
}

fn run_native_slice(
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

fn resolve_config(
    machine: &Path,
    filament: &Path,
    process: &Path,
    extra_roots: &[PathBuf],
) -> Result<Map<String, Value>, String> {
    let profile_paths = [
        machine.to_path_buf(),
        filament.to_path_buf(),
        process.to_path_buf(),
    ];
    let index = ProfilesIndex::load_roots(&profile_paths, extra_roots)?;

    let mut merged = Map::new();
    for path in [machine, process, filament] {
        let profile = index.resolve_path(path)?;
        overlay(&mut merged, profile);
    }
    finalize_resolved_config(&mut merged);
    Ok(merged)
}

fn resolve_config_value(
    value: &Value,
    temp_dirs: &mut Vec<TempDir>,
) -> Result<Map<String, Value>, String> {
    let value = if let Some(location) = object_location(value)? {
        let path = materialize_input(&location, "input.config", temp_dirs)?;
        load_profile(&path)?
    } else {
        value.clone()
    };

    let obj = expect_object(&value, "input.config")?;
    let profile_obj = obj
        .get("profiles")
        .and_then(Value::as_object)
        .unwrap_or(obj);

    if !(profile_obj.contains_key("machine")
        && profile_obj.contains_key("filament")
        && profile_obj.contains_key("process"))
    {
        return Ok(obj.clone());
    }

    let roots = profile_roots(obj)?;
    let machine = profile_source(profile_obj.get("machine").unwrap(), "machine", temp_dirs)?;
    let filament = profile_source(profile_obj.get("filament").unwrap(), "filament", temp_dirs)?;
    let process = profile_source(profile_obj.get("process").unwrap(), "process", temp_dirs)?;

    let source_paths = [&machine, &filament, &process]
        .iter()
        .filter_map(|source| match source {
            ProfileSource::Path(path) => Some(path.clone()),
            ProfileSource::Inline(_) => None,
        })
        .collect::<Vec<_>>();

    let index = ProfilesIndex::load_roots(&source_paths, &roots)?;
    let mut merged = Map::new();
    for source in [&machine, &process, &filament] {
        let profile = match source {
            ProfileSource::Path(path) => index.resolve_path(path)?,
            ProfileSource::Inline(value) => index.resolve_value(value)?,
        };
        overlay(&mut merged, profile);
    }
    finalize_resolved_config(&mut merged);
    Ok(merged)
}

fn finalize_resolved_config(config: &mut Map<String, Value>) {
    normalize_single_filament_stl_config(config);
}

fn normalize_single_filament_stl_config(config: &mut Map<String, Value>) {
    let multi_nozzle_profile = value_array_len(config, "nozzle_diameter") > 1
        || value_array_len(config, "physical_extruder_map") > 1;
    if !multi_nozzle_profile || config.contains_key("filament_map") {
        return;
    }

    let nozzle = first_array_string(config, "nozzle_diameter").unwrap_or_else(|| "0.4".to_owned());
    let extruder_type =
        first_array_string(config, "extruder_type").unwrap_or_else(|| "Direct Drive".to_owned());
    let nozzle_volume_type = first_array_string(config, "default_nozzle_volume_type")
        .or_else(|| first_array_string(config, "nozzle_volume_type"))
        .unwrap_or_else(|| "Standard".to_owned());
    let variant = first_array_string(config, "filament_extruder_variant")
        .or_else(|| first_variant_from_list(config, "print_extruder_variant"))
        .unwrap_or_else(|| format!("{extruder_type} {nozzle_volume_type}"));
    let bed_type = config
        .get("default_bed_type")
        .and_then(Value::as_str)
        .unwrap_or("Textured PEI Plate")
        .to_owned();

    set_array(config, "nozzle_diameter", [nozzle]);
    set_array(config, "physical_extruder_map", ["0"]);
    set_array(config, "extruder_type", [extruder_type]);
    set_array(config, "extruder_variant_list", [variant.clone()]);
    set_array(config, "extruder_max_nozzle_count", ["1"]);
    set_array(config, "nozzle_volume_type", [nozzle_volume_type.clone()]);
    set_array(
        config,
        "extruder_nozzle_stats",
        [format!("{nozzle_volume_type}#1")],
    );

    set_array(config, "printer_extruder_id", ["1"]);
    set_array(config, "printer_extruder_variant", [variant.clone()]);
    set_array(config, "print_extruder_id", ["1"]);
    set_array(config, "print_extruder_variant", [variant.clone()]);

    set_array(config, "filament_map", ["1"]);
    set_array(config, "filament_map_2", ["0"]);
    set_array(config, "filament_nozzle_map", ["0"]);
    set_array(config, "filament_volume_map", ["0"]);
    set_array(config, "filament_printable", ["1"]);
    set_array(config, "filament_self_index", ["1"]);
    set_array(config, "filament_extruder_variant", [variant]);

    config.insert("curr_bed_type".to_owned(), Value::String(bed_type));
    config.insert(
        "enable_prime_tower".to_owned(),
        Value::String("0".to_owned()),
    );
    config.insert(
        "filament_map_mode".to_owned(),
        Value::String("Manual".to_owned()),
    );
}

fn value_array_len(config: &Map<String, Value>, key: &str) -> usize {
    config
        .get(key)
        .and_then(Value::as_array)
        .map(Vec::len)
        .unwrap_or(0)
}

fn first_array_string(config: &Map<String, Value>, key: &str) -> Option<String> {
    config
        .get(key)
        .and_then(Value::as_array)
        .and_then(|values| values.first())
        .and_then(Value::as_str)
        .map(str::to_owned)
}

fn first_variant_from_list(config: &Map<String, Value>, key: &str) -> Option<String> {
    config
        .get(key)
        .and_then(Value::as_array)
        .and_then(|values| {
            values
                .iter()
                .filter_map(Value::as_str)
                .find(|variant| variant.contains("Standard"))
                .or_else(|| values.iter().filter_map(Value::as_str).next())
        })
        .map(str::to_owned)
}

fn set_array<I, S>(config: &mut Map<String, Value>, key: &str, values: I)
where
    I: IntoIterator<Item = S>,
    S: Into<String>,
{
    config.insert(
        key.to_owned(),
        Value::Array(
            values
                .into_iter()
                .map(|value| Value::String(value.into()))
                .collect(),
        ),
    );
}

struct ProfilesIndex {
    name_to_path: HashMap<String, PathBuf>,
    path_cache: HashMap<PathBuf, Value>,
}

impl ProfilesIndex {
    fn load_roots(profile_paths: &[PathBuf], extra_roots: &[PathBuf]) -> Result<Self, String> {
        let mut roots = BTreeSet::new();
        for path in profile_paths {
            roots.insert(profile_scan_root(path));
        }
        for root in extra_roots {
            roots.insert(root.clone());
        }

        let mut path_cache = HashMap::new();
        let mut name_to_path = HashMap::new();

        for root in roots {
            for entry in WalkDir::new(&root)
                .follow_links(true)
                .into_iter()
                .filter_map(Result::ok)
                .filter(|entry| entry.file_type().is_file())
            {
                let path = entry.path();
                if !is_profile_file(path) {
                    continue;
                }
                let value = load_profile(path)?;
                if let Some(name) = value.get("name").and_then(Value::as_str) {
                    name_to_path.insert(name.to_owned(), path.to_path_buf());
                }
                path_cache.insert(path.to_path_buf(), value);
            }
        }

        Ok(Self {
            name_to_path,
            path_cache,
        })
    }

    fn resolve_path(&self, path: &Path) -> Result<Map<String, Value>, String> {
        let value = self.load_path(path)?;
        let name = value
            .get("name")
            .and_then(Value::as_str)
            .ok_or_else(|| format!("profile has no string name: {}", path.display()))?;
        self.resolve_name(name)
    }

    fn resolve_value(&self, value: &Value) -> Result<Map<String, Value>, String> {
        let obj = value
            .as_object()
            .ok_or_else(|| "inline profile must be a JSON object".to_owned())?;
        let mut merged = Map::new();
        let mut visited = HashSet::new();

        if let Some(parent) = obj
            .get("inherits")
            .and_then(Value::as_str)
            .filter(|s| !s.is_empty())
        {
            self.resolve_into(parent, &mut merged, &mut visited)?;
        }

        self.apply_level(value.clone(), &mut merged, &mut visited)?;
        Ok(merged)
    }

    fn resolve_name(&self, name: &str) -> Result<Map<String, Value>, String> {
        let mut merged = Map::new();
        let mut visited = HashSet::new();
        self.resolve_into(name, &mut merged, &mut visited)?;
        Ok(merged)
    }

    fn resolve_into(
        &self,
        name: &str,
        merged: &mut Map<String, Value>,
        visited: &mut HashSet<String>,
    ) -> Result<(), String> {
        if !visited.insert(name.to_owned()) {
            return Err(format!("circular profile reference at {name}"));
        }

        let chain = self.build_chain(name)?;
        for profile in chain.into_iter().rev() {
            self.apply_level(profile, merged, visited)?;
        }
        Ok(())
    }

    fn apply_level(
        &self,
        value: Value,
        merged: &mut Map<String, Value>,
        visited: &mut HashSet<String>,
    ) -> Result<(), String> {
        let Value::Object(obj) = value else {
            return Ok(());
        };

        for include_name in parse_include_field(obj.get("include")) {
            self.resolve_into(&include_name, merged, visited)?;
        }

        for (key, value) in obj {
            if !STRIP_KEYS.contains(&key.as_str()) {
                merged.insert(key, value);
            }
        }

        Ok(())
    }

    fn build_chain(&self, start: &str) -> Result<Vec<Value>, String> {
        let mut chain = Vec::new();
        let mut current = start.to_owned();
        let mut seen = HashSet::new();

        loop {
            if !seen.insert(current.clone()) {
                return Err(format!("circular inheritance at {current}"));
            }
            let value = self.load_name(&current)?;
            let parent = value
                .get("inherits")
                .and_then(Value::as_str)
                .filter(|s| !s.is_empty())
                .map(str::to_owned);

            chain.push(value);

            match parent {
                Some(parent) => current = parent,
                None => break,
            }
        }

        Ok(chain)
    }

    fn load_name(&self, name: &str) -> Result<Value, String> {
        let path = self
            .name_to_path
            .get(name)
            .ok_or_else(|| format!("profile not found: {name}"))?;
        self.load_path(path)
    }

    fn load_path(&self, path: &Path) -> Result<Value, String> {
        if let Some(value) = self.path_cache.get(path) {
            return Ok(value.clone());
        }
        load_profile(path)
    }
}

fn load_profile(path: &Path) -> Result<Value, String> {
    let ext = path
        .extension()
        .and_then(|s| s.to_str())
        .unwrap_or_default();
    let raw = if ext == "jsonnet" {
        let output = Command::new("jsonnet")
            .arg(path)
            .output()
            .map_err(|e| format!("run jsonnet for {}: {e}", path.display()))?;
        if !output.status.success() {
            return Err(format!(
                "jsonnet failed for {}:\n{}",
                path.display(),
                String::from_utf8_lossy(&output.stderr)
            ));
        }
        String::from_utf8(output.stdout)
            .map_err(|e| format!("jsonnet emitted non-UTF8 for {}: {e}", path.display()))?
    } else {
        fs::read_to_string(path).map_err(|e| format!("read {}: {e}", path.display()))?
    };

    serde_json::from_str(&raw).map_err(|e| format!("parse {}: {e}", path.display()))
}

fn parse_include_field(value: Option<&Value>) -> Vec<String> {
    match value {
        Some(Value::Array(arr)) => arr
            .iter()
            .filter_map(Value::as_str)
            .map(str::to_owned)
            .collect(),
        Some(Value::String(s)) => vec![s.clone()],
        _ => Vec::new(),
    }
}

fn profile_source(
    value: &Value,
    label: &str,
    temp_dirs: &mut Vec<TempDir>,
) -> Result<ProfileSource, String> {
    if let Some(location) = object_location(value)? {
        return materialize_input(&location, label, temp_dirs).map(ProfileSource::Path);
    }

    match value {
        Value::String(location) => {
            materialize_input(location, label, temp_dirs).map(ProfileSource::Path)
        }
        Value::Object(_) => Ok(ProfileSource::Inline(value.clone())),
        _ => Err(format!("{label} profile must be a path or object")),
    }
}

fn profile_roots(obj: &Map<String, Value>) -> Result<Vec<PathBuf>, String> {
    let mut roots = Vec::new();
    for key in ["profile_roots", "profile_root"] {
        let Some(value) = obj.get(key) else {
            continue;
        };
        match value {
            Value::Array(values) => {
                for item in values {
                    let location = string_value(item, key)?;
                    if is_s3_location(&location) {
                        return Err(format!("{key} does not support s3 roots: {location}"));
                    }
                    roots.push(PathBuf::from(location));
                }
            }
            _ => {
                let location = string_value(value, key)?;
                if is_s3_location(&location) {
                    return Err(format!("{key} does not support s3 roots: {location}"));
                }
                roots.push(PathBuf::from(location));
            }
        }
    }
    Ok(roots)
}

fn object_location(value: &Value) -> Result<Option<String>, String> {
    let Some(obj) = value.as_object() else {
        return Ok(None);
    };
    if obj.contains_key("name") || obj.contains_key("inherits") || obj.contains_key("type") {
        return Ok(None);
    }
    for key in ["location", "path", "uri"] {
        if let Some(value) = obj.get(key) {
            return string_value(value, key).map(Some);
        }
    }
    Ok(None)
}

fn materialize_input(
    location: &str,
    label: &str,
    temp_dirs: &mut Vec<TempDir>,
) -> Result<PathBuf, String> {
    if !is_s3_location(location) {
        return Ok(PathBuf::from(location));
    }

    let dir = TempDir::new().map_err(|e| format!("create temp dir for {label}: {e}"))?;
    let filename = s3_filename(location).unwrap_or(label);
    let path = dir.path().join(filename);
    download_s3(location, &path, label)?;
    temp_dirs.push(dir);
    Ok(path)
}

fn prepare_output(
    location: &str,
    default_filename: &str,
    temp_dirs: &mut Vec<TempDir>,
) -> Result<PreparedOutput, String> {
    if !is_s3_location(location) {
        return Ok(PreparedOutput {
            requested: location.to_owned(),
            local_path: PathBuf::from(location),
            upload_uri: None,
        });
    }

    let dir = TempDir::new().map_err(|e| format!("create temp dir for output: {e}"))?;
    let filename = s3_filename(location).unwrap_or(default_filename);
    let local_path = dir.path().join(filename);
    temp_dirs.push(dir);
    Ok(PreparedOutput {
        requested: location.to_owned(),
        local_path,
        upload_uri: Some(location.to_owned()),
    })
}

fn upload_output(output: &PreparedOutput) -> Result<(), String> {
    if let Some(uri) = &output.upload_uri {
        upload_s3(&output.local_path, uri, "output")?;
    }
    Ok(())
}

fn write_json_to_location(location: &str, value: &Value) -> Result<(), String> {
    if !is_s3_location(location) {
        return write_json(Path::new(location), value);
    }

    let file = NamedTempFile::new().map_err(|e| format!("create temp file for {location}: {e}"))?;
    write_json(file.path(), value)?;
    upload_s3(file.path(), location, "resolved_config")
}

fn download_s3(uri: &str, destination: &Path, label: &str) -> Result<(), String> {
    let (bucket, key) = parse_s3_uri(uri)?;
    let destination = destination.to_path_buf();
    aws_runtime()?.block_on(async move {
        let config = aws_config::load_defaults(aws_config::BehaviorVersion::latest()).await;
        let client = aws_sdk_s3::Client::new(&config);
        let object = client
            .get_object()
            .bucket(bucket)
            .key(key)
            .send()
            .await
            .map_err(|e| format!("download {label} from {uri}: {e}"))?;
        let bytes = object
            .body
            .collect()
            .await
            .map_err(|e| format!("read s3 response for {label}: {e}"))?
            .into_bytes();
        fs::write(&destination, bytes).map_err(|e| format!("write {}: {e}", destination.display()))
    })
}

fn upload_s3(source: &Path, uri: &str, label: &str) -> Result<(), String> {
    let (bucket, key) = parse_s3_uri(uri)?;
    let source = source.to_path_buf();
    aws_runtime()?.block_on(async move {
        let config = aws_config::load_defaults(aws_config::BehaviorVersion::latest()).await;
        let client = aws_sdk_s3::Client::new(&config);
        let body = aws_sdk_s3::primitives::ByteStream::from_path(&source)
            .await
            .map_err(|e| format!("read {} for s3 upload: {e}", source.display()))?;
        client
            .put_object()
            .bucket(bucket)
            .key(key)
            .body(body)
            .send()
            .await
            .map_err(|e| format!("upload {label} to {uri}: {e}"))?;
        Ok(())
    })
}

fn aws_runtime() -> Result<tokio::runtime::Runtime, String> {
    tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .map_err(|e| format!("create async runtime for s3: {e}"))
}

fn parse_s3_uri(uri: &str) -> Result<(String, String), String> {
    let path = uri
        .strip_prefix("s3://")
        .ok_or_else(|| format!("not an s3 URI: {uri}"))?;
    let (bucket, key) = path
        .split_once('/')
        .ok_or_else(|| format!("s3 URI requires bucket and key: {uri}"))?;
    if bucket.is_empty() || key.is_empty() {
        return Err(format!("s3 URI requires bucket and key: {uri}"));
    }
    Ok((bucket.to_owned(), key.to_owned()))
}

fn send_callback(callback: &Value, payload: &Value, dry_run: bool) -> Result<(), String> {
    if callback.is_null() || callback == &Value::Bool(false) {
        return Ok(());
    }

    let (url, method, headers) = match callback {
        Value::String(url) => (url.clone(), "POST".to_owned(), Vec::new()),
        Value::Object(obj) => {
            let url = string_field(obj, "url")?;
            let method = optional_string(obj, "method").unwrap_or_else(|| "POST".to_owned());
            let headers = obj
                .get("headers")
                .and_then(Value::as_object)
                .map(|headers| {
                    headers
                        .iter()
                        .filter_map(|(key, value)| {
                            value.as_str().map(|value| (key.clone(), value.to_owned()))
                        })
                        .collect::<Vec<_>>()
                })
                .unwrap_or_default();
            (url, method, headers)
        }
        _ => return Err("callback must be null, false, URL string, or object".to_owned()),
    };

    let method = Method::from_bytes(method.as_bytes())
        .map_err(|e| format!("invalid callback method {method}: {e}"))?;

    if dry_run {
        println!("callback {} {}", method.as_str(), url);
        return Ok(());
    }

    let client = reqwest::blocking::Client::new();
    let mut request = client.request(method, &url).json(payload);
    for (key, value) in headers {
        request = request.header(&key, value);
    }
    request
        .send()
        .and_then(reqwest::blocking::Response::error_for_status)
        .map(|_| ())
        .map_err(|e| format!("callback request failed: {e}"))
}

fn input_kind(input: &Map<String, Value>, location: &str) -> Result<String, String> {
    if let Some(kind) = optional_string(input, "type").or_else(|| optional_string(input, "kind")) {
        let kind = kind.to_ascii_lowercase();
        return match kind.as_str() {
            "stl" | "3mf" => Ok(kind),
            "model" => infer_kind_from_location(location),
            _ => Err(format!("unsupported input type: {kind}")),
        };
    }
    infer_kind_from_location(location)
}

fn infer_kind_from_location(location: &str) -> Result<String, String> {
    let lower = location.to_ascii_lowercase();
    if lower.ends_with(".stl") {
        Ok("stl".to_owned())
    } else if lower.ends_with(".3mf") {
        Ok("3mf".to_owned())
    } else {
        Err(format!(
            "could not infer input type from location, use input.type: {location}"
        ))
    }
}

fn first_location(
    obj: &Map<String, Value>,
    keys: &[&str],
    context: &str,
) -> Result<String, String> {
    for key in keys {
        if let Some(value) = obj.get(*key) {
            return location_value(value, &format!("{context}.{key}"));
        }
    }
    Err(format!("{context} requires one of: {}", keys.join(", ")))
}

fn optional_location(
    obj: &Map<String, Value>,
    key: &str,
    context: &str,
) -> Result<Option<String>, String> {
    obj.get(key)
        .map(|value| location_value(value, context).map(Some))
        .unwrap_or(Ok(None))
}

fn location_value(value: &Value, context: &str) -> Result<String, String> {
    if let Some(location) = value.as_str() {
        return Ok(location.to_owned());
    }
    if let Some(location) = object_location(value)? {
        return Ok(location);
    }
    Err(format!("{context} must be a location string or object"))
}

fn object_field<'a>(
    obj: &'a Map<String, Value>,
    key: &str,
) -> Result<&'a Map<String, Value>, String> {
    obj.get(key)
        .and_then(Value::as_object)
        .ok_or_else(|| format!("{key} must be an object"))
}

fn string_field(obj: &Map<String, Value>, key: &str) -> Result<String, String> {
    obj.get(key)
        .map(|value| string_value(value, key))
        .unwrap_or_else(|| Err(format!("{key} is required")))
}

fn optional_string(obj: &Map<String, Value>, key: &str) -> Option<String> {
    obj.get(key).and_then(Value::as_str).map(str::to_owned)
}

fn optional_path(obj: &Map<String, Value>, key: &str) -> Option<PathBuf> {
    optional_string(obj, key).map(PathBuf::from)
}

fn string_value(value: &Value, context: &str) -> Result<String, String> {
    value
        .as_str()
        .map(str::to_owned)
        .ok_or_else(|| format!("{context} must be a string"))
}

fn expect_object<'a>(value: &'a Value, context: &str) -> Result<&'a Map<String, Value>, String> {
    value
        .as_object()
        .ok_or_else(|| format!("{context} must be a JSON object"))
}

fn overlay(base: &mut Map<String, Value>, next: Map<String, Value>) {
    for (key, value) in next {
        base.insert(key, value);
    }
}

fn is_profile_file(path: &Path) -> bool {
    matches!(
        path.extension().and_then(|s| s.to_str()),
        Some("json") | Some("jsonnet")
    )
}

fn profile_scan_root(path: &Path) -> PathBuf {
    path.parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .or_else(|| path.parent().map(Path::to_path_buf))
        .unwrap_or_else(|| PathBuf::from("."))
}

fn default_native_binary() -> PathBuf {
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

fn write_json(path: &Path, value: &Value) -> Result<(), String> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|e| format!("create {}: {e}", parent.display()))?;
    }
    let file = fs::File::create(path).map_err(|e| format!("create {}: {e}", path.display()))?;
    serde_json::to_writer_pretty(file, value).map_err(|e| format!("write {}: {e}", path.display()))
}

fn is_s3_location(location: &str) -> bool {
    location.starts_with("s3://")
}

fn s3_filename(location: &str) -> Option<&str> {
    location
        .rsplit('/')
        .next()
        .filter(|name| !name.is_empty() && *name != "s3:")
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::TempDir;

    fn write_jsonnet(dir: &Path, subdir: &str, filename: &str, body: &str) -> PathBuf {
        let dir = dir.join(subdir);
        fs::create_dir_all(&dir).unwrap();
        let path = dir.join(filename);
        let mut file = fs::File::create(&path).unwrap();
        file.write_all(body.as_bytes()).unwrap();
        path
    }

    #[test]
    fn resolves_inherits_and_include() {
        let tmp = TempDir::new().unwrap();
        let root = tmp.path();

        write_jsonnet(
            root,
            "machine",
            "template.jsonnet",
            r#"{ name: "machine_template", machine_start_gcode: "G28" }"#,
        );
        write_jsonnet(
            root,
            "machine",
            "base.jsonnet",
            r#"{ name: "base_machine", nozzle_diameter: [0.4], bed_temperature: [50] }"#,
        );
        let machine = write_jsonnet(
            root,
            "machine",
            "leaf.jsonnet",
            r#"{ name: "leaf_machine", inherits: "base_machine", include: "machine_template", bed_temperature: [60] }"#,
        );
        let filament = write_jsonnet(
            root,
            "filament",
            "pla.jsonnet",
            r#"{ name: "pla", filament_type: ["PLA"] }"#,
        );
        let process = write_jsonnet(
            root,
            "process",
            "standard.jsonnet",
            r#"{ name: "standard", layer_height: 0.2 }"#,
        );

        let config = resolve_config(&machine, &filament, &process, &[]).unwrap();
        assert_eq!(config["nozzle_diameter"], serde_json::json!([0.4]));
        assert_eq!(config["machine_start_gcode"], serde_json::json!("G28"));
        assert_eq!(config["bed_temperature"], serde_json::json!([60]));
        assert_eq!(config["filament_type"], serde_json::json!(["PLA"]));
        assert!(!config.contains_key("inherits"));
    }

    #[test]
    fn resolves_inline_job_profiles() {
        let tmp = TempDir::new().unwrap();
        let root = tmp.path();

        write_jsonnet(
            root,
            "machine",
            "base.jsonnet",
            r#"{ name: "base_machine", nozzle_diameter: [0.4] }"#,
        );
        write_jsonnet(
            root,
            "process",
            "base.jsonnet",
            r#"{ name: "base_process", layer_height: 0.2 }"#,
        );
        write_jsonnet(
            root,
            "filament",
            "base.jsonnet",
            r#"{ name: "base_filament", filament_type: ["PLA"] }"#,
        );

        let value = serde_json::json!({
            "profile_roots": [root],
            "machine": { "name": "machine", "inherits": "base_machine", "bed_temperature": [55] },
            "process": { "name": "process", "inherits": "base_process" },
            "filament": { "name": "filament", "inherits": "base_filament", "filament_colour": ["#fff"] }
        });
        let mut temp_dirs = Vec::new();
        let config = resolve_config_value(&value, &mut temp_dirs).unwrap();
        assert_eq!(config["nozzle_diameter"], serde_json::json!([0.4]));
        assert_eq!(config["bed_temperature"], serde_json::json!([55]));
        assert_eq!(config["layer_height"], serde_json::json!(0.2));
        assert_eq!(config["filament_colour"], serde_json::json!(["#fff"]));
    }
}
