use serde_json::{Map, Value};
use std::{
    collections::{BTreeSet, HashMap, HashSet},
    fs,
    path::{Path, PathBuf},
    process::Command,
};
use tempfile::TempDir;
use walkdir::WalkDir;

use crate::{
    json_utils::{expect_object, overlay, string_value},
    locations::{is_s3_location, materialize_input, object_location},
};

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

enum ProfileSource {
    Path(PathBuf),
    Inline(Value),
}

pub fn resolve_config(
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

pub fn resolve_config_refs(
    machine: &str,
    filament: &str,
    process: &str,
    extra_roots: &[PathBuf],
) -> Result<Map<String, Value>, String> {
    if let (Some(machine), Some(filament), Some(process)) = (
        existing_profile_path(machine),
        existing_profile_path(filament),
        existing_profile_path(process),
    ) {
        return resolve_config(&machine, &filament, &process, extra_roots);
    }

    let profile_paths = [machine, filament, process]
        .into_iter()
        .filter_map(existing_profile_path)
        .collect::<Vec<_>>();
    let index = ProfilesIndex::load_roots(&profile_paths, extra_roots)?;

    let mut merged = Map::new();
    for reference in [machine, process, filament] {
        let profile = index.resolve_ref(reference)?;
        overlay(&mut merged, profile);
    }
    finalize_resolved_config(&mut merged);
    Ok(merged)
}

pub fn resolve_config_value(
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

pub fn load_profile(path: &Path) -> Result<Value, String> {
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

    fn resolve_ref(&self, reference: &str) -> Result<Map<String, Value>, String> {
        if let Some(path) = existing_profile_path(reference) {
            return self.resolve_path(&path);
        }
        self.resolve_name(reference)
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

fn is_profile_file(path: &Path) -> bool {
    matches!(
        path.extension().and_then(|s| s.to_str()),
        Some("json") | Some("jsonnet")
    )
}

fn existing_profile_path(reference: &str) -> Option<PathBuf> {
    let path = PathBuf::from(reference);
    path.exists().then_some(path)
}

fn profile_scan_root(path: &Path) -> PathBuf {
    path.parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .or_else(|| path.parent().map(Path::to_path_buf))
        .unwrap_or_else(|| PathBuf::from("."))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

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
