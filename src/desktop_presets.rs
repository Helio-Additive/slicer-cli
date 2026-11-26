//! The Bambu root CLI's file inheritance path (desktop_presets.cpp 75–152,
//! 197–319, 356–424 and main.cpp 5459–5517).
use slicer::config::{ConfigBase, ConfigSubstitutionContext, ForwardCompatibilitySubstitutionRule};
use slicer::print_config::config_def::DynamicPrintConfig;
use std::{
    collections::BTreeMap,
    fs,
    path::{Path, PathBuf},
};

type Metadata = BTreeMap<String, String>;

fn read_preset(file: &Path) -> Option<(DynamicPrintConfig, Metadata)> {
    let mut config = DynamicPrintConfig::new();
    let mut metadata = Metadata::new();
    let mut reason = String::new();
    let mut substitutions =
        ConfigSubstitutionContext::new(ForwardCompatibilitySubstitutionRule::EnableSilent);
    if config.load_from_json_with_context(
        file.to_str()?,
        &mut substitutions,
        false,
        &mut metadata,
        &mut reason,
    ) != 0
        || !reason.is_empty()
    {
        return None;
    }
    Some((config, metadata))
}

struct Vendor {
    directory: PathBuf,
    kind: String,
    entries: Vec<serde_json::Value>,
}

impl Vendor {
    fn open(root: &Path, name: &str, kind: &str) -> Self {
        let bundle = fs::File::open(root.join(format!("{name}.json")))
            .ok()
            .and_then(|file| {
                serde_json::from_reader::<_, serde_json::Value>(std::io::BufReader::new(file)).ok()
            });
        let key = format!("{kind}_list");
        let entries = bundle
            .and_then(|doc| doc.get(&key)?.as_array().cloned())
            .unwrap_or_default();
        Self {
            directory: root.join(name),
            kind: kind.into(),
            entries,
        }
    }
    fn preset(&self, name: &str) -> Option<PathBuf> {
        if name.is_empty() {
            return None;
        }
        if let Some(entry) = self
            .entries
            .iter()
            .find(|entry| entry["name"].as_str() == Some(name))
        {
            if let Some(sub_path) = entry["sub_path"].as_str() {
                let file = self.directory.join(sub_path);
                if file.is_file() {
                    return Some(file);
                }
            }
        }
        let file = self.directory.join(&self.kind).join(format!("{name}.json"));
        file.is_file().then_some(file)
    }
}

fn included_names(metadata: &Metadata) -> Vec<String> {
    let Some(text) = metadata.get("include") else {
        return Vec::new();
    };
    if let Some(name) = text.strip_prefix('"').and_then(|s| s.strip_suffix('"')) {
        return vec![name.into()];
    }
    match serde_json::from_str::<serde_json::Value>(text) {
        Ok(serde_json::Value::Array(items)) => items
            .into_iter()
            .filter_map(|v| v.as_str().map(str::to_owned))
            .collect(),
        _ => Vec::new(),
    }
}

fn flatten(file: &Path, vendor: Option<&Vendor>, depth: u32) -> Option<DynamicPrintConfig> {
    if depth > 16 {
        return None;
    }
    let (own, metadata) = read_preset(file)?;
    let folder = file.parent()?;
    let mut base = DynamicPrintConfig::new();
    if let Some(parent) = metadata.get("inherits").filter(|p| !p.is_empty()) {
        let parent = match vendor {
            Some(vendor) => vendor.preset(parent)?,
            None => folder.join(format!("{parent}.json")),
        };
        base = flatten(&parent, vendor, depth + 1)?;
    }
    for name in included_names(&metadata) {
        let Some(included) = flatten(&folder.join(format!("{name}.json")), None, depth + 1) else {
            continue;
        };
        let keys = included
            .diff(&DynamicPrintConfig::full_print_config())
            .ok()?;
        base.apply_only(&included, &keys, true).ok()?;
    }
    base.apply(&own, true).ok()?;
    Some(base)
}

pub(crate) fn inherited_settings(root: &Path, file: &Path) -> Option<DynamicPrintConfig> {
    let (own, meta) = read_preset(file)?;
    let kind = meta.get("type")?.as_str();
    if !matches!(kind, "machine" | "process" | "filament") {
        return None;
    }
    let parent = meta.get("inherits").filter(|name| !name.is_empty())?;
    let folder = file.parent()?;
    let preferred = folder
        .parent()
        .and_then(Path::file_name)
        .and_then(|s| s.to_str())
        .filter(|vendor| {
            folder.file_name().is_some_and(|s| s == kind) && root.join(vendor).exists()
        });
    let mut vendors: Vec<String> = fs::read_dir(root)
        .ok()?
        .flatten()
        .map(|entry| entry.path())
        .filter(|file| file.extension().is_some_and(|s| s == "json"))
        .filter_map(|file| Some(file.file_stem()?.to_str()?.to_owned()))
        .collect();
    vendors.sort();
    for vendor in preferred
        .into_iter()
        .chain(vendors.iter().map(String::as_str))
    {
        let vendor = Vendor::open(root, vendor, kind);
        let Some(path) = vendor.preset(parent) else {
            continue;
        };
        if let Some(mut config) = flatten(&path, Some(&vendor), 0) {
            config.apply(&own, true).ok()?;
            config
                .set_deserialize_strict("inherits", parent, false)
                .ok()?;
            return Some(config);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use slicer::config::ConfigOptionResolver;

    #[test]
    fn external_files_resolve_the_native_machine_process_and_filament_parents() {
        let profiles = Path::new(env!("CARGO_MANIFEST_DIR")).join("references/BambuStudio/resources/profiles");
        let work = tempfile::tempdir().unwrap();
        let file = work.path().join("settings.json");
        for row in include_str!("../tests/rust-engine/data/desktop_presets/inherited.jsonl").lines() {
            let case: serde_json::Value = serde_json::from_str(row).unwrap();
            fs::write(&file, serde_json::to_vec(&case["file"]).unwrap()).unwrap();
            let config = inherited_settings(&profiles, &file);
            assert_eq!(config.is_some(), case["resolved"].as_bool().unwrap(), "{}", case["file"]);
            let Some(config) = config else { continue; };
            for key in ["inherits", "nozzle_diameter", "printer_model", "printer_variant",
                "wall_loops", "top_shell_layers", "layer_height", "sparse_infill_density",
                "filament_type", "filament_diameter", "filament_density", "nozzle_temperature",
                "printable_area", "use_relative_e_distances", "machine_start_gcode"] {
                let expected = case["selected"].get(key);
                let Some(option) = config.option(key) else {
                    assert!(expected.is_none(), "missing {key} for {}", case["file"]);
                    continue;
                };
                let value = option.serialize().unwrap();
                let actual = if key == "machine_start_gcode" {
                    let hash = value.bytes().fold(14695981039346656037u64,
                        |hash, byte| (hash ^ u64::from(byte)).wrapping_mul(1099511628211));
                    serde_json::json!([value.len(), hash])
                } else { value.into() };
                assert_eq!(Some(&actual), expected, "{key} for {}", case["file"]);
            }
        }
    }
}
