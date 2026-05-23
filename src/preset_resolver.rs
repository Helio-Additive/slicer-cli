//! BambuStudio preset inheritance resolver.
//!
//! BBL profiles compose via two mechanisms:
//!   1. `inherits` — string: a single parent profile whose fields this one
//!      overrides (linear chain).
//!   2. `include`  — string OR array of strings: sibling profiles whose
//!      fields get overlaid before this profile's own fields. Used primarily
//!      to pull in large gcode templates (machine_start_gcode, machine_end_gcode,
//!      change_filament_gcode, layer_change_gcode, time_lapse_gcode) from
//!      separate "template" files so the main machine profile stays small.
//!
//! Resolution order (matches `PresetBundle::load_system_presets` in
//! `references/BambuStudio/src/libslic3r/PresetBundle.cpp:4763-4802`):
//!   root → ... → leaf, and at each level:
//!     (a) merge in each included profile (recursively resolved) first,
//!     (b) then overlay this level's own fields, so the current profile wins.
//!
//! `inherits` crosses root boundaries transparently — a user-imported profile
//! may inherit a BBL base because both names resolve through the same flat
//! `name_to_path` map.

use crate::profiles::ProfilesIndex;
use serde_json::{Map, Value};
use std::{
    collections::{BTreeMap, HashMap, HashSet},
    path::PathBuf,
};

const STRIP_KEYS: &[&str] = &[
    "inherits",
    "include",
    "type",
    "instantiation",
    "setting_id",
    "description",
    "label",
    "version",
];

// ── Public API ────────────────────────────────────────────────────────────────

/// Fully-resolved profile as a flat JSON object. Walks the `inherits` chain
/// and applies every `include` at each level.
pub fn resolve_profile(index: &ProfilesIndex, name: &str) -> Result<Map<String, Value>, String> {
    resolve_with_map(&index.name_to_path, name)
}

/// Resolve a profile using a pre-built name→absolute-path map. Used inside
/// `ProfilesIndex::load_multi` before the index is fully constructed.
pub fn resolve_with_map(
    name_to_path: &HashMap<String, PathBuf>,
    name: &str,
) -> Result<Map<String, Value>, String> {
    let loader = |n: &str| -> Result<Value, String> {
        let path = name_to_path
            .get(n)
            .ok_or_else(|| format!("Profile not found in map: {n}"))?;
        let raw = std::fs::read_to_string(path).map_err(|e| format!("Cannot read '{n}': {e}"))?;
        serde_json::from_str(&raw).map_err(|e| format!("JSON parse error for '{n}': {e}"))
    };
    let mut merged: Map<String, Value> = Map::new();
    let mut visited: HashSet<String> = HashSet::new();
    resolve_into(&loader, name, &mut merged, &mut visited)?;
    Ok(merged)
}

/// Resolve a profile whose chain may reference both (a) other in-memory
/// profiles from an external bundle and (b) profiles already installed in
/// `existing_index`. Used by `POST /api/profiles/resolve-external` so a
/// dropped bundle or deep-inherits JSON can be fully flattened without
/// staging the import to disk.
///
/// Lookup order at each chain step:
///   1. `bundle_profiles_by_name`  — profiles shipped inside the bundle
///   2. `existing_index.name_to_path` — installed / built-in profiles
pub fn resolve_with_bundle(
    bundle_profiles_by_name: &BTreeMap<String, Value>,
    existing_index: Option<&ProfilesIndex>,
    name: &str,
) -> Result<Map<String, Value>, String> {
    let loader = |n: &str| -> Result<Value, String> {
        if let Some(v) = bundle_profiles_by_name.get(n) {
            return Ok(v.clone());
        }
        if let Some(idx) = existing_index {
            if let Some(path) = idx.name_to_path.get(n) {
                let raw =
                    std::fs::read_to_string(path).map_err(|e| format!("Cannot read '{n}': {e}"))?;
                return serde_json::from_str(&raw)
                    .map_err(|e| format!("JSON parse error for '{n}': {e}"));
            }
        }
        Err(format!("Profile not found: {n}"))
    };
    let mut merged: Map<String, Value> = Map::new();
    let mut visited: HashSet<String> = HashSet::new();
    resolve_into(&loader, name, &mut merged, &mut visited)?;
    Ok(merged)
}

// ── Internals ─────────────────────────────────────────────────────────────────

fn resolve_into<F>(
    loader: &F,
    name: &str,
    merged: &mut Map<String, Value>,
    visited: &mut HashSet<String>,
) -> Result<(), String>
where
    F: Fn(&str) -> Result<Value, String>,
{
    if !visited.insert(name.to_owned()) {
        return Err(format!("Circular reference at: {name}"));
    }

    let chain = build_chain(loader, name)?;
    for json in chain.into_iter().rev() {
        apply_level(json, loader, merged, visited)?;
    }

    Ok(())
}

/// Apply one level of the inherits chain to `merged`: first resolve every
/// profile listed in its `include` field, then overlay this level's own
/// fields so they win over included values.
fn apply_level<F>(
    json: Value,
    loader: &F,
    merged: &mut Map<String, Value>,
    visited: &mut HashSet<String>,
) -> Result<(), String>
where
    F: Fn(&str) -> Result<Value, String>,
{
    let Value::Object(obj) = json else {
        return Ok(());
    };

    let includes = parse_include_field(obj.get("include"));
    for inc_name in includes {
        resolve_into(loader, &inc_name, merged, visited)?;
    }

    for (k, v) in obj {
        if STRIP_KEYS.contains(&k.as_str()) {
            continue;
        }
        merged.insert(k, v);
    }

    Ok(())
}

/// BBS accepts `"include"` as either a string (single name) or an array of
/// strings. Anything else is treated as no includes.
fn parse_include_field(val: Option<&Value>) -> Vec<String> {
    match val {
        Some(Value::Array(arr)) => arr
            .iter()
            .filter_map(|v| v.as_str().map(str::to_owned))
            .collect(),
        Some(Value::String(s)) => vec![s.clone()],
        _ => Vec::new(),
    }
}

fn build_chain<F>(loader: &F, start: &str) -> Result<Vec<Value>, String>
where
    F: Fn(&str) -> Result<Value, String>,
{
    let mut chain: Vec<Value> = Vec::new();
    let mut current = start.to_owned();
    let mut visited = HashSet::new();

    loop {
        if !visited.insert(current.clone()) {
            return Err(format!("Circular inheritance at: {current}"));
        }

        let json = loader(&current)?;

        let inherits = json
            .get("inherits")
            .and_then(|v| v.as_str())
            .map(str::to_owned);

        chain.push(json);

        match inherits.as_deref() {
            Some("") | None => break, // empty string = no parent (same as absent)
            Some(parent) => current = parent.to_owned(),
        }
    }

    Ok(chain)
}
