//! BambuStudio BBL profile browser.
//!
//! Provides:
//!   GET /api/profiles — list all instantiable printers / processes / filaments
//!   GET /api/profiles/:type/:name — raw JSON for one profile (URL-encoded name)
//!
//! The index supports multiple roots: the bundled BBL profiles live under one
//! root, and each imported user vendor bundle lives under its own root below
//! the user-data `user-profiles/` dir. Built-in and user profiles share a
//! single flat name → absolute-path lookup, which means a user profile can
//! `inherits` a BBL base just like a BBL profile can.

use serde::Serialize;
use std::{
    collections::HashMap,
    path::{Path, PathBuf},
};

// ── Origin ────────────────────────────────────────────────────────────────────

/// Where a profile came from. Exposed on each entry so the UI can badge
/// user-imported profiles and so the agent can filter by provenance.
#[derive(Debug, Clone, Serialize, PartialEq, Eq)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum Origin {
    /// Bundled at build time under `references/BambuStudio/resources/profiles/BBL/`.
    Builtin,
    /// Imported at runtime. `vendor` is the top-level directory name under
    /// `user-profiles/` and matches the vendor bundle's `name` field.
    User { vendor: String },
    /// Discovered on the user's machine in an existing BambuStudio /
    /// OrcaSlicer install . `vendor` is encoded as
    /// `"<Slicer>/<scope>/<name>"`, e.g. `"BambuStudio/system/BBL"` or
    /// `"OrcaSlicer/user/default"`. Slash-delimited so the UI can split
    /// for display while the wire format stays a single string.
    LocalInstall { vendor: String },
}

impl Origin {
    pub fn is_builtin(&self) -> bool {
        matches!(self, Origin::Builtin)
    }

    /// Human-readable label for log lines and conflict descriptions.
    pub fn label(&self) -> String {
        match self {
            Origin::Builtin => "builtin (BBL)".to_string(),
            Origin::User { vendor } => format!("imported vendor '{vendor}'"),
            Origin::LocalInstall { vendor } => format!("local install '{vendor}'"),
        }
    }
}

/// One directory the index should scan. `dir` must contain `machine/`, `process/`,
/// and `filament/` subdirectories following the BBL layout.
#[derive(Debug, Clone)]
pub struct ProfileRoot {
    pub dir: PathBuf,
    pub origin: Origin,
}

// ── Response types ────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Serialize)]
pub struct MachineEntry {
    pub name: String,
    pub inherits: Option<String>,
    pub printer_model: Option<String>,
    /// First nozzle diameter from the `nozzle_diameter` array (e.g. `"0.4"`).
    pub nozzle_diameter: Option<String>,
    pub default_print_profile: Option<String>,
    /// Printable area corners, e.g. `["0x0","256x0","256x256","0x256"]`.
    pub printable_area: Vec<String>,
    /// Maximum printable height in mm.
    pub printable_height: Option<u32>,
    /// Bed model STL filename (e.g. `"bbl-3dp-X1.stl"`).
    pub bed_model: Option<String>,
    pub origin: Origin,
}

#[derive(Debug, Clone, Serialize)]
pub struct ProcessEntry {
    pub name: String,
    pub inherits: Option<String>,
    /// Machine names this process is compatible with.
    pub compatible_printers: Vec<String>,
    pub origin: Origin,
}

#[derive(Debug, Clone, Serialize)]
pub struct FilamentEntry {
    pub name: String,
    pub inherits: Option<String>,
    /// Machine names this filament is compatible with.
    pub compatible_printers: Vec<String>,
    /// Material category (e.g. `"PLA"`, `"ABS"`) — present if the file contains it.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filament_type: Option<String>,
    /// Brand / vendor name (e.g. `"Polymaker"`, `"eSUN"`) from the `filament_vendor` key.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub filament_vendor: Option<String>,
    pub origin: Origin,
}

#[derive(Debug, Serialize)]
pub struct ProfilesListing {
    pub machines: Vec<MachineEntry>,
    pub processes: Vec<ProcessEntry>,
    pub filaments: Vec<FilamentEntry>,
}

// ── Index ─────────────────────────────────────────────────────────────────────

/// Pre-built index of all instantiable profiles across every configured root.
#[derive(Clone)]
pub struct ProfilesIndex {
    pub roots: Vec<ProfileRoot>,
    pub machines: Vec<MachineEntry>,
    pub processes: Vec<ProcessEntry>,
    pub filaments: Vec<FilamentEntry>,
    /// name → absolute file path. Shared across all roots so inheritance
    /// and `include` resolution can cross root boundaries transparently.
    pub name_to_path: HashMap<String, PathBuf>,
    /// name → origin, parallel to `name_to_path` (entries not surfaced in
    /// the public listing — templates, non-instantiable bases — still live
    /// here so we can tell which root a `raw_profile` hit came from).
    pub name_to_origin: HashMap<String, Origin>,
}

impl ProfilesIndex {
    /// Backwards-compatible loader: one builtin root.
    pub fn load(bbl_dir: PathBuf) -> Option<Self> {
        Self::load_multi(vec![ProfileRoot {
            dir: bbl_dir,
            origin: Origin::Builtin,
        }])
    }

    /// Scan every root and build a unified index. Returns `None` if no root
    /// yielded at least one profile.
    pub fn load_multi(roots: Vec<ProfileRoot>) -> Option<Self> {
        fn parse_json_file(path: &Path) -> Option<serde_json::Value> {
            let content = std::fs::read_to_string(path).ok()?;
            serde_json::from_str(&content).ok()
        }

        // Recursively collect all .json file paths under `dir`.
        fn walk_json_files(dir: &Path, out: &mut Vec<PathBuf>) {
            let Ok(rd) = std::fs::read_dir(dir) else {
                return;
            };
            for entry in rd.flatten() {
                let path = entry.path();
                if path.is_dir() {
                    walk_json_files(&path, out);
                } else if path.extension().and_then(|e| e.to_str()) == Some("json") {
                    out.push(path);
                }
            }
        }

        // ── Pass 1: build complete name_to_path across ALL roots & subdirs ────
        // Includes non-instantiable bases (`fdm_filament_*`, etc.) and
        // `*template*` files — both referenced by `inherits` / `include`.
        let mut name_to_path: HashMap<String, PathBuf> = HashMap::new();
        let mut name_to_origin: HashMap<String, Origin> = HashMap::new();

        for root in &roots {
            if !root.dir.is_dir() {
                eprintln!("Profile root missing: {:?}", root.dir);
                continue;
            }
            for subdir in &["machine", "process", "filament"] {
                let dir = root.dir.join(subdir);
                let mut paths = Vec::new();
                walk_json_files(&dir, &mut paths);
                for path in paths {
                    let Some(json) = parse_json_file(&path) else {
                        continue;
                    };
                    let Some(name) = json.get("name").and_then(|v| v.as_str()) else {
                        continue;
                    };
                    // Later roots overwrite earlier ones under the same name.
                    // Builtins load first, so user imports win conflicts at
                    // lookup time — matches the conflict policy surfaced to
                    // the user during `start_import`.
                    name_to_path.insert(name.to_owned(), path.clone());
                    name_to_origin.insert(name.to_owned(), root.origin.clone());
                }
            }
        }

        // ── Pass 2: resolve each instantiable profile via inheritance chain ───
        let mut machines = Vec::new();
        let mut processes = Vec::new();
        let mut filaments = Vec::new();

        for root in &roots {
            for (subdir, kind) in [
                ("machine", "machine"),
                ("process", "process"),
                ("filament", "filament"),
            ] {
                let dir = root.dir.join(subdir);
                let mut all_paths = Vec::new();
                walk_json_files(&dir, &mut all_paths);

                for path in all_paths {
                    let fname = path.file_name().and_then(|f| f.to_str()).unwrap_or("");
                    if fname.contains("template") {
                        continue;
                    }

                    let Some(json) = parse_json_file(&path) else {
                        continue;
                    };
                    let Some(name) = json.get("name").and_then(|v| v.as_str()) else {
                        continue;
                    };

                    // Only instantiable profiles surface in the public listing.
                    let instantiation = json
                        .get("instantiation")
                        .and_then(|v| v.as_str())
                        .unwrap_or("true");
                    if instantiation != "true" {
                        continue;
                    }

                    // Skip if a later root shadowed this name — we'll emit it
                    // when we scan that root. This avoids listing the same name
                    // twice and keeps the "user wins conflict" policy.
                    if name_to_path.get(name).map(|p| p != &path).unwrap_or(false) {
                        continue;
                    }

                    let inherits = json
                        .get("inherits")
                        .and_then(|v| v.as_str())
                        .map(str::to_owned);

                    // Fully resolve this profile via its inheritance chain.
                    let resolved =
                        match crate::preset_resolver::resolve_with_map(&name_to_path, name) {
                            Ok(m) => m,
                            Err(e) => {
                                eprintln!("Could not resolve profile '{}': {}", name, e);
                                continue;
                            }
                        };

                    let str_arr = |key: &str| -> Vec<String> {
                        resolved
                            .get(key)
                            .and_then(|v| v.as_array())
                            .map(|arr| {
                                arr.iter()
                                    .filter_map(|x| x.as_str().map(str::to_owned))
                                    .collect()
                            })
                            .unwrap_or_default()
                    };

                    let str_arr_first = |key: &str| -> Option<String> {
                        resolved
                            .get(key)?
                            .as_array()?
                            .first()?
                            .as_str()
                            .map(str::to_owned)
                    };

                    let str_val = |key: &str| -> Option<String> {
                        resolved.get(key)?.as_str().map(str::to_owned)
                    };

                    let parse_height = |key: &str| -> Option<u32> {
                        let v = resolved.get(key)?;
                        v.as_str()
                            .and_then(|s| s.parse::<u32>().ok())
                            .or_else(|| v.as_u64().map(|n| n as u32))
                            .or_else(|| v.as_f64().map(|f| f as u32))
                    };

                    match kind {
                        "machine" => {
                            // bed_model lives on the printer-model profile
                            // (e.g. "Bambu Lab X1 Carbon"), not on the
                            // nozzle-variant. Look it up via printer_model.
                            let bed_model = str_val("bed_model").or_else(|| {
                                let model_name = str_val("printer_model")?;
                                crate::preset_resolver::resolve_with_map(&name_to_path, &model_name)
                                    .ok()
                                    .and_then(|m| m.get("bed_model")?.as_str().map(str::to_owned))
                            });

                            machines.push(MachineEntry {
                                name: name.to_owned(),
                                inherits,
                                printer_model: str_val("printer_model"),
                                nozzle_diameter: str_arr_first("nozzle_diameter"),
                                default_print_profile: str_val("default_print_profile"),
                                printable_area: str_arr("printable_area"),
                                printable_height: parse_height("printable_height"),
                                bed_model,
                                origin: root.origin.clone(),
                            });
                        }
                        "process" => processes.push(ProcessEntry {
                            name: name.to_owned(),
                            inherits,
                            compatible_printers: str_arr("compatible_printers"),
                            origin: root.origin.clone(),
                        }),
                        "filament" => filaments.push(FilamentEntry {
                            name: name.to_owned(),
                            inherits,
                            filament_type: str_arr_first("filament_type"),
                            filament_vendor: str_arr_first("filament_vendor"),
                            compatible_printers: str_arr("compatible_printers"),
                            origin: root.origin.clone(),
                        }),
                        _ => {}
                    }
                }
            }
        }

        machines.sort_by(|a, b| a.name.cmp(&b.name));
        processes.sort_by(|a, b| a.name.cmp(&b.name));
        filaments.sort_by(|a, b| a.name.cmp(&b.name));

        if machines.is_empty() && processes.is_empty() && filaments.is_empty() {
            eprintln!("Profile index empty — no usable roots");
            return None;
        }

        eprintln!(
            "Profiles indexed across {} roots: {} machines, {} processes, {} filaments",
            roots.len(),
            machines.len(),
            processes.len(),
            filaments.len()
        );

        Some(ProfilesIndex {
            roots,
            machines,
            processes,
            filaments,
            name_to_path,
            name_to_origin,
        })
    }

    /// Rebuild the index from the current roots. Call after a user-profile
    /// commit to pick up newly added vendors. Note that `roots` itself is
    /// unchanged — the caller is expected to mutate it (via `with_roots`)
    /// when a brand-new vendor directory appears.
    pub fn rescan(&self) -> Option<Self> {
        Self::load_multi(self.roots.clone())
    }

    /// Return a new index with `roots` replaced. Used when a commit introduces
    /// a vendor directory not yet in the root list.
    pub fn with_roots(roots: Vec<ProfileRoot>) -> Option<Self> {
        Self::load_multi(roots)
    }

    /// Return the raw file content for a profile by name.
    pub fn raw_profile(&self, name: &str) -> Option<String> {
        let path = self.name_to_path.get(name)?;
        std::fs::read_to_string(path).ok()
    }

    /// Lookup a machine entry by name.
    pub fn machine(&self, name: &str) -> Option<&MachineEntry> {
        self.machines.iter().find(|m| m.name == name)
    }

    /// Return the root directory whose origin matches the given origin.
    /// Used to locate sibling assets like bed STL files.
    pub fn root_dir_for(&self, origin: &Origin) -> Option<&Path> {
        self.roots
            .iter()
            .find(|r| r.origin == *origin)
            .map(|r| r.dir.as_path())
    }
}
