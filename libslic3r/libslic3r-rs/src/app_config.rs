//! AppConfig - Application configuration manager.
//!
//! Mirrors BambuStudio's `AppConfig` class for managing user preferences and settings.
//! stored in an INI-style format.

use std::collections::{HashMap, HashSet};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Application mode enum (Editor vs GCodeViewer)
/// AppConfig.hpp:33-37
pub enum AppMode {
    Editor,
    GCodeViewer,
}

#[derive(Debug, Clone)]
/// Application configuration manager, stores section->key->value settings
/// AppConfig.hpp:30-299
pub struct AppConfig {
    /// Internal storage: Section -> Key -> Value
    /// AppConfig.hpp:280
    storage: HashMap<String, HashMap<String, String>>,
    /// Dirty flag for unsaved changes
    /// AppConfig.hpp:285
    dirty: bool,
    /// Original version string from the loaded file
    /// AppConfig.hpp:287
    orig_version: String,
    /// Whether the data directory is legacy (pre-1.40.0)
    /// AppConfig.hpp:289
    legacy_datadir: bool,
    /// Current application mode
    /// AppConfig.hpp:278
    mode: AppMode,
    /// Vendor map: Vendor -> Model -> Variants
    /// AppConfig.hpp:283
    vendors: HashMap<String, HashMap<String, HashSet<String>>>,
}

/// Default implementation for AppConfig, delegates to new()
/// AppConfig.hpp:40-47
impl Default for AppConfig {
    // Default trait implementation delegates to constructor
    // AppConfig.hpp:40-47
    fn default() -> Self {
        // AppConfig.hpp:40
        Self::new()
    }
}

/// AppConfig methods implementation
/// AppConfig.hpp:30-299
impl AppConfig {
    // Create a new, empty AppConfig with defaults
    // AppConfig.hpp:40-47
    pub fn new() -> Self {
        // AppConfig.hpp:41-44
        let mut config = Self {
            storage: HashMap::new(),
            dirty: false,
            orig_version: String::new(),
            legacy_datadir: false,
            mode: AppMode::Editor,
            vendors: HashMap::new(),
        };
        // AppConfig.hpp:46
        config.reset();
        // AppConfig.hpp:47
        config
    }

    /// Reset configuration to defaults, clearing all storage
    /// AppConfig.cpp:84-88
    pub fn reset(&mut self) {
        // AppConfig.cpp:86
        self.storage.clear();
        // AppConfig.cpp:86
        self.vendors.clear();
        // AppConfig.cpp:86
        self.dirty = false;
        // AppConfig.cpp:87
        self.set_str("app", "version", env!("CARGO_PKG_VERSION"));
    }

    /// Get a value from a specific section and key
    /// AppConfig.hpp:70-81
    pub fn get(&self, section: &str, key: &str) -> Option<&String> {
        // AppConfig.hpp:73-78
        self.storage.get(section).and_then(|s| s.get(key))
    }

    /// Get a value from the default app section
    /// AppConfig.hpp:84-85
    pub fn get_app_key(&self, key: &str) -> Option<&String> {
        // AppConfig.hpp:85
        self.get("app", key)
    }

    /// Get a boolean value checking true or 1
    /// AppConfig.hpp:86
    pub fn get_bool(&self, section: &str, key: &str) -> bool {
        // AppConfig.hpp:86
        if let Some(val) = self.get(section, key) {
            // AppConfig.hpp:86
            val == "true" || val == "1"
        } else {
            // AppConfig.hpp:86
            false
        }
    }

    /// Set a string value in a section, marking dirty if changed
    /// AppConfig.hpp:104-119
    pub fn set_str(&mut self, section: &str, key: &str, value: &str) {
        // AppConfig.hpp:114
        let section_map = self.storage.entry(section.to_string()).or_default();
        // AppConfig.hpp:115-118
        match section_map.get_mut(key) {
            Some(existing) => {
                // AppConfig.hpp:115-116
                if existing != value {
                    // AppConfig.hpp:117
                    *existing = value.to_string();
                    // AppConfig.hpp:118
                    self.dirty = true;
                }
            }
            None => {
                // AppConfig.hpp:114
                section_map.insert(key.to_string(), value.to_string());
                // AppConfig.hpp:118
                self.dirty = true;
            }
        }
    }

    /// Set a boolean value in a section
    /// AppConfig.hpp:121-128
    pub fn set_bool(&mut self, section: &str, key: &str, value: bool) {
        // AppConfig.hpp:123-127
        let val_str =
            // AppConfig.hpp:124-126
            match value {
                true => "true",
                false => "false",
            };
        // AppConfig.hpp:123-127
        self.set_str(section, key, val_str);
    }

    /// Check if a key exists in a section
    /// AppConfig.hpp:139-146
    pub fn has(&self, section: &str, key: &str) -> bool {
        // AppConfig.hpp:141-145
        self.storage
            .get(section)
            .map(|s| s.contains_key(key))
            .unwrap_or(false)
    }

    /// Check if a section exists
    /// AppConfig.hpp:158-159
    pub fn has_section(&self, section: &str) -> bool {
        // AppConfig.hpp:159
        self.storage.contains_key(section)
    }

    /// Check if there are unsaved changes
    /// AppConfig.hpp:64
    pub fn dirty(&self) -> bool {
        // AppConfig.hpp:64
        self.dirty
    }

    /// Mark config as dirty
    /// AppConfig.hpp:67
    pub fn set_dirty(&mut self) {
        // AppConfig.hpp:67
        self.dirty = true;
    }

    /// Get the original version string
    /// AppConfig.hpp:232
    pub fn orig_version(&self) -> &str {
        // AppConfig.hpp:232
        &self.orig_version
    }

    /// Enable or disable a printer variant in the vendor map
    /// AppConfig.cpp:1139-1155
    pub fn set_variant(&mut self, vendor: &str, model: &str, variant: &str, enable: bool) {
        // AppConfig.cpp:1141
        let vendor_map = self.vendors.entry(vendor.to_string()).or_default();
        // AppConfig.cpp:1142
        let model_set = vendor_map.entry(model.to_string()).or_default();

        // AppConfig.cpp:1141-1151
        if enable {
            // AppConfig.cpp:1142-1143
            if model_set.insert(variant.to_string()) {
                // AppConfig.cpp:1154
                self.dirty = true;
            }
        } else {
            // AppConfig.cpp:1144-1151
            if model_set.remove(variant) {
                // AppConfig.cpp:1154
                self.dirty = true;
            }
        }
    }

    /// Check if a printer variant is enabled in the vendor map
    /// AppConfig.cpp:1131-1137
    pub fn get_variant(&self, vendor: &str, model: &str, variant: &str) -> bool {
        // AppConfig.cpp:1133-1136
        self.vendors
            .get(vendor)
            .and_then(|m| m.get(model))
            .map(|s| s.contains(variant))
            .unwrap_or(false)
    }
}
