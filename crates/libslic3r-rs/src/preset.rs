use crate::print_config::PrintConfig;

/// Preset structure containing configuration and metadata
/// Preset.hpp:191-200
pub struct Preset {
    pub name: String,
    pub config: PrintConfig,
    pub is_system: bool,
    pub is_dirty: bool,
}

/// Implementation of Preset methods
/// Preset.cpp:50-100
impl Preset {
    // Create a new user preset with the given name and configuration
    // Preset.cpp:60-70
    pub fn new(name: impl Into<String>, config: PrintConfig) -> Self {
        Self {
            name: name.into(),
            config,
            is_system: false,
            is_dirty: false,
        }
    }

    /// Create a new system preset with the given name and configuration
    /// Preset.cpp:75-85
    pub fn system(name: impl Into<String>, config: PrintConfig) -> Self {
        Self {
            name: name.into(),
            config,
            is_system: true,
            is_dirty: false,
        }
    }
}

/// PresetBundle manages collections of presets for print, filament, and printer
/// PresetBundle.hpp:420-450
pub struct PresetBundle {
    pub print_presets: Vec<Preset>,
    pub filament_presets: Vec<Preset>,
    pub printer_presets: Vec<Preset>,
    pub selected_print: usize,
    pub selected_filament: usize,
    pub selected_printer: usize,
}

/// Implementation of PresetBundle methods
/// PresetBundle.cpp:100-200
impl PresetBundle {
    // Create a new empty preset bundle
    // PresetBundle.cpp:110-120
    pub fn new() -> Self {
        Self {
            print_presets: Vec::new(),
            filament_presets: Vec::new(),
            printer_presets: Vec::new(),
            selected_print: 0,
            selected_filament: 0,
            selected_printer: 0,
        }
    }

    /// Add a print preset and return its index
    /// PresetBundle.cpp:150-160
    pub fn add_print_preset(&mut self, preset: Preset) -> usize {
        // Get current length to use as new preset's index
        // PresetBundle.cpp:152
        let idx = self.print_presets.len();
        // Push preset to the vector
        // PresetBundle.cpp:155
        self.print_presets.push(preset);
        idx
    }

    /// Get a print preset by index
    /// PresetBundle.cpp:165-170
    pub fn get_print_preset(&self, idx: usize) -> Option<&Preset> {
        self.print_presets.get(idx)
    }
}
