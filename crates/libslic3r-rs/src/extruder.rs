//! Extruder state management for G-code generation.
//!
//! 1:1 port of `Extruder.cpp` / `Extruder.hpp` (BambuStudio src/libslic3r).
//!
//! `coord_t` -> `i64`, `coordf_t` -> `f64`. This file faithfully mirrors the
//! control flow, constants, rounding and edge cases of the C++ source.
//!
//! Dependency note: the C++ `Extruder` holds a `GCodeConfig *` (a huge
//! `ConfigOption`-based config class) and reads it through `ConfigOptionVector`
//! accessors plus the free function `get_filament_config_idx` (defined in
//! `PrintConfig.cpp`). The Rust crate's `print_config::GCodeConfig` is a small
//! hand-rolled subset that does NOT carry the per-filament config vectors
//! (`filament_map`, `filament_nozzle_map`, `nozzle_volume_type`,
//! `extruder_type`, `filament_diameter`, ...), nor `ConfigOptionVector::get_at`,
//! nor `get_filament_config_idx`. To keep this file a faithful 1:1 translation
//! that builds and is independently testable, the exact fields the C++
//! `Extruder` reads are modelled here as a module-local `GCodeConfig` with
//! `ConfigOptionVector`-faithful `get_at` semantics, and the small deterministic
//! `get_filament_config_idx` chain (`get_extruder_index`,
//! `get_config_index_base`, `get_extruder_variant_string`) is ported faithfully
//! from `PrintConfig.cpp`. When the full `GCodeConfig`/`ConfigOptionVector`
//! infrastructure is ported, this local model should be replaced by the real
//! one. wasm-safe: no system/dylib dependencies.

use std::f64::consts::PI;

// ===========================================================================
// Enums and helpers ported faithfully from PrintConfig.hpp / PrintConfig.cpp,
// because the Extruder's config accessors require them and the Rust crate does
// not yet provide them.
// ===========================================================================

// PrintConfig.hpp:340
// enum ExtruderType {
//     etDirectDrive = 0,
//     etBowden,
//     etMaxExtruderType = etBowden
// };
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(i32)]
pub enum ExtruderType {
    EtDirectDrive = 0,
    EtBowden = 1,
}
// PrintConfig.hpp:343  etMaxExtruderType = etBowden
pub const ET_MAX_EXTRUDER_TYPE: i32 = ExtruderType::EtBowden as i32;

impl ExtruderType {
    // Faithful equivalent of `ExtruderType(int)` cast in C++.
    pub fn from_i32(v: i32) -> ExtruderType {
        match v {
            0 => ExtruderType::EtDirectDrive,
            _ => ExtruderType::EtBowden,
        }
    }
}

// PrintConfig.hpp:346
// enum NozzleVolumeType {
//     nvtStandard = 0,
//     nvtHighFlow,
//     nvtHybrid,
//     nvtTPUHighFlow,
//     nvtMaxNozzleVolumeType = nvtTPUHighFlow
// };
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(i32)]
pub enum NozzleVolumeType {
    NvtStandard = 0,
    NvtHighFlow = 1,
    NvtHybrid = 2,
    NvtTPUHighFlow = 3,
}
// PrintConfig.hpp:351  nvtMaxNozzleVolumeType = nvtTPUHighFlow
pub const NVT_MAX_NOZZLE_VOLUME_TYPE: i32 = NozzleVolumeType::NvtTPUHighFlow as i32;

impl NozzleVolumeType {
    // Faithful equivalent of `NozzleVolumeType(int)` cast in C++.
    pub fn from_i32(v: i32) -> NozzleVolumeType {
        match v {
            0 => NozzleVolumeType::NvtStandard,
            1 => NozzleVolumeType::NvtHighFlow,
            2 => NozzleVolumeType::NvtHybrid,
            _ => NozzleVolumeType::NvtTPUHighFlow,
        }
    }
}

// PrintConfig.cpp enum key-name tables, indexed by enum value.
// (s_keys_names_ExtruderType / s_keys_names_NozzleVolumeType)
const S_KEYS_NAMES_EXTRUDER_TYPE: [&str; 2] = ["DirectDrive", "Bowden"];
const S_KEYS_NAMES_NOZZLE_VOLUME_TYPE: [&str; 4] =
    ["Standard", "HighFlow", "Hybrid", "TPUHighFlow"];

// PrintConfig.cpp:528
// std::string get_extruder_variant_string(ExtruderType extruder_type, NozzleVolumeType nozzle_volume_type)
pub fn get_extruder_variant_string(
    extruder_type: ExtruderType,
    nozzle_volume_type: NozzleVolumeType,
) -> String {
    // PrintConfig.cpp:530
    let mut variant_string = String::new();

    // PrintConfig.cpp:532
    if (extruder_type as i32) > ET_MAX_EXTRUDER_TYPE {
        // PrintConfig.cpp:533 (logging) — unsupported ExtruderType
        // extruder_type = etDirectDrive;
        return variant_string;
    }
    // PrintConfig.cpp:537  auto nozzle_volume_types = get_valid_nozzle_volume_type();
    // PrintConfig.cpp:538  if (nozzle_volume_types.count(nozzle_volume_type) == 0)
    if (nozzle_volume_type as i32) > NVT_MAX_NOZZLE_VOLUME_TYPE {
        // PrintConfig.cpp:539 (logging) — unsupported NozzleVolumeType
        return variant_string;
    }
    // PrintConfig.cpp:543-545
    variant_string = S_KEYS_NAMES_EXTRUDER_TYPE[extruder_type as usize].to_string();
    variant_string += " ";
    variant_string += S_KEYS_NAMES_NOZZLE_VOLUME_TYPE[nozzle_volume_type as usize];
    // PrintConfig.cpp:546
    variant_string
}

// PrintConfig.cpp:549
// int get_config_index_base(NozzleVolumeType volume_type, ExtruderType extruder_type,
//     int variant_id_1based, const std::vector<std::string>& variant_list,
//     const std::vector<int>& variant_ids_1based)
pub fn get_config_index_base(
    volume_type: NozzleVolumeType,
    extruder_type: ExtruderType,
    variant_id_1based: i32,
    variant_list: &[String],
    variant_ids_1based: &[i32],
) -> i32 {
    // PrintConfig.cpp:551  assert(variant_list.size() == variant_ids_1based.size());
    debug_assert!(variant_list.len() == variant_ids_1based.len());
    // PrintConfig.cpp:552
    let extruder_variant = get_extruder_variant_string(extruder_type, volume_type);
    // PrintConfig.cpp:553
    for index in 0..variant_list.len() as i32 {
        // PrintConfig.cpp:554
        if extruder_variant == variant_list[index as usize]
            && variant_ids_1based[index as usize] == variant_id_1based
        {
            return index;
        }
    }
    // PrintConfig.cpp:556-558 (logging) — could not find parameter
    // PrintConfig.cpp:559
    0
}

// PrintConfig.cpp:92
// size_t get_extruder_index(const GCodeConfig& config, unsigned int filament_id)
pub fn get_extruder_index(config: &GCodeConfig, filament_id: u32) -> usize {
    // PrintConfig.cpp:94
    if (filament_id as usize) < config.filament_map.size() {
        // PrintConfig.cpp:95
        return (config.filament_map.get_at(filament_id as usize) - 1) as usize;
    }
    // PrintConfig.cpp:97
    0
}

// PrintConfig.cpp:100
// size_t get_filament_config_idx(const GCodeConfig &config, unsigned int filament_id)
pub fn get_filament_config_idx(config: &GCodeConfig, filament_id: u32) -> usize {
    // PrintConfig.cpp:102
    let volume_type =
        NozzleVolumeType::from_i32(config.filament_volume_map.get_at(filament_id as usize));
    // PrintConfig.cpp:103
    let extruder_type = ExtruderType::from_i32(
        config
            .extruder_type
            .get_at(get_extruder_index(config, filament_id)),
    );
    // PrintConfig.cpp:104
    let filament_variant_list = &config.filament_extruder_variant.values;
    // PrintConfig.cpp:105
    let filament_self_idx = &config.filament_self_index.values;
    // PrintConfig.cpp:106
    get_config_index_base(
        volume_type,
        extruder_type,
        (filament_id + 1) as i32,
        filament_variant_list,
        filament_self_idx,
    ) as usize
}

// PrintConfig.cpp:109
// size_t get_process_config_idx(const GCodeConfig& config, unsigned int filament_id)
pub fn get_process_config_idx(config: &GCodeConfig, filament_id: u32) -> usize {
    // PrintConfig.cpp:111
    let volume_type =
        NozzleVolumeType::from_i32(config.filament_volume_map.get_at(filament_id as usize));
    // PrintConfig.cpp:112
    let extruder_id = get_extruder_index(config, filament_id) as i32;
    // PrintConfig.cpp:113
    let extruder_type =
        ExtruderType::from_i32(config.extruder_type.get_at(extruder_id as usize));
    // PrintConfig.cpp:114
    let print_extruder_id = &config.printer_extruder_id.values;
    // PrintConfig.cpp:115
    let variant_list = &config.printer_extruder_variant.values;
    // PrintConfig.cpp:116
    get_config_index_base(
        volume_type,
        extruder_type,
        extruder_id + 1,
        variant_list,
        print_extruder_id,
    ) as usize
}

// ===========================================================================
// Module-local faithful model of the subset of `GCodeConfig` that `Extruder`
// reads, using `ConfigOptionVector`-faithful `get_at` semantics
// (Config.hpp:681). Replace with the real `GCodeConfig` once it is ported.
// ===========================================================================

// Faithful port of `ConfigOptionVector<T>::get_at` (Config.hpp:681-685):
//   assert(! this->values.empty());
//   return (i < this->values.size()) ? this->values[i] : this->values.front();
#[derive(Debug, Clone, Default)]
pub struct ConfigOptionVector<T: Clone> {
    pub values: Vec<T>,
}

impl<T: Clone> ConfigOptionVector<T> {
    pub fn new(values: Vec<T>) -> Self {
        ConfigOptionVector { values }
    }
    // Config.hpp:681
    pub fn get_at(&self, i: usize) -> T {
        debug_assert!(!self.values.is_empty());
        if i < self.values.len() {
            self.values[i].clone()
        } else {
            self.values[0].clone()
        }
    }
    // ConfigOptionVector::size()
    pub fn size(&self) -> usize {
        self.values.len()
    }
}

/// Subset of `GCodeConfig` read by `Extruder`. Field option-types match the
/// BambuStudio definitions (PrintConfig.hpp:1180-1310):
///   filament_map / filament_nozzle_map / filament_volume_map / filament_self_index : ConfigOptionInts
///   extruder_type : ConfigOptionEnumsGeneric (ints)
///   nozzle_volume_type : ConfigOptionEnumsGeneric (ints)
///   filament_extruder_variant : ConfigOptionStrings
///   filament_diameter / filament_density / filament_cost : ConfigOptionFloats
///   filament_flow_ratio / retract_before_wipe / retraction_length / z_hop /
///   retraction_speed / deretraction_speed / retract_restart_extra /
///   retract_length_toolchange / retract_restart_extra_toolchange : Nullable floats
#[derive(Debug, Clone, Default)]
pub struct GCodeConfig {
    pub use_relative_e_distances: bool,
    pub filament_map: ConfigOptionVector<i32>,
    pub filament_nozzle_map: ConfigOptionVector<i32>,
    pub filament_volume_map: ConfigOptionVector<i32>,
    pub filament_self_index: ConfigOptionVector<i32>,
    pub nozzle_volume_type: ConfigOptionVector<i32>,
    pub extruder_type: ConfigOptionVector<i32>,
    pub filament_extruder_variant: ConfigOptionVector<String>,
    pub filament_diameter: ConfigOptionVector<f64>,
    pub filament_density: ConfigOptionVector<f64>,
    pub filament_cost: ConfigOptionVector<f64>,
    pub filament_flow_ratio: ConfigOptionVector<f64>,
    pub retract_before_wipe: ConfigOptionVector<f64>,
    pub retraction_length: ConfigOptionVector<f64>,
    pub z_hop: ConfigOptionVector<f64>,
    pub retraction_speed: ConfigOptionVector<f64>,
    pub deretraction_speed: ConfigOptionVector<f64>,
    pub retract_restart_extra: ConfigOptionVector<f64>,
    pub retract_length_toolchange: ConfigOptionVector<f64>,
    pub retract_restart_extra_toolchange: ConfigOptionVector<f64>,
    // Additional fields read by GCodeWriter (PrintConfig.hpp). These complete the
    // subset of GCodeConfig that the writer requires.
    pub gcode_flavor: crate::print_config::GCodeFlavor,
    pub travel_speed: ConfigOptionVector<f64>,
    pub travel_speed_z: ConfigOptionVector<f64>,
    pub retract_lift_above: ConfigOptionVector<f64>,
    pub retract_lift_below: ConfigOptionVector<f64>,
    // ConfigOptionFloat `.value` scalars.
    pub prime_tower_lift_height: f64,
    pub prime_tower_lift_speed: f64,
    pub use_firmware_retraction: bool,
    pub accel_to_decel_enable: bool,
    pub accel_to_decel_factor: f64,
    // get_process_config_idx() inputs (PrintConfig.cpp:114-115).
    pub printer_extruder_id: ConfigOptionVector<i32>,
    pub printer_extruder_variant: ConfigOptionVector<String>,
}

// ===========================================================================
// Static shared E and retraction data for single-extruder multi-material
// machines.
// Extruder.cpp:5-6
//   std::vector<double> Extruder::m_share_E = {0.,0.};
//   std::vector<double> Extruder::m_share_retracted = {0.,0.};
// The C++ uses class statics; mirror them with module-level mutable statics
// guarded by the same access patterns as the original.
// ===========================================================================
static mut M_SHARE_E: [f64; 2] = [0.0, 0.0];
static mut M_SHARE_RETRACTED: [f64; 2] = [0.0, 0.0];

// Extruder.hpp:13
// class Extruder
pub struct Extruder {
    // Extruder.hpp:73-74  Reference to GCodeWriter's GCodeConfig instance.
    m_config: *const GCodeConfig,
    // Extruder.hpp:75-76  Print-wide global ID of this extruder.
    m_id: u32,
    // Extruder.hpp:77  Current state of the extruder axis (reset if relative E).
    m_e: f64,
    // Extruder.hpp:79  Extruder tachometer, used for extruded_volume()/used_filament().
    m_absolute_e: f64,
    // Extruder.hpp:81  Current positive amount of retraction.
    m_retracted: f64,
    // Extruder.hpp:83  Extra amount of priming on deretraction.
    m_restart_extra: f64,
    // Extruder.hpp:85
    m_e_per_mm3: f64,
    // Extruder.hpp:88  BBS: shared E / retraction for single-extruder multi-material.
    m_share_extruder: bool,
}

impl Extruder {
    // Extruder.cpp:11
    // Extruder::Extruder(unsigned int id, GCodeConfig *config, bool share_extruder)
    pub fn new(id: u32, config: *const GCodeConfig, share_extruder: bool) -> Extruder {
        // Extruder.cpp:12-14 member initializer list:
        //   m_id(id), m_config(config), m_share_extruder(share_extruder)
        let mut e = Extruder {
            m_config: config,
            m_id: id,
            m_e: 0.0,
            m_absolute_e: 0.0,
            m_retracted: 0.0,
            m_restart_extra: 0.0,
            m_e_per_mm3: 0.0,
            m_share_extruder: share_extruder,
        };
        // Extruder.cpp:16
        e.reset();
        // Extruder.cpp:17-19  cache values that are going to be called often
        e.m_e_per_mm3 = e.filament_flow_ratio();
        e.m_e_per_mm3 /= e.filament_crossection();
        e
    }

    // Helper: deref m_config (mirrors C++ `m_config->`/`*m_config`).
    #[inline]
    fn config(&self) -> &GCodeConfig {
        // assert(m_config);
        debug_assert!(!self.m_config.is_null());
        unsafe { &*self.m_config }
    }

    // Extruder.hpp:19
    // void reset()
    pub fn reset(&mut self) {
        // Extruder.hpp:20-21  BBS
        if self.m_share_extruder {
            // Extruder.hpp:22-23
            unsafe {
                M_SHARE_E = [0.0, 0.0];
                M_SHARE_RETRACTED = [0.0, 0.0];
            }
        } else {
            // Extruder.hpp:25-26
            self.m_e = 0.0;
            self.m_retracted = 0.0;
        }
        // Extruder.hpp:28-29
        self.m_restart_extra = 0.0;
        self.m_absolute_e = 0.0;
    }

    // Extruder.hpp:32
    // unsigned int id() const { return m_id; }
    pub fn id(&self) -> u32 {
        self.m_id
    }

    // Extruder.cpp:22
    // unsigned int Extruder::extruder_id() const
    pub fn extruder_id(&self) -> u32 {
        // Extruder.cpp:24  assert(m_config);
        let config = self.config();
        // Extruder.cpp:25
        if (self.m_id as usize) < config.filament_map.size() {
            // Extruder.cpp:26
            return (config.filament_map.get_at(self.m_id as usize) - 1) as u32;
        }
        // Extruder.cpp:28
        0
    }

    // Extruder.cpp:31
    // unsigned int Extruder::nozzle_id() const
    pub fn nozzle_id(&self) -> u32 {
        // Extruder.cpp:33  assert(m_config);
        let config = self.config();
        // Extruder.cpp:34
        if (self.m_id as usize) < config.filament_nozzle_map.size() {
            // Extruder.cpp:35
            return (config.filament_nozzle_map.get_at(self.m_id as usize) - 1) as u32;
        }
        // Extruder.cpp:37
        0
    }

    // Extruder.cpp:40
    // NozzleVolumeType Extruder::volume_type() const
    pub fn volume_type(&self) -> NozzleVolumeType {
        // Extruder.cpp:42  assert(m_config);
        let config = self.config();
        // Extruder.cpp:43
        if (self.m_id as usize) < config.nozzle_volume_type.size() {
            // Extruder.cpp:44
            return NozzleVolumeType::from_i32(config.nozzle_volume_type.get_at(self.m_id as usize));
        }
        // Extruder.cpp:47
        NozzleVolumeType::NvtStandard
    }

    // Extruder.cpp:50
    // ExtruderType Extruder::extruder_type() const
    pub fn extruder_type(&self) -> ExtruderType {
        // Extruder.cpp:52  assert(m_config);
        let config = self.config();
        // Extruder.cpp:53
        let ext_id = self.extruder_id();
        // Extruder.cpp:54
        if (ext_id as usize) < config.extruder_type.size() {
            // Extruder.cpp:55
            return ExtruderType::from_i32(config.extruder_type.get_at(ext_id as usize));
        }
        // Extruder.cpp:57
        ExtruderType::EtDirectDrive
    }

    // Extruder.cpp:61
    // double Extruder::extrude(double dE)
    pub fn extrude(&mut self, d_e: f64) -> f64 {
        let config = self.config();
        // Extruder.cpp:63  BBS
        if self.m_share_extruder {
            let extruder_id = self.extruder_id() as usize;
            // Extruder.cpp:65-66
            if config.use_relative_e_distances {
                unsafe {
                    M_SHARE_E[extruder_id] = 0.0;
                }
            }
            // Extruder.cpp:67
            unsafe {
                M_SHARE_E[extruder_id] += d_e;
            }
            // Extruder.cpp:68
            self.m_absolute_e += d_e;
            // Extruder.cpp:69-70
            if d_e < 0.0 {
                unsafe {
                    M_SHARE_RETRACTED[extruder_id] -= d_e;
                }
            }
        } else {
            // Extruder.cpp:72-74  in case of relative E distances we always reset to 0 before any output
            if config.use_relative_e_distances {
                self.m_e = 0.0;
            }
            // Extruder.cpp:75
            self.m_e += d_e;
            // Extruder.cpp:76
            self.m_absolute_e += d_e;
            // Extruder.cpp:77-78
            if d_e < 0.0 {
                self.m_retracted -= d_e;
            }
        }
        // Extruder.cpp:80
        d_e
    }

    // Extruder.cpp:83-89 (comment)
    /* This method makes sure the extruder is retracted by the specified amount
       of filament and returns the amount of filament retracted.
       If the extruder is already retracted by the same or a greater amount,
       this method is a no-op.
       The restart_extra argument sets the extra length to be used for
       unretraction. If we're actually performing a retraction, any restart_extra
       value supplied will overwrite the previous one if any. */
    // Extruder.cpp:90
    // double Extruder::retract(double length, double restart_extra)
    pub fn retract(&mut self, length: f64, restart_extra: f64) -> f64 {
        let config = self.config();
        // Extruder.cpp:92  BBS
        if self.m_share_extruder {
            let extruder_id = self.extruder_id() as usize;
            // Extruder.cpp:94-95
            if config.use_relative_e_distances {
                unsafe {
                    M_SHARE_E[extruder_id] = 0.0;
                }
            }
            // Extruder.cpp:96
            let to_retract = (length - unsafe { M_SHARE_RETRACTED[extruder_id] }).max(0.0);
            // Extruder.cpp:97
            self.m_restart_extra = restart_extra;
            // Extruder.cpp:98
            if to_retract > 0.0 {
                // Extruder.cpp:99
                unsafe {
                    M_SHARE_E[extruder_id] -= to_retract;
                }
                // Extruder.cpp:100
                self.m_absolute_e -= to_retract;
                // Extruder.cpp:101
                unsafe {
                    M_SHARE_RETRACTED[extruder_id] += to_retract;
                }
            }
            // Extruder.cpp:103
            to_retract
        } else {
            // Extruder.cpp:106-107  in case of relative E distances we always reset to 0 before any output
            if config.use_relative_e_distances {
                self.m_e = 0.0;
            }
            // Extruder.cpp:108
            let to_retract = (length - self.m_retracted).max(0.0);
            // Extruder.cpp:109
            self.m_restart_extra = restart_extra;
            // Extruder.cpp:110
            if to_retract > 0.0 {
                // Extruder.cpp:111
                self.m_e -= to_retract;
                // Extruder.cpp:112
                self.m_absolute_e -= to_retract;
                // Extruder.cpp:113
                self.m_retracted += to_retract;
            }
            // Extruder.cpp:115
            to_retract
        }
    }

    // Extruder.cpp:119
    // double Extruder::unretract()
    pub fn unretract(&mut self) -> f64 {
        // Extruder.cpp:121  BBS
        if self.m_share_extruder {
            let extruder_id = self.extruder_id() as usize;
            // Extruder.cpp:123
            let d_e = unsafe { M_SHARE_RETRACTED[extruder_id] } + self.m_restart_extra;
            // Extruder.cpp:124
            self.extrude(d_e);
            // Extruder.cpp:125
            unsafe {
                M_SHARE_RETRACTED[extruder_id] = 0.0;
            }
            // Extruder.cpp:126
            self.m_restart_extra = 0.0;
            // Extruder.cpp:127
            d_e
        } else {
            // Extruder.cpp:129
            let d_e = self.m_retracted + self.m_restart_extra;
            // Extruder.cpp:130
            self.extrude(d_e);
            // Extruder.cpp:131
            self.m_retracted = 0.0;
            // Extruder.cpp:132
            self.m_restart_extra = 0.0;
            // Extruder.cpp:133
            d_e
        }
    }

    // Extruder.hpp:42
    // double E() const { return m_share_extruder ? m_share_E[extruder_id()] : m_E; }
    pub fn e(&self) -> f64 {
        if self.m_share_extruder {
            unsafe { M_SHARE_E[self.extruder_id() as usize] }
        } else {
            self.m_e
        }
    }

    // Extruder.hpp:43
    // void reset_E() { m_E = 0.; m_share_E[extruder_id()] = 0.; }
    pub fn reset_e(&mut self) {
        self.m_e = 0.0;
        unsafe {
            M_SHARE_E[self.extruder_id() as usize] = 0.0;
        }
    }

    // Extruder.hpp:44
    // double e_per_mm(double mm3_per_mm) const { return mm3_per_mm * m_e_per_mm3; }
    pub fn e_per_mm(&self, mm3_per_mm: f64) -> f64 {
        mm3_per_mm * self.m_e_per_mm3
    }

    // Extruder.hpp:45
    // double e_per_mm3() const { return m_e_per_mm3; }
    pub fn e_per_mm3(&self) -> f64 {
        self.m_e_per_mm3
    }

    // Extruder.cpp:137-138  Used filament volume in mm^3.
    // double Extruder::extruded_volume() const
    pub fn extruded_volume(&self) -> f64 {
        // Extruder.cpp:140  BBS
        if self.m_share_extruder {
            // Extruder.cpp:142-143  FIXME: need to count m_retracted for share extruder machine
            self.used_filament() * self.filament_crossection()
        } else {
            // Extruder.cpp:145
            self.used_filament() * self.filament_crossection()
        }
    }

    // Extruder.cpp:149-150  Used filament length in mm.
    // double Extruder::used_filament() const
    pub fn used_filament(&self) -> f64 {
        // Extruder.cpp:152  BBS
        if self.m_share_extruder {
            // Extruder.cpp:154-155  FIXME: need to count retracted length for share-extruder machine
            self.m_absolute_e
        } else {
            // Extruder.cpp:157
            self.m_absolute_e + self.m_retracted
        }
    }

    // Extruder.cpp:161
    // double Extruder::filament_diameter() const
    pub fn filament_diameter(&self) -> f64 {
        // Extruder.cpp:163
        self.config().filament_diameter.get_at(self.m_id as usize)
    }

    // Extruder.hpp:52
    // double filament_crossection() const { return this->filament_diameter() * this->filament_diameter() * 0.25 * PI; }
    pub fn filament_crossection(&self) -> f64 {
        self.filament_diameter() * self.filament_diameter() * 0.25 * PI
    }

    // Extruder.cpp:166
    // double Extruder::filament_density() const
    pub fn filament_density(&self) -> f64 {
        // Extruder.cpp:168
        self.config().filament_density.get_at(self.m_id as usize)
    }

    // Extruder.cpp:171
    // double Extruder::filament_cost() const
    pub fn filament_cost(&self) -> f64 {
        // Extruder.cpp:173
        self.config().filament_cost.get_at(self.m_id as usize)
    }

    // Extruder.cpp:176
    // double Extruder::filament_flow_ratio() const
    pub fn filament_flow_ratio(&self) -> f64 {
        // Extruder.cpp:178
        let config = self.config();
        config
            .filament_flow_ratio
            .get_at(get_filament_config_idx(config, self.m_id))
    }

    // Extruder.cpp:181-182  Return a "retract_before_wipe" percentage as a factor clamped to <0, 1>
    // double Extruder::retract_before_wipe() const
    pub fn retract_before_wipe(&self) -> f64 {
        // Extruder.cpp:184
        let config = self.config();
        (config
            .retract_before_wipe
            .get_at(get_filament_config_idx(config, self.m_id))
            * 0.01)
            .max(0.0)
            .min(1.0)
    }

    // Extruder.cpp:187
    // double Extruder::retraction_length() const
    pub fn retraction_length(&self) -> f64 {
        // Extruder.cpp:189
        let config = self.config();
        config
            .retraction_length
            .get_at(get_filament_config_idx(config, self.m_id))
    }

    // Extruder.cpp:192
    // double Extruder::retract_lift() const
    pub fn retract_lift(&self) -> f64 {
        // Extruder.cpp:194
        let config = self.config();
        config.z_hop.get_at(get_filament_config_idx(config, self.m_id))
    }

    // Extruder.cpp:197
    // int Extruder::retract_speed() const
    pub fn retract_speed(&self) -> i32 {
        // Extruder.cpp:199  int(floor(retraction_speed.get_at(...)+0.5))
        let config = self.config();
        (config
            .retraction_speed
            .get_at(get_filament_config_idx(config, self.m_id))
            + 0.5)
            .floor() as i32
    }

    // Extruder.cpp:202
    // int Extruder::deretract_speed() const
    pub fn deretract_speed(&self) -> i32 {
        // Extruder.cpp:204  int speed = int(floor(deretraction_speed.get_at(...) + 0.5));
        let config = self.config();
        let speed = (config
            .deretraction_speed
            .get_at(get_filament_config_idx(config, self.m_id))
            + 0.5)
            .floor() as i32;
        // Extruder.cpp:205  return (speed > 0) ? speed : this->retract_speed();
        if speed > 0 {
            speed
        } else {
            self.retract_speed()
        }
    }

    // Extruder.cpp:208
    // double Extruder::retract_restart_extra() const
    pub fn retract_restart_extra(&self) -> f64 {
        // Extruder.cpp:210
        let config = self.config();
        config
            .retract_restart_extra
            .get_at(get_filament_config_idx(config, self.m_id))
    }

    // Extruder.cpp:213
    // double Extruder::retract_length_toolchange() const
    pub fn retract_length_toolchange(&self) -> f64 {
        // Extruder.cpp:215
        self.config()
            .retract_length_toolchange
            .get_at(self.extruder_id() as usize)
    }

    // Extruder.cpp:218
    // double Extruder::retract_restart_extra_toolchange() const
    pub fn retract_restart_extra_toolchange(&self) -> f64 {
        // Extruder.cpp:220
        self.config()
            .retract_restart_extra_toolchange
            .get_at(self.extruder_id() as usize)
    }

    // Extruder.hpp:65
    // bool is_share_extruder() const { return m_share_extruder; }
    pub fn is_share_extruder(&self) -> bool {
        self.m_share_extruder
    }

    // Extruder.hpp:66
    // double get_single_retracted_length() const { return m_retracted; }
    pub fn get_single_retracted_length(&self) -> f64 {
        self.m_retracted
    }

    // Extruder.hpp:67
    // double get_share_retracted_length() const { return m_share_retracted[extruder_id()]; }
    pub fn get_share_retracted_length(&self) -> f64 {
        unsafe { M_SHARE_RETRACTED[self.extruder_id() as usize] }
    }
}

// Extruder.hpp:94-97  Sort Extruder objects by the extruder id by default.
// inline bool operator==(const Extruder &e1, const Extruder &e2) { return e1.id() == e2.id(); }
// inline bool operator!=(const Extruder &e1, const Extruder &e2) { return e1.id() != e2.id(); }
// inline bool operator< (const Extruder &e1, const Extruder &e2) { return e1.id() <  e2.id(); }
// inline bool operator> (const Extruder &e1, const Extruder &e2) { return e1.id() >  e2.id(); }
impl PartialEq for Extruder {
    fn eq(&self, other: &Self) -> bool {
        self.id() == other.id()
    }
}
impl Eq for Extruder {}
impl PartialOrd for Extruder {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Extruder {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.id().cmp(&other.id())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Single-extruder config: one filament, absolute E.
    fn mock_config() -> GCodeConfig {
        GCodeConfig {
            use_relative_e_distances: false,
            filament_map: ConfigOptionVector::new(vec![1]),
            filament_nozzle_map: ConfigOptionVector::new(vec![1]),
            filament_volume_map: ConfigOptionVector::new(vec![0]),
            filament_self_index: ConfigOptionVector::new(vec![1]),
            nozzle_volume_type: ConfigOptionVector::new(vec![0]),
            extruder_type: ConfigOptionVector::new(vec![0]),
            filament_extruder_variant: ConfigOptionVector::new(vec![
                "DirectDrive Standard".to_string()
            ]),
            filament_diameter: ConfigOptionVector::new(vec![1.75]),
            filament_density: ConfigOptionVector::new(vec![1.24]),
            filament_cost: ConfigOptionVector::new(vec![20.0]),
            filament_flow_ratio: ConfigOptionVector::new(vec![1.0]),
            retract_before_wipe: ConfigOptionVector::new(vec![0.0]),
            retraction_length: ConfigOptionVector::new(vec![0.8]),
            z_hop: ConfigOptionVector::new(vec![0.0]),
            retraction_speed: ConfigOptionVector::new(vec![40.0]),
            deretraction_speed: ConfigOptionVector::new(vec![0.0]),
            retract_restart_extra: ConfigOptionVector::new(vec![0.0]),
            retract_length_toolchange: ConfigOptionVector::new(vec![10.0]),
            retract_restart_extra_toolchange: ConfigOptionVector::new(vec![0.0]),
        }
    }

    #[test]
    fn test_extruder_creation() {
        let config = mock_config();
        let extruder = Extruder::new(0, &config, false);
        assert_eq!(extruder.id(), 0);
        assert_eq!(extruder.e(), 0.0);
        assert!(!extruder.is_share_extruder());
    }

    #[test]
    fn test_extrude() {
        let config = mock_config();
        let mut extruder = Extruder::new(0, &config, false);
        let extruded = extruder.extrude(10.0);
        assert_eq!(extruded, 10.0);
        assert_eq!(extruder.e(), 10.0);
    }

    #[test]
    fn test_retract() {
        let config = mock_config();
        let mut extruder = Extruder::new(0, &config, false);
        extruder.extrude(10.0);
        let retracted = extruder.retract(2.0, 0.1);
        assert_eq!(retracted, 2.0);
    }

    #[test]
    fn test_unretract() {
        let config = mock_config();
        let mut extruder = Extruder::new(0, &config, false);
        extruder.extrude(10.0);
        extruder.retract(2.0, 0.1);
        let unretracted = extruder.unretract();
        assert_eq!(unretracted, 2.1); // 2.0 retracted + 0.1 extra
    }

    #[test]
    fn test_reset() {
        let config = mock_config();
        let mut extruder = Extruder::new(0, &config, false);
        extruder.extrude(10.0);
        extruder.retract(2.0, 0.1);
        extruder.reset();
        assert_eq!(extruder.e(), 0.0);
    }

    #[test]
    fn test_filament_crossection() {
        let config = mock_config();
        let extruder = Extruder::new(0, &config, false);
        let crossection = extruder.filament_crossection();
        let expected = 1.75 * 1.75 * 0.25 * PI;
        assert!((crossection - expected).abs() < 0.001);
    }

    #[test]
    fn test_extruder_ordering() {
        let config = mock_config();
        let e1 = Extruder::new(0, &config, false);
        let e2 = Extruder::new(1, &config, false);
        assert!(e1 < e2);
        assert_eq!(e1, e1);
        assert_ne!(e1, e2);
    }

    #[test]
    fn test_used_filament() {
        let config = mock_config();
        let mut extruder = Extruder::new(0, &config, false);
        extruder.extrude(10.0);
        extruder.retract(2.0, 0.0);
        // Used filament = m_absolute_E + m_retracted = 8.0 + 2.0 = 10.0
        assert_eq!(extruder.used_filament(), 10.0);
    }

    #[test]
    fn test_e_per_mm() {
        let config = mock_config();
        let extruder = Extruder::new(0, &config, false);
        let e = extruder.e_per_mm(0.5);
        assert!(e > 0.0);
    }

    #[test]
    fn test_retract_speed_rounding() {
        // retraction_speed 40.0 -> floor(40.0 + 0.5) = 40
        let config = mock_config();
        let extruder = Extruder::new(0, &config, false);
        assert_eq!(extruder.retract_speed(), 40);
        // deretraction_speed 0.0 -> floor(0.5) = 0 -> falls back to retract_speed()
        assert_eq!(extruder.deretract_speed(), 40);
    }
}
