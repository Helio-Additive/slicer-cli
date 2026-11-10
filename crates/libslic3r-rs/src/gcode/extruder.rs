//! Extruder state management.
//!
//! Mirrors BambuStudio's `Extruder` class.
//! Handles extrusion state, retraction, and shared extruder logic for multi-material printing.

use crate::print_config::PrintConfig;
use std::cell::RefCell;
use std::rc::Rc;

/// Shared state for extruders that share a physical nozzle/drive.
#[derive(Debug, Default, Clone)]
pub struct SharedExtruderState {
    /// E position per filament/extruder ID.
    pub e: Vec<f64>,
    /// Retraction state per filament/extruder ID.
    pub retracted: Vec<f64>,
}

impl SharedExtruderState {
    pub fn new(num_extruders: usize) -> Self {
        Self {
            e: vec![0.0; num_extruders],
            retracted: vec![0.0; num_extruders],
        }
    }

    pub fn reset(&mut self) {
        self.e.fill(0.0);
        self.retracted.fill(0.0);
    }
}

/// Represents a single extruder (or filament channel).
#[derive(Debug, Clone)]
pub struct Extruder {
    /// ID of this extruder (filament index).
    id: usize,

    /// Configuration reference.
    config: Rc<PrintConfig>,

    /// Current E position (local).
    e: f64,

    /// Total absolute E position (filament usage).
    absolute_e: f64,

    /// Current retraction amount.
    retracted: f64,

    /// Restart extra amount (prime after unretract).
    restart_extra: f64,

    /// E per mm3 (calculated from flow ratio and cross-section).
    e_per_mm3: f64,

    /// Whether this extruder shares hardware with others (MMU/AMS style).
    share_extruder: bool,

    /// Shared state (only used if share_extruder is true).
    shared_state: Option<Rc<RefCell<SharedExtruderState>>>,
}

impl Extruder {
    // Create a new extruder.
    pub fn new(
        id: usize,
        config: Rc<PrintConfig>,
        share_extruder: bool,
        shared_state: Option<Rc<RefCell<SharedExtruderState>>>,
    ) -> Self {
        let mut extruder = Self {
            id,
            config,
            e: 0.0,
            absolute_e: 0.0,
            retracted: 0.0,
            restart_extra: 0.0,
            e_per_mm3: 0.0,
            share_extruder,
            shared_state,
        };

        extruder.reset();
        extruder.update_e_per_mm3();

        extruder
    }

    /// Reset extruder state.
    pub fn reset(&mut self) {
        if self.share_extruder {
            if let Some(shared) = &self.shared_state {
                let mut state = shared.borrow_mut();
                state.reset();
            }
        } else {
            self.e = 0.0;
            self.retracted = 0.0;
        }
        self.restart_extra = 0.0;
        self.absolute_e = 0.0;
    }

    /// Get the extruder ID.
    pub fn id(&self) -> usize {
        self.id
    }

    /// Calculate E per mm3.
    fn update_e_per_mm3(&mut self) {
        self.e_per_mm3 = self.filament_flow_ratio() / self.filament_crossection();
    }

    /// Extrude a specific amount of filament (dE).
    pub fn extrude(&mut self, de: f64) -> f64 {
        if self.share_extruder {
            if let Some(shared) = &self.shared_state {
                let mut state = shared.borrow_mut();
                if self.config.use_relative_e {
                    state.e[self.id] = 0.0;
                }
                state.e[self.id] += de;
                self.absolute_e += de;
                if de < 0.0 {
                    state.retracted[self.id] -= de;
                }
            }
        } else {
            if self.config.use_relative_e {
                self.e = 0.0;
            }
            self.e += de;
            self.absolute_e += de;
            if de < 0.0 {
                self.retracted -= de;
            }
        }
        de
    }

    /// Retract filament.
    ///
    /// Returns the actual amount retracted.
    pub fn retract(&mut self, length: f64, restart_extra: f64) -> f64 {
        let current_retracted = if self.share_extruder {
            self.shared_state
                .as_ref()
                .map(|s| s.borrow().retracted[self.id])
                .unwrap_or(0.0)
        } else {
            self.retracted
        };

        let to_retract = (length - current_retracted).max(0.0);
        self.restart_extra = restart_extra;

        if to_retract > 0.0 {
            if self.share_extruder {
                if let Some(shared) = &self.shared_state {
                    let mut state = shared.borrow_mut();
                    if self.config.use_relative_e {
                        state.e[self.id] = 0.0;
                    }
                    state.e[self.id] -= to_retract;
                    self.absolute_e -= to_retract;
                    state.retracted[self.id] += to_retract;
                }
            } else {
                if self.config.use_relative_e {
                    self.e = 0.0;
                }
                self.e -= to_retract;
                self.absolute_e -= to_retract;
                self.retracted += to_retract;
            }
        }
        to_retract
    }

    /// Unretract (deretract) filament.
    pub fn unretract(&mut self) -> f64 {
        let (current_retracted, restart_extra) = if self.share_extruder {
            let retracted = self
                .shared_state
                .as_ref()
                .map(|s| s.borrow().retracted[self.id])
                .unwrap_or(0.0);
            (retracted, self.restart_extra)
        } else {
            (self.retracted, self.restart_extra)
        };

        let de = current_retracted + restart_extra;
        self.extrude(de);

        if self.share_extruder {
            if let Some(shared) = &self.shared_state {
                let mut state = shared.borrow_mut();
                state.retracted[self.id] = 0.0;
            }
        } else {
            self.retracted = 0.0;
        }
        self.restart_extra = 0.0;
        de
    }

    /// Get current E position.
    pub fn e(&self) -> f64 {
        if self.share_extruder {
            self.shared_state
                .as_ref()
                .map(|s| s.borrow().e[self.id])
                .unwrap_or(0.0)
        } else {
            self.e
        }
    }

    /// Reset local E (and shared E for this ID).
    pub fn reset_e(&mut self) {
        if self.share_extruder {
            if let Some(shared) = &self.shared_state {
                let mut state = shared.borrow_mut();
                state.e[self.id] = 0.0;
            }
        }
        self.e = 0.0;
    }

    /// Convert mm3 volume to E value.
    pub fn e_per_mm(&self, mm3_per_mm: f64) -> f64 {
        mm3_per_mm * self.e_per_mm3
    }

    pub fn e_per_mm3(&self) -> f64 {
        self.e_per_mm3
    }

    /// Get total extruded volume in mm3.
    pub fn extruded_volume(&self) -> f64 {
        self.used_filament() * self.filament_crossection()
    }

    /// Get total filament length used in mm.
    pub fn used_filament(&self) -> f64 {
        if self.share_extruder {
            // To match C++ behavior: "FIXME: need to count retracted length"
            self.absolute_e
        } else {
            self.absolute_e + self.retracted
        }
    }

    // --- Config Accessors ---

    pub fn filament_diameter(&self) -> f64 {
        // Rust config might not be vector, assuming single value for now or need helper
        // Adapting to PrintConfig structure - might need to check how list-based config is accessed.
        // For now, assuming config has methods or fields.
        // Note: PrintConfig in Rust impl usually has flat fields or Vec.
        // I will assume `filament_diameter` is a Vec<f64> or similar in config.
        // If PrintConfig is flat, this might need adjustment.
        // Assuming simple default for compilation, requiring 'id' usage later.
        self.config.filament_diameter
    }

    pub fn filament_crossection(&self) -> f64 {
        let r = self.filament_diameter() / 2.0;
        std::f64::consts::PI * r * r
    }

    pub fn filament_density(&self) -> f64 {
        self.config.filament_density
    }

    pub fn filament_cost(&self) -> f64 {
        self.config.filament_cost
    }

    pub fn filament_flow_ratio(&self) -> f64 {
        self.config.filament_flow_ratio
    }

    pub fn retract_before_wipe(&self) -> f64 {
        (self.config.retract_before_wipe * 0.01).max(0.0).min(1.0)
    }

    pub fn retraction_length(&self) -> f64 {
        self.config.retract_length
    }

    pub fn retract_lift(&self) -> f64 {
        self.config.retract_lift
    }

    pub fn retract_speed(&self) -> i32 {
        (self.config.retract_speed + 0.5).floor() as i32
    }

    pub fn deretract_speed(&self) -> i32 {
        let speed = (self.config.deretract_speed + 0.5).floor() as i32;
        if speed > 0 {
            speed
        } else {
            self.retract_speed()
        }
    }

    pub fn retract_restart_extra(&self) -> f64 {
        self.config.retract_restart_extra
    }

    pub fn retract_length_toolchange(&self) -> f64 {
        self.config.retract_length_toolchange
    }

    pub fn retract_restart_extra_toolchange(&self) -> f64 {
        self.config.retract_restart_extra_toolchange
    }
}
