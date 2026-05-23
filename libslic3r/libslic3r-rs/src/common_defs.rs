//! Common definitions and enumerations shared across the library
//!
//! Provides common type definitions consistent across the codebase,
//! particularly nozzle types for printer hardware specifications.
//!
//! C++ Reference: CommonDefs.hpp

/// Nozzle material type enumeration
/// CommonDefs.hpp:13-21
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum NozzleType {
    /// Undefined nozzle type
    /// CommonDefs.hpp:15
    Undefine = 0,

    /// Hardened steel nozzle (for abrasive filaments)
    /// CommonDefs.hpp:16
    HardenedSteel = 1,

    /// Stainless steel nozzle
    /// CommonDefs.hpp:17
    StainlessSteel = 2,

    /// Tungsten carbide nozzle (high durability)
    /// CommonDefs.hpp:18
    TungstenCarbide = 3,

    /// Brass nozzle (standard)
    /// CommonDefs.hpp:19
    Brass = 4,

    /// E3D nozzle type
    /// CommonDefs.hpp:20
    E3D = 5,
}

impl NozzleType {
    /// Total count of nozzle types
    /// CommonDefs.hpp:21
    pub const COUNT: usize = 6;

    /// Convert from integer value
    /// CommonDefs.hpp:13-21 (utility)
    pub fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(NozzleType::Undefine),
            1 => Some(NozzleType::HardenedSteel),
            2 => Some(NozzleType::StainlessSteel),
            3 => Some(NozzleType::TungstenCarbide),
            4 => Some(NozzleType::Brass),
            5 => Some(NozzleType::E3D),
            _ => None,
        }
    }

    /// Convert to integer value
    /// CommonDefs.hpp:13-21 (utility)
    pub fn to_u8(self) -> u8 {
        self as u8
    }

    /// Get human-readable name for nozzle type
    /// CommonDefs.hpp:13-21 (utility)
    pub fn name(self) -> &'static str {
        match self {
            NozzleType::Undefine => "Undefined",
            NozzleType::HardenedSteel => "Hardened Steel",
            NozzleType::StainlessSteel => "Stainless Steel",
            NozzleType::TungstenCarbide => "Tungsten Carbide",
            NozzleType::Brass => "Brass",
            NozzleType::E3D => "E3D",
        }
    }

    /// Check if nozzle type is defined
    /// CommonDefs.hpp:13-21 (utility)
    pub fn is_defined(self) -> bool {
        self != NozzleType::Undefine
    }
}

impl Default for NozzleType {
    fn default() -> Self {
        NozzleType::Undefine
    }
}

impl std::fmt::Display for NozzleType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.name())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nozzle_type_values() {
        assert_eq!(NozzleType::Undefine as u8, 0);
        assert_eq!(NozzleType::HardenedSteel as u8, 1);
        assert_eq!(NozzleType::Brass as u8, 4);
        assert_eq!(NozzleType::E3D as u8, 5);
    }

    #[test]
    fn test_nozzle_type_from_u8() {
        assert_eq!(NozzleType::from_u8(0), Some(NozzleType::Undefine));
        assert_eq!(NozzleType::from_u8(1), Some(NozzleType::HardenedSteel));
        assert_eq!(NozzleType::from_u8(5), Some(NozzleType::E3D));
        assert_eq!(NozzleType::from_u8(99), None);
    }

    #[test]
    fn test_nozzle_type_names() {
        assert_eq!(NozzleType::Brass.name(), "Brass");
        assert_eq!(NozzleType::HardenedSteel.name(), "Hardened Steel");
        assert_eq!(NozzleType::TungstenCarbide.name(), "Tungsten Carbide");
    }

    #[test]
    fn test_nozzle_type_is_defined() {
        assert!(!NozzleType::Undefine.is_defined());
        assert!(NozzleType::Brass.is_defined());
        assert!(NozzleType::HardenedSteel.is_defined());
    }

    #[test]
    fn test_nozzle_type_default() {
        let default = NozzleType::default();
        assert_eq!(default, NozzleType::Undefine);
    }

    #[test]
    fn test_nozzle_type_display() {
        assert_eq!(format!("{}", NozzleType::Brass), "Brass");
        assert_eq!(format!("{}", NozzleType::E3D), "E3D");
    }

    #[test]
    fn test_nozzle_type_count() {
        assert_eq!(NozzleType::COUNT, 6);
    }
}
