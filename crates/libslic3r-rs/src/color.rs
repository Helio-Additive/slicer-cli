//! Color utilities for the slicer.
//!
//! Mirrors BambuStudio's `Color` namespace and classes.
//! Provides RGB and RGBA color representations and utilities.

use std::fmt;

/// Derive traits for ColorRGB struct
/// Color.hpp:10-12
#[derive(Clone, Copy, PartialEq)]
/// RGB color with floating point components [0.0, 1.0]
/// Color.hpp:14-25
pub struct ColorRGB {
    data: [f32; 3],
}

/// Default trait implementation for ColorRGB
/// Color.cpp:10-16
impl Default for ColorRGB {
    // Create default ColorRGB with white color
    // Color.cpp:11-14
    fn default() -> Self {
        // Initialize with white (1.0, 1.0, 1.0)
        // Color.cpp:12-13
        Self {
            data: [1.0, 1.0, 1.0],
        }
    }
}

/// Implementation of ColorRGB methods
/// Color.cpp:19-85
impl ColorRGB {
    // Black color constant
    // Color.hpp:28
    pub const BLACK: Self = Self::new(0.0, 0.0, 0.0);
    // White color constant
    // Color.hpp:29
    pub const WHITE: Self = Self::new(1.0, 1.0, 1.0);
    // Red color constant
    // Color.hpp:30
    pub const RED: Self = Self::new(1.0, 0.0, 0.0);
    // Green color constant
    // Color.hpp:31
    pub const GREEN: Self = Self::new(0.0, 1.0, 0.0);
    // Blue color constant
    // Color.hpp:32
    pub const BLUE: Self = Self::new(0.0, 0.0, 1.0);
    // Yellow color constant
    // Color.hpp:33
    pub const YELLOW: Self = Self::new(1.0, 1.0, 0.0);
    // Cyan color constant
    // Color.hpp:34
    pub const CYAN: Self = Self::new(0.0, 1.0, 1.0);
    // Magenta color constant
    // Color.hpp:35
    pub const MAGENTA: Self = Self::new(1.0, 0.0, 1.0);

    // Create new RGB color from components
    // Color.cpp:22-26
    pub const fn new(r: f32, g: f32, b: f32) -> Self {
        // Initialize data array with RGB components
        // Color.cpp:23-25
        Self { data: [r, g, b] }
    }

    /// Get red component
    /// Color.cpp:29-31
    pub fn r(&self) -> f32 {
        self.data[0]
    }
    /// Get green component
    /// Color.cpp:33-35
    pub fn g(&self) -> f32 {
        self.data[1]
    }
    /// Get blue component
    /// Color.cpp:37-39
    pub fn b(&self) -> f32 {
        self.data[2]
    }

    /// Set red component with clamping
    /// Color.cpp:42-45
    pub fn set_r(&mut self, v: f32) {
        // Clamp value and assign to red component
        // Color.cpp:43
        self.data[0] = v.clamp(0.0, 1.0);
    }
    /// Set green component with clamping
    /// Color.cpp:47-50
    pub fn set_g(&mut self, v: f32) {
        // Clamp value and assign to green component
        // Color.cpp:48
        self.data[1] = v.clamp(0.0, 1.0);
    }
    /// Set blue component with clamping
    /// Color.cpp:52-55
    pub fn set_b(&mut self, v: f32) {
        // Clamp value and assign to blue component
        // Color.cpp:53
        self.data[2] = v.clamp(0.0, 1.0);
    }

    /// Convert RGB to RGBA with specified alpha
    /// Color.cpp:58-62
    pub fn to_rgba(&self, alpha: f32) -> ColorRGBA {
        // Create RGBA color from RGB components plus alpha
        // Color.cpp:59-61
        ColorRGBA::new(self.r(), self.g(), self.b(), alpha)
    }
}

/// Debug trait implementation for ColorRGB
/// Color.cpp:65-72
impl fmt::Debug for ColorRGB {
    // Format RGB color for debug output
    // Color.cpp:66-70
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Write formatted RGB values
        // Color.cpp:67-69
        write!(f, "RGB({:.2}, {:.2}, {:.2})", self.r(), self.g(), self.b())
    }
}

/// Derive traits for ColorRGBA struct
/// Color.hpp:65-67
#[derive(Clone, Copy, PartialEq)]
/// RGBA color with floating point components [0.0, 1.0]
/// Color.hpp:69-82
pub struct ColorRGBA {
    data: [f32; 4],
}

/// Default trait implementation for ColorRGBA
/// Color.cpp:75-81
impl Default for ColorRGBA {
    // Create default ColorRGBA with opaque white
    // Color.cpp:76-79
    fn default() -> Self {
        // Initialize with white and full opacity
        // Color.cpp:77-78
        Self {
            data: [1.0, 1.0, 1.0, 1.0],
        }
    }
}

/// Implementation of ColorRGBA methods
/// Color.cpp:84-145
impl ColorRGBA {
    // Black color constant with full opacity
    // Color.hpp:85
    pub const BLACK: Self = Self::new(0.0, 0.0, 0.0, 1.0);
    // White color constant with full opacity
    // Color.hpp:86
    pub const WHITE: Self = Self::new(1.0, 1.0, 1.0, 1.0);
    // Red color constant with full opacity
    // Color.hpp:87
    pub const RED: Self = Self::new(1.0, 0.0, 0.0, 1.0);
    // Green color constant with full opacity
    // Color.hpp:88
    pub const GREEN: Self = Self::new(0.0, 1.0, 0.0, 1.0);
    // Blue color constant with full opacity
    // Color.hpp:89
    pub const BLUE: Self = Self::new(0.0, 0.0, 1.0, 1.0);

    // Create new RGBA color from components
    // Color.cpp:87-91
    pub const fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        // Initialize data array with RGBA components
        // Color.cpp:88-90
        Self { data: [r, g, b, a] }
    }

    /// Get red component
    /// Color.cpp:94-96
    pub fn r(&self) -> f32 {
        self.data[0]
    }
    /// Get green component
    /// Color.cpp:98-100
    pub fn g(&self) -> f32 {
        self.data[1]
    }
    /// Get blue component
    /// Color.cpp:102-104
    pub fn b(&self) -> f32 {
        self.data[2]
    }
    /// Get alpha component
    /// Color.cpp:106-108
    pub fn a(&self) -> f32 {
        self.data[3]
    }

    /// Check if color has transparency
    /// Color.cpp:111-115
    pub fn is_transparent(&self) -> bool {
        // Compare alpha against 1.0
        // Color.cpp:112-114
        self.a() < 1.0
    }

    /// Convert RGBA to RGB by dropping alpha channel
    /// Color.cpp:118-122
    pub fn to_rgb(&self) -> ColorRGB {
        // Create RGB color from RGBA components
        // Color.cpp:119-121
        ColorRGB::new(self.r(), self.g(), self.b())
    }
}

/// Debug trait implementation for ColorRGBA
/// Color.cpp:125-132
impl fmt::Debug for ColorRGBA {
    // Format RGBA color for debug output
    // Color.cpp:126-130
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Write formatted RGBA values
        // Color.cpp:127-129
        write!(
            f,
            "RGBA({:.2}, {:.2}, {:.2}, {:.2})",
            self.r(),
            self.g(),
            self.b(),
            self.a()
        )
    }
}
