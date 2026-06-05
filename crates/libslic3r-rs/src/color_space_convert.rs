//! Color space conversion helpers.
//!
//! Faithful partial port of BambuStudio's `slic3r/Utils/ColorSpaceConvert.cpp`.
//! Only the symbols required by `flush_vol_calc.rs` are ported here (`rgb2hsv`).
//! The remaining conversions (XYZ/Lab/DeltaE*, wxColour helpers) live in the
//! GUI-layer source file and are out of scope for the libslic3r parity port
//! (they depend on wxWidgets / boost). They can be added on demand.

// The input r, g, b values should be in range [0, 1]. The output h is in range [0, 360], s is in range [0, 1] and v is in range [0, 1].
// ColorSpaceConvert.cpp:112-140
#[allow(clippy::many_single_char_names)]
pub fn rgb2hsv(r: f32, g: f32, b: f32, h: &mut f32, s: &mut f32, v: &mut f32) {
    // ColorSpaceConvert.cpp:115-117
    let cmax = r.max(g).max(b);
    let cmin = r.min(g).min(b);
    let delta = cmax - cmin;

    // ColorSpaceConvert.cpp:119-130
    if delta.abs() < 0.001 {
        *h = 0.0;
    } else if cmax == r {
        // C++ fmod(x, 6.f): truncated remainder, sign of dividend. Rust `%` on f32 matches.
        *h = 60.0 * (((g - b) / delta) % 6.0);
    } else if cmax == g {
        *h = 60.0 * ((b - r) / delta + 2.0);
    } else {
        *h = 60.0 * ((r - g) / delta + 4.0);
    }

    // ColorSpaceConvert.cpp:132-137
    if cmax.abs() < 0.001 {
        *s = 0.0;
    } else {
        *s = delta / cmax;
    }

    // ColorSpaceConvert.cpp:139
    *v = cmax;
}
