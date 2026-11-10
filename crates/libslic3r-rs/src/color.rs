//! Color utilities for the slicer.
//!
//! 1:1 faithful port of BambuStudio `src/libslic3r/Color.cpp` (+ `Color.hpp`).
//!
//! `coord_t`/`coordf_t` do not appear here; all components are `float` (`f32`).
//!
//! Divergence note: the `Randomizer` class wraps `std::mt19937` seeded from
//! `std::random_device` and `std::uniform_real_distribution<float>`. That RNG
//! is not byte-reproducible across C++ and Rust, and it only feeds the GUI-side
//! `opposite()` color generators (never the G-code path), so the Rust port uses
//! `rand::thread_rng()` + `gen_range` to match the same `[min, max)` semantics.

use rand::Rng;

// Color.hpp:4-5  #include <array>, <algorithm>

// libslic3r.h / Color.cpp:6  static const float INV_255 = 1.0f / 255.0f;
const INV_255: f32 = 1.0 / 255.0;

// Color.hpp:8  using RGB  = std::array<float, 3>;
pub type RGB = [f32; 3];
// Color.hpp:9  using RGBA = std::array<float, 4>;
pub type RGBA = [f32; 4];
// Color.hpp:10  const RGBA UNDEFINE_COLOR = {0,0,0,0};
pub const UNDEFINE_COLOR: RGBA = [0.0, 0.0, 0.0, 0.0];

// Color.cpp:9-18  bool color_is_equal(const RGBA a, const RGBA& b)
pub fn color_is_equal(a: RGBA, b: &RGBA) -> bool {
    // Color.cpp:11
    for i in 0..4 {
        // Color.cpp:12
        let value = (a[i] - b[i]).abs() * 255.0;
        // Color.cpp:13  //Floating-point precision
        if value >= 0.9 {
            // Color.cpp:14
            return false;
        }
    }
    // Color.cpp:17
    true
}

// Color.cpp:20-21
// Convert 3MF color strings to RGBA
// Supported formats: "#RRGGBB", "#RRGGBBAA", "RRGGBB", "RRGGBBAA"
// Color.cpp:22  RGBA convert_color_string_to_rgba(const std::string& color_str)
pub fn convert_color_string_to_rgba(color_str: &str) -> RGBA {
    // Color.cpp:24
    if color_str.is_empty() {
        // Color.cpp:25
        return UNDEFINE_COLOR;
    }
    // Color.cpp:27
    let mut hex_str = color_str;
    // Color.cpp:28  // Remove the leading '#'
    // Color.cpp:29
    if hex_str.as_bytes()[0] == b'#' {
        // Color.cpp:30
        hex_str = &hex_str[1..];
    }
    // Color.cpp:32  // Deciphering hexadecimal
    // Color.cpp:33
    if hex_str.len() == 6 {
        // Color.cpp:34  // RRGGBB format
        // Color.cpp:35  unsigned int r, g, b;
        // Color.cpp:36  if (sscanf(hex_str.c_str(), "%02x%02x%02x", &r, &g, &b) == 3)
        if let Some((r, g, b)) = sscanf_rgb(hex_str) {
            // Color.cpp:37
            return [r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0, 1.0];
        }
    } else if hex_str.len() == 8 {
        // Color.cpp:40-41  // RRGGBBAA format
        // Color.cpp:42  unsigned int r, g, b, a;
        // Color.cpp:43  if (sscanf(hex_str.c_str(), "%02x%02x%02x%02x", &r, &g, &b, &a) == 4)
        if let Some((r, g, b, a)) = sscanf_rgba(hex_str) {
            // Color.cpp:44
            return [
                r as f32 / 255.0,
                g as f32 / 255.0,
                b as f32 / 255.0,
                a as f32 / 255.0,
            ];
        }
    }
    // Color.cpp:47
    UNDEFINE_COLOR
}

// Helper for `sscanf(..., "%02x%02x%02x", ...)`: parse three 2-digit hex bytes
// from the start of the string. `%02x` reads up to two hex digits each.
fn sscanf_rgb(s: &str) -> Option<(u32, u32, u32)> {
    let r = u32::from_str_radix(s.get(0..2)?, 16).ok()?;
    let g = u32::from_str_radix(s.get(2..4)?, 16).ok()?;
    let b_ = u32::from_str_radix(s.get(4..6)?, 16).ok()?;
    Some((r, g, b_))
}

// Helper for `sscanf(..., "%02x%02x%02x%02x", ...)`.
fn sscanf_rgba(s: &str) -> Option<(u32, u32, u32, u32)> {
    let r = u32::from_str_radix(s.get(0..2)?, 16).ok()?;
    let g = u32::from_str_radix(s.get(2..4)?, 16).ok()?;
    let b = u32::from_str_radix(s.get(4..6)?, 16).ok()?;
    let a = u32::from_str_radix(s.get(6..8)?, 16).ok()?;
    Some((r, g, b, a))
}

// Color.cpp:50-52
// Conversion from RGB to HSV color space
// The input RGB values are in the range [0, 1]
// The output HSV values are in the ranges h = [0, 360], and s, v = [0, 1]
// Color.cpp:53  static void RGBtoHSV(float r, float g, float b, float& h, float& s, float& v)
#[allow(non_snake_case)]
fn rgb_to_hsv(r: f32, g: f32, b: f32, h: &mut f32, s: &mut f32, v: &mut f32) {
    // Color.cpp:55-57
    debug_assert!((0.0..=1.0).contains(&r));
    debug_assert!((0.0..=1.0).contains(&g));
    debug_assert!((0.0..=1.0).contains(&b));

    // Color.cpp:59
    let max_comp = r.max(g).max(b);
    // Color.cpp:60
    let min_comp = r.min(g).min(b);
    // Color.cpp:61
    let delta = max_comp - min_comp;

    // Color.cpp:63
    if delta > 0.0 {
        // Color.cpp:64
        if max_comp == r {
            // Color.cpp:65
            *h = 60.0 * (((g - b) / delta) % 6.0);
        }
        // Color.cpp:66
        else if max_comp == g {
            // Color.cpp:67
            *h = 60.0 * (((b - r) / delta) + 2.0);
        }
        // Color.cpp:68
        else if max_comp == b {
            // Color.cpp:69
            *h = 60.0 * (((r - g) / delta) + 4.0);
        }

        // Color.cpp:71
        *s = if max_comp > 0.0 { delta / max_comp } else { 0.0 };
    }
    // Color.cpp:73
    else {
        // Color.cpp:74
        *h = 0.0;
        // Color.cpp:75
        *s = 0.0;
    }
    // Color.cpp:77
    *v = max_comp;

    // Color.cpp:79
    while *h < 0.0 {
        *h += 360.0;
    }
    // Color.cpp:80
    while *h > 360.0 {
        *h -= 360.0;
    }

    // Color.cpp:82-84
    debug_assert!((0.0..=1.0).contains(&*s));
    debug_assert!((0.0..=1.0).contains(&*v));
    debug_assert!((0.0..=360.0).contains(&*h));
}

// Color.cpp:87-89
// Conversion from HSV to RGB color space
// The input HSV values are in the ranges h = [0, 360], and s, v = [0, 1]
// The output RGB values are in the range [0, 1]
// Color.cpp:90  static void HSVtoRGB(float h, float s, float v, float& r, float& g, float& b)
#[allow(non_snake_case)]
fn hsv_to_rgb(h: f32, s: f32, v: f32, r: &mut f32, g: &mut f32, b: &mut f32) {
    // Color.cpp:92-94
    debug_assert!((0.0..=1.0).contains(&s));
    debug_assert!((0.0..=1.0).contains(&v));
    debug_assert!((0.0..=360.0).contains(&h));

    // Color.cpp:96
    let chroma = v * s;
    // Color.cpp:97
    let h_prime = (h / 60.0) % 6.0;
    // Color.cpp:98
    let x = chroma * (1.0 - ((h_prime % 2.0) - 1.0).abs());
    // Color.cpp:99
    let m = v - chroma;

    // Color.cpp:101
    if (0.0..1.0).contains(&h_prime) {
        // Color.cpp:102-104
        *r = chroma;
        *g = x;
        *b = 0.0;
    }
    // Color.cpp:106
    else if (1.0..2.0).contains(&h_prime) {
        // Color.cpp:107-109
        *r = x;
        *g = chroma;
        *b = 0.0;
    }
    // Color.cpp:111
    else if (2.0..3.0).contains(&h_prime) {
        // Color.cpp:112-114
        *r = 0.0;
        *g = chroma;
        *b = x;
    }
    // Color.cpp:116
    else if (3.0..4.0).contains(&h_prime) {
        // Color.cpp:117-119
        *r = 0.0;
        *g = x;
        *b = chroma;
    }
    // Color.cpp:121
    else if (4.0..5.0).contains(&h_prime) {
        // Color.cpp:122-124
        *r = x;
        *g = 0.0;
        *b = chroma;
    }
    // Color.cpp:126
    else if (5.0..6.0).contains(&h_prime) {
        // Color.cpp:127-129
        *r = chroma;
        *g = 0.0;
        *b = x;
    }
    // Color.cpp:131
    else {
        // Color.cpp:132-134
        *r = 0.0;
        *g = 0.0;
        *b = 0.0;
    }

    // Color.cpp:137-139
    *r += m;
    *g += m;
    *b += m;

    // Color.cpp:141-143
    debug_assert!((0.0..=1.0).contains(&*r));
    debug_assert!((0.0..=1.0).contains(&*g));
    debug_assert!((0.0..=1.0).contains(&*b));
}

// Color.cpp:146-156  class Randomizer
struct Randomizer;

impl Randomizer {
    // Color.cpp:151-155  float random_float(float min, float max)
    fn random_float(&self, min: f32, max: f32) -> f32 {
        // Color.cpp:152-154
        // std::mt19937 seeded from std::random_device, uniform_real_distribution.
        // Not byte-reproducible across toolchains; see module divergence note.
        let mut rand_generator = rand::thread_rng();
        rand_generator.gen_range(min..max)
    }
}

// Color.hpp:15-77  class ColorRGB
#[derive(Clone, Copy)]
pub struct ColorRGB {
    // Color.hpp:17  std::array<float, 3> m_data{1.0f, 1.0f, 1.0f};
    m_data: [f32; 3],
}

impl Default for ColorRGB {
    // Color.hpp:20  ColorRGB() = default; (with member initializer {1,1,1})
    fn default() -> Self {
        Self {
            m_data: [1.0, 1.0, 1.0],
        }
    }
}

impl ColorRGB {
    // Color.cpp:158-161  ColorRGB::ColorRGB(float r, float g, float b)
    pub fn new(r: f32, g: f32, b: f32) -> Self {
        // Color.cpp:159
        Self {
            m_data: [r.clamp(0.0, 1.0), g.clamp(0.0, 1.0), b.clamp(0.0, 1.0)],
        }
    }

    // Color.cpp:163-166  ColorRGB::ColorRGB(unsigned char r, unsigned char g, unsigned char b)
    pub fn from_uchar(r: u8, g: u8, b: u8) -> Self {
        // Color.cpp:164
        Self {
            m_data: [
                (r as f32 * INV_255).clamp(0.0, 1.0),
                (g as f32 * INV_255).clamp(0.0, 1.0),
                (b as f32 * INV_255).clamp(0.0, 1.0),
            ],
        }
    }

    // Color.hpp:35  const float* const data() const
    pub fn data(&self) -> &[f32; 3] {
        &self.m_data
    }

    // Color.hpp:37  float r() const
    pub fn r(&self) -> f32 {
        self.m_data[0]
    }
    // Color.hpp:38  float g() const
    pub fn g(&self) -> f32 {
        self.m_data[1]
    }
    // Color.hpp:39  float b() const
    pub fn b(&self) -> f32 {
        self.m_data[2]
    }

    // Color.hpp:41  void r(float r)
    pub fn set_r(&mut self, r: f32) {
        self.m_data[0] = r.clamp(0.0, 1.0);
    }
    // Color.hpp:42  void g(float g)
    pub fn set_g(&mut self, g: f32) {
        self.m_data[1] = g.clamp(0.0, 1.0);
    }
    // Color.hpp:43  void b(float b)
    pub fn set_b(&mut self, b: f32) {
        self.m_data[2] = b.clamp(0.0, 1.0);
    }

    // Color.hpp:45-48  void set(unsigned int comp, float value)
    pub fn set(&mut self, comp: u32, value: f32) {
        // Color.hpp:46
        debug_assert!(comp <= 2);
        // Color.hpp:47
        self.m_data[comp as usize] = value.clamp(0.0, 1.0);
    }

    // Color.hpp:50  unsigned char r_uchar() const
    pub fn r_uchar(&self) -> u8 {
        (self.m_data[0] * 255.0) as u8
    }
    // Color.hpp:51  unsigned char g_uchar() const
    pub fn g_uchar(&self) -> u8 {
        (self.m_data[1] * 255.0) as u8
    }
    // Color.hpp:52  unsigned char b_uchar() const
    pub fn b_uchar(&self) -> u8 {
        (self.m_data[2] * 255.0) as u8
    }

    // Color.hpp:54  static const ColorRGB BLACK()
    pub fn black() -> ColorRGB {
        ColorRGB::new(0.0, 0.0, 0.0)
    }
    // Color.hpp:55  static const ColorRGB BLUE()
    pub fn blue() -> ColorRGB {
        ColorRGB::new(0.0, 0.0, 1.0)
    }
    // Color.hpp:56  static const ColorRGB BLUEISH()
    pub fn blueish() -> ColorRGB {
        ColorRGB::new(0.5, 0.5, 1.0)
    }
    // Color.hpp:57  static const ColorRGB CYAN()
    pub fn cyan() -> ColorRGB {
        ColorRGB::new(0.0, 1.0, 1.0)
    }
    // Color.hpp:58  static const ColorRGB DARK_GRAY()
    pub fn dark_gray() -> ColorRGB {
        ColorRGB::new(0.25, 0.25, 0.25)
    }
    // Color.hpp:59  static const ColorRGB DARK_YELLOW()
    pub fn dark_yellow() -> ColorRGB {
        ColorRGB::new(0.5, 0.5, 0.0)
    }
    // Color.hpp:60  static const ColorRGB GRAY()
    pub fn gray() -> ColorRGB {
        ColorRGB::new(0.5, 0.5, 0.5)
    }
    // Color.hpp:61  static const ColorRGB GREEN()
    pub fn green() -> ColorRGB {
        ColorRGB::new(0.0, 1.0, 0.0)
    }
    // Color.hpp:62  static const ColorRGB GREENISH()
    pub fn greenish() -> ColorRGB {
        ColorRGB::new(0.5, 1.0, 0.5)
    }
    // Color.hpp:63  static const ColorRGB LIGHT_GRAY()
    pub fn light_gray() -> ColorRGB {
        ColorRGB::new(0.75, 0.75, 0.75)
    }
    // Color.hpp:64  static const ColorRGB MAGENTA()
    pub fn magenta() -> ColorRGB {
        ColorRGB::new(1.0, 0.0, 1.0)
    }
    // Color.hpp:65  static const ColorRGB ORANGE()
    pub fn orange() -> ColorRGB {
        ColorRGB::new(0.92, 0.50, 0.26)
    }
    // Color.hpp:66  static const ColorRGB RED()
    pub fn red() -> ColorRGB {
        ColorRGB::new(1.0, 0.0, 0.0)
    }
    // Color.hpp:67  static const ColorRGB REDISH()
    pub fn redish() -> ColorRGB {
        ColorRGB::new(1.0, 0.5, 0.5)
    }
    // Color.hpp:68  static const ColorRGB YELLOW()
    pub fn yellow() -> ColorRGB {
        ColorRGB::new(1.0, 1.0, 0.0)
    }
    // Color.hpp:69  static const ColorRGB WHITE()
    pub fn white() -> ColorRGB {
        ColorRGB::new(1.0, 1.0, 1.0)
    }
    // Color.hpp:70  static const ColorRGB ORCA()
    pub fn orca() -> ColorRGB {
        ColorRGB::new(0.0, 150.0 / 255.0, 136.0 / 255.0)
    }
    // Color.hpp:71  static const ColorRGB WARNING()
    pub fn warning() -> ColorRGB {
        ColorRGB::new(241.0 / 255.0, 117.0 / 255.0, 78.0 / 255.0)
    }
    // Color.hpp:72  static const ColorRGB ERROR_COLOR()
    pub fn error_color() -> ColorRGB {
        ColorRGB::new(208.0 / 255.0, 27.0 / 255.0, 27.0 / 255.0)
    }

    // Color.hpp:74  static const ColorRGB X()
    pub fn x() -> ColorRGB {
        ColorRGB::new(0.75, 0.0, 0.0)
    }
    // Color.hpp:75  static const ColorRGB Y()
    pub fn y() -> ColorRGB {
        ColorRGB::new(0.0, 0.75, 0.0)
    }
    // Color.hpp:76  static const ColorRGB Z()
    pub fn z() -> ColorRGB {
        ColorRGB::new(0.0, 0.0, 0.75)
    }
}

// Color.hpp:27  bool operator == (const ColorRGB& other) const { return m_data == other.m_data; }
// Color.hpp:28  bool operator != (const ColorRGB& other) const { return !operator==(other); }
impl PartialEq for ColorRGB {
    fn eq(&self, other: &Self) -> bool {
        self.m_data == other.m_data
    }
}

// Color.cpp:168-178  bool ColorRGB::operator < (const ColorRGB& other) const
// Color.cpp:180-190  bool ColorRGB::operator > (const ColorRGB& other) const
impl PartialOrd for ColorRGB {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        // Color.cpp:170
        for i in 0..3 {
            // Color.cpp:171
            if self.m_data[i] < other.m_data[i] {
                // Color.cpp:172
                return Some(std::cmp::Ordering::Less);
            }
            // Color.cpp:173
            else if self.m_data[i] > other.m_data[i] {
                // Color.cpp:174
                return Some(std::cmp::Ordering::Greater);
            }
        }
        // Color.cpp:177
        Some(std::cmp::Ordering::Equal)
    }
}

// Color.cpp:192-199  ColorRGB ColorRGB::operator + (const ColorRGB& other) const
impl std::ops::Add for ColorRGB {
    type Output = ColorRGB;
    fn add(self, other: ColorRGB) -> ColorRGB {
        // Color.cpp:194
        let mut ret = ColorRGB::default();
        // Color.cpp:195
        for i in 0..3 {
            // Color.cpp:196
            ret.m_data[i] = (self.m_data[i] + other.m_data[i]).clamp(0.0, 1.0);
        }
        // Color.cpp:198
        ret
    }
}

// Color.cpp:201-209  ColorRGB ColorRGB::operator * (float value) const
impl std::ops::Mul<f32> for ColorRGB {
    type Output = ColorRGB;
    fn mul(self, value: f32) -> ColorRGB {
        // Color.cpp:203
        debug_assert!(value >= 0.0);
        // Color.cpp:204
        let mut ret = ColorRGB::default();
        // Color.cpp:205
        for i in 0..3 {
            // Color.cpp:206
            ret.m_data[i] = (value * self.m_data[i]).clamp(0.0, 1.0);
        }
        // Color.cpp:208
        ret
    }
}

// Color.cpp:291  ColorRGB operator * (float value, const ColorRGB& other) { return other * value; }
impl std::ops::Mul<ColorRGB> for f32 {
    type Output = ColorRGB;
    fn mul(self, other: ColorRGB) -> ColorRGB {
        other * self
    }
}

// Color.hpp:79-150  class ColorRGBA
#[derive(Clone, Copy)]
pub struct ColorRGBA {
    // Color.hpp:81  std::array<float, 4> m_data{ 1.0f, 1.0f, 1.0f, 1.0f };
    m_data: [f32; 4],
}

impl Default for ColorRGBA {
    // Color.hpp:84  ColorRGBA() = default; (with member initializer {1,1,1,1})
    fn default() -> Self {
        Self {
            m_data: [1.0, 1.0, 1.0, 1.0],
        }
    }
}

impl ColorRGBA {
    // Color.cpp:211-214  ColorRGBA::ColorRGBA(float r, float g, float b, float a)
    pub fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        // Color.cpp:212
        Self {
            m_data: [
                r.clamp(0.0, 1.0),
                g.clamp(0.0, 1.0),
                b.clamp(0.0, 1.0),
                a.clamp(0.0, 1.0),
            ],
        }
    }

    // Color.cpp:216-220  ColorRGBA::ColorRGBA(std::array<float, 4> color)
    pub fn from_array(color: [f32; 4]) -> Self {
        // Color.cpp:217
        Self {
            m_data: [
                color[0].clamp(0.0, 1.0),
                color[1].clamp(0.0, 1.0),
                color[2].clamp(0.0, 1.0),
                color[3].clamp(0.0, 1.0),
            ],
        }
    }

    // Color.cpp:221-224  ColorRGBA::ColorRGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
    pub fn from_uchar(r: u8, g: u8, b: u8, a: u8) -> Self {
        // Color.cpp:222
        Self {
            m_data: [
                (r as f32 * INV_255).clamp(0.0, 1.0),
                (g as f32 * INV_255).clamp(0.0, 1.0),
                (b as f32 * INV_255).clamp(0.0, 1.0),
                (a as f32 * INV_255).clamp(0.0, 1.0),
            ],
        }
    }

    // Color.hpp:102  const float* const data() const
    pub fn data(&self) -> &[f32; 4] {
        &self.m_data
    }
    // Color.hpp:103  const std::array<float, 4> &get_data() const
    pub fn get_data(&self) -> &[f32; 4] {
        &self.m_data
    }

    // Color.hpp:105  float r() const
    pub fn r(&self) -> f32 {
        self.m_data[0]
    }
    // Color.hpp:106  float g() const
    pub fn g(&self) -> f32 {
        self.m_data[1]
    }
    // Color.hpp:107  float b() const
    pub fn b(&self) -> f32 {
        self.m_data[2]
    }
    // Color.hpp:108  float a() const
    pub fn a(&self) -> f32 {
        self.m_data[3]
    }

    // Color.hpp:110  void r(float r)
    pub fn set_r(&mut self, r: f32) {
        self.m_data[0] = r.clamp(0.0, 1.0);
    }
    // Color.hpp:111  void g(float g)
    pub fn set_g(&mut self, g: f32) {
        self.m_data[1] = g.clamp(0.0, 1.0);
    }
    // Color.hpp:112  void b(float b)
    pub fn set_b(&mut self, b: f32) {
        self.m_data[2] = b.clamp(0.0, 1.0);
    }
    // Color.hpp:113  void a(float a)
    pub fn set_a(&mut self, a: f32) {
        self.m_data[3] = a.clamp(0.0, 1.0);
    }

    // Color.cpp:270-275  void ColorRGBA::gamma_correct()
    pub fn gamma_correct(&mut self) {
        // Color.cpp:271
        let coe = 1.0 / 2.2_f32;
        // Color.cpp:272
        for i in 0..4 {
            // Color.cpp:273
            self.m_data[i] = self.m_data[i].powf(coe);
        }
    }

    // Color.cpp:277-283  void ColorRGBA::gamma_correct(RGBA &color)
    pub fn gamma_correct_rgba(color: &mut RGBA) {
        // Color.cpp:279
        let coe = 1.0 / 2.2_f32;
        // Color.cpp:280
        for i in 0..4 {
            // Color.cpp:281
            color[i] = color[i].powf(coe);
        }
    }

    // Color.cpp:285-289  float ColorRGBA::gamma_correct(float value)
    pub fn gamma_correct_value(value: f32) -> f32 {
        // Color.cpp:287
        let coe = 1.0 / 2.2_f32;
        // Color.cpp:288
        value.powf(coe)
    }

    // Color.hpp:117-120  void set(unsigned int comp, float value)
    pub fn set(&mut self, comp: u32, value: f32) {
        // Color.hpp:118
        debug_assert!(comp <= 3);
        // Color.hpp:119
        self.m_data[comp as usize] = value.clamp(0.0, 1.0);
    }

    // Color.hpp:122  unsigned char r_uchar() const
    pub fn r_uchar(&self) -> u8 {
        (self.m_data[0] * 255.0) as u8
    }
    // Color.hpp:123  unsigned char g_uchar() const
    pub fn g_uchar(&self) -> u8 {
        (self.m_data[1] * 255.0) as u8
    }
    // Color.hpp:124  unsigned char b_uchar() const
    pub fn b_uchar(&self) -> u8 {
        (self.m_data[2] * 255.0) as u8
    }
    // Color.hpp:125  unsigned char a_uchar() const
    pub fn a_uchar(&self) -> u8 {
        (self.m_data[3] * 255.0) as u8
    }

    // Color.hpp:127  bool is_transparent() const
    pub fn is_transparent(&self) -> bool {
        self.m_data[3] < 1.0
    }

    // Color.hpp:129  static const ColorRGBA BLACK()
    pub fn black() -> ColorRGBA {
        ColorRGBA::new(0.0, 0.0, 0.0, 1.0)
    }
    // Color.hpp:130  static const ColorRGBA BLUE()
    pub fn blue() -> ColorRGBA {
        ColorRGBA::new(0.0, 0.0, 1.0, 1.0)
    }
    // Color.hpp:131  static const ColorRGBA BLUEISH()
    pub fn blueish() -> ColorRGBA {
        ColorRGBA::new(0.5, 0.5, 1.0, 1.0)
    }
    // Color.hpp:132  static const ColorRGBA CYAN()
    pub fn cyan() -> ColorRGBA {
        ColorRGBA::new(0.0, 1.0, 1.0, 1.0)
    }
    // Color.hpp:133  static const ColorRGBA DARK_GRAY()
    pub fn dark_gray() -> ColorRGBA {
        ColorRGBA::new(0.25, 0.25, 0.25, 1.0)
    }
    // Color.hpp:134  static const ColorRGBA DARK_YELLOW()
    pub fn dark_yellow() -> ColorRGBA {
        ColorRGBA::new(0.5, 0.5, 0.0, 1.0)
    }
    // Color.hpp:135  static const ColorRGBA GRAY()
    pub fn gray() -> ColorRGBA {
        ColorRGBA::new(0.5, 0.5, 0.5, 1.0)
    }
    // Color.hpp:136  static const ColorRGBA GREEN()
    pub fn green() -> ColorRGBA {
        ColorRGBA::new(0.0, 1.0, 0.0, 1.0)
    }
    // Color.hpp:137  static const ColorRGBA GREENISH()
    pub fn greenish() -> ColorRGBA {
        ColorRGBA::new(0.5, 1.0, 0.5, 1.0)
    }
    // Color.hpp:138  static const ColorRGBA LIGHT_GRAY()
    pub fn light_gray() -> ColorRGBA {
        ColorRGBA::new(0.75, 0.75, 0.75, 1.0)
    }
    // Color.hpp:139  static const ColorRGBA MAGENTA()
    pub fn magenta() -> ColorRGBA {
        ColorRGBA::new(1.0, 0.0, 1.0, 1.0)
    }
    // Color.hpp:140  static const ColorRGBA ORANGE()
    pub fn orange() -> ColorRGBA {
        ColorRGBA::new(0.923, 0.504, 0.264, 1.0)
    }
    // Color.hpp:141  static const ColorRGBA RED()
    pub fn red() -> ColorRGBA {
        ColorRGBA::new(1.0, 0.0, 0.0, 1.0)
    }
    // Color.hpp:142  static const ColorRGBA REDISH()
    pub fn redish() -> ColorRGBA {
        ColorRGBA::new(1.0, 0.5, 0.5, 1.0)
    }
    // Color.hpp:143  static const ColorRGBA YELLOW()
    pub fn yellow() -> ColorRGBA {
        ColorRGBA::new(1.0, 1.0, 0.0, 1.0)
    }
    // Color.hpp:144  static const ColorRGBA WHITE()
    pub fn white() -> ColorRGBA {
        ColorRGBA::new(1.0, 1.0, 1.0, 1.0)
    }
    // Color.hpp:145  static const ColorRGBA ORCA()
    pub fn orca() -> ColorRGBA {
        ColorRGBA::new(0.0, 150.0 / 255.0, 136.0 / 255.0, 1.0)
    }

    // Color.hpp:147  static const ColorRGBA X()
    pub fn x() -> ColorRGBA {
        ColorRGBA::new(0.75, 0.0, 0.0, 1.0)
    }
    // Color.hpp:148  static const ColorRGBA Y()
    pub fn y() -> ColorRGBA {
        ColorRGBA::new(0.0, 0.75, 0.0, 1.0)
    }
    // Color.hpp:149  static const ColorRGBA Z()
    pub fn z() -> ColorRGBA {
        ColorRGBA::new(0.0, 0.0, 0.75, 1.0)
    }
}

// Color.hpp:92-95
// bool operator==(const ColorRGBA &other) const { return color_is_equal(m_data, other.m_data); }
// bool operator != (const ColorRGBA& other) const { return !operator==(other); }
impl PartialEq for ColorRGBA {
    fn eq(&self, other: &Self) -> bool {
        color_is_equal(self.m_data, &other.m_data)
    }
}

// Color.cpp:226-236  bool ColorRGBA::operator < (const ColorRGBA& other) const
// Color.cpp:238-248  bool ColorRGBA::operator > (const ColorRGBA& other) const
impl PartialOrd for ColorRGBA {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        // Color.cpp:228  (note: C++ iterates only i < 3, ignoring alpha)
        for i in 0..3 {
            // Color.cpp:229
            if self.m_data[i] < other.m_data[i] {
                // Color.cpp:230
                return Some(std::cmp::Ordering::Less);
            }
            // Color.cpp:231
            else if self.m_data[i] > other.m_data[i] {
                // Color.cpp:232
                return Some(std::cmp::Ordering::Greater);
            }
        }
        // Color.cpp:235
        Some(std::cmp::Ordering::Equal)
    }
}

// Color.cpp:250-257  ColorRGBA ColorRGBA::operator + (const ColorRGBA& other) const
impl std::ops::Add for ColorRGBA {
    type Output = ColorRGBA;
    fn add(self, other: ColorRGBA) -> ColorRGBA {
        // Color.cpp:252
        let mut ret = ColorRGBA::default();
        // Color.cpp:253  (note: C++ iterates only i < 3, leaving alpha = default 1.0)
        for i in 0..3 {
            // Color.cpp:254
            ret.m_data[i] = (self.m_data[i] + other.m_data[i]).clamp(0.0, 1.0);
        }
        // Color.cpp:256
        ret
    }
}

// Color.cpp:259-268  ColorRGBA ColorRGBA::operator * (float value) const
impl std::ops::Mul<f32> for ColorRGBA {
    type Output = ColorRGBA;
    fn mul(self, value: f32) -> ColorRGBA {
        // Color.cpp:261
        debug_assert!(value >= 0.0);
        // Color.cpp:262
        let mut ret = ColorRGBA::default();
        // Color.cpp:263
        for i in 0..3 {
            // Color.cpp:264
            ret.m_data[i] = (value * self.m_data[i]).clamp(0.0, 1.0);
        }
        // Color.cpp:266
        ret.m_data[3] = self.m_data[3];
        // Color.cpp:267
        ret
    }
}

// Color.cpp:292  ColorRGBA operator * (float value, const ColorRGBA& other) { return other * value; }
impl std::ops::Mul<ColorRGBA> for f32 {
    type Output = ColorRGBA;
    fn mul(self, other: ColorRGBA) -> ColorRGBA {
        other * self
    }
}

// Color.cpp:294-298  ColorRGB lerp(const ColorRGB& a, const ColorRGB& b, float t)
pub fn lerp_rgb(a: &ColorRGB, b: &ColorRGB, t: f32) -> ColorRGB {
    // Color.cpp:296
    debug_assert!((0.0..=1.0).contains(&t));
    // Color.cpp:297
    (1.0 - t) * *a + t * *b
}

// Color.cpp:300-304  ColorRGBA lerp(const ColorRGBA& a, const ColorRGBA& b, float t)
pub fn lerp_rgba(a: &ColorRGBA, b: &ColorRGBA, t: f32) -> ColorRGBA {
    // Color.cpp:302
    debug_assert!((0.0..=1.0).contains(&t));
    // Color.cpp:303
    (1.0 - t) * *a + t * *b
}

// Color.cpp:306-309  ColorRGB complementary(const ColorRGB& color)
pub fn complementary_rgb(color: &ColorRGB) -> ColorRGB {
    // Color.cpp:308
    ColorRGB::new(1.0 - color.r(), 1.0 - color.g(), 1.0 - color.b())
}

// Color.cpp:311-314  ColorRGBA complementary(const ColorRGBA& color)
pub fn complementary_rgba(color: &ColorRGBA) -> ColorRGBA {
    // Color.cpp:313
    ColorRGBA::new(1.0 - color.r(), 1.0 - color.g(), 1.0 - color.b(), color.a())
}

// Color.cpp:316-324  ColorRGB saturate(const ColorRGB& color, float factor)
pub fn saturate_rgb(color: &ColorRGB, factor: f32) -> ColorRGB {
    // Color.cpp:318
    let (mut h, mut s, mut v) = (0.0f32, 0.0f32, 0.0f32);
    // Color.cpp:319
    rgb_to_hsv(color.r(), color.g(), color.b(), &mut h, &mut s, &mut v);
    // Color.cpp:320
    s = (s * factor).clamp(0.0, 1.0);
    // Color.cpp:321
    let (mut r, mut g, mut b) = (0.0f32, 0.0f32, 0.0f32);
    // Color.cpp:322
    hsv_to_rgb(h, s, v, &mut r, &mut g, &mut b);
    // Color.cpp:323
    ColorRGB::new(r, g, b)
}

// Color.cpp:326-329  ColorRGBA saturate(const ColorRGBA& color, float factor)
pub fn saturate_rgba(color: &ColorRGBA, factor: f32) -> ColorRGBA {
    // Color.cpp:328
    to_rgba_alpha(&saturate_rgb(&to_rgb(color), factor), color.a())
}

// Color.cpp:331-347  ColorRGB opposite(const ColorRGB& color)
pub fn opposite_rgb(color: &ColorRGB) -> ColorRGB {
    // Color.cpp:333
    let (mut h, mut s, mut v) = (0.0f32, 0.0f32, 0.0f32);
    // Color.cpp:334
    rgb_to_hsv(color.r(), color.g(), color.b(), &mut h, &mut s, &mut v);

    // Color.cpp:336  // 65 instead 60 to avoid circle values
    h += 65.0;
    // Color.cpp:337
    if h > 360.0 {
        // Color.cpp:338
        h -= 360.0;
    }

    // Color.cpp:340
    let rnd = Randomizer;
    // Color.cpp:341
    s = rnd.random_float(0.65, 1.0);
    // Color.cpp:342
    v = rnd.random_float(0.65, 1.0);

    // Color.cpp:344
    let (mut r, mut g, mut b) = (0.0f32, 0.0f32, 0.0f32);
    // Color.cpp:345
    hsv_to_rgb(h, s, v, &mut r, &mut g, &mut b);
    // Color.cpp:346
    ColorRGB::new(r, g, b)
}

// Color.cpp:349-373  ColorRGB opposite(const ColorRGB& a, const ColorRGB& b)
pub fn opposite_rgb2(a: &ColorRGB, b: &ColorRGB) -> ColorRGB {
    // Color.cpp:351
    let (mut ha, mut sa, mut va) = (0.0f32, 0.0f32, 0.0f32);
    // Color.cpp:352
    rgb_to_hsv(a.r(), a.g(), a.b(), &mut ha, &mut sa, &mut va);
    // Color.cpp:353
    let (mut hb, mut sb, mut vb) = (0.0f32, 0.0f32, 0.0f32);
    // Color.cpp:354
    rgb_to_hsv(b.r(), b.g(), b.b(), &mut hb, &mut sb, &mut vb);

    // Color.cpp:356
    let mut delta_h = (ha - hb).abs();
    // Color.cpp:357
    let mut start_h = if delta_h > 180.0 {
        ha.min(hb)
    } else {
        ha.max(hb)
    };

    // Color.cpp:359  // to avoid circle change of colors for 120 deg
    start_h += 5.0;
    // Color.cpp:360
    if delta_h < 180.0 {
        // Color.cpp:361
        delta_h = 360.0 - delta_h;
    }

    // Color.cpp:363
    let rnd = Randomizer;
    // Color.cpp:364
    let mut out_h = start_h + 0.5 * delta_h;
    // Color.cpp:365
    if out_h > 360.0 {
        // Color.cpp:366
        out_h -= 360.0;
    }
    // Color.cpp:367
    let out_s = rnd.random_float(0.65, 1.0);
    // Color.cpp:368
    let out_v = rnd.random_float(0.65, 1.0);

    // Color.cpp:370
    let (mut out_r, mut out_g, mut out_b) = (0.0f32, 0.0f32, 0.0f32);
    // Color.cpp:371
    hsv_to_rgb(out_h, out_s, out_v, &mut out_r, &mut out_g, &mut out_b);
    // Color.cpp:372
    ColorRGB::new(out_r, out_g, out_b)
}

// Color.cpp:375-378  bool can_decode_color(const std::string &color)
pub fn can_decode_color(color: &str) -> bool {
    // Color.cpp:377
    (color.len() == 7 && color.as_bytes().first() == Some(&b'#'))
        || (color.len() == 9 && color.as_bytes().first() == Some(&b'#'))
}

// Color.cpp:380-388  bool decode_color(const std::string& color_in, ColorRGB& color_out)
pub fn decode_color_rgb(color_in: &str, color_out: &mut ColorRGB) -> bool {
    // Color.cpp:382
    let mut rgba = ColorRGBA::default();
    // Color.cpp:383
    if !decode_color_rgba(color_in, &mut rgba) {
        // Color.cpp:384
        return false;
    }

    // Color.cpp:386
    *color_out = to_rgb(&rgba);
    // Color.cpp:387
    true
}

// Color.cpp:390-425  bool decode_color(const std::string& color_in, ColorRGBA& color_out)
pub fn decode_color_rgba(color_in: &str, color_out: &mut ColorRGBA) -> bool {
    // Color.cpp:392-397  lambda hex_digit_to_int
    let hex_digit_to_int = |c: u8| -> i32 {
        if c.is_ascii_digit() {
            (c - b'0') as i32
        } else if (b'A'..=b'F').contains(&c) {
            (c - b'A') as i32 + 10
        } else if (b'a'..=b'f').contains(&c) {
            (c - b'a') as i32 + 10
        } else {
            -1
        }
    };

    // Color.cpp:399
    *color_out = ColorRGBA::black();
    // Color.cpp:400
    if can_decode_color(color_in) {
        // Color.cpp:401  const char *c = color_in.data() + 1;
        let bytes = color_in.as_bytes();
        let mut c = 1usize;
        // Color.cpp:402
        if color_in.len() == 7 {
            // Color.cpp:403
            for i in 0..3u32 {
                // Color.cpp:404
                let digit1 = hex_digit_to_int(bytes[c]);
                c += 1;
                // Color.cpp:405
                let digit2 = hex_digit_to_int(bytes[c]);
                c += 1;
                // Color.cpp:406
                if digit1 != -1 && digit2 != -1 {
                    // Color.cpp:407
                    color_out.set(i, (digit1 * 16 + digit2) as f32 * INV_255);
                }
            }
        } else {
            // Color.cpp:410
            for i in 0..4u32 {
                // Color.cpp:411
                let digit1 = hex_digit_to_int(bytes[c]);
                c += 1;
                // Color.cpp:412
                let digit2 = hex_digit_to_int(bytes[c]);
                c += 1;
                // Color.cpp:413
                if digit1 != -1 && digit2 != -1 {
                    // Color.cpp:414
                    color_out.set(i, (digit1 * 16 + digit2) as f32 * INV_255);
                }
            }
        }
    } else {
        // Color.cpp:418
        return false;
    }

    // Color.cpp:420-423
    debug_assert!((0.0..=1.0).contains(&color_out.r()));
    debug_assert!((0.0..=1.0).contains(&color_out.g()));
    debug_assert!((0.0..=1.0).contains(&color_out.b()));
    debug_assert!((0.0..=1.0).contains(&color_out.a()));
    // Color.cpp:424
    true
}

// Color.cpp:427-435  bool decode_colors(const std::vector<std::string>&, std::vector<ColorRGB>&)
pub fn decode_colors_rgb(colors_in: &[String], colors_out: &mut Vec<ColorRGB>) -> bool {
    // Color.cpp:429
    *colors_out = vec![ColorRGB::black(); colors_in.len()];
    // Color.cpp:430
    for i in 0..colors_in.len() {
        // Color.cpp:431
        if !decode_color_rgb(&colors_in[i], &mut colors_out[i]) {
            // Color.cpp:432
            return false;
        }
    }
    // Color.cpp:434
    true
}

// Color.cpp:437-445  bool decode_colors(const std::vector<std::string>&, std::vector<ColorRGBA>&)
pub fn decode_colors_rgba(colors_in: &[String], colors_out: &mut Vec<ColorRGBA>) -> bool {
    // Color.cpp:439
    *colors_out = vec![ColorRGBA::black(); colors_in.len()];
    // Color.cpp:440
    for i in 0..colors_in.len() {
        // Color.cpp:441
        if !decode_color_rgba(&colors_in[i], &mut colors_out[i]) {
            // Color.cpp:442
            return false;
        }
    }
    // Color.cpp:444
    true
}

// Color.cpp:447-452  std::string encode_color(const ColorRGB& color)
pub fn encode_color_rgb(color: &ColorRGB) -> String {
    // Color.cpp:449-450  ::sprintf(buffer, "#%02X%02X%02X", r_uchar(), g_uchar(), b_uchar());
    format!(
        "#{:02X}{:02X}{:02X}",
        color.r_uchar(),
        color.g_uchar(),
        color.b_uchar()
    )
}

// Color.cpp:454  std::string encode_color(const ColorRGBA& color) { return encode_color(to_rgb(color)); }
pub fn encode_color_rgba(color: &ColorRGBA) -> String {
    encode_color_rgb(&to_rgb(color))
}

// Color.cpp:456  ColorRGB to_rgb(const ColorRGBA& other_rgba)
pub fn to_rgb(other_rgba: &ColorRGBA) -> ColorRGB {
    ColorRGB::new(other_rgba.r(), other_rgba.g(), other_rgba.b())
}

// Color.cpp:457  ColorRGBA to_rgba(const ColorRGB& other_rgb)
pub fn to_rgba(other_rgb: &ColorRGB) -> ColorRGBA {
    ColorRGBA::new(other_rgb.r(), other_rgb.g(), other_rgb.b(), 1.0)
}

// Color.cpp:458  ColorRGBA to_rgba(const ColorRGB& other_rgb, float alpha)
pub fn to_rgba_alpha(other_rgb: &ColorRGB, alpha: f32) -> ColorRGBA {
    ColorRGBA::new(other_rgb.r(), other_rgb.g(), other_rgb.b(), alpha)
}

// Color.cpp:460-468  ColorRGBA picking_decode(unsigned int id)
pub fn picking_decode(id: u32) -> ColorRGBA {
    // Color.cpp:462-467
    ColorRGBA::new(
        ((id >> 0) & 0xff) as f32 * INV_255,  // red
        ((id >> 8) & 0xff) as f32 * INV_255,  // green
        ((id >> 16) & 0xff) as f32 * INV_255, // blue
        // checksum for validating against unwanted alpha blending and multi sampling
        picking_checksum_alpha_channel(
            (id & 0xff) as u8,
            ((id >> 8) & 0xff) as u8,
            ((id >> 16) & 0xff) as u8,
        ) as f32
            * INV_255,
    )
}

// Color.cpp:470  unsigned int picking_encode(unsigned char r, unsigned char g, unsigned char b)
pub fn picking_encode(r: u8, g: u8, b: u8) -> u32 {
    r as u32 + ((g as u32) << 8) + ((b as u32) << 16)
}

// Color.cpp:472-483  unsigned char picking_checksum_alpha_channel(...)
pub fn picking_checksum_alpha_channel(red: u8, green: u8, blue: u8) -> u8 {
    // Color.cpp:474  // 8 bit hash for the color
    // Color.cpp:475  unsigned char b = ((((37 * red) + green) & 0x0ff) * 37 + blue) & 0x0ff;
    let mut b: u8 = ((((37u32 * red as u32) + green as u32) & 0x0ff) * 37 + blue as u32) as u8;
    // Color.cpp:476  // Increase enthropy by a bit reversal
    // Color.cpp:477
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    // Color.cpp:478
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    // Color.cpp:479
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    // Color.cpp:480  // Flip every second bit to increase the enthropy even more.
    // Color.cpp:481
    b ^= 0x55;
    // Color.cpp:482
    b
}
