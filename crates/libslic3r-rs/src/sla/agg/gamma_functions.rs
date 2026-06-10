//! Faithful port of the vendored AGG header `src/agg/agg_gamma_functions.h`
//! (Anti-Grain Geometry 2.4, as bundled with BambuStudio).
//!
//! C++ Reference:
//! - agg/agg_gamma_functions.h
//!
//! In C++ the gamma functors are function objects with `double operator()
//! (double) const`; the rasterizer's `gamma()` is a template over them. The
//! Rust port models `operator()` as the `GammaFunction::call` trait method.

/// `double operator()(double x) const` of the C++ gamma functors.
pub trait GammaFunction {
    fn call(&self, x: f64) -> f64;
}

// agg_gamma_functions.h:24-28
// struct gamma_none
// {
//     double operator()(double x) const { return x; }
// };
#[derive(Debug, Clone, Copy, Default)]
pub struct GammaNone;

impl GammaFunction for GammaNone {
    fn call(&self, x: f64) -> f64 {
        x
    }
}

// agg_gamma_functions.h:31-48  class gamma_power
#[derive(Debug, Clone, Copy)]
pub struct GammaPower {
    // agg_gamma_functions.h:47  double m_gamma;
    m_gamma: f64,
}

impl GammaPower {
    // agg_gamma_functions.h:35  gamma_power() : m_gamma(1.0) {}
    pub fn new() -> Self {
        Self { m_gamma: 1.0 }
    }

    // agg_gamma_functions.h:36  gamma_power(double g) : m_gamma(g) {}
    pub fn new_with(g: f64) -> Self {
        Self { m_gamma: g }
    }

    // agg_gamma_functions.h:38  void gamma(double g) { m_gamma = g; }
    pub fn set_gamma(&mut self, g: f64) {
        self.m_gamma = g;
    }

    // agg_gamma_functions.h:39  double gamma() const { return m_gamma; }
    pub fn gamma(&self) -> f64 {
        self.m_gamma
    }
}

impl Default for GammaPower {
    fn default() -> Self {
        Self::new()
    }
}

impl GammaFunction for GammaPower {
    // agg_gamma_functions.h:41-44
    // double operator() (double x) const
    // {
    //     return pow(x, m_gamma);
    // }
    fn call(&self, x: f64) -> f64 {
        x.powf(self.m_gamma)
    }
}

// agg_gamma_functions.h:51-68  class gamma_threshold
#[derive(Debug, Clone, Copy)]
pub struct GammaThreshold {
    // agg_gamma_functions.h:67  double m_threshold;
    m_threshold: f64,
}

impl GammaThreshold {
    // agg_gamma_functions.h:55  gamma_threshold() : m_threshold(0.5) {}
    pub fn new() -> Self {
        Self { m_threshold: 0.5 }
    }

    // agg_gamma_functions.h:56  gamma_threshold(double t) : m_threshold(t) {}
    pub fn new_with(t: f64) -> Self {
        Self { m_threshold: t }
    }

    // agg_gamma_functions.h:58  void threshold(double t) { m_threshold = t; }
    pub fn set_threshold(&mut self, t: f64) {
        self.m_threshold = t;
    }

    // agg_gamma_functions.h:59  double threshold() const { return m_threshold; }
    pub fn threshold(&self) -> f64 {
        self.m_threshold
    }
}

impl Default for GammaThreshold {
    fn default() -> Self {
        Self::new()
    }
}

impl GammaFunction for GammaThreshold {
    // agg_gamma_functions.h:61-64
    // double operator() (double x) const
    // {
    //     return (x < m_threshold) ? 0.0 : 1.0;
    // }
    fn call(&self, x: f64) -> f64 {
        if x < self.m_threshold {
            0.0
        } else {
            1.0
        }
    }
}

// agg_gamma_functions.h:71-94  class gamma_linear
#[derive(Debug, Clone, Copy)]
pub struct GammaLinear {
    // agg_gamma_functions.h:92-93  double m_start; double m_end;
    m_start: f64,
    m_end: f64,
}

impl GammaLinear {
    // agg_gamma_functions.h:75  gamma_linear() : m_start(0.0), m_end(1.0) {}
    pub fn new() -> Self {
        Self {
            m_start: 0.0,
            m_end: 1.0,
        }
    }

    // agg_gamma_functions.h:76  gamma_linear(double s, double e) : m_start(s), m_end(e) {}
    pub fn new_with(s: f64, e: f64) -> Self {
        Self {
            m_start: s,
            m_end: e,
        }
    }

    // agg_gamma_functions.h:78  void set(double s, double e) { m_start = s; m_end = e; }
    pub fn set(&mut self, s: f64, e: f64) {
        self.m_start = s;
        self.m_end = e;
    }

    // agg_gamma_functions.h:79  void start(double s) { m_start = s; }
    pub fn set_start(&mut self, s: f64) {
        self.m_start = s;
    }

    // agg_gamma_functions.h:80  void end(double e) { m_end = e; }
    pub fn set_end(&mut self, e: f64) {
        self.m_end = e;
    }

    // agg_gamma_functions.h:81  double start() const { return m_start; }
    pub fn start(&self) -> f64 {
        self.m_start
    }

    // agg_gamma_functions.h:82  double end() const { return m_end; }
    pub fn end(&self) -> f64 {
        self.m_end
    }
}

impl Default for GammaLinear {
    fn default() -> Self {
        Self::new()
    }
}

impl GammaFunction for GammaLinear {
    // agg_gamma_functions.h:84-89
    // double operator() (double x) const
    // {
    //     if(x < m_start) return 0.0;
    //     if(x > m_end) return 1.0;
    //     return (x - m_start) / (m_end - m_start);
    // }
    fn call(&self, x: f64) -> f64 {
        if x < self.m_start {
            return 0.0;
        }
        if x > self.m_end {
            return 1.0;
        }
        (x - self.m_start) / (self.m_end - self.m_start)
    }
}

// agg_gamma_functions.h:97-116  class gamma_multiply
#[derive(Debug, Clone, Copy)]
pub struct GammaMultiply {
    // agg_gamma_functions.h:115  double m_mul;
    m_mul: f64,
}

impl GammaMultiply {
    // agg_gamma_functions.h:101  gamma_multiply() : m_mul(1.0) {}
    pub fn new() -> Self {
        Self { m_mul: 1.0 }
    }

    // agg_gamma_functions.h:102  gamma_multiply(double v) : m_mul(v) {}
    pub fn new_with(v: f64) -> Self {
        Self { m_mul: v }
    }

    // agg_gamma_functions.h:104  void value(double v) { m_mul = v; }
    pub fn set_value(&mut self, v: f64) {
        self.m_mul = v;
    }

    // agg_gamma_functions.h:105  double value() const { return m_mul; }
    pub fn value(&self) -> f64 {
        self.m_mul
    }
}

impl Default for GammaMultiply {
    fn default() -> Self {
        Self::new()
    }
}

impl GammaFunction for GammaMultiply {
    // agg_gamma_functions.h:107-112
    // double operator() (double x) const
    // {
    //     double y = x * m_mul;
    //     if(y > 1.0) y = 1.0;
    //     return y;
    // }
    fn call(&self, x: f64) -> f64 {
        let mut y = x * self.m_mul;
        if y > 1.0 {
            y = 1.0;
        }
        y
    }
}

// agg_gamma_functions.h:118-121
// inline double sRGB_to_linear(double x)
// {
//     return (x <= 0.04045) ? (x / 12.92) : pow((x + 0.055) / (1.055), 2.4);
// }
#[inline]
pub fn srgb_to_linear(x: f64) -> f64 {
    if x <= 0.04045 {
        x / 12.92
    } else {
        ((x + 0.055) / 1.055).powf(2.4)
    }
}

// agg_gamma_functions.h:123-126
// inline double linear_to_sRGB(double x)
// {
//     return (x <= 0.0031308) ? (x * 12.92) : (1.055 * pow(x, 1 / 2.4) - 0.055);
// }
#[inline]
pub fn linear_to_srgb(x: f64) -> f64 {
    if x <= 0.0031308 {
        x * 12.92
    } else {
        1.055 * x.powf(1.0 / 2.4) - 0.055
    }
}
