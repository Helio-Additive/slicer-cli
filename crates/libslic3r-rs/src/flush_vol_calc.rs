//! Flush volume calculator.
//!
//! Faithful 1:1 port of BambuStudio's `FlushVolCalc.{hpp,cpp}`.
//! Calculates the amount of filament to flush during tool changes.
//!
//! Precision note: the C++ helpers mix `float` and `double` arithmetic via the
//! implicit promotions of unqualified `cos`/`sin` (which bind to the C library
//! `double` overloads) versus `std::cos`/`std::sin` (the `float` overloads) and
//! `double` literals. Those promotions are reproduced exactly below for
//! byte-exact parity.

// FlushVolCalc.cpp:1-5 (includes)
use crate::color_space_convert::rgb2hsv; // RGB2HSV from ColorSpaceConvert.hpp
use crate::flush_vol_predictor::flush_predict::RGBColor;
use crate::flush_vol_predictor::GenericFlushPredictor;

// FlushVolCalc.cpp:8 namespace Slic3r

// FlushVolCalc.cpp:10
pub const G_MIN_FLUSH_VOLUME_FROM_SUPPORT: i32 = 700;
// FlushVolCalc.cpp:11
pub const G_FLUSH_VOLUME_TO_SUPPORT: i32 = 230;

// FlushVolCalc.cpp:13
pub const G_MAX_FLUSH_VOLUME: i32 = 900;

// FlushVolCalc.cpp:15-18
fn to_radians(degree: f32) -> f32 {
    degree / 180.0 * std::f32::consts::PI
}

// FlushVolCalc.cpp:21-24
fn get_luminance(r: f32, g: f32, b: f32) -> f32 {
    // C++: r * 0.3 + g * 0.59 + b * 0.11 with double literals -> computed in
    // double, narrowed to float on return.
    (r as f64 * 0.3 + g as f64 * 0.59 + b as f64 * 0.11) as f32
}

// FlushVolCalc.cpp:26-29
fn calc_triangle_3rd_edge(edge_a: f32, edge_b: f32, degree_ab: f32) -> f32 {
    (edge_a * edge_a + edge_b * edge_b
        - 2.0 * edge_a * edge_b * to_radians(degree_ab).cos())
    .sqrt()
}

// FlushVolCalc.cpp:31-40
#[allow(clippy::too_many_arguments)]
fn delta_hs_bbs(h1: f32, s1: f32, v1: f32, h2: f32, s2: f32, v2: f32) -> f32 {
    // FlushVolCalc.cpp:33-34
    let h1_rad = to_radians(h1);
    let h2_rad = to_radians(h2);

    // FlushVolCalc.cpp:36
    // std::cos(h1_rad) -> float overload; cos(h2_rad) -> double C overload.
    let dx = (h1_rad.cos() * s1 * v1) as f64
        - (h2_rad as f64).cos() * s2 as f64 * v2 as f64;
    let dx = dx as f32;
    // FlushVolCalc.cpp:37
    let dy = (h1_rad.sin() * s1 * v1) as f64
        - (h2_rad as f64).sin() * s2 as f64 * v2 as f64;
    let dy = dy as f32;
    // FlushVolCalc.cpp:38
    let dxy = (dx * dx + dy * dy).sqrt();
    // FlushVolCalc.cpp:39
    1.2f32.min(dxy)
}

// FlushVolCalc.hpp:15-37 (class FlushVolCalculator)
pub struct FlushVolCalculator {
    // FlushVolCalc.hpp:33
    m_min_flush_vol: i32,
    // FlushVolCalc.hpp:34
    m_max_flush_vol: i32,
    // FlushVolCalc.hpp:35
    #[allow(dead_code)]
    m_multiplier: f32,
    // FlushVolCalc.hpp:36
    m_flush_dataset: i32,
}

impl FlushVolCalculator {
    // FlushVolCalc.cpp:42-45
    pub fn new(min: i32, max: i32, flush_dataset: i32, multiplier: f32) -> Self {
        FlushVolCalculator {
            m_min_flush_vol: min,
            m_max_flush_vol: max,
            m_multiplier: multiplier,
            m_flush_dataset: flush_dataset,
        }
    }

    // FlushVolCalc.hpp:18 (multiplier = 1.0f default)
    pub fn with_default_multiplier(min: i32, max: i32, flush_dataset: i32) -> Self {
        FlushVolCalculator::new(min, max, flush_dataset, 1.0)
    }

    // FlushVolCalc.cpp:47-55
    #[allow(clippy::too_many_arguments)]
    pub fn get_flush_vol_from_data(
        &self,
        src_r: u8,
        src_g: u8,
        src_b: u8,
        dst_r: u8,
        dst_g: u8,
        dst_b: u8,
        flush: &mut f32,
    ) -> bool {
        // FlushVolCalc.cpp:50-52
        let pd = GenericFlushPredictor::new(self.m_flush_dataset);
        let src = RGBColor::new(src_r, src_g, src_b);
        let dst = RGBColor::new(dst_r, dst_g, dst_b);

        // FlushVolCalc.cpp:54
        pd.predict(&src, &dst, flush)
    }

    // FlushVolCalc.cpp:57-99
    pub fn calc_flush_vol_rgb(
        &self,
        src_r: u8,
        src_g: u8,
        src_b: u8,
        dst_r: u8,
        dst_g: u8,
        dst_b: u8,
    ) -> i32 {
        // FlushVolCalc.cpp:60-61
        let mut flush_volume: f32 = 0.0;
        if self.m_flush_dataset == 0
            && self.get_flush_vol_from_data(
                src_r,
                src_g,
                src_b,
                dst_r,
                dst_g,
                dst_b,
                &mut flush_volume,
            )
        {
            // FlushVolCalc.cpp:62
            return flush_volume as i32;
        }
        // FlushVolCalc.cpp:63-65
        let src_r_f: f32;
        let src_g_f: f32;
        let src_b_f: f32;
        let dst_r_f: f32;
        let dst_g_f: f32;
        let dst_b_f: f32;
        let mut from_hsv_h: f32 = 0.0;
        let mut from_hsv_s: f32 = 0.0;
        let mut from_hsv_v: f32 = 0.0;
        let mut to_hsv_h: f32 = 0.0;
        let mut to_hsv_s: f32 = 0.0;
        let mut to_hsv_v: f32 = 0.0;

        // FlushVolCalc.cpp:67-72
        src_r_f = src_r as f32 / 255.0;
        src_g_f = src_g as f32 / 255.0;
        src_b_f = src_b as f32 / 255.0;
        dst_r_f = dst_r as f32 / 255.0;
        dst_g_f = dst_g as f32 / 255.0;
        dst_b_f = dst_b as f32 / 255.0;

        // Calculate color distance in HSV color space
        // FlushVolCalc.cpp:75-76
        rgb2hsv(
            src_r_f,
            src_g_f,
            src_b_f,
            &mut from_hsv_h,
            &mut from_hsv_s,
            &mut from_hsv_v,
        );
        rgb2hsv(
            dst_r_f,
            dst_g_f,
            dst_b_f,
            &mut to_hsv_h,
            &mut to_hsv_s,
            &mut to_hsv_v,
        );
        // FlushVolCalc.cpp:77
        let mut hs_dist = delta_hs_bbs(
            from_hsv_h, from_hsv_s, from_hsv_v, to_hsv_h, to_hsv_s, to_hsv_v,
        );

        // 1. Color difference is more obvious if the dest color has high luminance
        // 2. Color difference is more obvious if the source color has low luminance
        // FlushVolCalc.cpp:81-82
        let from_lumi = get_luminance(src_r_f, src_g_f, src_b_f);
        let to_lumi = get_luminance(dst_r_f, dst_g_f, dst_b_f);
        // FlushVolCalc.cpp:83
        let lumi_flush: f32;
        // FlushVolCalc.cpp:84-92
        if to_lumi >= from_lumi {
            lumi_flush = (to_lumi - from_lumi).powf(0.7) * 560.0;
        } else {
            lumi_flush = (from_lumi - to_lumi) * 80.0;

            // FlushVolCalc.cpp:90: double literals -> computed in double, narrowed to float.
            let inter_hsv_v = (0.67 * to_hsv_v as f64 + 0.33 * from_hsv_v as f64) as f32;
            hs_dist = inter_hsv_v.min(hs_dist);
        }
        // FlushVolCalc.cpp:93
        let hs_flush = 230.0 * hs_dist;

        // FlushVolCalc.cpp:95
        flush_volume = calc_triangle_3rd_edge(hs_flush, lumi_flush, 120.0);
        // FlushVolCalc.cpp:96
        flush_volume = flush_volume.max(60.0);

        // FlushVolCalc.cpp:98
        flush_volume as i32
    }

    // FlushVolCalc.cpp:101-128
    #[allow(clippy::too_many_arguments)]
    pub fn calc_flush_vol(
        &self,
        src_a: u8,
        mut src_r: u8,
        mut src_g: u8,
        mut src_b: u8,
        dst_a: u8,
        mut dst_r: u8,
        mut dst_g: u8,
        mut dst_b: u8,
    ) -> i32 {
        // BBS: Transparent materials are treated as white materials
        // FlushVolCalc.cpp:105-110
        if src_a == 0 {
            src_r = 255;
            src_g = 255;
            src_b = 255;
        }
        if dst_a == 0 {
            dst_r = 255;
            dst_g = 255;
            dst_b = 255;
        }

        // FlushVolCalc.cpp:112-114
        let mut flush_volume: f32 = 0.0;
        if self.m_flush_dataset != 0
            && self.get_flush_vol_from_data(
                src_r,
                src_g,
                src_b,
                dst_r,
                dst_g,
                dst_b,
                &mut flush_volume,
            )
        {
            return (flush_volume as i32).min(self.m_max_flush_vol);
        }

        // FlushVolCalc.cpp:117
        flush_volume = self.calc_flush_vol_rgb(src_r, src_g, src_b, dst_r, dst_g, dst_b) as f32;

        // FlushVolCalc.cpp:119-120
        const DARK_COLOR_THRES: f32 = 180.0 / 255.0;
        const LIGHT_COLOR_THRES: f32 = 75.0 / 255.0;
        // FlushVolCalc.cpp:121-122
        // NOTE: C++ passes the raw `unsigned char` channels (0..255) to
        // get_luminance here, not the normalized [0,1] floats. The thresholds
        // (~0.706 and ~0.294) are therefore effectively never exceeded by the
        // 0..255-scaled luminance check below being < light_color_thres; this
        // surprising behavior is preserved verbatim from the C++ source.
        let is_from_dark = get_luminance(src_r as f32, src_g as f32, src_b as f32) > DARK_COLOR_THRES;
        let is_to_light = get_luminance(dst_r as f32, dst_g as f32, dst_b as f32) < LIGHT_COLOR_THRES;
        // FlushVolCalc.cpp:123-124
        if self.m_flush_dataset != 0 && is_from_dark && is_to_light {
            // C++: flush_volume *= 1.3 (double literal) -> double, narrowed to float.
            flush_volume = (flush_volume as f64 * 1.3) as f32;
        }

        // FlushVolCalc.cpp:126
        flush_volume += self.m_min_flush_vol as f32;
        // FlushVolCalc.cpp:127
        (flush_volume as i32).min(self.m_max_flush_vol)
    }
}
