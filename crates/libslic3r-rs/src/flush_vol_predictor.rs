//! Flush volume predictor.
//!
//! Faithful 1:1 port of BambuStudio's `FlushVolPredictor.{hpp,cpp}`.
//! This module is a dependency of `flush_vol_calc.rs` (`GenericFlushPredictor`).

// FlushVolPredictor.cpp:1-6 (includes)
use crate::utils::resources_dir;
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader};

// FlushVolPredictor.cpp:8 namespace FlushPredict
pub mod flush_predict {
    // FlushVolPredictor.hpp:11-18
    #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
    pub struct RGBColor {
        pub r: u8,
        pub g: u8,
        pub b: u8,
    }

    impl RGBColor {
        // FlushVolPredictor.hpp:16
        pub fn new(r_: u8, g_: u8, b_: u8) -> Self {
            // FlushVolPredictor.hpp:16
            RGBColor { r: r_, g: g_, b: b_ }
        }
    }

    // FlushVolPredictor.hpp:20-27
    #[derive(Debug, Clone, Copy, Default)]
    pub struct LABColor {
        pub l: f64,
        pub a: f64,
        pub b: f64,
    }

    impl LABColor {
        // FlushVolPredictor.hpp:26
        pub fn new(l_: f64, a_: f64, b_: f64) -> Self {
            // FlushVolPredictor.hpp:26
            LABColor {
                l: l_,
                a: a_,
                b: b_,
            }
        }
    }

    // FlushVolPredictor.cpp:10-12 (static helper; unused in C++ source too)
    #[allow(dead_code)]
    fn rad_to_deg(rad: f64) -> f64 {
        180.0 / std::f64::consts::PI * rad
    }

    // FlushVolPredictor.cpp:14-16
    fn deg_to_rad(deg: f64) -> f64 {
        deg * std::f64::consts::PI / 180.0
    }

    // transfer colour in RGB space to LAB space
    // FlushVolPredictor.cpp:17-61
    pub fn rgb2_lab(color: &RGBColor) -> LABColor {
        // FlushVolPredictor.cpp:19-25
        let gamma = |x: f64| -> f64 {
            if x > 0.04045 {
                ((x + 0.055) / 1.055).powf(2.4)
            } else {
                x / 12.92
            }
        };
        // FlushVolPredictor.cpp:26-34
        let rgb2xyz = |color: &RGBColor| -> (f64, f64, f64) {
            let r = gamma(color.r as f64 / 255.0) * 100.0;
            let g = gamma(color.g as f64 / 255.0) * 100.0;
            let b = gamma(color.b as f64 / 255.0) * 100.0;

            let x = 0.412453 * r + 0.357580 * g + 0.180423 * b;
            let y = 0.212671 * r + 0.715160 * g + 0.072169 * b;
            let z = 0.019334 * r + 0.119193 * g + 0.950227 * b;
            (x, y, z)
        };

        // FlushVolPredictor.cpp:36-38
        const XN: f64 = 95.0489;
        const YN: f64 = 100.0;
        const ZN: f64 = 108.8840;

        // FlushVolPredictor.cpp:40-46
        let f = |t: f64| -> f64 {
            const THRESHOLD: f64 = 0.008856f32 as f64;
            if t > THRESHOLD {
                t.powf(1.0 / 3.0)
            } else {
                7.787 * t + 0.137931
            }
        };

        // FlushVolPredictor.cpp:48-60
        let xyz_color = rgb2xyz(color);
        let x = xyz_color.0;
        let y = xyz_color.1;
        let z = xyz_color.2;
        let xn = f(x / XN);
        let yn = f(y / YN);
        let zn = f(z / ZN);

        let l = 116.0 * yn - 16.0;
        let a = 500.0 * (xn - yn);
        let b = 200.0 * (yn - zn);
        LABColor::new(l, a, b)
    }

    // calculate DeltaE2000
    // FlushVolPredictor.cpp:63-151
    pub fn calc_color_distance_lab(lab1: &LABColor, lab2: &LABColor) -> f32 {
        // FlushVolPredictor.cpp:65
        let pow_25_to_7: f64 = (25.0f64).powi(7);

        // FlushVolPredictor.cpp:67-78
        let c1 = (lab1.a * lab1.a + lab1.b * lab1.b).sqrt();
        let c2 = (lab2.a * lab2.a + lab2.b * lab2.b).sqrt();
        let c_mean = (c1 + c2) / 2.0;
        let pow_c_mean_to_7 = c_mean.powi(7);
        let g = 0.5 * (1.0 - (pow_c_mean_to_7 / (pow_c_mean_to_7 + pow_25_to_7)).sqrt());

        let p_l1 = lab1.l;
        let p_l2 = lab2.l;
        let p_a1 = (1.0 + g) * lab1.a;
        let p_a2 = (1.0 + g) * lab2.a;
        let p_b1 = lab1.b;
        let p_b2 = lab2.b;
        let p_c1 = (p_a1 * p_a1 + p_b1 * p_b1).sqrt();
        let p_c2 = (p_a2 * p_a2 + p_b2 * p_b2).sqrt();
        // FlushVolPredictor.cpp:80-87
        let p_h1: f64;
        if p_a1 == 0.0 && p_b1 == 0.0 {
            p_h1 = 0.0;
        } else {
            // C++: atan2(p_b1, p_a1); Rust's f64::atan2 is self.atan2(other) == atan2(self, other)
            let mut h = p_b1.atan2(p_a1);
            if h < 0.0 {
                h += std::f64::consts::PI * 2.0;
            }
            p_h1 = h;
        }
        // FlushVolPredictor.cpp:88-96
        let p_h2: f64;
        if p_a2 == 0.0 && p_b2 == 0.0 {
            p_h2 = 0.0;
        } else {
            let mut h = p_b2.atan2(p_a2);
            if h < 0.0 {
                h += std::f64::consts::PI * 2.0;
            }
            p_h2 = h;
        }

        // FlushVolPredictor.cpp:98-99
        let delta_l = p_l2 - p_l1;
        let delta_c = p_c2 - p_c1;

        // FlushVolPredictor.cpp:101-112
        let delta_h: f64;
        let p_c_multi = p_c1 * p_c2;
        if p_c_multi == 0.0 {
            delta_h = 0.0;
        } else {
            let mut dh = p_h2 - p_h1;
            if dh < -std::f64::consts::PI {
                dh += 2.0 * std::f64::consts::PI;
            } else if dh > std::f64::consts::PI {
                dh -= 2.0 * std::f64::consts::PI;
            }
            delta_h = 2.0 * p_c_multi.sqrt() * (dh / 2.0).sin();
        }

        // FlushVolPredictor.cpp:115-116
        let p_l_mean = (p_l1 + p_l2) / 2.0;
        let p_c_mean = (p_c1 + p_c2) / 2.0;

        // FlushVolPredictor.cpp:118-131
        let p_h_mean: f64;
        let p_h_sum = p_h1 + p_h2;
        if p_c1 * p_c2 == 0.0 {
            p_h_mean = p_h_sum;
        } else if (p_h1 - p_h2).abs() <= std::f64::consts::PI {
            p_h_mean = p_h_sum / 2.0;
        } else if p_h_sum < 2.0 * std::f64::consts::PI {
            p_h_mean = (p_h_sum + 2.0 * std::f64::consts::PI) / 2.0;
        } else {
            p_h_mean = (p_h_sum - 2.0 * std::f64::consts::PI) / 2.0;
        }

        // FlushVolPredictor.cpp:133
        let t = 1.0 - 0.17 * (p_h_mean - deg_to_rad(30.0)).cos()
            + 0.24 * (2.0 * p_h_mean).cos()
            + 0.32 * (3.0 * p_h_mean + deg_to_rad(6.0)).cos()
            - 0.2 * (4.0 * p_h_mean - deg_to_rad(63.0)).cos();
        // FlushVolPredictor.cpp:134
        let dtheta = deg_to_rad(30.0)
            * (-((p_h_mean - deg_to_rad(275.0)) / deg_to_rad(25.0)).powi(2)).exp();

        // FlushVolPredictor.cpp:136-137
        let pow_p_cmean_to_7 = p_c_mean.powi(7);
        let r_c = 2.0 * (pow_p_cmean_to_7 / (pow_p_cmean_to_7 + pow_25_to_7)).sqrt();

        // FlushVolPredictor.cpp:139-143
        let pow_p_lmean_to_2 = (p_l_mean - 50.0).powi(2);
        let s_l = 1.0 + (0.015 * pow_p_lmean_to_2) / (20.0 + pow_p_lmean_to_2).sqrt();
        let s_c = 1.0 + 0.045 * p_c_mean;
        let s_h = 1.0 + 0.015 * p_c_mean * t;
        let r_t = -(2.0 * dtheta).sin() * r_c;

        // FlushVolPredictor.cpp:145
        let k_l = 1.0;
        let k_c = 1.0;
        let k_h = 1.0;

        // FlushVolPredictor.cpp:147-150
        let de = ((delta_l / (k_l * s_l)).powi(2)
            + (delta_c / (k_c * s_c)).powi(2)
            + (delta_h / (k_h * s_h)).powi(2)
            + (r_t * (delta_c / (k_c * s_c)) * (delta_h / (k_h * s_h))))
        .sqrt();
        de as f32
    }

    // FlushVolPredictor.cpp:153-157
    pub fn calc_color_distance(color1: &RGBColor, color2: &RGBColor) -> f32 {
        let lab1 = rgb2_lab(color1);
        let lab2 = rgb2_lab(color2);
        calc_color_distance_lab(&lab1, &lab2)
    }

    // check if DeltaE is within the threshold. We consider colors within the threshold to be the same
    // FlushVolPredictor.cpp:159-165
    pub fn is_similar_color(from: &RGBColor, to: &RGBColor, distance_threshold: f32) -> bool {
        let color_distance = calc_color_distance(from, to);
        if color_distance > distance_threshold {
            return false;
        }
        true
    }

    // FlushVolPredictor.hpp:34 default threshold
    pub const DEFAULT_DISTANCE_THRESHOLD: f32 = 5.0;
}

use flush_predict::RGBColor;

// FlushVolPredictor.cpp:169-182 (class FlushVolPredictor)
pub struct FlushVolPredictor {
    // FlushVolPredictor.cpp:179
    m_flush_map: HashMap<u64, f32>,
    // FlushVolPredictor.cpp:180
    m_colors: Vec<RGBColor>,
    // FlushVolPredictor.cpp:181
    m_valid: bool,
}

impl Default for FlushVolPredictor {
    // FlushVolPredictor.cpp:177 (FlushVolPredictor() = default)
    fn default() -> Self {
        FlushVolPredictor {
            m_flush_map: HashMap::new(),
            m_colors: Vec::new(),
            m_valid: false,
        }
    }
}

impl FlushVolPredictor {
    // FlushVolPredictor.cpp:184-189
    pub fn get_min_flush_volume(&self) -> i32 {
        if !self.m_valid {
            return i32::MAX;
        }
        // min over the flush map values
        let min = self
            .m_flush_map
            .values()
            .cloned()
            .fold(f32::INFINITY, f32::min);
        min as i32
    }

    // FlushVolPredictor.cpp:191-202
    fn generate_hash_key(from: &RGBColor, to: &RGBColor) -> u64 {
        let mut key: u64 = 0;
        key |= (from.r as u64) << 40;
        key |= (from.g as u64) << 32;
        key |= (from.b as u64) << 24;
        key |= (to.r as u64) << 16;
        key |= (to.g as u64) << 8;
        key |= to.b as u64;
        key
    }

    // FlushVolPredictor.cpp:204-269
    pub fn new(data_file: &str) -> Self {
        // FlushVolPredictor.cpp:206-220
        let rgb_hex_to_dec = |hexstr: &str, color: &mut RGBColor| -> bool {
            if hexstr.is_empty() || hexstr.len() != 7 || !hexstr.starts_with('#') {
                debug_assert!(false);
                color.r = 0;
                color.g = 0;
                color.b = 0;
                return false;
            }

            // FlushVolPredictor.cpp:213-217
            let hex_to_byte =
                |hex: &str| -> i32 { u32::from_str_radix(hex, 16).unwrap_or(0) as i32 };
            color.r = hex_to_byte(&hexstr[1..3]) as u8;
            color.g = hex_to_byte(&hexstr[3..5]) as u8;
            color.b = hex_to_byte(&hexstr[5..7]) as u8;
            true
        };

        let mut result = FlushVolPredictor::default();

        // FlushVolPredictor.cpp:222-227
        let file = match File::open(data_file) {
            Ok(f) => f,
            Err(_) => {
                result.m_valid = false;
                return result;
            }
        };
        let mut reader = BufReader::new(file);
        let mut line = String::new();
        // FlushVolPredictor.cpp:229-230
        line.clear();
        let _ = reader.read_line(&mut line); // skip color description line
        line.clear();
        let _ = reader.read_line(&mut line);
        // read and save color lists
        // FlushVolPredictor.cpp:232-244
        {
            for color in line.split_whitespace() {
                let mut c = RGBColor::default();
                if !rgb_hex_to_dec(color, &mut c) {
                    result.m_valid = false;
                    return result;
                }
                result.m_colors.push(c);
            }
        }
        // FlushVolPredictor.cpp:245
        line.clear();
        let _ = reader.read_line(&mut line); // skip colume name line
        // FlushVolPredictor.cpp:246-267
        loop {
            line.clear();
            let n = reader.read_line(&mut line).unwrap_or(0);
            if n == 0 {
                break;
            }
            let mut iter = line.split_whitespace();
            let rgb_from = iter.next();
            let rgb_to = iter.next();
            // C++:253 `iss >> rgb_from >> rgb_to >> value` — operator>> for float is
            // prefix-lenient (e.g. "12abc" extracts 12). Rust `parse::<f32>()` is strict
            // and rejects a trailing-garbage token. This only differs on malformed data
            // files (shipped flush_data_*.txt are well-formed), so behavior matches in
            // practice; both fail the same way when a token is missing.
            let value = iter.next().and_then(|s| s.parse::<f32>().ok());
            if let (Some(rgb_from), Some(rgb_to), Some(value)) = (rgb_from, rgb_to, value) {
                let mut from = RGBColor::default();
                let mut to = RGBColor::default();
                // transfer hex str to rgb format
                if !rgb_hex_to_dec(rgb_from, &mut from) {
                    result.m_valid = false;
                    return result;
                }
                if !rgb_hex_to_dec(rgb_to, &mut to) {
                    result.m_valid = false;
                    return result;
                }
                // generate hash key for two rgb color
                let key = FlushVolPredictor::generate_hash_key(&from, &to);
                result.m_flush_map.entry(key).or_insert(value);
            } else {
                result.m_valid = false;
                return result;
            }
        }
        result.m_valid = true;
        result
    }

    // FlushVolPredictor.cpp:271-302
    pub fn predict(&self, from: &RGBColor, to: &RGBColor, flush: &mut f32) -> bool {
        if !self.m_valid {
            return false;
        }

        // find similar colors in color list
        // FlushVolPredictor.cpp:276-290
        let mut similar_from: Option<RGBColor> = None;
        let mut similar_to: Option<RGBColor> = None;
        for color in &self.m_colors {
            if flush_predict::is_similar_color(color, from, flush_predict::DEFAULT_DISTANCE_THRESHOLD)
            {
                similar_from = Some(*color);
                break;
            }
        }
        for color in &self.m_colors {
            if flush_predict::is_similar_color(color, to, flush_predict::DEFAULT_DISTANCE_THRESHOLD) {
                similar_to = Some(*color);
                break;
            }
        }

        // `from` and `to` should have similar colors in list
        // FlushVolPredictor.cpp:292-294
        let (similar_from, similar_to) = match (similar_from, similar_to) {
            (Some(f), Some(t)) => (f, t),
            _ => return false,
        };

        // FlushVolPredictor.cpp:296-301
        let key = FlushVolPredictor::generate_hash_key(&similar_from, &similar_to);
        match self.m_flush_map.get(&key) {
            None => false,
            Some(v) => {
                *flush = *v;
                true
            }
        }
    }
}

// FlushVolPredictor.cpp:305: static std::unordered_map<int, FlushVolPredictor> predictor_instances;
// We mirror the C++ memoization of predictor instances keyed by dataset value.
use std::sync::Mutex;
use std::sync::OnceLock;

fn predictor_instances() -> &'static Mutex<HashMap<i32, std::sync::Arc<FlushVolPredictor>>> {
    static INSTANCES: OnceLock<Mutex<HashMap<i32, std::sync::Arc<FlushVolPredictor>>>> =
        OnceLock::new();
    INSTANCES.get_or_init(|| Mutex::new(HashMap::new()))
}

// FlushVolPredictor.hpp:40-49 (class GenericFlushPredictor)
pub struct GenericFlushPredictor {
    // FlushVolPredictor.hpp:48
    predictor: Option<std::sync::Arc<FlushVolPredictor>>,
}

impl GenericFlushPredictor {
    // FlushVolPredictor.cpp:307-323
    pub fn new(dataset_value: i32) -> Self {
        let mut instances = predictor_instances().lock().unwrap();
        // FlushVolPredictor.cpp:309-310
        if let Some(p) = instances.get(&dataset_value) {
            return GenericFlushPredictor {
                predictor: Some(p.clone()),
            };
        }
        // FlushVolPredictor.cpp:311-322
        let mut path = resources_dir();
        if dataset_value == 0 {
            path.push("flush/flush_data_standard.txt");
        } else if dataset_value == 1 {
            path.push("flush/flush_data_dual_standard.txt");
        } else if dataset_value == 2 {
            path.push("flush/flush_data_dual_highflow.txt");
        }
        let predictor = std::sync::Arc::new(FlushVolPredictor::new(&path.to_string_lossy()));
        instances.insert(dataset_value, predictor.clone());
        GenericFlushPredictor {
            predictor: Some(predictor),
        }
    }

    // FlushVolPredictor.cpp:326-330
    pub fn predict(&self, from: &RGBColor, to: &RGBColor, flush: &mut f32) -> bool {
        match &self.predictor {
            None => false,
            Some(p) => p.predict(from, to, flush),
        }
    }

    // FlushVolPredictor.cpp:332-337
    pub fn get_min_flush_volume(&self) -> i32 {
        match &self.predictor {
            None => i32::MAX,
            Some(p) => p.get_min_flush_volume(),
        }
    }
}
