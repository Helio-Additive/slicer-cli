//! Port of `ObjColorUtils.hpp` / `ObjColorUtils.cpp` from Bambu Studio's libslic3r (pin 56a57d0).
//!
//! - `QuantKMeans` (ObjColorUtils.hpp:19-334): the color quantization of an OBJ's colors (and of
//!   images), k-means in Lab space through OpenCV;
//! - `obj_color_deal_algo` (the CLI's OBJ color import, BambuStudio.cpp:1025);
//! - the standard 3MF color groups as a textured mesh: `standard_3mf_corner_color`,
//!   `volume_to_textured_mesh`, `append_textured_mesh`, `standard_3mf_to_textured_meshes`.
//!
//! `QuantKMeans` is written on OpenCV (`cv::Mat`, `cv::kmeans`, `cv::cvtColor`, `cv::theRNG`). The
//! module [`cv`] below is a port of the parts of OpenCV 4.13.0 it calls (the version slicer_cli
//! links): `cv::RNG`, `cv::kmeans` with k-means++ seeding, `cv::hal::normL2Sqr_`, the `cv::sum` of a
//! row of doubles, and `cv::cvtColor` of 8-bit pixels between RGB and Lab / HSV (`RGB2Lab_b`,
//! `Lab2RGBinteger`, `RGB2HSV_b`, `HSV2RGB_b`). A `cv::Mat` is a typed buffer here ([`cv::Mat8u`],
//! [`cv::Mat32f`]); the members `m_flatten_labels` (an N x 1 `CV_32S` Mat) and `m_centers8UC3` (a
//! K x 1 `CV_8UC3` Mat) are vectors of their elements.
//!
//! OpenCV builds the integer tables of its 8-bit Lab conversions (`sRGBGammaTab_b`,
//! `sRGBInvGammaTab_b`, `LabCbrtTab_b`, color_lab.cpp:1234-1305) with its own software floating
//! point (`softfloat` / `softdouble`), whose basic operations are IEEE ones; its `cbrt` is ported
//! ([`cv::softfloat_cbrt`]). FIDELITY-NOTE: its `pow` (softfloat.cpp `f64_pow`) is `f64::powf`
//! here. The conversions of every 8-bit color (RGB to Lab) and of every 8-bit Lab triple (Lab to
//! RGB), which go through every entry of those tables, are compared with OpenCV's by
//! `tests/obj_color_utils_native_parity.rs`.
//!
//! On ARM builds OpenCV hands the 8-bit RGB -> HSV conversion to its carotene HAL
//! ([`cv::carotene_rgb2hsv`]); elsewhere it runs `RGB2HSV_b` ([`cv::rgb2hsv_b`]).

use std::collections::{BTreeMap, HashMap};

use crate::color::RGBA;
use crate::format::obj::{TriangleColor, VolumeColorInfo};
use crate::model::{Model, ModelVolume};
use crate::texture_painting::TexturedMesh;

/// ObjColorUtils.hpp:16 `STANDARD_3MF_UNCOLORED_RGB`
pub const STANDARD_3MF_UNCOLORED_RGB: [usize; 3] = [128, 128, 128];

// =============================================================================================
// The parts of OpenCV 4.13.0 that QuantKMeans calls
// =============================================================================================

pub mod cv {
    /// An 8-bit `cv::Mat` (`CV_8UC<channels>`), row-major and continuous.
    #[derive(Debug, Clone, Default, PartialEq, Eq)]
    pub struct Mat8u {
        pub rows: usize,
        pub cols: usize,
        pub channels: usize,
        pub data: Vec<u8>,
    }

    impl Mat8u {
        /// `cv::Mat(rows, cols, CV_8UC<channels>)`: zero filled (OpenCV leaves it uninitialized).
        pub fn new(rows: usize, cols: usize, channels: usize) -> Self {
            Mat8u { rows, cols, channels, data: vec![0; rows * cols * channels] }
        }
        /// `at<cv::Vec<uchar, cn>>(i, j)` as a slice of the pixel's channels.
        pub fn px(&self, i: usize, j: usize) -> &[u8] {
            let o = (i * self.cols + j) * self.channels;
            &self.data[o..o + self.channels]
        }
        pub fn px_mut(&mut self, i: usize, j: usize) -> &mut [u8] {
            let o = (i * self.cols + j) * self.channels;
            &mut self.data[o..o + self.channels]
        }
        /// `at<cv::Vec3b>(i, 0)` of an N x 1 three channel Mat.
        pub fn vec3b(&self, i: usize) -> [u8; 3] {
            let p = self.px(i, 0);
            [p[0], p[1], p[2]]
        }
    }

    /// A float `cv::Mat` (`CV_32FC<channels>`), row-major and continuous.
    #[derive(Debug, Clone, Default, PartialEq)]
    pub struct Mat32f {
        pub rows: usize,
        pub cols: usize,
        pub channels: usize,
        pub data: Vec<f32>,
    }

    impl Mat32f {
        pub fn new(rows: usize, cols: usize, channels: usize) -> Self {
            Mat32f { rows, cols, channels, data: vec![0.0; rows * cols * channels] }
        }
        /// `ptr<float>(i)` of the Mat seen as `rows x (cols * channels)` single channel floats.
        pub fn row(&self, i: usize) -> &[f32] {
            let w = self.cols * self.channels;
            &self.data[i * w..(i + 1) * w]
        }
        pub fn row_mut(&mut self, i: usize) -> &mut [f32] {
            let w = self.cols * self.channels;
            &mut self.data[i * w..(i + 1) * w]
        }
    }

    // ---------------------------------------------------------------------------------------
    // core/operations.hpp:394-419 `cv::RNG`
    // ---------------------------------------------------------------------------------------

    /// `cv::RNG`: the multiply-with-carry generator of OpenCV.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct Rng {
        pub state: u64,
    }

    impl Default for Rng {
        /// operations.hpp:394 `RNG::RNG() { state = 0xffffffff; }`
        fn default() -> Self {
            Rng { state: 0xffff_ffff }
        }
    }

    impl Rng {
        /// operations.hpp:395 `RNG::RNG(uint64 _state)`
        pub fn new(state: u64) -> Self {
            Rng { state: if state != 0 { state } else { 0xffff_ffff } }
        }
        /// operations.hpp:415-419 `unsigned RNG::next()`
        pub fn next(&mut self) -> u32 {
            self.state = ((self.state as u32) as u64).wrapping_mul(4164903690u64).wrapping_add((self.state >> 32) as u32 as u64);
            self.state as u32
        }
        /// operations.hpp:402 `operator unsigned()`
        pub fn as_unsigned(&mut self) -> u32 {
            self.next()
        }
        /// operations.hpp:403 `operator float()`
        pub fn as_float(&mut self) -> f32 {
            self.next() as f32 * 2.3283064365386962890625e-10f32
        }
        /// operations.hpp:404 `operator double()`
        pub fn as_double(&mut self) -> f64 {
            let t = self.next();
            (((t as u64) << 32) | self.next() as u64) as f64 * 5.4210108624275221700372640043497e-20
        }
    }

    thread_local! {
        /// core/src/rand.cpp `cv::theRNG()`: the thread's generator.
        static THE_RNG: std::cell::Cell<Rng> = std::cell::Cell::new(Rng::default());
    }

    /// `cv::theRNG().state`
    pub fn the_rng_state() -> u64 {
        THE_RNG.with(|r| r.get().state)
    }

    /// `cv::theRNG().state = state`
    pub fn set_the_rng_state(state: u64) {
        THE_RNG.with(|r| r.set(Rng { state }));
    }

    // ---------------------------------------------------------------------------------------
    // core/src/norm.dispatch.cpp:154-179 `float normL2Sqr_(const float* a, const float* b, int n)`
    // ---------------------------------------------------------------------------------------

    /// `cv::hal::normL2Sqr_`: below four floats (one NEON vector) the SIMD loops do not run and the
    /// sum starts from the reduce of the zero accumulators; `d += t*t` is fused (OpenCV is built
    /// with clang's default `-ffp-contract=on`).
    pub fn norm_l2_sqr(a: &[f32], b: &[f32], n: usize) -> f32 {
        const VLANES: usize = 4;
        let mut j = 0usize;
        let mut d: f32;
        let mut v_d0 = [0.0f32; VLANES];
        let mut v_d1 = [0.0f32; VLANES];
        let mut v_d2 = [0.0f32; VLANES];
        let mut v_d3 = [0.0f32; VLANES];
        while j + 4 * VLANES <= n {
            for l in 0..VLANES {
                let t0 = a[j + l] - b[j + l];
                let t1 = a[j + VLANES + l] - b[j + VLANES + l];
                v_d0[l] = t0.mul_add(t0, v_d0[l]);
                let t2 = a[j + 2 * VLANES + l] - b[j + 2 * VLANES + l];
                v_d1[l] = t1.mul_add(t1, v_d1[l]);
                let t3 = a[j + 3 * VLANES + l] - b[j + 3 * VLANES + l];
                v_d2[l] = t2.mul_add(t2, v_d2[l]);
                v_d3[l] = t3.mul_add(t3, v_d3[l]);
            }
            j += 4 * VLANES;
        }
        // v_reduce_sum(v_add(v_add(v_add(v_d0, v_d1), v_d2), v_d3)): vaddvq_f32, pairwise
        let mut s = [0.0f32; VLANES];
        for l in 0..VLANES {
            s[l] = ((v_d0[l] + v_d1[l]) + v_d2[l]) + v_d3[l];
        }
        d = (s[0] + s[1]) + (s[2] + s[3]);
        while j < n {
            let t = a[j] - b[j];
            d = t.mul_add(t, d);
            j += 1;
        }
        d
    }

    /// core/src/sum.simd.hpp:297-357 `sum_<double, double>` of one channel (no SIMD for doubles):
    /// `cv::sum(Mat(Size(N, 1), CV_64F, ...))[0]`.
    pub fn sum_f64(src: &[f64]) -> f64 {
        let len = src.len();
        let mut i = 0usize;
        let mut s0 = 0.0f64;
        // CV_ENABLE_UNROLLED
        while i + 4 <= len {
            s0 += src[i] + src[i + 1] + src[i + 2] + src[i + 3];
            i += 4;
        }
        while i < len {
            s0 += src[i];
            i += 1;
        }
        s0
    }

    // ---------------------------------------------------------------------------------------
    // core/src/kmeans.cpp
    // ---------------------------------------------------------------------------------------

    /// `cv::TermCriteria`
    #[derive(Debug, Clone, Copy)]
    pub struct TermCriteria {
        pub type_: i32,
        pub max_count: i32,
        pub epsilon: f64,
    }
    impl TermCriteria {
        pub const COUNT: i32 = 1;
        pub const EPS: i32 = 2;
    }

    /// `cv::KMEANS_RANDOM_CENTERS` / `KMEANS_PP_CENTERS` / `KMEANS_USE_INITIAL_LABELS`
    pub const KMEANS_RANDOM_CENTERS: i32 = 0;
    pub const KMEANS_PP_CENTERS: i32 = 2;
    pub const KMEANS_USE_INITIAL_LABELS: i32 = 1;

    /// kmeans.cpp:55-60 `generateRandomCenter`
    fn generate_random_center(dims: usize, box_: &[[f32; 2]], center: &mut [f32], rng: &mut Rng) {
        let margin = 1.0f32 / dims as f32;
        for j in 0..dims {
            // `((float)rng*(1.f+margin*2.f)-margin)*(box[j][1] - box[j][0]) + box[j][0]`
            let r = rng.as_float();
            center[j] = r.mul_add(margin.mul_add(2.0, 1.0), -margin).mul_add(box_[j][1] - box_[j][0], box_[j][0]);
        }
    }

    /// kmeans.cpp:91-168 `generateCentersPP(data, out_centers, K, rng, trials)`: k-means++ seeding.
    fn generate_centers_pp(data: &Mat32f, out_centers: &mut Mat32f, k_count: usize, rng: &mut Rng, trials: i32) -> Result<(), String> {
        let dims = data.cols * data.channels;
        let n = data.rows;
        let mut centers: Vec<usize> = vec![0; k_count];
        let mut buf: Vec<f32> = vec![0.0; n * 3];
        // dist, tdist, tdist2: three thirds of the buffer, swapped by offset
        let (mut dist, mut tdist, mut tdist2) = (0usize, n, 2 * n);
        let mut sum0: f64 = 0.0;

        centers[0] = (rng.as_unsigned() % n as u32) as usize;

        for i in 0..n {
            buf[dist + i] = norm_l2_sqr(data.row(i), data.row(centers[0]), dims);
            sum0 += buf[dist + i] as f64;
        }

        for k in 1..k_count {
            let mut best_sum = f64::MAX;
            let mut best_center: i64 = -1;

            for _j in 0..trials {
                let mut p = rng.as_double() * sum0;
                let mut ci = 0usize;
                while ci < n - 1 {
                    p -= buf[dist + ci] as f64;
                    if p <= 0.0 {
                        break;
                    }
                    ci += 1;
                }

                // KMeansPPDistanceComputer (kmeans.cpp:62-89), per sample
                for i in 0..n {
                    let v = norm_l2_sqr(data.row(i), data.row(ci), dims);
                    let d = buf[dist + i];
                    buf[tdist2 + i] = if d < v { d } else { v };
                }
                let mut s: f64 = 0.0;
                let mut i = 0usize;
                // CV_ENABLE_UNROLLED: eight floats summed in float, then added
                while i + 7 < n {
                    let t = buf[tdist2 + i]
                        + buf[tdist2 + i + 1]
                        + buf[tdist2 + i + 2]
                        + buf[tdist2 + i + 3]
                        + buf[tdist2 + i + 4]
                        + buf[tdist2 + i + 5]
                        + buf[tdist2 + i + 6]
                        + buf[tdist2 + i + 7];
                    s += t as f64;
                    i += 8;
                }
                while i < n {
                    s += buf[tdist2 + i] as f64;
                    i += 1;
                }

                if s < best_sum {
                    best_sum = s;
                    best_center = ci as i64;
                    std::mem::swap(&mut tdist, &mut tdist2);
                }
            }
            if best_center < 0 {
                return Err("kmeans: can't update cluster center (check input for huge or NaN values)".to_string());
            }
            centers[k] = best_center as usize;
            sum0 = best_sum;
            std::mem::swap(&mut dist, &mut tdist);
        }

        for k in 0..k_count {
            let src = data.row(centers[k]).to_vec();
            let dst = out_centers.row_mut(k);
            dst[..dims].copy_from_slice(&src[..dims]);
        }
        Ok(())
    }

    /// kmeans.cpp:236-469 `double cv::kmeans(data, K, bestLabels, criteria, attempts, flags,
    /// centers)` with `theRNG()`: `data` is N samples of `cols * channels` floats; `best_labels`
    /// gets N labels and `centers_out` K rows of the sample dimension. Returns the compactness.
    ///
    /// The `parallel_for_` loops are run in order (each sample writes its own entries).
    pub fn kmeans(
        data0: &Mat32f,
        k_count: i32,
        best_labels: &mut Vec<i32>,
        mut criteria: TermCriteria,
        mut attempts: i32,
        flags: i32,
        centers_out: &mut Mat32f,
    ) -> Result<f64, String> {
        const SPP_TRIALS: i32 = 3;
        let isrow = data0.rows == 1;
        let n = if isrow { data0.cols } else { data0.rows };
        let dims = (if isrow { 1 } else { data0.cols }) * data0.channels;

        attempts = attempts.max(1);
        if k_count <= 0 {
            return Err("K > 0".to_string());
        }
        if (n as i64) < k_count as i64 {
            return Err("There can't be more clusters than elements".to_string());
        }
        let k_count = k_count as usize;

        let data = Mat32f { rows: n, cols: dims, channels: 1, data: data0.data.clone() };

        // _bestLabels.create(N, 1, CV_32S, -1, true)
        if best_labels.len() != n {
            *best_labels = vec![0; n];
        }
        let mut labels: Vec<i32>;
        if flags & KMEANS_USE_INITIAL_LABELS != 0 {
            labels = best_labels.clone();
            for i in 0..n {
                if labels[i] as u32 >= k_count as u32 {
                    return Err("(unsigned)_labels.at<int>(i) < (unsigned)K".to_string());
                }
            }
        } else {
            labels = vec![0; n];
        }

        let mut centers = Mat32f::new(k_count, dims, 1);
        let mut old_centers = Mat32f::new(k_count, dims, 1);
        let mut temp = Mat32f::new(1, dims, 1);
        let mut counters: Vec<i32> = vec![0; k_count];
        let mut dists: Vec<f64> = vec![0.0; n];

        if criteria.type_ & TermCriteria::EPS != 0 {
            criteria.epsilon = criteria.epsilon.max(0.0);
        } else {
            criteria.epsilon = f32::EPSILON as f64;
        }
        criteria.epsilon *= criteria.epsilon;

        if criteria.type_ & TermCriteria::COUNT != 0 {
            criteria.max_count = criteria.max_count.max(2).min(100);
        } else {
            criteria.max_count = 100;
        }

        if k_count == 1 {
            attempts = 1;
            criteria.max_count = 2;
        }

        let mut box_: Vec<[f32; 2]> = vec![[0.0, 0.0]; dims];
        if flags & KMEANS_PP_CENTERS == 0 {
            {
                let sample = data.row(0);
                for j in 0..dims {
                    box_[j] = [sample[j], sample[j]];
                }
            }
            for i in 1..n {
                let sample = data.row(i);
                for j in 0..dims {
                    let v = sample[j];
                    box_[j][0] = box_[j][0].min(v);
                    box_[j][1] = box_[j][1].max(v);
                }
            }
        }

        let mut best_compactness = f64::MAX;
        // `RNG& rng = theRNG();`: the thread's generator, written back below
        let mut rng = Rng { state: the_rng_state() };
        for a in 0..attempts {
            let mut compactness = 0.0f64;

            let mut iter = 0i32;
            loop {
                let mut max_center_shift = if iter == 0 { f64::MAX } else { 0.0 };

                std::mem::swap(&mut centers, &mut old_centers);

                if iter == 0 && (a > 0 || flags & KMEANS_USE_INITIAL_LABELS == 0) {
                    if flags & KMEANS_PP_CENTERS != 0 {
                        generate_centers_pp(&data, &mut centers, k_count, &mut rng, SPP_TRIALS)?;
                    } else {
                        for k in 0..k_count {
                            generate_random_center(dims, &box_, centers.row_mut(k), &mut rng);
                        }
                    }
                } else {
                    // compute centers
                    for v in centers.data.iter_mut() {
                        *v = 0.0;
                    }
                    for k in 0..k_count {
                        counters[k] = 0;
                    }

                    for i in 0..n {
                        let k = labels[i] as usize;
                        for j in 0..dims {
                            let s = data.data[i * dims + j];
                            centers.data[k * dims + j] += s;
                        }
                        counters[k] += 1;
                    }

                    for k in 0..k_count {
                        if counters[k] != 0 {
                            continue;
                        }

                        // if some cluster appeared to be empty then:
                        //   1. find the biggest cluster
                        //   2. find the farthest from the center point in the biggest cluster
                        //   3. exclude the farthest point from the biggest cluster and form a new 1-point cluster.
                        let mut max_k = 0usize;
                        for k1 in 1..k_count {
                            if counters[max_k] < counters[k1] {
                                max_k = k1;
                            }
                        }

                        let mut max_dist = 0.0f64;
                        let mut farthest_i: i64 = -1;
                        let scale = 1.0f32 / counters[max_k] as f32;
                        for j in 0..dims {
                            temp.data[j] = centers.data[max_k * dims + j] * scale;
                        }

                        for i in 0..n {
                            if labels[i] as usize != max_k {
                                continue;
                            }
                            let dist = norm_l2_sqr(data.row(i), &temp.data, dims) as f64;

                            if max_dist <= dist {
                                max_dist = dist;
                                farthest_i = i as i64;
                            }
                        }

                        counters[max_k] -= 1;
                        counters[k] += 1;
                        let farthest_i = farthest_i as usize;
                        labels[farthest_i] = k as i32;

                        for j in 0..dims {
                            let s = data.data[farthest_i * dims + j];
                            centers.data[max_k * dims + j] -= s;
                            centers.data[k * dims + j] += s;
                        }
                    }

                    for k in 0..k_count {
                        if counters[k] == 0 {
                            return Err("counters[k] != 0".to_string());
                        }

                        let scale = 1.0f32 / counters[k] as f32;
                        for j in 0..dims {
                            centers.data[k * dims + j] *= scale;
                        }

                        if iter > 0 {
                            let mut dist = 0.0f64;
                            for j in 0..dims {
                                let t = (centers.data[k * dims + j] - old_centers.data[k * dims + j]) as f64;
                                dist = t.mul_add(t, dist);
                            }
                            max_center_shift = max_center_shift.max(dist);
                        }
                    }
                }

                iter += 1;
                let is_last_iter = iter == criteria.max_count.max(2) || max_center_shift <= criteria.epsilon;

                if is_last_iter {
                    // don't re-assign labels to avoid creation of empty clusters
                    // KMeansDistanceComputer<true>
                    for i in 0..n {
                        dists[i] = norm_l2_sqr(data.row(i), centers.row(labels[i] as usize), dims) as f64;
                    }
                    compactness = sum_f64(&dists);
                    break;
                } else {
                    // assign labels: KMeansDistanceComputer<false>
                    for i in 0..n {
                        let sample = data.row(i);
                        let mut k_best = 0usize;
                        let mut min_dist = f64::MAX;

                        for k in 0..k_count {
                            let dist = norm_l2_sqr(sample, centers.row(k), dims) as f64;

                            if min_dist > dist {
                                min_dist = dist;
                                k_best = k;
                            }
                        }

                        dists[i] = min_dist;
                        labels[i] = k_best as i32;
                    }
                }
            }

            if compactness < best_compactness {
                best_compactness = compactness;
                // `centers.copyTo(_centers)`: K x dims
                *centers_out = centers.clone();
                best_labels.copy_from_slice(&labels);
            }
        }
        set_the_rng_state(rng.state);

        Ok(best_compactness)
    }

    // ---------------------------------------------------------------------------------------
    // imgproc/src/color_lab.cpp: the 8-bit RGB <-> Lab conversions (bit exact integer paths)
    // ---------------------------------------------------------------------------------------

    /// color.simd_helpers.hpp:5 `CV_DESCALE(x, n)`
    #[inline]
    fn cv_descale(x: i32, n: u32) -> i32 {
        (x + (1 << (n - 1))) >> n
    }

    /// `saturate_cast<uchar>(int)`
    #[inline]
    fn saturate_u8(v: i32) -> u8 {
        v.clamp(0, 255) as u8
    }

    /// `cvRound(softfloat)` / `cvRound(softdouble)`: to nearest, ties to even.
    #[inline]
    fn cv_round_f32(v: f32) -> i32 {
        v.round_ties_even() as i32
    }
    #[inline]
    fn cv_round_f64(v: f64) -> i32 {
        v.round_ties_even() as i32
    }

    const XYZ_SHIFT: u32 = 12;
    const LAB_SHIFT: u32 = XYZ_SHIFT;
    const GAMMA_SHIFT: u32 = 3;
    const LAB_SHIFT2: u32 = LAB_SHIFT + GAMMA_SHIFT;
    const LAB_CBRT_TAB_SIZE_B: usize = 256 * 3 / 2 * (1 << GAMMA_SHIFT);
    const INV_GAMMA_SHIFT: u32 = 12;
    const INV_GAMMA_TAB_SIZE: usize = 1 << INV_GAMMA_SHIFT;
    const LUT_BASE: i32 = 1 << 14;
    const LAB_BASE: i32 = 1 << 14;
    const MIN_AB_VALUE: i32 = -8145;

    /// color_lab.cpp:103-115 `sRGB2XYZ_D65`
    const SRGB2XYZ_D65: [u64; 9] = [
        0x3fda65a14488c60d,
        0x3fd6e297396d0918,
        0x3fc71819d2391d58,
        0x3fcb38cda6e75ff6,
        0x3fe6e297396d0918,
        0x3fb279aae6c8f755,
        0x3f93cc4ac6cdaf4b,
        0x3fbe836eb4e98138,
        0x3fee68427418d691,
    ];
    /// color_lab.cpp:119-130 `XYZ2sRGB_D65`
    const XYZ2SRGB_D65: [u64; 9] = [
        0x4009ec804102ff8f,
        0xbff8982a9930be0e,
        0xbfdfe7ff583a53b9,
        0xbfef042528ae74f3,
        0x3ffe040f23897204,
        0x3fa546d3f9e7b80b,
        0x3fac7de5082cf52c,
        0xbfca1e14bdfd2631,
        0x3ff0eabef06b3786,
    ];
    /// color_lab.cpp:941-943 `D65`
    fn d65() -> [f64; 3] {
        [f64::from_bits(0x3fee6a22b3892ee8), 1.0, f64::from_bits(0x3ff16b8950763a19)]
    }

    /// core/src/softfloat.cpp:3897-3930 `f32_cbrt`: a quartic rational polynomial in double over
    /// the mantissa scaled into [0.125, 1), the result mantissa truncated.
    pub fn softfloat_cbrt(x: f32) -> f32 {
        //special cases
        if x.is_nan() {
            return f32::NAN;
        }
        if x.is_infinite() {
            return x;
        }
        let v = x.to_bits();
        let s = (v >> 31) as u32;
        let mut ex: i32 = ((v >> 23) & 0xff) as i32 - 127;
        let mut shx = ex % 3;
        shx -= if shx >= 0 { 3 } else { 0 };
        ex = (ex - shx) / 3 - 1; /* exponent of cube root */
        // packToF64UI(0, shx + 1023, fracF32UI(x) << 29)
        let mut fr = f64::from_bits((((shx + 1023) as u64) << 52).wrapping_add(((v & 0x7f_ffff) as u64) << 29));

        /* 0.125 <= fr < 1.0 */
        /* Use quartic rational polynomial with error < 2^(-24) */
        let a1 = f64::from_bits(0x4046a09e6653ba70); //  45.2548339756803022511987494
        let a2 = f64::from_bits(0x406808f46c6116e0); // 192.2798368355061050458134625
        let a3 = f64::from_bits(0x405dca97439cae14); // 119.1654824285581628956914143
        let a4 = f64::from_bits(0x402add70d2827500); //  13.43250139086239872172837314
        let a5 = f64::from_bits(0x3fc4f15f83f55d2d); //   0.1636161226585754240958355063
        let a6 = f64::from_bits(0x402d9e20660edb21); //  14.80884093219134573786480845
        let a7 = f64::from_bits(0x4062ff15c0285815); // 151.9714051044435648658557668
        let a8 = f64::from_bits(0x406510d06a8112ce); // 168.5254414101568283957668343
        let a9 = f64::from_bits(0x4040fecbc9e2c375); //  33.9905941350215598754191872
        let a10 = 1.0f64;

        // softdouble operations: each rounded, none fused
        fr = ((((a1 * fr + a2) * fr + a3) * fr + a4) * fr + a5) / ((((a6 * fr + a7) * fr + a8) * fr + a9) * fr + a10);
        /* fr *= 2^ex * sign */

        // checks for "+0" and "-0", reset sign bit
        // packToF32UI(s, ex + 127, fracF64UI(fr) >> 29)
        let frac = ((fr.to_bits() & 0x000f_ffff_ffff_ffff) >> 29) as u32;
        let y = if (v & ((1u32 << 31) - 1)) != 0 { (s << 31).wrapping_add(((ex + 127) as u32) << 23).wrapping_add(frac) } else { 0 };
        f32::from_bits(y)
    }

    /// color_lab.cpp:1023-1030 `applyGamma(softfloat x)`
    fn apply_gamma(x: f32) -> f32 {
        let gamma_threshold = 809.0f64 / 20000.0;
        let gamma_low_scale = 323.0f64 / 25.0;
        let gamma_power = 12.0f64 / 5.0;
        let gamma_xshift = 11.0f64 / 200.0;
        let xd = x as f64;
        (if xd <= gamma_threshold { xd / gamma_low_scale } else { ((xd + gamma_xshift) / (1.0 + gamma_xshift)).powf(gamma_power) }) as f32
    }

    /// color_lab.cpp:1033-1040 `applyInvGamma(softfloat x)`
    fn apply_inv_gamma(x: f32) -> f32 {
        let gamma_inv_threshold = 7827.0f64 / 2500000.0;
        let gamma_low_scale = 323.0f64 / 25.0;
        let gamma_power = 12.0f64 / 5.0;
        let gamma_xshift = 11.0f64 / 200.0;
        let xd = x as f64;
        (if xd <= gamma_inv_threshold { xd * gamma_low_scale } else { xd.powf(1.0 / gamma_power) * (1.0 + gamma_xshift) - gamma_xshift }) as f32
    }

    /// The integer tables of color_lab.cpp:1234-1308 `createLabTabs()` the 8-bit conversions use.
    pub struct LabTabs {
        pub srgb_gamma_tab_b: [u16; 256],
        pub linear_gamma_tab_b: [u16; 256],
        pub srgb_inv_gamma_tab_b: Vec<u16>,
        pub linear_inv_gamma_tab_b: Vec<u16>,
        pub lab_cbrt_tab_b: Vec<u16>,
        pub lab_to_yf_b: [u16; 512],
        pub ab_to_xz_b: Vec<i32>,
    }

    fn create_lab_tabs() -> LabTabs {
        let f255 = 255.0f32;
        let lthresh = 216.0f32 / 24389.0; // 0.008856f = (6/29)^3
        let lscale = 841.0f32 / 108.0; // 7.787f = (29/3)^3/(29*4)
        let lbias = 16.0f32 / 116.0;

        let mut srgb_gamma_tab_b = [0u16; 256];
        let mut linear_gamma_tab_b = [0u16; 256];
        let int_scale = (255 * (1 << GAMMA_SHIFT)) as f32;
        for i in 0..256 {
            let x = i as f32 / f255;
            srgb_gamma_tab_b[i] = cv_round_f32(int_scale * apply_gamma(x)) as u16;
            linear_gamma_tab_b[i] = (i * (1 << GAMMA_SHIFT)) as u16;
        }
        let mut srgb_inv_gamma_tab_b = vec![0u16; INV_GAMMA_TAB_SIZE];
        let mut linear_inv_gamma_tab_b = vec![0u16; INV_GAMMA_TAB_SIZE];
        let inv_scale = 1.0f32 / INV_GAMMA_TAB_SIZE as f32;
        for i in 0..INV_GAMMA_TAB_SIZE {
            let x = inv_scale * i as f32;
            srgb_inv_gamma_tab_b[i] = cv_round_f32(f255 * apply_inv_gamma(x)) as u16;
            // cvTrunc
            linear_inv_gamma_tab_b[i] = ((f255 * x).trunc() as i32) as u16;
        }

        let cb_tab_scale = 1.0f32 / (f255 * (1 << GAMMA_SHIFT) as f32);
        let lshift2 = (1 << LAB_SHIFT2) as f32;
        let mut lab_cbrt_tab_b = vec![0u16; LAB_CBRT_TAB_SIZE_B];
        for i in 0..LAB_CBRT_TAB_SIZE_B {
            let x = cb_tab_scale * i as f32;
            let v = if x < lthresh { x.mul_add(lscale, lbias) } else { softfloat_cbrt(x) };
            lab_cbrt_tab_b[i] = cv_round_f32(lshift2 * v) as u16;
        }

        //Lookup table for L to y and ify calculations
        let mut lab_to_yf_b = [0u16; 512];
        for i in 0..256i32 {
            let (y, ify);
            //8 * 255.0 / 100.0 == 20.4
            if i <= 20 {
                //yy = li / 903.3f;
                //y = L*100/903.3f; 903.3f = (29/3)^3, 255 = 17*3*5
                y = cv_round_f32((i * LUT_BASE * 20 * 9) as f32 / (17 * 29 * 29 * 29) as f32);
                //fy = 7.787f * yy + 16.0f / 116.0f; 7.787f = (29/3)^3/(29*4)
                ify = cv_round_f32(LUT_BASE as f32 * (16.0f32 / 116.0f32 + (i * 5) as f32 / (3 * 17 * 29) as f32));
            } else {
                //fy = (li + 16.0f) / 116.0f;
                let fy = (i * 100 * LUT_BASE) as f32 / (255 * 116) as f32 + (16 * LUT_BASE) as f32 / 116.0f32;
                ify = cv_round_f32(fy);
                //yy = fy * fy * fy;
                y = cv_round_f32(fy * fy * fy / (LUT_BASE * LUT_BASE) as f32);
            }

            lab_to_yf_b[(i * 2) as usize] = y as u16; // 0 <= y <= BASE
            lab_to_yf_b[(i * 2 + 1) as usize] = ify as u16; // 2260 <= ify <= BASE
        }

        LabTabs {
            srgb_gamma_tab_b,
            linear_gamma_tab_b,
            srgb_inv_gamma_tab_b,
            linear_inv_gamma_tab_b,
            lab_cbrt_tab_b,
            lab_to_yf_b,
            ab_to_xz_b: init_lut_for_abxz(),
        }
    }

    /// color_lab.cpp:1086-1107 `initLUTforABXZ()`
    fn init_lut_for_abxz() -> Vec<i32> {
        let mut res = vec![0i32; (LAB_BASE * 9 / 4) as usize];
        for i in MIN_AB_VALUE..(LAB_BASE * 9 / 4 + MIN_AB_VALUE) {
            let v;
            //6.f/29.f*BASE = 3389.730
            if i <= 3390 {
                //fxz[k] = (fxz[k] - 16.0f / 116.0f) / 7.787f;
                // 7.787f = (29/3)^3/(29*4)
                v = i * 108 / 841 - LUT_BASE * 16 / 116 * 108 / 841;
            } else {
                //fxz[k] = fxz[k] * fxz[k] * fxz[k];
                v = i * i / LUT_BASE * i / LUT_BASE;
            }
            res[(i - MIN_AB_VALUE) as usize] = v; // -1335 <= v <= 88231
        }
        res
    }

    /// color_lab.cpp:1344-1348 `initLabTabs()`: computed once.
    pub fn lab_tabs() -> &'static LabTabs {
        static TABS: std::sync::OnceLock<LabTabs> = std::sync::OnceLock::new();
        TABS.get_or_init(create_lab_tabs)
    }

    /// color_lab.cpp:1573-1606 `RGB2Lab_b` (no custom coefficients or white point, sRGB): the
    /// coefficients.
    fn rgb2lab_b_coeffs(blue_idx: usize) -> [i32; 9] {
        let white_pt = d65();
        let lshift = (1 << LAB_SHIFT) as f64;
        let mut coeffs = [0i32; 9];
        for i in 0..3 {
            let c = [f64::from_bits(SRGB2XYZ_D65[i * 3]), f64::from_bits(SRGB2XYZ_D65[i * 3 + 1]), f64::from_bits(SRGB2XYZ_D65[i * 3 + 2])];
            coeffs[i * 3 + (blue_idx ^ 2)] = cv_round_f64(lshift * c[0] / white_pt[i]);
            coeffs[i * 3 + 1] = cv_round_f64(lshift * c[1] / white_pt[i]);
            coeffs[i * 3 + blue_idx] = cv_round_f64(lshift * c[2] / white_pt[i]);
        }
        coeffs
    }

    /// color_lab.cpp:1671-1887 `RGB2Lab_b::operator()` (the scalar loop; the NEON and SIMD loops
    /// compute the same integers): `n` pixels of `scn` channels.
    pub fn rgb2lab_b(src: &[u8], dst: &mut [u8], n: usize, scn: usize, blue_idx: usize, srgb: bool) {
        let tabs = lab_tabs();
        let coeffs = rgb2lab_b_coeffs(blue_idx);
        let lscale: i32 = (116 * 255 + 50) / 100;
        let lshift: i32 = -((16 * 255 * (1 << LAB_SHIFT2) + 50) / 100);
        let tab: &[u16] = if srgb { &tabs.srgb_gamma_tab_b } else { &tabs.linear_gamma_tab_b };
        let [c0, c1, c2, c3, c4, c5, c6, c7, c8] = coeffs;
        for i in 0..n {
            let s = &src[i * scn..];
            let r = tab[s[0] as usize] as i32;
            let g = tab[s[1] as usize] as i32;
            let b = tab[s[2] as usize] as i32;
            let fx = tabs.lab_cbrt_tab_b[cv_descale(r * c0 + g * c1 + b * c2, LAB_SHIFT) as usize] as i32;
            let fy = tabs.lab_cbrt_tab_b[cv_descale(r * c3 + g * c4 + b * c5, LAB_SHIFT) as usize] as i32;
            let fz = tabs.lab_cbrt_tab_b[cv_descale(r * c6 + g * c7 + b * c8, LAB_SHIFT) as usize] as i32;

            let l = cv_descale(lscale * fy + lshift, LAB_SHIFT2);
            let a = cv_descale(500 * (fx - fy) + 128 * (1 << LAB_SHIFT2), LAB_SHIFT2);
            let bb = cv_descale(200 * (fy - fz) + 128 * (1 << LAB_SHIFT2), LAB_SHIFT2);

            dst[i * 3] = saturate_u8(l);
            dst[i * 3 + 1] = saturate_u8(a);
            dst[i * 3 + 2] = saturate_u8(bb);
        }
    }

    /// color_lab.cpp:2399-2436 `Lab2RGBinteger` (no custom coefficients or white point): the
    /// coefficients.
    fn lab2rgb_integer_coeffs(blue_idx: usize) -> [i32; 9] {
        let white_pt = d65();
        let lshift = (1 << LAB_SHIFT) as f64;
        let mut coeffs = [0i32; 9];
        for i in 0..3 {
            let c = [f64::from_bits(XYZ2SRGB_D65[i]), f64::from_bits(XYZ2SRGB_D65[i + 3]), f64::from_bits(XYZ2SRGB_D65[i + 6])];
            coeffs[i + blue_idx * 3] = cv_round_f64(lshift * c[0] * white_pt[i]);
            coeffs[i + 3] = cv_round_f64(lshift * c[1] * white_pt[i]);
            coeffs[i + (blue_idx ^ 2) * 3] = cv_round_f64(lshift * c[2] * white_pt[i]);
        }
        coeffs
    }

    /// color_lab.cpp:2438-2491 `Lab2RGBinteger::process` and the scalar loop of its `operator()`
    /// (2675-2686; the SIMD loop computes the same integers); `Lab2RGB_b` runs it
    /// (`enableBitExactness`).
    pub fn lab2rgb_b(src: &[u8], dst: &mut [u8], n: usize, dcn: usize, blue_idx: usize, is_srgb: bool) {
        const BASE: i32 = 1 << 14;
        const SHIFT: u32 = LAB_SHIFT + (14 - INV_GAMMA_SHIFT);
        let tabs = lab_tabs();
        let coeffs = lab2rgb_integer_coeffs(blue_idx);
        let [c0, c1, c2, c3, c4, c5, c6, c7, c8] = coeffs;
        for i in 0..n {
            let (ll, aa, bb) = (src[i * 3] as i32, src[i * 3 + 1] as i32, src[i * 3 + 2] as i32);
            let y = tabs.lab_to_yf_b[(ll * 2) as usize] as i32;
            let ify = tabs.lab_to_yf_b[(ll * 2 + 1) as usize] as i32;

            //float fxz[] = { ai / 500.0f + fy, fy - bi / 200.0f };
            //adiv = aa*BASE/500 - 128*BASE/500, bdiv = bb*BASE/200 - 128*BASE/200;
            //approximations with reasonable precision
            let adiv = ((5 * aa * 53687 + (1 << 7)) >> 13) - 128 * BASE / 500;
            let bdiv = ((bb * 41943 + (1 << 4)) >> 9) - 128 * BASE / 200 + 1;

            let mut ifxz = [ify + adiv, ify - bdiv];
            for k in 0..2 {
                let v = &mut ifxz[k];
                *v = tabs.ab_to_xz_b[(*v - MIN_AB_VALUE) as usize];
            }
            let x = ifxz[0];
            let z = ifxz[1];

            let mut ro = cv_descale(c0 * x + c1 * y + c2 * z, SHIFT);
            let mut go = cv_descale(c3 * x + c4 * y + c5 * z, SHIFT);
            let mut bo = cv_descale(c6 * x + c7 * y + c8 * z, SHIFT);

            ro = ro.min(INV_GAMMA_TAB_SIZE as i32 - 1).max(0);
            go = go.min(INV_GAMMA_TAB_SIZE as i32 - 1).max(0);
            bo = bo.min(INV_GAMMA_TAB_SIZE as i32 - 1).max(0);

            if is_srgb {
                let tab = &tabs.srgb_inv_gamma_tab_b;
                ro = tab[ro as usize] as i32;
                go = tab[go as usize] as i32;
                bo = tab[bo as usize] as i32;
            } else {
                // rgb = (rgb*255) >> inv_gamma_shift
                ro = ((ro << 8) - ro) >> INV_GAMMA_SHIFT;
                go = ((go << 8) - go) >> INV_GAMMA_SHIFT;
                bo = ((bo << 8) - bo) >> INV_GAMMA_SHIFT;
            }

            let d = &mut dst[i * dcn..];
            d[0] = saturate_u8(bo);
            d[1] = saturate_u8(go);
            d[2] = saturate_u8(ro);
            if dcn == 4 {
                d[3] = 255;
            }
        }
    }

    // ---------------------------------------------------------------------------------------
    // imgproc/src/color_hsv.simd.hpp: the 8-bit RGB <-> HSV conversions
    // ---------------------------------------------------------------------------------------

    /// color_hsv.simd.hpp:61-86 `RGB2HSV_b::TablesSingleton`: `saturate_cast<int>(double)` rounds
    /// to nearest even.
    struct HsvTables {
        sdiv_table: [i32; 256],
        hdiv_table180: [i32; 256],
        hdiv_table256: [i32; 256],
    }

    fn hsv_tables() -> &'static HsvTables {
        static TABLES: std::sync::OnceLock<HsvTables> = std::sync::OnceLock::new();
        TABLES.get_or_init(|| {
            const HSV_SHIFT: i32 = 12;
            let mut t = HsvTables { sdiv_table: [0; 256], hdiv_table180: [0; 256], hdiv_table256: [0; 256] };
            for i in 1..256 {
                t.sdiv_table[i] = ((255 << HSV_SHIFT) as f64 / (1.0 * i as f64)).round_ties_even() as i32;
                t.hdiv_table180[i] = ((180 << HSV_SHIFT) as f64 / (6.0 * i as f64)).round_ties_even() as i32;
                t.hdiv_table256[i] = ((256 << HSV_SHIFT) as f64 / (6.0 * i as f64)).round_ties_even() as i32;
            }
            t
        })
    }

    /// color_hsv.simd.hpp:88-260 `RGB2HSV_b::operator()` (the scalar loop; the SIMD loop computes
    /// the same integers).
    pub fn rgb2hsv_b(src: &[u8], dst: &mut [u8], n: usize, scn: usize, blue_idx: usize, hrange: i32) {
        const HSV_SHIFT: i32 = 12;
        let tables = hsv_tables();
        let hdiv_table: &[i32; 256] = if hrange == 180 { &tables.hdiv_table180 } else { &tables.hdiv_table256 };
        let sdiv_table = &tables.sdiv_table;
        let hr = hrange;
        for i in 0..n {
            let s0 = &src[i * scn..];
            let b = s0[blue_idx] as i32;
            let g = s0[1] as i32;
            let r = s0[blue_idx ^ 2] as i32;
            let mut v = b;
            let mut vmin = b;

            // CV_CALC_MAX_8U / CV_CALC_MIN_8U
            v = v.max(g);
            v = v.max(r);
            vmin = vmin.min(g);
            vmin = vmin.min(r);

            let diff = saturate_u8(v - vmin) as i32;
            let vr: i32 = if v == r { -1 } else { 0 };
            let vg: i32 = if v == g { -1 } else { 0 };

            let s = (diff * sdiv_table[v as usize] + (1 << (HSV_SHIFT - 1))) >> HSV_SHIFT;
            let mut h = (vr & (g - b)) + (!vr & ((vg & (b - r + 2 * diff)) + ((!vg) & (r - g + 4 * diff))));
            h = (h * hdiv_table[diff as usize] + (1 << (HSV_SHIFT - 1))) >> HSV_SHIFT;
            h += if h < 0 { hr } else { 0 };

            dst[i * 3] = saturate_u8(h);
            dst[i * 3 + 1] = s as u8;
            dst[i * 3 + 2] = v as u8;
        }
    }

    /// The ARM `FRECPE` of a single (Arm ARM `FPRecipEstimate`, `RecipEstimate`; FPCR.AH = 0):
    /// an 8-bit estimate of the reciprocal.
    fn frecpe_f32(x: f32) -> f32 {
        if x.is_nan() {
            return f32::NAN;
        }
        let bits = x.to_bits();
        let sign = bits & 0x8000_0000;
        if x == 0.0 {
            return f32::from_bits(sign | 0x7f80_0000);
        }
        if x.is_infinite() || x.abs() >= 2.0f32.powi(126) {
            return f32::from_bits(sign);
        }
        let mut fraction: u64 = ((bits & 0x7f_ffff) as u64) << 29;
        let mut exp: i32 = ((bits >> 23) & 0xff) as i32;
        if exp == 0 {
            if fraction & (1u64 << 51) == 0 {
                exp = -1;
                fraction = (fraction & ((1u64 << 50) - 1)) << 2;
            } else {
                fraction = (fraction & ((1u64 << 51) - 1)) << 1;
            }
        }
        let scaled: u32 = 0x100 | ((fraction >> 44) & 0xff) as u32;
        let mut result_exp: i32 = 253 - exp;
        // RecipEstimate(a): a = a*2+1; b = 2^19 DIV a; r = (b+1) DIV 2
        let a = scaled * 2 + 1;
        let b = (1u32 << 19) / a;
        let estimate = (b + 1) / 2;
        let mut fraction: u64 = ((estimate & 0xff) as u64) << 44;
        if result_exp == 0 {
            fraction = (1u64 << 51) | (fraction >> 1);
        } else if result_exp == -1 {
            fraction = (1u64 << 50) | (fraction >> 2);
            result_exp = 0;
        }
        f32::from_bits(sign | (((result_exp as u32) & 0xff) << 23) | ((fraction >> 29) as u32))
    }

    /// The ARM `FRECPS`: `2 - a*b` fused (2 for an infinity times a zero).
    fn frecps_f32(a: f32, b: f32) -> f32 {
        if (a.is_infinite() && b == 0.0) || (a == 0.0 && b.is_infinite()) {
            return 2.0;
        }
        (-a).mul_add(b, 2.0)
    }

    /// The reciprocal carotene computes: `FRECPE` and one Newton step (`vrecpsq_f32`, `vmulq_f32`).
    fn carotene_recip(x: f32) -> f32 {
        let e = frecpe_f32(x);
        e * frecps_f32(e, x)
    }

    /// carotene/src/colorconvert.cpp:1087-1209 `convertToHSV` of the NEON vectors, one lane.
    fn carotene_convert_to_hsv_lane(r: u8, g: u8, b: u8, hrange: i32) -> [u8; 3] {
        const HSV_SHIFT: i32 = 12;
        let vsdiv_table = (255 << HSV_SHIFT) as f32;
        let vhdiv_table = (hrange << HSV_SHIFT) as f32;
        let vmax = r.max(g).max(b);
        let vmin = r.min(g).min(b);
        let diff = (vmax - vmin) as i32;
        let v = vmax as i32;
        let (r, g, b) = (r as i32, g as i32, b as i32);
        let veq_r = v == r;
        let veq_g = v == g;
        // the masks select (g - b), (b - r + 2 diff), (r - g + 4 diff) as in the scalar formula, in 16 bits
        let h16: i16 = if veq_r {
            (g - b) as i16
        } else if veq_g {
            (b - r + 2 * diff) as i16
        } else {
            (r - g + 4 * diff) as i16
        };

        let f_v = carotene_recip(v as f32);
        let f_hd = carotene_recip((diff as u32 * 6) as f32);
        // vcvtnq_u32_f32: to nearest even, saturating
        let res1 = (f_v * vsdiv_table).round_ties_even() as u32;
        let res3 = (f_hd * vhdiv_table).round_ties_even() as u32;

        let add1 = (diff as u32).wrapping_mul(res1).wrapping_add(1 << (HSV_SHIFT - 1));
        let add3 = (h16 as i32 as u32).wrapping_mul(res3).wrapping_add(1 << (HSV_SHIFT - 1));
        // vshrn_n_s32(x, 8): arithmetic shift, narrowed to 16 bits; vshrn_n_s16(x, 4): to 8 bits
        let shr1 = ((add1 as i32) >> 8) as i16;
        let shr3 = ((add3 as i32) >> 8) as i16;
        let s8 = (shr1 >> 4) as i8;
        let h8 = (shr3 >> 4) as i8;
        let hadd = if shr3 < 0 { hrange as i8 } else { 0 };
        let h = h8.wrapping_add(hadd);
        [h as u8, s8 as u8, vmax]
    }

    /// carotene/src/colorconvert.cpp:1264-1289 `convertToHSV` of one pixel.
    fn carotene_convert_to_hsv(r: i32, g: i32, b: i32, hrange: i32, hsv_shift: i32) -> [u8; 3] {
        let mut v = b;
        let mut vmin = b;

        // fastSaturate8u: the max / min of bytes
        v += (g - v).clamp(0, 255);
        v += (r - v).clamp(0, 255);
        vmin -= (vmin - g).clamp(0, 255);
        vmin -= (vmin - r).clamp(0, 255);

        let diff = v - vmin;
        let vr: i32 = if v == r { -1 } else { 0 };
        let vg: i32 = if v == g { -1 } else { 0 };

        // `s32(diff * (255 << hsv_shift) * (1.0f/(f32)v))`: the float product truncated (an
        // infinity or NaN converts as the ARM conversion does: saturated, NaN to 0)
        let s = (((diff * (255 << hsv_shift)) as f32 * (1.0f32 / v as f32)) as i32 + (1 << (hsv_shift - 1))) >> hsv_shift;
        let mut h = (vr & (g - b)) + (!vr & ((vg & (b - r + 2 * diff)) + ((!vg) & (r - g + 4 * diff))));
        // `s32((hrange << hsv_shift)/(6.f*diff) + 0.5)`: the float quotient plus the double 0.5
        let hdiv = (((hrange << hsv_shift) as f32 / (6.0f32 * diff as f32)) as f64 + 0.5) as i32;
        h = (h.wrapping_mul(hdiv) + (1 << (hsv_shift - 1))) >> hsv_shift;
        h += if h < 0 { hrange } else { 0 };

        [h.clamp(0, 255) as u8, s as u8, v as u8]
    }

    /// carotene/src/colorconvert.cpp:1840-1890 `rgb2hsv` of one row: blocks of eight pixels through
    /// the NEON path, the rest one by one. OpenCV's `cvtBGRtoHSV` hands the 8-bit RGB -> HSV
    /// conversion to carotene (hal/carotene/hal/tegra_hal.hpp:1739, `cv_hal_cvtBGRtoHSV`) on ARM
    /// builds.
    pub fn carotene_rgb2hsv(src: &[u8], dst: &mut [u8], width: usize, hrange: i32) {
        let roiw8 = if width >= 7 { width - 7 } else { 0 };
        const HSV_SHIFT: i32 = 12;
        let mut j = 0usize;
        while j < roiw8 {
            for l in 0..8 {
                let p = &src[(j + l) * 3..];
                let hsv = carotene_convert_to_hsv_lane(p[0], p[1], p[2], hrange);
                dst[(j + l) * 3..(j + l) * 3 + 3].copy_from_slice(&hsv);
            }
            j += 8;
        }
        while j < width {
            let p = &src[j * 3..];
            let hsv = carotene_convert_to_hsv(p[0] as i32, p[1] as i32, p[2] as i32, hrange, HSV_SHIFT);
            dst[j * 3..j * 3 + 3].copy_from_slice(&hsv);
            j += 1;
        }
    }

    /// color_hsv.simd.hpp:424-429 `ComputeSectorAndClampedH`
    fn compute_sector_and_clamped_h(h: &mut f32, sector: &mut i32) {
        // cvFloor(float)
        *sector = h.floor() as i32;
        *h -= *sector as f32;
        *sector %= 6;
        *sector += if *sector < 0 { 6 } else { 0 };
    }

    /// color_hsv.simd.hpp:432-456 `HSV2RGB_native`: `1.f - s*h` and `1.f - s*(1.f - h)` are fused.
    fn hsv2rgb_native(mut h: f32, s: f32, v: f32, b: &mut f32, g: &mut f32, r: &mut f32, hscale: f32) {
        if s == 0.0 {
            *b = v;
            *g = v;
            *r = v;
        } else {
            const SECTOR_DATA: [[usize; 3]; 6] = [[1, 3, 0], [1, 0, 2], [3, 0, 1], [0, 2, 1], [0, 1, 3], [2, 1, 0]];
            let mut tab = [0.0f32; 4];
            let mut sector = 0i32;
            h *= hscale;
            compute_sector_and_clamped_h(&mut h, &mut sector);

            tab[0] = v;
            tab[1] = v * (1.0 - s);
            tab[2] = v * (-s).mul_add(h, 1.0);
            tab[3] = v * (-s).mul_add(1.0 - h, 1.0);

            *b = tab[SECTOR_DATA[sector as usize][0]];
            *g = tab[SECTOR_DATA[sector as usize][1]];
            *r = tab[SECTOR_DATA[sector as usize][2]];
        }
    }

    /// color_hsv.simd.hpp:372-420 `HSV2RGB_simd` of one lane (separate NEON operations).
    fn hsv2rgb_simd_lane(h: f32, s: f32, v: f32, hscale: f32) -> (f32, f32, f32) {
        let mut v_h = h * hscale;
        let v_pre_sector = v_h.trunc();
        v_h -= v_pre_sector;
        let v_tab0 = v;
        let v_tab1 = v * (1.0 - s);
        let v_tab2 = v * (1.0 - s * v_h);
        let v_tab3 = v * (1.0 - s * (1.0 - v_h));

        let mut v_sector = v_pre_sector * (1.0f32 / 6.0f32);
        v_sector = v_sector.trunc();
        v_sector = v_pre_sector - v_sector * 6.0;

        let mut oh = if v_sector < 2.0 { v_tab1 } else { 0.0 };
        oh = if v_sector == 2.0 { v_tab3 } else { oh };
        oh = if v_sector == 3.0 { v_tab0 } else { oh };
        oh = if v_sector == 4.0 { v_tab0 } else { oh };
        oh = if v_sector > 4.0 { v_tab2 } else { oh };

        let mut os = if v_sector < 1.0 { v_tab3 } else { s };
        os = if v_sector == 1.0 { v_tab0 } else { os };
        os = if v_sector == 2.0 { v_tab0 } else { os };
        os = if v_sector == 3.0 { v_tab2 } else { os };
        os = if v_sector > 3.0 { v_tab1 } else { os };

        let mut ov = if v_sector < 1.0 { v_tab0 } else { v };
        ov = if v_sector == 1.0 { v_tab2 } else { ov };
        ov = if v_sector == 2.0 { v_tab1 } else { ov };
        ov = if v_sector == 3.0 { v_tab1 } else { ov };
        ov = if v_sector == 4.0 { v_tab3 } else { ov };
        ov = if v_sector > 4.0 { v_tab0 } else { ov };

        (oh, os, ov)
    }

    /// color_hsv.simd.hpp:518-663 `HSV2RGB_b::operator()` of one row of `n` pixels: blocks of
    /// sixteen pixels through the SIMD path (`v_trunc` of the scaled values, `v_pack_u` / `v_pack`
    /// saturation), the rest through the scalar path (`saturate_cast`, rounding).
    pub fn hsv2rgb_b(src: &[u8], dst: &mut [u8], n: usize, dcn: usize, blue_idx: usize, hrange: i32) {
        const VSIZE: usize = 4;
        let hscale = 6.0f32 / hrange as f32;
        let mut j = 0usize;
        let mut di = 0usize;
        let saturate_trunc = |x: f32| -> u8 {
            // v_trunc to int32, then v_pack_u (saturate to u16) and v_pack (saturate to u8)
            let t = if x.is_nan() { 0 } else { x.trunc().clamp(i32::MIN as f32, i32::MAX as f32) as i64 };
            t.clamp(0, 255) as u8
        };
        while n >= VSIZE * 4 && j <= (n - VSIZE * 4) * 3 {
            for l in 0..VSIZE * 4 {
                let h = src[j + l * 3] as f32;
                let s = src[j + l * 3 + 1] as f32 * (1.0f32 / 255.0f32);
                let v = src[j + l * 3 + 2] as f32 * (1.0f32 / 255.0f32);
                let (b, g, r) = hsv2rgb_simd_lane(h, s, v, hscale);
                let (bb, gg, rr) = (saturate_trunc(b * 255.0), saturate_trunc(g * 255.0), saturate_trunc(r * 255.0));
                let d = &mut dst[di + l * dcn..];
                if blue_idx == 0 {
                    d[0] = bb;
                    d[1] = gg;
                    d[2] = rr;
                } else {
                    d[0] = rr;
                    d[1] = gg;
                    d[2] = bb;
                }
                if dcn == 4 {
                    d[3] = 255;
                }
            }
            j += 3 * 4 * VSIZE;
            di += dcn * 4 * VSIZE;
        }

        while j < n * 3 {
            let h = src[j] as f32;
            let s = src[j + 1] as f32 * (1.0f32 / 255.0f32);
            let v = src[j + 2] as f32 * (1.0f32 / 255.0f32);
            let (mut b, mut g, mut r) = (0.0f32, 0.0f32, 0.0f32);
            hsv2rgb_native(h, s, v, &mut b, &mut g, &mut r, hscale);

            // saturate_cast<uchar>(float): cvRound, then clamp
            let sat = |x: f32| -> u8 { (x.round_ties_even() as i32).clamp(0, 255) as u8 };
            let d = &mut dst[di..];
            d[blue_idx] = sat(b * 255.0);
            d[1] = sat(g * 255.0);
            d[blue_idx ^ 2] = sat(r * 255.0);

            if dcn == 4 {
                d[3] = 255;
            }
            j += 3;
            di += dcn;
        }
    }

    /// The `cv::cvtColor` codes QuantKMeans uses.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum ColorConversionCode {
        Rgb2Hsv,
        Hsv2Rgb,
        Rgb2Lab,
        Lab2Rgb,
    }

    /// `cv::cvtColor(src, dst, code)` of an 8-bit three channel Mat (imgproc/src/color.cpp:
    /// `COLOR_RGB2HSV` hrange 180, `COLOR_RGB2Lab` sRGB, blue index 2), row by row
    /// (`CvtColorLoop`, color.simd_helpers.hpp:167-174). The destination may be the source.
    pub fn cvt_color(src: &Mat8u, code: ColorConversionCode) -> Mat8u {
        let mut dst = Mat8u::new(src.rows, src.cols, 3);
        let scn = src.channels;
        for row in 0..src.rows {
            let s = &src.data[row * src.cols * scn..(row + 1) * src.cols * scn];
            let d = &mut dst.data[row * src.cols * 3..(row + 1) * src.cols * 3];
            match code {
                ColorConversionCode::Rgb2Hsv => {
                    // carotene takes the three channel 8-bit conversion on ARM builds (WITH_CAROTENE)
                    if cfg!(target_arch = "aarch64") && scn == 3 {
                        carotene_rgb2hsv(s, d, src.cols, 180)
                    } else {
                        rgb2hsv_b(s, d, src.cols, scn, 2, 180)
                    }
                }
                ColorConversionCode::Hsv2Rgb => hsv2rgb_b(s, d, src.cols, 3, 2, 180),
                ColorConversionCode::Rgb2Lab => rgb2lab_b(s, d, src.cols, scn, 2, true),
                ColorConversionCode::Lab2Rgb => lab2rgb_b(s, d, src.cols, 3, 2, true),
            }
        }
        dst
    }
}

// =============================================================================================
// ObjColorUtils.hpp:19-334 — class QuantKMeans
// =============================================================================================

use cv::{Mat32f, Mat8u};

/// ObjColorUtils.hpp:21-26 `struct ClusterInfo`
#[derive(Debug, Clone, Copy)]
struct ClusterInfo {
    idx: i32,
    count: i32,
    center: [u8; 3],
}

/// ObjColorUtils.hpp:19-334 `class QuantKMeans`
#[derive(Debug, Clone)]
pub struct QuantKMeans {
    pub m_alpha_thres: i32,
    pub m_do_convert: bool,
    /// `cv::Mat m_flatten_labels`: N x 1 `CV_32S`.
    pub m_flatten_labels: Vec<i32>,
    /// `cv::Mat m_centers8UC3`: K x 1 `CV_8UC3`.
    pub m_centers8_uc3: Vec<[u8; 3]>,
}

impl Default for QuantKMeans {
    fn default() -> Self {
        QuantKMeans::new(10)
    }
}

impl QuantKMeans {
    /// ObjColorUtils.hpp:33 `QuantKMeans(int alpha_thres = 10)`
    pub fn new(alpha_thres: i32) -> Self {
        QuantKMeans { m_alpha_thres: alpha_thres, m_do_convert: false, m_flatten_labels: Vec::new(), m_centers8_uc3: Vec::new() }
    }

    /// ObjColorUtils.hpp:34-42 `void apply(cv::Mat &ori_image, cv::Mat &new_image, int num_cluster,
    /// int color_space)`: an 8-bit three channel image quantized into `new_image`.
    pub fn apply_image(&mut self, ori_image: &Mat8u, new_image: &mut Mat8u, num_cluster: i32, color_space: i32) -> Result<(), String> {
        let mut image = Mat8u::default();
        self.convert_color_space(ori_image, &mut image, color_space, false);
        let flatten_image = self.flatten(&image);

        self.apply_flatten(&flatten_image, num_cluster, color_space)?;
        self.replace_centers(ori_image, new_image);
        Ok(())
    }

    /// ObjColorUtils.hpp:43-55 `void apply_aplha(cv::Mat &ori_image, cv::Mat &new_image, int
    /// num_cluster, int color_space)`: an 8-bit four channel image, the pixels below the alpha
    /// threshold left out.
    pub fn apply_aplha(&mut self, ori_image: &Mat8u, new_image: &mut Mat8u, num_cluster: i32, color_space: i32) -> Result<(), String> {
        let flatten_image8_uc3 = self.flatten_alpha(ori_image);
        let mut image8_uc3 = Mat8u::default();
        self.convert_color_space(&flatten_image8_uc3, &mut image8_uc3, color_space, false);
        let mut image32_fc3 = Mat32f::new(image8_uc3.rows, 1, 3);
        for i in 0..image8_uc3.rows {
            let p = image8_uc3.vec3b(i);
            image32_fc3.data[i * 3..i * 3 + 3].copy_from_slice(&[p[0] as f32, p[1] as f32, p[2] as f32]);
        }

        self.apply_flatten(&image32_fc3, num_cluster, color_space)?;
        self.repalce_centers_aplha(ori_image, new_image);
        Ok(())
    }

    /// ObjColorUtils.hpp:56-66 `void apply(cv::Mat &flatten_image, int num_cluster, int color_space)`:
    /// k-means of N x 1 float samples; the centers saturated to bytes, converted back.
    pub fn apply_flatten(&mut self, flatten_image: &Mat32f, mut num_cluster: i32, color_space: i32) -> Result<(), String> {
        let mut centers32_fc3 = Mat32f::default();
        // `fmin(flatten_image.rows, num_cluster)` in double, stored as int
        num_cluster = (flatten_image.rows as f64).min(num_cluster as f64) as i32;
        cv::kmeans(
            flatten_image,
            num_cluster,
            &mut self.m_flatten_labels,
            cv::TermCriteria { type_: cv::TermCriteria::EPS + cv::TermCriteria::COUNT, max_count: 300, epsilon: 0.5 },
            3,
            cv::KMEANS_PP_CENTERS,
            &mut centers32_fc3,
        )?;
        self.m_centers8_uc3 = vec![[0u8; 3]; num_cluster as usize];
        for i in 0..num_cluster as usize {
            // Vec3f -> Vec3b: saturate_cast<uchar>(float), rounding to nearest even
            let c = centers32_fc3.row(i);
            let sat = |x: f32| -> u8 { (x.round_ties_even() as i32).clamp(0, 255) as u8 };
            self.m_centers8_uc3[i] = [sat(c[0]), sat(c[1]), sat(c[2])];
        }

        let centers = self.centers_mat();
        let mut converted = Mat8u::default();
        self.convert_color_space(&centers, &mut converted, color_space, true);
        self.set_centers_mat(&converted);
        Ok(())
    }

    /// ObjColorUtils.hpp:67-79 `void apply(const std::vector<std::array<float, 4>> &ori_colors,
    /// cluster_results, labels, int num_cluster = -1, int max_cluster = 15, int color_space = 2)`
    pub fn apply(
        &mut self,
        ori_colors: &[[f32; 4]],
        cluster_results: &mut Vec<[f32; 4]>,
        labels: &mut Vec<i32>,
        num_cluster: i32,
        max_cluster: i32,
        color_space: i32,
    ) -> Result<(), String> {
        let flatten_image8_uc3 = self.flatten_vector(ori_colors);

        self.apply_mat(&flatten_image8_uc3, cluster_results, labels, num_cluster, max_cluster, color_space)
    }

    /// ObjColorUtils.hpp:80-125 `void apply(const cv::Mat &flatten_image8UC3, cluster_results,
    /// labels, int num_cluster = -1, int max_cluster = 15, int color_space = 2)`: the colors
    /// themselves when there are no more than requested (in RGB, else in the color space), k-means
    /// otherwise; the clusters sorted by size.
    pub fn apply_mat(
        &mut self,
        flatten_image8_uc3: &Mat8u,
        cluster_results: &mut Vec<[f32; 4]>,
        labels: &mut Vec<i32>,
        num_cluster: i32,
        max_cluster: i32,
        color_space: i32,
    ) -> Result<(), String> {
        let mut image8_uc3 = Mat8u::default();
        self.m_do_convert = true;
        self.convert_color_space(flatten_image8_uc3, &mut image8_uc3, color_space, false);

        let mut image32_fc3 = Mat32f::new(image8_uc3.rows, 1, 3);
        for i in 0..image8_uc3.rows {
            let p = image8_uc3.vec3b(i);
            image32_fc3.data[i * 3..i * 3 + 3].copy_from_slice(&[p[0] as f32, p[1] as f32, p[2] as f32]);
        }

        let _best_cluster = 1;
        // `fmin(num_cluster, max_cluster)` in double, stored as int
        let mut real_num_cluster: i32 = if num_cluster < 1 { max_cluster } else { (num_cluster as f64).min(max_cluster as f64) as i32 };

        let mut centers32_fc3 = Mat32f::default();
        if !self.more_than_request(flatten_image8_uc3, real_num_cluster, -1) {
            real_num_cluster = self.compute_num_colors(flatten_image8_uc3);
            self.m_do_convert = false;
            self.apply_color_cluster_image(flatten_image8_uc3, &mut centers32_fc3);
        } else if !self.more_than_request(&image8_uc3, real_num_cluster, -1) {
            real_num_cluster = self.compute_num_colors(&image8_uc3);
            self.apply_color_cluster_image(&image8_uc3, &mut centers32_fc3);
        } else {
            self.apply_color_cluster_kmeans(&image32_fc3, real_num_cluster, &mut centers32_fc3)?;
        }

        self.m_centers8_uc3 = vec![[0u8; 3]; real_num_cluster as usize];
        for i in 0..real_num_cluster as usize {
            let center = centers32_fc3.row(i);
            // uchar(float): the conversion truncates
            self.m_centers8_uc3[i] = [center[0] as u8, center[1] as u8, center[2] as u8];
        }

        if self.m_do_convert {
            let centers = self.centers_mat();
            let mut converted = Mat8u::default();
            self.convert_color_space(&centers, &mut converted, color_space, true);
            self.set_centers_mat(&converted);
        }
        self.sort_center_and_relabel();

        cluster_results.clear();
        labels.clear();
        for i in 0..flatten_image8_uc3.rows {
            labels.push(self.m_flatten_labels[i]);
        }
        for i in 0..real_num_cluster as usize {
            let c = self.m_centers8_uc3[i];
            let center = [c[0] as f32, c[1] as f32, c[2] as f32];
            cluster_results.push([center[0] / 255.0f32, center[1] / 255.0f32, center[2] / 255.0f32, 1.0f32]);
        }
        Ok(())
    }

    /// ObjColorUtils.hpp:127-134 `apply_color_cluster_kmeans`: `cv::kmeans` with the global
    /// generator seeded with 12345, its state restored afterwards.
    pub fn apply_color_cluster_kmeans(&mut self, image32_fc3: &Mat32f, num_cluster: i32, center32_fc3: &mut Mat32f) -> Result<(), String> {
        let old_state = cv::the_rng_state();
        cv::set_the_rng_state(12345);
        let r = cv::kmeans(
            image32_fc3,
            num_cluster,
            &mut self.m_flatten_labels,
            cv::TermCriteria { type_: cv::TermCriteria::EPS + cv::TermCriteria::COUNT, max_count: 300, epsilon: 0.5 },
            3,
            cv::KMEANS_PP_CENTERS,
            center32_fc3,
        );
        cv::set_the_rng_state(old_state);
        r.map(|_| ())
    }

    /// ObjColorUtils.hpp:136-153 `apply_color_cluster_image`: each distinct color a cluster, in
    /// order of appearance.
    pub fn apply_color_cluster_image(&mut self, image8_uc3: &Mat8u, center32_fc3: &mut Mat32f) {
        let mut find_index: i32;
        let mut unique_image: Vec<[u8; 3]> = Vec::new();
        let mut cur_color: [u8; 3];
        self.m_flatten_labels = vec![0; image8_uc3.rows];
        for i in 0..image8_uc3.rows {
            cur_color = image8_uc3.vec3b(i);
            find_index = self.find_color_index(&cur_color, &unique_image);
            if find_index == -1 {
                find_index = unique_image.len() as i32;
                unique_image.push(cur_color);
            }
            self.m_flatten_labels[i] = find_index;
        }
        // `cv::Mat un8U(uniqueImage.size(), 1, CV_8UC3, data); un8U.convertTo(center32FC3, CV_32FC3)`
        *center32_fc3 = Mat32f::new(unique_image.len(), 1, 3);
        for (i, c) in unique_image.iter().enumerate() {
            center32_fc3.data[i * 3..i * 3 + 3].copy_from_slice(&[c[0] as f32, c[1] as f32, c[2] as f32]);
        }
    }

    /// ObjColorUtils.hpp:155-184 `sort_center_and_relabel`: the clusters by decreasing count, then
    /// by center; the labels remapped.
    ///
    /// As in the C++, the counts are taken over the first K labels only.
    pub fn sort_center_and_relabel(&mut self) {
        let k_count = self.m_centers8_uc3.len();
        let nums = self.m_flatten_labels.len();

        let mut counts: Vec<i32> = vec![0; k_count];
        for i in 0..k_count {
            counts[self.m_flatten_labels[i] as usize] += 1;
        }

        let mut info: Vec<ClusterInfo> = Vec::with_capacity(k_count);
        for i in 0..k_count {
            info.push(ClusterInfo { idx: i as i32, count: counts[i], center: self.m_centers8_uc3[i] });
        }

        crate::clipper::libcxx_sort(&mut info, |a: &ClusterInfo, b: &ClusterInfo| {
            if a.count != b.count {
                return a.count > b.count;
            }
            if a.center[0] != b.center[0] {
                return a.center[0] < b.center[0];
            }
            if a.center[1] != b.center[1] {
                return a.center[1] < b.center[1];
            }
            a.center[2] < b.center[2]
        });

        let mut remap: Vec<i32> = vec![0; k_count];
        for new_label in 0..k_count {
            remap[info[new_label].idx as usize] = new_label as i32;
        }

        let mut new_centers: Vec<[u8; 3]> = vec![[0; 3]; k_count];
        for i in 0..k_count {
            new_centers[i] = info[i].center;
        }
        self.m_centers8_uc3 = new_centers;

        for i in 0..nums {
            let old_label = self.m_flatten_labels[i];
            self.m_flatten_labels[i] = remap[old_label as usize];
        }
    }

    /// ObjColorUtils.hpp:186-199 `more_than_request(image8UC3, target_num, degree = 1)`
    pub fn more_than_request(&self, image8_uc3: &Mat8u, target_num: i32, degree: i32) -> bool {
        let mut unique_image: Vec<[u8; 3]> = Vec::new();
        let mut cur_color: [u8; 3];
        for i in 0..image8_uc3.rows {
            cur_color = image8_uc3.vec3b(i);
            if !self.is_in(&cur_color, &unique_image) {
                unique_image.push(cur_color);
                // `uniqueImage.size() >= target_num`: the int converted to size_t
                if degree > 0 && unique_image.len() >= target_num as usize {
                    return true;
                }
                if degree < 0 && unique_image.len() > target_num as usize {
                    return true;
                }
            }
        }
        false
    }

    /// ObjColorUtils.hpp:201-211 `compute_num_colors`
    pub fn compute_num_colors(&self, image8_uc3: &Mat8u) -> i32 {
        let mut unique_image: Vec<[u8; 3]> = Vec::new();
        let mut cur_color: [u8; 3];
        for i in 0..image8_uc3.rows {
            cur_color = image8_uc3.vec3b(i);
            if !self.is_in(&cur_color, &unique_image) {
                unique_image.push(cur_color);
            }
        }

        unique_image.len() as i32
    }

    /// ObjColorUtils.hpp:213-219 `find_color_index`
    pub fn find_color_index(&self, cur_color: &[u8; 3], unique_image: &[[u8; 3]]) -> i32 {
        for i in 0..unique_image.len() {
            if cur_color[0] == unique_image[i][0] && cur_color[1] == unique_image[i][1] && cur_color[2] == unique_image[i][2] {
                return i as i32;
            }
        }
        -1
    }

    /// ObjColorUtils.hpp:221-226 `is_in`
    pub fn is_in(&self, cur_color: &[u8; 3], unique_image: &[[u8; 3]]) -> bool {
        for color in unique_image {
            if cur_color[0] == color[0] && cur_color[1] == color[1] && cur_color[2] == color[2] {
                return true;
            }
        }
        false
    }

    /// ObjColorUtils.hpp:228-246 `repeat_center(cur_cluster, centers32FC3, color_space)`: whether
    /// two of the first centers are the same color once converted back.
    pub fn repeat_center(&self, cur_cluster: i32, centers32_fc3: &Mat32f, color_space: i32) -> bool {
        let mut centers8_uc3 = Mat8u::new(cur_cluster as usize, 1, 3);
        for i in 0..cur_cluster as usize {
            let center = centers32_fc3.row(i);
            centers8_uc3.data[i * 3..i * 3 + 3].copy_from_slice(&[center[0] as u8, center[1] as u8, center[2] as u8]);
        }
        let mut converted = Mat8u::default();
        self.convert_color_space(&centers8_uc3, &mut converted, color_space, true);
        let centers8_uc3 = converted;
        let mut unique_centers: Vec<[u8; 3]> = Vec::new();
        let mut cur_center: [u8; 3];
        for i in 0..cur_cluster as usize {
            cur_center = centers8_uc3.vec3b(i);
            if !self.is_in(&cur_center, &unique_centers) {
                unique_centers.push(cur_center);
            } else {
                return true;
            }
        }
        false
    }

    /// ObjColorUtils.hpp:248-257 `replace_centers(ori_image, new_image)`
    pub fn replace_centers(&self, ori_image: &Mat8u, new_image: &mut Mat8u) {
        for i in 0..ori_image.rows {
            for j in 0..ori_image.cols {
                let idx = self.m_flatten_labels[i * ori_image.cols + j];
                let pixel = self.m_centers8_uc3[idx as usize];
                new_image.px_mut(i, j)[..3].copy_from_slice(&pixel);
            }
        }
    }

    /// ObjColorUtils.hpp:258-275 `repalce_centers_aplha(ori_image, new_image)`
    pub fn repalce_centers_aplha(&self, ori_image: &Mat8u, new_image: &mut Mat8u) {
        let mut cnt = 0usize;
        let mut idx: i32;
        let mut center: [u8; 3];
        for i in 0..ori_image.rows {
            for j in 0..ori_image.cols {
                let p = ori_image.px(i, j);
                let pixel = [p[0], p[1], p[2], p[3]];
                if (pixel[3] as i32) < self.m_alpha_thres {
                    new_image.px_mut(i, j)[..4].copy_from_slice(&pixel);
                } else {
                    idx = self.m_flatten_labels[cnt];
                    cnt += 1;
                    center = self.m_centers8_uc3[idx as usize];
                    new_image.px_mut(i, j)[..4].copy_from_slice(&[center[0], center[1], center[2], pixel[3]]);
                }
            }
        }
    }

    /// ObjColorUtils.hpp:277-295 `convert_color_space(ori_image, image, color_space, reverse)`:
    /// 0 none, 1 HSV, 2 Lab.
    pub fn convert_color_space(&self, ori_image: &Mat8u, image: &mut Mat8u, color_space: i32, reverse: bool) {
        match color_space {
            0 => *image = ori_image.clone(),
            1 => {
                if reverse {
                    *image = cv::cvt_color(ori_image, cv::ColorConversionCode::Hsv2Rgb);
                } else {
                    *image = cv::cvt_color(ori_image, cv::ColorConversionCode::Rgb2Hsv);
                }
            }
            2 => {
                if reverse {
                    *image = cv::cvt_color(ori_image, cv::ColorConversionCode::Lab2Rgb);
                } else {
                    *image = cv::cvt_color(ori_image, cv::ColorConversionCode::Rgb2Lab);
                }
            }
            _ => {}
        }
    }

    /// ObjColorUtils.hpp:297-308 `flatten(image)`: the pixels of an 8-bit three channel image as
    /// N x 1 floats.
    pub fn flatten(&self, image: &Mat8u) -> Mat32f {
        let num_pixels = image.rows * image.cols;
        let mut img = Mat32f::new(num_pixels, 1, 3);
        for i in 0..image.rows {
            for j in 0..image.cols {
                let p = image.px(i, j);
                let o = (i * image.cols + j) * 3;
                img.data[o..o + 3].copy_from_slice(&[p[0] as f32, p[1] as f32, p[2] as f32]);
            }
        }
        img
    }

    /// ObjColorUtils.hpp:309-327 `flatten_alpha(image)`: the pixels of a four channel image at or
    /// above the alpha threshold, as N x 1 bytes.
    pub fn flatten_alpha(&self, image: &Mat8u) -> Mat8u {
        let mut num_pixels = (image.rows * image.cols) as i64;
        for i in 0..image.rows {
            for j in 0..image.cols {
                let pixel = image.px(i, j);
                if (pixel[3] as i32) < self.m_alpha_thres {
                    num_pixels -= 1;
                }
            }
        }

        let mut img = Mat8u::new(num_pixels.max(0) as usize, 1, 3);
        let mut cnt = 0usize;
        for i in 0..image.rows {
            for j in 0..image.cols {
                let pixel = image.px(i, j);
                if (pixel[3] as i32) >= self.m_alpha_thres {
                    img.data[cnt * 3..cnt * 3 + 3].copy_from_slice(&[pixel[0], pixel[1], pixel[2]]);
                    cnt += 1;
                }
            }
        }
        img
    }

    /// ObjColorUtils.hpp:328-337 `flatten_vector(ori_colors)`: `cv::Vec3b((int)(c*255.f), ...)`,
    /// the ints narrowed to bytes.
    pub fn flatten_vector(&self, ori_colors: &[[f32; 4]]) -> Mat8u {
        let num_pixels = ori_colors.len();

        let mut image8_uc3 = Mat8u::new(num_pixels, 1, 3);
        for i in 0..num_pixels {
            let pixel = ori_colors[i];
            image8_uc3.data[i * 3] = ((pixel[0] * 255.0f32) as i32) as u8;
            image8_uc3.data[i * 3 + 1] = ((pixel[1] * 255.0f32) as i32) as u8;
            image8_uc3.data[i * 3 + 2] = ((pixel[2] * 255.0f32) as i32) as u8;
        }
        image8_uc3
    }

    /// `m_centers8UC3` as the K x 1 Mat the conversions take.
    fn centers_mat(&self) -> Mat8u {
        let mut m = Mat8u::new(self.m_centers8_uc3.len(), 1, 3);
        for (i, c) in self.m_centers8_uc3.iter().enumerate() {
            m.data[i * 3..i * 3 + 3].copy_from_slice(c);
        }
        m
    }

    fn set_centers_mat(&mut self, m: &Mat8u) {
        self.m_centers8_uc3 = (0..m.rows).map(|i| m.vec3b(i)).collect();
    }
}

// =============================================================================================
// ObjColorUtils.cpp
// =============================================================================================

/// ObjColorUtils.cpp:9-21 `bool obj_color_deal_algo(input_colors, cluster_colors_from_algo,
/// cluster_labels_from_algo, char &cluster_number, int max_cluster)`: `char` is signed (the
/// macOS and x86 ABIs).
pub fn obj_color_deal_algo(
    input_colors: &[RGBA],
    cluster_colors_from_algo: &mut Vec<RGBA>,
    cluster_labels_from_algo: &mut Vec<i32>,
    cluster_number: &mut i8,
    max_cluster: i32,
) -> bool {
    let mut quant = QuantKMeans::new(10);
    // A cv::Exception would leave the function; the outputs are left as they are.
    let _ = quant.apply(input_colors, cluster_colors_from_algo, cluster_labels_from_algo, *cluster_number as i32, max_cluster, 2);
    if *cluster_number == -1 {
        return false;
    }
    true
}

/// ObjColorUtils.cpp:23 `UNCOLORED_FACE_FALLBACK`
const UNCOLORED_FACE_FALLBACK: RGBA = [0.5, 0.5, 0.5, 1.0];

/// ObjColorUtils.cpp:25-43 `standard_3mf_corner_color(color_group_map, color_info, binding,
/// corner)`: the color of a face corner from its own group, else the volume's default color;
/// `None` when neither has one.
pub fn standard_3mf_corner_color<'a>(
    color_group_map: &'a BTreeMap<i32, Vec<String>>,
    color_info: &VolumeColorInfo,
    binding: &TriangleColor,
    corner: i32,
) -> Option<&'a String> {
    let color_at = |pid: i32, index: i32| -> Option<&'a String> {
        if pid < 0 {
            return None;
        }
        let group = match color_group_map.get(&pid) {
            Some(group) if !group.is_empty() => group,
            _ => return None,
        };
        let color = &group[if index >= 0 && index < group.len() as i32 { index as usize } else { 0 }];
        if color.is_empty() {
            None
        } else {
            Some(color)
        }
    };
    if let Some(color) = color_at(binding.pid, binding.indices[corner as usize]) {
        return Some(color);
    }
    color_at(color_info.pid, color_info.pindex)
}

/// ObjColorUtils.cpp:45-130 `static bool volume_to_textured_mesh(volume, color_group_map,
/// color_info, out, has_colored, has_uncolored)`: the volume's faces with their corner colors and
/// their average color (un-indexed, three vertices a face).
fn volume_to_textured_mesh(
    volume: &ModelVolume,
    color_group_map: &BTreeMap<i32, Vec<String>>,
    color_info: Option<&VolumeColorInfo>,
    out: &mut TexturedMesh,
    has_colored: &mut bool,
    has_uncolored: &mut bool,
) -> bool {
    *out = TexturedMesh::default();
    let its = &volume.mesh().its;
    let face_count = its.indices.len();
    if its.vertices.is_empty() || face_count == 0 {
        return false;
    }

    let mut color_cache: HashMap<String, RGBA> = HashMap::new();
    let mut to_rgba = |color_str: &String| -> RGBA {
        *color_cache.entry(color_str.clone()).or_insert_with(|| crate::color::convert_color_string_to_rgba(color_str))
    };

    let mut vertex_colors: Vec<[f32; 4]> = vec![UNCOLORED_FACE_FALLBACK; face_count * 3];
    let mut face_colors: Vec<[usize; 3]> = vec![STANDARD_3MF_UNCOLORED_RGB; face_count];
    let mut colored_faces = 0usize;

    if let Some(color_info) = color_info {
        for face_idx in 0..face_count {
            let binding = &color_info.triangle_colors[face_idx];

            let mut corner_colors: [RGBA; 3] = [[0.0; 4]; 3];
            let mut colored_corners = 0;
            for corner in 0..3 {
                let color_str = standard_3mf_corner_color(color_group_map, color_info, binding, corner as i32);
                corner_colors[corner] = match color_str {
                    Some(color_str) => to_rgba(color_str),
                    None => UNCOLORED_FACE_FALLBACK,
                };
                if color_str.is_some() {
                    colored_corners += 1;
                }
            }
            if colored_corners == 0 {
                continue;
            }
            if colored_corners < 3 {
                *has_uncolored = true;
            }
            colored_faces += 1;

            for corner in 0..3 {
                vertex_colors[face_idx * 3 + corner] = corner_colors[corner];
            }
            for channel in 0..3 {
                let averaged = (corner_colors[0][channel] + corner_colors[1][channel] + corner_colors[2][channel]) / 3.0f32 * 255.0f32;
                // std::lround(float): half away from zero
                face_colors[face_idx][channel] = averaged.clamp(0.0, 255.0).round() as i64 as usize;
            }
        }
    }

    out.vertices = vec![[0.0f32; 3]; face_count * 3];
    out.indices = vec![[0i32; 3]; face_count];
    for i in 0..face_count {
        for corner in 0..3 {
            let v = &its.vertices[its.indices[i][corner] as usize];
            out.vertices[i * 3 + corner] = [v.x, v.y, v.z];
        }
        out.indices[i] = [(i * 3) as i32, (i * 3 + 1) as i32, (i * 3 + 2) as i32];
    }
    out.precomputed_face_colors = face_colors;
    out.precomputed_vertex_colors = vertex_colors;

    if colored_faces > 0 {
        *has_colored = true;
    }
    if colored_faces < face_count {
        *has_uncolored = true;
    }

    log::info!("built textured mesh from standard 3mf colors, faces={}, colored faces={}", face_count, colored_faces);
    true
}

/// ObjColorUtils.cpp:132-151 `static void append_textured_mesh(dst, src, world)`: `src` appended
/// to `dst`, its vertices transformed by `world` when given.
fn append_textured_mesh(dst: &mut TexturedMesh, mut src: TexturedMesh, world: Option<&crate::geometry::geometry::Transform3d>) {
    use crate::geometry::geometry::Vec3d;
    let base = dst.vertices.len() as i32;
    dst.vertices.reserve(src.vertices.len());
    for v in src.vertices.iter_mut() {
        if let Some(world) = world {
            let p = crate::geometry::geometry::eigen::transform_mul_vec(world, &Vec3d::new(v[0] as f64, v[1] as f64, v[2] as f64));
            *v = [p.x as f32, p.y as f32, p.z as f32];
        }
        dst.vertices.push(*v);
    }
    dst.indices.reserve(src.indices.len());
    for f in src.indices.iter() {
        dst.indices.push([f[0] + base, f[1] + base, f[2] + base]);
    }
    dst.precomputed_face_colors.extend_from_slice(&src.precomputed_face_colors);
    dst.precomputed_vertex_colors.extend_from_slice(&src.precomputed_vertex_colors);
}

/// ObjColorUtils.cpp:153-204 `bool standard_3mf_to_textured_meshes(model, color_group_map,
/// volume_color_data, out, has_uncolored)`: the model parts with standard 3MF colors as one
/// textured mesh (in world coordinates when there are several parts); whether any face is colored.
pub fn standard_3mf_to_textured_meshes(
    model: &Model,
    color_group_map: &BTreeMap<i32, Vec<String>>,
    volume_color_data: &HashMap<i32, VolumeColorInfo>,
    out: &mut TexturedMesh,
    has_uncolored: &mut bool,
) -> bool {
    *out = TexturedMesh::default();
    *has_uncolored = false;

    struct ColoredPart<'a> {
        object: &'a crate::model::ModelObject,
        volume: &'a ModelVolume,
        color_info: Option<&'a VolumeColorInfo>,
    }
    let mut parts: Vec<ColoredPart> = Vec::new();
    for object in model.objects.iter() {
        for volume in object.volumes.iter() {
            if !volume.is_model_part() {
                continue;
            }
            // `volume_color_data.find(volume->id().id)`: the size_t ID as the int key
            let color_iter = volume_color_data.get(&(volume.id().id as i32));
            let colored = matches!(color_iter, Some(info) if info.triangle_colors.len() == volume.mesh().its.indices.len());
            parts.push(ColoredPart { object, volume, color_info: if colored { color_iter } else { None } });
        }
    }
    if parts.is_empty() {
        return false;
    }

    let bake_world = parts.len() > 1;
    let mut appended = 0;
    let mut colored = false;
    for part in parts.iter() {
        let mut piece = TexturedMesh::default();
        if !volume_to_textured_mesh(part.volume, color_group_map, part.color_info, &mut piece, &mut colored, has_uncolored) {
            continue;
        }
        let mut world_ptr: Option<crate::geometry::geometry::Transform3d> = None;
        if bake_world {
            let mut world = part.volume.get_transformation().get_matrix(false, false, false, false);
            if !part.object.instances.is_empty() {
                world = crate::model::eigen_mul_affine(&part.object.instances[0].get_transformation().get_matrix(false, false, false, false), &world);
            }
            world_ptr = Some(world);
        }
        append_textured_mesh(out, piece, world_ptr.as_ref());
        appended += 1;
    }
    out.is_standard_3mf = true;
    colored && appended > 0 && !out.vertices.is_empty()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rng_sequence() {
        let mut rng = cv::Rng::new(12345);
        // state = 12345 * 4164903690 + 0
        assert_eq!(rng.next(), (12345u64 * 4164903690u64) as u32);
        assert_eq!(rng.state, 12345u64 * 4164903690u64);
    }

    #[test]
    fn hsv_round_trip_of_grays() {
        let src = cv::Mat8u { rows: 1, cols: 3, channels: 3, data: vec![0, 0, 0, 128, 128, 128, 255, 255, 255] };
        let hsv = cv::cvt_color(&src, cv::ColorConversionCode::Rgb2Hsv);
        assert_eq!(hsv.data, vec![0, 0, 0, 0, 0, 128, 0, 0, 255]);
        let back = cv::cvt_color(&hsv, cv::ColorConversionCode::Hsv2Rgb);
        assert_eq!(back.data, src.data);
    }

    #[test]
    fn lab_of_white_and_black() {
        let src = cv::Mat8u { rows: 2, cols: 1, channels: 3, data: vec![0, 0, 0, 255, 255, 255] };
        let lab = cv::cvt_color(&src, cv::ColorConversionCode::Rgb2Lab);
        assert_eq!(lab.data, vec![0, 128, 128, 255, 128, 128]);
    }

    #[test]
    fn few_colors_are_their_own_clusters() {
        let colors: Vec<RGBA> = vec![[1.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 1.0], [1.0, 0.0, 0.0, 1.0]];
        let mut clusters = Vec::new();
        let mut labels = Vec::new();
        let mut n: i8 = -1;
        let r = obj_color_deal_algo(&colors, &mut clusters, &mut labels, &mut n, 15);
        assert!(!r);
        // sort_center_and_relabel counts the first K labels only: a tie, broken by the centers
        assert_eq!(clusters, vec![[0.0, 1.0, 0.0, 1.0], [1.0, 0.0, 0.0, 1.0]]);
        assert_eq!(labels, vec![1, 0, 1]);
    }

    #[test]
    fn corner_color_falls_back_to_the_volume() {
        let mut map = BTreeMap::new();
        map.insert(1, vec!["#FF0000".to_string(), "".to_string()]);
        let mut info = VolumeColorInfo::new();
        info.pid = 1;
        info.pindex = 0;
        let mut binding = TriangleColor::new();
        binding.pid = 1;
        binding.indices = [1, 5, -1];
        // corner 0: the empty color of index 1, then the volume's
        assert_eq!(standard_3mf_corner_color(&map, &info, &binding, 0).map(|s| s.as_str()), Some("#FF0000"));
        // corner 1: out of range takes the first color of the group
        assert_eq!(standard_3mf_corner_color(&map, &info, &binding, 1).map(|s| s.as_str()), Some("#FF0000"));
        binding.pid = -1;
        info.pid = 7;
        assert_eq!(standard_3mf_corner_color(&map, &info, &binding, 2), None);
    }
}
