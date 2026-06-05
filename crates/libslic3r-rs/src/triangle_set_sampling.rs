//! Faithful 1:1 port of `TriangleSetSampling.{hpp,cpp}` from BambuStudio libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/TriangleSetSampling.hpp
//! - src/libslic3r/TriangleSetSampling.cpp
//!
//! Computes a uniform-by-area random sampling of an indexed triangle set.
//!
//! Fidelity notes (byte-exact G-code parity):
//! - C++ stores mesh vertices as `Vec3f` (Eigen `Matrix<float,3,1>`); we mirror this
//!   with nalgebra `Vector3<f32>`. The area computation casts to `double` exactly as
//!   in C++.
//! - `area_sum` is accumulated in `float` (f32) in C++, even though the per-triangle
//!   areas are doubles. We reproduce this (`area_sum: f32`) so the cumulative key
//!   sequence and `total_area` match bit-for-bit.
//! - The RNG is `std::mt19937_64` seeded with `27644437`, and the `[0,1)` samples are
//!   produced via libstdc++'s `std::generate_canonical<double, 53>` path (one engine
//!   draw per double for a 64-bit engine). See [`Mt19937_64`] and
//!   [`UniformRealDistribution`].
//! - Eigen evaluates `Vec3f * double` by casting the scalar to `float`, keeping the
//!   product in f32; we replicate that by casting `(1 - sq_u)` etc. to f32.

use nalgebra::Vector3;

/// 3D single-precision vector, mirroring C++ `Vec3f` (Eigen `Matrix<float,3,1>`).
/// Point.hpp
pub type Vec3f = Vector3<f32>;
/// 3D double-precision vector, mirroring C++ `Vec3d` (Eigen `Matrix<double,3,1>`).
/// Point.hpp
pub type Vec3d = Vector3<f64>;
/// 3D integer index vector, mirroring C++ `Vec3i` / `stl_triangle_vertex_indices`.
/// Point.hpp
pub type Vec3i = Vector3<i32>;

/// Indexed triangle set, mirroring C++ `indexed_triangle_set` (admesh/stl.h).
///
/// Vertices are stored as `Vec3f` and triangles as `Vec3i` index triples, accessed
/// via `.x()`, `.y()`, `.z()` exactly as in the C++ source.
#[derive(Debug, Clone, Default)]
pub struct indexed_triangle_set {
    /// Vertex positions (single precision), matching C++ `std::vector<stl_vertex>`.
    pub vertices: Vec<Vec3f>,
    /// Triangle vertex indices, matching C++ `std::vector<stl_triangle_vertex_indices>`.
    pub indices: Vec<Vec3i>,
}

/// C++ class `TriangleSetSamples`
/// TriangleSetSampling.hpp:9-14
#[derive(Debug, Clone, Default)]
pub struct TriangleSetSamples {
    // TriangleSetSampling.hpp:10
    pub total_area: f32,
    // TriangleSetSampling.hpp:11
    pub positions: Vec<Vec3f>,
    // TriangleSetSampling.hpp:12
    pub normals: Vec<Vec3f>,
    // TriangleSetSampling.hpp:13
    pub triangle_indices: Vec<usize>,
}

// TriangleSetSampling.cpp:9
pub fn sample_its_uniform_parallel(
    samples_count: usize,
    triangle_set: &indexed_triangle_set,
) -> TriangleSetSamples {
    // TriangleSetSampling.cpp:10
    let mut triangles_area: Vec<f64> = vec![0.0; triangle_set.indices.len()];

    // TriangleSetSampling.cpp:12-22 — tbb::parallel_for over [0, indices.size())
    {
        use rayon::prelude::*;
        triangles_area
            .par_iter_mut()
            .enumerate()
            .for_each(|(t_idx, area_out)| {
                // TriangleSetSampling.cpp:16
                let a = triangle_set.vertices[triangle_set.indices[t_idx].x as usize];
                // TriangleSetSampling.cpp:17
                let b = triangle_set.vertices[triangle_set.indices[t_idx].y as usize];
                // TriangleSetSampling.cpp:18
                let c = triangle_set.vertices[triangle_set.indices[t_idx].z as usize];
                // TriangleSetSampling.cpp:19
                let area = 0.5 * f64::from((b - a).cross(&(c - a)).norm());
                // TriangleSetSampling.cpp:20
                *area_out = area;
            });
    }

    // TriangleSetSampling.cpp:24
    // std::map<double, size_t> area_sum_to_triangle_idx;
    let mut area_sum_to_triangle_idx: std::collections::BTreeMap<ordered_float::OrderedFloat<f64>, usize> =
        std::collections::BTreeMap::new();
    // TriangleSetSampling.cpp:25
    let mut area_sum: f32 = 0.0;
    // TriangleSetSampling.cpp:26
    for t_idx in 0..triangles_area.len() {
        // TriangleSetSampling.cpp:27 — `float area_sum += double`: C++ promotes
        // area_sum to double, adds in double, then truncates the result back to float
        // on assignment. We reproduce that exact sequence here.
        area_sum = (f64::from(area_sum) + triangles_area[t_idx]) as f32;
        // TriangleSetSampling.cpp:28 — key is the f32 area_sum promoted to double
        area_sum_to_triangle_idx.insert(ordered_float::OrderedFloat(f64::from(area_sum)), t_idx);
    }

    // TriangleSetSampling.cpp:31
    let mut mersenne_engine = Mt19937_64::new(27644437);
    // TriangleSetSampling.cpp:32 — random numbers on interval [0, 1)
    // TriangleSetSampling.cpp:33
    let fdistribution = UniformRealDistribution::new();

    // TriangleSetSampling.cpp:35-37
    let mut get_random = || {
        // TriangleSetSampling.cpp:36 — braced-init evaluates x, y, z left-to-right
        let x = fdistribution.sample(&mut mersenne_engine);
        let y = fdistribution.sample(&mut mersenne_engine);
        let z = fdistribution.sample(&mut mersenne_engine);
        Vec3d::new(x, y, z)
    };

    // TriangleSetSampling.cpp:39
    let mut random_samples: Vec<Vec3d> = vec![Vec3d::zeros(); samples_count];
    // TriangleSetSampling.cpp:40 — std::generate(begin, end, get_random)
    for s in random_samples.iter_mut() {
        *s = get_random();
    }

    // TriangleSetSampling.cpp:42
    let mut result = TriangleSetSamples::default();
    // TriangleSetSampling.cpp:43
    result.total_area = area_sum;
    // TriangleSetSampling.cpp:44
    result.positions = vec![Vec3f::zeros(); samples_count];
    // TriangleSetSampling.cpp:45
    result.normals = vec![Vec3f::zeros(); samples_count];
    // TriangleSetSampling.cpp:46
    result.triangle_indices = vec![0usize; samples_count];

    // TriangleSetSampling.cpp:48-66 — tbb::parallel_for over [0, samples_count)
    {
        use rayon::prelude::*;
        // We zip the three disjoint output buffers so each parallel iteration writes
        // its own slot, exactly as the C++ indexes result.{positions,normals,...}[s_idx].
        result
            .positions
            .par_iter_mut()
            .zip(result.normals.par_iter_mut())
            .zip(result.triangle_indices.par_iter_mut())
            .enumerate()
            .for_each(|(s_idx, ((position_out, normal_out), triangle_index_out))| {
                // TriangleSetSampling.cpp:52
                let t_sample = random_samples[s_idx].x * f64::from(area_sum);
                // TriangleSetSampling.cpp:53 — area_sum_to_triangle_idx.upper_bound(t_sample)->second
                let t_idx = *area_sum_to_triangle_idx
                    .range((
                        std::ops::Bound::Excluded(ordered_float::OrderedFloat(t_sample)),
                        std::ops::Bound::Unbounded,
                    ))
                    .next()
                    .unwrap()
                    .1;

                // TriangleSetSampling.cpp:55
                let sq_u = random_samples[s_idx].y.sqrt();
                // TriangleSetSampling.cpp:56
                let v = random_samples[s_idx].z;

                // TriangleSetSampling.cpp:58
                let a: Vec3f = triangle_set.vertices[triangle_set.indices[t_idx].x as usize];
                // TriangleSetSampling.cpp:59
                let b: Vec3f = triangle_set.vertices[triangle_set.indices[t_idx].y as usize];
                // TriangleSetSampling.cpp:60
                let c: Vec3f = triangle_set.vertices[triangle_set.indices[t_idx].z as usize];

                // TriangleSetSampling.cpp:62
                // result.positions[s_idx] = A * (1 - sq_u) + B * (sq_u * (1 - v)) + C * (v * sq_u);
                // Eigen casts the double scalar to float for `Vec3f * double`.
                *position_out = a * ((1.0 - sq_u) as f32)
                    + b * ((sq_u * (1.0 - v)) as f32)
                    + c * ((v * sq_u) as f32);
                // TriangleSetSampling.cpp:63
                *normal_out = (b - a).cross(&(c - b)).normalize();
                // TriangleSetSampling.cpp:64
                *triangle_index_out = t_idx;
            });
    }

    // TriangleSetSampling.cpp:68
    result
}

/// Faithful port of `std::mt19937_64` (libstdc++ `mersenne_twister_engine` with the
/// standard 64-bit parameters). Used by [`sample_its_uniform_parallel`] seeded with
/// `27644437`.
pub struct Mt19937_64 {
    mt: [u64; Self::N],
    index: usize,
}

impl Mt19937_64 {
    // mt19937_64 parameters (w=64, n=312, m=156, r=31, ...)
    const N: usize = 312;
    const M: usize = 156;
    const MATRIX_A: u64 = 0xB502_6F5A_A966_19E9;
    const UPPER_MASK: u64 = 0xFFFF_FFFF_8000_0000; // most significant 33 bits
    const LOWER_MASK: u64 = 0x0000_0000_7FFF_FFFF; // least significant 31 bits

    /// Seed the engine, mirroring libstdc++'s seeding of `mersenne_twister_engine`.
    pub fn new(seed: u64) -> Self {
        let mut mt = [0u64; Self::N];
        mt[0] = seed;
        for i in 1..Self::N {
            // x = 6364136223846793005 * (mt[i-1] ^ (mt[i-1] >> 62)) + i
            mt[i] = 6_364_136_223_846_793_005u64
                .wrapping_mul(mt[i - 1] ^ (mt[i - 1] >> 62))
                .wrapping_add(i as u64);
        }
        Self {
            mt,
            index: Self::N,
        }
    }

    fn generate(&mut self) {
        for i in 0..Self::N {
            let y = (self.mt[i] & Self::UPPER_MASK)
                | (self.mt[(i + 1) % Self::N] & Self::LOWER_MASK);
            let mut next = self.mt[(i + Self::M) % Self::N] ^ (y >> 1);
            if y & 1 != 0 {
                next ^= Self::MATRIX_A;
            }
            self.mt[i] = next;
        }
        self.index = 0;
    }

    /// Produce the next 64-bit value, applying the standard tempering.
    pub fn next_u64(&mut self) -> u64 {
        if self.index >= Self::N {
            self.generate();
        }
        let mut y = self.mt[self.index];
        self.index += 1;
        // Tempering (b=0x71d67fffeda60000, c=0xfff7eee000000000, l=43, s=17, t=37, u=29, d=...)
        y ^= (y >> 29) & 0x5555_5555_5555_5555;
        y ^= (y << 17) & 0x71D6_7FFF_EDA6_0000;
        y ^= (y << 37) & 0xFFF7_EEE0_0000_0000;
        y ^= y >> 43;
        y
    }
}

/// Faithful port of libstdc++'s `std::uniform_real_distribution<double>` over the
/// default interval `[0, 1)`. The result is produced by
/// `std::generate_canonical<double, numeric_limits<double>::digits>(urng)`.
pub struct UniformRealDistribution {
    a: f64,
    b: f64,
}

impl UniformRealDistribution {
    /// Default-constructed: interval `[0, 1)`.
    pub fn new() -> Self {
        Self { a: 0.0, b: 1.0 }
    }

    /// Draw one value, mirroring libstdc++:
    /// `(b - a) * generate_canonical<double, digits>(urng) + a`.
    pub fn sample(&self, urng: &mut Mt19937_64) -> f64 {
        (self.b - self.a) * generate_canonical_f64_53(urng) + self.a
    }
}

impl Default for UniformRealDistribution {
    fn default() -> Self {
        Self::new()
    }
}

/// Faithful port of libstdc++'s `std::generate_canonical<double, 53, mt19937_64>`.
///
/// For a 64-bit engine producing `[0, 2^64-1]` and `bits = min(53, 64) = 53`,
/// `__k = ceil(53 / 64) = 1`, so exactly one engine draw is consumed and
/// `__ret = __sum / __tmp` where `__sum = (urng() - min)` and `__tmp = (max - min) + 1`.
fn generate_canonical_f64_53(urng: &mut Mt19937_64) -> f64 {
    // __r = static_cast<long double>(urng.max() - urng.min()) + 1.0L;
    // For mt19937_64, (max - min) + 1 == 2^64.
    let r: f64 = 2.0f64.powi(64);

    // __sum accumulation over __k = 1 draws.
    // __sum = (urng() - urng.min()) * __factor + __sum, with __factor starting at 1.
    let sum: f64 = urng.next_u64() as f64;
    let tmp: f64 = r;

    // __ret = __sum / __tmp;
    let mut ret = sum / tmp;
    // if (__builtin_expect(__ret >= 1, 0)) __ret = ...nextafter-style clamp...
    // libstdc++ clamps to just below 1 if rounding produces exactly 1.
    if ret >= 1.0 {
        ret = 1.0 - f64::EPSILON / 2.0;
    }
    ret
}
