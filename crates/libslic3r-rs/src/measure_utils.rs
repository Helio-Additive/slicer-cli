//! Faithful 1:1 port of `src/libslic3r/MeasureUtils.hpp`.
//!
//! This is a header-only C++ file (all definitions are inline); there is no
//! corresponding `.cpp`. Line references below point into `MeasureUtils.hpp`.
//!
//! ///|/ Copyright (c) Prusa Research 2022 Enrico Turri @enricoturri1966
//! ///|/
//! ///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
//! ///|/

use crate::geometry::Vec3d;

// MeasureUtils.hpp:13-15
// Utility class used to calculate distance circle-circle
// Adaptation of code found in:
// https://github.com/davideberly/GeometricTools/blob/master/GTE/Mathematics/Polynomial1.h

// MeasureUtils.hpp:17
#[derive(Debug, Clone)]
pub struct Polynomial1 {
    // MeasureUtils.hpp:93-95
    // The class is designed so that m_coefficient.size() >= 1.
    m_coefficient: Vec<f64>,
}

impl Polynomial1 {
    // MeasureUtils.hpp:20-28
    pub fn from_values(values: &[f64]) -> Self {
        // C++ 11 will call the default constructor for
        // Polynomial1<Real> p{}, so it is guaranteed that
        // values.size() > 0.
        let mut m_coefficient = vec![0.0; values.len()]; // MeasureUtils.hpp:25
        m_coefficient[..values.len()].copy_from_slice(values); // MeasureUtils.hpp:26
        let mut result = Polynomial1 { m_coefficient };
        result.eliminate_leading_zeros(); // MeasureUtils.hpp:27
        result
    }

    // Construction and destruction.  The first constructor creates a
    // polynomial of the specified degree but sets all coefficients to
    // zero (to ensure initialization).  You are responsible for setting
    // the coefficients, presumably with the degree-term set to a nonzero
    // number.  In the second constructor, the degree is the number of
    // initializers plus 1, but then adjusted so that coefficient[degree]
    // is not zero (unless all initializer values are zero).
    // MeasureUtils.hpp:37-39
    pub fn from_degree(degree: u32) -> Self {
        Polynomial1 {
            m_coefficient: vec![0.0; degree as usize + 1],
        }
    }

    // Eliminate any leading zeros in the polynomial, except in the case
    // the degree is 0 and the coefficient is 0.  The elimination is
    // necessary when arithmetic operations cause a decrease in the degree
    // of the result.  For example, (1 + x + x^2) + (1 + 2*x - x^2) =
    // (2 + 3*x).  The inputs both have degree 2, so the result is created
    // with degree 2.  After the addition we find that the degree is in
    // fact 1 and resize the array of coefficients.  This function is
    // called internally by the arithmetic operators, but it is exposed in
    // the public interface in case you need it for your own purposes.
    // MeasureUtils.hpp:50-63
    pub fn eliminate_leading_zeros(&mut self) {
        let size = self.m_coefficient.len(); // MeasureUtils.hpp:52
        if size > 1 {
            // MeasureUtils.hpp:53
            let zero = 0.0; // MeasureUtils.hpp:54
            let mut leading: i32; // MeasureUtils.hpp:55
            leading = size as i32 - 1; // MeasureUtils.hpp:56
            while leading > 0 {
                if self.m_coefficient[leading as usize] != zero {
                    // MeasureUtils.hpp:57
                    break; // MeasureUtils.hpp:58
                }
                leading -= 1; // MeasureUtils.hpp:56 (--leading)
            }

            leading += 1; // MeasureUtils.hpp:61 (++leading)
            self.m_coefficient.resize(leading as usize, 0.0); // MeasureUtils.hpp:61
        }
    }

    // Set all coefficients to the specified value.
    // MeasureUtils.hpp:66-69
    pub fn set_coefficients(&mut self, value: f64) {
        for c in self.m_coefficient.iter_mut() {
            *c = value; // MeasureUtils.hpp:68 (std::fill)
        }
    }

    // MeasureUtils.hpp:71-75
    #[inline]
    pub fn get_degree(&self) -> u32 {
        // By design, m_coefficient.size() > 0.
        (self.m_coefficient.len() - 1) as u32 // MeasureUtils.hpp:74
    }

    // Evaluate the polynomial.  If the polynomial is invalid, the
    // function returns zero.
    // MeasureUtils.hpp:82-91
    pub fn eval(&self, t: f64) -> f64 {
        let mut i = self.m_coefficient.len() as i32; // MeasureUtils.hpp:84
        i -= 1; // MeasureUtils.hpp:85 (--i)
        let mut result = self.m_coefficient[i as usize]; // MeasureUtils.hpp:85
        i -= 1; // MeasureUtils.hpp:86 (--i)
        while i >= 0 {
            result *= t; // MeasureUtils.hpp:87
            result += self.m_coefficient[i as usize]; // MeasureUtils.hpp:88
            i -= 1; // MeasureUtils.hpp:86 (--i)
        }
        result // MeasureUtils.hpp:90
    }
}

// MeasureUtils.hpp:77-78
// inline const double& operator[](uint32_t i) const { return m_coefficient[i]; }
// inline double& operator[](uint32_t i) { return m_coefficient[i]; }
impl std::ops::Index<u32> for Polynomial1 {
    type Output = f64;
    #[inline]
    fn index(&self, i: u32) -> &f64 {
        &self.m_coefficient[i as usize]
    }
}

impl std::ops::IndexMut<u32> for Polynomial1 {
    #[inline]
    fn index_mut(&mut self, i: u32) -> &mut f64 {
        &mut self.m_coefficient[i as usize]
    }
}

// MeasureUtils.hpp:98-110
// inline Polynomial1 operator * (const Polynomial1& p0, const Polynomial1& p1)
impl std::ops::Mul<&Polynomial1> for &Polynomial1 {
    type Output = Polynomial1;
    fn mul(self, p1: &Polynomial1) -> Polynomial1 {
        let p0 = self;
        let p0_degree = p0.get_degree(); // MeasureUtils.hpp:100
        let p1_degree = p1.get_degree(); // MeasureUtils.hpp:101
        let mut result = Polynomial1::from_degree(p0_degree + p1_degree); // MeasureUtils.hpp:102
        result.set_coefficients(0.0); // MeasureUtils.hpp:103
        for i0 in 0..=p0_degree {
            // MeasureUtils.hpp:104
            for i1 in 0..=p1_degree {
                // MeasureUtils.hpp:105
                result[i0 + i1] += p0[i0] * p1[i1]; // MeasureUtils.hpp:106
            }
        }
        result // MeasureUtils.hpp:109
    }
}

// MeasureUtils.hpp:112-139
// inline Polynomial1 operator + (const Polynomial1& p0, const Polynomial1& p1)
impl std::ops::Add<&Polynomial1> for &Polynomial1 {
    type Output = Polynomial1;
    fn add(self, p1: &Polynomial1) -> Polynomial1 {
        let p0 = self;
        let p0_degree = p0.get_degree(); // MeasureUtils.hpp:114
        let p1_degree = p1.get_degree(); // MeasureUtils.hpp:115
        let mut i: u32; // MeasureUtils.hpp:116
        if p0_degree >= p1_degree {
            // MeasureUtils.hpp:117
            let mut result = Polynomial1::from_degree(p0_degree); // MeasureUtils.hpp:118
            i = 0;
            while i <= p1_degree {
                // MeasureUtils.hpp:119
                result[i] = p0[i] + p1[i]; // MeasureUtils.hpp:120
                i += 1;
            }
            while i <= p0_degree {
                // MeasureUtils.hpp:122
                result[i] = p0[i]; // MeasureUtils.hpp:123
                i += 1;
            }
            result.eliminate_leading_zeros(); // MeasureUtils.hpp:125
            result // MeasureUtils.hpp:126
        } else {
            let mut result = Polynomial1::from_degree(p1_degree); // MeasureUtils.hpp:129
            i = 0;
            while i <= p0_degree {
                // MeasureUtils.hpp:130
                result[i] = p0[i] + p1[i]; // MeasureUtils.hpp:131
                i += 1;
            }
            while i <= p1_degree {
                // MeasureUtils.hpp:133
                result[i] = p1[i]; // MeasureUtils.hpp:134
                i += 1;
            }
            result.eliminate_leading_zeros(); // MeasureUtils.hpp:136
            result // MeasureUtils.hpp:137
        }
    }
}

// MeasureUtils.hpp:141-168
// inline Polynomial1 operator - (const Polynomial1& p0, const Polynomial1& p1)
impl std::ops::Sub<&Polynomial1> for &Polynomial1 {
    type Output = Polynomial1;
    fn sub(self, p1: &Polynomial1) -> Polynomial1 {
        let p0 = self;
        let p0_degree = p0.get_degree(); // MeasureUtils.hpp:143
        let p1_degree = p1.get_degree(); // MeasureUtils.hpp:144
        let mut i: u32; // MeasureUtils.hpp:145
        if p0_degree >= p1_degree {
            // MeasureUtils.hpp:146
            let mut result = Polynomial1::from_degree(p0_degree); // MeasureUtils.hpp:147
            i = 0;
            while i <= p1_degree {
                // MeasureUtils.hpp:148
                result[i] = p0[i] - p1[i]; // MeasureUtils.hpp:149
                i += 1;
            }
            while i <= p0_degree {
                // MeasureUtils.hpp:151
                result[i] = p0[i]; // MeasureUtils.hpp:152
                i += 1;
            }
            result.eliminate_leading_zeros(); // MeasureUtils.hpp:154
            result // MeasureUtils.hpp:155
        } else {
            let mut result = Polynomial1::from_degree(p1_degree); // MeasureUtils.hpp:157
            i = 0;
            while i <= p0_degree {
                // MeasureUtils.hpp:159
                result[i] = p0[i] - p1[i]; // MeasureUtils.hpp:160
                i += 1;
            }
            while i <= p1_degree {
                // MeasureUtils.hpp:162
                result[i] = -p1[i]; // MeasureUtils.hpp:163
                i += 1;
            }
            result.eliminate_leading_zeros(); // MeasureUtils.hpp:165
            result // MeasureUtils.hpp:166
        }
    }
}

// MeasureUtils.hpp:170-178
// inline Polynomial1 operator * (double scalar, const Polynomial1& p)
impl std::ops::Mul<&Polynomial1> for f64 {
    type Output = Polynomial1;
    fn mul(self, p: &Polynomial1) -> Polynomial1 {
        let scalar = self;
        let degree = p.get_degree(); // MeasureUtils.hpp:172
        let mut result = Polynomial1::from_degree(degree); // MeasureUtils.hpp:173
        for i in 0..=degree {
            // MeasureUtils.hpp:174
            result[i] = scalar * p[i]; // MeasureUtils.hpp:175
        }
        result // MeasureUtils.hpp:177
    }
}

// MeasureUtils.hpp:180-182
// Utility class used to calculate distance circle-circle
// Adaptation of code found in:
// https://github.com/davideberly/GeometricTools/blob/master/GTE/Mathematics/RootsPolynomial.h

// MeasureUtils.hpp:184
pub struct RootsPolynomial;

impl RootsPolynomial {
    // General equations: sum_{i=0}^{d} c(i)*t^i = 0.  The input array 'c'
    // must have at least d+1 elements and the output array 'root' must
    // have at least d elements.

    // Find the roots on (-infinity,+infinity).
    // MeasureUtils.hpp:192-226
    pub fn find(mut degree: i32, c: &[f64], max_iterations: u32, roots: &mut [f64]) -> i32 {
        // In C++ the guard is `degree >= 0 && c != nullptr`; the slice is
        // always non-null here, so we only check the degree.
        if degree >= 0 {
            // MeasureUtils.hpp:194
            let zero = 0.0; // MeasureUtils.hpp:195
            while degree >= 0 && c[degree as usize] == zero {
                // MeasureUtils.hpp:196
                degree -= 1; // MeasureUtils.hpp:197
            }

            if degree > 0 {
                // MeasureUtils.hpp:200
                // Compute the Cauchy bound.
                let one = 1.0; // MeasureUtils.hpp:202
                let inv_leading = one / c[degree as usize]; // MeasureUtils.hpp:203
                let mut max_value = zero; // MeasureUtils.hpp:204
                for i in 0..degree {
                    // MeasureUtils.hpp:205
                    let value = (c[i as usize] * inv_leading).abs(); // MeasureUtils.hpp:206
                    if value > max_value {
                        // MeasureUtils.hpp:207
                        max_value = value; // MeasureUtils.hpp:208
                    }
                }
                let bound = one + max_value; // MeasureUtils.hpp:210

                Self::find_recursive(degree, c, -bound, bound, max_iterations, roots)
            // MeasureUtils.hpp:212
            } else if degree == 0 {
                // MeasureUtils.hpp:214
                // The polynomial is a nonzero constant.
                0 // MeasureUtils.hpp:216
            } else {
                // The polynomial is identically zero.
                roots[0] = zero; // MeasureUtils.hpp:219
                1 // MeasureUtils.hpp:220
            }
        } else {
            // Invalid degree or c.
            0 // MeasureUtils.hpp:225
        }
    }

    // If you know that p(tmin) * p(tmax) <= 0, then there must be at
    // least one root in [tmin, tmax].  Compute it using bisection.
    // MeasureUtils.hpp:230-275
    // (overload of `Find` taking an interval [tmin, tmax] and a single root)
    pub fn find_bisection(
        degree: i32,
        c: &[f64],
        mut tmin: f64,
        mut tmax: f64,
        max_iterations: u32,
        root: &mut f64,
    ) -> bool {
        let zero = 0.0; // MeasureUtils.hpp:232
        let mut pmin = Self::evaluate(degree, c, tmin); // MeasureUtils.hpp:233
        if pmin == zero {
            // MeasureUtils.hpp:234
            *root = tmin; // MeasureUtils.hpp:235
            return true; // MeasureUtils.hpp:236
        }
        let mut pmax = Self::evaluate(degree, c, tmax); // MeasureUtils.hpp:238
        if pmax == zero {
            // MeasureUtils.hpp:239
            *root = tmax; // MeasureUtils.hpp:240
            return true; // MeasureUtils.hpp:241
        }

        if pmin * pmax > zero {
            // MeasureUtils.hpp:244
            // It is not known whether the interval bounds a root.
            return false; // MeasureUtils.hpp:246
        }

        if tmin >= tmax {
            // MeasureUtils.hpp:248
            // Invalid ordering of interval endpoitns.
            return false; // MeasureUtils.hpp:250
        }

        let mut i = 1; // MeasureUtils.hpp:252
        while i <= max_iterations {
            *root = 0.5 * (tmin + tmax); // MeasureUtils.hpp:253

            // This test is designed for 'float' or 'double' when tmin
            // and tmax are consecutive floating-point numbers.
            if *root == tmin || *root == tmax {
                // MeasureUtils.hpp:257
                break; // MeasureUtils.hpp:258
            }

            let p = Self::evaluate(degree, c, *root); // MeasureUtils.hpp:260
            let product = p * pmin; // MeasureUtils.hpp:261
            if product < zero {
                // MeasureUtils.hpp:262
                tmax = *root; // MeasureUtils.hpp:263
                pmax = p; // MeasureUtils.hpp:264
                let _ = pmax;
            } else if product > zero {
                // MeasureUtils.hpp:266
                tmin = *root; // MeasureUtils.hpp:267
                pmin = p; // MeasureUtils.hpp:268
            } else {
                break; // MeasureUtils.hpp:271
            }
            i += 1; // MeasureUtils.hpp:252 (++i)
        }

        true // MeasureUtils.hpp:274
    }

    // Support for the Find functions.
    // MeasureUtils.hpp:278-339
    pub fn find_recursive(
        degree: i32,
        c: &[f64],
        tmin: f64,
        tmax: f64,
        max_iterations: u32,
        roots: &mut [f64],
    ) -> i32 {
        // The base of the recursion.
        let zero = 0.0; // MeasureUtils.hpp:281
        let mut root = zero; // MeasureUtils.hpp:282
        if degree == 1 {
            // MeasureUtils.hpp:283
            let num_roots: i32; // MeasureUtils.hpp:284
            if c[1] != zero {
                // MeasureUtils.hpp:285
                root = -c[0] / c[1]; // MeasureUtils.hpp:286
                num_roots = 1; // MeasureUtils.hpp:287
            } else if c[0] == zero {
                // MeasureUtils.hpp:289
                root = zero; // MeasureUtils.hpp:290
                num_roots = 1; // MeasureUtils.hpp:291
            } else {
                num_roots = 0; // MeasureUtils.hpp:294
            }

            if num_roots > 0 && tmin <= root && root <= tmax {
                // MeasureUtils.hpp:296
                roots[0] = root; // MeasureUtils.hpp:297
                return 1; // MeasureUtils.hpp:298
            }
            return 0; // MeasureUtils.hpp:300
        }

        // Find the roots of the derivative polynomial scaled by 1/degree.
        // The scaling avoids the factorial growth in the coefficients;
        // for example, without the scaling, the high-order term x^d
        // becomes (d!)*x through multiple differentiations.  With the
        // scaling we instead get x.  This leads to better numerical
        // behavior of the root finder.
        let deriv_degree = degree - 1; // MeasureUtils.hpp:309
        let mut deriv_coeff = vec![0.0f64; deriv_degree as usize + 1]; // MeasureUtils.hpp:310
        let mut deriv_roots = vec![0.0f64; deriv_degree as usize]; // MeasureUtils.hpp:311
        {
            let mut i = 0; // MeasureUtils.hpp:312
            let mut ip1 = 1;
            while i <= deriv_degree {
                deriv_coeff[i as usize] = c[ip1 as usize] * (ip1 as f64) / (degree as f64); // MeasureUtils.hpp:313
                i += 1;
                ip1 += 1;
            }
        }
        let num_deriv_roots = Self::find_recursive(
            degree - 1,
            &deriv_coeff[..],
            tmin,
            tmax,
            max_iterations,
            &mut deriv_roots[..],
        ); // MeasureUtils.hpp:315

        let mut num_roots = 0; // MeasureUtils.hpp:317
        if num_deriv_roots > 0 {
            // MeasureUtils.hpp:318
            // Find root on [tmin,derivRoots[0]].
            if Self::find_bisection(degree, c, tmin, deriv_roots[0], max_iterations, &mut root) {
                // MeasureUtils.hpp:320
                roots[num_roots as usize] = root; // MeasureUtils.hpp:321
                num_roots += 1;
            }

            // Find root on [derivRoots[i],derivRoots[i+1]].
            {
                let mut i = 0; // MeasureUtils.hpp:324
                let mut ip1 = 1;
                while i <= num_deriv_roots - 2 {
                    if Self::find_bisection(
                        degree,
                        c,
                        deriv_roots[i as usize],
                        deriv_roots[ip1 as usize],
                        max_iterations,
                        &mut root,
                    ) {
                        // MeasureUtils.hpp:325
                        roots[num_roots as usize] = root; // MeasureUtils.hpp:326
                        num_roots += 1;
                    }
                    i += 1;
                    ip1 += 1;
                }
            }

            // Find root on [derivRoots[numDerivRoots-1],tmax].
            if Self::find_bisection(
                degree,
                c,
                deriv_roots[num_deriv_roots as usize - 1],
                tmax,
                max_iterations,
                &mut root,
            ) {
                // MeasureUtils.hpp:330
                roots[num_roots as usize] = root; // MeasureUtils.hpp:331
                num_roots += 1;
            }
        } else {
            // The polynomial is monotone on [tmin,tmax], so has at most one root.
            if Self::find_bisection(degree, c, tmin, tmax, max_iterations, &mut root) {
                // MeasureUtils.hpp:335
                roots[num_roots as usize] = root; // MeasureUtils.hpp:336
                num_roots += 1;
            }
        }
        num_roots // MeasureUtils.hpp:338
    }

    // MeasureUtils.hpp:341-349
    pub fn evaluate(degree: i32, c: &[f64], t: f64) -> f64 {
        let mut i = degree; // MeasureUtils.hpp:343
        let mut result = c[i as usize]; // MeasureUtils.hpp:344
        i -= 1; // MeasureUtils.hpp:345 (--i)
        while i >= 0 {
            result = t * result + c[i as usize]; // MeasureUtils.hpp:346
            i -= 1; // MeasureUtils.hpp:345 (--i)
        }
        result // MeasureUtils.hpp:348
    }
}

// MeasureUtils.hpp:352-353
// Adaptation of code found in:
// https://github.com/davideberly/GeometricTools/blob/master/GTE/Mathematics/Vector.h

// Construct a single vector orthogonal to the nonzero input vector.  If
// the maximum absolute component occurs at index i, then the orthogonal
// vector U has u[i] = v[i+1], u[i+1] = -v[i], and all other components
// zero.  The index addition i+1 is computed modulo N.
// MeasureUtils.hpp:359-385
pub fn get_orthogonal(v: &Vec3d, unit_length: bool) -> Vec3d {
    let mut cmax = v.component(0).abs(); // MeasureUtils.hpp:361
    let mut imax = 0i32; // MeasureUtils.hpp:362
    for i in 1..3 {
        // MeasureUtils.hpp:363
        let c = v.component(i as usize).abs(); // MeasureUtils.hpp:364
        if c > cmax {
            // MeasureUtils.hpp:365
            cmax = c; // MeasureUtils.hpp:366
            imax = i; // MeasureUtils.hpp:367
        }
    }
    let _ = cmax;

    let mut result = Vec3d::zero(); // MeasureUtils.hpp:371
    let mut inext = imax + 1; // MeasureUtils.hpp:372
    if inext == 3 {
        // MeasureUtils.hpp:373
        inext = 0; // MeasureUtils.hpp:374
    }

    result.set_component(imax as usize, v.component(inext as usize)); // MeasureUtils.hpp:376
    result.set_component(inext as usize, -v.component(imax as usize)); // MeasureUtils.hpp:377
    if unit_length {
        // MeasureUtils.hpp:378
        let sqr_distance = result.component(imax as usize) * result.component(imax as usize)
            + result.component(inext as usize) * result.component(inext as usize); // MeasureUtils.hpp:379
        let inv_length = 1.0 / sqr_distance.sqrt(); // MeasureUtils.hpp:380
        result.set_component(
            imax as usize,
            result.component(imax as usize) * inv_length,
        ); // MeasureUtils.hpp:381
        result.set_component(
            inext as usize,
            result.component(inext as usize) * inv_length,
        ); // MeasureUtils.hpp:382
    }
    result // MeasureUtils.hpp:384
}
