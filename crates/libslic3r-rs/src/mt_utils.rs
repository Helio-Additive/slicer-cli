//! 1:1 port of `MTUtils.hpp` from BambuStudio's libslic3r.
//!
//! C++ Reference:
//! - src/libslic3r/MTUtils.hpp
//!
//! This is a header-only utility file. It provides a spin mutex, a thread-safe
//! cached-object wrapper, and a handful of arithmetic-sequence helpers
//! (`linspace_vector`, `linspace_array`, `grid`) plus an `all_of` convenience.

// MTUtils.hpp:4 #include <atomic>       // for std::atomic_flag and memory orders
use std::sync::atomic::{AtomicBool, Ordering};
// MTUtils.hpp:5-7 #include <mutex>/<functional>/<utility> handled via std/closures below.
use std::ops::{Add, Div, Mul, Sub};

// MTUtils.hpp:14 namespace Slic3r { ... }

/// Handy little spin mutex for the cached meshes.
/// Implements the "Lockable" concept
// MTUtils.hpp:16-29 class SpinMutex
pub struct SpinMutex {
    // MTUtils.hpp:20 std::atomic_flag m_flg;
    m_flg: AtomicBool,
}

impl SpinMutex {
    // MTUtils.hpp:21 static const auto MO_ACQ = std::memory_order_acquire;
    const MO_ACQ: Ordering = Ordering::Acquire;
    // MTUtils.hpp:22 static const auto MO_REL = std::memory_order_release;
    const MO_REL: Ordering = Ordering::Release;

    // MTUtils.hpp:25 inline SpinMutex() { m_flg.clear(MO_REL); }
    #[inline]
    pub fn new() -> Self {
        let s = SpinMutex { m_flg: AtomicBool::new(false) };
        s.m_flg.store(false, Self::MO_REL);
        s
    }

    // MTUtils.hpp:26 inline void lock() { while (m_flg.test_and_set(MO_ACQ)) ; }
    #[inline]
    pub fn lock(&self) {
        while self.m_flg.swap(true, Self::MO_ACQ) {}
    }

    // MTUtils.hpp:27 inline bool try_lock() { return !m_flg.test_and_set(MO_ACQ); }
    #[inline]
    pub fn try_lock(&self) -> bool {
        !self.m_flg.swap(true, Self::MO_ACQ)
    }

    // MTUtils.hpp:28 inline void unlock() { m_flg.clear(MO_REL); }
    #[inline]
    pub fn unlock(&self) {
        self.m_flg.store(false, Self::MO_REL);
    }
}

impl Default for SpinMutex {
    fn default() -> Self {
        Self::new()
    }
}

/// A wrapper class around arbitrary object that needs thread safe caching.
// MTUtils.hpp:31-75 template<class T> class CachedObject
pub struct CachedObject<T> {
    // MTUtils.hpp:39 T m_obj; // the object itself
    m_obj: T,
    // MTUtils.hpp:40 bool m_valid; // invalidation flag
    m_valid: bool,
    // MTUtils.hpp:41 SpinMutex m_lck; // to make the caching thread safe
    m_lck: SpinMutex,

    // the setter will be called just before the object's const value is
    // about to be retrieved.
    // MTUtils.hpp:45 std::function<void(T &)> m_setter;
    m_setter: Setter<T>,
}

// MTUtils.hpp:36 using Setter = std::function<void(T &)>;
// Method type which refreshes the object when it has been invalidated
pub type Setter<T> = Box<dyn Fn(&mut T) + Send + Sync>;

impl<T> CachedObject<T> {
    // Forwarded constructor
    // MTUtils.hpp:49-52 template<class... Args>
    //     inline CachedObject(Setter &&fn, Args &&... args)
    //         : m_obj(...), m_valid(false), m_setter(fn) {}
    #[inline]
    pub fn new(fn_: Setter<T>, obj: T) -> Self {
        CachedObject { m_obj: obj, m_valid: false, m_lck: SpinMutex::new(), m_setter: fn_ }
    }

    // invalidate the value of the object. The object will be refreshed at
    // the next retrieval (Setter will be called). The data that is used in
    // the setter function should be guarded as well during modification so
    // the modification has to take place in fn.
    // MTUtils.hpp:58-63 template<class Fn> void invalidate(Fn &&fn)
    pub fn invalidate<F: FnOnce()>(&mut self, fn_: F) {
        // MTUtils.hpp:60 std::lock_guard<SpinMutex> lck(m_lck);
        self.m_lck.lock();
        // MTUtils.hpp:61 fn();
        fn_();
        // MTUtils.hpp:62 m_valid = false;
        self.m_valid = false;
        self.m_lck.unlock();
    }

    // Get the const object properly updated.
    // MTUtils.hpp:66-74 inline const T &get()
    #[inline]
    pub fn get(&mut self) -> &T {
        // MTUtils.hpp:68 std::lock_guard<SpinMutex> lck(m_lck);
        self.m_lck.lock();
        // MTUtils.hpp:69 if (!m_valid) {
        if !self.m_valid {
            // MTUtils.hpp:70 m_setter(m_obj);
            (self.m_setter)(&mut self.m_obj);
            // MTUtils.hpp:71 m_valid = true;
            self.m_valid = true;
        }
        self.m_lck.unlock();
        // MTUtils.hpp:73 return m_obj;
        &self.m_obj
    }
}

// MTUtils.hpp:77-84 template<class C> bool all_of(const C &container)
pub fn all_of<I, V>(container: I) -> bool
where
    I: IntoIterator<Item = V>,
    V: Into<bool>,
{
    // MTUtils.hpp:79-83 return std::all_of(container.begin(), container.end(),
    //     [](const typename C::value_type &v) { return static_cast<bool>(v); });
    container.into_iter().all(|v| v.into())
}

//template<class T>
//using remove_cvref_t = std::remove_reference_t<std::remove_cv_t<T>>;
// MTUtils.hpp:86-87

/// Exactly like Matlab <https://www.mathworks.com/help/matlab/ref/linspace.html>
// MTUtils.hpp:89-104 template<class T, class I, class = IntegerOnly<I>>
//     inline std::vector<T> linspace_vector(const ArithmeticOnly<T> &start,
//                                           const T &stop, const I &n)
#[inline]
pub fn linspace_vector<T>(start: T, stop: T, n: usize) -> Vec<T>
where
    T: Copy
        + Default
        + Add<Output = T>
        + Sub<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
        + From<u32>,
{
    // MTUtils.hpp:95 std::vector<T> vals(n, T());
    let mut vals: Vec<T> = vec![T::default(); n];

    // MTUtils.hpp:97 T stride = (stop - start) / n;
    // C++ divides by `n` (the integer count) promoted to T.
    let stride: T = (stop - start) / T::from(n as u32);
    // MTUtils.hpp:98 size_t i = 0;
    let mut i: usize = 0;
    // MTUtils.hpp:99-101 std::generate(..., [&i, start, stride] {
    //     return start + i++ * stride; });
    for v in vals.iter_mut() {
        *v = start + T::from(i as u32) * stride;
        i += 1;
    }

    // MTUtils.hpp:103 return vals;
    vals
}

// MTUtils.hpp:106-118 template<size_t N, class T>
//     inline std::array<ArithmeticOnly<T>, N> linspace_array(const T &start, const T &stop)
#[inline]
pub fn linspace_array<const N: usize, T>(start: T, stop: T) -> [T; N]
where
    T: Copy
        + Default
        + Add<Output = T>
        + Sub<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
        + From<u32>,
{
    // MTUtils.hpp:109 std::array<T, N> vals = {T()};
    let mut vals: [T; N] = [T::default(); N];

    // MTUtils.hpp:111 T stride = (stop - start) / N;
    let stride: T = (stop - start) / T::from(N as u32);
    // MTUtils.hpp:112 size_t i = 0;
    let mut i: usize = 0;
    // MTUtils.hpp:113-115 std::generate(..., [&i, start, stride] {
    //     return start + i++ * stride; });
    for v in vals.iter_mut() {
        *v = start + T::from(i as u32) * stride;
        i += 1;
    }

    // MTUtils.hpp:117 return vals;
    vals
}

/// A set of equidistant values starting from 'start' (inclusive), ending
/// in the closest multiple of 'stride' less than or equal to 'end' and
/// leaving 'stride' space between each value.
/// Very similar to Matlab \[start:stride:end\] notation.
// MTUtils.hpp:120-137 template<class T>
//     inline std::vector<ArithmeticOnly<T>> grid(const T &start, const T &stop, const T &stride)
#[inline]
pub fn grid<T>(start: T, stop: T, stride: T) -> Vec<T>
where
    T: Copy
        + Default
        + Add<Output = T>
        + Sub<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
        + From<i32>
        + Into<f64>,
{
    // MTUtils.hpp:129 std::vector<T> vals(size_t(std::ceil((stop - start) / stride)), T());
    let count: usize = (((stop - start) / stride).into()).ceil() as usize;
    let mut vals: Vec<T> = vec![T::default(); count];

    // MTUtils.hpp:131 int i = 0;
    let mut i: i32 = 0;
    // MTUtils.hpp:132-134 std::generate(..., [&i, start, stride] {
    //     return start + i++ * stride; });
    for v in vals.iter_mut() {
        *v = start + T::from(i) * stride;
        i += 1;
    }

    // MTUtils.hpp:136 return vals;
    vals
}

/// MTUtils.hpp:120-137 — `grid<T>` instantiated at `T = float`
/// (used by `SLA/Pad.cpp:512` and `SLA/SupportTreeBuilder.cpp:40`).
///
/// The generic [`grid`] above cannot be instantiated at `f32` because
/// `f32: From<i32>` does not exist in Rust; this is the identical algorithm
/// with the C++ implicit `int -> float` promotion written as `as` casts, and
/// all arithmetic carried out in `f32` exactly as the `T = float`
/// instantiation does.
#[inline]
pub fn grid_f32(start: f32, stop: f32, stride: f32) -> Vec<f32> {
    // MTUtils.hpp:129 std::vector<T> vals(size_t(std::ceil((stop - start) / stride)), T());
    let count: usize = ((stop - start) / stride).ceil() as usize;
    let mut vals: Vec<f32> = vec![0.0f32; count];

    // MTUtils.hpp:131 int i = 0;
    let mut i: i32 = 0;
    // MTUtils.hpp:132-134 std::generate(..., [&i, start, stride] {
    //     return start + i++ * stride; });
    for v in vals.iter_mut() {
        *v = start + (i as f32) * stride;
        i += 1;
    }

    // MTUtils.hpp:136 return vals;
    vals
}

// MTUtils.hpp:139 } // namespace Slic3r
