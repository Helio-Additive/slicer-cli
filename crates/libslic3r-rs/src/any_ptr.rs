//! Faithful 1:1 port of BambuStudio `src/libslic3r/AnyPtr.hpp`.
//!
//! Header-only template class. The C++ has no `.cpp` translation unit; all
//! logic lives in the header and is reproduced here line-by-line.

use std::sync::Arc;

// AnyPtr.hpp:8  namespace Slic3r {

// AnyPtr.hpp:10-25
// A general purpose pointer holder that can hold any type of smart pointer
// or raw pointer which can own or not own any object they point to.
// In case a raw pointer is stored, it is not destructed so ownership is
// assumed to be foreign.
//
// The stored pointer is not checked for being null when dereferenced.
//
// This is a movable only object due to the fact that it can possibly hold
// a unique_ptr which can only be moved.
//
// Drawbacks:
// No custom deleters are supported when storing a unique_ptr, but overloading
// std::default_delete for a particular type could be a workaround
//
// raw array types are problematic, since std::default_delete also does not
// support them well.
//
// AnyPtr.hpp:26  template<class T> class AnyPtr
//
// C++ backs the storage with `boost::variant<T*, std::unique_ptr<T>,
// std::shared_ptr<T>>`. The Rust equivalent is an enum with the same three
// alternatives in the same order:
//   - RawPtr -> `*mut T`           (non-owning, foreign ownership)
//   - UPtr   -> `Box<T>`           (std::unique_ptr<T>)
//   - ShPtr  -> `Arc<T>`           (std::shared_ptr<T>; BambuStudio's
//                                   shared_ptr is thread-safe, hence Arc)
//
// This is a move-only type: there is intentionally no `Clone`/`Copy` impl,
// mirroring `AnyPtr(const AnyPtr &other) = delete;` (AnyPtr.hpp:62) and
// `AnyPtr &operator=(const AnyPtr &other) = delete;` (AnyPtr.hpp:70).
#[derive(Debug)]
pub enum AnyPtr<T> {
    // AnyPtr.hpp:30  boost::variant alternative 0: T*
    RawPtr(*mut T),
    // AnyPtr.hpp:30  boost::variant alternative 1: std::unique_ptr<T>
    UPtr(Box<T>),
    // AnyPtr.hpp:30  boost::variant alternative 2: std::shared_ptr<T>
    ShPtr(Arc<T>),
}

// AnyPtr.hpp:28  enum { RawPtr, UPtr, ShPtr };
// The boost::variant discriminant values (the `which()` indices) that the C++
// switch statements compare against. Preserved as constants for the line-by-line
// translation of those switches.
const RAW_PTR: usize = 0;
const U_PTR: usize = 1;
const SH_PTR: usize = 2;

impl<T> AnyPtr<T> {
    // AnyPtr.hpp:32-41  template<class Self> static T *get_ptr(Self &&s)
    //
    //     switch (s.ptr.which()) {
    //     case RawPtr: return boost::get<T *>(s.ptr);
    //     case UPtr: return boost::get<std::unique_ptr<T>>(s.ptr).get();
    //     case ShPtr: return boost::get<std::shared_ptr<T>>(s.ptr).get();
    //     }
    //     return nullptr;
    //
    // In C++ this is a single templated helper used by both the const and
    // non-const `operator*`/`operator->`/`get`. Rust splits it into a shared
    // implementation returning `*mut T` (the borrow checker provides the
    // const/non-const distinction at the call sites).
    fn get_ptr(&self) -> *mut T {
        match self {
            // case RawPtr: return boost::get<T *>(s.ptr);   AnyPtr.hpp:35
            AnyPtr::RawPtr(p) => *p,
            // case UPtr: ... .get();                        AnyPtr.hpp:36
            AnyPtr::UPtr(b) => b.as_ref() as *const T as *mut T,
            // case ShPtr: ... .get();                       AnyPtr.hpp:37
            AnyPtr::ShPtr(a) => Arc::as_ptr(a) as *mut T,
        }
    }

    // AnyPtr.hpp:34  s.ptr.which()
    // Returns the boost::variant discriminant index for the held alternative.
    fn which(&self) -> usize {
        match self {
            AnyPtr::RawPtr(_) => RAW_PTR,
            AnyPtr::UPtr(_) => U_PTR,
            AnyPtr::ShPtr(_) => SH_PTR,
        }
    }

    // AnyPtr.hpp:48  AnyPtr() noexcept = default;
    //
    // A default-constructed boost::variant holds its first alternative
    // (`T*`) value-initialized, i.e. a null raw pointer.
    pub fn new() -> Self {
        AnyPtr::RawPtr(std::ptr::null_mut())
    }

    // AnyPtr.hpp:50  AnyPtr(T *p) noexcept : ptr{p} {}
    // AnyPtr.hpp:54  template<class TT, class = SimilarPtrOnly<TT>> AnyPtr(TT *p) noexcept : ptr{p} {}
    //
    // The `SimilarPtrOnly` / `is_convertible_v<TT*, T*>` overloads exist in
    // C++ purely to accept pointers to derived types; Rust lacks inheritance
    // pointer conversions, so they collapse into this single constructor.
    pub fn from_raw(p: *mut T) -> Self {
        AnyPtr::RawPtr(p)
    }

    // AnyPtr.hpp:52  AnyPtr(std::nullptr_t) noexcept {};
    //
    // Constructing from nullptr leaves the default (null raw pointer) state.
    pub fn from_null() -> Self {
        AnyPtr::RawPtr(std::ptr::null_mut())
    }

    // AnyPtr.hpp:55  template<class TT = T, class = SimilarPtrOnly<TT>>
    //                AnyPtr(std::unique_ptr<TT> p) noexcept
    //                    : ptr{std::unique_ptr<T>(std::move(p))} {}
    pub fn from_unique(p: Box<T>) -> Self {
        AnyPtr::UPtr(p)
    }

    // AnyPtr.hpp:56  template<class TT = T, class = SimilarPtrOnly<TT>>
    //                AnyPtr(std::shared_ptr<TT> p) noexcept
    //                    : ptr{std::shared_ptr<T>(std::move(p))} {}
    pub fn from_shared(p: Arc<T>) -> Self {
        AnyPtr::ShPtr(p)
    }

    // AnyPtr.hpp:58  AnyPtr(AnyPtr &&other) noexcept : ptr{std::move(other.ptr)} {}
    // AnyPtr.hpp:60  template<class TT, ...> AnyPtr(AnyPtr<TT> &&other) noexcept ...
    //
    // Move construction is provided natively by Rust's move semantics; no
    // explicit method is required (and the cross-type derived overload has no
    // Rust analogue).

    // AnyPtr.hpp:101  const T &operator*() const noexcept { return *get_ptr(*this); }
    // AnyPtr.hpp:102  T &      operator*() noexcept { return *get_ptr(*this); }
    //
    // The stored pointer is not checked for being null when dereferenced
    // (AnyPtr.hpp:15). `unsafe`: the caller upholds that invariant, exactly as
    // C++ dereferences `get_ptr` without a null check.
    #[allow(clippy::should_implement_trait)]
    pub fn as_ref(&self) -> &T {
        unsafe { &*self.get_ptr() }
    }

    // AnyPtr.hpp:102  T &operator*() noexcept { return *get_ptr(*this); }
    pub fn as_mut(&mut self) -> &mut T {
        unsafe { &mut *self.get_ptr() }
    }

    // AnyPtr.hpp:104  T *      operator->() noexcept { return get_ptr(*this); }
    // AnyPtr.hpp:105  const T *operator->() const noexcept { return get_ptr(*this); }
    // AnyPtr.hpp:107  T *      get() noexcept { return get_ptr(*this); }
    // AnyPtr.hpp:108  const T *get() const noexcept { return get_ptr(*this); }
    pub fn get(&self) -> *mut T {
        self.get_ptr()
    }

    // AnyPtr.hpp:110-119  operator bool() const noexcept
    //
    //     switch (ptr.which()) {
    //     case RawPtr: return bool(boost::get<T *>(ptr));
    //     case UPtr: return bool(boost::get<std::unique_ptr<T>>(ptr));
    //     case ShPtr: return bool(boost::get<std::shared_ptr<T>>(ptr));
    //     }
    //     return false;
    //
    // For RawPtr the result is whether the raw pointer is non-null. A Box / Arc
    // in Rust can never be empty (they always own a value), so the UPtr / ShPtr
    // arms are always true, matching a non-null unique_ptr / shared_ptr.
    #[allow(clippy::wrong_self_convention)]
    pub fn is_valid(&self) -> bool {
        match self.which() {
            // case RawPtr: return bool(boost::get<T *>(ptr));
            RAW_PTR => match self {
                AnyPtr::RawPtr(p) => !p.is_null(),
                _ => unreachable!(),
            },
            // case UPtr: return bool(boost::get<std::unique_ptr<T>>(ptr));
            U_PTR => true,
            // case ShPtr: return bool(boost::get<std::shared_ptr<T>>(ptr));
            SH_PTR => true,
            // return false;
            _ => false,
        }
    }

    // AnyPtr.hpp:121-130
    // If the stored pointer is a shared pointer, returns a reference
    // counted copy. Empty shared pointer is returned otherwise.
    //
    //     std::shared_ptr<T> get_shared_cpy() const noexcept
    //     {
    //         std::shared_ptr<T> ret;
    //         if (ptr.which() == ShPtr) ret = boost::get<std::shared_ptr<T>>(ptr);
    //         return ret;
    //     }
    //
    // C++ returns an empty shared_ptr when the variant is not ShPtr; the Rust
    // equivalent of an empty shared_ptr is `None`.
    pub fn get_shared_cpy(&self) -> Option<Arc<T>> {
        let mut ret: Option<Arc<T>> = None;

        if self.which() == SH_PTR {
            if let AnyPtr::ShPtr(a) = self {
                ret = Some(Arc::clone(a));
            }
        }

        ret
    }

    // AnyPtr.hpp:132-136
    // If the underlying pointer is unique, convert to shared pointer
    //
    //     void convert_unique_to_shared() noexcept
    //     {
    //         if (ptr.which() == UPtr)
    //             ptr = std::shared_ptr<T>{std::move(boost::get<std::unique_ptr<T>>(ptr))};
    //     }
    pub fn convert_unique_to_shared(&mut self) {
        if self.which() == U_PTR {
            // Extract the Box, then replace `self` with the shared pointer.
            // The temporary default state (null raw pointer) is never observed
            // because it is immediately overwritten, matching the in-place
            // C++ `ptr = ...` assignment.
            let taken = std::mem::replace(self, AnyPtr::RawPtr(std::ptr::null_mut()));
            if let AnyPtr::UPtr(b) = taken {
                *self = AnyPtr::ShPtr(Arc::from(b));
            }
        }
    }

    // AnyPtr.hpp:138-139
    // Returns true if the data is owned by this AnyPtr instance
    //     bool is_owned() const noexcept { return ptr.which() == UPtr || ptr.which() == ShPtr; }
    pub fn is_owned(&self) -> bool {
        self.which() == U_PTR || self.which() == SH_PTR
    }
}

// AnyPtr.hpp:48  AnyPtr() noexcept = default;
impl<T> Default for AnyPtr<T> {
    fn default() -> Self {
        AnyPtr::new()
    }
}

// AnyPtr.hpp:50,54  AnyPtr(T *p) / AnyPtr(TT *p)
impl<T> From<*mut T> for AnyPtr<T> {
    fn from(p: *mut T) -> Self {
        AnyPtr::from_raw(p)
    }
}

// AnyPtr.hpp:55  AnyPtr(std::unique_ptr<TT> p)
impl<T> From<Box<T>> for AnyPtr<T> {
    fn from(p: Box<T>) -> Self {
        AnyPtr::from_unique(p)
    }
}

// AnyPtr.hpp:56  AnyPtr(std::shared_ptr<TT> p)
impl<T> From<Arc<T>> for AnyPtr<T> {
    fn from(p: Arc<T>) -> Self {
        AnyPtr::from_shared(p)
    }
}

// AnyPtr.hpp:142  } // namespace Slic3r

#[cfg(test)]
mod tests {
    use super::*;

    // AnyPtr.hpp:50  AnyPtr(T *p)
    #[test]
    fn test_from_raw() {
        let mut value = 42;
        let ptr = AnyPtr::from_raw(&mut value as *mut i32);
        // operator bool() -> true for non-null raw pointer
        assert!(ptr.is_valid());
        assert_eq!(*ptr.as_ref(), 42);
        // raw pointer is not owned
        assert!(!ptr.is_owned());
    }

    // AnyPtr.hpp:55  AnyPtr(std::unique_ptr<TT> p)
    #[test]
    fn test_from_unique() {
        let ptr = AnyPtr::from_unique(Box::new(42));
        assert!(ptr.is_owned());
        assert!(ptr.is_valid());
        assert_eq!(*ptr.as_ref(), 42);
    }

    // AnyPtr.hpp:56  AnyPtr(std::shared_ptr<TT> p)
    #[test]
    fn test_from_shared() {
        let ptr = AnyPtr::from_shared(Arc::new(42));
        assert!(ptr.is_owned());
        assert!(ptr.is_valid());
        assert_eq!(*ptr.as_ref(), 42);
    }

    // AnyPtr.hpp:48 / AnyPtr.hpp:52  default / nullptr_t
    #[test]
    fn test_default_is_null_raw() {
        let ptr: AnyPtr<i32> = AnyPtr::default();
        // operator bool() -> false for null raw pointer
        assert!(!ptr.is_valid());
        assert!(!ptr.is_owned());
        assert!(ptr.get().is_null());
    }

    // AnyPtr.hpp:121-130  get_shared_cpy
    #[test]
    fn test_get_shared_cpy() {
        let arc = Arc::new(42);
        let ptr = AnyPtr::from_shared(Arc::clone(&arc));
        let copy = ptr.get_shared_cpy();
        assert!(copy.is_some());
        assert_eq!(*copy.unwrap(), 42);
        // original + ptr + copy
        assert_eq!(Arc::strong_count(&arc), 3);

        // Empty shared pointer returned otherwise.
        let raw = AnyPtr::from_raw(std::ptr::null_mut::<i32>());
        assert!(raw.get_shared_cpy().is_none());
    }

    // AnyPtr.hpp:132-136  convert_unique_to_shared
    #[test]
    fn test_convert_unique_to_shared() {
        let mut ptr = AnyPtr::from_unique(Box::new(42));
        assert!(matches!(ptr, AnyPtr::UPtr(_)));

        ptr.convert_unique_to_shared();
        assert!(matches!(ptr, AnyPtr::ShPtr(_)));
        assert!(ptr.is_owned());
        assert_eq!(*ptr.as_ref(), 42);

        // No-op when not unique.
        let mut raw = AnyPtr::from_raw(std::ptr::null_mut::<i32>());
        raw.convert_unique_to_shared();
        assert!(matches!(raw, AnyPtr::RawPtr(_)));
    }

    // AnyPtr.hpp:138-139  is_owned
    #[test]
    fn test_is_owned() {
        let raw = AnyPtr::from_raw(std::ptr::null_mut::<i32>());
        assert!(!raw.is_owned());

        let uptr = AnyPtr::from_unique(Box::new(42));
        assert!(uptr.is_owned());

        let shptr = AnyPtr::from_shared(Arc::new(42));
        assert!(shptr.is_owned());
    }

    // AnyPtr.hpp:102  T &operator*()
    #[test]
    fn test_as_mut() {
        let mut ptr = AnyPtr::from_unique(Box::new(42));
        *ptr.as_mut() = 100;
        assert_eq!(*ptr.as_ref(), 100);
    }
}
