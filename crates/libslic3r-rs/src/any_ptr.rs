//! Generic smart pointer holder that can store raw, Box, Arc, or Rc pointers
//!
//! C++ Reference:
//! - AnyPtr.hpp
//!
//! This module provides a type-erased pointer holder similar to C++'s boost::variant
//! of raw pointer, unique_ptr, and shared_ptr. It can hold ownership or just reference
//! depending on what type of pointer is stored.
//!
//! ## Key Differences from C++
//!
//! - Rust uses `Box<T>` instead of `std::unique_ptr<T>`
//! - Rust uses `Arc<T>` instead of `std::shared_ptr<T>` for thread-safe sharing
//! - Rust uses `Rc<T>` for single-threaded reference counting (not in C++)
//! - No need for custom deleters (Rust's Drop trait handles cleanup)
//! - Move semantics are explicit in Rust (no move constructors needed)

use std::rc::Rc;
use std::sync::Arc;

/// Generic smart pointer holder that can own or reference data
/// AnyPtr.hpp:26-157
///
/// This type can hold:
/// - Raw pointer (*const T) - no ownership, borrowed reference
/// - Box<T> - unique ownership (like std::unique_ptr)
/// - Arc<T> - shared ownership, thread-safe (like std::shared_ptr)
/// - Rc<T> - shared ownership, single-threaded
///
/// C++: template<class T> class AnyPtr
#[derive(Debug)]
pub enum AnyPtr<T> {
    /// Raw pointer (no ownership)
    /// AnyPtr.hpp:29
    /// C++: boost::variant alternative 0: T*
    Raw(*const T),

    /// Owned unique pointer
    /// AnyPtr.hpp:29
    /// C++: boost::variant alternative 1: std::unique_ptr<T>
    Unique(Box<T>),

    /// Shared pointer (thread-safe)
    /// AnyPtr.hpp:29
    /// C++: boost::variant alternative 2: std::shared_ptr<T>
    Shared(Arc<T>),

    /// Reference-counted pointer (single-threaded)
    /// (Not in C++ version, but useful in Rust)
    RefCounted(Rc<T>),
}

impl<T> AnyPtr<T> {
    /// Create from raw pointer
    /// AnyPtr.hpp:47
    /// C++: AnyPtr(T *p) noexcept : ptr{p} {}
    pub fn from_raw(ptr: *const T) -> Self {
        AnyPtr::Raw(ptr)
    }

    /// Create from Box (unique ownership)
    /// AnyPtr.hpp:51
    /// C++: AnyPtr(std::unique_ptr<TT> p) noexcept : ptr{std::unique_ptr<T>(std::move(p))} {}
    pub fn from_box(ptr: Box<T>) -> Self {
        AnyPtr::Unique(ptr)
    }

    /// Create from Arc (shared ownership, thread-safe)
    /// AnyPtr.hpp:52
    /// C++: AnyPtr(std::shared_ptr<TT> p) noexcept : ptr{std::shared_ptr<T>(std::move(p))} {}
    pub fn from_arc(ptr: Arc<T>) -> Self {
        AnyPtr::Shared(ptr)
    }

    /// Create from Rc (shared ownership, single-threaded)
    /// (Not in C++ version)
    pub fn from_rc(ptr: Rc<T>) -> Self {
        AnyPtr::RefCounted(ptr)
    }

    /// Get raw pointer to the data
    /// AnyPtr.hpp:31-40
    /// C++: template<class Self> static T *get_ptr(Self &&s)
    pub fn get(&self) -> *const T {
        match self {
            AnyPtr::Raw(p) => *p,
            AnyPtr::Unique(b) => b.as_ref() as *const T,
            AnyPtr::Shared(a) => Arc::as_ptr(a),
            AnyPtr::RefCounted(r) => Rc::as_ptr(r),
        }
    }

    /// Get mutable raw pointer to the data (if uniquely owned)
    /// AnyPtr.hpp:31-40
    pub fn get_mut(&mut self) -> Option<*mut T> {
        match self {
            AnyPtr::Unique(b) => Some(b.as_mut() as *mut T),
            _ => None,
        }
    }

    /// Check if pointer is null
    /// AnyPtr.hpp:119-128
    /// C++: operator bool() const noexcept
    pub fn is_null(&self) -> bool {
        match self {
            AnyPtr::Raw(p) => p.is_null(),
            AnyPtr::Unique(b) => b.as_ref() as *const T == std::ptr::null(),
            AnyPtr::Shared(a) => Arc::as_ptr(a) == std::ptr::null(),
            AnyPtr::RefCounted(r) => Rc::as_ptr(r) == std::ptr::null(),
        }
    }

    /// Check if pointer is valid (not null)
    /// AnyPtr.hpp:119-128
    /// C++: operator bool() const noexcept
    pub fn is_some(&self) -> bool {
        !self.is_null()
    }

    /// Get a shared copy if the underlying pointer is Arc
    /// Returns None if not an Arc
    /// AnyPtr.hpp:131-137
    /// C++: std::shared_ptr<T> get_shared_cpy() const noexcept
    pub fn get_shared_copy(&self) -> Option<Arc<T>> {
        match self {
            AnyPtr::Shared(a) => Some(Arc::clone(a)),
            _ => None,
        }
    }

    /// Get a reference-counted copy if the underlying pointer is Rc
    /// Returns None if not an Rc
    pub fn get_rc_copy(&self) -> Option<Rc<T>> {
        match self {
            AnyPtr::RefCounted(r) => Some(Rc::clone(r)),
            _ => None,
        }
    }

    /// Convert unique ownership (Box) to shared ownership (Arc)
    /// AnyPtr.hpp:140-142
    /// C++: void convert_unique_to_shared() noexcept
    pub fn convert_unique_to_shared(&mut self) {
        if let AnyPtr::Unique(b) = std::mem::replace(self, AnyPtr::Raw(std::ptr::null())) {
            *self = AnyPtr::Shared(Arc::from(b));
        }
    }

    /// Convert unique ownership (Box) to reference-counted (Rc)
    pub fn convert_unique_to_rc(&mut self) {
        if let AnyPtr::Unique(b) = std::mem::replace(self, AnyPtr::Raw(std::ptr::null())) {
            *self = AnyPtr::RefCounted(Rc::from(b));
        }
    }

    /// Check if the data is owned by this AnyPtr instance
    /// AnyPtr.hpp:145
    /// C++: bool is_owned() const noexcept { return ptr.which() == UPtr || ptr.which() == ShPtr; }
    pub fn is_owned(&self) -> bool {
        matches!(
            self,
            AnyPtr::Unique(_) | AnyPtr::Shared(_) | AnyPtr::RefCounted(_)
        )
    }

    /// Dereference to get a reference (panics if null)
    /// AnyPtr.hpp:104-105
    /// C++: const T &operator*() const noexcept { return *get_ptr(*this); }
    /// C++: T &operator*() noexcept { return *get_ptr(*this); }
    pub fn as_ref(&self) -> &T {
        unsafe {
            self.get()
                .as_ref()
                .expect("Attempted to dereference null AnyPtr")
        }
    }

    /// Try to get a reference (returns None if null)
    pub fn try_as_ref(&self) -> Option<&T> {
        if self.is_null() {
            None
        } else {
            unsafe { self.get().as_ref() }
        }
    }

    /// Try to get a mutable reference (only for Unique ownership)
    pub fn try_as_mut(&mut self) -> Option<&mut T> {
        match self {
            AnyPtr::Unique(b) => Some(b.as_mut()),
            _ => None,
        }
    }
}

impl<T> Default for AnyPtr<T> {
    /// Create a null AnyPtr
    /// AnyPtr.hpp:45
    /// C++: AnyPtr() noexcept = default;
    fn default() -> Self {
        AnyPtr::Raw(std::ptr::null())
    }
}

impl<T> From<*const T> for AnyPtr<T> {
    /// Convert raw pointer to AnyPtr
    /// AnyPtr.hpp:47
    fn from(ptr: *const T) -> Self {
        AnyPtr::from_raw(ptr)
    }
}

impl<T> From<Box<T>> for AnyPtr<T> {
    /// Convert Box to AnyPtr
    /// AnyPtr.hpp:51
    fn from(ptr: Box<T>) -> Self {
        AnyPtr::from_box(ptr)
    }
}

impl<T> From<Arc<T>> for AnyPtr<T> {
    /// Convert Arc to AnyPtr
    /// AnyPtr.hpp:52
    fn from(ptr: Arc<T>) -> Self {
        AnyPtr::from_arc(ptr)
    }
}

impl<T> From<Rc<T>> for AnyPtr<T> {
    /// Convert Rc to AnyPtr
    fn from(ptr: Rc<T>) -> Self {
        AnyPtr::from_rc(ptr)
    }
}

impl<T> Clone for AnyPtr<T> {
    /// Clone the AnyPtr
    /// Only works for Shared (Arc) and RefCounted (Rc) variants
    /// Panics for Raw and Unique variants
    fn clone(&self) -> Self {
        match self {
            AnyPtr::Raw(p) => AnyPtr::Raw(*p),
            AnyPtr::Unique(_) => panic!("Cannot clone AnyPtr with unique ownership"),
            AnyPtr::Shared(a) => AnyPtr::Shared(Arc::clone(a)),
            AnyPtr::RefCounted(r) => AnyPtr::RefCounted(Rc::clone(r)),
        }
    }
}

// Note: We don't implement Deref/DerefMut because AnyPtr can be null
// Users should call as_ref() or try_as_ref() explicitly

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_from_raw() {
        let value = 42;
        let ptr = AnyPtr::from_raw(&value as *const i32);
        assert!(!ptr.is_null());
        assert!(ptr.is_some());
        assert_eq!(*ptr.as_ref(), 42);
    }

    #[test]
    fn test_from_box() {
        let value = Box::new(42);
        let ptr = AnyPtr::from_box(value);
        assert!(ptr.is_owned());
        assert_eq!(*ptr.as_ref(), 42);
    }

    #[test]
    fn test_from_arc() {
        let value = Arc::new(42);
        let ptr = AnyPtr::from_arc(value);
        assert!(ptr.is_owned());
        assert_eq!(*ptr.as_ref(), 42);
    }

    #[test]
    fn test_from_rc() {
        let value = Rc::new(42);
        let ptr = AnyPtr::from_rc(value);
        assert!(ptr.is_owned());
        assert_eq!(*ptr.as_ref(), 42);
    }

    #[test]
    fn test_default() {
        let ptr: AnyPtr<i32> = AnyPtr::default();
        assert!(ptr.is_null());
        assert!(!ptr.is_some());
    }

    #[test]
    fn test_get_shared_copy() {
        let arc = Arc::new(42);
        let ptr = AnyPtr::from_arc(Arc::clone(&arc));
        let copy = ptr.get_shared_copy();
        assert!(copy.is_some());
        assert_eq!(*copy.unwrap(), 42);
        assert_eq!(Arc::strong_count(&arc), 3); // original + ptr + copy
    }

    #[test]
    fn test_get_rc_copy() {
        let rc = Rc::new(42);
        let ptr = AnyPtr::from_rc(Rc::clone(&rc));
        let copy = ptr.get_rc_copy();
        assert!(copy.is_some());
        assert_eq!(*copy.unwrap(), 42);
        assert_eq!(Rc::strong_count(&rc), 3); // original + ptr + copy
    }

    #[test]
    fn test_convert_unique_to_shared() {
        let mut ptr = AnyPtr::from_box(Box::new(42));
        assert!(matches!(ptr, AnyPtr::Unique(_)));

        ptr.convert_unique_to_shared();
        assert!(matches!(ptr, AnyPtr::Shared(_)));
        assert_eq!(*ptr.as_ref(), 42);
    }

    #[test]
    fn test_convert_unique_to_rc() {
        let mut ptr = AnyPtr::from_box(Box::new(42));
        assert!(matches!(ptr, AnyPtr::Unique(_)));

        ptr.convert_unique_to_rc();
        assert!(matches!(ptr, AnyPtr::RefCounted(_)));
        assert_eq!(*ptr.as_ref(), 42);
    }

    #[test]
    fn test_is_owned() {
        let raw_ptr = AnyPtr::from_raw(&42 as *const i32);
        assert!(!raw_ptr.is_owned());

        let box_ptr = AnyPtr::from_box(Box::new(42));
        assert!(box_ptr.is_owned());

        let arc_ptr = AnyPtr::from_arc(Arc::new(42));
        assert!(arc_ptr.is_owned());

        let rc_ptr = AnyPtr::from_rc(Rc::new(42));
        assert!(rc_ptr.is_owned());
    }

    #[test]
    fn test_try_as_ref() {
        let ptr = AnyPtr::from_box(Box::new(42));
        assert_eq!(ptr.try_as_ref(), Some(&42));

        let null_ptr: AnyPtr<i32> = AnyPtr::default();
        assert_eq!(null_ptr.try_as_ref(), None);
    }

    #[test]
    fn test_try_as_mut() {
        let mut ptr = AnyPtr::from_box(Box::new(42));
        if let Some(val) = ptr.try_as_mut() {
            *val = 100;
        }
        assert_eq!(*ptr.as_ref(), 100);

        let mut arc_ptr = AnyPtr::from_arc(Arc::new(42));
        assert!(arc_ptr.try_as_mut().is_none()); // Arc is not mutable
    }

    #[test]
    fn test_clone_shared() {
        let arc = Arc::new(42);
        let ptr1 = AnyPtr::from_arc(Arc::clone(&arc));
        let ptr2 = ptr1.clone();
        assert_eq!(*ptr1.as_ref(), 42);
        assert_eq!(*ptr2.as_ref(), 42);
        assert_eq!(Arc::strong_count(&arc), 3); // original + ptr1 + ptr2
    }

    #[test]
    fn test_clone_rc() {
        let rc = Rc::new(42);
        let ptr1 = AnyPtr::from_rc(Rc::clone(&rc));
        let ptr2 = ptr1.clone();
        assert_eq!(*ptr1.as_ref(), 42);
        assert_eq!(*ptr2.as_ref(), 42);
        assert_eq!(Rc::strong_count(&rc), 3); // original + ptr1 + ptr2
    }

    #[test]
    #[should_panic(expected = "Cannot clone AnyPtr with unique ownership")]
    fn test_clone_unique_panics() {
        let ptr = AnyPtr::from_box(Box::new(42));
        let _clone = ptr.clone(); // Should panic
    }
}
