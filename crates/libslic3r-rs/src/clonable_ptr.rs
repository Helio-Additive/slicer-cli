//! Smart pointer with clone-on-copy semantics
//!
//! C++ Reference:
//! - clonable_ptr.hpp
//!
//! This module provides a smart pointer similar to Box, but with the key difference
//! that copying the pointer calls clone() on the underlying value, rather than
//! moving or sharing ownership. This is useful for value types that need deep copying.

use std::fmt;
use std::ops::{Deref, DerefMut};

/// A smart pointer that clones on copy
/// clonable_ptr.hpp:26-140
///
/// Similar to Box<T>, but implements Clone by calling T::clone().
/// This allows deep copying of the pointed-to value when the pointer is copied.
///
/// C++: template<class T> class clonable_ptr
///
/// The C++ class also exposes `typedef T element_type;` (clonable_ptr.hpp:31); in
/// Rust the element type is the generic parameter `T` itself, so no alias is needed.
pub struct ClonablePtr<T: Clone> {
    /// Native pointer to the managed object
    /// clonable_ptr.hpp:139
    /// C++: T* px; //!< Native pointer
    ptr: Option<Box<T>>,
}

impl<T: Clone> ClonablePtr<T> {
    /// Create a new empty ClonablePtr
    /// clonable_ptr.hpp:32-35
    /// C++: clonable_ptr() noexcept : px(nullptr) {}
    pub fn new() -> Self {
        ClonablePtr { ptr: None }
    }

    /// Create a ClonablePtr from a value
    /// clonable_ptr.hpp:37-40
    /// C++: explicit clonable_ptr(T* p) noexcept : px(p) {}
    pub fn from_value(value: T) -> Self {
        ClonablePtr {
            ptr: Some(Box::new(value)),
        }
    }

    /// Create a ClonablePtr from a boxed value
    pub fn from_box(boxed: Box<T>) -> Self {
        ClonablePtr { ptr: Some(boxed) }
    }

    /// Reset to empty, dropping the current value
    /// clonable_ptr.hpp:74-78
    /// C++: inline void reset() noexcept { destroy(); }
    pub fn reset(&mut self) {
        self.ptr = None;
    }

    /// Reset with a new value, dropping the old one
    /// clonable_ptr.hpp:79-85
    /// C++: void reset(T* p) noexcept
    /// {
    ///     assert((nullptr == p) || (px != p)); // auto-reset not allowed
    ///     destroy();
    ///     px = p;
    /// }
    ///
    /// Rust takes the value by move (not a raw pointer), so the C++
    /// `(nullptr == p) || (px != p)` auto-reset assertion cannot be
    /// violated: a moved-in value can never alias the currently held box.
    pub fn reset_with(&mut self, value: T) {
        self.ptr = Some(Box::new(value));
    }

    /// Reset with a new boxed value, dropping the old one
    /// clonable_ptr.hpp:79-85
    /// C++: void reset(T* p) noexcept
    /// {
    ///     assert((nullptr == p) || (px != p)); // auto-reset not allowed
    ///     destroy();
    ///     px = p;
    /// }
    pub fn reset_box(&mut self, boxed: Box<T>) {
        // assert((nullptr == p) || (px != p)); // auto-reset not allowed
        // In Rust an owned Box cannot alias the currently held box, so the
        // auto-reset assertion is always satisfied.
        debug_assert!(
            self.ptr.as_ref().map_or(true, |cur| !std::ptr::eq(
                cur.as_ref() as *const T,
                boxed.as_ref() as *const T
            )),
            "auto-reset not allowed"
        );
        self.ptr = Some(boxed);
    }

    /// Swap contents with another ClonablePtr
    /// clonable_ptr.hpp:87-93
    /// C++: void swap(clonable_ptr& rhs) noexcept
    /// {
    ///     T *tmp = px;
    ///     px = rhs.px;
    ///     rhs.px = tmp;
    /// }
    pub fn swap(&mut self, other: &mut Self) {
        std::mem::swap(&mut self.ptr, &mut other.ptr);
    }

    /// Release the ownership of the pointer without destroying the object.
    /// clonable_ptr.hpp:95-99
    /// C++: inline void release() noexcept { px = nullptr; }
    ///
    /// The C++ `release()` sets `px = nullptr` and returns void, leaking the
    /// object unless the caller already retained the raw pointer elsewhere.
    /// Rust has no raw owning pointer to retain, so to release ownership
    /// without dropping we hand the owned `Box<T>` back to the caller.
    pub fn take(&mut self) -> Option<Box<T>> {
        self.ptr.take()
    }

    /// Get a raw pointer (without transferring ownership)
    /// clonable_ptr.hpp:118-122
    /// C++: inline T* get() const noexcept { return px; }
    pub fn get(&self) -> Option<&T> {
        self.ptr.as_deref()
    }

    /// Get a mutable raw pointer
    pub fn get_mut(&mut self) -> Option<&mut T> {
        self.ptr.as_deref_mut()
    }

    /// Check if the pointer is non-null
    /// clonable_ptr.hpp:101-105
    /// C++: inline operator bool() const noexcept { return (nullptr != px); // TODO nullptrptr }
    pub fn is_some(&self) -> bool {
        self.ptr.is_some()
    }

    /// Check if the pointer is null
    pub fn is_none(&self) -> bool {
        self.ptr.is_none()
    }

    /// Unwrap the value, panicking if None
    pub fn unwrap(self) -> T {
        *self.ptr.expect("Called unwrap on None ClonablePtr")
    }

    /// Unwrap or return a default value
    pub fn unwrap_or(self, default: T) -> T {
        match self.ptr {
            Some(boxed) => *boxed,
            None => default,
        }
    }

    /// Convert to Option<T>
    pub fn into_inner(self) -> Option<T> {
        self.ptr.map(|b| *b)
    }
}

impl<T: Clone> Default for ClonablePtr<T> {
    /// Create an empty ClonablePtr
    /// clonable_ptr.hpp:32-35
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Clone> Clone for ClonablePtr<T> {
    /// Clone by calling clone() on the underlying value
    /// clonable_ptr.hpp:43-47
    /// C++: clonable_ptr(const clonable_ptr& rhs) : px(rhs ? rhs.px->clone() : nullptr) {}
    ///
    /// C++ also defines copy-assignment (clonable_ptr.hpp:54-60):
    ///     clonable_ptr& operator=(const clonable_ptr& rhs)
    ///     { delete px; px = rhs ? rhs.px->clone() : nullptr; return *this; }
    /// In Rust the derived `Clone`-based assignment (`a = b.clone()`) provides
    /// the equivalent: the old value is dropped and a fresh deep copy installed.
    /// The move constructor / move assignment (clonable_ptr.hpp:48-53, 61-68)
    /// correspond to Rust's built-in move semantics.
    fn clone(&self) -> Self {
        ClonablePtr {
            ptr: self.ptr.as_ref().map(|b| Box::new((**b).clone())),
        }
    }
}

impl<T: Clone> From<T> for ClonablePtr<T> {
    /// Create from a value
    fn from(value: T) -> Self {
        Self::from_value(value)
    }
}

impl<T: Clone> From<Box<T>> for ClonablePtr<T> {
    /// Create from a Box
    fn from(boxed: Box<T>) -> Self {
        Self::from_box(boxed)
    }
}

impl<T: Clone> From<Option<T>> for ClonablePtr<T> {
    /// Create from an Option
    fn from(opt: Option<T>) -> Self {
        ClonablePtr {
            ptr: opt.map(Box::new),
        }
    }
}

impl<T: Clone> Deref for ClonablePtr<T> {
    type Target = T;

    /// Dereference to get a reference to the value
    /// clonable_ptr.hpp:107-117
    /// C++: inline T& operator*() const noexcept { assert(nullptr != px); return *px; }
    /// C++: inline T* operator->() const noexcept { assert(nullptr != px); return px; }
    /// Both `operator*` and `operator->` assert non-null then return the
    /// pointee; in Rust both collapse onto `Deref::deref` (with method/field
    /// access through `.` standing in for `operator->`).
    fn deref(&self) -> &T {
        self.ptr
            .as_deref()
            .expect("Attempted to dereference null ClonablePtr")
    }
}

impl<T: Clone> DerefMut for ClonablePtr<T> {
    /// Dereference to get a mutable reference to the value
    fn deref_mut(&mut self) -> &mut T {
        self.ptr
            .as_deref_mut()
            .expect("Attempted to dereference null ClonablePtr")
    }
}

impl<T: Clone + fmt::Debug> fmt::Debug for ClonablePtr<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.ptr {
            Some(val) => write!(f, "ClonablePtr({:?})", val),
            None => write!(f, "ClonablePtr(None)"),
        }
    }
}

impl<T: Clone + fmt::Display> fmt::Display for ClonablePtr<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self.ptr {
            Some(val) => write!(f, "{}", val),
            None => write!(f, "None"),
        }
    }
}

impl<T: Clone> ClonablePtr<T> {
    /// Raw pointer address of the managed object, or null when empty.
    ///
    /// This mirrors the C++ `get()` (clonable_ptr.hpp:118-122) which returns
    /// the native `T* px`. The comparison operators below compare these raw
    /// addresses, exactly as the C++ free operators compare `l.get()` vs
    /// `r.get()` (pointer identity, NOT pointed-to value equality).
    fn raw_ptr(&self) -> *const T {
        match &self.ptr {
            Some(b) => b.as_ref() as *const T,
            None => std::ptr::null(),
        }
    }
}

impl<T: Clone> PartialEq for ClonablePtr<T> {
    /// Compare two ClonablePtrs for equality
    /// clonable_ptr.hpp:142-150
    /// C++: template<class T, class U> inline bool operator==(const clonable_ptr<T>& l, const clonable_ptr<U>& r) noexcept { return (l.get() == r.get()); }
    /// C++: template<class T, class U> inline bool operator!=(const clonable_ptr<T>& l, const clonable_ptr<U>& r) noexcept { return (l.get() != r.get()); }
    ///
    /// The C++ comparison operators compare the raw pointer addresses
    /// (`l.get() == r.get()`), i.e. pointer identity, NOT the pointed-to
    /// values. We faithfully mirror that here by comparing the heap address of
    /// the managed object (`raw_ptr()`); `!=` is the negation, matching
    /// operator!= (clonable_ptr.hpp:147-150).
    fn eq(&self, other: &Self) -> bool {
        self.raw_ptr() == other.raw_ptr()
    }
}

impl<T: Clone> Eq for ClonablePtr<T> {}

impl<T: Clone> PartialOrd for ClonablePtr<T> {
    /// Compare two ClonablePtrs for ordering
    /// clonable_ptr.hpp:151-166
    /// C++: operator<=, operator<, operator>=, operator> all compare l.get() vs r.get()
    ///
    /// As with `eq`, the C++ ordering operators compare raw pointer addresses,
    /// so we order by the managed object's heap address (`raw_ptr()`).
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl<T: Clone> Ord for ClonablePtr<T> {
    /// clonable_ptr.hpp:151-166: ordering by raw pointer address.
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.raw_ptr().cmp(&other.raw_ptr())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let ptr: ClonablePtr<i32> = ClonablePtr::new();
        assert!(ptr.is_none());
        assert!(!ptr.is_some());
    }

    #[test]
    fn test_from_value() {
        let ptr = ClonablePtr::from_value(42);
        assert!(ptr.is_some());
        assert_eq!(*ptr, 42);
    }

    #[test]
    fn test_clone() {
        let ptr1 = ClonablePtr::from_value(42);
        let ptr2 = ptr1.clone();

        assert_eq!(*ptr1, 42);
        assert_eq!(*ptr2, 42);

        // They are independent deep copies, so (matching C++ pointer-identity
        // operator==, clonable_ptr.hpp:142-146) they compare UNEQUAL: the clone
        // owns a distinct heap object with a different address.
        assert_ne!(ptr1, ptr2);
    }

    #[test]
    fn test_reset() {
        let mut ptr = ClonablePtr::from_value(42);
        assert!(ptr.is_some());

        ptr.reset();
        assert!(ptr.is_none());
    }

    #[test]
    fn test_reset_with() {
        let mut ptr = ClonablePtr::from_value(42);
        ptr.reset_with(100);
        assert_eq!(*ptr, 100);
    }

    #[test]
    fn test_reset_box() {
        let mut ptr = ClonablePtr::from_value(42);
        ptr.reset_box(Box::new(100));
        assert_eq!(*ptr, 100);

        let mut empty: ClonablePtr<i32> = ClonablePtr::new();
        empty.reset_box(Box::new(7));
        assert_eq!(*empty, 7);
    }

    #[test]
    fn test_swap() {
        let mut ptr1 = ClonablePtr::from_value(42);
        let mut ptr2 = ClonablePtr::from_value(100);

        ptr1.swap(&mut ptr2);

        assert_eq!(*ptr1, 100);
        assert_eq!(*ptr2, 42);
    }

    #[test]
    fn test_take() {
        let mut ptr = ClonablePtr::from_value(42);
        let taken = ptr.take();

        assert!(ptr.is_none());
        assert!(taken.is_some());
        assert_eq!(*taken.unwrap(), 42);
    }

    #[test]
    fn test_get() {
        let ptr = ClonablePtr::from_value(42);
        assert_eq!(ptr.get(), Some(&42));

        let empty: ClonablePtr<i32> = ClonablePtr::new();
        assert_eq!(empty.get(), None);
    }

    #[test]
    fn test_get_mut() {
        let mut ptr = ClonablePtr::from_value(42);
        if let Some(val) = ptr.get_mut() {
            *val = 100;
        }
        assert_eq!(*ptr, 100);
    }

    #[test]
    fn test_deref() {
        let ptr = ClonablePtr::from_value(42);
        let val: &i32 = &*ptr;
        assert_eq!(*val, 42);
    }

    #[test]
    fn test_deref_mut() {
        let mut ptr = ClonablePtr::from_value(42);
        *ptr = 100;
        assert_eq!(*ptr, 100);
    }

    #[test]
    #[should_panic(expected = "Attempted to dereference null ClonablePtr")]
    fn test_deref_panic() {
        let ptr: ClonablePtr<i32> = ClonablePtr::new();
        let _val = *ptr; // Should panic
    }

    #[test]
    fn test_unwrap() {
        let ptr = ClonablePtr::from_value(42);
        assert_eq!(ptr.unwrap(), 42);
    }

    #[test]
    #[should_panic(expected = "Called unwrap on None ClonablePtr")]
    fn test_unwrap_panic() {
        let ptr: ClonablePtr<i32> = ClonablePtr::new();
        ptr.unwrap(); // Should panic
    }

    #[test]
    fn test_unwrap_or() {
        let ptr = ClonablePtr::from_value(42);
        assert_eq!(ptr.unwrap_or(100), 42);

        let empty: ClonablePtr<i32> = ClonablePtr::new();
        assert_eq!(empty.unwrap_or(100), 100);
    }

    #[test]
    fn test_into_inner() {
        let ptr = ClonablePtr::from_value(42);
        assert_eq!(ptr.into_inner(), Some(42));

        let empty: ClonablePtr<i32> = ClonablePtr::new();
        assert_eq!(empty.into_inner(), None);
    }

    #[test]
    fn test_equality() {
        // C++ operator== compares raw pointer addresses (clonable_ptr.hpp:142-146),
        // not pointed-to values. Two independently-allocated pointers are never
        // equal even with the same payload; a pointer is equal only to itself.
        let ptr1 = ClonablePtr::from_value(42);
        let ptr2 = ClonablePtr::from_value(42);
        let ptr3 = ClonablePtr::from_value(100);

        assert_ne!(ptr1, ptr2);
        assert_ne!(ptr1, ptr3);

        #[allow(clippy::eq_op)]
        {
            assert_eq!(ptr1, ptr1);
        }

        // Two empty pointers both carry the null address, so they are equal.
        let e1: ClonablePtr<i32> = ClonablePtr::new();
        let e2: ClonablePtr<i32> = ClonablePtr::new();
        assert_eq!(e1, e2);
    }

    #[test]
    fn test_ordering() {
        // C++ ordering operators (clonable_ptr.hpp:151-166) compare raw pointer
        // addresses, not values. A null (empty) pointer sorts before any
        // allocated one, and a pointer is equal only to itself.
        let empty: ClonablePtr<i32> = ClonablePtr::new();
        let ptr = ClonablePtr::from_value(20);

        assert!(empty < ptr);
        assert!(ptr > empty);

        #[allow(clippy::eq_op)]
        {
            assert!(ptr == ptr);
        }
    }

    #[test]
    fn test_from_option() {
        let ptr1 = ClonablePtr::from(Some(42));
        assert!(ptr1.is_some());
        assert_eq!(*ptr1, 42);

        let ptr2: ClonablePtr<i32> = ClonablePtr::from(None);
        assert!(ptr2.is_none());
    }

    #[test]
    fn test_debug_display() {
        let ptr = ClonablePtr::from_value(42);
        let debug_str = format!("{:?}", ptr);
        assert!(debug_str.contains("42"));

        let display_str = format!("{}", ptr);
        assert!(display_str.contains("42"));
    }

    #[test]
    fn test_clone_deep_copy() {
        // Test with a more complex type to ensure deep cloning
        let original = ClonablePtr::from_value(vec![1, 2, 3]);
        let mut cloned = original.clone();

        // Modify the clone
        cloned.push(4);

        // Original should be unchanged
        assert_eq!(*original, vec![1, 2, 3]);
        assert_eq!(*cloned, vec![1, 2, 3, 4]);
    }
}
