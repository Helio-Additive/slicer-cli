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
/// clonable_ptr.hpp:26-166
///
/// Similar to Box<T>, but implements Clone by calling T::clone().
/// This allows deep copying of the pointed-to value when the pointer is copied.
///
/// C++: template<class T> class clonable_ptr
pub struct ClonablePtr<T: Clone> {
    /// Native pointer to the managed object
    /// clonable_ptr.hpp:163
    /// C++: T* px;
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
    /// clonable_ptr.hpp:77-80
    /// C++: inline void reset() noexcept { destroy(); }
    pub fn reset(&mut self) {
        self.ptr = None;
    }

    /// Reset with a new value, dropping the old one
    /// clonable_ptr.hpp:82-87
    /// C++: void reset(T* p) noexcept
    pub fn reset_with(&mut self, value: T) {
        self.ptr = Some(Box::new(value));
    }

    /// Swap contents with another ClonablePtr
    /// clonable_ptr.hpp:89-94
    /// C++: void swap(clonable_ptr& rhs) noexcept
    pub fn swap(&mut self, other: &mut Self) {
        std::mem::swap(&mut self.ptr, &mut other.ptr);
    }

    /// Release ownership and return the inner Box
    /// clonable_ptr.hpp:96-99
    /// C++: inline void release() noexcept { px = nullptr; }
    pub fn take(&mut self) -> Option<Box<T>> {
        self.ptr.take()
    }

    /// Get a raw pointer (without transferring ownership)
    /// clonable_ptr.hpp:129-133
    /// C++: inline T* get() const noexcept { return px; }
    pub fn get(&self) -> Option<&T> {
        self.ptr.as_deref()
    }

    /// Get a mutable raw pointer
    pub fn get_mut(&mut self) -> Option<&mut T> {
        self.ptr.as_deref_mut()
    }

    /// Check if the pointer is non-null
    /// clonable_ptr.hpp:102-105
    /// C++: inline operator bool() const noexcept { return (nullptr != px); }
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
    /// clonable_ptr.hpp:42-45
    /// C++: clonable_ptr(const clonable_ptr& rhs) : px(rhs ? rhs.px->clone() : nullptr) {}
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
    /// clonable_ptr.hpp:108-112
    /// C++: inline T& operator*() const noexcept { assert(nullptr != px); return *px; }
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

impl<T: Clone + PartialEq> PartialEq for ClonablePtr<T> {
    /// Compare two ClonablePtrs for equality
    /// clonable_ptr.hpp:169-172
    /// C++: template<class T, class U> inline bool operator==(const clonable_ptr<T>& l, const clonable_ptr<U>& r) noexcept
    fn eq(&self, other: &Self) -> bool {
        self.ptr == other.ptr
    }
}

impl<T: Clone + Eq> Eq for ClonablePtr<T> {}

impl<T: Clone + PartialOrd> PartialOrd for ClonablePtr<T> {
    /// Compare two ClonablePtrs for ordering
    /// clonable_ptr.hpp:177-180
    /// C++: template<class T, class U> inline bool operator<(const clonable_ptr<T>& l, const clonable_ptr<U>& r) noexcept
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.ptr.partial_cmp(&other.ptr)
    }
}

impl<T: Clone + Ord> Ord for ClonablePtr<T> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.ptr.cmp(&other.ptr)
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

        // They should be independent copies
        assert_eq!(ptr1, ptr2);
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
        let ptr1 = ClonablePtr::from_value(42);
        let ptr2 = ClonablePtr::from_value(42);
        let ptr3 = ClonablePtr::from_value(100);

        assert_eq!(ptr1, ptr2);
        assert_ne!(ptr1, ptr3);
    }

    #[test]
    fn test_ordering() {
        let ptr1 = ClonablePtr::from_value(10);
        let ptr2 = ClonablePtr::from_value(20);
        let ptr3 = ClonablePtr::from_value(20);

        assert!(ptr1 < ptr2);
        assert!(ptr2 > ptr1);
        assert!(ptr2 == ptr3);
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
