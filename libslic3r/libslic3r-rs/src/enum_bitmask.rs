//! Type-safe bitmask operations on enums
//!
//! C++ Reference:
//! - enum_bitmask.hpp
//!
//! This module provides a type-safe way to use enums as bitmasks, similar to
//! C++'s enum_bitmask template. In Rust, we achieve this using the `bitflags`
//! crate, which is the idiomatic way to handle bitmask enums.
//!
//! ## Usage
//!
//! Instead of the C++ template approach, Rust code should use the `bitflags!`
//! macro from the bitflags crate. This module provides documentation and examples
//! for how to port C++ enum_bitmask usage to Rust.
//!
//! ## Example
//!
//! C++ code:
//! ```cpp
//! enum class Options {
//!     Opt1 = 0,
//!     Opt2 = 1,
//!     Opt3 = 2
//! };
//! ENABLE_ENUM_BITMASK_OPERATORS(Options)
//!
//! enum_bitmask<Options> flags = Options::Opt1 | Options::Opt2;
//! if (flags & Options::Opt1) { ... }
//! ```
//!
//! Rust equivalent:
//! ```rust
//! use bitflags::bitflags;
//!
//! bitflags! {
//!     pub struct Options: u32 {
//!         const OPT1 = 0b001;
//!         const OPT2 = 0b010;
//!         const OPT3 = 0b100;
//!     }
//! }
//!
//! let flags = Options::OPT1 | Options::OPT2;
//! if flags.contains(Options::OPT1) { ... }
//! ```

use std::fmt;
use std::marker::PhantomData;
use std::ops::{BitAnd, BitOr};

/// Type-safe bitmask wrapper for enum types
/// enum_bitmask.hpp:13-46
///
/// This is a Rust adaptation of the C++ enum_bitmask template.
/// In Rust, it's more idiomatic to use the `bitflags!` macro, but this
/// wrapper is provided for direct porting of C++ code.
///
/// C++: template<class option_type, typename = typename std::enable_if<std::is_enum<option_type>::value>::type> class enum_bitmask
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct EnumBitmask<E> {
    /// Internal bitmask storage
    /// enum_bitmask.hpp:46
    /// C++: underlying_type m_bits = 0;
    bits: u32,
    _phantom: PhantomData<E>,
}

impl<E> EnumBitmask<E>
where
    E: Into<u32> + Copy,
{
    /// Create a new empty bitmask
    /// enum_bitmask.hpp:25-26
    /// C++: constexpr enum_bitmask() : m_bits(0) {}
    pub const fn empty() -> Self {
        EnumBitmask {
            bits: 0,
            _phantom: PhantomData,
        }
    }

    /// Create a bitmask with a single bit set
    /// enum_bitmask.hpp:28-32
    /// C++: constexpr enum_bitmask(option_type o) : m_bits(mask_value(o)) {}
    pub fn new(option: E) -> Self {
        EnumBitmask {
            bits: Self::mask_value(option),
            _phantom: PhantomData,
        }
    }

    /// Create a bitmask from a raw bit value
    /// enum_bitmask.hpp:21
    /// C++: explicit constexpr enum_bitmask(underlying_type o) : m_bits(o) {}
    pub const fn from_bits(bits: u32) -> Self {
        EnumBitmask {
            bits,
            _phantom: PhantomData,
        }
    }

    /// Get the raw bit value
    pub const fn bits(&self) -> u32 {
        self.bits
    }

    /// Convert enum value to bit position mask
    /// enum_bitmask.hpp:18
    /// C++: static constexpr underlying_type mask_value(option_type o) { return 1 << static_cast<underlying_type>(o); }
    fn mask_value(option: E) -> u32 {
        1 << option.into()
    }

    /// Check if a specific bit is set
    /// enum_bitmask.hpp:41-42
    /// C++: constexpr bool operator&(option_type t) { return m_bits & mask_value(t); }
    /// C++: constexpr bool has(option_type t) { return m_bits & mask_value(t); }
    pub fn has(&self, option: E) -> bool {
        (self.bits & Self::mask_value(option)) != 0
    }

    /// Check if the bitmask contains all bits from another mask
    pub fn contains(&self, other: Self) -> bool {
        (self.bits & other.bits) == other.bits
    }

    /// Check if the bitmask is empty (no bits set)
    pub const fn is_empty(&self) -> bool {
        self.bits == 0
    }

    /// Check if the bitmask has any bits set
    pub const fn is_some(&self) -> bool {
        self.bits != 0
    }

    /// Set a bit
    pub fn set(&mut self, option: E) {
        self.bits |= Self::mask_value(option);
    }

    /// Clear a bit
    pub fn clear(&mut self, option: E) {
        self.bits &= !Self::mask_value(option);
    }

    /// Toggle a bit
    pub fn toggle(&mut self, option: E) {
        self.bits ^= Self::mask_value(option);
    }

    /// Combine with another enum option
    /// enum_bitmask.hpp:34-35
    /// C++: constexpr enum_bitmask operator|(option_type t) { return enum_bitmask(m_bits | mask_value(t)); }
    pub fn with(self, option: E) -> Self {
        EnumBitmask {
            bits: self.bits | Self::mask_value(option),
            _phantom: PhantomData,
        }
    }

    /// Combine with another bitmask
    /// enum_bitmask.hpp:37-38
    /// C++: constexpr enum_bitmask operator|(enum_bitmask<option_type> t) { return enum_bitmask(m_bits | t.m_bits); }
    pub fn union(self, other: Self) -> Self {
        EnumBitmask {
            bits: self.bits | other.bits,
            _phantom: PhantomData,
        }
    }

    /// Intersect with another bitmask
    pub fn intersection(self, other: Self) -> Self {
        EnumBitmask {
            bits: self.bits & other.bits,
            _phantom: PhantomData,
        }
    }

    /// Remove bits from this mask that are set in another mask
    pub fn difference(self, other: Self) -> Self {
        EnumBitmask {
            bits: self.bits & !other.bits,
            _phantom: PhantomData,
        }
    }
}

impl<E> Default for EnumBitmask<E>
where
    E: Into<u32> + Copy,
{
    fn default() -> Self {
        Self::empty()
    }
}

impl<E> fmt::Debug for EnumBitmask<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "EnumBitmask(0x{:08x})", self.bits)
    }
}

impl<E> fmt::Display for EnumBitmask<E> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "0x{:08x}", self.bits)
    }
}

impl<E> BitOr for EnumBitmask<E>
where
    E: Into<u32> + Copy,
{
    type Output = Self;

    /// Combine two bitmasks with OR
    /// enum_bitmask.hpp:37-38
    fn bitor(self, rhs: Self) -> Self::Output {
        self.union(rhs)
    }
}

impl<E> BitOr<E> for EnumBitmask<E>
where
    E: Into<u32> + Copy,
{
    type Output = Self;

    /// Combine bitmask with enum value using OR
    /// enum_bitmask.hpp:34-35
    fn bitor(self, rhs: E) -> Self::Output {
        self.with(rhs)
    }
}

impl<E> BitAnd for EnumBitmask<E>
where
    E: Into<u32> + Copy,
{
    type Output = Self;

    /// Combine two bitmasks with AND
    fn bitand(self, rhs: Self) -> Self::Output {
        self.intersection(rhs)
    }
}

impl<E> BitAnd<E> for EnumBitmask<E>
where
    E: Into<u32> + Copy,
{
    type Output = bool;

    /// Test if a specific bit is set
    /// enum_bitmask.hpp:41
    fn bitand(self, rhs: E) -> Self::Output {
        self.has(rhs)
    }
}

impl<E> From<E> for EnumBitmask<E>
where
    E: Into<u32> + Copy,
{
    /// Create bitmask from single enum value
    /// enum_bitmask.hpp:28-32
    fn from(option: E) -> Self {
        Self::new(option)
    }
}

/// Helper function for conditional bit setting
/// enum_bitmask.hpp:67-71
/// C++: template <class option_type> constexpr std::enable_if_t<is_enum_bitmask_type_v<option_type>, enum_bitmask<option_type>> only_if(bool condition, option_type opt)
pub fn only_if<E>(condition: bool, option: E) -> EnumBitmask<E>
where
    E: Into<u32> + Copy,
{
    if condition {
        EnumBitmask::new(option)
    } else {
        EnumBitmask::empty()
    }
}

/// Helper function for conditional bitmask
/// enum_bitmask.hpp:73-77
/// C++: template <class option_type> constexpr std::enable_if_t<is_enum_bitmask_type_v<option_type>, enum_bitmask<option_type>> only_if(bool condition, enum_bitmask<option_type> opt)
pub fn only_if_mask<E>(condition: bool, mask: EnumBitmask<E>) -> EnumBitmask<E>
where
    E: Into<u32> + Copy,
{
    if condition {
        mask
    } else {
        EnumBitmask::empty()
    }
}

/// Trait for enabling bitmask operations on enums
/// enum_bitmask.hpp:49-51
///
/// In C++, this is done with:
/// ```cpp
/// template<typename Enum> struct is_enum_bitmask_type { static const bool enable = false; };
/// #define ENABLE_ENUM_BITMASK_OPERATORS(x) template<> struct is_enum_bitmask_type<x> { static const bool enable = true; };
/// ```
///
/// In Rust, implement this trait for your enum type to enable bitmask operations.
pub trait EnumBitmaskType: Into<u32> + Copy {
    /// Enable bitmask operations for this enum
    const ENABLE: bool = true;
}

/// Macro to implement EnumBitmaskType for an enum
///
/// Usage:
/// ```rust
/// #[repr(u32)]
/// enum MyOptions {
///     Opt1 = 0,
///     Opt2 = 1,
///     Opt3 = 2,
/// }
///
/// enable_enum_bitmask!(MyOptions);
/// ```
#[macro_export]
macro_rules! enable_enum_bitmask {
    ($enum_type:ty) => {
        impl $crate::enum_bitmask::EnumBitmaskType for $enum_type {}

        impl std::ops::BitOr for $enum_type {
            type Output = $crate::enum_bitmask::EnumBitmask<Self>;

            fn bitor(self, rhs: Self) -> Self::Output {
                $crate::enum_bitmask::EnumBitmask::new(self).with(rhs)
            }
        }
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[repr(u32)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum TestOptions {
        Opt1 = 0,
        Opt2 = 1,
        Opt3 = 2,
        Opt4 = 3,
    }

    impl From<TestOptions> for u32 {
        fn from(opt: TestOptions) -> u32 {
            opt as u32
        }
    }

    impl EnumBitmaskType for TestOptions {}

    #[test]
    fn test_empty() {
        let mask: EnumBitmask<TestOptions> = EnumBitmask::empty();
        assert!(mask.is_empty());
        assert!(!mask.is_some());
        assert_eq!(mask.bits(), 0);
    }

    #[test]
    fn test_new_single_bit() {
        let mask = EnumBitmask::new(TestOptions::Opt1);
        assert!(!mask.is_empty());
        assert!(mask.is_some());
        assert_eq!(mask.bits(), 0b0001);
    }

    #[test]
    fn test_has() {
        let mask = EnumBitmask::new(TestOptions::Opt2);
        assert!(!mask.has(TestOptions::Opt1));
        assert!(mask.has(TestOptions::Opt2));
        assert!(!mask.has(TestOptions::Opt3));
    }

    #[test]
    fn test_with() {
        let mask = EnumBitmask::new(TestOptions::Opt1).with(TestOptions::Opt3);
        assert!(mask.has(TestOptions::Opt1));
        assert!(!mask.has(TestOptions::Opt2));
        assert!(mask.has(TestOptions::Opt3));
        assert_eq!(mask.bits(), 0b0101);
    }

    #[test]
    fn test_union() {
        let mask1 = EnumBitmask::new(TestOptions::Opt1);
        let mask2 = EnumBitmask::new(TestOptions::Opt2);
        let combined = mask1.union(mask2);

        assert!(combined.has(TestOptions::Opt1));
        assert!(combined.has(TestOptions::Opt2));
        assert_eq!(combined.bits(), 0b0011);
    }

    #[test]
    fn test_intersection() {
        let mask1 = EnumBitmask::new(TestOptions::Opt1)
            .with(TestOptions::Opt2)
            .with(TestOptions::Opt3);
        let mask2 = EnumBitmask::new(TestOptions::Opt2).with(TestOptions::Opt4);

        let intersection = mask1.intersection(mask2);
        assert!(!intersection.has(TestOptions::Opt1));
        assert!(intersection.has(TestOptions::Opt2));
        assert!(!intersection.has(TestOptions::Opt3));
        assert!(!intersection.has(TestOptions::Opt4));
    }

    #[test]
    fn test_difference() {
        let mask1 = EnumBitmask::new(TestOptions::Opt1)
            .with(TestOptions::Opt2)
            .with(TestOptions::Opt3);
        let mask2 = EnumBitmask::new(TestOptions::Opt2);

        let diff = mask1.difference(mask2);
        assert!(diff.has(TestOptions::Opt1));
        assert!(!diff.has(TestOptions::Opt2));
        assert!(diff.has(TestOptions::Opt3));
    }

    #[test]
    fn test_set_clear_toggle() {
        let mut mask = EnumBitmask::new(TestOptions::Opt1);

        mask.set(TestOptions::Opt2);
        assert!(mask.has(TestOptions::Opt1));
        assert!(mask.has(TestOptions::Opt2));

        mask.clear(TestOptions::Opt1);
        assert!(!mask.has(TestOptions::Opt1));
        assert!(mask.has(TestOptions::Opt2));

        mask.toggle(TestOptions::Opt2);
        assert!(!mask.has(TestOptions::Opt2));

        mask.toggle(TestOptions::Opt3);
        assert!(mask.has(TestOptions::Opt3));
    }

    #[test]
    fn test_contains() {
        let full = EnumBitmask::new(TestOptions::Opt1)
            .with(TestOptions::Opt2)
            .with(TestOptions::Opt3);
        let partial = EnumBitmask::new(TestOptions::Opt1).with(TestOptions::Opt2);

        assert!(full.contains(partial));
        assert!(!partial.contains(full));
    }

    #[test]
    fn test_bitor_operator() {
        let mask1 = EnumBitmask::new(TestOptions::Opt1);
        let mask2 = EnumBitmask::new(TestOptions::Opt2);
        let combined = mask1 | mask2;

        assert!(combined.has(TestOptions::Opt1));
        assert!(combined.has(TestOptions::Opt2));
    }

    #[test]
    fn test_bitor_with_enum() {
        let mask = EnumBitmask::new(TestOptions::Opt1) | TestOptions::Opt2;
        assert!(mask.has(TestOptions::Opt1));
        assert!(mask.has(TestOptions::Opt2));
    }

    #[test]
    fn test_bitand_operator() {
        let mask1 = EnumBitmask::new(TestOptions::Opt1)
            .with(TestOptions::Opt2)
            .with(TestOptions::Opt3);
        let mask2 = EnumBitmask::new(TestOptions::Opt2).with(TestOptions::Opt4);

        let intersection = mask1 & mask2;
        assert!(intersection.has(TestOptions::Opt2));
        assert_eq!(intersection.bits(), 0b0010);
    }

    #[test]
    fn test_bitand_with_enum() {
        let mask = EnumBitmask::new(TestOptions::Opt1).with(TestOptions::Opt2);
        assert!(mask & TestOptions::Opt1);
        assert!(mask & TestOptions::Opt2);
        assert!(!(mask & TestOptions::Opt3));
    }

    #[test]
    fn test_from_enum() {
        let mask: EnumBitmask<TestOptions> = TestOptions::Opt1.into();
        assert!(mask.has(TestOptions::Opt1));
    }

    #[test]
    fn test_only_if_true() {
        let mask = only_if(true, TestOptions::Opt1);
        assert!(mask.has(TestOptions::Opt1));
    }

    #[test]
    fn test_only_if_false() {
        let mask = only_if(false, TestOptions::Opt1);
        assert!(mask.is_empty());
    }

    #[test]
    fn test_only_if_mask_true() {
        let original = EnumBitmask::new(TestOptions::Opt1);
        let result = only_if_mask(true, original);
        assert!(result.has(TestOptions::Opt1));
    }

    #[test]
    fn test_only_if_mask_false() {
        let original = EnumBitmask::new(TestOptions::Opt1);
        let result = only_if_mask(false, original);
        assert!(result.is_empty());
    }

    #[test]
    fn test_from_bits() {
        let mask = EnumBitmask::<TestOptions>::from_bits(0b1010);
        assert!(!mask.has(TestOptions::Opt1)); // bit 0
        assert!(mask.has(TestOptions::Opt2)); // bit 1
        assert!(!mask.has(TestOptions::Opt3)); // bit 2
        assert!(mask.has(TestOptions::Opt4)); // bit 3
    }

    #[test]
    fn test_debug_display() {
        let mask = EnumBitmask::new(TestOptions::Opt1).with(TestOptions::Opt2);
        let debug_str = format!("{:?}", mask);
        assert!(debug_str.contains("0x00000003") || debug_str.contains("0x3"));

        let display_str = format!("{}", mask);
        assert!(display_str.contains("0x00000003") || display_str.contains("0x3"));
    }

    #[test]
    fn test_chaining() {
        let mask = EnumBitmask::new(TestOptions::Opt1)
            .with(TestOptions::Opt2)
            .with(TestOptions::Opt3)
            .with(TestOptions::Opt4);

        assert!(mask.has(TestOptions::Opt1));
        assert!(mask.has(TestOptions::Opt2));
        assert!(mask.has(TestOptions::Opt3));
        assert!(mask.has(TestOptions::Opt4));
        assert_eq!(mask.bits(), 0b1111);
    }
}
