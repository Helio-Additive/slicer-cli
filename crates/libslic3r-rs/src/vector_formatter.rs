//! Vector and Map formatting utilities for logging and debugging
//!
//! C++ Reference:
//! - VectorFormatter.hpp (40 lines)
//!
//! This module provides wrapper types that implement Display for pretty-printing
//! collections in log output. Useful for debugging and diagnostic messages.

use std::collections::HashMap;
use std::fmt;

/// Wrapper for formatting vectors and slices with Display trait
/// VectorFormatter.hpp:7-23
pub struct VectorFormatter<'a, T> {
    /// Reference to the vector to format
    /// VectorFormatter.hpp:9
    vec: &'a [T],
}

impl<'a, T> VectorFormatter<'a, T> {
    /// Create a new VectorFormatter wrapping a slice
    /// VectorFormatter.hpp:10
    pub fn new(vec: &'a [T]) -> Self {
        Self { vec }
    }
}

/// Display implementation for VectorFormatter
/// Formats as: [item1, item2, item3]
/// VectorFormatter.hpp:12-22
impl<'a, T: fmt::Display> fmt::Display for VectorFormatter<'a, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // VectorFormatter.hpp:14
        write!(f, "[")?;

        // Iterate through elements
        // VectorFormatter.hpp:15-18
        for (i, item) in self.vec.iter().enumerate() {
            if i > 0 {
                write!(f, ", ")?;
            }
            write!(f, "{}", item)?;
        }

        // VectorFormatter.hpp:19
        write!(f, "]")
    }
}

/// Debug implementation mirrors Display
impl<'a, T: fmt::Display> fmt::Debug for VectorFormatter<'a, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(self, f)
    }
}

/// Wrapper for formatting HashMaps with Display trait
/// VectorFormatter.hpp:26-40
pub struct MapFormatter<'a, K, V> {
    /// Reference to the map to format
    /// VectorFormatter.hpp:28
    map: &'a HashMap<K, V>,
}

impl<'a, K, V> MapFormatter<'a, K, V> {
    /// Create a new MapFormatter wrapping a HashMap
    /// VectorFormatter.hpp:29
    pub fn new(map: &'a HashMap<K, V>) -> Self {
        Self { map }
    }
}

/// Display implementation for MapFormatter
/// Formats as: [key1 : value1, key2 : value2]
/// VectorFormatter.hpp:31-39
impl<'a, K: fmt::Display, V: fmt::Display> fmt::Display for MapFormatter<'a, K, V> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // VectorFormatter.hpp:33
        write!(f, "[")?;

        // Iterate through key-value pairs
        // VectorFormatter.hpp:34-37
        for (i, (key, value)) in self.map.iter().enumerate() {
            if i > 0 {
                write!(f, ", ")?;
            }
            write!(f, "{} : {}", key, value)?;
        }

        // VectorFormatter.hpp:38
        write!(f, "]")
    }
}

/// Debug implementation mirrors Display
impl<'a, K: fmt::Display, V: fmt::Display> fmt::Debug for MapFormatter<'a, K, V> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(self, f)
    }
}

/// Convenience function to create a VectorFormatter
pub fn format_vec<T>(vec: &[T]) -> VectorFormatter<T> {
    VectorFormatter::new(vec)
}

/// Convenience function to create a MapFormatter
pub fn format_map<K, V>(map: &HashMap<K, V>) -> MapFormatter<K, V> {
    MapFormatter::new(map)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vector_formatter_empty() {
        let vec: Vec<i32> = vec![];
        let formatted = VectorFormatter::new(&vec);
        assert_eq!(format!("{}", formatted), "[]");
    }

    #[test]
    fn test_vector_formatter_single() {
        let vec = vec![42];
        let formatted = VectorFormatter::new(&vec);
        assert_eq!(format!("{}", formatted), "[42]");
    }

    #[test]
    fn test_vector_formatter_multiple() {
        let vec = vec![1, 2, 3, 4, 5];
        let formatted = VectorFormatter::new(&vec);
        assert_eq!(format!("{}", formatted), "[1, 2, 3, 4, 5]");
    }

    #[test]
    fn test_vector_formatter_strings() {
        let vec = vec!["hello", "world", "test"];
        let formatted = VectorFormatter::new(&vec);
        assert_eq!(format!("{}", formatted), "[hello, world, test]");
    }

    #[test]
    fn test_map_formatter_empty() {
        let map: HashMap<i32, &str> = HashMap::new();
        let formatted = MapFormatter::new(&map);
        assert_eq!(format!("{}", formatted), "[]");
    }

    #[test]
    fn test_map_formatter_single() {
        let mut map = HashMap::new();
        map.insert(1, "one");
        let formatted = MapFormatter::new(&map);
        assert_eq!(format!("{}", formatted), "[1 : one]");
    }

    #[test]
    fn test_map_formatter_multiple() {
        let mut map = HashMap::new();
        map.insert("a", 1);
        map.insert("b", 2);
        map.insert("c", 3);
        let formatted = MapFormatter::new(&map);
        let output = format!("{}", formatted);

        // HashMap iteration order is not guaranteed, so check components
        assert!(output.starts_with("["));
        assert!(output.ends_with("]"));
        assert!(output.contains("a : 1"));
        assert!(output.contains("b : 2"));
        assert!(output.contains("c : 3"));
    }

    #[test]
    fn test_convenience_functions() {
        let vec = vec![10, 20, 30];
        assert_eq!(format!("{}", format_vec(&vec)), "[10, 20, 30]");

        let mut map = HashMap::new();
        map.insert("x", 100);
        let output = format!("{}", format_map(&map));
        assert_eq!(output, "[x : 100]");
    }

    #[test]
    fn test_debug_trait() {
        let vec = vec![1, 2, 3];
        let formatted = VectorFormatter::new(&vec);
        assert_eq!(format!("{:?}", formatted), "[1, 2, 3]");
    }

    #[test]
    fn test_with_complex_types() {
        #[derive(Debug)]
        struct Point {
            x: i32,
            y: i32,
        }

        impl fmt::Display for Point {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "({}, {})", self.x, self.y)
            }
        }

        let points = vec![Point { x: 0, y: 0 }, Point { x: 10, y: 20 }];
        let formatted = VectorFormatter::new(&points);
        assert_eq!(format!("{}", formatted), "[(0, 0), (10, 20)]");
    }
}
