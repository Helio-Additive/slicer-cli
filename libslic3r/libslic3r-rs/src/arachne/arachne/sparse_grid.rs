//! Sparse grid for spatial queries in Arachne.
//!
//! Provides efficient nearest-neighbor and range queries.

use crate::geometry::Point;
use crate::CoordF;
use std::collections::HashMap;

/// Sparse grid for spatial indexing
/// Arachne/utils/SparseGrid.hpp:24-29
pub struct SparseGrid<T> {
    cell_size: CoordF,
    cells: HashMap<(i64, i64), Vec<T>>,
}

/// Implementation of SparseGrid methods
/// Arachne/utils/SparseGrid.hpp:96-99
impl<T> SparseGrid<T> {
    /// Create a new sparse grid with the given cell size
    /// Arachne/utils/SparseGrid.hpp:40-42
    pub fn new(cell_size: CoordF) -> Self {
        // Initialize sparse grid with cell size and empty cell map
        // Arachne/utils/SparseGrid.hpp:96-99
        Self {
            cell_size,
            cells: HashMap::new(),
        }
    }

    /// Insert an item at a position - maps point to grid cell and inserts item into that cell
    /// Arachne/utils/SparseGrid.hpp:27-28
    pub fn insert(&mut self, position: Point, item: T) {
        // Calculate grid cell coordinates and insert item
        // Arachne/utils/SparseGrid.hpp:27-28
        let cell = self.cell_for_point(position);
        // Insert item into the cell's vector in the hash map
        // Arachne/utils/SparseGrid.hpp:27-28
        self.cells.entry(cell).or_default().push(item);
    }

    /// Get the cell coordinates for a point - converts point coordinates to grid cell indices
    /// Arachne/utils/SquareGrid.hpp:35-40
    fn cell_for_point(&self, point: Point) -> (i64, i64) {
        // Convert point coordinates to grid cell by dividing by cell size
        // Arachne/utils/SquareGrid.hpp:37
        (
            (point.x as f64 / self.cell_size).floor() as i64,
            (point.y as f64 / self.cell_size).floor() as i64,
        )
    }

    /// Get items in a cell - retrieves all items stored in the specified grid cell
    /// Arachne/utils/SparseGrid.hpp:82-89
    pub fn get_cell(&self, cell: (i64, i64)) -> Option<&Vec<T>> {
        // Lookup cell in hash map
        // Arachne/utils/SparseGrid.hpp:92
        self.cells.get(&cell)
    }
}
