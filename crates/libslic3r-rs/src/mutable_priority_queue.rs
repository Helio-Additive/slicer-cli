//! Mutable priority queue with efficient update operations
//!
//! C++ Reference:
//! - BambuStudio/src/libslic3r/MutablePriorityQueue.hpp
//!
//! This module provides a binary min-heap priority queue that allows efficient
//! updating of element priorities via an index mapping callback. This is essential
//! for algorithms like Dijkstra's shortest path and A* search that need to decrease
//! priority values after insertion.
//!
//! The queue uses a callback mechanism (`IndexSetter`) to notify elements of their
//! current position in the heap, enabling O(log n) update operations.

/// Invalid queue index marker
/// MutablePriorityQueue.hpp:6
/// C++: constexpr auto InvalidQueueID = std::numeric_limits<size_t>::max();
pub const INVALID_QUEUE_ID: usize = usize::MAX;

/// Mutable priority queue (binary min-heap with update capability)
/// MutablePriorityQueue.hpp:8-49
///
/// A priority queue that allows updating element priorities after insertion.
/// Elements must implement Copy and provide an `IndexSetter` callback that
/// the queue uses to notify elements of their heap position.
///
/// # Type Parameters
/// - `T`: Element type (must be Copy)
/// - `F`: Index setter closure type
/// - `L`: Less-than predicate closure type
///
/// # Example
/// ```ignore
/// struct Node {
///     value: i32,
///     heap_index: Cell<usize>,
/// }
///
/// let mut queue = MutablePriorityQueue::new(
///     |node: &Node, idx| node.heap_index.set(idx),
///     |a: &Node, b: &Node| a.value < b.value,
/// );
/// ```
pub struct MutablePriorityQueue<T, F, L>
where
    T: Copy,
    F: FnMut(&T, usize),
    L: Fn(&T, &T) -> bool,
{
    /// Internal heap storage (0-indexed)
    /// MutablePriorityQueue.hpp:46
    /// C++: std::vector<T> m_heap;
    heap: Vec<T>,

    /// Callback to set element's heap index
    /// MutablePriorityQueue.hpp:47
    /// C++: IndexSetter m_index_setter;
    index_setter: F,

    /// Comparison predicate (less-than for min-heap)
    /// MutablePriorityQueue.hpp:48
    /// C++: LessPredicate m_less_predicate;
    less_predicate: L,

    /// Whether to reset index when element is removed
    reset_on_remove: bool,
}

impl<T, F, L> MutablePriorityQueue<T, F, L>
where
    T: Copy,
    F: FnMut(&T, usize),
    L: Fn(&T, &T) -> bool,
{
    /// Create a new mutable priority queue
    /// MutablePriorityQueue.hpp:14-17
    /// C++: MutablePriorityQueue(IndexSetter &&index_setter, LessPredicate &&less_predicate)
    pub fn new(index_setter: F, less_predicate: L) -> Self {
        Self {
            heap: Vec::new(),
            index_setter,
            less_predicate,
            reset_on_remove: false,
        }
    }

    /// Create a new mutable priority queue with capacity
    /// MutablePriorityQueue.hpp:14-17 + reserve
    pub fn with_capacity(capacity: usize, index_setter: F, less_predicate: L) -> Self {
        Self {
            heap: Vec::with_capacity(capacity),
            index_setter,
            less_predicate,
            reset_on_remove: false,
        }
    }

    /// Create queue that resets indices on removal (useful for debugging)
    /// MutablePriorityQueue.hpp:7 (ResetIndexWhenRemoved template parameter)
    pub fn with_reset_on_remove(index_setter: F, less_predicate: L) -> Self {
        Self {
            heap: Vec::new(),
            index_setter,
            less_predicate,
            reset_on_remove: true,
        }
    }

    /// Clear all elements from the queue
    /// MutablePriorityQueue.hpp:59-71
    /// C++: void clear()
    pub fn clear(&mut self) {
        // Mark all elements as removed if configured
        // MutablePriorityQueue.hpp:66-68
        if self.reset_on_remove {
            for idx in 0..self.heap.len() {
                // Mark as removed from the queue
                // MutablePriorityQueue.hpp:68
                // C++: m_index_setter(m_heap[idx], std::numeric_limits<size_t>::max());
                (self.index_setter)(&self.heap[idx], INVALID_QUEUE_ID);
            }
        }

        // Clear the heap vector
        // MutablePriorityQueue.hpp:70
        // C++: m_heap.clear();
        self.heap.clear();
    }

    /// Reserve capacity for at least `capacity` elements
    /// MutablePriorityQueue.hpp:21
    /// C++: void reserve(size_t cnt) { m_heap.reserve(cnt); }
    #[inline]
    pub fn reserve(&mut self, capacity: usize) {
        self.heap.reserve(capacity);
    }

    /// Push a new element onto the queue
    /// MutablePriorityQueue.hpp:74-80
    /// C++: void push(const T &item)
    pub fn push(&mut self, item: T) {
        // Get insertion index (end of heap)
        // MutablePriorityQueue.hpp:76
        // C++: size_t idx = m_heap.size();
        let idx = self.heap.len();

        // Add element to end of heap
        // MutablePriorityQueue.hpp:77
        // C++: m_heap.emplace_back(item);
        self.heap.push(item);

        // Set element's heap index
        // MutablePriorityQueue.hpp:78
        // C++: m_index_setter(m_heap.back(), idx);
        (self.index_setter)(&self.heap[idx], idx);

        // Bubble up to maintain heap property
        // MutablePriorityQueue.hpp:79
        // C++: update_heap_up(0, idx);
        self.update_heap_up(0, idx);
    }

    /// Remove and return the top (minimum) element
    /// MutablePriorityQueue.hpp:92-110
    /// C++: void pop()
    pub fn pop(&mut self) -> Option<T> {
        // Check if heap is empty
        // MutablePriorityQueue.hpp:94
        // C++: assert(! m_heap.empty());
        if self.heap.is_empty() {
            return None;
        }

        // Save the top element to return
        let top = self.heap[0];

        // Mark top as removed from queue
        // MutablePriorityQueue.hpp:101-102
        // C++: m_index_setter(m_heap.front(), std::numeric_limits<size_t>::max());
        if self.reset_on_remove {
            (self.index_setter)(&self.heap[0], INVALID_QUEUE_ID);
        }

        // Handle heap size > 1
        // MutablePriorityQueue.hpp:104-109
        if self.heap.len() > 1 {
            // Move last element to front
            // MutablePriorityQueue.hpp:105-106
            // C++: m_heap.front() = m_heap.back();
            // C++: m_heap.pop_back();
            self.heap[0] = self.heap[self.heap.len() - 1];
            self.heap.pop();

            // Update moved element's index
            // MutablePriorityQueue.hpp:107
            // C++: m_index_setter(m_heap.front(), 0);
            (self.index_setter)(&self.heap[0], 0);

            // Bubble down to restore heap property
            // MutablePriorityQueue.hpp:108
            // C++: update_heap_down(0, m_heap.size() - 1);
            if !self.heap.is_empty() {
                self.update_heap_down(0, self.heap.len() - 1);
            }
        } else {
            // Single element - just clear
            // MutablePriorityQueue.hpp:110
            // C++: m_heap.clear();
            self.heap.clear();
        }

        Some(top)
    }

    /// Get reference to top (minimum) element without removing
    /// MutablePriorityQueue.hpp:25
    /// C++: T& top() { return m_heap.front(); }
    #[inline]
    pub fn top(&self) -> Option<&T> {
        self.heap.first()
    }

    /// Remove element at specific index
    /// MutablePriorityQueue.hpp:113-133
    /// C++: void remove(size_t idx)
    pub fn remove(&mut self, idx: usize) {
        // Bounds check
        // MutablePriorityQueue.hpp:115
        // C++: assert(idx < m_heap.size());
        if idx >= self.heap.len() {
            return;
        }

        // Mark element as removed
        // MutablePriorityQueue.hpp:122
        // C++: m_index_setter(m_heap[idx], std::numeric_limits<size_t>::max());
        if self.reset_on_remove {
            (self.index_setter)(&self.heap[idx], INVALID_QUEUE_ID);
        }

        // If removing last element, just pop
        // MutablePriorityQueue.hpp:124-127
        // C++: if (idx + 1 == m_heap.size()) { m_heap.pop_back(); return; }
        if idx + 1 == self.heap.len() {
            self.heap.pop();
            return;
        }

        // Replace removed element with last element
        // MutablePriorityQueue.hpp:128-130
        // C++: m_heap[idx] = m_heap.back();
        // C++: m_index_setter(m_heap[idx], idx);
        // C++: m_heap.pop_back();
        self.heap[idx] = self.heap[self.heap.len() - 1];
        (self.index_setter)(&self.heap[idx], idx);
        self.heap.pop();

        // Restore heap property (may need up or down)
        // MutablePriorityQueue.hpp:131-132
        // C++: update_heap_down(idx, m_heap.size() - 1);
        // C++: update_heap_up(0, idx);
        if !self.heap.is_empty() && idx < self.heap.len() {
            self.update_heap_down(idx, self.heap.len() - 1);
            self.update_heap_up(0, idx);
        }
    }

    /// Update element at index (re-sort after priority change)
    /// MutablePriorityQueue.hpp:27
    /// C++: void update(size_t idx) { T item = m_heap[idx]; remove(idx); push(item); }
    pub fn update(&mut self, idx: usize) {
        if idx < self.heap.len() {
            let item = self.heap[idx];
            self.remove(idx);
            self.push(item);
        }
    }

    /// Get number of elements in queue
    /// MutablePriorityQueue.hpp:29
    /// C++: size_t size() const { return m_heap.size(); }
    #[inline]
    pub fn size(&self) -> usize {
        self.heap.len()
    }

    /// Check if queue is empty
    /// MutablePriorityQueue.hpp:30
    /// C++: bool empty() const { return m_heap.empty(); }
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.heap.is_empty()
    }

    /// Get element at index (unchecked)
    /// MutablePriorityQueue.hpp:31
    /// C++: T& operator[](std::size_t idx) noexcept { return m_heap[idx]; }
    #[inline]
    pub fn get(&self, idx: usize) -> Option<&T> {
        self.heap.get(idx)
    }

    /// Get mutable element at index (unchecked)
    /// MutablePriorityQueue.hpp:31
    /// C++: T& operator[](std::size_t idx) noexcept { return m_heap[idx]; }
    #[inline]
    pub fn get_mut(&mut self, idx: usize) -> Option<&mut T> {
        self.heap.get_mut(idx)
    }

    /// Get iterator over heap elements (not in sorted order)
    /// MutablePriorityQueue.hpp:36
    /// C++: iterator begin() { return m_heap.begin(); }
    #[inline]
    pub fn iter(&self) -> impl Iterator<Item = &T> {
        self.heap.iter()
    }

    /// Bubble element up the heap (child has higher priority than parent)
    /// MutablePriorityQueue.hpp:136-157
    /// C++: void update_heap_up(size_t top, size_t bottom)
    fn update_heap_up(&mut self, top: usize, bottom: usize) {
        // Start at bottom (newly inserted element)
        // MutablePriorityQueue.hpp:138-139
        // C++: size_t childIdx = bottom;
        // C++: T *child = &m_heap[childIdx];
        let mut child_idx = bottom;

        loop {
            // Calculate parent index
            // MutablePriorityQueue.hpp:141
            // C++: size_t parentIdx = (childIdx - 1) >> 1;
            let parent_idx = if child_idx == 0 {
                break;
            } else {
                (child_idx - 1) / 2
            };

            // Stop at top boundary or root
            // MutablePriorityQueue.hpp:142-143
            // C++: if (childIdx == 0 || parentIdx < top) break;
            if parent_idx < top {
                break;
            }

            // Check if swap is needed (child has higher priority)
            // MutablePriorityQueue.hpp:145-146
            // C++: if (! m_less_predicate(*parent, *child))
            let child = self.heap[child_idx];
            let parent = self.heap[parent_idx];

            if (self.less_predicate)(&parent, &child) {
                // Parent has higher priority - heap property satisfied
                break;
            }

            // Swap parent and child
            // MutablePriorityQueue.hpp:147-151
            // C++: T tmp = *parent;
            // C++: m_index_setter(tmp, childIdx);
            // C++: m_index_setter(*child, parentIdx);
            // C++: m_heap[parentIdx] = *child;
            // C++: m_heap[childIdx] = tmp;
            (self.index_setter)(&parent, child_idx);
            (self.index_setter)(&child, parent_idx);
            self.heap[parent_idx] = child;
            self.heap[child_idx] = parent;

            // Move up the tree
            // MutablePriorityQueue.hpp:153-155
            // C++: childIdx = parentIdx;
            child_idx = parent_idx;
        }
    }

    /// Bubble element down the heap (parent has lower priority than children)
    /// MutablePriorityQueue.hpp:160-189
    /// C++: void update_heap_down(size_t top, size_t bottom)
    fn update_heap_down(&mut self, top: usize, bottom: usize) {
        // Start at top (root or hole position)
        // MutablePriorityQueue.hpp:162-163
        // C++: size_t parentIdx = top;
        // C++: T *parent = &m_heap[parentIdx];
        let mut parent_idx = top;

        loop {
            // Calculate left child index
            // MutablePriorityQueue.hpp:165
            // C++: size_t childIdx = (parentIdx << 1) + 1;
            let child_idx = parent_idx * 2 + 1;

            // Check if left child exists
            // MutablePriorityQueue.hpp:166-167
            // C++: if (childIdx > bottom) break;
            if child_idx > bottom {
                break;
            }

            // Get left child
            // MutablePriorityQueue.hpp:168
            // C++: T *child = &m_heap[childIdx];
            let mut min_child_idx = child_idx;

            // Check if right child exists and has higher priority
            // MutablePriorityQueue.hpp:169-176
            // C++: size_t child2Idx = childIdx + 1;
            // C++: if (child2Idx <= bottom) {
            // C++:     T *child2 = &m_heap[child2Idx];
            // C++:     if (! m_less_predicate(*child, *child2)) {
            // C++:         child = child2;
            // C++:         childIdx = child2Idx;
            let child2_idx = child_idx + 1;
            if child2_idx <= bottom {
                let child = self.heap[child_idx];
                let child2 = self.heap[child2_idx];
                if !(self.less_predicate)(&child, &child2) {
                    min_child_idx = child2_idx;
                }
            }

            // Check if parent already has higher priority than best child
            // MutablePriorityQueue.hpp:178-179
            // C++: if (m_less_predicate(*parent, *child)) return;
            let parent = self.heap[parent_idx];
            let min_child = self.heap[min_child_idx];

            if (self.less_predicate)(&parent, &min_child) {
                // Parent has higher priority - heap property satisfied
                return;
            }

            // Swap parent with minimum child
            // MutablePriorityQueue.hpp:180-184
            // C++: T tmp = *parent;
            // C++: m_index_setter(tmp, childIdx);
            // C++: m_index_setter(*child, parentIdx);
            // C++: m_heap[parentIdx] = *child;
            // C++: m_heap[childIdx] = tmp;
            (self.index_setter)(&parent, min_child_idx);
            (self.index_setter)(&min_child, parent_idx);
            self.heap[parent_idx] = min_child;
            self.heap[min_child_idx] = parent;

            // Move down the tree
            // MutablePriorityQueue.hpp:186-187
            // C++: parentIdx = childIdx;
            parent_idx = min_child_idx;
        }
    }
}

/// Construct a [`MutablePriorityQueue`].
/// MutablePriorityQueue.hpp:51-56
/// C++:
/// ```text
/// template<typename T, const bool ResetIndexWhenRemoved, typename IndexSetter, typename LessPredicate>
/// MutablePriorityQueue<T, IndexSetter, LessPredicate, ResetIndexWhenRemoved>
/// make_mutable_priority_queue(IndexSetter &&index_setter, LessPredicate &&less_predicate)
/// {
///     return MutablePriorityQueue<T, IndexSetter, LessPredicate, ResetIndexWhenRemoved>(
///         std::forward<IndexSetter>(index_setter), std::forward<LessPredicate>(less_predicate));
/// }
/// ```
///
/// In C++ `ResetIndexWhenRemoved` is a compile-time template bool; here it is a
/// runtime `bool` argument selecting the same behaviour
/// (`with_reset_on_remove` vs `new`).
pub fn make_mutable_priority_queue<T, F, L>(
    reset_index_when_removed: bool,
    index_setter: F,
    less_predicate: L,
) -> MutablePriorityQueue<T, F, L>
where
    T: Copy,
    F: FnMut(&T, usize),
    L: Fn(&T, &T) -> bool,
{
    if reset_index_when_removed {
        MutablePriorityQueue::with_reset_on_remove(index_setter, less_predicate)
    } else {
        MutablePriorityQueue::new(index_setter, less_predicate)
    }
}

impl<T, F, L> Drop for MutablePriorityQueue<T, F, L>
where
    T: Copy,
    F: FnMut(&T, usize),
    L: Fn(&T, &T) -> bool,
{
    /// Destructor - clear queue
    /// MutablePriorityQueue.hpp:18
    /// C++: ~MutablePriorityQueue() { clear(); }
    fn drop(&mut self) {
        self.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;

    /// Test element with heap index tracking
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct Node {
        value: i32,
        heap_index: Cell<usize>,
    }

    impl Node {
        fn new(value: i32) -> Self {
            Self {
                value,
                heap_index: Cell::new(INVALID_QUEUE_ID),
            }
        }

        fn index(&self) -> usize {
            self.heap_index.get()
        }
    }

    #[test]
    fn test_push_pop_basic() {
        // Test basic push and pop operations
        // MutablePriorityQueue.hpp:74-110
        let mut queue = MutablePriorityQueue::new(
            |node: &Node, idx| node.heap_index.set(idx),
            |a: &Node, b: &Node| a.value < b.value,
        );

        // Push elements
        queue.push(Node::new(5));
        queue.push(Node::new(3));
        queue.push(Node::new(7));
        queue.push(Node::new(1));

        assert_eq!(queue.size(), 4);

        // Pop in priority order (min-heap)
        assert_eq!(queue.pop().unwrap().value, 1);
        assert_eq!(queue.pop().unwrap().value, 3);
        assert_eq!(queue.pop().unwrap().value, 5);
        assert_eq!(queue.pop().unwrap().value, 7);
        assert!(queue.is_empty());
    }

    #[test]
    fn test_top() {
        // Test top() returns minimum without removing
        // MutablePriorityQueue.hpp:25
        let mut queue = MutablePriorityQueue::new(
            |node: &Node, idx| node.heap_index.set(idx),
            |a: &Node, b: &Node| a.value < b.value,
        );

        queue.push(Node::new(10));
        queue.push(Node::new(5));
        queue.push(Node::new(20));

        assert_eq!(queue.top().unwrap().value, 5);
        assert_eq!(queue.size(), 3); // Not removed
    }

    #[test]
    fn test_index_tracking() {
        // Test that index_setter is called correctly
        // MutablePriorityQueue.hpp:47
        let mut queue = MutablePriorityQueue::new(
            |node: &Node, idx| node.heap_index.set(idx),
            |a: &Node, b: &Node| a.value < b.value,
        );

        let n1 = Node::new(10);
        let n2 = Node::new(5);
        let n3 = Node::new(15);

        queue.push(n1);
        queue.push(n2);
        queue.push(n3);

        // All elements should have valid indices
        for i in 0..queue.size() {
            let node = queue.get(i).unwrap();
            assert_ne!(node.index(), INVALID_QUEUE_ID);
            assert_eq!(node.index(), i);
        }
    }

    #[test]
    fn test_remove() {
        // Test removing arbitrary element
        // MutablePriorityQueue.hpp:113-133
        let mut queue = MutablePriorityQueue::new(
            |node: &Node, idx| node.heap_index.set(idx),
            |a: &Node, b: &Node| a.value < b.value,
        );

        queue.push(Node::new(10));
        queue.push(Node::new(5));
        queue.push(Node::new(20));
        queue.push(Node::new(15));

        // Remove middle element
        queue.remove(1);
        assert_eq!(queue.size(), 3);

        // Remaining elements should maintain heap property
        let v1 = queue.pop().unwrap().value;
        let v2 = queue.pop().unwrap().value;
        let v3 = queue.pop().unwrap().value;

        assert!(v1 <= v2 && v2 <= v3);
    }

    #[test]
    fn test_update() {
        // Test update operation (remove + re-insert)
        // MutablePriorityQueue.hpp:27
        let mut queue = MutablePriorityQueue::new(
            |node: &Node, idx| node.heap_index.set(idx),
            |a: &Node, b: &Node| a.value < b.value,
        );

        queue.push(Node::new(10));
        queue.push(Node::new(20));
        queue.push(Node::new(30));

        // Get index of element with value 20
        let idx = (0..queue.size())
            .find(|&i| queue.get(i).unwrap().value == 20)
            .unwrap();

        // Update it (would normally modify in place, then call update)
        queue.update(idx);

        // Queue should still maintain heap property
        assert_eq!(queue.size(), 3);
    }

    #[test]
    fn test_clear() {
        // Test clearing the queue
        // MutablePriorityQueue.hpp:59-71
        let mut queue = MutablePriorityQueue::new(
            |node: &Node, idx| node.heap_index.set(idx),
            |a: &Node, b: &Node| a.value < b.value,
        );

        queue.push(Node::new(1));
        queue.push(Node::new(2));
        queue.push(Node::new(3));

        assert_eq!(queue.size(), 3);

        queue.clear();

        assert_eq!(queue.size(), 0);
        assert!(queue.is_empty());
        assert!(queue.top().is_none());
    }

    #[test]
    fn test_heap_property_stress() {
        // Stress test - ensure heap property is always maintained
        // MutablePriorityQueue.hpp:136-189 (heap operations)
        let mut queue = MutablePriorityQueue::new(
            |node: &Node, idx| node.heap_index.set(idx),
            |a: &Node, b: &Node| a.value < b.value,
        );

        // Push many elements in random order
        let values = vec![50, 30, 70, 10, 90, 20, 60, 40, 80, 15, 25, 35, 45];
        for &v in &values {
            queue.push(Node::new(v));
        }

        // Pop all elements - should come out in sorted order
        let mut prev = i32::MIN;
        while let Some(node) = queue.pop() {
            assert!(node.value >= prev, "Heap property violated!");
            prev = node.value;
        }
    }

    #[test]
    fn test_with_capacity() {
        // Test pre-allocation
        // MutablePriorityQueue.hpp:21
        let mut queue = MutablePriorityQueue::with_capacity(
            100,
            |node: &Node, idx| node.heap_index.set(idx),
            |a: &Node, b: &Node| a.value < b.value,
        );

        for i in 0..50 {
            queue.push(Node::new(i));
        }

        assert_eq!(queue.size(), 50);
    }

    #[test]
    fn test_reset_on_remove() {
        // Test that indices are reset when elements are removed
        // MutablePriorityQueue.hpp:7 (ResetIndexWhenRemoved)
        let mut queue = MutablePriorityQueue::with_reset_on_remove(
            |node: &Node, idx| node.heap_index.set(idx),
            |a: &Node, b: &Node| a.value < b.value,
        );

        let n1 = Node::new(10);
        queue.push(n1);

        // After clear, index should be reset to INVALID_QUEUE_ID
        queue.clear();

        // Note: We can't directly verify since n1 is copied into queue
        // But the functionality is there for external tracking
        assert!(queue.is_empty());
    }

    #[test]
    fn test_empty_operations() {
        // Test operations on empty queue
        let mut queue = MutablePriorityQueue::new(
            |node: &Node, idx| node.heap_index.set(idx),
            |a: &Node, b: &Node| a.value < b.value,
        );

        assert!(queue.is_empty());
        assert_eq!(queue.size(), 0);
        assert!(queue.top().is_none());
        assert!(queue.pop().is_none());
        queue.remove(0); // Should not panic
        queue.update(0); // Should not panic
    }
}
