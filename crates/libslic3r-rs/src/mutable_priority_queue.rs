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

    /// Bubble element up the heap.
    /// MutablePriorityQueue.hpp:136-157
    /// C++: void update_heap_up(size_t top, size_t bottom)
    ///
    /// NOTE: A faithful 1:1 translation. The C++ loop does NOT break after a
    /// successful (or skipped) comparison; it always shifts up to `top`,
    /// carrying `child` along (`child = parent`). The previous Rust version
    /// added an early `break` on the predicate, which diverged from C++ in the
    /// `index_setter` side-effect ordering; that has been corrected.
    fn update_heap_up(&mut self, top: usize, bottom: usize) {
        // MutablePriorityQueue.hpp:138-139
        // C++: size_t childIdx = bottom;
        // C++: T *child = &m_heap[childIdx];
        let mut child_idx = bottom;
        // `child` mirrors `T *child` by carrying the value across iterations.
        let mut child = self.heap[child_idx];
        // MutablePriorityQueue.hpp:140
        // C++: for (;;) {
        loop {
            // MutablePriorityQueue.hpp:141
            // C++: size_t parentIdx = (childIdx - 1) >> 1;
            let parent_idx = (child_idx.wrapping_sub(1)) >> 1;
            // MutablePriorityQueue.hpp:142-143
            // C++: if (childIdx == 0 || parentIdx < top) break;
            if child_idx == 0 || parent_idx < top {
                break;
            }
            // MutablePriorityQueue.hpp:144
            // C++: T *parent = &m_heap[parentIdx];
            let parent = self.heap[parent_idx];
            // switch nodes
            // MutablePriorityQueue.hpp:145-152
            // C++: if (! m_less_predicate(*parent, *child)) {
            if !(self.less_predicate)(&parent, &child) {
                // C++: T tmp = *parent;
                // C++: m_index_setter(tmp,    childIdx);
                // C++: m_index_setter(*child, parentIdx);
                // C++: m_heap[parentIdx] = *child;
                // C++: m_heap[childIdx]  = tmp;
                let tmp = parent;
                (self.index_setter)(&tmp, child_idx);
                (self.index_setter)(&child, parent_idx);
                self.heap[parent_idx] = child;
                self.heap[child_idx] = tmp;
            }
            // shift up
            // MutablePriorityQueue.hpp:153-155
            // C++: childIdx = parentIdx;
            // C++: child = parent;
            child_idx = parent_idx;
            // `child = parent` carries the value at `m_heap[parentIdx]`. Note that
            // after a swap `m_heap[parentIdx]` now holds the old child value, but
            // C++ keeps `child` pointing at that address, so `child` becomes the
            // value now residing at `parentIdx`.
            child = self.heap[parent_idx];
        }
    }

    /// Bubble element down the heap.
    /// MutablePriorityQueue.hpp:160-189
    /// C++: void update_heap_down(size_t top, size_t bottom)
    fn update_heap_down(&mut self, top: usize, bottom: usize) {
        // MutablePriorityQueue.hpp:162-163
        // C++: size_t parentIdx = top;
        // C++: T *parent = &m_heap[parentIdx];
        let mut parent_idx = top;
        let mut parent = self.heap[parent_idx];
        // MutablePriorityQueue.hpp:164
        // C++: for (;;) {
        loop {
            // MutablePriorityQueue.hpp:165
            // C++: size_t childIdx = (parentIdx << 1) + 1;
            let mut child_idx = (parent_idx << 1) + 1;
            // MutablePriorityQueue.hpp:166-167
            // C++: if (childIdx > bottom) break;
            if child_idx > bottom {
                break;
            }
            // MutablePriorityQueue.hpp:168
            // C++: T *child = &m_heap[childIdx];
            let mut child = self.heap[child_idx];
            // MutablePriorityQueue.hpp:169-176
            // C++: size_t child2Idx = childIdx + 1;
            let child2_idx = child_idx + 1;
            // C++: if (child2Idx <= bottom) {
            if child2_idx <= bottom {
                // C++: T *child2 = &m_heap[child2Idx];
                let child2 = self.heap[child2_idx];
                // C++: if (! m_less_predicate(*child, *child2)) {
                if !(self.less_predicate)(&child, &child2) {
                    // C++: child = child2;
                    // C++: childIdx = child2Idx;
                    child = child2;
                    child_idx = child2_idx;
                }
            }
            // MutablePriorityQueue.hpp:177-178
            // C++: if (m_less_predicate(*parent, *child)) return;
            if (self.less_predicate)(&parent, &child) {
                return;
            }
            // switch nodes
            // MutablePriorityQueue.hpp:179-184
            // C++: T tmp = *parent;
            // C++: m_index_setter(tmp,    childIdx);
            // C++: m_index_setter(*child, parentIdx);
            // C++: m_heap[parentIdx] = *child;
            // C++: m_heap[childIdx] = tmp;
            let tmp = parent;
            (self.index_setter)(&tmp, child_idx);
            (self.index_setter)(&child, parent_idx);
            self.heap[parent_idx] = child;
            self.heap[child_idx] = tmp;
            // shift down
            // MutablePriorityQueue.hpp:185-187
            // C++: parentIdx = childIdx;
            // C++: parent = child;
            parent_idx = child_idx;
            parent = self.heap[child_idx];
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

// Binary heap addressing of a hierarchy of binary miniheaps by a higher level binary heap.
// Conceptually it works the same as a plain binary heap, however it is cache friendly.
// A binary block of "block_size" implements a binary miniheap of (block_size / 2) leaves and
// ((block_size / 2) - 1) nodes, thus wasting a single element. To make addressing simpler,
// the zero'th element inside each miniheap is wasted, thus for example a single element heap is
// 2 elements long and the 1st element starts at address 1.
//
// Mostly copied from the following great source:
// https://playfulprogramming.blogspot.com/2015/08/cache-optimizing-priority-queue.html
// https://github.com/rollbear/prio_queue/blob/master/prio_queue.hpp
// original source Copyright Björn Fahller 2015, Boost Software License, Version 1.0, http://www.boost.org/LICENSE_1_0.txt
// MutablePriorityQueue.hpp:191-251
// C++: template <std::size_t blocking> struct SkipHeapAddressing
///
/// Cache-friendly skip-heap addressing helper. In C++ `blocking` is a
/// compile-time `std::size_t` template parameter; here it is a runtime `usize`
/// (`block_size`) carried inside the struct so that the same arithmetic can be
/// performed without const generics.
#[derive(Clone, Copy)]
pub struct SkipHeapAddressing {
    /// MutablePriorityQueue.hpp:206
    /// C++: static const constexpr std::size_t block_size = blocking;
    pub block_size: usize,
    /// MutablePriorityQueue.hpp:207
    /// C++: static const constexpr std::size_t block_mask = block_size - 1;
    pub block_mask: usize,
}

impl SkipHeapAddressing {
    /// Construct addressing for the given block size.
    /// MutablePriorityQueue.hpp:206-208
    /// C++: static_assert((block_size & block_mask) == 0U, "block size must be 2^n for some integer n");
    pub fn new(block_size: usize) -> Self {
        let block_mask = block_size - 1;
        // C++ static_assert: block size must be 2^n for some integer n.
        debug_assert!((block_size & block_mask) == 0, "block size must be 2^n for some integer n");
        Self { block_size, block_mask }
    }

    /// MutablePriorityQueue.hpp:210-218
    /// C++: static inline std::size_t child_of(std::size_t node_no) noexcept
    #[inline]
    pub fn child_of(&self, node_no: usize) -> usize {
        // MutablePriorityQueue.hpp:211-215
        // C++: if (! is_block_leaf(node_no))
        if !self.is_block_leaf(node_no) {
            // If not a leaf, then it is sufficient to just traverse down inside a miniheap.
            // The following line is equivalent to, but quicker than
            // return block_base(node_no) + 2 * block_offset(node_no);
            // C++: return node_no + block_offset(node_no);
            return node_no + self.block_offset(node_no);
        }
        // Otherwise skip to a root of a child miniheap.
        // MutablePriorityQueue.hpp:217
        // C++: return (block_base(node_no) + 1 + child_no(node_no) * 2) * block_size + 1;
        (self.block_base(node_no) + 1 + self.child_no(node_no) * 2) * self.block_size + 1
    }

    /// MutablePriorityQueue.hpp:220-235
    /// C++: static inline std::size_t parent_of(std::size_t node_no) noexcept
    #[inline]
    pub fn parent_of(&self, node_no: usize) -> usize {
        // MutablePriorityQueue.hpp:221
        // C++: auto const node_root = block_base(node_no); // 16
        let node_root = self.block_base(node_no); // 16
        // MutablePriorityQueue.hpp:222-224
        // C++: if (! is_block_root(node_no))
        if !self.is_block_root(node_no) {
            // If not a block (miniheap) root, then it is sufficient to just traverse up inside a miniheap.
            // C++: return node_root + block_offset(node_no) / 2;
            return node_root + self.block_offset(node_no) / 2;
        }
        // Otherwise skipping from a root of one miniheap into leaf of another miniheap.
        // Address of a parent miniheap block. One miniheap branches at (block_size / 2) leaves to (block_size) miniheaps.
        // MutablePriorityQueue.hpp:227
        // C++: auto const parent_base = block_base(node_root / block_size - 1); // 0
        let parent_base = self.block_base(node_root / self.block_size - 1); // 0
        // Index of a leaf of a parent miniheap, which is a parent of node_no.
        // MutablePriorityQueue.hpp:229
        // C++: auto const child = ((node_no - block_size) / block_size - parent_base) / 2;
        let child = ((node_no - self.block_size) / self.block_size - parent_base) / 2;
        // MutablePriorityQueue.hpp:230-234
        // C++: return parent_base + block_size / 2 + child; // 30
        // Address of a parent miniheap
        parent_base +
            // Address of a leaf of a parent miniheap
            self.block_size / 2 + child // 30
    }

    /// Leafs are stored inside the second half of a block.
    /// MutablePriorityQueue.hpp:238
    /// C++: static inline bool is_block_leaf(std::size_t node_no) noexcept { return (node_no & (block_size >> 1)) != 0U; }
    #[inline]
    pub fn is_block_leaf(&self, node_no: usize) -> bool {
        (node_no & (self.block_size >> 1)) != 0
    }

    /// Unused space aka padding to facilitate quick addressing.
    /// MutablePriorityQueue.hpp:240
    /// C++: static inline bool is_padding(std::size_t node_no) noexcept { return block_offset(node_no) == 0U; }
    #[inline]
    pub fn is_padding(&self, node_no: usize) -> bool {
        self.block_offset(node_no) == 0
    }

    // Following methods are internal, but made public for unit tests.
    //private:
    /// Address is a root of a block (of a miniheap).
    /// MutablePriorityQueue.hpp:244
    /// C++: static inline bool is_block_root(std::size_t node_no) noexcept { return block_offset(node_no) == 1U; }
    #[inline]
    pub fn is_block_root(&self, node_no: usize) -> bool {
        self.block_offset(node_no) == 1
    }

    /// Offset inside a block (inside a miniheap).
    /// MutablePriorityQueue.hpp:246
    /// C++: static inline std::size_t block_offset(std::size_t node_no) noexcept { return node_no & block_mask; }
    #[inline]
    pub fn block_offset(&self, node_no: usize) -> usize {
        node_no & self.block_mask
    }

    /// Base address of a block (a miniheap).
    /// MutablePriorityQueue.hpp:248
    /// C++: static inline std::size_t block_base(std::size_t node_no) noexcept { return node_no & ~block_mask; }
    #[inline]
    pub fn block_base(&self, node_no: usize) -> usize {
        node_no & !self.block_mask
    }

    /// Index of a leaf.
    /// MutablePriorityQueue.hpp:250
    /// C++: static inline std::size_t child_no(std::size_t node_no) noexcept { assert(is_block_leaf(node_no)); return node_no & (block_mask >> 1); }
    #[inline]
    pub fn child_no(&self, node_no: usize) -> usize {
        debug_assert!(self.is_block_leaf(node_no));
        node_no & (self.block_mask >> 1)
    }
}

/// Default block size (`blocking`) for [`MutableSkipHeapPriorityQueue`].
/// MutablePriorityQueue.hpp:255
/// C++: std::size_t blocking = 32
pub const DEFAULT_SKIP_HEAP_BLOCKING: usize = 32;

/// Cache friendly variant of MutablePriorityQueue, implemented as a binary heap of binary miniheaps,
/// building upon SkipHeapAddressing.
/// MutablePriorityQueue.hpp:253-299
/// C++: template<typename T, typename IndexSetter, typename LessPredicate, std::size_t blocking = 32, const bool ResetIndexWhenRemoved = false> class MutableSkipHeapPriorityQueue
///
/// `T` must be trivially copyable (`Copy`) and constructible (`Default`, used
/// to create the padding element — C++ `T()`).
pub struct MutableSkipHeapPriorityQueue<T, F, L>
where
    T: Copy + Default,
    F: FnMut(&T, usize),
    L: Fn(&T, &T) -> bool,
{
    /// MutablePriorityQueue.hpp:296
    /// C++: std::vector<T> m_heap;
    heap: Vec<T>,
    /// MutablePriorityQueue.hpp:297
    /// C++: IndexSetter m_index_setter;
    index_setter: F,
    /// MutablePriorityQueue.hpp:298
    /// C++: LessPredicate m_less_predicate;
    less_predicate: L,
    /// `using address = SkipHeapAddressing<blocking>;`
    /// MutablePriorityQueue.hpp:260
    address: SkipHeapAddressing,
    /// Runtime mirror of the C++ `ResetIndexWhenRemoved` template bool.
    /// MutablePriorityQueue.hpp:255
    reset_on_remove: bool,
}

impl<T, F, L> MutableSkipHeapPriorityQueue<T, F, L>
where
    T: Copy + Default,
    F: FnMut(&T, usize),
    L: Fn(&T, &T) -> bool,
{
    /// MutablePriorityQueue.hpp:263-266
    /// C++: MutableSkipHeapPriorityQueue(IndexSetter &&index_setter, LessPredicate &&less_predicate)
    ///
    /// `blocking` is the C++ template `std::size_t blocking` (default 32) and
    /// `reset_on_remove` is the C++ template `ResetIndexWhenRemoved` (default false).
    pub fn new(blocking: usize, reset_on_remove: bool, index_setter: F, less_predicate: L) -> Self {
        Self {
            heap: Vec::new(),
            index_setter,
            less_predicate,
            address: SkipHeapAddressing::new(blocking),
            reset_on_remove,
        }
    }

    /// Clear all elements from the queue.
    /// MutablePriorityQueue.hpp:309-323
    /// C++: void clear()
    pub fn clear(&mut self) {
        // Only mark as removed from the queue in release mode, if configured so.
        // MutablePriorityQueue.hpp:314
        // C++: if (ResetIndexWhenRemoved)
        if self.reset_on_remove {
            // MutablePriorityQueue.hpp:317-320
            // C++: for (size_t idx = 0; idx < m_heap.size(); ++ idx)
            for idx in 0..self.heap.len() {
                // Mark as removed from the queue.
                // C++: if (! address::is_padding(idx))
                if !self.address.is_padding(idx) {
                    // C++: m_index_setter(m_heap[idx], std::numeric_limits<size_t>::max());
                    (self.index_setter)(&self.heap[idx], INVALID_QUEUE_ID);
                }
            }
        }
        // MutablePriorityQueue.hpp:322
        // C++: m_heap.clear();
        self.heap.clear();
    }

    /// Reserve capacity. Reserve one unused element per miniheap.
    /// MutablePriorityQueue.hpp:270-271
    /// C++: void reserve(size_t cnt) { m_heap.reserve(cnt + ((cnt + (address::block_size - 1)) / (address::block_size - 1))); }
    #[inline]
    pub fn reserve(&mut self, cnt: usize) {
        self.heap
            .reserve(cnt + ((cnt + (self.address.block_size - 1)) / (self.address.block_size - 1)));
    }

    /// Push a new element (by value; mirrors both C++ `push(const T&)` and `push(T&&)`).
    /// MutablePriorityQueue.hpp:325-345
    /// C++: void push(const T &item) / void push(T &&item)
    pub fn push(&mut self, item: T) {
        // MutablePriorityQueue.hpp:328-329
        // C++: if (address::is_padding(m_heap.size())) m_heap.emplace_back(T());
        if self.address.is_padding(self.heap.len()) {
            self.heap.push(T::default());
        }
        // MutablePriorityQueue.hpp:330
        // C++: size_t idx = m_heap.size();
        let idx = self.heap.len();
        // MutablePriorityQueue.hpp:331
        // C++: m_heap.emplace_back(item);
        self.heap.push(item);
        // MutablePriorityQueue.hpp:332
        // C++: m_index_setter(m_heap.back(), idx);
        (self.index_setter)(&self.heap[idx], idx);
        // MutablePriorityQueue.hpp:333
        // C++: update_heap_up(1, idx);
        self.update_heap_up(1, idx);
    }

    /// Remove the top (minimum) element.
    /// MutablePriorityQueue.hpp:347-367
    /// C++: void pop()
    ///
    /// Returns the removed top element (`None` if empty); C++ returns `void`
    /// and asserts non-empty.
    pub fn pop(&mut self) -> Option<T> {
        // MutablePriorityQueue.hpp:350
        // C++: assert(! m_heap.empty());
        if self.heap.is_empty() {
            return None;
        }
        // The top is at index 1 (index 0 is padding).
        let top = self.heap[1];
        // Only mark as removed from the queue in release mode, if configured so.
        // MutablePriorityQueue.hpp:353-357
        // C++: if (ResetIndexWhenRemoved) m_index_setter(m_heap[1], std::numeric_limits<size_t>::max());
        if self.reset_on_remove {
            (self.index_setter)(&self.heap[1], INVALID_QUEUE_ID);
        }
        // Zero'th element is padding, thus non-empty queue must have at least two elements.
        // MutablePriorityQueue.hpp:360-366
        // C++: if (m_heap.size() > 2) {
        if self.heap.len() > 2 {
            // C++: m_heap[1] = m_heap.back();
            self.heap[1] = self.heap[self.heap.len() - 1];
            // C++: this->pop_back();
            self.pop_back();
            // C++: m_index_setter(m_heap[1], 1);
            (self.index_setter)(&self.heap[1], 1);
            // C++: update_heap_down(1, m_heap.size() - 1);
            self.update_heap_down(1, self.heap.len() - 1);
        } else {
            // C++: m_heap.clear();
            self.heap.clear();
        }
        Some(top)
    }

    /// Get reference to the top (minimum) element.
    /// MutablePriorityQueue.hpp:275
    /// C++: T& top() { return m_heap[1]; }
    #[inline]
    pub fn top(&self) -> Option<&T> {
        self.heap.get(1)
    }

    /// Remove element at index `idx`.
    /// MutablePriorityQueue.hpp:369-391
    /// C++: void remove(size_t idx)
    pub fn remove(&mut self, idx: usize) {
        // MutablePriorityQueue.hpp:372-373
        // C++: assert(idx < m_heap.size());
        // C++: assert(! address::is_padding(idx));
        debug_assert!(idx < self.heap.len());
        debug_assert!(!self.address.is_padding(idx));
        // Only mark as removed from the queue in release mode, if configured so.
        // MutablePriorityQueue.hpp:376-380
        // C++: if (ResetIndexWhenRemoved) m_index_setter(m_heap[idx], std::numeric_limits<size_t>::max());
        if self.reset_on_remove {
            (self.index_setter)(&self.heap[idx], INVALID_QUEUE_ID);
        }
        // MutablePriorityQueue.hpp:382-385
        // C++: if (idx + 1 == m_heap.size()) { this->pop_back(); return; }
        if idx + 1 == self.heap.len() {
            self.pop_back();
            return;
        }
        // MutablePriorityQueue.hpp:386-388
        // C++: m_heap[idx] = m_heap.back();
        // C++: m_index_setter(m_heap[idx], idx);
        // C++: this->pop_back();
        self.heap[idx] = self.heap[self.heap.len() - 1];
        (self.index_setter)(&self.heap[idx], idx);
        self.pop_back();
        // MutablePriorityQueue.hpp:389-390
        // C++: update_heap_down(idx, m_heap.size() - 1);
        // C++: update_heap_up(1, idx);
        self.update_heap_down(idx, self.heap.len() - 1);
        self.update_heap_up(1, idx);
    }

    /// Re-sort element at `idx` after its priority changed (remove + re-insert).
    /// MutablePriorityQueue.hpp:277
    /// C++: void update(size_t idx) { assert(! address::is_padding(idx)); T item = m_heap[idx]; remove(idx); push(item); }
    pub fn update(&mut self, idx: usize) {
        debug_assert!(!self.address.is_padding(idx));
        let item = self.heap[idx];
        self.remove(idx);
        self.push(item);
    }

    /// Number of elements in the queue.
    /// There is one padding element stored at each miniheap, thus lower the number of elements by the number of miniheaps.
    /// MutablePriorityQueue.hpp:279
    /// C++: size_t size() const noexcept { return m_heap.size() - (m_heap.size() + address::block_size - 1) / address::block_size; }
    #[inline]
    pub fn size(&self) -> usize {
        self.heap.len() - (self.heap.len() + self.address.block_size - 1) / self.address.block_size
    }

    /// Whether the queue is empty.
    /// MutablePriorityQueue.hpp:280
    /// C++: bool empty() const { return m_heap.empty(); }
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.heap.is_empty()
    }

    /// Indexed access (mirrors C++ `operator[]`).
    /// MutablePriorityQueue.hpp:281
    /// C++: T& operator[](std::size_t idx) noexcept { assert(! address::is_padding(idx)); return m_heap[idx]; }
    #[inline]
    pub fn get(&self, idx: usize) -> Option<&T> {
        debug_assert!(!self.address.is_padding(idx));
        self.heap.get(idx)
    }

    /// Mutable indexed access (mirrors C++ `operator[]`).
    /// MutablePriorityQueue.hpp:281
    /// C++: T& operator[](std::size_t idx) noexcept { assert(! address::is_padding(idx)); return m_heap[idx]; }
    #[inline]
    pub fn get_mut(&mut self, idx: usize) -> Option<&mut T> {
        debug_assert!(!self.address.is_padding(idx));
        self.heap.get_mut(idx)
    }

    /// Pop the trailing element, plus a trailing padding element if present.
    /// MutablePriorityQueue.hpp:287-293
    /// C++: void pop_back() noexcept
    fn pop_back(&mut self) {
        // C++: assert(m_heap.size() > 1);
        // C++: assert(! address::is_padding(m_heap.size() - 1));
        debug_assert!(self.heap.len() > 1);
        debug_assert!(!self.address.is_padding(self.heap.len() - 1));
        // C++: m_heap.pop_back();
        self.heap.pop();
        // C++: if (address::is_padding(m_heap.size() - 1)) m_heap.pop_back();
        if self.address.is_padding(self.heap.len() - 1) {
            self.heap.pop();
        }
    }

    /// Bubble element up the skip-heap.
    /// MutablePriorityQueue.hpp:393-417
    /// C++: void update_heap_up(size_t top, size_t bottom)
    fn update_heap_up(&mut self, top: usize, bottom: usize) {
        // MutablePriorityQueue.hpp:396-397
        // C++: assert(! address::is_padding(top));
        // C++: assert(! address::is_padding(bottom));
        debug_assert!(!self.address.is_padding(top));
        debug_assert!(!self.address.is_padding(bottom));
        // MutablePriorityQueue.hpp:398-399
        // C++: size_t childIdx = bottom;
        // C++: T *child = &m_heap[childIdx];
        let mut child_idx = bottom;
        let mut child = self.heap[child_idx];
        // MutablePriorityQueue.hpp:400
        // C++: for (;;) {
        loop {
            // MutablePriorityQueue.hpp:401
            // C++: size_t parentIdx = address::parent_of(childIdx);
            let parent_idx = self.address.parent_of(child_idx);
            // MutablePriorityQueue.hpp:402-403
            // C++: if (childIdx == 1 || parentIdx < top) break;
            if child_idx == 1 || parent_idx < top {
                break;
            }
            // MutablePriorityQueue.hpp:404
            // C++: T *parent = &m_heap[parentIdx];
            let parent = self.heap[parent_idx];
            // switch nodes
            // MutablePriorityQueue.hpp:405-412
            // C++: if (! m_less_predicate(*parent, *child)) {
            if !(self.less_predicate)(&parent, &child) {
                // C++: T tmp = *parent;
                // C++: m_index_setter(tmp,    childIdx);
                // C++: m_index_setter(*child, parentIdx);
                // C++: m_heap[parentIdx] = *child;
                // C++: m_heap[childIdx]  = tmp;
                let tmp = parent;
                (self.index_setter)(&tmp, child_idx);
                (self.index_setter)(&child, parent_idx);
                self.heap[parent_idx] = child;
                self.heap[child_idx] = tmp;
            }
            // shift up
            // MutablePriorityQueue.hpp:413-415
            // C++: childIdx = parentIdx;
            // C++: child = parent;
            child_idx = parent_idx;
            child = self.heap[parent_idx];
        }
    }

    /// Bubble element down the skip-heap.
    /// MutablePriorityQueue.hpp:419-451
    /// C++: void update_heap_down(size_t top, size_t bottom)
    fn update_heap_down(&mut self, top: usize, bottom: usize) {
        // MutablePriorityQueue.hpp:422-423
        // C++: assert(! address::is_padding(top));
        // C++: assert(! address::is_padding(bottom));
        debug_assert!(!self.address.is_padding(top));
        debug_assert!(!self.address.is_padding(bottom));
        // MutablePriorityQueue.hpp:424-425
        // C++: size_t parentIdx = top;
        // C++: T *parent = &m_heap[parentIdx];
        let mut parent_idx = top;
        let mut parent = self.heap[parent_idx];
        // MutablePriorityQueue.hpp:426
        // C++: for (;;) {
        loop {
            // MutablePriorityQueue.hpp:427
            // C++: size_t childIdx = address::child_of(parentIdx);
            let mut child_idx = self.address.child_of(parent_idx);
            // MutablePriorityQueue.hpp:428-429
            // C++: if (childIdx > bottom) break;
            if child_idx > bottom {
                break;
            }
            // MutablePriorityQueue.hpp:430
            // C++: T *child = &m_heap[childIdx];
            let mut child = self.heap[child_idx];
            // MutablePriorityQueue.hpp:431
            // C++: size_t child2Idx = childIdx + (address::is_block_leaf(parentIdx) ? address::block_size : 1);
            let child2_idx = child_idx
                + if self.address.is_block_leaf(parent_idx) {
                    self.address.block_size
                } else {
                    1
                };
            // MutablePriorityQueue.hpp:432-438
            // C++: if (child2Idx <= bottom) {
            if child2_idx <= bottom {
                // C++: T *child2 = &m_heap[child2Idx];
                let child2 = self.heap[child2_idx];
                // C++: if (! m_less_predicate(*child, *child2)) {
                if !(self.less_predicate)(&child, &child2) {
                    // C++: child = child2;
                    // C++: childIdx = child2Idx;
                    child = child2;
                    child_idx = child2_idx;
                }
            }
            // MutablePriorityQueue.hpp:439-440
            // C++: if (m_less_predicate(*parent, *child)) return;
            if (self.less_predicate)(&parent, &child) {
                return;
            }
            // switch nodes
            // MutablePriorityQueue.hpp:441-446
            // C++: T tmp = *parent;
            // C++: m_index_setter(tmp,    childIdx);
            // C++: m_index_setter(*child, parentIdx);
            // C++: m_heap[parentIdx] = *child;
            // C++: m_heap[childIdx]  = tmp;
            let tmp = parent;
            (self.index_setter)(&tmp, child_idx);
            (self.index_setter)(&child, parent_idx);
            self.heap[parent_idx] = child;
            self.heap[child_idx] = tmp;
            // shift down
            // MutablePriorityQueue.hpp:447-449
            // C++: parentIdx = childIdx;
            // C++: parent = child;
            parent_idx = child_idx;
            parent = self.heap[child_idx];
        }
    }
}

/// Construct a [`MutableSkipHeapPriorityQueue`].
/// MutablePriorityQueue.hpp:301-307
/// C++:
/// ```text
/// template<typename T, std::size_t BlockSize, const bool ResetIndexWhenRemoved, typename IndexSetter, typename LessPredicate>
/// MutableSkipHeapPriorityQueue<T, IndexSetter, LessPredicate, BlockSize, ResetIndexWhenRemoved>
///     make_miniheap_mutable_priority_queue(IndexSetter &&index_setter, LessPredicate &&less_predicate)
/// {
///     return MutableSkipHeapPriorityQueue<T, IndexSetter, LessPredicate, BlockSize, ResetIndexWhenRemoved>(
///         std::forward<IndexSetter>(index_setter), std::forward<LessPredicate>(less_predicate));
/// }
/// ```
///
/// In C++ `BlockSize` and `ResetIndexWhenRemoved` are compile-time template
/// parameters; here they are runtime arguments selecting the same behaviour.
pub fn make_miniheap_mutable_priority_queue<T, F, L>(
    block_size: usize,
    reset_index_when_removed: bool,
    index_setter: F,
    less_predicate: L,
) -> MutableSkipHeapPriorityQueue<T, F, L>
where
    T: Copy + Default,
    F: FnMut(&T, usize),
    L: Fn(&T, &T) -> bool,
{
    MutableSkipHeapPriorityQueue::new(block_size, reset_index_when_removed, index_setter, less_predicate)
}

impl<T, F, L> Drop for MutableSkipHeapPriorityQueue<T, F, L>
where
    T: Copy + Default,
    F: FnMut(&T, usize),
    L: Fn(&T, &T) -> bool,
{
    /// Destructor - clear queue.
    /// MutablePriorityQueue.hpp:267
    /// C++: ~MutableSkipHeapPriorityQueue() { clear(); }
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
