//! Port of `src/glu-libtess/src/priorityq-heap.c` (the binary heap behind the
//! sorted priority queue). Keys are vertex indices (`NIL` = NULL); `LEQ` is
//! `VertLeq` on the vertices (priorityq-heap.c:52-55), passed as a closure.

use super::mesh::NIL;

pub const INIT_SIZE: i64 = 32;

#[derive(Clone, Copy, Debug, Default)]
pub struct PQhandleElem {
    pub key: usize,
    pub node: i64,
}

#[derive(Clone, Debug)]
pub struct PriorityQHeap {
    pub nodes: Vec<i64>,
    pub handles: Vec<PQhandleElem>,
    pub size: i64,
    pub max: i64,
    pub free_list: i64,
    pub initialized: bool,
}

impl PriorityQHeap {
    /// `pqNewPriorityQ` (priorityq-heap.c:58-84).
    pub fn pq_new_priority_q() -> PriorityQHeap {
        let mut pq = PriorityQHeap {
            nodes: vec![0; (INIT_SIZE + 1) as usize],
            handles: vec![PQhandleElem { key: NIL, node: 0 }; (INIT_SIZE + 1) as usize],
            size: 0,
            max: INIT_SIZE,
            free_list: 0,
            initialized: false,
        };
        pq.nodes[1] = 1; // so that Minimum() returns NULL
        pq.handles[1].key = NIL;
        pq
    }

    /// `FloatDown` (priorityq-heap.c:95-122).
    fn float_down(&mut self, curr: i64, leq: &impl Fn(usize, usize) -> bool) {
        let mut curr = curr;
        let h_curr = self.nodes[curr as usize];
        loop {
            let mut child = curr << 1;
            if child < self.size
                && leq(self.handles[self.nodes[(child + 1) as usize] as usize].key, self.handles[self.nodes[child as usize] as usize].key)
            {
                child += 1;
            }
            let h_child = self.nodes[child as usize];
            if child > self.size || leq(self.handles[h_curr as usize].key, self.handles[h_child as usize].key) {
                self.nodes[curr as usize] = h_curr;
                self.handles[h_curr as usize].node = curr;
                break;
            }
            self.nodes[curr as usize] = h_child;
            self.handles[h_child as usize].node = curr;
            curr = child;
        }
    }

    /// `FloatUp` (priorityq-heap.c:125-145).
    fn float_up(&mut self, curr: i64, leq: &impl Fn(usize, usize) -> bool) {
        let mut curr = curr;
        let h_curr = self.nodes[curr as usize];
        loop {
            let parent = curr >> 1;
            let h_parent = self.nodes[parent as usize];
            if parent == 0 || leq(self.handles[h_parent as usize].key, self.handles[h_curr as usize].key) {
                self.nodes[curr as usize] = h_curr;
                self.handles[h_curr as usize].node = curr;
                break;
            }
            self.nodes[curr as usize] = h_parent;
            self.handles[h_parent as usize].node = curr;
            curr = parent;
        }
    }

    /// `pqInit` (priorityq-heap.c:148-158).
    pub fn pq_init(&mut self, leq: &impl Fn(usize, usize) -> bool) {
        // This method of building a heap is O(n), rather than O(n lg n).
        let mut i = self.size;
        while i >= 1 {
            self.float_down(i, leq);
            i -= 1;
        }
        self.initialized = true;
    }

    /// `pqInsert` (priorityq-heap.c:162-204).
    pub fn pq_insert(&mut self, key_new: usize, leq: &impl Fn(usize, usize) -> bool) -> i64 {
        self.size += 1;
        let curr = self.size;
        if (curr * 2) > self.max {
            // If the heap overflows, double its size.
            self.max <<= 1;
            self.nodes.resize((self.max + 1) as usize, 0);
            self.handles.resize((self.max + 1) as usize, PQhandleElem { key: NIL, node: 0 });
        }
        let free_handle;
        if self.free_list == 0 {
            free_handle = curr;
        } else {
            free_handle = self.free_list;
            self.free_list = self.handles[free_handle as usize].node;
        }
        self.nodes[curr as usize] = free_handle;
        self.handles[free_handle as usize].node = curr;
        self.handles[free_handle as usize].key = key_new;
        if self.initialized {
            self.float_up(curr, leq);
        }
        free_handle
    }

    /// `pqExtractMin` (priorityq-heap.c:207-226).
    pub fn pq_extract_min(&mut self, leq: &impl Fn(usize, usize) -> bool) -> usize {
        let h_min = self.nodes[1];
        let min = self.handles[h_min as usize].key;
        if self.size > 0 {
            self.nodes[1] = self.nodes[self.size as usize];
            let n1 = self.nodes[1];
            self.handles[n1 as usize].node = 1;

            self.handles[h_min as usize].key = NIL;
            self.handles[h_min as usize].node = self.free_list;
            self.free_list = h_min;

            self.size -= 1;
            if self.size > 0 {
                self.float_down(1, leq);
            }
        }
        min
    }

    /// `pqDelete` (priorityq-heap.c:229-254).
    pub fn pq_delete(&mut self, h_curr: i64, leq: &impl Fn(usize, usize) -> bool) {
        let curr = self.handles[h_curr as usize].node;
        self.nodes[curr as usize] = self.nodes[self.size as usize];
        let nc = self.nodes[curr as usize];
        self.handles[nc as usize].node = curr;

        self.size -= 1;
        if curr <= self.size {
            if curr <= 1 || leq(self.handles[self.nodes[(curr >> 1) as usize] as usize].key, self.handles[self.nodes[curr as usize] as usize].key) {
                self.float_down(curr, leq);
            } else {
                self.float_up(curr, leq);
            }
        }
        self.handles[h_curr as usize].key = NIL;
        self.handles[h_curr as usize].node = self.free_list;
        self.free_list = h_curr;
    }

    /// `__gl_pqHeapMinimum` (priorityq-heap.h:104).
    #[inline]
    pub fn pq_minimum(&self) -> usize {
        self.handles[self.nodes[1] as usize].key
    }

    /// `__gl_pqHeapIsEmpty` (priorityq-heap.h:105).
    #[inline]
    pub fn pq_is_empty(&self) -> bool {
        self.size == 0
    }
}
