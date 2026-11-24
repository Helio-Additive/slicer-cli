//! Port of `src/glu-libtess/src/priorityq.c` (the sorted-array priority queue
//! in front of the heap: the initial vertices are sorted once with the C's
//! randomized quicksort, later insertions go to the heap). Keys are vertex
//! indices, `NIL` = NULL; `keys[i]` plays the C's `pq->keys[i]` and `order`
//! holds indices into `keys` (the C's `PQkey **order`).

use super::mesh::NIL;
use super::priorityq_heap::{PriorityQHeap, INIT_SIZE};

#[derive(Clone, Debug)]
pub struct PriorityQ {
    pub heap: PriorityQHeap,
    pub keys: Vec<usize>,
    pub order: Vec<usize>,
    pub size: i64,
    pub max: i64,
    pub initialized: bool,
}

impl PriorityQ {
    /// `pqNewPriorityQ` (priorityq.c:47-71).
    pub fn pq_new_priority_q() -> PriorityQ {
        PriorityQ {
            heap: PriorityQHeap::pq_new_priority_q(),
            keys: vec![NIL; INIT_SIZE as usize],
            order: Vec::new(),
            size: 0,
            max: INIT_SIZE,
            initialized: false,
        }
    }

    /// `pqInit` (priorityq.c:89-162): sort the indirect pointers in
    /// descending order with the C's randomized quicksort (seed 2016473283,
    /// `seed = seed * 1539415821 + 1` on an unsigned long), small lists by
    /// insertion sort.
    pub fn pq_init(&mut self, leq: &impl Fn(usize, usize) -> bool) -> bool {
        let lt = |order: &Vec<usize>, keys: &Vec<usize>, x: usize, piv: usize| -> bool { !leq(keys[piv], keys[order[x]]) };
        let gt = |order: &Vec<usize>, keys: &Vec<usize>, x: usize, piv: usize| -> bool { !leq(keys[order[x]], keys[piv]) };
        let mut seed: u64 = 2016473283;
        // (pq->size + 1) entries (the IBM malloc(0) patch).
        self.order = vec![0usize; (self.size + 1) as usize];
        let mut p: i64 = 0;
        let mut r: i64 = self.size - 1;
        {
            let mut i = p;
            let mut piv = 0usize;
            while i <= r {
                self.order[i as usize] = piv;
                piv += 1;
                i += 1;
            }
        }
        // Sort the indirect pointers in descending order, using randomized Quicksort
        let mut stack: Vec<(i64, i64)> = Vec::with_capacity(50);
        stack.push((p, r));
        while let Some((sp, sr)) = stack.pop() {
            p = sp;
            r = sr;
            while r > p + 10 {
                seed = seed.wrapping_mul(1539415821).wrapping_add(1);
                let mut i = p + (seed % ((r - p + 1) as u64)) as i64;
                let piv = self.order[i as usize];
                self.order[i as usize] = self.order[p as usize];
                self.order[p as usize] = piv;
                i = p - 1;
                let mut j = r + 1;
                loop {
                    loop {
                        i += 1;
                        if !gt(&self.order, &self.keys, i as usize, piv) {
                            break;
                        }
                    }
                    loop {
                        j -= 1;
                        if !lt(&self.order, &self.keys, j as usize, piv) {
                            break;
                        }
                    }
                    self.order.swap(i as usize, j as usize);
                    if !(i < j) {
                        break;
                    }
                }
                self.order.swap(i as usize, j as usize); // Undo last swap
                if i - p < r - j {
                    stack.push((j + 1, r));
                    r = i - 1;
                } else {
                    stack.push((p, i - 1));
                    p = j + 1;
                }
            }
            // Insertion sort small lists
            let mut i = p + 1;
            while i <= r {
                let piv = self.order[i as usize];
                let mut j = i;
                while j > p && lt(&self.order, &self.keys, (j - 1) as usize, piv) {
                    self.order[j as usize] = self.order[(j - 1) as usize];
                    j -= 1;
                }
                self.order[j as usize] = piv;
                i += 1;
            }
        }
        self.max = self.size;
        self.initialized = true;
        self.heap.pq_init(leq); // always succeeds
        true
    }

    /// `pqInsert` (priorityq.c:166-194).
    pub fn pq_insert(&mut self, key_new: usize, leq: &impl Fn(usize, usize) -> bool) -> i64 {
        if self.initialized {
            return self.heap.pq_insert(key_new, leq);
        }
        let curr = self.size;
        self.size += 1;
        if self.size >= self.max {
            // If the heap overflows, double its size.
            self.max <<= 1;
            self.keys.resize(self.max as usize, NIL);
        }
        self.keys[curr as usize] = key_new;
        // Negative handles index the sorted array.
        -(curr + 1)
    }

    #[inline]
    fn order_key(&self, i: i64) -> usize {
        self.keys[self.order[i as usize]]
    }

    /// `pqExtractMin` (priorityq.c:197-218).
    pub fn pq_extract_min(&mut self, leq: &impl Fn(usize, usize) -> bool) -> usize {
        if self.size == 0 {
            return self.heap.pq_extract_min(leq);
        }
        let sort_min = self.order_key(self.size - 1);
        if !self.heap.pq_is_empty() {
            let heap_min = self.heap.pq_minimum();
            if leq(heap_min, sort_min) {
                return self.heap.pq_extract_min(leq);
            }
        }
        loop {
            self.size -= 1;
            if !(self.size > 0 && self.order_key(self.size - 1) == NIL) {
                break;
            }
        }
        sort_min
    }

    /// `pqMinimum` (priorityq.c:221-237).
    pub fn pq_minimum(&self, leq: &impl Fn(usize, usize) -> bool) -> usize {
        if self.size == 0 {
            return self.heap.pq_minimum();
        }
        let sort_min = self.order_key(self.size - 1);
        if !self.heap.pq_is_empty() {
            let heap_min = self.heap.pq_minimum();
            if leq(heap_min, sort_min) {
                return heap_min;
            }
        }
        sort_min
    }

    /// `pqIsEmpty` (priorityq.c:240-244).
    pub fn pq_is_empty(&self) -> bool {
        (self.size == 0) && self.heap.pq_is_empty()
    }

    /// `pqDelete` (priorityq.c:247-261).
    pub fn pq_delete(&mut self, curr: i64, leq: &impl Fn(usize, usize) -> bool) {
        if curr >= 0 {
            self.heap.pq_delete(curr, leq);
            return;
        }
        let curr = -(curr + 1);
        self.keys[curr as usize] = NIL;
        while self.size > 0 && self.order_key(self.size - 1) == NIL {
            self.size -= 1;
        }
    }
}
