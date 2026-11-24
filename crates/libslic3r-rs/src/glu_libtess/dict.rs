//! Port of `src/glu-libtess/src/dict.c` / `dict-list.h` (the sorted doubly
//! linked list the sweep keeps its active regions in). Nodes live in an arena;
//! node 0 is the head, whose key is `NIL` (the C's NULL). The ordering
//! predicate `leq(frame, key1, key2)` needs the tessellator, so the functions
//! that call it take it as a closure.

use super::mesh::NIL;

#[derive(Clone, Debug)]
pub struct DictNode {
    pub key: usize,
    pub next: usize,
    pub prev: usize,
}

#[derive(Clone, Debug)]
pub struct Dict {
    pub nodes: Vec<DictNode>,
}

pub const DICT_HEAD: usize = 0;

impl Dict {
    /// `dictNewDict` (dict.c:44-58).
    pub fn dict_new_dict() -> Dict {
        Dict { nodes: vec![DictNode { key: NIL, next: DICT_HEAD, prev: DICT_HEAD }] }
    }

    /// `dictInsertBefore` (dict.c:72-91). `leq(a, b)` is `(*dict->leq)(frame, a, b)`.
    pub fn dict_insert_before(&mut self, node: usize, key: usize, mut leq: impl FnMut(usize, usize) -> bool) -> usize {
        let mut node = node;
        loop {
            node = self.nodes[node].prev;
            if !(self.nodes[node].key != NIL && !leq(self.nodes[node].key, key)) {
                break;
            }
        }
        let new_node = self.nodes.len();
        let nn = self.nodes[node].next;
        self.nodes.push(DictNode { key, next: nn, prev: node });
        self.nodes[nn].prev = new_node;
        self.nodes[node].next = new_node;
        new_node
    }

    /// `dictInsert` = `dictInsertBefore(d, &d->head, k)` (dict-list.h).
    pub fn dict_insert(&mut self, key: usize, leq: impl FnMut(usize, usize) -> bool) -> usize {
        self.dict_insert_before(DICT_HEAD, key, leq)
    }

    /// `dictDelete` (dict.c:94-100).
    pub fn dict_delete(&mut self, node: usize) {
        let (next, prev) = (self.nodes[node].next, self.nodes[node].prev);
        self.nodes[next].prev = prev;
        self.nodes[prev].next = next;
    }

    /// `dictSearch` (dict.c:103-111). `leq_key(node_key)` is
    /// `(*dict->leq)(frame, key, node->key)` for the searched key.
    pub fn dict_search(&self, mut leq_key: impl FnMut(usize) -> bool) -> usize {
        let mut node = DICT_HEAD;
        loop {
            node = self.nodes[node].next;
            if !(self.nodes[node].key != NIL && !leq_key(self.nodes[node].key)) {
                break;
            }
        }
        node
    }

    /// `dictKey`.
    #[inline]
    pub fn dict_key(&self, n: usize) -> usize { self.nodes[n].key }
    /// `dictSucc`.
    #[inline]
    pub fn dict_succ(&self, n: usize) -> usize { self.nodes[n].next }
    /// `dictPred`.
    #[inline]
    pub fn dict_pred(&self, n: usize) -> usize { self.nodes[n].prev }
    /// `dictMin`.
    #[inline]
    pub fn dict_min(&self) -> usize { self.nodes[DICT_HEAD].next }
    /// `dictMax`.
    #[inline]
    pub fn dict_max(&self) -> usize { self.nodes[DICT_HEAD].prev }
}
