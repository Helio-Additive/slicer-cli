//! The comparator-based `std::sort` and its heap fallback used by Bambu's
//! polygon sweeps. This ports the LLVM 17 and later libc++ introsort.
#![forbid(unsafe_code)]

// ---------------------------------------------------------------------------
// `std::sort` as libc++ implements it (`<__algorithm/sort.h>`, the `__introsort`
// of LLVM 17 and later). ClipperLib sorts the local minima by Y and the
// intersections of a scanbeam by Y with `std::sort`, which is not stable: the
// order of elements with equal keys is the algorithm's, and the sweep takes them
// in that order. Up to 23 elements libc++ sorts by insertion and keeps equal
// elements in place; from 24 on it partitions and does not. The functions below
// follow the header one for one, for a type that is not arithmetic or a
// comparator that is not `std::less` (so: no branchless networks, no bitset
// partition). Iterators are indices into the slice; `__first - 1` is `first - 1`.
// ---------------------------------------------------------------------------

/// sort.h `__sort3` (the arm without `__use_branchless_sort`): stable, 0-2 swaps.
fn libcxx_sort3<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], x: usize, y: usize, z: usize, c: &mut F) -> bool {
    if !c(&v[y], &v[x]) {
        // if x <= y
        if !c(&v[z], &v[y]) {
            // if y <= z
            return false; // x <= y && y <= z
        }
        v.swap(y, z); // x <= z && y < z
        if c(&v[y], &v[x]) {
            // if x > y
            v.swap(x, y); // x < y && y <= z
        }
        return true; // x <= y && y < z
    }
    if c(&v[z], &v[y]) {
        // x > y, if y > z
        v.swap(x, z); // x < y && y < z
        return true;
    }
    v.swap(x, y); // x > y && y <= z
    if c(&v[z], &v[y]) {
        // if y > z
        v.swap(y, z); // x <= y && y < z
    }
    true
}

/// sort.h `__sort4` (the arm without `__use_branchless_sort`).
fn libcxx_sort4<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], x1: usize, x2: usize, x3: usize, x4: usize, c: &mut F) {
    libcxx_sort3(v, x1, x2, x3, c);
    if c(&v[x4], &v[x3]) {
        v.swap(x3, x4);
        if c(&v[x3], &v[x2]) {
            v.swap(x2, x3);
            if c(&v[x2], &v[x1]) {
                v.swap(x1, x2);
            }
        }
    }
}

/// sort.h `__sort5` (the arm without `__use_branchless_sort`).
fn libcxx_sort5<T: Copy, F: FnMut(&T, &T) -> bool>(
    v: &mut [T],
    x1: usize,
    x2: usize,
    x3: usize,
    x4: usize,
    x5: usize,
    c: &mut F,
) {
    libcxx_sort4(v, x1, x2, x3, x4, c);
    if c(&v[x5], &v[x4]) {
        v.swap(x4, x5);
        if c(&v[x4], &v[x3]) {
            v.swap(x3, x4);
            if c(&v[x3], &v[x2]) {
                v.swap(x2, x3);
                if c(&v[x2], &v[x1]) {
                    v.swap(x1, x2);
                }
            }
        }
    }
}

/// sort.h `__insertion_sort`.
fn libcxx_insertion_sort<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], first: usize, last: usize, c: &mut F) {
    if first == last {
        return;
    }
    let mut i = first + 1;
    while i != last {
        let mut j = i - 1;
        if c(&v[i], &v[j]) {
            let t = v[i];
            let mut k = j;
            j = i;
            // do { *j = *k; j = k; } while (j != first && comp(t, *--k));
            loop {
                v[j] = v[k];
                j = k;
                if j == first {
                    break;
                }
                k -= 1;
                if !c(&t, &v[k]) {
                    break;
                }
            }
            v[j] = t;
        }
        i += 1;
    }
}

/// sort.h `__insertion_sort_unguarded`: assumes an element at `first - 1` that is not
/// greater than any element of the range, so the inner loop has no bounds check.
fn libcxx_insertion_sort_unguarded<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], first: usize, last: usize, c: &mut F) {
    if first == last {
        return;
    }
    let mut i = first + 1;
    while i != last {
        let mut j = i - 1;
        if c(&v[i], &v[j]) {
            let t = v[i];
            let mut k = j;
            j = i;
            // do { *j = *k; j = k; } while (comp(t, *--k));
            loop {
                v[j] = v[k];
                j = k;
                k -= 1;
                if !c(&t, &v[k]) {
                    break;
                }
            }
            v[j] = t;
        }
        i += 1;
    }
}

/// sort.h `__insertion_sort_incomplete`: gives up after 8 insertions and tells whether
/// the range ended up sorted.
fn libcxx_insertion_sort_incomplete<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], first: usize, last: usize, c: &mut F) -> bool {
    match last - first {
        0 | 1 => return true,
        2 => {
            if c(&v[last - 1], &v[first]) {
                v.swap(first, last - 1);
            }
            return true;
        }
        3 => {
            libcxx_sort3(v, first, first + 1, last - 1, c);
            return true;
        }
        4 => {
            libcxx_sort4(v, first, first + 1, first + 2, last - 1, c);
            return true;
        }
        5 => {
            libcxx_sort5(v, first, first + 1, first + 2, first + 3, last - 1, c);
            return true;
        }
        _ => {}
    }
    let mut j = first + 2;
    libcxx_sort3(v, first, first + 1, j, c);
    const LIMIT: u32 = 8;
    let mut count: u32 = 0;
    let mut i = j + 1;
    while i != last {
        if c(&v[i], &v[j]) {
            let t = v[i];
            let mut k = j;
            j = i;
            loop {
                v[j] = v[k];
                j = k;
                if j == first {
                    break;
                }
                k -= 1;
                if !c(&t, &v[k]) {
                    break;
                }
            }
            v[j] = t;
            count += 1;
            if count == LIMIT {
                i += 1;
                return i == last;
            }
        }
        j = i;
        i += 1;
    }
    true
}

/// sort.h `__partition_with_equals_on_right`: `v[first]` is the pivot; elements
/// equivalent to it stay to its right. Returns the pivot's position and whether the
/// range was already partitioned.
fn libcxx_partition_with_equals_on_right<T: Copy, F: FnMut(&T, &T) -> bool>(
    v: &mut [T],
    mut first: usize,
    mut last: usize,
    c: &mut F,
) -> (usize, bool) {
    let begin = first;
    let pivot = v[first];
    // Find the first element greater or equal to the pivot.
    loop {
        first += 1;
        if !c(&v[first], &pivot) {
            break;
        }
    }
    // Find the last element less than the pivot.
    if begin == first - 1 {
        // while (first < last && !comp(*--last, pivot));
        while first < last && {
            last -= 1;
            !c(&v[last], &pivot)
        } {}
    } else {
        loop {
            last -= 1;
            if c(&v[last], &pivot) {
                break;
            }
        }
    }
    let already_partitioned = first >= last;
    while first < last {
        v.swap(first, last);
        loop {
            first += 1;
            if !c(&v[first], &pivot) {
                break;
            }
        }
        loop {
            last -= 1;
            if c(&v[last], &pivot) {
                break;
            }
        }
    }
    // Move the pivot to its correct position.
    let pivot_pos = first - 1;
    if begin != pivot_pos {
        v[begin] = v[pivot_pos];
    }
    v[pivot_pos] = pivot;
    (pivot_pos, already_partitioned)
}

/// sort.h `__partition_with_equals_on_left`: elements equivalent to the pivot go to
/// its left. Returns the position after the pivot.
fn libcxx_partition_with_equals_on_left<T: Copy, F: FnMut(&T, &T) -> bool>(
    v: &mut [T],
    mut first: usize,
    mut last: usize,
    c: &mut F,
) -> usize {
    let begin = first;
    let pivot = v[first];
    if c(&pivot, &v[last - 1]) {
        loop {
            first += 1;
            if c(&pivot, &v[first]) {
                break;
            }
        }
    } else {
        // while (++first < last && !comp(pivot, *first)) {}
        loop {
            first += 1;
            if !(first < last && !c(&pivot, &v[first])) {
                break;
            }
        }
    }

    if first < last {
        loop {
            last -= 1;
            if !c(&pivot, &v[last]) {
                break;
            }
        }
    }
    while first < last {
        v.swap(first, last);
        loop {
            first += 1;
            if c(&pivot, &v[first]) {
                break;
            }
        }
        loop {
            last -= 1;
            if !c(&pivot, &v[last]) {
                break;
            }
        }
    }
    let pivot_pos = first - 1;
    if begin != pivot_pos {
        v[begin] = v[pivot_pos];
    }
    v[pivot_pos] = pivot;
    first
}

/// sift_down.h `__sift_down`, on the heap `s[..len]`.
fn libcxx_sift_down<T: Copy, F: FnMut(&T, &T) -> bool>(s: &mut [T], c: &mut F, len: usize, mut start: usize) {
    // left-child of start is at 2 * start + 1, right-child at 2 * start + 2
    let mut child = start;
    if len < 2 || (len - 2) / 2 < child {
        return;
    }
    child = 2 * child + 1;
    if child + 1 < len && c(&s[child], &s[child + 1]) {
        // right-child exists and is greater than left-child
        child += 1;
    }
    // check if we are in heap-order
    if c(&s[child], &s[start]) {
        // we are, start is larger than its largest child
        return;
    }
    let top = s[start];
    loop {
        // we are not in heap-order, swap the parent with its largest child
        s[start] = s[child];
        start = child;
        if (len - 2) / 2 < child {
            break;
        }
        // recompute the child based off of the updated parent
        child = 2 * child + 1;
        if child + 1 < len && c(&s[child], &s[child + 1]) {
            child += 1;
        }
        // check if we are in heap-order
        if c(&s[child], &top) {
            break;
        }
    }
    s[start] = top;
}

/// sift_down.h `__floyd_sift_down`, on the heap `s[..len]`; returns the hole.
fn libcxx_floyd_sift_down<T: Copy, F: FnMut(&T, &T) -> bool>(s: &mut [T], c: &mut F, len: usize) -> usize {
    let mut hole = 0usize;
    let mut child = 0usize;
    loop {
        // child_i += child + 1; child = 2 * child + 1: the index of the left child.
        child = 2 * child + 1;
        if child + 1 < len && c(&s[child], &s[child + 1]) {
            // right-child exists and is greater than left-child
            child += 1;
        }
        // swap hole with its largest child
        s[hole] = s[child];
        hole = child;
        // if hole is now a leaf, we're done
        if child > (len - 2) / 2 {
            return hole;
        }
    }
}

/// push_heap.h `__sift_up`, on `s[..last]` with `len == last`.
fn libcxx_sift_up<T: Copy, F: FnMut(&T, &T) -> bool>(s: &mut [T], mut last: usize, c: &mut F, mut len: usize) {
    if len > 1 {
        len = (len - 2) / 2;
        let mut ptr = len;
        last -= 1;
        if c(&s[ptr], &s[last]) {
            let t = s[last];
            loop {
                s[last] = s[ptr];
                last = ptr;
                if len == 0 {
                    break;
                }
                len = (len - 1) / 2;
                ptr = len;
                if !c(&s[ptr], &t) {
                    break;
                }
            }
            s[last] = t;
        }
    }
}

/// pop_heap.h `__pop_heap`, on the heap `s[..last]` with `len == last`.
fn libcxx_pop_heap<T: Copy, F: FnMut(&T, &T) -> bool>(s: &mut [T], mut last: usize, c: &mut F, len: usize) {
    if len > 1 {
        let top = s[0]; // create a hole at first
        let mut hole = libcxx_floyd_sift_down(s, c, len);
        last -= 1;
        if hole == last {
            s[hole] = top;
        } else {
            s[hole] = s[last];
            hole += 1;
            s[last] = top;
            libcxx_sift_up(s, hole, c, hole);
        }
    }
}

/// partial_sort.h `__partial_sort_impl(first, last, last)`, which is what `__introsort`
/// falls back to at depth 0: `__make_heap` (make_heap.h) then `__sort_heap` (sort_heap.h).
fn libcxx_heap_sort<T: Copy, F: FnMut(&T, &T) -> bool>(s: &mut [T], c: &mut F) {
    if s.is_empty() {
        return;
    }
    // __make_heap
    let n = s.len();
    if n > 1 {
        // start from the first parent, there is no need to consider children
        let mut start = (n - 2) / 2;
        loop {
            libcxx_sift_down(s, c, n, start);
            if start == 0 {
                break;
            }
            start -= 1;
        }
    }
    // for (i = middle; i != last; ++i): middle is last, nothing to do.
    // __sort_heap
    let mut last = n;
    let mut n = n;
    while n > 1 {
        libcxx_pop_heap(s, last, c, n);
        last -= 1;
        n -= 1;
    }
}

/// sort.h `__introsort` with `_UseBitSetPartition == false`.
fn libcxx_introsort<T: Copy, F: FnMut(&T, &T) -> bool>(
    v: &mut [T],
    mut first: usize,
    mut last: usize,
    c: &mut F,
    mut depth: isize,
    mut leftmost: bool,
) {
    // Upper bound for using insertion sort for sorting.
    const LIMIT: usize = 24;
    // Lower bound for using Tuckey's ninther technique for median computation.
    const NINTHER_THRESHOLD: usize = 128;
    loop {
        let len = last - first;
        match len {
            0 | 1 => return,
            2 => {
                if c(&v[last - 1], &v[first]) {
                    v.swap(first, last - 1);
                }
                return;
            }
            3 => {
                libcxx_sort3(v, first, first + 1, last - 1, c);
                return;
            }
            4 => {
                libcxx_sort4(v, first, first + 1, first + 2, last - 1, c);
                return;
            }
            5 => {
                libcxx_sort5(v, first, first + 1, first + 2, first + 3, last - 1, c);
                return;
            }
            _ => {}
        }
        // Use insertion sort if the length of the range is below the specified limit.
        if len < LIMIT {
            if leftmost {
                libcxx_insertion_sort(v, first, last, c);
            } else {
                libcxx_insertion_sort_unguarded(v, first, last, c);
            }
            return;
        }
        if depth == 0 {
            // Fallback to heap sort as Introsort suggests.
            libcxx_heap_sort(&mut v[first..last], c);
            return;
        }
        depth -= 1;
        {
            let half_len = len / 2;
            // Use Tuckey's ninther technique or median of 3 for pivot selection
            // depending on the length of the range being sorted.
            if len > NINTHER_THRESHOLD {
                libcxx_sort3(v, first, first + half_len, last - 1, c);
                libcxx_sort3(v, first + 1, first + (half_len - 1), last - 2, c);
                libcxx_sort3(v, first + 2, first + (half_len + 1), last - 3, c);
                libcxx_sort3(v, first + (half_len - 1), first + half_len, first + (half_len + 1), c);
                v.swap(first, first + half_len);
            } else {
                libcxx_sort3(v, first + half_len, first, last - 1, c);
            }
        }
        // The elements to the left of the current range are already sorted. If the
        // range is not the leftmost one and the pivot is the same as the highest
        // element of the range to the left, all the elements in [first, pivot] are
        // equal to the pivot, and the left side needs no sorting.
        if !leftmost && !c(&v[first - 1], &v[first]) {
            first = libcxx_partition_with_equals_on_left(v, first, last, c);
            continue;
        }
        let (mut i, already_partitioned) = libcxx_partition_with_equals_on_right(v, first, last, c);
        // [first, i) < *i and *i <= [i+1, last)
        // If we were given a perfect partition, see if insertion sort is quick...
        if already_partitioned {
            let fs = libcxx_insertion_sort_incomplete(v, first, i, c);
            if libcxx_insertion_sort_incomplete(v, i + 1, last, c) {
                if fs {
                    return;
                }
                last = i;
                continue;
            } else if fs {
                i += 1;
                first = i;
                continue;
            }
        }
        // Sort the left partition recursively and the right partition with tail recursion elimination.
        libcxx_introsort(v, first, i, c, depth, leftmost);
        leftmost = false;
        i += 1;
        first = i;
    }
}

/// `std::sort(first, last, comp)` of libc++: sort.h `__sort_impl` -> `__sort_dispatch`,
/// with the depth limit `2 * __log2i(last - first)`. `comp(a, b)` is "a goes before b".
pub fn libcxx_sort<T: Copy, F: FnMut(&T, &T) -> bool>(v: &mut [T], mut comp: F) {
    let n = v.len();
    // __log2i: 0 for 0, else the index of the highest set bit.
    let log2i = if n == 0 { 0 } else { (usize::BITS - 1 - n.leading_zeros()) as isize };
    let depth_limit = 2 * log2i;
    libcxx_introsort(v, 0, n, &mut comp, depth_limit, true);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn libcxx_sort_keeps_equal_elements_in_place_below_24_only() {
        // Up to 23 elements libc++ sorts by insertion: stable.
        for n in 0..24usize {
            let mut v: Vec<(i32, usize)> = (0..n).map(|i| (((i * 7) % 3) as i32, i)).collect();
            let mut want = v.clone();
            want.sort_by_key(|e| e.0);
            libcxx_sort(&mut v, |a, b| a.0 < b.0);
            assert_eq!(v, want, "n = {n}");
        }
        // From 24 on it partitions. The order std::sort of libc++ (LLVM 17 and later)
        // gives the ids of 40 elements with the keys (i * 7) % 3:
        let mut v: Vec<(i32, i32)> = (0..40).map(|i| ((i * 7) % 3, i)).collect();
        libcxx_sort(&mut v, |a, b| a.0 < b.0);
        let ids: Vec<i32> = v.iter().map(|e| e.1).collect();
        assert_eq!(
            ids,
            [
                39, 3, 27, 9, 12, 15, 18, 0, 21, 24, 30, 33, 6, 36, 13, 16, 19, 22, 25, 37, 10, 7, 34, 4, 31, 1, 28, 8,
                26, 29, 23, 17, 32, 14, 11, 35, 5, 2, 38, 20
            ]
        );
        // And of 150 (past the ninther threshold), the first and the last ten.
        let mut v: Vec<(i32, i32)> = (0..150).map(|i| ((i * 7) % 3, i)).collect();
        libcxx_sort(&mut v, |a, b| a.0 < b.0);
        let ids: Vec<i32> = v.iter().map(|e| e.1).collect();
        assert_eq!(ids[..10], [51, 144, 147, 3, 141, 138, 6, 135, 132, 9]);
        assert_eq!(ids[140..], [140, 59, 47, 143, 56, 53, 146, 2, 74, 101]);
        assert!(v.windows(2).all(|w| w[0].0 <= w[1].0));
    }

    #[test]
    fn libcxx_sort_and_its_heap_sort_order_any_input() {
        let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
        let mut xs = move || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            s
        };
        for round in 0..400 {
            let n = (xs() % [7, 30, 200, 1500][round % 4]) as usize;
            let keys = 1 + xs() % [1, 3, 50, 100_000][(round / 4) % 4];
            let v: Vec<(u64, usize)> = (0..n).map(|i| (xs() % keys, i)).collect();
            let mut want = v.clone();
            want.sort();
            for descending in [false, true] {
                let mut a = v.clone();
                libcxx_sort(&mut a, |x, y| if descending { y.0 < x.0 } else { x.0 < y.0 });
                assert!(a.windows(2).all(|w| if descending { w[0].0 >= w[1].0 } else { w[0].0 <= w[1].0 }));
                a.sort();
                assert_eq!(a, want);
            }
            let mut b = v.clone();
            libcxx_heap_sort(&mut b, &mut |x: &(u64, usize), y: &(u64, usize)| x.0 < y.0);
            assert!(b.windows(2).all(|w| w[0].0 <= w[1].0));
            b.sort();
            assert_eq!(b, want);
        }
    }
}
