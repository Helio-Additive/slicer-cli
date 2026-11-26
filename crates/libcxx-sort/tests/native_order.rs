//! Compare the complete order of ties with libc++ on both sides of its cutoffs.
#[test]
fn comparable_records_match_libcxx() {
    for row in include_str!("data/orders.txt").lines() {
        let mut numbers = row.split_whitespace().map(|v| v.parse::<usize>().unwrap());
        let len = numbers.next().unwrap();
        let distinct = numbers.next().unwrap() as u32;
        let mut state = 17_u32;
        let mut items: Vec<_> = (0..len)
            .map(|i| {
                state = state.wrapping_mul(1664525).wrapping_add(1013904223);
                ((state >> 16) % distinct, i)
            })
            .collect();
        libcxx_sort::libcxx_sort(&mut items, |a, b| a.0 < b.0);
        assert!(
            items.iter().map(|p| p.1).eq(numbers),
            "length {len}, {distinct} distinct keys"
        );
    }
}
