// nanosvg.h 1157–1198. The native buffer holds 63 item bytes and a terminator.
// Its input pointer advances across the entire item even when the buffer fills.
pub(crate) fn parse_number(input: &[u8]) -> usize {
    let mut i = usize::from(matches!(input.first(), Some(b'+' | b'-')));
    while input.get(i).is_some_and(u8::is_ascii_digit) {
        i += 1;
    }
    if input.get(i) == Some(&b'.') {
        i += 1;
        while input.get(i).is_some_and(u8::is_ascii_digit) {
            i += 1;
        }
    }
    if matches!(input.get(i), Some(b'e' | b'E')) && !matches!(input.get(i + 1), Some(b'm' | b'x')) {
        i += 1;
        if matches!(input.get(i), Some(b'+' | b'-')) {
            i += 1;
        }
        while input.get(i).is_some_and(u8::is_ascii_digit) {
            i += 1;
        }
    }
    i
}

// nsvg__getNextPathItem, nanosvg.h 1213–1230.
// nanosvg.h 1090–1154. strtoll saturates each digit sequence separately.
fn digits(input: &mut &[u8]) -> (i64, usize) {
    let mut result = 0i64;
    let mut count = 0;
    while let Some((&b, tail)) = input.split_first() {
        if !b.is_ascii_digit() {
            break;
        }
        result = result
            .saturating_mul(10)
            .saturating_add(i64::from(b - b'0'));
        count += 1;
        *input = tail;
    }
    (result, count)
}

fn sign(input: &mut &[u8]) -> f64 {
    match input.first() {
        Some(b'-') => {
            *input = &input[1..];
            -1.0
        }
        Some(b'+') => {
            *input = &input[1..];
            1.0
        }
        _ => 1.0,
    }
}

pub(crate) fn atof(mut input: &[u8]) -> f64 {
    let multiplier = sign(&mut input);
    let (int_part, n_int) = digits(&mut input);
    let mut value = int_part as f64;
    let mut n_frac = 0;
    if input.first() == Some(&b'.') {
        input = &input[1..];
        let (fraction, count) = digits(&mut input);
        n_frac = count;
        if count > 0 {
            value += fraction as f64 / 10.0f64.powf(count as f64);
        }
    }
    if n_int == 0 && n_frac == 0 {
        return 0.0;
    }
    if matches!(input.first(), Some(b'E' | b'e')) {
        input = &input[1..];
        let exponent_sign = sign(&mut input);
        let (exponent, count) = digits(&mut input);
        if count > 0 {
            value *= 10.0f64.powf(exponent_sign * exponent as f64);
        }
    }
    value * multiplier
}
