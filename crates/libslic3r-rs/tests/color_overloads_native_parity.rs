//! Color.cpp's `ColorRGB` / `ColorRGBA` overloads under their C++ names (`ColorOverloads`: complementary,
//! saturate, decode_color, decode_colors, encode_color; Color.cpp:306-454) against the pin. Vectors from
//! `tests/data/color_overloads/gen.cpp`: 300 colours (greys and hue edges included) with saturation factors 0..3,
//! 136 strings for decode_color (valid 7 and 9 character forms, upper and lower case, bad digits, wrong lengths,
//! no '#', empty) and 45 decode_colors groups (an early failure leaves the rest black). Every line equal, floats
//! bit for bit.
use slicer::color::{ColorOverloads, ColorRGB, ColorRGBA};

/// C's `%a` for a double (normal numbers and zero).
fn fmt_a(x: f64) -> String {
    if x == 0.0 {
        return if x.is_sign_negative() { "-0x0p+0".into() } else { "0x0p+0".into() };
    }
    let bits = x.to_bits();
    let sign = if bits >> 63 != 0 { "-" } else { "" };
    let exp = ((bits >> 52) & 0x7ff) as i64 - 1023;
    let mut mant = format!("{:013x}", bits & ((1u64 << 52) - 1));
    while mant.ends_with('0') {
        mant.pop();
    }
    let e = if exp >= 0 { format!("+{exp}") } else { format!("{exp}") };
    if mant.is_empty() {
        format!("{sign}0x1p{e}")
    } else {
        format!("{sign}0x1.{mant}p{e}")
    }
}

fn a(x: f32) -> String {
    fmt_a(x as f64)
}

fn hexf(s: &str) -> f32 {
    let (neg, s) = match s.strip_prefix('-') {
        Some(r) => (true, r),
        None => (false, s),
    };
    let s = s.strip_prefix("0x").expect("hex float");
    let (mant, exp) = s.split_once('p').expect("exponent");
    let exp: i32 = exp.parse().expect("exp");
    let (int_part, frac_part) = mant.split_once('.').unwrap_or((mant, ""));
    let m = u64::from_str_radix(&format!("{int_part}{frac_part}"), 16).expect("mantissa");
    let v = (m as f64) * 2f64.powi(exp - 4 * frac_part.len() as i32);
    (if neg { -v } else { v }) as f32
}

fn rgb(c: &ColorRGB) -> String {
    format!("{} {} {}", a(c.r()), a(c.g()), a(c.b()))
}

fn rgba(c: &ColorRGBA) -> String {
    format!("{} {} {} {}", a(c.r()), a(c.g()), a(c.b()), a(c.a()))
}

#[test]
fn color_overloads_match_the_pin() {
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/color_overloads/vectors.txt")).unwrap();
    let mut strs: Vec<String> = Vec::new();
    let (mut n, mut bad) = (0usize, 0usize);
    for line in text.lines().filter(|l| !l.starts_with('#')) {
        let got = if let Some(rest) = line.strip_prefix("c ") {
            let f: Vec<&str> = rest.split(' ').collect();
            let (r, g, b, al, factor) = (hexf(f[0]), hexf(f[1]), hexf(f[2]), hexf(f[3]), hexf(f[4]));
            let c3 = ColorRGB::new(r, g, b);
            let c4 = ColorRGBA::new(r, g, b, al);
            format!(
                "c {} {} {} {} {} | {} | {} | {} | {} | {} {}",
                a(r),
                a(g),
                a(b),
                a(al),
                a(factor),
                rgb(&ColorRGB::complementary(&c3)),
                rgba(&ColorRGBA::complementary(&c4)),
                rgb(&ColorRGB::saturate(&c3, factor)),
                rgba(&ColorRGBA::saturate(&c4, factor)),
                ColorRGB::encode_color(&c3),
                ColorRGBA::encode_color(&c4)
            )
        } else if let Some(rest) = line.strip_prefix("d [") {
            let (s, _) = rest.split_once(']').unwrap();
            strs.push(s.to_string());
            let mut o3 = ColorRGB::new(0.25, 0.5, 0.75);
            let mut o4 = ColorRGBA::new(0.25, 0.5, 0.75, 0.125);
            let b3 = ColorRGB::decode_color(s, &mut o3);
            let b4 = ColorRGBA::decode_color(s, &mut o4);
            format!("d [{}] {} {} {} {}", s, b3 as i32, rgb(&o3), b4 as i32, rgba(&o4))
        } else if let Some(rest) = line.strip_prefix("ds ") {
            let k: usize = rest.split(' ').next().unwrap().parse().unwrap();
            let input: Vec<String> = strs[k..k + 4].to_vec();
            let mut o3 = vec![ColorRGB::new(0.5, 0.5, 0.5); 2];
            let mut o4 = vec![ColorRGBA::new(0.5, 0.5, 0.5, 0.5); 5];
            let b3 = ColorRGB::decode_colors(&input, &mut o3);
            let b4 = ColorRGBA::decode_colors(&input, &mut o4);
            let mut s = format!("ds {} {} {}", k, b3 as i32, o3.len());
            for c in &o3 {
                s.push(' ');
                s.push_str(&rgb(c));
            }
            s.push_str(&format!(" | {} {}", b4 as i32, o4.len()));
            for c in &o4 {
                s.push(' ');
                s.push_str(&rgba(c));
            }
            s
        } else {
            panic!("unknown line {line}");
        };
        n += 1;
        if got != line {
            if bad < 10 {
                eprintln!("got  {got}\nwant {line}");
            }
            bad += 1;
        }
    }
    assert_eq!(n, 481);
    assert_eq!(bad, 0, "{bad} of {n} lines differ");
    eprintln!("color overloads: {n} lines equal");
}
