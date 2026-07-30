//! Wipe/prime-tower export integration — port of BambuStudio's
//! `WipeTowerIntegration` (src/libslic3r/GCode.cpp).
//!
//! The wipe tower is *generated* in `gcode/wipe_tower.rs` in tower-local
//! coordinates (corner at the origin) and stored on `Print::wipe_tower_results`.
//! This module owns the *export* side: rewriting that gcode into bed
//! coordinates and (eventually) interleaving it into the main gcode stream at
//! each tool change.
//!
//! Ported so far:
//! - [`transform_gcode`] — GCode.cpp:298, the local→absolute coordinate rewrite.
//!
//! Not yet ported: the stateful `WipeTowerIntegration::append_tcr`
//! (GCode.cpp:647), which coordinates retraction / travel-to-tower / wipe-path /
//! filament start-end gcode with the running `GCode` generator state. That is a
//! larger follow-up tightly coupled to the exporter; `transform_gcode` is its
//! reusable, side-effect-free core.

use crate::gcode::wipe_tower::Vec2f;

/// Port of `transform_gcode` (GCode.cpp:298-346).
///
/// A wipe-tower `ToolChangeResult.gcode` assumes the tower corner sits at the
/// origin (tower-local coordinates, except priming lines). This rewrites every
/// `G1 ` move's X/Y by `Rotation2D(angle) * pos + translation`, leaving the E/F
/// words and every non-`G1` line untouched, so the tower can be placed at its
/// bed position (`translation` = tower position, `angle` = tower rotation in
/// radians). `pos` seeds the running position, used for a `G1` line that omits
/// an axis (it carries the previous value forward, exactly like the C++ scratch
/// `pos`).
///
/// Faithful to the C++ dedup: a `G1` move whose transformed position equals the
/// previous one is left unmodified, and an axis unchanged from the previous move
/// is omitted. The C++ `never_skip_tag` branch is dropped — the Rust wipe-tower
/// generator never emits that tag.
pub fn transform_gcode(gcode: &str, mut pos: Vec2f, translation: Vec2f, angle: f32) -> String {
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    // C++ seeds old_pos with a sentinel no real coordinate matches, so the first
    // move always writes both axes.
    let mut old_pos = Vec2f::new(-1000.1, -1000.1);

    // Process line-by-line. Splitting on '\n' and re-joining on '\n' preserves
    // the original newline structure exactly (unlike the C++ getline loop, which
    // appends one spurious trailing blank line — a quirk not worth reproducing).
    let processed: Vec<String> = gcode
        .split('\n')
        .map(|line| {
            if !line.starts_with("G1 ") {
                return line.to_string();
            }
            // Char-scan the whole line: X/Y consume their number into `pos`;
            // every other character is copied to `line_out` (so `line_out`
            // retains the leading "G1 " and the E/F words, minus the X/Y).
            let mut line_out = String::new();
            let mut it = line.chars().peekable();
            while let Some(&ch) = it.peek() {
                if ch == 'X' || ch == 'Y' {
                    it.next();
                    let mut num = String::new();
                    while let Some(&c2) = it.peek() {
                        if c2.is_ascii_digit()
                            || c2 == '.'
                            || c2 == '-'
                            || c2 == '+'
                            || c2 == 'e'
                            || c2 == 'E'
                        {
                            num.push(c2);
                            it.next();
                        } else {
                            break;
                        }
                    }
                    let val = num.trim().parse::<f32>().unwrap_or(0.0);
                    if ch == 'X' {
                        pos.x = val;
                    } else {
                        pos.y = val;
                    }
                } else {
                    line_out.push(ch);
                    it.next();
                }
            }

            // transformed_pos = Rotation2Df(angle) * pos + translation
            let tx = pos.x * cos_a - pos.y * sin_a + translation.x;
            let ty = pos.x * sin_a + pos.y * cos_a + translation.y;

            if tx != old_pos.x || ty != old_pos.y {
                let mut oss = String::from("G1 ");
                if tx != old_pos.x {
                    oss.push_str(&format!(" X{:.3}", tx));
                }
                if ty != old_pos.y {
                    oss.push_str(&format!(" Y{:.3}", ty));
                }
                oss.push(' ');
                old_pos = Vec2f::new(tx, ty);
                // Replace the leading "G1 " of line_out with the rebuilt prefix.
                line_out.replacen("G1 ", &oss, 1)
            } else {
                // Transformed position unchanged from the previous move: emit the
                // original line untouched (C++ leaves `line` as-is here).
                line.to_string()
            }
        })
        .collect();

    processed.join("\n")
}
// Unit tests live in `tests/wipe_tower_transform.rs` (integration target): the
// crate's in-lib `#[cfg(test)]` target does not currently compile, so tests that
// must actually run are integration tests against the public API.
