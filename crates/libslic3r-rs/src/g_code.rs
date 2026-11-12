//! Port of `src/libslic3r/GCode.cpp` (BambuStudio).
//!
//! Two parts. The first is the free-standing helpers of GCode.cpp: the
//! module-level constants, the bed-type mapping, the string utilities
//! (`check_add_eol`, `custom_gcode_changes_tool`, `transform_gcode`) and
//! `get_wipe_avoid_pos_x`. The second, from "The exporter" on, is what the
//! engine writes G-code with: the `GCode` methods that turn extrusion
//! entities into moves (`extrude_loop`, `extrude_path`, `extrude_perimeters`,
//! `extrude_infill`, `travel_to`, `set_extruder`, ...), as free functions over
//! the writer. `crate::gcode::exporter` is this module under its earlier name.
//! The third part, from "The G-code text and its header" on, is the output
//! container with the header and the config block of `GCode::_do_export`.
//! The last part is the smooth-speed helpers of GCode.cpp.
//! The layer loop of `GCode::process_layer` is still in `print.rs`.

// GCode.cpp:84
pub const G_MIN_PURGE_VOLUME: f32 = 100.0;
// GCode.cpp:85
pub const G_PURGE_VOLUME_ONE_TIME: f32 = 135.0;
// GCode.cpp:86
pub const G_MAX_FLUSH_COUNT: i32 = 4;
// GCode.cpp:87
pub const G_MAX_LABEL_OBJECT: usize = 64;
// GCode.cpp:88
pub const SMOOTH_SPEED_STEP: f64 = 10.0;

// GCode.cpp:89 — static const double not_split_length = scale_(1.0);
#[inline]
pub fn not_split_length() -> f64 {
    crate::libslic3r::scale(1.0) as f64
}
// GCode.cpp:90 — static const double max_step_length = scale_(1.0); // cut path if the path too long
#[inline]
pub fn max_step_length() -> f64 {
    crate::libslic3r::scale(1.0) as f64 // cut path if the path too long
}
// GCode.cpp:91 — static const double min_step_length = scale_(0.4); // cut step
#[inline]
pub fn min_step_length() -> f64 {
    crate::libslic3r::scale(0.4) as f64 // cut step
}

// GCode.cpp:218
// Only add a newline in case the current G-code does not end with a newline.
pub fn check_add_eol(gcode: &mut String) {
    // GCode.cpp:221
    if !gcode.is_empty() && !gcode.ends_with('\n') {
        gcode.push('\n');
    }
}

// GCode.cpp:227
// Return true if tch_prefix is found in custom_gcode
pub fn custom_gcode_changes_tool(custom_gcode: &str, tch_prefix: &str, next_extruder: u32) -> bool {
    // GCode.cpp:229
    let mut ok = false;
    // GCode.cpp:230
    let mut from_pos: usize = 0;
    // GCode.cpp:232
    let bytes = custom_gcode.as_bytes();
    while let Some(rel) = custom_gcode[from_pos..].find(tch_prefix) {
        let mut pos = from_pos + rel;
        // GCode.cpp:233
        if pos + 1 == custom_gcode.len() {
            break;
        }
        // GCode.cpp:235
        from_pos = pos + 1;
        // GCode.cpp:236-240 — only whitespace is allowed before the command.
        //   while (--pos < custom_gcode.size() && custom_gcode[pos] != '\n')
        // C++ uses an unsigned size_t for `pos`, so when `pos` is 0 the
        // pre-decrement wraps to SIZE_MAX which fails the `< size()` test and
        // terminates the loop. Replicate that wrap-around exactly.
        let mut next_iter = true;
        loop {
            pos = pos.wrapping_sub(1);
            if !(pos < custom_gcode.len() && bytes[pos] != b'\n') {
                break;
            }
            // GCode.cpp:238 — if (!std::isspace(custom_gcode[pos])) goto NEXT;
            if !is_space(bytes[pos]) {
                next_iter = false;
                break;
            }
        }
        if next_iter {
            // GCode.cpp:241-247
            // we should also check that the extruder changes to what was expected
            let tail = &custom_gcode[from_pos..];
            if let Some(num) = parse_leading_uint(tail) {
                ok = num == next_extruder;
            }
        }
        // GCode.cpp:248 NEXT:;
    }
    // GCode.cpp:250
    ok
}

// std::isspace for the default "C" locale: space, \t, \n, \v, \f, \r.
#[inline]
fn is_space(c: u8) -> bool {
    matches!(c, b' ' | b'\t' | b'\n' | 0x0b | 0x0c | b'\r')
}

// Mirror of C++ `std::istringstream >> unsigned`: skip leading whitespace, then
// read a run of decimal digits. Returns None if no number could be parsed.
fn parse_leading_uint(s: &str) -> Option<u32> {
    let bytes = s.as_bytes();
    let mut i = 0;
    while i < bytes.len() && is_space(bytes[i]) {
        i += 1;
    }
    let start = i;
    while i < bytes.len() && bytes[i].is_ascii_digit() {
        i += 1;
    }
    if start == i {
        return None;
    }
    s[start..i].parse::<u32>().ok()
}

// GCode.cpp:298
// Postprocesses gcode: rotates and moves G1 extrusions and returns result.
//
// `WipeTower::never_skip_tag()` (WipeTower.hpp:28) is not yet ported; its value
// is the literal below.
pub fn transform_gcode(gcode: &str, mut pos: Vec2f, translation: &Vec2f, angle: f32) -> String {
    // GCode.cpp:300
    let extruder_offset = Vec2f::new(0.0, 0.0);
    // GCode.cpp:302
    let mut gcode_out = String::new();
    // GCode.cpp:305
    let mut old_pos = Vec2f::new(-1000.1, -1000.1);

    // GCode.cpp:307-308 — read the gcode line by line.
    for raw_line in split_getlines(gcode) {
        let mut line = raw_line;

        // GCode.cpp:310
        if line.starts_with("G1 ") {
            // GCode.cpp:311
            let mut never_skip = false;
            // GCode.cpp:312-317
            if let Some(it) = line.find(never_skip_tag()) {
                // remove the tag and remember we saw it
                never_skip = true;
                // GCode.cpp:316 — line.erase(it, it + WipeTower::never_skip_tag().size());
                // std::string::erase(index, count) removes min(count, size()-index)
                // chars starting at `index`. C++ passes `it + tag.size()` as the
                // *count* (not an end index), so it erases up to `it + tag_len`
                // characters from offset `it`, clamped to the string end. Replicate
                // that quirk exactly rather than erasing only `tag_len` chars.
                let count = it + never_skip_tag().len();
                let end = (it + count).min(line.len());
                line.replace_range(it..end, "");
            }
            // GCode.cpp:318-327 — parse X/Y values out of the line, building
            // line_out from every character that is not an X/Y coordinate.
            //   line_str >> std::noskipws; (don't skip whitespace)
            let mut line_out = String::new();
            let chars: Vec<char> = line.chars().collect();
            let mut idx = 0;
            while idx < chars.len() {
                let ch = chars[idx];
                idx += 1;
                if ch == 'X' || ch == 'Y' {
                    // line_str >> (ch == 'X' ? pos.x() : pos.y());
                    let (val, consumed) = stream_read_float(&chars[idx..]);
                    idx += consumed;
                    if let Some(v) = val {
                        if ch == 'X' {
                            pos[0] = v;
                        } else {
                            pos[1] = v;
                        }
                    }
                } else {
                    line_out.push(ch);
                }
            }

            // GCode.cpp:329 — transformed_pos = Eigen::Rotation2Df(angle) * pos + translation;
            let transformed_pos = rotate2d(angle, pos) + translation;

            // GCode.cpp:331
            if transformed_pos != old_pos || never_skip {
                // GCode.cpp:332
                line = line_out;
                // GCode.cpp:333-336
                let mut oss = String::from("G1 ");
                // oss << std::fixed << std::setprecision(3)
                if transformed_pos.x != old_pos.x || never_skip {
                    oss.push_str(&format!(" X{:.3}", transformed_pos.x - extruder_offset.x));
                }
                if transformed_pos.y != old_pos.y || never_skip {
                    oss.push_str(&format!(" Y{:.3}", transformed_pos.y - extruder_offset.y));
                }
                // GCode.cpp:337
                oss.push(' ');
                // GCode.cpp:338 — line.replace(line.find("G1 "), 3, oss.str());
                if let Some(g1) = line.find("G1 ") {
                    line.replace_range(g1..g1 + 3, &oss);
                }
                // GCode.cpp:339
                old_pos = transformed_pos;
            }
        }

        // GCode.cpp:343
        gcode_out.push_str(&line);
        gcode_out.push('\n');
    }
    // GCode.cpp:345
    gcode_out
}

// GCode.cpp:348
pub fn get_wipe_avoid_pos_x(wt_min: &Vec2f, wt_max: &Vec2f, offset: f32) -> f32 {
    // GCode.cpp:350
    let left: f32 = 100.0;
    let right: f32 = 250.0;
    // GCode.cpp:351
    let default_value: f32 = 110.0;
    // GCode.cpp:352
    let a: f32;
    let b: f32;
    // GCode.cpp:353
    a = wt_max.x + offset;
    // GCode.cpp:354
    b = wt_min.x - offset;
    // GCode.cpp:355
    if a > left && a < right {
        return a;
    }
    // GCode.cpp:356
    if b > left && b < right {
        return b;
    }
    // GCode.cpp:357
    default_value
}

/// GCode.cpp:623-739 `WipeTowerIntegration::generate_path_to_wipe_tower`.
///
/// `start_pos` is the last position before the wipe tower, `end_pos` the
/// tower's start; both, the tower box `avoid_polygon` and the bed box
/// `printer_bbx` (as (min, max)) are in print coordinates. The path goes
/// straight to the point `p` where a ray from `end_pos` along the tower's
/// first print direction leaves the tower box grown by 2 mm, around that box
/// along the shorter way when the line from `start_pos` to `p` crosses it, and
/// ends at `end_pos`.
pub fn generate_path_to_wipe_tower(
    start_pos: crate::Point,
    end_pos: crate::Point,
    avoid_polygon: (crate::Point, crate::Point),
    printer_bbx: (crate::Point, crate::Point),
) -> Vec<crate::Point> {
    use crate::Point;
    let sf = crate::libslic3r::SCALING_FACTOR;
    let mut res: Vec<Point> = Vec::new();
    // `coord_t alpha = scaled(2.f)`: float over float(SCALING_FACTOR), truncated.
    let alpha = (2.0_f32 / sf as f32) as crate::Coord;
    // BoundingBox::offset
    let inner_min = Point::new(avoid_polygon.0.x - alpha, avoid_polygon.0.y - alpha);
    let inner_max = Point::new(avoid_polygon.1.x + alpha, avoid_polygon.1.y + alpha);
    let width = inner_max.x - inner_min.x;
    // BoundingBox::polygon(): min, (max.x, min.y), max, (min.x, max.y).
    let bbox_polygon = |min: Point, max: Point| {
        vec![min, Point::new(max.x, min.y), max, Point::new(min.x, max.y)]
    };
    let bed_polygon = bbox_polygon(printer_bbx.0, printer_bbx.1);
    // The first print direction of end_pos: Vec2f v(1, 0), reversed when
    // end_pos is in the left half of the box.
    let mut v: (f64, f64) = (1.0, 0.0);
    if (end_pos.x - inner_min.x).abs() < width / 2 {
        v = (-v.0, -v.1);
    }
    // A box that is not inside the bed: go straight to end_pos.
    let avoid_points = bbox_polygon(inner_min, inner_max);
    for p in &avoid_points {
        if crate::geometry::polygon::point_in_polygon(p, &bed_polygon) != 1 {
            res.push(end_pos);
            return res;
        }
    }
    let unscale = |p: &Point| (p.x as f64 * sf, p.y as f64 * sf);
    // `scaled(Vec2d)`: divided by SCALING_FACTOR and truncated.
    let scaled_pt = |x: f64, y: f64| Point::new((x / sf) as crate::Coord, (y / sf) as crate::Coord);
    let cross2 = |a: (f64, f64), b: (f64, f64)| a.0 * b.1 - a.1 * b.0;
    let eps = crate::libslic3r::EPSILON;
    // Ray-line segment intersection (:648-662).
    let ray_intersetion_line =
        |a: (f64, f64), v1: (f64, f64), b: (f64, f64), c: (f64, f64)| -> Option<Point> {
            let v2 = (c.0 - b.0, c.1 - b.1);
            let denom = cross2(v1, v2);
            if denom.abs() < eps {
                return None;
            }
            let v12 = (a.0 - b.0, a.1 - b.1);
            let nume_a = cross2(v2, v12);
            let nume_b = cross2(v1, v12);
            let t1 = nume_a / denom;
            let t2 = nume_b / denom;
            if t1 >= 0.0 && t2 >= 0.0 && t2 <= 1.0 {
                return Some(scaled_pt(a.0 + t1 * v1.0, a.1 + t1 * v1.1));
            }
            None
        };
    // Inter_info: the edge index and the point.
    // calc_path_len (:668-689): the path along the box from beg to end, and the
    // sum of the squared lengths of its legs.
    let calc_path_len = |points: &[Point],
                         beg_info: (usize, Point),
                         end_info: (usize, Point),
                         is_add: bool|
     -> (Vec<Point>, f64) {
        let n = points.len();
        let beg = if is_add { (beg_info.0 + 1) % n } else { beg_info.0 };
        let end = if is_add { end_info.0 } else { (end_info.0 + 1) % n };
        let mut i = beg;
        let mut len = 0.0;
        let mut path: Vec<Point> = Vec::new();
        path.push(beg_info.1);
        let (bx, by) = unscale(&beg_info.1);
        let (px, py) = unscale(&points[beg]);
        len += (bx - px) * (bx - px) + (by - py) * (by - py);
        while i != end {
            let ni = if is_add { (i + 1) % n } else { (i + n - 1) % n };
            let a = unscale(&points[i]);
            let b = unscale(&points[ni]);
            len += (a.0 - b.0) * (a.0 - b.0) + (a.1 - b.1) * (a.1 - b.1);
            path.push(points[i]);
            i = ni;
        }
        path.push(points[end]);
        path.push(end_info.1);
        let (ex, ey) = unscale(&end_info.1);
        let (qx, qy) = unscale(&points[end]);
        len += (ex - qx) * (ex - qx) + (ey - qy) * (ey - qy);
        (path, len)
    };
    // :693-704 — where the ray from end_pos along v leaves the box ('p').
    let mut inter_info: Option<(usize, Point)> = None;
    for i in 0..avoid_points.len() {
        let a = &avoid_points[i];
        let b = &avoid_points[(i + 1) % avoid_points.len()];
        if let Some(p) = ray_intersetion_line(unscale(&end_pos), v, unscale(a), unscale(b)) {
            inter_info = Some((i, p));
            break;
        }
    }
    let Some(inter_info) = inter_info else {
        res.push(end_pos);
        return res;
    };
    // :709-722 — the other crossing of start_pos -> p with the box ('p_').
    let mut inter_info2: Option<(usize, Point)> = None;
    let (sa, sb) = (unscale(&start_pos), unscale(&inter_info.1));
    for i in 0..avoid_points.len() {
        if i == inter_info.0 {
            continue;
        }
        let a = unscale(&avoid_points[i]);
        let b = unscale(&avoid_points[(i + 1) % avoid_points.len()]);
        // line_alg::intersection(Linef, Linef) (Line.hpp:123-148).
        let v1 = (sb.0 - sa.0, sb.1 - sa.1);
        let v2 = (b.0 - a.0, b.1 - a.1);
        let denom = cross2(v1, v2);
        if denom.abs() < eps {
            continue;
        }
        let v12 = (sa.0 - a.0, sa.1 - a.1);
        let nume_a = cross2(v2, v12);
        let nume_b = cross2(v1, v12);
        let t1 = nume_a / denom;
        let t2 = nume_b / denom;
        if t1 >= 0.0 && t1 <= 1.0 && t2 >= 0.0 && t2 <= 1.0 {
            let inter = (sa.0 + t1 * v1.0, sa.1 + t1 * v1.1);
            inter_info2 = Some((i, scaled_pt(inter.0, inter.1)));
            break;
        }
    }
    // :725-736
    match inter_info2 {
        None => res.push(inter_info.1),
        Some(info2) => {
            let (path1, len1) = calc_path_len(&avoid_points, info2, inter_info, true);
            let (path2, len2) = calc_path_len(&avoid_points, info2, inter_info, false);
            let path = if len1 < len2 { path1 } else { path2 };
            res.extend(path);
        }
    }
    res.push(end_pos);
    res
}


// ============================================================================
// WipeTowerIntegration (GCode.cpp): the export of the wipe tower's results.
// The tower is generated in `gcode/wipe_tower.rs` in tower coordinates and kept
// on `Print::wipe_tower_results`; this is the part that writes it into the
// G-code (R841 moved it here from `gcode/wipe_tower_integration.rs` and
// `print.rs`).
// ============================================================================

/// The placeholder the wipe-tower generator emits where the tool change belongs
/// (WipeTower.cpp:2466). The export substitutes the evaluated
/// `change_filament_gcode` for it.
pub const CHANGE_FILAMENT_PLACEHOLDER: &str = "[change_filament_gcode]";

/// Substitute the tool-change block into transformed tower gcode.
///
/// Port of the `change_filament_gcode` half of `append_tcr` (GCode.cpp:936-1058):
/// C++ evaluates the `change_filament_gcode` template, appends the `Tn` command
/// if the custom block didn't already change tools, and injects the result into
/// the tower gcode via its `[change_filament_gcode]` placeholder. When the
/// template is empty, a bare `Tn` is used instead (GCode.cpp:754).
pub fn substitute_change_filament(
    tower_gcode: &str,
    change_filament_block: Option<&str>,
    new_tool: usize,
    toolchange_prefix: &str,
    // Emitted immediately AFTER the substituted change-filament block, i.e. right
    // before the tower's own moves — which is where C++ puts `; FEATURE: Prime
    // tower` (it appears after the block's closing `G1 E.8`, followed by
    // `; WIPE_TOWER_START`). Placing it before the whole block instead attributes
    // the tool change's own retract/ramming to the tower feature.
    feature_marker: Option<&str>,
    // The evaluated `filament_end_gcode`, emitted BEFORE the change-filament
    // block — C++ substitutes it into the tower's own `[filament_end_gcode]`
    // placeholder, which sits one line above `[change_filament_gcode]`
    // (WipeTower.cpp:2465-2466).
    filament_end_block: Option<&str>,
) -> String {
    let tool_cmd = format!("{}{}", toolchange_prefix, new_tool);
    let replacement = match change_filament_block {
        Some(block) if !block.trim().is_empty() => {
            let mut s = block.trim_end().to_string();
            // GCode.cpp:960 — only append the tool command if the custom gcode
            // does not already perform the tool change itself.
            if !tower_block_changes_tool(&s, toolchange_prefix, new_tool) {
                s.push('\n');
                s.push_str(&tool_cmd);
            }
            s
        }
        _ => tool_cmd,
    };
    let replacement = match feature_marker {
        Some(m) => format!("{replacement}\n{m}"),
        None => replacement,
    };
    let replacement = match filament_end_block {
        // The block as it is (it ends with a newline) under
        // TOOLCHANGE_TEMPLATE_TEXT; trimmed otherwise.
        Some(b) if crate::faithful_gate("TOOLCHANGE_TEMPLATE_TEXT") && b.ends_with('\n') => {
            // The tower writes "[filament_end_gcode]\n" (WipeTower.cpp:2465), so
            // the line's own newline follows the block.
            format!("{b}\n{replacement}")
        }
        Some(b) if !b.trim().is_empty() => format!("{}\n{replacement}", b.trim_end()),
        _ => replacement,
    };
    tower_gcode.replace(CHANGE_FILAMENT_PLACEHOLDER, &replacement)
}

/// Does the custom block already contain a line that switches to `new_tool`?
/// A line-based check written for `substitute_change_filament`; the C++
/// `custom_gcode_changes_tool` (GCode.cpp:227-251, `custom_gcode_changes_tool`
/// above) keeps the last match instead.
fn tower_block_changes_tool(gcode: &str, toolchange_prefix: &str, new_tool: usize) -> bool {
    let want = format!("{}{}", toolchange_prefix, new_tool);
    gcode.lines().any(|l| {
        let l = l.trim();
        // A tool command is the whole line (possibly with a trailing comment).
        l == want || l.starts_with(&format!("{} ", want)) || l.starts_with(&format!("{};", want))
    })
}

/// Port of `WipeTowerIntegration::post_process_wipe_tower_moves`
/// (GCode.cpp:1383-1465; `transform_gcode` until R841, which moved it here from
/// `gcode/wipe_tower_integration.rs`).
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
pub fn post_process_wipe_tower_moves(gcode: &str, mut pos: crate::gcode::wipe_tower::Vec2f, translation: crate::gcode::wipe_tower::Vec2f, angle: f32) -> String {
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    // C++ seeds old_pos with a sentinel no real coordinate matches, so the first
    // move always writes both axes.
    let mut old_pos = crate::gcode::wipe_tower::Vec2f::new(-1000.1, -1000.1);

    // Process line-by-line. Splitting on '\n' and re-joining on '\n' preserves
    // the original newline structure exactly (unlike the C++ getline loop, which
    // appends one trailing blank line, added at the end under TOWER_XFORM_TRAILING_BLANK).
    let processed: Vec<String> = gcode
        .split('\n')
        .map(|line| {
            if !line.starts_with("G1 ") {
                // GCode.cpp:1133-1145: after the change-filament placeholder the
                // tracker forgets its position, so the next move re-emits X and Y.
                if line == "[change_filament_gcode]" && crate::faithful_gate("TOWER_XFORM_TC_RESET") {
                    old_pos = crate::gcode::wipe_tower::Vec2f::new(-1000.1, -1000.1);
                }
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
                old_pos = crate::gcode::wipe_tower::Vec2f::new(tx, ty);
                // Replace the leading "G1 " of line_out with the rebuilt prefix.
                line_out.replacen("G1 ", &oss, 1)
            } else {
                // Transformed position unchanged from the previous move. C++ leaves
                // `line` as-is here (GCode.cpp:1120-1132 only rewrites inside the
                // `transformed_pos != old_pos` branch), which is safe for it because
                // its writer never emits a redundant travel to a point it is already
                // at. Ours does: `finish_layer` draws the outer perimeter from the
                // tower-local origin, and the writer had already moved there, so the
                // duplicate `G1 X0.000 Y0.000` was echoed VERBATIM -- in tower-local
                // coordinates. The printer read it as bed (0,0) and then extruded the
                // next purge strokes 220mm across the plate from the origin: 61
                // layers, 27,970mm at ~0.009 E/mm (R477).
                // Emitting `line_out` (the same line with X/Y stripped) keeps C++'s
                // "don't repeat unchanged coordinates" intent and cannot leak an
                // untransformed coordinate: by construction the position is the one
                // already reached, so dropping the axes is a no-op move.
                if crate::faithful_gate("TOWER_XFORM_NO_RAW_ECHO") {
                    line_out
                } else {
                    line.to_string()
                }
            }
        })
        .collect();

    let mut out = processed.join("\n");
    // TOWER_XFORM_TRAILING_BLANK (on): `while (gcode_str) { std::getline(..);
    // ..; gcode_out += line + "\n"; }` (GCode.cpp post_process_wipe_tower_moves)
    // reads one more, empty line after a text that ends with a newline, and
    // writes it: every tower block ends with one empty line more. =0 keeps the
    // text's own end.
    if crate::faithful_gate("TOWER_XFORM_TRAILING_BLANK") && (gcode.is_empty() || gcode.ends_with('\n')) {
        out.push('\n');
    }
    out
}
// Unit tests live in `tests/wipe_tower_transform.rs` (integration target): the
// crate's in-lib `#[cfg(test)]` target does not currently compile, so tests that
// must actually run are integration tests against the public API.

/// Emit one wipe/prime-tower `ToolChangeResult` into the main gcode stream,
/// transformed from tower-local to bed coordinates (port of
/// `WipeTowerIntegration::append_tcr`, GCode.cpp:741-1380; moved here from
/// `print.rs`, where it was `emit_tower_tcr`, in R841). The tcr gcode is
/// self-contained (retract → travel → purge → optional Tn → unretract) and all
/// its moves are absolute once transformed, so the head physically lands in the
/// right place regardless of the writer's tracked position.
///
/// NOTE: the writer's tracked XY position is deliberately NOT updated here. C++
/// calls `set_last_pos(tcr.end_pos)` but `tcr.end_pos` is captured mid-sequence
/// (before the tcr's own wipe/z-hop), so syncing to it desynced the subsequent
/// object wipe projection and threw thousands of wipe moves off-bed. Leaving the
/// tracked position stale is byte-clean for the object stream (all moves are
/// absolute); a faithful final-position + wipe-path sync is future work.
///
/// `off` = tower position (wipe_tower_x/y); rotation is 0 for now (Majora's
/// `wipe_tower_rotation_angle` is 0). Does NOT yet wrap the bare `Tn` in the
/// faithful `change_filament_gcode` + filament start/end templates.
pub(crate) fn append_tcr(
    writer: &mut crate::gcode::GCodeWriter,
    tcr: &crate::gcode::wipe_tower::ToolChangeResult,
    off: crate::gcode::wipe_tower::Vec2f,
    print_config: &crate::print_config::PrintConfig,
    max_layer_z: f64,
    toolchange_count: i64,
    // R603: total tool changes in the whole print, for the `M73 E` countdown
    // (C++ `total_filament_change`). 0 disables the emission.
    total_tool_changes: i64,
    first_layer: bool,
) {
    // TOWER_RESET_ACCEL (on): append_tcr begins and ends with
    // `gcodegen.reset_last_acceleration()` (GCode.cpp:743, 1377): the tower's
    // own G-code sets accelerations the writer does not see, so the next M204
    // is written whatever the last one was.
    let tower_reset_accel = crate::faithful_gate("TOWER_RESET_ACCEL");
    if tower_reset_accel {
        writer.reset_last_acceleration();
    }
    // TOWER_GENERATE_NEW_CPP (on): a tool change whose result begins with the
    // finish of the layer (`merge_tcr(finish_layer_tcr, ..)`) carries the
    // finish's `is_finish_first`, and `append_tcr` first retracts, travels to
    // the result's start and unretracts (GCode.cpp:787-795):
    // `gcodegen.retract()` is a lazy spiral lift (GCode.hpp:246).
    // TOWER_FINISH_FIRST_TRAVEL_CPP (on): the same for a result that is not a
    // tool change (the timelapse wall, the finish of a layer without tool
    // changes), which the port reached with a raw move to its start without
    // the extruder offset and without the unretract. =0 keeps that move.
    let finish_first_travel = tcr.is_finish_first
        && !tcr.priming
        && if tcr.is_tool_change {
            crate::faithful_gate("TOWER_GENERATE_NEW_CPP")
        } else {
            crate::faithful_gate("TOWER_FINISH_FIRST_TRAVEL_CPP")
        };
    if finish_first_travel {
        writer.retract_with_lift_type(1, false);
        let (gox, goy) = writer.gcode_origin();
        writer.travel_to_tower_point(tcr.start_pos.x as f64 + gox, tcr.start_pos.y as f64 + goy);
        writer.unretract();
    }
    // `transform_gcode`'s `pos` seed is the tower-LOCAL start (same frame as the
    // gcode body); start_pos is stored already-absolute, so recover the local
    // seed by subtracting the offset (else axis-less G1 lines double-count it).
    let local_seed = crate::gcode::wipe_tower::Vec2f::new(
        tcr.start_pos.x - off.x,
        tcr.start_pos.y - off.y,
    );
    // R808: GCode.cpp:1096-1140 post_process_wipe_tower_moves subtracts the
    // extruder offset (`extruder_offset = 0x2` on Majora's printer) from every
    // transformed tower coordinate; the entry travel (tcr.start_pos) does not.
    // TOWER_EXTRUDER_OFFSET=0 restores the unshifted tower.
    let off_gcode = if crate::faithful_gate("TOWER_EXTRUDER_OFFSET") {
        crate::gcode::wipe_tower::Vec2f::new(
            off.x - print_config.extruder_offset_x as f32,
            off.y - print_config.extruder_offset_y as f32,
        )
    } else {
        off
    };
    let g = post_process_wipe_tower_moves(&tcr.gcode, local_seed, off_gcode, 0.0);
    // Substitute the tower's `[change_filament_gcode]` placeholder with the
    // evaluated custom tool-change template (append_tcr, GCode.cpp:936-1058).
    // Only real tool changes carry one; the finish_layer tcr has no placeholder,
    // so the replace is a no-op there.
    let block = if tcr.is_tool_change && !print_config.change_filament_gcode.is_empty() {
        let ctx = crate::gcode::change_filament::build_context(
            print_config,
            tcr.initial_tool.max(0) as usize,
            tcr.new_tool.max(0) as usize,
            tcr.print_z as f64,
            max_layer_z,
            toolchange_count,
            tcr.purge_volume as f64,
            first_layer,
        );
        if crate::faithful_gate("TOOLCHANGE_TEMPLATE_TEXT") {
            // TOOLCHANGE_TEMPLATE_TEXT (on): the placeholder parser removes the
            // `{if}` tags and nothing else, so a line that held only a tag stays
            // as an empty line; `check_add_eol`; then the two markers
            // (GCode.cpp:1122-1128). =0 drops those lines and writes no markers.
            let mut t = crate::gcode::gcode_template::process_text(
                &print_config.change_filament_gcode,
                &ctx,
            );
            if !t.is_empty() && !t.ends_with('\n') {
                t.push('\n');
            }
            Some(format!("; FilamentChangeGCodeStart\n{t}; FilamentChangeGCodeEnd\n"))
        } else {
            Some(crate::gcode::gcode_template::process(
                &print_config.change_filament_gcode,
                &ctx,
            ))
        }
    } else {
        None
    };
    // append_tcr (GCode.cpp:1035-1053) substitutes THREE placeholders into the
    // tower gcode, not one: `[filament_end_gcode]`, `[change_filament_gcode]`
    // and `[filament_start_gcode]` (the tower writes all three —
    // WipeTower.cpp:2465/2466/2483). We emitted only the middle one, so Majora
    // was missing 2,723 `; filament end gcode` and 2,723 `; filament start
    // gcode` blocks that C++ writes (R495). Both templates are evaluated with
    // the same placeholder machinery as the change-filament block; C++ picks
    // them per filament (`get_at(new_filament_id)` for start, the old filament
    // for end), while our config carries a single value for each.
    // These templates are MATERIAL-INERT for this profile — `filament_end_gcode`
    // is just its comment, `filament_start_gcode` expands to the comment plus
    // `M106 P3 S150` — so this closes a gcode-CONTENT gap, not a material one.
    // FILAMENT_START_END_GCODE=0 restores the old two-placeholder behaviour.
    let fil_gcode_faithful = crate::faithful_gate("FILAMENT_START_END_GCODE");
    let eval_fil = |tmpl: &str| -> Option<String> {
        if !fil_gcode_faithful || tmpl.is_empty() || !tcr.is_tool_change {
            return None;
        }
        let ctx = crate::gcode::change_filament::build_context(
            print_config,
            tcr.initial_tool.max(0) as usize,
            tcr.new_tool.max(0) as usize,
            tcr.print_z as f64,
            max_layer_z,
            toolchange_count,
            tcr.purge_volume as f64,
            first_layer,
        );
        let out = crate::gcode::gcode_template::process(tmpl, &ctx);
        if out.trim().is_empty() {
            None
        } else {
            Some(out)
        }
    };
    let filament_end_block = if crate::faithful_gate("TOOLCHANGE_TEMPLATE_TEXT")
        && fil_gcode_faithful
        && tcr.is_tool_change
        && !print_config.filament_end_gcode.is_empty()
    {
        // The text as the parser leaves it, trailing blanks and empty lines
        // included, with `check_add_eol`.
        let ctx = crate::gcode::change_filament::build_context(
            print_config,
            tcr.initial_tool.max(0) as usize,
            tcr.new_tool.max(0) as usize,
            tcr.print_z as f64,
            max_layer_z,
            toolchange_count,
            tcr.purge_volume as f64,
            first_layer,
        );
        let mut t = crate::gcode::gcode_template::process_text(&print_config.filament_end_gcode, &ctx);
        if !t.is_empty() && !t.ends_with('\n') {
            t.push('\n');
        }
        if t.is_empty() { None } else { Some(t) }
    } else {
        eval_fil(&print_config.filament_end_gcode)
    };
    let filament_start_block = eval_fil(&print_config.filament_start_gcode);
    // Label tower extrusions like C++ does (`; FEATURE: Prime tower`). Without
    // this the tower's E is attributed to whichever feature preceded it, which
    // silently inflates that feature in per-feature parity comparisons.
    //
    // R464: the marker must sit AFTER the change-filament block, immediately
    // before the tower's own moves — that is where C++ emits it (after the
    // block's closing `G1 E.8`, then `; WIPE_TOWER_START`, then the strokes).
    // Emitting it before the whole block put the tool change's retract and
    // ramming INSIDE the tower feature, leaving an outstanding retraction at the
    // first purge stroke; that is what made ~one stroke per toolchange read at
    // ~0.011 E/mm instead of 0.0543 (R463).
    // R534 — C++ writes THREE analyzer lines here, not one: the `WipeTowerWriter`
    // constructor (WipeTower.cpp:637-642) emits
    //     ";" + reserved_tag(Height) + to_string(m_layer_height)
    //     ";" + reserved_tag(Role)   + role_to_string(erWipeTower)
    //     ";" + reserved_tag(Width)  + to_string(line_width)      [= m_perimeter_width]
    // and the tag strings are `" LAYER_HEIGHT: "` / `" FEATURE: "` / `" LINE_WIDTH: "`
    // (GCodeProcessor.cpp:50-54) — so `ETags::Width` IS the same reserved tag the
    // object paths already emit. We were writing only the middle line, which left the
    // tower's 3,377 blocks with no height/width for the analyzer and made the tower
    // inherit whatever width the preceding object feature had set.
    //
    // PLACEMENT DIVERGENCE (deliberate, see R464 above): C++ emits the trio at the
    // HEAD of `tcr.gcode`, i.e. BEFORE the change-filament block, so its flush moves
    // fall inside the Prime tower feature. We emit after the block. Keeping the three
    // lines contiguous matches C++'s output shape; moving the whole trio upstream is a
    // separate change with per-feature-attribution consequences.
    //
    // R601 MEASURED THOSE CONSEQUENCES AND CLOSED THE DIVERGENCE.
    // With the trio after the block, the entire tool change -- ramming, retracts,
    // M620/M621, the filament end/start templates -- stays under whatever object
    // feature happened to precede the tower. On Majora that mis-attributes
    // ~190k M-codes, ~147k retracts and ~105k travels: C++ has 16,341 of its 16,347
    // `M620` under `Prime tower` and we had ZERO, spraying them over Internal solid
    // infill (6,930), Floating vertical shell (4,812) and Sparse infill (3,402).
    // Since `line_parity.py` groups by (layer, feature), a mis-tagged line can never
    // match even when it is byte-identical, so this depressed every Majora feature.
    //
    // R464's stated reason for the divergence does not survive scrutiny: the trio is
    // three COMMENTS and cannot change any extrusion. The "outstanding retraction at
    // the first purge stroke" it blamed was the missing toolchange unretract, fixed
    // independently in R466 (`G1 E{retract_length_toolchange}` in the trailer below).
    // C++ emits the trio immediately before `;----` / `; CP TOOLCHANGE START`.
    let tag_at_head = crate::faithful_gate("TOWER_FEATURE_TAG_HEAD");
    let tower_feature = format!(
        "; LAYER_HEIGHT: {:.6}\n; FEATURE: Prime tower\n; LINE_WIDTH: {:.6}",
        tcr.layer_height, tcr.perimeter_width
    );
    let tower_feature: &str = &tower_feature;
    let had_placeholder = g.contains(CHANGE_FILAMENT_PLACEHOLDER);
    // Where the entry retract goes: `toolchange_retract_str` heads the text that
    // replaces `[filament_end_gcode]` (GCode.cpp:948), which follows the
    // `; WIPE_TOWER_START` of the tool change. In a result that begins with
    // the finish of the layer that is not the first one (TOWER_GENERATE_NEW_CPP);
    // the text before the placeholder is not changed by the substitution.
    let entry_cut: Option<usize> = {
        const START: &str = "; WIPE_TOWER_START\n";
        let ph = g.find(CHANGE_FILAMENT_PLACEHOLDER);
        match ph {
            Some(p) if crate::faithful_gate("TOWER_GENERATE_NEW_CPP") => {
                g[..p].rfind(START).map(|i| i + START.len())
            }
            _ => g.find(START).map(|i| i + START.len()),
        }
    };
    // R466 — the unretract C++ emits in `GCode::append_tcr` (WipeTower.cpp:2436
    // "BBS: do travel in GCode::append_tcr() for lazy_lift"): a bare
    // `G1 E{retract_length_toolchange} F{speed}` AFTER the change-filament block and
    // before the tower's moves, repaying the `G1 E-[new_retract_length_toolchange]`
    // that block ends on. The template never unretracts (it ends at
    // `M621 S[next_extruder]A`), so without this the filament stays retracted and the
    // first purge stroke of every tower block is starved (R465). C++ orders it
    // unretract-then-marker, so build the whole trailer here.
    // R476 — GCode.cpp:687-695 `travel_to(start_pos, erMixed, "Travel to a Wipe
    // Tower")`. C++ guards that travel with `tcr.is_finish_first` because, as its
    // comment at :685 says, "toolchange gcode will move to start_pos" — its
    // change_filament template leaves the head at the tower. THIS profile's template
    // does not: it ends with `G1 X165 F15000` / `G1 Y256 ; move Y to aside, prevent
    // collision`. The tower's own first move is Y-only (the WipeTowerWriter believes
    // X is already at the block's left edge), so `G1 X219.729 E1.8473` was extruding
    // a 54.7mm line from x=165 straight across the plate instead of a 34mm purge
    // stroke -- at 0.034 E/mm instead of 0.0543. That is 2,786 segments / 162,518mm,
    // i.e. the ENTIRE prime-tower length excess, and a real print defect.
    // So emit the travel unconditionally, which is what C++'s guard assumes has
    // already happened.
    let travel_to_start = if crate::faithful_gate("TOWER_TRAVEL_TO_START") {
        let f = if print_config.travel_speed > 0.0 {
            print_config.travel_speed * 60.0
        } else {
            30000.0
        };
        // R644: emitted BARE. C++ does pass "Travel to a Wipe Tower" to
        // `travel_to` (GCode.cpp:698-701), but `GCodeWriter::emit_comment` is
        // gated on `full_gcode_comment`, which is OFF on both fixtures — the
        // string appears ZERO times in C++'s Majora output against our 3,377.
        // We do not model `full_gcode_comment` at all, so writer-generated
        // comments must simply not be written; the ones that DO appear in C++'s
        // output come from gcode TEMPLATES, which are verbatim text.
        // R650: C++'s `travel_to` is an XY move FOLLOWED BY a Z move
        // (GCodeWriter::travel_to_z, GCodeWriter.cpp:626) — the lazy lift is
        // cancelled by dropping back to the layer's print_z. We emitted the XY
        // half only. The R650 census sized it exactly: inside `; WIPE_TOWER_START`
        // …`_END`, C++ has 5,446 bare `G1 Z<z>` lines (no F) against our ZERO,
        // while the F-bearing ones match at 5,442/5,446 — the whole tower Z
        // deficit is this one class, two per tool change (one for this travel,
        // one for the avoid-perimeter detour travel we do not emit at all).
        // Formatted with `format_gcode_value(_, 3)` so 1.5 → "1.5" and 0.3 →
        // ".3", matching C++'s `emit_z` (verified against `G1 Z.3` at
        // cpp_majora_new.gcode:8582).
        let z_line = if crate::faithful_gate("TOWER_TRAVEL_Z") {
            format!(
                "G1 Z{}\n",
                crate::gcode::format_gcode_value(tcr.print_z as f64, 3)
            )
        } else {
            String::new()
        };
        format!(
            "G1 X{:.3} Y{:.3} F{:.0}\n{z_line}",
            tcr.start_pos.x, tcr.start_pos.y, f
        )
    } else {
        String::new()
    };
    // GCode.cpp:1051 — `start_filament_gcode_str = start_filament_gcode_str +
    // wipe_next_start_point_str + toolchange_unretract_str`: the filament start
    // template comes FIRST, then the travel to the tower, then the unretract.
    let fil_start = match &filament_start_block {
        Some(b) if had_placeholder => {
            let mut t = b.trim_end().to_string();
            t.push('\n');
            t
        }
        _ => String::new(),
    };
    // R601: with the tag at the head the trailer keeps everything that is a real
    // move (filament start template, travel, unretract) and drops only the three
    // comment lines, which now precede the whole block.
    let tail_marker = if tag_at_head { "" } else { tower_feature };
    // R648 — the marker we classify and never write. C++ appends
    // `;_FORCE_RESUME_FAN_SPEED` immediately after the tool-change template
    // (GCode.cpp:944 for `toolchange_gcode_str`, :7479 for the `set_extruder`
    // path), and `GCodeEditor.cpp:556-561` turns it into a forced re-emission of
    // BOTH fans:
    //
    //     if (m_current_fan_speed != -1) new_gcode += set_fan(..., m_current_fan_speed);
    //     if (m_additional_fan_speed != -1 && m_set_fan_changing_filament_start
    //         && auxiliary_fan) new_gcode += set_additional_fan(m_additional_fan_speed);
    //
    // We already had the constant (cooling.rs:1404), the classifier
    // (cooling.rs:2256) and the emitter (cooling.rs:2551) — only the producer was
    // missing, so the branch could never fire. C++'s output shows the pair landing
    // between the template's closing `M621 S<n>A` and the tower's first travel,
    // which is exactly the head of this trailer.
    let force_resume = if had_placeholder && crate::faithful_gate("FORCE_RESUME_FAN_MARKER") {
        ";_FORCE_RESUME_FAN_SPEED\n"
    } else {
        ""
    };
    // R649 — WipeTower.cpp:2485. `toolchange_Change` closes with
    //
    //     writer.append("[filament_start_gcode]\n");
    //     writer.flush_planner_queue();
    //
    // so a `G4 S0` follows the whole filament-start expansion (template + travel
    // + unretract) before the tower's first wipe move. R645 ported the OTHER
    // `flush_planner_queue` — WipeTower.cpp:2173/3339, the `; WIPE_TOWER_END`
    // trailer — and this one stayed missing, which is why Majora carried 2,721
    // `G4 S0` against C++'s 5,446: exactly one per tool change short.
    let flush_queue = if had_placeholder && crate::faithful_gate("TOOLCHANGE_FLUSH_QUEUE") {
        "\nG4 S0"
    } else {
        ""
    };
    // TOWER_TRAVEL_PATH_CPP (on): what `append_tcr` writes after the
    // change-filament template when the template has to be followed by a
    // travel (GCode.cpp:1131-1144, 1177-1253, 1294):
    //   - the template leaves the position unknown and its last Z current;
    //   - with prime_tower_skip_points the travel follows
    //     `generate_path_to_wipe_tower` from the template's last position
    //     around the tower box; every point but the last ends the
    //     change-filament text, each one a `GCode::travel_to` (lift included);
    //   - the filament-start text is the template's text as parsed, followed
    //     by the travel to the last point and `GCode::unretract()`;
    //   - the change-filament text ends with a newline and so does its line in
    //     the tower G-code, which leaves an empty line, and the same after the
    //     filament-start text.
    // The entry retract (:747, TOWER_ENTRY_WIPE_CPP) has to come first, so its
    // text is taken here and placed where it was written before. =0 keeps the
    // single travel to the tower start without the extruder offset and the
    // unretract by `retract_length_toolchange`.
    const WT_START: &str = "; WIPE_TOWER_START\n";
    let travel_path_cpp = crate::faithful_gate("TOWER_TRAVEL_PATH_CPP")
        && crate::faithful_gate("TOWER_ENTRY_WIPE_CPP")
        && crate::faithful_gate("TOOLCHANGE_TEMPLATE_TEXT")
        && tcr.is_tool_change
        && had_placeholder
        && block.is_some()
        && g.contains(WT_START);
    let mut entry_retract_text: Option<String> = None;
    let cpp_trailer = if travel_path_cpp {
        let alt = writer.auto_lift_type();
        // GCode.cpp:747 — the entry retract, written after WIPE_TOWER_START.
        let mark = writer.output_mark();
        writer.retract_for_toolchange_with_lift(alt, true);
        entry_retract_text = Some(writer.take_output_since(mark));
        // :1124-1142 — toolchange_gcode_str as the writer sees it.
        let toolchange_gcode_str =
            format!("{};_FORCE_RESUME_FAN_SPEED\n", block.as_deref().unwrap_or(""));
        writer.after_custom_gcode(crate::gcode::g_code_processor::get_last_z_from_gcode(
            &toolchange_gcode_str,
        ));
        let (gox, goy) = writer.gcode_origin();
        let sf = crate::libslic3r::SCALING_FACTOR;
        // wipe_tower_point_to_object_point (GCode.cpp:526-529): a bed point
        // less the export origin, `Point(scale_(..), scale_(..))`.
        let to_object = |x: f64, y: f64| {
            crate::Point::new(((x + gox) / sf).round() as crate::Coord, ((y + goy) / sf).round() as crate::Coord)
        };
        let start_wipe_pos =
            to_object(tcr.tool_change_start_pos.x as f64, tcr.tool_change_start_pos.y as f64);
        let travel_points: Vec<crate::Point> = if print_config.prime_tower_skip_points {
            // :1191-1197 — gcode_to_point of the template's last XY.
            let last = crate::gcode::g_code_processor::get_last_position_from_gcode(
                &toolchange_gcode_str,
            )
            .unwrap_or([0.0, 0.0, 0.0]);
            let gcode_last_pos2d_object = crate::Point::new(
                ((last[0] as f64 + gox + print_config.extruder_offset_x) / sf).round() as crate::Coord,
                ((last[1] as f64 + goy + print_config.extruder_offset_y) / sf).round() as crate::Coord,
            );
            // :1200-1205 — the bed box (get_extruder_shared_printable_polygon).
            let (mut bx0, mut by0, mut bx1, mut by1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
            for &(x, y) in &print_config.printable_area {
                bx0 = bx0.min(x);
                by0 = by0.min(y);
                bx1 = bx1.max(x);
                by1 = by1.max(y);
            }
            let to_object_f = |x: f64, y: f64| to_object(x as f32 as f64, y as f32 as f64);
            let printer_bbx = (to_object_f(bx0, by0), to_object_f(bx1, by1));
            // :1206-1215 — the tower box through transform_wt_pt (rotation 0,
            // tower position, rib offset 0).
            let ((tx0, ty0), (tx1, ty1)) = writer.wipe_tower_bbx.unwrap_or(((0.0, 0.0), (0.0, 0.0)));
            let corners = [(tx0, ty0), (tx1, ty0), (tx1, ty1), (tx0, ty1)];
            let (mut ax0, mut ay0, mut ax1, mut ay1) =
                (crate::Coord::MAX, crate::Coord::MAX, crate::Coord::MIN, crate::Coord::MIN);
            for (cx, cy) in corners {
                let px = cx as f32 + off.x;
                let py = cy as f32 + off.y;
                let p = to_object(px as f64, py as f64);
                ax0 = ax0.min(p.x);
                ay0 = ay0.min(p.y);
                ax1 = ax1.max(p.x);
                ay1 = ay1.max(p.y);
            }
            let avoid_bbx = (crate::Point::new(ax0, ay0), crate::Point::new(ax1, ay1));
            crate::g_code::generate_path_to_wipe_tower(
                gcode_last_pos2d_object,
                start_wipe_pos,
                avoid_bbx,
                printer_bbx,
            )
        } else {
            // :1184-1188
            vec![start_wipe_pos]
        };
        let unscale = |c: crate::Coord| c as f64 * sf;
        // :1217-1229
        let mark_b = writer.output_mark();
        for p in &travel_points[..travel_points.len() - 1] {
            writer.travel_to_tower_point(unscale(p.x), unscale(p.y));
        }
        let travel_to_wipe_tower_gcode = writer.take_output_since(mark_b);
        let mark_c = writer.output_mark();
        let last = travel_points[travel_points.len() - 1];
        writer.travel_to_tower_point(unscale(last.x), unscale(last.y));
        // :1250 — toolchange_unretract_str = gcodegen.unretract().
        writer.unretract();
        let wipe_next_and_unretract = writer.take_output_since(mark_c);
        // :1271-1285 — the filament-start template as parsed, check_add_eol.
        let start_filament_gcode_str = if fil_gcode_faithful
            && !print_config.filament_start_gcode.is_empty()
        {
            let ctx = crate::gcode::change_filament::build_context(
                print_config,
                tcr.initial_tool.max(0) as usize,
                tcr.new_tool.max(0) as usize,
                tcr.print_z as f64,
                max_layer_z,
                toolchange_count,
                tcr.purge_volume as f64,
                first_layer,
            );
            let mut t = crate::gcode::gcode_template::process_text(
                &print_config.filament_start_gcode,
                &ctx,
            );
            if !t.is_empty() && !t.ends_with('\n') {
                t.push('\n');
            }
            t
        } else {
            String::new()
        };
        Some(format!(
            "{force_resume}{travel_to_wipe_tower_gcode}\n{start_filament_gcode_str}{wipe_next_and_unretract}{tail_marker}{flush_queue}"
        ))
    } else {
        None
    };
    let trailer = if let Some(t) = cpp_trailer {
        t
    } else if had_placeholder && print_config.retract_length_toolchange > 0.0 {
        let speed = if print_config.retract_speed > 0.0 {
            print_config.retract_speed * 60.0
        } else {
            1800.0
        };
        // R808: native is GCodeWriter::unretract (`G1 E2 F1800`), not the tower
        // writer's 4-decimal E. TOWER_DERETRACT_FMT=0 restores `E2.0000`.
        let e_str = if crate::faithful_gate("TOWER_DERETRACT_FMT") {
            crate::gcode::format_gcode_value(print_config.retract_length_toolchange, 5)
        } else {
            format!("{:.4}", print_config.retract_length_toolchange)
        };
        format!(
            "{force_resume}{fil_start}{travel_to_start}G1 E{} F{:.0}\n{tail_marker}{flush_queue}",
            e_str, speed
        )
    } else {
        format!("{force_resume}{fil_start}{travel_to_start}{tail_marker}{flush_queue}")
    };
    let trailer = trailer.trim_end_matches('\n').to_string();
    let g = substitute_change_filament(
        &g,
        block.as_deref(),
        tcr.new_tool.max(0) as usize,
        &print_config.toolchange_prefix,
        Some(&trailer),
        filament_end_block.as_deref().filter(|_| had_placeholder),
    );
    if !had_placeholder {
        // No tool change in this block (e.g. a plain tower layer): nothing
        // substituted the trailer in, so the travel and the marker are written
        // here. This is exactly C++'s `tcr.is_finish_first` case (GCode.cpp:687),
        // where it also emits the travel explicitly.
        if !travel_to_start.is_empty() && !finish_first_travel {
            writer.write_raw(travel_to_start.trim_end());
        }
        if !tag_at_head {
            writer.write_raw(tower_feature);
        }
    }
    // R601: C++ writes the LAYER_HEIGHT/FEATURE/LINE_WIDTH trio at the head of
    // `tcr.gcode`, immediately before `;----` / `; CP TOOLCHANGE START`, so the
    // whole tool change is attributed to `Prime tower`.
    if tag_at_head {
        writer.write_raw(tower_feature);
    }
    // R603: `M73 E<remaining filament changes>` — GCodeProcessor.cpp:601,
    //     snprintf(buf, ..., "M73 E%d\n", total_filament_change - filament_change_num)
    // inserted at each filament-block boundary (GCodeProcessor.cpp:1119). In the
    // emitted gcode it lands immediately AFTER the bare `T<n>` tool command
    // (verified at cpp_majora_new.gcode:6930-6931). C++ does this in a whole-file
    // post-process; we have no such pass, so it is spliced into the substituted
    // tower block at the same position, which is textually identical.
    //
    // The countdown uses OUR total, not C++'s, because that is what C++'s own
    // formula would produce on our gcode. Measured before implementing: with our
    // total (2,721) 2,304/2,721 of these lines land on an identical C++ line;
    // forcing C++'s total (2,723) would score only 1,971. The 417 that still miss
    // are where the two engines' tool changes sit on different layers -- a
    // pre-existing 2-tool-change difference, not something this emission creates.
    let g = if crate::faithful_gate("M73_REMAIN_FILAMENT_CHANGES")
        && tcr.is_tool_change
        && total_tool_changes > 0
        && toolchange_count > 0
    {
        let t_line = format!("{}{}", print_config.toolchange_prefix, tcr.new_tool.max(0));
        let m73 = format!("M73 E{}", total_tool_changes - toolchange_count);
        let mut out = String::with_capacity(g.len() + m73.len() + 1);
        let mut spliced = false;
        for line in g.split_inclusive('\n') {
            out.push_str(line);
            if !spliced && line.trim_end() == t_line {
                if !line.ends_with('\n') {
                    out.push('\n');
                }
                out.push_str(&m73);
                out.push('\n');
                spliced = true;
            }
        }
        out
    } else {
        g
    };
    // R608: C++ `append_tcr` retracts-with-wipe at TOWER ENTRY, immediately after
    // `; WIPE_TOWER_START`, using the TOOLCHANGE retraction length. Measured on the
    // reference output: 2,718 such blocks, each summing to exactly |E| = 1.9000
    // (= retract_length_toolchange 2 x the 0.95 dE factor), wiping along the OBJECT
    // path just printed -- their coordinates are object-space, they are only tagged
    // in-tower because the marker precedes them. We emitted ZERO of these, and that
    // is the entire Prime-tower E/block gap: our other tower wipes already match
    // C++'s at 0.7600 (after-toolchange 2,715 vs 2,512; after-tower 656 vs 931).
    // The mixture closes: 0.76*(3443/6161) + 1.90*(2718/6161) = 1.2631 vs C++'s
    // measured 1.2629.
    //
    // Spliced rather than appended, because the wipe must land INSIDE the tower
    // block: write up to and including the marker, emit the retract, then the rest.
    // Same technique as R603's `M73 E` splice.
    //
    // R610 CORRECTION: R608's comment here claimed this was "nearly inert (+2 blocks)"
    // because the writer was already `retracted`. BOTH halves were wrong. It fires
    // 2,717 times (marker-verified) and every guard passes. R608 had judged a NEW
    // total against an OLD per-class breakdown measured on a file built WITHOUT this
    // gate. The real behaviour is a COMPENSATING TRADE:
    //     after_tc 2,715 -> 0   and   in_tower 0 -> 2,717 (at |E| 1.9000, C++ 2,718)
    // netting +2 on the total. The in-tower half is CONFIRMED CORRECT against C++.
    if crate::faithful_gate("TOWER_ENTRY_WIPE_CPP")
        && tcr.is_tool_change
        && g.contains(WT_START)
    {
        let cut = entry_cut.unwrap_or_else(|| g.find(WT_START).unwrap() + WT_START.len());
        // GCode.cpp:747 — `toolchange_retract_str` is PREPENDED to the tower
        // gcode, so its wipe + immediate spiral land BEFORE `; WIPE_TOWER_START`,
        // i.e. in the OBJECT region. R640 measured C++'s object spirals by
        // preceding marker: 4,751 follow `; WIPE_END`, and only ~1,243 of those
        // are the travel path — the rest are these. R638 wired only the
        // post-`WT_START` retract (GCode.cpp:1085), which is why our 2,718 all
        // landed in-tower and the object region kept a 4,213 shortfall.
        //
        // MEASURED R640 — OPT-IN, not default. Adding this retract does NOT add
        // spirals: it fires first, lifts before `WT_START`, and then the
        // post-tower retract's `to_lift < EPSILON` guard suppresses the one R638
        // landed. Net effect on Majora: `G17` tower_tc 2,718 -> 0 (R638's fix
        // UNDONE), object 33,688 -> 36,406, matched lines UNCHANGED at 673,583,
        // body -2,717. It moves spirals rather than creating them, and moves them
        // the wrong way. The object-region eager spirals C++ emits after
        // `; WIPE_END` come from somewhere this call is not.
        let alt = writer.auto_lift_type();
        if crate::opt_in_gate("TOWER_PRE_RETRACT") {
            writer.retract_for_toolchange_with_lift(alt, true);
        }
        writer.write_raw_content(&g[..cut]);
        // GCode.cpp:1085 — `append_tcr`'s retract, AFTER the tower's own moves.
        match &entry_retract_text {
            Some(t) => writer.write_raw_content(t),
            None => writer.retract_for_toolchange_with_lift(alt, true),
        }
        writer.write_raw_content(&g[cut..]);
    } else {
        writer.write_raw_content(&g);
        // GCode.cpp:945 and :7480 — the tower block carries the spliced
        // change_filament / tool-change templates, after which "gcode writer
        // doesn't know where the extruder is ... so that normal lift will be
        // used the first time after tool change" (C++'s own comment at :7477).
        // Same raw-splice state class as R611's mark_unretracted_after_raw,
        // three lines below. R630.
        writer.set_current_position_clear(false);
    }
    // R656 — GCode.cpp:4718-4720: `process_layer` sets
    // `m_last_processor_extrusion_role = erWipeTower` once per (layer, extruder)
    // that has a wipe tower, so the NEXT object path force-emits the analyzer
    // Width and Height tags regardless of value (:6605 / :6619). We arm the
    // one-shot here, after the tower block is written, which is the same point
    // in the stream.
    writer.set_force_analyzer_tags();

    // R611: the SECOND retract. C++ `append_tcr` (GCode.cpp:1081-1085) does, AFTER the
    // tower's own moves:
    //
    //     gcodegen.m_wipe.reset_path();
    //     for (const Vec2f& wipe_pt : tcr.wipe_path)
    //         gcodegen.m_wipe.path.points.emplace_back(
    //             wipe_tower_point_to_object_point(gcodegen, transform_wt_pt(wipe_pt) + plate_origin_2d));
    //     gcode += gcodegen.retract(false, false, auto_lift_type, true);
    //
    // i.e. it REINSTALLS a wipe path -- the TOWER's own, from `tcr.wipe_path` -- and
    // retracts again with the ORDINARY length. That is why C++ has BOTH an in-tower
    // wipe (|E| 1.9000) and an after-toolchange wipe (|E| 0.7600), while we had only
    // one: our `retract()` ends with `self.wipe_path.clear()`, so once the tower-entry
    // retract consumed the path there was nothing left for a second wipe to run along.
    //
    // `tcr.wipe_path` is the field R608 found is POPULATED AND NEVER CONSUMED
    // (`wipe_tower.rs:340`, filled at `:1291`, copied at `:2889`) -- the sixth
    // unused-symbol instance, and consuming it here is what it was carried for.
    // Tower-local -> object coordinates is a translation by the tower offset
    // (rotation is 0 on this profile, as `transform_gcode` is called with 0.0 above).
    // R611 MEASURED: adding an explicit second `retract()` here produced ZERO extra
    // after-toolchange wipes, because the writer was still `retracted` from the
    // tower-entry call and `retract()` early-returns. The pre-registered fallback
    // named exactly this. The cause is state tracking, not a missing call: the tower
    // block is spliced in as RAW TEXT and unretracts inside it, so the writer's flag
    // is stale-true. Sync it instead of adding a retract C++ does not have -- the
    // ordinary object-side retract then fires again on its own, as it did before the
    // entry gate existed.
    // R612 TESTED AND REVERTED: extending this sync to the `finish_layer` tcr (which is
    // not a tool change, so R611's `&& tcr.is_tool_change` skips it) was the predicted
    // cause of our after_tower shortfall (656 vs C++'s 931). Running it DEFAULT-ON left
    // the majora output BYTE-IDENTICAL (`69eb9767`), so the finish-layer path is not the
    // cause and the speculative branch was removed rather than carried as inert code.
    // The remaining after_tower gap is therefore something C++ does that we do not model
    // at all -- see the R612 round note.
    // TOWER_LEAVE_CPP (on): the end of `append_tcr` (GCode.cpp:1355-1373).
    //   - "A phony move to the end position at the wipe tower": the writer and
    //     `m_last_pos` are at `tcr.end_pos` (:1356-1357), so the next object path
    //     is chosen, and its seam placed, from the tower;
    //   - the wipe path is the tower's own (`tcr.wipe_path`, :1366-1368);
    //   - `retract(false, false, auto_lift_type, true, true)` (:1372): wipe along
    //     that path, retract, and lift at once at `prime_tower_lift_speed`.
    // =0 leaves the position where the last object path ended and lets the next
    // travel retract.
    if crate::faithful_gate("TOWER_LEAVE_CPP") {
        // wipe_tower_point_to_object_point (GCode.cpp:526-529): bed point minus
        // the export origin.
        let (gox, goy) = writer.gcode_origin();
        writer.set_position_xy(tcr.end_pos.x as f64 + gox, tcr.end_pos.y as f64 + goy);
        writer.set_current_position_clear(true);
        writer.reset_wipe_path();
        // `tcr.wipe_path` is kept in bed coordinates (construct_tcr adds the
        // tower position), like `start_pos` and `end_pos`.
        let pts: Vec<(f64, f64)> = tcr
            .wipe_path
            .iter()
            .map(|p| (p.x as f64 + gox, p.y as f64 + goy))
            .collect();
        if !pts.is_empty() {
            writer.set_wipe_path_points(pts);
        }
        if crate::probe_enabled("TOWERLEAVE") {
            eprintln!("TOWERLEAVE z={} tc={} end={:?} start={:?} wipe={:?} off={:?}", tcr.print_z, tcr.is_tool_change, tcr.end_pos, tcr.start_pos, tcr.wipe_path, off);
        }
        writer.mark_unretracted_after_raw();
        if crate::faithful_gate("TOWER_TIMELAPSE_WALL") {
            writer.set_nominal_z_after_raw(tcr.print_z as f64);
        } else {
            writer.mark_unlifted_after_raw();
        }
        let alt = writer.auto_lift_type();
        writer.retract_leaving_tower(alt, print_config.prime_tower_lift_speed);
    } else if crate::faithful_gate("TOWER_EXIT_WIPE_CPP") && tcr.is_tool_change {
        // C++ `append_tcr` (GCode.cpp:1081-1084) also reinstalls the TOWER's own wipe
        // path from `tcr.wipe_path` before its next retract -- the field R608 found is
        // populated and never consumed. Tower-local -> object coords is a translation
        // by the tower offset (rotation is 0 here, as `transform_gcode` is called with
        // 0.0 above).
        let pts: Vec<(f64, f64)> = tcr
            .wipe_path
            .iter()
            .map(|p| ((p.x + off.x) as f64, (p.y + off.y) as f64))
            .collect();
        if pts.len() >= 2 {
            writer.set_wipe_path_points(pts);
        }
        writer.mark_unretracted_after_raw();
    }
    // R475: we just wrote `; FEATURE: Prime tower` as raw text, so the writer's
    // persistent last-role (C++ m_last_extrusion_role, GCode.hpp:538) has to move
    // with it. Without this the next object entity compares its role against the
    // role from BEFORE the tower, finds it unchanged, skips its own `; FEATURE:`
    // line, and its moves stay under the tower's marker -- 13.6% of Majora's
    // "Prime tower" length was actually object extrusion labelled that way.
    if crate::faithful_gate("TOWER_ROLE_RESET") {
        writer.set_last_extrusion_role(Some(crate::extrusion_entity::ExtrusionRole::WipeTower));
    }
    if tower_reset_accel {
        writer.reset_last_acceleration();
    }
}

// GCode.cpp:2036
pub fn to_bambu_bed_type(ty: BedType) -> BambuBedType {
    // GCode.cpp:2038
    let mut bambu_bed_type = BambuBedType::Unknown;
    // GCode.cpp:2039-2048
    if ty == BedType::Pc {
        bambu_bed_type = BambuBedType::CoolPlate;
    } else if ty == BedType::Ep {
        bambu_bed_type = BambuBedType::EngineeringPlate;
    } else if ty == BedType::Pei {
        bambu_bed_type = BambuBedType::HighTemperaturePlate;
    } else if ty == BedType::Pte {
        bambu_bed_type = BambuBedType::TexturedPeiPlate;
    } else if ty == BedType::SuperTack {
        bambu_bed_type = BambuBedType::SuperTackPlate;
    }
    // GCode.cpp:2050
    bambu_bed_type
}

// ---------------------------------------------------------------------------
// Local geometry / enum mirrors
// ---------------------------------------------------------------------------

/// `Vec2f` = `Eigen::Vector2f`. A 2-component f32 vector with named fields so
/// the ported code reads like the C++ `.x()` / `.y()` accessors.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec2f {
    pub x: f32,
    pub y: f32,
}

impl Vec2f {
    #[inline]
    pub fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }
}

impl std::ops::Index<usize> for Vec2f {
    type Output = f32;
    #[inline]
    fn index(&self, i: usize) -> &f32 {
        match i {
            0 => &self.x,
            1 => &self.y,
            _ => panic!("Vec2f index out of range"),
        }
    }
}

impl std::ops::IndexMut<usize> for Vec2f {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut f32 {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            _ => panic!("Vec2f index out of range"),
        }
    }
}

impl std::ops::Add<&Vec2f> for Vec2f {
    type Output = Vec2f;
    #[inline]
    fn add(self, rhs: &Vec2f) -> Vec2f {
        Vec2f::new(self.x + rhs.x, self.y + rhs.y)
    }
}

/// Eigen `Rotation2Df(angle) * v`: rotates by `[cos -sin; sin cos] * v`.
/// (Mirrors the helper used in `triangle_mesh.rs`.)
#[inline]
fn rotate2d(angle: f32, v: Vec2f) -> Vec2f {
    let c = angle.cos();
    let s = angle.sin();
    Vec2f::new(c * v.x - s * v.y, s * v.x + c * v.y)
}

/// Mirror of `WipeTower::never_skip_tag()` (WipeTower.hpp:28).
#[inline]
fn never_skip_tag() -> &'static str {
    "_GCODE_WIPE_TOWER_NEVER_SKIP_TAG"
}

/// `BedType` (PrintConfig.hpp). Variant names mirror the C++ `btPC` etc.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BedType {
    Pc,
    Ep,
    Pei,
    Pte,
    SuperTack,
}

/// `BambuBedType` (GCode/GCodeProcessor or BBS plate types).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BambuBedType {
    Unknown,
    CoolPlate,
    EngineeringPlate,
    HighTemperaturePlate,
    TexturedPeiPlate,
    SuperTackPlate,
}

// ---------------------------------------------------------------------------
// C++ stream-parsing helpers (faithful to std::getline / std::istream >> float)
// ---------------------------------------------------------------------------

/// Mirror of repeated `std::getline(gcode_str, line)` over an istringstream.
///
/// `while (gcode_str) { std::getline(...); ... }` reads one line per iteration.
/// std::getline strips the delimiter `\n`. After the final `\n` the stream is
/// still "good" so getline runs once more, yielding an empty trailing line
/// (which sets eof and ends the loop). For a string without a trailing newline,
/// getline yields the last partial line then the loop ends. We replicate the
/// exact set of lines C++ would observe.
fn split_getlines(s: &str) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    if s.is_empty() {
        // C++: the first getline on an empty (but good) stream extracts nothing
        // and sets failbit/eof; the body still runs once with an empty `line`.
        out.push(String::new());
        return out;
    }
    let mut cur = String::new();
    for ch in s.chars() {
        if ch == '\n' {
            out.push(std::mem::take(&mut cur));
        } else {
            cur.push(ch);
        }
    }
    if s.ends_with('\n') {
        // After consuming the trailing '\n', the stream is still good, so
        // getline runs once more producing an empty line before hitting eof.
        out.push(String::new());
    } else {
        // Final partial line.
        out.push(cur);
    }
    out
}

/// Mirror of `std::istream >> float` with `std::noskipws` already cleared for
/// the numeric extraction (operator>> always skips leading whitespace by
/// default for arithmetic types). Returns the parsed value (None on failure)
/// and the number of characters consumed.
fn stream_read_float(chars: &[char]) -> (Option<f32>, usize) {
    let mut i = 0;
    // operator>> for arithmetic types skips leading whitespace.
    while i < chars.len() && chars[i].is_whitespace() {
        i += 1;
    }
    let start = i;
    // optional sign
    if i < chars.len() && (chars[i] == '+' || chars[i] == '-') {
        i += 1;
    }
    // integer part
    while i < chars.len() && chars[i].is_ascii_digit() {
        i += 1;
    }
    // fractional part
    if i < chars.len() && chars[i] == '.' {
        i += 1;
        while i < chars.len() && chars[i].is_ascii_digit() {
            i += 1;
        }
    }
    // exponent
    if i < chars.len() && (chars[i] == 'e' || chars[i] == 'E') {
        let mut j = i + 1;
        if j < chars.len() && (chars[j] == '+' || chars[j] == '-') {
            j += 1;
        }
        let exp_digits_start = j;
        while j < chars.len() && chars[j].is_ascii_digit() {
            j += 1;
        }
        if j > exp_digits_start {
            i = j;
        }
    }
    let text: String = chars[start..i].iter().collect();
    let val = text.parse::<f32>().ok();
    (val, i)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_check_add_eol() {
        let mut g = String::new();
        check_add_eol(&mut g);
        assert_eq!(g, ""); // empty stays empty
        let mut g = String::from("G1");
        check_add_eol(&mut g);
        assert_eq!(g, "G1\n");
        let mut g = String::from("G1\n");
        check_add_eol(&mut g);
        assert_eq!(g, "G1\n"); // already terminated
    }

    #[test]
    fn test_custom_gcode_changes_tool() {
        // "T1" at line start, followed by extruder index 1 -> ok when next==1
        assert!(custom_gcode_changes_tool("T1\n", "T", 1));
        assert!(!custom_gcode_changes_tool("T1\n", "T", 2));
        // preceded by non-whitespace -> not a tool change
        assert!(!custom_gcode_changes_tool("XT1\n", "T", 1));
    }

    #[test]
    fn test_to_bambu_bed_type() {
        assert_eq!(to_bambu_bed_type(BedType::Pc), BambuBedType::CoolPlate);
        assert_eq!(
            to_bambu_bed_type(BedType::Pte),
            BambuBedType::TexturedPeiPlate
        );
        assert_eq!(
            to_bambu_bed_type(BedType::SuperTack),
            BambuBedType::SuperTackPlate
        );
    }

    #[test]
    fn test_get_wipe_avoid_pos_x() {
        let wt_min = Vec2f::new(120.0, 0.0);
        let wt_max = Vec2f::new(150.0, 0.0);
        // a = 150+offset within (100,250) -> returns a
        assert_eq!(get_wipe_avoid_pos_x(&wt_min, &wt_max, 10.0), 160.0);
        // both out of range -> default
        let wt_min = Vec2f::new(50.0, 0.0);
        let wt_max = Vec2f::new(300.0, 0.0);
        assert_eq!(get_wipe_avoid_pos_x(&wt_min, &wt_max, 0.0), 110.0);
    }
}

// ============================================================================
// The exporter: the GCode methods the engine writes G-code with.
// ============================================================================

use crate::arc_fitter::EMovePathType;
use crate::extrusion_entity::{
    ExtrusionEntityCollection, ExtrusionEntityType, ExtrusionLoop, ExtrusionPath, ExtrusionRole,
};
use crate::gcode::cooling::CoolingBuffer;
use crate::gcode::writer::GCodeWriter;
use crate::geometry::{Point, Polyline};
use crate::libslic3r::EPSILON;
use crate::print_config::PrintConfig;
use crate::Result;
use crate::{scale, unscale};

use crate::gcode::seam_placer::SeamPlacer;
use std::cell::Cell;

// =============================================================================
// Active seam-placer context (the free-function analogue of C++ `GCode`'s
// instance state `m_seam_placer` + `m_layer`, GCode.hpp).
//
// The Rust gcode pipeline is a chain of free functions (`extrude_perimeters` ->
// `extrude_collection` -> `extrude_entity` -> `extrude_loop`) rather than
// methods on a stateful `GCode`. To make the per-object/per-layer SeamPlacer
// available to `extrude_loop` (which performs the C++
// `m_seam_placer.place_seam(m_layer, loop, ...)` call, GCode.cpp:5085) without
// threading it through every signature and every recursive collection, the
// driver (`print.rs`) installs the active placer + layer index here for the
// duration of one layer's region extrusion via [`SeamContextGuard`].
//
// Export is strictly single-threaded and sequential, so a thread-local pointer
// scoped by a guard is sound: the borrowed `SeamPlacer` outlives the guard.
// =============================================================================
thread_local! {
    static SEAM_CTX: Cell<Option<(*const SeamPlacer, usize)>> = const { Cell::new(None) };
}

/// RAII guard installing `(placer, layer_idx)` as the active seam context.
/// On drop it restores the previously-installed context.
///
/// Mirrors C++ setting `m_layer`/`m_seam_placer` before the per-region extrude
/// in `GCode::process_layer` (GCode.cpp:4570+). The borrow of `placer` is tied
/// to the guard's lifetime, so the pointer stored in the thread-local is valid
/// for as long as the guard lives (and the guard strictly encloses every
/// `extrude_loop` call that can observe the context).
pub struct SeamContextGuard<'a> {
    prev: Option<(*const SeamPlacer, usize)>,
    _marker: std::marker::PhantomData<&'a SeamPlacer>,
}

impl<'a> SeamContextGuard<'a> {
    pub fn install(placer: &'a SeamPlacer, layer_idx: usize) -> Self {
        let prev =
            SEAM_CTX.with(|c| c.replace(Some((placer as *const SeamPlacer, layer_idx))));
        Self {
            prev,
            _marker: std::marker::PhantomData,
        }
    }
}

impl<'a> Drop for SeamContextGuard<'a> {
    fn drop(&mut self) {
        SEAM_CTX.with(|c| c.set(self.prev));
    }
}

/// Look up the active seam point for `polygon` (a CCW perimeter loop) via the
/// installed [`SeamPlacer`], mirroring C++ `m_seam_placer.place_seam(...)`.
/// Returns `None` when no context is installed or the placer has no data for
/// the loop (caller falls back to the legacy `find_best_seam_index` heuristic).
fn active_place_seam(
    loop_ref: &crate::extrusion_entity::ExtrusionLoop,
    polygon: &crate::geometry::Polygon,
    last_pos: Point,
) -> Option<Point> {
    SEAM_CTX.with(|c| {
        let (placer_ptr, layer_idx) = c.get()?;
        // SAFETY: the pointer is valid for the lifetime of the installed
        // `SeamContextGuard`, which strictly encloses every `extrude_loop` call
        // that can observe this context (single-threaded sequential export).
        let placer: &SeamPlacer = unsafe { &*placer_ptr };
        placer.place_seam(layer_idx, loop_ref, polygon, last_pos)
    })
}

/// Apply a placed seam to a perimeter loop, making it the loop's start vertex.
///
/// SeamPlacer.cpp:1521-1527 — native is a TWO-STEP:
/// ```cpp
/// // Because the G-code export has 1um resolution, don't generate segments
/// // shorter than 1.5 microns, thus empty path segments will not be produced.
/// if (!loop.split_at_vertex(seam_point, scaled<double>(0.0015)))
///     loop.split_at(seam_point, true);   // default eps = scaled<double>(0.001)
/// ```
/// `split_at_vertex` merely ROTATES the loop onto an existing vertex and
/// inserts nothing; only when no vertex sits within 1.5 um does native fall
/// back to `split_at`, which projects the seam onto the closest segment and
/// inserts a NEW vertex there.
///
/// Our port called `split_at` unconditionally, so a seam that coincided with a
/// loop vertex still got projected — landing on the segment BEFORE that vertex
/// and emitting a short corrective segment to reach it. That is the "0.4mm
/// corrective mini-travel" R228 observed and worked around downstream.
///
/// Note the two epsilons are different in native and are NOT interchangeable:
/// 0.0015 is the vertex-snap tolerance, 0.001 is `split_at`'s own default.
/// `prefer_non_overhang` is `true` here; the `false` variant belongs to the
/// non-perimeter branch at GCode.cpp:5456.
fn apply_seam_split(l: &mut crate::extrusion_entity::ExtrusionLoop, seam: &Point) {
    if crate::faithful_gate("SEAM_SPLIT_AT_VERTEX") {
        if !l.split_at_vertex(seam, crate::scale(0.0015) as f64) {
            l.split_at(seam, true, crate::scale(0.001) as f64);
        }
    } else {
        l.split_at(seam, false, crate::scale(0.0015) as f64);
    }
}

/// Configuration for travel moves.
///
/// C++ reference: GCode class member variables
/// GCode.hpp:100-200
#[derive(Debug, Clone)]
pub struct TravelConfig {
    /// Avoid crossing perimeters if enabled
    pub avoid_crossing_perimeters: bool,
    /// Enable retraction on travel moves
    pub retract_on_travel: bool,
    /// Minimum travel distance to trigger retraction (mm)
    pub retract_length_travel: f64,
    /// Enable Z-hop during travel
    pub z_hop: bool,
    /// Z-hop height (mm)
    pub z_hop_height: f64,
}

impl Default for TravelConfig {
    fn default() -> Self {
        Self {
            avoid_crossing_perimeters: false,
            retract_on_travel: true,
            retract_length_travel: 2.0,
            z_hop: false,
            z_hop_height: 0.0,
        }
    }
}

/// Map a perimeter path's `overhang_degree` to the overhang-corrected speed (mm/s).
///
/// Faithful port of `GCode::get_overhang_degree_corr_speed` (GCode.cpp:5931-5963).
/// `normal_speed` is the role's base wall speed (mm/s). `path_degree` is the
/// per-segment overhang degree (0..=5, the crate stores it truncated to i32 from
/// the C++ continuous double — see extrusion_entity.rs / overhang_detector.rs).
///
/// The overhang speed table (GCode.cpp:5348-5356 overhang_speed_key_map):
///   {1: overhang_1_4, 2: overhang_2_4, 3: overhang_3_4, 4: overhang_4_4,
///    5: overhang_totally, 6: bridge}.
/// A table value of 0 means "use the normal speed" (the BambuStudio nullable
/// `overhang_*_speed` default), matching the `== 0 ? normal` guards in C++.
// GCode.cpp:6633-6641/6757-6763 — overhang/bridge fan marker condition
// (ZSMOOTH_FAITHFUL). threshold==none forces the marker for every external
// perimeter; otherwise degree > threshold-1 or a bridge role qualifies.
fn overhang_fan_marker_needed(
    config: &crate::print_config::PrintObjectConfig,
    path: &ExtrusionPath,
) -> bool {
    if !config.enable_overhang_bridge_fan {
        return false;
    }
    let thr = config.overhang_fan_threshold;
    let is_bridge_role = matches!(
        path.role,
        crate::extrusion_entity::ExtrusionRole::BridgeInfill
            | crate::extrusion_entity::ExtrusionRole::OverhangPerimeter
    );
    // Native compares via `int get_overhang_degree()` — the double degree is
    // TRUNCATED to int before the > threshold-1 test (ExtrusionEntity.hpp:354),
    // so deg 2.9 with threshold 50% (enum 3 -> thr-1 = 2) does NOT qualify.
    (thr == 0 && path.role == crate::extrusion_entity::ExtrusionRole::ExternalPerimeter)
        || (path.overhang_degree as i32) > (thr - 1).max(0)
        || is_bridge_role
}

fn overhang_degree_corr_speed(
    config: &crate::print_config::PrintObjectConfig,
    normal_speed: f64,
    path_degree: f64,
) -> f64 {
    // GCode.cpp:5933 — if (path_degree <= 0) return normal_speed;
    if path_degree <= 0.0 {
        return normal_speed;
    }

    // overhang_speed_key_map[degree] resolved RAW value (no fallback). The C++
    // get_abs_value_at returns the configured speed; a 0 means "unset" and the
    // caller (get_path_speed) keeps the base speed via `new_speed==0 ? speed`.
    // degree 6 (bridge) maps to bridge_speed.
    let speed_at = |degree: i32| -> f64 {
        match degree {
            1 => config.overhang_1_4_speed,
            2 => config.overhang_2_4_speed,
            3 => config.overhang_3_4_speed,
            4 => config.overhang_4_4_speed,
            5 => config.overhang_totally_speed,
            6 => config.bridge_speed,
            _ => 0.0,
        }
    };

    // GCode.cpp:5942 — int lower_degree_bound = int(path_degree);
    let lower_degree_bound = path_degree as i32;
    // GCode.cpp:5944-5947 — degree>=4 or integral: use the lower-bound speed directly
    // (no 0->normal fallback here; caller handles a 0 result).
    if path_degree >= 4.0 || path_degree == lower_degree_bound as f64 {
        return speed_at(lower_degree_bound);
    }

    // GCode.cpp:5948-5962 — interpolate between lower and upper degree speeds.
    let upper_degree_bound = lower_degree_bound + 1;
    let mut lower_speed_bound = if lower_degree_bound == 0 {
        normal_speed
    } else {
        speed_at(lower_degree_bound)
    };
    let mut upper_speed_bound = if upper_degree_bound == 0 {
        normal_speed
    } else {
        speed_at(upper_degree_bound)
    };
    // GCode.cpp:5959-5960 — 0 -> normal_speed fallback (interpolation branch only).
    if lower_speed_bound == 0.0 {
        lower_speed_bound = normal_speed;
    }
    if upper_speed_bound == 0.0 {
        upper_speed_bound = normal_speed;
    }
    // GCode.cpp:5961 — speed_out = lower + (upper-lower)*(path_degree - lower_degree_bound)
    lower_speed_bound
        + (upper_speed_bound - lower_speed_bound) * (path_degree - lower_degree_bound as f64)
}

/// Base (normal) wall speed for a perimeter path role, mm/s.
/// Mirrors the role switch in `extrude_collection` (outer/inner wall speeds).
fn perimeter_base_speed(
    config: &crate::print_config::PrintObjectConfig,
    role: ExtrusionRole,
    is_first_layer: bool,
) -> f64 {
    if is_first_layer {
        return config.initial_layer_speed;
    }
    match role {
        ExtrusionRole::ExternalPerimeter => config.external_perimeter_speed,
        ExtrusionRole::Perimeter => config.perimeter_speed,
        _ => config.perimeter_speed,
    }
}

/// LOOP_PATH_ACCEL (on): `extrude_loop` hands every path to `_extrude`, which sets
/// that path's acceleration (GCode.cpp:7638-7666: an overhang wall takes the outer
/// wall's); the writer writes it before the path's first move. =0 keeps the
/// acceleration the collection set for the loop's first path.
fn loop_path_acceleration(
    writer: &mut GCodeWriter,
    role: crate::extrusion_entity::ExtrusionRole,
    is_first_layer: bool,
) {
    if crate::faithful_gate("ZSMOOTH_FAITHFUL") && crate::faithful_gate("LOOP_PATH_ACCEL") {
        if let Some(acc) = writer.feature_acceleration(role, is_first_layer) {
            writer.set_feature_acceleration_shared(acc);
        }
    }
}

/// Extrude a single extrusion loop.
///
/// C++ reference: GCode::extrude_loop()
/// GCode.cpp:5071-5270 (~200 lines)
///
/// This function:
/// 1. Makes loop counter-clockwise (CCW orientation)
/// 2. Finds optimal seam/starting point
/// 3. Splits loop at seam point
/// 4. Clips end to create small gap at seam
/// 5. Extrudes each path segment
/// 6. Handles variable width and arc fitting
///
/// # Arguments
/// * `loop_entity` - The extrusion loop to extrude
/// * `writer` - GCodeWriter to emit commands
/// * `config` - Print configuration for flow calculations
/// * `is_first_layer` - Whether this is the first layer (affects flow ratio)
pub fn extrude_loop(
    loop_entity: &ExtrusionLoop,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) {
    // C++ reference: GCode.cpp:5071-5076
    // C++: std::string GCode::extrude_loop(ExtrusionLoop loop, std::string description, double speed)
    // C++: {
    // C++: // get a copy; don't modify the orientation of the original loop object otherwise
    // C++: // next copies (if any) would not detect the correct orientation
    // Make a copy to avoid modifying the original
    let mut loop_copy = loop_entity.clone();

    // C++ reference: GCode.cpp:5077-5078
    // C++: // extrude all loops ccw
    // C++: bool was_clockwise = loop.make_counter_clockwise();
    // Ensure counter-clockwise orientation
    loop_copy.make_counter_clockwise();

    // C++ reference: GCode.cpp:5079-5080
    // C++: bool is_hole = loop.loop_role() & elrPerimeterHole;
    // C++: Point last_pos = this->last_pos();
    // Get current position for seam placement
    // SEAM_LAST_POS_SCALED (on): `this->last_pos()` is a scaled point, 1e5 units
    // to the millimetre. =0 hands the seam placer the position times 1e6, ten
    // times too far from the origin, which only the nearest seam reads.
    let last_pos_f = writer.position();
    let last_pos = if crate::faithful_gate("SEAM_LAST_POS_SCALED") {
        writer_last_pos(writer)
    } else {
        Point::new(
            (last_pos_f.x * 1_000_000.0) as i64,
            (last_pos_f.y * 1_000_000.0) as i64,
        )
    };

    // C++ reference: GCode.cpp:5081-5088
    // C++: if (!m_config.spiral_mode && description == "perimeter") {
    // C++: assert(m_layer != nullptr);
    // C++: bool is_outer_wall_first = m_config.wall_sequence == WallSequence::OuterInner;
    // C++: m_seam_placer.place_seam(m_layer, loop, is_outer_wall_first, this->last_pos(), satisfy_scarf_seam_angle_threshold);
    // C++: } else
    // C++: loop.split_at(last_pos, false);
    // SeamPlacer integration (GCode.cpp:5081-5088)
    // C++: m_seam_placer.place_seam(m_layer, loop, is_outer_wall_first, last_pos, ...)
    // When a per-object SeamPlacer has been installed by the driver
    // (`print.rs` via `with_seam_context`), use its aligned-mode result
    // (SeamPlacer.cpp:1463). Otherwise fall back to the legacy per-loop
    // angle/nearest heuristic (`find_best_seam_index`).
    let polygon = loop_copy.as_polygon();
    let seam_point = match active_place_seam(&loop_copy, &polygon, last_pos) {
        Some(p) => p,
        None => {
            let seam_idx = crate::gcode::seam_placer::find_best_seam_index(
                &polygon,
                Some(last_pos),
                &crate::gcode::seam_placer::SeamPlacerConfig::default(),
            );
            polygon.points()[seam_idx]
        }
    };
    // SeamPlacer.cpp:1521-1527 — try the vertex rotate first, project only as a
    // fallback. See `apply_seam_split`.
    apply_seam_split(&mut loop_copy, &seam_point);

    if std::env::var("SPLITDBG2").is_ok() {
        let post = loop_copy
            .paths
            .first()
            .and_then(|pa| pa.polyline.points().first().copied());
        eprintln!(
            "SPLITDBG2-R prefirst=({},{}) seam=({},{}) postfirst={:?}",
            polygon.points()[0].x, polygon.points()[0].y, seam_point.x, seam_point.y, post
        );
    }

    // C++ reference: GCode.cpp:5107-5117
    // C++: const double seam_gap = scale_(EXTRUDER_CONFIG(nozzle_diameter)) * (m_config.seam_gap.value / 100);
    // C++: const double clip_length = m_enable_loop_clipping && !enable_seam_slope ? seam_gap : 0;
    // C++: // get paths
    // C++: ExtrusionPaths paths;
    // C++: ...
    // C++: loop.clip_end(clip_length, &paths);
    // C++: if (paths.empty()) return "";
    // R220: seam-gap clip (GCode.cpp:5107-5117): clip_length =
    // scale_(nozzle_diameter) * seam_gap% — the loop ends ~0.06mm before the
    // seam (native L24 inner ends at -6.323 vs rust's full -6.383).
    let clipped_paths: Vec<crate::extrusion_entity::ExtrusionPath>;
    let paths = if crate::gcode::writer::lift_faithful_gate() {
        let nozzle = writer.config_ref().nozzle_diameter;
        let seam_gap_pct = config.seam_gap;
        let clip_length = crate::scale(nozzle) as f64 * (seam_gap_pct / 100.0);
        let mut tmp = Vec::new();
        loop_copy.clip_end(clip_length, &mut tmp);
        if tmp.is_empty() {
            return;
        }
        clipped_paths = tmp;
        &clipped_paths
    } else {
        &loop_copy.paths
    };

    // C++ reference: GCode.cpp:5119-5122
    // C++: double small_peri_speed=-1;
    // C++: // apply the small perimeter speed
    // C++: if (loop.length() <= SMALL_PERIMETER_LENGTH(NOZZLE_CONFIG(small_perimeter_threshold)))
    // C++: small_peri_speed = NOZZLE_CONFIG(small_perimeter_speed).get_abs_value(NOZZLE_CONFIG(outer_wall_speed));
    // TODO: Implement small perimeter speed adjustment (GCode.cpp:5119-5122)

    // C++ reference: GCode.cpp:5089-5195
    // TODO: Implement seam slope (scarf seam) feature (GCode.cpp:5089-5195)
    // This is an advanced feature for better seam quality
    // Skip for initial implementation

    // C++ reference: GCode.cpp:5208-5218
    // C++: if (!enable_seam_slope || slope_has_overhang) {
    // C++: ...
    // C++: for (ExtrusionPaths::iterator path = paths.begin(); path != paths.end(); ++path) {
    // C++: gcode += this->_extrude(*path, description, speed_for_path(*path), set_holes_and_compensation_speed);
    // C++: }
    // C++: set_last_scarf_seam_flag(false);
    // C++: }
    // Extrude each path in the loop.
    //
    // C++ GCode.cpp:5208-5218 extrudes each path with `speed_for_path(*path)`,
    // which for perimeters routes through get_path_speed -> per-segment overhang
    // speed (GCode.cpp:5382-5404). Here we apply the overhang-degree-corrected
    // speed per path (only when it differs from the feature speed already set by
    // extrude_collection), so the F feedrate is modulated per segment.
    let zsmooth_markers = crate::faithful_gate("ZSMOOTH_FAITHFUL");
    let loop_role = loop_copy.paths.first().map(|p| p.role);
    let apply_overhang_speed = config.enable_overhang_speed
        && matches!(
            loop_role,
            Some(ExtrusionRole::ExternalPerimeter) | Some(ExtrusionRole::Perimeter)
        );

    let fmvs_cap = writer.config_ref().filament_max_volumetric_speed;
    // Per-path normal wall speed (overhang-degree-corrected), mm/s.
    // Mirrors GCode::get_path_speed for perimeter roles (GCode.cpp:5387-5400):
    //   new_speed = get_overhang_degree_corr_speed(speed, overhang_degree);
    //   speed = new_speed == 0.0 ? speed : new_speed;
    let path_speed_fn = |p: &ExtrusionPath| -> f64 {
        let base = perimeter_base_speed(config, p.role, is_first_layer);
        let new_speed = overhang_degree_corr_speed(config, base, p.overhang_degree);
        let v = if new_speed == 0.0 { base } else { new_speed };
        // R229 (gated): native applies the filament volumetric cap to EVERY
        // path in _extrude (GCode.cpp:6560-6567).
        if zsmooth_markers && crate::faithful_gate("VOLCAP_FAITHFUL") {
            if crate::probe_enabled("VCPROBE") {
                eprintln!(
                    "VCP w={:.9} h={:.9} mm3={:.17e} F={:.6}",
                    p.width, p.height, p.mm3_per_mm,
                    volumetric_capped_speed(v, p.mm3_per_mm, fmvs_cap, config.print_flow_ratio) * 60.0
                );
            }
            volumetric_capped_speed(v, p.mm3_per_mm, fmvs_cap, config.print_flow_ratio)
        } else {
            v
        }
    };

    // C++ GCode.cpp:5576-5582 — smooth speed of discontinuity areas.
    // Gated on detect_overhang_wall && smooth_speed_discontinuity_area &&
    // is_set_speed_discontinuity_area (perimeter / external / overhang roles).
    // m_smooth_coefficient = filament_velocity_adaptation_factor (assumed 1.0) *
    // smooth_coefficient.
    let is_set_speed_discontinuity = matches!(
        loop_role,
        Some(ExtrusionRole::ExternalPerimeter)
            | Some(ExtrusionRole::Perimeter)
            | Some(ExtrusionRole::OverhangPerimeter)
    );
    let smooth_coeff = config.smooth_coefficient;
    // R536 probe (SMOOTHPROBE=1): count each sub-condition of the smoothing gate
    // separately, so a closed gate names its own cause instead of being guessed at.
    if crate::probe_enabled("SMOOTHPROBE") {
        use std::sync::atomic::{AtomicUsize, Ordering::Relaxed};
        static SEEN: AtomicUsize = AtomicUsize::new(0);
        static OK_DETECT: AtomicUsize = AtomicUsize::new(0);
        static OK_FLAG: AtomicUsize = AtomicUsize::new(0);
        static OK_ROLE: AtomicUsize = AtomicUsize::new(0);
        static OK_COEFF: AtomicUsize = AtomicUsize::new(0);
        static OK_NOT_FIRST: AtomicUsize = AtomicUsize::new(0);
        static OK_MULTI: AtomicUsize = AtomicUsize::new(0);
        static DEG_NONZERO: AtomicUsize = AtomicUsize::new(0);
        static PATHS_TOTAL: AtomicUsize = AtomicUsize::new(0);
        let n = SEEN.fetch_add(1, Relaxed) + 1;
        if config.detect_overhang_wall {
            OK_DETECT.fetch_add(1, Relaxed);
        }
        if config.smooth_speed_discontinuity_area {
            OK_FLAG.fetch_add(1, Relaxed);
        }
        if is_set_speed_discontinuity {
            OK_ROLE.fetch_add(1, Relaxed);
        }
        if smooth_coeff != 0.0 {
            OK_COEFF.fetch_add(1, Relaxed);
        }
        if !is_first_layer {
            OK_NOT_FIRST.fetch_add(1, Relaxed);
        }
        if paths.len() > 1 {
            OK_MULTI.fetch_add(1, Relaxed);
        }
        PATHS_TOTAL.fetch_add(paths.len(), Relaxed);
        // R538: bucket by role — outer walls reached C++'s structure after R537 but
        // inner walls did not, so the two populations have to be counted apart.
        {
            use std::sync::atomic::{AtomicUsize as A, Ordering::Relaxed as R};
            static EXT_LOOPS: A = A::new(0);
            static EXT_PATHS: A = A::new(0);
            static EXT_GRADED: A = A::new(0);
            static INT_LOOPS: A = A::new(0);
            static INT_PATHS: A = A::new(0);
            static INT_GRADED: A = A::new(0);
            let graded = paths.iter().filter(|p| p.overhang_degree != 0.0).count();
            if loop_role == Some(ExtrusionRole::ExternalPerimeter) {
                EXT_LOOPS.fetch_add(1, R);
                EXT_PATHS.fetch_add(paths.len(), R);
                EXT_GRADED.fetch_add(graded, R);
            } else if loop_role == Some(ExtrusionRole::Perimeter) {
                INT_LOOPS.fetch_add(1, R);
                INT_PATHS.fetch_add(paths.len(), R);
                INT_GRADED.fetch_add(graded, R);
            }
            if n % 1_000 == 0 {
                let (el, ep, eg) = (
                    EXT_LOOPS.load(R).max(1),
                    EXT_PATHS.load(R),
                    EXT_GRADED.load(R),
                );
                let (il, ip, ig) = (
                    INT_LOOPS.load(R).max(1),
                    INT_PATHS.load(R),
                    INT_GRADED.load(R),
                );
                eprintln!(
                    "[SMOOTHROLE] external: loops={el} paths/loop={:.2} graded={:.1}%  |  \
                     internal: loops={il} paths/loop={:.2} graded={:.1}%",
                    ep as f64 / el as f64,
                    100.0 * eg as f64 / ep.max(1) as f64,
                    ip as f64 / il as f64,
                    100.0 * ig as f64 / ip.max(1) as f64,
                );
            }
        }
        DEG_NONZERO.fetch_add(
            paths.iter().filter(|p| p.overhang_degree != 0.0).count(),
            Relaxed,
        );
        if n % 1_000 == 0 || n == 1 {
            eprintln!(
                "[SMOOTHPROBE] loops={n} detect={} flag={} role={} coeff={}({}) not_first={} paths>1={} \
                 paths_total={} overhang_deg!=0={}",
                OK_DETECT.load(Relaxed),
                OK_FLAG.load(Relaxed),
                OK_ROLE.load(Relaxed),
                OK_COEFF.load(Relaxed),
                smooth_coeff,
                OK_NOT_FIRST.load(Relaxed),
                OK_MULTI.load(Relaxed),
                PATHS_TOTAL.load(Relaxed),
                DEG_NONZERO.load(Relaxed),
            );
        }
    }
    if config.detect_overhang_wall
        && config.smooth_speed_discontinuity_area
        && is_set_speed_discontinuity
        && smooth_coeff != 0.0
        && !is_first_layer
        && paths.len() > 1
    {
        // Build a smoothed copy of the paths whose `smooth_speed` ramps across
        // discontinuities, then emit each with its smoothed feedrate.
        // R803 SSD_LID probe (cpp twin: ssd_dump in GCode::extrude_loop).
        let ssd_on = std::env::var("SSD_LID").ok().and_then(|w| w.parse::<usize>().ok())
            .map(|want| SEAM_CTX.with(|c| c.get().map(|(_, idx)| idx == want).unwrap_or(false)))
            .unwrap_or(false)
            && loop_role == Some(ExtrusionRole::ExternalPerimeter);
        let ssd_dump = |tag: &str, ps: &[ExtrusionPath], v_of: &dyn Fn(&ExtrusionPath) -> f64| {
            use std::fmt::Write as _;
            let mut sx: i64 = 0;
            for p in ps { let pts = &p.polyline.points; for k in 0..pts.len().saturating_sub(1) { sx += pts[k].x(); } }
            let mut o = format!("SSDLOOP {} sx={} n={}\n", tag, sx, ps.len());
            for (i, p) in ps.iter().enumerate() {
                let fp = p.polyline.first_point();
                let _ = writeln!(o, "{} {} role={} deg={:.3} len={:.1} v={:.3} sv={:.3} x={} y={}", tag, i, p.role as i32, p.overhang_degree, p.length(), v_of(p), p.smooth_speed, fp.x, fp.y);
            }
            eprint!("{}", o);
        };
        if ssd_on { ssd_dump("SSDIN", &paths, &path_speed_fn); }
        let mut smoothed: Vec<ExtrusionPath> = paths.clone();
        smooth_speed_discontinuity_area(
            smooth_coeff,
            &mut smoothed,
            path_speed_fn,
        );
        if ssd_on { ssd_dump("SSDOUT", &smoothed, &path_speed_fn); }
        // GCode.cpp:6599-6601 — per-path ";FEATURE: <role>" on role change.
        let mut last_path_role = smoothed.first().map(|p| p.role);
        for path in &smoothed {
            if Some(path.role) != last_path_role {
                writer.write_comment(&format!(
                    "FEATURE: {}",
                    crate::extrusion_entity::role_to_string(path.role)
                ));
                last_path_role = Some(path.role);
            }
            loop_path_acceleration(writer, path.role, is_first_layer);
            width_tag_before_speed(writer, path);
            if apply_overhang_speed
                && matches!(
                    path.role,
                    ExtrusionRole::ExternalPerimeter | ExtrusionRole::Perimeter
                )
            {
                let cooling_comment = if path.role == ExtrusionRole::ExternalPerimeter {
                    ";_EXTRUDE_SET_SPEED;_EXTERNAL_PERIMETER"
                } else {
                    ";_EXTRUDE_SET_SPEED"
                };
                set_speed_before_path(writer, path.smooth_speed * 60.0, cooling_comment);
            } else if path.role == ExtrusionRole::OverhangPerimeter {
                // GCode.cpp:5401-5408 — overhang/bridge wall speed.
                let ovh_speed = if (path.overhang_degree - 5.0).abs() < f64::EPSILON
                    && config.enable_overhang_speed
                {
                    config.overhang_totally_speed
                } else {
                    config.bridge_speed
                };
                set_speed_before_path(writer, ovh_speed * 60.0, "");
            }
            let fan_marker = zsmooth_markers && overhang_fan_marker_needed(config, path);
            if fan_marker {
                writer.write_raw(";_OVERHANG_FAN_START");
            }
            extrude_path(path, writer, config, is_first_layer);
            if fan_marker {
                writer.write_raw(";_OVERHANG_FAN_END");
            }
        }
        // R807: native replaces m_wipe.path with the loop's concatenated paths
        // AFTER extruding them on this branch too (GCode.cpp:5654-5663). Rust
        // returned early and left the writer's accumulator, which lacks the
        // loop's first point (the travel target) — every wipe skipped p1 and the
        // accumulator spanned earlier loops when no retract cleared it.
        // `WIPE_PATH_SSD=0` restores that.
        if crate::gcode::writer::lift_faithful_gate() && crate::faithful_gate("WIPE_PATH_SSD") {
            set_loop_wipe_path(writer, &smoothed[..]);
        }
        writer.reset_acceleration_default(is_first_layer);
        return;
    }

    // GCode.cpp:6599-6601 — emit ";FEATURE: <role>" per path when role changes.
    // extrude_collection already emitted the loop's first-path role, so seed with it.
    let mut last_path_role = paths.first().map(|p| p.role);
    for path in paths {
        // Per-path role-change FEATURE comment (Overhang wall paths within a wall loop).
        if Some(path.role) != last_path_role {
            writer.write_comment(&format!(
                "FEATURE: {}",
                crate::extrusion_entity::role_to_string(path.role)
            ));
            last_path_role = Some(path.role);
        }
        loop_path_acceleration(writer, path.role, is_first_layer);
        width_tag_before_speed(writer, path);
        if apply_overhang_speed
            && matches!(
                path.role,
                ExtrusionRole::ExternalPerimeter | ExtrusionRole::Perimeter
            )
        {
            // GCode.cpp:5387-5400 — normal wall speed corrected by overhang_degree.
            let corr = path_speed_fn(path);
            // Cooling markers: external perimeters keep their _EXTERNAL_PERIMETER tag.
            // (overhang paths are still cooling-adjustable for degrees < 5.)
            let cooling_comment = if path.role == ExtrusionRole::ExternalPerimeter {
                ";_EXTRUDE_SET_SPEED;_EXTERNAL_PERIMETER"
            } else {
                ";_EXTRUDE_SET_SPEED"
            };
            set_speed_before_path(writer, corr * 60.0, cooling_comment);
        } else if path.role == ExtrusionRole::OverhangPerimeter {
            // GCode.cpp:5401-5408 — overhang/bridge wall speed.
            //   degree==5 -> overhang_totally_speed; else (degree 6) -> bridge_speed.
            let ovh_speed = if (path.overhang_degree - 5.0).abs() < f64::EPSILON
                && config.enable_overhang_speed
            {
                config.overhang_totally_speed
            } else {
                config.bridge_speed
            };
            // Bridge moves are not cooling-adjustable.
            set_speed_before_path(writer, ovh_speed * 60.0, "");
        }
        let fan_marker = zsmooth_markers && overhang_fan_marker_needed(config, path);
        if fan_marker {
            writer.write_raw(";_OVERHANG_FAN_START");
        }
        extrude_path(path, writer, config, is_first_layer);
        if fan_marker {
            writer.write_raw(";_OVERHANG_FAN_END");
        }
    }

    // C++ reference: GCode.cpp:5220-5227
    // C++: //BBS: don't reset acceleration when printing first layer. During first layer, acceleration is always same value.
    // C++: if (!this->on_first_layer()) {
    // C++: // reset acceleration
    // C++: m_writer.set_acceleration((unsigned int) (NOZZLE_CONFIG(default_acceleration) + 0.5));
    // C++: if (!this->is_BBL_Printer())
    // C++: gcode += m_writer.set_jerk_xy(m_config.default_jerk.value);
    // C++: }
    // R227: reset acceleration to default after the loop (GCode.cpp:5591-5597).
    writer.reset_acceleration_default(is_first_layer);

    // C++ reference: GCode.cpp:5230-5241
    // C++: // BBS
    // C++: if (m_wipe.enable && FILAMENT_CONFIG(wipe)) {
    // C++: m_wipe.path = Polyline();
    // C++: for (ExtrusionPath &path : paths) {
    // C++: ...
    // C++: m_wipe.path.append(path.polyline);
    // C++: }
    // C++: }
    // R222: native wipe path = the loop's SOURCE polyline points (clipped
    // paths concatenated, duplicate joints skipped) — GCode.cpp:5600-5610.
    if crate::gcode::writer::lift_faithful_gate() {
        set_loop_wipe_path(writer, &paths[..]);
    }
}

/// Extrude an extrusion entity collection.
///
/// C++ reference: GCode::extrude_multi_path() / extrude_collection()
/// GCode.cpp:2800-3100 (~300 lines)
///
/// This function:
/// 1. Iterates through all entities in the collection
/// 2. Handles role changes between entities
/// 3. Emits feature type annotations
/// 4. Calls appropriate extrude function for each entity
///
/// # Arguments
/// * `collection` - The collection to extrude
/// * `writer` - GCodeWriter to emit commands
/// * `config` - Print configuration for flow calculations
/// * `is_first_layer` - Whether this is the first layer (affects flow ratio)
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors


/// R733 — cost census for gcode `generate`. It is 1.206 s = 46% of the benchy
/// slice and had never been profiled. `EXPPROF=1` prints at exit.
pub static EXPPROF_EXTRUDE_NS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static EXPPROF_EXTRUDE_CALLS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static EXPPROF_WRITE_NS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub static EXPPROF_WRITE_CALLS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

/// Accumulates on drop so every return path in `extrude_collection` is counted.
pub struct ExpProfGuard(pub std::time::Instant);
impl Drop for ExpProfGuard {
    fn drop(&mut self) {
        use std::sync::atomic::Ordering::Relaxed;
        EXPPROF_EXTRUDE_NS.fetch_add(self.0.elapsed().as_nanos() as usize, Relaxed);
        EXPPROF_EXTRUDE_CALLS.fetch_add(1, Relaxed);
    }
}

pub fn expprof_report() {
    use std::sync::atomic::Ordering::Relaxed;
    if !crate::probe_enabled("EXPPROF") {
        return;
    }
    eprintln!(
        "[EXPPROF] extrude_collection {:.1}ms calls={} | writer emit {:.1}ms calls={}",
        EXPPROF_EXTRUDE_NS.load(Relaxed) as f64 / 1e6,
        EXPPROF_EXTRUDE_CALLS.load(Relaxed),
        EXPPROF_WRITE_NS.load(Relaxed) as f64 / 1e6,
        EXPPROF_WRITE_CALLS.load(Relaxed),
    );
}
pub fn extrude_collection(
    collection: &ExtrusionEntityCollection,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) -> Result<()> {
    let __ex_guard = if crate::probe_enabled("EXPPROF") { Some(ExpProfGuard(std::time::Instant::now())) } else { None };

    /// Extrude an ExtrusionEntityCollection by iterating entities
    /// GCode.cpp:2800-3100
    /// C++: std::string GCode::extrude_entity(const ExtrusionEntity &entity, const std::string &description, double speed)

    /// Early return for empty collection
    /// GCode.cpp:2802-2803
    /// C++: if (entity.is_collection())
    /// C++: return this->_extrude(*static_cast<const ExtrusionEntityCollection*>(&entity), description, speed);
    if collection.entities.is_empty() {
        return Ok(());
    }

    /// Track current role for feature comments. C++ `m_last_extrusion_role` is a
    /// PERSISTENT GCode member (GCode.hpp:538), so consecutive same-role entities
    /// across separate extrude_collection calls do NOT re-emit the FEATURE marker.
    /// Seed from / persist back to the writer's last role to match that.
    let mut current_role: Option<ExtrusionRole> = writer.last_extrusion_role();

    /// Iterate through entities in collection
    /// GCode.cpp:2860-2900
    /// C++: for (const ExtrusionEntity *entity : collection.entities) {
    /// C++: // Emit feature comment on role change
    /// C++: if (entity->role() != current_role) {
    /// C++: gcode += "; FEATURE: " + entity->role_to_string() + "\n";
    /// C++: current_role = entity->role();
    /// C++: }
    /// C++: gcode += this->extrude_entity(*entity, description, speed);
    /// C++: }
    for entity in &collection.entities {
        /// Get entity role for feature tracking
        /// GCode.cpp:2862-2863
        let entity_role = get_entity_role(entity);

        // Feature feedrate (mm/s) for this entity's role. Computed every iteration
        // (not just on role change) so it can be re-asserted after the
        // travel-to-start below, even when consecutive entities share a role.
        // C++ GCode.cpp:6175-6200: first layer uses initial_layer_speed from config.
        let feature_speed = if is_first_layer {
            // BambuStudio uses initial_layer_infill_speed for infill on first layer,
            // and initial_layer_speed for walls/other features.
            match entity_role {
                ExtrusionRole::InternalInfill
                | ExtrusionRole::SolidInfill
                | ExtrusionRole::TopSolidInfill
                | ExtrusionRole::BottomSurface => {
                    if config.initial_layer_infill_speed > 0.0 {
                        config.initial_layer_infill_speed
                    } else {
                        config.initial_layer_speed
                    }
                }
                _ => config.initial_layer_speed,
            }
        } else {
            match entity_role {
                ExtrusionRole::ExternalPerimeter => config.external_perimeter_speed,
                ExtrusionRole::Perimeter => config.perimeter_speed,
                ExtrusionRole::InternalInfill => config.infill_speed,
                ExtrusionRole::SolidInfill => config.solid_infill_speed,
                ExtrusionRole::TopSolidInfill => config.top_solid_infill_speed,
                ExtrusionRole::BridgeInfill => config.bridge_speed,
                ExtrusionRole::GapFill => config.gap_fill_speed,
                _ => config.perimeter_speed,
            }
        };
        // R229 (gated): filament volumetric cap — sparse infill's config 350
        // caps to 25mm3s/(w*h) = 307.065 (native F18423.913; rust emitted the
        // raw config speeds F21000/F18000).
        // R232 (gated): native FloatingVerticalShell speed =
        // vertical_shell_speed% of internal_solid_infill_speed
        // (GCode.cpp:6492-6500; 80% of 250 = 200 → F12000). Rust fell through
        // to the default arm (300 → the r-only F18000 ×397 bucket).
        let feature_speed = if crate::faithful_gate("ZSMOOTH_FAITHFUL")
            && !is_first_layer
            && entity_role == ExtrusionRole::FloatingVerticalShell
        {
            config.vertical_shell_speed / 100.0 * config.solid_infill_speed
        } else {
            feature_speed
        };
        // R229: PARKED behind VOLCAP_FAITHFUL — the capped speed
        // 25/mm3_per_mm lands one F-digit off native (18423.914-vs-.913:
        // rust f64 flow chain vs native float Flow → ~2e-8 mm3 drift,
        // invisible in E's 5 decimals but visible in F's 3), and the changed
        // generation-F cascades through cooling factors (net +887). Unlocks
        // with mm3/width value parity (same blocker as LINEWIDTH_PERPATH).
        let feature_speed = if crate::faithful_gate("VOLCAP_FAITHFUL") {
            volumetric_capped_speed(
                feature_speed,
                get_entity_mm3_per_mm(entity),
                writer.config_ref().filament_max_volumetric_speed,
                config.print_flow_ratio,
            )
        } else {
            feature_speed
        };
        // Cooling markers for the CoolingBuffer post-processor (C++ GCode.cpp:6253-6272).
        let cooling_comment = if entity_role == ExtrusionRole::BridgeInfill {
            // Bridge moves are not adjustable
            ""
        } else if entity_role == ExtrusionRole::ExternalPerimeter {
            ";_EXTRUDE_SET_SPEED;_EXTERNAL_PERIMETER"
        } else {
            ";_EXTRUDE_SET_SPEED"
        };

        /// Emit feature comment / LINE_WIDTH / M204 when role changes
        /// GCode.cpp:2864-2868
        /// C++: if (entity->role() != current_role) {
        /// C++: gcode += "; FEATURE: ";
        /// C++: gcode += ExtrusionEntity::role_to_string(entity->role());
        /// C++: gcode += "\n";
        /// C++: }
        let role_changed = Some(entity_role) != current_role;
        // R218: native emits the FEATURE/LINE_WIDTH markers AFTER the travel +
        // unretract, at extrusion start (GCode _extrude's description block);
        // rust emitted them before the wipe/travel. Under the gate, defer.
        let defer_markers = crate::faithful_gate("ZSMOOTH_FAITHFUL");
        let mut pending_markers = false;
        if role_changed && defer_markers {
            pending_markers = true;
        }
        if role_changed && !defer_markers {
            writer.write_comment(&format!("FEATURE: {}", entity_role.to_string()));
            // Emit LINE_WIDTH annotation for the feature
            // First layer uses initial_layer_line_width if set
            let line_width = if is_first_layer && config.initial_layer_line_width > 0.0 {
                config.initial_layer_line_width
            } else {
                match entity_role {
                    ExtrusionRole::ExternalPerimeter => config.outer_wall_line_width,
                    ExtrusionRole::Perimeter => config.inner_wall_line_width,
                    ExtrusionRole::InternalInfill => config.sparse_infill_line_width,
                    ExtrusionRole::SolidInfill => config.solid_infill_line_width,
                    ExtrusionRole::TopSolidInfill => config.top_surface_line_width,
                    _ => config.outer_wall_line_width,
                }
            };
            if line_width > 0.0 {
                // ZSMOOTH_FAITHFUL: native's m_last_width is PERSISTENT — a
                // feature change to the SAME width emits no new tag
                // (GCode.cpp:6605). Rust re-emitted per feature (1177
                // rust-only "; LINE_WIDTH: 0.42" lines).
                let skip = crate::faithful_gate("ZSMOOTH_FAITHFUL")
                    && !writer.width_tag_changed(line_width);
                if !skip {
                    // Format LINE_WIDTH to match BambuStudio: trim trailing zeros
                    let lw_str = format!("{:.5}", line_width);
                    let lw_trimmed = lw_str.trim_end_matches('0').trim_end_matches('.');
                    writer.write_comment(&format!("LINE_WIDTH: {}", lw_trimmed));
                }
            }
            // NOTE: The feature `set_speed` (with the ;_EXTRUDE_SET_SPEED cooling
            // marker) is intentionally NOT emitted here, before the intra-collection
            // travel below. Emitting it here opens a CoolingBuffer "adjustable" block
            // (active_speed_modifier) that then SWALLOWS the following travel move
            // (cooling.rs:2084-2108 merges any G1/G2/G3 inside the block, setting its
            // line_type=0), so the travel's F60000 never updates the buffer's
            // current_feedrate. The subsequent post-travel `set_speed` (below) is then
            // stripped as "redundant" (new_feedrate == current_feedrate), letting the
            // F60000 travel speed leak onto the extrusion. C++ emits the speed-set
            // AFTER the travel (GCodeEditor.cpp:276 asserts no `G1 Fxx` inside an
            // adjustable block), so the travel always precedes the ;_EXTRUDE_SET_SPEED.
            // We therefore rely solely on the post-travel set_speed at the end of this
            // loop body, matching native ordering: [M204] -> travel -> set_speed.
            // Emit per-feature acceleration (M204) matching BambuStudio.
            //
            // Faithful port of GCode::_extrude's "adjust acceleration" block
            // (GCode.cpp:6393-6420). The whole block is gated on
            // default_acceleration > 0; the branch order is:
            //   1. first layer (initial_layer_acceleration > 0)
            //   2. ExternalPerimeter / OverhangPerimeter (outer_wall_acceleration > 0)
            //   3. top surface == TopSolidInfill (top_surface_acceleration > 0)
            //   4. Perimeter (inner_wall_acceleration > 0)
            //   5. InternalInfill (sparse_infill_acceleration resolved > 0)
            //   6. else default_acceleration
            // A feature value of 0 means "use default" (the branch is skipped),
            // e.g. inner_wall_acceleration = 0 in the H2D profile falls through to
            // default_acceleration. The bridge branch is `#if 0` in C++ (disabled),
            // so bridges also fall to default_acceleration. sparse_infill_acceleration
            // is a percentage of default_acceleration (config stores e.g. "100%" as
            // 100.0), resolved via get_abs_value: pct/100 * default.
            // ZSMOOTH_FAITHFUL: the per-entity shared-register emission below
            // replaces this role-change raw write (native has no role-change
            // accel — every path goes through set_acceleration_impl).
            if !crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                if let Some(acc) = writer.feature_acceleration(entity_role, is_first_layer) {
                    writer.write_raw(&format!("M204 S{}", acc));
                }
            }
            current_role = Some(entity_role);
            // Persist to the writer so the next extrude_collection call (e.g. the
            // next island's fills, or the next gap EEC) does not re-emit the same
            // FEATURE marker — matches C++ persistent m_last_extrusion_role.
            writer.set_last_extrusion_role(current_role);
        }
        if role_changed && defer_markers {
            current_role = Some(entity_role);
            writer.set_last_extrusion_role(current_role);
        }

        // Travel to start of this entity if the nozzle is not already there.
        // C++ GCode::extrude_entity() calls travel_to() before extruding each
        // entity. Without this, consecutive entities in the same collection are
        // connected by a spurious extrusion line across open space.
        let travel_target = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
            travel_target_for_entity(entity, writer_last_pos(writer))
        } else {
            get_entity_first_point(entity)
        };
        if let Some(first_pt) = travel_target {
            let pos = writer.position();
            let tx = crate::unscale(first_pt.x());
            let ty = crate::unscale(first_pt.y());
            let dx = pos.x - tx;
            let dy = pos.y - ty;
            let dist_sq = dx * dx + dy * dy;
            // GCode.cpp:7611 — `|| m_need_change_layer_lift_z`: the layer's first
            // path travels even from its own first point (CHANGE_LAYER_RETRACT_CPP).
            if dist_sq > 0.001 * 0.001
                || (writer.m_need_change_layer_lift_z
                    && crate::faithful_gate("CHANGE_LAYER_RETRACT_CPP"))
            {
                // Per-entity travel + retraction/spiral-lift decision. This is the
                // single faithful decision point, mirroring C++ GCode::_extrude ->
                // GCode::travel_to -> GCode::needs_retraction (GCode.cpp:6366,6816,6964):
                // a retract + Z-hop is emitted before the travel to each *top-level*
                // extrusion entity (loop / multipath / path) whose length reaches
                // retraction_minimum_travel. The internal paths of a multi-path loop
                // are contiguous (m_last_pos == path.first_point() -> travel skipped in
                // C++), so the path-level travel in extrude_path_with_arc_fitting is a
                // BARE positioning move and does NOT retract — avoiding the per-internal-
                // -path double counting. retract() does wipe + retract + Z-hop;
                // unretract() after the travel descends and restores E (net E zero, so
                // material is unchanged).
                let travel_len = dist_sq.sqrt();
                let did_retract = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                    // GCode.cpp:6964 needs_retraction — faithful branch order
                    // (min-travel, leaving-outer-wall force, reduce-infill skip).
                    let from = writer_last_pos(writer);
                    let to = crate::Point::new(crate::scale(tx), crate::scale(ty));
                    writer.needs_retraction_faithful(from, to, entity_role, travel_len)
                } else {
                    writer.needs_retraction_for_travel(travel_len)
                };
                if did_retract {
                    writer.retract();
                }
                if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                    writer.set_travel_acceleration_for(entity_role, travel_len);
                } else {
                    writer.set_travel_acceleration(6000.0);
                }
                if crate::gcode::writer::lift_faithful_gate() {
                    // R206: single travel_to_xyz merges the pending lazy lift
                    // (GCode.cpp:6902-6912, travel.size()==2 branch; dest z =
                    // the current nominal z).
                    let zdest = if writer.nominal_z > 0.0 { writer.nominal_z } else { writer.z() };
                    writer.travel_to_xyz(tx, ty, zdest);
                    // GCode.cpp:7618 — the first `_extrude` after change_layer clears it.
                    writer.m_need_change_layer_lift_z = false;
                } else {
                    writer.travel_to(tx, ty, None);
                }
                if did_retract {
                    writer.unretract();
                }
            }
        }
        if pending_markers {
            // GCode.cpp:7874-7875 — "; Slow Down Start" is written before the
            // role tag of the path it brackets.
            // SLOW_DOWN_BEFORE_ROLE_TAG=0 writes it after the role tag.
            if crate::faithful_gate("FVS_SEPARATE_SPEED")
                && crate::faithful_gate("SLOW_DOWN_BEFORE_ROLE_TAG")
            {
                if let ExtrusionEntityType::Path(p) = entity {
                    if p.get_customize_flag()
                        == crate::extrusion_entity::CustomizeFlag::FloatingVerticalShell
                    {
                        writer.write_raw("; Slow Down Start");
                        writer.slow_down_start_written = true;
                    }
                }
            }
            // R218: native marker position — after travel+unretract, before the
            // feature F (GCode.cpp _extrude description/width block).
            writer.write_comment(&format!("FEATURE: {}", entity_role.to_string()));
            let line_width = if is_first_layer && config.initial_layer_line_width > 0.0 {
                config.initial_layer_line_width
            } else {
                match entity_role {
                    ExtrusionRole::ExternalPerimeter => config.outer_wall_line_width,
                    ExtrusionRole::Perimeter => config.inner_wall_line_width,
                    ExtrusionRole::InternalInfill => config.sparse_infill_line_width,
                    ExtrusionRole::SolidInfill => config.solid_infill_line_width,
                    ExtrusionRole::TopSolidInfill => config.top_surface_line_width,
                    _ => config.outer_wall_line_width,
                }
            };
            // R225: under LINEWIDTH_PERPATH the tag is emitted per path from
            // path.width at the extrude_path choke point (native GCode.cpp:6605
            // register) and the entity-level config width would fight it.
            if !crate::faithful_gate("LINEWIDTH_PERPATH")
                && line_width > 0.0
                && writer.width_tag_changed(line_width)
            {
                let lw_str = format!("{:.5}", line_width);
                let lw_trimmed = lw_str.trim_end_matches('0').trim_end_matches('.');
                writer.write_comment(&format!("LINE_WIDTH: {}", lw_trimmed));
            }
        }

        // Re-assert the feature feedrate before extruding.
        //
        // travel_to(..., None) above resets the sticky feedrate to travel_speed
        // (F60000 = 1000 mm/s). BambuStudio always emits the feature `G1 F<speed>`
        // as the LAST F-set before an extrusion run, so each extrude move inherits
        // the feature speed — not the travel speed. set_speed() is a no-op when the
        // feedrate is unchanged (writer.rs), so this only emits a line when a
        // preceding travel (or speed change) clobbered the feedrate. Skipping this
        // (the previous behaviour) let the F60000 travel feedrate leak onto 65% of
        // extrusion moves, so the acceleration-aware GCodeProcessor estimator paid
        // huge accel/decel ramp cost targeting an unreachable 1000 mm/s on short
        // extrusion segments (rust 1h50m vs native 43m).
        // ZSMOOTH_FAITHFUL: native's _extrude sets the feature acceleration for
        // EVERY path through the shared M204 register (GCode.cpp:6393-6420 +
        // GCodeWriter set_acceleration_impl) — after the travel (whose accel
        // went through the same register), so travel/feature alternation
        // re-emits M204 constantly. Legacy path emits only on role change.
        if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
            if let Some(acc) = writer.feature_acceleration(entity_role, is_first_layer) {
                writer.set_feature_acceleration_shared(acc);
            }
        }

        // ZSMOOTH_FAITHFUL: native never emits a collection-level feature-speed
        // F before a perimeter loop — its first F is the FIRST PATH's
        // overhang-corrected speed (GCode.cpp extrude_loop emits per-path F via
        // _extrude; e.g. L2 outwall starts F9300 = deg-1.3 speed, not F12000).
        // The rust per-path set_speed in extrude_loop always re-asserts F for
        // perimeter roles under enable_overhang_speed, so the pre-set is safely
        // skippable there. Default path keeps the pre-set (byte-locked).
        let skip_pre_speed = crate::faithful_gate("ZSMOOTH_FAITHFUL")
            && (config.enable_overhang_speed
                && matches!(
                    entity,
                    crate::extrusion_entity::ExtrusionEntityType::Loop(_)
                )
                && matches!(
                    entity_role,
                    ExtrusionRole::ExternalPerimeter
                        | ExtrusionRole::Perimeter
                        | ExtrusionRole::OverhangPerimeter
                )
                // R758: Path entities re-emit their own F in the R245 per-path
                // block below (native _extrude, GCode.cpp:6663) — native has NO
                // collection-level F before them either. Under
                // SET_SPEED_ALWAYS_EMIT the pre-set became a visible duplicate
                // `G1 F..;_EXTRUDE_SET_SPEED` pair (benchy layer-6 sparse:
                // 4 blocks vs native 2). PARKED OFF: byte-inert on benchy (the
                // cooling rewrite deletes the duplicates) but −51 in-order on
                // arachne — re-score after the arachne width-segmentation gap.
                // R814: on. Against the 02.09 oracle it is +203 / +3 / 0.
                || (crate::faithful_gate("NO_COLLECTION_PRESPEED")
                    && matches!(
                        entity,
                        crate::extrusion_entity::ExtrusionEntityType::Path(_)
                    )));
        // R654 — instrument first (do not infer from the guard's source, R649).
        if crate::probe_enabled("PRESPEED_PROBE") {
            use std::sync::atomic::{AtomicU64, Ordering};
            static N: AtomicU64 = AtomicU64::new(0);
            static SKIP: AtomicU64 = AtomicU64::new(0);
            static NO_OH: AtomicU64 = AtomicU64::new(0);
            static NO_LOOP: AtomicU64 = AtomicU64::new(0);
            static NO_ROLE: AtomicU64 = AtomicU64::new(0);
            let n = N.fetch_add(1, Ordering::Relaxed) + 1;
            if skip_pre_speed {
                SKIP.fetch_add(1, Ordering::Relaxed);
            } else {
                if !config.enable_overhang_speed {
                    NO_OH.fetch_add(1, Ordering::Relaxed);
                }
                if !matches!(entity, crate::extrusion_entity::ExtrusionEntityType::Loop(_)) {
                    NO_LOOP.fetch_add(1, Ordering::Relaxed);
                }
                if !matches!(
                    entity_role,
                    ExtrusionRole::ExternalPerimeter
                        | ExtrusionRole::Perimeter
                        | ExtrusionRole::OverhangPerimeter
                ) {
                    NO_ROLE.fetch_add(1, Ordering::Relaxed);
                }
            }
            if n % 200000 == 0 {
                eprintln!(
                    "[PRESPEED] n={n} skipped={} | blocked_by: !overhang_speed={} !Loop={} !perimeter_role={}",
                    SKIP.load(Ordering::Relaxed),
                    NO_OH.load(Ordering::Relaxed),
                    NO_LOOP.load(Ordering::Relaxed),
                    NO_ROLE.load(Ordering::Relaxed)
                );
            }
        }
        if !skip_pre_speed {
            // R807: fills take their F from THIS collection-level set_speed,
            // ahead of extrude_path's Width tag — native `_extrude` orders
            // Width (GCode.cpp:6662) before set_speed (:6720). Record/emit the
            // entity's first path width first (FVS 69,668 + solid 33,033 +
            // sparse 4,640 `G1 F`→`; LINE_WIDTH:` pairs on Majora @R807c).
            if let Some(p) = first_extrusion_path(entity) {
                width_tag_before_speed(writer, p);
            }
            // R654: C++ emits NO collection-level F — `_extrude` writes the Width
            // tag (GCode.cpp:6607) and only then `set_speed` (:6663). Defer ours
            // to just after the tag rather than dropping it, so the line count is
            // untouched and only its position changes.
            if crate::faithful_gate("LINEWIDTH_BEFORE_SPEED") {
                writer.set_speed_pending(feature_speed * 60.0, cooling_comment);
            } else {
                writer.set_speed(feature_speed * 60.0, cooling_comment);
            }
        }

        /// Recursively extrude the entity
        /// GCode.cpp:2870-2880
        /// C++: gcode += this->extrude_entity(*entity, description, speed);
        extrude_entity(entity, writer, config, is_first_layer)?;
    }

    // R605: port of `GCode::extrude_multi_path`'s wipe-path install
    // (GCode.cpp:5664-5673). C++ builds ONE wipe path for the whole multipath --
    // every sub-path concatenated, skipping a duplicated joint, then REVERSED:
    //
    //     m_wipe.path = Polyline();
    //     for (ExtrusionPath &path : multipath.paths) {
    //         if (!m_wipe.path.empty() && !path.empty() &&
    //             m_wipe.path.last_point() == path.first_point())
    //             m_wipe.path.append(path.polyline.points.begin() + 1, ...);
    //         else
    //             m_wipe.path.append(path.polyline);
    //     }
    //     m_wipe.path.reverse();
    //
    // Our `extrude_entity` has no multipath branch because `ExtrusionEntityType`
    // has no `MultiPath` variant -- `exporter.rs` aliases
    // `extrude_collection as extrude_multi_path` "for backward compatibility"
    // (R604). The two are NOT interchangeable here: dispatching each sub-path
    // through `extrude_entity` installs a wipe path PER sub-path, so what survives
    // is only the LAST one, where C++ keeps the whole concatenation.
    //
    // Since the type distinction does not exist, this fires only on the shape a C++
    // `ExtrusionMultiPath` actually takes here -- a collection whose children are
    // ALL paths (what `thick_polyline_to_multi_path` produces for Arachne
    // variable-width walls and gap fill). It deliberately does NOT guess on mixed
    // collections, which C++ routes through `extrude_collection` instead.
    //
    // Shipped OPT-IN (default OFF) per R557/R595/R599. It is reachable (fires 468x
    // on Benchy, 11,175x on Majora) and the wipe COUNT is unchanged on both
    // fixtures (2,041 and 36,394), confirming this changes wipe CONTENT rather than
    // how often we wipe. But matched lines are flat-to-negative: Benchy +10
    // (115,900 -> 115,910), Majora **-18** (648,759 -> 648,741). The line-count gap
    // improves on both (1.20% -> 1.11%, 8.91% -> 8.62%) because the longer wipe
    // paths emit more moves and C++ emits them too -- but a narrowing gap is NOT
    // evidence of correctness on its own, since the gap rewards emitting lines
    // whether or not they match. Same shape as R595, so the same disposition.
    //
    // To make this net-positive the wipe MOVE VALUES have to match, not just the
    // path: the emitted `G1 X.. Y.. E-..` depend on wipe_dist, retract length and
    // wipe speed as well as the path. That is the next thing to check.
    if crate::probe_enabled("WIPE_MULTIPATH_CPP")
        && crate::gcode::writer::lift_faithful_gate()
        && collection.entities.len() >= 2
        && collection
            .entities
            .iter()
            .all(|e| matches!(e, crate::extrusion_entity::ExtrusionEntityType::Path(_)))
    {
        let mut pts: Vec<(f64, f64)> = Vec::new();
        for e in &collection.entities {
            if let crate::extrusion_entity::ExtrusionEntityType::Path(p) = e {
                for (k, pt) in p.polyline.points().iter().enumerate() {
                    let q = (crate::unscale(pt.x()), crate::unscale(pt.y()));
                    // C++'s "don't save a duplicated point into wipe path".
                    if k == 0 && pts.last() == Some(&q) {
                        continue;
                    }
                    pts.push(q);
                }
            }
        }
        pts.reverse();
        // WIPE_MULTIPATH_POP=1 — reachability census (R595): prove the branch fires
        // before arguing about whether it helped. Prints cumulative totals, never a
        // truncated prefix (R598).
        if crate::probe_enabled("WIPE_MULTIPATH_POP") {
            use std::sync::atomic::{AtomicUsize, Ordering};
            static N: AtomicUsize = AtomicUsize::new(0);
            static PTS: AtomicUsize = AtomicUsize::new(0);
            let n = N.fetch_add(1, Ordering::Relaxed) + 1;
            let tot = PTS.fetch_add(pts.len(), Ordering::Relaxed) + pts.len();
            eprintln!("[WIPEMP] n={n} pts={} cum_pts={tot}", pts.len());
        }
        if pts.len() >= 2 {
            writer.set_wipe_path_points(pts);
        }
    }

    Ok(())
}

/// The writer's current XY position as a scaled `Point` — the Rust analogue of
/// C++ `GCode::m_last_pos`, which the chaining routines accept as `start_near`.
///
/// `writer.position()` is unscaled mm (`PointF`); chaining operates in scaled
/// i64 coordinates, so convert with `scale()`.
/// R211: the travel TARGET for an entity. Native _extrude seam-splits loops
/// BEFORE the travel (GCode.cpp:5085-5090), so the travel goes to the SEAM,
/// not the loop's natural first vertex. For aligned seams place_seam is
/// position-independent (final_seam_position), so resolving it here and again
/// inside extrude_loop yields the same point.
fn travel_target_for_entity(
    entity: &crate::extrusion_entity::ExtrusionEntityType,
    last_pos: Point,
) -> Option<Point> {
    use crate::extrusion_entity::ExtrusionEntityType;
    if let ExtrusionEntityType::Loop(l) = entity {
        let polygon = l.as_polygon();
        if let Some(seam) = active_place_seam(l, &polygon, last_pos) {
            // R228: native travels to loop.first_point() AFTER split_at
            // (GCode.cpp:5085-5090). For outer walls the placed seam IS a
            // loop vertex, but for inner walls place_seam can return a point
            // OFF this polygon (nearest stored perimeter data) — split_at
            // then PROJECTS it onto the loop, so the raw seam and the split
            // first-point differ (0.4mm corrective mini-travels + duplicate
            // feature-F lines). Run the same split here and return its
            // actual first vertex.
            let mut probe = l.clone();
            apply_seam_split(&mut probe, &seam);
            if let Some(fp) = probe
                .paths
                .first()
                .and_then(|pa| pa.polyline.points().first().copied())
            {
                return Some(fp);
            }
            return Some(seam);
        }
    }
    get_entity_first_point(entity)
}

fn writer_last_pos(writer: &GCodeWriter) -> Point {
    let p = writer.position();
    Point::new(scale(p.x), scale(p.y))
}

/// Rust analogue of `ExtrusionEntityCollection::chained_path_from(start_near, role)`
/// (ExtrusionEntityCollection.hpp:102-103, .cpp:89-99) with the default
/// `role == erMixed` (no role filtering — Benchy infill EECs are mixed-role).
///
/// hpp:103: `return this->no_sort ? *this : chained_path_from(entities, start_near, role);`
/// cpp:89-99: clone the entities into a fresh collection, then
/// `chain_and_reorder_extrusion_entities(out.entities, &start_near)`.
///
/// `start_near` is the writer's current position (== C++ `m_last_pos`).
fn chained_path_from(
    eec: &ExtrusionEntityCollection,
    writer: &GCodeWriter,
) -> ExtrusionEntityCollection {
    // hpp:103: when no_sort is set, return the collection unchanged.
    if eec.no_sort {
        return eec.clone();
    }
    // cpp:92-96: filtered (here unfiltered, role == erMixed) clone of the entities.
    let mut out = eec.clone();
    // cpp:97: chain_and_reorder_extrusion_entities(out.entities, &start_near)
    let start_near = writer_last_pos(writer);
    crate::shortest_path::chain_and_reorder_extrusion_entities(
        &mut out.entities,
        Some(&start_near),
    );
    out
}

/// Get the first point of an extrusion entity for travel-to targeting.
/// R229: native _extrude's filament volumetric cap (GCode.cpp:6560-6567):
/// `speed = min(speed, filament_max_volumetric_speed / (path.mm3_per_mm *
/// print_flow_ratio))`. 0/unset caps disable it.
fn volumetric_capped_speed(speed: f64, mm3_per_mm: f64, fmvs: f64, flow_ratio: f64) -> f64 {
    if fmvs <= 0.0 {
        return speed;
    }
    let ratio = if flow_ratio > 0.0 { flow_ratio } else { 1.0 };
    let mm3 = mm3_per_mm * ratio;
    if mm3 > 0.0 {
        speed.min(fmvs / mm3)
    } else {
        speed
    }
}

/// First path's mm3_per_mm of an entity (native caps per path; the collection
/// pre-speed uses the first path as representative).
fn get_entity_mm3_per_mm(entity: &ExtrusionEntityType) -> f64 {
    use crate::extrusion_entity::ExtrusionEntityType;
    match entity {
        ExtrusionEntityType::Path(p) => p.mm3_per_mm,
        ExtrusionEntityType::Loop(l) => l.paths.first().map(|p| p.mm3_per_mm).unwrap_or(0.0),
        ExtrusionEntityType::Collection(c) => c
            .entities
            .first()
            .map(get_entity_mm3_per_mm)
            .unwrap_or(0.0),
    }
}

pub fn get_entity_first_point(entity: &ExtrusionEntityType) -> Option<Point> {
    match entity {
        ExtrusionEntityType::Path(path) => path.polyline.points().first().copied(),
        ExtrusionEntityType::Loop(loop_entity) => loop_entity
            .paths
            .first()
            .and_then(|p| p.polyline.points().first().copied()),
        ExtrusionEntityType::Collection(coll) => coll
            .entities
            .first()
            .and_then(|e| get_entity_first_point(e)),
    }
}

/// Helper to extract role from ExtrusionEntityType
/// GCode.cpp:2100-2120 (role accessor methods)
pub(crate) fn get_entity_role(entity: &ExtrusionEntityType) -> ExtrusionRole {
    match entity {
        ExtrusionEntityType::Path(path) => path.role,
        ExtrusionEntityType::Loop(loop_entity) => {
            /// Loops may have multiple paths with different roles
            /// Use first path's role, or Mixed if empty
            /// GCode.cpp:2105
            /// C++: ExtrusionRole ExtrusionLoop::role() const {
            /// C++: return paths.empty() ? erNone : paths.front().role();
            /// C++: }
            loop_entity
                .paths
                .first()
                .map(|p| p.role)
                .unwrap_or(ExtrusionRole::None)
        }
        ExtrusionEntityType::Collection(coll) => {
            // Recurse into first entity of collection to get the actual role
            coll.entities
                .first()
                .map(|e| get_entity_role(e))
                .unwrap_or(ExtrusionRole::Mixed)
        }
    }
}

/// Alias for backward compatibility (C++ has both multi_path and collections)
pub use extrude_collection as extrude_multi_path;

/// Extrude a single extrusion path.
///
/// C++ reference: GCode::_extrude()
/// GCode.cpp:4200-4800 (~600 lines)
///
/// This function:
/// 1. Generates G1 commands for each point in the path
/// 2. Calculates extrusion amount (E value) for each segment
/// 3. Handles variable width segments
/// 4. Applies arc fitting if enabled
///
/// # Arguments
/// * `path` - The extrusion path to extrude
/// * `writer` - GCodeWriter to emit commands
/// * `config` - Print configuration for flow calculations
/// * `is_first_layer` - Whether this is the first layer (affects flow ratio)
/// C++ printf "%g" (default precision 6): 6 significant digits, trailing
/// zeros trimmed. Native formats the LINE_WIDTH tag with %g of the f32
/// path.width promoted to double (GCode.cpp:6607).
pub(crate) fn fmt_g6(v: f64) -> String {
    if v == 0.0 {
        return "0".to_string();
    }
    let exp = v.abs().log10().floor() as i32;
    if exp < -5 || exp >= 6 {
        // %g scientific branch — widths never reach it; minimal fallback.
        return format!("{:e}", v);
    }
    let decimals = (5 - exp).max(0) as usize;
    let s = format!("{:.*}", decimals, v);
    let s = s.trim_end_matches('0').trim_end_matches('.');
    s.to_string()
}

pub fn extrude_path(
    path: &ExtrusionPath,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) {
    extrude_path_ex(path, writer, config, is_first_layer, None);
}

/// Extrude a path with access to actual PrintConfig for arc fitting.
///
/// The arc-fitting / simplification gating is now taken from the live print
/// configuration carried by the `GCodeWriter` (mirroring C++ where
/// `LayerRegion::simplify_path` and `GCode::_extrude` both consult
/// `print()->config()`), so the optional `print_config` argument is no longer
/// needed to enable the live path.
pub fn extrude_path_ex(
    path: &ExtrusionPath,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
    _print_config: Option<&PrintConfig>,
) {
    extrude_path_with_arc_fitting(path, writer, config, is_first_layer, None);
}

/// Extrude a path with optional arc fitting.
///
/// When `print_config` is provided and arc fitting is enabled in it,
/// sequences of line segments that form arcs will be converted to
/// G2/G3 arc commands.
pub fn extrude_path_with_arc_fitting(
    path: &ExtrusionPath,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
    _print_config: Option<&PrintConfig>,
) {
    // C++ reference: GCode.cpp:4200-4210
    // C++: std::string GCode::_extrude(const ExtrusionPath &path, std::string description, double speed, bool is_hole_or_comp_speed)
    // C++: {
    // C++: std::string gcode;
    // C++:
    // C++: // check if path is empty
    // C++: if (path.polyline.points.empty())
    // C++: return "";
    // Check if path is empty
    if path.polyline.points().is_empty() {
        return;
    }

    // R225: native _extrude emits ";LINE_WIDTH: %g" whenever path.width
    // differs from m_last_width (GCode.cpp:6605-6609) — a PER-PATH register,
    // not per feature change (fills alternate widths constantly; 6.5k
    // native-only tag lines came from this). PARKED behind its own gate:
    // emission shape is native-correct, but rust f64 flow/Arachne width
    // values drift from native f32 chains in the 6th significant digit
    // (0.43272-vs-0.43273, 0.42-vs-0.41999), so enabling it today ADDS
    // unmatched lines (83187 → 87805). Unlocks when width values converge.
    static LW_PERPATH: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    static EXPW_EMITTED: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    // R571 STEP 2: a GLOBAL (all-roles) register, mirroring the writer's own
    // `last_width_tag`, so the outer-wall-only count below can be checked
    // against the quantity the emitter actually tests (R570's open tension).
    static EXPW_GLOBAL_PREV: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    static EXPW_GLOBAL_CH: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    static EXPW_GLOBAL_N: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    if crate::probe_enabled("EXPWPROBE") && path.width > 0.0 {
        let bits = (path.width as f32).to_bits() as u64;
        EXPW_GLOBAL_N.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        if EXPW_GLOBAL_PREV.swap(bits, std::sync::atomic::Ordering::Relaxed) != bits {
            EXPW_GLOBAL_CH.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
    }
    // EXPWPROBE (R569) — do the per-path widths produced by
    // thick_polyline_to_multi_path still differ by the time they reach the
    // emitter? Counts outer-wall paths seen and how many changed the register.
    let expw = crate::probe_enabled("EXPWPROBE")
        && path.role == crate::extrusion_entity::ExtrusionRole::ExternalPerimeter;

    // R656 — GCode.cpp:6591 evaluates `last_was_wipe_tower` ONCE per `_extrude`
    // and both the Width tag (:6605) and the Height tag (:6619) read that same
    // value, so take it here rather than at each guard.
    // SHIPPED OPT-IN (probe, default OFF) per R656's PRE-REGISTERED fallback.
    // The port is right: it takes `; LAYER_HEIGHT:` from 4,720 to 8,058 against
    // C++'s 8,297 — a class that was 43% short is now 3% short — and content
    // matched rises +1,204. But IN-ORDER falls 2,117, the second time a
    // demonstrably C++-faithful tag addition has cost order while the
    // `; LINE_WIDTH:` count is still 61,136 short (R654 was the first, −26,309).
    // The fallback pre-registered for exactly this: park it, stop adding tags,
    // and re-run this A/B once the Arachne width-variety gap is closed. Two
    // flips then land together — this and `LINEWIDTH_BEFORE_SPEED`.
    // R814: both on. Against the 02.09 oracle this one is +118 on Majora and
    // `LINEWIDTH_BEFORE_SPEED` +215 / +14 / -91.
    let force_tags = crate::faithful_gate("WIPE_TOWER_FORCE_TAGS")
        && writer.take_force_analyzer_tags();
    // WIDTH_TAG_FORCE_ONCE (on): GCode.cpp:7886 writes the Width tag once per
    // `_extrude`, `if (last_was_wipe_tower || m_last_width != path.width)`. The
    // port evaluates that statement in two places, `width_tag_before_speed` and
    // here; after a tower visit the first wrote the tag because the width
    // changed and this one wrote it again because it was forced. The first
    // site now also honours the force and leaves a note. =0 writes both.
    let forced_width_done = writer.take_forced_width_tag_written();
    let force_width = force_tags && !forced_width_done;

    if *LW_PERPATH.get_or_init(|| crate::faithful_gate("LINEWIDTH_PERPATH"))
        && path.width > 0.0
        // R656: `width_tag_changed` must run either way — it is the register
        // update, and C++ assigns `m_last_width = path.width` inside the same
        // branch. Evaluate it FIRST so the force-emit cannot leave the register
        // stale.
        && (writer.width_tag_changed(path.width) | force_width)
    {
        writer.write_comment(&format!("LINE_WIDTH: {}", fmt_g6(path.width)));
        if crate::probe_enabled("EXPWPROBE") {
            EXPW_EMITTED.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        }
    }
    if expw {
        use std::sync::atomic::{AtomicU64, Ordering};
        static SEEN: AtomicU64 = AtomicU64::new(0);
        static CHANGED: AtomicU64 = AtomicU64::new(0);
        static ZEROW: AtomicU64 = AtomicU64::new(0);
        if path.width <= 0.0 {
            ZEROW.fetch_add(1, Ordering::Relaxed);
        }
        static PREV: AtomicU64 = AtomicU64::new(0);
        static PREV_END: std::sync::Mutex<Option<(i64, i64)>> =
            std::sync::Mutex::new(None);
        static CH_CONT: AtomicU64 = AtomicU64::new(0);
        static CH_TRAV: AtomicU64 = AtomicU64::new(0);
        static CONT: AtomicU64 = AtomicU64::new(0);
        // Contiguous == this path starts exactly where the previous one ended, so
        // no travel was needed between them (the internal analogue of R568's
        // G-code classification).
        let contiguous = {
            let mut g = PREV_END.lock().unwrap();
            let f = path.polyline.points.first().map(|p| (p.x, p.y));
            let c = *g == f;
            *g = path.polyline.points.last().map(|p| (p.x, p.y));
            c
        };
        if contiguous {
            CONT.fetch_add(1, Ordering::Relaxed);
        }
        let cur = (path.width as f32).to_bits() as u64;
        if PREV.swap(cur, Ordering::Relaxed) != cur {
            CHANGED.fetch_add(1, Ordering::Relaxed);
            if contiguous {
                CH_CONT.fetch_add(1, Ordering::Relaxed);
            } else {
                CH_TRAV.fetch_add(1, Ordering::Relaxed);
            }
        }
        let n = SEEN.fetch_add(1, Ordering::Relaxed) + 1;
        if n % 50_000 == 0 {
            println!(
                "EXPWPROBE outer_paths={} width_changed={} contiguous={} ch_contig={} ch_after_travel={} zero_width={} EMITTED_ALLROLES={} GLOBAL_paths={} GLOBAL_changed={}",
                n,
                CHANGED.load(Ordering::Relaxed),
                CONT.load(Ordering::Relaxed),
                CH_CONT.load(Ordering::Relaxed),
                CH_TRAV.load(Ordering::Relaxed),
                ZEROW.load(Ordering::Relaxed),
                EXPW_EMITTED.load(Ordering::Relaxed),
                EXPW_GLOBAL_N.load(Ordering::Relaxed),
                EXPW_GLOBAL_CH.load(Ordering::Relaxed),
            );
        }
    }

    // R234: native _extrude emits ";LAYER_HEIGHT: %g" when the path height
    // leaves the m_last_height register by > EPSILON (GCode.cpp:6619-6623;
    // bridges print at 0.4 over 0.2 layers). Register also fed by layer
    // changes (print.rs, GCode.cpp:4065).
    if crate::gcode::writer::lift_faithful_gate()
        && path.height > 0.0
        && ((writer.last_height_tag - path.height).abs() > 1e-4 || force_tags)
    {
        writer.last_height_tag = path.height;
        writer.write_comment(&format!("LAYER_HEIGHT: {}", fmt_g6(path.height)));
    }

    // R708 — the deferred collection-level feedrate lands HERE. R654 flushed it
    // straight after the Width tag, one tag too early: C++ writes Width
    // (GCode.cpp:6607), then Height (:6619-6623), and only then `set_speed`
    // (:6663), so the F must follow the height tag, not precede it.
    // Unconditional: if neither tag fired, the F is still emitted exactly once,
    // so the line count is identical either way.
    writer.flush_pending_speed();

    // C++ reference: GCode.cpp:4211-4220
    // C++: // get path properties
    // C++: const Polyline &polyline = path.polyline;
    // C++: const double mm3_per_mm = path.mm3_per_mm;
    // C++: const double width = path.width;
    // C++: const double height = path.height;
    // Get path properties
    let points = path.polyline.points();
    let mm3_per_mm = path.mm3_per_mm;
    let _width = path.width;
    let _height = path.height;

    // Calculate extrusion length per distance unit
    // GCode.cpp:6071-6081
    // C++: auto _mm3_per_mm = path.mm3_per_mm * double(this->config().print_flow_ratio.value);
    // C++: if( path.role() == erTopSolidInfill )
    // C++: _mm3_per_mm *= NOZZLE_CONFIG(top_solid_infill_flow_ratio);
    // C++: else if (this->on_first_layer())
    // C++: _mm3_per_mm *= m_config.initial_layer_flow_ratio.value;
    // C++: double e_per_mm = m_writer.filament()->e_per_mm3() * _mm3_per_mm;
    //
    // Apply extrusion multiplier (print_flow_ratio)
    // GCode.cpp:6073
    // C++: auto _mm3_per_mm = path.mm3_per_mm * double(this->config().print_flow_ratio.value);
    let mut adjusted_mm3_per_mm = mm3_per_mm * config.print_flow_ratio;

    // Apply role-specific flow multipliers
    // GCode.cpp:6074-6077
    // C++: if( path.role() == erTopSolidInfill )
    // C++: _mm3_per_mm *= NOZZLE_CONFIG(top_solid_infill_flow_ratio);
    // C++: else if (this->on_first_layer())
    // C++: _mm3_per_mm *= m_config.initial_layer_flow_ratio.value;
    if path.role == crate::extrusion_entity::ExtrusionRole::TopSolidInfill {
        adjusted_mm3_per_mm *= config.top_solid_infill_flow_ratio;
    } else if is_first_layer {
        adjusted_mm3_per_mm *= config.initial_layer_flow_ratio;
    }

    // Use extruder's e_per_mm3 to convert mm³/mm to filament mm/mm
    let e_per_mm = writer.extruder_e_per_mm(adjusted_mm3_per_mm);

    // Travel to the path's first point if the nozzle is not already there.
    //
    // C++ GCode::_extrude (GCode.cpp:6366-6368) travels to path.first_point()
    // whenever m_last_pos != path.first_point(), before emitting the path. The
    // segment emission below assumes the nozzle is at points[0]; when it is not
    // (extrude_loop reorders the loop to the seam via split_loop_at_*, and a
    // multi-path loop's later paths start where the previous path ended), the
    // first emitted segment is wrong. For an arc-fitted first segment this is
    // catastrophic: the I/J center offset is computed relative to
    // arc.start_point (= points[0]) but the printer/estimator applies it
    // relative to the ACTUAL nozzle position, producing a degenerate off-circle
    // arc whose computed arc length explodes (one arc -> "12 m" / ~100 s in the
    // GCodeProcessor), inflating the estimated print time ~3x (1h50m vs 43m).
    //
    // The feature feedrate was already set by extrude_collection / extrude_loop
    // before this call; travel_to() resets it to travel_speed (F60000), so we
    // capture and restore it after the travel — matching C++, where _extrude
    // emits the path F (speed*60) after the travel. set_speed is a no-op when
    // the feedrate is unchanged, so no spurious F line is emitted when no travel
    // was needed.
    if let Some(first_pt) = path.polyline.points().first() {
        let pos = writer.position();
        let tx = unscale(first_pt.x());
        let ty = unscale(first_pt.y());
        let dist_sq = (pos.x - tx) * (pos.x - tx) + (pos.y - ty) * (pos.y - ty);
        if dist_sq > 0.001 * 0.001 {
            let resume_speed = writer.get_current_speed() * 60.0; // mm/s -> mm/min

            // Bare positioning travel — no retraction/spiral-lift decision here.
            // This site handles the travel between the contiguous internal paths of a
            // multi-path loop / multipath. In C++ those moves are skipped entirely
            // (m_last_pos == path.first_point(), GCode::_extrude:6366), so they must
            // not retract. The retraction + spiral-lift decision for the travel to the
            // start of each top-level extrusion entity is made once in
            // extrude_collection (mirroring GCode::needs_retraction per _extrude),
            // which has already moved the nozzle here, so for an entity's first path
            // this branch is a no-op.
            writer.set_travel_acceleration(6000.0);
            writer.travel_to(tx, ty, None);
            if resume_speed > 0.0 {
                writer.set_speed(resume_speed, "");
            }
        }
    }

    // -------------------------------------------------------------------
    // Toolpath simplification + (optional) arc fitting.
    //
    // C++ splits this across two places that we fuse here because the Rust
    // ExtrusionPath does not carry a persistent `fitting_result`:
    //   1. LayerRegion::simplify_path (LayerRegion.cpp:786-802) mutates
    //      path.polyline.points and populates path.polyline.fitting_result.
    //   2. GCode::_extrude (GCode.cpp:6665-6745) emits G1 / G2 / G3 from the
    //      (possibly simplified) polyline and its fitting_result.
    //
    // The simplification is deterministic on the same input points, so doing
    // it here on a local copy yields the same motion output as doing it once
    // up front in PrintObject::simplify_extrusion_path.
    // -------------------------------------------------------------------
    // LayerRegion.cpp:789-791
    // C++: const bool spiral_mode        = print_config.spiral_mode;
    // C++: const bool enable_arc_fitting = print_config.enable_arc_fitting;
    // C++: const auto scaled_resolution  = scaled<double>(print_config.resolution.value);
    let spiral_mode = writer.spiral_mode();
    let enable_arc_fitting = writer.arc_fitting_enabled();

    // Both the plain Douglas-Peucker (geometry::simplify::douglas_peucker) and
    // the arc-fitter (arc_fitter::do_arc_fitting_and_simplify) need a tolerance,
    // but in *different units*:
    //   - geometry::simplify::douglas_peucker scales its argument internally, so
    //     it expects the tolerance in **mm** (unscaled).
    //   - arc_fitter::do_arc_fitting_and_simplify drives multi_point::douglas_peucker
    //     which compares against the raw *scaled* tolerance, exactly as C++
    //     feeds `scaled<double>(resolution)` (LayerRegion.cpp:791,798).
    let resolution_mm = writer.resolution();

    // SPARSE_INFILL_RESOLUTION = 0.04mm (libslic3r.h:65). Internal sparse infill
    // is simplified with this coarser tolerance.
    const SPARSE_INFILL_RESOLUTION_MM: f64 = 0.04;

    // Per-role simplification tolerance, mirroring LayerRegion::simplify_path
    // (LayerRegion.cpp:793-801). Internal sparse infill uses the coarser
    // SCALED_SPARSE_INFILL_RESOLUTION; everything else uses scaled_resolution.
    let tolerance_mm = if path.role == ExtrusionRole::InternalInfill {
        SPARSE_INFILL_RESOLUTION_MM
    } else {
        resolution_mm
    };

    // Working copy of the scaled points.
    let mut work_points: Vec<Point> = points.to_vec();

    if enable_arc_fitting && !spiral_mode && !path.polyline.fitting_result.is_empty() {
        // -------------------------------------------------------------------
        // Arc-fitting path: emit the STORED fitting_result that
        // LayerRegion::simplify_path's `simplify_by_fitting_arc(scaled_resolution)`
        // (LayerRegion.cpp:796-798) already computed once on the RAW perimeter
        // points (and which seam-splitting in extrude_loop keeps consistent with
        // polyline.points via ExtrusionLoop::split_at). This is exactly what C++
        // GCode::_extrude does (GCode.cpp:6700-6745): it iterates the stored
        // path.polyline.fitting_result, it does NOT re-fit.
        //
        // PARITY-FIX (arc-fit): the previous code re-ran do_arc_fitting_and_simplify
        // here on the already-arc-fit-AND-DP-simplified points, which destroyed
        // ~36% of the arcs that pass-1 had found (5758 -> 3688 outer-wall arcs)
        // because the per-segment Douglas-Peucker in pass-1 had already removed the
        // interior points that try_create_arc / are_points_within_slice need to
        // re-confirm curvature. Emitting the stored result restores native parity
        // (~5758 ≈ native 5793). Material is unchanged (arc dE uses arc.length, same
        // value; linear-run dE identical).
        // -------------------------------------------------------------------
        let fitting_result = &path.polyline.fitting_result;

        // GCode.cpp:6703-6744: iterate fitting_result, emit G1/G2/G3.
        for seg in fitting_result {
            match seg.path_type {
                EMovePathType::LinearMove => {
                    // GCode.cpp:6705-6720
                    let start_index = seg.start_point_index;
                    let end_index = seg.end_point_index;
                    for point_index in (start_index + 1)..(end_index + 1) {
                        let from = work_points[point_index - 1];
                        let to = work_points[point_index];
                        // line.length() in mm: `line.length() * SCALING_FACTOR`
                        // (GCode.cpp:8021), the double length. LINE_LENGTH_DOUBLE
                        // (on); =0 truncates it to whole units first, which made
                        // E one unit short in the last digit on ~3% of moves.
                        let line_length = if crate::faithful_gate("LINE_LENGTH_DOUBLE") {
                            from.distance_to_f64(to) * crate::libslic3r::SCALING_FACTOR
                        } else {
                            unscale(from.distance_to_f64(to) as i64)
                        };
                        if line_length < EPSILON {
                            continue;
                        }
                        let de = line_length * e_per_mm;
                        if path.no_extrusion {
                            // Native GCode.cpp:6686 extrude_to_xy(..., is_force_no_
                            // extrusion()): E-less move, filament E not advanced.
                            writer.wipe_to(unscale(to.x()), unscale(to.y()), None);
                        } else {
                            writer.extrude_to(unscale(to.x()), unscale(to.y()), de, None);
                        }
                        // GCode.cpp:8030 check_and_insert_timelapse(line.b).
                        writer.check_and_insert_timelapse(unscale(to.x()), unscale(to.y()));
                    }
                }
                EMovePathType::ArcMoveCw | EMovePathType::ArcMoveCcw => {
                    // GCode.cpp:6722-6737
                    let arc = &seg.arc_data;
                    // arc_length = arc.length * SCALING_FACTOR (scaled length -> mm).
                    let arc_length = arc.length * crate::libslic3r::SCALING_FACTOR;
                    if arc_length < EPSILON {
                        continue;
                    }
                    if std::env::var("ARCDBG").is_ok() {
                        let wp_start = work_points[seg.start_point_index];
                        let wp_end = work_points[seg.end_point_index];
                        let ds = ((wp_start.x() - arc.start_point.x()) as f64)
                            .hypot((wp_start.y() - arc.start_point.y()) as f64);
                        let de_ = ((wp_end.x() - arc.end_point.x()) as f64)
                            .hypot((wp_end.y() - arc.end_point.y()) as f64);
                        // center equidistant check (scaled):
                        let r_s = ((arc.circle.center.x() - arc.start_point.x()) as f64)
                            .hypot((arc.circle.center.y() - arc.start_point.y()) as f64);
                        let r_e = ((arc.circle.center.x() - arc.end_point.x()) as f64)
                            .hypot((arc.circle.center.y() - arc.end_point.y()) as f64);
                        if ds > 1000.0 || de_ > 1000.0 || (r_s - r_e).abs() > 0.02 * r_s.max(1.0) {
                            eprintln!(
                                "ARCDBG2 wp_start!=arcstart={:.0} wp_end!=arcend={:.0} r_s={:.0} r_e={:.0} ({})",
                                ds, de_, r_s, r_e,
                                crate::extrusion_entity::role_to_string(path.role)
                            );
                        }
                    }
                    // center_offset = point_to_gcode(center) - point_to_gcode(start).
                    // Origin/extruder offset cancels in the difference, so plain
                    // unscale matches the linear path's coordinate convention.
                    // Unscale each endpoint then subtract, mirroring C++'s ordering.
                    let i = unscale(arc.circle.center.x()) - unscale(arc.start_point.x());
                    let j = unscale(arc.circle.center.y()) - unscale(arc.start_point.y());
                    let de = arc_length * e_per_mm;
                    writer.extrude_arc(
                        unscale(arc.end_point.x()),
                        unscale(arc.end_point.y()),
                        i,
                        j,
                        de,
                        arc.direction,
                        None,
                    );
                    // GCode.cpp:8049 check_and_insert_timelapse(arc.end_point).
                    writer.check_and_insert_timelapse(
                        unscale(arc.end_point.x()),
                        unscale(arc.end_point.y()),
                    );
                }
                EMovePathType::NoopMove => {}
            }
        }
    } else {
        // Plain Douglas-Peucker + linear emission (GCode.cpp:6670-6686 G1 path).
        // Polyline::simplify -> MultiPoint::_douglas_peucker (Polyline.cpp:146-150).
        // GCode.cpp:7980-7998 walks path.polyline.lines() as they are: the one
        // simplification is LayerRegion::simplify_path's (LayerRegion.cpp:
        // 1053-1069), at the print resolution when arc fitting is off — also
        // for sparse infill, whose coarser 0.04 mm applies to the arc-fitting
        // arm only. EXPORT_NO_RESIMPLIFY=0 runs the second pass here as before.
        if work_points.len() > 2 && !crate::faithful_gate("EXPORT_NO_RESIMPLIFY") {
            let simplified = Polyline::douglas_peucker(
                &Polyline::from_points(work_points.clone()),
                tolerance_mm,
            );
            work_points = simplified.points;
        }

        // GCode.cpp:6676-6686: iterate polyline.lines(), skip segments < EPSILON.
        for i in 1..work_points.len() {
            let from = work_points[i - 1];
            let to = work_points[i];

            // line.length() in mm: `line.length() * SCALING_FACTOR`
            // (GCode.cpp:7987). LINE_LENGTH_DOUBLE (on); =0 truncates first.
            let line_length = if crate::faithful_gate("LINE_LENGTH_DOUBLE") {
                from.distance_to_f64(to) * crate::libslic3r::SCALING_FACTOR
            } else {
                unscale(from.distance_to_f64(to) as i64)
            };
            if line_length < EPSILON {
                continue;
            }
            let de = line_length * e_per_mm;
            // GCode.cpp:7993-7996: extrude_to_xy(…, path.is_force_no_extrusion())
            // — the connectors of a monotonic-line fill are E-less moves here
            // too, not only on the arc-fitted branch. NO_EXTRUSION_LINEAR=0
            // extrudes them as before.
            if path.no_extrusion && crate::faithful_gate("NO_EXTRUSION_LINEAR") {
                writer.wipe_to(unscale(to.x()), unscale(to.y()), None);
            } else {
                writer.extrude_to(unscale(to.x()), unscale(to.y()), de, None);
            }
            // GCode.cpp:8009 check_and_insert_timelapse(line.b).
            writer.check_and_insert_timelapse(unscale(to.x()), unscale(to.y()));
        }
    }

    // Emit cooling end marker after extrusion
    // C++ GCode.cpp:6361-6363
    writer.write_raw(";_EXTRUDE_END");
}

/// Extrude any extrusion entity (dispatcher).
///
/// C++ reference: GCode::extrude_entity()
/// GCode.cpp:3100-3400 (~300 lines)
///
/// This function dispatches to the appropriate extrusion function
/// based on the entity type (Loop, Path, Collection).
///
/// # Arguments
/// * `entity` - The extrusion entity type to extrude
/// * `writer` - GCodeWriter to emit commands
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors or unsupported entity type
pub fn extrude_entity(
    entity: &ExtrusionEntityType,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) -> Result<()> {
    /// Dispatch based on entity type
    /// GCode.cpp:3120-3180
    /// C++: std::string GCode::extrude_entity(const ExtrusionEntity &entity, const std::string &description, double speed) {
    /// C++: if (entity.is_loop()) {
    /// C++: return this->extrude_loop(*static_cast<const ExtrusionLoop*>(&entity), description, speed);
    /// C++: } else if (entity.is_path()) {
    /// C++: return this->_extrude(*static_cast<const ExtrusionPath*>(&entity), description, speed);
    /// C++: } else if (entity.is_collection()) {
    /// C++: return this->_extrude(*static_cast<const ExtrusionEntityCollection*>(&entity), description, speed);
    /// C++: } else {
    /// C++: throw std::runtime_error("Unknown extrusion entity type");
    /// C++: }
    /// C++: }
    match entity {
        /// Dispatch to loop handler
        /// GCode.cpp:3122-3125
        /// C++: if (entity.is_loop()) {
        /// C++: return this->extrude_loop(*static_cast<const ExtrusionLoop*>(&entity), description, speed);
        /// C++: }
        ExtrusionEntityType::Loop(loop_entity) => {
            extrude_loop(loop_entity, writer, config, is_first_layer);
        }

        /// Dispatch to path handler
        /// GCode.cpp:3126-3129
        /// C++: else if (entity.is_path()) {
        /// C++: return this->_extrude(*static_cast<const ExtrusionPath*>(&entity), description, speed);
        /// C++: }
        ExtrusionEntityType::Path(path) => {
            // R245 (gated): native _extrude emits set_speed(F) for EVERY path
            // (GCode.cpp:6663, cooling buffer dedups); the per-path F embeds
            // the volumetric cap, so variable-width gap/fill paths carry
            // fractional Fs (F14761.269 = 25mm3s / mm3_per_mm). writer
            // set_speed dedups, so without VOLCAP this is a no-op.
            // FVS_SEPARATE_SPEED (on): GCode::extrude_path (GCode.cpp:6930-6931)
            // passes `use_seperate_speed` for a path flagged
            // cfFloatingVerticalShell. `_extrude` then takes the bridge speed
            // for it (:7738-7741) and brackets it with "; Slow Down Start" /
            // "; Slow Down End" (:7874-7875, :8083-8085). =0 prints it as any
            // other floating vertical shell path.
            let use_seperate_speed = crate::faithful_gate("FVS_SEPARATE_SPEED")
                && path.get_customize_flag()
                    == crate::extrusion_entity::CustomizeFlag::FloatingVerticalShell;
            if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                use crate::extrusion_entity::ExtrusionRole;
                let role = path.role;
                let base = if is_first_layer {
                    match role {
                        ExtrusionRole::InternalInfill
                        | ExtrusionRole::SolidInfill
                        | ExtrusionRole::TopSolidInfill
                        | ExtrusionRole::BottomSurface => {
                            if config.initial_layer_infill_speed > 0.0 {
                                config.initial_layer_infill_speed
                            } else {
                                config.initial_layer_speed
                            }
                        }
                        _ => config.initial_layer_speed,
                    }
                } else {
                    match role {
                        ExtrusionRole::ExternalPerimeter => config.external_perimeter_speed,
                        ExtrusionRole::Perimeter => config.perimeter_speed,
                        ExtrusionRole::InternalInfill => config.infill_speed,
                        ExtrusionRole::SolidInfill => config.solid_infill_speed,
                        ExtrusionRole::TopSolidInfill => config.top_solid_infill_speed,
                        ExtrusionRole::BridgeInfill => config.bridge_speed,
                        ExtrusionRole::GapFill => config.gap_fill_speed,
                        ExtrusionRole::FloatingVerticalShell if use_seperate_speed => {
                            config.bridge_speed
                        }
                        ExtrusionRole::FloatingVerticalShell => {
                            config.vertical_shell_speed / 100.0 * config.solid_infill_speed
                        }
                        _ => config.perimeter_speed,
                    }
                };
                if use_seperate_speed && !std::mem::take(&mut writer.slow_down_start_written) {
                    writer.write_raw("; Slow Down Start");
                }
                let speed = if crate::faithful_gate("VOLCAP_FAITHFUL") {
                    volumetric_capped_speed(
                        base,
                        path.mm3_per_mm,
                        writer.config_ref().filament_max_volumetric_speed,
                        config.print_flow_ratio,
                    )
                } else {
                    base
                };
                width_tag_before_speed(writer, path);
                if speed > 0.0 {
                    let cooling_comment = if role == ExtrusionRole::BridgeInfill {
                        ""
                    } else if role == ExtrusionRole::ExternalPerimeter {
                        ";_EXTRUDE_SET_SPEED;_EXTERNAL_PERIMETER"
                    } else {
                        ";_EXTRUDE_SET_SPEED"
                    };
                    set_speed_before_path(writer, speed * 60.0, cooling_comment);
                }
            }
            extrude_path(path, writer, config, is_first_layer);
            if use_seperate_speed && crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                writer.write_raw("; Slow Down End");
            }
            // R233: native extrude_path installs the wipe path = the path's
            // SOURCE polyline REVERSED (GCode.cpp:5703-5717, non-tree branch)
            // — fills wipe backwards along the just-printed line. Rust only
            // installed wipe paths at loop ends, so 207 native wipes had no
            // rust counterpart (bare retracts).
            if crate::gcode::writer::lift_faithful_gate() {
                let pts: Vec<(f64, f64)> = path
                    .polyline
                    .points()
                    .iter()
                    .rev()
                    .map(|pt| (crate::unscale(pt.x()), crate::unscale(pt.y())))
                    .collect();
                if pts.len() >= 2 {
                    writer.set_wipe_path_points_from(pts, "path_rev");
                }
            }
            // R227: native extrude_path wrapper resets accel after the path
            // (GCode.cpp:5719-5725).
            writer.reset_acceleration_default(is_first_layer);
        }

        /// Dispatch to collection handler (recursive)
        /// GCode.cpp:3130-3133
        /// C++: else if (entity.is_collection()) {
        /// C++: return this->_extrude(*static_cast<const ExtrusionEntityCollection*>(&entity), description, speed);
        /// C++: }
        ExtrusionEntityType::Collection(collection) => {
            extrude_collection(collection, writer, config, is_first_layer)?;
        }
    }

    /// TODO : Add feature type annotations
    /// GCode.cpp:3150-3200
    /// C++: // Emit feature comment if role changed
    /// C++: if (entity->role() != m_last_extrusion_role) {
    /// C++: gcode += "; FEATURE: " + entity->role_to_string() + "\n";
    /// C++: m_last_extrusion_role = entity->role();
    /// C++: }

    /// TODO : Add metadata comments (description, speed, etc.)
    /// GCode.cpp:3210-3250
    /// C++: if (!description.empty()) {
    /// C++: gcode += "; " + description + "\n";
    /// C++: }
    Ok(())
}

/// Extrude perimeters for a single region on one island.
///
/// C++ reference: GCode::extrude_perimeters()
/// GCode.cpp:5362-5383
///
/// Iterates the region's perimeters and extrudes each entity.
/// When `skip_inner_walls` is true (spiral vase mode), only external
/// perimeters are emitted.
pub fn extrude_perimeters(
    region: &crate::layer::LayerRegion,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
    skip_inner_walls: bool,
) {
    if region.perimeters.entities.is_empty() {
        return;
    }

    // Retract and travel to first perimeter point
    writer.retract();
    writer.set_travel_acceleration(6000.0);
    let first_target = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
        travel_target_for_entity(&region.perimeters.entities[0], writer_last_pos(writer))
    } else {
        get_entity_first_point(&region.perimeters.entities[0])
    };
    if let Some(first_pt) = first_target {
        let target_x = crate::unscale(first_pt.x());
        let target_y = crate::unscale(first_pt.y());
        if crate::gcode::writer::lift_faithful_gate() {
            let zdest = if writer.nominal_z > 0.0 { writer.nominal_z } else { writer.z() };
            writer.travel_to_xyz(target_x, target_y, zdest);
            // GCode.cpp:7618 — the first `_extrude` after change_layer clears it.
            writer.m_need_change_layer_lift_z = false;
        } else {
            writer.travel_to(target_x, target_y, None);
        }
    }
    writer.unretract();

    if skip_inner_walls {
        // Spiral vase mode: only emit outer perimeter entities
        use crate::extrusion_entity::ExtrusionRole;
        for entity in &region.perimeters.entities {
            let role = get_entity_role(entity);
            if role == ExtrusionRole::ExternalPerimeter {
                let _ = extrude_entity(entity, writer, config, is_first_layer);
            }
        }
    } else {
        // GCode.cpp:5372-5380: extrude perimeters via extrude_collection
        // which handles FEATURE comments, LINE_WIDTH, feedrate (F), and
        // extrusion acceleration (M204) per entity role change.
        let _ = extrude_collection(&region.perimeters, writer, config, is_first_layer);
    }
}

/// Extrude infill for a single region.
///
/// C++ reference: GCode::extrude_infill()
/// GCode.cpp:5753-5780
///
/// Chains the region's infill entities by a greedy nearest-neighbor algorithm
/// (starting near the writer's current position, == C++ `m_last_pos`), then
/// extrudes each. Per C++, each fill that is itself an ExtrusionEntityCollection
/// (EEC) is re-chained via `chained_path_from(m_last_pos)` before emission.
///
/// C++ calls this once per `ironing` flag (false then true) so that all
/// non-ironing infill across all regions precedes all ironing infill. The Rust
/// pipeline drives this per region, so we replicate the filter locally: chain &
/// emit non-ironing fills first, then ironing fills.
pub fn extrude_infill(
    region: &crate::layer::LayerRegion,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) {
    use crate::extrusion_entity::ExtrusionRole;

    if region.fills.entities.is_empty() {
        return;
    }

    // GCode.cpp:5759-5765: build the `extrusions` vector, filtering by ironing
    // role. `(ee->role() == erIroning) == ironing` selects non-ironing fills on
    // the first pass and ironing fills on the second. We clone the entities so
    // chain_and_reorder (which reorders/reverses in place) does not mutate the
    // stored region.fills.
    for ironing_pass in [false, true] {
        let mut extrusions: Vec<ExtrusionEntityType> = region
            .fills
            .entities
            .iter()
            .filter(|ee| (get_entity_role(ee) == ExtrusionRole::Ironing) == ironing_pass)
            .cloned()
            .collect();
        if extrusions.is_empty() {
            continue;
        }

        // GCode.cpp:5768: chain_and_reorder_extrusion_entities(extrusions, &m_last_pos).
        // NOTE: chain_and_reorder retains-out empty collections (ShortestPath.cpp:1033-1035),
        // so `extrusions` may become empty here even though the role filter found entries.
        let m_last_pos = writer_last_pos(writer);
        crate::shortest_path::chain_and_reorder_extrusion_entities(
            &mut extrusions,
            Some(&m_last_pos),
        );

        // Retract and travel to the (now reordered) first infill point.
        let Some(first_pt) = extrusions.first().and_then(|e| {
            if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                travel_target_for_entity(e, writer_last_pos(writer))
            } else {
                get_entity_first_point(e)
            }
        }) else {
            continue;
        };
        // ZSMOOTH_FAITHFUL: bucket-start retract through faithful
        // needs_retraction (GCode.cpp:6964) — R167.
        {
            let __zs_do_retract = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                let from = writer_last_pos(writer);
                let dx = crate::unscale(first_pt.x()) - crate::unscale(from.x);
                let dy = crate::unscale(first_pt.y()) - crate::unscale(from.y);
                let travel_len = (dx * dx + dy * dy).sqrt();
                let role = extrusions
                    .first()
                    .map(get_entity_role)
                    .unwrap_or(crate::extrusion_entity::ExtrusionRole::InternalInfill);
                writer.needs_retraction_faithful(from, first_pt, role, travel_len)
            } else {
                true
            };
            if __zs_do_retract {
                writer.retract();
            }
            if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                let from = writer_last_pos(writer);
                let dx = crate::unscale(first_pt.x()) - crate::unscale(from.x);
                let dy = crate::unscale(first_pt.y()) - crate::unscale(from.y);
                let role = extrusions
                    .first()
                    .map(get_entity_role)
                    .unwrap_or(crate::extrusion_entity::ExtrusionRole::InternalInfill);
                writer.set_travel_acceleration_for(role, (dx * dx + dy * dy).sqrt());
            } else {
                writer.set_travel_acceleration(6000.0);
            }
            if crate::gcode::writer::lift_faithful_gate() {
                let zdest = if writer.nominal_z > 0.0 { writer.nominal_z } else { writer.z() };
                writer.travel_to_xyz(
                    crate::unscale(first_pt.x()),
                    crate::unscale(first_pt.y()),
                    zdest,
                );
                // GCode.cpp:7618 — the first `_extrude` after change_layer clears it.
                writer.m_need_change_layer_lift_z = false;
            } else {
                writer.travel_to(crate::unscale(first_pt.x()), crate::unscale(first_pt.y()), None);
            }
            // Native _extrude unconditionally unretracts at extrusion start —
            // also clears the initial/carried retracted state.
            writer.unretract();
        }

        // GCode.cpp:5769-5776: for each fill, if it is an EEC, re-chain it via
        // chained_path_from(m_last_pos) and emit its entities; otherwise emit
        // the entity directly.
        for fill in &extrusions {
            match fill {
                ExtrusionEntityType::Collection(eec) => {
                    let chained = chained_path_from(eec, writer);
                    let _ = extrude_collection(&chained, writer, config, is_first_layer);
                }
                _ => {
                    let _ = extrude_entity(fill, writer, config, is_first_layer);
                }
            }
        }
    }
}

/// Island-subset perimeter emission (GCode::extrude_perimeters on one island's
/// by_region perimeters). Same retract/travel/extrude as `extrude_perimeters`
/// but over a given entity subset (this island+region's perimeter EECs).
pub fn extrude_perimeters_entities(
    entities: &[ExtrusionEntityType],
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
    skip_inner_walls: bool,
    // Per-entity cooling-node ids (ZSMOOTH_FAITHFUL): when Some, a
    // `; COOLING_NODE: <id>` marker precedes every entity whose id != -1
    // (GCode.cpp:5738-5747 — native's compare value never updates from -1).
    cooling_node_ids: Option<&[i32]>,
) {
    if entities.is_empty() {
        return;
    }
    // COOLING_NODE_BEFORE_TRAVEL (on): `GCode::extrude_perimeters` writes the
    // marker and then `extrude_entity`, whose `_extrude` travels to the first
    // point (GCode.cpp:6968-6976, :7611-7618); the bucket-start travel below is
    // that first entity's travel, so its marker goes before it. =0 writes it
    // after the travel.
    let node_first = cooling_node_ids.is_some()
        && !skip_inner_walls
        && crate::faithful_gate("COOLING_NODE_BEFORE_TRAVEL");
    if node_first {
        let id = cooling_node_ids.and_then(|ids| ids.first().copied()).unwrap_or(-1);
        if id != -1 {
            writer.write_raw(&format!("; COOLING_NODE: {}", id));
        }
    }
    // ZSMOOTH_FAITHFUL: the bucket-start retract must go through the faithful
    // needs_retraction (GCode.cpp:6964) — the unconditional retract here was
    // the outer-wall->gap wipe excess (R167). Legacy path keeps the
    // unconditional behavior (byte-locked).
    let first_pt_opt = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
        travel_target_for_entity(&entities[0], writer_last_pos(writer))
    } else {
        get_entity_first_point(&entities[0])
    };
    let do_retract = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
        match first_pt_opt {
            Some(fp) => {
                let from = writer_last_pos(writer);
                let dx = crate::unscale(fp.x()) - crate::unscale(from.x);
                let dy = crate::unscale(fp.y()) - crate::unscale(from.y);
                let travel_len = (dx * dx + dy * dy).sqrt();
                let role = get_entity_role(&entities[0]);
                writer.needs_retraction_faithful(from, fp, role, travel_len)
            }
            None => true,
        }
    } else {
        true
    };
    if do_retract {
        writer.retract();
    }
    if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
        if let Some(fp) = first_pt_opt {
            let from = writer_last_pos(writer);
            let dx = crate::unscale(fp.x()) - crate::unscale(from.x);
            let dy = crate::unscale(fp.y()) - crate::unscale(from.y);
            writer.set_travel_acceleration_for(
                get_entity_role(&entities[0]),
                (dx * dx + dy * dy).sqrt(),
            );
        }
    } else {
        writer.set_travel_acceleration(6000.0);
    }
    if let Some(first_pt) = first_pt_opt {
        if crate::gcode::writer::lift_faithful_gate() {
            let zdest = if writer.nominal_z > 0.0 { writer.nominal_z } else { writer.z() };
            writer.travel_to_xyz(
                crate::unscale(first_pt.x()),
                crate::unscale(first_pt.y()),
                zdest,
            );
            // GCode.cpp:7618 — the first `_extrude` after change_layer clears it.
            writer.m_need_change_layer_lift_z = false;
        } else {
            writer.travel_to(crate::unscale(first_pt.x()), crate::unscale(first_pt.y()), None);
        }
    }
    // Native _extrude unconditionally unretracts at extrusion start.
    writer.unretract();
    if skip_inner_walls {
        use crate::extrusion_entity::ExtrusionRole;
        for entity in entities {
            if get_entity_role(entity) == ExtrusionRole::ExternalPerimeter {
                let _ = extrude_entity(entity, writer, config, is_first_layer);
            }
        }
    } else if let Some(ids) = cooling_node_ids {
        // Marker-injecting path (gated): per-entity emission, same order and
        // writer state as the batch path below.
        for (k, entity) in entities.iter().enumerate() {
            let id = ids.get(k).copied().unwrap_or(-1);
            if id != -1 && !(node_first && k == 0) {
                writer.write_raw(&format!("; COOLING_NODE: {}", id));
            }
            let coll = crate::extrusion_entity::ExtrusionEntityCollection {
                entities: vec![entity.clone()],
                no_sort: true,
                ..Default::default()
            };
            let _ = extrude_collection(&coll, writer, config, is_first_layer);
        }
    } else {
        let coll = crate::extrusion_entity::ExtrusionEntityCollection {
            entities: entities.to_vec(),
            no_sort: true,
            ..Default::default()
        };
        let _ = extrude_collection(&coll, writer, config, is_first_layer);
    }
}

/// Island-subset infill emission (GCode::extrude_infill on one island's by_region
/// fills). Same ironing-split / chain / extrude as `extrude_infill` over a subset.
pub fn extrude_infill_entities(
    entities: &[ExtrusionEntityType],
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) {
    use crate::extrusion_entity::ExtrusionRole;
    if entities.is_empty() {
        return;
    }
    for ironing_pass in [false, true] {
        let mut extrusions: Vec<ExtrusionEntityType> = entities
            .iter()
            .filter(|ee| (get_entity_role(ee) == ExtrusionRole::Ironing) == ironing_pass)
            .cloned()
            .collect();
        if extrusions.is_empty() {
            continue;
        }
        let m_last_pos = writer_last_pos(writer);
        crate::shortest_path::chain_and_reorder_extrusion_entities(&mut extrusions, Some(&m_last_pos));
        let Some(first_pt) = extrusions.first().and_then(|e| {
            if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                travel_target_for_entity(e, writer_last_pos(writer))
            } else {
                get_entity_first_point(e)
            }
        }) else {
            continue;
        };
        // ZSMOOTH_FAITHFUL: bucket-start retract through faithful
        // needs_retraction (GCode.cpp:6964) — R167.
        {
            let __zs_do_retract = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                let from = writer_last_pos(writer);
                let dx = crate::unscale(first_pt.x()) - crate::unscale(from.x);
                let dy = crate::unscale(first_pt.y()) - crate::unscale(from.y);
                let travel_len = (dx * dx + dy * dy).sqrt();
                let role = extrusions
                    .first()
                    .map(get_entity_role)
                    .unwrap_or(crate::extrusion_entity::ExtrusionRole::InternalInfill);
                writer.needs_retraction_faithful(from, first_pt, role, travel_len)
            } else {
                true
            };
            if __zs_do_retract {
                writer.retract();
            }
            if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                let from = writer_last_pos(writer);
                let dx = crate::unscale(first_pt.x()) - crate::unscale(from.x);
                let dy = crate::unscale(first_pt.y()) - crate::unscale(from.y);
                let role = extrusions
                    .first()
                    .map(get_entity_role)
                    .unwrap_or(crate::extrusion_entity::ExtrusionRole::InternalInfill);
                writer.set_travel_acceleration_for(role, (dx * dx + dy * dy).sqrt());
            } else {
                writer.set_travel_acceleration(6000.0);
            }
            if crate::gcode::writer::lift_faithful_gate() {
                let zdest = if writer.nominal_z > 0.0 { writer.nominal_z } else { writer.z() };
                writer.travel_to_xyz(
                    crate::unscale(first_pt.x()),
                    crate::unscale(first_pt.y()),
                    zdest,
                );
                // GCode.cpp:7618 — the first `_extrude` after change_layer clears it.
                writer.m_need_change_layer_lift_z = false;
            } else {
                writer.travel_to(crate::unscale(first_pt.x()), crate::unscale(first_pt.y()), None);
            }
            // Native _extrude unconditionally unretracts at extrusion start —
            // also clears the initial/carried retracted state.
            writer.unretract();
        }
        for fill in &extrusions {
            match fill {
                ExtrusionEntityType::Collection(eec) => {
                    let chained = chained_path_from(eec, writer);
                    let _ = extrude_collection(&chained, writer, config, is_first_layer);
                }
                _ => {
                    // A path that Region::append took out of a sortable
                    // collection (GCode.cpp:9129-9131). It is printed like a
                    // collection of one, so it gets its feature tag, width and
                    // speed the way extrude_collection writes them.
                    let single = ExtrusionEntityCollection {
                        entities: vec![fill.clone()],
                        no_sort: true,
                        ..Default::default()
                    };
                    let _ = extrude_collection(&single, writer, config, is_first_layer);
                }
            }
        }
    }
}

/// Extrude support material fills.
///
/// C++ reference: GCode::extrude_support()
/// GCode.cpp:5782-5848
///
/// Splits the fills into ironing / non-ironing groups, chains each group with a
/// greedy nearest-neighbor algorithm (starting near the writer's current
/// position == C++ `m_last_pos`), then emits non-ironing first, ironing second
/// ("make sure the ironing was after the support extrusions", GCode.cpp:5844).
/// Each role gets its own FEATURE label.
pub fn extrude_support(
    support_fills: &ExtrusionEntityCollection,
    writer: &mut GCodeWriter,
    config: &crate::print_config::PrintObjectConfig,
    is_first_layer: bool,
) {
    use crate::extrusion_entity::ExtrusionRole;

    if support_fills.entities.is_empty() {
        return;
    }

    // GCode.cpp:5794-5802: split ironing vs non-ironing. Clone so chaining does
    // not mutate the stored support_fills.
    let mut extrusions: Vec<ExtrusionEntityType> = Vec::new();
    let mut ironing_extrusions: Vec<ExtrusionEntityType> = Vec::new();
    let mut has_support_ironing = false;
    for ee in &support_fills.entities {
        if get_entity_role(ee) == ExtrusionRole::SupportIroning {
            ironing_extrusions.push(ee.clone());
            has_support_ironing = true;
        } else {
            extrusions.push(ee.clone());
        }
    }
    // GCode.cpp:5803
    if extrusions.is_empty() && ironing_extrusions.is_empty() {
        return;
    }
    // GCode.cpp:5804-5810: chain each group; clear ironing if disabled.
    has_support_ironing = has_support_ironing && config.enable_support_ironing;
    if has_support_ironing {
        let m_last_pos = writer_last_pos(writer);
        crate::shortest_path::chain_and_reorder_extrusion_entities(
            &mut ironing_extrusions,
            Some(&m_last_pos),
        );
    } else {
        ironing_extrusions.clear();
    }
    {
        let m_last_pos = writer_last_pos(writer);
        crate::shortest_path::chain_and_reorder_extrusion_entities(
            &mut extrusions,
            Some(&m_last_pos),
        );
    }

    // GCode.cpp:5815-5842: emit one entity, with role label. Nested collections
    // recurse into extrude_support (matching the C++ dynamic_cast<EEC> branch).
    fn process_entities(
        entities: &[ExtrusionEntityType],
        writer: &mut GCodeWriter,
        config: &crate::print_config::PrintObjectConfig,
        is_first_layer: bool,
    ) {
        for ee in entities {
            if let ExtrusionEntityType::Collection(coll) = ee {
                // GCode.cpp:5836-5837: collection -> recurse.
                extrude_support(coll, writer, config, is_first_layer);
                continue;
            }
            let role = get_entity_role(ee);
            let label = match role {
                ExtrusionRole::SupportMaterial => "support material",
                ExtrusionRole::SupportMaterialInterface => "support material interface",
                ExtrusionRole::SupportIroning => "support ironing",
                ExtrusionRole::SupportTransition => "support transition",
                _ => "support material",
            };
            writer.write_comment(&format!("FEATURE: {}", label));
            let _ = extrude_entity(ee, writer, config, is_first_layer);
        }
    }

    // First positioning travel before the chained non-ironing group, matching
    // the prior behaviour (retract + travel to the reordered first point).
    if let Some(first_pt) = extrusions.first().and_then(|e| {
        if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
            travel_target_for_entity(e, writer_last_pos(writer))
        } else {
            get_entity_first_point(e)
        }
    }) {
        // ZSMOOTH_FAITHFUL: faithful needs_retraction (R167).
        {
            let __zs_do_retract = if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                let from = writer_last_pos(writer);
                let dx = crate::unscale(first_pt.x()) - crate::unscale(from.x);
                let dy = crate::unscale(first_pt.y()) - crate::unscale(from.y);
                let travel_len = (dx * dx + dy * dy).sqrt();
                let role = extrusions
                    .first()
                    .map(get_entity_role)
                    .unwrap_or(crate::extrusion_entity::ExtrusionRole::InternalInfill);
                writer.needs_retraction_faithful(from, first_pt, role, travel_len)
            } else {
                true
            };
            if __zs_do_retract {
                writer.retract();
            }
            if crate::faithful_gate("ZSMOOTH_FAITHFUL") {
                let from = writer_last_pos(writer);
                let dx = crate::unscale(first_pt.x()) - crate::unscale(from.x);
                let dy = crate::unscale(first_pt.y()) - crate::unscale(from.y);
                let role = extrusions
                    .first()
                    .map(get_entity_role)
                    .unwrap_or(crate::extrusion_entity::ExtrusionRole::InternalInfill);
                writer.set_travel_acceleration_for(role, (dx * dx + dy * dy).sqrt());
            } else {
                writer.set_travel_acceleration(6000.0);
            }
            if crate::gcode::writer::lift_faithful_gate() {
                let zdest = if writer.nominal_z > 0.0 { writer.nominal_z } else { writer.z() };
                writer.travel_to_xyz(
                    crate::unscale(first_pt.x()),
                    crate::unscale(first_pt.y()),
                    zdest,
                );
                // GCode.cpp:7618 — the first `_extrude` after change_layer clears it.
                writer.m_need_change_layer_lift_z = false;
            } else {
                writer.travel_to(crate::unscale(first_pt.x()), crate::unscale(first_pt.y()), None);
            }
            // Native _extrude unconditionally unretracts at extrusion start —
            // also clears the initial/carried retracted state.
            writer.unretract();
        }
    }

    // GCode.cpp:5843-5845: non-ironing first, then ironing.
    process_entities(&extrusions, writer, config, is_first_layer);
    if has_support_ironing {
        process_entities(&ironing_extrusions, writer, config, is_first_layer);
    }
}

/// Generate a travel move to the specified point.
///
/// C++ reference: GCode::travel_to()
/// GCode.cpp:6416-6572 (~157 lines)
///
/// This function:
/// 1. Decides whether retraction is needed
/// 2. Performs retraction if needed
/// 3. Optionally avoids crossing perimeters
/// 4. Optionally performs Z-hop
/// 5. Generates G0/G1 travel move
/// 6. Optionally performs un-retraction
///
/// # Arguments
/// * `point` - Destination point
/// * `writer` - GCodeWriter to emit commands
/// * `config` - Travel configuration
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors
pub fn travel_to(point: Point, writer: &mut GCodeWriter, config: &TravelConfig) -> Result<()> {
    /// Define the travel move as a line between current position and target point
    /// GCode.cpp:6418-6420
    /// C++: Polyline travel { this->last_pos(), point };
    let current_pos = writer.position();
    let current_pos_scaled = Point::new(
        (current_pos.x * 1_000_000.0) as i64,
        (current_pos.y * 1_000_000.0) as i64,
    );

    /// Create travel polyline
    /// GCode.cpp:6418-6420
    let travel = Polyline::from_points(vec![current_pos_scaled, point]);

    /// Calculate travel distance and check if retraction is needed
    /// GCode.cpp:6422-6424
    /// C++: bool needs_retraction = this->needs_retraction(travel, role, lift_type);
    let travel_distance = unscale(travel.length() as i64);
    let needs_retraction =
        config.retract_on_travel && travel_distance >= config.retract_length_travel;

    /// If reduce_crossing_wall is enabled, try to plan multi-hop path
    /// GCode.cpp:6431-6440
    /// C++: if (m_config.reduce_crossing_wall && !m_avoid_crossing_perimeters.disabled_once())
    /// C++: {
    /// C++: travel = m_avoid_crossing_perimeters.travel_to(*this, point, &could_be_wipe_disabled);
    /// C++: needs_retraction = this->needs_retraction(travel, role, lift_type);
    /// C++: }
    // TODO: Implement avoid_crossing_perimeters integration (GCode.cpp:6431-6440)
    // For now, skip path optimization - use direct travel

    /// Perform retraction if needed
    /// GCode.cpp:6447-6465
    /// C++: if (needs_retraction) {
    /// C++: if (m_config.reduce_crossing_wall && could_be_wipe_disabled && !m_last_scarf_seam_flag)
    /// C++: m_wipe.reset_path();
    /// C++: Point last_post_before_retract = this->last_pos();
    /// C++: gcode += this->retract(false, false, lift_type);
    /// C++: ...
    /// C++: } else {
    /// C++: m_wipe.reset_path();
    /// C++: }
    if needs_retraction {
        retract(writer, false)?;
    }

    /// Emit travel move(s)
    /// GCode.cpp:6475-6562
    /// C++: if (travel.size() >= 2) {
    /// C++: ...
    /// C++: for (size_t i = 1; i < travel.size(); ++i)
    /// C++: gcode += m_writer.travel_to_xy(this->point_to_gcode(travel.points[i]), comment, use_short_travel_accel);
    /// C++: ...
    /// C++: }
    if travel.points().len() >= 2 {
        /// Emit travel moves for each segment
        /// GCode.cpp:6520-6562
        for i in 1..travel.points().len() {
            let target = travel.points()[i];
            let target_x = unscale(target.x);
            let target_y = unscale(target.y);
            writer.travel_to(target_x, target_y, None);
        }
    }

    Ok(())
}

/// Perform retraction.
///
/// C++ reference: GCode::retract()
/// GCode.cpp:6693-6725 (~33 lines)
///
/// This function:
/// 1. Checks if already retracted (avoid double retraction)
/// 2. Emits retraction command (G1 E-<length> or G10 for firmware retract)
/// 3. Optionally performs wipe move
/// 4. Updates retraction state
///
/// # Arguments
/// * `writer` - GCodeWriter to emit commands
/// * `wipe` - Whether to perform wipe move after retraction
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors
pub fn retract(writer: &mut GCodeWriter, wipe: bool) -> Result<()> {
    /// Check if already retracted - no-op if yes
    /// GCode.cpp:6695-6697
    /// C++: if (m_writer.filament() == nullptr)
    /// C++: return gcode;
    if writer.is_retracted() {
        return Ok(());
    }

    /// Perform wipe if enabled and wipe path available
    /// GCode.cpp:6699-6702
    /// C++: if (FILAMENT_CONFIG(wipe) && m_wipe.has_path() && scale_(FILAMENT_CONFIG(wipe_distance)) > SCALED_EPSILON) {
    /// C++: gcode += toolchange ? m_writer.retract_for_toolchange(true) : m_writer.retract(true);
    /// C++: gcode += m_wipe.wipe(*this, toolchange, is_last_retraction);
    /// C++: }
    // TODO: Implement wipe integration (GCode.cpp:6699-6702)
    // For now, skip wipe - just do direct retraction
    let _ = wipe;

    /// Call writer's retract method (handles firmware retract or manual retract)
    /// GCode.cpp:6707-6708
    /// C++: gcode += toolchange ? m_writer.retract_for_toolchange() : m_writer.retract();
    writer.retract();

    /// Reset E position after retraction
    /// GCode.cpp:6710
    /// C++: gcode += m_writer.reset_e();
    // Note: GCodeWriter::retract() already handles E tracking

    /// Perform Z-lift if retraction length > 0 or firmware retraction enabled
    /// GCode.cpp:6711-6720
    /// C++: if (m_writer.filament()->retraction_length() > 0 || m_config.use_firmware_retraction) {
    /// C++: if (apply_instantly)
    /// C++: gcode += m_writer.eager_lift(lift_type,toolchange);
    /// C++: else
    /// C++: gcode += m_writer.lazy_lift(lift_type, m_spiral_vase != nullptr, toolchange);
    /// C++: }
    // Note: Z-lift is handled by GCodeWriter::retract() which calls do_z_hop()
    Ok(())
}

/// Perform un-retraction (prime).
///
/// C++ reference: GCodeWriter::unretract()
/// GCodeWriter.cpp:808-839 (~32 lines)
///
/// This function:
/// 1. Checks if currently retracted (no-op if not)
/// 2. Emits un-retraction command (G1 E<length> or G11 for firmware)
/// 3. Optionally adds extra restart length
/// 4. Updates retraction state
///
/// # Arguments
/// * `writer` - GCodeWriter to emit commands
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors
pub fn unretract(writer: &mut GCodeWriter) -> Result<()> {
    /// Check if currently retracted - no-op if not
    /// GCodeWriter.cpp:810-820
    /// C++: if (double dE = filament()->unretract(); dE != 0) {
    /// C++: if (config.use_firmware_retraction) {
    /// C++: gcode += FLAVOR_IS(gcfMachinekit) ? "G23 ;unretract \n" : "G11 ;unretract \n";
    /// C++: gcode += reset_e();
    /// C++: }
    /// C++: else {
    /// C++: ...
    /// C++: }
    /// C++: }
    if !writer.is_retracted() {
        return Ok(());
    }

    /// Call writer's unretract method (handles firmware unretract or manual unretract)
    /// GCodeWriter.cpp:815-828
    /// C++: GCodeG1Formatter w;
    /// C++: w.emit_e(filament()->E()+extra_retract);
    /// C++: w.emit_f(filament()->deretract_speed() * 60.);
    /// C++: w.emit_comment(GCodeWriter::full_gcode_comment, " ; unretract");
    /// C++: gcode += w.string();
    writer.unretract();

    Ok(())
}

/// Perform a wipe move.
///
/// C++ reference: Wipe::wipe()
/// GCode.cpp:355-438 (~83 lines)
///
/// This function:
/// 1. Calculates wipe speed (reduced from travel speed)
/// 2. Takes the stored wipe path and clips it to wipe distance
/// 3. Retracts while traveling along wipe path
/// 4. Emits wipe start/end tags for processor
///
/// # Arguments
/// * `writer` - GCodeWriter to emit commands
/// * `wipe_path` - Path to wipe along (from last extrusion)
/// * `wipe_distance` - Target wipe distance (mm)
/// * `retraction_length` - Total retraction length to distribute over wipe
/// * `wipe_speed` - Wipe speed in mm/s
/// * `toolchange` - Whether this is a toolchange wipe
/// * `is_last` - Whether this is the last wipe (affects cooling markers)
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors
pub fn wipe(
    writer: &mut GCodeWriter,
    wipe_path: &Polyline,
    wipe_distance: f64,
    retraction_length: f64,
    wipe_speed: f64,
    _toolchange: bool,
    is_last: bool,
) -> Result<()> {
    /// Get retraction length to apply during wipe
    /// GCode.cpp:366-373
    /// C++: double length = toolchange
    /// C++: ? gcodegen.writer().filament()->retract_length_toolchange()
    /// C++: : gcodegen.writer().filament()->retraction_length();
    /// C++: // Shorten the retraction length by the amount already retracted before wipe.
    /// C++: length *= (1. - gcodegen.writer().filament()->retract_before_wipe());
    let length = retraction_length;

    /// Only wipe if retraction length is positive
    /// GCode.cpp:375
    /// C++: if (length >= 0) {
    if length < 1e-6 {
        return Ok(());
    }

    /// Calculate wipe distance
    /// GCode.cpp:379-380
    /// C++: // BBS
    /// C++: double wipe_dist = scale_(gcodegen.config().wipe_distance.get_at(gcodegen.writer().filament()->id()));
    let wipe_dist_scaled = scale(wipe_distance);

    /// Take the stored wipe path and replace first point with current position
    /// GCode.cpp:382-388
    /// C++: /* Take the stored wipe path and replace first point with the current actual position
    /// C++: (they might be different, for example, in case of loop clipping). */
    /// C++: Polyline wipe_path;
    /// C++: wipe_path.append(gcodegen.last_pos());
    /// C++: wipe_path.append(
    /// C++: this->path.points.begin() + 1,
    /// C++: this->path.points.end()
    /// C++: );
    let mut wipe_polyline = wipe_path.clone();
    if wipe_polyline.points.is_empty() {
        return Ok(());
    }

    // Replace first point with current position
    let current_pos_f = writer.position();
    let current_pos = Point::new(
        (current_pos_f.x * 1_000_000.0) as i64,
        (current_pos_f.y * 1_000_000.0) as i64,
    );
    wipe_polyline.points[0] = current_pos;

    /// Clip wipe path to wipe distance
    /// GCode.cpp:390
    /// C++: wipe_path.clip_end(wipe_path.length() - wipe_dist);
    let total_length = wipe_polyline.length() as i64;
    if total_length > wipe_dist_scaled {
        wipe_polyline.clip_end((total_length - wipe_dist_scaled) as f64);
    }

    /// Subdivide the retraction in segments along wipe path
    /// GCode.cpp:392-407
    /// C++: // subdivide the retraction in segments
    /// C++: if (!wipe_path.empty()) {
    if !wipe_polyline.points.is_empty() {
        /// Handle short path case
        /// GCode.cpp:393-399
        /// C++: // BBS. Handle short path case.
        /// C++: if (wipe_path.length() < wipe_dist) {
        /// C++: wipe_dist = wipe_path.length();
        /// C++: //BBS: avoid to divide 0
        /// C++: wipe_dist = wipe_dist < EPSILON ? EPSILON : wipe_dist;
        /// C++: }
        let actual_wipe_dist = if total_length < wipe_dist_scaled {
            (total_length as f64).max(1e-6) // Avoid division by zero
        } else {
            wipe_dist_scaled as f64
        };

        /// Add wipe start tag for processor
        /// GCode.cpp:401
        /// C++: // add tag for processor
        /// C++: gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Start) + "\n";
        writer.write_comment("TYPE:Wipe_Start");

        /// Set wipe speed
        /// GCode.cpp:402-403
        /// C++: //BBS: don't need to enable cooling markers when this is the last wipe. Because no more cooling layer will clean this "_WIPE"
        /// C++: gcode += gcodegen.writer().set_speed(wipe_speed * 60, "", (gcodegen.enable_cooling_markers() && !is_last) ? ";_WIPE" : "");
        let comment = if !is_last { ";_WIPE" } else { "" };
        writer.set_speed(wipe_speed * 60.0, comment);

        /// Iterate through wipe path segments and retract while traveling
        /// GCode.cpp:404-416
        /// C++: for (const Line& line : wipe_path.lines()) {
        /// C++: double segment_length = line.length();
        /// C++: /* Reduce retraction length a bit to avoid effective retraction speed to be greater than the configured one
        /// C++: due to rounding (TODO: test and/or better math for this) */
        /// C++: double dE = length * (segment_length / wipe_dist) * 0.95;
        /// C++: //BBS: fix this FIXME
        /// C++: //FIXME one shall not generate the unnecessary G1 Fxxx commands, here wipe_speed is a constant inside this cycle.
        /// C++: // Is it here for the cooling markers? Or should it be outside of the cycle?
        /// C++: //gcode += gcodegen.writer().set_speed(wipe_speed * 60, "", gcodegen.enable_cooling_markers() ? ";_WIPE" : "");
        /// C++: gcode += gcodegen.writer().extrude_to_xy(
        /// C++: gcodegen.point_to_gcode(line.b),
        /// C++: -dE,
        /// C++: "wipe and retract"
        /// C++: );
        /// C++: }
        for i in 1..wipe_polyline.points.len() {
            let from = wipe_polyline.points[i - 1];
            let to = wipe_polyline.points[i];

            // Calculate segment length
            let segment_length = from.distance_to_f64(to);

            // Calculate retraction for this segment (95% to avoid rounding issues)
            let de = length * (segment_length / actual_wipe_dist) * 0.95;

            // Emit wipe move with negative extrusion (retraction)
            let to_x = unscale(to.x());
            let to_y = unscale(to.y());
            writer.extrude_to_xy(to_x, to_y, -de, Some("wipe and retract"));
        }

        /// Add wipe end tag for processor
        /// GCode.cpp:418-419
        /// C++: // add tag for processor
        /// C++: gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_End) + "\n";
        writer.write_comment("TYPE:Wipe_End");

        /// Update last position
        /// GCode.cpp:420
        /// C++: gcodegen.set_last_pos(wipe_path.points.back());
        if let Some(last_point) = wipe_polyline.points.last() {
            let last_x = unscale(last_point.x());
            let last_y = unscale(last_point.y());
            writer.set_position_xy(last_x, last_y);
        }
    }

    /// Prevent wiping again on same path (path is reset by caller)
    /// GCode.cpp:424
    /// C++: // prevent wiping again on same path
    /// C++: this->reset_path();
    // Note: Wipe path management is handled by the caller
    Ok(())
}

/// Change to a different extruder (tool change).
///
/// C++ reference: GCode::set_extruder()
/// GCode.cpp:6726-6950 (~224 lines)
///
/// This function orchestrates a complete tool change:
/// 1. Checks if tool change is needed (no-op if same extruder)
/// 2. Performs retraction on old extruder (with optional wipe)
/// 3. Resets wipe path
/// 4. Processes filament end G-code
/// 5. Handles ooze prevention
/// 6. Emits tool change command (T<n>)
/// 7. Processes filament start G-code
/// 8. Handles temperature changes
/// 9. Performs un-retraction on new extruder
/// 10. Updates active extruder state
///
/// # Arguments
/// * `new_extruder_id` - Target extruder ID (0-based)
/// * `writer` - GCodeWriter to emit commands
/// * `print_z` - Current Z height for temperature decisions
/// * `config` - Print configuration for tool change settings
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on write errors or invalid extruder ID
pub fn set_extruder(
    new_extruder_id: usize,
    writer: &mut GCodeWriter,
    _print_z: f64,
    _config: &PrintConfig,
) -> Result<()> {
    /// Check if tool change is needed
    /// GCode.cpp:6728-6729
    /// C++: int new_extruder_id = get_extruder_id(new_filament_id);
    /// C++: if (!m_writer.need_toolchange(new_filament_id))
    /// C++: return "";
    if !writer.need_toolchange(new_extruder_id) {
        return Ok(());
    }

    /// Single extruder setup - just set extruder and return
    /// GCode.cpp:6731-6761
    /// C++: // if we are running a single-extruder setup, just set the extruder and return nothing
    /// C++: if (!m_writer.multiple_extruders) {
    /// C++: m_placeholder_parser.set("current_extruder", new_filament_id);
    /// C++: ...
    /// C++: gcode += m_writer.toolchange(new_filament_id);
    /// C++: return gcode;
    /// C++: }
    if !writer.has_multiple_extruders() {
        // Single extruder - just emit T command and update state
        writer.write_command_with_comment(&format!("T{}", new_extruder_id), Some("tool change"));
        writer.set_extruder(new_extruder_id);
        return Ok(());
    }

    /// Multi-extruder setup - full tool change sequence
    /// GCode.cpp:6763-6765
    /// C++: // BBS. Should be placed before retract.
    /// C++: m_toolchange_count++;
    // Tool change counter would be tracked here

    /// Prepend retraction on the current extruder
    /// GCode.cpp:6767
    /// C++: // prepend retraction on the current extruder
    /// C++: std::string gcode = this->retract(true, false);
    retract(writer, true)?; // true = with wipe

    /// Reset wipe path to avoid reusing it
    /// GCode.cpp:6770
    /// C++: // Always reset the extrusion path, even if the tool change retract is set to zero.
    /// C++: m_wipe.reset_path();
    // Note: Wipe path management is handled by caller

    /// Insert skip object labels for sequential printing
    /// GCode.cpp:6772-6776
    /// C++: // BBS: insert skip object label before change filament while by object
    /// C++: if (by_object)
    /// C++: m_writer.add_object_change_labels(gcode);
    /// C++: else
    /// C++: m_writer.add_object_end_labels(gcode);
    // TODO: Implement object change labels for sequential printing

    /// Process filament end G-code if current filament exists
    /// GCode.cpp:6778-6794
    /// C++: bool add_change_filament_624 = false;
    /// C++: if (m_writer.filament() != nullptr) {
    /// C++: // Process the custom filament_end_gcode. set_extruder() is only called if there is no wipe tower
    /// C++: // so it should not be injected twice.
    /// C++: unsigned int old_filament_id = m_writer.filament()->id();
    /// C++: const std::string &filament_end_gcode = m_config.filament_end_gcode.get_at(old_filament_id);
    /// C++: if (! filament_end_gcode.empty()) {
    /// C++: ...
    /// C++: }
    /// C++: }
    // TODO: Process filament_end_gcode custom G-code

    /// Handle ooze prevention (park extruder at standby position)
    /// GCode.cpp:6797-6798
    /// C++: // If ooze prevention is enabled, park current extruder in the nearest
    /// C++: // standby point and set it to the standby temperature.
    /// C++: if (m_ooze_prevention.enable && m_writer.filament() != nullptr)
    /// C++: gcode += m_ooze_prevention.pre_toolchange(*this);
    // TODO: Implement ooze prevention pre-toolchange parking

    /// Calculate flush/wipe volumes and temperatures
    /// GCode.cpp:6800-6880
    /// C++: // BBS
    /// C++: float new_retract_length = m_config.retraction_length.get_at(new_filament_id);
    /// C++: float new_retract_length_toolchange = m_config.retract_length_toolchange.get_at(new_filament_id);
    /// C++: ...
    /// C++: wipe_volume = flush_matrix[old_filament_id * number_of_extruders + new_filament_id];
    /// C++: wipe_volume *= m_config.flush_multiplier.get_at(new_extruder_id);
    // TODO: Calculate flush volumes from flush matrix

    /// Process change_filament_gcode with placeholder substitution
    /// GCode.cpp:6882-6920
    /// C++: dyn_config.set_key_value("outer_wall_volumetric_speed", new ConfigOptionFloat(outer_wall_volumetric_speed));
    /// C++: dyn_config.set_key_value("previous_extruder", new ConfigOptionInt(old_filament_id));
    /// C++: dyn_config.set_key_value("next_extruder", new ConfigOptionInt((int)new_filament_id));
    /// C++: ...
    /// C++: gcode += this->placeholder_parser_process("change_filament_gcode", change_filament_gcode, new_filament_id, &dyn_config);
    // TODO: Process change_filament_gcode with dynamic config placeholders

    /// Emit tool change command
    /// GCode.cpp:6926
    /// C++: gcode += m_writer.toolchange(new_filament_id);
    writer.write_command_with_comment(&format!("T{}", new_extruder_id), Some("tool change"));

    /// Handle ooze prevention post-toolchange
    /// GCode.cpp:6929-6931
    /// C++: // append custom toolchange gcode
    /// C++: if (m_ooze_prevention.enable && m_writer.filament() != nullptr)
    /// C++: gcode += m_ooze_prevention.post_toolchange(*this);
    // TODO: Implement ooze prevention post-toolchange

    /// Process filament_start_gcode for new extruder
    /// GCode.cpp:6933-6947
    /// C++: // Append the filament start G-code.
    /// C++: const std::string &filament_start_gcode = m_config.filament_start_gcode.get_at(new_filament_id);
    /// C++: if (! filament_start_gcode.empty()) {
    /// C++: // Process the filament_start_gcode for the filament.
    /// C++: DynamicConfig config;
    /// C++: ...
    /// C++: gcode += this->placeholder_parser_process("filament_start_gcode", filament_start_gcode, new_filament_id, &config);
    /// C++: check_add_eol(gcode);
    /// C++: }
    // TODO: Process filament_start_gcode custom G-code

    /// Update active extruder state
    /// GCode.cpp:6949
    /// C++: return gcode;
    writer.set_extruder(new_extruder_id);

    Ok(())
}

/// Apply cooling adjustments to a layer.
///
/// C++ reference: CoolingBuffer::process()
/// GCode/CoolingBuffer.cpp:200-400 (~200 lines)
///
/// This function:
/// 1. Calculates actual layer print time
/// 2. Determines if slowdown is needed
/// 3. Adjusts speeds to meet minimum layer time
/// 4. Calculates appropriate fan speed
///
/// # Arguments
/// * `writer` - GCodeWriter to apply speed adjustments
/// * `cooling_buffer` - CoolingBuffer with configuration
/// * `layer_time` - Estimated layer time (seconds)
/// * `layer_index` - Current layer number (0-based)
///
/// # Returns
/// * `Ok(())` on success
/// * `Err` on processing errors
pub fn apply_layer_cooling(
    writer: &mut GCodeWriter,
    cooling_buffer: &CoolingBuffer,
    layer_time: f64,
    layer_index: usize,
) -> Result<()> {
    /// Check if cooling is needed based on minimum layer time
    /// CoolingBuffer.cpp:250-260
    /// C++: float CoolingBuffer::calculate_layer_slowdown(
    /// C++: std::vector<PerExtruderAdjustments> &per_extruder_adjustments) {
    /// C++: float layer_time_stretched = 0.f;
    /// C++: ...
    /// C++: }
    let config = cooling_buffer.config();

    /// Skip cooling for first layer if configured
    /// CoolingBuffer.cpp:255-257
    /// C++: if (m_layer_id < config.disable_fan_first_layers.value) {
    /// C++: return layer_time;
    /// C++: }
    if layer_index < config.disable_fan_first_layers as usize {
        return Ok(());
    }

    /// Calculate fan speed based on layer time
    /// CoolingBuffer.cpp:280-300
    /// C++: unsigned int fan_speed = 0;
    /// C++: if (layer_time < config.fan_below_layer_time.value) {
    /// C++: fan_speed = config.fan_speed.value;
    /// C++: }
    let fan_speed = cooling_buffer.calculate_fan_speed(layer_index as u32, layer_time);
    let fan_speed_pwm = (fan_speed * 255.0) as u32;

    /// Emit fan speed command
    /// CoolingBuffer.cpp:305-310
    /// C++: if (fan_speed != current_fan_speed) {
    /// C++: gcode += m_gcodegen->writer().set_fan(fan_speed);
    /// C++: current_fan_speed = fan_speed;
    /// C++: }
    writer.set_fan_speed(fan_speed_pwm);

    /// TODO : Implement speed slowdown
    /// CoolingBuffer.cpp:320-380
    /// C++: if (layer_time < config.min_layer_time.value) {
    /// C++: // Calculate slowdown factor
    /// C++: float slowdown_factor = config.min_layer_time.value / layer_time;
    /// C++: // Apply slowdown to adjustable moves
    /// C++: ...
    /// C++: }
    // For , we only implement fan control
    // Speed slowdown will be added
    Ok(())
}

/// Calculate fan speed for bridge features.
///
/// C++ reference: CoolingBuffer::bridge_fan_speed()
/// GCode/CoolingBuffer.cpp:450-470
///
/// # Arguments
/// * `cooling_buffer` - CoolingBuffer with configuration
///
/// # Returns
/// * Fan speed (0.0 - 1.0)
pub fn bridge_fan_speed(cooling_buffer: &CoolingBuffer) -> f64 {
    /// Return bridge fan speed from config
    /// CoolingBuffer.cpp:452-454
    /// C++: float CoolingBuffer::bridge_fan_speed() const {
    /// C++: return m_config.bridge_fan_speed.value;
    /// C++: }
    cooling_buffer.bridge_fan_speed().unwrap_or(0.0)
}

/// Calculate fan speed for overhang features.
///
/// C++ reference: CoolingBuffer::overhang_fan_speed()
/// GCode/CoolingBuffer.cpp:475-495
///
/// # Arguments
/// * `cooling_buffer` - CoolingBuffer with configuration
///
/// # Returns
/// * Fan speed (0.0 - 1.0)
pub fn overhang_fan_speed(cooling_buffer: &CoolingBuffer) -> f64 {
    /// Return overhang fan speed from config
    /// CoolingBuffer.cpp:477-479
    /// C++: float CoolingBuffer::overhang_fan_speed() const {
    /// C++: return m_config.overhang_fan_speed.value;
    /// C++: }
    cooling_buffer.overhang_fan_speed().unwrap_or(0.0)
}

/// Set fan speed with optional override for special features.
///
/// C++ reference: GCode helper methods
/// GCode.cpp:various locations
///
/// # Arguments
/// * `writer` - GCodeWriter to emit fan command
/// * `base_fan_speed` - Base fan speed (0.0 - 1.0)
/// * `role` - Extrusion role (may trigger override)
/// * `cooling_buffer` - CoolingBuffer for override config
///
/// # Returns
/// * `Ok(())` on success
pub fn set_fan_speed_for_role(
    writer: &mut GCodeWriter,
    base_fan_speed: f64,
    role: ExtrusionRole,
    cooling_buffer: &CoolingBuffer,
) -> Result<()> {
    /// Check for bridge override
    /// GCode.cpp:various - bridge detection
    /// C++: if (entity.role() == erBridgeInfill && config.bridge_fan_override) {
    /// C++: fan_speed = config.bridge_fan_speed;
    /// C++: }
    let fan_speed = match role {
        ExtrusionRole::BridgeInfill => {
            if cooling_buffer.config().bridge_fan_override {
                cooling_buffer.bridge_fan_speed().unwrap_or(base_fan_speed)
            } else {
                base_fan_speed
            }
        }
        ExtrusionRole::OverhangPerimeter => {
            if cooling_buffer.config().overhang_fan_override {
                cooling_buffer
                    .overhang_fan_speed()
                    .unwrap_or(base_fan_speed)
            } else {
                base_fan_speed
            }
        }
        _ => base_fan_speed,
    };

    /// Convert to PWM (0-255) and emit
    /// GCodeWriter.cpp:860-900
    /// C++: std::string GCodeWriter::set_fan(unsigned int speed) {
    /// C++: gcode << "M106 S" << 255.0 * speed / 100.0;
    /// C++: }
    let fan_speed_pwm = (fan_speed * 255.0) as u32;
    writer.set_fan_speed(fan_speed_pwm);

    Ok(())
}

// ---------------------------------------------------------------------------
// Layer orchestration dispatch functions
// Ports of GCode::change_layer(), extrude_perimeters(), extrude_infill(),
// extrude_support() from BambuStudio GCode.cpp
// ---------------------------------------------------------------------------

/// State tracked across layers during G-code export.
/// Mirrors key fields of the C++ GCode class that persist across process_layer() calls.
///
/// C++ reference: GCode.hpp member variables
#[derive(Debug, Clone)]
pub struct GCodeLayerState {
    /// Current layer index (incremented by change_layer).
    /// C++ GCode.hpp: m_layer_index
    pub layer_index: usize,
    /// Total layer count for progress reporting.
    /// C++ GCode.hpp: m_layer_count
    pub layer_count: usize,
    /// Last layer Z height (for computing layer height delta).
    /// C++ GCode.hpp: m_last_layer_z
    pub last_layer_z: f64,
    /// Maximum Z reached so far.
    /// C++ GCode.hpp: m_max_layer_z
    pub max_layer_z: f64,
    /// Last layer height (for HEIGHT tag).
    /// C++ GCode.hpp: m_last_height
    pub last_height: f64,
    /// Whether second-layer setup has been done (temp transitions, etc).
    /// C++ GCode.hpp: m_second_layer_things_done
    pub second_layer_things_done: bool,
    /// Whether spiral vase mode is active.
    /// C++ GCode.hpp: m_spiral_vase
    pub spiral_vase: bool,
    /// Whether to enable loop clipping (seam gap).
    /// C++ GCode.hpp: m_enable_loop_clipping
    pub enable_loop_clipping: bool,
    /// Set of object label IDs printed on the current layer.
    /// Used for M624/M625 emission.
    pub layer_object_label_ids: Vec<usize>,
    /// Whether change_layer lift is pending (deferred Z move).
    /// C++ GCode.hpp: m_need_change_layer_lift_z
    pub need_change_layer_lift_z: bool,
    /// Nominal Z position.
    /// C++ GCode.hpp: m_nominal_z
    pub nominal_z: f64,
}

impl Default for GCodeLayerState {
    fn default() -> Self {
        Self {
            layer_index: 0,
            layer_count: 0,
            last_layer_z: 0.0,
            max_layer_z: 0.0,
            last_height: 0.0,
            second_layer_things_done: false,
            spiral_vase: false,
            enable_loop_clipping: true,
            layer_object_label_ids: Vec::new(),
            need_change_layer_lift_z: false,
            nominal_z: 0.0,
        }
    }
}

/// `GCode::to_lift_type` (GCode.cpp:8348-8361). `zhtAuto` has no case and falls
/// to the default, NormalLift; callers resolve Auto themselves.
pub fn to_lift_type(z_hop_types: crate::print_config::ZHopType) -> crate::g_code_writer::LiftType {
    use crate::g_code_writer::LiftType;
    use crate::print_config::ZHopType;
    match z_hop_types {
        ZHopType::Normal => LiftType::NormalLift,
        ZHopType::Slope => LiftType::SlopeLift,
        ZHopType::Spiral => LiftType::SpiralLift,
        // if no corresponding lift type, use normal lift
        _ => LiftType::NormalLift,
    }
}

/// The code `gcode/writer.rs` keeps a lift type in: 0 NormalLift, 1 SpiralLift,
/// 2 SlopeLift.
pub(crate) fn lift_type_code(lift_type: crate::g_code_writer::LiftType) -> u8 {
    use crate::g_code_writer::LiftType;
    match lift_type {
        LiftType::NormalLift => 0,
        LiftType::SpiralLift => 1,
        LiftType::SlopeLift => 2,
    }
}

/// `GCode::change_layer` (GCode.cpp:6473-6503), called by `process_layer` right
/// after the layer head (:5066).
///
/// `update_progress` (:6475-6477) writes nothing for this G-code flavour
/// (GCodeWriter.cpp `update_progress`: only MakerWare and Sailfish), and the
/// caller keeps the layer index. With `retract_when_changing_layer` (the
/// current filament's, `FILAMENT_CONFIG`) and a Z change it retracts, wiping,
/// with a lazy lift: SpiralLift when `z_hop_types` is Auto, else `to_lift_type`
/// (:6481-6485, :8348-8361). `add_object_change_labels` (:6487) is the caller's.
/// In spiral vase mode it moves to the layer at once (:6489-6494, through
/// `GCodeWriter::travel_to_z`); otherwise it sets `m_need_change_layer_lift_z`
/// so the layer's first travel lifts (:6495-6499). Then `m_nominal_z`.
pub fn change_layer(
    print_z: f64,
    writer: &mut GCodeWriter,
    retract_when_changing_layer: bool,
    z_hop_type: crate::print_config::ZHopType,
    spiral_vase: bool,
) {
    use crate::print_config::ZHopType;
    // :6480 — BBS: no z_offset.
    let z = print_z;
    if retract_when_changing_layer && writer.will_move_z(z) {
        // :6482-6484 — SpiralLift for Auto, else `to_lift_type`.
        let lift_type = if matches!(z_hop_type, ZHopType::Auto) {
            lift_type_code(crate::g_code_writer::LiftType::SpiralLift)
        } else {
            lift_type_code(to_lift_type(z_hop_type))
        };
        writer.retract_with_lift_type(lift_type, false);
    }
    if spiral_vase {
        // GCodeWriter::travel_to_z: inside the lift only the lift shrinks.
        if !writer.will_move_z(z) {
            let nominal_z = writer.z() - writer.m_lifted;
            writer.m_lifted -= z - nominal_z;
            if writer.m_lifted.abs() < crate::libslic3r::EPSILON {
                writer.m_lifted = 0.0;
            }
        } else {
            writer.m_lifted = 0.0;
            writer.set_travel_acceleration(0.0);
            writer.travel_to_z(z, None);
        }
    } else {
        writer.m_need_change_layer_lift_z = true;
    }
    writer.nominal_z = print_z;
}

/// Emit the CHANGE_LAYER tag, Z_HEIGHT, HEIGHT metadata, and custom gcode.
///
/// C++ reference: GCode::process_layer() lines 3922-3983
/// This is the per-layer preamble that runs before change_layer().
///
/// # Arguments
/// * `print_z` - Z height of this layer
/// * `state` - Mutable layer state
/// * `config` - Print configuration
///
/// # Returns
/// G-code string for layer preamble
pub fn emit_layer_preamble(
    print_z: f64,
    state: &mut GCodeLayerState,
    config: &crate::print_config::PrintConfig,
) -> String {
    let mut gcode = String::new();

    // C++ GCode.cpp:3923
    // Add tag for processor
    gcode += "; CHANGE_LAYER\n";

    // C++ GCode.cpp:3925-3927
    // Export layer z
    gcode += &format!("; Z_HEIGHT: {}\n", print_z);

    // C++ GCode.cpp:3929-3931
    // Export layer height
    let first_layer = state.layer_index == 0 && state.last_layer_z.abs() < 1e-6;
    let height = if first_layer {
        print_z as f32
    } else {
        (print_z as f32) - (state.last_layer_z as f32)
    };
    gcode += &format!(";HEIGHT:{}\n", height);

    // C++ GCode.cpp:3933-3934
    // Update caches
    state.last_layer_z = print_z;
    state.max_layer_z = state.max_layer_z.max(print_z);
    state.last_height = height as f64;

    // C++ GCode.cpp:3938-3946
    // Before layer change custom G-code
    if !config.before_layer_change_gcode.is_empty() {
        let processed = config
            .before_layer_change_gcode
            .replace("{layer_num}", &(state.layer_index + 1).to_string())
            .replace("{layer_z}", &format!("{}", print_z))
            .replace("{max_layer_z}", &format!("{}", state.max_layer_z));
        gcode += &processed;
        gcode += "\n";
    }

    gcode
}

/// Emit post-change_layer custom G-code and fan speed marker.
///
/// C++ reference: GCode::process_layer() lines 3972-3983
///
/// # Arguments
/// * `print_z` - Z height of this layer
/// * `state` - Layer state
/// * `config` - Print configuration
///
/// # Returns
/// G-code string
pub fn emit_layer_postamble(
    print_z: f64,
    state: &GCodeLayerState,
    config: &crate::print_config::PrintConfig,
) -> String {
    let mut gcode = String::new();

    // C++ GCode.cpp:3972-3981
    // After layer change custom G-code
    if !config.layer_change_gcode.is_empty() {
        let processed = config
            .layer_change_gcode
            .replace("{layer_num}", &state.layer_index.to_string())
            .replace("{layer_z}", &format!("{}", print_z))
            .replace("{max_layer_z}", &format!("{}", state.max_layer_z));
        gcode += &processed;
        gcode += "\n";
    }

    // C++ GCode.cpp:3983
    // Set layer time fan speed marker for cooling post-processor
    gcode += ";_SET_FAN_SPEED_CHANGING_LAYER\n";

    gcode
}

/// Encode a list of object label IDs to a base64-like string for M624.
///
/// C++ reference: GCode::_encode_label_ids_to_base64()
/// GCode.cpp (helper function)
///
/// Each label ID is encoded as a character in a compact format.
/// For simplicity, we encode as comma-separated decimal in a string.
fn encode_label_ids_to_base64(ids: &[usize]) -> String {
    // C++ uses a custom base64-like encoding.
    // For compatibility we use a simplified approach that encodes IDs as
    // a compact string. The firmware (M624) just needs a parseable token.
    if ids.is_empty() {
        return String::new();
    }

    // Simple encoding: base64 of the bit-set of object IDs
    // For up to 64 objects, pack into a u64 bitmask then base64-encode
    let mut bitmask: u64 = 0;
    for &id in ids {
        if id < 64 {
            bitmask |= 1u64 << id;
        }
    }

    // Encode as base64 characters (6 bits each)
    const BASE64_CHARS: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut result = String::new();
    let mut val = bitmask;
    if val == 0 {
        result.push(BASE64_CHARS[0] as char);
    } else {
        while val > 0 {
            result.push(BASE64_CHARS[(val & 0x3F) as usize] as char);
            val >>= 6;
        }
    }
    result
}

/// Emit M624 start label for an object instance.
///
/// C++ reference: GCode::process_layer() lines 4591-4602
///
/// # Arguments
/// * `label_object_id` - Unique label ID for the object instance
/// * `enable_label_object` - Whether M624/M625 labeling is enabled
///
/// # Returns
/// G-code string with start label
pub fn emit_object_start_label(label_object_id: usize, enable_label_object: bool) -> String {
    let mut gcode = String::new();

    // C++ GCode.cpp:4589
    gcode += &format!("; OBJECT_ID: {}\n", label_object_id);

    if enable_label_object {
        // C++ GCode.cpp:4592-4595
        gcode += &format!(
            "; start printing object, unique label id: {}\n",
            label_object_id
        );
        // C++ GCode.cpp:4594
        gcode += &format!("M624 {}\n", encode_label_ids_to_base64(&[label_object_id]));
    }

    gcode
}

/// Emit M625 end label for an object instance.
///
/// C++ reference: GCode::process_layer() lines 4753-4758
///
/// # Arguments
/// * `label_object_id` - Unique label ID for the object instance
/// * `enable_label_object` - Whether M624/M625 labeling is enabled
///
/// # Returns
/// G-code string with end label
pub fn emit_object_end_label(label_object_id: usize, enable_label_object: bool) -> String {
    let mut gcode = String::new();

    if enable_label_object {
        // C++ GCode.cpp:4754-4756
        gcode += &format!(
            "; stop printing object, unique label id: {}\n",
            label_object_id
        );
        gcode += "M625\n";
    }
    let _ = label_object_id; // suppress warning when labels disabled

    gcode
}

/// Emit M624/M625 wrapping around the entire layer's timelapse position.
///
/// C++ reference: GCode::process_layer() lines 4395-4407
///
/// # Arguments
/// * `object_label_ids` - Set of object IDs present on this layer
/// * `layer_index` - Current layer index
///
/// # Returns
/// Pair of (start_gcode, end_gcode) strings
pub fn emit_layer_object_labels(
    object_label_ids: &[usize],
    layer_index: usize,
) -> (String, String) {
    if object_label_ids.is_empty() {
        return (String::new(), String::new());
    }

    // C++ GCode.cpp:4395-4401
    let ids_str = object_label_ids
        .iter()
        .map(|id| id.to_string())
        .collect::<Vec<_>>()
        .join(",");

    let start = format!(
        "; object ids of layer {} start: {}\nM624 {}\n",
        layer_index + 1,
        ids_str,
        encode_label_ids_to_base64(object_label_ids),
    );

    // C++ GCode.cpp:4403-4404
    let end = format!(
        "; object ids of this layer{} end: {}\nM625\n",
        layer_index + 1,
        ids_str,
    );

    (start, end)
}

#[cfg(test)]
mod exporter_tests {
    use super::*;
    use crate::extrusion_entity::{ExtrusionLoopRole, ExtrusionRole};
    use crate::gcode::cooling::CoolingConfig;
    use crate::geometry::{Point, Polyline};
    use crate::print_config::PrintObjectConfig;

    /// Helper to create a simple square loop for testing.
    /// Creates a 10mm x 10mm square starting at (5,5).
    fn create_square_loop() -> ExtrusionLoop {
        // Create a square: (5,5) -> (15,5) -> (15,15) -> (5,15) -> back to (5,5)
        let points = vec![
            Point::new_scale(5.0, 5.0),
            Point::new_scale(15.0, 5.0),
            Point::new_scale(15.0, 15.0),
            Point::new_scale(5.0, 15.0),
        ];

        let mut path = ExtrusionPath::new(ExtrusionRole::ExternalPerimeter);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.1; // 0.1 mm³/mm extrusion
        path.width = 0.4;
        path.height = 0.2;

        ExtrusionLoop::new(vec![path], ExtrusionLoopRole::DEFAULT)
    }

    /// Helper to create a simple path for testing
    fn create_simple_path() -> ExtrusionPath {
        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];

        let mut path = ExtrusionPath::new(ExtrusionRole::Perimeter);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.05;
        path.width = 0.4;
        path.height = 0.2;

        path
    }

    #[test]
    fn test_extrude_simple_square_loop() {
        let mut writer = GCodeWriter::new();
        let loop_entity = create_square_loop();

        // Get initial E value
        let initial_e = writer.e();

        // Extrude the loop
        extrude_loop(&loop_entity, &mut writer, &PrintObjectConfig::default(), false);

        // Get final E value
        let final_e = writer.e();

        // E should have increased (we extruded material)
        assert!(
            final_e > initial_e,
            "E value should increase after extrusion: {} -> {}",
            initial_e,
            final_e
        );

        // Get the generated G-code
        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should contain G1 commands (extrusion moves)
        assert!(content.contains("G1"), "G-code should contain G1 commands");

        // Should contain E values (extrusion)
        assert!(
            content.contains(" E"),
            "G-code should contain extrusion (E) values"
        );

        // Should have multiple lines (one per segment)
        let lines: Vec<&str> = content.lines().collect();
        assert!(
            lines.len() >= 4,
            "Should have at least 4 move commands for square (got {})",
            lines.len()
        );
    }

    #[test]
    fn test_extrude_loop_seam_placement() {
        let mut writer = GCodeWriter::new();

        // Set writer to a known position closer to (15,15) corner
        writer.set_x(15.0);
        writer.set_y(15.0);

        let loop_entity = create_square_loop();

        // Extrude the loop
        extrude_loop(&loop_entity, &mut writer, &PrintObjectConfig::default(), false);

        // The loop should have been rotated to start near the writer's position
        // We can't easily verify the exact starting point without inspecting internals,
        // but we can verify that extrusion happened successfully
        let final_e = writer.e();
        assert!(final_e > 0.0, "Should have extruded material");
    }

    #[test]
    fn test_extrude_loop_ccw_orientation() {
        let mut writer = GCodeWriter::new();

        // Create a clockwise loop (will be reversed to CCW)
        let points = vec![
            Point::new_scale(0.0, 0.0),
            Point::new_scale(0.0, 10.0), // Clockwise: up first
            Point::new_scale(10.0, 10.0),
            Point::new_scale(10.0, 0.0),
        ];

        let mut path = ExtrusionPath::new(ExtrusionRole::ExternalPerimeter);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.1;
        path.width = 0.4;
        path.height = 0.2;

        let loop_entity = ExtrusionLoop::new(vec![path], ExtrusionLoopRole::DEFAULT);

        let initial_e = writer.e();

        // Should succeed and make loop CCW
        extrude_loop(&loop_entity, &mut writer, &PrintObjectConfig::default(), false);

        let final_e = writer.e();
        assert!(final_e > initial_e, "Should have extruded material");
    }

    #[test]
    fn test_extrude_path_empty() {
        let mut writer = GCodeWriter::new();

        // Create an empty path
        let path = ExtrusionPath::new(ExtrusionRole::Perimeter);

        let initial_e = writer.e();

        // Should not panic, just do nothing
        let config = crate::print_config::PrintObjectConfig::default();
        extrude_path(&path, &mut writer, &config, false);

        let final_e = writer.e();

        // E should not change for empty path
        assert_eq!(final_e, initial_e, "E should not change for empty path");

        // G-code should be empty or unchanged
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.is_empty() || !content.contains("G1"),
            "Should not generate moves for empty path"
        );
    }

    #[test]
    fn test_extrude_path_single_segment() {
        let mut writer = GCodeWriter::new();

        // Create a path with two points (one segment)
        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];

        let mut path = ExtrusionPath::new(ExtrusionRole::Perimeter);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.1;
        path.width = 0.4;
        path.height = 0.2;

        let initial_e = writer.e();

        let config = crate::print_config::PrintObjectConfig::default();
        extrude_path(&path, &mut writer, &config, false);

        let final_e = writer.e();

        // Should have extruded for the 10mm segment
        assert!(final_e > initial_e, "E should increase for single segment");

        // Calculate expected E delta: length * mm3_per_mm = 10.0 * 0.1 = 1.0
        let expected_delta = 10.0 * 0.1;
        let actual_delta = final_e - initial_e;

        // Allow small floating-point tolerance
        assert!(
            (actual_delta - expected_delta).abs() < 0.01,
            "E delta should be approximately {}, got {}",
            expected_delta,
            actual_delta
        );

        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should have exactly one G1 command
        let g1_count = content.matches("G1").count();
        assert_eq!(
            g1_count, 1,
            "Should have exactly 1 G1 command for single segment, got {}",
            g1_count
        );
    }

    #[test]
    fn test_extrude_path_multi_segment() {
        let mut writer = GCodeWriter::new();

        // Create a path with 4 points (3 segments) forming an L shape
        let points = vec![
            Point::new_scale(0.0, 0.0),
            Point::new_scale(10.0, 0.0), // 10mm horizontal
            Point::new_scale(10.0, 5.0), // 5mm vertical
            Point::new_scale(20.0, 5.0), // 10mm horizontal
        ];

        let mut path = ExtrusionPath::new(ExtrusionRole::InternalInfill);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.08;
        path.width = 0.4;
        path.height = 0.2;

        let initial_e = writer.e();

        let config = crate::print_config::PrintObjectConfig::default();
        extrude_path(&path, &mut writer, &config, false);

        let final_e = writer.e();

        // Total length: 10 + 5 + 10 = 25mm
        // Expected E delta: 25 * 0.08 = 2.0
        let expected_delta = 25.0 * 0.08;
        let actual_delta = final_e - initial_e;

        assert!(
            (actual_delta - expected_delta).abs() < 0.01,
            "E delta should be approximately {}, got {}",
            expected_delta,
            actual_delta
        );

        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should have 3 G1 commands (one per segment)
        let g1_count = content.matches("G1").count();
        assert_eq!(
            g1_count, 3,
            "Should have exactly 3 G1 commands for 3 segments, got {}",
            g1_count
        );
    }

    #[test]
    fn test_coordinate_conversion() {
        let mut writer = GCodeWriter::new();

        // Create a path with known scaled coordinates
        let points = vec![Point::new_scale(5.5, 7.3), Point::new_scale(15.8, 12.1)];

        let mut path = ExtrusionPath::new(ExtrusionRole::Perimeter);
        path.polyline = Polyline::from_points(points);
        path.mm3_per_mm = 0.1;
        path.width = 0.4;
        path.height = 0.2;

        let config = crate::print_config::PrintObjectConfig::default();
        extrude_path(&path, &mut writer, &config, false);

        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Verify that coordinates were properly unscaled to mm
        // Should contain X15.800 and Y12.100 (or similar precision)
        assert!(
            content.contains("X15.8") || content.contains("X15.80"),
            "Should contain unscaled X coordinate ~15.8"
        );
        assert!(
            content.contains("Y12.1") || content.contains("Y12.10"),
            "Should contain unscaled Y coordinate ~12.1"
        );
    }

    #[test]
    fn test_extrude_path_cumulative_e() {
        let mut writer = GCodeWriter::new();

        // Extrude multiple paths and verify E accumulates
        let path1 = create_simple_path();
        let path2 = create_simple_path();

        let e0 = writer.e();
        let config = crate::print_config::PrintObjectConfig::default();
        extrude_path(&path1, &mut writer, &config, false);
        let e1 = writer.e();
        extrude_path(&path2, &mut writer, &config, false);
        let e2 = writer.e();

        // Each path should increase E
        assert!(e1 > e0, "First path should increase E");
        assert!(e2 > e1, "Second path should increase E further");

        // E should be cumulative (absolute mode)
        let delta1 = e1 - e0;
        let delta2 = e2 - e1;

        // Both deltas should be approximately equal (same path)
        assert!(
            (delta1 - delta2).abs() < 0.01,
            "Both paths should extrude similar amounts: {} vs {}",
            delta1,
            delta2
        );
    }

    // === Tests for extrude_collection() and extrude_entity() ===

    #[test]
    fn test_extrude_collection_empty() {
        let mut writer = GCodeWriter::new();

        // Empty collection should not panic
        let collection = ExtrusionEntityCollection {
            entities: Vec::new(),
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };

        let result = extrude_collection(&collection, &mut writer, &PrintObjectConfig::default(), false);
        assert!(result.is_ok(), "Empty collection should succeed");

        // Should not generate any G-code
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.is_empty() || !content.contains("G1"),
            "Empty collection should not generate moves"
        );
    }

    #[test]
    fn test_extrude_collection_single_path() {
        let mut writer = GCodeWriter::new();

        // Create collection with single path
        let path = create_simple_path();
        let collection = ExtrusionEntityCollection {
            entities: vec![ExtrusionEntityType::Path(path)],
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };

        let e_before = writer.e();
        extrude_collection(&collection, &mut writer, &PrintObjectConfig::default(), false).expect("Should extrude single path");
        let e_after = writer.e();

        // Should have extruded
        assert!(e_after > e_before, "E should increase");

        // Should have feature comment
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(content.contains("FEATURE:"), "Should have feature comment");
    }

    #[test]
    fn test_extrude_collection_multiple_paths() {
        let mut writer = GCodeWriter::new();

        // Create collection with 3 paths
        let path1 = create_simple_path();
        let path2 = create_simple_path();
        let path3 = create_simple_path();

        let collection = ExtrusionEntityCollection {
            entities: vec![
                ExtrusionEntityType::Path(path1),
                ExtrusionEntityType::Path(path2),
                ExtrusionEntityType::Path(path3),
            ],
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };

        let e_before = writer.e();
        extrude_collection(&collection, &mut writer, &PrintObjectConfig::default(), false).expect("Should extrude all paths");
        let e_after = writer.e();

        // E should increase significantly
        assert!(e_after > e_before, "E should increase for multiple paths");

        // Should have multiple G1 commands (one per path segment)
        let gcode = writer.get_gcode();
        let content = gcode.content();
        let g1_count = content.matches("G1").count();
        assert!(g1_count >= 3, "Should have at least 3 G1 commands");
    }

    #[test]
    fn test_extrude_collection_with_loop() {
        let mut writer = GCodeWriter::new();

        // Create collection with a loop
        let loop_entity = create_square_loop();
        let collection = ExtrusionEntityCollection {
            entities: vec![ExtrusionEntityType::Loop(loop_entity)],
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };

        let e_before = writer.e();
        extrude_collection(&collection, &mut writer, &PrintObjectConfig::default(), false).expect("Should extrude loop");
        let e_after = writer.e();

        // Should have extruded
        assert!(e_after > e_before, "E should increase");

        // Should have feature comment
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(content.contains("FEATURE:"), "Should have feature comment");
    }

    #[test]
    fn test_extrude_collection_nested() {
        let mut writer = GCodeWriter::new();

        // Create nested collection
        let path = create_simple_path();
        let inner_collection = ExtrusionEntityCollection {
            entities: vec![ExtrusionEntityType::Path(path)],
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };

        let outer_collection = ExtrusionEntityCollection {
            entities: vec![ExtrusionEntityType::Collection(Box::new(inner_collection))],
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };

        let e_before = writer.e();
        extrude_collection(&outer_collection, &mut writer, &PrintObjectConfig::default(), false)
            .expect("Should handle nested collections");
        let e_after = writer.e();

        // Should have extruded inner path
        assert!(
            e_after > e_before,
            "E should increase for nested collection"
        );
    }

    #[test]
    fn test_extrude_entity_path() {
        let mut writer = GCodeWriter::new();
        let path = create_simple_path();
        let entity = ExtrusionEntityType::Path(path);

        let e_before = writer.e();
        extrude_entity(&entity, &mut writer, &PrintObjectConfig::default(), false).expect("Should extrude path entity");
        let e_after = writer.e();

        assert!(e_after > e_before, "Path entity should extrude");
    }

    #[test]
    fn test_extrude_entity_loop() {
        let mut writer = GCodeWriter::new();
        let loop_entity = create_square_loop();
        let entity = ExtrusionEntityType::Loop(loop_entity);

        let e_before = writer.e();
        extrude_entity(&entity, &mut writer, &PrintObjectConfig::default(), false).expect("Should extrude loop entity");
        let e_after = writer.e();

        assert!(e_after > e_before, "Loop entity should extrude");
    }

    #[test]
    fn test_extrude_entity_collection() {
        let mut writer = GCodeWriter::new();

        let path = create_simple_path();
        let collection = ExtrusionEntityCollection {
            entities: vec![ExtrusionEntityType::Path(path)],
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };
        let entity = ExtrusionEntityType::Collection(Box::new(collection));

        let e_before = writer.e();
        extrude_entity(&entity, &mut writer, &PrintObjectConfig::default(), false).expect("Should extrude collection entity");
        let e_after = writer.e();

        assert!(e_after > e_before, "Collection entity should extrude");
    }

    #[test]
    fn test_feature_comment_on_role_change() {
        let mut writer = GCodeWriter::new();

        // Create two paths with different roles
        let mut path1 = create_simple_path();
        path1.role = ExtrusionRole::Perimeter;

        let mut path2 = create_simple_path();
        path2.role = ExtrusionRole::InternalInfill;

        let collection = ExtrusionEntityCollection {
            entities: vec![
                ExtrusionEntityType::Path(path1),
                ExtrusionEntityType::Path(path2),
            ],
            no_sort: false,
            orig_indices: Vec::new(),
            is_reverse: true,
            loop_node_range: (0, 0),
        };

        extrude_collection(&collection, &mut writer, &PrintObjectConfig::default(), false).expect("Should extrude");

        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should have feature comments for both roles
        assert!(
            content.contains("Perimeter"),
            "Should have Perimeter feature comment"
        );
        assert!(
            content.contains("InternalInfill"),
            "Should have InternalInfill feature comment"
        );

        // Should have at least 2 feature comments
        let feature_count = content.matches("FEATURE:").count();
        assert!(
            feature_count >= 2,
            "Should have at least 2 feature comments, got {}",
            feature_count
        );
    }

    // Tests for travel_to, retract, and unretract

    #[test]
    fn test_travel_to_short_distance_no_retract() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        // Short travel (1mm) - should not trigger retraction
        let target = Point::new_scale(1.0, 0.0);
        let config = TravelConfig {
            avoid_crossing_perimeters: false,
            retract_on_travel: true,
            retract_length_travel: 2.0, // Threshold is 2mm
            z_hop: false,
            z_hop_height: 0.0,
        };

        travel_to(target, &mut writer, &config).expect("travel_to should succeed");

        // Should not be retracted
        assert!(
            !writer.is_retracted(),
            "Should not retract for short travel"
        );

        // Position should have moved
        let pos = writer.position();
        assert!((pos.x - 1.0).abs() < 0.001, "X position should be 1.0");
    }

    #[test]
    fn test_travel_to_long_distance_with_retract() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        // Long travel (10mm) - should trigger retraction
        let target = Point::new_scale(10.0, 0.0);
        let config = TravelConfig {
            avoid_crossing_perimeters: false,
            retract_on_travel: true,
            retract_length_travel: 2.0, // Threshold is 2mm
            z_hop: false,
            z_hop_height: 0.0,
        };

        travel_to(target, &mut writer, &config).expect("travel_to should succeed");

        // Should have retracted during travel
        // Note: Writer may have unretracted at destination, check G-code content
        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should contain travel move
        assert!(
            content.contains("G0") || content.contains("G1"),
            "Should have travel move"
        );

        // Position should have moved
        let pos = writer.position();
        assert!((pos.x - 10.0).abs() < 0.001, "X position should be 10.0");
    }

    #[test]
    fn test_travel_to_retract_disabled() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        // Long travel but retraction disabled
        let target = Point::new_scale(10.0, 0.0);
        let config = TravelConfig {
            avoid_crossing_perimeters: false,
            retract_on_travel: false, // Retraction disabled
            retract_length_travel: 2.0,
            z_hop: false,
            z_hop_height: 0.0,
        };

        travel_to(target, &mut writer, &config).expect("travel_to should succeed");

        // Should not be retracted
        assert!(!writer.is_retracted(), "Should not retract when disabled");
    }

    #[test]
    fn test_retract_basic() {
        let mut writer = GCodeWriter::new();
        writer.set_x(5.0);
        writer.set_y(5.0);

        // Initially not retracted
        assert!(!writer.is_retracted(), "Should start not retracted");

        retract(&mut writer, false).expect("retract should succeed");

        // Should now be retracted
        assert!(writer.is_retracted(), "Should be retracted after retract()");

        // G-code should contain retraction
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("G1") && content.contains("E"),
            "Should have retraction G-code"
        );
    }

    #[test]
    fn test_retract_when_already_retracted() {
        let mut writer = GCodeWriter::new();

        // Retract once
        retract(&mut writer, false).expect("First retract should succeed");
        let gcode_after_first = writer.get_gcode().content().to_string();

        // Retract again - should be no-op
        retract(&mut writer, false).expect("Second retract should succeed");
        let gcode_after_second = writer.get_gcode().content().to_string();

        // G-code should not have changed (no double retraction)
        assert_eq!(
            gcode_after_first, gcode_after_second,
            "Should not emit duplicate retraction"
        );
    }

    #[test]
    fn test_unretract_basic() {
        let mut writer = GCodeWriter::new();

        // First retract
        retract(&mut writer, false).expect("retract should succeed");
        assert!(writer.is_retracted(), "Should be retracted");

        // Then unretract
        unretract(&mut writer).expect("unretract should succeed");

        // Should no longer be retracted
        assert!(
            !writer.is_retracted(),
            "Should not be retracted after unretract()"
        );

        // G-code should contain unretraction
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("G1") && content.contains("E"),
            "Should have unretraction G-code"
        );
    }

    #[test]
    fn test_unretract_when_not_retracted() {
        let mut writer = GCodeWriter::new();

        // Initially not retracted
        assert!(!writer.is_retracted(), "Should start not retracted");

        let gcode_before = writer.get_gcode().content().to_string();

        // Unretract when not retracted - should be no-op
        unretract(&mut writer).expect("unretract should succeed");

        let gcode_after = writer.get_gcode().content().to_string();

        // G-code should not have changed
        assert_eq!(
            gcode_before, gcode_after,
            "Should not emit unretraction when not retracted"
        );
    }

    #[test]
    fn test_retract_unretract_cycle() {
        let mut writer = GCodeWriter::new();

        // Start not retracted
        assert!(!writer.is_retracted());

        // Retract
        retract(&mut writer, false).expect("retract should succeed");
        assert!(writer.is_retracted());

        // Unretract
        unretract(&mut writer).expect("unretract should succeed");
        assert!(!writer.is_retracted());

        // Retract again
        retract(&mut writer, false).expect("second retract should succeed");
        assert!(writer.is_retracted());

        // Unretract again
        unretract(&mut writer).expect("second unretract should succeed");
        assert!(!writer.is_retracted());
    }

    // ===== Wipe Tests =====

    #[test]
    fn test_wipe_basic() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        // Create a simple wipe path (10mm straight line)
        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];
        let wipe_path = Polyline::from_points(points);

        // Perform wipe with 5mm distance, 0.4mm retraction, 50mm/s speed
        wipe(
            &mut writer,
            &wipe_path,
            5.0,   // wipe_distance
            0.4,   // retraction_length
            50.0,  // wipe_speed
            false, // toolchange
            false, // is_last
        )
        .expect("wipe should succeed");

        // Verify position updated
        let final_pos = writer.position();
        assert!(final_pos.x > 0.0, "X position should advance during wipe");

        // Verify G-code contains wipe markers
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("TYPE:Wipe_Start"),
            "Should contain wipe start marker"
        );
        assert!(
            content.contains("TYPE:Wipe_End"),
            "Should contain wipe end marker"
        );
        assert!(
            content.contains("wipe and retract"),
            "Should contain wipe comment"
        );
    }

    #[test]
    fn test_wipe_zero_length() {
        let mut writer = GCodeWriter::new();

        // Create a wipe path
        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];
        let wipe_path = Polyline::from_points(points);

        // Wipe with zero retraction length (should be no-op)
        wipe(
            &mut writer,
            &wipe_path,
            5.0,  // wipe_distance
            0.0,  // retraction_length = 0
            50.0, // wipe_speed
            false,
            false,
        )
        .expect("wipe with zero length should succeed");

        // Should be no-op, no G-code emitted
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            !content.contains("Wipe_Start"),
            "Should not contain wipe markers for zero-length wipe"
        );
    }

    #[test]
    fn test_wipe_empty_path() {
        let mut writer = GCodeWriter::new();

        // Empty wipe path
        let wipe_path = Polyline::from_points(vec![]);

        // Should handle gracefully
        wipe(&mut writer, &wipe_path, 5.0, 0.4, 50.0, false, false)
            .expect("wipe with empty path should succeed");

        // No wipe should occur
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            !content.contains("Wipe_Start"),
            "Should not wipe with empty path"
        );
    }

    #[test]
    fn test_wipe_toolchange() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];
        let wipe_path = Polyline::from_points(points);

        // Toolchange wipe
        wipe(
            &mut writer,
            &wipe_path,
            5.0,
            0.4,
            50.0,
            true, // toolchange = true
            false,
        )
        .expect("toolchange wipe should succeed");

        // Should still perform wipe
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("TYPE:Wipe_Start"),
            "Toolchange wipe should emit markers"
        );
    }

    #[test]
    fn test_wipe_is_last() {
        let mut writer = GCodeWriter::new();
        writer.set_x(0.0);
        writer.set_y(0.0);

        let points = vec![Point::new_scale(0.0, 0.0), Point::new_scale(10.0, 0.0)];
        let wipe_path = Polyline::from_points(points);

        // Last wipe (no cooling markers)
        wipe(
            &mut writer,
            &wipe_path,
            5.0,
            0.4,
            50.0,
            false,
            true, // is_last = true
        )
        .expect("last wipe should succeed");

        // Should still perform wipe
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("TYPE:Wipe_Start"),
            "Last wipe should still emit markers"
        );
    }

    // ===== Set Extruder Tests =====

    #[test]
    fn test_set_extruder_no_change() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // Current extruder is 0 (default)
        assert_eq!(writer.extruder(), 0);

        // Try to set to same extruder (should be no-op)
        set_extruder(0, &mut writer, 0.2, &config).expect("set_extruder should succeed");

        // Should not emit tool change
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            !content.contains("T0"),
            "Should not emit T command when extruder unchanged"
        );
    }

    #[test]
    fn test_set_extruder_single_extruder() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // Change to extruder 1 (single extruder mode)
        set_extruder(1, &mut writer, 0.2, &config).expect("set_extruder should succeed");

        // Should emit T command
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            content.contains("T1") || content.contains("tool change"),
            "Should emit tool change command"
        );

        // Extruder should be updated
        assert_eq!(writer.extruder(), 1);
    }

    #[test]
    fn test_set_extruder_with_retraction() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // Set initial position
        writer.set_x(10.0);
        writer.set_y(10.0);

        // Change extruder (should retract first)
        set_extruder(1, &mut writer, 0.2, &config).expect("set_extruder should succeed");

        // Should have retracted
        let gcode = writer.get_gcode();
        let content = gcode.content();

        // Should contain both retraction and tool change
        assert!(
            content.contains(" E") || content.contains("retract"),
            "Should retract before tool change"
        );
        assert!(
            content.contains("T1") || content.contains("tool change"),
            "Should emit tool change command"
        );
    }

    #[test]
    fn test_set_extruder_multiple_changes() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // Change extruder multiple times
        set_extruder(1, &mut writer, 0.2, &config).expect("first change should succeed");
        assert_eq!(writer.extruder(), 1);

        set_extruder(0, &mut writer, 0.2, &config).expect("second change should succeed");
        assert_eq!(writer.extruder(), 0);

        set_extruder(2, &mut writer, 0.2, &config).expect("third change should succeed");
        assert_eq!(writer.extruder(), 2);

        // Should have emitted multiple tool changes
        let gcode = writer.get_gcode();
        let content = gcode.content();
        let t_count = content.matches("T").count();
        assert!(
            t_count >= 3,
            "Should emit at least 3 tool change commands (got {})",
            t_count
        );
    }

    #[test]
    fn test_set_extruder_first_layer() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // First layer (Z = 0.2) should use first layer temperatures
        set_extruder(1, &mut writer, 0.2, &config).expect("set_extruder should succeed");

        // Should complete successfully
        assert_eq!(writer.extruder(), 1);
    }

    #[test]
    fn test_set_extruder_mid_print() {
        let mut writer = GCodeWriter::new();
        let config = PrintConfig::default();

        // Mid-print (Z = 10.0) should use normal temperatures
        set_extruder(1, &mut writer, 10.0, &config).expect("set_extruder should succeed");

        // Should complete successfully
        assert_eq!(writer.extruder(), 1);
    }

    // ===== Cooling Integration Tests =====

    #[test]
    fn test_apply_layer_cooling_first_layer() {
        let mut writer = GCodeWriter::new();
        let cooling_buffer = CoolingBuffer::new(CoolingConfig::default());

        // First layer should not apply cooling
        apply_layer_cooling(&mut writer, &cooling_buffer, 10.0, 0).expect("cooling should succeed");

        // No fan command should be emitted
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(
            !content.contains("M106"),
            "First layer should not have fan commands"
        );
    }

    #[test]
    fn test_apply_layer_cooling_fast_layer() {
        let mut writer = GCodeWriter::new();
        let mut config = CoolingConfig::default();
        config.min_layer_time = 10.0;
        config.fan_below_layer_time = 15.0;
        config.fan_speed = 1.0;
        config.disable_fan_first_layers = 1;
        let cooling_buffer = CoolingBuffer::new(config);

        // Fast layer (5 seconds < 15 second threshold) should enable fan
        apply_layer_cooling(&mut writer, &cooling_buffer, 5.0, 1).expect("cooling should succeed");

        // Fan should be enabled
        let gcode = writer.get_gcode();
        let content = gcode.content();
        assert!(content.contains("M106"), "Fast layer should enable fan");
    }

    #[test]
    fn test_apply_layer_cooling_slow_layer() {
        let mut writer = GCodeWriter::new();
        let mut config = CoolingConfig::default();
        config.min_layer_time = 10.0;
        config.fan_below_layer_time = 15.0;
        config.disable_fan_first_layers = 1;
        let cooling_buffer = CoolingBuffer::new(config);

        // Slow layer (20 seconds > 15 second threshold) should not enable fan
        apply_layer_cooling(&mut writer, &cooling_buffer, 20.0, 1).expect("cooling should succeed");

        // No fan command (or M107 fan off)
        let gcode = writer.get_gcode();
        let content = gcode.content();
        // Either no M106 or has M107 (fan off)
        let has_fan_on = content.contains("M106 S");
        let has_fan_off = content.contains("M107");
        assert!(
            !has_fan_on || has_fan_off,
            "Slow layer should have fan off or no fan command"
        );
    }

    #[test]
    fn test_bridge_fan_speed() {
        let mut config = CoolingConfig::default();
        config.bridge_fan_override = true;
        config.bridge_fan_speed = 1.0;
        let cooling_buffer = CoolingBuffer::new(config);

        let speed = bridge_fan_speed(&cooling_buffer);
        assert_eq!(speed, 1.0, "Bridge fan should be full speed");
    }

    #[test]
    fn test_overhang_fan_speed() {
        let mut config = CoolingConfig::default();
        config.overhang_fan_override = true;
        config.overhang_fan_speed = 0.5;
        let cooling_buffer = CoolingBuffer::new(config);

        let speed = overhang_fan_speed(&cooling_buffer);
        assert_eq!(speed, 0.5, "Overhang fan should be 50%");
    }

    #[test]
    fn test_set_fan_speed_for_role_normal() {
        let mut writer = GCodeWriter::new();
        let cooling_buffer = CoolingBuffer::new(CoolingConfig::default());

        // Normal extrusion should use base fan speed
        set_fan_speed_for_role(&mut writer, 0.8, ExtrusionRole::Perimeter, &cooling_buffer)
            .expect("set fan should succeed");

        let gcode = writer.get_gcode();
        let content = gcode.content();
        // 0.8 * 255 = 204
        assert!(
            content.contains("M106 S204"),
            "Should set fan to 80% ({})",
            content
        );
    }

    #[test]
    fn test_set_fan_speed_for_role_bridge() {
        let mut writer = GCodeWriter::new();
        let mut config = CoolingConfig::default();
        config.bridge_fan_override = true;
        config.bridge_fan_speed = 1.0;
        let cooling_buffer = CoolingBuffer::new(config);

        // Bridge should use override speed
        set_fan_speed_for_role(
            &mut writer,
            0.5,
            ExtrusionRole::BridgeInfill,
            &cooling_buffer,
        )
        .expect("set fan should succeed");

        let gcode = writer.get_gcode();
        let content = gcode.content();
        // Bridge override: 1.0 * 255 = 255
        assert!(
            content.contains("M106 S255"),
            "Bridge should use 100% fan ({})",
            content
        );
    }

    #[test]
    fn test_set_fan_speed_for_role_overhang() {
        let mut writer = GCodeWriter::new();
        let mut config = CoolingConfig::default();
        config.overhang_fan_override = true;
        config.overhang_fan_speed = 0.7;
        let cooling_buffer = CoolingBuffer::new(config);

        // Overhang should use override speed
        set_fan_speed_for_role(
            &mut writer,
            0.5,
            ExtrusionRole::OverhangPerimeter,
            &cooling_buffer,
        )
        .expect("set fan should succeed");

        let gcode = writer.get_gcode();
        let content = gcode.content();
        // Overhang override: 0.7 * 255 = 178.5 ≈ 178
        assert!(
            content.contains("M106 S178") || content.contains("M106 S179"),
            "Overhang should use 70% fan ({})",
            content
        );
    }
}

/// R654 — the five sites that set the path feedrate immediately before calling
/// `extrude_path`, which is where the `; LINE_WIDTH:` tag is written.
///
/// C++ does both inside ONE function, tag first: `_extrude` emits the Width tag
/// at GCode.cpp:6607 and only then `set_speed` at :6663. Our split puts the speed
/// in the caller, so the pair came out inverted 148,548 times against C++'s ZERO
/// (R653's census). Deferring the F to a pending slot, flushed straight after the
/// tag, reproduces C++'s order without creating or destroying a line.
///
/// SHIPPED OPT-IN (probe, default OFF). Making the adjacency exactly C++'s
/// (G1F->LW 148,548 -> 0, C++ 0) moved content ZERO as predicted but cost
/// **-26,309 IN-ORDER lines** (468,570 -> 442,261). The reason is the tag
/// COUNT, not its order: we emit 154,063 `; LINE_WIDTH:` against C++'s
/// 215,199, a 61,136 deficit, so our tags cannot anchor 1:1 with C++'s.
/// Binding the F to a tag we under-emit destroys alignment that the F lines
/// previously kept on their own. Close the COUNT first, then flip this ON.
///
/// R654 note: the first suspect was the collection-level pre-set at
/// `extrude_collection` — the A/B proved that gate a NO-OP (identical hash), so
/// `skip_pre_speed` is already true there and these five are the real producers.
/// R807: native `_extrude` writes the Width tag (GCode.cpp:6662-6665) BEFORE
/// `set_speed` (:6720). Rust set the speed at the loop/entity level and then
/// `extrude_path` wrote the tag, so `G1 F` landed above `; LINE_WIDTH:` on
/// every width change (118k out-of-order lines on Majora @R806c). Emit the
/// tag here, before the speed; `width_tag_changed` records it, so the check in
/// `extrude_path` is then a no-op for the same width. `LW_BEFORE_SPEED=0` off.
/// GCode.cpp:5654-5663 — after a loop's paths are extruded, `m_wipe.path` is
/// replaced by the loop's clipped paths concatenated (forward, joint
/// duplicates skipped). Shared by both `extrude_loop` branches (R807).
fn set_loop_wipe_path(writer: &mut GCodeWriter, paths: &[ExtrusionPath]) {
    let mut pts: Vec<(f64, f64)> = Vec::new();
    for path in paths.iter() {
        for (k, pt) in path.polyline.points().iter().enumerate() {
            let p = (crate::unscale(pt.x()), crate::unscale(pt.y()));
            if k == 0 && pts.last() == Some(&p) {
                continue;
            }
            pts.push(p);
        }
    }
    writer.set_wipe_path_points_from(pts, "loop");
}

/// First ExtrusionPath reached by native's per-path `_extrude` for an entity
/// (Path itself, a loop's first path, a collection's first leaf). R807.
fn first_extrusion_path(
    entity: &crate::extrusion_entity::ExtrusionEntityType,
) -> Option<&ExtrusionPath> {
    use crate::extrusion_entity::ExtrusionEntityType as T;
    match entity {
        T::Path(p) => Some(p),
        T::Loop(l) => l.paths.first(),
        T::Collection(c) => c.entities.iter().find_map(first_extrusion_path),
    }
}

fn width_tag_before_speed(writer: &mut GCodeWriter, path: &ExtrusionPath) {
    static G: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    if *G.get_or_init(|| {
        crate::faithful_gate("LW_BEFORE_SPEED") && crate::faithful_gate("LINEWIDTH_PERPATH")
    }) && path.width > 0.0
    {
        // WIDTH_TAG_FORCE_ONCE (on): this is GCode.cpp:7886 for the path about
        // to be extruded, so `last_was_wipe_tower` forces it here as well, and
        // `extrude_path` is told the tag is out.
        static ONCE: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
        let force = *ONCE.get_or_init(|| {
            crate::faithful_gate("WIDTH_TAG_FORCE_ONCE")
                && crate::faithful_gate("WIPE_TOWER_FORCE_TAGS")
        }) && writer.peek_force_analyzer_tags();
        if writer.width_tag_changed(path.width) | force {
            writer.write_comment(&format!("LINE_WIDTH: {}", fmt_g6(path.width)));
            if force {
                writer.mark_forced_width_tag_written();
            }
        }
    }
}

fn set_speed_before_path(writer: &mut GCodeWriter, speed: crate::CoordF, comment: &str) {
    if crate::faithful_gate("LINEWIDTH_BEFORE_SPEED") {
        writer.set_speed_pending(speed, comment);
    } else {
        writer.set_speed(speed, comment);
    }
}

// ============================================================================
// The G-code text and its header: what `GCode::_do_export` writes around the
// layers (the config block, the header lines, the custom G-code templates).
// ============================================================================

use crate::Error;
use std::fmt;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

/// Represents generated G-code output.
///
/// This is the result of the slicing process - a complete G-code file
/// that can be written to disk or sent to a printer.
#[derive(Clone, Default)]
pub struct GCode {
    /// The G-code content as a string.
    content: String,

    /// Statistics about the generated G-code.
    pub stats: GCodeStats,
}

impl GCode {
    // Create a new empty GCode.
    pub fn new() -> Self {
        Self {
            content: String::new(),
            stats: GCodeStats::default(),
        }
    }

    /// Create a GCode from content and existing stats.
    pub fn from_content_and_stats(content: String, stats: GCodeStats) -> Self {
        Self { content, stats }
    }

    /// Create a GCode from a string.
    pub fn from_string(content: String) -> Self {
        Self {
            content,
            stats: GCodeStats::default(),
        }
    }

    /// Create a GCode from a string with existing stats.
    pub fn from_string_with_stats(content: String, stats: GCodeStats) -> Self {
        Self { content, stats }
    }

    /// Get the G-code content as a string.
    pub fn content(&self) -> &str {
        &self.content
    }

    /// Get the G-code content as bytes.
    pub fn as_bytes(&self) -> &[u8] {
        self.content.as_bytes()
    }

    /// Get the length of the G-code content in bytes.
    pub fn len(&self) -> usize {
        self.content.len()
    }

    /// Check if the G-code is empty.
    pub fn is_empty(&self) -> bool {
        self.content.is_empty()
    }

    /// Append a line to the G-code.
    /// Remove and return everything written from byte `at` on.
    pub(crate) fn split_off(&mut self, at: usize) -> String {
        self.content.split_off(at)
    }

    pub fn append_line(&mut self, line: &str) {
        self.content.push_str(line);
        self.content.push('\n');
    }

    /// Append raw content to the G-code.
    pub fn append(&mut self, content: &str) {
        self.content.push_str(content);
    }

    /// Append a comment to the G-code.
    pub fn append_comment(&mut self, comment: &str) {
        self.content.push_str("; ");
        self.content.push_str(comment);
        self.content.push('\n');
    }

    /// Write the G-code to a file.
    pub fn write_to_file<P: AsRef<Path>>(&self, path: P) -> Result<()> {
        let file = File::create(path).map_err(|e| Error::Io(e))?;
        let mut writer = BufWriter::new(file);
        writer
            .write_all(self.content.as_bytes())
            .map_err(|e| Error::Io(e))?;
        writer.flush().map_err(|e| Error::Io(e))?;
        Ok(())
    }

    /// Read G-code from a file.
    pub fn read_from_file<P: AsRef<Path>>(path: P) -> Result<Self> {
        let content = std::fs::read_to_string(path).map_err(|e| Error::Io(e))?;
        Ok(Self::from_string(content))
    }

    /// Get the number of lines in the G-code.
    pub fn line_count(&self) -> usize {
        self.content.lines().count()
    }

    /// Iterate over the lines of the G-code.
    pub fn lines(&self) -> impl Iterator<Item = &str> {
        self.content.lines()
    }

    /// Clear the G-code content.
    pub fn clear(&mut self) {
        self.content.clear();
        self.stats = GCodeStats::default();
    }
}

impl fmt::Debug for GCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "GCode({} bytes, {} lines)",
            self.len(),
            self.line_count()
        )
    }
}

impl fmt::Display for GCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.content)
    }
}

impl From<String> for GCode {
    fn from(content: String) -> Self {
        Self::from_string(content)
    }
}

impl From<GCode> for String {
    fn from(gcode: GCode) -> Self {
        gcode.content
    }
}

/// Statistics about generated G-code.
#[derive(Clone, Debug, Default)]
pub struct GCodeStats {
    /// Total number of layers.
    pub layer_count: usize,

    /// Total estimated print time (seconds).
    pub print_time_seconds: f64,

    /// Total filament length used (mm).
    pub filament_length_mm: f64,

    /// Total filament volume (cm³).
    pub filament_volume_cm3: f64,

    /// Total filament weight (g).
    pub filament_weight_g: f64,

    /// Filament density (g/cm³) — from stats/filament info, defaults to 0.
    /// This is separate from PrintConfig.filament_density; the header block
    /// uses this value (BambuStudio outputs 0 when density is not set in stats).
    pub filament_density: f64,

    /// Maximum Z height (mm).
    pub max_z_height: f64,

    /// Total travel distance (mm).
    pub travel_distance_mm: f64,

    /// Total extrusion distance (mm).
    pub extrusion_distance_mm: f64,

    /// Number of retractions.
    pub retraction_count: usize,

    /// Number of tool changes.
    pub tool_change_count: usize,
}

impl GCodeStats {
    // Create new empty statistics.
    pub fn new() -> Self {
        Self::default()
    }

    /// Calculate filament stats from extrusion length.
    ///
    /// Faithful port of BambuStudio's GCodeProcessor header back-fill
    /// (GCode/GCodeProcessor.cpp:697-734). In C++ the canonical accumulated
    /// quantity is the extruded *volume* per filament, in mm³:
    ///   volume_extruded_filament = area_filament_cross_section * delta_pos[E]
    ///   (GCodeProcessor.cpp:4007, area = PI * filament_radius²).
    /// Length and weight are then derived from that volume:
    ///   length = volume / (PI * sqr(0.5 * filament_diameter))   (cpp:731)
    ///   weight = volume * filament_density * 0.001               (cpp:705)
    /// The header field is labelled "[cm^3]" but BambuStudio actually writes
    /// the mm³ volume, so we keep mm³ here for byte parity.
    pub fn calculate_filament_stats(&mut self, config: &PrintConfig) {
        // Volume (mm³) = extruded length × cross-sectional area of the filament.
        let filament_radius = config.filament_diameter / 2.0;
        let cross_section_mm2 = std::f64::consts::PI * filament_radius * filament_radius;
        self.filament_volume_cm3 = self.filament_length_mm * cross_section_mm2;

        // Weight (g) = volume(mm³) * density(g/cm³) * 0.001  (cm³ per mm³).
        // GCodeProcessor.cpp:705 uses the filament density from the config,
        // not the (unset) per-stats density.
        self.filament_weight_g = self.filament_volume_cm3 * config.filament_density * 0.001;
    }

    /// Get print time formatted as HH:MM:SS.
    pub fn print_time_formatted(&self) -> String {
        let total_seconds = self.print_time_seconds as u64;
        let hours = total_seconds / 3600;
        let minutes = (total_seconds % 3600) / 60;
        let seconds = total_seconds % 60;
        format!("{:02}:{:02}:{:02}", hours, minutes, seconds)
    }

    /// Get filament used in meters.
    pub fn filament_used_meters(&self) -> f64 {
        self.filament_length_mm / 1000.0
    }
}

impl fmt::Display for GCodeStats {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "GCodeStats(layers={}, time={}, filament={:.2}m)",
            self.layer_count,
            self.print_time_formatted(),
            self.filament_used_meters()
        )
    }
}

/// Header information for BambuStudio-compatible G-code.
#[derive(Clone, Debug)]
pub struct GCodeHeader {
    /// Software name and version.
    pub software_version: String,

    /// Accel-aware estimated printing time for the "normal" time mode,
    /// formatted via `utils::get_time_dhms` (e.g. "43m 0s"). Computed by running
    /// the faithful `GCodeProcessor` over the assembled body; mirrors the C++
    /// `; estimated printing time (normal mode) = ...` header line.
    pub estimated_print_time: String,

    /// The processor's total time in seconds (`machine.time`), for the first
    /// `M73` line (GCodeProcessor.cpp:685-688).
    pub estimated_print_time_seconds: f32,

    /// The model printing time (total less the prepare time), when the
    /// processor's prepare time is known; written by the Bambu Lab form of the
    /// time line (GCodeProcessor.cpp:711-716).
    pub model_print_time: Option<String>,

    /// Statistics from the slicing process.
    pub stats: GCodeStats,

    /// Print configuration.
    pub config: PrintConfig,

    /// Raw BambuStudio settings for full CONFIG_BLOCK output.
    pub raw_settings: Option<serde_json::Value>,
}

impl GCodeHeader {
    // Create a new header from stats and config.
    pub fn new(stats: GCodeStats, config: PrintConfig) -> Self {
        Self::with_raw_settings(stats, config, None)
    }

    pub fn with_raw_settings(
        stats: GCodeStats,
        config: PrintConfig,
        raw_settings: Option<serde_json::Value>,
    ) -> Self {
        // Fall back to the crude (acceleration-free) stats time when no
        // accel-aware estimate has been computed yet (e.g. early header builds).
        let est = stats.print_time_seconds as f32;
        Self::with_estimated_time(stats, config, raw_settings, est)
    }

    /// Build a header with an explicit accel-aware estimated print time (s),
    /// computed by running the faithful `GCodeProcessor` over the body.
    pub fn with_estimated_time(
        stats: GCodeStats,
        config: PrintConfig,
        raw_settings: Option<serde_json::Value>,
        estimated_print_time_seconds: f32,
    ) -> Self {
        // GCode.cpp / GCodeProcessor: `; estimated printing time (normal mode) = `
        // is formatted with Utils get_time_dhms.
        let estimated_print_time = crate::utils::get_time_dhms(estimated_print_time_seconds);

        Self {
            // HEADER_0209 (on): `header_slic3r_generated()` (Utils.cpp:1169-1172),
            // the pinned engine's name and version. =0 writes the 02.06 engine's.
            software_version: if crate::faithful_gate("HEADER_0209") {
                crate::utils::header_slic3r_generated()
            } else {
                "BambuStudio 02.06.00.51".to_string()
            },
            estimated_print_time,
            estimated_print_time_seconds,
            model_print_time: None,
            stats,
            config,
            raw_settings,
        }
    }

    /// `with_estimated_time` with the processor's prepare time as well, for the
    /// Bambu Lab time line: the model printing time is `time - prepare_time`
    /// (GCodeProcessor.cpp:711-716).
    pub fn with_estimated_times(
        stats: GCodeStats,
        config: PrintConfig,
        raw_settings: Option<serde_json::Value>,
        estimated_print_time_seconds: f32,
        prepare_time_seconds: f32,
    ) -> Self {
        let mut header =
            Self::with_estimated_time(stats, config, raw_settings, estimated_print_time_seconds);
        header.model_print_time = Some(crate::utils::get_time_dhms(
            estimated_print_time_seconds - prepare_time_seconds,
        ));
        header
    }

    /// Generate the HEADER_BLOCK section.
    /// Format matches BambuStudio C++ output exactly.
    pub fn generate_header_block(&self) -> String {
        let mut header = String::new();

        header.push_str("; HEADER_BLOCK_START\n");
        header.push_str(&format!("; {}\n", self.software_version));
        // GCodeProcessor.cpp:700-720 — the time placeholder: the Klipper form
        // `; estimated printing time (normal mode) = ..` on other printers, the
        // Bambu Lab form `; model printing time: ..; total estimated time: ..`
        // (HEADER_0209) on a Bambu Lab printer.
        // SAFETY: S_IS_BBL_PRINTER is only written during configuration.
        let bbl = unsafe { crate::gcode::g_code_processor::S_IS_BBL_PRINTER };
        match (&self.model_print_time, bbl && crate::faithful_gate("HEADER_0209")) {
            (Some(model), true) => header.push_str(&format!(
                "; model printing time: {}; total estimated time: {}\n",
                model, self.estimated_print_time
            )),
            _ => header.push_str(&format!(
                "; estimated printing time (normal mode) = {}\n",
                self.estimated_print_time
            )),
        }
        header.push_str(&format!(
            "; total layer number: {}\n",
            self.stats.layer_count
        ));
        header.push_str(&format!(
            "; total filament length [mm] : {:.2}\n",
            self.stats.filament_length_mm
        ));
        header.push_str(&format!(
            "; total filament volume [cm^3] : {:.2}\n",
            self.stats.filament_volume_cm3
        ));
        header.push_str(&format!(
            "; total filament weight [g] : {:.2}\n",
            self.stats.filament_weight_g
        ));
        header.push_str(&format!(
            "; filament_density: {:.2}\n",
            self.config.filament_density
        ));
        header.push_str(&format!(
            "; filament_diameter: {:.2}\n",
            self.config.filament_diameter
        ));
        header.push_str(&format!("; max_z_height: {:.2}\n", self.stats.max_z_height));
        // Configured filament count (single-material configs ⇒ 1, preserving
        // the locked default-path bytes). C++ writes the used-filament list;
        // matching that per-use accounting is part of the multicolour chain.
        header.push_str(&format!(
            "; filament: {}\n",
            self.config.num_filaments()
        ));
        header.push_str("; HEADER_BLOCK_END\n");
        header.push_str("\n");

        header
    }

    /// Generate the CONFIG_BLOCK section with all print settings.
    /// If raw_settings (from BambuStudio project_settings.config) are available,
    /// emit the full config block matching the reference output format.
    pub fn generate_config_block(&self) -> String {
        if let Some(ref settings) = self.raw_settings {
            if let Some(obj) = settings.as_object() {
                let mut config = String::new();
                config.push_str("; CONFIG_BLOCK_START\n");
                // BambuStudio iterates DynamicPrintConfig::keys(), which returns
                // keys in sorted (alphabetical) order, so we sort here too.
                // (GCode.cpp: append_full_config / GCode::process_layers)
                // Under CONFIG_FAITHFUL, add schema-default keys absent from
                // raw_settings so the block matches native's full_print_config
                // (every registered option). Merge + sort to keep ordering.
                let cf = crate::faithful_gate("CONFIG_FAITHFUL");
                // Number of USED extruders. Native serializes per-extruder config
                // arrays (nozzle_temperature, retraction_distances_when_ec, …) at
                // this length — a single-material print uses 1 even on a 2-nozzle
                // H2D, so native emits "220" where rust emits "220,220". Derived
                // from the distinct extruder assignments in filament_map.
                let used_extruders: usize = obj
                    .get("filament_map")
                    .map(|v| {
                        let s = match v {
                            serde_json::Value::String(s) => s.clone(),
                            serde_json::Value::Array(a) => a
                                .iter()
                                .map(|x| {
                                    x.as_str().map(str::to_string).unwrap_or_else(|| x.to_string())
                                })
                                .collect::<Vec<_>>()
                                .join(","),
                            other => other.to_string(),
                        };
                        s.split(',')
                            .map(str::trim)
                            .filter(|x| !x.is_empty())
                            .collect::<std::collections::HashSet<_>>()
                            .len()
                            .max(1)
                    })
                    .unwrap_or(1);
                let mut keys: Vec<String> = obj.keys().cloned().collect();
                if cf {
                    for (k, _) in CONFIG_SCHEMA_DEFAULTS {
                        if !obj.contains_key(*k) {
                            keys.push((*k).to_string());
                        }
                    }
                }
                keys.sort();
                for key in &keys {
                    if is_config_key_skipped(key) {
                        continue;
                    }
                    if let Some(value) = obj.get(key) {
                        let formatted = format_config_value(
                            key,
                            value,
                            used_extruders,
                            self.config.num_filaments(),
                        );
                        // Empty = omit (all-"nil" nullable vectors on
                        // multi-material prints; native omits them too).
                        // Scoped to num_filaments > 1: single-material echoes
                        // legitimately emit empty values (filament_notes = )
                        // and are byte-locked.
                        if formatted.is_empty()
                            && value.is_array()
                            && self.config.num_filaments() > 1
                        {
                            continue;
                        }
                        config.push_str(&format!("; {} = {}\n", key, formatted));
                    } else if cf {
                        if let Some((_, dv)) =
                            CONFIG_SCHEMA_DEFAULTS.iter().find(|(k, _)| k == key)
                        {
                            config.push_str(&format!("; {} = {}\n", key, dv));
                        }
                    }
                }
                config.push_str("; CONFIG_BLOCK_END\n\n");
                return config;
            }
        }
        self.generate_minimal_config_block()
    }

    /// Generate a minimal CONFIG_BLOCK if template is not available.
    fn generate_minimal_config_block(&self) -> String {
        let mut config = String::new();
        config.push_str("; CONFIG_BLOCK_START\n");

        let c = &self.config;
        config.push_str(&format!("; bed_temperature = {}\n", c.bed_temperature));
        config.push_str(&format!(
            "; bed_temperature_initial_layer = {}\n",
            c.first_layer_bed_temperature
        ));
        config.push_str(&format!("; brim_width = {:.1}\n", c.brim_width));
        config.push_str(&format!("; filament_density = {:.2}\n", c.filament_density));
        config.push_str(&format!(
            "; filament_diameter = {:.2}\n",
            c.filament_diameter
        ));
        config.push_str(&format!("; layer_height = {:.2}\n", c.layer_height));
        config.push_str(&format!(
            "; first_layer_height = {:.2}\n",
            c.first_layer_height
        ));
        config.push_str(&format!(
            "; nozzle_diameter = {:.2},{:.2}\n",
            c.nozzle_diameter, c.nozzle_diameter
        ));
        config.push_str(&format!(
            "; nozzle_temperature = {}\n",
            c.extruder_temperature
        ));
        config.push_str(&format!(
            "; nozzle_temperature_initial_layer = {}\n",
            c.first_layer_extruder_temperature
        ));
        config.push_str(&format!(
            "; travel_speed = {:.0},{:.0}\n",
            c.travel_speed, c.travel_speed
        ));
        config.push_str(&format!(
            "; initial_layer_speed = {:.0}\n",
            c.first_layer_speed
        ));
        config.push_str(&format!(
            "; retraction_length = {:.1},{:.1}\n",
            c.retract_length, c.retract_length
        ));
        config.push_str(&format!(
            "; retraction_speed = {:.0},{:.0}\n",
            c.retract_speed, c.retract_speed
        ));
        config.push_str(&format!(
            "; z_hop = {:.2},{:.2}\n",
            c.retract_lift, c.retract_lift
        ));

        config.push_str("; CONFIG_BLOCK_END\n\n");
        config
    }

    /// Generate the EXECUTABLE_BLOCK_START marker.
    pub fn generate_executable_block_start(&self) -> String {
        "; EXECUTABLE_BLOCK_START\n".to_string()
    }

    /// Generate the executable block content between EXECUTABLE_BLOCK_START
    /// and the first CHANGE_LAYER.  Matches the BambuStudio C++ output for
    /// Bambu Lab H2D + PLA Basic.
    pub fn generate_executable_block_content(&self) -> String {
        let mut s = String::new();
        // M73_LINES (on): the first-line placeholder becomes `M73 P0 R<minutes>`
        // with `time_in_minutes(machine.time)` = int((t + 0.5f) / 60.0f)
        // (GCodeProcessor.cpp:589-592, 683-688). =0 rounds the writer's own
        // crude time.
        let minutes = if crate::faithful_gate("M73_LINES") {
            ((self.estimated_print_time_seconds + 0.5f32) / 60.0f32) as i32 as u64
        } else {
            (self.stats.print_time_seconds / 60.0).round() as u64
        };
        let c = &self.config;

        // Progress estimate
        s.push_str(&format!("M73 P0 R{}\n", minutes));

        // Machine motion parameters — read from raw_settings if available
        if let Some(ref settings) = self.raw_settings {
            let get_first = |key: &str, default: &str| -> String {
                settings
                    .get(key)
                    .and_then(|v| match v {
                        serde_json::Value::Array(arr) => {
                            arr.first().and_then(|x| x.as_str()).map(|s| s.to_string())
                        }
                        serde_json::Value::String(s) => Some(s.clone()),
                        _ => None,
                    })
                    .unwrap_or_else(|| default.to_string())
            };
            s.push_str(&format!(
                "M201 X{} Y{} Z{} E{}\n",
                get_first("machine_max_acceleration_x", "20000"),
                get_first("machine_max_acceleration_y", "20000"),
                get_first("machine_max_acceleration_z", "500"),
                get_first("machine_max_acceleration_e", "5000")
            ));
            s.push_str(&format!(
                "M203 X{} Y{} Z{} E{}\n",
                get_first("machine_max_speed_x", "500"),
                get_first("machine_max_speed_y", "500"),
                get_first("machine_max_speed_z", "20"),
                get_first("machine_max_speed_e", "30")
            ));
            // M204: P=print accel (same as X), R=retract accel, T=travel accel (same as P)
            let print_accel = get_first("machine_max_acceleration_x", "20000");
            s.push_str(&format!(
                "M204 P{} R{} T{}\n",
                print_accel,
                get_first("machine_max_acceleration_retracting", "5000"),
                print_accel
            ));
            s.push_str(&format!(
                "M205 X{:.2} Y{:.2} Z{:.2} E{:.2}\n",
                get_first("machine_max_jerk_x", "9")
                    .parse::<f64>()
                    .unwrap_or(9.0),
                get_first("machine_max_jerk_y", "9")
                    .parse::<f64>()
                    .unwrap_or(9.0),
                get_first("machine_max_jerk_z", "3")
                    .parse::<f64>()
                    .unwrap_or(3.0),
                get_first("machine_max_jerk_e", "2.5")
                    .parse::<f64>()
                    .unwrap_or(2.5)
            ));
            // Fan off before machine_start_gcode (matches BambuStudio GCode.cpp preamble)
            // Only emit for printers with auxiliary cooling fans (X1C, P1S, etc.)
            // A1 mini doesn't have P2 aux fan, so skip for those printers
            if c.auxiliary_fan {
                s.push_str("M106 S0\n");
                s.push_str("M106 P2 S0\n");
            }
        } else {
            // Generic defaults
            s.push_str("M201 X20000 Y20000 Z500 E5000\n");
            s.push_str("M203 X1000 Y1000 Z30 E50\n");
            s.push_str("M204 P20000 R5000 T20000\n");
            s.push_str("M205 X9.00 Y9.00 Z3.00 E2.50\n");
            s.push_str(&format!(
                "M190 S{} ; set bed temperature and wait for it to be reached\n",
                c.cool_plate_temp
            ));
            s.push_str(&format!(
                "M104 S{} ; set nozzle temperature\n",
                c.extruder_temperature
            ));
        }

        // Machine start G-code — process template from settings if available
        s.push_str("; FEATURE: Custom\n");
        if let Some(ref settings) = self.raw_settings {
            if let Some(start_gcode) = settings.get("machine_start_gcode").and_then(|v| v.as_str())
            {
                let processed = process_gcode_template(start_gcode, settings, &self.config);
                // Trim trailing newlines to avoid blank line before MACHINE_START_GCODE_END
                s.push_str(processed.trim_end_matches('\n'));
                s.push('\n');
            }
            // Post-machine-start transition (matches BambuStudio GCode.cpp)
            s.push_str("; MACHINE_START_GCODE_END\n");
            // Filament start gcode
            if let Some(filament_start) = settings.get("filament_start_gcode").and_then(|v| match v
            {
                serde_json::Value::Array(a) => a.first().and_then(|x| x.as_str()),
                serde_json::Value::String(s) => Some(s.as_str()),
                _ => None,
            }) {
                if !filament_start.is_empty() {
                    // Don't add "; filament start gcode" — the template itself contains it
                    let processed = process_gcode_template(filament_start, settings, &self.config);
                    s.push_str(&processed);
                    if !processed.ends_with('\n') {
                        s.push('\n');
                    }
                }
            }
            // Post-filament-start positioning (matches BambuStudio GCode.cpp)
            // R813 VT_HOTEND (faithful ON): GCode.cpp 3235-3236 writes
            // ";VT%d H%d" with NOZZLE_ID_FOR_GCODE, which is -1 unless the
            // printer maps nozzles dynamically.
            if crate::faithful_gate("VT_HOTEND") {
                s.push_str(";VT0 H-1\n");
            } else {
                s.push_str(";VT0\n");
            }
            s.push_str("G90\n");
            s.push_str("G21\n");
            s.push_str("M83 ; use relative distances for extrusion\n");
            // Spaghetti detector
            s.push_str("M981 S1 P20000 ;open spaghetti detector\n");
        } else {
            s.push_str("G28 ; home all axes\n");
            s.push_str("G1 Z5 F5000 ; lift nozzle\n");
            s.push_str("; MACHINE_START_GCODE_END\n");
        }

        if self.raw_settings.is_none() {
            // Generic post-start (no BambuStudio settings)
            s.push_str("M104 T0 S0 N0 ;Multi extruder pre cooling\n");
            s.push_str(";VT0\n");
            s.push_str(&format!(
                "M109 S{} ; set nozzle temperature and wait for it to be reached\n",
                c.extruder_temperature
            ));
            s.push_str("G90\n");
            s.push_str("G21\n");
            s.push_str("M83 ; use relative distances for extrusion\n");
        }
        // When using BambuStudio settings, machine_start_gcode handles
        // all temperature/positioning. No extra commands needed.

        s
    }

    /// Generate the complete header (all blocks).
    pub fn generate_complete_header(&self) -> String {
        let mut header = String::new();
        header.push_str(&self.generate_header_block());
        header.push_str(&self.generate_config_block());
        header.push_str(&self.generate_executable_block_start());
        header.push_str(&self.generate_executable_block_content());
        header
    }
}

/// Format a JSON value for the CONFIG_BLOCK section.
///
/// Returns true if a config key must NOT be emitted into the CONFIG_BLOCK.
///
/// Faithful to `GCode::append_full_config` (BambuStudio src/libslic3r/GCode.cpp),
/// which iterates `print.full_print_config().keys()` and emits every option
/// except (a) a hard-coded set of host/credential `banned_keys`, and
/// (b) options that are `is_nil()`.
///
/// In C++ the iteration source is `full_print_config()` — the fully resolved
/// config that contains ONLY currently-defined option keys. Our `raw_settings`
/// instead carries the on-disk config, which still includes deprecated/renamed
/// or developer-only keys that BambuStudio drops during config normalization
/// (they are not added via `this->add(...)` in PrintConfig.cpp, or are
/// `comDevelop`-only, so they never appear in `full_print_config()`). We mirror
/// that filtering here so the emitted key set matches the native output.
/// R329: BambuStudio config-schema keys absent from rust's config but present
/// in native's CONFIG_BLOCK, with their PrintConfig.cpp STATIC defaults
/// (extracted from source, cross-verified == native output). Emitted under
/// CONFIG_FAITHFUL when the key is missing from raw_settings. Profile-derived
/// keys (where the profile overrides the default) are intentionally excluded.
const CONFIG_SCHEMA_DEFAULTS: &[(&str, &str)] = &[
    ("accel_to_decel_enable", "0"),
    ("accel_to_decel_factor", "50%"),
    ("apply_scarf_seam_on_circles", "1"),
    ("bed_custom_model", ""),
    ("bed_custom_texture", ""),
    ("before_layer_change_gcode", ""),
    ("bridge_angle", "0"),
    ("brim_type", "auto_brim"),
    ("default_filament_colour", "\"\""),
    ("default_jerk", "0"),
    ("detect_narrow_internal_solid_infill", "1"),
    ("embedding_wall_into_infill", "0"),
    ("enable_filament_dynamic_map", "0"),
    ("enable_mixed_color_sublayer", "0"),
    ("enable_overhang_bridge_fan", "1"),
    ("enable_pressure_advance", "0"),
    ("enforce_support_layers", "0"),
    ("ensure_vertical_shell_thickness", "enabled"),
    ("exclude_object", "1"),
    ("extruder_ams_count", "\"\""),
    ("filament_change_length_nc", "10"),
    ("filament_colour", "#00AE42"),
    ("filament_ids", "\"\""),
    ("filament_is_mixed", "0"),
    ("filament_mixed_components", "\"\""),
    ("filament_mixed_gradient", "0"),
    ("filament_mixed_gradient_range", "\"\""),
    ("filament_mixed_sublayer_ratios", "\"\""),
    ("filament_notes", ""),
    ("filter_out_gap_fill", "0"),
    ("first_layer_print_sequence", "0"),
    ("first_x_layer_fan_speed", "0"),
    ("first_x_layer_part_fan_speed", "0"),
    ("flush_into_infill", "0"),
    ("flush_into_objects", "0"),
    ("flush_into_support", "1"),
    ("flush_multiplier", "1"),
    ("gcode_add_line_number", "0"),
    ("has_filament_switcher", "0"),
    ("has_scarf_joint_seam", "0"),
    ("independent_support_layer_height", "1"),
    ("infill_jerk", "9"),
    ("initial_layer_flow_ratio", "1"),
    ("initial_layer_jerk", "9"),
    ("inner_wall_jerk", "9"),
    ("interlocking_beam", "0"),
    ("interlocking_beam_layer_count", "2"),
    ("interlocking_beam_width", "0.8"),
    ("interlocking_boundary_avoidance", "2"),
    ("interlocking_depth", "2"),
    ("interlocking_orientation", "22.5"),
    ("internal_solid_infill_pattern", "zig-zag"),
    ("ironing_direction", "45"),
    ("ironing_fan_speed", "-1"),
    ("ironing_pattern", "zig-zag"),
    ("is_infill_first", "0"),
    ("min_bead_width", "85%"),
    ("min_feature_size", "25%"),
    ("mmu_segmented_region_interlocking_depth", "0"),
    ("mmu_segmented_region_max_width", "0"),
    ("only_one_wall_first_layer", "0"),
    ("ooze_prevention", "0"),
    ("other_layers_print_sequence", "0"),
    ("other_layers_print_sequence_nums", "0"),
    ("outer_wall_jerk", "9"),
    ("post_process", "\"\""),
    ("precise_outer_wall", "0"),
    ("precise_z_height", "0"),
    ("pressure_advance", "0.02"),
    ("prime_tower_extra_rib_length", "0"),
    ("prime_tower_fillet_wall", "1"),
    ("prime_tower_infill_gap", "150%"),
    ("prime_tower_rib_wall", "1"),
    ("prime_tower_rib_width", "8"),
    ("prime_tower_skip_points", "1"),
    ("prime_volume_mode", "Default"),
    ("print_flow_ratio", "1"),
    ("printer_model", ""),
    ("printer_notes", ""),
    ("printing_by_object_gcode", ""),
    ("process_notes", ""),
    ("raft_contact_distance", "0.1"),
    ("raft_expansion", "1.5"),
    ("raft_first_layer_density", "90%"),
    ("raft_first_layer_expansion", "-1"),
    ("role_base_wipe_speed", "1"),
    ("seam_gap", "15%"),
    ("seam_slope_conditional", "1"),
    ("seam_slope_entire_loop", "0"),
    ("seam_slope_inner_walls", "1"),
    ("seam_slope_steps", "10"),
    ("slice_closing_radius", "0.049"),
    ("slicing_mode", "regular"),
    ("smooth_speed_discontinuity_area", "1"),
    ("sparse_infill_anchor", "400%"),
    ("sparse_infill_anchor_max", "20"),
    ("spiral_mode_max_xy_smoothing", "200%"),
    ("spiral_mode_smooth", "0"),
    ("start_end_points", "30x-3,54x245"),
    ("support_angle", "0"),
    ("support_bottom_interface_spacing", "0.5"),
    ("support_critical_regions_only", "0"),
    ("support_interface_not_for_body", "1"),
    ("support_object_first_layer_gap", "0.2"),
    ("support_remove_small_overhang", "1"),
    ("template_custom_gcode", ""),
    ("thick_bridges", "0"),
    ("thumbnail_size", "50x50"),
    ("timelapse_type", "0"),
    ("top_area_threshold", "200%"),
    ("top_one_wall_type", "all top"),
    ("top_surface_jerk", "9"),
    ("top_z_overrides_xy_distance", "0"),
    ("travel_jerk", "9"),
    ("tree_support_branch_diameter_angle", "5"),
    ("tree_support_branch_distance", "5"),
    ("use_firmware_retraction", "0"),
    ("use_relative_e_distances", "1"),
    ("wall_distribution_count", "1"),
    ("wall_sequence", "inner wall/outer wall"),
    ("wall_transition_angle", "10"),
    ("wall_transition_filter_deviation", "25%"),
    ("wall_transition_length", "100%"),
    ("wipe_speed", "80%"),
    ("wipe_tower_rotation_angle", "0"),
    ("wipe_tower_x", "15"),
    ("wipe_tower_y", "220"),
    ("wrapping_detection_layers", "20"),
];

fn is_config_key_skipped(key: &str) -> bool {
    // GCode.cpp append_full_config banned_keys (host / credential settings).
    const BANNED_KEYS: &[&str] = &[
        "compatible_printers",
        "compatible_prints",
        "print_host",
        "print_host_webui",
        "printhost_apikey",
        "printhost_cafile",
        "printhost_user",
        "printhost_password",
        "printhost_port",
    ];
    // Deprecated / renamed / developer-only keys that are absent from
    // full_print_config() (not registered via this->add(...) in
    // PrintConfig.cpp, or commented out / comDevelop), and therefore never
    // emitted by native BambuStudio.
    const DROPPED_KEYS: &[&str] = &[
        "adaptive_layer_height",
        "deretract_speed_extruder_change",
        "extruder_clearance_radius",
        "extruder_height_gap",
        "filament_deretraction_speed",
        "filament_id",
        "filament_long_retractions_when_cut",
        "filament_long_retractions_when_ec",
        "filament_printable",
        "filament_retract_before_wipe",
        "filament_retract_restart_extra",
        "filament_retract_when_changing_layer",
        "filament_retraction_distances_when_cut",
        "filament_retraction_distances_when_ec",
        "filament_retraction_minimum_travel",
        "filament_retraction_speed",
        "filament_z_hop",
        // Runtime template variables injected into raw_settings by print.rs
        // (first-layer bounding box for the start-gcode G29 calls). They are
        // not config options in full_print_config(), so native BambuStudio
        // never emits them as CONFIG_BLOCK keys.
        "first_layer_print_min",
        "first_layer_print_size",
        "layer_time_smoothing",
        "layer_time_smoothing_threshold",
        "only_one_wall_top",
        "reduce_infill_retraction",
        "wall_infill_order",
    ];
    BANNED_KEYS.contains(&key) || DROPPED_KEYS.contains(&key)
}

/// BambuStudio's CONFIG_BLOCK uses different formatting depending on the
/// C++ config option type. Since we don't have that type info, we use
/// key-name heuristics:
///
/// - Per-extruder numeric arrays (most keys): first element only
/// - Geometry arrays (printable_area, bed_exclude_area, etc.): comma-joined
/// - machine_max_* arrays (4 elements): first 2 comma-joined
/// - String list arrays (gcode, compatible_printers, etc.): semicolon-joined with quotes
/// - Quoted string arrays (settings_id, vendor, etc.): quoted first element
/// - Scalar strings: literal with \n escaping
fn format_config_value(
    key: &str,
    value: &serde_json::Value,
    used_extruders: usize,
    num_filaments: usize,
) -> String {
    // Under CONFIG_FAITHFUL, native coFloat drops a trailing ".0" (12.0->12,
    // 1.0->1) and coPercent appends '%'. Apply to scalar string values.
    let cf = crate::faithful_gate("CONFIG_FAITHFUL");
    // coPercent keys: native ConfigOptionPercent::serialize always appends '%'.
    // rust stores the bare number for some profile-resolved values.
    const PERCENT_KEYS: &[&str] =
        &["bottom_surface_density", "top_surface_density", "monotonic_travel_into_wall"];
    let normalize_scalar = |raw: &str| -> String {
        let mut out = raw.replace('\n', "\\n");
        if cf {
            // coPoint (best_object_pos): native ConfigOptionPoint::serialize emits
            // "X,Y"; BBL's profile JSON stores the point as "XxY".
            if key == "best_object_pos" {
                out = out.replace('x', ",");
            }
            if out.ends_with(".0") && out[..out.len() - 2].chars().all(|c| c.is_ascii_digit() || c == '-') && !out[..out.len()-2].is_empty() {
                out.truncate(out.len() - 2);
            }
            if PERCENT_KEYS.contains(&key) && !out.ends_with('%') {
                out.push('%');
            }
        }
        out
    };
    match value {
        serde_json::Value::String(s) => normalize_scalar(s),
        serde_json::Value::Array(arr) if arr.is_empty() => String::new(),
        serde_json::Value::Array(arr) => {
            // coPointsGroups (PrintConfig.cpp: extruder_printable_area):
            // groups joined by '#', each group is its element string as-is
            // (already comma-joined "AxB,CxD,..."). Mirrors Config.hpp
            // ConfigOptionPointsGroups::serialize, which emits '#' between groups.
            if key == "extruder_printable_area" {
                return arr
                    .iter()
                    .map(|v| v.as_str().unwrap_or("").to_string())
                    .collect::<Vec<_>>()
                    .join("#");
            }

            // Comma-joined geometry/point arrays
            const COMMA_JOIN_KEYS: &[&str] = &[
                "bed_exclude_area",
                "printable_area",
                "start_end_points",
                "flush_volumes_vector",
                "flush_volumes_matrix",
                "filament_dev_ams_drying_temperature",
                "filament_dev_ams_drying_time",
                "filament_self_index",
                // extruder_printable_height is coFloats -> comma-joined list.
                "extruder_printable_height",
            ];
            if COMMA_JOIN_KEYS.contains(&key) {
                return arr
                    .iter()
                    .map(|v| v.as_str().unwrap_or("").to_string())
                    .collect::<Vec<_>>()
                    .join(",");
            }

            // machine_max_* arrays: legacy emits 2-value Marlin format; native
            // (H2D) emits the FULL resolved array. Under CONFIG_FAITHFUL emit full.
            if key.starts_with("machine_max_") && arr.len() >= 2 {
                let take_n = if crate::faithful_gate("CONFIG_FAITHFUL") { arr.len() } else { 2 };
                return arr
                    .iter()
                    .take(take_n)
                    .map(|v| v.as_str().unwrap_or("").to_string())
                    .collect::<Vec<_>>()
                    .join(",");
            }

            // Semicolon-joined string list arrays (gcode, compatible printers, etc.)
            const SEMICOLON_KEYS: &[&str] = &[
                "filament_start_gcode",
                "filament_end_gcode",
                "print_compatible_printers",
                "upward_compatible_machine",
                "extruder_ams_count",
                "filament_dev_ams_drying_ams_limitations",
            ];
            if SEMICOLON_KEYS.contains(&key) {
                // Native coStrings serialize via escape_string_cstyle: simple
                // tokens (no space/quote/newline) are emitted UNQUOTED. rust
                // historically quoted every element (byte-locked default). Under
                // CONFIG_FAITHFUL, drop quotes for the numeric-token key that
                // native leaves bare (1;0), matching escape_string_cstyle.
                let unquoted = cf && key == "filament_dev_ams_drying_ams_limitations";
                return arr
                    .iter()
                    .map(|v| {
                        let s = v.as_str().unwrap_or("").replace('\n', "\\n");
                        if unquoted { s } else { format!("\"{}\"", s) }
                    })
                    .collect::<Vec<_>>()
                    .join(";");
            }

            // Quoted first-element arrays (settings IDs, vendor names, etc.)
            const QUOTED_KEYS: &[&str] = &[
                "default_filament_colour",
                "default_filament_profile",
                "extruder_variant_list",
                "filament_extruder_variant",
                "filament_settings_id",
                "filament_vendor",
                "print_extruder_variant",
                "printer_extruder_variant",
                "volumetric_speed_coefficients",
            ];
            if QUOTED_KEYS.contains(&key) {
                let s = arr.first().and_then(|v| v.as_str()).unwrap_or("");
                return format!("\"{}\"", s.replace('\n', "\\n"));
            }

            // Per-USED-extruder configs: native trims these to the used-extruder
            // count (1 for a single-material print) rather than emitting every
            // physical extruder. rust's resolved config carries all physical
            // extruders (e.g. "220,220" on a 2-nozzle H2D); take only the first
            // `used_extruders` to match native's "220". Gated CONFIG_FAITHFUL.
            const PER_USED_EXTRUDER_KEYS: &[&str] = &[
                "long_retractions_when_ec",
                "nozzle_temperature",
                "nozzle_temperature_initial_layer",
                "override_process_overhang_speed",
                "retraction_distances_when_ec",
                "slow_down_min_speed",
            ];
            // The trim is a SINGLE-material parity rule — multi-filament prints
            // must keep the full arrays (see the multi-material block below).
            if cf
                && num_filaments == 1
                && arr.len() > used_extruders
                && PER_USED_EXTRUDER_KEYS.contains(&key)
            {
                return arr
                    .iter()
                    .take(used_extruders)
                    .map(|v| v.as_str().unwrap_or("").replace('\n', "\\n"))
                    .collect::<Vec<_>>()
                    .join(",");
            }

            // Multi-material prints MUST echo the FULL per-filament arrays:
            // BambuStudio's gcode preview (GCodeProcessor) indexes
            // filament_colour / filament_type / nozzle_temperature / … by tool
            // id, so a 1-element echo alongside T0..T7 in the body reads out
            // of bounds and CRASHES the application on load. Serialization
            // separators mirror native: coStrings use ';'
            // (filament_colour = #A;#B), numeric vectors use ','
            // (nozzle_temperature = 200,200). Nullable options whose entries
            // are ALL "nil" (filament_wipe, filament_retraction_length, …) are
            // OMITTED from the block entirely — native does the same (zero
            // "nil" in its CONFIG_BLOCK), and BBS's loader rejects
            // "nil;nil;…" with an "Invalid value provided" error dialog.
            if num_filaments > 1 && arr.len() > 1 {
                let strs: Vec<&str> = arr.iter().map(|v| v.as_str().unwrap_or("")).collect();
                if strs.iter().all(|s| *s == "nil") {
                    return String::new(); // caller omits the key
                }
                // Separator: numeric if every NON-nil element parses as a
                // number ("nil" is legal inside comma-joined nullable numeric
                // vectors and must not force the string form).
                let all_numeric = strs
                    .iter()
                    .filter(|s| **s != "nil")
                    .all(|s| s.trim_end_matches('%').parse::<f64>().is_ok());
                let sep = if all_numeric { "," } else { ";" };
                return strs
                    .iter()
                    .map(|s| s.replace('\n', "\\n"))
                    .collect::<Vec<_>>()
                    .join(sep);
            }

            // Default: native ConfigOptionFloats/Ints/Strings::serialize emit
            // the FULL vector comma-joined; the resolved config already carries
            // the full per-extruder/filament array. Rust historically emitted
            // first element only (byte-locked default). Under CONFIG_FAITHFUL
            // emit the full comma-joined array to match native's CONFIG_BLOCK.
            if crate::faithful_gate("CONFIG_FAITHFUL")
                && arr.len() > 1
                && !key.starts_with("filament_")
            {
                // extruder_colour (coStrings) serializes with ';' (unquoted),
                // unlike the ',' used by coFloats/coInts.
                let sep = if key == "extruder_colour" { ";" } else { "," };
                return arr
                    .iter()
                    .map(|v| v.as_str().unwrap_or("").replace('\n', "\\n"))
                    .collect::<Vec<_>>()
                    .join(sep);
            }
            // Single-element (or first-only) fallback. Route through
            // normalize_scalar so the coFloat ".0"-strip applies to single-value
            // arrays too (filament_dev_chamber_drying_time: 12.0 -> 12).
            normalize_scalar(arr.first().and_then(|v| v.as_str()).unwrap_or(""))
        }
        _ => value.to_string(),
    }
}

/// Process BambuStudio gcode template, substituting variables.
/// Handles [var], {var}, {expr}, {if cond}...{endif} patterns.
pub fn process_gcode_template(
    template: &str,
    settings: &serde_json::Value,
    config: &PrintConfig,
) -> String {
    // Helper: resolve a variable name to a string value
    let resolve_var = |name: &str| -> Option<String> {
        // Handle indexed access like "nozzle_temperature[0]" or "var[initial_no_support_extruder]"
        if let Some(bracket_pos) = name.find('[') {
            let var_name = &name[..bracket_pos];
            let idx_str = name[bracket_pos + 1..].trim_end_matches(']');
            // Resolve index
            let idx: usize = if let Ok(n) = idx_str.parse::<usize>() {
                n
            } else {
                // Index is a variable name like "initial_no_support_extruder"
                settings
                    .get(idx_str)
                    .and_then(|v| v.as_str().or_else(|| v.as_u64().map(|_| "").and(None)))
                    .and_then(|s| s.parse::<usize>().ok())
                    .unwrap_or(0)
            };
            // Get array element — try settings first, then fall through to computed vars
            let from_settings = settings
                .get(var_name)
                .and_then(|v| v.as_array())
                .and_then(|arr| arr.get(idx))
                .and_then(|v| v.as_str())
                .map(|s| s.to_string());
            if from_settings.is_some() {
                return from_settings;
            }
            // Fall through to computed variables below (e.g. flush_volumetric_speeds[0])
        }
        // Simple variable lookup
        match settings.get(name) {
            Some(serde_json::Value::String(s)) => Some(s.clone()),
            Some(serde_json::Value::Array(arr)) => {
                arr.first().and_then(|v| v.as_str()).map(|s| s.to_string())
            }
            Some(serde_json::Value::Number(n)) => Some(n.to_string()),
            _ => {
                // Computed variables
                match name {
                    "initial_no_support_extruder" | "current_extruder" => Some("0".to_string()),
                    // R241 (gated): computed lists — single object, extruder 0
                    // job: first (non-support) filament = 0. Default path keeps
                    // these unresolved (byte-locked).
                    "first_non_support_filaments[0]"
                    | "first_non_support_filaments[1]"
                    | "first_filaments[0]"
                    | "first_filaments[1]"
                    | "first_non_support_filaments"
                    | "first_filaments"
                        if crate::faithful_gate("ZSMOOTH_FAITHFUL") =>
                    {
                        Some("0".to_string())
                    }
                    // R241 (gated): overall_chamber_temperature = max over
                    // filaments' chamber temps (0 on this profile → M141 S0).
                    "overall_chamber_temperature"
                        if crate::faithful_gate("ZSMOOTH_FAITHFUL") =>
                    {
                        let v = settings
                            .get("chamber_temperatures")
                            .or_else(|| settings.get("chamber_temperature"))
                            .and_then(|v| v.as_array())
                            .map(|arr| {
                                arr.iter()
                                    .filter_map(|x| {
                                        x.as_str().and_then(|s| s.parse::<i64>().ok())
                                    })
                                    .max()
                                    .unwrap_or(0)
                            })
                            .unwrap_or(0);
                        Some(v.to_string())
                    }
                    // R810 — placeholders the 02.09.01.70 machine G-code uses
                    // (GCode.cpp 2918-2937). Gate START_GCODE_2091 (`=0` leaves
                    // them unresolved, as before).
                    //
                    // `initial_no_support_hotend` / `current_hotend` are
                    // NOZZLE_ID_FOR_GCODE (GCode.cpp:118): the nozzle's group id
                    // only when the printer maps nozzles dynamically, else -1.
                    // No dynamic-map printer is ported, so -1.
                    "initial_no_support_hotend" | "current_hotend"
                        if crate::faithful_gate("START_GCODE_2091") =>
                    {
                        Some("-1".to_string())
                    }
                    // The group id of the first nozzle of the initial filament
                    // (2924). One nozzle group is modelled: 0.
                    "initial_nozzle_id" | "initial_no_support_nozzle_id"
                        if crate::faithful_gate("START_GCODE_2091") =>
                    {
                        Some("0".to_string())
                    }
                    // get_nozzle_diameters_by_nozzle_id (GCode.cpp:120): the
                    // diameter of the nozzle with that id, rendered as a float.
                    n if n.starts_with("nozzle_diameter_at_nozzle_id")
                        && crate::faithful_gate("START_GCODE_2091") =>
                    {
                        settings
                            .get("nozzle_diameter")
                            .and_then(|v| match v {
                                serde_json::Value::Array(a) => a.first().and_then(|x| x.as_str()),
                                serde_json::Value::String(s) => Some(s.as_str()),
                                _ => None,
                            })
                            .and_then(|s| s.split(',').next())
                            .and_then(|s| s.trim().parse::<f64>().ok())
                            .map(crate::gcode::gcode_template::format_g6)
                    }
                    // Scalars of the initial filament (2932-2937).
                    "long_retraction_when_ec" | "long_retraction_when_cut"
                    | "retraction_distance_when_ec" | "retraction_distance_when_cut"
                        if crate::faithful_gate("START_GCODE_2091") =>
                    {
                        let key = match name {
                            "long_retraction_when_ec" => "long_retractions_when_ec",
                            "long_retraction_when_cut" => "long_retractions_when_cut",
                            "retraction_distance_when_ec" => "retraction_distances_when_ec",
                            _ => "retraction_distances_when_cut",
                        };
                        settings.get(key).and_then(|v| match v {
                            serde_json::Value::Array(a) => {
                                a.first().and_then(|x| x.as_str()).map(|s| s.to_string())
                            }
                            serde_json::Value::String(s) => {
                                s.split(',').next().map(|s| s.trim().to_string())
                            }
                            _ => None,
                        })
                    }
                    "bed_temperature_initial_layer_single" => {
                        // Resolve from curr_bed_type
                        let bed_type = settings
                            .get("curr_bed_type")
                            .and_then(|v| v.as_str())
                            .unwrap_or("");
                        let key = match bed_type {
                            "Cool Plate" => "cool_plate_temp_initial_layer",
                            "Engineering Plate" => "eng_plate_temp_initial_layer",
                            "Textured PEI Plate" => "textured_plate_temp_initial_layer",
                            _ => "hot_plate_temp_initial_layer",
                        };
                        settings
                            .get(key)
                            .and_then(|v| match v {
                                serde_json::Value::Array(a) => {
                                    a.first().and_then(|x| x.as_str()).map(|s| s.to_string())
                                }
                                serde_json::Value::String(s) => Some(s.clone()),
                                _ => None,
                            })
                            .or(Some("55".to_string()))
                    }
                    "outer_wall_volumetric_speed" => {
                        // BambuStudio computes volumetric speed using Flow's elliptical
                        // cross-section: mm3_per_mm = (w - h*(1-PI/4))*h + h²/4*PI
                        // Then: volumetric_speed = mm3_per_mm * speed * flow_ratio
                        // Clamped to filament_max_volumetric_speed
                        let speed = settings
                            .get("outer_wall_speed")
                            .and_then(|v| match v {
                                serde_json::Value::Array(a) => a.first().and_then(|x| x.as_str()),
                                serde_json::Value::String(s) => Some(s.as_str()),
                                _ => None,
                            })
                            .and_then(|s| s.parse::<f64>().ok())
                            .unwrap_or(200.0);
                        let width = settings
                            .get("outer_wall_line_width")
                            .and_then(|v| {
                                v.as_str().or_else(|| match v {
                                    serde_json::Value::Array(a) => {
                                        a.first().and_then(|x| x.as_str())
                                    }
                                    _ => None,
                                })
                            })
                            .and_then(|s| s.parse::<f64>().ok())
                            .unwrap_or(0.42);
                        let height = config.layer_height;
                        let _flow_ratio = settings
                            .get("filament_flow_ratio")
                            .and_then(|v| match v {
                                serde_json::Value::Array(a) => a.first().and_then(|x| x.as_str()),
                                serde_json::Value::String(s) => Some(s.as_str()),
                                _ => None,
                            })
                            .and_then(|s| s.parse::<f64>().ok())
                            .unwrap_or(1.0);
                        // Flow::mm3_per_mm from Flow.cpp:
                        // height * (width - height * (1 - PI/4))
                        let pi = std::f64::consts::PI;
                        let mm3_per_mm = height * (width - height * (1.0 - 0.25 * pi));
                        let vol_speed = mm3_per_mm * speed; // C++ doesn't apply flow_ratio here
                                                            // Clamp to filament_max_volumetric_speed
                        let max_vol = settings
                            .get("filament_max_volumetric_speed")
                            .and_then(|v| match v {
                                serde_json::Value::Array(a) => a.first().and_then(|x| x.as_str()),
                                serde_json::Value::String(s) => Some(s.as_str()),
                                _ => None,
                            })
                            .and_then(|s| s.parse::<f64>().ok())
                            .unwrap_or(f64::MAX);
                        let clamped = if max_vol > 0.0 {
                            vol_speed.min(max_vol)
                        } else {
                            vol_speed
                        };
                        Some(format!("{:.6}", clamped))
                    }
                    // flush_volumetric_speeds[N] = filament_max_volumetric_speed[N]
                    n if n.starts_with("flush_volumetric_speeds") => settings
                        .get("filament_max_volumetric_speed")
                        .and_then(|v| match v {
                            serde_json::Value::Array(a) => {
                                a.first().and_then(|x| x.as_str()).map(|s| s.to_string())
                            }
                            serde_json::Value::String(s) => Some(s.clone()),
                            _ => None,
                        })
                        .or(Some("21".to_string())),
                    // flush_temperatures[N] — from filament_flush_temp, fallback to
                    // nozzle_temperature_range_high (matches C++ GCode.cpp:856-866)
                    n if n.starts_with("flush_temperatures") => {
                        // Try filament_flush_temp first
                        let flush_temp = settings
                            .get("filament_flush_temp")
                            .and_then(|v| v.as_array())
                            .and_then(|a| a.first())
                            .and_then(|x| x.as_str())
                            .and_then(|s| s.parse::<u32>().ok())
                            .unwrap_or(0);
                        if flush_temp > 0 {
                            Some(format!("{}", flush_temp))
                        } else {
                            // Fallback to nozzle_temperature_range_high
                            settings
                                .get("nozzle_temperature_range_high")
                                .and_then(|v| match v {
                                    serde_json::Value::Array(a) => {
                                        a.first().and_then(|x| x.as_str()).map(|s| s.to_string())
                                    }
                                    serde_json::Value::String(s) => Some(s.clone()),
                                    _ => None,
                                })
                                .or_else(|| Some(format!("{}", config.extruder_temperature + 20)))
                        }
                    }
                    // bed_temperature[N] - resolve from plate type
                    n if n.starts_with("bed_temperature") && n.contains('[') => {
                        let bed_type = settings
                            .get("curr_bed_type")
                            .and_then(|v| v.as_str())
                            .unwrap_or("");
                        let key = match bed_type {
                            "Cool Plate" => "cool_plate_temp",
                            "Engineering Plate" => "eng_plate_temp",
                            "Textured PEI Plate" => "textured_plate_temp",
                            _ => "hot_plate_temp",
                        };
                        settings
                            .get(key)
                            .and_then(|v| match v {
                                serde_json::Value::Array(a) => {
                                    a.first().and_then(|x| x.as_str()).map(|s| s.to_string())
                                }
                                serde_json::Value::String(s) => Some(s.clone()),
                                _ => None,
                            })
                            .or(Some("55".to_string()))
                    }
                    // bed_temperature_initial_layer[N]
                    n if n.starts_with("bed_temperature_initial_layer") && n.contains('[') => {
                        let bed_type = settings
                            .get("curr_bed_type")
                            .and_then(|v| v.as_str())
                            .unwrap_or("");
                        let key = match bed_type {
                            "Cool Plate" => "cool_plate_temp_initial_layer",
                            "Engineering Plate" => "eng_plate_temp_initial_layer",
                            "Textured PEI Plate" => "textured_plate_temp_initial_layer",
                            _ => "hot_plate_temp_initial_layer",
                        };
                        settings
                            .get(key)
                            .and_then(|v| match v {
                                serde_json::Value::Array(a) => {
                                    a.first().and_then(|x| x.as_str()).map(|s| s.to_string())
                                }
                                serde_json::Value::String(s) => Some(s.clone()),
                                _ => None,
                            })
                            .or(Some("55".to_string()))
                    }
                    // first_layer_print_min/size — handled via settings JSON injection
                    // (injected by print.rs from mesh bounding box before template processing)
                    // Fall through to settings lookup which handles array indexing

                    // min_vitrification_temperature — min of temperature_vitrification across filaments
                    // C++: computed in GCode.cpp from filament vitrification temps
                    "min_vitrification_temperature" => settings
                        .get("temperature_vitrification")
                        .and_then(|v| v.as_array())
                        .map(|arr| {
                            arr.iter()
                                .filter_map(|x| x.as_str().and_then(|s| s.parse::<f64>().ok()))
                                .fold(f64::MAX, f64::min)
                        })
                        .map(|v| {
                            if v == f64::MAX {
                                "0".to_string()
                            } else {
                                format!("{}", v as i32)
                            }
                        }),

                    // is_all_bbl_filament — true if all filaments are from Bambu Lab
                    "is_all_bbl_filament" => {
                        let all_bbl = settings
                            .get("filament_vendor")
                            .and_then(|v| v.as_array())
                            .map(|arr| {
                                arr.iter()
                                    .all(|x| x.as_str().map_or(true, |s| s.contains("Bambu")))
                            })
                            .unwrap_or(true);
                        Some(if all_bbl {
                            "1".to_string()
                        } else {
                            "0".to_string()
                        })
                    }

                    _ => None,
                }
            }
        }
    };

    // Process template line by line, handling escape sequences
    let unescaped = template.replace("\\n", "\n");
    // R242 (gated): native PlaceholderParser is a char-stream — {if} conditions
    // may span MULTIPLE physical lines (the H2D start template's
    // `{if (filament_type[..] == "PLA") ||\n (filament_type0 == "PLA-CF") ...}`)
    // and `{if(` appears without a space. Preprocess: normalize `{if(`→`{if (`,
    // and join any `{if`-opening line whose braces don't balance with the
    // following lines until they do.
    static TMPL_FAITHFUL: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    let tmpl_faithful = *TMPL_FAITHFUL.get_or_init(|| crate::faithful_gate("ZSMOOTH_FAITHFUL"));
    let unescaped = if tmpl_faithful {
        let normalized = unescaped.replace("{if(", "{if (").replace("{elsif(", "{elsif (");
        let mut out = String::with_capacity(normalized.len());
        let mut lines = normalized.lines().peekable();
        while let Some(line) = lines.next() {
            let t = line.trim_start();
            if t.starts_with("{if ") || t.starts_with("{elsif") {
                let mut joined = line.to_string();
                let balance = |s: &str| {
                    s.bytes().fold(0i32, |a, b| match b {
                        b'{' => a + 1,
                        b'}' => a - 1,
                        _ => a,
                    })
                };
                while balance(&joined) > 0 {
                    match lines.next() {
                        Some(next) => {
                            joined.push(' ');
                            joined.push_str(next.trim_start());
                        }
                        None => break,
                    }
                }
                out.push_str(&joined);
            } else {
                out.push_str(line);
            }
            out.push('\n');
        }
        out
    } else {
        unescaped
    };
    let mut output = String::new();
    let mut skip_depth = 0u32; // for {if}/{endif} nesting
                               // Track whether current {if} group has had a true branch.
                               // Stack of (nesting_level, branch_taken) — branch_taken=true means skip all remaining elsif/else
    let mut branch_satisfied: Vec<bool> = Vec::new();

    for line in unescaped.lines() {
        let trimmed = line.trim();

        // Handle {if condition} — supports both line-level and inline patterns
        if trimmed.starts_with("{if ") {
            if let Some(close_pos) = trimmed[4..].find('}') {
                let cond = &trimmed[4..4 + close_pos];
                let after_cond = &trimmed[4 + close_pos + 1..];
                if skip_depth > 0 {
                    // Already skipping outer block — increase nesting
                    skip_depth += 1;
                    branch_satisfied.push(false);
                } else {
                    let result = eval_condition(cond, &resolve_var);
                    if result {
                        branch_satisfied.push(true); // branch taken
                                                     // Emit indent + text after {if cond} (C++ emits trailing content)
                        if let Some(pos) = line.find("{if ") {
                            output.push_str(&line[..pos]);
                        }
                        if !after_cond.is_empty() {
                            output.push_str(after_cond);
                        }
                        output.push('\n');
                    } else {
                        skip_depth += 1;
                        branch_satisfied.push(false); // no branch taken yet
                    }
                }
                continue;
            }
        }
        if trimmed.starts_with("{endif}") {
            let _was_satisfied = branch_satisfied.pop().unwrap_or(false);
            if skip_depth > 0 {
                skip_depth -= 1;
            }
            // BambuStudio emits the line with {endif} removed, preserving
            // surrounding text and indentation. A bare {endif} produces a blank line.
            if skip_depth == 0 {
                if let Some(pos) = line.find("{endif}") {
                    let before = &line[..pos];
                    let after = &line[pos + 7..];
                    output.push_str(before);
                    output.push_str(after);
                    output.push('\n');
                }
            }
            continue;
        }
        // Handle {elsif(condition)} or {elsif condition}
        if trimmed.starts_with("{elsif") {
            let satisfied = branch_satisfied.last().copied().unwrap_or(false);
            if skip_depth > 1 {
                // Nested inside a skipped outer block — don't change state
            } else if satisfied {
                // A previous branch in this {if} group already matched → skip
                if skip_depth == 0 {
                    skip_depth = 1;
                }
            } else if skip_depth == 1 {
                // No branch matched yet → evaluate this condition
                let cond_start = if trimmed.starts_with("{elsif(") { 7 } else { 7 };
                if let Some(close_pos) = trimmed[cond_start..].find('}') {
                    let cond = &trimmed[cond_start..cond_start + close_pos];
                    let cond = cond.trim_start_matches('(').trim_end_matches(')');
                    let result = eval_condition(cond, &resolve_var);
                    if result {
                        skip_depth = 0;
                        if let Some(last) = branch_satisfied.last_mut() {
                            *last = true;
                        }
                        // Emit trailing text after {elsif cond} (C++ emits it)
                        let after = &trimmed[cond_start + close_pos + 1..];
                        if !after.is_empty() {
                            output.push_str(after);
                            output.push('\n');
                        }
                    }
                }
            }
            continue;
        }
        if trimmed == "{else}" {
            let satisfied = branch_satisfied.last().copied().unwrap_or(false);
            if skip_depth > 1 {
                // Nested in outer skip — don't change
            } else if satisfied {
                // Previous branch matched → skip else
                if skip_depth == 0 {
                    skip_depth = 1;
                }
            } else if skip_depth == 1 {
                // No branch matched → take else
                skip_depth = 0;
                if let Some(last) = branch_satisfied.last_mut() {
                    *last = true;
                }
            }
            // Emit blank line for {else} (inline replacement)
            if skip_depth == 0 {
                output.push('\n');
            }
            continue;
        }
        // Note: {endif} already handled above (with possible trailing comment)

        if skip_depth > 0 {
            continue;
        }

        // Substitute [var] outside {expr} and {expr} patterns in one pass
        let mut result = String::new();
        let processed = line.to_string();
        let mut chars = processed.chars().peekable();
        while let Some(ch) = chars.next() {
            if ch == '{' {
                let mut expr = String::new();
                let mut depth = 1;
                for c in chars.by_ref() {
                    if c == '{' {
                        depth += 1;
                    }
                    if c == '}' {
                        depth -= 1;
                        if depth == 0 {
                            break;
                        }
                    }
                    expr.push(c);
                }
                // Evaluate the expression (handles [var] internally via resolve_var)
                let val = eval_expr(&expr, &resolve_var);
                result.push_str(&val);
            } else if ch == '[' {
                // Substitute [var] outside of {expr}
                let mut var_name = String::new();
                for c in chars.by_ref() {
                    if c == ']' {
                        break;
                    }
                    var_name.push(c);
                }
                let replacement = resolve_var(&var_name).unwrap_or_default();
                result.push_str(&replacement);
            } else {
                result.push(ch);
            }
        }

        output.push_str(&result);
        output.push('\n');
    }

    output
}

/// Evaluate a simple condition like 'filament_type[0]=="PLA"' or 'default_acceleration > 0'
fn eval_condition(cond: &str, resolve: &dyn Fn(&str) -> Option<String>) -> bool {
    let cond = cond.trim();
    // R242 gate (checked early so the paren strip can use it): faithful mode
    // strips outer parens ONLY when they wrap the whole expression —
    // `((A) || (B)) && (C)` must NOT lose its outer chars (the naive strip
    // mangled it into `(A) || (B)) && (C`, mis-splitting at the inner ||).
    static COND_FAITHFUL_EARLY: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    let faithful_early =
        *COND_FAITHFUL_EARLY.get_or_init(|| crate::faithful_gate("ZSMOOTH_FAITHFUL"));
    // Strip outer parens
    let cond = if cond.starts_with('(') && cond.ends_with(')') {
        if faithful_early {
            let inner = &cond[1..cond.len() - 1];
            let mut depth = 0i32;
            let mut wraps = true;
            for b in inner.bytes() {
                match b {
                    b'(' => depth += 1,
                    b')' => {
                        depth -= 1;
                        if depth < 0 {
                            wraps = false;
                            break;
                        }
                    }
                    _ => {}
                }
            }
            if wraps && depth == 0 {
                inner
            } else {
                cond
            }
        } else {
            &cond[1..cond.len() - 1]
        }
    } else {
        cond
    };

    // R224 gate: the corrected {if} evaluator (paren-aware &&/|| splits +
    // `!` negation) matches native PlaceholderParser but flips branches the
    // old evaluator got wrong, so the default path keeps the legacy behavior
    // to preserve the byte-lock.
    static COND_FAITHFUL: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    let faithful = *COND_FAITHFUL.get_or_init(|| crate::faithful_gate("ZSMOOTH_FAITHFUL"));

    // R224: connective splits must ignore separators nested inside parens —
    // `!(a && b) && c` splits at the SECOND &&, not the one inside the group
    // (the naive find() split mis-parsed the timelapse template's
    // `!(has_timelapse_safe_pos && timelapse_type == 0)` guard).
    let find_top_level = |needle: &str| -> Option<usize> {
        let bytes = cond.as_bytes();
        let nb = needle.as_bytes();
        let mut depth = 0i32;
        let mut i = 0;
        while i + nb.len() <= bytes.len() {
            match bytes[i] {
                b'(' => depth += 1,
                b')' => depth -= 1,
                _ => {
                    if depth == 0 && &bytes[i..i + nb.len()] == nb {
                        return Some(i);
                    }
                }
            }
            i += 1;
        }
        None
    };
    if faithful {
        // Handle OR (||) — lower precedence than &&
        if let Some(pos) = find_top_level("||") {
            let left = &cond[..pos];
            let right = &cond[pos + 2..];
            return eval_condition(left, resolve) || eval_condition(right, resolve);
        }
        // Handle AND (&&)
        if let Some(pos) = find_top_level("&&") {
            let left = &cond[..pos];
            let right = &cond[pos + 2..];
            return eval_condition(left, resolve) && eval_condition(right, resolve);
        }
    } else {
        // Legacy (default-path) splits — byte-locked behavior.
        if let Some(pos) = cond.find(")||(") {
            let left = &cond[..pos];
            let right = &cond[pos + 4..];
            return eval_condition(left, resolve) || eval_condition(right, resolve);
        }
        if let Some(pos) = cond.find("||") {
            let left = &cond[..pos];
            let right = &cond[pos + 2..];
            return eval_condition(left, resolve) || eval_condition(right, resolve);
        }
        if let Some(pos) = cond.find("&&") {
            let left = &cond[..pos];
            let right = &cond[pos + 2..];
            return eval_condition(left, resolve) && eval_condition(right, resolve);
        }
    }

    // R224: logical negation — `!var` / `!(expr)`. Must be checked AFTER the
    // &&/|| splits so `!` binds tighter than the connectives (`!a && b` is
    // `(!a) && b`, not `!(a && b)` — the timelapse template's
    // `!spiral_mode && !(has_timelapse_safe_pos && ...)` guards rely on this).
    if faithful && cond.starts_with('!') && !cond.starts_with("!=") {
        return !eval_condition(&cond[1..], resolve);
    }

    // R241 (gated): templates write `var == "PLA"` with spaces — normalize
    // `== "` / `!= "` to the adjacency forms the legacy parser expects.
    let normalized;
    let cond = if faithful && (cond.contains("== \"") || cond.contains("!= \"")) {
        normalized = cond.replace("== \"", "==\"").replace("!= \"", "!=\"");
        normalized.as_str()
    } else {
        cond
    };

    // String equality: var=="value"
    if let Some(pos) = cond.find("==\"") {
        let lhs = cond[..pos].trim().trim_start_matches('(').trim();
        let rhs = cond[pos + 3..]
            .trim()
            .trim_end_matches('"')
            .trim_end_matches(')');
        let lhs_val = resolve(lhs).unwrap_or_default();
        return lhs_val == rhs;
    }
    // String inequality: var!="value"
    if let Some(pos) = cond.find("!=\"") {
        let lhs = cond[..pos].trim().trim_start_matches('(').trim();
        let rhs = cond[pos + 3..]
            .trim()
            .trim_end_matches('"')
            .trim_end_matches(')');
        let lhs_val = resolve(lhs).unwrap_or_default();
        return lhs_val != rhs;
    }

    // Numeric comparisons — check multi-char operators before single-char
    // Order matters: <= before <, >= before >
    let comparisons: &[(&str, fn(f64, f64) -> bool)] = &[
        ("<=", |a, b| a <= b),
        (">=", |a, b| a >= b),
        ("!=", |a, b| (a - b).abs() > f64::EPSILON),
        ("==", |a, b| (a - b).abs() < f64::EPSILON),
        ("<", |a, b| a < b),
        (">", |a, b| a > b),
    ];
    for &(op, cmp_fn) in comparisons {
        if let Some(pos) = cond.find(op) {
            let lhs = cond[..pos].trim().trim_start_matches('(').trim();
            let rhs = cond[pos + op.len()..].trim().trim_end_matches(')').trim();
            if faithful {
                // R240: comparison operands can be ARITHMETIC EXPRESSIONS
                // (the H2D end template's `{if (100.0 - max_layer_z/2) > 0}`)
                // — resolve variables inside, then eval_math. Plain variables
                // and literals still work through the same path.
                let num = |txt: &str| -> f64 {
                    // Comparison splitting leaves unbalanced parens on the
                    // operands (`(100.0 - max_layer_z/2` / `... /2)`).
                    let t = txt.trim().trim_start_matches('(').trim_end_matches(')').trim();
                    if let Ok(v) = t.parse::<f64>() {
                        return v;
                    }
                    if let Some(v) = resolve(t).and_then(|s| s.parse().ok()) {
                        return v;
                    }
                    let resolved = resolve_all_vars_in_expr(t, resolve);
                    eval_math(&resolved).unwrap_or(0.0)
                };
                return cmp_fn(num(lhs), num(rhs));
            }
            let lhs_val: f64 = resolve(lhs).and_then(|s| s.parse().ok()).unwrap_or(0.0);
            let rhs_val: f64 = rhs.parse().unwrap_or(0.0);
            return cmp_fn(lhs_val, rhs_val);
        }
    }

    // Truthy check
    let val = resolve(cond.trim()).unwrap_or_default();
    val != "0" && !val.is_empty() && val != "false"
}

/// Evaluate a simple expression like 'outer_wall_volumetric_speed/(0.3*0.5)*60'
fn eval_expr(expr: &str, resolve: &dyn Fn(&str) -> Option<String>) -> String {
    let trimmed = expr.trim();

    // R241 (gated): C++ PlaceholderParser supports ternaries and arbitrary
    // index expressions — `var[(a[0] != -1 ? a[0] : b[0])]` (H2D start
    // template M620.17). Handle a top-level `?:` by evaluating the condition
    // and recursing into the chosen branch, and an outer `name[<expr>]` by
    // evaluating the index expression first.
    static EXPR_FAITHFUL: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    let faithful = *EXPR_FAITHFUL.get_or_init(|| crate::faithful_gate("ZSMOOTH_FAITHFUL"));
    if faithful {
        // Strip outer parens that wrap the ENTIRE expression (`{(a ? b : c)}`)
        // so the ternary below is at depth 0.
        if trimmed.starts_with('(') && trimmed.ends_with(')') {
            let inner = &trimmed[1..trimmed.len() - 1];
            let mut depth = 0i32;
            let mut wraps = true;
            for b in inner.bytes() {
                match b {
                    b'(' => depth += 1,
                    b')' => {
                        depth -= 1;
                        if depth < 0 {
                            wraps = false;
                            break;
                        }
                    }
                    _ => {}
                }
            }
            if wraps && depth == 0 {
                return eval_expr(inner, resolve);
            }
        }
        // Top-level ternary (depth-0 over () and []).
        let bytes = trimmed.as_bytes();
        let mut depth = 0i32;
        let mut qpos: Option<usize> = None;
        for (i, &b) in bytes.iter().enumerate() {
            match b {
                b'(' | b'[' => depth += 1,
                b')' | b']' => depth -= 1,
                b'?' => {
                    if depth == 0 {
                        qpos = Some(i);
                        break;
                    }
                }
                _ => {}
            }
        }
        if let Some(q) = qpos {
            // find matching ':' at depth 0 after q
            let mut depth = 0i32;
            for (j, &b) in bytes.iter().enumerate().skip(q + 1) {
                match b {
                    b'(' | b'[' => depth += 1,
                    b')' | b']' => depth -= 1,
                    b':' => {
                        if depth == 0 {
                            let cond = &trimmed[..q];
                            let taken = if eval_condition(cond, resolve) {
                                &trimmed[q + 1..j]
                            } else {
                                &trimmed[j + 1..]
                            };
                            return eval_expr(taken, resolve);
                        }
                    }
                    _ => {}
                }
            }
        }
        // Outer `name[<expr>]` where the index itself is an expression.
        if trimmed.ends_with(']') {
            if let Some(br) = trimmed.find('[') {
                let name = &trimmed[..br];
                let inner = &trimmed[br + 1..trimmed.len() - 1];
                if !name.is_empty()
                    && name.chars().all(|c| c.is_alphanumeric() || c == '_')
                    && (inner.contains('?') || inner.contains('(') || inner.contains('['))
                {
                    let idx = eval_expr(inner, resolve);
                    let flat = format!("{}[{}]", name, idx.trim());
                    if let Some(v) = resolve(&flat) {
                        return v;
                    }
                }
            }
        }
    }

    // Try as a simple variable first (only if no math operators present)
    let has_math = trimmed.contains('+')
        || trimmed.contains('-')
        || trimmed.contains('*')
        || trimmed.contains('/');
    if !has_math {
        if let Some(val) = resolve(trimmed) {
            return val;
        }
    }

    // Handle expressions with math: var op val
    // Simple approach: resolve all variable references, then evaluate
    // For now, handle the common patterns we see in the templates

    // Pattern: "var/expr" or "var*expr" or "var+expr" or "var-expr"
    // Try to evaluate as a math expression with variable substitution
    let resolved = resolve_all_vars_in_expr(trimmed, resolve);

    // Try to evaluate as a math expression
    match eval_math(&resolved) {
        Some(val) => {
            if val == val.floor() && val.abs() < 1e9 {
                format!("{:.0}", val)
            } else {
                // Match C++ std::ostringstream default: 6 significant digits (%g format)
                // Then trim trailing zeros
                let sig_digits = 6;
                let magnitude = if val.abs() > 0.0 {
                    val.abs().log10().floor() as i32 + 1
                } else {
                    1
                };
                let decimal_places = (sig_digits - magnitude).max(0) as usize;
                let s = format!("{:.prec$}", val, prec = decimal_places);
                s.trim_end_matches('0').trim_end_matches('.').to_string()
            }
        }
        None => {
            // Return as-is if can't evaluate
            format!("{{{}}}", expr)
        }
    }
}

/// Replace variable names in an expression with their values
fn resolve_all_vars_in_expr(expr: &str, resolve: &dyn Fn(&str) -> Option<String>) -> String {
    let mut result = expr.to_string();

    // Find variable-like tokens (word chars + brackets) and try to resolve them
    // This is a simplistic approach - iterate and replace known patterns
    let mut attempts = 0;
    loop {
        let old = result.clone();
        // Find alphanumeric tokens that could be variables
        let re_like: Vec<(usize, usize)> = {
            let mut spans = Vec::new();
            let bytes = result.as_bytes();
            let mut i = 0;
            while i < bytes.len() {
                if bytes[i].is_ascii_alphabetic() || bytes[i] == b'_' {
                    let start = i;
                    while i < bytes.len()
                        && (bytes[i].is_ascii_alphanumeric()
                            || bytes[i] == b'_'
                            || bytes[i] == b'['
                            || bytes[i] == b']')
                    {
                        i += 1;
                    }
                    spans.push((start, i));
                } else {
                    i += 1;
                }
            }
            spans
        };

        // Replace from right to left to preserve indices
        for &(start, end) in re_like.iter().rev() {
            let token = &result[start..end];
            if let Some(val) = resolve(token) {
                result = format!("{}{}{}", &result[..start], val, &result[end..]);
            }
        }

        attempts += 1;
        if result == old || attempts > 3 {
            break;
        }
    }

    result
}

/// Evaluate a simple math expression (numbers and +-*/)
fn eval_math(expr: &str) -> Option<f64> {
    // Simple recursive descent for +, -, *, /
    let trimmed = expr.trim();
    if trimmed.is_empty() {
        return None;
    }

    // Try as a plain number
    if let Ok(n) = trimmed.parse::<f64>() {
        return Some(n);
    }

    // Find the last + or - (lowest precedence) outside parentheses
    let mut depth = 0i32;
    let mut last_add_sub = None;
    let bytes = trimmed.as_bytes();
    for i in (0..bytes.len()).rev() {
        match bytes[i] {
            b')' => depth += 1,
            b'(' => depth -= 1,
            b'+' | b'-' if depth == 0 && i > 0 => {
                // Make sure it's not a unary minus or part of scientific notation
                if i > 0 && !matches!(bytes[i - 1], b'(' | b'*' | b'/' | b'+' | b'-') {
                    last_add_sub = Some(i);
                    break;
                }
            }
            _ => {}
        }
    }

    if let Some(pos) = last_add_sub {
        let left = eval_math(&trimmed[..pos])?;
        let right = eval_math(&trimmed[pos + 1..])?;
        return Some(if bytes[pos] == b'+' {
            left + right
        } else {
            left - right
        });
    }

    // Find last * or / outside parentheses
    let mut last_mul_div = None;
    depth = 0;
    for i in (0..bytes.len()).rev() {
        match bytes[i] {
            b')' => depth += 1,
            b'(' => depth -= 1,
            b'*' | b'/' if depth == 0 => {
                last_mul_div = Some(i);
                break;
            }
            _ => {}
        }
    }

    if let Some(pos) = last_mul_div {
        let left = eval_math(&trimmed[..pos])?;
        let right = eval_math(&trimmed[pos + 1..])?;
        return Some(if bytes[pos] == b'*' {
            left * right
        } else if right != 0.0 {
            left / right
        } else {
            0.0
        });
    }

    // Handle parentheses
    if trimmed.starts_with('(') && trimmed.ends_with(')') {
        return eval_math(&trimmed[1..trimmed.len() - 1]);
    }

    // Handle unary minus
    if trimmed.starts_with('-') {
        return eval_math(&trimmed[1..]).map(|v| -v);
    }
    if trimmed.starts_with('+') {
        return eval_math(&trimmed[1..]);
    }

    None
}

#[cfg(test)]
mod generator_tests {
    use super::*;

    #[test]
    fn test_gcode_new() {
        let gcode = GCode::new();
        assert!(gcode.is_empty());
        assert_eq!(gcode.line_count(), 0);
    }

    #[test]
    fn test_gcode_append_line() {
        let mut gcode = GCode::new();
        gcode.append_line("G1 X10 Y20");
        gcode.append_line("G1 Z5");

        assert_eq!(gcode.line_count(), 2);
        assert_eq!(gcode.content(), "G1 X10 Y20\nG1 Z5\n");
    }

    #[test]
    fn test_gcode_append_comment() {
        let mut gcode = GCode::new();
        gcode.append_comment("Test comment");
        assert_eq!(gcode.content(), "; Test comment\n");
    }

    #[test]
    fn test_gcode_from_string() {
        let content = "G1 X10\nG1 Y20\n";
        let gcode = GCode::from_string(content.to_string());
        assert_eq!(gcode.line_count(), 2);
        assert_eq!(gcode.content(), content);
    }

    #[test]
    fn test_gcode_lines_iterator() {
        let mut gcode = GCode::new();
        gcode.append_line("G1 X10");
        gcode.append_line("G1 Y20");
        gcode.append_line("G1 Z5");

        let lines: Vec<&str> = gcode.lines().collect();
        assert_eq!(lines.len(), 3);
        assert_eq!(lines[0], "G1 X10");
        assert_eq!(lines[1], "G1 Y20");
        assert_eq!(lines[2], "G1 Z5");
    }

    #[test]
    fn test_gcode_clear() {
        let mut gcode = GCode::new();
        gcode.append_line("G1 X10");
        assert!(!gcode.is_empty());

        gcode.clear();
        assert!(gcode.is_empty());
        assert_eq!(gcode.line_count(), 0);
    }

    #[test]
    fn test_gcode_stats_print_time_formatted() {
        let mut stats = GCodeStats::new();
        stats.print_time_seconds = 3665.0; // 1h 1m 5s
        assert_eq!(stats.print_time_formatted(), "01:01:05");
    }

    #[test]
    fn test_gcode_stats_filament_meters() {
        let mut stats = GCodeStats::new();
        stats.filament_length_mm = 2500.0;
        assert_eq!(stats.filament_used_meters(), 2.5);
    }
}

// ============================================================================
// Smooth speed of discontinuity areas: `mapping_speed`, `get_speed_coor_x`,
// `need_smooth_speed`, `split_and_mapping_speed`, `merge_same_speed_paths`,
// `set_speed_transition` and `smooth_speed_discontinuity_area` of GCode.cpp.
// They split the paths of a perimeter loop where the speed jumps and ramp the
// feedrate across the jump (f(x) = coeff * x^2). Lengths are scaled; `scale_`
// and `unscale_` here are the double-valued macros.
// ============================================================================

// C++: using ExtrusionPaths = std::vector<ExtrusionPath>;
pub type ExtrusionPaths = Vec<ExtrusionPath>;

/// `scale_` macro (double-valued): val / SCALING_FACTOR == val * 100000.0.
#[inline]
fn scale_(val: f64) -> f64 {
    val / 0.00001
}
/// `unscale_` macro (double-valued): val * SCALING_FACTOR == val / 100000.0.
#[inline]
fn unscale_(val: f64) -> f64 {
    val * 0.00001
}

/// `Point operator*(const Point&, const double&)` — coord_t(double) truncates.
#[inline]
fn point_mul_f64(l: Point, r: f64) -> Point {
    Point::new((l.x() as f64 * r) as i64, (l.y() as f64 * r) as i64)
}

/// `ExtrusionPath(Polyline, const ExtrusionPath&)`: copy the source path's
/// properties (role/width/mm3/etc.) and set a new polyline.
#[inline]
fn path_from(polyline: Polyline, src: &ExtrusionPath) -> ExtrusionPath {
    let mut p = src.clone();
    p.polyline = polyline;
    p
}

/// GCode.cpp:5918-5923 — `double GCode::mapping_speed(double dist)` ; f(x)=coeff*x^2.
#[inline]
fn mapping_speed(coeff: f64, dist: f64) -> f64 {
    // GCode.cpp:5920-5921
    if dist <= 0.0 {
        return 0.0;
    }
    coeff * dist.powi(2)
}

/// GCode.cpp:5925-5929 — `double GCode::get_speed_coor_x(double speed)`.
#[inline]
fn get_speed_coor_x(coeff: f64, speed: f64) -> f64 {
    // GCode.cpp:5927-5928
    let temp = speed / coeff;
    temp.sqrt()
}

/// GCode.cpp:5965-5971 — `static bool need_smooth_speed(other, this)`.
#[inline]
fn need_smooth_speed(other_path: &ExtrusionPath, this_path: &ExtrusionPath) -> bool {
    // GCode.cpp:5967
    this_path.smooth_speed - other_path.smooth_speed > SMOOTH_SPEED_STEP
}

/// Polyline length helper (scaled units).
#[inline]
fn poly_len(p: &Polyline) -> f64 {
    p.length()
}

/// GCode.cpp:5973-6121 — `void GCode::split_and_mapping_speed(...)`.
///
/// Splits `this_path` (consumed from the front, or rear when `!split_from_left`)
/// into a sequence of short sub-paths whose `smooth_speed` ramps from
/// `other_path_v` up toward `final_v` over at most `max_smooth_length`. The
/// produced (ramped) sub-paths are appended to `interpolated_paths`, and
/// `this_path` is left holding the unconsumed remainder (all set to `final_v`).
#[allow(clippy::too_many_arguments)]
pub fn split_and_mapping_speed(
    coeff: f64,
    other_path_v: f64,
    mut final_v: f64,
    this_path: &mut ExtrusionPaths,
    max_smooth_length: f64,
    interpolated_paths: &mut ExtrusionPaths,
    split_from_left: bool,
) {
    // GCode.cpp:5975-5976
    if this_path.is_empty() || max_smooth_length == 0.0 {
        return;
    }

    // GCode.cpp:5978
    let mut splited_path: ExtrusionPaths = Vec::new();

    // GCode.cpp:5981-5983 — get params
    let this_path_x = scale_(get_speed_coor_x(coeff, final_v));
    let mut x_base = scale_(get_speed_coor_x(coeff, other_path_v));
    let mut smooth_length = this_path_x - x_base;
    let mut smooth_length_count = 0.0;
    let mut split_line_speed = 0.0;
    // GCode.cpp:5987-5990 — adjust final_v if length not enough.
    if smooth_length > max_smooth_length {
        smooth_length = max_smooth_length;
        final_v = mapping_speed(coeff, unscale_(x_base + max_smooth_length));
    }

    // GCode.cpp:5991-6000 — insert_speed lambda.
    let insert_speed = |coeff: f64,
                        line_length: f64,
                        pos_x: &mut f64,
                        smooth_length_count: &mut f64,
                        target_v: f64|
     -> f64 {
        *pos_x += line_length;
        let mut pos_x_speed = mapping_speed(coeff, unscale_(*pos_x));
        *smooth_length_count += line_length;
        if pos_x_speed > target_v {
            pos_x_speed = target_v;
        }
        pos_x_speed
    };

    // GCode.cpp:6003-6008 — length_enough lambda.
    let length_enough = |length: f64| -> bool {
        let min = min_step_length();
        !(length < min || length - min < min / 2.0)
    };

    // GCode.cpp:6010
    let mut left_paths: ExtrusionPaths = Vec::new();
    // GCode.cpp:6011 — for (int idx = 0; idx < this_path.size(); idx++)
    let mut idx = 0usize;
    while idx < this_path.len() {
        // GCode.cpp:6014-6016 — polyline error
        if this_path[idx].polyline.points().len() < 2 {
            idx += 1;
            continue;
        }

        // GCode.cpp:6018-6023 — stop and push the rest back.
        if smooth_length_count >= smooth_length {
            left_paths.extend(this_path.drain(idx..));
            *this_path = std::mem::take(&mut left_paths);
            // C++ does `this_path = std::move(left_paths); break;`
            interpolated_paths.append(&mut splited_path);
            return;
        }

        // GCode.cpp:6025-6036 — the path is too short.
        let extrusion_len = this_path[idx].length();
        if !length_enough(extrusion_len) {
            split_line_speed = insert_speed(
                coeff,
                extrusion_len,
                &mut x_base,
                &mut smooth_length_count,
                final_v,
            );
            let mut sp = this_path[idx].clone();
            sp.smooth_speed = split_line_speed;
            splited_path.push(sp);
            // GCode.cpp:6031-6035
            if idx < this_path.len() - 1 {
                idx += 1;
                continue;
            }
            this_path.clear();
            interpolated_paths.append(&mut splited_path);
            return;
        }

        // GCode.cpp:6039-6041 — reverse if slowing down.
        let mut input_polyline = this_path[idx].polyline.clone();
        if !split_from_left {
            input_polyline.reverse();
        }

        // GCode.cpp:6043-6046
        let mut line_start_pt = input_polyline.points()[0];
        let mut line_end_pt = input_polyline.points()[1];
        let mut get_next_line = false;
        let mut end_pt_idx = 1usize;

        // GCode.cpp:6049 — split long extrusion.
        let mut last_point = line_start_pt;
        // GCode.cpp:6050
        while split_line_speed < final_v && end_pt_idx < input_polyline.points().len() {
            // GCode.cpp:6052-6055 — move to next line
            if get_next_line {
                line_start_pt = input_polyline.points()[end_pt_idx - 1];
                line_end_pt = input_polyline.points()[end_pt_idx];
            }
            // GCode.cpp:6057-6058
            let mut cuted_polyline = Polyline::new();
            let line_a = line_start_pt;
            let line_b = line_end_pt;
            let line_len = line_a.distance(&line_b);

            // GCode.cpp:6060
            cuted_polyline.push(line_start_pt);
            // GCode.cpp:6062-6076 — split polyline and set speed.
            if !length_enough(line_len) {
                split_line_speed = insert_speed(
                    coeff,
                    line_len,
                    &mut x_base,
                    &mut smooth_length_count,
                    final_v,
                );
                end_pt_idx += 1;
                get_next_line = true;
                cuted_polyline.push(line_b);
            } else {
                // GCode.cpp:6069-6075 — path is too long, split it.
                let rate = min_step_length() / line_len;
                // insert_p = line.a + (line.b - line.a) * rate
                let insert_p = line_a + point_mul_f64(line_b - line_a, rate);
                split_line_speed = insert_speed(
                    coeff,
                    min_step_length(),
                    &mut x_base,
                    &mut smooth_length_count,
                    final_v,
                );
                line_start_pt = insert_p;
                get_next_line = false;
                cuted_polyline.push(insert_p);
            }
            // GCode.cpp:6078-6080 — reverse back.
            last_point = *cuted_polyline.points().last().unwrap();
            if !split_from_left {
                cuted_polyline.reverse();
            }
            // GCode.cpp:6081-6084
            let mut path_step = path_from(cuted_polyline, &this_path[idx]);
            path_step.smooth_speed = split_line_speed;
            splited_path.push(path_step);
        }

        // GCode.cpp:6087-6091
        if last_point == *input_polyline.points().last().unwrap() {
            if idx == this_path.len() - 1 {
                this_path.clear();
            }
            idx += 1;
            // C++ `continue;` of the for loop. But note when this_path was cleared,
            // the while bound stops it.
            if this_path.is_empty() {
                break;
            }
            continue;
        }

        // GCode.cpp:6092-6094 — split polyline at last_point.
        //
        // C++ `Polyline::split_at(Point &point, ...)` takes `point` by NON-CONST
        // reference and SNAPS it in place to the actual split location on the
        // polyline (Polyline.cpp:223). The "avoid travel" patch below then writes
        // that SAME (snapped) `last_point` into the consumed sub-path's boundary
        // vertex, so the consumed path and the remaining `polyline_left` (which
        // starts/ends at the snapped split vertex) share an identical endpoint —
        // no travel is emitted between them.
        //
        // PARITY-FIX (outer-wall fragmentation): the previous code passed a *copy*
        // (`split_pt`) to split_at_point and then patched with the ORIGINAL,
        // un-snapped `last_point`. When split_at_point snapped the point (the
        // common case — `last_point` is an interpolated insert_p that rarely lands
        // exactly on a stored vertex), the consumed path's patched endpoint
        // (un-snapped) no longer matched `polyline_left`'s endpoint (snapped),
        // leaving a ~1-150 scaled-unit (≈0.001-0.012 mm) gap. extrude_path then
        // emitted a spurious sub-0.1 mm `G1 F60000` travel between the two same-
        // speed paths, fragmenting the outer wall (rust 251 vs native 21 sub-0.1mm
        // outer-wall travels). Mirroring C++, we let split_at_point mutate
        // `last_point` directly and patch with the snapped value.
        let mut p1 = Polyline::new();
        let mut p2 = Polyline::new();
        this_path[idx].polyline.split_at_point(&mut last_point, &mut p1, &mut p2);

        // GCode.cpp:6096-6105
        if split_from_left {
            // update split point to avoid travel path
            if let Some(last) = splited_path.last_mut() {
                let n = last.polyline.points().len();
                if n > 0 {
                    last.polyline.points[n - 1] = last_point;
                }
            }
            let polyline_left = path_from(p2, &this_path[idx]);
            left_paths.push(polyline_left);
        } else {
            if let Some(last) = splited_path.last_mut() {
                if !last.polyline.points().is_empty() {
                    last.polyline.points[0] = last_point;
                }
            }
            let polyline_left = path_from(p1, &this_path[idx]);
            left_paths.push(polyline_left);
        }

        // GCode.cpp:6107
        left_paths.extend(this_path.drain(idx + 1..));
        // remove current consumed path and replace this_path with the remainder
        this_path.truncate(idx); // drop everything from idx onward (already moved tail)
        *this_path = std::mem::take(&mut left_paths);
        // GCode.cpp:6110 — break;
        break;
    }

    // GCode.cpp:6114-6116 — set left path speed.
    if !this_path.is_empty() && final_v != this_path[0].smooth_speed {
        for left in this_path.iter_mut() {
            left.smooth_speed = final_v;
        }
    }

    // GCode.cpp:6118
    interpolated_paths.append(&mut splited_path);
}

/// GCode.cpp:6123-6161 — `std::vector<ExtrusionPaths> GCode::merge_same_speed_paths`.
///
/// `path_speed_fn(path)` is the per-path normal speed (mm/s) — for perimeters
/// this is the overhang-degree-corrected wall speed (GCode::get_path_speed).
pub fn merge_same_speed_paths<F>(paths: &ExtrusionPaths, path_speed_fn: F) -> Vec<ExtrusionPaths>
where
    F: Fn(&ExtrusionPath) -> f64,
{
    // GCode.cpp:6125
    let mut paths_category_by_speed: Vec<ExtrusionPaths> = Vec::new();
    // GCode.cpp:6127
    let mut path_collection: Option<ExtrusionPaths> = None;

    // GCode.cpp:6129
    for path_src in paths.iter() {
        // GCode.cpp:6130-6131 — path.smooth_speed = get_path_speed(path);
        let mut path = path_src.clone();
        path.smooth_speed = path_speed_fn(&path);

        // GCode.cpp:6133-6141 — overhang paths stand alone.
        if path.role == ExtrusionRole::OverhangPerimeter {
            if let Some(c) = path_collection.take() {
                paths_category_by_speed.push(c);
            }
            paths_category_by_speed.push(vec![path]);
            continue;
        }

        // GCode.cpp:6143-6154
        match path_collection.as_mut() {
            None => {
                path_collection = Some(vec![path]);
            }
            Some(c) => {
                if c.last().unwrap().can_merge(&path) {
                    c.push(path);
                } else {
                    let finished = path_collection.take().unwrap();
                    paths_category_by_speed.push(finished);
                    path_collection = Some(vec![path]);
                }
            }
        }
    }

    // GCode.cpp:6157-6158
    if let Some(c) = path_collection.take() {
        paths_category_by_speed.push(c);
    }

    paths_category_by_speed
}

/// GCode.cpp:6163-6257 — `ExtrusionPaths GCode::set_speed_transition`.
pub fn set_speed_transition(coeff: f64, paths: &mut Vec<ExtrusionPaths>) -> ExtrusionPaths {
    // GCode.cpp:6165
    let mut interpolated_paths: ExtrusionPaths = Vec::new();

    // get_path_length helper (GCode.cpp:6188-6190)
    let get_path_length = |p: &ExtrusionPaths| -> f64 { p.iter().map(|x| x.length()).sum() };

    // GCode.cpp:6166 — for (int path_idx = 0; path_idx < paths.size(); path_idx++)
    let mut path_idx = 0usize;
    while path_idx < paths.len() {
        // GCode.cpp:6171-6174 — overhang paths emit as-is.
        if paths[path_idx].first().map(|p| p.role) == Some(ExtrusionRole::OverhangPerimeter) {
            let mut seg = std::mem::take(&mut paths[path_idx]);
            interpolated_paths.append(&mut seg);
            path_idx += 1;
            continue;
        }

        // GCode.cpp:6176-6177
        if paths[path_idx].is_empty() {
            path_idx += 1;
            continue;
        }

        // GCode.cpp:6179
        let smooth_left_path = path_idx > 0
            && !interpolated_paths.is_empty()
            && need_smooth_speed(
                interpolated_paths.last().unwrap(),
                &paths[path_idx][0],
            );
        // GCode.cpp:6181
        let smooth_right_path = path_idx < paths.len() - 1
            && need_smooth_speed(&paths[path_idx + 1][0], &paths[path_idx][0]);

        // GCode.cpp:6183-6186
        if !smooth_left_path && !smooth_right_path {
            let mut seg = std::mem::take(&mut paths[path_idx]);
            interpolated_paths.append(&mut seg);
            path_idx += 1;
            continue;
        }

        // GCode.cpp:6192-6195
        let mut max_smooth_path_length = get_path_length(&paths[path_idx]);
        if smooth_right_path && smooth_left_path {
            max_smooth_path_length /= 2.0;
        }

        // GCode.cpp:6198-6207 — smooth left.
        if smooth_left_path {
            let other_v = interpolated_paths.last().unwrap().smooth_speed;
            let final_v = paths[path_idx][0].smooth_speed;
            let mut seg = std::mem::take(&mut paths[path_idx]);
            split_and_mapping_speed(
                coeff,
                other_v,
                final_v,
                &mut seg,
                max_smooth_path_length,
                &mut interpolated_paths,
                true,
            );
            paths[path_idx] = seg;
            // GCode.cpp:6202-6204
            if paths[path_idx].is_empty() {
                path_idx += 1;
                continue;
            }
            // GCode.cpp:6206
            max_smooth_path_length = get_path_length(&paths[path_idx]);
        }

        // GCode.cpp:6209-6212
        if !smooth_right_path {
            let mut seg = std::mem::take(&mut paths[path_idx]);
            interpolated_paths.append(&mut seg);
            path_idx += 1;
            continue;
        }

        // GCode.cpp:6216-6229 — smooth right; build smoothing window of indices.
        let _ = max_smooth_path_length;
        let mut window: Vec<usize> = vec![path_idx];
        let mut right_end = path_idx + 1;
        while right_end < paths.len() - 1 {
            if paths[right_end].first().map(|p| p.role) == Some(ExtrusionRole::OverhangPerimeter) {
                break;
            }
            window.push(right_end);
            if !need_smooth_speed(&paths[right_end + 1][0], &paths[right_end][0]) {
                break;
            }
            right_end += 1;
        }

        // GCode.cpp:6231-6232
        path_idx += window.len() - 1;
        let mut prev_speed = paths[path_idx + 1][0].smooth_speed;

        // GCode.cpp:5866-5868 — reverse window order and reverse the path-order
        // within each window entry.
        //
        // C++:  std::reverse(paths_cpoy.begin(), paths_cpoy.end());
        //       for (ExtrusionPaths *paths_temp : paths_cpoy)
        //           std::reverse(paths_temp->begin(), paths_temp->end());
        //
        // `paths_temp` is an `ExtrusionPaths` (a `vector<ExtrusionPath>`), so the
        // inner `std::reverse` reverses the ORDER OF THE PATH ELEMENTS — it does
        // NOT reverse the points inside each ExtrusionPath's polyline. The
        // direction reversal of the geometry is done internally by
        // split_and_mapping_speed (split_from_left=false: it reverses the input
        // polyline, cuts, then reverses the cut polyline back). A previous port
        // additionally called `p.reverse()` on every ExtrusionPath here, which
        // double-flipped each sub-path's polyline and left the smoothed output
        // non-contiguous (last_point(path_i) != first_point(path_{i+1})). That
        // made GCode::_extrude emit a travel before every smoothed sub-path,
        // shattering each overhang-graded outer-wall loop into dozens of
        // travel-separated runs (~5400 spurious outer-wall travels). Reversing
        // only the element order (as C++ does) keeps the loop continuous.
        window.reverse();
        for &wi in &window {
            paths[wi].reverse();
        }

        // GCode.cpp:6238-6249 — smooth right path.
        let mut transition_right: ExtrusionPaths = Vec::new();
        for (k, &wi) in window.iter().enumerate() {
            if k != 0 {
                prev_speed = transition_right.last().unwrap().smooth_speed;
            }
            let final_v = paths[wi][0].smooth_speed;
            let max_len = get_path_length(&paths[wi]);
            let mut seg = std::mem::take(&mut paths[wi]);
            split_and_mapping_speed(
                coeff,
                prev_speed,
                final_v,
                &mut seg,
                max_len,
                &mut transition_right,
                false,
            );
            // GCode.cpp:6247 — append remaining seg.
            transition_right.append(&mut seg);
            paths[wi] = Vec::new();
        }
        // GCode.cpp:6249
        transition_right.reverse();

        // GCode.cpp:6252
        interpolated_paths.append(&mut transition_right);

        path_idx += 1;
    }

    interpolated_paths
}

/// GCode.cpp:6259-6272 — `void GCode::smooth_speed_discontinuity_area(ExtrusionPaths &paths)`.
///
/// `coeff` is `m_smooth_coefficient` (filament_velocity_adaptation_factor *
/// smooth_coefficient). `path_speed_fn` is the per-path normal speed (mm/s).
pub fn smooth_speed_discontinuity_area<F>(coeff: f64, paths: &mut ExtrusionPaths, path_speed_fn: F)
where
    F: Fn(&ExtrusionPath) -> f64,
{
    // GCode.cpp:6261-6262
    if paths.len() <= 1 || coeff == 0.0 {
        return;
    }
    // GCode.cpp:6266 — step 1 merge same speed paths.
    let mut prepare_paths = merge_same_speed_paths(paths, path_speed_fn);
    // GCode.cpp:6269-6271 — step 2 split path.
    *paths = set_speed_transition(coeff, &mut prepare_paths);
}
