//! The engine's two routines on `ClipperLib_Z`, `clip_extrusion_port` and
//! `detect_floating_port` of `src/clipper_z.rs` (they run on the Rust `clipper_lib_z`),
//! against `cz_clip_extrusion` and `cz_detect_floating` of `clipper-z-sys` (the C++ on the
//! library): the same paths in the same order, each with the same points from the same start,
//! the same Z in every point, and for `detect_floating` the same number of difference paths.
//!
//! The inputs have the shape of the real ones. `clip_extrusion`: one open polyline whose Z is
//! the extrusion width at each point (constant, or varying as Arachne's do; now and then a
//! zero) against closed polygons with Z = 0, as an intersection and as a difference.
//! `detect_floating`: one open polyline whose Z is the index of the point, against closed
//! polygons whose Z go on counting. The polylines are walks, loops closed onto their start
//! (a perimeter), lines along the edges of the polygons and through their vertices; the
//! polygons those of `clipper_z_native_parity.rs` (its generators are copied here) and rings
//! beside the loop, so that the two cross many times; coordinates from tens to 2.6e7, and in
//! one test up to 1e9, where the float expressions of `clip_extrusion` round.
//!
//! The C++ runs under the guard of `cz_ref_clip_extrusion_guarded` and
//! `cz_ref_detect_floating_guarded`, which call `cz_clip_extrusion` and `cz_detect_floating`:
//! the library dies on some inputs (see `clipper_z_native_parity.rs`); those are counted and
//! not compared. `CLIPPER_Z_ENGINE_PARITY_LOG=<file>` writes the counts and every failing
//! input to that file.

use slicer::clipper_lib_z::{IntPoint, Path, Paths};
use slicer::clipper_z::{clip_extrusion_port, detect_floating_port, ClipType};
use slicer::clipper_z_utils::{ZPath, ZPaths};

const LIMIT: i64 = 1_000_000_000;

fn xorshift(s: &mut u64) -> u64 {
    *s ^= *s << 13;
    *s ^= *s >> 7;
    *s ^= *s << 17;
    *s
}

/// The seed of case `index` of a test: splitmix64 of the two, never zero.
fn case_seed(test: u64, index: u64) -> u64 {
    let mut z = test.wrapping_mul(0x9E37_79B9_7F4A_7C15).wrapping_add(index).wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    (z ^ (z >> 31)) | 1
}

/// A value in `-m..=m`.
fn r(m: i64, s: &mut u64) -> i64 {
    (xorshift(s) % (2 * m as u64 + 1)) as i64 - m
}

/// A value in `0..n`.
fn below(n: u64, s: &mut u64) -> u64 {
    xorshift(s) % n
}

fn pt(x: i64, y: i64, grid: i64) -> IntPoint {
    let snap = |v: i64| if grid > 1 { v.div_euclid(grid) * grid } else { v };
    IntPoint::new(snap(x).clamp(-LIMIT, LIMIT) as i32, snap(y).clamp(-LIMIT, LIMIT) as i32)
}

/// Zigzag with `teeth` downward points, all at Y = `base` (Clipper's Y grows downwards: these
/// are the local minima), closed over the top. `flat` gives every other tooth a horizontal
/// bottom, `ragged` varies the height of the peaks between the teeth.
fn comb(x0: i64, base: i64, teeth: i64, w: i64, h: i64, flat: bool, ragged: bool, s: &mut u64) -> Path {
    let mut p = Path::new();
    p.push(pt(x0, base - h, 1));
    for i in 0..teeth {
        let x = x0 + i * w;
        if flat && i % 2 == 1 && w >= 4 {
            p.push(pt(x + w / 4, base, 1));
            p.push(pt(x + w / 2 + w / 4, base, 1));
        } else {
            p.push(pt(x + w / 2, base, 1));
        }
        let peak = if ragged { h - below(h.max(2) as u64 / 2 + 1, s) as i64 } else { h };
        p.push(pt(x + w, base - peak.max(1), 1));
    }
    p.push(pt(x0 + teeth * w, base - 2 * h - 1, 1));
    p.push(pt(x0, base - 2 * h - 1, 1));
    p
}

fn polygon(s: &mut u64, scale: i64, grid: i64, earlier: &[Path]) -> Path {
    let mut poly = Path::new();
    match below(12, s) {
        // Random points: mostly self-intersecting.
        0 | 1 => {
            for _ in 0..3 + below(7, s) {
                poly.push(pt(r(scale, s), r(scale, s), grid));
            }
        }
        // Convex.
        2 => {
            let (cx, cy) = (r(scale / 2, s), r(scale / 2, s));
            let radius = (1 + below(scale as u64 / 2 + 1, s)) as f64;
            let n = 3 + below(9, s);
            let phase = below(628, s) as f64 / 100.0;
            for i in 0..n {
                let a = phase + i as f64 * std::f64::consts::TAU / n as f64;
                poly.push(pt(cx + (radius * a.cos()) as i64, cy + (radius * a.sin()) as i64, grid));
            }
        }
        // A star: concave, and with a low inner radius self-touching.
        3 => {
            let (cx, cy) = (r(scale / 2, s), r(scale / 2, s));
            let outer = (1 + below(scale as u64 / 2 + 1, s)) as f64;
            let inner = outer * (below(90, s) as f64 / 100.0);
            let n = 2 * (3 + below(5, s));
            let phase = below(628, s) as f64 / 100.0;
            for i in 0..n {
                let a = phase + i as f64 * std::f64::consts::TAU / n as f64;
                let rad = if i % 2 == 0 { outer } else { inner };
                poly.push(pt(cx + (rad * a.cos()) as i64, cy + (rad * a.sin()) as i64, grid));
            }
        }
        // Axis-aligned rectangle: horizontal edges, shared and overlapping on the grid.
        4 | 5 => {
            let (x0, y0) = (r(scale, s), r(scale, s));
            let (w, h) = (1 + below(scale as u64 + 1, s) as i64, 1 + below(scale as u64 + 1, s) as i64);
            let g = grid.max(1);
            poly.extend([pt(x0, y0, grid), pt(x0 + w + g, y0, grid), pt(x0 + w + g, y0 + h + g, grid), pt(x0, y0 + h + g, grid)]);
        }
        // Rectilinear walk: runs of horizontals and verticals, spikes, overlaps.
        6 => {
            let (mut x, mut y) = (r(scale / 2, s), r(scale / 2, s));
            let mut horizontal = below(2, s) == 0;
            for _ in 0..4 + below(8, s) {
                poly.push(pt(x, y, grid));
                if horizontal {
                    x += r(scale / 2, s);
                } else {
                    y += r(scale / 2, s);
                }
                if below(5, s) != 0 {
                    horizontal = !horizontal;
                }
            }
        }
        // A small comb: several local minima at one Y, on the grid's rows.
        7 => {
            let teeth = 2 + below(6, s) as i64;
            let w = (scale / (teeth + 1)).max(2);
            let h = (1 + below(scale as u64, s) as i64).min(scale);
            let base = pt(0, r(scale / 2, s), grid).y as i64;
            poly = comb(r(scale / 2, s), base, teeth, w, h, below(2, s) == 0, below(2, s) == 0, s);
        }
        // A copy of an earlier polygon: in place, moved by one unit, by one grid step, or by
        // its own width or height so that the two touch.
        8 | 9 if !earlier.is_empty() => {
            let src = &earlier[below(earlier.len() as u64, s) as usize];
            let (min_x, max_x) = (src.iter().map(|p| p.x).min().unwrap() as i64, src.iter().map(|p| p.x).max().unwrap() as i64);
            let (min_y, max_y) = (src.iter().map(|p| p.y).min().unwrap() as i64, src.iter().map(|p| p.y).max().unwrap() as i64);
            let (dx, dy) = match below(8, s) {
                0 => (0, 0),
                1 => (1, 0),
                2 => (0, 1),
                3 => (grid, 0),
                4 => (0, grid),
                5 => (max_x - min_x, 0),
                6 => (0, max_y - min_y),
                _ => ((max_x - min_x) / 2, (max_y - min_y) / 2),
            };
            for p in src {
                poly.push(pt(p.x as i64 + dx, p.y as i64 + dy, 1));
            }
        }
        // Nearly collinear: thin slivers and their rounding.
        10 => {
            let (x0, y0) = (r(scale, s), r(scale, s));
            let (dx, dy) = (r(scale / 4 + 1, s), r(scale / 4 + 1, s));
            for i in 0..3 + below(4, s) as i64 {
                poly.push(pt(x0 + i * dx + r(1, s), y0 + i * dy + r(1, s), 1));
            }
        }
        // Random points off the grid.
        _ => {
            for _ in 0..3 + below(5, s) {
                poly.push(pt(r(scale, s), r(scale, s), 1));
            }
        }
    }
    // Duplicate vertices, vertices in the middle of an edge, another start vertex, the
    // other direction.
    if below(4, s) == 0 && !poly.is_empty() {
        let i = below(poly.len() as u64, s) as usize;
        poly.insert(i, poly[i]);
    }
    if below(4, s) == 0 && poly.len() >= 2 {
        let i = below(poly.len() as u64, s) as usize;
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        let mid = IntPoint::new(((a.x as i64 + b.x as i64) / 2) as i32, ((a.y as i64 + b.y as i64) / 2) as i32);
        poly.insert(i + 1, mid);
    }
    if below(3, s) == 0 && !poly.is_empty() {
        let k = below(poly.len() as u64, s) as usize;
        poly.rotate_left(k);
    }
    if below(2, s) == 0 {
        poly.reverse();
    }
    poly
}

/// Removes a point that repeats its predecessor's place with another Z when the point
/// before the two or the point after them is at the same Y.
///
/// In this build such a pair is an edge of no length, which is horizontal, and
/// `GetHorzDirection` (clipper.cpp:2187-2200) gives it the direction right to left whatever
/// its neighbours do. Next to a real horizontal that runs left to right it is the turn of
/// `drop_horizontal_reversals`: `Clipper::ProcessHorizontal` dereferences the iterator it
/// never set, and the library dies (found with `nested case 622 of test 0x73a1`, a closed
/// polygon under PreserveCollinear, which keeps such a point). Whether the neighbour runs left
/// to right depends on the side the bound climbs it from, so every such pair goes. A pair
/// between two edges that are not horizontal stays.
fn drop_zero_length_beside_horizontal(path: &mut Path, closed: bool) {
    loop {
        let n = path.len();
        let mut remove = None;
        for i in 0..n {
            let j = i + 1;
            if j >= n && !closed {
                break;
            }
            let (a, b) = (path[i], path[j % n]);
            if !(a.x == b.x && a.y == b.y && a.z != b.z) || n < 3 {
                continue;
            }
            let before = if i > 0 { Some(path[i - 1]) } else if closed { Some(path[n - 1]) } else { None };
            let after = if j + 1 < n { Some(path[j + 1]) } else if closed { Some(path[(j + 1) % n]) } else { None };
            if before.map_or(false, |q| q.y == a.y) || after.map_or(false, |q| q.y == a.y) {
                remove = Some(j % n);
                break;
            }
        }
        match remove {
            Some(k) => {
                path.remove(k);
            }
            None => break,
        }
    }
}

/// An open path: random points, a walk on the grid, a horizontal or vertical line through
/// everything, or an earlier polygon taken as a polyline (so that it runs along its edges).
fn polyline(s: &mut u64, scale: i64, grid: i64, earlier: &[Path]) -> Path {
    let mut line = Path::new();
    match below(8, s) {
        0 | 1 => {
            for _ in 0..2 + below(7, s) {
                line.push(pt(r(scale, s), r(scale, s), grid));
            }
        }
        2 => {
            for _ in 0..2 + below(5, s) {
                line.push(pt(r(scale, s), r(scale, s), 1));
            }
        }
        3 => {
            let y = r(scale, s);
            line.extend([pt(-scale, y, grid), pt(scale, y, grid)]);
            if below(2, s) == 0 {
                line.insert(1, pt(r(scale, s), y, grid));
            }
        }
        4 => {
            let x = r(scale, s);
            line.extend([pt(x, -scale, grid), pt(x, scale, grid)]);
        }
        5 | 6 if !earlier.is_empty() => {
            line = earlier[below(earlier.len() as u64, s) as usize].clone();
            if below(2, s) == 0 && !line.is_empty() {
                // Closed back onto its start.
                line.push(line[0]);
            }
        }
        _ => {
            let (mut x, mut y) = (r(scale / 2, s), r(scale / 2, s));
            let mut horizontal = below(2, s) == 0;
            for _ in 0..2 + below(8, s) {
                line.push(pt(x, y, grid));
                if horizontal {
                    x += r(scale / 2, s);
                } else {
                    y += r(scale / 2, s);
                }
                if below(4, s) != 0 {
                    horizontal = !horizontal;
                }
            }
        }
    }
    if below(5, s) == 0 && !line.is_empty() {
        let i = below(line.len() as u64, s) as usize;
        line.insert(i, line[i]);
    }
    if below(2, s) == 0 {
        line.reverse();
    }
    drop_horizontal_reversals(&mut line);
    line
}

/// Removes the points at which an open path turns back along a horizontal line.
///
/// The library cannot take such a path: `Clipper::ProcessHorizontal` (clipper.cpp:2227-2246)
/// sets `maxIt` or `maxRit`, whichever the direction of the first horizontal of a run needs,
/// and leaves the other default-constructed; when the run turns round (only an open path can
/// do that: `AddPath` removes such a spike from a closed one) it dereferences that null
/// iterator as soon as `m_Maxima` is not empty, and the process dies. There is nothing to
/// compare the port with on these inputs.
fn drop_horizontal_reversals(line: &mut Path) {
    loop {
        // The points that differ from their predecessor: AddPath drops the others.
        let mut distinct: Vec<usize> = Vec::new();
        for i in 0..line.len() {
            if distinct.last().map_or(true, |&l| line[l] != line[i]) {
                distinct.push(i);
            }
        }
        let turn = distinct.windows(3).find(|w| {
            let (a, b, c) = (line[w[0]], line[w[1]], line[w[2]]);
            a.y == b.y && b.y == c.y && (b.x > a.x) != (c.x > b.x)
        });
        match turn {
            Some(w) => {
                line.remove(w[1]);
            }
            None => break,
        }
    }
}


/// The coordinates of real inputs: up to 2.6e7 (260 mm). `large` gives coordinates no bed
/// has, up to 1e9: there the squares of the callback and of the pass over the points left at
/// zero are not exact in a double, and the order of a sum and the fusing of a product show.
fn real_scale_and_grid(s: &mut u64, large: bool) -> (i64, i64) {
    let scale = if large {
        [60_000_000i64, 250_000_000, 450_000_000][below(3, s) as usize]
    } else {
        [40i64, 2_000, 400_000, 3_000_000, 12_000_000][below(5, s) as usize]
    };
    let grid = match below(4, s) {
        0 | 1 => 1,
        2 => (scale / 8).max(1),
        _ => (scale / 64).max(1),
    };
    (scale, grid)
}

/// A loop around (cx, cy), closed onto its first point: a perimeter.
fn loop_path(s: &mut u64, cx: i64, cy: i64, radius: i64, grid: i64) -> Path {
    let n = 5 + below(40, s);
    let phase = below(628, s) as f64 / 100.0;
    let wobble = below(3, s) == 0;
    let mut p = Path::new();
    for i in 0..n {
        let a = phase + i as f64 * std::f64::consts::TAU / n as f64;
        let rad = if wobble { radius as f64 * (0.7 + below(60, s) as f64 / 100.0) } else { radius as f64 };
        p.push(pt(cx + (rad * a.cos()) as i64, cy + (rad * a.sin()) as i64, grid));
    }
    p.push(p[0]);
    p
}

/// The subject (points without Z yet) and the clips of one case.
fn geometry(s: &mut u64, large: bool) -> (Path, Paths, i64) {
    let (scale, grid) = real_scale_and_grid(s, large);
    let mut clip = Paths::new();
    let subject;
    match below(4, s) {
        // A perimeter and rings beside it: the same loop moved or of another radius.
        0 | 1 => {
            let (cx, cy) = (r(scale / 4, s), r(scale / 4, s));
            let radius = scale / 4 + below(scale as u64 / 2 + 1, s) as i64;
            subject = loop_path(s, cx, cy, radius, grid);
            for _ in 0..1 + below(3, s) {
                let mut ring = match below(3, s) {
                    0 => subject[..subject.len() - 1].iter().map(|p| pt(p.x as i64 + r(radius / 3 + 1, s), p.y as i64 + r(radius / 3 + 1, s), 1)).collect(),
                    1 => {
                        let (dx, dy, percent) = (r(radius / 2 + 1, s), r(radius / 2 + 1, s), 50 + below(100, s) as i64);
                        loop_path(s, cx + dx, cy + dy, radius * percent / 100 + 1, grid)
                    }
                    _ => polygon(s, scale, grid, &clip),
                };
                if ring.len() > 1 && ring[0] == ring[ring.len() - 1] {
                    ring.pop();
                }
                clip.push(ring);
            }
        }
        // Any polygons, and a polyline over them: a walk, a line through everything, or a
        // polygon taken as a polyline (it runs along the edges and through the vertices).
        _ => {
            for _ in 0..1 + below(3, s) {
                let p = polygon(s, scale, grid, &clip);
                clip.push(p);
            }
            subject = polyline(s, scale, grid, &clip);
        }
    }
    (subject, clip, scale)
}

fn to_zpath(p: &Path) -> ZPath {
    p.iter().map(|q| (q.x as i64, q.y as i64, q.z as i64)).collect()
}

fn flat(paths: &ZPaths) -> (Vec<i32>, Vec<i32>) {
    (paths.iter().flatten().flat_map(|p| [p.0 as i32, p.1 as i32, p.2 as i32]).collect(), paths.iter().map(|p| p.len() as i32).collect())
}

unsafe fn take_paths(out: clipper_z_sys::CzZPaths) -> ZPaths {
    let mut paths = ZPaths::new();
    let mut k = 0usize;
    for i in 0..out.num_paths as usize {
        let len = *out.path_lens.add(i) as usize;
        let mut p = ZPath::with_capacity(len);
        for _ in 0..len {
            p.push((*out.coords.add(3 * k) as i64, *out.coords.add(3 * k + 1) as i64, *out.coords.add(3 * k + 2) as i64));
            k += 1;
        }
        paths.push(p);
    }
    assert_eq!(k, out.total_points as usize);
    clipper_z_sys::cz_free_zpaths(out);
    paths
}

fn log(text: &str) {
    if let Some(path) = std::env::var_os("CLIPPER_Z_ENGINE_PARITY_LOG") {
        use std::io::Write as _;
        let mut f = std::fs::OpenOptions::new().create(true).append(true).open(path).expect("the log file opens");
        writeln!(f, "{text}").unwrap();
    }
}

/// `cz_clip_extrusion`, as `clip_extrusion_shim` calls it. `None` when the library died.
fn library_clip_extrusion(subject: &ZPath, clip: &ZPaths, clip_type: ClipType) -> Option<ZPaths> {
    let (subject_flat, _) = flat(&vec![subject.clone()]);
    let (clip_flat, clip_lens) = flat(clip);
    let mut died = 0i32;
    let out = unsafe {
        clipper_z_sys::cz_ref_clip_extrusion_guarded(
            subject_flat.as_ptr(),
            subject.len() as i32,
            if clip_flat.is_empty() { std::ptr::null() } else { clip_flat.as_ptr() },
            if clip_lens.is_empty() { std::ptr::null() } else { clip_lens.as_ptr() },
            clip.len() as i32,
            clip_type as i32,
            &mut died,
        )
    };
    if died != 0 {
        return None;
    }
    Some(unsafe { take_paths(out) })
}

/// `cz_detect_floating`, as `detect_floating_shim` calls it. `None` when the library died.
fn library_detect_floating(subject: &ZPath, clip: &ZPaths, subject_idx_range: i32) -> Option<(ZPaths, usize)> {
    let (subject_flat, _) = flat(&vec![subject.clone()]);
    let (clip_flat, clip_lens) = flat(clip);
    let (mut died, mut num_diff) = (0i32, 0i32);
    let out = unsafe {
        clipper_z_sys::cz_ref_detect_floating_guarded(
            subject_flat.as_ptr(),
            subject.len() as i32,
            if clip_flat.is_empty() { std::ptr::null() } else { clip_flat.as_ptr() },
            if clip_lens.is_empty() { std::ptr::null() } else { clip_lens.as_ptr() },
            clip.len() as i32,
            subject_idx_range,
            &mut num_diff,
            &mut died,
        )
    };
    if died != 0 {
        return None;
    }
    Some((unsafe { take_paths(out) }, num_diff.max(0) as usize))
}

/// The subject of `clip_extrusion`: Z is the width of the extrusion at the point.
fn extrusion_case(test: u64, index: u64, large: bool) -> (ZPath, ZPaths) {
    let mut s = case_seed(test, index);
    let s = &mut s;
    let (mut subject, clip, _) = geometry(s, large);
    let base = 20_000 + below(60_000, s) as i32;
    let mode = below(10, s);
    for p in subject.iter_mut() {
        p.z = match mode {
            // One width for the whole path.
            0..=3 => base,
            // A width per point.
            4..=8 => base + r(base as i64 / 2, s) as i32,
            // A zero now and then: the callback's `<= 0` arms.
            _ => if below(4, s) == 0 { 0 } else { base + r(1000, s) as i32 },
        };
    }
    loop {
        let before = subject.len();
        drop_zero_length_beside_horizontal(&mut subject, false);
        drop_horizontal_reversals(&mut subject);
        if subject.len() == before {
            break;
        }
    }
    (to_zpath(&subject), clip.iter().map(to_zpath).collect())
}

fn run_clip_extrusion(test: u64, cases: u64, name: &str, large: bool) {
    let (mut died, mut nonempty, mut points, mut compared) = (0usize, 0usize, 0usize, 0usize);
    // Output points whose Z is the width of no subject point: interpolated by the callback or
    // by the pass over the points left at zero.
    let mut interpolated = 0usize;
    let mut failures: Vec<String> = Vec::new();
    for index in 0..cases {
        let (subject, clip) = extrusion_case(test, index, large);
        for clip_type in [ClipType::Intersection, ClipType::Difference] {
            let Some(want) = library_clip_extrusion(&subject, &clip, clip_type) else {
                died += 1;
                continue;
            };
            compared += 1;
            nonempty += !want.is_empty() as usize;
            for p in want.iter().flatten() {
                points += 1;
                interpolated += !subject.iter().any(|q| q.2 == p.2) as usize;
            }
            let got = std::panic::catch_unwind(|| clip_extrusion_port(&subject, &clip, clip_type));
            if got.as_ref().ok() != Some(&want) {
                failures.push(format!(
                    "case {index} of test {test:#x}, {clip_type:?}\n  subject: {subject:?}\n  clip: {clip:?}\n  library: {want:?}\n  port:    {got:?}"
                ));
            }
        }
    }
    log(&format!(
        "{name}: {cases} cases, {compared} comparisons, {nonempty} with a result ({points} points, {interpolated} with an interpolated Z), {died} where the library died, {} differ",
        failures.len()
    ));
    for failure in failures.iter().take(10) {
        log(failure);
    }
    assert!(failures.is_empty(), "{name}: {} of {compared} comparisons differ; the first:\n{}", failures.len(), failures[0]);
    assert!(nonempty * 3 > compared, "{name}: only {nonempty} of {compared} comparisons have a result");
    assert!(interpolated * 20 > points, "{name}: only {interpolated} of {points} points have an interpolated Z");
    assert!(died * 50 < compared, "{name}: the library died on {died} inputs");
}

#[test]
fn clip_extrusion_a() {
    run_clip_extrusion(0x81A1, 12_000, "clip_extrusion_a", false);
}

#[test]
fn clip_extrusion_b() {
    run_clip_extrusion(0x81B2, 12_000, "clip_extrusion_b", false);
}

#[test]
fn clip_extrusion_c() {
    run_clip_extrusion(0x81C3, 12_000, "clip_extrusion_c", false);
}

/// Coordinates up to 1e9, where the float expressions round.
#[test]
fn clip_extrusion_large_coordinates() {
    run_clip_extrusion(0x81D4, 12_000, "clip_extrusion_large_coordinates", true);
}

/// The inputs of `detect_floating_line`: Z counts the points, the subject's first.
fn floating_case(test: u64, index: u64) -> (ZPath, ZPaths, i32) {
    let mut s = case_seed(test, index);
    let s = &mut s;
    let (mut subject, mut clip, _) = geometry(s, false);
    loop {
        // Every point has its own Z: a repeated point is an edge of no length.
        for (i, p) in subject.iter_mut().enumerate() {
            p.z = i as i32;
        }
        let before = subject.len();
        drop_zero_length_beside_horizontal(&mut subject, false);
        drop_horizontal_reversals(&mut subject);
        if subject.len() == before {
            break;
        }
    }
    let mut idx = subject.len() as i32;
    for p in clip.iter_mut().flatten() {
        p.z = idx;
        idx += 1;
    }
    (to_zpath(&subject), clip.iter().map(to_zpath).collect(), subject.len() as i32)
}

fn run_detect_floating(test: u64, cases: u64, name: &str) {
    let (mut died, mut nonempty, mut points, mut hashed, mut compared) = (0usize, 0usize, 0usize, 0usize, 0usize);
    let mut failures: Vec<String> = Vec::new();
    for index in 0..cases {
        let (subject, clip, subject_idx_range) = floating_case(test, index);
        if subject.is_empty() {
            continue;
        }
        let Some(want) = library_detect_floating(&subject, &clip, subject_idx_range) else {
            died += 1;
            continue;
        };
        compared += 1;
        nonempty += (want.0.len() > 1) as usize;
        for p in want.0.iter().flatten() {
            points += 1;
            // The callback's hash is negative.
            hashed += (p.2 < 0) as usize;
        }
        let got = std::panic::catch_unwind(|| detect_floating_port(&subject, &clip, subject_idx_range));
        if got.as_ref().ok() != Some(&want) {
            failures.push(format!(
                "case {index} of test {test:#x}, subject_idx_range {subject_idx_range}\n  subject: {subject:?}\n  clip: {clip:?}\n  library: {want:?}\n  port:    {got:?}"
            ));
        }
    }
    log(&format!(
        "{name}: {cases} cases, {compared} compared, {nonempty} with more than one path ({points} points, {hashed} with the callback's hash), {died} where the library died, {} differ",
        failures.len()
    ));
    for failure in failures.iter().take(10) {
        log(failure);
    }
    assert!(failures.is_empty(), "{name}: {} of {compared} cases differ; the first:\n{}", failures.len(), failures[0]);
    assert!(nonempty * 3 > compared, "{name}: only {nonempty} of {compared} cases have more than one path");
    assert!(hashed * 20 > points, "{name}: only {hashed} of {points} points have the callback's hash");
    assert!(died * 50 < compared.max(1), "{name}: the library died on {died} inputs");
}

#[test]
fn detect_floating_a() {
    run_detect_floating(0x82A1, 15_000, "detect_floating_a");
}

#[test]
fn detect_floating_b() {
    run_detect_floating(0x82B2, 15_000, "detect_floating_b");
}

/// The smallest inputs, and the ones the wrappers treat on their own.
#[test]
fn degenerate_inputs() {
    let square: ZPath = vec![(0, 0, 0), (100, 0, 0), (100, 100, 0), (0, 100, 0)];
    let subjects: Vec<ZPath> = vec![
        vec![],
        vec![(50, 50, 40)],
        vec![(-50, 50, 40), (150, 50, 40)],
        vec![(-50, 50, 40), (50, 50, 40), (150, 50, 40)],
        vec![(-50, 50, 20), (50, 50, 40), (150, 50, 60)],
        vec![(-50, 0, 20), (50, 0, 40), (150, 0, 60)],
        vec![(0, 0, 20), (100, 100, 40), (200, 100, 60)],
        vec![(-50, -50, 20), (0, 0, 30), (50, 50, 40), (100, 100, 50), (150, 150, 60)],
        vec![(500, 500, 40), (600, 500, 40), (700, 600, 40)],
    ];
    for subject in &subjects {
        for clip in [vec![], vec![square.clone()], vec![square.clone(), vec![(50, 50, 0), (150, 50, 0), (150, 150, 0), (50, 150, 0)]]] {
            for clip_type in [ClipType::Intersection, ClipType::Difference, ClipType::Union, ClipType::Xor] {
                let want = if subject.is_empty() { Some(ZPaths::new()) } else { library_clip_extrusion(subject, &clip, clip_type) };
                assert_eq!(Some(clip_extrusion_port(subject, &clip, clip_type)), want, "{subject:?} {clip:?} {clip_type:?}");
            }
            // detect_floating: Z are the indices.
            let indexed: ZPath = subject.iter().enumerate().map(|(i, p)| (p.0, p.1, i as i64)).collect();
            let mut idx = indexed.len() as i64;
            let clip_indexed: ZPaths = clip
                .iter()
                .map(|path| {
                    path.iter()
                        .map(|p| {
                            idx += 1;
                            (p.0, p.1, idx - 1)
                        })
                        .collect()
                })
                .collect();
            let want = if indexed.is_empty() { Some((ZPaths::new(), 0)) } else { library_detect_floating(&indexed, &clip_indexed, indexed.len() as i32) };
            assert_eq!(Some(detect_floating_port(&indexed, &clip_indexed, indexed.len() as i32)), want, "{indexed:?} {clip_indexed:?}");
        }
    }
}
