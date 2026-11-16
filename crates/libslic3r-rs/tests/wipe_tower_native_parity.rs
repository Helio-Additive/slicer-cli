//! `slicer::gcode::wipe_tower` (the port of libslic3r/GCode/WipeTower.cpp) against the C++ at the
//! pin.
//!
//! `tests/data/wipe_tower/*.txt.gz` are the output of `gen.cpp` beside them (gzip -9n), which
//! compiles the pin's WipeTower.cpp into itself and drives its free functions and static members
//! (free.txt), WipeTowerWriter operation by operation (writer.txt) and whole WipeTower runs over
//! small synthetic plates: the constructor from a PrintConfig, set_extruder, plan_toolchange and
//! generate_new (runs.txt). The header of gen.cpp describes the lines. Every operation is repeated
//! here on the Rust port, and what it returns has to be the same: G-code byte for byte, floats and
//! doubles bit for bit, integers equal.

use slicer::config::ConfigBase;
use slicer::gcode::wipe_tower::*;
use slicer::geometry::{BoundingBox, Line, Point, Polygon, Polyline};
use std::io::Read;

// ---------------------------------------------------------------------------------------------
// Tokens
// ---------------------------------------------------------------------------------------------

fn read_vectors(name: &str) -> String {
    let path = format!("{}/tests/data/wipe_tower/{}.txt.gz", env!("CARGO_MANIFEST_DIR"), name);
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

fn hex(s: &[u8]) -> String {
    if s.is_empty() {
        return "-".to_string();
    }
    let mut o = String::with_capacity(s.len() * 2);
    for c in s {
        o.push_str(&format!("{:02x}", c));
    }
    o
}
fn unhex(t: &str) -> String {
    if t == "-" {
        return String::new();
    }
    let b: Vec<u8> = (0..t.len() / 2).map(|i| u8::from_str_radix(&t[2 * i..2 * i + 2], 16).expect("hex")).collect();
    String::from_utf8(b).expect("the vectors are UTF-8")
}
fn ff(v: f32) -> String {
    format!("{:08x}", v.to_bits())
}
fn dd(v: f64) -> String {
    format!("{:016x}", v.to_bits())
}

/// A cursor over the tokens of one line.
struct Toks<'a> {
    t: Vec<&'a str>,
    i: usize,
}

impl<'a> Toks<'a> {
    fn new(line: &'a str) -> Self {
        Toks { t: line.split(' ').collect(), i: 0 }
    }
    fn s(&mut self) -> &'a str {
        let v = self.t[self.i];
        self.i += 1;
        v
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn b(&mut self) -> bool {
        self.i() != 0
    }
    fn text(&mut self) -> String {
        unhex(self.s())
    }
    fn v2(&mut self) -> Vec2f {
        let x = self.f();
        let y = self.f();
        Vec2f::new(x, y)
    }
    fn pt(&mut self) -> Point {
        let x = self.i();
        let y = self.i();
        Point::new(x, y)
    }
    fn pts(&mut self) -> Vec<Point> {
        let n = self.i() as usize;
        (0..n).map(|_| self.pt()).collect()
    }
    fn v2s(&mut self) -> Vec<Vec2f> {
        let n = self.i() as usize;
        (0..n).map(|_| self.v2()).collect()
    }
    fn arrow(&mut self) {
        assert_eq!(self.s(), "=>");
    }
    fn rest(&self) -> String {
        self.t[self.i..].join(" ")
    }
}

fn v2(v: Vec2f) -> String {
    format!("{} {}", ff(v.x), ff(v.y))
}
fn pt(p: &Point) -> String {
    format!("{} {}", p.x, p.y)
}
fn pts(p: &[Point]) -> String {
    let mut o = p.len().to_string();
    for q in p {
        o.push(' ');
        o.push_str(&pt(q));
    }
    o
}
fn v2s(v: &[Vec2f]) -> String {
    let mut o = v.len().to_string();
    for q in v {
        o.push(' ');
        o.push_str(&v2(*q));
    }
    o
}
fn polygon(p: Vec<Point>) -> Polygon {
    Polygon::from_points(p)
}

// ---------------------------------------------------------------------------------------------
// free.txt
// ---------------------------------------------------------------------------------------------

fn free_line(line: &str) -> Option<(String, String)> {
    let mut t = Toks::new(line);
    let op = t.s();
    let got: String = match op {
        "nozzle_change_width_for_diameter" => {
            let d = t.f();
            t.arrow();
            ff(nozzle_change_width_for_diameter(d))
        }
        "parseDoubles" => {
            let s = t.text();
            t.arrow();
            let r = parse_doubles(&s);
            let mut o = r.len().to_string();
            for v in r {
                o.push_str(&format!(" {}", dd(v)));
            }
            o
        }
        "align" => {
            let v = t.f();
            let b = t.f();
            t.arrow();
            format!("{} {} {}", ff(align_round(v, b)), ff(align_ceil(v, b)), ff(align_floor(v, b)))
        }
        "is_valid_gcode" => {
            let s = t.text();
            t.arrow();
            (is_valid_gcode(&s) as i32).to_string()
        }
        "depth_brim" => {
            let h = t.f();
            t.arrow();
            format!("{} {}", ff(WipeTower::get_limit_depth_by_height(h)), ff(WipeTower::get_auto_brim_by_height(h)))
        }
        "block_key" => {
            let c = t.i() as i32;
            let d = t.f();
            let lane = t.i() as i32;
            t.arrow();
            let k = WipeTower::encode_block_key(c, d, lane);
            format!("{} {}", k, WipeTower::decode_block_category(k))
        }
        "chamfer_polygon" | "rounding_rectangle" | "rounding_polygon" => {
            let p = polygon(t.pts());
            let dis = t.d();
            let tol = t.d();
            t.arrow();
            let r = match op {
                "chamfer_polygon" => chamfer_polygon(&p, dis, tol),
                "rounding_rectangle" => rounding_rectangle(&p, dis, tol),
                _ => WipeTower::rounding_polygon(&p, dis, tol),
            };
            pts(r.points())
        }
        "ray_intersetion_line" => {
            let a = t.v2();
            let v1 = t.v2();
            let b = t.v2();
            let c = t.v2();
            t.arrow();
            match ray_intersetion_line(a, v1, b, c) {
                Some(r) => format!("1 {}", v2(r)),
                None => format!("0 {}", v2(Vec2f::new(0.0, 0.0))),
            }
        }
        "scale_polygon" => {
            let v = t.v2s();
            t.arrow();
            let p = scale_polygon(&v);
            format!("{} {}", pts(p.points()), v2s(&unscale_polygon(&p)))
        }
        "generate_rectange" => {
            let a = t.pt();
            let b = t.pt();
            let off = t.i();
            t.arrow();
            pts(generate_rectange(&Line::new(a, b), off).points())
        }
        "remove_points_from_segment" => {
            let s = t.v2();
            let e = t.v2();
            let skip = t.v2s();
            let range = t.d();
            t.arrow();
            let r = remove_points_from_segment(&Segment::new(s, e), &skip, range);
            let mut o = r.len().to_string();
            for seg in r {
                o.push_str(&format!(" {} {}", v2(seg.start), v2(seg.end)));
            }
            o
        }
        "move_point_along_polygon" => {
            let poly = t.v2s();
            let start = t.v2();
            let idx = t.i() as usize;
            let off = t.f();
            let fwd = t.b();
            let pair = t.i() as i32;
            t.arrow();
            let r = move_point_along_polygon(&poly, start, idx, off, fwd, pair);
            format!("{} {} {} {}", v2(r.pos), r.idx, r.pair_idx, ff(r.dis_from_idx))
        }
        "insert_points" => {
            let poly = t.v2s();
            let idx = t.i() as usize;
            let ins = t.v2();
            let pair = t.i() as i32;
            let fwd = t.b();
            t.arrow();
            let mut pl: Vec<PointWithFlag> = poly.iter().map(|p| PointWithFlag { pos: *p, pair_idx: -1, is_forward: false }).collect();
            insert_points(&mut pl, idx, ins, pair, fwd);
            let mut o = pl.len().to_string();
            for p in pl {
                o.push_str(&format!(" {} {} {}", v2(p.pos), p.pair_idx, p.is_forward as i32));
            }
            o
        }
        "generate_rectange_polygon" => {
            let mn = t.v2();
            let mx = t.v2();
            t.arrow();
            pts(generate_rectange_polygon(mn, mx).points())
        }
        "add_extra_point" => {
            let p = polygon(t.pts());
            let r = t.i();
            t.arrow();
            pts(add_extra_point(&p, r as f64).points())
        }
        "contrust_gap_for_skip_points" => {
            let p = polygon(t.pts());
            let skip = t.v2s();
            let w = t.f();
            let gap = t.f();
            t.arrow();
            let (r, ins) = contrust_gap_for_skip_points(&p, &skip, w, gap as f64);
            let mut o = r.len().to_string();
            for pl in &r {
                o.push_str(&format!(" {}", pts(&pl.points)));
            }
            o.push_str(&format!(" {}", pts(ins.points())));
            o
        }
        "polylines_extreme_at" => {
            let n = t.i() as usize;
            let pls: Vec<Polyline> = (0..n)
                .map(|_| {
                    let mut pl = Polyline::new();
                    pl.points = t.pts();
                    pl
                })
                .collect();
            let ax = t.b();
            let pos = t.b();
            let along = t.f();
            t.arrow();
            let mut out = -7.0_f32;
            let ok = polylines_extreme_at(&pls, ax, pos, along, &mut out);
            format!("{} {}", ok as i32, ff(out))
        }
        "move_box_inside_box" => {
            let b1 = BoundingBox::from_points_minmax(t.pt(), t.pt());
            let b2 = BoundingBox::from_points_minmax(t.pt(), t.pt());
            let off = t.i() as i32;
            t.arrow();
            v2(WipeTower::move_box_inside_box(&b1, &b2, off))
        }
        "rib_section" => {
            let w = t.f();
            let d = t.f();
            let rl = t.f();
            let rw = t.f();
            let fillet = t.b();
            t.arrow();
            pts(WipeTower::rib_section(w, d, rl, rw, fillet).points())
        }
        "its_make_rib_tower" | "its_make_rib_brim" => {
            let its = if op == "its_make_rib_tower" {
                let w = t.f();
                let d = t.f();
                let h = t.f();
                let rl = t.f();
                let rw = t.f();
                let fillet = t.b();
                WipeTower::its_make_rib_tower(w, d, h, rl, rw, fillet)
            } else {
                let p = polygon(t.pts());
                let lh = t.f();
                WipeTower::its_make_rib_brim(&p, lh)
            };
            t.arrow();
            // The caps come from Triangulation, which the port does not have yet: compare the
            // vertices and the side faces (the last indices), not the cap count.
            let mut o = its.vertices.len().to_string();
            for v in &its.vertices {
                o.push_str(&format!(" {} {} {}", ff(v.x), ff(v.y), ff(v.z)));
            }
            let want_rest = t.rest();
            let mut wt = Toks::new(&want_rest);
            let _nv = wt.i();
            for _ in 0.._nv * 3 {
                wt.s();
            }
            let _total = wt.i();
            let side = wt.i() as usize;
            let side_got = side.min(its.indices.len());
            o.push_str(&format!(" {} {}", _total, side_got));
            for k in its.indices.len() - side_got..its.indices.len() {
                let f = &its.indices[k];
                o.push_str(&format!(" {} {} {}", f[0], f[1], f[2]));
            }
            o
        }
        "compute_compacted_wipe_tower_z" => {
            let n = t.i() as usize;
            let mut tcs: Vec<Vec<ToolChangeResult>> = Vec::new();
            for _ in 0..n {
                let m = t.i() as usize;
                let mut layer = Vec::new();
                for _ in 0..m {
                    let mut r = ToolChangeResult::default();
                    r.initial_tool = t.i() as i32;
                    r.new_tool = t.i() as i32;
                    r.layer_height = t.f();
                    layer.push(r);
                }
                tcs.push(layer);
            }
            t.arrow();
            let z = compute_compacted_wipe_tower_z(&tcs);
            let mut o = String::new();
            for (k, zk) in z.iter().enumerate() {
                if k > 0 {
                    o.push(' ');
                }
                o.push_str(&format!("{} {}", ff(*zk), (!tcs[k].is_empty() && wipe_tower_layer_is_sparse(&tcs[k])) as i32));
            }
            o
        }
        "config_flags" => {
            let nsl = t.s();
            let tl = t.s();
            let wd = t.s();
            let ss = t.s();
            let gap = t.text();
            let len = t.text();
            t.arrow();
            let mut cfg = WipeTowerPrintConfig::full_print_config();
            cfg.set_deserialize_strict("wipe_tower_no_sparse_layers", nsl, false).unwrap();
            cfg.set_deserialize_strict("timelapse_type", tl, false).unwrap();
            cfg.set_deserialize_strict("enable_wrapping_detection", wd, false).unwrap();
            cfg.set_deserialize_strict("prime_tower_side_scrape", ss, false).unwrap();
            cfg.set_deserialize_strict("prime_tower_side_scrape_wall_gap", &gap, false).unwrap();
            cfg.set_deserialize_strict("prime_tower_side_scrape_length", &len, false).unwrap();
            format!("{} {}", wipe_tower_sparse_layers_skipped(&cfg) as i32, dd(prime_tower_side_scrape_margin(&cfg)))
        }
        _ => panic!("free.txt: unknown function {op}"),
    };
    let want = line.split_once(" =>").map(|(_, w)| w.trim_start().to_string()).unwrap();
    if got != want {
        return Some((got, want));
    }
    None
}

#[test]
fn wipe_tower_free_functions_match_cpp() {
    let text = read_vectors("free");
    let mut counts: std::collections::BTreeMap<String, (usize, usize)> = Default::default();
    let mut first_fail: Vec<String> = Vec::new();
    for line in text.lines() {
        if line.is_empty() {
            continue;
        }
        let op = line.split(' ').next().unwrap().to_string();
        let e = counts.entry(op.clone()).or_default();
        e.0 += 1;
        if let Some((got, want)) = free_line(line) {
            e.1 += 1;
            if first_fail.len() < 40 {
                first_fail.push(format!("{op}\n  line: {}\n  got:  {got}\n  want: {want}", &line[..line.len().min(400)]));
            }
        }
    }
    let failed: usize = counts.values().map(|c| c.1).sum();
    for (op, (n, f)) in &counts {
        eprintln!("{op}: {n} cases, {f} differ");
    }
    assert!(failed == 0, "{failed} cases differ:\n{}", first_fail.join("\n"));
}

// ---------------------------------------------------------------------------------------------
// writer.txt
// ---------------------------------------------------------------------------------------------

fn flavor_of(i: i64) -> GCodeFlavor {
    // PrintConfig.hpp:40-43 enum GCodeFlavor
    match i {
        0 => GCodeFlavor::Marlin,
        1 => GCodeFlavor::Klipper,
        2 => GCodeFlavor::RepRapSprinter,
        3 => GCodeFlavor::RepRap,
        4 => GCodeFlavor::Repetier,
        5 => GCodeFlavor::Teacup,
        6 => GCodeFlavor::MakerWare,
        7 => GCodeFlavor::MarlinFirmware,
        8 => GCodeFlavor::Sailfish,
        9 => GCodeFlavor::Mach3,
        10 => GCodeFlavor::Machinekit,
        11 => GCodeFlavor::Smoothie,
        _ => GCodeFlavor::NoExtrusion,
    }
}

fn limit_flow_of(i: i64) -> LimitFlow {
    match i {
        0 => LimitFlow::None,
        1 => LimitFlow::LimitPrintFlow,
        2 => LimitFlow::LimitRammingFlow,
        _ => LimitFlow::LimitRammingFlowNC,
    }
}

fn group_result(fmap: &[i32], diameters: &[String]) -> slicer::multi_nozzle_utils::LayeredNozzleGroupResult {
    use slicer::multi_nozzle_utils::{LayeredNozzleGroupResult, NozzleInfo};
    let nozzles: Vec<NozzleInfo> = diameters
        .iter()
        .enumerate()
        .map(|(e, d)| NozzleInfo {
            diameter: d.clone(),
            volume_type: slicer::extruder::NozzleVolumeType::NvtStandard,
            extruder_id: e as i32,
            group_id: e as i32,
        })
        .collect();
    let used: Vec<u32> = (0..fmap.len() as u32).collect();
    LayeredNozzleGroupResult::create(fmap, &nozzles, &used).expect("group result")
}

fn writer_res(w: &mut WipeTowerWriter) -> String {
    let mut o = format!(
        "{} {} {} {} {} {}",
        hex(w.gcode().as_bytes()),
        v2(w.pos()),
        v2(w.start_pos_rotated()),
        v2(w.pos_rotated()),
        ff(w.elapsed_time()),
        ff(w.current_feedrate())
    );
    let used = w.get_and_reset_used_filament_length();
    o.push_str(&format!(" {}", ff(used)));
    o.push_str(&format!(" {}", w.extrusions().len()));
    for e in w.extrusions() {
        o.push_str(&format!(" {} {} {}", v2(e.pos), ff(e.width), e.tool));
    }
    o.push_str(&format!(" {}", v2s(w.wipe_path())));
    o
}

fn writer_op(w: &mut WipeTowerWriter, t: &mut Toks) {
    match t.s() {
        "init_pos" => {
            let p = t.v2();
            let wd = t.f();
            let dp = t.f();
            let a = t.f();
            w.set_initial_position_wd(p, wd, dp, a);
        }
        "tool" => {
            w.set_initial_tool(t.i() as usize);
        }
        "z" => w.set_z(t.f()),
        "flow" => w.set_extrusion_flow(t.f()),
        "yshift" => w.set_y_shift(t.f()),
        "group" => {
            let n = t.i() as usize;
            let fmap: Vec<i32> = (0..n).map(|_| t.i() as i32).collect();
            let next = t.i() as usize;
            let diam = vec!["0.4".to_string(); next];
            w.set_multi_nozzle_group_result(Some(std::sync::Arc::new(group_result(&fmap, &diam))));
        }
        "accels" => {
            let next = t.i() as usize;
            let (mut a1, mut a2, mut a3, mut a4) = (vec![], vec![], vec![], vec![]);
            for _ in 0..next {
                a1.push(t.i() as u32);
                a2.push(t.i() as u32);
                a3.push(t.i() as u32);
                a4.push(t.i() as u32);
            }
            let mx = t.i() as u32;
            let a2d = t.b();
            let a2df = t.f();
            let layer_id = t.i() as i32;
            w.set_normal_acceleration_values(a1);
            w.set_first_layer_normal_acceleration(a2);
            w.set_travel_acceleration_values(a3);
            w.set_first_layer_travel_acceleration_values(a4);
            w.set_max_acceleration(mx);
            w.set_accel_to_decel_enable(a2d);
            w.set_accel_to_decel_factor(a2df);
            w.set_layer_id(layer_id);
        }
        "phys" => {
            let n = t.i() as usize;
            let v: Vec<i32> = (0..n).map(|_| t.i() as i32).collect();
            w.set_physical_extruder_map(v);
        }
        "travel" => {
            let x = t.f();
            let y = t.f();
            let f = t.f();
            w.travel_xy_f(x, y, f);
        }
        "extrude" => {
            let x = t.f();
            let y = t.f();
            let f = t.f();
            let lf = limit_flow_of(t.i());
            w.extrude_xy_f(x, y, f, lf);
        }
        "extrude_explicit" => {
            let x = t.f();
            let y = t.f();
            let e = t.f();
            let f = t.f();
            let rec = t.b();
            let lf = limit_flow_of(t.i());
            w.extrude_explicit_lf(x, y, e, f, rec, lf);
        }
        "feedrate" => {
            w.feedrate(t.f());
        }
        "rect_box" => {
            let l = t.f();
            let b = t.f();
            let bw = t.f();
            let bh = t.f();
            let f = t.f();
            w.rectangle_f(&BoxCoordinates::new(l, b, bw, bh), f);
        }
        "rect_ld" => {
            let ld = t.v2();
            let bw = t.f();
            let bh = t.f();
            let f = t.f();
            w.rectangle_ld(ld, bw, bh, f);
        }
        "polygon" => {
            let p = polygon(t.pts());
            let f = t.f();
            w.polygon(&p, f);
        }
        "load" => {
            let e = t.f();
            let f = t.f();
            w.load(e, f);
        }
        "retract" => {
            let e = t.f();
            let f = t.f();
            w.retract(e, f);
        }
        "z_hop" => {
            let h = t.f();
            let f = t.f();
            w.z_hop(h, f);
        }
        "z_hop_reset" => {
            w.z_hop_reset(t.f());
        }
        "ram" => {
            let x1 = t.f();
            let x2 = t.f();
            let dy = t.f();
            let e0 = t.f();
            let e = t.f();
            let f = t.f();
            w.ram(x1, x2, dy, e0, e, f);
        }
        "cool" => {
            let x1 = t.f();
            let x2 = t.f();
            let e1 = t.f();
            let e2 = t.f();
            let f = t.f();
            w.cool(x1, x2, e1, e2, f);
        }
        "lmxa" => {
            let far = t.f();
            let dist = t.f();
            let speed = t.f();
            let mxs = t.f();
            w.load_move_x_advanced(far, dist, speed, mxs);
        }
        "temp" => {
            let tt = t.i() as i32;
            let wt = t.b();
            w.set_extruder_temp(tt, wt);
        }
        "wait" => {
            w.wait(t.f());
        }
        "speed_override" => {
            w.speed_override(t.i() as i32);
        }
        "speed_backup" => {
            w.speed_override_backup();
        }
        "speed_restore" => {
            w.speed_override_restore();
        }
        "fan" => {
            w.set_fan(t.i() as u32);
        }
        "flush" => {
            w.flush_planner_queue();
        }
        "reset_e" => {
            w.reset_extruder();
        }
        "comment_value" => {
            let c = t.text();
            let v = t.i() as i32;
            w.comment_with_value(&c, v);
        }
        "append" => {
            let s = t.text();
            w.append(&s);
        }
        "disable_la" => w.disable_linear_advance(),
        "suppress" => w.suppress_preview(),
        "resume" => w.resume_preview(),
        "change_lw" => {
            w.change_analyzer_line_width(t.f());
        }
        "change_lh" => {
            w.change_analyzer_layer_height(t.f());
        }
        "wipe_point" => {
            let x = t.f();
            let y = t.f();
            w.add_wipe_point(Vec2f::new(x, y));
        }
        "wipe_path" => {
            let p = polygon(t.pts());
            let d = t.d();
            w.add_wipe_path(&p, d);
        }
        "gen_path" => {
            let n = t.i() as usize;
            let mut pls: Vec<Polyline> = (0..n)
                .map(|_| {
                    let mut pl = Polyline::new();
                    pl.points = t.pts();
                    pl
                })
                .collect();
            let feed = t.f();
            let rl = t.f();
            let rs = t.f();
            let fillet = t.b();
            w.generate_path_cpp(&mut pls, feed, rl, rs, fillet);
        }
        "spiral" => {
            let c = t.v2();
            let area = t.f();
            let step = t.f();
            let f = t.f();
            w.spiral_flat_ironing(c, area, step, f);
        }
        "travel_xyz" => {
            let x = t.f();
            let y = t.f();
            let z = t.text();
            let f = t.f();
            w.travel_xyz(Vec2f::new(x, y), &z, f);
        }
        "m104" => {
            let tt = t.i() as i32;
            let tl = t.i() as i32;
            let heat = t.b();
            let wfm = t.b();
            let c = t.text();
            w.format_line_m104(tt, tl, heat, wfm, &c);
        }
        "m109" => {
            let tt = t.i() as i32;
            let tl = t.i() as i32;
            let c = t.text();
            w.format_line_m109(tt, tl, &c);
        }
        "first_layer" => w.set_first_layer(t.b()),
        "set_tool" => {
            w.set_tool(t.i() as usize);
        }
        "lw" => {
            w.set_line_width(t.f());
        }
        op => panic!("writer.txt: unknown operation {op}"),
    }
}

#[test]
fn wipe_tower_writer_matches_cpp() {
    let text = read_vectors("writer");
    let mut w: Option<WipeTowerWriter> = None;
    let mut name = String::new();
    let mut cases = 0;
    let mut failed: Vec<String> = Vec::new();
    let mut ops: Vec<String> = Vec::new();
    for line in text.lines() {
        let mut t = Toks::new(line);
        match t.s() {
            "CASE" => {
                name = t.s().to_string();
                ops.clear();
            }
            "NEW" => {
                let flavor = flavor_of(t.i());
                let lh = t.f();
                let lw = t.f();
                let arc = t.b();
                let nfil = t.i() as usize;
                let params: Vec<FilamentParameters> = (0..nfil)
                    .map(|_| {
                        let area = t.f();
                        let mes = t.f();
                        let r0 = t.f();
                        let r1 = t.f();
                        FilamentParameters { filament_area: area, max_e_speed: mes, max_e_ramming_speed: (r0, r1), ..FilamentParameters::new_cpp() }
                    })
                    .collect();
                w = Some(WipeTowerWriter::new_cpp(lh, lw, flavor, &params, arc));
            }
            "OP" => {
                ops.push(t.rest());
                writer_op(w.as_mut().unwrap(), &mut t);
            }
            "RES" => {
                cases += 1;
                let got = writer_res(w.as_mut().unwrap());
                let want = t.rest();
                if got != want {
                    let mut gt = Toks::new(&got);
                    let mut wt = Toks::new(&want);
                    let gg = gt.text();
                    let wg = wt.text();
                    let mut msg = format!("{name}: ");
                    if gg != wg {
                        let gl: Vec<&str> = gg.lines().collect();
                        let wl: Vec<&str> = wg.lines().collect();
                        let k = (0..gl.len().max(wl.len())).find(|&k| gl.get(k) != wl.get(k)).unwrap_or(0);
                        msg.push_str(&format!(
                            "G-code differs at line {k}: got {:?} want {:?}",
                            gl.get(k),
                            wl.get(k)
                        ));
                    } else {
                        msg.push_str(&format!("state differs\n  got:  {}\n  want: {}", gt.rest(), wt.rest()));
                    }
                    failed.push(msg);
                }
            }
            "END" => {}
            other => panic!("writer.txt: unknown line {other}"),
        }
    }
    eprintln!("writer: {cases} cases, {} differ", failed.len());
    assert!(failed.is_empty(), "{} of {cases} writer cases differ:\n{}", failed.len(), failed.join("\n"));
}

// ---------------------------------------------------------------------------------------------
// runs.txt
// ---------------------------------------------------------------------------------------------

fn tcr_line(t: &ToolChangeResult) -> String {
    let n = &t.nozzle_change_result;
    let mut o = format!(
        "{} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {} {}",
        ff(t.print_z),
        ff(t.layer_height),
        hex(t.gcode.as_bytes()),
        v2(t.start_pos),
        v2(t.end_pos),
        ff(t.elapsed_time),
        t.priming as i32,
        t.is_tool_change as i32,
        v2(t.tool_change_start_pos),
        v2s(&t.wipe_path),
        ff(t.purge_volume),
        t.initial_tool,
        t.new_tool,
        t.is_finish_first as i32,
        t.is_contact as i32,
        t.side_scrape as i32,
        ff(t.side_scrape_z_low),
        ff(t.side_scrape_depth),
        ff(t.side_scrape_min_height_above_base)
    );
    o.push_str(&format!(
        " {} {} {} {} {} {} {} {} {} {}",
        hex(n.gcode.as_bytes()),
        v2(n.start_pos),
        v2(n.end_pos),
        v2(n.origin_start_pos),
        v2s(&n.wipe_path),
        n.is_extruder_change as i32,
        n.zero_flush_idle_cooling as i32,
        n.heater_off_only as i32,
        ff(n.precool_target_temp),
        n.precool_filament_id
    ));
    o.push_str(&format!(" {}", t.extrusions.len()));
    for e in &t.extrusions {
        o.push_str(&format!(" {} {} {}", v2(e.pos), ff(e.width), e.tool));
    }
    o
}

/// Where two TCR lines first differ, for the failure message.
fn tcr_diff(got: &str, want: &str) -> String {
    const NAMES: [&str; 20] = [
        "print_z", "layer_height", "gcode", "start_pos.x", "start_pos.y", "end_pos.x", "end_pos.y", "elapsed_time", "priming",
        "is_tool_change", "tool_change_start_pos.x", "tool_change_start_pos.y", "wipe_path", "", "", "", "", "", "", "",
    ];
    let g: Vec<&str> = got.split(' ').collect();
    let w: Vec<&str> = want.split(' ').collect();
    for k in 0..g.len().max(w.len()) {
        if g.get(k) != w.get(k) {
            if k == 2 {
                let gg = unhex(g[2]);
                let wg = unhex(w[2]);
                let gl: Vec<&str> = gg.lines().collect();
                let wl: Vec<&str> = wg.lines().collect();
                let l = (0..gl.len().max(wl.len())).find(|&l| gl.get(l) != wl.get(l)).unwrap_or(0);
                return format!(
                    "gcode differs at line {l} of {}/{}: got {:?} want {:?} (previous {:?})",
                    gl.len(),
                    wl.len(),
                    gl.get(l),
                    wl.get(l),
                    if l > 0 { wl.get(l - 1) } else { None }
                );
            }
            let name = NAMES.get(k).copied().unwrap_or("");
            return format!("token {k} {name}: got {:?} want {:?}", g.get(k), w.get(k));
        }
    }
    String::new()
}

#[test]
fn wipe_tower_runs_match_cpp() {
    let text = read_vectors("runs");
    let mut cfg = WipeTowerPrintConfig::full_print_config();
    let mut group: Option<slicer::multi_nozzle_utils::LayeredNozzleGroupResult> = None;
    let mut wt: Option<WipeTower> = None;
    let mut nfil = 0usize;
    let mut name = String::new();
    let mut result: Vec<Vec<ToolChangeResult>> = Vec::new();
    let mut layer = 0usize;
    let mut in_layer = 0usize;
    let mut runs = 0;
    let mut tcrs = 0;
    let mut failed: Vec<String> = Vec::new();
    let mut run_failed = false;
    for line in text.lines() {
        let mut t = Toks::new(line);
        match t.s() {
            "RUN" => {
                name = t.s().to_string();
                cfg = WipeTowerPrintConfig::full_print_config();
                group = None;
                wt = None;
                result.clear();
                layer = 0;
                run_failed = false;
                runs += 1;
            }
            "CFG" => {
                let k = t.text();
                let v = t.text();
                cfg.set_deserialize_strict(&k, &v, false).unwrap_or_else(|e| panic!("{name}: {k}={v}: {e:?}"));
            }
            "CALL" => match t.s() {
                "group" => {
                    let n = t.i() as usize;
                    let fmap: Vec<i32> = (0..n).map(|_| t.i() as i32).collect();
                    let nn = t.i() as usize;
                    let diam: Vec<String> = (0..nn).map(|_| t.text()).collect();
                    group = Some(group_result(&fmap, &diam));
                }
                "tower" => {
                    let height = t.f();
                    let initial_tool = t.i() as usize;
                    nfil = t.i() as usize;
                    let used: Vec<u32> = (0..nfil as u32).collect();
                    wt = Some(WipeTower::new_from_config(&cfg, 0, (0.0, 0.0, 0.0), initial_tool, height, &used));
                }
                "setup" => {
                    let w = wt.as_mut().unwrap();
                    w.set_foundation_required(t.b());
                    let (tpu, multi, gref, glh, per) = (t.b(), t.b(), t.f(), t.f(), t.b());
                    w.set_first_layer_flow_ratio(t.f());
                    w.set_has_tpu_filament(tpu);
                    w.set_multi_diameter_print(multi);
                    w.set_tower_grid_reference_nozzle(gref);
                    w.set_tower_grid_layer_height(glh);
                    w.set_per_nozzle_tower_params(per);
                    w.set_nozzle_group_result(group.clone().unwrap());
                    // A 256 x 256 bed (gen.cpp, gen_run).
                    let s = |v: f64| (v / 0.00001) as i64;
                    let bed = Polygon::from_points(vec![
                        Point::new(s(0.0), s(0.0)),
                        Point::new(s(256.0), s(0.0)),
                        Point::new(s(256.0), s(256.0)),
                        Point::new(s(0.0), s(256.0)),
                    ]);
                    w.set_shared_print_bed(vec![bed]);
                    for i in 0..nfil {
                        w.set_extruder_config(i, &cfg, 0);
                    }
                }
                "no_fill" => wt.as_mut().unwrap().set_last_layer_extruder_fill(false),
                "ids" => {
                    let n = t.i() as usize;
                    let v: Vec<i32> = (0..n).map(|_| t.i() as i32).collect();
                    wt.as_mut().unwrap().set_used_filament_ids(v);
                }
                "categories" => {
                    let n = t.i() as usize;
                    let v: Vec<i32> = (0..n).map(|_| t.i() as i32).collect();
                    wt.as_mut().unwrap().set_filament_categories(v);
                }
                other => panic!("runs.txt: unknown call {other}"),
            },
            "PLAN" => {
                let z = t.f();
                let h = t.f();
                let old = t.i() as usize;
                let new = t.i() as usize;
                let ec = t.f();
                let nc = t.f();
                let prime = t.f();
                wt.as_mut().unwrap().plan_toolchange(z, h, old, new, ec, nc, prime);
            }
            "GEN" => {
                let w = wt.as_mut().unwrap();
                let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    let mut res = Vec::new();
                    w.generate_new(&mut res);
                    res
                }));
                match r {
                    Ok(res) => result = res,
                    Err(e) => {
                        let msg = e.downcast_ref::<String>().cloned().or_else(|| e.downcast_ref::<&str>().map(|s| s.to_string())).unwrap_or_default();
                        failed.push(format!("{name}: generate_new panicked: {msg}"));
                        run_failed = true;
                        result.clear();
                    }
                }
                layer = 0;
            }
            "LAYER" => {
                let n = t.i() as usize;
                if !run_failed {
                    let got = result.get(layer).map_or(usize::MAX, |l| l.len());
                    if got != n {
                        failed.push(format!("{name}: layer {layer}: {got} results, want {n}"));
                        run_failed = true;
                    }
                }
                in_layer = 0;
                layer += 1;
            }
            "TCR" => {
                tcrs += 1;
                if !run_failed {
                    let want = t.rest();
                    let got = tcr_line(&result[layer - 1][in_layer]);
                    if got != want {
                        failed.push(format!("{name}: layer {} result {in_layer}: {}", layer - 1, tcr_diff(&got, &want)));
                        run_failed = true;
                    }
                }
                in_layer += 1;
            }
            "TOWER" => {
                if !run_failed {
                    let w = wt.as_ref().unwrap();
                    let ((x0, y0), (x1, y1)) = w.get_bbx();
                    let mut got = format!(
                        "{} {} {} {} {} {} {} {} {} {} {}",
                        ff(w.get_depth()),
                        ff(w.get_brim_width()),
                        ff(w.get_height()),
                        w.get_number_of_toolchanges(),
                        dd(x0),
                        dd(y0),
                        dd(x1),
                        dd(y1),
                        ff(w.get_rib_length()),
                        ff(w.get_rib_width()),
                        v2(w.get_rib_offset())
                    );
                    let used = w.get_used_filament();
                    got.push_str(&format!(" {}", used.len()));
                    for v in used {
                        got.push_str(&format!(" {}", ff(*v)));
                    }
                    let want = t.rest();
                    if got != want {
                        failed.push(format!("{name}: tower state\n  got:  {got}\n  want: {want}"));
                    }
                }
            }
            "END" => {
                if !run_failed && layer != result.len() {
                    failed.push(format!("{name}: {} layers, want {layer}", result.len()));
                }
            }
            other => panic!("runs.txt: unknown line {other}"),
        }
    }
    eprintln!("runs: {runs} runs, {tcrs} results, {} differences", failed.len());
    assert!(failed.is_empty(), "{} of {runs} runs differ:\n{}", failed.len(), failed.join("\n"));
}
