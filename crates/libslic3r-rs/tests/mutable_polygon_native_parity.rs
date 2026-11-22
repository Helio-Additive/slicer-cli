//! `slicer::mutable_polygon` (the port of libslic3r/MutablePolygon.cpp and MutablePolygon.hpp) against the
//! C++ at the pin.
//!
//! `tests/data/mutable_polygon/vectors.txt.gz` is the output of `gen.cpp` beside it, which compiles the
//! pin's MutablePolygon.cpp in (with the members of the class made public); the header of gen.cpp describes
//! the lines. Every line is `<what and inputs> -> <results>`: the same calls run here and have to give the
//! same line, with the faithful arm of the gate `MUTABLE_POLYGON_PIN` (the default).
//!
//! - `ops`: the container. Constructors, both `assign`, both `polygon`, `clear`, `insert` and `remove`
//!   through the polygon and through an iterator, the iterators (const and not), `==` / `!=`, and a `range`
//!   with all of its operations; after every operation the storage (`m_data` with its links, `m_size`,
//!   `m_head`, the head of the free list) has to be the pin's. `capacity()` is not compared: it is the
//!   allocator's number.
//! - `rd0`, `rd1`, `rd2`, `rd3`: the four overloads of `remove_duplicates`.
//! - `cnc`: `clip_narrow_corner`, called as `smooth_outward` calls it.
//! - `so`, `sop`, `sops`, `soex`: the four overloads of `smooth_outward`.

use slicer::geometry::{ExPolygon, ExPolygons, Point, Polygon, Polygons};
use slicer::libslic3r::{SCALED_EPSILON, SCALING_FACTOR};
use slicer::mutable_polygon::{
    clip_narrow_corner, remove_duplicates, remove_duplicates_angle_overload, remove_duplicates_eps_overload,
    remove_duplicates_expolygons_overload, smooth_outward, smooth_outward_expolygons_overload, smooth_outward_polygon_overload,
    smooth_outward_polygons_overload, ConstIterator, Iterator as MpIterator, MutablePolygon, MutablePolygonConst, MutablePolygonRange,
    Range, Vec2i64,
};
use std::collections::BTreeMap;
use std::io::Read;

fn read_vectors() -> String {
    let path = format!("{}/tests/data/mutable_polygon/vectors.txt.gz", env!("CARGO_MANIFEST_DIR"));
    let file = std::fs::File::open(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let mut text = String::new();
    flate2::read::GzDecoder::new(file).read_to_string(&mut text).expect("gzip");
    text
}

struct Tok<'a> {
    t: Vec<&'a str>,
    k: usize,
}
impl<'a> Tok<'a> {
    fn new(s: &'a str) -> Self {
        Tok { t: s.split(' ').filter(|x| !x.is_empty()).collect(), k: 0 }
    }
    fn s(&mut self) -> &'a str {
        self.k += 1;
        self.t[self.k - 1]
    }
    fn i(&mut self) -> i64 {
        self.s().parse().expect("integer")
    }
    fn u(&mut self) -> usize {
        self.s().parse().expect("count")
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn pt(&mut self) -> Point {
        let x = self.i();
        let y = self.i();
        Point::new(x, y)
    }
    fn poly(&mut self) -> Polygon {
        let n = self.u();
        let mut p = Polygon::default();
        for _ in 0..n {
            let pt = self.pt();
            p.points.push(pt);
        }
        p
    }
    fn polys(&mut self) -> Polygons {
        let n = self.u();
        (0..n).map(|_| self.poly()).collect()
    }
    fn expolys(&mut self) -> ExPolygons {
        let n = self.u();
        let mut out = Vec::new();
        for _ in 0..n {
            let holes = self.u();
            let mut e = ExPolygon::default();
            e.contour = self.poly();
            for _ in 0..holes {
                let h = self.poly();
                e.holes.push(h);
            }
            out.push(e);
        }
        out
    }
}

fn hd(v: f64) -> String {
    format!(" {:016x}", v.to_bits())
}

fn poly_s(p: &Polygon) -> String {
    let mut s = format!(" {}", p.points.len());
    for pt in &p.points {
        s += &format!(" {} {}", pt.x, pt.y);
    }
    s
}

fn polys_s(ps: &Polygons) -> String {
    let mut s = format!(" {}", ps.len());
    for p in ps {
        s += &poly_s(p);
    }
    s
}

fn expolys_s(es: &ExPolygons) -> String {
    let mut s = format!(" {}", es.len());
    for e in es {
        s += &format!(" {}", e.holes.len());
        s += &poly_s(&e.contour);
        for h in &e.holes {
            s += &poly_s(h);
        }
    }
    s
}

/// gen.cpp `put_state()`
fn state(mp: &MutablePolygon) -> String {
    let (data, size, head, head_free) = mp.members();
    let mut s = format!(" S {size} {head} {head_free} {}", data.len());
    for (p, prev, next) in &data {
        s += &format!(" {} {} {prev} {next}", p.x, p.y);
    }
    s += &format!(" E {} {}", mp.empty() as i32, mp.valid() as i32);
    s += &format!(
        " B {} {} {} {} {} {}",
        mp.cbegin().idx(),
        mp.cend().idx(),
        mp.begin().idx(),
        mp.end().idx(),
        MutablePolygonConst::begin(mp).idx(),
        MutablePolygonConst::end(mp).idx()
    );
    s += " P";
    s += &poly_s(&mp.polygon());
    s
}

/// begin() advanced k times.
fn advanced(mp: &MutablePolygon, k: usize) -> MpIterator {
    let mut it = mp.begin();
    for _ in 0..k {
        it.inc(mp);
    }
    it
}

fn range_s(rg: &Range) -> String {
    format!(" {} {} {}", rg.begin().idx(), rg.end().idx(), rg.empty() as i32)
}

/// One line `ops <case> <operation> ...`: the results and the state after it.
fn run_op(t: &mut Tok, mp: &mut MutablePolygon, rg: &mut Option<Range>) -> String {
    let op = t.s();
    let mut out = String::new();
    match op {
        "new" => {
            *mp = MutablePolygon::new();
            assert!(*mp == MutablePolygon::default());
        }
        "insert_it" | "insert_mp" => {
            let k = t.u();
            let pt = t.pt();
            let it = advanced(mp, k);
            let res = if op == "insert_it" { it.insert(mp, pt) } else { mp.insert(it, pt) };
            out += &format!(" {}", res.idx());
        }
        "remove_it" => {
            let k = t.u();
            let mut it = advanced(mp, k);
            let res = it.remove(mp);
            out += &format!(" {} {} {}", it.idx(), it.valid() as i32, res.idx());
        }
        "remove_mp" => {
            let k = t.u();
            let it = advanced(mp, k);
            let res = mp.remove(it);
            out += &format!(" {} {}", res.idx(), res.valid() as i32);
        }
        "walk" => {
            let k = t.u();
            let j = t.u();
            let mut it = mp.begin();
            for _ in 0..k {
                it.inc(mp);
            }
            out += &format!(
                " {} {} {} {} {} {} {}",
                it.idx(),
                it.prev(mp).idx(),
                it.next(mp).idx(),
                it.point(mp).x,
                it.point(mp).y,
                it.size(mp),
                it.valid() as i32
            );
            for _ in 0..j {
                it.dec(mp);
            }
            out += &format!(" {} {} {}", it.idx(), (it == mp.begin()) as i32, (it != mp.end()) as i32);
            let mut cit: ConstIterator = MutablePolygonConst::begin(mp);
            for _ in 0..k {
                cit.inc(mp);
            }
            out += &format!(
                " {} {} {} {} {} {} {}",
                cit.idx(),
                cit.prev(mp).idx(),
                cit.next(mp).idx(),
                cit.point(mp).x,
                cit.point(mp).y,
                cit.size(mp),
                cit.valid() as i32
            );
            for _ in 0..j {
                cit.dec(mp);
            }
            out += &format!(
                " {} {} {}",
                cit.idx(),
                (cit == MutablePolygonConst::begin(mp)) as i32,
                (cit != MutablePolygonConst::end(mp)) as i32
            );
        }
        "range_new" => {
            let r = Range::new(mp);
            // range(begin, end), MutablePolygon.hpp:71
            let r2 = Range::from_iterators(mp.begin(), mp.end());
            assert_eq!(range_s(&r), range_s(&r2));
            out += &range_s(&r);
            *rg = Some(r);
        }
        "range_process_next" => {
            let r = rg.as_mut().expect("range");
            let res = r.process_next(mp);
            out += &format!(" {}", res.idx());
            out += &range_s(r);
        }
        "range_advance_front" => {
            let r = rg.as_mut().expect("range");
            r.advance_front(mp);
            out += &range_s(r);
        }
        "range_retract_back" => {
            let r = rg.as_mut().expect("range");
            r.retract_back(mp);
            out += &range_s(r);
        }
        "range_remove_front" | "range_remove_back" => {
            let k = t.u();
            let it = advanced(mp, k);
            let r = rg.as_mut().expect("range");
            let res = if op == "range_remove_front" { r.remove_front(mp, it) } else { r.remove_back(mp, it) };
            out += &format!(" {}", res.idx());
            out += &range_s(r);
        }
        "range_done" => *rg = None,
        "assign_poly" => {
            let reserve = t.u();
            let p = t.poly();
            mp.assign(&p, reserve);
        }
        "assign_range" => {
            let reserve = t.u();
            let p = t.poly();
            MutablePolygonRange::assign(mp, &p.points, reserve);
        }
        "clear" => mp.clear(),
        "polygon_out" => {
            let mut o = Polygon::default();
            o.points = vec![Point::new(1, 2), Point::new(3, 4)];
            MutablePolygonRange::polygon(mp, &mut o);
            out += &poly_s(&o);
        }
        "eq" => {
            let p = t.poly();
            let other = MutablePolygon::from_polygon(&p, 0);
            out += &format!(" {} {} {} {}", (*mp == other) as i32, (*mp != other) as i32, (other == *mp) as i32, (other != *mp) as i32);
        }
        "ctor_poly" => {
            let reserve = t.u();
            let p = t.poly();
            *mp = MutablePolygon::from_polygon(&p, reserve);
        }
        "ctor_list" | "ctor_range" => {
            let reserve = t.u();
            let p = t.poly();
            *mp = MutablePolygon::from_points(&p.points, reserve);
        }
        other => panic!("unknown operation {other}"),
    }
    out + &state(mp)
}

#[test]
fn mutable_polygon_matches_the_pin() {
    assert!(
        std::env::var("MUTABLE_POLYGON_PIN").map(|v| v != "0").unwrap_or(true),
        "the vectors are the pin's: run with the gate MUTABLE_POLYGON_PIN on"
    );
    let text = read_vectors();
    let mut by_kind: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    let mut shown = 0;
    // The container of the `ops` case in hand and its range.
    let mut ops_case = usize::MAX;
    let mut ops_mp = MutablePolygon::new();
    let mut ops_rg: Option<Range> = None;
    for (line_no, line) in text.lines().enumerate() {
        let (lhs, want) = line.split_once(" ->").expect("arrow");
        let mut t = Tok::new(lhs);
        let kind = t.s();
        let got = match kind {
            "const" => hd(0.01 / SCALING_FACTOR) + &hd(SCALED_EPSILON) + &hd(SCALED_EPSILON * SCALED_EPSILON),
            "ops" => {
                let case = t.u();
                if case != ops_case {
                    ops_case = case;
                    ops_mp = MutablePolygon::new();
                    ops_rg = None;
                }
                run_op(&mut t, &mut ops_mp, &mut ops_rg)
            }
            "rd0" => {
                let p = t.poly();
                let mut mp = MutablePolygon::from_polygon(&p, p.points.len() * 2);
                remove_duplicates(&mut mp);
                state(&mp)
            }
            "rd1" => {
                let eps = t.d();
                let p = t.poly();
                let mut mp = MutablePolygon::from_polygon(&p, p.points.len() * 2);
                remove_duplicates_eps_overload::remove_duplicates(&mut mp, eps);
                state(&mp)
            }
            "rd2" => {
                let scaled_eps = t.i();
                let max_angle = t.d();
                let p = t.poly();
                let mut mp = MutablePolygon::from_polygon(&p, p.points.len() * 2);
                remove_duplicates_angle_overload::remove_duplicates(&mut mp, scaled_eps, max_angle);
                state(&mp)
            }
            "rd3" => {
                let scaled_eps = t.i();
                let max_angle = t.d();
                let es = t.expolys();
                expolys_s(&remove_duplicates_expolygons_overload::remove_duplicates(es, scaled_eps, max_angle))
            }
            "cnc" => {
                let clip = t.i();
                let k = t.u();
                let p = t.poly();
                let mut mp = MutablePolygon::from_polygon(&p, p.points.len() * 2);
                let mut rg = Range::new(&mp);
                let mut it1 = rg.process_next(&mp);
                for _ in 0..k {
                    it1 = rg.process_next(&mp);
                }
                let mut it0 = it1.prev(&mp);
                let mut it2 = it1.next(&mp);
                let (p0, p1, p2) = (*it0.point(&mp), *it1.point(&mp), *it2.point(&mp));
                let l2v02 = (p2.x - p0.x) * (p2.x - p0.x) + (p2.y - p0.y) * (p2.y - p0.y);
                it1.remove(&mut mp);
                let res = clip_narrow_corner(Vec2i64 { x: p1.x, y: p1.y }, &mut it0, &mut it2, &mut rg, l2v02, clip, &mut mp);
                format!(" {} {} {} {} {}", res as i32, it0.idx(), it2.idx(), rg.begin().idx(), rg.end().idx()) + &state(&mp)
            }
            "so" => {
                let clip = t.i();
                let p = t.poly();
                let mut mp = MutablePolygon::from_polygon(&p, p.points.len() * 2);
                smooth_outward(&mut mp, clip);
                state(&mp)
            }
            "sop" => {
                let clip = t.i();
                let p = t.poly();
                poly_s(&smooth_outward_polygon_overload::smooth_outward(p, clip))
            }
            "sops" => {
                let clip = t.i();
                let ps = t.polys();
                polys_s(&smooth_outward_polygons_overload::smooth_outward(ps, clip))
            }
            "soex" => {
                let clip = t.i();
                let es = t.expolys();
                expolys_s(&smooth_outward_expolygons_overload::smooth_outward(es, clip))
            }
            other => panic!("line {}: unknown kind {other}", line_no + 1),
        };
        assert_eq!(t.k, t.t.len(), "line {}: tokens left over", line_no + 1);
        let entry = by_kind.entry(kind.to_string()).or_insert((0, 0));
        entry.0 += 1;
        if got != want {
            entry.1 += 1;
            if shown < 8 {
                shown += 1;
                let cut = |s: &str| if s.len() > 900 { format!("{}...", &s[..900]) } else { s.to_string() };
                eprintln!("line {}: {}\n  expected{}\n  got     {}", line_no + 1, cut(lhs), cut(want), cut(&got));
            }
        }
    }
    let mut bad = 0;
    for (kind, (n, b)) in &by_kind {
        eprintln!("{kind}: {n} lines, {b} mismatches");
        bad += b;
    }
    for kind in ["const", "ops", "rd0", "rd1", "rd2", "rd3", "cnc", "so", "sop", "sops", "soex"] {
        assert!(by_kind.get(kind).map(|e| e.0 > 0).unwrap_or(false), "no line of kind {kind} in the vectors");
    }
    assert_eq!(bad, 0, "lines of the vectors that differ");
}
