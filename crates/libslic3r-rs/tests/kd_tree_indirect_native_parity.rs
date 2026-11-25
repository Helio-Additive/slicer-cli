//! KDTreeIndirect (KDTreeIndirect.hpp) against the pin, compiled with the engine's flags, on the tree
//! types the pin instantiates: `KDTreeIndirect<2, double>` (ShortestPath.cpp:105, 516) and
//! `KDTreeIndirect<3, float>` (SeamPlacer.cpp:279, 612; SeamPlacer.hpp:115). Vectors from
//! `tests/data/kd_tree_indirect/gen.cpp` (300 trees of each type, 24 queries each, many of them
//! midpoints of point pairs, where the rounding of `dist += d * d` decides; gzip -9n). Per tree the
//! built node array, per query `find_closest_point`, `find_closest_points<3>`, `find_nearby_points`
//! by radius and by box must be equal. With KD_TREE_DIST_FMA=0 (the unfused sum) the closest-point
//! results differ on the near ties.

use slicer::kd_tree_indirect::{
    find_closest_point, find_closest_point_eps, find_closest_points, find_closest_points_eps, find_nearby_points,
    find_nearby_points_bbox, find_nearby_points_eps, KDTreeIndirect, NPOS,
};
use slicer::libslic3r::EPSILON;
use std::io::Read;

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn n(&mut self) -> usize {
        self.s().parse().expect("count")
    }
    fn idx(&mut self) -> usize {
        let v: i64 = self.s().parse().expect("index");
        if v < 0 {
            NPOS
        } else {
            v as usize
        }
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double"))
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float"))
    }
    fn tag(&mut self, t: &str) {
        assert_eq!(self.s(), t);
    }
    fn list(&mut self, t: &str) -> Vec<usize> {
        self.tag(t);
        let n = self.n();
        (0..n).map(|_| self.idx()).collect()
    }
}

#[derive(Default)]
struct Counts {
    trees: usize,
    queries: usize,
    nodes: usize,
    c1: usize,
    c3: usize,
    nr: usize,
    nb: usize,
}

#[test]
fn kd_tree_indirect_matches_the_pin() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/kd_tree_indirect/vectors.txt.gz");
    let mut text = String::new();
    flate2::read::GzDecoder::new(std::fs::File::open(path).unwrap()).read_to_string(&mut text).unwrap();
    let mut lines = text.lines().peekable();
    let mut c = Counts::default();
    let mut first_bad: Option<String> = None;
    while let Some(line) = lines.next() {
        let mut t = Tok(line.split_whitespace());
        t.tag("tree");
        let kind = t.s();
        let n = t.n();
        c.trees += 1;
        let queries: Vec<&str> = {
            let mut v = Vec::new();
            while let Some(l) = lines.peek() {
                if l.starts_with('q') {
                    v.push(lines.next().unwrap());
                } else {
                    break;
                }
            }
            v
        };
        match kind {
            "d2" => {
                let pts: Vec<[f64; 2]> = (0..n).map(|_| [t.d(), t.d()]).collect();
                t.tag("nodes");
                let m = t.n();
                let want_nodes: Vec<usize> = (0..m).map(|_| t.idx()).collect();
                let tree = KDTreeIndirect::<2, f64, _>::with_indices(|idx: usize, dim: usize| pts[idx][dim], n);
                if tree.nodes() != want_nodes.as_slice() {
                    c.nodes += 1;
                    first_bad.get_or_insert_with(|| format!("nodes of d2 tree {}", c.trees));
                }
                for ql in &queries {
                    let mut q = Tok(ql.split_whitespace());
                    q.tag("q");
                    let center = [q.d(), q.d()];
                    let r = q.d();
                    let filt = q.n() == 1;
                    q.tag("->");
                    q.tag("c1");
                    let want_c1 = q.idx();
                    q.tag("c3");
                    let want_c3 = [q.idx(), q.idx(), q.idx()];
                    let want_nr = q.list("nr");
                    let want_nb = q.list("nb");
                    let filter = |idx: usize| !filt || idx % 5 != 0;
                    c.queries += 1;
                    if find_closest_point(&tree, &center, filter) != want_c1 {
                        c.c1 += 1;
                        first_bad.get_or_insert_with(|| format!("c1 d2: {ql}"));
                    }
                    if find_closest_points::<3, 2, f64, _, _, _>(&tree, &center, filter) != want_c3 {
                        c.c3 += 1;
                        first_bad.get_or_insert_with(|| format!("c3 d2: {ql}"));
                    }
                    if find_nearby_points(&tree, &center, r, filter) != want_nr {
                        c.nr += 1;
                        first_bad.get_or_insert_with(|| format!("nr d2: {ql}"));
                    }
                    let (bmin, bmax) = ([center[0] - r, center[1] - r], [center[0] + r, center[1] + r]);
                    if find_nearby_points_bbox(&tree, &bmin, &bmax, filter) != want_nb {
                        c.nb += 1;
                        first_bad.get_or_insert_with(|| format!("nb d2: {ql}"));
                    }
                }
            }
            "f3" => {
                let pts: Vec<[f32; 3]> = (0..n).map(|_| [t.f(), t.f(), t.f()]).collect();
                t.tag("nodes");
                let m = t.n();
                let want_nodes: Vec<usize> = (0..m).map(|_| t.idx()).collect();
                let tree = KDTreeIndirect::<3, f32, _>::with_indices(|idx: usize, dim: usize| pts[idx][dim], n);
                if tree.nodes() != want_nodes.as_slice() {
                    c.nodes += 1;
                    first_bad.get_or_insert_with(|| format!("nodes of f3 tree {}", c.trees));
                }
                // KDTreeIndirect.hpp:65, 219: CoordType(EPSILON) and numeric_limits<float>::max()
                let (eps, max) = (EPSILON as f32, f32::MAX);
                for ql in &queries {
                    let mut q = Tok(ql.split_whitespace());
                    q.tag("q");
                    let center = [q.f(), q.f(), q.f()];
                    let r = q.f();
                    let filt = q.n() == 1;
                    q.tag("->");
                    q.tag("c1");
                    let want_c1 = q.idx();
                    q.tag("c3");
                    let want_c3 = [q.idx(), q.idx(), q.idx()];
                    let want_nr = q.list("nr");
                    let want_nb = q.list("nb");
                    let filter = |idx: usize| !filt || idx % 5 != 0;
                    c.queries += 1;
                    if find_closest_point_eps(&tree, &center, eps, max, filter) != want_c1 {
                        c.c1 += 1;
                        first_bad.get_or_insert_with(|| format!("c1 f3: {ql}"));
                    }
                    if find_closest_points_eps::<3, 3, f32, _, _, _>(&tree, &center, eps, max, filter) != want_c3 {
                        c.c3 += 1;
                        first_bad.get_or_insert_with(|| format!("c3 f3: {ql}"));
                    }
                    if find_nearby_points_eps(&tree, &center, r, eps, filter) != want_nr {
                        c.nr += 1;
                        first_bad.get_or_insert_with(|| format!("nr f3: {ql}"));
                    }
                    let bmin = [center[0] - r, center[1] - r, center[2] - r];
                    let bmax = [center[0] + r, center[1] + r, center[2] + r];
                    if find_nearby_points_bbox(&tree, &bmin, &bmax, filter) != want_nb {
                        c.nb += 1;
                        first_bad.get_or_insert_with(|| format!("nb f3: {ql}"));
                    }
                }
            }
            other => panic!("unknown tree kind {other}"),
        }
    }
    eprintln!(
        "kd_tree_indirect: {} trees, {} queries; mismatches: nodes {}, closest {}, closest3 {}, nearby {}, box {}; first: {:?}",
        c.trees, c.queries, c.nodes, c.c1, c.c3, c.nr, c.nb, first_bad
    );
    assert_eq!(c.trees, 600);
    assert_eq!(c.queries, 600 * 24);
    assert_eq!((c.nodes, c.c1, c.c3, c.nr, c.nb), (0, 0, 0, 0, 0), "first: {first_bad:?}");
}
