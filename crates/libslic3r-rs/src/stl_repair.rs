//! Faithful 1:1 port of BambuStudio admesh mesh repair (the subset benchy
//! exercises): degenerate-facet removal + exact-edge connectivity
//! (`stl_check_facets_exact`, admesh/connect.cpp) + topology-based shared-vertex
//! generation (`stl_generate_shared_vertices`, admesh/shared.cpp).
//!
//! C++ `TriangleMesh::from_stl(repair=true)` keeps FACET-SOUP (per-facet 3 f32
//! verts) through repair, then builds the indexed set via the neighbor-graph fan
//! traversal — a DIFFERENT vertex identity than rust's prior exact-f32-bit
//! HashMap dedup (which kept +93 verts C++ merges/removes). This module
//! reproduces C++'s mesh so the slicer sees the identical triangulation.
//!
//! NOTE: benchy is manifold after the exact check, so `stl_check_facets_nearby`
//! (the tolerance/iteration merge) is NOT run by C++ for it and is NOT ported
//! here (gated/out-of-scope until a non-manifold model needs it).

use crate::geometry::Point3F;

/// A facet vertex, stored as the f32 bit pattern triple (exact identity, matching
/// C++ `stl_vertex` f32 comparison + the HashEdge byte key).
#[derive(Clone, Copy, PartialEq)]
pub struct FVert {
    x: f32,
    y: f32,
    z: f32,
}

impl FVert {
    #[inline]
    fn eq_exact(&self, o: &FVert) -> bool {
        // C++ degenerate test is `facet.vertex[i] == facet.vertex[j]` — Eigen Vec3f
        // VALUE equality, where -0.0 == +0.0. Use f32 == (not to_bits) so a +0/-0
        // pair counts as degenerate exactly as C++ does (matches the 552 count).
        self.x == o.x && self.y == o.y && self.z == o.z
    }
    // connect.cpp vertex_lower: (a0!=b0)?a0<b0 : (a1!=b1)?a1<b1 : a2<b2 (f32 compare).
    #[inline]
    fn lower(&self, o: &FVert) -> bool {
        if self.x != o.x {
            self.x < o.x
        } else if self.y != o.y {
            self.y < o.y
        } else {
            self.z < o.z
        }
    }
}

#[derive(Clone, Copy)]
struct Neighbors {
    neighbor: [i32; 3],
    which_vertex_not: [i8; 3],
}
impl Neighbors {
    fn new() -> Self {
        Neighbors { neighbor: [-1; 3], which_vertex_not: [-1; 3] }
    }
    #[inline]
    fn num_neighbors(&self) -> i32 {
        3 - ((self.neighbor[0] == -1) as i32
            + (self.neighbor[1] == -1) as i32
            + (self.neighbor[2] == -1) as i32)
    }
}

// HashEdge: key = the two edge verts (ordered by vertex_lower), byte-copied f32
// bits; facet_number + which_edge (+3 if stored backwards). connect.cpp.
#[derive(Clone, Copy)]
struct HashEdge {
    key: [u32; 6],
    facet_number: i32,
    which_edge: i32,
}

impl HashEdge {
    fn load_exact(a: &FVert, b: &FVert, facet: i32, edge: i32) -> HashEdge {
        let mut which_edge = edge;
        let (lo, hi) = if a.lower(b) {
            (a, b)
        } else {
            which_edge += 3; // stored backwards
            (b, a)
        };
        // Negative-zero → positive-zero normalization (connect.cpp:72-84) so equal
        // edges with -0/+0 compare equal by key.
        #[inline]
        fn norm0(b: u32) -> u32 {
            if b == 0x8000_0000 { 0 } else { b }
        }
        let key = [
            norm0(lo.x.to_bits()),
            norm0(lo.y.to_bits()),
            norm0(lo.z.to_bits()),
            norm0(hi.x.to_bits()),
            norm0(hi.y.to_bits()),
            norm0(hi.z.to_bits()),
        ];
        HashEdge { key, facet_number: facet, which_edge }
    }
    // connect.cpp: ((k0/11 + k1/7 + k2/3) ^ (k3/11 + k4/7 + k5/3)) % M.
    #[inline]
    fn hash(&self, m: i32) -> i32 {
        let k = &self.key;
        let a = k[0] / 11 + k[1] / 7 + k[2] / 3;
        let b = k[3] / 11 + k[4] / 7 + k[5] / 3;
        ((a ^ b) % (m as u32)) as i32
    }
    #[inline]
    fn key_eq(&self, o: &HashEdge) -> bool {
        self.key == o.key
    }
}

// connect.cpp hash_size_from_nr_faces: smallest good prime > nr_faces*3*2 − 1.
fn hash_size_from_nr_faces(nr_faces: usize) -> i32 {
    const PRIMES: [u32; 15] = [
        98317, 196613, 393241, 786433, 1572869, 3145739, 6291469, 12582917, 25165843, 50331653,
        100663319, 201326611, 402653189, 805306457, 1610612741,
    ];
    let target = (nr_faces as u64) * 3 * 2;
    for &p in &PRIMES {
        if (p as u64) > target.saturating_sub(1) {
            return p as i32;
        }
    }
    *PRIMES.last().unwrap() as i32
}

/// admesh `record_neighbors` (connect.cpp): pair facet a's edge with facet b's.
fn record_neighbors(
    neighbors: &mut [Neighbors],
    stats: &mut Stats,
    a: &HashEdge,
    b: &HashEdge,
) {
    let af = a.facet_number as usize;
    let bf = b.facet_number as usize;
    let ae = (a.which_edge % 3) as usize;
    let be = (b.which_edge % 3) as usize;
    neighbors[af].neighbor[ae] = b.facet_number;
    neighbors[af].which_vertex_not[ae] = ((b.which_edge + 2) % 3) as i8;
    neighbors[bf].neighbor[be] = a.facet_number;
    neighbors[bf].which_vertex_not[be] = ((a.which_edge + 2) % 3) as i8;
    if (a.which_edge < 3 && b.which_edge < 3) || (a.which_edge > 2 && b.which_edge > 2) {
        // opposite orientation
        neighbors[af].which_vertex_not[ae] += 3;
        neighbors[bf].which_vertex_not[be] += 3;
    }
    stats.connected_edges += 2;
    match neighbors[af].num_neighbors() {
        1 => stats.connected_facets_1_edge += 1,
        2 => stats.connected_facets_2_edge += 1,
        3 => stats.connected_facets_3_edge += 1,
        _ => {}
    }
    match neighbors[bf].num_neighbors() {
        1 => stats.connected_facets_1_edge += 1,
        2 => stats.connected_facets_2_edge += 1,
        3 => stats.connected_facets_3_edge += 1,
        _ => {}
    }
}

#[derive(Default)]
pub struct Stats {
    pub number_of_facets: usize,
    pub degenerate_facets: i32,
    pub facets_removed: i32,
    pub connected_edges: i32,
    pub connected_facets_1_edge: i32,
    pub connected_facets_2_edge: i32,
    pub connected_facets_3_edge: i32,
}

/// Run admesh exact repair on facet-soup: remove degenerate facets, build the
/// exact-edge neighbor graph, then generate shared vertices via fan traversal.
/// Returns (vertices, indices, stats). Faithful to C++ for manifold-after-exact
/// meshes (benchy); nearby-merge is not applied.
pub fn repair_and_index(
    mut facets: Vec<[FVert; 3]>,
) -> (Vec<Point3F>, Vec<[u32; 3]>, Stats) {
    let mut stats = Stats::default();

    // --- stl_check_facets_exact: remove degenerate facets (two verts exactly
    // equal), swap-with-last (connect.cpp). ---
    let mut i = 0;
    while i < facets.len() {
        let f = &facets[i];
        if f[0].eq_exact(&f[1]) || f[1].eq_exact(&f[2]) || f[0].eq_exact(&f[2]) {
            let last = facets.len() - 1;
            facets.swap(i, last);
            facets.pop();
            stats.facets_removed += 1;
            stats.degenerate_facets += 1;
        } else {
            i += 1;
        }
    }
    stats.number_of_facets = facets.len();

    // --- exact-edge connectivity (hash table, match-and-remove + record_neighbors). ---
    let mut neighbors = vec![Neighbors::new(); facets.len()];
    let m = hash_size_from_nr_faces(facets.len());
    // chained hash: per bucket, a Vec of HashEdge (insertion-ordered, match=front-most equal).
    let mut heads: Vec<Vec<HashEdge>> = vec![Vec::new(); m as usize];
    for (fi, f) in facets.iter().enumerate() {
        for j in 0..3 {
            let edge = HashEdge::load_exact(&f[j], &f[(j + 1) % 3], fi as i32, j as i32);
            let chain = edge.hash(m) as usize;
            // C++ insert_edge: if the chain head (or the first equal in chain) matches,
            // record + delete it; else append. Faithful: find first key-equal in chain;
            // if found, match+remove; else push.
            // edges_equal (connect.cpp:237): different facet AND equal key.
            let bucket = &mut heads[chain];
            if let Some(pos) = bucket
                .iter()
                .position(|e| e.facet_number != edge.facet_number && e.key_eq(&edge))
            {
                let matched = bucket.remove(pos);
                record_neighbors(&mut neighbors, &mut stats, &edge, &matched);
            } else {
                bucket.push(edge);
            }
        }
    }

    // --- stl_generate_shared_vertices: fan traversal of the neighbor graph. ---
    let nf = facets.len();
    let mut indices: Vec<[i32; 3]> = vec![[-1, -1, -1]; nf];
    let mut vertices: Vec<Point3F> = Vec::with_capacity(nf / 2);
    let mut fan_stamp: u32 = 0;
    let mut visited: Vec<u32> = vec![0; nf];

    for facet_idx in 0..nf {
        for j in 0..3 {
            if indices[facet_idx][j] != -1 {
                continue;
            }
            // New shared vertex from facet_idx's j-th vert.
            let v = &facets[facet_idx][j];
            vertices.push(Point3F::new(v.x as f64, v.y as f64, v.z as f64));
            let new_idx = (vertices.len() - 1) as i32;
            let mut facet_in_fan = facet_idx as i32;
            let mut edge_direction = false;
            let mut traversal_reversed = false;
            let mut vnot: i32 = ((j + 2) % 3) as i32;
            fan_stamp += 1;
            loop {
                let next_edge: i32;
                let pivot_vertex: i32;
                if vnot > 2 {
                    if !edge_direction {
                        pivot_vertex = (vnot + 2) % 3;
                        next_edge = pivot_vertex;
                    } else {
                        pivot_vertex = (vnot + 1) % 3;
                        next_edge = vnot % 3;
                    }
                    edge_direction = !edge_direction;
                } else if !edge_direction {
                    pivot_vertex = (vnot + 1) % 3;
                    next_edge = vnot;
                } else {
                    pivot_vertex = (vnot + 2) % 3;
                    next_edge = pivot_vertex;
                }
                indices[facet_in_fan as usize][pivot_vertex as usize] = new_idx;
                visited[facet_in_fan as usize] = fan_stamp;

                let next_facet = neighbors[facet_in_fan as usize].neighbor[next_edge as usize];
                if next_facet == -1 {
                    if traversal_reversed {
                        break;
                    } else {
                        edge_direction = true;
                        vnot = ((j + 1) % 3) as i32;
                        traversal_reversed = true;
                        facet_in_fan = facet_idx as i32;
                    }
                } else if next_facet == facet_idx as i32 {
                    break;
                } else if next_facet >= nf as i32 {
                    break;
                } else if visited[next_facet as usize] == fan_stamp {
                    break;
                } else {
                    vnot = neighbors[facet_in_fan as usize].which_vertex_not[next_edge as usize]
                        as i32;
                    facet_in_fan = next_facet;
                }
            }
        }
    }

    let out_indices: Vec<[u32; 3]> = indices
        .into_iter()
        .map(|t| [t[0] as u32, t[1] as u32, t[2] as u32])
        .collect();
    (vertices, out_indices, stats)
}

/// Build the facet-soup FVert triples from raw STL facet vertices (f32).
pub fn make_facet(v0: [f32; 3], v1: [f32; 3], v2: [f32; 3]) -> [FVert; 3] {
    [
        FVert { x: v0[0], y: v0[1], z: v0[2] },
        FVert { x: v1[0], y: v1[1], z: v1[2] },
        FVert { x: v2[0], y: v2[1], z: v2[2] },
    ]
}
