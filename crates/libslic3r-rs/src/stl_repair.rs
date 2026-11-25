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

// ===========================================================================
// admesh's whole import repair, as TriangleMesh.cpp:102-197
// (`trianglemesh_repair_on_import`) runs it, then `TriangleMesh::from_stl`
// (TriangleMesh.cpp:204-235): stl_check_facets_exact, stl_check_facets_nearby
// (two iterations), stl_remove_unconnected_facets, stl_fix_normal_directions,
// stl_fix_normal_values, stl_calculate_volume, stl_verify_neighbors, a second
// stl_check_facets_exact when degenerate facets were removed,
// stl_generate_shared_vertices and the its_volume flip.
// The facet soup keeps the normals the readers parsed (stlinit.cpp:261-330).
// ===========================================================================

/// admesh `stl_facet` (stl.h:58-66) without the two attribute bytes.
#[derive(Clone, Copy)]
pub struct StlFacet {
    pub vertex: [[f32; 3]; 3],
    pub normal: [f32; 3],
}

impl StlFacet {
    pub fn new(normal: [f32; 3], v0: [f32; 3], v1: [f32; 3], v2: [f32; 3]) -> StlFacet {
        StlFacet { vertex: [v0, v1, v2], normal }
    }
}

/// admesh `stl_neighbors` (stl.h:83-99); `char` is signed on the pin's arm64 (Apple ABI).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StlNeighbors {
    pub neighbor: [i32; 3],
    pub which_vertex_not: [i8; 3],
}

impl StlNeighbors {
    // stl.h:84-92
    pub fn new() -> Self {
        StlNeighbors { neighbor: [-1; 3], which_vertex_not: [-1; 3] }
    }
    pub fn reset(&mut self) {
        *self = StlNeighbors::new();
    }
    // stl.h:93
    pub fn num_neighbors(&self) -> i32 {
        3 - ((self.neighbor[0] == -1) as i32
            + (self.neighbor[1] == -1) as i32
            + (self.neighbor[2] == -1) as i32)
    }
}

impl Default for StlNeighbors {
    fn default() -> Self {
        Self::new()
    }
}

/// admesh `stl_stats` (stl.h:101-147), the fields the repair reads or writes.
#[derive(Clone, Debug, Default)]
pub struct StlStats {
    pub number_of_facets: u32,
    pub max: [f32; 3],
    pub min: [f32; 3],
    pub size: [f32; 3],
    pub bounding_diameter: f32,
    pub shortest_edge: f32,
    pub volume: f32,
    pub connected_edges: i32,
    pub connected_facets_1_edge: i32,
    pub connected_facets_2_edge: i32,
    pub connected_facets_3_edge: i32,
    pub facets_w_1_bad_edge: i32,
    pub facets_w_2_bad_edge: i32,
    pub facets_w_3_bad_edge: i32,
    pub edges_fixed: i32,
    pub degenerate_facets: i32,
    pub facets_removed: i32,
    pub facets_added: i32,
    pub facets_reversed: i32,
    pub backwards_edges: i32,
    pub normals_fixed: i32,
    pub number_of_parts: i32,
}

/// admesh `stl_file` (stl.h:150-170).
#[derive(Clone, Default)]
pub struct StlFile {
    pub facet_start: Vec<StlFacet>,
    pub neighbors_start: Vec<StlNeighbors>,
    pub stats: StlStats,
}

#[inline]
fn v3(a: [f32; 3]) -> crate::normal_utils::Vec3f {
    crate::normal_utils::Vec3f::new(a[0], a[1], a[2])
}
#[inline]
fn a3(v: crate::normal_utils::Vec3f) -> [f32; 3] {
    [v.x, v.y, v.z]
}
#[inline]
fn vsub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
// Eigen `stl_vertex ==` is value equality (-0 == +0).
#[inline]
fn veq(a: &[f32; 3], b: &[f32; 3]) -> bool {
    a[0] == b[0] && a[1] == b[1] && a[2] == b[2]
}

/// stl.h:402-404 `stl_calculate_normal`: (v1 - v0).cross(v2 - v0) in float.
pub fn stl_calculate_normal(facet: &StlFacet) -> [f32; 3] {
    let u = v3(vsub(facet.vertex[1], facet.vertex[0]));
    let w = v3(vsub(facet.vertex[2], facet.vertex[0]));
    a3(crate::triangle_set_sampling::eigen_cross3f(&u, &w))
}

/// stl.h:405-411 `stl_normalize_vector`: the length in double, the scale `float(1.0 / length)`.
pub fn stl_normalize_vector(normal: &mut [f32; 3]) {
    let x = normal[0] as f64;
    let y = normal[1] as f64;
    let z = normal[2] as f64;
    let length = ((x * x + y * y) + z * z).sqrt();
    if length < 0.000000000001 {
        *normal = [0.0; 3];
    } else {
        let s = (1.0 / length) as f32;
        normal[0] *= s;
        normal[1] *= s;
        normal[2] *= s;
    }
}

/// stlinit.cpp:370-389 `stl_facet_stats`.
pub fn stl_facet_stats(stl: &mut StlFile, facet: &StlFacet, first: &mut bool) {
    if *first {
        stl.stats.min = facet.vertex[0];
        stl.stats.max = facet.vertex[0];
        let d = vsub(facet.vertex[1], facet.vertex[0]);
        let diff = [d[0].abs(), d[1].abs(), d[2].abs()];
        stl.stats.shortest_edge = diff[0].max(diff[1].max(diff[2]));
        *first = false;
    }
    for i in 0..3 {
        for k in 0..3 {
            // Eigen cwiseMin / cwiseMax: std::min(a, b) / std::max(a, b) per coefficient.
            let v = facet.vertex[i][k];
            if v < stl.stats.min[k] {
                stl.stats.min[k] = v;
            }
            if stl.stats.max[k] < v {
                stl.stats.max[k] = v;
            }
        }
    }
}

impl StlFile {
    /// stlinit.cpp:157-336 `stl_read` after the facets are read: the facets go in
    /// order, `stl_facet_stats` over each one (a facet with a NaN vertex is the
    /// zero facet of `stl_allocate` and gets no stats, stlinit.cpp:319-328), then
    /// size and bounding_diameter (stlinit.cpp:334-335).
    /// `facets[i] == None` is a facet the reader skipped as NaN.
    pub fn from_read_facets(facets: Vec<Option<StlFacet>>) -> StlFile {
        let mut stl = StlFile::default();
        // stl.h:118 `float volume = -1.f;`
        stl.stats.volume = -1.0;
        stl.stats.number_of_facets = facets.len() as u32;
        stl.neighbors_start = vec![StlNeighbors::new(); facets.len()];
        let mut first = true;
        stl.facet_start = Vec::with_capacity(facets.len());
        for f in facets {
            match f {
                Some(f) => {
                    stl.facet_start.push(f);
                    stl_facet_stats(&mut stl, &f, &mut first);
                }
                None => stl.facet_start.push(StlFacet {
                    vertex: [[0.0; 3]; 3],
                    normal: [0.0; 3],
                }),
            }
        }
        // stlinit.cpp:334-335
        stl.stats.size = vsub(stl.stats.max, stl.stats.min);
        stl.stats.bounding_diameter =
            crate::triangle_set_sampling::eigen_squared_norm3f(&v3(stl.stats.size)).sqrt();
        stl
    }
}

/// connect.cpp:39-117 `HashEdge` with the `next` link as an index.
#[derive(Clone, Copy)]
struct AdmeshHashEdge {
    key: [u32; 6],
    facet_number: i32,
    which_edge: i32,
}

impl AdmeshHashEdge {
    // connect.cpp:45
    #[inline]
    fn hash(&self, m: i32) -> i32 {
        let k = &self.key;
        // uint32 sum and xor, then `% M` with M converted to unsigned.
        ((((k[0] / 11).wrapping_add(k[1] / 7).wrapping_add(k[2] / 3))
            ^ ((k[3] / 11).wrapping_add(k[4] / 7).wrapping_add(k[5] / 3)))
            % (m as u32)) as i32
    }

    // connect.cpp:54-84
    fn load_exact(&mut self, stats: &mut StlStats, a: &[f32; 3], b: &[f32; 3]) {
        {
            let d = vsub(*a, *b);
            let diff = [d[0].abs(), d[1].abs(), d[2].abs()];
            let max_diff = diff[0].max(diff[1].max(diff[2]));
            // std::min(max_diff, shortest_edge)
            stats.shortest_edge = if stats.shortest_edge < max_diff { stats.shortest_edge } else { max_diff };
        }
        let fa = FVert { x: a[0], y: a[1], z: a[2] };
        let fb = FVert { x: b[0], y: b[1], z: b[2] };
        let (a, b) = if fa.lower(&fb) {
            (a, b)
        } else {
            self.which_edge += 3;
            (b, a)
        };
        let k = [a[0], a[1], a[2], b[0], b[1], b[2]];
        for i in 0..6 {
            let bits = k[i].to_bits();
            // connect.cpp:74-83 negative zero to positive zero
            self.key[i] = if bits == 0x8000_0000 { 0 } else { bits };
        }
    }

    // connect.cpp:86-112
    fn load_nearby(&mut self, stats: &StlStats, a: &[f32; 3], b: &[f32; 3], tolerance: f32) -> bool {
        let cell = |p: &[f32; 3]| -> [i32; 3] {
            [
                ((p[0] - stats.min[0]) / tolerance) as i32,
                ((p[1] - stats.min[1]) / tolerance) as i32,
                ((p[2] - stats.min[2]) / tolerance) as i32,
            ]
        };
        let vertex1 = cell(a);
        let vertex2 = cell(b);
        if vertex1 == vertex2 {
            return false;
        }
        let lower = if vertex1[0] != vertex2[0] {
            vertex1[0] < vertex2[0]
        } else if vertex1[1] != vertex2[1] {
            vertex1[1] < vertex2[1]
        } else {
            vertex1[2] < vertex2[2]
        };
        let (lo, hi) = if lower {
            (vertex1, vertex2)
        } else {
            self.which_edge += 3;
            (vertex2, vertex1)
        };
        self.key = [lo[0] as u32, lo[1] as u32, lo[2] as u32, hi[0] as u32, hi[1] as u32, hi[2] as u32];
        true
    }
}

/// connect.cpp:119-260 `HashTableEdges`: chains as vectors in list order; a match
/// removes the first equal edge of the chain (head first), a miss appends.
struct HashTableEdges {
    heads: Vec<Vec<AdmeshHashEdge>>,
    m: i32,
}

#[derive(Clone, Copy)]
enum MatchKind {
    Exact,
    Nearby,
}

impl HashTableEdges {
    // connect.cpp:120-127
    fn new(number_of_faces: usize) -> Self {
        let m = hash_size_from_nr_faces_admesh(number_of_faces);
        HashTableEdges { heads: vec![Vec::new(); m as usize], m }
    }

    // connect.cpp:172-237
    fn insert_edge(&mut self, stl: &mut StlFile, edge: &AdmeshHashEdge, kind: MatchKind) {
        let chain_number = edge.hash(self.m) as usize;
        let chain = &mut self.heads[chain_number];
        // connect.cpp:240-243 edges_equal
        if let Some(pos) = chain
            .iter()
            .position(|e| edge.facet_number != e.facet_number && edge.key == e.key)
        {
            let link = chain.remove(pos);
            match kind {
                MatchKind::Exact => admesh_record_neighbors(stl, edge, &link),
                MatchKind::Nearby => match_neighbors_nearby(stl, edge, &link),
            }
        } else {
            chain.push(*edge);
        }
    }
}

// connect.cpp:160-167: std::upper_bound(primes, nr_faces * 3 * 2 - 1), size_t arithmetic.
fn hash_size_from_nr_faces_admesh(nr_faces: usize) -> i32 {
    const PRIMES: [u64; 15] = [
        98317, 196613, 393241, 786433, 1572869, 3145739, 6291469, 12582917, 25165843, 50331653,
        100663319, 201326611, 402653189, 805306457, 1610612741,
    ];
    let target = (nr_faces as u64).wrapping_mul(6).wrapping_sub(1);
    for &p in &PRIMES {
        if p > target {
            return p as i32;
        }
    }
    *PRIMES.last().unwrap() as i32
}

// connect.cpp:246-279 record_neighbors
fn admesh_record_neighbors(stl: &mut StlFile, edge_a: &AdmeshHashEdge, edge_b: &AdmeshHashEdge) {
    let af = edge_a.facet_number as usize;
    let bf = edge_b.facet_number as usize;
    let ae = (edge_a.which_edge % 3) as usize;
    let be = (edge_b.which_edge % 3) as usize;
    stl.neighbors_start[af].neighbor[ae] = edge_b.facet_number;
    stl.neighbors_start[af].which_vertex_not[ae] = ((edge_b.which_edge + 2) % 3) as i8;
    stl.neighbors_start[bf].neighbor[be] = edge_a.facet_number;
    stl.neighbors_start[bf].which_vertex_not[be] = ((edge_a.which_edge + 2) % 3) as i8;
    if (edge_a.which_edge < 3 && edge_b.which_edge < 3) || (edge_a.which_edge > 2 && edge_b.which_edge > 2) {
        stl.neighbors_start[af].which_vertex_not[ae] += 3;
        stl.neighbors_start[bf].which_vertex_not[be] += 3;
    }
    stl.stats.connected_edges += 2;
    for f in [af, bf] {
        match stl.neighbors_start[f].num_neighbors() {
            1 => stl.stats.connected_facets_1_edge += 1,
            2 => stl.stats.connected_facets_2_edge += 1,
            3 => stl.stats.connected_facets_3_edge += 1,
            _ => {}
        }
    }
}

// connect.cpp:341-410 change_vertices
fn change_vertices(stl: &mut StlFile, mut facet_num: i32, mut vnot: i32, new_vertex: [f32; 3]) {
    let first_facet = facet_num;
    let mut direction = false;
    loop {
        let pivot_vertex;
        let next_edge;
        if vnot > 2 {
            if direction {
                pivot_vertex = (vnot + 1) % 3;
                next_edge = vnot % 3;
            } else {
                pivot_vertex = (vnot + 2) % 3;
                next_edge = pivot_vertex;
            }
            direction = !direction;
        } else if direction {
            pivot_vertex = (vnot + 2) % 3;
            next_edge = pivot_vertex;
        } else {
            pivot_vertex = (vnot + 1) % 3;
            next_edge = vnot;
        }
        stl.facet_start[facet_num as usize].vertex[pivot_vertex as usize] = new_vertex;
        let n = stl.neighbors_start[facet_num as usize];
        vnot = n.which_vertex_not[next_edge as usize] as i32;
        facet_num = n.neighbor[next_edge as usize];
        if facet_num == -1 {
            break;
        }
        if facet_num == first_facet {
            // connect.cpp:403-407: back to the beginning (a mobius part), logged.
            return;
        }
    }
}

// connect.cpp:281-430 match_neighbors_nearby
fn match_neighbors_nearby(stl: &mut StlFile, edge_a: &AdmeshHashEdge, edge_b: &AdmeshHashEdge) {
    admesh_record_neighbors(stl, edge_a, edge_b);
    let mut facet1 = -1;
    let mut facet2 = -1;
    let mut vertex1 = 0;
    let mut vertex2 = 0;
    let mut new_vertex1 = [0.0f32; 3];
    let mut new_vertex2 = [0.0f32; 3];
    {
        let (v1a, v2a) = if edge_a.which_edge < 3 {
            (edge_a.which_edge, (edge_a.which_edge + 1) % 3)
        } else {
            ((edge_a.which_edge + 1) % 3, edge_a.which_edge % 3)
        };
        let (v1b, v2b) = if edge_b.which_edge < 3 {
            (edge_b.which_edge, (edge_b.which_edge + 1) % 3)
        } else {
            ((edge_b.which_edge + 1) % 3, edge_b.which_edge % 3)
        };
        let fa = edge_a.facet_number as usize;
        let fb = edge_b.facet_number as usize;
        if !veq(&stl.facet_start[fa].vertex[v1a as usize], &stl.facet_start[fb].vertex[v1b as usize]) {
            if stl.neighbors_start[fa].neighbor[v1a as usize] == -1
                && stl.neighbors_start[fa].neighbor[((v1a + 2) % 3) as usize] == -1
            {
                facet1 = edge_a.facet_number;
                vertex1 = v1a;
                new_vertex1 = stl.facet_start[fb].vertex[v1b as usize];
            } else {
                facet1 = edge_b.facet_number;
                vertex1 = v1b;
                new_vertex1 = stl.facet_start[fa].vertex[v1a as usize];
            }
        }
        if !veq(&stl.facet_start[fa].vertex[v2a as usize], &stl.facet_start[fb].vertex[v2b as usize]) {
            if stl.neighbors_start[fa].neighbor[v2a as usize] == -1
                && stl.neighbors_start[fa].neighbor[((v2a + 2) % 3) as usize] == -1
            {
                facet2 = edge_a.facet_number;
                vertex2 = v2a;
                new_vertex2 = stl.facet_start[fb].vertex[v2b as usize];
            } else {
                facet2 = edge_b.facet_number;
                vertex2 = v2b;
                new_vertex2 = stl.facet_start[fa].vertex[v2a as usize];
            }
        }
    }
    if facet1 != -1 {
        let mut vnot1 = if facet1 == edge_a.facet_number {
            (edge_a.which_edge + 2) % 3
        } else {
            (edge_b.which_edge + 2) % 3
        };
        if (vnot1 + 2) % 3 == vertex1 {
            vnot1 += 3;
        }
        change_vertices(stl, facet1, vnot1, new_vertex1);
    }
    if facet2 != -1 {
        let mut vnot2 = if facet2 == edge_a.facet_number {
            (edge_a.which_edge + 2) % 3
        } else {
            (edge_b.which_edge + 2) % 3
        };
        if (vnot2 + 2) % 3 == vertex2 {
            vnot2 += 3;
        }
        change_vertices(stl, facet2, vnot2, new_vertex2);
    }
    stl.stats.edges_fixed += 2;
}

/// connect.cpp:435-480 `stl_check_facets_exact`.
pub fn stl_check_facets_exact(stl: &mut StlFile) {
    stl.stats.connected_edges = 0;
    stl.stats.connected_facets_1_edge = 0;
    stl.stats.connected_facets_2_edge = 0;
    stl.stats.connected_facets_3_edge = 0;
    let mut i = 0u32;
    while i < stl.stats.number_of_facets {
        let f = &stl.facet_start[i as usize];
        if veq(&f.vertex[0], &f.vertex[1]) || veq(&f.vertex[1], &f.vertex[2]) || veq(&f.vertex[0], &f.vertex[2]) {
            stl.stats.number_of_facets -= 1;
            stl.facet_start[i as usize] = stl.facet_start[stl.stats.number_of_facets as usize];
            stl.facet_start.pop();
            stl.neighbors_start.pop();
            stl.stats.facets_removed += 1;
            stl.stats.degenerate_facets += 1;
        } else {
            i += 1;
        }
    }
    let mut hash_table = HashTableEdges::new(stl.stats.number_of_facets as usize);
    for n in stl.neighbors_start.iter_mut() {
        n.reset();
    }
    for i in 0..stl.stats.number_of_facets {
        let facet = stl.facet_start[i as usize];
        for j in 0..3 {
            let mut edge = AdmeshHashEdge { key: [0; 6], facet_number: i as i32, which_edge: j as i32 };
            edge.load_exact(&mut stl.stats, &facet.vertex[j], &facet.vertex[(j + 1) % 3]);
            hash_table.insert_edge(stl, &edge, MatchKind::Exact);
        }
    }
}

/// connect.cpp:482-505 `stl_check_facets_nearby`.
pub fn stl_check_facets_nearby(stl: &mut StlFile, tolerance: f32) {
    if stl.stats.connected_facets_3_edge == stl.stats.number_of_facets as i32 {
        return;
    }
    let mut hash_table = HashTableEdges::new(stl.stats.number_of_facets as usize);
    for i in 0..stl.stats.number_of_facets {
        // connect.cpp:493 the facet is copied before its edges are inserted.
        let facet = stl.facet_start[i as usize];
        for j in 0..3 {
            if stl.neighbors_start[i as usize].neighbor[j] == -1 {
                let mut edge = AdmeshHashEdge { key: [0; 6], facet_number: i as i32, which_edge: j as i32 };
                let stats = stl.stats.clone();
                if edge.load_nearby(&stats, &facet.vertex[j], &facet.vertex[(j + 1) % 3], tolerance) {
                    hash_table.insert_edge(stl, &edge, MatchKind::Nearby);
                }
            }
        }
    }
}

// connect.cpp:512-545 remove_facet
fn remove_facet(stl: &mut StlFile, facet_number: i32) {
    stl.stats.facets_removed += 1;
    let nn = stl.neighbors_start[facet_number as usize].num_neighbors();
    // switch with fall through
    if nn >= 3 {
        stl.stats.connected_facets_3_edge -= 1;
    }
    if nn >= 2 {
        stl.stats.connected_facets_2_edge -= 1;
    }
    if nn >= 1 {
        stl.stats.connected_facets_1_edge -= 1;
    }
    stl.stats.number_of_facets -= 1;
    if facet_number < stl.stats.number_of_facets as i32 {
        let last = stl.stats.number_of_facets as usize;
        stl.facet_start[facet_number as usize] = stl.facet_start[last];
        stl.neighbors_start[facet_number as usize] = stl.neighbors_start[last];
        // `neighbors` is a reference to neighbors_start[facet_number] at the pin: read it live.
        for i in 0..3 {
            let neighbors = stl.neighbors_start[facet_number as usize];
            if neighbors.neighbor[i] != -1 {
                let o = neighbors.neighbor[i] as usize;
                let k = ((neighbors.which_vertex_not[i] as i32 + 1) % 3) as usize;
                if stl.neighbors_start[o].neighbor[k] != stl.stats.number_of_facets as i32 {
                    // connect.cpp:536-539: logged, returns before the pop_backs.
                    return;
                }
                stl.neighbors_start[o].neighbor[k] = facet_number;
            }
        }
    }
    stl.facet_start.pop();
    stl.neighbors_start.pop();
}

// connect.cpp:548-554 update_connects_remove_1
fn update_connects_remove_1(stl: &mut StlFile, facet_num: i32) {
    match stl.neighbors_start[facet_num as usize].num_neighbors() {
        1 => stl.stats.connected_facets_1_edge -= 1,
        2 => stl.stats.connected_facets_2_edge -= 1,
        3 => stl.stats.connected_facets_3_edge -= 1,
        _ => {}
    }
}

// connect.cpp:547-627 remove_degenerate
fn remove_degenerate(stl: &mut StlFile, facet: i32) {
    let f = stl.facet_start[facet as usize];
    let edge_to_collapse;
    if veq(&f.vertex[0], &f.vertex[1]) {
        if veq(&f.vertex[1], &f.vertex[2]) {
            let nbr = stl.neighbors_start[facet as usize].neighbor;
            edge_to_collapse = if nbr[0] == -1 { 0 } else if nbr[1] == -1 { 1 } else { 2 };
        } else {
            edge_to_collapse = 0;
        }
    } else if veq(&f.vertex[1], &f.vertex[2]) {
        edge_to_collapse = 1;
    } else if veq(&f.vertex[2], &f.vertex[0]) {
        edge_to_collapse = 2;
    } else {
        return;
    }
    let edge = [(edge_to_collapse + 1) % 3, (edge_to_collapse + 2) % 3, edge_to_collapse];
    let ns = stl.neighbors_start[facet as usize];
    let neighbor = [ns.neighbor[edge[0]], ns.neighbor[edge[1]], ns.neighbor[edge[2]]];
    let mut vnot = [
        ns.which_vertex_not[edge[0]] as i32,
        ns.which_vertex_not[edge[1]] as i32,
        ns.which_vertex_not[edge[2]] as i32,
    ];
    if neighbor[0] == -1 && neighbor[1] != -1 {
        update_connects_remove_1(stl, neighbor[1]);
    }
    if neighbor[1] == -1 && neighbor[0] != -1 {
        update_connects_remove_1(stl, neighbor[0]);
    }
    if neighbor[0] >= 0 {
        if neighbor[1] >= 0 {
            if vnot[0] > 2 {
                if vnot[1] > 2 {
                    vnot[0] -= 3;
                    vnot[1] -= 3;
                } else {
                    vnot[1] += 3;
                }
            } else if vnot[1] > 2 {
                vnot[0] += 3;
            }
        }
        let k = ((vnot[0] + 1) % 3) as usize;
        stl.neighbors_start[neighbor[0] as usize].neighbor[k] =
            if neighbor[0] == neighbor[1] { -1 } else { neighbor[1] };
        stl.neighbors_start[neighbor[0] as usize].which_vertex_not[k] = vnot[1] as i8;
    }
    if neighbor[1] >= 0 {
        let k = ((vnot[1] + 1) % 3) as usize;
        stl.neighbors_start[neighbor[1] as usize].neighbor[k] =
            if neighbor[0] == neighbor[1] { -1 } else { neighbor[0] };
        stl.neighbors_start[neighbor[1] as usize].which_vertex_not[k] = vnot[0] as i8;
    }
    if neighbor[2] >= 0 {
        update_connects_remove_1(stl, neighbor[2]);
        let k = ((vnot[2] + 1) % 3) as usize;
        stl.neighbors_start[neighbor[2] as usize].neighbor[k] = -1;
    }
    remove_facet(stl, facet);
}

/// connect.cpp:507-650 `stl_remove_unconnected_facets`.
pub fn stl_remove_unconnected_facets(stl: &mut StlFile) {
    let mut i = 0u32;
    while i < stl.stats.number_of_facets {
        let f = &stl.facet_start[i as usize];
        if veq(&f.vertex[0], &f.vertex[1]) || veq(&f.vertex[0], &f.vertex[2]) || veq(&f.vertex[1], &f.vertex[2]) {
            remove_degenerate(stl, i as i32);
        } else {
            i += 1;
        }
    }
    if stl.stats.connected_facets_1_edge < stl.stats.number_of_facets as i32 {
        let mut i = 0u32;
        while i < stl.stats.number_of_facets {
            if stl.neighbors_start[i as usize].num_neighbors() == 0 {
                remove_facet(stl, i as i32);
            } else {
                i += 1;
            }
        }
    }
}

// normals.cpp:33-68 reverse_facet
fn reverse_facet(stl: &mut StlFile, facet_num: i32) {
    stl.stats.facets_reversed += 1;
    let fi = facet_num as usize;
    let neighbor = stl.neighbors_start[fi].neighbor;
    let vnot = stl.neighbors_start[fi].which_vertex_not;
    stl.facet_start[fi].vertex.swap(0, 1);
    for (i, add) in [(0usize, 3i32), (1, 4), (2, 2)] {
        if neighbor[i] != -1 {
            let k = ((vnot[i] as i32 + 1) % 3) as usize;
            let w = &mut stl.neighbors_start[neighbor[i] as usize].which_vertex_not[k];
            *w = ((*w as i32 + add) % 6) as i8;
        }
    }
    let n = &mut stl.neighbors_start[fi];
    n.neighbor[1] = neighbor[2];
    n.neighbor[2] = neighbor[1];
    n.which_vertex_not[1] = vnot[2];
    n.which_vertex_not[2] = vnot[1];
    for k in 0..3 {
        n.which_vertex_not[k] = ((n.which_vertex_not[k] as i32 + 3) % 6) as i8;
    }
}

// normals.cpp:71-118 check_normal_vector; true if the normal was flipped.
fn check_normal_vector(stl: &mut StlFile, facet_num: i32, normal_fix_flag: bool) -> bool {
    let fi = facet_num as usize;
    let mut normal = stl_calculate_normal(&stl.facet_start[fi]);
    stl_normalize_vector(&mut normal);
    let eps = 0.001f32;
    let within = |a: [f32; 3], b: [f32; 3]| -> bool {
        (a[0] - b[0]).abs() < eps && (a[1] - b[1]).abs() < eps && (a[2] - b[2]).abs() < eps
    };
    if within(normal, stl.facet_start[fi].normal) {
        stl.facet_start[fi].normal = normal;
        return false;
    }
    let mut test_norm = stl.facet_start[fi].normal;
    stl_normalize_vector(&mut test_norm);
    if within(normal, test_norm) {
        if normal_fix_flag {
            stl.facet_start[fi].normal = normal;
            stl.stats.normals_fixed += 1;
        }
        return false;
    }
    test_norm = [test_norm[0] * -1.0, test_norm[1] * -1.0, test_norm[2] * -1.0];
    if within(normal, test_norm) {
        if normal_fix_flag {
            stl.facet_start[fi].normal = normal;
            stl.stats.normals_fixed += 1;
        }
        return true;
    }
    if normal_fix_flag {
        stl.facet_start[fi].normal = normal;
        stl.stats.normals_fixed += 1;
    }
    false
}

/// normals.cpp:120-219 `stl_fix_normal_directions`. The pool list is a stack
/// (nodes are added to the front and taken from the front).
pub fn stl_fix_normal_directions(stl: &mut StlFile) {
    if stl.stats.number_of_facets == 0 {
        return;
    }
    let nf = stl.stats.number_of_facets as usize;
    let mut list: Vec<i32> = Vec::new(); // top = last
    let mut norm_sw = vec![0u8; nf];
    let mut reversed_ids: Vec<i32> = Vec::with_capacity(nf);
    let mut facet_num: i32 = 0;
    if check_normal_vector(stl, 0, false) {
        reverse_facet(stl, 0);
        reversed_ids.push(0);
    }
    norm_sw[facet_num as usize] = 1;
    let mut checked: i32 = 1;
    loop {
        let mut force_exit = false;
        for j in 0..3 {
            if stl.neighbors_start[facet_num as usize].which_vertex_not[j] > 2 {
                let nb = stl.neighbors_start[facet_num as usize].neighbor[j];
                if nb != -1 {
                    if norm_sw[nb as usize] == 1 {
                        for id in (0..reversed_ids.len()).rev() {
                            reverse_facet(stl, reversed_ids[id]);
                        }
                        force_exit = true;
                        break;
                    }
                    reverse_facet(stl, nb);
                    reversed_ids.push(nb);
                }
            }
            let nb = stl.neighbors_start[facet_num as usize].neighbor[j];
            if nb != -1 && norm_sw[nb as usize] != 1 {
                list.push(nb);
            }
        }
        if force_exit {
            break;
        }
        if let Some(top) = list.pop() {
            facet_num = top;
            if norm_sw[facet_num as usize] != 1 {
                norm_sw[facet_num as usize] = 1;
                checked += 1;
            }
        } else {
            stl.stats.number_of_parts += 1;
            if checked >= stl.stats.number_of_facets as i32 {
                break;
            }
            for i in 0..stl.stats.number_of_facets {
                if norm_sw[i as usize] == 0 {
                    facet_num = i as i32;
                    if check_normal_vector(stl, i as i32, false) {
                        reverse_facet(stl, i as i32);
                        reversed_ids.push(i as i32);
                    }
                    norm_sw[facet_num as usize] = 1;
                    checked += 1;
                    break;
                }
            }
        }
    }
}

/// normals.cpp:221-225 `stl_fix_normal_values`.
pub fn stl_fix_normal_values(stl: &mut StlFile) {
    for i in 0..stl.stats.number_of_facets {
        check_normal_vector(stl, i as i32, true);
    }
}

/// normals.cpp:227-237 `stl_reverse_all_facets`.
pub fn stl_reverse_all_facets(stl: &mut StlFile) {
    for i in 0..stl.stats.number_of_facets {
        reverse_facet(stl, i as i32);
        let mut normal = stl_calculate_normal(&stl.facet_start[i as usize]);
        stl_normalize_vector(&mut normal);
        stl.facet_start[i as usize].normal = normal;
    }
}

/// util.cpp:248-273 `get_area`: the cross products in double (each component one
/// contracted expression, `a*b - c*d` = fma(a, b, -(c*d))), summed into a float normal.
pub fn get_area(facet: &StlFacet) -> f32 {
    let v = &facet.vertex;
    let mut cross = [[0.0f64; 3]; 3];
    for i in 0..3 {
        let a = v[i];
        let b = v[(i + 1) % 3];
        let (a0, a1, a2) = (a[0] as f64, a[1] as f64, a[2] as f64);
        let (b0, b1, b2) = (b[0] as f64, b[1] as f64, b[2] as f64);
        cross[i][0] = a1.mul_add(b2, -(a2 * b1));
        cross[i][1] = a2.mul_add(b0, -(a0 * b2));
        cross[i][2] = a0.mul_add(b1, -(a1 * b0));
    }
    let sum = [
        (cross[0][0] + cross[1][0] + cross[2][0]) as f32,
        (cross[0][1] + cross[1][1] + cross[2][1]) as f32,
        (cross[0][2] + cross[1][2] + cross[2][2]) as f32,
    ];
    let mut n = stl_calculate_normal(facet);
    stl_normalize_vector(&mut n);
    0.5f32 * crate::triangle_set_sampling::eigen_dot3f(&v3(n), &v3(sum))
}

// util.cpp:275-287 get_volume
fn get_volume(stl: &StlFile) -> f32 {
    let p0 = stl.facet_start[0].vertex[0];
    let mut volume = 0.0f32;
    for i in 0..stl.stats.number_of_facets as usize {
        let f = &stl.facet_start[i];
        let height = crate::triangle_set_sampling::eigen_dot3f(&v3(f.normal), &v3(vsub(f.vertex[0], p0)));
        let area = get_area(f);
        volume += (area * height) / 3.0f32;
    }
    volume
}

/// util.cpp:289-296 `stl_calculate_volume`.
pub fn stl_calculate_volume(stl: &mut StlFile) {
    stl.stats.volume = get_volume(stl);
    if (stl.stats.volume as f64) < 0.0 {
        stl_reverse_all_facets(stl);
        stl.stats.volume = -stl.stats.volume;
    }
}

/// util.cpp:32-67 `stl_verify_neighbors`: counts the backwards edges (the
/// mismatches are only logged at the pin).
pub fn stl_verify_neighbors(stl: &mut StlFile) {
    stl.stats.backwards_edges = 0;
    for i in 0..stl.stats.number_of_facets as usize {
        for j in 0..3 {
            let neighbor = stl.neighbors_start[i].neighbor[j];
            if neighbor == -1 {
                continue;
            }
            let vnot = stl.neighbors_start[i].which_vertex_not[j];
            if vnot >= 3 {
                stl.stats.backwards_edges += 1;
            }
        }
    }
}

/// TriangleMesh.cpp:102-197, in triangle_mesh.rs (the file paired with TriangleMesh.cpp).
pub use crate::triangle_mesh::trianglemesh_repair_on_import;

/// shared.cpp:34-122 `stl_generate_shared_vertices`.
pub fn stl_generate_shared_vertices(stl: &StlFile) -> (Vec<[f32; 3]>, Vec<[i32; 3]>) {
    let nf = stl.stats.number_of_facets as usize;
    let mut indices: Vec<[i32; 3]> = vec![[-1, -1, -1]; nf];
    let mut vertices: Vec<[f32; 3]> = Vec::with_capacity(nf / 2);
    let mut fan_traversal_stamp: u32 = 0;
    let mut visited: Vec<u32> = vec![0; nf];
    for facet_idx in 0..nf {
        for j in 0..3 {
            if indices[facet_idx][j] != -1 {
                continue;
            }
            vertices.push(stl.facet_start[facet_idx].vertex[j]);
            let mut facet_in_fan_idx = facet_idx as i32;
            let mut edge_direction = false;
            let mut traversal_reversed = false;
            let mut vnot: i32 = ((j + 2) % 3) as i32;
            fan_traversal_stamp = fan_traversal_stamp.wrapping_add(1);
            loop {
                let next_edge;
                let pivot_vertex;
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
                indices[facet_in_fan_idx as usize][pivot_vertex as usize] = (vertices.len() - 1) as i32;
                visited[facet_in_fan_idx as usize] = fan_traversal_stamp;
                let next_facet = stl.neighbors_start[facet_in_fan_idx as usize].neighbor[next_edge as usize];
                if next_facet == -1 {
                    if traversal_reversed {
                        break;
                    }
                    edge_direction = true;
                    vnot = ((j + 1) % 3) as i32;
                    traversal_reversed = true;
                    facet_in_fan_idx = facet_idx as i32;
                } else if next_facet == facet_idx as i32 {
                    break;
                } else if next_facet >= nf as i32 {
                    break;
                } else if visited[next_facet as usize] == fan_traversal_stamp {
                    break;
                } else {
                    vnot = stl.neighbors_start[facet_in_fan_idx as usize].which_vertex_not[next_edge as usize] as i32;
                    facet_in_fan_idx = next_facet;
                }
            }
        }
    }
    (vertices, indices)
}

/// TriangleMesh.cpp:204-235 `TriangleMesh::from_stl(stl, repair)` (triangle_mesh.rs), as vertices
/// and index triples.
pub fn from_stl(stl: &mut StlFile, repair: bool) -> (Vec<Point3F>, Vec<[u32; 3]>) {
    let mut mesh = crate::triangle_mesh::TriangleMesh::new();
    mesh.from_stl(stl, repair);
    (
        mesh.vertices().to_vec(),
        mesh.indices().iter().map(|t| [t.indices[0] as u32, t.indices[1] as u32, t.indices[2] as u32]).collect(),
    )
}
