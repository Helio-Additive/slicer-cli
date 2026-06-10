//! Faithful port of libslic3r/SLA/Clustering.{hpp,cpp}
//!
//! C++ Reference:
//! - SLA/Clustering.hpp
//! - SLA/Clustering.cpp
//!
//! NOTE on the backing store: the C++ implementation uses a
//! `boost::geometry::index::rtree<PointIndexEl, bgi::rstar<16, 4>>`
//! (Clustering.cpp:10). Boost is a native header-only dependency that we do
//! not pull in (wasm-safe); the Rust port stores the elements in a flat `Vec`
//! and answers the same queries by scanning — exactly like the existing
//! `sla::spat_index::PointIndex` port. The supported query semantics
//! (k-nearest by Euclidean distance, arbitrary predicate filtering) are
//! identical; only the iteration/result order of tied elements may differ
//! from the rtree's internal order, which the C++ API leaves unspecified.
//!
//! Function-name mapping for the three C++ `cluster` overloads (Rust has no
//! overloading):
//! - Clustering.cpp:19  (anonymous namespace, rtree + query fn) -> `cluster_impl`
//! - Clustering.cpp:93  (indices + pointfn + dist)              -> `cluster`
//! - Clustering.cpp:113 (indices + pointfn + predicate)         -> `cluster_by_predicate`
//! - Clustering.cpp:136 (Eigen::MatrixXd of points)             -> `cluster_points`

use crate::geometry::Vec3d;
use crate::sla::spat_index::PointIndexEl;

// Clustering.hpp:11 — using ClusterEl = std::vector<unsigned>;
pub type ClusterEl = Vec<u32>;
// Clustering.hpp:12 — using ClusteredPoints = std::vector<ClusterEl>;
pub type ClusteredPoints = Vec<ClusterEl>;

// Clustering.cpp:9-10 —
// namespace bgi = boost::geometry::index;
// using Index3D = bgi::rtree< PointIndexEl, bgi::rstar<16, 4> /* ? */ >;
// (Vec-backed in the Rust port; see module note above.)
type Index3D = Vec<PointIndexEl>;

// ---------------------------------------------------------------------------
// Clustering.cpp:12 — namespace { (anonymous namespace: file-private items)
// ---------------------------------------------------------------------------

// Clustering.cpp:14-17 —
// bool cmp_ptidx_elements(const PointIndexEl& e1, const PointIndexEl& e2)
fn cmp_ptidx_elements(e1: &PointIndexEl, e2: &PointIndexEl) -> bool {
    // Clustering.cpp:16
    e1.1 < e2.1
}

/// `std::set_difference(tmp, cluster, back_inserter(newpts), cmp_ptidx_elements)`
/// helper used at Clustering.cpp:37-39. Both inputs must be sorted by
/// `cmp_ptidx_elements`; mirrors the standard-library merge algorithm exactly
/// (including its handling of duplicates).
fn set_difference_by_ptidx(first1: &[PointIndexEl], first2: &[PointIndexEl]) -> Vec<PointIndexEl> {
    let mut result: Vec<PointIndexEl> = Vec::new();
    let mut i1 = 0usize;
    let mut i2 = 0usize;
    while i1 < first1.len() {
        if i2 == first2.len() {
            result.extend_from_slice(&first1[i1..]);
            break;
        }
        if cmp_ptidx_elements(&first1[i1], &first2[i2]) {
            result.push(first1[i1].clone());
            i1 += 1;
        } else {
            if !cmp_ptidx_elements(&first2[i2], &first1[i1]) {
                i1 += 1;
            }
            i2 += 1;
        }
    }
    result
}

/// `bgi::nearest(p, k)` query against the Vec-backed `Index3D`
/// (Clustering.cpp:79-82): the boost rtree k-nearest query yields elements in
/// order of increasing Euclidean distance from `el`; reproduced by sorting.
fn index3d_query_nearest(sindex: &Index3D, el: &Vec3d, k: u32) -> Vec<PointIndexEl> {
    let mut ret: Vec<PointIndexEl> = sindex.clone();
    let dist_sq = |p: &Vec3d| -> f64 { (*p - *el).length_squared() };
    ret.sort_by(|a, b| {
        dist_sq(&a.0)
            .partial_cmp(&dist_sq(&b.0))
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    ret.truncate(k as usize);
    ret
}

/// `rtree::remove(value)` against the Vec-backed `Index3D`
/// (Clustering.cpp:58): removes a single element equal to `el`.
fn index3d_remove(sindex: &mut Index3D, el: &PointIndexEl) {
    if let Some(pos) = sindex.iter().position(|e| e == el) {
        sindex.remove(pos);
    }
}

// Clustering.cpp:26-50 —
// Recursive function for visiting all the points in a given distance to
// each other
// std::function<void(Elems&, Elems&)> group =
//     [&sindex, &group, max_points, qfn](Elems& pts, Elems& cluster)
// (The C++ recursive lambda becomes a recursive free function in Rust; the
// captures `sindex`, `max_points` and `qfn` are threaded through explicitly.)
fn group(
    sindex: &Index3D,
    max_points: u32,
    qfn: &dyn Fn(&Index3D, &PointIndexEl) -> Vec<PointIndexEl>,
    pts: &[PointIndexEl],
    cluster: &mut Vec<PointIndexEl>,
) {
    // Clustering.cpp:31
    for p in pts {
        // Clustering.cpp:32
        let mut tmp: Vec<PointIndexEl> = qfn(sindex, p);

        // Clustering.cpp:34
        tmp.sort_by(|a, b| a.1.cmp(&b.1));

        // Clustering.cpp:36-39
        let newpts: Vec<PointIndexEl> = set_difference_by_ptidx(&tmp, cluster);

        // Clustering.cpp:41-42
        let c: i32 = if max_points != 0 && newpts.len() + cluster.len() > max_points as usize {
            (max_points as usize - cluster.len()) as i32
        } else {
            newpts.len() as i32
        };

        // Clustering.cpp:44
        cluster.extend_from_slice(&newpts[..c as usize]);
        // Clustering.cpp:45
        cluster.sort_by(|a, b| a.1.cmp(&b.1));

        // Clustering.cpp:47-48
        if !newpts.is_empty() && (max_points == 0 || cluster.len() < max_points as usize) {
            group(sindex, max_points, qfn, &newpts, cluster);
        }
    }
}

// Clustering.cpp:19-71 —
// ClusteredPoints cluster(Index3D &sindex,
//                         unsigned max_points,
//                         std::function<std::vector<PointIndexEl>(
//                             const Index3D &, const PointIndexEl &)> qfn)
fn cluster_impl(
    sindex: &mut Index3D,
    max_points: u32,
    qfn: &dyn Fn(&Index3D, &PointIndexEl) -> Vec<PointIndexEl>,
) -> ClusteredPoints {
    // Clustering.cpp:24 — using Elems = std::vector<PointIndexEl>;
    // (Clustering.cpp:28-50 — the recursive `group` lambda is the free
    // function `group` above.)

    // Clustering.cpp:52
    let mut clusters: Vec<Vec<PointIndexEl>> = Vec::new();
    // Clustering.cpp:53 — for(auto it = sindex.begin(); it != sindex.end();)
    loop {
        let first = match sindex.first() {
            Some(el) => el.clone(),
            None => break,
        };
        // Clustering.cpp:54
        let mut cluster: Vec<PointIndexEl> = Vec::new();
        // Clustering.cpp:55
        let pts: Vec<PointIndexEl> = vec![first];
        // Clustering.cpp:56
        group(sindex, max_points, qfn, &pts, &mut cluster);

        // Clustering.cpp:58
        for c in &cluster {
            index3d_remove(sindex, c);
        }
        // Clustering.cpp:59 — it = sindex.begin(); (loop restart)

        // Clustering.cpp:61
        clusters.push(cluster);
    }

    // Clustering.cpp:64
    let mut result: ClusteredPoints = Vec::new();
    // Clustering.cpp:65-68
    for cluster in &clusters {
        result.push(Vec::new());
        for c in cluster {
            result.last_mut().unwrap().push(c.1);
        }
    }

    // Clustering.cpp:70
    result
}

// Clustering.cpp:73-88 —
// std::vector<PointIndexEl> distance_queryfn(const Index3D& sindex,
//                                            const PointIndexEl& p,
//                                            double dist,
//                                            unsigned max_points)
fn distance_queryfn(
    sindex: &Index3D,
    p: &PointIndexEl,
    dist: f64,
    max_points: u32,
) -> Vec<PointIndexEl> {
    // Clustering.cpp:78-82 — tmp.reserve(max_points);
    // sindex.query(bgi::nearest(p.first, max_points), back_inserter(tmp));
    let mut tmp: Vec<PointIndexEl> = index3d_query_nearest(sindex, &p.0, max_points);

    // Clustering.cpp:84-85 —
    // for(auto it = tmp.begin(); it < tmp.end(); ++it)
    //     if((p.first - it->first).norm() > dist) it = tmp.erase(it);
    // NOTE: faithfully reproduces the C++ iterator behavior where
    // `it = tmp.erase(it)` followed by the loop's `++it` SKIPS the element
    // immediately after each erased one.
    let mut i = 0usize;
    while i < tmp.len() {
        if (p.0 - tmp[i].0).norm() > dist {
            tmp.remove(i);
        }
        i += 1;
    }

    // Clustering.cpp:87
    tmp
}

// Clustering.cpp:90 — } // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Clustering.cpp:92-110 —
// Clustering a set of points by the given criteria
// ClusteredPoints cluster(
//     const std::vector<unsigned>& indices,
//     std::function<Vec3d(unsigned)> pointfn,
//     double dist,
//     unsigned max_points)
pub fn cluster(
    indices: &[u32],
    pointfn: &dyn Fn(u32) -> Vec3d,
    dist: f64,
    max_points: u32,
) -> ClusteredPoints {
    // Clustering.cpp:99-100 — A spatial index for querying the nearest points
    let mut sindex: Index3D = Index3D::new();

    // Clustering.cpp:102-103 — Build the index
    for &idx in indices {
        sindex.push((pointfn(idx), idx));
    }

    // Clustering.cpp:105-109
    cluster_impl(&mut sindex, max_points, &|sidx, p| {
        distance_queryfn(sidx, p, dist, max_points)
    })
}

// Clustering.cpp:112-134 —
// Clustering a set of points by the given criteria
// ClusteredPoints cluster(
//     const std::vector<unsigned>& indices,
//     std::function<Vec3d(unsigned)> pointfn,
//     std::function<bool(const PointIndexEl&, const PointIndexEl&)> predicate,
//     unsigned max_points)
pub fn cluster_by_predicate(
    indices: &[u32],
    pointfn: &dyn Fn(u32) -> Vec3d,
    predicate: &dyn Fn(&PointIndexEl, &PointIndexEl) -> bool,
    max_points: u32,
) -> ClusteredPoints {
    // Clustering.cpp:119-120 — A spatial index for querying the nearest points
    let mut sindex: Index3D = Index3D::new();

    // Clustering.cpp:122-123 — Build the index
    for &idx in indices {
        sindex.push((pointfn(idx), idx));
    }

    // Clustering.cpp:125-133
    cluster_impl(&mut sindex, max_points, &|sidx, p| {
        // Clustering.cpp:128 — tmp.reserve(max_points);
        let mut tmp: Vec<PointIndexEl> = Vec::with_capacity(max_points as usize);
        // Clustering.cpp:129-131 — sidx.query(bgi::satisfies(...), back_inserter(tmp));
        for e in sidx.iter() {
            if predicate(p, e) {
                tmp.push(e.clone());
            }
        }
        tmp
    })
}

// Clustering.cpp:136-150 —
// ClusteredPoints cluster(const Eigen::MatrixXd& pts, double dist, unsigned max_points)
// (The Eigen n-by-3 row matrix is represented as a slice of `Vec3d` rows.)
pub fn cluster_points(pts: &[Vec3d], dist: f64, max_points: u32) -> ClusteredPoints {
    // Clustering.cpp:138-139 — A spatial index for querying the nearest points
    let mut sindex: Index3D = Index3D::new();

    // Clustering.cpp:141-143 — Build the index
    for (i, row) in pts.iter().enumerate() {
        sindex.push((*row, i as u32));
    }

    // Clustering.cpp:145-149
    cluster_impl(&mut sindex, max_points, &|sidx, p| {
        distance_queryfn(sidx, p, dist, max_points)
    })
}

/// Helper: `std::next_permutation` equivalent over a slice (lexicographic).
/// Returns `false` (after restoring the first/sorted permutation) when the
/// input is the last permutation, mirroring std::next_permutation's contract.
/// (Used by `cluster_centroid`, Clustering.hpp:69.)
fn next_permutation<T: Ord>(arr: &mut [T]) -> bool {
    if arr.len() < 2 {
        return false;
    }
    let mut i = arr.len() - 1;
    loop {
        let i1 = i;
        i -= 1;
        if arr[i] < arr[i1] {
            let mut i2 = arr.len() - 1;
            while !(arr[i] < arr[i2]) {
                i2 -= 1;
            }
            arr.swap(i, i2);
            arr[i1..].reverse();
            return true;
        }
        if i == 0 {
            arr.reverse();
            return false;
        }
    }
}

// Clustering.hpp:30-77 —
// This function returns the position of the centroid in the input 'clust'
// vector of point indices.
// template<class DistFn, class PointFn>
// long cluster_centroid(const ClusterEl &clust, PointFn pointfn, DistFn df)
pub fn cluster_centroid<P>(
    clust: &ClusterEl,
    pointfn: impl Fn(u32) -> P,
    df: impl Fn(P, P) -> f64,
) -> i64 {
    // Clustering.hpp:35-40
    match clust.len() {
        0 => return -1, // Clustering.hpp:36 — /* empty cluster */
        1 => return 0,  // Clustering.hpp:37 — /* only one element */
        2 => return 0,  // Clustering.hpp:38 — /* if two elements, there is no center */
        _ => {}
    }

    // Clustering.hpp:42-48 —
    // The function works by calculating for each point the average distance
    // from all the other points in the cluster. We create a selector bitmask of
    // the same size as the cluster. The bitmask will have two true bits and
    // false bits for the rest of items and we will loop through all the
    // permutations of the bitmask (combinations of two points). Get the
    // distance for the two points and add the distance to the averages.
    // The point with the smallest average than wins.

    // Clustering.hpp:50-51 —
    // The complexity should be O(n^2) but we will mostly apply this function
    // for small clusters only (cca 3 elements)

    // Clustering.hpp:53 — create full zero bitmask
    let mut sel: Vec<bool> = vec![false; clust.len()];
    // Clustering.hpp:54 — insert the two ones
    let n = sel.len();
    for s in &mut sel[n - 2..] {
        *s = true;
    }
    // Clustering.hpp:55 — store the average distances
    let mut avgs: Vec<f64> = vec![0.0; clust.len()];

    // Clustering.hpp:57-69 — do { ... } while(std::next_permutation(sel));
    loop {
        // Clustering.hpp:58-60
        let mut idx: [usize; 2] = [0; 2];
        let mut j = 0usize;
        for i in 0..clust.len() {
            if sel[i] {
                idx[j] = i;
                j += 1;
            }
        }

        // Clustering.hpp:62-63
        let d = df(pointfn(clust[idx[0]]), pointfn(clust[idx[1]]));

        // Clustering.hpp:65-66 — add the distance to the sums for both associated points
        for &i in idx.iter() {
            avgs[i] += d;
        }

        // Clustering.hpp:68-69 — now continue with the next permutation of the bitmask with two 1s
        if !next_permutation(&mut sel) {
            break;
        }
    }

    // Clustering.hpp:71-72 — Divide by point size in the cluster to get the average (may be redundant)
    for a in &mut avgs {
        *a /= clust.len() as f64;
    }

    // Clustering.hpp:74-76 — get the lowest average distance and return the index
    // (std::min_element returns the first smallest element.)
    let mut minit = 0usize;
    for i in 1..avgs.len() {
        if avgs[i] < avgs[minit] {
            minit = i;
        }
    }
    minit as i64
}
