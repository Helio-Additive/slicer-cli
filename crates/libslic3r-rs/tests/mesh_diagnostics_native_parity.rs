//! `mesh_diagnostics` and `mesh_raycast` against MeshDiagnostics.cpp and MeshRaycast.cpp at the pin.
//!
//! `tests/data/mesh_diagnostics/vectors.txt` is the output of `gen.cpp` beside it, which compiles
//! the two C++ files themselves with the pin's headers (the header comment there says how, and
//! describes the stream). The meshes are triangle soups (open, non-manifold and same-direction
//! edges, bowtie vertices, degenerate faces, indices out of range) and closed meshes of one or two
//! parts, each either way round. For every mesh: `its_edge_diagnostics` with its neighbours,
//! `its_quick_diagnostics`, `its_mesh_diagnostics`, the ray test alone (`its_detect_reversed_faces`
//! on zeroed stats), and for the closed ones `TriangleRaycaster::all_hits` on eight rays, every hit
//! by the bits of its t, u and v.

use slicer::mesh_diagnostics::*;
use slicer::mesh_raycast::TriangleRaycaster;
use slicer::triangle_set_sampling::{indexed_triangle_set, Vec3d, Vec3f, Vec3i};

const VECTORS: &str = include_str!("data/mesh_diagnostics/vectors.txt");

struct Tok<'a>(std::str::SplitWhitespace<'a>);

impl<'a> Tok<'a> {
    fn s(&mut self) -> &'a str {
        self.0.next().expect("token")
    }
    fn tag(&mut self, tag: &str) {
        assert_eq!(self.s(), tag);
    }
    fn u(&mut self) -> usize {
        self.s().parse().expect("unsigned")
    }
    fn i(&mut self) -> i32 {
        self.s().parse().expect("integer")
    }
    fn b(&mut self) -> bool {
        self.u() != 0
    }
    fn d(&mut self) -> f64 {
        f64::from_bits(u64::from_str_radix(self.s(), 16).expect("double bits"))
    }
    fn f(&mut self) -> f32 {
        f32::from_bits(u32::from_str_radix(self.s(), 16).expect("float bits"))
    }
    fn vec3d(&mut self) -> Vec3d {
        Vec3d::new(self.d(), self.d(), self.d())
    }
}

#[test]
fn diagnostics_and_ray_hits() {
    let mut t = Tok(VECTORS.split_whitespace());
    t.tag("N");
    let count = t.u();
    assert_eq!(count, 450 + 54 + 6);
    let (mut ray_test_true, mut ray_test_false, mut rays, mut hits_total) = (0, 0, 0, 0);
    let mut defects = [0usize; 4];
    for mesh in 0..count {
        t.tag("M");
        let (nv, nf) = (t.u(), t.u());
        let its = indexed_triangle_set {
            vertices: (0..nv).map(|_| Vec3f::new(t.f(), t.f(), t.f())).collect(),
            indices: (0..nf).map(|_| Vec3i::new(t.i(), t.i(), t.i())).collect(),
        };

        t.tag("E");
        // Like gen.cpp: the vector is not empty on entry.
        let mut neighbors = vec![Vec3i::new(7, 7, 7); 3];
        let e = its_edge_diagnostics(&its, Some(&mut neighbors));
        assert_eq!((e.open_edges, e.non_manifold_edges, e.same_direction_edges), (t.u(), t.u(), t.u()), "mesh {mesh}: edges");
        assert!(!e.has_reversed_faces && e.non_manifold_vertices == 0);
        if nf == 0 {
            // An empty mesh leaves the vector as it was.
            assert_eq!(neighbors, vec![Vec3i::new(7, 7, 7); 3]);
        } else {
            let want: Vec<Vec3i> = (0..nf).map(|_| Vec3i::new(t.i(), t.i(), t.i())).collect();
            assert_eq!(neighbors, want, "mesh {mesh}: neighbors");
        }
        // Without neighbours the counts are the same.
        assert_eq!(its_edge_diagnostics(&its, None), e);

        t.tag("Q");
        let q = its_quick_diagnostics(&its, None);
        assert_eq!(
            (q.open_edges, q.non_manifold_edges, q.same_direction_edges, q.has_reversed_faces),
            (t.u(), t.u(), t.u(), t.b()),
            "mesh {mesh}: quick"
        );
        assert_eq!(q.non_manifold_vertices, 0);

        t.tag("D");
        let d = its_mesh_diagnostics(&its);
        assert_eq!(
            (d.open_edges, d.non_manifold_edges, d.non_manifold_vertices, d.same_direction_edges, d.has_reversed_faces),
            (t.u(), t.u(), t.u(), t.u(), t.b()),
            "mesh {mesh}: full"
        );
        for (k, n) in [d.open_edges, d.non_manifold_edges, d.non_manifold_vertices, d.same_direction_edges].iter().enumerate() {
            defects[k] += usize::from(*n > 0);
        }

        t.tag("S");
        let mut z = MeshDiagnosticStats::default();
        its_detect_reversed_faces(&its, &mut z);
        assert_eq!(z.has_reversed_faces, t.b(), "mesh {mesh}: ray test");
        if z.has_reversed_faces {
            ray_test_true += 1;
        } else {
            ray_test_false += 1;
        }

        t.tag("R");
        let num_rays = t.u();
        if num_rays == 0 {
            continue;
        }
        let caster = TriangleRaycaster::new(&its);
        for ray in 0..num_rays {
            let (origin, dir) = (t.vec3d(), t.vec3d());
            t.tag("->");
            let mut coplanar = -1;
            let hits = caster.all_hits(&origin, &dir, Some(&mut coplanar));
            assert_eq!(coplanar, 0);
            let n = t.u();
            assert_eq!(hits.len(), n, "mesh {mesh} ray {ray}: hit count");
            let want: Vec<(i32, u64, u64, u64)> =
                (0..n).map(|_| (t.i(), t.d().to_bits(), t.d().to_bits(), t.d().to_bits())).collect();
            for (k, hit) in hits.iter().enumerate() {
                // Hits at the same t come in the order std::sort left them in, which is not the
                // order of a stable sort: compare those as a set.
                let same_t = |w: &&(i32, u64, u64, u64)| w.1 == hit.t.to_bits();
                assert_eq!(want[k].1, hit.t.to_bits(), "mesh {mesh} ray {ray} hit {k}: t");
                assert!(
                    want.iter().filter(same_t).any(|w| *w == (hit.face, hit.t.to_bits(), hit.u.to_bits(), hit.v.to_bits())),
                    "mesh {mesh} ray {ray} hit {k}: {hit:?} is not among {want:x?}"
                );
            }
            rays += 1;
            hits_total += n;
        }
    }
    assert!(t.0.next().is_none(), "tokens left over");
    // The vectors are not all the trivial answer.
    assert!(ray_test_true >= 100 && ray_test_false >= 100, "ray test: {ray_test_true} true, {ray_test_false} false");
    assert!(defects.iter().all(|n| *n >= 100), "meshes per defect: {defects:?}");
    assert!(rays >= 400 && hits_total >= 400, "{rays} rays, {hits_total} hits");
}
