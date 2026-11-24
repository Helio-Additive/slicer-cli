//! Port of `src/glu-libtess/src/tessmono.c` (triangulation of the monotone
//! regions the sweep produced).

use super::geom::{edge_sign, vert_leq, St};
use super::mesh::{GluMesh, F_HEAD, E_HEAD};

impl GluMesh {
    #[inline]
    fn st(&self, v: usize) -> St {
        St::from(&self.v[v])
    }
    /// geom.h `EdgeGoesLeft`.
    #[inline]
    fn edge_goes_left(&self, e: usize) -> bool {
        vert_leq(self.st(self.dst(e)), self.st(self.org(e)))
    }
    /// geom.h `EdgeGoesRight`.
    #[inline]
    fn edge_goes_right(&self, e: usize) -> bool {
        vert_leq(self.st(self.org(e)), self.st(self.dst(e)))
    }

    /// `__gl_meshTessellateMonoRegion` (tessmono.c:67-122).
    pub fn gl_mesh_tessellate_mono_region(&mut self, face: usize) -> bool {
        // All edges are oriented CCW around the boundary of the region.
        // First, find the half-edge whose origin vertex is rightmost.
        let mut up = self.f[face].an_edge;
        while vert_leq(self.st(self.dst(up)), self.st(self.org(up))) {
            up = self.lprev(up);
        }
        while vert_leq(self.st(self.org(up)), self.st(self.dst(up))) {
            up = self.lnext(up);
        }
        let mut lo = self.lprev(up);

        while self.lnext(up) != lo {
            if vert_leq(self.st(self.dst(up)), self.st(self.org(lo))) {
                // up->Dst is on the left.  It is safe to form triangles from lo->Org.
                while self.lnext(lo) != up
                    && (self.edge_goes_left(self.lnext(lo))
                        || edge_sign(self.st(self.org(lo)), self.st(self.dst(lo)), self.st(self.dst(self.lnext(lo)))) <= 0.0)
                {
                    let ln = self.lnext(lo);
                    let temp_half_edge = self.mesh_connect(ln, lo);
                    lo = self.sym(temp_half_edge);
                }
                lo = self.lprev(lo);
            } else {
                // lo->Org is on the left.  We can make CCW triangles from up->Dst.
                while self.lnext(lo) != up
                    && (self.edge_goes_right(self.lprev(up))
                        || edge_sign(self.st(self.dst(up)), self.st(self.org(up)), self.st(self.org(self.lprev(up)))) >= 0.0)
                {
                    let lp = self.lprev(up);
                    let temp_half_edge = self.mesh_connect(up, lp);
                    up = self.sym(temp_half_edge);
                }
                up = self.lnext(up);
            }
        }
        // Now lo->Org == up->Dst == the leftmost vertex.  The remaining region
        // can be tessellated in a fan from this leftmost vertex.
        while self.lnext(self.lnext(lo)) != up {
            let ln = self.lnext(lo);
            let temp_half_edge = self.mesh_connect(ln, lo);
            lo = self.sym(temp_half_edge);
        }
        true
    }

    /// `__gl_meshTessellateInterior` (tessmono.c:129-142).
    pub fn gl_mesh_tessellate_interior(&mut self) -> bool {
        let mut f = self.f[F_HEAD].next;
        while f != F_HEAD {
            // Make sure we don''t try to tessellate the new triangles.
            let next = self.f[f].next;
            if self.f[f].inside {
                if !self.gl_mesh_tessellate_mono_region(f) {
                    return false;
                }
            }
            f = next;
        }
        true
    }

    /// `__gl_meshDiscardExterior` (tessmono.c:150-162).
    pub fn gl_mesh_discard_exterior(&mut self) {
        let mut f = self.f[F_HEAD].next;
        while f != F_HEAD {
            // Since f will be destroyed, save its next pointer.
            let next = self.f[f].next;
            if !self.f[f].inside {
                self.mesh_zap_face(f);
            }
            f = next;
        }
    }

    /// `__gl_meshSetWindingNumber` (tessmono.c:174-201).
    pub fn gl_mesh_set_winding_number(&mut self, value: i32, keep_only_boundary: bool) -> bool {
        let mut e = self.e[E_HEAD].next;
        while e != E_HEAD {
            let e_next = self.e[e].next;
            if self.f[self.rface(e)].inside != self.f[self.lface(e)].inside {
                // This is a boundary edge (one side is interior, one is exterior).
                self.e[e].winding = if self.f[self.lface(e)].inside { value } else { -value };
            } else {
                // Both regions are interior, or both are exterior.
                if !keep_only_boundary {
                    self.e[e].winding = 0;
                } else if !self.mesh_delete(e) {
                    return false;
                }
            }
            e = e_next;
        }
        true
    }
}
