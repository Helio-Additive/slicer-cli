//! Port of `src/glu-libtess/src/mesh.c` / `mesh.h` (SGI libtess half-edge mesh).
//!
//! The C allocates vertices, faces and edge pairs with `malloc` and links them
//! with pointers. Here every object lives in an arena (`Vec`) and pointers are
//! indices; `NIL` is the C's `NULL`. An edge pair occupies the indices `2k`
//! (`e`) and `2k + 1` (`eSym`), so the C's address comparison
//! `eNext->Sym < eNext` (the pair's first member has the lower address,
//! mesh.c:EdgePair) is the same comparison of indices. The dummy headers are
//! vertex 0, face 0 and edges 0 / 1 (`eHead` before `eHeadSym` in `GLUmesh`).
//! Freed objects stay in the arena unused (nothing compares their addresses).

pub const NIL: usize = usize::MAX;

/// mesh.h `struct GLUvertex`. `data` is the client's vertex data: the
/// Tesselate.cpp client passes a pointer to the (x, y, z) triple it fed and
/// reads only x and y, so the data is kept as those two values.
#[derive(Clone, Debug)]
pub struct GluVertex {
    pub next: usize,
    pub prev: usize,
    pub an_edge: usize,
    pub data: Option<[f64; 2]>,
    pub coords: [f64; 3],
    pub s: f64,
    pub t: f64,
    pub pq_handle: i64,
}

impl GluVertex {
    fn blank() -> Self {
        GluVertex { next: NIL, prev: NIL, an_edge: NIL, data: None, coords: [0.0; 3], s: 0.0, t: 0.0, pq_handle: 0 }
    }
}

/// mesh.h `struct GLUface`.
#[derive(Clone, Debug)]
pub struct GluFace {
    pub next: usize,
    pub prev: usize,
    pub an_edge: usize,
    pub trail: usize,
    pub marked: bool,
    pub inside: bool,
}

impl GluFace {
    fn blank() -> Self {
        GluFace { next: NIL, prev: NIL, an_edge: NIL, trail: NIL, marked: false, inside: false }
    }
}

/// mesh.h `struct GLUhalfEdge`.
#[derive(Clone, Debug)]
pub struct GluHalfEdge {
    pub next: usize,
    pub sym: usize,
    pub onext: usize,
    pub lnext: usize,
    pub org: usize,
    pub lface: usize,
    pub active_region: usize,
    pub winding: i32,
}

impl GluHalfEdge {
    fn blank() -> Self {
        GluHalfEdge { next: NIL, sym: NIL, onext: NIL, lnext: NIL, org: NIL, lface: NIL, active_region: NIL, winding: 0 }
    }
}

/// mesh.h `struct GLUmesh`: vertex 0 = vHead, face 0 = fHead, edges 0/1 = eHead/eHeadSym.
#[derive(Clone, Debug)]
pub struct GluMesh {
    pub v: Vec<GluVertex>,
    pub f: Vec<GluFace>,
    pub e: Vec<GluHalfEdge>,
}

pub const V_HEAD: usize = 0;
pub const F_HEAD: usize = 0;
pub const E_HEAD: usize = 0;

impl GluMesh {
    // mesh.h macros
    #[inline]
    pub fn sym(&self, e: usize) -> usize { self.e[e].sym }
    #[inline]
    pub fn onext(&self, e: usize) -> usize { self.e[e].onext }
    #[inline]
    pub fn lnext(&self, e: usize) -> usize { self.e[e].lnext }
    #[inline]
    pub fn org(&self, e: usize) -> usize { self.e[e].org }
    #[inline]
    pub fn lface(&self, e: usize) -> usize { self.e[e].lface }
    /// `#define Rface Sym->Lface`
    #[inline]
    pub fn rface(&self, e: usize) -> usize { self.e[self.e[e].sym].lface }
    /// `#define Dst Sym->Org`
    #[inline]
    pub fn dst(&self, e: usize) -> usize { self.e[self.e[e].sym].org }
    /// `#define Oprev Sym->Lnext`
    #[inline]
    pub fn oprev(&self, e: usize) -> usize { self.e[self.e[e].sym].lnext }
    /// `#define Lprev Onext->Sym`
    #[inline]
    pub fn lprev(&self, e: usize) -> usize { self.e[self.e[e].onext].sym }
    /// `#define Dprev Lnext->Sym`
    #[inline]
    pub fn dprev(&self, e: usize) -> usize { self.e[self.e[e].lnext].sym }
    /// `#define Rprev Sym->Onext`
    #[inline]
    pub fn rprev(&self, e: usize) -> usize { self.e[self.e[e].sym].onext }
    /// `#define Dnext Rprev->Sym`
    #[inline]
    pub fn dnext(&self, e: usize) -> usize { self.e[self.rprev(e)].sym }
    /// `#define Rnext Oprev->Sym`
    #[inline]
    pub fn rnext(&self, e: usize) -> usize { self.e[self.oprev(e)].sym }
    #[inline]
    pub fn set_dst(&mut self, e: usize, v: usize) { let s = self.e[e].sym; self.e[s].org = v; }
    #[inline]
    pub fn set_rface(&mut self, e: usize, f: usize) { let s = self.e[e].sym; self.e[s].lface = f; }

    fn alloc_vertex(&mut self) -> usize { self.v.push(GluVertex::blank()); self.v.len() - 1 }
    fn alloc_face(&mut self) -> usize { self.f.push(GluFace::blank()); self.f.len() - 1 }

    /// mesh.c MakeEdge (mesh.c:92-133): a new edge pair inserted before eNext.
    fn make_edge(&mut self, e_next: usize) -> usize {
        let e = self.e.len();
        let e_sym = e + 1;
        self.e.push(GluHalfEdge::blank());
        self.e.push(GluHalfEdge::blank());
        let mut e_next = e_next;
        // Make sure eNext points to the first edge of the edge pair
        if self.e[e_next].sym < e_next {
            e_next = self.e[e_next].sym;
        }
        // Insert in circular doubly-linked list before eNext.
        // Note that the prev pointer is stored in Sym->next.
        let e_prev = self.e[self.e[e_next].sym].next;
        self.e[e_sym].next = e_prev;
        let eps = self.e[e_prev].sym;
        self.e[eps].next = e;
        self.e[e].next = e_next;
        let ens = self.e[e_next].sym;
        self.e[ens].next = e_sym;

        self.e[e].sym = e_sym;
        self.e[e].onext = e;
        self.e[e].lnext = e_sym;
        self.e[e].org = NIL;
        self.e[e].lface = NIL;
        self.e[e].winding = 0;
        self.e[e].active_region = NIL;

        self.e[e_sym].sym = e;
        self.e[e_sym].onext = e_sym;
        self.e[e_sym].lnext = e;
        self.e[e_sym].org = NIL;
        self.e[e_sym].lface = NIL;
        self.e[e_sym].winding = 0;
        self.e[e_sym].active_region = NIL;
        e
    }

    /// mesh.c Splice (mesh.c:144-152).
    fn splice(&mut self, a: usize, b: usize) {
        let a_onext = self.e[a].onext;
        let b_onext = self.e[b].onext;
        let s = self.e[a_onext].sym;
        self.e[s].lnext = b;
        let s = self.e[b_onext].sym;
        self.e[s].lnext = a;
        self.e[a].onext = b_onext;
        self.e[b].onext = a_onext;
    }

    /// mesh.c MakeVertex (mesh.c:160-183).
    fn make_vertex(&mut self, v_new: usize, e_orig: usize, v_next: usize) {
        // insert in circular doubly-linked list before vNext
        let v_prev = self.v[v_next].prev;
        self.v[v_new].prev = v_prev;
        self.v[v_prev].next = v_new;
        self.v[v_new].next = v_next;
        self.v[v_next].prev = v_new;

        self.v[v_new].an_edge = e_orig;
        self.v[v_new].data = None;
        // leave coords, s, t undefined

        // fix other edges on this vertex loop
        let mut e = e_orig;
        loop {
            self.e[e].org = v_new;
            e = self.e[e].onext;
            if e == e_orig {
                break;
            }
        }
    }

    /// mesh.c MakeFace (mesh.c:191-220).
    fn make_face(&mut self, f_new: usize, e_orig: usize, f_next: usize) {
        // insert in circular doubly-linked list before fNext
        let f_prev = self.f[f_next].prev;
        self.f[f_new].prev = f_prev;
        self.f[f_prev].next = f_new;
        self.f[f_new].next = f_next;
        self.f[f_next].prev = f_new;

        self.f[f_new].an_edge = e_orig;
        self.f[f_new].trail = NIL;
        self.f[f_new].marked = false;

        // The new face is marked "inside" if the old one was.
        self.f[f_new].inside = self.f[f_next].inside;

        // fix other edges on this face loop
        let mut e = e_orig;
        loop {
            self.e[e].lface = f_new;
            e = self.e[e].lnext;
            if e == e_orig {
                break;
            }
        }
    }

    /// mesh.c KillEdge (mesh.c:225-239).
    fn kill_edge(&mut self, e_del: usize) {
        let mut e_del = e_del;
        // Half-edges are allocated in pairs, see EdgePair above
        if self.e[e_del].sym < e_del {
            e_del = self.e[e_del].sym;
        }
        // delete from circular doubly-linked list
        let e_next = self.e[e_del].next;
        let e_prev = self.e[self.e[e_del].sym].next;
        let s = self.e[e_next].sym;
        self.e[s].next = e_prev;
        let s = self.e[e_prev].sym;
        self.e[s].next = e_next;
    }

    /// mesh.c KillVertex (mesh.c:245-264).
    fn kill_vertex(&mut self, v_del: usize, new_org: usize) {
        let e_start = self.v[v_del].an_edge;
        // change the origin of all affected edges
        let mut e = e_start;
        loop {
            self.e[e].org = new_org;
            e = self.e[e].onext;
            if e == e_start {
                break;
            }
        }
        // delete from circular doubly-linked list
        let v_prev = self.v[v_del].prev;
        let v_next = self.v[v_del].next;
        self.v[v_next].prev = v_prev;
        self.v[v_prev].next = v_next;
    }

    /// mesh.c KillFace (mesh.c:269-288).
    fn kill_face(&mut self, f_del: usize, new_lface: usize) {
        let e_start = self.f[f_del].an_edge;
        // change the left face of all affected edges
        let mut e = e_start;
        loop {
            self.e[e].lface = new_lface;
            e = self.e[e].lnext;
            if e == e_start {
                break;
            }
        }
        // delete from circular doubly-linked list
        let f_prev = self.f[f_del].prev;
        let f_next = self.f[f_del].next;
        self.f[f_next].prev = f_prev;
        self.f[f_prev].next = f_next;
    }

    /// `__gl_meshMakeEdge` (mesh.c:296-322): one edge, two vertices, a loop.
    pub fn mesh_make_edge(&mut self) -> usize {
        let new_vertex1 = self.alloc_vertex();
        let new_vertex2 = self.alloc_vertex();
        let new_face = self.alloc_face();
        let e = self.make_edge(E_HEAD);
        self.make_vertex(new_vertex1, e, V_HEAD);
        let es = self.e[e].sym;
        self.make_vertex(new_vertex2, es, V_HEAD);
        self.make_face(new_face, e, F_HEAD);
        e
    }

    /// `__gl_meshSplice` (mesh.c:342-386).
    pub fn mesh_splice(&mut self, e_org: usize, e_dst: usize) -> bool {
        let mut joining_loops = false;
        let mut joining_vertices = false;
        if e_org == e_dst {
            return true;
        }
        if self.e[e_dst].org != self.e[e_org].org {
            // We are merging two disjoint vertices -- destroy eDst->Org
            joining_vertices = true;
            self.kill_vertex(self.e[e_dst].org, self.e[e_org].org);
        }
        if self.e[e_dst].lface != self.e[e_org].lface {
            // We are connecting two disjoint loops -- destroy eDst->Lface
            joining_loops = true;
            self.kill_face(self.e[e_dst].lface, self.e[e_org].lface);
        }
        // Change the edge structure
        self.splice(e_dst, e_org);
        if !joining_vertices {
            let new_vertex = self.alloc_vertex();
            // We split one vertex into two -- the new vertex is eDst->Org.
            let o = self.e[e_org].org;
            self.make_vertex(new_vertex, e_dst, o);
            let o = self.e[e_org].org;
            self.v[o].an_edge = e_org;
        }
        if !joining_loops {
            let new_face = self.alloc_face();
            // We split one loop into two -- the new loop is eDst->Lface.
            let lf = self.e[e_org].lface;
            self.make_face(new_face, e_dst, lf);
            let lf = self.e[e_org].lface;
            self.f[lf].an_edge = e_org;
        }
        true
    }

    /// `__gl_meshDelete` (mesh.c:398-450).
    pub fn mesh_delete(&mut self, e_del: usize) -> bool {
        let e_del_sym = self.e[e_del].sym;
        let mut joining_loops = false;
        // First step: disconnect the origin vertex eDel->Org.
        if self.e[e_del].lface != self.rface(e_del) {
            // We are joining two loops into one -- remove the left face
            joining_loops = true;
            self.kill_face(self.e[e_del].lface, self.rface(e_del));
        }
        if self.e[e_del].onext == e_del {
            self.kill_vertex(self.e[e_del].org, NIL);
        } else {
            // Make sure that eDel->Org and eDel->Rface point to valid half-edges
            let rf = self.rface(e_del);
            self.f[rf].an_edge = self.oprev(e_del);
            let o = self.e[e_del].org;
            self.v[o].an_edge = self.e[e_del].onext;
            let op = self.oprev(e_del);
            self.splice(e_del, op);
            if !joining_loops {
                let new_face = self.alloc_face();
                // We are splitting one loop into two -- create a new loop for eDel.
                let lf = self.e[e_del].lface;
                self.make_face(new_face, e_del, lf);
            }
        }
        // Now we disconnect eDel->Dst.
        if self.e[e_del_sym].onext == e_del_sym {
            self.kill_vertex(self.e[e_del_sym].org, NIL);
            self.kill_face(self.e[e_del_sym].lface, NIL);
        } else {
            // Make sure that eDel->Dst and eDel->Lface point to valid half-edges
            let lf = self.e[e_del].lface;
            self.f[lf].an_edge = self.oprev(e_del_sym);
            let o = self.e[e_del_sym].org;
            self.v[o].an_edge = self.e[e_del_sym].onext;
            let op = self.oprev(e_del_sym);
            self.splice(e_del_sym, op);
        }
        // Any isolated vertices or faces have already been freed.
        self.kill_edge(e_del);
        true
    }

    /// `__gl_meshAddEdgeVertex` (mesh.c:462-483).
    pub fn mesh_add_edge_vertex(&mut self, e_org: usize) -> usize {
        let e_new = self.make_edge(e_org);
        let e_new_sym = self.e[e_new].sym;
        // Connect the new edge appropriately
        let ln = self.e[e_org].lnext;
        self.splice(e_new, ln);
        // Set the vertex and face information
        self.e[e_new].org = self.dst(e_org);
        {
            let new_vertex = self.alloc_vertex();
            let o = self.e[e_new].org;
            self.make_vertex(new_vertex, e_new_sym, o);
        }
        let lf = self.e[e_org].lface;
        self.e[e_new].lface = lf;
        self.e[e_new_sym].lface = lf;
        e_new
    }

    /// `__gl_meshSplitEdge` (mesh.c:490-511).
    pub fn mesh_split_edge(&mut self, e_org: usize) -> usize {
        let temp_half_edge = self.mesh_add_edge_vertex(e_org);
        let e_new = self.e[temp_half_edge].sym;
        // Disconnect eOrg from eOrg->Dst and connect it to eNew->Org
        let eos = self.e[e_org].sym;
        let op = self.oprev(eos);
        self.splice(eos, op);
        self.splice(eos, e_new);
        // Set the vertex and face information
        let no = self.e[e_new].org;
        self.set_dst(e_org, no);
        let nd = self.dst(e_new);
        self.v[nd].an_edge = self.e[e_new].sym; // may have pointed to eOrg->Sym
        let rf = self.rface(e_org);
        self.set_rface(e_new, rf);
        self.e[e_new].winding = self.e[e_org].winding; // copy old winding information
        let ens = self.e[e_new].sym;
        self.e[ens].winding = self.e[eos].winding;
        e_new
    }

    /// `__gl_meshConnect` (mesh.c:523-554).
    pub fn mesh_connect(&mut self, e_org: usize, e_dst: usize) -> usize {
        let mut joining_loops = false;
        let e_new = self.make_edge(e_org);
        let e_new_sym = self.e[e_new].sym;
        if self.e[e_dst].lface != self.e[e_org].lface {
            // We are connecting two disjoint loops -- destroy eDst->Lface
            joining_loops = true;
            self.kill_face(self.e[e_dst].lface, self.e[e_org].lface);
        }
        // Connect the new edge appropriately
        let ln = self.e[e_org].lnext;
        self.splice(e_new, ln);
        self.splice(e_new_sym, e_dst);
        // Set the vertex and face information
        self.e[e_new].org = self.dst(e_org);
        self.e[e_new_sym].org = self.e[e_dst].org;
        let lf = self.e[e_org].lface;
        self.e[e_new].lface = lf;
        self.e[e_new_sym].lface = lf;
        // Make sure the old face points to a valid half-edge
        self.f[lf].an_edge = e_new_sym;
        if !joining_loops {
            let new_face = self.alloc_face();
            // We split one loop into two -- the new loop is eNew->Lface
            let lf = self.e[e_org].lface;
            self.make_face(new_face, e_new, lf);
        }
        e_new
    }

    /// `__gl_meshZapFace` (mesh.c:566-606).
    pub fn mesh_zap_face(&mut self, f_zap: usize) {
        let e_start = self.f[f_zap].an_edge;
        // walk around face, deleting edges whose right face is also NULL
        let mut e_next = self.e[e_start].lnext;
        loop {
            let e = e_next;
            e_next = self.e[e].lnext;
            self.e[e].lface = NIL;
            if self.rface(e) == NIL {
                // delete the edge -- see __gl_MeshDelete above
                if self.e[e].onext == e {
                    self.kill_vertex(self.e[e].org, NIL);
                } else {
                    // Make sure that e->Org points to a valid half-edge
                    let o = self.e[e].org;
                    self.v[o].an_edge = self.e[e].onext;
                    let op = self.oprev(e);
                    self.splice(e, op);
                }
                let e_sym = self.e[e].sym;
                if self.e[e_sym].onext == e_sym {
                    self.kill_vertex(self.e[e_sym].org, NIL);
                } else {
                    // Make sure that eSym->Org points to a valid half-edge
                    let o = self.e[e_sym].org;
                    self.v[o].an_edge = self.e[e_sym].onext;
                    let op = self.oprev(e_sym);
                    self.splice(e_sym, op);
                }
                self.kill_edge(e);
            }
            if e == e_start {
                break;
            }
        }
        // delete from circular doubly-linked list
        let f_prev = self.f[f_zap].prev;
        let f_next = self.f[f_zap].next;
        self.f[f_next].prev = f_prev;
        self.f[f_prev].next = f_next;
    }

    /// `__gl_meshNewMesh` (mesh.c:612-654).
    pub fn mesh_new_mesh() -> GluMesh {
        let mut mesh = GluMesh { v: vec![GluVertex::blank()], f: vec![GluFace::blank()], e: vec![GluHalfEdge::blank(), GluHalfEdge::blank()] };
        mesh.v[0].next = 0;
        mesh.v[0].prev = 0;
        mesh.f[0].next = 0;
        mesh.f[0].prev = 0;
        mesh.e[0].next = 0;
        mesh.e[0].sym = 1;
        mesh.e[1].next = 1;
        mesh.e[1].sym = 0;
        mesh
    }
}

/// `__gl_meshNewMesh` (mesh.c:612-654) under its C name.
pub fn gl_mesh_new_mesh() -> GluMesh {
    GluMesh::mesh_new_mesh()
}
