//! Port of `src/glu-libtess/src/sweep.c` / `sweep.h` (the plane sweep that
//! computes the planar arrangement and splits it into monotone regions).
//! Regions live in `tess.regions` (an arena; `NIL` = NULL); the edge
//! dictionary is `tess.dict` with region indices as keys.

use super::dict::Dict;
use super::geom::{edge_eval, edge_intersect, edge_sign, vert_eq, vert_l1dist, vert_leq, St};
use super::mesh::{GluMesh, E_HEAD, F_HEAD, NIL, V_HEAD};
use super::priorityq::PriorityQ;
use super::tess::{
    ActiveRegion, GluTesselator, GLU_TESS_MAX_COORD, GLU_TESS_NEED_COMBINE_CALLBACK, GLU_TESS_WINDING_ABS_GEQ_TWO,
    GLU_TESS_WINDING_NEGATIVE, GLU_TESS_WINDING_NONZERO, GLU_TESS_WINDING_ODD, GLU_TESS_WINDING_POSITIVE,
};

/// sweep.c:1047 (`4 * GLU_TESS_MAX_COORD`)
const SENTINEL_COORD: f64 = 4.0 * GLU_TESS_MAX_COORD;

#[inline]
fn st(mesh: &GluMesh, v: usize) -> St {
    St::from(&mesh.v[v])
}

/// `EdgeLeq` (sweep.c:93-132) on the regions' upper edges.
fn edge_leq(mesh: &GluMesh, event: usize, e1: usize, e2: usize) -> bool {
    let ev = st(mesh, event);
    if mesh.dst(e1) == event {
        if mesh.dst(e2) == event {
            // Two edges right of the sweep line which meet at the sweep event.
            // Sort them by slope.
            if vert_leq(st(mesh, mesh.org(e1)), st(mesh, mesh.org(e2))) {
                return edge_sign(st(mesh, mesh.dst(e2)), st(mesh, mesh.org(e1)), st(mesh, mesh.org(e2))) <= 0.0;
            }
            return edge_sign(st(mesh, mesh.dst(e1)), st(mesh, mesh.org(e2)), st(mesh, mesh.org(e1))) >= 0.0;
        }
        return edge_sign(st(mesh, mesh.dst(e2)), ev, st(mesh, mesh.org(e2))) <= 0.0;
    }
    if mesh.dst(e2) == event {
        return edge_sign(st(mesh, mesh.dst(e1)), ev, st(mesh, mesh.org(e1))) >= 0.0;
    }
    // General case - compute signed distance *from* e1, e2 to event
    let t1 = edge_eval(st(mesh, mesh.dst(e1)), ev, st(mesh, mesh.org(e1)));
    let t2 = edge_eval(st(mesh, mesh.dst(e2)), ev, st(mesh, mesh.org(e2)));
    t1 >= t2
}

/// `#define AddWinding(eDst,eSrc)`
fn add_winding(mesh: &mut GluMesh, e_dst: usize, e_src: usize) {
    mesh.e[e_dst].winding += mesh.e[e_src].winding;
    let ds = mesh.sym(e_dst);
    let ss = mesh.sym(e_src);
    mesh.e[ds].winding += mesh.e[ss].winding;
}

fn pq_leq(mesh: &GluMesh) -> impl Fn(usize, usize) -> bool + '_ {
    move |a, b| vert_leq(st(mesh, a), st(mesh, b))
}

impl GluTesselator {
    #[inline]
    fn region_below(&self, r: usize) -> usize {
        self.dict.dict_key(self.dict.dict_pred(self.regions[r].node_up))
    }
    #[inline]
    fn region_above(&self, r: usize) -> usize {
        self.dict.dict_key(self.dict.dict_succ(self.regions[r].node_up))
    }
    #[inline]
    fn vst(&self, v: usize) -> St {
        st(self.m(), v)
    }

    /// `DeleteRegion` (sweep.c:135-148).
    fn delete_region(&mut self, reg: usize) {
        let e_up = self.regions[reg].e_up;
        self.mm().e[e_up].active_region = NIL;
        let n = self.regions[reg].node_up;
        self.dict.dict_delete(n);
    }

    /// `FixUpperEdge` (sweep.c:151-163).
    fn fix_upper_edge(&mut self, reg: usize, new_edge: usize) -> bool {
        let e_up = self.regions[reg].e_up;
        if !self.mm().mesh_delete(e_up) {
            return false;
        }
        self.regions[reg].fix_upper_edge = false;
        self.regions[reg].e_up = new_edge;
        self.mm().e[new_edge].active_region = reg;
        true
    }

    /// `TopLeftRegion` (sweep.c:165-187).
    fn top_left_region(&mut self, reg: usize) -> usize {
        let mut reg = reg;
        let org = self.m().org(self.regions[reg].e_up);
        // Find the region above the uppermost edge with the same origin
        loop {
            reg = self.region_above(reg);
            if self.m().org(self.regions[reg].e_up) != org {
                break;
            }
        }
        // If the edge above was a temporary edge introduced by ConnectRightVertex,
        // now is the time to fix it.
        if self.regions[reg].fix_upper_edge {
            let rb = self.region_below(reg);
            let a = self.m().sym(self.regions[rb].e_up);
            let b = self.m().lnext(self.regions[reg].e_up);
            let e = self.mm().mesh_connect(a, b);
            self.fix_upper_edge(reg, e);
            reg = self.region_above(reg);
        }
        reg
    }

    /// `TopRightRegion` (sweep.c:189-198).
    fn top_right_region(&self, reg: usize) -> usize {
        let mut reg = reg;
        let dst = self.m().dst(self.regions[reg].e_up);
        // Find the region above the uppermost edge with the same destination
        loop {
            reg = self.region_above(reg);
            if self.m().dst(self.regions[reg].e_up) != dst {
                break;
            }
        }
        reg
    }

    /// `AddRegionBelow` (sweep.c:200-222).
    fn add_region_below(&mut self, reg_above: usize, e_new_up: usize) -> usize {
        let reg_new = self.regions.len();
        self.regions.push(ActiveRegion {
            e_up: e_new_up,
            node_up: NIL,
            winding_number: 0,
            inside: false,
            sentinel: false,
            dirty: false,
            fix_upper_edge: false,
        });
        let node = self.regions[reg_above].node_up;
        let GluTesselator { dict, mesh, regions, event, .. } = self;
        let mesh = mesh.as_ref().unwrap();
        let ev = *event;
        let n = dict.dict_insert_before(node, reg_new, |k1, k2| edge_leq(mesh, ev, regions[k1].e_up, regions[k2].e_up));
        self.regions[reg_new].node_up = n;
        self.mm().e[e_new_up].active_region = reg_new;
        reg_new
    }

    /// `IsWindingInside` (sweep.c:224-245).
    fn is_winding_inside(&self, n: i32) -> bool {
        match self.winding_rule {
            GLU_TESS_WINDING_ODD => (n & 1) != 0,
            GLU_TESS_WINDING_NONZERO => n != 0,
            GLU_TESS_WINDING_POSITIVE => n > 0,
            GLU_TESS_WINDING_NEGATIVE => n < 0,
            GLU_TESS_WINDING_ABS_GEQ_TWO => (n >= 2) || (n <= -2),
            _ => false,
        }
    }

    /// `ComputeWinding` (sweep.c:248-252).
    fn compute_winding(&mut self, reg: usize) {
        let ra = self.region_above(reg);
        let w = self.regions[ra].winding_number + self.m().e[self.regions[reg].e_up].winding;
        self.regions[reg].winding_number = w;
        self.regions[reg].inside = self.is_winding_inside(w);
    }

    /// `FinishRegion` (sweep.c:255-268).
    fn finish_region(&mut self, reg: usize) {
        let e = self.regions[reg].e_up;
        let f = self.m().lface(e);
        let inside = self.regions[reg].inside;
        self.mm().f[f].inside = inside;
        self.mm().f[f].an_edge = e; // optimization for __gl_meshTessellateMonoRegion()
        self.delete_region(reg);
    }

    /// `FinishLeftRegions` (sweep.c:271-322). `reg_last == NIL` is the C's NULL.
    fn finish_left_regions(&mut self, reg_first: usize, reg_last: usize) -> usize {
        let mut reg_prev = reg_first;
        let mut e_prev = self.regions[reg_first].e_up;
        while reg_prev != reg_last {
            self.regions[reg_prev].fix_upper_edge = false; // placement was OK
            let reg = self.region_below(reg_prev);
            let mut e = self.regions[reg].e_up;
            if self.m().org(e) != self.m().org(e_prev) {
                if !self.regions[reg].fix_upper_edge {
                    // Remove the last left-going edge.
                    self.finish_region(reg_prev);
                    break;
                }
                // If the edge below was a temporary edge introduced by
                // ConnectRightVertex, now is the time to fix it.
                let a = self.m().lprev(e_prev);
                let b = self.m().sym(e);
                e = self.mm().mesh_connect(a, b);
                self.fix_upper_edge(reg, e);
            }
            // Relink edges so that ePrev->Onext == e
            if self.m().onext(e_prev) != e {
                let op = self.m().oprev(e);
                self.mm().mesh_splice(op, e);
                self.mm().mesh_splice(e_prev, e);
            }
            self.finish_region(reg_prev); // may change reg->eUp
            e_prev = self.regions[reg].e_up;
            reg_prev = reg;
        }
        e_prev
    }

    /// `AddRightEdges` (sweep.c:325-388). `e_top_left == NIL` is NULL.
    fn add_right_edges(&mut self, reg_up: usize, e_first: usize, e_last: usize, e_top_left: usize, clean_up: bool) {
        let mut first_time = true;
        // Insert the new right-going edges in the dictionary
        let mut e = e_first;
        loop {
            let es = self.m().sym(e);
            self.add_region_below(reg_up, es);
            e = self.m().onext(e);
            if e == e_last {
                break;
            }
        }
        // Walk *all* right-going edges from e->Org, in the dictionary order,
        // updating the winding numbers of each region, and re-linking the mesh
        // edges to match the dictionary ordering (if necessary).
        let mut e_top_left = e_top_left;
        if e_top_left == NIL {
            let rb = self.region_below(reg_up);
            e_top_left = self.m().rprev(self.regions[rb].e_up);
        }
        let mut reg_prev = reg_up;
        let mut e_prev = e_top_left;
        let mut reg;
        loop {
            reg = self.region_below(reg_prev);
            e = self.m().sym(self.regions[reg].e_up);
            if self.m().org(e) != self.m().org(e_prev) {
                break;
            }
            if self.m().onext(e) != e_prev {
                // Unlink e from its current position, and relink below ePrev
                let op = self.m().oprev(e);
                self.mm().mesh_splice(op, e);
                let op = self.m().oprev(e_prev);
                self.mm().mesh_splice(op, e);
            }
            // Compute the winding number and "inside" flag for the new regions
            let w = self.regions[reg_prev].winding_number - self.m().e[e].winding;
            self.regions[reg].winding_number = w;
            self.regions[reg].inside = self.is_winding_inside(w);

            // Check for two outgoing edges with same slope -- process these
            // before any intersection tests (see example in __gl_computeInterior).
            self.regions[reg_prev].dirty = true;
            if !first_time && self.check_for_right_splice(reg_prev) {
                add_winding(self.mm(), e, e_prev);
                self.delete_region(reg_prev);
                self.mm().mesh_delete(e_prev);
            }
            first_time = false;
            reg_prev = reg;
            e_prev = e;
        }
        self.regions[reg_prev].dirty = true;
        if clean_up {
            // Check for intersections between newly adjacent edges.
            self.walk_dirty_regions(reg_prev);
        }
    }

    /// `CallCombine` (sweep.c:391-415).
    fn call_combine_fn(&mut self, isect: usize, data: [Option<[f64; 2]>; 4], weights: [f32; 4], needed: bool) {
        // Copy coord data in case the callback changes it.
        let coords = self.m().v[isect].coords;
        self.mm().v[isect].data = None;
        let out = match self.call_combine {
            Some(f) => f(&coords, &data, &weights),
            None => None,
        };
        self.mm().v[isect].data = out;
        if self.m().v[isect].data.is_none() {
            if !needed {
                self.mm().v[isect].data = data[0];
            } else if !self.fatal_error {
                // The only way fatal error is when two edges are found to intersect,
                // but the user has not provided the callback necessary to handle
                // generated intersection points.
                self.call_error(GLU_TESS_NEED_COMBINE_CALLBACK);
                self.fatal_error = true;
            }
        }
    }

    /// `SpliceMergeVertices` (sweep.c:417-432).
    fn splice_merge_vertices(&mut self, e1: usize, e2: usize) {
        let mut data: [Option<[f64; 2]>; 4] = [None, None, None, None];
        let weights: [f32; 4] = [0.5, 0.5, 0.0, 0.0];
        data[0] = self.m().v[self.m().org(e1)].data;
        data[1] = self.m().v[self.m().org(e2)].data;
        let o = self.m().org(e1);
        self.call_combine_fn(o, data, weights, false);
        self.mm().mesh_splice(e1, e2);
    }

    /// `VertexWeights` (sweep.c:434-454): weights are GLfloat.
    fn vertex_weights(&mut self, isect: usize, org: usize, dst: usize, weights: &mut [f32]) {
        let mesh = self.mm();
        let t1 = vert_l1dist(st(mesh, org), st(mesh, isect));
        let t2 = vert_l1dist(st(mesh, dst), st(mesh, isect));
        weights[0] = (0.5 * t2 / (t1 + t2)) as f32;
        weights[1] = (0.5 * t1 / (t1 + t2)) as f32;
        let (w0, w1) = (weights[0] as f64, weights[1] as f64);
        let oc = mesh.v[org].coords;
        let dc = mesh.v[dst].coords;
        for k in 0..3 {
            mesh.v[isect].coords[k] += w0.mul_add(oc[k], w1 * dc[k]);
        }
    }

    /// `GetIntersectData` (sweep.c:456-479).
    fn get_intersect_data(&mut self, isect: usize, org_up: usize, dst_up: usize, org_lo: usize, dst_lo: usize) {
        let data = [self.m().v[org_up].data, self.m().v[dst_up].data, self.m().v[org_lo].data, self.m().v[dst_lo].data];
        let mut weights = [0.0f32; 4];
        self.mm().v[isect].coords = [0.0; 3];
        self.vertex_weights(isect, org_up, dst_up, &mut weights[0..2]);
        self.vertex_weights(isect, org_lo, dst_lo, &mut weights[2..4]);
        self.call_combine_fn(isect, data, weights, true);
    }

    /// `pqDelete` / `pqInsert` on tess->pq with VertLeq.
    fn pq_delete_v(&mut self, handle: i64) {
        let GluTesselator { pq, mesh, .. } = self;
        let mesh = mesh.as_ref().unwrap();
        pq.as_mut().unwrap().pq_delete(handle, &pq_leq(mesh));
    }
    fn pq_insert_v(&mut self, v: usize) -> i64 {
        let GluTesselator { pq, mesh, .. } = self;
        let mesh = mesh.as_ref().unwrap();
        pq.as_mut().unwrap().pq_insert(v, &pq_leq(mesh))
    }

    /// `CheckForRightSplice` (sweep.c:481-541).
    fn check_for_right_splice(&mut self, reg_up: usize) -> bool {
        let reg_lo = self.region_below(reg_up);
        let e_up = self.regions[reg_up].e_up;
        let e_lo = self.regions[reg_lo].e_up;
        let m = self.m();
        if vert_leq(st(m, m.org(e_up)), st(m, m.org(e_lo))) {
            if edge_sign(st(m, m.dst(e_lo)), st(m, m.org(e_up)), st(m, m.org(e_lo))) > 0.0 {
                return false;
            }
            // eUp->Org appears to be below eLo
            if !vert_eq(st(m, m.org(e_up)), st(m, m.org(e_lo))) {
                // Splice eUp->Org into eLo
                let els = m.sym(e_lo);
                self.mm().mesh_split_edge(els);
                let op = self.m().oprev(e_lo);
                self.mm().mesh_splice(e_up, op);
                self.regions[reg_up].dirty = true;
                self.regions[reg_lo].dirty = true;
            } else if m.org(e_up) != m.org(e_lo) {
                // merge the two vertices, discarding eUp->Org
                let h = m.v[m.org(e_up)].pq_handle;
                self.pq_delete_v(h);
                let op = self.m().oprev(e_lo);
                self.splice_merge_vertices(op, e_up);
            }
        } else {
            if edge_sign(st(m, m.dst(e_up)), st(m, m.org(e_lo)), st(m, m.org(e_up))) < 0.0 {
                return false;
            }
            // eLo->Org appears to be above eUp, so splice eLo->Org into eUp
            let ra = self.region_above(reg_up);
            self.regions[ra].dirty = true;
            self.regions[reg_up].dirty = true;
            let eus = self.m().sym(e_up);
            self.mm().mesh_split_edge(eus);
            let op = self.m().oprev(e_lo);
            self.mm().mesh_splice(op, e_up);
        }
        true
    }

    /// `CheckForLeftSplice` (sweep.c:543-584).
    fn check_for_left_splice(&mut self, reg_up: usize) -> bool {
        let reg_lo = self.region_below(reg_up);
        let e_up = self.regions[reg_up].e_up;
        let e_lo = self.regions[reg_lo].e_up;
        let m = self.m();
        if vert_leq(st(m, m.dst(e_up)), st(m, m.dst(e_lo))) {
            if edge_sign(st(m, m.dst(e_up)), st(m, m.dst(e_lo)), st(m, m.org(e_up))) < 0.0 {
                return false;
            }
            // eLo->Dst is above eUp, so splice eLo->Dst into eUp
            let ra = self.region_above(reg_up);
            self.regions[ra].dirty = true;
            self.regions[reg_up].dirty = true;
            let e = self.mm().mesh_split_edge(e_up);
            let els = self.m().sym(e_lo);
            self.mm().mesh_splice(els, e);
            let lf = self.m().lface(e);
            let inside = self.regions[reg_up].inside;
            self.mm().f[lf].inside = inside;
        } else {
            if edge_sign(st(m, m.dst(e_lo)), st(m, m.dst(e_up)), st(m, m.org(e_lo))) > 0.0 {
                return false;
            }
            // eUp->Dst is below eLo, so splice eUp->Dst into eLo
            self.regions[reg_up].dirty = true;
            self.regions[reg_lo].dirty = true;
            let e = self.mm().mesh_split_edge(e_lo);
            let a = self.m().lnext(e_up);
            let b = self.m().sym(e_lo);
            self.mm().mesh_splice(a, b);
            let rf = self.m().rface(e);
            let inside = self.regions[reg_up].inside;
            self.mm().f[rf].inside = inside;
        }
        true
    }

    /// `CheckForIntersect` (sweep.c:587-759).
    fn check_for_intersect(&mut self, reg_up: usize) -> bool {
        let mut reg_up = reg_up;
        let mut reg_lo = self.region_below(reg_up);
        let mut e_up = self.regions[reg_up].e_up;
        let mut e_lo = self.regions[reg_lo].e_up;
        let m = self.m();
        let org_up = m.org(e_up);
        let org_lo = m.org(e_lo);
        let dst_up = m.dst(e_up);
        let dst_lo = m.dst(e_lo);
        let event = self.event;
        let (s_org_up, s_org_lo, s_dst_up, s_dst_lo, s_event) = (st(m, org_up), st(m, org_lo), st(m, dst_up), st(m, dst_lo), st(m, event));

        if org_up == org_lo {
            return false; // right endpoints are the same
        }
        let t_min_up = if s_org_up.t <= s_dst_up.t { s_org_up.t } else { s_dst_up.t };
        let t_max_lo = if s_org_lo.t >= s_dst_lo.t { s_org_lo.t } else { s_dst_lo.t };
        if t_min_up > t_max_lo {
            return false; // t ranges do not overlap
        }
        if vert_leq(s_org_up, s_org_lo) {
            if edge_sign(s_dst_lo, s_org_up, s_org_lo) > 0.0 {
                return false;
            }
        } else if edge_sign(s_dst_up, s_org_lo, s_org_up) < 0.0 {
            return false;
        }

        // At this point the edges intersect, at least marginally
        let mut isect = edge_intersect(s_dst_up, s_org_up, s_dst_lo, s_org_lo);

        if vert_leq(isect, s_event) {
            // The intersection point lies slightly to the left of the sweep line,
            // so move it until it's slightly to the right of the sweep line.
            isect.s = s_event.s;
            isect.t = s_event.t;
        }
        // Similarly, if the computed intersection lies to the right of the
        // rightmost origin (which should rarely happen), it can cause
        // unbelievable inefficiency on sufficiently degenerate inputs.
        let org_min = if vert_leq(s_org_up, s_org_lo) { s_org_up } else { s_org_lo };
        if vert_leq(org_min, isect) {
            isect.s = org_min.s;
            isect.t = org_min.t;
        }

        if vert_eq(isect, s_org_up) || vert_eq(isect, s_org_lo) {
            // Easy case -- intersection at one of the right endpoints
            self.check_for_right_splice(reg_up);
            return false;
        }

        if (!vert_eq(s_dst_up, s_event) && edge_sign(s_dst_up, s_event, isect) >= 0.0)
            || (!vert_eq(s_dst_lo, s_event) && edge_sign(s_dst_lo, s_event, isect) <= 0.0)
        {
            // Very unusual -- the new upper or lower edge would pass on the
            // wrong side of the sweep event, or through it.
            if dst_lo == event {
                // Splice dstLo into eUp, and process the new region(s)
                let eus = self.m().sym(e_up);
                self.mm().mesh_split_edge(eus);
                let els = self.m().sym(e_lo);
                self.mm().mesh_splice(els, e_up);
                reg_up = self.top_left_region(reg_up);
                let rb = self.region_below(reg_up);
                e_up = self.regions[rb].e_up;
                self.finish_left_regions(rb, reg_lo);
                let op = self.m().oprev(e_up);
                self.add_right_edges(reg_up, op, e_up, e_up, true);
                return true;
            }
            if dst_up == event {
                // Splice dstUp into eLo, and process the new region(s)
                let els = self.m().sym(e_lo);
                self.mm().mesh_split_edge(els);
                let a = self.m().lnext(e_up);
                let b = self.m().oprev(e_lo);
                self.mm().mesh_splice(a, b);
                reg_lo = reg_up;
                reg_up = self.top_right_region(reg_up);
                let rb = self.region_below(reg_up);
                let e = self.m().rprev(self.regions[rb].e_up);
                self.regions[reg_lo].e_up = self.m().oprev(e_lo);
                e_lo = self.finish_left_regions(reg_lo, NIL);
                let a = self.m().onext(e_lo);
                let b = self.m().rprev(e_up);
                self.add_right_edges(reg_up, a, b, e, true);
                return true;
            }
            // Special case: called from ConnectRightVertex.  If either
            // edge passes on the wrong side of tess->event, split it
            // (and wait for ConnectRightVertex to splice it appropriately).
            if edge_sign(s_dst_up, s_event, isect) >= 0.0 {
                let ra = self.region_above(reg_up);
                self.regions[ra].dirty = true;
                self.regions[reg_up].dirty = true;
                let eus = self.m().sym(e_up);
                self.mm().mesh_split_edge(eus);
                let o = self.m().org(e_up);
                self.mm().v[o].s = s_event.s;
                self.mm().v[o].t = s_event.t;
            }
            if edge_sign(s_dst_lo, s_event, isect) <= 0.0 {
                self.regions[reg_up].dirty = true;
                self.regions[reg_lo].dirty = true;
                let els = self.m().sym(e_lo);
                self.mm().mesh_split_edge(els);
                let o = self.m().org(e_lo);
                self.mm().v[o].s = s_event.s;
                self.mm().v[o].t = s_event.t;
            }
            // leave the rest for ConnectRightVertex
            return false;
        }

        // General case -- split both edges, splice into new vertex.
        let eus = self.m().sym(e_up);
        self.mm().mesh_split_edge(eus);
        let els = self.m().sym(e_lo);
        self.mm().mesh_split_edge(els);
        let op = self.m().oprev(e_lo);
        self.mm().mesh_splice(op, e_up);
        let o = self.m().org(e_up);
        self.mm().v[o].s = isect.s;
        self.mm().v[o].t = isect.t;
        let h = self.pq_insert_v(o);
        self.mm().v[o].pq_handle = h;
        self.get_intersect_data(o, org_up, dst_up, org_lo, dst_lo);
        let ra = self.region_above(reg_up);
        self.regions[ra].dirty = true;
        self.regions[reg_up].dirty = true;
        self.regions[reg_lo].dirty = true;
        false
    }

    /// `WalkDirtyRegions` (sweep.c:761-840).
    fn walk_dirty_regions(&mut self, reg_up: usize) {
        let mut reg_up = reg_up;
        let mut reg_lo = self.region_below(reg_up);
        loop {
            // Find the lowest dirty region (we walk from the bottom up).
            while self.regions[reg_lo].dirty {
                reg_up = reg_lo;
                reg_lo = self.region_below(reg_lo);
            }
            if !self.regions[reg_up].dirty {
                reg_lo = reg_up;
                reg_up = self.region_above(reg_up);
                if reg_up == NIL || !self.regions[reg_up].dirty {
                    // We've walked all the dirty regions
                    return;
                }
            }
            self.regions[reg_up].dirty = false;
            let mut e_up = self.regions[reg_up].e_up;
            let mut e_lo = self.regions[reg_lo].e_up;

            if self.m().dst(e_up) != self.m().dst(e_lo) {
                // Check that the edge ordering is obeyed at the Dst vertices.
                if self.check_for_left_splice(reg_up) {
                    // If the upper or lower edge was marked fixUpperEdge, then
                    // we no longer need it.
                    if self.regions[reg_lo].fix_upper_edge {
                        self.delete_region(reg_lo);
                        self.mm().mesh_delete(e_lo);
                        reg_lo = self.region_below(reg_up);
                        e_lo = self.regions[reg_lo].e_up;
                    } else if self.regions[reg_up].fix_upper_edge {
                        self.delete_region(reg_up);
                        self.mm().mesh_delete(e_up);
                        reg_up = self.region_above(reg_lo);
                        e_up = self.regions[reg_up].e_up;
                    }
                }
            }
            if self.m().org(e_up) != self.m().org(e_lo) {
                if self.m().dst(e_up) != self.m().dst(e_lo)
                    && !self.regions[reg_up].fix_upper_edge
                    && !self.regions[reg_lo].fix_upper_edge
                    && (self.m().dst(e_up) == self.event || self.m().dst(e_lo) == self.event)
                {
                    // When all else fails in CheckForIntersect(), it uses tess->event
                    // as the intersection location.
                    if self.check_for_intersect(reg_up) {
                        // WalkDirtyRegions() was called recursively; we're done
                        return;
                    }
                } else {
                    // Even though we can't use CheckForIntersect(), the Org vertices
                    // may violate the dictionary edge ordering.  Check and correct this.
                    self.check_for_right_splice(reg_up);
                }
            }
            if self.m().org(e_up) == self.m().org(e_lo) && self.m().dst(e_up) == self.m().dst(e_lo) {
                // A degenerate loop consisting of only two edges -- delete it.
                add_winding(self.mm(), e_lo, e_up);
                self.delete_region(reg_up);
                self.mm().mesh_delete(e_up);
                reg_up = self.region_above(reg_lo);
            }
        }
    }

    /// `ConnectRightVertex` (sweep.c:843-920).
    fn connect_right_vertex(&mut self, reg_up: usize, e_bottom_left: usize) {
        let mut reg_up = reg_up;
        let mut e_bottom_left = e_bottom_left;
        let mut e_top_left = self.m().onext(e_bottom_left);
        let reg_lo = self.region_below(reg_up);
        let e_up = self.regions[reg_up].e_up;
        let e_lo = self.regions[reg_lo].e_up;
        let mut degenerate = false;

        if self.m().dst(e_up) != self.m().dst(e_lo) {
            self.check_for_intersect(reg_up);
        }

        // Possible new degeneracies: upper or lower edge of regUp may pass
        // through vEvent, or may coincide with new intersection vertex
        if vert_eq(self.vst(self.m().org(e_up)), self.vst(self.event)) {
            let op = self.m().oprev(e_top_left);
            self.mm().mesh_splice(op, e_up);
            reg_up = self.top_left_region(reg_up);
            let rb = self.region_below(reg_up);
            e_top_left = self.regions[rb].e_up;
            self.finish_left_regions(rb, reg_lo);
            degenerate = true;
        }
        if vert_eq(self.vst(self.m().org(e_lo)), self.vst(self.event)) {
            let op = self.m().oprev(e_lo);
            self.mm().mesh_splice(e_bottom_left, op);
            e_bottom_left = self.finish_left_regions(reg_lo, NIL);
            degenerate = true;
        }
        if degenerate {
            let a = self.m().onext(e_bottom_left);
            self.add_right_edges(reg_up, a, e_top_left, e_top_left, true);
            return;
        }

        // Non-degenerate situation -- need to add a temporary, fixable edge.
        // Connect to the closer of eLo->Org, eUp->Org.
        let mut e_new = if vert_leq(self.vst(self.m().org(e_lo)), self.vst(self.m().org(e_up))) { self.m().oprev(e_lo) } else { e_up };
        let lp = self.m().lprev(e_bottom_left);
        e_new = self.mm().mesh_connect(lp, e_new);

        // Prevent cleanup, otherwise eNew might disappear before we've even
        // had a chance to mark it as a temporary edge.
        let on = self.m().onext(e_new);
        self.add_right_edges(reg_up, e_new, on, on, false);
        let ar = self.m().e[self.m().sym(e_new)].active_region;
        self.regions[ar].fix_upper_edge = true;
        self.walk_dirty_regions(reg_up);
    }

    /// `ConnectLeftDegenerate` (sweep.c:931-984). TOLERANCE_NONZERO is FALSE.
    fn connect_left_degenerate(&mut self, reg_up: usize, v_event: usize) {
        let mut reg_up = reg_up;
        let e = self.regions[reg_up].e_up;
        if vert_eq(self.vst(self.m().org(e)), self.vst(v_event)) {
            // e->Org is an unprocessed vertex - just combine them, and wait
            // for e->Org to be pulled from the queue
            let ae = self.m().v[v_event].an_edge;
            self.splice_merge_vertices(e, ae);
            return;
        }
        if !vert_eq(self.vst(self.m().dst(e)), self.vst(v_event)) {
            // General case -- splice vEvent into edge e which passes through it
            let es = self.m().sym(e);
            self.mm().mesh_split_edge(es);
            if self.regions[reg_up].fix_upper_edge {
                // This edge was fixable -- delete unused portion of original edge
                let on = self.m().onext(e);
                self.mm().mesh_delete(on);
                self.regions[reg_up].fix_upper_edge = false;
            }
            let ae = self.m().v[v_event].an_edge;
            self.mm().mesh_splice(ae, e);
            self.sweep_event(v_event); // recurse
            return;
        }

        // vEvent coincides with e->Dst, which has already been processed.
        // Splice in the additional right-going edges.
        reg_up = self.top_right_region(reg_up);
        let reg = self.region_below(reg_up);
        let mut e_top_right = self.m().sym(self.regions[reg].e_up);
        let mut e_top_left = self.m().onext(e_top_right);
        let e_last = e_top_left;
        if self.regions[reg].fix_upper_edge {
            // Here e->Dst has only a single fixable edge going right.
            // We can delete it since now we have some real right-going edges.
            self.delete_region(reg);
            self.mm().mesh_delete(e_top_right);
            e_top_right = self.m().oprev(e_top_left);
        }
        let ae = self.m().v[v_event].an_edge;
        self.mm().mesh_splice(ae, e_top_right);
        // EdgeGoesLeft(eTopLeft)
        if !vert_leq(self.vst(self.m().dst(e_top_left)), self.vst(self.m().org(e_top_left))) {
            // e->Dst had no left-going edges -- indicate this to AddRightEdges()
            e_top_left = NIL;
        }
        let on = self.m().onext(e_top_right);
        self.add_right_edges(reg_up, on, e_last, e_top_left, true);
    }

    /// `ConnectLeftVertex` (sweep.c:987-1037).
    fn connect_left_vertex(&mut self, v_event: usize) {
        // Get a pointer to the active region containing vEvent
        let tmp_e_up = self.m().sym(self.m().v[v_event].an_edge);
        let reg_up = {
            let GluTesselator { dict, mesh, regions, event, .. } = &*self;
            let mesh = mesh.as_ref().unwrap();
            let ev = *event;
            // __gl_dictListSearch
            let n = dict.dict_search(|k| edge_leq(mesh, ev, tmp_e_up, regions[k].e_up));
            dict.dict_key(n)
        };
        let reg_lo = self.region_below(reg_up);
        let e_up = self.regions[reg_up].e_up;
        let e_lo = self.regions[reg_lo].e_up;

        // Try merging with U or L first
        if edge_sign(self.vst(self.m().dst(e_up)), self.vst(v_event), self.vst(self.m().org(e_up))) == 0.0 {
            self.connect_left_degenerate(reg_up, v_event);
            return;
        }

        // Connect vEvent to rightmost processed vertex of either chain.
        // e->Dst is the vertex that we will connect to vEvent.
        let reg = if vert_leq(self.vst(self.m().dst(e_lo)), self.vst(self.m().dst(e_up))) { reg_up } else { reg_lo };

        if self.regions[reg_up].inside || self.regions[reg].fix_upper_edge {
            let e_new;
            if reg == reg_up {
                let a = self.m().sym(self.m().v[v_event].an_edge);
                let b = self.m().lnext(e_up);
                e_new = self.mm().mesh_connect(a, b);
            } else {
                let a = self.m().dnext(e_lo);
                let b = self.m().v[v_event].an_edge;
                let temp_half_edge = self.mm().mesh_connect(a, b);
                e_new = self.m().sym(temp_half_edge);
            }
            if self.regions[reg].fix_upper_edge {
                self.fix_upper_edge(reg, e_new);
            } else {
                let r = self.add_region_below(reg_up, e_new);
                self.compute_winding(r);
            }
            self.sweep_event(v_event);
        } else {
            // The new vertex is in a region which does not belong to the polygon.
            // We don't need to connect this vertex to the rest of the mesh.
            let ae = self.m().v[v_event].an_edge;
            self.add_right_edges(reg_up, ae, ae, NIL, true);
        }
    }

    /// `SweepEvent` (sweep.c:1040-1084).
    fn sweep_event(&mut self, v_event: usize) {
        self.event = v_event; // for access in EdgeLeq()

        // Check if this vertex is the right endpoint of an edge that is
        // already in the dictionary.
        let mut e = self.m().v[v_event].an_edge;
        while self.m().e[e].active_region == NIL {
            e = self.m().onext(e);
            if e == self.m().v[v_event].an_edge {
                // All edges go right -- not incident to any processed edges
                self.connect_left_vertex(v_event);
                return;
            }
        }

        // Processing consists of two phases: first we "finish" all the
        // active regions where both the upper and lower edges terminate
        // at vEvent.
        let ar = self.m().e[e].active_region;
        let reg_up = self.top_left_region(ar);
        let reg = self.region_below(reg_up);
        let e_top_left = self.regions[reg].e_up;
        let e_bottom_left = self.finish_left_regions(reg, NIL);

        // Next we process all the right-going edges from vEvent.
        if self.m().onext(e_bottom_left) == e_top_left {
            // No right-going edges -- add a temporary "fixable" edge
            self.connect_right_vertex(reg_up, e_bottom_left);
        } else {
            let a = self.m().onext(e_bottom_left);
            self.add_right_edges(reg_up, a, e_top_left, e_top_left, true);
        }
    }

    /// `AddSentinel` (sweep.c:1090-1114).
    fn add_sentinel(&mut self, t: f64) {
        let e = self.mm().mesh_make_edge();
        let o = self.m().org(e);
        let d = self.m().dst(e);
        self.mm().v[o].s = SENTINEL_COORD;
        self.mm().v[o].t = t;
        self.mm().v[d].s = -SENTINEL_COORD;
        self.mm().v[d].t = t;
        self.event = d; // initialize it

        let reg = self.regions.len();
        self.regions.push(ActiveRegion { e_up: e, node_up: NIL, winding_number: 0, inside: false, sentinel: true, dirty: false, fix_upper_edge: false });
        let GluTesselator { dict, mesh, regions, event, .. } = self;
        let mesh = mesh.as_ref().unwrap();
        let ev = *event;
        // __gl_dictListInsertBefore
        let n = dict.dict_insert(reg, |k1, k2| edge_leq(mesh, ev, regions[k1].e_up, regions[k2].e_up));
        self.regions[reg].node_up = n;
    }

    /// `InitEdgeDict` (sweep.c:1117-1129).
    fn init_edge_dict(&mut self) {
        self.dict = Dict::dict_new_dict();
        self.regions.clear();
        self.add_sentinel(-SENTINEL_COORD);
        self.add_sentinel(SENTINEL_COORD);
    }

    /// `DoneEdgeDict` (sweep.c:1132-1153).
    fn done_edge_dict(&mut self) {
        loop {
            let reg = self.dict.dict_key(self.dict.dict_min());
            if reg == NIL {
                break;
            }
            self.delete_region(reg);
        }
    }

    /// `RemoveDegenerateEdges` (sweep.c:1156-1189).
    fn remove_degenerate_edges(&mut self) {
        let mut e = self.m().e[E_HEAD].next;
        while e != E_HEAD {
            let mut e_next = self.m().e[e].next;
            let mut e_lnext = self.m().lnext(e);

            if vert_eq(self.vst(self.m().org(e)), self.vst(self.m().dst(e))) && self.m().lnext(self.m().lnext(e)) != e {
                // Zero-length edge, contour has at least 3 edges
                self.splice_merge_vertices(e_lnext, e); // deletes e->Org
                self.mm().mesh_delete(e); // e is a self-loop
                e = e_lnext;
                e_lnext = self.m().lnext(e);
            }
            if self.m().lnext(e_lnext) == e {
                // Degenerate contour (one or two edges)
                if e_lnext != e {
                    if e_lnext == e_next || e_lnext == self.m().sym(e_next) {
                        e_next = self.m().e[e_next].next;
                    }
                    self.mm().mesh_delete(e_lnext);
                }
                if e == e_next || e == self.m().sym(e_next) {
                    e_next = self.m().e[e_next].next;
                }
                self.mm().mesh_delete(e);
            }
            e = e_next;
        }
    }

    /// `InitPriorityQ` (sweep.c:1191-1215).
    fn init_priority_q(&mut self) -> bool {
        let mut pq = PriorityQ::pq_new_priority_q();
        {
            let mesh = self.mesh.as_mut().unwrap();
            let mut v = mesh.v[V_HEAD].next;
            while v != V_HEAD {
                let h = {
                    let m: &GluMesh = mesh;
                    pq.pq_insert(v, &pq_leq(m))
                };
                mesh.v[v].pq_handle = h;
                v = mesh.v[v].next;
            }
            let m: &GluMesh = mesh;
            pq.pq_init(&pq_leq(m));
        }
        self.pq = Some(pq);
        true
    }

    /// `DonePriorityQ` (sweep.c:1218-1221).
    fn done_priority_q(&mut self) {
        self.pq = None;
    }

    /// `__gl_computeInterior` (sweep.c:1260-1310).
    pub fn gl_compute_interior(&mut self) -> bool {
        self.fatal_error = false;
        // Each vertex defines an event for our sweep line.
        self.remove_degenerate_edges();
        if !self.init_priority_q() {
            return false; // if error
        }
        self.init_edge_dict();

        loop {
            let v = {
                let GluTesselator { pq, mesh, .. } = self;
                let mesh = mesh.as_ref().unwrap();
                pq.as_mut().unwrap().pq_extract_min(&pq_leq(mesh))
            };
            if v == NIL {
                break;
            }
            loop {
                let v_next = {
                    let GluTesselator { pq, mesh, .. } = &*self;
                    let mesh = mesh.as_ref().unwrap();
                    pq.as_ref().unwrap().pq_minimum(&pq_leq(mesh))
                };
                if v_next == NIL || !vert_eq(self.vst(v_next), self.vst(v)) {
                    break;
                }
                // Merge together all vertices at exactly the same location.
                let v_next = {
                    let GluTesselator { pq, mesh, .. } = self;
                    let mesh = mesh.as_ref().unwrap();
                    pq.as_mut().unwrap().pq_extract_min(&pq_leq(mesh))
                };
                let a = self.m().v[v].an_edge;
                let b = self.m().v[v_next].an_edge;
                self.splice_merge_vertices(a, b);
            }
            self.sweep_event(v);
        }

        // Set tess->event for debugging purposes
        let r = self.dict.dict_key(self.dict.dict_min());
        self.event = self.m().org(self.regions[r].e_up);
        self.done_edge_dict();
        self.done_priority_q();

        if !self.mm().remove_degenerate_faces() {
            return false;
        }
        true
    }
}

impl GluMesh {
    /// `RemoveDegenerateFaces` (sweep.c:1228-1249).
    fn remove_degenerate_faces(&mut self) -> bool {
        let mut f = self.f[F_HEAD].next;
        while f != F_HEAD {
            let f_next = self.f[f].next;
            let e = self.f[f].an_edge;
            if self.lnext(self.lnext(e)) == e {
                // A face with only two edges
                let on = self.onext(e);
                add_winding(self, on, e);
                if !self.mesh_delete(e) {
                    return false;
                }
            }
            f = f_next;
        }
        true
    }
}
