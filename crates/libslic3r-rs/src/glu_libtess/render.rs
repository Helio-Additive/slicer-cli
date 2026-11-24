//! Port of `src/glu-libtess/src/render.c` (output of the tessellated mesh as
//! fans, strips and lonely triangles; the single-contour cache fast path).

use super::mesh::{F_HEAD, NIL};
use super::tess::{
    GluTesselator, GLU_TESS_WINDING_ABS_GEQ_TWO, GLU_TESS_WINDING_NEGATIVE, GLU_TESS_WINDING_NONZERO, GLU_TESS_WINDING_ODD,
    GLU_TESS_WINDING_POSITIVE, GL_LINE_LOOP, GL_TRIANGLES, GL_TRIANGLE_FAN, GL_TRIANGLE_STRIP,
};

/// render.c `struct FaceCount` (the render function as an enum).
#[derive(Clone, Copy, Debug)]
struct FaceCount {
    size: i64,
    e_start: usize,
    render: RenderKind,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RenderKind {
    Fan,
    Strip,
    Triangle,
}

const SIGN_INCONSISTENT: i32 = 2;

impl GluTesselator {
    /// `#define Marked(f) (! (f)->inside || (f)->marked)`
    #[inline]
    fn marked(&self, f: usize) -> bool {
        let m = self.m();
        !m.f[f].inside || m.f[f].marked
    }

    /// `#define AddToTrail(f,t) ((f)->trail = (t), (t) = (f), (f)->marked = TRUE)`
    #[inline]
    fn add_to_trail(&mut self, f: usize, t: &mut usize) {
        let m = self.mm();
        m.f[f].trail = *t;
        *t = f;
        m.f[f].marked = true;
    }

    /// `#define FreeTrail(t)`
    #[inline]
    fn free_trail(&mut self, mut t: usize) {
        let m = self.mm();
        while t != NIL {
            m.f[t].marked = false;
            t = m.f[t].trail;
        }
    }

    /// `__gl_renderMesh` (render.c:82-106).
    pub fn gl_render_mesh(&mut self) {
        // Make a list of separate triangles so we can render them all at once
        self.lonely_tri_list = NIL;
        {
            let m = self.mm();
            let mut f = m.f[F_HEAD].next;
            while f != F_HEAD {
                m.f[f].marked = false;
                f = m.f[f].next;
            }
        }
        let mut f = self.m().f[F_HEAD].next;
        while f != F_HEAD {
            // We examine all faces in an arbitrary order.  Whenever we find
            // an unprocessed face F, we output a group of faces including F
            // whose size is maximum.
            if self.m().f[f].inside && !self.m().f[f].marked {
                self.render_maximum_face_group(f);
            }
            f = self.m().f[f].next;
        }
        if self.lonely_tri_list != NIL {
            let l = self.lonely_tri_list;
            self.render_lonely_triangles(l);
            self.lonely_tri_list = NIL;
        }
    }

    /// `RenderMaximumFaceGroup` (render.c:109-135).
    fn render_maximum_face_group(&mut self, f_orig: usize) {
        let e = self.m().f[f_orig].an_edge;
        let mut max = FaceCount { size: 1, e_start: e, render: RenderKind::Triangle };
        if !self.flag_boundary {
            let ln = self.m().lnext(e);
            let lp = self.m().lprev(e);
            let new_face = self.maximum_fan(e);
            if new_face.size > max.size {
                max = new_face;
            }
            let new_face = self.maximum_fan(ln);
            if new_face.size > max.size {
                max = new_face;
            }
            let new_face = self.maximum_fan(lp);
            if new_face.size > max.size {
                max = new_face;
            }
            let new_face = self.maximum_strip(e);
            if new_face.size > max.size {
                max = new_face;
            }
            let new_face = self.maximum_strip(ln);
            if new_face.size > max.size {
                max = new_face;
            }
            let new_face = self.maximum_strip(lp);
            if new_face.size > max.size {
                max = new_face;
            }
        }
        match max.render {
            RenderKind::Fan => self.render_fan(max.e_start, max.size),
            RenderKind::Strip => self.render_strip(max.e_start, max.size),
            RenderKind::Triangle => self.render_triangle(max.e_start, max.size),
        }
    }

    /// `MaximumFan` (render.c:152-174).
    fn maximum_fan(&mut self, e_orig: usize) -> FaceCount {
        let mut new_face = FaceCount { size: 0, e_start: NIL, render: RenderKind::Fan };
        let mut trail = NIL;
        let mut e = e_orig;
        while !self.marked(self.m().lface(e)) {
            let lf = self.m().lface(e);
            self.add_to_trail(lf, &mut trail);
            new_face.size += 1;
            e = self.m().onext(e);
        }
        e = e_orig;
        while !self.marked(self.m().rface(e)) {
            let rf = self.m().rface(e);
            self.add_to_trail(rf, &mut trail);
            new_face.size += 1;
            e = self.m().oprev(e);
        }
        new_face.e_start = e;
        self.free_trail(trail);
        new_face
    }

    /// `MaximumStrip` (render.c:179-228).
    fn maximum_strip(&mut self, e_orig: usize) -> FaceCount {
        let mut new_face = FaceCount { size: 0, e_start: NIL, render: RenderKind::Strip };
        let mut head_size: i64 = 0;
        let mut tail_size: i64 = 0;
        let mut trail = NIL;

        let mut e = e_orig;
        while !self.marked(self.m().lface(e)) {
            let lf = self.m().lface(e);
            self.add_to_trail(lf, &mut trail);
            tail_size += 1;
            e = self.m().dprev(e);
            if self.marked(self.m().lface(e)) {
                break;
            }
            let lf = self.m().lface(e);
            self.add_to_trail(lf, &mut trail);
            // for-increment: ++tailSize, e = e->Onext
            tail_size += 1;
            e = self.m().onext(e);
        }
        let e_tail = e;

        e = e_orig;
        while !self.marked(self.m().rface(e)) {
            let rf = self.m().rface(e);
            self.add_to_trail(rf, &mut trail);
            head_size += 1;
            e = self.m().oprev(e);
            if self.marked(self.m().rface(e)) {
                break;
            }
            let rf = self.m().rface(e);
            self.add_to_trail(rf, &mut trail);
            // for-increment: ++headSize, e = e->Dnext
            head_size += 1;
            e = self.m().dnext(e);
        }
        let e_head = e;

        new_face.size = tail_size + head_size;
        if (tail_size & 1) == 0 {
            new_face.e_start = self.m().sym(e_tail);
        } else if (head_size & 1) == 0 {
            new_face.e_start = e_head;
        } else {
            // Both sides have odd length, we must shorten one of them.
            new_face.size -= 1;
            new_face.e_start = self.m().onext(e_head);
        }
        self.free_trail(trail);
        new_face
    }

    /// `RenderTriangle` (render.c:231-239).
    fn render_triangle(&mut self, e: usize, _size: i64) {
        // Just add the triangle to a triangle list, so we can render all
        // the separate triangles at once.
        let lf = self.m().lface(e);
        let mut t = self.lonely_tri_list;
        self.add_to_trail(lf, &mut t);
        self.lonely_tri_list = t;
    }

    /// `RenderLonelyTriangles` (render.c:242-271).
    fn render_lonely_triangles(&mut self, f: usize) {
        let mut f = f;
        let mut edge_state: i32 = -1; // force edge state output for first vertex
        self.call_begin(GL_TRIANGLES);
        while f != NIL {
            // Loop once for each edge (there will always be 3 edges)
            let mut e = self.m().f[f].an_edge;
            loop {
                if self.flag_boundary {
                    // Set the "edge state" to TRUE just before we output the
                    // first vertex of each edge on the polygon boundary.
                    let new_state = (!self.m().f[self.m().rface(e)].inside) as i32;
                    if edge_state != new_state {
                        edge_state = new_state;
                        // CALL_EDGE_FLAG_OR_EDGE_FLAG_DATA: no edge-flag callback in this port's clients
                    }
                }
                let d = self.m().v[self.m().org(e)].data;
                self.call_vertex(d);
                e = self.m().lnext(e);
                if e == self.m().f[f].an_edge {
                    break;
                }
            }
            f = self.m().f[f].trail;
        }
        self.call_end();
    }

    /// `RenderFan` (render.c:274-295).
    fn render_fan(&mut self, e: usize, size: i64) {
        let mut e = e;
        let mut _size = size;
        self.call_begin(GL_TRIANGLE_FAN);
        let d = self.m().v[self.m().org(e)].data;
        self.call_vertex(d);
        let d = self.m().v[self.m().dst(e)].data;
        self.call_vertex(d);
        while !self.marked(self.m().lface(e)) {
            let lf = self.m().lface(e);
            self.mm().f[lf].marked = true;
            _size -= 1;
            e = self.m().onext(e);
            let d = self.m().v[self.m().dst(e)].data;
            self.call_vertex(d);
        }
        self.call_end();
    }

    /// `RenderStrip` (render.c:298-325).
    fn render_strip(&mut self, e: usize, size: i64) {
        let mut e = e;
        let mut _size = size;
        self.call_begin(GL_TRIANGLE_STRIP);
        let d = self.m().v[self.m().org(e)].data;
        self.call_vertex(d);
        let d = self.m().v[self.m().dst(e)].data;
        self.call_vertex(d);
        while !self.marked(self.m().lface(e)) {
            let lf = self.m().lface(e);
            self.mm().f[lf].marked = true;
            _size -= 1;
            e = self.m().dprev(e);
            let d = self.m().v[self.m().org(e)].data;
            self.call_vertex(d);
            if self.marked(self.m().lface(e)) {
                break;
            }
            let lf = self.m().lface(e);
            self.mm().f[lf].marked = true;
            _size -= 1;
            e = self.m().onext(e);
            let d = self.m().v[self.m().dst(e)].data;
            self.call_vertex(d);
        }
        self.call_end();
    }

    /// `__gl_renderBoundary` (render.c:332-349).
    pub fn gl_render_boundary(&mut self) {
        let mut f = self.m().f[F_HEAD].next;
        while f != F_HEAD {
            if self.m().f[f].inside {
                self.call_begin(GL_LINE_LOOP);
                let mut e = self.m().f[f].an_edge;
                loop {
                    let d = self.m().v[self.m().org(e)].data;
                    self.call_vertex(d);
                    e = self.m().lnext(e);
                    if e == self.m().f[f].an_edge {
                        break;
                    }
                }
                self.call_end();
            }
            f = self.m().f[f].next;
        }
    }

    /// render.c `ComputeNormal` (render.c:363-427), on the vertex cache.
    fn compute_normal_cache(&self, norm: &mut [f64; 3], check: bool) -> i32 {
        let v0 = &self.cache[0];
        let vn = self.cache_count;
        let mut sign = 0;
        if !check {
            *norm = [0.0; 3];
        }
        let mut vc = 1usize;
        let mut xc = self.cache[vc].coords[0] - v0.coords[0];
        let mut yc = self.cache[vc].coords[1] - v0.coords[1];
        let mut zc = self.cache[vc].coords[2] - v0.coords[2];
        loop {
            vc += 1;
            if !(vc < vn) {
                break;
            }
            let (xp, yp, zp) = (xc, yc, zc);
            xc = self.cache[vc].coords[0] - v0.coords[0];
            yc = self.cache[vc].coords[1] - v0.coords[1];
            zc = self.cache[vc].coords[2] - v0.coords[2];

            // Compute (vp - v0) cross (vc - v0)
            let n = [yp.mul_add(zc, -(zp * yc)), zp.mul_add(xc, -(xp * zc)), xp.mul_add(yc, -(yp * xc))];

            let dot = n[2].mul_add(norm[2], n[0].mul_add(norm[0], n[1] * norm[1]));
            if !check {
                // Reverse the contribution of back-facing triangles to get
                // a reasonable normal for self-intersecting polygons (see above)
                if dot >= 0.0 {
                    norm[0] += n[0];
                    norm[1] += n[1];
                    norm[2] += n[2];
                } else {
                    norm[0] -= n[0];
                    norm[1] -= n[1];
                    norm[2] -= n[2];
                }
            } else if dot != 0.0 {
                // Check the new orientation for consistency with previous triangles
                if dot > 0.0 {
                    if sign < 0 {
                        return SIGN_INCONSISTENT;
                    }
                    sign = 1;
                } else {
                    if sign > 0 {
                        return SIGN_INCONSISTENT;
                    }
                    sign = -1;
                }
            }
        }
        sign
    }

    /// `__gl_renderCache` (render.c:435-502).
    pub fn gl_render_cache(&mut self) -> bool {
        let vn = self.cache_count;
        let mut norm = [0.0f64; 3];
        if self.cache_count < 3 {
            // Degenerate contour -- no output
            return true;
        }
        norm[0] = self.normal[0];
        norm[1] = self.normal[1];
        norm[2] = self.normal[2];
        if norm[0] == 0.0 && norm[1] == 0.0 && norm[2] == 0.0 {
            self.compute_normal_cache(&mut norm, false);
        }
        let sign = self.compute_normal_cache(&mut norm, true);
        if sign == SIGN_INCONSISTENT {
            // Fan triangles did not have a consistent orientation
            return false;
        }
        if sign == 0 {
            // All triangles were degenerate
            return true;
        }
        // Make sure we do the right thing for each winding rule
        match self.winding_rule {
            GLU_TESS_WINDING_ODD | GLU_TESS_WINDING_NONZERO => {}
            GLU_TESS_WINDING_POSITIVE => {
                if sign < 0 {
                    return true;
                }
            }
            GLU_TESS_WINDING_NEGATIVE => {
                if sign > 0 {
                    return true;
                }
            }
            GLU_TESS_WINDING_ABS_GEQ_TWO => return true,
            _ => {}
        }
        self.call_begin(if self.boundary_only {
            GL_LINE_LOOP
        } else if self.cache_count > 3 {
            GL_TRIANGLE_FAN
        } else {
            GL_TRIANGLES
        });
        let d = self.cache[0].data;
        self.call_vertex(d);
        if sign > 0 {
            for vc in 1..vn {
                let d = self.cache[vc].data;
                self.call_vertex(d);
            }
        } else {
            let mut vc = vn - 1;
            while vc > 0 {
                let d = self.cache[vc].data;
                self.call_vertex(d);
                vc -= 1;
            }
        }
        self.call_end();
        true
    }
}
