//! Port of `src/glu-libtess/src/normal.c` (polygon normal and projection onto
//! the sweep plane; `TRUE_PROJECT` and `SLANTED_SWEEP` are not defined in the
//! pin's build, so the projection is perpendicular to a coordinate axis with
//! S_UNIT_X = 1, S_UNIT_Y = 0).

use super::mesh::{F_HEAD, NIL, V_HEAD};
use super::tess::{GluTesselator, GLU_TESS_MAX_COORD};

const S_UNIT_X: f64 = 1.0;
const S_UNIT_Y: f64 = 0.0;

/// `#define Dot(u,v) (u[0]*v[0] + u[1]*v[1] + u[2]*v[2])`, contracted.
#[inline]
fn dot(u: &[f64; 3], v: &[f64; 3]) -> f64 {
    u[2].mul_add(v[2], u[0].mul_add(v[0], u[1] * v[1]))
}

#[inline]
fn abs(x: f64) -> f64 {
    if x < 0.0 { -x } else { x }
}

/// `LongAxis` (normal.c:76-83).
fn long_axis(v: &[f64; 3]) -> usize {
    let mut i = 0;
    if abs(v[1]) > abs(v[0]) {
        i = 1;
    }
    if abs(v[2]) > abs(v[i]) {
        i = 2;
    }
    i
}

impl GluTesselator {
    /// `ComputeNormal` (normal.c:85-150).
    fn compute_normal(&self, norm: &mut [f64; 3]) {
        let mesh = self.m();
        let mut max_val = [-2.0 * GLU_TESS_MAX_COORD; 3];
        let mut min_val = [2.0 * GLU_TESS_MAX_COORD; 3];
        let mut max_vert = [NIL; 3];
        let mut min_vert = [NIL; 3];
        let mut v = mesh.v[V_HEAD].next;
        while v != V_HEAD {
            for i in 0..3 {
                let c = mesh.v[v].coords[i];
                if c < min_val[i] {
                    min_val[i] = c;
                    min_vert[i] = v;
                }
                if c > max_val[i] {
                    max_val[i] = c;
                    max_vert[i] = v;
                }
            }
            v = mesh.v[v].next;
        }
        // Find two vertices separated by at least 1/sqrt(3) of the maximum
        // distance between any two vertices
        let mut i = 0;
        if max_val[1] - min_val[1] > max_val[0] - min_val[0] {
            i = 1;
        }
        if max_val[2] - min_val[2] > max_val[i] - min_val[i] {
            i = 2;
        }
        if min_val[i] >= max_val[i] {
            // All vertices are the same -- normal doesn't matter
            norm[0] = 0.0;
            norm[1] = 0.0;
            norm[2] = 1.0;
            return;
        }
        // Look for a third vertex which forms the triangle with maximum area
        let mut max_len2 = 0.0;
        let v1 = &mesh.v[min_vert[i]];
        let v2 = &mesh.v[max_vert[i]];
        let d1 = [v1.coords[0] - v2.coords[0], v1.coords[1] - v2.coords[1], v1.coords[2] - v2.coords[2]];
        let mut v = mesh.v[V_HEAD].next;
        while v != V_HEAD {
            let c = &mesh.v[v].coords;
            let d2 = [c[0] - v2.coords[0], c[1] - v2.coords[1], c[2] - v2.coords[2]];
            let t_norm = [
                d1[1].mul_add(d2[2], -(d1[2] * d2[1])),
                d1[2].mul_add(d2[0], -(d1[0] * d2[2])),
                d1[0].mul_add(d2[1], -(d1[1] * d2[0])),
            ];
            let t_len2 = t_norm[2].mul_add(t_norm[2], t_norm[0].mul_add(t_norm[0], t_norm[1] * t_norm[1]));
            if t_len2 > max_len2 {
                max_len2 = t_len2;
                *norm = t_norm;
            }
            v = mesh.v[v].next;
        }
        if max_len2 <= 0.0 {
            // All points lie on a single line -- any decent normal will do
            *norm = [0.0; 3];
            norm[long_axis(&d1)] = 1.0;
        }
    }

    /// `CheckOrientation` (normal.c:153-180).
    fn check_orientation(&mut self) {
        let mesh = self.mesh.as_mut().unwrap();
        // When we compute the normal automatically, we choose the orientation
        // so that the sum of the signed areas of all contours is non-negative.
        let mut area = 0.0f64;
        let mut f = mesh.f[F_HEAD].next;
        while f != F_HEAD {
            let mut e = mesh.f[f].an_edge;
            if mesh.e[e].winding > 0 {
                loop {
                    let o = &mesh.v[mesh.org(e)];
                    let d = &mesh.v[mesh.dst(e)];
                    area = (o.s - d.s).mul_add(o.t + d.t, area);
                    e = mesh.lnext(e);
                    if e == mesh.f[f].an_edge {
                        break;
                    }
                }
            }
            f = mesh.f[f].next;
        }
        if area < 0.0 {
            // Reverse the orientation by flipping all the t-coordinates
            let mut v = mesh.v[V_HEAD].next;
            while v != V_HEAD {
                mesh.v[v].t = -mesh.v[v].t;
                v = mesh.v[v].next;
            }
            self.t_unit = [-self.t_unit[0], -self.t_unit[1], -self.t_unit[2]];
        }
    }

    /// `__gl_projectPolygon` (normal.c:198-257).
    pub fn gl_project_polygon(&mut self) {
        let mut norm = self.normal;
        let mut computed_normal = false;
        if norm[0] == 0.0 && norm[1] == 0.0 && norm[2] == 0.0 {
            self.compute_normal(&mut norm);
            computed_normal = true;
        }
        let i = long_axis(&norm);
        // Project perpendicular to a coordinate axis -- better numerically
        self.s_unit[i] = 0.0;
        self.s_unit[(i + 1) % 3] = S_UNIT_X;
        self.s_unit[(i + 2) % 3] = S_UNIT_Y;
        self.t_unit[i] = 0.0;
        self.t_unit[(i + 1) % 3] = if norm[i] > 0.0 { -S_UNIT_Y } else { S_UNIT_Y };
        self.t_unit[(i + 2) % 3] = if norm[i] > 0.0 { S_UNIT_X } else { -S_UNIT_X };

        // Project the vertices onto the sweep plane
        let (s_unit, t_unit) = (self.s_unit, self.t_unit);
        let mesh = self.mesh.as_mut().unwrap();
        let mut v = mesh.v[V_HEAD].next;
        while v != V_HEAD {
            let c = mesh.v[v].coords;
            mesh.v[v].s = dot(&c, &s_unit);
            mesh.v[v].t = dot(&c, &t_unit);
            v = mesh.v[v].next;
        }
        if computed_normal {
            self.check_orientation();
        }
    }
}
