//! Port of `src/glu-libtess/src/tess.c` / `tess.h` (the GLU tessellator
//! object and its begin/vertex/end API).
//!
//! The C calls the client through function pointers. Here the rendering
//! callbacks (`begin`, `vertex`, `end`, `error`, with polygon data) are
//! recorded in order in `out` and the client replays them; the combine
//! callback is a function pointer that returns the new vertex's data, as
//! `GLU_TESS_COMBINE_DATA` does. `setjmp`/`longjmp` only handle allocation
//! failures, which cannot happen here.

use super::dict::Dict;
use super::mesh::{GluMesh, NIL};
use super::priorityq::PriorityQ;

pub const GL_LINE_LOOP: u32 = 0x0002;
pub const GL_TRIANGLES: u32 = 0x0004;
pub const GL_TRIANGLE_STRIP: u32 = 0x0005;
pub const GL_TRIANGLE_FAN: u32 = 0x0006;

pub const GLU_TESS_WINDING_ODD: u32 = 100130;
pub const GLU_TESS_WINDING_NONZERO: u32 = 100131;
pub const GLU_TESS_WINDING_POSITIVE: u32 = 100132;
pub const GLU_TESS_WINDING_NEGATIVE: u32 = 100133;
pub const GLU_TESS_WINDING_ABS_GEQ_TWO: u32 = 100134;

pub const GLU_TESS_MISSING_BEGIN_POLYGON: u32 = 100151;
pub const GLU_TESS_MISSING_BEGIN_CONTOUR: u32 = 100152;
pub const GLU_TESS_MISSING_END_POLYGON: u32 = 100153;
pub const GLU_TESS_MISSING_END_CONTOUR: u32 = 100154;
pub const GLU_TESS_COORD_TOO_LARGE: u32 = 100155;
pub const GLU_TESS_NEED_COMBINE_CALLBACK: u32 = 100156;

/// glu-libtess.h:173
pub const GLU_TESS_MAX_COORD: f64 = 1.0e150;
/// tess.c:47
pub const GLU_TESS_DEFAULT_TOLERANCE: f64 = 0.0;
/// tess.h:52
pub const TESS_MAX_CACHE: usize = 100;

/// tess.h `enum TessState`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum TessState {
    TDormant,
    TInPolygon,
    TInContour,
}

/// tess.h `CachedVertex`.
#[derive(Clone, Copy, Debug, Default)]
pub struct CachedVertex {
    pub coords: [f64; 3],
    pub data: Option<[f64; 2]>,
}

/// sweep.h `struct ActiveRegion`.
#[derive(Clone, Debug)]
pub struct ActiveRegion {
    pub e_up: usize,
    pub node_up: usize,
    pub winding_number: i32,
    pub inside: bool,
    pub sentinel: bool,
    pub dirty: bool,
    pub fix_upper_edge: bool,
}

/// A rendering callback as the client receives it.
#[derive(Clone, Debug, PartialEq)]
pub enum TessOut {
    Begin(u32),
    Vertex(Option<[f64; 2]>),
    End,
    Error(u32),
}

/// `GLU_TESS_COMBINE_DATA`: (coords, data[4], weight[4]) -> outData.
pub type CombineFn = fn(&[f64; 3], &[Option<[f64; 2]>; 4], &[f32; 4]) -> Option<[f64; 2]>;

/// tess.h `struct GLUtesselator`.
pub struct GluTesselator {
    pub state: TessState,
    pub last_edge: usize,
    pub mesh: Option<GluMesh>,
    pub normal: [f64; 3],
    pub s_unit: [f64; 3],
    pub t_unit: [f64; 3],
    pub rel_tolerance: f64,
    pub winding_rule: u32,
    pub fatal_error: bool,
    pub dict: Dict,
    pub pq: Option<PriorityQ>,
    pub event: usize,
    pub regions: Vec<ActiveRegion>,
    pub call_combine: Option<CombineFn>,
    pub flag_boundary: bool,
    pub boundary_only: bool,
    pub lonely_tri_list: usize,
    pub empty_cache: bool,
    pub cache_count: usize,
    pub cache: [CachedVertex; TESS_MAX_CACHE],
    /// The rendering callbacks in call order.
    pub out: Vec<TessOut>,
}

impl GluTesselator {
    #[inline]
    pub fn m(&self) -> &GluMesh {
        self.mesh.as_ref().unwrap()
    }
    #[inline]
    pub fn mm(&mut self) -> &mut GluMesh {
        self.mesh.as_mut().unwrap()
    }

    /// CALL_ERROR_OR_ERROR_DATA (tess.h:160-163).
    pub(crate) fn call_error(&mut self, errnum: u32) {
        self.out.push(TessOut::Error(errnum));
    }
    /// CALL_BEGIN_OR_BEGIN_DATA (tess.h:135-138).
    pub(crate) fn call_begin(&mut self, which: u32) {
        self.out.push(TessOut::Begin(which));
    }
    /// CALL_VERTEX_OR_VERTEX_DATA (tess.h:140-143).
    pub(crate) fn call_vertex(&mut self, data: Option<[f64; 2]>) {
        self.out.push(TessOut::Vertex(data));
    }
    /// CALL_END_OR_END_DATA (tess.h:150-153).
    pub(crate) fn call_end(&mut self) {
        self.out.push(TessOut::End);
    }

    /// `gluNewTess` (tess.c:90-142).
    pub fn glu_new_tess() -> GluTesselator {
        GluTesselator {
            state: TessState::TDormant,
            last_edge: NIL,
            mesh: None,
            normal: [0.0; 3],
            s_unit: [0.0; 3],
            t_unit: [0.0; 3],
            rel_tolerance: GLU_TESS_DEFAULT_TOLERANCE,
            winding_rule: GLU_TESS_WINDING_ODD,
            fatal_error: false,
            dict: Dict::dict_new_dict(),
            pq: None,
            event: NIL,
            regions: Vec::new(),
            call_combine: None,
            flag_boundary: false,
            boundary_only: false,
            lonely_tri_list: NIL,
            empty_cache: false,
            cache_count: 0,
            cache: [CachedVertex::default(); TESS_MAX_CACHE],
            out: Vec::new(),
        }
    }

    /// `MakeDormant` (tess.c:144-155).
    fn make_dormant(&mut self) {
        // This is the only place where we can free the mesh
        self.mesh = None;
        self.state = TessState::TDormant;
        self.last_edge = NIL;
    }

    /// `GotoState` (tess.c:159-198).
    fn goto_state(&mut self, new_state: TessState) {
        while self.state != new_state {
            // We change the current state one level at a time
            if self.state < new_state {
                match self.state {
                    TessState::TDormant => {
                        self.call_error(GLU_TESS_MISSING_BEGIN_POLYGON);
                        self.glu_tess_begin_polygon();
                    }
                    TessState::TInPolygon => {
                        self.call_error(GLU_TESS_MISSING_BEGIN_CONTOUR);
                        self.glu_tess_begin_contour();
                    }
                    _ => {}
                }
            } else {
                match self.state {
                    TessState::TInContour => {
                        self.call_error(GLU_TESS_MISSING_END_CONTOUR);
                        self.glu_tess_end_contour();
                    }
                    TessState::TInPolygon => {
                        self.call_error(GLU_TESS_MISSING_END_POLYGON);
                        // gluTessEndPolygon( tess ) is too much work!
                        self.make_dormant();
                    }
                    _ => {}
                }
            }
        }
    }

    #[inline]
    fn require_state(&mut self, s: TessState) {
        if self.state != s {
            self.goto_state(s);
        }
    }

    /// `gluTessNormal` (tess.c:305-311).
    pub fn glu_tess_normal(&mut self, x: f64, y: f64, z: f64) {
        self.normal = [x, y, z];
    }

    /// `AddVertex` (tess.c:404-440).
    fn add_vertex(&mut self, coords: &[f64; 3], data: Option<[f64; 2]>) -> bool {
        let mut e = self.last_edge;
        let mesh = self.mesh.as_mut().unwrap();
        if e == NIL {
            // Make a self-loop (one vertex, one edge).
            e = mesh.mesh_make_edge();
            let es = mesh.sym(e);
            mesh.mesh_splice(e, es);
        } else {
            // Create a new vertex and edge which immediately follow e
            // in the ordering around the left face.
            mesh.mesh_split_edge(e);
            e = mesh.lnext(e);
        }
        // The new vertex is now e->Org.
        let o = mesh.org(e);
        mesh.v[o].data = data;
        mesh.v[o].coords = *coords;
        // The winding of an edge says how the winding number changes as we
        // cross from the edge's right face to its left face.
        mesh.e[e].winding = 1;
        let es = mesh.sym(e);
        mesh.e[es].winding = -1;
        self.last_edge = e;
        true
    }

    /// `CacheVertex` (tess.c:443-453).
    fn cache_vertex(&mut self, coords: &[f64; 3], data: Option<[f64; 2]>) {
        let v = &mut self.cache[self.cache_count];
        v.data = data;
        v.coords = *coords;
        self.cache_count += 1;
    }

    /// `EmptyCache` (tess.c:456-474).
    fn empty_cache_fn(&mut self) -> bool {
        self.mesh = Some(GluMesh::mesh_new_mesh());
        for i in 0..self.cache_count {
            let v = self.cache[i];
            if !self.add_vertex(&v.coords, v.data) {
                return false;
            }
        }
        self.cache_count = 0;
        self.empty_cache = false;
        true
    }

    /// `gluTessVertex` (tess.c:477-521).
    pub fn glu_tess_vertex(&mut self, coords: &[f64; 3], data: Option<[f64; 2]>) {
        let mut too_large = false;
        let mut clamped = [0.0f64; 3];
        self.require_state(TessState::TInContour);
        if self.empty_cache {
            self.empty_cache_fn();
            self.last_edge = NIL;
        }
        for i in 0..3 {
            let mut x = coords[i];
            if x < -GLU_TESS_MAX_COORD {
                x = -GLU_TESS_MAX_COORD;
                too_large = true;
            }
            if x > GLU_TESS_MAX_COORD {
                x = GLU_TESS_MAX_COORD;
                too_large = true;
            }
            clamped[i] = x;
        }
        if too_large {
            self.call_error(GLU_TESS_COORD_TOO_LARGE);
        }
        if self.mesh.is_none() {
            if self.cache_count < TESS_MAX_CACHE {
                self.cache_vertex(&clamped, data);
                return;
            }
            self.empty_cache_fn();
        }
        self.add_vertex(&clamped, data);
    }

    /// `gluTessBeginPolygon` (tess.c:524-534).
    pub fn glu_tess_begin_polygon(&mut self) {
        self.require_state(TessState::TDormant);
        self.state = TessState::TInPolygon;
        self.cache_count = 0;
        self.empty_cache = false;
        self.mesh = None;
    }

    /// `gluTessBeginContour` (tess.c:537-551).
    pub fn glu_tess_begin_contour(&mut self) {
        self.require_state(TessState::TInPolygon);
        self.state = TessState::TInContour;
        self.last_edge = NIL;
        if self.cache_count > 0 {
            // Just set a flag so we don't get confused by empty contours
            self.empty_cache = true;
        }
    }

    /// `gluTessEndContour` (tess.c:554-559).
    pub fn glu_tess_end_contour(&mut self) {
        self.require_state(TessState::TInContour);
        self.state = TessState::TInPolygon;
    }

    /// `gluTessEndPolygon` (tess.c:562-647). The callMesh branch is not
    /// reachable (no `GLU_TESS_MESH` callback in this port's clients).
    pub fn glu_tess_end_polygon(&mut self) {
        self.require_state(TessState::TInPolygon);
        self.state = TessState::TDormant;

        if self.mesh.is_none() {
            if !self.flag_boundary {
                // Try some special code to make the easy cases go quickly
                // (eg. convex polygons).
                if self.gl_render_cache() {
                    return;
                }
            }
            self.empty_cache_fn();
        }

        // Determine the polygon normal and project vertices onto the plane
        // of the polygon.
        self.gl_project_polygon();

        // __gl_computeInterior( tess ) computes the planar arrangement specified
        // by the given contours, and further subdivides this arrangement
        // into regions.
        self.gl_compute_interior();

        if !self.fatal_error {
            // If the user wants only the boundary contours, we throw away all edges
            // except those which separate the interior from the exterior.
            // Otherwise we tessellate all the regions marked "inside".
            if self.boundary_only {
                self.mm().gl_mesh_set_winding_number(1, true);
            } else {
                self.mm().gl_mesh_tessellate_interior();
            }
            // (the rendering callbacks are registered by the client)
            if self.boundary_only {
                self.gl_render_boundary();
            } else {
                self.gl_render_mesh();
            }
        }
        self.mesh = None;
    }
}
