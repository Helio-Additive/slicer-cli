//! Faithful 1:1 port of BambuStudio `QuadricEdgeCollapse.cpp`.
//!
//! Simplify mesh by Quadric metric.
//! paper: https://people.eecs.berkeley.edu/~jrs/meshpapers/GarlandHeckbert2.pdf
//! sum up: https://users.csc.calpoly.edu/~zwood/teaching/csc570/final06/jseeba/
//! inspiration: https://github.com/sp4cerat/Fast-Quadric-Mesh-Simplification
//!
//! C++ source: src/libslic3r/QuadricEdgeCollapse.cpp (+ .hpp)
//!
//! coord_t -> i64, coordf_t -> f64 (per port rules). Triangle vertex indices are
//! `stl_triangle_vertex_indices` (Eigen int32) in C++; mirrored here as the
//! existing `Triangle` index type (`u32`). The heavy quadric math is done in
//! `f64` (Vec3d), while vertices/normals are stored as `f32` (Vec3f) exactly as
//! in C++ (`stl_vertex` / `TriangleInfo::n`).

// QuadricEdgeCollapse.cpp:1-7
use crate::mutable_priority_queue::make_miniheap_mutable_priority_queue;
use crate::triangle_mesh::{Triangle, TriangleMesh};
use std::cell::RefCell;
use std::rc::Rc;

// Lightweight 3-component f64 vector mirroring Eigen `Vec3d`.
// (Crate `Point3F` is f64 but carries unrelated mm/scaled helpers; a private
// minimal vector keeps the arithmetic byte-faithful to the C++ Eigen ops.)
#[derive(Clone, Copy, Default)]
struct Vec3d {
    x: f64,
    y: f64,
    z: f64,
}
impl Vec3d {
    #[inline]
    fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
    #[inline]
    fn x(&self) -> f64 {
        self.x
    }
    #[inline]
    fn y(&self) -> f64 {
        self.y
    }
    #[inline]
    fn z(&self) -> f64 {
        self.z
    }
    #[inline]
    fn cross(&self, o: &Vec3d) -> Vec3d {
        Vec3d::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
    #[inline]
    fn dot(&self, o: &Vec3d) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    #[inline]
    fn normalize(&mut self) {
        // Eigen `normalize()` divides by the L2 norm.
        let n = (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        self.x /= n;
        self.y /= n;
        self.z /= n;
    }
    #[inline]
    fn cast_f32(&self) -> Vec3f {
        Vec3f::new(self.x as f32, self.y as f32, self.z as f32)
    }
}
impl std::ops::Add for Vec3d {
    type Output = Vec3d;
    #[inline]
    fn add(self, o: Vec3d) -> Vec3d {
        Vec3d::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl std::ops::Sub for Vec3d {
    type Output = Vec3d;
    #[inline]
    fn sub(self, o: Vec3d) -> Vec3d {
        Vec3d::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl std::ops::Mul<f64> for Vec3d {
    type Output = Vec3d;
    #[inline]
    fn mul(self, s: f64) -> Vec3d {
        Vec3d::new(self.x * s, self.y * s, self.z * s)
    }
}
impl std::ops::Div<f64> for Vec3d {
    type Output = Vec3d;
    #[inline]
    fn div(self, s: f64) -> Vec3d {
        Vec3d::new(self.x / s, self.y / s, self.z / s)
    }
}

// Lightweight 3-component f32 vector mirroring Eigen `Vec3f` (`stl_vertex`).
#[derive(Clone, Copy, Default)]
struct Vec3f {
    x: f32,
    y: f32,
    z: f32,
}
impl Vec3f {
    #[inline]
    fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
    #[inline]
    fn dot(&self, o: &Vec3f) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    #[inline]
    fn cross(&self, o: &Vec3f) -> Vec3f {
        Vec3f::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
    #[inline]
    fn normalize(&mut self) {
        let n = (self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        self.x /= n;
        self.y /= n;
        self.z /= n;
    }
    #[inline]
    fn cast_f64(&self) -> Vec3d {
        Vec3d::new(self.x as f64, self.y as f64, self.z as f64)
    }
}
impl std::ops::Sub for Vec3f {
    type Output = Vec3f;
    #[inline]
    fn sub(self, o: Vec3f) -> Vec3f {
        Vec3f::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

// Bridge between the crate `TriangleMesh` (`Point3F`, f64 storage) and the
// f32 `stl_vertex` semantics of C++ `its.vertices`.
#[inline]
fn its_vertex(its: &TriangleMesh, vi: u32) -> Vec3f {
    let p = its.vertex(vi);
    Vec3f::new(p.x as f32, p.y as f32, p.z as f32)
}
#[inline]
fn its_set_vertex(its: &mut TriangleMesh, vi: u32, v: Vec3f) {
    its.vertices_mut()[vi as usize] = crate::geometry::Point3F::new(v.x as f64, v.y as f64, v.z as f64);
}

// only private namespace not neccessary be in .hpp
// QuadricEdgeCollapse.cpp:14
mod quadric_edge_collapse {
    use super::{Triangle, TriangleMesh, Vec3d, Vec3f};

    // SymetricMatrix
    // QuadricEdgeCollapse.cpp:16
    #[derive(Clone, Copy)]
    pub struct SymMat {
        // using T = double;  QuadricEdgeCollapse.cpp:17
        // static const constexpr size_t N = 10;  QuadricEdgeCollapse.cpp:18
        // T m[N];  QuadricEdgeCollapse.cpp:19
        pub m: [f64; 10],
    }

    impl SymMat {
        pub const N: usize = 10;

        // explicit SymMat(ArithmeticOnly<T> c = T()) { std::fill(m, m + N, c); }
        // QuadricEdgeCollapse.cpp:21
        pub fn new(c: f64) -> Self {
            SymMat { m: [c; Self::N] }
        }

        // Make plane
        // SymMat(T a, T b, T c, T d)  QuadricEdgeCollapse.cpp:24-30
        pub fn plane(a: f64, b: f64, c: f64, d: f64) -> Self {
            let mut m = [0.0f64; Self::N];
            // QuadricEdgeCollapse.cpp:26
            m[0] = a * a;
            m[1] = a * b;
            m[2] = a * c;
            m[3] = a * d;
            // QuadricEdgeCollapse.cpp:27
            m[4] = b * b;
            m[5] = b * c;
            m[6] = b * d;
            // QuadricEdgeCollapse.cpp:28
            m[7] = c * c;
            m[8] = c * d;
            // QuadricEdgeCollapse.cpp:29
            m[9] = d * d;
            SymMat { m }
        }

        // T operator[](int c) const { return m[c]; }
        // QuadricEdgeCollapse.cpp:32
        #[inline]
        pub fn at(&self, c: usize) -> f64 {
            self.m[c]
        }

        // Determinant
        // QuadricEdgeCollapse.cpp:35-44
        #[allow(clippy::too_many_arguments)]
        pub fn det(
            &self,
            a11: usize,
            a12: usize,
            a13: usize,
            a21: usize,
            a22: usize,
            a23: usize,
            a31: usize,
            a32: usize,
            a33: usize,
        ) -> f64 {
            let m = &self.m;
            // QuadricEdgeCollapse.cpp:39-41
            let det = m[a11] * m[a22] * m[a33] + m[a13] * m[a21] * m[a32] + m[a12] * m[a23] * m[a31]
                - m[a13] * m[a22] * m[a31]
                - m[a11] * m[a23] * m[a32]
                - m[a12] * m[a21] * m[a33];
            // QuadricEdgeCollapse.cpp:43
            det
        }

        // const SymMat &operator+=(const SymMat &n)  QuadricEdgeCollapse.cpp:46-50
        #[inline]
        pub fn add_assign(&mut self, n: &SymMat) {
            // QuadricEdgeCollapse.cpp:48
            for i in 0..Self::N {
                self.m[i] += n.m[i];
            }
        }
    }

    // using Vertices = std::vector<stl_vertex>;            QuadricEdgeCollapse.cpp:53
    // using Triangle = stl_triangle_vertex_indices;        QuadricEdgeCollapse.cpp:54
    // using Indices  = std::vector<stl_triangle_vertex_indices>;  QuadricEdgeCollapse.cpp:55
    // using ThrowOnCancel = std::function<void(void)>;     QuadricEdgeCollapse.cpp:56
    // using StatusFn = std::function<void(int)>;           QuadricEdgeCollapse.cpp:57
    // Vertices/Indices map onto `TriangleMesh` (its.vertices / its.indices).

    // smallest error caused by edges, identify smallest edge in triangle
    // QuadricEdgeCollapse.cpp:58-68
    #[derive(Clone, Copy)]
    pub struct Error {
        // float value = -1.; // identifying of smallest edge is stored inside of TriangleInfo
        // QuadricEdgeCollapse.cpp:61
        pub value: f32,
        // uint32_t triangle_index = 0;  QuadricEdgeCollapse.cpp:62
        pub triangle_index: u32,
    }
    impl Error {
        // Error(float value, uint32_t triangle_index)  QuadricEdgeCollapse.cpp:63-66
        pub fn new(value: f32, triangle_index: u32) -> Self {
            Error {
                value,
                triangle_index,
            }
        }
    }
    // Error() = default;  QuadricEdgeCollapse.cpp:67
    impl Default for Error {
        fn default() -> Self {
            // float value = -1.;  QuadricEdgeCollapse.cpp:61
            // uint32_t triangle_index = 0;  QuadricEdgeCollapse.cpp:62
            Error {
                value: -1.0,
                triangle_index: 0,
            }
        }
    }
    // using Errors = std::vector<Error>;  QuadricEdgeCollapse.cpp:69
    pub type Errors = Vec<Error>;

    // merge information together - faster access during processing
    // QuadricEdgeCollapse.cpp:71-81
    #[derive(Clone, Copy)]
    pub struct TriangleInfo {
        // Vec3f n; // normalized normal - used for check when fliped
        // QuadricEdgeCollapse.cpp:73
        pub n: Vec3f,
        // range(0 .. 2),
        // unsigned char min_index = 0; // identify edge for minimal Error -> lightweight Error structure
        // QuadricEdgeCollapse.cpp:76
        pub min_index: u8,
    }
    impl TriangleInfo {
        // TriangleInfo() = default;  QuadricEdgeCollapse.cpp:78
        pub fn new() -> Self {
            TriangleInfo {
                n: Vec3f::default(),
                min_index: 0,
            }
        }
        // bool is_deleted() const { return n.x() > 2.f; }  QuadricEdgeCollapse.cpp:79
        #[inline]
        pub fn is_deleted(&self) -> bool {
            self.n.x > 2.0
        }
        // void set_deleted() { n.x() = 3.f; }  QuadricEdgeCollapse.cpp:80
        #[inline]
        pub fn set_deleted(&mut self) {
            self.n.x = 3.0;
        }
    }
    // using TriangleInfos = std::vector<TriangleInfo>;  QuadricEdgeCollapse.cpp:82
    pub type TriangleInfos = Vec<TriangleInfo>;

    // QuadricEdgeCollapse.cpp:83-88
    #[derive(Clone, Copy)]
    pub struct VertexInfo {
        // SymMat q; // sum quadric of surround triangles  QuadricEdgeCollapse.cpp:84
        pub q: SymMat,
        // uint32_t start = 0, count = 0; // vertex neighbor triangles  QuadricEdgeCollapse.cpp:85
        pub start: u32,
        pub count: u32,
    }
    impl VertexInfo {
        // VertexInfo() = default;  QuadricEdgeCollapse.cpp:86
        pub fn new() -> Self {
            VertexInfo {
                q: SymMat::new(0.0),
                start: 0,
                count: 0,
            }
        }
        // bool is_deleted() const { return count == 0; }  QuadricEdgeCollapse.cpp:87
        #[inline]
        pub fn is_deleted(&self) -> bool {
            self.count == 0
        }
    }
    // using VertexInfos = std::vector<VertexInfo>;  QuadricEdgeCollapse.cpp:89
    pub type VertexInfos = Vec<VertexInfo>;

    // QuadricEdgeCollapse.cpp:90-94
    #[derive(Clone, Copy)]
    pub struct EdgeInfo {
        // uint32_t t_index=0; // triangle index  QuadricEdgeCollapse.cpp:91
        pub t_index: u32,
        // unsigned char edge = 0; // 0 or 1 or 2  QuadricEdgeCollapse.cpp:92
        pub edge: u8,
    }
    impl EdgeInfo {
        // EdgeInfo() = default;  QuadricEdgeCollapse.cpp:93
        pub fn new() -> Self {
            EdgeInfo {
                t_index: 0,
                edge: 0,
            }
        }
    }
    // using EdgeInfos = std::vector<EdgeInfo>;  QuadricEdgeCollapse.cpp:95
    pub type EdgeInfos = Vec<EdgeInfo>;

    // DTO for change neighbors
    // QuadricEdgeCollapse.cpp:97-105
    #[derive(Clone, Copy)]
    pub struct CopyEdgeInfo {
        // uint32_t start;  QuadricEdgeCollapse.cpp:99
        pub start: u32,
        // uint32_t count;  QuadricEdgeCollapse.cpp:100
        pub count: u32,
        // uint32_t move;   QuadricEdgeCollapse.cpp:101
        pub move_: u32,
    }
    impl CopyEdgeInfo {
        // CopyEdgeInfo(uint32_t start, uint32_t count, uint32_t move)  QuadricEdgeCollapse.cpp:102-104
        pub fn new(start: u32, count: u32, move_: u32) -> Self {
            CopyEdgeInfo { start, count, move_ }
        }
    }
    // using CopyEdgeInfos = std::vector<CopyEdgeInfo>;  QuadricEdgeCollapse.cpp:106
    pub type CopyEdgeInfos = Vec<CopyEdgeInfo>;

    // constants --> may be move to config
    // QuadricEdgeCollapse.cpp:145-155
    // const uint32_t check_cancel_period = 16; // how many edge to reduce before call throw_on_cancel
    pub const CHECK_CANCEL_PERIOD: u32 = 16;
    // const size_t max_triangle_count_for_one_vertex = 50;
    pub const MAX_TRIANGLE_COUNT_FOR_ONE_VERTEX: usize = 50;
    // change speed of progress bargraph
    // const int status_init_size = 10; // in percents
    pub const STATUS_INIT_SIZE: i32 = 10;
    // parts of init size
    // const int status_normal_size = 25;
    pub const STATUS_NORMAL_SIZE: usize = 25;
    // const int status_sum_quadric = 25;
    pub const STATUS_SUM_QUADRIC: usize = 25;
    // const int status_set_offsets = 10;
    pub const STATUS_SET_OFFSETS: i32 = 10;
    // const int status_calc_errors = 30;
    pub const STATUS_CALC_ERRORS: usize = 30;
    // const int status_create_refs = 10;
    pub const STATUS_CREATE_REFS: usize = 10;

    // ----- function bodies -----

    // QuadricEdgeCollapse.cpp:349-359
    pub fn create_normal(triangle: &Triangle, its: &TriangleMesh) -> Vec3d {
        // Vec3d v0 = vertices[triangle[0]].cast<double>();  QuadricEdgeCollapse.cpp:352
        let v0 = super::its_vertex(its, triangle.indices[0]).cast_f64();
        // Vec3d v1 = vertices[triangle[1]].cast<double>();  QuadricEdgeCollapse.cpp:353
        let v1 = super::its_vertex(its, triangle.indices[1]).cast_f64();
        // Vec3d v2 = vertices[triangle[2]].cast<double>();  QuadricEdgeCollapse.cpp:354
        let v2 = super::its_vertex(its, triangle.indices[2]).cast_f64();
        // n = triangle normal
        // Vec3d n = (v1 - v0).cross(v2 - v0);  QuadricEdgeCollapse.cpp:356
        let mut n = (v1 - v0).cross(&(v2 - v0));
        // n.normalize();  QuadricEdgeCollapse.cpp:357
        n.normalize();
        n
    }

    // QuadricEdgeCollapse.cpp:361-364
    pub fn calculate_determinant(q: &SymMat) -> f64 {
        // return q.det(0, 1, 2, 1, 4, 5, 2, 5, 7);  QuadricEdgeCollapse.cpp:363
        q.det(0, 1, 2, 1, 4, 5, 2, 5, 7)
    }

    // QuadricEdgeCollapse.cpp:366-372
    pub fn calculate_vertex_det(det: f64, q: &SymMat) -> Vec3d {
        // double det_1 = -1 / det;  QuadricEdgeCollapse.cpp:367
        let det_1 = -1.0 / det;
        // double det_x = q.det(1, 2, 3, 4, 5, 6, 5, 7, 8); // vx = A41/det(q_delta)  QuadricEdgeCollapse.cpp:368
        let det_x = q.det(1, 2, 3, 4, 5, 6, 5, 7, 8);
        // double det_y = q.det(0, 2, 3, 1, 5, 6, 2, 7, 8); // vy = A42/det(q_delta)  QuadricEdgeCollapse.cpp:369
        let det_y = q.det(0, 2, 3, 1, 5, 6, 2, 7, 8);
        // double det_z = q.det(0, 1, 3, 1, 4, 6, 2, 5, 8); // vz = A43/det(q_delta)  QuadricEdgeCollapse.cpp:370
        let det_z = q.det(0, 1, 3, 1, 4, 6, 2, 5, 8);
        // return Vec3d(det_1 * det_x, -det_1 * det_y, det_1 * det_z);  QuadricEdgeCollapse.cpp:371
        Vec3d::new(det_1 * det_x, -det_1 * det_y, det_1 * det_z)
    }

    // QuadricEdgeCollapse.cpp:374-380
    pub fn create_vertices(id_v1: u32, id_v2: u32, its: &TriangleMesh) -> [Vec3d; 3] {
        // Vec3d v0 = vertices[id_v1].cast<double>();  QuadricEdgeCollapse.cpp:376
        let v0 = super::its_vertex(its, id_v1).cast_f64();
        // Vec3d v1 = vertices[id_v2].cast<double>();  QuadricEdgeCollapse.cpp:377
        let v1 = super::its_vertex(its, id_v2).cast_f64();
        // Vec3d vm = (v0 + v1) / 2.;  QuadricEdgeCollapse.cpp:378
        let vm = (v0 + v1) / 2.0;
        // return {v0, v1, vm};  QuadricEdgeCollapse.cpp:379
        [v0, v1, vm]
    }

    // QuadricEdgeCollapse.cpp:382-389
    pub fn vertices_error(q: &SymMat, vertices: &[Vec3d; 3]) -> [f64; 3] {
        // QuadricEdgeCollapse.cpp:385-388
        [
            vertex_error(q, &vertices[0]),
            vertex_error(q, &vertices[1]),
            vertex_error(q, &vertices[2]),
        ]
    }

    // QuadricEdgeCollapse.cpp:391-405
    pub fn calculate_error_v(id_v1: u32, id_v2: u32, q: &SymMat, its: &TriangleMesh) -> f64 {
        // double det = calculate_determinant(q);  QuadricEdgeCollapse.cpp:396
        let det = calculate_determinant(q);
        // if (std::abs(det) < std::numeric_limits<double>::epsilon()) {  QuadricEdgeCollapse.cpp:397
        if det.abs() < f64::EPSILON {
            // can't divide by zero
            // auto verts  = create_vertices(id_v1, id_v2, vertices);  QuadricEdgeCollapse.cpp:399
            let verts = create_vertices(id_v1, id_v2, its);
            // auto errors = vertices_error(q, verts);  QuadricEdgeCollapse.cpp:400
            let errors = vertices_error(q, &verts);
            // return *std::min_element(std::begin(errors), std::end(errors));  QuadricEdgeCollapse.cpp:401
            return min_element(&errors);
        }
        // Vec3d vertex = calculate_vertex(det, q);  QuadricEdgeCollapse.cpp:403
        let vertex = calculate_vertex_det(det, q);
        // return vertex_error(q, vertex);  QuadricEdgeCollapse.cpp:404
        vertex_error(q, &vertex)
    }

    // similar as calculate error but focus on new vertex without calculation of error
    // QuadricEdgeCollapse.cpp:408-422
    pub fn calculate_vertex(id_v1: u32, id_v2: u32, q: &SymMat, its: &TriangleMesh) -> Vec3f {
        // double det = calculate_determinant(q);  QuadricEdgeCollapse.cpp:413
        let det = calculate_determinant(q);
        // if (std::abs(det) < std::numeric_limits<double>::epsilon()) {  QuadricEdgeCollapse.cpp:414
        if det.abs() < f64::EPSILON {
            // can't divide by zero
            // auto verts  = create_vertices(id_v1, id_v2, vertices);  QuadricEdgeCollapse.cpp:416
            let verts = create_vertices(id_v1, id_v2, its);
            // auto errors = vertices_error(q, verts);  QuadricEdgeCollapse.cpp:417
            let errors = vertices_error(q, &verts);
            // auto mit = std::min_element(std::begin(errors), std::end(errors));  QuadricEdgeCollapse.cpp:418
            let mit = min_element_index(&errors);
            // return verts[mit - std::begin(errors)].cast<float>();  QuadricEdgeCollapse.cpp:419
            return verts[mit].cast_f32();
        }
        // return calculate_vertex(det, q).cast<float>();  QuadricEdgeCollapse.cpp:421
        calculate_vertex_det(det, q).cast_f32()
    }

    // QuadricEdgeCollapse.cpp:424-430
    pub fn vertex_error(q: &SymMat, vertex: &Vec3d) -> f64 {
        // const double &x = vertex.x(), &y = vertex.y(), &z = vertex.z();  QuadricEdgeCollapse.cpp:426
        let x = vertex.x();
        let y = vertex.y();
        let z = vertex.z();
        // QuadricEdgeCollapse.cpp:427-429
        q.at(0) * x * x
            + 2.0 * q.at(1) * x * y
            + 2.0 * q.at(2) * x * z
            + 2.0 * q.at(3) * x
            + q.at(4) * y * y
            + 2.0 * q.at(5) * y * z
            + 2.0 * q.at(6) * y
            + q.at(7) * z * z
            + 2.0 * q.at(8) * z
            + q.at(9)
    }

    // QuadricEdgeCollapse.cpp:432-438
    pub fn create_quadric(t: &Triangle, n: &Vec3d, its: &TriangleMesh) -> SymMat {
        // Vec3d v0 = vertices[t[0]].cast<double>();  QuadricEdgeCollapse.cpp:436
        let v0 = super::its_vertex(its, t.indices[0]).cast_f64();
        // return SymMat(n.x(), n.y(), n.z(), -n.dot(v0));  QuadricEdgeCollapse.cpp:437
        SymMat::plane(n.x(), n.y(), n.z(), -n.dot(&v0))
    }

    // std::min_element helpers (faithful to C++ first-min selection on ties).
    #[inline]
    fn min_element(errors: &[f64; 3]) -> f64 {
        errors[min_element_index(errors)]
    }
    #[inline]
    fn min_element_index(errors: &[f64; 3]) -> usize {
        // std::min_element returns the first element that is strictly less than
        // the others (keeps the earlier index on ties).
        let mut idx = 0usize;
        for i in 1..errors.len() {
            if errors[i] < errors[idx] {
                idx = i;
            }
        }
        idx
    }

    // QuadricEdgeCollapse.cpp:440-536
    #[allow(clippy::type_complexity)]
    pub fn init(
        its: &TriangleMesh,
        throw_on_cancel: &mut dyn FnMut(),
        status_fn: &mut dyn FnMut(i32),
    ) -> (TriangleInfos, VertexInfos, EdgeInfos, Errors) {
        // int status_offset = 0;  QuadricEdgeCollapse.cpp:443
        let mut status_offset: usize = 0;
        // TriangleInfos t_infos(its.indices.size());  QuadricEdgeCollapse.cpp:444
        let mut t_infos: TriangleInfos = vec![TriangleInfo::new(); its.indices().len()];
        // VertexInfos   v_infos(its.vertices.size());  QuadricEdgeCollapse.cpp:445
        let mut v_infos: VertexInfos = vec![VertexInfo::new(); its.vertices().len()];
        {
            // std::vector<SymMat> triangle_quadrics(its.indices.size());  QuadricEdgeCollapse.cpp:447
            let mut triangle_quadrics: Vec<SymMat> = vec![SymMat::new(0.0); its.indices().len()];
            // calculate normals
            // tbb::parallel_for(...)  QuadricEdgeCollapse.cpp:449-462 (ported serially)
            for i in 0..its.indices().len() {
                // const Triangle &t = its.indices[i];  QuadricEdgeCollapse.cpp:452
                let t = its.indices()[i];
                // Vec3d normal = create_normal(t, its.vertices);  QuadricEdgeCollapse.cpp:454
                let normal = create_normal(&t, its);
                // t_info.n = normal.cast<float>();  QuadricEdgeCollapse.cpp:455
                t_infos[i].n = normal.cast_f32();
                // triangle_quadrics[i] = create_quadric(t, normal, its.vertices);  QuadricEdgeCollapse.cpp:456
                triangle_quadrics[i] = create_quadric(&t, &normal, its);
                // QuadricEdgeCollapse.cpp:457-460
                if i % 1_000_000 == 0 {
                    throw_on_cancel();
                    status_fn(
                        (status_offset + (i * STATUS_NORMAL_SIZE) / its.indices().len()) as i32,
                    );
                }
            }
            // END parallel for
            // status_offset += status_normal_size;  QuadricEdgeCollapse.cpp:463
            status_offset += STATUS_NORMAL_SIZE;

            // sum quadrics
            // QuadricEdgeCollapse.cpp:466-478
            for i in 0..its.indices().len() {
                // const Triangle &t = its.indices[i];  QuadricEdgeCollapse.cpp:467
                let t = its.indices()[i];
                // const SymMat &q = triangle_quadrics[i];  QuadricEdgeCollapse.cpp:468
                let q = triangle_quadrics[i];
                // QuadricEdgeCollapse.cpp:469-473
                for e in 0..3 {
                    // VertexInfo &v_info = v_infos[t[e]];  QuadricEdgeCollapse.cpp:470
                    let v_info = &mut v_infos[t.indices[e] as usize];
                    // v_info.q += q;  QuadricEdgeCollapse.cpp:471
                    v_info.q.add_assign(&q);
                    // ++v_info.count; // triangle count  QuadricEdgeCollapse.cpp:472
                    v_info.count += 1;
                }
                // QuadricEdgeCollapse.cpp:474-477
                if i % 1_000_000 == 0 {
                    throw_on_cancel();
                    status_fn(
                        (status_offset + (i * STATUS_SUM_QUADRIC) / its.indices().len()) as i32,
                    );
                }
            }
            // status_offset += status_sum_quadric;  QuadricEdgeCollapse.cpp:479
            status_offset += STATUS_SUM_QUADRIC;
        } // remove triangle quadrics

        // set offseted starts
        // QuadricEdgeCollapse.cpp:483-489
        let mut triangle_start: u32 = 0;
        for v_info in v_infos.iter_mut() {
            // v_info.start = triangle_start;  QuadricEdgeCollapse.cpp:485
            v_info.start = triangle_start;
            // triangle_start += v_info.count;  QuadricEdgeCollapse.cpp:486
            triangle_start += v_info.count;
            // set filled vertex to zero
            // v_info.count = 0;  QuadricEdgeCollapse.cpp:488
            v_info.count = 0;
        }
        // assert(its.indices.size() * 3 == triangle_start);  QuadricEdgeCollapse.cpp:490
        debug_assert_eq!(its.indices().len() * 3, triangle_start as usize);

        // status_offset += status_set_offsets;  QuadricEdgeCollapse.cpp:492
        status_offset += STATUS_SET_OFFSETS as usize;
        // throw_on_cancel();  QuadricEdgeCollapse.cpp:493
        throw_on_cancel();
        // status_fn(status_offset);  QuadricEdgeCollapse.cpp:494
        status_fn(status_offset as i32);

        // calc error
        // Errors errors(its.indices.size());  QuadricEdgeCollapse.cpp:497
        let mut errors: Errors = vec![Error::default(); its.indices().len()];
        // tbb::parallel_for(...)  QuadricEdgeCollapse.cpp:498-510 (ported serially)
        for i in 0..its.indices().len() {
            // const Triangle &t = its.indices[i];  QuadricEdgeCollapse.cpp:501
            let t = its.indices()[i];
            // errors[i] = calculate_error(i, t, its.vertices, v_infos, t_info.min_index);  QuadricEdgeCollapse.cpp:503
            let mut min_index = t_infos[i].min_index;
            errors[i] = calculate_error(i as u32, &t, its, &v_infos, &mut min_index);
            t_infos[i].min_index = min_index;
            // QuadricEdgeCollapse.cpp:504-507
            if i % 1_000_000 == 0 {
                throw_on_cancel();
                status_fn((status_offset + (i * STATUS_CALC_ERRORS) / its.indices().len()) as i32);
            }
            // if (i % 1000000 == 0) throw_on_cancel();  QuadricEdgeCollapse.cpp:508
            if i % 1_000_000 == 0 {
                throw_on_cancel();
            }
        }
        // END parallel for

        // status_offset += status_calc_errors;  QuadricEdgeCollapse.cpp:512
        status_offset += STATUS_CALC_ERRORS;

        // create reference
        // EdgeInfos e_infos(its.indices.size() * 3);  QuadricEdgeCollapse.cpp:515
        let mut e_infos: EdgeInfos = vec![EdgeInfo::new(); its.indices().len() * 3];
        // QuadricEdgeCollapse.cpp:516-531
        for i in 0..its.indices().len() {
            // const Triangle &t = its.indices[i];  QuadricEdgeCollapse.cpp:517
            let t = its.indices()[i];
            for j in 0..3 {
                // VertexInfo &v_info = v_infos[t[j]];  QuadricEdgeCollapse.cpp:519
                let v_info = &mut v_infos[t.indices[j] as usize];
                // size_t ei = v_info.start + v_info.count;  QuadricEdgeCollapse.cpp:520
                let ei = (v_info.start + v_info.count) as usize;
                // assert(ei < e_infos.size());  QuadricEdgeCollapse.cpp:521
                debug_assert!(ei < e_infos.len());
                // EdgeInfo &e_info = e_infos[ei];  QuadricEdgeCollapse.cpp:522
                let e_info = &mut e_infos[ei];
                // e_info.t_index = i;  QuadricEdgeCollapse.cpp:523
                e_info.t_index = i as u32;
                // e_info.edge = j;  QuadricEdgeCollapse.cpp:524
                e_info.edge = j as u8;
                // ++v_info.count;  QuadricEdgeCollapse.cpp:525
                v_infos[t.indices[j] as usize].count += 1;
            }
            // QuadricEdgeCollapse.cpp:527-530
            if i % 1_000_000 == 0 {
                throw_on_cancel();
                status_fn((status_offset + (i * STATUS_CREATE_REFS) / its.indices().len()) as i32);
            }
        }

        // throw_on_cancel();  QuadricEdgeCollapse.cpp:533
        throw_on_cancel();
        // status_fn(100);  QuadricEdgeCollapse.cpp:534
        status_fn(100);
        // return {t_infos, v_infos, e_infos, errors};  QuadricEdgeCollapse.cpp:535
        (t_infos, v_infos, e_infos, errors)
    }

    // QuadricEdgeCollapse.cpp:538-556
    pub fn find_triangle_index1(
        vi: u32,
        v_info: &VertexInfo,
        ti0: u32,
        e_infos: &EdgeInfos,
        its: &TriangleMesh,
    ) -> Option<u32> {
        // coord_t vi_coord = static_cast<coord_t>(vi);  QuadricEdgeCollapse.cpp:544
        let vi_coord: i64 = vi as i64;
        // uint32_t end = v_info.start + v_info.count;  QuadricEdgeCollapse.cpp:545
        let end = v_info.start + v_info.count;
        // QuadricEdgeCollapse.cpp:546-553
        let mut ei = v_info.start;
        while ei < end {
            // const EdgeInfo &e_info = e_infos[ei];  QuadricEdgeCollapse.cpp:547
            let e_info = e_infos[ei as usize];
            // if (e_info.t_index == ti0) continue;  QuadricEdgeCollapse.cpp:548
            if e_info.t_index == ti0 {
                ei += 1;
                continue;
            }
            // const Triangle& t = indices[e_info.t_index];  QuadricEdgeCollapse.cpp:549
            let t = its.indices()[e_info.t_index as usize];
            // if (t[(e_info.edge + 1) % 3] == vi_coord ||
            //     t[(e_info.edge + 2) % 3] == vi_coord)  QuadricEdgeCollapse.cpp:550-551
            if t.indices[((e_info.edge as usize) + 1) % 3] as i64 == vi_coord
                || t.indices[((e_info.edge as usize) + 2) % 3] as i64 == vi_coord
            {
                // return e_info.t_index;  QuadricEdgeCollapse.cpp:552
                return Some(e_info.t_index);
            }
            ei += 1;
        }
        // triangle0 is on border and do NOT have twin edge
        // return {};  QuadricEdgeCollapse.cpp:555
        None
    }

    // QuadricEdgeCollapse.cpp:558-588
    pub fn reorder_edges(e_infos: &mut EdgeInfos, v_info: &VertexInfo, ti0: u32, ti1: u32) {
        // swap edge info of ti0 and ti1 to end(last one and one before)
        // size_t v_info_end = v_info.start + v_info.count - 2;  QuadricEdgeCollapse.cpp:564
        let v_info_end = (v_info.start + v_info.count - 2) as usize;
        // EdgeInfo &e_info_ti0 = e_infos[v_info_end];      QuadricEdgeCollapse.cpp:565
        // EdgeInfo &e_info_ti1 = e_infos[v_info_end+1];    QuadricEdgeCollapse.cpp:566
        let idx_ti0 = v_info_end;
        let idx_ti1 = v_info_end + 1;
        // bool is_swaped = false;  QuadricEdgeCollapse.cpp:567
        let mut is_swaped = false;
        // QuadricEdgeCollapse.cpp:568-587
        for ei in (v_info.start as usize)..v_info_end {
            // EdgeInfo &e_info = e_infos[ei];  QuadricEdgeCollapse.cpp:569
            // if (e_info.t_index == ti0) {  QuadricEdgeCollapse.cpp:570
            if e_infos[ei].t_index == ti0 {
                // std::swap(e_info, e_info_ti0);  QuadricEdgeCollapse.cpp:571
                e_infos.swap(ei, idx_ti0);
                // if (is_swaped) return;  QuadricEdgeCollapse.cpp:572
                if is_swaped {
                    return;
                }
                // if (e_info.t_index == ti1) {  QuadricEdgeCollapse.cpp:573
                if e_infos[ei].t_index == ti1 {
                    // std::swap(e_info, e_info_ti1);  QuadricEdgeCollapse.cpp:574
                    e_infos.swap(ei, idx_ti1);
                    // return;  QuadricEdgeCollapse.cpp:575
                    return;
                }
                // is_swaped = true;  QuadricEdgeCollapse.cpp:577
                is_swaped = true;
            // } else if (e_info.t_index == ti1) {  QuadricEdgeCollapse.cpp:578
            } else if e_infos[ei].t_index == ti1 {
                // std::swap(e_info, e_info_ti1);  QuadricEdgeCollapse.cpp:579
                e_infos.swap(ei, idx_ti1);
                // if (is_swaped) return;  QuadricEdgeCollapse.cpp:580
                if is_swaped {
                    return;
                }
                // if (e_info.t_index == ti0) {  QuadricEdgeCollapse.cpp:581
                if e_infos[ei].t_index == ti0 {
                    // std::swap(e_info, e_info_ti0);  QuadricEdgeCollapse.cpp:582
                    e_infos.swap(ei, idx_ti0);
                    // return;  QuadricEdgeCollapse.cpp:583
                    return;
                }
                // is_swaped = true;  QuadricEdgeCollapse.cpp:585
                is_swaped = true;
            }
        }
    }

    // QuadricEdgeCollapse.cpp:590-625
    #[allow(clippy::too_many_arguments)]
    pub fn is_flipped(
        new_vertex: &Vec3f,
        _ti0: u32,
        _ti1: u32,
        v_info: &VertexInfo,
        t_infos: &TriangleInfos,
        e_infos: &EdgeInfos,
        its: &TriangleMesh,
    ) -> bool {
        // static const float thr_pos = 1.0f - std::numeric_limits<float>::epsilon();  QuadricEdgeCollapse.cpp:598
        let thr_pos: f32 = 1.0 - f32::EPSILON;
        // static const float thr_neg = -thr_pos;  QuadricEdgeCollapse.cpp:599
        let thr_neg: f32 = -thr_pos;
        // static const float dot_thr = 0.2f; // Value from simplify mesh cca 80 DEG  QuadricEdgeCollapse.cpp:600
        let dot_thr: f32 = 0.2;

        // for each vertex triangles
        // size_t v_info_end = v_info.start + v_info.count-2;  QuadricEdgeCollapse.cpp:603
        let v_info_end = (v_info.start + v_info.count - 2) as usize;
        // QuadricEdgeCollapse.cpp:604-623
        for ei in (v_info.start as usize)..v_info_end {
            // assert(ei < e_infos.size());  QuadricEdgeCollapse.cpp:605
            debug_assert!(ei < e_infos.len());
            // const EdgeInfo &e_info = e_infos[ei];  QuadricEdgeCollapse.cpp:606
            let e_info = e_infos[ei];
            // const Triangle &t = its.indices[e_info.t_index];  QuadricEdgeCollapse.cpp:607
            let t = its.indices()[e_info.t_index as usize];
            // const Vec3f &normal = t_infos[e_info.t_index].n;  QuadricEdgeCollapse.cpp:608
            let normal = t_infos[e_info.t_index as usize].n;
            // const Vec3f &vf = its.vertices[t[(e_info.edge + 1) % 3]];  QuadricEdgeCollapse.cpp:609
            let vf = super::its_vertex(its, t.indices[((e_info.edge as usize) + 1) % 3]);
            // const Vec3f &vs = its.vertices[t[(e_info.edge + 2) % 3]];  QuadricEdgeCollapse.cpp:610
            let vs = super::its_vertex(its, t.indices[((e_info.edge as usize) + 2) % 3]);

            // Vec3f d1 = vf - new_vertex;  QuadricEdgeCollapse.cpp:612
            let mut d1 = vf - *new_vertex;
            // d1.normalize();  QuadricEdgeCollapse.cpp:613
            d1.normalize();
            // Vec3f d2 = vs - new_vertex;  QuadricEdgeCollapse.cpp:614
            let mut d2 = vs - *new_vertex;
            // d2.normalize();  QuadricEdgeCollapse.cpp:615
            d2.normalize();

            // float dot = d1.dot(d2);  QuadricEdgeCollapse.cpp:617
            let dot = d1.dot(&d2);
            // if (dot > thr_pos || dot < thr_neg) return true;  QuadricEdgeCollapse.cpp:618
            if dot > thr_pos || dot < thr_neg {
                return true;
            }
            // IMPROVE: propagate new normal
            // Vec3f n = d1.cross(d2);  QuadricEdgeCollapse.cpp:620
            let mut n = d1.cross(&d2);
            // n.normalize();  QuadricEdgeCollapse.cpp:621
            n.normalize();
            // if(n.dot(normal) < dot_thr) return true;  QuadricEdgeCollapse.cpp:622
            if n.dot(&normal) < dot_thr {
                return true;
            }
        }
        // return false;  QuadricEdgeCollapse.cpp:624
        false
    }

    // QuadricEdgeCollapse.cpp:627-645
    pub fn degenerate(
        vi: u32,
        _ti0: u32,
        _ti1: u32,
        v_info: &VertexInfo,
        e_infos: &EdgeInfos,
        its: &TriangleMesh,
    ) -> bool {
        // check surround triangle do not contain vertex index
        // protect from creation of triangle with two same vertices inside
        // size_t v_info_end = v_info.start + v_info.count - 2;  QuadricEdgeCollapse.cpp:636
        let v_info_end = (v_info.start + v_info.count - 2) as usize;
        // QuadricEdgeCollapse.cpp:637-643
        for ei in (v_info.start as usize)..v_info_end {
            // assert(ei < e_infos.size());  QuadricEdgeCollapse.cpp:638
            debug_assert!(ei < e_infos.len());
            // const EdgeInfo &e_info = e_infos[ei];  QuadricEdgeCollapse.cpp:639
            let e_info = e_infos[ei];
            // const Triangle &t = indices[e_info.t_index];  QuadricEdgeCollapse.cpp:640
            let t = its.indices()[e_info.t_index as usize];
            // QuadricEdgeCollapse.cpp:641-642
            for i in 0..3 {
                // if (static_cast<uint32_t>(t[i]) == vi) return true;
                if t.indices[i] == vi {
                    return true;
                }
            }
        }
        // return false;  QuadricEdgeCollapse.cpp:644
        false
    }

    // QuadricEdgeCollapse.cpp:647-691
    #[allow(clippy::too_many_arguments)]
    pub fn create_no_volume(
        vi0: u32,
        vi1: u32,
        _ti0: u32,
        _ti1: u32,
        v_info0: &VertexInfo,
        v_info1: &VertexInfo,
        e_infos: &EdgeInfos,
        its: &TriangleMesh,
    ) -> bool {
        // check that triangles around vertex0 doesn't have half edge
        // with opposit order in set of triangles around vertex1
        // protect from creation of two triangles with oposit order - no volume space
        // size_t v_info0_end = v_info0.start + v_info0.count - 2;  QuadricEdgeCollapse.cpp:656
        let v_info0_end = (v_info0.start + v_info0.count - 2) as usize;
        // size_t v_info1_end = v_info1.start + v_info1.count - 2;  QuadricEdgeCollapse.cpp:657
        let v_info1_end = (v_info1.start + v_info1.count - 2) as usize;
        // QuadricEdgeCollapse.cpp:658-689
        for ei0 in (v_info0.start as usize)..v_info0_end {
            // const EdgeInfo &e_info0 = e_infos[ei0];  QuadricEdgeCollapse.cpp:659
            let e_info0 = e_infos[ei0];
            // const Triangle &t0 = indices[e_info0.t_index];  QuadricEdgeCollapse.cpp:660
            let t0 = its.indices()[e_info0.t_index as usize];
            // edge CCW vertex indices are t0vi0, t0vi1
            // size_t t0i = 0;  QuadricEdgeCollapse.cpp:662
            let mut t0i = 0usize;
            // uint32_t t0vi0 = static_cast<uint32_t>(t0[t0i]);  QuadricEdgeCollapse.cpp:663
            let mut t0vi0 = t0.indices[t0i];
            // if (t0vi0 == vi0) {  QuadricEdgeCollapse.cpp:664
            if t0vi0 == vi0 {
                // ++t0i;  QuadricEdgeCollapse.cpp:665
                t0i += 1;
                // t0vi0 = static_cast<uint32_t>(t0[t0i]);  QuadricEdgeCollapse.cpp:666
                t0vi0 = t0.indices[t0i];
            }
            // ++t0i;  QuadricEdgeCollapse.cpp:668
            t0i += 1;
            // uint32_t t0vi1 = static_cast<uint32_t>(t0[t0i]);  QuadricEdgeCollapse.cpp:669
            let mut t0vi1 = t0.indices[t0i];
            // if (t0vi1 == vi0) {  QuadricEdgeCollapse.cpp:670
            if t0vi1 == vi0 {
                // ++t0i;  QuadricEdgeCollapse.cpp:671
                t0i += 1;
                // t0vi1 = static_cast<uint32_t>(t0[t0i]);  QuadricEdgeCollapse.cpp:672
                t0vi1 = t0.indices[t0i];
            }
            // QuadricEdgeCollapse.cpp:674-688
            for ei1 in (v_info1.start as usize)..v_info1_end {
                // const EdgeInfo &e_info1 = e_infos[ei1];  QuadricEdgeCollapse.cpp:675
                let e_info1 = e_infos[ei1];
                // const Triangle &t1 = indices[e_info1.t_index];  QuadricEdgeCollapse.cpp:676
                let t1 = its.indices()[e_info1.t_index as usize];
                // size_t t1i = 0;  QuadricEdgeCollapse.cpp:677
                let mut t1i = 0usize;
                // for (; t1i < 3; ++t1i) if (static_cast<uint32_t>(t1[t1i]) == t0vi1) break;  QuadricEdgeCollapse.cpp:678
                while t1i < 3 {
                    if t1.indices[t1i] == t0vi1 {
                        break;
                    }
                    t1i += 1;
                }
                // if (t1i >= 3) continue; // without vertex index from triangle 0  QuadricEdgeCollapse.cpp:679
                if t1i >= 3 {
                    continue;
                }
                // check if second index is same too
                // ++t1i;  QuadricEdgeCollapse.cpp:681
                t1i += 1;
                // if (t1i == 3) t1i = 0; // triangle loop(modulo 3)  QuadricEdgeCollapse.cpp:682
                if t1i == 3 {
                    t1i = 0;
                }
                // if (static_cast<uint32_t>(t1[t1i]) == vi1) {  QuadricEdgeCollapse.cpp:683
                if t1.indices[t1i] == vi1 {
                    // ++t1i;  QuadricEdgeCollapse.cpp:684
                    t1i += 1;
                    // if (t1i == 3) t1i = 0; // triangle loop(modulo 3)  QuadricEdgeCollapse.cpp:685
                    if t1i == 3 {
                        t1i = 0;
                    }
                }
                // if (static_cast<uint32_t>(t1[t1i]) == t0vi0) return true;  QuadricEdgeCollapse.cpp:687
                if t1.indices[t1i] == t0vi0 {
                    return true;
                }
            }
        }
        // return false;  QuadricEdgeCollapse.cpp:690
        false
    }

    // QuadricEdgeCollapse.cpp:693-707
    pub fn calculate_3errors(t: &Triangle, its: &TriangleMesh, v_infos: &VertexInfos) -> Vec3d {
        // Vec3d error;  QuadricEdgeCollapse.cpp:697
        let mut error = Vec3d::default();
        // QuadricEdgeCollapse.cpp:698-705
        for j in 0..3usize {
            // size_t j2 = (j == 2) ? 0 : (j + 1);  QuadricEdgeCollapse.cpp:699
            let j2 = if j == 2 { 0 } else { j + 1 };
            // uint32_t vi0 = t[j];  QuadricEdgeCollapse.cpp:700
            let vi0 = t.indices[j];
            // uint32_t vi1 = t[j2];  QuadricEdgeCollapse.cpp:701
            let vi1 = t.indices[j2];
            // SymMat q(v_infos[vi0].q); // copy  QuadricEdgeCollapse.cpp:702
            let mut q = v_infos[vi0 as usize].q;
            // q += v_infos[vi1].q;  QuadricEdgeCollapse.cpp:703
            q.add_assign(&v_infos[vi1 as usize].q);
            // error[j] = calculate_error(vi0, vi1, q, vertices);  QuadricEdgeCollapse.cpp:704
            let e = calculate_error_v(vi0, vi1, &q, its);
            match j {
                0 => error.x = e,
                1 => error.y = e,
                _ => error.z = e,
            }
        }
        // return error;  QuadricEdgeCollapse.cpp:706
        error
    }

    // QuadricEdgeCollapse.cpp:709-720
    pub fn calculate_error(
        ti: u32,
        t: &Triangle,
        its: &TriangleMesh,
        v_infos: &VertexInfos,
        min_index: &mut u8,
    ) -> Error {
        // Vec3d error = calculate_3errors(t, vertices, v_infos);  QuadricEdgeCollapse.cpp:715
        let error = calculate_3errors(t, its, v_infos);
        // select min error
        // min_index = (error[0] < error[1]) ? ((error[0] < error[2]) ? 0 : 2) :
        //                                     ((error[1] < error[2]) ? 1 : 2);  QuadricEdgeCollapse.cpp:717-718
        *min_index = if error.x < error.y {
            if error.x < error.z {
                0
            } else {
                2
            }
        } else if error.y < error.z {
            1
        } else {
            2
        };
        // return Error(static_cast<float>(error[min_index]), ti);  QuadricEdgeCollapse.cpp:719
        let e_val = match *min_index {
            0 => error.x,
            1 => error.y,
            _ => error.z,
        };
        Error::new(e_val as f32, ti)
    }

    // QuadricEdgeCollapse.cpp:722-738
    pub fn remove_triangle(e_infos: &mut EdgeInfos, v_info: &mut VertexInfo, ti: u32) {
        // auto e_info     = e_infos.begin() + v_info.start;  QuadricEdgeCollapse.cpp:726
        // auto e_info_end = e_info + v_info.count - 1;  QuadricEdgeCollapse.cpp:727
        let start = v_info.start as usize;
        let end = start + (v_info.count as usize) - 1; // index of e_info_end element
        // for (; e_info != e_info_end; ++e_info) {  QuadricEdgeCollapse.cpp:728-734
        let mut idx = start;
        while idx != end {
            // if (e_info->t_index == ti) {  QuadricEdgeCollapse.cpp:729
            if e_infos[idx].t_index == ti {
                // *e_info = *e_info_end;  QuadricEdgeCollapse.cpp:730
                e_infos[idx] = e_infos[end];
                // --v_info.count;  QuadricEdgeCollapse.cpp:731
                v_info.count -= 1;
                // return;  QuadricEdgeCollapse.cpp:732
                return;
            }
            idx += 1;
        }
        // assert(e_info_end->t_index == ti);  QuadricEdgeCollapse.cpp:735
        debug_assert_eq!(e_infos[end].t_index, ti);
        // last triangle is ti
        // --v_info.count;  QuadricEdgeCollapse.cpp:737
        v_info.count -= 1;
    }

    // QuadricEdgeCollapse.cpp:740-826
    #[allow(clippy::too_many_arguments)]
    pub fn change_neighbors(
        e_infos: &mut EdgeInfos,
        v_infos: &mut VertexInfos,
        ti0: u32,
        ti1: u32,
        vi0: u32,
        vi1: u32,
        vi_top0: u32,
        t1: &Triangle,
        infos: &mut CopyEdgeInfos,
        e_infos1: &mut EdgeInfos,
    ) {
        // have to copy Edge info from higher vertex index into smaller
        // assert(vi0 < vi1);  QuadricEdgeCollapse.cpp:752
        debug_assert!(vi0 < vi1);

        // vertex index of triangle 1 which is not vi0 nor vi1
        // uint32_t vi_top1 = t1[0];  QuadricEdgeCollapse.cpp:755
        let mut vi_top1 = t1.indices[0];
        // if (vi_top1 == vi0 || vi_top1 == vi1) {  QuadricEdgeCollapse.cpp:756
        if vi_top1 == vi0 || vi_top1 == vi1 {
            // vi_top1 = t1[1];  QuadricEdgeCollapse.cpp:757
            vi_top1 = t1.indices[1];
            // if (vi_top1 == vi0 || vi_top1 == vi1) vi_top1 = t1[2];  QuadricEdgeCollapse.cpp:758
            if vi_top1 == vi0 || vi_top1 == vi1 {
                vi_top1 = t1.indices[2];
            }
        }

        // remove_triangle(e_infos, v_infos[vi_top0], ti0);  QuadricEdgeCollapse.cpp:761
        {
            let mut v = v_infos[vi_top0 as usize];
            remove_triangle(e_infos, &mut v, ti0);
            v_infos[vi_top0 as usize] = v;
        }
        // remove_triangle(e_infos, v_infos[vi_top1], ti1);  QuadricEdgeCollapse.cpp:762
        {
            let mut v = v_infos[vi_top1 as usize];
            remove_triangle(e_infos, &mut v, ti1);
            v_infos[vi_top1 as usize] = v;
        }

        // VertexInfo &v_info0 = v_infos[vi0];  QuadricEdgeCollapse.cpp:764
        // VertexInfo &v_info1 = v_infos[vi1];  QuadricEdgeCollapse.cpp:765
        // uint32_t new_triangle_count = v_info0.count + v_info1.count - 4;  QuadricEdgeCollapse.cpp:767
        let new_triangle_count = v_infos[vi0 as usize].count + v_infos[vi1 as usize].count - 4;
        // remove_triangle(e_infos, v_info0, ti0);  QuadricEdgeCollapse.cpp:768
        {
            let mut v = v_infos[vi0 as usize];
            remove_triangle(e_infos, &mut v, ti0);
            v_infos[vi0 as usize] = v;
        }
        // remove_triangle(e_infos, v_info0, ti1);  QuadricEdgeCollapse.cpp:769
        {
            let mut v = v_infos[vi0 as usize];
            remove_triangle(e_infos, &mut v, ti1);
            v_infos[vi0 as usize] = v;
        }

        // copy second's edge infos out of e_infos, to free size
        // e_infos1.clear();  QuadricEdgeCollapse.cpp:772
        e_infos1.clear();
        // e_infos1.reserve(v_info1.count - 2);  QuadricEdgeCollapse.cpp:773
        e_infos1.reserve((v_infos[vi1 as usize].count - 2) as usize);
        // uint32_t v_info_s_end = v_info1.start + v_info1.count;  QuadricEdgeCollapse.cpp:774
        let v_info_s_end = v_infos[vi1 as usize].start + v_infos[vi1 as usize].count;
        // QuadricEdgeCollapse.cpp:775-780
        {
            let ei_start = v_infos[vi1 as usize].start;
            for ei in ei_start..v_info_s_end {
                // const EdgeInfo &e_info = e_infos[ei];  QuadricEdgeCollapse.cpp:776
                let e_info = e_infos[ei as usize];
                // if (e_info.t_index == ti0) continue;  QuadricEdgeCollapse.cpp:777
                if e_info.t_index == ti0 {
                    continue;
                }
                // if (e_info.t_index == ti1) continue;  QuadricEdgeCollapse.cpp:778
                if e_info.t_index == ti1 {
                    continue;
                }
                // e_infos1.emplace_back(e_info);  QuadricEdgeCollapse.cpp:779
                e_infos1.push(e_info);
            }
        }
        // v_info1.count = 0;  QuadricEdgeCollapse.cpp:781
        v_infos[vi1 as usize].count = 0;

        // uint32_t need = (new_triangle_count < v_info0.count)? 0:
        //               (new_triangle_count - v_info0.count);  QuadricEdgeCollapse.cpp:783-784
        let mut need = if new_triangle_count < v_infos[vi0 as usize].count {
            0
        } else {
            new_triangle_count - v_infos[vi0 as usize].count
        };

        // uint32_t      act_vi     = vi0 + 1;  QuadricEdgeCollapse.cpp:786
        let mut act_vi = vi0 + 1;
        // VertexInfo *act_v_info = &v_infos[act_vi];  QuadricEdgeCollapse.cpp:787
        // uint32_t      act_start  = act_v_info->start;  QuadricEdgeCollapse.cpp:788
        let mut act_start = v_infos[act_vi as usize].start;
        // uint32_t      last_end   = v_info0.start + v_info0.count;  QuadricEdgeCollapse.cpp:789
        let mut last_end = v_infos[vi0 as usize].start + v_infos[vi0 as usize].count;

        // infos.clear();  QuadricEdgeCollapse.cpp:791
        infos.clear();
        // infos.reserve(need);  QuadricEdgeCollapse.cpp:792
        infos.reserve(need as usize);

        // QuadricEdgeCollapse.cpp:794-811
        loop {
            // uint32_t save = act_start - last_end;  QuadricEdgeCollapse.cpp:795
            let save = act_start - last_end;
            // if (save > 0) {  QuadricEdgeCollapse.cpp:796
            if save > 0 {
                // if (save >= need) break;  QuadricEdgeCollapse.cpp:797
                if save >= need {
                    break;
                }
                // need -= save;  QuadricEdgeCollapse.cpp:798
                need -= save;
                // infos.emplace_back(act_v_info->start, act_v_info->count, need);  QuadricEdgeCollapse.cpp:799
                infos.push(CopyEdgeInfo::new(
                    v_infos[act_vi as usize].start,
                    v_infos[act_vi as usize].count,
                    need,
                ));
            } else {
                // infos.back().count += act_v_info->count;  QuadricEdgeCollapse.cpp:801
                let last = infos.len() - 1;
                infos[last].count += v_infos[act_vi as usize].count;
            }
            // last_end = act_v_info->start + act_v_info->count;  QuadricEdgeCollapse.cpp:803
            last_end = v_infos[act_vi as usize].start + v_infos[act_vi as usize].count;
            // act_v_info->start += need;  QuadricEdgeCollapse.cpp:804
            v_infos[act_vi as usize].start += need;
            // ++act_vi;  QuadricEdgeCollapse.cpp:805
            act_vi += 1;
            // if (act_vi < v_infos.size()) {  QuadricEdgeCollapse.cpp:806
            if (act_vi as usize) < v_infos.len() {
                // act_v_info = &v_infos[act_vi];  QuadricEdgeCollapse.cpp:807
                // act_start  = act_v_info->start;  QuadricEdgeCollapse.cpp:808
                act_start = v_infos[act_vi as usize].start;
            } else {
                // act_start = e_infos.size(); // fix for edge between last two triangles  QuadricEdgeCollapse.cpp:810
                act_start = e_infos.len() as u32;
            }
        }

        // copy by c_infos
        // QuadricEdgeCollapse.cpp:814-818
        for i in (1..=infos.len()).rev() {
            // const CopyEdgeInfo &c_info = infos[i - 1];  QuadricEdgeCollapse.cpp:815
            let c_info = infos[i - 1];
            // for (uint32_t ei = c_info.start + c_info.count - 1; ei >= c_info.start; --ei)  QuadricEdgeCollapse.cpp:816
            let mut ei = c_info.start + c_info.count - 1;
            loop {
                // e_infos[ei + c_info.move] = e_infos[ei]; // copy  QuadricEdgeCollapse.cpp:817
                e_infos[(ei + c_info.move_) as usize] = e_infos[ei as usize];
                if ei == c_info.start {
                    break;
                }
                ei -= 1;
            }
        }

        // copy triangle from first info into second
        // QuadricEdgeCollapse.cpp:821-825
        for ei_s in 0..e_infos1.len() {
            // uint32_t ei_f = v_info0.start + v_info0.count;  QuadricEdgeCollapse.cpp:822
            let ei_f = v_infos[vi0 as usize].start + v_infos[vi0 as usize].count;
            // e_infos[ei_f] = e_infos1[ei_s]; // copy  QuadricEdgeCollapse.cpp:823
            e_infos[ei_f as usize] = e_infos1[ei_s];
            // ++v_info0.count;  QuadricEdgeCollapse.cpp:824
            v_infos[vi0 as usize].count += 1;
        }
    }

    // QuadricEdgeCollapse.cpp:828-856
    pub fn compact(
        v_infos: &VertexInfos,
        t_infos: &TriangleInfos,
        e_infos: &EdgeInfos,
        its: &mut TriangleMesh,
    ) {
        // uint32_t vi_new = 0;  QuadricEdgeCollapse.cpp:833
        let mut vi_new: u32 = 0;
        // QuadricEdgeCollapse.cpp:834-845
        for vi in 0..v_infos.len() as u32 {
            // const VertexInfo &v_info = v_infos[vi];  QuadricEdgeCollapse.cpp:835
            let v_info = v_infos[vi as usize];
            // if (v_info.is_deleted()) continue; // deleted  QuadricEdgeCollapse.cpp:836
            if v_info.is_deleted() {
                continue;
            }
            // uint32_t e_info_end = v_info.start + v_info.count;  QuadricEdgeCollapse.cpp:837
            let e_info_end = v_info.start + v_info.count;
            // QuadricEdgeCollapse.cpp:838-842
            for ei in v_info.start..e_info_end {
                // const EdgeInfo &e_info = e_infos[ei];  QuadricEdgeCollapse.cpp:839
                let e_info = e_infos[ei as usize];
                // change vertex index
                // its.indices[e_info.t_index][e_info.edge] = vi_new;  QuadricEdgeCollapse.cpp:841
                its.indices_mut()[e_info.t_index as usize].indices[e_info.edge as usize] = vi_new;
            }
            // compact vertices
            // its.vertices[vi_new++] = its.vertices[vi];  QuadricEdgeCollapse.cpp:844
            let v = its.vertices()[vi as usize];
            its.vertices_mut()[vi_new as usize] = v;
            vi_new += 1;
        }
        // remove vertices tail
        // its.vertices.erase(its.vertices.begin() + vi_new, its.vertices.end());  QuadricEdgeCollapse.cpp:847
        its.vertices_mut().truncate(vi_new as usize);

        // uint32_t ti_new = 0;  QuadricEdgeCollapse.cpp:849
        let mut ti_new: u32 = 0;
        // QuadricEdgeCollapse.cpp:850-854
        for ti in 0..t_infos.len() as u32 {
            // const TriangleInfo &t_info = t_infos[ti];  QuadricEdgeCollapse.cpp:851
            let t_info = t_infos[ti as usize];
            // if (t_info.is_deleted()) continue;  QuadricEdgeCollapse.cpp:852
            if t_info.is_deleted() {
                continue;
            }
            // its.indices[ti_new++] = its.indices[ti];  QuadricEdgeCollapse.cpp:853
            let t = its.indices()[ti as usize];
            its.indices_mut()[ti_new as usize] = t;
            ti_new += 1;
        }
        // its.indices.erase(its.indices.begin() + ti_new, its.indices.end());  QuadricEdgeCollapse.cpp:855
        its.indices_mut().truncate(ti_new as usize);
    }
}

use quadric_edge_collapse::*;

/// Simplify mesh by Quadric metric.
///
/// QuadricEdgeCollapse.cpp:160-347
/// QuadricEdgeCollapse.hpp:21-26
///
/// - `its`: IN/OUT triangle mesh to be simplified.
/// - `triangle_count`: Wanted triangle count.
/// - `max_error`: Maximal Quadric for reduce. When `None` then max float is
///   used. Output (`Some`): Last used ErrorValue to collapse edge.
/// - `throw_on_cancel`: Could stop process of calculation.
/// - `status_fn`: Give a feed back to user about progress. Values 1 - 100.
pub fn its_quadric_edge_collapse(
    its: &mut TriangleMesh,
    triangle_count: u32,
    max_error: &mut Option<f32>,
    throw_on_cancel: Option<Box<dyn FnMut()>>,
    status_fn: Option<Box<dyn FnMut(i32)>>,
) {
    // check input
    // if (triangle_count >= its.indices.size()) return;  QuadricEdgeCollapse.cpp:168
    if triangle_count as usize >= its.indices().len() {
        return;
    }
    // float maximal_error = (max_error == nullptr)? std::numeric_limits<float>::max() : *max_error;  QuadricEdgeCollapse.cpp:169
    let maximal_error: f32 = match max_error {
        None => f32::MAX,
        Some(e) => *e,
    };
    // if (maximal_error <= 0.f) return;  QuadricEdgeCollapse.cpp:170
    if maximal_error <= 0.0 {
        return;
    }
    // if (throw_on_cancel == nullptr) throw_on_cancel = []() {};  QuadricEdgeCollapse.cpp:171
    let mut throw_on_cancel: Box<dyn FnMut()> = throw_on_cancel.unwrap_or_else(|| Box::new(|| {}));
    // if (status_fn == nullptr) status_fn = [](int) {};  QuadricEdgeCollapse.cpp:172
    let mut status_fn: Box<dyn FnMut(i32)> = status_fn.unwrap_or_else(|| Box::new(|_| {}));

    // StatusFn init_status_fn = [&](int percent) {  QuadricEdgeCollapse.cpp:174-177
    // (init takes its own status closure; mirror the body here.)
    let mut init_status_fn = |percent: i32| {
        // float n_percent = percent * status_init_size / 100.f;  QuadricEdgeCollapse.cpp:175
        let n_percent = percent as f32 * STATUS_INIT_SIZE as f32 / 100.0;
        // status_fn(static_cast<int>(std::round(n_percent)));  QuadricEdgeCollapse.cpp:176
        status_fn(n_percent.round() as i32);
    };

    // TriangleInfos t_infos; // only normals with information about deleted triangle  QuadricEdgeCollapse.cpp:179
    // VertexInfos   v_infos;  QuadricEdgeCollapse.cpp:180
    // EdgeInfos     e_infos;  QuadricEdgeCollapse.cpp:181
    // Errors        errors;  QuadricEdgeCollapse.cpp:182
    // std::tie(t_infos, v_infos, e_infos, errors) = init(its, throw_on_cancel, init_status_fn);  QuadricEdgeCollapse.cpp:183
    let (mut t_infos, mut v_infos, mut e_infos, errors) =
        init(its, &mut throw_on_cancel, &mut init_status_fn);
    // throw_on_cancel();  QuadricEdgeCollapse.cpp:184
    throw_on_cancel();
    // status_fn(status_init_size);  QuadricEdgeCollapse.cpp:185
    status_fn(STATUS_INIT_SIZE);

    // convert from triangle index to mutable priority queue index
    // std::vector<size_t> ti_2_mpqi(its.indices.size(), {0});  QuadricEdgeCollapse.cpp:191
    let ti_2_mpqi: Rc<RefCell<Vec<usize>>> =
        Rc::new(RefCell::new(vec![0usize; its.indices().len()]));
    // auto setter = [&ti_2_mpqi](const Error &e, size_t index) { ti_2_mpqi[e.triangle_index] = index; };  QuadricEdgeCollapse.cpp:192
    let setter = {
        let ti_2_mpqi = Rc::clone(&ti_2_mpqi);
        move |e: &Error, index: usize| {
            ti_2_mpqi.borrow_mut()[e.triangle_index as usize] = index;
        }
    };
    // auto less = [](const Error &e1, const Error &e2) -> bool { return e1.value < e2.value; };  QuadricEdgeCollapse.cpp:193
    let less = |e1: &Error, e2: &Error| -> bool { e1.value < e2.value };
    // auto mpq = make_miniheap_mutable_priority_queue<Error, 32, false>(std::move(setter), std::move(less));  QuadricEdgeCollapse.cpp:194
    let mut mpq = make_miniheap_mutable_priority_queue::<Error, _, _>(32, false, setter, less);
    // mpq.reserve(its.indices.size());  QuadricEdgeCollapse.cpp:196
    mpq.reserve(its.indices().len());
    // for (Error &error :errors) mpq.push(error);  QuadricEdgeCollapse.cpp:197
    for error in errors.iter() {
        mpq.push(*error);
    }

    // CopyEdgeInfos ceis;  QuadricEdgeCollapse.cpp:199
    // ceis.reserve(max_triangle_count_for_one_vertex);  QuadricEdgeCollapse.cpp:200
    let mut ceis: CopyEdgeInfos = Vec::with_capacity(MAX_TRIANGLE_COUNT_FOR_ONE_VERTEX);
    // EdgeInfos e_infos_swap;  QuadricEdgeCollapse.cpp:201
    // e_infos_swap.reserve(max_triangle_count_for_one_vertex);  QuadricEdgeCollapse.cpp:202
    let mut e_infos_swap: EdgeInfos = Vec::with_capacity(MAX_TRIANGLE_COUNT_FOR_ONE_VERTEX);
    // std::vector<uint32_t> changed_triangle_indices;  QuadricEdgeCollapse.cpp:203
    // changed_triangle_indices.reserve(2 * max_triangle_count_for_one_vertex);  QuadricEdgeCollapse.cpp:204
    let mut changed_triangle_indices: Vec<u32> =
        Vec::with_capacity(2 * MAX_TRIANGLE_COUNT_FOR_ONE_VERTEX);

    // uint32_t actual_triangle_count = its.indices.size();  QuadricEdgeCollapse.cpp:206
    let mut actual_triangle_count: u32 = its.indices().len() as u32;
    // uint32_t count_triangle_to_reduce = actual_triangle_count - triangle_count;  QuadricEdgeCollapse.cpp:207
    let count_triangle_to_reduce: u32 = actual_triangle_count - triangle_count;
    // auto increase_status = [&]() { ... };  QuadricEdgeCollapse.cpp:208-214
    // (Inlined below at call sites because it captures `actual_triangle_count`.)

    // modulo for update status, call each percent only once
    // uint32_t status_mod = std::max(uint32_t(16),
    //     count_triangle_to_reduce / (100 - status_init_size));  QuadricEdgeCollapse.cpp:216-217
    let status_mod: u32 =
        std::cmp::max(16u32, count_triangle_to_reduce / (100 - STATUS_INIT_SIZE as u32));

    // uint32_t iteration_number = 0;  QuadricEdgeCollapse.cpp:219
    let mut iteration_number: u32 = 0;
    // float last_collapsed_error = 0.f;  QuadricEdgeCollapse.cpp:220
    let mut last_collapsed_error: f32 = 0.0;
    // while (actual_triangle_count > triangle_count && !mpq.empty()) {  QuadricEdgeCollapse.cpp:221
    while actual_triangle_count > triangle_count && !mpq.is_empty() {
        // ++iteration_number;  QuadricEdgeCollapse.cpp:222
        iteration_number += 1;
        // if (iteration_number % status_mod == 0) increase_status();  QuadricEdgeCollapse.cpp:223
        if iteration_number % status_mod == 0 {
            // increase_status();  QuadricEdgeCollapse.cpp:208-214
            // double reduced = (actual_triangle_count - triangle_count) / (double) count_triangle_to_reduce;
            let reduced = (actual_triangle_count - triangle_count) as f64
                / count_triangle_to_reduce as f64;
            // double status = status_init_size + (100 - status_init_size) * (1. - reduced);
            let status = STATUS_INIT_SIZE as f64 + (100 - STATUS_INIT_SIZE) as f64 * (1.0 - reduced);
            // status_fn(static_cast<int>(std::round(status)));
            status_fn(status.round() as i32);
        }
        // if (iteration_number % check_cancel_period == 0) throw_on_cancel();  QuadricEdgeCollapse.cpp:224
        if iteration_number % CHECK_CANCEL_PERIOD == 0 {
            throw_on_cancel();
        }

        // triangle index 0
        // Error e = mpq.top(); // copy  QuadricEdgeCollapse.cpp:227
        let mut e: Error = *mpq.top().unwrap();
        // if (e.value >= maximal_error) break; // Too big error  QuadricEdgeCollapse.cpp:228
        if e.value >= maximal_error {
            break;
        }
        // mpq.pop();  QuadricEdgeCollapse.cpp:229
        mpq.pop();
        // uint32_t ti0 = e.triangle_index;  QuadricEdgeCollapse.cpp:230
        let ti0 = e.triangle_index;
        // TriangleInfo &t_info0 = t_infos[ti0];  QuadricEdgeCollapse.cpp:231
        // if (t_info0.is_deleted()) continue;  QuadricEdgeCollapse.cpp:232
        if t_infos[ti0 as usize].is_deleted() {
            continue;
        }
        // assert(t_info0.min_index < 3);  QuadricEdgeCollapse.cpp:233
        debug_assert!(t_infos[ti0 as usize].min_index < 3);

        // const Triangle &t0 = its.indices[ti0];  QuadricEdgeCollapse.cpp:235
        let t0 = its.indices()[ti0 as usize];
        let min_index0 = t_infos[ti0 as usize].min_index as usize;
        // uint32_t vi0 = t0[t_info0.min_index];  QuadricEdgeCollapse.cpp:236
        let mut vi0 = t0.indices[min_index0];
        // uint32_t vi1 = t0[(t_info0.min_index+1) %3];  QuadricEdgeCollapse.cpp:237
        let mut vi1 = t0.indices[(min_index0 + 1) % 3];
        // Need by move of neighbor edge infos in function: change_neighbors
        // if (vi0 > vi1) std::swap(vi0, vi1);  QuadricEdgeCollapse.cpp:239
        if vi0 > vi1 {
            std::mem::swap(&mut vi0, &mut vi1);
        }
        // VertexInfo &v_info0 = v_infos[vi0];  QuadricEdgeCollapse.cpp:240
        // VertexInfo &v_info1 = v_infos[vi1];  QuadricEdgeCollapse.cpp:241
        let v_info0 = v_infos[vi0 as usize];
        let v_info1 = v_infos[vi1 as usize];
        // assert(!v_info0.is_deleted() && !v_info1.is_deleted());  QuadricEdgeCollapse.cpp:242
        debug_assert!(!v_info0.is_deleted() && !v_info1.is_deleted());

        // new vertex position
        // SymMat q(v_info0.q);  QuadricEdgeCollapse.cpp:245
        let mut q = v_info0.q;
        // q += v_info1.q;  QuadricEdgeCollapse.cpp:246
        q.add_assign(&v_info1.q);
        // Vec3f new_vertex0 = calculate_vertex(vi0, vi1, q, its.vertices);  QuadricEdgeCollapse.cpp:247
        let new_vertex0 = calculate_vertex(vi0, vi1, &q, its);
        // set of triangle indices that change quadric
        // uint32_t ti1 = -1; // triangle 1 index  QuadricEdgeCollapse.cpp:249
        let mut ti1: u32 = u32::MAX;
        // auto ti1_opt = (v_info0.count < v_info1.count)?  QuadricEdgeCollapse.cpp:250-252
        let ti1_opt = if v_info0.count < v_info1.count {
            find_triangle_index1(vi1, &v_info0, ti0, &e_infos, its)
        } else {
            find_triangle_index1(vi0, &v_info1, ti0, &e_infos, its)
        };
        // if (ti1_opt.has_value()) {  QuadricEdgeCollapse.cpp:253
        if let Some(v) = ti1_opt {
            // ti1 = *ti1_opt;  QuadricEdgeCollapse.cpp:254
            ti1 = v;
            // reorder_edges(e_infos, v_info0, ti0, ti1);  QuadricEdgeCollapse.cpp:255
            reorder_edges(&mut e_infos, &v_info0, ti0, ti1);
            // reorder_edges(e_infos, v_info1, ti0, ti1);  QuadricEdgeCollapse.cpp:256
            reorder_edges(&mut e_infos, &v_info1, ti0, ti1);
        }
        // if (!ti1_opt.has_value() || ... )  QuadricEdgeCollapse.cpp:258-263
        if ti1_opt.is_none()
            || degenerate(vi0, ti0, ti1, &v_info1, &e_infos, its)
            || degenerate(vi1, ti0, ti1, &v_info0, &e_infos, its)
            || create_no_volume(vi0, vi1, ti0, ti1, &v_info0, &v_info1, &e_infos, its)
            || is_flipped(&new_vertex0, ti0, ti1, &v_info0, &t_infos, &e_infos, its)
            || is_flipped(&new_vertex0, ti0, ti1, &v_info1, &t_infos, &e_infos, its)
        {
            // try other triangle's edge
            // Vec3d errors = calculate_3errors(t0, its.vertices, v_infos);  QuadricEdgeCollapse.cpp:265
            let errors = calculate_3errors(&t0, its, &v_infos);
            // Vec3i ord = ...  QuadricEdgeCollapse.cpp:266-272
            let ord: [i32; 3] = if errors.x < errors.y {
                if errors.x < errors.z {
                    if errors.y < errors.z {
                        [0, 1, 2]
                    } else {
                        [0, 2, 1]
                    }
                } else {
                    [2, 0, 1]
                }
            } else if errors.y < errors.z {
                if errors.x < errors.z {
                    [1, 0, 2]
                } else {
                    [1, 2, 0]
                }
            } else {
                [2, 1, 0]
            };
            // QuadricEdgeCollapse.cpp:273-283
            let min_index0 = t_infos[ti0 as usize].min_index as i32;
            if min_index0 == ord[0] {
                // t_info0.min_index = ord[1];  QuadricEdgeCollapse.cpp:274
                t_infos[ti0 as usize].min_index = ord[1] as u8;
                // e.value = errors[t_info0.min_index];  QuadricEdgeCollapse.cpp:275
                e.value = vec3d_at(&errors, t_infos[ti0 as usize].min_index as usize) as f32;
            } else if min_index0 == ord[1] {
                // t_info0.min_index = ord[2];  QuadricEdgeCollapse.cpp:277
                t_infos[ti0 as usize].min_index = ord[2] as u8;
                // e.value = errors[t_info0.min_index];  QuadricEdgeCollapse.cpp:278
                e.value = vec3d_at(&errors, t_infos[ti0 as usize].min_index as usize) as f32;
            } else {
                // error is changed when surround edge is reduced
                // t_info0.min_index = 3; // bad index -> invalidate  QuadricEdgeCollapse.cpp:281
                t_infos[ti0 as usize].min_index = 3;
                // e.value = maximal_error;  QuadricEdgeCollapse.cpp:282
                e.value = maximal_error;
            }
            // IMPROVE: check mpq top if it is ti1 with same edge
            // mpq.push(e);  QuadricEdgeCollapse.cpp:285
            mpq.push(e);
            // continue;  QuadricEdgeCollapse.cpp:286
            continue;
        }

        // last_collapsed_error = e.value;  QuadricEdgeCollapse.cpp:289
        last_collapsed_error = e.value;
        // changed_triangle_indices.clear();  QuadricEdgeCollapse.cpp:290
        changed_triangle_indices.clear();
        // changed_triangle_indices.reserve(v_info0.count + v_info1.count - 4);  QuadricEdgeCollapse.cpp:291
        changed_triangle_indices.reserve((v_info0.count + v_info1.count - 4) as usize);

        // for each vertex0 triangles
        // uint32_t v_info0_end = v_info0.start + v_info0.count - 2;  QuadricEdgeCollapse.cpp:294
        let v_info0_end = v_info0.start + v_info0.count - 2;
        // QuadricEdgeCollapse.cpp:295-299
        for di in v_info0.start..v_info0_end {
            // assert(di < e_infos.size());  QuadricEdgeCollapse.cpp:296
            debug_assert!((di as usize) < e_infos.len());
            // uint32_t ti = e_infos[di].t_index;  QuadricEdgeCollapse.cpp:297
            let ti = e_infos[di as usize].t_index;
            // changed_triangle_indices.emplace_back(ti);  QuadricEdgeCollapse.cpp:298
            changed_triangle_indices.push(ti);
        }

        // for each vertex1 triangles
        // uint32_t v_info1_end = v_info1.start + v_info1.count - 2;  QuadricEdgeCollapse.cpp:302
        let v_info1_end = v_info1.start + v_info1.count - 2;
        // QuadricEdgeCollapse.cpp:303-310
        for di in v_info1.start..v_info1_end {
            // assert(di < e_infos.size());  QuadricEdgeCollapse.cpp:304
            debug_assert!((di as usize) < e_infos.len());
            // EdgeInfo &e_info = e_infos[di];  QuadricEdgeCollapse.cpp:305
            let e_info = e_infos[di as usize];
            // uint32_t ti = e_info.t_index;  QuadricEdgeCollapse.cpp:306
            let ti = e_info.t_index;
            // Triangle &t = its.indices[ti];  QuadricEdgeCollapse.cpp:307
            // t[e_info.edge] = vi0; // change index  QuadricEdgeCollapse.cpp:308
            its.indices_mut()[ti as usize].indices[e_info.edge as usize] = vi0;
            // changed_triangle_indices.emplace_back(ti);  QuadricEdgeCollapse.cpp:309
            changed_triangle_indices.push(ti);
        }
        // v_info0.q = q;  QuadricEdgeCollapse.cpp:311
        v_infos[vi0 as usize].q = q;

        // fix neighbors
        // vertex index of triangle 0 which is not vi0 nor vi1
        // uint32_t vi_top0 = t0[(t_info0.min_index + 2) % 3];  QuadricEdgeCollapse.cpp:315
        let vi_top0 = t0.indices[(t_infos[ti0 as usize].min_index as usize + 2) % 3];
        // const Triangle &t1 = its.indices[ti1];  QuadricEdgeCollapse.cpp:316
        let t1 = its.indices()[ti1 as usize];
        // change_neighbors(e_infos, v_infos, ti0, ti1, vi0, vi1, vi_top0, t1, ceis, e_infos_swap);  QuadricEdgeCollapse.cpp:317-318
        change_neighbors(
            &mut e_infos,
            &mut v_infos,
            ti0,
            ti1,
            vi0,
            vi1,
            vi_top0,
            &t1,
            &mut ceis,
            &mut e_infos_swap,
        );

        // Change vertex
        // its.vertices[vi0] = new_vertex0;  QuadricEdgeCollapse.cpp:321
        its_set_vertex(its, vi0, new_vertex0);

        // fix errors - must be after set neighbors - v_infos
        // mpq.remove(ti_2_mpqi[ti1]);  QuadricEdgeCollapse.cpp:324
        let ti1_mpqi = ti_2_mpqi.borrow()[ti1 as usize];
        mpq.remove(ti1_mpqi);
        // for (uint32_t ti : changed_triangle_indices) {  QuadricEdgeCollapse.cpp:325-331
        for idx in 0..changed_triangle_indices.len() {
            let ti = changed_triangle_indices[idx];
            // size_t priority_queue_index = ti_2_mpqi[ti];  QuadricEdgeCollapse.cpp:326
            let priority_queue_index = ti_2_mpqi.borrow()[ti as usize];
            // TriangleInfo& t_info = t_infos[ti];  QuadricEdgeCollapse.cpp:327
            // t_info.n = create_normal(its.indices[ti], its.vertices).cast<float>(); // recalc normals  QuadricEdgeCollapse.cpp:328
            let tri = its.indices()[ti as usize];
            t_infos[ti as usize].n = create_normal(&tri, its).cast_f32();
            // mpq[priority_queue_index] = calculate_error(ti, its.indices[ti], its.vertices, v_infos, t_info.min_index);  QuadricEdgeCollapse.cpp:329
            let mut min_index = t_infos[ti as usize].min_index;
            let err = calculate_error(ti, &tri, its, &v_infos, &mut min_index);
            t_infos[ti as usize].min_index = min_index;
            if let Some(slot) = mpq.get_mut(priority_queue_index) {
                *slot = err;
            }
            // mpq.update(priority_queue_index);  QuadricEdgeCollapse.cpp:330
            mpq.update(priority_queue_index);
        }

        // set triangle(0 + 1) indices as deleted
        // TriangleInfo &t_info1 = t_infos[ti1];  QuadricEdgeCollapse.cpp:334
        // t_info0.set_deleted();  QuadricEdgeCollapse.cpp:335
        t_infos[ti0 as usize].set_deleted();
        // t_info1.set_deleted();  QuadricEdgeCollapse.cpp:336
        t_infos[ti1 as usize].set_deleted();
        // triangle counter decrementation
        // actual_triangle_count-=2;  QuadricEdgeCollapse.cpp:338
        actual_triangle_count -= 2;
    }

    // compact triangle
    // compact(v_infos, t_infos, e_infos, its);  QuadricEdgeCollapse.cpp:345
    compact(&v_infos, &t_infos, &e_infos, its);
    // if (max_error != nullptr) *max_error = last_collapsed_error;  QuadricEdgeCollapse.cpp:346
    if let Some(e) = max_error.as_mut() {
        *e = last_collapsed_error;
    }
}

// Helper mirroring Eigen `Vec3d::operator[]` for indices 0..2.
#[inline]
fn vec3d_at(v: &Vec3d, i: usize) -> f64 {
    match i {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}
