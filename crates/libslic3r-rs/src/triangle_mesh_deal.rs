//! Faithful 1:1 port of BambuStudio's `src/libslic3r/TriangleMeshDeal.cpp`.
//!
//! The C++ source contains a single method, `TriangleMeshDeal::smooth_triangle_mesh`,
//! which performs one step of Loop subdivision on a `TriangleMesh` using libigl's
//! `igl::loop`. libigl is a header-only template library, so the relevant igl
//! functions (`igl::loop`, `igl::triangle_triangle_adjacency`,
//! `igl::vertex_triangle_adjacency`, `igl::adjacency_list`) are inlined here as
//! pure-Rust ports rather than pulled in as a native dependency. This keeps the
//! module wasm-safe (no system/dylib deps).
//!
//! References:
//!   - TriangleMeshDeal.cpp / TriangleMeshDeal.hpp
//!   - libigl: igl/loop.cpp, igl/triangle_triangle_adjacency.cpp,
//!             igl/vertex_triangle_adjacency.cpp, igl/adjacency_list.cpp

use crate::geometry::Point3F;
use crate::triangle_mesh::{Triangle, TriangleMesh};

/// Mesh repair / processing operations.
///
/// Mirrors C++ `class TriangleMeshDeal` (TriangleMeshDeal.hpp:7-11).
pub struct TriangleMeshDeal;

impl TriangleMeshDeal {
    // TriangleMeshDeal.cpp:9 — static TriangleMesh smooth_triangle_mesh(const TriangleMesh &mesh, bool &ok)
    pub fn smooth_triangle_mesh(mesh: &TriangleMesh, ok: &mut bool) -> TriangleMesh {
        // TriangleMeshDeal.cpp:11-53 — anonymous scope block
        {
            // TriangleMeshDeal.cpp:14-15 — Eigen::MatrixXi OF, F; Eigen::MatrixXd OV, V;
            // Modelled as flat row-major (rows x 3) matrices: OV/V hold f64, OF/F hold i32.
            // TriangleMeshDeal.cpp:16 — auto vertices_count = mesh.its.vertices.size();
            let vertices_count = mesh.vertices().len();
            // TriangleMeshDeal.cpp:17 — OV = Eigen::MatrixXd(vertices_count, 3);
            let mut ov: Vec<[f64; 3]> = vec![[0.0; 3]; vertices_count];
            // TriangleMeshDeal.cpp:18-21 — copy vertices into OV
            for i in 0..vertices_count {
                // TriangleMeshDeal.cpp:19 — auto v = mesh.its.vertices[i];
                let v = mesh.vertices()[i];
                // TriangleMeshDeal.cpp:20 — OV.row(i) << v[0], v[1], v[2];
                // mesh.its.vertices is Vec3f (float); widen to double for MatrixXd.
                ov[i] = [v.x, v.y, v.z];
            }
            // TriangleMeshDeal.cpp:22 — auto indices_count = mesh.its.indices.size();
            let indices_count = mesh.indices().len();
            // TriangleMeshDeal.cpp:23 — OF = Eigen::MatrixXi(indices_count, 3);
            let mut of: Vec<[i32; 3]> = vec![[0; 3]; indices_count];
            // TriangleMeshDeal.cpp:24-27 — copy indices into OF
            for i in 0..indices_count {
                // TriangleMeshDeal.cpp:25 — auto face = mesh.its.indices[i];
                let face = mesh.indices()[i];
                // TriangleMeshDeal.cpp:26 — OF.row(i) << face[0], face[1], face[2];
                of[i] = [
                    face.indices[0] as i32,
                    face.indices[1] as i32,
                    face.indices[2] as i32,
                ];
            }
            // TriangleMeshDeal.cpp:28 — //igl:: read_triangle_mesh( ... , OV, OF);
            // TriangleMeshDeal.cpp:29 — V = OV;
            let mut v_mat = ov;
            // TriangleMeshDeal.cpp:30 — F = OF;
            let mut f_mat = of;

            // TriangleMeshDeal.cpp:32 — //igl::upsample(Eigen::MatrixXd(V), Eigen::MatrixXi(F), V, F);
            // TriangleMeshDeal.cpp:33 — ok = true;
            *ok = true;
            // TriangleMeshDeal.cpp:34 — if (!igl::loop(Eigen::MatrixXd(V), Eigen::MatrixXi(F), V, F)) {
            if !igl_loop(&mut v_mat, &mut f_mat, 1) {
                // TriangleMeshDeal.cpp:35 — ok = false;
                *ok = false;
                // TriangleMeshDeal.cpp:36 — return TriangleMesh();
                return TriangleMesh::new();
            }
            // TriangleMeshDeal.cpp:38 — //igl::false_barycentric_subdivision(Eigen::MatrixXd(V), Eigen::MatrixXi(F), V, F);
            // TriangleMeshDeal.cpp:39 — indexed_triangle_set its;
            let mut its_vertices: Vec<Point3F> = Vec::new();
            let mut its_indices: Vec<Triangle> = Vec::new();
            // TriangleMeshDeal.cpp:40 — int vertex_count = V.rows();
            let vertex_count = v_mat.len();
            // TriangleMeshDeal.cpp:41 — its.vertices.resize(vertex_count);
            its_vertices.resize(vertex_count, Point3F::new(0.0, 0.0, 0.0));
            // TriangleMeshDeal.cpp:42-44 — its.vertices[i] = V.row(i).cast<float>();
            for i in 0..vertex_count {
                // V is MatrixXd (double); its.vertices is Vec3f (float). The cast<float>()
                // narrows each component to f32, so round-trip through f32 to match exactly.
                let row = v_mat[i];
                its_vertices[i] = Point3F::new(
                    row[0] as f32 as f64,
                    row[1] as f32 as f64,
                    row[2] as f32 as f64,
                );
            }
            // TriangleMeshDeal.cpp:45 — int indice_count = F.rows();
            let indice_count = f_mat.len();
            // TriangleMeshDeal.cpp:46 — its.indices.resize(indice_count);
            its_indices.resize(indice_count, Triangle::new(0, 0, 0));
            // TriangleMeshDeal.cpp:47-50 — its.indices[i] = Slic3r::Vec3i(cur[0], cur[1], cur[2]);
            for i in 0..indice_count {
                // TriangleMeshDeal.cpp:48 — auto cur = F.row(i);
                let cur = f_mat[i];
                // TriangleMeshDeal.cpp:49 — its.indices[i] = Slic3r::Vec3i(cur[0], cur[1], cur[2]);
                its_indices[i] = Triangle::new(cur[0] as u32, cur[1] as u32, cur[2] as u32);
            }
            // TriangleMeshDeal.cpp:51 — TriangleMesh result_mesh(its);
            let result_mesh = TriangleMesh::from_parts(its_vertices, its_indices);
            // TriangleMeshDeal.cpp:52 — return result_mesh;
            result_mesh
        }
    }
}

// ---------------------------------------------------------------------------
// Inlined libigl dependencies (header-only templates in C++).
// ---------------------------------------------------------------------------

/// Port of `igl::vertex_triangle_adjacency` (the NI-cumsum overload).
///
/// vertex_triangle_adjacency.cpp:50-82
///
/// Inputs:
///   `f`  m by 3 list of triangle faces.
///   `n`  number of mesh vertices.
/// Outputs:
///   `vf` flattened (3*m) list of incident faces, grouped by vertex.
///   `ni` (n+1) cumulative offsets into `vf`.
fn vertex_triangle_adjacency(f: &[[i32; 3]], n: usize, vf: &mut Vec<i32>, ni: &mut Vec<i32>) {
    // vertex_triangle_adjacency.cpp:59 — VectorXI vfd = VectorXI::Zero(n);
    let mut vfd: Vec<i32> = vec![0; n];
    // vertex_triangle_adjacency.cpp:60-66 — count vertex-face degree
    for i in 0..f.len() {
        for j in 0..3 {
            vfd[f[i][j] as usize] += 1;
        }
    }
    // vertex_triangle_adjacency.cpp:67 — igl::cumsum(vfd,1,NI);
    // cumsum along dimension 1 (down each column of a column-vector) is a prefix sum.
    let mut ni_inner: Vec<i32> = vec![0; n];
    {
        let mut acc: i32 = 0;
        for i in 0..n {
            acc += vfd[i];
            ni_inner[i] = acc;
        }
    }
    // vertex_triangle_adjacency.cpp:68-69 — NI = (DerivedNI(n+1)<<0,NI).finished(); (prepend a zero)
    ni.clear();
    ni.resize(n + 1, 0);
    ni[0] = 0;
    for i in 0..n {
        ni[i + 1] = ni_inner[i];
    }
    // vertex_triangle_adjacency.cpp:71 — vfd = NI; (vfd now acts as a counter)
    let mut vfd_counter: Vec<i32> = ni.clone();
    // vertex_triangle_adjacency.cpp:73 — VF.derived() = Eigen::VectorXi(3*F.rows());
    *vf = vec![0; 3 * f.len()];
    // vertex_triangle_adjacency.cpp:74-81 — scatter face indices into VF
    for i in 0..f.len() {
        for j in 0..3 {
            let vidx = f[i][j] as usize;
            vf[vfd_counter[vidx] as usize] = i as i32;
            vfd_counter[vidx] += 1;
        }
    }
}

/// Port of `igl::triangle_triangle_adjacency` (the `TT`-only matrix overload).
///
/// triangle_triangle_adjacency.cpp:37-71
///
/// Builds the per-corner triangle-triangle adjacency `tt` (m by 3), where
/// `tt[f][k]` is the index of the face sharing edge (k, k+1) of face `f`, or -1.
fn triangle_triangle_adjacency_tt(f: &[[i32; 3]], tt: &mut Vec<[i32; 3]>) {
    // triangle_triangle_adjacency.cpp:42 — const int n = F.maxCoeff()+1;
    let mut n: i32 = -1;
    for row in f.iter() {
        for &val in row.iter() {
            if val > n {
                n = val;
            }
        }
    }
    let n = (n + 1) as usize;
    // triangle_triangle_adjacency.cpp:44-45 — vertex_triangle_adjacency(F,n,VF,NI);
    let mut vf: Vec<i32> = Vec::new();
    let mut ni: Vec<i32> = Vec::new();
    vertex_triangle_adjacency(f, n, &mut vf, &mut ni);
    // triangle_triangle_adjacency.cpp:46 — TT = DerivedTT::Constant(F.rows(),3,-1);
    *tt = vec![[-1i32; 3]; f.len()];
    // triangle_triangle_adjacency.cpp:48-70 — for each face f
    for face in 0..f.len() {
        // triangle_triangle_adjacency.cpp:51 — Loop over corners
        for k in 0..3 {
            // triangle_triangle_adjacency.cpp:53 — int vi = F(f,k), vin = F(f,(k+1)%3);
            let vi = f[face][k];
            let vin = f[face][(k + 1) % 3];
            // triangle_triangle_adjacency.cpp:55 — for (int j = NI[vi]; j < NI[vi+1]; j++)
            let start = ni[vi as usize];
            let end = ni[vi as usize + 1];
            let mut j = start;
            while j < end {
                // triangle_triangle_adjacency.cpp:57 — int fn = VF[j];
                let fnb = vf[j as usize];
                // triangle_triangle_adjacency.cpp:59 — Not this face
                if fnb != face as i32 {
                    // triangle_triangle_adjacency.cpp:62 — Face neighbor also has [vi,vin] edge
                    if f[fnb as usize][0] == vin
                        || f[fnb as usize][1] == vin
                        || f[fnb as usize][2] == vin
                    {
                        // triangle_triangle_adjacency.cpp:64-65 — TT(f,k) = fn; break;
                        tt[face][k] = fnb;
                        break;
                    }
                }
                let _ = vi; // vi only used to index NI above
                j += 1;
            }
        }
    }
}

/// Port of `igl::triangle_triangle_adjacency` (the `TT, TTi` matrix overload).
///
/// triangle_triangle_adjacency.cpp:116-144
///
/// Computes triangle-triangle adjacency `tt` together with the corner indices
/// `tti` (which corner of the neighbor matches each edge), or -1.
fn triangle_triangle_adjacency_tt_tti(
    f: &[[i32; 3]],
    tt: &mut Vec<[i32; 3]>,
    tti: &mut Vec<[i32; 3]>,
) {
    // triangle_triangle_adjacency.cpp:121 — triangle_triangle_adjacency(F,TT);
    triangle_triangle_adjacency_tt(f, tt);
    // triangle_triangle_adjacency.cpp:122 — TTi = DerivedTTi::Constant(TT.rows(),TT.cols(),-1);
    *tti = vec![[-1i32; 3]; tt.len()];
    // triangle_triangle_adjacency.cpp:124-143 — for each face f
    for face in 0..f.len() {
        // triangle_triangle_adjacency.cpp:126 — for(int k = 0;k<3;k++)
        for k in 0..3 {
            // triangle_triangle_adjacency.cpp:128 — int vi = F(f,k), vj = F(f,(k+1)%3);
            let vi = f[face][k];
            let vj = f[face][(k + 1) % 3];
            // triangle_triangle_adjacency.cpp:129 — int fn = TT(f,k);
            let fnb = tt[face][k];
            // triangle_triangle_adjacency.cpp:130 — if(fn >= 0)
            if fnb >= 0 {
                // triangle_triangle_adjacency.cpp:132 — for(int kn = 0;kn<3;kn++)
                for kn in 0..3 {
                    // triangle_triangle_adjacency.cpp:134 — int vin = F(fn,kn), vjn = F(fn,(kn+1)%3);
                    let vin = f[fnb as usize][kn];
                    let vjn = f[fnb as usize][(kn + 1) % 3];
                    // triangle_triangle_adjacency.cpp:135 — if(vi == vjn && vin == vj)
                    if vi == vjn && vin == vj {
                        // triangle_triangle_adjacency.cpp:137-138 — TTi(f,k) = kn; break;
                        tti[face][k] = kn as i32;
                        break;
                    }
                }
            }
        }
    }
}

/// Port of `igl::adjacency_list` (the matrix overload, `sorted` variant).
///
/// adjacency_list.cpp:13-128
///
/// Builds the vertex-vertex adjacency list `a`. When `sorted` is true the
/// neighbours are ordered around each vertex (assuming manifoldness).
fn adjacency_list(f: &[[i32; 3]], a: &mut Vec<Vec<i32>>, sorted: bool) {
    // adjacency_list.cpp:19 — A.clear();
    a.clear();
    // adjacency_list.cpp:20 — A.resize(F.maxCoeff()+1);
    let mut max_coeff: i32 = -1;
    for row in f.iter() {
        for &val in row.iter() {
            if val > max_coeff {
                max_coeff = val;
            }
        }
    }
    a.resize((max_coeff + 1) as usize, Vec::new());

    // adjacency_list.cpp:23-34 — Loop over faces, push both directions of each edge
    let cols = 3usize;
    for i in 0..f.len() {
        for j in 0..cols {
            // adjacency_list.cpp:29-30 — Get indices of edge: s --> d
            let s = f[i][j];
            let d = f[i][(j + 1) % cols];
            // adjacency_list.cpp:31-32 — A.at(s).push_back(d); A.at(d).push_back(s);
            a[s as usize].push(d);
            a[d as usize].push(s);
        }
    }

    // adjacency_list.cpp:37-41 — Remove duplicates (sort + unique)
    for i in 0..a.len() {
        a[i].sort_unstable();
        a[i].dedup();
    }

    // adjacency_list.cpp:44-127 — If needed, sort every VV
    if sorted {
        // adjacency_list.cpp:49-50 — std::vector<std::vector<std::vector<int>>> SR; SR.resize(A.size());
        let mut sr: Vec<Vec<[i32; 2]>> = vec![Vec::new(); a.len()];

        // adjacency_list.cpp:52-68 — for every vertex s store ordered edge (d, v)
        for i in 0..f.len() {
            for j in 0..cols {
                // adjacency_list.cpp:58-59 — edge s --> d
                let s = f[i][j];
                let d = f[i][(j + 1) % cols];
                // adjacency_list.cpp:61 — opposing vertex v
                let v = f[i][(j + 2) % cols];
                // adjacency_list.cpp:63-66 — e = {d, v}; SR[s].push_back(e);
                sr[s as usize].push([d, v]);
            }
        }

        // adjacency_list.cpp:70-126 — for every vertex v, reorder A[v]
        for v in 0..sr.len() {
            // adjacency_list.cpp:72-73 — references to A[v] and SR[v]
            // (vv is a[v], srv is sr[v])
            let srv = sr[v].clone();
            // adjacency_list.cpp:75 — std::vector<std::vector<int>> pn = sr;
            let mut pn: Vec<[i32; 2]> = srv.clone();

            // adjacency_list.cpp:78-97 — Compute previous/next for every element in sr
            for i in 0..srv.len() {
                // adjacency_list.cpp:80-81 — int a = sr[i][0]; int b = sr[i][1];
                let a_e = srv[i][0];
                let b_e = srv[i][1];

                // adjacency_list.cpp:83-88 — search for previous
                let mut p: i32 = -1;
                for j in 0..srv.len() {
                    if srv[j][1] == a_e {
                        p = j as i32;
                    }
                }
                pn[i][0] = p;

                // adjacency_list.cpp:90-95 — search for next
                let mut nn: i32 = -1;
                for j in 0..srv.len() {
                    if srv[j][0] == b_e {
                        nn = j as i32;
                    }
                }
                pn[i][1] = nn;
            }

            // adjacency_list.cpp:99-103 — assume manifoldness (look for beginning of chain)
            let mut c: usize = 0;
            for _j in 0..=srv.len() {
                if pn[c][0] != -1 {
                    c = pn[c][0] as usize;
                }
            }

            // adjacency_list.cpp:105 — if (pn[c][0] == -1) // border case
            if pn[c][0] == -1 {
                // adjacency_list.cpp:108-114 — produce new vv relation (border)
                for j in 0..srv.len() {
                    a[v][j] = srv[c][0];
                    if pn[c][1] != -1 {
                        c = pn[c][1] as usize;
                    }
                }
                // adjacency_list.cpp:114 — vv.back() = sr[c][1];
                if let Some(last) = a[v].last_mut() {
                    *last = srv[c][1];
                }
            } else {
                // adjacency_list.cpp:118-124 — produce new vv relation (closed loop)
                for j in 0..srv.len() {
                    a[v][j] = srv[c][0];
                    c = pn[c][1] as usize;
                }
            }
        }
    }
}

/// Sparse-matrix triplet, modelling Eigen's `Eigen::Triplet<SType>`.
///
/// loop.cpp:28 — typedef Eigen::Triplet<SType> Triplet_t;
struct Triplet {
    row: usize,
    col: usize,
    val: f64,
}

/// Port of the single-step `igl::loop` overload.
///
/// loop.cpp:21-152
///
/// Given the face list `f` (with `n_verts` vertices), builds the subdivision
/// triplets `s` (the sparse subdivision matrix, `n_newverts` by `n_verts`) and
/// the new face list `nf`. Returns false on a malformed adjacency (matching the
/// C++ early-out at loop.cpp:60).
fn igl_loop_step(
    n_verts: usize,
    f: &[[i32; 3]],
    s: &mut Vec<Triplet>,
    nf: &mut Vec<[i32; 3]>,
) -> bool {
    // loop.cpp:30-31 — Ref. https://graphics.stanford.edu/~mdfisher/subdivision.html
    //                  Heavily borrowing from igl::upsample

    // loop.cpp:33-34 — triangle_triangle_adjacency(F, FF, FFi);
    let mut ff: Vec<[i32; 3]> = Vec::new();
    let mut ffi: Vec<[i32; 3]> = Vec::new();
    triangle_triangle_adjacency_tt_tti(f, &mut ff, &mut ffi);
    // loop.cpp:35-36 — adjacency_list(F, adjacencyList, true);
    let mut adjacency_list_vv: Vec<Vec<i32>> = Vec::new();
    adjacency_list(f, &mut adjacency_list_vv, true);

    // loop.cpp:38 — Compute the number and positions of the vertices to insert (on edges)
    // loop.cpp:39 — Eigen::MatrixXi NI = Eigen::MatrixXi::Constant(FF.rows(), FF.cols(), -1);
    let ff_rows = ff.len();
    let mut ni: Vec<[i32; 3]> = vec![[-1i32; 3]; ff_rows];
    // loop.cpp:40 — Eigen::MatrixXi NIdoubles = Eigen::MatrixXi::Zero(FF.rows(), FF.cols());
    let mut ni_doubles: Vec<[i32; 3]> = vec![[0i32; 3]; ff_rows];
    // loop.cpp:41 — Eigen::VectorXi vertIsOnBdry = Eigen::VectorXi::Zero(n_verts);
    let mut vert_is_on_bdry: Vec<i32> = vec![0; n_verts];
    // loop.cpp:42 — int counter = 0;
    let mut counter: i32 = 0;
    // loop.cpp:43-71 — assign new-vertex indices on each edge
    for i in 0..ff_rows {
        for j in 0..3 {
            // loop.cpp:47 — if(NI(i,j) == -1)
            if ni[i][j] == -1 {
                // loop.cpp:49-50 — NI(i,j) = counter; NIdoubles(i,j) = 0;
                ni[i][j] = counter;
                ni_doubles[i][j] = 0;
                // loop.cpp:51 — if (FF(i,j) != -1)
                if ff[i][j] != -1 {
                    // loop.cpp:53 — If it is not a boundary
                    // loop.cpp:54 — int adj_triangle = FF(i, j);
                    let adj_triangle = ff[i][j];
                    // loop.cpp:55 — int adj_edge = FFi(i, j);
                    let adj_edge = ffi[i][j];
                    // loop.cpp:56 — bounds check on adj_triangle / adj_edge
                    if adj_triangle >= 0
                        && (adj_triangle as usize) < ni.len()
                        && adj_edge >= 0
                        && (adj_edge as usize) < 3
                    {
                        // loop.cpp:57-58 — NI(adj_triangle, adj_edge) = counter; NIdoubles(i, j) = 1;
                        ni[adj_triangle as usize][adj_edge as usize] = counter;
                        ni_doubles[i][j] = 1;
                    } else {
                        // loop.cpp:60 — return false;
                        return false;
                    }
                } else {
                    // loop.cpp:63 — Mark boundary vertices for later
                    // loop.cpp:65-66 — vertIsOnBdry(F(i,j)) = 1; vertIsOnBdry(F(i,(j+1)%3)) = 1;
                    vert_is_on_bdry[f[i][j] as usize] = 1;
                    vert_is_on_bdry[f[i][(j + 1) % 3] as usize] = 1;
                }
                // loop.cpp:68 — ++counter;
                counter += 1;
            }
        }
    }

    // loop.cpp:73 — const int& n_odd = n_verts;
    let n_odd = n_verts as i32;
    // loop.cpp:74 — const int& n_even = counter;
    let _n_even = counter;
    // loop.cpp:75 — const int n_newverts = n_odd + n_even;
    let n_newverts = n_odd + counter;

    // loop.cpp:77-78 — Construct vertex positions
    let mut triplet_list: Vec<Triplet> = Vec::new();
    // loop.cpp:79-107 — Old vertices (odd)
    for i in 0..(n_odd as usize) {
        // loop.cpp:82 — const std::vector<int>& localAdjList = adjacencyList[i];
        let local_adj_list = &adjacency_list_vv[i];
        // loop.cpp:83 — if(vertIsOnBdry(i)==1)
        if vert_is_on_bdry[i] == 1 {
            // loop.cpp:85 — Boundary vertex
            // loop.cpp:86 — tripletList.emplace_back(i, localAdjList.front(), 1./8.);
            triplet_list.push(Triplet {
                row: i,
                col: *local_adj_list.first().unwrap() as usize,
                val: 1. / 8.,
            });
            // loop.cpp:87 — tripletList.emplace_back(i, localAdjList.back(), 1./8.);
            triplet_list.push(Triplet {
                row: i,
                col: *local_adj_list.last().unwrap() as usize,
                val: 1. / 8.,
            });
            // loop.cpp:88 — tripletList.emplace_back(i, i, 3./4.);
            triplet_list.push(Triplet {
                row: i,
                col: i,
                val: 3. / 4.,
            });
        } else {
            // loop.cpp:91 — const int n = localAdjList.size();
            let n = local_adj_list.len();
            // loop.cpp:92 — const SType dn = n;
            let dn = n as f64;
            // loop.cpp:93 — SType beta;
            let beta: f64;
            // loop.cpp:94-100 — beta selection
            if n == 3 {
                // loop.cpp:96 — beta = 3./16.;
                beta = 3. / 16.;
            } else {
                // loop.cpp:99 — beta = 3./8./dn;
                beta = 3. / 8. / dn;
            }
            // loop.cpp:101-104 — for each adjacency: tripletList.emplace_back(i, localAdjList[j], beta);
            for j in 0..n {
                triplet_list.push(Triplet {
                    row: i,
                    col: local_adj_list[j] as usize,
                    val: beta,
                });
            }
            // loop.cpp:105 — tripletList.emplace_back(i, i, 1.-dn*beta);
            triplet_list.push(Triplet {
                row: i,
                col: i,
                val: 1. - dn * beta,
            });
        }
    }
    // loop.cpp:108-129 — New vertices (even)
    for i in 0..ff_rows {
        // loop.cpp:111 — for(int j=0; j<3; ++j)
        for j in 0..3 {
            // loop.cpp:113 — if(NIdoubles(i,j)==0)
            if ni_doubles[i][j] == 0 {
                // loop.cpp:115 — if(FF(i,j)==-1)
                if ff[i][j] == -1 {
                    // loop.cpp:117 — Boundary vertex
                    // loop.cpp:118 — tripletList.emplace_back(NI(i,j) + n_odd, F(i,j), 1./2.);
                    triplet_list.push(Triplet {
                        row: (ni[i][j] + n_odd) as usize,
                        col: f[i][j] as usize,
                        val: 1. / 2.,
                    });
                    // loop.cpp:119 — tripletList.emplace_back(NI(i,j) + n_odd, F(i, (j+1)%3), 1./2.);
                    triplet_list.push(Triplet {
                        row: (ni[i][j] + n_odd) as usize,
                        col: f[i][(j + 1) % 3] as usize,
                        val: 1. / 2.,
                    });
                } else {
                    // loop.cpp:122 — tripletList.emplace_back(NI(i,j) + n_odd, F(i,j), 3./8.);
                    triplet_list.push(Triplet {
                        row: (ni[i][j] + n_odd) as usize,
                        col: f[i][j] as usize,
                        val: 3. / 8.,
                    });
                    // loop.cpp:123 — tripletList.emplace_back(NI(i,j) + n_odd, F(i, (j+1)%3), 3./8.);
                    triplet_list.push(Triplet {
                        row: (ni[i][j] + n_odd) as usize,
                        col: f[i][(j + 1) % 3] as usize,
                        val: 3. / 8.,
                    });
                    // loop.cpp:124 — tripletList.emplace_back(NI(i,j) + n_odd, F(i, (j+2)%3), 1./8.);
                    triplet_list.push(Triplet {
                        row: (ni[i][j] + n_odd) as usize,
                        col: f[i][(j + 2) % 3] as usize,
                        val: 1. / 8.,
                    });
                    // loop.cpp:125 — tripletList.emplace_back(NI(i,j) + n_odd, F(FF(i,j), (FFi(i,j)+2)%3), 1./8.);
                    let ffij = ff[i][j] as usize;
                    let ffiij = ffi[i][j] as usize;
                    triplet_list.push(Triplet {
                        row: (ni[i][j] + n_odd) as usize,
                        col: f[ffij][(ffiij + 2) % 3] as usize,
                        val: 1. / 8.,
                    });
                }
            }
        }
    }
    // loop.cpp:130-131 — S.resize(n_newverts, n_verts); S.setFromTriplets(...);
    // We return the triplet list directly; n_newverts is implied by the
    // maximum row index produced (== n_newverts). The caller applies S to V.
    let _ = n_newverts;
    *s = triplet_list;

    // loop.cpp:133 — Build the new topology (Every face is replaced by four)
    // loop.cpp:134 — NF.resize(F.rows()*4, 3);
    *nf = vec![[0i32; 3]; f.len() * 4];
    // loop.cpp:135-150 — split each face into four
    for i in 0..f.len() {
        // loop.cpp:137-138 — Eigen::VectorXi VI(6); VI << F(i,0), F(i,1), F(i,2), NI(i,0)+n_odd, NI(i,1)+n_odd, NI(i,2)+n_odd;
        let vi = [
            f[i][0],
            f[i][1],
            f[i][2],
            ni[i][0] + n_odd,
            ni[i][1] + n_odd,
            ni[i][2] + n_odd,
        ];
        // loop.cpp:140-144 — f0, f1, f2, f3
        let f0 = [vi[0], vi[3], vi[5]];
        let f1 = [vi[1], vi[4], vi[3]];
        let f2 = [vi[3], vi[4], vi[5]];
        let f3 = [vi[4], vi[2], vi[5]];
        // loop.cpp:146-149 — assign the four new faces
        nf[(i * 4) + 0] = f0;
        nf[(i * 4) + 1] = f1;
        nf[(i * 4) + 2] = f2;
        nf[(i * 4) + 3] = f3;
    }
    // loop.cpp:151 — return true;
    true
}

/// Port of the multi-step `igl::loop` overload.
///
/// loop.cpp:159-179
///
/// Performs `number_of_subdivs` steps of Loop subdivision in place on the
/// vertex matrix `nv` (rows of f64 triplets) and face matrix `nf`.
/// Returns false if any step fails.
fn igl_loop(nv: &mut Vec<[f64; 3]>, nf: &mut Vec<[i32; 3]>, number_of_subdivs: i32) -> bool {
    // loop.cpp:166 — NV = V;  (nv is already V on entry — in-place V==NV)
    // loop.cpp:167 — NF = F;  (nf is already F on entry — in-place F==NF)
    // loop.cpp:168-177 — for(int i=0; i<number_of_subdivs; ++i)
    for _i in 0..number_of_subdivs {
        // loop.cpp:170 — DerivedNF tempF = NF;
        let temp_f = nf.clone();
        // loop.cpp:171 — Eigen::SparseMatrix<...> S;
        let mut s: Vec<Triplet> = Vec::new();
        // loop.cpp:172 — if (!loop(NV.rows(), tempF, S, NF)) return false;
        if !igl_loop_step(nv.len(), &temp_f, &mut s, nf) {
            return false;
        }
        // loop.cpp:176 — NV = (S*NV).eval();
        // S has n_newverts rows and NV.rows() columns; apply S to NV (n_newverts x 3).
        let mut new_nv: Vec<[f64; 3]> = compute_new_vertices(&s, nv);
        // .eval() forces materialization — already materialized above.
        std::mem::swap(nv, &mut new_nv);
    }
    // loop.cpp:178 — return true;
    true
}

/// Compute `S * NV` where `S` is given as a triplet list.
///
/// loop.cpp:176 — NV = (S*NV).eval();
///
/// The number of output rows is `max(row index) + 1` (== n_newverts, since the
/// triplet rows cover [0, n_newverts)).
fn compute_new_vertices(s: &[Triplet], nv: &[[f64; 3]]) -> Vec<[f64; 3]> {
    // Determine output row count (n_newverts).
    let mut n_rows: usize = 0;
    for t in s.iter() {
        if t.row + 1 > n_rows {
            n_rows = t.row + 1;
        }
    }
    let mut result: Vec<[f64; 3]> = vec![[0.0; 3]; n_rows];
    // Eigen evaluates the sparse-dense product by accumulating, in column-major
    // order over S, but the sum is the same regardless of order: each output row
    // r accumulates sum_k S(r,k) * NV(k,:).
    for t in s.iter() {
        let src = nv[t.col];
        result[t.row][0] += t.val * src[0];
        result[t.row][1] += t.val * src[1];
        result[t.row][2] += t.val * src[2];
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    // A single tetrahedron is a closed manifold mesh with no boundary,
    // so Loop subdivision should succeed and quadruple the face count.
    fn tetrahedron() -> TriangleMesh {
        let vertices = vec![
            Point3F::new(0.0, 0.0, 0.0),
            Point3F::new(1.0, 0.0, 0.0),
            Point3F::new(0.0, 1.0, 0.0),
            Point3F::new(0.0, 0.0, 1.0),
        ];
        let indices = vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(0, 3, 2),
            Triangle::new(1, 2, 3),
        ];
        TriangleMesh::from_parts(vertices, indices)
    }

    #[test]
    fn test_smooth_closed_mesh() {
        let mesh = tetrahedron();
        let mut ok = false;
        let result = TriangleMeshDeal::smooth_triangle_mesh(&mesh, &mut ok);
        assert!(ok);
        // loop.cpp:134 — NF.resize(F.rows()*4, 3): each face becomes four.
        assert_eq!(result.indices().len(), mesh.indices().len() * 4);
        // n_newverts = n_odd (4 verts) + n_even (6 unique edges of a tetrahedron) = 10.
        assert_eq!(result.vertices().len(), 10);
    }
}
