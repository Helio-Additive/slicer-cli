// R187/R188: ShortEdgeCollapse kernels, SEPARATE TU — compiled against the
// SAME Eigen the native binary uses (nix eigen3 3.4.0 via pkg-config; the
// transform shims in eigen_transform_shim.cpp stay on the vendored 3.3.7 that
// their byte-locked gates were validated with). Eigen 3.3.7 vs 3.4.0 differ in
// f32 normalized()/norm codegen — that version skew was the R187 dots ulp gap.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <Eigen/Geometry>

// ---------------------------------------------------------------------------
// R187: exact-native min_vertex_dot_product kernel (ShortEdgeCollapse.cpp:43-55)
// Copied verbatim from the reference tree so the Eigen f32 codegen matches the
// native binary (same compiler/flags): TriangleMesh.hpp:330-334 face normals
// (note the DOUBLE normalize) + NormalUtils.cpp:5-15/96-126 nelson-weighted
// vertex normals + the min-dot loop.
// ---------------------------------------------------------------------------
#include <vector>
#include <Eigen/Geometry>

extern "C" void secol_min_vertex_dots(const float *verts, int64_t n_verts,
                                      const int32_t *indices, int64_t n_tris,
                                      float *out)
{
    using Vec3f = Eigen::Matrix<float, 3, 1>;
    using Vec3i = Eigen::Matrix<int, 3, 1>;

    std::vector<Vec3f> vertices(n_verts);
    for (int64_t i = 0; i < n_verts; ++i)
        vertices[i] = Vec3f(verts[3 * i], verts[3 * i + 1], verts[3 * i + 2]);
    std::vector<Vec3i> faces(n_tris);
    for (int64_t i = 0; i < n_tris; ++i)
        faces[i] = Vec3i(indices[3 * i], indices[3 * i + 1], indices[3 * i + 2]);

    // its_face_normals (TriangleMesh.hpp:331-334): face_normal = normalized
    // cross, then face_normal_normalized normalizes AGAIN (native double
    // normalize, copied exactly).
    std::vector<Vec3f> face_normals;
    face_normals.reserve(n_tris);
    for (const Vec3i &face : faces) {
        const Vec3f v[3]{vertices[face[0]], vertices[face[1]], vertices[face[2]]};
        Vec3f fn = (v[1] - v[0]).cross(v[2] - v[1]).normalized();
        face_normals.push_back(fn.normalized());
    }

    // NormalUtils::create_normals_nelson_weighted (NormalUtils.cpp:96-126).
    std::vector<Vec3f> normals(n_verts, Vec3f(0.f, 0.f, 0.f));
    std::vector<float> count(n_verts, 0.f);
    for (const Vec3i &indice : faces) {
        // create_triangle_normal (NormalUtils.cpp:5-15)
        const Vec3f &v0 = vertices[indice[0]];
        const Vec3f &v1 = vertices[indice[1]];
        const Vec3f &v2 = vertices[indice[2]];
        Vec3f normal = (v1 - v0).cross(v2 - v0);
        normal.normalize();

        float e0 = (v0 - v1).norm();
        float e1 = (v1 - v2).norm();
        float e2 = (v2 - v0).norm();

        Vec3f coefs(e0 * e2, e0 * e1, e1 * e2);
        for (int i = 0; i < 3; ++i) {
            const float &weight = coefs[i];
            normals[indice[i]] += normal * weight;
            count[indice[i]] += weight;
        }
    }
    for (int64_t i = 0; i < n_verts; ++i)
        normals[i] /= count[i];

    // ShortEdgeCollapse.cpp:43-55 — the min-dot loop.
    for (int64_t i = 0; i < n_verts; ++i)
        out[i] = 1.0f;
    for (int64_t face_idx = 0; face_idx < n_tris; ++face_idx) {
        const Vec3i &t = faces[face_idx];
        const Vec3f &n = face_normals[face_idx];
        out[t[0]] = std::min(out[t[0]], n.dot(normals[t[0]]));
        out[t[1]] = std::min(out[t[1]], n.dot(normals[t[1]]));
        out[t[2]] = std::min(out[t[2]], n.dot(normals[t[2]]));
    }
}

// ---------------------------------------------------------------------------
// R188: FULL native its_short_edge_collpase (ShortEdgeCollapse.cpp:11-185),
// copied verbatim. R187's dots-only shim left round-1 at ±6 faces: the edge
// squaredNorm-vs-threshold check (line 113) and the f32/f64-mixed edge_len
// evolution (line 95) are additional Eigen-f32 codegen surfaces. Whole kernel
// runs here so ALL float decisions come from the same compiler+flags as native.
// Neighbors: create_face_neighbors_index (MeshSplitImpl.hpp:292-341) sequential
// copy — on a manifold mesh each directed edge has exactly one opposite match,
// so ex_seq == ex_tbb. std::shuffle/mt19937_64: real libc++ (== native).
// ---------------------------------------------------------------------------
#include <unordered_map>
#include <random>
#include <algorithm>

extern "C" int64_t secol_collapse(const float *verts, int64_t n_verts,
                                  const int32_t *indices, int64_t n_tris,
                                  int64_t target_triangle_count,
                                  float *out_verts, int64_t *out_n_verts,
                                  int32_t *out_indices, int64_t *out_n_tris)
{
    using Vec3f = Eigen::Matrix<float, 3, 1>;
    using Vec3i = Eigen::Matrix<int, 3, 1>;
    using Vec2i = Eigen::Matrix<int, 2, 1>;

    std::vector<Vec3f> mesh_vertices(n_verts);
    for (int64_t i = 0; i < n_verts; ++i)
        mesh_vertices[i] = Vec3f(verts[3 * i], verts[3 * i + 1], verts[3 * i + 2]);
    std::vector<Vec3i> mesh_indices(n_tris);
    for (int64_t i = 0; i < n_tris; ++i)
        mesh_indices[i] = Vec3i(indices[3 * i], indices[3 * i + 1], indices[3 * i + 2]);

    // --- VertexFaceIndex::create (TriangleMesh.cpp:1903-1926) ---
    std::vector<size_t> v2f_start(n_verts + 1, 0);
    for (const Vec3i &face : mesh_indices) {
        ++v2f_start[face(0) + 1];
        ++v2f_start[face(1) + 1];
        ++v2f_start[face(2) + 1];
    }
    for (size_t i = 2; i < v2f_start.size(); ++i)
        v2f_start[i] += v2f_start[i - 1];
    std::vector<size_t> v2f_all(v2f_start.back(), 0);
    for (size_t face_idx = 0; face_idx < mesh_indices.size(); ++face_idx) {
        const Vec3i &face = mesh_indices[face_idx];
        for (int i = 0; i < 3; ++i)
            v2f_all[v2f_start[face(i)]++] = face_idx;
    }
    for (auto i = int(v2f_start.size()) - 1; i > 0; --i)
        v2f_start[i] = v2f_start[i - 1];
    v2f_start.front() = 0;

    // --- create_face_neighbors_index (MeshSplitImpl.hpp:292-341), sequential ---
    static constexpr int no_value = -1;
    std::vector<Vec3i> triangles_neighbors(n_tris, Vec3i(no_value, no_value, no_value));
    for (size_t face_idx = 0; face_idx < mesh_indices.size(); ++face_idx) {
        Vec3i &neighbor = triangles_neighbors[face_idx];
        const Vec3i &triangle_indices = mesh_indices[face_idx];
        for (int edge_index = 0; edge_index < 3; ++edge_index) {
            int &neighbor_edge = neighbor[edge_index];
            if (neighbor_edge != no_value)
                continue;
            // its_triangle_edge (TriangleMesh.hpp:256-260)
            int next_edge_idx = (edge_index == 2) ? 0 : edge_index + 1;
            Vec2i edge_indices(triangle_indices[edge_index], triangle_indices[next_edge_idx]);
            for (size_t k = v2f_start[edge_indices[0]]; k < v2f_start[edge_indices[0] + 1]; ++k) {
                const size_t other_face = v2f_all[k];
                if (other_face <= face_idx) continue;
                const Vec3i &face_indices = mesh_indices[other_face];
                // its_triangle_vertex_index (TriangleMesh.hpp:249-254)
                int vertex_index = edge_indices[1] == face_indices[0] ? 0 :
                                   edge_indices[1] == face_indices[1] ? 1 :
                                   edge_indices[1] == face_indices[2] ? 2 : -1;
                if (vertex_index < 0) continue;
                if (edge_indices[0] != face_indices[(vertex_index + 1) % 3]) continue;
                if (triangles_neighbors[other_face][vertex_index] != no_value)
                    continue;
                neighbor_edge = other_face;
                triangles_neighbors[other_face][vertex_index] = face_idx;
                break;
            }
        }
    }

    // --- its_short_edge_collpase body (ShortEdgeCollapse.cpp:11-185) ---
    std::vector<size_t> vertices_index_mapping(n_verts);
    for (size_t idx = 0; idx < vertices_index_mapping.size(); ++idx)
        vertices_index_mapping[idx] = idx;
    std::vector<size_t> flatten_queue;
    auto get_final_index = [&vertices_index_mapping, &flatten_queue](const size_t &orig_index) {
        flatten_queue.clear();
        size_t idx = orig_index;
        while (vertices_index_mapping[idx] != idx) {
            flatten_queue.push_back(idx);
            idx = vertices_index_mapping[idx];
        }
        for (size_t i : flatten_queue) {
            vertices_index_mapping[i] = idx;
        }
        return idx;
    };

    std::vector<bool> face_removal_flags(n_tris, false);

    std::vector<float> min_vertex_dot_product(n_verts, 1);
    secol_min_vertex_dots(verts, n_verts, indices, n_tris, min_vertex_dot_product.data());


    auto remove_face = [&triangles_neighbors, &face_removal_flags](int face_idx, int other_face_idx) {
        if (face_idx < 0) {
            return;
        }
        face_removal_flags[face_idx] = true;
        Vec3i neighbors = triangles_neighbors[face_idx];
        int n_a = neighbors[0] != other_face_idx ? neighbors[0] : neighbors[1];
        int n_b = neighbors[2] != other_face_idx ? neighbors[2] : neighbors[1];
        if (n_a > 0)
            for (int i = 0; i < 3; ++i) {
                int &n = triangles_neighbors[n_a][i];
                if (n == face_idx) {
                    n = n_b;
                    break;
                }
            }
        if (n_b > 0)
            for (int i = 0; i < 3; ++i) {
                int &n = triangles_neighbors[n_b][i];
                if (n == face_idx) {
                    n = n_a;
                    break;
                }
            }
    };

    std::mt19937_64 generator { 27644437 };
    std::vector<size_t> face_indices(n_tris);
    for (size_t idx = 0; idx < face_indices.size(); ++idx)
        face_indices[idx] = idx;
    std::vector<size_t> tmp_face_indices(n_tris);

    float decimation_ratio = 1.0f;
    float edge_len = 0.2f;

    while (face_indices.size() > (size_t) target_triangle_count) {
        edge_len = edge_len * (1.0f + 1.0 - decimation_ratio);
        float max_edge_len_squared = edge_len * edge_len;

        std::shuffle(face_indices.begin(), face_indices.end(), generator);

        int allowed_face_removals = int(face_indices.size()) - int(target_triangle_count);
        for (const size_t &face_idx : face_indices) {
            if (face_removal_flags[face_idx]) {
                continue;
            }

            for (size_t edge_idx = 0; edge_idx < 3; ++edge_idx) {
                size_t vertex_index_keep = get_final_index(mesh_indices[face_idx][edge_idx]);
                size_t vertex_index_remove = get_final_index(mesh_indices[face_idx][(edge_idx + 1) % 3]);
                if ((mesh_vertices[vertex_index_keep] - mesh_vertices[vertex_index_remove]).squaredNorm()
                        > max_edge_len_squared) {
                    continue;
                }
                if (min_vertex_dot_product[vertex_index_remove] < min_vertex_dot_product[vertex_index_keep]) {
                    size_t tmp = vertex_index_keep;
                    vertex_index_keep = vertex_index_remove;
                    vertex_index_remove = tmp;
                }

                {
                    vertices_index_mapping[vertex_index_remove] = vertices_index_mapping[vertex_index_keep];
                }

                int neighbor_to_remove_face_idx = triangles_neighbors[face_idx][edge_idx];
                remove_face(face_idx, neighbor_to_remove_face_idx);
                remove_face(neighbor_to_remove_face_idx, face_idx);
                allowed_face_removals -= 2;

                break;
            }

            if (allowed_face_removals <= 0) { break; }
        }

        size_t prev_size = face_indices.size();
        tmp_face_indices.clear();
        for (size_t face_idx : face_indices) {
            if (!face_removal_flags[face_idx]) {
                tmp_face_indices.push_back(face_idx);
            }
        }
        face_indices.swap(tmp_face_indices);

        decimation_ratio = float(prev_size - face_indices.size()) / float(prev_size);
    }

    std::unordered_map<size_t, size_t> final_vertices_mapping;
    std::vector<Vec3f> final_vertices;
    std::vector<Vec3i> final_indices;
    final_indices.reserve(face_indices.size());
    for (size_t idx : face_indices) {
        Vec3i final_face;
        for (size_t i = 0; i < 3; ++i) {
            final_face[i] = get_final_index(mesh_indices[idx][i]);
        }
        if (final_face[0] == final_face[1] || final_face[1] == final_face[2] || final_face[2] == final_face[0]) {
            continue;
        }

        for (size_t i = 0; i < 3; ++i) {
            if (final_vertices_mapping.find(final_face[i]) == final_vertices_mapping.end()) {
                final_vertices_mapping[final_face[i]] = final_vertices.size();
                final_vertices.push_back(mesh_vertices[final_face[i]]);
            }
            final_face[i] = final_vertices_mapping[final_face[i]];
        }

        final_indices.push_back(final_face);
    }

    if ((int64_t) final_vertices.size() > n_verts || (int64_t) final_indices.size() > n_tris)
        return -1; // caller buffers sized at input counts; collapse only shrinks
    for (size_t i = 0; i < final_vertices.size(); ++i) {
        out_verts[3 * i] = final_vertices[i].x();
        out_verts[3 * i + 1] = final_vertices[i].y();
        out_verts[3 * i + 2] = final_vertices[i].z();
    }
    for (size_t i = 0; i < final_indices.size(); ++i) {
        out_indices[3 * i] = final_indices[i][0];
        out_indices[3 * i + 1] = final_indices[i][1];
        out_indices[3 * i + 2] = final_indices[i][2];
    }
    *out_n_verts = (int64_t) final_vertices.size();
    *out_n_tris = (int64_t) final_indices.size();
    return 0;
}

// ---------------------------------------------------------------------------
// R188b: native sample_its_uniform_parallel (TriangleSetSampling.cpp:9-68).
// With decimation bit-exact, the next seam-chain divergence was here: the f32
// cross/norm triangle areas differ by ulps between nalgebra and Eigen codegen
// (total_area was 1 ulp off), shifting the area-prefix map and every sample.
// tbb::parallel_for loops write disjoint slots — sequential is identical.
// mt19937_64 + uniform_real_distribution: REAL libc++ (== native binary).
// ---------------------------------------------------------------------------
#include <map>

extern "C" void secol_sample_uniform(const float *verts, int64_t n_verts,
                                     const int32_t *indices, int64_t n_tris,
                                     int64_t samples_count,
                                     float *out_positions,   // 3*samples_count
                                     float *out_normals,     // 3*samples_count
                                     int64_t *out_tri_idx,   // samples_count
                                     float *out_total_area)
{
    using Vec3f = Eigen::Matrix<float, 3, 1>;
    using Vec3d = Eigen::Matrix<double, 3, 1>;
    using Vec3i = Eigen::Matrix<int, 3, 1>;

    std::vector<Vec3f> vertices(n_verts);
    for (int64_t i = 0; i < n_verts; ++i)
        vertices[i] = Vec3f(verts[3 * i], verts[3 * i + 1], verts[3 * i + 2]);
    std::vector<Vec3i> tris(n_tris);
    for (int64_t i = 0; i < n_tris; ++i)
        tris[i] = Vec3i(indices[3 * i], indices[3 * i + 1], indices[3 * i + 2]);

    std::vector<double> triangles_area(n_tris);
    for (int64_t t_idx = 0; t_idx < n_tris; ++t_idx) {
        const Vec3f &a = vertices[tris[t_idx].x()];
        const Vec3f &b = vertices[tris[t_idx].y()];
        const Vec3f &c = vertices[tris[t_idx].z()];
        double area = double(0.5 * (b - a).cross(c - a).norm());
        triangles_area[t_idx] = area;
    }

    std::map<double, size_t> area_sum_to_triangle_idx;
    float area_sum = 0;
    for (size_t t_idx = 0; t_idx < triangles_area.size(); ++t_idx) {
        area_sum += triangles_area[t_idx];
        area_sum_to_triangle_idx[area_sum] = t_idx;
    }

    std::mt19937_64 mersenne_engine { 27644437 };
    std::uniform_real_distribution<double> fdistribution;

    auto get_random = [&fdistribution, &mersenne_engine]() {
        return Vec3d { fdistribution(mersenne_engine), fdistribution(mersenne_engine), fdistribution(mersenne_engine) };
    };

    std::vector<Vec3d> random_samples(samples_count);
    std::generate(random_samples.begin(), random_samples.end(), get_random);

    *out_total_area = area_sum;
    for (int64_t s_idx = 0; s_idx < samples_count; ++s_idx) {
        double t_sample = random_samples[s_idx].x() * area_sum;
        size_t t_idx = area_sum_to_triangle_idx.upper_bound(t_sample)->second;

        double sq_u = std::sqrt(random_samples[s_idx].y());
        double v = random_samples[s_idx].z();

        Vec3f A = vertices[tris[t_idx].x()];
        Vec3f B = vertices[tris[t_idx].y()];
        Vec3f C = vertices[tris[t_idx].z()];

        Vec3f pos = A * (1 - sq_u) + B * (sq_u * (1 - v)) + C * (v * sq_u);
        Vec3f nrm = ((B - A).cross(C - B)).normalized();
        out_positions[3 * s_idx] = pos.x();
        out_positions[3 * s_idx + 1] = pos.y();
        out_positions[3 * s_idx + 2] = pos.z();
        out_normals[3 * s_idx] = nrm.x();
        out_normals[3 * s_idx + 1] = nrm.y();
        out_normals[3 * s_idx + 2] = nrm.z();
        out_tri_idx[s_idx] = (int64_t) t_idx;
    }
}

// ---------------------------------------------------------------------------
// R190: native raycast_visibility (SeamPlacer.cpp:135-214, non-negative-volume
// branch — Benchy has no negative volumes) + Frame/sample_hemisphere_uniform
// (SeamPlacer.cpp:77-125) + its_face_normal (TriangleMesh.hpp:331-334) +
// AABBTreeIndirect build/first-hit (local verbatim header copy). tbb loop is
// per-slot → sequential identical. 112/750k edge-grazing ray decisions differed
// between nalgebra and the native Eigen/igl codegen; whole kernel runs here.
// ---------------------------------------------------------------------------
#include "aabb_tree_indirect_native.hpp"

namespace secol_raycast_detail {
using Vec3f = Eigen::Matrix<float, 3, 1>;
using Vec2f = Eigen::Matrix<float, 2, 1>;
using Vec3d = Eigen::Matrix<double, 3, 1>;
using Vec3i = Eigen::Matrix<int, 3, 1>;
static constexpr double PI = 3.141592653589793238;

// SeamPlacer.cpp:77-110
class Frame
{
public:
    Frame()
    {
        mX = Vec3f(1, 0, 0);
        mY = Vec3f(0, 1, 0);
        mZ = Vec3f(0, 0, 1);
    }

    void set_from_z(const Vec3f &z)
    {
        mZ         = z.normalized();
        Vec3f tmpZ = mZ;
        Vec3f tmpX = (std::abs(tmpZ.x()) > 0.99f) ? Vec3f(0, 1, 0) : Vec3f(1, 0, 0);
        mY         = (tmpZ.cross(tmpX)).normalized();
        mX         = mY.cross(tmpZ);
    }

    Vec3f to_world(const Vec3f &a) const { return a.x() * mX + a.y() * mY + a.z() * mZ; }

private:
    Vec3f mX, mY, mZ;
};

// SeamPlacer.cpp:120-125
inline Vec3f sample_hemisphere_uniform(const Vec2f &samples)
{
    float term1 = 2.0f * float(PI) * samples.x();
    float term2 = 2.0f * sqrt(samples.y() - samples.y() * samples.y());
    return {cos(term1) * term2, sin(term1) * term2, abs(1.0f - 2.0f * samples.y())};
}

// TriangleMesh.hpp:331-334 (double normalize)
inline Vec3f its_face_normal_local(const std::vector<Vec3f> &vertices, const Vec3i &face)
{
    const Vec3f v[3]{vertices[face[0]], vertices[face[1]], vertices[face[2]]};
    return (v[1] - v[0]).cross(v[2] - v[1]).normalized().normalized();
}
} // namespace secol_raycast_detail

extern "C" void secol_raycast_visibility(const float *verts, int64_t n_verts,
                                         const int32_t *indices, int64_t n_tris,
                                         const float *sample_positions,
                                         const float *sample_normals,
                                         int64_t n_samples,
                                         int64_t sqr_rays_per_sample_point,
                                         float *out_visibility)
{
    using namespace secol_raycast_detail;

    std::vector<Vec3f> vertices(n_verts);
    for (int64_t i = 0; i < n_verts; ++i)
        vertices[i] = Vec3f(verts[3 * i], verts[3 * i + 1], verts[3 * i + 2]);
    std::vector<Vec3i> faces(n_tris);
    for (int64_t i = 0; i < n_tris; ++i)
        faces[i] = Vec3i(indices[3 * i], indices[3 * i + 1], indices[3 * i + 2]);

    auto raycasting_tree = Slic3r::AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(vertices, faces);

    // SeamPlacer.cpp:142-152 — hemisphere ray directions.
    float              step_size = 1.0f / sqr_rays_per_sample_point;
    std::vector<Vec3f> precomputed_sample_directions(sqr_rays_per_sample_point * sqr_rays_per_sample_point);
    for (size_t x_idx = 0; x_idx < (size_t) sqr_rays_per_sample_point; ++x_idx) {
        float sample_x = x_idx * step_size + step_size / 2.0;
        for (size_t y_idx = 0; y_idx < (size_t) sqr_rays_per_sample_point; ++y_idx) {
            size_t dir_index                         = x_idx * sqr_rays_per_sample_point + y_idx;
            float  sample_y                          = y_idx * step_size + step_size / 2.0;
            precomputed_sample_directions[dir_index] = sample_hemisphere_uniform({sample_x, sample_y});
        }
    }

    // SeamPlacer.cpp:157-181 (model_contains_negative_parts == false branch).
    const float decrease_step = 1.0f / (sqr_rays_per_sample_point * sqr_rays_per_sample_point);
    for (int64_t s_idx = 0; s_idx < n_samples; ++s_idx) {
        out_visibility[s_idx] = 1.0f;
        const Vec3f center(sample_positions[3 * s_idx], sample_positions[3 * s_idx + 1], sample_positions[3 * s_idx + 2]);
        const Vec3f normal(sample_normals[3 * s_idx], sample_normals[3 * s_idx + 1], sample_normals[3 * s_idx + 2]);
        Frame f;
        f.set_from_z(normal);

        for (const auto &dir : precomputed_sample_directions) {
            Vec3f final_ray_dir = (f.to_world(dir));
            igl::Hit hitpoint;
            Vec3d final_ray_dir_d = final_ray_dir.cast<double>();
            Vec3d ray_origin_d    = (center + normal * 0.01f).cast<double>(); // start above surface.
            bool  hit = Slic3r::AABBTreeIndirect::intersect_ray_first_hit(vertices, faces, raycasting_tree, ray_origin_d, final_ray_dir_d, hitpoint);
            if (hit && its_face_normal_local(vertices, faces[hitpoint.id]).dot(final_ray_dir) <= 0) { out_visibility[s_idx] -= decrease_step; }
        }
    }
}
