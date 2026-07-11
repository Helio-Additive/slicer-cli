// R200: raycast kernel in its OWN translation unit. R190 appended this code to
// secol_shim.cpp and the collapse kernel's FP codegen shifted with the TU
// context (R199 forensics: 15982-vs-15986 tris from the same source). One
// kernel per TU pins codegen; secol_shim.cpp is back to its R188 content.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <Eigen/Geometry>

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
