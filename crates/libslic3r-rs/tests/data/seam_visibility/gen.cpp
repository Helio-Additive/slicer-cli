// Native vectors for SeamPlacer's GlobalModelInfo::calculate_point_visibility (SeamPlacer.cpp:301-326)
// over the pin's KDTreeIndirect (KDTreeIndirect.hpp) built as compute_global_occlusion builds it
// (SeamPlacer.cpp:264-271, 611-612). The struct lives in SeamPlacer.cpp's SeamPlacerImpl namespace;
// this generator carries CoordinateFunctor and the function body verbatim over the fields it reads.
// Samples and queries come from an LCG; half of the queries sit at the search radius from a sample
// (computed in float), where the fused distance of find_nearby_points decides whether the sample counts.
// Output (raw float bits, %08x):
//   "R <radius>" , "S <i> <px> <py> <pz> <nx> <ny> <nz> <visibility>" per sample,
//   "Q <qx> <qy> <qz> <visibility> <nearby count>" per query.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <vector>
#include "libslic3r/Point.hpp"
#include "libslic3r/KDTreeIndirect.hpp"

using namespace Slic3r;

// SeamPlacer.cpp:264-271
struct CoordinateFunctor
{
    const std::vector<Vec3f> *coordinates;
    CoordinateFunctor(const std::vector<Vec3f> *coords) : coordinates(coords) {}
    CoordinateFunctor() : coordinates(nullptr) {}

    const float &operator()(size_t idx, size_t dim) const { return coordinates->operator[](idx)[dim]; }
};

struct Samples
{
    std::vector<Vec3f> positions;
    std::vector<Vec3f> normals;
};

// The fields of GlobalModelInfo (SeamPlacer.cpp:274-280) that calculate_point_visibility reads.
struct GlobalModelInfo
{
    Samples                                     mesh_samples;
    std::vector<float>                          mesh_samples_visibility;
    CoordinateFunctor                           mesh_samples_coordinate_functor;
    KDTreeIndirect<3, float, CoordinateFunctor> mesh_samples_tree{CoordinateFunctor{}};
    float                                       mesh_samples_radius;

    // SeamPlacer.cpp:301-326, verbatim.
    float calculate_point_visibility(const Vec3f &position) const
    {
        std::vector<size_t> points = find_nearby_points(mesh_samples_tree, position, mesh_samples_radius);
        if (points.empty()) { return 1.0f; }

        auto compute_dist_to_plane = [](const Vec3f &position, const Vec3f &plane_origin, const Vec3f &plane_normal) {
            Vec3f orig_to_point = position - plane_origin;
            return std::abs(orig_to_point.dot(plane_normal));
        };

        float total_weight     = 0;
        float total_visibility = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            size_t sample_idx = points[i];

            Vec3f sample_point  = this->mesh_samples.positions[sample_idx];
            Vec3f sample_normal = this->mesh_samples.normals[sample_idx];

            float weight = mesh_samples_radius - compute_dist_to_plane(position, sample_point, sample_normal);
            weight += (mesh_samples_radius - (position - sample_point).norm());
            total_visibility += weight * mesh_samples_visibility[sample_idx];
            total_weight += weight;
        }

        return total_visibility / total_weight;
    }
};

static uint64_t g_seed = 0x2545F4914F6CDD1Dull;
static uint32_t lcg() { g_seed = g_seed * 6364136223846793005ull + 1442695040888963407ull; return uint32_t(g_seed >> 33); }
static float    unitf() { return float(double(lcg()) / 2147483648.0); }
static unsigned fb(float f) { unsigned u; std::memcpy(&u, &f, 4); return u; }

static Vec3f random_unit()
{
    for (;;) {
        Vec3f v(2.f * unitf() - 1.f, 2.f * unitf() - 1.f, 2.f * unitf() - 1.f);
        float n2 = v.squaredNorm();
        if (n2 > 0.01f && n2 <= 1.f) return v / std::sqrt(n2);
    }
}

int main()
{
    const int scenarios[][2] = {{3000, 2000}, {20000, 4000}, {500, 1500}};
    const float radii[]      = {1.5f, 0.75f, 3.0f};
    const float sizes[]      = {20.f, 25.f, 12.f};
    for (int s = 0; s < 3; ++s) {
        GlobalModelInfo gmi;
        const int       n = scenarios[s][0], nq = scenarios[s][1];
        for (int i = 0; i < n; ++i) {
            gmi.mesh_samples.positions.emplace_back(sizes[s] * unitf() - 0.5f * sizes[s], sizes[s] * unitf(), sizes[s] * unitf() + 0.2f);
            gmi.mesh_samples.normals.emplace_back(random_unit());
            gmi.mesh_samples_visibility.emplace_back(unitf());
        }
        gmi.mesh_samples_radius              = radii[s];
        gmi.mesh_samples_coordinate_functor = CoordinateFunctor(&gmi.mesh_samples.positions);
        gmi.mesh_samples_tree = KDTreeIndirect<3, float, CoordinateFunctor>(gmi.mesh_samples_coordinate_functor, gmi.mesh_samples.positions.size());

        std::printf("SCENARIO %d\nR %08x\n", s, fb(gmi.mesh_samples_radius));
        for (int i = 0; i < n; ++i) {
            const Vec3f &p = gmi.mesh_samples.positions[i], &nn = gmi.mesh_samples.normals[i];
            std::printf("S %d %08x %08x %08x %08x %08x %08x %08x\n", i, fb(p.x()), fb(p.y()), fb(p.z()), fb(nn.x()), fb(nn.y()), fb(nn.z()),
                        fb(gmi.mesh_samples_visibility[i]));
        }
        for (int q = 0; q < nq; ++q) {
            Vec3f pos;
            if (q % 2 == 0) {
                pos = Vec3f(sizes[s] * unitf() - 0.5f * sizes[s], sizes[s] * unitf(), sizes[s] * unitf() + 0.2f);
            } else {
                // At the radius from a sample.
                const Vec3f &c = gmi.mesh_samples.positions[lcg() % uint32_t(n)];
                Vec3f        d = random_unit();
                pos            = Vec3f(c.x() + gmi.mesh_samples_radius * d.x(), c.y() + gmi.mesh_samples_radius * d.y(), c.z() + gmi.mesh_samples_radius * d.z());
            }
            float  vis = gmi.calculate_point_visibility(pos);
            size_t cnt = find_nearby_points(gmi.mesh_samples_tree, pos, gmi.mesh_samples_radius).size();
            std::printf("Q %08x %08x %08x %08x %zu\n", fb(pos.x()), fb(pos.y()), fb(pos.z()), fb(vis), cnt);
        }
    }
    return 0;
}
