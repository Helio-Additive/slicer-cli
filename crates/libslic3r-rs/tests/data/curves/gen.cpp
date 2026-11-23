// Native vectors for Geometry/Bicubic.hpp CubicKernelWrapper<...<float>>::kernel and
// Geometry/Curves.hpp fit_cubic_bspline / get_fitted_value (Dimension 3, float), as the seam
// placer uses them. Build with the engine's compile and link line (target/parity-tools/build.sh);
// run: gen > vectors.txt. Lines:
//   k <kind> <x> => <kernel(x)>            kind: b (B-spline), c (Catmul-Rom), l (linear)
//   fit <n> <segments> <x_i y_i z_i p_i w_i>... => <cols> <coefficients row-major> <start> <segment_size> | <fitted value at each p_i>
// Floats are printed with %a.
#include "libslic3r/Geometry/Curves.hpp"
#include <cstdio>
#include <cstdint>
#include <vector>

using namespace Slic3r;

static uint32_t g_seed = 12345u;
static uint32_t lcg() { g_seed = g_seed * 1664525u + 1013904223u; return g_seed; }
static float uf(float lo, float hi) { return lo + (hi - lo) * float(lcg() >> 8) / float(1u << 24); }

int main()
{
    for (int i = 0; i < 3000; ++i) {
        float x = uf(-2.5f, 2.5f);
        if (i < 9) x = float(i - 4) * 0.5f;
        std::printf("k b %a => %a\n", x, Geometry::CubicBSplineKernel<float>::kernel(x));
        std::printf("k c %a => %a\n", x, Geometry::CubicCatmulRomKernel<float>::kernel(x));
        std::printf("k l %a => %a\n", x, Geometry::LinearKernel<float>::kernel(x));
    }
    for (int c = 0; c < 200; ++c) {
        size_t n = 8 + lcg() % 40;
        size_t segments = 1 + lcg() % std::max<size_t>(1, n / 4);
        std::vector<Vec3f> obs;
        std::vector<float> pts, ws;
        float p = uf(-5.f, 5.f);
        float cx = uf(-50.f, 50.f), cy = uf(-50.f, 50.f), cz = uf(0.f, 20.f);
        for (size_t i = 0; i < n; ++i) {
            obs.emplace_back(cx + uf(-1.f, 1.f), cy + uf(-1.f, 1.f), cz + float(i) * 0.2f);
            pts.push_back(p);
            p += uf(0.05f, 1.f);
            ws.push_back(uf(0.1f, 2.f));
        }
        auto curve = Geometry::fit_cubic_bspline(obs, pts, ws, segments);
        std::printf("fit %zu %zu", n, segments);
        for (size_t i = 0; i < n; ++i)
            std::printf(" %a %a %a %a %a", obs[i].x(), obs[i].y(), obs[i].z(), pts[i], ws[i]);
        std::printf(" => %d", int(curve.coefficients.cols()));
        for (int r = 0; r < 3; ++r)
            for (int col = 0; col < curve.coefficients.cols(); ++col)
                std::printf(" %a", curve.coefficients(r, col));
        std::printf(" %a %a |", curve.start, curve.segment_size);
        for (size_t i = 0; i < n; ++i) {
            Vec3f v = curve.get_fitted_value(pts[i]);
            std::printf(" %a %a %a", v.x(), v.y(), v.z());
        }
        std::printf("\n");
    }
    return 0;
}
