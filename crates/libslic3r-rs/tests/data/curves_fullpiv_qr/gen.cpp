// Native vectors for Eigen's FullPivHouseholderQR<MatrixXf> as Geometry/Curves.hpp:168-173 uses it
// (`T.fullPivHouseholderQr()` and `QR.solve(data_points.row(dim).transpose())`), and for
// Geometry::fit_cubic_bspline with Dimension 2 as SeamPlacer.cpp:1340-1341 calls it. Build with the
// engine's compile and link line (target/parity-tools/build.sh, the Eigen slicer_cli is built
// with); run: gen <cases> > vectors.txt. Floats are printed as the hex of their IEEE bits.
// Lines:
//   q <rows> <cols> <T column-major> <b> => <rank> <x>
//   f <n> <segments> <x_i y_i p_i w_i>... => <cols> <coefficients row-major>
#include "libslic3r/Geometry/Curves.hpp"
#include <Eigen/Dense>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <vector>

using namespace Slic3r;

static uint32_t g_seed = 951u;
static uint32_t lcg() { g_seed = g_seed * 1664525u + 1013904223u; return g_seed; }
static float uf(float lo, float hi) { return lo + (hi - lo) * float(lcg() >> 8) / float(1u << 24); }
static void pf(float v) { uint32_t u; std::memcpy(&u, &v, 4); std::printf(" %08x", u); }

int main(int argc, char **argv)
{
    int cases = argc > 1 ? std::atoi(argv[1]) : 400;
    for (int c = 0; c < cases; ++c) {
        // q: general matrices: dense random, B-spline-like bands, repeated columns (rank deficient),
        // small integers (exact ties in the pivot search).
        int kind = c % 4;
        int rows = 2 + int(lcg() % 120);
        int cols = 1 + int(lcg() % std::min(rows, 40));
        Eigen::MatrixXf T(rows, cols);
        T.setZero();
        for (int j = 0; j < cols; ++j)
            for (int i = 0; i < rows; ++i) {
                if (kind == 0) T(i, j) = uf(-1.f, 1.f);
                else if (kind == 1) { int d = i * cols / rows - j; if (d >= -2 && d <= 1) T(i, j) = uf(0.f, 0.7f) * uf(0.3f, 3.f); }
                else if (kind == 2) T(i, j) = (j > 0 && (lcg() % 5) == 0) ? T(i, j - 1) : uf(-2.f, 2.f);
                else T(i, j) = float(int(lcg() % 7) - 3);
            }
        Eigen::VectorXf b(rows);
        for (int i = 0; i < rows; ++i) b(i) = uf(-50.f, 50.f);
        const auto QR = T.fullPivHouseholderQr();
        Eigen::VectorXf x = QR.solve(b);
        std::printf("q %d %d", rows, cols);
        for (int j = 0; j < cols; ++j)
            for (int i = 0; i < rows; ++i) pf(T(i, j));
        for (int i = 0; i < rows; ++i) pf(b(i));
        std::printf(" => %d", int(QR.rank()));
        for (int i = 0; i < cols; ++i) pf(x(i));
        std::printf("\n");
    }
    for (int c = 0; c < cases; ++c) {
        // f: seam-string-like fits (layer z as the observation point, xy jitter, weights from the
        // angle penalty range), Dimension 2, endpoints_level_of_freedom 0.
        size_t n = 6 + lcg() % 300;
        std::vector<Vec2f> obs;
        std::vector<float> pts, ws;
        float z = uf(0.1f, 2.f), lh = (lcg() % 3 == 0) ? 0.16f : uf(0.08f, 0.32f);
        float cx = uf(-120.f, 120.f), cy = uf(-120.f, 120.f), dx = uf(-0.05f, 0.05f), dy = uf(-0.05f, 0.05f);
        float total_length = 0.f;
        for (size_t i = 0; i < n; ++i) {
            obs.emplace_back(cx + dx * float(i) + uf(-0.4f, 0.4f), cy + dy * float(i) + uf(-0.4f, 0.4f));
            pts.push_back(z);
            z += lh;
            ws.push_back(1.0f / (0.1f + uf(0.f, 1.f)) + ((lcg() % 17) == 0 ? 3.f : 0.f));
            total_length += lh;
        }
        size_t segments = std::max(size_t(1), size_t(std::max(0.0f, total_length) / 4.0f));
        auto curve = Geometry::fit_cubic_bspline(obs, pts, ws, segments);
        std::printf("f %zu %zu", n, segments);
        for (size_t i = 0; i < n; ++i) { pf(obs[i].x()); pf(obs[i].y()); pf(pts[i]); pf(ws[i]); }
        std::printf(" => %d", int(curve.coefficients.cols()));
        for (int r = 0; r < 2; ++r)
            for (int col = 0; col < curve.coefficients.cols(); ++col) pf(curve.coefficients(r, col));
        std::printf("\n");
    }
    return 0;
}
