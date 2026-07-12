// Medial-axis shim (R269): runs BOOST::POLYGON's voronoi builder — the SAME
// boost 1.87.0 headers the native binary compiles against (devbox nix profile)
// — so the Voronoi vertex doubles are bit-exact to native, then walks the
// diagram with a verbatim adaptation of Geometry/MedialAxis.cpp. The rust
// Voronoi port's vertex arithmetic differs in last ulps, which drifts every
// medial-axis width (gap-fill LINE_WIDTH / volumetric-capped F digits).
//
// Pattern: aabb_tree_indirect_native.hpp — verbatim logic, minimal local types.

#include <boost/polygon/voronoi.hpp>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <cassert>

namespace ma_shim {

using coord_t  = int64_t;
using coordf_t = double;

struct MaPoint {
    coord_t x, y;
    MaPoint() : x(0), y(0) {}
    MaPoint(coord_t x_, coord_t y_) : x(x_), y(y_) {}
    // Point(double,double) in Slic3r = coord_t(lrint(v)) — round-to-nearest.
    MaPoint(double x_, double y_) : x(coord_t(std::lrint(x_))), y(coord_t(std::lrint(y_))) {}
    MaPoint operator-(const MaPoint &o) const { return MaPoint(x - o.x, y - o.y); }
    double norm() const { return std::sqrt(double(x) * double(x) + double(y) * double(y)); }
};

struct MaLine {
    MaPoint a, b;
    MaLine() {}
    MaLine(const MaPoint &a_, const MaPoint &b_) : a(a_), b(b_) {}
    // Slic3r Line::distance_to(Point): segment-clamped distance (Line.cpp).
    double distance_to(const MaPoint &p) const {
        const double dx = double(b.x - a.x), dy = double(b.y - a.y);
        const double l2 = dx * dx + dy * dy;
        if (l2 == 0.0) {
            const double ex = double(p.x - a.x), ey = double(p.y - a.y);
            return std::sqrt(ex * ex + ey * ey);
        }
        const double t = (double(p.x - a.x) * dx + double(p.y - a.y) * dy) / l2;
        double px, py;
        if (t <= 0.0)      { px = double(a.x); py = double(a.y); }
        else if (t >= 1.0) { px = double(b.x); py = double(b.y); }
        else               { px = double(a.x) + t * dx; py = double(a.y) + t * dy; }
        const double ex = double(p.x) - px, ey = double(p.y) - py;
        return std::sqrt(ex * ex + ey * ey);
    }
    // Slic3r Line::orientation() = atan2 in [0, 2PI).
    double orientation() const {
        double angle = std::atan2(double(b.y - a.y), double(b.x - a.x));
        if (angle < 0) angle += 2.0 * M_PI;
        return angle;
    }
};

} // namespace ma_shim

// boost::polygon concept registration for MaPoint / MaLine (segment).
namespace boost { namespace polygon {
template<> struct geometry_concept<ma_shim::MaPoint> { typedef point_concept type; };
template<> struct point_traits<ma_shim::MaPoint> {
    typedef ma_shim::coord_t coordinate_type;
    static inline coordinate_type get(const ma_shim::MaPoint &pt, orientation_2d orient) {
        return (orient == HORIZONTAL) ? pt.x : pt.y;
    }
};
template<> struct geometry_concept<ma_shim::MaLine> { typedef segment_concept type; };
template<> struct segment_traits<ma_shim::MaLine> {
    typedef ma_shim::coord_t     coordinate_type;
    typedef ma_shim::MaPoint     point_type;
    static inline point_type get(const ma_shim::MaLine &line, direction_1d dir) {
        return dir.to_int() ? line.b : line.a;
    }
};
}} // namespace boost::polygon

namespace ma_shim {

using VD = boost::polygon::voronoi_diagram<double>;

// ---------------------------------------------------------------------------
// Smoke self-test: voronoi over a unit square's 4 segment lines; returns the
// number of finite primary edges. Verifies the boost builder links & runs.
// ---------------------------------------------------------------------------
extern "C" int64_t ma_selftest() {
    std::vector<MaLine> lines;
    const coord_t s = 1000000;
    MaPoint p0(coord_t(0), coord_t(0)), p1(s, coord_t(0)), p2(s, s), p3(coord_t(0), s);
    lines.emplace_back(p0, p1);
    lines.emplace_back(p1, p2);
    lines.emplace_back(p2, p3);
    lines.emplace_back(p3, p0);
    VD vd;
    boost::polygon::construct_voronoi(lines.begin(), lines.end(), &vd);
    int64_t n = 0;
    for (auto e = vd.edges().begin(); e != vd.edges().end(); ++e)
        if (e->is_primary() && e->is_finite()) ++n;
    return n;
}

} // namespace ma_shim
