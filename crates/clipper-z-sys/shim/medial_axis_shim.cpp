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
        // Verbatim Line.hpp distance_to_squared(line, point, &np):
        // v/va/vb are integer diffs cast to double; interior case is
        // (t*v - va).squaredNorm() — do NOT reconstruct the foot point,
        // that rounds differently.
        const double vx  = double(b.x - a.x), vy  = double(b.y - a.y);
        const double vax = double(p.x - a.x), vay = double(p.y - a.y);
        const double l2  = vx * vx + vy * vy;
        if (l2 == 0.0)
            return std::sqrt(vax * vax + vay * vay);
        const double t = (vax * vx + vay * vy) / l2;
        if (t <= 0.0)
            return std::sqrt(vax * vax + vay * vay);
        if (t >= 1.0) {
            const double vbx = double(p.x - b.x), vby = double(p.y - b.y);
            return std::sqrt(vbx * vbx + vby * vby);
        }
        const double ex = t * vx - vax, ey = t * vy - vay;
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
// Vertex/Edge/Cell categories stored in boost's color bits — verbatim from
// Geometry/VoronoiOffset.hpp:30-97.
// ---------------------------------------------------------------------------
enum class VertexCategory : unsigned char { OnContour, Inside, Outside, Unknown };
enum class EdgeCategory : unsigned char { PointsToContour, PointsInside, PointsOutside, Unknown };
enum class CellCategory : unsigned char { Boundary, Inside, Outside, Unknown };

inline VertexCategory vertex_category(const VD::vertex_type &v) { return static_cast<VertexCategory>(v.color()); }
inline VertexCategory vertex_category(const VD::vertex_type *v) { return static_cast<VertexCategory>(v->color()); }
inline void set_vertex_category(VD::vertex_type *v, VertexCategory c) { v->color(static_cast<VD::vertex_type::color_type>(c)); }
inline EdgeCategory edge_category(const VD::edge_type &e) { return static_cast<EdgeCategory>(e.color()); }
inline EdgeCategory edge_category(const VD::edge_type *e) { return static_cast<EdgeCategory>(e->color()); }
inline void set_edge_category(VD::edge_type *e, EdgeCategory c) { e->color(static_cast<VD::edge_type::color_type>(c)); }
inline CellCategory cell_category(const VD::cell_type &c) { return static_cast<CellCategory>(c.color()); }
inline CellCategory cell_category(const VD::cell_type *c) { return static_cast<CellCategory>(c->color()); }
inline void set_cell_category(const VD::cell_type *c, CellCategory cc) { c->color(static_cast<VD::cell_type::color_type>(cc)); }

inline const MaPoint &contour_point(const VD::cell_type &cell, const MaLine &line)
    { return (cell.source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) ? line.a : line.b; }
inline const MaPoint &contour_point(const VD::cell_type &cell, const std::vector<MaLine> &lines)
    { return contour_point(cell, lines[cell.source_index()]); }

// detail::vertex_equal_to_point — VoronoiOffset.cpp:204-226 (boost ULP compare).
template<typename VertexType>
inline bool vertex_equal_to_point(const VertexType &vertex, const MaPoint &ipt)
{
    using ulp_cmp_type = boost::polygon::detail::ulp_comparison<double>;
    ulp_cmp_type ulp_cmp;
    static constexpr int ULPS = boost::polygon::voronoi_diagram_traits<double>::vertex_equality_predicate_type::ULPS;
    return ulp_cmp(vertex.x(), double(ipt.x), ULPS) == ulp_cmp_type::EQUAL &&
           ulp_cmp(vertex.y(), double(ipt.y), ULPS) == ulp_cmp_type::EQUAL;
}
inline bool vertex_equal_to_point(const VD::vertex_type *vertex, const MaPoint &ipt)
    { return vertex_equal_to_point(*vertex, ipt); }

// detail::on_site — VoronoiOffset.cpp:236-244 (Vec2d pt = raw vertex doubles).
struct MaVec2d { double x, y; };
inline bool on_site(const std::vector<MaLine> &lines, const VD::cell_type &cell, const MaVec2d &pt)
{
    const MaLine &line = lines[cell.source_index()];
    struct VLike { double xx, yy; double x() const { return xx; } double y() const { return yy; } };
    VLike v{pt.x, pt.y};
    auto on_contour = [&v](const MaPoint &ipt) { return vertex_equal_to_point(v, ipt); };
    if (cell.contains_point())
        return on_contour(contour_point(cell, line));
    return on_contour(line.a) || on_contour(line.b);
}

inline double cross2d(double ax, double ay, double bx, double by) { return ax * by - ay * bx; }

// annotate_inside_outside — verbatim adaptation of VoronoiOffset.cpp:640-967
// (asserts and debug stripped; arithmetic untouched).
static void reset_inside_outside_annotations(VD &vd)
{
    for (const VD::vertex_type &v : vd.vertices())
        set_vertex_category(const_cast<VD::vertex_type*>(&v), VertexCategory::Unknown);
    for (const VD::edge_type &e : vd.edges())
        set_edge_category(const_cast<VD::edge_type*>(&e), EdgeCategory::Unknown);
    for (const VD::cell_type &c : vd.cells())
        set_cell_category(&c, CellCategory::Unknown);
}

static void annotate_inside_outside(VD &vd, const std::vector<MaLine> &lines)
{
    reset_inside_outside_annotations(vd);

    auto annotate_vertex = [](const VD::vertex_type *vertex, VertexCategory c) {
        if (vertex == nullptr) return;
        set_vertex_category(const_cast<VD::vertex_type*>(vertex), c);
    };
    auto annotate_edge = [](const VD::edge_type *edge, EdgeCategory c) {
        set_edge_category(const_cast<VD::edge_type*>(edge), c);
    };
    auto annotate_cell = [](const VD::cell_type *cell, CellCategory new_cc) -> bool {
        CellCategory cc = cell_category(cell);
        switch (cc) {
        case CellCategory::Unknown: break;
        case CellCategory::Outside:
            if (new_cc == CellCategory::Inside) new_cc = CellCategory::Boundary;
            break;
        case CellCategory::Inside:
            if (new_cc == CellCategory::Outside) new_cc = CellCategory::Boundary;
            break;
        case CellCategory::Boundary: return false;
        }
        if (cc != new_cc) { set_cell_category(cell, new_cc); return true; }
        return false;
    };

    // Mark vertices on the input contour.
    for (const VD::edge_type &edge : vd.edges()) {
        const VD::vertex_type *v = edge.vertex0();
        if (v != nullptr) {
            MaVec2d pv{v->x(), v->y()};
            if (on_site(lines, *edge.cell(), pv))
                annotate_vertex(v, VertexCategory::OnContour);
        }
    }
    // Secondary edges: one side is on the source contour.
    for (const VD::edge_type &edge : vd.edges()) {
        if (edge.is_secondary() && edge.vertex0() != nullptr) {
            const MaPoint &pt_on_contour = edge.cell()->contains_point()
                ? contour_point(*edge.cell(), lines)
                : contour_point(*edge.twin()->cell(), lines);
            if (edge.vertex1() == nullptr) {
                annotate_vertex(edge.vertex0(), VertexCategory::OnContour);
            } else {
                const VD::vertex_type *v0 = edge.vertex0();
                if (vertex_equal_to_point(v0, pt_on_contour))
                    annotate_vertex(v0, VertexCategory::OnContour);
                else
                    annotate_vertex(edge.vertex1(), VertexCategory::OnContour);
            }
        }
    }
    // Infinite edges are outside; classify finite edges via segment side tests.
    for (const VD::edge_type &edge : vd.edges())
        if (edge.vertex1() == nullptr) {
            const VD::cell_type *cell  = edge.cell();
            const VD::cell_type *cell2 = edge.twin()->cell();
            annotate_edge(&edge, EdgeCategory::PointsOutside);
            annotate_edge(edge.twin(), edge.is_secondary() ? EdgeCategory::PointsToContour : EdgeCategory::PointsOutside);
            annotate_vertex(edge.vertex0(), edge.is_secondary() ? VertexCategory::OnContour : VertexCategory::Outside);
            if (cell->contains_segment())
                std::swap(cell, cell2);
            annotate_cell(cell, CellCategory::Outside);
            annotate_cell(cell2, cell2->contains_point() ? CellCategory::Outside : CellCategory::Boundary);
        } else if (edge.vertex0() != nullptr) {
            const VD::cell_type *cell = edge.cell();
            const MaLine        *line = cell->contains_segment() ? &lines[cell->source_index()] : nullptr;
            if (line == nullptr) {
                cell = edge.twin()->cell();
                line = cell->contains_segment() ? &lines[cell->source_index()] : nullptr;
            }
            if (line) {
                const VD::vertex_type *v1    = edge.vertex1();
                const VD::cell_type   *cell2 = (cell == edge.cell()) ? edge.twin()->cell() : edge.cell();
                VertexCategory v0_category = vertex_category(edge.vertex0());
                VertexCategory v1_category = vertex_category(edge.vertex1());
                bool on_contour = v0_category == VertexCategory::OnContour || v1_category == VertexCategory::OnContour;
                if (on_contour && v1_category == VertexCategory::OnContour) {
                    annotate_edge(&edge, EdgeCategory::PointsToContour);
                } else {
                    const double l0x = double(line->a.x), l0y = double(line->a.y);
                    const double lvx = double(line->b.x - line->a.x), lvy = double(line->b.y - line->a.y);
                    double side = cross2d(v1->x() - l0x, v1->y() - l0y, lvx, lvy);
                    auto vc = side > 0. ? VertexCategory::Outside : VertexCategory::Inside;
                    annotate_vertex(v1, vc);
                    auto ec = vc == VertexCategory::Outside ? EdgeCategory::PointsOutside : EdgeCategory::PointsInside;
                    annotate_edge(&edge, ec);
                    annotate_vertex(edge.vertex0(), on_contour ? VertexCategory::OnContour : vc);
                    annotate_edge(edge.twin(), on_contour ? EdgeCategory::PointsToContour : ec);
                    annotate_cell(cell, on_contour ? CellCategory::Boundary :
                        (vc == VertexCategory::Outside ? CellCategory::Outside : CellCategory::Inside));
                    annotate_cell(cell2, (on_contour && cell2->contains_segment()) ? CellCategory::Boundary :
                        (vc == VertexCategory::Outside ? CellCategory::Outside : CellCategory::Inside));
                }
            }
        }

    // Expansion round for Point-Point edges.
    std::vector<const VD::cell_type*> cell_queue;
    for (const VD::edge_type &edge : vd.edges()) {
        if (edge_category(edge) == EdgeCategory::Unknown) {
            const VD::cell_type &cell  = *edge.cell();
            const VD::cell_type &cell2 = *edge.twin()->cell();
            CellCategory cc  = cell_category(cell);
            CellCategory cc2 = cell_category(cell2);
            CellCategory cc_new = cc;
            if (cc_new == CellCategory::Unknown)
                cc_new = cc2;
            if (cc_new == CellCategory::Unknown) {
                VertexCategory vc = vertex_category(edge.vertex0());
                if (vc != VertexCategory::Unknown)
                    cc_new = (vc == VertexCategory::Outside) ? CellCategory::Outside : CellCategory::Inside;
            }
            if (cc_new != CellCategory::Unknown) {
                VertexCategory vc = (cc_new == CellCategory::Outside) ? VertexCategory::Outside : VertexCategory::Inside;
                annotate_vertex(edge.vertex0(), vc);
                annotate_vertex(edge.vertex1(), vc);
                auto ec_new = (cc_new == CellCategory::Outside) ? EdgeCategory::PointsOutside : EdgeCategory::PointsInside;
                annotate_edge(&edge, ec_new);
                annotate_edge(edge.twin(), ec_new);
                if (cc != cc_new) { annotate_cell(&cell, cc_new); cell_queue.emplace_back(&cell); }
                if (cc2 != cc_new) { annotate_cell(&cell2, cc_new); cell_queue.emplace_back(&cell2); }
            }
        }
    }
    // Final seed fill.
    while (!cell_queue.empty()) {
        const VD::cell_type *cell = cell_queue.back();
        const CellCategory   cc   = cell_category(cell);
        cell_queue.pop_back();
        const VD::edge_type *first_edge = cell->incident_edge();
        const VD::edge_type *edge       = first_edge;
        const auto ec_new = (cc == CellCategory::Outside) ? EdgeCategory::PointsOutside : EdgeCategory::PointsInside;
        do {
            if (edge_category(edge) == EdgeCategory::Unknown) {
                set_edge_category(const_cast<VD::edge_type*>(edge), ec_new);
                set_edge_category(const_cast<VD::edge_type*>(edge->twin()), ec_new);
                const VD::cell_type *cell2 = edge->twin()->cell();
                CellCategory cc2 = cell_category(cell2);
                if (cc2 != cc) {
                    set_cell_category(cell2, cc);
                    cell_queue.emplace_back(cell2);
                }
            }
            edge = edge->next();
        } while (edge != first_edge);
    }
}

// ---------------------------------------------------------------------------
// MedialAxis — verbatim adaptation of Geometry/MedialAxis.cpp:445-690 +
// MedialAxis.hpp edge_data. Widths are SCALED doubles (VD coordinate space).
// ---------------------------------------------------------------------------
struct MaThickPolyline {
    std::vector<MaPoint> points;
    std::vector<double>  width;
    bool endpoint_first = false, endpoint_second = false;
    void clear() { points.clear(); width.clear(); endpoint_first = endpoint_second = false; }
};

class MedialAxis {
public:
    MedialAxis(double min_width, double max_width, const std::vector<MaLine> &lines)
        : m_lines(lines), m_min_width(min_width), m_max_width(max_width) {}

    void build(std::vector<MaThickPolyline> *polylines)
    {
        boost::polygon::construct_voronoi(m_lines.begin(), m_lines.end(), &m_vd);
        annotate_inside_outside(m_vd, m_lines);

        m_edge_data.assign(m_vd.edges().size() / 2, EdgeData{});
        for (auto edge = m_vd.edges().begin(); edge != m_vd.edges().end(); edge += 2)
            if (edge->is_primary() && edge->is_finite() &&
                (vertex_category(edge->vertex0()) == VertexCategory::Inside ||
                 vertex_category(edge->vertex1()) == VertexCategory::Inside) &&
                this->validate_edge(&*edge))
                this->edge_data(*edge).first.active = true;

        MaThickPolyline reverse_polyline;
        for (auto seed_edge = m_vd.edges().begin(); seed_edge != m_vd.edges().end(); seed_edge += 2)
            if (EdgeData &seed_edge_data = this->edge_data(*seed_edge).first; seed_edge_data.active) {
                seed_edge_data.active = false;

                MaThickPolyline polyline;
                polyline.points.emplace_back(seed_edge->vertex0()->x(), seed_edge->vertex0()->y());
                polyline.points.emplace_back(seed_edge->vertex1()->x(), seed_edge->vertex1()->y());
                polyline.width.emplace_back(seed_edge_data.width_start);
                polyline.width.emplace_back(seed_edge_data.width_end);
                this->process_edge_neighbors(&*seed_edge, &polyline);

                reverse_polyline.clear();
                this->process_edge_neighbors(seed_edge->twin(), &reverse_polyline);
                polyline.points.insert(polyline.points.begin(), reverse_polyline.points.rbegin(), reverse_polyline.points.rend());
                polyline.width.insert(polyline.width.begin(), reverse_polyline.width.rbegin(), reverse_polyline.width.rend());
                polyline.endpoint_first = reverse_polyline.endpoint_second;

                if (!polyline.points.empty() &&
                    polyline.points.front().x == polyline.points.back().x &&
                    polyline.points.front().y == polyline.points.back().y) {
                    polyline.endpoint_first = false;
                    polyline.endpoint_second = false;
                }
                polylines->emplace_back(std::move(polyline));
            }
    }

private:
    struct EdgeData {
        bool   active      = false;
        double width_start = 0.;
        double width_end   = 0.;
    };

    std::pair<EdgeData&, bool> edge_data(const VD::edge_type &edge) {
        size_t edge_id = &edge - &m_vd.edges().front();
        return { m_edge_data[edge_id / 2], (edge_id & 1) != 0 };
    }

    void process_edge_neighbors(const VD::edge_type *edge, MaThickPolyline *polyline)
    {
        for (;;) {
            const VD::edge_type *twin = edge->twin();
            size_t               num_neighbors  = 0;
            const VD::edge_type *first_neighbor = nullptr;
            for (const VD::edge_type *neighbor = twin->rot_next(); neighbor != twin; neighbor = neighbor->rot_next())
                if (this->edge_data(*neighbor).first.active) {
                    if (num_neighbors == 0)
                        first_neighbor = neighbor;
                    ++num_neighbors;
                }
            if (num_neighbors == 1) {
                if (std::pair<EdgeData&, bool> neighbor_data = this->edge_data(*first_neighbor);
                    neighbor_data.first.active) {
                    neighbor_data.first.active = false;
                    polyline->points.emplace_back(first_neighbor->vertex1()->x(), first_neighbor->vertex1()->y());
                    if (neighbor_data.second) {
                        polyline->width.push_back(neighbor_data.first.width_end);
                        polyline->width.push_back(neighbor_data.first.width_start);
                    } else {
                        polyline->width.push_back(neighbor_data.first.width_start);
                        polyline->width.push_back(neighbor_data.first.width_end);
                    }
                    edge = first_neighbor;
                    continue;
                }
            } else if (num_neighbors == 0) {
                polyline->endpoint_second = true;
            }
            break;
        }
    }

    bool validate_edge(const VD::edge_type *edge)
    {
        auto retrieve_segment = [this](const VD::cell_type *cell) -> const MaLine& { return m_lines[cell->source_index()]; };
        auto retrieve_endpoint = [retrieve_segment](const VD::cell_type *cell) -> const MaPoint& {
            const MaLine &line = retrieve_segment(cell);
            return cell->source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT ? line.a : line.b;
        };

        // Native overflow guard is inside #ifndef CLIPPERLIB_INT32, and
        // BambuStudio clipper.hpp #defines CLIPPERLIB_INT32 unconditionally
        // (clipper.hpp:83) — so the guard is compiled OUT in native. Omit it.

        const MaLine line(MaPoint(edge->vertex0()->x(), edge->vertex0()->y()),
                          MaPoint(edge->vertex1()->x(), edge->vertex1()->y()));

        const VD::cell_type *cell_l = edge->cell();
        const VD::cell_type *cell_r = edge->twin()->cell();
        const MaLine &segment_l = retrieve_segment(cell_l);
        const MaLine &segment_r = retrieve_segment(cell_r);

        double w0 = cell_r->contains_segment()
            ? segment_r.distance_to(line.a) * 2
            : (retrieve_endpoint(cell_r) - line.a).norm() * 2;
        double w1 = cell_l->contains_segment()
            ? segment_l.distance_to(line.b) * 2
            : (retrieve_endpoint(cell_l) - line.b).norm() * 2;

        // SCALED_EPSILON = scale_(EPSILON) = 1e-4 / 1e-5 = 10 (libslic3r.h:52,58,84).
        static constexpr double kScaledEps = 10.0;

        if (cell_l->contains_segment() && cell_r->contains_segment()) {
            double angle = std::fabs(segment_r.orientation() - segment_l.orientation());
            if (angle > M_PI)
                angle = 2. * M_PI - angle;
            if (M_PI - angle > M_PI / 8.) {
                const double dx = double(line.b.x - line.a.x), dy = double(line.b.y - line.a.y);
                const double line_length = std::sqrt(dx * dx + dy * dy);
                if (w0 < kScaledEps || w1 < kScaledEps || line_length >= m_min_width)
                    return false;
            }
        } else {
            if (w0 < kScaledEps || w1 < kScaledEps)
                return false;
        }

        if ((w0 >= m_min_width || w1 >= m_min_width) &&
            (w0 <= m_max_width || w1 <= m_max_width)) {
            std::pair<EdgeData&, bool> ed = this->edge_data(*edge);
            if (ed.second)
                std::swap(w0, w1);
            ed.first.width_start = w0;
            ed.first.width_end   = w1;
            return true;
        }
        return false;
    }

    VD                        m_vd;
    const std::vector<MaLine> &m_lines;
    double                    m_min_width;
    double                    m_max_width;
    std::vector<EdgeData>     m_edge_data;
};

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
