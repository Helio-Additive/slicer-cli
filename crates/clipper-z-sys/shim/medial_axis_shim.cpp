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
    bool operator==(const MaPoint &o) const { return x == o.x && y == o.y; }
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
// ---------------------------------------------------------------------------
// ExPolygon::medial_axis post-processing — verbatim ExPolygon.cpp:263-371 +
// helpers (Polygon::intersection, point_projection, on_boundary; Line.hpp
// line_alg::intersection). All double arithmetic mirrors the Eigen
// expressions; double->coord_t casts TRUNCATE (Eigen cast<>), except
// point_projection's foot which is floor(v + 0.5) like native.
// ---------------------------------------------------------------------------

static const double kMaScaledEpsilon = 10.0;   // SCALED_EPSILON (libslic3r.h)
static const double kMaEpsilon       = 1e-4;   // EPSILON

static inline double ma_seg_length(const MaPoint &a, const MaPoint &b) {
    const double dx = double(b.x - a.x), dy = double(b.y - a.y);
    return std::sqrt(dx * dx + dy * dy);
}

static double ma_polyline_length(const MaThickPolyline &pl) {
    double len = 0;
    for (size_t i = 1; i < pl.points.size(); ++i)
        len += ma_seg_length(pl.points[i - 1], pl.points[i]);
    return len;
}

static void ma_polyline_reverse(MaThickPolyline &pl) {
    std::reverse(pl.points.begin(), pl.points.end());
    std::reverse(pl.width.begin(), pl.width.end());
    std::swap(pl.endpoint_first, pl.endpoint_second);
}

// line_alg::intersection (Line.hpp:123-150): doubles, cross2 denom with
// EPSILON guard, t1/t2 in [0,1], result = (l1.a + t1*v1).cast<coord_t>().
static bool ma_line_intersection(const MaPoint &l1a, const MaPoint &l1b,
                                 const MaPoint &l2a, const MaPoint &l2b,
                                 MaPoint *out)
{
    const double v1x = double(l1b.x - l1a.x), v1y = double(l1b.y - l1a.y);
    const double v2x = double(l2b.x - l2a.x), v2y = double(l2b.y - l2a.y);
    const double denom = v1x * v2y - v1y * v2x;
    if (std::fabs(denom) < kMaEpsilon)
        return false;
    const double v12x = double(l1a.x - l2a.x), v12y = double(l1a.y - l2a.y);
    const double nume_a = v2x * v12y - v2y * v12x;
    const double nume_b = v1x * v12y - v1y * v12x;
    const double t1 = nume_a / denom;
    const double t2 = nume_b / denom;
    if (t1 >= 0 && t1 <= 1.0 && t2 >= 0 && t2 <= 1.0) {
        out->x = coord_t(double(l1a.x) + t1 * v1x);
        out->y = coord_t(double(l1a.y) + t1 * v1y);
        return true;
    }
    return false;
}

// Polygon::intersection (Polygon.cpp:188-199): closing edge (front,back)
// FIRST, then consecutive edges; first hit wins.
static bool ma_polygon_intersection(const std::vector<MaPoint> &pts,
                                    const MaPoint &la, const MaPoint &lb,
                                    MaPoint *out)
{
    if (pts.size() < 2)
        return false;
    if (ma_line_intersection(pts.front(), pts.back(), la, lb, out))
        return true;
    for (size_t i = 1; i < pts.size(); ++i)
        if (ma_line_intersection(pts[i - 1], pts[i], la, lb, out))
            return true;
    return false;
}

// Polygon::point_projection (Polygon.cpp:310-345).
static MaPoint ma_point_projection(const std::vector<MaPoint> &pts, const MaPoint &point)
{
    MaPoint proj = point;
    double dmin = std::numeric_limits<double>::max();
    if (!pts.empty()) {
        for (size_t i = 0; i < pts.size(); ++i) {
            const MaPoint &pt0 = pts[i];
            const MaPoint &pt1 = pts[(i + 1 == pts.size()) ? 0 : i + 1];
            double d = ma_seg_length(pt0, point);
            if (d < dmin) { dmin = d; proj = pt0; }
            d = ma_seg_length(pt1, point);
            if (d < dmin) { dmin = d; proj = pt1; }
            const double v1x = double(pt1.x - pt0.x), v1y = double(pt1.y - pt0.y);
            const double div = v1x * v1x + v1y * v1y;
            if (div > 0.) {
                const double v2x = double(point.x - pt0.x), v2y = double(point.y - pt0.y);
                const double t = (v1x * v2x + v1y * v2y) / div;
                if (t > 0. && t < 1.) {
                    const MaPoint foot(coord_t(std::floor(double(pt0.x) + t * v1x + 0.5)),
                                       coord_t(std::floor(double(pt0.y) + t * v1y + 0.5)));
                    d = ma_seg_length(foot, point);
                    if (d < dmin) { dmin = d; proj = foot; }
                }
            }
        }
    }
    return proj;
}

// Polygon::on_boundary (Polygon.hpp:71-72).
static bool ma_polygon_on_boundary(const std::vector<MaPoint> &pts, const MaPoint &point, double eps)
{
    const MaPoint proj = ma_point_projection(pts, point);
    const double ex = double(proj.x - point.x), ey = double(proj.y - point.y);
    return ex * ex + ey * ey < eps * eps;
}

struct MaExPolygon {
    std::vector<MaPoint>              contour;
    std::vector<std::vector<MaPoint>> holes;

    // ExPolygon::on_boundary (ExPolygon.cpp:121-129).
    bool on_boundary(const MaPoint &point, double eps) const {
        if (ma_polygon_on_boundary(contour, point, eps))
            return true;
        for (const auto &hole : holes)
            if (ma_polygon_on_boundary(hole, point, eps))
                return true;
        return false;
    }

    // ExPolygon::lines (ExPolygon.cpp:433-441) via to_lines (Polygon.hpp):
    // per polygon: consecutive edges then closing (back,front); polygons with
    // <=2 points contribute nothing; contour first, then holes in order.
    std::vector<MaLine> lines() const {
        std::vector<MaLine> out;
        auto add = [&out](const std::vector<MaPoint> &pts) {
            if (pts.size() > 2) {
                for (size_t i = 1; i < pts.size(); ++i)
                    out.emplace_back(pts[i - 1], pts[i]);
                out.emplace_back(pts.back(), pts.front());
            }
        };
        add(contour);
        for (const auto &hole : holes)
            add(hole);
        return out;
    }
};

// ExPolygon::medial_axis (ExPolygon.cpp:263-371).
static void ma_expolygon_medial_axis(const MaExPolygon &expoly,
                                     double min_width, double max_width,
                                     std::vector<MaThickPolyline> *polylines)
{
    const std::vector<MaLine> lines = expoly.lines();
    MedialAxis ma(min_width, max_width, lines);
    std::vector<MaThickPolyline> pp;
    ma.build(&pp);

    double max_w = 0;
    for (auto it = pp.begin(); it != pp.end(); ++it)
        max_w = fmaxf(max_w, *std::max_element(it->width.begin(), it->width.end()));

    bool removed = false;
    for (size_t i = 0; i < pp.size(); ++i) {
        MaThickPolyline &polyline = pp[i];

        MaPoint new_front = polyline.points.front();
        MaPoint new_back  = polyline.points.back();
        if (polyline.endpoint_first && !expoly.on_boundary(new_front, kMaScaledEpsilon)) {
            double p1x = double(polyline.points.front().x), p1y = double(polyline.points.front().y);
            double p2x = double(polyline.points[1].x),      p2y = double(polyline.points[1].y);
            if (polyline.points.size() == 2) {
                p2x = (p1x + p2x) * 0.5;
                p2y = (p1y + p2y) * 0.5;
            }
            // p1 -= (p2 - p1).normalized() * max_width; Eigen normalized() is
            // v / sqrt(squaredNorm) (returns v unchanged when squaredNorm == 0).
            const double vx = p2x - p1x, vy = p2y - p1y;
            const double z = vx * vx + vy * vy;
            double nx = vx, ny = vy;
            if (z > 0.) { const double n = std::sqrt(z); nx = vx / n; ny = vy / n; }
            p1x -= nx * max_width;
            p1y -= ny * max_width;
            ma_polygon_intersection(expoly.contour,
                                    MaPoint(coord_t(p1x), coord_t(p1y)),
                                    MaPoint(coord_t(p2x), coord_t(p2y)), &new_front);
        }
        if (polyline.endpoint_second && !expoly.on_boundary(new_back, kMaScaledEpsilon)) {
            double p1x = double((polyline.points.end() - 2)->x), p1y = double((polyline.points.end() - 2)->y);
            double p2x = double(polyline.points.back().x),       p2y = double(polyline.points.back().y);
            if (polyline.points.size() == 2) {
                p1x = (p1x + p2x) * 0.5;
                p1y = (p1y + p2y) * 0.5;
            }
            const double vx = p2x - p1x, vy = p2y - p1y;
            const double z = vx * vx + vy * vy;
            double nx = vx, ny = vy;
            if (z > 0.) { const double n = std::sqrt(z); nx = vx / n; ny = vy / n; }
            p2x += nx * max_width;
            p2y += ny * max_width;
            ma_polygon_intersection(expoly.contour,
                                    MaPoint(coord_t(p1x), coord_t(p1y)),
                                    MaPoint(coord_t(p2x), coord_t(p2y)), &new_back);
        }
        polyline.points.front() = new_front;
        polyline.points.back()  = new_back;

        if ((polyline.endpoint_first || polyline.endpoint_second)
            && ma_polyline_length(polyline) < max_w * 2) {
            pp.erase(pp.begin() + i);
            --i;
            removed = true;
            continue;
        }
    }

    if (removed) {
        for (size_t i = 0; i < pp.size(); ++i) {
            MaThickPolyline &polyline = pp[i];
            if (polyline.endpoint_first && polyline.endpoint_second)
                continue;
            for (size_t j = i + 1; j < pp.size(); ++j) {
                MaThickPolyline &other = pp[j];
                if (polyline.points.back() == other.points.back()) {
                    ma_polyline_reverse(other);
                } else if (polyline.points.front() == other.points.back()) {
                    ma_polyline_reverse(polyline);
                    ma_polyline_reverse(other);
                } else if (polyline.points.front() == other.points.front()) {
                    ma_polyline_reverse(polyline);
                } else if (!(polyline.points.back() == other.points.front())) {
                    continue;
                }
                polyline.points.insert(polyline.points.end(), other.points.begin() + 1, other.points.end());
                polyline.width.insert(polyline.width.end(), other.width.begin(), other.width.end());
                polyline.endpoint_second = other.endpoint_second;
                pp.erase(pp.begin() + j);
                j = i; // restart search from i+1
            }
        }
    }

    polylines->insert(polylines->end(), pp.begin(), pp.end());
}

// ---------------------------------------------------------------------------
// FFI
// ---------------------------------------------------------------------------
struct MaBuildResult {
    int64_t *coords;     // 2 * total_points (x,y per point)
    double  *widths;     // total_widths (per polyline: 2*(points-1))
    int32_t *pl_sizes;   // num_polylines point counts
    uint8_t *endpoints;  // 2 * num_polylines (first, second flags)
    int32_t  num_polylines;
    int32_t  total_points;
    int32_t  total_widths;
};

extern "C" MaBuildResult ma_build(const int64_t *contour_xy, int32_t contour_n,
                                  const int64_t *holes_xy, const int32_t *hole_lens, int32_t hole_num,
                                  double min_width, double max_width)
{
    MaExPolygon expoly;
    expoly.contour.reserve(size_t(contour_n));
    for (int32_t i = 0; i < contour_n; ++i)
        expoly.contour.emplace_back(coord_t(contour_xy[2 * i]), coord_t(contour_xy[2 * i + 1]));
    size_t off = 0;
    expoly.holes.resize(size_t(hole_num));
    for (int32_t h = 0; h < hole_num; ++h) {
        auto &hole = expoly.holes[size_t(h)];
        hole.reserve(size_t(hole_lens[h]));
        for (int32_t i = 0; i < hole_lens[h]; ++i, ++off)
            hole.emplace_back(coord_t(holes_xy[2 * off]), coord_t(holes_xy[2 * off + 1]));
    }

    std::vector<MaThickPolyline> pls;
    ma_expolygon_medial_axis(expoly, min_width, max_width, &pls);

    MaBuildResult res{};
    res.num_polylines = int32_t(pls.size());
    size_t tp = 0, tw = 0;
    for (const auto &pl : pls) { tp += pl.points.size(); tw += pl.width.size(); }
    res.total_points = int32_t(tp);
    res.total_widths = int32_t(tw);
    res.coords    = static_cast<int64_t*>(std::malloc(sizeof(int64_t) * 2 * (tp ? tp : 1)));
    res.widths    = static_cast<double*>(std::malloc(sizeof(double) * (tw ? tw : 1)));
    res.pl_sizes  = static_cast<int32_t*>(std::malloc(sizeof(int32_t) * (pls.empty() ? 1 : pls.size())));
    res.endpoints = static_cast<uint8_t*>(std::malloc(sizeof(uint8_t) * 2 * (pls.empty() ? 1 : pls.size())));
    size_t ci = 0, wi = 0;
    for (size_t k = 0; k < pls.size(); ++k) {
        const auto &pl = pls[k];
        res.pl_sizes[k] = int32_t(pl.points.size());
        res.endpoints[2 * k]     = pl.endpoint_first ? 1 : 0;
        res.endpoints[2 * k + 1] = pl.endpoint_second ? 1 : 0;
        for (const auto &pt : pl.points) { res.coords[ci++] = pt.x; res.coords[ci++] = pt.y; }
        for (double w : pl.width) res.widths[wi++] = w;
    }
    return res;
}

extern "C" void ma_free(MaBuildResult res)
{
    std::free(res.coords);
    std::free(res.widths);
    std::free(res.pl_sizes);
    std::free(res.endpoints);
}

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
