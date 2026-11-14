// Test-only exports: single predicates and functors of boost::polygon's Voronoi builder
// (detail/voronoi_predicates.hpp), so a port can be compared with them one call at a time.
// Nothing in the engine calls these.
//
// The distance functions of `distance_predicate` are private; this translation unit (and
// only this one) reads the boost headers with `private` spelt `public`. The standard headers
// are included first, untouched.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <list>
#include <map>
#include <queue>
#include <utility>
#include <vector>

#define private public
#include "boost/polygon/voronoi_builder.hpp"
#undef private

namespace {
using namespace boost::polygon;
using namespace boost::polygon::detail;

typedef voronoi_predicates<voronoi_ctype_traits<int32> > VP;
typedef site_event<int32> site_t;
typedef circle_event<double> circle_t;
typedef point_2d<int32> point_t;

// A site: [x0, y0, x1, y1, sorted_index, is_inverse]. The points are given as the site holds
// them (after `inverse()`, if it is inverse).
site_t make_site(const int32_t *s) {
    site_t r = s[5] ? site_t(s[2], s[3], s[0], s[1]) : site_t(s[0], s[1], s[2], s[3]);
    r.sorted_index(size_t(s[4]));
    if (s[5])
        r.inverse();
    return r;
}

void put(const circle_t &c, double *out) {
    out[0] = c.x();
    out[1] = c.y();
    out[2] = c.lower_x();
}
} // namespace

extern "C" {

// circle_formation_predicate::operator(). out = [x, y, lower_x] (zeros where nothing was computed).
int32_t bv_test_circle_formation(const int32_t *s1, const int32_t *s2, const int32_t *s3, double *out) {
    VP::circle_formation_predicate<site_t, circle_t> pred;
    circle_t c(0.0, 0.0, 0.0);
    bool r = pred(make_site(s1), make_site(s2), make_site(s3), c);
    put(c, out);
    return r ? 1 : 0;
}

// lazy_circle_formation_functor: kind 0 = ppp, 1 = pps, 2 = pss, 3 = sss; `index` is the
// segment_index of pps and the point_index of pss.
void bv_test_lazy_circle(int32_t kind, int32_t index, const int32_t *s1, const int32_t *s2, const int32_t *s3,
                         double *out) {
    VP::lazy_circle_formation_functor<site_t, circle_t> f;
    circle_t c(0.0, 0.0, 0.0);
    site_t a = make_site(s1), b = make_site(s2), d = make_site(s3);
    switch (kind) {
    case 0: f.ppp(a, b, d, c); break;
    case 1: f.pps(a, b, d, index, c); break;
    case 2: f.pss(a, b, d, index, c); break;
    default: f.sss(a, b, d, c); break;
    }
    put(c, out);
}

// mp_circle_formation_functor with every coordinate recomputed.
void bv_test_mp_circle(int32_t kind, int32_t index, const int32_t *s1, const int32_t *s2, const int32_t *s3,
                       double *out) {
    VP::mp_circle_formation_functor<site_t, circle_t> f;
    circle_t c(0.0, 0.0, 0.0);
    site_t a = make_site(s1), b = make_site(s2), d = make_site(s3);
    switch (kind) {
    case 0: f.ppp(a, b, d, c); break;
    case 1: f.pps(a, b, d, index, c); break;
    case 2: f.pss(a, b, d, index, c); break;
    default: f.sss(a, b, d, c); break;
    }
    put(c, out);
}

// distance_predicate: the return value has bit 0 = operator()(left, right, point),
// bits 1-2 = fast_ps(left, right, point, reverse) + 2 (when left is a point and right a
// segment, else 0). out[0] = find_distance_to_point_arc(left, point),
// out[1] = find_distance_to_segment_arc(right, point).
int32_t bv_test_distance(const int32_t *left, const int32_t *right, int32_t px, int32_t py, int32_t reverse,
                         double *out) {
    VP::distance_predicate<site_t> d;
    site_t l = make_site(left), r = make_site(right);
    point_t p(px, py);
    int32_t ret = d(l, r, p) ? 1 : 0;
    if (!l.is_segment() && r.is_segment())
        ret |= (int32_t(d.fast_ps(l, r, p, reverse != 0)) + 2) << 1;
    out[0] = d.find_distance_to_point_arc(l, p);
    out[1] = d.find_distance_to_segment_arc(r, p);
    return ret;
}

// node_comparison_predicate on the nodes (l1, r1) and (l2, r2).
int32_t bv_test_node_comparison(const int32_t *l1, const int32_t *r1, const int32_t *l2, const int32_t *r2) {
    typedef beach_line_node_key<site_t> key_t;
    VP::node_comparison_predicate<key_t> cmp;
    return cmp(key_t(make_site(l1), make_site(r1)), key_t(make_site(l2), make_site(r2))) ? 1 : 0;
}

// event_comparison_predicate on two sites.
int32_t bv_test_event_comparison(const int32_t *s1, const int32_t *s2) {
    VP::event_comparison_predicate<site_t, circle_t> cmp;
    return cmp(make_site(s1), make_site(s2)) ? 1 : 0;
}

} // extern "C"
