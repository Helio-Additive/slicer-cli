// Native vectors for the `determine_bridging_angle` lambda of PrintObject::bridge_over_infill
// (PrintObject.cpp:3081-3160). The lambda is local to the function, so its body is copied here verbatim
// and compiled with the engine's own flags. Inputs: deterministic bridged areas (star polygons, some with a
// hole) and anchors (a set of parallel infill lines at a random angle plus random lines), over six infill
// patterns. Output: `in ba <pattern> <infill_direction> <polys> <nlines> <lines>` then `ba <angle>`, doubles
// as the hex of their bits.
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <map>
#include <vector>

#include "libslic3r/AABBTreeLines.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/PrintConfig.hpp"

using namespace Slic3r;

// PrintObject.cpp:3081-3160, verbatim.
static auto determine_bridging_angle = [](const Polygons &bridged_area, const Lines &anchors, InfillPattern dominant_pattern, double infill_direction) {
        AABBTreeLines::LinesDistancer<Line> lines_tree(anchors);

        // Check it the infill that require a fixed infill angle.
        switch (dominant_pattern) {
        case ip3DHoneycomb:
        case ipCrossHatch:
            return (infill_direction + 45.0) * 2.0 * M_PI / 360.;
        default: break;
        }

        std::map<double, int> counted_directions;
        for (const Polygon &p : bridged_area) {
            double acc_distance = 0;
            for (int point_idx = 0; point_idx < int(p.points.size()) - 1; ++point_idx) {
                Vec2d  start        = p.points[point_idx].cast<double>();
                Vec2d  next         = p.points[point_idx + 1].cast<double>();
                Vec2d  v            = next - start; // vector from next to current
                double dist_to_next = v.norm();
                acc_distance += dist_to_next;
                if (acc_distance > scaled(2.0)) {
                    acc_distance = 0.0;
                    v.normalize();
                    int   lines_count = int(std::ceil(dist_to_next / scaled(2.0)));
                    float step_size   = dist_to_next / lines_count;
                    for (int i = 0; i < lines_count; ++i) {
                        Point a                   = (start + v * (i * step_size)).cast<coord_t>();
                        auto [distance, index, p] = lines_tree.distance_from_lines_extra<false>(a);
                        double angle = lines_tree.get_line(index).orientation();
                        if (angle > PI) {
                            angle -= PI;
                        }
                        angle += PI * 0.5;
                        counted_directions[angle]++;
                    }
                }
            }
        }

        std::pair<double, int> best_dir{0, 0};
        // sliding window accumulation
        for (const auto &dir : counted_directions) {
            int    score_acc          = 0;
            double dir_acc            = 0;
            double window_start_angle = dir.first - PI * 0.1;
            double window_end_angle   = dir.first + PI * 0.1;
            for (auto dirs_window = counted_directions.lower_bound(window_start_angle);
                dirs_window != counted_directions.upper_bound(window_end_angle); dirs_window++) {
                dir_acc += dirs_window->first * dirs_window->second;
                score_acc += dirs_window->second;
            }
            // current span of directions is 0.5 PI to 1.5 PI (due to the aproach.). Edge values should also account for the
            //  opposite direction.
            if (window_start_angle < 0.5 * PI) {
                for (auto dirs_window = counted_directions.lower_bound(1.5 * PI - (0.5 * PI - window_start_angle));
                    dirs_window != counted_directions.end(); dirs_window++) {
                    dir_acc += dirs_window->first * dirs_window->second;
                    score_acc += dirs_window->second;
                }
            }
            if (window_start_angle > 1.5 * PI) {
                for (auto dirs_window = counted_directions.begin();
                    dirs_window != counted_directions.upper_bound(window_start_angle - 1.5 * PI); dirs_window++) {
                    dir_acc += dirs_window->first * dirs_window->second;
                    score_acc += dirs_window->second;
                }
            }

            if (score_acc > best_dir.second) {
                best_dir = {dir_acc / score_acc, score_acc};
            }
        }
        double bridging_angle = best_dir.first;
        if (bridging_angle == 0) {
            bridging_angle = 0.001;
        }
        switch (dominant_pattern) {
        case ipHilbertCurve: bridging_angle += 0.25 * PI; break;
        case ipOctagramSpiral: bridging_angle += (1.0 / 16.0) * PI; break;
        default: break;
        }

        return bridging_angle;
        };

static uint64_t g_seed = 0x2545F4914F6CDD1DULL;
static uint32_t rnd()
{
    g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL;
    return uint32_t(g_seed >> 33);
}
// [lo, hi]
static int64_t rint_(int64_t lo, int64_t hi) { return lo + int64_t(rnd() % uint32_t(hi - lo + 1)); }
static double  runit() { return double(rnd() % 1000000) / 1000000.; }

static uint64_t bits(double v)
{
    uint64_t u;
    memcpy(&u, &v, 8);
    return u;
}

static InfillPattern pattern_of(int k)
{
    switch (k) {
    case 0: return ipRectilinear;
    case 1: return ipLine;
    case 2: return ipHilbertCurve;
    case 3: return ipOctagramSpiral;
    case 4: return ipCrossHatch;
    default: return ip3DHoneycomb;
    }
}

static Polygon star(int64_t cx, int64_t cy, int64_t r0, int64_t r1, int n, double phase, bool cw)
{
    Polygon p;
    for (int i = 0; i < n; ++i) {
        double a = phase + 2. * M_PI * double(i) / double(n);
        int64_t r = rint_(r0, r1);
        p.points.emplace_back(cx + int64_t(double(r) * std::cos(a)), cy + int64_t(double(r) * std::sin(a)));
    }
    if (cw) std::reverse(p.points.begin(), p.points.end());
    return p;
}

// `gen <file>`: runs the lambda on the "in ba" lines of <file> (as the port's probe BA_DUMP prints them)
// instead of the synthetic cases, and prints them with their results.
static int run_file(const char *path)
{
    FILE *f = std::fopen(path, "r");
    if (f == nullptr) return 1;
    static char buf[1 << 24];
    while (std::fgets(buf, sizeof(buf), f) != nullptr) {
        if (std::strncmp(buf, "in ba ", 6) != 0) continue;
        char *s = buf + 6;
        auto  next_i = [&s]() { return (long long) std::strtoll(s, &s, 10); };
        int   k      = int(next_i());
        uint64_t u   = std::strtoull(s, &s, 16);
        double dir;
        std::memcpy(&dir, &u, 8);
        Polygons area; area.resize(size_t(next_i()));
        for (Polygon &p : area) {
            p.points.resize(size_t(next_i()));
            for (Point &q : p.points) { q.x() = next_i(); q.y() = next_i(); }
        }
        Lines anchors; anchors.resize(size_t(next_i()));
        for (Line &l : anchors) { l.a.x() = next_i(); l.a.y() = next_i(); l.b.x() = next_i(); l.b.y() = next_i(); }
        std::fputs(buf, stdout);
        double r = determine_bridging_angle(area, anchors, pattern_of(k), dir);
        printf("ba %016" PRIx64 "\n", bits(r));
    }
    std::fclose(f);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 2) return run_file(argv[1]);
    for (int c = 0; c < 400; ++c) {
        int     k   = c % 6;
        double  dir = double(rint_(0, 359));
        int64_t cx  = rint_(-5000000, 5000000), cy = rint_(-5000000, 5000000);
        int64_t r1  = rint_(300000, 3000000);
        int64_t r0  = r1 / 2 + rint_(0, r1 / 4);
        Polygons area;
        int      nb = 1 + int(rnd() % 2);
        for (int b = 0; b < nb; ++b) {
            int64_t ox = b == 0 ? 0 : rint_(-2 * r1, 2 * r1), oy = b == 0 ? 0 : rint_(-2 * r1, 2 * r1);
            int     n  = c % 5 == 0 ? 4 : int(rint_(3, 40));
            area.push_back(star(cx + ox, cy + oy, r0, r1, n, runit() * 6.283, false));
            if (rnd() % 3 == 0) area.push_back(star(cx + ox, cy + oy, r0 / 4, r0 / 3, int(rint_(3, 12)), runit(), true));
        }
        // anchors: parallel lines at a random angle across the box, plus random lines
        Lines   anchors;
        double  la   = runit() * M_PI;
        int64_t sp   = rint_(200000, 1500000);
        int64_t ext  = 3 * r1;
        Vec2d   d(std::cos(la), std::sin(la)), nrm(-std::sin(la), std::cos(la));
        int     nl   = int(2 * ext / sp) + 1;
        for (int i = 0; i < nl; ++i) {
            double off = double(-ext + i * sp);
            Vec2d  o   = Vec2d(double(cx), double(cy)) + nrm * off;
            if (rnd() % 4 == 0) continue;
            anchors.emplace_back(Point(int64_t(o.x() - d.x() * double(ext)), int64_t(o.y() - d.y() * double(ext))),
                                 Point(int64_t(o.x() + d.x() * double(ext)), int64_t(o.y() + d.y() * double(ext))));
        }
        int nr = int(rnd() % 6);
        for (int i = 0; i < nr; ++i)
            anchors.emplace_back(Point(cx + rint_(-ext, ext), cy + rint_(-ext, ext)), Point(cx + rint_(-ext, ext), cy + rint_(-ext, ext)));
        if (anchors.empty())
            anchors.emplace_back(Point(cx - ext, cy), Point(cx + ext, cy));

        printf("in ba %d %016" PRIx64 " %zu", k, bits(dir), area.size());
        for (const Polygon &p : area) {
            printf(" %zu", p.points.size());
            for (const Point &q : p.points) printf(" %" PRId64 " %" PRId64, int64_t(q.x()), int64_t(q.y()));
        }
        printf(" %zu", anchors.size());
        for (const Line &l : anchors)
            printf(" %" PRId64 " %" PRId64 " %" PRId64 " %" PRId64, int64_t(l.a.x()), int64_t(l.a.y()), int64_t(l.b.x()), int64_t(l.b.y()));
        printf("\n");
        double r = determine_bridging_angle(area, anchors, pattern_of(k), dir);
        printf("ba %016" PRIx64 "\n", bits(r));
    }
    return 0;
}
