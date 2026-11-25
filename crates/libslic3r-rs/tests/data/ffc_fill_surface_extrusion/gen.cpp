// Native vectors for tests/ffc_fill_surface_extrusion_native_parity.rs.
// Runs the pin's FillFloatingConcentric (the override file
// libslic3r/bambustudio/libslic3r/Fill/FillFloatingConcentric.cpp:880-1000:
// fill_surface_arachne_floating and fill_surface_extrusion's per-line
// floating_thick_polyline_to_extrusion_paths loop) on fill inputs recorded by the port's FFCDUMP
// probe (fill/fill_floating_concentric.rs) from a real slice, on one thread, and prints each input
// back followed by "EXP <n>" and the pin's paths ("Q <npts> <x y>... <width> <mm3_per_mm> <height>
// <floating flag>", doubles as IEEE-754 bits in hex) and "END".
// Usage: gen <FFCDUMP file> > vectors.txt
#include "libslic3r/Fill/FillFloatingConcentric.cpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Surface.hpp"
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <tbb/global_control.h>
using namespace Slic3r;

static double dbits(const std::string &h) { uint64_t u = std::stoull(h, nullptr, 16); double d; std::memcpy(&d, &u, 8); return d; }
static unsigned long long bits(double d) { unsigned long long u; std::memcpy(&u, &d, 8); return u; }

struct FFC : public FillFloatingConcentric {
    void set_cfg(const PrintConfig *pc, const PrintObjectConfig *poc) { print_config = pc; print_object_config = poc; }
    FloatingThickPolylines run(const Surface *s, const FillParams &p) { return fill_surface_arachne_floating(s, p); }
};

static Points read_pts(std::istringstream &is)
{
    size_t n; is >> n; Points pts; pts.reserve(n);
    for (size_t i = 0; i < n; ++i) { long long x, y; is >> x >> y; pts.emplace_back(coord_t(x), coord_t(y)); }
    return pts;
}

static ExPolygons read_expolys(std::ifstream &in, size_t n, std::string &echo)
{
    ExPolygons out;
    std::string line;
    for (size_t i = 0; i < n; ++i) {
        std::getline(in, line);
        echo += line + "\n";
        std::istringstream is(line);
        std::string tag; is >> tag;
        ExPolygon e;
        e.contour.points = read_pts(is);
        size_t nh; is >> nh;
        for (size_t h = 0; h < nh; ++h) { Polygon hp; hp.points = read_pts(is); e.holes.push_back(hp); }
        out.push_back(e);
    }
    return out;
}

int main(int argc, char **argv)
{
    tbb::global_control c(tbb::global_control::max_allowed_parallelism, 1);
    std::ifstream in(argv[1]);
    std::string line;
    int call = 0;
    while (std::getline(in, line)) {
        if (line.compare(0, 5, "CALL ") != 0) continue;
        std::string echo = line + "\n";
        std::istringstream is(line.substr(5));
        std::string hw, hh, hn, hs, hlh, hsp, hnc; int bridge, detect; long long clip;
        is >> hw >> hh >> hn >> hs >> bridge >> hlh >> hsp >> clip >> detect >> hnc;
        std::string tag; size_t n;
        std::getline(in, line); echo += line + "\n"; { std::istringstream e(line); e >> tag >> n; }
        ExPolygons expolys = read_expolys(in, n, echo);
        std::getline(in, line); echo += line + "\n"; { std::istringstream e(line); e >> tag >> n; }
        ExPolygons unsup = read_expolys(in, n, echo);
        std::getline(in, line); echo += line + "\n"; { std::istringstream e(line); e >> tag >> n; }
        Polygons sparse;
        for (size_t i = 0; i < n; ++i) { std::getline(in, line); echo += line + "\n"; std::istringstream e(line); e >> tag; Polygon p; p.points = read_pts(e); sparse.push_back(p); }

        PrintConfig pc;
        pc.nozzle_diameter.values = { dbits(hnc) };
        PrintObjectConfig poc;
        poc.detect_floating_vertical_shell.value = detect != 0;
        FFC f;
        f.set_cfg(&pc, &poc);
        f.spacing = dbits(hsp);
        f.loop_clipping = coord_t(clip);
        f.no_overlap_expolygons = expolys;
        f.lower_layer_unsupport_areas = unsup;
        f.lower_sparse_polys = sparse;
        FillParams params;
        params.flow = Flow(float(dbits(hw)), float(dbits(hh)), float(dbits(hn)));
        params.layer_height = dbits(hlh);
        params.extrusion_role = erFloatingVerticalShell;
        Surface surf(stInternalSolid, expolys.empty() ? ExPolygon() : expolys.front());
        FloatingThickPolylines floating_lines = f.run(&surf, params);
        ExtrusionPaths all;
        if (!floating_lines.empty()) {
            Flow new_flow = params.flow.with_spacing(f.spacing);
            const float tolerance = float(scale_(0.05));
            for (const auto &fl : floating_lines) {
                ExtrusionPaths paths = floating_thick_polyline_to_extrusion_paths(fl, params.extrusion_role, new_flow, tolerance);
                append(all, std::move(paths));
            }
        }
        fputs(echo.c_str(), stdout);
        printf("EXP %zu\n", all.size());
        for (const ExtrusionPath &p : all) {
            printf("Q %zu", p.polyline.points.size());
            for (const Point &q : p.polyline.points) printf(" %lld %lld", (long long) q.x(), (long long) q.y());
            printf(" %016llx %016llx %016llx %d\n", bits(double(p.width)), bits(p.mm3_per_mm), bits(double(p.height)),
                   int(p.get_customize_flag() == CustomizeFlag::cfFloatingVerticalShell));
        }
        printf("END\n");
        ++call;
    }
    return 0;
}
