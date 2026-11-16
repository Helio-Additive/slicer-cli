// Reference vectors for crates/libslic3r-rs/src/flow.rs, taken from the C++ at the pin.
//
// The program links the engine's libslic3r (Flow.cpp as slicer_cli compiles it: clang, -O3,
// -ffp-contract=on, the default on arm64 macOS) and calls the Flow members on generated inputs.
// Build it against a configured Bambu Studio engine build tree with the compile and link lines
// CMake uses for a libslic3r test program, then run it:
//   ./flow_gen vectors.txt && gzip -9n vectors.txt   (into crates/libslic3r-rs/tests/data/flow/)
//
// Floats are written as the hex of their bits (8 digits), doubles as the hex of theirs (16).
// One case per line:
//   AUTO role nd -> w                          Flow::auto_extrusion_width
//   RRS w h -> out | ERR                       Flow::rounded_rectangle_extrusion_spacing
//   RRW s h -> out                             Flow::rounded_rectangle_extrusion_width_from_spacing
//   BES d -> out                               Flow::bridge_extrusion_spacing
//   FLOW w h nd -> F | ERR                     Flow(w, h, nd)
//   BRIDGE d nd -> F                           Flow::bridging_flow(d, nd)
//   WW w h nd w2 -> F | ERR                    Flow(w, h, nd).with_width(w2)
//   WH w h nd h2 -> F | ERR                    Flow(w, h, nd).with_height(h2)
//   WSP b w h nd s -> F | ERR                  (b ? bridging_flow(w, nd) : Flow(w, h, nd)).with_spacing(s)
//   WCS w h nd a -> F | ERR                    Flow(w, h, nd).with_cross_section(a)
//   WFR w h nd ratio -> F | ERR                Flow(w, h, nd).with_flow_ratio(ratio), ratio a double
//   CFG key has_key v_key has_line v_line has_nd n nd... first preset_nd nd multi -> EW RES EWN
//       Flow::extrusion_width(key, config, first) ("-" without the key), resolve_extrusion_width_for_nozzle
//       and extrusion_width_for_nozzle(ConfigOptionFloat(v_key), ...): a double, or E: and the hex of what()
// where F is "width height spacing nozzle bridge mm3_per_mm scaled_width scaled_spacing
// scaled_elephant_foot_spacing", mm3_per_mm a double ("ERR" when it throws).

#include "libslic3r/Flow.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>

using namespace Slic3r;

// The vectors go to the file named on the command line: the engine logs to stdout.
static FILE *g_out = stdout;

static uint64_t s = 0x2545F4914F6CDD1Dull;
static uint64_t rnd() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
static double   unit() { return double(rnd() >> 11) / double(1ull << 53); }
static float    uni(double a, double b) { return float(a + (b - a) * unit()); }

static unsigned fb(float f) { unsigned u; memcpy(&u, &f, 4); return u; }
static unsigned long long db(double d) { unsigned long long u; memcpy(&u, &d, 8); return u; }

static void pf(const Flow &f)
{
    fprintf(g_out, " %08x %08x %08x %08x %d", fb(f.width()), fb(f.height()), fb(f.spacing()), fb(f.nozzle_diameter()), int(f.bridge()));
    try {
        fprintf(g_out, " %016llx", db(f.mm3_per_mm()));
    } catch (const std::exception &) {
        fprintf(g_out, " ERR");
    }
    fprintf(g_out, " %d %d %d\n", f.scaled_width(), f.scaled_spacing(), f.scaled_elephant_foot_spacing());
}

static const float nozzles[] = { 0.2f, 0.4f, 0.6f, 0.8f, 0.25f, 0.5f };
static const float widths[]  = { 0.42f, 0.45f, 0.4f, 0.5f, 0.62f, 0.82f, 0.22f, 0.3f, 1.0f, 0.35f };
static const float heights[] = { 0.2f, 0.16f, 0.08f, 0.12f, 0.24f, 0.28f, 0.1f, 0.3f, 0.06f, 0.4f };

static float nozzle() { return (rnd() & 1) ? nozzles[rnd() % 6] : uni(0.1, 1.2); }
static float width(float nd) { return (rnd() & 1) ? widths[rnd() % 10] : uni(0.5 * nd, 2.2 * nd); }
static float height(float w) { return (rnd() & 1) ? heights[rnd() % 10] : uni(0.02, double(w) * 1.1); }

int main(int argc, char **argv)
{
    if (argc > 1)
        g_out = fopen(argv[1], "w");
    const char *roles[] = { "frExternalPerimeter", "frPerimeter", "frInfill", "frSolidInfill", "frTopSolidInfill",
                            "frSupportMaterial", "frSupportMaterialInterface", "frSupportTransition" };
    (void)roles;
    for (int i = 0; i < 400; ++ i) {
        int role = int(rnd() % 8);
        float nd = nozzle();
        fprintf(g_out, "AUTO %d %08x -> %08x\n", role, fb(nd), fb(Flow::auto_extrusion_width(FlowRole(role), nd)));
    }
    for (int i = 0; i < 1200; ++ i) {
        float w = width(nozzle()), h = (i % 7 == 0) ? uni(0.0, 3.0) : height(w);
        fprintf(g_out, "RRS %08x %08x ->", fb(w), fb(h));
        try { fprintf(g_out, " %08x\n", fb(Flow::rounded_rectangle_extrusion_spacing(w, h))); } catch (const std::exception &) { fprintf(g_out, " ERR\n"); }
        float sp = uni(0.05, 1.5);
        fprintf(g_out, "RRW %08x %08x -> %08x\n", fb(sp), fb(h), fb(Flow::rounded_rectangle_extrusion_width_from_spacing(sp, h)));
        float d = nozzle();
        fprintf(g_out, "BES %08x -> %08x\n", fb(d), fb(Flow::bridge_extrusion_spacing(d)));
    }
    for (int i = 0; i < 1200; ++ i) {
        float nd = nozzle(), w = width(nd), h = height(w);
        fprintf(g_out, "FLOW %08x %08x %08x ->", fb(w), fb(h), fb(nd));
        try { Flow f(w, h, nd); pf(f); } catch (const std::exception &) { fprintf(g_out, " ERR\n"); }
        float d = nozzle();
        fprintf(g_out, "BRIDGE %08x %08x ->", fb(d), fb(nd));
        pf(Flow::bridging_flow(d, nd));
    }
    for (int i = 0; i < 1200; ++ i) {
        float nd = nozzle(), w = width(nd), h = height(w);
        float w2 = width(nd), h2 = height(w);
        fprintf(g_out, "WW %08x %08x %08x %08x ->", fb(w), fb(h), fb(nd), fb(w2));
        try { pf(Flow(w, h, nd).with_width(w2)); } catch (const std::exception &) { fprintf(g_out, " ERR\n"); }
        fprintf(g_out, "WH %08x %08x %08x %08x ->", fb(w), fb(h), fb(nd), fb(h2));
        try { pf(Flow(w, h, nd).with_height(h2)); } catch (const std::exception &) { fprintf(g_out, " ERR\n"); }
        int b = int(rnd() & 1);
        float sp = (rnd() & 1) ? uni(0.1, 1.5) : uni(0.9, 1.1) * w;
        if (b) {
            // with_spacing asserts m_width >= m_height for non-bridges only.
            fprintf(g_out, "WSP 1 %08x %08x %08x %08x ->", fb(w), fb(w), fb(nd), fb(sp));
            try { pf(Flow::bridging_flow(w, nd).with_spacing(sp)); } catch (const std::exception &) { fprintf(g_out, " ERR\n"); }
        } else if (w >= h) {
            fprintf(g_out, "WSP 0 %08x %08x %08x %08x ->", fb(w), fb(h), fb(nd), fb(sp));
            try { pf(Flow(w, h, nd).with_spacing(sp)); } catch (const std::exception &) { fprintf(g_out, " ERR\n"); }
        }
        if (w >= h) {
            float a = 0.f;
            try {
                double m = Flow(w, h, nd).mm3_per_mm();
                switch (rnd() % 4) {
                case 0: a = float(m * uni(1.0, 2.5)); break;
                case 1: a = float(m * uni(0.05, 1.0)); break;
                case 2: a = float(m + (int(rnd() % 5) - 2) * 0.5e-4); break;
                default: a = float(m); break;
                }
            } catch (const std::exception &) { a = uni(0.001, 0.5); }
            fprintf(g_out, "WCS %08x %08x %08x %08x ->", fb(w), fb(h), fb(nd), fb(a));
            try { pf(Flow(w, h, nd).with_cross_section(a)); } catch (const std::exception &) { fprintf(g_out, " ERR\n"); }
            double ratio = (rnd() & 1) ? double(uni(0.3, 2.0)) : (rnd() & 1) ? 1.0 : 0.95 + 0.01 * double(rnd() % 11);
            fprintf(g_out, "WFR %08x %08x %08x %016llx ->", fb(w), fb(h), fb(nd), db(ratio));
            try { pf(Flow(w, h, nd).with_flow_ratio(ratio)); } catch (const std::exception &) { fprintf(g_out, " ERR\n"); }
        }
    }
    // The config-driven widths. The config holds the role's width, line_width and nozzle_diameter,
    // each present or not (a missing one is written "-").
    const char *keys[] = { "inner_wall_line_width", "line_width", "initial_layer_line_width", "outer_wall_line_width",
                           "sparse_infill_line_width", "skeleton_infill_line_width", "initial_layer_infill_line_width",
                           "internal_solid_infill_line_width", "skin_infill_line_width", "top_surface_line_width",
                           "support_line_width", "bridge_line_width" };
    for (int i = 0; i < 600; ++ i) {
        const char *key = keys[rnd() % 12];
        DynamicConfig cfg;
        auto value = [](int k) { return k == 0 ? 0. : double(uni(0.1, 1.2)); };
        int has_key = int(rnd() % 4 != 0), has_line = int(rnd() % 4 != 0), has_nd = int(rnd() % 4 != 0);
        double v_key = value(int(rnd() % 3)), v_line = value(int(rnd() % 3));
        std::vector<double> nds;
        for (size_t k = 0, n = 1 + rnd() % 3; k < n; ++ k) nds.push_back(double(nozzle()));
        if (has_line) cfg.set_key_value("line_width", new ConfigOptionFloat(v_line));
        if (has_key) cfg.set_key_value(key, new ConfigOptionFloat(v_key));
        if (has_nd) cfg.set_key_value("nozzle_diameter", new ConfigOptionFloatsNullable(nds));
        unsigned first = unsigned(rnd() % nds.size());
        float preset_nd = (rnd() % 5 == 0) ? 0.f : nozzle(), nd = (rnd() % 5 == 0) ? 0.f : nozzle();
        int multi = int(rnd() & 1);
        fprintf(g_out, "CFG %s %d %016llx %d %016llx %d %zu", key, has_key, db(v_key), has_line, db(v_line), has_nd, nds.size());
        for (double d : nds) fprintf(g_out, " %016llx", db(d));
        fprintf(g_out, " %u %08x %08x %d ->", first, fb(preset_nd), fb(nd), multi);
        auto run = [](auto f) {
            try { fprintf(g_out, " %016llx", db(f())); }
            catch (const std::exception &e) { fprintf(g_out, " E:"); for (const char *c = e.what(); *c; ++ c) fprintf(g_out, "%02x", (unsigned char)*c); }
        };
        // Flow::extrusion_width(opt_key, config, first) asserts a non-null option: only with the key.
        if (has_key) run([&] { return Flow::extrusion_width(key, cfg, first); }); else fprintf(g_out, " -");
        run([&] { return Flow::resolve_extrusion_width_for_nozzle(key, cfg, preset_nd, nd, multi != 0); });
        ConfigOptionFloat width(v_key);
        run([&] { return Flow::extrusion_width_for_nozzle(width, preset_nd, nd, multi != 0); });
        fprintf(g_out, "\n");
    }
    return 0;
}
