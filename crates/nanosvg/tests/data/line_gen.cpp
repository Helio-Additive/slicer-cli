// Build against BambuStudio 56a57d0 src/nanosvg (C++17, -O3).
// Capture complete SVG line paths, including every cubic point and the bounds.
#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"
#include <cstdio>
#include <cstring>
#include <string>

static void bits(float value)
{
    unsigned result;
    static_assert(sizeof(result) == sizeof(value));
    std::memcpy(&result, &value, sizeof(result));
    std::printf(" %08x", result);
}

static void document(const std::string &attrs)
{
    std::string input = "<svg width='128' height='80' viewBox='0 0 128 80' font-size='13'><line " + attrs + "/></svg>";
    std::printf("svg %s\n", input.c_str());
    NSVGimage *image = nsvgParse(input.data(), "px", 96.f);
    for (auto *shape = image->shapes; shape; shape = shape->next)
        for (auto *p = shape->paths; p; p = p->next) {
            std::printf("path %d %d", int(p->closed), p->npts);
            for (float v : p->bounds) bits(v);
            for (int i = 0; i < p->npts * 2; ++i) bits(p->pts[i]);
            std::puts("");
        }
    std::puts("end");
    nsvgDelete(image);
}

int main()
{
    for (const char *unit : {"", "px", "pt", "pc", "mm", "cm", "in", "em", "ex", "%", "MM", "unknown", "%tail"}) {
        std::string v = std::string("1.25") + unit;
        document("x1='" + v + "' y1='25%' x2='-3.125' y2='1.75em'");
        document("x1='10' y1='" + v + "' x2='7' x2='21' y2='90%' transform='matrix(0 1 -1 0 3 -8)'");
    }
    document("x1='2em' font-size='20' x2='2em' y2='1ex'");
    document("font-size='2em' x1='1em' style='stroke: red; font-size: 50%' x2='1em'");
    document("style=' font-size : 3ex ; font-size: 1.5em;' x1='1em' y1='2ex'");
    document("x1='-0' y1='2px' x2='3.125mm' y2='8%'");
    document("");
    document("x1='0' y1='3' x2='0' y2='3'");
}
