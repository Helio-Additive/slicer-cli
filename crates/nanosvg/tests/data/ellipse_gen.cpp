// Build against BambuStudio 56a57d0 src/nanosvg (C++17, -O3).
// Capture circle and ellipse paths in complete SVG documents.
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

static void document(const std::string &name, const std::string &attrs)
{
    std::string input = "<svg width='128' height='80' viewBox='0 0 128 80' font-size='13'><" + name + " " + attrs + "/></svg>";
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
    for (const char *unit : {"", "px", "pt", "pc", "mm", "cm", "in", "em", "ex", "%", "MM", "unknown"}) {
        const std::string v = std::string("1.375") + unit;
        document("circle", "cx='-1.9375' cy='60%' r='" + v + "'");
        document("ellipse", "cx='14.25' cy='-5.125' rx='" + v + "' ry='2.75em'");
        document("ellipse", "rx='22%' " "cx='" + v + "' ry='3' ry='-1.125pc' transform='matrix(0 1 -1 0 3 -8)'");
    }
    document("circle", "");
    document("circle", "cx='10' r='-0'");
    document("circle", "r='-7.1234567%' cx='26%' cy='2.71828cm'");
    document("circle", "cx='1em' style='font-size: 1.75em' r='1.125ex' cy='2em'");
    document("circle", "r='1' r='2.92' cx='-17.32' transform='matrix(0 1 -1 0 3 -8)'");
    document("ellipse", "");
    document("ellipse", "rx='2' ry='0'");
    document("ellipse", "rx='0' ry='2'");
    document("ellipse", "rx='-5.5' ry='-3.3' cx='4.125' cy='-7.111'");
    document("ellipse", "font-size='10' rx='3em' style='font-size: 50%' ry='0.3ex'");
}
