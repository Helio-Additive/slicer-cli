// Build against BambuStudio 56a57d0 src/nanosvg (C++17, -O3).
// Capture sharp and rounded rectangle paths in complete SVG documents.
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
    for (const char *suffix : {"","px","pt","pc","mm","cm","in","em","ex","%","MM","unknown"}) {
        std::string n = std::string("1.375") + suffix;
        document("rect", "x='-3.14' y='23.25%' width='"+n+"' height='12.75' rx='0.75' ry='0.36'");
        document("rect", "x='"+n+"' y='-1.7mm' rx='22%' width='19%' height='4em' transform='matrix(0 1 -1 0 3 -8)'");
    }
    for (const char *attrs : {"","width='3'","height='4'","width='-8' height='7' rx='3'","width='5' height='-9' ry='1'",
                              "x='8' y='5.5' width='-8' height='-5'","width='-0' height='4'",
                              "width='24' height='8' rx='0.000009999' ry='0.01'","width='24' height='8' rx='0.00001' ry='0.0001'",
                              "width='24' height='8' rx='0.07' ry='0.00009999'","width='24' height='8' rx='-4.375'",
                              "width='24' height='8' ry='-3.125'","width='24' height='8' rx='21' ry='19'",
                              "width='24' height='8' rx='0' ry='1.1'","width='24' height='8' rx='1.5' ry='-0'",
                              "width='29' height='5' rx='0.33' rx='-1.37'","width='6em' style='font-size: 7.75' height='2em' rx='0.099em'",
                              "x='4em' font-size='60%' width='2em' height='1.25ex' style='font-size: 3' ry='0.2ex'",
                              "x='-123.4567' y='0.01238888' width='13.714477' height='5.2378134' rx='2.37014' ry='0.114789'"})
        document("rect",attrs);
}
