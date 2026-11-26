// BambuStudio 56a57d0 src/nanosvg/nanosvg.h 789-805, 1494-1514,
// 1526-1545. Compile with -std=c++17 -O3 and the pin's NanoSVG include path.
#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"
#include <cstdio>
#include <cstring>
#include <string>

static unsigned bits(float value)
{
    unsigned result;
    static_assert(sizeof(result) == sizeof(value));
    std::memcpy(&result, &value, sizeof(result));
    return result;
}

static void sample(NSVGparser* parser, const std::string& text, float origin, float length)
{
    NSVGcoordinate raw = nsvg__parseCoordinateRaw(text.c_str());
    float value = nsvg__parseCoordinate(parser, text.c_str(), origin, length);
    std::printf("%s|%08x %08x %08x %08x %d %08x %08x\n", text.c_str(),
                bits(parser->dpi), bits(nsvg__getAttr(parser)->fontSize), bits(origin),
                bits(length), int(raw.units), bits(raw.value), bits(value));
}

int main()
{
    NSVGparser* parser = nsvg__createParser();
    for (float dpi : {72.f, 96.f, 144.f}) {
        parser->dpi = dpi;
        nsvg__getAttr(parser)->fontSize = dpi == 72.f ? 0.f : 13.f;
        for (const char* unit : {"", "px", "pt", "pc", "mm", "cm", "in", "em", "ex", "%", "MM", "unknown", "%tail"})
            for (const char* number : {"0", "-0", "-2.5", ".125", "17.325901", "1e-2", "garbage", " 3"})
                sample(parser, std::string(number) + unit, -17.03125f, 137.125f);
    }
    for (const char* value : {"", ".", "12Em", "2px.more", "1.5 cm", "+1.75e+2pt",
         "12345678912345678912345678912345678912345678912345678912345678912345mm"})
        sample(parser, value, 32.f, 16.f);
    nsvg__deleteParser(parser);
}
