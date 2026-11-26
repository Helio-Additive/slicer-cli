// BambuStudio 56a57d0 src/nanosvg/nanosvg.h 1090-1230,2556-2590,3002-3032.
// clang++ -std=c++17 -O3 -Ireferences/BambuStudio/src/nanosvg poly_gen.cpp -o gen
// ./gen > polygons.txt
#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <string>
#include <vector>

static void bits(float value) {
    uint32_t word;
    std::memcpy(&word, &value, sizeof word);
    std::printf(" %08x", word);
}

static void emit(const char* points, const char* second, bool closed, bool transform = false) {
    const char* tag = closed ? "polygon" : "polyline";
    std::string input = "<svg width='100' height='100' viewBox='0 0 100 100'><";
    input += tag;
    input += " points='";
    input += points;
    input += "'";
    if (second) {
        input += " points='";
        input += second;
        input += "'";
    }
    if (transform)
        input += " transform='matrix(0 1 -1 0 3 -8)'";
    input += "/></svg>";
    std::printf("svg %s\n", input.c_str());
    NSVGimage* image = nsvgParse(input.data(), "px", 96.f);
    for (NSVGshape* shape = image->shapes; shape; shape = shape->next) {
        for (NSVGpath* path = shape->paths; path; path = path->next) {
            std::printf("path %d %d", int(path->closed), path->npts);
            for (float value : path->bounds)
                bits(value);
            for (int i = 0; i < path->npts * 2; ++i)
                bits(path->pts[i]);
            std::puts("");
        }
    }
    std::puts("end");
    nsvgDelete(image);
}

int main() {
    const char* cases[] = {
        "",
        "0",
        "0 0",
        "0 0 1",
        "0 0 1 ",
        "  0 0 1,",
        "0 0 2 3 ",
        "0,0 12.5,-3.75",
        "0,0 1e-1 2E+1",
        "0,0 .5-.25",
        "0,0 3em 4ex",
        "0,0 .123456789123456789123456789 2.5",
        "0,0 -9223372036854775812 -0",
        "0,0 3x 4",
        "0 0 + .",
        "0 0 12e 3",
        "0 0 .5123456789012345678901234567890123456789012345678901234567890123456789 2",
        "0 0 1 1 1 1 0 0",
        "4,7,0,2,3,2,4,0",
        "1 2,3,4,5"};
    std::vector<std::string> decimals;
    for (int length : {1, 8, 17, 18, 19, 20, 61, 62, 63, 64, 65, 80}) {
        const std::string digits(length, '9');
        decimals.push_back("0,0 -." + digits + "e-2 3");
        decimals.push_back(digits + ",2 5,-1");
        decimals.push_back(".0000" + digits + "," + digits + " 2 1");
    }
    for (bool closed : {false, true}) {
        for (const auto& points : decimals)
            emit(points.c_str(), nullptr, closed);
        for (const char* points : cases)
            emit(points, nullptr, closed);
        emit("1.25 .5 4.5 -2.75 9 0", nullptr, closed, true);
        emit("0 0", "2 3", closed);
        emit("0 0 3", "4 5", closed);
        emit("0 0 3 ", "4 5", closed);
    }
}
