// BambuStudio 56a57d0, src/nanosvg/nanosvg.h 241–355.
// c++ -std=c++17 -Ireferences/BambuStudio/src/nanosvg crates/nanosvg/tests/data/gen.cpp -o gen
// ./gen > crates/nanosvg/tests/data/xml.txt
#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"
#include <iostream>
#include <string>
#include <string_view>

static void field(std::string_view bytes)
{
    static const char hex[] = "0123456789abcdef";
    std::cout << ' ';
    if (bytes.empty())
        std::cout << '-';
    for (unsigned char b : bytes)
        std::cout << hex[b >> 4] << hex[b & 15];
}

static void start(void*, const char* name, const char** attributes)
{
    std::cout << 'S';
    field(name);
    for (const char** p = attributes; *p; ++p)
        field(*p);
    std::cout << '\n';
}

static void end(void*, const char* name)
{
    std::cout << 'E';
    field(name);
    std::cout << '\n';
}

static void content(void*, const char* value)
{
    std::cout << 'C';
    field(value);
    std::cout << '\n';
}

static void sample(std::string input)
{
    std::cout << 'I';
    field(input);
    std::cout << '\n';
    nsvg__parseXML(input.data(), start, end, content, nullptr);
}

int main()
{
    using namespace std::string_literals;
    sample("<?xml version='1.0'?><svg width='12' height=\"42\"><rect x='0' /></svg>");
    sample("<svg><g/> \ttext  <text>  label &amp; \n</text><g /></svg>tail");
    sample("<?xml?><!DOCTYPE svg><!-- note -->leading<svg bad >title</svg>");
    sample("<svg><path d=\"M0 0L-1.5e-2 20Z\" aria-label='\xc3\xa9'/></svg>");
    sample("<svg>\v \r\t\n&#48; \n<g / >\0</svg>"s);
    sample("<s a='' b = \"4\" broken 'unclosed>< / ></>");
    for (int count : {126, 127, 128}) {
        std::string input = "<svg";
        for (int i = 0; i < count; ++i)
            input += " a" + std::to_string(i) + "='" + std::to_string(i) + "'";
        sample(input + " />");
    }
}
