// Native vectors for Format/3mf's PrusaFileParser (3mf.hpp:7-27, 3mf.cpp:274-358), the check the
// pin's Model::read_from_archive runs on every .3mf before it reads it (Model.cpp:494-510).
//
//   gen xml                 -> deterministic XML documents (LCG), each parsed by expat with the
//                              parser's own handlers as check_3mf_from_prusa does (3mf.cpp:301-316);
//                              prints the document (hex) and m_from_prusa / m_is_application_key
//   gen files <label=path>… -> check_3mf_from_prusa on each file; prints label and the result
//
// Build with the engine's compile and link line (target/parity-tools/build.sh in a checkout).
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <expat.h>
#define private public
#include "libslic3r/Format/3mf.hpp"
#undef private

using Slic3r::PrusaFileParser;

static uint64_t g_seed = 0x3D3A1F2Bu;
static uint32_t lcg() { g_seed = g_seed * 6364136223846793005ULL + 1442695040888963407ULL; return (uint32_t)(g_seed >> 33); }
static uint32_t pick(uint32_t n) { return lcg() % n; }

static void print_hex(const std::string &s) {
    if (s.empty()) { printf("-"); return; }
    for (unsigned char c : s) printf("%02x", c);
}

static const char *NAME_VALUES[] = {"Application", "Description", "", "MyApplication", "application",
                                    "Title", "BambuStudio:Application", "Appl&#105;cation", "Designer"};
static const char *ATTR_KEYS[] = {"name", "Name", "type", "value"};
static const char *TEXTS[] = {"PrusaSlicer-2.6.0+MacOS", "BambuStudio-02.04.00.70", "", "prusaslicer",
                              "xxPrusaSlicerxx", "Prusa", "Slicer", "Prusa&#83;licer", "Prusa&amp;Slicer",
                              "&lt;PrusaSlicer&gt;", "Prusa\nSlicer", "PrusaSlicer\r\nnext", "a&#x50;rusaSlicer",
                              " PrusaSlicer ", "Bambu &amp; PrusaSlicer"};

static std::string random_text() { return TEXTS[pick(sizeof(TEXTS) / sizeof(TEXTS[0]))]; }

static std::string random_metadata() {
    std::string s = "<metadata";
    uint32_t na = pick(3);
    for (uint32_t a = 0; a < na; ++a) {
        const char *k = ATTR_KEYS[pick(4)];
        if (a == 1 && !strcmp(k, "name")) k = "type"; // no duplicate attributes (expat refuses them)
        if (a == 1 && na == 2 && s.find(std::string(" ") + k + "=") != std::string::npos) continue;
        s += std::string(" ") + k + "=\"" + NAME_VALUES[pick(sizeof(NAME_VALUES) / sizeof(NAME_VALUES[0]))] + "\"";
    }
    uint32_t form = pick(5);
    if (form == 0) return s + "/>";
    if (form == 1) return s + "><![CDATA[" + random_text().substr(0, 0) + "PrusaSlicer" + (pick(2) ? "\nx" : "") + "]]></metadata>";
    if (form == 2) return s + "><b>" + random_text() + "</b></metadata>";
    return s + ">" + random_text() + "</metadata>";
}

static std::string random_doc() {
    std::string d = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    if (pick(4) == 0) d += "<!-- PrusaSlicer -->\n";
    d += "<model unit=\"millimeter\" xml:lang=\"en-US\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n";
    uint32_t n = 1 + pick(6);
    for (uint32_t i = 0; i < n; ++i) {
        uint32_t k = pick(8);
        if (k < 4) d += " " + random_metadata() + "\n";
        else if (k == 4) d += " <resources><object id=\"1\" type=\"model\"/></resources>\n";
        else if (k == 5) d += random_text() + "\n"; // mixed content after an element
        else if (k == 6) d += " <!-- PrusaSlicer --><?pi PrusaSlicer?>\n";
        else d += " <build/>\n";
    }
    d += "</model>\n";
    if (pick(6) == 0) d = d.substr(0, pick((uint32_t)d.size()) + 1); // truncated document
    return d;
}

int main(int argc, char **argv) {
    if (argc < 2) return 2;
    if (!strcmp(argv[1], "xml")) {
        const int N = 600;
        for (int c = 0; c < N; ++c) {
            std::string doc = random_doc();
            PrusaFileParser p;
            // 3mf.cpp:301-316 without the zip: the parser, its user data and handlers, one final buffer.
            p.m_parser = XML_ParserCreate(nullptr);
            XML_SetUserData(p.m_parser, (void *) &p);
            XML_SetElementHandler(p.m_parser, PrusaFileParser::start_element_handler, nullptr);
            XML_SetCharacterDataHandler(p.m_parser, PrusaFileParser::characters_handler);
            void *buf = XML_GetBuffer(p.m_parser, (int) doc.size());
            memcpy(buf, doc.data(), doc.size());
            XML_ParseBuffer(p.m_parser, (int) doc.size(), 1);
            XML_ParserFree(p.m_parser);
            printf("xml ");
            print_hex(doc);
            printf(" %d %d\n", (int) p.m_from_prusa, (int) p.m_is_application_key);
        }
        return 0;
    }
    if (!strcmp(argv[1], "files")) {
        for (int i = 2; i < argc; ++i) {
            std::string a = argv[i];
            size_t eq = a.find('=');
            std::string label = a.substr(0, eq), path = a.substr(eq + 1);
            PrusaFileParser p;
            bool r = p.check_3mf_from_prusa(path);
            printf("file %s %d\n", label.c_str(), (int) r);
        }
        return 0;
    }
    return 2;
}
