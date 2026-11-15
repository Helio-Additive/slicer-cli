// Reference vectors for crates/libslic3r-rs/src/format/resource_path_utils.rs, taken from the C++ at
// the pin.
//
// The program includes references/BambuStudio/src/libslic3r/Format/ResourcePathUtils.hpp as it is,
// with the system's Boost.Filesystem (version 3 semantics, its default) and the pin's Log.hpp.
//
// Build and run, from the repository root (PREFIX is where Boost is installed; STUB is a directory
// holding an empty libslic3r_version.h, the header CMake generates; the last argument is a
// directory that does not exist yet, which the program creates, fills and leaves):
//   clang++ -w -std=c++17 -O2 -DBOOST_LOG_DYN_LINK -I $STUB -I references/BambuStudio/src/libslic3r \
//       -I references/BambuStudio/src -I $PREFIX/include \
//       crates/libslic3r-rs/tests/data/resource_path_utils/gen.cpp \
//       -L $PREFIX/lib -lboost_filesystem -lboost_log -lboost_thread -o /tmp/rpu_gen
//   /tmp/rpu_gen /tmp/rpu_tree > crates/libslic3r-rs/tests/data/resource_path_utils/vectors.txt
//
// Output, one case per line. Strings are the hex of their bytes ("-" when empty). In the file system
// cases the directory the program made is written $ROOT, in the inputs and in the results; the
// program runs with it as the current directory.
//   HEX 256 values                                 hex_digit_value of every byte
//   STR s -> ascii_lower_copy percent_decode_copy strip_file_uri_prefix_copy file_uri_has_remote_authority
//   PATH s -> root_name root_directory is_absolute filename n elements..      boost::filesystem::path
//             portable_path_copy looks_like_windows_absolute_path filename_from_portable_path
//   JOIN a b -> a / b
//   DIR path                                       a directory of the tree
//   FILE path                                      a file of the tree
//   CHILD directory requested -> find_child_case_insensitive
//   EXIST path -> resolve_existing_path_case_insensitive
//   REL base path -> resolve_existing_relative_path_case_insensitive
//   EXT base raw allow_basename_fallback -> resolve_external_resource_path
// The file system cases ask for names in the case they have, or for names that do not exist in any
// case, so that the answers are the same on a file system that ignores case and on one that does
// not; only CHILD, which reads the directory itself, asks in another case.

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "Format/ResourcePathUtils.hpp"

// Log.hpp asks this before it logs; it is defined in Log.cpp. Nothing is logged here: the log would
// go to the same stream as the vectors.
namespace Slic3r { namespace log_detail { bool log_enabled(LogLevel) { return false; } } }

using namespace Slic3r::resource_path;
namespace fs = boost::filesystem;

static uint64_t s = 0x9E3779B97F4A7C15ull;
static uint64_t rnd() { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return s; }
static uint64_t rnd(uint64_t n) { return rnd() % n; }
static std::string g_root;

static void ps(const std::string &v)
{
    printf(" ");
    if (v.empty()) printf("-");
    for (unsigned char c : v) printf("%02x", c);
}
// With the directory of the tree written $ROOT.
static std::string rooted(std::string v)
{
    for (size_t pos = v.find(g_root); pos != std::string::npos; pos = v.find(g_root, pos + 5))
        v.replace(pos, g_root.size(), "$ROOT");
    return v;
}
static void pr(const std::string &v) { ps(rooted(v)); }

template<size_t N> static std::string pieces(const char *const (&table)[N], size_t max_count)
{
    std::string out;
    for (size_t i = 0, n = rnd(max_count + 1); i < n; ++ i) out += table[rnd(N)];
    return out;
}

int main(int argc, char **argv)
{
    if (argc != 2) return 1;

    printf("HEX");
    for (int c = 0; c < 256; ++ c) printf(" %d", hex_digit_value(char(c)));
    printf("\n");

    static const char *const str_pieces[] = {
        "file://", "FILE://", "File:///", "localhost", "LocalHost/", "/", "/", "C:", "c:/", "%", "%4", "%41", "%2f", "%2F", "%zz", "%e4%b8%ad",
        "a", "B", "Z", "tex.PNG", " ", "\\", "\xc3\x89", "\xe4\xb8\xad", "host", "//", "%25", "@", "[", "`", "{" };
    for (int i = 0; i < 700; ++ i) {
        std::string in = pieces(str_pieces, 7);
        if (i < 12) {
            static const char *fixed[] = { "", "file://", "file:///", "file://localhost", "file://localhost/", "file:///C:/a/b.png", "file://host/share/a.png",
                                           "file:///tmp/a%20b.png", "FILE://LOCALHOST/C:/x", "%", "%4", "file://c:/x" };
            in = fixed[i];
        }
        printf("STR"); ps(in); printf(" ->");
        ps(ascii_lower_copy(in)); ps(percent_decode_copy(in)); ps(strip_file_uri_prefix_copy(in));
        printf(" %d\n", int(file_uri_has_remote_authority(in)));
    }

    static const char *const path_pieces[] = { "/", "/", "/", "//", "\\", "a", "b.png", "Dir", ".", "..", "C:", "net", "x y", "\xc3\xa9" };
    for (int i = 0; i < 500; ++ i) {
        std::string in = pieces(path_pieces, 6);
        if (i < 14) {
            static const char *fixed[] = { "", "/", "//", "///", "//net", "//net/", "//net/a/", "a/b/", "C:/x/y", "C:\\x\\y", "c:x", "a\\b\\c.png", ".", "./" };
            in = fixed[i];
        }
        const fs::path p(in);
        printf("PATH"); ps(in); printf(" ->");
        ps(p.root_name().string()); ps(p.root_directory().string()); printf(" %d", int(p.is_absolute())); ps(p.filename().string());
        std::vector<std::string> elements;
        for (const fs::path &e : p) elements.push_back(e.string());
        printf(" %zu", elements.size());
        for (const std::string &e : elements) ps(e);
        ps(portable_path_copy(p).string()); printf(" %d", int(looks_like_windows_absolute_path(p))); ps(filename_from_portable_path(p).string());
        printf("\n");
    }
    for (int i = 0; i < 200; ++ i) {
        const std::string a = pieces(path_pieces, 4), b = pieces(path_pieces, 3);
        printf("JOIN"); ps(a); ps(b); printf(" ->"); ps((fs::path(a) / b).string()); printf("\n");
    }

    // The tree.
    fs::create_directories(argv[1]);
    fs::current_path(argv[1]);
    g_root = fs::current_path().string();
    static const char *dirs[]  = { "models", "models/Textures", "models/Textures/Sub Dir", "other", "models/\xc3\x89t\xc3\xa9", "empty" };
    static const char *files[] = { "models/scene.gltf", "models/Textures/Wood.PNG", "models/Textures/metal.jpg", "models/Textures/Sub Dir/a b.png",
                                   "other/Wood.PNG", "top.png", "models/\xc3\x89t\xc3\xa9/\xe4\xb8\xad.png", "models/Textures/100%.png", "models/C:" };
    for (const char *d : dirs) { fs::create_directories(d); printf("DIR"); ps(d); printf("\n"); }
    for (const char *f : files) { std::ofstream(f) << "x"; printf("FILE"); ps(f); printf("\n"); }

    // find_child_case_insensitive reads the directory, so it may ask in any case.
    const std::vector<std::string> child_dirs = { "models", "models/Textures", g_root + "/models/Textures", "models/Textures/", "other", "missing", "top.png", "", ".", "empty", "models/Textures/Sub Dir" };
    const std::vector<std::string> child_names = { "wood.png", "WOOD.PNG", "Wood.PNG", "METAL.JPG", "textures", "TEXTURES", "scene.GLTF", "x/y/Metal.Jpg", "sub dir", "nothing", "", "A B.PNG", "wood.png/", "\xc3\x89T\xc3\x89", "\xc3\xa9t\xc3\xa9", "c:", "100%.PNG" };
    for (const std::string &d : child_dirs)
        for (const std::string &n : child_names) {
            printf("CHILD"); pr(d); ps(n); printf(" ->"); pr(find_child_case_insensitive(d, n, "gen").string()); printf("\n");
        }

    // Names in their own case, and names that exist in no case.
    const std::vector<std::string> paths = {
        "", "top.png", "./top.png", "models/Textures/Wood.PNG", "models\\Textures\\Wood.PNG", "models/Textures/missing.png", "missing/Wood.PNG",
        "models/Textures/Sub Dir/a b.png", "models/../top.png", "models/./scene.gltf", "models//Textures///metal.jpg", "models/Textures/", "models/Textures",
        "models/scene.gltf/x", "top.png/", "nothing", "models/\xc3\x89t\xc3\xa9/\xe4\xb8\xad.png", "//nohost/share/x.png", "C:/missing/x.png", "models/C:", ".", ".." };
    std::vector<std::string> all = paths;
    for (const std::string &p : paths) all.push_back(g_root + "/" + p);
    all.push_back(g_root);
    all.push_back(g_root + "/");
    all.push_back("/");
    for (const std::string &p : all) {
        printf("EXIST"); pr(p); printf(" ->"); pr(resolve_existing_path_case_insensitive(p, "gen").string()); printf("\n");
    }
    const std::vector<std::string> bases = { "", "models", "models/", g_root + "/models", "models/Textures", "missing", "top.png", "." };
    for (const std::string &b : bases)
        for (const std::string &p : { std::string("scene.gltf"), std::string("Textures/Wood.PNG"), std::string("Textures\\metal.jpg"), std::string("../top.png"),
                                      std::string("Wood.PNG"), std::string("missing.png"), std::string(""), g_root + "/top.png", g_root + "/missing.png",
                                      std::string("/nonexistent-root-entry/x"), std::string("Sub Dir/a b.png") }) {
            printf("REL"); pr(b); pr(p); printf(" ->"); pr(resolve_existing_relative_path_case_insensitive(b, p, "gen").string()); printf("\n");
        }
    const std::vector<std::string> raws = {
        "", "scene.gltf", "Textures/Wood.PNG", "Textures\\Wood.PNG", "Textures/Sub%20Dir/a%20b.png", "Textures/100%25.png", "Textures/100%.png",
        "file://" + g_root + "/models/Textures/metal.jpg", "FILE://localhost" + g_root + "/top.png", "file://remotehost/share/metal.jpg",
        "file://remotehost/share/missing.jpg", "file:///C:/Users/x/Desktop/Wood.PNG", "C:\\Users\\x\\Desktop\\metal.jpg", "C:/Users/x/missing.png",
        "/abs/missing/dir/metal.jpg", "/abs/missing/dir/", "some/where/else/scene.gltf", "some/where/else/", "%C3%89t%C3%A9/%E4%B8%AD.png",
        "\xc3\x89t\xc3\xa9/\xe4\xb8\xad.png", "../top.png", "x/../../top.png", "missing.png", "file://", "file:///", "file://localhost", "%", "Textures", "Textures/",
        g_root + "/models/scene.gltf", "file:///" + g_root.substr(1) + "/missing/top.png", "d:/a/b/Wood.PNG", "//host/share/Wood.PNG" };
    for (const std::string &b : { std::string("models"), std::string("models/Textures"), g_root + "/models", std::string(""), std::string("missing"), std::string("other") })
        for (const std::string &r : raws)
            for (int fallback = 0; fallback < 2; ++ fallback) {
                printf("EXT"); pr(b); pr(r); printf(" %d ->", fallback); pr(resolve_external_resource_path(b, r, "gen", fallback != 0).string()); printf("\n");
            }
    return 0;
}
