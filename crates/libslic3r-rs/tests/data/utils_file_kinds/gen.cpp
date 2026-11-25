// Vectors for tests/utils_file_kinds_native_parity.rs: the pin's is_plain_file, is_ini_file, is_idx_file and
// is_gallery_file(directory_entry, ".png") (utils.cpp:867-904) over the entries of a directory the generator
// lays out (the test lays out the same one). Usage: gen <empty directory> > vectors.txt
#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <boost/filesystem.hpp>
#include "libslic3r/Utils.hpp"

using namespace Slic3r;
namespace fs = boost::filesystem;

int main(int argc, char **argv)
{
    if (argc < 2) return 1;
    fs::path dir(argv[1]);
    for (const char *f : {"a.ini", "b.INI", "c.Idx", "d.txt", ".ini", "e.ini.bak", "f.", "g.png", "h.PNG", "noext", "x.y.idx"})
        std::ofstream((dir / f).string()) << "x";
    fs::create_directory(dir / "sub.ini");
    fs::create_symlink("a.ini", dir / "link.ini");
    fs::create_symlink("missing.ini", dir / "broken.ini");
    fs::create_symlink("sub.ini", dir / "dirlink.ini");
    std::map<std::string, std::string> out;
    for (const fs::directory_entry &e : fs::directory_iterator(dir)) {
        char buf[64];
        snprintf(buf, sizeof(buf), "%d %d %d %d", int(is_plain_file(e)), int(is_ini_file(e)), int(is_idx_file(e)), int(is_gallery_file(e, ".png")));
        out[e.path().filename().string()] = buf;
    }
    for (const auto &kv : out) printf("%s %s\n", kv.first.c_str(), kv.second.c_str());
    return 0;
}
