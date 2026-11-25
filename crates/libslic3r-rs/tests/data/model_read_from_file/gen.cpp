// Native vectors for Model::read_from_file (Model.cpp:306-491): run from crates/libslic3r-rs with the files'
// paths relative to it (they become ModelObject::input_file). For each file and LoadStrategy value prints
// the exception message or the model: objects, volumes, instances, mesh sizes and vertex sums.
//
//   gen <options> <temporary dir> <path>...   (the temporary dir is where load_bbs_3mf puts its backup)
//
// Build with the engine's compile and link line (target/parity-tools/build.sh in a checkout).
#include <cstdio>
#include <string>
#include <vector>
#include "libslic3r/Model.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Utils.hpp"

using namespace Slic3r;

static void print_hex(const std::string &s) {
    if (s.empty()) { printf("-"); return; }
    for (unsigned char c : s) printf("%02x", c);
}

int main(int argc, char **argv) {
    if (argc < 4) return 2;
    int options = atoi(argv[1]);
    set_temporary_dir(argv[2]);
    for (int a = 3; a < argc; ++a) {
        std::string path = argv[a];
        printf("case %s %d\n", path.c_str(), options);
        try {
            bool is_xxx = false;
            ConfigSubstitutionContext subs(ForwardCompatibilitySubstitutionRule::EnableSilent);
            DynamicPrintConfig config;
            Model model = Model::read_from_file(path, &config, &subs, LoadStrategy(options), nullptr, nullptr, &is_xxx);
            printf("ok %zu %d %zu %d\n", model.objects.size(), (int) is_xxx, subs.substitutions.size(), (int) (model.texture_mesh != nullptr));
            for (const ModelObject *o : model.objects) {
                printf("obj ");
                print_hex(o->name);
                printf(" ");
                print_hex(o->input_file);
                printf(" %zu %zu\n", o->volumes.size(), o->instances.size());
                for (const ModelVolume *v : o->volumes) {
                    const indexed_triangle_set &its = v->mesh().its;
                    double sx = 0, sy = 0, sz = 0;
                    for (const auto &p : its.vertices) { sx += p.x(); sy += p.y(); sz += p.z(); }
                    long long si = 0;
                    for (const auto &f : its.indices) si += (long long) f[0] * 3 + f[1] * 5 + f[2] * 7;
                    printf("vol %zu %zu %a %a %a %lld\n", its.indices.size(), its.vertices.size(), sx, sy, sz, si);
                }
            }
        } catch (const std::exception &e) {
            printf("err ");
            print_hex(e.what());
            printf("\n");
        }
    }
    return 0;
}
