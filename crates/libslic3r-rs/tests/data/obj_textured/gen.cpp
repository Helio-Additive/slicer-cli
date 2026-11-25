// Native vectors for obj_to_textured_mesh (OBJ.cpp:287-420) and the textured-OBJ branch of Model::read_from_file
// (Model.cpp:344-370): run from crates/libslic3r-rs with the files' paths relative to it.
//
//   gen <path>...
//
// Per file: load_obj with its MtlData (OBJ.cpp:248-264) and the ObjInfo fields obj_to_textured_mesh reads, then
// obj_to_textured_mesh on the loaded volume's mesh with the OBJ's directory ("direct"), then the texture mesh
// Model::read_from_file builds after turning the mesh Y-up to Z-up ("read"). A textured mesh prints its
// vertices, faces, UV pool and UV faces, material colours, texture map, per-face material ids and per texture
// its width, height, channels, size and byte sum.
//
// Build with the engine's compile and link line (target/parity-tools/build.sh in a checkout).
#include <cstdio>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include "libslic3r/Model.hpp"
#include "libslic3r/Format/OBJ.hpp"
#include "libslic3r/TexturePainting.hpp"

using namespace Slic3r;

static void print_tex(const char *tag, bool ok, const TexturedMesh &t) {
    printf("%s %d %zu %zu %zu %zu %zu %zu %zu %zu %zu\n", tag, (int) ok, t.vertices.size(), t.indices.size(), t.uv_coords.size(),
           t.uv_indices.size(), t.material_colors.size(), t.material_texture_map.size(), t.material_ids.size(), t.textures.size(),
           t.precomputed_face_colors.size());
    for (const auto &v : t.vertices) printf("v %a %a %a\n", v[0], v[1], v[2]);
    for (const auto &f : t.indices) printf("f %d %d %d\n", f[0], f[1], f[2]);
    for (const auto &uv : t.uv_coords) printf("uv %a %a\n", uv[0], uv[1]);
    for (const auto &f : t.uv_indices) printf("uf %d %d %d\n", f[0], f[1], f[2]);
    for (const auto &c : t.material_colors) printf("mc %a %a %a %a\n", c[0], c[1], c[2], c[3]);
    printf("mt");
    for (int m : t.material_texture_map) printf(" %d", m);
    printf("\nmi");
    for (int m : t.material_ids) printf(" %d", m);
    printf("\n");
    for (const auto &ti : t.textures) {
        unsigned long long sum = 0;
        for (unsigned char b : ti.data) sum += b;
        printf("tex %d %d %d %zu %llu\n", ti.width, ti.height, ti.channels, ti.data.size(), sum);
    }
}

int main(int argc, char **argv) {
    for (int a = 1; a < argc; ++a) {
        std::string path = argv[a];
        printf("case %s\n", path.c_str());
        Model model;
        ObjInfo obj_info;
        ObjParser::MtlData mtl_data;
        std::string message;
        bool ok = load_obj(path.c_str(), &model, obj_info, message, nullptr, false, &mtl_data);
        printf("load %d %d %zu %zu %zu\n", (int) ok, (int) obj_info.has_uv_png, obj_info.uvs.size(), obj_info.usemtls.size(),
               mtl_data.mtl_orders.size());
        if (!ok) continue;
        for (const auto &u : obj_info.uvs) printf("ouv %a %a %a %a %a %a\n", u[0].x(), u[0].y(), u[1].x(), u[1].y(), u[2].x(), u[2].y());
        for (const auto &u : obj_info.usemtls) printf("um %d %d %s\n", u.face_start, u.face_end, u.name.c_str());
        std::string obj_dir = boost::filesystem::path(path).parent_path().string();
        TexturedMesh direct;
        bool dok = obj_to_textured_mesh(obj_info, model.objects.back()->volumes[0]->mesh().its, mtl_data, obj_dir, direct);
        print_tex("direct", dok, direct);
        try {
            DynamicPrintConfig config;
            ConfigSubstitutionContext subs(ForwardCompatibilitySubstitutionRule::EnableSilent);
            Model m2 = Model::read_from_file(path, &config, &subs, LoadStrategy::AddDefaultInstances);
            if (m2.texture_mesh)
                print_tex("read", true, *m2.texture_mesh);
            else
                printf("read none\n");
        } catch (const std::exception &e) {
            printf("read err %s\n", e.what());
        }
    }
    return 0;
}
