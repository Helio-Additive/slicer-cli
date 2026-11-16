// Generator of the vectors of tests/bbs_3mf_native_parity.rs: libslic3r's Format/bbs_3mf.cpp at
// the pin (references/BambuStudio/src/libslic3r; the override layer has no copy of it).
//
// Build, with REPO = this repository, BUILD = a built tree of the Bambu Studio engine
// (docs/building.md, -DENGINE=bambu) and DATA = this directory: compile with the compile line the
// build uses for a libslic3r source (its defines and include directories, the override layer
// $REPO/libslic3r/bambustudio before the pin, -std=c++17 -O3 -DNDEBUG and the forced include of
// cgal_54_compat.hpp), then link with the build's libslic3r_core and the libraries it links:
//
//   c++ <the build's defines and include directories> -std=c++17 -O3 -DNDEBUG \
//       -include $REPO/libslic3r/bambustudio/cgal_54_compat.hpp -c $DATA/gen.cpp -o gen.o
//   c++ gen.o $BUILD/libslic3r_core.a <the libraries libslic3r_core links> -o gen
//   ./gen $REPO <an empty scratch directory> > $DATA/vectors.txt
//
// (about 100 KB, kept uncompressed; the test also reads a vectors.txt.gz).
//
// `./gen $REPO <scratch> <a.3mf>` dumps the load of one more file only (used to compare a
// project that is not part of the repository, see the test).
//
// Two parts:
//  - load: each 3MF of the repository (crates/libslic3r-rs/tests/data/painted_cube.3mf and
//    tests/fixtures/*.3mf) is read with load_bbs_3mf (LoadModel | LoadConfig) and the result is
//    dumped: the project config, the embedded presets, the model info, every object (config,
//    layer heights, layer config ranges, brim ears, cut id, instances, volumes with their
//    matrices, sources, mesh digests, configs and paint data) and the plates.
//  - store: two models built in code are written with store_bbs_3mf (SaveStrategy::Zip64, and
//    SplitModel | Zip64) and the archive is dumped: the entry names (sorted: the object files of
//    the Production Extension reach the archive in the order the tbb tasks finish) and the text
//    of every XML / JSON / config / txt entry, normalised (dates, part GUIDs, the instance IDs
//    written as identify_id made relative to the model's ID, the object blocks of
//    model_settings.config sorted by id: the exporter orders them by object address). Each archive is then read back with load_bbs_3mf and dumped
//    as in the first part.
//
// Line format: tokens separated by one space; a double is its 16 hexadecimal digits, a float its
// 8, an integer decimal, a text the hexadecimal of its bytes ("-" when empty). A config is its
// key count and an FNV-1a digest of its "key=serialized value\n" lines. Entry texts are written
// between a line "ENTRY <name> <bytes>" and a line "@@END@@".

#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/Semver.hpp"
#include "libslic3r/Utils.hpp"

#include <miniz.h>
#include <boost/log/core.hpp>
#include <boost/filesystem.hpp>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

using namespace Slic3r;

static std::string hD(double v)
{
    uint64_t u;
    std::memcpy(&u, &v, 8);
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long) u);
    return b;
}
static std::string hF(float v)
{
    uint32_t u;
    std::memcpy(&u, &v, 4);
    char b[16];
    std::snprintf(b, sizeof(b), "%08x", u);
    return b;
}
static std::string hI(long long v) { return std::to_string(v); }
static std::string hB(bool v) { return v ? "1" : "0"; }
static std::string hT(const std::string &s)
{
    if (s.empty()) return "-";
    std::string o;
    char b[4];
    for (unsigned char c : s) {
        std::snprintf(b, sizeof(b), "%02x", c);
        o += b;
    }
    return o;
}
static uint64_t fnv(const std::string &s)
{
    uint64_t h = 1469598103934665603ULL;
    for (unsigned char c : s) {
        h ^= c;
        h *= 1099511628211ULL;
    }
    return h;
}
static std::string hX(uint64_t v)
{
    char b[32];
    std::snprintf(b, sizeof(b), "%016llx", (unsigned long long) v);
    return b;
}
static std::string hM(const Transform3d &t)
{
    std::string o;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) o += (o.empty() ? "" : " ") + hD(t.matrix()(r, c));
    return o;
}
static std::string V3(const Vec3d &v) { return hD(v.x()) + " " + hD(v.y()) + " " + hD(v.z()); }
// FNV-1a over the float bits of the vertices and the indices
static std::string ITS(const indexed_triangle_set &its)
{
    uint64_t h = 1469598103934665603ULL;
    auto mix = [&h](uint32_t v) {
        for (int k = 0; k < 4; ++k) {
            h ^= (v >> (8 * k)) & 0xff;
            h *= 1099511628211ULL;
        }
    };
    for (const Vec3f &v : its.vertices)
        for (int k = 0; k < 3; ++k) {
            uint32_t u;
            float f = v(k);
            std::memcpy(&u, &f, 4);
            mix(u);
        }
    for (const Vec3i &f : its.indices)
        for (int k = 0; k < 3; ++k) mix(uint32_t(f(k)));
    return hI(its.vertices.size()) + " " + hI(its.indices.size()) + " " + hX(h);
}
static std::string CFG(const DynamicPrintConfig &c)
{
    std::string s;
    t_config_option_keys keys = c.keys();
    for (const std::string &k : keys) s += k + "=" + c.opt_serialize(k) + "\n";
    return hI(keys.size()) + " " + hX(fnv(s));
}
static std::string FACETS(const FacetsAnnotation &f)
{
    const auto &d = f.get_data();
    std::string s;
    for (const auto &p : d.first) s += std::to_string(p.first) + "," + std::to_string(p.second) + ";";
    s += "|";
    for (bool b : d.second) s += b ? '1' : '0';
    return hI(d.first.size()) + " " + hI(d.second.size()) + " " + hX(fnv(s));
}
static std::string basename_of(const std::string &p)
{
    std::string q = p;
    std::replace(q.begin(), q.end(), '\\', '/');
    size_t pos = q.find_last_of('/');
    return pos == std::string::npos ? q : q.substr(pos + 1);
}
static std::string norm_path(const std::string &p, const std::string &backup)
{
    if (!backup.empty() && p.compare(0, backup.size(), backup) == 0) return "<backup>" + p.substr(backup.size());
    return p;
}

// id_base >= 0: the loaded identify IDs are instance IDs of a model of this process (an archive
// stored by the store part), written relative to that model's ID.
static std::string LOADED(size_t id, long long id_base)
{
    if (id_base < 0 || id == 0) return hI((long long) id); // 0: no identify ID was loaded
    return "@" + hI((long long) id - id_base);
}

static void dump_model(std::ostream &out, Model &model, PlateDataPtrs &plates, std::vector<Preset *> &presets, DynamicPrintConfig &config,
                       bool normalize_guid, long long id_base)
{
    const std::string backup = model.get_backup_path();
    out << "CFG " << CFG(config) << "\n";
    for (const Preset *p : presets) out << "PRESET " << int(p->type) << " " << hT(p->name) << " " << CFG(p->config) << " " << hT(p->version.to_string()) << "\n";
    if (model.model_info) {
        const ModelInfo &mi = *model.model_info;
        out << "MI " << hT(mi.model_name) << " " << hT(mi.origin) << " " << hT(mi.description) << " " << hT(mi.license) << " " << hT(mi.copyright)
            << " " << hT(mi.cover_file) << "\n";
        for (const auto &kv : mi.metadata_items) out << "MD " << hT(kv.first) << " " << hT(kv.second) << "\n";
    } else
        out << "MI none\n";
    if (model.design_info)
        out << "DI " << hT(model.design_info->Designer) << " " << hT(model.design_info->DesignerUserId) << " " << hT(model.design_info->DesignerId) << "\n";
    else
        out << "DI none\n";
    if (model.profile_info)
        out << "PI " << hT(model.profile_info->ProfileTile) << " " << hT(model.profile_info->ProfileCover) << " " << hT(model.profile_info->ProfileDescription)
            << " " << hT(model.profile_info->ProfileUserId) << " " << hT(model.profile_info->ProfileUserName) << " " << hT(model.design_id) << "\n";
    else
        out << "PI none\n";
    out << "OBJECTS " << model.objects.size() << "\n";
    for (size_t oi = 0; oi < model.objects.size(); ++oi) {
        const ModelObject *o = model.objects[oi];
        out << "O " << oi << " " << hT(o->name) << " " << hT(o->module_name) << " " << o->instances.size() << " " << o->volumes.size() << " "
            << CFG(o->config.get()) << "\n";
        for (const std::string &k : o->config.keys()) out << "OC " << k << " " << hT(o->config.opt_serialize(k)) << "\n";
        {
            std::vector<double> lhp = o->layer_height_profile.get();
            out << "LHP " << lhp.size();
            for (double v : lhp) out << " " << hD(v);
            out << "\n";
        }
        for (const auto &r : o->layer_config_ranges) out << "LCR " << hD(r.first.first) << " " << hD(r.first.second) << " " << CFG(r.second.get()) << "\n";
        out << "BP " << o->brim_points.size();
        for (const BrimPoint &p : o->brim_points)
            out << " " << hF(p.pos.x()) << " " << hF(p.pos.y()) << " " << hF(p.pos.z()) << " " << hF(p.head_front_radius) << " " << p.volume_idx;
        out << "\n";
        out << "CUT " << o->cut_id.id().id << " " << o->cut_id.check_sum() << " " << o->cut_id.connectors_cnt() << "\n";
        for (size_t ii = 0; ii < o->instances.size(); ++ii) {
            ModelInstance *i = o->instances[ii];
            out << "I " << ii << " " << hB(i->printable) << " " << LOADED(i->loaded_id, id_base) << " " << hM(i->get_matrix()) << " " << hB(i->is_assemble_initialized()) << " "
                << V3(i->get_offset_to_assembly()) << "\n";
        }
        for (size_t vi = 0; vi < o->volumes.size(); ++vi) {
            const ModelVolume *v = o->volumes[vi];
            std::string guid = v->part_guid();
            if (normalize_guid && !guid.empty()) guid = "<guid>";
            out << "V " << vi << " " << hT(v->name) << " " << int(v->type()) << " " << hT(guid) << " " << hT(basename_of(v->source.input_file)) << " "
                << v->source.object_idx << " " << v->source.volume_idx << " " << V3(v->source.mesh_offset) << " " << hB(v->source.is_converted_from_inches)
                << " " << hB(v->source.is_converted_from_meters) << " " << hM(v->source.transform.get_matrix()) << "\n";
            out << "VM " << hM(v->get_matrix()) << " " << hB(v->is_assemble_initialized()) << "\n";
            const RepairedMeshErrors &e = v->mesh().stats().repaired_errors;
            out << "VMESH " << ITS(v->mesh().its) << " " << e.edges_fixed << " " << e.degenerate_facets << " " << e.facets_removed << " " << e.facets_reversed
                << " " << e.backwards_edges << " " << V3(v->mesh().get_init_shift()) << "\n";
            out << "VC " << CFG(v->config.get()) << "\n";
            for (const std::string &k : v->config.keys()) out << "VCK " << k << " " << hT(v->config.opt_serialize(k)) << "\n";
            out << "VF " << FACETS(v->supported_facets) << " " << FACETS(v->seam_facets) << " " << FACETS(v->mmu_segmentation_facets) << " "
                << FACETS(v->fuzzy_skin_facets) << "\n";
            out << "VT " << hT(v->get_text_info().m_text) << " " << hT(v->get_text_info().m_font_name) << " " << hB(v->emboss_shape.has_value()) << "\n";
            out << "VCUT " << hB(v->cut_info.is_connector) << " " << int(v->cut_info.connector_type) << " " << hF(v->cut_info.radius) << " "
                << hF(v->cut_info.height) << " " << hF(v->cut_info.radius_tolerance) << " " << hF(v->cut_info.height_tolerance) << "\n";
        }
    }
    out << "PLATES " << plates.size() << "\n";
    for (size_t pi = 0; pi < plates.size(); ++pi) {
        const PlateData *p = plates[pi];
        out << "P " << pi << " " << p->plate_index << " " << hT(p->plate_name) << " " << hB(p->locked) << "\n";
        for (const auto &kv : p->obj_inst_map) out << "POI " << kv.first << " " << kv.second.first << " " << LOADED(kv.second.second, id_base) << "\n";
        out << "PF " << hT(norm_path(p->gcode_file, backup)) << " " << hT(norm_path(p->thumbnail_file, backup)) << " "
            << hT(norm_path(p->no_light_thumbnail_file, backup)) << " " << hT(norm_path(p->top_file, backup)) << " " << hT(norm_path(p->pick_file, backup))
            << " " << hT(norm_path(p->pattern_bbox_file, backup)) << "\n";
        out << "PS " << hT(p->gcode_prediction) << " " << hT(p->gcode_weight) << " " << hB(p->toolpath_outside) << " " << hB(p->is_support_used) << " "
            << hB(p->is_label_object_enabled) << " " << hB(p->support_material_on_wipe_tower) << " " << hT(p->printer_model_id) << " "
            << hT(p->nozzle_diameters) << " " << hT(p->nozzle_volume_types) << "\n";
        for (const FilamentInfo &f : p->slice_filaments_info) {
            out << "PFIL " << f.id << " " << hT(f.type) << " " << hT(f.color) << " " << hT(f.filament_id) << " " << hF(f.used_m) << " " << hF(f.used_g) << " "
                << f.group_id.size();
            for (int g : f.group_id) out << " " << g;
            out << " " << hD(f.nozzle_diameter) << " " << hT(f.nozzle_volume_type) << " " << hB(f.used_for_object) << " " << hB(f.used_for_support) << " "
                << hB(f.directly_used) << " " << hD(f.total_load_time) << " " << hD(f.total_unload_time) << "\n";
        }
        for (const auto &w : p->warnings) out << "PW " << hT(w.msg) << " " << w.level << "\n";
        out << "PMAP " << p->filament_maps.size();
        for (int v : p->filament_maps) out << " " << v;
        out << " | " << p->filament_change_sequence.size();
        for (unsigned v : p->filament_change_sequence) out << " " << v;
        out << " | " << p->nozzle_change_sequence.size();
        for (unsigned v : p->nozzle_change_sequence) out << " " << v;
        out << " | " << p->optimal_assignment.size();
        for (int v : p->optimal_assignment) out << " " << v;
        out << "\n";
        for (const auto &n : p->nozzles_info) out << "PNOZ " << n.group_id << " " << n.extruder_id << " " << hT(n.diameter) << " " << int(n.volume_type) << "\n";
        out << "PCFG " << CFG(p->config) << "\n";
        for (const std::string &k : p->config.keys()) out << "PCK " << k << " " << hT(p->config.opt_serialize(k)) << "\n";
    }
    for (const auto &kv : model.plates_custom_gcodes) {
        out << "CG " << kv.first << " " << int(kv.second.mode) << " " << kv.second.gcodes.size() << "\n";
        for (const CustomGCode::Item &it : kv.second.gcodes)
            out << "CGI " << hD(it.print_z) << " " << int(it.type) << " " << it.extruder << " " << hT(it.color) << " " << hT(it.extra) << "\n";
    }
    out << "ASM " << hX(fnv(model.get_assembly_tree_json_str())) << " " << hX(fnv(model.get_assembly_steps_json_str())) << " "
        << hX(fnv(model.get_assembly_model_json_str())) << "\n";
}

static void load_and_dump(std::ostream &out, const std::string &label, const std::string &path, bool normalize_guid, long long id_base)
{
    out << "FILE " << label << "\n";
    Model model;
    DynamicPrintConfig config;
    ConfigSubstitutionContext subs(ForwardCompatibilitySubstitutionRule::Enable);
    PlateDataPtrs plates;
    std::vector<Preset *> presets;
    bool is_bbl = false;
    Semver file_version;
    bool res = false;
    try {
        res = load_bbs_3mf(path.c_str(), &config, &subs, &model, &plates, &presets, &is_bbl, &file_version, nullptr,
                           LoadStrategy::LoadModel | LoadStrategy::LoadConfig, nullptr, 0);
    } catch (const std::exception &e) {
        out << "EXCEPTION " << hT(e.what()) << "\n";
    }
    out << "R " << hB(res) << " " << hB(is_bbl) << " " << hT(file_version.to_string()) << "\n";
    dump_model(out, model, plates, presets, config, normalize_guid, id_base);
    release_PlateData_list(plates);
    for (Preset *p : presets) delete p;
}

// ---------------------------------------------------------------------------------------------
// store
// ---------------------------------------------------------------------------------------------

static bool is_text_entry(const std::string &name)
{
    auto ends = [&name](const char *s) {
        size_t n = std::strlen(s);
        return name.size() >= n && name.compare(name.size() - n, n, s) == 0;
    };
    return ends(".xml") || ends(".model") || ends(".config") || ends(".rels") || ends(".json") || ends(".txt");
}

// relative object IDs: the number after `prefix` becomes (number - base)
static void relative_ids(std::string &s, const std::string &prefix, size_t base)
{
    size_t pos = 0;
    while ((pos = s.find(prefix, pos)) != std::string::npos) {
        size_t start = pos + prefix.size();
        size_t end = start;
        while (end < s.size() && isdigit((unsigned char) s[end])) ++end;
        if (end > start) {
            long long v = std::stoll(s.substr(start, end - start));
            std::string r = "@" + std::to_string(v - (long long) base);
            s.replace(start, end - start, r);
            pos = start + r.size();
        } else
            pos = start;
    }
}

static void replace_between(std::string &s, const std::string &open, const std::string &close, const std::string &with)
{
    size_t pos = 0;
    while ((pos = s.find(open, pos)) != std::string::npos) {
        size_t start = pos + open.size();
        size_t end = s.find(close, start);
        if (end == std::string::npos) break;
        s.replace(start, end - start, with);
        pos = start + with.size() + close.size();
    }
}

// The objects of model_settings.config come in the order of the exporter's
// std::map<const ModelObject*, ObjectData>, i.e. by the addresses of the objects: the
// "  <object id=..." blocks are sorted by their id.
static std::string sort_object_blocks(const std::string &s)
{
    const std::string open = "  <object id=\"", close = "  </object>\n";
    size_t first = s.find(open);
    if (first == std::string::npos) return s;
    std::vector<std::pair<long long, std::string>> blocks;
    size_t pos = first;
    while (s.compare(pos, open.size(), open) == 0) {
        size_t end = s.find(close, pos);
        if (end == std::string::npos) return s;
        end += close.size();
        blocks.emplace_back(std::stoll(s.substr(pos + open.size())), s.substr(pos, end - pos));
        pos = end;
    }
    std::stable_sort(blocks.begin(), blocks.end(), [](const auto &a, const auto &b) { return a.first < b.first; });
    std::string out = s.substr(0, first);
    for (const auto &b : blocks) out += b.second;
    return out + s.substr(pos);
}

static std::string normalize_entry(std::string s, size_t base)
{
    s = sort_object_blocks(s);
    replace_between(s, "<metadata name=\"CreationDate\">", "</metadata>", "DATE");
    replace_between(s, "<metadata name=\"ModificationDate\">", "</metadata>", "DATE");
    replace_between(s, " uuid=\"", "\"", "UUID");
    relative_ids(s, "key=\"identify_id\" value=\"", base);
    relative_ids(s, "identify_id=\"", base);
    return s;
}

static void dump_archive(std::ostream &out, const std::string &path, size_t base)
{
    mz_zip_archive archive;
    mz_zip_zero_struct(&archive);
    if (!mz_zip_reader_init_file(&archive, path.c_str(), 0)) {
        out << "ARCHIVE none\n";
        return;
    }
    std::vector<std::pair<std::string, std::string>> entries;
    mz_uint n = mz_zip_reader_get_num_files(&archive);
    for (mz_uint i = 0; i < n; ++i) {
        mz_zip_archive_file_stat stat;
        if (!mz_zip_reader_file_stat(&archive, i, &stat)) continue;
        std::string data(stat.m_uncomp_size, 0);
        mz_zip_reader_extract_to_mem(&archive, i, data.data(), data.size(), 0);
        entries.emplace_back(stat.m_filename, data);
    }
    mz_zip_reader_end(&archive);
    std::sort(entries.begin(), entries.end(), [](const auto &a, const auto &b) { return a.first < b.first; });
    out << "ARCHIVE " << entries.size() << "\n";
    for (auto &e : entries) {
        if (is_text_entry(e.first)) {
            std::string t = normalize_entry(e.second, base);
            out << "ENTRY " << e.first << " " << t.size() << "\n" << t << "\n@@END@@\n";
        } else
            out << "BINARY " << e.first << " " << e.second.size() << "\n";
    }
}

static TriangleMesh cube(double x, double y, double z) { return TriangleMesh(its_make_cube(x, y, z)); }

static void build_model(Model &model, PlateDataPtrs &plates)
{
    ModelObject *o1 = model.add_object();
    o1->name = "cube_a";
    ModelVolume *v1 = o1->add_volume(cube(10., 10., 10.));
    v1->name = "cube_a_part";
    v1->config.set_key_value("extruder", new ConfigOptionInt(2));
    o1->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(25));
    o1->config.set_key_value("extruder", new ConfigOptionInt(1));
    ModelInstance *i1 = o1->add_instance();
    i1->set_offset(Vec3d(50., 60., 5.));
    ModelInstance *i2 = o1->add_instance();
    i2->set_offset(Vec3d(120., 60., 5.));
    i2->set_rotation(Vec3d(0., 0., 0.5));
    o1->layer_height_profile.set({0., 0.2, 10., 0.2});
    {
        DynamicPrintConfig cfg;
        cfg.set_key_value("layer_height", new ConfigOptionFloat(0.1));
        o1->layer_config_ranges[{2.0, 4.0}].assign_config(std::move(cfg));
    }
    o1->brim_points.emplace_back(1.f, 2.f, 0.f, 0.5f, 0);
    o1->brim_points.emplace_back(-3.25f, 4.5f, 0.f, 0.75f, 0);
    v1->mmu_segmentation_facets.set_triangle_from_string(0, "8");
    v1->mmu_segmentation_facets.set_triangle_from_string(3, "C");
    v1->supported_facets.set_triangle_from_string(1, "4");
    v1->seam_facets.set_triangle_from_string(2, "8");

    ModelObject *o2 = model.add_object();
    o2->name = "two & parts";
    o2->module_name = "mod";
    ModelVolume *v2 = o2->add_volume(cube(5., 6., 7.));
    v2->name = "part \"main\"";
    v2->source.input_file = "/some/dir/part.stl";
    v2->source.object_idx = 0;
    v2->source.volume_idx = 1;
    v2->source.mesh_offset = Vec3d(1., 2., 3.);
    ModelVolume *v3 = o2->add_volume(cube(2., 2., 2.), ModelVolumeType::PARAMETER_MODIFIER);
    v3->name = "modifier";
    v3->config.set_key_value("wall_loops", new ConfigOptionInt(3));
    v3->set_offset(Vec3d(1.5, -0.5, 2.));
    o2->config.set_key_value("extruder", new ConfigOptionInt(2));
    ModelInstance *i3 = o2->add_instance();
    i3->set_offset(Vec3d(100., 100., 0.));
    i3->printable = true;

    model.plates_custom_gcodes[0].mode = CustomGCode::Mode::SingleExtruder;
    model.plates_custom_gcodes[0].gcodes.push_back(CustomGCode::Item{2.4, CustomGCode::ColorChange, 1, "#FF0000", ""});
    model.plates_custom_gcodes[0].gcodes.push_back(CustomGCode::Item{5.0, CustomGCode::Custom, 1, "", "G4 S1"});

    model.model_info = std::make_shared<ModelInfo>();
    model.model_info->model_name = "Test & <model>";
    model.model_info->description = "a description";
    model.model_info->license = "BY-SA";
    model.design_info = std::make_shared<ModelDesignInfo>();
    model.design_info->Designer = "alice";
    model.design_info->DesignerUserId = "42";

    PlateData *p = new PlateData();
    p->plate_index = 0;
    p->plate_name = "Plate One";
    p->locked = false;
    p->objects_and_instances = {{0, 0}, {0, 1}, {1, 0}};
    p->is_sliced_valid = true;
    p->gcode_prediction = "1234";
    p->gcode_weight = "5.67";
    p->first_layer_time = "88";
    FilamentInfo fi;
    fi.id = 0;
    fi.type = "PLA";
    fi.color = "#FFFFFF";
    fi.used_m = 1.234f;
    fi.used_g = 3.21f;
    fi.filament_id = "GFL99";
    fi.group_id = {0};
    fi.nozzle_diameter = 0.4;
    fi.nozzle_volume_type = "Standard";
    fi.used_for_object = true;
    p->slice_filaments_info.push_back(fi);
    GCodeProcessorResult::SliceWarning sw;
    sw.msg = "test_warning";
    sw.level = 1;
    sw.error_code = "1000C001";
    p->warnings.push_back(sw);
    p->filament_change_sequence = {0, 1, 0};
    p->nozzle_change_sequence = {0};
    p->config.set_key_value("curr_bed_type", new ConfigOptionEnum<BedType>(btPEI));
    p->config.set_key_value("print_sequence", new ConfigOptionEnum<PrintSequence>(PrintSequence::ByObject));
    p->config.set_key_value("first_layer_print_sequence", new ConfigOptionInts({1, 2}));
    p->config.set_key_value("spiral_mode", new ConfigOptionBool(false));
    p->config.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
    p->config.set_key_value("filament_map", new ConfigOptionInts({1, 2}));
    plates.push_back(p);
}

static void store_scenario(std::ostream &out, const std::string &label, const std::string &dir, SaveStrategy strategy)
{
    out << "STORE " << label << "\n";
    Model model;
    PlateDataPtrs plates;
    build_model(model, plates);
    // full_print_config() clones the static defaults, whose enum vectors have no key map (their
    // serialization crashes); new_from_defaults_keys creates the options from their definitions.
    DynamicPrintConfig *defaults = DynamicPrintConfig::new_from_defaults_keys(FullPrintConfig::defaults().keys());
    DynamicPrintConfig config = *defaults;
    delete defaults;
    std::string path = dir + "/" + label + ".3mf";
    StoreParams sp;
    sp.path = path.c_str();
    sp.model = &model;
    sp.plate_data_list = plates;
    sp.config = &config;
    sp.strategy = strategy;
    bool res = store_bbs_3mf(sp);
    out << "SR " << hB(res) << "\n";
    dump_archive(out, path, model.id().id);
    release_PlateData_list(plates);
    load_and_dump(out, label + " reload", path, true, (long long) model.id().id);
}

int main(int argc, char **argv)
{
    boost::log::core::get()->set_logging_enabled(false);
    if (argc < 3) {
        std::fprintf(stderr, "usage: gen <repo> <scratch dir> [a.3mf]\n");
        return 1;
    }
    const std::string repo = argv[1];
    const std::string dir = argv[2];
    // the backup directories of the models (Model::get_backup_path) are made under it
    set_temporary_dir(dir);
    std::ostream &out = std::cout;
    if (argc > 3) {
        load_and_dump(out, basename_of(argv[3]), argv[3], false, -1);
        return 0;
    }
    const char *files[] = {"crates/libslic3r-rs/tests/data/painted_cube.3mf", "tests/fixtures/calib_base.3mf",       "tests/fixtures/calib_base_orca.3mf",
                           "tests/fixtures/calib_base_snapmaker.3mf",          "tests/fixtures/legacy_token.3mf",     "tests/fixtures/legacy_token_orca.3mf",
                           "tests/fixtures/legacy_token_snapmaker.3mf"};
    for (const char *f : files) load_and_dump(out, f, repo + "/" + f, false, -1);
    store_scenario(out, "store_zip64", dir, SaveStrategy::Zip64);
    store_scenario(out, "store_split", dir, SaveStrategy::SplitModel | SaveStrategy::Zip64);
    return 0;
}
