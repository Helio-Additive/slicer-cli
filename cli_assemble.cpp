// cli_assemble.cpp — see cli_assemble.hpp.
#include "cli_assemble.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <optional>
#include <set>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
#include <boost/nowide/fstream.hpp>
#include <nlohmann/json.hpp>

#include "libslic3r/Format/OBJ.hpp"
#include "libslic3r/ObjColorUtils.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/Utils.hpp"

namespace slicer_cli {
namespace {

using json = nlohmann::json;
using namespace Slic3r;

// PartPlate.hpp 36 in both engines.
constexpr int kMaxPlateCount = 36;

StepResult fail(int code, std::string message) { return StepResult{code, std::move(message)}; }

// The colour distance of the desktop's GUI (GuiColor.cpp 33-40, identical in
// both engines): CIE76 between the two colours in Lab, with RGB2Lab and
// DeltaE76 from slic3r/Utils/ColorSpaceConvert.cpp (22-31, 79-111, 234-237).
// GUI code, so not in this build's libslic3r; ported here unchanged.
double pivot_rgb(double n) { return (n > 0.04045 ? std::pow((n + 0.055) / 1.055, 2.4) : n / 12.92) * 100.0; }
double pivot_xyz(double n) {
    double i = std::cbrt(n);
    return n > 0.008856 ? i : 7.787 * n + 16.0 / 116.0;
}
void rgb_to_xyz(float R, float G, float B, float* X, float* Y, float* Z) {
    R = pivot_rgb(R);
    G = pivot_rgb(G);
    B = pivot_rgb(B);
    *X = 0.412453f * R + 0.357580f * G + 0.180423f * B;
    *Y = 0.212671f * R + 0.715160f * G + 0.072169f * B;
    *Z = 0.019334f * R + 0.119193f * G + 0.950227f * B;
}
void xyz_to_lab(float X, float Y, float Z, float* L, float* a, float* b) {
    double REF_X = 95.047;
    double REF_Y = 100.000;
    double REF_Z = 108.883;
    double x = pivot_xyz(X / REF_X);
    double y = pivot_xyz(Y / REF_Y);
    double z = pivot_xyz(Z / REF_Z);
    *L = 116.0 * y - 16.0;
    *a = 500.0 * (x - y);
    *b = 200.0 * (y - z);
}
void rgb_to_lab(float R, float G, float B, float* L, float* a, float* b) {
    float X = 0.0f, Y = 0.0f, Z = 0.0f;
    rgb_to_xyz(R, G, B, &X, &Y, &Z);
    xyz_to_lab(X, Y, Z, L, a, b);
}
float delta_e76(float l1, float a1, float b1, float l2, float a2, float b2) {
    return std::sqrt(std::pow((l1 - l2), 2) + std::pow((a1 - a2), 2) + std::pow((b1 - b2), 2));
}
float calc_color_distance(RGBA c1, RGBA c2) {
    float lab[2][3];
    rgb_to_lab(c1[0], c1[1], c1[2], &lab[0][0], &lab[0][1], &lab[0][2]);
    rgb_to_lab(c2[0], c2[1], c2[2], &lab[1][0], &lab[1][1], &lab[1][2]);
    return delta_e76(lab[0][0], lab[0][1], lab[0][2], lab[1][0], lab[1][1], lab[1][2]);
}
struct ColorDistValue {   // GuiColor.hpp 7-11
    int   id;
    float distance;
};

// A per-copy list: `count` values, or one for every copy (B 807-846).
template<class T>
bool per_copy_ok(const std::vector<T>& values, int count) {
    return values.empty() || int(values.size()) == count || values.size() == 1;
}

AssembleHeightRange read_range(const json& j) {
    AssembleHeightRange range;
    range.min_z        = j.at("min_z").get<float>();
    range.max_z        = j.at("max_z").get<float>();
    range.range_params = j.at("range_params").get<std::map<std::string, std::string>>();
    return range;
}

// merge_or_add_object (BambuStudio.cpp 907-937; OrcaSlicer.cpp 765-793).
void merge_or_add_object(AssemblePlate& plate, Model& model, int assemble_index,
                         std::map<int, ModelObject*>& merged_objects, ModelObject* ori_object,
                         ModelVolumeType type) {
    if (assemble_index > 0) {
        auto iter = merged_objects.find(assemble_index);
        ModelObject* new_object = nullptr;
        if (iter == merged_objects.end()) {
            new_object = model.add_object();
            new_object->name = "assemble_" + std::to_string(assemble_index);
            merged_objects[assemble_index] = new_object;
            plate.loaded_obj_list.emplace_back(new_object);
#ifdef ENGINE_ORCA
            new_object->config.assign_config(ori_object->config.get());
#endif
        } else
            new_object = iter->second;

        for (ModelVolume* volume : ori_object->volumes) {
#ifdef ENGINE_ORCA
            (void)type;
            ModelVolume* new_volume = new_object->add_volume(*volume);
            // The object's filament; an OBJ whose colours set the filaments
            // per face has none (the official reads it unchecked).
            new_volume->config.set_key_value("extruder", new ConfigOptionInt(
                ori_object->config.has("extruder") ? ori_object->config.extruder() : 0));
#else
            ModelVolume* new_volume = new_object->add_volume(*volume, type);
            if (type == ModelVolumeType::MODEL_PART || type == ModelVolumeType::PARAMETER_MODIFIER)
                new_volume->config.apply(ori_object->config);
#endif
        }
    } else {
        ModelObject* new_object = model.add_object(*ori_object);
        plate.loaded_obj_list.emplace_back(new_object);
    }
}

// convert_obj_cluster_colors (BambuStudio.cpp 941-1038; OrcaSlicer.cpp
// 795-849): the OBJ's colours clustered into filaments, a colour seen before
// keeping its filament (BambuStudio only), and past the filament limit the
// nearest filament colour.
bool convert_obj_cluster_colors(std::vector<RGBA>& input_colors, std::vector<RGBA>& all_colours,
                                int max_filament_count, std::vector<unsigned char>& output_filament_ids,
                                int& first_filament_id, bool first_time_using_makerlab = false,
                                std::vector<RGBA> mtl_colors = {}) {
    if (input_colors.empty())
        return false;
    std::vector<RGBA> cluster_colors;
    std::vector<int>  cluster_labels;
    char              cluster_number = -1;
#ifdef ENGINE_ORCA
    (void)first_time_using_makerlab;
    (void)mtl_colors;
    obj_color_deal_algo(input_colors, cluster_colors, cluster_labels, cluster_number, (int) EnforcerBlockerType::ExtruderMax);
#else
    if (first_time_using_makerlab && mtl_colors.size() < (int) EnforcerBlockerType::ExtruderMax) {
        cluster_colors = mtl_colors;
        cluster_labels.clear();
        cluster_labels.reserve(input_colors.size());
        std::set<int> cluster_number_set;
        for (size_t i = 0; i < input_colors.size(); i++) {
            bool can_find = false;
            for (size_t j = 0; j < cluster_colors.size(); j++) {
                if (color_is_equal(input_colors[i], cluster_colors[j])) {
                    cluster_labels.emplace_back(int(j));
                    can_find = true;
                    cluster_number_set.insert(int(j));
                    break;
                }
            }
            if (!can_find)
                cluster_labels.emplace_back(0);
        }
        cluster_number = char(cluster_number_set.size());
    } else {
        obj_color_deal_algo(input_colors, cluster_colors, cluster_labels, cluster_number, (int) EnforcerBlockerType::ExtruderMax);
    }
#endif
    std::vector<int> cluster_color_maps(cluster_colors.size(), 1);
    const int init_size = int(all_colours.size());
    first_filament_id = max_filament_count;
    for (size_t i = 0; i < cluster_colors.size(); i++) {
#ifndef ENGINE_ORCA
        auto previous_color = std::find(all_colours.begin(), all_colours.end(), cluster_colors[i]);
        if (previous_color != all_colours.end()) {
            cluster_color_maps[i] = int(previous_color - all_colours.begin()) + 1;
        } else
#endif
        if ((init_size + int(i) + 1) <= max_filament_count) {
            all_colours.push_back(cluster_colors[i]);
            cluster_color_maps[i] = int(all_colours.size());
        } else {
            // The nearest colour already given a filament. The official scans
            // max_filament_count entries (BambuStudio.cpp 1008-1013), but a
            // cluster that matched an earlier colour added none, so fewer may
            // exist and it reads past the end; only the colours there are
            // compared. The capacity rule above is the official one.
            const size_t known = std::min(all_colours.size(), size_t(max_filament_count));
            if (known == 0) {
                all_colours.push_back(cluster_colors[i]);
                cluster_color_maps[i] = int(all_colours.size());
            } else {
                std::vector<ColorDistValue> color_dists(known);
                for (size_t j = 0; j < known; j++) {
                    color_dists[j].distance = calc_color_distance(cluster_colors[i], all_colours[j]);
                    color_dists[j].id       = int(j) + 1;
                }
                std::sort(color_dists.begin(), color_dists.end(),
                          [](const ColorDistValue& a, const ColorDistValue& b) { return a.distance < b.distance; });
                cluster_color_maps[i] = color_dists[0].id;
            }
        }
        if (cluster_color_maps[i] < first_filament_id)
            first_filament_id = cluster_color_maps[i];
    }
    output_filament_ids.resize(input_colors.size());
    for (size_t i = 0; i < input_colors.size(); i++)
        output_filament_ids[i] = (unsigned char) cluster_color_maps[cluster_labels[i]];
    return true;
}

void apply_params(ModelConfigObject& target, const std::map<std::string, std::string>& params,
                  ConfigSubstitutionContext& subst) {
    for (const auto& [key, value] : params)
        target.set_deserialize(key, value, subst);
}

void apply_ranges(ModelObject* object, const std::vector<AssembleHeightRange>& ranges, ConfigSubstitutionContext& subst) {
    for (const AssembleHeightRange& range : ranges) {
        DynamicPrintConfig range_config;
        for (const auto& [key, value] : range.range_params)
            range_config.set_deserialize(key, value, subst);
        object->layer_config_ranges[{range.min_z, range.max_z}].assign_config(std::move(range_config));
    }
}

#ifdef _WIN32
constexpr char kDirSeparator = '\\';
#else
constexpr char kDirSeparator = '/';
#endif

} // namespace

StepResult load_assemble_plate_list(const std::string& file, std::vector<AssemblePlate>& plates) {
    if (!boost::filesystem::exists(boost::filesystem::path(file)))
        return fail(CLI_FILE_NOTFOUND, "The assemble list " + file + " does not exist; check the path given to --load-assemble-list");
    try {
        json root;
        boost::nowide::ifstream ifs(file);
        ifs >> root;
        ifs.close();

        const json& plates_json = root.at("plates");
        const int plate_count = int(plates_json.size());
        if (plate_count <= 0 || plate_count > kMaxPlateCount)
            return fail(CLI_CONFIG_FILE_ERROR, "The assemble list has " + std::to_string(plate_count) +
                                                   " plates; it takes 1 to " + std::to_string(kMaxPlateCount) + ".");
        plates.resize(plate_count);
        for (int plate_index = 0; plate_index < plate_count; plate_index++) {
            AssemblePlate& plate = plates[plate_index];
            const json& plate_json = plates_json[plate_index];
            const std::string where = "plate " + std::to_string(plate_index + 1);
            plate.plate_name   = plate_json.at("plate_name").get<std::string>();
            plate.need_arrange = plate_json.at("need_arrange").get<bool>();
            if (plate_json.contains("plate_params"))
                plate.plate_params = plate_json["plate_params"].get<std::map<std::string, std::string>>();

            const json& objects_json = plate_json.at("objects");
            const int object_count = int(objects_json.size());
            if (object_count <= 0)
                return fail(CLI_CONFIG_FILE_ERROR, "The assemble list's " + where + " has no objects; give the plate at least one object");
            plate.objects.resize(object_count);
            for (int object_index = 0; object_index < object_count; object_index++) {
                AssembleObject& object = plate.objects[object_index];
                const json& object_json = objects_json[object_index];
                object.path  = object_json.at("path").get<std::string>();
                object.count = object_json.at("count").get<int>();
                const std::string what = "object " + object.path + " on " + where;
                if (object.count <= 0)
                    return fail(CLI_CONFIG_FILE_ERROR, "The assemble list's " + what + " has a count of " +
                                                           std::to_string(object.count) + "; it needs 1 or more.");
#ifndef ENGINE_ORCA
                // B 802-805; OrcaSlicer's list has no subtype.
                if (object_json.contains("subtype"))
                    object.subtype = ModelVolume::type_from_string(object_json["subtype"].get<std::string>());
                else
                    object.subtype = ModelVolumeType::MODEL_PART;
#endif
                auto list_refusal = [&](const char* key, size_t size) {
                    return fail(CLI_CONFIG_FILE_ERROR, "The assemble list's " + what + " gives " + std::to_string(size) +
                                                           " " + key + " for " + std::to_string(object.count) +
                                                           " copies; give one for each copy or one for all.");
                };
                object.filaments = object_json.at("filaments").get<std::vector<int>>();
                if (!per_copy_ok(object.filaments, object.count))
                    return list_refusal("filaments", object.filaments.size());
                if (object_json.contains("assemble_index")) {
                    object.assemble_index = object_json["assemble_index"].get<std::vector<int>>();
                    if (!per_copy_ok(object.assemble_index, object.count))
                        return list_refusal("assemble_index", object.assemble_index.size());
                }
                for (const char* key : {"pos_x", "pos_y", "pos_z"}) {
                    if (!object_json.contains(key)) continue;
                    std::vector<float>& values = key[4] == 'x' ? object.pos_x : key[4] == 'y' ? object.pos_y : object.pos_z;
                    values = object_json[key].get<std::vector<float>>();
                    if (!per_copy_ok(values, object.count))
                        return list_refusal(key, values.size());
                }
                if (object_json.contains("print_params"))
                    object.print_params = object_json["print_params"].get<std::map<std::string, std::string>>();
                if (object_json.contains("height_ranges"))
                    for (const json& range : object_json["height_ranges"])
                        object.height_ranges.push_back(read_range(range));
            }
            if (plate_json.contains("assembled_params")) {
                for (const json& entry : plate_json["assembled_params"]) {
                    AssembledParams params;
                    const int assemble_index = entry.at("assemble_index").get<int>();
                    if (entry.contains("print_params"))
                        params.print_params = entry["print_params"].get<std::map<std::string, std::string>>();
                    if (entry.contains("height_ranges"))
                        for (const json& range : entry["height_ranges"])
                            params.height_ranges.push_back(read_range(range));
                    plate.assembled_params.emplace(assemble_index, std::move(params));
                }
            }
        }
    } catch (const std::exception& e) {
        return fail(CLI_CONFIG_FILE_ERROR, "The assemble list " + file + " could not be read: " + e.what() +
                                               "; check the file");
    }
    return {};
}

StepResult construct_assemble_list(std::vector<AssemblePlate>& plates, Model& model,
                                   std::vector<DynamicPrintConfig>& plate_configs, std::vector<RGBA>& all_colours) {
    ConfigSubstitutionContext config_substitutions(ForwardCompatibilitySubstitutionRule::Enable);
    Model temp_model;
    const int max_filament_count = int(size_t(EnforcerBlockerType::ExtruderMax));
    struct TempRelease {
        Model& m;
        ~TempRelease() { m.clear_objects(); m.clear_materials(); }
    } temp_release{temp_model};

    plate_configs.assign(plates.size(), DynamicPrintConfig());
    for (size_t index = 0; index < plates.size(); index++) {
        std::map<int, ModelObject*> merged_objects;
        std::set<int> used_filaments;
        AssemblePlate& plate = plates[index];
        const std::string where = "plate " + std::to_string(index + 1);

        for (const auto& [key, value] : plate.plate_params)
            plate_configs[index].set_deserialize(key, value, config_substitutions);

        for (size_t obj_index = 0; obj_index < plate.objects.size(); obj_index++) {
            AssembleObject& assemble_object = plate.objects[obj_index];
            std::string object_name, object_1_name;
            ModelObject* object = nullptr;
            TriangleMesh mesh;
            bool skip_filament = false;
            const std::string what = assemble_object.path + " (" + where + ", object " + std::to_string(obj_index + 1) + ")";

            if (!boost::filesystem::exists(boost::filesystem::path(assemble_object.path)))
                return fail(CLI_FILE_NOTFOUND, "The assemble list names " + what + ", which does not exist; check the path");

            const char* path_str = assemble_object.path.c_str();
            const char* last_slash = std::strrchr(path_str, kDirSeparator);
            object_name.assign(last_slash == nullptr ? path_str : last_slash + 1);

            if (boost::algorithm::iends_with(assemble_object.path, ".stl")) {
                if (!mesh.ReadSTLFile(path_str, true, nullptr))
                    return fail(CLI_DATA_FILE_ERROR, "The STL " + what + " could not be read; check the file");
                if (mesh.empty())
                    return fail(CLI_DATA_FILE_ERROR, "The STL " + what + " holds no mesh; check the file");
                object_name.erase(object_name.end() - 4, object_name.end());
                object_1_name = object_name + "_1";
                object = temp_model.add_object(object_1_name.c_str(), path_str, std::move(mesh));
                if (!object)
                    return fail(CLI_DATA_FILE_ERROR, "The STL " + what + " could not be added; check the file");
            }
#ifdef ENGINE_ORCA
            else if (boost::algorithm::iends_with(assemble_object.path, ".obj"))
#else
            else if (boost::algorithm::iends_with(assemble_object.path, ".obj") &&
                     assemble_object.subtype == ModelVolumeType::MODEL_PART)
#endif
            {
                std::string message;
                ObjInfo obj_info;
                bool result = load_obj(path_str, &mesh, obj_info, message);
                if (!result)
                    return fail(CLI_DATA_FILE_ERROR, "The OBJ " + what + " holds no usable mesh: " + message + "; check the file");
                if (!obj_info.lost_material_name.empty())
                    return fail(CLI_DATA_FILE_ERROR, "The OBJ " + what + " uses the material " +
                                                         obj_info.lost_material_name +
                                                         ", which its MTL file lacks; check the file");
                if (!obj_info.face_colors.empty() && obj_info.face_colors.size() < mesh.facets_count())
                    return fail(CLI_DATA_FILE_ERROR, "The OBJ " + what + " gives some faces no colour; check its MTL and OBJ files.");
                object_name.erase(object_name.end() - 4, object_name.end());
                object_1_name = object_name + "_1";
                Model obj_temp_model;
                ModelObject* temp_object = obj_temp_model.add_object(object_1_name.c_str(), path_str, std::move(mesh));
                if (!temp_object)
                    return fail(CLI_DATA_FILE_ERROR, "The OBJ " + what + " could not be added; check the file");
                std::vector<unsigned char> output_filament_ids;
                int first_filament_id = 0;
#ifdef ENGINE_ORCA
                if (!obj_info.vertex_colors.empty()) {
                    convert_obj_cluster_colors(obj_info.vertex_colors, all_colours, max_filament_count, output_filament_ids, first_filament_id);
                    if (!output_filament_ids.empty()) {
                        unsigned char first_extruder_id = output_filament_ids.front();
                        result = Model::obj_import_vertex_color_deal(output_filament_ids, first_extruder_id, &obj_temp_model);
                    }
                    skip_filament = true;
                } else if (!obj_info.face_colors.empty() && !obj_info.has_uv_png) {
                    convert_obj_cluster_colors(obj_info.face_colors, all_colours, max_filament_count, output_filament_ids, first_filament_id);
                    if (!output_filament_ids.empty()) {
                        unsigned char first_extruder_id = output_filament_ids.front();
                        result = Model::obj_import_face_color_deal(output_filament_ids, first_extruder_id, &obj_temp_model);
                    }
                    skip_filament = true;
                }
#else
                if (!obj_info.vertex_colors.empty()) {
                    convert_obj_cluster_colors(obj_info.vertex_colors, all_colours, max_filament_count, output_filament_ids, first_filament_id);
                    if (!output_filament_ids.empty())
                        result = Model::obj_import_color_deal(output_filament_ids, std::optional<unsigned char>(first_filament_id),
                                                              &obj_temp_model, [](int) { return true; });
                    skip_filament = true;
                } else if (!obj_info.face_colors.empty() && !obj_info.has_uv_png) {
                    convert_obj_cluster_colors(obj_info.face_colors, all_colours, max_filament_count, output_filament_ids, first_filament_id,
                                               obj_info.first_time_using_makerlab, obj_info.mtl_colors);
                    if (!output_filament_ids.empty())
                        result = Model::obj_import_color_deal(output_filament_ids, std::optional<unsigned char>(first_filament_id),
                                                              &obj_temp_model, [](int) { return false; });
                    skip_filament = true;
                }
#endif
                if (!result)
                    return fail(CLI_DATA_FILE_ERROR, "The colours of the OBJ " + what +
                                                         " could not be turned into filaments; check the file");
                object = temp_model.add_object(*temp_object);
                obj_temp_model.clear_objects();
                obj_temp_model.clear_materials();
                if (!object)
                    return fail(CLI_DATA_FILE_ERROR, "The OBJ " + what + " could not be added; check the file");
            } else {
                return fail(CLI_INVALID_PARAMS, "The assemble list names " + what +
                                                    "; it takes STL files, and OBJ files as normal parts.");
            }

            if (!skip_filament) {
                if (assemble_object.filaments.empty())
                    return fail(CLI_CONFIG_FILE_ERROR, "The assemble list gives " + what + " no filament; give a filament for it");
                object->config.set_key_value("extruder", new ConfigOptionInt(assemble_object.filaments[0]));
                used_filaments.emplace(assemble_object.filaments[0]);
            } else {
                if (assemble_object.filaments.empty())
                    assemble_object.filaments.resize(1, 0);
                assemble_object.filaments[0] = 0;
                for (const ModelVolume* mv : object->volumes) {
                    std::vector<int> volume_extruders = mv->get_extruders();
                    used_filaments.insert(volume_extruders.begin(), volume_extruders.end());
                }
            }

            apply_params(object->config, assemble_object.print_params, config_substitutions);

            if (!assemble_object.height_ranges.empty()) {
#ifndef ENGINE_ORCA
                if (assemble_object.subtype != ModelVolumeType::MODEL_PART)
                    return fail(CLI_INVALID_PARAMS, "The assemble list gives " + what +
                                                        " height ranges; only a normal part can have them.");
#endif
                apply_ranges(object, assemble_object.height_ranges, config_substitutions);
            }

            if (assemble_object.pos_x.empty()) assemble_object.pos_x.resize(1, 0.f);
            if (assemble_object.pos_y.empty()) assemble_object.pos_y.resize(1, 0.f);
            if (assemble_object.pos_z.empty()) assemble_object.pos_z.resize(1, 0.f);
            if (assemble_object.assemble_index.empty()) {
#ifndef ENGINE_ORCA
                if (assemble_object.subtype != ModelVolumeType::MODEL_PART)
                    return fail(CLI_INVALID_PARAMS, "The assemble list gives " + what +
                                                        " no assemble_index; only a normal part can stand as its own object.");
#endif
                assemble_object.assemble_index.resize(1, 0);
            }
#ifndef ENGINE_ORCA
            if (assemble_object.subtype != ModelVolumeType::MODEL_PART && assemble_object.assemble_index[0] == 0)
                return fail(CLI_INVALID_PARAMS, "The assemble list gives " + what +
                                                    " assemble_index 0; only a normal part can stand as its own object.");
#endif

            object->translate(assemble_object.pos_x[0], assemble_object.pos_y[0], assemble_object.pos_z[0]);
            merge_or_add_object(plate, model, assemble_object.assemble_index[0], merged_objects, object, assemble_object.subtype);

            for (int copy_index = 1; copy_index < assemble_object.count; copy_index++) {
                size_t array_index = 0;
                ModelObject* copy_obj = temp_model.add_object(*object);
                copy_obj->name = object_name + "_" + std::to_string(copy_index + 1);

                // As the official: a copy of the first (already moved) object,
                // moved again by its own position. The official picks the
                // copy's entry by pos_x's length for all three lists (B 1257-1259),
                // reading past a shorter pos_y or pos_z; each list is read by
                // its own length here, so one value serves every copy.
                const auto at = [copy_index](const std::vector<float>& v) {
                    return size_t(copy_index) < v.size() ? v[size_t(copy_index)] : v[0];
                };
                copy_obj->translate(at(assemble_object.pos_x), at(assemble_object.pos_y), at(assemble_object.pos_z));

                array_index = size_t(copy_index) < assemble_object.filaments.size() ? size_t(copy_index) : 0;
                if (!skip_filament) {
                    copy_obj->config.set_key_value("extruder", new ConfigOptionInt(assemble_object.filaments[array_index]));
                    used_filaments.emplace(assemble_object.filaments[array_index]);
                } else {
                    assemble_object.filaments[array_index] = 0;
                }

                array_index = size_t(copy_index) < assemble_object.assemble_index.size() ? size_t(copy_index) : 0;
#ifndef ENGINE_ORCA
                if (assemble_object.subtype != ModelVolumeType::MODEL_PART && assemble_object.assemble_index[array_index] == 0)
                    return fail(CLI_INVALID_PARAMS, "The assemble list gives copy " + std::to_string(copy_index + 1) + " of " + what +
                                                        " assemble_index 0; only a normal part can stand as its own object.");
#endif
                merge_or_add_object(plate, model, assemble_object.assemble_index[array_index], merged_objects, copy_obj,
                                    assemble_object.subtype);
            }
        }

        // The settings of each merged object (B 1291-1329; O 1082-1120).
        if (!merged_objects.empty() && !plate.assembled_params.empty()) {
            for (auto& [assemble_index, assemble_obj] : merged_objects) {
                auto it = plate.assembled_params.find(assemble_index);
                if (it == plate.assembled_params.end()) continue;
                apply_params(assemble_obj->config, it->second.print_params, config_substitutions);
                apply_ranges(assemble_obj, it->second.height_ranges, config_substitutions);
            }
        }
        plate.filaments_count = int(used_filaments.size());
        plate.objects.clear();
        plate.objects.shrink_to_fit();
        plate.plate_params.clear();
    }
    return {};
}

StepResult load_assemble_list(const std::string& file, AssembleList& list) {
    StepResult r = load_assemble_plate_list(file, list.plates);
    if (r.code != 0)
        return r;
    try {
        r = construct_assemble_list(list.plates, list.model, list.plate_configs, list.colours);
        if (r.code != 0)
            return r;
    } catch (const std::exception& e) {
        return fail(CLI_DATA_FILE_ERROR, std::string("The assemble list's parts could not be built: ") + e.what() +
                                              "; check the file");
    }
    list.model.add_default_instances();
    return {};
}

} // namespace slicer_cli
