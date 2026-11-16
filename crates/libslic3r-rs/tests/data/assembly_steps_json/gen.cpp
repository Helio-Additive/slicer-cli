// Reference vectors for crates/libslic3r-rs/src/format/assembly_steps_json.rs, taken from the C++
// at the pin.
//
// The program compiles references/BambuStudio/src/libslic3r/Format/AssemblyStepsJson.cpp as it is,
// with the nlohmann::json the engine bundles. The libslic3r headers it includes are replaced by
// the stand-ins below (their include guards are defined first; Log.hpp, which has none, by
// stub/libslic3r/Log.hpp): the Eigen types of Point.hpp, Geometry::Transformation with the members
// the unit and the Model stand-ins call (bodies as in Geometry.cpp), and a Model, ModelObject,
// ModelVolume and ModelInstance with just the members the unit reads and writes, each following the
// Model.hpp / Model.cpp lines named beside it.
//
// Build and run, from the repository root (macOS arm64 clang is the reference build; Eigen and
// Boost are the ones the engine is built with, see docs/rust-engine/README.md):
//   D=crates/libslic3r-rs/tests/data/assembly_steps_json
//   clang++ -std=c++17 -O2 -ffp-contract=on -DNDEBUG -I $D/stub -I references/BambuStudio/src \
//       -I references/BambuStudio/src/libslic3r -I "$(brew --prefix)/include/eigen3" \
//       -I "$(brew --prefix)/include" -L "$(brew --prefix)/lib" -lboost_filesystem \
//       $D/gen.cpp -o /tmp/asj_gen
//   /tmp/asj_gen steps /tmp/asj_tmp.json > $D/vectors_steps.txt
//   /tmp/asj_gen model /tmp/asj_tmp.json > $D/vectors_model.txt
// (the second argument is a scratch file the LOAD cases are written to and read from).
//
// Output: lines. A line is either one text (escaped: \\ \n \r \s for a space, \xHH for another
// control byte, \e alone for the empty text) or tokens separated by spaces. A text the C++
// returned that is longer than 160 bytes is written as `#<FNV-1a 64 of its bytes, hex> <length>`.
// Doubles and floats are the hex of their bits.
//   DOC / text                      the document the cases below run on, until the next DOC
//   RT / ok font vw vh nitems / title / out
//                                   AssemblyStepJson::load_from_string, the getters, to_json_string
//   LOAD / the same                 AssemblyStepJson::load of a file holding the document
//   FIN / 0|1                       assembly_steps_json_last_step_is_final_assembly
//   MARK / out                      assembly_steps_json_mark_last_step_final_assembly
//   QUAL / out / out                assembly_steps_json_with_qualified_assembly(true), (false)
//   META / version has qualified needs
//                                   assembly_steps_json_file_meta, and
//                                   assembly_steps_version_needs_recompute of its version
//   VER version / out               assembly_steps_json_with_version
//   QRES / 4 texts                  qualified_assembly_result_json(q, n) for q, n in 00 01 10 11
//   MODEL / <model>                 the model the cases below run on, until the next MODEL
//   COV / text / 0|1                assembly_steps_json_last_step_covers_model(doc, model, text)
//   TREE / ok font / error / <tree> / out
//                                   AssemblyStepsTreeData::from_json_string(doc, tree, model,
//                                   &error, &font), then tree.to_json_string()
//   AM2J / out                      assembly_model_to_json_string(model)
//   J2AM / ok|false|throw / error or what() / <model> / out
//                                   assembly_model_from_json_string(doc, out, model, &error), the
//                                   model it left in `out`, assembly_model_to_json_string(out)
// <model>: M nobj, per object `N` (a null pointer) or `O id name extruder nvol`, per volume `n` or
//   `v id name type extruder part_guid src_guid T16 A16 init` (the transformation, the assemble
//   transformation, m_assemble_initialized), then ninst, per instance `n` or
//   `i T16 A16 off3 init dirty`. extruder is `-` where the config has no such option.
// <tree>: T nnodes, per node `type id step name object_idx object_id visible is_final_assembly
//   inherited_from_step_id n ids.. n children.. checked kf.node_idx kf.object_idx kf.is_folder
//   nentries`, checked being `-` or `n {key value}..` in key order, per entry `need_save
//   is_sub_assembly is_interpolation play_node_idx play_frame_idx n {a b name}.. n {a b guid}..
//   json` (volume_names, volume_guids, KeyFrame::to_json dumped without indent); then `R n roots..
//   vw vh baseline n guids..`.

#define slic3r_Geometry_hpp_
#define slic3r_Model_hpp_
#define slic3r_Utils_hpp_

#include <Eigen/Geometry>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace Slic3r {

// Point.hpp:53-73
using Vec2d       = Eigen::Matrix<double, 2, 1, Eigen::DontAlign>;
using Vec3d       = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;
using Matrix3d    = Eigen::Matrix<double, 3, 3, Eigen::DontAlign>;
using Transform3d = Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign>;

// Utils.hpp
inline std::string data_dir() { return "data_dir"; }

namespace Geometry {

// Geometry.cpp:347-353
static Transform3d extract_rotation_matrix(const Transform3d &trafo)
{
    Matrix3d rotation;
    Matrix3d scale;
    trafo.computeRotationScaling(&rotation, &scale);
    return Transform3d(rotation);
}

// Geometry.cpp:363-369
static Transform3d extract_scale(const Transform3d &trafo)
{
    Matrix3d rotation;
    Matrix3d scale;
    trafo.computeRotationScaling(&rotation, &scale);
    return Transform3d(scale);
}

// Geometry.cpp:425-429
inline void scale_transform(Transform3d &transform, const Vec3d &scale)
{
    transform = Transform3d::Identity();
    transform.scale(scale);
}

// Geometry.cpp:434-439
inline Transform3d scale_transform(const Vec3d &scale)
{
    Transform3d transform;
    scale_transform(transform, scale);
    return transform;
}

// Geometry.hpp:395-464, the members used here
class Transformation
{
    Transform3d   m_matrix{Transform3d::Identity()};
    mutable Vec3d m_temp_offset{Vec3d::Zero()};
    mutable Vec3d m_temp_scaling_factor{Vec3d::Ones()};

public:
    Transformation() = default;
    Transformation(const Transformation &trans) : m_matrix(trans.get_matrix()) {}
    Transformation &operator=(const Transformation &) = default;
    explicit Transformation(const Transform3d &transform) : m_matrix(transform) {}

    const Vec3d &get_offset() const
    {
        m_temp_offset = m_matrix.translation();
        return m_temp_offset;
    }

    // Geometry.cpp:475-480
    const Vec3d &get_scaling_factor() const
    {
        const Transform3d scale = extract_scale(m_matrix);
        m_temp_scaling_factor   = {std::abs(scale(0, 0)), std::abs(scale(1, 1)), std::abs(scale(2, 2))};
        return m_temp_scaling_factor;
    }

    // Geometry.cpp:491-498 (the assert left out: the engine is built with NDEBUG)
    void set_scaling_factor(const Vec3d &scaling_factor)
    {
        const Vec3d offset     = get_offset();
        m_matrix               = extract_rotation_matrix(m_matrix) * scale_transform(scaling_factor);
        m_matrix.translation() = offset;
    }

    // Geometry.cpp:597-601, with the default arguments
    const Transform3d &get_matrix() const { return m_matrix; }
    void               set_from_transform(const Transform3d &transform) { m_matrix = transform; }
};

} // namespace Geometry

// ObjectID.hpp
struct ObjectID
{
    size_t id{0};
};

class ObjectBase
{
public:
    ObjectID id() const { return m_id; }
    void     gen_set_id(size_t id) { m_id.id = id; }

protected:
    ObjectBase() : m_id{++s_last_id} {}
    ObjectBase(const ObjectBase &rhs) : m_id(rhs.id()) {}

private:
    ObjectID             m_id;
    static inline size_t s_last_id = 0;
};

// Config.hpp, as far as the unit uses it: the `extruder` option of a config.
struct ConfigOption
{
    virtual ~ConfigOption() = default;
    virtual int getInt() const = 0;
};

struct ConfigOptionInt : ConfigOption
{
    int value;
    explicit ConfigOptionInt(int v) : value(v) {}
    int getInt() const override { return value; }
};

struct ModelConfig
{
    std::shared_ptr<ConfigOption> extruder;
    const ConfigOption *option(const std::string &key) const { return key == "extruder" ? extruder.get() : nullptr; }
    void set_key_value(const std::string &key, ConfigOption *opt)
    {
        if (key != "extruder")
            abort();
        extruder.reset(opt);
    }
};

class Model;
class ModelObject;

// Model.hpp:331-338
enum class ModelVolumeType : int { INVALID = -1, MODEL_PART = 0, NEGATIVE_VOLUME, PARAMETER_MODIFIER, SUPPORT_BLOCKER, SUPPORT_ENFORCER };

class ModelVolume : public ObjectBase
{
public:
    std::string  name;
    ModelConfig  config;
    ModelObject *object;

    explicit ModelVolume(ModelObject *object) : object(object) {}
    // Model.hpp:1303-1315
    ModelVolume(ModelObject *object, const ModelVolume &other)
        : ObjectBase(other), name(other.name), config(other.config), object(object), m_type(other.m_type)
        , m_transformation(other.m_transformation), m_assemble_transformation(other.m_assemble_transformation)
        , m_assemble_initialized(other.m_assemble_initialized), m_part_guid(other.m_part_guid)
        , m_assembly_src_guid(other.m_assembly_src_guid)
    {}

    bool is_model_part() const { return m_type == ModelVolumeType::MODEL_PART; }
    // Model.hpp:1186-1192
    const std::string &part_guid() const { return m_part_guid; }
    void               set_part_guid(const std::string &guid) { m_part_guid = guid; }
    // Model.cpp:2215-2221. The generator gives every model part a GUID, so that nothing random
    // comes into the vectors.
    const std::string &ensure_part_guid(bool force = false) const
    {
        if (m_part_guid.empty() || force)
            abort();
        return m_part_guid;
    }
    const std::string &assembly_src_guid() const { return m_assembly_src_guid; }
    void               set_assembly_src_guid(const std::string &guid) { m_assembly_src_guid = guid; }
    // Model.cpp:3627-3632
    const Geometry::Transformation &get_assemble_transformation() const
    {
        if (!m_assemble_initialized)
            return m_transformation;
        return m_assemble_transformation;
    }
    // Model.cpp:3640-3644
    void set_assemble_from_transform(const Transform3d &transform)
    {
        m_assemble_initialized = true;
        m_assemble_transformation.set_from_transform(transform);
    }
    // Model.cpp:3338-3349
    int extruder_id() const;

    ModelVolumeType                  m_type{ModelVolumeType::MODEL_PART};
    Geometry::Transformation         m_transformation;
    mutable Geometry::Transformation m_assemble_transformation;
    bool                             m_assemble_initialized{false};
    mutable std::string              m_part_guid;
    std::string                      m_assembly_src_guid;
};

class ModelInstance : public ObjectBase
{
public:
    // Model.hpp:1577
    explicit ModelInstance(ModelObject *) : m_assemble_initialized(false) {}

    // Model.cpp:3978-3986
    const Geometry::Transformation &get_assemble_transformation() const
    {
        if (m_assemble_scalling_factor_dirty) {
            m_assemble_transformation.set_scaling_factor(m_transformation.get_scaling_factor());
            m_assemble_scalling_factor_dirty = false;
        }
        return m_assemble_transformation;
    }
    // Model.cpp:3994-3998
    void set_assemble_from_transform(const Transform3d &transform)
    {
        m_assemble_initialized = true;
        m_assemble_transformation.set_from_transform(transform);
    }
    // Model.hpp:1499-1500
    void         set_offset_to_assembly(const Vec3d &offset) { m_offset_to_assembly = offset; }
    const Vec3d &get_offset_to_assembly() const { return m_offset_to_assembly; }

    Geometry::Transformation         m_transformation;
    mutable Geometry::Transformation m_assemble_transformation;
    Vec3d                            m_offset_to_assembly{0.0, 0.0, 0.0};
    bool                             m_assemble_initialized;
    mutable bool                     m_assemble_scalling_factor_dirty{true};
};

class ModelObject : public ObjectBase
{
public:
    std::string                  name;
    ModelConfig                  config;
    std::vector<ModelVolume *>   volumes;
    std::vector<ModelInstance *> instances;

    // Model.cpp:1476-1491
    ModelVolume *add_volume(const ModelVolume &other)
    {
        ModelVolume *v = new ModelVolume(this, other);
        this->volumes.push_back(v);
        return v;
    }
    // Model.cpp:1603-1612
    ModelInstance *add_instance()
    {
        ModelInstance *i = new ModelInstance(this);
        this->instances.push_back(i);
        return i;
    }
};

// Model.cpp:3338-3349
inline int ModelVolume::extruder_id() const
{
    int extruder_id = -1;
    {
        const ConfigOption *opt = this->config.option("extruder");
        if ((opt == nullptr) || (opt->getInt() == 0))
            opt = this->object->config.option("extruder");
        extruder_id = (opt == nullptr) ? 1 : opt->getInt();
    }
    return extruder_id;
}

class Model
{
public:
    std::vector<ModelObject *> objects;

    // Model.cpp:581-585
    ModelObject *add_object()
    {
        this->objects.emplace_back(new ModelObject());
        return this->objects.back();
    }
    // Model.cpp:719-732
    void clear_objects() { this->objects.clear(); }
};

} // namespace Slic3r

#include "Format/AssemblyStepsJson.cpp"

using namespace Slic3r;
using nlohmann::json;

// ---- output ----

static std::string esc(const std::string &s)
{
    if (s.empty())
        return "\\e";
    std::string out;
    char        buf[8];
    for (unsigned char c : s) {
        if (c == '\\')
            out += "\\\\";
        else if (c == '\n')
            out += "\\n";
        else if (c == '\r')
            out += "\\r";
        else if (c == ' ')
            out += "\\s";
        else if (c < 0x20 || c == 0x7f) {
            snprintf(buf, sizeof(buf), "\\x%02X", c);
            out += buf;
        } else
            out += (char) c;
    }
    return out;
}

static std::string fnv(const std::string &s)
{
    uint64_t h = 0xcbf29ce484222325ull;
    for (unsigned char c : s) {
        h ^= c;
        h *= 0x100000001b3ull;
    }
    char buf[64];
    snprintf(buf, sizeof(buf), "#%016llx %zu", (unsigned long long) h, s.size());
    return buf;
}

// A text the C++ returned.
// (With ASJ_FULL set in the environment every text is written out, to look at a difference.)
static std::string out(const std::string &s) { return s.size() > 160 && getenv("ASJ_FULL") == nullptr ? fnv(s) : esc(s); }

static std::string dbits(double v)
{
    uint64_t b;
    memcpy(&b, &v, 8);
    char buf[32];
    snprintf(buf, sizeof(buf), "%016llx", (unsigned long long) b);
    return buf;
}

static std::string fbits(float v)
{
    uint32_t b;
    memcpy(&b, &v, 4);
    char buf[32];
    snprintf(buf, sizeof(buf), "%08x", b);
    return buf;
}

static std::string mat(const Transform3d &t)
{
    std::string s;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            s += " " + dbits(t.matrix()(r, c));
    return s;
}

static std::string ext(const ModelConfig &config)
{
    const ConfigOption *opt = config.option("extruder");
    return opt == nullptr ? "-" : std::to_string(opt->getInt());
}

static std::string dump_model(const Model &model)
{
    std::string s = "M " + std::to_string(model.objects.size());
    for (const ModelObject *mo : model.objects) {
        if (mo == nullptr) {
            s += " N";
            continue;
        }
        s += " O " + std::to_string(mo->id().id) + " " + esc(mo->name) + " " + ext(mo->config) + " " + std::to_string(mo->volumes.size());
        for (const ModelVolume *mv : mo->volumes) {
            if (mv == nullptr) {
                s += " n";
                continue;
            }
            s += " v " + std::to_string(mv->id().id) + " " + esc(mv->name) + " " + std::to_string((int) mv->m_type) + " " + ext(mv->config) + " " +
                 esc(mv->m_part_guid) + " " + esc(mv->m_assembly_src_guid) + mat(mv->m_transformation.get_matrix()) +
                 mat(mv->m_assemble_transformation.get_matrix()) + " " + std::to_string((int) mv->m_assemble_initialized);
        }
        s += " " + std::to_string(mo->instances.size());
        for (const ModelInstance *mi : mo->instances) {
            if (mi == nullptr) {
                s += " n";
                continue;
            }
            s += " i" + mat(mi->m_transformation.get_matrix()) + mat(mi->m_assemble_transformation.get_matrix()) + " " +
                 dbits(mi->m_offset_to_assembly.x()) + " " + dbits(mi->m_offset_to_assembly.y()) + " " + dbits(mi->m_offset_to_assembly.z()) + " " +
                 std::to_string((int) mi->m_assemble_initialized) + " " + std::to_string((int) mi->m_assemble_scalling_factor_dirty);
        }
    }
    return s;
}

static std::string dump_tree(const AssemblyStepsTreeData &tree)
{
    std::string s = "T " + std::to_string(tree.nodes.size());
    for (const AssemblyStepsTreeNode &n : tree.nodes) {
        s += " " + std::to_string((int) n.type) + " " + std::to_string(n.id) + " " + std::to_string(n.step) + " " + esc(n.name) + " " +
             std::to_string(n.object_idx) + " " + std::to_string(n.object_id) + " " + std::to_string((int) n.visible) + " " +
             std::to_string(n.is_final_assembly) + " " + std::to_string(n.inherited_from_step_id);
        s += " " + std::to_string(n.inherited_object_ids.size());
        for (size_t id : n.inherited_object_ids)
            s += " " + std::to_string(id);
        s += " " + std::to_string(n.children.size());
        for (int c : n.children)
            s += " " + std::to_string(c);
        if (!n.assembly_tree_checked)
            s += " -";
        else {
            std::map<std::string, bool> sorted(n.assembly_tree_checked->begin(), n.assembly_tree_checked->end());
            s += " " + std::to_string(sorted.size());
            for (const auto &item : sorted)
                s += " " + esc(item.first) + " " + std::to_string((int) item.second);
        }
        s += " " + std::to_string(n.kf_data.node_idx) + " " + std::to_string(n.kf_data.object_idx) + " " + std::to_string((int) n.kf_data.is_folder) +
             " " + std::to_string(n.kf_data.entries.size());
        for (const KeyFrameEntry &e : n.kf_data.entries) {
            s += " " + std::to_string((int) e.need_save) + " " + std::to_string((int) e.data.is_sub_assembly) + " " +
                 std::to_string((int) e.data.is_interpolation) + " " + std::to_string(e.data.play_node_idx) + " " +
                 std::to_string(e.data.play_frame_idx);
            s += " " + std::to_string(e.data.volume_names.size());
            for (const auto &p : e.data.volume_names)
                s += " " + std::to_string(p.first.first) + " " + std::to_string(p.first.second) + " " + esc(p.second);
            s += " " + std::to_string(e.data.volume_guids.size());
            for (const auto &p : e.data.volume_guids)
                s += " " + std::to_string(p.first.first) + " " + std::to_string(p.first.second) + " " + esc(p.second);
            json kj;
            e.data.to_json(kj);
            s += " " + out(kj.dump());
        }
    }
    s += " R " + std::to_string(tree.roots.size());
    for (int r : tree.roots)
        s += " " + std::to_string(r);
    s += " " + std::to_string(tree.camera_ref_viewport_w) + " " + std::to_string(tree.camera_ref_viewport_h) + " " +
         std::to_string((int) tree.has_loaded_recorded_baseline) + " " + std::to_string(tree.loaded_recorded_volumes.size());
    for (const std::string &g : tree.loaded_recorded_volumes)
        s += " " + esc(g);
    return s;
}

// ---- deterministic inputs ----

static uint64_t g_seed = 0x9E3779B97F4A7C15ull;
static uint32_t rnd()
{
    g_seed = g_seed * 6364136223846793005ull + 1442695040888963407ull;
    return (uint32_t)(g_seed >> 33);
}
static int  rint(int n) { return (int) (rnd() % (uint32_t) n); }
static bool chance(int pct) { return rint(100) < pct; }

// Doubles whose shortest decimal form has few digits, so that every JSON reader agrees on them.
static double rdouble()
{
    switch (rint(8)) {
    case 0: return rint(2001) - 1000;
    case 1: return (rint(200001) - 100000) / 64.0;
    case 2: return (rint(2000001) - 1000000) / 1000.0;
    case 3: return 0.0;
    case 4: return (rint(2001) - 1000) / 1e7;
    case 5: return (rint(2001) - 1000) * 1048576.0;
    case 6: return rint(2) ? 1.0 : -1.0;
    default: return (rint(513) - 256) / 256.0;
    }
}

static json wrong()
{
    switch (rint(13)) {
    case 0: return nullptr;
    case 1: return true;
    case 2: return false;
    case 3: return rint(5);
    case 4: return -1 - rint(5);
    case 5: return 2.5;
    case 6: return "x";
    case 7: return json::array({1, "a"});
    case 8: return json::object({{"k", 1}});
    case 9: return 4294967297ll;
    case 10: return -4294967295ll;
    case 11: return 9223372036854775808ull;
    default: return 18446744073709551615ull;
    }
}

template<class F> static void put(json &j, const char *key, F proper, int present = 82)
{
    const int r = rint(100);
    if (r < present)
        j[key] = proper();
    else if (r < present + (100 - present) / 2) {
    } else
        j[key] = wrong();
}

static const char *NAMES[] = {"cube", "gear", "base", "", "W\xC3\xBCrfel", "part a", "line1\nline2\t\"q\"\\"};
static const char *GUIDS[] = {"g-1", "g-2", "g-3", "g-4", "g-5", "g-6", "g-7", "g-8"};
static std::string rname() { return NAMES[rint(7)]; }
static std::string rguid() { return GUIDS[rint(8)]; }

static json jnum() { return chance(20) ? json(rint(400) - 100) : json(rdouble()); }
static json vec2() { return chance(4) ? json::array({jnum()}) : json::array({jnum(), jnum()}); }
static json color()
{
    json a   = json::array();
    int  n   = chance(45) ? 3 : 4;
    if (chance(6))
        n = chance(50) ? 2 : 5;
    for (int i = 0; i < n; ++i) {
        switch (rint(10)) {
        case 0: a.push_back(-5 - rint(300)); break;
        case 1: a.push_back(256 + rint(1000)); break;
        case 2: a.push_back(rint(512) / 2.0); break; // .5 values: std::round goes away from zero
        case 3: a.push_back(-0.5); break;
        case 4: a.push_back(254.49 + rint(3)); break;
        case 5: a.push_back(chance(15) ? json("255") : json(1e300 * (rint(2) ? 1 : -1))); break;
        default: a.push_back(rint(256)); break;
        }
    }
    return a;
}
static json bound_volumes()
{
    json a = json::array();
    for (int i = rint(4); i > 0; --i) {
        switch (rint(12)) {
        case 0: a.push_back(json::array({rint(5)})); break;
        case 1: a.push_back(json::array({rint(5), "b"})); break;
        case 2: a.push_back(json::array({1.5, rint(5)})); break;
        case 3: a.push_back(rint(5)); break;
        default: a.push_back(json::array({rint(8) - 1, rint(6) - 1})); break;
        }
    }
    return a;
}
static json ray()
{
    json j = json::object();
    put(j, "bound_volumes", bound_volumes);
    put(j, "arrow_start_offset", vec2);
    put(j, "arrow_end_offset", vec2);
    return j;
}
static json arrow_svg()
{
    json j = json::object();
    put(j, "svg_name", [] { return json(rname()); });
    put(j, "label_size", vec2);
    put(j, "color", color);
    const int r = rint(10);
    if (r < 6) {
        json rays = json::array();
        for (int i = 1 + rint(2); i > 0; --i)
            rays.push_back(chance(92) ? ray() : wrong());
        j["rays"] = rays;
    } else if (r < 7)
        j["rays"] = json::array();
    if (r >= 5) {
        put(j, "bound_volumes", bound_volumes);
        put(j, "arrow_start_offset", vec2);
        put(j, "arrow_end_offset", vec2);
    }
    return j;
}
static json text_label()
{
    json j = json::object();
    put(j, "text", [] { return json(rname()); });
    put(j, "bound_volumes", bound_volumes);
    put(j, "pos_offset", vec2);
    put(j, "size", vec2);
    put(j, "color", color);
    put(j, "background_color", color);
    return j;
}
static json shape_note()
{
    json j = json::object();
    put(j, "bound_volumes", bound_volumes);
    put(j, "pos_offset", vec2);
    put(j, "size", vec2);
    put(j, "color", color);
    return j;
}
static json plain_arrow()
{
    json j = json::object();
    put(j, "bound_volumes", bound_volumes);
    put(j, "arrow_start_offset", vec2);
    put(j, "arrow_end_offset", vec2);
    put(j, "color", color);
    return j;
}
static json part_number_label()
{
    json j = json::object();
    put(j, "object_idx", [] { return json(rint(7) - 1); });
    put(j, "volume_idx", [] { return json(rint(5) - 1); });
    put(j, "part_name", [] { return json(rname()); });
    put(j, "uuid", [] { return json(rguid()); });
    put(j, "arrow_start_offset", vec2);
    put(j, "arrow_end_offset", vec2);
    put(j, "visible", [] { return json(chance(50)); }, 60);
    put(j, "in_explosion_state", [] { return json(chance(50)); }, 60);
    if (chance(25))
        j["is_show"] = chance(80) ? json(chance(50)) : wrong();
    return j;
}
template<class F> static json some(F make, int max)
{
    json a = json::array();
    for (int i = rint(max + 1); i > 0; --i)
        a.push_back(chance(94) ? make() : wrong());
    return a;
}
static json note()
{
    json j = json::object();
    put(j, "arrow_svgs", [] { return some(arrow_svg, 2); });
    put(j, "text_labels", [] { return some(text_label, 2); });
    put(j, "circle_notes", [] { return some(shape_note, 1); });
    put(j, "rectangle_notes", [] { return some(shape_note, 1); });
    put(j, "plain_arrows", [] { return some(plain_arrow, 1); });
    put(j, "part_number_labels", [] { return some(part_number_label, 3); });
    if (chance(45))
        j["show_part_labels"] = chance(85) ? json(chance(50)) : wrong();
    if (chance(30))
        j["is_show"] = chance(85) ? json(chance(50)) : wrong();
    return j;
}
// `general` false: the identity, or a translation. That is what the instances of the model cases
// get, see rmodel().
static json mat16_of(bool general)
{
    json      a    = json::array();
    const int kind = rint(general ? 4 : 2);
    for (int i = 0; i < 16; ++i) {
        if (kind == 0)
            a.push_back((i % 5 == 0) ? 1.0 : 0.0);
        else if (kind == 1)
            a.push_back((i % 5 == 0) ? json(1) : (i % 4 == 3 && i < 12 ? json(rdouble()) : json(0)));
        else
            a.push_back(rdouble());
    }
    const int r = rint(100);
    if (r < 5)
        a.erase(a.begin() + rint(16));
    else if (r < 10)
        a[rint(16)] = chance(50) ? json("1") : json(true);
    else if (r < 12)
        a.push_back(0);
    return a;
}
static json mat16() { return mat16_of(true); }
static json mat16_simple() { return mat16_of(false); }
static json keyframe()
{
    json j = json::object();
    put(j, "id", [] { return json(rint(5)); });
    put(j, "name", [] { return json(rname()); });
    put(j, "is_camera_define", [] { return json(chance(50)); });
    put(j, "camera_user_defined", [] { return json(chance(50)); });
    put(j, "view_matrix", mat16);
    put(j, "projection_matrix", mat16);
    put(j, "camera_target", [] {
        json a = json::array({jnum(), jnum(), jnum()});
        if (chance(6))
            a.erase(a.begin());
        else if (chance(6))
            a[rint(3)] = "t";
        return a;
    });
    put(j, "camera_zoom", jnum);
    put(j, "camera_margin_factor", jnum);
    put(j, "labels_show_type", [] { return json(chance(85) ? rint(3) : (chance(50) ? 7 : -1)); });
    put(j, "object_transformations", [] {
        json a = json::array();
        for (int i = rint(3); i > 0; --i) {
            json item = json::object();
            put(item, "object_id", [] { return json(rint(6)); }, 90);
            put(item, "transformation", mat16, 90);
            a.push_back(chance(95) ? item : wrong());
        }
        return a;
    });
    put(j, "volume_transformations", [] {
        json a = json::array();
        for (int i = rint(4); i > 0; --i) {
            json item = json::object();
            put(item, "object_id", [] { return json(rint(6)); }, 90);
            put(item, "volume_idx", [] { return json(rint(3)); }, 70);
            put(item, "volume_name", [] { return json(rname()); }, 60);
            put(item, "uuid", [] { return json(chance(20) ? std::string() : rguid()); }, 60);
            put(item, "transformation", mat16, 90);
            a.push_back(chance(95) ? item : wrong());
        }
        return a;
    });
    if (chance(45))
        j["assembly_note"] = chance(92) ? note() : wrong();
    return j;
}
// The model the documents of the model cases are made for: most of their `single` items name an
// object of it, by index, id or name.
static const Model *g_model = nullptr;

static json single()
{
    json j    = json::object();
    j["type"] = "single";
    if (g_model != nullptr && !g_model->objects.empty() && chance(75)) {
        const int          oi = rint((int) g_model->objects.size());
        const ModelObject *mo = g_model->objects[oi];
        if (chance(85))
            j["name"] = (mo != nullptr && chance(80)) ? mo->name : rname();
        j["keyframes"] = some(keyframe, 2);
        if (chance(70))
            j["object_idx"] = chance(70) ? oi : rint(7) - 1;
        if (chance(60))
            j["object_id"] = (mo != nullptr && chance(80)) ? (int) mo->id().id : rint(13);
        return j;
    }
    put(j, "name", [] { return json(rname()); }, 92);
    put(j, "keyframes", [] { return some(keyframe, 2); });
    put(j, "object_idx", [] { return json(rint(7) - 1); });
    put(j, "object_id", [] { return json(chance(8) ? -3 : rint(13)); });
    return j;
}
static json sub(int depth)
{
    json j    = json::object();
    j["type"] = "sub";
    put(j, "name", [] { return json(rname()); }, 92);
    put(j, "keyframes", [] { return some(keyframe, 1); });
    put(j, "id", [] { return json(rint(8) - 1); });
    put(j, "step", [] { return json(rint(9)); });
    switch (rint(8)) {
    case 0: break;
    case 1: j["is_final_assembly"] = true; break;
    case 2: j["is_final_assembly"] = false; break;
    case 3: j["is_final_assembly"] = 1; break;
    case 4: j["is_final_assembly"] = 2; break;
    case 5: j["is_final_assembly"] = chance(50) ? json(5) : wrong(); break;
    default: j["is_final_assembly"] = 0; break;
    }
    put(j, "inherited_from_step_id", [] { return json(rint(8) - 1); }, 60);
    put(j, "inherited_object_ids", [] {
        json a = json::array();
        for (int i = rint(4); i > 0; --i) {
            switch (rint(8)) {
            case 0: a.push_back(-1 - rint(3)); break;
            case 1: a.push_back(0); break;
            case 2: a.push_back(1.5); break;
            case 3: a.push_back("7"); break;
            default: a.push_back(rint(13)); break;
            }
        }
        return a;
    }, 60);
    put(j, "assembly_tree_checked", [] {
        json o = json::object();
        for (int i = rint(4); i > 0; --i)
            o[rguid()] = chance(85) ? json(chance(50)) : wrong();
        return o;
    }, 50);
    put(j, "children", [depth] {
        json a = json::array();
        for (int i = rint(4); i > 0; --i) {
            const int r = rint(20);
            if (r == 0)
                a.push_back(wrong());
            else if (r == 1)
                a.push_back(json::object({{"type", "other"}, {"name", "n"}}));
            else if (r < 6 && depth < 2)
                a.push_back(sub(depth + 1));
            else
                a.push_back(single());
        }
        return a;
    }, 90);
    return j;
}
static json steps_doc()
{
    json j = json::object();
    put(j, "version", [] {
        switch (rint(7)) {
        case 0: return json(1.0);
        case 1: return json(1.1);
        case 2: return json(1.3);
        case 3: return json(1);
        case 4: return json("1.2");
        case 5: return json(1.1995);
        default: return json(1.2);
        }
    });
    put(j, "assembly_part_number_label_font_size", [] {
        switch (rint(6)) {
        case 0: return json(0);
        case 1: return json(-2.5);
        case 2: return json(14);
        case 3: return json(0.1);
        default: return json(rint(400) / 8.0);
        }
    });
    put(j, "camera_ref_viewport_w", [] { return json(rint(4000)); });
    put(j, "camera_ref_viewport_h", [] { return json(rint(3000)); });
    put(j, "pdf_export", [] {
        json p = json::object();
        put(p, "title", [] { return json(rname()); });
        return p;
    });
    put(j, "items", [] {
        json a = json::array();
        for (int i = (g_model != nullptr ? 2 : 0) + rint(5); i > 0; --i) {
            const int r = rint(20);
            if (r == 0)
                a.push_back(wrong());
            else if (r < 4)
                a.push_back(single());
            else
                a.push_back(sub(0));
        }
        return a;
    }, 94);
    switch (rint(9)) {
    case 0: j["qualified_assembly"] = true; break;
    case 1: j["qualified_assembly"] = false; break;
    case 2: j["qualified_assembly"] = "true"; break;
    case 3: j["qualified_assembly"] = "false"; break;
    case 4: j["qualified_assembly"] = 1; break;
    default: break;
    }
    return j;
}

static Transform3d rtransform(int kind)
{
    Transform3d t = Transform3d::Identity();
    if (kind == 1 || kind == 2)
        for (int r = 0; r < 3; ++r)
            t.matrix()(r, 3) = rdouble();
    if (kind == 2)
        for (int r = 0; r < 3; ++r)
            t.matrix()(r, r) = (1 + rint(40)) / 8.0;
    if (kind >= 3)
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
                t.matrix()(r, c) = (rint(2001) - 1000) / 64.0;
    return t;
}

static void rextruder(ModelConfig &config)
{
    switch (rint(4)) {
    case 0: break;
    case 1: config.set_key_value("extruder", new ConfigOptionInt(0)); break;
    case 2: config.set_key_value("extruder", new ConfigOptionInt(1)); break;
    default: config.set_key_value("extruder", new ConfigOptionInt(3)); break;
    }
}

// `nulls`: the model has null pointers among its objects, volumes and instances.
//
// An instance whose scaling factor is dirty has its assemble transformation decomposed by
// ModelInstance::get_assemble_transformation (Transformation::set_scaling_factor, Eigen's
// computeRotationScaling). That is Geometry.cpp's arithmetic, not this unit's, and the Rust
// Transformation does not give Eigen's bits for a general matrix; so a dirty instance gets a
// matrix both are exact on (the identity or a translation), and only an
// instance that is not dirty a general one.
static Model rmodel(bool nulls)
{
    Model model;
    for (int no = rint(6); no > 0; --no) {
        if (nulls && chance(12)) {
            model.objects.push_back(nullptr);
            continue;
        }
        ModelObject *mo = model.add_object();
        mo->gen_set_id((size_t) rint(13));
        mo->name = rname();
        rextruder(mo->config);
        for (int nv = rint(4); nv > 0; --nv) {
            if (nulls && chance(12)) {
                mo->volumes.push_back(nullptr);
                continue;
            }
            ModelVolume *mv = new ModelVolume(mo);
            mv->gen_set_id((size_t) rint(40));
            mv->name = rname();
            if (chance(30))
                mv->m_type = chance(50) ? ModelVolumeType::NEGATIVE_VOLUME : ModelVolumeType::PARAMETER_MODIFIER;
            rextruder(mv->config);
            if (mv->is_model_part() || chance(50))
                mv->m_part_guid = rguid();
            if (chance(30))
                mv->m_assembly_src_guid = rguid();
            mv->m_transformation.set_from_transform(rtransform(rint(5)));
            mv->m_assemble_transformation.set_from_transform(rtransform(rint(5)));
            mv->m_assemble_initialized = chance(50);
            mo->volumes.push_back(mv);
        }
        for (int ni = rint(4); ni > 0; --ni) {
            if (nulls && chance(12)) {
                mo->instances.push_back(nullptr);
                continue;
            }
            ModelInstance *mi = mo->add_instance();
            const bool dirty = chance(60);
            mi->m_transformation.set_from_transform(rtransform(dirty ? rint(2) : rint(5)));
            mi->m_assemble_transformation.set_from_transform(rtransform(dirty ? rint(2) : rint(5)));
            mi->m_offset_to_assembly             = Vec3d(rdouble(), rdouble(), rdouble());
            mi->m_assemble_initialized           = chance(50);
            mi->m_assemble_scalling_factor_dirty = dirty;
        }
    }
    return model;
}

static json amodel_doc()
{
    json j = json::object();
    put(j, "version", [] { return json(1); });
    put(j, "nodes", [] {
        json a = json::array();
        for (int i = rint(5); i > 0; --i) {
            if (chance(6)) {
                a.push_back(wrong());
                continue;
            }
            json o = json::object();
            put(o, "id", [] { return json(rint(13)); });
            put(o, "type", [] { return json("group"); });
            put(o, "name", [] { return json(rname()); });
            put(o, "nodes", [] {
                json v = json::array();
                for (int k = rint(4); k > 0; --k) {
                    if (chance(5)) {
                        v.push_back(wrong());
                        continue;
                    }
                    json jv = json::object();
                    put(jv, "volume", [] { return json(chance(8) ? std::string("g-none") : (chance(5) ? std::string() : rguid())); }, 90);
                    put(jv, "name", [] { return json(rname()); });
                    put(jv, "assemble_transform", mat16);
                    v.push_back(jv);
                }
                return v;
            });
            auto offset = [] {
                json a = json::array({jnum(), jnum(), jnum()});
                if (chance(6))
                    a.erase(a.begin());
                else if (chance(4))
                    a[rint(3)] = "t";
                return a;
            };
            const int r = rint(10);
            if (r < 4) {
                put(o, "assemble_transform", mat16_simple);
                put(o, "offset_to_assembly", offset);
            } else if (r < 8) {
                json insts = json::array();
                for (int k = rint(4); k > 0; --k) {
                    if (chance(5)) {
                        insts.push_back(wrong());
                        continue;
                    }
                    json ji = json::object();
                    put(ji, "assemble_transform", mat16_simple);
                    put(ji, "offset_to_assembly", offset);
                    insts.push_back(ji);
                }
                o["instances"] = insts;
            } else if (r < 9)
                o["instances"] = wrong();
            a.push_back(o);
        }
        return a;
    }, 92);
    return j;
}

// A document whose last step, with the steps it inherits from, names every object of the model:
// by index, by id, or by the id `amodel` records for it.
static json covering_doc(const Model &model, json &amodel, bool complete)
{
    json first  = json::object({{"type", "sub"}, {"id", 1}, {"children", json::array()}});
    json second = json::object({{"type", "sub"}, {"id", 2}, {"inherited_from_step_id", 1}, {"children", json::array()}, {"inherited_object_ids", json::array()}});
    json nodes  = json::array();
    for (int oi = 0; oi < (int) model.objects.size(); ++oi) {
        const ModelObject *mo        = model.objects[oi];
        const int          persisted = 100 + oi;
        nodes.push_back(json::object({{"id", persisted}}));
        if (mo == nullptr || (!complete && oi == 0))
            continue;
        json &step = (oi % 2 == 0) ? first : second;
        switch (oi % 4) {
        case 0: step["children"].push_back(json::object({{"type", "single"}, {"object_idx", oi}})); break;
        case 1: second["inherited_object_ids"].push_back(persisted); break;
        case 2: step["children"].push_back(json::object({{"type", "single"}, {"object_id", persisted}})); break;
        default:
            if (mo->id().id != 0)
                step["children"].push_back(json::object({{"type", "sub"}, {"children", json::array({json::object({{"type", "single"}, {"object_id", mo->id().id}})})}}));
            else
                step["children"].push_back(json::object({{"type", "single"}, {"object_idx", oi}}));
            break;
        }
    }
    amodel = json::object({{"version", 1}, {"nodes", nodes}});
    return json::object({{"version", 1.2}, {"items", json::array({first, second, json::object({{"type", "sub"}, {"id", 9}, {"is_final_assembly", 2}})})}});
}

// ---- cases ----

static const char *g_tmp = "asj_gen_tmp.json";

static void doc_line(const std::string &text) { printf("DOC\n%s\n", esc(text).c_str()); }

static void rt_out(bool ok, const AssemblyStepJson &doc)
{
    printf("%d %s %d %d %zu\n%s\n%s\n", (int) ok, fbits(doc.get_assembly_part_number_label_font_size()).c_str(), doc.get_camera_ref_viewport_w(),
           doc.get_camera_ref_viewport_h(), doc.get_items().size(), esc(doc.get_pdf_export_params().title).c_str(), out(doc.to_json_string()).c_str());
}

static void case_rt(const std::string &text)
{
    AssemblyStepJson doc;
    const bool       ok = doc.load_from_string(text);
    printf("RT\n");
    rt_out(ok, doc);
}

static void case_load(const std::string &text)
{
    {
        std::ofstream f(g_tmp, std::ios::binary);
        f << text;
    }
    AssemblyStepJson doc;
    const bool       ok = doc.load(g_tmp);
    printf("LOAD\n");
    rt_out(ok, doc);
}

static const double VERSIONS[] = {1.0, 1.1, 1.2, 1.2000001, 1.20001, 1.3, 2.0, 0.0, -1.5, 1.1995};

static void string_cases(const std::string &text)
{
    case_rt(text);
    printf("FIN\n%d\n", (int) assembly_steps_json_last_step_is_final_assembly(text));
    printf("MARK\n%s\n", out(assembly_steps_json_mark_last_step_final_assembly(text)).c_str());
    printf("QUAL\n%s\n%s\n", out(assembly_steps_json_with_qualified_assembly(text, true)).c_str(),
           out(assembly_steps_json_with_qualified_assembly(text, false)).c_str());
    const AssemblyStepsFileMeta meta = assembly_steps_json_file_meta(text);
    printf("META\n%s %d %d %d\n", dbits(meta.version).c_str(), (int) meta.has_qualified, (int) meta.qualified,
           (int) assembly_steps_version_needs_recompute(meta.version));
    const double version = VERSIONS[rint(10)];
    printf("VER %s\n%s\n", dbits(version).c_str(), out(assembly_steps_json_with_version(text, version)).c_str());
}

static void steps_vectors()
{
    printf("QRES\n");
    for (int q = 0; q < 2; ++q)
        for (int n = 0; n < 2; ++n)
            printf("%s\n", esc(qualified_assembly_result_json(q != 0, n != 0)).c_str());

    const std::vector<std::string> fixed = {
        "",
        "null",
        "[]",
        "{}",
        "{",
        "123",
        "123abc",
        "\"str\"",
        "true false",
        "{} xyz",
        "{}}",
        " \n\t{ \"items\" : [ ] } \n",
        "\xEF\xBB\xBF{\"version\":1.2,\"items\":[]}",
        "{\"items\":{}}",
        "{\"items\":[{\"type\":\"sub\"}]}",
        "{\"items\":[{\"type\":\"sub\",\"is_final_assembly\":true}]}",
        "{\"items\":[{\"type\":\"sub\",\"is_final_assembly\":1},{\"type\":\"sub\",\"is_final_assembly\":2}]}",
        "{\"items\":[{\"type\":\"sub\",\"is_final_assembly\":2}]}",
        "{\"items\":[{\"type\":\"sub\",\"is_final_assembly\":1.0}]}",
        "{\"items\":[{\"type\":\"single\",\"object_idx\":-0,\"object_id\":-0,\"keyframes\":[{\"id\":-0,\"camera_zoom\":-0,\"camera_margin_factor\":-0.0}]}]}",
        "{\"items\":[{\"type\":\"sub\",\"id\":3,\"type\":\"single\",\"object_idx\":2}]}",
        "{\"version\":1.2,\"qualified_assembly\":true}",
        "{\"version\":1.2,\"qualified_assembly\":\"true\"}",
        "{\"version\":\"1.2\",\"qualified_assembly\":null}",
        "{\"version\":1.19999999,\"qualified_assembly\":false,\"z\":[1,2,{\"a\":null}],\"a\":\"\\u00e9\\ud83d\\ude00\\u0001\\u007f\\\"\\\\/\"}",
        "{\"version\":12e-1}",
        "{\"version\":1E400}",
        "{\"a\":\"\\ud800\"}",
        "{\"a\":01}",
        "{\"a\":1,}",
        "{'a':1}",
        "{\"a\":1} // c",
        "/* c */ {\"a\":1}",
        "{\"a\":NaN}",
        "[{\"type\":\"sub\"}]",
        "{\"version\":1.2,\"big\":[18446744073709551615,18446744073709551616,-9223372036854775808,-9223372036854775809,1e2,1E+2,0.1e-2,-0,-0.0,5e-324,1.7976931348623157e308]}",
        "{\"assembly_part_number_label_font_size\":16777217,\"camera_ref_viewport_w\":4294967297,\"camera_ref_viewport_h\":-4294967295}",
        "{\"assembly_part_number_label_font_size\":1e39,\"pdf_export\":{\"title\":7},\"items\":[{\"type\":\"single\",\"object_id\":9223372036854775808,\"name\":5}]}",
        "{\"items\":[{\"type\":\"single\",\"keyframes\":[5,null,\"k\",[],{\"assembly_note\":{\"part_number_labels\":[{\"is_show\":true},{\"is_show\":false},7]}},"
        "{\"assembly_note\":{\"part_number_labels\":[{\"is_show\":false}],\"is_show\":5}},"
        "{\"assembly_note\":{\"arrow_svgs\":[{\"rays\":[]},{\"rays\":5},{\"rays\":[5]},{}],\"text_labels\":[{\"color\":[1.5,2.5,-0.5]},{\"color\":[0.49999999999999994,254.5,255.5,1e10]}]}}]}]}",
        "{\"items\":[{\"type\":\"sub\",\"id\":1,\"children\":[{\"type\":\"sub\",\"children\":[{\"type\":\"sub\",\"children\":[{\"type\":\"single\"}]}]}],"
        "\"inherited_object_ids\":[18446744073709551615,-1,1.0,true],\"assembly_tree_checked\":{\"b\":true,\"a\":false,\"c\":1,\"\":true}}]}",
    };
    for (const std::string &text : fixed) {
        doc_line(text);
        string_cases(text);
        case_load(text);
    }

    for (int i = 0; i < 64; ++i) {
        const json        j    = steps_doc();
        const std::string text = (i % 16 == 15) ? j.dump(1) : j.dump();
        doc_line(text);
        string_cases(text);
        if (i % 8 == 0)
            case_load(text);
    }
}

static void model_cases(const Model &model, bool with_j2am)
{
    printf("MODEL\n%s\n", dump_model(model).c_str());
    g_model = &model;
    for (int complete = 0; complete < 2; ++complete) {
        json              amodel;
        const std::string text = covering_doc(model, amodel, complete != 0).dump();
        doc_line(text);
        printf("COV\n%s\n%d\n", esc(amodel.dump()).c_str(), (int) assembly_steps_json_last_step_covers_model(text, model, amodel.dump()));
        printf("COV\n%s\n%d\n", esc(std::string()).c_str(), (int) assembly_steps_json_last_step_covers_model(text, model, std::string()));
    }
    for (int i = 0; i < 5; ++i) {
        const std::string text = (i == 4) ? std::string("{") : steps_doc().dump();
        doc_line(text);
        const std::string amodel = (i % 3 == 0) ? std::string() : (i == 2 ? std::string("{]") : amodel_doc().dump());
        printf("COV\n%s\n%d\n", esc(amodel).c_str(), (int) assembly_steps_json_last_step_covers_model(text, model, amodel));

        AssemblyStepsTreeData tree;
        std::string           error = "unset";
        float                 font  = -1.f;
        const bool            ok    = AssemblyStepsTreeData::from_json_string(text, tree, model, &error, &font);
        printf("TREE\n%d %s\n%s\n%s\n%s\n", (int) ok, fbits(font).c_str(), esc(error).c_str(), dump_tree(tree).c_str(), out(tree.to_json_string()).c_str());
    }
    {
        doc_line("");
        AssemblyStepsTreeData tree;
        std::string           error = "unset";
        float                 font  = -1.f;
        const bool            ok    = AssemblyStepsTreeData::from_json_string("", tree, model, &error, &font);
        printf("TREE\n%d %s\n%s\n%s\n%s\n", (int) ok, fbits(font).c_str(), esc(error).c_str(), dump_tree(tree).c_str(), out(tree.to_json_string()).c_str());
    }
    g_model = nullptr;
    if (!with_j2am)
        return;
    std::vector<std::string> docs = {"", "{", "[]", "{\"nodes\":5}", "{\"nodes\":[]}", "{\"version\":1,\"nodes\":[{}]}"};
    for (int i = 0; i < 10; ++i)
        docs.push_back(amodel_doc().dump());
    for (const std::string &text : docs) {
        doc_line(text);
        Model       out_model;
        out_model.add_object()->name = "stale";
        std::string error            = "unset";
        std::string result;
        try {
            result = assembly_model_from_json_string(text, out_model, model, &error) ? "ok" : "false";
        } catch (const std::exception &e) {
            result = "throw";
            error  = e.what();
        }
        printf("J2AM\n%s\n%s\n%s\n", result.c_str(), esc(error).c_str(), dump_model(out_model).c_str());
        printf("%s\n", out(assembly_model_to_json_string(out_model)).c_str());
    }
}

static void model_vectors()
{
    for (int i = 0; i < 6; ++i) {
        const bool  nulls   = i % 2 == 0;
        const Model model   = rmodel(nulls);
        model_cases(model, !nulls);
        // Twice: the first call takes the scaling factor into the dirty instances.
        printf("AM2J\n%s\n", out(assembly_model_to_json_string(model)).c_str());
        printf("AM2J\n%s\n", out(assembly_model_to_json_string(model)).c_str());
    }
}

int main(int argc, char **argv)
{
    if (argc < 2)
        return 1;
    if (argc > 2)
        g_tmp = argv[2];
    if (std::string(argv[1]) == "steps")
        steps_vectors();
    else {
        g_seed ^= 0x5DEECE66Dull;
        model_vectors();
    }
    return 0;
}
