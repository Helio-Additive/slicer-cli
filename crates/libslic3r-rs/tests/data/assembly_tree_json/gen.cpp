// Reference vectors for crates/libslic3r-rs/src/format/assembly_tree_json.rs, taken from the C++
// at the pin.
//
// The program compiles references/BambuStudio/src/libslic3r/Format/AssemblyTreeJson.cpp as it is,
// with the nlohmann::json the engine bundles.
//
// Build and run, from the repository root (macOS arm64 clang is the reference build):
//   D=crates/libslic3r-rs/tests/data/assembly_tree_json
//   clang++ -std=c++17 -O2 -ffp-contract=on -I references/BambuStudio/src \
//       -I references/BambuStudio/src/libslic3r $D/gen.cpp -o /tmp/atj_gen
//   /tmp/atj_gen > $D/vectors.txt
//
// Output: lines. A line is either one text (escaped: \\ \n \r \s for a space, \xHH for another
// control byte, \e alone for the empty text) or tokens separated by spaces.
//   TO / <tree> / out               AssemblyTreeData::to_json_string() of the tree
//   FROM / text / ok / error / <tree> / out
//                                   AssemblyTreeData::from_json_string(text, tree, &error) on a
//                                   tree that held one node, one root and one checked index
//                                   before; then the tree and its to_json_string()
// <tree>: A nnodes, per node `id parent_id object_idx volume_idx selectable uid label n
//   children..`, then `n roots.. n checked_node_indices..`.

#include "Format/AssemblyTreeJson.cpp"

#include <cstdint>
#include <cstdio>

using namespace Slic3r;

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

static std::string ints(const std::vector<int> &v)
{
    std::string s = " " + std::to_string(v.size());
    for (int i : v)
        s += " " + std::to_string(i);
    return s;
}

static std::string dump_tree(const AssemblyTreeData &tree)
{
    std::string s = "A " + std::to_string(tree.nodes.size());
    for (const AssemblyTreeNodeData &n : tree.nodes)
        s += " " + std::to_string(n.id) + " " + std::to_string(n.parent_id) + " " + std::to_string(n.object_idx) + " " + std::to_string(n.volume_idx) +
             " " + std::to_string((int) n.selectable) + " " + esc(n.uid) + " " + esc(n.label) + ints(n.children);
    return s + ints(tree.roots) + ints(tree.checked_node_indices);
}

static uint64_t g_seed = 0x243F6A8885A308D3ull;
static uint32_t rnd()
{
    g_seed = g_seed * 6364136223846793005ull + 1442695040888963407ull;
    return (uint32_t)(g_seed >> 33);
}
static int  rint(int n) { return (int) (rnd() % (uint32_t) n); }
static bool chance(int pct) { return rint(100) < pct; }

static const char *TEXTS[] = {"", "a", "uid-1", "W\xC3\xBCrfel", "two words", "q\"\\\n\t\x01", "\xF0\x9F\x98\x80"};
static std::string rtext() { return TEXTS[rint(7)]; }

static json wrong()
{
    switch (rint(16)) {
    case 0: return nullptr;
    case 1: return true;
    case 2: return false;
    case 3: return 2.75;
    case 4: return -2.75;
    case 5: return "x";
    case 6: return json::array();
    case 7: return json::array({1, true, 2.9, -7});
    case 8: return json::array({1, "a"});
    case 9: return json::object({{"k", 1}});
    case 10: return 4294967297ll;
    case 11: return -4294967295ll;
    case 12: return 18446744073709551615ull;
    case 13: return 1e300;
    case 14: return json::array({json::array()});
    default: return -1e300;
    }
}

template<class F> static void put(json &j, const char *key, F proper, int present)
{
    const int r = rint(100);
    if (r < present)
        j[key] = proper();
    else if (r < present + (100 - present) / 2) {
    } else
        j[key] = wrong();
}

static json rints()
{
    json a = json::array();
    for (int i = rint(5); i > 0; --i)
        a.push_back(rint(12) - 2);
    return a;
}

static json rnode()
{
    json j = json::object();
    put(j, "id", [] { return json(rint(12) - 2); }, 86);
    put(j, "parent_id", [] { return json(rint(12) - 2); }, 86);
    put(j, "object_idx", [] { return json(rint(12) - 2); }, 86);
    put(j, "volume_idx", [] { return json(rint(12) - 2); }, 86);
    put(j, "selectable", [] { return json(chance(50)); }, 86);
    put(j, "uid", [] { return json(rtext()); }, 86);
    put(j, "label", [] { return json(rtext()); }, 86);
    put(j, "children", rints, 86);
    return j;
}

static json rdoc()
{
    json j = json::object();
    put(j, "version", [] { return json(1); }, 80);
    put(j, "roots", rints, 86);
    put(j, "checked_node_indices", rints, 60);
    put(j, "nodes", [] {
        json a = json::array();
        for (int i = rint(6); i > 0; --i)
            a.push_back(chance(95) ? rnode() : wrong());
        return a;
    }, 90);
    return j;
}

static AssemblyTreeData rtree()
{
    AssemblyTreeData tree;
    for (int i = rint(6); i > 0; --i) {
        AssemblyTreeNodeData n;
        if (chance(85)) {
            n.id         = rint(2000) - 1000;
            n.parent_id  = rint(12) - 2;
            n.object_idx = rint(12) - 2;
            n.volume_idx = chance(5) ? INT32_MIN : (chance(5) ? INT32_MAX : rint(12) - 2);
            n.selectable = chance(50);
            n.uid        = rtext();
            n.label      = rtext();
            for (int k = rint(5); k > 0; --k)
                n.children.push_back(rint(12) - 2);
        }
        tree.nodes.push_back(n);
    }
    for (int k = rint(5); k > 0; --k)
        tree.roots.push_back(rint(12) - 2);
    for (int k = rint(5); k > 0; --k)
        tree.checked_node_indices.push_back(rint(12) - 2);
    return tree;
}

static void case_from(const std::string &text)
{
    AssemblyTreeData tree;
    tree.nodes.emplace_back();
    tree.roots.push_back(7);
    tree.checked_node_indices.push_back(8);
    std::string error = "unset";
    const bool  ok    = AssemblyTreeData::from_json_string(text, tree, &error);
    printf("FROM\n%s\n%d\n%s\n%s\n%s\n", esc(text).c_str(), (int) ok, esc(error).c_str(), dump_tree(tree).c_str(), esc(tree.to_json_string()).c_str());
}

int main()
{
    for (int i = 0; i < 40; ++i) {
        const AssemblyTreeData tree = rtree();
        printf("TO\n%s\n%s\n", dump_tree(tree).c_str(), esc(tree.to_json_string()).c_str());
        case_from(tree.to_json_string());
    }
    const char *fixed[] = {
        "",
        "{",
        "null",
        "[]",
        "5",
        "\"s\"",
        "true",
        "{}",
        "{\"nodes\":[]}",
        "{\"nodes\":{}}",
        "{\"nodes\":5,\"roots\":[]}",
        "{\"nodes\":[{}]}",
        "{\"nodes\":[{},{\"id\":5},{\"id\":-3},{\"id\":0}]}",
        "{\"nodes\":[{},{\"id\":5}],\"checked_node_indices\":[]}",
        "{\"nodes\":[{}],\"checked_node_indices\":null}",
        "{\"nodes\":[null]}",
        "{\"nodes\":[[]]}",
        "{\"nodes\":[{\"id\":\"5\"}]}",
        "{\"nodes\":[{\"id\":true,\"parent_id\":false,\"object_idx\":2.9,\"volume_idx\":-2.9}]}",
        "{\"nodes\":[{\"id\":-0,\"parent_id\":-0.0,\"object_idx\":1e2,\"volume_idx\":4294967296}]}",
        "{\"nodes\":[{\"selectable\":1}]}",
        "{\"nodes\":[{\"selectable\":null}]}",
        "{\"nodes\":[{\"uid\":5}]}",
        "{\"nodes\":[{\"label\":[]}]}",
        "{\"nodes\":[{\"children\":{}}]}",
        "{\"nodes\":[{\"children\":[1,[2]]}]}",
        "{\"nodes\":[{\"children\":[1,true,2.5,null]}]}",
        "{\"roots\":\"r\"}",
        "{\"roots\":[1,2,\"3\"]}",
        "{\"roots\":[true,false,1.9,-1.9,3000000000,-3000000000,1e300,-1e300]}",
        "{\"roots\":[1],\"roots\":[2,3]}",
        "{\"roots\":[1]} trailing",
        "\xEF\xBB\xBF{\"roots\":[1]}",
        " {\n\"version\" : 9 , \"roots\" : [ 4 ] , \"nodes\" : [ { \"uid\" : \"\\u00e9\\ud83d\\ude00\" , \"label\" : \"a\\tb\" } ] }\n",
    };
    for (const char *text : fixed)
        case_from(text);
    for (int i = 0; i < 120; ++i)
        case_from(rdoc().dump());
    return 0;
}
