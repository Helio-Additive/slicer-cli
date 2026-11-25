#!/usr/bin/env python3
"""Map every Bambu Studio libslic3r source file to its Rust counterpart.

    python3 scripts/parity/file_map.py            # writes docs/rust-engine/FILE_MAP.md
    python3 scripts/parity/file_map.py --json F   # also writes every matched function to F
    python3 scripts/parity/file_map.py --check    # exit 1 if the committed map is out of date

The C++ side is the pinned `references/BambuStudio/src/libslic3r`, the Rust
side `crates/libslic3r-rs/src`. A C++ unit is a `.cpp` with the header of the
same name (or a header alone). Its Rust counterpart is the `.rs` file whose
name has the same words, in any casing style. A Rust module name cannot start
with a digit, so a C++ file whose name does spells the number out in Rust; those
pairings are listed by hand in `RUST_STEM_ALIASES` (`Format/3mf` is
`format/three_mf.rs`) and are compared by the alias's words.

Per pair the script lists, by name, the C++ functions with no Rust function
of the same name, the Rust functions with no C++ function, and for every
matched function an ESTIMATE of how closely the body follows the C++ one.
The estimate is mechanical (see `classify`): it compares the identifiers and
numeric constants of the two bodies and their sizes, and looks for the
markers the port leaves where it knows it deviates. It says where to look; it
does not prove a body faithful.

Needs the `tree_sitter_language_pack` Python package.
"""
import argparse
import json
import os
import re
import sys
from collections import Counter, defaultdict

from tree_sitter_language_pack import get_parser

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CPP_ROOT = os.path.join(REPO, "references", "BambuStudio", "src", "libslic3r")
RS_ROOT = os.path.join(REPO, "crates", "libslic3r-rs", "src")
# The override layer: a file here replaces the engine's file of the same relative path when
# `slicer_cli` is built, so it is what the port has to follow.
OVERRIDE_REL = "libslic3r/bambustudio/libslic3r"
OVERRIDE_ROOT = os.path.join(REPO, *OVERRIDE_REL.split("/"))


def tracked_overrides():
    """Relative paths of the override layer's files that are checked in (a local, untracked
    copy made for a diagnosis must not change the map)."""
    import subprocess
    try:
        out = subprocess.run(["git", "-C", REPO, "ls-files", OVERRIDE_REL], capture_output=True, text=True,
                             check=True).stdout
    except Exception:
        return set()
    return {os.path.relpath(os.path.join(REPO, l), OVERRIDE_ROOT) for l in out.splitlines() if l.strip()}


OVERRIDES = tracked_overrides()


def cpp_source(rel_dir, f):
    rel = os.path.normpath(os.path.join(rel_dir, f))
    if rel in OVERRIDES:
        return os.path.join(OVERRIDE_ROOT, rel)
    return os.path.join(CPP_ROOT, rel_dir, f)
OUT_MD = os.path.join(REPO, "docs", "rust-engine", "FILE_MAP.md")

CPP_EXT = (".cpp", ".hpp", ".h", ".c", ".cc")

# Where a file sits on the way from a model to G-code. Tier 1 is what the
# Benchy and Majora jobs run through, tier 2 supports and what else an FFF
# print can turn on, tier 3 the rest of the FFF library, tier 4 SLA.
TIERS = [
    (4, r"^SLA/|^SLAPrint|^SLA\b"),
    (2, r"^Support/|TreeSupport|SupportMaterial|SupportSpotsGenerator|^Interlocking/|FuzzySkin|^Feature/|"
        r"SlicingAdaptive|OverhangDetector|^GCode/SmallAreaInfillFlowCompensator|Ironing|Brim"),
    (1, r"^Print(Base|Object|ObjectSlice|ObjectSupportMaterial|Apply|Region|Config)?\.|^Print\.|^Layer(Region)?\.|"
        r"^PerimeterGenerator|^Fill/|^GCode\.|^GCode/|^GCodeWriter|^GCodeReader|^Arachne/|^Slicing\.|"
        r"^TriangleMeshSlicer|^TriangleMesh\.|^Clipper|^Flow\.|^Extruder\.|^Extrusion|^Surface|^Polygon|^Polyline|"
        r"^ExPolygon|^Point\.|^Line\.|^MultiPoint|^BoundingBox|^Geometry|^ShortestPath|^BridgeDetector|"
        r"^MultiMaterialSegmentation|^ElephantFootCompensation|^PrincipalComponents2D|^RegionExpansion|"
        r"^Algorithm/|^AABB|^KDTree|^EdgeGrid|^Config\.|^Preset|^Model\.|^ModelArrange|^Format/bbs_3mf|^Format/STL|"
        r"^Tesselate|^Triangulation|^TriangleSelector|^MutablePolygon|^PolygonTrimmer|^VariableWidth|^Utils\.|"
        r"^utils\.|^libslic3r\.|^ArcFitter|^Circle|^Arc|^MedialAxis|^IntersectionPoints|^Int128|^MinAreaBoundingBox|"
        r"^MTUtils|^Thread|^Time|^LocalesUtils|^PlaceholderParser|^CustomGCode|^ParameterUtils|^ProjectTask|"
        r"^calib|^Calib|^FilamentGroup|^ToolOrdering|^Orient|^Arrange|^NozzleVolume|^MultiNozzle|^FlushVolCalc|"
        r"^BuildVolume|^AppConfig|^I18N|^Semver|^Exception|^Technologies|^enum_bitmask|^SVG\.|^Color|^clonable_ptr|"
        r"^StaticMap|^MutablePriorityQueue|^BlacklistedLibraryCheck|^Platform|^FileParserError|^format\.|^miniz"),
]


def tier_of(rel):
    for tier, pat in TIERS:
        if re.search(pat, rel):
            return tier
    return 3


def words(name):
    """`ExPolygonCollection`, `ex_polygon_collection`, `ex-polygon-collection` -> the same word list."""
    out = []
    for part in re.split(r"[^A-Za-z0-9]+", name):
        out += re.findall(r"[A-Z]+(?=[A-Z][a-z])|[A-Z]?[a-z]+|[A-Z]+|[0-9]+", part)
    return [w.lower() for w in out]


def letters(name):
    return re.sub(r"[^a-z0-9]", "", name.lower())


# C++ file stems that start with a digit, and the name the Rust file has instead (a Rust module
# name cannot start with a digit: `mod 3mf;` does not parse). Explicit, one entry per such file.
RUST_STEM_ALIASES = {"3mf": "three_mf"}


def rust_stem(stem):
    """The name the Rust file of the C++ unit `stem` has: the stem itself, or its alias."""
    return RUST_STEM_ALIASES.get(stem, stem)


def read(path):
    with open(path, "rb") as f:
        return f.read()


CPP = get_parser("cpp")
RUST = get_parser("rust")

IDENT_STOP = {
    "std", "this", "self", "size", "begin", "end", "first", "second", "true", "false", "nullptr", "none", "some",
    "i", "j", "k", "it", "x", "y", "z", "p", "a", "b", "c", "n", "len", "push", "pushback", "emplaceback", "back",
    "front", "empty", "isempty", "clone", "iter", "itermut", "intoiter", "collect", "map", "unwrap", "ok", "err",
    "vec", "new", "as", "into", "from", "cast", "staticcast", "get", "at", "clear", "reserve", "move", "ref", "mut",
    "let", "auto", "const", "return", "crate", "super",
}
# What the port writes where it knows it leaves the C++ behind. Plain TODO is not one of them: a faithful
# port carries the C++ comments along, TODOs included.
DEVIATION = re.compile(r"BLOCKED|FIDELITY-NOTE|TODO: [Pp]ort|[Nn]ot (yet )?(ported|implemented)|NOT ported|PARTIAL")
STUB = re.compile(r"\btodo!\s*\(|\bunimplemented!\s*\(")


def norm_number(text):
    t = text.lower().replace("_", "").replace("'", "")
    t = re.sub(r"(f32|f64|i8|i16|i32|i64|i128|u8|u16|u32|u64|u128|usize|isize)$", "", t)
    if not t.startswith("0x"):
        t = re.sub(r"[fulz]+$", "", t)
    try:
        v = int(t, 0) if re.fullmatch(r"0x[0-9a-f]+|[0-9]+", t) else float(t)
    except ValueError:
        return t
    if isinstance(v, float) and v == int(v) and abs(v) < 1e15:
        v = int(v)
    return repr(round(v, 9)) if isinstance(v, float) else str(v)


def body_tokens(node, src, lang):
    """(number of code leaves, set of identifiers, set of numeric constants) of a function body."""
    count = 0
    idents, nums = set(), set()
    stack = [node]
    num_types = ("number_literal",) if lang == "cpp" else ("integer_literal", "float_literal")
    while stack:
        n = stack.pop()
        if n.type in ("comment", "line_comment", "block_comment"):
            continue
        if n.type in num_types:
            count += 1
            nums.add(norm_number(src[n.start_byte:n.end_byte].decode("utf8", "replace")))
            continue
        if n.child_count == 0:
            count += 1
            if n.type in ("identifier", "field_identifier"):
                key = letters(src[n.start_byte:n.end_byte].decode("utf8", "replace"))
                if key and key not in IDENT_STOP:
                    idents.add(key)
            continue
        stack.extend(n.children)
    return count, idents, nums


def cpp_functions(path):
    src = read(path)
    tree = CPP.parse(src)
    out = []

    def text(n):
        return src[n.start_byte:n.end_byte].decode("utf8", "replace")

    def name_of(decl):
        # function_declarator -> its declarator names the function
        d = decl
        while d is not None and d.type not in ("function_declarator",):
            nxt = d.child_by_field_name("declarator")
            if nxt is None:
                return None
            d = nxt
        if d is None:
            return None
        inner = d.child_by_field_name("declarator")
        return None if inner is None else text(inner)

    def visit(n, scope):
        if n.type in ("class_specifier", "struct_specifier", "union_specifier"):
            nm = n.child_by_field_name("name")
            body = n.child_by_field_name("body")
            if body is not None:
                sc = scope + [text(nm)] if nm is not None else scope
                for c in body.children:
                    visit(c, sc)
            return
        if n.type == "namespace_definition":
            body = n.child_by_field_name("body")
            if body is not None:
                for c in body.children:
                    visit(c, scope)
            return
        if n.type == "function_definition":
            decl = n.child_by_field_name("declarator")
            body = n.child_by_field_name("body")
            raw = name_of(decl) if decl is not None else None
            if raw and body is not None:
                raw = re.sub(r"\s+", "", raw)
                raw = re.sub(r"<[^<>]*>", "", raw)
                parts = [p for p in raw.split("::") if p]
                cls = "::".join(scope + parts[:-1]) if (scope or len(parts) > 1) else ""
                fn = parts[-1] if parts else raw
                cnt, ids, nums = body_tokens(body, src, "cpp")
                last_cls = (scope + parts[:-1])[-1] if (scope or len(parts) > 1) else ""
                kind = "fn"
                if fn.startswith("~"):
                    kind = "dtor"
                elif fn.startswith("operator"):
                    kind = "operator"
                elif last_cls and fn == last_cls:
                    kind = "ctor"
                out.append({
                    "name": fn, "cls": cls, "kind": kind, "line": n.start_point[0] + 1,
                    "tokens": cnt, "idents": ids, "nums": nums,
                })
            return  # lambdas and local classes belong to their function
        for c in n.children:
            visit(c, scope)

    visit(tree.root_node, [])
    return out


def rust_functions(path):
    src = read(path)
    tree = RUST.parse(src)
    out = []

    def text(n):
        return src[n.start_byte:n.end_byte].decode("utf8", "replace")

    def is_test_attr(n):
        t = text(n)
        return n.type == "attribute_item" and ("cfg(test)" in t or t.strip() in ("#[test]",) or "#[test]" in t)

    def visit(n, scope, in_test):
        pending_test = False
        for c in n.children:
            if c.type == "attribute_item":
                if is_test_attr(c):
                    pending_test = True
                continue
            if c.type in ("line_comment", "block_comment"):
                continue
            test_here = in_test or pending_test
            pending_test = False
            if c.type == "mod_item":
                body = c.child_by_field_name("body")
                nm = c.child_by_field_name("name")
                if body is not None:
                    visit(body, scope, test_here or (nm is not None and text(nm) in ("tests", "test")))
            elif c.type in ("impl_item", "trait_item"):
                ty = c.child_by_field_name("type") or c.child_by_field_name("name")
                body = c.child_by_field_name("body")
                tname = re.sub(r"<.*", "", text(ty)) if ty is not None else ""
                if body is not None:
                    visit(body, tname.split("::")[-1], test_here)
            elif c.type == "function_item":
                nm = c.child_by_field_name("name")
                body = c.child_by_field_name("body")
                if nm is None or body is None:
                    continue
                cnt, ids, nums = body_tokens(body, src, "rust")
                full = text(c)
                out.append({
                    "name": text(nm), "cls": scope, "line": c.start_point[0] + 1, "tokens": cnt,
                    "idents": ids, "nums": nums, "test": test_here,
                    "deviation": bool(DEVIATION.search(full)), "stub": bool(STUB.search(full)),
                })
            elif c.type == "declaration_list":
                visit(c, scope, test_here)

    visit(tree.root_node, "", False)
    return out


def classify(cf, rf):
    """Estimate how closely a Rust body follows the C++ one: (label, containment, size ratio)."""
    ct, rt = cf["tokens"], rf["tokens"]
    ratio = rt / ct if ct else 1.0
    cset = cf["idents"] | {"#" + n for n in cf["nums"] if n not in ("0", "1", "2")}
    rset = rf["idents"] | {"#" + n for n in rf["nums"] if n not in ("0", "1", "2")}
    cont = len(cset & rset) / len(cset) if cset else 1.0
    if rf["stub"] or (rt <= 8 and ct >= 60):
        return "stub", cont, ratio
    if ct < 25:
        # a getter, a one-line wrapper: too small to measure, counted as ported unless it says otherwise
        return ("partial" if rf["deviation"] and rt < 0.5 * ct else "faithful"), cont, ratio
    if cont < 0.30 or ratio < 0.20:
        return "divergent", cont, ratio
    if cont >= 0.55 and ratio >= 0.50 and not rf["deviation"]:
        return "faithful", cont, ratio
    return "partial", cont, ratio


def collect_cpp():
    units = {}
    for d, _, files in os.walk(CPP_ROOT):
        for f in sorted(files):
            stem, ext = os.path.splitext(f)
            if ext not in CPP_EXT:
                continue
            rel_dir = os.path.relpath(d, CPP_ROOT)
            rel_dir = "" if rel_dir == "." else rel_dir.replace(os.sep, "/")
            units.setdefault((rel_dir, stem), []).append(f)
    return units


def collect_rust():
    files = []
    for d, _, fs in os.walk(RS_ROOT):
        for f in sorted(fs):
            if f.endswith(".rs"):
                rel = os.path.relpath(os.path.join(d, f), RS_ROOT).replace(os.sep, "/")
                files.append(rel)
    return sorted(files)


def build():
    units = collect_cpp()
    rust_files = collect_rust()
    rust_fns = {rel: rust_functions(os.path.join(RS_ROOT, rel)) for rel in rust_files}
    by_letters = defaultdict(list)
    for rel in rust_files:
        stem = os.path.splitext(os.path.basename(rel))[0]
        by_letters[letters(stem)].append(rel)

    # every Rust function by name, to say where a "missing" C++ function lives instead
    rust_index = defaultdict(set)
    for rel, fns in rust_fns.items():
        for f in fns:
            if not f["test"]:
                rust_index[letters(f["name"])].add(rel)

    pairs, unmatched_cpp, used_rust = [], [], set()
    cpp_index = defaultdict(set)
    cpp_fns_by_unit = {}
    for (rel_dir, stem), files in sorted(units.items()):
        fns = []
        for f in files:
            fns += cpp_functions(cpp_source(rel_dir, f))
        cpp_fns_by_unit[(rel_dir, stem)] = fns
        for f in fns:
            cpp_index[letters(f["name"])].add((rel_dir + "/" if rel_dir else "") + stem)

    def dir_key(r):
        d = os.path.dirname(r)
        return "/".join(letters(p) for p in d.split("/")) if d else ""

    def cpp_dir_key(rel_dir):
        return "/".join(letters(p) for p in rel_dir.split("/")) if rel_dir else ""

    # A Rust file belongs to the C++ unit of its name in its own directory when there is one.
    home = set()
    for (rel_dir, stem) in units:
        for c in by_letters.get(letters(rust_stem(stem)), []):
            if dir_key(c) == cpp_dir_key(rel_dir):
                home.add(c)

    twins = []
    for (rel_dir, stem), files in sorted(units.items()):
        rel = (rel_dir + "/" if rel_dir else "") + stem
        want_dir = cpp_dir_key(rel_dir)
        all_cands = by_letters.get(letters(rust_stem(stem)), [])
        same_dir = [c for c in all_cands if dir_key(c) == want_dir]
        # files of this name elsewhere that are not at home with another C++ unit of the same name
        moved = [c for c in all_cands if c not in home]
        cands = same_dir + moved
        fns = cpp_fns_by_unit[(rel_dir, stem)]
        exts = "+".join(sorted(os.path.splitext(f)[1] for f in files))
        if not cands:
            unmatched_cpp.append({
                "cpp": rel, "ext": exts, "tier": tier_of(rel + "."), "functions": len([f for f in fns if f["kind"] == "fn"]),
                "tokens": sum(f["tokens"] for f in fns),
            })
            continue
        primary = cands[0]
        used_rust.add(primary)
        cpp_names = {letters(f["name"]) for f in fns}
        kept = [primary]
        for c in cands[1:]:
            # A second Rust file of this name: the same code under the same name (a twin) when a fair share of
            # its functions are this unit's, otherwise something else that happens to be called the same.
            cf = [f for f in rust_fns[c] if not f["test"]]
            share = sum(1 for f in cf if letters(f["name"]) in cpp_names) / len(cf) if cf else 0.0
            if share >= 0.25:
                used_rust.add(c)
                kept.append(c)
                twins.append({"rust": c, "cpp": rel, "paired": primary, "functions": len(cf),
                              "share": round(share, 2)})
        cands = kept
        rstem = os.path.splitext(os.path.basename(primary))[0]
        name_match = "words" if words(rstem) == words(rust_stem(stem)) else "letters"
        placement = "same" if dir_key(primary) == want_dir else "moved"

        rfns = [f for f in rust_fns[primary] if not f["test"]]
        r_by = defaultdict(list)
        for f in rfns:
            r_by[letters(f["name"])].append(f)
        c_by = defaultdict(list)
        special = Counter()
        for f in fns:
            if f["kind"] != "fn":
                special[f["kind"]] += 1
                continue
            c_by[letters(f["name"])].append(f)

        matched, missing, extra = [], [], []
        for key, cl in sorted(c_by.items()):
            rl = list(r_by.get(key, []))
            scored = []
            for ci, cf in enumerate(cl):
                for ri, rf in enumerate(rl):
                    label, cont, ratio = classify(cf, rf)
                    scored.append((cont, -abs(1 - ratio), ci, ri, label, ratio))
            scored.sort(reverse=True)
            used_c, used_r = set(), set()
            for cont, _, ci, ri, label, ratio in scored:
                if ci in used_c or ri in used_r:
                    continue
                used_c.add(ci)
                used_r.add(ri)
                cf, rf = cl[ci], rl[ri]
                matched.append({
                    "cpp": (cf["cls"] + "::" if cf["cls"] else "") + cf["name"], "cpp_line": cf["line"],
                    "rust": (rf["cls"] + "::" if rf["cls"] else "") + rf["name"], "rust_line": rf["line"],
                    "estimate": label, "shared": round(cont, 2), "size": round(ratio, 2),
                    "cpp_tokens": cf["tokens"],
                })
            for ci, cf in enumerate(cl):
                if ci not in used_c:
                    elsewhere = sorted(rust_index.get(key, set()) - {primary})
                    missing.append({
                        "cpp": (cf["cls"] + "::" if cf["cls"] else "") + cf["name"], "cpp_line": cf["line"],
                        "cpp_tokens": cf["tokens"], "elsewhere": elsewhere[:3], "overload": bool(used_c),
                    })
            for ri, rf in enumerate(rl):
                if ri not in used_r:
                    extra.append(rf)
        for key, rl in sorted(r_by.items()):
            if key not in c_by:
                extra += rl
        extras = []
        for rf in sorted(extra, key=lambda f: f["line"]):
            src_units = sorted(cpp_index.get(letters(rf["name"]), set()) - {rel})
            extras.append({
                "rust": (rf["cls"] + "::" if rf["cls"] else "") + rf["name"], "rust_line": rf["line"],
                "cpp_elsewhere": src_units[:3],
            })
        est = Counter(m["estimate"] for m in matched)
        pairs.append({
            "cpp": rel, "ext": exts, "rust": primary, "other_rust": cands[1:],
            "name_match": name_match, "placement": placement, "tier": tier_of(rel + "."),
            "cpp_functions": sum(len(v) for v in c_by.values()), "rust_functions": len(rfns),
            "special": dict(special), "estimate": dict(est),
            "matched": sorted(matched, key=lambda m: m["cpp_line"]), "missing": sorted(missing, key=lambda m: m["cpp_line"]),
            "extra": extras,
        })

    unmatched_rust = []
    for rel in rust_files:
        if rel in used_rust:
            continue
        stem = os.path.splitext(os.path.basename(rel))[0]
        fns = [f for f in rust_fns[rel] if not f["test"]]
        # which C++ unit do its functions come from, by name?
        votes = Counter()
        for f in fns:
            for u in cpp_index.get(letters(f["name"]), ()):
                votes[u] += 1
        ranked_votes = sorted(votes.items(), key=lambda kv: (-kv[1], kv[0]))[:3]
        guess = [u for u, n in ranked_votes if n >= max(2, len(fns) // 4)]
        kind = "module" if stem in ("mod", "lib", "main") else "no C++ file of this name"
        namesakes = sorted((d + "/" if d else "") + st for (d, st) in units
                           if letters(rust_stem(st)) == letters(stem))
        if namesakes:
            kind = "same name as a C++ file in another directory"
            guess = namesakes
        unmatched_rust.append({"rust": rel, "kind": kind, "functions": len(fns), "likely_source": guess})
    # directories
    cpp_dirs = set()
    for (d, _) in units:
        while d:
            cpp_dirs.add(d)
            d = os.path.dirname(d)
    cpp_dirs = sorted(cpp_dirs)
    rs_dirs = sorted({os.path.dirname(r) for r in rust_files if os.path.dirname(r)})
    dirs = []
    for d in cpp_dirs:
        hit = [r for r in rs_dirs if cpp_dir_key(d) == dir_key(r + "/x")]
        same = bool(hit) and [words(p) for p in d.split("/")] == [words(p) for p in hit[0].split("/")]
        dirs.append({"cpp": d, "rust": hit[0] if hit else "", "match": "words" if same else ("letters" if hit else "none")})
    rs_only_dirs = [r for r in rs_dirs if not any(cpp_dir_key(d) == dir_key(r + "/x") for d in cpp_dirs)]
    return {"pairs": pairs, "cpp_without_rust": unmatched_cpp, "rust_without_cpp": unmatched_rust, "twins": twins,
            "dirs": dirs, "rust_only_dirs": rs_only_dirs}


def render(data):
    pairs = data["pairs"]
    L = []
    w = L.append
    tot = Counter()
    for p in pairs:
        tot.update(p["estimate"])
    n_missing = sum(len(p["missing"]) for p in pairs)
    n_missing_else = sum(1 for p in pairs for m in p["missing"] if m["elsewhere"])
    n_missing_ovl = sum(1 for p in pairs for m in p["missing"] if m["overload"] and not m["elsewhere"])
    n_missing_small = sum(1 for p in pairs for m in p["missing"]
                          if not m["overload"] and not m["elsewhere"] and m["cpp_tokens"] < 25)
    n_extra = sum(len(p["extra"]) for p in pairs)
    n_cpp_fn = sum(p["cpp_functions"] for p in pairs) + sum(u["functions"] for u in data["cpp_without_rust"])
    n_matched = sum(tot.values())
    w("# File map: Bambu Studio `libslic3r` and its Rust port")
    w("")
    w("Generated by `python3 scripts/parity/file_map.py`; do not edit by hand. Regenerate it in every round that")
    w("touches `crates/libslic3r-rs/src` (`--check` fails when the committed copy is stale). `--json <file>` writes")
    w("the same data with every matched function, the ones estimated faithful included.")
    w("")
    w("C++ side: `references/BambuStudio/src/libslic3r` at the pin. Rust side: `crates/libslic3r-rs/src`.")
    if OVERRIDES:
        w("")
        w("Where the override layer (`" + OVERRIDE_REL + "`) carries a file, that file is read in place of the")
        w("engine's, because it is the one `slicer_cli` compiles: "
          + ", ".join("`" + o + "`" for o in sorted(OVERRIDES)) + ".")
    w("")
    w("## How to read it")
    w("")
    w("- A **C++ unit** is a `.cpp` together with the header of the same name, or a header on its own.")
    w("- Its **Rust file** is the `.rs` whose name has the same words in another casing style")
    w("  (`ExPolygonCollection` and `ex_polygon_collection`). Where only the letters agree and the word breaks")
    w("  do not (`ExPolygon` and `expolygon`, `GCode` and `gcode`), the pair is listed and flagged *word breaks*.")
    w("- Functions are matched by name inside a pair, in any casing style. Constructors, destructors and")
    w("  operators have no name to match by and are only counted. Rust test functions are left out.")
    w("- **Missing** is a C++ function with no Rust function of that name in the paired file; when a function of")
    w("  that name exists in another Rust file, that file is named. **Rust only** is the reverse.")
    w("- The **estimate** for a matched function is mechanical. *stub*: the Rust body is `todo!`/`unimplemented!`")
    w("  or a few tokens against a large C++ body. *divergent*: under 30% of the C++ body's identifiers and")
    w("  numeric constants appear in the Rust body, or the Rust body is under a fifth of its size. *faithful*: at")
    w("  least 60% appear, the Rust body is at least half the size, and it carries none of the port's deviation")
    w("  markers (`BLOCKED`, `FIDELITY-NOTE`, `TODO: port`, \"not ported\", \"not implemented\", `PARTIAL`).")
    w("  *partial*: everything between. Bodies under 25 tokens are counted as faithful. The estimate tells where to")
    w("  look first; a *faithful* here is not a proof, and a gated function is judged with both arms in its body.")
    w("  A body that reaches the same result through other helpers than the C++ names reads as *partial*.")
    w("- **Tier** 1 is the way from a model to G-code for an FFF print without supports, which is what the")
    w("  Benchy and Majora jobs take; 2 supports and other FFF options; 3 the rest of the FFF library; 4 SLA.")
    w("  The tiers are assigned by file-name patterns in the script, not measured from a run.")
    w("")
    w("## Headline")
    w("")
    n_units = len(pairs) + len(data["cpp_without_rust"])
    w("| | |")
    w("|-|-|")
    w(f"| C++ units | {n_units} |")
    w(f"| with a Rust file of the same name | {len(pairs)} ({100 * len(pairs) / n_units:.0f}%) |")
    w(f"| of those, same words | {sum(1 for p in pairs if p['name_match'] == 'words')} |")
    w(f"| of those, same letters but other word breaks | {sum(1 for p in pairs if p['name_match'] == 'letters')} |")
    w(f"| of those, in another directory than the C++ file | {sum(1 for p in pairs if p['placement'] == 'moved')} |")
    w(f"| C++ units with no Rust file | {len(data['cpp_without_rust'])} |")
    n_mod = sum(1 for r in data["rust_without_cpp"] if r["kind"] == "module")
    w(f"| Rust files with no C++ file of that name | {len(data['rust_without_cpp']) - n_mod} "
      f"(plus {n_mod} `mod.rs`/`lib.rs`/`main.rs`) |")
    w(f"| Rust files that share a C++ unit's name with another Rust file | {len(data['twins'])} |")
    w(f"| C++ functions (named, with a body) | {n_cpp_fn} |")
    w(f"| matched to a Rust function in the paired file | {n_matched} ({100 * n_matched / n_cpp_fn:.0f}%) |")
    for k in ("faithful", "partial", "divergent", "stub"):
        w(f"| estimate: {k} | {tot[k]} ({100 * tot[k] / max(1, n_matched):.0f}% of matched) |")
    w(f"| missing in the paired Rust file | {n_missing} |")
    w(f"| of those, a function of that name exists in another Rust file | {n_missing_else} |")
    w(f"| of the rest, another overload of the name is ported | {n_missing_ovl} |")
    w(f"| of the rest, under 25 tokens (accessors, one-liners) | {n_missing_small} |")
    w(f"| of the rest, 25 tokens or more | {n_missing - n_missing_else - n_missing_ovl - n_missing_small} |")
    w(f"| in a C++ unit with no Rust file | {sum(u['functions'] for u in data['cpp_without_rust'])} |")
    w(f"| Rust functions with no C++ function of that name in the pair | {n_extra} |")
    w("")
    w("By tier:")
    w("")
    w("| Tier | C++ units | paired | C++ functions | matched | faithful | partial | divergent | stub | missing |")
    w("|-|-|-|-|-|-|-|-|-|-|")
    for t in (1, 2, 3, 4):
        ps = [p for p in pairs if p["tier"] == t]
        us = [u for u in data["cpp_without_rust"] if u["tier"] == t]
        e = Counter()
        for p in ps:
            e.update(p["estimate"])
        cf = sum(p["cpp_functions"] for p in ps) + sum(u["functions"] for u in us)
        w(f"| {t} | {len(ps) + len(us)} | {len(ps)} | {cf} | {sum(e.values())} | {e['faithful']} | {e['partial']} | "
          f"{e['divergent']} | {e['stub']} | {sum(len(p['missing']) for p in ps) + sum(u['functions'] for u in us)} |")
    w("")

    def real_missing(p):
        return [m for m in p["missing"] if not m["elsewhere"] and not m["overload"] and m["cpp_tokens"] >= 25]

    def gap(p):
        e = p["estimate"]
        return len(real_missing(p)) + e.get("stub", 0) + e.get("divergent", 0)

    w("## Biggest gaps")
    w("")
    w("Pairs ordered by tier, then by the number of C++ functions that are stubs, divergent, or missing in every")
    w("Rust file (overloads of a ported name and bodies under 25 tokens left out). *Missing, real* is that last count.")
    w("")
    w("| Tier | C++ unit | Rust file | C++ fns | faithful | partial | divergent | stub | missing | missing, real |")
    w("|-|-|-|-|-|-|-|-|-|-|")
    ranked = sorted(pairs, key=lambda p: (p["tier"], -gap(p), p["cpp"]))
    for p in [p for p in ranked if gap(p) > 0][:80]:
        e = p["estimate"]
        w(f"| {p['tier']} | `{p['cpp']}` | `{p['rust']}` | {p['cpp_functions']} | {e.get('faithful', 0)} | "
          f"{e.get('partial', 0)} | {e.get('divergent', 0)} | {e.get('stub', 0)} | {len(p['missing'])} | "
          f"{len(real_missing(p))} |")
    w("")
    w("## C++ units with no Rust file")
    w("")
    w("| Tier | C++ unit | Files | Functions | Body tokens |")
    w("|-|-|-|-|-|")
    for u in sorted(data["cpp_without_rust"], key=lambda u: (u["tier"], -u["tokens"], u["cpp"])):
        w(f"| {u['tier']} | `{u['cpp']}` | {u['ext']} | {u['functions']} | {u['tokens']} |")
    w("")
    w("## Rust files whose name matches no C++ file")
    w("")
    w("`mod.rs`, `lib.rs` and `main.rs` are module files. For the others, *likely source* is the C++ unit most of")
    w("their function names come from, when there is one.")
    w("")
    w("| Rust file | Kind | Functions | Likely source |")
    w("|-|-|-|-|")
    for r in data["rust_without_cpp"]:
        src = ", ".join(f"`{s}`" for s in r["likely_source"]) or ""
        w(f"| `{r['rust']}` | {r['kind']} | {r['functions']} | {src} |")
    w("")
    w("## Rust files that share a C++ unit's name with another Rust file")
    w("")
    w("The C++ unit is paired with the first; the second holds code under the same name. One of the two is the")
    w("port, the other has to be merged into it or renamed after what it really ports.")
    w("")
    w("| Rust file | Functions | Of them named like the C++ unit's | C++ unit | Paired with |")
    w("|-|-|-|-|-|")
    for t in data["twins"]:
        w(f"| `{t['rust']}` | {t['functions']} | {int(t['share'] * 100)}% | `{t['cpp']}` | `{t['paired']}` |")
    w("")
    w("## Directories")
    w("")
    w("| C++ directory | Rust directory | Name |")
    w("|-|-|-|")
    for d in data["dirs"]:
        note = {"words": "same words", "letters": "same letters, other word breaks", "none": "no Rust directory"}[d["match"]]
        w(f"| `{d['cpp']}` | {('`' + d['rust'] + '`') if d['rust'] else ''} | {note} |")
    for r in data["rust_only_dirs"]:
        w(f"| | `{r}` | no C++ directory |")
    w("")
    w("## Pairs whose names differ in more than casing")
    w("")
    w("Same letters, other word breaks. Renaming them is a decision of its own; they are listed, not changed.")
    w("")
    w("| C++ unit | Rust file | C++ words | Rust words |")
    w("|-|-|-|-|")
    for p in pairs:
        if p["name_match"] == "letters":
            cs = os.path.basename(p["cpp"])
            rs = os.path.splitext(os.path.basename(p["rust"]))[0]
            w(f"| `{p['cpp']}` | `{p['rust']}` | {' '.join(words(cs))} | {' '.join(words(rs))} |")
    w("")
    w("## Pairs in another directory than the C++ file")
    w("")
    w("| C++ unit | Rust file |")
    w("|-|-|")
    for p in pairs:
        if p["placement"] == "moved":
            w(f"| `{p['cpp']}` | `{p['rust']}` |")
    w("")
    w("## Every pair")
    w("")
    for p in sorted(pairs, key=lambda p: (p["tier"], p["cpp"])):
        e = p["estimate"]
        sp = p["special"]
        w(f"### `{p['cpp']}` ({p['ext']}) and `{p['rust']}`")
        w("")
        flags = []
        if p["name_match"] == "letters":
            flags.append("word breaks differ")
        if p["placement"] == "moved":
            flags.append("other directory")
        w(f"Tier {p['tier']}. C++ functions {p['cpp_functions']}"
          + (f" (plus {sp.get('ctor', 0)} constructors, {sp.get('dtor', 0)} destructors, {sp.get('operator', 0)} operators)"
             if sp else "")
          + f", Rust functions {p['rust_functions']}. Matched {sum(e.values())}: faithful {e.get('faithful', 0)}, "
            f"partial {e.get('partial', 0)}, divergent {e.get('divergent', 0)}, stub {e.get('stub', 0)}."
          + (" Flags: " + ", ".join(flags) + "." if flags else ""))
        w("")
        not_ok = [m for m in p["matched"] if m["estimate"] != "faithful"]
        if not_ok:
            w("Not estimated faithful (shared identifiers and constants, Rust size over C++ size):")
            w("")
            for m in not_ok:
                w(f"- {m['estimate']}: `{m['cpp']}` :{m['cpp_line']} and `{m['rust']}` :{m['rust_line']} "
                  f"({int(m['shared'] * 100)}%, {m['size']}x)")
            w("")
        if p["missing"]:
            w("Missing in the Rust file:")
            w("")
            for m in p["missing"]:
                where = (" — in " + ", ".join(f"`{x}`" for x in m["elsewhere"])) if m["elsewhere"] else ""
                ovl = ", overload" if m["overload"] else ""
                w(f"- `{m['cpp']}` :{m['cpp_line']} ({m['cpp_tokens']} tokens{ovl}){where}")
            w("")
        if p["extra"]:
            w("Rust only:")
            w("")
            for x in p["extra"]:
                where = (" — C++ has it in " + ", ".join(f"`{u}`" for u in x["cpp_elsewhere"])) if x["cpp_elsewhere"] else ""
                w(f"- `{x['rust']}` :{x['rust_line']}{where}")
            w("")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="fail if the committed map differs from a fresh one")
    ap.add_argument("--json", metavar="FILE", help="also write the data, every matched function included")
    args = ap.parse_args()
    data = build()
    md = render(data)
    if args.check:
        ok = os.path.exists(OUT_MD) and open(OUT_MD).read() == md
        print("file map is up to date" if ok else "file map is stale: run scripts/parity/file_map.py")
        sys.exit(0 if ok else 1)
    os.makedirs(os.path.dirname(OUT_MD), exist_ok=True)
    with open(OUT_MD, "w") as f:
        f.write(md)
    if args.json:
        with open(args.json, "w") as f:
            f.write(json.dumps(data, indent=1, sort_keys=True) + "\n")
    print(md.split("## Biggest gaps")[0])


if __name__ == "__main__":
    main()
