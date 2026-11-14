#!/usr/bin/env python3
"""The port checklist: every C++ source file of Bambu Studio's libslic3r at the pin, its classes
and functions, the Rust file each maps to and a status.

    python3 scripts/parity/port_checklist.py            # writes docs/parity/PORT_CHECKLIST.md
    python3 scripts/parity/port_checklist.py --check    # exit 1 when the committed copy is stale

It reads the same data as scripts/parity/file_map.py (same parser, same pairing, same estimate) and
adds two things kept by hand in scripts/parity/port_checklist_verified.json: the functions that are
verified, each with its evidence, and nothing else. Regenerate it in every round.

Status of a function:
  not started  no Rust function of that name anywhere, or the unit has no Rust file
  partial      a Rust function of that name exists but not in the file named after the C++ file, or
               only another overload of it is ported, or the body is estimated partial, divergent
               or a stub
  ported       a Rust function of that name is in the paired file and its body is estimated faithful
  verified     ported, and listed in port_checklist_verified.json with evidence: a twin probe that
               agreed with the C++ over a fixture, or a test that compares with C++ output
"""
import argparse
import json
import os
import re
import sys
from collections import OrderedDict, defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import file_map as fm  # noqa: E402

OUT_MD = os.path.join(fm.REPO, "docs", "parity", "PORT_CHECKLIST.md")
VERIFIED_JSON = os.path.join(HERE, "port_checklist_verified.json")
REVIEWED_JSON = os.path.join(HERE, "port_checklist_reviewed.json")
SHIMS = ["clipper-z-sys", "clipper2-z-sys", "eigen-transform-sys", "boost-voronoi-sys"]
ORDER = ["not started", "partial", "ported", "verified"]
MARK = {"not started": "[ ]", "partial": "[~]", "ported": "[x]", "verified": "[v]"}


def load_verified():
    if not os.path.exists(VERIFIED_JSON):
        return {}
    out = {}
    for e in json.load(open(VERIFIED_JSON)):
        for fn in e["functions"]:
            out[(e["unit"], fn)] = (e["round"], e["evidence"])
    return out


def load_reviewed():
    """Functions a round read side by side with the C++ and found to be the same logic, where the
    mechanical estimate says partial or divergent (other identifiers for the same thing: `upgrade()`
    for `lock()`, scalars for an Eigen vector)."""
    if not os.path.exists(REVIEWED_JSON):
        return {}
    out = {}
    for e in json.load(open(REVIEWED_JSON)):
        for fn in e["functions"]:
            out[(e["unit"], fn)] = e["round"]
    return out


def unit_files(units, name):
    rel_dir, stem = os.path.split(name)
    return rel_dir, sorted(units.get((rel_dir, stem), []))


def qual(cls, name):
    return (cls + "::" + name) if cls else name


def rows_of_pair(p, verified, reviewed):
    """One row per named C++ function of a paired unit."""
    rows = []
    for m in p["matched"]:
        est = m["estimate"]
        st = "ported" if est == "faithful" else "partial"
        note = "" if est == "faithful" else est
        key = (p["cpp"], m["cpp"])
        if key in reviewed and st == "partial" and est != "stub":
            st = "ported"
            note = "read against the C++ in R%s (the estimate says %s)" % (reviewed[key], est)
        if key in verified:
            # Evidence against the C++ outranks the mechanical estimate of the body.
            st = "verified"
            note = "R%s: %s" % verified[key]
            if est != "faithful":
                note += " (the body reads as %s to the estimate)" % est
        rows.append((m["cpp"], m["cpp_line"], m["cpp_tokens"], p["rust"], m.get("rust_line"), st, note))
    for m in p["missing"]:
        if m.get("elsewhere"):
            st, rust, note = "partial", ", ".join(m["elsewhere"][:3]), "not in the paired file"
        elif m.get("overload"):
            st, rust, note = "partial", p["rust"], "another overload of the name is ported"
        else:
            st, rust, note = "not started", p["rust"], ""
        rows.append((m["cpp"], m["cpp_line"], m["cpp_tokens"], rust, None, st, note))
    return rows


def rows_of_unpaired(units, name, rust_index):
    rel_dir, files = unit_files(units, name)
    rows = []
    for f in files:
        for c in fm.cpp_functions(fm.cpp_source(rel_dir, f)):
            if c["kind"] != "fn":
                continue
            where = sorted(rust_index.get(fm.letters(c["name"]), ()))
            if where:
                rows.append((qual(c["cls"], c["name"]), c["line"], c["tokens"], ", ".join(where[:3]), None,
                             "partial", "a function of this name is in another Rust file"))
            else:
                rows.append((qual(c["cls"], c["name"]), c["line"], c["tokens"], "", None, "not started", ""))
    return rows


# C++ the engine compiles that sits outside `src/libslic3r`: a unit name for the checklist, the
# files, and the Rust file that ports it. ClipperLib is what `clipper-z-sys` wraps.
EXTRA_UNITS = [
    ("clipper/clipper", ["clipper/clipper.cpp", "clipper/clipper.hpp"], "clipper.rs"),
    ("clipper2/clipper.core", ["clipper2/Clipper2Lib/include/clipper2/clipper.core.h"], "clipper2/clipper_core.rs"),
    ("clipper2/clipper.engine", ["clipper2/Clipper2Lib/src/clipper.engine.cpp",
                                 "clipper2/Clipper2Lib/include/clipper2/clipper.engine.h"], "clipper2/clipper_engine.rs"),
    ("clipper2/clipper.offset", ["clipper2/Clipper2Lib/src/clipper.offset.cpp",
                                 "clipper2/Clipper2Lib/include/clipper2/clipper.offset.h"], "clipper2/clipper_offset.rs"),
    ("clipper2/clipper.rectclip", ["clipper2/Clipper2Lib/src/clipper.rectclip.cpp",
                                   "clipper2/Clipper2Lib/include/clipper2/clipper.rectclip.h"], "clipper2/clipper_rectclip.rs"),
]


def rows_of_extra(name, files, rust, verified):
    """Rows of a unit outside libslic3r: a function is ported when the Rust file has one of
    that name, verified when the evidence list names it."""
    src_root = os.path.dirname(fm.CPP_ROOT)
    rs_path = os.path.join(fm.RS_ROOT, rust)
    have = {}
    if os.path.exists(rs_path):
        for f in fm.rust_functions(rs_path):
            if not f["test"]:
                have.setdefault(fm.letters(f["name"]), (f["line"], bool(f.get("deviation"))))
    rows, special, credited = [], {}, set()
    for rel in files:
        for c in fm.cpp_functions(os.path.join(src_root, rel)):
            if c["kind"] != "fn":
                special[c["kind"]] = special.get(c["kind"], 0) + 1
                continue
            q = qual(c["cls"], c["name"])
            line, deviates = have.get(fm.letters(c["name"]), (None, False))
            if line is None:
                rows.append((q, c["line"], c["tokens"], rust, None, "not started", ""))
            elif q in credited:
                # The Rust file has one function of this name: it stands for the first overload.
                rows.append((q, c["line"], c["tokens"], rust, None, "partial",
                             "another overload of the name is ported"))
            elif deviates:
                # The Rust body carries a deviation marker (FIDELITY-NOTE and the like).
                credited.add(q)
                rows.append((q, c["line"], c["tokens"], rust, line, "partial",
                             "the Rust body is marked as deviating from the C++"))
            elif credited.add(q) is None and (name, q) in verified:
                rows.append((q, c["line"], c["tokens"], rust, line, "verified", "R%s: %s" % verified[(name, q)]))
            else:
                rows.append((q, c["line"], c["tokens"], rust, line, "ported", ""))
    return rows, special


def shim_calls():
    """Extern functions of each shim crate that the engine calls, with the number of call sites."""
    out = OrderedDict()
    for crate in SHIMS:
        mod = crate.replace("-", "_")
        counts = defaultdict(int)
        for d, _, fs in os.walk(fm.RS_ROOT):
            for f in fs:
                if not f.endswith(".rs"):
                    continue
                src = open(os.path.join(d, f), errors="replace").read()
                for m in re.finditer(re.escape(mod) + r"::([a-z][A-Za-z0-9_]*)", src):
                    counts[m.group(1)] += 1
        out[crate] = dict(sorted(counts.items()))
    return out


def build():
    data = fm.build()
    units = fm.collect_cpp()
    verified = load_verified()
    reviewed = load_reviewed()
    rust_index = defaultdict(set)
    for rel in fm.collect_rust():
        for f in fm.rust_functions(os.path.join(fm.RS_ROOT, rel)):
            if not f["test"]:
                rust_index[fm.letters(f["name"])].add(rel)

    files = []
    used = set()
    for p in data["pairs"]:
        rows = rows_of_pair(p, verified, reviewed)
        used.update(k for k in verified if k[0] == p["cpp"] and any(r[0] == k[1] for r in rows))
        files.append({"cpp": p["cpp"], "ext": p["ext"], "tier": p["tier"], "rust": p["rust"],
                      "name_match": p["name_match"], "special": p.get("special", {}), "rows": rows})
    for u in data["cpp_without_rust"]:
        files.append({"cpp": u["cpp"], "ext": u["ext"], "tier": u["tier"], "rust": "",
                      "name_match": "", "special": {}, "rows": rows_of_unpaired(units, u["cpp"], rust_index)})
    for name, cpp_files, rust in EXTRA_UNITS:
        rows, special = rows_of_extra(name, cpp_files, rust, verified)
        used.update(k for k in verified if k[0] == name and any(r[0] == k[1] for r in rows))
        files.append({"cpp": name, "ext": ".cpp+.hpp (outside libslic3r)", "tier": 1,
                      "rust": rust if os.path.exists(os.path.join(fm.RS_ROOT, rust)) else "",
                      "name_match": "words", "special": special, "rows": rows})
    files.sort(key=lambda f: f["cpp"].lower())
    stale = sorted(k for k in verified if k not in used)
    return files, stale, shim_calls()


def tally(rows):
    n = {s: 0 for s in ORDER}
    t = {s: 0 for s in ORDER}
    for r in rows:
        n[r[5]] += 1
        t[r[5]] += max(r[2], 1)
    return n, t


def pct(a, b):
    return "%.1f%%" % (100.0 * a / b) if b else "n/a"


def file_status(f):
    n, _ = tally(f["rows"])
    total = sum(n.values())
    if not f["rust"]:
        return "not started"
    if total == 0:
        return "ported"
    if n["verified"] == total:
        return "verified"
    if n["ported"] + n["verified"] == total:
        return "ported"
    return "partial"


def render(files, stale, shims):
    all_rows = [r for f in files for r in f["rows"]]
    n, t = tally(all_rows)
    total, ttotal = sum(n.values()), sum(t.values())
    done, tdone = n["ported"] + n["verified"], t["ported"] + t["verified"]
    fstat = {s: 0 for s in ORDER}
    for f in files:
        fstat[file_status(f)] += 1
    shim_fns = sum(len(v) for v in shims.values())

    o = []
    w = o.append
    w("# Port checklist: Bambu Studio `libslic3r` to Rust\n")
    w("Generated by `python3 scripts/parity/port_checklist.py`; do not edit by hand. Regenerate it in every round")
    w("(`--check` fails when the committed copy is stale). The verified entries and their evidence are kept by hand")
    w("in `scripts/parity/port_checklist_verified.json`.\n")
    w("C++ side: `references/BambuStudio/src/libslic3r` at the pin (an override-layer file is read in its place),")
    w("and the C++ the engine compiles from outside it: " + ", ".join("`src/%s`" % f for _, fs, _ in EXTRA_UNITS for f in fs) + ".")
    w("Rust side: `crates/libslic3r-rs/src`. The parser, the pairing of files and the estimate of how closely a")
    w("body follows are those of [`../rust-engine/FILE_MAP.md`](../rust-engine/FILE_MAP.md).\n")
    w("## Overall\n")
    w("**%s of the named C++ functions are ported or verified (%d of %d); %s by size of body.**\n" % (
        pct(done, total), done, total, pct(tdone, ttotal)))
    w("| Status | Functions | Share | By body tokens |")
    w("|-|-|-|-|")
    for s in ORDER:
        w("| %s `%s` | %d | %s | %s |" % (s, MARK[s], n[s], pct(n[s], total), pct(t[s], ttotal)))
    w("")
    w("| Files (C++ units) | Count |")
    w("|-|-|")
    w("| all | %d |" % len(files))
    w("| no Rust file of that name | %d |" % fstat["not started"])
    w("| partial | %d |" % fstat["partial"])
    w("| every named function ported | %d |" % fstat["ported"])
    w("| every named function verified | %d |" % fstat["verified"])
    w("")
    w("C++ still run through the FFI shim crates: %d functions called from the engine (%s). They are listed at" % (
        shim_fns, ", ".join("`%s` %d" % (k, len(v)) for k, v in shims.items())))
    w("the end; the goal is none.\n")
    w("## How to read it\n")
    w("- A file here is a C++ unit: a `.cpp` with the header of the same name, or a header on its own.")
    w("- `[ ]` **not started**: no Rust function of that name anywhere, or the unit has no Rust file.")
    w("- `[~]` **partial**: the name exists in Rust but not in the file named after the C++ file, or only another")
    w("  overload is ported, or the body is estimated *partial*, *divergent* or a *stub*.")
    w("- `[x]` **ported**: in the paired Rust file, body estimated *faithful*. The estimate is mechanical (shared")
    w("  identifiers and constants, relative size, no deviation marker); it says where to look, it is not a proof.")
    w("  A function the estimate calls partial or divergent counts as ported when a round has read it side by")
    w("  side with the C++ and found the same logic; those are listed in `scripts/parity/port_checklist_reviewed.json`")
    w("  and say so in their note.")
    w("- `[v]` **verified**: ported, and a twin probe agreed with the C++ over a fixture or a test compares with")
    w("  C++ output; the evidence is in the note. G-code that matches on three fixtures does not by itself verify")
    w("  a function, so this column is short on purpose.")
    w("- Constructors, destructors and operators have no name to match by; they are counted per file and are not")
    w("  in the percentages. Classes are the scopes the functions sit in; a type with no function body is not listed.")
    w("- The percentage counts every named function with a body, accessors included, in all four tiers (tier 4 is")
    w("  SLA, which `slicer_cli` does not use).\n")
    if stale:
        w("Entries of `port_checklist_verified.json` that match no function now: " + ", ".join(
            "`%s` in `%s`" % (fn, u) for u, fn in stale) + ".\n")

    w("## Files\n")
    w("| C++ unit | Tier | Rust file | Status | Functions | not started | partial | ported | verified |")
    w("|-|-|-|-|-|-|-|-|-|")
    for f in files:
        fn, _ = tally(f["rows"])
        w("| `%s` | %d | %s | %s | %d | %d | %d | %d | %d |" % (
            f["cpp"], f["tier"], ("`%s`" % f["rust"]) if f["rust"] else "none", file_status(f),
            sum(fn.values()), fn["not started"], fn["partial"], fn["ported"], fn["verified"]))
    w("")

    w("## Functions, file by file\n")
    for f in files:
        fn, _ = tally(f["rows"])
        w("### `%s` (%s)\n" % (f["cpp"], f["ext"]))
        head = "Rust file: %s. Status: **%s**." % (("`%s`" % f["rust"]) if f["rust"] else "none", file_status(f))
        if f["name_match"] and f["name_match"] != "words":
            head += " The names agree by letters only (%s)." % f["name_match"]
        sp = f["special"]
        if sp:
            head += " Not listed: " + ", ".join("%d %s" % (sp[k], {"ctor": "constructors", "dtor": "destructors",
                                                                   "operator": "operators"}[k])
                                                for k in ("ctor", "dtor", "operator") if sp.get(k)) + "."
        w(head + "\n")
        if not f["rows"]:
            w("No named function with a body.\n")
            continue
        by_cls = OrderedDict()
        for r in sorted(f["rows"], key=lambda r: (r[0].rsplit("::", 1)[0] if "::" in r[0] else "", r[1])):
            cls = r[0].rsplit("::", 1)[0] if "::" in r[0] else ""
            by_cls.setdefault(cls, []).append(r)
        for cls, rows in by_cls.items():
            w("**%s**\n" % (("class `%s`" % cls) if cls else "free functions"))
            for name, line, _tokens, rust, rline, st, note in rows:
                short = name.rsplit("::", 1)[-1]
                where = ""
                if rust:
                    where = " → `%s%s`" % (rust, (":%d" % rline) if rline else "")
                w("- %s `%s` (line %d)%s%s" % (MARK[st], short, line, where, (" — " + note) if note else ""))
            w("")

    w("## C++ behind the FFI shims\n")
    w("Functions of the shim crates called from `crates/libslic3r-rs/src`, with the number of call sites. What each")
    w("stands for is in [`../rust-engine/parity/not-ported.md`](../rust-engine/parity/not-ported.md).\n")
    w("| Shim crate | Function | Call sites |")
    w("|-|-|-|")
    for crate, fns in shims.items():
        if not fns:
            w("| `%s` | none called by the engine | 0 |" % crate)
        for name, c in fns.items():
            w("| `%s` | `%s` | %d |" % (crate, name, c))
    w("")
    return "\n".join(o)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="fail if the committed checklist differs from a fresh one")
    args = ap.parse_args()
    files, stale, shims = build()
    md = render(files, stale, shims)
    if args.check:
        ok = os.path.exists(OUT_MD) and open(OUT_MD).read() == md
        print("port checklist is up to date" if ok else "port checklist is stale: run scripts/parity/port_checklist.py")
        sys.exit(0 if ok else 1)
    os.makedirs(os.path.dirname(OUT_MD), exist_ok=True)
    with open(OUT_MD, "w") as f:
        f.write(md)
    print(md.split("## How to read it")[0])


if __name__ == "__main__":
    main()
