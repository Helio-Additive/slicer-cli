#!/usr/bin/env python3
"""resolve_preset.py VENDOR_DIR PRINTER OUT_PREFIX [process=NAME] [FILAMENT...]

Flattens a vendor's system presets (walks `inherits`, as the official CLIs need: they read the
JSON files as given) into OUT_PREFIX-machine.json, -process.json and -filament.json (and
-filament2.json ... for more filaments). The process and filament are the printer's defaults
(default_print_profile / default_filament_profile) unless process=NAME / FILAMENT names are given.
VENDOR_DIR: <package>/resources/profiles/BBL (Bambu build), profiles-orca/<Vendor> (Orca build),
or an official app's resources/profiles/<Vendor>.
"""
import json, os, sys


def load(vendor, kind):
    out = {}
    for root, _, files in os.walk(os.path.join(vendor, kind)):
        for f in files:
            if f.endswith(".json"):
                try:
                    d = json.load(open(os.path.join(root, f), encoding="utf-8"))
                except Exception:
                    continue
                out[d.get("name", f[:-5])] = d
    return out


def main():
    vendor, printer, prefix = sys.argv[1:4]
    rest = sys.argv[4:]
    proc_arg = [a[len("process="):] for a in rest if a.startswith("process=")]
    fils = [a for a in rest if not a.startswith("process=")]
    P = {k: load(vendor, k) for k in ("machine", "process", "filament")}

    def resolve(kind, name, seen=()):
        if name not in P[kind] or name in seen:
            raise SystemExit("no %s preset %r in %s" % (kind, name, vendor))
        d = P[kind][name]
        r = resolve(kind, d["inherits"], seen + (name,)) if d.get("inherits") else {}
        r.update(d)
        r.pop("inherits", None)
        return r

    m = resolve("machine", printer)
    first = lambda v: v[0] if isinstance(v, list) else v
    proc = proc_arg[0] if proc_arg else first(m.get("default_print_profile"))
    if not fils:
        fils = [first(m.get("default_filament_profile"))]
    out = [("machine", printer, m), ("process", proc, resolve("process", proc))]
    for i, f in enumerate(fils):
        out.append(("filament" if i == 0 else "filament%d" % (i + 1), f, resolve("filament", f)))
    for tag, name, d in out:
        d = dict(d)
        d.update({"type": "filament" if tag.startswith("filament") else tag, "from": "system",
                  "name": name, "instantiation": "true"})
        json.dump(d, open("%s-%s.json" % (prefix, tag), "w", encoding="utf-8"), indent=1)
    print(json.dumps({"printer": printer, "process": proc, "filaments": fils}))


if __name__ == "__main__":
    main()
