#!/usr/bin/env python3
"""flatten_preset.py <profiles_root> <type: machine|process|filament> <name> <out.json>
Resolve a vendor preset's `inherits` chain (searching every vendor folder under profiles_root,
e.g. Snapmaker/ and OrcaFilamentLibrary/) into one self-contained JSON, for --load-settings /
--load-filaments. Child keys override parent keys; bookkeeping keys are taken from the leaf."""
import glob, json, os, sys
root, typ, name, out = sys.argv[1:5]
index = {}
for p in glob.glob(os.path.join(root, "*", typ, "**", "*.json"), recursive=True):
    try: d = json.load(open(p, encoding="utf-8-sig"))
    except Exception: continue
    if "name" in d: index.setdefault(d["name"], d)
if name not in index: sys.exit(f"preset not found: {typ} {name!r}")
chain, n = [], name
while n:
    d = index.get(n)
    if d is None: sys.exit(f"inherits target not found: {n!r} (chain {[c['name'] for c in chain]})")
    chain.append(d); n = d.get("inherits", "")
merged = {}
for d in reversed(chain):
    merged.update(d)
# Snapmaker Orca 2.4.1's CLI calls normalize_fdm() per settings file, and its normalize_fdm reads
# nozzle_diameter unchecked whenever wipe_tower_filament is present (fork PrintConfig.cpp ~7872), so a
# process file carrying it segfaults. "0" is the default, so dropping it changes no setting (D-006).
if typ == "process" and str(merged.get("wipe_tower_filament", "0")) == "0":
    merged.pop("wipe_tower_filament", None)
merged["name"] = name; merged["inherits"] = ""; merged["from"] = "system"; merged["instantiation"] = "true"
json.dump(merged, open(out, "w"), indent=1)
print(f"{name}: {len(chain)} levels, {len(merged)} keys -> {out}")
